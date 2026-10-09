//! Nodal state struct, `Default`, `set_sample_rate`, `reset` and setters.

use super::reset::emit_dc_block_history_reseed;
use super::sites::{
    counts_unconverged_commit, emits_hold, has_be_instance, live_g_c, schur_exact_seed,
};
use crate::codegen::ir::{CircuitIR, DeviceParams};
use crate::codegen::rust_emitter::dk_emitter::{emit_warmup_call, NoiseEmission};
use crate::codegen::rust_emitter::helpers::{
    body_effect_mosfets, carries_q_dot, device_param_template_data, emit_stateful_default_fields,
    emit_stateful_set_sample_rate_body, emit_stateful_state_fields, emit_stateful_state_restore,
    fmt_f64, history_zero_row_ranges, oversampling_info, q_dot_start, section_banner,
    self_heating_device_data, stateful_device_data,
};
use crate::codegen::rust_emitter::inject_tap::{
    emit_inject_os_state_fields, emit_inject_os_state_init, emit_inject_os_state_reset,
};
use crate::codegen::rust_emitter::RustEmitter;

impl RustEmitter {
    /// Emit state struct, Default impl, set_sample_rate, and reset for nodal solver.
    /// `setter_stamps` collects every literal `(row, col)` that an emitted
    /// `.pot` / `.switch` / `.runtime R` / `.wiper` setter writes into
    /// `a` / `a_neg` / `a_be` / `a_neg_be` / `g_work` / `c_work`. Recording at
    /// the emission site (rather than re-deriving the geometry from the IR)
    /// keeps the equilibration pattern from drifting if a stamp site is later
    /// added or changed — see `EquilPattern`.
    pub(super) fn emit_nodal_state(
        &self,
        ir: &CircuitIR,
        use_full_nodal: bool,
        noise: &NoiseEmission,
        setter_stamps: &mut std::collections::BTreeSet<(usize, usize)>,
    ) -> String {
        let m = ir.topology.m;
        // Multi-input ports (M=0 only): the input-history state field becomes a
        // per-port array. See multi-input-ports-plan.md.
        let multi_input = ir.solver_config.num_inputs() > 1;
        let has_pots = !ir.pots.is_empty();
        let has_switches = !ir.switches.is_empty();
        let has_sat_ind = !ir.saturating_inductors.is_empty();
        // The op-amp slew block reads `state.current_sample_rate`, so that field
        // must exist whenever any op-amp has a finite SR — independent of
        // pots/switches. Behavioral B-source circuits (forced nodal) with a
        // slew-limited op-amp and no pots/switches exposed this gap.
        // The adaptive sub-step ladder also reads it (its `alpha_sub` must
        // track the runtime sample rate, not the baked codegen rate), so the
        // field is emitted for every build with a Newton solve on either
        // nodal route.
        let has_opamp_slew = ir.opamps.iter().any(|oa| oa.sr.is_finite());
        let has_substep_ladder = emits_hold(ir);
        // Device self-heating reads `state.current_sample_rate` for its
        // thermal dt (emit_self_heating_thermal_updates, DK parity) — the
        // field must exist even for a Schur-routed thermal circuit with no
        // pots/switches/slew.
        let has_thermal_sr_consumer = ir.device_slots.iter().any(|s| match &s.params {
            DeviceParams::Bjt(bp) => bp.has_self_heating(),
            DeviceParams::Diode(dp) => dp.has_self_heating(),
            DeviceParams::Tube(tp) => tp.has_self_heating(),
            _ => false,
        });
        let needs_rebuild_state = has_pots || has_switches || has_sat_ind;
        // Stateful devices (Phase 0c) read `state.current_sample_rate` for the
        // after-solve update() dt (DK parity), so the field must exist.
        let has_stateful = !stateful_device_data(ir).is_empty();
        // The runtime BE-latch detector derives its EMA coefficient from the
        // live sample rate (fs-invariant time constant), so it needs the field.
        let needs_current_sr = needs_rebuild_state
            || has_opamp_slew
            || has_substep_ladder
            || has_thermal_sr_consumer
            || has_stateful
            || ir.solver_config.runtime_be_latch
            // `set_oversampling` rebuilds at the current host rate.
            || super::super::runtime_os::runtime(ir).is_some();

        let mut code = section_banner("STATE STRUCTURE (Nodal solver)");

        // DC OP constant
        let has_dc_op = ir.has_dc_op;
        if has_dc_op {
            let dc_op_values = ir
                .dc_operating_point
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// DC operating point: steady-state node voltages\npub const DC_OP: [f64; N] = [{}];\n\n",
                dc_op_values
            ));
        }

        // IC=-bearing capacitors: initial-state seed for `v_prev`. Independent
        // of `has_dc_op` — a circuit with no DC sources but an IC= cap still
        // needs this constant. See `docs/aidocs/DC_OP.md` "IC= initial condition".
        let has_cap_ic = ir.v_prev_ic_seed.is_some();
        if let Some(v_prev_ic) = &ir.v_prev_ic_seed {
            let v_prev_ic_values = v_prev_ic
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// Initial-state seed for `v_prev`: node voltages after solving the DC\n\
                 /// system with each `IC=`-bearing capacitor temporarily replaced by an\n\
                 /// ideal DC voltage source of that value (SPICE `.IC`/UIC semantics).\n\
                 /// Affects only capacitors carrying an explicit `IC=`; used solely to seed\n\
                 /// the transient's `v_prev`. `DC_OP` above (when present) remains the pure\n\
                 /// IC-free DC bias point.\n\
                 pub const V_PREV_IC_SEED: [f64; N] = [{}];\n\n",
                v_prev_ic_values
            ));
        }
        if let Some(q) = &ir.q_dot_ic_seed {
            let values = q.iter().map(|v| fmt_f64(*v)).collect::<Vec<_>>().join(", ");
            code.push_str(&format!(
                "/// Charge derivative `C·dx/dt` at the IC-seeded start, paired with\n\
                 /// `V_PREV_IC_SEED`: each `IC=` capacitor's current at t = 0.\n\
                 pub const Q_DOT_IC_SEED: [f64; N] = [{values}];\n\n"
            ));
        }

        // DC NL currents
        let has_dc_nl = m > 0
            && !ir.dc_nl_currents.is_empty()
            && ir.dc_nl_currents.iter().any(|&v| v.abs() > 1e-30);
        if has_dc_nl {
            let dc_nl_i_values = ir
                .dc_nl_currents
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// DC operating point: nonlinear device currents at bias point\npub const DC_NL_I: [f64; M] = [{}];\n\n",
                dc_nl_i_values
            ));
        }

        // DC NL currents at the IC-seeded operating point — paired ONLY
        // with V_PREV_IC_SEED (never with the plain DC_OP/DC_NL_I pair used
        // by the reset fallback). See the pairing comment on
        // `dc_nl_currents_ic_seed` in `codegen/ir/mod.rs`.
        let has_dc_nl_ic_seed = m > 0 && ir.dc_nl_currents_ic_seed.is_some();
        if has_dc_nl_ic_seed {
            let dc_nl_i_ic_seed_values = ir
                .dc_nl_currents_ic_seed
                .as_ref()
                .unwrap()
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// Nonlinear device currents at the same IC-seeded operating point as\n\
                 /// `V_PREV_IC_SEED`. MUST be used together with `V_PREV_IC_SEED` to seed\n\
                 /// `i_nl_prev`/`i_nl_prev_prev` — never with the plain `DC_OP`/`DC_NL_I`\n\
                 /// pair, which the per-sample magnitude/NaN-reset fallback resets state to.\n\
                 pub const DC_NL_I_IC_SEED: [f64; M] = [{}];\n\n",
                dc_nl_i_ic_seed_values
            ));
        }

        // State struct
        code.push_str("/// Circuit state for one processing channel (nodal solver).\n");
        code.push_str("///\n");
        code.push_str("/// Contains per-sample state and sample-rate-dependent matrices.\n");
        code.push_str(
            "/// Call [`set_sample_rate`](CircuitState::set_sample_rate) before processing\n",
        );
        code.push_str("/// if the host sample rate differs from [`SAMPLE_RATE`].\n");
        // When using full-LU nodal path, emit a separate cold struct for
        // Schur complement + work matrices. These are only accessed on
        // pot/switch/sample-rate changes, not per sample. Keeping them
        // heap-allocated via Box prevents them from polluting L2 cache.
        if use_full_nodal {
            code.push_str(
                "/// Cold state: matrices only accessed on pot/switch/sample-rate changes.\n",
            );
            code.push_str("/// Heap-allocated to keep the per-sample hot path in L2 cache.\n");
            code.push_str("#[derive(Clone, Debug)]\n");
            code.push_str("pub struct CircuitStateCold {\n");
            code.push_str("    pub s: [[f64; N]; N],\n");
            if m > 0 {
                code.push_str("    pub k: [[f64; M]; M],\n");
                code.push_str("    pub s_ni: [[f64; M]; N],\n");
            }
            code.push_str("    pub s_be: [[f64; N]; N],\n");
            if m > 0 {
                code.push_str("    pub k_be: [[f64; M]; M],\n");
                code.push_str("    pub s_ni_be: [[f64; M]; N],\n");
            }
            if has_pots || has_switches || has_sat_ind {
                code.push_str("    pub g_work: [[f64; N]; N],\n");
                code.push_str("    pub c_work: [[f64; N]; N],\n");
            }
            code.push_str("}\n\n");
        }

        code.push_str("#[derive(Clone, Debug)]\n");
        code.push_str("pub struct CircuitState {\n");
        code.push_str("    /// Previous node voltages v[n-1]\n");
        code.push_str("    pub v_prev: [f64; N],\n\n");
        if carries_q_dot(ir) {
            code.push_str(
                "    /// Charge derivative `C·dx/dt` at v[n-1] (capacitor currents; `dΦ/dt` on\n\
                 \x20   /// inductor branch rows). The trapezoidal history is `alpha·C·v[n-1]`\n\
                 \x20   /// plus this, so KCL holds exactly at every committed sample.\n\
                 \x20   pub q_dot: [f64; N],\n\n",
            );
        }
        code.push_str("    /// Previous nonlinear currents i_nl[n-1]\n");
        code.push_str("    pub i_nl_prev: [f64; M],\n\n");
        code.push_str(
            "    /// Nonlinear currents from two samples ago i_nl[n-2] (for NR predictor)\n",
        );
        code.push_str("    pub i_nl_prev_prev: [f64; M],\n\n");
        code.push_str("    /// DC operating point (for reset/sleep/wake)\n");
        code.push_str("    pub dc_operating_point: [f64; N],\n\n");
        code.push_str("    /// Previous input sample for trapezoidal integration\n");
        if multi_input {
            code.push_str("    pub inputs_prev: [f64; NUM_INPUTS],\n\n");
        } else {
            code.push_str("    pub input_prev: f64,\n\n");
        }
        if ir.solver_config.has_inject_or_tap() {
            code.push_str(
                "    /// Previous `.inject` values (per inner sample), for the trapezoidal\n\
                 \x20   /// history term of Thevenin injections. Advanced once per inner call\n\
                 \x20   /// alongside `input_prev`.\n\
                 \x20   pub injections_prev: [f64; NUM_INJECT],\n\n",
            );
        }
        code.push_str("    /// NR convergence diagnostic from the last sample's solve.\n");
        code.push_str("    ///\n");
        code.push_str("    /// Semantics:\n");
        code.push_str(
            "    /// - 0..MAX_ITER-1: 0-indexed iteration at which convergence was detected\n",
        );
        code.push_str("    ///   (i.e. `last_nr_iterations + 1` iterations actually ran)\n");
        code.push_str("    /// - MAX_ITER: loop exhausted without convergence → NR failed\n");
        code.push_str("    ///\n");
        code.push_str(
            "    /// The check `last_nr_iterations >= MAX_ITER` is the BE-fallback trigger;\n",
        );
        code.push_str(
            "    /// storing `iter` (not `iter + 1`) is intentional so that convergence at\n",
        );
        code.push_str("    /// the final permitted iteration (iter == MAX_ITER - 1) does not false-trigger.\n");
        code.push_str("    pub last_nr_iterations: u32,\n\n");
        // Behavioral B-source ddt/idt companion state.
        if ir.behavioral_sources.iter().any(|b| b.time_dependent) {
            code.push_str("    /// Simulation time (s), advanced one dt per inner sample\n");
            code.push_str("    pub sim_time: f64,\n");
            code.push_str("    /// 1/dt for behavioral ddt (set by set_sample_rate)\n");
            code.push_str("    pub bsrc_inv_dt: f64,\n");
            code.push_str("    /// dt/2 for behavioral idt (set by set_sample_rate)\n");
            code.push_str("    pub bsrc_half_dt: f64,\n");
            code.push_str("    /// Previous inner-expression value per ddt/idt slot\n");
            code.push_str("    pub bsrc_x_prev: [f64; N_BSRC_SLOTS],\n");
            code.push_str("    /// Running integral per idt slot\n");
            code.push_str("    pub bsrc_int_prev: [f64; N_BSRC_SLOTS],\n\n");
        }
        // Plugin-driven scalar params (.runtime <name> min max as field).
        for r in &ir.behavioral_scalar_runtimes {
            code.push_str(&format!(
                "    /// Plugin scalar `{}` (range [{}, {}]); set via set_runtime_{}\n",
                r.name, r.min, r.max, r.field_name
            ));
            code.push_str(&format!("    pub {}: f64,\n", r.field_name));
        }
        if !ir.behavioral_scalar_runtimes.is_empty() {
            code.push('\n');
        }
        if ir.dc_block {
            code.push_str("    /// DC blocking filter: previous input samples (one per output)\n");
            code.push_str("    pub dc_block_x_prev: [f64; NUM_OUTPUTS],\n");
            code.push_str("    /// DC blocking filter: previous output samples (one per output)\n");
            code.push_str("    pub dc_block_y_prev: [f64; NUM_OUTPUTS],\n");
            code.push_str(
                "    /// DC blocking filter coefficient (recomputed on sample rate change)\n",
            );
            code.push_str("    pub dc_block_r: f64,\n\n");
        }
        code.push_str("    /// Diagnostic: peak absolute output (pre-clamp)\n");
        code.push_str("    pub diag_peak_output: f64,\n");
        code.push_str(
            "    /// Diagnostic: number of times output exceeded the ±output_clamp_v ceiling\n",
        );
        code.push_str("    pub diag_clamp_count: u64,\n");
        code.push_str("    /// Diagnostic: number of times NR hit max iterations\n");
        code.push_str("    pub diag_nr_max_iter_count: u64,\n");
        code.push_str(
            "    /// Diagnostic: device-samples where a pentode's grid conducts (Vgk > 0)\n\
             \x20   /// or a BJT's base-collector junction is forward biased (saturation) —\n\
             \x20   /// the regions the grid-off / forward-active reductions assume are never\n\
             \x20   /// entered. Counted on the full models too (characterization, not a guard).\n",
        );
        code.push_str("    pub diag_region_exit_count: u64,\n");
        if super::super::helpers::has_reduced_device(ir) {
            code.push_str(
                "    /// Diagnostic: samples on which a REDUCED device (a forward-active BJT, a\n\
                 \x20   /// grid-off pentode) left the region its reduction assumes. Its model\n\
                 \x20   /// no longer describes the device there, so each is also counted in\n\
                 \x20   /// `diag_unsolved_sample_count` and refused by every verb.\n",
            );
            code.push_str("    pub diag_reduced_model_exit_count: u64,\n");
        }
        code.push_str("    /// Diagnostic: number of backward Euler fallback activations\n");
        code.push_str("    pub diag_be_fallback_count: u64,\n");
        code.push_str(
            "    /// Diagnostic: samples that were NEVER SOLVED, whatever the route and\n\
             \x20   /// the failure mechanism (a death-spiral hold, or an unconverged\n\
             \x20   /// iterate committed). Present on every generated build (always 0\n\
             \x20   /// where no Newton solve exists). A nonzero value means this render\n\
             \x20   /// contains samples that are not solutions; `melange validate`,\n\
             \x20   /// `simulate` and the golden harness refuse on it. The\n\
             \x20   /// mechanism-specific counters below are detail.\n",
        );
        code.push_str("    pub diag_unsolved_sample_count: u64,\n");
        // Declared ONLY where the mechanism exists: a build with no Newton
        // solve that can end unsolved would report a permanent, reassuring zero
        // for something that cannot happen, which is the same misleading-
        // diagnostic shape this counter was added to remove.
        let hold = emits_hold(ir);
        if hold {
            code.push_str(
                "    /// Diagnostic: samples on which EVERY Newton path failed (trap +\n\
             \x20   /// sub-step + BE) and the death-spiral hold committed the PREVIOUS\n\
             \x20   /// state as this sample's answer.\n\
             \x20   ///\n\
             \x20   /// **A nonzero value means this render contains samples that are not\n\
             \x20   /// solutions.** The hold emits a bounded, smooth value, so peak, RMS,\n\
             \x20   /// clamp count and correlation all read healthy — no level-based check\n\
             \x20   /// can see it. This counter is the only witness.\n\
             \x20   ///\n\
             \x20   /// Worse, the hold is a FIXED POINT under constant input: the next\n\
             \x20   /// sample re-poses the bit-identical problem from the same `v_prev` and\n\
             \x20   /// fails identically, so one hard sample can freeze the circuit until\n\
             \x20   /// the input changes. Measured on a transformer-coupled preamp input\n\
             \x20   /// block: 43199\n\
             \x20   /// consecutive held samples, output 22 dB adrift, peak a healthy\n\
             \x20   /// -0.50 dBFS (design review).\n\
             \x20   ///\n\
             \x20   /// Distinct from `diag_be_fallback_count`, which counts a RECOVERY:\n\
             \x20   /// a converged solution by another consistent scheme. This counts a\n\
             \x20   /// non-solution shipped as output.\n",
            );
            code.push_str("    pub diag_nr_hold_count: u64,\n");
        }
        // A pinned solve that does not converge is committed, not held, where
        // no Newton solve takes its outcome (see pin_failure_is_committed).
        if counts_unconverged_commit(ir, use_full_nodal) {
            code.push_str(
                "    /// Diagnostic: samples whose op-amp rail pin (the active-set pinned\n\
                 \x20   /// Newton) did not converge, or whose pinned system was singular,\n\
                 \x20   /// and whose iterate was committed anyway. Counted at the failure,\n\
                 \x20   /// once per internal (oversampled) sample. Nonzero means this render\n\
                 \x20   /// contains samples that were never solved (design review).\n",
            );
            code.push_str("    pub diag_nr_unconverged_commit_count: u64,\n");
        }
        code.push_str(
            "    /// Diagnostic: number of times the runtime BE-latch engaged (rising\n\
             \x20   /// edges). Nonzero means the solver detected a self-sustaining Nyquist\n\
             \x20   /// limit cycle on this stream and permanently switched that instance to\n\
             \x20   /// the L-stable backward-Euler path (cleared by reset()). A plugin can\n\
             \x20   /// surface this as \"solver degraded\". Always 0 on backward-Euler and\n\
             \x20   /// force-trap builds.\n",
        );
        code.push_str("    pub diag_be_latch_count: u64,\n");
        code.push_str("    /// Diagnostic: number of samples where the active-set rail resolve\n");
        code.push_str("    /// pinned at least one op-amp output (ActiveSet/ActiveSetBe only)\n");
        code.push_str("    pub diag_active_set_pin_count: u64,\n");
        code.push_str("    /// Diagnostic: number of times NaN triggered state reset\n");
        code.push_str("    pub diag_nan_reset_count: u64,\n");
        code.push_str(
            "    /// Diagnostic: input samples clamped to +/-INPUT_LIMIT_V (the circuit\n\
             \x20   /// was driven with a smaller input than requested)\n\
             \x20   pub diag_input_clamp_count: u64,\n\
             \x20   /// Diagnostic: NaN/Inf host input samples replaced by 0\n\
             \x20   pub diag_input_nan_count: u64,\n",
        );
        code.push_str(&super::super::runtime_inputs::counter_field("    "));
        code.push_str(
            "    /// Diagnostic: number of times a finite-but-implausible iterate\n\
             \x20   /// (state magnitude beyond any physically-realizable circuit value)\n\
             \x20   /// triggered state reset. NaN/Inf is caught by diag_nan_reset_count;\n\
             \x20   /// this counts the \"diverged but still is_finite()\" gap.\n",
        );
        code.push_str("    pub diag_magnitude_reset_count: u64,\n");
        code.push_str("    /// Diagnostic: number of samples that needed adaptive sub-stepping\n");
        code.push_str("    pub diag_substep_count: u64,\n");
        if schur_exact_seed(ir, use_full_nodal) {
            code.push_str(
                "    /// Diagnostic: Newton solves whose warm start fell back to the\n\
                 \x20   /// first-order current predictor because full-LU's start was not\n\
                 \x20   /// reachable through the device currents (K·i_nl = N_v·v_prev − p\n\
                 \x20   /// inconsistent; a singular but consistent K is solved exactly).\n\
                 \x20   /// The exact start (full-LU's `v_prev`) keeps a regenerative circuit\n\
                 \x20   /// on its branch; the predictor can land on another root with no\n\
                 \x20   /// other counter moving, so a nonzero value flags samples solved\n\
                 \x20   /// without that guarantee.\n",
            );
            code.push_str("    pub diag_warm_start_fallback_count: u64,\n");
        }
        code.push_str(&super::super::subsample_fire::emit_subsample_fire_state_fields(ir));
        code.push_str("    /// Diagnostic: number of LU refactorizations performed\n");
        code.push_str("    pub diag_refactor_count: u64,\n");
        code.push_str(
            "    /// Diagnostic: number of NR iterations whose Armijo line search was\n\
             \x20   /// searched and failed (no `s` down to the 2^-10 floor met the\n\
             \x20   /// sufficient-decrease test on a finite r0 > 1e-9). On such an\n\
             \x20   /// iteration the loop takes the un-line-searched pnjlim/damp-limited\n\
             \x20   /// step and continues; the always-checked residual gate still decides\n\
             \x20   /// convergence. This is an expected event on stiff high-gain circuits,\n\
             \x20   /// not an error — it is surfaced (not silenced) so the fall-through is\n\
             \x20   /// observable. A non-finite or <=1e-9 r0 skips the search entirely and\n\
             \x20   /// does NOT count here (ls_ok stays true).\n",
        );
        code.push_str("    pub diag_ls_fail_count: u64,\n");
        code.push_str("    /// Diagnostic: number of samples hit by the global voltage-damping\n");
        code.push_str("    /// safety net. This is a legacy safeguard that scales v_new toward\n");
        code.push_str("    /// v_prev when any node moves more than ~2V (or 5% of max DC OP) in\n");
        code.push_str(
            "    /// one sample. Per CLAUDE.md, output limiting must never mask solver\n",
        );
        code.push_str(
            "    /// bugs — treat a nonzero count here as a signal that the solver needs\n",
        );
        code.push_str("    /// investigation, not as an acceptable steady-state.\n");
        code.push_str("    pub diag_voltage_damp_count: u64,\n\n");

        // Runtime BE-latch detector working state (trapezoidal builds only).
        if ir.solver_config.runtime_be_latch {
            code.push_str(
                "    /// Runtime BE-latch estimator on the primary output: its running mean,\n\
                 \x20   /// the previous mean-removed sample, and EMAs of x*x_prev and x*x.\n",
            );
            code.push_str("    pub be_x_mean: f64,\n");
            code.push_str("    pub be_x_prev: f64,\n");
            code.push_str("    pub be_r1_num: f64,\n");
            code.push_str("    pub be_pow: f64,\n");
            code.push_str(
                "    /// Runtime BE-latch estimator on the input (the input-awareness gate).\n",
            );
            code.push_str("    pub be_in_x_mean: f64,\n");
            code.push_str("    pub be_in_x_prev: f64,\n");
            code.push_str("    pub be_in_r1_num: f64,\n");
            code.push_str("    pub be_in_pow: f64,\n");
            code.push_str(
                "    /// Runtime BE-latch program reference: the smaller of the passband gain x\n\
                 \x20   /// input amplitude (be_ref_in) and the output's own excursion from its\n\
                 \x20   /// operating point (be_env), each held decaying at be_ref_decay per\n\
                 \x20   /// sample; and that decay at the running rate (set by set_sample_rate).\n",
            );
            code.push_str("    pub be_ref_in: f64,\n");
            code.push_str("    pub be_env: f64,\n");
            code.push_str("    pub be_ref: f64,\n");
            code.push_str("    pub be_ref_decay: f64,\n");
            code.push_str(
                "    /// Runtime BE-latch: true once a stiff alternating mode was detected;\n\
                 \x20   /// forces the L-stable BE path for the rest of the stream (cleared by\n\
                 \x20   /// reset()).\n",
            );
            code.push_str("    pub be_latched: bool,\n\n");
        }

        // Breakpoint-BE countdown (trapezoidal builds with a capacitor/inductor
        // .switch, or a glow device). Armed by such a set_switch_* (held while a
        // glow is lit) to a small const; while > 0 the sample is
        // solved with the L-stable BE matrices (re-seeds q_dot on the new
        // component values, damps the excited mode), then decremented. Zero at
        // rest → byte-inert.
        if ir.solver_config.breakpoint_be {
            code.push_str(
                "    /// Breakpoint-BE: samples remaining to solve on the backward-Euler\n\
                 \x20   /// matrices after a .switch swaps a capacitor or an inductor (armed by\n\
                 \x20   /// set_switch_*) or while a glow device is lit, decremented per sample,\n\
                 \x20   /// cleared by\n\
                 \x20   /// reset().\n",
            );
            code.push_str("    pub breakpoint_be: u32,\n\n");
        }

        // DC settling state: when DC OP didn't converge at codegen time, the warmup
        // fast-forwards at low sample rate to charge coupling caps. The settled state
        // is cached for subsequent resets to avoid repeating the expensive warmup.
        if !ir.dc_op_converged && m > 0 {
            code.push_str("    /// Whether low-rate DC warmup has been completed\n");
            code.push_str("    pub dc_settled: bool,\n");
            code.push_str(
                "    /// Settled nonlinear currents from low-rate warmup (replaces DC_NL_I on reset)\n",
            );
            code.push_str("    pub settled_i_nl: [f64; M],\n\n");
        }

        // Cross-timestep chord state (persisted LU for full LU path). Behavioral
        // B-sources force the full-LU path even at M=0, so they need it too.
        if m > 0 || !ir.behavioral_sources.is_empty() || !ir.saturating_inductors.is_empty() {
            code.push_str(
                "    // --- Cross-timestep chord method state (persisted LU factors) ---\n",
            );
            code.push_str(
                "    /// LU-factored Jacobian from previous convergence (chord method)\n",
            );
            code.push_str("    pub chord_lu: [[f64; N]; N],\n");
            code.push_str("    /// Row equilibration scaling from LU factorization\n");
            code.push_str("    pub chord_dr: [f64; N],\n");
            code.push_str("    /// Column equilibration scaling from LU factorization\n");
            code.push_str("    pub chord_dc: [f64; N],\n");
            code.push_str("    /// Row permutation from LU factorization\n");
            code.push_str("    pub chord_perm: [usize; N],\n");
            code.push_str("    /// Device Jacobian consistent with chord_lu (for companion RHS)\n");
            code.push_str("    pub chord_j_dev: [f64; M * M],\n");
            let nb = body_effect_mosfets(ir).len();
            if nb > 0 {
                code.push_str(&format!(
                    "    /// MOSFET body-effect transconductance frozen with the chord.\n    pub chord_body_gmb: [f64; {nb}],\n"
                ));
            }
            code.push_str(
                "    /// Whether chord LU factors are valid (false until first convergence)\n",
            );
            code.push_str("    pub chord_valid: bool,\n");
            code.push_str(
                "    /// Whether chord_lu holds DENSE (partial-pivoting) factors from the\n",
            );
            code.push_str(
                "    /// runtime fallback (sparse static pivoting rejected: tiny pivot or\n",
            );
            code.push_str("    /// excessive element growth). Selects the matching back-solve.\n");
            code.push_str("    pub chord_dense: bool,\n\n");
            if use_full_nodal && has_be_instance(ir) {
                code.push_str(
                    "    // --- Chord cache of the backward-Euler solve (latch, fallback,\n    // breakpoint): kept apart so neither integrator clobbers the other ---\n",
                );
                code.push_str("    pub chord_be_lu: [[f64; N]; N],\n");
                code.push_str("    pub chord_be_dr: [f64; N],\n");
                code.push_str("    pub chord_be_dc: [f64; N],\n");
                code.push_str("    pub chord_be_perm: [usize; N],\n");
                code.push_str("    pub chord_be_j_dev: [f64; M * M],\n");
                let nb = body_effect_mosfets(ir).len();
                if nb > 0 {
                    code.push_str(&format!("    pub chord_be_body_gmb: [f64; {nb}],\n"));
                }
                code.push_str("    pub chord_be_valid: bool,\n");
                code.push_str("    pub chord_be_dense: bool,\n\n");
            }
        }

        code.push_str(
            "    /// A matrix: G + alpha*C (trapezoidal), recomputed by set_sample_rate\n",
        );
        code.push_str("    pub a: [[f64; N]; N],\n");
        code.push_str(
            "    /// A_neg matrix: alpha*C (charge-form history), recomputed by set_sample_rate\n",
        );
        code.push_str("    pub a_neg: [[f64; N]; N],\n");
        code.push_str(
            "    /// A_be matrix: G + (1/T)*C (backward Euler), recomputed by set_sample_rate\n",
        );
        code.push_str("    pub a_be: [[f64; N]; N],\n");
        code.push_str("    /// A_neg_be matrix: (1/T)*C (backward Euler history), recomputed by set_sample_rate\n");
        code.push_str("    pub a_neg_be: [[f64; N]; N],\n");

        // Schur complement matrices + work matrices.
        // When use_full_nodal, these are in a heap-allocated CircuitStateCold
        // to keep the hot per-sample working set in L2 cache.
        if use_full_nodal {
            code.push_str(
                "    /// Cold state: Schur complement + work matrices (heap-allocated,\n",
            );
            code.push_str(
                "    /// only accessed on pot/switch changes and sample rate changes).\n",
            );
            code.push_str("    pub cold: Box<CircuitStateCold>,\n");
        } else {
            // Schur path: all matrices inline (small N, no cache pressure)
            code.push_str(
                "    /// S matrix: A^{-1} (trapezoidal), recomputed by set_sample_rate\n",
            );
            code.push_str("    pub s: [[f64; N]; N],\n");
            if m > 0 {
                code.push_str("    /// K matrix: N_v * S * N_i (nonlinear kernel), recomputed by set_sample_rate\n");
                code.push_str("    pub k: [[f64; M]; M],\n");
                code.push_str(
                    "    /// S_NI matrix: S * N_i (voltage recovery), recomputed by set_sample_rate\n",
                );
                code.push_str("    pub s_ni: [[f64; M]; N],\n");
            }
            code.push_str(
                "    /// S_be matrix: A_be^{-1} (backward Euler), recomputed by set_sample_rate\n",
            );
            code.push_str("    pub s_be: [[f64; N]; N],\n");
            if m > 0 {
                code.push_str("    /// K_be matrix: N_v * S_be * N_i (BE kernel), recomputed by set_sample_rate\n");
                code.push_str("    pub k_be: [[f64; M]; M],\n");
                code.push_str("    /// S_NI_be matrix: S_be * N_i (BE voltage recovery), recomputed by set_sample_rate\n");
                code.push_str("    pub s_ni_be: [[f64; M]; N],\n");
            }
            if schur_exact_seed(ir, use_full_nodal) {
                for k in ["k", "k_be"] {
                    code.push_str(&format!(
                        "    /// Newton warm start: the `{k}` last factored, its rank-revealing LU\n\
                         \x20   /// factors, row and column orders, and rank (refactored when `{k}` changes).\n\
                         \x20   pub ws_{k}_key: [[f64; M]; M],\n\
                         \x20   pub ws_{k}_lu: [[f64; M]; M],\n\
                         \x20   pub ws_{k}_pr: [usize; M],\n\
                         \x20   pub ws_{k}_pc: [usize; M],\n\
                         \x20   pub ws_{k}_rank: usize,\n"
                    ));
                }
            }
        }
        code.push('\n');

        // Mutable G and C for pot/switch/saturating-inductor re-stamping (cold
        // in the full-nodal layout). `current_sample_rate` is also needed by the
        // op-amp slew block, so it's emitted whenever `needs_current_sr`.
        if needs_rebuild_state && !use_full_nodal {
            code.push_str("    /// Working G matrix (modified by pots/switches)\n");
            code.push_str("    pub g_work: [[f64; N]; N],\n");
            code.push_str("    /// Working C matrix (modified by switches/saturating inductors)\n");
            code.push_str("    pub c_work: [[f64; N]; N],\n");
        }
        if needs_current_sr {
            code.push_str(
                "    /// Current HOST sample rate (Hz), as passed to `set_sample_rate`.\n",
            );
            code.push_str(
                "    /// Multiply by `OVERSAMPLING_FACTOR` to get the internal rate at\n",
            );
            code.push_str("    /// consumption sites (rebuild_matrices, op-amp slew dt, substep\n");
            code.push_str("    /// alpha). Matches the DK path's field semantics (API parity).\n");
            code.push_str("    pub current_sample_rate: f64,\n");
        }
        code.push_str(super::super::runtime_os::state_field_decl(ir));
        if needs_rebuild_state {
            code.push_str(
                "    /// Lazy rebuild flag: set by set_pot/set_switch, cleared by process_sample\n",
            );
            code.push_str("    pub matrices_dirty: bool,\n");
        }
        if needs_current_sr || needs_rebuild_state {
            code.push('\n');
        }

        // Pot state fields
        for (idx, _pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "    /// Potentiometer {}: current resistance (ohms)\n\
                 \x20   pub pot_{}_resistance: f64,\n\
                 \x20   /// Potentiometer {}: resistance at the last committed sample (kept in\n\
                 \x20   /// step with the current resistance; not read by the solver)\n\
                 \x20   pub pot_{}_resistance_prev: f64,\n",
                idx, idx, idx, idx
            ));
        }
        if has_pots {
            code.push('\n');
        }

        // Switch state fields
        for (idx, _sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!(
                "    /// Switch {}: current position (0-indexed)\n\
                 \x20   pub switch_{}_position: usize,\n",
                idx, idx
            ));
        }
        if has_switches {
            code.push('\n');
        }

        // Device parameter state fields (runtime-adjustable)
        let device_params = device_param_template_data(ir);
        if !device_params.is_empty() {
            code.push_str("\n    // --- Runtime-adjustable device parameters ---\n");
            for dev in &device_params {
                for p in &dev.params {
                    code.push_str(&format!(
                        "    /// Device {} {} ({}) — runtime adjustable\n",
                        dev.dev_num, p.const_suffix, dev.device_type
                    ));
                    code.push_str(&format!(
                        "    pub device_{}_{}: f64,\n",
                        dev.dev_num, p.field_suffix
                    ));
                }
            }
        }

        // BJT self-heating thermal state
        let thermal_devices = self_heating_device_data(ir);
        if !thermal_devices.is_empty() {
            code.push_str("\n    // --- BJT self-heating thermal state ---\n");
            for td in &thermal_devices {
                code.push_str(&format!(
                    "    /// BJT {} junction temperature [K]\n\
                     \x20   pub device_{}_tj: f64,\n",
                    td.dev_num, td.dev_num
                ));
            }
        }

        // Stateful-device opaque state blocks (Phase 0c). Shared field emitter
        // with the DK path; empty when no device is stateful.
        {
            let sdevs = stateful_device_data(ir);
            if !sdevs.is_empty() {
                code.push('\n');
                code.push_str(&emit_stateful_state_fields(&sdevs));
            }
        }

        // Oversampling state
        let os_factor = ir.solver_config.oversampling_factor;
        if super::super::runtime_os::runtime(ir).is_some() {
            code.push_str(&super::super::runtime_os::os_state_fields(ir));
        } else if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "\n    /// Oversampler upsampling half-band filter state (single input)\n\
                     \x20   pub os_up_state: [f64; {}],\n\
                     \x20   /// Oversampler downsampling half-band filter state (per output)\n\
                     \x20   pub os_dn_state: [[f64; {}]; NUM_OUTPUTS],\n",
                os_info.state_size, os_info.state_size
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "    /// 4x oversampler outer upsampling filter state\n\
                         \x20   pub os_up_state_outer: [f64; {}],\n\
                         \x20   /// 4x oversampler outer downsampling filter state\n\
                         \x20   pub os_dn_state_outer: [[f64; {}]; NUM_OUTPUTS],\n",
                    os_info.state_size_outer, os_info.state_size_outer
                ));
            }
            code.push_str(&emit_inject_os_state_fields(ir));
        }

        // Runtime voltage sources (.runtime directive): host-driven values,
        // stamped into the augmented-MNA RHS by the per-sample emitter block.
        for rt in &ir.runtime_sources {
            code.push_str(&format!(
                "    /// Runtime value for voltage source {} (RHS row {}).\n\
                 \x20   /// A non-finite value reads as 0 for that host sample and is counted in\n\
                 \x20   /// `diag_runtime_nan_count`; the field itself is left as written.\n\
                 \x20   pub {}: f64,\n\
                 \x20   /// `{}` as the RHS stamps read it: sanitised once per host sample.\n\
                 \x20   {}{}: f64,\n",
                rt.vs_name,
                rt.vs_row,
                rt.field_name,
                rt.field_name,
                rt.field_name,
                super::super::runtime_inputs::SANITIZED_SUFFIX
            ));
        }

        // Authentic circuit noise — Phase 1 thermal state fields.
        // Empty string when noise mode is Off.
        if noise.enabled {
            code.push_str("\n    // --- Authentic circuit noise (Phase 1) ---\n");
            code.push_str(&noise.state_fields);
        }

        code.push_str("}\n\n");

        // Default impl
        code.push_str("impl Default for CircuitState {\n");
        code.push_str("    fn default() -> Self {\n");
        // Noise: prelude statements (e.g. `let noise_thermal_scale = …;`).
        // Must precede the struct literal so the field-shorthand bindings
        // referenced by `noise.default_fields` are in scope.
        if noise.enabled {
            code.push_str(&noise.default_stmts);
        }
        code.push_str("        let mut state = Self {\n");
        if has_cap_ic {
            code.push_str("            v_prev: V_PREV_IC_SEED,\n");
        } else if has_dc_op {
            code.push_str("            v_prev: DC_OP,\n");
        } else {
            code.push_str("            v_prev: [0.0; N],\n");
        }
        if carries_q_dot(ir) {
            code.push_str(&format!("            q_dot: {},\n", q_dot_start(ir)));
        }
        if has_dc_nl_ic_seed {
            code.push_str("            i_nl_prev: DC_NL_I_IC_SEED,\n");
            code.push_str("            i_nl_prev_prev: DC_NL_I_IC_SEED,\n");
        } else if has_dc_nl {
            code.push_str("            i_nl_prev: DC_NL_I,\n");
            code.push_str("            i_nl_prev_prev: DC_NL_I,\n");
        } else {
            code.push_str("            i_nl_prev: [0.0; M],\n");
            code.push_str("            i_nl_prev_prev: [0.0; M],\n");
        }
        if has_dc_op {
            code.push_str("            dc_operating_point: DC_OP,\n");
        } else {
            code.push_str("            dc_operating_point: [0.0; N],\n");
        }
        if multi_input {
            code.push_str("            inputs_prev: [0.0; NUM_INPUTS],\n");
        } else {
            code.push_str("            input_prev: 0.0,\n");
        }
        if ir.solver_config.has_inject_or_tap() {
            code.push_str("            injections_prev: [0.0; NUM_INJECT],\n");
        }
        code.push_str("            last_nr_iterations: 0,\n");
        if ir.behavioral_sources.iter().any(|b| b.time_dependent) {
            code.push_str("            sim_time: 0.0,\n");
            code.push_str("            bsrc_inv_dt: BSRC_INV_DT_DEFAULT,\n");
            code.push_str("            bsrc_half_dt: BSRC_HALF_DT_DEFAULT,\n");
            code.push_str("            bsrc_x_prev: [0.0; N_BSRC_SLOTS],\n");
            code.push_str("            bsrc_int_prev: [0.0; N_BSRC_SLOTS],\n");
        }
        for r in &ir.behavioral_scalar_runtimes {
            code.push_str(&format!(
                "            {}: {:.17e},\n",
                r.field_name, r.default
            ));
        }
        if !ir.dc_op_converged && m > 0 {
            code.push_str("            dc_settled: false,\n");
            if has_dc_nl {
                code.push_str("            settled_i_nl: DC_NL_I,\n");
            } else {
                code.push_str("            settled_i_nl: [0.0; M],\n");
            }
        }
        // Initialize DC blocking filter from the true t=0 output value so the
        // first sample sees zero delta: V_PREV_IC_SEED (matches v_prev) when
        // the circuit has IC=-bearing caps, else the plain DC operating point.
        if ir.dc_block {
            let output_nodes = &ir.solver_config.output_nodes;
            let dc_x: Vec<String> = output_nodes
                .iter()
                .map(|&node| {
                    if let Some(v_prev_ic) = &ir.v_prev_ic_seed {
                        if node < v_prev_ic.len() {
                            return fmt_f64(v_prev_ic[node]);
                        }
                    }
                    if has_dc_op && node < ir.dc_operating_point.len() {
                        fmt_f64(ir.dc_operating_point[node])
                    } else {
                        "0.0".to_string()
                    }
                })
                .collect();
            code.push_str(&format!(
                "            dc_block_x_prev: [{}],\n",
                dc_x.join(", ")
            ));
            code.push_str(&format!(
                "            dc_block_y_prev: [{}],\n",
                vec!["0.0"; output_nodes.len()].join(", ")
            ));
            code.push_str("            dc_block_r: DC_BLOCK_R,\n");
        }
        code.push_str("            diag_peak_output: 0.0,\n");
        code.push_str("            diag_clamp_count: 0,\n");
        code.push_str("            diag_nr_max_iter_count: 0,\n");
        code.push_str("            diag_region_exit_count: 0,\n");
        if super::super::helpers::has_reduced_device(ir) {
            code.push_str("            diag_reduced_model_exit_count: 0,\n");
        }
        code.push_str("            diag_be_fallback_count: 0,\n");
        code.push_str("            diag_unsolved_sample_count: 0,\n");
        if emits_hold(ir) {
            code.push_str("            diag_nr_hold_count: 0,\n");
        }
        if counts_unconverged_commit(ir, use_full_nodal) {
            code.push_str("            diag_nr_unconverged_commit_count: 0,\n");
        }
        code.push_str("            diag_be_latch_count: 0,\n");
        code.push_str("            diag_active_set_pin_count: 0,\n");
        code.push_str("            diag_nan_reset_count: 0,\n");
        code.push_str(
            "            diag_input_clamp_count: 0,\n            diag_input_nan_count: 0,\n            diag_runtime_nan_count: 0,\n",
        );
        code.push_str("            diag_magnitude_reset_count: 0,\n");
        code.push_str("            diag_substep_count: 0,\n");
        if schur_exact_seed(ir, use_full_nodal) {
            code.push_str("            diag_warm_start_fallback_count: 0,\n");
        }
        code.push_str(&super::super::subsample_fire::emit_subsample_fire_default_fields(ir));
        code.push_str("            diag_refactor_count: 0,\n");
        code.push_str("            diag_ls_fail_count: 0,\n");
        code.push_str("            diag_voltage_damp_count: 0,\n");
        if ir.solver_config.runtime_be_latch {
            for f in [
                "be_x_mean",
                "be_x_prev",
                "be_r1_num",
                "be_pow",
                "be_in_x_mean",
                "be_in_x_prev",
                "be_in_r1_num",
                "be_in_pow",
            ] {
                code.push_str(&format!("            {f}: 0.0,\n"));
            }
            code.push_str("            be_ref_in: 0.0,\n");
            code.push_str("            be_env: 0.0,\n");
            code.push_str("            be_ref: 0.0,\n");
            code.push_str(if super::super::runtime_os::runtime(ir).is_some() {
                "            be_ref_decay: be_latch_ref_decay(SAMPLE_RATE * OVERSAMPLING_FACTOR as f64, OVERSAMPLING_FACTOR),\n"
            } else {
                "            be_ref_decay: be_latch_ref_decay(SAMPLE_RATE * OVERSAMPLING_FACTOR as f64),\n"
            });
            code.push_str("            be_latched: false,\n");
        }
        if ir.solver_config.breakpoint_be {
            code.push_str("            breakpoint_be: 0,\n");
        }
        if m > 0 || !ir.behavioral_sources.is_empty() || !ir.saturating_inductors.is_empty() {
            code.push_str("            chord_lu: [[0.0; N]; N],\n");
            code.push_str("            chord_dr: [1.0; N],\n");
            code.push_str("            chord_dc: [1.0; N],\n");
            code.push_str("            chord_perm: {{ let mut p = [0usize; N]; let mut i = 0; while i < N { p[i] = i; i += 1; } p }},\n");
            code.push_str("            chord_j_dev: [0.0; M * M],\n");
            let nb = body_effect_mosfets(ir).len();
            if nb > 0 {
                code.push_str(&format!("            chord_body_gmb: [0.0; {nb}],\n"));
            }
            code.push_str("            chord_valid: false,\n");
            code.push_str("            chord_dense: false,\n");
            if use_full_nodal && has_be_instance(ir) {
                code.push_str("            chord_be_lu: [[0.0; N]; N],\n");
                code.push_str("            chord_be_dr: [1.0; N],\n");
                code.push_str("            chord_be_dc: [1.0; N],\n");
                code.push_str("            chord_be_perm: {{ let mut p = [0usize; N]; let mut i = 0; while i < N { p[i] = i; i += 1; } p }},\n");
                code.push_str("            chord_be_j_dev: [0.0; M * M],\n");
                let nb = body_effect_mosfets(ir).len();
                if nb > 0 {
                    code.push_str(&format!("            chord_be_body_gmb: [0.0; {nb}],\n"));
                }
                code.push_str("            chord_be_valid: false,\n");
                code.push_str("            chord_be_dense: false,\n");
            }
        }
        code.push_str("            a: A_DEFAULT,\n");
        code.push_str("            a_neg: A_NEG_DEFAULT,\n");
        code.push_str("            a_be: A_BE_DEFAULT,\n");
        code.push_str("            a_neg_be: A_NEG_BE_DEFAULT,\n");
        if use_full_nodal {
            // Cold fields go into Box<CircuitStateCold>
            code.push_str("            cold: Box::new(CircuitStateCold {\n");
            code.push_str("                s: S_DEFAULT,\n");
            if m > 0 {
                code.push_str("                k: K_DEFAULT,\n");
                code.push_str("                s_ni: S_NI_DEFAULT,\n");
            }
            code.push_str("                s_be: S_BE_DEFAULT,\n");
            if m > 0 {
                code.push_str("                k_be: K_BE_DEFAULT,\n");
                code.push_str("                s_ni_be: S_NI_BE_DEFAULT,\n");
            }
            if has_pots || has_switches || has_sat_ind {
                code.push_str("                g_work: G,\n");
                code.push_str("                c_work: C,\n");
            }
            code.push_str("            }),\n");
        } else {
            code.push_str("            s: S_DEFAULT,\n");
            if m > 0 {
                code.push_str("            k: K_DEFAULT,\n");
                code.push_str("            s_ni: S_NI_DEFAULT,\n");
            }
            code.push_str("            s_be: S_BE_DEFAULT,\n");
            if m > 0 {
                code.push_str("            k_be: K_BE_DEFAULT,\n");
                code.push_str("            s_ni_be: S_NI_BE_DEFAULT,\n");
            }
            if schur_exact_seed(ir, use_full_nodal) {
                for k in ["k", "k_be"] {
                    // NaN key: the first solve factors.
                    code.push_str(&format!(
                        "            ws_{k}_key: [[f64::NAN; M]; M],\n\
                         \x20           ws_{k}_lu: [[0.0; M]; M],\n\
                         \x20           ws_{k}_pr: [0; M],\n\
                         \x20           ws_{k}_pc: [0; M],\n\
                         \x20           ws_{k}_rank: 0,\n"
                    ));
                }
            }
        }

        if needs_rebuild_state && !use_full_nodal {
            code.push_str("            g_work: G,\n");
            code.push_str("            c_work: C,\n");
        }
        if needs_current_sr {
            // HOST rate (matches the DK path / `set_sample_rate` semantics).
            code.push_str(&format!(
                "            current_sample_rate: {:.17e},\n",
                ir.solver_config.sample_rate
            ));
        }
        code.push_str(super::super::runtime_os::state_field_init(ir));
        if needs_rebuild_state {
            code.push_str("            matrices_dirty: false,\n");
        }
        for (idx, pot) in ir.pots.iter().enumerate() {
            let r_nom = 1.0 / pot.g_nominal;
            code.push_str(&format!(
                "            pot_{}_resistance: {:.17e},\n\
                 \x20           pot_{}_resistance_prev: {:.17e},\n",
                idx, r_nom, idx, r_nom
            ));
        }

        for (idx, _sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!("            switch_{}_position: 0,\n", idx));
        }

        for dev in &device_params {
            for p in &dev.params {
                code.push_str(&format!(
                    "            device_{}_{}: DEVICE_{}_{},\n",
                    dev.dev_num, p.field_suffix, dev.dev_num, p.const_suffix
                ));
            }
        }

        for td in &thermal_devices {
            code.push_str(&format!(
                "            device_{}_tj: DEVICE_{}_TAMB,\n",
                td.dev_num, td.dev_num
            ));
        }

        // Stateful-device state blocks: seed via the shared Default emitter.
        code.push_str(&emit_stateful_default_fields(&stateful_device_data(ir)));

        if super::super::runtime_os::runtime(ir).is_some() {
            code.push_str(&super::super::runtime_os::os_state_inits(ir));
        } else if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "            os_up_state: [0.0; {}],\n\
                 \x20           os_dn_state: [[0.0; {}]; NUM_OUTPUTS],\n",
                os_info.state_size, os_info.state_size
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "            os_up_state_outer: [0.0; {}],\n\
                     \x20           os_dn_state_outer: [[0.0; {}]; NUM_OUTPUTS],\n",
                    os_info.state_size_outer, os_info.state_size_outer
                ));
            }
            code.push_str(&emit_inject_os_state_init(ir));
        }

        // Runtime voltage sources (.runtime directive): init to 0, and so does
        // the sanitised copy the RHS stamps read.
        for rt in &ir.runtime_sources {
            code.push_str(&format!(
                "            {}: 0.0,\n            {}{}: 0.0,\n",
                rt.field_name,
                rt.field_name,
                super::super::runtime_inputs::SANITIZED_SUFFIX
            ));
        }

        // Noise: per-source RNG arrays + scalars (matches `state_fields` order).
        if noise.enabled {
            code.push_str(&noise.default_fields);
        }

        code.push_str("        };\n");
        code.push_str("        state.warmup();\n");
        code.push_str("        state\n");
        code.push_str("    }\n");
        code.push_str("}\n\n");

        // impl CircuitState
        code.push_str("impl CircuitState {\n");

        // Plugin-driven scalar param setters (.runtime <name> min max as field).
        for r in &ir.behavioral_scalar_runtimes {
            code.push_str(&format!(
                "    /// Set plugin scalar `{}` (clamped to [{}, {}]).\n\
                 \x20   ///\n\
                 \x20   /// A non-finite value is counted in `diag_runtime_nan_count`: ±inf\n\
                 \x20   /// clamps to the range end, NaN leaves the value unchanged.\n",
                r.name, r.min, r.max
            ));
            code.push_str(&format!(
                "    #[inline]\n    pub fn set_runtime_{}(&mut self, value: f64) {{\n",
                r.field_name
            ));
            code.push_str(&format!(
                "        if !value.is_finite() {{ self.diag_runtime_nan_count += 1; if value.is_nan() {{ return; }} }}\n\
                 \x20       self.{} = value.clamp({:.17e}, {:.17e});\n",
                r.field_name, r.min, r.max
            ));
            code.push_str("    }\n\n");
        }

        code.push_str(&super::super::runtime_os::emit_methods(ir));
        // reset()
        code.push_str(
            "    /// Reset to the factory state.\n\
             \x20   ///\n\
             \x20   /// \"Factory state\" means everything returns to what codegen baked in:\n\
             \x20   /// the DC operating point, the baked nonlinear bias currents, and\n\
             \x20   /// nominal pot/switch values — all restored together so they agree.\n\
             \x20   /// The sample rate is kept (it is not a control; the matrices are\n\
             \x20   /// rebuilt for it). Controls are the caller's: re-apply pots and\n\
             \x20   /// switches after `reset()`, as after construction.\n",
        );
        let restores_from = code.len();
        code.push_str("    pub fn reset(&mut self) {\n");
        if has_cap_ic {
            code.push_str("        self.v_prev = V_PREV_IC_SEED;\n");
        } else {
            code.push_str("        self.v_prev = self.dc_operating_point;\n");
        }
        if carries_q_dot(ir) {
            code.push_str(&format!("        self.q_dot = {};\n", q_dot_start(ir)));
        }
        if has_dc_nl_ic_seed {
            // has_cap_ic already forced `self.v_prev = V_PREV_IC_SEED` above,
            // overriding the dc_settled/quiescent warmup machinery below for
            // v_prev — i_nl_prev must follow the same override so the pair
            // stays KCL-consistent (see the pairing comment on
            // `dc_nl_currents_ic_seed` in `codegen/ir/mod.rs`).
            code.push_str("        self.i_nl_prev = DC_NL_I_IC_SEED;\n");
            code.push_str("        self.i_nl_prev_prev = DC_NL_I_IC_SEED;\n");
        } else if !ir.dc_op_converged && m > 0 {
            code.push_str("        if self.dc_settled {\n");
            code.push_str("            self.i_nl_prev = self.settled_i_nl;\n");
            code.push_str("            self.i_nl_prev_prev = self.settled_i_nl;\n");
            code.push_str("        } else {\n");
            if has_dc_nl {
                code.push_str("            self.i_nl_prev = DC_NL_I;\n");
                code.push_str("            self.i_nl_prev_prev = DC_NL_I;\n");
            } else {
                code.push_str("            self.i_nl_prev = [0.0; M];\n");
                code.push_str("            self.i_nl_prev_prev = [0.0; M];\n");
            }
            code.push_str("        }\n");
        } else if has_dc_nl {
            code.push_str("        self.i_nl_prev = DC_NL_I;\n");
            code.push_str("        self.i_nl_prev_prev = DC_NL_I;\n");
        } else {
            code.push_str("        self.i_nl_prev = [0.0; M];\n");
            code.push_str("        self.i_nl_prev_prev = [0.0; M];\n");
        }
        if multi_input {
            code.push_str("        self.inputs_prev = [0.0; NUM_INPUTS];\n");
        } else {
            code.push_str("        self.input_prev = 0.0;\n");
        }
        if ir.solver_config.has_inject_or_tap() {
            code.push_str("        self.injections_prev = [0.0; NUM_INJECT];\n");
        }
        code.push_str("        self.last_nr_iterations = 0;\n");
        if ir.behavioral_sources.iter().any(|b| b.time_dependent) {
            code.push_str("        self.sim_time = 0.0;\n");
            code.push_str("        self.bsrc_x_prev = [0.0; N_BSRC_SLOTS];\n");
            code.push_str("        self.bsrc_int_prev = [0.0; N_BSRC_SLOTS];\n");
        }
        if m > 0 {
            code.push_str("        self.chord_valid = false;\n");
            if use_full_nodal && has_be_instance(ir) {
                code.push_str("        self.chord_be_valid = false;\n");
            }
        }
        // Re-init DC blocker from the same t=0 state as v_prev above
        // (prevents transient on reset)
        if ir.dc_block {
            emit_dc_block_history_reseed(&mut code, ir, "        ", "self", has_cap_ic);
        }
        code.push_str("        self.diag_peak_output = 0.0;\n");
        code.push_str("        self.diag_clamp_count = 0;\n");
        code.push_str("        self.diag_nr_max_iter_count = 0;\n");
        code.push_str("        self.diag_region_exit_count = 0;\n");
        if super::super::helpers::has_reduced_device(ir) {
            code.push_str("        self.diag_reduced_model_exit_count = 0;\n");
        }
        code.push_str("        self.diag_be_fallback_count = 0;\n");
        code.push_str("        self.diag_unsolved_sample_count = 0;\n");
        if emits_hold(ir) {
            code.push_str("        self.diag_nr_hold_count = 0;\n");
        }
        if counts_unconverged_commit(ir, use_full_nodal) {
            code.push_str("        self.diag_nr_unconverged_commit_count = 0;\n");
        }
        code.push_str("        self.diag_be_latch_count = 0;\n");
        code.push_str("        self.diag_active_set_pin_count = 0;\n");
        code.push_str("        self.diag_nan_reset_count = 0;\n");
        code.push_str(
            "        self.diag_input_clamp_count = 0;\n        self.diag_input_nan_count = 0;\n        self.diag_runtime_nan_count = 0;\n",
        );
        code.push_str("        self.diag_magnitude_reset_count = 0;\n");
        code.push_str("        self.diag_voltage_damp_count = 0;\n");
        code.push_str("        self.diag_substep_count = 0;\n");
        if schur_exact_seed(ir, use_full_nodal) {
            code.push_str("        self.diag_warm_start_fallback_count = 0;\n");
        }
        code.push_str(&super::super::subsample_fire::emit_subsample_fire_reset(ir));
        code.push_str("        self.diag_refactor_count = 0;\n");
        code.push_str("        self.diag_ls_fail_count = 0;\n");
        if ir.solver_config.runtime_be_latch {
            for f in [
                "be_x_mean",
                "be_x_prev",
                "be_r1_num",
                "be_pow",
                "be_in_x_mean",
                "be_in_x_prev",
                "be_in_r1_num",
                "be_in_pow",
            ] {
                code.push_str(&format!("        self.{f} = 0.0;\n"));
            }
            code.push_str("        self.be_ref_in = 0.0;\n");
            code.push_str("        self.be_env = 0.0;\n");
            code.push_str("        self.be_ref = 0.0;\n");
            code.push_str("        self.be_latched = false;\n");
        }
        if ir.solver_config.breakpoint_be {
            code.push_str("        self.breakpoint_be = 0;\n");
        }
        let cp = if use_full_nodal {
            "self.cold."
        } else {
            "self."
        };
        // Working G/C snap back to nominal here; every rate-dependent matrix
        // derived from them is restored at the END of reset(), once the pot,
        // switch and saturation fields below have also been restored.
        if has_pots || has_switches || has_sat_ind {
            code.push_str(&format!("        {}g_work = G;\n", cp));
            code.push_str(&format!("        {}c_work = C;\n", cp));
        }
        for (idx, pot) in ir.pots.iter().enumerate() {
            let r_nom = 1.0 / pot.g_nominal;
            code.push_str(&format!(
                "        self.pot_{}_resistance = {:.17e};\n\
                 \x20       self.pot_{}_resistance_prev = {:.17e};\n",
                idx, r_nom, idx, r_nom
            ));
        }
        for (idx, _sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!("        self.switch_{}_position = 0;\n", idx));
        }
        for dev in &device_params {
            for p in &dev.params {
                code.push_str(&format!(
                    "        self.device_{}_{} = DEVICE_{}_{};\n",
                    dev.dev_num, p.field_suffix, dev.dev_num, p.const_suffix
                ));
            }
        }
        for td in &thermal_devices {
            code.push_str(&format!(
                "        self.device_{}_tj = DEVICE_{}_TAMB;\n",
                td.dev_num, td.dev_num
            ));
        }
        // Stateful-device state blocks: restore to seed (shared emitter, `self`).
        code.push_str(&emit_stateful_state_restore(
            &stateful_device_data(ir),
            "self.",
        ));
        if super::super::runtime_os::runtime(ir).is_some() {
            code.push_str("        self.reset_oversampler();\n");
        } else if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "        self.os_up_state = [0.0; {}];\n\
                 \x20       self.os_dn_state = [[0.0; {}]; NUM_OUTPUTS];\n",
                os_info.state_size, os_info.state_size
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "        self.os_up_state_outer = [0.0; {}];\n\
                     \x20       self.os_dn_state_outer = [[0.0; {}]; NUM_OUTPUTS];\n",
                    os_info.state_size_outer, os_info.state_size_outer
                ));
            }
            code.push_str(&emit_inject_os_state_reset(ir, "self", "        "));
        }
        // Runtime voltage sources: clear field to 0 so the VS contributes no
        // RHS stamp until the host writes a new value; the sanitised copy too.
        for rt in &ir.runtime_sources {
            code.push_str(&format!(
                "        self.{} = 0.0;\n        self.{}{} = 0.0;\n",
                rt.field_name,
                rt.field_name,
                super::super::runtime_inputs::SANITIZED_SUFFIX
            ));
        }
        // Noise: re-seed RNGs from stored master seed; user prefs untouched.
        if noise.enabled {
            code.push_str(&noise.reset_body);
        }
        // Restore the rate-dependent matrices so they agree with the pot,
        // switch and saturation state restored above. Two defects fixed here
        // (F9):
        //
        //  * `a`/`a_neg`/`a_be`/`a_neg_be` were never restored at all, and
        //    `matrices_dirty` was never set — so a `reset()` after a pot move
        //    left the working A matrices holding the moved-pot stamp while
        //    `g_work` read nominal, with no rebuild scheduled. The DK path has
        //    always set `matrices_dirty` at this point in `reset()`
        //    (`templates/rust/state.rs.tera`); the nodal path never did.
        //
        //  * The `*_DEFAULT` constants bake `SAMPLE_RATE`, so reloading them
        //    unconditionally installed codegen-rate Schur matrices whenever the
        //    host ran at any other rate, while `a`/`a_neg` kept the live rate.
        //
        // The dispatch mirrors `set_sample_rate()`. Its fast path is gated on
        // `all_default` as well as the rate; here that precondition holds by
        // construction, because every pot and switch was just restored to
        // nominal a few lines above.
        let emit_matrix_defaults = |code: &mut String, ind: &str| {
            code.push_str(&format!("{ind}self.a = A_DEFAULT;\n"));
            code.push_str(&format!("{ind}self.a_neg = A_NEG_DEFAULT;\n"));
            code.push_str(&format!("{ind}self.a_be = A_BE_DEFAULT;\n"));
            code.push_str(&format!("{ind}self.a_neg_be = A_NEG_BE_DEFAULT;\n"));
            code.push_str(&format!("{ind}{cp}s = S_DEFAULT;\n"));
            if m > 0 {
                code.push_str(&format!("{ind}{cp}k = K_DEFAULT;\n"));
                code.push_str(&format!("{ind}{cp}s_ni = S_NI_DEFAULT;\n"));
            }
            code.push_str(&format!("{ind}{cp}s_be = S_BE_DEFAULT;\n"));
            if m > 0 {
                code.push_str(&format!("{ind}{cp}k_be = K_BE_DEFAULT;\n"));
                code.push_str(&format!("{ind}{cp}s_ni_be = S_NI_BE_DEFAULT;\n"));
            }
        };
        if needs_current_sr {
            code.push_str("        if (self.current_sample_rate - SAMPLE_RATE).abs() < 0.5 {\n");
            emit_matrix_defaults(&mut code, "            ");
            code.push_str("        } else {\n");
            code.push_str(&format!(
                "            self.rebuild_matrices(self.current_sample_rate * {});\n",
                super::super::runtime_os::factor_f64_literal(ir, "self")
            ));
            code.push_str("        }\n");
        } else {
            emit_matrix_defaults(&mut code, "        ");
        }
        if needs_rebuild_state {
            code.push_str("        self.matrices_dirty = false;\n");
        }
        code.push_str("        self.warmup();\n");
        code.push_str("    }\n\n");

        // warmup()
        code.push_str(
            "    /// Run silent warmup samples to settle into the correct operating point.\n",
        );
        code.push_str("    ///\n");
        code.push_str(
            "    /// Call after construction or reset() to ensure the circuit settles into\n",
        );
        code.push_str(
            "    /// the physically correct basin of attraction. Without warmup, circuits\n",
        );
        code.push_str("    /// with high-gain op-amps may lock into a parasitic equilibrium.\n");
        code.push_str("    ///\n");
        code.push_str("    /// The default `CircuitState::default()` calls this automatically.\n");
        code.push_str("    pub fn warmup(&mut self) {\n");
        // `has_cap_ic` skips the low-rate destructive DC settle below: an
        // IC=-bearing capacitor's prescribed initial voltage is the whole
        // point of the feature (SPICE `.IC`/UIC semantics) — silently
        // fast-forwarding 1000 samples of silence at 200 Hz toward the
        // plain (non-IC) quiescent point before any real audio is
        // processed would erase it before the caller ever sees the IC
        // state. v_prev/i_nl_prev were already seeded consistently from
        // V_PREV_IC_SEED/DC_NL_I_IC_SEED at construction/reset(); only the
        // standard 50-sample warmup below applies.
        if !ir.dc_op_converged && m > 0 && !has_cap_ic {
            let target_rate =
                ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
            code.push_str("        if !self.dc_settled {\n");
            code.push_str(
                "            // DC OP didn't converge — fast-forward to DC steady state.\n",
            );
            code.push_str("            // 1000 samples at 200Hz = 5 seconds of circuit time,\n");
            code.push_str(
                "            // enough for coupling caps up to RC ~1s to fully charge.\n",
            );
            code.push_str("            self.rebuild_matrices(200.0);\n");
            code.push_str("            for _ in 0..1000 {\n");
            code.push_str(&emit_warmup_call(ir, "                ", false));
            code.push_str("            }\n");
            // A runtime-oversampling build settles at the running factor.
            if super::super::runtime_os::runtime(ir).is_some() {
                code.push_str(
                    "            self.rebuild_matrices(SAMPLE_RATE * (self.oversampling as f64));\n",
                );
            } else {
                code.push_str(&format!(
                    "            self.rebuild_matrices({:.17e});\n",
                    target_rate,
                ));
            }
            code.push_str("            // Cache settled state for future resets\n");
            code.push_str("            self.dc_operating_point = self.v_prev;\n");
            code.push_str("            self.settled_i_nl = self.i_nl_prev;\n");
            code.push_str("            self.dc_settled = true;\n");
            code.push_str("        }\n");
        }
        code.push_str("        for _ in 0..50 {\n");
        code.push_str(&emit_warmup_call(ir, "            ", false));
        code.push_str("        }\n");
        code.push_str("    }\n\n");

        // set_dc_operating_point()
        code.push_str("    /// Set DC operating point (call after DC analysis)\n");
        code.push_str("    pub fn set_dc_operating_point(&mut self, v_dc: [f64; N]) {\n");
        code.push_str("        self.dc_operating_point = v_dc;\n");
        code.push_str("        self.v_prev = v_dc;\n");
        if carries_q_dot(ir) {
            code.push_str("        self.q_dot = [0.0; N];\n");
        }
        code.push_str("    }\n\n");

        // dc_op() accessor. Lets plugins
        // read the baked DC bias point without reaching into the dynamic
        // `v_prev` field (which carries per-sample updates).
        code.push_str("    /// Read the baked DC operating point for this circuit.\n");
        code.push_str("    ///\n");
        code.push_str(
            "    /// Solution to the nonlinear DC system computed at codegen time (at nominal\n",
        );
        code.push_str(
            "    /// pot/switch values) and frozen into `DC_OP`. Prefer this over `v_prev` when\n",
        );
        code.push_str(
            "    /// you want the *designed* bias point — `v_prev` carries per-sample dynamics.\n",
        );
        code.push_str("    #[inline]\n");
        code.push_str("    pub fn dc_op(&self) -> &[f64; N] {\n");
        code.push_str("        &self.dc_operating_point\n");
        code.push_str("    }\n\n");

        // recompute_dc_op(). The nodal route ships a
        // stub: emits the method surface uniformly with the DK
        // path but the body only bumps `diag_nr_max_iter_count` and
        // returns. Nodal-routed plugins use the
        // `WARMUP_SAMPLES_RECOMMENDED` silence loop (the documented path
        // for nodal circuits). A nodal NR body is not implemented — see `emit_recompute_dc_op_body_nodal` in
        // `dc_op_emitter.rs` for the rationale. Feature-gated so the
        // default codegen path stays byte-identical.
        if ir.solver_config.emit_dc_op_recompute {
            let body = super::super::dc_op_emitter::emit_recompute_dc_op_body_nodal(ir)
                .expect("nodal stub body must be infallible");
            code.push_str(
                "    /// Re-solve the DC operating point at the current pot/switch values.\n\
                 \x20   ///\n\
                 \x20   /// **Not audio-thread safe.** Intended for plugin initialization\n\
                 \x20   /// after applying per-instance pot/switch jitter.\n\
                 \x20   ///\n\
                 \x20   /// # Nodal route: stub only\n\
                 \x20   ///\n\
                 \x20   /// The runtime DC OP solve is shipped on the DK path only. Nodal\n\
                 \x20   /// circuits (Schur and full-LU alike) use the\n\
                 \x20   /// `WARMUP_SAMPLES_RECOMMENDED` silence loop — this is\n\
                 \x20   /// the documented path for nodal circuits, not a placeholder. The\n\
                 \x20   /// warmup loop runs the full per-sample NR and is guaranteed to\n\
                 \x20   /// converge to the physically correct DC OP.\n\
                 \x20   ///\n\
                 \x20   /// Calling this method on a nodal-routed circuit bumps\n\
                 \x20   /// `diag_nr_max_iter_count` and returns without touching\n\
                 \x20   /// `dc_operating_point` or `v_prev`. The standard fallback\n\
                 \x20   /// pattern — check the counter, run warmup on tick — handles\n\
                 \x20   /// both DK-convergence-failure and nodal-stub cases uniformly,\n\
                 \x20   /// so plugin host code doesn't need a solver-path branch.\n\
                 \x20   ///\n\
                 \x20   /// See `docs/aidocs/DC_OP.md` \"Runtime DC OP recompute\" for the\n\
                 \x20   /// DK-path semantics.\n\
                 \x20   pub fn recompute_dc_op(&mut self) {\n",
            );
            code.push_str(&body);
            code.push_str("    }\n\n");

            // settle_dc_op() — the "recompute + warmup fallback" wrapper.
            // Identical body to the DK path; the path-specific behavior
            // lives in `recompute_dc_op`. On the nodal path the stub
            // always ticks the counter, so this always falls through to
            // the warmup loop — that's the intended contract.
            code.push_str(
                "    /// Settle to the DC operating point at current pot / switch / device\n\
                 \x20   /// values — the \"recompute with warmup fallback\" convenience wrapper.\n\
                 \x20   ///\n\
                 \x20   /// Calls [`recompute_dc_op`](CircuitState::recompute_dc_op) first;\n\
                 \x20   /// if the NR failed to update state (detected via\n\
                 \x20   /// `diag_nr_max_iter_count` advancing), falls back to\n\
                 \x20   /// `WARMUP_SAMPLES_RECOMMENDED` iterations of `process_sample(0.0, self)`.\n\
                 \x20   /// Always leaves the circuit at a valid equilibrium — either the exact\n\
                 \x20   /// NR solution or the warmup-loop-converged steady state.\n\
                 \x20   ///\n\
                 \x20   /// On this (nodal full-LU) path the runtime NR is a permanent stub,\n\
                 \x20   /// so this method always falls through to the warmup loop today.\n\
                 \x20   /// Plugin code doesn't need to branch on path — the contract is\n\
                 \x20   /// uniform and the wrapper handles the routing.\n\
                 \x20   ///\n\
                 \x20   /// **Not audio-thread safe.** Call from plugin initialization or\n\
                 \x20   /// parameter-change callbacks.\n\
                 \x20   pub fn settle_dc_op(&mut self) {\n",
            );
            code.push_str(&super::super::dc_op_emitter::emit_settle_dc_op_body(ir));
            code.push_str("    }\n\n");
        }

        // dc_op_dump() — pretty printer using NODE_<NAME> constants from P3.
        if !ir.named_constants.nodes.is_empty() {
            code.push_str(
                "    /// Pretty-print the DC operating point with user-assigned node names.\n",
            );
            code.push_str("    ///\n");
            code.push_str(
                "    /// Writes one line per named node to `stderr`. Intended for plugin\n",
            );
            code.push_str(
                "    /// bring-up diagnostics, not the audio thread (formatting allocates).\n",
            );
            code.push_str("    pub fn dc_op_dump(&self) {\n");
            code.push_str("        let dc = &self.dc_operating_point;\n");
            for (name, idx) in &ir.named_constants.nodes {
                code.push_str(&format!(
                    "        eprintln!(\"  V({}) = {{:+.4}} V\", dc[{}]);\n",
                    name, idx
                ));
            }
            code.push_str("    }\n\n");
        }

        // set_sample_rate()
        code.push_str(
            "    /// Recompute all sample-rate-dependent matrices for a new sample rate.\n",
        );
        code.push_str("    ///\n");
        code.push_str(
            "    /// Call this once during plugin initialization (NOT on the audio thread).\n",
        );
        code.push_str("    /// Rebuilds A, A_neg, A_be, A_neg_be from stored G and C matrices.\n");
        code.push_str("    pub fn set_sample_rate(&mut self, sample_rate: f64) {\n");
        code.push_str("        if !sample_rate.is_finite() {\n");
        code.push_str(
            "            self.diag_runtime_nan_count += 1; // counted, nothing changes\n",
        );
        code.push_str("            return;\n");
        code.push_str("        }\n");
        code.push_str("        if !(sample_rate > 0.0) {\n");
        code.push_str("            return;\n");
        code.push_str("        }\n\n");
        // Noise: refresh thermal_scale at the new fs (and noise_fs cache).
        // Runs unconditionally so the fast-path early return below still
        // sees the updated coefficient — temperature_k may have changed
        // since codegen.
        if noise.enabled {
            code.push_str(&noise.set_sample_rate_body);
            code.push('\n');
        }
        // Record the HOST rate BEFORE any early return, so every downstream
        // consumer (lazy rebuild_matrices, slew dt, substep alpha) sees the
        // rate the host actually requested. This used to be assigned only on
        // the full-rebuild path below the same-rate early return, so after
        // returning to the codegen rate the field kept the previous rate and
        // every later pot-triggered rebuild ran at a stale rate.
        if needs_current_sr {
            code.push_str("        self.current_sample_rate = sample_rate;\n\n");
        }
        if ir.solver_config.runtime_be_latch {
            let decay = if super::super::runtime_os::runtime(ir).is_some() {
                "be_latch_ref_decay(sample_rate * self.oversampling as f64, self.oversampling)"
            } else {
                "be_latch_ref_decay(sample_rate * OVERSAMPLING_FACTOR as f64)"
            };
            code.push_str(&format!(
                "        // The latch reference's memory follows the ring decay at this rate.\n\
                 \x20       self.be_ref_decay = {decay};\n\n",
            ));
        }
        // Stateful-device rate-baked coefficients (Phase 0c). Empty in 1a
        // (CdsLdr recomputes its coefficient live from current_sample_rate).
        code.push_str(&emit_stateful_set_sample_rate_body(
            ir,
            &stateful_device_data(ir),
        ));
        code.push_str("        // If same as codegen sample rate, reset to defaults\n");
        code.push_str("        if (sample_rate - SAMPLE_RATE).abs() < 0.5 {\n");
        // The *_DEFAULT matrices bake NOMINAL pot resistances and switch
        // position 0. Loading them while a pot/switch is off-default would
        // silently desync matrices from the pot/switch state fields, so the
        // fast path is only taken when everything is at its default; any
        // moved pot/switch falls through to the full rebuild (which stamps
        // from the live g_work/c_work).
        if has_pots || has_switches {
            code.push_str("            let all_default = true\n");
            for (idx, pot) in ir.pots.iter().enumerate() {
                let r_nom = 1.0 / pot.g_nominal;
                code.push_str(&format!(
                    "                && (self.pot_{idx}_resistance - {r_nom:.17e}).abs() <= {r_nom:.17e} * 1e-6\n"
                ));
            }
            for (idx, _sw) in ir.switches.iter().enumerate() {
                code.push_str(&format!(
                    "                && self.switch_{idx}_position == 0\n"
                ));
            }
            code.push_str("                ;\n");
            code.push_str("            if all_default {\n");
        }
        code.push_str("            self.a = A_DEFAULT;\n");
        code.push_str("            self.a_neg = A_NEG_DEFAULT;\n");
        code.push_str("            self.a_be = A_BE_DEFAULT;\n");
        code.push_str("            self.a_neg_be = A_NEG_BE_DEFAULT;\n");
        if matches!(
            ir.solver_config.opamp_rail_mode,
            crate::codegen::OpampRailMode::BoyleDiodes
        ) {
            code.push_str(
                "            // BoyleDiodes-only: invalidate the cross-timestep chord LU.\n",
            );
            code.push_str("            // The chord_lu / chord_j_dev cache holds a *factored*\n");
            code.push_str("            // matrix and is paired with a specific (v_prev, j_dev)\n");
            code.push_str(
                "            // pair from the most recent refactor. After the 50-sample\n",
            );
            code.push_str(
                "            // default warmup, chord_j_dev still reflects the deeply-\n",
            );
            code.push_str("            // reverse-biased catch diodes (j_dev ≈ 1e-31), and the\n");
            code.push_str(
                "            // adaptive trigger doesn't fire on the first signal sample\n",
            );
            code.push_str(
                "            // because both `j_dev` and `chord_j_dev` are still ≈ 1e-31\n",
            );
            code.push_str(
                "            // — but v_prev has drifted enough during warmup that the\n",
            );
            code.push_str("            // back-solve against the stale factor produces a wrong\n");
            code.push_str(
                "            // linear prediction. Forcing a refactor here is harmless\n",
            );
            code.push_str("            // for non-BoyleDiodes modes (no Schottky-class circuit\n");
            code.push_str(
                "            // exhibits the same staleness pattern), but it shifts the\n",
            );
            code.push_str(
                "            // first-iteration NR state for leveling-amplifier and other\n",
            );
            code.push_str("            // attack-timing-sensitive control circuits, so we gate\n");
            code.push_str("            // the reset on BoyleDiodes mode only.\n");
            code.push_str("            self.chord_valid = false;\n");
            if use_full_nodal && has_be_instance(ir) {
                code.push_str("            self.chord_be_valid = false;\n");
            }
        }
        code.push_str(&format!("            {}s = S_DEFAULT;\n", cp));
        code.push_str(&format!("            {}s_be = S_BE_DEFAULT;\n", cp));
        if m > 0 {
            code.push_str(&format!("            {}k = K_DEFAULT;\n", cp));
            code.push_str(&format!("            {}s_ni = S_NI_DEFAULT;\n", cp));
            code.push_str(&format!("            {}k_be = K_BE_DEFAULT;\n", cp));
            code.push_str(&format!("            {}s_ni_be = S_NI_BE_DEFAULT;\n", cp));
        }
        // Sub-step (2× rate) matrices must also snap back to defaults —
        // they were rebuilt by any earlier off-rate set_sample_rate call.
        if ir.dc_block {
            code.push_str("            self.dc_block_r = DC_BLOCK_R;\n");
        }
        code.push_str(
            "            // POLICY: transient state — oversampler filter history,\n\
             \x20           // DC-blocker history, solver v_prev/i_nl history — is\n\
             \x20           // PRESERVED here. Only coefficients and matrices are\n\
             \x20           // restored: a same-rate call is a no-op reconfiguration\n\
             \x20           // and must not click (DK-template parity).\n",
        );
        code.push_str("            return;\n");
        if has_pots || has_switches {
            // Close the `if all_default` guard — off-default pots/switches
            // rebuild at the live values, still preserving transient state
            // (mirrors the DK pot-variant same-rate off-default arm).
            code.push_str("            }\n");
            code.push_str(
                "            // Same rate but pots/switches are off-default: rebuild the\n\
                 \x20           // matrices at the live values. Rate-dependent coefficients\n\
                 \x20           // equal their baked defaults at this rate; transient state\n\
                 \x20           // is preserved (see POLICY above).\n",
            );
            if ir.dc_block {
                code.push_str("            self.dc_block_r = DC_BLOCK_R;\n");
            }
            if ir.behavioral_sources.iter().any(|b| b.time_dependent) {
                code.push_str(&format!(
                    "            self.bsrc_inv_dt = sample_rate * {};\n\
                     \x20           self.bsrc_half_dt = 0.5 / (sample_rate * {});\n",
                    super::super::runtime_os::factor_f64_literal(ir, "self"),
                    super::super::runtime_os::factor_f64_literal(ir, "self")
                ));
            }
            code.push_str(&format!(
                "            self.rebuild_matrices(sample_rate * {});\n",
                super::super::runtime_os::factor_f64_literal(ir, "self")
            ));
            code.push_str("            return;\n");
        }
        code.push_str("        }\n\n");

        code.push_str(&format!(
            "        let internal_rate = sample_rate * {};\n",
            super::super::runtime_os::factor_f64_literal(ir, "self")
        ));
        if ir.behavioral_sources.iter().any(|b| b.time_dependent) {
            code.push_str("        self.bsrc_inv_dt = internal_rate;\n");
            code.push_str("        self.bsrc_half_dt = 0.5 / internal_rate;\n");
        }
        code.push_str("        self.rebuild_matrices(internal_rate);\n\n");

        // DC block recomputation
        if ir.dc_block {
            // The cutoff is interpolated as a literal (`5.0`) so the EMITTED
            // expression is unchanged; generated code has no path to
            // `crate::codegen::policy`.
            code.push_str(&format!(
                "        // Recompute DC blocking coefficient\n\
                 \x20       self.dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * {} / internal_rate;\n",
                crate::codegen::policy::dc_block_cutoff_hz_literal()
            ));
            emit_dc_block_history_reseed(&mut code, ir, "        ", "self", false);
        }

        if super::super::runtime_os::runtime(ir).is_some() {
            code.push_str("        self.reset_oversampler();\n");
        } else if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "        self.os_up_state = [0.0; {}];\n\
                 \x20       self.os_dn_state = [[0.0; {}]; NUM_OUTPUTS];\n",
                os_info.state_size, os_info.state_size
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "        self.os_up_state_outer = [0.0; {}];\n\
                     \x20       self.os_dn_state_outer = [[0.0; {}]; NUM_OUTPUTS];\n",
                    os_info.state_size_outer, os_info.state_size_outer
                ));
            }
            code.push_str(&emit_inject_os_state_reset(ir, "self", "        "));
        }

        code.push_str("    }\n\n");

        // rebuild_matrices: recompute A/A_neg/A_be/A_neg_be from G+C
        let (g_src, c_src) = live_g_c(ir, use_full_nodal, "self");
        let (g_src, c_src) = (g_src.as_str(), c_src.as_str());

        // Which Schur matrices does the generated per-sample code actually read?
        //
        // * S / K / S_NI and their BE twins are read only by the nodal-Schur
        //   per-sample path. The full-LU path solves against A / A_be /
        //   chord_lu and never touches them, so building them there is dead
        //   work — three N×N inversions plus three Schur products per pot,
        //   switch, or sample-rate change.
        //
        // The full-LU sub-step builds its own `a_neg_sub` locally at the
        // runtime rate; nothing persists sub-step matrices.
        let schur_matrices_live = !use_full_nodal;
        code.push_str("    /// Recompute A, A_neg, A_be, A_neg_be from G and C.\n");
        code.push_str("    ///\n");
        code.push_str("    /// Called by set_sample_rate, set_pot, and set_switch.\n");
        if schur_matrices_live {
            code.push_str(
                "    /// Also recomputes the Schur matrices S, K, S_NI (and BE variants),\n",
            );
            code.push_str("    /// which includes O(N^3) matrix inversion.\n");
        } else {
            code.push_str(
                "    /// The Schur matrices (S/K/S_NI) are not rebuilt: the full-LU path\n",
            );
            code.push_str("    /// never reads them.\n");
        }
        super::super::runtime_os::switch_restores(ir, &mut code, restores_from);
        code.push_str("    pub fn rebuild_matrices(&mut self, internal_rate: f64) {\n");
        if ir.solver_config.backward_euler {
            code.push_str("        let alpha = internal_rate; // backward Euler: alpha = 1/T\n");
        } else {
            code.push_str("        let alpha = 2.0 * internal_rate; // trapezoidal: alpha = 2/T\n");
        }
        code.push_str("        let alpha_be = internal_rate;\n");
        code.push('\n');

        // Build A = G + alpha*C and the history matrix A_neg = alpha*C (both
        // integrators; the charge form carries no G term in its history).
        // No Boyle elimination here — the op-amp Gm is stamped directly in G (ideal model).
        let a_neg_formula = format!("alpha * {}[i][j]", c_src);
        code.push_str(&format!(
            "        for i in 0..N {{\n\
             \x20           for j in 0..N {{\n\
             \x20               self.a[i][j] = {}[i][j] + alpha * {}[i][j];\n\
             \x20               self.a_neg[i][j] = {};\n\
             \x20               self.a_be[i][j] = {}[i][j] + alpha_be * {}[i][j];\n\
             \x20               self.a_neg_be[i][j] = alpha_be * {}[i][j];\n\
             \x20           }}\n\
             \x20       }}\n",
            g_src, c_src, a_neg_formula, g_src, c_src, c_src
        ));

        // Zero the algebraic rows in A_neg and A_neg_be: the same rows the IR
        // zeroes in the baked constants (not inductor rows, not parasitic-BJT
        // internal nodes).
        for (lo, hi) in history_zero_row_ranges(ir) {
            code.push_str(&format!(
                "        for i in {}..{} {{\n\
                 \x20           for j in 0..N {{\n\
                 \x20               self.a_neg[i][j] = 0.0;\n\
                 \x20               self.a_neg_be[i][j] = 0.0;\n\
                 \x20           }}\n\
                 \x20       }}\n",
                lo, hi
            ));
        }

        // Emit `S_NI = S · N_i` (N×M) followed by `K = N_v · S_NI` (M×M) for one
        // Schur triple.
        //
        // K was previously formed directly as `K[i][j] = Σ_a N_v[i][a] · (Σ_b
        // S[a][b] · N_i[b][j])`, recomputing the inner `Σ_b` — which depends
        // only on (a, j), not on i — for every one of the M² outputs. That is
        // O(M²N²) where the factored form is O(N²M + M²N): 135 424 multiply-adds
        // instead of 19 872 at N=46, M=8, and it runs three times per rebuild.
        //
        // The factored form is bit-for-bit identical, not merely equivalent:
        // `s_ni[a][j]` is accumulated over `a` in the same ascending order the
        // inner loop used, so every partial sum — and therefore every rounding —
        // matches the old code exactly. Only the redundant recomputation is gone.
        let emit_schur_products = |code: &mut String, s_mat: &str, s_ni_mat: &str, k_mat: &str| {
            if m == 0 {
                return;
            }
            code.push_str("            // S_NI = S * N_i\n");
            code.push_str("            for i in 0..N {\n");
            code.push_str("                for j in 0..M {\n");
            code.push_str("                    let mut sum = 0.0;\n");
            code.push_str(&format!(
                "                    for a in 0..N {{ sum += {cp}{s_mat}[i][a] * N_I[a][j]; }}\n"
            ));
            code.push_str(&format!(
                "                    {cp}{s_ni_mat}[i][j] = sum;\n"
            ));
            code.push_str("                }\n");
            code.push_str("            }\n");
            code.push_str("            // K = N_v * S_NI\n");
            code.push_str("            for i in 0..M {\n");
            code.push_str("                for j in 0..M {\n");
            code.push_str("                    let mut sum = 0.0;\n");
            code.push_str(&format!(
                "                    for a in 0..N {{ sum += N_V[i][a] * {cp}{s_ni_mat}[a][j]; }}\n"
            ));
            code.push_str(&format!("                    {cp}{k_mat}[i][j] = sum;\n"));
            code.push_str("                }\n");
            code.push_str("            }\n");
        };

        // Recompute Schur complement matrices: S = A^{-1}, S_NI = S*N_i, K = N_v*S_NI
        if schur_matrices_live {
            code.push_str("\n        // Recompute S = A^{-1} (trapezoidal)\n");
            code.push_str("        if let Some(inv) = invert_n(&self.a) {\n");
            code.push_str(&format!("            {}s = inv;\n", cp));
            emit_schur_products(&mut code, "s", "s_ni", "k");
            code.push_str("        }\n");

            // Recompute S_be = A_be^{-1}
            code.push_str("        // Recompute S_be = A_be^{-1} (backward Euler)\n");
            code.push_str("        if let Some(inv) = invert_n(&self.a_be) {\n");
            code.push_str(&format!("            {}s_be = inv;\n", cp));
            emit_schur_products(&mut code, "s_be", "s_ni_be", "k_be");
            code.push_str("        }\n");
        }

        // Invalidate cross-timestep chord LU — the A matrix changed
        if m > 0 {
            code.push_str("\n        // Invalidate chord LU cache (matrices changed)\n");
            code.push_str("        self.chord_valid = false;\n");
            if use_full_nodal && has_be_instance(ir) {
                code.push_str("        self.chord_be_valid = false;\n");
            }
        }

        // Invalidate the sub-sample-fire Schur-triple LRU — G/C changed, so every
        // cached (rate, be) triple is stale. Covers pot/switch/runtime-R (via
        // matrices_dirty), set_sample_rate, and the saturating-L resync rebuild.
        if ir.solver_config.subsample_fire {
            code.push_str(
                "\n        // Drop all sub-sample-fire Schur-triple LRU entries (matrices changed)\n",
            );
            code.push_str("        self.ssf_lru_len = 0;\n");
            code.push_str("        self.ssf_lru_evict = 0;\n");
        }

        code.push_str("    }\n\n");

        // set_pot_N() / set_runtime_R_<field>() methods — O(1) delta stamping
        // into A/A_be. Since A = G + alpha*C, changing G by delta_g means A
        // changes by delta_g at the same entries. The history matrices
        // (alpha*C) have no G term.
        //
        // Neither setter reseeds NR state. Callers that need a fresh NR
        // seed (preset recall, raw unsmoothed jumps) must follow with
        // `recompute_dc_op()`. On the nodal path `recompute_dc_op()` is a
        // stub (see `emit_recompute_dc_op_body_nodal`); nodal preset recall falls back
        // to WARMUP_SAMPLES_RECOMMENDED samples of NR catch-up.
        for (idx, pot) in ir.pots.iter().enumerate() {
            let np = pot.node_p;
            let nq = pot.node_q;
            let (setter_name, doc_noun) = match &pot.runtime_field {
                Some(field) => (
                    format!("set_runtime_R_{field}"),
                    format!("runtime resistor `{field}`"),
                ),
                None => (format!("set_pot_{idx}"), format!("potentiometer {idx}")),
            };
            let min_const = match &pot.runtime_field {
                Some(field) => format!("RUNTIME_R_{}_MIN", field.to_ascii_uppercase()),
                None => format!("POT_{idx}_MIN_R"),
            };
            let max_const = match &pot.runtime_field {
                Some(field) => format!("RUNTIME_R_{}_MAX", field.to_ascii_uppercase()),
                None => format!("POT_{idx}_MAX_R"),
            };
            if let Some(field) = &pot.runtime_field {
                // Emit a read-only accessor so the plugin can inspect current R.
                code.push_str(&format!(
                    "    /// Current resistance of runtime resistor `{field}` (ohms).\n\
                     \x20   #[inline]\n\
                     \x20   pub fn {field}(&self) -> f64 {{ self.pot_{idx}_resistance }}\n\n",
                ));
            }
            code.push_str(&format!(
                "    /// Set {doc_noun} (clamped to [{:.1}..{:.1}] ohms).\n\
                 \x20   ///\n\
                 \x20   /// Updates g_work and rebuilds all matrices (O(N^3)).\n\
                 \x20   /// Call per-block, not per-sample.\n",
                pot.min_resistance, pot.max_resistance
            ));
            code.push_str(&format!(
                "    pub fn {setter_name}(&mut self, resistance: f64) {{\n",
            ));
            code.push_str(&format!(
                "        if !resistance.is_finite() {{ return; }}\n\
                 \x20       let r = resistance.clamp({min_const}, {max_const});\n\
                 \x20       if (r - self.pot_{}_resistance).abs() < 1e-12 {{ return; }}\n\n\
                 \x20       // Delta conductance: stamp into A and A_be (the history matrices\n\
                 \x20       // A_neg = alpha*C and A_neg_be = C/T carry no G term)\n\
                 \x20       let delta_g = 1.0 / r - 1.0 / self.pot_{}_resistance;\n",
                idx, idx
            ));

            // Emit conductance stamp into g_work (full dimension if Boyle, reduced otherwise)
            // Pot node indices (np, nq) are 1-indexed MNA nodes (< n_nodes), same in both systems.
            let gw = if use_full_nodal {
                "self.cold.g_work"
            } else {
                "self.g_work"
            };
            // Record the 2x2 block this two-terminal stamp can touch. Same
            // (np, nq) the closures below format, so the record cannot drift
            // from what is emitted.
            for (a, b) in [(np, np), (nq, nq), (np, nq), (nq, np)] {
                if a > 0 && b > 0 {
                    setter_stamps.insert((a - 1, b - 1));
                }
            }
            let emit_g_work_stamp = |code: &mut String| {
                if np > 0 {
                    code.push_str(&format!(
                        "        {}[{}][{}] += delta_g;\n",
                        gw,
                        np - 1,
                        np - 1
                    ));
                }
                if nq > 0 {
                    code.push_str(&format!(
                        "        {}[{}][{}] += delta_g;\n",
                        gw,
                        nq - 1,
                        nq - 1
                    ));
                }
                if np > 0 && nq > 0 {
                    code.push_str(&format!(
                        "        {}[{}][{}] -= delta_g;\n\
                         \x20       {}[{}][{}] -= delta_g;\n",
                        gw,
                        np - 1,
                        nq - 1,
                        gw,
                        nq - 1,
                        np - 1
                    ));
                }
            };

            {
                // Delta stamp A and A_be directly (fast path)
                let emit_delta_stamp = |code: &mut String, matrix: &str, sign: &str| {
                    if np > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] {sign}= delta_g;\n",
                            np - 1,
                            np - 1
                        ));
                    }
                    if nq > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] {sign}= delta_g;\n",
                            nq - 1,
                            nq - 1
                        ));
                    }
                    if np > 0 && nq > 0 {
                        let neg_sign = if sign == "+" { "-" } else { "+" };
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] {neg_sign}= delta_g;\n\
                             \x20       self.{matrix}[{}][{}] {neg_sign}= delta_g;\n",
                            np - 1,
                            nq - 1,
                            nq - 1,
                            np - 1
                        ));
                    }
                };

                emit_delta_stamp(&mut code, "a", "+");
                emit_delta_stamp(&mut code, "a_be", "+");
                // The history matrices (A_neg = alpha*C, A_neg_be = alpha_be*C)
                // carry no G term: a pot does not touch them.

                // Also update g_work for consistency
                if has_pots || has_switches {
                    code.push_str(
                        "\n        // Update working G for sample rate rebuild consistency\n",
                    );
                    emit_g_work_stamp(&mut code);
                }
            }

            code.push_str(&format!("        self.pot_{}_resistance = r;\n", idx));
            code.push_str("        self.matrices_dirty = true;\n");
            // A pot changes a conductance only, which the charge-form history
            // (q_dot = C·dx/dt, no G term) does not carry: no breakpoint-BE.

            // Authentic-noise coefficient refresh (Step 2 / Phase 1.5).
            // Keeps `state.noise_thermal_sqrt_inv_r[k]` in sync with the
            // live pot / runtime-R value so the per-sample Johnson-Nyquist
            // variance tracks the knob. Emitted only when noise is enabled
            // at codegen AND this pot backs a thermal source.
            if noise.enabled {
                if let Some(slot) = noise.pot_to_noise_slot.get(idx).copied().flatten() {
                    code.push_str(&format!(
                        "        self.noise_thermal_sqrt_inv_r[{slot}] = (1.0 / r).sqrt();\n",
                    ));
                }
                if let Some(slot) = noise.pot_to_r_flicker_slot.get(idx).copied().flatten() {
                    code.push_str(&format!(
                        "        self.noise_r_flicker_inv_r[{slot}] = 1.0 / r;\n",
                    ));
                }
                // Phase 4: op-amp en_g_diag absolute recompute for pots at an
                // op-amp in+ (shared refresh_opamp_en_g_diag from noise.methods).
                if noise
                    .pot_to_opamp_en_refresh
                    .get(idx)
                    .is_some_and(|v| !v.is_empty())
                {
                    code.push_str("        self.refresh_opamp_en_g_diag();\n");
                }
            }

            // No NR-state reseed: callers that need one (preset recall,
            // raw unsmoothed jumps) should follow with `recompute_dc_op()`.
            // Nodal circuits route through the stub body — falls back to
            // WARMUP_SAMPLES_RECOMMENDED if a full nodal recompute is ever
            // needed (see `emit_recompute_dc_op_body_nodal`).
            code.push_str("    }\n\n");
        }

        // set_switch_N() methods
        for (idx, sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!(
                "    /// Set switch {} position (0-indexed, {} positions).\n\
                 \x20   ///\n\
                 \x20   /// A switch flip is a topology step — follow with `recompute_dc_op()`\n\
                 \x20   /// on DK circuits to refresh the NR seed. On nodal (stub recompute)\n\
                 \x20   /// NR re-converges essentially immediately (typically the next\n\
                 \x20   /// sample, even across a real topology change). Any residual DC\n\
                 \x20   /// settle is bounded ABOVE by WARMUP_SAMPLES_RECOMMENDED, but that\n\
                 \x20   /// is a worst-case FULL-DC-settle bound (max node RC, including\n\
                 \x20   /// high-impedance nodes off the audio path) — the AUDIBLE settle is\n\
                 \x20   /// usually far shorter (often ~0 ms). Measure the transient on the\n\
                 \x20   /// output; do NOT mute or size a fade for the full WARMUP bound.\n",
                idx, sw.num_positions
            ));
            code.push_str(&format!(
                "    pub fn set_switch_{}(&mut self, position: usize) {{\n\
                 \x20       if position >= SWITCH_{}_NUM_POSITIONS {{ return; }}\n\
                 \x20       if position == self.switch_{}_position {{ return; }}\n\n",
                idx, idx, idx
            ));

            for (ci, comp) in sw.components.iter().enumerate() {
                let np = comp.node_p;
                let nq = comp.node_q;
                let matrix = if comp.component_type == 'R' {
                    if use_full_nodal {
                        "cold.g_work"
                    } else {
                        "g_work"
                    }
                } else if use_full_nodal {
                    "cold.c_work"
                } else {
                    "c_work"
                };

                code.push_str(&format!(
                    "        // Switch {} component {} ({}, type {})\n",
                    idx, ci, comp.name, comp.component_type
                ));
                code.push_str(&format!(
                    "        let old_val_{} = SWITCH_{}_COMP_{}_VALUES[self.switch_{}_position];\n\
                     \x20       let new_val_{} = SWITCH_{}_COMP_{}_VALUES[position];\n",
                    ci, idx, ci, idx, ci, idx, ci
                ));

                if comp.component_type == 'R' {
                    // Resistor: unstamp old conductance, stamp new
                    code.push_str(&format!(
                        "        let g_old_{ci} = 1.0 / old_val_{ci};\n\
                         \x20       let g_new_{ci} = 1.0 / new_val_{ci};\n\
                         \x20       let delta_{ci} = g_new_{ci} - g_old_{ci};\n"
                    ));
                } else {
                    // Capacitor or Inductor in C matrix: delta is new - old directly
                    code.push_str(&format!(
                        "        let delta_{ci} = new_val_{ci} - old_val_{ci};\n"
                    ));
                }

                // Record every position this switch component can touch:
                // the augmented-row diagonal for an L in augmented MNA, else
                // the 2x2 block on its two circuit nodes.
                if let Some(aug_row) = comp.augmented_row.filter(|_| comp.component_type == 'L') {
                    setter_stamps.insert((aug_row, aug_row));
                } else {
                    for (a, b) in [(np, np), (nq, nq), (np, nq), (nq, np)] {
                        if a > 0 && b > 0 {
                            setter_stamps.insert((a - 1, b - 1));
                        }
                    }
                }

                // Stamp delta into g_work or c_work
                if let Some(aug_row) = comp.augmented_row.filter(|_| comp.component_type == 'L') {
                    // Augmented MNA: L value on diagonal of branch variable row
                    let cw = if use_full_nodal {
                        "self.cold.c_work"
                    } else {
                        "self.c_work"
                    };
                    code.push_str(&format!(
                        "        {cw}[{aug_row}][{aug_row}] += delta_{ci};\n"
                    ));
                } else {
                    // R or C: conductance stamp at circuit nodes
                    if np > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] += delta_{ci};\n",
                            np - 1,
                            np - 1
                        ));
                    }
                    if nq > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] += delta_{ci};\n",
                            nq - 1,
                            nq - 1
                        ));
                    }
                    if np > 0 && nq > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] -= delta_{ci};\n\
                             \x20       self.{matrix}[{}][{}] -= delta_{ci};\n",
                            np - 1,
                            nq - 1,
                            nq - 1,
                            np - 1
                        ));
                    }
                }
                code.push('\n');
            }

            // Recompute off-diagonal mutual inductance entries
            if !sw.mutual_entries.is_empty() {
                code.push_str("        // Update mutual inductance off-diagonal entries\n");
                let cw_m = if use_full_nodal {
                    "self.cold.c_work"
                } else {
                    "self.c_work"
                };
                for me in &sw.mutual_entries {
                    setter_stamps.insert((me.row_a, me.row_b));
                    setter_stamps.insert((me.row_b, me.row_a));
                    setter_stamps.insert((me.row_a, me.row_a));
                    setter_stamps.insert((me.row_b, me.row_b));
                    code.push_str(&format!(
                        "        {{\n\
                         \x20           let m = {:.17e}_f64 * ({cw_m}[{}][{}] * {cw_m}[{}][{}]).sqrt();\n\
                         \x20           {cw_m}[{}][{}] = m;\n\
                         \x20           {cw_m}[{}][{}] = m;\n\
                         \x20       }}\n",
                        me.coupling,
                        me.row_a, me.row_a,
                        me.row_b, me.row_b,
                        me.row_a, me.row_b,
                        me.row_b, me.row_a,
                    ));
                }
                code.push('\n');
            }

            code.push_str(&format!(
                "        self.switch_{}_position = position;\n",
                idx
            ));
            code.push_str("        self.matrices_dirty = true;\n");
            // A switch that swaps a capacitor or an inductor leaves q_dot built
            // on the old value: arm breakpoint-BE so the swap sample is solved on
            // the backward-Euler matrices, which re-seed q_dot from their own
            // capacitor currents. A resistor-only switch is a conductance change
            // the charge-form history does not carry: no breakpoint.
            if ir.solver_config.breakpoint_be
                && sw
                    .components
                    .iter()
                    .any(|c| matches!(c.component_type, 'C' | 'L'))
            {
                code.push_str("        self.breakpoint_be = BREAKPOINT_BE_SAMPLES;\n");
            }

            // Authentic-noise coefficient refresh for any R-type components
            // in this switch that back a thermal noise source. Keeps the
            // live `noise_thermal_sqrt_inv_r` in sync with the freshly-
            // selected R for the next sample.
            if noise.enabled {
                if let Some(slots) = noise.switch_comp_to_noise_slot.get(idx) {
                    for (ci, maybe_slot) in slots.iter().enumerate() {
                        if let Some(slot) = maybe_slot {
                            code.push_str(&format!(
                                "        self.noise_thermal_sqrt_inv_r[{slot}] = (1.0 / SWITCH_{idx}_COMP_{ci}_VALUES[position]).sqrt();\n",
                            ));
                        }
                    }
                }
                if let Some(slots) = noise.switch_comp_to_r_flicker_slot.get(idx) {
                    for (ci, maybe_slot) in slots.iter().enumerate() {
                        if let Some(slot) = maybe_slot {
                            code.push_str(&format!(
                                "        self.noise_r_flicker_inv_r[{slot}] = 1.0 / SWITCH_{idx}_COMP_{ci}_VALUES[position];\n",
                            ));
                        }
                    }
                }
                // Phase 4: switch R at an op-amp in+ → en_g_diag recompute.
                if noise
                    .switch_to_opamp_en_refresh
                    .get(idx)
                    .copied()
                    .unwrap_or(false)
                {
                    code.push_str("        self.refresh_opamp_en_g_diag();\n");
                }
            }

            // No NR-state reseed: a switch flip is a topology step, but
            // reseeding `v_prev` mid-signal snaps the trajectory to DC_OP
            // and is audible as a click. Callers should follow with
            // `recompute_dc_op()` on DK circuits, or accept NR catch-up
            // on nodal circuits (stub) over WARMUP_SAMPLES_RECOMMENDED.

            code.push_str("    }\n\n");
        }

        // Authentic circuit noise — Phase 1 public API
        // (set_noise_enabled / set_noise_gain / set_thermal_gain /
        //  set_temperature_k / set_seed). Empty when noise mode is Off.
        if noise.enabled {
            code.push_str(&noise.methods);
        }

        code.push_str("}\n\n");

        code
    }
}
