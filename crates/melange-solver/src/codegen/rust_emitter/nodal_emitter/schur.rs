//! Nodal-Schur `process_sample`, its solve, Newton, warm start and rail handling.

use super::reset::emit_nodal_nan_reset;
use super::sites::{
    emit_history_rhs, emit_q_dot_commit, emit_q_dot_locals, NoiseMode, PinSite, SchurSite,
};
use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::dk_emitter::{emit_noise_replay_body, NoiseEmission};
use crate::codegen::rust_emitter::helpers::{
    body_effect_jacobian_term, carries_q_dot, emit_body_effect_at_iterate, emit_glow_lit_be_hold,
    emit_pentode_nr_dk_stamp, emit_stateful_update, has_latched_device, section_banner,
    stateful_device_data,
};
use crate::codegen::rust_emitter::nr_helpers::{
    emit_nr_singular_fallback, emit_schur_nr_limit_and_converge,
};
use crate::codegen::rust_emitter::RustEmitter;
use crate::codegen::CodegenError;

impl RustEmitter {
    /// Emit process_sample using Schur complement: precomputed S = A^{-1}, M-dim NR.
    ///
    /// This replaces the old O(N^3)-per-iteration LU solve with:
    /// 1. Build RHS (same as before)
    /// 2. Linear prediction: v_pred = S * rhs (O(N^2))
    /// 3. Extract device voltages: p = N_v * v_pred (O(M*N))
    /// 4. M-dim NR (same as DK: O(M^3) per iteration)
    /// 5. Recover full v = v_pred + S_NI * i_nl (O(M*N))
    /// Emit the runtime BE-latch Nyquist-cycle detector (shared by the Schur
    /// and full-LU nodal process loops).
    ///
    /// Maintains a normalized lag-1 autocorrelation of both the primary output
    /// and the drive input via EMAs (`*_r1_num` ≈ E[x·x₋₁], `*_pow` ≈ E[x²]).
    /// A self-sustaining Nyquist `(-1)^n` limit cycle — the trapezoidal artifact
    /// a *large-signal* operating point can reach even when the compile-time
    /// quiescent-OP spectral-radius analysis found trap stable — drives the
    /// output `r1 = num/pow → −1`.
    ///
    /// The load-bearing discriminator is **input-awareness**: a limit cycle is
    /// self-generated, so it persists regardless of the drive, whereas
    /// near-Nyquist output that merely tracks a *bright input* (a legitimate
    /// high-frequency sine has `r1 = cos(ω)`, e.g. −0.87 at 20 kHz / 48 k) is
    /// not a pathology. We latch only when the output is strongly anti-
    /// correlated AND the input is not (or the input is silent). Without this
    /// gate a bright sweep tone near Nyquist would false-trip the net and
    /// needlessly derate the circuit to backward Euler (observed on
    /// sus-bus/sweep at 15.8 kHz during golden-audio regression).
    ///
    /// On latch, the process loop forces the L-stable BE fallback for the rest
    /// of the stream (sticky until `reset()`); BE damps the cycle and the
    /// circuit's own bias/servo state then relaxes on its RC.
    ///
    /// Emitted only when `runtime_be_latch` (a genuine trapezoidal build, not
    /// force-trap). The EMA coefficient is derived from the live sample rate so
    /// the detector time constant is fs-invariant.
    pub(super) fn emit_be_latch_detector(code: &mut String, ir: &CircuitIR, indent: &str) {
        if !ir.solver_config.runtime_be_latch {
            return;
        }
        // The running factor and the latch reference at it (per factor in a
        // runtime-oversampling build).
        let fx = super::super::runtime_os::factor_expr(ir, "state");
        let gain =
            super::super::runtime_os::baked(ir, "BE_LATCH_PASSBAND_GAIN", "state.oversampling");
        let cost =
            super::super::runtime_os::baked(ir, "BE_LATCH_BE_COST_REL", "state.oversampling");
        code.push_str(&format!(
            "{indent}// Runtime BE-latch. Track the lag-1 ratio of the mean-removed output over\n\
             {indent}// the estimator window; for one mode x = A*z^n it equals z, for a mixture\n\
             {indent}// it is the power-weighted mean of the components' factors. Engage at\n\
             {indent}// ratio <= -exp(-alpha): the output is an alternating mode that outlives\n\
             {indent}// the window and dominates it, and the input is not itself one. That is a\n\
             {indent}// stiff mode trapezoidal keeps ringing. A decaying transient tail, or a\n\
             {indent}// small ring under program, does not qualify. Once engaged, the L-stable\n\
             {indent}// BE path runs for the rest of the stream (cleared by reset()).\n\
             {indent}if !state.be_latched {{\n\
             {indent}    let be_ema = (1.0 / (BE_LATCH_TAU_S * state.current_sample_rate * {fx} as f64)).clamp(1e-4, 0.5);\n\
             {indent}    let be_x = v[OUTPUT_NODES[0]];\n\
             {indent}    let be_x = if be_x.is_finite() {{ be_x }} else {{ 0.0 }};\n\
             {indent}    state.be_x_mean += be_ema * (be_x - state.be_x_mean);\n\
             {indent}    let be_x = be_x - state.be_x_mean;\n\
             {indent}    state.be_r1_num += be_ema * (be_x * state.be_x_prev - state.be_r1_num);\n\
             {indent}    state.be_pow += be_ema * (be_x * be_x - state.be_pow);\n\
             {indent}    state.be_x_prev = be_x;\n\
             {indent}    let be_u = if input.is_finite() {{ input }} else {{ 0.0 }};\n\
             {indent}    // Program reference: the program the output actually carries. On the\n\
             {indent}    // ring predicate's scale that is passband gain x input amplitude, but a\n\
             {indent}    // circuit that clips never delivers that linear extrapolation, so it is\n\
             {indent}    // bounded by the output's own excursion from its operating point. Each\n\
             {indent}    // is remembered as long as this circuit's slowest ring. (A ring sits in\n\
             {indent}    // the output envelope too, but where it matters it is small against the\n\
             {indent}    // program.)\n\
             {indent}    state.be_ref_in = ({gain} * be_u.abs()).max(state.be_ref_in * state.be_ref_decay);\n\
             {indent}    state.be_env = (v[OUTPUT_NODES[0]] - state.dc_operating_point[OUTPUT_NODES[0]]).abs().max(state.be_env * state.be_ref_decay);\n\
             {indent}    state.be_ref = state.be_ref_in.min(state.be_env);\n\
             {indent}    state.be_in_x_mean += be_ema * (be_u - state.be_in_x_mean);\n\
             {indent}    let be_u = be_u - state.be_in_x_mean;\n\
             {indent}    state.be_in_r1_num += be_ema * (be_u * state.be_in_x_prev - state.be_in_r1_num);\n\
             {indent}    state.be_in_pow += be_ema * (be_u * be_u - state.be_in_pow);\n\
             {indent}    state.be_in_x_prev = be_u;\n\
             {indent}    // Cross-multiplied (the powers are >= 0): r1_num/pow <= -exp(-alpha).\n\
             {indent}    let be_enter = -(-be_ema).exp();\n\
             {indent}    // An alternation inside the solver's own node tolerance (RELTOL*|v| +\n\
             {indent}    // VNTOL, the Newton node-step test) cannot be told apart from\n\
             {indent}    // convergence noise, so it is not evidence.\n\
             {indent}    let be_tol = 1e-3 * state.be_x_mean.abs() + 1e-6;\n\
             {indent}    // A ring below -60 dB of the program that excited it is one the\n\
             {indent}    // compile-time ring predicate left on trapezoidal: not evidence either.\n\
             {indent}    // (An alternation of amplitude A has power A^2.)\n\
             {indent}    // Nor is one quieter than backward Euler's own in-band damage: the\n\
             {indent}    // compile-time choice keeps trapezoidal there, and so does the latch.\n\
             {indent}    let be_floor = f64::max(be_tol, BE_LATCH_RING_REL.max({cost}) * state.be_ref);\n\
             {indent}    let out_ring = state.be_pow > be_floor * be_floor\n\
             {indent}        && state.be_r1_num <= be_enter * state.be_pow;\n\
             {indent}    let in_ring = state.be_in_pow > BE_LATCH_POWER_FLOOR\n\
             {indent}        && state.be_in_r1_num <= be_enter * state.be_in_pow;\n\
             {indent}    if out_ring && !in_ring {{\n\
             {indent}        state.be_latched = true;\n\
             {indent}        state.diag_be_latch_count += 1;\n\
             {indent}    }}\n\
             {indent}}}\n"
        ));
    }

    pub(super) fn emit_nodal_schur_process_sample(
        ir: &CircuitIR,
        noise: &NoiseEmission,
        setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
    ) -> Result<String, CodegenError> {
        let m = ir.topology.m;
        let os_factor = ir.solver_config.oversampling_factor;
        let has_pots = !ir.pots.is_empty();
        // Multi-input ports (M=0 only): gates every input-related emission
        // divergence so single-input output stays byte-identical. Multi-input is
        // rejected at the CLI unless M==0 and oversampling==1, so the sub-step /
        // BE-fallback blocks below (all M>0) never coexist with `multi_input`,
        // but they are still gated defensively. See multi-input-ports-plan.md.
        let multi_input = ir.solver_config.num_inputs() > 1;
        // `.inject`/`.tap`: when present, the inner solve gains an `injections`
        // param and returns raw `.tap` node voltages; the public entry becomes
        // the per-inner-sample array API (emit_oversampler / emit_inject_wrapper_1x).
        // inject_or_tap and multi_input are mutually exclusive (CLI-rejected).
        let inject_or_tap = ir.solver_config.has_inject_or_tap();
        let inner_sig = if inject_or_tap {
            ", injections: [f64; NUM_INJECT]"
        } else {
            ""
        };
        let inner_ret = if inject_or_tap {
            "([f64; NUM_OUTPUTS], [f64; NUM_TAP])"
        } else {
            "[f64; NUM_OUTPUTS]"
        };

        let mut code = section_banner(
            "PROCESS SAMPLE (Schur complement: M-dim NR via precomputed S = A^{-1})",
        );

        // Function signature
        if os_factor > 1 || inject_or_tap || super::super::runtime_os::runtime(ir).is_some() {
            code.push_str("/// Process a single sample at the internal (oversampled) rate.\n");
            code.push_str("///\n");
            code.push_str("/// Called by `process_sample()` through the oversampling chain.\n");
            code.push_str("#[inline(always)]\n");
            code.push_str(&format!(
                "fn process_sample_inner(input: f64{inner_sig}, state: &mut CircuitState) -> {inner_ret} {{\n"
            ));
        } else {
            code.push_str("/// Process a single audio sample through the circuit.\n");
            code.push_str("///\n");
            code.push_str(
                "/// Uses Schur complement NR: precomputes S = A^{-1}, iterates in M-space.\n",
            );
            code.push_str("/// Cost: O(N^2) linear prediction + O(M^3) per NR iteration.\n");
            code.push_str("#[inline]\n");
            if multi_input {
                code.push_str("pub fn process_sample(inputs: [f64; NUM_INPUTS], state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n");
            } else {
                code.push_str("pub fn process_sample(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n");
            }
        }

        // Input sanitization
        if multi_input {
            code.push_str(
                "    let mut inputs = inputs;\n    for v in inputs.iter_mut() { *v = if !v.is_finite() { state.diag_input_nan_count += 1; 0.0 } else if v.abs() > INPUT_LIMIT_V { state.diag_input_clamp_count += 1; v.clamp(-INPUT_LIMIT_V, INPUT_LIMIT_V) } else { *v }; }\n\n",
            );
        } else {
            code.push_str(
                "    let input = if !input.is_finite() { state.diag_input_nan_count += 1; 0.0 } else if input.abs() > INPUT_LIMIT_V { state.diag_input_clamp_count += 1; input.clamp(-INPUT_LIMIT_V, INPUT_LIMIT_V) } else { input };\n\n",
            );
        }
        if inject_or_tap {
            code.push_str(
                "    // Sanitize injections (NaN/Inf → 0). No magnitude clamp: a feedback value\n\
                 \x20   // is arbitrary and the plausibility guard catches any runaway.\n\
                 \x20   let mut injections = injections;\n\
                 \x20   for v in injections.iter_mut() { *v = if v.is_finite() { *v } else { state.diag_runtime_nan_count += 1; 0.0 }; }\n\n",
            );
        }
        // `.runtime V` fields are sanitised once per HOST sample: here only when
        // this function is the host entry (fixed 1×, no `.inject`); every other
        // build's wrapper or dispatcher does it before calling the inner function.
        if os_factor == 1 && !inject_or_tap && super::super::runtime_os::runtime(ir).is_none() {
            code.push_str(&super::super::runtime_inputs::sanitize_block(ir, "    "));
        }

        // Saturating inductors force the full-LU sub-path (their flux device
        // lives in its NR loop), and every other saturating element is refused
        // while the MNA is built — so the Schur path never sees saturation.
        assert!(
            ir.saturating_inductors.is_empty(),
            "Schur process_sample emitted for a circuit with saturating inductors; \
             saturation must route to full-LU"
        );

        // Lazy rebuild: process all pot/switch changes in one batch
        let has_rebuild = has_pots || !ir.switches.is_empty();
        if has_rebuild {
            code.push_str(
                "    // Lazy rebuild: batch all pot/switch changes into one matrix rebuild\n\
                 \x20   if state.matrices_dirty {\n\
                 \x20       state.rebuild_matrices(state.current_sample_rate * OVERSAMPLING_FACTOR as f64);\n\
                 \x20       state.matrices_dirty = false;\n\
                 \x20   }\n\n",
            );
        }

        // Flush denormals in state vectors (prevents 50-100x CPU penalty during silence)
        code.push_str("    for v in state.v_prev.iter_mut() { *v = *v + 1e-25 - 1e-25; }\n");
        if carries_q_dot(ir) {
            code.push_str("    for v in state.q_dot.iter_mut() { *v = *v + 1e-25 - 1e-25; }\n");
        }
        if m > 0 {
            code.push_str("    for v in state.i_nl_prev.iter_mut() { *v = *v + 1e-25 - 1e-25; }\n");
        }
        code.push('\n');

        emit_q_dot_locals(
            &mut code,
            ir,
            "    ",
            ir.topology.m > 0 || ir.solver_config.breakpoint_be,
        );
        Self::emit_schur_solve(&mut code, ir, noise, setter_stamps)?;

        // NaN/Inf recovery: shared reset + DC-OP return. Schur path has no
        // cross-timestep chord LU to invalidate, so is_full_lu = false.
        emit_nodal_nan_reset(&mut code, ir, "    ", false, noise);

        // Op-amp slew-rate limiting (nodal Schur path). Clamp the per-sample
        // voltage delta at each op-amp output node to ±SR*dt. This is
        // mathematically equivalent to clamping the Boyle dominant-pole
        // integrator input current to ±I_slew = ±SR*C_dom: the per-sample
        // voltage step of an integrator with current `i_in` through cap
        // `C_dom` is `Δv = (i_in*dt)/C_dom`, so capping `|Δv| ≤ SR*dt`
        // caps `|i_in| ≤ SR*C_dom`. Emitted only for op-amps with finite
        // SR; circuits without `SR=` in the .model generate identical code.
        Self::emit_opamp_slew_limit(&mut code, ir, "    ", "v");

        // Every Newton path failed: keep the previous state (shared with full-LU).
        if m > 0 {
            Self::emit_death_spiral_hold(&mut code, ir, false);
        }

        // Runtime BE-latch detector (updates state.be_latched for next sample).
        Self::emit_be_latch_detector(&mut code, ir, "    ");

        // Stateful-device (Phase 0c) after-solve update — BEFORE state.v_prev = v
        // so v_prev holds the prior sample. Shared with the DK path.
        //
        // With sub-sample fire active (nodal-Schur, latched device, flag not
        // off) the update is folded into the breakpoint re-solve block, which
        // may replace `v`/`i_nl` with the end of the dark/lit sub-step pair.
        if ir.solver_config.subsample_fire && m > 0 {
            code.push_str(&super::super::subsample_fire::emit_subsample_fire_block(
                ir, noise,
            )?);
        } else {
            code.push_str(&emit_stateful_update(&stateful_device_data(ir)));
        }

        // State update
        code.push_str("    // State update\n");
        emit_q_dot_commit(
            &mut code,
            ir,
            "    ",
            ir.topology.m > 0 || ir.solver_config.breakpoint_be,
        );
        code.push_str("    state.v_prev = v;\n");
        // Breakpoint-BE countdown: this sample was solved on the BE matrices
        // (both the m=0 override above and the m>0 BE fallback, forced via the
        // `converged` guard). One decrement per sample, after the solve.
        if ir.solver_config.breakpoint_be {
            code.push_str("    if state.breakpoint_be > 0 { state.breakpoint_be -= 1; }\n");
        }
        // Glow lit-hold re-arm (after the decrement; empty for non-glow).
        code.push_str(&emit_glow_lit_be_hold(ir));
        // Commit input_prev here (NOT at the RHS build) so the sub-step input
        // interpolation earlier in the sample still sees last sample's value.
        if multi_input {
            code.push_str("    state.inputs_prev = inputs;\n");
        } else {
            code.push_str("    state.input_prev = input;\n");
        }
        if inject_or_tap {
            code.push_str("    state.injections_prev = injections;\n");
        }
        if m > 0 {
            code.push_str("    state.i_nl_prev_prev = state.i_nl_prev;\n");
            code.push_str("    state.i_nl_prev = i_nl;\n");
        }
        for (idx, _pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "    state.pot_{}_resistance_prev = state.pot_{}_resistance;\n",
                idx, idx
            ));
        }
        code.push('\n');

        // Device self-heating thermal update (BJT, diode, triode) — shared
        // exact-exponential emitter (see emit_self_heating_thermal_updates).
        Self::emit_self_heating_thermal_updates(&mut code, ir);

        // (NaN check already done before state update)

        code.push_str(&super::super::helpers::emit_region_exit_lines(ir, "    "));

        // Output extraction
        code.push_str("    // Extract outputs, DC blocking, and scaling\n");
        code.push_str("    let mut output = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let raw_out = v[OUTPUT_NODES[out_idx]];\n");
        code.push_str("        let raw_out = if raw_out.is_finite() { raw_out } else { 0.0 };\n");
        if ir.dc_block {
            code.push_str("        let dc_blocked = raw_out - state.dc_block_x_prev[out_idx]\n");
            code.push_str("            + state.dc_block_r * state.dc_block_y_prev[out_idx];\n");
            code.push_str("        state.dc_block_x_prev[out_idx] = raw_out;\n");
            // Denormal bias on the blocker feedback (matches the DK template):
            // keeps y_prev out of the denormal range during long silences.
            code.push_str("        state.dc_block_y_prev[out_idx] = dc_blocked + 1e-20;\n");
            code.push_str("        let scaled = dc_blocked * OUTPUT_SCALES[out_idx];\n");
        } else {
            code.push_str("        let scaled = raw_out * OUTPUT_SCALES[out_idx];\n");
        }
        code.push_str("        let abs_out = scaled.abs();\n");
        code.push_str(
            "        if abs_out > state.diag_peak_output { state.diag_peak_output = abs_out; }\n",
        );
        if ir.dc_block {
            let clamp_v = ir.solver_config.output_clamp_v;
            code.push_str(&format!(
                "        if abs_out > {clamp_v:e} {{ state.diag_clamp_count += 1; }}\n"
            ));
            code.push_str(&format!(
                "        output[out_idx] = scaled.clamp(-{clamp_v:e}, {clamp_v:e});\n"
            ));
        } else {
            code.push_str(
                "        output[out_idx] = if scaled.is_finite() { scaled } else { 0.0 };\n",
            );
        }
        code.push_str("    }\n");
        if inject_or_tap {
            // Raw inner-rate taps: the finalized node voltages, WITHOUT the
            // output pipeline (no DC-block, scale, clamp, decimation). Same
            // `v` the outputs are read from, before the wrapper decimates.
            code.push_str(
                "    let mut taps = [0.0f64; NUM_TAP];\n\
                 \x20   for t in 0..NUM_TAP {\n\
                 \x20       let raw = v[TAP_NODES[t]];\n\
                 \x20       taps[t] = if raw.is_finite() { raw } else { 0.0 };\n\
                 \x20   }\n\
                 \x20   (output, taps)\n",
            );
        } else {
            code.push_str("    output\n");
        }
        code.push_str("}\n\n");

        Ok(code)
    }

    /// Emit DK-style device evaluation for a single device in Schur NR (default indent "        ").
    fn emit_dk_device_eval_for_nodal_schur(
        code: &mut String,
        dev_num: usize,
        slot: &crate::codegen::ir::DeviceSlot,
    ) -> Result<(), CodegenError> {
        Self::emit_dk_device_eval_for_nodal_schur_indented(code, dev_num, slot, "        ")
    }

    /// Emit DK-style device evaluation for a single device at given indent.
    ///
    /// Declares `i_dev{s}` and `jdev_{i}_{j}` local variables matching the
    /// DK `solve_nonlinear` naming convention. Uses `v_d{s}` from the caller.
    pub(crate) fn emit_dk_device_eval_for_nodal_schur_indented(
        code: &mut String,
        dev_num: usize,
        slot: &crate::codegen::ir::DeviceSlot,
        indent: &str,
    ) -> Result<(), CodegenError> {
        use crate::codegen::ir::{DeviceParams, DeviceType};
        let s = slot.start_idx;
        let d = dev_num;

        match (&slot.device_type, &slot.params) {
            (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                if dp.has_rs() && dp.has_bv() {
                    code.push_str(&format!(
                        "{indent}let (i_rs{s}, g_rs{s}) = diode_eval_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS);\n\
                         {indent}let i_dev{s} = i_rs{s} + diode_breakdown_current(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n\
                         {indent}let jdev_{s}_{s} = g_rs{s} + diode_breakdown_conductance(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n"
                    ));
                } else if dp.has_rs() {
                    code.push_str(&format!(
                        "{indent}let (i_dev{s}, jdev_{s}_{s}) = diode_eval_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS);\n"
                    ));
                } else if dp.has_bv() {
                    code.push_str(&format!(
                        "{indent}let i_dev{s} = diode_current(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt) + diode_breakdown_current(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n\
                         {indent}let jdev_{s}_{s} = diode_conductance(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt) + diode_breakdown_conductance(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n"
                    ));
                } else {
                    code.push_str(&format!(
                        "{indent}let i_dev{s} = diode_current(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt);\n\
                         {indent}let jdev_{s}_{s} = diode_conductance(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt);\n"
                    ));
                }
            }
            (DeviceType::Bjt, DeviceParams::Bjt(bp)) => {
                let s1 = s + 1;
                if bp.has_parasitics() && !slot.has_internal_mna_nodes {
                    code.push_str(&format!(
                        "{indent}let (i_dev{s}, i_dev{s1}, bjt{d}_jac) = bjt_with_parasitics(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, DEVICE_{d}_NR, state.device_{d}_bf, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF, DEVICE_{d}_IKR, DEVICE_{d}_ISE, DEVICE_{d}_NE, DEVICE_{d}_ISC, DEVICE_{d}_NC, DEVICE_{d}_RB, DEVICE_{d}_RC, DEVICE_{d}_RE);\n"
                    ));
                } else {
                    // Combined evaluation: shared exp() across ic, ib, jacobian
                    // (parasitics handled by MNA internal nodes when has_internal_mna_nodes)
                    code.push_str(&format!(
                        "{indent}let (i_dev{s}, i_dev{s1}, bjt{d}_jac) = bjt_evaluate(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, DEVICE_{d}_NR, state.device_{d}_bf, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF, DEVICE_{d}_IKR, DEVICE_{d}_ISE, DEVICE_{d}_NE, DEVICE_{d}_ISC, DEVICE_{d}_NC);\n"
                    ));
                }
                code.push_str(&format!(
                    "{indent}let jdev_{s}_{s} = bjt{d}_jac[0];\n\
                     {indent}let jdev_{s}_{s1} = bjt{d}_jac[1];\n\
                     {indent}let jdev_{s1}_{s} = bjt{d}_jac[2];\n\
                     {indent}let jdev_{s1}_{s1} = bjt{d}_jac[3];\n"
                ));
            }
            (DeviceType::BjtForwardActive, DeviceParams::Bjt(_bp)) => {
                code.push_str(&format!(
                    "{indent}let vbe_{d} = v_d{s} * DEVICE_{d}_SIGN;\n\
                     {indent}let (exp_be_{d}, dexp_be_{d}) = junction_exp(vbe_{d} / (DEVICE_{d}_NF * state.device_{d}_vt), state.device_{d}_is);\n\
                     {indent}let i_dev{s} = state.device_{d}_is * (exp_be_{d} - 1.0) * DEVICE_{d}_SIGN;\n\
                     {indent}let jdev_{s}_{s} = state.device_{d}_is / (DEVICE_{d}_NF * state.device_{d}_vt) * dexp_be_{d};\n"
                ));
            }
            (DeviceType::Jfet, DeviceParams::Jfet(jp)) => {
                let s1 = s + 1;
                let call = super::super::nr_helpers::jfet_evaluate_call(
                    jp,
                    d,
                    &format!("v_d{s1}"),
                    &format!("v_d{s}"),
                    &format!("DEVICE_{d}_SIGN"),
                );
                code.push_str(&format!(
                    "{indent}let (i_dev{s}, i_dev{s1}, jfet{d}_jac) = {call};\n"
                ));
                code.push_str(&format!(
                    "{indent}let jdev_{s}_{s} = jfet{d}_jac[1];\n\
                     {indent}let jdev_{s}_{s1} = jfet{d}_jac[0];\n\
                     {indent}let jdev_{s1}_{s} = jfet{d}_jac[3];\n\
                     {indent}let jdev_{s1}_{s1} = jfet{d}_jac[2];\n"
                ));
            }
            (DeviceType::Mosfet, DeviceParams::Mosfet(_)) => {
                let s1 = s + 1;
                code.push_str(&format!(
                    "{indent}let i_dev{s} = mosfet_id(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n\
                     {indent}let i_dev{s1} = mosfet_ig(v_d{s1}, DEVICE_{d}_SIGN);\n"
                ));
                code.push_str(&format!(
                    "{indent}let mos{d}_jac = mosfet_jacobian(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n"
                ));
                code.push_str(&format!(
                    "{indent}let jdev_{s}_{s} = mos{d}_jac[1];\n\
                     {indent}let jdev_{s}_{s1} = mos{d}_jac[0];\n\
                     {indent}let jdev_{s1}_{s} = mos{d}_jac[3];\n\
                     {indent}let jdev_{s1}_{s1} = mos{d}_jac[2];\n"
                ));
            }
            (DeviceType::Tube, DeviceParams::Tube(tp)) => {
                let s1 = s + 1;
                if tp.is_pentode() {
                    // Pentode / beam tetrode NR block. See
                    // [`pentode_dispatch`] for the 8-way helper family
                    // selection and [`emit_pentode_nr_dk_stamp`] for the
                    // shared (primary + BE fallback) DK Schur emitter.
                    emit_pentode_nr_dk_stamp(code, tp, d, s, indent);
                } else {
                    // Self-heating Vgk drift: see `nr_helpers.rs` for rationale.
                    let vgk_expr = if tp.has_self_heating() {
                        format!(
                            "(v_d{s} + DEVICE_{d}_VBIAS_ALPHA * (state.device_{d}_tj - DEVICE_{d}_TAMB))"
                        )
                    } else {
                        format!("v_d{s}")
                    };
                    if tp.has_rgi() {
                        code.push_str(&format!(
                            "{indent}let (i_dev{s}, i_dev{s1}, tube{d}_jac) = tube_evaluate_with_rgi({vgk_expr}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_gg, state.device_{d}_xi, state.device_{d}_cg, state.device_{d}_lambda, DEVICE_{d}_RGI);\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}let (i_dev{s}, i_dev{s1}, tube{d}_jac) = tube_evaluate({vgk_expr}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_gg, state.device_{d}_xi, state.device_{d}_cg, state.device_{d}_lambda);\n"
                        ));
                    }
                    code.push_str(&format!(
                        "{indent}let jdev_{s}_{s} = tube{d}_jac[0];\n\
                         {indent}let jdev_{s}_{s1} = tube{d}_jac[1];\n\
                         {indent}let jdev_{s1}_{s} = tube{d}_jac[2];\n\
                         {indent}let jdev_{s1}_{s1} = tube{d}_jac[3];\n"
                    ));
                }
            }
            (DeviceType::Vca, DeviceParams::Vca(_vp)) => {
                let s1 = s + 1;
                code.push_str(&format!(
                    "{indent}let i_dev{s} = vca_current(v_d{s}, v_d{s1}, state.device_{d}_g0, state.device_{d}_vscale, DEVICE_{d}_THD);\n\
                     {indent}let i_dev{s1} = 0.0;\n\
                     {indent}let vca{d}_jac = vca_jacobian(v_d{s}, v_d{s1}, state.device_{d}_g0, state.device_{d}_vscale, DEVICE_{d}_THD);\n\
                     {indent}let jdev_{s}_{s} = vca{d}_jac[0];\n\
                     {indent}let jdev_{s}_{s1} = vca{d}_jac[1];\n\
                     {indent}let jdev_{s1}_{s} = vca{d}_jac[2];\n\
                     {indent}let jdev_{s1}_{s1} = vca{d}_jac[3];\n"
                ));
            }
            (DeviceType::Ldr, DeviceParams::Ldr(_)) => {
                // Opto/LDR resistance path: linear in-solve, frozen state block.
                // i = v_d / r_state, jac = 1/r_state. `.max(1e-12)` guards the
                // division (mirrors ldr.rs).
                code.push_str(&format!(
                    "{indent}let ldr_r{d} = state.device_{d}_state[0].max(1e-12);\n\
                     {indent}let i_dev{s} = v_d{s} / ldr_r{d};\n\
                     {indent}let jdev_{s}_{s} = 1.0 / ldr_r{d};\n"
                ));
            }
            (DeviceType::Glow, DeviceParams::Glow(gp)) => {
                // Glow / neon lamp: FROZEN latch (dark < 0.5 → ROFF resistor
                // through the origin, lit → maintaining line). Latch frozen this
                // solve; update() flips it. MUST mirror the DK site (nr_helpers).
                if gp.has_sections() {
                    // Relaxing lit branch: invert g(I)=v_d via the inner Newton
                    // helper, reading the frozen section current-lags Ī_i.
                    code.push_str(&format!(
                        "{indent}let glow_lit{d} = state.device_{d}_state[0] >= 0.5;\n\
                         {indent}let (i_dev{s}, jdev_{s}_{s}) = if glow_lit{d} {{\n\
                         {indent}    let glow_i_bar{d} = [state.device_{d}_state[1], state.device_{d}_state[2], state.device_{d}_state[3], state.device_{d}_state[4]];\n\
                         {indent}    glow_lit_eval(v_d{s}, DEVICE_{d}_V0, DEVICE_{d}_RT, &[DEVICE_{d}_K1, DEVICE_{d}_K2, DEVICE_{d}_K3, DEVICE_{d}_K4], &glow_i_bar{d}, DEVICE_{d}_IFLOOR, DEVICE_{d}_KSUB, DEVICE_{d}_I_N)\n\
                         {indent}}} else {{\n\
                         {indent}    let glow_g{d} = 1.0 / DEVICE_{d}_ROFF;\n\
                         {indent}    (v_d{s} * glow_g{d}, glow_g{d})\n\
                         {indent}}};\n"
                    ));
                } else {
                    // Static maintaining line i=(v−V0)/RS (lit) / resistor
                    // through the origin (dark). Jacobian 1/glow_r either way
                    // (the V0 term is affine). Byte-identical to the historical
                    // model.
                    code.push_str(&format!(
                        "{indent}let glow_lit{d} = state.device_{d}_state[0] >= 0.5;\n\
                         {indent}let glow_r{d} = if glow_lit{d} {{ DEVICE_{d}_RS }} else {{ DEVICE_{d}_ROFF }};\n\
                         {indent}let glow_emf{d} = if glow_lit{d} {{ DEVICE_{d}_V0 }} else {{ 0.0 }};\n\
                         {indent}let i_dev{s} = (v_d{s} - glow_emf{d}) / glow_r{d};\n\
                         {indent}let jdev_{s}_{s} = 1.0 / glow_r{d};\n"
                    ));
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// The solve part of a nodal-Schur sample: the primary solve and, on a
    /// trapezoidal build, the backward-Euler instance of the same routine
    /// (latch, breakpoint, trapezoidal failure, ActiveSetBe rail).
    fn emit_schur_solve(
        code: &mut String,
        ir: &CircuitIR,
        noise: &NoiseEmission,
        setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
    ) -> Result<(), CodegenError> {
        let m = ir.topology.m;
        let multi_input = ir.solver_config.num_inputs() > 1;
        let primary = SchurSite::primary(ir);
        let be = SchurSite::be_instance(ir);

        if m == 0 {
            // Linear circuit: v_pred is the answer. A breakpoint sample (after a
            // capacitor/inductor .switch swap) takes the BE solve: a_neg_be = (1/T)C has no G
            // term, so the swapped conductance is not double-counted, and BE
            // damps trap's z=-1 mode at the source.
            code.push_str("    // Linear circuit: v = v_pred (no NR needed)\n");
            let binding = if Self::m0_rail_handling_mutates_v(ir) {
                "let mut v"
            } else {
                "let v"
            };
            // Each branch resolves a rail pin on its own integrator's matrix and
            // right-hand side, so a breakpoint sample is the BE build's sample.
            if ir.solver_config.breakpoint_be && !ir.solver_config.backward_euler {
                code.push_str(&format!(
                    "    {binding};\n    if state.breakpoint_be > 0 {{\n    q_be = true;\n"
                ));
                Self::emit_schur_rhs_pred(code, ir, noise, &be, NoiseMode::Draw);
                code.push_str("    v = v_pred;\n");
                Self::emit_nodal_m0_rail_handling(code, ir, "    ", be.a, PinSite::Schur);
                code.push_str("    } else {\n");
                Self::emit_schur_rhs_pred(code, ir, noise, &primary, NoiseMode::Draw);
                code.push_str("    v = v_pred;\n");
                Self::emit_nodal_m0_rail_handling(code, ir, "    ", primary.a, PinSite::Schur);
                code.push_str("    }\n\n");
            } else {
                Self::emit_schur_rhs_pred(code, ir, noise, &primary, NoiseMode::Draw);
                code.push_str(&format!("    {binding} = v_pred;\n\n"));
                Self::emit_nodal_m0_rail_handling(code, ir, "    ", primary.a, PinSite::Schur);
            }
            return Ok(());
        }

        // Each solve is followed by the sub-step ladder (the full-LU routine,
        // shared) on its own integrator, then by its rail handling on whichever
        // `v` the solve ended with. `converged` is the ladder's contract: this
        // solve's outcome, which the death-spiral hold reads at the end.
        let not_converged_count = "    if state.last_nr_iterations >= MAX_ITER as u32 {\n\
             \x20       state.diag_nr_max_iter_count += 1;\n\
             \x20   }\n";
        let converged_now = "    converged = state.last_nr_iterations < MAX_ITER as u32;\n";
        if ir.solver_config.backward_euler {
            Self::emit_schur_rhs_pred(code, ir, noise, &primary, NoiseMode::Draw);
            Self::emit_schur_newton(code, ir, &primary, true)?;
            code.push_str(not_converged_count);
            code.push_str("    let mut converged = state.last_nr_iterations < MAX_ITER as u32;\n");
            Self::emit_substep_ladder(code, ir, noise, setter_stamps, true, false);
            Self::emit_schur_rail_handling(code, ir, &primary);
            code.push('\n');
            return Ok(());
        }

        let active_set_be = matches!(
            ir.solver_config.opamp_rail_mode,
            crate::codegen::OpampRailMode::ActiveSetBe
        );
        let forced = match (
            ir.solver_config.runtime_be_latch,
            ir.solver_config.breakpoint_be,
        ) {
            (true, true) => Some("state.be_latched || state.breakpoint_be > 0"),
            (true, false) => Some("state.be_latched"),
            (false, true) => Some("state.breakpoint_be > 0"),
            (false, false) => None,
        };
        // Every path writes `v` and `converged` before they are read; without
        // a forced BE sample the trapezoidal block always runs, so the initial
        // values are dead.
        code.push_str(
            "    #[allow(unused_assignments)]\n    let mut v = [0.0f64; N];\n    let mut i_nl = [0.0f64; M];\n\
             \x20   #[allow(unused_assignments)]\n    let mut converged = false;\n",
        );
        if active_set_be {
            code.push_str("    let mut active_set_engaged = false;\n");
        }
        if ir.solver_config.subsample_fire && !multi_input {
            // The sub-sample-fire re-solve stamps the input at function scope.
            code.push_str("    let input_conductance = 1.0 / INPUT_RESISTANCE;\n");
        }
        match forced {
            Some(f) => code.push_str(&format!("    let be_first = {f};\n    if !be_first {{\n")),
            None => code.push_str("    {\n"),
        }
        Self::emit_schur_rhs_pred(code, ir, noise, &primary, NoiseMode::Draw);
        Self::emit_schur_newton(code, ir, &primary, false)?;
        // Max-iter counts a genuine trapezoidal exhaustion, whether or not the
        // ladder or the BE solve then recovers it.
        code.push_str(not_converged_count);
        code.push_str(converged_now);
        Self::emit_substep_ladder(code, ir, noise, setter_stamps, false, false);
        Self::emit_schur_rail_handling(code, ir, &primary);
        code.push_str("    }\n\n");

        // `trap_ok` = the trapezoidal solve was accepted (the sub-sample-fire
        // block reads it too).
        let mut trap_ok = "converged".to_string();
        if active_set_be {
            trap_ok.push_str(" && !active_set_engaged");
        }
        code.push_str(&format!("    let trap_ok = {trap_ok};\n"));
        code.push_str(
            "    // Backward-Euler solve: the same routine a BE build runs, on the BE\n\
             \x20   // kernel (s_be/k_be/s_ni_be, a_be/a_neg_be).\n",
        );
        code.push_str("    if !trap_ok {\n");
        // Diag contract: be_fallback counts every entry.
        code.push_str("        state.diag_be_fallback_count += 1;\n");
        code.push_str("        q_be = true;\n");
        if carries_q_dot(ir) {
            // A trapezoidal ladder that converged but engaged an ActiveSetBe
            // rail left its charge state here; the BE solve replaces it.
            code.push_str("        q_sub = None;\n");
        }
        if ir.solver_config.breakpoint_be {
            code.push_str(
                "        let be_iter_budget = if state.breakpoint_be > 0 { BREAKPOINT_BE_MAX_ITER } else { MAX_ITER };\n",
            );
        }
        let noise_mode = match forced {
            Some(_) => NoiseMode::DrawIf("be_first"),
            None => NoiseMode::Replay,
        };
        Self::emit_schur_rhs_pred(code, ir, noise, &be, noise_mode);
        Self::emit_schur_newton(code, ir, &be, false)?;
        if forced.is_some() {
            // A forced sample's BE solve is its primary: count its exhaustion.
            code.push_str(
                "        if be_first && state.last_nr_iterations >= MAX_ITER as u32 {\n\
                 \x20           state.diag_nr_max_iter_count += 1;\n\
                 \x20       }\n",
            );
        }
        code.push_str(converged_now);
        Self::emit_substep_ladder(code, ir, noise, setter_stamps, true, false);
        Self::emit_schur_rail_handling(code, ir, &be);
        code.push_str("    }\n\n");
        Ok(())
    }

    /// Schur Step 1-2 for one integrator (`site`): the right-hand side into a
    /// local `rhs`, then the linear prediction `v_pred = S·rhs`.
    fn emit_schur_rhs_pred(
        code: &mut String,
        ir: &CircuitIR,
        noise: &NoiseEmission,
        site: &SchurSite,
        noise_mode: NoiseMode,
    ) {
        emit_history_rhs(
            code,
            ir,
            site.rhs_const,
            site.a_neg,
            site.a_neg_sparsity(ir),
            !site.be,
        );
        // NOTE: `state.input_prev` is deliberately NOT committed here. The
        // ActiveSetBe sub-step machinery below interpolates the input ramp as
        // `(input - state.input_prev) / N_SUB`, so committing before the
        // sub-steps read it would zero the ramp exactly on the hard-transient
        // samples that trigger sub-stepping. The commit happens in the
        // end-of-sample state-update block (matching the DK template).
        code.push('\n');

        // Runtime voltage sources (.runtime directive): host-driven per-sample values.
        // Stamped after the DC RHS_CONST and the input stamp so the field value is
        // additive with any DC bias declared on the voltage source itself.
        if !ir.runtime_sources.is_empty() {
            code.push_str(
                "    // Runtime voltage sources (.runtime directive), sanitised copies\n",
            );
            for rt in &ir.runtime_sources {
                code.push_str(&format!(
                    "    rhs[{}] += state.{}{};\n",
                    rt.vs_row,
                    rt.field_name,
                    super::super::runtime_inputs::SANITIZED_SUFFIX
                ));
            }
            code.push('\n');
        }

        // Authentic circuit noise — Phase 1 thermal stamp.
        //
        // Stamped once per audio sample (NOT per NR iteration), after all
        // deterministic RHS contributions and before the linear prediction
        // `v_pred = S * rhs` so noise is shaped by the circuit's transfer
        // function exactly like the input source. BE-fallback samples don't
        // re-draw — they reuse the trapezoidal NR's noise contribution
        // implicitly via `v_prev` history.
        //
        // The fragment is `""` when noise mode is Off — zero bytes emitted
        // and the build is byte-identical to a noiseless one.
        if noise.enabled {
            match noise_mode {
                NoiseMode::Draw => code.push_str(&noise.rhs_stamp),
                NoiseMode::Replay => {
                    code.push_str(&emit_noise_replay_body(noise.replay_counts, "rhs", "    "))
                }
                NoiseMode::DrawIf(cond) => {
                    code.push_str(&format!("    if {cond} {{\n"));
                    code.push_str(&noise.rhs_stamp);
                    code.push_str("    } else {\n");
                    code.push_str(&emit_noise_replay_body(
                        noise.replay_counts,
                        "rhs",
                        "        ",
                    ));
                    code.push_str("    }\n");
                }
            }
            code.push('\n');
        }

        // Step 2: Linear prediction v_pred = S * rhs (O(N^2))
        code.push_str("    // Step 2: Linear prediction v_pred = S * rhs (O(N^2))\n");
        code.push_str("    let mut v_pred = [0.0f64; N];\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        let mut sum = 0.0;\n");
        code.push_str(&format!(
            "        for j in 0..N {{ sum += {}[i][j] * rhs[j]; }}\n",
            site.s
        ));
        code.push_str("        v_pred[i] = sum;\n");
        code.push_str("    }\n\n");
    }

    /// Schur Step 3-5 for one integrator (`site`): the M-dimensional Newton
    /// loop, recovery of the full `v`, and op-amp rail handling. The BE build's
    /// solve and a trapezoidal build's backward-Euler solve (latch, fallback,
    /// breakpoint, ActiveSetBe rail) are this one routine. With `declare`
    /// false it writes the caller's `v`, `i_nl`, `active_set_engaged`.
    fn emit_schur_newton(
        code: &mut String,
        ir: &CircuitIR,
        site: &SchurSite,
        declare: bool,
    ) -> Result<(), CodegenError> {
        let m = ir.topology.m;
        // Step 3: Extract device voltages p = N_v * v_pred (O(M*N))
        code.push_str("    // Step 3: Extract device voltages p = N_v * v_pred (sparse)\n");
        code.push_str("    let mut p = [0.0f64; M];\n");
        for i in 0..m {
            let nz_cols = &ir.sparsity.n_v.nz_by_row[i];
            if nz_cols.is_empty() {
                continue;
            }
            let terms: Vec<String> = nz_cols
                .iter()
                .map(|&j| format!("N_V[{}][{}] * v_pred[{}]", i, j, j))
                .collect();
            code.push_str(&format!("    p[{}] = {};\n", i, terms.join(" + ")));
        }
        code.push('\n');

        // MOSFET body effect: evaluated inside the Newton loops below, at
        // each iterate (see helpers::emit_body_effect_at_iterate).

        // Step 4: M-dim NR (same structure as DK solve_nonlinear)
        code.push_str("    // Step 4: M-dim Newton-Raphson (Schur complement)\n");
        if declare {
            code.push_str("    let mut i_nl = [0.0f64; M];\n");
        }
        if has_latched_device(ir) {
            // Glow present → ZERO-ORDER warm start: copy the previous i_nl
            // (a memcpy — `copy_from_slice` keeps clippy quiet). The
            // first-order predictor `2·i_prev − i_prev_prev` extrapolates the
            // stiff lit-discharge current (RS↔ROFF is a ~1e5 conductance step)
            // into the cathode diode's reverse breakdown, which the Schur
            // convergence accepts (nodal divergence to ~1e6 V). The overshoot
            // spans the whole lit discharge, not just the flip, so
            // flip-adjacent narrowing is insufficient (measured); unconditional
            // zero-order-when-glow is clean and tighter. Compile-time gated on
            // latched-device presence → byte-identical for every non-glow circuit.
            code.push_str("    i_nl.copy_from_slice(&state.i_nl_prev);\n");
        } else {
            Self::emit_schur_warm_start(code, ir, site);
        }
        // Convergence is determined post-loop by `state.last_nr_iterations
        // < MAX_ITER as u32` (see emission a few lines below). Earlier
        // versions of the emitter declared `let mut converged = false;`
        // here and set it inside the NR loop; that was dead code because
        // every emit path immediately shadowed it with `let converged =
        // …;` after the loop. Removing the dead declaration eliminates a
        // clippy `unused_assignments` warning in generated code.
        code.push_str("    state.last_nr_iterations = MAX_ITER as u32;\n\n");

        // Trapezoidal NR loop
        code.push_str(&format!("    for iter in 0..{} {{\n", site.iter_budget));

        // 4a. Compute v_d = p + K * i_nl
        code.push_str("        // 4a. Compute controlling voltages: v_d = p + K * i_nl\n");
        for i in 0..m {
            code.push_str(&format!("        let v_d{} = p[{}]", i, i));
            for &j in &site.k_sparsity(ir).nz_by_row[i] {
                code.push_str(&format!(" + {}[{}][{}] * i_nl[{}]", site.k, i, j, j));
            }
            code.push_str(";\n");
        }
        code.push('\n');

        emit_body_effect_at_iterate(code, ir, "v_pred", site.s_ni, "        ");
        // 4b. Evaluate device currents and Jacobian (reuse DK style)
        code.push_str("        // 4b. Evaluate device currents and Jacobians\n");
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            Self::emit_dk_device_eval_for_nodal_schur(code, dev_num, slot)?;
        }
        code.push('\n');

        // 4c. Residuals
        code.push_str("        // 4c. Residuals: f(i) = i_nl - i_dev = 0\n");
        for i in 0..m {
            code.push_str(&format!("        let f{} = i_nl[{}] - i_dev{};\n", i, i, i));
        }
        code.push('\n');

        // 4d. NR Jacobian: J[i][j] = delta_ij - sum_k(jdev_ik * K[k][j])
        code.push_str("        // 4d. Jacobian: J[i][j] = delta_ij - jdev * K\n");
        for i in 0..m {
            let slot = ir
                .device_slots
                .iter()
                .find(|s| i >= s.start_idx && i < s.start_idx + s.dimension)
                .ok_or_else(|| {
                    CodegenError::InvalidConfig(format!(
                        "no device slot found for M-dimension index {}",
                        i
                    ))
                })?;
            let blk_start = slot.start_idx;
            let blk_dim = slot.dimension;
            for j in 0..m {
                let diag = if i == j { "1.0" } else { "0.0" };
                let mut terms = String::new();
                for k in blk_start..blk_start + blk_dim {
                    terms.push_str(&format!(" - jdev_{}_{} * {}[{}][{}]", i, k, site.k, k, j));
                }
                terms.push_str(&body_effect_jacobian_term(ir, i, j, site.s_ni));
                // Separator is load-bearing: `j{i}{j}` without it collides
                // at M≥12 (e.g. j110 could be i=1,j=10 or i=11,j=0).
                code.push_str(&format!("        let j{}_{} = {}{};\n", i, j, diag, terms));
            }
        }
        code.push('\n');

        // 4e. Solve the M×M linear system (1×1, 2×2 Cramer, 3..16 Gauss)
        // Uses `break` on convergence (not `return i_nl` like DK's solve_nonlinear)
        match m {
            1 => {
                code.push_str("        // Solve 1x1: delta = f / J\n");
                code.push_str("        let det = j0_0;\n");
                code.push_str("        if det.abs() < 1e-15 {\n");
                emit_nr_singular_fallback(code, 1, "            ");
                code.push_str("            continue;\n");
                code.push_str("        }\n");
                code.push_str("        let delta0 = f0 / det;\n\n");
                emit_schur_nr_limit_and_converge(code, ir, 1, "        ", site.k);
            }
            2 => {
                code.push_str("        // Solve 2x2 (Cramer's rule)\n");
                code.push_str("        let det = j0_0 * j1_1 - j0_1 * j1_0;\n");
                code.push_str("        if det.abs() < 1e-15 {\n");
                emit_nr_singular_fallback(code, 2, "            ");
                code.push_str("            continue;\n");
                code.push_str("        }\n");
                code.push_str("        let inv_det = 1.0 / det;\n");
                code.push_str("        let delta0 = inv_det * (j1_1 * f0 - j0_1 * f1);\n");
                code.push_str("        let delta1 = inv_det * (-j1_0 * f0 + j0_0 * f1);\n\n");
                emit_schur_nr_limit_and_converge(code, ir, 2, "        ", site.k);
            }
            3..=crate::dk::MAX_M => {
                Self::generate_schur_gauss_elim_k(code, ir, m, site.k);
            }
            _ => {
                return Err(CodegenError::UnsupportedTopology(crate::dk::max_m_refusal(
                    m,
                )));
            }
        }

        code.push_str("    }\n\n"); // end trapezoidal NR loop

        // Step 5: Recover full v = v_pred + S_NI * i_nl
        code.push_str("    // Step 5: Recover full node voltages: v = v_pred + S_NI * i_nl\n");
        if declare {
            code.push_str("    let mut v = v_pred;\n");
        } else {
            code.push_str("    v = v_pred;\n");
        }
        code.push_str("    for i in 0..N {\n");
        code.push_str(&format!(
            "        for j in 0..M {{ v[i] += {}[i][j] * i_nl[j]; }}\n",
            site.s_ni
        ));
        code.push_str("    }\n");

        Ok(())
    }

    /// The Schur Newton's starting point: the device currents `i_nl` whose
    /// controlling voltages `p + K·i_nl` equal `N_v·v_prev`, the point the
    /// full-LU Newton starts from (`v = v_prev`). One starting-point definition
    /// for both nodal sub-paths.
    ///
    /// Near a regenerative fold the implicit step has more than one root, and
    /// which one Newton reaches depends on where it starts. The first-order
    /// predictor `2·i_prev − i_prev_prev` started the Schur Newton elsewhere,
    /// and it reached a genuine root on the switched branch before the fold:
    /// an IC-seeded transistor astable at 192 kHz settled to a 0.4617 ms period
    /// against ngspice's 1.1662 ms, with every sample KCL-valid and no counter
    /// moving. From this start it settles to 1.1664 ms (design review).
    ///
    /// `K`'s rank-revealing LU is cached in the state and refactored only when
    /// `K` changes (a rebuild, a rate change, a reset). A singular `K` (devices
    /// sharing a controlling voltage) is solved exactly; see
    /// [`emit_k_seed_helpers`]. Only an inconsistent system falls back to the
    /// first-order predictor, counted in `diag_warm_start_fallback_count`.
    fn emit_schur_warm_start(code: &mut String, ir: &CircuitIR, site: &SchurSite) {
        let m = ir.topology.m;
        let k = site.k;
        let name = k.strip_prefix("state.").unwrap_or(k);
        code.push_str(
            "    // Warm start at full-LU's starting point: solve K·i_nl = N_v·v_prev − p.\n",
        );
        code.push_str("    {\n");
        code.push_str(&format!(
            "        if {k} != state.ws_{name}_key {{\n\
             \x20           let (lu, pr, pc, rank) = k_factor(&{k});\n\
             \x20           state.ws_{name}_lu = lu;\n\
             \x20           state.ws_{name}_pr = pr;\n\
             \x20           state.ws_{name}_pc = pc;\n\
             \x20           state.ws_{name}_rank = rank;\n\
             \x20           state.ws_{name}_key = {k};\n\
             \x20       }}\n"
        ));
        code.push_str("        let mut kb = [0.0f64; M];\n");
        for i in 0..m {
            let terms: Vec<String> = ir.sparsity.n_v.nz_by_row[i]
                .iter()
                .map(|&j| format!("N_V[{i}][{j}] * state.v_prev[{j}]"))
                .collect();
            let vl = if terms.is_empty() {
                "0.0".to_string()
            } else {
                terms.join(" + ")
            };
            code.push_str(&format!("        kb[{i}] = {vl} - p[{i}];\n"));
        }
        code.push_str(&format!(
            "        match k_solve(&state.ws_{name}_lu, &state.ws_{name}_pr, &state.ws_{name}_pc, state.ws_{name}_rank, kb) {{\n\
             \x20           Some(x) if x.iter().all(|v| v.is_finite()) => i_nl = x,\n\
             \x20           _ => {{\n\
             \x20               // Unreachable start: the first-order predictor, counted.\n\
             \x20               for i in 0..M {{ i_nl[i] = 2.0 * state.i_nl_prev[i] - state.i_nl_prev_prev[i]; }}\n\
             \x20               state.diag_warm_start_fallback_count += 1;\n\
             \x20           }}\n\
             \x20       }}\n\
             \x20   }}\n"
        ));
    }

    /// The op-amp rail handling of one Schur solve (`site`), on the final `v`
    /// of that solve: its Newton, or the sub-step ladder that rescued it.
    fn emit_schur_rail_handling(code: &mut String, ir: &CircuitIR, site: &SchurSite) {
        // Op-amp supply rail handling.
        //
        // * `Hard`  — apply the post-NR `v[out].clamp(VEE, VCC)` mutation
        //             (matches pre-2026-04 behavior). This corrupts cap
        //             history for AC-coupled downstream stages; only use
        //             on circuits with DC-coupled downstream.
        // * `ActiveSet` — call `emit_nodal_active_set_resolve` against
        //             `state.a` (trapezoidal). Detects rail violations
        //             and pins them via row/column elimination, then
        //             re-solves the whole network so KCL is satisfied at
        //             every node with the clamped outputs. The
        //             auto-resolver's choice.
        // * `ActiveSetBe` — detect rail violations here without mutating;
        //             if any are detected, fall through to the BE fallback
        //             below (which re-runs NR with backward-Euler matrices
        //             and then applies the active-set row/col elimination
        //             using `state.a_be`). Explicit mode only: it damps
        //             whole rail plateaus, so its error is first-order
        //             where ActiveSet is not.
        // * `BoyleDiodes` — physical catch diodes are already in the
        //             MNA via `augment_netlist_with_boyle_diodes`. NR
        //             handles saturation naturally through the diode
        //             exponential, producing a soft knee. Emit nothing
        //             here.
        // * `None`  — no clamping; caller accepts unbounded output.
        use crate::codegen::OpampRailMode;
        match ir.solver_config.opamp_rail_mode {
            OpampRailMode::Hard => {
                for oa in &ir.opamps {
                    // rail_clamp_stmt returns None for op-amps that only
                    // appear in OpampIR for slew-rate limiting (VCC/VEE
                    // both infinite) and clamps only finite bounds.
                    let target = format!("v[{}]", oa.n_out_idx);
                    if let Some(stmt) = Self::rail_clamp_stmt(&target, oa.vclamp_lo, oa.vclamp_hi) {
                        code.push_str(&format!("    {stmt}\n"));
                    }
                }
            }
            OpampRailMode::ActiveSet => {
                // Pin and re-solve on the site's own integrator.
                Self::emit_nodal_active_set_resolve(
                    code,
                    ir,
                    "    ",
                    site.a,
                    "rhs",
                    PinSite::Schur,
                );
            }
            OpampRailMode::ActiveSetBe if site.be => {
                // The backward-Euler solve pins and re-solves on its own
                // matrices: this is where ActiveSetBe's resolve belongs.
                Self::emit_nodal_active_set_resolve(
                    code,
                    ir,
                    "    ",
                    site.a,
                    "rhs",
                    PinSite::Schur,
                );
            }
            OpampRailMode::ActiveSetBe => {
                // Detect-only on the trapezoidal solve; an engaged rail
                // hands the sample to the backward-Euler solve.
                Self::emit_nodal_active_set_check(code, ir, "    ", "active_set_engaged");
            }
            OpampRailMode::BoyleDiodes => {
                // Catch diodes are physically in the circuit — no extra
                // post-NR mutation needed.
            }
            OpampRailMode::None => {
                // No clamping — caller accepts unbounded op-amp output.
            }
            OpampRailMode::Auto => {
                // resolve_opamp_rail_mode() is responsible for converting
                // Auto to a concrete mode before reaching the emitter.
                unreachable!(
                    "OpampRailMode::Auto should have been resolved in ir::from_mna; \
                         emitter should only see concrete modes"
                );
            }
        }
    }
}
