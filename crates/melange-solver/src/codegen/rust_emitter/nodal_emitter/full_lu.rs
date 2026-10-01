//! Full-LU `process_sample`, solve and RHS.

use super::behavioral::emit_behavioral_time_update;
use super::reset::emit_nodal_nan_reset;
use super::residual::emit_kcl_residual_fns;
use super::sat_ind::emit_sat_ind_history;
use super::sites::{
    emit_chord_cache, emit_history_rhs, emit_q_dot_commit, emit_q_dot_locals, emits_hold,
    has_be_instance, NewtonSite, NoiseMode, PinSite,
};
use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::dk_emitter::{emit_noise_replay_body, NoiseEmission};
use crate::codegen::rust_emitter::helpers::{
    body_effect_mosfets, carries_q_dot, emit_glow_lit_be_hold, emit_stateful_update,
    section_banner, stateful_device_data,
};
use crate::codegen::rust_emitter::RustEmitter;

impl RustEmitter {
    /// Emit the complete process_sample function for the nodal solver (O(N^3) LU path).
    ///
    /// Used when the Schur path is unstable: K ≈ 0 (device Jacobian provides
    /// essential damping not captured by Schur), positive K diagonal, or
    /// ill-conditioned K. Matches the runtime NodalSolver's solve_equilibrated().
    /// `setter_stamps` carries the literal `(row, col)` positions the emitted
    /// dynamic-parameter setters can write into `g_work`/`c_work` (see
    /// `EquilPattern`). The saturating-inductor row residual needs it for the
    /// same reason the equilibration pattern does: to know which entries of an
    /// augmented row are codegen-time constants and which can move at runtime.
    pub(super) fn emit_nodal_process_sample(
        ir: &CircuitIR,
        noise: &NoiseEmission,
        setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
    ) -> String {
        let m = ir.topology.m;
        let os_factor = ir.solver_config.oversampling_factor;
        // Multi-input ports (M=0 only): see emit_nodal_schur_process_sample.
        // Multi-input is CLI-restricted to M==0 && OS==1, so the sub-step /
        // BE-fallback blocks (all M>0) never coexist with `multi_input`; still
        // gated defensively.
        let multi_input = ir.solver_config.num_inputs() > 1;
        let has_behavioral = !ir.behavioral_sources.is_empty();
        let has_sat_ind = !ir.saturating_inductors.is_empty();
        // Site-local integrator scalar for the saturating-inductor flux stamps.
        // Matches the base matrix each NR site is built from: `state.a`/`a_be`
        // are rebuilt at `internal_rate = current_sample_rate * OS` (2×trap,
        // 1×BE — see rebuild_matrices), so we recompute the same product here.
        let sat_int_rate = "state.current_sample_rate * OVERSAMPLING_FACTOR as f64";
        let sat_alpha_main = if ir.solver_config.backward_euler {
            format!("({sat_int_rate})")
        } else {
            format!("(2.0 * {sat_int_rate})")
        };
        let has_bsrc_time = ir.behavioral_sources.iter().any(|b| b.time_dependent);
        // Forcing backward Euler (latch, breakpoint) is decided in
        // `emit_nodal_solve`, which skips the trapezoidal solve on such samples.

        // `.inject`/`.tap`: see emit_nodal_schur_process_sample. Mutually
        // exclusive with multi_input (CLI-rejected).
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

        // Nonlinear-device circuits get the Armijo line search + node-KCL
        // residual convergence gate in all three NR loops (trap / sub-step / BE).
        // Emitted only for m > 0: with no devices there is no exponential
        // stiffness to limit-cycle on, and the linear LU solve is already exact,
        // so m == 0 circuits stay byte-identical.
        let use_line_search = m > 0;

        let mut code = String::new();
        if use_line_search {
            code.push_str(&emit_kcl_residual_fns(ir, setter_stamps));
        }
        code.push_str(&section_banner(
            "PROCESS SAMPLE (Full-nodal NR with LU solve)",
        ));

        // Function signature
        if os_factor > 1 || inject_or_tap {
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
                "/// Uses full-nodal Newton-Raphson with LU factorization per iteration.\n",
            );
            code.push_str("/// Includes backward Euler fallback for unconditional stability.\n");
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
                 \x20   for v in injections.iter_mut() { *v = if v.is_finite() { *v } else { state.diag_input_nan_count += 1; 0.0 }; }\n\n",
            );
        }

        // Behavioral ddt/idt scaling locals (referenced by the resolver).
        if has_bsrc_time {
            code.push_str("    let bsrc_inv_dt = state.bsrc_inv_dt;\n");
            code.push_str("    let bsrc_half_dt = state.bsrc_half_dt;\n\n");
        }

        // Lazy rebuild: batch all pot/switch changes into one matrix rebuild
        let has_pots = !ir.pots.is_empty();
        let has_switches = !ir.switches.is_empty();
        if has_pots || has_switches {
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

        // Handle linear circuits (M=0): direct LU solve, no NR iteration.
        // Behavioral B-sources are nonlinear even when M=0, so they take the NR
        // path below. Saturating inductors are also nonlinear (flux
        // stamp on their augmented branch row), so an M=0 circuit with any
        // saturating inductor must iterate too.
        let primary = NewtonSite::primary(ir);
        emit_q_dot_locals(
            &mut code,
            ir,
            "    ",
            has_be_instance(ir) || ir.solver_config.breakpoint_be,
        );
        if m == 0 && !has_behavioral && !has_sat_ind {
            // A breakpoint sample (a capacitor/inductor .switch swap, a lit
            // glow) solves the same direct LU on the BE matrices, as the BE build does.
            let linear_solve = |code: &mut String, site: &NewtonSite, sat_alpha: &str| {
                Self::emit_nodal_rhs(code, ir, noise, site, NoiseMode::Draw);
                code.push_str("    // Linear circuit: direct LU solve (no NR needed)\n");
                code.push_str(&format!("    let mut g_aug = {};\n", site.a));
                code.push_str(
                    "    // Gmin regularization: improves conditioning for high-gain VCCS (op-amps)\n",
                );
                code.push_str("    for i in 0..N_NODES { g_aug[i][i] += 1e-12; }\n");
                code.push_str("    let mut v = rhs;\n");
                code.push_str("    if !lu_solve(&mut g_aug, &mut v) {\n");
                code.push_str("        v = state.v_prev;\n");
                if carries_q_dot(ir) {
                    code.push_str("        q_sub = Some(state.q_dot);\n");
                }
                code.push_str("    }\n\n");

                // Op-amp supply rail handling. The M=0 branch historically
                // emitted none in any mode; see emit_nodal_m0_rail_handling.
                // (The blanket "no VSAT clamping" rule below applies to
                // arbitrary nodes — the Hard rail clamp here is scoped to
                // op-amp OUTPUT nodes, matching the M>0 paths' Hard mode.)
                Self::emit_nodal_m0_rail_handling(
                    code,
                    ir,
                    "    ",
                    site.a,
                    PinSite::FullLu {
                        sat_alpha,
                        setter_stamps,
                    },
                );
            };
            if ir.solver_config.breakpoint_be && !ir.solver_config.backward_euler {
                let be = NewtonSite::be_instance(ir);
                code.push_str(
                    "    #[allow(unused_mut)]\n    let mut v = if state.breakpoint_be > 0 {\n    q_be = true;\n",
                );
                linear_solve(&mut code, &be, &be.sat_alpha);
                code.push_str("    v\n    } else {\n");
                linear_solve(&mut code, &primary, &sat_alpha_main);
                code.push_str("    v\n    };\n\n");
            } else {
                linear_solve(&mut code, &primary, &sat_alpha_main);
            }

            // No VSAT clamping — matches runtime NodalSolver. Clamping any node
            // creates inconsistency with unclamped neighbors (e.g., 100Ω apart but
            // 227V difference), corrupting the trapezoidal history feedback.
        } else {
            Self::emit_nodal_solve(&mut code, ir, noise, setter_stamps);
        }

        // NaN/Inf recovery: shared reset + DC-OP return. Full-LU path
        // invalidates the cross-timestep chord LU factorization.
        emit_nodal_nan_reset(&mut code, ir, "    ", true, noise);

        // No VSAT clamping on v — clamping any subset of nodes creates physical
        // inconsistency with unclamped neighbors (e.g., 100Ω resistor between
        // clamped node at 13V and unclamped node at 240V), corrupting the
        // history (a_neg * v_prev + q_dot). Matches runtime NodalSolver.
        // Output is clamped downstream by DC block (±10V) or ear protection.

        // Op-amp slew-rate limiting (nodal full-LU path). Clamp
        // `|v[out] - v_prev[out]|` to `SR*dt` for each op-amp with finite
        // SR. Equivalent to clamping the Boyle dominant-pole integrator
        // input current at ±SR*C_dom. Zero code is emitted when all
        // op-amps have infinite SR.
        Self::emit_opamp_slew_limit(&mut code, ir, "    ", "v");

        // Death spiral protection: when ALL NR paths fail (trap + substep + BE),
        // do NOT store the bad partial iterate into v_prev/i_nl_prev. Each failed
        // sample would hand a progressively worse initial condition to the next,
        // creating a cascade. Instead, keep the previous (presumably converged)
        // state so the next sample starts from a reasonable point. The chord LU
        // is invalidated to force a fresh factorization.
        //
        // Guard mirrors the full-LU-path predicate used at emit sites 2443/2773/
        // 3181: behavioral B-sources (and saturating inductors) force the full-LU
        // path even at M=0, so an M=0 behavioral circuit that fails trap NR needs
        // this protection too. Without it the diverged trap iterate is committed
        // to v_prev unconditionally — and with the fallbacks below gated off for
        // behavioral circuits, a genuine trap failure now relies on this hold.
        Self::emit_death_spiral_hold(&mut code, ir, true);

        // Runtime BE-latch detector (updates state.be_latched for next sample).
        Self::emit_be_latch_detector(&mut code, ir, "    ");

        // Step 3: Update state
        code.push_str("    // Step 3: Update state\n");
        if has_bsrc_time {
            code.push_str("    // Behavioral ddt/idt companion-state update (at converged v)\n");
            emit_behavioral_time_update(&mut code, ir, "    ");
        }
        // Stateful-device (Phase 0c) after-solve update — BEFORE state.v_prev = v
        // so v_prev holds the prior sample. Shared with the DK path.
        code.push_str(&emit_stateful_update(&stateful_device_data(ir)));
        emit_q_dot_commit(
            &mut code,
            ir,
            "    ",
            has_be_instance(ir) || ir.solver_config.breakpoint_be,
        );
        code.push_str("    state.v_prev = v;\n");
        // Breakpoint-BE countdown: this sample was solved on the BE matrices via
        // the forced BE fallback. One decrement per sample, after the solve.
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
        // Whenever there is a Newton loop (devices, behavioral sources or
        // saturating inductors), its chord and i_nl locals are written back;
        // gating this on M > 0 left them as dead stores on M = 0 circuits,
        // which fail `-D warnings` in users' plugin builds.
        if m > 0 || has_behavioral || has_sat_ind {
            code.push_str("    state.i_nl_prev_prev = state.i_nl_prev;\n");
            code.push_str("    state.i_nl_prev = i_nl;\n");
            // Persist chord LU for cross-timestep reuse
            code.push_str("    state.chord_lu = chord_lu;\n");
            code.push_str("    state.chord_dr = chord_dr;\n");
            code.push_str("    state.chord_dc = chord_dc;\n");
            code.push_str("    state.chord_perm = chord_perm;\n");
            code.push_str("    state.chord_j_dev = chord_j_dev;\n");
            if !body_effect_mosfets(ir).is_empty() {
                code.push_str("    state.chord_body_gmb = chord_body_gmb;\n");
            }
            code.push_str("    state.chord_valid = chord_valid;\n");
            if ir.sparsity.lu.is_some() {
                code.push_str("    state.chord_dense = chord_dense;\n");
            }
        }
        for (idx, _pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "    state.pot_{}_resistance_prev = state.pot_{}_resistance;\n",
                idx, idx
            ));
        }
        code.push('\n');

        // Step 3b: Device self-heating thermal update (BJT, diode, triode) —
        // shared exact-exponential emitter, same as the Schur path.
        Self::emit_self_heating_thermal_updates(&mut code, ir);

        // (NaN check already done before state update)

        // Genuine trap max-iter counter (single site for this path): the
        // field is pessimistically initialized to MAX_ITER before the trap
        // loop and only overwritten on convergence, so this fires exactly
        // once per sample whose trapezoidal NR failed — including LU-factor
        // failures — regardless of whether substep/BE later recovered it.
        // Every build with a Newton loop counts, M = 0 included: a saturating
        // inductor or a behavioral source iterates without adding to M.
        if m > 0 || has_behavioral || has_sat_ind {
            code.push_str("    if state.last_nr_iterations >= MAX_ITER as u32 {\n");
            code.push_str("        state.diag_nr_max_iter_count += 1;\n");
            code.push_str("    }\n\n");
        }
        code.push_str(&super::super::helpers::emit_region_exit_lines(ir, "    "));

        // Step 4: Extract outputs, apply DC blocking and scaling
        code.push_str("    // Step 4: Extract outputs, DC blocking, and scaling\n");
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

        code
    }

    /// The Newton part of a full-LU sample: the primary solve and, on a
    /// trapezoidal build, the backward-Euler instance of the same routine.
    fn emit_nodal_solve(
        code: &mut String,
        ir: &CircuitIR,
        noise: &NoiseEmission,
        setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
    ) {
        let primary = NewtonSite::primary(ir);
        let active_set_be = matches!(
            ir.solver_config.opamp_rail_mode,
            crate::codegen::OpampRailMode::ActiveSetBe
        );
        code.push_str("    // Step 2: Newton-Raphson in full augmented voltage space\n");
        code.push_str("    let mut v = state.v_prev;\n");
        code.push_str("    let mut converged = false;\n");
        // Pessimistic init: overwritten with the converging iteration index on
        // success, so a failure leaves MAX_ITER for the post-loop max-iter
        // counter instead of last sample's stale value.
        code.push_str("    state.last_nr_iterations = MAX_ITER as u32;\n");
        code.push_str("    let mut i_nl = [0.0f64; M];\n");
        if active_set_be {
            code.push_str("    let mut active_set_engaged = false;\n");
        }
        code.push_str("    // Cross-timestep chord: start from previous sample's converged LU\n");
        emit_chord_cache(code, ir, "state.chord_", "    ", true);
        code.push('\n');

        if !has_be_instance(ir) {
            Self::emit_nodal_rhs(code, ir, noise, &primary, NoiseMode::Draw);
            Self::emit_nodal_newton(code, ir, noise, setter_stamps, &primary);
            return;
        }

        // A sample that must be backward Euler (latched, or a breakpoint)
        // skips the trapezoidal solve: its result would be discarded.
        let forced = match (
            ir.solver_config.runtime_be_latch,
            ir.solver_config.breakpoint_be,
        ) {
            (true, true) => Some("state.be_latched || state.breakpoint_be > 0"),
            (true, false) => Some("state.be_latched"),
            (false, true) => Some("state.breakpoint_be > 0"),
            (false, false) => None,
        };
        match forced {
            Some(f) => code.push_str(&format!("    let be_first = {f};\n    if !be_first {{\n")),
            None => code.push_str("    {\n"),
        }
        Self::emit_nodal_rhs(code, ir, noise, &primary, NoiseMode::Draw);
        Self::emit_nodal_newton(code, ir, noise, setter_stamps, &primary);
        code.push_str("    }\n\n");

        let mut cond = "!converged".to_string();
        if active_set_be {
            cond.push_str(" || active_set_engaged");
        }
        if forced.is_some() {
            cond.push_str(" || be_first");
        }
        code.push_str(
            "    // Backward-Euler solve: the same routine a BE build runs (main loop,\n\
             \x20   // sub-step, pin-and-resolve), on the BE matrices and its own chord.\n",
        );
        code.push_str(&format!("    if {cond} {{\n"));
        // Diag contract: be_fallback counts every entry (latched samples too).
        code.push_str("        state.diag_be_fallback_count += 1;\n");
        code.push_str("        chord_valid = false;\n");
        code.push_str("        converged = false;\n");
        code.push_str("        let primary_iters = state.last_nr_iterations;\n");
        code.push_str("        v = state.v_prev;\n");
        code.push_str("        q_be = true;\n        q_sub = None;\n");
        code.push_str("        state.last_nr_iterations = MAX_ITER as u32;\n");
        if ir.solver_config.breakpoint_be {
            code.push_str(
                "        let be_iter_budget = if state.breakpoint_be > 0 { BREAKPOINT_BE_MAX_ITER } else { MAX_ITER };\n",
            );
        }
        emit_chord_cache(code, ir, "state.chord_be_", "        ", true);
        let be = NewtonSite::be_instance(ir);
        let noise_mode = match forced {
            Some(_) => NoiseMode::DrawIf("be_first"),
            None => NoiseMode::Replay,
        };
        Self::emit_nodal_rhs(code, ir, noise, &be, noise_mode);
        Self::emit_nodal_newton(code, ir, noise, setter_stamps, &be);
        emit_chord_cache(code, ir, "state.chord_be_", "        ", false);
        // The max-iter counter reports the sample's primary solve.
        match forced {
            Some(_) => code
                .push_str("        if !be_first { state.last_nr_iterations = primary_iters; }\n"),
            None => code.push_str("        state.last_nr_iterations = primary_iters;\n"),
        }
        code.push_str("    }\n\n");
    }

    /// Step 1 of a full-LU sample: the right-hand side for one integrator
    /// (`site`) into a local `rhs` — the history and inputs
    /// ([`emit_history_rhs`]), runtime sources, noise, saturating-inductor
    /// flux history.
    fn emit_nodal_rhs(
        code: &mut String,
        ir: &CircuitIR,
        noise: &NoiseEmission,
        site: &NewtonSite,
        noise_mode: NoiseMode,
    ) {
        let has_sat_ind = !ir.saturating_inductors.is_empty();
        emit_history_rhs(
            code,
            ir,
            site.rhs_const,
            site.a_neg,
            site.a_neg_sparsity(ir),
            !site.be,
        );
        // NOTE: `state.input_prev` is deliberately NOT committed here. The
        // adaptive sub-stepping below interpolates the input ramp as
        // `(input - state.input_prev) / subdiv`, so committing before the
        // sub-steps read it would zero the ramp exactly on the hard-transient
        // samples that trigger sub-stepping. The commit happens in the
        // end-of-sample state-update block (matching the DK template).
        code.push('\n');

        // Runtime voltage sources (.runtime directive): host-driven per-sample values.
        if !ir.runtime_sources.is_empty() {
            code.push_str("    // Runtime voltage sources (.runtime directive)\n");
            for rt in &ir.runtime_sources {
                code.push_str(&format!(
                    "    rhs[{}] += state.{};\n",
                    rt.vs_row, rt.field_name
                ));
            }
            code.push('\n');
        }

        // Authentic circuit noise — Phase 1 thermal stamp.
        // One draw per audio sample, before NR (and before the M=0 direct
        // LU solve). See `emit_nodal_schur_process_sample` for the full
        // rationale; the placement is identical here.
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

        // Saturating-inductor history: swap the baked-in linear flux
        // `alpha·L0·i_prev` (already in rhs via A_neg·v_prev) for `alpha·Φ(i_prev)`.
        if has_sat_ind {
            code.push_str(
                "    // Saturating inductor flux history (Φ(i_prev) replaces L0·i_prev)\n",
            );
            emit_sat_ind_history(code, ir, "rhs", "state.v_prev", &site.sat_alpha, "    ");
            code.push('\n');
        }
    }

    /// The death-spiral hold, one rule for both nodal routes: when every
    /// Newton path failed (the solve, the sub-step ladder and, on a
    /// trapezoidal build, the backward-Euler solve with its own ladder), the
    /// sample commits the PREVIOUS state, not the diverged iterate, and
    /// `diag_nr_hold_count` counts it. `full_lu` adds the invalidation of the
    /// cross-sample chord factorisations, which only full-LU keeps. Emitted
    /// wherever a Newton solve can end unsolved; see [`emits_hold`].
    pub(super) fn emit_death_spiral_hold(code: &mut String, ir: &CircuitIR, full_lu: bool) {
        if !emits_hold(ir) {
            return;
        }
        code.push_str("    if !converged {\n");
        if full_lu {
            code.push_str(
                "        // NR failed on all paths — keep previous state, invalidate chord.\n",
            );
        } else {
            code.push_str("        // NR failed on all paths — keep previous state.\n");
        }
        code.push_str(
            "        // THIS SAMPLE IS NOT A SOLUTION. It is bounded and smooth, so no\n\
             \x20       // level measurand can see it; diag_nr_hold_count is the witness.\n",
        );
        code.push_str("        state.diag_nr_hold_count += 1;\n");
        code.push_str("        state.diag_unsolved_sample_count += 1;\n");
        code.push_str("        v = state.v_prev;\n");
        if carries_q_dot(ir) {
            code.push_str("        q_sub = Some(state.q_dot);\n");
        }
        code.push_str("        i_nl = state.i_nl_prev;\n");
        if full_lu {
            code.push_str("        chord_valid = false;\n");
            if has_be_instance(ir) {
                code.push_str("        state.chord_be_valid = false;\n");
            }
        }
        code.push_str("    }\n\n");
    }
}
