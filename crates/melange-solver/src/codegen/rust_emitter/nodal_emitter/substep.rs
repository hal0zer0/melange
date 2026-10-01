//! The adaptive sub-step ladder shared by the Schur and full-LU solves.

use super::residual::{emit_armijo_line_search, emit_sparse_nv_matvec, row_iter};
use super::sat_ind::{
    emit_sat_ind_companion, emit_sat_ind_history, emit_sat_ind_jacobian, emit_sat_ind_q_dot,
    emit_sat_ind_row_residual, emit_sat_ind_step_limit,
};
use super::sites::live_g_c;
use super::stamps::{
    emit_body_gmb_companion, emit_body_gmb_stamp, emit_hard_rail_clamp, emit_nodal_companion_rhs,
    emit_nodal_jacobian_stamp,
};
use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::dk_emitter::{
    emit_inject_substep_stamp, emit_noise_replay_body, NoiseEmission,
};
use crate::codegen::rust_emitter::helpers::{carries_q_dot, history_zero_row_ranges, kcl_rows};
use crate::codegen::rust_emitter::RustEmitter;

impl RustEmitter {
    /// The adaptive sub-step ladder: when this sample's Newton solve failed,
    /// cut the timestep (2x .. 2^SUBSTEP_MAX_POWER) and re-solve the full
    /// N-dimensional system under the integrator of the solve it rescues
    /// (`be`), from the live G/C. One ladder for both nodal routes, called after
    /// each Newton solve (a Schur build is a reduced form of the same
    /// equations, so its rescue solves them in full). `full_nodal` names the
    /// route, which only decides where the live G/C are held.
    /// Reads the caller's `converged`, `input`, `state`; on success writes `v`,
    /// `i_nl`, `q_sub` (charge state) and sets `converged`.
    pub(super) fn emit_substep_ladder(
        code: &mut String,
        ir: &CircuitIR,
        noise: &NoiseEmission,
        setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
        be: bool,
        full_nodal: bool,
    ) {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let multi_input = ir.solver_config.num_inputs() > 1;
        let has_sat_ind = !ir.saturating_inductors.is_empty();
        let inject_or_tap = ir.solver_config.has_inject_or_tap();
        let use_line_search = m > 0;
        if ir.behavioral_sources.is_empty() {
            // Adaptive sub-stepping: when trapezoidal NR fails, subdivide the timestep
            // and retry with tighter capacitor conductances. This is how ngspice handles
            // positive-feedback circuits (compressor sidechains, oscillators, etc.).
            // Adaptive sub-stepping by LOCAL REFINEMENT: when a sample's Newton
            // solve fails, walk the sample in sub-steps, bisecting only a
            // sub-step that fails and keeping the converged prefix (SPICE-style
            // timestep control), then growing the step back once it is aligned.
            // Time is kept in integer units of T/2^SUBSTEP_MAX_POWER, so a
            // sub-step's end is exact and the input interpolation below reads it
            // as `(step + 1) / subdiv`. Bounded two ways: the finest step
            // (T/2^SUBSTEP_MAX_POWER) and a total of SUBSTEP_BUDGET sub-step
            // attempts per sample. A uniform restart at 2^n (the previous
            // ladder) re-solved the whole sample at every level and never
            // reached a regenerative fold at base rates (design review).
            code.push_str(
                "    // Adaptive sub-stepping: local refinement of the failing sub-step\n",
            );
            code.push_str("    if !converged {\n");
            code.push_str(
                "        let subdiv: u64 = 1u64 << SUBSTEP_MAX_POWER; // time units per sample\n",
            );
            code.push_str("        let mut t_units: u64 = 0;\n");
            code.push_str("        let mut h_pow: u32 = 1; // sub-step = T / 2^h_pow\n");
            code.push_str("        let mut built_pow: u32 = 0;\n");
            code.push_str("        let mut attempts: u32 = 0;\n");
            // Read inside the rebuild before any use; the initial value is dead
            // on builds whose sub-step reads it nowhere else.
            code.push_str(
                "        #[allow(unused_assignments)]\n        let mut alpha_sub = 0.0f64;\n",
            );
            code.push_str("        let mut a_sub = [[0.0f64; N]; N];\n");
            code.push_str("        let mut a_neg_sub = [[0.0f64; N]; N];\n");
            code.push_str("        let mut v_sub = state.v_prev;\n");
            code.push_str("        let mut i_nl_sub = state.i_nl_prev;\n");
            // The charge derivative carried across the sub-steps: each is a
            // full step of its integrator at `alpha_sub`.
            let q_sub_carried = carries_q_dot(ir);
            if q_sub_carried {
                code.push_str("        let mut q_s = state.q_dot;\n");
            }
            if !multi_input {
                code.push_str(
                    "        let input_step = (input - state.input_prev) / subdiv as f64;\n",
                );
            }
            code.push_str("        let mut all_sub_converged = true;\n");
            code.push_str("        while t_units < subdiv {\n");
            code.push_str(
                "            if attempts >= SUBSTEP_BUDGET { all_sub_converged = false; break; }\n\
                 \x20           attempts += 1;\n\
                 \x20           let h_units = subdiv >> h_pow;\n",
            );
            code.push_str("            if built_pow != h_pow {\n");
            // alpha_sub tracks the RUNTIME host rate (× oversampling), not
            // the compile-time codegen rate — a baked literal here made the
            // sub-step matrices inconsistent with the state matrices after
            // any `set_sample_rate` to a non-codegen rate.
            // The sub-step must solve the SAME scheme the build is pinned to.
            // It used to be hard-wired trapezoidal, so a deck that pinned BE
            // got trap sub-steps behind its back — and the sub-step fires at
            // discontinuities, which is exactly where the two schemes differ
            // (measured 3.51 V vs 6.57 V against a BE-pinned reference at the
            // step edge). A silent integrator swap inside the recovery path is
            // the same class of defect as a silent wrong answer (design review).
            if be {
                code.push_str(
                "                // Backward Euler: alpha = 1/dt, matching the pinned scheme.\n                alpha_sub = state.current_sample_rate * OVERSAMPLING_FACTOR as f64 * (1u64 << h_pow) as f64;\n",
            );
            } else {
                code.push_str(
                "                alpha_sub = 2.0 * state.current_sample_rate * OVERSAMPLING_FACTOR as f64 * (1u64 << h_pow) as f64;\n",
            );
            }
            // From the same G/C `rebuild_matrices` reads (the setters'
            // working copies when there are knobs).
            let (g_src, c_src) = live_g_c(ir, full_nodal, "state");
            code.push_str("                // Rebuild A and A_neg at this sub-step\n");
            code.push_str("                for i in 0..N {\n");
            code.push_str("                    for j in 0..N {\n");
            code.push_str(&format!(
                "                        a_sub[i][j] = {g_src}[i][j] + alpha_sub * {c_src}[i][j];\n"
            ));
            // Charge form: the history is alpha*C under both integrators.
            code.push_str(&format!(
                "                        a_neg_sub[i][j] = alpha_sub * {c_src}[i][j];\n"
            ));
            code.push_str("                    }\n");
            code.push_str("                }\n");
            // Zero the algebraic rows (as the baked A_neg does)
            for (lo, hi) in history_zero_row_ranges(ir) {
                code.push_str(&format!(
                    "                for i in {}..{} {{ for j in 0..N {{ a_neg_sub[i][j] = 0.0; }} }}\n",
                    lo, hi
                ));
            }
            // Gmin on A_sub — 1e-12, matching every other Gmin stamp in the
            // nodal emitter (1e-6 was strong enough to skew high-impedance
            // nodes by an audible amount on sub-stepped samples).
            code.push_str("                for i in 0..N_NODES { a_sub[i][i] += 1e-12; }\n");
            code.push_str("                built_pow = h_pow;\n");
            code.push_str("            }\n");
            code.push_str(
                "            // This sub-step ends at unit `step + 1` of `subdiv`.\n\
                 \x20           let step = t_units + h_units - 1;\n\
                 \x20           let v_sub0 = v_sub;\n\
                 \x20           let i_nl_sub0 = i_nl_sub;\n",
            );
            if !multi_input {
                code.push_str(
                    "                let inp_s = state.input_prev + input_step * (step + 1) as f64;\n",
                );
            }
            // Build sub-step RHS
            code.push_str("                // Sub-step RHS\n");
            if ir.has_dc_sources {
                code.push_str("                let mut rhs_s = RHS_CONST;\n");
            } else {
                code.push_str("                let mut rhs_s = [0.0f64; N];\n");
            }
            code.push_str("                for i in 0..N { for j in 0..N { rhs_s[i] += a_neg_sub[i][j] * v_sub[j]; } }\n");
            if q_sub_carried && !be {
                code.push_str("                for i in 0..N { rhs_s[i] += q_s[i]; }\n");
            }
            // Saturating-inductor flux history (sub-step: base v_sub, alpha_sub)
            if has_sat_ind {
                emit_sat_ind_history(code, ir, "rhs_s", "v_sub", "alpha_sub", "                ");
            }
            // The input at the sub-step's end (the charge form's n+1).
            if multi_input {
                code.push_str(
                    "                for k in 0..NUM_INPUTS {\n\
                     \x20                   let step_k = (inputs[k] - state.inputs_prev[k]) / subdiv as f64;\n\
                     \x20                   let inp_s = state.inputs_prev[k] + step_k * (step + 1) as f64;\n\
                     \x20                   rhs_s[INPUT_NODES[k]] += inp_s / INPUT_RESISTANCES[k];\n\
                     \x20               }\n",
                );
            } else {
                code.push_str(
                    "                rhs_s[INPUT_NODE] += inp_s * (1.0 / INPUT_RESISTANCE);\n",
                );
            }
            if inject_or_tap {
                code.push_str(&emit_inject_substep_stamp(
                    ir,
                    "rhs_s",
                    "                ",
                    "subdiv",
                ));
            }
            // Runtime voltage sources: integration-scheme-independent; every
            // from-scratch RHS rebuild must re-stamp them.
            if !ir.runtime_sources.is_empty() {
                code.push_str("                // Runtime voltage sources (.runtime directive)\n");
                for rt in &ir.runtime_sources {
                    code.push_str(&format!(
                        "                rhs_s[{}] += state.{};\n",
                        rt.vs_row, rt.field_name
                    ));
                }
            }
            // Noise replay: this is a from-scratch RHS rebuild, so it must
            // re-stamp the per-source currents the primary `rhs_stamp`
            // already drew and cached this sample. Omitting it dropped the
            // noise for the whole sample while the RNG stream stayed
            // aligned, making the loss invisible to every determinism
            // check (F10). Drawing fresh values here instead would break
            // determinism outright: sub-stepping is signal-dependent, so the
            // stream position would become a function of the audio.
            if noise.enabled {
                code.push_str(
                    "                // Noise replay (cached i_n; consumes no RNG draws).\n",
                );
                code.push_str(&emit_noise_replay_body(
                    noise.replay_counts,
                    "rhs_s",
                    "                ",
                ));
            }
            // Sub-step NR loop
            code.push_str("                let mut sub_converged = false;\n");
            code.push_str("                for _iter in 0..MAX_ITER {\n");
            code.push_str("                    let mut v_nl = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(
                ir,
                "v_nl",
                "v_sub",
                "                    ",
            ));
            code.push_str("                    let mut i_nl = [0.0f64; M];\n");
            code.push_str("                    let mut j_dev = [0.0f64; M * M];\n");
            // Device evaluation
            Self::emit_nodal_device_evaluation_body(code, ir, "                    ", "v_sub");
            code.push('\n');
            // Build G_aug from a_sub
            code.push_str("                    let mut g_s = a_sub;\n");
            emit_nodal_jacobian_stamp(code, ir, m, "g_s", "                    ");
            emit_body_gmb_stamp(code, ir, "g_s", "body_gmb", "                    ");
            if has_sat_ind {
                emit_sat_ind_jacobian(
                    code,
                    ir,
                    "g_s",
                    "v_sub",
                    "alpha_sub",
                    "                    ",
                );
            }
            // Build companion RHS
            code.push_str("                    let mut rhs_w = rhs_s;\n");
            emit_nodal_companion_rhs(code, ir, m, "rhs_w", "j_dev", "                    ");
            emit_body_gmb_companion(
                code,
                ir,
                "rhs_w",
                "body_gmb",
                "v_sub",
                "                    ",
            );
            if has_sat_ind {
                emit_sat_ind_companion(
                    code,
                    ir,
                    "rhs_w",
                    "v_sub",
                    "alpha_sub",
                    "                    ",
                );
            }
            // LU solve
            code.push_str("                    let mut v_new_s = rhs_w;\n");
            code.push_str("                    if !lu_solve(&mut g_s, &mut v_new_s) { break; }\n");
            // Saturating inductors: the node-row step below never looks at a
            // branch-current row, so the flux rows get the main loop's step
            // check and flux-row residual through this flag.
            if has_sat_ind {
                code.push_str("                    let mut sub_step_exceeded = false;\n");
            }
            // Op-amp supply rail clamping (VCC/VEE) in sub-step. Hard mode
            // only — same gating rationale as the trap-loop per-iteration
            // clamp above: None must stay unbounded, ActiveSet/ActiveSetBe
            // rely on their post-convergence pin-and-resolve seeing the
            // genuine (unclamped) violation, and BoyleDiodes saturates via
            // physical catch diodes.
            if matches!(
                ir.solver_config.opamp_rail_mode,
                crate::codegen::OpampRailMode::Hard
            ) {
                emit_hard_rail_clamp(code, ir, "v_new_s", "                    ", None, false);
            }
            // Convergence check + update
            if use_line_search {
                // pnjlim + node damping (parity with the trap/BE loops): the
                // sub-step inner NR was previously RAW undamped Newton. On a
                // stiff junction the raw step overshoots past the fast_exp
                // clamp where the device Jacobian flattens to ~0, making the
                // Newton direction non-descent and defeating the line search.
                // Limiting keeps the iterate where the Jacobian is valid.
                code.push_str("                    let v_new = v_new_s;\n");
                code.push_str("                    let mut alpha = 1.0_f64;\n");
                Self::emit_nodal_voltage_limiting_indented(code, ir, "                    ");
                code.push_str(&format!(
                        "                    {{\n\
                         \x20                       let mut max_node_dv = 0.0_f64;\n\
                         \x20                       for i in 0..{n_nodes} {{ let dv = alpha * (v_new_s[i] - v_sub[i]); max_node_dv = max_node_dv.max(dv.abs()); }}\n\
                         \x20                       let mut max_v = 0.0_f64;\n\
                         \x20                       for i in 0..{n_nodes} {{ max_v = max_v.max(v_sub[i].abs()); }}\n\
                         \x20                       let damp_thresh = 10.0_f64.max(max_v * 0.05);\n\
                         \x20                       if max_node_dv > damp_thresh {{ alpha *= damp_thresh / max_node_dv; }}\n\
                         \x20                   }}\n"
                    ));
                emit_sat_ind_step_limit(
                    code,
                    ir,
                    "v_sub",
                    "v_new_s",
                    "alpha",
                    "                    ",
                );
                emit_armijo_line_search(
                    code,
                    "                    ",
                    "v_sub",
                    "v_new_s",
                    "alpha",
                    "a_sub",
                    "rhs_s",
                    "ls_ok",
                    "i_nl",
                );
                // Fall-through on Armijo failure (see trap site): take the
                // un-line-searched limited step and continue; the residual gate
                // decides convergence. A line search may only help.
                code.push_str("                    if !ls_ok { state.diag_ls_fail_count += 1; }\n");
                code.push_str("                    let mut max_step = 0.0f64;\n");
                code.push_str(&format!(
                    "                    for i in {} {{ let step = v_new_s[i] - v_sub[i]; if step.abs() > max_step {{ max_step = step.abs(); }} }}\n",
                    row_iter(&kcl_rows(ir), "0..N_NODES")
                ));
                for si in &ir.saturating_inductors {
                    let k = si.aug_row;
                    code.push_str(&format!(
                            "                    {{ let step = alpha * (v_new_s[{k}] - v_sub[{k}]); let threshold = 1e-3 * v_sub[{k}].abs().max((v_sub[{k}] + step).abs()) + 1e-6; if !(step.abs() < threshold) {{ sub_step_exceeded = true; }} }}\n"
                        ));
                }
                code.push_str("                    for i in 0..N { v_sub[i] += alpha * (v_new_s[i] - v_sub[i]); }\n");
            } else {
                code.push_str("                    let mut max_step = 0.0f64;\n");
                code.push_str("                    for i in 0..N_NODES {\n");
                code.push_str("                        let step = v_new_s[i] - v_sub[i];\n");
                code.push_str(
                    "                        if step.abs() > max_step { max_step = step.abs(); }\n",
                );
                code.push_str("                    }\n");
                if has_sat_ind {
                    // The flux-row limit scales the step (no line search
                    // here to do it).
                    code.push_str("                    let mut alpha = 1.0_f64;\n");
                    emit_sat_ind_step_limit(
                        code,
                        ir,
                        "v_sub",
                        "v_new_s",
                        "alpha",
                        "                    ",
                    );
                }
                for si in &ir.saturating_inductors {
                    let k = si.aug_row;
                    code.push_str(&format!(
                            "                    {{ let step = alpha * (v_new_s[{k}] - v_sub[{k}]); let threshold = 1e-3 * v_sub[{k}].abs().max((v_sub[{k}] + step).abs()) + 1e-6; if !(step.abs() < threshold) {{ sub_step_exceeded = true; }} }}\n"
                        ));
                }
                if has_sat_ind {
                    code.push_str("                    for i in 0..N { v_sub[i] += alpha * (v_new_s[i] - v_sub[i]); }\n");
                } else {
                    code.push_str("                    v_sub = v_new_s;\n");
                }
            }
            // Re-evaluate devices at the updated v_sub so i_nl_sub is consistent.
            // Uses `_final` variant: reads v_nl_final, writes i_nl (no j_dev update).
            code.push_str("                    // Re-extract i_nl at updated v\n");
            code.push_str("                    let mut v_nl_final = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(
                ir,
                "v_nl_final",
                "v_sub",
                "                    ",
            ));
            Self::emit_nodal_device_evaluation_final(code, ir, "                    ", "v_sub");
            code.push_str("                    i_nl_sub = i_nl;\n");
            // Convergence: raw (undamped) Newton step small AND — the always-
            // checked precondition of the line search — the true node-KCL
            // residual within tolerance, so a line-search-shrunk step can
            // never report a non-root as converged.
            if has_sat_ind {
                emit_sat_ind_row_residual(
                    code,
                    ir,
                    setter_stamps,
                    "rhs_s",
                    "alpha_sub",
                    None,
                    "v_sub",
                    "a_sub",
                    "sub_step_exceeded",
                    "                    ",
                );
            }
            let sat_gate = if has_sat_ind {
                " && !sub_step_exceeded"
            } else {
                ""
            };
            if use_line_search {
                code.push_str(&format!("                    if max_step < TOL + 1e-3{sat_gate} && kcl_residual(&v_sub, &rhs_s, &a_sub, state).1 {{\n"));
            } else {
                code.push_str(&format!(
                    "                    if max_step < TOL + 1e-3{sat_gate} {{\n"
                ));
            }
            code.push_str("                        sub_converged = true;\n");
            code.push_str("                        break;\n");
            code.push_str("                    }\n");
            code.push_str("                }\n"); // end sub-step NR loop
            code.push_str("                if sub_converged {\n");
            if q_sub_carried {
                // Charge form: advance q_dot across this sub-step (trapezoidal
                // alpha_sub*C*dv - q, or the backward-Euler alpha_sub*C*dv).
                let tail = if be { "" } else { " - q_s[i]" };
                code.push_str(&format!(
                        "                for i in 0..N {{ let mut acc = 0.0; for j in 0..N {{ acc += a_neg_sub[i][j] * (v_sub[j] - v_sub0[j]); }} q_s[i] = acc{tail}; }}\n"
                    ));
                emit_sat_ind_q_dot(
                    code,
                    ir,
                    "q_s",
                    "v_sub",
                    "v_sub0",
                    "alpha_sub",
                    "                ",
                );
            }
            code.push_str(
                    "                    t_units += h_units;\n\
                     \x20                   // Grow back once aligned to the coarser grid.\n\
                     \x20                   if h_pow > 1 && t_units % (h_units << 1) == 0 { h_pow -= 1; }\n\
                     \x20               } else {\n\
                     \x20                   // Bisect this sub-step, from its start.\n\
                     \x20                   v_sub = v_sub0;\n\
                     \x20                   i_nl_sub = i_nl_sub0;\n\
                     \x20                   if h_pow >= SUBSTEP_MAX_POWER { all_sub_converged = false; break; }\n\
                     \x20                   h_pow += 1;\n\
                     \x20               }\n",
                );
            code.push_str("            }\n"); // end sub-step loop
            code.push_str("            if all_sub_converged {\n");
            code.push_str("                v = v_sub;\n");
            if q_sub_carried {
                code.push_str("                q_sub = Some(q_s);\n");
            }
            code.push_str("                i_nl = i_nl_sub;\n");
            code.push_str("                converged = true;\n");
            code.push_str("                state.diag_substep_count += 1;\n");
            code.push_str("            }\n");
            code.push_str("    }\n\n"); // end if !converged
        } // end: behavioral circuits omit the adaptive sub-step fallback
    }
}
