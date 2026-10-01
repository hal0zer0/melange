//! The full-LU Newton iteration.

use super::behavioral::{
    behavioral_convergence_nodes, behavioral_damp_skip_literal, emit_behavioral_evals,
    emit_behavioral_jacobian, emit_behavioral_rhs,
};
use super::residual::{emit_armijo_line_search, emit_sparse_nv_matvec};
use super::sat_ind::{
    emit_sat_ind_companion, emit_sat_ind_jacobian, emit_sat_ind_row_residual,
    emit_sat_ind_step_limit,
};
use super::sites::{NewtonSite, PinSite};
use super::stamps::{
    emit_body_gmb_companion, emit_body_gmb_stamp, emit_hard_rail_clamp, emit_nodal_companion_rhs,
    emit_nodal_jacobian_stamp,
};
use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::dk_emitter::NoiseEmission;
use crate::codegen::rust_emitter::helpers::body_effect_mosfets;
use crate::codegen::rust_emitter::RustEmitter;

impl RustEmitter {
    /// Step 2 of a full-LU sample, for one integrator (`site`): the Newton
    /// loop, the adaptive sub-step, and the op-amp pin-and-resolve. The BE
    /// build, a trapezoidal build's BE instance (latch, fallback, breakpoint)
    /// and the trapezoidal solve are all this one routine; they differ only in
    /// `site`. Writes the caller's `v`, `converged`, `i_nl`, chord locals.
    pub(super) fn emit_nodal_newton(
        code: &mut String,
        ir: &CircuitIR,
        noise: &NoiseEmission,
        setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
        site: &NewtonSite,
    ) {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let has_behavioral = !ir.behavioral_sources.is_empty();
        let has_sat_ind = !ir.saturating_inductors.is_empty();
        let use_line_search = m > 0;
        let active_set_be_mode_full_lu = matches!(
            ir.solver_config.opamp_rail_mode,
            crate::codegen::OpampRailMode::ActiveSetBe
        );
        // The chord (a reused LU) is emitted unless the Jacobian must be
        // refactored every iteration anyway.
        let chord = !(has_behavioral || has_sat_ind);
        if chord {
            code.push_str("    let mut exit_step = false;\n");
        }
        // Trapezoidal NR loop
        code.push_str(&format!("    for iter in 0..{} {{\n", site.iter_budget));

        // 2a. Extract nonlinear voltages: v_nl = N_v * v (sparse)
        code.push_str("        // 2a. Extract nonlinear voltages: v_nl = N_v * v (sparse)\n");
        code.push_str("        let mut v_nl = [0.0f64; M];\n");
        for i in 0..m {
            let nz_cols = &ir.sparsity.n_v.nz_by_row[i];
            if nz_cols.is_empty() {
                continue;
            }
            let terms: Vec<String> = nz_cols
                .iter()
                .map(|&j| format!("N_V[{}][{}] * v[{}]", i, j, j))
                .collect();
            code.push_str(&format!("        v_nl[{}] = {};\n", i, terms.join(" + ")));
        }
        code.push('\n');

        // 2b. Evaluate device currents and Jacobian
        code.push_str("        // 2b. Evaluate device currents and Jacobian (block-diagonal)\n");
        // i_nl is declared in outer scope (line above NR loop), j_dev is per-iteration
        code.push_str("        let mut j_dev = [0.0f64; M * M];\n");
        Self::emit_nodal_device_evaluation_body(code, ir, "        ", "v");
        code.push('\n');

        // Behavioral B-source value + partials at the current iterate.
        // Used by both the Jacobian (2c) and companion RHS (2d) below.
        if has_behavioral {
            code.push_str("        // Behavioral B-source evaluation (value + partials)\n");
            emit_behavioral_evals(code, ir, "        ");
            code.push('\n');
        }

        // 2c. Build and factor Jacobian: G_aug = A - N_i * J_dev * N_v (chord method)
        // Factor LU periodically: iter 0, then every CHORD_REFACTOR iterations.
        // Between refactors, reuse stored LU for O(N²) back-solve instead of O(N³) factor.
        // J_dev is block-diagonal; N_i and N_v have ~2 nonzero entries per device dim.
        // Compile-time unrolled: ~64 stamps for 4 tubes vs 107K dense iterations.
        code.push_str(
            "        // 2c. Build and factor Jacobian (adaptive chord: reuse across timesteps)\n",
        );
        // Refactor when: (a) no valid LU, (b) within-sample periodic
        // refresh, OR (c) any device's diagonal Jacobian has diverged
        // from `chord_j_dev` by >50 % relative.
        //
        // The cross-timestep chord persistence holds `chord_j_dev`
        // frozen across many samples. That's fine while a device
        // conducts smoothly, but a stale chord across an exponential
        // knee is device-generic: any diode/junction crossing from
        // reverse- to forward-bias moves `j_dev` by many orders of
        // magnitude between refactors (the extreme case is the Boyle
        // catch diode: ≈1e-31 S deeply reverse-biased vs ≈1e1 S at
        // the rail — 32 OOM). Without the adaptive trigger, refactors
        // only happen at iter % 5 == 0 / iter >= 10, by which point
        // pnjlim has damped the steps so far that the chord can't
        // catch up within MAX_ITER.
        //
        // The trigger is emitted unconditionally (was BoyleDiodes-only):
        // it only compares the already-computed per-iteration `j_dev`
        // diagonal against the persisted `chord_j_dev` — O(M) compares,
        // no extra device evaluations — and for smoothly-conducting
        // devices it simply never fires.
        //
        // The 50 % threshold is empirical: lower (e.g. 20 %) causes
        // spurious refactors that trap pnjlim's step damping in a
        // different slow-oscillation regime; higher (e.g. 80 %)
        // misses the diode-knee transition.
        if has_behavioral || has_sat_ind {
            // Behavioral B-source and saturating-inductor Jacobian entries
            // change every iteration (nonlinear, not part of the frozen chord
            // device block `chord_j_dev`), so refactor each iteration — this
            // keeps the inductor Jacobian (stamped below) consistent with the
            // companion RHS, both evaluated at the current iterate `v`.
            // Correctness over the chord's per-sample speedup (perf: Phase 1
            // deferred an L_diff-drift refactor trigger, §3.4).
            code.push_str("        let need_refactor = true;\n");
        } else {
            code.push_str("        let mut need_refactor = exit_step || !chord_valid || (iter > 0 && iter % CHORD_REFACTOR == 0) || iter >= 10;\n");
            code.push_str("        if !need_refactor {\n");
            code.push_str("            for k in 0..M {\n");
            code.push_str("                let jk = j_dev[k * M + k];\n");
            code.push_str("                let ck = chord_j_dev[k * M + k];\n");
            code.push_str("                let mx = jk.abs().max(ck.abs());\n");
            code.push_str("                if mx > 1e-20 && (jk - ck).abs() / mx > 0.5 {\n");
            code.push_str("                    need_refactor = true;\n");
            code.push_str("                    break;\n");
            code.push_str("                }\n");
            code.push_str("            }\n");
            code.push_str("        }\n");
        }
        code.push_str("        if need_refactor {\n");
        code.push_str("            chord_j_dev = j_dev;\n");
        code.push_str(&format!("            chord_lu = {};\n", site.a));
        {
            // Build transpose of N_i sparsity: for each device dim i, which nodes a are nonzero
            emit_nodal_jacobian_stamp(code, ir, m, "chord_lu", "            ");
        }
        // MOSFET body effect: gmb is part of the factored chord, so it is
        // frozen with it and the companion below uses the same values.
        if !body_effect_mosfets(ir).is_empty() {
            code.push_str("            chord_body_gmb = body_gmb;\n");
            emit_body_gmb_stamp(code, ir, "chord_lu", "chord_body_gmb", "            ");
        }
        // Saturating-inductor Jacobian: alpha·L0 → alpha·L_diff(v[k]) at [k][k].
        if has_sat_ind {
            emit_sat_ind_jacobian(code, ir, "chord_lu", "v", &site.sat_alpha, "            ");
        }
        // Behavioral B-source Jacobian: stamp ∂f/∂V directly into G_aug.
        if has_behavioral {
            emit_behavioral_jacobian(code, ir, "            ");
        }
        // Factor: try sparse LU (if available). The sparse schedule uses
        // STATIC (symbolic) pivoting, which can fail at runtime — a pivot
        // numerically near zero, or excessive element growth (checked
        // inside sparse_lu_factor). On rejection, re-factor DENSE with
        // partial pivoting in the same NR iteration on the saved stamped
        // G_aug; `chord_dense` records which factorization the persisted
        // chord_lu holds so back-solves (this sample and later ones via
        // cross-timestep persistence) dispatch correctly.
        if ir.sparsity.lu.is_some() {
            code.push_str("            let g_aug_stamped = chord_lu;\n");
            code.push_str(
                "            if sparse_lu_factor(&mut chord_lu, &mut chord_dr, &mut chord_dc) {\n",
            );
            code.push_str("                chord_dense = false;\n");
            code.push_str("            } else {\n");
            code.push_str(
                "                // Sparse factor rejected (tiny pivot or growth-factor check):\n",
            );
            code.push_str("                // re-factor DENSE with partial pivoting in this same iteration.\n");
            code.push_str("                chord_lu = g_aug_stamped;\n");
            // On failure, break with `converged` false; the pessimistic
            // `last_nr_iterations = MAX_ITER` init means the post-loop
            // diagnostic counts this sample exactly once (no direct
            // increment here — it would double-count).
            code.push_str("                if !lu_factor(&mut chord_lu, &mut chord_dr, &mut chord_dc, &mut chord_perm) {\n");
            code.push_str("                    break;\n");
            code.push_str("                }\n");
            code.push_str("                chord_dense = true;\n");
            code.push_str("            }\n");
        } else {
            code.push_str(
                    "            if !lu_factor(&mut chord_lu, &mut chord_dr, &mut chord_dc, &mut chord_perm) {\n",
                );
            code.push_str("                break;\n");
            code.push_str("            }\n");
        }
        code.push_str("            chord_valid = true;\n");
        code.push_str("            state.diag_refactor_count += 1;\n");
        code.push_str("        }\n\n");

        // 2d. Build companion RHS: rhs_base + N_i * (i_nl - J_dev_0 * v_nl) (sparse)
        // Uses chord_j_dev (from iter 0) to match the LU factorization.
        // Using current j_dev here would create an inconsistent fixed point.
        code.push_str(
                "        // 2d. Build companion RHS: rhs + N_i * (i_nl - chord_J_dev * v_nl) (sparse)\n",
            );
        code.push_str("        let mut rhs_work = rhs;\n");
        emit_nodal_companion_rhs(code, ir, m, "rhs_work", "chord_j_dev", "        ");
        emit_body_gmb_companion(code, ir, "rhs_work", "chord_body_gmb", "v", "        ");
        // Saturating-inductor companion current (consistent with the Jacobian
        // stamp above — same iterate `v`, re-factored every iteration).
        if has_sat_ind {
            emit_sat_ind_companion(code, ir, "rhs_work", "v", &site.sat_alpha, "        ");
        }
        // Behavioral B-source companion current: f(v) - Σ (∂f/∂V)·v.
        if has_behavioral {
            emit_behavioral_rhs(code, ir, "        ");
        }
        code.push('\n');

        // 2e. Solve with stored LU factors (O(N²) back-solve)
        if ir.sparsity.lu.is_some() {
            code.push_str(
                "        // 2e. Back-solve with stored LU factors (sparse, or the dense\n",
            );
            code.push_str(
                "        // runtime fallback when the sparse factorization was rejected)\n",
            );
            code.push_str("        let mut v_new = rhs_work;\n");
            code.push_str("        if chord_dense {\n");
            code.push_str("            lu_back_solve(&chord_lu, &chord_dr, &chord_dc, &chord_perm, &mut v_new);\n");
            code.push_str("        } else {\n");
            code.push_str(
                "            sparse_lu_back_solve(&chord_lu, &chord_dr, &chord_dc, &mut v_new);\n",
            );
            code.push_str("        }\n\n");
        } else {
            code.push_str(
                "        // 2e. Solve with stored LU factors (chord: O(N²) back-solve)\n",
            );
            code.push_str("        let mut v_new = rhs_work;\n");
            code.push_str(
                    "        lu_back_solve(&chord_lu, &chord_dr, &chord_dc, &chord_perm, &mut v_new);\n\n",
                );
        }

        // Op-amp output clamping inside NR loop — prevents physically impossible
        // voltages that destabilize downstream device evaluation. Applied after the
        // back-solve, before device voltage limiting.
        //
        // Emitted for `Hard` mode ONLY. This clamp originally ran for
        // every mode with clampable op-amps (b5c5011 death-spiral
        // protection), but that contradicted the mode contracts:
        //   * `None` must leave the output unbounded (raw-diagnostic use);
        //   * `ActiveSet`/`ActiveSetBe` deliberately skip the in-NR clamp
        //     so NR converges to the unconstrained solution and the
        //     post-convergence pin-and-resolve sees a genuine violation
        //     (the clamp pinned v EXACTLY at the rail, so the resolve's
        //     detection never fired and ActiveSet degraded to Hard
        //     semantics on this path);
        //   * `BoyleDiodes` models saturation with physical catch diodes —
        //     clamping on top double-limits.
        // Non-Hard modes keep their divergence protection from the global
        // node damping (damp_thresh), SPICE device voltage limiting, the
        // residual check, the substep retry, the BE fallback, and the
        // failed-sample state-keep below.
        //
        // For `ActiveSetBe` we used to ALSO raise `active_set_engaged = true`
        // here so the post-convergence check didn't miss a "v pinned exactly at
        // rail" case (the check used strict inequality). That was load-bearing
        // for 4kbuscomp but sticky — intermediate NR iterations can ride AOL
        // into rail-range values mid-solve on high-gain feedback clippers
        // (pipe-shouter at Tone=1.0 was seeing the flag latched for entire
        // linear-regime buffers, firing BE fallback on every sample), crushing
        // H2 purity and dropping gain 5–15 dB. The post-convergence check now
        // uses inclusive `>=` / `<=` (see `emit_nodal_active_set_check`), which
        // covers the 4kbuscomp pinned-at-rail case without tracking per-NR-
        // iteration transients.
        if matches!(
            ir.solver_config.opamp_rail_mode,
            crate::codegen::OpampRailMode::Hard
        ) {
            emit_hard_rail_clamp(
                code,
                ir,
                "v_new",
                "        ",
                Some("        // Per-iteration op-amp output rail clamp (Hard mode)\n"),
                true,
            );
        }

        // 2f. SPICE-style voltage limiting + global node damping
        code.push_str("        // 2f. SPICE voltage limiting + node damping\n");
        code.push_str("        let mut alpha = 1.0_f64;\n\n");

        // Layer 1: Device voltage limiting
        code.push_str("        // Layer 1: SPICE device voltage limiting\n");
        Self::emit_nodal_voltage_limiting(code, ir);
        code.push('\n');

        // Layer 2: Global node voltage damping (adaptive threshold).
        // Behavioral V={} output nodes are algebraically forced (V=f), so
        // they're excluded — a legit large value (the ddt discriminator
        // startup spike) must not throttle every other node's step.
        code.push_str("        // Layer 2: Global node voltage damping\n");
        code.push_str("        {\n");
        let damp_skip = if has_behavioral {
            let lit = behavioral_damp_skip_literal(ir);
            code.push_str(&format!("            let damp_skip = {lit};\n"));
            "if damp_skip.contains(&i) { continue; } "
        } else {
            ""
        };
        code.push_str("            let mut max_node_dv = 0.0_f64;\n");
        code.push_str(&format!(
            "            for i in 0..{} {{\n\
                 \x20               {}let dv = alpha * (v_new[i] - v[i]);\n\
                 \x20               max_node_dv = max_node_dv.max(dv.abs());\n\
                 \x20           }}\n",
            n_nodes, damp_skip
        ));
        // Adaptive threshold: max(10V, 5% of max node voltage)
        code.push_str("            let mut max_v = 0.0_f64;\n");
        code.push_str(&format!(
            "            for i in 0..{} {{ {}max_v = max_v.max(v[i].abs()); }}\n",
            n_nodes, damp_skip
        ));
        code.push_str("            let damp_thresh = 10.0_f64.max(max_v * 0.05);\n");
        code.push_str("            if max_node_dv > damp_thresh {\n");
        // No `.max(0.01)` floor here (removed 2026-08-03): flooring the
        // ratio bounds how much the RAW step gets shrunk BY, not what
        // the resulting damped step IS. When a companion-model LU solve
        // produces a pathologically large raw delta (observed: 3.8e7 V
        // at a device-state transition on wurli-power-amp), a 1% floor
        // still lets a catastrophic multiple of `damp_thresh` through
        // (0.01 * 3.8e7 =~ 380,000 V, not <=10 V), launching the
        // trajectory into a nonphysical regime the remaining NR
        // iterations can't recover from. Uncapped division keeps the
        // worst-case per-iteration node step at exactly `damp_thresh`
        // regardless of the raw delta's magnitude. Regression:
        // nodal_be_fallback_alpha_floor_tests.rs.
        code.push_str("                alpha *= damp_thresh / max_node_dv;\n");
        code.push_str("            }\n");
        code.push_str("        }\n\n");

        // Layer 3: saturating-inductor branch-current limit (all four
        // Newton sites carry it; see `emit_sat_ind_step_limit`).
        emit_sat_ind_step_limit(code, ir, "v", "v_new", "alpha", "        ");

        // Armijo backtracking line search (trap site): scales the already-
        // limited step along the ray v -> v + alpha*(v_new-v) to enforce a
        // monotone node-KCL residual decrease. On a non-descent direction the
        // search fails (ls_ok=false); the loop then takes the un-line-searched
        // pnjlim/node-damping-limited step (alpha keeps its limiter value; the
        // `alpha *= s` scaling only runs on acceptance) and CONTINUES — the
        // always-checked residual gate below, not a bail, decides convergence.
        // A line search may only help; its failure must be no worse than not
        // having it (design review). `limited` records whether pnjlim or
        // node-damping shrank alpha below 1, gating the voltage-step check (a
        // limited step's small size is not evidence of convergence — only the
        // residual gate is). Left as `alpha < 1.0` so a fall-through's natural
        // value is used unchanged (do NOT force it true — that would SKIP the
        // step check and be more permissive, the opposite of intended).
        if use_line_search {
            emit_armijo_line_search(
                code, "        ", "v", "v_new", "alpha", site.a, "rhs", "ls_ok", "i_nl",
            );
            code.push_str("        if !ls_ok { state.diag_ls_fail_count += 1; }\n");
            code.push_str("        let limited = alpha < 1.0;\n");
        }

        // Apply damped Newton step and check convergence
        // Compute step BEFORE updating v, so convergence check sees the actual delta
        // (the check runs every iteration, including iter 0 — a zero-step
        // first iteration is a legitimate converged warm start).
        // Convergence check on nonlinear device nodes only (N_V nonzero columns).
        // Linearized stages respond linearly and converge passively — checking
        // all N nodes causes spurious chord refactors when linear coupling
        // ripple from nonlinear stages hasn't settled to sub-µV precision.
        code.push_str("        // Compute damped step, check convergence, then apply\n");
        code.push_str("        let mut max_step_exceeded = false;\n");
        // DK/Schur convergence contract: the voltage-step check is only
        // meaningful when the step was NOT limited this iteration. When it
        // was, the step's small size is an artifact of damping, not
        // convergence — the always-checked ||F|| residual gate decides.
        if use_line_search {
            code.push_str("        if !limited {\n");
        }
        {
            let mut device_nodes: Vec<usize> = ir
                .sparsity
                .n_v
                .nz_by_row
                .iter()
                .flat_map(|row| row.iter().copied())
                .collect();
            // Behavioral B-sources are often M=0 (no N_v rows); include their
            // terminal + referenced nodes so the check isn't vacuously true.
            device_nodes.extend(behavioral_convergence_nodes(ir));
            // Saturating inductors: check the augmented branch-current row so
            // NR actually iterates on the flux nonlinearity. Essential at M=0,
            // where the N_v/behavioral node set is empty and the step check
            // would otherwise be vacuously "converged" on iteration 0.
            for si in &ir.saturating_inductors {
                device_nodes.push(si.aug_row);
            }
            device_nodes.sort();
            device_nodes.dedup();
            let extra = if use_line_search { "    " } else { "" };
            for &node in &device_nodes {
                code.push_str(&format!(
                        "        {extra}{{ let step = alpha * (v_new[{node}] - v[{node}]); let threshold = 1e-3 * v[{node}].abs().max((v[{node}] + step).abs()) + 1e-6; if !(step.abs() < threshold) {{ max_step_exceeded = true; }} }}\n"
                    ));
            }
        }
        if use_line_search {
            code.push_str("        }\n");
        }
        code.push_str("        for i in 0..N { v[i] += alpha * (v_new[i] - v[i]); }\n");

        // Mid-NR op-amp output clamping (VCC/VEE).
        //
        // In `Hard` and `None` modes we either apply the clamp every
        // iteration (Hard) or do nothing (None). In `ActiveSet` mode the
        // in-NR clamp is SKIPPED — NR converges to whatever the linear
        // system dictates, then a post-convergence constrained resolve
        // pins the rail-violating nodes and re-solves the network to a
        // KCL-consistent state (see emit_nodal_active_set_resolve).
        //
        // The reason we skip the mid-NR clamp for ActiveSet: the hard
        // clamp writes `v_clamped` back into `v` but the rest of the
        // solve is consistent with `v_unclamped`. That inconsistency
        // corrupts the trapezoidal cap history on the next sample. The
        // active-set resolve rebuilds the full v so KCL is satisfied at
        // every node given the clamped value — no corruption.
        //
        // BoyleDiodes mode will eventually make this block unreachable
        // (rail saturation will be modeled by nonlinear catch diodes in
        // J_dev), but until that lands we still fall through to Hard
        // semantics if requested.
        //
        // Step 4 fix: when we do apply the clamp, re-check convergence
        // against the POST-clamp v so a silent mutation of v[out] can't
        // be reported as "converged". The previous code computed the
        // step magnitude against the unclamped v[out] + step — if
        // clamping then shifted v[out] by a significant amount, NR would
        // still report converged because `max_step_exceeded` was computed
        // on the pre-clamp step. This check catches that case.
        use crate::codegen::OpampRailMode;
        // BoyleDiodes mode has already inserted physical catch diodes into
        // the MNA; the NR solve naturally drives v[out] toward the rails
        // via those diodes. Emitting a post-NR hard clamp on top would
        // double-limit and re-introduce the KCL corruption we're trying
        // to avoid. So BoyleDiodes skips the in-NR clamp entirely, same
        // as ActiveSet.
        let emit_in_nr_clamp = matches!(ir.solver_config.opamp_rail_mode, OpampRailMode::Hard);
        if emit_in_nr_clamp && !ir.opamps.is_empty() {
            for oa in &ir.opamps {
                let target = format!("v[{}]", oa.n_out_idx);
                let Some(stmt) = Self::rail_clamp_stmt(&target, oa.vclamp_lo, oa.vclamp_hi) else {
                    continue;
                };
                code.push_str(&format!(
                        "        {{\n\
                         \x20           let v_pre_clamp = v[{idx}];\n\
                         \x20           {stmt}\n\
                         \x20           // Post-clamp convergence re-check: if the clamp mutated\n\
                         \x20           // v[{idx}] meaningfully, the iteration hasn't really\n\
                         \x20           // converged — the rest of the network is still consistent\n\
                         \x20           // with the pre-clamp value.\n\
                         \x20           let clamp_delta = (v[{idx}] - v_pre_clamp).abs();\n\
                         \x20           let clamp_thresh = 1e-3 * v[{idx}].abs().max(v_pre_clamp.abs()) + 1e-6;\n\
                         \x20           if clamp_delta >= clamp_thresh {{ max_step_exceeded = true; }}\n\
                         \x20       }}\n",
                        idx = oa.n_out_idx,
                    ));
            }
        }

        // Residual-based convergence safety net (unconditional whenever a
        // chord is in use — i.e. always on this path when M > 0).
        //
        // The voltage-step check above (`max_step_exceeded`) declares
        // convergence whenever the damped Newton step is small. That
        // criterion is necessary but NOT sufficient when the chord
        // persistence holds a stale Jacobian: the LU back-solve uses
        // `chord_j_dev` while the actual device current at the new
        // operating point is `i_dev(v_nl_new)`. If `chord_j_dev` is
        // grossly out of date, the LU's "fixed point" is a non-physical
        // state where KCL is satisfied for the LINEARISED network but
        // not for the actual nonlinear devices — NR happily reports
        // converged on a wildly wrong v. A stale chord across an
        // exponential knee is device-generic (any diode/junction
        // crossing reverse→forward moves j_dev by many OOM between
        // refactors); the historical extreme is the Boyle catch diode
        // (≈1e-31 S reverse-biased vs ≈1e+1 S at the rail, 32 OOM),
        // and 4kbuscomp's ActiveSetBe precision rectifiers hit the
        // same class — which is why this used to be rail-mode-gated
        // (BoyleDiodes, later + ActiveSet/ActiveSetBe) and is now
        // emitted unconditionally, matching the DK path, whose
        // residual check has always been ungated (see
        // `nr_helpers.rs::emit_nr_limit_and_converge`, "Current
        // residual check: always").
        //
        // Cost: the block only runs on the ACCEPTING iteration (inside
        // `if !max_step_exceeded`), so it adds M device evaluations
        // once per sample, not per NR iteration.
        //
        // Mismatch ⇒ NR keeps iterating, triggering the adaptive
        // >50 %-j_dev refactor above, the periodic chord refactor, the
        // sub-step retry, or the BE fallback — any of which can break
        // out of the stale-chord trap.
        //
        // Implementation note: the device-evaluation helper writes
        // into local `v_nl`, `i_nl`, `j_dev` arrays of fixed names.
        // We use a nested `{ }` block so Rust shadows those bindings
        // — the throwaway `j_dev` inside the block doesn't disturb
        // the chord-stamped `j_dev` in the outer scope.
        if m > 0 {
            code.push_str(
                "        // Residual check: re-evaluate i_nl at the post-step v and force\n",
            );
            code.push_str(
                "        // NR to keep iterating if the chord linearisation produced an\n",
            );
            code.push_str(
                "        // inconsistent fixed point. Required for any topology where the\n",
            );
            code.push_str("        // per-iteration op-amp rail clamp + stale chord_j_dev can\n");
            code.push_str("        // combine to produce false convergence (voltage step small,\n");
            code.push_str("        // but device KCL residual huge).\n");
            // #2: hold the residual-check device currents so the converged
            // branch can reuse them (same v ⇒ identical i_nl) instead of a
            // second device evaluation.
            code.push_str("        let mut i_nl_resid = [0.0f64; M];\n");
            code.push_str("        if !max_step_exceeded {\n");
            code.push_str("            let i_nl_chord = i_nl;\n");
            // `_final` (i_nl-only, no `j_dev`) rather than the full
            // per-iteration evaluation body: this residual check only
            // ever consumes `i_nl_resid` below, never a Jacobian. The
            // full body unconditionally emits `j_dev[...] = jac[...]`
            // writes for every device; declaring `j_dev` here just to
            // discard it produced a genuine dead store (rustc's
            // `unused_assignments` flagged the last device's last
            // entry — e.g. `j_dev[195] = jac[3];` on wurli-power-amp,
            // M=14 — under `-D warnings`).
            code.push_str("            let mut v_nl_final = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(
                ir,
                "v_nl_final",
                "v",
                "            ",
            ));
            code.push_str("            let mut i_nl = [0.0f64; M];\n");
            Self::emit_nodal_device_evaluation_final(code, ir, "            ", "v");
            code.push_str("            i_nl_resid = i_nl;\n");
            code.push_str(
                "            // Tolerance matches DK Schur path: ABSTOL=1e-12, RELTOL=1e-3,\n",
            );
            code.push_str(
                "            // with a 1e-9 floor on the magnitude denominator so devices\n",
            );
            code.push_str(
                "            // with i_nl ≈ 0 don't have an unreachably tight tolerance.\n",
            );
            code.push_str("            for i in 0..M {\n");
            code.push_str("                let r = (i_nl[i] - i_nl_chord[i]).abs();\n");
            code.push_str("                let tol = 1e-3 * i_nl[i].abs().max(i_nl_chord[i].abs()).max(1e-9) + 1e-12;\n");
            // Negated form: a NaN residual must read as NOT converged.
            code.push_str("                if !(r <= tol) {\n");
            code.push_str("                    max_step_exceeded = true;\n");
            code.push_str("                    break;\n");
            code.push_str("                }\n");
            code.push_str("            }\n");
            code.push_str("        }\n\n");
        }

        // Flux-row analogue of the device residual check above. Emitted
        // whether or not `m > 0`: a circuit can carry a saturating
        // inductor with no nonlinear devices at all.
        emit_sat_ind_row_residual(
            code,
            ir,
            setter_stamps,
            "rhs",
            &site.sat_alpha,
            if m > 0 { Some("i_nl_resid") } else { None },
            "v",
            site.a,
            "max_step_exceeded",
            "        ",
        );

        // Node-KCL residual gate (ALWAYS, precondition of the line search):
        // a backtracking search can shrink the damped step toward zero at a
        // stagnation point, so convergence must require the true equation
        // residual ||F|| = ||A·v - rhs - N_i·i_nl|| within tolerance — never
        // the (possibly line-search-damped) step size alone. The device-chord
        // consistency check above is a linearization test, not an equation
        // residual, so it cannot substitute for this.
        if use_line_search {
            // Saving 1: the gate residual is at the committed `v`, where the
            // device-chord check (guarded by the same `!max_step_exceeded`)
            // already evaluated `i_nl_resid = i_nl(N_v·v)`. The mid-NR op-amp
            // clamp (Hard) mutates `v` BEFORE that check, so `i_nl_resid` and
            // this gate see the SAME post-clamp `v` — no eval between them.
            // Reuse it via `kcl_residual_inl` (bit-identical to re-evaluating).
            // `&&` short-circuits, so `i_nl_resid` is only read when it was
            // populated (the `if !max_step_exceeded` branch ran).
            if m > 0 {
                code.push_str(&format!(
                        "        let (_, kcl_ok, kcl_inf) = if max_step_exceeded {{ (f64::INFINITY, false, f64::INFINITY) }} else {{ kcl_residual_inl(&v, &rhs, &{}, &i_nl_resid) }};\n\
                         \x20       let converged_check = !max_step_exceeded && kcl_ok;\n\n",
                        site.a
                    ));
            } else {
                code.push_str(&format!(
                        "        let (_, kcl_ok, kcl_inf) = if max_step_exceeded {{ (f64::INFINITY, false, f64::INFINITY) }} else {{ kcl_residual(&v, &rhs, &{}, state) }};\n\
                         \x20       let converged_check = !max_step_exceeded && kcl_ok;\n\n",
                        site.a
                    ));
            }
        } else {
            code.push_str("        let converged_check = !max_step_exceeded;\n\n");
        }

        // Exit on an exact-Jacobian step. A chord step leaves a KCL residual
        // (J - J_chord)·Δ, first order in the step, which the node-step test
        // cannot see: on a stiff junction row the node tolerance is tens of µA
        // of current. Where no capacitor damps it (a capless nonlinear row),
        // trapezoidal integration carries that residual forward, alternating in
        // sign, until a backward-Euler sample. So a chord-accepted iterate whose
        // node residual is above the row test's absolute floor takes one more
        // Newton step, refactored at the accepted point, and that step is the
        // one accepted.
        //
        // The gate is the row test's own norm and floor: max over node rows of
        // |F_row| > 1e-9 A. Below it there is nothing to carry: 1e-9 A per sample
        // over a ~100-sample rail plateau is ~1e-7 A, under the ~0.4 µA the
        // exact-Jacobian path itself leaves on the witness. So on quiet signal the
        // chord keeps its reuse across samples and pays no refactor. The gate
        // covers node rows (amps) only; saturating-inductor flux rows keep their
        // own residual check. A stopping test on the chord's contraction rate
        // was measured and changes nothing: the accepted chord step is already
        // far inside the node tolerance.
        if chord {
            assert!(
                use_line_search,
                "the chord exit gate reads the node residual, computed when M > 0"
            );
            code.push_str(&format!(
                "        if converged_check && !need_refactor && !exit_step && kcl_inf > 1e-9 && iter + 1 < {} {{\n\
                 \x20           exit_step = true;\n\
                 \x20           continue;\n\
                 \x20       }}\n",
                site.iter_budget
            ));
        }
        code.push_str("        if converged_check {\n");
        code.push_str("            converged = true;\n");
        code.push_str("            state.last_nr_iterations = iter as u32;\n");

        // Final device currents at the converged point. For m > 0 the
        // residual check above already evaluated i_nl at this exact v (v is
        // unchanged since), so reuse it instead of a second device
        // evaluation. Byte-identical: emit_nodal_device_evaluation_body and
        // _final apply the same current expressions to v_nl = N_v·v.
        if m > 0 {
            code.push_str(
                "            // Final device currents: reuse the residual-check eval (same v)\n",
            );
            code.push_str("            i_nl = i_nl_resid;\n");
        } else {
            code.push_str("            // Final device evaluation at converged point\n");
            code.push_str("            let mut v_nl_final = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(
                ir,
                "v_nl_final",
                "v",
                "            ",
            ));
            Self::emit_nodal_device_evaluation_final(code, ir, "            ", "v");
        }

        // ActiveSet (plain): the pin-and-resolve used to be emitted right
        // here, inside the trap NR convergence block — which silently
        // skipped substep-recovered samples. It now runs once after the
        // trap+substep section (see "ActiveSet (plain) — pin and
        // re-solve" below), covering both convergence paths.

        code.push_str("            break;\n");
        code.push_str("        }\n");
        code.push_str("    }\n\n"); // end trapezoidal NR loop

        // Behavioral B-sources are stamped ONLY in the primary trapezoidal NR
        // loop; the adaptive sub-step and BE fallbacks below rebuild the Newton
        // system from the base G/C matrices, so they would solve a SOURCE-LESS
        // network (the B-source dropped) and then falsely report convergence,
        // committing a wrong result and corrupting trap history. For behavioral
        // circuits we therefore omit BOTH fallbacks — a trap-NR failure falls
        // through to the death-spiral state-hold instead. Behavioral circuits are
        // already BE-primary (IntegratorSelection::BeBehavioral), so this drops
        // only a same-scheme BE restart, not an L-stability rescue. This mirrors
        // the DK BE fallback's own companion-magnetics gate (ir/mod.rs:1891-1910).
        // Non-behavioral circuits emit byte-identically. (behavioral + ActiveSetBe
        // full-LU is hard-errored earlier in from_kernel, so no behavioral circuit
        // reaches these blocks needing the BE path for rail resolution.)
        Self::emit_substep_ladder(code, ir, noise, setter_stamps, site.be, true);

        // ActiveSetBe rail engagement check (post-trap, post-substep).
        // Runs on the final v from either the regular NR loop or the
        // substep recovery, whichever produced the converged result. If
        // any op-amp output is railed, set the flag so the BE solve runs
        // the sample (ActiveSetBe: backward Euler on every engaged sample).
        //
        // Plain ActiveSet doesn't take this path — its trap+pin happens
        // inside the NR break block above.
        if active_set_be_mode_full_lu && !site.be {
            code.push_str("    if converged {\n");
            Self::emit_nodal_active_set_check(code, ir, "        ", "active_set_engaged");
            code.push_str("    }\n\n");
        }

        // ActiveSet (plain) — pin and re-solve on the site's matrices. Runs on
        // the final converged v from EITHER the regular trap NR
        // loop or the substep recovery (it used to be emitted inside the
        // trap NR convergence block, which skipped substep-recovered
        // samples). ActiveSetBe takes a different path: detect-only above,
        // BE fallback for the actual resolve.
        // On a backward-Euler solve (the BE build, or the BE instance of a
        // trapezoidal build) ActiveSetBe resolves here too: that is where
        // its pin-and-resolve belongs.
        if matches!(
            ir.solver_config.opamp_rail_mode,
            crate::codegen::OpampRailMode::ActiveSet
        ) || (site.be && active_set_be_mode_full_lu)
        {
            code.push_str("    if converged {\n");
            Self::emit_nodal_active_set_resolve(
                code,
                ir,
                "        ",
                site.a,
                "rhs",
                PinSite::FullLu {
                    sat_alpha: &site.sat_alpha,
                    setter_stamps,
                },
            );
            code.push_str("    }\n\n");
        }
    }
}
