//! Op-amp rail handling: m=0 rail handling, active-set resolve / check,
//! slew limit and rail clamps.

use super::residual::emit_sparse_nv_matvec;
use super::sat_ind::{
    emit_sat_ind_companion, emit_sat_ind_jacobian, emit_sat_ind_row_residual,
    emit_sat_ind_step_limit,
};
use super::sites::{pin_failure_is_committed, PinSite};
use super::stamps::{
    emit_body_gmb_companion, emit_body_gmb_stamp, emit_nodal_companion_rhs,
    emit_nodal_jacobian_stamp,
};
use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::RustEmitter;

impl RustEmitter {
    /// True when [`Self::emit_nodal_m0_rail_handling`] will emit code that
    /// mutates `v` for this IR — callers use it to decide between
    /// `let v = …;` and `let mut v = …;` on the linear (M=0) paths.
    pub(super) fn m0_rail_handling_mutates_v(ir: &CircuitIR) -> bool {
        use crate::codegen::OpampRailMode;
        let any_clampable = ir
            .opamps
            .iter()
            .any(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite());
        match ir.solver_config.opamp_rail_mode {
            OpampRailMode::Hard | OpampRailMode::ActiveSet | OpampRailMode::ActiveSetBe => {
                any_clampable
            }
            OpampRailMode::None | OpampRailMode::BoyleDiodes | OpampRailMode::Auto => false,
        }
    }

    /// Emit op-amp supply-rail handling for the linear (M=0) nodal paths.
    ///
    /// Historically the M=0 branches (Schur `v = v_pred` and full-LU direct
    /// LU solve) emitted NO rail handling in ANY mode — a linear circuit with
    /// finite-rail op-amps driven past its rails produced physically
    /// impossible output regardless of the requested mode. This helper emits
    /// the mode-appropriate handling on the final `v`:
    ///
    /// * `Hard` — the plain output clamp. Cheap, and KCL-safe exactly for the
    ///   `AllDcCoupled` topologies the auto-resolver picks Hard for.
    /// * `ActiveSet` — the full pin-and-resolve against `state.a`/`rhs`. On a
    ///   linear circuit this is a single extra dense LU solve per
    ///   rail-engaged sample (no device re-evaluation: M=0).
    /// * `ActiveSetBe` — same pin-and-resolve against the trap matrices. The
    ///   M=0 paths have no BE fallback machinery, so the BE-matrix re-solve
    ///   ActiveSetBe normally uses is unavailable; a codegen-time warning
    ///   records the honest degrade (risk: trap+pin ringing on sustained
    ///   rail engagement into a cap-coupled load).
    /// * `BoyleDiodes` — nothing. Catch-diode augmentation makes every
    ///   clamped op-amp contribute M≥1, so a BoyleDiodes circuit that still
    ///   has M=0 has no clamped op-amps and nothing to do.
    /// * `None` — nothing, by contract.
    ///
    /// The caller must have a mutable `v`, `rhs`, `state`, and (for the
    /// ActiveSet modes) the generated `lu_solve` in scope — the `needs_lu_solve`
    /// gate in `emit_nodal` already covers ActiveSet/ActiveSetBe on the Schur
    /// path, and the full-LU path always emits `lu_solve`.
    pub(super) fn emit_nodal_m0_rail_handling(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
        a: &str,
        site: PinSite<'_>,
    ) {
        use crate::codegen::OpampRailMode;
        let any_clampable = ir
            .opamps
            .iter()
            .any(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite());
        match ir.solver_config.opamp_rail_mode {
            OpampRailMode::Hard => {
                if any_clampable {
                    code.push_str(&format!(
                        "{indent}// Op-amp supply rail clamp (Hard mode, linear circuit)\n"
                    ));
                    for oa in &ir.opamps {
                        let target = format!("v[{}]", oa.n_out_idx);
                        if let Some(stmt) =
                            Self::rail_clamp_stmt(&target, oa.vclamp_lo, oa.vclamp_hi)
                        {
                            code.push_str(&format!("{indent}{stmt}\n"));
                        }
                    }
                    code.push('\n');
                }
            }
            OpampRailMode::ActiveSet => {
                Self::emit_nodal_active_set_resolve(code, ir, indent, a, "rhs", site);
            }
            OpampRailMode::ActiveSetBe => {
                if any_clampable {
                    log::warn!(
                        "Nodal M=0: ActiveSetBe requested but the linear path has no BE \
                         fallback machinery — degrading to the trap-matrix ActiveSet \
                         pin-and-resolve for rail-engaged samples (risk: trap+pin ringing \
                         on sustained rail engagement into cap-coupled loads)"
                    );
                }
                Self::emit_nodal_active_set_resolve(code, ir, indent, a, "rhs", site);
            }
            OpampRailMode::BoyleDiodes => {
                // Catch-diode augmentation adds M≥1 per clamped op-amp, so an
                // M=0 BoyleDiodes circuit has no clamped op-amps — nothing to do.
            }
            OpampRailMode::None => {
                // No clamping — caller accepts unbounded op-amp output.
            }
            OpampRailMode::Auto => {
                unreachable!(
                    "OpampRailMode::Auto should have been resolved in ir::from_mna; \
                     emitter should only see concrete modes"
                );
            }
        }
    }

    /// Emit the active-set constrained-resolve block for a nodal NR path.
    ///
    /// Called after NR has run in the Schur or full-LU nodal paths when an
    /// active-set rail mode is selected. At entry `v` holds the *unclamped*
    /// solution (op-amp outputs possibly outside their rails) and `i_nl` the
    /// device currents at it.
    ///
    /// The resolve:
    ///   1. Detects which op-amp outputs are at or beyond their VCC/VEE rails.
    ///      If none are, it leaves `v` alone — this is also the release test:
    ///      a pin only exists on samples whose unconstrained solution violates
    ///      the rail.
    ///   2. Otherwise runs Newton on the PINNED nonlinear system
    ///      `A·v = rhs + N_i·i_nl(N_v·v)` with each pinned row replaced by
    ///      `v[k] = c_k` (row/column elimination), using `matrix_name`: each
    ///      iteration re-evaluates the devices, stamps `−N_i·J_dev·N_v` and the
    ///      companion current, solves, and applies the same pnjlim/fetlim and
    ///      10 V node-step limits as the full-LU loops.
    ///   3. Commits the pinned solution with `i_nl` re-evaluated at it, so the
    ///      next sample's history terms are KCL-consistent.
    ///   4. Sets `last_nr_iterations` from the pinned solve: converged clears an
    ///      unpinned failure, not converged marks the sample unsolved.
    ///
    /// **Why Newton, not one linear solve.** The pin can move `v[out]` by volts
    /// in a sample, and an output coupling cap passes that step straight to
    /// downstream devices, so the unpinned solve's `i_nl` is not valid at the
    /// pinned voltages. A single linear solve with it frozen is not a solution:
    /// on a single-supply overdrive with a diode clipper after the output cap
    /// it drove the clipper node to −2 V and re-evaluated a reverse diode at
    /// 3.6e9 A (regression: `opamp_railing_regression_tests.rs`).
    ///
    /// If the dense LU solve fails, `v` stays at the unclamped solution and
    /// `diag_nr_max_iter_count` records it.
    ///
    /// # Caller contract
    ///
    /// The caller must have `v` (mutable), `i_nl` (mutable), and a variable
    /// named `rhs_name` in scope. `rhs` is the linear RHS *before* any nonlinear
    /// companion contribution — `A_neg·v_prev + q_dot + rhs_const + input·G_in`
    /// (trapezoidal) or `A_neg_be·v_prev + rhs_const_be + input·G_in` (backward
    /// Euler) — and `matrix_name` (`"state.a"` / `"state.a_be"`) must match the
    /// integrator that produced it.
    /// `site` says which route this is (see [`PinSite`]); a full-LU site also
    /// carries its saturating-inductor alpha and setter stamps.
    pub(super) fn emit_nodal_active_set_resolve(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
        matrix_name: &str,
        rhs_name: &str,
        site: PinSite<'_>,
    ) {
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            ir.topology.n
        };

        // Only clampable op-amps participate — ones with at least one finite rail.
        let clampable: Vec<&crate::codegen::ir::OpampIR> = ir
            .opamps
            .iter()
            .filter(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
            .collect();
        if clampable.is_empty() {
            return;
        }
        let sat = match site {
            PinSite::FullLu {
                sat_alpha,
                setter_stamps,
            } if !ir.saturating_inductors.is_empty() => Some((sat_alpha, setter_stamps)),
            _ => None,
        };
        assert!(
            ir.saturating_inductors.is_empty() || sat.is_some(),
            "active-set resolve emitted without the saturating-inductor alpha"
        );

        code.push_str(&format!(
            "{indent}// --- Active-set op-amp rail resolve ---\n"
        ));
        code.push_str(&format!(
            "{indent}// Pin any rail-violating op-amp outputs and re-solve for\n"
        ));
        code.push_str(&format!(
            "{indent}// a KCL-consistent v (see emit_nodal_active_set_resolve docs).\n"
        ));
        code.push_str(&format!("{indent}{{\n"));

        // Step 1: detect violations. `pinned_N` carries either `Some(rail)` or
        // `None` based on whether v[out] exceeds its range. `any_pinned` short-
        // circuits the whole resolve when nothing needs clamping (the common case).
        //
        // Inclusive comparison (`>=` / `<=`) is deliberate and must stay
        // coherent with `emit_nodal_active_set_check` (same rationale as its
        // c3d3eae fix): if anything upstream — the Hard-seeded DC OP, a
        // previous sample's resolve, or any residual clamp — has left `v[out]`
        // sitting EXACTLY at the rail bit-for-bit, strict `>` / `<` never
        // fires, `any_pinned` stays false, and the KCL-consistent pin-and-
        // resolve silently never runs (historically this degraded ActiveSet
        // to Hard semantics on the full-LU path). Re-resolving a value already
        // at the rail is idempotent, so the inclusive form costs at most one
        // redundant LU solve on exactly-at-rail samples.
        code.push_str(&format!("{indent}    let mut any_pinned = false;\n"));
        for (idx, oa) in clampable.iter().enumerate() {
            // The load line: the unpinned solution exceeds the saturated
            // output's ceiling `limit - R_SAG*I_load` exactly when
            // `w = v_out + R_SAG*I_load` passes the limit (see
            // `opamp_load_line_expr`). Engagement is continuous: the pinned
            // row puts `v_out` on that same line.
            //
            // Emit a violation branch per FINITE rail only — formatting an
            // infinite bound would render the invalid Rust token `-inf`/`inf`
            // for single-supply op-amps (only one of VCC/VEE specified).
            // `clampable` guarantees at least one branch is emitted.
            code.push_str(&format!(
                "{indent}    let w_{idx} = {};\n",
                opamp_load_line_expr(oa, "v")
            ));
            let mut branches = String::new();
            if oa.vclamp_hi.is_finite() {
                branches.push_str(&format!(
                    "if w_{idx} >= {hi:.17e} {{\n\
                     {indent}        any_pinned = true;\n\
                     {indent}        Some({hi:.17e})\n\
                     {indent}    }} else ",
                    hi = oa.vclamp_hi,
                ));
            }
            if oa.vclamp_lo.is_finite() {
                branches.push_str(&format!(
                    "if w_{idx} <= {lo:.17e} {{\n\
                     {indent}        any_pinned = true;\n\
                     {indent}        Some({lo:.17e})\n\
                     {indent}    }} else ",
                    lo = oa.vclamp_lo,
                ));
            }
            code.push_str(&format!(
                "{indent}    let pinned_{idx}: Option<f64> = {branches}{{\n\
                 {indent}        None\n\
                 {indent}    }};\n",
            ));
        }

        code.push_str(&format!("{indent}    if any_pinned {{\n"));
        code.push_str(&format!(
            "{indent}        state.diag_active_set_pin_count += 1;\n"
        ));

        // Steps 2-5: Newton on the PINNED nonlinear system.
        //
        // The pin moves v[out] by up to volts in one sample, and a coupling
        // cap passes that step straight to downstream devices (a capacitor is a
        // short to a step). So the device currents from the unpinned solve are
        // NOT valid at the pinned voltages, and a single linear resolve with
        // them frozen is not a solution: measured on a single-supply overdrive
        // with a diode clipper after the output cap, it swung the clipper node
        // to -2 V and re-evaluated a reverse diode at 3.6e9 A, which the next
        // sample's history carried into a 1e20 A divergence.
        //
        // Each iteration evaluates the devices at the current pinned iterate,
        // stamps their Jacobian and companion current into `matrix_name` (BE
        // matrices for ActiveSetBe, trap for ActiveSet — the caller's choice),
        // eliminates the pinned rows, and solves. Same pnjlim/fetlim limiting
        // and 10 V node-step cap as the full-LU loops. A pinned solve that does
        // not converge marks the sample unsolved (`last_nr_iterations =
        // MAX_ITER`) so the verbs refuse it rather than commit it quietly.
        //
        // Release: a pin applies only when THIS sample's unpinned solution is
        // outside the rail (step 1), so it lets go as soon as the unconstrained
        // solution returns inside.
        let mut device_nodes: Vec<usize> = ir
            .sparsity
            .n_v
            .nz_by_row
            .iter()
            .flat_map(|row| row.iter().copied())
            .collect();
        device_nodes.sort();
        device_nodes.dedup();
        let it = format!("{indent}            ");
        code.push_str(&format!(
            "{indent}        let mut v_pin = v;
"
        ));
        for (idx, oa) in clampable.iter().enumerate() {
            code.push_str(&format!(
                "{indent}        if let Some(c_k) = pinned_{idx} {{ v_pin[{node}] = c_k; }}\n",
                node = oa.n_out_idx
            ));
        }
        // A saturating inductor's current starts from the previous sample, not
        // from the unpinned solve. That solve had the op-amp far past its rail
        // (tens of volts), so it drives the winding deep into saturation, where
        // L_diff is tiny and Newton on the tanh flux law overshoots across the
        // knee and settles into a 2-cycle (measured: an op-amp railing into a
        // 2 mA choke started at 37 mA and alternated -3.1 / +7.0 mA to
        // MAX_ITER). Inductor current is continuous, so the previous sample is
        // the natural start. It does not guarantee convergence: Newton on tanh
        // can still 2-cycle from a start that is far away, as after a large
        // step within one sample. A pin that reaches MAX_ITER is counted below.
        if sat.is_some() {
            for idx in 0..ir.saturating_inductors.len() {
                code.push_str(&format!(
                    "{indent}        v_pin[SAT_IND_{idx}_AUG_ROW] = state.v_prev[SAT_IND_{idx}_AUG_ROW];\n"
                ));
            }
        }
        code.push_str(&format!(
            "{indent}        let mut pin_converged = false;\n\
             {indent}        let mut pin_iters = MAX_ITER as u32;\n\
             {indent}        let mut pin_lu_ok = true;\n\
             {indent}        for _pit in 0..MAX_ITER {{\n"
        ));
        if m > 0 {
            code.push_str(&format!("{it}let mut v_nl = [0.0f64; M];\n"));
            code.push_str(&emit_sparse_nv_matvec(ir, "v_nl", "v_pin", &it));
            code.push_str(&format!("{it}let mut j_dev = [0.0f64; M * M];\n"));
            Self::emit_nodal_device_evaluation_body(code, ir, &it, "v_pin");
        }
        code.push_str(&format!(
            "{it}let mut g_as = {matrix_name};\n\
             {it}let mut rhs_as = {rhs_name};\n"
        ));
        if m > 0 {
            emit_nodal_jacobian_stamp(code, ir, m, "g_as", &it);
            emit_nodal_companion_rhs(code, ir, m, "rhs_as", "j_dev", &it);
            emit_body_gmb_stamp(code, ir, "g_as", "body_gmb", &it);
            emit_body_gmb_companion(code, ir, "rhs_as", "body_gmb", "v_pin", &it);
        }
        // Saturating inductors: Newton on the flux rows too, at this site's
        // alpha, linearised at the pinned iterate. Stamped before pin
        // elimination so an inductor on the op-amp output node moves its
        // column's contribution to the RHS with everything else.
        if let Some((sat_alpha, _)) = sat {
            emit_sat_ind_jacobian(code, ir, "g_as", "v_pin", sat_alpha, &it);
            emit_sat_ind_companion(code, ir, "rhs_as", "v_pin", sat_alpha, &it);
        }
        // Pin: the railed output is its swing limit `c_k` behind `R_SAG`.
        // Its row keeps the node's KCL; the VCCS leaves it (its Gm entries
        // are removed and its output conductance `1/ROUT` becomes `1/R_SAG`)
        // and the limit enters as the source current `c_k/R_SAG`. So the
        // output sits at `c_k - R_SAG*I_load`, on the load line the
        // detection above measured against.
        for (idx, oa) in clampable.iter().enumerate() {
            let node = oa.n_out_idx;
            let mut unstamp = String::new();
            if let Some(np) = oa.n_plus_idx {
                unstamp.push_str(&format!("{it}    g_as[{node}][{np}] += {:.17e};\n", oa.gm));
            }
            if let Some(nm) = oa.n_minus_idx {
                unstamp.push_str(&format!("{it}    g_as[{node}][{nm}] -= {:.17e};\n", oa.gm));
            }
            code.push_str(&format!(
                "{it}if let Some(c_k) = pinned_{idx} {{\n\
                 {unstamp}\
                 {it}    g_as[{node}][{node}] += {dg:.17e};\n\
                 {it}    rhs_as[{node}] += {g_sag:.17e} * c_k;\n\
                 {it}}}\n",
                dg = oa.g_sag - oa.g_out,
                g_sag = oa.g_sag,
            ));
        }
        code.push_str(&format!(
            "{it}let mut v_new = rhs_as;\n\
             {it}if !lu_solve(&mut g_as, &mut v_new) {{ pin_lu_ok = false; break; }}\n\
             {it}let mut alpha = 1.0_f64;\n"
        ));
        if m > 0 {
            Self::emit_nodal_voltage_limiting_indented(code, ir, &it);
        }
        // Unwitnessed by design: this Newton starts from v_prev, so no reachable
        // step crosses the knee and the limit never fires here. It is carried
        // for parity (site-count tripwire in saturation_step_limit_tests.rs);
        // a start that can jump (a predictor) would need a witness.
        if sat.is_some() {
            emit_sat_ind_step_limit(code, ir, "v_pin", "v_new", "alpha", &it);
        }
        code.push_str(&format!(
            "{it}let limited = alpha < 1.0;\n\
             {it}{{\n\
             {it}    let mut max_node_dv = 0.0_f64;\n\
             {it}    for i in 0..{n_nodes} {{ max_node_dv = max_node_dv.max((alpha * (v_new[i] - v_pin[i])).abs()); }}\n\
             {it}    if max_node_dv > 10.0 {{ alpha *= 10.0 / max_node_dv; }}\n\
             {it}}}\n\
             {it}let mut pin_step_exceeded = limited || alpha < 1.0;\n"
        ));
        // Step check over the device nodes and, as in the main loop, every
        // saturating inductor's branch-current row.
        let mut step_rows = device_nodes.clone();
        if sat.is_some() {
            step_rows.extend(ir.saturating_inductors.iter().map(|si| si.aug_row));
            step_rows.sort();
            step_rows.dedup();
        }
        for &node in &step_rows {
            code.push_str(&format!(
                "{it}{{ let step = alpha * (v_new[{node}] - v_pin[{node}]); let threshold = 1e-3 * v_pin[{node}].abs().max((v_pin[{node}] + step).abs()) + 1e-6; if !(step.abs() < threshold) {{ pin_step_exceeded = true; }} }}\n"
            ));
        }
        code.push_str(&format!(
            "{it}for i in 0..N {{ v_pin[i] += alpha * (v_new[i] - v_pin[i]); }}\n"
        ));
        // The flux-row residual, as in the main loop: the step check alone
        // accepts Newton's first iterate, whose remainder the flux integrates.
        // Pinned rows are op-amp output nodes, never flux rows, so every flux
        // row is checked; `rhs_name` is the pre-companion RHS carrying the
        // site's history term.
        if let Some((sat_alpha, stamps)) = sat {
            emit_sat_ind_row_residual(
                code,
                ir,
                stamps,
                rhs_name,
                sat_alpha,
                None,
                "v_pin",
                matrix_name,
                "pin_step_exceeded",
                &it,
            );
        }
        code.push_str(&format!(
            "{it}if !pin_step_exceeded {{ pin_converged = true; pin_iters = _pit as u32; break; }}\n\
             {indent}        }}\n"
        ));
        // Commit the pinned iterate, with i_nl re-evaluated at it, so the
        // committed state (and the q_dot built from it) matches the voltages.
        code.push_str(&format!(
            "{indent}        if pin_lu_ok {{\n\
             {indent}            v = v_pin;\n"
        ));
        if m > 0 {
            code.push_str(&format!("{indent}            {{\n"));
            code.push_str(&format!(
                "{indent}                let mut v_nl_final = [0.0f64; M];\n\
                 {indent}                for i in 0..M {{\n\
                 {indent}                    let mut sum = 0.0;\n\
                 {indent}                    for j in 0..N {{ sum += N_V[i][j] * v[j]; }}\n\
                 {indent}                    v_nl_final[i] = sum;\n\
                 {indent}                }}\n",
            ));
            Self::emit_nodal_device_evaluation_final(
                code,
                ir,
                &format!("{indent}                "),
                "v",
            );
            code.push_str(&format!("{indent}            }}\n"));
        }
        code.push_str(&format!(
            "{indent}        }} else {{\n\
             {indent}            // LU failed — keep unclamped v. The output-stage\n\
             {indent}            // clamp and diag counters still catch pathological cases.\n\
             {indent}            state.diag_nr_max_iter_count += 1;\n\
             {indent}        }}\n\
             {indent}        // The pinned system is the one this sample actually solves,\n\
             {indent}        // so its outcome is the sample's: a converged pin clears an\n\
             {indent}        // unpinned failure (whose op-amp sat far outside its rail),\n\
             {indent}        // and a pin that did not converge marks the sample unsolved\n\
             {indent}        // so every verb refuses it.\n\
             {indent}        state.last_nr_iterations = if pin_converged && pin_lu_ok {{ pin_iters }} else {{ MAX_ITER as u32 }};\n",
        ));
        // Where the pinned iterate is committed either way (full-LU, and the
        // Newton-free M = 0 Schur solve), count the failure here. A Schur
        // Newton hands the pinned outcome to `converged`, so a failed pin goes
        // to the backward-Euler solve and then the death-spiral hold, like a
        // failed Newton solve.
        if pin_failure_is_committed(ir, &site) {
            code.push_str(&format!(
                "{indent}        if !(pin_converged && pin_lu_ok) {{ state.diag_nr_unconverged_commit_count += 1; state.diag_unsolved_sample_count += 1; }}\n"
            ));
        } else {
            code.push_str(&format!(
                "{indent}        converged = pin_converged && pin_lu_ok;\n"
            ));
        }
        code.push_str(&format!("{indent}    }}\n{indent}}}\n"));
    }

    /// Emit per-op-amp slew-rate limiting on the converged node voltages.
    ///
    /// For each op-amp whose `.model OA(SR=…)` sets a finite slew rate, this
    /// emits a per-sample voltage-delta clamp of the form
    ///
    /// ```ignore
    /// {
    ///     let prev = state.v_prev[OUT];
    ///     let max_dv = OA{idx}_SR * (1.0 / (state.current_sample_rate * OVERSAMPLING_FACTOR as f64));
    ///     let delta  = v_name[OUT] - prev;
    ///     v_name[OUT] = prev + delta.clamp(-max_dv, max_dv);
    /// }
    /// ```
    ///
    /// where `OA{idx}_SR` is a per-device constant in V/s and
    /// `state.current_sample_rate` is the HOST sample rate (DK-parity
    /// semantics), so the slew dt multiplies in `OVERSAMPLING_FACTOR` to get
    /// the internal (post-oversampling) rate.
    ///
    /// ## Physical justification
    ///
    /// In the Boyle macromodel the dominant pole is a cap `C_dom` integrating
    /// the `Gm*(v+ - v-)` current at an internal gain node. Real op-amps
    /// slew-limit because their input stage can't source more than `I_tail`
    /// into `C_dom`, capping `|dV/dt| = I_tail / C_dom ≡ SR`. The equivalent
    /// integrator current limit is `I_slew = SR * C_dom`. melange currently
    /// stamps the op-amp as an ideal VCCS (no explicit integrator), so the
    /// same limit is applied directly in voltage space as
    /// `|Δv_out| ≤ SR*dt`. This is numerically identical to
    /// `|i_in_integrator| ≤ I_slew`, because
    /// `Δv_integrator = (i_in*dt)/C_dom`.
    ///
    /// ## Rail-mode interaction
    ///
    /// The slew limit is applied AFTER the rail clamp so slew-limited
    /// transients can't overshoot the rails, and BEFORE `state.v_prev = v`
    /// so the cap history `(2/T)·C·v_prev` term on the next sample reflects
    /// the slew-limited voltage (preserving KCL). The limit is compatible
    /// with `None`, `Hard`, `ActiveSet`, and `ActiveSetBe` rail modes.
    /// `BoyleDiodes` mode is supported but note that the existing
    /// BoyleDiodes heavy-clip convergence issues (see
    /// `docs/aidocs/OPAMP_RAIL_MODES.md`) are independent of slew limiting.
    ///
    /// No code is emitted when all op-amps have `sr = INFINITY`, so
    /// circuits without `SR=` in their .model card produce byte-identical
    /// generated code to the pre-slew-rate behaviour.
    pub(super) fn emit_opamp_slew_limit(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
        v_name: &str,
    ) {
        let slew_opamps: Vec<(usize, &crate::codegen::ir::OpampIR)> = ir
            .opamps
            .iter()
            .enumerate()
            .filter(|(_, oa)| oa.sr.is_finite())
            .collect();
        if slew_opamps.is_empty() {
            return;
        }
        code.push_str(&format!(
            "{indent}// Op-amp slew-rate limiting: clamp |Δv_out| ≤ SR*dt\n"
        ));
        code.push_str(&format!(
            "{indent}// (equivalent to clamping Boyle C_dom integrator input to ±SR*C_dom)\n"
        ));
        code.push_str(&format!(
            "{indent}let _oa_slew_dt = 1.0 / (state.current_sample_rate * OVERSAMPLING_FACTOR as f64);\n"
        ));
        for (idx, oa) in &slew_opamps {
            code.push_str(&format!(
                "{indent}{{\n\
                 {indent}    let prev = state.v_prev[{node}];\n\
                 {indent}    let max_dv = OA{idx}_SR * _oa_slew_dt;\n\
                 {indent}    let delta = {v_name}[{node}] - prev;\n\
                 {indent}    {v_name}[{node}] = prev + delta.clamp(-max_dv, max_dv);\n\
                 {indent}}}\n",
                node = oa.n_out_idx,
            ));
        }
    }

    /// Emit a cheap rail-violation check that sets `<flag_name> = true` if any
    /// clampable op-amp output is outside its VCC/VEE range. Used in the
    /// trapezoidal NR path to decide whether to fall through to the BE
    /// fallback (where the active-set resolve runs against BE matrices, which
    /// don't develop the Nyquist limit cycle that trap+pin does).
    ///
    /// The caller must have `v` (immutable read access) and the boolean flag
    /// in scope. This emits no resolve, no LU solve, no state mutation — only
    /// the check.
    pub(super) fn emit_nodal_active_set_check(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
        flag_name: &str,
    ) {
        let clampable: Vec<&crate::codegen::ir::OpampIR> = ir
            .opamps
            .iter()
            .filter(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
            .collect();
        if clampable.is_empty() {
            return;
        }

        // Inclusive comparison is deliberate. When the NR-inner rail clamp
        // pins `v_new[n] = hi` exactly on a clip-engaged sample, the final
        // converged `v[n]` lands at the rail to the last bit. Strict `>` / `<`
        // misses that case — the BE fallback never runs, KCL residual sits
        // out through the clamp, and cap history drifts into the sub-Hz
        // oscillation observed on 4kbuscomp. `>=` / `<=` catches both the
        // "pinned exactly" case and any tiny numerical overshoot past the
        // rail, covering 4kbuscomp without over-eager NR-iteration tracking.
        for oa in &clampable {
            // Compare the load-line quantity against FINITE rails only (an
            // infinite bound would emit the invalid token `inf`/`-inf`).
            // `clampable` guarantees at least one comparison per op-amp.
            let w = opamp_load_line_expr(oa, "v");
            let mut conds: Vec<String> = Vec::new();
            if oa.vclamp_hi.is_finite() {
                conds.push(format!("{w} >= {hi:.17e}", hi = oa.vclamp_hi));
            }
            if oa.vclamp_lo.is_finite() {
                conds.push(format!("{w} <= {lo:.17e}", lo = oa.vclamp_lo));
            }
            code.push_str(&format!(
                "{indent}if {} {{ {flag_name} = true; }}\n",
                conds.join(" || ")
            ));
        }
    }

    /// Format a rail-clamp expression for `target` (e.g. `"v[3]"`) that is
    /// safe for single-sided rails: `.max(lo)` / `.min(hi)` are emitted only
    /// for FINITE bounds. Returns `None` when neither bound is finite.
    ///
    /// The previous `{target}.clamp({lo:.17e}, {hi:.17e})` formatting rendered
    /// `f64::NEG_INFINITY` as the invalid Rust token `-inf` whenever exactly
    /// one of VCC/VEE was specified (the guards only skipped when BOTH were
    /// infinite), producing generated code that failed to compile.
    fn rail_clamp_expr(target: &str, lo: f64, hi: f64) -> Option<String> {
        let mut expr = target.to_string();
        if lo.is_finite() {
            expr.push_str(&format!(".max({lo:.17e})"));
        }
        if hi.is_finite() {
            expr.push_str(&format!(".min({hi:.17e})"));
        }
        if expr.len() == target.len() {
            None
        } else {
            Some(expr)
        }
    }

    /// Format a full rail-clamp statement `{target} = <clamped>;` (no
    /// trailing newline). Returns `None` when neither bound is finite.
    pub(super) fn rail_clamp_stmt(target: &str, lo: f64, hi: f64) -> Option<String> {
        Self::rail_clamp_expr(target, lo, hi).map(|expr| format!("{target} = {expr};"))
    }
}

/// The load-line quantity of an op-amp on the node vector `v`:
/// `w = v_out + R_SAG * I_load`, with `I_load = Gm*(v+ - v-) - v_out/ROUT`
/// the current the linear model delivers into the circuit. A saturated output
/// can supply at most `limit - R_SAG*I_load`, so the linear solution is past
/// the rail exactly when `w` passes the limit. With `R_SAG = ROUT`, `w` is the
/// internal (Thevenin) voltage `AOL*(v+ - v-)`. Returned as a bare sum (a
/// wrapping pair of parentheses is an `unused_parens` warning in a `let`).
fn opamp_load_line_expr(oa: &crate::codegen::ir::OpampIR, v: &str) -> String {
    let r_sag = 1.0 / oa.g_sag;
    let a = 1.0 - oa.g_out * r_sag;
    let b = oa.gm * r_sag;
    let vp = oa
        .n_plus_idx
        .map_or("0.0".to_string(), |i| format!("{v}[{i}]"));
    let vm = oa
        .n_minus_idx
        .map_or("0.0".to_string(), |i| format!("{v}[{i}]"));
    format!(
        "{v}[{o}] * {a:.17e} + {b:.17e} * ({vp} - {vm})",
        o = oa.n_out_idx
    )
}
