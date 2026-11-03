//! Node-space KCL residual helpers, sparse `N_V`/`N_I` products, and the
//! Armijo line search.

use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::helpers::kcl_rows;
use crate::codegen::rust_emitter::RustEmitter;

/// The Rust iterator over `rows` in emitted code: `contiguous` (the site's
/// own spelling of `0..n_nodes`) when the rows are exactly the circuit nodes,
/// the explicit list otherwise.
pub(super) fn row_iter(rows: &[usize], contiguous: &str) -> String {
    if rows.iter().enumerate().all(|(k, &r)| k == r) {
        contiguous.to_string()
    } else {
        format!(
            "[{}]",
            rows.iter()
                .map(|r| r.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        )
    }
}

/// The residual helpers (`kcl_residual`, `kcl_residual_inl`) the Newton
/// convergence gate and the Armijo line search read: the full-LU Newton, and
/// the sub-step ladder on both nodal routes. Emitted when M > 0.
///
/// They cover EVERY row the Newton step solves, each with its true residual
/// at the helpers' own `v`:
/// - KCL rows (`kcl_rows`: circuit nodes, parasitic-BJT internal nodes):
///   `A·v − rhs − q`, `q = N_i·i_nl` plus behavioral `I={}` currents, in amps,
///   tolerance `RELTOL·den + KCL_ABSTOL` (1e-3·den + 1e-9 A). Only these rows
///   feed the returned infinity norm (`max_abs`, the chord-exit gate's
///   `KCL_ABSTOL` test).
/// - Algebraic rows (`n_nodes..n_aug` minus the above: voltage sources, VCVS,
///   ideal transformers, op-amp internal and VCA rows) and inductor branch
///   rows (`n_aug..n`): `A·v − rhs − q`, tolerance `RELTOL·den + VNTOL`
///   (1e-3·den + 1e-6), the node-step test's floor, in the row's own unit
///   (volts for a source or branch equation).
/// - Behavioral `V={}` rows: `(v[n+] − v[n−]) − f(v)` (`A·v − rhs` is neither
///   the linearised nor the true residual there), same volt tolerance.
/// - Saturating-inductor rows: the flux residual of `emit_sat_ind_row_residual`
///   with its own `1e-5·den` test; the helpers take the site's `alpha` as
///   `sat_alpha` for it.
/// Every row enters `||F||_2` in its own unit (volts added to amps in
/// quadrature). A merit over the KCL rows alone left the Newton direction's
/// first-order decrease at `−||F_kcl||²`; when a source or branch row carried
/// the residual (a time-stepped behavioral source, a rectifier's first
/// conduction into an empty reservoir) that was tiny against the full step's
/// device overshoot, and the line search backtracked a correct step to
/// `1/32..1/1024` on every iteration: one warm-up sample took 250 Newton
/// iterations and, held, froze the render for 0.65 s. With every row in the
/// merit the same sample converges in 4. The gate reading every row closes
/// the matching hole: after a damped step nothing else certified the source
/// and branch equations.
pub(super) fn emit_kcl_residual_fns(
    ir: &CircuitIR,
    setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
    full_lu: bool,
) -> String {
    let rows = kcl_rows(ir);
    let has_sat = !ir.saturating_inductors.is_empty();
    let sat_param = if has_sat { ", sat_alpha: f64" } else { "" };
    // Rows outside the KCL set that take the plain `A·v − rhs − q` form:
    // everything but behavioral V={} rows and saturating-inductor rows, which
    // have their own residual forms below.
    let aug_rows: Vec<usize> = {
        let kcl: std::collections::BTreeSet<usize> = rows.iter().copied().collect();
        let own: std::collections::BTreeSet<usize> = ir
            .saturating_inductors
            .iter()
            .map(|si| si.aug_row)
            .chain(
                ir.behavioral_sources
                    .iter()
                    .filter(|b| b.is_voltage)
                    .filter_map(|b| b.aug_row),
            )
            .collect();
        (0..ir.topology.n)
            .filter(|r| !kcl.contains(r) && !own.contains(r))
            .collect()
    };
    let mut code = String::new();
    // Shared node-space KCL residual helper, generic over the A matrix +
    // rhs so the trap (state.a/rhs), sub-step (a_sub/rhs_s) and BE
    // (state.a_be/rhs_be) sites share one implementation. Returns
    // (||F||_2, all-node-rows-within-tolerance) for F = A·v - rhs -
    // N_i·i_nl(N_v·v) over the KCL rows (plus the behavioral `I={}`
    // currents, which are not Newton unknowns: `emit_behavioral_current_residual`) (`kcl_rows`: circuit nodes and
    // parasitic-BJT internal nodes). The ||.||_2 drives the
    // Armijo ratio test; the bool is the always-checked convergence gate.
    code.push_str(
        "/// Residual of every row the Newton step solves, for the nodal NR\n\
         /// convergence gate and the Armijo line search: `F = A·v - rhs - q`\n\
         /// (`q` = device and behavioral currents) on KCL, source and branch\n\
         /// rows, the true `(v+ - v-) - f(v)` on behavioral V={} rows and the\n\
         /// flux residual on saturating-inductor rows. Returns `(||F||_2 over\n\
         /// every row, all_rows_within_tol, ||F||_inf over the KCL rows)`.\n\
         /// Generic over the A matrix + rhs so the trap / sub-step / BE sites\n\
         /// share one body.\n",
    );
    code.push_str("#[inline]\n");
    code.push_str(&format!(
        "fn kcl_residual(v: &[f64; N], rhs: &[f64; N], amat: &[[f64; N]; N], state: &CircuitState{sat_param}) -> (f64, bool, f64) {{\n",
    ));
    // Shared residual tail: `q = N_i·i_nl`, then `||F||_2` over node rows
    // with the sparse `A·v` matvec. Emitted verbatim into BOTH residual
    // functions below, so `kcl_residual` (device-evaluating) and
    // `kcl_residual_inl` (`i_nl` reused) are bit-for-bit identical past the
    // point i_nl is available — the guarantee the r0 CSE rests on.
    //
    // The `A·v` term dominates this residual (it runs >=2x per NR iteration
    // via the convergence gate + Armijo r0). Emit it sparsely over the
    // structural nonzeros of the raw forward A — a byte-identical transform
    // (skipped columns are structural zeros). Fall back to the dense sweep
    // only when the forward matrices are unavailable.
    let mut tail = String::new();
    tail.push_str("    let mut q = [0.0f64; N];\n");
    tail.push_str(&emit_sparse_ni_matvec_add(ir, "q", "i_nl", "    "));
    super::behavioral::emit_behavioral_current_residual(&mut tail, ir, "    ");
    tail.push_str("    let mut norm_sq = 0.0f64;\n");
    tail.push_str("    let mut max_abs = 0.0f64;\n");
    tail.push_str("    let mut ok = true;\n");
    // KCL rows: amps, KCL_ABSTOL floor, the only rows in the infinity norm.
    tail.push_str("    // KCL rows (A)\n");
    emit_a_residual_rows(
        ir,
        setter_stamps,
        &rows,
        "KCL_ABSTOL",
        true,
        &mut tail,
        "    ",
    );
    // Source, constraint and inductor branch rows: VNTOL floor (the node-step
    // test's), never in the infinity norm.
    if !aug_rows.is_empty() {
        tail.push_str("    // source / constraint / inductor branch rows (V)\n");
        emit_a_residual_rows(
            ir,
            setter_stamps,
            &aug_rows,
            "VNTOL",
            false,
            &mut tail,
            "    ",
        );
    }
    super::behavioral::emit_behavioral_voltage_row_residual(&mut tail, ir, "    ");
    super::sat_ind::emit_sat_ind_merit_rows(
        &mut tail,
        ir,
        setter_stamps,
        "rhs",
        Some("i_nl"),
        "v",
        "amat",
        "    ",
    );
    tail.push_str("    (norm_sq.sqrt(), ok, max_abs)\n");
    tail.push_str("}\n\n");

    // (1) Device-evaluating residual — used by the always-checked
    // convergence gate (at v_new) and the Armijo backtrack trials (at
    // v + s·alpha·d). Evaluates i_nl at N_v·v via the device model.
    code.push_str("    let mut v_nl_final = [0.0f64; M];\n");
    code.push_str(&emit_sparse_nv_matvec(ir, "v_nl_final", "v", "    "));
    code.push_str("    let mut i_nl = [0.0f64; M];\n");
    RustEmitter::emit_nodal_device_evaluation_final(&mut code, ir, "    ", "v");
    code.push_str(&tail);

    // (2) i_nl-reusing residual: the residual tail on device currents
    // already evaluated at this exact v (no pnjlim on the eval voltage,
    // limiting is applied to the *step*). Used for the Armijo r0 (the NR
    // body's currents), the convergence gate (the device-chord check's
    // currents) and, through (3)/(4), the line-search trials.
    code.push_str(
        "/// KCL residual reusing an already-computed `i_nl` (Armijo r0 CSE);\n\
         /// bit-identical to `kcl_residual` when `i_nl == i_nl(N_v·v)`.\n",
    );
    code.push_str("#[inline]\n");
    code.push_str(&format!(
        "fn kcl_residual_inl(v: &[f64; N], rhs: &[f64; N], amat: &[[f64; N]; N], i_nl: &[f64; M]{}{sat_param}) -> (f64, bool, f64) {{\n",
        if super::behavioral::residual_needs_state(ir) { ", state: &CircuitState" } else { "" }
    ));
    code.push_str(&tail);

    // (3) Device-evaluating residual that also hands back the device currents
    // it evaluated, for the Newton sites' accepted-trial reuse
    // (`armijo_trial_reuse`). Literally (1): the same `N_v·v` and `_final`
    // device evaluation, then the same tail via `kcl_residual_inl`. Builds
    // that take (4) instead do not need it, and only the full-LU Newton
    // sites call either (`full_lu`).
    if full_lu && armijo_trial_reuse(ir) && !armijo_jac_reuse(ir) {
        code.push_str(
            "/// `kcl_residual`, also returning the device currents it evaluated at `v`\n\
             /// (the Armijo trial the Newton step keeps; see the Newton loop).\n",
        );
        code.push_str("#[inline]\n");
        code.push_str(
            "fn kcl_residual_eval(v: &[f64; N], rhs: &[f64; N], amat: &[[f64; N]; N], state: &CircuitState, i_nl_out: &mut [f64; M]) -> (f64, bool, f64) {\n",
        );
        code.push_str("    let mut v_nl_final = [0.0f64; M];\n");
        code.push_str(&emit_sparse_nv_matvec(ir, "v_nl_final", "v", "    "));
        code.push_str("    let mut i_nl = [0.0f64; M];\n");
        RustEmitter::emit_nodal_device_evaluation_final(&mut code, ir, "    ", "v");
        code.push_str("    *i_nl_out = i_nl;\n");
        code.push_str("    kcl_residual_inl(v, rhs, amat, &i_nl)\n");
        code.push_str("}\n\n");
    }
    // (4) The same, evaluating the devices with the Newton body's code, which
    // also writes the device Jacobian (`armijo_jac_reuse`).
    if full_lu && armijo_jac_reuse(ir) {
        code.push_str(
            "/// `kcl_residual`, evaluating the devices with the Newton body's code: also\n\
             /// returns their currents and writes their Jacobian block into `j_dev`.\n",
        );
        code.push_str("#[inline]\n");
        code.push_str(
            "fn kcl_residual_eval_jac(v: &[f64; N], rhs: &[f64; N], amat: &[[f64; N]; N], state: &CircuitState, i_nl_out: &mut [f64; M], j_dev: &mut [f64; M * M]) -> (f64, bool, f64) {\n",
        );
        code.push_str("    let mut v_nl = [0.0f64; M];\n");
        code.push_str(&emit_sparse_nv_matvec(ir, "v_nl", "v", "    "));
        code.push_str("    let mut i_nl = [0.0f64; M];\n");
        for k in super::sites::j_dev_block_entries(ir) {
            code.push_str(&format!("    j_dev[{k}] = 0.0;\n"));
        }
        RustEmitter::emit_nodal_device_evaluation_body(&mut code, ir, "    ", "v");
        code.push_str("    *i_nl_out = i_nl;\n");
        code.push_str("    kcl_residual_inl(v, rhs, amat, &i_nl)\n");
        code.push_str("}\n\n");
    }
    code
}

/// True when, on top of [`armijo_r0_reuse`], the line-search trials evaluate
/// the devices with the Newton body's code (currents and Jacobian), so that
/// after an accepted trial the next iteration starts from the trial's `i_nl`
/// and `j_dev` instead of evaluating the devices again at the same `v`.
///
/// The body evaluation is a function of `v_nl = N_v·v` and of read-only state
/// only, except for a body-effect MOSFET, which writes its effective
/// threshold into `state` and a `body_gmb` local; builds with one keep the
/// separate evaluations. For the classes [`armijo_r0_reuse`] admits, the body
/// code's currents are the `_final` code's, so the device-chord check and the
/// gate read the same values either way.
pub(super) fn armijo_jac_reuse(ir: &CircuitIR) -> bool {
    armijo_r0_reuse(ir) && crate::codegen::rust_emitter::helpers::body_effect_mosfets(ir).is_empty()
}

/// True when a full-LU Newton site keeps the accepted Armijo trial's
/// evaluation instead of repeating it.
///
/// On acceptance at fraction `s` the trial point is `v + (alpha·s)·d`, and the
/// step then applied is `v + alpha'·d` with `alpha' = alpha·s`: the same two
/// roundings, so the committed iterate IS the trial point, bit for bit. The
/// device-chord check and the KCL gate that follow evaluate `i_nl` and the
/// residual at that iterate with the same `_final` device code and the same
/// residual tail the trial ran (`kcl_residual` is `N_v·v`, `_final`, then the
/// `kcl_residual_inl` tail), so the trial's `i_nl` and `(||F||, ok, inf)` are
/// what they would compute. That holds only while nothing moves `v` between
/// the step and those checks, and while the residual helpers need nothing but
/// `v`, `rhs`, `A` and the device currents:
/// - `Hard` op-amp rails clamp `v` after the step (the clamp is emitted only
///   when some op-amp exists), so a `Hard` build with op-amps keeps the
///   re-evaluation;
/// - behavioral sources and saturating inductors add `state` / `sat_alpha`
///   terms to the residual and refactor every iteration; they keep it too.
pub(super) fn armijo_trial_reuse(ir: &CircuitIR) -> bool {
    ir.topology.m > 0
        && ir.behavioral_sources.is_empty()
        && ir.saturating_inductors.is_empty()
        && !(matches!(
            ir.solver_config.opamp_rail_mode,
            crate::codegen::OpampRailMode::Hard
        ) && !ir.opamps.is_empty())
}

/// True when, on top of [`armijo_trial_reuse`], the next iteration's Armijo
/// `r0` may be the accepted trial's `||F||`.
///
/// `r0` is computed from the device currents the Newton body evaluated at the
/// iterate (`emit_nodal_device_evaluation_body`); the trial's come from the
/// `_final` evaluation. For the device classes listed here the two apply the
/// same current expression to the same `v_nl` (the same helper call, or for an
/// RS diode `diode_eval_with_rs` whose current is `diode_current_with_rs` by
/// construction), so they agree bit for bit. Triodes and pentodes are not
/// listed: their body helper (`tube_evaluate*`) and their `_final` helpers
/// (`tube_ip*` / `tube_is*`) differ at the plate-current cutoff, so those
/// builds recompute `r0`. A device class added later stays off this list
/// until its pair is checked.
pub(super) fn armijo_r0_reuse(ir: &CircuitIR) -> bool {
    use crate::codegen::ir::DeviceType;
    armijo_trial_reuse(ir)
        && ir.device_slots.iter().all(|slot| {
            matches!(
                slot.device_type,
                DeviceType::Diode
                    | DeviceType::Bjt
                    | DeviceType::BjtForwardActive
                    | DeviceType::Jfet
                    | DeviceType::Mosfet
                    | DeviceType::Vca
                    | DeviceType::Ldr
                    | DeviceType::Glow
            )
        })
}

// ============================================================================
// Sparse N_V / N_I helpers
// ============================================================================

/// Emit a sparse `result = N_V[row] * vec` product.
///
/// Instead of `for j in 0..N { result += N_V[row][j] * vec[j]; }`,
/// emits only the nonzero terms: `N_V[row][c1] * vec[c1] + N_V[row][c2] * vec[c2]`.
/// N_V typically has 2 nonzeros per row (±1 at device nodes), so this is ~28x faster at N=57.
pub(super) fn emit_sparse_nv_dot(
    ir: &CircuitIR,
    row: usize,
    result_var: &str,
    vec_var: &str,
    indent: &str,
) -> String {
    let nz = &ir.sparsity.n_v.nz_by_row;
    if row < nz.len() && !nz[row].is_empty() {
        let terms: Vec<String> = nz[row]
            .iter()
            .map(|&col| format!("N_V[{}][{}] * {}[{}]", row, col, vec_var, col))
            .collect();
        format!("{indent}let {result_var} = {};\n", terms.join(" + "))
    } else {
        format!("{indent}let {result_var} = 0.0;\n")
    }
}

/// Emit sparse `v_nl[i] = sum_j N_V[i][j] * vec[j]` for all M rows.
pub(super) fn emit_sparse_nv_matvec(
    ir: &CircuitIR,
    result_arr: &str,
    vec_var: &str,
    indent: &str,
) -> String {
    let m = ir.topology.m;
    let mut code = String::new();
    for i in 0..m {
        let nz = &ir.sparsity.n_v.nz_by_row;
        if i < nz.len() && !nz[i].is_empty() {
            let terms: Vec<String> = nz[i]
                .iter()
                .map(|&col| format!("N_V[{}][{}] * {}[{}]", i, col, vec_var, col))
                .collect();
            code.push_str(&format!(
                "{indent}{result_arr}[{i}] = {};\n",
                terms.join(" + ")
            ));
        }
    }
    code
}

/// Emit sparse `rhs[i] += sum_j N_I[i][j] * vec[j]` for all N rows.
pub(super) fn emit_sparse_ni_matvec_add(
    ir: &CircuitIR,
    result_arr: &str,
    vec_var: &str,
    indent: &str,
) -> String {
    let n = ir.topology.n;
    let mut code = String::new();
    let nz = &ir.sparsity.n_i.nz_by_row;
    for i in 0..n {
        if i < nz.len() && !nz[i].is_empty() {
            let terms: Vec<String> = nz[i]
                .iter()
                .map(|&col| format!("N_I[{}][{}] * {}[{}]", i, col, vec_var, col))
                .collect();
            code.push_str(&format!(
                "{indent}{result_arr}[{i}] += {};\n",
                terms.join(" + ")
            ));
        }
    }
    code
}

/// Emit the residual rows `rows` of `F = A·v − rhs − q` into `code`: the
/// sparse form when the forward matrices are available, the dense sweep
/// otherwise. `abs_floor` is the tolerance's absolute floor as source text
/// (`KCL_ABSTOL` amps on KCL rows, `VNTOL` on source / branch rows); `track_inf`
/// says whether the rows feed the KCL-only infinity norm `max_abs`.
#[allow(clippy::too_many_arguments)]
fn emit_a_residual_rows(
    ir: &CircuitIR,
    setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
    rows: &[usize],
    abs_floor: &str,
    track_inf: bool,
    code: &mut String,
    indent: &str,
) {
    match emit_sparse_a_residual_matvec(ir, setter_stamps, rows, abs_floor, track_inf, indent) {
        Some(sparse) => code.push_str(&sparse),
        None => {
            code.push_str(&format!(
                "{indent}for i in {} {{\n",
                row_iter(rows, &format!("0..{}", rows.len()))
            ));
            code.push_str(&format!("{indent}    let mut acc = -rhs[i] - q[i];\n"));
            code.push_str(&format!(
                "{indent}    let mut den = rhs[i].abs().max(q[i].abs());\n"
            ));
            code.push_str(&format!("{indent}    for j in 0..N {{\n"));
            code.push_str(&format!("{indent}        let t = amat[i][j] * v[j];\n"));
            code.push_str(&format!("{indent}        acc += t;\n"));
            code.push_str(&format!(
                "{indent}        let a = t.abs(); if a > den {{ den = a; }}\n"
            ));
            code.push_str(&format!("{indent}    }}\n"));
            code.push_str(&format!("{indent}    norm_sq += acc * acc;\n"));
            if track_inf {
                // NaN-free max (see emit_sparse_a_residual_matvec).
                code.push_str(&format!(
                    "{indent}    {{ let a = acc.abs(); if a > max_abs {{ max_abs = a; }} }}\n"
                ));
            }
            // Per-row relative tolerance: the same RELTOL as the device and
            // sat-inductor residual checks, with an absolute floor so a row
            // carrying ~zero does not demand an unreachable tolerance.
            code.push_str(&format!(
                "{indent}    if !(acc.abs() <= RELTOL * den + {abs_floor}) {{ ok = false; }}\n"
            ));
            code.push_str(&format!("{indent}}}\n"));
        }
    }
}

/// Emit the node-space KCL-residual `A·v` matvec, sparsified over the structural
/// nonzero columns of the raw forward `amat` (state.a / a_sub / state.a_be).
///
/// BYTE-IDENTICAL to the dense `for j in 0..N` loop it replaces: the skipped
/// columns are STRUCTURAL zeros of the raw A matrix, so their term
/// `amat[i][j]*v[j]` is exactly `0.0`, which changes neither the ascending-order
/// running sum `acc` (`x + 0.0 == x`) nor the running max `den` (`0.0` never
/// exceeds a non-negative `den`). Columns are emitted in ascending order so the
/// float summation order of the surviving nonzeros is preserved exactly.
///
/// The column set per node row is the structural superset of the raw A pattern:
///   - `nz(a_matrix) ∪ nz(a_matrix_be)` — every position live at the codegen
///     config, at full augmented dimension (transformer/inductor incidence,
///     mutual coupling, voltage-source rows all included). These two forward
///     matrices are `G + β·C` for β ∈ {2/T, 1/T}; a position is absent from BOTH
///     only if `G = C = 0` there, so their union equals the value-independent
///     `nz(G) ∪ nz(C)` and also covers the sub-step matrix `G + (4/T)·C`.
///   - `setter_stamps` — every `(row,col)` a `.pot`/`.switch`/`.wiper`/`.runtime`
///     setter writes, so a switch entry that is open (≈0, below threshold) at the
///     codegen position but closed (large) at another position is still summed.
///   - the diagonal `(i,i)` — the saturating-inductor augmented-row term.
/// This mirrors `build_equil_pattern` parts (1)-(3), which ship byte-identical on
/// these decks (incl. the sat-inductor `steve-1073-output`). Part (4), the device
/// Jacobian envelope, is intentionally omitted: the raw `amat` carries no device
/// stamps (those go into `chord_lu`), so those columns are structurally `0.0`.
///
/// Returns `None` (caller keeps the dense loop) when the forward matrices are not
/// available at the expected dimension.
fn emit_sparse_a_residual_matvec(
    ir: &CircuitIR,
    setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
    rows: &[usize],
    abs_floor: &str,
    track_inf: bool,
    indent: &str,
) -> Option<String> {
    use std::collections::BTreeSet;
    let n = ir.topology.n;
    let a = &ir.matrices.a_matrix;
    let a_be = &ir.matrices.a_matrix_be;
    if a.len() != n * n || a_be.len() != n * n || ir.sparsity.a.nz_by_row.len() != n {
        return None;
    }
    let mut code = String::new();
    for &i in rows {
        // A's structural pattern (G's and C's stamped positions) covers both
        // the trapezoidal A and the backward-Euler A_be at any rate.
        let mut cols: BTreeSet<usize> = ir.sparsity.a.nz_by_row[i].iter().copied().collect();
        for &(r, c) in setter_stamps {
            if r == i {
                cols.insert(c);
            }
        }
        cols.insert(i);
        code.push_str(&format!("{indent}{{\n"));
        code.push_str(&format!("{indent}    let mut acc = -rhs[{i}] - q[{i}];\n"));
        code.push_str(&format!(
            "{indent}    let mut den = rhs[{i}].abs().max(q[{i}].abs());\n"
        ));
        for j in cols {
            code.push_str(&format!(
                "{indent}    {{ let t = amat[{i}][{j}] * v[{j}]; acc += t; let a = t.abs(); if a > den {{ den = a; }} }}\n"
            ));
        }
        code.push_str(&format!("{indent}    norm_sq += acc * acc;\n"));
        // Running max of |acc| as a compare, not `f64::max`: `max` must handle a
        // NaN in either operand and lowers to several instructions per row
        // (+8 % instructions per sample on a 70-row full-LU deck). `max_abs`
        // starts at 0 and is never NaN; a NaN `acc` leaves it unchanged under
        // both forms, and `abs` never yields -0, so the value is bit-identical.
        if track_inf {
            code.push_str(&format!(
                "{indent}    {{ let a = acc.abs(); if a > max_abs {{ max_abs = a; }} }}\n"
            ));
        }
        code.push_str(&format!(
            "{indent}    if !(acc.abs() <= RELTOL * den + {abs_floor}) {{ ok = false; }}\n"
        ));
        code.push_str(&format!("{indent}}}\n"));
    }
    Some(code)
}

/// The trailing `sat_alpha` argument the residual helpers take when the
/// circuit has saturating inductors (`, <site alpha>`), or nothing.
///
/// A site's `sat_alpha` is parenthesised for the arithmetic it is spliced
/// into (`{alpha} * (...)` in `sat_ind.rs`). As a bare call argument those
/// parentheses would trip rustc's `unused_parens` lint in the generated
/// crate, so one outer pair is peeled here.
pub(super) fn residual_sat_arg(ir: &CircuitIR, sat_alpha: &str) -> String {
    if ir.saturating_inductors.is_empty() {
        return String::new();
    }
    let bare = sat_alpha
        .strip_prefix('(')
        .and_then(|s| s.strip_suffix(')'))
        .filter(|inner| {
            // Only peel when the pair wraps the whole expression: the depth
            // must never return to zero before the end of `inner`.
            let mut depth = 0i32;
            inner.chars().all(|c| {
                depth += match c {
                    '(' => 1,
                    ')' => -1,
                    _ => 0,
                };
                depth >= 0
            })
        })
        .unwrap_or(sat_alpha);
    format!(", {bare}")
}

/// Emit an Armijo backtracking line search on the node-space KCL residual for
/// one full-LU NR site.
///
/// The pnjlim/node-damping limiting has already been applied ONCE to produce the
/// step direction `{new} - {v}` scaled by `{alpha}`. This searches along that
/// single ray (no per-backtrack re-limiting): it scales the fraction `s` down
/// from 1 by halves, floored at 2^-10, and accepts the first `s` satisfying the
/// Armijo sufficient-decrease `||F(v + s·alpha·d)|| <= (1 - c·s)·||F(v)||`
/// (c = 1e-4). On acceptance `{alpha}` is multiplied by `s` (a no-op when
/// `s == 1`, so a sample that already takes the full limited step is unaffected).
/// If no `s` down to the floor satisfies Armijo — a non-descent / near-singular
/// direction — `{ls_ok}` is set false. The caller then takes the un-line-searched
/// step (`{alpha}` keeps its pnjlim/node-damping limiter value, since `{alpha} *= s`
/// runs only on acceptance) and CONTINUES the loop. It is the always-checked
/// `||F||` residual gate — NOT the line search — that prevents committing a
/// non-root iterate: the gate rejects any iterate whose true equation residual is
/// not within tolerance, so a failed search can never smuggle a bad state through.
/// The prior behavior (break to the sub-step / BE fallback on failure) was strictly
/// worse: the residual gate already closed the false-convergence hole, and the bail
/// routed an otherwise-recoverable iterate into the least-protected fallback, where
/// it could limit-cycle. Contract (design review): a line search may only
/// HELP; its failure must be no worse than not having it.
///
/// The guard `if r0.is_finite() && r0 > 1e-9` means a non-finite or already-tiny
/// r0 SKIPS the search, leaving `{ls_ok}` true. So `{ls_ok} == false` is
/// specifically "searched and failed" — exactly what `diag_ls_fail_count` counts;
/// it is a surfaced expected event on stiff circuits, not a masked hole.
///
/// This is the globalization that turns the limit-cycle divergence on stiff
/// high-gain feedback amplifiers into monotone convergence. It is paired with
/// the always-checked `||F||` residual gate at each site: a backtracking search
/// can drive `s·alpha -> 0` at a stagnation point, so convergence must be
/// decided by the true equation residual, never by the (now arbitrarily small)
/// damped step alone.
#[allow(clippy::too_many_arguments)]
pub(super) fn emit_armijo_line_search(
    ir: &CircuitIR,
    code: &mut String,
    indent: &str,
    v: &str,
    new: &str,
    alpha: &str,
    amat: &str,
    rhs: &str,
    ls_ok: &str,
    inl: &str,
    sat_alpha: &str,
    reuse: bool,
) {
    let state_arg = super::behavioral::kcl_residual_inl_state_arg(ir);
    let sat_arg = residual_sat_arg(ir, sat_alpha);
    if reuse {
        // Newton-site form (`armijo_trial_reuse`): the accepted trial's device
        // currents and residual are kept in `ls_i_nl` / `ls_res` and `ls_hit`
        // says they belong to the iterate the step is about to commit. With
        // `armijo_r0_reuse`, `ls_prev_hit` says the previous iteration's
        // accepted trial is the current `v`, and its `||F||` is this `r0`.
        assert!(
            state_arg.is_empty() && sat_arg.is_empty(),
            "trial reuse is only emitted for builds whose residual reads v, rhs, A and i_nl"
        );
        let trial = if armijo_jac_reuse(ir) {
            format!("kcl_residual_eval_jac(&vc, &{rhs}, &{amat}, state, &mut ls_i_nl, &mut j_dev)")
        } else {
            format!("kcl_residual_eval(&vc, &{rhs}, &{amat}, state, &mut ls_i_nl)")
        };
        let r0 = if armijo_r0_reuse(ir) {
            format!(
                "if ls_prev_hit {{ ls_res }} else {{ kcl_residual_inl(&{v}, &{rhs}, &{amat}, &{inl}) }}"
            )
        } else {
            format!("kcl_residual_inl(&{v}, &{rhs}, &{amat}, &{inl})")
        };
        code.push_str(&format!(
            "{indent}// Armijo backtracking line search on ||F|| along the limited Newton ray.\n\
             {indent}let mut {ls_ok} = true;\n\
             {indent}let mut ls_hit = false;\n\
             {indent}{{\n\
             {indent}    // r0 = ||F({v})||: from the device currents `{inl}` the NR body\n\
             {indent}    // evaluated at this exact {v}, or the previous iteration's accepted\n\
             {indent}    // trial, which was this {v} (bit-identical to a fresh kcl_residual).\n\
             {indent}    let (r0, _, _) = {r0};\n\
             {indent}    if r0.is_finite() && r0 > 1e-9 {{\n\
             {indent}        let mut s = 1.0_f64;\n\
             {indent}        let mut accepted = false;\n\
             {indent}        while s >= (1.0 / 1024.0) {{\n\
             {indent}            let mut vc = {v};\n\
             {indent}            for i in 0..N {{ vc[i] += ({alpha} * s) * ({new}[i] - {v}[i]); }}\n\
             {indent}            let rc = {trial};\n\
             {indent}            if rc.0 <= (1.0 - 1e-4 * s) * r0 {{ accepted = true; ls_res = rc; break; }}\n\
             {indent}            s *= 0.5;\n\
             {indent}        }}\n\
             {indent}        if accepted {{ {alpha} *= s; ls_hit = true; }} else {{ {ls_ok} = false; }}\n\
             {indent}    }}\n\
             {indent}}}\n"
        ));
        return;
    }
    code.push_str(&format!(
        "{indent}// Armijo backtracking line search on ||F|| along the limited Newton ray.\n\
         {indent}let mut {ls_ok} = true;\n\
         {indent}{{\n\
         {indent}    // r0 = ||F({v})||: reuses the device currents `{inl}` the NR body\n\
         {indent}    // already evaluated at this exact {v} (bit-identical to a fresh\n\
         {indent}    // kcl_residual — same N_v·v, same device fn, no pnjlim on the eval).\n\
         {indent}    let (r0, _, _) = kcl_residual_inl(&{v}, &{rhs}, &{amat}, &{inl}{state_arg}{sat_arg});\n\
         {indent}    if r0.is_finite() && r0 > 1e-9 {{\n\
         {indent}        let mut s = 1.0_f64;\n\
         {indent}        let mut accepted = false;\n\
         {indent}        while s >= (1.0 / 1024.0) {{\n\
         {indent}            let mut vc = {v};\n\
         {indent}            for i in 0..N {{ vc[i] += ({alpha} * s) * ({new}[i] - {v}[i]); }}\n\
         {indent}            let (rc, _, _) = kcl_residual(&vc, &{rhs}, &{amat}, state{sat_arg});\n\
         {indent}            if rc <= (1.0 - 1e-4 * s) * r0 {{ accepted = true; break; }}\n\
         {indent}            s *= 0.5;\n\
         {indent}        }}\n\
         {indent}        if accepted {{ {alpha} *= s; }} else {{ {ls_ok} = false; }}\n\
         {indent}    }}\n\
         {indent}}}\n"
    ));
}
