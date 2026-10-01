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

/// The node-space KCL residual helpers (`kcl_residual`, `kcl_residual_inl`)
/// the Newton convergence gate and the Armijo line search read: the full-LU
/// Newton, and the sub-step ladder on both nodal routes. Emitted when M > 0.
pub(super) fn emit_kcl_residual_fns(
    ir: &CircuitIR,
    setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
) -> String {
    let rows = kcl_rows(ir);
    let mut code = String::new();
    // Shared node-space KCL residual helper, generic over the A matrix +
    // rhs so the trap (state.a/rhs), sub-step (a_sub/rhs_s) and BE
    // (state.a_be/rhs_be) sites share one implementation. Returns
    // (||F||_2, all-node-rows-within-tolerance) for F = A·v - rhs -
    // N_i·i_nl(N_v·v) over the KCL rows (`kcl_rows`: circuit nodes and
    // parasitic-BJT internal nodes). The ||.||_2 drives the
    // Armijo ratio test; the bool is the always-checked convergence gate.
    code.push_str(
        "/// Node-space KCL residual for the nodal NR convergence gate and\n\
         /// Armijo line search. `F = A·v - rhs - N_i·i_nl(N_v·v)` over every KCL\n\
         /// row (circuit nodes and parasitic-BJT internal nodes). Returns\n\
         /// `(||F||_2, all_rows_within_tol, ||F||_inf)`. Generic over the\n\
         /// A matrix + rhs so the trap / sub-step / BE sites share one body.\n",
    );
    code.push_str("#[inline]\n");
    code.push_str(
        "fn kcl_residual(v: &[f64; N], rhs: &[f64; N], amat: &[[f64; N]; N], state: &CircuitState) -> (f64, bool, f64) {\n",
    );
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
    tail.push_str("    let mut norm_sq = 0.0f64;\n");
    tail.push_str("    let mut max_abs = 0.0f64;\n");
    tail.push_str("    let mut ok = true;\n");
    match emit_sparse_a_residual_matvec(ir, setter_stamps, &rows, "    ") {
        Some(sparse) => tail.push_str(&sparse),
        None => {
            tail.push_str(&format!(
                "    for i in {} {{\n",
                row_iter(&rows, &format!("0..{}", rows.len()))
            ));
            tail.push_str("        let mut acc = -rhs[i] - q[i];\n");
            tail.push_str("        let mut den = rhs[i].abs().max(q[i].abs());\n");
            tail.push_str("        for j in 0..N {\n");
            tail.push_str("            let t = amat[i][j] * v[j];\n");
            tail.push_str("            acc += t;\n");
            tail.push_str("            let a = t.abs(); if a > den { den = a; }\n");
            tail.push_str("        }\n");
            tail.push_str("        norm_sq += acc * acc;\n");
            tail.push_str("        max_abs = max_abs.max(acc.abs());\n");
            // Per-node relative KCL tolerance: same RELTOL=1e-3 as the
            // device and sat-inductor residual checks, with a 1e-9 A
            // absolute floor so a node carrying ~zero net current does not
            // demand an unreachable tolerance.
            tail.push_str("        if !(acc.abs() <= 1e-3 * den + 1e-9) { ok = false; }\n");
            tail.push_str("    }\n");
        }
    }
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

    // (2) i_nl-reusing residual for the Armijo r0 only. The main NR body
    // already evaluated i_nl at this exact v (same N_v·v — no pnjlim on the
    // eval voltage, limiting is applied to the *step* — and the same
    // #[inline(always)] device fn as `_final`), so F(v) is bit-identical to
    // (1) without re-running the per-device parasitic inner-NR. Only r0
    // qualifies: the convergence gate is at v_new and the backtrack trials
    // are at v + s·alpha·d, both different v, and keep (1).
    code.push_str(
        "/// KCL residual reusing an already-computed `i_nl` (Armijo r0 CSE);\n\
         /// bit-identical to `kcl_residual` when `i_nl == i_nl(N_v·v)`.\n",
    );
    code.push_str("#[inline]\n");
    code.push_str(
        "fn kcl_residual_inl(v: &[f64; N], rhs: &[f64; N], amat: &[[f64; N]; N], i_nl: &[f64; M]) -> (f64, bool, f64) {\n",
    );
    code.push_str(&tail);
    code
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
    indent: &str,
) -> Option<String> {
    use crate::lu::SPARSITY_THRESHOLD;
    use std::collections::BTreeSet;
    let n = ir.topology.n;
    let a = &ir.matrices.a_matrix;
    let a_be = &ir.matrices.a_matrix_be;
    if a.len() != n * n || a_be.len() != n * n {
        return None;
    }
    let mut code = String::new();
    for &i in rows {
        let mut cols: BTreeSet<usize> = BTreeSet::new();
        for j in 0..n {
            if a[i * n + j].abs() >= SPARSITY_THRESHOLD
                || a_be[i * n + j].abs() >= SPARSITY_THRESHOLD
            {
                cols.insert(j);
            }
        }
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
        code.push_str(&format!("{indent}    max_abs = max_abs.max(acc.abs());\n"));
        code.push_str(&format!(
            "{indent}    if !(acc.abs() <= 1e-3 * den + 1e-9) {{ ok = false; }}\n"
        ));
        code.push_str(&format!("{indent}}}\n"));
    }
    Some(code)
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
    code: &mut String,
    indent: &str,
    v: &str,
    new: &str,
    alpha: &str,
    amat: &str,
    rhs: &str,
    ls_ok: &str,
    inl: &str,
) {
    code.push_str(&format!(
        "{indent}// Armijo backtracking line search on ||F|| along the limited Newton ray.\n\
         {indent}let mut {ls_ok} = true;\n\
         {indent}{{\n\
         {indent}    // r0 = ||F({v})||: reuses the device currents `{inl}` the NR body\n\
         {indent}    // already evaluated at this exact {v} (bit-identical to a fresh\n\
         {indent}    // kcl_residual — same N_v·v, same device fn, no pnjlim on the eval).\n\
         {indent}    let (r0, _, _) = kcl_residual_inl(&{v}, &{rhs}, &{amat}, &{inl});\n\
         {indent}    if r0.is_finite() && r0 > 1e-9 {{\n\
         {indent}        let mut s = 1.0_f64;\n\
         {indent}        let mut accepted = false;\n\
         {indent}        while s >= (1.0 / 1024.0) {{\n\
         {indent}            let mut vc = {v};\n\
         {indent}            for i in 0..N {{ vc[i] += ({alpha} * s) * ({new}[i] - {v}[i]); }}\n\
         {indent}            let (rc, _, _) = kcl_residual(&vc, &{rhs}, &{amat}, state);\n\
         {indent}            if rc <= (1.0 - 1e-4 * s) * r0 {{ accepted = true; break; }}\n\
         {indent}            s *= 0.5;\n\
         {indent}        }}\n\
         {indent}        if accepted {{ {alpha} *= s; }} else {{ {ls_ok} = false; }}\n\
         {indent}    }}\n\
         {indent}}}\n"
    ));
}
