//! Saturating-inductor branch-row emission.

use crate::codegen::ir::CircuitIR;

/// Newton step limiter on each saturating inductor's branch-current row.
///
/// Node damping covers node rows and pnjlim covers devices; nothing limited the
/// augmented flux rows. From deep saturation (`L_diff ~ L_air`) a Newton step
/// crossing the knee is amps long and lands deep on the other side, where the
/// slope is flat again: Newton 2-cycles and exhausts MAX_ITER. Like pnjlim, the
/// limit is a ratio on the shared step fraction `alpha` (so the Newton
/// direction is kept), unfloored, since a deep-saturation overshoot needs far
/// less than 1 %.
pub(super) fn emit_sat_ind_step_limit(
    code: &mut String,
    ir: &CircuitIR,
    v: &str,
    v_new: &str,
    alpha: &str,
    indent: &str,
) {
    // Only a step that crosses the knee (|i| = Isat, or a sign change) and
    // lands more than Isat past it is scaled, to land at 2·Isat on its side.
    // Steps within one regime (both deep, or both below the knee) are nearly
    // linear and pass unscaled. Measured against a blanket
    // |Δi| ≤ 2·Isat + 0.5·|i|: both removed every MAX_ITER exhaustion on the
    // square-wave witnesses, but the blanket form throttled converging steps
    // (99 V 1 kHz square: 2.749 vs 2.666 iterations/sample) and moved
    // converged outputs (up to 2.8e-7); this form is bit-identical to the
    // unlimited build wherever that build converged.
    for (idx, _si) in ir.saturating_inductors.iter().enumerate() {
        code.push_str(&format!(
            "{indent}{{ let i0 = {v}[SAT_IND_{idx}_AUG_ROW]; let i1 = {v_new}[SAT_IND_{idx}_AUG_ROW]; \
             let is = SAT_IND_{idx}_ISAT; \
             if (i0 * i1 < 0.0 || (i0.abs() > is) != (i1.abs() > is)) && i1.abs() > 2.0 * is {{ \
             let r = (i1.signum() * 2.0 * is - i0) / (i1 - i0); \
             if r < {alpha} {{ {alpha} = r; }} }} }}\n"
        ));
    }
}

/// History correction (once per sample, after the base `A_neg·v_prev` RHS build):
/// swaps the baked-in `alpha·L0·i_prev` for `alpha·Φ(i_prev)`.
pub(super) fn emit_sat_ind_history(
    code: &mut String,
    ir: &CircuitIR,
    rhs: &str,
    vprev: &str,
    alpha: &str,
    indent: &str,
) {
    for (idx, _si) in ir.saturating_inductors.iter().enumerate() {
        code.push_str(&format!(
            "{indent}{{ let ip = {vprev}[SAT_IND_{idx}_AUG_ROW]; \
             let phi = SAT_IND_{idx}_LMAG * SAT_IND_{idx}_ISAT * (ip / SAT_IND_{idx}_ISAT).tanh() + SAT_IND_{idx}_LAIR * ip; \
             {rhs}[SAT_IND_{idx}_AUG_ROW] += {alpha} * (phi - SAT_IND_{idx}_L0 * ip); }}\n"
        ));
    }
}

/// Charge-form history on a saturating inductor's branch row: the linear
/// `alpha·L0·Δi` that `H·(v − v_prev)` put in `q` becomes `alpha·ΔΦ`.
pub(super) fn emit_sat_ind_q_dot(
    code: &mut String,
    ir: &CircuitIR,
    q: &str,
    v_new: &str,
    v_old: &str,
    alpha: &str,
    indent: &str,
) {
    for (idx, _si) in ir.saturating_inductors.iter().enumerate() {
        code.push_str(&format!(
            "{indent}{{ let flux = |i: f64| SAT_IND_{idx}_LMAG * SAT_IND_{idx}_ISAT * (i / SAT_IND_{idx}_ISAT).tanh() + SAT_IND_{idx}_LAIR * i - SAT_IND_{idx}_L0 * i; \
             {q}[SAT_IND_{idx}_AUG_ROW] += {alpha} * (flux({v_new}[SAT_IND_{idx}_AUG_ROW]) - flux({v_old}[SAT_IND_{idx}_AUG_ROW])); }}\n"
        ));
    }
}

/// Jacobian correction (at each factor): base matrix has `alpha·L0` at `[k][k]`;
/// correct it to `alpha·L_diff(i0)`. `iterate` is the current NR iterate vector.
pub(super) fn emit_sat_ind_jacobian(
    code: &mut String,
    ir: &CircuitIR,
    mat: &str,
    iterate: &str,
    alpha: &str,
    indent: &str,
) {
    for (idx, _si) in ir.saturating_inductors.iter().enumerate() {
        code.push_str(&format!(
            "{indent}{{ let i0 = {iterate}[SAT_IND_{idx}_AUG_ROW]; \
             let cx = (i0 / SAT_IND_{idx}_ISAT).clamp(-40.0, 40.0).cosh(); \
             let ld = (SAT_IND_{idx}_LMAG / (cx * cx) + SAT_IND_{idx}_LAIR).max(SAT_IND_{idx}_L0 * 1e-6); \
             {mat}[SAT_IND_{idx}_AUG_ROW][SAT_IND_{idx}_AUG_ROW] += {alpha} * (ld - SAT_IND_{idx}_L0); }}\n"
        ));
    }
}

/// Companion-RHS correction (consistent with the factored Jacobian, same
/// `iterate`): moves the linearization constant `alpha·(L_diff(i0)·i0 − Φ(i0))`
/// to the RHS so the back-solve yields the Newton step.
pub(super) fn emit_sat_ind_companion(
    code: &mut String,
    ir: &CircuitIR,
    rhs: &str,
    iterate: &str,
    alpha: &str,
    indent: &str,
) {
    for (idx, _si) in ir.saturating_inductors.iter().enumerate() {
        code.push_str(&format!(
            "{indent}{{ let i0 = {iterate}[SAT_IND_{idx}_AUG_ROW]; \
             let cx = (i0 / SAT_IND_{idx}_ISAT).clamp(-40.0, 40.0).cosh(); \
             let ld = (SAT_IND_{idx}_LMAG / (cx * cx) + SAT_IND_{idx}_LAIR).max(SAT_IND_{idx}_L0 * 1e-6); \
             let phi = SAT_IND_{idx}_LMAG * SAT_IND_{idx}_ISAT * (i0 / SAT_IND_{idx}_ISAT).tanh() + SAT_IND_{idx}_LAIR * i0; \
             {rhs}[SAT_IND_{idx}_AUG_ROW] += {alpha} * (ld * i0 - phi); }}\n"
        ));
    }
}

/// Remove one balanced enclosing paren pair from an emitted expression, so it
/// can be bound with `let` without tripping `unused_parens` in generated code
/// compiled under `-D warnings`. Returns the input unchanged unless the first
/// character's paren closes exactly at the last character.
fn strip_outer_parens(expr: &str) -> &str {
    let t = expr.trim();
    if !t.starts_with('(') || !t.ends_with(')') {
        return t;
    }
    let mut depth = 0usize;
    for (i, ch) in t.char_indices() {
        match ch {
            '(' => depth += 1,
            ')' => {
                depth -= 1;
                // The opening paren closed before the end: not an enclosing pair
                // (e.g. `(a) * (b)`), so the parens are not removable.
                if depth == 0 && i + ch.len_utf8() != t.len() {
                    return t;
                }
            }
            _ => {}
        }
    }
    &t[1..t.len() - 1]
}

/// Emit the post-step consistency check for the saturating-inductor augmented
/// rows: the exact analogue, for the flux rows, of the device residual check
/// that `emit_nodal_process_sample` already emits for `i_nl`.
///
/// ## The check
///
/// For saturating inductor `s` on augmented row `k`, the converged row equation
/// (see `SATURATING_TRANSFORMERS.md` §3.2 for the augmented-row sign
/// convention) is
///
/// ```text
/// R_k = Σ_{j≠k} A[k][j]·v[j]        incidence / ideal-transformer couplings
///     + (A[k][k] − alpha·L0)·v[k]   the alpha·L0 self term cancels; see below
///     − Σ_i N_I[k][i]·i_nl(v)[i]    structurally empty on augmented rows
///     + ( alpha·Φ(v[k]) − rhs[k] )  the flux-change term; see "pairing" below
///     == 0
/// ```
///
/// The `alpha·L_diff(i0)` Jacobian stamp and the `alpha·(L_diff(i0)·i0 − Φ(i0))`
/// companion stamp cancel identically at any fixed point, so `R_k` is
/// independent of *which* iterate the linearization was frozen at. That is
/// precisely what makes it a usable residual: it is built from the row's
/// EQUATION, not from the iteration, so it can see a Jacobian/companion
/// inconsistency as well as a stale chord.
///
/// ## Why the obvious cheaper check (Φ-vs-Φ) does NOT work
///
/// The tempting analogue is to hold the `Φ(i0)` that fed the companion and
/// compare it against `Φ(v_post[k])` after the step — mirroring
/// `i_nl` vs `i_nl_chord`. **That test can never fire, and it is worth
/// recording why so it is not re-proposed.**
///
/// 1. The augmented row `k` is ALREADY in the emitted voltage-step convergence
///    list, so on the accepting iteration `|Δi_k| ≤ RELTOL·max|i_k| + 1e-6`
///    already holds.
/// 2. `Φ(i) = L_mag·Isat·tanh(i/Isat) + L_air·i` is Lipschitz with constant
///    `L0`, because `L_diff(i) = L_mag/cosh²(i/Isat) + L_air ≤ L0`. Hence
///    `|Φ(i_post) − Φ(i_pre)| ≤ L0·(RELTOL·max|i_k| + 1e-6)`.
/// 3. Any tolerance for that comparison that is dimensionally consistent with
///    the row's own step check — `RELTOL·max|Φ| + L0·1e-6` — equals that bound
///    in the small-signal limit (`Φ → L0·i`), and in saturation the local slope
///    `1/cosh²(x)` shrinks faster than `|Φ|/(L0·|i|) = tanh(x)/x`. The fire
///    condition reduces to `2x/sinh(2x) > 1`, which is false for every `x > 0`.
///
/// So the drift test is inert at every operating point and gets MORE inert the
/// deeper the knee. Measured on all five shipping `ISAT=` circuits: zero fires
/// across silence / 1 kHz / log sweep / step and a forced knee at `i/Isat` =
/// 14.8. Picking a tighter floor to make it fire only produces spurious fires
/// at flux zero crossings (up to 93 % of samples, +47 % NR iterations) and
/// still catches nothing, because a self-consistency test cannot see an
/// equation that is wrong at its own fixed point.
///
/// ## Three numerical choices, all load-bearing
///
/// * **The `alpha·L0·v[k]` self term is removed analytically, not by
///   subtraction.** `alpha·L0` reaches ~2.6e5 on a console-preamp output transformer
///   while the row's actual content is volts; folding `A[k][k]·v[k]` into the
///   sum and relying on the later terms to cancel it would discard ~11
///   significant digits. `A[k][k] = g[k][k] + alpha·C[k][k]` and
///   `C[k][k] = L0`, so the residue is exactly `g[k][k]` — a codegen-time
///   constant (structurally **zero** on every shipping circuit, since
///   `build_augmented_matrices` stamps only the ±1 incidence entries into row
///   `k`). When a `.switch L` can rewrite `C[k][k]` at runtime the constant is
///   not known, and the emitted fallback `(A[k][k] − alpha·L0)·v[k]` is still
///   exact: the two operands are within a factor of two, so the subtraction is
///   exact by Sterbenz.
/// * **`alpha·Φ(v[k])` and `rhs[k]` are summed as a pair BEFORE either reaches
///   the denominator.** `rhs[k]` carries `alpha·Φ(i_prev)`; individually both
///   are ~1e4 while their difference is the ~10 V the row is actually about.
///   Letting the stiff magnitude set the tolerance would make the check
///   unfireable — the same stiff-denominator trap the Φ-vs-Φ form falls into.
/// * **The tolerance is `1e-5·den`, floored at `64·eps` of the stiff pair.**
///   `den` is the largest term after pairing, i.e. the row's per-sample
///   increment (`alpha·ΔΦ`, the volts across the winding), not the flux
///   itself. Whatever residual is accepted here is integrated into the flux
///   history, and under a DC bias it has one sign (Newton's remainder
///   `alpha·Φ''·Δi²/2`, with `Φ''` signed by the bias), so it accumulates with
///   the L/R time constant instead of averaging out. The old `1e-3·den + 1e-6`
///   accepted the first Newton iterate on every sample and drifted a 5 mA
///   biased core by −1.6e-5 A over 2 s; at `1e-5` the drift is ~1e-10 A, for
///   about one more iteration per sample. Tighter buys nothing at that point
///   and leaves less room above the rounding floor at waveform turning points,
///   where `den` goes to zero. The floor covers what the pairing
///   cannot remove: rounding in `alpha·Φ − rhs[k]` is a few ulp of the
///   operands, not of their difference. A fixed absolute floor would reopen
///   the same drift at small increments.
///
/// The predicate is written negated (`!(x <= tol)`) so a NaN residual reads as
/// NOT converged, matching the device check.
///
/// ## Scope
///
/// Every Newton site that can commit a sample: the main trapezoidal/BE loop,
/// the adaptive sub-step, the backward-Euler fallback and the op-amp
/// active-set pinned Newton, so a sample counts as solved by one definition
/// wherever it was solved. `v`, `mat` and `flag` name the site's iterate, its
/// base matrix (`A = G + alpha·C` at the site's alpha: mutual-inductance
/// entries scale with it) and the not-converged flag the check sets.
#[allow(clippy::too_many_arguments)]
pub(super) fn emit_sat_ind_row_residual(
    code: &mut String,
    ir: &CircuitIR,
    setter_stamps: &std::collections::BTreeSet<(usize, usize)>,
    rhs: &str,
    alpha: &str,
    i_nl: Option<&str>,
    v: &str,
    mat: &str,
    flag: &str,
    indent: &str,
) {
    if ir.saturating_inductors.is_empty() {
        return;
    }
    let n = ir.topology.n;
    let m = ir.topology.m;
    let g = &ir.matrices.g_matrix;
    let c = &ir.matrices.c_matrix;
    // Guard: the row enumeration below reads G/C directly, so bail to no check
    // at all rather than emit a residual over a matrix we cannot address.
    if g.len() != n * n || c.len() != n * n {
        log::warn!("sat-inductor row residual: unexpected G/C dimensions, check not emitted");
        return;
    }

    code.push_str(&format!(
        "{indent}// Saturating-inductor augmented-row residual: the flux-row analogue\n\
         {indent}// of the device residual check above. See `emit_sat_ind_row_residual`.\n"
    ));
    // `alpha` arrives fully parenthesised because every other consumer splices
    // it into a larger expression. Binding it verbatim would trip
    // `unused_parens` in generated code compiled under `-D warnings`, so strip
    // one balanced outer pair (and only when it really is balanced).
    let sat_al_expr = strip_outer_parens(alpha);
    code.push_str(&format!("{indent}if !{flag} {{\n"));
    code.push_str(&format!("{indent}    let sat_al = {sat_al_expr};\n"));

    for (idx, si) in ir.saturating_inductors.iter().enumerate() {
        let k = si.aug_row;
        if k >= n {
            log::warn!(
                "sat-inductor {} aug_row {} out of range, row residual not emitted",
                si.name,
                k
            );
            continue;
        }
        // Structural nonzeros of row k of A = G + alpha*C, plus every position
        // a dynamic-parameter setter can write into that row. Exact equality
        // against 0.0 (not SPARSITY_THRESHOLD) so no small-but-real coupling is
        // ever dropped from the residual.
        let mut cols: Vec<usize> = (0..n)
            .filter(|&j| j != k && (g[k * n + j] != 0.0 || c[k * n + j] != 0.0))
            .collect();
        for &(a, b) in setter_stamps.iter() {
            if a == k && b != k && !cols.contains(&b) {
                cols.push(b);
            }
        }
        cols.sort_unstable();

        code.push_str(&format!(
            "{indent}    {{ // saturating inductor {idx} ({name})\n",
            name = si.name
        ));
        code.push_str(&format!(
            "{indent}        let k = SAT_IND_{idx}_AUG_ROW;\n\
             {indent}        let mut acc = 0.0f64;\n\
             {indent}        let mut den = 0.0f64;\n"
        ));
        for j in cols {
            code.push_str(&format!(
                "{indent}        {{ let t = {mat}[k][{j}] * {v}[{j}]; acc += t; \
                 let a = t.abs(); if a > den {{ den = a; }} }}\n"
            ));
        }
        // Diagonal: A[k][k]*v[k] minus the alpha*L0*v[k] that the flux term
        // below reintroduces. See the doc comment.
        let diag_is_static = c[k * n + k] == si.l0 && !setter_stamps.contains(&(k, k));
        if diag_is_static {
            let g_kk = g[k * n + k];
            if g_kk != 0.0 {
                code.push_str(&format!(
                    "{indent}        {{ let t = {g_kk:.17e} * {v}[k]; acc += t; \
                     let a = t.abs(); if a > den {{ den = a; }} }}\n"
                ));
            }
            // g[k][k] == 0.0: the whole self term vanishes, nothing to emit.
        } else {
            code.push_str(&format!(
                "{indent}        {{ let t = ({mat}[k][k] - sat_al * SAT_IND_{idx}_L0) * {v}[k]; \
                 acc += t; let a = t.abs(); if a > den {{ den = a; }} }}\n"
            ));
        }
        // Device currents injected into this row (structurally empty for an
        // inductor branch row, emitted only if N_I says otherwise). A site that
        // has no post-step i_nl to offer must not silently drop a nonzero one.
        if m > 0 && ir.matrices.n_i.len() == n * m {
            let injects = (0..m).any(|i| ir.matrices.n_i[k * m + i] != 0.0);
            assert!(
                !injects || i_nl.is_some(),
                "saturating inductor {} row {k} carries a device current, but this \
                 residual site has no i_nl to include",
                si.name
            );
        }
        if let Some(i_nl) = i_nl {
            if m > 0 && ir.matrices.n_i.len() == n * m {
                for i in 0..m {
                    if ir.matrices.n_i[k * m + i] != 0.0 {
                        code.push_str(&format!(
                            "{indent}        {{ let t = -N_I[k][{i}] * {i_nl}[{i}]; acc += t; \
                             let a = t.abs(); if a > den {{ den = a; }} }}\n"
                        ));
                    }
                }
            }
        }
        // Flux-change term, kept paired so the stiff alpha*Phi magnitude never
        // reaches `den`; it sets only the rounding floor.
        code.push_str(&format!(
            "{indent}        let phi = SAT_IND_{idx}_LMAG * SAT_IND_{idx}_ISAT \
             * ({v}[k] / SAT_IND_{idx}_ISAT).tanh() + SAT_IND_{idx}_LAIR * {v}[k];\n\
             {indent}        let ap = sat_al * phi;\n\
             {indent}        let stiff = ap.abs().max({rhs}[k].abs());\n\
             {indent}        {{ let t = ap - {rhs}[k]; acc += t; \
             let a = t.abs(); if a > den {{ den = a; }} }}\n"
        ));
        code.push_str(&format!(
            "{indent}        if !(acc.abs() <= (1e-5 * den).max(64.0 * f64::EPSILON * stiff)) {{ {flag} = true; }}\n"
        ));
        code.push_str(&format!("{indent}    }}\n"));
    }
    code.push_str(&format!("{indent}}}\n\n"));
}
