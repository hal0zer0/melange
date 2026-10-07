//! Codegen-time matrix helpers — LU inversion, K computation, sparsity
//! analysis, MNA stamping primitives for flat row-major matrices.
//!
//! All functions are pure transformations on `[f64]` slices indexed
//! `[i * n + j]`. The equilibrated LU inversion in `invert_flat_matrix`
//! keeps `cond(A)` inside f64 precision regardless of unit imbalance
//! between G_in and internal conductances. Keep in sync with
//! `dc_op::equilibrate` and the runtime DK `invert_n_equilibrated`.

use crate::mna::MnaSystem;

use crate::structural::{self, Pattern};

use super::{CodegenError, Matrices, MatrixSparsity, LU_PIVOT_EPSILON};

/// The sparsity of a matrix assembled directly from stamps (`A_neg = alpha·C`,
/// `C/T`, `N_v`, `N_i`): the positions holding a value other than exactly
/// `0.0`. No inversion has run on these, so a written position carries its
/// stamp's value, never rounding noise. Matrices formed through `A⁻¹` (S, K)
/// take their pattern from [`settle_structural_sparsity`] instead.
pub(super) fn analyze_matrix_sparsity(data: &[f64], rows: usize, cols: usize) -> MatrixSparsity {
    sparsity_of_pattern(&structural::nonzero_pattern(data, rows, cols))
}

/// A boolean pattern as the emitters' per-row column lists.
pub(super) fn sparsity_of_pattern(pattern: &Pattern) -> MatrixSparsity {
    let rows = pattern.len();
    let cols = pattern.first().map_or(0, Vec::len);
    let nz_by_row: Vec<Vec<usize>> = pattern
        .iter()
        .map(|r| (0..r.len()).filter(|&j| r[j]).collect())
        .collect();
    MatrixSparsity {
        rows,
        cols,
        nnz: nz_by_row.iter().map(Vec::len).sum(),
        nz_by_row,
    }
}

/// The K positions of every parasitic-absorbed BJT's 2×2 block: on the DK
/// route `state.k` holds `K − R_p` there (`dk_emitter::parasitic_r_p_dk` and
/// `k_eff_adjust_stmts`, same gate), so the block must be in K's pattern even
/// where `N_v·S·N_i` is structurally zero. The DK emitter refuses a K pattern
/// that misses one.
pub(super) fn parasitic_bjt_k_positions(
    device_slots: &[crate::device_types::DeviceSlot],
) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    for slot in device_slots {
        if let crate::device_types::DeviceParams::Bjt(bp) = &slot.params {
            if bp.has_parasitics() && !slot.has_internal_mna_nodes && slot.dimension == 2 {
                let s = slot.start_idx;
                out.extend([(s, s), (s, s + 1), (s + 1, s), (s + 1, s + 1)]);
            }
        }
    }
    out
}

/// Safety factor `c` in the rounding bound `c·n·eps·cond∞(Â)·max|Ŝ|` (on the
/// equilibrated system) that entries outside the structural pattern must
/// meet. The classical
/// backward-error result for an inverse formed by pivoted elimination bounds
/// the error of each entry by a small multiple of `n·eps·cond(A)·|S|`; 10
/// covers that multiple with room, so a deck that meets the theory is never
/// refused for it.
const STRUCTURAL_NOISE_SAFETY: f64 = 10.0;

/// The outcome of [`settle_structural_sparsity`].
pub(super) struct StructuralSettlement {
    /// Structural pattern of `A = G + alpha·C` (and of the backward-Euler
    /// `G + C/T`): the union of G's and C's stamped positions, so no rate makes
    /// an entry vanish by cancellation.
    pub a: Pattern,
    /// Structural pattern of `K = N_v·S·N_i` (shared by `K_BE`), plus any
    /// positions the caller adds (`extra_k`).
    pub k: Pattern,
    /// The largest `|entry| / bound` over the entries of S, S_BE, K and K_BE
    /// outside their structural pattern, before they were set to zero. At most
    /// 1 on every build that is not refused.
    pub noise_ratio: f64,
}

/// Infinity norm (largest absolute row sum) of a flat row-major matrix.
fn norm_inf(a: &[f64], rows: usize, cols: usize) -> f64 {
    (0..rows)
        .map(|i| {
            a[i * cols..(i + 1) * cols]
                .iter()
                .map(|v| v.abs())
                .sum::<f64>()
        })
        .fold(0.0, f64::max)
}

/// Decide which entries of the inverse-derived matrices exist from the
/// circuit's structure (see [`crate::structural`]), then make the stored
/// values agree with it.
///
/// 1. `A`'s pattern is the stamped positions of G and C; `S = A⁻¹` has the
///    structural inverse pattern, and `K = N_v·S·N_i` the boolean product.
///    `extra_k` adds K positions the emitted code writes on top of `N_v·S·N_i`
///    (the DK route's parasitic-BJT block).
/// 2. An entry of S, S_BE, K or K_BE outside its pattern is zero in exact
///    arithmetic; the computed value there is rounding noise from the
///    inversion. Each must lie under its own rounding bound, derived from the
///    equilibrated condition number of the matrix the inversion worked on
///    (see the bound comment below). A larger one means the pattern missed a stamp: the
///    build is refused, naming the entry.
/// 3. Those entries are then set to exactly `0.0`, so the shipped constants
///    and every runtime seed carry no noise.
///
/// `a` is the matrix `S` was inverted from and `a_be` the one `S_BE` was
/// (required whenever `S_BE` is present), so the condition estimate is of the
/// matrix the inversion actually worked on.
pub(super) fn settle_structural_sparsity(
    matrices: &mut Matrices,
    n: usize,
    m: usize,
    extra_k: &[(usize, usize)],
    a: &[f64],
    a_be: Option<&[f64]>,
) -> Result<StructuralSettlement, CodegenError> {
    let mx = matrices;
    if mx.g_matrix.len() != n * n || mx.c_matrix.len() != n * n || mx.s.len() != n * n {
        return Err(CodegenError::InvalidConfig(format!(
            "structural sparsity: G, C and S must be {n}x{n} (lengths {}, {}, {})",
            mx.g_matrix.len(),
            mx.c_matrix.len(),
            mx.s.len()
        )));
    }
    let g = structural::nonzero_pattern(&mx.g_matrix, n, n);
    let c = structural::nonzero_pattern(&mx.c_matrix, n, n);
    let a_pat: Pattern = g
        .iter()
        .zip(&c)
        .map(|(gr, cr)| gr.iter().zip(cr).map(|(x, y)| *x || *y).collect())
        .collect();
    let s_pat = structural::inverse_pattern(&a_pat).ok_or_else(|| {
        CodegenError::InvalidConfig(
            "structural sparsity: G + C is structurally singular (no assignment of \
             equations to unknowns exists), yet its inverse was computed"
                .to_string(),
        )
    })?;
    let mut k = if m > 0 {
        let nv = structural::nonzero_pattern(&mx.n_v, m, n);
        let ni = structural::nonzero_pattern(&mx.n_i, n, m);
        structural::product(&structural::product(&nv, &s_pat), &ni)
    } else {
        Vec::new()
    };
    for &(i, j) in extra_k {
        k[i][j] = true;
    }

    // Per-entry rounding bounds for an inverse `s` of `a`, following how it
    // was computed: `s = diag(dc)·Ŝ·diag(dr)` where `Ŝ` inverts the
    // equilibrated `Â = diag(dr)·a·diag(dc)` (the scales `invert_flat_matrix`
    // applied). The pivoted inversion of `Â` errs by at most
    // `E = c·n·eps·cond∞(Â)·max|Ŝ|` per entry, so `s[i][j]` errs by at most
    // `E·dc[i]·dr[j]`, and `K = N_v·s·N_i` by at most
    // `E·(Σ_a |N_v[i][a]|·dc[a])·(Σ_b dr[b]·|N_i[b][j]|)`.
    struct Bounds {
        e: f64,
        dr: Vec<f64>,
        dc: Vec<f64>,
    }
    let bounds_of = |name: &str, a: &[f64], s: &[f64]| -> Result<Bounds, CodegenError> {
        if a.len() != n * n {
            return Err(CodegenError::InvalidConfig(format!(
                "structural sparsity: the matrix {name} was inverted from must be {n}x{n} \
                 (length {})",
                a.len()
            )));
        }
        let (dr, dc) = equilibration_scales(a, n);
        let mut a_hat = vec![0.0; n * n];
        let mut s_hat = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..n {
                a_hat[i * n + j] = a[i * n + j] * dr[i] * dc[j];
                s_hat[i * n + j] = s[i * n + j] / (dc[i] * dr[j]);
            }
        }
        let cond = norm_inf(&a_hat, n, n) * norm_inf(&s_hat, n, n);
        let max_hat = s_hat.iter().fold(0.0f64, |acc, v| acc.max(v.abs()));
        Ok(Bounds {
            e: STRUCTURAL_NOISE_SAFETY * n as f64 * f64::EPSILON * cond * max_hat,
            dr,
            dc,
        })
    };
    let mut noise_ratio = 0.0f64;
    // Check every entry outside `pat` against its bound, then zero it.
    let mut settle = |name: &str,
                      vals: &mut Vec<f64>,
                      pat: &Pattern,
                      dim: usize,
                      bound: &dyn Fn(usize, usize) -> f64|
     -> Result<(), CodegenError> {
        if vals.len() != dim * dim || dim == 0 {
            return Ok(());
        }
        for i in 0..dim {
            for j in 0..dim {
                let v = &mut vals[i * dim + j];
                if pat[i][j] || *v == 0.0 {
                    continue;
                }
                let b = bound(i, j);
                let ratio = v.abs() / b;
                if ratio.is_nan() || ratio > 1.0 {
                    return Err(CodegenError::InvalidConfig(format!(
                        "structural sparsity: {name}[{i}][{j}] = {v:e} lies outside the \
                         structural pattern but above its rounding bound {b:e} (derived \
                         from the equilibrated condition number of the matrix the \
                         inversion used). The pattern missed a stamped position; this is \
                         a melange bug, please report it with the deck."
                    )));
                }
                noise_ratio = noise_ratio.max(ratio);
                *v = 0.0;
            }
        }
        Ok(())
    };
    // Weights carrying an S error into K: row i of N_v against dc, column j of
    // N_i against dr.
    let k_weights = |b: &Bounds| -> (Vec<f64>, Vec<f64>) {
        let wv = (0..m)
            .map(|i| (0..n).map(|a| mx.n_v[i * n + a].abs() * b.dc[a]).sum())
            .collect();
        let wi = (0..m)
            .map(|j| (0..n).map(|r| b.dr[r] * mx.n_i[r * m + j].abs()).sum())
            .collect();
        (wv, wi)
    };
    let sb = bounds_of("S", a, &mx.s)?;
    let (wv, wi) = k_weights(&sb);
    settle("K", &mut mx.k, &k, m, &|i, j| sb.e * wv[i] * wi[j])?;
    settle("S", &mut mx.s, &s_pat, n, &|i, j| {
        sb.e * sb.dc[i] * sb.dr[j]
    })?;
    if mx.s_be.len() == n * n {
        let a_be = a_be.ok_or_else(|| {
            CodegenError::InvalidConfig(
                "structural sparsity: S_BE is present but the matrix it was inverted from \
                 was not supplied"
                    .to_string(),
            )
        })?;
        let bb = bounds_of("S_BE", a_be, &mx.s_be)?;
        let (wv, wi) = k_weights(&bb);
        settle("K_BE", &mut mx.k_be, &k, m, &|i, j| bb.e * wv[i] * wi[j])?;
        settle("S_BE", &mut mx.s_be, &s_pat, n, &|i, j| {
            bb.e * bb.dc[i] * bb.dr[j]
        })?;
    }
    Ok(StructuralSettlement {
        a: a_pat,
        k,
        noise_ratio,
    })
}

// Sparsity analysis functions (compute_g_aug_pattern, amd_ordering,
// find_row_swaps, symbolic_lu) are defined in crate::lu and called
// via lu::compute_g_aug_pattern(...) etc. at the call sites below.

/// The row and column scales `(dr, dc)` that [`invert_flat_matrix`] applies
/// before factoring: each row divided by its largest magnitude, then each
/// column of the result by its largest. The equilibrated matrix is
/// `diag(dr)·A·diag(dc)`.
fn equilibration_scales(a: &[f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    let scale = |max: f64| {
        if max > LU_PIVOT_EPSILON {
            1.0 / max
        } else {
            1.0
        }
    };
    let dr: Vec<f64> = (0..n)
        .map(|i| {
            scale(
                a[i * n..(i + 1) * n]
                    .iter()
                    .fold(0.0f64, |m, v| m.max(v.abs())),
            )
        })
        .collect();
    let dc: Vec<f64> = (0..n)
        .map(|j| scale((0..n).fold(0.0f64, |m, i| m.max((a[i * n + j] * dr[i]).abs()))))
        .collect();
    (dr, dc)
}

/// Invert a flat row-major N×N matrix using Gaussian elimination with partial pivoting.
///
/// Returns `CodegenError::InvalidConfig` if the matrix is singular.
pub(super) fn invert_flat_matrix(a: &[f64], n: usize) -> Result<Vec<f64>, CodegenError> {
    // Asymmetric row/column equilibration before factorisation: keeps
    // cond(A) inside f64 precision when G_in (≈1 S) dominates internal
    // conductances (1e-4 to 1e-6 S) by 4-6 decades. Matches the runtime
    // DK `invert_n_equilibrated` and DC-OP `equilibrate` helpers.
    let (dr, dc) = equilibration_scales(a, n);
    let mut a_eq = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..n {
            a_eq[i * n + j] = a[i * n + j] * dr[i] * dc[j];
        }
    }

    // Build augmented [A_eq | I]
    let mut aug = vec![0.0f64; n * 2 * n];
    for i in 0..n {
        for j in 0..n {
            aug[i * 2 * n + j] = a_eq[i * n + j];
        }
        aug[i * 2 * n + n + i] = 1.0;
    }

    let w = 2 * n;
    for col in 0..n {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = aug[col * w + col].abs();
        for row in (col + 1)..n {
            let v = aug[row * w + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < LU_PIVOT_EPSILON {
            return Err(CodegenError::InvalidConfig(format!(
                "Matrix is singular (pivot {:.2e} at row {}) — check for floating nodes or missing ground path",
                max_val, col
            )));
        }
        if max_row != col {
            for j in 0..w {
                aug.swap(col * w + j, max_row * w + j);
            }
        }
        let pivot = aug[col * w + col];
        for row in (col + 1)..n {
            let factor = aug[row * w + col] / pivot;
            for j in col..w {
                aug[row * w + j] -= factor * aug[col * w + j];
            }
        }
    }

    // Back substitution
    for col in (0..n).rev() {
        let pivot = aug[col * w + col];
        if pivot.abs() < LU_PIVOT_EPSILON {
            return Err(CodegenError::InvalidConfig(format!(
                "Matrix is singular (pivot {:.2e} at row {}) — check for floating nodes or missing ground path",
                pivot.abs(),
                col
            )));
        }
        for j in 0..w {
            aug[col * w + j] /= pivot;
        }
        for row in 0..col {
            let factor = aug[row * w + col];
            for j in 0..w {
                aug[row * w + j] -= factor * aug[col * w + j];
            }
        }
    }

    // Extract the equilibrated inverse, then de-equilibrate:
    // A_eq^-1 → A^-1 = D_c · A_eq^-1 · D_r
    let mut result = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..n {
            result[i * n + j] = dc[i] * aug[i * w + n + j] * dr[j];
        }
    }
    Ok(result)
}

/// Compute K = N_v * S * N_i from flat row-major matrices.
///
/// N_v is M×N, S is N×N, N_i is N×M (all flat row-major).
pub(super) fn compute_k_from_s(
    s: &[f64],
    n_v: &[f64],
    n_i: &[f64],
    n: usize,
    m: usize,
) -> Vec<f64> {
    // First compute S * N_i → S_NI (N×M)
    let mut s_ni = vec![0.0f64; n * m];
    for i in 0..n {
        for j in 0..m {
            let mut sum = 0.0;
            for k in 0..n {
                sum += s[i * n + k] * n_i[k * m + j];
            }
            s_ni[i * m + j] = sum;
        }
    }
    // Then K = N_v * S_NI → K (M×M)
    let mut k = vec![0.0f64; m * m];
    for i in 0..m {
        for j in 0..m {
            let mut sum = 0.0;
            for ki in 0..n {
                sum += n_v[i * n + ki] * s_ni[ki * m + j];
            }
            k[i * m + j] = sum;
        }
    }
    k
}

/// Validate that a device model parameter is positive and finite.
pub(super) fn validate_positive_finite(value: f64, param_label: &str) -> Result<(), CodegenError> {
    if value <= 0.0 || !value.is_finite() {
        return Err(CodegenError::InvalidConfig(format!(
            "{param_label} must be positive finite, got {value}"
        )));
    }
    Ok(())
}

/// Compute backward Euler fallback matrices for the DK codegen path.
///
/// Returns (s_be, k_be, a_neg_be, rhs_const_be) or empty vecs if BE fallback is disabled.
/// The BE matrices use alpha_be = 1/T (instead of trapezoidal alpha = 2/T).
///
/// Inductors are augmented branch rows (L in the C matrix), for which
/// `A_be = G + (1/T)·C`, `A_neg_be = (1/T)·C` is the exact BE discretization of
/// the branch equations.
pub(super) fn compute_dk_be_fallback(
    g_matrix: &[f64],
    c_matrix: &[f64],
    n: usize,
    m: usize,
    n_nodes: usize,
    n_v: &[f64],
    n_i: &[f64],
    internal_rate: f64,
    mna: &MnaSystem,
) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>), CodegenError> {
    let alpha_be = internal_rate; // BE: alpha = 1/T

    // Build A_be = G + alpha_be * C
    let mut a_be = vec![0.0f64; n * n];
    let mut a_neg_be = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..n {
            let g = g_matrix[i * n + j];
            let c = c_matrix[i * n + j];
            a_be[i * n + j] = g + alpha_be * c;
            a_neg_be[i * n + j] = alpha_be * c; // BE: no -G term
        }
    }

    // #5: Blanket-zero ALL augmented algebraic rows (n_nodes..n_aug) in
    // A_neg_be — including the Boyle op-amp internal / current-mode VCA /
    // behavioral-V rows the former per-type (VS/VCVS/xfmr) enumeration missed,
    // which left stale trapezoidal history on those BE constraints. Shared with
    // the trap-path and os>1 builders via super::zero_augmented_history_rows.
    // Inductor branch rows (n_aug..n) keep their history and are untouched.
    super::zero_augmented_history_rows(
        &mut a_neg_be,
        n,
        n_nodes,
        mna.n_aug,
        &mna.bjt_internal_nodes,
    );

    // S_be = A_be^{-1}
    let s_be = invert_flat_matrix(&a_be, n)?;

    // K_be = N_v * S_be * N_i
    let k_be = if m > 0 {
        compute_k_from_s(&s_be, n_v, n_i, n, m)
    } else {
        Vec::new()
    };

    // BE rhs_const: current sources ×1 (not ×2), VS ×1
    let mut rhs_const_be = vec![0.0f64; n];
    for src in &mna.current_sources {
        crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_plus_idx, src.dc_value);
        crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_minus_idx, -src.dc_value);
    }
    for vs in &mna.voltage_sources {
        let k_row = n_nodes + vs.ext_idx;
        if k_row < n {
            rhs_const_be[k_row] = vs.dc_value;
        }
    }

    Ok((s_be, k_be, a_neg_be, rhs_const_be, a_be))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mna::MnaSystem;

    /// #5 regression: `compute_dk_be_fallback` must zero EVERY augmented
    /// algebraic row (`n_nodes..n_aug`) in `A_neg_be`, not just the
    /// VS/VCVS/ideal-transformer rows the old per-type enumeration handled.
    /// A current-mode VCA (internal + sense branch), a behavioral `V={}`
    /// source, or a Boyle op-amp adds an augmented row beyond those three
    /// types; leaving stale trapezoidal history there feeds a spurious z=-1
    /// term into an algebraic constraint on the BE path.
    ///
    /// The MNA here has one such extra augmented row (`n_aug = n_nodes + 1`)
    /// with NO voltage sources / VCVS / transformers — so the pre-fix code
    /// would have zeroed nothing and left `alpha*C = 48000` on that row.
    #[test]
    fn be_fallback_zeros_all_augmented_rows_not_just_vs_vcvs() {
        let n_nodes = 2usize;
        let n = 3usize; // index 2 is a VCA/behavioral-style augmented row
        let mut mna = MnaSystem::new(n_nodes, 0, 0, 0);
        mna.n_aug = n; // augmented row present, but not a VS/VCVS/xfmr
        assert!(mna.voltage_sources.is_empty());
        assert!(mna.vcvs_sources.is_empty());
        assert!(mna.ideal_transformers.is_empty());

        // G = I so A_be = G + alpha*C is invertible; C nonzero only on the
        // augmented row, so its A_neg_be entry is nonzero before zeroing.
        let mut g = vec![0.0f64; n * n];
        let mut c = vec![0.0f64; n * n];
        for d in 0..n {
            g[d * n + d] = 1.0;
        }
        c[2 * n + 2] = 1.0;

        let internal_rate = 48_000.0;
        let (_s_be, _k_be, a_neg_be, _rhs, _a_be) =
            compute_dk_be_fallback(&g, &c, n, 0, n_nodes, &[], &[], internal_rate, &mna)
                .expect("BE fallback build");

        // The augmented row (2) must be fully zeroed despite not being a
        // VS/VCVS/xfmr row. Pre-#5, a_neg_be[2*n + 2] would be 48000.0.
        for j in 0..n {
            assert_eq!(
                a_neg_be[2 * n + j],
                0.0,
                "augmented row 2 not zeroed at col {j}"
            );
        }
    }
}
