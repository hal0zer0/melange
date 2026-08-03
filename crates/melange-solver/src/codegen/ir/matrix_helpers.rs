//! Codegen-time matrix helpers — LU inversion, K computation, sparsity
//! analysis, MNA stamping primitives for flat row-major matrices.
//!
//! All functions are pure transformations on `[f64]` slices indexed
//! `[i * n + j]`. The equilibrated LU inversion in `invert_flat_matrix`
//! keeps `cond(A)` inside f64 precision regardless of unit imbalance
//! between G_in and internal conductances. Keep in sync with
//! `dc_op::equilibrate` and the runtime DK `invert_n_equilibrated`.

use crate::mna::MnaSystem;

use super::{CodegenError, MatrixSparsity, LU_PIVOT_EPSILON, SPARSITY_THRESHOLD};

pub(super) fn analyze_matrix_sparsity(data: &[f64], rows: usize, cols: usize) -> MatrixSparsity {
    let mut nnz = 0;
    let mut nz_by_row = Vec::with_capacity(rows);
    for i in 0..rows {
        let mut row_nz = Vec::new();
        for j in 0..cols {
            if data[i * cols + j].abs() >= SPARSITY_THRESHOLD {
                row_nz.push(j);
                nnz += 1;
            }
        }
        nz_by_row.push(row_nz);
    }
    MatrixSparsity {
        rows,
        cols,
        nnz,
        nz_by_row,
    }
}

// Sparsity analysis functions (compute_g_aug_pattern, amd_ordering,
// find_row_swaps, symbolic_lu) are defined in crate::lu and called
// via lu::compute_g_aug_pattern(...) etc. at the call sites below.

/// Stamp mutual conductance between two 2-terminal elements into a flat row-major matrix.
/// Node indices are 1-indexed; 0 means ground.
pub(super) fn stamp_flat_mutual(
    mat: &mut [f64],
    n: usize,
    a: usize,
    b: usize,
    c: usize,
    d: usize,
    g: f64,
) {
    if a > 0 && c > 0 {
        mat[(a - 1) * n + (c - 1)] += g;
    }
    if b > 0 && d > 0 {
        mat[(b - 1) * n + (d - 1)] += g;
    }
    if a > 0 && d > 0 {
        mat[(a - 1) * n + (d - 1)] -= g;
    }
    if b > 0 && c > 0 {
        mat[(b - 1) * n + (c - 1)] -= g;
    }
}

/// Stamp a conductance between two nodes into a flat row-major matrix.
/// Node indices are 1-indexed; 0 means ground.
pub(super) fn stamp_flat_conductance(
    mat: &mut [f64],
    n: usize,
    node_i: usize,
    node_j: usize,
    g: f64,
) {
    match (node_i > 0, node_j > 0) {
        (true, true) => {
            let i = node_i - 1;
            let j = node_j - 1;
            mat[i * n + i] += g;
            mat[j * n + j] += g;
            mat[i * n + j] -= g;
            mat[j * n + i] -= g;
        }
        (true, false) => {
            mat[(node_i - 1) * n + (node_i - 1)] += g;
        }
        (false, true) => {
            mat[(node_j - 1) * n + (node_j - 1)] += g;
        }
        (false, false) => {}
    }
}

/// Invert a flat row-major N×N matrix using Gaussian elimination with partial pivoting.
///
/// Returns `CodegenError::InvalidConfig` if the matrix is singular.
pub(super) fn invert_flat_matrix(a: &[f64], n: usize) -> Result<Vec<f64>, CodegenError> {
    // Asymmetric row/column equilibration before factorisation: keeps
    // cond(A) inside f64 precision when G_in (≈1 S) dominates internal
    // conductances (1e-4 to 1e-6 S) by 4-6 decades. Matches the runtime
    // DK `invert_n_equilibrated` and DC-OP `equilibrate` helpers.
    let mut dr = vec![1.0f64; n];
    let mut dc = vec![1.0f64; n];
    let mut a_eq = vec![0.0f64; n * n];
    a_eq.copy_from_slice(a);
    for i in 0..n {
        let mut row_max = 0.0_f64;
        for j in 0..n {
            let v = a_eq[i * n + j].abs();
            if v > row_max {
                row_max = v;
            }
        }
        dr[i] = if row_max > LU_PIVOT_EPSILON {
            1.0 / row_max
        } else {
            1.0
        };
    }
    for i in 0..n {
        for j in 0..n {
            a_eq[i * n + j] *= dr[i];
        }
    }
    for j in 0..n {
        let mut col_max = 0.0_f64;
        for i in 0..n {
            let v = a_eq[i * n + j].abs();
            if v > col_max {
                col_max = v;
            }
        }
        dc[j] = if col_max > LU_PIVOT_EPSILON {
            1.0 / col_max
        } else {
            1.0
        };
    }
    for i in 0..n {
        for j in 0..n {
            a_eq[i * n + j] *= dc[j];
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
/// # Companion-model magnetics are NOT supported here
///
/// `g_matrix`/`c_matrix` are the raw MNA matrices — they carry no
/// companion-model inductor / coupled-inductor / transformer-group stamps
/// (the shipped trap matrices get those separately via
/// `stamp_dk_companion_inductors`, with the trapezoidal `g_eq = T/(2L)`).
/// Consequently the A_be/S_be built here treat companion-modeled magnetics
/// as open circuits, and the per-sample BE RHS in process_sample.rs.tera
/// stamps no magnetics history currents at all. Callers must therefore gate
/// the BE fallback OFF when companion-path magnetics exist (done in
/// `CircuitIR::from_kernel_with_dc_op`). A future BE-consistent
/// implementation needs, per companion element:
///   * conductance stamps with the BE companion value `g_eq = T/L`
///     (coupled/xfmr: `T·L⁻¹` of the full inductance matrix, i.e. the
///     half_t factors doubled) stamped into A_be — NOT the trap `T/(2L)`;
///   * history current injection ×1 (`i_hist = i[n-1]`, not the
///     doubled-trapezoidal `2·i[n-1]`) stamped into the BE RHS each
///     fallback sample, plus a state-update convention that keeps the trap
///     path's doubled history intact for the next trap sample.
/// Augmented-inductor systems (branch rows, L in the C matrix) need none of
/// this — `A_be = G + (1/T)·C`, `A_neg_be = (1/T)·C` is already the exact
/// BE discretization of the branch equations.
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
) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>), CodegenError> {
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
    super::zero_augmented_history_rows(&mut a_neg_be, n, n_nodes, mna.n_aug);

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

    Ok((s_be, k_be, a_neg_be, rhs_const_be))
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
        let (_s_be, _k_be, a_neg_be, _rhs) =
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
