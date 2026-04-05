//! Pre-allocated LU factorization and solve routines for real-time audio.
//!
//! Provides zero-allocation LU factor/solve for the runtime NodalSolver,
//! plus sparsity analysis (AMD ordering, symbolic LU) shared with codegen.

/// Threshold below which pivot/diagonal entries are treated as singular.
const SINGULARITY_THRESHOLD: f64 = 1e-15;

/// Threshold below which matrix entries are treated as structural zeros.
pub const SPARSITY_THRESHOLD: f64 = 1e-20;

/// Pre-allocated workspace for dense LU factorization and solve.
///
/// All buffers are allocated once at solver construction; no heap allocation
/// occurs during `lu_factor_inplace` or `lu_back_solve_inplace`.
#[derive(Clone)]
pub struct LuWorkspace {
    /// Matrix dimension.
    pub n: usize,
    /// LU factors stored in-place (equilibrated), n*n flat row-major.
    pub lu: Vec<f64>,
    /// Equilibration scaling vector (length n).
    pub d: Vec<f64>,
    /// Row permutation from partial pivoting (length n).
    pub perm: Vec<usize>,
    /// Temporary vector for forward/backward substitution (length n).
    pub x: Vec<f64>,
}

impl LuWorkspace {
    /// Create a new workspace for an n x n system.
    pub fn new(n: usize) -> Self {
        Self {
            n,
            lu: vec![0.0; n * n],
            d: vec![1.0; n],
            perm: (0..n).collect(),
            x: vec![0.0; n],
        }
    }
}

/// Equilibrate and LU-factorize an n x n matrix in-place.
///
/// The input matrix `a_flat` (n*n row-major) is copied into `ws.lu` with
/// diagonal equilibration applied. The factored result, equilibration vector
/// `ws.d`, and row permutation `ws.perm` are stored in the workspace.
///
/// Returns `true` on success, `false` if the matrix is singular.
/// No heap allocation.
pub fn lu_factor_inplace(a_flat: &[f64], ws: &mut LuWorkspace, n: usize) -> bool {
    // Equilibrate: d[i] = 1/sqrt(|a[i][i]|)
    for i in 0..n {
        let diag = a_flat[i * n + i].abs();
        ws.d[i] = if diag > SINGULARITY_THRESHOLD {
            1.0 / diag.sqrt()
        } else {
            1.0
        };
    }

    // Copy equilibrated matrix into workspace
    for i in 0..n {
        for j in 0..n {
            ws.lu[i * n + j] = ws.d[i] * a_flat[i * n + j] * ws.d[j];
        }
    }

    // Initialize permutation
    for i in 0..n {
        ws.perm[i] = i;
    }

    // LU factorize with partial pivoting
    for col in 0..n {
        // Find pivot row
        let mut max_row = col;
        let mut max_val = ws.lu[col * n + col].abs();
        for row in (col + 1)..n {
            let v = ws.lu[row * n + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < SINGULARITY_THRESHOLD {
            return false; // Singular
        }

        // Swap rows if needed
        if max_row != col {
            for j in 0..n {
                ws.lu.swap(col * n + j, max_row * n + j);
            }
            ws.perm.swap(col, max_row);
        }

        // Eliminate below pivot
        let pivot = ws.lu[col * n + col];
        for row in (col + 1)..n {
            let factor = ws.lu[row * n + col] / pivot;
            ws.lu[row * n + col] = factor; // Store L factor
            for j in (col + 1)..n {
                ws.lu[row * n + j] -= factor * ws.lu[col * n + j];
            }
        }
    }

    true
}

/// Solve Ax = b using pre-factored LU from `lu_factor_inplace`.
///
/// The solution is written back into `rhs` in-place. Uses `ws.x` as scratch.
/// The workspace fields `ws.lu`, `ws.d`, `ws.perm` must contain valid factors
/// from a prior call to `lu_factor_inplace`.
///
/// No heap allocation.
pub fn lu_solve(ws: &mut LuWorkspace, rhs: &mut [f64], n: usize) {
    // Build permuted, scaled RHS
    for i in 0..n {
        let pi = ws.perm[i];
        ws.x[i] = ws.d[pi] * rhs[pi];
    }

    // Forward substitution (L * y = rhs)
    for i in 1..n {
        let mut sum = 0.0;
        for j in 0..i {
            sum += ws.lu[i * n + j] * ws.x[j];
        }
        ws.x[i] -= sum;
    }

    // Backward substitution (U * x_eq = y)
    for i in (0..n).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..n {
            sum += ws.lu[i * n + j] * ws.x[j];
        }
        ws.x[i] = (ws.x[i] - sum) / ws.lu[i * n + i];
    }

    // Undo equilibration
    for i in 0..n {
        rhs[i] = ws.d[i] * ws.x[i];
    }
}

// ============================================================================
// Sparse LU: sparsity analysis, AMD ordering, symbolic factorization
// ============================================================================

use crate::device_types::DeviceSlot;
use serde::{Deserialize, Serialize};

/// Compile-time or construction-time sparse LU elimination schedule.
///
/// Computed from the G_aug = A - N_i*J_dev*N_v sparsity pattern. Used by codegen
/// to emit straight-line code, and by runtime NodalSolver to skip structural zeros.
///
/// **All indices are in the original node ordering.** The AMD fill-reducing
/// ordering determines the ORDER of operations, not the physical layout.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct LuSparsity {
    /// Straight-line elimination operations (original indices, AMD emission order).
    pub ops: Vec<LuOp>,
    /// AMD elimination order: elim_order[step] = original column eliminated at that step.
    pub elim_order: Vec<usize>,
    /// Row swaps applied before elimination (for zero diagonals).
    pub row_swaps: Vec<(usize, usize)>,
    /// L entries (row, col) in original indices, ordered for forward substitution.
    pub l_nnz: Vec<(usize, usize)>,
    /// U entries (row, col) in original indices, ordered for backward substitution.
    pub u_nnz: Vec<(usize, usize)>,
    /// Total FLOPs for factorization.
    pub factor_flops: usize,
    /// Total FLOPs for forward+backward solve.
    pub solve_flops: usize,
    /// Matrix dimension.
    pub n: usize,
}

/// A single operation in the sparse LU elimination sequence.
/// All row/col indices are in the **original** node ordering.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LuOp {
    /// `a[row][col] /= a[col][col]` — compute L factor.
    DivPivot { row: usize, col: usize },
    /// `a[row][j] -= a[row][col] * a[col][j]` — elimination.
    SubMul { row: usize, col: usize, j: usize },
}

/// Compute sparsity pattern of G_aug = A - N_i * J_dev * N_v.
///
/// The pattern is the union of A's nonzeros and the Jacobian stamp positions.
/// This is topology-fixed: pot/switch changes only modify values, not positions.
///
/// Returns per-row sorted lists of nonzero column indices.
pub fn compute_g_aug_pattern(
    a_flat: &[f64],
    n_i_flat: &[f64],
    n_v_flat: &[f64],
    n: usize,
    m: usize,
    device_slots: &[DeviceSlot],
) -> Vec<Vec<usize>> {
    use std::collections::BTreeSet;

    // Start with A's nonzero pattern
    let mut pattern: Vec<BTreeSet<usize>> = (0..n).map(|_| BTreeSet::new()).collect();

    for i in 0..n {
        for j in 0..n {
            if a_flat[i * n + j].abs() >= SPARSITY_THRESHOLD {
                pattern[i].insert(j);
            }
        }
    }

    // Add Jacobian stamp positions: N_i[:,dev_i] * N_v[dev_j,:]
    let mut ni_nz_by_dev = vec![Vec::new(); m];
    for a in 0..n {
        for i in 0..m {
            if n_i_flat[a * m + i].abs() >= SPARSITY_THRESHOLD {
                ni_nz_by_dev[i].push(a);
            }
        }
    }

    for slot in device_slots {
        let s = slot.start_idx;
        let dim = slot.dimension;
        for di in 0..dim {
            let dev_i = s + di;
            let ni_nodes = &ni_nz_by_dev[dev_i];
            for dj in 0..dim {
                let dev_j = s + dj;
                for b in 0..n {
                    if n_v_flat[dev_j * n + b].abs() >= SPARSITY_THRESHOLD {
                        for &a in ni_nodes {
                            pattern[a].insert(b);
                        }
                    }
                }
            }
        }
    }

    pattern
        .into_iter()
        .map(|s| s.into_iter().collect())
        .collect()
}

/// Approximate Minimum Degree (AMD) ordering for fill-reducing LU.
///
/// Greedy heuristic: at each step, eliminate the node with the fewest
/// connections in the current elimination graph.
pub fn amd_ordering(pattern: &[Vec<usize>], n: usize) -> Vec<usize> {
    use std::collections::BTreeSet;

    // Build symmetric adjacency (LU fill depends on structural symmetry)
    let mut adj: Vec<BTreeSet<usize>> = (0..n)
        .map(|i| pattern[i].iter().copied().collect())
        .collect();
    // Symmetrize: if (i,j) exists, add (j,i)
    for i in 0..n {
        let cols: Vec<usize> = adj[i].iter().copied().collect();
        for j in cols {
            adj[j].insert(i);
        }
    }

    let mut perm = Vec::with_capacity(n);
    let mut eliminated = vec![false; n];

    for _ in 0..n {
        // Find un-eliminated node with minimum degree (ties broken by index)
        let mut best = usize::MAX;
        let mut best_deg = usize::MAX;
        for i in 0..n {
            if eliminated[i] {
                continue;
            }
            let deg = adj[i].iter().filter(|&&j| !eliminated[j]).count();
            if deg < best_deg {
                best_deg = deg;
                best = i;
            }
        }
        perm.push(best);
        eliminated[best] = true;

        // Update elimination graph: connect all neighbors of `best` to each other
        let neighbors: Vec<usize> = adj[best]
            .iter()
            .copied()
            .filter(|&j| !eliminated[j])
            .collect();
        for &u in &neighbors {
            for &v in &neighbors {
                if u != v {
                    adj[u].insert(v);
                }
            }
        }
    }

    perm
}

/// Find row swaps needed for zero diagonals in AMD elimination order.
///
/// For each step k in the AMD order, if the pivot position has no structural
/// nonzero, find a later row to swap with. Returns (r1, r2) pairs.
pub fn find_row_swaps(
    pattern: &[Vec<usize>],
    elim_order: &[usize],
    n: usize,
) -> Vec<(usize, usize)> {
    let mut eliminated = vec![false; n];
    let mut swaps = Vec::new();

    for &pivot_col in elim_order {
        if !pattern[pivot_col].contains(&pivot_col) {
            for r in 0..n {
                if !eliminated[r] && r != pivot_col && pattern[r].contains(&pivot_col) {
                    swaps.push((pivot_col, r));
                    break;
                }
            }
        }
        eliminated[pivot_col] = true;
    }

    if !swaps.is_empty() {
        log::info!(
            "Sparse LU: {} row swaps needed for zero diagonals",
            swaps.len()
        );
    }

    swaps
}

/// Symbolic LU factorization in original index space.
///
/// Traces Gaussian elimination symbolically with AMD ordering to find:
/// - Fill-in pattern and elimination operations (in original indices)
/// - L and U nonzero patterns for forward/backward substitution
///
/// All indices in the output are **original node indices**.
pub fn symbolic_lu(
    pattern: &[Vec<usize>],
    elim_order: &[usize],
    row_swaps: &[(usize, usize)],
    n: usize,
) -> LuSparsity {
    use std::collections::BTreeSet;

    // Build working pattern (mutable, tracks fill-in) — apply row swaps
    let mut pat: Vec<BTreeSet<usize>> = pattern
        .iter()
        .map(|row| row.iter().copied().collect())
        .collect();
    for &(r1, r2) in row_swaps {
        let tmp = pat[r1].clone();
        pat[r1] = pat[r2].clone();
        pat[r2] = tmp;
    }

    let mut eliminated = vec![false; n];
    let mut ops = Vec::new();
    let mut l_nnz = Vec::new();
    let mut u_nnz = Vec::new();
    let mut factor_flops = 0usize;

    for &pivot in elim_order {
        eliminated[pivot] = true;

        // U entries: row=pivot, columns that are NOT yet eliminated (including pivot itself)
        let u_cols: Vec<usize> = pat[pivot]
            .iter()
            .copied()
            .filter(|&c| !eliminated[c] || c == pivot)
            .collect();
        for &c in &u_cols {
            u_nnz.push((pivot, c));
        }

        // Find rows that have nonzero at column `pivot` and are not yet eliminated
        let rows_to_elim: Vec<usize> = (0..n)
            .filter(|&r| !eliminated[r] && pat[r].contains(&pivot))
            .collect();

        for &row in &rows_to_elim {
            ops.push(LuOp::DivPivot { row, col: pivot });
            l_nnz.push((row, pivot));
            factor_flops += 1;

            let pivot_cols: Vec<usize> = pat[pivot]
                .iter()
                .copied()
                .filter(|&c| !eliminated[c] && c != pivot)
                .collect();
            for &j in &pivot_cols {
                ops.push(LuOp::SubMul { row, col: pivot, j });
                factor_flops += 2; // multiply + subtract
                pat[row].insert(j); // fill-in
            }
        }
    }

    let solve_flops = l_nnz.len() * 2 + u_nnz.len() * 2;

    log::info!(
        "Sparse LU: N={}, factor_flops={} (dense ~{}), L_nnz={}, U_nnz={}, solve_flops={} (dense ~{})",
        n, factor_flops, n * n * n / 3, l_nnz.len(), u_nnz.len(), solve_flops, n * n * 2,
    );

    LuSparsity {
        ops,
        elim_order: elim_order.to_vec(),
        row_swaps: row_swaps.to_vec(),
        l_nnz,
        u_nnz,
        factor_flops,
        solve_flops,
        n,
    }
}

/// Sparse LU factorization using a precomputed elimination schedule.
///
/// Factorizes `a_flat` (n*n row-major) in-place following the operations in `plan`.
/// Writes equilibration scaling into `d`. Returns `false` if a pivot is too small
/// (caller should fall back to dense LU).
///
/// No heap allocation.
pub fn sparse_lu_factor(a_flat: &mut [f64], d: &mut [f64], plan: &LuSparsity) -> bool {
    let n = plan.n;

    // Equilibrate: d[i] = 1/sqrt(|a[i][i]|)
    for i in 0..n {
        let diag = a_flat[i * n + i].abs();
        d[i] = if diag > SINGULARITY_THRESHOLD {
            1.0 / diag.sqrt()
        } else {
            1.0
        };
    }
    for i in 0..n {
        for j in 0..n {
            a_flat[i * n + j] *= d[i] * d[j];
        }
    }

    // Static row swaps
    for &(r1, r2) in &plan.row_swaps {
        for j in 0..n {
            a_flat.swap(r1 * n + j, r2 * n + j);
        }
    }

    // Execute sparse elimination ops
    for op in &plan.ops {
        match op {
            LuOp::DivPivot { row, col } => {
                let pivot = a_flat[*col * n + *col];
                if pivot.abs() < 1e-30 {
                    return false; // Fall back to dense
                }
                a_flat[*row * n + *col] /= pivot;
            }
            LuOp::SubMul { row, col, j } => {
                a_flat[*row * n + *j] -= a_flat[*row * n + *col] * a_flat[*col * n + *j];
            }
        }
    }

    true
}

/// Sparse LU back-solve using a precomputed elimination schedule.
///
/// Solves Ax = b given `a_lu` (factored in-place by `sparse_lu_factor`),
/// equilibration `d`, and the `plan`. Solution is written into `rhs`.
/// `x_tmp` is scratch space (length n).
///
/// No heap allocation.
pub fn sparse_lu_solve(
    a_lu: &[f64],
    d: &[f64],
    plan: &LuSparsity,
    rhs: &mut [f64],
    x_tmp: &mut [f64],
) {
    let n = plan.n;

    // Equilibrate RHS
    for i in 0..n {
        x_tmp[i] = d[i] * rhs[i];
    }

    // Apply row swaps
    for &(r1, r2) in &plan.row_swaps {
        x_tmp.swap(r1, r2);
    }

    // Sparse forward substitution (L, in AMD order)
    for &(row, col) in &plan.l_nnz {
        x_tmp[row] -= a_lu[row * n + col] * x_tmp[col];
    }

    // Sparse backward substitution (U, reverse AMD order)
    for &pivot in plan.elim_order.iter().rev() {
        // Subtract off-diagonal U entries for this pivot row
        for &(r, c) in &plan.u_nnz {
            if r == pivot && c != pivot {
                x_tmp[pivot] -= a_lu[pivot * n + c] * x_tmp[c];
            }
        }
        x_tmp[pivot] /= a_lu[pivot * n + pivot];
    }

    // De-equilibrate
    for i in 0..n {
        rhs[i] = d[i] * x_tmp[i];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lu_factor_solve_identity() {
        let n = 3;
        // Identity matrix
        let mut a = vec![0.0; n * n];
        a[0] = 1.0;
        a[4] = 1.0;
        a[8] = 1.0;
        let mut ws = LuWorkspace::new(n);
        assert!(lu_factor_inplace(&a, &mut ws, n));

        let mut rhs = vec![1.0, 2.0, 3.0];
        lu_solve(&mut ws, &mut rhs, n);
        assert!((rhs[0] - 1.0).abs() < 1e-12);
        assert!((rhs[1] - 2.0).abs() < 1e-12);
        assert!((rhs[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_lu_factor_solve_3x3() {
        let n = 3;
        // A = [[2, 1, -1], [-3, -1, 2], [-2, 1, 2]]
        // Solution for b = [8, -11, -3] is x = [2, 3, -1]
        let a = vec![2.0, 1.0, -1.0, -3.0, -1.0, 2.0, -2.0, 1.0, 2.0];
        let mut ws = LuWorkspace::new(n);
        assert!(lu_factor_inplace(&a, &mut ws, n));

        let mut rhs = vec![8.0, -11.0, -3.0];
        lu_solve(&mut ws, &mut rhs, n);
        assert!((rhs[0] - 2.0).abs() < 1e-10);
        assert!((rhs[1] - 3.0).abs() < 1e-10);
        assert!((rhs[2] - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_lu_singular_matrix() {
        let n = 2;
        let a = vec![1.0, 2.0, 2.0, 4.0]; // Singular: row2 = 2*row1
        let mut ws = LuWorkspace::new(n);
        assert!(!lu_factor_inplace(&a, &mut ws, n));
    }

    #[test]
    fn test_lu_matches_solve_equilibrated() {
        // Compare against the existing solve_equilibrated for a circuit-like matrix.
        // This is an A = G + 2C/T matrix (non-singular due to C contribution).
        let n = 4;
        let a_flat = vec![
            1.1e-3, -1e-3, 0.0, 0.0, -1e-3, 2.2e-3, -1e-3, 0.0, 0.0, -1e-3, 1.6e-3, -0.5e-3,
            0.0, 0.0, -0.5e-3, 0.6e-3,
        ];
        let b = vec![1e-3, 0.0, 0.0, 0.0];

        // Solve with new LU
        let mut ws = LuWorkspace::new(n);
        assert!(lu_factor_inplace(&a_flat, &mut ws, n));
        let mut rhs_new = b.clone();
        lu_solve(&mut ws, &mut rhs_new, n);

        // Solve with existing solve_equilibrated
        let a_2d: Vec<Vec<f64>> = (0..n).map(|i| a_flat[i * n..(i + 1) * n].to_vec()).collect();
        let rhs_old = crate::dk::solve_equilibrated(&a_2d, &b).unwrap();

        // Should match within reasonable tolerance (no iterative refinement in new version)
        for i in 0..n {
            let diff = (rhs_new[i] - rhs_old[i]).abs();
            let scale = rhs_old[i].abs().max(1e-10);
            assert!(
                diff / scale < 1e-6,
                "Mismatch at [{}]: new={}, old={}, diff={}",
                i,
                rhs_new[i],
                rhs_old[i],
                diff
            );
        }
    }
}
