//! Compile-time sparse LU planning for codegen.
//!
//! Computes the AMD-ordered, symbolically-factorized elimination schedule
//! used by `CircuitIR` and emitted into generated code as straight-line
//! `sparse_lu_factor` / `sparse_lu_back_solve` functions. The runtime
//! dense LU implementation that used to share this file was removed with
//! the runtime solvers — only the sparsity-analysis pipeline remains here.

use crate::device_types::DeviceSlot;
use serde::{Deserialize, Serialize};

/// Threshold below which matrix entries are treated as structural zeros.
pub const SPARSITY_THRESHOLD: f64 = 1e-20;

/// Compile-time sparse LU elimination schedule.
///
/// Computed from the G_aug = A - N_i*J_dev*N_v sparsity pattern. Consumed
/// by codegen (`emit_sparse_lu_factor`) to emit straight-line code that
/// skips structural zeros.
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
