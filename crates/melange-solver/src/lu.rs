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

/// Jacobian stamp geometry for one behavioral (`B`) source, in matrix index
/// space. Node indices (`n_plus_idx`, `n_minus_idx`, `referenced_node_indices`)
/// use the MNA 1-based convention (0 = ground, matrix row/col = idx - 1);
/// `aug_row` is already a 0-based matrix row index.
///
/// Built by the codegen sparse-LU gate from `mna::BehavioralSourceInfo` so the
/// symbolic pattern covers every position `emit_behavioral_jacobian` writes.
#[derive(Debug, Clone)]
pub struct BehavioralStamp {
    /// `V={}` source (augmented constraint row) vs `I={}` (node-row current).
    pub is_voltage: bool,
    /// Augmented branch-current row for `V={}` sources (0-based matrix index).
    pub aug_row: Option<usize>,
    /// 1-based n+ terminal node index (0 = ground).
    pub n_plus_idx: usize,
    /// 1-based n- terminal node index (0 = ground).
    pub n_minus_idx: usize,
    /// 1-based indices of every node the expression references (0 = ground).
    pub referenced_node_indices: Vec<usize>,
}

/// Compute sparsity pattern of G_aug = A - N_i * J_dev * N_v (plus behavioral
/// B-source Jacobian stamps).
///
/// The pattern is the union of A's nonzeros, the device Jacobian stamp
/// positions, and the behavioral B-source Jacobian stamp positions.
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
    behavioral: &[BehavioralStamp],
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

    // Behavioral B-source Jacobian stamp positions. This must mirror
    // `nodal_emitter::emit_behavioral_jacobian` exactly, which stamps:
    //
    //   V={} (constraint row r = aug_row):
    //     chord_lu[r][col] -= ∂f/∂V(k)     for every non-ground referenced
    //                                       node k (col = k_idx - 1)
    //     (the linear ±1 couplings (r, np-1), (r, nm-1), (np-1, r), (nm-1, r)
    //      are stamped into base G by the MNA build and therefore already in
    //      A's pattern — unioned here anyway as belt-and-braces)
    //
    //   I={} (node rows):
    //     chord_lu[np-1][col] += ∂f/∂V(k)
    //     chord_lu[nm-1][col] -= ∂f/∂V(k)  for every non-ground referenced k
    //
    // Positions absent from this pattern would never be eliminated by the
    // symbolic schedule — the stamped values would be silently ignored,
    // converging to a wrong fixed point with no diagnostic.
    for b in behavioral {
        let ref_cols: Vec<usize> = b
            .referenced_node_indices
            .iter()
            .filter(|&&idx| idx > 0)
            .map(|&idx| idx - 1)
            .collect();
        if b.is_voltage {
            let Some(r) = b.aug_row else { continue };
            for &c in &ref_cols {
                pattern[r].insert(c);
            }
            for &t in &[b.n_plus_idx, b.n_minus_idx] {
                if t > 0 {
                    pattern[r].insert(t - 1);
                    pattern[t - 1].insert(r);
                }
            }
        } else {
            for &t in &[b.n_plus_idx, b.n_minus_idx] {
                if t > 0 {
                    for &c in &ref_cols {
                        pattern[t - 1].insert(c);
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
/// nonzero, find a not-yet-eliminated row to swap with. Returns (r1, r2) pairs.
///
/// Swaps are applied to a WORKING copy of the pattern as they are chosen:
/// `symbolic_lu` and the emitted `sparse_lu_factor` both apply all swaps
/// up-front before elimination, so a later swap decision must see the rows as
/// already permuted by the earlier swaps. Deciding against the stale
/// (un-swapped) pattern could pick a donor row whose pivot-column nonzero was
/// already swapped away (overlapping swaps — e.g. multiple inductor branch
/// rows plus VS constraint rows, all with structurally-zero diagonals).
pub fn find_row_swaps(
    pattern: &[Vec<usize>],
    elim_order: &[usize],
    n: usize,
) -> Vec<(usize, usize)> {
    use std::collections::BTreeSet;

    let mut work: Vec<BTreeSet<usize>> = pattern
        .iter()
        .map(|row| row.iter().copied().collect())
        .collect();
    let mut eliminated = vec![false; n];
    let mut swaps = Vec::new();

    for &pivot_col in elim_order {
        if !work[pivot_col].contains(&pivot_col) {
            // Candidate donors: rows still active whose CURRENT content has a
            // structural nonzero at the pivot column. Prefer a donor whose own
            // diagonal survives the swap (the displaced pivot-row content
            // contains the donor's column) — a purely greedy first-hit choice
            // can strand the donor row with a structurally-zero diagonal that
            // no later swap can repair.
            let donor = |work: &[BTreeSet<usize>], require_diag: bool| -> Option<usize> {
                (0..n).find(|&r| {
                    !eliminated[r]
                        && r != pivot_col
                        && work[r].contains(&pivot_col)
                        && (!require_diag || work[pivot_col].contains(&r))
                })
            };
            if let Some(r) = donor(&work, true).or_else(|| donor(&work, false)) {
                swaps.push((pivot_col, r));
                work.swap(pivot_col, r);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device_types::{DeviceParams, DeviceType, DiodeParams};

    // ── compute_g_aug_pattern tests ──────────────────────────────────

    #[test]
    fn test_pattern_diagonal_a_no_devices() {
        // 3x3 diagonal A, no devices → pattern is diagonal only
        let n = 3;
        let m = 0;
        let a_flat = vec![
            1.0, 0.0, 0.0, // row 0
            0.0, 2.0, 0.0, // row 1
            0.0, 0.0, 3.0, // row 2
        ];
        let pattern = compute_g_aug_pattern(&a_flat, &[], &[], n, m, &[], &[]);
        assert_eq!(pattern.len(), 3);
        assert_eq!(pattern[0], vec![0]);
        assert_eq!(pattern[1], vec![1]);
        assert_eq!(pattern[2], vec![2]);
    }

    #[test]
    fn test_pattern_with_device_stamps() {
        // 3x3 A with one 1D device. N_i stamps at nodes 0,1; N_v reads from nodes 0,1.
        // This should add fill-in at (0,0), (0,1), (1,0), (1,1).
        let n = 3;
        let m = 1;
        // Tridiagonal A
        let a_flat = vec![
            1.0, 0.5, 0.0, // row 0
            0.5, 1.0, 0.5, // row 1
            0.0, 0.5, 1.0, // row 2
        ];
        // N_i: 3x1, device injects at node 0 (-1) and node 1 (+1)
        let n_i_flat = vec![-1.0, 1.0, 0.0]; // [n0, n1, n2] for device 0
                                             // N_v: 1x3, device reads v[0] - v[1]
        let n_v_flat = vec![1.0, -1.0, 0.0]; // device 0 reads from nodes 0,1

        let slots = vec![DeviceSlot {
            device_type: DeviceType::Diode,
            start_idx: 0,
            dimension: 1,
            params: DeviceParams::Diode(DiodeParams {
                is: 1e-14,
                n_vt: 0.026,
                cjo: 0.0,
                rs: 0.0,
                bv: f64::INFINITY,
                ibv: 1e-10,
                rth: f64::INFINITY,
                cth: 1e-3,
                xti: 3.0,
                eg: 1.11,
                tamb: 300.15,
            }),
            has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
            stateful: None,
        }];

        let pattern = compute_g_aug_pattern(&a_flat, &n_i_flat, &n_v_flat, n, m, &slots, &[]);

        // Node 0 and 1 should connect to each other (from A tridiagonal + device stamps)
        assert!(pattern[0].contains(&0));
        assert!(pattern[0].contains(&1));
        assert!(pattern[1].contains(&0));
        assert!(pattern[1].contains(&1));
        // Node 2 should only have self (from A) and connection to node 1 (from A)
        assert!(pattern[2].contains(&1));
        assert!(pattern[2].contains(&2));
        // Node 2 should NOT connect to node 0 (no device path)
        assert!(!pattern[2].contains(&0));
    }

    #[test]
    fn test_pattern_behavioral_stamps() {
        // 5x5: nodes 0..3 in a chain, row 4 = behavioral V={} constraint row.
        // The V source is B out 0 V={f(V(n1), V(n2))} with out = node 3 (1-based
        // node index 4), referencing 1-based nodes 2 and 3 (matrix cols 1, 2).
        let n = 5;
        let a_flat = vec![
            1.0, 0.5, 0.0, 0.0, 0.0, // row 0
            0.5, 1.0, 0.5, 0.0, 0.0, // row 1
            0.0, 0.5, 1.0, 0.5, 0.0, // row 2
            0.0, 0.0, 0.5, 1.0, 1.0, // row 3 (branch-current coupling in G)
            0.0, 0.0, 0.0, 1.0, 0.0, // row 4 (constraint row, zero diagonal)
        ];
        let bsrc_v = BehavioralStamp {
            is_voltage: true,
            aug_row: Some(4),
            n_plus_idx: 4, // matrix row 3
            n_minus_idx: 0,
            referenced_node_indices: vec![2, 3, 0], // cols 1, 2 (+ ground skipped)
        };
        let pattern = compute_g_aug_pattern(&a_flat, &[], &[], n, 0, &[], &[bsrc_v]);
        // Constraint row gains the ∂f/∂V(k) columns:
        assert!(pattern[4].contains(&1), "aug row must contain ref col 1");
        assert!(pattern[4].contains(&2), "aug row must contain ref col 2");
        // Belt-and-braces linear couplings:
        assert!(pattern[4].contains(&3));
        assert!(pattern[3].contains(&4));
        // Ground reference contributes nothing:
        assert!(!pattern[4].contains(&0));

        // I={} form: stamps land on the terminal node rows instead.
        let bsrc_i = BehavioralStamp {
            is_voltage: false,
            aug_row: None,
            n_plus_idx: 1,                    // matrix row 0
            n_minus_idx: 4,                   // matrix row 3
            referenced_node_indices: vec![3], // col 2
        };
        let pattern_i = compute_g_aug_pattern(&a_flat, &[], &[], n, 0, &[], &[bsrc_i]);
        assert!(pattern_i[0].contains(&2), "n+ row must contain ref col");
        assert!(pattern_i[3].contains(&2), "n- row must contain ref col");
        // No aug-row stamps for I={}:
        assert!(!pattern_i[4].contains(&1));
    }

    // ── amd_ordering tests ──────────────────────────────────────────

    #[test]
    fn test_amd_ordering_all_nodes_present() {
        // Dense 4x4 pattern
        let pattern: Vec<Vec<usize>> = vec![
            vec![0, 1, 2, 3],
            vec![0, 1, 2, 3],
            vec![0, 1, 2, 3],
            vec![0, 1, 2, 3],
        ];
        let perm = amd_ordering(&pattern, 4);
        assert_eq!(perm.len(), 4);
        let mut sorted = perm.clone();
        sorted.sort();
        assert_eq!(
            sorted,
            vec![0, 1, 2, 3],
            "all nodes must appear exactly once"
        );
    }

    #[test]
    fn test_amd_ordering_star_graph() {
        // Star: node 0 connects to all, others only connect to 0
        let pattern = vec![
            vec![0, 1, 2, 3], // node 0: hub
            vec![0, 1],       // node 1: leaf
            vec![0, 2],       // node 2: leaf
            vec![0, 3],       // node 3: leaf
        ];
        let perm = amd_ordering(&pattern, 4);
        // After symmetrization, hub has degree 4, leaves have degree 2
        // Minimum degree heuristic should eliminate lower-degree nodes first
        // Hub (node 0) should NOT be first (it has highest degree)
        // After symmetrization all nodes connect to node 0, and after
        // eliminating the first leaf all remaining leaves connect to each other
        // through fill-in, so the degree ordering is non-trivial. The key
        // property: all nodes appear exactly once and the ordering is valid.
        let mut sorted = perm.clone();
        sorted.sort();
        assert_eq!(
            sorted,
            vec![0, 1, 2, 3],
            "all nodes must appear exactly once"
        );
    }

    #[test]
    fn test_amd_ordering_diagonal() {
        // Diagonal: each node connects only to itself
        let pattern = vec![vec![0], vec![1], vec![2]];
        let perm = amd_ordering(&pattern, 3);
        assert_eq!(perm.len(), 3);
        // All have degree 1, so order should be sequential (ties broken by index)
        assert_eq!(perm, vec![0, 1, 2]);
    }

    // ── find_row_swaps tests ────────────────────────────────────────

    #[test]
    fn test_find_row_swaps_none_needed() {
        let pattern = vec![vec![0, 1], vec![0, 1], vec![2]];
        let elim_order = vec![0, 1, 2];
        let swaps = find_row_swaps(&pattern, &elim_order, 3);
        assert!(
            swaps.is_empty(),
            "no swaps needed when all diagonals present"
        );
    }

    #[test]
    fn test_find_row_swaps_one_needed() {
        // Node 0 has no self-connection, but node 1 does and connects to col 0
        let pattern = vec![
            vec![1],    // row 0: no diagonal entry
            vec![0, 1], // row 1: has entries at col 0 and 1
        ];
        let elim_order = vec![0, 1];
        let swaps = find_row_swaps(&pattern, &elim_order, 2);
        assert_eq!(swaps.len(), 1);
        assert_eq!(swaps[0], (0, 1), "should swap row 0 with row 1");
    }

    #[test]
    fn test_find_row_swaps_overlapping_aug_rows() {
        // MNA-shaped 8x8 with FOUR augmented rows:
        //   rows 4, 5 — VS constraint rows (zero diagonals, need swaps):
        //     VS1: node0 → gnd     (row 4 = {0}; row 0 carries col 4)
        //     VS2: node0 → node1   (row 5 = {0,1}; rows 0,1 carry col 5)
        //   rows 6, 7 — inductor branch rows (alpha·L diagonal present, no swap)
        //
        // The swaps OVERLAP through the shared node-0 row: the swap for pivot 4
        // consumes row 0, and a stale-pattern donor scan for pivot 5 would then
        // pick row 0 again (its ORIGINAL content contains col 5) even though
        // that content has already been swapped away — leaving row 5 with a
        // structurally-zero diagonal. The working-pattern algorithm must pick
        // row 1 instead.
        let n = 8;
        let pattern: Vec<Vec<usize>> = vec![
            vec![0, 4, 5], // node 0
            vec![1, 5],    // node 1
            vec![2, 6],    // node 2 (inductor 1 terminal)
            vec![3, 7],    // node 3 (inductor 2 terminal)
            vec![0],       // VS1 constraint row — no diagonal
            vec![0, 1],    // VS2 constraint row — no diagonal
            vec![2, 6],    // inductor branch 1 — diagonal present
            vec![3, 7],    // inductor branch 2 — diagonal present
        ];
        // Force the aug rows to be eliminated first (worst case for swaps).
        let elim_order = vec![4, 5, 6, 7, 0, 1, 2, 3];
        let swaps = find_row_swaps(&pattern, &elim_order, n);

        // Verify structurally: after applying the swap schedule, every pivot
        // has a structural nonzero on its diagonal at elimination time
        // (accounting for fill-in, mirroring symbolic_lu).
        {
            use std::collections::BTreeSet;
            let mut pat: Vec<BTreeSet<usize>> = pattern
                .iter()
                .map(|row| row.iter().copied().collect())
                .collect();
            for &(r1, r2) in &swaps {
                pat.swap(r1, r2);
            }
            let mut eliminated = vec![false; n];
            for &pivot in &elim_order {
                assert!(
                    pat[pivot].contains(&pivot),
                    "pivot {pivot} has a structurally-zero diagonal after swaps {swaps:?}"
                );
                eliminated[pivot] = true;
                let rows_to_elim: Vec<usize> = (0..n)
                    .filter(|&r| !eliminated[r] && pat[r].contains(&pivot))
                    .collect();
                let fill_cols: Vec<usize> = pat[pivot]
                    .iter()
                    .copied()
                    .filter(|&c| !eliminated[c] && c != pivot)
                    .collect();
                for &row in &rows_to_elim {
                    for &j in &fill_cols {
                        pat[row].insert(j);
                    }
                }
            }
        }

        // Verify numerically: execute the full symbolic schedule and compare
        // the solve against dense LU.
        let sparsity = symbolic_lu(&pattern, &elim_order, &swaps, n);
        #[rustfmt::skip]
        let a_orig: Vec<Vec<f64>> = vec![
            vec![2.0, 0.0, 0.0, 0.0, 1.0,  1.0, 0.0,  0.0],
            vec![0.0, 2.0, 0.0, 0.0, 0.0, -1.0, 0.0,  0.0],
            vec![0.0, 0.0, 2.0, 0.0, 0.0,  0.0, 1.0,  0.0],
            vec![0.0, 0.0, 0.0, 2.0, 0.0,  0.0, 0.0,  1.0],
            vec![1.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0,  0.0],
            vec![1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0],
            vec![0.0, 0.0, 1.0, 0.0, 0.0,  0.0, -0.5, 0.0],
            vec![0.0, 0.0, 0.0, 1.0, 0.0,  0.0, 0.0, -0.5],
        ];
        let b_orig: Vec<f64> = (1..=8).map(|i| i as f64).collect();

        let mut a: Vec<Vec<f64>> = a_orig.clone();
        for &(r1, r2) in &sparsity.row_swaps {
            a.swap(r1, r2);
        }
        for op in &sparsity.ops {
            match op {
                LuOp::DivPivot { row, col } => {
                    assert!(
                        a[*col][*col].abs() > 1e-12,
                        "numerically-zero pivot at col {col} — swap schedule is broken"
                    );
                    a[*row][*col] /= a[*col][*col];
                }
                LuOp::SubMul { row, col, j } => {
                    a[*row][*j] -= a[*row][*col] * a[*col][*j];
                }
            }
        }
        let mut b = b_orig.clone();
        for &(r1, r2) in &sparsity.row_swaps {
            b.swap(r1, r2);
        }
        for &(row, col) in &sparsity.l_nnz {
            b[row] -= a[row][col] * b[col];
        }
        // Backward substitution grouped by pivot in reverse elimination order,
        // mirroring the emitted `sparse_lu_back_solve` (subtract off-diagonal
        // U entries, then divide by the diagonal).
        {
            let mut u_by_pivot: std::collections::HashMap<usize, Vec<usize>> =
                std::collections::HashMap::new();
            for &(row, col) in &sparsity.u_nnz {
                if col != row {
                    u_by_pivot.entry(row).or_default().push(col);
                }
            }
            for &pivot in sparsity.elim_order.iter().rev() {
                if let Some(cols) = u_by_pivot.get(&pivot) {
                    for &c in cols {
                        b[pivot] -= a[pivot][c] * b[c];
                    }
                }
                b[pivot] /= a[pivot][pivot];
            }
        }

        let x_dense = crate::dc_op::solve_linear(&a_orig, &b_orig).unwrap();
        for i in 0..n {
            assert!(
                (b[i] - x_dense[i]).abs() < 1e-10,
                "sparse x[{i}]={} vs dense x[{i}]={}",
                b[i],
                x_dense[i]
            );
        }
    }

    // ── symbolic_lu tests ───────────────────────────────────────────

    #[test]
    fn test_symbolic_lu_roundtrip_3x3() {
        // 3x3 dense matrix — symbolic schedule should produce same result as dense LU
        let pattern = vec![vec![0, 1, 2], vec![0, 1, 2], vec![0, 1, 2]];
        let elim_order = amd_ordering(&pattern, 3);
        let swaps = find_row_swaps(&pattern, &elim_order, 3);
        let sparsity = symbolic_lu(&pattern, &elim_order, &swaps, 3);

        // Execute the symbolic schedule on actual values
        let mut a = [
            vec![2.0, 1.0, 1.0],
            vec![4.0, 3.0, 3.0],
            vec![8.0, 7.0, 9.0],
        ];
        // Apply row swaps
        for &(r1, r2) in &sparsity.row_swaps {
            a.swap(r1, r2);
        }
        // Execute ops
        for op in &sparsity.ops {
            match op {
                LuOp::DivPivot { row, col } => {
                    a[*row][*col] /= a[*col][*col];
                }
                LuOp::SubMul { row, col, j } => {
                    a[*row][*j] -= a[*row][*col] * a[*col][*j];
                }
            }
        }

        // Now use the factored matrix to solve Ax=b
        let b_orig = vec![1.0, 2.0, 3.0];
        let mut b = b_orig.clone();
        // Apply same row swaps to b
        for &(r1, r2) in &sparsity.row_swaps {
            b.swap(r1, r2);
        }
        // Forward substitution (L)
        for &(row, col) in &sparsity.l_nnz {
            b[row] -= a[row][col] * b[col];
        }
        // Backward substitution (U)
        for &(row, col) in sparsity.u_nnz.iter().rev() {
            if row == col {
                b[row] /= a[row][col];
            } else {
                b[row] -= a[row][col] * b[col];
            }
        }

        // Compare against dense LU
        let a_orig = vec![
            vec![2.0, 1.0, 1.0],
            vec![4.0, 3.0, 3.0],
            vec![8.0, 7.0, 9.0],
        ];
        let x_dense = crate::dc_op::solve_linear(&a_orig, &b_orig).unwrap();

        for i in 0..3 {
            assert!(
                (b[i] - x_dense[i]).abs() < 1e-10,
                "sparse LU x[{i}]={} vs dense x[{i}]={}",
                b[i],
                x_dense[i]
            );
        }
    }

    #[test]
    fn test_symbolic_lu_flop_counts() {
        // Dense 3x3: factor_flops should be 2*(n-1) + 2*(n-2) + ... for dense
        // For n=3 dense: step 0 eliminates 2 rows with 2 cols each = 2*(1+2) = 6
        // step 1 eliminates 1 row with 1 col = 1+2 = 3
        // Total: ~8 (varies with AMD ordering)
        let pattern = vec![vec![0, 1, 2], vec![0, 1, 2], vec![0, 1, 2]];
        let elim_order = amd_ordering(&pattern, 3);
        let swaps = find_row_swaps(&pattern, &elim_order, 3);
        let sparsity = symbolic_lu(&pattern, &elim_order, &swaps, 3);

        assert!(
            sparsity.factor_flops > 0,
            "dense 3x3 should have nonzero factor flops"
        );
        assert!(
            sparsity.solve_flops > 0,
            "dense 3x3 should have nonzero solve flops"
        );
        assert_eq!(sparsity.n, 3);
    }

    #[test]
    fn test_symbolic_lu_sparse_fewer_flops() {
        // Tridiagonal 4x4 should have fewer flops than dense 4x4
        let dense_pattern = vec![
            vec![0, 1, 2, 3],
            vec![0, 1, 2, 3],
            vec![0, 1, 2, 3],
            vec![0, 1, 2, 3],
        ];
        let sparse_pattern = vec![vec![0, 1], vec![0, 1, 2], vec![1, 2, 3], vec![2, 3]];

        let eo_d = amd_ordering(&dense_pattern, 4);
        let sw_d = find_row_swaps(&dense_pattern, &eo_d, 4);
        let sp_d = symbolic_lu(&dense_pattern, &eo_d, &sw_d, 4);

        let eo_s = amd_ordering(&sparse_pattern, 4);
        let sw_s = find_row_swaps(&sparse_pattern, &eo_s, 4);
        let sp_s = symbolic_lu(&sparse_pattern, &eo_s, &sw_s, 4);

        assert!(
            sp_s.factor_flops < sp_d.factor_flops,
            "sparse ({}) should have fewer factor flops than dense ({})",
            sp_s.factor_flops,
            sp_d.factor_flops
        );
    }
}
