//! Circuit intermediate representation (IR) for language-agnostic code generation.
//!
//! `CircuitIR` captures everything a code emitter needs to produce a working solver,
//! without referencing any Rust-specific types from the MNA/DK pipeline.

use serde::{Deserialize, Serialize};

use crate::dc_op::{self, DcOpConfig};
use crate::dk::{self, DkKernel};
use crate::mna::MnaSystem;
use crate::parser::{Element, Netlist};

use super::{CodegenConfig, CodegenError};

/// Solver method for code generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum SolverMode {
    /// DK method: precompute S=A⁻¹, NR in M-dimensional current space.
    /// Fast (O(N²+M³) per sample) but requires well-conditioned A and K[i][i]<0.
    #[default]
    Dk,
    /// Full-nodal NR: LU solve in N-dimensional voltage space per NR iteration.
    /// Handles any circuit topology including transformer-coupled NFB.
    /// Slower (O(N³) per sample) but universally convergent.
    Nodal,
}

/// Language-agnostic intermediate representation of a compiled circuit.
///
/// Built from `DkKernel` + `MnaSystem` + `Netlist` + `CodegenConfig`, this
/// struct contains every piece of data an emitter needs — matrices, device
/// parameters, solver config — without referencing the builder types.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub struct CircuitIR {
    pub metadata: CircuitMetadata,
    pub topology: Topology,
    /// DK or Nodal solver mode
    #[serde(default)]
    pub solver_mode: SolverMode,
    pub solver_config: SolverConfig,
    pub matrices: Matrices,
    pub dc_operating_point: Vec<f64>,
    pub device_slots: Vec<DeviceSlot>,
    pub has_dc_sources: bool,
    pub has_dc_op: bool,
    /// M-vector: nonlinear device currents at DC operating point
    #[serde(default)]
    pub dc_nl_currents: Vec<f64>,
    /// Whether the nonlinear DC OP solver converged
    #[serde(default)]
    pub dc_op_converged: bool,
    /// Whether to include DC blocking filter on outputs.
    pub dc_block: bool,
    pub inductors: Vec<InductorIR>,
    pub coupled_inductors: Vec<CoupledInductorIR>,
    pub transformer_groups: Vec<TransformerGroupIR>,
    pub pots: Vec<PotentiometerIR>,
    pub switches: Vec<SwitchIR>,
    /// Op-amp output voltage saturation clamps.
    /// Only populated for op-amps with finite VSAT.
    #[serde(default)]
    pub opamps: Vec<OpampIR>,
    /// Pre-analyzed sparsity patterns for compile-time matrices.
    #[serde(default)]
    pub sparsity: SparseInfo,
}

/// Circuit metadata (name, title, generator version).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CircuitMetadata {
    pub circuit_name: String,
    pub title: String,
    pub generator_version: String,
}

/// Circuit topology dimensions.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub struct Topology {
    /// System dimension = n_aug (n + num_vs + num_vcvs), or n_nodal when augmented inductors are used.
    /// This is the size of all N-indexed matrices and vectors in the solver.
    pub n: usize,
    /// Original circuit node count (excluding ground and augmented VS/VCVS variables).
    /// Output node indices must be < n_nodes.
    #[serde(default)]
    pub n_nodes: usize,
    /// Total nonlinear dimension (sum of device dimensions)
    pub m: usize,
    /// Number of physical nonlinear devices
    pub num_devices: usize,
    /// Boundary between VS/VCVS rows and inductor branch variables.
    /// Equal to mna.n_aug (= n_nodes + num_vs + num_vcvs).
    /// A_neg rows for VS/VCVS algebraic constraints (at n_nodes + vs.ext_idx etc.)
    /// should be zeroed. Inductor rows (n_aug..n) and internal BJT node rows should NOT.
    #[serde(default)]
    pub n_aug: usize,
    /// True when inductors use augmented MNA (branch current variables in G/C)
    /// instead of companion model (history currents in state).
    #[serde(default)]
    pub augmented_inductors: bool,
}

/// Solver configuration baked into the generated code.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub struct SolverConfig {
    pub sample_rate: f64,
    /// alpha = 2/T (trapezoidal) or 1/T (backward Euler) at internal sample rate
    pub alpha: f64,
    pub tolerance: f64,
    pub max_iterations: usize,
    pub input_node: usize,
    /// Output node indices (one per output channel)
    #[serde(default = "default_output_nodes")]
    pub output_nodes: Vec<usize>,
    pub input_resistance: f64,
    /// Oversampling factor (1, 2, or 4). Default 1 (no oversampling).
    #[serde(default = "default_oversampling_factor")]
    pub oversampling_factor: usize,
    /// Output scale factors applied after DC blocking (one per output)
    #[serde(default = "default_output_scales")]
    pub output_scales: Vec<f64>,
    /// Silent samples to process after pot-triggered matrix rebuild (default 64).
    #[serde(default = "default_pot_settle_samples")]
    pub pot_settle_samples: usize,
    /// Use backward Euler integration (unconditionally stable, first-order).
    #[serde(default)]
    pub backward_euler: bool,
}

fn default_pot_settle_samples() -> usize {
    64
}

fn default_output_nodes() -> Vec<usize> {
    vec![0]
}

fn default_oversampling_factor() -> usize {
    1
}

fn default_output_scales() -> Vec<f64> {
    vec![1.0]
}

/// All matrices needed by the generated solver (flattened row-major).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Matrices {
    /// S = A^{-1}, N×N row-major (default for codegen sample rate)
    pub s: Vec<f64>,
    /// A_neg = alpha*C - G, N×N row-major (default for codegen sample rate)
    pub a_neg: Vec<f64>,
    /// Nonlinear kernel K = N_v * S * N_i, M×M row-major (default for codegen sample rate)
    pub k: Vec<f64>,
    /// Voltage extraction N_v, M×N row-major
    pub n_v: Vec<f64>,
    /// Current injection N_i, N×M row-major (kernel storage order)
    pub n_i: Vec<f64>,
    /// Constant RHS contribution from DC sources, length N
    pub rhs_const: Vec<f64>,
    /// Raw conductance matrix G, N×N row-major (sample-rate independent)
    /// Includes input conductance but NOT inductor companion conductances
    #[serde(default)]
    pub g_matrix: Vec<f64>,
    /// Raw capacitance matrix C, N×N row-major (sample-rate independent)
    #[serde(default)]
    pub c_matrix: Vec<f64>,

    // --- Nodal solver matrices (only populated when solver_mode == Nodal) ---
    /// A = G + (2/T)*C, N×N row-major (trapezoidal forward matrix)
    #[serde(default)]
    pub a_matrix: Vec<f64>,
    /// A_be = G + (1/T)*C, N×N row-major (backward Euler fallback)
    #[serde(default)]
    pub a_matrix_be: Vec<f64>,
    /// A_neg_be = (1/T)*C, N×N row-major (backward Euler history)
    #[serde(default)]
    pub a_neg_be: Vec<f64>,
    /// RHS constant for backward Euler (DC sources × 1, not × 2)
    #[serde(default)]
    pub rhs_const_be: Vec<f64>,

    // --- Schur complement matrices for nodal solver (S = A^{-1}, computed at codegen time) ---
    /// S_be = A_be^{-1}, N×N row-major (backward Euler, for BE fallback in Schur NR)
    #[serde(default)]
    pub s_be: Vec<f64>,
    /// K_be = N_v * S_be * N_i, M×M row-major (backward Euler kernel for BE fallback)
    #[serde(default)]
    pub k_be: Vec<f64>,
    /// Spectral radius of S * A_neg (trapezoidal feedback operator).
    /// Values > 1 mean the Schur path is unstable. Only computed for nodal path.
    #[serde(default)]
    pub spectral_radius_s_aneg: f64,
}

/// Op-amp output voltage saturation for code generation.
///
/// When VSAT is finite, the op-amp output node voltage is clamped to ±VSAT
/// after each LU solve in the nodal solver. This prevents runaway voltages
/// in open-loop or high-gain configurations.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpampIR {
    /// Output node index (0-indexed, in the N-dimensional system)
    pub n_out_idx: usize,
    /// Boyle internal gain node index (0-indexed), if GBW is finite.
    /// This node also needs VSAT clamping to prevent divergence.
    #[serde(default)]
    pub n_internal_idx: Option<usize>,
    /// Output saturation voltage (INFINITY = no saturation)
    pub vsat: f64,
}

/// Potentiometer parameters for code generation (Sherman-Morrison precomputed data).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PotentiometerIR {
    /// S * u where u is the pot's node difference vector (N-vector)
    pub su: Vec<f64>,
    /// u^T * S * u (scalar for SM denominator)
    pub usu: f64,
    /// Nominal conductance 1/R_nom
    pub g_nominal: f64,
    /// N_v * su (M-vector, for K correction in NR loop)
    pub nv_su: Vec<f64>,
    /// su^T * N_i = (S*u)^T * N_i (M-vector, for correction to K and S*N_i products)
    pub u_ni: Vec<f64>,
    /// Positive terminal node index (0 = ground, 1-indexed)
    pub node_p: usize,
    /// Negative terminal node index (0 = ground, 1-indexed)
    pub node_q: usize,
    /// Minimum resistance (ohms)
    pub min_resistance: f64,
    /// Maximum resistance (ohms)
    pub max_resistance: f64,
    /// True if one terminal is grounded
    pub grounded: bool,
}

/// Component within a switch directive for code generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchComponentIR {
    pub name: String,
    /// 'R', 'C', or 'L'
    pub component_type: char,
    /// Node index (1-indexed, 0 = ground)
    pub node_p: usize,
    /// Node index (1-indexed, 0 = ground)
    pub node_q: usize,
    /// Nominal value from netlist
    pub nominal_value: f64,
    /// For 'L' components: index into the inductors vec (for g_eq recomputation)
    pub inductor_index: Option<usize>,
    /// For 'L' components in augmented MNA: row index in augmented C matrix
    /// where the inductance value lives (c_work[k][k] = L). None for DK path
    /// or non-inductor components.
    #[serde(default)]
    pub augmented_row: Option<usize>,
}

/// Switch parameters for code generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchIR {
    /// Switch index (0-based)
    pub index: usize,
    /// Components controlled by this switch
    pub components: Vec<SwitchComponentIR>,
    /// Position values: positions[pos][comp] = value
    pub positions: Vec<Vec<f64>>,
    /// Number of positions
    pub num_positions: usize,
}

/// Inductor parameters for code generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InductorIR {
    pub name: String,
    /// Node index (1-indexed, 0=ground)
    pub node_i: usize,
    /// Node index (1-indexed, 0=ground)
    pub node_j: usize,
    /// Equivalent conductance T/(2L) (at the codegen sample rate)
    pub g_eq: f64,
    /// Raw inductance value in henries (for sample rate recomputation)
    #[serde(default)]
    pub inductance: f64,
}

/// Coupled inductor pair parameters for code generation (transformer).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoupledInductorIR {
    pub name: String,
    pub l1_name: String,
    pub l2_name: String,
    pub l1_node_i: usize,
    pub l1_node_j: usize,
    pub l2_node_i: usize,
    pub l2_node_j: usize,
    pub l1_inductance: f64,
    pub l2_inductance: f64,
    pub coupling: f64,
    /// Self-conductance for L1: (T/2) * L2 / det
    pub g_self_1: f64,
    /// Self-conductance for L2: (T/2) * L1 / det
    pub g_self_2: f64,
    /// Mutual conductance: -(T/2) * M / det
    pub g_mutual: f64,
}

/// Multi-winding transformer group for codegen.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransformerGroupIR {
    pub name: String,
    pub num_windings: usize,
    pub winding_names: Vec<String>,
    pub winding_node_i: Vec<usize>,
    pub winding_node_j: Vec<usize>,
    pub inductances: Vec<f64>,
    /// NxN coupling matrix, flat row-major
    pub coupling_flat: Vec<f64>,
    /// NxN admittance matrix Y = (T/2) * inv(L), flat row-major (at codegen sample rate)
    pub y_matrix: Vec<f64>,
}

// Re-export device types from the shared module (always compiled, no tera dependency).
pub use crate::device_types::{
    BjtParams, DeviceParams, DeviceSlot, DeviceType, DiodeParams, JfetParams, MosfetParams,
    TubeParams, VcaParams,
};

/// Sparsity pattern for a single matrix.
///
/// Stores per-row lists of nonzero column indices, enabling emitters to
/// skip structural zeros without ad-hoc `!= 0.0` checks. Entries with
/// `|x| < SPARSITY_THRESHOLD` are treated as structural zeros.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MatrixSparsity {
    pub rows: usize,
    pub cols: usize,
    /// Total number of nonzero entries
    pub nnz: usize,
    /// For each row, sorted list of column indices with nonzero entries
    pub nz_by_row: Vec<Vec<usize>>,
}

/// Pre-analyzed sparsity information for all compile-time matrices.
///
/// Populated by `analyze_sparsity()` at the end of `from_kernel()`.
/// Emitters use this to generate code that skips structural zeros.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SparseInfo {
    /// A_neg matrix (N×N) — history matrix
    pub a_neg: MatrixSparsity,
    /// N_v matrix (M×N) — voltage extraction
    pub n_v: MatrixSparsity,
    /// N_i matrix (N×M) — current injection
    pub n_i: MatrixSparsity,
    /// K matrix (M×M) — nonlinear kernel
    pub k: MatrixSparsity,
    /// Sparse LU elimination schedule for G_aug (full LU path only)
    pub lu: Option<LuSparsity>,
}

/// Compile-time sparse LU elimination schedule.
///
/// Computed at codegen time from the G_aug = A - N_i*J_dev*N_v sparsity pattern.
/// The emitter generates straight-line code with one operation per `LuOp` —
/// no loops, no indirect indexing, no runtime sparsity tracking.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct LuSparsity {
    /// Fill-reducing column permutation (AMD ordering)
    pub col_perm: Vec<usize>,
    /// Row pre-permutation to fix zero diagonals (voltage source rows)
    pub row_perm: Vec<usize>,
    /// Straight-line elimination operations (in execution order)
    pub ops: Vec<LuOp>,
    /// Nonzero positions in L (below diagonal), after permutation
    pub l_nnz: Vec<(usize, usize)>,
    /// Nonzero positions in U (on/above diagonal), after permutation
    pub u_nnz: Vec<(usize, usize)>,
    /// Total FLOPs for factorization
    pub factor_flops: usize,
    /// Total FLOPs for forward+backward solve
    pub solve_flops: usize,
    /// Matrix dimension
    pub n: usize,
}

/// A single operation in the sparse LU elimination sequence.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LuOp {
    /// `a[row][col] /= a[col][col]` — compute L factor
    DivPivot { row: usize, col: usize },
    /// `a[row][j] -= a[row][col] * a[col][j]` — elimination
    SubMul { row: usize, col: usize, j: usize },
}

/// Threshold below which matrix entries are treated as structural zeros.
const SPARSITY_THRESHOLD: f64 = 1e-20;

/// Analyze sparsity pattern of a flattened row-major matrix.
fn analyze_matrix_sparsity(data: &[f64], rows: usize, cols: usize) -> MatrixSparsity {
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

/// Compute sparsity pattern of G_aug = A - N_i * J_dev * N_v.
///
/// The pattern is the union of A's nonzeros and the Jacobian stamp positions.
/// This is topology-fixed: pot/switch changes only modify values, not positions.
fn compute_g_aug_pattern(
    a_flat: &[f64],
    n_i_flat: &[f64],
    n_v_flat: &[f64],
    n: usize,
    m: usize,
    device_slots: &[DeviceSlot],
) -> Vec<Vec<usize>> {
    // Start with A's nonzero pattern
    let mut pattern: Vec<std::collections::BTreeSet<usize>> =
        (0..n).map(|_| std::collections::BTreeSet::new()).collect();

    for i in 0..n {
        for j in 0..n {
            if a_flat[i * n + j].abs() >= SPARSITY_THRESHOLD {
                pattern[i].insert(j);
            }
        }
    }

    // Add Jacobian stamp positions: N_i[:,dev_i] * N_v[dev_j,:]
    // for each device block (di, dj) within the block-diagonal J_dev
    let mut ni_nz_by_dev = vec![Vec::new(); m];
    for a in 0..n {
        for &i in (0..m).collect::<Vec<_>>().iter() {
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

    pattern.into_iter().map(|s| s.into_iter().collect()).collect()
}

/// Approximate Minimum Degree (AMD) ordering for fill-reducing LU.
///
/// Greedy heuristic: at each step, eliminate the node with the fewest
/// connections in the current elimination graph. This is a simplified AMD
/// that works well for circuit matrices (typically near-optimal for N<100).
fn amd_ordering(pattern: &[Vec<usize>], n: usize) -> Vec<usize> {
    // Build symmetric adjacency (LU fill depends on structural symmetry)
    let mut adj: Vec<std::collections::BTreeSet<usize>> =
        (0..n).map(|i| pattern[i].iter().copied().collect()).collect();
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

/// Row pre-permutation to fix zero diagonals (voltage source constraint rows).
///
/// After column permutation by `col_perm`, some diagonal entries may be
/// structurally zero. This finds row swaps to place nonzeros on the diagonal.
fn fix_zero_diagonals(pattern: &[Vec<usize>], col_perm: &[usize], n: usize) -> Vec<usize> {
    // Build permuted pattern: permuted_row[i] has columns in permuted space
    let mut col_to_pcol = vec![0; n];
    for (pcol, &orig_col) in col_perm.iter().enumerate() {
        col_to_pcol[orig_col] = pcol;
    }

    let mut row_perm: Vec<usize> = (0..n).collect();

    // Greedy: for each column k, ensure row_perm[k]'s row has a nonzero at column col_perm[k]
    for k in 0..n {
        let target_col = col_perm[k];
        // Check if current row has nonzero at target_col
        let cur_row = row_perm[k];
        if pattern[cur_row].contains(&target_col) {
            continue;
        }
        // Find a row below k that has nonzero at target_col
        let mut found = false;
        for j in (k + 1)..n {
            let cand_row = row_perm[j];
            if pattern[cand_row].contains(&target_col) {
                row_perm.swap(k, j);
                found = true;
                break;
            }
        }
        if !found {
            // No row has a nonzero here — matrix may be singular.
            // Leave as-is; dense fallback will handle it at runtime.
            log::warn!(
                "Sparse LU: no nonzero at diagonal position {} (col {}), dense fallback may be needed",
                k, target_col
            );
        }
    }

    row_perm
}

/// Symbolic LU factorization: determine fill-in pattern and elimination schedule.
///
/// Given the G_aug sparsity pattern (after row/column permutation), traces
/// Gaussian elimination symbolically to find:
/// - Which (row, col) positions get fill-in during factorization
/// - The exact sequence of DivPivot and SubMul operations needed
/// - The L and U nonzero patterns for forward/backward substitution
fn symbolic_lu(
    pattern: &[Vec<usize>],
    row_perm: &[usize],
    col_perm: &[usize],
    n: usize,
) -> LuSparsity {
    // Build permuted pattern in (prow, pcol) space
    let mut col_to_pcol = vec![0; n];
    for (pcol, &orig_col) in col_perm.iter().enumerate() {
        col_to_pcol[orig_col] = pcol;
    }

    // pmat[prow][pcol] = true if nonzero
    let mut pmat: Vec<std::collections::BTreeSet<usize>> =
        (0..n).map(|_| std::collections::BTreeSet::new()).collect();
    for prow in 0..n {
        let orig_row = row_perm[prow];
        for &orig_col in &pattern[orig_row] {
            pmat[prow].insert(col_to_pcol[orig_col]);
        }
    }

    let mut ops = Vec::new();
    let mut l_nnz = Vec::new();
    let mut u_nnz = Vec::new();
    let mut factor_flops = 0usize;

    // Gaussian elimination in permuted space
    for col in 0..n {
        // U entries: row=col, all columns >= col that are nonzero in pmat[col]
        for &j in &pmat[col] {
            if j >= col {
                u_nnz.push((col, j));
            }
        }

        // For each row below col that has a nonzero in column col
        let rows_with_entry: Vec<usize> = (col + 1..n)
            .filter(|&r| pmat[r].contains(&col))
            .collect();

        for &row in &rows_with_entry {
            // L factor: a[row][col] /= a[col][col]
            ops.push(LuOp::DivPivot { row, col });
            l_nnz.push((row, col));
            factor_flops += 1;

            // Elimination: for each column j > col in pivot row
            let pivot_cols: Vec<usize> = pmat[col]
                .iter()
                .copied()
                .filter(|&j| j > col)
                .collect();
            for &j in &pivot_cols {
                ops.push(LuOp::SubMul { row, col, j });
                factor_flops += 2; // multiply + subtract
                // Fill-in: position (row, j) becomes nonzero
                pmat[row].insert(j);
            }
        }
    }

    // Solve FLOPs: forward sub touches L entries, backward sub touches U entries
    let solve_flops = l_nnz.len() * 2 + u_nnz.len() * 2;

    log::info!(
        "Sparse LU: N={}, factor_flops={} (dense would be {}), L_nnz={}, U_nnz={}, solve_flops={} (dense would be {})",
        n, factor_flops, n * n * n / 3, l_nnz.len(), u_nnz.len(), solve_flops, n * n * 2,
    );

    LuSparsity {
        col_perm: col_perm.to_vec(),
        row_perm: row_perm.to_vec(),
        ops,
        l_nnz,
        u_nnz,
        factor_flops,
        solve_flops,
        n,
    }
}

/// Stamp mutual conductance between two 2-terminal elements into a flat row-major matrix.
/// Node indices are 1-indexed; 0 means ground.
fn stamp_flat_mutual(mat: &mut [f64], n: usize, a: usize, b: usize, c: usize, d: usize, g: f64) {
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
fn stamp_flat_conductance(mat: &mut [f64], n: usize, node_i: usize, node_j: usize, g: f64) {
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
fn invert_flat_matrix(a: &[f64], n: usize) -> Result<Vec<f64>, CodegenError> {
    // Build augmented [A | I]
    let mut aug = vec![0.0f64; n * 2 * n];
    for i in 0..n {
        for j in 0..n {
            aug[i * 2 * n + j] = a[i * n + j];
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
        if max_val < 1e-30 {
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
        if pivot.abs() < 1e-30 {
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

    // Extract result
    let mut result = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..n {
            result[i * n + j] = aug[i * w + n + j];
        }
    }
    Ok(result)
}

/// Compute K = N_v * S * N_i from flat row-major matrices.
///
/// N_v is M×N, S is N×N, N_i is N×M (all flat row-major).
fn compute_k_from_s(s: &[f64], n_v: &[f64], n_i: &[f64], n: usize, m: usize) -> Vec<f64> {
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
fn validate_positive_finite(value: f64, param_label: &str) -> Result<(), CodegenError> {
    if value <= 0.0 || !value.is_finite() {
        return Err(CodegenError::InvalidConfig(format!(
            "{param_label} must be positive finite, got {value}"
        )));
    }
    Ok(())
}

impl CircuitIR {
    /// Build a `CircuitIR` from the compiled kernel, MNA system, netlist, and config.
    ///
    /// # Errors
    /// Returns `CodegenError::InvalidConfig` if any device model parameter is invalid.
    pub fn from_kernel(
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
    ) -> Result<Self, CodegenError> {
        Self::from_kernel_with_dc_op(kernel, mna, netlist, config, None)
    }

    /// Build CircuitIR from DK kernel with an optional pre-computed DC operating point.
    ///
    /// When `dc_op_result` is provided, it is used instead of running the DC OP solver.
    /// This is useful when the MNA has been expanded with internal nodes after the DC OP
    /// was computed on the original (unexpanded) MNA — the DC OP converges better on
    /// the smaller system.
    pub fn from_kernel_with_dc_op(
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        dc_op_result: Option<dc_op::DcOpResult>,
    ) -> Result<Self, CodegenError> {
        let n = kernel.n; // = n_aug (system dimension)
        let n_nodes = kernel.n_nodes; // original circuit node count
        let m = kernel.m;

        if m > crate::dk::MAX_M {
            return Err(CodegenError::UnsupportedTopology(format!(
                "code generation supports at most M={} nonlinear dimensions, got M={}",
                crate::dk::MAX_M,
                m
            )));
        }

        // Detect augmented inductors: kernel.inductors is empty when from_mna_augmented
        // was used (companion vectors cleared), and kernel.n > mna.n_aug means extra
        // inductor branch variables were added.
        let augmented_inductors = kernel.inductors.is_empty() && (kernel.n > mna.n_aug);

        let topology = Topology {
            n,
            n_nodes,
            m,
            num_devices: kernel.num_devices,
            n_aug: mna.n_aug,
            augmented_inductors,
        };

        let os_factor = config.oversampling_factor;
        let internal_rate = config.sample_rate * os_factor as f64;

        // Auto-detect stiffness: if the trapezoidal time-stepping operator S*A_neg
        // has spectral radius > 1.001 (above stability boundary), the circuit
        // is too stiff for trapezoidal and needs backward Euler.
        // Note: for DK codegen, auto_be=true means the circuit should be routed
        // to the nodal solver instead (BE on DK still diverges for high-S circuits).
        let auto_be = if !config.backward_euler && n > 0 {
            // Compute spectral radius estimate via power iteration on S_trap * A_neg_trap
            let s_trap = &kernel.s;
            let a_neg_trap = &kernel.a_neg;
            // One matrix-vector multiply: y = S * A_neg * x, iterate to find dominant eigenvalue
            let mut x = vec![1.0 / (n as f64).sqrt(); n];
            let mut y = vec![0.0; n];
            let mut spectral_radius = 0.0;
            for _ in 0..20 {
                // 20 power iterations
                // y = S * (A_neg * x)
                let mut ax = vec![0.0; n];
                for i in 0..n {
                    for j in 0..n {
                        ax[i] += a_neg_trap[i * n + j] * x[j];
                    }
                }
                for i in 0..n {
                    y[i] = 0.0;
                    for j in 0..n {
                        y[i] += s_trap[i * n + j] * ax[j];
                    }
                }
                let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
                if norm < 1e-30 {
                    break;
                }
                spectral_radius = norm / x.iter().map(|v| v * v).sum::<f64>().sqrt();
                for i in 0..n {
                    x[i] = y[i] / norm;
                }
            }
            if spectral_radius > 1.002 {
                log::info!("Auto-selecting backward Euler: spectral radius {:.4} > 1.002 (trapezoidal unstable)", spectral_radius);
                true
            } else {
                false
            }
        } else {
            false
        };
        let be = config.backward_euler || auto_be;
        let alpha = if be {
            internal_rate
        } else {
            2.0 * internal_rate
        };

        // Validate output_nodes against circuit node count
        for (i, &node) in config.output_nodes.iter().enumerate() {
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (circuit node count)",
                    i, node, n_nodes
                )));
            }
        }

        let solver_config = SolverConfig {
            sample_rate: config.sample_rate,
            alpha,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_nodes: config.output_nodes.clone(),
            input_resistance: config.input_resistance,
            oversampling_factor: os_factor,
            output_scales: config.output_scales.clone(),
            pot_settle_samples: config.pot_settle_samples,
            backward_euler: be,
        };

        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        // Store the raw G and C matrices for runtime sample rate recomputation.
        // The MNA G matrix already includes input conductance (stamped before kernel build).
        // When augmented inductors are used, kernel.n > mna.n_aug, so we need the
        // augmented G/C (with inductor KCL/KVL/L stamps) at the full n_nodal dimension.
        let (g_matrix, c_matrix) = if augmented_inductors {
            let aug = mna.build_augmented_matrices();
            (
                dk::flatten_matrix(&aug.g, n, n),
                dk::flatten_matrix(&aug.c, n, n),
            )
        } else {
            (
                dk::flatten_matrix(&mna.g, n, n),
                dk::flatten_matrix(&mna.c, n, n),
            )
        };

        let matrices = if os_factor > 1 {
            // Recompute matrices at internal (oversampled) rate from G and C.
            let alpha = 2.0 * internal_rate;
            let t = 1.0 / internal_rate;

            // Build A = G + alpha*C
            let mut a_flat = vec![0.0f64; n * n];
            let mut a_neg_flat = vec![0.0f64; n * n];
            for i in 0..n {
                for j in 0..n {
                    let g = g_matrix[i * n + j];
                    let c = c_matrix[i * n + j];
                    a_flat[i * n + j] = g + alpha * c;
                    a_neg_flat[i * n + j] = alpha * c - g;
                }
            }

            if !augmented_inductors {
                // Companion model path: stamp inductor conductances at internal rate
                for ind in &kernel.inductors {
                    let g_eq = t / (2.0 * ind.inductance);
                    stamp_flat_conductance(&mut a_flat, n, ind.node_i, ind.node_j, g_eq);
                    stamp_flat_conductance(&mut a_neg_flat, n, ind.node_i, ind.node_j, -g_eq);
                }

                // Stamp coupled inductor companion conductances at internal rate
                for ci in &kernel.coupled_inductors {
                    let m_val = ci.coupling * (ci.l1_inductance * ci.l2_inductance).sqrt();
                    let det = ci.l1_inductance * ci.l2_inductance - m_val * m_val;
                    let half_t = t / 2.0;
                    let gs1 = half_t * ci.l2_inductance / det;
                    let gs2 = half_t * ci.l1_inductance / det;
                    let gm = -half_t * m_val / det;
                    stamp_flat_conductance(&mut a_flat, n, ci.l1_node_i, ci.l1_node_j, gs1);
                    stamp_flat_conductance(&mut a_neg_flat, n, ci.l1_node_i, ci.l1_node_j, -gs1);
                    stamp_flat_conductance(&mut a_flat, n, ci.l2_node_i, ci.l2_node_j, gs2);
                    stamp_flat_conductance(&mut a_neg_flat, n, ci.l2_node_i, ci.l2_node_j, -gs2);
                    stamp_flat_mutual(
                        &mut a_flat,
                        n,
                        ci.l1_node_i,
                        ci.l1_node_j,
                        ci.l2_node_i,
                        ci.l2_node_j,
                        gm,
                    );
                    stamp_flat_mutual(
                        &mut a_flat,
                        n,
                        ci.l2_node_i,
                        ci.l2_node_j,
                        ci.l1_node_i,
                        ci.l1_node_j,
                        gm,
                    );
                    stamp_flat_mutual(
                        &mut a_neg_flat,
                        n,
                        ci.l1_node_i,
                        ci.l1_node_j,
                        ci.l2_node_i,
                        ci.l2_node_j,
                        -gm,
                    );
                    stamp_flat_mutual(
                        &mut a_neg_flat,
                        n,
                        ci.l2_node_i,
                        ci.l2_node_j,
                        ci.l1_node_i,
                        ci.l1_node_j,
                        -gm,
                    );
                }
            }

            // Zero VS/VCVS/ideal-transformer algebraic rows in A_neg (NOT inductor
            // rows or internal BJT node rows — they need trapezoidal history).
            for vs in &mna.voltage_sources {
                let row = n_nodes + vs.ext_idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let num_vs = mna.voltage_sources.len();
            for (vcvs_idx, _) in mna.vcvs_sources.iter().enumerate() {
                let row = n_nodes + num_vs + vcvs_idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let num_vcvs = mna.vcvs_sources.len();
            for (xfmr_idx, _) in mna.ideal_transformers.iter().enumerate() {
                let row = n_nodes + num_vs + num_vcvs + xfmr_idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }

            // Invert A to get S
            let s = invert_flat_matrix(&a_flat, n)?;

            // Compute K = N_v * S * N_i
            let k = compute_k_from_s(&s, &kernel.n_v, &kernel.n_i, n, m);

            Matrices {
                s,
                a_neg: a_neg_flat,
                k,
                n_v: kernel.n_v.clone(),
                n_i: kernel.n_i.clone(),
                rhs_const: kernel.rhs_const.clone(),
                g_matrix,
                c_matrix,
                a_matrix: Vec::new(),
                a_matrix_be: Vec::new(),
                a_neg_be: Vec::new(),
                rhs_const_be: Vec::new(),
                s_be: Vec::new(),
                k_be: Vec::new(),
                spectral_radius_s_aneg: 0.0,
            }
        } else if be {
            // Backward Euler: recompute S, A_neg, K from G/C with alpha = 1/T
            let mut a_flat = vec![0.0f64; n * n];
            let mut a_neg_flat = vec![0.0f64; n * n];
            for i in 0..n {
                for j in 0..n {
                    let g = g_matrix[i * n + j];
                    let c = c_matrix[i * n + j];
                    a_flat[i * n + j] = g + alpha * c;
                    a_neg_flat[i * n + j] = alpha * c; // BE: no -G term
                }
            }
            // Zero VS/VCVS algebraic rows in A_neg
            for vs in &mna.voltage_sources {
                let row = mna.n + vs.ext_idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let num_vs = mna.voltage_sources.len();
            for (idx, _) in mna.vcvs_sources.iter().enumerate() {
                let row = mna.n + num_vs + idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let num_vcvs = mna.vcvs_sources.len();
            for (idx, _) in mna.ideal_transformers.iter().enumerate() {
                let row = mna.n + num_vs + num_vcvs + idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let s_flat = invert_flat_matrix(&a_flat, n)?;
            let k_flat = if m > 0 {
                compute_k_from_s(&s_flat, &kernel.n_v, &kernel.n_i, n, m)
            } else {
                Vec::new()
            };
            // BE rhs_const: current sources x1 (not x2), VS x1
            let mut rhs_const_be = vec![0.0f64; n];
            for src in &mna.current_sources {
                crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_plus_idx, src.dc_value);
                crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_minus_idx, -src.dc_value);
            }
            for vs in &mna.voltage_sources {
                let k_row = mna.n + vs.ext_idx;
                if k_row < n {
                    rhs_const_be[k_row] = vs.dc_value;
                }
            }
            Matrices {
                s: s_flat,
                a_neg: a_neg_flat,
                k: k_flat,
                n_v: kernel.n_v.clone(),
                n_i: kernel.n_i.clone(),
                rhs_const: rhs_const_be,
                g_matrix,
                c_matrix,
                a_matrix: Vec::new(),
                a_matrix_be: Vec::new(),
                a_neg_be: Vec::new(),
                rhs_const_be: Vec::new(),
                s_be: Vec::new(),
                k_be: Vec::new(),
                spectral_radius_s_aneg: 0.0,
            }
        } else {
            // Standard trapezoidal: use kernel matrices directly, no BE fallback.
            Matrices {
                s: kernel.s.clone(),
                a_neg: kernel.a_neg.clone(),
                k: kernel.k.clone(),
                n_v: kernel.n_v.clone(),
                n_i: kernel.n_i.clone(),
                rhs_const: kernel.rhs_const.clone(),
                g_matrix,
                c_matrix,
                a_matrix: Vec::new(),
                a_matrix_be: Vec::new(),
                a_neg_be: Vec::new(),
                rhs_const_be: Vec::new(),
                s_be: Vec::new(),
                k_be: Vec::new(),
                spectral_radius_s_aneg: 0.0,
            }
        };

        // Note: BE fallback matrices are only populated when be=true (auto-detected
        // or --backward-euler flag). Normal trapezoidal circuits have empty s_be/k_be/etc.
        // This ensures no state struct layout changes for well-conditioned circuits.

        let mut device_slots = Self::build_device_info_with_mna(netlist, Some(mna))?;
        Self::resolve_mosfet_nodes(&mut device_slots, mna);

        let inductors: Vec<InductorIR> = kernel
            .inductors
            .iter()
            .map(|ind| {
                // Recompute g_eq at internal rate when oversampling
                let g_eq = if os_factor > 1 {
                    1.0 / (2.0 * internal_rate * ind.inductance)
                } else {
                    ind.g_eq
                };
                InductorIR {
                    name: ind.name.to_string(),
                    node_i: ind.node_i,
                    node_j: ind.node_j,
                    g_eq,
                    inductance: ind.inductance,
                }
            })
            .collect();

        let coupled_inductors: Vec<CoupledInductorIR> = kernel
            .coupled_inductors
            .iter()
            .map(|ci| {
                // Recompute conductances at internal rate when oversampling
                let (g_self_1, g_self_2, g_mutual) = if os_factor > 1 {
                    let t = 1.0 / internal_rate;
                    let m_val = ci.coupling * (ci.l1_inductance * ci.l2_inductance).sqrt();
                    let det = ci.l1_inductance * ci.l2_inductance - m_val * m_val;
                    let half_t = t / 2.0;
                    (
                        half_t * ci.l2_inductance / det,
                        half_t * ci.l1_inductance / det,
                        -half_t * m_val / det,
                    )
                } else {
                    (ci.g_self_1, ci.g_self_2, ci.g_mutual)
                };
                CoupledInductorIR {
                    name: ci.name.clone(),
                    l1_name: ci.l1_name.clone(),
                    l2_name: ci.l2_name.clone(),
                    l1_node_i: ci.l1_node_i,
                    l1_node_j: ci.l1_node_j,
                    l2_node_i: ci.l2_node_i,
                    l2_node_j: ci.l2_node_j,
                    l1_inductance: ci.l1_inductance,
                    l2_inductance: ci.l2_inductance,
                    coupling: ci.coupling,
                    g_self_1,
                    g_self_2,
                    g_mutual,
                }
            })
            .collect();

        let transformer_groups: Vec<TransformerGroupIR> = kernel
            .transformer_groups
            .iter()
            .map(|g| {
                let w = g.num_windings;
                // Recompute Y matrix at internal rate when oversampling
                let y_matrix = if os_factor > 1 {
                    let t = 1.0 / internal_rate;
                    let half_t = t / 2.0;
                    let mut l_mat = vec![vec![0.0f64; w]; w];
                    for i in 0..w {
                        for j in 0..w {
                            l_mat[i][j] = g.coupling_matrix[i][j]
                                * (g.inductances[i] * g.inductances[j]).sqrt();
                        }
                    }
                    let y_raw = crate::mna::invert_small_matrix(&l_mat);
                    let mut y_flat = vec![0.0f64; w * w];
                    for i in 0..w {
                        for j in 0..w {
                            y_flat[i * w + j] = half_t * y_raw[i][j];
                        }
                    }
                    y_flat
                } else {
                    g.y_matrix.clone()
                };
                let mut coupling_flat = vec![0.0f64; w * w];
                for i in 0..w {
                    for j in 0..w {
                        coupling_flat[i * w + j] = g.coupling_matrix[i][j];
                    }
                }
                TransformerGroupIR {
                    name: g.name.clone(),
                    num_windings: w,
                    winding_names: g.winding_names.clone(),
                    winding_node_i: g.winding_node_i.clone(),
                    winding_node_j: g.winding_node_j.clone(),
                    inductances: g.inductances.clone(),
                    coupling_flat,
                    y_matrix,
                }
            })
            .collect();

        let pots = kernel
            .pots
            .iter()
            .map(|p| PotentiometerIR {
                su: p.su.clone(),
                usu: p.usu,
                g_nominal: p.g_nominal,
                nv_su: p.nv_su.clone(),
                u_ni: p.u_ni.clone(),
                node_p: p.node_p,
                node_q: p.node_q,
                min_resistance: p.min_resistance,
                max_resistance: p.max_resistance,
                grounded: p.grounded,
            })
            .collect();

        // Build switches from MNA resolved info
        let switches: Vec<SwitchIR> = mna
            .switches
            .iter()
            .enumerate()
            .map(|(idx, sw)| {
                let components = sw
                    .components
                    .iter()
                    .map(|comp| {
                        // For inductor components, find matching index in the inductors vec
                        let inductor_index = if comp.component_type == 'L' {
                            kernel
                                .inductors
                                .iter()
                                .position(|ind| ind.name.eq_ignore_ascii_case(&comp.name))
                        } else {
                            None
                        };
                        SwitchComponentIR {
                            name: comp.name.clone(),
                            component_type: comp.component_type,
                            node_p: comp.node_p,
                            node_q: comp.node_q,
                            nominal_value: comp.nominal_value,
                            inductor_index,
                            augmented_row: None, // DK path uses companion model
                        }
                    })
                    .collect();
                SwitchIR {
                    index: idx,
                    components,
                    positions: sw.positions.clone(),
                    num_positions: sw.positions.len(),
                }
            })
            .collect();

        let has_dc_sources = kernel.rhs_const.iter().any(|&v| v != 0.0);

        // Use pre-computed DC OP if available, otherwise run solver.
        // Pre-computed DC OP is used when the MNA has been expanded with internal
        // nodes — the solver converges better on the original (unexpanded) system.
        let dc_result = if let Some(pre) = dc_op_result {
            pre
        } else {
            let dc_op_config = DcOpConfig {
                tolerance: config.dc_op_tolerance,
                max_iterations: config.dc_op_max_iterations,
                input_node: config.input_node,
                input_resistance: config.input_resistance,
                ..DcOpConfig::default()
            };
            dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config)
        };
        // Check DC OP significance on the truncated vector (n_aug), not the full
        // n_dc vector which includes inductor branch currents.
        let dc_op_len = dc_result.v_node.len();
        let dc_op_truncated = &dc_result.v_node[..kernel.n.min(dc_op_len)];
        let has_dc_op = dc_op_truncated.iter().any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_nl_currents = dc_result.i_nl.clone();

        if !dc_result.converged && m > 0 {
            log::warn!(
                "nonlinear DC OP solver did not converge (method: {:?}), using best estimate",
                dc_result.method
            );
        }

        // Forward-active BJT detection happens BEFORE from_kernel is called.
        // The caller (detect_forward_active_bjts + CLI) handles MNA/kernel rebuild.
        // By the time we get here, kernel/mna already have the correct M dimension.

        // Analyze sparsity patterns for compile-time matrices
        let sparsity = SparseInfo {
            a_neg: analyze_matrix_sparsity(&matrices.a_neg, n, n),
            n_v: analyze_matrix_sparsity(&matrices.n_v, m, n),
            n_i: analyze_matrix_sparsity(&matrices.n_i, n, m),
            k: analyze_matrix_sparsity(&matrices.k, m, m),
            lu: None, // DK path doesn't use full LU
        };

        Ok(CircuitIR {
            metadata,
            topology,
            solver_mode: SolverMode::Dk,
            solver_config,
            matrices,
            // For augmented inductors, the DC OP solver returns n_aug-sized vectors
            // but the kernel dimension is n_nodal = n_aug + n_inductor_vars.
            // Pad with zeros for inductor branch currents (DC OP doesn't solve them).
            // For companion model, truncate to kernel.n (= n_aug).
            dc_operating_point: {
                // DC OP may return fewer nodes than kernel.n (e.g., when computed
                // on unexpanded MNA before internal node expansion). Pad with zeros.
                let mut dc = dc_result.v_node.clone();
                dc.resize(kernel.n, 0.0);
                dc.truncate(kernel.n);
                // Clamp op-amp Boyle internal nodes to VSAT.
                // The DC OP solver doesn't know about VSAT and can converge
                // to voltages beyond rail limits at internal nodes.
                for oa in &mna.opamps {
                    if oa.vsat.is_finite() && oa.n_out_idx > 0 {
                        let o = oa.n_out_idx - 1;
                        if o < dc.len() {
                            dc[o] = dc[o].clamp(-oa.vsat, oa.vsat);
                        }
                    }
                }
                dc
            },
            device_slots,
            has_dc_sources,
            has_dc_op,
            dc_nl_currents,
            dc_op_converged,
            dc_block: config.dc_block,
            inductors,
            coupled_inductors,
            transformer_groups,
            pots,
            switches,
            opamps: mna
                .opamps
                .iter()
                .filter(|oa| oa.vsat.is_finite() && oa.n_out_idx > 0)
                .map(|oa| OpampIR {
                    n_out_idx: oa.n_out_idx - 1,
                    n_internal_idx: if oa.n_internal_idx > 0 {
                        Some(oa.n_internal_idx - 1)
                    } else {
                        None
                    },
                    vsat: oa.vsat,
                })
                .collect(),
            sparsity,
        })
    }

    /// Build CircuitIR for the nodal solver path (no DkKernel needed).
    ///
    /// Uses augmented MNA: inductors are branch current variables in G/C.
    /// The generated code does full N×N NR per sample instead of DK's M×M.
    pub fn from_mna(
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
    ) -> Result<Self, CodegenError> {
        let n_nodes = mna.n;
        let n_aug = mna.n_aug;
        let m = mna.m;

        if m > dk::MAX_M {
            return Err(CodegenError::InvalidConfig(format!(
                "Nonlinear dimension M={} exceeds MAX_M={}",
                m,
                dk::MAX_M
            )));
        }

        // Build augmented G/C matrices (includes inductor branch variables)
        let mut aug = mna.build_augmented_matrices();
        let n = aug.n_nodal;

        // Gmin regularization: prevent singular Jacobians on floating nodes.
        // Matches runtime NodalSolver (solver.rs Gmin stamping).
        for i in 0..n_nodes {
            aug.g[i][i] += 1e-12;
        }

        let sample_rate = config.sample_rate;
        let internal_rate = sample_rate * config.oversampling_factor as f64;
        let alpha = if config.backward_euler {
            internal_rate
        } else {
            2.0 * internal_rate
        };
        let alpha_be = internal_rate;

        let topology = Topology {
            n,
            n_nodes,
            m,
            num_devices: mna.num_devices,
            n_aug,
            augmented_inductors: true,
        };

        // Validate output_nodes against circuit node count
        for (i, &node) in config.output_nodes.iter().enumerate() {
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (circuit node count)",
                    i, node, n_nodes
                )));
            }
        }

        let solver_config = SolverConfig {
            sample_rate,
            alpha,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_nodes: config.output_nodes.clone(),
            input_resistance: config.input_resistance,
            oversampling_factor: config.oversampling_factor,
            output_scales: config.output_scales.clone(),
            pot_settle_samples: config.pot_settle_samples,
            backward_euler: config.backward_euler,
        };

        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        // Flatten G and C at n_nodal dimension
        let g_matrix = dk::flatten_matrix(&aug.g, n, n);
        let c_matrix = dk::flatten_matrix(&aug.c, n, n);

        // Build A = G + alpha*C, A_neg = alpha*C - G (trapezoidal) or alpha*C (BE)
        let be = config.backward_euler;
        let mut a_flat = vec![0.0f64; n * n];
        let mut a_neg_flat = vec![0.0f64; n * n];
        for i in 0..n {
            for j in 0..n {
                let g = aug.g[i][j];
                let c = aug.c[i][j];
                a_flat[i * n + j] = g + alpha * c;
                a_neg_flat[i * n + j] = if be { alpha * c } else { alpha * c - g };
            }
        }
        // Zero ALL augmented rows in A_neg (n_nodes..n_aug).
        // This matches the runtime NodalSolver (solver.rs line ~3199) which
        // zeros all rows from n_nodes..n_aug, not just VS/VCVS rows.
        //
        // For augmented variables that are algebraic (VS, VCVS), this is
        // mandatory — they have no dynamics (no C), so A_neg = -G and the
        // trapezoidal history term A_neg*v_prev would inject unstable feedback.
        //
        // For Boyle op-amp internal nodes, zeroing A_neg effectively uses
        // backward Euler instead of trapezoidal for the dominant pole. This is
        // unconditionally stable and avoids the Gm-induced spectral radius > 1
        // instability that trapezoidal creates for high-gain VCCS elements
        // (AOL>10000 → Gm>1000 → A_neg has ±1000 entries → unstable feedback).
        //
        // Inductor branch rows (n_aug..n_nodal) are NOT zeroed — they have
        // real L dynamics in C that need trapezoidal integration.
        if n_aug > n_nodes {
            for i in n_nodes..n_aug {
                if i < n {
                    for j in 0..n {
                        a_neg_flat[i * n + j] = 0.0;
                    }
                }
            }
        }

        // Build backward Euler: A_be = G + (1/T)*C, A_neg_be = (1/T)*C
        let mut a_be_flat = vec![0.0f64; n * n];
        let mut a_neg_be_flat = vec![0.0f64; n * n];
        for i in 0..n {
            for j in 0..n {
                let g = aug.g[i][j];
                let c = aug.c[i][j];
                a_be_flat[i * n + j] = g + alpha_be * c;
                a_neg_be_flat[i * n + j] = alpha_be * c;
            }
        }
        // Zero all augmented rows in A_neg_be (matching A_neg treatment)
        if n_aug > n_nodes {
            for i in n_nodes..n_aug {
                if i < n {
                    for j in 0..n {
                        a_neg_be_flat[i * n + j] = 0.0;
                    }
                }
            }
        }

        // Expand N_v (m × n_aug → m × n_nodal) and N_i (n_aug × m → n_nodal × m)
        let mut n_v_flat = vec![0.0f64; m * n];
        for i in 0..m {
            for j in 0..n_aug {
                n_v_flat[i * n + j] = mna.n_v[i][j];
            }
        }
        let mut n_i_flat = vec![0.0f64; n * m];
        for i in 0..n_aug {
            for j in 0..m {
                n_i_flat[i * m + j] = mna.n_i[i][j];
            }
        }

        // Build rhs_const: trapezoidal (node rows ×2, VS rows ×1) or BE (all ×1)
        let rhs_const = if be {
            // BE: current sources ×1 (not ×2), VS ×1
            let mut rc = vec![0.0f64; n];
            for src in &mna.current_sources {
                crate::mna::inject_rhs_current(&mut rc, src.n_plus_idx, src.dc_value);
                crate::mna::inject_rhs_current(&mut rc, src.n_minus_idx, -src.dc_value);
            }
            for vs in &mna.voltage_sources {
                let k = mna.n + vs.ext_idx;
                if k < n {
                    rc[k] = vs.dc_value;
                }
            }
            rc
        } else {
            let rhs_const_base = dk::build_rhs_const(mna);
            let mut rc = vec![0.0f64; n];
            for i in 0..n_aug {
                rc[i] = rhs_const_base[i];
            }
            rc
        };

        // Build BE rhs_const (node rows ×1, VS rows ×1) — for fallback
        let mut rhs_const_be = vec![0.0f64; n];
        for src in &mna.current_sources {
            crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_plus_idx, src.dc_value);
            crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_minus_idx, -src.dc_value);
        }
        for vs in &mna.voltage_sources {
            let k = mna.n + vs.ext_idx;
            if k < n {
                rhs_const_be[k] = vs.dc_value;
            }
        }

        // Compute S = A^{-1} for Schur complement NR (O(M³) instead of O(N³) per iteration)
        let s_flat = invert_flat_matrix(&a_flat, n)?;
        let k_flat = if m > 0 {
            compute_k_from_s(&s_flat, &n_v_flat, &n_i_flat, n, m)
        } else {
            Vec::new()
        };

        // Also compute S_be = A_be^{-1} for backward Euler fallback
        let s_be_flat = invert_flat_matrix(&a_be_flat, n)?;
        let k_be_flat = if m > 0 {
            compute_k_from_s(&s_be_flat, &n_v_flat, &n_i_flat, n, m)
        } else {
            Vec::new()
        };

        // Compute spectral radius of S * A_neg to detect Schur instability.
        // When rho(S * A_neg) > 1, the trapezoidal feedback v_pred = S*(A_neg*v_prev + ...)
        // amplifies errors exponentially. Route to full LU NR instead.
        let spectral_radius_s_aneg = if n > 0 && !s_flat.is_empty() {
            let mut x = vec![1.0 / (n as f64).sqrt(); n];
            let mut rho = 0.0f64;
            for _ in 0..100 {
                // y = S * (A_neg * x)
                let mut ax = vec![0.0; n];
                for i in 0..n {
                    for j in 0..n {
                        ax[i] += a_neg_flat[i * n + j] * x[j];
                    }
                }
                let mut y = vec![0.0; n];
                for i in 0..n {
                    for j in 0..n {
                        y[i] += s_flat[i * n + j] * ax[j];
                    }
                }
                let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
                if norm < 1e-30 {
                    break;
                }
                rho = norm / x.iter().map(|v| v * v).sum::<f64>().sqrt();
                for v in &mut x {
                    *v = 0.0;
                }
                for (i, yi) in y.iter().enumerate() {
                    x[i] = yi / norm;
                }
            }
            if rho > 0.999 {
                log::info!(
                    "Nodal: spectral_radius(S*A_neg) = {:.4} (> 0.999, Schur path unstable)",
                    rho
                );
            }
            rho
        } else {
            0.0
        };

        let matrices = Matrices {
            s: s_flat,
            k: k_flat,
            a_neg: a_neg_flat,
            n_v: n_v_flat,
            n_i: n_i_flat,
            rhs_const,
            g_matrix,
            c_matrix,
            a_matrix: a_flat,
            a_matrix_be: a_be_flat,
            a_neg_be: a_neg_be_flat,
            rhs_const_be,
            s_be: s_be_flat,
            k_be: k_be_flat,
            spectral_radius_s_aneg,
        };

        // Run DC OP
        let dc_op_config = DcOpConfig {
            input_node: config.input_node,
            input_resistance: config.input_resistance,
            ..DcOpConfig::default()
        };
        // Build device info with MNA so FA reductions are reflected in dimensions
        let mut device_slots = Self::build_device_info_with_mna(netlist, Some(mna))?;

        let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);
        let dc_op_truncated = &dc_result.v_node[..n_aug.min(dc_result.v_node.len())];
        let has_dc_op = dc_op_truncated.iter().any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_nl_currents = dc_result.i_nl.clone();
        let has_dc_sources = !mna.voltage_sources.is_empty() || !mna.current_sources.is_empty();

        // Pad DC OP to n_nodal (inductor branch currents = 0)
        let mut dc_operating_point = dc_result.v_node.clone();
        dc_operating_point.resize(n, 0.0);
        // Clamp op-amp output + internal Boyle nodes to VSAT
        for oa in &mna.opamps {
            if oa.vsat.is_finite() && oa.n_out_idx > 0 {
                let o = oa.n_out_idx - 1;
                if o < dc_operating_point.len() {
                    dc_operating_point[o] = dc_operating_point[o].clamp(-oa.vsat, oa.vsat);
                }
                if oa.n_internal_idx > 0 {
                    let int = oa.n_internal_idx - 1;
                    if int < dc_operating_point.len() {
                        dc_operating_point[int] = dc_operating_point[int].clamp(-oa.vsat, oa.vsat);
                    }
                }
            }
        }
        Self::resolve_mosfet_nodes(&mut device_slots, mna);

        // Sparsity analysis (K is now computed for Schur complement NR)
        let lu_sparsity = if m > 0 {
            // Compute G_aug = A - N_i*J_dev*N_v sparsity pattern
            let g_aug_pattern = compute_g_aug_pattern(
                &matrices.a_matrix, &matrices.n_i, &matrices.n_v,
                n, m, &device_slots,
            );
            let g_aug_nnz: usize = g_aug_pattern.iter().map(|r| r.len()).sum();
            let density = g_aug_nnz as f64 / (n * n) as f64;
            log::info!(
                "Sparse LU: G_aug pattern has {} nonzeros out of {} ({:.1}% density)",
                g_aug_nnz, n * n, density * 100.0
            );
            // Only use sparse LU if matrix is sufficiently sparse (< 40% density)
            // and large enough to benefit (N >= 8)
            // TODO: sparse LU has a permutation bug — disabled until fixed.
            // When enabled, reduces factorization from O(N³) to ~1000 FLOPs for Pultec.
            if false && density < 0.4 && n >= 8 {
                let col_perm = amd_ordering(&g_aug_pattern, n);
                let row_perm = fix_zero_diagonals(&g_aug_pattern, &col_perm, n);
                let lu = symbolic_lu(&g_aug_pattern, &row_perm, &col_perm, n);
                Some(lu)
            } else {
                log::info!("Sparse LU: skipping (density {:.1}%, N={})", density * 100.0, n);
                None
            }
        } else {
            None
        };

        let sparsity = SparseInfo {
            a_neg: analyze_matrix_sparsity(&matrices.a_neg, n, n),
            n_v: analyze_matrix_sparsity(&matrices.n_v, m, n),
            n_i: analyze_matrix_sparsity(&matrices.n_i, n, m),
            k: analyze_matrix_sparsity(&matrices.k, m, m),
            lu: lu_sparsity,
        };

        Ok(CircuitIR {
            metadata,
            topology,
            solver_mode: SolverMode::Nodal,
            solver_config,
            matrices,
            dc_operating_point,
            device_slots,
            has_dc_sources,
            has_dc_op,
            dc_nl_currents,
            dc_op_converged,
            dc_block: config.dc_block,
            inductors: Vec::new(), // no companion model
            coupled_inductors: Vec::new(),
            transformer_groups: Vec::new(),
            pots: mna
                .pots
                .iter()
                .map(|p| PotentiometerIR {
                    su: Vec::new(), // not used in nodal (no Sherman-Morrison)
                    usu: 0.0,
                    g_nominal: p.g_nominal,
                    nv_su: Vec::new(),
                    u_ni: Vec::new(),
                    node_p: p.node_p,
                    node_q: p.node_q,
                    min_resistance: p.min_resistance,
                    max_resistance: p.max_resistance,
                    grounded: p.grounded,
                })
                .collect(),
            switches: {
                // Build inductor name → augmented row mapping for switch L components.
                // In augmented MNA, each inductor's L value lives on the C matrix diagonal
                // at row n_aug + offset (not at circuit node rows).
                let mut inductor_aug_rows: std::collections::HashMap<String, usize> =
                    std::collections::HashMap::new();
                let mut var_idx = n_aug;
                for ind in &mna.inductors {
                    inductor_aug_rows.insert(ind.name.to_ascii_uppercase(), var_idx);
                    var_idx += 1;
                }
                for ci in &mna.coupled_inductors {
                    inductor_aug_rows.insert(ci.l1_name.to_ascii_uppercase(), var_idx);
                    inductor_aug_rows.insert(ci.l2_name.to_ascii_uppercase(), var_idx + 1);
                    var_idx += 2;
                }
                for group in &mna.transformer_groups {
                    for (widx, name) in group.winding_names.iter().enumerate() {
                        inductor_aug_rows.insert(name.to_ascii_uppercase(), var_idx + widx);
                    }
                    var_idx += group.num_windings;
                }

                mna.switches
                    .iter()
                    .enumerate()
                    .map(|(idx, sw)| SwitchIR {
                        index: idx,
                        components: sw
                            .components
                            .iter()
                            .map(|comp| {
                                let augmented_row = if comp.component_type == 'L' {
                                    inductor_aug_rows
                                        .get(&comp.name.to_ascii_uppercase())
                                        .copied()
                                } else {
                                    None
                                };
                                SwitchComponentIR {
                                    name: comp.name.clone(),
                                    component_type: comp.component_type,
                                    node_p: comp.node_p,
                                    node_q: comp.node_q,
                                    nominal_value: comp.nominal_value,
                                    inductor_index: None,
                                    augmented_row,
                                }
                            })
                            .collect(),
                        positions: sw.positions.clone(),
                        num_positions: sw.positions.len(),
                    })
                    .collect()
            },
            opamps: mna
                .opamps
                .iter()
                .filter(|oa| oa.vsat.is_finite() && oa.n_out_idx > 0)
                .map(|oa| OpampIR {
                    n_out_idx: oa.n_out_idx - 1,
                    n_internal_idx: if oa.n_internal_idx > 0 {
                        Some(oa.n_internal_idx - 1)
                    } else {
                        None
                    },
                    vsat: oa.vsat,
                })
                .collect(),
            sparsity,
        })
    }

    /// Build device slot map and resolve per-device parameters from netlist.
    ///
    /// # Errors
    /// Returns `CodegenError::InvalidConfig` if any device model parameter is non-positive or non-finite.
    /// Detect BJTs that are forward-active at the DC operating point.
    ///
    /// Returns the names (uppercased) of BJTs with Vbc < -1.0V that can be
    /// modeled as 1D (Vbe→Ic only), reducing M by 1 each.
    ///
    /// Call this BEFORE building the final MNA/kernel. If non-empty, rebuild
    /// MNA with `from_netlist_forward_active()` and kernel before calling `from_kernel()`.
    pub fn detect_forward_active_bjts(
        mna: &crate::mna::MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
    ) -> std::collections::HashSet<String> {
        use crate::dc_op::{self, DcOpConfig};

        let device_slots = Self::build_device_info(netlist).unwrap_or_default();
        if device_slots.is_empty() {
            return std::collections::HashSet::new();
        }

        let dc_op_config = DcOpConfig {
            tolerance: config.dc_op_tolerance,
            max_iterations: config.dc_op_max_iterations,
            input_node: config.input_node,
            input_resistance: config.input_resistance,
            ..DcOpConfig::default()
        };
        let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);

        let mut forward_active = std::collections::HashSet::new();
        for (slot_idx, slot) in device_slots.iter().enumerate() {
            if slot.device_type == DeviceType::Bjt && slot_idx < mna.nonlinear_devices.len() {
                let bp = if let DeviceParams::Bjt(bp) = &slot.params {
                    bp
                } else {
                    continue;
                };
                let dev = &mna.nonlinear_devices[slot_idx];
                let nc = dev.node_indices[0];
                let nb = dev.node_indices[1];
                let v_c = if nc > 0 && nc - 1 < dc_result.v_node.len() {
                    dc_result.v_node[nc - 1]
                } else {
                    0.0
                };
                let v_b = if nb > 0 && nb - 1 < dc_result.v_node.len() {
                    dc_result.v_node[nb - 1]
                } else {
                    0.0
                };
                let vbc = v_b - v_c;
                let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                let vbc_eff = sign * vbc;
                // Forward-active: allow parasitics and GP models.
                // 1D FA ignores parasitics (small for forward-active: Ib*RB << Vbe).
                // GP qb(Vbc) is ~constant when Vbc is deeply reverse-biased.
                if vbc_eff < -1.0 {
                    let name = dev.name.to_ascii_uppercase();
                    log::info!(
                        "BJT '{}' forward-active (Vbc={:.3}V). Using 1D model.",
                        name,
                        vbc_eff
                    );
                    forward_active.insert(name);
                }
            }
        }
        forward_active
    }

    pub fn build_device_info(netlist: &Netlist) -> Result<Vec<DeviceSlot>, CodegenError> {
        Self::build_device_info_with_mna(netlist, None)
    }

    /// Build device info, optionally using MNA device dimensions (for forward-active BJTs).
    pub fn build_device_info_with_mna(
        netlist: &Netlist,
        mna: Option<&crate::mna::MnaSystem>,
    ) -> Result<Vec<DeviceSlot>, CodegenError> {
        let mut slots = Vec::new();
        let mut dim_offset = 0;
        let mut nl_dev_idx = 0; // tracks position in mna.nonlinear_devices

        for elem in &netlist.elements {
            match elem {
                Element::Diode { model, .. } => {
                    let params = Self::resolve_diode_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Diode,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Diode(params),
                        has_internal_mna_nodes: false,
                    });
                    dim_offset += 1;
                    nl_dev_idx += 1;
                }
                Element::Bjt { name, model, .. } => {
                    // Skip linearized BJTs — they're not in the nonlinear system
                    let is_linearized = mna.is_some_and(|m| {
                        m.linearized_bjts
                            .iter()
                            .any(|l| l.name.eq_ignore_ascii_case(name))
                    });
                    if is_linearized {
                        continue; // Don't create a DeviceSlot, don't increment nl_dev_idx
                    }
                    let params = Self::resolve_bjt_params(netlist, model)?;
                    // Check if MNA has this BJT as forward-active (1D)
                    let is_fa = mna.is_some_and(|m| {
                        nl_dev_idx < m.nonlinear_devices.len()
                            && m.nonlinear_devices[nl_dev_idx].device_type
                                == crate::mna::NonlinearDeviceType::BjtForwardActive
                    });
                    let (dev_type, dim) = if is_fa {
                        (DeviceType::BjtForwardActive, 1)
                    } else {
                        (DeviceType::Bjt, 2)
                    };
                    slots.push(DeviceSlot {
                        device_type: dev_type,
                        start_idx: dim_offset,
                        dimension: dim,
                        params: DeviceParams::Bjt(params),
                        has_internal_mna_nodes: false,
                    });
                    dim_offset += dim;
                    nl_dev_idx += 1;
                }
                Element::Jfet { model, .. } => {
                    let params = Self::resolve_jfet_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Jfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Jfet(params),
                        has_internal_mna_nodes: false,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Triode { model, .. } => {
                    let params = Self::resolve_tube_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Tube,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Tube(params),
                        has_internal_mna_nodes: false,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Mosfet { model, .. } => {
                    let params = Self::resolve_mosfet_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Mosfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Mosfet(params),
                        has_internal_mna_nodes: false,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Vca { model, .. } => {
                    let params = Self::resolve_vca_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Vca,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Vca(params),
                        has_internal_mna_nodes: false,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                _ => {}
            }
        }

        // Mark BJTs with MNA-level internal nodes
        if let Some(m) = mna {
            for slot in &mut slots {
                if slot.device_type == DeviceType::Bjt
                    && m.bjt_internal_nodes
                        .iter()
                        .any(|n| n.start_idx == slot.start_idx)
                {
                    slot.has_internal_mna_nodes = true;
                }
            }
        }

        Ok(slots)
    }

    /// Resolve MOSFET source/bulk node indices from MNA nonlinear device info.
    ///
    /// Called after `build_device_info` to populate `source_node` and `bulk_node`
    /// fields in MosfetParams, which are needed for body effect (GAMMA/PHI).
    fn resolve_mosfet_nodes(slots: &mut [DeviceSlot], mna: &MnaSystem) {
        let mut mosfet_idx = 0;
        for slot in slots.iter_mut() {
            if let DeviceParams::Mosfet(ref mut mp) = slot.params {
                if mp.has_body_effect() {
                    // Find the matching MOSFET in MNA nonlinear_devices
                    for dev in &mna.nonlinear_devices {
                        if dev.device_type == crate::mna::NonlinearDeviceType::Mosfet
                            && dev.start_idx == slot.start_idx
                        {
                            // node_indices: [drain, gate, source, bulk]
                            // node_indices are 1-based (0 = ground)
                            // For the N-dimensional system, node index i maps to v[i-1]
                            mp.source_node = dev.node_indices[2];
                            mp.bulk_node = dev.node_indices[3];
                            break;
                        }
                    }
                }
                mosfet_idx += 1;
            }
        }
        let _ = mosfet_idx; // suppress unused warning
    }

    /// Resolve diode model parameters from the netlist, with validation.
    ///
    /// Resolution order: explicit `.model` param → catalog → generic default.
    fn resolve_diode_params(netlist: &Netlist, model: &str) -> Result<DiodeParams, CodegenError> {
        let vt = melange_primitives::VT_ROOM;
        let cat = melange_devices::catalog::diodes::lookup(model);
        let is = Self::lookup_model_param(netlist, model, "IS")
            .or_else(|| cat.map(|c| c.is))
            .unwrap_or(2.52e-9);
        let n = Self::lookup_model_param(netlist, model, "N")
            .or_else(|| cat.map(|c| c.n))
            .unwrap_or(1.0);

        validate_positive_finite(is, "diode model IS")?;
        validate_positive_finite(n, "diode model N")?;

        // Junction capacitance (optional, default 0.0)
        let cjo = Self::lookup_model_param(netlist, model, "CJO").unwrap_or(0.0);
        if cjo < 0.0 || !cjo.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model CJO must be non-negative and finite, got {cjo}"
            )));
        }

        // Series resistance (optional, default 0.0)
        let rs = Self::lookup_model_param(netlist, model, "RS").unwrap_or(0.0);
        if rs < 0.0 || !rs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model RS must be non-negative and finite, got {rs}"
            )));
        }

        // Reverse breakdown voltage (optional, default infinity = disabled)
        let bv = Self::lookup_model_param(netlist, model, "BV").unwrap_or(f64::INFINITY);
        if bv.is_finite() {
            validate_positive_finite(bv, "diode model BV")?;
        }

        // Reverse breakdown current (optional, default 1e-10)
        let ibv = Self::lookup_model_param(netlist, model, "IBV").unwrap_or(1e-10);
        if ibv.is_finite() {
            validate_positive_finite(ibv, "diode model IBV")?;
        }

        Self::warn_unrecognized_params(netlist, model, &["IS", "N", "CJO", "RS", "BV", "IBV"]);

        Ok(DiodeParams {
            is,
            n_vt: n * vt,
            cjo,
            rs,
            bv,
            ibv,
        })
    }

    /// Resolve BJT model parameters from the netlist, with validation.
    ///
    /// Gummel-Poon parameters (VAF, VAR, IKF, IKR) default to infinity,
    /// which collapses qb→1.0, giving exact Ebers-Moll behavior.
    fn resolve_bjt_params(netlist: &Netlist, model: &str) -> Result<BjtParams, CodegenError> {
        let cat = melange_devices::catalog::bjts::lookup(model);
        let vt = Self::lookup_model_param(netlist, model, "VT")
            .or_else(|| cat.map(|c| c.vt))
            .unwrap_or(melange_primitives::VT_ROOM);
        let is = Self::lookup_model_param(netlist, model, "IS")
            .or_else(|| cat.map(|c| c.is))
            .unwrap_or(1.26e-14);
        let beta_f = Self::lookup_model_param(netlist, model, "BF")
            .or_else(|| cat.map(|c| c.beta_f))
            .unwrap_or(200.0);
        let beta_r = Self::lookup_model_param(netlist, model, "BR")
            .or_else(|| cat.map(|c| c.beta_r))
            .unwrap_or(3.0);

        validate_positive_finite(is, "BJT model IS")?;
        validate_positive_finite(vt, "BJT model VT")?;
        validate_positive_finite(beta_f, "BJT model BF")?;
        validate_positive_finite(beta_r, "BJT model BR")?;

        let is_pnp = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PNP"))
            .unwrap_or(cat.map(|c| c.is_pnp).unwrap_or(false));

        // Gummel-Poon parameters (default to infinity = pure Ebers-Moll)
        let vaf = Self::lookup_model_param(netlist, model, "VAF")
            .or_else(|| Self::lookup_model_param(netlist, model, "VA"))
            .or_else(|| cat.map(|c| c.vaf))
            .unwrap_or(f64::INFINITY);
        let var = Self::lookup_model_param(netlist, model, "VAR")
            .or_else(|| Self::lookup_model_param(netlist, model, "VB"))
            .or_else(|| cat.map(|c| c.var))
            .unwrap_or(f64::INFINITY);
        let ikf = Self::lookup_model_param(netlist, model, "IKF")
            .or_else(|| Self::lookup_model_param(netlist, model, "JBF"))
            .or_else(|| cat.map(|c| c.ikf))
            .unwrap_or(f64::INFINITY);
        let ikr = Self::lookup_model_param(netlist, model, "IKR")
            .or_else(|| Self::lookup_model_param(netlist, model, "JBR"))
            .or_else(|| cat.map(|c| c.ikr))
            .unwrap_or(f64::INFINITY);

        // Validate: if finite, must be positive
        if vaf.is_finite() {
            validate_positive_finite(vaf, "BJT model VAF")?;
        }
        if var.is_finite() {
            validate_positive_finite(var, "BJT model VAR")?;
        }
        if ikf.is_finite() {
            validate_positive_finite(ikf, "BJT model IKF")?;
        }
        if ikr.is_finite() {
            validate_positive_finite(ikr, "BJT model IKR")?;
        }

        // Junction capacitances (optional, default 0.0)
        let cje = Self::lookup_model_param(netlist, model, "CJE").unwrap_or(0.0);
        if cje < 0.0 || !cje.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CJE must be non-negative and finite, got {cje}"
            )));
        }
        let cjc = Self::lookup_model_param(netlist, model, "CJC").unwrap_or(0.0);
        if cjc < 0.0 || !cjc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CJC must be non-negative and finite, got {cjc}"
            )));
        }

        // Forward emission coefficient (default 1.0 = ideal)
        let nf = Self::lookup_model_param(netlist, model, "NF").unwrap_or(1.0);
        if nf <= 0.0 || !nf.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NF must be positive and finite, got {nf}"
            )));
        }

        // B-E leakage saturation current (default 0.0 = disabled)
        let ise = Self::lookup_model_param(netlist, model, "ISE").unwrap_or(0.0);
        if ise < 0.0 || !ise.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model ISE must be non-negative and finite, got {ise}"
            )));
        }

        // B-E leakage emission coefficient (default 1.5)
        let ne = Self::lookup_model_param(netlist, model, "NE").unwrap_or(1.5);
        if ne <= 0.0 || !ne.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NE must be positive and finite, got {ne}"
            )));
        }

        // Reverse emission coefficient (default 1.0 = ideal)
        let nr = Self::lookup_model_param(netlist, model, "NR").unwrap_or(1.0);
        if nr <= 0.0 || !nr.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NR must be positive and finite, got {nr}"
            )));
        }

        // B-C leakage saturation current (default 0.0 = disabled)
        let isc = Self::lookup_model_param(netlist, model, "ISC").unwrap_or(0.0);
        if isc < 0.0 || !isc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model ISC must be non-negative and finite, got {isc}"
            )));
        }

        // B-C leakage emission coefficient (default 2.0)
        let nc = Self::lookup_model_param(netlist, model, "NC").unwrap_or(2.0);
        if nc <= 0.0 || !nc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NC must be positive and finite, got {nc}"
            )));
        }

        // Parasitic series resistances (optional, default 0.0)
        let rb = Self::lookup_model_param(netlist, model, "RB").unwrap_or(0.0);
        let rc = Self::lookup_model_param(netlist, model, "RC").unwrap_or(0.0);
        let re = Self::lookup_model_param(netlist, model, "RE").unwrap_or(0.0);
        if rb < 0.0 || !rb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RB must be non-negative and finite, got {rb}"
            )));
        }
        if rc < 0.0 || !rc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RC must be non-negative and finite, got {rc}"
            )));
        }
        if re < 0.0 || !re.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RE must be non-negative and finite, got {re}"
            )));
        }

        // Self-heating parameters (optional)
        let rth = Self::lookup_model_param(netlist, model, "RTH").unwrap_or(f64::INFINITY);
        if rth.is_finite() && rth <= 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RTH must be positive (or infinite to disable), got {rth}"
            )));
        }

        let cth = Self::lookup_model_param(netlist, model, "CTH").unwrap_or(1e-3);
        if cth <= 0.0 || !cth.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CTH must be positive and finite, got {cth}"
            )));
        }

        let xti = Self::lookup_model_param(netlist, model, "XTI").unwrap_or(3.0);
        if !xti.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model XTI must be finite, got {xti}"
            )));
        }

        let eg = Self::lookup_model_param(netlist, model, "EG").unwrap_or(1.11);
        if eg <= 0.0 || !eg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model EG must be positive and finite, got {eg}"
            )));
        }

        let tamb = Self::lookup_model_param(netlist, model, "TAMB").unwrap_or(300.15);
        if tamb <= 0.0 || !tamb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model TAMB must be positive and finite, got {tamb}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &[
                "IS", "BF", "BR", "VAF", "VAR", "IKF", "IKR", "CJE", "CJC", "NF", "NR", "ISE",
                "NE", "ISC", "NC", "RB", "RC", "RE", "RTH", "CTH", "XTI", "EG", "TAMB",
            ],
        );

        Ok(BjtParams {
            is,
            vt,
            beta_f,
            beta_r,
            is_pnp,
            vaf,
            var,
            ikf,
            ikr,
            cje,
            cjc,
            nf,
            nr,
            ise,
            ne,
            isc,
            nc,
            rb,
            rc,
            re,
            rth,
            cth,
            xti,
            eg,
            tamb,
        })
    }

    /// Resolve JFET model parameters from the netlist, with validation.
    ///
    /// 2D Shichman-Hodges: IDSS, VP, and LAMBDA control triode + saturation regions.
    fn resolve_jfet_params(netlist: &Netlist, model: &str) -> Result<JfetParams, CodegenError> {
        let cat = melange_devices::catalog::jfets::lookup(model);

        // Determine channel type first — default VP depends on polarity.
        let is_p_channel = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
            .unwrap_or(cat.map(|c| c.is_p_channel).unwrap_or(false));

        let default_vp = cat
            .map(|c| c.vp)
            .unwrap_or(if is_p_channel { 2.0 } else { -2.0 });
        let vp = Self::lookup_model_param(netlist, model, "VTO").unwrap_or(default_vp);
        // ngspice BETA = IDSS / VP^2, so IDSS = BETA * VP^2
        let idss = if let Some(raw_idss) = Self::lookup_model_param(netlist, model, "IDSS") {
            raw_idss
        } else if let Some(beta) = Self::lookup_model_param(netlist, model, "BETA") {
            beta * vp * vp
        } else {
            cat.map(|c| c.idss).unwrap_or(2e-3)
        };
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.001);

        validate_positive_finite(idss, "JFET model IDSS")?;
        if !vp.is_finite() || vp.abs() < 1e-15 {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model VP must be finite and nonzero, got {vp}"
            )));
        }
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Junction capacitances (optional, default 0.0)
        let cgs = Self::lookup_model_param(netlist, model, "CGS").unwrap_or(0.0);
        if cgs < 0.0 || !cgs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model CGS must be non-negative and finite, got {cgs}"
            )));
        }
        let cgd = Self::lookup_model_param(netlist, model, "CGD").unwrap_or(0.0);
        if cgd < 0.0 || !cgd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model CGD must be non-negative and finite, got {cgd}"
            )));
        }

        // Ohmic drain/source resistances (optional, default 0.0)
        let rd = Self::lookup_model_param(netlist, model, "RD").unwrap_or(0.0);
        if rd < 0.0 || !rd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model RD must be non-negative and finite, got {rd}"
            )));
        }
        let rs_param = Self::lookup_model_param(netlist, model, "RS").unwrap_or(0.0);
        if rs_param < 0.0 || !rs_param.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model RS must be non-negative and finite, got {rs_param}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &["VTO", "BETA", "IDSS", "LAMBDA", "CGS", "CGD", "RD", "RS"],
        );

        Ok(JfetParams {
            idss,
            vp,
            lambda,
            is_p_channel,
            cgs,
            cgd,
            rd,
            rs_param,
        })
    }

    /// Resolve MOSFET model parameters from the netlist, with validation.
    fn resolve_mosfet_params(netlist: &Netlist, model: &str) -> Result<MosfetParams, CodegenError> {
        let cat = melange_devices::catalog::mosfets::lookup(model);

        let is_p_channel = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PM"))
            .unwrap_or(cat.map(|c| c.is_p_channel).unwrap_or(false));

        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(0.1);
        let default_vt = cat
            .map(|c| c.vt)
            .unwrap_or(if is_p_channel { -2.0 } else { 2.0 });
        let vt = Self::lookup_model_param(netlist, model, "VTO")
            .or_else(|| Self::lookup_model_param(netlist, model, "VT"))
            .unwrap_or(default_vt);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.01);

        validate_positive_finite(kp, "MOSFET model KP")?;
        if !vt.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model VT must be finite, got {vt}"
            )));
        }
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Junction capacitances (optional, default 0.0)
        let cgs = Self::lookup_model_param(netlist, model, "CGS").unwrap_or(0.0);
        if cgs < 0.0 || !cgs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model CGS must be non-negative and finite, got {cgs}"
            )));
        }
        let cgd = Self::lookup_model_param(netlist, model, "CGD").unwrap_or(0.0);
        if cgd < 0.0 || !cgd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model CGD must be non-negative and finite, got {cgd}"
            )));
        }

        // Ohmic drain/source resistances (optional, default 0.0)
        let rd = Self::lookup_model_param(netlist, model, "RD").unwrap_or(0.0);
        if rd < 0.0 || !rd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model RD must be non-negative and finite, got {rd}"
            )));
        }
        let rs_param = Self::lookup_model_param(netlist, model, "RS").unwrap_or(0.0);
        if rs_param < 0.0 || !rs_param.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model RS must be non-negative and finite, got {rs_param}"
            )));
        }

        // Body effect parameters (optional, default 0.0 = disabled)
        let gamma = Self::lookup_model_param(netlist, model, "GAMMA").unwrap_or(0.0);
        let phi = Self::lookup_model_param(netlist, model, "PHI").unwrap_or(0.6);
        if gamma < 0.0 || !gamma.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model GAMMA must be non-negative and finite, got {gamma}"
            )));
        }
        if phi <= 0.0 || !phi.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model PHI must be positive and finite, got {phi}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &[
                "KP", "VTO", "LAMBDA", "CGS", "CGD", "RD", "RS", "GAMMA", "PHI",
            ],
        );

        // source_node and bulk_node will be resolved later from the MNA system
        Ok(MosfetParams {
            kp,
            vt,
            lambda,
            is_p_channel,
            cgs,
            cgd,
            rd,
            rs_param,
            gamma,
            phi,
            source_node: 0,
            bulk_node: 0,
        })
    }

    /// Resolve tube/triode model parameters from the netlist, with validation.
    ///
    /// Resolution order: explicit `.model` param → catalog → generic default (12AX7).
    fn resolve_tube_params(netlist: &Netlist, model: &str) -> Result<TubeParams, CodegenError> {
        let cat = melange_devices::catalog::tubes::lookup(model);
        let mu = Self::lookup_model_param(netlist, model, "MU")
            .or_else(|| cat.map(|c| c.mu))
            .unwrap_or(100.0);
        let ex = Self::lookup_model_param(netlist, model, "EX")
            .or_else(|| cat.map(|c| c.ex))
            .unwrap_or(1.4);
        let kg1 = Self::lookup_model_param(netlist, model, "KG1")
            .or_else(|| cat.map(|c| c.kg1))
            .unwrap_or(1060.0);
        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(600.0);
        let kvb = Self::lookup_model_param(netlist, model, "KVB")
            .or_else(|| cat.map(|c| c.kvb))
            .unwrap_or(300.0);
        let ig_max = Self::lookup_model_param(netlist, model, "IG_MAX")
            .or_else(|| cat.map(|c| c.ig_max))
            .unwrap_or(2e-3);
        let vgk_onset = Self::lookup_model_param(netlist, model, "VGK_ONSET")
            .or_else(|| cat.map(|c| c.vgk_onset))
            .unwrap_or(0.5);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.0);

        validate_positive_finite(mu, "tube model MU")?;
        validate_positive_finite(ex, "tube model EX")?;
        validate_positive_finite(kg1, "tube model KG1")?;
        validate_positive_finite(kp, "tube model KP")?;
        validate_positive_finite(kvb, "tube model KVB")?;
        validate_positive_finite(ig_max, "tube model IG_MAX")?;
        validate_positive_finite(vgk_onset, "tube model VGK_ONSET")?;

        // Validate optional lambda: must be non-negative and finite
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Inter-electrode capacitances (optional, default 0.0)
        let ccg = Self::lookup_model_param(netlist, model, "CCG").unwrap_or(0.0);
        if ccg < 0.0 || !ccg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CCG must be non-negative and finite, got {ccg}"
            )));
        }
        let cgp = Self::lookup_model_param(netlist, model, "CGP").unwrap_or(0.0);
        if cgp < 0.0 || !cgp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CGP must be non-negative and finite, got {cgp}"
            )));
        }
        let ccp = Self::lookup_model_param(netlist, model, "CCP").unwrap_or(0.0);
        if ccp < 0.0 || !ccp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CCP must be non-negative and finite, got {ccp}"
            )));
        }

        // Grid internal resistance (optional, default 0.0 = disabled)
        let rgi = Self::lookup_model_param(netlist, model, "RGI").unwrap_or(0.0);
        if rgi < 0.0 || !rgi.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model RGI must be non-negative and finite, got {rgi}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &[
                "MU",
                "EX",
                "KG1",
                "KP",
                "KVB",
                "IG_MAX",
                "VGK_ONSET",
                "LAMBDA",
                "CCG",
                "CGP",
                "CCP",
                "RGI",
            ],
        );

        Ok(TubeParams {
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max,
            vgk_onset,
            lambda,
            ccg,
            cgp,
            ccp,
            rgi,
        })
    }

    /// Resolve VCA model parameters from the netlist, with validation.
    ///
    /// 2D current-mode exponential gain: I_sig = G0 * exp(-Vc / VSCALE) * V_sig
    fn resolve_vca_params(netlist: &Netlist, model: &str) -> Result<VcaParams, CodegenError> {
        let vscale = Self::lookup_model_param(netlist, model, "VSCALE").unwrap_or(0.05298);
        let g0 = Self::lookup_model_param(netlist, model, "G0").unwrap_or(1.0);
        let thd = Self::lookup_model_param(netlist, model, "THD").unwrap_or(0.0);
        let noise_floor = Self::lookup_model_param(netlist, model, "NOISE_FLOOR").unwrap_or(0.0);

        validate_positive_finite(vscale, "VCA model VSCALE")?;
        validate_positive_finite(g0, "VCA model G0")?;
        if thd < 0.0 || !thd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "VCA model THD must be non-negative and finite, got {thd}"
            )));
        }
        if noise_floor < 0.0 || !noise_floor.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "VCA model NOISE_FLOOR must be non-negative and finite, got {noise_floor}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &["VSCALE", "G0", "THD", "NOISE_FLOOR", "MODE"],
        );

        Ok(VcaParams {
            vscale,
            g0,
            thd,
            noise_floor,
        })
    }

    /// Warn on unrecognized .model parameters (typo protection).
    fn warn_unrecognized_params(netlist: &Netlist, model_name: &str, known: &[&str]) {
        if let Some(m) = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
        {
            for (key, _) in &m.params {
                let upper = key.to_ascii_uppercase();
                if !known.iter().any(|k| k.eq_ignore_ascii_case(&upper)) {
                    log::warn!(
                        ".model {}: unrecognized parameter '{}' (ignored)",
                        model_name,
                        key,
                    );
                }
            }
        }
    }

    /// Look up a parameter from a `.model` directive, case-insensitive.
    fn lookup_model_param(netlist: &Netlist, model_name: &str, param_name: &str) -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param_name))
                    .map(|(_, v)| *v)
            })
    }

    /// Access S matrix element S[i][j]
    pub fn s(&self, i: usize, j: usize) -> f64 {
        self.matrices.s[i * self.topology.n + j]
    }

    /// Access K matrix element K[i][j]
    pub fn k(&self, i: usize, j: usize) -> f64 {
        self.matrices.k[i * self.topology.m + j]
    }

    /// Access N_v matrix element N_v[i][j]
    pub fn n_v(&self, i: usize, j: usize) -> f64 {
        self.matrices.n_v[i * self.topology.n + j]
    }

    /// Access N_i matrix element N_i[i][j] (N×M storage: node × device)
    pub fn n_i(&self, i: usize, j: usize) -> f64 {
        self.matrices.n_i[i * self.topology.m + j]
    }

    /// Access A_neg matrix element A_neg[i][j]
    pub fn a_neg(&self, i: usize, j: usize) -> f64 {
        self.matrices.a_neg[i * self.topology.n + j]
    }

    /// Access G matrix element G[i][j]
    pub fn g(&self, i: usize, j: usize) -> f64 {
        self.matrices.g_matrix[i * self.topology.n + j]
    }

    /// Access C matrix element C[i][j]
    pub fn c(&self, i: usize, j: usize) -> f64 {
        self.matrices.c_matrix[i * self.topology.n + j]
    }

    /// Access A matrix element A[i][j] (trapezoidal, nodal mode only)
    pub fn a_matrix(&self, i: usize, j: usize) -> f64 {
        self.matrices.a_matrix[i * self.topology.n + j]
    }

    /// Access A_be matrix element A_be[i][j] (backward Euler, nodal mode only)
    pub fn a_matrix_be(&self, i: usize, j: usize) -> f64 {
        self.matrices.a_matrix_be[i * self.topology.n + j]
    }

    /// Access A_neg_be matrix element A_neg_be[i][j] (backward Euler history, nodal mode only)
    pub fn a_neg_be(&self, i: usize, j: usize) -> f64 {
        self.matrices.a_neg_be[i * self.topology.n + j]
    }

    /// Access S_be matrix element S_be[i][j] (backward Euler, nodal Schur)
    pub fn s_be(&self, i: usize, j: usize) -> f64 {
        self.matrices.s_be[i * self.topology.n + j]
    }

    /// Access K_be matrix element K_be[i][j] (backward Euler, nodal Schur)
    pub fn k_be(&self, i: usize, j: usize) -> f64 {
        self.matrices.k_be[i * self.topology.m + j]
    }
}
