//! Small matrix helpers: inversion and ground-aware conductance, VCCS and RHS stamps.

/// Invert a small NxN matrix using Gaussian elimination with partial pivoting.
/// Used for multi-winding transformer inductance matrix inversion.
///
/// Returns `Err` with a description when the matrix holds a non-finite entry
/// or a pivot falls below `1e-30` (singular). The `1e-30` threshold is
/// absolute, so it catches exact singularity only: a near-singular matrix
/// (coupling k → 1) can still come back as a large, inaccurate inverse.
/// Callers that need a scale-relative check (the DK kernel build) apply
/// their own residual test on the result.
pub(crate) fn invert_small_matrix(a: &[Vec<f64>]) -> Result<Vec<Vec<f64>>, String> {
    let n = a.len();
    // Guard: NaN/Inf bypass the pivot < 1e-30 singularity check
    for i in 0..n {
        for j in 0..n {
            if !a[i][j].is_finite() {
                return Err(format!(
                    "non-finite value {} in inductance matrix at [{i}][{j}]",
                    a[i][j]
                ));
            }
        }
    }
    // Build augmented matrix [A | I]
    let mut aug = vec![vec![0.0f64; 2 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][n + i] = 1.0;
    }
    // Forward elimination with partial pivoting
    for col in 0..n {
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            return Err(format!(
                "singular inductance matrix (pivot {:.2e} in column {col})",
                max_val
            ));
        }
        if max_row != col {
            aug.swap(col, max_row);
        }
        let pivot = aug[col][col];
        for j in col..(2 * n) {
            aug[col][j] /= pivot;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in col..(2 * n) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }
    // Extract inverse from augmented matrix
    let mut result = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            result[i][j] = aug[i][n + j];
        }
    }
    Ok(result)
}

/// Stamp a conductance `g` between two nodes that may be grounded (index 0).
///
/// This is the standard MNA conductance stamp with ground-node handling:
/// - Both nodes non-ground: full 2x2 stamp into the matrix
/// - One node grounded: single diagonal entry
/// - Both grounded: no-op
///
/// Node indices use the MNA convention where 0 = ground (excluded from matrix),
/// and non-zero indices are 1-based (matrix row/col = index - 1).
pub(super) fn stamp_conductance_to_ground(
    mat: &mut [Vec<f64>],
    node_i: usize,
    node_j: usize,
    g: f64,
) {
    match (node_i > 0, node_j > 0) {
        (true, true) => {
            let i = node_i - 1;
            let j = node_j - 1;
            mat[i][i] += g;
            mat[j][j] += g;
            mat[i][j] -= g;
            mat[j][i] -= g;
        }
        (true, false) => {
            mat[node_i - 1][node_i - 1] += g;
        }
        (false, true) => {
            mat[node_j - 1][node_j - 1] += g;
        }
        (false, false) => {}
    }
}

/// Inject a current into the RHS vector at a node, handling ground (index 0).
///
/// Positive current is injected at `node` (node_map convention: 0 = ground).
pub(crate) fn inject_rhs_current(rhs: &mut [f64], node: usize, current: f64) {
    if node > 0 {
        rhs[node - 1] += current;
    }
}

/// Stamp a voltage-controlled current source (VCCS, SPICE `G` element) into
/// the G matrix.
///
/// Node-current direction (explicit, do NOT "harmonize" with the op-amp
/// stamps): the element DRAWS `I = gm * (V_ctrl_p - V_ctrl_n)` OUT of node
/// `out_p` and injects it INTO node `out_n` — i.e. positive current flows
/// from `out_p` through the source to `out_n` (standard SPICE G-element
/// convention). The op-amp VCCS stamps use the OPPOSITE orientation
/// (they inject `+Gm·(V+ − V−)` INTO the output node) and therefore carry a
/// negated gm; both are correct for their element.
/// Node indices use MNA convention: 0 = ground (excluded from matrix).
///
/// G stamps (G[k][j]·Vj = current LEAVING node k):
///   G[out_p, ctrl_p] += gm
///   G[out_p, ctrl_n] -= gm
///   G[out_n, ctrl_p] -= gm
///   G[out_n, ctrl_n] += gm
pub(super) fn stamp_vccs(
    mat: &mut [Vec<f64>],
    out_p: usize,
    out_n: usize,
    ctrl_p: usize,
    ctrl_n: usize,
    gm: f64,
) {
    if out_p > 0 {
        let o = out_p - 1;
        if ctrl_p > 0 {
            mat[o][ctrl_p - 1] += gm;
        }
        if ctrl_n > 0 {
            mat[o][ctrl_n - 1] -= gm;
        }
    }
    if out_n > 0 {
        let o = out_n - 1;
        if ctrl_p > 0 {
            mat[o][ctrl_p - 1] -= gm;
        }
        if ctrl_n > 0 {
            mat[o][ctrl_n - 1] += gm;
        }
    }
}
