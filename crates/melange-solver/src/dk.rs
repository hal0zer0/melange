//! DK (Discrete K)-method kernel extraction.
//!
//! Implements the Discrete K-method from David Yeh's 2009 thesis:
//! "Digital Implementation of Musical Distortion Circuits by Analysis and Simulation"
//!
//! The core idea: Reduce an N-node linear system to an M×M nonlinear kernel,
//! where M is the total voltage dimension (sum of device dimensions).
//!
//! For example:
//! - 1 diode: M = 1
//! - 1 BJT: M = 2 (Vbe, Vbc)
//! - 2 diodes + 1 BJT: M = 1 + 1 + 2 = 4

use crate::mna::{MnaSystem, VS_CONDUCTANCE};
use std::sync::Arc;

/// Information about an inductor for companion model.
#[derive(Debug, Clone)]
pub struct InductorInfo {
    pub name: Arc<str>,
    pub node_i: usize,
    pub node_j: usize,
    pub inductance: f64,
    /// Equivalent conductance g_eq = T/(2L)
    pub g_eq: f64,
    /// History current for companion model
    pub i_hist: f64,
    /// Previous inductor current
    pub i_prev: f64,
    /// Previous voltage across inductor
    pub v_prev: f64,
}

/// Precomputed Sherman-Morrison data for a potentiometer.
///
/// When a pot changes resistance from R_nom to R_new, the conductance change
/// `delta_g = 1/R_new - 1/R_nom` produces a rank-1 update to the A matrix.
/// These precomputed vectors enable O(N^2) correction instead of O(N^3) re-inversion.
#[derive(Debug, Clone)]
pub struct SmPotData {
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

/// DK kernel for a circuit.
///
/// This is the reduced representation used for efficient simulation.
/// All matrices are stored in flattened row-major format for better cache locality.
#[derive(Debug, Clone)]
pub struct DkKernel {
    /// Number of nodes (original system dimension)
    pub n: usize,
    /// Total voltage dimension (sum of device dimensions)
    pub m: usize,
    /// Number of physical nonlinear devices
    pub num_devices: usize,
    /// Sample rate
    pub sample_rate: f64,
    /// Precomputed S = A^{-1} matrix (N x N), row-major: index = i * n + j
    pub s: Vec<f64>,
    /// Nonlinear kernel K = N_v * S * N_i (M x M), row-major: index = i * m + j
    pub k: Vec<f64>,
    /// Voltage extraction matrix N_v (M x N), row-major: index = i * n + j
    pub n_v: Vec<f64>,
    /// Current injection matrix N_i (N x M), row-major: index = i * m + j
    pub n_i: Vec<f64>,
    /// A_neg matrix for history term (N x N), row-major: index = i * n + j
    pub a_neg: Vec<f64>,
    /// Constant sources contribution to RHS (N)
    pub rhs_const: Vec<f64>,
    /// Inductor companion model info
    pub inductors: Vec<InductorInfo>,
    /// Potentiometer Sherman-Morrison precomputed data
    pub pots: Vec<SmPotData>,
}

/// Error type for DK reduction.
#[derive(Debug, Clone)]
pub enum DkError {
    /// The A matrix is singular or nearly singular and cannot be inverted.
    SingularMatrix(String),
    /// An invalid parameter was provided (e.g., non-positive sample rate).
    InvalidParameter(String),
    /// An inductor has a non-positive inductance value.
    InvalidInductance { name: String, value: f64 },
    /// A K matrix diagonal entry is non-negative, indicating incorrect feedback polarity.
    ///
    /// K diagonals must be negative for stable NR convergence. A non-negative diagonal
    /// means the circuit topology or N_i/N_v wiring is incorrect.
    InvalidKDiagonal {
        /// Index of the offending diagonal entry.
        index: usize,
        /// The non-negative value found.
        value: f64,
    },
}

impl std::fmt::Display for DkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DkError::SingularMatrix(msg) => write!(f, "DK error: {}", msg),
            DkError::InvalidParameter(msg) => write!(f, "DK error: {}", msg),
            DkError::InvalidInductance { name, value } => {
                write!(f, "DK error: inductor '{}' has non-positive inductance: {}", name, value)
            }
            DkError::InvalidKDiagonal { index, value } => {
                write!(
                    f,
                    "DK error: K diagonal [{}][{}] = {} is non-negative (expected < 0 for stable feedback)",
                    index, index, value
                )
            }
        }
    }
}

impl std::error::Error for DkError {}

/// Maximum supported nonlinear dimension (sum of all device dimensions).
///
/// Circuits with more than MAX_M nonlinear dimensions are rejected to prevent
/// unbounded allocation and O(M^3) NR solve cost.
pub const MAX_M: usize = 8;

impl DkKernel {
    /// Build DK kernel from MNA system.
    ///
    /// # Errors
    /// Returns an error if `sample_rate` is not positive and finite, if the
    /// A matrix is singular, if any inductor has non-positive inductance,
    /// or if the total nonlinear dimension exceeds [`MAX_M`].
    pub fn from_mna(mna: &MnaSystem, sample_rate: f64) -> Result<Self, DkError> {
        if !(sample_rate > 0.0 && sample_rate.is_finite()) {
            return Err(DkError::InvalidParameter(
                format!("sample_rate must be positive and finite, got {}", sample_rate)
            ));
        }

        let n = mna.n;
        let m = mna.m;

        if m > MAX_M {
            return Err(DkError::InvalidParameter(
                format!("nonlinear dimension m={} exceeds MAX_M={}", m, MAX_M)
            ));
        }

        // Get A matrix
        let a = mna.get_a_matrix(sample_rate);

        // Invert A to get S
        let s_2d = invert_matrix(&a)
            .map_err(|e| DkError::SingularMatrix(format!("Failed to invert A matrix: {}", e)))?;

        // Condition number estimate: cond(A) ~ ||A||_inf * ||S||_inf
        // If cond > 1e12, emit a diagnostic warning (not a hard error).
        {
            let norm_a = infinity_norm(&a);
            let norm_s = infinity_norm(&s_2d);
            let cond = norm_a * norm_s;
            if cond > 1e12 {
                eprintln!(
                    "Warning: A matrix condition number estimate is {:.2e} (threshold 1e12). \
                     Results may be numerically inaccurate.",
                    cond
                );
            }
        }

        let s = flatten_matrix(&s_2d, n, n);

        // Compute K = N_v * S * N_i  (M x M)
        //
        // No negation needed. N_i uses the "current injection" convention:
        //   N_i[anode] = -1 (current extracted from anode node)
        //   N_i[cathode] = +1 (current injected into cathode node)
        // This means K is naturally negative for stable circuits,
        // providing the correct negative feedback for NR convergence.
        //
        // First: S * N_i (N x M)
        let s_ni = mat_mul(&s_2d, &mna.n_i);
        // Then: N_v * (S * N_i) (M x M)
        let k_2d = mat_mul(&mna.n_v, &s_ni);

        // Validate K diagonal: all K[i][i] must be negative for stable NR feedback.
        // A non-negative diagonal indicates incorrect circuit topology or wiring.
        // Only check when M > 0 (nonlinear devices present); for M=0, K is empty.
        if m > 0 {
            for i in 0..m {
                if k_2d[i][i] >= 0.0 {
                    return Err(DkError::InvalidKDiagonal {
                        index: i,
                        value: k_2d[i][i],
                    });
                }
            }
        }

        let k = flatten_matrix(&k_2d, m, m);

        // Get A_neg matrix for history
        let a_neg_2d = mna.get_a_neg_matrix(sample_rate);
        let a_neg = flatten_matrix(&a_neg_2d, n, n);

        // Build constant sources from voltage and current sources
        let rhs_const = build_rhs_const(mna, sample_rate);

        // Precompute Sherman-Morrison vectors for pots
        let mut pots = Vec::with_capacity(mna.pots.len());
        for pot_info in &mna.pots {
            // Build the u vector: u[p-1] = 1, u[q-1] = -1 (for non-grounded)
            // If one terminal is grounded, only the non-ground terminal has +1
            let mut u = vec![0.0; n];
            if pot_info.node_p > 0 {
                u[pot_info.node_p - 1] = 1.0;
            }
            if pot_info.node_q > 0 {
                u[pot_info.node_q - 1] = -1.0;
            }

            // su = S * u (N-vector)
            let su = mat_vec_mul(&s_2d, &u);

            // usu = u^T * S * u (scalar)
            let mut usu = 0.0;
            for i in 0..n {
                usu += u[i] * su[i];
            }

            // nv_su = N_v * su (M-vector)
            let nv_su = if m > 0 {
                mat_vec_mul(&mna.n_v, &su)
            } else {
                Vec::new()
            };

            // u_ni[j] = su^T * N_i[:,j] = (S*u)^T * N_i[:,j] (M-vector)
            // Sherman-Morrison requires su^T*N_i (not u^T*N_i) because
            // S' = S - scale * (S*u) * (u^T*S) and S is symmetric, so u^T*S = su^T
            let mut u_ni = vec![0.0; m];
            for j in 0..m {
                let mut sum = 0.0;
                for i in 0..n {
                    sum += su[i] * mna.n_i[i][j];
                }
                u_ni[j] = sum;
            }

            pots.push(SmPotData {
                su,
                usu,
                g_nominal: pot_info.g_nominal,
                nv_su,
                u_ni,
                node_p: pot_info.node_p,
                node_q: pot_info.node_q,
                min_resistance: pot_info.min_resistance,
                max_resistance: pot_info.max_resistance,
                grounded: pot_info.grounded,
            });
        }

        // Flatten N_v and N_i from MNA (which still uses 2D format)
        let n_v = flatten_matrix(&mna.n_v, m, n);
        let n_i = flatten_matrix(&mna.n_i, n, m);

        // Initialize inductor companion models
        let t = 1.0 / sample_rate;
        let mut inductors = Vec::with_capacity(mna.inductors.len());
        for ind in &mna.inductors {
            if ind.value <= 0.0 {
                return Err(DkError::InvalidInductance {
                    name: ind.name.clone(),
                    value: ind.value,
                });
            }
            let g_eq = t / (2.0 * ind.value);  // T/(2L)
            inductors.push(InductorInfo {
                name: Arc::from(ind.name.clone().into_boxed_str()),
                node_i: ind.node_i,
                node_j: ind.node_j,
                inductance: ind.value,
                g_eq,
                i_hist: 0.0,
                i_prev: 0.0,
                v_prev: 0.0,
            });
        }

        Ok(Self {
            n,
            m,
            num_devices: mna.num_devices,
            sample_rate,
            s,
            k,
            n_v,
            n_i,
            a_neg,
            rhs_const,
            inductors,
            pots,
        })
    }

    /// Access S matrix element S[i][j]
    #[inline(always)]
    pub fn s(&self, i: usize, j: usize) -> f64 {
        self.s[i * self.n + j]
    }

    /// Access K matrix element K[i][j]
    #[inline(always)]
    pub fn k(&self, i: usize, j: usize) -> f64 {
        self.k[i * self.m + j]
    }

    /// Access N_v matrix element N_v[i][j]
    #[inline(always)]
    pub fn n_v(&self, i: usize, j: usize) -> f64 {
        self.n_v[i * self.n + j]
    }

    /// Access N_i matrix element N_i[i][j]
    #[inline(always)]
    pub fn n_i(&self, i: usize, j: usize) -> f64 {
        self.n_i[i * self.m + j]
    }

    /// Access A_neg matrix element A_neg[i][j]
    #[inline(always)]
    pub fn a_neg(&self, i: usize, j: usize) -> f64 {
        self.a_neg[i * self.n + j]
    }

    /// Compute linear prediction: v_pred = S * rhs
    ///
    /// Writes result into the provided output buffer.
    #[allow(clippy::needless_range_loop)]
    pub fn predict_into(&self, rhs: &[f64], v_pred: &mut [f64]) {
        assert_eq!(rhs.len(), self.n);
        assert_eq!(v_pred.len(), self.n);

        for i in 0..self.n {
            let mut sum = 0.0;
            let row_offset = i * self.n;
            for j in 0..self.n {
                sum += self.s[row_offset + j] * rhs[j];
            }
            v_pred[i] = sum;
        }
    }

    /// Extract predicted nonlinear voltages: p = N_v * v_pred
    ///
    /// Writes result into the provided output buffer.
    #[allow(clippy::needless_range_loop)]
    pub fn extract_prediction_into(&self, v_pred: &[f64], p: &mut [f64]) {
        assert_eq!(v_pred.len(), self.n);
        assert_eq!(p.len(), self.m);

        for i in 0..self.m {
            let mut sum = 0.0;
            let row_offset = i * self.n;
            for j in 0..self.n {
                sum += self.n_v[row_offset + j] * v_pred[j];
            }
            p[i] = sum;
        }
    }

    /// Compute kernel contribution: k_contrib = K * i_nl
    ///
    /// Writes result into the provided output buffer.
    #[allow(clippy::needless_range_loop)]
    pub fn kernel_contribution_into(&self, i_nl: &[f64], k_contrib: &mut [f64]) {
        assert_eq!(i_nl.len(), self.m);
        assert_eq!(k_contrib.len(), self.m);

        for i in 0..self.m {
            let mut sum = 0.0;
            let row_offset = i * self.m;
            for j in 0..self.m {
                sum += self.k[row_offset + j] * i_nl[j];
            }
            k_contrib[i] = sum;
        }
    }

    /// Apply correction: v = v_pred + S * N_i * (i_nl - i_nl_prev)
    ///
    /// This computes the final node voltages after solving for i_nl.
    /// The correction uses the delta current since v_pred already includes
    /// the effect of i_nl_prev from the RHS.
    ///
    /// Writes result into the provided output buffer.
    /// 
    /// # Arguments
    /// - `v_pred`: Prediction vector (n elements)
    /// - `i_nl`: New nonlinear currents (m elements)
    /// - `i_nl_prev`: Previous nonlinear currents (m elements)
    /// - `v`: Output buffer for result (n elements)
    /// - `ni_inl`: Temporary buffer for N_i * di_nl (n elements, pre-allocated)
    #[allow(clippy::needless_range_loop)]
    pub fn apply_correction_into(&self, v_pred: &[f64], i_nl: &[f64], i_nl_prev: &[f64], v: &mut [f64], ni_inl: &mut [f64]) {
        assert_eq!(v_pred.len(), self.n);
        assert_eq!(i_nl.len(), self.m);
        assert_eq!(i_nl_prev.len(), self.m);
        assert_eq!(v.len(), self.n);
        assert_eq!(ni_inl.len(), self.n);

        // v = v_pred + S * N_i * (i_nl - i_nl_prev)
        // First compute N_i * (i_nl - i_nl_prev) (N vector)
        for i in 0..self.n {
            let mut sum = 0.0;
            let row_offset = i * self.m;
            for j in 0..self.m {
                let di_nl = i_nl[j] - i_nl_prev[j];
                sum += self.n_i[row_offset + j] * di_nl;
            }
            ni_inl[i] = sum;
        }

        // Then S * (N_i * di_nl)
        for i in 0..self.n {
            let mut sum = v_pred[i]; // Start with prediction
            let row_offset = i * self.n;
            for j in 0..self.n {
                sum += self.s[row_offset + j] * ni_inl[j];
            }
            v[i] = sum;
        }
    }

    /// Update inductor state after solving.
    ///
    /// Call this after computing final node voltages to update
    /// inductor current and history for next timestep.
    ///
    /// Uses trapezoidal companion model:
    /// - Equivalent conductance: g_eq = T/(2L)
    /// - History current: J_n = i_{n-1} + g_eq * v_{n-1}
    ///
    /// The history current J_n is the Norton equivalent source.
    pub fn update_inductors(&mut self, v_node: &[f64]) {
        for ind in &mut self.inductors {
            // Get voltage across inductor at current timestep
            let v_i = if ind.node_i > 0 { v_node[ind.node_i - 1] } else { 0.0 };
            let v_j = if ind.node_j > 0 { v_node[ind.node_j - 1] } else { 0.0 };
            let v_l_new = v_i - v_j;
            
            let g_eq = ind.g_eq;  // T/(2L)
            
            // Update inductor current using trapezoidal rule:
            // i_new = i_prev + g_eq * (v_prev + v_new)
            let i_new = ind.i_prev + g_eq * (ind.v_prev + v_l_new);
            
            // Compute history current for NEXT timestep's companion model:
            // J = i_new - g_eq * v_new = i_prev + g_eq * v_prev
            // This is the Norton equivalent current source.
            ind.i_hist = i_new - g_eq * v_l_new;
            
            // Save state for next iteration
            ind.i_prev = i_new;
            ind.v_prev = v_l_new;
        }
    }
}

/// Build constant RHS vector from DC sources.
///
/// For trapezoidal rule, DC sources contribute 2*V to RHS.
#[allow(clippy::needless_range_loop)]
fn build_rhs_const(mna: &MnaSystem, _sample_rate: f64) -> Vec<f64> {
    let mut rhs = vec![0.0; mna.n];

    // Current sources: contribute I to RHS
    // Convention: current flows from n_plus to n_minus (into n_plus, out of n_minus)
    // RHS injection: positive at n_plus (current enters), negative at n_minus (current leaves)
    for src in &mna.current_sources {
        if src.n_plus_idx > 0 {
            rhs[src.n_plus_idx - 1] += src.dc_value;
        }
        if src.n_minus_idx > 0 {
            rhs[src.n_minus_idx - 1] -= src.dc_value;
        }
    }

    // Voltage sources: use Norton equivalent (large conductance + current source)
    // For DC voltage V between n+ and n-, stamp G_large between nodes and I = V * G_large
    // The conductance is already stamped into G in the MNA builder below,
    // so here we just add the current contribution.
    for vs in &mna.voltage_sources {
        let current = vs.dc_value * VS_CONDUCTANCE;
        if vs.n_plus_idx > 0 {
            rhs[vs.n_plus_idx - 1] += current;
        }
        if vs.n_minus_idx > 0 {
            rhs[vs.n_minus_idx - 1] -= current;
        }
    }

    // Multiply by 2 for trapezoidal rule
    for i in 0..rhs.len() {
        rhs[i] *= 2.0;
    }

    rhs
}

/// Flatten a 2D matrix into a 1D row-major vector.
///
/// Index calculation: index = row * cols + col
fn flatten_matrix(matrix: &[Vec<f64>], rows: usize, cols: usize) -> Vec<f64> {
    if rows == 0 || cols == 0 {
        return Vec::new();
    }
    let mut flat = vec![0.0; rows * cols];
    for i in 0..rows {
        for j in 0..cols {
            flat[i * cols + j] = matrix[i][j];
        }
    }
    flat
}

/// Pivot threshold for detecting singular matrices in Gaussian elimination.
const SINGULARITY_THRESHOLD: f64 = 1e-15;

/// Matrix inversion using Gaussian elimination.
///
/// For small matrices (N ≤ 8), this is acceptable.
#[allow(clippy::needless_range_loop)]
pub(crate) fn invert_matrix(a: &[Vec<f64>]) -> Result<Vec<Vec<f64>>, String> {
    let n = a.len();
    if n == 0 {
        return Ok(Vec::new());
    }

    // Create augmented matrix [A | I]
    let mut aug = vec![vec![0.0; 2 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][n + i] = 1.0;
    }

    // Gaussian elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }

        if max_val < SINGULARITY_THRESHOLD {
            return Err("Matrix is singular or nearly singular".to_string());
        }

        // Swap rows
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Scale pivot row
        let pivot = aug[col][col];
        for j in col..(2 * n) {
            aug[col][j] /= pivot;
        }

        // Eliminate column
        for row in 0..n {
            if row != col {
                let factor = aug[row][col];
                for j in col..(2 * n) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }
    }

    // Extract inverse
    let mut inv = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            inv[i][j] = aug[i][n + j];
        }
    }

    Ok(inv)
}

/// Matrix multiplication: C = A * B
/// 
/// # Panics
/// Panics if either matrix is empty or if dimensions are incompatible.
#[allow(clippy::needless_range_loop)]
pub(crate) fn mat_mul(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    let m = a.len();
    let n = b[0].len();
    let p = b.len();

    let mut c = vec![vec![0.0; n]; m];
    for i in 0..m {
        for j in 0..n {
            let mut sum = 0.0;
            for k in 0..p {
                sum += a[i][k] * b[k][j];
            }
            c[i][j] = sum;
        }
    }
    c
}

/// Compute the infinity norm of a matrix: max over rows of the sum of absolute values.
///
/// This is used for condition number estimation: cond(A) ~ ||A||_inf * ||A^{-1}||_inf.
fn infinity_norm(a: &[Vec<f64>]) -> f64 {
    let mut max_row_sum = 0.0_f64;
    for row in a {
        let row_sum: f64 = row.iter().map(|x| x.abs()).sum();
        max_row_sum = max_row_sum.max(row_sum);
    }
    max_row_sum
}

/// Matrix-vector multiplication: y = A * x
#[allow(dead_code)]
#[allow(clippy::needless_range_loop)]
pub(crate) fn mat_vec_mul(a: &[Vec<f64>], x: &[f64]) -> Vec<f64> {
    let m = a.len();
    let mut y = vec![0.0; m];
    for i in 0..m {
        let mut sum = 0.0;
        for j in 0..x.len() {
            sum += a[i][j] * x[j];
        }
        y[i] = sum;
    }
    y
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mna::MnaSystem;
    use crate::parser::Netlist;

    #[test]
    fn test_matrix_invert() {
        let a = vec![
            vec![4.0, 7.0],
            vec![2.0, 6.0],
        ];
        let inv = invert_matrix(&a).unwrap();

        // A * A^-1 = I
        let product = mat_mul(&a, &inv);
        assert!((product[0][0] - 1.0).abs() < 1e-10);
        assert!((product[1][1] - 1.0).abs() < 1e-10);
        assert!(product[0][1].abs() < 1e-10);
        assert!(product[1][0].abs() < 1e-10);
    }

    #[test]
    fn test_flatten_matrix() {
        let matrix = vec![
            vec![1.0, 2.0, 3.0],
            vec![4.0, 5.0, 6.0],
        ];
        let flat = flatten_matrix(&matrix, 2, 3);
        assert_eq!(flat, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_dk_kernel_accessors() {
        let spice = r#"Diode Clipper
Rin in out 1k
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        // Test that accessor methods work correctly
        // K should be 1x1 for single diode
        assert_eq!(kernel.k.len(), 1);
        
        // Test accessor (only element is at [0][0])
        let k00 = kernel.k(0, 0);
        assert!(k00.is_finite());
        
        // Verify accessor matches direct index
        assert_eq!(kernel.k(0, 0), kernel.k[0]);
    }

    #[test]
    fn test_dk_kernel_from_rc() {
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        assert_eq!(kernel.n, 2);
        assert_eq!(kernel.m, 0); // No nonlinear devices
        assert_eq!(kernel.num_devices, 0);
        
        // For linear circuits, matrices should have correct dimensions
        assert_eq!(kernel.s.len(), 4); // n * n = 2 * 2
        assert_eq!(kernel.a_neg.len(), 4); // n * n = 2 * 2
    }

    #[test]
    fn test_dk_kernel_with_diode() {
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        assert_eq!(kernel.n, 2); // in, out
        assert_eq!(kernel.m, 1); // 1 diode = 1 dimension
        assert_eq!(kernel.num_devices, 1);

        // K should be 1x1 for single diode
        assert_eq!(kernel.k.len(), 1);
        
        // N_v should be 1x2 (M x N)
        assert_eq!(kernel.n_v.len(), 2); // m * n = 1 * 2
        
        // N_i should be 2x1 (N x M)
        assert_eq!(kernel.n_i.len(), 2); // n * m = 2 * 1
    }

    #[test]
    fn test_dk_kernel_with_bjt() {
        let spice = r#"Common Emitter
Q1 coll base emit 2N2222
Rc coll vcc 1k
R1 base 0 100k
Re emit 0 100
Rbias vcc 0 10k
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        assert_eq!(kernel.m, 2); // 1 BJT = 2 dimensions
        assert_eq!(kernel.num_devices, 1);

        // K should be 2x2
        assert_eq!(kernel.k.len(), 4); // m * m = 2 * 2
        
        // N_v should be 2xN (M x N)
        assert_eq!(kernel.n_v.len(), 2 * kernel.n); // m * n
        
        // N_i should be Nx2 (N x M)
        assert_eq!(kernel.n_i.len(), 2 * kernel.n); // n * m
    }

    #[test]
    fn test_predict_into() {
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let rhs = vec![1.0, 0.0];
        let mut v_pred = vec![0.0; 2];
        kernel.predict_into(&rhs, &mut v_pred);

        // Results should be finite
        assert!(v_pred[0].is_finite());
        assert!(v_pred[1].is_finite());
    }

    #[test]
    fn test_extract_prediction_into() {
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let v_pred = vec![1.0, 0.5];
        let mut p = vec![0.0; kernel.m];
        kernel.extract_prediction_into(&v_pred, &mut p);

        // Result should be finite
        assert!(p[0].is_finite());
    }

    #[test]
    fn test_kernel_contribution_into() {
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let i_nl = vec![1e-6];
        let mut k_contrib = vec![0.0; kernel.m];
        kernel.kernel_contribution_into(&i_nl, &mut k_contrib);

        // Result should be finite
        assert!(k_contrib[0].is_finite());
    }

    #[test]
    fn test_apply_correction_into() {
        let spice = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

        let v_pred = vec![1.0, 0.5];
        let i_nl = vec![1e-6];
        let i_nl_prev = vec![0.5e-6];
        let mut v = vec![0.0; kernel.n];
        let mut ni_inl = vec![0.0; kernel.n];

        kernel.apply_correction_into(&v_pred, &i_nl, &i_nl_prev, &mut v, &mut ni_inl);

        // Results should be finite
        assert!(v[0].is_finite());
        assert!(v[1].is_finite());
    }
}
