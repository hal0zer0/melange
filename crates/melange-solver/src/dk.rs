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

use crate::mna::{inject_rhs_current, invert_small_matrix, MnaSystem};
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

/// Information about a coupled inductor pair for companion model.
#[derive(Debug, Clone)]
pub struct CoupledInductorState {
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
    /// Previous current through L1
    pub i1_prev: f64,
    /// Previous current through L2
    pub i2_prev: f64,
    /// Previous voltage across L1
    pub v1_prev: f64,
    /// Previous voltage across L2
    pub v2_prev: f64,
    /// History current for L1 (Norton equivalent)
    pub i1_hist: f64,
    /// History current for L2 (Norton equivalent)
    pub i2_hist: f64,
}

/// Multi-winding transformer group state for NxN companion model.
#[derive(Debug, Clone)]
pub struct TransformerGroupState {
    pub name: String,
    pub num_windings: usize,
    pub winding_names: Vec<String>,
    pub winding_node_i: Vec<usize>,
    pub winding_node_j: Vec<usize>,
    pub inductances: Vec<f64>,
    pub coupling_matrix: Vec<Vec<f64>>,
    /// NxN admittance matrix Y = (T/2) * inv(L), stored flat row-major
    pub y_matrix: Vec<f64>,
    /// Previous current through each winding
    pub i_prev: Vec<f64>,
    /// Previous voltage across each winding
    pub v_prev: Vec<f64>,
    /// History current for each winding (Norton equivalent)
    pub i_hist: Vec<f64>,
    /// Pre-allocated scratch buffer for current voltages (real-time safe)
    pub v_new: Vec<f64>,
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
    /// Mirror of `PotInfo::runtime_field`. If Some, this pot was declared
    /// via `.runtime R` — emitter uses this identifier for the setter name
    /// and skips the DC-OP snap block.
    pub runtime_field: Option<String>,
}

/// DK kernel for a circuit.
///
/// This is the reduced representation used for efficient simulation.
/// All matrices are stored in flattened row-major format for better cache locality.
#[derive(Debug, Clone)]
pub struct DkKernel {
    /// System dimension = n_aug (n + num_vs + num_vcvs).
    /// This is the size of all N-indexed vectors and N×N matrices.
    pub n: usize,
    /// Original circuit node count (excluding ground and augmented VS/VCVS variables).
    /// Output extraction uses indices < n_nodes.
    pub n_nodes: usize,
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
    /// Inductor companion model info (uncoupled)
    pub inductors: Vec<InductorInfo>,
    /// Coupled inductor pair companion model info (2-winding)
    pub coupled_inductors: Vec<CoupledInductorState>,
    /// Multi-winding transformer group companion model info (3+ windings)
    pub transformer_groups: Vec<TransformerGroupState>,
    /// Potentiometer Sherman-Morrison precomputed data
    pub pots: Vec<SmPotData>,
    /// Wiper potentiometer groups (passthrough from MNA for codegen)
    pub wiper_groups: Vec<crate::mna::WiperGroupInfo>,
    /// Gang groups (passthrough from MNA for codegen)
    pub gang_groups: Vec<crate::mna::GangGroupInfo>,
}

/// Error type for DK reduction.
#[derive(Debug, Clone)]
#[non_exhaustive]
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
    /// An upstream MNA error.
    Mna(crate::mna::MnaError),
}

impl std::fmt::Display for DkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DkError::SingularMatrix(msg) => write!(f, "DK error: {}", msg),
            DkError::InvalidParameter(msg) => write!(f, "DK error: {}", msg),
            DkError::InvalidInductance { name, value } => {
                write!(
                    f,
                    "DK error: inductor '{}' has non-positive inductance: {}",
                    name, value
                )
            }
            DkError::InvalidKDiagonal { index, value } => {
                write!(
                    f,
                    "DK error: K diagonal [{}][{}] = {} is non-negative (expected < 0 for stable feedback)",
                    index, index, value
                )
            }
            DkError::Mna(e) => write!(f, "DK error: {}", e),
        }
    }
}

impl std::error::Error for DkError {}

impl From<crate::mna::MnaError> for DkError {
    fn from(e: crate::mna::MnaError) -> Self {
        DkError::Mna(e)
    }
}

/// Maximum supported nonlinear dimension (sum of all device dimensions).
///
/// Circuits with more than MAX_M nonlinear dimensions are rejected to prevent
/// unbounded allocation and O(M^3) NR solve cost.
///
/// M=24 supports the heaviest circuits in the stable catalog: Uniquorn v2's
/// 8-stage active-NR cascade + CF buffer + clipper (M=20) with headroom for
/// v3's additions and future split-band saturation designs. Circuits at
/// M≥10 typically route to the nodal full-LU path via the `|K|>1e8` /
/// `ρ(S·A_neg)>0.999` / positive-K-diagonal guards — MAX_M bounds the
/// kernel-build hard reject, not which solver path actually runs.
pub const MAX_M: usize = 24;

/// Maximum supported system dimension (total rows/columns in the A/S matrices).
///
/// Includes circuit nodes, voltage source branch currents, VCVS augmented rows,
/// and inductor branch variables. Prevents O(N^3) blowup from matrix inversion
/// on pathologically large circuits.
///
/// N=256 is generous for any real audio circuit (Pultec EQP-1A is ~41 nodes).
pub const MAX_N: usize = 256;

impl DkKernel {
    /// Build DK kernel from MNA system.
    ///
    /// # Errors
    /// Returns an error if `sample_rate` is not positive and finite, if the
    /// A matrix is singular, if any inductor has non-positive inductance,
    /// or if the total nonlinear dimension exceeds [`MAX_M`].
    pub fn from_mna(mna: &MnaSystem, sample_rate: f64) -> Result<Self, DkError> {
        if !(sample_rate > 0.0 && sample_rate.is_finite()) {
            return Err(DkError::InvalidParameter(format!(
                "sample_rate must be positive and finite, got {}",
                sample_rate
            )));
        }

        // Auto-insert parasitic caps if C matrix is all zeros and circuit has
        // nonlinear devices. Without capacitors, A = G and the trapezoidal
        // integrator degenerates (no energy storage → no dynamics).
        let patched_mna;
        let mna = if mna.m > 0 && !mna.c.iter().any(|row| row.iter().any(|&v| v != 0.0)) {
            log::info!(
                "C matrix is all zeros with M={} nonlinear devices; auto-inserting parasitic caps",
                mna.m
            );
            patched_mna = {
                let mut m = mna.clone();
                m.add_parasitic_caps();
                m
            };
            &patched_mna
        } else {
            mna
        };

        let n_nodes = mna.n;
        let n = mna.n_aug; // augmented system dimension
        let m = mna.m;

        if m > MAX_M {
            return Err(DkError::InvalidParameter(format!(
                "nonlinear dimension m={} exceeds MAX_M={}",
                m, MAX_M
            )));
        }

        if n > MAX_N {
            return Err(DkError::InvalidParameter(format!(
                "System dimension {} exceeds MAX_N={} (too many nodes)",
                n, MAX_N
            )));
        }

        // Get A matrix (n_aug × n_aug)
        let a = mna.get_a_matrix(sample_rate)?;

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
                log::warn!(
                    "A matrix condition number estimate is {:.2e} (threshold 1e12). \
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
        // N_v is M × n_aug and N_i is n_aug × M; augmented entries are zero,
        // so K = N_v * S * N_i reduces to the same result as before.
        //
        // First: S * N_i (n_aug x M)
        let s_ni = mat_mul(&s_2d, &mna.n_i);
        // Then: N_v * (S * N_i) (M x M)
        let k_2d = mat_mul(&mna.n_v, &s_ni);

        // Validate K diagonal: K[i][i] must be negative for stable NR feedback.
        // A non-negative diagonal indicates incorrect circuit topology or wiring.
        // Exception: dimensions where N_i column is all zeros (e.g., VCA control port,
        // MOSFET insulated gate) have K[i][i] = 0 by construction — no current feedback.
        // These dimensions are valid; the NR system has an identity row there.
        for (i, row) in k_2d.iter().enumerate() {
            if row[i] >= 0.0 {
                // Check if this dimension has zero N_i column (no current injection)
                let ni_col_all_zero = mna.n_i.iter().all(|ni_row| ni_row[i].abs() < 1e-30);
                if ni_col_all_zero {
                    log::debug!(
                        "K[{}][{}] = {} (zero N_i column — no current feedback, OK)",
                        i,
                        i,
                        row[i]
                    );
                } else {
                    return Err(DkError::InvalidKDiagonal {
                        index: i,
                        value: row[i],
                    });
                }
            }
        }

        let k = flatten_matrix(&k_2d, m, m);

        // Get A_neg matrix for history (augmented rows are all zeros — algebraic constraints)
        let a_neg_2d = mna.get_a_neg_matrix(sample_rate)?;
        let a_neg = flatten_matrix(&a_neg_2d, n, n);

        // Build constant sources from voltage and current sources
        let rhs_const = build_rhs_const(mna);

        // Precompute Sherman-Morrison vectors for pots.
        // u is n_aug-sized; pot node indices are < n_nodes < n, so they fit.
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
            let usu: f64 = u.iter().zip(su.iter()).map(|(a, b)| a * b).sum();

            // nv_su = N_v * su (M-vector)
            let nv_su = if m > 0 {
                mat_vec_mul(&mna.n_v, &su)
            } else {
                Vec::new()
            };

            // u_ni[j] = su^T * N_i[:,j] = (S*u)^T * N_i[:,j] (M-vector)
            // Sherman-Morrison requires su^T*N_i (not u^T*N_i) because
            // S' = S - scale * (S*u) * (u^T*S) and S is symmetric, so u^T*S = su^T
            let u_ni: Vec<f64> = (0..m)
                .map(|j| su.iter().enumerate().map(|(i, &s)| s * mna.n_i[i][j]).sum())
                .collect();

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
                runtime_field: pot_info.runtime_field.clone(),
            });
        }

        // Flatten N_v (M × n_aug) and N_i (n_aug × M) from MNA
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
            let g_eq = t / (2.0 * ind.value); // T/(2L)
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

        // Initialize coupled inductor companion models
        let mut coupled_inductors = Vec::with_capacity(mna.coupled_inductors.len());
        for ci in &mna.coupled_inductors {
            if ci.l1_value <= 0.0 {
                return Err(DkError::InvalidInductance {
                    name: ci.l1_name.clone(),
                    value: ci.l1_value,
                });
            }
            if ci.l2_value <= 0.0 {
                return Err(DkError::InvalidInductance {
                    name: ci.l2_name.clone(),
                    value: ci.l2_value,
                });
            }
            let m_val = ci.coupling * (ci.l1_value * ci.l2_value).sqrt();
            let det = ci.l1_value * ci.l2_value - m_val * m_val;
            if det <= 0.0 {
                return Err(DkError::InvalidParameter(format!(
                    "Coupled inductors '{}'-'{}': det = L1*L2 - M² = {:.6e} <= 0 (k={} too close to 1)",
                    ci.l1_name, ci.l2_name, det, ci.coupling
                )));
            }
            let half_t = t / 2.0;
            let g_self_1 = half_t * ci.l2_value / det;
            let g_self_2 = half_t * ci.l1_value / det;
            let g_mutual = -half_t * m_val / det;

            coupled_inductors.push(CoupledInductorState {
                name: ci.name.clone(),
                l1_name: ci.l1_name.clone(),
                l2_name: ci.l2_name.clone(),
                l1_node_i: ci.l1_node_i,
                l1_node_j: ci.l1_node_j,
                l2_node_i: ci.l2_node_i,
                l2_node_j: ci.l2_node_j,
                l1_inductance: ci.l1_value,
                l2_inductance: ci.l2_value,
                coupling: ci.coupling,
                g_self_1,
                g_self_2,
                g_mutual,
                i1_prev: 0.0,
                i2_prev: 0.0,
                v1_prev: 0.0,
                v2_prev: 0.0,
                i1_hist: 0.0,
                i2_hist: 0.0,
            });
        }

        // Initialize multi-winding transformer groups
        let mut transformer_groups = Vec::with_capacity(mna.transformer_groups.len());
        for group in &mna.transformer_groups {
            let w = group.num_windings;
            for (idx, l) in group.inductances.iter().enumerate() {
                if *l <= 0.0 {
                    return Err(DkError::InvalidInductance {
                        name: group.winding_names[idx].clone(),
                        value: *l,
                    });
                }
            }
            // Build NxN inductance matrix
            let mut l_mat = vec![vec![0.0f64; w]; w];
            for i in 0..w {
                for j in 0..w {
                    l_mat[i][j] = group.coupling_matrix[i][j]
                        * (group.inductances[i] * group.inductances[j]).sqrt();
                }
            }
            // Compute Y = (T/2) * inv(L)
            let y_raw = invert_small_matrix(&l_mat);
            let half_t = t / 2.0;
            let mut y_flat = vec![0.0f64; w * w];
            for i in 0..w {
                for j in 0..w {
                    y_flat[i * w + j] = half_t * y_raw[i][j];
                }
            }
            transformer_groups.push(TransformerGroupState {
                name: group.name.clone(),
                num_windings: w,
                winding_names: group.winding_names.clone(),
                winding_node_i: group.winding_node_i.clone(),
                winding_node_j: group.winding_node_j.clone(),
                inductances: group.inductances.clone(),
                coupling_matrix: group.coupling_matrix.clone(),
                y_matrix: y_flat,
                i_prev: vec![0.0; w],
                v_prev: vec![0.0; w],
                i_hist: vec![0.0; w],
                v_new: vec![0.0; w],
            });
        }

        Ok(Self {
            n,       // = n_aug (system dimension including VS/VCVS variables)
            n_nodes, // = mna.n (original circuit node count)
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
            coupled_inductors,
            transformer_groups,
            pots,
            wiper_groups: mna.wiper_groups.clone(),
            gang_groups: mna.gang_groups.clone(),
        })
    }

    /// Build DK kernel from MNA using augmented MNA for inductors.
    ///
    /// Each inductor winding adds an extra variable (branch current j_L) with
    /// inductance L in the C matrix. This gives A[k][k] = 2L/T (well-conditioned)
    /// instead of the companion model's T/(2L) (ill-conditioned for large L).
    ///
    /// The resulting kernel has `n = n_nodal = n_aug + total_inductor_windings`.
    /// S, A_neg, N_v, N_i are all at the expanded dimension. K stays M×M.
    /// The companion model vectors (inductors, coupled_inductors, transformer_groups)
    /// are left empty — A_neg handles all trapezoidal inductor history automatically.
    ///
    /// Use this for circuits with large inductors (transformers) and nonlinear devices.
    pub fn from_mna_augmented(mna: &MnaSystem, sample_rate: f64) -> Result<Self, DkError> {
        if !(sample_rate > 0.0 && sample_rate.is_finite()) {
            return Err(DkError::InvalidParameter(format!(
                "sample_rate must be positive and finite, got {}",
                sample_rate
            )));
        }

        // Auto-insert parasitic caps if C matrix is all zeros and circuit has
        // nonlinear devices. Without capacitors, A = G and the trapezoidal
        // integrator degenerates (no energy storage → no dynamics).
        let patched_mna;
        let mna = if mna.m > 0 && !mna.c.iter().any(|row| row.iter().any(|&v| v != 0.0)) {
            log::info!(
                "C matrix is all zeros with M={} nonlinear devices; auto-inserting parasitic caps",
                mna.m
            );
            patched_mna = {
                let mut m = mna.clone();
                m.add_parasitic_caps();
                m
            };
            &patched_mna
        } else {
            mna
        };

        let n_nodes = mna.n;
        let n_aug = mna.n_aug;
        let m = mna.m;

        if m > MAX_M {
            return Err(DkError::InvalidParameter(format!(
                "nonlinear dimension m={} exceeds MAX_M={}",
                m, MAX_M
            )));
        }

        // Count inductor winding variables
        let n_uncoupled = mna.inductors.len();
        let n_coupled_windings: usize = mna.coupled_inductors.len() * 2;
        let n_xfmr_windings: usize = mna.transformer_groups.iter().map(|g| g.num_windings).sum();
        let n_inductor_vars = n_uncoupled + n_coupled_windings + n_xfmr_windings;
        let n = n_aug + n_inductor_vars; // n_nodal

        if n > MAX_N {
            return Err(DkError::InvalidParameter(format!(
                "System dimension {} exceeds MAX_N={} (too many nodes)",
                n, MAX_N
            )));
        }

        let t = 1.0 / sample_rate;
        let alpha = 2.0 / t;

        // Validate inductor values
        for ind in &mna.inductors {
            if ind.value <= 0.0 {
                return Err(DkError::InvalidInductance {
                    name: ind.name.clone(),
                    value: ind.value,
                });
            }
        }
        for ci in &mna.coupled_inductors {
            if ci.l1_value <= 0.0 {
                return Err(DkError::InvalidInductance {
                    name: ci.l1_name.clone(),
                    value: ci.l1_value,
                });
            }
            if ci.l2_value <= 0.0 {
                return Err(DkError::InvalidInductance {
                    name: ci.l2_name.clone(),
                    value: ci.l2_value,
                });
            }
        }
        for group in &mna.transformer_groups {
            for (idx, &l) in group.inductances.iter().enumerate() {
                if l <= 0.0 {
                    return Err(DkError::InvalidInductance {
                        name: group.winding_names[idx].clone(),
                        value: l,
                    });
                }
            }
        }

        // Build augmented G/C matrices using shared method
        let aug = mna.build_augmented_matrices();
        let g_nod = aug.g;
        let c_nod = aug.c;

        // Build A = G + (2/T)*C
        let mut a = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in 0..n {
                a[i][j] = g_nod[i][j] + alpha * c_nod[i][j];
            }
        }

        // Invert A to get S
        let s_2d = invert_matrix(&a)
            .map_err(|e| DkError::SingularMatrix(format!("Failed to invert augmented A: {}", e)))?;

        {
            let norm_a = infinity_norm(&a);
            let norm_s = infinity_norm(&s_2d);
            let cond = norm_a * norm_s;
            if cond > 1e12 {
                log::warn!(
                    "Augmented A matrix condition number estimate is {:.2e} (threshold 1e12).",
                    cond
                );
            }
        }

        let s = flatten_matrix(&s_2d, n, n);

        // Expand N_v (m × n_aug → m × n_nodal) and N_i (n_aug × m → n_nodal × m)
        let mut n_v_2d = vec![vec![0.0; n]; m];
        for i in 0..m {
            for j in 0..n_aug {
                n_v_2d[i][j] = mna.n_v[i][j];
            }
        }
        let mut n_i_2d = vec![vec![0.0; m]; n];
        for i in 0..n_aug {
            for j in 0..m {
                n_i_2d[i][j] = mna.n_i[i][j];
            }
        }

        // Compute K = N_v * S * N_i (M × M)
        let s_ni = mat_mul(&s_2d, &n_i_2d);
        let k_2d = mat_mul(&n_v_2d, &s_ni);

        for (i, row) in k_2d.iter().enumerate() {
            if row[i] >= 0.0 {
                // For the augmented MNA / nodal solver path, the K matrix is not used
                // for NR iteration — the nodal solver does full Newton-Raphson on the
                // augmented system directly. A non-negative K diagonal may occur with
                // feedback windings (e.g., push-pull output transformer tertiary) and
                // does not indicate an unstable circuit. Downgrade to warning.
                log::warn!(
                    "K diagonal [{}][{}] = {} is non-negative. \
                     This is expected for circuits with transformer-coupled NFB. \
                     The nodal solver handles this correctly via full NR.",
                    i,
                    i,
                    row[i]
                );
            }
        }

        let k = flatten_matrix(&k_2d, m, m);
        let n_v = flatten_matrix(&n_v_2d, m, n);
        let n_i = flatten_matrix(&n_i_2d, n, m);

        // Build A_neg = (2/T)*C - G, zeroing VS/VCVS/ideal-transformer rows only.
        // Internal BJT nodes and inductor branch rows are NOT zeroed (they need history).
        let mut a_neg_2d = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in 0..n {
                a_neg_2d[i][j] = alpha * c_nod[i][j] - g_nod[i][j];
            }
        }
        // Zero algebraic constraint rows (VS, VCVS, ideal transformers)
        for vs in &mna.voltage_sources {
            let row = n_nodes + vs.ext_idx;
            if row < n {
                for j in 0..n {
                    a_neg_2d[row][j] = 0.0;
                }
            }
        }
        let num_vs = mna.voltage_sources.len();
        for (vcvs_idx, _) in mna.vcvs_sources.iter().enumerate() {
            let row = n_nodes + num_vs + vcvs_idx;
            if row < n {
                for j in 0..n {
                    a_neg_2d[row][j] = 0.0;
                }
            }
        }
        let num_vcvs = mna.vcvs_sources.len();
        for (xfmr_idx, _) in mna.ideal_transformers.iter().enumerate() {
            let row = n_nodes + num_vs + num_vcvs + xfmr_idx;
            if row < n {
                for j in 0..n {
                    a_neg_2d[row][j] = 0.0;
                }
            }
        }
        let a_neg = flatten_matrix(&a_neg_2d, n, n);

        // Build rhs_const: reuse existing build_rhs_const (n_aug-sized), pad with zeros
        let rhs_const_base = build_rhs_const(mna);
        let mut rhs_const = vec![0.0; n];
        for i in 0..n_aug {
            rhs_const[i] = rhs_const_base[i];
        }

        // Precompute Sherman-Morrison pot vectors at the expanded dimension
        let mut pots = Vec::with_capacity(mna.pots.len());
        for pot_info in &mna.pots {
            let mut u = vec![0.0; n];
            if pot_info.node_p > 0 {
                u[pot_info.node_p - 1] = 1.0;
            }
            if pot_info.node_q > 0 {
                u[pot_info.node_q - 1] = -1.0;
            }
            let su = mat_vec_mul(&s_2d, &u);
            let usu: f64 = u.iter().zip(su.iter()).map(|(a, b)| a * b).sum();
            let nv_su = if m > 0 {
                mat_vec_mul(&n_v_2d, &su)
            } else {
                Vec::new()
            };
            let u_ni: Vec<f64> = (0..m)
                .map(|j| su.iter().enumerate().map(|(i, &s)| s * n_i_2d[i][j]).sum())
                .collect();
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
                runtime_field: pot_info.runtime_field.clone(),
            });
        }

        if n_inductor_vars > 0 {
            log::info!(
                "DK kernel (augmented): n_nodal={} (n_aug={} + {} inductor vars), M={}",
                n,
                n_aug,
                n_inductor_vars,
                m
            );
        }

        Ok(Self {
            n,
            n_nodes,
            m,
            num_devices: mna.num_devices,
            sample_rate,
            s,
            k,
            n_v,
            n_i,
            a_neg,
            rhs_const,
            inductors: Vec::new(), // empty — augmented MNA handles history
            coupled_inductors: Vec::new(), // empty
            transformer_groups: Vec::new(), // empty
            pots,
            wiper_groups: mna.wiper_groups.clone(),
            gang_groups: mna.gang_groups.clone(),
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

    /// Apply correction: v = v_pred + S * N_i * i_nl
    ///
    /// This computes the final node voltages after solving for i_nl.
    /// Uses full i_nl (not delta) to implement trapezoidal nonlinear integration:
    /// since v_pred already includes N_i * i_nl_prev from the RHS, the net result is
    /// v = S * (rhs_base + N_i * i_nl_prev + N_i * i_nl) = S * (... + N_i * (i_nl + i_nl_prev)),
    /// which is the proper trapezoidal average of nonlinear currents.
    ///
    /// Writes result into the provided output buffer.
    ///
    /// # Arguments
    /// - `v_pred`: Prediction vector (n elements)
    /// - `i_nl`: New nonlinear currents (m elements)
    /// - `i_nl_prev`: Previous nonlinear currents (m elements, unused — kept for API compatibility)
    /// - `v`: Output buffer for result (n elements)
    /// - `ni_inl`: Temporary buffer for N_i * i_nl (n elements, pre-allocated)
    #[allow(clippy::needless_range_loop)]
    pub fn apply_correction_into(
        &self,
        v_pred: &[f64],
        i_nl: &[f64],
        _i_nl_prev: &[f64],
        v: &mut [f64],
        ni_inl: &mut [f64],
    ) {
        assert_eq!(v_pred.len(), self.n);
        assert_eq!(i_nl.len(), self.m);
        assert_eq!(v.len(), self.n);
        assert_eq!(ni_inl.len(), self.n);

        // v = v_pred + S * N_i * i_nl
        // First compute N_i * i_nl (N vector)
        for i in 0..self.n {
            let mut sum = 0.0;
            let row_offset = i * self.m;
            for j in 0..self.m {
                sum += self.n_i[row_offset + j] * i_nl[j];
            }
            ni_inl[i] = sum;
        }

        // Then S * (N_i * i_nl)
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
            let v_i = if ind.node_i > 0 {
                v_node[ind.node_i - 1]
            } else {
                0.0
            };
            let v_j = if ind.node_j > 0 {
                v_node[ind.node_j - 1]
            } else {
                0.0
            };
            let v_l_new = v_i - v_j;

            let g_eq = ind.g_eq; // T/(2L)

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

    /// Update coupled inductor state after solving.
    ///
    /// Uses trapezoidal companion model for coupled inductors:
    ///   i1_new = i1_prev + g_self_1*(v1_prev + v1_new) + g_mutual*(v2_prev + v2_new)
    ///   i2_new = i2_prev + g_mutual*(v1_prev + v1_new) + g_self_2*(v2_prev + v2_new)
    ///   i1_hist = i1_new - g_self_1*v1_new - g_mutual*v2_new
    ///   i2_hist = i2_new - g_mutual*v1_new - g_self_2*v2_new
    pub fn update_coupled_inductors(&mut self, v_node: &[f64]) {
        for ci in &mut self.coupled_inductors {
            let v1i = if ci.l1_node_i > 0 {
                v_node[ci.l1_node_i - 1]
            } else {
                0.0
            };
            let v1j = if ci.l1_node_j > 0 {
                v_node[ci.l1_node_j - 1]
            } else {
                0.0
            };
            let v1_new = v1i - v1j;

            let v2i = if ci.l2_node_i > 0 {
                v_node[ci.l2_node_i - 1]
            } else {
                0.0
            };
            let v2j = if ci.l2_node_j > 0 {
                v_node[ci.l2_node_j - 1]
            } else {
                0.0
            };
            let v2_new = v2i - v2j;

            let i1_new = ci.i1_prev
                + ci.g_self_1 * (ci.v1_prev + v1_new)
                + ci.g_mutual * (ci.v2_prev + v2_new);
            let i2_new = ci.i2_prev
                + ci.g_mutual * (ci.v1_prev + v1_new)
                + ci.g_self_2 * (ci.v2_prev + v2_new);

            ci.i1_hist = i1_new - ci.g_self_1 * v1_new - ci.g_mutual * v2_new;
            ci.i2_hist = i2_new - ci.g_mutual * v1_new - ci.g_self_2 * v2_new;

            ci.i1_prev = i1_new;
            ci.i2_prev = i2_new;
            ci.v1_prev = v1_new;
            ci.v2_prev = v2_new;
        }
    }

    /// Update multi-winding transformer group companion model state.
    ///
    /// For each group with W windings, uses the NxN admittance matrix Y:
    ///   i_k_new = i_k_prev + sum_j Y[k][j] * (v_j_prev + v_j_new)
    ///   i_k_hist = i_k_new - sum_j Y[k][j] * v_j_new
    pub fn update_transformer_groups(&mut self, v_node: &[f64]) {
        for group in &mut self.transformer_groups {
            let w = group.num_windings;
            // Extract current voltages across each winding (pre-allocated buffer)
            for k in 0..w {
                let vi = if group.winding_node_i[k] > 0 {
                    v_node[group.winding_node_i[k] - 1]
                } else {
                    0.0
                };
                let vj = if group.winding_node_j[k] > 0 {
                    v_node[group.winding_node_j[k] - 1]
                } else {
                    0.0
                };
                group.v_new[k] = vi - vj;
            }
            // Compute new currents and history
            for k in 0..w {
                let mut i_new = group.i_prev[k];
                let mut y_v_new = 0.0f64;
                for j in 0..w {
                    let y_kj = group.y_matrix[k * w + j];
                    i_new += y_kj * (group.v_prev[j] + group.v_new[j]);
                    y_v_new += y_kj * group.v_new[j];
                }
                group.i_hist[k] = i_new - y_v_new;
                group.i_prev[k] = i_new;
            }
            // Update previous voltages
            for k in 0..w {
                group.v_prev[k] = group.v_new[k];
            }
        }
    }
}

/// Build constant RHS vector from DC sources for augmented MNA.
///
/// For trapezoidal rule:
/// - Current source rows (indices 0..n-1): multiply by 2 (proper trapezoidal average).
/// - Voltage source augmented rows (indices n..n+num_vs-1): set to V_dc (NOT multiplied
///   by 2, because these are algebraic constraints, not differential equations).
/// - VCVS augmented rows: 0 (homogeneous constraint).
pub(crate) fn build_rhs_const(mna: &MnaSystem) -> Vec<f64> {
    let n = mna.n;
    let n_aug = mna.n_aug;
    let mut rhs = vec![0.0; n_aug];

    // Current sources: inject into node rows (0..n-1)
    for src in &mna.current_sources {
        inject_rhs_current(&mut rhs, src.n_plus_idx, src.dc_value);
        inject_rhs_current(&mut rhs, src.n_minus_idx, -src.dc_value);
    }

    // Multiply the node rows by 2 for trapezoidal rule.
    // Augmented rows (n..n_aug-1) are NOT multiplied — algebraic constraints have no history.
    for val in rhs[..n].iter_mut() {
        *val *= 2.0;
    }

    // Voltage sources: set augmented row to V_dc (KVL constraint: V+ - V- = V_dc).
    // This is NOT multiplied by 2 — it's an algebraic constraint, not a trapezoidal term.
    for vs in &mna.voltage_sources {
        let k = n + vs.ext_idx;
        rhs[k] = vs.dc_value;
    }

    // VCVS augmented rows are 0 (homogeneous KVL: V_out+ - V_out- - gain*(V_ctrl+ - V_ctrl-) = 0).
    // Already zero from initialization.

    rhs
}

/// Solve A * x = b using equilibrated LU decomposition with iterative refinement.
///
/// Same numerical approach as `invert_matrix` but solves a single system instead
/// of computing the full inverse. Used by the NodalSolver for per-iteration Jacobian
/// solves where cond(A) can be ~1e9 (too ill-conditioned for basic LU).
#[allow(clippy::needless_range_loop)]
pub fn solve_equilibrated(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = a.len();
    if n == 0 {
        return Some(Vec::new());
    }

    // Step 1: Equilibrate
    let mut d = vec![1.0; n];
    for i in 0..n {
        let diag = a[i][i].abs();
        if diag > SINGULARITY_THRESHOLD {
            d[i] = 1.0 / diag.sqrt();
        }
    }

    let mut a_eq = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            a_eq[i][j] = d[i] * a[i][j] * d[j];
        }
    }

    // Step 2: LU factorize with partial pivoting
    let mut perm = vec![0usize; n];
    for i in 0..n {
        perm[i] = i;
    }

    for col in 0..n {
        let mut max_row = col;
        let mut max_val = a_eq[col][col].abs();
        for row in (col + 1)..n {
            if a_eq[row][col].abs() > max_val {
                max_val = a_eq[row][col].abs();
                max_row = row;
            }
        }
        if max_val < SINGULARITY_THRESHOLD {
            return None; // Singular
        }
        if max_row != col {
            a_eq.swap(col, max_row);
            perm.swap(col, max_row);
        }
        let pivot = a_eq[col][col];
        for row in (col + 1)..n {
            let factor = a_eq[row][col] / pivot;
            a_eq[row][col] = factor;
            for j in (col + 1)..n {
                a_eq[row][j] -= factor * a_eq[col][j];
            }
        }
    }

    // Step 3: Solve LU * x_eq = D * P * b
    // Build permuted, scaled RHS
    let mut x = vec![0.0; n];
    for i in 0..n {
        x[i] = d[perm[i]] * b[perm[i]];
    }

    // Forward substitution (L * y = rhs)
    for i in 1..n {
        let mut sum = 0.0;
        for j in 0..i {
            sum += a_eq[i][j] * x[j];
        }
        x[i] -= sum;
    }

    // Backward substitution (U * x_eq = y)
    for i in (0..n).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..n {
            sum += a_eq[i][j] * x[j];
        }
        x[i] = (x[i] - sum) / a_eq[i][i];
    }

    // Step 4: Iterative refinement
    // Compute residual in permuted equilibrated space: r = D*P*b - (P*A_eq)*x
    let mut r = vec![0.0; n];
    for i in 0..n {
        let pi = perm[i];
        let mut ax_i = 0.0;
        for j in 0..n {
            ax_i += d[pi] * a[pi][j] * d[j] * x[j];
        }
        r[i] = d[pi] * b[pi] - ax_i;
    }

    // Solve LU * dx = r
    for i in 1..n {
        let mut sum = 0.0;
        for j in 0..i {
            sum += a_eq[i][j] * r[j];
        }
        r[i] -= sum;
    }
    for i in (0..n).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..n {
            sum += a_eq[i][j] * r[j];
        }
        r[i] = (r[i] - sum) / a_eq[i][i];
    }

    // Apply correction and undo equilibration: x_final = D * (x_eq + dx)
    for i in 0..n {
        x[i] = d[i] * (x[i] + r[i]);
    }

    Some(x)
}

/// Flatten a 2D matrix into a 1D row-major vector.
///
/// Index calculation: index = row * cols + col
pub(crate) fn flatten_matrix(matrix: &[Vec<f64>], rows: usize, cols: usize) -> Vec<f64> {
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

/// Matrix inversion using equilibrated LU decomposition with iterative refinement.
///
/// Steps:
/// 1. Equilibrate: D * A * D where D = diag(1/sqrt(|A[i][i]|)) to normalize entries.
/// 2. LU factorize with partial pivoting.
/// 3. Solve each column of A^{-1} via forward/backward substitution.
/// 4. One round of iterative refinement per column for extra accuracy.
/// 5. Undo equilibration: S = D * S_eq * D.
///
/// This gives ~6 additional correct digits vs naive Gauss-Jordan for ill-conditioned
/// matrices (cond > 1e9), which is critical for circuits with large inductors.
#[allow(clippy::needless_range_loop)]
pub(crate) fn invert_matrix(a: &[Vec<f64>]) -> Result<Vec<Vec<f64>>, String> {
    let n = a.len();
    if n == 0 {
        return Ok(Vec::new());
    }

    // Step 1: Compute symmetric equilibration D = diag(1/sqrt(|A[i][i]|))
    let mut d = vec![1.0; n];
    for i in 0..n {
        let diag = a[i][i].abs();
        if diag > SINGULARITY_THRESHOLD {
            d[i] = 1.0 / diag.sqrt();
        }
    }

    // Build equilibrated matrix A_eq = D * A * D
    let mut a_eq = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            a_eq[i][j] = d[i] * a[i][j] * d[j];
        }
    }

    // Step 2: LU factorize with partial pivoting (in-place)
    // perm[k] = row that was pivoted into position k
    let mut perm = vec![0usize; n];
    for i in 0..n {
        perm[i] = i;
    }

    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = a_eq[col][col].abs();
        for row in (col + 1)..n {
            if a_eq[row][col].abs() > max_val {
                max_val = a_eq[row][col].abs();
                max_row = row;
            }
        }

        if max_val < SINGULARITY_THRESHOLD {
            return Err(format!(
                "Matrix is singular or nearly singular (pivot {:.2e} at col {})",
                max_val, col
            ));
        }

        if max_row != col {
            a_eq.swap(col, max_row);
            perm.swap(col, max_row);
        }

        // Compute multipliers and eliminate below
        let pivot = a_eq[col][col];
        for row in (col + 1)..n {
            let factor = a_eq[row][col] / pivot;
            a_eq[row][col] = factor; // Store L factor in lower triangle
            for j in (col + 1)..n {
                a_eq[row][j] -= factor * a_eq[col][j];
            }
        }
    }

    // Step 3: Solve each column via forward/backward substitution
    let mut s_eq = vec![vec![0.0; n]; n];

    for col in 0..n {
        // Build permuted RHS (identity column)
        let mut b = vec![0.0; n];
        for i in 0..n {
            b[i] = if perm[i] == col { 1.0 } else { 0.0 };
        }

        // Forward substitution (L * y = Pb)
        for i in 1..n {
            let mut sum = 0.0;
            for j in 0..i {
                sum += a_eq[i][j] * b[j];
            }
            b[i] -= sum;
        }

        // Backward substitution (U * x = y)
        for i in (0..n).rev() {
            let mut sum = 0.0;
            for j in (i + 1)..n {
                sum += a_eq[i][j] * b[j];
            }
            b[i] = (b[i] - sum) / a_eq[i][i];
        }

        // Step 4: One round of iterative refinement
        // Compute residual r = P*e_col - (P*A_eq)*x in the permuted system.
        // A_eq was overwritten by LU, so recompute (P*A_eq)*x from original `a`:
        //   (P*A_eq)[i][j] = A_eq[perm[i]][j] = d[perm[i]] * a[perm[i]][j] * d[j]
        let mut r = vec![0.0; n];
        for i in 0..n {
            let pi = perm[i];
            let mut ax_i = 0.0;
            for j in 0..n {
                ax_i += d[pi] * a[pi][j] * d[j] * b[j];
            }
            r[i] = (if pi == col { 1.0 } else { 0.0 }) - ax_i;
        }

        // Solve LU * dx = r (same factored system)
        // Forward substitution
        for i in 1..n {
            let mut sum = 0.0;
            for j in 0..i {
                sum += a_eq[i][j] * r[j];
            }
            r[i] -= sum;
        }
        // Backward substitution
        for i in (0..n).rev() {
            let mut sum = 0.0;
            for j in (i + 1)..n {
                sum += a_eq[i][j] * r[j];
            }
            r[i] = (r[i] - sum) / a_eq[i][i];
        }

        // Apply correction
        for i in 0..n {
            b[i] += r[i];
        }

        for i in 0..n {
            s_eq[i][col] = b[i];
        }
    }

    // Step 5: Undo equilibration: S = D * S_eq * D
    let mut inv = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            inv[i][j] = d[i] * s_eq[i][j] * d[j];
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
        let a = vec![vec![4.0, 7.0], vec![2.0, 6.0]];
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
        let matrix = vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]];
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
