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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SolverMode {
    /// DK method: precompute S=A⁻¹, NR in M-dimensional current space.
    /// Fast (O(N²+M³) per sample) but requires well-conditioned A and K[i][i]<0.
    Dk,
    /// Full-nodal NR: LU solve in N-dimensional voltage space per NR iteration.
    /// Handles any circuit topology including transformer-coupled NFB.
    /// Slower (O(N³) per sample) but universally convergent.
    Nodal,
}

impl Default for SolverMode {
    fn default() -> Self { SolverMode::Dk }
}

/// Language-agnostic intermediate representation of a compiled circuit.
///
/// Built from `DkKernel` + `MnaSystem` + `Netlist` + `CodegenConfig`, this
/// struct contains every piece of data an emitter needs — matrices, device
/// parameters, solver config — without referencing the builder types.
#[derive(Debug, Clone, Serialize, Deserialize)]
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
    pub inductors: Vec<InductorIR>,
    pub coupled_inductors: Vec<CoupledInductorIR>,
    pub transformer_groups: Vec<TransformerGroupIR>,
    pub pots: Vec<PotentiometerIR>,
    pub switches: Vec<SwitchIR>,
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
    /// When augmented_inductors is true, A_neg rows n_nodes..n_aug should be zeroed,
    /// but inductor rows n_aug..n should NOT be zeroed.
    #[serde(default)]
    pub n_aug: usize,
    /// True when inductors use augmented MNA (branch current variables in G/C)
    /// instead of companion model (history currents in state).
    #[serde(default)]
    pub augmented_inductors: bool,
}

/// Solver configuration baked into the generated code.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolverConfig {
    pub sample_rate: f64,
    /// alpha = 2 * internal_sample_rate (trapezoidal integration constant)
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

/// Per-device resolved parameters, stored in each `DeviceSlot`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceParams {
    Diode(DiodeParams),
    Bjt(BjtParams),
    Jfet(JfetParams),
    Mosfet(MosfetParams),
    Tube(TubeParams),
}

/// Diode model parameters (resolved from `.model` directive or defaults).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiodeParams {
    /// Saturation current
    pub is: f64,
    /// Ideality factor * thermal voltage
    pub n_vt: f64,
}

/// BJT parameters (Ebers-Moll or Gummel-Poon, resolved from `.model` directive).
///
/// When `vaf`, `var`, `ikf`, `ikr` are all infinite (the default), this reduces
/// to the basic Ebers-Moll model (backward compatible). Any finite GP parameter
/// activates the Gummel-Poon extension with Early effect and high-injection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BjtParams {
    /// Saturation current
    pub is: f64,
    /// Thermal voltage
    pub vt: f64,
    /// Forward current gain
    pub beta_f: f64,
    /// Reverse current gain
    pub beta_r: f64,
    /// True if PNP (false = NPN)
    #[serde(default)]
    pub is_pnp: bool,
    /// Forward Early voltage [V] (inf = no Early effect)
    #[serde(default = "default_infinity", deserialize_with = "deserialize_f64_or_infinity")]
    pub vaf: f64,
    /// Reverse Early voltage [V] (inf = no Early effect)
    #[serde(default = "default_infinity", deserialize_with = "deserialize_f64_or_infinity")]
    pub var: f64,
    /// Forward knee current [A] (inf = no high injection)
    #[serde(default = "default_infinity", deserialize_with = "deserialize_f64_or_infinity")]
    pub ikf: f64,
    /// Reverse knee current [A] (inf = no high injection)
    #[serde(default = "default_infinity", deserialize_with = "deserialize_f64_or_infinity")]
    pub ikr: f64,
}

fn default_infinity() -> f64 {
    f64::INFINITY
}

/// Deserialize an f64 that may be null (JSON cannot represent infinity).
/// Maps null → f64::INFINITY so old serialized data is handled gracefully.
fn deserialize_f64_or_infinity<'de, D>(deserializer: D) -> Result<f64, D::Error>
where D: serde::Deserializer<'de> {
    let opt: Option<f64> = Option::deserialize(deserializer)?;
    Ok(opt.unwrap_or(f64::INFINITY))
}

impl BjtParams {
    /// Returns true if any Gummel-Poon parameter is finite.
    pub fn is_gummel_poon(&self) -> bool {
        self.vaf.is_finite() || self.var.is_finite() || self.ikf.is_finite() || self.ikr.is_finite()
    }
}

/// A slot in the nonlinear system: maps a device to its M-dimension range.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceSlot {
    /// Device type tag (for NR dispatch)
    pub device_type: DeviceType,
    /// Starting index in the M-dimension vectors
    pub start_idx: usize,
    /// Number of dimensions this device occupies
    pub dimension: usize,
    /// Per-device resolved parameters (from `.model` directive or defaults)
    pub params: DeviceParams,
}

/// JFET model parameters (resolved from `.model` directive or defaults).
///
/// Codegen uses 2D Shichman-Hodges: Vgs and Vds control Id (triode + saturation regions).
/// Gate current Ig (dimension 2) is effectively zero for reverse-biased gate.
/// This matches the MNA stamping where JFET is 2D (dimension=2, controlling voltages=Vgs, Vds).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JfetParams {
    /// Saturation current IDSS [A]
    pub idss: f64,
    /// Pinch-off voltage [V] (negative for N-channel, positive for P-channel)
    pub vp: f64,
    /// Channel length modulation [1/V]
    pub lambda: f64,
    /// True if P-channel (false = N-channel)
    pub is_p_channel: bool,
}

/// MOSFET model parameters (Level 1 SPICE, triode + saturation).
///
/// Codegen uses 2D: Vgs and Vds control Id (triode + saturation regions).
/// Gate current Ig (dimension 2) is zero (insulated gate).
/// MNA stamping: 2D (dimension=2, controlling voltages=Vgs, Vds).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MosfetParams {
    /// Transconductance parameter KP [A/V²]
    pub kp: f64,
    /// Threshold voltage VT [V] (positive for N-channel, negative for P-channel)
    pub vt: f64,
    /// Channel length modulation [1/V]
    pub lambda: f64,
    /// True if P-channel (false = N-channel)
    pub is_p_channel: bool,
}

/// Tube/triode model parameters (Koren + improved grid current).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TubeParams {
    /// Amplification factor (mu)
    pub mu: f64,
    /// Exponent for Koren's equation
    pub ex: f64,
    /// Kg1 coefficient
    pub kg1: f64,
    /// Kp coefficient
    pub kp: f64,
    /// Kvb coefficient (for knee shaping)
    pub kvb: f64,
    /// Maximum grid current [A]
    pub ig_max: f64,
    /// Grid current onset voltage [V]
    pub vgk_onset: f64,
    /// Channel-length modulation coefficient [1/V]. 0.0 = disabled (default).
    #[serde(default)]
    pub lambda: f64,
}

impl DeviceParams {
}

/// Nonlinear device type tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceType {
    Diode,
    Bjt,
    Jfet,
    Mosfet,
    Tube,
}

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

/// Stamp mutual conductance between two 2-terminal elements into a flat row-major matrix.
/// Node indices are 1-indexed; 0 means ground.
fn stamp_flat_mutual(mat: &mut [f64], n: usize, a: usize, b: usize, c: usize, d: usize, g: f64) {
    if a > 0 && c > 0 { mat[(a - 1) * n + (c - 1)] += g; }
    if b > 0 && d > 0 { mat[(b - 1) * n + (d - 1)] += g; }
    if a > 0 && d > 0 { mat[(a - 1) * n + (d - 1)] -= g; }
    if b > 0 && c > 0 { mat[(b - 1) * n + (c - 1)] -= g; }
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
        (true, false) => { mat[(node_i - 1) * n + (node_i - 1)] += g; }
        (false, true) => { mat[(node_j - 1) * n + (node_j - 1)] += g; }
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
            for j in 0..w { aug.swap(col * w + j, max_row * w + j); }
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
                pivot.abs(), col
            )));
        }
        for j in 0..w { aug[col * w + j] /= pivot; }
        for row in 0..col {
            let factor = aug[row * w + col];
            for j in 0..w { aug[row * w + j] -= factor * aug[col * w + j]; }
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
        let n = kernel.n;         // = n_aug (system dimension)
        let n_nodes = kernel.n_nodes; // original circuit node count
        let m = kernel.m;

        if m > crate::dk::MAX_M {
            return Err(CodegenError::UnsupportedTopology(format!(
                "code generation supports at most M={} nonlinear dimensions, got M={}",
                crate::dk::MAX_M, m
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

        let solver_config = SolverConfig {
            sample_rate: config.sample_rate,
            alpha: 2.0 * internal_rate,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_nodes: config.output_nodes.clone(),
            input_resistance: config.input_resistance,
            oversampling_factor: os_factor,
            output_scales: config.output_scales.clone(),
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
            (dk::flatten_matrix(&aug.g, n, n), dk::flatten_matrix(&aug.c, n, n))
        } else {
            (dk::flatten_matrix(&mna.g, n, n), dk::flatten_matrix(&mna.c, n, n))
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
                    stamp_flat_mutual(&mut a_flat, n, ci.l1_node_i, ci.l1_node_j, ci.l2_node_i, ci.l2_node_j, gm);
                    stamp_flat_mutual(&mut a_flat, n, ci.l2_node_i, ci.l2_node_j, ci.l1_node_i, ci.l1_node_j, gm);
                    stamp_flat_mutual(&mut a_neg_flat, n, ci.l1_node_i, ci.l1_node_j, ci.l2_node_i, ci.l2_node_j, -gm);
                    stamp_flat_mutual(&mut a_neg_flat, n, ci.l2_node_i, ci.l2_node_j, ci.l1_node_i, ci.l1_node_j, -gm);
                }
            }

            // Zero VS/VCVS algebraic rows in A_neg (NOT inductor rows).
            // These have no capacitance and no history — rhs_const provides V_dc directly.
            let a_neg_zero_end = if augmented_inductors { mna.n_aug } else { n };
            if n_nodes < a_neg_zero_end {
                for i in n_nodes..a_neg_zero_end {
                    for j in 0..n {
                        a_neg_flat[i * n + j] = 0.0;
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
            }
        } else {
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
            }
        };

        let device_slots = Self::build_device_info(netlist)?;

        let inductors: Vec<InductorIR> = kernel.inductors.iter().map(|ind| {
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
        }).collect();

        let coupled_inductors: Vec<CoupledInductorIR> = kernel.coupled_inductors.iter().map(|ci| {
            // Recompute conductances at internal rate when oversampling
            let (g_self_1, g_self_2, g_mutual) = if os_factor > 1 {
                let t = 1.0 / internal_rate;
                let m_val = ci.coupling * (ci.l1_inductance * ci.l2_inductance).sqrt();
                let det = ci.l1_inductance * ci.l2_inductance - m_val * m_val;
                let half_t = t / 2.0;
                (half_t * ci.l2_inductance / det, half_t * ci.l1_inductance / det, -half_t * m_val / det)
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
        }).collect();

        let transformer_groups: Vec<TransformerGroupIR> = kernel.transformer_groups.iter().map(|g| {
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
        }).collect();

        let pots = kernel.pots.iter().map(|p| PotentiometerIR {
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
        }).collect();

        // Build switches from MNA resolved info
        let switches: Vec<SwitchIR> = mna.switches.iter().enumerate().map(|(idx, sw)| {
            let components = sw.components.iter().map(|comp| {
                // For inductor components, find matching index in the inductors vec
                let inductor_index = if comp.component_type == 'L' {
                    kernel.inductors.iter().position(|ind| {
                        ind.name.eq_ignore_ascii_case(&comp.name)
                    })
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
                }
            }).collect();
            SwitchIR {
                index: idx,
                components,
                positions: sw.positions.clone(),
                num_positions: sw.positions.len(),
            }
        }).collect();

        let has_dc_sources = kernel.rhs_const.iter().any(|&v| v != 0.0);

        // Use nonlinear DC OP solver (falls back to linear for M=0)
        let dc_op_config = DcOpConfig {
            tolerance: config.dc_op_tolerance,
            max_iterations: config.dc_op_max_iterations,
            input_node: config.input_node,
            input_resistance: config.input_resistance,
            ..DcOpConfig::default()
        };
        let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);
        // Check DC OP significance on the truncated vector (n_aug), not the full
        // n_dc vector which includes inductor branch currents.
        let dc_op_truncated = &dc_result.v_node[..kernel.n];
        let has_dc_op = dc_op_truncated.iter().any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_nl_currents = dc_result.i_nl.clone();

        if !dc_result.converged && m > 0 {
            log::warn!(
                "nonlinear DC OP solver did not converge (method: {:?}), using best estimate",
                dc_result.method
            );
        }

        // Analyze sparsity patterns for compile-time matrices
        let sparsity = SparseInfo {
            a_neg: analyze_matrix_sparsity(&matrices.a_neg, n, n),
            n_v: analyze_matrix_sparsity(&matrices.n_v, m, n),
            n_i: analyze_matrix_sparsity(&matrices.n_i, n, m),
            k: analyze_matrix_sparsity(&matrices.k, m, m),
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
            dc_operating_point: if augmented_inductors {
                let mut dc = dc_result.v_node.clone();
                dc.resize(kernel.n, 0.0); // pad with zeros for inductor branch currents
                dc
            } else {
                dc_result.v_node[..kernel.n].to_vec()
            },
            device_slots,
            has_dc_sources,
            has_dc_op,
            dc_nl_currents,
            dc_op_converged,
            inductors,
            coupled_inductors,
            transformer_groups,
            pots,
            switches,
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

        // Build augmented G/C matrices (includes inductor branch variables)
        let aug = mna.build_augmented_matrices();
        let n = aug.n_nodal;

        let sample_rate = config.sample_rate;
        let t = 1.0 / sample_rate;
        let alpha = 2.0 / t;
        let alpha_be = 1.0 / t;

        let topology = Topology {
            n,
            n_nodes,
            m,
            num_devices: mna.num_devices,
            n_aug,
            augmented_inductors: true,
        };

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
        };

        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        // Flatten G and C at n_nodal dimension
        let g_matrix = dk::flatten_matrix(&aug.g, n, n);
        let c_matrix = dk::flatten_matrix(&aug.c, n, n);

        // Build trapezoidal A = G + alpha*C, A_neg = alpha*C - G
        let mut a_flat = vec![0.0f64; n * n];
        let mut a_neg_flat = vec![0.0f64; n * n];
        for i in 0..n {
            for j in 0..n {
                let g = aug.g[i][j];
                let c = aug.c[i][j];
                a_flat[i * n + j] = g + alpha * c;
                a_neg_flat[i * n + j] = alpha * c - g;
            }
        }
        // Zero VS/VCVS algebraic rows in A_neg (NOT inductor rows)
        for i in n_nodes..n_aug {
            for j in 0..n {
                a_neg_flat[i * n + j] = 0.0;
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
        for i in n_nodes..n_aug {
            for j in 0..n {
                a_neg_be_flat[i * n + j] = 0.0;
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

        // Build trapezoidal rhs_const (node rows ×2, VS rows ×1)
        let rhs_const_base = dk::build_rhs_const(mna);
        let mut rhs_const = vec![0.0f64; n];
        for i in 0..n_aug {
            rhs_const[i] = rhs_const_base[i];
        }

        // Build BE rhs_const (node rows ×1, VS rows ×1)
        let mut rhs_const_be = vec![0.0f64; n];
        for src in &mna.current_sources {
            crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_plus_idx, src.dc_value);
            crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_minus_idx, -src.dc_value);
        }
        for vs in &mna.voltage_sources {
            let k = mna.n + vs.ext_idx;
            if k < n { rhs_const_be[k] = vs.dc_value; }
        }

        let matrices = Matrices {
            s: Vec::new(),     // not used in nodal mode
            k: Vec::new(),     // not used in nodal mode
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
        };

        // Run DC OP
        let dc_op_config = DcOpConfig {
            input_node: config.input_node,
            input_resistance: config.input_resistance,
            ..DcOpConfig::default()
        };
        let dc_result = dc_op::solve_dc_operating_point(mna, &Self::build_device_info(netlist)?, &dc_op_config);
        let dc_op_truncated = &dc_result.v_node[..n_aug.min(dc_result.v_node.len())];
        let has_dc_op = dc_op_truncated.iter().any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_nl_currents = dc_result.i_nl.clone();
        let has_dc_sources = !mna.voltage_sources.is_empty() || !mna.current_sources.is_empty();

        // Pad DC OP to n_nodal (inductor branch currents = 0)
        let mut dc_operating_point = dc_result.v_node.clone();
        dc_operating_point.resize(n, 0.0);

        let device_slots = Self::build_device_info(netlist)?;

        // Sparsity analysis
        let sparsity = SparseInfo {
            a_neg: analyze_matrix_sparsity(&matrices.a_neg, n, n),
            n_v: analyze_matrix_sparsity(&matrices.n_v, m, n),
            n_i: analyze_matrix_sparsity(&matrices.n_i, n, m),
            k: analyze_matrix_sparsity(&[], 0, 0), // no K in nodal mode
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
            inductors: Vec::new(),          // no companion model
            coupled_inductors: Vec::new(),
            transformer_groups: Vec::new(),
            pots: Vec::new(),               // TODO: pot support for nodal (Phase 3)
            switches: Vec::new(),           // TODO: switch support for nodal (Phase 3)
            sparsity,
        })
    }

    /// Build device slot map and resolve per-device parameters from netlist.
    ///
    /// # Errors
    /// Returns `CodegenError::InvalidConfig` if any device model parameter is non-positive or non-finite.
    fn build_device_info(netlist: &Netlist) -> Result<Vec<DeviceSlot>, CodegenError> {
        let mut slots = Vec::new();
        let mut dim_offset = 0;

        for elem in &netlist.elements {
            match elem {
                Element::Diode { model, .. } => {
                    let params = Self::resolve_diode_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Diode,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Diode(params),
                    });
                    dim_offset += 1;
                }
                Element::Bjt { model, .. } => {
                    let params = Self::resolve_bjt_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Bjt,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Bjt(params),
                    });
                    dim_offset += 2;
                }
                Element::Jfet { model, .. } => {
                    let params = Self::resolve_jfet_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Jfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Jfet(params),
                    });
                    dim_offset += 2;
                }
                Element::Triode { model, .. } => {
                    let params = Self::resolve_tube_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Tube,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Tube(params),
                    });
                    dim_offset += 2;
                }
                Element::Mosfet { model, .. } => {
                    let params = Self::resolve_mosfet_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Mosfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Mosfet(params),
                    });
                    dim_offset += 2;
                }
                _ => {}
            }
        }

        Ok(slots)
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

        Ok(DiodeParams { is, n_vt: n * vt })
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

        let is_pnp = netlist.models.iter()
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

        Ok(BjtParams { is, vt, beta_f, beta_r, is_pnp, vaf, var, ikf, ikr })
    }

    /// Resolve JFET model parameters from the netlist, with validation.
    ///
    /// 2D Shichman-Hodges: IDSS, VP, and LAMBDA control triode + saturation regions.
    fn resolve_jfet_params(netlist: &Netlist, model: &str) -> Result<JfetParams, CodegenError> {
        let cat = melange_devices::catalog::jfets::lookup(model);

        // Determine channel type first — default VP depends on polarity.
        let is_p_channel = netlist.models.iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
            .unwrap_or(cat.map(|c| c.is_p_channel).unwrap_or(false));

        let default_vp = cat.map(|c| c.vp).unwrap_or(if is_p_channel { 2.0 } else { -2.0 });
        let vp = Self::lookup_model_param(netlist, model, "VTO")
            .unwrap_or(default_vp);
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
            return Err(CodegenError::InvalidConfig(
                format!("JFET model VP must be finite and nonzero, got {vp}")
            ));
        }
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(
                format!("JFET model LAMBDA must be non-negative and finite, got {lambda}")
            ));
        }

        Ok(JfetParams { idss, vp, lambda, is_p_channel })
    }

    /// Resolve MOSFET model parameters from the netlist, with validation.
    fn resolve_mosfet_params(netlist: &Netlist, model: &str) -> Result<MosfetParams, CodegenError> {
        let cat = melange_devices::catalog::mosfets::lookup(model);

        let is_p_channel = netlist.models.iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PM"))
            .unwrap_or(cat.map(|c| c.is_p_channel).unwrap_or(false));

        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(0.1);
        let default_vt = cat.map(|c| c.vt).unwrap_or(if is_p_channel { -2.0 } else { 2.0 });
        let vt = Self::lookup_model_param(netlist, model, "VTO")
            .or_else(|| Self::lookup_model_param(netlist, model, "VT"))
            .unwrap_or(default_vt);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.01);

        validate_positive_finite(kp, "MOSFET model KP")?;
        if !vt.is_finite() {
            return Err(CodegenError::InvalidConfig(
                format!("MOSFET model VT must be finite, got {vt}")
            ));
        }
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(
                format!("MOSFET model LAMBDA must be non-negative and finite, got {lambda}")
            ));
        }

        Ok(MosfetParams { kp, vt, lambda, is_p_channel })
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
            return Err(CodegenError::InvalidConfig(
                format!("tube model LAMBDA must be non-negative and finite, got {lambda}")
            ));
        }

        Ok(TubeParams { mu, ex, kg1, kp, kvb, ig_max, vgk_onset, lambda })
    }

    /// Build the legacy `devices` list (first occurrence of each device type).
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

}
