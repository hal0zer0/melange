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

/// Language-agnostic intermediate representation of a compiled circuit.
///
/// Built from `DkKernel` + `MnaSystem` + `Netlist` + `CodegenConfig`, this
/// struct contains every piece of data an emitter needs — matrices, device
/// parameters, solver config — without referencing the builder types.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CircuitIR {
    pub metadata: CircuitMetadata,
    pub topology: Topology,
    pub solver_config: SolverConfig,
    pub matrices: Matrices,
    pub dc_operating_point: Vec<f64>,
    pub devices: Vec<DeviceIR>,
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
    pub pots: Vec<PotentiometerIR>,
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
    /// Number of circuit nodes (excluding ground)
    pub n: usize,
    /// Total nonlinear dimension (sum of device dimensions)
    pub m: usize,
    /// Number of physical nonlinear devices
    pub num_devices: usize,
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
    pub output_node: usize,
    pub input_resistance: f64,
    /// Oversampling factor (1, 2, or 4). Default 1 (no oversampling).
    #[serde(default = "default_oversampling_factor")]
    pub oversampling_factor: usize,
}

fn default_oversampling_factor() -> usize {
    1
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

/// Resolved parameters for a single nonlinear device (legacy, kept for JSON compat).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceIR {
    Diode(DiodeParams),
    Bjt(BjtParams),
    Tube(TubeParams),
}

/// Per-device resolved parameters, stored in each `DeviceSlot`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceParams {
    Diode(DiodeParams),
    Bjt(BjtParams),
    Jfet(JfetParams),
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
/// Codegen uses 1D simplification: only Vgs controls Id (saturation-only, ignores Vds).
/// This matches the MNA stamping where JFET is 1D (dimension=1, controlling voltage=Vgs).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JfetParams {
    /// Saturation current IDSS [A]
    pub idss: f64,
    /// Pinch-off voltage [V] (negative for N-channel, positive for P-channel)
    pub vp: f64,
    /// Channel length modulation [1/V] (not used in 1D codegen, stored for reference)
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
}

/// Nonlinear device type tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceType {
    Diode,
    Bjt,
    Jfet,
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
/// Returns the identity matrix if singular (graceful degradation).
fn invert_flat_matrix(a: &[f64], n: usize) -> Vec<f64> {
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
            // Singular — return identity
            let mut result = vec![0.0f64; n * n];
            for i in 0..n { result[i * n + i] = 1.0; }
            return result;
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
            let mut result = vec![0.0f64; n * n];
            for i in 0..n { result[i * n + i] = 1.0; }
            return result;
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
    result
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
    /// Returns `CodegenError::InvalidConfig` if the netlist contains JFET or MOSFET
    /// devices, which are not yet supported in code generation.
    pub fn from_kernel(
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
    ) -> Result<Self, CodegenError> {
        let n = kernel.n;
        let m = kernel.m;

        if m > crate::dk::MAX_M {
            return Err(CodegenError::UnsupportedTopology(format!(
                "code generation supports at most M={} nonlinear dimensions, got M={}",
                crate::dk::MAX_M, m
            )));
        }

        let topology = Topology {
            n,
            m,
            num_devices: kernel.num_devices,
        };

        let os_factor = config.oversampling_factor;
        let internal_rate = config.sample_rate * os_factor as f64;

        let solver_config = SolverConfig {
            sample_rate: config.sample_rate,
            alpha: 2.0 * internal_rate,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_node: config.output_node,
            input_resistance: config.input_resistance,
            oversampling_factor: os_factor,
        };

        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        // Store the raw G and C matrices for runtime sample rate recomputation.
        // The MNA G matrix already includes input conductance (stamped before kernel build).
        let g_matrix = dk::flatten_matrix(&mna.g, n, n);
        let c_matrix = dk::flatten_matrix(&mna.c, n, n);

        let matrices = if os_factor > 1 {
            // Recompute matrices at internal (oversampled) rate from G and C.
            let alpha = 2.0 * internal_rate;
            let t = 1.0 / internal_rate;

            // Build A = G + alpha*C (add inductor companion g_eq at internal rate)
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

            // Stamp inductor companion conductances at internal rate
            for ind in &kernel.inductors {
                let g_eq = t / (2.0 * ind.inductance);
                stamp_flat_conductance(&mut a_flat, n, ind.node_i, ind.node_j, g_eq);
                stamp_flat_conductance(&mut a_neg_flat, n, ind.node_i, ind.node_j, -g_eq);
            }

            // Invert A to get S
            let s = invert_flat_matrix(&a_flat, n);

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
            }
        };

        let (device_slots, devices) = Self::build_device_info(netlist)?;

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
        let has_dc_op = dc_result.v_node.iter().any(|&v| v.abs() > 1e-15);
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
            solver_config,
            matrices,
            dc_operating_point: dc_result.v_node,
            devices,
            device_slots,
            has_dc_sources,
            has_dc_op,
            dc_nl_currents,
            dc_op_converged,
            inductors,
            pots,
            sparsity,
        })
    }

    /// Build device slot map and resolve per-device parameters from netlist.
    ///
    /// # Errors
    /// Returns `CodegenError::InvalidConfig` if JFET or MOSFET elements are present,
    /// or if any device model parameter is non-positive or non-finite.
    fn build_device_info(netlist: &Netlist) -> Result<(Vec<DeviceSlot>, Vec<DeviceIR>), CodegenError> {
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
                        dimension: 1,
                        params: DeviceParams::Jfet(params),
                    });
                    dim_offset += 1;
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
                Element::Mosfet { name, .. } => {
                    return Err(CodegenError::InvalidConfig(
                        format!("MOSFET '{}' not supported in code generation", name)
                    ));
                }
                _ => {}
            }
        }

        // Legacy devices list: first occurrence of each type (for JSON backward compat)
        let devices = Self::build_legacy_devices(&slots);

        Ok((slots, devices))
    }

    /// Resolve diode model parameters from the netlist, with validation.
    fn resolve_diode_params(netlist: &Netlist, model: &str) -> Result<DiodeParams, CodegenError> {
        let vt = 0.02585;
        let is = Self::lookup_model_param(netlist, model, "IS").unwrap_or(2.52e-9);
        let n = Self::lookup_model_param(netlist, model, "N").unwrap_or(1.0);

        validate_positive_finite(is, "diode model IS")?;
        validate_positive_finite(n, "diode model N")?;

        Ok(DiodeParams { is, n_vt: n * vt })
    }

    /// Resolve BJT model parameters from the netlist, with validation.
    ///
    /// Gummel-Poon parameters (VAF, VAR, IKF, IKR) default to infinity,
    /// which collapses qb→1.0, giving exact Ebers-Moll behavior.
    fn resolve_bjt_params(netlist: &Netlist, model: &str) -> Result<BjtParams, CodegenError> {
        let vt = Self::lookup_model_param(netlist, model, "VT").unwrap_or(0.02585);
        let is = Self::lookup_model_param(netlist, model, "IS").unwrap_or(1.26e-14);
        let beta_f = Self::lookup_model_param(netlist, model, "BF").unwrap_or(200.0);
        let beta_r = Self::lookup_model_param(netlist, model, "BR").unwrap_or(3.0);

        validate_positive_finite(is, "BJT model IS")?;
        validate_positive_finite(vt, "BJT model VT")?;
        validate_positive_finite(beta_f, "BJT model BF")?;
        validate_positive_finite(beta_r, "BJT model BR")?;

        let is_pnp = netlist.models.iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PNP"))
            .unwrap_or(false);

        // Gummel-Poon parameters (default to infinity = pure Ebers-Moll)
        let vaf = Self::lookup_model_param(netlist, model, "VAF")
            .or_else(|| Self::lookup_model_param(netlist, model, "VA"))
            .unwrap_or(f64::INFINITY);
        let var = Self::lookup_model_param(netlist, model, "VAR")
            .or_else(|| Self::lookup_model_param(netlist, model, "VB"))
            .unwrap_or(f64::INFINITY);
        let ikf = Self::lookup_model_param(netlist, model, "IKF")
            .or_else(|| Self::lookup_model_param(netlist, model, "JBF"))
            .unwrap_or(f64::INFINITY);
        let ikr = Self::lookup_model_param(netlist, model, "IKR")
            .or_else(|| Self::lookup_model_param(netlist, model, "JBR"))
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
    /// Uses 1D simplification: only IDSS and VP are used in codegen.
    /// Lambda is stored but not used in the saturation-only model.
    fn resolve_jfet_params(netlist: &Netlist, model: &str) -> Result<JfetParams, CodegenError> {
        // Determine channel type first — default VP depends on polarity.
        let is_p_channel = netlist.models.iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
            .unwrap_or(false);

        let idss = Self::lookup_model_param(netlist, model, "IDSS")
            .or_else(|| Self::lookup_model_param(netlist, model, "BETA"))
            .unwrap_or(2e-3);
        let default_vp = if is_p_channel { 2.0 } else { -2.0 };
        let vp = Self::lookup_model_param(netlist, model, "VTO")
            .unwrap_or(default_vp);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA").unwrap_or(0.001);

        validate_positive_finite(idss, "JFET model IDSS")?;
        if !vp.is_finite() || vp.abs() < 1e-15 {
            return Err(CodegenError::InvalidConfig(
                format!("JFET model VP must be finite and nonzero, got {vp}")
            ));
        }

        Ok(JfetParams { idss, vp, lambda, is_p_channel })
    }

    /// Resolve tube/triode model parameters from the netlist, with validation.
    ///
    /// Defaults are 12AX7 (ECC83) values.
    fn resolve_tube_params(netlist: &Netlist, model: &str) -> Result<TubeParams, CodegenError> {
        let mu = Self::lookup_model_param(netlist, model, "MU").unwrap_or(100.0);
        let ex = Self::lookup_model_param(netlist, model, "EX").unwrap_or(1.4);
        let kg1 = Self::lookup_model_param(netlist, model, "KG1").unwrap_or(1060.0);
        let kp = Self::lookup_model_param(netlist, model, "KP").unwrap_or(600.0);
        let kvb = Self::lookup_model_param(netlist, model, "KVB").unwrap_or(300.0);
        let ig_max = Self::lookup_model_param(netlist, model, "IG_MAX").unwrap_or(2e-3);
        let vgk_onset = Self::lookup_model_param(netlist, model, "VGK_ONSET").unwrap_or(0.5);

        validate_positive_finite(mu, "tube model MU")?;
        validate_positive_finite(ex, "tube model EX")?;
        validate_positive_finite(kg1, "tube model KG1")?;
        validate_positive_finite(kp, "tube model KP")?;
        validate_positive_finite(kvb, "tube model KVB")?;
        validate_positive_finite(ig_max, "tube model IG_MAX")?;
        validate_positive_finite(vgk_onset, "tube model VGK_ONSET")?;

        Ok(TubeParams { mu, ex, kg1, kp, kvb, ig_max, vgk_onset })
    }

    /// Build the legacy `devices` list (first occurrence of each device type).
    fn build_legacy_devices(slots: &[DeviceSlot]) -> Vec<DeviceIR> {
        let mut devices = Vec::new();
        let mut has_diode = false;
        let mut has_bjt = false;

        for slot in slots {
            match &slot.params {
                DeviceParams::Diode(p) if !has_diode => {
                    devices.push(DeviceIR::Diode(p.clone()));
                    has_diode = true;
                }
                DeviceParams::Bjt(p) if !has_bjt => {
                    devices.push(DeviceIR::Bjt(p.clone()));
                    has_bjt = true;
                }
                _ => {}
            }
            if has_diode && has_bjt {
                break;
            }
        }

        devices
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

    /// Get the first diode params (if any diodes exist).
    pub fn diode_params(&self) -> Option<&DiodeParams> {
        self.devices.iter().find_map(|d| match d {
            DeviceIR::Diode(p) => Some(p),
            _ => None,
        })
    }

    /// Get the first BJT params (if any BJTs exist).
    pub fn bjt_params(&self) -> Option<&BjtParams> {
        self.devices.iter().find_map(|d| match d {
            DeviceIR::Bjt(p) => Some(p),
            _ => None,
        })
    }
}
