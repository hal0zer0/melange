//! Circuit intermediate representation (IR) for language-agnostic code generation.
//!
//! `CircuitIR` captures everything a code emitter needs to produce a working solver,
//! without referencing any Rust-specific types from the MNA/DK pipeline.

use serde::{Deserialize, Serialize};

use crate::dk::{self, DkKernel};
use crate::mna::{MnaSystem, VS_CONDUCTANCE};
use crate::parser::{Element, Netlist};

use super::{CodegenConfig, CodegenError};

// ============================================================================
// Top-level IR
// ============================================================================

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
    pub inductors: Vec<InductorIR>,
    pub pots: Vec<PotentiometerIR>,
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
    /// alpha = 2 * sample_rate (trapezoidal integration constant)
    pub alpha: f64,
    pub tolerance: f64,
    pub max_iterations: usize,
    pub input_node: usize,
    pub output_node: usize,
    pub input_resistance: f64,
}

/// All matrices needed by the generated solver (flattened row-major).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Matrices {
    /// S = A^{-1}, N×N row-major
    pub s: Vec<f64>,
    /// A_neg = alpha*C - G, N×N row-major
    pub a_neg: Vec<f64>,
    /// Nonlinear kernel K = N_v * S * N_i, M×M row-major
    pub k: Vec<f64>,
    /// Voltage extraction N_v, M×N row-major
    pub n_v: Vec<f64>,
    /// Current injection N_i, N×M row-major (kernel storage order)
    pub n_i: Vec<f64>,
    /// Constant RHS contribution from DC sources, length N
    pub rhs_const: Vec<f64>,
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
    /// Equivalent conductance T/(2L)
    pub g_eq: f64,
}

/// Resolved parameters for a single nonlinear device (legacy, kept for JSON compat).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceIR {
    Diode(DiodeParams),
    Bjt(BjtParams),
}

/// Per-device resolved parameters, stored in each `DeviceSlot`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceParams {
    Diode(DiodeParams),
    Bjt(BjtParams),
}

/// Diode model parameters (resolved from `.model` directive or defaults).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiodeParams {
    /// Saturation current
    pub is: f64,
    /// Ideality factor * thermal voltage
    pub n_vt: f64,
}

/// BJT Ebers-Moll parameters (resolved from `.model` directive or defaults).
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

/// Nonlinear device type tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceType {
    Diode,
    Bjt,
}

// ============================================================================
// Builder
// ============================================================================

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

        if m > 4 {
            return Err(CodegenError::UnsupportedTopology(format!(
                "code generation supports at most M=4 nonlinear dimensions, got M={}", m
            )));
        }

        // --- Topology ---
        let topology = Topology {
            n,
            m,
            num_devices: kernel.num_devices,
        };

        // --- Solver config ---
        let solver_config = SolverConfig {
            sample_rate: config.sample_rate,
            alpha: 2.0 * config.sample_rate,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_node: config.output_node,
            input_resistance: config.input_resistance,
        };

        // --- Metadata ---
        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        // --- Matrices (direct copy from kernel) ---
        let matrices = Matrices {
            s: kernel.s.clone(),
            a_neg: kernel.a_neg.clone(),
            k: kernel.k.clone(),
            n_v: kernel.n_v.clone(),
            n_i: kernel.n_i.clone(),
            rhs_const: kernel.rhs_const.clone(),
        };

        // --- Device slots + resolved params ---
        let (device_slots, devices) = Self::build_device_info(netlist)?;

        // --- Inductors ---
        let inductors: Vec<InductorIR> = kernel.inductors.iter().map(|ind| InductorIR {
            name: ind.name.to_string(),
            node_i: ind.node_i,
            node_j: ind.node_j,
            g_eq: ind.g_eq,
        }).collect();

        // --- Potentiometers ---
        let pots: Vec<PotentiometerIR> = kernel.pots.iter().map(|p| PotentiometerIR {
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

        // --- DC sources ---
        let has_dc_sources = kernel.rhs_const.iter().any(|&v| v != 0.0);

        // --- DC operating point (includes input conductance) ---
        let dc_op = Self::compute_dc_operating_point(
            mna,
            config.input_node,
            config.input_resistance,
        );
        let has_dc_op = dc_op.iter().any(|&v| v != 0.0);

        Ok(CircuitIR {
            metadata,
            topology,
            solver_config,
            matrices,
            dc_operating_point: dc_op,
            devices,
            device_slots,
            has_dc_sources,
            has_dc_op,
            inductors,
            pots,
        })
    }

    /// Build device slot map and resolve per-device parameters from netlist.
    ///
    /// # Errors
    /// Returns `CodegenError::InvalidConfig` if JFET or MOSFET elements are present.
    fn build_device_info(netlist: &Netlist) -> Result<(Vec<DeviceSlot>, Vec<DeviceIR>), CodegenError> {
        let mut slots = Vec::new();
        let mut devices = Vec::new();
        let mut dim_offset = 0;

        // Track first occurrence of each type for the legacy `devices` vec
        let mut first_diode: Option<DiodeParams> = None;
        let mut first_bjt: Option<BjtParams> = None;

        for elem in &netlist.elements {
            match elem {
                Element::Diode { model, .. } => {
                    let vt = 0.02585;
                    let is = Self::lookup_model_param(netlist, model, "IS")
                        .unwrap_or(2.52e-9);
                    let n = Self::lookup_model_param(netlist, model, "N")
                        .unwrap_or(1.0);
                    if is <= 0.0 || !is.is_finite() {
                        return Err(CodegenError::InvalidConfig(
                            format!("diode model IS must be positive finite, got {}", is)
                        ));
                    }
                    if n <= 0.0 || !n.is_finite() {
                        return Err(CodegenError::InvalidConfig(
                            format!("diode model N must be positive finite, got {}", n)
                        ));
                    }
                    let params = DiodeParams { is, n_vt: n * vt };

                    if first_diode.is_none() {
                        first_diode = Some(params.clone());
                    }

                    slots.push(DeviceSlot {
                        device_type: DeviceType::Diode,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Diode(params),
                    });
                    dim_offset += 1;
                }
                Element::Bjt { model, .. } => {
                    let vt = Self::lookup_model_param(netlist, model, "VT")
                        .unwrap_or(0.02585);
                    let is = Self::lookup_model_param(netlist, model, "IS")
                        .unwrap_or(1.26e-14);
                    let beta_f = Self::lookup_model_param(netlist, model, "BF")
                        .unwrap_or(200.0);
                    let beta_r = Self::lookup_model_param(netlist, model, "BR")
                        .unwrap_or(3.0);
                    if is <= 0.0 || !is.is_finite() {
                        return Err(CodegenError::InvalidConfig(
                            format!("BJT model IS must be positive finite, got {}", is)
                        ));
                    }
                    if vt <= 0.0 || !vt.is_finite() {
                        return Err(CodegenError::InvalidConfig(
                            format!("BJT model VT must be positive finite, got {}", vt)
                        ));
                    }
                    if beta_f <= 0.0 || !beta_f.is_finite() {
                        return Err(CodegenError::InvalidConfig(
                            format!("BJT model BF must be positive finite, got {}", beta_f)
                        ));
                    }
                    if beta_r <= 0.0 || !beta_r.is_finite() {
                        return Err(CodegenError::InvalidConfig(
                            format!("BJT model BR must be positive finite, got {}", beta_r)
                        ));
                    }
                    // Look up polarity from .model directive (default NPN)
                    let is_pnp = netlist.models.iter()
                        .find(|m| m.name.eq_ignore_ascii_case(model))
                        .map(|m| m.model_type.to_uppercase().starts_with("PNP"))
                        .unwrap_or(false);
                    let params = BjtParams { is, vt, beta_f, beta_r, is_pnp };

                    if first_bjt.is_none() {
                        first_bjt = Some(params.clone());
                    }

                    slots.push(DeviceSlot {
                        device_type: DeviceType::Bjt,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Bjt(params),
                    });
                    dim_offset += 2;
                }
                Element::Jfet { name, .. } => {
                    return Err(CodegenError::InvalidConfig(
                        format!("JFET '{}' not supported in code generation", name)
                    ));
                }
                Element::Mosfet { name, .. } => {
                    return Err(CodegenError::InvalidConfig(
                        format!("MOSFET '{}' not supported in code generation", name)
                    ));
                }
                _ => {} // Skip non-device elements (R, C, L, sources)
            }
        }

        // Legacy devices list (first of each type, for backward compat)
        if let Some(dp) = first_diode {
            devices.push(DeviceIR::Diode(dp));
        }
        if let Some(bp) = first_bjt {
            devices.push(DeviceIR::Bjt(bp));
        }

        Ok((slots, devices))
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

    /// Compute the linear DC operating point: v_dc = G_dc^{-1} * b_dc
    ///
    /// Treats all capacitors as open circuits and all nonlinear devices as open.
    /// Includes the input conductance (1/input_resistance) in the G matrix.
    /// Returns zeros if G is singular or there are no DC sources.
    fn compute_dc_operating_point(
        mna: &MnaSystem,
        input_node: usize,
        input_resistance: f64,
    ) -> Vec<f64> {
        let n = mna.n;
        if n == 0 {
            return Vec::new();
        }

        // Build DC source vector (Norton currents, no *2 trapezoidal factor)
        let mut b_dc = vec![0.0; n];
        for vs in &mna.voltage_sources {
            let current = vs.dc_value * VS_CONDUCTANCE;
            if vs.n_plus_idx > 0 {
                b_dc[vs.n_plus_idx - 1] += current;
            }
            if vs.n_minus_idx > 0 {
                b_dc[vs.n_minus_idx - 1] -= current;
            }
        }

        // Current sources: same sign convention as dk.rs build_rhs_const
        for src in &mna.current_sources {
            if src.n_plus_idx > 0 {
                b_dc[src.n_plus_idx - 1] += src.dc_value;
            }
            if src.n_minus_idx > 0 {
                b_dc[src.n_minus_idx - 1] -= src.dc_value;
            }
        }

        // If no DC sources, return zeros
        if b_dc.iter().all(|&v| v == 0.0) {
            return vec![0.0; n];
        }

        // Clone G and stamp input conductance
        let mut g_dc = mna.g.clone();
        if input_node < n && input_resistance > 0.0 {
            g_dc[input_node][input_node] += 1.0 / input_resistance;
        }

        // Invert G matrix (now including input conductance)
        let g_inv = match dk::invert_matrix(&g_dc) {
            Ok(inv) => inv,
            Err(_) => return vec![0.0; n],
        };

        // v_dc = G_inv * b_dc
        let mut v_dc = vec![0.0; n];
        for i in 0..n {
            for j in 0..n {
                v_dc[i] += g_inv[i][j] * b_dc[j];
            }
        }

        v_dc
    }

    // --- Matrix accessors (mirror DkKernel's API for convenience) ---

    /// Access S matrix element S[i][j]
    #[inline(always)]
    pub fn s(&self, i: usize, j: usize) -> f64 {
        self.matrices.s[i * self.topology.n + j]
    }

    /// Access K matrix element K[i][j]
    #[inline(always)]
    pub fn k(&self, i: usize, j: usize) -> f64 {
        self.matrices.k[i * self.topology.m + j]
    }

    /// Access N_v matrix element N_v[i][j]
    #[inline(always)]
    pub fn n_v(&self, i: usize, j: usize) -> f64 {
        self.matrices.n_v[i * self.topology.n + j]
    }

    /// Access N_i matrix element N_i[i][j] (N×M storage: node × device)
    #[inline(always)]
    pub fn n_i(&self, i: usize, j: usize) -> f64 {
        self.matrices.n_i[i * self.topology.m + j]
    }

    /// Access A_neg matrix element A_neg[i][j]
    #[inline(always)]
    pub fn a_neg(&self, i: usize, j: usize) -> f64 {
        self.matrices.a_neg[i * self.topology.n + j]
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
