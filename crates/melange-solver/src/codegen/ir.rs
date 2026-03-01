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
                "code generation supports at most M=8 nonlinear dimensions, got M={}", m
            )));
        }

        let topology = Topology {
            n,
            m,
            num_devices: kernel.num_devices,
        };

        let solver_config = SolverConfig {
            sample_rate: config.sample_rate,
            alpha: 2.0 * config.sample_rate,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_node: config.output_node,
            input_resistance: config.input_resistance,
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

        let matrices = Matrices {
            s: kernel.s.clone(),
            a_neg: kernel.a_neg.clone(),
            k: kernel.k.clone(),
            n_v: kernel.n_v.clone(),
            n_i: kernel.n_i.clone(),
            rhs_const: kernel.rhs_const.clone(),
            g_matrix,
            c_matrix,
        };

        let (device_slots, devices) = Self::build_device_info(netlist)?;

        let inductors = kernel.inductors.iter().map(|ind| InductorIR {
            name: ind.name.to_string(),
            node_i: ind.node_i,
            node_j: ind.node_j,
            g_eq: ind.g_eq,
            inductance: ind.inductance,
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
            eprintln!(
                "Warning: nonlinear DC OP solver did not converge (method: {:?}), using best estimate",
                dc_result.method
            );
        }

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

        Ok(BjtParams { is, vt, beta_f, beta_r, is_pnp })
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
