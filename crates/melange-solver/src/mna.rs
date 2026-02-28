//! Modified Nodal Analysis (MNA) matrix assembler.
//!
//! Builds the G (conductance) and C (capacitance) matrices from a parsed netlist.
//! Also constructs the N_v and N_i matrices for nonlinear device connection.
//!
//! # Multi-dimensional Devices
//!
//! Devices like BJTs have multiple controlling voltages (Vbe, Vbc) and currents
//! (Ic, Ib). The MNA system tracks:
//! - `n`: number of circuit nodes (excluding ground)
//! - `m`: total voltage dimension (sum of all device dimensions)
//! - `num_devices`: number of physical devices
//!
//! For a circuit with 1 diode + 1 BJT:
//! - diode: dimension 1 (controls 1 voltage, contributes 1 current)
//! - BJT: dimension 2 (controls 2 voltages, contributes 2 currents)
//! - m = 1 + 2 = 3

use crate::parser::{Element, Netlist};
use std::collections::HashMap;

/// MNA system matrices.
///
/// Represents the linear part of the circuit:
///   (G + s*C) * V = I
///
/// After discretization with timestep T:
///   A = 2*C/T + G  (trapezoidal rule)
#[derive(Debug, Clone)]
pub struct MnaSystem {
    /// Number of nodes (excluding ground)
    pub n: usize,
    /// Total voltage dimension (sum of device dimensions)
    pub m: usize,
    /// Number of physical nonlinear devices
    pub num_devices: usize,
    /// Conductance matrix (symmetric, n×n)
    pub g: Vec<Vec<f64>>,
    /// Capacitance matrix (symmetric, n×n)
    pub c: Vec<Vec<f64>>,
    /// Nonlinear voltage extraction matrix (m×n)
    /// Row i maps node voltages to controlling voltage i
    pub n_v: Vec<Vec<f64>>,
    /// Nonlinear current injection matrix (n×m)
    /// Column j maps nonlinear current j to node currents
    pub n_i: Vec<Vec<f64>>,
    /// Node name to index mapping (0 = ground, not stored)
    pub node_map: HashMap<String, usize>,
    /// Nonlinear device info
    pub nonlinear_devices: Vec<NonlinearDeviceInfo>,
    /// Voltage source info (for extended MNA)
    pub voltage_sources: Vec<VoltageSourceInfo>,
    /// Current source contributions to RHS
    pub current_sources: Vec<CurrentSourceInfo>,
    /// Inductor elements for companion model
    pub inductors: Vec<InductorElement>,
    /// Potentiometer info (resolved from .pot directives)
    pub pots: Vec<PotInfo>,
}

/// Inductor element info for companion model.
#[derive(Debug, Clone)]
pub struct InductorElement {
    pub name: String,
    pub node_i: usize,
    pub node_j: usize,
    pub value: f64,
}

/// Information about a nonlinear device in the MNA system.
#[derive(Debug, Clone)]
pub struct NonlinearDeviceInfo {
    pub name: String,
    pub device_type: NonlinearDeviceType,
    /// Device dimension (1 for diode, 2 for BJT, etc.)
    pub dimension: usize,
    /// Starting row in N_v / column in N_i for this device
    pub start_idx: usize,
    pub nodes: Vec<String>,
    pub node_indices: Vec<usize>,
}

/// Types of nonlinear devices supported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NonlinearDeviceType {
    Diode,
    Bjt,
    Jfet,
    Mosfet,
}

/// Voltage source information for extended MNA.
#[derive(Debug, Clone)]
pub struct VoltageSourceInfo {
    pub name: String,
    pub n_plus: String,
    pub n_minus: String,
    pub n_plus_idx: usize,
    pub n_minus_idx: usize,
    pub dc_value: f64,
    /// Index in extended MNA (for voltage source currents)
    pub ext_idx: usize,
}

/// Current source information.
#[derive(Debug, Clone)]
pub struct CurrentSourceInfo {
    pub name: String,
    pub n_plus_idx: usize,
    pub n_minus_idx: usize,
    pub dc_value: f64,
}

/// Potentiometer information resolved from .pot directive.
#[derive(Debug, Clone)]
pub struct PotInfo {
    /// Name of the resistor this pot controls
    pub name: String,
    /// 0-indexed MNA node index for the positive terminal (0 = grounded)
    pub node_p: usize,
    /// 0-indexed MNA node index for the negative terminal (0 = grounded)
    pub node_q: usize,
    /// Nominal conductance (1/R_nominal, from the resistor value in the netlist)
    pub g_nominal: f64,
    /// Minimum resistance in ohms
    pub min_resistance: f64,
    /// Maximum resistance in ohms
    pub max_resistance: f64,
    /// True if one terminal is grounded (simplifies SM update)
    pub grounded: bool,
}

impl MnaSystem {
    /// Create a new empty MNA system.
    pub fn new(n: usize, m: usize, num_devices: usize, num_vs: usize) -> Self {
        Self {
            n,
            m,
            num_devices,
            g: vec![vec![0.0; n]; n],
            c: vec![vec![0.0; n]; n],
            n_v: vec![vec![0.0; n]; m],
            n_i: vec![vec![0.0; m]; n],
            node_map: HashMap::new(),
            nonlinear_devices: Vec::new(),
            voltage_sources: Vec::with_capacity(num_vs),
            current_sources: Vec::new(),
            inductors: Vec::new(),
            pots: Vec::new(),
        }
    }

    /// Assemble MNA matrices from a netlist.
    pub fn from_netlist(netlist: &Netlist) -> Result<Self, MnaError> {
        let builder = MnaBuilder::new();
        builder.build(netlist)
    }

    /// Stamp a resistor between two nodes.
    ///
    /// G[i,i] += g, G[j,j] += g, G[i,j] -= g, G[j,i] -= g
    pub fn stamp_resistor(&mut self, i: usize, j: usize, resistance: f64) {
        if resistance == 0.0 {
            return; // Short circuit - handled differently
        }
        let g = 1.0 / resistance;
        self.g[i][i] += g;
        self.g[j][j] += g;
        self.g[i][j] -= g;
        self.g[j][i] -= g;
    }

    /// Stamp a capacitor between two nodes.
    pub fn stamp_capacitor(&mut self, i: usize, j: usize, capacitance: f64) {
        self.c[i][i] += capacitance;
        self.c[j][j] += capacitance;
        self.c[i][j] -= capacitance;
        self.c[j][i] -= capacitance;
    }

    /// Stamp input conductance to ground at a node.
    /// 
    /// This represents the Thevenin equivalent of the input source:
    /// a voltage source in series with a resistance.
    /// The conductance g = 1/R is stamped from the node to ground.
    pub fn stamp_input_conductance(&mut self, node: usize, conductance: f64) {
        if conductance > 0.0 {
            self.g[node][node] += conductance;
        }
    }

    /// Stamp a two-terminal nonlinear device (diode).
    ///
    /// The device current flows from anode (i) to cathode (j).
    /// Controlling voltage is V_i - V_j.
    pub fn stamp_nonlinear_2terminal(
        &mut self,
        device_idx: usize,
        i: usize,
        j: usize,
    ) {
        // N_v extracts controlling voltage: v_nl = v_i - v_j
        self.n_v[device_idx][i] = 1.0;
        self.n_v[device_idx][j] = -1.0;

        // N_i injects current: i_i = -i_nl, i_j = i_nl
        self.n_i[i][device_idx] = -1.0;
        self.n_i[j][device_idx] = 1.0;
    }

    /// Stamp a BJT (2-dimensional device).
    ///
    /// Controlling voltages: Vbe (base - emitter), Vbc (base - collector)
    /// Currents: Ic (collector current), Ib (base current)
    ///
    /// start_idx: starting row/column in N_v/N_i for this device
    pub fn stamp_bjt(
        &mut self,
        start_idx: usize,
        nc: usize,
        nb: usize,
        ne: usize,
    ) {
        // Row start_idx: Vbe = Vb - Ve
        self.n_v[start_idx][nb] = 1.0;
        self.n_v[start_idx][ne] = -1.0;

        // Row start_idx+1: Vbc = Vb - Vc
        self.n_v[start_idx + 1][nb] = 1.0;
        self.n_v[start_idx + 1][nc] = -1.0;

        // Column start_idx: Ic enters collector, exits emitter
        // N_i convention: positive = current entering node from device
        self.n_i[nc][start_idx] = -1.0; // Ic enters collector (current into device = negative)
        self.n_i[ne][start_idx] = 1.0;  // Ic exits emitter (current out of device = positive by KCL)

        // Column start_idx+1: Ib enters base, exits emitter
        self.n_i[nb][start_idx + 1] = -1.0; // Ib enters base
        self.n_i[ne][start_idx + 1] = 1.0;  // Ib exits emitter (KCL conservation)
    }

    /// Get the A matrix for a given sample rate (trapezoidal discretization).
    ///
    /// A = 2*C/T + G (includes inductor companion model conductances)
    ///
    /// # Panics
    /// Panics if `sample_rate` is not positive and finite.
    #[allow(clippy::needless_range_loop)]
    pub fn get_a_matrix(&self, sample_rate: f64) -> Vec<Vec<f64>> {
        assert!(sample_rate > 0.0 && sample_rate.is_finite(), 
                "sample_rate must be positive and finite, got {}", sample_rate);
        let t = 1.0 / sample_rate;
        let alpha = 2.0 / t; // 2/T for trapezoidal

        let mut a = vec![vec![0.0; self.n]; self.n];
        for i in 0..self.n {
            for j in 0..self.n {
                a[i][j] = self.g[i][j] + alpha * self.c[i][j];
            }
        }
        
        // Add inductor companion model conductances: g_eq = T/(2L)
        let g_eq_inductor = t / 2.0;
        for ind in &self.inductors {
            if ind.node_i > 0 && ind.node_j > 0 {
                let i = ind.node_i - 1;
                let j = ind.node_j - 1;
                let g = g_eq_inductor / ind.value;
                a[i][i] += g;
                a[j][j] += g;
                a[i][j] -= g;
                a[j][i] -= g;
            } else if ind.node_i > 0 {
                let i = ind.node_i - 1;
                let g = g_eq_inductor / ind.value;
                a[i][i] += g;
            } else if ind.node_j > 0 {
                let j = ind.node_j - 1;
                let g = g_eq_inductor / ind.value;
                a[j][j] += g;
            }
        }
        
        a
    }

    /// Get the A_neg matrix for history term (trapezoidal discretization).
    ///
    /// A_neg = 2*C/T - G (includes inductor companion model)
    ///
    /// # Panics
    /// Panics if `sample_rate` is not positive and finite.
    #[allow(clippy::needless_range_loop)]
    pub fn get_a_neg_matrix(&self, sample_rate: f64) -> Vec<Vec<f64>> {
        assert!(sample_rate > 0.0 && sample_rate.is_finite(),
                "sample_rate must be positive and finite, got {}", sample_rate);
        let t = 1.0 / sample_rate;
        let alpha = 2.0 / t;

        let mut a_neg = vec![vec![0.0; self.n]; self.n];
        for i in 0..self.n {
            for j in 0..self.n {
                a_neg[i][j] = alpha * self.c[i][j] - self.g[i][j];
            }
        }
        
        // Inductor companion model contributes to A_neg with opposite sign to A.
        // A has +g_eq (like resistor), A_neg has -g_eq (opposite: inductor stores energy)
        let g_eq_inductor = t / 2.0;
        for ind in &self.inductors {
            if ind.node_i > 0 && ind.node_j > 0 {
                let i = ind.node_i - 1;
                let j = ind.node_j - 1;
                let g = g_eq_inductor / ind.value;
                a_neg[i][i] -= g;
                a_neg[j][j] -= g;
                a_neg[i][j] += g;
                a_neg[j][i] += g;
            } else if ind.node_i > 0 {
                let i = ind.node_i - 1;
                let g = g_eq_inductor / ind.value;
                a_neg[i][i] -= g;
            } else if ind.node_j > 0 {
                let j = ind.node_j - 1;
                let g = g_eq_inductor / ind.value;
                a_neg[j][j] -= g;
            }
        }
        
        a_neg
    }
}

/// Norton equivalent conductance for voltage sources [S].
///
/// A large conductance value used to model ideal voltage sources as
/// Norton equivalents (current source + parallel conductance).
pub const VS_CONDUCTANCE: f64 = 1e6;

/// Error type for MNA assembly.
#[derive(Debug, Clone)]
pub enum MnaError {
    /// A component has an invalid value (e.g., negative resistance).
    InvalidComponentValue {
        component: String,
        value: f64,
        reason: String,
    },
    /// An invalid parameter was provided.
    InvalidParameter(String),
    /// A circuit topology error (e.g., unknown node reference).
    TopologyError(String),
}

impl std::fmt::Display for MnaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MnaError::InvalidComponentValue { component, value, reason } => {
                write!(f, "MNA error: invalid value for '{}': {} ({})", component, value, reason)
            }
            MnaError::InvalidParameter(msg) => write!(f, "MNA error: {}", msg),
            MnaError::TopologyError(msg) => write!(f, "MNA error: {}", msg),
        }
    }
}

impl std::error::Error for MnaError {}

/// Builder for MNA systems.
struct MnaBuilder {
    node_map: HashMap<String, usize>,
    next_node_idx: usize,
    nonlinear_devices: Vec<NonlinearDeviceInfo>,
    voltage_sources: Vec<VoltageSourceInfo>,
    current_sources: Vec<CurrentSourceInfo>,
    inductors: Vec<InductorElement>,
    elements: Vec<ElementInfo>,
    /// Total voltage dimension accumulated so far
    total_dimension: usize,
}

#[allow(dead_code)]
struct ElementInfo {
    element_type: ElementType,
    nodes: Vec<usize>,
    value: f64,
    name: String,
    dc_value: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
enum ElementType {
    Resistor,
    Capacitor,
    Inductor,
    VoltageSource,
    CurrentSource,
    Diode,
    Bjt,
    Jfet,
    Mosfet,
}

impl MnaBuilder {
    fn new() -> Self {
        let mut node_map = HashMap::new();
        node_map.insert("0".to_string(), 0); // Ground is always node 0

        Self {
            node_map,
            next_node_idx: 1,
            nonlinear_devices: Vec::new(),
            voltage_sources: Vec::new(),
            current_sources: Vec::new(),
            inductors: Vec::new(),
            elements: Vec::new(),
            total_dimension: 0,
        }
    }

    fn build(mut self, netlist: &Netlist) -> Result<MnaSystem, MnaError> {
        // First pass: collect all node names and assign indices
        for element in &netlist.elements {
            self.collect_nodes(element)?;
        }

        // Second pass: categorize elements and build device info
        for element in &netlist.elements {
            self.categorize_element(element)?;
        }

        // Create MNA system with correct dimensions
        let n = self.next_node_idx - 1; // Exclude ground
        let m = self.total_dimension;
        let num_devices = self.nonlinear_devices.len();
        let num_vs = self.voltage_sources.len();
        let mut mna = MnaSystem::new(n, m, num_devices, num_vs);

        // Resolve pot directives before moving node_map
        for pot_dir in &netlist.pots {
            let resistor = netlist.elements.iter().find(|e| {
                matches!(e, Element::Resistor { name, .. } if name == &pot_dir.resistor_name)
            });
            if let Some(Element::Resistor { n_plus, n_minus, value, .. }) = resistor {
                let node_p = self.node_map[n_plus];
                let node_q = self.node_map[n_minus];
                let grounded = node_p == 0 || node_q == 0;
                mna.pots.push(PotInfo {
                    name: pot_dir.resistor_name.clone(),
                    node_p,
                    node_q,
                    g_nominal: 1.0 / *value,
                    min_resistance: pot_dir.min_value,
                    max_resistance: pot_dir.max_value,
                    grounded,
                });
            } else {
                return Err(MnaError::TopologyError(format!(
                    ".pot references resistor '{}' which was not found",
                    pot_dir.resistor_name
                )));
            }
        }

        mna.node_map = self.node_map;
        mna.nonlinear_devices = self.nonlinear_devices;
        mna.voltage_sources = self.voltage_sources;
        mna.current_sources = self.current_sources;
        mna.inductors = self.inductors;

        // Stamp linear elements
        for elem in &self.elements {
            match elem.element_type {
                ElementType::Resistor => {
                    if elem.nodes.len() >= 2 {
                        let node_i = elem.nodes[0];
                        let node_j = elem.nodes[1];
                        let r = elem.value;
                        if r == 0.0 {
                            continue; // Short circuit - skip to avoid division by zero
                        }

                        // Handle resistor to ground
                        if node_i == 0 && node_j > 0 {
                            let j = node_j - 1;
                            let g = 1.0 / r;
                            mna.g[j][j] += g;
                        } else if node_j == 0 && node_i > 0 {
                            let i = node_i - 1;
                            let g = 1.0 / r;
                            mna.g[i][i] += g;
                        } else if node_i > 0 && node_j > 0 {
                            let i = node_i - 1;
                            let j = node_j - 1;
                            mna.stamp_resistor(i, j, r);
                        }
                    }
                }
                ElementType::Capacitor => {
                    if elem.nodes.len() >= 2 {
                        let node_i = elem.nodes[0];
                        let node_j = elem.nodes[1];
                        let c = elem.value;

                        if node_i == 0 && node_j > 0 {
                            let j = node_j - 1;
                            mna.c[j][j] += c;
                        } else if node_j == 0 && node_i > 0 {
                            let i = node_i - 1;
                            mna.c[i][i] += c;
                        } else if node_i > 0 && node_j > 0 {
                            let i = node_i - 1;
                            let j = node_j - 1;
                            mna.stamp_capacitor(i, j, c);
                        }
                    }
                }
                ElementType::Inductor => {
                    // Inductors are handled in DK kernel with companion model
                    // We store them but stamping happens at kernel creation
                    // since we need sample rate
                }
                ElementType::VoltageSource => {
                    // Norton equivalent: stamp large conductance between nodes
                    // The current contribution is handled in build_rhs_const
                    if elem.nodes.len() >= 2 {
                        let node_i = elem.nodes[0]; // n_plus
                        let node_j = elem.nodes[1]; // n_minus
                        if node_i == 0 && node_j > 0 {
                            let j = node_j - 1;
                            mna.g[j][j] += VS_CONDUCTANCE;
                        } else if node_j == 0 && node_i > 0 {
                            let i = node_i - 1;
                            mna.g[i][i] += VS_CONDUCTANCE;
                        } else if node_i > 0 && node_j > 0 {
                            let i = node_i - 1;
                            let j = node_j - 1;
                            mna.g[i][i] += VS_CONDUCTANCE;
                            mna.g[j][j] += VS_CONDUCTANCE;
                            mna.g[i][j] -= VS_CONDUCTANCE;
                            mna.g[j][i] -= VS_CONDUCTANCE;
                        }
                    }
                }
                _ => {} // Nonlinear handled separately
            }
        }

        // Stamp nonlinear devices (collect indices first to avoid borrow issues)
        let device_info: Vec<_> = mna.nonlinear_devices.iter()
            .map(|d| (d.device_type, d.dimension, d.start_idx, d.node_indices.clone()))
            .collect();
            
        for (dev_type, _dim, start_idx, node_indices) in device_info {
            match dev_type {
                NonlinearDeviceType::Diode => {
                    if node_indices.len() >= 2 {
                        let node_i = node_indices[0];
                        let node_j = node_indices[1];

                        // For diode from node_i (anode) to node_j (cathode):
                        // v_d = v_anode - v_cathode
                        // If anode grounded (v_i=0): v_d = -v_j, so N_v[j] = -1 extracts -v_j
                        // If cathode grounded (v_j=0): v_d = v_i, so N_v[i] = 1 extracts v_i
                        //
                        // N_i convention: positive = current INJECTED INTO node
                        // For current i_d flowing anode→cathode:
                        // - Extracted from anode: N_i[anode] = -1
                        // - Injected into cathode: N_i[cathode] = +1
                        if node_i == 0 && node_j > 0 {
                            let j = node_j - 1;
                            mna.n_v[start_idx][j] = -1.0;  // v_d = 0 - v_j = -v_j
                            mna.n_i[j][start_idx] = 1.0;   // Current injected into cathode
                        } else if node_j == 0 && node_i > 0 {
                            let i = node_i - 1;
                            mna.n_v[start_idx][i] = 1.0;   // v_d = v_i - 0 = v_i
                            mna.n_i[i][start_idx] = -1.0;  // Current extracted from anode
                        } else if node_i > 0 && node_j > 0 {
                            let i = node_i - 1;
                            let j = node_j - 1;
                            mna.stamp_nonlinear_2terminal(start_idx, i, j);
                        }
                    }
                }
                NonlinearDeviceType::Bjt => {
                    if node_indices.len() >= 3 {
                        let c_raw = node_indices[0];
                        let b_raw = node_indices[1];
                        let e_raw = node_indices[2];

                        // All-grounded BJTs are rejected in categorize_element,
                        // so c/b/e can't all be 0 here.
                        if c_raw > 0 && b_raw > 0 && e_raw > 0 {
                            // No grounded terminals — use standard stamp
                            mna.stamp_bjt(start_idx, c_raw - 1, b_raw - 1, e_raw - 1);
                        } else {
                            // Per-terminal ground handling
                            // N_v row 0 (Vbe): +1 at B, -1 at E
                            if b_raw > 0 { mna.n_v[start_idx][b_raw - 1] = 1.0; }
                            if e_raw > 0 { mna.n_v[start_idx][e_raw - 1] = -1.0; }
                            // N_v row 1 (Vbc): +1 at B, -1 at C
                            if b_raw > 0 { mna.n_v[start_idx + 1][b_raw - 1] = 1.0; }
                            if c_raw > 0 { mna.n_v[start_idx + 1][c_raw - 1] = -1.0; }
                            // N_i col 0 (Ic): -1 at C, +1 at E
                            if c_raw > 0 { mna.n_i[c_raw - 1][start_idx] = -1.0; }
                            if e_raw > 0 { mna.n_i[e_raw - 1][start_idx] = 1.0; }
                            // N_i col 1 (Ib): -1 at B, +1 at E
                            if b_raw > 0 { mna.n_i[b_raw - 1][start_idx + 1] = -1.0; }
                            if e_raw > 0 { mna.n_i[e_raw - 1][start_idx + 1] = 1.0; }
                        }
                    }
                }
                NonlinearDeviceType::Jfet => {
                    // JFET: controlling voltage Vgs, drain current Id
                    // Nodes: [nd, ng, ns]
                    if node_indices.len() >= 3 {
                        let d_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let s_raw = node_indices[2];

                        // N_v: Vgs = Vg - Vs (controlling voltage)
                        if g_raw > 0 { mna.n_v[start_idx][g_raw - 1] = 1.0; }
                        if s_raw > 0 { mna.n_v[start_idx][s_raw - 1] = -1.0; }

                        // N_i: Id flows drain→source
                        if d_raw > 0 { mna.n_i[d_raw - 1][start_idx] = -1.0; }
                        if s_raw > 0 { mna.n_i[s_raw - 1][start_idx] = 1.0; }
                    }
                }
                NonlinearDeviceType::Mosfet => {
                    // MOSFET: controlling voltage Vgs, drain current Id
                    // Nodes: [nd, ng, ns, nb]
                    if node_indices.len() >= 3 {
                        let d_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let s_raw = node_indices[2];

                        // N_v: Vgs = Vg - Vs (controlling voltage)
                        if g_raw > 0 { mna.n_v[start_idx][g_raw - 1] = 1.0; }
                        if s_raw > 0 { mna.n_v[start_idx][s_raw - 1] = -1.0; }

                        // N_i: Id flows drain→source
                        if d_raw > 0 { mna.n_i[d_raw - 1][start_idx] = -1.0; }
                        if s_raw > 0 { mna.n_i[s_raw - 1][start_idx] = 1.0; }
                    }
                }
            }
        }

        Ok(mna)
    }

    fn collect_nodes(&mut self, element: &Element) -> Result<(), MnaError> {
        let nodes = match element {
            Element::Resistor { n_plus, n_minus, .. } => vec![n_plus, n_minus],
            Element::Capacitor { n_plus, n_minus, .. } => vec![n_plus, n_minus],
            Element::Inductor { n_plus, n_minus, .. } => vec![n_plus, n_minus],
            Element::VoltageSource { n_plus, n_minus, .. } => vec![n_plus, n_minus],
            Element::CurrentSource { n_plus, n_minus, .. } => vec![n_plus, n_minus],
            Element::Diode { n_plus, n_minus, .. } => vec![n_plus, n_minus],
            Element::Bjt { nc, nb, ne, .. } => vec![nc, nb, ne],
            Element::Jfet { nd, ng, ns, .. } => vec![nd, ng, ns],
            Element::Mosfet { nd, ng, ns, nb, .. } => vec![nd, ng, ns, nb],
            Element::SubcktInstance { name, .. } => {
                return Err(MnaError::TopologyError(
                    format!("subcircuit instance '{}' not supported (expand subcircuits before MNA)", name)
                ));
            }
        };

        for node in nodes {
            if !self.node_map.contains_key(node) {
                self.node_map.insert(node.clone(), self.next_node_idx);
                self.next_node_idx += 1;
            }
        }

        Ok(())
    }

    fn categorize_element(&mut self, element: &Element) -> Result<(), MnaError> {
        match element {
            Element::Resistor { name, n_plus, n_minus, value } => {
                self.elements.push(ElementInfo {
                    element_type: ElementType::Resistor,
                    nodes: vec![
                        self.node_map[n_plus],
                        self.node_map[n_minus],
                    ],
                    value: *value,
                    name: name.clone(),
                    dc_value: None,
                });
            }
            Element::Capacitor { name, n_plus, n_minus, value, .. } => {
                self.elements.push(ElementInfo {
                    element_type: ElementType::Capacitor,
                    nodes: vec![
                        self.node_map[n_plus],
                        self.node_map[n_minus],
                    ],
                    value: *value,
                    name: name.clone(),
                    dc_value: None,
                });
            }
            Element::Inductor { name, n_plus, n_minus, value } => {
                let node_i = self.node_map[n_plus];
                let node_j = self.node_map[n_minus];
                self.elements.push(ElementInfo {
                    element_type: ElementType::Inductor,
                    nodes: vec![node_i, node_j],
                    value: *value,
                    name: name.clone(),
                    dc_value: None,
                });
                // Also add to inductors list for DK kernel companion model
                self.inductors.push(InductorElement {
                    name: name.clone(),
                    node_i,
                    node_j,
                    value: *value,
                });
            }
            Element::VoltageSource { name, n_plus, n_minus, dc, .. } => {
                // Add as an element so it gets stamped (Norton equivalent conductance)
                self.elements.push(ElementInfo {
                    element_type: ElementType::VoltageSource,
                    nodes: vec![
                        self.node_map[n_plus],
                        self.node_map[n_minus],
                    ],
                    value: dc.unwrap_or(0.0),
                    name: name.clone(),
                    dc_value: *dc,
                });
                let start_idx = self.next_node_idx - 1 + self.voltage_sources.len();
                self.voltage_sources.push(VoltageSourceInfo {
                    name: name.clone(),
                    n_plus: n_plus.clone(),
                    n_minus: n_minus.clone(),
                    n_plus_idx: self.node_map[n_plus],
                    n_minus_idx: self.node_map[n_minus],
                    dc_value: dc.unwrap_or(0.0),
                    ext_idx: start_idx,
                });
            }
            Element::CurrentSource { name, n_plus, n_minus, dc } => {
                self.current_sources.push(CurrentSourceInfo {
                    name: name.clone(),
                    n_plus_idx: self.node_map[n_plus],
                    n_minus_idx: self.node_map[n_minus],
                    dc_value: dc.unwrap_or(0.0),
                });
            }
            Element::Diode { name, n_plus, n_minus, .. } => {
                let ni = self.node_map[n_plus];
                let nj = self.node_map[n_minus];
                if ni == 0 && nj == 0 {
                    return Err(MnaError::TopologyError(
                        format!("diode '{}' has both terminals grounded", name)
                    ));
                }
                let start_idx = self.total_dimension;
                self.total_dimension += 1; // Diode is 1-dimensional
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Diode,
                    dimension: 1,
                    start_idx,
                    nodes: vec![n_plus.clone(), n_minus.clone()],
                    node_indices: vec![
                        self.node_map[n_plus],
                        self.node_map[n_minus],
                    ],
                });
            }
            Element::Bjt { name, nc, nb, ne, .. } => {
                let nc_idx = self.node_map[nc];
                let nb_idx = self.node_map[nb];
                let ne_idx = self.node_map[ne];
                if nc_idx == 0 && nb_idx == 0 && ne_idx == 0 {
                    return Err(MnaError::TopologyError(
                        format!("BJT '{}' has all terminals grounded", name)
                    ));
                }
                let start_idx = self.total_dimension;
                self.total_dimension += 2; // BJT is 2-dimensional (Vbe, Vbc)
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Bjt,
                    dimension: 2,
                    start_idx,
                    nodes: vec![nc.clone(), nb.clone(), ne.clone()],
                    node_indices: vec![
                        self.node_map[nc],
                        self.node_map[nb],
                        self.node_map[ne],
                    ],
                });
            }
            Element::Jfet { name, nd, ng, ns, .. } => {
                let nd_idx = self.node_map[nd];
                let ng_idx = self.node_map[ng];
                let ns_idx = self.node_map[ns];
                if nd_idx == 0 && ng_idx == 0 && ns_idx == 0 {
                    return Err(MnaError::TopologyError(
                        format!("JFET '{}' has all terminals grounded", name)
                    ));
                }
                let start_idx = self.total_dimension;
                self.total_dimension += 1; // Simplified: 1D for now
                let _idx = self.nonlinear_devices.len();
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Jfet,
                    dimension: 1,
                    start_idx,
                    nodes: vec![nd.clone(), ng.clone(), ns.clone()],
                    node_indices: vec![
                        self.node_map[nd],
                        self.node_map[ng],
                        self.node_map[ns],
                    ],
                });
            }
            Element::Mosfet { name, nd, ng, ns, nb, .. } => {
                let nd_idx = self.node_map[nd];
                let ng_idx = self.node_map[ng];
                let ns_idx = self.node_map[ns];
                if nd_idx == 0 && ng_idx == 0 && ns_idx == 0 {
                    return Err(MnaError::TopologyError(
                        format!("MOSFET '{}' has all terminals grounded", name)
                    ));
                }
                let start_idx = self.total_dimension;
                self.total_dimension += 1; // Simplified: 1D for now
                let _idx = self.nonlinear_devices.len();
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Mosfet,
                    dimension: 1,
                    start_idx,
                    nodes: vec![nd.clone(), ng.clone(), ns.clone(), nb.clone()],
                    node_indices: vec![
                        self.node_map[nd],
                        self.node_map[ng],
                        self.node_map[ns],
                        self.node_map[nb],
                    ],
                });
            }
            Element::SubcktInstance { name, .. } => {
                return Err(MnaError::TopologyError(
                    format!("subcircuit instance '{}' not supported (expand subcircuits before MNA)", name)
                ));
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::Netlist;

    #[test]
    fn test_mna_rc_circuit() {
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.n, 2); // in, out
        assert_eq!(mna.m, 0); // No nonlinear devices

        // Check resistor stamp (1k = 0.001 S)
        assert!((mna.g[0][0] - 0.001).abs() < 1e-10);
        assert!((mna.g[1][1] - 0.001).abs() < 1e-10);
        assert!((mna.g[0][1] + 0.001).abs() < 1e-10);
        assert!((mna.g[1][0] + 0.001).abs() < 1e-10);

        // Check capacitor stamp
        assert!((mna.c[1][1] - 1e-6).abs() < 1e-15);
    }

    #[test]
    fn test_mna_diode_clipper() {
        let spice = r#"Diode Clipper
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.n, 2); // in, out
        assert_eq!(mna.m, 1); // 1 diode = 1 dimension
        assert_eq!(mna.num_devices, 1);

        // Check N_v: v_d = v_in - v_out
        assert_eq!(mna.n_v[0][0], 1.0);
        assert_eq!(mna.n_v[0][1], -1.0);

        // Check N_i: current injected at in and out
        assert_eq!(mna.n_i[0][0], -1.0);
        assert_eq!(mna.n_i[1][0], 1.0);
    }

    #[test]
    fn test_mna_bjt_dimensions() {
        let spice = r#"Common Emitter
Q1 coll base emit 2N2222
R1 coll vcc 1k
R2 base 0 100k
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        // Nodes: coll, base, emit, vcc (but vcc has no DC path, may be dropped)
        // Actually vcc connects through R1, so all 4 nodes exist
        assert_eq!(mna.n, 4); // coll, base, emit, vcc (minus ground)
        assert_eq!(mna.m, 2); // 1 BJT = 2 dimensions (Vbe, Vbc)
        assert_eq!(mna.num_devices, 1);

        // Find the BJT device
        let bjt = &mna.nonlinear_devices[0];
        assert_eq!(bjt.device_type, NonlinearDeviceType::Bjt);
        assert_eq!(bjt.dimension, 2);
        assert_eq!(bjt.start_idx, 0);

        // Check N_v for BJT
        // Row 0: Vbe = Vb - Ve
        // Row 1: Vbc = Vb - Vc
        let base_idx = bjt.node_indices[1] - 1; // 0-indexed
        let emit_idx = bjt.node_indices[2] - 1;
        let coll_idx = bjt.node_indices[0] - 1;

        assert_eq!(mna.n_v[0][base_idx], 1.0);
        assert_eq!(mna.n_v[0][emit_idx], -1.0);
        assert_eq!(mna.n_v[1][base_idx], 1.0);
        assert_eq!(mna.n_v[1][coll_idx], -1.0);
    }
}
