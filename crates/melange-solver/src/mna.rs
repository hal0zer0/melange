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
    /// Op-amp info (for VCCS stamping)
    pub opamps: Vec<OpampInfo>,
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

/// Op-amp information for VCCS stamping.
///
/// Modeled as a voltage-controlled current source with transconductance
/// Gm = AOL / Rout, plus output conductance Go = 1 / Rout. This stamps
/// directly into the G matrix and does NOT add nonlinear dimensions.
#[derive(Debug, Clone)]
pub struct OpampInfo {
    pub name: String,
    /// Non-inverting input node index (0 = ground)
    pub n_plus_idx: usize,
    /// Inverting input node index (0 = ground)
    pub n_minus_idx: usize,
    /// Output node index (0 = ground)
    pub n_out_idx: usize,
    /// Open-loop gain (default 200,000)
    pub aol: f64,
    /// Output resistance in ohms (default 1)
    pub r_out: f64,
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
            opamps: Vec::new(),
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

    /// Build a discretized system matrix from G and C with inductor companion models.
    ///
    /// Computes `result[i][j] = g_sign * G[i][j] + alpha * C[i][j]` for each element,
    /// then stamps inductor companion conductances with the given `g_sign`.
    ///
    /// - `g_sign = +1`: produces A = G + (2/T)*C  (forward matrix)
    /// - `g_sign = -1`: produces A_neg = (2/T)*C - G  (history matrix)
    #[allow(clippy::needless_range_loop)]
    fn build_discretized_matrix(&self, sample_rate: f64, g_sign: f64) -> Vec<Vec<f64>> {
        if !(sample_rate > 0.0 && sample_rate.is_finite()) {
            eprintln!("WARNING: invalid sample_rate {}, using 44100.0", sample_rate);
            return self.build_discretized_matrix(44100.0, g_sign);
        }
        let t = 1.0 / sample_rate;
        let alpha = 2.0 / t; // 2/T for trapezoidal

        let mut mat = vec![vec![0.0; self.n]; self.n];
        for i in 0..self.n {
            for j in 0..self.n {
                mat[i][j] = g_sign * self.g[i][j] + alpha * self.c[i][j];
            }
        }

        // Inductor companion model conductances: g_eq = T/(2L).
        // In A (g_sign=+1) inductors add +g_eq (like a resistor).
        // In A_neg (g_sign=-1) inductors add -g_eq (opposite sign).
        let g_eq_factor = t / 2.0;
        for ind in &self.inductors {
            let g = g_sign * g_eq_factor / ind.value;
            stamp_conductance_to_ground(&mut mat, ind.node_i, ind.node_j, g);
        }

        mat
    }

    /// Get the A matrix for a given sample rate (trapezoidal discretization).
    ///
    /// A = G + (2/T)*C (includes inductor companion model conductances)
    ///
    /// # Panics
    /// Panics if `sample_rate` is not positive and finite.
    pub fn get_a_matrix(&self, sample_rate: f64) -> Vec<Vec<f64>> {
        self.build_discretized_matrix(sample_rate, 1.0)
    }

    /// Get the A_neg matrix for history term (trapezoidal discretization).
    ///
    /// A_neg = (2/T)*C - G (includes inductor companion model)
    ///
    /// # Panics
    /// Panics if `sample_rate` is not positive and finite.
    pub fn get_a_neg_matrix(&self, sample_rate: f64) -> Vec<Vec<f64>> {
        self.build_discretized_matrix(sample_rate, -1.0)
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

/// Stamp a conductance `g` between two nodes that may be grounded (index 0).
///
/// This is the standard MNA conductance stamp with ground-node handling:
/// - Both nodes non-ground: full 2x2 stamp into the matrix
/// - One node grounded: single diagonal entry
/// - Both grounded: no-op
///
/// Node indices use the MNA convention where 0 = ground (excluded from matrix),
/// and non-zero indices are 1-based (matrix row/col = index - 1).
fn stamp_conductance_to_ground(mat: &mut [Vec<f64>], node_i: usize, node_j: usize, g: f64) {
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

/// Builder for MNA systems.
struct MnaBuilder {
    node_map: HashMap<String, usize>,
    next_node_idx: usize,
    nonlinear_devices: Vec<NonlinearDeviceInfo>,
    voltage_sources: Vec<VoltageSourceInfo>,
    current_sources: Vec<CurrentSourceInfo>,
    inductors: Vec<InductorElement>,
    opamps: Vec<OpampInfo>,
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
    Opamp,
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
            opamps: Vec::new(),
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

        // Resolve op-amp model parameters from netlist .model directives
        for (oa, elem) in self.opamps.iter_mut().zip(
            netlist.elements.iter().filter(|e| matches!(e, Element::Opamp { .. }))
        ) {
            if let Element::Opamp { model, .. } = elem {
                if let Some(m) = netlist.models.iter().find(|m| m.name.eq_ignore_ascii_case(model)) {
                    if m.model_type != "OA" {
                        return Err(MnaError::InvalidParameter(format!(
                            "Op-amp {} references model '{}' with type '{}', expected 'OA'",
                            model, m.name, m.model_type
                        )));
                    }
                    for (key, val) in &m.params {
                        match key.to_ascii_uppercase().as_str() {
                            "AOL" => oa.aol = *val,
                            "ROUT" => oa.r_out = *val,
                            _ => {} // Ignore unknown params (GBW, etc.)
                        }
                    }
                }
            }
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
                matches!(e, Element::Resistor { name, .. } if name.eq_ignore_ascii_case(&pot_dir.resistor_name))
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
        mna.opamps = self.opamps;

        // Stamp linear elements
        for elem in &self.elements {
            match elem.element_type {
                ElementType::Resistor => {
                    if elem.nodes.len() >= 2 && elem.value != 0.0 {
                        let g = 1.0 / elem.value;
                        stamp_conductance_to_ground(&mut mna.g, elem.nodes[0], elem.nodes[1], g);
                    }
                }
                ElementType::Capacitor => {
                    if elem.nodes.len() >= 2 {
                        stamp_conductance_to_ground(&mut mna.c, elem.nodes[0], elem.nodes[1], elem.value);
                    }
                }
                ElementType::Inductor => {
                    // Inductors are handled in DK kernel with companion model.
                    // Stamping happens at kernel creation since we need sample rate.
                }
                ElementType::VoltageSource => {
                    // Norton equivalent: stamp large conductance between nodes.
                    // The current contribution is handled in build_rhs_const.
                    if elem.nodes.len() >= 2 {
                        stamp_conductance_to_ground(&mut mna.g, elem.nodes[0], elem.nodes[1], VS_CONDUCTANCE);
                    }
                }
                _ => {} // Nonlinear and op-amps handled separately
            }
        }

        // Stamp op-amps as VCCS into G matrix
        for oa in &mna.opamps {
            let gm = oa.aol / oa.r_out; // Transconductance
            let go = 1.0 / oa.r_out;    // Output conductance

            let out = oa.n_out_idx;
            let np = oa.n_plus_idx;
            let nm = oa.n_minus_idx;

            if out > 0 {
                let o = out - 1;
                if np > 0 { mna.g[o][np - 1] += gm; }
                if nm > 0 { mna.g[o][nm - 1] -= gm; }
                mna.g[o][o] += go;
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
            Element::Opamp { n_plus, n_minus, n_out, .. } => vec![n_plus, n_minus, n_out],
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
                let node_indices = vec![self.node_map[n_plus], self.node_map[n_minus]];
                if node_indices.iter().all(|&idx| idx == 0) {
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
                    node_indices,
                });
            }
            Element::Bjt { name, nc, nb, ne, .. } => {
                let node_indices = vec![
                    self.node_map[nc],
                    self.node_map[nb],
                    self.node_map[ne],
                ];
                if node_indices.iter().all(|&idx| idx == 0) {
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
                    node_indices,
                });
            }
            Element::Jfet { name, nd, ng, ns, .. } => {
                let node_indices = vec![
                    self.node_map[nd],
                    self.node_map[ng],
                    self.node_map[ns],
                ];
                if node_indices.iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(
                        format!("JFET '{}' has all terminals grounded", name)
                    ));
                }
                let start_idx = self.total_dimension;
                self.total_dimension += 1; // Simplified: 1D for now
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Jfet,
                    dimension: 1,
                    start_idx,
                    nodes: vec![nd.clone(), ng.clone(), ns.clone()],
                    node_indices,
                });
            }
            Element::Mosfet { name, nd, ng, ns, nb, .. } => {
                let node_indices = vec![
                    self.node_map[nd],
                    self.node_map[ng],
                    self.node_map[ns],
                    self.node_map[nb],
                ];
                // Check drain/gate/source (not bulk) for all-grounded
                if node_indices[..3].iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(
                        format!("MOSFET '{}' has all terminals grounded", name)
                    ));
                }
                let start_idx = self.total_dimension;
                self.total_dimension += 1; // Simplified: 1D for now
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Mosfet,
                    dimension: 1,
                    start_idx,
                    nodes: vec![nd.clone(), ng.clone(), ns.clone(), nb.clone()],
                    node_indices,
                });
            }
            Element::Opamp { name, n_plus, n_minus, n_out, .. } => {
                let np_idx = self.node_map[n_plus];
                let nm_idx = self.node_map[n_minus];
                let no_idx = self.node_map[n_out];

                if no_idx == 0 {
                    return Err(MnaError::TopologyError(
                        format!("op-amp '{}' has output connected to ground", name)
                    ));
                }
                if np_idx == 0 && nm_idx == 0 {
                    return Err(MnaError::TopologyError(
                        format!("op-amp '{}' has both inputs grounded", name)
                    ));
                }

                // Op-amps are LINEAR — do NOT add to nonlinear dimension M
                self.opamps.push(OpampInfo {
                    name: name.clone(),
                    n_plus_idx: np_idx,
                    n_minus_idx: nm_idx,
                    n_out_idx: no_idx,
                    aol: 200_000.0,
                    r_out: 1.0,
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

    // ===== Op-amp MNA tests =====

    #[test]
    fn test_mna_opamp_basic() {
        let spice = r#"Opamp Test
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
.model opamp OA(AOL=200000)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.n, 3); // in, inv, out
        assert_eq!(mna.m, 0); // Op-amp is linear
        assert_eq!(mna.num_devices, 0);
        assert_eq!(mna.opamps.len(), 1);
        assert_eq!(mna.opamps[0].aol, 200_000.0);
    }

    #[test]
    fn test_mna_opamp_vccs_stamping() {
        let spice = r#"Opamp VCCS Test
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
.model opamp OA(AOL=200000 ROUT=1)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let inv_idx = *mna.node_map.get("inv").unwrap();
        let out_idx = *mna.node_map.get("out").unwrap();

        let o = out_idx - 1;
        let i = inv_idx - 1;

        // G[out, inv] should have -Gm - g_R2
        let g_r2 = 1.0 / 100_000.0;
        assert!((mna.g[o][i] - (-200_000.0 - g_r2)).abs() < 1e-6,
            "G[out,inv] should be -Gm - g_R2, got {}", mna.g[o][i]);

        // G[out, out] should have Go + g_R2
        let expected_go = 1.0 + g_r2;
        assert!((mna.g[o][o] - expected_go).abs() < 1e-6,
            "G[out,out] should include Go={}, got {}", expected_go, mna.g[o][o]);
    }

    #[test]
    fn test_mna_opamp_output_grounded_error() {
        let spice = r#"Opamp Output Grounded
R1 in inv 10k
U1 inp inv 0 opamp
.model opamp OA(AOL=200000)
"#;
        let result = Netlist::parse(spice).and_then(|n| {
            MnaSystem::from_netlist(&n).map_err(|e| crate::parser::ParseError {
                line: 0,
                message: format!("{}", e),
            })
        });
        assert!(result.is_err(), "Op-amp with grounded output should error");
    }

    #[test]
    fn test_mna_opamp_no_nonlinear_dimensions() {
        let spice = r#"Opamp With Diode
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
D1 out 0 D1N4148
.model opamp OA(AOL=200000)
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.m, 1); // Only diode
        assert_eq!(mna.num_devices, 1);
        assert_eq!(mna.opamps.len(), 1);
    }
}
