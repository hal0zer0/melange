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
    /// Inductor elements for companion model (uncoupled only)
    pub inductors: Vec<InductorElement>,
    /// Coupled inductor pairs for transformer companion model (2-winding only)
    pub coupled_inductors: Vec<CoupledInductorInfo>,
    /// Multi-winding transformer groups (3+ windings on shared core)
    pub transformer_groups: Vec<TransformerGroupInfo>,
    /// Potentiometer info (resolved from .pot directives)
    pub pots: Vec<PotInfo>,
    /// Switch info (resolved from .switch directives)
    pub switches: Vec<SwitchInfo>,
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

/// Coupled inductor pair info for transformer companion model.
///
/// Two inductors L1 and L2 with coupling coefficient k have mutual
/// inductance M = k * sqrt(L1 * L2). The companion model stamps
/// both self-conductances and cross-coupling conductances.
#[derive(Debug, Clone)]
pub struct CoupledInductorInfo {
    pub name: String,
    pub l1_name: String,
    pub l2_name: String,
    pub l1_node_i: usize,
    pub l1_node_j: usize,
    pub l2_node_i: usize,
    pub l2_node_j: usize,
    pub l1_value: f64,
    pub l2_value: f64,
    pub coupling: f64,
}

/// Multi-winding transformer group info.
///
/// Groups 3+ inductors that share a magnetic core (connected via K directives).
/// The companion model uses an NxN inductance matrix and its inverse for
/// admittance stamping, instead of per-pair 2x2 inversions.
#[derive(Debug, Clone)]
pub struct TransformerGroupInfo {
    /// Auto-generated group name (e.g. "xfmr_0")
    pub name: String,
    /// Number of windings in this group
    pub num_windings: usize,
    /// Inductor names in group order
    pub winding_names: Vec<String>,
    /// Positive node index for each winding (1-indexed, 0=ground)
    pub winding_node_i: Vec<usize>,
    /// Negative node index for each winding (1-indexed, 0=ground)
    pub winding_node_j: Vec<usize>,
    /// Self-inductance for each winding
    pub inductances: Vec<f64>,
    /// NxN coupling coefficient matrix (symmetric, diagonal = 1.0)
    pub coupling_matrix: Vec<Vec<f64>>,
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
    Tube,
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

/// Component within a switch directive, resolved to MNA node indices.
#[derive(Debug, Clone)]
pub struct SwitchComponentInfo {
    /// Component name (e.g. "C_hfb")
    pub name: String,
    /// Component type: 'R', 'C', or 'L'
    pub component_type: char,
    /// Node index (0 = ground, 1-indexed for MNA)
    pub node_p: usize,
    /// Node index (0 = ground, 1-indexed for MNA)
    pub node_q: usize,
    /// Nominal value (from netlist element definition)
    pub nominal_value: f64,
}

/// Switch information resolved from .switch directive.
#[derive(Debug, Clone)]
pub struct SwitchInfo {
    /// Components controlled by this switch
    pub components: Vec<SwitchComponentInfo>,
    /// Position values: positions[pos][comp] = value for that position
    pub positions: Vec<Vec<f64>>,
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
            coupled_inductors: Vec::new(),
            transformer_groups: Vec::new(),
            pots: Vec::new(),
            switches: Vec::new(),
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

    /// Stamp a capacitor between two 1-indexed nodes (0 = ground).
    ///
    /// Unlike `stamp_capacitor` which takes 0-indexed node numbers (MNA internal),
    /// this takes netlist-style 1-indexed nodes where 0 means ground. This is
    /// useful for stamping junction capacitances computed from device node indices.
    pub fn stamp_capacitor_raw(&mut self, node_a: usize, node_b: usize, cap: f64) {
        if cap <= 0.0 {
            return;
        }
        match (node_a > 0, node_b > 0) {
            (true, true) => {
                let (a, b) = (node_a - 1, node_b - 1);
                self.c[a][a] += cap;
                self.c[b][b] += cap;
                self.c[a][b] -= cap;
                self.c[b][a] -= cap;
            }
            (true, false) => {
                self.c[node_a - 1][node_a - 1] += cap;
            }
            (false, true) => {
                self.c[node_b - 1][node_b - 1] += cap;
            }
            (false, false) => {}
        }
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

    /// Stamp triode nonlinear matrices.
    ///
    /// Triode is 2D per device:
    /// - Dimension 0: Ip (plate current), controlled by Vgk
    /// - Dimension 1: Ig (grid current), controlled by Vpk (for Jacobian coupling)
    ///
    /// N_v extracts controlling voltages:
    /// - Row start_idx: Vgk = V_grid - V_cathode
    /// - Row start_idx+1: Vpk = V_plate - V_cathode
    ///
    /// N_i injects currents:
    /// - Column start_idx: Ip flows plate→cathode
    /// - Column start_idx+1: Ig flows grid→cathode
    pub fn stamp_triode(
        &mut self,
        start_idx: usize,
        ng: usize,
        np: usize,
        nk: usize,
    ) {
        // Row start_idx: Vgk = V_grid - V_cathode
        self.n_v[start_idx][ng] = 1.0;
        self.n_v[start_idx][nk] = -1.0;

        // Row start_idx+1: Vpk = V_plate - V_cathode
        self.n_v[start_idx + 1][np] = 1.0;
        self.n_v[start_idx + 1][nk] = -1.0;

        // Column start_idx: Ip enters plate, exits cathode
        self.n_i[np][start_idx] = -1.0; // Ip enters plate (current into device)
        self.n_i[nk][start_idx] = 1.0;  // Ip exits cathode

        // Column start_idx+1: Ig enters grid, exits cathode
        self.n_i[ng][start_idx + 1] = -1.0; // Ig enters grid
        self.n_i[nk][start_idx + 1] = 1.0;  // Ig exits cathode
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
            log::warn!("invalid sample_rate {}, using 44100.0", sample_rate);
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

        // Coupled inductor companion model: self + mutual conductances.
        // For two coupled inductors L1, L2 with coupling k:
        //   M = k * sqrt(L1 * L2)
        //   det = L1*L2 - M^2
        //   g_self_1 = (T/2) * L2 / det,  g_self_2 = (T/2) * L1 / det
        //   g_mutual = -(T/2) * M / det
        for ci in &self.coupled_inductors {
            let m = ci.coupling * (ci.l1_value * ci.l2_value).sqrt();
            let det = ci.l1_value * ci.l2_value - m * m;
            let gs1 = g_sign * g_eq_factor * ci.l2_value / det;
            let gs2 = g_sign * g_eq_factor * ci.l1_value / det;
            let gm = g_sign * (-g_eq_factor) * m / det;

            // Self-conductances (stamped like regular inductors)
            stamp_conductance_to_ground(&mut mat, ci.l1_node_i, ci.l1_node_j, gs1);
            stamp_conductance_to_ground(&mut mat, ci.l2_node_i, ci.l2_node_j, gs2);

            // Mutual conductance cross-coupling between L1 and L2 (symmetric)
            stamp_mutual_conductance(&mut mat, ci.l1_node_i, ci.l1_node_j, ci.l2_node_i, ci.l2_node_j, gm);
            stamp_mutual_conductance(&mut mat, ci.l2_node_i, ci.l2_node_j, ci.l1_node_i, ci.l1_node_j, gm);
        }

        // Multi-winding transformer groups: NxN admittance stamping.
        // Build the full inductance matrix, invert it, multiply by T/2,
        // and stamp all self and mutual admittance entries.
        for group in &self.transformer_groups {
            let w = group.num_windings;
            // Build inductance matrix L[i][j] = k[i][j] * sqrt(L_i * L_j)
            let mut l_mat = vec![vec![0.0f64; w]; w];
            for i in 0..w {
                for j in 0..w {
                    l_mat[i][j] = group.coupling_matrix[i][j]
                        * (group.inductances[i] * group.inductances[j]).sqrt();
                }
            }
            // Invert: Y_raw = inv(L)
            let y_raw = invert_small_matrix(&l_mat);
            // Scale by T/2 and apply sign
            let scale = g_sign * g_eq_factor;
            // Stamp admittance entries
            for i in 0..w {
                // Self-conductance Y[i][i]
                let y_self = scale * y_raw[i][i];
                stamp_conductance_to_ground(&mut mat,
                    group.winding_node_i[i], group.winding_node_j[i], y_self);
                // Mutual conductance Y[i][j] for j > i (stamp both directions)
                for j in (i + 1)..w {
                    let y_mut = scale * y_raw[i][j];
                    stamp_mutual_conductance(&mut mat,
                        group.winding_node_i[i], group.winding_node_j[i],
                        group.winding_node_i[j], group.winding_node_j[j], y_mut);
                    stamp_mutual_conductance(&mut mat,
                        group.winding_node_i[j], group.winding_node_j[j],
                        group.winding_node_i[i], group.winding_node_j[i], y_mut);
                }
            }
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

    /// Add parasitic junction capacitances across nonlinear device terminals.
    ///
    /// Stamps a small capacitor across each physical junction of every nonlinear
    /// device. This models real semiconductor junction capacitance and ensures
    /// the C matrix is non-trivial for purely resistive nonlinear circuits,
    /// preventing the trapezoidal-rule A matrix from being singular.
    ///
    /// Junction topology per device type:
    /// - **Diode**: anode-cathode (Cj)
    /// - **BJT**: base-emitter (Cje) + base-collector (Cjc)
    /// - **JFET**: gate-source (Cgs) + gate-drain (Cgd)
    /// - **MOSFET**: gate-source (Cgs) + gate-drain (Cgd)
    /// - **Tube**: grid-cathode (Cgk) + plate-cathode (Cpk)
    ///
    /// Uses [`PARASITIC_CAP`] (10pF) and [`stamp_capacitor_raw`](Self::stamp_capacitor_raw).
    pub fn add_parasitic_caps(&mut self) {
        // Collect junction node pairs first to avoid borrowing self immutably
        // (nonlinear_devices) and mutably (stamp_capacitor_raw) at the same time.
        let mut junctions: Vec<(String, usize, usize)> = Vec::new();

        for dev in &self.nonlinear_devices {
            match dev.device_type {
                NonlinearDeviceType::Diode => {
                    // node_indices: [anode, cathode]
                    junctions.push((dev.name.clone(), dev.node_indices[0], dev.node_indices[1]));
                }
                NonlinearDeviceType::Bjt => {
                    // node_indices: [collector, base, emitter]
                    let (nc, nb, ne) = (dev.node_indices[0], dev.node_indices[1], dev.node_indices[2]);
                    // B-E junction (Cje)
                    junctions.push((dev.name.clone(), nb, ne));
                    // B-C junction (Cjc)
                    junctions.push((dev.name.clone(), nb, nc));
                }
                NonlinearDeviceType::Jfet => {
                    // node_indices: [drain, gate, source]
                    let (nd, ng, ns) = (dev.node_indices[0], dev.node_indices[1], dev.node_indices[2]);
                    // G-S junction (Cgs)
                    junctions.push((dev.name.clone(), ng, ns));
                    // G-D junction (Cgd)
                    junctions.push((dev.name.clone(), ng, nd));
                }
                NonlinearDeviceType::Mosfet => {
                    // node_indices: [drain, gate, source, bulk]
                    let (nd, ng, ns) = (dev.node_indices[0], dev.node_indices[1], dev.node_indices[2]);
                    // G-S junction (Cgs)
                    junctions.push((dev.name.clone(), ng, ns));
                    // G-D junction (Cgd)
                    junctions.push((dev.name.clone(), ng, nd));
                }
                NonlinearDeviceType::Tube => {
                    // node_indices: [grid, plate, cathode]
                    let (ng, np, nk) = (dev.node_indices[0], dev.node_indices[1], dev.node_indices[2]);
                    // Grid-cathode (Cgk)
                    junctions.push((dev.name.clone(), ng, nk));
                    // Plate-cathode (Cpk)
                    junctions.push((dev.name.clone(), np, nk));
                }
            }
        }

        for (name, node_a, node_b) in &junctions {
            self.stamp_capacitor_raw(*node_a, *node_b, PARASITIC_CAP);
            log::debug!(
                "Parasitic cap {}: node({})-node({}) = {:.0e} F",
                name, node_a, node_b, PARASITIC_CAP,
            );
        }
    }
}

/// Norton equivalent conductance for voltage sources [S].
///
/// A large conductance value used to model ideal voltage sources as
/// Norton equivalents (current source + parallel conductance).
pub const VS_CONDUCTANCE: f64 = 1e6;

/// Parasitic junction capacitance for nonlinear device stabilization [F].
///
/// 10pF is representative of small-signal semiconductor junction capacitances
/// (typical Cj = 2-10pF for diodes, Cbc = 2-8pF for BJTs). At audio
/// frequencies this is negligible (10pF @ 20kHz = ~800kOhm impedance) but
/// provides enough reactance for the trapezoidal-rule DK discretization to
/// form a well-conditioned A matrix (2C/T ~ 8.8e-7 at 44.1kHz).
///
/// Caps are stamped *across device junctions* (not node-to-ground) to model
/// physical junction capacitance without introducing artificial ground coupling.
pub const PARASITIC_CAP: f64 = 10e-12;

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

/// Invert a small NxN matrix using Gaussian elimination with partial pivoting.
/// Used for multi-winding transformer inductance matrix inversion.
/// Returns identity matrix as fallback if singular (with log warning).
pub(crate) fn invert_small_matrix(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
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
            log::warn!("Singular inductance matrix in transformer group (pivot {:.2e})", max_val);
            // Return identity as fallback
            let mut result = vec![vec![0.0; n]; n];
            for i in 0..n { result[i][i] = 1.0; }
            return result;
        }
        if max_row != col {
            aug.swap(col, max_row);
        }
        let pivot = aug[col][col];
        for j in col..(2 * n) {
            aug[col][j] /= pivot;
        }
        for row in 0..n {
            if row == col { continue; }
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
    result
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

/// Stamp mutual conductance between two 2-terminal elements.
///
/// For a mutual conductance `g` between element 1 (nodes a, b) and
/// element 2 (nodes c, d), the stamp adds cross-coupling:
///   mat[a][c] += g, mat[b][d] += g, mat[a][d] -= g, mat[b][c] -= g
///
/// Node indices use MNA convention: 0 = ground (excluded from matrix).
fn stamp_mutual_conductance(mat: &mut [Vec<f64>], a: usize, b: usize, c: usize, d: usize, g: f64) {
    // a-c coupling
    if a > 0 && c > 0 { mat[a - 1][c - 1] += g; }
    // b-d coupling
    if b > 0 && d > 0 { mat[b - 1][d - 1] += g; }
    // a-d coupling (negative)
    if a > 0 && d > 0 { mat[a - 1][d - 1] -= g; }
    // b-c coupling (negative)
    if b > 0 && c > 0 { mat[b - 1][c - 1] -= g; }
}

/// Inject a current into the RHS vector at a node, handling ground (index 0).
///
/// Positive current is injected at `node` (node_map convention: 0 = ground).
pub(crate) fn inject_rhs_current(rhs: &mut [f64], node: usize, current: f64) {
    if node > 0 {
        rhs[node - 1] += current;
    }
}

/// Stamp a voltage-controlled current source (VCCS) into the G matrix.
///
/// Current `I = gm * (V_ctrl_p - V_ctrl_n)` flows into `out_p` and out of `out_n`.
/// Node indices use MNA convention: 0 = ground (excluded from matrix).
///
/// G stamps:
///   G[out_p, ctrl_p] += gm
///   G[out_p, ctrl_n] -= gm
///   G[out_n, ctrl_p] -= gm
///   G[out_n, ctrl_n] += gm
fn stamp_vccs(mat: &mut [Vec<f64>], out_p: usize, out_n: usize, ctrl_p: usize, ctrl_n: usize, gm: f64) {
    if out_p > 0 {
        let o = out_p - 1;
        if ctrl_p > 0 { mat[o][ctrl_p - 1] += gm; }
        if ctrl_n > 0 { mat[o][ctrl_n - 1] -= gm; }
    }
    if out_n > 0 {
        let o = out_n - 1;
        if ctrl_p > 0 { mat[o][ctrl_p - 1] -= gm; }
        if ctrl_n > 0 { mat[o][ctrl_n - 1] += gm; }
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
    Vcvs,
    Vccs,
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
            if let Element::Opamp { model, .. } = elem
                && let Some(m) = netlist.models.iter().find(|m| m.name.eq_ignore_ascii_case(model))
            {
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

        // Resolve switch directives
        for sw_dir in &netlist.switches {
            let mut components = Vec::new();
            for comp_name in &sw_dir.component_names {
                // For expanded subcircuit names like "X1.C1", use base name after last dot
                let base = comp_name.rsplit('.').next().unwrap_or(comp_name);
                let first_char = base.chars().next().unwrap_or(' ').to_ascii_uppercase();
                let (node_p, node_q, nominal_value) = match first_char {
                    'R' => {
                        let elem = netlist.elements.iter().find(|e| {
                            matches!(e, Element::Resistor { name, .. } if name.eq_ignore_ascii_case(comp_name))
                        });
                        if let Some(Element::Resistor { n_plus, n_minus, value, .. }) = elem {
                            (self.node_map[n_plus], self.node_map[n_minus], *value)
                        } else {
                            return Err(MnaError::TopologyError(format!(
                                ".switch references component '{}' which was not found", comp_name
                            )));
                        }
                    }
                    'C' => {
                        let elem = netlist.elements.iter().find(|e| {
                            matches!(e, Element::Capacitor { name, .. } if name.eq_ignore_ascii_case(comp_name))
                        });
                        if let Some(Element::Capacitor { n_plus, n_minus, value, .. }) = elem {
                            (self.node_map[n_plus], self.node_map[n_minus], *value)
                        } else {
                            return Err(MnaError::TopologyError(format!(
                                ".switch references component '{}' which was not found", comp_name
                            )));
                        }
                    }
                    'L' => {
                        let elem = netlist.elements.iter().find(|e| {
                            matches!(e, Element::Inductor { name, .. } if name.eq_ignore_ascii_case(comp_name))
                        });
                        if let Some(Element::Inductor { n_plus, n_minus, value, .. }) = elem {
                            (self.node_map[n_plus], self.node_map[n_minus], *value)
                        } else {
                            return Err(MnaError::TopologyError(format!(
                                ".switch references component '{}' which was not found", comp_name
                            )));
                        }
                    }
                    _ => {
                        return Err(MnaError::TopologyError(format!(
                            ".switch component '{}' must start with R, C, or L", comp_name
                        )));
                    }
                };
                components.push(SwitchComponentInfo {
                    name: comp_name.clone(),
                    component_type: first_char,
                    node_p,
                    node_q,
                    nominal_value,
                });
            }
            mna.switches.push(SwitchInfo {
                components,
                positions: sw_dir.positions.clone(),
            });
        }

        // Resolve coupling (K) directives: group inductors into transformer groups.
        // 2-winding pairs (inductors that appear in exactly one K directive each)
        // use the existing CoupledInductorInfo path. Multi-winding groups (3+
        // inductors connected by multiple K directives) use TransformerGroupInfo.
        let mut coupled_inductor_names = std::collections::HashSet::new();

        // Collect all inductor info referenced by K directives
        struct InductorRef {
            name: String,
            node_i: usize,
            node_j: usize,
            value: f64,
        }
        let mut inductor_refs: std::collections::HashMap<String, InductorRef> = std::collections::HashMap::new();
        for coupling in &netlist.couplings {
            for ind_name in [&coupling.inductor1_name, &coupling.inductor2_name] {
                let lower = ind_name.to_ascii_lowercase();
                if inductor_refs.contains_key(&lower) {
                    continue;
                }
                if let Some(Element::Inductor { name, n_plus, n_minus, value, .. }) =
                    netlist.elements.iter().find(|e| {
                        matches!(e, Element::Inductor { name, .. } if name.eq_ignore_ascii_case(ind_name))
                    })
                {
                    // Reject coupled inductors in .switch directives
                    for sw in &netlist.switches {
                        for comp_name in &sw.component_names {
                            if comp_name.eq_ignore_ascii_case(name) {
                                return Err(MnaError::TopologyError(format!(
                                    "Coupled inductor '{}' cannot also be in a .switch directive",
                                    comp_name
                                )));
                            }
                        }
                    }
                    inductor_refs.insert(lower, InductorRef {
                        name: name.clone(),
                        node_i: self.node_map[n_plus],
                        node_j: self.node_map[n_minus],
                        value: *value,
                    });
                }
            }
        }

        // Build a graph of inductor connections via K directives (union-find)
        let ind_names: Vec<String> = inductor_refs.keys().cloned().collect();
        let mut parent: std::collections::HashMap<String, String> = ind_names.iter()
            .map(|n| (n.clone(), n.clone())).collect();
        fn find(parent: &mut std::collections::HashMap<String, String>, x: &str) -> String {
            let p = parent[x].clone();
            if p == x { return p; }
            let root = find(parent, &p);
            parent.insert(x.to_string(), root.clone());
            root
        }
        fn union(parent: &mut std::collections::HashMap<String, String>, a: &str, b: &str) {
            let ra = find(parent, a);
            let rb = find(parent, b);
            if ra != rb {
                parent.insert(ra, rb);
            }
        }
        for coupling in &netlist.couplings {
            let l1 = coupling.inductor1_name.to_ascii_lowercase();
            let l2 = coupling.inductor2_name.to_ascii_lowercase();
            union(&mut parent, &l1, &l2);
        }

        // Group inductors by their root in the union-find
        let mut groups: std::collections::HashMap<String, Vec<String>> = std::collections::HashMap::new();
        for name in &ind_names {
            let root = find(&mut parent, name);
            groups.entry(root).or_default().push(name.clone());
        }

        // Process each group
        for (_root, mut members) in groups {
            // Sort members for deterministic ordering (by original element order)
            members.sort_by_key(|m| {
                netlist.elements.iter().position(|e| {
                    matches!(e, Element::Inductor { name, .. } if name.to_ascii_lowercase() == *m)
                }).unwrap_or(usize::MAX)
            });

            for m in &members {
                coupled_inductor_names.insert(m.clone());
            }

            if members.len() == 2 {
                // 2-winding: use existing CoupledInductorInfo path
                let r1 = &inductor_refs[&members[0]];
                let r2 = &inductor_refs[&members[1]];
                // Find the coupling between these two
                let k_val = netlist.couplings.iter().find(|c| {
                    let a = c.inductor1_name.to_ascii_lowercase();
                    let b = c.inductor2_name.to_ascii_lowercase();
                    (a == members[0] && b == members[1]) || (a == members[1] && b == members[0])
                }).map(|c| c.coupling).unwrap_or(0.0);
                let k_name = netlist.couplings.iter().find(|c| {
                    let a = c.inductor1_name.to_ascii_lowercase();
                    let b = c.inductor2_name.to_ascii_lowercase();
                    (a == members[0] && b == members[1]) || (a == members[1] && b == members[0])
                }).map(|c| c.name.clone()).unwrap_or_default();
                mna.coupled_inductors.push(CoupledInductorInfo {
                    name: k_name,
                    l1_name: r1.name.clone(),
                    l2_name: r2.name.clone(),
                    l1_node_i: r1.node_i,
                    l1_node_j: r1.node_j,
                    l2_node_i: r2.node_i,
                    l2_node_j: r2.node_j,
                    l1_value: r1.value,
                    l2_value: r2.value,
                    coupling: k_val,
                });
            } else {
                // Multi-winding (3+): build NxN inductance matrix and invert.
                // The per-pair approach incorrectly stamps self-conductance once per K
                // directive an inductor appears in, giving wrong effective inductances.
                // The NxN approach computes the correct admittance matrix Y = inv(L).
                let w = members.len();
                let mut coupling_matrix = vec![vec![0.0f64; w]; w];
                for i in 0..w {
                    coupling_matrix[i][i] = 1.0; // Self-coupling = 1.0
                }
                // Fill in coupling coefficients from K directives
                for coupling in &netlist.couplings {
                    let a = coupling.inductor1_name.to_ascii_lowercase();
                    let b = coupling.inductor2_name.to_ascii_lowercase();
                    if let (Some(ia), Some(ib)) = (
                        members.iter().position(|m| *m == a),
                        members.iter().position(|m| *m == b),
                    ) {
                        coupling_matrix[ia][ib] = coupling.coupling;
                        coupling_matrix[ib][ia] = coupling.coupling;
                    }
                }
                let mut winding_node_i = Vec::with_capacity(w);
                let mut winding_node_j = Vec::with_capacity(w);
                let mut inductances = Vec::with_capacity(w);
                let mut winding_names = Vec::with_capacity(w);
                for m in &members {
                    let r = &inductor_refs[m];
                    winding_node_i.push(r.node_i);
                    winding_node_j.push(r.node_j);
                    inductances.push(r.value);
                    winding_names.push(r.name.clone());
                }
                let group_idx = mna.transformer_groups.len();
                mna.transformer_groups.push(TransformerGroupInfo {
                    name: format!("xfmr_{}", group_idx),
                    num_windings: w,
                    winding_names,
                    winding_node_i,
                    winding_node_j,
                    inductances,
                    coupling_matrix,
                });
            }
        }

        // Remove coupled inductors from the uncoupled inductors list
        self.inductors.retain(|ind| !coupled_inductor_names.contains(&ind.name.to_ascii_lowercase()));

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
                ElementType::Vcvs => {
                    // Norton equivalent of VCVS: output conductance + controlled VCCS
                    // G_out = VS_CONDUCTANCE, Gm = gain * G_out
                    if elem.nodes.len() >= 4 {
                        let out_p = elem.nodes[0];
                        let out_n = elem.nodes[1];
                        let ctrl_p = elem.nodes[2];
                        let ctrl_n = elem.nodes[3];
                        let g_out = VS_CONDUCTANCE;
                        let gm = elem.value * g_out;

                        // Stamp output conductance between out+ and out-
                        stamp_conductance_to_ground(&mut mna.g, out_p, out_n, g_out);

                        // Stamp VCCS: Gm from control to output
                        stamp_vccs(&mut mna.g, out_p, out_n, ctrl_p, ctrl_n, gm);
                    }
                }
                ElementType::Vccs => {
                    // Direct VCCS stamp into G matrix
                    if elem.nodes.len() >= 4 {
                        let out_p = elem.nodes[0];
                        let out_n = elem.nodes[1];
                        let ctrl_p = elem.nodes[2];
                        let ctrl_n = elem.nodes[3];
                        let gm = elem.value;

                        stamp_vccs(&mut mna.g, out_p, out_n, ctrl_p, ctrl_n, gm);
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
                    // JFET: 2D — Id (drain current) + Ig (gate current)
                    // Nodes: [nd, ng, ns]
                    if node_indices.len() >= 3 {
                        let d_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let s_raw = node_indices[2];

                        // N_v row 0 (Vgs): +1 at G, -1 at S
                        if g_raw > 0 { mna.n_v[start_idx][g_raw - 1] = 1.0; }
                        if s_raw > 0 { mna.n_v[start_idx][s_raw - 1] = -1.0; }
                        // N_v row 1 (Vds): +1 at D, -1 at S
                        if d_raw > 0 { mna.n_v[start_idx + 1][d_raw - 1] = 1.0; }
                        if s_raw > 0 { mna.n_v[start_idx + 1][s_raw - 1] = -1.0; }

                        // N_i col 0 (Id): -1 at D (extracted), +1 at S (injected)
                        if d_raw > 0 { mna.n_i[d_raw - 1][start_idx] = -1.0; }
                        if s_raw > 0 { mna.n_i[s_raw - 1][start_idx] = 1.0; }
                        // N_i col 1 (Ig): -1 at G (extracted), +1 at S (injected)
                        if g_raw > 0 { mna.n_i[g_raw - 1][start_idx + 1] = -1.0; }
                        if s_raw > 0 { mna.n_i[s_raw - 1][start_idx + 1] = 1.0; }
                    }
                }
                NonlinearDeviceType::Mosfet => {
                    // MOSFET: 2D — Id (drain current) + Ig (gate current = 0, insulated gate)
                    // Nodes: [nd, ng, ns, nb]
                    if node_indices.len() >= 3 {
                        let d_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let s_raw = node_indices[2];

                        // N_v row 0 (Vgs): +1 at G, -1 at S
                        if g_raw > 0 { mna.n_v[start_idx][g_raw - 1] = 1.0; }
                        if s_raw > 0 { mna.n_v[start_idx][s_raw - 1] = -1.0; }
                        // N_v row 1 (Vds): +1 at D, -1 at S
                        if d_raw > 0 { mna.n_v[start_idx + 1][d_raw - 1] = 1.0; }
                        if s_raw > 0 { mna.n_v[start_idx + 1][s_raw - 1] = -1.0; }

                        // N_i col 0 (Id): -1 at D (extracted), +1 at S (injected)
                        if d_raw > 0 { mna.n_i[d_raw - 1][start_idx] = -1.0; }
                        if s_raw > 0 { mna.n_i[s_raw - 1][start_idx] = 1.0; }
                        // N_i col 1 (Ig): effectively zero (insulated gate), but stamp for framework
                        if g_raw > 0 { mna.n_i[g_raw - 1][start_idx + 1] = -1.0; }
                        if s_raw > 0 { mna.n_i[s_raw - 1][start_idx + 1] = 1.0; }
                    }
                }
                NonlinearDeviceType::Tube => {
                    // Triode: 2D — Ip (plate current) + Ig (grid current)
                    // Nodes: [ng, np, nk] (grid, plate, cathode)
                    if node_indices.len() >= 3 {
                        let g_raw = node_indices[0];
                        let p_raw = node_indices[1];
                        let k_raw = node_indices[2];

                        if g_raw > 0 && p_raw > 0 && k_raw > 0 {
                            mna.stamp_triode(start_idx, g_raw - 1, p_raw - 1, k_raw - 1);
                        } else {
                            // Per-terminal ground handling
                            // N_v row 0 (Vgk): +1 at grid, -1 at cathode
                            if g_raw > 0 { mna.n_v[start_idx][g_raw - 1] = 1.0; }
                            if k_raw > 0 { mna.n_v[start_idx][k_raw - 1] = -1.0; }
                            // N_v row 1 (Vpk): +1 at plate, -1 at cathode
                            if p_raw > 0 { mna.n_v[start_idx + 1][p_raw - 1] = 1.0; }
                            if k_raw > 0 { mna.n_v[start_idx + 1][k_raw - 1] = -1.0; }
                            // N_i col 0 (Ip): -1 at plate, +1 at cathode
                            if p_raw > 0 { mna.n_i[p_raw - 1][start_idx] = -1.0; }
                            if k_raw > 0 { mna.n_i[k_raw - 1][start_idx] = 1.0; }
                            // N_i col 1 (Ig): -1 at grid, +1 at cathode
                            if g_raw > 0 { mna.n_i[g_raw - 1][start_idx + 1] = -1.0; }
                            if k_raw > 0 { mna.n_i[k_raw - 1][start_idx + 1] = 1.0; }
                        }
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
            Element::Triode { n_grid, n_plate, n_cathode, .. } => vec![n_grid, n_plate, n_cathode],
            Element::Opamp { n_plus, n_minus, n_out, .. } => vec![n_plus, n_minus, n_out],
            Element::Vcvs { out_p, out_n, ctrl_p, ctrl_n, .. } => vec![out_p, out_n, ctrl_p, ctrl_n],
            Element::Vccs { out_p, out_n, ctrl_p, ctrl_n, .. } => vec![out_p, out_n, ctrl_p, ctrl_n],
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
                self.total_dimension += 2; // 2D: Id(Vgs,Vds) + Ig(Vgs)
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Jfet,
                    dimension: 2,
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
                self.total_dimension += 2; // 2D: Id(Vgs,Vds) + Ig(=0, insulated gate)
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Mosfet,
                    dimension: 2,
                    start_idx,
                    nodes: vec![nd.clone(), ng.clone(), ns.clone(), nb.clone()],
                    node_indices,
                });
            }
            Element::Triode { name, n_grid, n_plate, n_cathode, .. } => {
                let node_indices = vec![
                    self.node_map[n_grid],
                    self.node_map[n_plate],
                    self.node_map[n_cathode],
                ];
                let start_idx = self.total_dimension;
                self.total_dimension += 2; // 2D: plate current + grid current
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Tube,
                    dimension: 2,
                    start_idx,
                    nodes: vec![n_grid.clone(), n_plate.clone(), n_cathode.clone()],
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
            Element::Vcvs { name, out_p, out_n, ctrl_p, ctrl_n, gain } => {
                // VCVS is LINEAR — do NOT add to nonlinear dimension M
                // Stored as ElementInfo for G matrix stamping (Norton equivalent)
                self.elements.push(ElementInfo {
                    element_type: ElementType::Vcvs,
                    nodes: vec![
                        self.node_map[out_p],
                        self.node_map[out_n],
                        self.node_map[ctrl_p],
                        self.node_map[ctrl_n],
                    ],
                    value: *gain,
                    name: name.clone(),
                    dc_value: None,
                });
            }
            Element::Vccs { name, out_p, out_n, ctrl_p, ctrl_n, gm } => {
                // VCCS is LINEAR — do NOT add to nonlinear dimension M
                // Stored as ElementInfo for G matrix stamping
                self.elements.push(ElementInfo {
                    element_type: ElementType::Vccs,
                    nodes: vec![
                        self.node_map[out_p],
                        self.node_map[out_n],
                        self.node_map[ctrl_p],
                        self.node_map[ctrl_n],
                    ],
                    value: *gm,
                    name: name.clone(),
                    dc_value: None,
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

    // ===== VCCS (G element) MNA tests =====

    #[test]
    fn test_mna_vccs_basic() {
        // G1 out 0 in 0 0.01 — current gm*(V_in - 0) flows into out
        let spice = r#"VCCS Test
R1 in 0 1k
R2 out 0 1k
G1 out 0 in 0 0.01
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.m, 0); // VCCS is linear
        assert_eq!(mna.num_devices, 0);

        let in_idx = *mna.node_map.get("in").unwrap();
        let out_idx = *mna.node_map.get("out").unwrap();

        let o = out_idx - 1;
        let i = in_idx - 1;

        // G[out, in] should have +gm = +0.01
        assert!((mna.g[o][i] - 0.01).abs() < 1e-15,
            "G[out,in] should be +gm=0.01, got {}", mna.g[o][i]);

        // G[out, out] should only have resistor conductance (1/1k = 0.001)
        assert!((mna.g[o][o] - 0.001).abs() < 1e-15,
            "G[out,out] should be 0.001, got {}", mna.g[o][o]);
    }

    #[test]
    fn test_mna_vccs_differential() {
        // G1 out 0 inp inn 0.01 — ctrl is differential
        let spice = r#"VCCS Differential
R1 out 0 1k
G1 out 0 inp inn 0.01
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let out_idx = *mna.node_map.get("out").unwrap();
        let inp_idx = *mna.node_map.get("inp").unwrap();
        let inn_idx = *mna.node_map.get("inn").unwrap();

        let o = out_idx - 1;

        // G[out, inp] should have +gm
        assert!((mna.g[o][inp_idx - 1] - 0.01).abs() < 1e-15,
            "G[out,inp] should be +gm, got {}", mna.g[o][inp_idx - 1]);

        // G[out, inn] should have -gm
        assert!((mna.g[o][inn_idx - 1] - (-0.01)).abs() < 1e-15,
            "G[out,inn] should be -gm, got {}", mna.g[o][inn_idx - 1]);
    }

    #[test]
    fn test_mna_vccs_out_n_not_ground() {
        // G1 out_p out_n ctrl 0 0.01 — output negative node is not ground
        let spice = r#"VCCS Non-Ground Output
R1 out_p 0 1k
R2 out_n 0 1k
G1 out_p out_n ctrl 0 0.01
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let op = *mna.node_map.get("out_p").unwrap() - 1;
        let on = *mna.node_map.get("out_n").unwrap() - 1;
        let ctrl = *mna.node_map.get("ctrl").unwrap() - 1;

        // G[out_p, ctrl] += gm
        assert!((mna.g[op][ctrl] - 0.01).abs() < 1e-15);
        // G[out_n, ctrl] -= gm
        assert!((mna.g[on][ctrl] - (-0.01)).abs() < 1e-15);
    }

    // ===== VCVS (E element) MNA tests =====

    #[test]
    fn test_mna_vcvs_basic() {
        // E1 out 0 in 0 10 — voltage gain of 10
        let spice = r#"VCVS Test
R1 in 0 1k
R2 out 0 1k
E1 out 0 in 0 10
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.m, 0); // VCVS is linear
        assert_eq!(mna.num_devices, 0);

        let in_idx = *mna.node_map.get("in").unwrap();
        let out_idx = *mna.node_map.get("out").unwrap();

        let o = out_idx - 1;
        let i = in_idx - 1;

        // VCVS stamps: G_out at output (VS_CONDUCTANCE = 1e6)
        // G[out, out] should have VS_CONDUCTANCE + 1/R2
        let g_r2 = 1.0 / 1000.0;
        let expected = VS_CONDUCTANCE + g_r2;
        assert!((mna.g[o][o] - expected).abs() / expected < 1e-10,
            "G[out,out] should include VS_CONDUCTANCE, got {}", mna.g[o][o]);

        // G[out, in] should have gain * VS_CONDUCTANCE = 10 * 1e6 = 1e7
        let gm = 10.0 * VS_CONDUCTANCE;
        assert!((mna.g[o][i] - gm).abs() / gm < 1e-10,
            "G[out,in] should be gain*VS_CONDUCTANCE={}, got {}", gm, mna.g[o][i]);
    }

    #[test]
    fn test_mna_vcvs_differential_output() {
        // E1 out_p out_n in 0 5 — VCVS with non-ground output negative
        let spice = r#"VCVS Diff Output
R1 out_p 0 1k
R2 out_n 0 1k
E1 out_p out_n in 0 5
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let op = *mna.node_map.get("out_p").unwrap() - 1;
        let on = *mna.node_map.get("out_n").unwrap() - 1;
        let inp = *mna.node_map.get("in").unwrap() - 1;

        let g_out = VS_CONDUCTANCE;
        let gm = 5.0 * g_out;

        // Output conductance: stamp_conductance_to_ground stamps between out_p and out_n
        // G[out_p, out_p] += G_out
        // G[out_n, out_n] += G_out
        // G[out_p, out_n] -= G_out
        // G[out_n, out_p] -= G_out
        let g_r = 1.0 / 1000.0;
        assert!((mna.g[op][op] - (g_out + g_r)).abs() / g_out < 1e-10);
        assert!((mna.g[on][on] - (g_out + g_r)).abs() / g_out < 1e-10);
        assert!((mna.g[op][on] - (-g_out)).abs() / g_out < 1e-10);
        assert!((mna.g[on][op] - (-g_out)).abs() / g_out < 1e-10);

        // VCCS stamp: gm from ctrl to output
        // G[out_p, in] += gm
        assert!((mna.g[op][inp] - gm).abs() / gm < 1e-10);
        // G[out_n, in] -= gm
        assert!((mna.g[on][inp] - (-gm)).abs() / gm < 1e-10);
    }

    #[test]
    fn test_mna_vccs_with_opamp() {
        // VCCS and op-amp coexist in same circuit
        let spice = r#"VCCS With Opamp
R1 in inv 10k
R2 inv out 100k
U1 0 inv out opamp
G1 out2 0 out 0 0.001
R3 out2 0 1k
.model opamp OA(AOL=200000)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.m, 0); // Both linear
        assert_eq!(mna.opamps.len(), 1);

        let out_idx = *mna.node_map.get("out").unwrap();
        let out2_idx = *mna.node_map.get("out2").unwrap();

        let o2 = out2_idx - 1;
        let o = out_idx - 1;

        // G[out2, out] should have VCCS gm = 0.001
        assert!((mna.g[o2][o] - 0.001).abs() < 1e-15,
            "G[out2,out] should be VCCS gm=0.001, got {}", mna.g[o2][o]);
    }

    #[test]
    fn test_mna_vcvs_no_nonlinear_dimensions() {
        // VCVS + diode: only the diode adds nonlinear dimensions
        let spice = r#"VCVS With Diode
R1 in 0 1k
E1 mid 0 in 0 10
D1 mid out D1N4148
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.m, 1); // Only diode
        assert_eq!(mna.num_devices, 1);
    }

    // ===== Parasitic capacitance tests =====

    #[test]
    fn test_parasitic_cap_value() {
        assert!((PARASITIC_CAP - 10e-12).abs() < 1e-25, "PARASITIC_CAP should be 10pF");
    }

    #[test]
    fn test_parasitic_cap_diode_across_junction() {
        // Diode: one cap between anode and cathode (across junction)
        let spice = r#"Diode Parasitic
D1 in out D1N4148
R1 out 0 1k
.model D1N4148 D(IS=1e-15)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

        // C matrix should be all zeros before parasitic caps
        for i in 0..mna.n {
            for j in 0..mna.n {
                assert_eq!(mna.c[i][j], 0.0, "C[{i}][{j}] should be 0 before parasitic caps");
            }
        }

        mna.add_parasitic_caps();

        let anode = *mna.node_map.get("in").unwrap();
        let cathode = *mna.node_map.get("out").unwrap();
        let a = anode - 1; // 0-indexed
        let k = cathode - 1;

        // Diagonal: both nodes get +PARASITIC_CAP
        assert!((mna.c[a][a] - PARASITIC_CAP).abs() < 1e-25,
            "C[anode][anode] should be PARASITIC_CAP, got {}", mna.c[a][a]);
        assert!((mna.c[k][k] - PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][cathode] should be PARASITIC_CAP, got {}", mna.c[k][k]);

        // Off-diagonal: negative (cap between nodes, not to ground)
        assert!((mna.c[a][k] + PARASITIC_CAP).abs() < 1e-25,
            "C[anode][cathode] should be -PARASITIC_CAP, got {}", mna.c[a][k]);
        assert!((mna.c[k][a] + PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][anode] should be -PARASITIC_CAP, got {}", mna.c[k][a]);
    }

    #[test]
    fn test_parasitic_cap_bjt_two_junctions() {
        // BJT: two caps — B-E and B-C
        let spice = r#"BJT Parasitic
Q1 coll base emit 2N2222
R1 coll 0 1k
R2 base 0 100k
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.add_parasitic_caps();

        let nc = *mna.node_map.get("coll").unwrap() - 1;
        let nb = *mna.node_map.get("base").unwrap() - 1;
        let ne = *mna.node_map.get("emit").unwrap() - 1;

        // Base gets caps from both B-E and B-C junctions: 2 * PARASITIC_CAP
        assert!((mna.c[nb][nb] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
            "C[base][base] should be 2*PARASITIC_CAP, got {}", mna.c[nb][nb]);

        // Collector gets cap from B-C junction only
        assert!((mna.c[nc][nc] - PARASITIC_CAP).abs() < 1e-25,
            "C[coll][coll] should be PARASITIC_CAP, got {}", mna.c[nc][nc]);

        // Emitter gets cap from B-E junction only
        assert!((mna.c[ne][ne] - PARASITIC_CAP).abs() < 1e-25,
            "C[emit][emit] should be PARASITIC_CAP, got {}", mna.c[ne][ne]);

        // Off-diagonal: B-E junction
        assert!((mna.c[nb][ne] + PARASITIC_CAP).abs() < 1e-25,
            "C[base][emit] should be -PARASITIC_CAP, got {}", mna.c[nb][ne]);
        assert!((mna.c[ne][nb] + PARASITIC_CAP).abs() < 1e-25,
            "C[emit][base] should be -PARASITIC_CAP, got {}", mna.c[ne][nb]);

        // Off-diagonal: B-C junction
        assert!((mna.c[nb][nc] + PARASITIC_CAP).abs() < 1e-25,
            "C[base][coll] should be -PARASITIC_CAP, got {}", mna.c[nb][nc]);
        assert!((mna.c[nc][nb] + PARASITIC_CAP).abs() < 1e-25,
            "C[coll][base] should be -PARASITIC_CAP, got {}", mna.c[nc][nb]);

        // Collector-Emitter: no direct parasitic cap
        assert!((mna.c[nc][ne]).abs() < 1e-25,
            "C[coll][emit] should be 0, got {}", mna.c[nc][ne]);
    }

    #[test]
    fn test_parasitic_cap_jfet_two_junctions() {
        // JFET: two caps — G-S and G-D
        let spice = r#"JFET Parasitic
J1 drain gate source JN
R1 drain 0 1k
R2 gate 0 1M
.model JN NJ(VTO=-2.0)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.add_parasitic_caps();

        let nd = *mna.node_map.get("drain").unwrap() - 1;
        let ng = *mna.node_map.get("gate").unwrap() - 1;
        let ns = *mna.node_map.get("source").unwrap() - 1;

        // Gate gets caps from both G-S and G-D junctions: 2 * PARASITIC_CAP
        assert!((mna.c[ng][ng] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
            "C[gate][gate] should be 2*PARASITIC_CAP, got {}", mna.c[ng][ng]);

        // Off-diagonal: G-S
        assert!((mna.c[ng][ns] + PARASITIC_CAP).abs() < 1e-25,
            "C[gate][source] should be -PARASITIC_CAP, got {}", mna.c[ng][ns]);

        // Off-diagonal: G-D
        assert!((mna.c[ng][nd] + PARASITIC_CAP).abs() < 1e-25,
            "C[gate][drain] should be -PARASITIC_CAP, got {}", mna.c[ng][nd]);

        // Drain-Source: no direct parasitic cap
        assert!((mna.c[nd][ns]).abs() < 1e-25,
            "C[drain][source] should be 0, got {}", mna.c[nd][ns]);
    }

    #[test]
    fn test_parasitic_cap_mosfet_two_junctions() {
        // MOSFET: two caps — G-S and G-D
        let spice = r#"MOSFET Parasitic
M1 drain gate source source NM1
R1 drain 0 1k
.model NM1 NM(VTO=2.0 KP=0.1)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.add_parasitic_caps();

        let nd = *mna.node_map.get("drain").unwrap() - 1;
        let ng = *mna.node_map.get("gate").unwrap() - 1;
        let ns = *mna.node_map.get("source").unwrap() - 1;

        // Gate gets caps from G-S and G-D: 2 * PARASITIC_CAP
        assert!((mna.c[ng][ng] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
            "C[gate][gate] should be 2*PARASITIC_CAP, got {}", mna.c[ng][ng]);

        // Off-diagonal: G-S
        assert!((mna.c[ng][ns] + PARASITIC_CAP).abs() < 1e-25,
            "C[gate][source] should be -PARASITIC_CAP, got {}", mna.c[ng][ns]);

        // Off-diagonal: G-D
        assert!((mna.c[ng][nd] + PARASITIC_CAP).abs() < 1e-25,
            "C[gate][drain] should be -PARASITIC_CAP, got {}", mna.c[ng][nd]);
    }

    #[test]
    fn test_parasitic_cap_tube_two_junctions() {
        // Tube: two caps — grid-cathode (Cgk) and plate-cathode (Cpk)
        let spice = r#"Tube Parasitic
T1 grid plate cathode 12AX7
R1 plate 0 100k
R2 grid 0 1M
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.add_parasitic_caps();

        let ng = *mna.node_map.get("grid").unwrap() - 1;
        let np = *mna.node_map.get("plate").unwrap() - 1;
        let nk = *mna.node_map.get("cathode").unwrap() - 1;

        // Cathode gets caps from both Cgk and Cpk: 2 * PARASITIC_CAP
        assert!((mna.c[nk][nk] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][cathode] should be 2*PARASITIC_CAP, got {}", mna.c[nk][nk]);

        // Grid gets cap from Cgk only
        assert!((mna.c[ng][ng] - PARASITIC_CAP).abs() < 1e-25,
            "C[grid][grid] should be PARASITIC_CAP, got {}", mna.c[ng][ng]);

        // Plate gets cap from Cpk only
        assert!((mna.c[np][np] - PARASITIC_CAP).abs() < 1e-25,
            "C[plate][plate] should be PARASITIC_CAP, got {}", mna.c[np][np]);

        // Off-diagonal: grid-cathode
        assert!((mna.c[ng][nk] + PARASITIC_CAP).abs() < 1e-25,
            "C[grid][cathode] should be -PARASITIC_CAP, got {}", mna.c[ng][nk]);
        assert!((mna.c[nk][ng] + PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][grid] should be -PARASITIC_CAP, got {}", mna.c[nk][ng]);

        // Off-diagonal: plate-cathode
        assert!((mna.c[np][nk] + PARASITIC_CAP).abs() < 1e-25,
            "C[plate][cathode] should be -PARASITIC_CAP, got {}", mna.c[np][nk]);
        assert!((mna.c[nk][np] + PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][plate] should be -PARASITIC_CAP, got {}", mna.c[nk][np]);

        // Grid-Plate: no direct parasitic cap
        assert!((mna.c[ng][np]).abs() < 1e-25,
            "C[grid][plate] should be 0, got {}", mna.c[ng][np]);
    }

    #[test]
    fn test_parasitic_cap_count_per_device() {
        // Verify correct number of junction caps: 1 for diode, 2 for BJT
        // All device terminals are non-ground so every junction produces off-diagonal entries
        let spice = r#"Mixed Devices
D1 in mid D1N4148
Q1 out mid emit 2N2222
R1 out 0 1k
R2 emit 0 100
.model D1N4148 D(IS=1e-15)
.model 2N2222 NPN(IS=1e-15 BF=200)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.add_parasitic_caps();

        // Count total nonzero off-diagonal C entries (each junction adds 2 off-diag entries)
        let mut off_diag_count = 0;
        for i in 0..mna.n {
            for j in 0..mna.n {
                if i != j && mna.c[i][j].abs() > 1e-25 {
                    off_diag_count += 1;
                }
            }
        }
        // Diode: 1 junction (anode-cathode) = 2 off-diagonal entries
        // BJT: 2 junctions (B-E + B-C, all non-ground) = 4 off-diagonal entries
        // Total: 6 off-diagonal entries
        assert_eq!(off_diag_count, 6,
            "Expected 6 off-diagonal C entries (1 diode junction + 2 BJT junctions), got {off_diag_count}");
    }
}
