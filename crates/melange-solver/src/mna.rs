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
///
/// When voltage sources or VCVS elements are present, the system is augmented
/// with extra unknowns for the source currents. The augmented dimension is
///   n_aug = n + num_vs + num_vcvs
/// All matrices (G, C, N_v, N_i) are expanded to n_aug × n_aug.
#[derive(Debug, Clone)]
pub struct MnaSystem {
    /// Number of circuit nodes (excluding ground)
    pub n: usize,
    /// Augmented system dimension: n + num_vs + num_vcvs.
    /// Equal to n when no voltage sources or VCVS elements are present.
    pub n_aug: usize,
    /// Total voltage dimension (sum of device dimensions)
    pub m: usize,
    /// Number of physical nonlinear devices
    pub num_devices: usize,
    /// Conductance matrix (n_aug × n_aug — includes augmented VS/VCVS rows)
    pub g: Vec<Vec<f64>>,
    /// Capacitance matrix (n_aug × n_aug — augmented rows are all zeros)
    pub c: Vec<Vec<f64>>,
    /// Nonlinear voltage extraction matrix (m × n_aug)
    /// Row i maps node voltages to controlling voltage i
    pub n_v: Vec<Vec<f64>>,
    /// Nonlinear current injection matrix (n_aug × m)
    /// Column j maps nonlinear current j to node currents
    pub n_i: Vec<Vec<f64>>,
    /// Node name to index mapping (0 = ground, not stored)
    pub node_map: HashMap<String, usize>,
    /// Nonlinear device info
    pub nonlinear_devices: Vec<NonlinearDeviceInfo>,
    /// Voltage source info (for augmented MNA)
    pub voltage_sources: Vec<VoltageSourceInfo>,
    /// VCVS augmented row info (one entry per VCVS element, in element order)
    pub vcvs_sources: Vec<VcvsAugInfo>,
    /// Current source contributions to RHS
    pub current_sources: Vec<CurrentSourceInfo>,
    /// Inductor elements for companion model (uncoupled only)
    pub inductors: Vec<InductorElement>,
    /// Coupled inductor pairs for transformer companion model (2-winding only)
    pub coupled_inductors: Vec<CoupledInductorInfo>,
    /// Multi-winding transformer groups (3+ windings on shared core)
    pub transformer_groups: Vec<TransformerGroupInfo>,
    /// Ideal transformer couplings (decomposed from large tightly-coupled groups).
    /// Each coupling enforces V_sec = n * V_pri algebraically via augmented MNA.
    pub ideal_transformers: Vec<IdealTransformerCoupling>,
    /// Potentiometer info (resolved from .pot directives)
    pub pots: Vec<PotInfo>,
    /// Switch info (resolved from .switch directives)
    pub switches: Vec<SwitchInfo>,
    /// Op-amp info (for VCCS stamping)
    pub opamps: Vec<OpampInfo>,
    /// Internal nodes for parasitic BJTs (RB/RC/RE in transient MNA).
    /// Empty when no parasitic BJTs are present or all are forward-active.
    pub bjt_internal_nodes: Vec<BjtTransientInternalNodes>,
}

/// Augmented-MNA extra row/column info for a VCVS element.
///
/// Each VCVS adds one extra unknown j_vs (the source current) and one
/// algebraic constraint row: V_out+ - V_out- = gain*(V_ctrl+ - V_ctrl-).
#[derive(Debug, Clone)]
pub struct VcvsAugInfo {
    /// 0-based index within VCVS sources (extra row k = n + num_vs + vcvs_idx)
    pub aug_idx: usize,
}

/// Ideal transformer coupling from decomposition of large tightly-coupled inductors.
///
/// Enforces V(sec_p) - V(sec_n) = turns_ratio * (V(pri_p) - V(pri_n)) as an
/// algebraic constraint via augmented MNA. Also injects reflected current at the
/// primary: I_pri = turns_ratio * I_sec (power conservation).
///
/// Created when a transformer group has max(L) > 1H and max(k) > 0.8.
/// The original coupled inductors are replaced by:
/// - One magnetizing inductance (L_ref, added to uncoupled inductors)
/// - One IdealTransformerCoupling per non-reference winding
#[derive(Debug, Clone)]
pub struct IdealTransformerCoupling {
    pub name: String,
    /// Reference (primary) winding positive node (1-indexed, 0=ground)
    pub pri_node_p: usize,
    /// Reference (primary) winding negative node
    pub pri_node_n: usize,
    /// Secondary winding positive node
    pub sec_node_p: usize,
    /// Secondary winding negative node
    pub sec_node_n: usize,
    /// Turns ratio: V_sec = turns_ratio * V_pri
    pub turns_ratio: f64,
}

/// Threshold: transformer groups with max(L) above this use ideal decomposition.
/// Currently disabled (1e30) — ideal transformer decomposition creates algebraic loops
/// in circuits with global feedback through transformers (e.g. Pultec NFB).
/// The DK method requires at least one sample of reactive delay in every feedback loop.
const IDEAL_XFMR_L_THRESHOLD: f64 = 1e30;
/// Threshold: transformer groups with max(k) above this use ideal decomposition.
const IDEAL_XFMR_K_THRESHOLD: f64 = 0.8;

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
    /// Forward-active BJT (1D): Vbc is always reverse-biased, so only Vbe→Ic is tracked.
    /// Ib = Ic/BF is folded into N_i stamping. Reduces M by 1 per BJT.
    BjtForwardActive,
    Jfet,
    Mosfet,
    Tube,
}

/// Internal node indices for a parasitic BJT in the transient MNA system.
///
/// When a BJT has non-zero RB/RC/RE, the MNA is expanded with internal nodes
/// (basePrime, collectorPrime, emitterPrime). N_v/N_i reference these internal
/// nodes, and parasitic conductances (1/R) are stamped in G between external
/// and internal nodes. This eliminates the inner 2D NR loop per device.
#[derive(Debug, Clone)]
pub struct BjtTransientInternalNodes {
    /// Device name (e.g. "Q1")
    pub device_name: String,
    /// M-dimension start index for this BJT
    pub start_idx: usize,
    /// 0-indexed internal base node (None if RB=0, uses external)
    pub int_base: Option<usize>,
    /// 0-indexed internal collector node (None if RC=0, uses external)
    pub int_collector: Option<usize>,
    /// 0-indexed internal emitter node (None if RE=0, uses external)
    pub int_emitter: Option<usize>,
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

/// Result of building augmented G/C matrices with inductor branch variables.
///
/// Each inductor winding adds an extra variable (branch current) in the system.
/// The inductance L appears in the C matrix diagonal. The total dimension is
/// `n_nodal = n_aug + n_inductor_vars`.
#[derive(Debug, Clone)]
pub struct AugmentedMatrices {
    /// Augmented G matrix (n_nodal x n_nodal)
    pub g: Vec<Vec<f64>>,
    /// Augmented C matrix (n_nodal x n_nodal)
    pub c: Vec<Vec<f64>>,
    /// Total nodal dimension = n_aug + n_inductor_vars
    pub n_nodal: usize,
    /// Number of inductor branch variables added
    pub n_inductor_vars: usize,
}

impl MnaSystem {
    /// Create a new empty MNA system.
    ///
    /// Matrices are sized at `n × n`; the builder expands them to `n_aug × n_aug`
    /// after counting voltage sources and VCVS elements.
    pub fn new(n: usize, m: usize, num_devices: usize, num_vs: usize) -> Self {
        Self {
            n,
            n_aug: n, // Will be expanded by the builder
            m,
            num_devices,
            g: vec![vec![0.0; n]; n],
            c: vec![vec![0.0; n]; n],
            n_v: vec![vec![0.0; n]; m],
            n_i: vec![vec![0.0; m]; n],
            node_map: HashMap::new(),
            nonlinear_devices: Vec::new(),
            voltage_sources: Vec::with_capacity(num_vs),
            vcvs_sources: Vec::new(),
            current_sources: Vec::new(),
            inductors: Vec::new(),
            coupled_inductors: Vec::new(),
            transformer_groups: Vec::new(),
            ideal_transformers: Vec::new(),
            pots: Vec::new(),
            switches: Vec::new(),
            opamps: Vec::new(),
            bjt_internal_nodes: Vec::new(),
        }
    }

    /// Assemble MNA matrices from a netlist.
    pub fn from_netlist(netlist: &Netlist) -> Result<Self, MnaError> {
        let builder = MnaBuilder::new();
        builder.build(netlist)
    }

    /// Build MNA system with specified BJTs using forward-active (1D) model.
    ///
    /// BJTs whose names are in `forward_active` are modeled as 1D (Vbe→Ic only),
    /// reducing M by 1 per forward-active BJT. Used after DC OP analysis confirms
    /// Vbc is deeply reverse-biased.
    pub fn from_netlist_forward_active(
        netlist: &Netlist,
        forward_active: &std::collections::HashSet<String>,
    ) -> Result<Self, MnaError> {
        let mut builder = MnaBuilder::new();
        builder.forward_active_bjts = forward_active.clone();
        builder.build(netlist)
    }

    /// Stamp a resistor between two nodes.
    ///
    /// G[i,i] += g, G[j,j] += g, G[i,j] -= g, G[j,i] -= g
    /// Check if this MNA system has any inductors (uncoupled, coupled pairs, or transformer groups).
    pub fn has_inductors(&self) -> bool {
        !self.inductors.is_empty()
            || !self.coupled_inductors.is_empty()
            || !self.transformer_groups.is_empty()
    }

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

    /// Stamp junction capacitances from device model parameters into the C matrix.
    ///
    /// Reads cap fields from each `DeviceSlot`'s params and stamps them across
    /// the appropriate device junctions. Only stamps non-zero values.
    ///
    /// Must be called BEFORE building the DK kernel (so caps are included in A = G + 2C/T).
    ///
    /// Device slots must be in the same order as `self.nonlinear_devices`.
    pub fn stamp_device_junction_caps(&mut self, device_slots: &[crate::codegen::ir::DeviceSlot]) {
        use crate::codegen::ir::DeviceParams;

        // Collect (node_a, node_b, cap) tuples first to avoid borrow conflict
        // (self.nonlinear_devices borrowed immutably, self.stamp_capacitor_raw borrows mutably).
        let mut caps: Vec<(usize, usize, f64)> = Vec::new();

        for (dev_info, slot) in self.nonlinear_devices.iter().zip(device_slots.iter()) {
            match &slot.params {
                DeviceParams::Tube(p) => {
                    // node_indices: [grid, plate, cathode]
                    let (ng, np, nk) = (
                        dev_info.node_indices[0],
                        dev_info.node_indices[1],
                        dev_info.node_indices[2],
                    );
                    if p.ccg > 0.0 {
                        caps.push((nk, ng, p.ccg));
                    }
                    if p.cgp > 0.0 {
                        caps.push((ng, np, p.cgp));
                    }
                    if p.ccp > 0.0 {
                        caps.push((nk, np, p.ccp));
                    }
                }
                DeviceParams::Bjt(p) => {
                    // node_indices: [collector, base, emitter] (1-indexed, 0=ground)
                    let (nc, nb, ne) = (
                        dev_info.node_indices[0],
                        dev_info.node_indices[1],
                        dev_info.node_indices[2],
                    );
                    // If internal nodes exist, stamp caps at internal nodes (not external)
                    let int = self.bjt_internal_nodes.iter().find(|n| n.start_idx == slot.start_idx);
                    // CJE: base-emitter junction
                    if p.cje > 0.0 {
                        let cap_b = int.and_then(|n| n.int_base).map(|i| i + 1).unwrap_or(nb);
                        let cap_e = int.and_then(|n| n.int_emitter).map(|i| i + 1).unwrap_or(ne);
                        caps.push((cap_b, cap_e, p.cje));
                    }
                    // CJC: base-collector junction
                    if p.cjc > 0.0 {
                        let cap_b = int.and_then(|n| n.int_base).map(|i| i + 1).unwrap_or(nb);
                        let cap_c = int.and_then(|n| n.int_collector).map(|i| i + 1).unwrap_or(nc);
                        caps.push((cap_b, cap_c, p.cjc));
                    }
                }
                DeviceParams::Jfet(p) => {
                    // node_indices: [drain, gate, source]
                    let (nd, ng, ns) = (
                        dev_info.node_indices[0],
                        dev_info.node_indices[1],
                        dev_info.node_indices[2],
                    );
                    if p.cgs > 0.0 {
                        caps.push((ng, ns, p.cgs));
                    }
                    if p.cgd > 0.0 {
                        caps.push((ng, nd, p.cgd));
                    }
                }
                DeviceParams::Mosfet(p) => {
                    // node_indices: [drain, gate, source, bulk]
                    let (nd, ng, ns) = (
                        dev_info.node_indices[0],
                        dev_info.node_indices[1],
                        dev_info.node_indices[2],
                    );
                    if p.cgs > 0.0 {
                        caps.push((ng, ns, p.cgs));
                    }
                    if p.cgd > 0.0 {
                        caps.push((ng, nd, p.cgd));
                    }
                }
                DeviceParams::Diode(p) => {
                    // node_indices: [anode, cathode]
                    let (na, nc) = (dev_info.node_indices[0], dev_info.node_indices[1]);
                    if p.cjo > 0.0 {
                        caps.push((na, nc, p.cjo));
                    }
                }
            }
        }

        for (node_a, node_b, cap) in &caps {
            self.stamp_capacitor_raw(*node_a, *node_b, *cap);
            log::debug!(
                "Junction cap: node({})-node({}) = {:.2e} F",
                node_a,
                node_b,
                cap,
            );
        }
    }

    /// Expand MNA with internal nodes for parasitic BJTs (RB/RC/RE).
    ///
    /// For each non-forward-active BJT with non-zero parasitic resistances, creates
    /// internal nodes (basePrime, collectorPrime, emitterPrime) in the MNA system.
    /// Stamps parasitic conductances (1/R) in G between external and internal nodes,
    /// and redirects N_v/N_i to reference internal nodes for the intrinsic model.
    ///
    /// This eliminates the inner 2D NR loop per parasitic BJT in generated code —
    /// the DK kernel absorbs the parasitic R into S = A⁻¹ automatically.
    ///
    /// Must be called AFTER the MNA is fully built (all matrices at n_aug dimension)
    /// but BEFORE DK kernel construction.
    pub fn expand_bjt_internal_nodes(&mut self, device_slots: &[crate::codegen::ir::DeviceSlot]) {
        use crate::codegen::ir::{DeviceParams, DeviceType};

        // Collect expansion info first
        struct Expansion {
            start_idx: usize,
            device_name: String,
            ext_c: usize, // 1-indexed external nodes (0=ground)
            ext_b: usize,
            ext_e: usize,
            rb: f64,
            rc: f64,
            re: f64,
        }

        let mut expansions = Vec::new();
        for (_dev_idx, (dev_info, slot)) in self.nonlinear_devices.iter().zip(device_slots.iter()).enumerate() {
            if slot.device_type == DeviceType::Bjt {
                if let DeviceParams::Bjt(bp) = &slot.params {
                    if bp.has_parasitics() && dev_info.node_indices.len() >= 3 {
                        expansions.push(Expansion {
                            start_idx: slot.start_idx,
                            device_name: dev_info.name.clone(),
                            ext_c: dev_info.node_indices[0],
                            ext_b: dev_info.node_indices[1],
                            ext_e: dev_info.node_indices[2],
                            rb: bp.rb,
                            rc: bp.rc,
                            re: bp.re,
                        });
                    }
                }
            }
        }

        if expansions.is_empty() {
            return;
        }

        // Count new internal nodes needed
        let mut num_new = 0usize;
        for exp in &expansions {
            if exp.rb > 0.0 { num_new += 1; }
            if exp.rc > 0.0 { num_new += 1; }
            if exp.re > 0.0 { num_new += 1; }
        }

        let old_n_aug = self.n_aug;
        let new_n_aug = old_n_aug + num_new;

        // Expand G, C matrices: extend existing rows, add new rows
        for row in self.g.iter_mut() {
            row.resize(new_n_aug, 0.0);
        }
        for row in self.c.iter_mut() {
            row.resize(new_n_aug, 0.0);
        }
        for _ in old_n_aug..new_n_aug {
            self.g.push(vec![0.0; new_n_aug]);
            self.c.push(vec![0.0; new_n_aug]);
        }

        // Expand N_v (m × n_aug): extend each row
        for row in self.n_v.iter_mut() {
            row.resize(new_n_aug, 0.0);
        }

        // Expand N_i (n_aug × m): add new zero rows
        for _ in old_n_aug..new_n_aug {
            self.n_i.push(vec![0.0; self.m]);
        }

        // Allocate internal nodes and stamp
        let mut next_idx = old_n_aug; // 0-indexed internal node index

        for exp in &expansions {
            let s = exp.start_idx;

            // Create internal node indices (0-indexed into the matrix)
            let int_b = if exp.rb > 0.0 { let idx = next_idx; next_idx += 1; Some(idx) } else { None };
            let int_c = if exp.rc > 0.0 { let idx = next_idx; next_idx += 1; Some(idx) } else { None };
            let int_e = if exp.re > 0.0 { let idx = next_idx; next_idx += 1; Some(idx) } else { None };

            // External node indices (0-indexed; None if grounded)
            let ext_b_0 = if exp.ext_b > 0 { Some(exp.ext_b - 1) } else { None };
            let ext_c_0 = if exp.ext_c > 0 { Some(exp.ext_c - 1) } else { None };
            let ext_e_0 = if exp.ext_e > 0 { Some(exp.ext_e - 1) } else { None };

            // Stamp parasitic conductances in G: 1/R between external and internal
            if let Some(ib) = int_b {
                let g = 1.0 / exp.rb;
                self.g[ib][ib] += g;
                if let Some(eb) = ext_b_0 {
                    self.g[eb][eb] += g;
                    self.g[eb][ib] -= g;
                    self.g[ib][eb] -= g;
                }
            }
            if let Some(ic) = int_c {
                let g = 1.0 / exp.rc;
                self.g[ic][ic] += g;
                if let Some(ec) = ext_c_0 {
                    self.g[ec][ec] += g;
                    self.g[ec][ic] -= g;
                    self.g[ic][ec] -= g;
                }
            }
            if let Some(ie) = int_e {
                let g = 1.0 / exp.re;
                self.g[ie][ie] += g;
                if let Some(ee) = ext_e_0 {
                    self.g[ee][ee] += g;
                    self.g[ee][ie] -= g;
                    self.g[ie][ee] -= g;
                }
            }

            // Effective nodes for N_v/N_i (internal if parasitic R, else external)
            let eff_b = int_b.or(ext_b_0);
            let eff_c = int_c.or(ext_c_0);
            let eff_e = int_e.or(ext_e_0);

            // Clear existing N_v/N_i entries for this BJT
            for j in 0..new_n_aug { self.n_v[s][j] = 0.0; self.n_v[s + 1][j] = 0.0; }
            for i in 0..new_n_aug { self.n_i[i][s] = 0.0; self.n_i[i][s + 1] = 0.0; }

            // Re-stamp N_v: Vbe_int = V(basePrime) - V(emitterPrime)
            if let Some(b) = eff_b { self.n_v[s][b] = 1.0; }
            if let Some(e) = eff_e { self.n_v[s][e] = -1.0; }
            // N_v: Vbc_int = V(basePrime) - V(collectorPrime)
            if let Some(b) = eff_b { self.n_v[s + 1][b] = 1.0; }
            if let Some(c) = eff_c { self.n_v[s + 1][c] = -1.0; }

            // N_i: Ic enters collectorPrime, exits emitterPrime
            if let Some(c) = eff_c { self.n_i[c][s] = -1.0; }
            if let Some(e) = eff_e { self.n_i[e][s] = 1.0; }
            // N_i: Ib enters basePrime, exits emitterPrime
            if let Some(b) = eff_b { self.n_i[b][s + 1] = -1.0; }
            if let Some(e) = eff_e { self.n_i[e][s + 1] = 1.0; }

            self.bjt_internal_nodes.push(BjtTransientInternalNodes {
                device_name: exp.device_name.clone(),
                start_idx: s,
                int_base: int_b,
                int_collector: int_c,
                int_emitter: int_e,
            });

            log::debug!(
                "BJT {} internal nodes: base={:?} collector={:?} emitter={:?} (RB={} RC={} RE={})",
                exp.device_name, int_b, int_c, int_e, exp.rb, exp.rc, exp.re,
            );
        }

        // Update n_aug only — do NOT increment self.n.
        // self.n stays at the original circuit node count. The internal nodes are
        // appended after n_aug, like inductor branch variables. The A_neg zeroing
        // targets rows self.n..n_aug (VS/VCVS algebraic constraints), which must
        // NOT include the internal nodes (they need trapezoidal history terms).
        self.n_aug = new_n_aug;
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
    pub fn stamp_nonlinear_2terminal(&mut self, device_idx: usize, i: usize, j: usize) {
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
    pub fn stamp_bjt(&mut self, start_idx: usize, nc: usize, nb: usize, ne: usize) {
        // Row start_idx: Vbe = Vb - Ve
        self.n_v[start_idx][nb] = 1.0;
        self.n_v[start_idx][ne] = -1.0;

        // Row start_idx+1: Vbc = Vb - Vc
        self.n_v[start_idx + 1][nb] = 1.0;
        self.n_v[start_idx + 1][nc] = -1.0;

        // Column start_idx: Ic enters collector, exits emitter
        // N_i convention: positive = current entering node from device
        self.n_i[nc][start_idx] = -1.0; // Ic enters collector (current into device = negative)
        self.n_i[ne][start_idx] = 1.0; // Ic exits emitter (current out of device = positive by KCL)

        // Column start_idx+1: Ib enters base, exits emitter
        self.n_i[nb][start_idx + 1] = -1.0; // Ib enters base
        self.n_i[ne][start_idx + 1] = 1.0; // Ib exits emitter (KCL conservation)
    }

    /// Stamp forward-active BJT nonlinear matrices (1D).
    ///
    /// Only tracks Vbe→Ic. Base current Ib = Ic/BF is folded into N_i,
    /// so KCL at all three terminals is satisfied with a single NR dimension.
    ///
    /// N_v: single row extracting Vbe = Vb - Ve
    /// N_i: single column with Ic at collector, Ic/BF at base, (Ic + Ic/BF) at emitter
    pub fn stamp_bjt_forward_active(
        &mut self,
        start_idx: usize,
        nc: usize,
        nb: usize,
        ne: usize,
        beta_f: f64,
    ) {
        // N_v: extract Vbe = Vb - Ve
        self.n_v[start_idx][nb] = 1.0;
        self.n_v[start_idx][ne] = -1.0;

        // N_i: single column with Ic + Ib = Ic * (1 + 1/BF) for KCL
        self.n_i[nc][start_idx] = -1.0; // Ic extracted from collector
        self.n_i[nb][start_idx] = -1.0 / beta_f; // Ib = Ic/BF extracted from base
        self.n_i[ne][start_idx] = 1.0 + 1.0 / beta_f; // Ic + Ib injected into emitter
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
    pub fn stamp_triode(&mut self, start_idx: usize, ng: usize, np: usize, nk: usize) {
        // Row start_idx: Vgk = V_grid - V_cathode
        self.n_v[start_idx][ng] = 1.0;
        self.n_v[start_idx][nk] = -1.0;

        // Row start_idx+1: Vpk = V_plate - V_cathode
        self.n_v[start_idx + 1][np] = 1.0;
        self.n_v[start_idx + 1][nk] = -1.0;

        // Column start_idx: Ip enters plate, exits cathode
        self.n_i[np][start_idx] = -1.0; // Ip enters plate (current into device)
        self.n_i[nk][start_idx] = 1.0; // Ip exits cathode

        // Column start_idx+1: Ig enters grid, exits cathode
        self.n_i[ng][start_idx + 1] = -1.0; // Ig enters grid
        self.n_i[nk][start_idx + 1] = 1.0; // Ig exits cathode
    }

    /// Build a discretized system matrix from G and C with inductor companion models.
    ///
    /// Computes `result[i][j] = g_sign * G[i][j] + alpha * C[i][j]` for each element,
    /// then stamps inductor companion conductances with the given `g_sign`.
    ///
    /// - `g_sign = +1`: produces A = G + (2/T)*C  (forward matrix)
    /// - `g_sign = -1`: produces A_neg = (2/T)*C - G  (history matrix)
    ///
    /// For augmented MNA (voltage sources/VCVS present), rows n..n_aug-1 are algebraic
    /// constraints with no capacitance. In A_neg (g_sign < 0), those rows must be ALL
    /// ZEROS because there is no trapezoidal history for algebraic constraints.
    #[allow(clippy::needless_range_loop)]
    fn build_discretized_matrix(&self, sample_rate: f64, g_sign: f64) -> Vec<Vec<f64>> {
        if !(sample_rate > 0.0 && sample_rate.is_finite()) {
            log::warn!("invalid sample_rate {}, using 44100.0", sample_rate);
            return self.build_discretized_matrix(44100.0, g_sign);
        }
        let t = 1.0 / sample_rate;
        let alpha = 2.0 / t; // 2/T for trapezoidal
        let n_aug = self.n_aug;

        let mut mat = vec![vec![0.0; n_aug]; n_aug];
        for i in 0..n_aug {
            for j in 0..n_aug {
                mat[i][j] = g_sign * self.g[i][j] + alpha * self.c[i][j];
            }
        }

        // Zero A_neg rows for algebraic constraints (VS/VCVS/ideal transformers).
        // These rows have no capacitance — no trapezoidal history term.
        // Important: do NOT zero internal BJT node rows (they are physical nodes
        // with capacitance and need trapezoidal history).
        if g_sign < 0.0 {
            // VS rows: at indices n + vs.ext_idx
            for vs in &self.voltage_sources {
                let row = self.n + vs.ext_idx;
                if row < n_aug {
                    for j in 0..n_aug {
                        mat[row][j] = 0.0;
                    }
                }
            }
            // VCVS rows: at indices n + num_vs + vcvs_idx
            let num_vs = self.voltage_sources.len();
            for (vcvs_idx, _) in self.vcvs_sources.iter().enumerate() {
                let row = self.n + num_vs + vcvs_idx;
                if row < n_aug {
                    for j in 0..n_aug {
                        mat[row][j] = 0.0;
                    }
                }
            }
            // Ideal transformer rows: at indices n + num_vs + num_vcvs + xfmr_idx
            let num_vcvs = self.vcvs_sources.len();
            for (xfmr_idx, _) in self.ideal_transformers.iter().enumerate() {
                let row = self.n + num_vs + num_vcvs + xfmr_idx;
                if row < n_aug {
                    for j in 0..n_aug {
                        mat[row][j] = 0.0;
                    }
                }
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
            stamp_mutual_conductance(
                &mut mat,
                ci.l1_node_i,
                ci.l1_node_j,
                ci.l2_node_i,
                ci.l2_node_j,
                gm,
            );
            stamp_mutual_conductance(
                &mut mat,
                ci.l2_node_i,
                ci.l2_node_j,
                ci.l1_node_i,
                ci.l1_node_j,
                gm,
            );
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
                stamp_conductance_to_ground(
                    &mut mat,
                    group.winding_node_i[i],
                    group.winding_node_j[i],
                    y_self,
                );
                // Mutual conductance Y[i][j] for j > i (stamp both directions)
                for j in (i + 1)..w {
                    let y_mut = scale * y_raw[i][j];
                    stamp_mutual_conductance(
                        &mut mat,
                        group.winding_node_i[i],
                        group.winding_node_j[i],
                        group.winding_node_i[j],
                        group.winding_node_j[j],
                        y_mut,
                    );
                    stamp_mutual_conductance(
                        &mut mat,
                        group.winding_node_i[j],
                        group.winding_node_j[j],
                        group.winding_node_i[i],
                        group.winding_node_j[i],
                        y_mut,
                    );
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
                NonlinearDeviceType::Bjt | NonlinearDeviceType::BjtForwardActive => {
                    // node_indices: [collector, base, emitter]
                    let (nc, nb, ne) = (
                        dev.node_indices[0],
                        dev.node_indices[1],
                        dev.node_indices[2],
                    );
                    // B-E junction (Cje)
                    junctions.push((dev.name.clone(), nb, ne));
                    // B-C junction (Cjc) — still present even for forward-active (linear cap)
                    junctions.push((dev.name.clone(), nb, nc));
                }
                NonlinearDeviceType::Jfet => {
                    // node_indices: [drain, gate, source]
                    let (nd, ng, ns) = (
                        dev.node_indices[0],
                        dev.node_indices[1],
                        dev.node_indices[2],
                    );
                    // G-S junction (Cgs)
                    junctions.push((dev.name.clone(), ng, ns));
                    // G-D junction (Cgd)
                    junctions.push((dev.name.clone(), ng, nd));
                }
                NonlinearDeviceType::Mosfet => {
                    // node_indices: [drain, gate, source, bulk]
                    let (nd, ng, ns) = (
                        dev.node_indices[0],
                        dev.node_indices[1],
                        dev.node_indices[2],
                    );
                    // G-S junction (Cgs)
                    junctions.push((dev.name.clone(), ng, ns));
                    // G-D junction (Cgd)
                    junctions.push((dev.name.clone(), ng, nd));
                }
                NonlinearDeviceType::Tube => {
                    // node_indices: [grid, plate, cathode]
                    let (ng, np, nk) = (
                        dev.node_indices[0],
                        dev.node_indices[1],
                        dev.node_indices[2],
                    );
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
                name,
                node_a,
                node_b,
                PARASITIC_CAP,
            );
        }
    }

    /// Build augmented G and C matrices with inductor branch variables.
    ///
    /// Copies `self.g` / `self.c` into `n_nodal x n_nodal` matrices, then stamps
    /// KCL/KVL/inductance for all inductor types (uncoupled, coupled pairs, and
    /// transformer groups).
    ///
    /// **Does NOT add Gmin** — that is caller-specific (e.g. NodalSolver adds it,
    /// DK kernel does not).
    ///
    /// Returns `AugmentedMatrices` with the expanded G, C, total dimension, and
    /// the number of inductor variables added.
    pub fn build_augmented_matrices(&self) -> AugmentedMatrices {
        let n_aug = self.n_aug;

        // Count inductor winding variables
        let n_uncoupled = self.inductors.len();
        let n_coupled_windings: usize = self.coupled_inductors.len() * 2;
        let n_xfmr_windings: usize = self.transformer_groups.iter().map(|g| g.num_windings).sum();
        let n_inductor_vars = n_uncoupled + n_coupled_windings + n_xfmr_windings;
        let n_nodal = n_aug + n_inductor_vars;

        // Copy base MNA matrices into expanded dimension
        let mut g_nod = vec![vec![0.0; n_nodal]; n_nodal];
        let mut c_nod = vec![vec![0.0; n_nodal]; n_nodal];
        for i in 0..n_aug {
            for j in 0..n_aug {
                g_nod[i][j] = self.g[i][j];
                c_nod[i][j] = self.c[i][j];
            }
        }

        // Stamp inductor augmented variables
        let mut var_idx = n_aug;

        // Uncoupled inductors: 1 variable each
        for ind in &self.inductors {
            let k = var_idx;
            let ni = ind.node_i; // 1-indexed, 0 = ground
            let nj = ind.node_j;

            // KCL: j_L enters node_i, exits node_j
            if ni > 0 {
                g_nod[ni - 1][k] += 1.0;
            }
            if nj > 0 {
                g_nod[nj - 1][k] -= 1.0;
            }
            // KVL row: -V_i + V_j (= -L * dj_L/dt, with L in C)
            if ni > 0 {
                g_nod[k][ni - 1] -= 1.0;
            }
            if nj > 0 {
                g_nod[k][nj - 1] += 1.0;
            }
            // Self-inductance in C matrix
            c_nod[k][k] = ind.value;

            var_idx += 1;
        }

        // Coupled inductor pairs: 2 variables each
        for ci in &self.coupled_inductors {
            let k1 = var_idx;
            let k2 = var_idx + 1;

            // Winding 1 KCL/KVL
            if ci.l1_node_i > 0 {
                g_nod[ci.l1_node_i - 1][k1] += 1.0;
            }
            if ci.l1_node_j > 0 {
                g_nod[ci.l1_node_j - 1][k1] -= 1.0;
            }
            if ci.l1_node_i > 0 {
                g_nod[k1][ci.l1_node_i - 1] -= 1.0;
            }
            if ci.l1_node_j > 0 {
                g_nod[k1][ci.l1_node_j - 1] += 1.0;
            }

            // Winding 2 KCL/KVL
            if ci.l2_node_i > 0 {
                g_nod[ci.l2_node_i - 1][k2] += 1.0;
            }
            if ci.l2_node_j > 0 {
                g_nod[ci.l2_node_j - 1][k2] -= 1.0;
            }
            if ci.l2_node_i > 0 {
                g_nod[k2][ci.l2_node_i - 1] -= 1.0;
            }
            if ci.l2_node_j > 0 {
                g_nod[k2][ci.l2_node_j - 1] += 1.0;
            }

            // Self-inductances
            c_nod[k1][k1] = ci.l1_value;
            c_nod[k2][k2] = ci.l2_value;
            // Mutual inductance M = k * sqrt(L1 * L2)
            let m_val = ci.coupling * (ci.l1_value * ci.l2_value).sqrt();
            c_nod[k1][k2] = m_val;
            c_nod[k2][k1] = m_val;

            var_idx += 2;
        }

        // Transformer groups: N variables each
        for group in &self.transformer_groups {
            let w = group.num_windings;
            let base_k = var_idx;

            for widx in 0..w {
                let k = base_k + widx;
                let ni = group.winding_node_i[widx];
                let nj = group.winding_node_j[widx];

                // KCL/KVL stamps for winding
                if ni > 0 {
                    g_nod[ni - 1][k] += 1.0;
                }
                if nj > 0 {
                    g_nod[nj - 1][k] -= 1.0;
                }
                if ni > 0 {
                    g_nod[k][ni - 1] -= 1.0;
                }
                if nj > 0 {
                    g_nod[k][nj - 1] += 1.0;
                }

                // Inductance sub-matrix: L[i][j] = k_ij * sqrt(Li * Lj)
                for widx2 in 0..w {
                    let k2 = base_k + widx2;
                    c_nod[k][k2] = group.coupling_matrix[widx][widx2]
                        * (group.inductances[widx] * group.inductances[widx2]).sqrt();
                }
            }

            var_idx += w;
        }

        AugmentedMatrices {
            g: g_nod,
            c: c_nod,
            n_nodal,
            n_inductor_vars,
        }
    }
}

/// Large conductance for modeling short circuits at DC [S].
///
/// Used only in the DC operating-point solver to stamp inductors as short circuits.
/// NOT used for voltage sources or VCVS — those use augmented MNA (extra variables
/// for source currents, enforcing V_plus - V_minus = V_dc exactly).
pub const DC_SHORT_CONDUCTANCE: f64 = 1e3;

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
            MnaError::InvalidComponentValue {
                component,
                value,
                reason,
            } => {
                write!(
                    f,
                    "MNA error: invalid value for '{}': {} ({})",
                    component, value, reason
                )
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
            log::warn!(
                "Singular inductance matrix in transformer group (pivot {:.2e})",
                max_val
            );
            // Return identity as fallback
            let mut result = vec![vec![0.0; n]; n];
            for i in 0..n {
                result[i][i] = 1.0;
            }
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
    if a > 0 && c > 0 {
        mat[a - 1][c - 1] += g;
    }
    // b-d coupling
    if b > 0 && d > 0 {
        mat[b - 1][d - 1] += g;
    }
    // a-d coupling (negative)
    if a > 0 && d > 0 {
        mat[a - 1][d - 1] -= g;
    }
    // b-c coupling (negative)
    if b > 0 && c > 0 {
        mat[b - 1][c - 1] -= g;
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
fn stamp_vccs(
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
    /// BJT names to model as forward-active (1D instead of 2D)
    forward_active_bjts: std::collections::HashSet<String>,
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
            forward_active_bjts: std::collections::HashSet::new(),
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
            netlist
                .elements
                .iter()
                .filter(|e| matches!(e, Element::Opamp { .. })),
        ) {
            if let Element::Opamp { model, .. } = elem
                && let Some(m) = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model))
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
            if let Some(Element::Resistor {
                n_plus,
                n_minus,
                value,
                ..
            }) = resistor
            {
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
                        if let Some(Element::Resistor {
                            n_plus,
                            n_minus,
                            value,
                            ..
                        }) = elem
                        {
                            (self.node_map[n_plus], self.node_map[n_minus], *value)
                        } else {
                            return Err(MnaError::TopologyError(format!(
                                ".switch references component '{}' which was not found",
                                comp_name
                            )));
                        }
                    }
                    'C' => {
                        let elem = netlist.elements.iter().find(|e| {
                            matches!(e, Element::Capacitor { name, .. } if name.eq_ignore_ascii_case(comp_name))
                        });
                        if let Some(Element::Capacitor {
                            n_plus,
                            n_minus,
                            value,
                            ..
                        }) = elem
                        {
                            (self.node_map[n_plus], self.node_map[n_minus], *value)
                        } else {
                            return Err(MnaError::TopologyError(format!(
                                ".switch references component '{}' which was not found",
                                comp_name
                            )));
                        }
                    }
                    'L' => {
                        let elem = netlist.elements.iter().find(|e| {
                            matches!(e, Element::Inductor { name, .. } if name.eq_ignore_ascii_case(comp_name))
                        });
                        if let Some(Element::Inductor {
                            n_plus,
                            n_minus,
                            value,
                            ..
                        }) = elem
                        {
                            (self.node_map[n_plus], self.node_map[n_minus], *value)
                        } else {
                            return Err(MnaError::TopologyError(format!(
                                ".switch references component '{}' which was not found",
                                comp_name
                            )));
                        }
                    }
                    _ => {
                        return Err(MnaError::TopologyError(format!(
                            ".switch component '{}' must start with R, C, or L",
                            comp_name
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
        let mut inductor_refs: std::collections::HashMap<String, InductorRef> =
            std::collections::HashMap::new();
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
        let mut parent: std::collections::HashMap<String, String> =
            ind_names.iter().map(|n| (n.clone(), n.clone())).collect();
        fn find(parent: &mut std::collections::HashMap<String, String>, x: &str) -> String {
            let p = parent[x].clone();
            if p == x {
                return p;
            }
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
        let mut groups: std::collections::HashMap<String, Vec<String>> =
            std::collections::HashMap::new();
        for name in &ind_names {
            let root = find(&mut parent, name);
            groups.entry(root).or_default().push(name.clone());
        }

        // Track internal nodes added by ideal transformer decomposition.
        // Internal nodes are 1-indexed, starting after the last circuit node.
        let mut next_internal_node = n + 1;

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

            // Check if this group qualifies for ideal transformer decomposition:
            // large inductances + tight coupling → companion model creates ill-conditioning.
            let max_l = members
                .iter()
                .map(|m| inductor_refs[m].value)
                .fold(0.0_f64, f64::max);
            let max_k = netlist
                .couplings
                .iter()
                .filter(|c| {
                    let a = c.inductor1_name.to_ascii_lowercase();
                    let b = c.inductor2_name.to_ascii_lowercase();
                    members.contains(&a) && members.contains(&b)
                })
                .map(|c| c.coupling)
                .fold(0.0_f64, f64::max);

            if max_l > IDEAL_XFMR_L_THRESHOLD && max_k > IDEAL_XFMR_K_THRESHOLD {
                // Decompose into: leakage inductors + ideal transformer couplings + magnetizing inductance.
                //
                // Standard T-model equivalent circuit per winding:
                //   original_node_p ── L_leak ── internal_node ── (ideal xfmr) ── ref internal nodes
                //   original_node_n ─────────────────────────────/
                //
                // The leakage inductor provides the reactive delay needed for the DK
                // method's trapezoidal integration — without it, the ideal transformer
                // creates an algebraic loop that produces positive K diagonals.
                //
                // For each winding i:
                //   - Leakage: L_leak_i = (1 - k_avg_i²) × L_i
                //   - k_avg_i = average coupling of winding i to all other windings
                //
                // The leakage inductor connects from the original positive node to a new
                // internal node. The original negative node is shared. The ideal transformer
                // connects between the internal nodes.

                // Pick reference winding (largest inductance).
                let ref_idx = members
                    .iter()
                    .enumerate()
                    .max_by(|(_, a), (_, b)| {
                        inductor_refs[*a]
                            .value
                            .partial_cmp(&inductor_refs[*b].value)
                            .unwrap()
                    })
                    .map(|(i, _)| i)
                    .unwrap_or(0);
                let ref_ind = &inductor_refs[&members[ref_idx]];
                let l_ref = ref_ind.value;

                // Compute average coupling per winding
                let w = members.len();
                let mut k_avg = vec![0.0f64; w];
                let mut k_count = vec![0usize; w];
                for coupling in &netlist.couplings {
                    let a = coupling.inductor1_name.to_ascii_lowercase();
                    let b = coupling.inductor2_name.to_ascii_lowercase();
                    if let (Some(ia), Some(ib)) = (
                        members.iter().position(|m| *m == a),
                        members.iter().position(|m| *m == b),
                    ) {
                        k_avg[ia] += coupling.coupling;
                        k_count[ia] += 1;
                        k_avg[ib] += coupling.coupling;
                        k_count[ib] += 1;
                    }
                }
                for i in 0..w {
                    if k_count[i] > 0 {
                        k_avg[i] /= k_count[i] as f64;
                    }
                }

                // Create internal nodes for each winding.
                // Track the next available node index across all groups.
                let mut internal_nodes_p = Vec::with_capacity(w);
                for (i, m) in members.iter().enumerate() {
                    let ind = &inductor_refs[m];
                    let k_i = k_avg[i];
                    let l_leak = (1.0 - k_i * k_i) * ind.value;
                    // Minimum leakage: ensure non-zero reactive element for numerical stability
                    let l_leak = l_leak.max(ind.value * 1e-4);

                    // Allocate internal node (1-indexed). Use next_internal_node counter
                    // that starts at n+1 and increments across all groups.
                    let internal_p = next_internal_node;
                    next_internal_node += 1;
                    internal_nodes_p.push(internal_p);

                    // Add leakage inductor: original_node_p → internal_node_p
                    self.inductors.push(InductorElement {
                        name: format!("{}_leak", ind.name),
                        node_i: ind.node_i,
                        node_j: internal_p,
                        value: l_leak,
                    });
                }

                // Add magnetizing inductance between reference winding's internal nodes.
                let ref_internal_p = internal_nodes_p[ref_idx];
                let ref_neg = inductor_refs[&members[ref_idx]].node_j;
                self.inductors.push(InductorElement {
                    name: format!("{}_mag", ref_ind.name),
                    node_i: ref_internal_p,
                    node_j: ref_neg,
                    value: l_ref,
                });

                // For each non-reference winding: add ideal transformer coupling
                // between internal nodes (after leakage inductors).
                for (i, m) in members.iter().enumerate() {
                    if i == ref_idx {
                        continue;
                    }
                    let ind = &inductor_refs[m];
                    let n_turns = (ind.value / l_ref).sqrt();

                    mna.ideal_transformers.push(IdealTransformerCoupling {
                        name: format!("ideal_{}_{}", ref_ind.name, ind.name),
                        pri_node_p: ref_internal_p,
                        pri_node_n: ref_neg,
                        sec_node_p: internal_nodes_p[i],
                        sec_node_n: ind.node_j,
                        turns_ratio: n_turns,
                    });
                }

                log::info!(
                    "Ideal transformer decomposition: {} windings, L_ref={:.3}H, {} couplings, {} internal nodes",
                    members.len(),
                    l_ref,
                    members.len() - 1,
                    w
                );

                // Don't add to coupled_inductors or transformer_groups — replaced by ideal model.
                continue;
            }

            if members.len() == 2 {
                // 2-winding: use existing CoupledInductorInfo path
                let r1 = &inductor_refs[&members[0]];
                let r2 = &inductor_refs[&members[1]];
                // Find the coupling between these two
                let k_val = netlist
                    .couplings
                    .iter()
                    .find(|c| {
                        let a = c.inductor1_name.to_ascii_lowercase();
                        let b = c.inductor2_name.to_ascii_lowercase();
                        (a == members[0] && b == members[1]) || (a == members[1] && b == members[0])
                    })
                    .map(|c| c.coupling)
                    .unwrap_or(0.0);
                let k_name = netlist
                    .couplings
                    .iter()
                    .find(|c| {
                        let a = c.inductor1_name.to_ascii_lowercase();
                        let b = c.inductor2_name.to_ascii_lowercase();
                        (a == members[0] && b == members[1]) || (a == members[1] && b == members[0])
                    })
                    .map(|c| c.name.clone())
                    .unwrap_or_default();
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
                // Validate: check that the inductance matrix is positive definite.
                // A non-PD matrix means the coupling coefficients are physically
                // inconsistent (e.g., k_ab=0.95, k_bc=0.95, k_ac=0.50 is impossible).
                {
                    let mut l_mat = vec![vec![0.0f64; w]; w];
                    for i in 0..w {
                        for j in 0..w {
                            l_mat[i][j] =
                                coupling_matrix[i][j] * (inductances[i] * inductances[j]).sqrt();
                        }
                    }
                    // Check via Cholesky-like: all leading minors must be positive.
                    // For small w (≤8), compute determinant directly.
                    let det = if w == 2 {
                        l_mat[0][0] * l_mat[1][1] - l_mat[0][1] * l_mat[1][0]
                    } else {
                        // Use the invert_small_matrix helper — if it returns near-zero
                        // diagonal entries, the matrix is singular or non-PD.
                        let inv = invert_small_matrix(&l_mat);
                        // Check: all diagonal entries of inv should be positive for PD
                        let min_diag: f64 = inv
                            .iter()
                            .enumerate()
                            .map(|(i, row)| row[i])
                            .fold(f64::INFINITY, f64::min);
                        min_diag // positive means PD
                    };
                    if det <= 0.0 || !det.is_finite() {
                        log::warn!(
                            "Transformer group '{}' ({} windings) has non-positive-definite inductance matrix. \
                             This means the coupling coefficients are physically inconsistent. \
                             Check that all K values are compatible (all windings on the same core \
                             should have similar coupling coefficients).",
                            format!("xfmr_{}", mna.transformer_groups.len()),
                            w
                        );
                    }
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
        self.inductors
            .retain(|ind| !coupled_inductor_names.contains(&ind.name.to_ascii_lowercase()));

        // Expand MNA matrices if ideal transformer decomposition added internal nodes
        let num_internal = next_internal_node - (n + 1);
        if num_internal > 0 {
            let new_n = n + num_internal;
            for row in &mut mna.g {
                row.resize(new_n, 0.0);
            }
            for row in &mut mna.c {
                row.resize(new_n, 0.0);
            }
            for _ in 0..num_internal {
                mna.g.push(vec![0.0; new_n]);
                mna.c.push(vec![0.0; new_n]);
            }
            for row in &mut mna.n_v {
                row.resize(new_n, 0.0);
            }
            for _ in 0..num_internal {
                mna.n_i.push(vec![0.0; mna.m]);
            }
            mna.n = new_n;
        }

        mna.node_map = self.node_map;
        mna.nonlinear_devices = self.nonlinear_devices;
        mna.voltage_sources = self.voltage_sources;
        mna.current_sources = self.current_sources;
        mna.inductors = self.inductors;
        mna.opamps = self.opamps;

        // -----------------------------------------------------------------
        // Expand matrices for augmented MNA (voltage sources + VCVS).
        //
        // Count VCVS elements in element order.
        // Use mna.n (which may have grown due to internal nodes from ideal transformers).
        let n_base = mna.n;
        let num_vs = mna.voltage_sources.len();
        let num_vcvs = self
            .elements
            .iter()
            .filter(|e| matches!(e.element_type, ElementType::Vcvs))
            .count();
        let num_ideal_xfmr = mna.ideal_transformers.len();
        let n_aug = n_base + num_vs + num_vcvs + num_ideal_xfmr;
        mna.n_aug = n_aug;

        // Expand G, C, N_v, N_i to n_aug dimensions (extra rows/cols are zero).
        if n_aug > n_base {
            // Expand each existing row by appending zeros
            for row in &mut mna.g {
                row.resize(n_aug, 0.0);
            }
            for row in &mut mna.c {
                row.resize(n_aug, 0.0);
            }
            for row in &mut mna.n_v {
                row.resize(n_aug, 0.0);
            }
            // Add new zero rows for the augmented variables
            for _ in n_base..n_aug {
                mna.g.push(vec![0.0; n_aug]);
                mna.c.push(vec![0.0; n_aug]);
            }
            // Expand N_i rows by appending zeros (column count stays m)
            for row in &mut mna.n_i {
                // N_i is n×m; we add n_aug-n more zero rows
                let _ = row; // row width is m, not n — no resize needed
            }
            // Add zero rows for augmented variables in N_i
            for _ in n_base..n_aug {
                mna.n_i.push(vec![0.0; mna.m]);
            }
        }

        // Stamp voltage sources with augmented MNA.
        // For VS between n+ and n- with current j_vs (extra unknown at row/col k):
        //   KVL constraint row k: G[k][n+] = +1, G[k][n-] = -1
        //   Current injection col k: G[n+][k] = +1, G[n-][k] = -1
        for vs in &mna.voltage_sources {
            let k = n_base + vs.ext_idx; // augmented row/col index
            let np = vs.n_plus_idx;
            let nm = vs.n_minus_idx;
            // Current injection column: j_vs enters n+, exits n-
            if np > 0 {
                mna.g[np - 1][k] += 1.0;
                mna.g[k][np - 1] += 1.0;
            }
            if nm > 0 {
                mna.g[nm - 1][k] -= 1.0;
                mna.g[k][nm - 1] -= 1.0;
            }
        }

        // Stamp VCVS elements with augmented MNA.
        // For VCVS: V_out+ - V_out- = gain*(V_ctrl+ - V_ctrl-)
        //   Extra unknown j_vcvs at row/col k = n + num_vs + vcvs_idx
        //   Current injection column k: enters out+, exits out-
        //   KVL constraint row k: G[k][out+]=+1, G[k][out-]=-1,
        //                          G[k][ctrl+]=-gain, G[k][ctrl-]=+gain
        let mut vcvs_idx = 0;
        for elem in &self.elements {
            if let ElementType::Vcvs = elem.element_type
                && elem.nodes.len() >= 4
            {
                let k = n_base + num_vs + vcvs_idx;
                let out_p = elem.nodes[0];
                let out_n = elem.nodes[1];
                let ctrl_p = elem.nodes[2];
                let ctrl_n = elem.nodes[3];
                let gain = elem.value;

                // Current injection column: j_vcvs enters out+, exits out-
                if out_p > 0 {
                    mna.g[out_p - 1][k] += 1.0;
                }
                if out_n > 0 {
                    mna.g[out_n - 1][k] -= 1.0;
                }
                // KVL constraint row: V_out+ - V_out- - gain*(V_ctrl+ - V_ctrl-) = 0
                if out_p > 0 {
                    mna.g[k][out_p - 1] += 1.0;
                }
                if out_n > 0 {
                    mna.g[k][out_n - 1] -= 1.0;
                }
                if ctrl_p > 0 {
                    mna.g[k][ctrl_p - 1] -= gain;
                }
                if ctrl_n > 0 {
                    mna.g[k][ctrl_n - 1] += gain;
                }

                mna.vcvs_sources.push(VcvsAugInfo { aug_idx: vcvs_idx });
                vcvs_idx += 1;
            }
        }

        // Stamp ideal transformer couplings with augmented MNA.
        // For ideal transformer with turns ratio n:
        //   V_sec = n * V_pri  (voltage coupling)
        //   I_pri = n * I_sec  (current coupling, power conservation)
        //
        // Extra unknown j (secondary current) at row/col k = n + num_vs + num_vcvs + xfmr_idx.
        // KVL row k: G[k][sec+]=+1, G[k][sec-]=-1, G[k][pri+]=-n, G[k][pri-]=+n
        // Current col k: G[sec+][k]=+1, G[sec-][k]=-1, G[pri+][k]=+n, G[pri-][k]=-n
        for (xi, xfmr) in mna.ideal_transformers.iter().enumerate() {
            let k = n_base + num_vs + num_vcvs + xi;
            let sp = xfmr.sec_node_p;
            let sn = xfmr.sec_node_n;
            let pp = xfmr.pri_node_p;
            let pn = xfmr.pri_node_n;
            let nr = xfmr.turns_ratio;

            // KVL constraint row: V(sec+) - V(sec-) - n*(V(pri+) - V(pri-)) = 0
            if sp > 0 {
                mna.g[k][sp - 1] += 1.0;
            }
            if sn > 0 {
                mna.g[k][sn - 1] -= 1.0;
            }
            if pp > 0 {
                mna.g[k][pp - 1] -= nr;
            }
            if pn > 0 {
                mna.g[k][pn - 1] += nr;
            }

            // Current injection column: j enters sec+, exits sec-; n*j enters pri+, exits pri-
            if sp > 0 {
                mna.g[sp - 1][k] += 1.0;
            }
            if sn > 0 {
                mna.g[sn - 1][k] -= 1.0;
            }
            if pp > 0 {
                mna.g[pp - 1][k] += nr;
            }
            if pn > 0 {
                mna.g[pn - 1][k] -= nr;
            }
        }

        // Stamp linear elements (resistors, capacitors, VCCS; skip VS/VCVS now handled above)
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
                        stamp_conductance_to_ground(
                            &mut mna.c,
                            elem.nodes[0],
                            elem.nodes[1],
                            elem.value,
                        );
                    }
                }
                ElementType::Inductor => {
                    // Inductors are handled in DK kernel with companion model.
                    // Stamping happens at kernel creation since we need sample rate.
                }
                ElementType::VoltageSource => {
                    // Handled above with augmented MNA stamping (not Norton equivalent).
                }
                ElementType::Vcvs => {
                    // Handled above with augmented MNA stamping (not Norton equivalent).
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
            let go = 1.0 / oa.r_out; // Output conductance

            let out = oa.n_out_idx;
            let np = oa.n_plus_idx;
            let nm = oa.n_minus_idx;

            if out > 0 {
                let o = out - 1;
                if np > 0 {
                    mna.g[o][np - 1] += gm;
                }
                if nm > 0 {
                    mna.g[o][nm - 1] -= gm;
                }
                mna.g[o][o] += go;
            }
        }

        // Stamp nonlinear devices (collect indices first to avoid borrow issues)
        let device_info: Vec<_> = mna
            .nonlinear_devices
            .iter()
            .map(|d| {
                (
                    d.device_type,
                    d.dimension,
                    d.start_idx,
                    d.node_indices.clone(),
                    d.name.clone(),
                )
            })
            .collect();

        for (dev_type, _dim, start_idx, node_indices, dev_name) in device_info {
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
                            mna.n_v[start_idx][j] = -1.0; // v_d = 0 - v_j = -v_j
                            mna.n_i[j][start_idx] = 1.0; // Current injected into cathode
                        } else if node_j == 0 && node_i > 0 {
                            let i = node_i - 1;
                            mna.n_v[start_idx][i] = 1.0; // v_d = v_i - 0 = v_i
                            mna.n_i[i][start_idx] = -1.0; // Current extracted from anode
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
                            if b_raw > 0 {
                                mna.n_v[start_idx][b_raw - 1] = 1.0;
                            }
                            if e_raw > 0 {
                                mna.n_v[start_idx][e_raw - 1] = -1.0;
                            }
                            // N_v row 1 (Vbc): +1 at B, -1 at C
                            if b_raw > 0 {
                                mna.n_v[start_idx + 1][b_raw - 1] = 1.0;
                            }
                            if c_raw > 0 {
                                mna.n_v[start_idx + 1][c_raw - 1] = -1.0;
                            }
                            // N_i col 0 (Ic): -1 at C, +1 at E
                            if c_raw > 0 {
                                mna.n_i[c_raw - 1][start_idx] = -1.0;
                            }
                            if e_raw > 0 {
                                mna.n_i[e_raw - 1][start_idx] = 1.0;
                            }
                            // N_i col 1 (Ib): -1 at B, +1 at E
                            if b_raw > 0 {
                                mna.n_i[b_raw - 1][start_idx + 1] = -1.0;
                            }
                            if e_raw > 0 {
                                mna.n_i[e_raw - 1][start_idx + 1] = 1.0;
                            }
                        }
                    }
                }
                NonlinearDeviceType::BjtForwardActive => {
                    if node_indices.len() >= 3 {
                        let c_raw = node_indices[0];
                        let b_raw = node_indices[1];
                        let e_raw = node_indices[2];

                        // Look up BF from the netlist model for BF-scaled N_i
                        let beta_f = netlist
                            .elements
                            .iter()
                            .find_map(|e| {
                                if let crate::parser::Element::Bjt { name: n, model, .. } = e {
                                    if n.eq_ignore_ascii_case(&dev_name) {
                                        netlist
                                            .models
                                            .iter()
                                            .find(|m| m.name.eq_ignore_ascii_case(model))
                                            .and_then(|m| {
                                                m.params
                                                    .iter()
                                                    .find(|(k, _)| k.eq_ignore_ascii_case("BF"))
                                                    .map(|(_, v)| *v)
                                            })
                                    } else {
                                        None
                                    }
                                } else {
                                    None
                                }
                            })
                            .unwrap_or(200.0);

                        if c_raw > 0 && b_raw > 0 && e_raw > 0 {
                            mna.stamp_bjt_forward_active(
                                start_idx,
                                c_raw - 1,
                                b_raw - 1,
                                e_raw - 1,
                                beta_f,
                            );
                        } else {
                            // Per-terminal ground handling for 1D forward-active
                            if b_raw > 0 {
                                mna.n_v[start_idx][b_raw - 1] = 1.0;
                            }
                            if e_raw > 0 {
                                mna.n_v[start_idx][e_raw - 1] = -1.0;
                            }
                            if c_raw > 0 {
                                mna.n_i[c_raw - 1][start_idx] = -1.0;
                            }
                            if b_raw > 0 {
                                mna.n_i[b_raw - 1][start_idx] = -1.0 / beta_f;
                            }
                            if e_raw > 0 {
                                mna.n_i[e_raw - 1][start_idx] = 1.0 + 1.0 / beta_f;
                            }
                        }
                    }
                }
                NonlinearDeviceType::Jfet => {
                    // JFET: 2D — dim 0: (Vds, Id), dim 1: (Vgs, Ig)
                    //
                    // Dimension pairing for stable K diagonal (K[i][i] < 0):
                    //   dim 0: N_v row extracts Vds, N_i col injects Id (drain current drives
                    //          drain-source voltage → K[0][0] = dVds/dId < 0 always)
                    //   dim 1: N_v row extracts Vgs, N_i col injects Ig (gate current drives
                    //          gate-source voltage → K[1][1] = dVgs/dIg < 0 always)
                    //
                    // This ordering ensures K[i][i] < 0 even when source is VS-pinned,
                    // because drain always has a finite load (never VS-pinned in typical circuits).
                    // Nodes: [nd, ng, ns]
                    if node_indices.len() >= 3 {
                        let d_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let s_raw = node_indices[2];

                        // N_v row 0 (Vds): +1 at D, -1 at S
                        if d_raw > 0 {
                            mna.n_v[start_idx][d_raw - 1] = 1.0;
                        }
                        if s_raw > 0 {
                            mna.n_v[start_idx][s_raw - 1] = -1.0;
                        }
                        // N_v row 1 (Vgs): +1 at G, -1 at S
                        if g_raw > 0 {
                            mna.n_v[start_idx + 1][g_raw - 1] = 1.0;
                        }
                        if s_raw > 0 {
                            mna.n_v[start_idx + 1][s_raw - 1] = -1.0;
                        }

                        // N_i col 0 (Id): -1 at D (extracted), +1 at S (injected)
                        if d_raw > 0 {
                            mna.n_i[d_raw - 1][start_idx] = -1.0;
                        }
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx] = 1.0;
                        }
                        // N_i col 1 (Ig): -1 at G (extracted), +1 at S (injected)
                        if g_raw > 0 {
                            mna.n_i[g_raw - 1][start_idx + 1] = -1.0;
                        }
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx + 1] = 1.0;
                        }
                    }
                }
                NonlinearDeviceType::Mosfet => {
                    // MOSFET: 2D — dim 0: (Vds, Id), dim 1: (Vgs, Ig=0)
                    //
                    // Dimension pairing for stable K diagonal (K[i][i] < 0):
                    //   dim 0: N_v row extracts Vds, N_i col injects Id (drain current drives
                    //          drain-source voltage → K[0][0] = dVds/dId < 0 always, even when
                    //          source is VS-pinned, because drain always has a finite load resistor)
                    //   dim 1: N_v row extracts Vgs, N_i col injects Ig (gate current drives
                    //          gate-source voltage → K[1][1] = dVgs/dIg < 0 always because
                    //          gate always has at least a parasitic capacitor)
                    // Nodes: [nd, ng, ns, nb]
                    if node_indices.len() >= 3 {
                        let d_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let s_raw = node_indices[2];

                        // N_v row 0 (Vds): +1 at D, -1 at S
                        if d_raw > 0 {
                            mna.n_v[start_idx][d_raw - 1] = 1.0;
                        }
                        if s_raw > 0 {
                            mna.n_v[start_idx][s_raw - 1] = -1.0;
                        }
                        // N_v row 1 (Vgs): +1 at G, -1 at S
                        if g_raw > 0 {
                            mna.n_v[start_idx + 1][g_raw - 1] = 1.0;
                        }
                        if s_raw > 0 {
                            mna.n_v[start_idx + 1][s_raw - 1] = -1.0;
                        }

                        // N_i col 0 (Id): -1 at D (extracted), +1 at S (injected)
                        if d_raw > 0 {
                            mna.n_i[d_raw - 1][start_idx] = -1.0;
                        }
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx] = 1.0;
                        }
                        // N_i col 1 (Ig): effectively zero (insulated gate), but stamp for framework
                        if g_raw > 0 {
                            mna.n_i[g_raw - 1][start_idx + 1] = -1.0;
                        }
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx + 1] = 1.0;
                        }
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
                            if g_raw > 0 {
                                mna.n_v[start_idx][g_raw - 1] = 1.0;
                            }
                            if k_raw > 0 {
                                mna.n_v[start_idx][k_raw - 1] = -1.0;
                            }
                            // N_v row 1 (Vpk): +1 at plate, -1 at cathode
                            if p_raw > 0 {
                                mna.n_v[start_idx + 1][p_raw - 1] = 1.0;
                            }
                            if k_raw > 0 {
                                mna.n_v[start_idx + 1][k_raw - 1] = -1.0;
                            }
                            // N_i col 0 (Ip): -1 at plate, +1 at cathode
                            if p_raw > 0 {
                                mna.n_i[p_raw - 1][start_idx] = -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx] = 1.0;
                            }
                            // N_i col 1 (Ig): -1 at grid, +1 at cathode
                            if g_raw > 0 {
                                mna.n_i[g_raw - 1][start_idx + 1] = -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx + 1] = 1.0;
                            }
                        }
                    }
                }
            }
        }

        Ok(mna)
    }

    fn collect_nodes(&mut self, element: &Element) -> Result<(), MnaError> {
        let nodes = match element {
            Element::Resistor {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::Capacitor {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::Inductor {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::VoltageSource {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::CurrentSource {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::Diode {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::Bjt { nc, nb, ne, .. } => vec![nc, nb, ne],
            Element::Jfet { nd, ng, ns, .. } => vec![nd, ng, ns],
            Element::Mosfet { nd, ng, ns, nb, .. } => vec![nd, ng, ns, nb],
            Element::Triode {
                n_grid,
                n_plate,
                n_cathode,
                ..
            } => vec![n_grid, n_plate, n_cathode],
            Element::Opamp {
                n_plus,
                n_minus,
                n_out,
                ..
            } => vec![n_plus, n_minus, n_out],
            Element::Vcvs {
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                ..
            } => vec![out_p, out_n, ctrl_p, ctrl_n],
            Element::Vccs {
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                ..
            } => vec![out_p, out_n, ctrl_p, ctrl_n],
            Element::SubcktInstance { name, .. } => {
                return Err(MnaError::TopologyError(format!(
                    "subcircuit instance '{}' not supported (expand subcircuits before MNA)",
                    name
                )));
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
            Element::Resistor {
                name,
                n_plus,
                n_minus,
                value,
            } => {
                self.elements.push(ElementInfo {
                    element_type: ElementType::Resistor,
                    nodes: vec![self.node_map[n_plus], self.node_map[n_minus]],
                    value: *value,
                    name: name.clone(),
                    dc_value: None,
                });
            }
            Element::Capacitor {
                name,
                n_plus,
                n_minus,
                value,
                ..
            } => {
                self.elements.push(ElementInfo {
                    element_type: ElementType::Capacitor,
                    nodes: vec![self.node_map[n_plus], self.node_map[n_minus]],
                    value: *value,
                    name: name.clone(),
                    dc_value: None,
                });
            }
            Element::Inductor {
                name,
                n_plus,
                n_minus,
                value,
            } => {
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
            Element::VoltageSource {
                name,
                n_plus,
                n_minus,
                dc,
                ..
            } => {
                // Record element for augmented MNA stamping (done after matrix expansion).
                // Do NOT add Norton equivalent conductance here.
                self.elements.push(ElementInfo {
                    element_type: ElementType::VoltageSource,
                    nodes: vec![self.node_map[n_plus], self.node_map[n_minus]],
                    value: dc.unwrap_or(0.0),
                    name: name.clone(),
                    dc_value: *dc,
                });
                // ext_idx = 0-based index within voltage sources (used as offset in augmented rows)
                let ext_idx = self.voltage_sources.len();
                self.voltage_sources.push(VoltageSourceInfo {
                    name: name.clone(),
                    n_plus: n_plus.clone(),
                    n_minus: n_minus.clone(),
                    n_plus_idx: self.node_map[n_plus],
                    n_minus_idx: self.node_map[n_minus],
                    dc_value: dc.unwrap_or(0.0),
                    ext_idx,
                });
            }
            Element::CurrentSource {
                name,
                n_plus,
                n_minus,
                dc,
            } => {
                self.current_sources.push(CurrentSourceInfo {
                    name: name.clone(),
                    n_plus_idx: self.node_map[n_plus],
                    n_minus_idx: self.node_map[n_minus],
                    dc_value: dc.unwrap_or(0.0),
                });
            }
            Element::Diode {
                name,
                n_plus,
                n_minus,
                ..
            } => {
                let node_indices = vec![self.node_map[n_plus], self.node_map[n_minus]];
                if node_indices.iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "diode '{}' has both terminals grounded",
                        name
                    )));
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
            Element::Bjt {
                name, nc, nb, ne, ..
            } => {
                let node_indices = vec![self.node_map[nc], self.node_map[nb], self.node_map[ne]];
                if node_indices.iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "BJT '{}' has all terminals grounded",
                        name
                    )));
                }
                let is_forward_active = self
                    .forward_active_bjts
                    .contains(&name.to_ascii_uppercase());
                let (device_type, dimension) = if is_forward_active {
                    (NonlinearDeviceType::BjtForwardActive, 1)
                } else {
                    (NonlinearDeviceType::Bjt, 2)
                };
                let start_idx = self.total_dimension;
                self.total_dimension += dimension;
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type,
                    dimension,
                    start_idx,
                    nodes: vec![nc.clone(), nb.clone(), ne.clone()],
                    node_indices,
                });
            }
            Element::Jfet {
                name, nd, ng, ns, ..
            } => {
                let node_indices = vec![self.node_map[nd], self.node_map[ng], self.node_map[ns]];
                if node_indices.iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "JFET '{}' has all terminals grounded",
                        name
                    )));
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
            Element::Mosfet {
                name,
                nd,
                ng,
                ns,
                nb,
                ..
            } => {
                let node_indices = vec![
                    self.node_map[nd],
                    self.node_map[ng],
                    self.node_map[ns],
                    self.node_map[nb],
                ];
                // Check drain/gate/source (not bulk) for all-grounded
                if node_indices[..3].iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "MOSFET '{}' has all terminals grounded",
                        name
                    )));
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
            Element::Triode {
                name,
                n_grid,
                n_plate,
                n_cathode,
                ..
            } => {
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
            Element::Opamp {
                name,
                n_plus,
                n_minus,
                n_out,
                ..
            } => {
                let np_idx = self.node_map[n_plus];
                let nm_idx = self.node_map[n_minus];
                let no_idx = self.node_map[n_out];

                if no_idx == 0 {
                    return Err(MnaError::TopologyError(format!(
                        "op-amp '{}' has output connected to ground",
                        name
                    )));
                }
                if np_idx == 0 && nm_idx == 0 {
                    return Err(MnaError::TopologyError(format!(
                        "op-amp '{}' has both inputs grounded",
                        name
                    )));
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
            Element::Vcvs {
                name,
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                gain,
            } => {
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
            Element::Vccs {
                name,
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                gm,
            } => {
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
                return Err(MnaError::TopologyError(format!(
                    "subcircuit instance '{}' not supported (expand subcircuits before MNA)",
                    name
                )));
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
        assert!(
            (mna.g[o][i] - (-200_000.0 - g_r2)).abs() < 1e-6,
            "G[out,inv] should be -Gm - g_R2, got {}",
            mna.g[o][i]
        );

        // G[out, out] should have Go + g_R2
        let expected_go = 1.0 + g_r2;
        assert!(
            (mna.g[o][o] - expected_go).abs() < 1e-6,
            "G[out,out] should include Go={}, got {}",
            expected_go,
            mna.g[o][o]
        );
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
        assert!(
            (mna.g[o][i] - 0.01).abs() < 1e-15,
            "G[out,in] should be +gm=0.01, got {}",
            mna.g[o][i]
        );

        // G[out, out] should only have resistor conductance (1/1k = 0.001)
        assert!(
            (mna.g[o][o] - 0.001).abs() < 1e-15,
            "G[out,out] should be 0.001, got {}",
            mna.g[o][o]
        );
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
        assert!(
            (mna.g[o][inp_idx - 1] - 0.01).abs() < 1e-15,
            "G[out,inp] should be +gm, got {}",
            mna.g[o][inp_idx - 1]
        );

        // G[out, inn] should have -gm
        assert!(
            (mna.g[o][inn_idx - 1] - (-0.01)).abs() < 1e-15,
            "G[out,inn] should be -gm, got {}",
            mna.g[o][inn_idx - 1]
        );
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
        // With augmented MNA, VCVS adds an extra row/col at index n + num_vs + 0.
        let spice = r#"VCVS Test
R1 in 0 1k
R2 out 0 1k
E1 out 0 in 0 10
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.m, 0); // VCVS is linear
        assert_eq!(mna.num_devices, 0);

        let n = mna.n;
        // n_aug = n + 0 VS + 1 VCVS = n + 1
        assert_eq!(mna.n_aug, n + 1);
        assert_eq!(mna.g.len(), n + 1, "G should be n_aug × n_aug");

        let in_idx = *mna.node_map.get("in").unwrap();
        let out_idx = *mna.node_map.get("out").unwrap();
        let o = out_idx - 1;
        let i = in_idx - 1;
        let k = n; // augmented row for VCVS (no voltage sources, so k = n + 0 + 0)

        // G[out][k] should be +1 (current injection column: j_vcvs enters out+)
        assert!(
            (mna.g[o][k] - 1.0).abs() < 1e-15,
            "G[out][k] should be +1 for VCVS current injection, got {}",
            mna.g[o][k]
        );

        // G[k][out] should be +1 (KVL row: V_out+)
        assert!(
            (mna.g[k][o] - 1.0).abs() < 1e-15,
            "G[k][out] should be +1 for KVL constraint, got {}",
            mna.g[k][o]
        );

        // G[k][in] should be -gain = -10 (KVL row: -gain * V_ctrl+)
        assert!(
            (mna.g[k][i] - (-10.0)).abs() < 1e-15,
            "G[k][in] should be -gain=-10 for KVL constraint, got {}",
            mna.g[k][i]
        );

        // G[out][out] should only have 1/R2 (no VS_CONDUCTANCE in augmented MNA)
        let g_r2 = 1.0 / 1000.0;
        assert!(
            (mna.g[o][o] - g_r2).abs() < 1e-10,
            "G[out][out] should only have 1/R2={}, got {} (augmented MNA: no Norton equiv)",
            g_r2,
            mna.g[o][o]
        );
    }

    #[test]
    fn test_mna_vcvs_differential_output() {
        // E1 out_p out_n in 0 5 — VCVS with non-ground output negative
        // With augmented MNA, VCVS adds an extra row/col (no VS in this circuit).
        let spice = r#"VCVS Diff Output
R1 out_p 0 1k
R2 out_n 0 1k
E1 out_p out_n in 0 5
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let n = mna.n;
        assert_eq!(mna.n_aug, n + 1, "One VCVS adds 1 augmented dimension");

        let op = *mna.node_map.get("out_p").unwrap() - 1;
        let on = *mna.node_map.get("out_n").unwrap() - 1;
        let inp = *mna.node_map.get("in").unwrap() - 1;
        let k = n; // augmented row for the VCVS

        // Current injection column: j_vcvs enters out+ (G[out_p][k] = +1), exits out- (G[out_n][k] = -1)
        let g_r = 1.0 / 1000.0;
        // G[out_p][out_p] should only have 1/R1 (no Norton equivalent conductance)
        assert!(
            (mna.g[op][op] - g_r).abs() < 1e-10,
            "G[op][op] should only be 1/R1 in augmented MNA, got {}",
            mna.g[op][op]
        );
        assert!(
            (mna.g[on][on] - g_r).abs() < 1e-10,
            "G[on][on] should only be 1/R2 in augmented MNA, got {}",
            mna.g[on][on]
        );

        // Current injection: j_vcvs enters out+, exits out-
        assert!(
            (mna.g[op][k] - 1.0).abs() < 1e-15,
            "G[out_p][k] should be +1 (current injection), got {}",
            mna.g[op][k]
        );
        assert!(
            (mna.g[on][k] - (-1.0)).abs() < 1e-15,
            "G[out_n][k] should be -1 (current injection), got {}",
            mna.g[on][k]
        );

        // KVL constraint row: G[k][out_p] = +1, G[k][out_n] = -1, G[k][in] = -gain = -5
        assert!(
            (mna.g[k][op] - 1.0).abs() < 1e-15,
            "G[k][out_p] should be +1 (KVL), got {}",
            mna.g[k][op]
        );
        assert!(
            (mna.g[k][on] - (-1.0)).abs() < 1e-15,
            "G[k][out_n] should be -1 (KVL), got {}",
            mna.g[k][on]
        );
        assert!(
            (mna.g[k][inp] - (-5.0)).abs() < 1e-15,
            "G[k][in] should be -gain=-5 (KVL), got {}",
            mna.g[k][inp]
        );
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
        assert!(
            (mna.g[o2][o] - 0.001).abs() < 1e-15,
            "G[out2,out] should be VCCS gm=0.001, got {}",
            mna.g[o2][o]
        );
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
        assert!(
            (PARASITIC_CAP - 10e-12).abs() < 1e-25,
            "PARASITIC_CAP should be 10pF"
        );
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
                assert_eq!(
                    mna.c[i][j], 0.0,
                    "C[{i}][{j}] should be 0 before parasitic caps"
                );
            }
        }

        mna.add_parasitic_caps();

        let anode = *mna.node_map.get("in").unwrap();
        let cathode = *mna.node_map.get("out").unwrap();
        let a = anode - 1; // 0-indexed
        let k = cathode - 1;

        // Diagonal: both nodes get +PARASITIC_CAP
        assert!(
            (mna.c[a][a] - PARASITIC_CAP).abs() < 1e-25,
            "C[anode][anode] should be PARASITIC_CAP, got {}",
            mna.c[a][a]
        );
        assert!(
            (mna.c[k][k] - PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][cathode] should be PARASITIC_CAP, got {}",
            mna.c[k][k]
        );

        // Off-diagonal: negative (cap between nodes, not to ground)
        assert!(
            (mna.c[a][k] + PARASITIC_CAP).abs() < 1e-25,
            "C[anode][cathode] should be -PARASITIC_CAP, got {}",
            mna.c[a][k]
        );
        assert!(
            (mna.c[k][a] + PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][anode] should be -PARASITIC_CAP, got {}",
            mna.c[k][a]
        );
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
        assert!(
            (mna.c[nb][nb] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
            "C[base][base] should be 2*PARASITIC_CAP, got {}",
            mna.c[nb][nb]
        );

        // Collector gets cap from B-C junction only
        assert!(
            (mna.c[nc][nc] - PARASITIC_CAP).abs() < 1e-25,
            "C[coll][coll] should be PARASITIC_CAP, got {}",
            mna.c[nc][nc]
        );

        // Emitter gets cap from B-E junction only
        assert!(
            (mna.c[ne][ne] - PARASITIC_CAP).abs() < 1e-25,
            "C[emit][emit] should be PARASITIC_CAP, got {}",
            mna.c[ne][ne]
        );

        // Off-diagonal: B-E junction
        assert!(
            (mna.c[nb][ne] + PARASITIC_CAP).abs() < 1e-25,
            "C[base][emit] should be -PARASITIC_CAP, got {}",
            mna.c[nb][ne]
        );
        assert!(
            (mna.c[ne][nb] + PARASITIC_CAP).abs() < 1e-25,
            "C[emit][base] should be -PARASITIC_CAP, got {}",
            mna.c[ne][nb]
        );

        // Off-diagonal: B-C junction
        assert!(
            (mna.c[nb][nc] + PARASITIC_CAP).abs() < 1e-25,
            "C[base][coll] should be -PARASITIC_CAP, got {}",
            mna.c[nb][nc]
        );
        assert!(
            (mna.c[nc][nb] + PARASITIC_CAP).abs() < 1e-25,
            "C[coll][base] should be -PARASITIC_CAP, got {}",
            mna.c[nc][nb]
        );

        // Collector-Emitter: no direct parasitic cap
        assert!(
            (mna.c[nc][ne]).abs() < 1e-25,
            "C[coll][emit] should be 0, got {}",
            mna.c[nc][ne]
        );
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
        assert!(
            (mna.c[ng][ng] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
            "C[gate][gate] should be 2*PARASITIC_CAP, got {}",
            mna.c[ng][ng]
        );

        // Off-diagonal: G-S
        assert!(
            (mna.c[ng][ns] + PARASITIC_CAP).abs() < 1e-25,
            "C[gate][source] should be -PARASITIC_CAP, got {}",
            mna.c[ng][ns]
        );

        // Off-diagonal: G-D
        assert!(
            (mna.c[ng][nd] + PARASITIC_CAP).abs() < 1e-25,
            "C[gate][drain] should be -PARASITIC_CAP, got {}",
            mna.c[ng][nd]
        );

        // Drain-Source: no direct parasitic cap
        assert!(
            (mna.c[nd][ns]).abs() < 1e-25,
            "C[drain][source] should be 0, got {}",
            mna.c[nd][ns]
        );
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
        assert!(
            (mna.c[ng][ng] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
            "C[gate][gate] should be 2*PARASITIC_CAP, got {}",
            mna.c[ng][ng]
        );

        // Off-diagonal: G-S
        assert!(
            (mna.c[ng][ns] + PARASITIC_CAP).abs() < 1e-25,
            "C[gate][source] should be -PARASITIC_CAP, got {}",
            mna.c[ng][ns]
        );

        // Off-diagonal: G-D
        assert!(
            (mna.c[ng][nd] + PARASITIC_CAP).abs() < 1e-25,
            "C[gate][drain] should be -PARASITIC_CAP, got {}",
            mna.c[ng][nd]
        );
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
        assert!(
            (mna.c[nk][nk] - 2.0 * PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][cathode] should be 2*PARASITIC_CAP, got {}",
            mna.c[nk][nk]
        );

        // Grid gets cap from Cgk only
        assert!(
            (mna.c[ng][ng] - PARASITIC_CAP).abs() < 1e-25,
            "C[grid][grid] should be PARASITIC_CAP, got {}",
            mna.c[ng][ng]
        );

        // Plate gets cap from Cpk only
        assert!(
            (mna.c[np][np] - PARASITIC_CAP).abs() < 1e-25,
            "C[plate][plate] should be PARASITIC_CAP, got {}",
            mna.c[np][np]
        );

        // Off-diagonal: grid-cathode
        assert!(
            (mna.c[ng][nk] + PARASITIC_CAP).abs() < 1e-25,
            "C[grid][cathode] should be -PARASITIC_CAP, got {}",
            mna.c[ng][nk]
        );
        assert!(
            (mna.c[nk][ng] + PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][grid] should be -PARASITIC_CAP, got {}",
            mna.c[nk][ng]
        );

        // Off-diagonal: plate-cathode
        assert!(
            (mna.c[np][nk] + PARASITIC_CAP).abs() < 1e-25,
            "C[plate][cathode] should be -PARASITIC_CAP, got {}",
            mna.c[np][nk]
        );
        assert!(
            (mna.c[nk][np] + PARASITIC_CAP).abs() < 1e-25,
            "C[cathode][plate] should be -PARASITIC_CAP, got {}",
            mna.c[nk][np]
        );

        // Grid-Plate: no direct parasitic cap
        assert!(
            (mna.c[ng][np]).abs() < 1e-25,
            "C[grid][plate] should be 0, got {}",
            mna.c[ng][np]
        );
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
        assert_eq!(
            off_diag_count, 6,
            "Expected 6 off-diagonal C entries (1 diode junction + 2 BJT junctions), got {off_diag_count}"
        );
    }
}
