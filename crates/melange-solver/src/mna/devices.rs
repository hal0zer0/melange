//! Nonlinear device info: slots, junction ports, BJT internal nodes, linearized devices, VCAs.

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
    /// Phase 1b grid-off reduction: per-slot frozen Vg2k for grid-off
    /// pentodes. Populated by `from_netlist_with_grid_off` from the
    /// detection map. Zero for non-grid-off devices and for pentodes
    /// running the full 3D path.
    pub vg2k_frozen: f64,
}

/// Physical port for a single junction current carried by a nonlinear
/// device's NR slot. One port per slot that has a real shot/flicker-
/// generating current; the noise collectors iterate these instead of
/// indexing `node_indices` by hand.
///
/// `node_pos` / `node_neg` are oriented so a positive `i_nl[start_idx +
/// slot_offset]` flows from `node_pos` into `node_neg` — the same
/// convention shot/flicker stampers expect for Norton-equivalent
/// injection.
#[derive(Debug, Clone)]
pub struct JunctionCurrentPort {
    /// Display name suffix for this port (e.g. "Ic", "Ib", "Ip", "Id").
    /// Empty for single-junction devices like diodes — emitted code uses
    /// the bare device name in that case.
    pub label: &'static str,
    /// Offset from `NonlinearDeviceInfo::start_idx`. 0 = primary current
    /// (Ic / Id / Ip), 1 = secondary (Ib for BJTs).
    pub slot_offset: usize,
    pub node_pos: usize,
    pub node_neg: usize,
}

impl NonlinearDeviceInfo {
    /// Physical ports for each NR-current slot that carries a junction
    /// current. Single source of truth for shot- and flicker-noise port
    /// lookup — the only place that knows the per-device-type mapping
    /// from `node_indices` slots to physical (positive, negative) terminals.
    ///
    /// Tube branch handles both triode (3-node, `[grid, plate, cathode]`)
    /// and pentode (4/5-node, `[plate, grid, cathode, screen, [supp]]`)
    /// orderings — they're not the same, despite sharing
    /// `NonlinearDeviceType::Tube`.
    pub fn junction_current_ports(&self) -> Vec<JunctionCurrentPort> {
        let n = &self.node_indices;
        match self.device_type {
            NonlinearDeviceType::Diode if n.len() >= 2 => vec![JunctionCurrentPort {
                label: "",
                slot_offset: 0,
                node_pos: n[0], // anode
                node_neg: n[1], // cathode
            }],
            NonlinearDeviceType::Bjt if n.len() >= 3 => vec![
                JunctionCurrentPort {
                    label: "Ic",
                    slot_offset: 0,
                    node_pos: n[0], // collector
                    node_neg: n[2], // emitter
                },
                JunctionCurrentPort {
                    label: "Ib",
                    slot_offset: 1,
                    node_pos: n[1], // base
                    node_neg: n[2], // emitter
                },
            ],
            NonlinearDeviceType::BjtForwardActive if n.len() >= 3 => {
                vec![JunctionCurrentPort {
                    label: "Ic",
                    slot_offset: 0,
                    node_pos: n[0], // collector
                    node_neg: n[2], // emitter
                }]
            }
            NonlinearDeviceType::Jfet | NonlinearDeviceType::Mosfet if n.len() >= 3 => {
                vec![JunctionCurrentPort {
                    label: "Id",
                    slot_offset: 0,
                    node_pos: n[0], // drain
                    node_neg: n[2], // source
                }]
            }
            NonlinearDeviceType::Tube => match n.len() {
                // Triode: node_indices = [grid, plate, cathode]; Ip flows
                // plate → cathode.
                3 => vec![JunctionCurrentPort {
                    label: "Ip",
                    slot_offset: 0,
                    node_pos: n[1],
                    node_neg: n[2],
                }],
                // Pentode (4 = no suppressor, 5 = with suppressor):
                // node_indices = [plate, grid, cathode, screen, [supp]];
                // Ip flows plate → cathode in both forms. Phase 5 partition
                // (Ig2 shot) is deferred — would land here as a second port.
                4 | 5 => vec![JunctionCurrentPort {
                    label: "Ip",
                    slot_offset: 0,
                    node_pos: n[0],
                    node_neg: n[2],
                }],
                _ => Vec::new(),
            },
            // VCAs have no junction current (sig port is small-signal linear,
            // ctrl port is a control voltage, not a transport current).
            NonlinearDeviceType::Vca => Vec::new(),
            // Truncated node_indices (defensive): no ports rather than panic.
            _ => Vec::new(),
        }
    }
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
    Vca,
    /// Opto/LDR photoresistor (1D). `node_indices = [r+, r-, ctrl+, ctrl-]`;
    /// only the first pair forms the NR resistance dimension (`I = v_d / R`,
    /// R frozen during the solve). The control pair drives the after-solve
    /// state advance and is otherwise electrically inert (draws no current).
    Ldr,
    /// Glow-discharge / neon lamp (1D, EXPERIMENTAL). `node_indices = [a, k]`.
    /// The resistance path is a plain 2-terminal monotone resistor whose value
    /// is selected by the FROZEN latch state block during the solve; the latch
    /// flips after the solve on the VO/VD thresholds. N_v/N_i structure is
    /// identical to a diode/LDR resistance path.
    Glow,
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

/// Linearized BJT info: the device's small-signal Jacobian stamped into G at DC OP.
///
/// The BJT is removed from the nonlinear system (M reduced by 2) and its
/// behavior captured by its terminal-current Jacobian and DC bias currents in
/// the linear system.
#[derive(Debug, Clone)]
pub struct LinearizedBjtInfo {
    pub name: String,
    /// 1-indexed node indices (0 = ground)
    pub nc: usize,
    pub nb: usize,
    pub ne: usize,
    /// Terminal-current Jacobian at the DC OP, against the EXTERNAL
    /// terminal-pair voltages (Vbe = V(nb) - V(ne), Vbc = V(nb) - V(nc)).
    /// Ic and Ib are the currents into the collector and base terminals.
    pub dic_dvbe: f64,
    pub dic_dvbc: f64,
    pub dib_dvbe: f64,
    pub dib_dvbc: f64,
    /// B-E and B-C capacitances at the DC OP (depletion plus the B-E
    /// diffusion term, `BjtParams::linearized_junction_caps`), stamped
    /// between the external terminals.
    pub cbe: f64,
    pub cbc: f64,
    /// DC bias currents
    pub ic_dc: f64,
    pub ib_dc: f64,
    /// Operating-point controlling voltages in EXTERNAL node space
    /// (Vbe0 = V(nb) - V(ne), Vbc0 = V(nb) - V(nc) at the DC OP).
    /// Required so `stamp_linearized_bjts` can inject the proper Norton
    /// constant I0 - J·v0: the small-signal Jacobian is stamped
    /// against FULL node voltages, so the companion current source must
    /// subtract the linear-model current at the OP or the linearized
    /// circuit's DC fixed point drifts away from the operating point.
    pub vbe0: f64,
    pub vbc0: f64,
    /// PNP: the forward-active region's signs flip (`vbc_eff = -Vbc`,
    /// collector current negative).
    pub is_pnp: bool,
}

/// Linearized triode info: small-signal conductances stamped into G at DC OP.
///
/// The triode is removed from the nonlinear system (M reduced by 2) and its
/// behavior captured by gm (transconductance) and gp (plate conductance = 1/rp)
/// plus DC bias currents. Only valid when the triode operates in its linear
/// region (grid well below conduction onset, no grid current).
#[derive(Debug, Clone)]
pub struct LinearizedTriodeInfo {
    pub name: String,
    /// 1-indexed node indices (0 = ground)
    pub ng: usize, // grid
    pub np: usize, // plate
    pub nk: usize, // cathode
    /// Transconductance dIp/dVgk (VCCS: plate current controlled by grid-cathode voltage)
    pub gm: f64,
    /// Plate conductance dIp/dVpk = 1/rp (shunt between plate and cathode)
    pub gp: f64,
    /// DC plate current at operating point
    pub ip_dc: f64,
    /// DC grid current at operating point (should be ~0 for valid linearization)
    pub ig_dc: f64,
    /// Operating-point controlling voltages in EXTERNAL node space
    /// (Vgk0 = V(ng) - V(nk), Vpk0 = V(np) - V(nk) at the DC OP).
    /// Used by `stamp_linearized_triodes` for the Norton constant
    /// I0 - g·v0 — see `LinearizedBjtInfo::vbe0` for the invariant.
    pub vgk0: f64,
    pub vpk0: f64,
    /// Inter-electrode capacitances (`CCG` cathode-grid, `CGP` grid-plate,
    /// `CCP` cathode-plate): linear, so they stay in the circuit unchanged
    /// when the plate law is linearized.
    pub ccg: f64,
    pub cgp: f64,
    pub ccp: f64,
    /// Grid-cathode voltage at which the grid starts to conduct (the
    /// manufacturers' 0.3 uA starting point of the grid law), the edge of the
    /// region the linearization assumes.
    pub grid_onset: f64,
}

/// VCA (Voltage-Controlled Amplifier) info for MNA system.
///
/// A 4-terminal nonlinear device with signal and control paths:
/// - sig+/sig-: signal current path (I_sig = G0 * exp(-V_ctrl / VSCALE) * V_sig)
/// - ctrl+/ctrl-: control voltage (high impedance, draws no current)
///
/// M=2 per VCA: dim 0 maps V_signal → I_signal, dim 1 maps V_control → I_control (= 0).
#[derive(Debug, Clone)]
pub struct VcaInfo {
    pub name: String,
    /// Signal positive node index (1-indexed, 0 = ground)
    pub n_sig_p_idx: usize,
    /// Signal negative node index (1-indexed, 0 = ground)
    pub n_sig_n_idx: usize,
    /// Control positive node index (1-indexed, 0 = ground)
    pub n_ctrl_p_idx: usize,
    /// Control negative node index (1-indexed, 0 = ground)
    pub n_ctrl_n_idx: usize,
    /// Control voltage scale (default 0.05298 V/neper, THAT 2180A)
    pub vscale: f64,
    /// Nominal gain at V_ctrl = 0 (default 1.0)
    pub g0: f64,
    /// Current mode: I_out = G(Vc) * I_in (THAT 2180 style translinear).
    /// When false (default): voltage mode, I_out = G(Vc) * V_sig.
    pub current_mode: bool,
    /// Augmented row for current-sensing branch current (1-indexed, 0 = none).
    pub n_sense_idx: usize,
    /// Internal signal node sig+_int (1-indexed, 0 = none).
    /// Between sensing source and dummy termination resistor.
    pub n_internal_idx: usize,
}
