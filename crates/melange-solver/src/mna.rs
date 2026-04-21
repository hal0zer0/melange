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
    /// Runtime voltage sources (resolved from `.runtime` directives).
    /// Each entry carries the VS's aug-MNA row (for RHS stamping) plus the
    /// Rust field name codegen should emit on CircuitState. See Oomox P1.
    pub runtime_sources: Vec<RuntimeSourceInfo>,
    /// Switch info (resolved from .switch directives)
    pub switches: Vec<SwitchInfo>,
    /// Op-amp info (for VCCS stamping)
    pub opamps: Vec<OpampInfo>,
    /// VCA info (nonlinear voltage-controlled amplifier)
    pub vcas: Vec<VcaInfo>,
    /// Internal nodes for parasitic BJTs (RB/RC/RE in transient MNA).
    /// Empty when no parasitic BJTs are present or all are forward-active.
    pub bjt_internal_nodes: Vec<BjtTransientInternalNodes>,
    /// Linearized BJTs: small-signal conductances stamped into G.
    /// These BJTs are NOT in the nonlinear device list (M reduced by 2 each).
    pub linearized_bjts: Vec<LinearizedBjtInfo>,
    /// Linearized triodes: small-signal gm + 1/rp stamped into G.
    /// These triodes are NOT in the nonlinear device list (M reduced by 2 each).
    pub linearized_triodes: Vec<LinearizedTriodeInfo>,
    /// Pot default overrides: resistor name (uppercase) → default resistance.
    /// When a .pot has a default value, the G matrix is stamped at this value
    /// (not the component declaration value). Empty for circuits without .pot defaults.
    pub pot_default_overrides: HashMap<String, f64>,
    /// Switch initial-position overrides: component name (uppercase) → (type, pos-0 value).
    /// When a component appears in a `.switch` directive, the G/C matrix is stamped at
    /// the switch's position-0 value (not the component declaration value). The initial
    /// switch_position is always 0, so stamping pos-0 keeps the initial state
    /// self-consistent. Empty when no `.switch` directives are present.
    pub switch_default_overrides: HashMap<String, (char, f64)>,
    /// Wiper potentiometer groups (links two pots as complementary legs).
    pub wiper_groups: Vec<WiperGroupInfo>,
    /// Gang groups (links multiple pots/wipers under one parameter).
    pub gang_groups: Vec<GangGroupInfo>,
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
    /// Saturation current for iron-core model. None = linear (default).
    /// When set, L_eff(I) = value / cosh²(I / isat).
    pub isat: Option<f64>,
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
    /// Saturation current for winding 1. None = linear.
    pub l1_isat: Option<f64>,
    /// Saturation current for winding 2. None = linear.
    pub l2_isat: Option<f64>,
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
    /// Per-winding saturation current. None = linear.
    pub winding_isats: Vec<Option<f64>>,
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
    /// Phase 1b grid-off reduction: per-slot frozen Vg2k for grid-off
    /// pentodes. Populated by `from_netlist_with_grid_off` from the
    /// detection map. Zero for non-grid-off devices and for pentodes
    /// running the full 3D path.
    pub vg2k_frozen: f64,
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

/// Linearized BJT info: small-signal conductances stamped into G at DC OP.
///
/// The BJT is removed from the nonlinear system (M reduced by 2) and its
/// behavior captured by g-parameters and DC bias currents in the linear system.
#[derive(Debug, Clone)]
pub struct LinearizedBjtInfo {
    pub name: String,
    /// 1-indexed node indices (0 = ground)
    pub nc: usize,
    pub nb: usize,
    pub ne: usize,
    /// Small-signal conductances (computed from DC OP Jacobian)
    pub gm: f64, // dIc/dVbe (transconductance)
    pub gpi: f64, // dIb/dVbe (input conductance)
    pub gmu: f64, // dIc/dVbc (feedback conductance)
    pub go: f64,  // dIb/dVbc (reverse base conductance)
    /// DC bias currents
    pub ic_dc: f64,
    pub ib_dc: f64,
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
    /// Output saturation voltage [V] (default: infinity = no saturation).
    /// Typical NE5534: 13.0. When finite, output is clamped to ±VSAT.
    /// Superseded by VCC/VEE when those are specified.
    pub vsat: f64,
    /// Positive supply rail voltage [V] (default: infinity = no upper clamp).
    /// When finite, output is clamped to ≤ VCC. Takes priority over VSAT.
    /// Example: VCC=9 for single-supply 9V, VCC=18 for charge-pump 18V rail.
    pub vcc: f64,
    /// Negative supply rail voltage [V] (default: -infinity = no lower clamp).
    /// When finite, output is clamped to ≥ VEE. Takes priority over VSAT.
    /// Example: VEE=0 for single-supply, VEE=-9 for charge-pump negative rail.
    pub vee: f64,
    /// Voltage drop from VCC to the maximum output voltage the op-amp can
    /// actually drive [V]. Models the output-stage transistor saturation
    /// (in Boyle macromodels, the `V_upper = VCC - VOH_DROP` rail-offset
    /// source). Default 1.5 V for TL072/NE5532-class parts. Set smaller
    /// for rail-to-rail op-amps (e.g. `VOH_DROP=0.05` for LMV358 light load).
    /// Only consulted when `OpampRailMode::BoyleDiodes` is active; ignored
    /// by the `Hard` and `ActiveSet` rail-handling modes.
    pub voh_drop: f64,
    /// Voltage drop from VEE to the minimum output voltage the op-amp can
    /// actually drive [V]. Symmetrical counterpart to `voh_drop`. Default 1.5 V.
    pub vol_drop: f64,
    /// Gain-bandwidth product [Hz] (default: infinity = no dominant pole).
    /// When finite, a dominant pole capacitor C = AOL / (2π × GBW × ROUT)
    /// is stamped at an internal Boyle gain node.
    /// Typical NE5534: 10e6. TL074: 3e6.
    pub gbw: f64,
    /// Slew rate [V/s] (default: infinity = no slew limiting).
    ///
    /// Models the large-signal output voltage-rate limit of real op-amps.
    /// Parsed from the `.model` card in V/μs (SPICE convention) and converted
    /// to V/s internally. Examples: TL072 = 13 V/μs → `sr = 13e6`; NE5532 =
    /// 9 V/μs → `sr = 9e6`; LM358 = 0.3 V/μs → `sr = 0.3e6`.
    ///
    /// Emitted as a per-sample voltage-delta clamp on the op-amp output node
    /// (equivalent to clamping the Boyle dominant-pole integrator input current
    /// to ±`SR * C_dom`). When `sr` is infinite no clamp is emitted, so circuits
    /// without `SR=` in their .model get byte-identical generated code to the
    /// pre-slew-rate behaviour.
    pub sr: f64,
    /// Input bias current [A] (default 0 = ideal, no bias).
    ///
    /// Models the small DC current that flows into (or out of) each op-amp
    /// input pin on real hardware. Parsed from `.model OA(IB=...)`; typical
    /// values: TL074 = 30e-12 (30 pA JFET input), LM358 = 45e-9 (45 nA
    /// bipolar), NE5532 = 200e-9, OPA134 = 100e-15.
    ///
    /// Stamped as a symmetric DC current source pushing `IB` into both
    /// `n_plus` and `n_minus` from the external circuit's perspective (sign
    /// convention: positive IB = current flows out of the op-amp input pin
    /// into the external node — appropriate for PNP-input bipolar op-amps and
    /// JFET-input parts where gate leakage is outgoing). For NPN-input op-amps
    /// specify `IB=-45n` etc.
    ///
    /// Physical significance beyond DC offset: at integrator nodes the real-
    /// hardware bias current + finite input resistance (modeled via `RIN`)
    /// together drain accumulated charge off the feedback cap, bounding
    /// integrator wind-up on any residual DC offset at the input. A purely
    /// ideal op-amp with no IB and infinite input impedance winds up
    /// unbounded when the upstream network imposes a DC offset — a known
    /// failure mode of melange-emitted transient NR on circuits like the
    /// SSL 4kbuscomp sidechain integrator.
    pub ib: f64,
    /// Input resistance [Ω] from each input pin to ground (default +∞ = no
    /// leakage path, ideal). Typical values: TL074 (JFET) = 1e12, LM358
    /// (bipolar) = 1e6 to 1e7, NE5532 = 3e5.
    ///
    /// Stamped as a shunt conductance `1/RIN` at each input node. Provides
    /// a DC leakage path that bounds integrator wind-up: a 33-µs integrator
    /// with 1 TΩ input resistance will have a 10-second wind-up decay
    /// envelope. Finite RIN is what makes real integrator circuits stable
    /// under DC offsets that would otherwise cause ideal-op-amp models to
    /// drift without bound.
    pub rin: f64,
    /// User override for the transient-NR AOL cap.
    /// `INFINITY` (default) defers to the auto-detect heuristic in
    /// `crates/melange-solver/src/codegen/ir.rs::effective_aol_cap`. Set finite
    /// to force a specific cap; set to `oa.aol` (or larger) to fully disable
    /// the cap on this op-amp; set to a small value (e.g. 1000) to force the
    /// cap on a circuit the auto-detect missed. Only consulted in the codegen
    /// path; runtime DC OP uses its own `AOL_DC_MAX = 1000` cap.
    pub aol_transient_cap: f64,
    /// Boyle internal gain node index (1-indexed, 0 = none).
    /// No longer used with IIR op-amp model (always 0).
    pub n_internal_idx: usize,
    /// Dominant pole capacitance for IIR filter: C_dom = AOL / (2*pi*GBW*ROUT).
    /// Set during MNA stamping when GBW is finite; 0.0 when GBW is infinite.
    pub iir_c_dom: f64,
    /// `BoyleDiodes`-mode internal gain node index (1-indexed, 0 = not in
    /// BoyleDiodes mode). When non-zero, the op-amp's transconductance is
    /// stamped at this node instead of at `n_out_idx`, with the high
    /// impedance load `R_BOYLE_INT_LOAD` setting both Gm and Go so the
    /// catch diodes (placed externally between `n_int_idx` and rail
    /// references) only have to balance against ~1 µS instead of the
    /// op-amp's nominal Gm = AOL/r_out (typically 4000 S for TL072). The
    /// op-amp output node is then driven by an external unity-gain buffer
    /// VCCS sourced from this node — see
    /// [`crate::codegen::ir::augment_netlist_with_boyle_diodes`].
    ///
    /// Auto-detected during MNA stamping by looking up
    /// `_oa_int_{safe_name}` in `node_map`. The augment helper synthesizes
    /// that name (referenced from buffer/diode elements), so the field is
    /// non-zero only when the op-amp has gone through the augmentation pass.
    pub n_int_idx: usize,
}

/// Effective output resistance for the
/// [`OpampRailMode::BoyleDiodes`] internal gain node (Ω).
///
/// Both `Gm_int = AOL / R_BOYLE_INT_LOAD` and `Go_int = 1 / R_BOYLE_INT_LOAD`
/// are derived from this single value, so the open-loop voltage gain
/// from V+/V- to the internal node is preserved at AOL: a `Gm * v_diff`
/// current source feeding a `1/R` shunt sets `V_int = AOL · v_diff`
/// regardless of the absolute value of `R`. The choice trades two
/// constraints:
///
/// - **Catch-diode dominance**: at deep saturation the catch diode runs
///   at ~1 S forward conductance, so we need `1/R_BOYLE_INT_LOAD` to be
///   ≪ 1 S for the diode to anchor V_int. ⇒ R ≫ 1 Ω.
/// - **MNA pivoting against the rest of the circuit**: the off-diagonal
///   `Gm_int = AOL/R` should be in the same magnitude range as the
///   user circuit's typical entries (1 µS to 1 mS) to keep partial
///   pivoting numerically well-conditioned. With `AOL = 200000`,
///   `R = 1 MΩ` gives `Gm = 0.2 S` — comfortably above the user
///   circuit's ~1 mS ceiling so the int-node row is consistently
///   chosen as the pivot for the V+/V- columns, and the diode load
///   `Go = 1 µS` is well below the diode's full-on 1 S so the diode
///   still anchors the rail.
pub const R_BOYLE_INT_LOAD: f64 = 1.0e6;

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

/// Runtime voltage source resolved from `.runtime` directive.
///
/// `vs_row` is the aug-MNA row where the VS's KVL constraint lives
/// (= `n_nodes + ext_idx`). Each sample, codegen stamps the plugin-supplied
/// value into this row of the RHS vector.
#[derive(Debug, Clone)]
pub struct RuntimeSourceInfo {
    /// Name of the voltage source (as written in the netlist, preserving case).
    pub vs_name: String,
    /// Generated `CircuitState` field name (valid Rust identifier).
    pub field_name: String,
    /// Aug-MNA row index where `state.<field_name>` is stamped each sample.
    pub vs_row: usize,
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
///
/// Also used for `.runtime R` entries (audio-rate resistor modulation).
/// When `runtime_field = Some(field_name)`, this pot represents a runtime
/// resistor rather than a user-facing knob: codegen emits
/// `set_runtime_R_<field_name>` and a read-only accessor; the plugin
/// template does NOT emit a nih-plug FloatParam for it. Setter body
/// matches `.pot` since the 2026-04-20 reseed strip.
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
    /// If Some, this entry is a `.runtime R` rather than a `.pot`. The
    /// contained string is the Rust identifier for the generated setter
    /// (`set_runtime_R_<field>`) and getter. None = ordinary user knob.
    pub runtime_field: Option<String>,
}

/// Wiper potentiometer group — links two `PotInfo` entries as complementary legs.
///
/// A single UI parameter (wiper position 0.0–1.0) controls both resistances:
/// R_cw = pos * (total - 2) + 1, R_ccw = (1-pos) * (total - 2) + 1.
#[derive(Debug, Clone)]
pub struct WiperGroupInfo {
    /// Index into `MnaSystem::pots` for the CW (top→wiper) leg
    pub cw_pot_index: usize,
    /// Index into `MnaSystem::pots` for the CCW (wiper→bottom) leg
    pub ccw_pot_index: usize,
    /// Total resistance (R_cw + R_ccw = total)
    pub total_resistance: f64,
    /// Default wiper position (0.0–1.0)
    pub default_position: f64,
    /// Optional human-readable label
    pub label: Option<String>,
}

/// Gang group — links multiple `.pot` and/or `.wiper` entries under a single parameter.
///
/// All members are controlled by one position value (0.0–1.0).
/// Pot members: R = max - pos * (max - min). Wiper members: wiper_pos = pos.
#[derive(Debug, Clone)]
pub struct GangGroupInfo {
    /// Human-readable label for the gang parameter
    pub label: String,
    /// Pot members: (pot_index into MnaSystem::pots, inverted)
    pub pot_members: Vec<(usize, bool)>,
    /// Wiper group members: (wiper_group_index into MnaSystem::wiper_groups, inverted)
    pub wiper_members: Vec<(usize, bool)>,
    /// Default position (0.0–1.0)
    pub default_position: f64,
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
            runtime_sources: Vec::new(),
            switches: Vec::new(),
            opamps: Vec::new(),
            vcas: Vec::new(),
            bjt_internal_nodes: Vec::new(),
            linearized_bjts: Vec::new(),
            linearized_triodes: Vec::new(),
            pot_default_overrides: HashMap::new(),
            switch_default_overrides: HashMap::new(),
            wiper_groups: Vec::new(),
            gang_groups: Vec::new(),
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

    /// Build MNA system with forward-active and linearized BJTs.
    ///
    /// Linearized BJTs are completely removed from the nonlinear system (M reduced
    /// by 2 per device). Their small-signal conductances must be stamped separately
    /// via `stamp_linearized_bjts()` after DC OP computation.
    pub fn from_netlist_with_linearized(
        netlist: &Netlist,
        forward_active: &std::collections::HashSet<String>,
        linearized: &std::collections::HashSet<String>,
    ) -> Result<Self, MnaError> {
        let mut builder = MnaBuilder::new();
        builder.forward_active_bjts = forward_active.clone();
        builder.linearized_bjts = linearized.clone();
        builder.build(netlist)
    }

    /// Build MNA system with specified pentodes using grid-off (2D) reduction.
    ///
    /// Pentodes whose names appear as keys in `grid_off` are modeled as 2D
    /// (Vgk → Ip, Vpk → Ig2, Ig1 dropped, Vg2k frozen at the map value),
    /// reducing M by 1 per grid-off pentode. Used after DC-OP analysis
    /// confirms Vgk is below cutoff across the operating point — the
    /// per-pentode map value is the DC-OP-converged Vg2k to freeze.
    ///
    /// Mirrors `from_netlist_forward_active` for BJTs, but with a map
    /// rather than a set so the per-slot `vg2k_frozen` field on
    /// [`NonlinearDeviceInfo`] can be populated in one pass.
    pub fn from_netlist_with_grid_off(
        netlist: &Netlist,
        grid_off: &std::collections::HashMap<String, f64>,
    ) -> Result<Self, MnaError> {
        let mut builder = MnaBuilder::new();
        builder.grid_off_pentodes = grid_off.clone();
        builder.build(netlist)
    }

    /// Build MNA system with both BJT forward-active and pentode grid-off reductions.
    ///
    /// Used by circuits (e.g. Plexi-class amps) that simultaneously benefit
    /// from FA-reduced biasing BJTs and grid-off-reduced power pentodes.
    pub fn from_netlist_with_grid_off_and_fa(
        netlist: &Netlist,
        forward_active: &std::collections::HashSet<String>,
        grid_off: &std::collections::HashMap<String, f64>,
    ) -> Result<Self, MnaError> {
        let mut builder = MnaBuilder::new();
        builder.forward_active_bjts = forward_active.clone();
        builder.grid_off_pentodes = grid_off.clone();
        builder.build(netlist)
    }

    /// Build MNA system with all device reductions applied simultaneously.
    ///
    /// Combines forward-active BJTs, linearized BJTs, linearized triodes,
    /// and grid-off pentodes in a single rebuild. Use this when a circuit
    /// benefits from multiple reduction types (e.g. triode cascade with
    /// FA-reduced biasing BJTs and linearized clean stages).
    pub fn from_netlist_with_all_reductions(
        netlist: &Netlist,
        forward_active: &std::collections::HashSet<String>,
        linearized_bjts: &std::collections::HashSet<String>,
        linearized_triodes: &std::collections::HashSet<String>,
        grid_off: &std::collections::HashMap<String, f64>,
    ) -> Result<Self, MnaError> {
        let mut builder = MnaBuilder::new();
        builder.forward_active_bjts = forward_active.clone();
        builder.linearized_bjts = linearized_bjts.clone();
        builder.linearized_triodes = linearized_triodes.clone();
        builder.grid_off_pentodes = grid_off.clone();
        builder.build(netlist)
    }

    /// Stamp linearized BJT small-signal conductances into G matrix.
    ///
    /// Must be called AFTER DC OP computation provides the g-parameters.
    /// Each linearized BJT gets:
    /// - gm (VCCS): Ic = gm * Vbe, current flows C→E controlled by B-E
    /// - gpi (conductance): dIb/dVbe between B and E
    /// - gmu (conductance): dIc/dVbc between B and C
    /// - go (conductance): dIb/dVbc between B and C
    /// - DC bias currents as current source injections
    pub fn stamp_linearized_bjts(&mut self) {
        for bjt in &self.linearized_bjts.clone() {
            let nc = bjt.nc; // 1-indexed
            let nb = bjt.nb;
            let ne = bjt.ne;

            // gpi: conductance between B and E (dIb/dVbe)
            if bjt.gpi.abs() > 1e-30 {
                if nb > 0 && ne > 0 {
                    let b = nb - 1;
                    let e = ne - 1;
                    self.g[b][b] += bjt.gpi;
                    self.g[e][e] += bjt.gpi;
                    self.g[b][e] -= bjt.gpi;
                    self.g[e][b] -= bjt.gpi;
                } else if nb > 0 {
                    self.g[nb - 1][nb - 1] += bjt.gpi;
                } else if ne > 0 {
                    self.g[ne - 1][ne - 1] += bjt.gpi;
                }
            }

            // gm: VCCS — Ic = gm * Vbe, current flows from C to E, controlled by B-E
            // Stamp: G[c][b] += gm, G[c][e] -= gm, G[e][b] -= gm, G[e][e] += gm
            if bjt.gm.abs() > 1e-30 {
                if nc > 0 && nb > 0 {
                    self.g[nc - 1][nb - 1] += bjt.gm;
                }
                if nc > 0 && ne > 0 {
                    self.g[nc - 1][ne - 1] -= bjt.gm;
                }
                if ne > 0 && nb > 0 {
                    self.g[ne - 1][nb - 1] -= bjt.gm;
                }
                if ne > 0 {
                    self.g[ne - 1][ne - 1] += bjt.gm;
                }
            }

            // gmu: conductance between B and C (dIc/dVbc)
            if bjt.gmu.abs() > 1e-30 {
                if nb > 0 && nc > 0 {
                    let b = nb - 1;
                    let c = nc - 1;
                    self.g[b][b] += bjt.gmu;
                    self.g[c][c] += bjt.gmu;
                    self.g[b][c] -= bjt.gmu;
                    self.g[c][b] -= bjt.gmu;
                } else if nb > 0 {
                    self.g[nb - 1][nb - 1] += bjt.gmu;
                } else if nc > 0 {
                    self.g[nc - 1][nc - 1] += bjt.gmu;
                }
            }

            // go: conductance between B and C (dIb/dVbc)
            if bjt.go.abs() > 1e-30 && nb > 0 && nc > 0 {
                let b = nb - 1;
                let c = nc - 1;
                self.g[b][b] += bjt.go;
                self.g[c][c] += bjt.go;
                self.g[b][c] -= bjt.go;
                self.g[c][b] -= bjt.go;
            }

            // DC bias currents (as current source injections)
            // Ic flows into collector, out of emitter
            if nc > 0 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ic_dc", bjt.name),
                    n_plus_idx: 0,
                    n_minus_idx: nc,
                    dc_value: bjt.ic_dc,
                });
            }
            // Ib flows into base, out of emitter
            if nb > 0 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ib_dc", bjt.name),
                    n_plus_idx: 0,
                    n_minus_idx: nb,
                    dc_value: bjt.ib_dc,
                });
            }
            // Ie = -(Ic + Ib) flows out of emitter
            if ne > 0 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ie_dc", bjt.name),
                    n_plus_idx: ne,
                    n_minus_idx: 0,
                    dc_value: bjt.ic_dc + bjt.ib_dc,
                });
            }
        }
    }

    /// Stamp linearized triode small-signal conductances into G matrix.
    ///
    /// Must be called AFTER DC OP computation provides the g-parameters.
    /// Each linearized triode gets:
    /// - gm (VCCS): Ip = gm * Vgk, current flows P→K controlled by G-K voltage
    /// - gp (conductance): 1/rp between plate and cathode
    /// - DC bias currents as current source injections
    pub fn stamp_linearized_triodes(&mut self) {
        for tube in &self.linearized_triodes.clone() {
            let ng = tube.ng; // 1-indexed (0 = ground)
            let np = tube.np;
            let nk = tube.nk;

            // gm: VCCS — Ip = gm * Vgk, current from plate to cathode, controlled by G-K
            // Stamp: G[p][g] += gm, G[p][k] -= gm, G[k][g] -= gm, G[k][k] += gm
            if tube.gm.abs() > 1e-30 {
                if np > 0 && ng > 0 {
                    self.g[np - 1][ng - 1] += tube.gm;
                }
                if np > 0 && nk > 0 {
                    self.g[np - 1][nk - 1] -= tube.gm;
                }
                if nk > 0 && ng > 0 {
                    self.g[nk - 1][ng - 1] -= tube.gm;
                }
                if nk > 0 {
                    self.g[nk - 1][nk - 1] += tube.gm;
                }
            }

            // gp = 1/rp: plate conductance between plate and cathode
            // Stamp: G[p][p] += gp, G[k][k] += gp, G[p][k] -= gp, G[k][p] -= gp
            if tube.gp.abs() > 1e-30 {
                if np > 0 && nk > 0 {
                    let p = np - 1;
                    let k = nk - 1;
                    self.g[p][p] += tube.gp;
                    self.g[k][k] += tube.gp;
                    self.g[p][k] -= tube.gp;
                    self.g[k][p] -= tube.gp;
                } else if np > 0 {
                    self.g[np - 1][np - 1] += tube.gp;
                } else if nk > 0 {
                    self.g[nk - 1][nk - 1] += tube.gp;
                }
            }

            // DC bias currents
            // Ip flows into plate (from cathode through tube)
            if np > 0 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ip_dc", tube.name),
                    n_plus_idx: 0,
                    n_minus_idx: np,
                    dc_value: tube.ip_dc,
                });
            }
            // Ig flows into grid (only non-negligible when tube is near grid conduction)
            if ng > 0 && tube.ig_dc.abs() > 1e-15 {
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ig_dc", tube.name),
                    n_plus_idx: 0,
                    n_minus_idx: ng,
                    dc_value: tube.ig_dc,
                });
            }
            // Ik = Ip + Ig flows out of cathode
            if nk > 0 {
                let ik = tube.ip_dc + tube.ig_dc;
                self.current_sources.push(CurrentSourceInfo {
                    name: format!("{}_Ik_dc", tube.name),
                    n_plus_idx: nk,
                    n_minus_idx: 0,
                    dc_value: ik,
                });
            }
        }
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
    pub fn stamp_device_junction_caps(&mut self, device_slots: &[crate::device_types::DeviceSlot]) {
        use crate::device_types::DeviceParams;

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
                    let int = self
                        .bjt_internal_nodes
                        .iter()
                        .find(|n| n.start_idx == slot.start_idx);
                    // CJE: base-emitter junction
                    if p.cje > 0.0 {
                        let cap_b = int.and_then(|n| n.int_base).map(|i| i + 1).unwrap_or(nb);
                        let cap_e = int.and_then(|n| n.int_emitter).map(|i| i + 1).unwrap_or(ne);
                        caps.push((cap_b, cap_e, p.cje));
                    }
                    // CJC: base-collector junction
                    if p.cjc > 0.0 {
                        let cap_b = int.and_then(|n| n.int_base).map(|i| i + 1).unwrap_or(nb);
                        let cap_c = int
                            .and_then(|n| n.int_collector)
                            .map(|i| i + 1)
                            .unwrap_or(nc);
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
                DeviceParams::Vca(_) => {
                    // VCA has no junction capacitances
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
    pub fn expand_bjt_internal_nodes(&mut self, device_slots: &[crate::device_types::DeviceSlot]) {
        use crate::device_types::{DeviceParams, DeviceType};

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
        for (dev_info, slot) in self.nonlinear_devices.iter().zip(device_slots.iter()) {
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
            if exp.rb > 0.0 {
                num_new += 1;
            }
            if exp.rc > 0.0 {
                num_new += 1;
            }
            if exp.re > 0.0 {
                num_new += 1;
            }
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
            let int_b = if exp.rb > 0.0 {
                let idx = next_idx;
                next_idx += 1;
                Some(idx)
            } else {
                None
            };
            let int_c = if exp.rc > 0.0 {
                let idx = next_idx;
                next_idx += 1;
                Some(idx)
            } else {
                None
            };
            let int_e = if exp.re > 0.0 {
                let idx = next_idx;
                next_idx += 1;
                Some(idx)
            } else {
                None
            };

            // External node indices (0-indexed; None if grounded)
            let ext_b_0 = if exp.ext_b > 0 {
                Some(exp.ext_b - 1)
            } else {
                None
            };
            let ext_c_0 = if exp.ext_c > 0 {
                Some(exp.ext_c - 1)
            } else {
                None
            };
            let ext_e_0 = if exp.ext_e > 0 {
                Some(exp.ext_e - 1)
            } else {
                None
            };

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
            for j in 0..new_n_aug {
                self.n_v[s][j] = 0.0;
                self.n_v[s + 1][j] = 0.0;
            }
            for i in 0..new_n_aug {
                self.n_i[i][s] = 0.0;
                self.n_i[i][s + 1] = 0.0;
            }

            // Re-stamp N_v: Vbe_int = V(basePrime) - V(emitterPrime)
            if let Some(b) = eff_b {
                self.n_v[s][b] = 1.0;
            }
            if let Some(e) = eff_e {
                self.n_v[s][e] = -1.0;
            }
            // N_v: Vbc_int = V(basePrime) - V(collectorPrime)
            if let Some(b) = eff_b {
                self.n_v[s + 1][b] = 1.0;
            }
            if let Some(c) = eff_c {
                self.n_v[s + 1][c] = -1.0;
            }

            // N_i: Ic enters collectorPrime, exits emitterPrime
            if let Some(c) = eff_c {
                self.n_i[c][s] = -1.0;
            }
            if let Some(e) = eff_e {
                self.n_i[e][s] = 1.0;
            }
            // N_i: Ib enters basePrime, exits emitterPrime
            if let Some(b) = eff_b {
                self.n_i[b][s + 1] = -1.0;
            }
            if let Some(e) = eff_e {
                self.n_i[e][s + 1] = 1.0;
            }

            self.bjt_internal_nodes.push(BjtTransientInternalNodes {
                device_name: exp.device_name.clone(),
                start_idx: s,
                int_base: int_b,
                int_collector: int_c,
                int_emitter: int_e,
            });

            log::debug!(
                "BJT {} internal nodes: base={:?} collector={:?} emitter={:?} (RB={} RC={} RE={})",
                exp.device_name,
                int_b,
                int_c,
                int_e,
                exp.rb,
                exp.rc,
                exp.re,
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

    /// Stamp grid-off reduced pentode nonlinear matrices (2D).
    ///
    /// Phase 1b reduction: when DC-OP confirms the pentode is biased in
    /// grid-cutoff (`Vgk < 0` with margin), the control-grid current `Ig1`
    /// is identically zero and the screen voltage `Vg2k` is approximately
    /// held constant by external bypass caps. Drop the `Ig1` NR dimension
    /// entirely; the screen voltage is passed into the device math as a
    /// per-slot constant stored in `DeviceSlot.vg2k_frozen` (written by
    /// DC-OP detection).
    ///
    /// N_v: two rows
    ///   - Row start_idx:     Vgk  = V[grid]  − V[cathode]
    ///   - Row start_idx + 1: Vpk  = V[plate] − V[cathode]
    /// N_i: two columns
    ///   - Col start_idx:     Ip  enters plate, exits cathode
    ///   - Col start_idx + 1: Ig2 enters screen, exits cathode
    ///
    /// Note that Ig1 is dropped (grid-cutoff) and the Vg2k probe is ALSO
    /// dropped — the device math reads Vg2k from the frozen constant, not
    /// from an N_v row.
    pub fn stamp_pentode_grid_off(
        &mut self,
        start_idx: usize,
        n_plate: usize,
        n_grid: usize,
        n_cathode: usize,
        n_screen: usize,
    ) {
        // Row start_idx: Vgk
        self.n_v[start_idx][n_grid] = 1.0;
        self.n_v[start_idx][n_cathode] = -1.0;

        // Row start_idx + 1: Vpk
        self.n_v[start_idx + 1][n_plate] = 1.0;
        self.n_v[start_idx + 1][n_cathode] = -1.0;

        // Col start_idx: Ip (plate current — flows plate → cathode through the device)
        self.n_i[n_plate][start_idx] = -1.0;
        self.n_i[n_cathode][start_idx] = 1.0;

        // Col start_idx + 1: Ig2 (screen current — flows screen → cathode through the device)
        self.n_i[n_screen][start_idx + 1] = -1.0;
        self.n_i[n_cathode][start_idx + 1] = 1.0;
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
    fn build_discretized_matrix(
        &self,
        sample_rate: f64,
        g_sign: f64,
    ) -> Result<Vec<Vec<f64>>, MnaError> {
        if !(sample_rate > 0.0 && sample_rate.is_finite()) {
            return Err(MnaError::InvalidParameter(format!(
                "invalid sample_rate: {} (must be positive and finite)",
                sample_rate
            )));
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

        Ok(mat)
    }

    /// Get the A matrix for a given sample rate (trapezoidal discretization).
    ///
    /// A = G + (2/T)*C (includes inductor companion model conductances)
    ///
    /// Returns `Err(MnaError::InvalidParameter)` if `sample_rate` is not positive and finite.
    pub fn get_a_matrix(&self, sample_rate: f64) -> Result<Vec<Vec<f64>>, MnaError> {
        self.build_discretized_matrix(sample_rate, 1.0)
    }

    /// Get the A_neg matrix for history term (trapezoidal discretization).
    ///
    /// A_neg = (2/T)*C - G (includes inductor companion model)
    ///
    /// Returns `Err(MnaError::InvalidParameter)` if `sample_rate` is not positive and finite.
    pub fn get_a_neg_matrix(&self, sample_rate: f64) -> Result<Vec<Vec<f64>>, MnaError> {
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
    /// - **Triode** (Tube, dim=2): grid-cathode (Cgk) + plate-cathode (Cpk)
    /// - **Pentode** (Tube, dim=3): grid-cathode (Cgk) + grid-plate (Cgp,
    ///   Miller cap) + plate-cathode (Cpk) + screen-cathode (Csk) +
    ///   screen-plate (Csp). Suppressor (if present) is treated as cathode-tied
    ///   in phase 1a and contributes no extra parasitic caps. TODO(phase 1b):
    ///   honor user-provided explicit Cgk/Cgp/Cpk/Csk/Csp from `.model`.
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
                    // Tube device family. Three shapes:
                    //   - Triode (dim=2, 3 nodes): [grid, plate, cathode]
                    //   - Pentode (dim=3, 4-5 nodes): [plate, grid, cathode, screen, (suppressor?)]
                    //   - Grid-off pentode (dim=2, 4-5 nodes): [plate, grid, cathode, screen, (suppressor?)]
                    // Node layout differs — see the `Element::Triode` /
                    // `Element::Pentode` arms in `categorize_element`.
                    // Suppressor is cathode-tied in phase 1a and gets no caps.
                    let is_pentode_shape = dev.node_indices.len() >= 4;
                    if dev.dimension == 3 && is_pentode_shape {
                        // Sharp-cutoff pentode: 5 junction caps.
                        // Cgk, Cgp, Cpk, Csk, Csp.
                        let np = dev.node_indices[0];
                        let ng = dev.node_indices[1];
                        let nk = dev.node_indices[2];
                        let nscr = dev.node_indices[3];
                        // Grid-cathode (Cgk)
                        junctions.push((dev.name.clone(), ng, nk));
                        // Grid-plate (Cgp, Miller)
                        junctions.push((dev.name.clone(), ng, np));
                        // Plate-cathode (Cpk)
                        junctions.push((dev.name.clone(), np, nk));
                        // Screen-cathode (Csk)
                        junctions.push((dev.name.clone(), nscr, nk));
                        // Screen-plate (Csp)
                        junctions.push((dev.name.clone(), nscr, np));
                    } else if dev.dimension == 2 && is_pentode_shape {
                        // Grid-off pentode: screen is an input, not an NR
                        // unknown, so only 3 junction caps are meaningful.
                        // Drop CSK/CSP; keep CGK (input), CGP (Miller), CPK.
                        let np = dev.node_indices[0];
                        let ng = dev.node_indices[1];
                        let nk = dev.node_indices[2];
                        // Grid-cathode (Cgk)
                        junctions.push((dev.name.clone(), ng, nk));
                        // Grid-plate (Cgp, Miller)
                        junctions.push((dev.name.clone(), ng, np));
                        // Plate-cathode (Cpk)
                        junctions.push((dev.name.clone(), np, nk));
                    } else {
                        // Triode: nodes are [grid, plate, cathode].
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
                NonlinearDeviceType::Vca => {
                    // node_indices: [sig_p, sig_n, ctrl_p, ctrl_n]
                    // Signal path junction (sig+ to sig-)
                    junctions.push((dev.name.clone(), dev.node_indices[0], dev.node_indices[1]));
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

/// Maximum circuit node count. Prevents unbounded allocation from pathological netlists.
/// 256 is generous for any real audio circuit (Pultec EQP-1A uses ~41 nodes).
pub const MAX_N: usize = 256;

/// Error type for MNA assembly.
#[derive(Debug, Clone)]
#[non_exhaustive]
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
    /// An upstream parse error.
    Parse(crate::parser::ParseError),
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
            MnaError::Parse(e) => write!(f, "MNA error: {}", e),
        }
    }
}

impl std::error::Error for MnaError {}

impl From<crate::parser::ParseError> for MnaError {
    fn from(e: crate::parser::ParseError) -> Self {
        MnaError::Parse(e)
    }
}

/// Invert a small NxN matrix using Gaussian elimination with partial pivoting.
/// Used for multi-winding transformer inductance matrix inversion.
/// Returns identity matrix as fallback if singular (with log warning).
pub(crate) fn invert_small_matrix(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
    // Guard: NaN/Inf bypass the pivot < 1e-30 singularity check
    for i in 0..n {
        for j in 0..n {
            if !a[i][j].is_finite() {
                log::warn!(
                    "Non-finite value in inductance matrix at [{i}][{j}]: {}",
                    a[i][j]
                );
                let mut result = vec![vec![0.0; n]; n];
                for k in 0..n {
                    result[k][k] = 1.0;
                }
                return result;
            }
        }
    }
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
    vcas: Vec<VcaInfo>,
    elements: Vec<ElementInfo>,
    /// Total voltage dimension accumulated so far
    total_dimension: usize,
    /// BJT names to model as forward-active (1D instead of 2D)
    forward_active_bjts: std::collections::HashSet<String>,
    /// BJT names to linearize at DC OP (removed from nonlinear system entirely)
    linearized_bjts: std::collections::HashSet<String>,
    /// Triode names to linearize at DC OP (removed from nonlinear system entirely, M-2 each)
    linearized_triodes: std::collections::HashSet<String>,
    /// Pentode names to model with grid-off reduction (2D instead of 3D).
    /// Phase 1b: when DC-OP confirms Vgk is below cutoff, drop the Ig1 NR
    /// dimension and freeze Vg2k at its DC-OP value (stored per-slot in
    /// `DeviceSlot.vg2k_frozen`).
    /// Phase 1b grid-off pentode map: name → frozen Vg2k value.
    /// The key set acts as the "is this pentode grid-off?" discriminator
    /// during `categorize_element`; the per-entry value is written into
    /// the resulting `NonlinearDeviceInfo.vg2k_frozen` so downstream
    /// codegen can emit it as a per-slot constant.
    grid_off_pentodes: std::collections::HashMap<String, f64>,
}

struct ElementInfo {
    element_type: ElementType,
    nodes: Vec<usize>,
    value: f64,
    name: String,
}

#[derive(Debug, Clone, Copy)]
enum ElementType {
    Resistor,
    Capacitor,
    Inductor,
    VoltageSource,
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
            vcas: Vec::new(),
            elements: Vec::new(),
            total_dimension: 0,
            forward_active_bjts: std::collections::HashSet::new(),
            linearized_bjts: std::collections::HashSet::new(),
            linearized_triodes: std::collections::HashSet::new(),
            grid_off_pentodes: std::collections::HashMap::new(),
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
            if let Element::Opamp { model, .. } = elem {
                if let Some(m) = netlist
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
                            "VSAT" => oa.vsat = *val,
                            "VCC" => oa.vcc = *val,
                            "VEE" => oa.vee = *val,
                            "GBW" => oa.gbw = *val,
                            // SR is specified in V/μs (SPICE convention) and
                            // stored in V/s internally — multiply by 1e6.
                            "SR" => oa.sr = *val * 1.0e6,
                            "VOH_DROP" => oa.voh_drop = *val,
                            "VOL_DROP" => oa.vol_drop = *val,
                            "AOL_TRANSIENT_CAP" => oa.aol_transient_cap = *val,
                            "IB" => oa.ib = *val,
                            "RIN" => oa.rin = *val,
                            _ => log::warn!(
                                ".model {}: unrecognized parameter '{}' (ignored)",
                                m.name,
                                key
                            ),
                        }
                    }
                }
            }
        }

        // Resolve op-amp output voltage clamps from VCC/VEE/VSAT/GBW.
        // Priority: VCC/VEE (explicit) > VSAT (symmetric) > GBW auto-default > none.
        for oa in self.opamps.iter_mut() {
            // Resolve VCC (upper clamp): VCC > +VSAT > GBW auto-default
            if !oa.vcc.is_finite() {
                if oa.vsat.is_finite() {
                    oa.vcc = oa.vsat;
                } else if oa.gbw.is_finite() {
                    oa.vcc = 13.0;
                }
            }
            // Resolve VEE (lower clamp): VEE > -VSAT > GBW auto-default
            if !oa.vee.is_finite() {
                if oa.vsat.is_finite() {
                    oa.vee = -oa.vsat;
                } else if oa.gbw.is_finite() {
                    oa.vee = -13.0;
                }
            }
            if oa.vcc.is_finite() || oa.vee.is_finite() {
                log::debug!(
                    "Op-amp {}: output clamp VCC={:.1}V, VEE={:.1}V",
                    oa.name,
                    oa.vcc,
                    oa.vee
                );
            }
        }

        // Resolve VCA model parameters from netlist .model directives
        for (vca, elem) in self.vcas.iter_mut().zip(
            netlist
                .elements
                .iter()
                .filter(|e| matches!(e, Element::Vca { .. })),
        ) {
            if let Element::Vca { model, .. } = elem {
                if let Some(m) = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model))
                {
                    if m.model_type != "VCA" {
                        return Err(MnaError::InvalidParameter(format!(
                            "VCA {} references model '{}' with type '{}', expected 'VCA'",
                            model, m.name, m.model_type
                        )));
                    }
                    for (key, val) in &m.params {
                        match key.to_ascii_uppercase().as_str() {
                            "VSCALE" => vca.vscale = *val,
                            "G0" => vca.g0 = *val,
                            "MODE" => vca.current_mode = *val != 0.0,
                            _ => log::warn!(
                                ".model {}: unrecognized VCA parameter '{}' (ignored)",
                                m.name,
                                key
                            ),
                        }
                    }
                }
            }
        }

        // Create MNA system with correct dimensions
        let n = self.next_node_idx - 1; // Exclude ground
        if n > MAX_N {
            return Err(MnaError::TopologyError(format!(
                "Circuit has {} nodes, exceeding MAX_N={}",
                n, MAX_N
            )));
        }
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
                // Use pot default if specified, otherwise fall back to component value
                let nominal_r = pot_dir.default_value.unwrap_or(*value);
                mna.pots.push(PotInfo {
                    name: pot_dir.resistor_name.clone(),
                    node_p,
                    node_q,
                    g_nominal: 1.0 / nominal_r,
                    min_resistance: pot_dir.min_value,
                    max_resistance: pot_dir.max_value,
                    grounded,
                    runtime_field: None,
                });
            } else {
                return Err(MnaError::TopologyError(format!(
                    ".pot references resistor '{}' which was not found",
                    pot_dir.resistor_name
                )));
            }
        }

        // Resolve .runtime R directives. These share the pot table so
        // rebuild_matrices / DK-kernel / nodal-emitter machinery applies
        // unchanged; `runtime_field = Some(...)` tells downstream codegen
        // to emit `set_runtime_R_<field>` (with a read-only accessor)
        // rather than `set_pot_N`. Setter bodies are otherwise identical
        // since the 2026-04-20 reseed strip.
        for rr_dir in &netlist.runtime_resistors {
            let resistor = netlist.elements.iter().find(|e| {
                matches!(e, Element::Resistor { name, .. } if name.eq_ignore_ascii_case(&rr_dir.resistor_name))
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
                    name: rr_dir.resistor_name.clone(),
                    node_p,
                    node_q,
                    g_nominal: 1.0 / *value,
                    min_resistance: rr_dir.min_value,
                    max_resistance: rr_dir.max_value,
                    grounded,
                    runtime_field: Some(rr_dir.field_name.clone()),
                });
            } else {
                return Err(MnaError::TopologyError(format!(
                    ".runtime R references resistor '{}' which was not found",
                    rr_dir.resistor_name
                )));
            }
        }

        // Build pot default override map: resistor name → default resistance.
        // When a .pot has a default value that differs from the component value,
        // the G matrix should be stamped at the pot default, not the component value.
        let pot_default_overrides: std::collections::HashMap<String, f64> = netlist
            .pots
            .iter()
            .filter_map(|p| {
                p.default_value
                    .map(|dv| (p.resistor_name.to_ascii_uppercase(), dv))
            })
            .collect();
        mna.pot_default_overrides = pot_default_overrides;

        // Resolve wiper group directives: find the two pot indices for each wiper
        for wiper_dir in &netlist.wipers {
            let cw_idx = mna
                .pots
                .iter()
                .position(|p| p.name.eq_ignore_ascii_case(&wiper_dir.resistor_cw));
            let ccw_idx = mna
                .pots
                .iter()
                .position(|p| p.name.eq_ignore_ascii_case(&wiper_dir.resistor_ccw));
            if let (Some(cw), Some(ccw)) = (cw_idx, ccw_idx) {
                mna.wiper_groups.push(WiperGroupInfo {
                    cw_pot_index: cw,
                    ccw_pot_index: ccw,
                    total_resistance: wiper_dir.total_resistance,
                    default_position: wiper_dir.default_position.unwrap_or(0.5),
                    label: wiper_dir.label.clone(),
                });
            }
            // If not found, the pot validation already caught it
        }

        // Resolve gang directives: link pot/wiper indices
        for gang_dir in &netlist.gangs {
            let mut pot_members = Vec::new();
            let mut wiper_members = Vec::new();

            for member in &gang_dir.members {
                let name_upper = member.resistor_name.to_ascii_uppercase();

                // Check if this member is a pot
                if let Some(pot_idx) = mna
                    .pots
                    .iter()
                    .position(|p| p.name.eq_ignore_ascii_case(&name_upper))
                {
                    // Check if this pot belongs to a wiper group
                    let in_wiper = mna.wiper_groups.iter().enumerate().find(|(_, wg)| {
                        wg.cw_pot_index == pot_idx || wg.ccw_pot_index == pot_idx
                    });
                    if let Some((wg_idx, _)) = in_wiper {
                        // This is a wiper member — add the wiper group (avoid duplicates)
                        if !wiper_members.iter().any(|&(idx, _): &(usize, bool)| idx == wg_idx) {
                            wiper_members.push((wg_idx, member.inverted));
                        }
                    } else {
                        // This is a standalone pot member
                        pot_members.push((pot_idx, member.inverted));
                    }
                }
                // If not found in pots, it might be a wiper resistor name that wasn't
                // expanded. The parser validation already caught missing references.
            }

            mna.gang_groups.push(GangGroupInfo {
                label: gang_dir.label.clone(),
                pot_members,
                wiper_members,
                default_position: gang_dir.default_position.unwrap_or(0.5),
            });
        }

        // Resolve switch directives.
        //
        // The initial state of a switch is always position 0, so G/C/L are stamped
        // at the position-0 value (the "canonical baseline") rather than the static
        // netlist declaration. This keeps the initial state self-consistent: the
        // default matrices reflect the default position, and `set_switch_N(non-zero)`
        // applies the correct incremental delta from the pos-0 baseline.
        //
        // Circuit authors who write `R_gain oa_neg 0 10k` but then list `43k 10k ...`
        // as the switch positions no longer need the declaration to match pos-0 — we
        // log a note so the mismatch is visible, but the simulation behaves correctly.
        for sw_dir in &netlist.switches {
            let mut components = Vec::new();
            for (ci, comp_name) in sw_dir.component_names.iter().enumerate() {
                // For expanded subcircuit names like "X1.C1", use base name after last dot
                let base = comp_name.rsplit('.').next().unwrap_or(comp_name);
                let first_char = base.chars().next().unwrap_or(' ').to_ascii_uppercase();
                let pos_0 = sw_dir.positions[0][ci];
                let (node_p, node_q, static_value) = match first_char {
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
                if (static_value - pos_0).abs() > static_value.abs() * 1e-12 {
                    log::info!(
                        ".switch {}: component '{}' declared at {:.6e} but pos-0 is {:.6e}; \
                         stamping pos-0 as the initial value",
                        comp_name, comp_name, static_value, pos_0,
                    );
                }
                mna.switch_default_overrides
                    .insert(comp_name.to_ascii_uppercase(), (first_char, pos_0));
                components.push(SwitchComponentInfo {
                    name: comp_name.clone(),
                    component_type: first_char,
                    node_p,
                    node_q,
                    nominal_value: pos_0,
                });
            }
            mna.switches.push(SwitchInfo {
                components,
                positions: sw_dir.positions.clone(),
            });
        }

        // Apply switch pos-0 overrides to already-collected uncoupled inductor values.
        // `self.inductors` was populated during `categorize_element` (before switch
        // resolution), so its `value` field still holds the static netlist declaration.
        // `build_augmented_matrices` and the DK companion model read this directly, so
        // we normalize it here to keep the augmented MNA consistent with G/C.
        for ind in &mut self.inductors {
            if let Some((_, pos_0)) = mna
                .switch_default_overrides
                .get(&ind.name.to_ascii_uppercase())
            {
                ind.value = *pos_0;
            }
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
            isat: Option<f64>,
        }
        let mut inductor_refs: std::collections::HashMap<String, InductorRef> =
            std::collections::HashMap::new();
        for coupling in &netlist.couplings {
            for ind_name in [&coupling.inductor1_name, &coupling.inductor2_name] {
                let lower = ind_name.to_ascii_lowercase();
                if inductor_refs.contains_key(&lower) {
                    continue;
                }
                if let Some(Element::Inductor { name, n_plus, n_minus, value, isat }) =
                    netlist.elements.iter().find(|e| {
                        matches!(e, Element::Inductor { name, .. } if name.eq_ignore_ascii_case(ind_name))
                    })
                {
                    // Apply switch pos-0 override so coupled-inductor paths
                    // (CoupledInductorInfo, TransformerGroupInfo, ideal-transformer
                    // decomposition) use the same initial L as the augmented C matrix.
                    let effective_value = mna
                        .switch_default_overrides
                        .get(&name.to_ascii_uppercase())
                        .filter(|(kind, _)| *kind == 'L')
                        .map(|(_, v)| *v)
                        .unwrap_or(*value);
                    inductor_refs.insert(lower, InductorRef {
                        name: name.clone(),
                        node_i: self.node_map[n_plus],
                        node_j: self.node_map[n_minus],
                        value: effective_value,
                        isat: *isat,
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
                // total_cmp is NaN-safe; parser already rejects non-finite L
                // at parse_positive_value, but programmatic construction could
                // still produce NaN and panic partial_cmp().unwrap().
                let ref_idx = members
                    .iter()
                    .enumerate()
                    .max_by(|(_, a), (_, b)| {
                        inductor_refs[*a]
                            .value
                            .total_cmp(&inductor_refs[*b].value)
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
                        isat: None,
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
                    isat: None,
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
                    l1_isat: r1.isat,
                    l2_isat: r2.isat,
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
                let mut winding_isats = Vec::with_capacity(w);
                for m in &members {
                    let r = &inductor_refs[m];
                    winding_node_i.push(r.node_i);
                    winding_node_j.push(r.node_j);
                    inductances.push(r.value);
                    winding_names.push(r.name.clone());
                    winding_isats.push(r.isat);
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
                    winding_isats,
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
        mna.vcas = self.vcas;

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
        // IIR op-amp model: GBW op-amps no longer create internal nodes.
        // The dominant pole is modeled as an external IIR filter in codegen.
        let num_opamp_internal = 0;
        // Count current-mode VCAs (each needs an augmented row for current sensing)
        // Each current-mode VCA needs 2 augmented rows:
        // [0] internal node (sig+_int) with dummy R to ground
        // [1] sensing source branch current
        let num_vca_augmented = mna.vcas.iter().filter(|v| v.current_mode).count() * 2;
        let n_aug =
            n_base + num_vs + num_vcvs + num_ideal_xfmr + num_opamp_internal + num_vca_augmented;
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

        // Resolve `.runtime` directives to aug-MNA rows. Parser already
        // validated each directive's VS name exists; here we translate the
        // name to its stamped row. The row is `n_base + ext_idx` to match
        // the aug-MNA convention used by the VS stamping loop above.
        for rt in &netlist.runtime_sources {
            let vs_idx = mna
                .voltage_sources
                .iter()
                .position(|v| v.name.eq_ignore_ascii_case(&rt.vs_name))
                .ok_or_else(|| {
                    MnaError::TopologyError(format!(
                        ".runtime references voltage source '{}' that was not resolved in MNA",
                        rt.vs_name
                    ))
                })?;
            let vs = &mna.voltage_sources[vs_idx];
            mna.runtime_sources.push(RuntimeSourceInfo {
                vs_name: vs.name.clone(),
                field_name: rt.field_name.clone(),
                vs_row: n_base + vs.ext_idx,
            });
        }

        // Stamp VCVS elements with augmented MNA.
        // For VCVS: V_out+ - V_out- = gain*(V_ctrl+ - V_ctrl-)
        //   Extra unknown j_vcvs at row/col k = n + num_vs + vcvs_idx
        //   Current injection column k: enters out+, exits out-
        //   KVL constraint row k: G[k][out+]=+1, G[k][out-]=-1,
        //                          G[k][ctrl+]=-gain, G[k][ctrl-]=+gain
        let mut vcvs_idx = 0;
        for elem in &self.elements {
            if let ElementType::Vcvs = elem.element_type {
                if elem.nodes.len() >= 4 {
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
                        // Override priority: switch pos-0 > pot default > static value.
                        // Switch wins because a component can only be in one .switch and
                        // pos-0 defines the canonical initial stamp. Pots and switches
                        // on the same resistor are nonsensical; switch takes precedence.
                        let key = elem.name.to_ascii_uppercase();
                        let r = mna
                            .switch_default_overrides
                            .get(&key)
                            .filter(|(kind, _)| *kind == 'R')
                            .map(|(_, v)| *v)
                            .or_else(|| mna.pot_default_overrides.get(&key).copied())
                            .unwrap_or(elem.value);
                        let g = 1.0 / r;
                        stamp_conductance_to_ground(&mut mna.g, elem.nodes[0], elem.nodes[1], g);
                    }
                }
                ElementType::Capacitor => {
                    if elem.nodes.len() >= 2 {
                        // Switch pos-0 override for switched capacitors (e.g. tone-switch caps)
                        let c_val = mna
                            .switch_default_overrides
                            .get(&elem.name.to_ascii_uppercase())
                            .filter(|(kind, _)| *kind == 'C')
                            .map(|(_, v)| *v)
                            .unwrap_or(elem.value);
                        stamp_conductance_to_ground(
                            &mut mna.c,
                            elem.nodes[0],
                            elem.nodes[1],
                            c_val,
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
            }
        }

        // Stamp op-amps as VCCS into G matrix.
        //
        // Three stamping modes, in priority order:
        //
        // 1. **BoyleDiodes internal gain node** — auto-detected by looking up
        //    `_oa_int_{safe_name}` in `node_map`. The augment helper
        //    `codegen::ir::augment_netlist_with_boyle_diodes` synthesizes
        //    catch diodes + a unity-gain output buffer that reference this
        //    node, so its presence in `node_map` is the signal that this
        //    op-amp is in BoyleDiodes mode. Gm/Go are stamped at the
        //    internal node with `R_BOYLE_INT_LOAD = 1 MΩ` as the effective
        //    output resistance — making the catch diode's exponential
        //    conductance only have to balance against ~1 µS instead of
        //    ~4000 S. The original output node is left untouched (the
        //    buffer VCCS handles it).
        //
        // 2. **GBW IIR model** — when `oa.gbw` is finite, stamp at the
        //    output for DC OP. The dominant pole is stripped and re-applied
        //    in codegen as an external IIR filter (currently disabled
        //    behind `if !has_vca && false`, kept for future re-enable).
        //
        // 3. **Simple linear VCCS** — direct stamp at the output node.
        //    Original behavior; used for all op-amps that aren't in the
        //    BoyleDiodes augmentation path.
        for oa in &mut mna.opamps {
            let out = oa.n_out_idx;
            let np = oa.n_plus_idx;
            let nm = oa.n_minus_idx;

            if out == 0 {
                continue;
            }

            // BoyleDiodes auto-detection: look up the synthesized internal
            // node by name. If present, switch to internal-node stamping for
            // this op-amp and skip the rest of the dispatch.
            let safe_name: String = oa
                .name
                .chars()
                .map(|c| {
                    if c.is_ascii_alphanumeric() || c == '_' {
                        c
                    } else {
                        '_'
                    }
                })
                .collect();
            let int_node_key = format!("_oa_int_{}", safe_name);
            if let Some(&int_idx_one) = mna.node_map.get(&int_node_key) {
                // 1-indexed in node_map; convert to 0-indexed matrix row.
                let int_row = int_idx_one - 1;
                // Effective Gm/Go derived from R_BOYLE_INT_LOAD so the
                // catch diode (anchored externally to this node) only
                // fights ~1 µS, not the op-amp's nominal 1/r_out.
                let gm_int = oa.aol / R_BOYLE_INT_LOAD;
                let go_int = 1.0 / R_BOYLE_INT_LOAD;
                if np > 0 {
                    mna.g[int_row][np - 1] += gm_int;
                }
                if nm > 0 {
                    mna.g[int_row][nm - 1] -= gm_int;
                }
                mna.g[int_row][int_row] += go_int;

                oa.n_int_idx = int_idx_one;
                log::debug!(
                    "Op-amp {} (BoyleDiodes): int_node={}, Gm_int={:.3e}, Go_int={:.3e}",
                    oa.name,
                    int_idx_one,
                    gm_int,
                    go_int,
                );
                continue;
            }

            let gm = oa.aol / oa.r_out;
            let go = 1.0 / oa.r_out;
            let o = out - 1;

            let has_gbw = oa.gbw.is_finite() && oa.gbw > 0.0;

            if has_gbw {
                // IIR op-amp model: stamp Gm at output (same as non-GBW) for DC OP.
                // The GBW dominant pole is modeled as an external IIR filter in codegen,
                // which strips Gm from G before building A for transient simulation.
                // NO internal node is created — this avoids Boyle's Gm~4000 S conditioning disaster.
                if np > 0 {
                    mna.g[o][np - 1] += gm;
                }
                if nm > 0 {
                    mna.g[o][nm - 1] -= gm;
                }
                mna.g[o][o] += go;

                // Store IIR parameters for codegen
                let c_dom = oa.aol / (2.0 * std::f64::consts::PI * oa.gbw * oa.r_out);
                oa.iir_c_dom = c_dom;
                // n_internal_idx stays 0 (no internal node)

                log::debug!(
                    "Op-amp {} (IIR): GBW={:.0}Hz, C_dom={:.3e}F, Gm={:.2}, Go={:.4}",
                    oa.name,
                    oa.gbw,
                    c_dom,
                    gm,
                    go,
                );
            } else {
                // Simple VCCS (no GBW): direct stamp at output (original behavior)
                if np > 0 {
                    mna.g[o][np - 1] += gm;
                }
                if nm > 0 {
                    mna.g[o][nm - 1] -= gm;
                }
                mna.g[o][o] += go;
            }

            // Input-stage parasitics (IB + RIN): tiny effects that matter for
            // circuits where the op-amp input node is a high-impedance
            // integrator (e.g. SSL 4kbuscomp sidechain U10 where a 3.3 MΩ /
            // 10 pF integrator winds up unboundedly under any DC offset
            // without a bleed path to ground). Default IB=0 / RIN=+∞
            // preserves ideal-op-amp behavior byte-identically.
            //
            // IB sign convention: positive IB injects `+IB` at both input
            // nodes (current flowing out of the op-amp pin into the external
            // circuit — PNP-input/JFET default). For NPN-input parts specify
            // negative IB in the .model card.
            if oa.ib != 0.0 {
                if np > 0 {
                    mna.current_sources.push(CurrentSourceInfo {
                        name: format!("_{}_IB_plus", oa.name),
                        n_plus_idx: np,
                        n_minus_idx: 0,
                        dc_value: oa.ib,
                    });
                }
                if nm > 0 {
                    mna.current_sources.push(CurrentSourceInfo {
                        name: format!("_{}_IB_minus", oa.name),
                        n_plus_idx: nm,
                        n_minus_idx: 0,
                        dc_value: oa.ib,
                    });
                }
            }
            // RIN: shunt conductance 1/RIN from each input pin to ground.
            // Physical significance: bounds integrator wind-up via a DC
            // leakage path (τ_leak = RIN · C_integ). For TL074 JFET input
            // RIN=1e12 gives g_in=1pS, effectively zero but finite — enough
            // to pull any accumulated offset to ground over ~seconds of
            // circuit time, preventing unbounded growth that the infinite-
            // input-Z ideal model produces.
            if oa.rin.is_finite() && oa.rin > 0.0 {
                let g_in = 1.0 / oa.rin;
                if np > 0 {
                    mna.g[np - 1][np - 1] += g_in;
                }
                if nm > 0 {
                    mna.g[nm - 1][nm - 1] += g_in;
                }
            }
        }

        // Allocate augmented rows for current-mode VCA sensing sources
        let vca_aug_base = n_base + num_vs + num_vcvs + num_ideal_xfmr + num_opamp_internal;
        let mut vca_aug_idx = 0;
        for vca in &mut mna.vcas {
            if vca.current_mode {
                let int_node = vca_aug_base + vca_aug_idx; // sig+_int
                let sense_row = vca_aug_base + vca_aug_idx + 1; // branch current
                vca_aug_idx += 2;
                vca.n_internal_idx = int_node + 1; // 1-indexed
                vca.n_sense_idx = sense_row + 1; // 1-indexed

                // Stamp virtual-ground terminator from sig+_int to ground.
                //
                // A real current-mode VCA (THAT 2180 / DBX 2150) holds sig+
                // at virtual ground via the log-antilog feedback loop. The
                // sense source between sig+ and sig+_int copies that
                // potential to sig+_int; the terminator here forces sig+_int
                // toward 0 V so sig+ sits near ground. Rdummy must be
                // substantially smaller than any realistic series drive
                // resistor, otherwise the drive resistor and Rdummy form a
                // current divider that attenuates the sensed input current
                // by Rdummy / (Rdrive + Rdummy).
                //
                // The original Rdummy=1 MΩ caused a ~32 dB passband loss on
                // circuits like 4kbuscomp-audiopath (Rdrive=27 kΩ): measured
                // −36.77 dB instead of the expected −5.1 dB (15K/27K
                // transimpedance ratio). At Rdummy=1 Ω the divider error is
                // <0.001 dB for any drive resistor ≥1 Ω.
                let g_dummy = 1.0; // 1 ohm
                mna.g[int_node][int_node] += g_dummy;

                log::debug!(
                    "VCA {} (current mode): internal={}, sense={}, Rdummy=1 ohm",
                    vca.name,
                    int_node,
                    sense_row
                );
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

        for (dev_type, dim, start_idx, node_indices, dev_name) in device_info {
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
                    // Tube device family. Three shapes:
                    //   - Triode (dim=2, 3 nodes): [grid, plate, cathode]
                    //   - Pentode (dim=3, 4-5 nodes): [plate, grid, cathode, screen, (suppressor?)]
                    //   - Grid-off pentode (dim=2, 4-5 nodes): [plate, grid, cathode, screen, (suppressor?)]
                    // The node ordering inside `node_indices` matches the layout
                    // produced by `categorize_element`. Triode vs grid-off pentode
                    // are distinguished by node_indices.len() (3 vs 4+).
                    let is_pentode_shape = node_indices.len() >= 4;
                    if dim == 2 && is_pentode_shape {
                        // Grid-off pentode: 2D NR (Vgk→Ip, Vpk→Ig2). Ig1 is
                        // dropped (grid-cutoff); Vg2k is frozen in the device
                        // math, not stamped as an N_v row.
                        let p_raw = node_indices[0];
                        let g_raw = node_indices[1];
                        let k_raw = node_indices[2];
                        let s_raw = node_indices[3]; // screen (g2)

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
                        // N_i col 1 (Ig2): -1 at screen, +1 at cathode
                        if s_raw > 0 {
                            mna.n_i[s_raw - 1][start_idx + 1] = -1.0;
                        }
                        if k_raw > 0 {
                            mna.n_i[k_raw - 1][start_idx + 1] = 1.0;
                        }
                    } else if dim == 3 {
                        // Pentode 3D layout (rows / columns must agree so the
                        // 3x3 device Jacobian block stays consistent):
                        //   row/col 0: Ip   ↔ Vgk
                        //   row/col 1: Ig2  ↔ Vpk
                        //   row/col 2: Ig1  ↔ Vg2k
                        //
                        // The codegen template (device_tube.rs.tera) and the
                        // DC-OP solver consume this exact ordering — do NOT
                        // reshuffle it without updating those consumers.
                        //
                        // Phase 1a: the optional suppressor node (node_indices[4],
                        // when present) is treated as electrically tied to the
                        // cathode. We do not stamp any N_v / N_i entries for it.
                        // This is the universal case for audio power tubes
                        // (6L6/6V6/KT88 beam tetrodes, EL84/EL34 strapped pentodes,
                        // and EF86 whose suppressor is wired to cathode externally).
                        // TODO(phase 1b): if a user ever wires the suppressor to
                        // a non-cathode node we silently model it as cathode-tied.
                        if node_indices.len() >= 4 {
                            let p_raw = node_indices[0];
                            let g_raw = node_indices[1];
                            let k_raw = node_indices[2];
                            let s_raw = node_indices[3]; // screen (g2)

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
                            // N_v row 2 (Vg2k): +1 at screen, -1 at cathode
                            if s_raw > 0 {
                                mna.n_v[start_idx + 2][s_raw - 1] = 1.0;
                            }
                            if k_raw > 0 {
                                mna.n_v[start_idx + 2][k_raw - 1] = -1.0;
                            }

                            // N_i col 0 (Ip): -1 at plate (extracted), +1 at cathode (injected)
                            if p_raw > 0 {
                                mna.n_i[p_raw - 1][start_idx] = -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx] = 1.0;
                            }
                            // N_i col 1 (Ig2): -1 at screen, +1 at cathode
                            if s_raw > 0 {
                                mna.n_i[s_raw - 1][start_idx + 1] = -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx + 1] = 1.0;
                            }
                            // N_i col 2 (Ig1): -1 at grid, +1 at cathode
                            if g_raw > 0 {
                                mna.n_i[g_raw - 1][start_idx + 2] = -1.0;
                            }
                            if k_raw > 0 {
                                mna.n_i[k_raw - 1][start_idx + 2] = 1.0;
                            }
                        }
                    } else if node_indices.len() >= 3 {
                        // Triode 2D — Ip (plate current) + Ig (grid current)
                        // Nodes: [ng, np, nk] (grid, plate, cathode)
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
                NonlinearDeviceType::Vca => {
                    // VCA: 2D — dim 0: signal, dim 1: control (I_ctrl = 0)
                    // 4 terminals: [sig_p, sig_n, ctrl_p, ctrl_n]
                    if node_indices.len() >= 4 {
                        let sp_raw = node_indices[0]; // sig+
                        let sn_raw = node_indices[1]; // sig-
                        let cp_raw = node_indices[2]; // ctrl+
                        let cn_raw = node_indices[3]; // ctrl-

                        // Find the VcaInfo for this device to check current_mode
                        let vca_idx = mna
                            .vcas
                            .iter()
                            .position(|v| v.name == dev_name)
                            .unwrap_or(0);
                        let is_current_mode = mna.vcas[vca_idx].current_mode;

                        if is_current_mode {
                            // CURRENT MODE (THAT 2180 style translinear):
                            //   I_out = G(Vc) * I_in
                            //
                            // Uses an internal node (sig+_int) to break the path between
                            // sig+ and sig-. The 0V sensing source between sig+ and
                            // sig+_int measures I_in. A dummy 1MEG resistor terminates
                            // sig+_int to ground (the VCA absorbs the input current).
                            // N_i injects G*I_in at sig- independently — no linear
                            // pass-through from sig+ to sig-.
                            //
                            // This matches ngspice's CCCS topology:
                            //   Vsense sig+ sig+_int 0
                            //   Rdummy sig+_int 0 1MEG
                            //   Bvca 0 sig- I={G*I(Vsense)}
                            let k = mna.vcas[vca_idx].n_sense_idx;
                            let int = mna.vcas[vca_idx].n_internal_idx;
                            if k > 0 && int > 0 {
                                let k0 = k - 1; // sense row (0-indexed)
                                let int0 = int - 1; // internal node (0-indexed)

                                // 0V sensing source between sig+ and sig+_int
                                // KVL: V(sig+) - V(sig+_int) = 0
                                if sp_raw > 0 {
                                    mna.g[k0][sp_raw - 1] += 1.0;
                                    mna.g[sp_raw - 1][k0] += 1.0;
                                }
                                mna.g[k0][int0] -= 1.0;
                                mna.g[int0][k0] -= 1.0;
                                // (Rdummy at sig+_int already stamped during allocation)

                                // N_v row 0: extract branch current (sense variable)
                                mna.n_v[start_idx][k0] = 1.0;

                                // N_i col 0: inject I_out = G*I_in at sig- ONLY
                                // No injection at sig+ — decoupled input/output
                                if sn_raw > 0 {
                                    mna.n_i[sn_raw - 1][start_idx] = 1.0;
                                }
                            }
                        } else {
                            // VOLTAGE MODE (original): I_out = G(Vc) * V_sig
                            // N_v row 0 (V_signal): +1 at sig+, -1 at sig-
                            if sp_raw > 0 {
                                mna.n_v[start_idx][sp_raw - 1] = 1.0;
                            }
                            if sn_raw > 0 {
                                mna.n_v[start_idx][sn_raw - 1] = -1.0;
                            }
                            // N_i col 0 (I_signal): -1 at sig+ (extracted), +1 at sig- (injected)
                            if sp_raw > 0 {
                                mna.n_i[sp_raw - 1][start_idx] = -1.0;
                            }
                            if sn_raw > 0 {
                                mna.n_i[sn_raw - 1][start_idx] = 1.0;
                            }
                        }

                        // N_v row 1 (V_control): +1 at ctrl+, -1 at ctrl- (same for both modes)
                        if cp_raw > 0 {
                            mna.n_v[start_idx + 1][cp_raw - 1] = 1.0;
                        }
                        if cn_raw > 0 {
                            mna.n_v[start_idx + 1][cn_raw - 1] = -1.0;
                        }
                        // N_i col 1 (I_control): NO stamping — control draws no current
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
            Element::Pentode {
                n_plate,
                n_grid,
                n_cathode,
                n_screen,
                n_suppressor,
                ..
            } => {
                let mut nodes = vec![n_plate, n_grid, n_cathode, n_screen];
                if let Some(ns) = n_suppressor {
                    nodes.push(ns);
                }
                nodes
            }
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
            Element::Vca {
                n_sig_p,
                n_sig_n,
                n_ctrl_p,
                n_ctrl_n,
                ..
            } => vec![n_sig_p, n_sig_n, n_ctrl_p, n_ctrl_n],
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
                });
            }
            Element::Inductor {
                name,
                n_plus,
                n_minus,
                value,
                isat,
            } => {
                let node_i = self.node_map[n_plus];
                let node_j = self.node_map[n_minus];
                self.elements.push(ElementInfo {
                    element_type: ElementType::Inductor,
                    nodes: vec![node_i, node_j],
                    value: *value,
                    name: name.clone(),
                });
                // Also add to inductors list for DK kernel companion model
                self.inductors.push(InductorElement {
                    name: name.clone(),
                    node_i,
                    node_j,
                    value: *value,
                    isat: *isat,
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
                    vg2k_frozen: 0.0,
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
                let is_linearized = self.linearized_bjts.contains(&name.to_ascii_uppercase());
                if is_linearized {
                    // Linearized BJTs are removed from the nonlinear system entirely.
                    // Their small-signal conductances are stamped into G after DC OP.
                    // Don't push to nonlinear_devices, don't increment total_dimension.
                    log::info!(
                        "BJT '{}' linearized at DC OP (removed from NR, M reduced by 2)",
                        name
                    );
                } else {
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
                    vg2k_frozen: 0.0,
                    });
                }
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
                    vg2k_frozen: 0.0,
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
                    vg2k_frozen: 0.0,
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
                let is_linearized =
                    self.linearized_triodes.contains(&name.to_ascii_uppercase());
                if is_linearized {
                    // Linearized triodes are removed from the nonlinear system entirely.
                    // Their small-signal gm + 1/rp are stamped into G after DC OP.
                    log::info!(
                        "Triode '{}' linearized at DC OP (removed from NR, M reduced by 2)",
                        name
                    );
                } else {
                    let start_idx = self.total_dimension;
                    self.total_dimension += 2; // 2D: plate current + grid current
                    self.nonlinear_devices.push(NonlinearDeviceInfo {
                        name: name.clone(),
                        device_type: NonlinearDeviceType::Tube,
                        dimension: 2,
                        start_idx,
                        nodes: vec![n_grid.clone(), n_plate.clone(), n_cathode.clone()],
                        node_indices,
                        vg2k_frozen: 0.0,
                    });
                }
            }
            Element::Pentode {
                name,
                n_plate,
                n_grid,
                n_cathode,
                n_screen,
                n_suppressor,
                ..
            } => {
                // 3D (sharp-cutoff) contribution: Ip (Vgk, Vpk, Vg2k),
                // Ig2 (same voltages), Ig1 (Vgk-only). Phase 1b adds an
                // optional 2D grid-off reduction when DC-OP confirms Vgk
                // is below cutoff — Ig1 drops out entirely and Vg2k is
                // frozen at its DC-OP value (stored per-slot in
                // `DeviceSlot.vg2k_frozen`).
                //
                // Node layout in `nodes` / `node_indices` is plate-grid-cathode-screen
                // (+ optional suppressor). Downstream MNA stamping code must use
                // this order for N_v / N_i construction in BOTH the 3D and 2D
                // grid-off paths.
                let mut nodes = vec![
                    n_plate.clone(),
                    n_grid.clone(),
                    n_cathode.clone(),
                    n_screen.clone(),
                ];
                let mut node_indices = vec![
                    self.node_map[n_plate],
                    self.node_map[n_grid],
                    self.node_map[n_cathode],
                    self.node_map[n_screen],
                ];
                if let Some(ns) = n_suppressor {
                    nodes.push(ns.clone());
                    node_indices.push(self.node_map[ns]);
                }
                let name_upper = name.to_ascii_uppercase();
                let grid_off_entry = self.grid_off_pentodes.get(&name_upper).copied();
                let is_grid_off = grid_off_entry.is_some();
                // Grid-off pentode: 2D NR block (Vgk→Ip, Vpk→Ig2). The
                // device type stays `Tube` — the `TubeKind::SharpPentodeGridOff`
                // discriminator lives on `TubeParams.kind` at the codegen
                // layer, not on `NonlinearDeviceType`.
                let dimension = if is_grid_off { 2 } else { 3 };
                let start_idx = self.total_dimension;
                self.total_dimension += dimension;
                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Tube,
                    dimension,
                    start_idx,
                    nodes,
                    node_indices,
                    vg2k_frozen: grid_off_entry.unwrap_or(0.0),
                });
                log::info!(
                    "Pentode '{}' using {} NR block (grid-off = {})",
                    name,
                    if is_grid_off { "2D" } else { "3D" },
                    is_grid_off
                );
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
                    vsat: f64::INFINITY,
                    vcc: f64::INFINITY,
                    vee: f64::NEG_INFINITY,
                    gbw: f64::INFINITY,
                    sr: f64::INFINITY,
                    ib: 0.0,
                    rin: f64::INFINITY,
                    // Boyle-macromodel default: TL072/NE5532-class parts can swing
                    // to within ~1.5 V of each rail under typical load. Rail-to-rail
                    // parts should override this in their .model OA() entry.
                    voh_drop: 1.5,
                    vol_drop: 1.5,
                    aol_transient_cap: f64::INFINITY,
                    n_internal_idx: 0,
                    iir_c_dom: 0.0,
                    n_int_idx: 0,
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
                });
            }
            Element::Vca {
                name,
                n_sig_p,
                n_sig_n,
                n_ctrl_p,
                n_ctrl_n,
                ..
            } => {
                let sp = self.node_map[n_sig_p];
                let sn = self.node_map[n_sig_n];
                let cp = self.node_map[n_ctrl_p];
                let cn = self.node_map[n_ctrl_n];

                let node_indices = vec![sp, sn, cp, cn];

                if node_indices.iter().all(|&idx| idx == 0) {
                    return Err(MnaError::TopologyError(format!(
                        "VCA '{}' has all terminals grounded",
                        name
                    )));
                }

                let start_idx = self.total_dimension;
                self.total_dimension += 2; // 2D: I_signal(V_sig, V_ctrl) + I_control(=0)

                self.nonlinear_devices.push(NonlinearDeviceInfo {
                    name: name.clone(),
                    device_type: NonlinearDeviceType::Vca,
                    dimension: 2,
                    start_idx,
                    nodes: vec![
                        n_sig_p.clone(),
                        n_sig_n.clone(),
                        n_ctrl_p.clone(),
                        n_ctrl_n.clone(),
                    ],
                    node_indices,
                    vg2k_frozen: 0.0,
                });

                self.vcas.push(VcaInfo {
                    name: name.clone(),
                    n_sig_p_idx: sp,
                    n_sig_n_idx: sn,
                    n_ctrl_p_idx: cp,
                    n_ctrl_n_idx: cn,
                    vscale: 0.05298, // default, resolved from model later
                    g0: 1.0,
                    current_mode: false,
                    n_sense_idx: 0,
                    n_internal_idx: 0,
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

    // ===== VCA MNA tests =====

    #[test]
    fn test_mna_vca_basic() {
        let spice = r#"VCA Test
R1 sig_in 0 1k
R2 sig_out 0 1k
R3 ctrl 0 100k
Y1 sig_in sig_out ctrl 0 vca1
.model vca1 VCA(VSCALE=0.05298 G0=1.0)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.m, 2, "VCA should add M=2 (signal + control dimensions)");
        assert_eq!(mna.num_devices, 1, "Should have 1 nonlinear device");
        assert_eq!(mna.vcas.len(), 1, "Should have 1 VCA");
        assert_eq!(
            mna.nonlinear_devices[0].device_type,
            NonlinearDeviceType::Vca
        );
        assert_eq!(mna.nonlinear_devices[0].dimension, 2);
        assert_eq!(mna.nonlinear_devices[0].start_idx, 0);

        // Check VCA model parameters resolved
        assert!((mna.vcas[0].vscale - 0.05298).abs() < 1e-10);
        assert!((mna.vcas[0].g0 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_mna_vca_nv_stamping() {
        // VCA with all terminals non-ground
        let spice = r#"VCA N_v Test
R1 sp 0 1k
R2 sn 0 1k
R3 cp 0 100k
R4 cn 0 100k
Y1 sp sn cp cn vca1
.model vca1 VCA(VSCALE=0.05298 G0=1.0)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sp_idx = *mna.node_map.get("sp").unwrap();
        let sn_idx = *mna.node_map.get("sn").unwrap();
        let cp_idx = *mna.node_map.get("cp").unwrap();
        let cn_idx = *mna.node_map.get("cn").unwrap();

        // N_v row 0 (V_signal): +1 at sig+, -1 at sig-
        assert_eq!(mna.n_v[0][sp_idx - 1], 1.0, "N_v[0][sig+] should be +1");
        assert_eq!(mna.n_v[0][sn_idx - 1], -1.0, "N_v[0][sig-] should be -1");

        // N_v row 1 (V_control): +1 at ctrl+, -1 at ctrl-
        assert_eq!(mna.n_v[1][cp_idx - 1], 1.0, "N_v[1][ctrl+] should be +1");
        assert_eq!(mna.n_v[1][cn_idx - 1], -1.0, "N_v[1][ctrl-] should be -1");

        // Verify no cross-contamination: signal row has no ctrl nodes
        assert_eq!(mna.n_v[0][cp_idx - 1], 0.0, "N_v[0][ctrl+] should be 0");
        assert_eq!(mna.n_v[0][cn_idx - 1], 0.0, "N_v[0][ctrl-] should be 0");
        // Control row has no signal nodes
        assert_eq!(mna.n_v[1][sp_idx - 1], 0.0, "N_v[1][sig+] should be 0");
        assert_eq!(mna.n_v[1][sn_idx - 1], 0.0, "N_v[1][sig-] should be 0");
    }

    #[test]
    fn test_mna_vca_ni_stamping() {
        // VCA: signal current at sig+/sig-, no control current
        let spice = r#"VCA N_i Test
R1 sp 0 1k
R2 sn 0 1k
R3 cp 0 100k
R4 cn 0 100k
Y1 sp sn cp cn vca1
.model vca1 VCA(VSCALE=0.05298 G0=1.0)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sp_idx = *mna.node_map.get("sp").unwrap();
        let sn_idx = *mna.node_map.get("sn").unwrap();
        let cp_idx = *mna.node_map.get("cp").unwrap();
        let cn_idx = *mna.node_map.get("cn").unwrap();

        // N_i col 0 (I_signal): -1 at sig+ (extracted), +1 at sig- (injected)
        assert_eq!(
            mna.n_i[sp_idx - 1][0],
            -1.0,
            "N_i[sig+][0] should be -1 (current extracted)"
        );
        assert_eq!(
            mna.n_i[sn_idx - 1][0],
            1.0,
            "N_i[sig-][0] should be +1 (current injected)"
        );

        // N_i col 1 (I_control): NO stamping — control draws no current
        assert_eq!(
            mna.n_i[cp_idx - 1][1],
            0.0,
            "N_i[ctrl+][1] should be 0 (no control current)"
        );
        assert_eq!(
            mna.n_i[cn_idx - 1][1],
            0.0,
            "N_i[ctrl-][1] should be 0 (no control current)"
        );

        // Also verify no signal current in control nodes
        assert_eq!(mna.n_i[cp_idx - 1][0], 0.0, "N_i[ctrl+][0] should be 0");
        assert_eq!(mna.n_i[cn_idx - 1][0], 0.0, "N_i[ctrl-][0] should be 0");

        // No control current in signal nodes either (col 1)
        assert_eq!(mna.n_i[sp_idx - 1][1], 0.0, "N_i[sig+][1] should be 0");
        assert_eq!(mna.n_i[sn_idx - 1][1], 0.0, "N_i[sig-][1] should be 0");
    }

    // ===== Pentode MNA tests (phase 1a) =====
    //
    // Pentode 3D layout — locked in by row/col convention shared with the
    // codegen template (device_tube.rs.tera) and the DC-OP solver:
    //   row/col 0: Ip   ↔ Vgk
    //   row/col 1: Ig2  ↔ Vpk
    //   row/col 2: Ig1  ↔ Vg2k
    // Phase 1a treats the suppressor (n_suppressor) as cathode-tied; no
    // N_v/N_i entries are stamped for it.

    /// EL84 model directive used by the pentode tests below.
    /// Beam-tetrode-friendly Reefman params (matches parser.rs unit tests).
    const EL84_MODEL: &str = ".model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 \
                              KP=152.4 KVB=4015.8 ALPHA_S=7.66 \
                              A_FACTOR=4.344e-4 BETA_FACTOR=0.148)";

    #[test]
    fn test_pentode_dimension_is_3() {
        let spice = format!(
            "Pentode dimension test\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
            EL84_MODEL
        );
        let netlist = Netlist::parse(&spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.m, 3, "Pentode should add M=3 (Ip + Ig2 + Ig1)");
        assert_eq!(mna.num_devices, 1, "Should have exactly 1 nonlinear device");
        let dev = &mna.nonlinear_devices[0];
        assert_eq!(dev.device_type, NonlinearDeviceType::Tube);
        assert_eq!(dev.dimension, 3);
        assert_eq!(dev.start_idx, 0);
    }

    #[test]
    fn test_pentode_nv_ni_stamping() {
        // Minimal pentode test rig: V1 on plate, resistors on grid/screen/cathode.
        let spice = format!(
            "Pentode N_v / N_i stamping test\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
            EL84_MODEL
        );
        let netlist = Netlist::parse(&spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let p_idx = *mna.node_map.get("plate").unwrap() - 1;
        let g_idx = *mna.node_map.get("grid").unwrap() - 1;
        let k_idx = *mna.node_map.get("cath").unwrap() - 1;
        let s_idx = *mna.node_map.get("screen").unwrap() - 1;

        // ----- N_v rows (controlling voltages extracted from node voltages) -----
        // Row 0 = Vgk = V_grid - V_cathode
        assert_eq!(mna.n_v[0][g_idx], 1.0, "N_v[0][grid] should be +1 (Vgk)");
        assert_eq!(mna.n_v[0][k_idx], -1.0, "N_v[0][cath] should be -1 (Vgk)");
        assert_eq!(mna.n_v[0][p_idx], 0.0, "N_v[0][plate] should be 0");
        assert_eq!(mna.n_v[0][s_idx], 0.0, "N_v[0][screen] should be 0");

        // Row 1 = Vpk = V_plate - V_cathode
        assert_eq!(mna.n_v[1][p_idx], 1.0, "N_v[1][plate] should be +1 (Vpk)");
        assert_eq!(mna.n_v[1][k_idx], -1.0, "N_v[1][cath] should be -1 (Vpk)");
        assert_eq!(mna.n_v[1][g_idx], 0.0, "N_v[1][grid] should be 0");
        assert_eq!(mna.n_v[1][s_idx], 0.0, "N_v[1][screen] should be 0");

        // Row 2 = Vg2k = V_screen - V_cathode
        assert_eq!(mna.n_v[2][s_idx], 1.0, "N_v[2][screen] should be +1 (Vg2k)");
        assert_eq!(mna.n_v[2][k_idx], -1.0, "N_v[2][cath] should be -1 (Vg2k)");
        assert_eq!(mna.n_v[2][p_idx], 0.0, "N_v[2][plate] should be 0");
        assert_eq!(mna.n_v[2][g_idx], 0.0, "N_v[2][grid] should be 0");

        // ----- N_i columns (currents injected into node voltages) -----
        // Col 0 = Ip: extracted from plate, injected into cathode
        assert_eq!(mna.n_i[p_idx][0], -1.0, "N_i[plate][0] should be -1 (Ip out)");
        assert_eq!(mna.n_i[k_idx][0], 1.0, "N_i[cath][0] should be +1 (Ip in)");
        assert_eq!(mna.n_i[g_idx][0], 0.0, "N_i[grid][0] should be 0");
        assert_eq!(mna.n_i[s_idx][0], 0.0, "N_i[screen][0] should be 0");

        // Col 1 = Ig2: extracted from screen, injected into cathode
        assert_eq!(mna.n_i[s_idx][1], -1.0, "N_i[screen][1] should be -1 (Ig2 out)");
        assert_eq!(mna.n_i[k_idx][1], 1.0, "N_i[cath][1] should be +1 (Ig2 in)");
        assert_eq!(mna.n_i[p_idx][1], 0.0, "N_i[plate][1] should be 0");
        assert_eq!(mna.n_i[g_idx][1], 0.0, "N_i[grid][1] should be 0");

        // Col 2 = Ig1: extracted from grid, injected into cathode
        assert_eq!(mna.n_i[g_idx][2], -1.0, "N_i[grid][2] should be -1 (Ig1 out)");
        assert_eq!(mna.n_i[k_idx][2], 1.0, "N_i[cath][2] should be +1 (Ig1 in)");
        assert_eq!(mna.n_i[p_idx][2], 0.0, "N_i[plate][2] should be 0");
        assert_eq!(mna.n_i[s_idx][2], 0.0, "N_i[screen][2] should be 0");
    }

    #[test]
    fn test_pentode_ignores_suppressor() {
        // Build the same circuit two ways: with and without an explicit
        // suppressor node. In phase 1a the suppressor is electrically silent,
        // so the resulting N_v / N_i blocks must be byte-identical for the
        // 4 "real" pentode terminals (plate / grid / cath / screen).
        let spice4 = format!(
            "4-node pentode\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
            EL84_MODEL
        );
        // Same circuit but with `sup` as an explicit 5th terminal.
        // We add an R from sup to ground so it has somewhere to live in the
        // node map; the stamping itself must NOT mention `sup`.
        let spice5 = format!(
            "5-node pentode (suppressor explicit)\n\
             P1 plate grid cath screen sup EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             Rsup sup 0 1\n\
             {}\n",
            EL84_MODEL
        );

        let netlist4 = Netlist::parse(&spice4).unwrap();
        let netlist5 = Netlist::parse(&spice5).unwrap();
        let mna4 = MnaSystem::from_netlist(&netlist4).unwrap();
        let mna5 = MnaSystem::from_netlist(&netlist5).unwrap();

        // Both registrations share the same nonlinear dimension (3D).
        assert_eq!(mna4.m, 3);
        assert_eq!(mna5.m, 3);

        // Look up the four "real" pentode nodes in BOTH MNAs and verify the
        // N_v / N_i entries match exactly. The 5-node case adds an extra
        // node (`sup`) but no additional N_v / N_i contributions.
        let p4 = *mna4.node_map.get("plate").unwrap() - 1;
        let g4 = *mna4.node_map.get("grid").unwrap() - 1;
        let k4 = *mna4.node_map.get("cath").unwrap() - 1;
        let s4 = *mna4.node_map.get("screen").unwrap() - 1;
        let p5 = *mna5.node_map.get("plate").unwrap() - 1;
        let g5 = *mna5.node_map.get("grid").unwrap() - 1;
        let k5 = *mna5.node_map.get("cath").unwrap() - 1;
        let s5 = *mna5.node_map.get("screen").unwrap() - 1;

        for row in 0..3 {
            assert_eq!(mna4.n_v[row][p4], mna5.n_v[row][p5], "N_v row {} plate", row);
            assert_eq!(mna4.n_v[row][g4], mna5.n_v[row][g5], "N_v row {} grid", row);
            assert_eq!(mna4.n_v[row][k4], mna5.n_v[row][k5], "N_v row {} cath", row);
            assert_eq!(
                mna4.n_v[row][s4], mna5.n_v[row][s5],
                "N_v row {} screen",
                row
            );
        }
        for col in 0..3 {
            assert_eq!(mna4.n_i[p4][col], mna5.n_i[p5][col], "N_i col {} plate", col);
            assert_eq!(mna4.n_i[g4][col], mna5.n_i[g5][col], "N_i col {} grid", col);
            assert_eq!(mna4.n_i[k4][col], mna5.n_i[k5][col], "N_i col {} cath", col);
            assert_eq!(
                mna4.n_i[s4][col], mna5.n_i[s5][col],
                "N_i col {} screen",
                col
            );
        }

        // The suppressor row in the 5-node case should be electrically silent
        // — no N_v / N_i contributions tied to it.
        let sup5 = *mna5.node_map.get("sup").unwrap() - 1;
        for row in 0..3 {
            assert_eq!(
                mna5.n_v[row][sup5], 0.0,
                "suppressor must not appear in N_v row {}",
                row
            );
        }
        for col in 0..3 {
            assert_eq!(
                mna5.n_i[sup5][col], 0.0,
                "suppressor must not appear in N_i col {}",
                col
            );
        }
    }

    #[test]
    fn test_pentode_parasitic_caps() {
        // Purely-resistive pentode circuit (no explicit caps in the netlist)
        // should pick up 5 auto-inserted junction caps:
        //   Cgk, Cgp, Cpk, Csk, Csp
        // Each junction adds 2 off-diagonal C entries (symmetric stamp), so
        // 5 junctions => 10 off-diagonal entries.
        let spice = format!(
            "Pentode parasitic\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
            EL84_MODEL
        );
        let netlist = Netlist::parse(&spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.add_parasitic_caps();

        let p = *mna.node_map.get("plate").unwrap() - 1;
        let g = *mna.node_map.get("grid").unwrap() - 1;
        let k = *mna.node_map.get("cath").unwrap() - 1;
        let s = *mna.node_map.get("screen").unwrap() - 1;

        // Each pentode terminal accumulates one diagonal cap per junction it
        // touches. Topology recap: Cgk(g,k), Cgp(g,p), Cpk(p,k), Csk(s,k),
        // Csp(s,p). Resulting junction counts:
        //   plate: Cgp + Cpk + Csp = 3 caps
        //   grid : Cgk + Cgp        = 2 caps
        //   cath : Cgk + Cpk + Csk  = 3 caps
        //   screen: Csk + Csp       = 2 caps
        let approx = |actual: f64, expected_n: usize| {
            let expected = (expected_n as f64) * PARASITIC_CAP;
            (actual - expected).abs() < 1e-25
        };

        assert!(
            approx(mna.c[p][p], 3),
            "C[plate][plate] = {} (expected 3*PARASITIC_CAP)",
            mna.c[p][p]
        );
        assert!(
            approx(mna.c[g][g], 2),
            "C[grid][grid] = {} (expected 2*PARASITIC_CAP)",
            mna.c[g][g]
        );
        assert!(
            approx(mna.c[k][k], 3),
            "C[cath][cath] = {} (expected 3*PARASITIC_CAP)",
            mna.c[k][k]
        );
        assert!(
            approx(mna.c[s][s], 2),
            "C[screen][screen] = {} (expected 2*PARASITIC_CAP)",
            mna.c[s][s]
        );

        // Each junction must show up as a NEGATIVE off-diagonal entry on both
        // sides of the symmetric stamp (the `stamp_capacitor_raw` convention).
        let off = |a: usize, b: usize| (mna.c[a][b] + PARASITIC_CAP).abs() < 1e-25;
        assert!(off(g, k), "C[grid][cath] = {} (Cgk)", mna.c[g][k]);
        assert!(off(k, g), "C[cath][grid] = {} (Cgk)", mna.c[k][g]);
        assert!(off(g, p), "C[grid][plate] = {} (Cgp)", mna.c[g][p]);
        assert!(off(p, g), "C[plate][grid] = {} (Cgp)", mna.c[p][g]);
        assert!(off(p, k), "C[plate][cath] = {} (Cpk)", mna.c[p][k]);
        assert!(off(k, p), "C[cath][plate] = {} (Cpk)", mna.c[k][p]);
        assert!(off(s, k), "C[screen][cath] = {} (Csk)", mna.c[s][k]);
        assert!(off(k, s), "C[cath][screen] = {} (Csk)", mna.c[k][s]);
        assert!(off(s, p), "C[screen][plate] = {} (Csp)", mna.c[s][p]);
        assert!(off(p, s), "C[plate][screen] = {} (Csp)", mna.c[p][s]);

        // Sanity: total off-diagonal nonzero count over the 4 pentode
        // terminals should be exactly 10 (5 junctions x 2 entries each).
        let pentode_nodes = [p, g, k, s];
        let mut off_diag_count = 0;
        for &i in &pentode_nodes {
            for &j in &pentode_nodes {
                if i != j && mna.c[i][j].abs() > 1e-25 {
                    off_diag_count += 1;
                }
            }
        }
        assert_eq!(
            off_diag_count, 10,
            "Pentode should have 10 off-diagonal C entries (5 junctions), got {}",
            off_diag_count
        );
    }

    // ===== Grid-off pentode (phase 1b) tests =====
    //
    // Grid-off reduction drops the Ig1 NR dimension (row/col 2) when DC-OP
    // confirms Vgk is below cutoff, and freezes Vg2k at its DC-OP value.
    // Remaining NR shape:
    //   row/col 0: Ip  ↔ Vgk
    //   row/col 1: Ig2 ↔ Vpk
    // The resulting N_v has 4 nonzero entries; N_i has 4 nonzero entries.

    #[test]
    fn test_stamp_pentode_grid_off_shape() {
        // Build an empty-ish MnaSystem with 4 real nodes (+ ground) and 2
        // NR dimensions, then stamp a grid-off pentode directly. The test
        // verifies ONLY the N_v / N_i shape — no device math, no netlist.
        //
        // We use `from_netlist` to get a consistent allocation; the circuit
        // has 4 resistors so the node map has 4 real indices (1..=4).
        let spice = "grid-off shape rig\n\
                     R1 plate 0 1\n\
                     R2 grid 0 1\n\
                     R3 cath 0 1\n\
                     R4 screen 0 1\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

        // Re-size the nonlinear matrices to have M=2 (grid-off block).
        mna.m = 2;
        mna.n_v = vec![vec![0.0; mna.n]; 2];
        mna.n_i = vec![vec![0.0; 2]; mna.n];

        let p = *mna.node_map.get("plate").unwrap() - 1;
        let g = *mna.node_map.get("grid").unwrap() - 1;
        let k = *mna.node_map.get("cath").unwrap() - 1;
        let s = *mna.node_map.get("screen").unwrap() - 1;

        mna.stamp_pentode_grid_off(0, p, g, k, s);

        // ----- N_v rows -----
        // Row 0 = Vgk
        assert_eq!(mna.n_v[0][g], 1.0, "N_v[0][grid] = +1 (Vgk)");
        assert_eq!(mna.n_v[0][k], -1.0, "N_v[0][cath] = -1 (Vgk)");
        assert_eq!(mna.n_v[0][p], 0.0, "N_v[0][plate] = 0");
        assert_eq!(mna.n_v[0][s], 0.0, "N_v[0][screen] = 0 (Vg2k dropped)");

        // Row 1 = Vpk
        assert_eq!(mna.n_v[1][p], 1.0, "N_v[1][plate] = +1 (Vpk)");
        assert_eq!(mna.n_v[1][k], -1.0, "N_v[1][cath] = -1 (Vpk)");
        assert_eq!(mna.n_v[1][g], 0.0, "N_v[1][grid] = 0");
        assert_eq!(mna.n_v[1][s], 0.0, "N_v[1][screen] = 0");

        // Count nonzero entries — must be exactly 4.
        let nv_nz: usize = mna
            .n_v
            .iter()
            .map(|row| row.iter().filter(|&&v| v != 0.0).count())
            .sum();
        assert_eq!(nv_nz, 4, "grid-off N_v must have exactly 4 nonzero entries");

        // ----- N_i columns -----
        // Col 0 = Ip
        assert_eq!(mna.n_i[p][0], -1.0, "N_i[plate][0] = -1 (Ip out)");
        assert_eq!(mna.n_i[k][0], 1.0, "N_i[cath][0] = +1 (Ip in)");
        assert_eq!(mna.n_i[g][0], 0.0, "N_i[grid][0] = 0 (Ig1 dropped)");
        assert_eq!(mna.n_i[s][0], 0.0, "N_i[screen][0] = 0");

        // Col 1 = Ig2
        assert_eq!(mna.n_i[s][1], -1.0, "N_i[screen][1] = -1 (Ig2 out)");
        assert_eq!(mna.n_i[k][1], 1.0, "N_i[cath][1] = +1 (Ig2 in)");
        assert_eq!(mna.n_i[p][1], 0.0, "N_i[plate][1] = 0");
        assert_eq!(mna.n_i[g][1], 0.0, "N_i[grid][1] = 0");

        let ni_nz: usize = mna
            .n_i
            .iter()
            .map(|row| row.iter().filter(|&&v| v != 0.0).count())
            .sum();
        assert_eq!(ni_nz, 4, "grid-off N_i must have exactly 4 nonzero entries");
    }

    #[test]
    fn test_from_netlist_with_grid_off_reduces_dimension() {
        let spice = format!(
            "grid-off dimension reduction\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
            EL84_MODEL
        );
        let netlist = Netlist::parse(&spice).unwrap();

        let mna_full = MnaSystem::from_netlist(&netlist).unwrap();
        assert_eq!(mna_full.m, 3, "sharp-cutoff pentode should have M=3");

        // vg2k_frozen value is a structural test — 250.0 V is a typical
        // EL84 screen bias, but the math isn't exercised here; only the
        // MNA dimension reduction is checked.
        let mut grid_off = std::collections::HashMap::new();
        grid_off.insert("P1".to_string(), 250.0);
        let mna_reduced = MnaSystem::from_netlist_with_grid_off(&netlist, &grid_off).unwrap();

        assert_eq!(
            mna_reduced.m,
            mna_full.m - 1,
            "grid-off pentode should reduce M by exactly 1"
        );
        assert_eq!(mna_reduced.m, 2, "grid-off pentode should have M=2");
        assert_eq!(mna_reduced.num_devices, 1);
        assert_eq!(mna_reduced.nonlinear_devices[0].dimension, 2);
    }

    #[test]
    fn test_grid_off_pentode_stays_tube_device_type() {
        // The TubeKind::SharpPentodeGridOff discriminator lives on
        // TubeParams.kind at the codegen layer, not on NonlinearDeviceType.
        // The MNA layer must keep reporting `NonlinearDeviceType::Tube`
        // so that the same device-type switches in codegen / DK / NR
        // continue to dispatch correctly.
        let spice = format!(
            "grid-off device type\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
            EL84_MODEL
        );
        let netlist = Netlist::parse(&spice).unwrap();
        let mut grid_off = std::collections::HashMap::new();
        grid_off.insert("P1".to_string(), 250.0);
        let mna = MnaSystem::from_netlist_with_grid_off(&netlist, &grid_off).unwrap();

        let dev = &mna.nonlinear_devices[0];
        assert_eq!(
            dev.device_type,
            NonlinearDeviceType::Tube,
            "grid-off pentode must still report NonlinearDeviceType::Tube"
        );
        assert_eq!(dev.dimension, 2);
        assert_eq!(dev.nodes.len(), 4, "nodes vector must still be plate/grid/cath/screen");
    }

    #[test]
    fn test_from_netlist_without_grid_off_unchanged() {
        // Byte-identity guard: passing an empty grid-off set must produce
        // a structurally identical MNA system to the default constructor.
        let spice = format!(
            "grid-off empty set identity\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
            EL84_MODEL
        );
        let netlist = Netlist::parse(&spice).unwrap();
        let mna_default = MnaSystem::from_netlist(&netlist).unwrap();
        let empty: std::collections::HashMap<String, f64> = std::collections::HashMap::new();
        let mna_empty = MnaSystem::from_netlist_with_grid_off(&netlist, &empty).unwrap();

        assert_eq!(mna_default.n, mna_empty.n);
        assert_eq!(mna_default.m, mna_empty.m);
        assert_eq!(mna_default.g, mna_empty.g);
        assert_eq!(mna_default.c, mna_empty.c);
        assert_eq!(mna_default.n_v, mna_empty.n_v);
        assert_eq!(mna_default.n_i, mna_empty.n_i);
        assert_eq!(mna_default.num_devices, mna_empty.num_devices);
        assert_eq!(
            mna_default.nonlinear_devices[0].dimension,
            mna_empty.nonlinear_devices[0].dimension,
        );
        assert_eq!(
            mna_default.nonlinear_devices[0].device_type,
            mna_empty.nonlinear_devices[0].device_type,
        );
    }

    #[test]
    fn test_grid_off_pentode_parasitic_caps_three_junctions() {
        // Grid-off pentodes should emit 3 junction caps (Cgk, Cgp, Cpk),
        // NOT the 5 of the sharp-cutoff case. CSK and CSP are dropped
        // because the screen is effectively an input (Vg2k is frozen),
        // not an NR unknown.
        let spice = format!(
            "grid-off parasitic caps\n\
             P1 plate grid cath screen EL84\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 screen 0 470k\n\
             R3 cath 0 130\n\
             {}\n",
            EL84_MODEL
        );
        let netlist = Netlist::parse(&spice).unwrap();
        let mut grid_off = std::collections::HashMap::new();
        grid_off.insert("P1".to_string(), 250.0);
        let mut mna = MnaSystem::from_netlist_with_grid_off(&netlist, &grid_off).unwrap();
        mna.add_parasitic_caps();

        let p = *mna.node_map.get("plate").unwrap() - 1;
        let g = *mna.node_map.get("grid").unwrap() - 1;
        let k = *mna.node_map.get("cath").unwrap() - 1;
        let s = *mna.node_map.get("screen").unwrap() - 1;

        // Screen must not have any parasitic cap entries (CSK and CSP dropped).
        assert!(
            mna.c[s][k].abs() < 1e-25,
            "CSK must be absent for grid-off pentode, got {}",
            mna.c[s][k]
        );
        assert!(
            mna.c[k][s].abs() < 1e-25,
            "CSK (transpose) must be absent for grid-off pentode"
        );
        assert!(
            mna.c[s][p].abs() < 1e-25,
            "CSP must be absent for grid-off pentode, got {}",
            mna.c[s][p]
        );
        assert!(
            mna.c[p][s].abs() < 1e-25,
            "CSP (transpose) must be absent for grid-off pentode"
        );

        // Remaining junctions (Cgk, Cgp, Cpk) must still be present.
        let off = |a: usize, b: usize| (mna.c[a][b] + PARASITIC_CAP).abs() < 1e-25;
        assert!(off(g, k), "Cgk must still be present");
        assert!(off(g, p), "Cgp must still be present");
        assert!(off(p, k), "Cpk must still be present");

        // Off-diagonal count across the 4 pentode terminals should be 6
        // (3 junctions x 2 entries each).
        let pentode_nodes = [p, g, k, s];
        let mut off_diag_count = 0;
        for &i in &pentode_nodes {
            for &j in &pentode_nodes {
                if i != j && mna.c[i][j].abs() > 1e-25 {
                    off_diag_count += 1;
                }
            }
        }
        assert_eq!(
            off_diag_count, 6,
            "grid-off pentode should have 6 off-diagonal C entries (3 junctions), got {}",
            off_diag_count
        );
    }

    // ── Triode linearization tests ──────────────────────────────────────

    const TRIODE_12AX7_MODEL: &str = ".model 12AX7 VT(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)";

    #[test]
    fn test_linearized_triode_reduces_m_by_2() {
        let spice = format!(
            "triode linearization M reduction\n\
             V1 plate1 0 250\n\
             R1 grid1 0 1Meg\n\
             R2 cath1 0 1.5k\n\
             V2 plate2 0 250\n\
             R3 grid2 0 1Meg\n\
             R4 cath2 0 1.5k\n\
             T1 grid1 plate1 cath1 12AX7\n\
             T2 grid2 plate2 cath2 12AX7\n\
             {}\n",
            TRIODE_12AX7_MODEL
        );
        let netlist = crate::parser::Netlist::parse(&spice).unwrap();

        // Full MNA: both triodes as nonlinear → M=4
        let mna_full = MnaSystem::from_netlist(&netlist).unwrap();
        assert_eq!(mna_full.m, 4, "2 triodes should give M=4");

        // Linearize T1 → M should reduce by 2
        let mut lin_set = std::collections::HashSet::new();
        lin_set.insert("T1".to_string());
        let mna_lin1 = MnaSystem::from_netlist_with_all_reductions(
            &netlist,
            &std::collections::HashSet::new(),
            &std::collections::HashSet::new(),
            &lin_set,
            &std::collections::HashMap::new(),
        )
        .unwrap();
        assert_eq!(mna_lin1.m, 2, "linearizing T1 should give M=2");
        assert_eq!(mna_lin1.num_devices, 1, "only T2 should remain as nonlinear");

        // Linearize both → M=0
        lin_set.insert("T2".to_string());
        let mna_lin2 = MnaSystem::from_netlist_with_all_reductions(
            &netlist,
            &std::collections::HashSet::new(),
            &std::collections::HashSet::new(),
            &lin_set,
            &std::collections::HashMap::new(),
        )
        .unwrap();
        assert_eq!(mna_lin2.m, 0, "linearizing both triodes should give M=0");
        assert_eq!(mna_lin2.num_devices, 0);
    }

    #[test]
    fn test_stamp_linearized_triodes_gm() {
        // Verify gm VCCS stamping: G[p][g] += gm, G[p][k] -= gm,
        //                           G[k][g] -= gm, G[k][k] += gm
        let spice = format!(
            "triode gm stamp\n\
             R1 grid 0 1Meg\n\
             R2 plate 0 100k\n\
             R3 cath 0 1.5k\n\
             T1 grid plate cath 12AX7\n\
             {}\n",
            TRIODE_12AX7_MODEL
        );
        let netlist = crate::parser::Netlist::parse(&spice).unwrap();

        // Build linearized MNA (triode removed from NR)
        let mut lin_set = std::collections::HashSet::new();
        lin_set.insert("T1".to_string());
        let mut mna = MnaSystem::from_netlist_with_all_reductions(
            &netlist,
            &std::collections::HashSet::new(),
            &std::collections::HashSet::new(),
            &lin_set,
            &std::collections::HashMap::new(),
        )
        .unwrap();
        assert_eq!(mna.m, 0, "linearized triode should have M=0");

        // Save G matrix before stamping
        let g_before: Vec<Vec<f64>> = mna.g.clone();

        let g = *mna.node_map.get("grid").unwrap();
        let p = *mna.node_map.get("plate").unwrap();
        let k = *mna.node_map.get("cath").unwrap();

        // Stamp with known gm and gp values
        let gm = 1.5e-3; // typical 12AX7 gm
        let gp = 6.25e-5; // rp = 16k → 1/rp
        mna.linearized_triodes = vec![LinearizedTriodeInfo {
            name: "T1".to_string(),
            ng: g,
            np: p,
            nk: k,
            gm,
            gp,
            ip_dc: 1.2e-3,
            ig_dc: 0.0,
        }];
        mna.stamp_linearized_triodes();

        // Check combined gm + gp stamps (0-indexed)
        let gi = g - 1;
        let pi = p - 1;
        let ki = k - 1;
        let eps = 1e-12;

        // Expected deltas from gm VCCS + gp plate conductance:
        //   G[p][g] += gm
        //   G[p][k] -= gm - gp   (both gm VCCS and gp shunt contribute)
        //   G[p][p] += gp
        //   G[k][g] -= gm
        //   G[k][k] += gm + gp   (both contribute)
        //   G[k][p] -= gp
        assert!((mna.g[pi][gi] - g_before[pi][gi] - gm).abs() < eps, "G[p][g] += gm");
        assert!(
            (mna.g[pi][ki] - g_before[pi][ki] + gm + gp).abs() < eps,
            "G[p][k] -= (gm + gp)"
        );
        assert!((mna.g[pi][pi] - g_before[pi][pi] - gp).abs() < eps, "G[p][p] += gp");
        assert!((mna.g[ki][gi] - g_before[ki][gi] + gm).abs() < eps, "G[k][g] -= gm");
        assert!(
            (mna.g[ki][ki] - g_before[ki][ki] - gm - gp).abs() < eps,
            "G[k][k] += (gm + gp)"
        );
        assert!((mna.g[ki][pi] - g_before[ki][pi] + gp).abs() < eps, "G[k][p] -= gp");
    }

    #[test]
    fn test_stamp_linearized_triode_dc_bias_currents() {
        let spice = format!(
            "triode dc bias\n\
             R1 grid 0 1Meg\n\
             R2 plate 0 100k\n\
             R3 cath 0 1.5k\n\
             T1 grid plate cath 12AX7\n\
             {}\n",
            TRIODE_12AX7_MODEL
        );
        let netlist = crate::parser::Netlist::parse(&spice).unwrap();

        let mut lin_set = std::collections::HashSet::new();
        lin_set.insert("T1".to_string());
        let mut mna = MnaSystem::from_netlist_with_all_reductions(
            &netlist,
            &std::collections::HashSet::new(),
            &std::collections::HashSet::new(),
            &lin_set,
            &std::collections::HashMap::new(),
        )
        .unwrap();

        let cs_count_before = mna.current_sources.len();
        let ip_dc = 1.2e-3;
        let ig_dc = 0.0; // no grid current (valid linearization)

        let g = *mna.node_map.get("grid").unwrap();
        let p = *mna.node_map.get("plate").unwrap();
        let k = *mna.node_map.get("cath").unwrap();

        mna.linearized_triodes = vec![LinearizedTriodeInfo {
            name: "T1".to_string(),
            ng: g,
            np: p,
            nk: k,
            gm: 1.5e-3,
            gp: 6.25e-5,
            ip_dc,
            ig_dc,
        }];
        mna.stamp_linearized_triodes();

        // Should add Ip_dc and Ik_dc current sources (Ig_dc skipped since ~0)
        assert_eq!(
            mna.current_sources.len(),
            cs_count_before + 2,
            "should add Ip_dc + Ik_dc (Ig_dc skipped when ~0)"
        );

        // Ip_dc: ground→plate
        let ip_src = mna.current_sources.iter().find(|cs| cs.name.contains("Ip_dc"));
        assert!(ip_src.is_some(), "should have Ip_dc current source");
        let ip_src = ip_src.unwrap();
        assert_eq!(ip_src.n_plus_idx, 0);
        assert_eq!(ip_src.n_minus_idx, p);
        assert!((ip_src.dc_value - ip_dc).abs() < 1e-15);

        // Ik_dc: cathode→ground (Ip + Ig)
        let ik_src = mna.current_sources.iter().find(|cs| cs.name.contains("Ik_dc"));
        assert!(ik_src.is_some(), "should have Ik_dc current source");
        let ik_src = ik_src.unwrap();
        assert_eq!(ik_src.n_plus_idx, k);
        assert_eq!(ik_src.n_minus_idx, 0);
        assert!((ik_src.dc_value - ip_dc).abs() < 1e-15);
    }

    #[test]
    fn test_linearized_triode_empty_set_identity() {
        // Empty linearized_triodes set should produce identical MNA to default
        let spice = format!(
            "triode empty set identity\n\
             V1 plate 0 250\n\
             R1 grid 0 1Meg\n\
             R2 cath 0 1.5k\n\
             T1 grid plate cath 12AX7\n\
             {}\n",
            TRIODE_12AX7_MODEL
        );
        let netlist = crate::parser::Netlist::parse(&spice).unwrap();

        let mna_default = MnaSystem::from_netlist(&netlist).unwrap();
        let mna_empty = MnaSystem::from_netlist_with_all_reductions(
            &netlist,
            &std::collections::HashSet::new(),
            &std::collections::HashSet::new(),
            &std::collections::HashSet::new(),
            &std::collections::HashMap::new(),
        )
        .unwrap();

        assert_eq!(mna_default.n, mna_empty.n);
        assert_eq!(mna_default.m, mna_empty.m);
        assert_eq!(mna_default.g, mna_empty.g);
        assert_eq!(mna_default.c, mna_empty.c);
        assert_eq!(mna_default.n_v, mna_empty.n_v);
        assert_eq!(mna_default.n_i, mna_empty.n_i);
        assert_eq!(mna_default.num_devices, mna_empty.num_devices);
    }
}
