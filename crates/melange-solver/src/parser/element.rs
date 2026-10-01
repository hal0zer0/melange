//! Circuit elements and inductor saturation descriptors.

/// How an inductor's saturated incremental inductance was authored.
///
/// Past saturation the iron's magnetisation is spent and dB/dH falls to µ0,
/// so the winding keeps its air-core inductance: the flux law is
/// `Φ(i) = L_mag·Isat·tanh(i/Isat) + L_air·i` with `L_mag + L_air = L0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SatFloor {
    /// `LAIR=<fraction>`: L_air as a fraction of L0 (0 allowed, for bisecting
    /// and for comparison with tanh-only work).
    Explicit(f64),
    /// `CORE=<class>`: a rule-of-thumb class default.
    Class(CoreClass),
}

/// Core classes with rule-of-thumb air-core fractions (L_air/L0 ~ g/mu_eff).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoreClass {
    /// Gapped chokes and single-ended output transformers: 1e-3.
    Gapped,
    /// Ungapped silicon steel: 3e-4.
    Steel,
    /// High-nickel (Mumetal-type) cores: 3e-5.
    Nickel,
}

/// The air-core fraction used when a saturating inductor declares neither
/// `LAIR=` nor `CORE=` (ungapped steel, rule-of-thumb).
pub const DEFAULT_AIR_FLOOR: f64 = 3e-4;

/// Resolve an authored floor to `(L_air/L0, provenance)`. `None` is the
/// default, which every caller must announce.
pub fn resolve_air_floor(floor: Option<SatFloor>) -> (f64, &'static str) {
    match floor {
        Some(SatFloor::Explicit(v)) => (v, "LAIR="),
        Some(SatFloor::Class(CoreClass::Gapped)) => (1e-3, "CORE=gapped (rule-of-thumb)"),
        Some(SatFloor::Class(CoreClass::Steel)) => (3e-4, "CORE=steel (rule-of-thumb)"),
        Some(SatFloor::Class(CoreClass::Nickel)) => (3e-5, "CORE=nickel (rule-of-thumb)"),
        None => (DEFAULT_AIR_FLOOR, "default (rule-of-thumb, ungapped steel)"),
    }
}

/// How an authored `ISAT=` relates to the model's saturation current.
///
/// The model's `ISAT` is the tanh scale current: the core's saturation flux is
/// `L_mag·ISAT`, and at `i = ISAT` the magnetizing slope is `sech²(1) = 0.42`
/// of its small-signal value. A datasheet "saturation current" is a different
/// quantity, the current at which the inductance has dropped by some fraction;
/// these forms convert it (analog-EE review).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IsatSpec {
    /// `ISAT=<i> ISAT_DROP=<d> [ISAT_BASIS=incremental|apparent]`: at `i` the
    /// inductance has fallen by the fraction `d`.
    Drop { drop: f64, basis: IsatBasis },
    /// `L_AT_IDC=<L>,<I>`: the inductance is `L` at DC bias `I` (the "L at
    /// rated DC" rating of chokes and single-ended output transformers). An
    /// incremental drop of `1 - L/L0` at `I`.
    LAtIdc { l: f64 },
}

/// Which inductance a datasheet drop refers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IsatBasis {
    /// `dΦ/di` at the bias point: an LCR meter's small signal over a DC bias,
    /// the usual measurement. The default.
    Incremental,
    /// `Φ/i`, e.g. from a volt-second measurement.
    Apparent,
}

/// A circuit element (component).
#[derive(Debug, Clone, PartialEq)]
pub enum Element {
    /// Resistor: Rname n+ n- value [KF=val] [AF=val]
    ///
    /// `kf`/`af` are opt-in 1/f flicker noise parameters (Hooge bias-squared
    /// form, Phase 3.5). Per-sample injected current is
    /// `sqrt(KF·fs) · |I_R(t)|^(AF/2) · kellett_pink(N(0,1))` where
    /// `I_R(t) = (V_+ − V_−) / R` is the live current through this resistor.
    /// Both default to `None`; codegen-time `kf == None || kf == Some(0.0)`
    /// emits no flicker source for this resistor — byte-identical to a
    /// pre-Phase-3.5 build. Default `AF = 2.0` when `KF` is set without `AF`
    /// (Hooge's exponent for resistors). See `docs/aidocs/NOISE.md`.
    Resistor {
        name: String,
        n_plus: String,
        n_minus: String,
        value: f64,
        kf: Option<f64>,
        af: Option<f64>,
    },
    /// Capacitor: Cname n+ n- value [IC=initial]
    Capacitor {
        name: String,
        n_plus: String,
        n_minus: String,
        value: f64,
        ic: Option<f64>,
    },
    /// Inductor: Lname n+ n- value [ISAT=value [ISAT_DROP=d [ISAT_BASIS=b]] |
    /// L_AT_IDC=L,I] [LAIR=fraction | CORE=class] [TURNS=t] [LM=henries]
    Inductor {
        name: String,
        n_plus: String,
        n_minus: String,
        value: f64,
        /// Saturation current for iron-core model. None = linear (default).
        /// As authored: with `isat_spec` it is a datasheet current, converted
        /// to the model's tanh scale current when the MNA is built.
        isat: Option<f64>,
        /// How `isat` was specified; `None` = it is the model's ISAT.
        isat_spec: Option<IsatSpec>,
        /// Air-core floor of a saturating inductor's incremental inductance, as
        /// authored (`LAIR=` or `CORE=`); `None` takes the default. Resolve
        /// with [`resolve_air_floor`].
        air_floor: Option<SatFloor>,
        /// Relative turns of this winding of a saturating shared core
        /// (`TURNS=`); given on every winding of the group or none.
        turns: Option<f64>,
        /// The core's magnetizing inductance seen from this winding (`LM=`),
        /// on exactly one winding of a saturating shared core. Declaring it
        /// declares one core loop linking every winding.
        lm: Option<f64>,
    },
    /// Voltage source: Vname n+ n- [DC value] [AC mag phase]
    VoltageSource {
        name: String,
        n_plus: String,
        n_minus: String,
        dc: Option<f64>,
        ac: Option<(f64, f64)>,
    },
    /// Current source: Iname n+ n- [DC value]
    CurrentSource {
        name: String,
        n_plus: String,
        n_minus: String,
        dc: Option<f64>,
    },
    /// Diode: Dname n+ n- modelname
    Diode {
        name: String,
        n_plus: String,
        n_minus: String,
        model: String,
    },
    /// BJT: Qname nc nb ne \[ns\] modelname
    Bjt {
        name: String,
        nc: String,
        nb: String,
        ne: String,
        model: String,
    },
    /// JFET: Jname nd ng ns modelname
    Jfet {
        name: String,
        nd: String,
        ng: String,
        ns: String,
        model: String,
    },
    /// MOSFET: Mname nd ng ns nb modelname
    Mosfet {
        name: String,
        nd: String,
        ng: String,
        ns: String,
        nb: String,
        model: String,
    },
    /// Op-amp: Uname n_plus n_minus n_out modelname
    ///
    /// Modeled as a high-gain VCCS -- does NOT add nonlinear dimensions.
    Opamp {
        name: String,
        /// Non-inverting input node
        n_plus: String,
        /// Inverting input node
        n_minus: String,
        /// Output node
        n_out: String,
        /// Model name (references .model with OA type)
        model: String,
    },
    /// Triode: Tname n_grid n_plate n_cathode modelname
    ///
    /// M=2 per triode: plate current (Ip) and grid current (Ig).
    Triode {
        name: String,
        /// Grid node
        n_grid: String,
        /// Plate (anode) node
        n_plate: String,
        /// Cathode node
        n_cathode: String,
        /// Model name (references .model with TUBE type)
        model: String,
    },
    /// Pentode (or beam tetrode): Pname n_plate n_grid n_cathode n_screen \[n_suppressor\] modelname
    ///
    /// Node order is **plate-grid-cathode-screen** (LTspice/PSpice/Ayumi convention).
    /// Note this differs from the existing `Triode` (`T`) element which uses
    /// grid-plate-cathode ordering. New netlists should use the plate-first
    /// convention for pentodes; triode netlists are unchanged.
    ///
    /// The suppressor grid is optional: when absent, melange models beam
    /// tetrodes and strapped pentodes as "suppressor tied to cathode"
    /// (the universal case in audio power tubes). EF86-class true pentodes
    /// with an independent suppressor brought out of the envelope may pass
    /// a 5th node; the current kernel still models the suppressor as
    /// electrically tied to the cathode (phase 1a limitation).
    ///
    /// M=3 per pentode in the NR system: plate current (Ip), screen current
    /// (Ig2), and control-grid current (Ig1). Uses Reefman Derk §4.4 math
    /// (see `TubeKind::SharpPentode`).
    Pentode {
        name: String,
        /// Plate (anode) node
        n_plate: String,
        /// Control grid (g1) node
        n_grid: String,
        /// Cathode node
        n_cathode: String,
        /// Screen grid (g2) node
        n_screen: String,
        /// Optional suppressor grid (g3) node. `None` means strapped to cathode.
        n_suppressor: Option<String>,
        /// Model name (references `.model` with VP/VPENTODE type)
        model: String,
    },
    /// Opto/LDR photoresistor: Oname r+ r- ctrl+ ctrl- modelname
    ///
    /// `r+ r-` are the photoresistor terminals (the NR-controlled resistance
    /// path; 1 NR dimension, `I = (V(r+)−V(r-)) / r_state`). `ctrl+ ctrl-` are
    /// the foreign brightness-control node pair that drives the after-solve
    /// state update — `V(ctrl+) − V(ctrl-)` is the normalized brightness in
    /// \[0,1\] (0 = dark → RMAX, 1 = bright → RMIN), clamped inside `update()`.
    /// The LED is NOT modeled inside the device (a physically-driven
    /// optocoupler is future composition: a `D` + behavioral B-source feeding
    /// the brightness node). References a `.model` of type `LDR`.
    Ldr {
        name: String,
        /// Photoresistor terminal + (resistance path)
        n_plus: String,
        /// Photoresistor terminal − (resistance path)
        n_minus: String,
        /// Brightness-control node + (foreign; drives update())
        n_ctrl_p: String,
        /// Brightness-control node − (foreign; drives update())
        n_ctrl_n: String,
        /// Model name (references .model with LDR type)
        model: String,
    },
    /// Glow-discharge / neon lamp: Nname a k modelname (EXPERIMENTAL, Phase 0c
    /// Stage 2a). A 2-terminal gas-discharge relaxation element; references a
    /// `.model … NEON(VO VM IK RS IHOLD ROFF)` card. Throwaway experimental
    /// syntax — the `N` letter and `NEON` model type are provisional.
    Glow {
        name: String,
        /// Anode node
        n_anode: String,
        /// Cathode node
        n_cathode: String,
        /// Model name (references .model with NEON type)
        model: String,
    },
    /// VCA: Yname sig+ sig- ctrl+ ctrl- modelname
    ///
    /// M=2 per VCA: signal current (I_sig) and control current (I_ctrl=0).
    Vca {
        name: String,
        /// Signal positive node
        n_sig_p: String,
        /// Signal negative node
        n_sig_n: String,
        /// Control positive node
        n_ctrl_p: String,
        /// Control negative node
        n_ctrl_n: String,
        /// Model name (references .model with VCA type)
        model: String,
    },
    /// Voltage-Controlled Voltage Source: Ename out+ out- ctrl+ ctrl- gain
    ///
    /// Modeled as Norton equivalent (VCCS + output conductance).
    /// Does NOT add nonlinear dimensions.
    Vcvs {
        name: String,
        /// Positive output node
        out_p: String,
        /// Negative output node
        out_n: String,
        /// Positive control node
        ctrl_p: String,
        /// Negative control node
        ctrl_n: String,
        /// Voltage gain
        gain: f64,
    },
    /// Voltage-Controlled Current Source: Gname out+ out- ctrl+ ctrl- gm
    ///
    /// Direct G matrix stamp. Does NOT add nonlinear dimensions.
    Vccs {
        name: String,
        /// Positive output node (current flows into this node)
        out_p: String,
        /// Negative output node (current flows out of this node)
        out_n: String,
        /// Positive control node
        ctrl_p: String,
        /// Negative control node
        ctrl_n: String,
        /// Transconductance (A/V)
        gm: f64,
    },
    /// Subcircuit instance: Xname nodes... subcktname
    SubcktInstance {
        name: String,
        nodes: Vec<String>,
        subckt: String,
    },
    /// Behavioral (arbitrary-expression) source — SPICE3 `B` element.
    ///
    /// `B<name> n+ n- V={expr}` is a nonlinear voltage source (augmented MNA
    /// constraint `V(n+) - V(n-) = expr`); `I={expr}` is a nonlinear current
    /// source injecting `expr` amps from `n+` to `n-`. The expression may
    /// reference arbitrary node voltages / branch currents / `time`, so these
    /// route the circuit to the nodal solver (the DK `N_v`/`N_i` reduction
    /// assumes a single controlling node-pair voltage per dimension).
    BSource {
        name: String,
        n_plus: String,
        n_minus: String,
        kind: BSourceKind,
        expr: crate::expr::Expr,
    },
}

/// Output quantity controlled by a behavioral [`Element::BSource`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BSourceKind {
    /// `V={expr}` — behavioral voltage source.
    Voltage,
    /// `I={expr}` — behavioral current source.
    Current,
}

impl Element {
    /// Get the element name.
    pub fn name(&self) -> &str {
        match self {
            Element::Resistor { name, .. }
            | Element::Capacitor { name, .. }
            | Element::Inductor { name, .. }
            | Element::VoltageSource { name, .. }
            | Element::CurrentSource { name, .. }
            | Element::Diode { name, .. }
            | Element::Bjt { name, .. }
            | Element::Jfet { name, .. }
            | Element::Mosfet { name, .. }
            | Element::Opamp { name, .. }
            | Element::Triode { name, .. }
            | Element::Pentode { name, .. }
            | Element::Vca { name, .. }
            | Element::Ldr { name, .. }
            | Element::Glow { name, .. }
            | Element::Vcvs { name, .. }
            | Element::Vccs { name, .. }
            | Element::SubcktInstance { name, .. }
            | Element::BSource { name, .. } => name,
        }
    }

    /// Get the referenced model name, if this is a device that requires a `.model` card.
    pub fn model_name(&self) -> Option<&str> {
        match self {
            Element::Diode { model, .. }
            | Element::Bjt { model, .. }
            | Element::Jfet { model, .. }
            | Element::Mosfet { model, .. }
            | Element::Opamp { model, .. }
            | Element::Triode { model, .. }
            | Element::Pentode { model, .. }
            | Element::Vca { model, .. }
            | Element::Ldr { model, .. }
            | Element::Glow { model, .. } => Some(model),
            _ => None,
        }
    }

    /// Every circuit node this element physically connects to.
    ///
    /// Returns one entry per terminal, in the element's canonical node order,
    /// for **all** element variants. Ground appears as `"0"` (node names are
    /// normalized at parse time, so `gnd`/`ground` are already folded to `"0"`).
    ///
    /// Only physical *terminals* are returned. For [`Element::BSource`] that is
    /// `n_plus`/`n_minus`; nodes referenced inside its expression are controlling
    /// inputs, not terminals, and are intentionally excluded. For
    /// [`Element::Vcvs`]/[`Element::Vccs`] the control-input node pair *is* a
    /// physical connection (the controlling voltage is sensed there), so it is
    /// included alongside the output pair.
    ///
    /// The node list must stay in lockstep with `super::normalize_element_nodes` —
    /// any variant that carries a node there must yield it here.
    pub fn nodes(&self) -> Vec<&str> {
        match self {
            Element::Resistor {
                n_plus, n_minus, ..
            }
            | Element::Capacitor {
                n_plus, n_minus, ..
            }
            | Element::Inductor {
                n_plus, n_minus, ..
            }
            | Element::VoltageSource {
                n_plus, n_minus, ..
            }
            | Element::CurrentSource {
                n_plus, n_minus, ..
            }
            | Element::Diode {
                n_plus, n_minus, ..
            }
            | Element::BSource {
                n_plus, n_minus, ..
            } => vec![n_plus, n_minus],
            Element::Bjt { nc, nb, ne, .. } => vec![nc, nb, ne],
            Element::Jfet { nd, ng, ns, .. } => vec![nd, ng, ns],
            Element::Mosfet { nd, ng, ns, nb, .. } => vec![nd, ng, ns, nb],
            Element::Opamp {
                n_plus,
                n_minus,
                n_out,
                ..
            } => vec![n_plus, n_minus, n_out],
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
                let mut v = vec![
                    n_plate.as_str(),
                    n_grid.as_str(),
                    n_cathode.as_str(),
                    n_screen.as_str(),
                ];
                if let Some(ns) = n_suppressor {
                    v.push(ns);
                }
                v
            }
            Element::Vca {
                n_sig_p,
                n_sig_n,
                n_ctrl_p,
                n_ctrl_n,
                ..
            } => vec![n_sig_p, n_sig_n, n_ctrl_p, n_ctrl_n],
            // Resistance path (r+, r-) FIRST — the N_v/N_i reduction reads the
            // first two node_indices for the 1-D resistance dimension; the
            // control pair follows and is a physical connection (its voltage is
            // sensed to drive the state update).
            Element::Ldr {
                n_plus,
                n_minus,
                n_ctrl_p,
                n_ctrl_n,
                ..
            } => vec![n_plus, n_minus, n_ctrl_p, n_ctrl_n],
            Element::Vcvs {
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                ..
            }
            | Element::Vccs {
                out_p,
                out_n,
                ctrl_p,
                ctrl_n,
                ..
            } => vec![out_p, out_n, ctrl_p, ctrl_n],
            Element::Glow {
                n_anode, n_cathode, ..
            } => vec![n_anode, n_cathode],
            Element::SubcktInstance { nodes, .. } => nodes.iter().map(String::as_str).collect(),
        }
    }
}
