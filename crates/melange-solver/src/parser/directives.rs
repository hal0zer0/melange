//! Records for melange's dot-command directives.

/// A `.mismatch` directive — per-device-type parameter jitter spec.
///
/// Syntax: `.mismatch <type> <param>=<tol> [<param>=<tol> ...]` where
/// `type` is a single-character device class (`D` = diode, `Q` = BJT,
/// `J` = JFET, `M` = MOSFET, `T` = triode/pentode) and each `tol` is a
/// dimensionless fraction (0.02 → ±2% worst case, drawn uniform).
///
/// Multiple `.mismatch` directives for the same type are merged: the
/// last tolerance wins per param name.
#[derive(Debug, Clone, PartialEq)]
pub struct MismatchSpec {
    /// Single-character device class, uppercased at parse time.
    pub device_class: char,
    /// `(param_name_uppercase, tolerance)` pairs, e.g. `("IS", 0.02)`.
    pub params: Vec<(String, f64)>,
}

/// A runtime voltage source directive (.runtime Vname as field_name).
///
/// Binds a voltage source already declared in the netlist to a mutable
/// `pub <field>: f64` on the generated `CircuitState`. The plugin host
/// writes the field; codegen stamps the value into the VS's KVL constraint
/// row each sample.
#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeDirective {
    /// Name of the voltage source (case-insensitive match to Element::VoltageSource).
    pub vs_name: String,
    /// Rust identifier used for the generated CircuitState field.
    /// Validated at parse time: must be non-empty, ASCII, start with a
    /// letter or underscore, and contain only `[A-Za-z0-9_]`.
    pub field_name: String,
}

/// A runtime resistor directive (.runtime Rname min max as field_name).
///
/// Audio-rate resistor modulation target. Codegen emits
/// `set_runtime_R_<field>(r)` on `CircuitState` that clamps to [min, max]
/// and marks matrices dirty. Plugin template does NOT emit a nih-plug
/// knob for these; the plugin drives the setter from its own envelope
/// follower. Semantically identical to `.pot R`'s setter since the
/// 2026-04-20 reseed strip — the remaining differences are API shape
/// (field-named setter + read-only accessor, no plugin knob).
#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeResistorDirective {
    /// Name of the resistor this directive claims (case-insensitive match).
    pub resistor_name: String,
    /// Minimum resistance (ohms), used for clamp at runtime.
    pub min_value: f64,
    /// Maximum resistance (ohms), used for clamp at runtime.
    pub max_value: f64,
    /// Rust identifier used for the generated setter name
    /// (`set_runtime_R_<field_name>`) and getter (`<field_name>()`).
    pub field_name: String,
}

/// A bare plugin-driven scalar (`​.runtime <name> <min> <max> as <field>`).
///
/// Unlike `.runtime R/V`, this is not attached to any element — it's a free
/// scalar the plugin sets via `set_runtime_<field>`, referenced by name inside
/// behavioral `B`-source expressions (e.g. `strength`, `f_offset`). Dispatched
/// when the `.runtime` target starts with neither `V` nor `R` (SPICE component
/// names must, so a non-R/V target is unambiguously a scalar).
#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeScalarDirective {
    /// Parameter name as referenced in expressions.
    pub name: String,
    /// Clamp minimum.
    pub min_value: f64,
    /// Clamp maximum.
    pub max_value: f64,
    /// Rust identifier for the state field / `set_runtime_<field>` setter.
    pub field_name: String,
}

/// Source impedance for a `.inject` runtime feedback source.
///
/// Impedance is MANDATORY on `.inject` (rejected at parse if absent): an ideal
/// source with no series/shunt conductance would clamp the injection node and
/// destroy the dry signal path. See `local-docs/inject-directive-plan.md`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InjectImpedance {
    /// `R=<ohms>` — Thevenin source: the runtime value is a VOLTAGE behind
    /// series resistance `R`. Stamp `G = 1/R` into `g[node][node]` before the
    /// kernel; `rhs[node] += (val + val_prev) * G` (trap) / `val * G` (BE).
    Thevenin(f64),
    /// `RSHUNT=<ohms>` — Norton source: the runtime value is a CURRENT injected
    /// at the node with shunt resistance `RSHUNT`. Stamp `G = 1/RSHUNT` into the
    /// diagonal; `rhs[node] += val` (integration-scheme-independent, like a
    /// current source).
    Norton(f64),
}

/// The rate at which a `.inject` source is supplied (`rate=host|inner`).
///
/// Only matters under oversampling; at `OVERSAMPLING_FACTOR == 1` the two
/// are the same thing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InjectRate {
    /// `rate=host` (the default): an audio-rate circuit input, supplied once
    /// per HOST sample and upsampled through its own copy of the half-band
    /// up-filter the audio input uses (same coefficients, same group delay).
    #[default]
    Host,
    /// `rate=inner`: supplied per inner (oversampled) sub-step and routed
    /// straight to the inner solve. The caller owns band-limiting. The rate
    /// for a feedback loop closed from a `.tap` at the inner rate.
    Inner,
}

/// A `.inject <node> <field> R=<ohms>|RSHUNT=<ohms> [rate=host|inner]`
/// directive.
///
/// Declares a runtime source the host drives per sample — per host sample
/// (`rate=host`, the default) or per inner sub-step (`rate=inner`). The
/// source value is known at sample start, so it enters the per-sample RHS as
/// an ordinary constant — the NR loop never sees it. Reuses the multi-input
/// RHS-stamp machinery. Single-ended (node-to-ground) only.
#[derive(Debug, Clone, PartialEq)]
pub struct InjectDirective {
    /// Injection node (normalized name). Must be a non-ground circuit node.
    pub node: String,
    /// Rust identifier naming this injection (emitted in `INJECT_NAMES`, gives
    /// the injection a stable order/identity). Not a `CircuitState` field — the
    /// value is supplied as a `process_sample` argument, not a setter.
    pub field_name: String,
    /// Mandatory source impedance (Thevenin `R=` or Norton `RSHUNT=`).
    pub impedance: InjectImpedance,
    /// Declared supply rate (`rate=host|inner`; `host` when omitted).
    pub rate: InjectRate,
}

/// A `.tap <node> [name]` directive.
///
/// Declares a RAW, pre-decimation inner-rate probe of `node`. `process_sample`
/// returns per-inner-sample tap values (`taps_inner: [[f64; NUM_TAP]; OS]`) so
/// a feedback caller's inner-rate model can run on the un-band-limited node
/// signal. Emitted SEPARATELY from output nodes even when a node coincides
/// (different semantics: raw inner-rate vs DC-blocked/scaled/decimated output).
#[derive(Debug, Clone, PartialEq)]
pub struct TapDirective {
    /// Tapped node (normalized name). Must be a non-ground circuit node.
    pub node: String,
    /// Human-readable label emitted in `TAP_NAMES`. Defaults to the node name.
    pub name: String,
}

/// A `.port <node> ...` declaration — one of the board's pins.
///
/// **Direction-neutral by construction.** A board pin is a place the outside
/// world connects to: an output tap read by whatever the board feeds, an input
/// this particular build leaves undriven, or a pin that is both depending on
/// how the board is wired into the instrument. Which one it is on any given
/// compile is what `-i` / `-n` say; which pins EXIST is a property of the
/// circuit, and that is what this records.
///
/// Its whole effect is on [`crate::topology`]: a declared pin counts as one
/// connection for the dangling-node check, so a multi-output board compiled one
/// output at a time is not read as five typos. It stamps nothing, is not a DC
/// path (an undriven pin behind a coupling cap is still a floating island, and
/// still warns), and never reaches codegen.
#[derive(Debug, Clone, PartialEq)]
pub struct PortDirective {
    /// The pin's node (normalized name). Must be a node some element names.
    pub node: String,
    /// 1-based raw source line of the `.port` statement that declared it, so a
    /// declaration naming a node that does not exist can point at itself.
    pub line: usize,
}

/// A potentiometer directive (.pot Rname min max).
///
/// Marks a resistor as runtime-variable with a min/max range.
/// The resistor's existing value in the netlist becomes the nominal
/// value for Sherman-Morrison precomputation.
#[derive(Debug, Clone, PartialEq)]
pub struct PotDirective {
    /// Name of the resistor this pot controls (e.g. "R1")
    pub resistor_name: String,
    /// Minimum resistance value (ohms)
    pub min_value: f64,
    /// Maximum resistance value (ohms)
    pub max_value: f64,
    /// Default resistance for plugin parameter (ohms). If None, uses netlist nominal.
    pub default_value: Option<f64>,
    /// Optional human-readable label (e.g. "Volume")
    pub label: Option<String>,
}

/// A wiper potentiometer directive (.wiper R_cw R_ccw total_R).
///
/// Models a 3-terminal pot with top, wiper, and bottom lugs.
/// Internally expands into two linked `PotDirective` entries that share
/// a wiper node. A single UI parameter (position 0.0–1.0) controls both.
#[derive(Debug, Clone, PartialEq)]
pub struct WiperDirective {
    /// Name of the clockwise (top→wiper) leg resistor
    pub resistor_cw: String,
    /// Name of the counter-clockwise (wiper→bottom) leg resistor
    pub resistor_ccw: String,
    /// Total resistance (R_cw + R_ccw = total)
    pub total_resistance: f64,
    /// Default wiper position (0.0–1.0). When omitted, parsing sets it from
    /// the legs' netlist values (as a `.pot` defaults to its netlist value).
    pub default_position: Option<f64>,
    /// Optional human-readable label (e.g. "Tone")
    pub label: Option<String>,
}

/// A switch directive (.switch C1,L1 val0a/val0b val1a/val1b ...).
///
/// Defines a rotary switch that selects among discrete component values.
/// Multiple components can be ganged (changed simultaneously) by listing
/// names separated by commas, with position values separated by slashes.
#[derive(Debug, Clone, PartialEq)]
pub struct SwitchDirective {
    /// Component names controlled by this switch (e.g. ["C_hfb", "L_hfb"])
    pub component_names: Vec<String>,
    /// Position values: `positions[pos][comp]` = value for that component at that position
    pub positions: Vec<Vec<f64>>,
    /// Optional human-readable label (e.g. "Bright")
    pub label: Option<String>,
}

/// A gang directive (.gang "Label" member1 member2 ...).
///
/// Links multiple `.pot` and/or `.wiper` entries to a single UI parameter.
/// All members are controlled by one position value (0.0–1.0).
/// Pot members map position to resistance: R = max - pos * (max - min).
/// Wiper members map position directly to wiper position.
/// Prefix a member name with `!` to invert its response.
#[derive(Debug, Clone, PartialEq)]
pub struct GangDirective {
    /// Human-readable label for the gang parameter (e.g. "Gain")
    pub label: String,
    /// Members of this gang (pot or wiper component references)
    pub members: Vec<GangMember>,
    /// Default position (0.0–1.0). None means 0.5.
    pub default_position: Option<f64>,
}

/// A member of a gang directive.
#[derive(Debug, Clone, PartialEq)]
pub struct GangMember {
    /// Resistor name (must exist in a .pot or .wiper directive)
    pub resistor_name: String,
    /// If true, this member's response is inverted (1.0 - pos)
    pub inverted: bool,
}

/// A coupling directive (K element) for coupled inductors / transformers.
///
/// Standard SPICE syntax: `K1 L1 L2 0.95`
/// Couples two inductors with mutual inductance M = k * sqrt(L1 * L2).
#[derive(Debug, Clone, PartialEq)]
pub struct CouplingDirective {
    /// Name of the coupling element (e.g. "K1")
    pub name: String,
    /// Name of the first inductor (e.g. "L1")
    pub inductor1_name: String,
    /// Name of the second inductor (e.g. "L2")
    pub inductor2_name: String,
    /// Coupling coefficient k (0 < k < 1)
    pub coupling: f64,
}
