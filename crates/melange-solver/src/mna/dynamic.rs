//! Runtime-parameter info: switches, runtime sources, pots, wiper and gang groups.

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
    /// Optional human-readable label from the `.switch` directive (e.g. "Oboe 8").
    /// Emitted as `SWITCH_LABELS` so consumers can assert their own enum against
    /// directive order instead of relying on a silent positional mapping.
    pub label: Option<String>,
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
/// R_cw = (1-pos) * (total - 20) + 10, R_ccw = pos * (total - 20) + 10
/// (MIN_LEG_R = 10 Ω end-stop on each leg; pos = 1 → wiper at the CW end →
/// R_cw at its 10 Ω minimum). Must match `parser::expand_wipers` and the
/// CLI/plugin wiper mapping (`WIPER_MIN_LEG_R`).
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
