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

mod augmented;
mod builder;
mod companion;
mod devices;
mod dynamic;
mod helpers;
mod internal_nodes;
mod magnetics;
mod opamp;
mod parasitic;
mod reduce;
mod sources;
mod stamp;

pub use augmented::*;
pub use devices::*;
pub use dynamic::*;
pub(crate) use helpers::*;
pub use magnetics::*;
pub use opamp::*;
pub use parasitic::*;
pub use sources::*;

/// MNA system matrices.
///
/// Represents the linear part of the circuit:
///   (G + s*C) * V = I
///
/// After discretization with timestep T:
///   A = 2*C/T + G  (trapezoidal rule)
///
/// When elements that need branch-current unknowns or algebraic constraint
/// rows are present, the system is augmented with extra rows/columns beyond
/// the `n` circuit nodes. All matrices (G, C, N_v, N_i) are expanded to
/// n_aug × n_aug (N_v to m × n_aug, N_i to n_aug × m).
#[derive(Debug, Clone)]
pub struct MnaSystem {
    /// Number of circuit nodes (excluding ground)
    pub n: usize,
    /// Augmented system dimension. Rows `n..n_aug` are, in index order:
    ///   1. Voltage-source branch currents (`n + vs.ext_idx`), one per VS
    ///   2. VCVS branch currents (`n + num_vs + vcvs_idx`)
    ///   3. Ideal-transformer coupling currents (`n + num_vs + num_vcvs + xfmr_idx`)
    ///   4. Current-mode VCA rows, two per current-mode VCA: the internal
    ///      `sig+_int` node (with a 1 Ω dummy terminator) followed by the
    ///      0 V sensing-source branch current (`vca.n_internal_idx` /
    ///      `vca.n_sense_idx`, both stored 1-indexed)
    ///   5. Behavioral `V={}` source branch currents (`BehavioralSourceInfo::aug_row`)
    /// All of the above are algebraic constraint/branch rows with no
    /// capacitance — they are blanket-zeroed in A_neg (no trapezoidal
    /// history). Additionally, `expand_bjt_internal_nodes` can append
    /// parasitic-BJT internal nodes AFTER these rows (growing n_aug);
    /// those are physical nodes with G and C stamps and are excluded from
    /// the A_neg zeroing (see `build_discretized_matrix`).
    /// Equal to n when none of the above elements are present.
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
    /// Behavioral (arbitrary-expression) `B`-sources. Stamped directly in node
    /// space by codegen; their presence forces nodal routing.
    pub behavioral_sources: Vec<BehavioralSourceInfo>,
    /// VCVS augmented row info (one entry per VCVS element, in element order)
    pub vcvs_sources: Vec<VcvsAugInfo>,
    /// Current source contributions to RHS
    pub current_sources: Vec<CurrentSourceInfo>,
    /// Capacitors carrying an explicit `IC=` initial condition. Empty for
    /// circuits that don't use `IC=` (the common case — keeps those
    /// circuits byte-identical downstream).
    pub capacitor_ics: Vec<CapacitorIcInfo>,
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
    /// Rust field name codegen should emit on CircuitState.
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
    /// The operating point `.linearize` extracted its small-signal parameters
    /// at, by node name. It satisfies this system's DC equations by
    /// construction (the linearized devices' Norton constants come from it), so
    /// the DC operating point starts there. `None` when nothing is linearized,
    /// or when the bias solve did not converge (a build refuses that unless
    /// `--allow-unconverged-dc-op`).
    pub linearize_bias_nodes: Option<std::collections::BTreeMap<String, f64>>,
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

impl MnaSystem {
    /// Whether any inductor saturates (`ISAT=`). Saturation lives only on
    /// uncoupled inductors — a saturating tightly-coupled group is realized as
    /// a T-model whose single `{ref}_mag` inductor carries the core's ISAT, and
    /// every other saturating coupled group is refused while the MNA is built.
    pub fn has_saturating_inductor(&self) -> bool {
        self.inductors.iter().any(|ind| ind.isat.is_some())
    }

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
            behavioral_sources: Vec::new(),
            vcvs_sources: Vec::new(),
            current_sources: Vec::new(),
            capacitor_ics: Vec::new(),
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
            linearize_bias_nodes: None,
            linearized_triodes: Vec::new(),
            pot_default_overrides: HashMap::new(),
            switch_default_overrides: HashMap::new(),
            wiper_groups: Vec::new(),
            gang_groups: Vec::new(),
        }
    }

    /// Check if this MNA system has any inductors (uncoupled, coupled pairs, or transformer groups).
    pub fn has_inductors(&self) -> bool {
        !self.inductors.is_empty()
            || !self.coupled_inductors.is_empty()
            || !self.transformer_groups.is_empty()
    }

    /// Every node name, ordered by MNA index — which is netlist appearance
    /// order, ground (`"0"`) first.
    ///
    /// **Use this for any node list a human or a diff will read.** `node_map`
    /// is a `HashMap`, so iterating its keys gives a different order on every
    /// run: three runs of one failing command produced `["0","a","b","out"]`,
    /// `["b","out","a","0"]` and `["out","a","b","0"]`. An error message that
    /// will not compare equal to itself cannot be diffed, pasted into a bug
    /// report or asserted on in a test. Same defect class as the codegen
    /// HashMap-ordering bug fixed in `49ecaa4`, which is why the rule is
    /// "nothing user-visible is ordered by hash iteration".
    pub fn node_names_in_index_order(&self) -> Vec<&str> {
        let mut names: Vec<(usize, &str)> = self
            .node_map
            .iter()
            .map(|(name, idx)| (*idx, name.as_str()))
            .collect();
        // Index first, name as a tiebreaker so the order is total even if two
        // names ever share an index.
        names.sort_unstable();
        names.into_iter().map(|(_, name)| name).collect()
    }
}

/// Large conductance for modeling short circuits at DC \[S\].
///
/// Used only in the DC operating-point solver to stamp inductors as short circuits.
/// NOT used for voltage sources or VCVS — those use augmented MNA (extra variables
/// for source currents, enforcing V_plus - V_minus = V_dc exactly).
pub const DC_SHORT_CONDUCTANCE: f64 = 1e3;

/// Maximum supported system dimension.
///
/// One bound, checked twice: against the circuit node count when the MNA is
/// assembled (prevents unbounded allocation from pathological netlists), and
/// against the full augmented dimension — circuit nodes, voltage-source
/// branch currents, VCVS augmented rows, inductor branch variables — when the
/// DK kernel is built (prevents O(N^3) blowup from matrix inversion).
///
/// N=256 is generous for any real audio circuit (a passive tube EQ is ~41 nodes).
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

#[cfg(test)]
mod tests;
