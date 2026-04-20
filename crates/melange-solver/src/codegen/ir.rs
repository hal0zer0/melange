//! Circuit intermediate representation (IR) for language-agnostic code generation.
//!
//! `CircuitIR` captures everything a code emitter needs to produce a working solver,
//! without referencing any Rust-specific types from the MNA/DK pipeline.

use serde::{Deserialize, Serialize};

use crate::dc_op::{self, DcOpConfig};
use crate::dk::{self, DkKernel};
use crate::lu::{self, SPARSITY_THRESHOLD};
pub use crate::lu::{LuOp, LuSparsity};
use crate::mna::MnaSystem;
use crate::parser::{Element, Netlist};

use super::{CodegenConfig, CodegenError};

/// Solver method for code generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum SolverMode {
    /// DK method: precompute S=A⁻¹, NR in M-dimensional current space.
    /// Fast (O(N²+M³) per sample) but requires well-conditioned A and K[i][i]<0.
    #[default]
    Dk,
    /// Full-nodal NR: LU solve in N-dimensional voltage space per NR iteration.
    /// Handles any circuit topology including transformer-coupled NFB.
    /// Slower (O(N³) per sample) but universally convergent.
    Nodal,
}

/// Language-agnostic intermediate representation of a compiled circuit.
///
/// Built from `DkKernel` + `MnaSystem` + `Netlist` + `CodegenConfig`, this
/// struct contains every piece of data an emitter needs — matrices, device
/// parameters, solver config — without referencing the builder types.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub struct CircuitIR {
    pub metadata: CircuitMetadata,
    pub topology: Topology,
    /// DK or Nodal solver mode
    #[serde(default)]
    pub solver_mode: SolverMode,
    pub solver_config: SolverConfig,
    pub matrices: Matrices,
    pub dc_operating_point: Vec<f64>,
    pub device_slots: Vec<DeviceSlot>,
    pub has_dc_sources: bool,
    pub has_dc_op: bool,
    /// M-vector: nonlinear device currents at DC operating point
    #[serde(default)]
    pub dc_nl_currents: Vec<f64>,
    /// Whether the nonlinear DC OP solver converged
    #[serde(default)]
    pub dc_op_converged: bool,
    /// DC OP convergence method name (e.g. "DirectNR", "SourceStepping").
    #[serde(default)]
    pub dc_op_method: String,
    /// DC OP total NR iterations used.
    #[serde(default)]
    pub dc_op_iterations: usize,
    /// Whether to include DC blocking filter on outputs.
    pub dc_block: bool,
    pub inductors: Vec<InductorIR>,
    pub coupled_inductors: Vec<CoupledInductorIR>,
    pub transformer_groups: Vec<TransformerGroupIR>,
    /// Saturating (iron-core) inductors: per-sample L(I) update via SM rank-1.
    #[serde(default)]
    pub saturating_inductors: Vec<SaturatingInductorIR>,
    /// Saturating coupled inductor pairs: per-sample 2×2 eigendecomposition + 2 SM rank-1.
    #[serde(default)]
    pub saturating_coupled: Vec<SaturatingCoupledInductorIR>,
    /// Saturating transformer groups (3+ windings): per-sample W² elementary SM rank-1.
    #[serde(default)]
    pub saturating_xfmr_groups: Vec<SaturatingTransformerGroupIR>,
    pub pots: Vec<PotentiometerIR>,
    /// Wiper potentiometer groups (two linked pots per group).
    #[serde(default)]
    pub wiper_groups: Vec<WiperGroupIR>,
    /// Gang groups (multiple pots/wipers under one parameter).
    #[serde(default)]
    pub gang_groups: Vec<GangGroupIR>,
    pub switches: Vec<SwitchIR>,
    /// Op-amp output voltage saturation clamps.
    /// Only populated for op-amps with finite VSAT.
    #[serde(default)]
    pub opamps: Vec<OpampIR>,
    /// Pre-analyzed sparsity patterns for compile-time matrices.
    #[serde(default)]
    pub sparsity: SparseInfo,
    /// IIR op-amp filters: per-op-amp discrete-time dominant pole data.
    /// Populated for each op-amp with finite GBW > 0.
    #[serde(default)]
    pub opamp_iir: Vec<OpampIirData>,
    /// Authentic circuit noise configuration + source lists. See `NoiseIR`.
    #[serde(default)]
    pub noise: NoiseIR,
    /// Named topology constants emitted into generated code so plugins can
    /// reference nodes, VS rows, and pots by name instead of by numeric index.
    /// See Oomox plugin roadmap P2 + P3.
    #[serde(default)]
    pub named_constants: NamedConstantsIR,
    /// `.runtime`-bound voltage sources. Codegen emits one `pub <field>: f64`
    /// on `CircuitState` per entry and stamps `rhs[vs_row] += state.<field>`
    /// in both trapezoidal and backward-Euler RHS builders. See Oomox P1.
    #[serde(default)]
    pub runtime_sources: Vec<RuntimeSourceIR>,
}

/// `.runtime` voltage source in IR form.
///
/// Mirrors `mna::RuntimeSourceInfo` but without the MNA coupling so it can
/// round-trip through serde for IR snapshotting.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RuntimeSourceIR {
    /// Voltage source name (for diagnostics).
    pub vs_name: String,
    /// Rust identifier for the `CircuitState` field.
    pub field_name: String,
    /// Aug-MNA row that receives `state.<field_name>` each sample.
    pub vs_row: usize,
}

/// Named topology constants for plugin-runtime indexing.
///
/// Emitted as `pub const NODE_<N>: usize`, `pub const VSOURCE_<N>_RHS_ROW: usize`,
/// and `pub const POT_<N>_INDEX: usize` so plugin code can refer to matrix/vector
/// rows and pot slots by name rather than by position-dependent numeric index.
///
/// Names are sanitized to SCREAMING_SNAKE: non-alphanumeric → `_`, leading digit
/// prefixed with `_`, uppercased. Collisions (two netlist names sanitizing to the
/// same ident) get a numeric suffix (`_2`, `_3`, …) in declaration order.
///
/// Auto-generated internal nodes (BJT `basePrime`/`colPrime`/`emitPrime`, transformer
/// branch currents) are intentionally NOT emitted — those are solver implementation
/// details and plugins must not take dependencies on them.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NamedConstantsIR {
    /// User-named circuit nodes: (sanitized const suffix, 0-based node index).
    /// Ground is implicit (index 0) and not emitted.
    pub nodes: Vec<(String, usize)>,
    /// Voltage sources: (sanitized const suffix, RHS row = n_nodes + vs.ext_idx).
    /// The row index is the aug-MNA row where the VS's KVL constraint lives,
    /// i.e. where a `.runtime` voltage source (P1) stamps its per-sample value.
    pub vsources: Vec<(String, usize)>,
    /// Pots: (sanitized const suffix, index into the pot array on CircuitState).
    pub pots: Vec<(String, usize)>,
}

/// Circuit metadata (name, title, generator version).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CircuitMetadata {
    pub circuit_name: String,
    pub title: String,
    pub generator_version: String,
}

/// Circuit topology dimensions.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub struct Topology {
    /// System dimension = n_aug (n + num_vs + num_vcvs), or n_nodal when augmented inductors are used.
    /// This is the size of all N-indexed matrices and vectors in the solver.
    pub n: usize,
    /// Original circuit node count (excluding ground and augmented VS/VCVS variables).
    /// Output node indices must be < n_nodes.
    #[serde(default)]
    pub n_nodes: usize,
    /// Total nonlinear dimension (sum of device dimensions)
    pub m: usize,
    /// Number of physical nonlinear devices
    pub num_devices: usize,
    /// Boundary between VS/VCVS rows and inductor branch variables.
    /// Equal to mna.n_aug (= n_nodes + num_vs + num_vcvs).
    /// A_neg rows for VS/VCVS algebraic constraints (at n_nodes + vs.ext_idx etc.)
    /// should be zeroed. Inductor rows (n_aug..n) and internal BJT node rows should NOT.
    #[serde(default)]
    pub n_aug: usize,
    /// True when inductors use augmented MNA (branch current variables in G/C)
    /// instead of companion model (history currents in state).
    #[serde(default)]
    pub augmented_inductors: bool,
    /// Number of devices linearized at DC OP (triodes + BJTs).
    /// Linearized devices are stamped as small-signal conductances in G,
    /// creating high-gain coupling chains that inflate S = A^{-1} entries
    /// without making K = N_V * S * N_I ill-conditioned.
    #[serde(default)]
    pub num_linearized_devices: usize,
}

/// Solver configuration baked into the generated code.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub struct SolverConfig {
    pub sample_rate: f64,
    /// alpha = 2/T (trapezoidal) or 1/T (backward Euler) at internal sample rate
    pub alpha: f64,
    pub tolerance: f64,
    pub max_iterations: usize,
    pub input_node: usize,
    /// Output node indices (one per output channel)
    #[serde(default = "default_output_nodes")]
    pub output_nodes: Vec<usize>,
    pub input_resistance: f64,
    /// Oversampling factor (1, 2, or 4). Default 1 (no oversampling).
    #[serde(default = "default_oversampling_factor")]
    pub oversampling_factor: usize,
    /// Output scale factors applied after DC blocking (one per output)
    #[serde(default = "default_output_scales")]
    pub output_scales: Vec<f64>,
    /// Silent samples to process after pot-triggered matrix rebuild (default 64).
    #[serde(default = "default_pot_settle_samples")]
    pub pot_settle_samples: usize,
    /// Use backward Euler integration (unconditionally stable, first-order).
    #[serde(default)]
    pub backward_euler: bool,
    /// Resolved op-amp supply rail saturation strategy.
    ///
    /// If the user's [`CodegenConfig::opamp_rail_mode`] was [`OpampRailMode::Auto`],
    /// this holds the concrete mode chosen by [`resolve_opamp_rail_mode`]. If the
    /// user specified a concrete mode, that mode is stored verbatim. The emitter
    /// never sees [`OpampRailMode::Auto`] — it's resolved by the time the IR is
    /// built.
    ///
    /// [`CodegenConfig::opamp_rail_mode`]: crate::codegen::CodegenConfig::opamp_rail_mode
    /// [`OpampRailMode::Auto`]: crate::codegen::OpampRailMode::Auto
    #[serde(default = "default_opamp_rail_mode")]
    pub opamp_rail_mode: crate::codegen::OpampRailMode,
    /// Emit `CircuitState::recompute_dc_op()` for runtime DC operating-point
    /// re-solve (Oomox roadmap P6 / Phase E). Default `false` → output is
    /// byte-identical to pre-Phase-E codegen. Threaded from
    /// [`CodegenConfig::emit_dc_op_recompute`].
    ///
    /// [`CodegenConfig::emit_dc_op_recompute`]: crate::codegen::CodegenConfig::emit_dc_op_recompute
    #[serde(default)]
    pub emit_dc_op_recompute: bool,
}

fn default_opamp_rail_mode() -> crate::codegen::OpampRailMode {
    // Deserialized IRs without an explicit mode default to the pre-2026-04
    // behavior (hard clamp). `Auto` is only valid as a user-facing input;
    // by the time an IR is constructed, the mode is always concrete.
    crate::codegen::OpampRailMode::Hard
}

/// Reason recorded for an auto-detection decision. Used for logging so users
/// can see *why* a particular mode was chosen. Each variant carries enough
/// information to be reconstructable from the MNA alone — no references to
/// specific circuit names.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OpampRailModeReason {
    /// User explicitly specified the mode; no auto-detection ran.
    UserRequested,
    /// Circuit has no op-amps with finite supply rails, so no clamping is needed.
    NoClampedOpamps,
    /// All op-amp outputs are DC-coupled to their downstream networks. The
    /// hard post-NR clamp is safe — no cap history to corrupt — and cheap.
    AllDcCoupled,
    /// At least one clamped op-amp has an output cap coupling into a downstream
    /// non-feedback node. The post-NR hard clamp would corrupt that cap's
    /// trapezoidal history on rail-violating samples. Auto-select `ActiveSet`
    /// to keep KCL consistent. (When `BoyleDiodes` lands, this reason will
    /// also be used to upgrade distortion-class circuits further.)
    AcCoupledDownstream,
    /// At least one clamped op-amp output is connected via an R-only path
    /// (no series caps) to a nonlinear device terminal. This is the
    /// sidechain / control-path topology pattern (e.g. compressor envelope
    /// detector: op-amp output → R → diode anode → cap to ground). The
    /// rail-clamped DC value drives the nonlinear device's operating point,
    /// so any damping (BE switch on engagement) would change the envelope
    /// dynamics. Use plain `ActiveSet` (trap+pin) which preserves the steady
    /// rail value.
    AcCoupledDownstreamControlPath,
    /// All clamped op-amps have only AC-coupled downstream paths to
    /// nonlinear devices (or no nonlinear devices at all). This is the
    /// audio-path topology where the op-amp drives a coupling cap to the
    /// next stage. Trap+pin develops a Nyquist-rate limit cycle on these
    /// circuits when the rail is engaged across multiple samples; switch to
    /// `ActiveSetBe` so the BE fallback runs the constrained re-solve and
    /// damps the ringing.
    AcCoupledDownstreamAudioPath,
}

impl OpampRailModeReason {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::UserRequested => "user requested",
            Self::NoClampedOpamps => "no op-amps with finite VCC/VEE",
            Self::AllDcCoupled => "op-amps only DC-coupled downstream (hard clamp safe)",
            Self::AcCoupledDownstream => "op-amp output has cap-coupled downstream (active-set keeps cap history consistent)",
            Self::AcCoupledDownstreamControlPath => {
                "op-amp output has R-only path to a nonlinear device terminal (sidechain/control path — trap+pin preserves steady DC rail)"
            }
            Self::AcCoupledDownstreamAudioPath => {
                "op-amp output is cap-coupled with no R-only path to nonlinear devices (audio path — BE-on-clamp damps Nyquist limit cycle)"
            }
        }
    }
}

/// Result of [`resolve_opamp_rail_mode`]: the concrete mode that should be
/// baked into the generated code, plus the reason it was chosen.
#[derive(Debug, Clone)]
pub struct ResolvedOpampRailMode {
    pub mode: crate::codegen::OpampRailMode,
    pub reason: OpampRailModeReason,
}

/// Returns `true` if the op-amp output node `out_idx` (1-indexed) has any
/// capacitive coupling to a non-feedback node in the MNA `C` matrix.
///
/// "Non-feedback" means: not the op-amp's own inverting input, not ground,
/// and not the out node itself. Feedback caps (between `out` and the `-`
/// input) are fine for the hard-clamp path because they're not downstream
/// integrators — they're part of the op-amp's own closed loop. Output
/// coupling caps (between `out` and a different stage's input) are the
/// ones that corrupt cap history when the op-amp is post-clamped.
///
/// Returns `false` if the op-amp has `out_idx == 0` (grounded / invalid).
fn opamp_has_ac_coupled_downstream(
    mna: &crate::mna::MnaSystem,
    opamp: &crate::mna::OpampInfo,
) -> bool {
    if opamp.n_out_idx == 0 {
        return false;
    }
    let out = opamp.n_out_idx - 1;   // 0-indexed
    // Optional: the op-amp's own inverting input (for skipping feedback caps).
    // 0 means grounded in the MNA's 1-indexed convention.
    let inverting_input: Option<usize> = if opamp.n_minus_idx > 0 {
        Some(opamp.n_minus_idx - 1)
    } else {
        None
    };

    // C matrix is NxN (where N is mna.n, the current MNA dimension). A cap
    // between nodes i and j stamps into C[i][i], C[j][j], C[i][j], C[j][i].
    // We scan row `out` of C for non-zero off-diagonal entries.
    let n = mna.c.len();
    if out >= n {
        return false;
    }
    let row = &mna.c[out];
    for (other, &c) in row.iter().enumerate() {
        if other == out {
            continue; // self-diagonal
        }
        if c == 0.0 {
            continue;
        }
        if let Some(nm) = inverting_input {
            if other == nm {
                // Feedback cap — doesn't count as downstream coupling.
                continue;
            }
        }
        // Any other non-zero entry is a coupling cap to another circuit node.
        return true;
    }
    false
}

/// Returns `true` if the op-amp's output node has an R-only path (no series
/// capacitors) to any nonlinear device terminal.
///
/// This is the **control-path / sidechain criterion**: when the op-amp output
/// is wired through resistors directly to a diode/BJT/JFET/MOSFET/tube/VCA
/// terminal, the rail-clamped DC value drives that device's operating point.
/// Switching to BE on engagement (the [`OpampRailMode::ActiveSetBe`] strategy)
/// would damp this DC behavior and change the envelope dynamics — wrong for
/// compressors, expanders, ALCs, and any sidechain rectifier. So if any
/// clampable op-amp matches this pattern, the auto-detector picks plain
/// [`OpampRailMode::ActiveSet`] (trap+pin), which preserves the rail value.
///
/// **What "R-only path" means**: BFS from the op-amp output through resistor
/// edges only. We never traverse capacitor edges. The op-amp's own inverting
/// input is excluded (resistor feedback through the closed loop is not a
/// downstream control path). Ground is excluded.
///
/// **What "nonlinear device terminal" means**: any node listed in
/// [`crate::mna::NonlinearDeviceInfo::node_indices`] for any device in
/// `mna.nonlinear_devices` — diode anode/cathode, BJT base/collector/emitter,
/// JFET/MOSFET gate/drain/source, tube grid/plate/cathode, VCA signal/control
/// pins.
///
/// **Bound**: BFS depth is bounded by the number of nodes (no infinite loops
/// possible — the visited set guarantees termination in O(N + E) where N is
/// nodes and E is resistor count).
///
/// **Why R-only and not "any linear element"**: caps in series provide
/// AC-coupling that decouples the DC rail value from the downstream device's
/// operating point. With a series cap, the cap charges to the rail offset
/// and the downstream node sees only AC excursion. The rail value's specific
/// magnitude doesn't affect the downstream operating point. Without caps,
/// the rail value IS what the downstream node sees at DC.
///
/// [`OpampRailMode::ActiveSet`]: crate::codegen::OpampRailMode::ActiveSet
/// [`OpampRailMode::ActiveSetBe`]: crate::codegen::OpampRailMode::ActiveSetBe
fn opamp_has_r_only_path_to_nonlinear(
    netlist: &crate::parser::Netlist,
    mna: &crate::mna::MnaSystem,
    opamp: &crate::mna::OpampInfo,
) -> bool {
    use crate::parser::Element;
    use std::collections::HashSet;

    if opamp.n_out_idx == 0 {
        return false;
    }

    // Resolve the op-amp output and inverting-input node names from indices.
    // We use names because netlist Elements are name-keyed.
    let name_of_idx = |idx: usize| -> Option<String> {
        if idx == 0 {
            return None;
        }
        mna.node_map
            .iter()
            .find_map(|(name, &i)| if i == idx { Some(name.clone()) } else { None })
    };
    let out_name = match name_of_idx(opamp.n_out_idx) {
        Some(n) => n,
        None => return false,
    };
    let inv_name = name_of_idx(opamp.n_minus_idx); // None if grounded

    // Collect names of all nodes that are DC voltage source terminals. These
    // are bias rails (Vbias, VCC, VEE, etc.) — KCL-clamped to a fixed
    // potential by an ideal source, so they're effectively ground for AC
    // purposes. The BFS treats them as stop nodes: we don't traverse through
    // them, and we don't count them as nonlinear-device terminals even if a
    // diode happens to anchor to them.
    //
    // Why this matters: in distortion pedals, shunt clipping diodes are
    // anchored to a DC bias node (e.g. Klon: D2 anode = diode_jct, cathode =
    // vbias). The bias rail is a voltage source. If we counted vbias as a
    // "nonlinear terminal", every R-network path to the bias rail would
    // falsely register as a control path, because every op-amp's bias path
    // touches the rail somewhere. The DC level on the op-amp output isn't
    // actually driving the diode through that path — the diode is referenced
    // to vbias, not to the op-amp signal.
    let dc_source_nodes: HashSet<String> = netlist
        .elements
        .iter()
        .filter_map(|e| {
            if let Element::VoltageSource { n_plus, n_minus, .. } = e {
                Some([n_plus.clone(), n_minus.clone()])
            } else {
                None
            }
        })
        .flatten()
        .filter(|n| n != "0")
        .collect();

    // Collect names of all nodes that are nonlinear-device terminals AND not
    // DC source nodes. The exclusion prevents the false positive described
    // above.
    let nonlinear_terminal_names: HashSet<&String> = mna
        .nonlinear_devices
        .iter()
        .flat_map(|d| d.nodes.iter())
        .filter(|n| !dc_source_nodes.contains(n.as_str()))
        .collect();

    // BFS from the op-amp output through resistor edges only. Stop at:
    // - ground ("0")
    // - the op-amp's own inverting input (feedback loop)
    // - any DC voltage source node (bias rails are AC-ground)
    //
    // The starting node is NEVER counted as a downstream terminal. A
    // nonlinear device directly anchored to the op-amp output is a local
    // feedback element (classic diode-feedback clipper: D1 anode = op-amp
    // output, cathode = inv input) — not a control-path driver. A real
    // control path requires at least one R-hop so the op-amp drives the
    // downstream device through a signal-path resistor (e.g. VCR ALC
    // sidechain: `Rsc op_out sc_node 10k; D1 sc_node cv_node`). Tracking
    // the starting node explicitly via `start_node` keeps the BFS
    // conditional simple.
    let start_node = out_name.clone();
    let mut visited: HashSet<String> = HashSet::new();
    let mut frontier: Vec<String> = vec![out_name.clone()];
    visited.insert(out_name);

    while let Some(node) = frontier.pop() {
        // If this node is itself a (filtered) nonlinear-device terminal,
        // we've found a real control path — but only if we reached it
        // via an R-hop from the op-amp output, not as the seed itself.
        if node != start_node && nonlinear_terminal_names.contains(&node) {
            return true;
        }

        // Walk every resistor in the netlist looking for one that touches `node`.
        // O(N · E) total over the BFS — fine for typical circuit sizes (E < 1000).
        for elem in &netlist.elements {
            if let Element::Resistor {
                n_plus, n_minus, ..
            } = elem
            {
                let other = if n_plus.eq_ignore_ascii_case(&node) {
                    Some(n_minus.clone())
                } else if n_minus.eq_ignore_ascii_case(&node) {
                    Some(n_plus.clone())
                } else {
                    None
                };

                if let Some(other_node) = other {
                    // Skip ground.
                    if other_node == "0" {
                        continue;
                    }
                    // Skip the op-amp's own inverting input (feedback path).
                    if let Some(inv) = &inv_name {
                        if other_node.eq_ignore_ascii_case(inv) {
                            continue;
                        }
                    }
                    // Skip DC voltage source nodes — bias rails terminate
                    // the BFS because they're AC-ground.
                    if dc_source_nodes.contains(&other_node) {
                        continue;
                    }
                    if !visited.contains(&other_node) {
                        visited.insert(other_node.clone());
                        frontier.push(other_node);
                    }
                }
            }
        }
    }

    false
}

/// Resolve an [`OpampRailMode`] request into a concrete mode using the MNA
/// as the basis for auto-detection.
///
/// # Decision rules (Auto mode)
///
/// 1. **No op-amps with finite VCC/VEE** → [`OpampRailMode::None`]. Nothing
///    to clamp; the user modeled all op-amps as ideal VCCSs.
///
/// 2. **All clamped op-amp outputs are DC-coupled to their downstream**
///    (no output coupling caps, only feedback caps) → [`OpampRailMode::Hard`].
///    The cheap post-NR hard clamp is KCL-safe in this topology because
///    there's no cap history to corrupt.
///
/// 3. **At least one clamped op-amp has a cap coupling to a non-feedback
///    downstream node** → [`OpampRailMode::ActiveSet`]. The hard clamp
///    would corrupt that cap's trapezoidal history on rail-violating
///    samples; the active-set resolve keeps KCL consistent.
///
/// The future `OpampRailMode::BoyleDiodes` upgrade (Step 5 in the task
/// series) will make rule 3 promote distortion-class circuits even further
/// — to a Boyle catch-diode model that produces the soft exponential knee
/// characteristic of real op-amp saturation. For Step 6 we stop at
/// `ActiveSet`, which fixes the numerical bug without changing distortion
/// character; `BoyleDiodes` will change character and lands separately.
///
/// # Explicit user overrides
///
/// Any explicit mode (anything other than [`OpampRailMode::Auto`]) is
/// returned verbatim with [`OpampRailModeReason::UserRequested`]. The
/// resolver NEVER silently upgrades a user's explicit choice — even
/// `None` on a circuit that would otherwise pick `ActiveSet` is honored,
/// because the override is how users bisect and measure.
///
/// [`OpampRailMode`]: crate::codegen::OpampRailMode
/// [`OpampRailMode::None`]: crate::codegen::OpampRailMode::None
/// [`OpampRailMode::Hard`]: crate::codegen::OpampRailMode::Hard
/// [`OpampRailMode::ActiveSet`]: crate::codegen::OpampRailMode::ActiveSet
/// [`OpampRailMode::BoyleDiodes`]: crate::codegen::OpampRailMode::BoyleDiodes
pub fn resolve_opamp_rail_mode(
    mna: &crate::mna::MnaSystem,
    requested: crate::codegen::OpampRailMode,
) -> ResolvedOpampRailMode {
    use crate::codegen::OpampRailMode;

    // Explicit request wins. User overrides exist to bisect and measure; the
    // auto-detector must never silently override a user choice.
    if requested != OpampRailMode::Auto {
        return ResolvedOpampRailMode {
            mode: requested,
            reason: OpampRailModeReason::UserRequested,
        };
    }

    // Collect op-amps that actually need clamping. Infinities (set by the
    // MNA resolver when no rails were declared) mean the user modeled an
    // ideal VCCS — nothing to clamp.
    let clamped_opamps: Vec<&crate::mna::OpampInfo> = mna
        .opamps
        .iter()
        .filter(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()))
        .collect();

    if clamped_opamps.is_empty() {
        return ResolvedOpampRailMode {
            mode: OpampRailMode::None,
            reason: OpampRailModeReason::NoClampedOpamps,
        };
    }

    // Check whether ANY clamped op-amp has a non-feedback output coupling cap.
    // A single offender is enough to force ActiveSet for the whole circuit —
    // we can't mix modes within one generated function, and the KCL corruption
    // only needs one cap to blow up downstream.
    let ac_coupled = clamped_opamps
        .iter()
        .any(|oa| opamp_has_ac_coupled_downstream(mna, oa));

    if ac_coupled {
        ResolvedOpampRailMode {
            mode: OpampRailMode::ActiveSet,
            reason: OpampRailModeReason::AcCoupledDownstream,
        }
    } else {
        ResolvedOpampRailMode {
            mode: OpampRailMode::Hard,
            reason: OpampRailModeReason::AllDcCoupled,
        }
    }
}

/// Refine an [`OpampRailMode::ActiveSet`] auto-decision by inspecting the
/// netlist topology. If no clampable op-amp has an R-only path to a nonlinear
/// device terminal, the circuit is "audio-path" — upgrade to
/// [`OpampRailMode::ActiveSetBe`] which damps the trap+pin Nyquist limit
/// cycle by switching to BE matrices on rail engagement. If at least one
/// op-amp does have such a path, the circuit is "control-path" (sidechain
/// rectifier, compressor envelope detector, etc.) and stays on plain
/// [`OpampRailMode::ActiveSet`] so the steady DC rail value is preserved.
///
/// This refinement runs *after* [`resolve_opamp_rail_mode`] because the
/// initial resolve only needs the MNA, but the topology check needs the raw
/// netlist (to enumerate `Element::Resistor` rather than the post-stamping
/// `G` matrix, which conflates resistors with op-amp VCCS entries). Splitting
/// the resolve in two avoids threading `&Netlist` through ~20 call sites that
/// don't need it.
///
/// # When this is called
///
/// Call this from any code path that has both the MNA and the original
/// netlist available (e.g. `CodeGenerator::generate_with_dc_op` and
/// `CodeGenerator::generate_nodal`). User overrides are honored unchanged —
/// the refinement only runs when the input was an Auto decision that
/// resolved to `ActiveSet`.
///
/// # Why "any" rather than "all"
///
/// One sidechain op-amp is enough to ruin the compressor envelope if BE
/// damping kicks in for the whole circuit. Generated code emits one rail
/// strategy for the entire process_sample function — we can't mix
/// strategies per op-amp. So we conservatively pick the strategy that's
/// safe for ALL clampable op-amps: if even one needs the trap+pin steady
/// rail, plain ActiveSet wins.
pub fn refine_active_set_for_audio_path(
    resolved: ResolvedOpampRailMode,
    mna: &crate::mna::MnaSystem,
    netlist: &crate::parser::Netlist,
) -> ResolvedOpampRailMode {
    use crate::codegen::OpampRailMode;

    // Only refine the auto-resolved ActiveSet decision. User overrides
    // (UserRequested) and other modes pass through unchanged.
    if resolved.mode != OpampRailMode::ActiveSet
        || resolved.reason != OpampRailModeReason::AcCoupledDownstream
    {
        return resolved;
    }

    let clamped_opamps: Vec<&crate::mna::OpampInfo> = mna
        .opamps
        .iter()
        .filter(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()))
        .collect();

    let any_control_path = clamped_opamps
        .iter()
        .any(|oa| opamp_has_r_only_path_to_nonlinear(netlist, mna, oa));

    if any_control_path {
        ResolvedOpampRailMode {
            mode: OpampRailMode::ActiveSet,
            reason: OpampRailModeReason::AcCoupledDownstreamControlPath,
        }
    } else {
        ResolvedOpampRailMode {
            mode: OpampRailMode::ActiveSetBe,
            reason: OpampRailModeReason::AcCoupledDownstreamAudioPath,
        }
    }
}

/// Name of the synthetic catch-diode model inserted by
/// [`augment_netlist_with_boyle_diodes`]. Standard Boyle-macromodel silicon
/// diode parameters (`Is = 1e-15, N = 1`), matching every commercial SPICE
/// op-amp model's output-stage clamp diodes.
pub const BOYLE_CATCH_DIODE_MODEL: &str = "D_BOYLE_CATCH";

/// Augment a parsed netlist with the Boyle-style internal-gain-node op-amp
/// model (catch diodes + output buffer), returning a fresh [`Netlist`] that
/// contains the original elements plus the synthesized internal-node
/// scaffolding.
///
/// # Mechanism
///
/// For each op-amp with at least one finite supply rail, this function adds
/// the elements required to turn melange's flat linear-VCCS op-amp model into
/// a two-stage Boyle macromodel:
///
/// ```text
///   V+ ─┐                                          ┌─→ V_out
///       │                                          │
///   V- ─┴── Gm ──→ [_oa_int_{name}] ── buffer ─────┘
///                       │  ▲                      │
///              R_int ───┘  │                      R_out (shunt)
///              (1 MΩ,       │                      ↓
///              stamped by   │                      0
///              MNA, not     │
///              in netlist)  │
///                           │
///                catch diodes pin
///                this node to ±rail
/// ```
///
/// **Per clamped op-amp**, the function adds:
///
/// - A unity-gain output buffer VCCS `G_oa_buf_{name}` from
///   `_oa_int_{name}` (control) → original output node (output) with
///   `gm = 1 / r_out`. Combined with the output shunt resistor below, this
///   gives `V_out = V_int` at DC and source impedance `≈ r_out`.
///
/// - An output shunt resistor `R_oa_buf_out_{name}` from the original
///   output node to ground with `R = r_out`. Provides the KCL ground path
///   the buffer needs and sets the closed-loop output impedance.
///
/// - For each finite rail, a reference node `_boyle_hi_{name}` /
///   `_boyle_lo_{name}` pinned to `VCC − VOH_DROP` / `VEE + VOL_DROP` by a
///   synthesized DC voltage source, and a catch diode between
///   **`_oa_int_{name}`** and that reference node. The diodes are placed
///   on the internal node — NOT on the original output — so they only have
///   to balance the small current produced by the in-MNA Gm injection
///   (≈ 0.2 A peak for an `AOL = 200000` part), instead of fighting the
///   ~4000 S linear-VCCS conductance present on the original output.
///
/// - One top-level `.model D_BOYLE_CATCH D(IS=1e-15 N=1)` definition, added
///   only once. Standard silicon parameters used by every commercial SPICE
///   op-amp macromodel (Boyle/Cohn/Pederson/Solomon JSSC 1974; reproduced
///   in TI's PSpice TL072 model, ngspice's built-in op-amp, LTSpice's
///   `UniversalOpamp2`, etc.).
///
/// The internal node `_oa_int_{name}` is NOT a netlist resistor — it's
/// referenced only by the buffer/diodes. When MNA stamping sees the name in
/// `node_map`, it switches that op-amp's transconductance stamp from the
/// original output node to the internal node (using
/// [`crate::mna::R_BOYLE_INT_LOAD`] as the effective output resistance). See
/// [`crate::mna::MnaSystem::from_netlist`] for the dispatch.
///
/// VOH_DROP and VOL_DROP default to 1.5 V in [`crate::mna::OpampInfo`] for
/// TL072/NE5532-class parts and can be overridden in the user's
/// `.model OA(VOH_DROP=… VOL_DROP=…)` for rail-to-rail op-amps.
///
/// # Why this specific topology
///
/// A linear VCCS (the pre-2026-04 op-amp model in melange) has unbounded
/// output. Three ways to bound it:
///
/// 1. **Hard clamp `v[out] = clamp(v[out], VEE, VCC)`** — cheap but
///    (a) breaks KCL for AC-coupled downstream caps, and (b) produces a
///    square-law limiter's flat odd-harmonic series instead of the soft
///    exponential knee real op-amps exhibit.
///
/// 2. **Piecewise linear catch sources** (the
///    [`OpampRailMode::ActiveSet`](crate::codegen::OpampRailMode::ActiveSet)
///    path) — KCL-consistent but still produces hard-clip harmonics.
///
/// 3. **Boyle catch diodes on an internal high-impedance gain node** (this
///    function) — silicon diodes anchored to rail-offset DC sources, hung
///    off the *internal* node so the diode's exponential conductance only
///    has to balance ~1 µS (the internal load), not ~4000 S (Gm of the
///    original linear VCCS). Produces the smooth exponential knee that
///    matches measured TL072 saturation and reproduces the "mid drive
///    crunch" of guitar overdrive pedals like the Klon Centaur. NR
///    converges where the naive on-output placement diverged.
///
/// # Contract
///
/// The returned netlist must compile through
/// [`crate::mna::MnaSystem::from_netlist`] without errors. All synthesized
/// node names are prefixed with `_boyle_` or `_oa_int_` to avoid collisions
/// with user-defined names. If a user happens to have a node named
/// `_boyle_hi_U1A` or `_oa_int_U1A`, behavior is undefined — treat the
/// prefixes as reserved.
///
/// Op-amps with no finite rails (ideal VCCS) or with grounded output
/// (`n_out_idx == 0`, which should have already been rejected earlier)
/// are skipped silently.
///
/// The original `netlist` is cloned, not consumed — callers can still
/// use the un-augmented netlist after this returns.
pub fn augment_netlist_with_boyle_diodes(
    netlist: &crate::parser::Netlist,
    mna: &crate::mna::MnaSystem,
) -> crate::parser::Netlist {
    use crate::parser::{Element, Model};

    let mut augmented = netlist.clone();

    // Add the shared catch-diode model once.
    if !augmented
        .models
        .iter()
        .any(|m| m.name == BOYLE_CATCH_DIODE_MODEL)
    {
        augmented.models.push(Model {
            name: BOYLE_CATCH_DIODE_MODEL.to_string(),
            model_type: "D".to_string(),
            params: vec![
                ("IS".to_string(), 1e-15),
                ("N".to_string(), 1.0),
            ],
        });
    }

    // Build a reverse lookup from node index (1-indexed, matching node_map)
    // back to node name so we can emit human-readable synthesized elements.
    // We only need names for op-amp output nodes.
    let name_of_idx = |idx: usize| -> Option<String> {
        mna.node_map
            .iter()
            .find_map(|(name, &i)| if i == idx { Some(name.clone()) } else { None })
    };

    for oa in &mna.opamps {
        if oa.n_out_idx == 0 {
            continue;
        }
        let has_upper = oa.vcc.is_finite();
        let has_lower = oa.vee.is_finite();
        if !has_upper && !has_lower {
            continue;
        }

        // Look up the user's output node name so the diode element reads
        // naturally in generated code. If the node isn't in the map something
        // is badly wrong upstream; skip silently rather than crashing.
        let out_name = match name_of_idx(oa.n_out_idx) {
            Some(s) => s,
            None => continue,
        };

        // Sanitize the op-amp name for use in synthesized identifiers:
        // SPICE node names are typically ASCII alphanumeric plus `_`.
        // We don't lowercase because the MNA node_map preserves case.
        // MUST match the sanitization in `crate::mna::MnaSystem::from_netlist`
        // op-amp stamping — the MNA dispatcher looks up `_oa_int_{safe_name}`
        // by exactly this rule.
        let safe_name: String = oa
            .name
            .chars()
            .map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' })
            .collect();

        let int_node = format!("_oa_int_{}", safe_name);
        let buf_out_node = format!("_oa_buf_out_{}", safe_name);

        // Dominant-pole capacitor from the internal gain node to ground.
        //
        // This is the "C1" compensation cap in Boyle's 1974 macromodel.
        // Together with the R1 load on the internal node (stamped directly
        // in `mna.rs` op-amp dispatch as `Go_int = 1 / R_BOYLE_INT_LOAD`),
        // it creates the first pole at `f_p = 1 / (2π · R1 · C_dom)`.
        // The unity-gain crossover frequency is then
        // `GBW = Gm1 / (2π · C_dom)` where `Gm1 = AOL / R1` (also stamped
        // in MNA dispatch), so inverting to solve for C_dom:
        //
        //   C_dom = Gm1 / (2π · GBW) = AOL / (2π · GBW · R1)
        //
        // For TL072 at AOL=200k, GBW=3 MHz, R1=1 MΩ: C_dom ≈ 10.6 nF.
        // That feels large because R1 is 1 MΩ (a macromodel convenience,
        // not a physical value); what matters for numerical behaviour is
        // the *trap-rule conductance* 2·C_dom/T. For the example at 48 kHz
        // that's 2·10.6e-9·48000 ≈ 1 mS — comfortably well-conditioned,
        // safely above the R1 shunt (1 µS) and well below rail-engaged
        // diode conductance (~1 S).
        //
        // Critically, the dominant pole SMOOTHS the internal-node voltage
        // across the rail transition. Without C_dom the catch diode
        // switches state infinitely fast between samples, which is what
        // excited the C15 Nyquist resonator in every previous attempt.
        // With C_dom present, the int-node voltage transitions smoothly
        // over a few samples, so the downstream cap-coupled output path
        // sees a continuously-differentiable signal and the trap-rule
        // integrator stays stable.
        //
        // If no GBW is specified in `.model OA()`, fall back to 10 nF —
        // gives a first pole near 16 Hz with R1=1 MΩ, rolling off 6 dB/oct
        // to unity gain around 2 MHz. Audibly transparent, numerically
        // well-conditioned.
        // NOTE: The Boyle 1974 macromodel includes a dominant-pole cap
        // at the internal gain node for AOL frequency rolloff, but in
        // melange's numerical experiments at 48 kHz, adding a cap from
        // int_node to ground creates an ill-conditioned row (the int
        // node has a tiny pre-cap diagonal from R1 = 1 MΩ vs large
        // off-diagonal Gm = 0.1 S) that breaks the compile-time sparse
        // LU at cap values above ~5 pF. Since audio-band bandwidth is
        // already much lower than any op-amp's GBW, the missing pole
        // has no audible effect, and we omit C_dom entirely in this
        // implementation. If a future test case requires band-limited
        // op-amp behaviour, the cap can be added at the buffer output
        // node (`_oa_buf_out_`) where the numerics are well-conditioned.

        // Output buffer VCVS: forces `V(buf_out) = V(int)` regardless of
        // load at the buffer output node. The user-facing output node
        // (`out_name`) is reached from `buf_out` through a series `R_ro`,
        // so the effective op-amp output impedance from `out_name` looking
        // back is `R_ro`. This matches Boyle's 1974 two-stage topology:
        // high-Z internal gain node → unity-gain buffer → small output
        // series R → external load.
        //
        // The VCVS form (not VCCS + shunt-to-ground) is essential: a
        // shunt-to-ground at the output would fight the op-amp's DC bias
        // for any circuit without a hard pull-up (e.g. Klon biases its
        // op-amps via a high-Z divider at vbias=4.5V — a 50Ω shunt would
        // dump 90 mA into ground at idle). The series-R topology adds
        // real source impedance WITHOUT creating any DC sink to ground.
        augmented.elements.push(Element::Vcvs {
            name: format!("E_oa_buf_{}", safe_name),
            out_p: buf_out_node.clone(),
            out_n: "0".to_string(),
            ctrl_p: int_node.clone(),
            ctrl_n: "0".to_string(),
            gain: 1.0,
        });

        // Output series resistance. Boyle's canonical value is 75 Ω,
        // matching TL072-class bipolar op-amps. For feedback-dominated
        // topologies (all audio circuits we target), closed-loop output
        // impedance is `R_ro / (1 + loop_gain) ≈ µΩ`, so 75 Ω has no
        // audible effect. For high-impedance loads (cap-coupled,
        // 10 kΩ+ feedback networks) the 75 Ω drop is a fraction of
        // a millivolt. Use the op-amp's `r_out` field if the user
        // overrode it in `.model OA(ROUT=…)`, otherwise 75 Ω.
        let r_out = if oa.r_out > 1.0 { oa.r_out } else { 75.0 };
        augmented.elements.push(Element::Resistor {
            name: format!("R_oa_ro_{}", safe_name),
            n_plus: buf_out_node,
            n_minus: out_name.clone(),
            value: r_out,
        });

        if has_upper {
            // Upper catch: internal node → (VCC − VOH_DROP) reference node.
            // Diode anode is the internal node so positive overdrive (V_int
            // climbing toward VCC) forward-biases the diode and pins V_int.
            let rail_node = format!("_boyle_hi_{}", safe_name);
            augmented.elements.push(Element::VoltageSource {
                name: format!("V_boyle_hi_{}", safe_name),
                n_plus: rail_node.clone(),
                n_minus: "0".to_string(),
                dc: Some(oa.vcc - oa.voh_drop),
                ac: None,
            });
            augmented.elements.push(Element::Diode {
                name: format!("D_boyle_hi_{}", safe_name),
                n_plus: int_node.clone(),
                n_minus: rail_node,
                model: BOYLE_CATCH_DIODE_MODEL.to_string(),
            });
        }

        if has_lower {
            // Lower catch: (VEE + VOL_DROP) reference node → internal node.
            let rail_node = format!("_boyle_lo_{}", safe_name);
            augmented.elements.push(Element::VoltageSource {
                name: format!("V_boyle_lo_{}", safe_name),
                n_plus: rail_node.clone(),
                n_minus: "0".to_string(),
                dc: Some(oa.vee + oa.vol_drop),
                ac: None,
            });
            augmented.elements.push(Element::Diode {
                name: format!("D_boyle_lo_{}", safe_name),
                n_plus: rail_node,
                n_minus: int_node.clone(),
                model: BOYLE_CATCH_DIODE_MODEL.to_string(),
            });
        }
    }

    augmented
}

fn default_pot_settle_samples() -> usize {
    64
}

fn default_output_nodes() -> Vec<usize> {
    vec![0]
}

fn default_oversampling_factor() -> usize {
    1
}

fn default_output_scales() -> Vec<f64> {
    vec![1.0]
}

/// Sanitize a netlist name to a Rust `SCREAMING_SNAKE_CASE` constant-suffix:
/// uppercase, non-alphanumeric → `_`, leading digit prefixed with `_`.
///
/// Collisions are NOT handled here — the caller must dedupe within its own scope
/// (see [`build_named_constants`]).
fn sanitize_const_suffix(name: &str) -> String {
    let mut out = String::with_capacity(name.len() + 1);
    for c in name.chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c.to_ascii_uppercase());
        } else {
            out.push('_');
        }
    }
    if out.chars().next().map_or(false, |c| c.is_ascii_digit()) {
        out.insert(0, '_');
    }
    if out.is_empty() {
        out.push('_');
    }
    out
}

/// Dedupe sanitized suffixes in declaration order by appending `_2`, `_3`, ….
/// Pure function; easier to unit-test than inlining into `build_named_constants`.
fn dedupe_in_order(pairs: Vec<(String, usize)>) -> Vec<(String, usize)> {
    let mut seen: std::collections::HashMap<String, usize> =
        std::collections::HashMap::new();
    let mut out = Vec::with_capacity(pairs.len());
    for (name, idx) in pairs {
        let count = seen.entry(name.clone()).or_insert(0);
        *count += 1;
        let final_name = if *count == 1 {
            name
        } else {
            format!("{}_{}", name, *count)
        };
        out.push((final_name, idx));
    }
    out
}

/// Build [`NamedConstantsIR`] from an already-built MNA + topology.
///
/// `n_nodes` must be the original circuit node count (excluding augmented
/// VS/VCVS rows and augmented inductor branch currents), matching
/// `Topology::n_nodes`. VS row indices are computed as `n_nodes + ext_idx`.
///
/// Skips the ground entry (`"0"` → 0). Does not currently filter auto-inserted
/// internal nodes because `mna.node_map` only contains user-named nodes plus
/// ground (BJT prime nodes live on `mna.bjt_internal_nodes`, transformer
/// decomposition nodes are allocated past `node_map`).
pub(crate) fn build_named_constants(
    mna: &crate::mna::MnaSystem,
    n_nodes: usize,
) -> NamedConstantsIR {
    let nodes_raw: Vec<(String, usize)> = {
        let mut v: Vec<(String, usize)> = mna
            .node_map
            .iter()
            .filter(|(name, idx)| **idx != 0 && name.as_str() != "0")
            .map(|(name, idx)| (sanitize_const_suffix(name), *idx - 1))
            .collect();
        // Sort by index so emission order is deterministic (HashMap iteration
        // order is randomized per-process).
        v.sort_by_key(|(_, idx)| *idx);
        dedupe_in_order(v)
    };

    let vsources_raw: Vec<(String, usize)> = mna
        .voltage_sources
        .iter()
        .map(|vs| (sanitize_const_suffix(&vs.name), n_nodes + vs.ext_idx))
        .collect();
    let vsources = dedupe_in_order(vsources_raw);

    let pots: Vec<(String, usize)> = {
        let raw: Vec<(String, usize)> = mna
            .pots
            .iter()
            .enumerate()
            .map(|(i, p)| (sanitize_const_suffix(&p.name), i))
            .collect();
        dedupe_in_order(raw)
    };

    NamedConstantsIR {
        nodes: nodes_raw,
        vsources,
        pots,
    }
}

#[cfg(test)]
mod named_constants_tests {
    use super::*;

    #[test]
    fn sanitize_uppercase_alphanumeric() {
        assert_eq!(sanitize_const_suffix("vin"), "VIN");
        assert_eq!(sanitize_const_suffix("Vin"), "VIN");
        assert_eq!(sanitize_const_suffix("out_1"), "OUT_1");
    }

    #[test]
    fn sanitize_nonalphanumeric_to_underscore() {
        assert_eq!(sanitize_const_suffix("n+1"), "N_1");
        assert_eq!(sanitize_const_suffix("a.b"), "A_B");
        assert_eq!(sanitize_const_suffix("x1-x2"), "X1_X2");
    }

    #[test]
    fn sanitize_leading_digit_gets_underscore_prefix() {
        assert_eq!(sanitize_const_suffix("12ax7"), "_12AX7");
        assert_eq!(sanitize_const_suffix("3.3v"), "_3_3V");
    }

    #[test]
    fn sanitize_empty_yields_underscore() {
        assert_eq!(sanitize_const_suffix(""), "_");
    }

    #[test]
    fn dedupe_in_order_suffixes_duplicates() {
        let input = vec![
            ("FOO".to_string(), 1),
            ("BAR".to_string(), 2),
            ("FOO".to_string(), 3),
            ("FOO".to_string(), 4),
            ("BAR".to_string(), 5),
        ];
        let out = dedupe_in_order(input);
        assert_eq!(
            out,
            vec![
                ("FOO".to_string(), 1),
                ("BAR".to_string(), 2),
                ("FOO_2".to_string(), 3),
                ("FOO_3".to_string(), 4),
                ("BAR_2".to_string(), 5),
            ]
        );
    }
}

/// All matrices needed by the generated solver (flattened row-major).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Matrices {
    /// S = A^{-1}, N×N row-major (default for codegen sample rate)
    pub s: Vec<f64>,
    /// A_neg = alpha*C - G, N×N row-major (default for codegen sample rate)
    pub a_neg: Vec<f64>,
    /// Nonlinear kernel K = N_v * S * N_i, M×M row-major (default for codegen sample rate)
    pub k: Vec<f64>,
    /// Voltage extraction N_v, M×N row-major
    pub n_v: Vec<f64>,
    /// Current injection N_i, N×M row-major (kernel storage order)
    pub n_i: Vec<f64>,
    /// Constant RHS contribution from DC sources, length N
    pub rhs_const: Vec<f64>,
    /// Raw conductance matrix G, N×N row-major (sample-rate independent).
    /// Includes input conductance but NOT inductor companion conductances.
    /// For IIR op-amp circuits, Gm is stripped (only Go remains at output nodes).
    #[serde(default)]
    pub g_matrix: Vec<f64>,
    /// Raw capacitance matrix C, N×N row-major (sample-rate independent, at reduced dimension).
    #[serde(default)]
    pub c_matrix: Vec<f64>,
    // --- Nodal solver matrices (only populated when solver_mode == Nodal) ---
    /// A = G + (2/T)*C, N×N row-major (trapezoidal forward matrix)
    #[serde(default)]
    pub a_matrix: Vec<f64>,
    /// A_be = G + (1/T)*C, N×N row-major (backward Euler fallback)
    #[serde(default)]
    pub a_matrix_be: Vec<f64>,
    /// A_neg_be = (1/T)*C, N×N row-major (backward Euler history)
    #[serde(default)]
    pub a_neg_be: Vec<f64>,
    /// RHS constant for backward Euler (DC sources × 1, not × 2)
    #[serde(default)]
    pub rhs_const_be: Vec<f64>,

    // --- Schur complement matrices for nodal solver (S = A^{-1}, computed at codegen time) ---
    /// S_be = A_be^{-1}, N×N row-major (backward Euler, for BE fallback in Schur NR)
    #[serde(default)]
    pub s_be: Vec<f64>,
    /// K_be = N_v * S_be * N_i, M×M row-major (backward Euler kernel for BE fallback)
    #[serde(default)]
    pub k_be: Vec<f64>,
    /// Spectral radius of S * A_neg (trapezoidal feedback operator).
    /// Values > 1 mean the Schur path is unstable. Only computed for nodal path.
    #[serde(default)]
    pub spectral_radius_s_aneg: f64,

    // --- Sub-step matrices (trap at 2× internal rate, for ActiveSetBe sub-stepping) ---
    /// S_sub = (G + (4/T)*C)^{-1}, N×N row-major (trap at 2× rate)
    #[serde(default)]
    pub s_sub: Vec<f64>,
    /// K_sub = N_v * S_sub * N_i, M×M row-major (sub-step kernel)
    #[serde(default)]
    pub k_sub: Vec<f64>,
    /// A_neg_sub = (4/T)*C - G, N×N row-major (sub-step history)
    #[serde(default)]
    pub a_neg_sub: Vec<f64>,
}

/// IIR op-amp dominant pole filter data for codegen.
///
/// Instead of stamping Gm (~4000 S) into the MNA matrix (which causes catastrophic
/// conditioning), the GBW dominant pole is modeled as a per-sample discrete-time
/// IIR filter. The filter computes the VCCS current externally and injects it
/// into the RHS vector, keeping the MNA matrix well-conditioned.
///
/// The continuous-time transfer function is: H(s) = Gm / (1 + s*C_dom/Go)
/// Discretized via bilinear transform: y[n] = a1*y[n-1] + b0*(x[n] + x[n-1])
/// where x = V+ - V-, and the output current injected at the output node is Go*y.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpampIirData {
    /// Transconductance Gm = AOL/ROUT
    pub gm: f64,
    /// Output conductance Go = 1/ROUT
    pub go: f64,
    /// Dominant pole capacitance C_dom = AOL/(2*pi*GBW*ROUT)
    pub c_dom: f64,
    /// 0-indexed non-inverting input node (None if grounded)
    pub np_idx: Option<usize>,
    /// 0-indexed inverting input node (None if grounded)
    pub nm_idx: Option<usize>,
    /// 0-indexed output node
    pub out_idx: usize,
    /// Upper voltage clamp (VCC)
    pub vclamp_hi: f64,
    /// Lower voltage clamp (VEE)
    pub vclamp_lo: f64,
}

/// Op-amp output voltage clamping for code generation.
///
/// When VCC/VEE are finite, the op-amp output node voltage is clamped to
/// [VEE, VCC] after each LU solve or after final voltage reconstruction.
/// This prevents runaway voltages in open-loop or high-gain configurations.
/// Supports asymmetric supply rails (e.g., VCC=9, VEE=0 for single-supply).
///
/// When `sr` is finite the generated code also applies a per-sample
/// voltage-delta clamp on the op-amp output node (slew-rate limiting). The
/// clamp is mathematically equivalent to limiting the Boyle dominant-pole
/// integrator input current to ±`SR * C_dom`, but expressed directly in
/// voltage space as `|Δv_out| ≤ SR * dt`. This is gated at codegen time:
/// when `sr` is infinite, no slew code is emitted at all, so op-amps
/// without `SR=` in their .model produce byte-identical generated code to
/// the pre-slew-rate behaviour.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpampIR {
    /// Output node index (0-indexed, in the N-dimensional system)
    pub n_out_idx: usize,
    /// Non-inverting input node index (0-indexed, `None` if grounded)
    pub n_plus_idx: Option<usize>,
    /// Inverting input node index (0-indexed, `None` if grounded)
    pub n_minus_idx: Option<usize>,
    /// Upper voltage clamp (VCC). INFINITY = no upper clamp.
    pub vclamp_hi: f64,
    /// Lower voltage clamp (VEE). NEG_INFINITY = no lower clamp.
    pub vclamp_lo: f64,
    /// Slew rate [V/s]. INFINITY = no slew limiting (no code emitted).
    /// Parsed from `.model OA(SR=…)` as V/μs, converted to V/s in MNA.
    pub sr: f64,
    /// Excess VCCS transconductance to subtract from the G matrix in sub-step
    /// and main NR `a_sub`/`g_aug` builds. `gm_delta = gm_full - gm_capped`
    /// where `gm_capped = AOL_SUB_MAX / r_out`. Zero when AOL is already low
    /// enough. Prevents the LU back-solve from producing extreme voltages at
    /// op-amp outputs that contaminate neighboring nodes before the post-solve
    /// rail clamp fires.
    pub gm_delta: f64,
}

/// Cap on effective AOL used when building sub-step / main NR matrices.
/// Matches the DC OP `AOL_DC_MAX` constant in `dc_op.rs`. A capped Gm
/// keeps the LU back-solve from producing extreme voltages (400kV+) at
/// op-amp outputs that contaminate neighboring nodes via back-substitution
/// before the post-solve rail clamp fires.
pub(crate) const AOL_SUB_MAX: f64 = 1000.0;

/// Returns `true` when the op-amp's non-inverting input sits on a non-zero
/// DC voltage source (a "DC rail"). Ground is intentionally excluded so
/// soft-clipper topologies (e.g. Klon Centaur, where the clipping op-amp's
/// `n_plus` is ground) are NOT classified as sidechain rectifiers — the
/// `vbias`/`vcc`/`vee`-style supply rails used by precision rectifiers in
/// 4kbuscomp (`U8`/`U9` on `vee12`) qualify.
fn opamp_n_plus_on_dc_rail(oa: &crate::mna::OpampInfo, mna: &crate::mna::MnaSystem) -> bool {
    if oa.n_plus_idx == 0 {
        return false;
    }
    mna.voltage_sources.iter().any(|vs| {
        vs.dc_value != 0.0
            && (vs.n_plus_idx == oa.n_plus_idx || vs.n_minus_idx == oa.n_plus_idx)
    })
}

/// BFS from `start` to `goal` over resistor edges in the netlist. Returns
/// `true` iff a path exists that traverses only resistors. Caps, inductors,
/// devices, and sources do NOT count as edges (they don't form a DC short
/// for the purpose of "diode in feedback" detection — the feedback resistor
/// network is what forms the virtual-ground summing junction).
fn r_only_path_exists(
    start: usize,
    goal: usize,
    netlist: &crate::parser::Netlist,
    mna: &crate::mna::MnaSystem,
) -> bool {
    if start == 0 || goal == 0 {
        return false;
    }
    if start == goal {
        return true;
    }
    let mut visited = vec![false; mna.n + 1];
    let mut queue = vec![start];
    visited[start] = true;
    while let Some(node) = queue.pop() {
        for el in &netlist.elements {
            if let crate::parser::Element::Resistor { n_plus, n_minus, .. } = el {
                let p = mna.node_map.get(n_plus).copied().unwrap_or(0);
                let m = mna.node_map.get(n_minus).copied().unwrap_or(0);
                let neighbor = if p == node {
                    Some(m)
                } else if m == node {
                    Some(p)
                } else {
                    None
                };
                if let Some(nb) = neighbor {
                    if nb == goal {
                        return true;
                    }
                    if nb != 0 && nb < visited.len() && !visited[nb] {
                        visited[nb] = true;
                        queue.push(nb);
                    }
                }
            }
        }
    }
    false
}

/// Rule D' classifier: returns `true` when the op-amp matches the precision-
/// rectifier / comparator topology where capping AOL in the transient solver
/// is safe (the op-amp is "designed to rail" — its loop is open in normal
/// operation, only closed by a diode at the swing extreme).
///
/// Two conditions must both hold:
/// 1. `n_plus` is on a non-zero DC rail (auto-detected via VS connectivity).
///    Ground does not count, so soft-clipper topologies that bias `n_plus`
///    to ground (Klon) are excluded.
/// 2. At least one diode connects the op-amp output to the inverting input,
///    optionally through a pure-resistor path (covers full-wave summing
///    rectifiers like `U9`, where the diode goes through the summing R).
fn opamp_is_sidechain_rectifier(
    oa: &crate::mna::OpampInfo,
    netlist: &crate::parser::Netlist,
    mna: &crate::mna::MnaSystem,
) -> bool {
    if !opamp_n_plus_on_dc_rail(oa, mna) {
        return false;
    }
    if oa.n_minus_idx == 0 || oa.n_out_idx == 0 {
        return false;
    }
    for dev in &mna.nonlinear_devices {
        if dev.device_type != crate::mna::NonlinearDeviceType::Diode {
            continue;
        }
        if dev.node_indices.len() < 2 {
            continue;
        }
        let (a, k) = (dev.node_indices[0], dev.node_indices[1]);
        let touches_out = a == oa.n_out_idx || k == oa.n_out_idx;
        if !touches_out {
            continue;
        }
        let other = if a == oa.n_out_idx { k } else { a };
        if other == oa.n_minus_idx {
            return true;
        }
        if r_only_path_exists(other, oa.n_minus_idx, netlist, mna) {
            return true;
        }
    }
    false
}

/// Effective AOL cap for an op-amp. Priority:
/// 1. User override via `.model OA(AOL_TRANSIENT_CAP=N)` (any finite value).
/// 2. Auto-detect via Rule D' → `AOL_SUB_MAX` for sidechain rectifiers.
/// 3. No cap (`f64::INFINITY`) for everything else.
fn effective_aol_cap(
    oa: &crate::mna::OpampInfo,
    netlist: &crate::parser::Netlist,
    mna: &crate::mna::MnaSystem,
) -> f64 {
    if oa.aol_transient_cap.is_finite() {
        return oa.aol_transient_cap;
    }
    if opamp_is_sidechain_rectifier(oa, netlist, mna) {
        return AOL_SUB_MAX;
    }
    f64::INFINITY
}

/// Build an `OpampIR` from the MNA `OpampInfo`, computing the Gm delta
/// for sub-step matrix corrections.
fn opamp_ir_from_info(
    oa: &crate::mna::OpampInfo,
    netlist: &crate::parser::Netlist,
    mna: &crate::mna::MnaSystem,
) -> OpampIR {
    let aol_cap = effective_aol_cap(oa, netlist, mna);
    let aol_eff = oa.aol.min(aol_cap);
    let gm_full = oa.aol / oa.r_out;
    let gm_capped = aol_eff / oa.r_out;
    let gm_delta = (gm_full - gm_capped).max(0.0);
    OpampIR {
        n_out_idx: oa.n_out_idx - 1,
        n_plus_idx: if oa.n_plus_idx > 0 { Some(oa.n_plus_idx - 1) } else { None },
        n_minus_idx: if oa.n_minus_idx > 0 { Some(oa.n_minus_idx - 1) } else { None },
        vclamp_hi: oa.vcc,
        vclamp_lo: oa.vee,
        sr: oa.sr,
        gm_delta,
    }
}

/// Potentiometer parameters for code generation (Sherman-Morrison precomputed data).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PotentiometerIR {
    /// S * u where u is the pot's node difference vector (N-vector)
    pub su: Vec<f64>,
    /// u^T * S * u (scalar for SM denominator)
    pub usu: f64,
    /// Nominal conductance 1/R_nom
    pub g_nominal: f64,
    /// N_v * su (M-vector, for K correction in NR loop)
    pub nv_su: Vec<f64>,
    /// su^T * N_i = (S*u)^T * N_i (M-vector, for correction to K and S*N_i products)
    pub u_ni: Vec<f64>,
    /// Positive terminal node index (0 = ground, 1-indexed)
    pub node_p: usize,
    /// Negative terminal node index (0 = ground, 1-indexed)
    pub node_q: usize,
    /// Minimum resistance (ohms)
    pub min_resistance: f64,
    /// Maximum resistance (ohms)
    pub max_resistance: f64,
    /// True if one terminal is grounded
    pub grounded: bool,
    /// If Some, this entry was declared via `.runtime R` rather than `.pot`.
    /// The contained string is the Rust identifier the emitter uses for
    /// `set_runtime_R_<field>` (no DC-OP warm re-init) and the `<field>()`
    /// getter. Plugin template skips nih-plug knob emission for these.
    #[serde(default)]
    pub runtime_field: Option<String>,
}

/// Wiper potentiometer group for code generation.
///
/// Links two `PotentiometerIR` entries as complementary legs of a 3-terminal pot.
/// A single position parameter (0.0–1.0) controls both resistances.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiperGroupIR {
    /// Index into `CircuitIR::pots` for the CW (top→wiper) leg
    pub cw_pot_index: usize,
    /// Index into `CircuitIR::pots` for the CCW (wiper→bottom) leg
    pub ccw_pot_index: usize,
    /// Total resistance (R_cw + R_ccw = total)
    pub total_resistance: f64,
    /// Default wiper position (0.0–1.0)
    pub default_position: f64,
    /// Optional human-readable label
    pub label: Option<String>,
}

/// Gang group — links multiple pots/wipers under a single 0-1 parameter.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GangGroupIR {
    /// Human-readable label
    pub label: String,
    /// Pot members: pot_index, min_resistance, max_resistance, inverted
    pub pot_members: Vec<GangPotMemberIR>,
    /// Wiper members: wiper_group_index, total_resistance, inverted
    pub wiper_members: Vec<GangWiperMemberIR>,
    /// Default position (0.0–1.0)
    pub default_position: f64,
}

/// A pot member of a gang group.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GangPotMemberIR {
    /// Index into `CircuitIR::pots`
    pub pot_index: usize,
    /// Minimum resistance (ohms)
    pub min_resistance: f64,
    /// Maximum resistance (ohms)
    pub max_resistance: f64,
    /// If true, position mapping is inverted
    pub inverted: bool,
}

/// A wiper member of a gang group.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GangWiperMemberIR {
    /// Index into `CircuitIR::wiper_groups`
    pub wiper_group_index: usize,
    /// Total resistance of the wiper pot
    pub total_resistance: f64,
    /// If true, position mapping is inverted
    pub inverted: bool,
}

/// Component within a switch directive for code generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchComponentIR {
    pub name: String,
    /// 'R', 'C', or 'L'
    pub component_type: char,
    /// Node index (1-indexed, 0 = ground)
    pub node_p: usize,
    /// Node index (1-indexed, 0 = ground)
    pub node_q: usize,
    /// Nominal value from netlist
    pub nominal_value: f64,
    /// For 'L' components: index into the inductors vec (for g_eq recomputation)
    pub inductor_index: Option<usize>,
    /// For 'L' components in augmented MNA: row index in augmented C matrix
    /// where the inductance value lives (c_work[k][k] = L). None for DK path
    /// or non-inductor components.
    #[serde(default)]
    pub augmented_row: Option<usize>,
    /// For 'L' components in a coupled inductor pair (DK path): index into
    /// `coupled_inductors` vec. None for uncoupled inductors or non-inductors.
    #[serde(default)]
    pub coupled_inductor_index: Option<usize>,
    /// Which winding (1 = L1, 2 = L2) within the coupled pair.
    /// Only meaningful when `coupled_inductor_index` is Some.
    #[serde(default)]
    pub coupled_winding: Option<u8>,
}

/// Mutual inductance entry that must be recomputed when a switch changes
/// an inductor value in the augmented MNA C matrix.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchMutualEntry {
    /// Augmented row index of first coupled inductor
    pub row_a: usize,
    /// Augmented row index of second coupled inductor
    pub row_b: usize,
    /// Coupling coefficient k from the K directive
    pub coupling: f64,
}

/// Switch parameters for code generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchIR {
    /// Switch index (0-based)
    pub index: usize,
    /// Components controlled by this switch
    pub components: Vec<SwitchComponentIR>,
    /// Position values: positions[pos][comp] = value
    pub positions: Vec<Vec<f64>>,
    /// Number of positions
    pub num_positions: usize,
    /// Off-diagonal mutual inductance entries that depend on inductor values
    /// in this switch. After diagonal L updates, each entry is recomputed:
    /// `C[a][b] = C[b][a] = k * sqrt(C[a][a] * C[b][b])`.
    #[serde(default)]
    pub mutual_entries: Vec<SwitchMutualEntry>,
}

/// Inductor parameters for code generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InductorIR {
    pub name: String,
    /// Node index (1-indexed, 0=ground)
    pub node_i: usize,
    /// Node index (1-indexed, 0=ground)
    pub node_j: usize,
    /// Equivalent conductance T/(2L) (at the codegen sample rate)
    pub g_eq: f64,
    /// Raw inductance value in henries (for sample rate recomputation)
    #[serde(default)]
    pub inductance: f64,
}

/// Saturating (iron-core) inductor for per-sample L(I) update.
///
/// The effective inductance follows: L_eff(I) = l0 / cosh²(I / isat).
/// At each sample, the previous branch current `v_prev[aug_row]` is used to
/// compute L_eff, then a Sherman-Morrison rank-1 update corrects S, K, A_neg.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SaturatingInductorIR {
    pub name: String,
    /// Nominal inductance (henries)
    pub l0: f64,
    /// Saturation current (amps) — L drops to L0/4 at I = 1.32*isat
    pub isat: f64,
    /// Row index in the augmented system (C[aug_row][aug_row] = L)
    pub aug_row: usize,
    /// Index into the uncoupled/coupled/transformer inductor arrays for
    /// identifying which inductor this is (for naming constants).
    pub inductor_index: usize,
}

/// Saturating coupled inductor pair for per-sample L(I) update.
///
/// Two windings sharing a core. When either saturates, L_eff drops and
/// M_eff = κ * sqrt(L1_eff * L2_eff) changes too. The 2×2 delta block
/// is decomposed via eigendecomposition into 2 rank-1 SM updates to S.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SaturatingCoupledInductorIR {
    pub name: String,
    pub l1_name: String,
    pub l2_name: String,
    pub l1_l0: f64,
    pub l2_l0: f64,
    pub l1_isat: f64,
    pub l2_isat: f64,
    pub coupling: f64,
    /// Augmented row index for winding 1
    pub k1: usize,
    /// Augmented row index for winding 2
    pub k2: usize,
}

/// Saturating W-winding transformer group for per-sample L(I) update.
///
/// Each winding saturates independently based on its own branch current.
/// The W×W delta block is decomposed into W diagonal + W*(W-1) off-diagonal
/// elementary rank-1 SM updates (W² total).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SaturatingTransformerGroupIR {
    pub name: String,
    pub num_windings: usize,
    pub winding_names: Vec<String>,
    /// Nominal inductance per winding
    pub l0s: Vec<f64>,
    /// Saturation current per winding (1e6 = effectively linear)
    pub isats: Vec<f64>,
    /// Coupling coefficients: flat W×W row-major (κ[i][j])
    pub coupling_flat: Vec<f64>,
    /// Augmented row index per winding
    pub aug_rows: Vec<usize>,
}

/// Coupled inductor pair parameters for code generation (transformer).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoupledInductorIR {
    pub name: String,
    pub l1_name: String,
    pub l2_name: String,
    pub l1_node_i: usize,
    pub l1_node_j: usize,
    pub l2_node_i: usize,
    pub l2_node_j: usize,
    pub l1_inductance: f64,
    pub l2_inductance: f64,
    pub coupling: f64,
    /// Self-conductance for L1: (T/2) * L2 / det
    pub g_self_1: f64,
    /// Self-conductance for L2: (T/2) * L1 / det
    pub g_self_2: f64,
    /// Mutual conductance: -(T/2) * M / det
    pub g_mutual: f64,
}

/// Multi-winding transformer group for codegen.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransformerGroupIR {
    pub name: String,
    pub num_windings: usize,
    pub winding_names: Vec<String>,
    pub winding_node_i: Vec<usize>,
    pub winding_node_j: Vec<usize>,
    pub inductances: Vec<f64>,
    /// NxN coupling matrix, flat row-major
    pub coupling_flat: Vec<f64>,
    /// NxN admittance matrix Y = (T/2) * inv(L), flat row-major (at codegen sample rate)
    pub y_matrix: Vec<f64>,
}

// Re-export device types from the shared module (always compiled, no tera dependency).
pub use crate::device_types::{
    BjtParams, DeviceParams, DeviceSlot, DeviceType, DiodeParams, JfetParams, MosfetParams,
    ScreenForm, TubeKind, TubeParams, VcaParams,
};

/// Sparsity pattern for a single matrix.
///
/// Stores per-row lists of nonzero column indices, enabling emitters to
/// skip structural zeros without ad-hoc `!= 0.0` checks. Entries with
/// `|x| < SPARSITY_THRESHOLD` are treated as structural zeros.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MatrixSparsity {
    pub rows: usize,
    pub cols: usize,
    /// Total number of nonzero entries
    pub nnz: usize,
    /// For each row, sorted list of column indices with nonzero entries
    pub nz_by_row: Vec<Vec<usize>>,
}

/// Johnson-Nyquist (thermal) noise source stamped at one fixed resistor.
///
/// Emitted as a Norton current source in the MNA RHS: for sample rate `fs`
/// and temperature `T`, the per-sample current is
/// `sqrt(4·k_B·T·fs / resistance) · N(0,1)` injected at `node_i` and
/// extracted at `node_j`. See `docs/aidocs/NOISE.md`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThermalNoiseSource {
    /// Resistor component name (for debug / future per-source overrides).
    pub name: String,
    /// 1-indexed positive node (0 = ground).
    pub node_i: usize,
    /// 1-indexed negative node (0 = ground).
    pub node_j: usize,
    /// Resistance in ohms at the default operating point.
    ///
    /// For dynamic sources (`pot_slot` or `switch_slot` is `Some(_)`) this
    /// is the codegen-time default; the runtime value lives in
    /// `state.noise_thermal_sqrt_inv_r` and is refreshed by the matching
    /// `set_pot_N` / `set_runtime_R_<field>` / `set_switch_N` setter.
    pub resistance: f64,
    /// `Some(i)` when this R is a `.pot` / `.runtime R` / `.wiper` member
    /// backed by `mna.pots[i]`. The emitter injects a coefficient update
    /// into the corresponding pot / runtime-R setter so the per-sample
    /// `sqrt(1/R)` tracks the live resistance. `None` for fixed resistors
    /// and switch-R components.
    #[serde(default)]
    pub pot_slot: Option<usize>,
    /// `Some((switch_idx, comp_idx))` when this R is an R-type component
    /// under a `.switch` directive. The emitter injects a coefficient
    /// update into the corresponding `set_switch_N(position)` setter so
    /// the per-sample `sqrt(1/R)` tracks the discrete R value selected by
    /// the current position. `None` for fixed and pot-backed resistors.
    #[serde(default)]
    pub switch_slot: Option<(usize, usize)>,
}

/// Shot (junction) noise source stamped at one forward-biased PN junction.
///
/// Emitted as a Norton current source in the MNA RHS: for sample rate `fs`
/// and instantaneous bias current `|I(t)|` (read from `state.i_nl_prev`),
/// the per-sample current is `sqrt(4·q·|I|·fs) · N(0,1)` injected at
/// `node_i` and extracted at `node_j`. The `4·q·fs` matches thermal's
/// trap-MNA calibration (2× the textbook one-sided `2·q·|I|` PSD). See
/// `docs/aidocs/NOISE.md` "Constant derivation" for why.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShotNoiseSource {
    /// Device name (for debug / future per-device overrides).
    pub name: String,
    /// Index into `state.i_nl_prev` providing the instantaneous bias current.
    pub slot_idx: usize,
    /// 1-indexed positive injection node (0 = ground).
    pub node_i: usize,
    /// 1-indexed negative injection node (0 = ground).
    pub node_j: usize,
}

/// Flicker (1/f) noise source stamped at one current-carrying junction.
///
/// Emitted as a Norton current source whose amplitude is shaped by a Paul
/// Kellett 7-pole pink filter (≈ -3 dB/oct slope, ±0.5 dB over 10 Hz-20 kHz).
/// For sample rate `fs`, instantaneous bias current `|I(t)|` (read from
/// `state.i_nl_prev`), device-specific `KF` and `AF` model params, the
/// per-sample injected current is
/// `kellett(sqrt(4·KF·fs) · |I|^(AF/2) · N(0,1))`.
/// Same `4·…·fs` 2× trap-MNA compensation as thermal and shot; the Kellett
/// cascade has ~unity RMS power gain so the white-input PSD shapes into
/// 1/f at the output. See `docs/aidocs/NOISE.md`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlickerNoiseSource {
    /// Device name (and port suffix where relevant, e.g. "Q1.Ic").
    pub name: String,
    /// Index into `state.i_nl_prev` providing the instantaneous bias current.
    pub slot_idx: usize,
    /// 1-indexed positive injection node (0 = ground).
    pub node_i: usize,
    /// 1-indexed negative injection node (0 = ground).
    pub node_j: usize,
    /// Flicker coefficient `KF` from the device's `.model` (0 → source is
    /// filtered out before this struct is built; every emitted source has
    /// `kf > 0`).
    pub kf: f64,
    /// Flicker exponent `AF` from the device's `.model`. ngspice default 1.0.
    pub af: f64,
}

/// Noise configuration baked into the generated code.
///
/// Built from [`crate::codegen::NoiseMode`] + a scan of `netlist.elements`.
/// Emitters gate all noise-related emission on `mode != NoiseMode::Off` and
/// iterate the per-phase source vectors.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct NoiseIR {
    /// Compile-time mode. `NoiseMode::Off` → zero emission, zero runtime cost.
    #[serde(default)]
    pub mode: crate::codegen::NoiseMode,
    /// Master seed for deterministic noise. `0` → system entropy at `default()`.
    #[serde(default)]
    pub master_seed: u64,
    /// Johnson-Nyquist noise sources (Phase 1).
    #[serde(default)]
    pub thermal_sources: Vec<ThermalNoiseSource>,
    /// Shot (junction) noise sources (Phase 2). Populated only when
    /// `mode.includes_shot()`.
    #[serde(default)]
    pub shot_sources: Vec<ShotNoiseSource>,
    /// Flicker (1/f) noise sources (Phase 3). Populated only when
    /// `mode.includes_full()` AND the device's `.model` supplies a positive
    /// `KF`. Zero-`KF` devices produce no entry — zero codegen overhead.
    #[serde(default)]
    pub flicker_sources: Vec<FlickerNoiseSource>,
}

/// Collect Johnson-Nyquist thermal noise sources from the netlist.
///
/// Includes every `Element::Resistor` that maps to a circuit node, with
/// three source kinds (all indexed uniformly at
/// `state.noise_thermal_sqrt_inv_r[k]`):
///
/// - **Static** (both `pot_slot` and `switch_slot` are `None`) —
///   fixed-value resistor. The per-sample coefficient `sqrt(1/R)` is baked
///   at codegen time.
/// - **Pot-backed** (`pot_slot = Some(i)`) — `.pot` / `.wiper` /
///   `.runtime R` member backed by `mna.pots[i]`. The coefficient is
///   state-backed and refreshed inside `set_pot_N` /
///   `set_runtime_R_<field>`.
/// - **Switch-backed** (`switch_slot = Some((sw, comp))`) — R-type
///   component under a `.switch` directive. The coefficient is refreshed
///   inside `set_switch_N(position)` using the position-indexed value
///   array emitted by each solver's constants block.
///
/// Node indices are MNA 1-indexed (0 = ground) via `mna.node_map`. Resistors
/// whose terminals fail to resolve (would collapse to ground–ground) are
/// filtered out — they inject nothing useful.
pub fn collect_thermal_noise_sources(
    netlist: &Netlist,
    mna: &MnaSystem,
) -> Vec<ThermalNoiseSource> {
    use std::collections::HashMap;

    // Upper-cased name → pot index in mna.pots. Covers .pot, .runtime R,
    // and wiper-derived pot entries (wipers decompose into cw/ccw pots
    // that both live in mna.pots).
    let mut pot_slot: HashMap<String, usize> = HashMap::new();
    for (i, p) in mna.pots.iter().enumerate() {
        pot_slot.insert(p.name.to_ascii_uppercase(), i);
    }

    // Upper-cased name → (switch_idx, comp_idx) for R-type switch
    // components. C and L components don't generate thermal noise and are
    // skipped here — they also aren't Element::Resistor at the netlist
    // level, so the loop below never sees them.
    let mut switch_slot_map: HashMap<String, (usize, usize)> = HashMap::new();
    for (sw_idx, sw) in mna.switches.iter().enumerate() {
        for (ci, c) in sw.components.iter().enumerate() {
            if c.component_type == 'R' {
                switch_slot_map.insert(c.name.to_ascii_uppercase(), (sw_idx, ci));
            }
        }
    }

    let mut sources = Vec::new();
    for el in &netlist.elements {
        if let Element::Resistor {
            name,
            n_plus,
            n_minus,
            value,
        } = el
        {
            let upper = name.to_ascii_uppercase();
            if !value.is_finite() || *value <= 0.0 {
                continue;
            }
            let node_i = mna.node_map.get(n_plus).copied().unwrap_or(0);
            let node_j = mna.node_map.get(n_minus).copied().unwrap_or(0);
            if node_i == 0 && node_j == 0 {
                continue;
            }
            let pot_idx = pot_slot.get(&upper).copied();
            let sw_idx = switch_slot_map.get(&upper).copied();
            // A single R cannot be under both `.pot` and `.switch` in the
            // parser (those directives validate mutually exclusive
            // membership), so at most one of pot_idx / sw_idx is Some.
            // Resolve initial resistance from the backing structure:
            //   - pot: use `1/g_nominal` (MNA-canonical value)
            //   - switch: use position-0 value (matches Default::default)
            //   - neither: use the netlist literal value
            let resistance = if let Some(i) = pot_idx {
                let g = mna.pots[i].g_nominal;
                if g > 0.0 && g.is_finite() { 1.0 / g } else { *value }
            } else if let Some((sw, comp)) = sw_idx {
                let positions = &mna.switches[sw].positions;
                let r0 = positions.first().and_then(|row| row.get(comp)).copied();
                match r0 {
                    Some(v) if v.is_finite() && v > 0.0 => v,
                    _ => *value,
                }
            } else {
                *value
            };
            sources.push(ThermalNoiseSource {
                name: name.clone(),
                node_i,
                node_j,
                resistance,
                pot_slot: pot_idx,
                switch_slot: sw_idx,
            });
        }
    }
    sources
}

/// Collect shot-noise sources from the nonlinear device list.
///
/// Emits one `ShotNoiseSource` per current-carrying junction, with the
/// `slot_idx` pointing at the relevant entry of `state.i_nl_prev`:
///
/// - Diode: 1 source at (anode, cathode), slot = start_idx (Id).
/// - BJT (2D): 2 sources — (collector, emitter) at slot=start_idx (Ic),
///   (base, emitter) at slot=start_idx+1 (Ib).
/// - BJT forward-active (1D): 1 source at (collector, emitter),
///   slot=start_idx. The Ib shot is folded in via the BF stamping.
/// - JFET / MOSFET: 1 source at (drain, source), slot=start_idx (Id).
///   Gate shot is ≈ 0 for MOS and deferred for JFET reverse-bias leakage.
/// - Tube (triode or pentode): 1 source at (plate, cathode),
///   slot=start_idx (Ip). Pentode partition noise is Step 7 work.
/// - VCA: skipped — shot at a control port is not physically meaningful.
///
/// The returned sources stamp Norton currents at the **external** device
/// nodes even when the nodal path has internal-node expansion active.
/// In that case the parasitic series R shapes the injection from outside,
/// not from inside the junction — a small-magnitude approximation that is
/// inaudible at audio rates and avoids threading internal-node indices
/// through the collector. Revisit if BJT RB/RE become tonally relevant
/// for shot content.
pub fn collect_shot_noise_sources(mna: &MnaSystem) -> Vec<ShotNoiseSource> {
    use crate::mna::NonlinearDeviceType;
    let mut sources = Vec::new();
    for dev in &mna.nonlinear_devices {
        let nodes = &dev.node_indices;
        match dev.device_type {
            NonlinearDeviceType::Diode => {
                // node_indices = [anode, cathode]
                if nodes.len() >= 2 {
                    sources.push(ShotNoiseSource {
                        name: dev.name.clone(),
                        slot_idx: dev.start_idx,
                        node_i: nodes[0],
                        node_j: nodes[1],
                    });
                }
            }
            NonlinearDeviceType::Bjt => {
                // node_indices = [collector, base, emitter]
                if nodes.len() >= 3 {
                    let (c, b, e) = (nodes[0], nodes[1], nodes[2]);
                    // Ic shot: C↔E
                    sources.push(ShotNoiseSource {
                        name: format!("{}.Ic", dev.name),
                        slot_idx: dev.start_idx,
                        node_i: c,
                        node_j: e,
                    });
                    // Ib shot: B↔E
                    sources.push(ShotNoiseSource {
                        name: format!("{}.Ib", dev.name),
                        slot_idx: dev.start_idx + 1,
                        node_i: b,
                        node_j: e,
                    });
                }
            }
            NonlinearDeviceType::BjtForwardActive => {
                // node_indices = [collector, base, emitter] ; Ib folded in,
                // so only Ic gets a shot source.
                if nodes.len() >= 3 {
                    sources.push(ShotNoiseSource {
                        name: format!("{}.Ic", dev.name),
                        slot_idx: dev.start_idx,
                        node_i: nodes[0],
                        node_j: nodes[2],
                    });
                }
            }
            NonlinearDeviceType::Jfet | NonlinearDeviceType::Mosfet => {
                // node_indices = [drain, gate, source, (bulk)]
                if nodes.len() >= 3 {
                    sources.push(ShotNoiseSource {
                        name: format!("{}.Id", dev.name),
                        slot_idx: dev.start_idx,
                        node_i: nodes[0],
                        node_j: nodes[2],
                    });
                }
            }
            NonlinearDeviceType::Tube => {
                // Triode node_indices = [plate, grid, cathode];
                // pentode node_indices = [plate, grid, screen, cathode].
                // Ip stamps plate-to-cathode in both cases.
                if nodes.len() >= 3 {
                    let plate = nodes[0];
                    let cathode = *nodes.last().unwrap();
                    sources.push(ShotNoiseSource {
                        name: format!("{}.Ip", dev.name),
                        slot_idx: dev.start_idx,
                        node_i: plate,
                        node_j: cathode,
                    });
                }
            }
            NonlinearDeviceType::Vca => {
                // Skipped: shot at a VCA control port is not physically
                // meaningful; the VCA sig port is linear in the small-
                // signal sense and doesn't have a junction current.
            }
        }
    }
    sources
}

/// Collect flicker (1/f) noise sources from the nonlinear device list.
///
/// Per-device port layout mirrors `collect_shot_noise_sources` — Diode 1,
/// BJT 2 (Ic + Ib), BjtForwardActive 1 (Ic only), JFET/MOSFET 1 (Id), Tube 1
/// (Ip), VCA skipped — but each device is filtered on `KF > 0` so circuits
/// with zero-`KF` (the default) models produce no flicker entries and incur
/// no per-sample cost. `AF` defaults to 1.0 (ngspice convention).
///
/// Model lookup walks `netlist.elements` once to map device name → model
/// name, then `netlist.models` for the KF/AF pair. The cost is O(N_dev +
/// N_elem) per codegen invocation; negligible compared to matrix assembly.
pub fn collect_flicker_noise_sources(
    netlist: &Netlist,
    mna: &MnaSystem,
) -> Vec<FlickerNoiseSource> {
    use crate::mna::NonlinearDeviceType;
    use std::collections::HashMap;

    // Build device-name → model-name map once.
    let mut model_for: HashMap<String, String> = HashMap::new();
    for el in &netlist.elements {
        match el {
            Element::Diode { name, model, .. }
            | Element::Bjt { name, model, .. }
            | Element::Jfet { name, model, .. }
            | Element::Mosfet { name, model, .. }
            | Element::Triode { name, model, .. }
            | Element::Pentode { name, model, .. } => {
                model_for.insert(name.to_ascii_uppercase(), model.clone());
            }
            _ => {}
        }
    }

    // Resolve (KF, AF) for a given device by its name, returning `None` when
    // the device is not modeled (shouldn't happen — defensive) or KF <= 0.
    let kf_af = |dev_name: &str| -> Option<(f64, f64)> {
        let model = model_for.get(&dev_name.to_ascii_uppercase())?;
        let m = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))?;
        let mut kf = 0.0_f64;
        let mut af = 1.0_f64;
        for (k, v) in &m.params {
            match k.to_ascii_uppercase().as_str() {
                "KF" => kf = *v,
                "AF" => af = *v,
                _ => {}
            }
        }
        if kf > 0.0 && kf.is_finite() && af > 0.0 && af.is_finite() {
            Some((kf, af))
        } else {
            None
        }
    };

    let mut sources = Vec::new();
    for dev in &mna.nonlinear_devices {
        let Some((kf, af)) = kf_af(&dev.name) else {
            continue;
        };
        let nodes = &dev.node_indices;
        match dev.device_type {
            NonlinearDeviceType::Diode => {
                if nodes.len() >= 2 {
                    sources.push(FlickerNoiseSource {
                        name: dev.name.clone(),
                        slot_idx: dev.start_idx,
                        node_i: nodes[0],
                        node_j: nodes[1],
                        kf,
                        af,
                    });
                }
            }
            NonlinearDeviceType::Bjt => {
                if nodes.len() >= 3 {
                    let (c, b, e) = (nodes[0], nodes[1], nodes[2]);
                    sources.push(FlickerNoiseSource {
                        name: format!("{}.Ic", dev.name),
                        slot_idx: dev.start_idx,
                        node_i: c,
                        node_j: e,
                        kf,
                        af,
                    });
                    sources.push(FlickerNoiseSource {
                        name: format!("{}.Ib", dev.name),
                        slot_idx: dev.start_idx + 1,
                        node_i: b,
                        node_j: e,
                        kf,
                        af,
                    });
                }
            }
            NonlinearDeviceType::BjtForwardActive => {
                if nodes.len() >= 3 {
                    sources.push(FlickerNoiseSource {
                        name: format!("{}.Ic", dev.name),
                        slot_idx: dev.start_idx,
                        node_i: nodes[0],
                        node_j: nodes[2],
                        kf,
                        af,
                    });
                }
            }
            NonlinearDeviceType::Jfet | NonlinearDeviceType::Mosfet => {
                if nodes.len() >= 3 {
                    sources.push(FlickerNoiseSource {
                        name: format!("{}.Id", dev.name),
                        slot_idx: dev.start_idx,
                        node_i: nodes[0],
                        node_j: nodes[2],
                        kf,
                        af,
                    });
                }
            }
            NonlinearDeviceType::Tube => {
                if nodes.len() >= 3 {
                    let plate = nodes[0];
                    let cathode = *nodes.last().unwrap();
                    sources.push(FlickerNoiseSource {
                        name: format!("{}.Ip", dev.name),
                        slot_idx: dev.start_idx,
                        node_i: plate,
                        node_j: cathode,
                        kf,
                        af,
                    });
                }
            }
            NonlinearDeviceType::Vca => {
                // Skipped: no junction current; shot/flicker not defined.
            }
        }
    }
    sources
}

/// Pre-analyzed sparsity information for all compile-time matrices.
///
/// Populated by `analyze_sparsity()` at the end of `from_kernel()`.
/// Emitters use this to generate code that skips structural zeros.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SparseInfo {
    /// A_neg matrix (N×N) — history matrix
    pub a_neg: MatrixSparsity,
    /// N_v matrix (M×N) — voltage extraction
    pub n_v: MatrixSparsity,
    /// N_i matrix (N×M) — current injection
    pub n_i: MatrixSparsity,
    /// K matrix (M×M) — nonlinear kernel
    pub k: MatrixSparsity,
    /// Sparse LU elimination schedule for G_aug (full LU path only)
    pub lu: Option<LuSparsity>,
}

// LuSparsity, LuOp, and SPARSITY_THRESHOLD are defined in crate::lu
// and re-imported at the top of this file.

/// Analyze sparsity pattern of a flattened row-major matrix.
fn analyze_matrix_sparsity(data: &[f64], rows: usize, cols: usize) -> MatrixSparsity {
    let mut nnz = 0;
    let mut nz_by_row = Vec::with_capacity(rows);
    for i in 0..rows {
        let mut row_nz = Vec::new();
        for j in 0..cols {
            if data[i * cols + j].abs() >= SPARSITY_THRESHOLD {
                row_nz.push(j);
                nnz += 1;
            }
        }
        nz_by_row.push(row_nz);
    }
    MatrixSparsity {
        rows,
        cols,
        nnz,
        nz_by_row,
    }
}

// Sparsity analysis functions (compute_g_aug_pattern, amd_ordering,
// find_row_swaps, symbolic_lu) are defined in crate::lu and called
// via lu::compute_g_aug_pattern(...) etc. at the call sites below.

/// Stamp mutual conductance between two 2-terminal elements into a flat row-major matrix.
/// Node indices are 1-indexed; 0 means ground.
fn stamp_flat_mutual(mat: &mut [f64], n: usize, a: usize, b: usize, c: usize, d: usize, g: f64) {
    if a > 0 && c > 0 {
        mat[(a - 1) * n + (c - 1)] += g;
    }
    if b > 0 && d > 0 {
        mat[(b - 1) * n + (d - 1)] += g;
    }
    if a > 0 && d > 0 {
        mat[(a - 1) * n + (d - 1)] -= g;
    }
    if b > 0 && c > 0 {
        mat[(b - 1) * n + (c - 1)] -= g;
    }
}

/// Stamp a conductance between two nodes into a flat row-major matrix.
/// Node indices are 1-indexed; 0 means ground.
fn stamp_flat_conductance(mat: &mut [f64], n: usize, node_i: usize, node_j: usize, g: f64) {
    match (node_i > 0, node_j > 0) {
        (true, true) => {
            let i = node_i - 1;
            let j = node_j - 1;
            mat[i * n + i] += g;
            mat[j * n + j] += g;
            mat[i * n + j] -= g;
            mat[j * n + i] -= g;
        }
        (true, false) => {
            mat[(node_i - 1) * n + (node_i - 1)] += g;
        }
        (false, true) => {
            mat[(node_j - 1) * n + (node_j - 1)] += g;
        }
        (false, false) => {}
    }
}

/// Invert a flat row-major N×N matrix using Gaussian elimination with partial pivoting.
///
/// Returns `CodegenError::InvalidConfig` if the matrix is singular.
fn invert_flat_matrix(a: &[f64], n: usize) -> Result<Vec<f64>, CodegenError> {
    // Build augmented [A | I]
    let mut aug = vec![0.0f64; n * 2 * n];
    for i in 0..n {
        for j in 0..n {
            aug[i * 2 * n + j] = a[i * n + j];
        }
        aug[i * 2 * n + n + i] = 1.0;
    }

    let w = 2 * n;
    for col in 0..n {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = aug[col * w + col].abs();
        for row in (col + 1)..n {
            let v = aug[row * w + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            return Err(CodegenError::InvalidConfig(format!(
                "Matrix is singular (pivot {:.2e} at row {}) — check for floating nodes or missing ground path",
                max_val, col
            )));
        }
        if max_row != col {
            for j in 0..w {
                aug.swap(col * w + j, max_row * w + j);
            }
        }
        let pivot = aug[col * w + col];
        for row in (col + 1)..n {
            let factor = aug[row * w + col] / pivot;
            for j in col..w {
                aug[row * w + j] -= factor * aug[col * w + j];
            }
        }
    }

    // Back substitution
    for col in (0..n).rev() {
        let pivot = aug[col * w + col];
        if pivot.abs() < 1e-30 {
            return Err(CodegenError::InvalidConfig(format!(
                "Matrix is singular (pivot {:.2e} at row {}) — check for floating nodes or missing ground path",
                pivot.abs(),
                col
            )));
        }
        for j in 0..w {
            aug[col * w + j] /= pivot;
        }
        for row in 0..col {
            let factor = aug[row * w + col];
            for j in 0..w {
                aug[row * w + j] -= factor * aug[col * w + j];
            }
        }
    }

    // Extract result
    let mut result = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..n {
            result[i * n + j] = aug[i * w + n + j];
        }
    }
    Ok(result)
}

/// Compute K = N_v * S * N_i from flat row-major matrices.
///
/// N_v is M×N, S is N×N, N_i is N×M (all flat row-major).
fn compute_k_from_s(s: &[f64], n_v: &[f64], n_i: &[f64], n: usize, m: usize) -> Vec<f64> {
    // First compute S * N_i → S_NI (N×M)
    let mut s_ni = vec![0.0f64; n * m];
    for i in 0..n {
        for j in 0..m {
            let mut sum = 0.0;
            for k in 0..n {
                sum += s[i * n + k] * n_i[k * m + j];
            }
            s_ni[i * m + j] = sum;
        }
    }
    // Then K = N_v * S_NI → K (M×M)
    let mut k = vec![0.0f64; m * m];
    for i in 0..m {
        for j in 0..m {
            let mut sum = 0.0;
            for ki in 0..n {
                sum += n_v[i * n + ki] * s_ni[ki * m + j];
            }
            k[i * m + j] = sum;
        }
    }
    k
}

/// Validate that a device model parameter is positive and finite.
fn validate_positive_finite(value: f64, param_label: &str) -> Result<(), CodegenError> {
    if value <= 0.0 || !value.is_finite() {
        return Err(CodegenError::InvalidConfig(format!(
            "{param_label} must be positive finite, got {value}"
        )));
    }
    Ok(())
}

/// Compute backward Euler fallback matrices for the DK codegen path.
///
/// Returns (s_be, k_be, a_neg_be, rhs_const_be) or empty vecs if BE fallback is disabled.
/// The BE matrices use alpha_be = 1/T (instead of trapezoidal alpha = 2/T).
fn compute_dk_be_fallback(
    g_matrix: &[f64],
    c_matrix: &[f64],
    n: usize,
    m: usize,
    n_nodes: usize,
    n_v: &[f64],
    n_i: &[f64],
    internal_rate: f64,
    mna: &MnaSystem,
) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>), CodegenError> {
    let alpha_be = internal_rate; // BE: alpha = 1/T

    // Build A_be = G + alpha_be * C
    let mut a_be = vec![0.0f64; n * n];
    let mut a_neg_be = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..n {
            let g = g_matrix[i * n + j];
            let c = c_matrix[i * n + j];
            a_be[i * n + j] = g + alpha_be * c;
            a_neg_be[i * n + j] = alpha_be * c; // BE: no -G term
        }
    }

    // Zero VS/VCVS/ideal-transformer algebraic rows in A_neg_be
    for vs in &mna.voltage_sources {
        let row = n_nodes + vs.ext_idx;
        if row < n {
            for j in 0..n {
                a_neg_be[row * n + j] = 0.0;
            }
        }
    }
    let num_vs = mna.voltage_sources.len();
    for (idx, _) in mna.vcvs_sources.iter().enumerate() {
        let row = n_nodes + num_vs + idx;
        if row < n {
            for j in 0..n {
                a_neg_be[row * n + j] = 0.0;
            }
        }
    }
    let num_vcvs = mna.vcvs_sources.len();
    for (idx, _) in mna.ideal_transformers.iter().enumerate() {
        let row = n_nodes + num_vs + num_vcvs + idx;
        if row < n {
            for j in 0..n {
                a_neg_be[row * n + j] = 0.0;
            }
        }
    }

    // S_be = A_be^{-1}
    let s_be = invert_flat_matrix(&a_be, n)?;

    // K_be = N_v * S_be * N_i
    let k_be = if m > 0 {
        compute_k_from_s(&s_be, n_v, n_i, n, m)
    } else {
        Vec::new()
    };

    // BE rhs_const: current sources ×1 (not ×2), VS ×1
    let mut rhs_const_be = vec![0.0f64; n];
    for src in &mna.current_sources {
        crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_plus_idx, src.dc_value);
        crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_minus_idx, -src.dc_value);
    }
    for vs in &mna.voltage_sources {
        let k_row = n_nodes + vs.ext_idx;
        if k_row < n {
            rhs_const_be[k_row] = vs.dc_value;
        }
    }

    Ok((s_be, k_be, a_neg_be, rhs_const_be))
}

impl CircuitIR {
    /// Build a `CircuitIR` from the compiled kernel, MNA system, netlist, and config.
    ///
    /// # Errors
    /// Returns `CodegenError::InvalidConfig` if any device model parameter is invalid.
    pub fn from_kernel(
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
    ) -> Result<Self, CodegenError> {
        Self::from_kernel_with_dc_op(kernel, mna, netlist, config, None)
    }

    /// Build CircuitIR from DK kernel with an optional pre-computed DC operating point.
    ///
    /// When `dc_op_result` is provided, it is used instead of running the DC OP solver.
    /// This is useful when the MNA has been expanded with internal nodes after the DC OP
    /// was computed on the original (unexpanded) MNA — the DC OP converges better on
    /// the smaller system.
    pub fn from_kernel_with_dc_op(
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        dc_op_result: Option<dc_op::DcOpResult>,
    ) -> Result<Self, CodegenError> {
        let n = kernel.n; // = n_aug (system dimension)
        let n_nodes = kernel.n_nodes; // original circuit node count
        let m = kernel.m;

        if m > crate::dk::MAX_M {
            return Err(CodegenError::UnsupportedTopology(format!(
                "code generation supports at most M={} nonlinear dimensions, got M={}",
                crate::dk::MAX_M,
                m
            )));
        }

        // Detect augmented inductors: kernel.inductors is empty when from_mna_augmented
        // was used (companion vectors cleared), and kernel.n > mna.n_aug means extra
        // inductor branch variables were added.
        let augmented_inductors = kernel.inductors.is_empty() && (kernel.n > mna.n_aug);

        let topology = Topology {
            n,
            n_nodes,
            m,
            num_devices: kernel.num_devices,
            n_aug: mna.n_aug,
            augmented_inductors,
            num_linearized_devices: mna.linearized_triodes.len()
                + mna.linearized_bjts.len(),
        };

        let os_factor = config.oversampling_factor;
        let internal_rate = config.sample_rate * os_factor as f64;

        // Auto-detect stiffness: if the trapezoidal time-stepping operator S*A_neg
        // has spectral radius > 1.001 (above stability boundary), the circuit
        // is too stiff for trapezoidal and needs backward Euler.
        // Note: for DK codegen, auto_be=true means the circuit should be routed
        // to the nodal solver instead (BE on DK still diverges for high-S circuits).
        let auto_be = if !config.backward_euler && n > 0 {
            // Compute spectral radius estimate via power iteration on S_trap * A_neg_trap
            let s_trap = &kernel.s;
            let a_neg_trap = &kernel.a_neg;
            // One matrix-vector multiply: y = S * A_neg * x, iterate to find dominant eigenvalue
            let mut x = vec![1.0 / (n as f64).sqrt(); n];
            let mut y = vec![0.0; n];
            let mut spectral_radius = 0.0;
            for _ in 0..20 {
                // 20 power iterations
                // y = S * (A_neg * x)
                let mut ax = vec![0.0; n];
                for i in 0..n {
                    for j in 0..n {
                        ax[i] += a_neg_trap[i * n + j] * x[j];
                    }
                }
                for i in 0..n {
                    y[i] = 0.0;
                    for j in 0..n {
                        y[i] += s_trap[i * n + j] * ax[j];
                    }
                }
                let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
                if norm < 1e-30 {
                    break;
                }
                spectral_radius = norm / x.iter().map(|v| v * v).sum::<f64>().sqrt();
                for i in 0..n {
                    x[i] = y[i] / norm;
                }
            }
            if spectral_radius > 1.002 {
                log::info!("Auto-selecting backward Euler: spectral radius {:.4} > 1.002 (trapezoidal unstable)", spectral_radius);
                true
            } else {
                false
            }
        } else {
            false
        };
        let be = config.backward_euler || auto_be;
        let alpha = if be {
            internal_rate
        } else {
            2.0 * internal_rate
        };

        // Validate output_nodes against circuit node count
        for (i, &node) in config.output_nodes.iter().enumerate() {
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (circuit node count)",
                    i, node, n_nodes
                )));
            }
        }

        let rail_mode = refine_active_set_for_audio_path(
            resolve_opamp_rail_mode(mna, config.opamp_rail_mode),
            mna,
            netlist,
        );
        log::info!(
            "Op-amp rail mode: {} ({})",
            rail_mode.mode,
            rail_mode.reason.as_str()
        );

        let solver_config = SolverConfig {
            sample_rate: config.sample_rate,
            alpha,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_nodes: config.output_nodes.clone(),
            input_resistance: config.input_resistance,
            oversampling_factor: os_factor,
            output_scales: config.output_scales.clone(),
            pot_settle_samples: config.pot_settle_samples,
            backward_euler: be,
            opamp_rail_mode: rail_mode.mode,
            emit_dc_op_recompute: config.emit_dc_op_recompute,
        };

        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        // Store the raw G and C matrices for runtime sample rate recomputation.
        // The MNA G matrix already includes input conductance (stamped before kernel build).
        // When augmented inductors are used, kernel.n > mna.n_aug, so we need the
        // augmented G/C (with inductor KCL/KVL/L stamps) at the full n_nodal dimension.
        let (g_matrix, c_matrix) = if augmented_inductors {
            let aug = mna.build_augmented_matrices();
            (
                dk::flatten_matrix(&aug.g, n, n),
                dk::flatten_matrix(&aug.c, n, n),
            )
        } else {
            (
                dk::flatten_matrix(&mna.g, n, n),
                dk::flatten_matrix(&mna.c, n, n),
            )
        };

        let matrices = if os_factor > 1 {
            // Recompute matrices at internal (oversampled) rate from G and C.
            let alpha = 2.0 * internal_rate;
            let t = 1.0 / internal_rate;

            // Build A = G + alpha*C
            let mut a_flat = vec![0.0f64; n * n];
            let mut a_neg_flat = vec![0.0f64; n * n];
            for i in 0..n {
                for j in 0..n {
                    let g = g_matrix[i * n + j];
                    let c = c_matrix[i * n + j];
                    a_flat[i * n + j] = g + alpha * c;
                    a_neg_flat[i * n + j] = alpha * c - g;
                }
            }

            if !augmented_inductors {
                // Companion model path: stamp inductor conductances at internal rate
                for ind in &kernel.inductors {
                    let g_eq = t / (2.0 * ind.inductance);
                    stamp_flat_conductance(&mut a_flat, n, ind.node_i, ind.node_j, g_eq);
                    stamp_flat_conductance(&mut a_neg_flat, n, ind.node_i, ind.node_j, -g_eq);
                }

                // Stamp coupled inductor companion conductances at internal rate
                for ci in &kernel.coupled_inductors {
                    let m_val = ci.coupling * (ci.l1_inductance * ci.l2_inductance).sqrt();
                    let det = ci.l1_inductance * ci.l2_inductance - m_val * m_val;
                    let half_t = t / 2.0;
                    let gs1 = half_t * ci.l2_inductance / det;
                    let gs2 = half_t * ci.l1_inductance / det;
                    let gm = -half_t * m_val / det;
                    stamp_flat_conductance(&mut a_flat, n, ci.l1_node_i, ci.l1_node_j, gs1);
                    stamp_flat_conductance(&mut a_neg_flat, n, ci.l1_node_i, ci.l1_node_j, -gs1);
                    stamp_flat_conductance(&mut a_flat, n, ci.l2_node_i, ci.l2_node_j, gs2);
                    stamp_flat_conductance(&mut a_neg_flat, n, ci.l2_node_i, ci.l2_node_j, -gs2);
                    stamp_flat_mutual(
                        &mut a_flat,
                        n,
                        ci.l1_node_i,
                        ci.l1_node_j,
                        ci.l2_node_i,
                        ci.l2_node_j,
                        gm,
                    );
                    stamp_flat_mutual(
                        &mut a_flat,
                        n,
                        ci.l2_node_i,
                        ci.l2_node_j,
                        ci.l1_node_i,
                        ci.l1_node_j,
                        gm,
                    );
                    stamp_flat_mutual(
                        &mut a_neg_flat,
                        n,
                        ci.l1_node_i,
                        ci.l1_node_j,
                        ci.l2_node_i,
                        ci.l2_node_j,
                        -gm,
                    );
                    stamp_flat_mutual(
                        &mut a_neg_flat,
                        n,
                        ci.l2_node_i,
                        ci.l2_node_j,
                        ci.l1_node_i,
                        ci.l1_node_j,
                        -gm,
                    );
                }
            }

            // Zero VS/VCVS/ideal-transformer algebraic rows in A_neg (NOT inductor
            // rows or internal BJT node rows — they need trapezoidal history).
            for vs in &mna.voltage_sources {
                let row = n_nodes + vs.ext_idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let num_vs = mna.voltage_sources.len();
            for (vcvs_idx, _) in mna.vcvs_sources.iter().enumerate() {
                let row = n_nodes + num_vs + vcvs_idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let num_vcvs = mna.vcvs_sources.len();
            for (xfmr_idx, _) in mna.ideal_transformers.iter().enumerate() {
                let row = n_nodes + num_vs + num_vcvs + xfmr_idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }

            // Invert A to get S
            let s = invert_flat_matrix(&a_flat, n)?;

            // Compute K = N_v * S * N_i
            let k = compute_k_from_s(&s, &kernel.n_v, &kernel.n_i, n, m);

            // Compute BE fallback matrices for adaptive per-sample fallback
            let want_be_fallback = !config.backward_euler && !config.disable_be_fallback && m > 0;
            let (s_be, k_be, a_neg_be, rhs_const_be) = if want_be_fallback {
                compute_dk_be_fallback(
                    &g_matrix, &c_matrix, n, m, n_nodes,
                    &kernel.n_v, &kernel.n_i, internal_rate, mna,
                )?
            } else {
                (Vec::new(), Vec::new(), Vec::new(), Vec::new())
            };

            Matrices {
                s,
                a_neg: a_neg_flat,
                k,
                n_v: kernel.n_v.clone(),
                n_i: kernel.n_i.clone(),
                rhs_const: kernel.rhs_const.clone(),
                g_matrix,
                c_matrix,
                a_matrix: Vec::new(),
                a_matrix_be: Vec::new(),
                a_neg_be,
                rhs_const_be,
                s_be,
                k_be,
                spectral_radius_s_aneg: 0.0,
                s_sub: Vec::new(),
                k_sub: Vec::new(),
                a_neg_sub: Vec::new(),
            }
        } else if be {
            // Backward Euler: recompute S, A_neg, K from G/C with alpha = 1/T
            let mut a_flat = vec![0.0f64; n * n];
            let mut a_neg_flat = vec![0.0f64; n * n];
            for i in 0..n {
                for j in 0..n {
                    let g = g_matrix[i * n + j];
                    let c = c_matrix[i * n + j];
                    a_flat[i * n + j] = g + alpha * c;
                    a_neg_flat[i * n + j] = alpha * c; // BE: no -G term
                }
            }
            // Zero VS/VCVS algebraic rows in A_neg
            for vs in &mna.voltage_sources {
                let row = mna.n + vs.ext_idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let num_vs = mna.voltage_sources.len();
            for (idx, _) in mna.vcvs_sources.iter().enumerate() {
                let row = mna.n + num_vs + idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let num_vcvs = mna.vcvs_sources.len();
            for (idx, _) in mna.ideal_transformers.iter().enumerate() {
                let row = mna.n + num_vs + num_vcvs + idx;
                if row < n {
                    for j in 0..n {
                        a_neg_flat[row * n + j] = 0.0;
                    }
                }
            }
            let s_flat = invert_flat_matrix(&a_flat, n)?;
            let k_flat = if m > 0 {
                compute_k_from_s(&s_flat, &kernel.n_v, &kernel.n_i, n, m)
            } else {
                Vec::new()
            };
            // BE rhs_const: current sources x1 (not x2), VS x1
            let mut rhs_const_be = vec![0.0f64; n];
            for src in &mna.current_sources {
                crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_plus_idx, src.dc_value);
                crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_minus_idx, -src.dc_value);
            }
            for vs in &mna.voltage_sources {
                let k_row = mna.n + vs.ext_idx;
                if k_row < n {
                    rhs_const_be[k_row] = vs.dc_value;
                }
            }
            Matrices {
                s: s_flat,
                a_neg: a_neg_flat,
                k: k_flat,
                n_v: kernel.n_v.clone(),
                n_i: kernel.n_i.clone(),
                rhs_const: rhs_const_be,
                g_matrix,
                c_matrix,
                a_matrix: Vec::new(),
                a_matrix_be: Vec::new(),
                a_neg_be: Vec::new(),
                rhs_const_be: Vec::new(),
                s_be: Vec::new(),
                k_be: Vec::new(),
                spectral_radius_s_aneg: 0.0,
                s_sub: Vec::new(),
                k_sub: Vec::new(),
                a_neg_sub: Vec::new(),
            }
        } else {
            // Standard trapezoidal: use kernel matrices directly.
            // Also compute BE fallback matrices for adaptive per-sample fallback.
            let want_be_fallback = !config.disable_be_fallback && m > 0;
            let (s_be, k_be, a_neg_be, rhs_const_be) = if want_be_fallback {
                compute_dk_be_fallback(
                    &g_matrix, &c_matrix, n, m, n_nodes,
                    &kernel.n_v, &kernel.n_i, internal_rate, mna,
                )?
            } else {
                (Vec::new(), Vec::new(), Vec::new(), Vec::new())
            };

            Matrices {
                s: kernel.s.clone(),
                a_neg: kernel.a_neg.clone(),
                k: kernel.k.clone(),
                n_v: kernel.n_v.clone(),
                n_i: kernel.n_i.clone(),
                rhs_const: kernel.rhs_const.clone(),
                g_matrix,
                c_matrix,
                a_matrix: Vec::new(),
                a_matrix_be: Vec::new(),
                a_neg_be,
                rhs_const_be,
                s_be,
                k_be,
                spectral_radius_s_aneg: 0.0,
                s_sub: Vec::new(),
                k_sub: Vec::new(),
                a_neg_sub: Vec::new(),
            }
        };

        // BE fallback matrices are populated for nonlinear circuits (m>0) unless
        // config.disable_be_fallback is set. Linear circuits (m=0) skip BE fallback
        // since they don't have NR iteration that could diverge.

        let mut device_slots = Self::build_device_info_with_mna(netlist, Some(mna))?;
        Self::resolve_mosfet_nodes(&mut device_slots, mna);

        let inductors: Vec<InductorIR> = kernel
            .inductors
            .iter()
            .map(|ind| {
                // Recompute g_eq at internal rate when oversampling
                let g_eq = if os_factor > 1 {
                    1.0 / (2.0 * internal_rate * ind.inductance)
                } else {
                    ind.g_eq
                };
                InductorIR {
                    name: ind.name.to_string(),
                    node_i: ind.node_i,
                    node_j: ind.node_j,
                    g_eq,
                    inductance: ind.inductance,
                }
            })
            .collect();

        let coupled_inductors: Vec<CoupledInductorIR> = kernel
            .coupled_inductors
            .iter()
            .map(|ci| {
                // Recompute conductances at internal rate when oversampling
                let (g_self_1, g_self_2, g_mutual) = if os_factor > 1 {
                    let t = 1.0 / internal_rate;
                    let m_val = ci.coupling * (ci.l1_inductance * ci.l2_inductance).sqrt();
                    let det = ci.l1_inductance * ci.l2_inductance - m_val * m_val;
                    let half_t = t / 2.0;
                    (
                        half_t * ci.l2_inductance / det,
                        half_t * ci.l1_inductance / det,
                        -half_t * m_val / det,
                    )
                } else {
                    (ci.g_self_1, ci.g_self_2, ci.g_mutual)
                };
                CoupledInductorIR {
                    name: ci.name.clone(),
                    l1_name: ci.l1_name.clone(),
                    l2_name: ci.l2_name.clone(),
                    l1_node_i: ci.l1_node_i,
                    l1_node_j: ci.l1_node_j,
                    l2_node_i: ci.l2_node_i,
                    l2_node_j: ci.l2_node_j,
                    l1_inductance: ci.l1_inductance,
                    l2_inductance: ci.l2_inductance,
                    coupling: ci.coupling,
                    g_self_1,
                    g_self_2,
                    g_mutual,
                }
            })
            .collect();

        let transformer_groups: Vec<TransformerGroupIR> = kernel
            .transformer_groups
            .iter()
            .map(|g| {
                let w = g.num_windings;
                // Recompute Y matrix at internal rate when oversampling
                let y_matrix = if os_factor > 1 {
                    let t = 1.0 / internal_rate;
                    let half_t = t / 2.0;
                    let mut l_mat = vec![vec![0.0f64; w]; w];
                    for i in 0..w {
                        for j in 0..w {
                            l_mat[i][j] = g.coupling_matrix[i][j]
                                * (g.inductances[i] * g.inductances[j]).sqrt();
                        }
                    }
                    let y_raw = crate::mna::invert_small_matrix(&l_mat);
                    let mut y_flat = vec![0.0f64; w * w];
                    for i in 0..w {
                        for j in 0..w {
                            y_flat[i * w + j] = half_t * y_raw[i][j];
                        }
                    }
                    y_flat
                } else {
                    g.y_matrix.clone()
                };
                let mut coupling_flat = vec![0.0f64; w * w];
                for i in 0..w {
                    for j in 0..w {
                        coupling_flat[i * w + j] = g.coupling_matrix[i][j];
                    }
                }
                TransformerGroupIR {
                    name: g.name.clone(),
                    num_windings: w,
                    winding_names: g.winding_names.clone(),
                    winding_node_i: g.winding_node_i.clone(),
                    winding_node_j: g.winding_node_j.clone(),
                    inductances: g.inductances.clone(),
                    coupling_flat,
                    y_matrix,
                }
            })
            .collect();

        let pots = kernel
            .pots
            .iter()
            .map(|p| PotentiometerIR {
                su: p.su.clone(),
                usu: p.usu,
                g_nominal: p.g_nominal,
                nv_su: p.nv_su.clone(),
                u_ni: p.u_ni.clone(),
                node_p: p.node_p,
                node_q: p.node_q,
                min_resistance: p.min_resistance,
                max_resistance: p.max_resistance,
                grounded: p.grounded,
                runtime_field: p.runtime_field.clone(),
            })
            .collect();

        let wiper_groups: Vec<WiperGroupIR> = kernel
            .wiper_groups
            .iter()
            .map(|wg| WiperGroupIR {
                cw_pot_index: wg.cw_pot_index,
                ccw_pot_index: wg.ccw_pot_index,
                total_resistance: wg.total_resistance,
                default_position: wg.default_position,
                label: wg.label.clone(),
            })
            .collect();

        let gang_groups: Vec<GangGroupIR> = kernel
            .gang_groups
            .iter()
            .map(|gg| GangGroupIR {
                label: gg.label.clone(),
                pot_members: gg
                    .pot_members
                    .iter()
                    .map(|&(pot_idx, inverted)| GangPotMemberIR {
                        pot_index: pot_idx,
                        min_resistance: mna.pots[pot_idx].min_resistance,
                        max_resistance: mna.pots[pot_idx].max_resistance,
                        inverted,
                    })
                    .collect(),
                wiper_members: gg
                    .wiper_members
                    .iter()
                    .map(|&(wg_idx, inverted)| GangWiperMemberIR {
                        wiper_group_index: wg_idx,
                        total_resistance: mna.wiper_groups[wg_idx].total_resistance,
                        inverted,
                    })
                    .collect(),
                default_position: gg.default_position,
            })
            .collect();

        // Build switches from MNA resolved info
        let switches: Vec<SwitchIR> = mna
            .switches
            .iter()
            .enumerate()
            .map(|(idx, sw)| {
                let components = sw
                    .components
                    .iter()
                    .map(|comp| {
                        // For inductor components, find matching index in the inductors vec
                        let inductor_index = if comp.component_type == 'L' {
                            kernel
                                .inductors
                                .iter()
                                .position(|ind| ind.name.eq_ignore_ascii_case(&comp.name))
                        } else {
                            None
                        };
                        // For coupled inductor windings (not in uncoupled list),
                        // find the coupled pair index and winding number.
                        let (coupled_inductor_index, coupled_winding) =
                            if comp.component_type == 'L' && inductor_index.is_none() {
                                let mut found = None;
                                for (ci_idx, ci) in kernel.coupled_inductors.iter().enumerate() {
                                    if ci.l1_name.eq_ignore_ascii_case(&comp.name) {
                                        found = Some((ci_idx, 1u8));
                                        break;
                                    }
                                    if ci.l2_name.eq_ignore_ascii_case(&comp.name) {
                                        found = Some((ci_idx, 2u8));
                                        break;
                                    }
                                }
                                found.map_or((None, None), |(i, w)| (Some(i), Some(w)))
                            } else {
                                (None, None)
                            };
                        SwitchComponentIR {
                            name: comp.name.clone(),
                            component_type: comp.component_type,
                            node_p: comp.node_p,
                            node_q: comp.node_q,
                            nominal_value: comp.nominal_value,
                            inductor_index,
                            augmented_row: None, // DK path uses companion model
                            coupled_inductor_index,
                            coupled_winding,
                        }
                    })
                    .collect();
                SwitchIR {
                    index: idx,
                    components,
                    positions: sw.positions.clone(),
                    num_positions: sw.positions.len(),
                    mutual_entries: Vec::new(), // DK path uses companion model
                }
            })
            .collect();

        let has_dc_sources = kernel.rhs_const.iter().any(|&v| v != 0.0);

        // Use pre-computed DC OP if available, otherwise run solver.
        // Pre-computed DC OP is used when the MNA has been expanded with internal
        // nodes — the solver converges better on the original (unexpanded) system.
        let dc_result = if let Some(pre) = dc_op_result {
            pre
        } else {
            let dc_op_config = DcOpConfig {
                tolerance: config.dc_op_tolerance,
                max_iterations: config.dc_op_max_iterations,
                input_node: config.input_node,
                input_resistance: config.input_resistance,
                ..DcOpConfig::default()
            };
            dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config)
        };
        // Check DC OP significance on the truncated vector (n_aug), not the full
        // n_dc vector which includes inductor branch currents.
        let dc_op_len = dc_result.v_node.len();
        let dc_op_truncated = &dc_result.v_node[..kernel.n.min(dc_op_len)];
        let has_dc_op = dc_op_truncated.iter().any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_op_method = format!("{:?}", dc_result.method);
        let dc_op_iterations = dc_result.iterations;
        let dc_nl_currents = dc_result.i_nl.clone();

        if !dc_result.converged && m > 0 {
            log::warn!(
                "nonlinear DC OP solver did not converge (method: {:?}), using best estimate",
                dc_result.method
            );
        }

        // Forward-active BJT detection happens BEFORE from_kernel is called.
        // The caller (detect_forward_active_bjts + CLI) handles MNA/kernel rebuild.
        // By the time we get here, kernel/mna already have the correct M dimension.

        // Analyze sparsity patterns for compile-time matrices
        let sparsity = SparseInfo {
            a_neg: analyze_matrix_sparsity(&matrices.a_neg, n, n),
            n_v: analyze_matrix_sparsity(&matrices.n_v, m, n),
            n_i: analyze_matrix_sparsity(&matrices.n_i, n, m),
            k: analyze_matrix_sparsity(&matrices.k, m, m),
            lu: None, // DK path doesn't use full LU
        };

        let named_constants = build_named_constants(mna, topology.n_nodes);
        let runtime_sources: Vec<RuntimeSourceIR> = mna
            .runtime_sources
            .iter()
            .map(|rt| RuntimeSourceIR {
                vs_name: rt.vs_name.clone(),
                field_name: rt.field_name.clone(),
                vs_row: rt.vs_row,
            })
            .collect();

        Ok(CircuitIR {
            metadata,
            topology,
            solver_mode: SolverMode::Dk,
            solver_config,
            matrices,
            // For augmented inductors, the DC OP solver returns n_aug-sized vectors
            // but the kernel dimension is n_nodal = n_aug + n_inductor_vars.
            // Pad with zeros for inductor branch currents (DC OP doesn't solve them).
            // For companion model, truncate to kernel.n (= n_aug).
            dc_operating_point: {
                // DC OP may return fewer nodes than kernel.n (e.g., when computed
                // on unexpanded MNA before internal node expansion). Pad with zeros.
                let mut dc = dc_result.v_node.clone();
                dc.resize(kernel.n, 0.0);
                dc.truncate(kernel.n);
                // Clamp op-amp output nodes to VCC/VEE supply rails.
                // The DC OP solver doesn't know about supply rails and can converge
                // to voltages beyond rail limits at output/internal nodes.
                for oa in &mna.opamps {
                    if (oa.vcc.is_finite() || oa.vee.is_finite()) && oa.n_out_idx > 0 {
                        let o = oa.n_out_idx - 1;
                        if o < dc.len() {
                            dc[o] = dc[o].clamp(oa.vee, oa.vcc);
                        }
                    }
                }
                dc
            },
            device_slots,
            has_dc_sources,
            has_dc_op,
            dc_nl_currents,
            dc_op_converged,
            dc_op_method,
            dc_op_iterations,
            dc_block: config.dc_block,
            inductors,
            coupled_inductors,
            transformer_groups,
            saturating_inductors: Vec::new(), // DK path: saturation routes to nodal
            saturating_coupled: Vec::new(),
            saturating_xfmr_groups: Vec::new(),
            pots,
            wiper_groups,
            gang_groups,
            switches,
            opamps: mna
                .opamps
                .iter()
                .filter(|oa| {
                    // Include op-amps that need any codegen-emitted post-NR
                    // processing: rail clamping (finite VCC/VEE) OR slew-rate
                    // limiting (finite SR). Pure ideal op-amps with all three
                    // infinite are skipped — the emitter produces no op-amp
                    // code for them, preserving byte-identical output for
                    // existing circuits.
                    (oa.vcc.is_finite() || oa.vee.is_finite() || oa.sr.is_finite())
                        && oa.n_out_idx > 0
                })
                .map(|oa| opamp_ir_from_info(oa, netlist, mna))
                .collect(),
            sparsity,
            opamp_iir: Vec::new(), // IIR op-amp handled in nodal path only
            noise: NoiseIR {
                mode: config.noise_mode,
                master_seed: config.noise_master_seed,
                thermal_sources: if config.noise_mode.includes_thermal() {
                    collect_thermal_noise_sources(netlist, mna)
                } else {
                    Vec::new()
                },
                shot_sources: if config.noise_mode.includes_shot() {
                    collect_shot_noise_sources(mna)
                } else {
                    Vec::new()
                },
                flicker_sources: if config.noise_mode.includes_full() {
                    collect_flicker_noise_sources(netlist, mna)
                } else {
                    Vec::new()
                },
            },
            named_constants,
            runtime_sources,
        })
    }

    /// Build CircuitIR for the nodal solver path (no DkKernel needed).
    ///
    /// Uses augmented MNA: inductors are branch current variables in G/C.
    /// The generated code does full N×N NR per sample instead of DK's M×M.
    pub fn from_mna(
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
    ) -> Result<Self, CodegenError> {
        let n_nodes = mna.n;
        let n_aug = mna.n_aug;
        let m = mna.m;

        if m > dk::MAX_M {
            return Err(CodegenError::InvalidConfig(format!(
                "Nonlinear dimension M={} exceeds MAX_M={}",
                m,
                dk::MAX_M
            )));
        }

        // Build augmented G/C matrices (includes inductor branch variables)
        let mut aug = mna.build_augmented_matrices();
        let n = aug.n_nodal;

        // Gmin regularization: prevent singular Jacobians on floating nodes.
        // Matches runtime NodalSolver (solver.rs Gmin stamping).
        for i in 0..n_nodes {
            aug.g[i][i] += 1e-12;
        }

        let sample_rate = config.sample_rate;
        let internal_rate = sample_rate * config.oversampling_factor as f64;
        // Provisional integrator. If the nodal auto-detector decides the trap
        // propagation operator `S*A_neg` is unstable (spectral_radius > 1.002,
        // matching the `schur_unstable` gate in `nodal_emitter.rs`), the
        // promotion block below swaps in the BE matrices already built as the
        // transient fallback and flips `be`/`alpha`/`solver_config` in place.
        let mut alpha = if config.backward_euler {
            internal_rate
        } else {
            2.0 * internal_rate
        };
        let alpha_be = internal_rate;

        // Validate output_nodes against circuit node count
        for (i, &node) in config.output_nodes.iter().enumerate() {
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (circuit node count)",
                    i, node, n_nodes
                )));
            }
        }

        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        // Build A = G + alpha*C, A_neg = alpha*C - G (trapezoidal) or alpha*C (BE)
        // NOTE: These initial matrices include Gm stamps. After DC OP, we strip Gm
        // from aug.g and rebuild A/A_neg for transient (IIR filter handles VCCS externally).
        let be = config.backward_euler;
        let mut a_flat = vec![0.0f64; n * n];
        let mut a_neg_flat = vec![0.0f64; n * n];
        for i in 0..n {
            for j in 0..n {
                let g = aug.g[i][j];
                let c = aug.c[i][j];
                a_flat[i * n + j] = g + alpha * c;
                a_neg_flat[i * n + j] = if be { alpha * c } else { alpha * c - g };
            }
        }
        // Zero ALL augmented rows in A_neg (n_nodes..n_aug).
        // This matches the runtime NodalSolver (solver.rs line ~3199) which
        // zeros all rows from n_nodes..n_aug, not just VS/VCVS rows.
        //
        // For augmented variables that are algebraic (VS, VCVS), this is
        // mandatory — they have no dynamics (no C), so A_neg = -G and the
        // trapezoidal history term A_neg*v_prev would inject unstable feedback.
        //
        // For Boyle op-amp internal nodes, zeroing A_neg effectively uses
        // backward Euler instead of trapezoidal for the dominant pole. This is
        // unconditionally stable and avoids the Gm-induced spectral radius > 1
        // instability that trapezoidal creates for high-gain VCCS elements
        // (AOL>10000 → Gm>1000 → A_neg has ±1000 entries → unstable feedback).
        //
        // Inductor branch rows (n_aug..n_nodal) are NOT zeroed — they have
        // real L dynamics in C that need trapezoidal integration.
        if n_aug > n_nodes {
            for i in n_nodes..n_aug {
                if i < n {
                    for j in 0..n {
                        a_neg_flat[i * n + j] = 0.0;
                    }
                }
            }
        }

        // Build backward Euler: A_be = G + (1/T)*C, A_neg_be = (1/T)*C
        let mut a_be_flat = vec![0.0f64; n * n];
        let mut a_neg_be_flat = vec![0.0f64; n * n];
        for i in 0..n {
            for j in 0..n {
                let g = aug.g[i][j];
                let c = aug.c[i][j];
                a_be_flat[i * n + j] = g + alpha_be * c;
                a_neg_be_flat[i * n + j] = alpha_be * c;
            }
        }
        // Zero all augmented rows in A_neg_be (matching A_neg treatment)
        if n_aug > n_nodes {
            for i in n_nodes..n_aug {
                if i < n {
                    for j in 0..n {
                        a_neg_be_flat[i * n + j] = 0.0;
                    }
                }
            }
        }

        // Expand N_v (m × n_aug → m × n_nodal) and N_i (n_aug × m → n_nodal × m)
        let mut n_v_flat = vec![0.0f64; m * n];
        for i in 0..m {
            for j in 0..n_aug {
                n_v_flat[i * n + j] = mna.n_v[i][j];
            }
        }
        let mut n_i_flat = vec![0.0f64; n * m];
        for i in 0..n_aug {
            for j in 0..m {
                n_i_flat[i * m + j] = mna.n_i[i][j];
            }
        }

        // Build rhs_const: trapezoidal (node rows ×2, VS rows ×1) or BE (all ×1)
        let mut rhs_const = if be {
            // BE: current sources ×1 (not ×2), VS ×1
            let mut rc = vec![0.0f64; n];
            for src in &mna.current_sources {
                crate::mna::inject_rhs_current(&mut rc, src.n_plus_idx, src.dc_value);
                crate::mna::inject_rhs_current(&mut rc, src.n_minus_idx, -src.dc_value);
            }
            for vs in &mna.voltage_sources {
                let k = mna.n + vs.ext_idx;
                if k < n {
                    rc[k] = vs.dc_value;
                }
            }
            rc
        } else {
            let rhs_const_base = dk::build_rhs_const(mna);
            let mut rc = vec![0.0f64; n];
            for i in 0..n_aug {
                rc[i] = rhs_const_base[i];
            }
            rc
        };

        // Build BE rhs_const (node rows ×1, VS rows ×1) — for fallback
        let mut rhs_const_be = vec![0.0f64; n];
        for src in &mna.current_sources {
            crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_plus_idx, src.dc_value);
            crate::mna::inject_rhs_current(&mut rhs_const_be, src.n_minus_idx, -src.dc_value);
        }
        for vs in &mna.voltage_sources {
            let k = mna.n + vs.ext_idx;
            if k < n {
                rhs_const_be[k] = vs.dc_value;
            }
        }

        // === IIR op-amp: PURE EXPLICIT bilinear-transformed dominant pole ===
        //
        // Goal: avoid stamping Gm (~1000 S) in G (causes catastrophic conditioning)
        // while still getting correct DC gain AOL.
        //
        // Continuous-time: V_int = Gm*(V+ - V-) / (Go + s*C_dom), I_out = Go*V_int.
        // Bilinear transform with a 1-sample input delay (x[n] → x[n-1]) gives a
        // pure explicit update:
        //   y[n] = a1*y[n-1] + 2*b0*x[n-1]
        //   a1 = (alpha*C_dom - Go) / (alpha*C_dom + Go)
        //   b0 = Gm / (alpha*C_dom + Go)
        //
        // Output current injected via RHS (trapezoidal):
        //   I_inject = Go*(y[n] + y[n-1])
        //
        // Both y[n] and y[n-1] depend only on PREVIOUS state, so NOTHING related to
        // Gm or x[n] appears in the matrix. The Gm stamps from mna.rs are stripped
        // from G entirely — only Go (output resistance) remains at G[o][o].
        //
        // DC gain check: y_ss*(1-a1) = 2*b0*x_ss  ⇒  y_ss = (Gm/Go)*x_ss = AOL*x_ss ✓
        //
        // Tradeoff: 1-sample phase delay at high frequencies — negligible for audio
        // at typical oversampled rates (48-192 kHz), and the win is that the matrix
        // stays well-conditioned regardless of AOL.
        // IIR op-amp path selection:
        // - No VCAs (simple op-amp circuits): use pure explicit IIR — strip Gm from G,
        //   inject via RHS. Matrix conditioning is excellent (max|A| ~ Go).
        // - With VCAs: the VCA feedback loop needs tight implicit coupling to the op-amp.
        //   Leave Gm in G (ideal op-amp, no GBW rolloff in solver). Accept the ill-
        //   conditioning — for VCR-ALC-class circuits it's tolerable at N~18.
        let has_vca = !mna.vcas.is_empty();
        let mut opamp_iir_data: Vec<OpampIirData> = Vec::new();
        // IIR op-amp path is currently disabled: the explicit 1-sample-delay
        // feedback causes main-NR divergence on circuits like the Klon Centaur
        // (sub-step fallback runs every sample, drops us to 0.25x realtime, and
        // itself omits the IIR RHS stamp so the op-amps don't function).
        // Falling through to the ideal op-amp path (Gm in G) works for all
        // currently validated circuits. Re-enable when IIR math is fixed.
        #[allow(clippy::overly_complex_bool_expr)]
        if !has_vca && false {
        for oa in &mna.opamps {
            if oa.gbw.is_finite() && oa.gbw > 0.0 && oa.n_out_idx > 0 {
                let o = oa.n_out_idx - 1;
                let gm = oa.aol / oa.r_out;
                let go = 1.0 / oa.r_out;
                let np = oa.n_plus_idx;
                let nm = oa.n_minus_idx;
                let c_dom = oa.iir_c_dom;

                // Pure-explicit IIR: strip Gm from G entirely.
                if np > 0 && o < n {
                    aug.g[o][np - 1] -= gm;
                }
                if nm > 0 && o < n {
                    aug.g[o][nm - 1] += gm;
                }

                let np_idx = if np > 0 { Some(np - 1) } else { None };
                let nm_idx = if nm > 0 { Some(nm - 1) } else { None };

                opamp_iir_data.push(OpampIirData {
                    gm,
                    go,
                    c_dom,
                    np_idx,
                    nm_idx,
                    out_idx: o,
                    vclamp_hi: oa.vcc,
                    vclamp_lo: oa.vee,
                });

                log::info!(
                    "IIR op-amp {} (pure explicit): Gm={:.2} stripped from G[{}], Go={:.4}, C_dom={:.3e}",
                    oa.name, gm, o, go, c_dom,
                );
            }
        }
        } // end if !has_vca

        // Selective op-amp VCCS Gm cap.
        //
        // High-AOL op-amps (Gm ≈ AOL/r_out, often 200,000 S) make the LU back-
        // substitution produce 400 kV+ values at the op-amp output row, which
        // contaminate neighboring nodes via fill-in BEFORE the post-solve rail
        // clamp fires. Capping Gm to AOL_SUB_MAK / r_out at G-matrix build time
        // bounds the back-sub voltage and eliminates the contamination at zero
        // runtime cost (the cap is baked into the constant G/A/A_neg matrices).
        //
        // The cap is applied selectively — only to op-amps that match Rule D'
        // (precision rectifier / comparator topology: n_plus on a non-zero DC
        // rail AND a diode connects output to inverting input, optionally
        // through a pure-resistor path). Audio-path op-amps keep full AOL so
        // their virtual-ground feedback loops have correct gain.
        //
        // See:
        //   - `opamp_is_sidechain_rectifier` for the classification rule
        //   - `docs/aidocs/DEBUGGING.md` "Precision Rectifier Transient NR"
        //
        // User override: `.model OA(AOL_TRANSIENT_CAP=N)` forces a specific cap.
        for oa in &mna.opamps {
            if oa.n_out_idx == 0 {
                continue;
            }
            let aol_cap = effective_aol_cap(oa, netlist, mna);
            if !aol_cap.is_finite() || oa.aol <= aol_cap {
                continue;
            }
            let gm_full = oa.aol / oa.r_out;
            let gm_capped = aol_cap / oa.r_out;
            let delta = gm_full - gm_capped;
            let o = oa.n_out_idx - 1;
            if o >= n {
                continue;
            }
            if oa.n_plus_idx > 0 && oa.n_plus_idx - 1 < n {
                aug.g[o][oa.n_plus_idx - 1] -= delta;
            }
            if oa.n_minus_idx > 0 && oa.n_minus_idx - 1 < n {
                aug.g[o][oa.n_minus_idx - 1] += delta;
            }
            log::info!(
                "Selective Gm cap on op-amp {}: AOL {:.0} → {:.0} (delta_Gm={:.1} S)",
                oa.name, oa.aol, aol_cap, delta,
            );
        }

        // Now rebuild A/A_neg from the (possibly modified) G matrix
        for i in 0..n {
            for j in 0..n {
                let g = aug.g[i][j];
                let c = aug.c[i][j];
                a_flat[i * n + j] = g + alpha * c;
                a_neg_flat[i * n + j] = if be { alpha * c } else { alpha * c - g };
                a_be_flat[i * n + j] = g + alpha_be * c;
                a_neg_be_flat[i * n + j] = alpha_be * c;
            }
        }
        // Re-zero augmented rows in A_neg and A_neg_be
        if n_aug > n_nodes {
            for i in n_nodes..n_aug {
                if i < n {
                    for j in 0..n {
                        a_neg_flat[i * n + j] = 0.0;
                        a_neg_be_flat[i * n + j] = 0.0;
                    }
                }
            }
        }

        // Flatten G and C (with Gm stripped from IIR op-amp outputs) for codegen constants
        let g_matrix = dk::flatten_matrix(&aug.g, n, n);
        let c_matrix = dk::flatten_matrix(&aug.c, n, n);

        let topology = Topology {
            n,
            n_nodes,
            m,
            num_devices: mna.num_devices,
            n_aug,
            augmented_inductors: true,
            num_linearized_devices: mna.linearized_triodes.len()
                + mna.linearized_bjts.len(),
        };

        let rail_mode = refine_active_set_for_audio_path(
            resolve_opamp_rail_mode(mna, config.opamp_rail_mode),
            mna,
            netlist,
        );
        log::info!(
            "Op-amp rail mode: {} ({})",
            rail_mode.mode,
            rail_mode.reason.as_str()
        );

        // Provisional solver_config. `alpha` and `backward_euler` may still be
        // updated by the auto-BE promotion block below.
        let mut solver_config = SolverConfig {
            sample_rate,
            alpha,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_nodes: config.output_nodes.clone(),
            input_resistance: config.input_resistance,
            oversampling_factor: config.oversampling_factor,
            output_scales: config.output_scales.clone(),
            pot_settle_samples: config.pot_settle_samples,
            backward_euler: be,
            opamp_rail_mode: rail_mode.mode,
            emit_dc_op_recompute: config.emit_dc_op_recompute,
        };

        // Compute S = A^{-1} for Schur complement NR (O(M³) instead of O(N³) per iteration)
        let mut s_flat = invert_flat_matrix(&a_flat, n)?;
        let mut k_flat = if m > 0 {
            compute_k_from_s(&s_flat, &n_v_flat, &n_i_flat, n, m)
        } else {
            Vec::new()
        };

        // Also compute S_be = A_be^{-1} for backward Euler fallback
        let s_be_flat = invert_flat_matrix(&a_be_flat, n)?;
        let k_be_flat = if m > 0 {
            compute_k_from_s(&s_be_flat, &n_v_flat, &n_i_flat, n, m)
        } else {
            Vec::new()
        };

        // Sub-step matrices: trap at 2× the internal rate (alpha_sub = 4/T).
        // Used by ActiveSetBe sub-stepping to damp the discrete-time Nyquist
        // artifact from the pin-and-resolve step. Precomputed so sub-steps
        // are O(N²) matvec, same cost as the normal Schur prediction.
        let alpha_sub = 2.0 * internal_rate * 2.0; // trap alpha at 2x rate
        let mut a_sub_flat = vec![0.0f64; n * n];
        let mut a_neg_sub_flat = vec![0.0f64; n * n];
        for i in 0..n {
            for j in 0..n {
                let g = g_matrix[i * n + j];
                let c = c_matrix[i * n + j];
                a_sub_flat[i * n + j] = g + alpha_sub * c;
                a_neg_sub_flat[i * n + j] = alpha_sub * c - g;
            }
        }
        // Zero algebraic rows in A_neg_sub (same rows as A_neg)
        for vs in &mna.voltage_sources {
            let row = n_nodes + vs.ext_idx;
            if row < n {
                for j in 0..n {
                    a_neg_sub_flat[row * n + j] = 0.0;
                }
            }
        }
        let num_vs_sub = mna.voltage_sources.len();
        for (idx, _) in mna.vcvs_sources.iter().enumerate() {
            let row = n_nodes + num_vs_sub + idx;
            if row < n {
                for j in 0..n {
                    a_neg_sub_flat[row * n + j] = 0.0;
                }
            }
        }
        let num_vcvs_sub = mna.vcvs_sources.len();
        for (idx, _) in mna.ideal_transformers.iter().enumerate() {
            let row = n_nodes + num_vs_sub + num_vcvs_sub + idx;
            if row < n {
                for j in 0..n {
                    a_neg_sub_flat[row * n + j] = 0.0;
                }
            }
        }
        let s_sub_flat = invert_flat_matrix(&a_sub_flat, n)?;
        let k_sub_flat = if m > 0 {
            compute_k_from_s(&s_sub_flat, &n_v_flat, &n_i_flat, n, m)
        } else {
            Vec::new()
        };

        // Compute spectral radius of S * A_neg to detect Schur instability.
        // When rho(S * A_neg) > 1, the trapezoidal feedback v_pred = S*(A_neg*v_prev + ...)
        // amplifies errors exponentially. Route to full LU NR instead.
        let mut spectral_radius_s_aneg = if n > 0 && !s_flat.is_empty() {
            let mut x = vec![1.0 / (n as f64).sqrt(); n];
            let mut rho = 0.0f64;
            for _ in 0..100 {
                // y = S * (A_neg * x)
                let mut ax = vec![0.0; n];
                for i in 0..n {
                    for j in 0..n {
                        ax[i] += a_neg_flat[i * n + j] * x[j];
                    }
                }
                let mut y = vec![0.0; n];
                for i in 0..n {
                    for j in 0..n {
                        y[i] += s_flat[i * n + j] * ax[j];
                    }
                }
                let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
                if norm < 1e-30 {
                    break;
                }
                rho = norm / x.iter().map(|v| v * v).sum::<f64>().sqrt();
                x.fill(0.0);
                for (i, yi) in y.iter().enumerate() {
                    x[i] = yi / norm;
                }
            }
            if rho > 0.99 {
                log::info!(
                    "Nodal: spectral_radius(S*A_neg) = {:.4} (marginally stable; Schur used when K well-conditioned)",
                    rho
                );
            }
            rho
        } else {
            0.0
        };

        // Auto-BE promotion for the nodal path.
        //
        // The trap propagation operator `S*A_neg` is A-stable but not
        // L-stable: on stiff nodal circuits its spectral radius can exceed 1
        // and seed a stationary Nyquist-rate `(-1)^n` limit cycle in
        // `v_prev`. Inaudible above SR/2 but large enough (observed A2/A1 ≈
        // 1.73 on pipe-shouter small-signal) to wreck narrow post-circuit
        // EQs and inflate RMS meters. BE is L-stable and always has ρ ≤ 1.
        //
        // The BE matrices are already built above as the transient BE
        // fallback (`a_be_flat`, `a_neg_be_flat`, `s_be_flat`, `k_be_flat`,
        // `rhs_const_be`). When trap is unstable we clone them into the
        // primary slot and flip `alpha`/`solver_config.backward_euler` so
        // every downstream emitter picks BE formulas. `config.force_trap`
        // is the escape hatch for bisection only — trap on a circuit where
        // the auto-detector fires produces a real Nyquist-rate artifact.
        //
        // Threshold 1.002 matches `schur_unstable` in `nodal_emitter.rs`.
        // Strict `> 1.0` false-fires on trivial passive networks where
        // power iteration converges to exactly 1.0 plus float noise (pure
        // RC lowpass, etc.) and would promote them unnecessarily.
        const BE_PROMOTION_THRESHOLD: f64 = 1.002;
        if !be && !config.force_trap && spectral_radius_s_aneg > BE_PROMOTION_THRESHOLD {
            log::warn!(
                "Nodal: auto-enabling backward Euler — spectral_radius(S*A_neg) = \
                 {:.4} > {:.3} under trapezoidal (dominant eigenmode would grow \
                 unboundedly — Nyquist-rate limit cycle in v_prev). BE is L-stable. \
                 Override with --force-trap only to reproduce legacy trap output.",
                spectral_radius_s_aneg, BE_PROMOTION_THRESHOLD
            );
            alpha = alpha_be;
            a_flat = a_be_flat.clone();
            a_neg_flat = a_neg_be_flat.clone();
            rhs_const = rhs_const_be.clone();
            s_flat = s_be_flat.clone();
            k_flat = k_be_flat.clone();
            solver_config.backward_euler = true;
            solver_config.alpha = alpha;
            // Recompute rho on BE matrices. BE is L-stable so this must be
            // ≤ 1; a violation would mean the BE matrix builder has a bug,
            // which we flag loudly rather than silently pushing into the
            // emitter.
            let new_rho = if n > 0 && !s_flat.is_empty() {
                let mut x = vec![1.0 / (n as f64).sqrt(); n];
                let mut rho = 0.0f64;
                for _ in 0..100 {
                    let mut ax = vec![0.0; n];
                    for i in 0..n {
                        for j in 0..n {
                            ax[i] += a_neg_flat[i * n + j] * x[j];
                        }
                    }
                    let mut y = vec![0.0; n];
                    for i in 0..n {
                        for j in 0..n {
                            y[i] += s_flat[i * n + j] * ax[j];
                        }
                    }
                    let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
                    if norm < 1e-30 {
                        break;
                    }
                    rho = norm / x.iter().map(|v| v * v).sum::<f64>().sqrt();
                    x.fill(0.0);
                    for (i, yi) in y.iter().enumerate() {
                        x[i] = yi / norm;
                    }
                }
                rho
            } else {
                0.0
            };
            if new_rho > 1.0 + 1e-6 {
                log::error!(
                    "Nodal: BE matrices still have spectral_radius(S_be*A_neg_be) = \
                     {:.4} > 1 after auto-promotion. BE is L-stable by construction \
                     — matrix builder has a stamping bug.",
                    new_rho
                );
            }
            spectral_radius_s_aneg = new_rho;
        }
        let _ = alpha_be;

        let matrices = Matrices {
            s: s_flat,
            k: k_flat,
            a_neg: a_neg_flat,
            n_v: n_v_flat,
            n_i: n_i_flat,
            rhs_const,
            g_matrix,
            c_matrix,
            a_matrix: a_flat,
            a_matrix_be: a_be_flat,
            a_neg_be: a_neg_be_flat,
            rhs_const_be,
            s_be: s_be_flat,
            k_be: k_be_flat,
            spectral_radius_s_aneg,
            s_sub: s_sub_flat,
            k_sub: k_sub_flat,
            a_neg_sub: a_neg_sub_flat,
        };

        // Run DC OP (operates on the original MNA system — which still has Gm for correct DC point)
        // NOTE: The MNA's G matrix still has Gm stamped (we only stripped from aug.g which is a copy).
        // DC OP uses mna.g directly, so it sees the full Gm and computes the correct bias point.
        let dc_op_config = DcOpConfig {
            input_node: config.input_node,
            input_resistance: config.input_resistance,
            ..DcOpConfig::default()
        };
        // Build device info with MNA so FA reductions are reflected in dimensions
        let mut device_slots = Self::build_device_info_with_mna(netlist, Some(mna))?;

        let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);
        let dc_op_truncated = &dc_result.v_node[..n_aug.min(dc_result.v_node.len())];
        let has_dc_op = dc_op_truncated.iter().any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_op_method = format!("{:?}", dc_result.method);
        let dc_op_iterations = dc_result.iterations;
        let dc_nl_currents = dc_result.i_nl.clone();
        let has_dc_sources = !mna.voltage_sources.is_empty() || !mna.current_sources.is_empty();

        // Resize DC OP to n_nodal dimension. Do NOT clamp op-amp outputs
        // to supply rails here — the emitted per-sample active-set resolve
        // already handles rail violations at runtime, and pre-clamping the
        // stored DC_OP creates a v_prev / i_nl_prev inconsistency (v_out
        // clamped but `dc_nl_currents` comes from the unclamped solve, so
        // the downstream diode states encoded in `i_nl` don't match the
        // clamped nodes) that slowly drifts the state over thousands of
        // samples before NR blows up (observed 4kbuscomp failure: ~2300
        // stable samples then 1e27 V explosion).
        let mut dc_operating_point = dc_result.v_node.clone();
        dc_operating_point.resize(n, 0.0);
        Self::resolve_mosfet_nodes(&mut device_slots, mna);

        // Sparsity analysis (K is now computed for Schur complement NR)
        let lu_sparsity = if m > 0 {
            // Compute G_aug = A - N_i*J_dev*N_v sparsity pattern
            let g_aug_pattern = lu::compute_g_aug_pattern(
                &matrices.a_matrix,
                &matrices.n_i,
                &matrices.n_v,
                n,
                m,
                &device_slots,
            );
            let g_aug_nnz: usize = g_aug_pattern.iter().map(|r| r.len()).sum();
            let density = g_aug_nnz as f64 / (n * n) as f64;
            log::info!(
                "Sparse LU: G_aug pattern has {} nonzeros out of {} ({:.1}% density)",
                g_aug_nnz,
                n * n,
                density * 100.0
            );
            // Only use sparse LU if matrix is sufficiently sparse (< 40% density)
            // and large enough to benefit (N >= 8)
            if density < 0.4 && n >= 8 {
                let elim_order = lu::amd_ordering(&g_aug_pattern, n);
                let row_swaps = lu::find_row_swaps(&g_aug_pattern, &elim_order, n);
                let lu_plan = lu::symbolic_lu(&g_aug_pattern, &elim_order, &row_swaps, n);
                Some(lu_plan)
            } else {
                log::info!(
                    "Sparse LU: skipping (density {:.1}%, N={})",
                    density * 100.0,
                    n
                );
                None
            }
        } else {
            None
        };

        let sparsity = SparseInfo {
            a_neg: analyze_matrix_sparsity(&matrices.a_neg, n, n),
            n_v: analyze_matrix_sparsity(&matrices.n_v, m, n),
            n_i: analyze_matrix_sparsity(&matrices.n_i, n, m),
            k: analyze_matrix_sparsity(&matrices.k, m, m),
            lu: lu_sparsity,
        };

        let named_constants = build_named_constants(mna, topology.n_nodes);
        let runtime_sources: Vec<RuntimeSourceIR> = mna
            .runtime_sources
            .iter()
            .map(|rt| RuntimeSourceIR {
                vs_name: rt.vs_name.clone(),
                field_name: rt.field_name.clone(),
                vs_row: rt.vs_row,
            })
            .collect();

        Ok(CircuitIR {
            metadata,
            topology,
            solver_mode: SolverMode::Nodal,
            solver_config,
            matrices,
            dc_operating_point,
            device_slots,
            has_dc_sources,
            has_dc_op,
            dc_nl_currents,
            dc_op_converged,
            dc_op_method,
            dc_op_iterations,
            dc_block: config.dc_block,
            inductors: Vec::new(), // no companion model
            coupled_inductors: Vec::new(),
            transformer_groups: Vec::new(),
            pots: mna
                .pots
                .iter()
                .map(|p| PotentiometerIR {
                    su: Vec::new(), // not used in nodal (no Sherman-Morrison)
                    usu: 0.0,
                    g_nominal: p.g_nominal,
                    nv_su: Vec::new(),
                    u_ni: Vec::new(),
                    node_p: p.node_p,
                    node_q: p.node_q,
                    min_resistance: p.min_resistance,
                    max_resistance: p.max_resistance,
                    grounded: p.grounded,
                    runtime_field: p.runtime_field.clone(),
                })
                .collect(),
            wiper_groups: mna
                .wiper_groups
                .iter()
                .map(|wg| WiperGroupIR {
                    cw_pot_index: wg.cw_pot_index,
                    ccw_pot_index: wg.ccw_pot_index,
                    total_resistance: wg.total_resistance,
                    default_position: wg.default_position,
                    label: wg.label.clone(),
                })
                .collect(),
            gang_groups: mna
                .gang_groups
                .iter()
                .map(|gg| GangGroupIR {
                    label: gg.label.clone(),
                    pot_members: gg
                        .pot_members
                        .iter()
                        .map(|&(pot_idx, inverted)| GangPotMemberIR {
                            pot_index: pot_idx,
                            min_resistance: mna.pots[pot_idx].min_resistance,
                            max_resistance: mna.pots[pot_idx].max_resistance,
                            inverted,
                        })
                        .collect(),
                    wiper_members: gg
                        .wiper_members
                        .iter()
                        .map(|&(wg_idx, inverted)| GangWiperMemberIR {
                            wiper_group_index: wg_idx,
                            total_resistance: mna.wiper_groups[wg_idx].total_resistance,
                            inverted,
                        })
                        .collect(),
                    default_position: gg.default_position,
                })
                .collect(),
            switches: {
                // Build inductor name → augmented row mapping for switch L components.
                // In augmented MNA, each inductor's L value lives on the C matrix diagonal
                // at row n_aug + offset (not at circuit node rows).
                let mut inductor_aug_rows: std::collections::HashMap<String, usize> =
                    std::collections::HashMap::new();
                let mut var_idx = n_aug; // Original n_aug in the full system
                for ind in &mna.inductors {
                    inductor_aug_rows.insert(ind.name.to_ascii_uppercase(), var_idx);
                    var_idx += 1;
                }
                for ci in &mna.coupled_inductors {
                    inductor_aug_rows.insert(ci.l1_name.to_ascii_uppercase(), var_idx);
                    inductor_aug_rows.insert(ci.l2_name.to_ascii_uppercase(), var_idx + 1);
                    var_idx += 2;
                }
                for group in &mna.transformer_groups {
                    for (widx, name) in group.winding_names.iter().enumerate() {
                        inductor_aug_rows.insert(name.to_ascii_uppercase(), var_idx + widx);
                    }
                    var_idx += group.num_windings;
                }
                // Inductor augmented row indices are used directly at the N dimension.

                mna.switches
                    .iter()
                    .enumerate()
                    .map(|(idx, sw)| {
                        // Collect inductor names in this switch for mutual lookup
                        let switch_inductor_names: std::collections::HashSet<String> = sw
                            .components
                            .iter()
                            .filter(|c| c.component_type == 'L')
                            .map(|c| c.name.to_ascii_uppercase())
                            .collect();

                        // Build mutual entries for coupled pairs where at least one winding is in this switch
                        let mut mutual_entries = Vec::new();
                        for ci in &mna.coupled_inductors {
                            let l1 = ci.l1_name.to_ascii_uppercase();
                            let l2 = ci.l2_name.to_ascii_uppercase();
                            if switch_inductor_names.contains(&l1) || switch_inductor_names.contains(&l2) {
                                if let (Some(&ra), Some(&rb)) = (
                                    inductor_aug_rows.get(&l1),
                                    inductor_aug_rows.get(&l2),
                                ) {
                                    mutual_entries.push(SwitchMutualEntry {
                                        row_a: ra,
                                        row_b: rb,
                                        coupling: ci.coupling,
                                    });
                                }
                            }
                        }
                        for group in &mna.transformer_groups {
                            for i in 0..group.num_windings {
                                for j in (i + 1)..group.num_windings {
                                    let ni = group.winding_names[i].to_ascii_uppercase();
                                    let nj = group.winding_names[j].to_ascii_uppercase();
                                    if switch_inductor_names.contains(&ni) || switch_inductor_names.contains(&nj) {
                                        if let (Some(&ra), Some(&rb)) = (
                                            inductor_aug_rows.get(&ni),
                                            inductor_aug_rows.get(&nj),
                                        ) {
                                            mutual_entries.push(SwitchMutualEntry {
                                                row_a: ra,
                                                row_b: rb,
                                                coupling: group.coupling_matrix[i][j],
                                            });
                                        }
                                    }
                                }
                            }
                        }

                        SwitchIR {
                            index: idx,
                            components: sw
                                .components
                                .iter()
                                .map(|comp| {
                                    let augmented_row = if comp.component_type == 'L' {
                                        inductor_aug_rows
                                            .get(&comp.name.to_ascii_uppercase())
                                            .copied()
                                    } else {
                                        None
                                    };
                                    SwitchComponentIR {
                                        name: comp.name.clone(),
                                        component_type: comp.component_type,
                                        node_p: comp.node_p,
                                        node_q: comp.node_q,
                                        nominal_value: comp.nominal_value,
                                        inductor_index: None,
                                        augmented_row,
                                        coupled_inductor_index: None,
                                        coupled_winding: None,
                                    }
                                })
                                .collect(),
                            positions: sw.positions.clone(),
                            num_positions: sw.positions.len(),
                            mutual_entries,
                        }
                    })
                    .collect()
            },
            saturating_inductors: {
                // Build list of inductors with ISAT (iron-core saturation).
                // Reuse the same augmented row mapping as switches.
                let mut sat_inds = Vec::new();
                let mut sat_var_idx = n_aug;
                for (i, ind) in mna.inductors.iter().enumerate() {
                    if let Some(isat) = ind.isat {
                        sat_inds.push(SaturatingInductorIR {
                            name: ind.name.clone(),
                            l0: ind.value,
                            isat,
                            aug_row: sat_var_idx,
                            inductor_index: i,
                        });
                    }
                    sat_var_idx += 1;
                }
                sat_inds
            },
            saturating_coupled: {
                // Coupled inductor pairs with ISAT on either winding.
                // Augmented rows: uncoupled inductors are at n_aug..n_aug+n_uncoupled,
                // coupled pairs start at n_aug+n_uncoupled, 2 rows each.
                let ci_base = n_aug + mna.inductors.len();
                let mut sat_ci = Vec::new();
                for (i, ci) in mna.coupled_inductors.iter().enumerate() {
                    if ci.l1_isat.is_some() || ci.l2_isat.is_some() {
                        // Both windings must have ISAT for the coupled model.
                        // If only one has ISAT, use the other's L0 as its "isat"
                        // (effectively infinite — no saturation on that winding).
                        let l1_isat = ci.l1_isat.unwrap_or(1e6);
                        let l2_isat = ci.l2_isat.unwrap_or(1e6);
                        sat_ci.push(SaturatingCoupledInductorIR {
                            name: ci.name.clone(),
                            l1_name: ci.l1_name.clone(),
                            l2_name: ci.l2_name.clone(),
                            l1_l0: ci.l1_value,
                            l2_l0: ci.l2_value,
                            l1_isat,
                            l2_isat,
                            coupling: ci.coupling,
                            k1: ci_base + i * 2,
                            k2: ci_base + i * 2 + 1,
                        });
                    }
                }
                sat_ci
            },
            saturating_xfmr_groups: {
                // Transformer groups (3+ windings) with ISAT on any winding.
                // Aug rows: after uncoupled (n_uncoupled) + coupled pairs (n_coupled*2).
                let xfmr_base = n_aug + mna.inductors.len() + mna.coupled_inductors.len() * 2;
                let mut sat_xfmr = Vec::new();
                let mut xfmr_offset = 0usize;
                for (i, group) in mna.transformer_groups.iter().enumerate() {
                    if group.winding_isats.iter().any(|isat| isat.is_some()) {
                        let w = group.num_windings;
                        let aug_rows: Vec<usize> = (0..w).map(|wi| xfmr_base + xfmr_offset + wi).collect();
                        let isats: Vec<f64> = group.winding_isats.iter()
                            .map(|isat| isat.unwrap_or(1e6))
                            .collect();
                        let coupling_flat: Vec<f64> = group.coupling_matrix.iter()
                            .flat_map(|row| row.iter().copied())
                            .collect();
                        sat_xfmr.push(SaturatingTransformerGroupIR {
                            name: group.name.clone(),
                            num_windings: w,
                            winding_names: group.winding_names.clone(),
                            l0s: group.inductances.clone(),
                            isats,
                            coupling_flat,
                            aug_rows,
                        });
                    }
                    let _ = i;
                    xfmr_offset += group.num_windings;
                }
                sat_xfmr
            },
            opamps: mna
                .opamps
                .iter()
                .filter(|oa| {
                    (oa.vcc.is_finite() || oa.vee.is_finite() || oa.sr.is_finite())
                        && oa.n_out_idx > 0
                })
                .map(|oa| opamp_ir_from_info(oa, netlist, mna))
                .collect(),
            sparsity,
            opamp_iir: opamp_iir_data,
            noise: NoiseIR {
                mode: config.noise_mode,
                master_seed: config.noise_master_seed,
                thermal_sources: if config.noise_mode.includes_thermal() {
                    collect_thermal_noise_sources(netlist, mna)
                } else {
                    Vec::new()
                },
                shot_sources: if config.noise_mode.includes_shot() {
                    collect_shot_noise_sources(mna)
                } else {
                    Vec::new()
                },
                flicker_sources: if config.noise_mode.includes_full() {
                    collect_flicker_noise_sources(netlist, mna)
                } else {
                    Vec::new()
                },
            },
            named_constants,
            runtime_sources,
        })
    }

    /// Build device slot map and resolve per-device parameters from netlist.
    ///
    /// # Errors
    /// Returns `CodegenError::InvalidConfig` if any device model parameter is non-positive or non-finite.
    /// Detect BJTs that are forward-active at the DC operating point.
    ///
    /// Returns the names (uppercased) of BJTs with Vbc < -0.5V that can be
    /// modeled as 1D (Vbe→Ic only), reducing M by 1 each.
    ///
    /// Call this BEFORE building the final MNA/kernel. If non-empty, rebuild
    /// MNA with `from_netlist_forward_active()` and kernel before calling `from_kernel()`.
    pub fn detect_forward_active_bjts(
        mna: &crate::mna::MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
    ) -> std::collections::HashSet<String> {
        use crate::dc_op::{self, DcOpConfig};

        let device_slots = Self::build_device_info(netlist).unwrap_or_default();
        if device_slots.is_empty() {
            return std::collections::HashSet::new();
        }

        let dc_op_config = DcOpConfig {
            tolerance: config.dc_op_tolerance,
            max_iterations: config.dc_op_max_iterations,
            input_node: config.input_node,
            input_resistance: config.input_resistance,
            ..DcOpConfig::default()
        };
        let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);

        let mut forward_active = std::collections::HashSet::new();
        for (slot_idx, slot) in device_slots.iter().enumerate() {
            if slot.device_type == DeviceType::Bjt && slot_idx < mna.nonlinear_devices.len() {
                let bp = if let DeviceParams::Bjt(bp) = &slot.params {
                    bp
                } else {
                    continue;
                };
                let dev = &mna.nonlinear_devices[slot_idx];
                let nc = dev.node_indices[0];
                let nb = dev.node_indices[1];
                let v_c = if nc > 0 && nc - 1 < dc_result.v_node.len() {
                    dc_result.v_node[nc - 1]
                } else {
                    0.0
                };
                let v_b = if nb > 0 && nb - 1 < dc_result.v_node.len() {
                    dc_result.v_node[nb - 1]
                } else {
                    0.0
                };
                let vbc = v_b - v_c;
                let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                let vbc_eff = sign * vbc;
                // Forward-active: allow parasitics and GP models.
                // 1D FA ignores parasitics (small for forward-active: Ib*RB << Vbe).
                // GP qb(Vbc) is ~constant when Vbc is deeply reverse-biased.
                // Threshold -0.5V provides adequate margin for audio-level signals
                // (typical stage Vce margin is 0.85V in cascaded topologies).
                if vbc_eff < -0.5 {
                    let name = dev.name.to_ascii_uppercase();
                    log::info!(
                        "BJT '{}' forward-active (Vbc={:.3}V). Using 1D model.",
                        name,
                        vbc_eff
                    );
                    forward_active.insert(name);
                }
            }
        }
        forward_active
    }

    /// Phase 1b grid-off pentode detection.
    ///
    /// Runs DC-OP on the provided MNA system and inspects each pentode's
    /// converged `Vgk` and `Vg2k`. Pentodes with `Vgk < -(vgk_onset + 0.5)`
    /// across the operating point (i.e. well below grid cutoff) qualify
    /// for grid-off reduction: `Ig1` is identically zero and `Vg2k` is
    /// approximately held constant by external bypass caps, so the NR
    /// block can drop from 3D to 2D.
    ///
    /// Returns a `HashMap<String, f64>` mapping pentode name (uppercased)
    /// to the DC-OP-converged `Vg2k` value that should be frozen in the
    /// reduced device. The caller uses the map to:
    ///
    /// 1. Collect the set of names and pass them to
    ///    [`MnaSystem::from_netlist_with_grid_off`] to rebuild MNA with
    ///    `dimension: 2` pentode slots
    /// 2. Build `device_slots` via [`build_device_info_with_mna`] — that
    ///    function will detect the MNA's reduced dimension and set
    ///    `TubeParams.kind = SharpPentodeGridOff` automatically
    /// 3. Iterate the returned slots and write the per-slot
    ///    `vg2k_frozen` value from this map
    ///
    /// Mirrors [`detect_forward_active_bjts`] for the BJT case.
    ///
    /// **Auto-detection only.** The `--tube-grid-fa on/off` CLI overrides
    /// bypass this detection (force-on creates a map with every pentode,
    /// force-off returns an empty map regardless of bias).
    /// Detect pentodes eligible for grid-off dimension reduction.
    ///
    /// When `force_all` is true, every non-variable-mu pentode is marked as
    /// grid-off (the `--tube-grid-fa on` escape hatch), bypassing the normal
    /// `Vgk < cutoff_threshold` check.  The DC-OP Vg2k value is still read
    /// and used as the frozen screen voltage.
    pub fn detect_grid_off_pentodes(
        mna: &crate::mna::MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        force_all: bool,
    ) -> std::collections::HashMap<String, f64> {
        use crate::dc_op::{self, DcOpConfig};

        // Must use build_device_info_with_mna so that FA-reduced BJTs (dim=1) produce
        // the correct start_idx values matching mna.m. Using build_device_info(netlist)
        // without MNA gives unreduced dimensions, causing v_nl OOB when FA reduction
        // has already happened (e.g. Q1 reduced 2D→1D shifts all subsequent start_idx).
        let device_slots = Self::build_device_info_with_mna(netlist, Some(mna)).unwrap_or_default();
        if device_slots.is_empty() {
            return std::collections::HashMap::new();
        }

        let dc_op_config = DcOpConfig {
            tolerance: config.dc_op_tolerance,
            max_iterations: config.dc_op_max_iterations,
            input_node: config.input_node,
            input_resistance: config.input_resistance,
            ..DcOpConfig::default()
        };
        let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);

        let mut grid_off = std::collections::HashMap::new();
        for (slot_idx, slot) in device_slots.iter().enumerate() {
            // Grid-off only applies to pentodes in the full-3D state;
            // skip non-tubes and slots already reduced to 2D.
            if slot.device_type != DeviceType::Tube || slot.dimension != 3 {
                continue;
            }
            let tp = match &slot.params {
                DeviceParams::Tube(tp) if tp.is_pentode() => tp,
                _ => continue,
            };
            // Variable-mu pentodes (6K7, EF89) are excluded from grid-off —
            // they're designed specifically for continuous bias changes
            // under sidechain control, and freezing Vg2k contradicts that
            // usage pattern. Schema `validate()` also rejects this combo.
            if tp.is_variable_mu() {
                continue;
            }
            if slot_idx >= mna.nonlinear_devices.len() {
                continue;
            }
            let dev = &mna.nonlinear_devices[slot_idx];
            // Pentode node order (from `categorize_element` in mna.rs):
            // [plate, grid, cathode, screen] with optional [, suppressor].
            if dev.node_indices.len() < 4 {
                continue;
            }
            let n_plate = dev.node_indices[0];
            let n_grid = dev.node_indices[1];
            let n_cathode = dev.node_indices[2];
            let n_screen = dev.node_indices[3];
            let v_at = |n: usize| -> f64 {
                if n > 0 && n - 1 < dc_result.v_node.len() {
                    dc_result.v_node[n - 1]
                } else {
                    0.0
                }
            };
            let v_grid = v_at(n_grid);
            let v_cathode = v_at(n_cathode);
            let v_screen = v_at(n_screen);
            let vgk = v_grid - v_cathode;
            let vg2k = v_screen - v_cathode;
            // Grid-off threshold: Vgk must be below -(vgk_onset + 0.5) so
            // there's a safety margin around the Leach grid-current onset.
            // `vgk_onset` is the positive voltage at which grid conduction
            // begins; grid-off requires Vgk to be well-negative.
            let cutoff_threshold = -(tp.vgk_onset + 0.5);
            // Additionally require a plausible positive Vg2k — a frozen
            // value near zero would suggest DC-OP didn't actually solve
            // for the screen supply.
            let plate = v_at(n_plate);
            let passes_threshold = force_all || (vgk < cutoff_threshold && vg2k > 1.0);
            if passes_threshold {
                let name = dev.name.to_ascii_uppercase();
                log::info!(
                    "Pentode '{}' grid-off{}(Vgk={:.3}V, Vg2k={:.3}V, Vpk={:.3}V). Using 2D model.",
                    name,
                    if force_all { " (forced) " } else { " " },
                    vgk,
                    vg2k,
                    plate - v_cathode
                );
                grid_off.insert(name, vg2k);
            }
        }
        grid_off
    }

    pub fn build_device_info(netlist: &Netlist) -> Result<Vec<DeviceSlot>, CodegenError> {
        Self::build_device_info_with_mna(netlist, None)
    }

    /// Build device info, optionally using MNA device dimensions (for forward-active BJTs).
    pub fn build_device_info_with_mna(
        netlist: &Netlist,
        mna: Option<&crate::mna::MnaSystem>,
    ) -> Result<Vec<DeviceSlot>, CodegenError> {
        let mut slots = Vec::new();
        let mut dim_offset = 0;
        let mut nl_dev_idx = 0; // tracks position in mna.nonlinear_devices

        for elem in &netlist.elements {
            match elem {
                Element::Diode { model, .. } => {
                    let params = Self::resolve_diode_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Diode,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Diode(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                    });
                    dim_offset += 1;
                    nl_dev_idx += 1;
                }
                Element::Bjt { name, model, .. } => {
                    // Skip linearized BJTs — they're not in the nonlinear system
                    let is_linearized = mna.is_some_and(|m| {
                        m.linearized_bjts
                            .iter()
                            .any(|l| l.name.eq_ignore_ascii_case(name))
                    });
                    if is_linearized {
                        continue; // Don't create a DeviceSlot, don't increment nl_dev_idx
                    }
                    let params = Self::resolve_bjt_params(netlist, model)?;
                    // Check if MNA has this BJT as forward-active (1D)
                    let is_fa = mna.is_some_and(|m| {
                        nl_dev_idx < m.nonlinear_devices.len()
                            && m.nonlinear_devices[nl_dev_idx].device_type
                                == crate::mna::NonlinearDeviceType::BjtForwardActive
                    });
                    let (dev_type, dim) = if is_fa {
                        (DeviceType::BjtForwardActive, 1)
                    } else {
                        (DeviceType::Bjt, 2)
                    };
                    slots.push(DeviceSlot {
                        device_type: dev_type,
                        start_idx: dim_offset,
                        dimension: dim,
                        params: DeviceParams::Bjt(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                    });
                    dim_offset += dim;
                    nl_dev_idx += 1;
                }
                Element::Jfet { model, .. } => {
                    let params = Self::resolve_jfet_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Jfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Jfet(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Triode { name, model, .. } => {
                    // Skip linearized triodes — they're not in the nonlinear system
                    let is_linearized = mna.is_some_and(|m| {
                        m.linearized_triodes
                            .iter()
                            .any(|l| l.name.eq_ignore_ascii_case(name))
                    });
                    if is_linearized {
                        continue; // Don't create a DeviceSlot, don't increment nl_dev_idx
                    }
                    let params = Self::resolve_tube_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Tube,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Tube(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Pentode { model, .. } => {
                    let mut params = Self::resolve_pentode_params(netlist, model)?;
                    // Check if MNA has this pentode as grid-off (2D reduced).
                    // Phase 1b: after DC-OP detects Vgk < cutoff, the MNA is
                    // rebuilt via `from_netlist_with_grid_off` which stamps
                    // dimension=2 for the named pentodes. Here we reflect that
                    // back into `TubeParams.kind` so codegen dispatches to
                    // `*_pentode_grid_off` helpers.
                    let is_grid_off = mna.is_some_and(|m| {
                        nl_dev_idx < m.nonlinear_devices.len()
                            && m.nonlinear_devices[nl_dev_idx].device_type
                                == crate::mna::NonlinearDeviceType::Tube
                            && m.nonlinear_devices[nl_dev_idx].dimension == 2
                            && m.nonlinear_devices[nl_dev_idx].nodes.len() >= 4
                    });
                    let dim = if is_grid_off { 2 } else { 3 };
                    if is_grid_off {
                        params.kind = crate::device_types::TubeKind::SharpPentodeGridOff;
                    }
                    let vg2k_frozen = if is_grid_off {
                        mna.and_then(|m| {
                            if nl_dev_idx < m.nonlinear_devices.len() {
                                let v = m.nonlinear_devices[nl_dev_idx].vg2k_frozen;
                                if v.abs() > 1e-15 { Some(v) } else { None }
                            } else {
                                None
                            }
                        })
                        .unwrap_or(0.0)
                    } else {
                        0.0
                    };
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Tube,
                        start_idx: dim_offset,
                        dimension: dim,
                        params: DeviceParams::Tube(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen,
                    });
                    dim_offset += dim;
                    nl_dev_idx += 1;
                }
                Element::Mosfet { model, .. } => {
                    let params = Self::resolve_mosfet_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Mosfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Mosfet(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Vca { model, .. } => {
                    let params = Self::resolve_vca_params(netlist, model)?;
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Vca,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Vca(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                _ => {}
            }
        }

        // Mark BJTs with MNA-level internal nodes
        if let Some(m) = mna {
            for slot in &mut slots {
                if slot.device_type == DeviceType::Bjt
                    && m.bjt_internal_nodes
                        .iter()
                        .any(|n| n.start_idx == slot.start_idx)
                {
                    slot.has_internal_mna_nodes = true;
                }
            }
        }

        Ok(slots)
    }

    /// Resolve MOSFET source/bulk node indices from MNA nonlinear device info.
    ///
    /// Called after `build_device_info` to populate `source_node` and `bulk_node`
    /// fields in MosfetParams, which are needed for body effect (GAMMA/PHI).
    fn resolve_mosfet_nodes(slots: &mut [DeviceSlot], mna: &MnaSystem) {
        let mut mosfet_idx = 0;
        for slot in slots.iter_mut() {
            if let DeviceParams::Mosfet(ref mut mp) = slot.params {
                if mp.has_body_effect() {
                    // Find the matching MOSFET in MNA nonlinear_devices
                    for dev in &mna.nonlinear_devices {
                        if dev.device_type == crate::mna::NonlinearDeviceType::Mosfet
                            && dev.start_idx == slot.start_idx
                        {
                            // node_indices: [drain, gate, source, bulk]
                            // node_indices are 1-based (0 = ground)
                            // For the N-dimensional system, node index i maps to v[i-1]
                            mp.source_node = dev.node_indices[2];
                            mp.bulk_node = dev.node_indices[3];
                            break;
                        }
                    }
                }
                mosfet_idx += 1;
            }
        }
        let _ = mosfet_idx; // suppress unused warning
    }

    /// Resolve diode model parameters from the netlist, with validation.
    ///
    /// Resolution order: explicit `.model` param → catalog → generic default.
    fn resolve_diode_params(netlist: &Netlist, model: &str) -> Result<DiodeParams, CodegenError> {
        let vt = melange_primitives::VT_ROOM;
        let cat = melange_devices::catalog::diodes::lookup(model);
        let is = Self::lookup_model_param(netlist, model, "IS")
            .or_else(|| cat.map(|c| c.is))
            .unwrap_or(2.52e-9);
        let n = Self::lookup_model_param(netlist, model, "N")
            .or_else(|| cat.map(|c| c.n))
            .unwrap_or(1.0);

        validate_positive_finite(is, "diode model IS")?;
        validate_positive_finite(n, "diode model N")?;

        // Junction capacitance (optional, default 0.0)
        let cjo = Self::lookup_model_param(netlist, model, "CJO").unwrap_or(0.0);
        if cjo < 0.0 || !cjo.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model CJO must be non-negative and finite, got {cjo}"
            )));
        }

        // Series resistance (optional, default 0.0)
        let rs = Self::lookup_model_param(netlist, model, "RS").unwrap_or(0.0);
        if rs < 0.0 || !rs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model RS must be non-negative and finite, got {rs}"
            )));
        }

        // Reverse breakdown voltage (optional, default infinity = disabled)
        let bv = Self::lookup_model_param(netlist, model, "BV").unwrap_or(f64::INFINITY);
        if bv.is_finite() {
            validate_positive_finite(bv, "diode model BV")?;
        }

        // Reverse breakdown current (optional, default 1e-10)
        let ibv = Self::lookup_model_param(netlist, model, "IBV").unwrap_or(1e-10);
        if ibv.is_finite() {
            validate_positive_finite(ibv, "diode model IBV")?;
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &["IS", "N", "CJO", "RS", "BV", "IBV", "KF", "AF"],
        );

        Ok(DiodeParams {
            is,
            n_vt: n * vt,
            cjo,
            rs,
            bv,
            ibv,
        })
    }

    /// Resolve BJT model parameters from the netlist, with validation.
    ///
    /// Gummel-Poon parameters (VAF, VAR, IKF, IKR) default to infinity,
    /// which collapses qb→1.0, giving exact Ebers-Moll behavior.
    fn resolve_bjt_params(netlist: &Netlist, model: &str) -> Result<BjtParams, CodegenError> {
        let cat = melange_devices::catalog::bjts::lookup(model);
        let vt = Self::lookup_model_param(netlist, model, "VT")
            .or_else(|| cat.map(|c| c.vt))
            .unwrap_or(melange_primitives::VT_ROOM);
        let is = Self::lookup_model_param(netlist, model, "IS")
            .or_else(|| cat.map(|c| c.is))
            .unwrap_or(1.26e-14);
        let beta_f = Self::lookup_model_param(netlist, model, "BF")
            .or_else(|| cat.map(|c| c.beta_f))
            .unwrap_or(200.0);
        let beta_r = Self::lookup_model_param(netlist, model, "BR")
            .or_else(|| cat.map(|c| c.beta_r))
            .unwrap_or(3.0);

        validate_positive_finite(is, "BJT model IS")?;
        validate_positive_finite(vt, "BJT model VT")?;
        validate_positive_finite(beta_f, "BJT model BF")?;
        validate_positive_finite(beta_r, "BJT model BR")?;

        let is_pnp = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PNP"))
            .unwrap_or(cat.map(|c| c.is_pnp).unwrap_or(false));

        // Gummel-Poon parameters (default to infinity = pure Ebers-Moll)
        let vaf = Self::lookup_model_param(netlist, model, "VAF")
            .or_else(|| Self::lookup_model_param(netlist, model, "VA"))
            .or_else(|| cat.map(|c| c.vaf))
            .unwrap_or(f64::INFINITY);
        let var = Self::lookup_model_param(netlist, model, "VAR")
            .or_else(|| Self::lookup_model_param(netlist, model, "VB"))
            .or_else(|| cat.map(|c| c.var))
            .unwrap_or(f64::INFINITY);
        let ikf = Self::lookup_model_param(netlist, model, "IKF")
            .or_else(|| Self::lookup_model_param(netlist, model, "JBF"))
            .or_else(|| cat.map(|c| c.ikf))
            .unwrap_or(f64::INFINITY);
        let ikr = Self::lookup_model_param(netlist, model, "IKR")
            .or_else(|| Self::lookup_model_param(netlist, model, "JBR"))
            .or_else(|| cat.map(|c| c.ikr))
            .unwrap_or(f64::INFINITY);

        // Validate: if finite, must be positive
        if vaf.is_finite() {
            validate_positive_finite(vaf, "BJT model VAF")?;
        }
        if var.is_finite() {
            validate_positive_finite(var, "BJT model VAR")?;
        }
        if ikf.is_finite() {
            validate_positive_finite(ikf, "BJT model IKF")?;
        }
        if ikr.is_finite() {
            validate_positive_finite(ikr, "BJT model IKR")?;
        }

        // Junction capacitances (optional, default 0.0)
        let cje = Self::lookup_model_param(netlist, model, "CJE").unwrap_or(0.0);
        if cje < 0.0 || !cje.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CJE must be non-negative and finite, got {cje}"
            )));
        }
        let cjc = Self::lookup_model_param(netlist, model, "CJC").unwrap_or(0.0);
        if cjc < 0.0 || !cjc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CJC must be non-negative and finite, got {cjc}"
            )));
        }

        // Forward emission coefficient (default 1.0 = ideal)
        let nf = Self::lookup_model_param(netlist, model, "NF").unwrap_or(1.0);
        if nf <= 0.0 || !nf.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NF must be positive and finite, got {nf}"
            )));
        }

        // B-E leakage saturation current (default 0.0 = disabled)
        let ise = Self::lookup_model_param(netlist, model, "ISE").unwrap_or(0.0);
        if ise < 0.0 || !ise.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model ISE must be non-negative and finite, got {ise}"
            )));
        }

        // B-E leakage emission coefficient (default 1.5)
        let ne = Self::lookup_model_param(netlist, model, "NE").unwrap_or(1.5);
        if ne <= 0.0 || !ne.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NE must be positive and finite, got {ne}"
            )));
        }

        // Reverse emission coefficient (default 1.0 = ideal)
        let nr = Self::lookup_model_param(netlist, model, "NR").unwrap_or(1.0);
        if nr <= 0.0 || !nr.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NR must be positive and finite, got {nr}"
            )));
        }

        // B-C leakage saturation current (default 0.0 = disabled)
        let isc = Self::lookup_model_param(netlist, model, "ISC").unwrap_or(0.0);
        if isc < 0.0 || !isc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model ISC must be non-negative and finite, got {isc}"
            )));
        }

        // B-C leakage emission coefficient (default 2.0)
        let nc = Self::lookup_model_param(netlist, model, "NC").unwrap_or(2.0);
        if nc <= 0.0 || !nc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model NC must be positive and finite, got {nc}"
            )));
        }

        // Parasitic series resistances (optional, default 0.0)
        let rb = Self::lookup_model_param(netlist, model, "RB").unwrap_or(0.0);
        let rc = Self::lookup_model_param(netlist, model, "RC").unwrap_or(0.0);
        let re = Self::lookup_model_param(netlist, model, "RE").unwrap_or(0.0);
        if rb < 0.0 || !rb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RB must be non-negative and finite, got {rb}"
            )));
        }
        if rc < 0.0 || !rc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RC must be non-negative and finite, got {rc}"
            )));
        }
        if re < 0.0 || !re.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RE must be non-negative and finite, got {re}"
            )));
        }

        // Self-heating parameters (optional)
        let rth = Self::lookup_model_param(netlist, model, "RTH").unwrap_or(f64::INFINITY);
        if rth.is_finite() && rth <= 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model RTH must be positive (or infinite to disable), got {rth}"
            )));
        }

        let cth = Self::lookup_model_param(netlist, model, "CTH").unwrap_or(1e-3);
        if cth <= 0.0 || !cth.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CTH must be positive and finite, got {cth}"
            )));
        }

        let xti = Self::lookup_model_param(netlist, model, "XTI").unwrap_or(3.0);
        if !xti.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model XTI must be finite, got {xti}"
            )));
        }

        let eg = Self::lookup_model_param(netlist, model, "EG").unwrap_or(1.11);
        if eg <= 0.0 || !eg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model EG must be positive and finite, got {eg}"
            )));
        }

        let tamb = Self::lookup_model_param(netlist, model, "TAMB").unwrap_or(300.15);
        if tamb <= 0.0 || !tamb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model TAMB must be positive and finite, got {tamb}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &[
                "IS", "BF", "BR", "VAF", "VAR", "IKF", "IKR", "CJE", "CJC", "NF", "NR", "ISE",
                "NE", "ISC", "NC", "RB", "RC", "RE", "RTH", "CTH", "XTI", "EG", "TAMB", "KF",
                "AF",
            ],
        );

        Ok(BjtParams {
            is,
            vt,
            beta_f,
            beta_r,
            is_pnp,
            vaf,
            var,
            ikf,
            ikr,
            cje,
            cjc,
            nf,
            nr,
            ise,
            ne,
            isc,
            nc,
            rb,
            rc,
            re,
            rth,
            cth,
            xti,
            eg,
            tamb,
        })
    }

    /// Resolve JFET model parameters from the netlist, with validation.
    ///
    /// 2D Shichman-Hodges: IDSS, VP, and LAMBDA control triode + saturation regions.
    fn resolve_jfet_params(netlist: &Netlist, model: &str) -> Result<JfetParams, CodegenError> {
        let cat = melange_devices::catalog::jfets::lookup(model);

        // Determine channel type first — default VP depends on polarity.
        let is_p_channel = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
            .unwrap_or(cat.map(|c| c.is_p_channel).unwrap_or(false));

        let default_vp = cat
            .map(|c| c.vp)
            .unwrap_or(if is_p_channel { 2.0 } else { -2.0 });
        let vp = Self::lookup_model_param(netlist, model, "VTO").unwrap_or(default_vp);
        // ngspice BETA = IDSS / VP^2, so IDSS = BETA * VP^2
        let idss = if let Some(raw_idss) = Self::lookup_model_param(netlist, model, "IDSS") {
            raw_idss
        } else if let Some(beta) = Self::lookup_model_param(netlist, model, "BETA") {
            beta * vp * vp
        } else {
            cat.map(|c| c.idss).unwrap_or(2e-3)
        };
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.001);

        validate_positive_finite(idss, "JFET model IDSS")?;
        if !vp.is_finite() || vp.abs() < 1e-15 {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model VP must be finite and nonzero, got {vp}"
            )));
        }
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Junction capacitances (optional, default 0.0)
        let cgs = Self::lookup_model_param(netlist, model, "CGS").unwrap_or(0.0);
        if cgs < 0.0 || !cgs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model CGS must be non-negative and finite, got {cgs}"
            )));
        }
        let cgd = Self::lookup_model_param(netlist, model, "CGD").unwrap_or(0.0);
        if cgd < 0.0 || !cgd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model CGD must be non-negative and finite, got {cgd}"
            )));
        }

        // Ohmic drain/source resistances (optional, default 0.0)
        let rd = Self::lookup_model_param(netlist, model, "RD").unwrap_or(0.0);
        if rd < 0.0 || !rd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model RD must be non-negative and finite, got {rd}"
            )));
        }
        let rs = Self::lookup_model_param(netlist, model, "RS").unwrap_or(0.0);
        if rs < 0.0 || !rs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model RS must be non-negative and finite, got {rs}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &["VTO", "BETA", "IDSS", "LAMBDA", "CGS", "CGD", "RD", "RS", "KF", "AF"],
        );

        Ok(JfetParams {
            idss,
            vp,
            lambda,
            is_p_channel,
            cgs,
            cgd,
            rd,
            rs,
        })
    }

    /// Resolve MOSFET model parameters from the netlist, with validation.
    fn resolve_mosfet_params(netlist: &Netlist, model: &str) -> Result<MosfetParams, CodegenError> {
        let cat = melange_devices::catalog::mosfets::lookup(model);

        let is_p_channel = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model))
            .map(|m| m.model_type.to_uppercase().starts_with("PM"))
            .unwrap_or(cat.map(|c| c.is_p_channel).unwrap_or(false));

        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(0.1);
        let default_vt = cat
            .map(|c| c.vt)
            .unwrap_or(if is_p_channel { -2.0 } else { 2.0 });
        let vt = Self::lookup_model_param(netlist, model, "VTO")
            .or_else(|| Self::lookup_model_param(netlist, model, "VT"))
            .unwrap_or(default_vt);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.01);

        validate_positive_finite(kp, "MOSFET model KP")?;
        if !vt.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model VT must be finite, got {vt}"
            )));
        }
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Junction capacitances (optional, default 0.0)
        let cgs = Self::lookup_model_param(netlist, model, "CGS").unwrap_or(0.0);
        if cgs < 0.0 || !cgs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model CGS must be non-negative and finite, got {cgs}"
            )));
        }
        let cgd = Self::lookup_model_param(netlist, model, "CGD").unwrap_or(0.0);
        if cgd < 0.0 || !cgd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model CGD must be non-negative and finite, got {cgd}"
            )));
        }

        // Ohmic drain/source resistances (optional, default 0.0)
        let rd = Self::lookup_model_param(netlist, model, "RD").unwrap_or(0.0);
        if rd < 0.0 || !rd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model RD must be non-negative and finite, got {rd}"
            )));
        }
        let rs = Self::lookup_model_param(netlist, model, "RS").unwrap_or(0.0);
        if rs < 0.0 || !rs.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model RS must be non-negative and finite, got {rs}"
            )));
        }

        // Body effect parameters (optional, default 0.0 = disabled)
        let gamma = Self::lookup_model_param(netlist, model, "GAMMA").unwrap_or(0.0);
        let phi = Self::lookup_model_param(netlist, model, "PHI").unwrap_or(0.6);
        if gamma < 0.0 || !gamma.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model GAMMA must be non-negative and finite, got {gamma}"
            )));
        }
        if phi <= 0.0 || !phi.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model PHI must be positive and finite, got {phi}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &[
                "KP", "VTO", "LAMBDA", "CGS", "CGD", "RD", "RS", "GAMMA", "PHI", "KF", "AF",
            ],
        );

        // source_node and bulk_node will be resolved later from the MNA system
        Ok(MosfetParams {
            kp,
            vt,
            lambda,
            is_p_channel,
            cgs,
            cgd,
            rd,
            rs,
            gamma,
            phi,
            source_node: 0,
            bulk_node: 0,
        })
    }

    /// Resolve tube/triode model parameters from the netlist, with validation.
    ///
    /// Resolution order: explicit `.model` param → catalog → generic default (12AX7).
    fn resolve_tube_params(netlist: &Netlist, model: &str) -> Result<TubeParams, CodegenError> {
        let cat = melange_devices::catalog::tubes::lookup(model);
        let mu = Self::lookup_model_param(netlist, model, "MU")
            .or_else(|| cat.map(|c| c.mu))
            .unwrap_or(100.0);
        let ex = Self::lookup_model_param(netlist, model, "EX")
            .or_else(|| cat.map(|c| c.ex))
            .unwrap_or(1.4);
        let kg1 = Self::lookup_model_param(netlist, model, "KG1")
            .or_else(|| cat.map(|c| c.kg1))
            .unwrap_or(1060.0);
        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(600.0);
        let kvb = Self::lookup_model_param(netlist, model, "KVB")
            .or_else(|| cat.map(|c| c.kvb))
            .unwrap_or(300.0);
        let ig_max = Self::lookup_model_param(netlist, model, "IG_MAX")
            .or_else(|| cat.map(|c| c.ig_max))
            .unwrap_or(2e-3);
        let vgk_onset = Self::lookup_model_param(netlist, model, "VGK_ONSET")
            .or_else(|| cat.map(|c| c.vgk_onset))
            .unwrap_or(0.5);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.0);

        // Reefman §5 variable-mu (remote-cutoff) parameters. Optional — default
        // 0.0 means sharp single-section Koren. The triode catalog
        // (`TubeCatalogEntry`) does NOT carry these fields in phase 1c, so the
        // only source is the `.model` directive; callers who want a variable-mu
        // triode must spell `MU_B`/`SVAR`/`EX_B` explicitly.
        let mu_b = Self::lookup_model_param(netlist, model, "MU_B").unwrap_or(0.0);
        let svar = Self::lookup_model_param(netlist, model, "SVAR").unwrap_or(0.0);
        let ex_b = Self::lookup_model_param(netlist, model, "EX_B").unwrap_or(0.0);

        validate_positive_finite(mu, "tube model MU")?;
        validate_positive_finite(ex, "tube model EX")?;
        validate_positive_finite(kg1, "tube model KG1")?;
        validate_positive_finite(kp, "tube model KP")?;
        validate_positive_finite(kvb, "tube model KVB")?;
        validate_positive_finite(ig_max, "tube model IG_MAX")?;
        validate_positive_finite(vgk_onset, "tube model VGK_ONSET")?;

        // Validate optional lambda: must be non-negative and finite
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Reefman §5 variable-mu constraints (mirrors `TubeParams::validate()`).
        // Surfacing them at the resolver level gives a clearer error site than
        // the downstream `params.validate()` call, and lets us mention the
        // `.model` directive name in the diagnostic.
        if !svar.is_finite() || !(0.0..=1.0).contains(&svar) {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model SVAR must be in [0, 1] and finite, got {svar}"
            )));
        }
        if svar > 0.0 {
            if !mu_b.is_finite() || mu_b <= 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "variable-mu tube MU_B must be positive and finite when SVAR>0, got {mu_b}"
                )));
            }
            if !ex_b.is_finite() || ex_b <= 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "variable-mu tube EX_B must be positive and finite when SVAR>0, got {ex_b}"
                )));
            }
        }

        // Inter-electrode capacitances (optional, default 0.0)
        let ccg = Self::lookup_model_param(netlist, model, "CCG").unwrap_or(0.0);
        if ccg < 0.0 || !ccg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CCG must be non-negative and finite, got {ccg}"
            )));
        }
        let cgp = Self::lookup_model_param(netlist, model, "CGP").unwrap_or(0.0);
        if cgp < 0.0 || !cgp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CGP must be non-negative and finite, got {cgp}"
            )));
        }
        let ccp = Self::lookup_model_param(netlist, model, "CCP").unwrap_or(0.0);
        if ccp < 0.0 || !ccp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CCP must be non-negative and finite, got {ccp}"
            )));
        }

        // Grid internal resistance (optional, default 0.0 = disabled)
        let rgi = Self::lookup_model_param(netlist, model, "RGI").unwrap_or(0.0);
        if rgi < 0.0 || !rgi.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model RGI must be non-negative and finite, got {rgi}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &[
                "MU",
                "EX",
                "KG1",
                "KP",
                "KVB",
                "IG_MAX",
                "VGK_ONSET",
                "LAMBDA",
                "CCG",
                "CGP",
                "CCP",
                "RGI",
                "MU_B",
                "SVAR",
                "EX_B",
                "KF",
                "AF",
            ],
        );

        Ok(TubeParams {
            kind: crate::device_types::TubeKind::SharpTriode,
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max,
            vgk_onset,
            lambda,
            ccg,
            cgp,
            ccp,
            rgi,
            kg2: 0.0,
            alpha_s: 0.0,
            a_factor: 0.0,
            beta_factor: 0.0,
            screen_form: crate::device_types::ScreenForm::Rational,
            mu_b,
            svar,
            ex_b,
        })
    }

    /// Resolve pentode model parameters from the netlist, with validation.
    ///
    /// Uses Reefman's pentode equations (see `pentode_equations.md` memory
    /// file). Reads MU, EX, KG1, KG2, KP, KVB, ALPHA_S, A_FACTOR, BETA_FACTOR,
    /// SCREEN_FORM plus the shared triode-compatible params (IG_MAX, VGK_ONSET,
    /// CCG/CGP/CCP, RGI).
    ///
    /// Resolution order (each parameter independently):
    ///   1. Explicit `.model NAME VP(PARAM=value)` in the netlist
    ///   2. `PENTODE_CATALOG` entry keyed by the model name (e.g. `EL84-P`,
    ///      `6L6GC-T`)
    ///   3. Generic EL84-shaped fallback default (lets a bare `.model FOO VP()`
    ///      still produce a working — if wrong — pentode so codegen doesn't
    ///      crash on unfitted circuits)
    ///
    /// The screen-current form (`Rational` / `Exponential`) resolves the same
    /// way: explicit `SCREEN_FORM=0|1` param overrides catalog, catalog
    /// provides the right default for fitted tubes (EL84/EL34/EF86 →
    /// Rational, 6L6GC/6V6GT → Exponential), and the fallback is `Rational`.
    ///
    /// Returns a `TubeParams` with `kind = SharpPentode`. Callers should
    /// `validate()` the result; this function does explicit `Err` on missing
    /// required pentode params (KG2, ALPHA_S).
    fn resolve_pentode_params(netlist: &Netlist, model: &str) -> Result<TubeParams, CodegenError> {
        // Catalog lookup first — if the model name matches a PentodeCatalogEntry,
        // we use those fitted params as the fallback. Explicit `.model VP(...)`
        // parameters override on a per-field basis (user can replace any subset).
        let cat = melange_devices::catalog::tubes::lookup_pentode(model);

        let mu = Self::lookup_model_param(netlist, model, "MU")
            .or_else(|| cat.map(|c| c.mu))
            .unwrap_or(23.36);
        let ex = Self::lookup_model_param(netlist, model, "EX")
            .or_else(|| cat.map(|c| c.ex))
            .unwrap_or(1.138);
        let kg1 = Self::lookup_model_param(netlist, model, "KG1")
            .or_else(|| cat.map(|c| c.kg1))
            .unwrap_or(117.4);
        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(152.4);
        let kvb = Self::lookup_model_param(netlist, model, "KVB")
            .or_else(|| cat.map(|c| c.kvb))
            .unwrap_or(4015.8);
        let kg2 = Self::lookup_model_param(netlist, model, "KG2")
            .or_else(|| cat.map(|c| c.kg2))
            .unwrap_or(1275.0);
        let alpha_s = Self::lookup_model_param(netlist, model, "ALPHA_S")
            .or_else(|| cat.map(|c| c.alpha_s))
            .unwrap_or(7.66);
        // `A` alone collides with other SPICE conventions (e.g. AC), so the
        // model directive uses the more explicit name `A_FACTOR`.
        let a_factor = Self::lookup_model_param(netlist, model, "A_FACTOR")
            .or_else(|| cat.map(|c| c.a_factor))
            .unwrap_or(4.344e-4);
        let beta_factor = Self::lookup_model_param(netlist, model, "BETA_FACTOR")
            .or_else(|| cat.map(|c| c.beta_factor))
            .unwrap_or(0.148);
        let ig_max = Self::lookup_model_param(netlist, model, "IG_MAX")
            .or_else(|| cat.map(|c| c.ig_max))
            .unwrap_or(8e-3);
        let vgk_onset = Self::lookup_model_param(netlist, model, "VGK_ONSET")
            .or_else(|| cat.map(|c| c.vgk_onset))
            .unwrap_or(0.7);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA").unwrap_or(0.0);

        // Reefman §5 variable-mu (remote-cutoff) parameters. Resolution order
        // matches every other field: explicit `.model` > catalog > default 0.0.
        let mu_b = Self::lookup_model_param(netlist, model, "MU_B")
            .or_else(|| cat.map(|c| c.mu_b))
            .unwrap_or(0.0);
        let svar = Self::lookup_model_param(netlist, model, "SVAR")
            .or_else(|| cat.map(|c| c.svar))
            .unwrap_or(0.0);
        let ex_b = Self::lookup_model_param(netlist, model, "EX_B")
            .or_else(|| cat.map(|c| c.ex_b))
            .unwrap_or(0.0);

        // Screen form: catalog value wins over the default; explicit
        // `SCREEN_FORM=0|1|2` in the .model directive wins over the catalog.
        //   0 = Rational   (Derk §4.4)
        //   1 = Exponential (DerkE §4.5)
        //   2 = Classical   (Norman Koren 1996 / Cohen-Hélie 2010)
        let screen_form = {
            use crate::device_types::ScreenForm;
            let explicit = Self::lookup_model_param(netlist, model, "SCREEN_FORM");
            match explicit {
                Some(v) if v == 0.0 => ScreenForm::Rational,
                Some(v) if v == 1.0 => ScreenForm::Exponential,
                Some(v) if v == 2.0 => ScreenForm::Classical,
                Some(v) => {
                    return Err(CodegenError::InvalidConfig(format!(
                        "pentode model SCREEN_FORM must be 0 (Rational), \
                         1 (Exponential), or 2 (Classical), got {v}"
                    )));
                }
                None => match cat.map(|c| c.screen_form) {
                    Some(melange_devices::tube::ScreenForm::Exponential) => {
                        ScreenForm::Exponential
                    }
                    Some(melange_devices::tube::ScreenForm::Classical) => {
                        ScreenForm::Classical
                    }
                    _ => ScreenForm::Rational,
                },
            }
        };

        validate_positive_finite(mu, "pentode model MU")?;
        validate_positive_finite(ex, "pentode model EX")?;
        validate_positive_finite(kg1, "pentode model KG1")?;
        validate_positive_finite(kg2, "pentode model KG2")?;
        validate_positive_finite(kp, "pentode model KP")?;
        validate_positive_finite(kvb, "pentode model KVB")?;
        validate_positive_finite(ig_max, "pentode model IG_MAX")?;
        validate_positive_finite(vgk_onset, "pentode model VGK_ONSET")?;

        // Classical Koren does not use alpha_s / a_factor / beta_factor at
        // all — they're ignored by the `*_pentode_classical` helpers. Skip
        // the Derk-specific invariants when the screen form is Classical.
        let uses_derk_shape = !matches!(screen_form, crate::device_types::ScreenForm::Classical);
        if uses_derk_shape {
            validate_positive_finite(alpha_s, "pentode model ALPHA_S")?;
            if !a_factor.is_finite() || a_factor < 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "pentode model A_FACTOR must be non-negative and finite, got {a_factor}"
                )));
            }
            if !beta_factor.is_finite() || beta_factor < 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "pentode model BETA_FACTOR must be non-negative and finite, got {beta_factor}"
                )));
            }
        }
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model LAMBDA must be non-negative and finite, got {lambda}"
            )));
        }

        // Reefman §5 variable-mu constraints (mirrors `TubeParams::validate()`).
        // Surfacing them at the resolver level gives a clearer error site than
        // the downstream `params.validate()` call.
        if !svar.is_finite() || !(0.0..=1.0).contains(&svar) {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model SVAR must be in [0, 1] and finite, got {svar}"
            )));
        }
        if svar > 0.0 {
            if !mu_b.is_finite() || mu_b <= 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "variable-mu pentode MU_B must be positive and finite when SVAR>0, got {mu_b}"
                )));
            }
            if !ex_b.is_finite() || ex_b <= 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "variable-mu pentode EX_B must be positive and finite when SVAR>0, got {ex_b}"
                )));
            }
            // Variable-mu + Classical is unsupported — Reefman §5 is built on
            // the Derk softplus structure, not the Classical arctan knee.
            if matches!(screen_form, crate::device_types::ScreenForm::Classical) {
                return Err(CodegenError::InvalidConfig(
                    "variable-mu Classical Koren pentodes are not implemented; \
                     use SCREEN_FORM=0 (Rational) for variable-mu tubes (6K7/EF89 pattern)"
                        .to_string(),
                ));
            }
        }

        let ccg = Self::lookup_model_param(netlist, model, "CCG").unwrap_or(0.0);
        if ccg < 0.0 || !ccg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model CCG must be non-negative and finite, got {ccg}"
            )));
        }
        let cgp = Self::lookup_model_param(netlist, model, "CGP").unwrap_or(0.0);
        if cgp < 0.0 || !cgp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model CGP must be non-negative and finite, got {cgp}"
            )));
        }
        let ccp = Self::lookup_model_param(netlist, model, "CCP").unwrap_or(0.0);
        if ccp < 0.0 || !ccp.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model CCP must be non-negative and finite, got {ccp}"
            )));
        }
        let rgi = Self::lookup_model_param(netlist, model, "RGI").unwrap_or(0.0);
        if rgi < 0.0 || !rgi.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "pentode model RGI must be non-negative and finite, got {rgi}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &[
                "MU",
                "EX",
                "KG1",
                "KG2",
                "KP",
                "KVB",
                "ALPHA_S",
                "A_FACTOR",
                "BETA_FACTOR",
                "SCREEN_FORM",
                "IG_MAX",
                "VGK_ONSET",
                "LAMBDA",
                "CCG",
                "CGP",
                "CCP",
                "RGI",
                "MU_B",
                "SVAR",
                "EX_B",
                "KF",
                "AF",
            ],
        );

        let params = TubeParams {
            kind: crate::device_types::TubeKind::SharpPentode,
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max,
            vgk_onset,
            lambda,
            ccg,
            cgp,
            ccp,
            rgi,
            kg2,
            alpha_s,
            a_factor,
            beta_factor,
            screen_form,
            // Phase 1c: variable-mu §5 params. Defaults to sharp (svar=0).
            // Resolved by a follow-up (task P1c-03) which reads MU_B / SVAR /
            // EX_B from the .model directive with catalog fallback.
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        };
        params
            .validate()
            .map_err(CodegenError::InvalidConfig)?;
        Ok(params)
    }

    /// Resolve VCA model parameters from the netlist, with validation.
    ///
    /// 2D current-mode exponential gain: I_sig = G0 * exp(-Vc / VSCALE) * V_sig
    fn resolve_vca_params(netlist: &Netlist, model: &str) -> Result<VcaParams, CodegenError> {
        let vscale = Self::lookup_model_param(netlist, model, "VSCALE").unwrap_or(0.05298);
        let g0 = Self::lookup_model_param(netlist, model, "G0").unwrap_or(1.0);
        let thd = Self::lookup_model_param(netlist, model, "THD").unwrap_or(0.0);

        validate_positive_finite(vscale, "VCA model VSCALE")?;
        validate_positive_finite(g0, "VCA model G0")?;
        if thd < 0.0 || !thd.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "VCA model THD must be non-negative and finite, got {thd}"
            )));
        }

        Self::warn_unrecognized_params(
            netlist,
            model,
            &["VSCALE", "G0", "THD", "MODE"],
        );

        Ok(VcaParams {
            vscale,
            g0,
            thd,
        })
    }

    /// Warn on unrecognized .model parameters (typo protection).
    fn warn_unrecognized_params(netlist: &Netlist, model_name: &str, known: &[&str]) {
        if let Some(m) = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
        {
            for (key, _) in &m.params {
                let upper = key.to_ascii_uppercase();
                if !known.iter().any(|k| k.eq_ignore_ascii_case(&upper)) {
                    log::warn!(
                        ".model {}: unrecognized parameter '{}' (ignored)",
                        model_name,
                        key,
                    );
                }
            }
        }
    }

    /// Look up a parameter from a `.model` directive, case-insensitive.
    fn lookup_model_param(netlist: &Netlist, model_name: &str, param_name: &str) -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param_name))
                    .map(|(_, v)| *v)
            })
    }

    /// Access S matrix element S[i][j]
    pub fn s(&self, i: usize, j: usize) -> f64 {
        let n = self.topology.n;
        debug_assert!(
            i < n && j < n,
            "s({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            n,
            n
        );
        self.matrices.s[i * n + j]
    }

    /// Access K matrix element K[i][j]
    pub fn k(&self, i: usize, j: usize) -> f64 {
        let m = self.topology.m;
        debug_assert!(
            i < m && j < m,
            "k({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            m,
            m
        );
        self.matrices.k[i * m + j]
    }

    /// Access N_v matrix element N_v[i][j] (M×N storage: device × node)
    pub fn n_v(&self, i: usize, j: usize) -> f64 {
        let n = self.topology.n;
        debug_assert!(
            i < self.topology.m && j < n,
            "n_v({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            self.topology.m,
            n
        );
        self.matrices.n_v[i * n + j]
    }

    /// Access N_i matrix element N_i[i][j] (N×M storage: node × device)
    pub fn n_i(&self, i: usize, j: usize) -> f64 {
        let m = self.topology.m;
        debug_assert!(
            i < self.topology.n && j < m,
            "n_i({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            self.topology.n,
            m
        );
        self.matrices.n_i[i * m + j]
    }

    /// Access A_neg matrix element A_neg[i][j]
    pub fn a_neg(&self, i: usize, j: usize) -> f64 {
        let n = self.topology.n;
        debug_assert!(
            i < n && j < n,
            "a_neg({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            n,
            n
        );
        self.matrices.a_neg[i * n + j]
    }

    /// Access G matrix element G[i][j]
    pub fn g(&self, i: usize, j: usize) -> f64 {
        let n = self.topology.n;
        debug_assert!(
            i < n && j < n,
            "g({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            n,
            n
        );
        self.matrices.g_matrix[i * n + j]
    }

    /// Access C matrix element C[i][j]
    pub fn c(&self, i: usize, j: usize) -> f64 {
        let n = self.topology.n;
        debug_assert!(
            i < n && j < n,
            "c({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            n,
            n
        );
        self.matrices.c_matrix[i * n + j]
    }

    /// Access A matrix element A[i][j] (trapezoidal, nodal mode only)
    pub fn a_matrix(&self, i: usize, j: usize) -> f64 {
        let n = self.topology.n;
        debug_assert!(
            i < n && j < n,
            "a_matrix({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            n,
            n
        );
        self.matrices.a_matrix[i * n + j]
    }

    /// Access A_be matrix element A_be[i][j] (backward Euler, nodal mode only)
    pub fn a_matrix_be(&self, i: usize, j: usize) -> f64 {
        let n = self.topology.n;
        debug_assert!(
            i < n && j < n,
            "a_matrix_be({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            n,
            n
        );
        self.matrices.a_matrix_be[i * n + j]
    }

    /// Access A_neg_be matrix element A_neg_be[i][j] (backward Euler history, nodal mode only)
    pub fn a_neg_be(&self, i: usize, j: usize) -> f64 {
        let n = self.topology.n;
        debug_assert!(
            i < n && j < n,
            "a_neg_be({}, {}) out of bounds for {}x{} matrix",
            i,
            j,
            n,
            n
        );
        self.matrices.a_neg_be[i * n + j]
    }

    /// Access S_be matrix element S_be[i][j] (backward Euler, nodal Schur)
    pub fn s_be(&self, i: usize, j: usize) -> f64 {
        self.matrices.s_be[i * self.topology.n + j]
    }

    /// Access K_be matrix element K_be[i][j] (backward Euler, nodal Schur)
    pub fn k_be(&self, i: usize, j: usize) -> f64 {
        self.matrices.k_be[i * self.topology.m + j]
    }

    /// Access S_sub matrix element S_sub[i][j] (trap at 2× rate, sub-stepping)
    pub fn s_sub(&self, i: usize, j: usize) -> f64 {
        self.matrices.s_sub[i * self.topology.n + j]
    }

    /// Access K_sub matrix element K_sub[i][j] (trap at 2× rate, sub-stepping)
    pub fn k_sub(&self, i: usize, j: usize) -> f64 {
        self.matrices.k_sub[i * self.topology.m + j]
    }

    /// Access A_neg_sub matrix element A_neg_sub[i][j] (trap at 2× rate history)
    pub fn a_neg_sub(&self, i: usize, j: usize) -> f64 {
        self.matrices.a_neg_sub[i * self.topology.n + j]
    }
}

#[cfg(test)]
mod opamp_rail_mode_tests {
    use super::*;
    use crate::codegen::OpampRailMode;
    use crate::mna::{MnaSystem, OpampInfo};

    /// Build a minimal MNA with `opamps` attached. We don't care about the rest of
    /// the MNA state for resolver tests — the resolver only reads `mna.opamps`.
    fn mna_with_opamps(opamps: Vec<OpampInfo>) -> MnaSystem {
        let mut mna = MnaSystem::new(1, 0, 0, 0);
        mna.opamps = opamps;
        mna
    }

    fn opamp_with_rails(vcc: f64, vee: f64) -> OpampInfo {
        OpampInfo {
            name: "U_TEST".to_string(),
            n_plus_idx: 1,
            n_minus_idx: 2,
            n_out_idx: 3,
            aol: 200_000.0,
            r_out: 50.0,
            vsat: f64::INFINITY,
            vcc,
            vee,
            gbw: f64::INFINITY,
            sr: f64::INFINITY,
            ib: 0.0,
            rin: f64::INFINITY,
            voh_drop: 1.5,
            vol_drop: 1.5,
            aol_transient_cap: f64::INFINITY,
            n_internal_idx: 0,
            iir_c_dom: 0.0,
            n_int_idx: 0,
        }
    }

    #[test]
    fn resolver_honors_explicit_user_choice_hard() {
        let mna = mna_with_opamps(vec![opamp_with_rails(9.0, 0.0)]);
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Hard);
        assert_eq!(r.mode, OpampRailMode::Hard);
        assert_eq!(r.reason, OpampRailModeReason::UserRequested);
    }

    #[test]
    fn resolver_honors_explicit_user_choice_active_set() {
        let mna = mna_with_opamps(vec![opamp_with_rails(9.0, 0.0)]);
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::ActiveSet);
        assert_eq!(r.mode, OpampRailMode::ActiveSet);
        assert_eq!(r.reason, OpampRailModeReason::UserRequested);
    }

    #[test]
    fn resolver_honors_explicit_user_choice_boyle() {
        let mna = mna_with_opamps(vec![opamp_with_rails(9.0, 0.0)]);
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::BoyleDiodes);
        assert_eq!(r.mode, OpampRailMode::BoyleDiodes);
        assert_eq!(r.reason, OpampRailModeReason::UserRequested);
    }

    #[test]
    fn resolver_honors_explicit_none_even_with_clamped_opamps() {
        // User override must not be silently upgraded even when the circuit
        // would benefit from clamping. The escape hatch has to be trustworthy.
        let mna = mna_with_opamps(vec![opamp_with_rails(9.0, 0.0)]);
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::None);
        assert_eq!(r.mode, OpampRailMode::None);
        assert_eq!(r.reason, OpampRailModeReason::UserRequested);
    }

    #[test]
    fn resolver_auto_no_opamps_picks_none() {
        let mna = mna_with_opamps(vec![]);
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(r.mode, OpampRailMode::None);
        assert_eq!(r.reason, OpampRailModeReason::NoClampedOpamps);
    }

    #[test]
    fn resolver_auto_opamps_without_rails_picks_none() {
        // Op-amps with infinite VCC and VEE are ideal VCCSs — no clamp needed.
        let mna = mna_with_opamps(vec![opamp_with_rails(f64::INFINITY, f64::NEG_INFINITY)]);
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(r.mode, OpampRailMode::None);
        assert_eq!(r.reason, OpampRailModeReason::NoClampedOpamps);
    }

    // --- Cap-coupling helpers for resolver topology tests ---
    //
    // `mna_with_opamps` creates a 1-node MNA which isn't enough for cap
    // stamps. This helper grows the C matrix to the requested node count
    // and returns a ready-to-use MnaSystem with op-amps attached. Node
    // numbering is 1-indexed (0 = ground) to match the MnaSystem convention.
    fn mna_with_opamps_and_caps(
        n_nodes: usize,
        opamps: Vec<OpampInfo>,
        caps: &[(usize, usize, f64)],
    ) -> MnaSystem {
        let mut mna = MnaSystem::new(n_nodes, 0, 0, 0);
        mna.opamps = opamps;
        for &(i, j, c) in caps {
            // Convert 1-indexed inputs to 0-indexed matrix indices; skip
            // ground (0) terminals as usual.
            if i == 0 || j == 0 {
                continue;
            }
            let ii = i - 1;
            let jj = j - 1;
            mna.c[ii][ii] += c;
            mna.c[jj][jj] += c;
            mna.c[ii][jj] -= c;
            mna.c[jj][ii] -= c;
        }
        mna
    }

    fn opamp_at_nodes(np: usize, nm: usize, out: usize, vcc: f64, vee: f64) -> OpampInfo {
        OpampInfo {
            name: "U_TEST".to_string(),
            n_plus_idx: np,
            n_minus_idx: nm,
            n_out_idx: out,
            aol: 200_000.0,
            r_out: 50.0,
            vsat: f64::INFINITY,
            vcc,
            vee,
            gbw: f64::INFINITY,
            sr: f64::INFINITY,
            ib: 0.0,
            rin: f64::INFINITY,
            voh_drop: 1.5,
            vol_drop: 1.5,
            aol_transient_cap: f64::INFINITY,
            n_internal_idx: 0,
            iir_c_dom: 0.0,
            n_int_idx: 0,
        }
    }

    #[test]
    fn resolver_auto_opamps_with_single_rail_picks_hard_when_dc_coupled() {
        // Single finite rail, no cap coupling → Hard mode.
        // Single-node MNA: out_idx=1, no caps at all.
        let mna = mna_with_opamps_and_caps(
            3,
            vec![opamp_at_nodes(2, 1, 1, 9.0, f64::NEG_INFINITY)],
            &[],
        );
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(r.mode, OpampRailMode::Hard);
        assert_eq!(r.reason, OpampRailModeReason::AllDcCoupled);
    }

    #[test]
    fn resolver_auto_opamps_with_both_rails_picks_hard_when_dc_coupled() {
        let mna = mna_with_opamps_and_caps(
            3,
            vec![opamp_at_nodes(2, 1, 1, 9.0, 0.0)],
            &[],
        );
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(r.mode, OpampRailMode::Hard);
        assert_eq!(r.reason, OpampRailModeReason::AllDcCoupled);
    }

    #[test]
    fn resolver_auto_feedback_cap_alone_is_dc_coupled() {
        // Op-amp with only a feedback cap between output (node 1) and its
        // own inverting input (node 1 here — a unity follower has nm = out).
        // No downstream coupling → Hard is safe.
        //
        // Topology: unity follower where output feeds its own - input.
        // feedback cap from output (1) to - input (1): self-loop, ignored.
        // Add a separate coupling cap from a different node (2) to ground
        // (doesn't touch op-amp output).
        let opamps = vec![opamp_at_nodes(3, 1, 1, 9.0, 0.0)]; // np=3, nm=1, out=1
        let mna = mna_with_opamps_and_caps(
            3,
            opamps,
            &[(2, 0, 1e-6)], // cap from node 2 to ground, not touching op-amp
        );
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(r.mode, OpampRailMode::Hard);
        assert_eq!(r.reason, OpampRailModeReason::AllDcCoupled);
    }

    #[test]
    fn resolver_auto_feedback_cap_from_out_to_minus_input_stays_hard() {
        // Inverting amp: op-amp out=3, nm=2, np=1 (vbias). A feedback cap
        // from out (3) to nm (2) should NOT trigger ActiveSet — it's a
        // feedback cap, not a downstream coupling cap.
        let opamps = vec![opamp_at_nodes(1, 2, 3, 9.0, 0.0)];
        let mna = mna_with_opamps_and_caps(
            3,
            opamps,
            &[(3, 2, 820e-12)], // feedback cap between out (3) and - input (2)
        );
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(r.mode, OpampRailMode::Hard);
        assert_eq!(r.reason, OpampRailModeReason::AllDcCoupled);
    }

    #[test]
    fn resolver_auto_cap_from_out_to_downstream_picks_active_set() {
        // Op-amp out=3, nm=2, np=1. Output coupling cap from node 3 to a
        // downstream node 4 (which is not the inverting input). This is
        // the Klon-C15 pattern — must trigger ActiveSet.
        let opamps = vec![opamp_at_nodes(1, 2, 3, 9.0, 0.0)];
        let mna = mna_with_opamps_and_caps(
            4,
            opamps,
            &[(3, 4, 4.7e-6)], // coupling cap from out (3) to downstream (4)
        );
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(r.mode, OpampRailMode::ActiveSet);
        assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
    }

    #[test]
    fn resolver_auto_mixed_opamps_one_ac_coupled_forces_active_set() {
        // Two op-amps: one with only feedback cap (safe on Hard), one with
        // downstream coupling cap (needs ActiveSet). Any offender forces
        // ActiveSet for the whole circuit because modes are global.
        let opamps = vec![
            opamp_at_nodes(1, 2, 3, 9.0, 0.0), // feedback only
            opamp_at_nodes(4, 5, 6, 9.0, 0.0), // will have downstream cap
        ];
        let mna = mna_with_opamps_and_caps(
            7,
            opamps,
            &[
                (3, 2, 820e-12), // feedback cap on first op-amp — OK
                (6, 7, 4.7e-6),  // downstream coupling on second op-amp — triggers ActiveSet
            ],
        );
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(r.mode, OpampRailMode::ActiveSet);
        assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
    }

    #[test]
    fn resolver_auto_opamp_with_zero_out_idx_ignored() {
        // An op-amp whose output is ground (out_idx = 0) can't be clamped;
        // the MNA builder would have dropped it, but defensively the resolver
        // should treat it as "not a clamp candidate".
        let mut oa = opamp_with_rails(9.0, 0.0);
        oa.n_out_idx = 0;
        let mna = mna_with_opamps(vec![oa]);
        let r = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(r.mode, OpampRailMode::None);
        assert_eq!(r.reason, OpampRailModeReason::NoClampedOpamps);
    }

    // Integration tests using synthetic circuits that exercise the same
    // auto-detection code paths as the real circuits (which now live in
    // the melange-audio/circuits repo).

    fn parse_and_resolve(spice: &str) -> ResolvedOpampRailMode {
        let netlist = crate::parser::Netlist::parse(spice)
            .unwrap_or_else(|e| panic!("failed to parse: {}", e));
        let mna = MnaSystem::from_netlist(&netlist)
            .unwrap_or_else(|e| panic!("failed to build MNA: {}", e));
        resolve_opamp_rail_mode(&mna, OpampRailMode::Auto)
    }

    #[test]
    fn opamp_with_ac_coupled_downstream_picks_active_set() {
        // Synthetic: op-amp with AC-coupled downstream stage.
        // Exercises the same AcCoupledDownstream path as Klon's topology.
        let spice = "\
Opamp AC-Coupled Downstream Test
R1 in sum 4.7k
R2 sum out 47k
C1 out out_ac 100n
R3 out_ac 0 100k
U1 0 sum out OA1
.model OA1 OA(AOL=100k GBW=3e6 ROUT=75 VCC=4.5 VEE=-4.5)
";
        let r = parse_and_resolve(spice);
        assert_eq!(r.mode, OpampRailMode::ActiveSet);
        assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
    }

    #[test]
    fn circuit_without_opamps_picks_none() {
        // Synthetic: tubes and passives, no op-amps. Same path as Pultec.
        let spice = "\
No Op-Amp Test
R1 in grid 68k
R2 plate 0 100k
C1 grid 0 22p
V1 plate 0 DC 250
";
        let r = parse_and_resolve(spice);
        assert_eq!(r.mode, OpampRailMode::None);
        assert_eq!(r.reason, OpampRailModeReason::NoClampedOpamps);
    }

    #[test]
    fn single_opamp_no_downstream_coupling_picks_concrete_mode() {
        // Synthetic: single op-amp, no AC-coupled downstream.
        // Must resolve to a concrete mode (never Auto).
        let spice = "\
Single Op-Amp Test
R1 in neg 10k
R2 neg out 100k
U1 in neg out OA1
.model OA1 OA(AOL=100k GBW=1e6 ROUT=100 VCC=15 VEE=-15)
";
        let r = parse_and_resolve(spice);
        assert_ne!(r.mode, OpampRailMode::Auto);
    }

    #[test]
    fn augment_netlist_with_boyle_diodes_produces_valid_mna() {
        // Synthetic: 2 op-amps with finite rails. Tests that the Boyle
        // catch-diode augmentation helper synthesizes the correct elements
        // and that the augmented MNA builds without error.
        use crate::codegen::OpampRailMode;
        let spice = "\
Boyle Diodes Augmentation Test
R1 in sum1 4.7k
R2 sum1 out1 47k
C1 out1 out1_ac 100n
R3 out1_ac sum2 10k
R4 sum2 out 47k
C2 out out_ac 100n
R5 out_ac 0 100k
U1 0 sum1 out1 OA1
U2 0 sum2 out OA1
.model OA1 OA(AOL=100k GBW=3e6 ROUT=75 VCC=4.5 VEE=-4.5)
";
        let netlist = crate::parser::Netlist::parse(spice).expect("parse");
        let mna = MnaSystem::from_netlist(&netlist).expect("mna");

        // Sanity: 2 op-amps with finite rails.
        let clamped_opamps = mna
            .opamps
            .iter()
            .filter(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()))
            .count();
        assert_eq!(clamped_opamps, 2, "Should have 2 clamped op-amps");

        let aug_netlist = augment_netlist_with_boyle_diodes(&netlist, &mna);

        // Exactly one D_BOYLE_CATCH model.
        let catch_models = aug_netlist
            .models
            .iter()
            .filter(|m| m.name == BOYLE_CATCH_DIODE_MODEL)
            .count();
        assert_eq!(catch_models, 1);

        // Diode Is/N match Boyle-standard silicon.
        let catch_model = aug_netlist
            .models
            .iter()
            .find(|m| m.name == BOYLE_CATCH_DIODE_MODEL)
            .unwrap();
        assert_eq!(catch_model.model_type, "D");
        let is_val = catch_model
            .params
            .iter()
            .find(|(k, _)| k == "IS")
            .map(|(_, v)| *v)
            .unwrap();
        assert!((is_val - 1e-15).abs() < 1e-20, "Is should be 1e-15, got {is_val}");
        let n_val = catch_model
            .params
            .iter()
            .find(|(k, _)| k == "N")
            .map(|(_, v)| *v)
            .unwrap();
        assert!((n_val - 1.0).abs() < 1e-12, "N should be 1.0, got {n_val}");

        // Count the synthesized elements:
        //   * 2 op-amps × 2 rails × (1 VS + 1 diode) = 4 VS + 4 diodes
        //   * 2 op-amps × 1 buffer VCVS               = 2 VCVS
        let vs_added = aug_netlist
            .elements
            .iter()
            .filter(|e| {
                matches!(e, crate::parser::Element::VoltageSource { name, .. } if name.starts_with("V_boyle_"))
            })
            .count();
        let diodes_added = aug_netlist
            .elements
            .iter()
            .filter(|e| {
                matches!(e, crate::parser::Element::Diode { name, .. } if name.starts_with("D_boyle_"))
            })
            .count();
        let buffer_vcvs_added = aug_netlist
            .elements
            .iter()
            .filter(|e| {
                matches!(e, crate::parser::Element::Vcvs { name, .. } if name.starts_with("E_oa_buf_"))
            })
            .count();
        assert_eq!(vs_added, 4, "Expected 4 rail-reference voltage sources (2 op-amps × 2 rails)");
        assert_eq!(diodes_added, 4, "Expected 4 catch diodes (2 op-amps × 2 rails)");
        assert_eq!(buffer_vcvs_added, 2, "Expected 2 output-buffer VCVS (1 per clamped op-amp)");

        let r_ro_added = aug_netlist
            .elements
            .iter()
            .filter(|e| {
                matches!(e, crate::parser::Element::Resistor { name, .. } if name.starts_with("R_oa_ro_"))
            })
            .count();
        assert_eq!(r_ro_added, 2, "Expected 2 output-buffer series resistors (1 per clamped op-amp)");

        // Rebuild MNA from the augmented netlist — this must succeed.
        let aug_mna = MnaSystem::from_netlist(&aug_netlist).expect("augmented MNA build");

        // Dimensions grow by:
        //   * +8 nodes: 2 internal gain + 2 buffer-output + 4 rail-reference
        //   * +4 nonlinear devices (catch diodes)
        //   * +4 voltage sources (rail-reference DC sources)
        assert_eq!(
            aug_mna.n,
            mna.n + 8,
            "augmented n should grow by 4 per clamped op-amp (int + buf_out + 2 rail-ref)"
        );
        assert_eq!(
            aug_mna.m,
            mna.m + 4,
            "augmented m should grow by 2 per clamped op-amp"
        );
        assert_eq!(
            aug_mna.voltage_sources.len(),
            mna.voltage_sources.len() + 4,
            "augmented VS count should grow by 2 per clamped op-amp"
        );

        // Original nodes keep their indices.
        assert_eq!(mna.node_map["in"], aug_mna.node_map["in"]);
        assert_eq!(mna.node_map["out"], aug_mna.node_map["out"]);

        // Each clamped op-amp must now have a non-zero n_int_idx.
        for oa in &aug_mna.opamps {
            if oa.vcc.is_finite() || oa.vee.is_finite() {
                assert_ne!(oa.n_int_idx, 0, "op-amp {} should be in BoyleDiodes mode", oa.name);
            }
        }

        // Auto-detect on the un-augmented MNA picks ActiveSet (AC-coupled downstream).
        let resolved = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        assert_eq!(resolved.mode, OpampRailMode::ActiveSet);
    }

    #[test]
    fn multi_opamp_ac_coupled_picks_active_set() {
        // Synthetic: 3 op-amps with AC-coupled downstream stages.
        // Exercises the same path as VCR ALC topology.
        let spice = "\
Multi Op-Amp AC-Coupled Test
R1 in sum1 10k
R2 sum1 out1 100k
C1 out1 mid 100n
R3 mid sum2 10k
R4 sum2 out2 100k
C2 out2 out_ac 100n
R5 out_ac sum3 10k
R6 sum3 out 100k
R7 out 0 100k
U1 0 sum1 out1 OA1
U2 0 sum2 out2 OA1
U3 0 sum3 out OA1
.model OA1 OA(AOL=100k GBW=1e6 ROUT=100 VSAT=13)
";
        let r = parse_and_resolve(spice);
        assert_eq!(r.mode, OpampRailMode::ActiveSet);
        assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
    }

    fn parse_and_refine(spice: &str) -> ResolvedOpampRailMode {
        let netlist = crate::parser::Netlist::parse(spice)
            .unwrap_or_else(|e| panic!("failed to parse: {}", e));
        let mna = MnaSystem::from_netlist(&netlist)
            .unwrap_or_else(|e| panic!("failed to build MNA: {}", e));
        let resolved = resolve_opamp_rail_mode(&mna, OpampRailMode::Auto);
        refine_active_set_for_audio_path(resolved, &mna, &netlist)
    }

    /// Pipe-shouter / tube-screamer feedback-clipper pattern: two antiparallel
    /// diodes span the op-amp output and its inverting input. Before the
    /// starting-node exclusion, the BFS returned true on iteration 0 (the
    /// op-amp output was itself a diode terminal), which demoted the whole
    /// family of overdrive pedals from `ActiveSetBe` to `ActiveSet` and
    /// starved the sub-step Nyquist damping that runs on rail engagement.
    #[test]
    fn feedback_clipper_refines_to_active_set_be() {
        let spice = "\
Feedback Clipper Test (overdrive pedal pattern)
R1 in sum 10k
R2 sum clip_out 100k
C1 clip_out ac_out 1u
R3 ac_out 0 100k
D1 clip_out sum DCLIP
D2 sum clip_out DCLIP
U1 0 sum clip_out OA1
.model DCLIP D(IS=1e-14 N=1.9)
.model OA1 OA(AOL=200k GBW=3e6 ROUT=75 VCC=4.5 VEE=-4.5)
";
        let r = parse_and_refine(spice);
        assert_eq!(
            r.mode,
            OpampRailMode::ActiveSetBe,
            "feedback clipper must refine to ActiveSetBe — the diodes are in \
             the op-amp's own feedback loop, not a downstream control path"
        );
    }

    /// VCR ALC sidechain-rectifier pattern: the op-amp output drives a
    /// rectifier diode *through a series resistor* (`Rsc op_out sc_node`).
    /// The R-hop makes this a genuine control path: the rail voltage
    /// directly drives the downstream device's operating point. Stays on
    /// `ActiveSet` so the steady DC rail value is preserved across a
    /// rail-engage boundary (BE's sub-step would corrupt envelope dynamics).
    #[test]
    fn sidechain_rectifier_stays_on_active_set() {
        let spice = "\
Sidechain Rectifier Test (compressor/ALC pattern)
R1 in sum 10k
R2 sum op_out 100k
C1 op_out ac_out 100n
R3 ac_out 0 100k
Rsc op_out sc_node 10k
D1 sc_node cv_node DRECT
Rrel cv_node 0 2MEG
U1 0 sum op_out OA1
.model DRECT D(IS=2e-9 N=1.906)
.model OA1 OA(AOL=200k GBW=3e6 ROUT=75 VCC=9 VEE=-9)
";
        let r = parse_and_refine(spice);
        assert_eq!(
            r.mode,
            OpampRailMode::ActiveSet,
            "sidechain rectifier must stay on ActiveSet — the op-amp drives \
             the detector diode through Rsc, which IS a downstream control path"
        );
    }
}
