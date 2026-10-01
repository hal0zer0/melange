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
use crate::model_params::ModelClass;
use crate::parser::{Element, Netlist};

use super::{CodegenConfig, CodegenError};

pub mod noise;
pub use noise::*;

pub mod opamp_rail;
pub use opamp_rail::*;

mod matrix_helpers;
use matrix_helpers::*;

mod aux_params;
mod build_dk;
mod build_nodal;
mod device_info;
mod model_card;
mod reductions;
mod semiconductor_params;
mod tube_params;

/// Gmin regularisation conductance added to every node diagonal of the
/// augmented MNA matrix before NR. Prevents singular Jacobians on floating
/// nodes. The DC operating-point solver (`dc_op::build_dc_system`) stamps the
/// same value as its node-diagonal floor. It is the nodal route's only node
/// Gmin: every emitted solve's matrix (main, sub-step, chord, active-set pin,
/// M=0 direct solve) is built from this `G` and adds none of its own, so they
/// all solve the circuit the DC operating point was solved on.
pub(crate) const GMIN_REGULARISATION: f64 = 1e-12;

/// Pivot magnitude below which the local Gaussian-elimination routine
/// declares the matrix singular and aborts codegen with a "missing ground
/// path" diagnostic.
const LU_PIVOT_EPSILON: f64 = 1e-30;

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
    /// Initial-state seed for `v_prev` when the circuit has one or more
    /// `IC=`-bearing capacitors (SPICE `.IC`/UIC semantics). Computed by
    /// temporarily replacing each such capacitor with an ideal DC voltage
    /// source of the specified value and re-solving the (otherwise
    /// unchanged) DC operating point system. `None` when no capacitor in
    /// the netlist declares `IC=` — this keeps IC-free circuits on the
    /// exact same codegen path as before (byte-identical output).
    /// `dc_operating_point`/`DC_OP` above is unaffected and continues to
    /// represent the pure IC-free DC bias point.
    #[serde(default)]
    pub v_prev_ic_seed: Option<Vec<f64>>,
    pub device_slots: Vec<DeviceSlot>,
    /// Per-device MNA node indices, parallel to `device_slots` (same order,
    /// same length). Each entry is the device's terminal list in the MNA's
    /// 1-based index space (0 = ground), e.g. BJT `[collector, base,
    /// emitter]`, pentode `[plate, grid, cathode, screen(, suppressor)]`.
    /// Used by the emitters for the `diag_region_exit_count`
    /// characterization (Vgk > 0 on any pentode, Vbc forward on any BJT),
    /// which needs terminal voltages that are not all N_V rows of a
    /// reduced slot. Empty when the IR was built without an MNA.
    #[serde(default)]
    pub device_node_indices: Vec<Vec<usize>>,
    pub has_dc_sources: bool,
    pub has_dc_op: bool,
    /// M-vector: nonlinear device currents at DC operating point
    #[serde(default)]
    pub dc_nl_currents: Vec<f64>,
    /// M-vector: nonlinear device currents at the IC-seeded operating point
    /// (see `v_prev_ic_seed`) — used ONLY to seed `i_nl_prev` at
    /// construction/reset() time, paired with `v_prev_ic_seed`. Must stay
    /// paired with `v_prev_ic_seed` (never with the plain
    /// `dc_operating_point`/`dc_nl_currents`), or the per-sample state pair
    /// fed into the first trapezoidal step is KCL-inconsistent and NR
    /// diverges a few hundred samples in. `None` iff `v_prev_ic_seed` is
    /// `None`.
    #[serde(default)]
    pub dc_nl_currents_ic_seed: Option<Vec<f64>>,
    /// Charge derivative `q_dot = C·ẋ` at the IC-seeded point, paired with
    /// `v_prev_ic_seed` (trapezoidal builds). The IC solve holds each `IC=`
    /// capacitor with a voltage source, so the circuit is not at rest there:
    /// that source's current is the capacitor's current at t = 0. It is
    /// `RHS_CONST + N_i·i_nl − G·x` on the rows that carry charge, and zero on
    /// the algebraic rows. `None` when `v_prev_ic_seed` is `None` or the build
    /// is backward Euler (which carries no `q_dot`).
    #[serde(default)]
    pub q_dot_ic_seed: Option<Vec<f64>>,
    /// Whether the nonlinear DC OP solver converged
    #[serde(default)]
    pub dc_op_converged: bool,
    /// `.linearize` took its small-signal parameters from a bias solve that
    /// did not converge (a build only gets here with
    /// `--allow-unconverged-dc-op`); recorded in the provenance.
    #[serde(default)]
    pub linearize_bias_unconverged: bool,
    /// DC OP convergence method name (e.g. "DirectNR", "SourceStepping").
    #[serde(default)]
    pub dc_op_method: String,
    /// Railed op-amp outputs at the DC operating point: `dc_op::RailPin::label`
    /// ("none", "pinned N", or the fallback and why).
    #[serde(default)]
    pub dc_op_rail_pin: String,
    /// DC OP total NR iterations used.
    #[serde(default)]
    pub dc_op_iterations: usize,
    /// Whether to include DC blocking filter on outputs.
    pub dc_block: bool,
    /// Saturating (iron-core) inductors: flux devices in the full-LU NR loop.
    /// A saturating tightly-coupled transformer appears here as its T-model
    /// `{ref}_mag` magnetizing inductor (shared-core saturation).
    #[serde(default)]
    pub saturating_inductors: Vec<SaturatingInductorIR>,
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
    /// Authentic circuit noise configuration + source lists. See `NoiseIR`.
    #[serde(default)]
    pub noise: NoiseIR,
    /// Named topology constants emitted into generated code so plugins can
    /// reference nodes, VS rows, and pots by name instead of by numeric index.
    #[serde(default)]
    pub named_constants: NamedConstantsIR,
    /// `.runtime`-bound voltage sources. Codegen emits one `pub <field>: f64`
    /// on `CircuitState` per entry and stamps `rhs[vs_row] += state.<field>`
    /// in both trapezoidal and backward-Euler RHS builders.
    #[serde(default)]
    pub runtime_sources: Vec<RuntimeSourceIR>,
    /// Behavioral (`B`) arbitrary-expression sources. Stamped directly into the
    /// node-space Newton system by the nodal emitter (forces nodal routing).
    #[serde(default)]
    pub behavioral_sources: Vec<BehavioralSourceIR>,
    /// `.param name = value` constants referenced by name in behavioral
    /// expressions (resolved to a baked literal).
    #[serde(default)]
    pub behavioral_param_consts: Vec<(String, f64)>,
    /// Bare plugin-driven scalar params (`.runtime <name> <min> <max> as
    /// <field>`) referenced in behavioral expressions (resolved to a live state
    /// field; setter emitted).
    #[serde(default)]
    pub behavioral_scalar_runtimes: Vec<ScalarRuntimeIR>,
    /// Diagnostic: spectral radius evaluated by the trap-stability
    /// discriminator on the trap `S·A_neg` pair at the shipped rate (the
    /// internal rate under oversampling). 0.0 when the discriminator did
    /// not run (explicit `--backward-euler`, `--force-trap`, or `m == 0`).
    /// Unlike `matrices.spectral_radius_s_aneg` (an emitter-contract value
    /// that the nodal path overwrites with the post-promotion BE rho), this
    /// always holds the trap-side measurement, so `CodegenMeta` can report
    /// the value that actually triggered auto-BE.
    #[serde(default)]
    pub trap_discriminator_rho: f64,
    /// How the shipped integration scheme was selected. Recorded at the
    /// decision site in the IR builders so downstream reporting (CLI summary,
    /// `CodegenMeta`) states the actual reason instead of re-deriving it from
    /// config flags — a directive-pinned BE build is NOT "auto-selected".
    #[serde(default)]
    pub integrator_selection: IntegratorSelection,
    /// Why a default-trapezoidal build stayed trapezoidal or was promoted to
    /// backward Euler: the ring predicate's verdict in one line
    /// (`codegen::ring::RingVerdict::reason`). Empty when the integrator was
    /// pinned (flag, directive, behavioral sources) and nothing was decided.
    #[serde(default)]
    pub integration_reason: String,
    /// The runtime BE-latch's program reference, from the ring predicate's
    /// verdict on this trapezoidal build (`None` when nothing was decided:
    /// such builds carry no latch). See [`BeLatchReference`].
    #[serde(default)]
    pub be_latch_reference: Option<BeLatchReference>,
}

/// What the runtime BE-latch needs to judge a ring against the program that
/// excited it, on the ring predicate's own scale (`codegen::ring`): the
/// passband gain, and the continuous-time poles that can ring at fs/2, whose
/// slowest decay (remapped at the host rate) sets how long the reference
/// remembers the program.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BeLatchReference {
    /// Pink-weighted RMS gain over 20 Hz–20 kHz, primary input to primary
    /// output (`codegen::ring::passband_gain`).
    pub passband_gain: f64,
    /// Continuous-time poles `(re, im)` in rad/s, upper half-plane only.
    pub ring_poles: Vec<(f64, f64)>,
    /// An index-2 pole (exactly `z = −1`) rings forever: hold the reference.
    pub hold: bool,
    /// Backward Euler's worst in-band change relative to the passband (the
    /// ring predicate's `E_BE`), where the comparison holds (at most
    /// `ring::BE_COMPARISON_VALID_REL`); `None` where it does not, and the
    /// latch falls back to the -60 dB threshold alone, as the predicate does.
    #[serde(default)]
    pub be_cost_rel: Option<f64>,
}

/// Why the shipped integration scheme is what it is.
///
/// Precedence mirrors `resolve_integrator_pref` + the per-path forcing rules:
/// CLI flag > `.integrator` directive > behavioral-source forcing (nodal) >
/// auto-promotion > default trapezoidal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum IntegratorSelection {
    /// Trapezoidal, nothing overrode it (the default scheme).
    #[default]
    TrapDefault,
    /// Trapezoidal pinned by `--force-trap` (also opts out of auto-promotion
    /// and the runtime BE-latch net).
    TrapCliFlag,
    /// Trapezoidal pinned by `.integrator trap` in the netlist.
    TrapDirective,
    /// Backward Euler requested by `--backward-euler`.
    BeCliFlag,
    /// Backward Euler pinned by `.integrator be` in the netlist.
    BeDirective,
    /// Backward Euler forced because behavioral `B` sources are stamped
    /// current-only, which is exact under BE (nodal path only).
    BeBehavioral,
    /// Backward Euler auto-promoted by the trap-stability discriminator
    /// (spectral radius over threshold / Nyquist-marginal).
    BeAuto,
}

impl IntegratorSelection {
    /// `true` for every backward-Euler variant.
    #[must_use]
    pub fn is_backward_euler(self) -> bool {
        matches!(
            self,
            Self::BeCliFlag | Self::BeDirective | Self::BeBehavioral | Self::BeAuto
        )
    }

    /// Human-readable label with the reason — used in the generated file's
    /// provenance header so a consumer can tell (e.g.) a `--force-trap` build
    /// from an auto-promoted-BE one without inferring it from `MAX_ITER`.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::TrapDefault => "trapezoidal",
            Self::TrapCliFlag => "trapezoidal (--force-trap)",
            Self::TrapDirective => "trapezoidal (.integrator trap)",
            Self::BeCliFlag => "backward-euler (--backward-euler)",
            Self::BeDirective => "backward-euler (.integrator be)",
            Self::BeBehavioral => "backward-euler (behavioral-source forced)",
            Self::BeAuto => "backward-euler (auto-promoted)",
        }
    }

    /// Machine-readable reason the integration scheme was chosen, for the
    /// generated `// provenance:` JSON. Coarser than `label()` on purpose: a
    /// consumer can assert *why* BE is in effect (an intended contract vs a
    /// solver decision) without string-matching the human label.
    /// `explicit` = user asked (`.integrator be` / `--backward-euler`);
    /// `auto-promoted` = the trap-stability discriminator promoted it;
    /// `behavioral` = forced by behavioral `B` sources; `trap` = trapezoidal.
    #[must_use]
    pub fn integration_source(self) -> &'static str {
        match self {
            Self::TrapDefault | Self::TrapCliFlag | Self::TrapDirective => "trap",
            Self::BeCliFlag | Self::BeDirective => "explicit",
            Self::BeBehavioral => "behavioral",
            Self::BeAuto => "auto-promoted",
        }
    }
}

/// A plugin-driven scalar param in IR form (mirrors
/// `parser::RuntimeScalarDirective`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScalarRuntimeIR {
    pub name: String,
    pub field_name: String,
    pub min: f64,
    pub max: f64,
    /// Default value (clamped into `[min, max]`) used at construction.
    pub default: f64,
}

/// Behavioral (`B`) source in IR form. Mirrors `mna::BehavioralSourceInfo`,
/// carrying the parsed expression so codegen can emit its value + Jacobian.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BehavioralSourceIR {
    pub name: String,
    /// `true` for `V={}` (voltage constraint), `false` for `I={}` (current).
    pub is_voltage: bool,
    pub n_plus_idx: usize,
    pub n_minus_idx: usize,
    /// Node name → resolved index for every node the expression references.
    pub referenced_node_indices: std::collections::BTreeMap<String, usize>,
    pub expr: crate::expr::Expr,
    /// Augmented branch-current row for `V={}` sources (`n_aug` index); `None`
    /// for `I={}`.
    pub aug_row: Option<usize>,
    /// `true` if the expression contains `ddt`/`idt`/`time`.
    pub time_dependent: bool,
}

/// Build the IR view of `mna.behavioral_sources`.
///
/// `aug_row` is left `None` here; the augmented branch-current rows for `V={}`
/// sources are allocated when the nodal emitter consumes these (see
/// `docs/aidocs/BEHAVIORAL_SOURCES.md §Codegen integration plan`).
fn build_behavioral_sources_ir(mna: &MnaSystem) -> Vec<BehavioralSourceIR> {
    mna.behavioral_sources
        .iter()
        .map(|b| BehavioralSourceIR {
            name: b.name.clone(),
            is_voltage: matches!(b.kind, crate::parser::BSourceKind::Voltage),
            n_plus_idx: b.n_plus_idx,
            n_minus_idx: b.n_minus_idx,
            referenced_node_indices: b.referenced_node_indices.clone(),
            expr: b.expr.clone(),
            aug_row: b.aug_row,
            time_dependent: b.expr.is_time_dependent(),
        })
        .collect()
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
    /// Every non-ground node's ORIGINAL (un-sanitized) name paired with its
    /// 0-based index — the SAME index space as `DC_OP`/`dc_operating_point`.
    /// Emitted as the `NODE_NAMES: [&str; N]` parallel array + `dc_op_by_name`
    /// lookup so reading a node's operating point is a lookup, not N recompiles.
    /// Unlike `nodes`, this includes solver-internal
    /// nodes (BJT `basePrime`, transformer branches) so the array is a complete
    /// parallel of `DC_OP`; rows with no node name (augmented VS/inductor
    /// branch-current rows) are left `""`.
    #[serde(default)]
    pub node_names: Vec<(String, usize)>,
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
    /// True when the circuit has inductors: augmented-MNA branch current
    /// variables on rows n_aug..n (L in C).
    #[serde(default)]
    pub augmented_inductors: bool,
    /// Number of devices linearized at DC OP (triodes + BJTs).
    /// Linearized devices are stamped as small-signal conductances in G,
    /// creating high-gain coupling chains that inflate S = A^{-1} entries
    /// without making K = N_V * S * N_I ill-conditioned.
    #[serde(default)]
    pub num_linearized_devices: usize,
    /// The region each `.linearize`d device's small-signal model is valid
    /// in, checked on every sample. A linearized device is a reduced model:
    /// outside that region the answer comes from a model that no longer
    /// describes the device, so the sample counts as a reduced-model exit.
    #[serde(default)]
    pub linearized_checks: Vec<LinearizedCheck>,
    /// Rows whose history is zeroed in every history matrix (`A_neg`,
    /// `A_neg_be`, the sub-step and sub-sample-fire twins): the algebraic
    /// augmented rows `n_nodes..n_aug` (voltage sources, VCVS, ideal
    /// transformers, op-amp internal and VCA rows), minus the parasitic-BJT
    /// internal nodes that `expand_bjt_internal_nodes` appends there. Those are
    /// physical G/C nodes and keep their history. Every emitted rebuild zeroes
    /// exactly these rows, so a runtime rebuild matches the baked constants.
    #[serde(default)]
    pub history_zero_rows: Vec<usize>,
}

/// A resolved `.inject` runtime feedback source.
///
/// Node index is 0-indexed into the MNA node vector (matches
/// [`SolverConfig::input_node`]). The source conductance `1/resistance` is
/// stamped into `g[node][node]` before the kernel is built (so it is baked
/// into `S` and present at the DC operating point), exactly like an input
/// port. See `local-docs/inject-directive-plan.md`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InjectionSpec {
    /// 0-indexed MNA node row where the source is stamped.
    pub node: usize,
    /// Rust identifier naming this injection (emitted in `INJECT_NAMES`).
    pub name: String,
    /// Source impedance in ohms (series `R` for Thevenin, shunt `RSHUNT`
    /// for Norton). Conductance `1/resistance` is stamped into the diagonal.
    pub resistance: f64,
    /// `true` = Norton (runtime value is a CURRENT: `rhs[node] += val`).
    /// `false` = Thevenin (runtime value is a VOLTAGE:
    /// `rhs[node] += (val + val_prev) * G` for trap, `val * G` for BE).
    pub norton: bool,
}

/// A resolved `.tap` raw inner-rate probe.
///
/// Emitted SEPARATELY from output nodes even when a node coincides: taps are
/// raw, pre-decimation inner-rate values (read after Step-6c damping, before
/// the Step-9 output pipeline), whereas outputs are DC-blocked / scaled /
/// decimated.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TapSpec {
    /// 0-indexed MNA node row to read raw each inner sample.
    pub node: usize,
    /// Human-readable label emitted in `TAP_NAMES`.
    pub name: String,
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
    /// Extra input node indices for multi-input (M=0) circuits, beyond the
    /// primary port ([`Self::input_node`]). Empty for single-input, in which
    /// case the emitted code is byte-identical to the pre-multi-input path.
    #[serde(default)]
    pub extra_input_nodes: Vec<usize>,
    /// Per-port resistance parallel to [`Self::extra_input_nodes`]. The primary
    /// port uses [`Self::input_resistance`].
    #[serde(default)]
    pub extra_input_resistances: Vec<f64>,
    /// Oversampling factor (1, 2, or 4). Default 1 (no oversampling).
    #[serde(default = "default_oversampling_factor")]
    pub oversampling_factor: usize,
    /// Output scale factors applied after DC blocking (one per output)
    #[serde(default = "default_output_scales")]
    pub output_scales: Vec<f64>,
    /// Post-DC-block output limiter ceiling in volts. See
    /// [`CodegenConfig::output_clamp_v`] for full docs.
    #[serde(default = "default_output_clamp_v")]
    pub output_clamp_v: f64,
    /// Use backward Euler integration (unconditionally stable, first-order).
    #[serde(default)]
    pub backward_euler: bool,
    /// Emit the runtime BE-latch safety net on a trapezoidal build.
    ///
    /// When `true`, the generated per-sample loop carries a cheap lag-1
    /// anti-correlation detector on the output; if the solver falls into a
    /// self-sustaining Nyquist (`(-1)^n`) limit cycle — the trapezoidal
    /// artifact that a *large-signal* operating point can reach even though
    /// the compile-time ring predicate (`codegen::ring`, evaluated on the
    /// linearised rest state; `docs/aidocs/RING_PREDICATE.md`) kept the build
    /// trapezoidal — it latches to the L-stable backward-Euler path for the
    /// rest of the stream (cleared by `reset()`). The latch's floor applies
    /// the same rule: a ring must be louder than both the ring threshold and
    /// backward Euler's own in-band change before it engages. Set `false` for backward-Euler
    /// builds (nothing to catch) and whenever trap is force-pinned
    /// (`--force-trap` / `.integrator trap`), which opts out of the net.
    /// Only the nodal codegen path emits it today.
    #[serde(default)]
    pub runtime_be_latch: bool,
    /// Emit event-triggered *breakpoint backward-Euler* on a trapezoidal build
    /// with a `.switch` that swaps a capacitor or an inductor (or a glow device).
    ///
    /// A mid-run reactance change leaves the carried charge derivative `q_dot`
    /// built on the old value: after a capacitor change, `alpha·C_new·v_prev`
    /// meets a `q_dot` built on `C_old`. A conductance change (a pot, a
    /// resistor-only switch) does not: the charge-form history carries no `G`.
    ///
    /// When `true`, such a `set_switch_*` (and a lit glow device) arms a one-sample countdown
    /// (`BREAKPOINT_BE_SAMPLES = 1`) that routes the next sample through the
    /// L-stable backward-Euler matrices. The BE sample does not read `q_dot`,
    /// re-seeds it from its own capacitor currents, and damps the mode the step
    /// excited. Exactly one
    /// sample: a second BE sample over-damps and can knock a marginal
    /// self-oscillator (an organ frequency-divider stage under `--force-trap`) into the wrong
    /// equilibrium. Byte-neutral for runs that never call a setter (e.g. golden
    /// fixtures at their default position).
    ///
    /// Independent of `--force-trap`: this corrects a definite discretization
    /// bug at an explicit event, not the heuristic Nyquist-latch net that
    /// force-trap opts out of. Gated off for backward-Euler builds (nothing to
    /// fix). `.runtime R` (audio-rate, continuous) does
    /// NOT arm it — arming every sample would pin BE permanently, and its tiny
    /// per-sample Δg self-corrects.
    #[serde(default)]
    pub breakpoint_be: bool,
    /// Requested nodal sub-path override (see
    /// [`crate::codegen::NodalSubPathOverride`]). `Auto` is the shipping
    /// behaviour; the forcing modes are diagnostic escape hatches.
    #[serde(default)]
    pub nodal_sub_path_override: crate::codegen::NodalSubPathOverride,
    /// Escape hatch for the fail-loud refusal of a section/D/KSUB glow on the
    /// nodal full-LU sub-path (design review). See
    /// [`crate::codegen::CodegenConfig::allow_static_glow_on_full_lu`].
    #[serde(default)]
    pub allow_static_glow_on_full_lu: bool,
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
    /// Why [`Self::opamp_rail_mode`] was chosen, in words (the resolver's
    /// reason). Emitted as `OPAMP_RAIL_MODE_REASON` so the choice is
    /// assertable from the generated code.
    #[serde(default)]
    pub opamp_rail_mode_reason: String,
    /// Emit `CircuitState::recompute_dc_op()` for runtime DC operating-point
    /// re-solve. Default `false` → no `recompute_dc_op` / `settle_dc_op`
    /// methods are emitted. Threaded from
    /// [`CodegenConfig::emit_dc_op_recompute`].
    ///
    /// [`CodegenConfig::emit_dc_op_recompute`]: crate::codegen::CodegenConfig::emit_dc_op_recompute
    #[serde(default)]
    pub emit_dc_op_recompute: bool,
    /// Runtime feedback-injection sources (`.inject`). Empty for decks without
    /// `.inject`, in which case emission is byte-identical to today's path.
    #[serde(default)]
    pub injections: Vec<InjectionSpec>,
    /// Raw inner-rate tap probes (`.tap`). Empty when no `.tap` directive.
    #[serde(default)]
    pub taps: Vec<TapSpec>,
    /// Requested sub-sample fire mode (`--subsample-fire`), kept so the emitter
    /// can distinguish a forced `on` (refuse where unimplemented) from `auto`
    /// (fall back silently). See [`crate::codegen::SubsampleFireMode`].
    #[serde(default)]
    pub subsample_fire_mode: crate::codegen::SubsampleFireMode,
    /// Diagnostic lit sub-step multiplier (`factor * tau`); default 0.5 (= tau/2),
    /// set from `CodegenConfig::subsample_lit_factor`. 0 / unset → treated as 0.5
    /// by the emitter.
    #[serde(default)]
    pub subsample_lit_factor: f64,
    /// Resolved: emit the variable-dt glow-strike breakpoint re-solve. `true`
    /// only on the nodal route with a latched (glow) device and a mode other
    /// than `off`; the emitter additionally clears it on the full-LU sub-path
    /// (Stage A implements nodal-Schur only). `false` → byte-identical to the
    /// pre-feature emitter.
    #[serde(default)]
    pub subsample_fire: bool,
}

fn default_output_nodes() -> Vec<usize> {
    vec![0]
}

impl SolverConfig {
    /// Number of input ports (1 for the single-input case).
    pub fn num_inputs(&self) -> usize {
        1 + self.extra_input_nodes.len()
    }

    /// All input node indices, port 0 first then the extra ports in order.
    pub fn input_node_indices(&self) -> Vec<usize> {
        std::iter::once(self.input_node)
            .chain(self.extra_input_nodes.iter().copied())
            .collect()
    }

    /// All input port resistances, parallel to [`Self::input_node_indices`].
    pub fn input_resistance_values(&self) -> Vec<f64> {
        std::iter::once(self.input_resistance)
            .chain(self.extra_input_resistances.iter().copied())
            .collect()
    }

    /// Number of `.inject` runtime feedback sources.
    pub fn num_inject(&self) -> usize {
        self.injections.len()
    }

    /// Number of `.tap` raw inner-rate probes.
    pub fn num_tap(&self) -> usize {
        self.taps.len()
    }

    /// Whether the generated `process_sample` API differs from the classic
    /// `process_sample(input, state)` shape (i.e. any `.inject` or `.tap`).
    /// When false, emission MUST be byte-identical to the pre-inject path.
    pub fn has_inject_or_tap(&self) -> bool {
        !self.injections.is_empty() || !self.taps.is_empty()
    }
}

fn default_oversampling_factor() -> usize {
    1
}

fn default_output_scales() -> Vec<f64> {
    vec![1.0]
}

fn default_output_clamp_v() -> f64 {
    10.0
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
    if out.chars().next().is_some_and(|c| c.is_ascii_digit()) {
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
    let mut seen: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
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

    // ORIGINAL node names in DC_OP index space (idx - 1, ground excluded).
    // Includes solver-internal nodes so `NODE_NAMES` is a complete parallel of
    // `DC_OP`. `node_map` keys are unique, so no dedupe is needed.
    let node_names: Vec<(String, usize)> = {
        let mut v: Vec<(String, usize)> = mna
            .node_map
            .iter()
            .filter(|(name, idx)| **idx != 0 && name.as_str() != "0")
            .map(|(name, idx)| (name.clone(), *idx - 1))
            .collect();
        v.sort_by_key(|(_, idx)| *idx);
        v
    };

    NamedConstantsIR {
        nodes: nodes_raw,
        vsources,
        pots,
        node_names,
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
    /// History matrix `alpha·C` (algebraic rows zeroed), N×N row-major, at the
    /// codegen sample rate. Trapezoidal builds use the charge form: this plus
    /// the carried charge derivative `q_dot`, see [`charge_form_history`].
    /// Backward-Euler builds: `(1/T)·C`.
    pub a_neg: Vec<f64>,
    /// Nonlinear kernel K = N_v * S * N_i, M×M row-major (default for codegen sample rate)
    pub k: Vec<f64>,
    /// Voltage extraction N_v, M×N row-major
    pub n_v: Vec<f64>,
    /// Current injection N_i, N×M row-major (kernel storage order)
    pub n_i: Vec<f64>,
    /// Constant RHS contribution from DC sources, length N. ×1 on every row:
    /// under both integrators a source enters once, at `n+1`.
    pub rhs_const: Vec<f64>,
    /// Raw conductance matrix G, N×N row-major (sample-rate independent).
    /// Includes input conductance.
    /// An op-amp whose card sets `AOL_TRANSIENT_CAP` has its Gm reduced to the
    /// cap; every other op-amp keeps full Gm stamped.
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
    /// Spectral radius of S * A_neg on the pair the build carries: the
    /// trapezoidal whole-system pair (`A_neg = alpha·C − G`, measured before
    /// the charge-form history replaces it), or the backward-Euler pair on a
    /// BE build (flag, directive, behavioral forcing, or auto-promotion, where
    /// it is recomputed on the BE matrices). Read by the nodal emitter's
    /// Schur-vs-full-LU gate: values > 1 mean the Schur linear prediction
    /// amplifies errors. Only computed for nodal path.
    #[serde(default)]
    pub spectral_radius_s_aneg: f64,
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
    /// where `gm_capped = AOL_TRANSIENT_CAP / r_out`. Zero when the card sets
    /// no cap (or one at or above AOL).
    pub gm_delta: f64,
    /// Transconductance of the VCCS as stamped in the transient matrices,
    /// `min(AOL, AOL_TRANSIENT_CAP) / ROUT` [S].
    #[serde(default)]
    pub gm: f64,
    /// `1 / ROUT` [S]: the linear model's output conductance.
    #[serde(default)]
    pub g_out: f64,
    /// `1 / R_SAG` [S]: the saturated output's conductance to its swing limit.
    #[serde(default)]
    pub g_sag: f64,
}

/// The transient-solve AOL cap an op-amp card asks for with
/// `.model OA(AOL_TRANSIENT_CAP=N)`, or `f64::INFINITY` (no cap).
///
/// An author's key, applied as written. melange adds no cap of its own: a
/// cap lowers AOL in the transient solve only, which moves the answer. (An
/// automatic cap on precision-rectifier op-amps, added against LU back-
/// substitution contamination under a post-solve clamp, was measured
/// unneeded under the charge form and active-set pinning, and wrong: a
/// biased half-wave rectifier sat 4.5 mV off ngspice with it, 0.2 uV
/// without.)
fn effective_aol_cap(oa: &crate::mna::OpampInfo) -> f64 {
    oa.aol_transient_cap
}

/// The DC operating point's saturation for a railed op-amp output: the same
/// the transient rail mode applies, so the first sample does not move it.
pub fn dc_rail_for(mode: crate::codegen::OpampRailMode) -> crate::dc_op::DcRail {
    use crate::codegen::OpampRailMode;
    use crate::dc_op::DcRail;
    match mode {
        OpampRailMode::ActiveSet | OpampRailMode::ActiveSetBe => DcRail::LoadLine,
        OpampRailMode::Hard => DcRail::Terminal,
        // Boyle's catch diodes are devices the DC solve already carries.
        OpampRailMode::BoyleDiodes | OpampRailMode::None => DcRail::Free,
        // Auto is resolved before any DC solve; treat as the default.
        OpampRailMode::Auto => DcRail::LoadLine,
    }
}

/// What a build's DC operating point is solved with, besides the circuit.
#[derive(Debug, Clone, Copy)]
pub struct DcOpRequest {
    /// The build's op-amp rail mode: a railed op-amp sits where it puts it.
    pub opamp_rail_mode: crate::codegen::OpampRailMode,
    /// See [`crate::codegen::CodegenConfig::dc_op_max_iterations`].
    pub max_iterations: Option<usize>,
}

impl From<crate::codegen::OpampRailMode> for DcOpRequest {
    fn from(opamp_rail_mode: crate::codegen::OpampRailMode) -> Self {
        Self {
            opamp_rail_mode,
            max_iterations: None,
        }
    }
}

impl From<&crate::codegen::CodegenConfig> for DcOpRequest {
    fn from(config: &crate::codegen::CodegenConfig) -> Self {
        Self {
            opamp_rail_mode: config.opamp_rail_mode,
            max_iterations: config.dc_op_max_iterations,
        }
    }
}

/// The DC operating-point solver settings of a build of `mna`: every DC solve
/// a build makes (the operating point it ships, the `IC=` seed, the reduction
/// detectors, the `.linearize` bias point, the capacitance preflight) uses
/// these, so they all solve the same problem. A railed op-amp sits where the
/// build's rail mode puts it.
/// A `.linearize`d device's validity region (see
/// [`Topology::linearized_checks`]). Node indices are MNA 1-based (0 =
/// ground); the currents and slopes are the linearization's own.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LinearizedCheck {
    /// Valid while the linear plate current `ip_dc + gm·(vgk − vgk0) +
    /// gp·(vpk − vpk0)` stays at or above zero (a triode's plate current
    /// cannot reverse: below zero the tube is cut off) and the grid below its
    /// conduction onset `grid_onset` (the grid law's 0.3 uA starting point).
    Triode {
        name: String,
        ng: usize,
        np: usize,
        nk: usize,
        ip_dc: f64,
        gm: f64,
        gp: f64,
        vgk0: f64,
        vpk0: f64,
        grid_onset: f64,
    },
    /// Valid in the forward-active region: the linear collector current
    /// `ic_dc + dic_dvbe·(vbe − vbe0) + dic_dvbc·(vbc − vbc0)` keeps its
    /// forward sign (at zero the transistor is cut off) and the B-C junction
    /// stays reverse biased (`vbc_eff <= 0`, the forward-active reduction's
    /// own criterion; forward is saturation).
    Bjt {
        name: String,
        nc: usize,
        nb: usize,
        ne: usize,
        ic_dc: f64,
        dic_dvbe: f64,
        dic_dvbc: f64,
        vbe0: f64,
        vbc0: f64,
        is_pnp: bool,
    },
}

/// One [`LinearizedCheck`] per linearized device of `mna`.
fn linearized_checks(mna: &MnaSystem) -> Vec<LinearizedCheck> {
    let triodes = mna
        .linearized_triodes
        .iter()
        .map(|t| LinearizedCheck::Triode {
            name: t.name.clone(),
            ng: t.ng,
            np: t.np,
            nk: t.nk,
            ip_dc: t.ip_dc,
            gm: t.gm,
            gp: t.gp,
            vgk0: t.vgk0,
            vpk0: t.vpk0,
            grid_onset: t.grid_onset,
        });
    let bjts = mna.linearized_bjts.iter().map(|b| LinearizedCheck::Bjt {
        name: b.name.clone(),
        nc: b.nc,
        nb: b.nb,
        ne: b.ne,
        ic_dc: b.ic_dc,
        dic_dvbe: b.dic_dvbe,
        dic_dvbc: b.dic_dvbc,
        vbe0: b.vbe0,
        vbc0: b.vbc0,
        is_pnp: b.is_pnp,
    });
    triodes.chain(bjts).collect()
}

/// Devices are linearized but no bias point was recorded: the bias solve they
/// were linearized at did not converge (see
/// [`MnaSystem::linearize_bias_nodes`]).
fn linearize_bias_unconverged(mna: &MnaSystem) -> bool {
    (!mna.linearized_bjts.is_empty() || !mna.linearized_triodes.is_empty())
        && mna.linearize_bias_nodes.is_none()
}

pub fn dc_op_config(mna: &MnaSystem, request: impl Into<DcOpRequest>) -> DcOpConfig {
    let request = request.into();
    let mut config = DcOpConfig {
        rail: dc_rail_for(resolve_opamp_rail_mode(mna, request.opamp_rail_mode).mode),
        seed_nodes: mna.linearize_bias_nodes.clone(),
        ..DcOpConfig::default()
    };
    if let Some(n) = request.max_iterations {
        config.max_iterations = n;
    }
    config
}

/// The DC operating point a build of `mna` ships (see [`dc_op_config`]).
pub fn solve_dc_op(
    mna: &MnaSystem,
    netlist: &Netlist,
    request: impl Into<DcOpRequest>,
) -> Result<dc_op::DcOpResult, CodegenError> {
    let device_slots = CircuitIR::build_device_info_with_mna(netlist, Some(mna))?;
    Ok(dc_op::solve_dc_operating_point(
        mna,
        &device_slots,
        &dc_op_config(mna, request),
    ))
}

/// Build an `OpampIR` from the MNA `OpampInfo`, computing the Gm delta
/// for sub-step matrix corrections.
fn opamp_ir_from_info(oa: &crate::mna::OpampInfo) -> OpampIR {
    let aol_cap = effective_aol_cap(oa);
    let aol_eff = oa.aol.min(aol_cap);
    let gm_full = oa.aol / oa.r_out;
    let gm_capped = aol_eff / oa.r_out;
    let gm_delta = (gm_full - gm_capped).max(0.0);
    OpampIR {
        n_out_idx: oa.n_out_idx - 1,
        n_plus_idx: if oa.n_plus_idx > 0 {
            Some(oa.n_plus_idx - 1)
        } else {
            None
        },
        n_minus_idx: if oa.n_minus_idx > 0 {
            Some(oa.n_minus_idx - 1)
        } else {
            None
        },
        vclamp_hi: oa.vcc,
        vclamp_lo: oa.vee,
        sr: oa.sr,
        gm_delta,
        gm: gm_capped,
        g_out: 1.0 / oa.r_out,
        g_sag: 1.0 / oa.r_sag,
    }
}

/// Potentiometer parameters for code generation (topology + range).
///
/// The per-sample Sherman-Morrison correction vectors (su/usu/nv_su/u_ni)
/// were removed — pot changes are handled by per-block `rebuild_matrices`
/// in the generated code (Batch D). SM survives only in the
/// saturating-inductor rank-1 update.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PotentiometerIR {
    /// Nominal conductance 1/R_nom
    pub g_nominal: f64,
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
    /// the `set_runtime_R_<field>` setter and the `<field>()` read-only
    /// accessor. Plugin template skips nih-plug knob emission for these.
    /// Setter body is identical to `.pot` since the 2026-04-20 reseed strip.
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
    /// For 'L' components: row index in the augmented C matrix where the
    /// inductance value lives (c_work[k][k] = L). None for non-inductors.
    #[serde(default)]
    pub augmented_row: Option<usize>,
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
    /// Human-readable label: the `.switch` directive's label if given, else the
    /// controlled component names joined with `+`. Emitted as `SWITCH_LABELS`.
    #[serde(default)]
    pub label: String,
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

/// Saturating (iron-core) inductor.
///
/// A flux device solved inside the full-LU NR loop: flux
/// Φ(i) = L_mag·isat·tanh(i/isat) + L_air·i with L_mag = (1−lair)·l0 and
/// L_air = lair·l0 (so the small-signal inductance is still l0), differential
/// inductance L_diff = L_mag/cosh²(i/isat) + L_air as the Jacobian entry, and a
/// history correction that swaps the baked `α·l0·i_prev` for `α·Φ(i_prev)`
/// (see `SATURATING_TRANSFORMERS.md` §3). L_air is the winding's air-core
/// inductance: past saturation dB/dH falls to µ0, not to zero.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SaturatingInductorIR {
    pub name: String,
    /// Nominal inductance (henries)
    pub l0: f64,
    /// Saturation current (amps), the tanh scale current:
    /// L_diff = L_mag/cosh²(1) + L_air ≈ 0.42·L_mag + L_air at i = isat.
    pub isat: f64,
    /// Row index in the augmented system (C[aug_row][aug_row] = L)
    pub aug_row: usize,
    /// Index into the uncoupled/coupled/transformer inductor arrays for
    /// identifying which inductor this is (for naming constants).
    pub inductor_index: usize,
    /// Air-core fraction L_air/l0 (see the struct doc).
    #[serde(default)]
    pub lair: f64,
    /// Where `lair` came from (`LAIR=`, `CORE=<class>` or the default).
    #[serde(default)]
    pub lair_source: String,
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
    /// A_neg_be matrix (N×N) — backward-Euler history. Empty pattern when the
    /// circuit has no BE fallback. Kept distinct from `a_neg`: a near-cancellation
    /// in `alpha*C - G` can zero an `a_neg` entry that is nonzero in the BE
    /// `alpha_be*C`, so `a_neg`'s pattern cannot be reused to prune A_neg_be.
    pub a_neg_be: MatrixSparsity,
    /// K matrix (M×M) — nonlinear kernel
    pub k: MatrixSparsity,
    /// K_be matrix (M×M) — the backward-Euler kernel, so a trapezoidal Schur
    /// build's BE solve skips exactly the entries a BE build's `k` pattern
    /// skips (the threshold drops tiny values, not only zeros). Empty when
    /// there is no BE kernel.
    #[serde(default)]
    pub k_be: MatrixSparsity,
    /// Sparse LU elimination schedule for G_aug (full LU path only)
    pub lu: Option<LuSparsity>,
    /// G_aug sparsity-pattern density (0.0..1.0 fraction of nonzeros)
    /// computed for the sparse-LU decision on the nodal path. 0.0 when no
    /// pattern was computed (DK path, or `m == 0`). Surfaced through
    /// `CodegenMeta::sparse_lu_density`.
    #[serde(default)]
    pub g_aug_density: f64,
}

// LuSparsity, LuOp, and SPARSITY_THRESHOLD are defined in crate::lu
// and re-imported at the top of this file.

/// Blanket-zero ALL augmented algebraic rows (`n_nodes..mna_n_aug`) in a DK
/// history matrix — same semantics as the nodal branch's blanket zeroing in
/// `from_mna` and the emitted `rebuild_matrices()`.
///
/// A per-type enumeration (VS, VCVS, ideal transformers) misses Boyle
/// op-amp internal rows, current-mode VCA rows (internal node + sense
/// branch), and behavioral-V rows, leaving stale history feedback on those
/// algebraic constraints.
///
/// Row layout on the DK path (verified against mna.rs):
///   0..n_nodes                circuit nodes (keep history)
///   n_nodes..mna_n_aug        VS / VCVS / ideal-xfmr / Boyle op-amp
///                             internal / current-mode VCA (internal +
///                             sense) / behavioral-V rows → zeroed here.
///                             For Boyle internal nodes this is effectively
///                             BE for the dominant pole — deliberate,
///                             matches the nodal branch (unconditionally
///                             stable for high-Gm VCCS).
///   mna_n_aug..n              inductor branch rows appended by
///                             build_augmented_matrices (L lives in C on
///                             these rows) → NOT zeroed, they need
///                             trapezoidal history.
/// BJT internal nodes (expand_bjt_internal_nodes) also land inside
/// n_nodes..n_aug, but only on the nodal route — the DK path never expands
/// them (see melange-cli routing).
/// Build the NoiseIR from the config's noise mode. Shared by `from_mna` and
/// `from_kernel_with_dc_op`, which constructed it identically — each source
/// collector runs only when the mode includes its noise class.
fn build_noise_ir(config: &CodegenConfig, netlist: &Netlist, mna: &MnaSystem) -> NoiseIR {
    NoiseIR {
        mode: config.noise_mode,
        master_seed: config.noise_master_seed,
        thermal_sources: if config.noise_mode.includes_thermal() {
            collect_thermal_noise_sources(netlist, mna)
        } else {
            Vec::new()
        },
        shot_sources: if config.noise_mode.includes_shot() {
            collect_shot_noise_sources(netlist, mna)
        } else {
            Vec::new()
        },
        flicker_sources: if config.noise_mode.includes_full() {
            collect_flicker_noise_sources(netlist, mna)
        } else {
            Vec::new()
        },
        resistor_flicker_sources: if config.noise_mode.includes_full() {
            collect_resistor_flicker_noise_sources(netlist, mna)
        } else {
            Vec::new()
        },
        partition_sources: if config.noise_mode.includes_full() {
            collect_pentode_partition_sources(netlist, mna)
        } else {
            Vec::new()
        },
        opamp_noise_sources: if config.noise_mode.includes_full() {
            collect_opamp_noise_sources(mna)
        } else {
            Vec::new()
        },
    }
}

fn zero_augmented_history_rows(
    a_neg_flat: &mut [f64],
    n: usize,
    n_nodes: usize,
    mna_n_aug: usize,
    bjt_internal: &[crate::mna::BjtTransientInternalNodes],
) {
    for row in history_zero_rows(n, n_nodes, mna_n_aug, bjt_internal) {
        for j in 0..n {
            a_neg_flat[row * n + j] = 0.0;
        }
    }
}

/// The rows [`zero_augmented_history_rows`] zeroes; see
/// [`Topology::history_zero_rows`].
fn history_zero_rows(
    n: usize,
    n_nodes: usize,
    mna_n_aug: usize,
    bjt_internal: &[crate::mna::BjtTransientInternalNodes],
) -> Vec<usize> {
    // Parasitic-BJT internal nodes are appended into [n_nodes, n_aug) by
    // expand_bjt_internal_nodes but are PHYSICAL nodes (real G/C stamps), NOT
    // algebraic VS/inductor constraint rows — they must KEEP their trapezoidal
    // history (A_neg row = alpha*C - G). Zeroing them makes the DC OP not a trap
    // fixed point, which kicks a z=-1 (-1)^n ring on the (capacitor-less) collector
    // row. Exclude them, mirroring dk.rs's is_bjt_internal mask. (design review,
    // 2026-09-13; verified via the parasitic-RB trap ring / noise test.)
    let mut is_bjt_internal = vec![false; mna_n_aug];
    for bn in bjt_internal {
        for idx in [bn.int_base, bn.int_collector, bn.int_emitter]
            .into_iter()
            .flatten()
        {
            if idx < mna_n_aug {
                is_bjt_internal[idx] = true;
            }
        }
    }
    (n_nodes..mna_n_aug.min(n))
        .filter(|&row| !is_bjt_internal[row])
        .collect()
}

/// The charge-form history matrix: `alpha·C` with the algebraic rows zeroed.
/// It multiplies `x_n` in the per-sample RHS
///
/// ```text
///     A·x_{n+1} − N_i·i_nl(x_{n+1}) = RHS_CONST + H·x_n + q_dot_n + b_{n+1}
/// ```
///
/// with `q_dot` the carried charge derivative (`C·ẋ`: a capacitor current on
/// node rows, `dΦ/dt` on inductor branch rows; trapezoidal builds only). The
/// whole-system form `H = alpha·C − G` fed every accepted KCL residual on the
/// algebraic combinations back as a z = −1 memory; this form enforces KCL at
/// `n+1` exactly. Companion-modeled inductors (DK library path) carry their
/// whole known current in their history source, not here. See
/// docs/aidocs/COMPANION_MODELS.md.
fn charge_form_history(c_matrix: &[f64], n: usize, alpha: f64, zero_rows: &[usize]) -> Vec<f64> {
    let mut h: Vec<f64> = c_matrix.iter().map(|&c| alpha * c).collect();
    for &row in zero_rows {
        for j in 0..n {
            h[row * n + j] = 0.0;
        }
    }
    h
}

/// DC sources at ×1: current sources on their node rows, voltage-source
/// values on their extension rows. The constant RHS of both integrators.
fn rhs_const_1x(mna: &MnaSystem, n: usize) -> Vec<f64> {
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
}

/// The charge derivative at an IC-seeded start: see
/// [`CircuitIR::q_dot_ic_seed`]. `matrices` must already be the shipped
/// (charge-form) set.
fn q_dot_at(matrices: &Matrices, n: usize, m: usize, x: &[f64], i_nl: &[f64]) -> Vec<f64> {
    (0..n)
        .map(|i| {
            if matrices.a_neg[i * n..(i + 1) * n].iter().all(|&h| h == 0.0) {
                return 0.0;
            }
            let mut q = matrices.rhs_const.get(i).copied().unwrap_or(0.0);
            for k in 0..m.min(i_nl.len()) {
                q += matrices.n_i[i * m + k] * i_nl[k];
            }
            for j in 0..n.min(x.len()) {
                q -= matrices.g_matrix[i * n + j] * x[j];
            }
            q
        })
        .collect()
}

/// Build the DK trapezoidal forward matrix `A = G + alpha·C`
/// (`alpha = 2·rate`) from raw G/C at an arbitrary rate. The oversampled
/// build ships `S = A⁻¹` at the internal rate; its history is the
/// charge-form `alpha·C` (`charge_form_history`).
fn build_dk_trap_a_at_rate(g_matrix: &[f64], c_matrix: &[f64], n: usize, rate: f64) -> Vec<f64> {
    let alpha = 2.0 * rate;
    let mut a_flat = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..n {
            a_flat[i * n + j] = g_matrix[i * n + j] + alpha * c_matrix[i * n + j];
        }
    }
    a_flat
}

/// Build the DK backward-Euler (S, A_neg, rhs_const) set from raw G/C at an
/// arbitrary rate. Used by the BE + oversampling bake so the shipped
/// constants use ONE integrator convention: A = G + (1/T)·C,
/// A_neg = (1/T)·C (no −G term), rhs_const ×1 — exactly mirroring the
/// os=1 BE branch of `from_kernel_with_dc_op`, evaluated at the internal
/// rate, plus the blanket algebraic-row zeroing that the emitted
/// `rebuild_matrices()` applies (baked constants must equal the first
/// runtime rebuild's output).
#[allow(clippy::too_many_arguments)]
fn build_dk_be_matrices_at_rate(
    g_matrix: &[f64],
    c_matrix: &[f64],
    n: usize,
    n_nodes: usize,
    mna_n_aug: usize,
    rate: f64,
    mna: &MnaSystem,
) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>), CodegenError> {
    let alpha = rate; // BE: alpha = 1/T

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

    // BE A_neg = alpha·C is already zero on most algebraic rows, but
    // Boyle-internal / current-mode-VCA / behavioral-V rows can carry C
    // entries — blanket-zero to match the trap arm and the runtime rebuild.
    zero_augmented_history_rows(
        &mut a_neg_flat,
        n,
        n_nodes,
        mna_n_aug,
        &mna.bjt_internal_nodes,
    );

    let s = invert_flat_matrix(&a_flat, n)?;

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

    Ok((s, a_neg_flat, rhs_const_be))
}

/// Resolve the effective `(backward_euler, force_trap, selection)` triple
/// from the CLI flags and an optional `.integrator` netlist directive.
///
/// Precedence: an explicit CLI flag (`--backward-euler` / `--force-trap`)
/// always wins over the directive; the directive wins over the automatic
/// promotion (the ring predicate, `codegen::ring`). `.integrator be` ⇒ `backward_euler = true`;
/// `.integrator trap` ⇒ `force_trap = true` (suppresses auto-promotion and,
/// downstream, the runtime BE-latch safety net).
///
/// The returned `IntegratorSelection` records WHY, so reporting can
/// distinguish "directive honored" from "auto-promotion fired". It is
/// provisional: the IR builders overwrite it with `BeAuto`/`BeBehavioral`
/// when those forcings apply. Public because the CLI shares it (single
/// source of truth for the precedence rules — e.g. the max-iter auto-tuner
/// must budget for the integrator that actually ships).
pub fn resolve_integrator_flags(
    cli_backward_euler: bool,
    cli_force_trap: bool,
    pref: Option<crate::parser::IntegratorPref>,
) -> (bool, bool, IntegratorSelection) {
    let mut be = cli_backward_euler;
    let mut force_trap = cli_force_trap;
    let mut selection = if be {
        IntegratorSelection::BeCliFlag
    } else if force_trap {
        IntegratorSelection::TrapCliFlag
    } else {
        IntegratorSelection::TrapDefault
    };
    match pref {
        Some(crate::parser::IntegratorPref::Be) if !force_trap => {
            if !be {
                selection = IntegratorSelection::BeDirective;
            }
            be = true;
        }
        Some(crate::parser::IntegratorPref::Trap) if !be => {
            if !force_trap {
                selection = IntegratorSelection::TrapDirective;
            }
            force_trap = true;
        }
        _ => {}
    }
    (be, force_trap, selection)
}

fn resolve_integrator_pref(
    config: &CodegenConfig,
    pref: Option<crate::parser::IntegratorPref>,
) -> (bool, bool, IntegratorSelection) {
    resolve_integrator_flags(config.backward_euler, config.force_trap, pref)
}

/// A default-trapezoidal IR the ring predicate promotes: rebuild it with
/// backward Euler under this configuration, and record this reason.
struct Promotion {
    verdict: crate::codegen::ring::RingVerdict,
    config: CodegenConfig,
    reason: String,
}

impl CircuitIR {
    /// The Newton budget the emitted `MAX_ITER` carries, so provenance and the
    /// console never disagree with the const they describe: the configured
    /// budget, raised to [`crate::codegen::policy::NODAL_MAX_ITER_FLOOR`] on the
    /// nodal route. DK is deliberately not floored.
    pub fn effective_max_iter(&self) -> usize {
        match self.solver_mode {
            SolverMode::Nodal => self
                .solver_config
                .max_iterations
                .max(crate::codegen::policy::NODAL_MAX_ITER_FLOOR),
            SolverMode::Dk => self.solver_config.max_iterations,
        }
    }

    /// Decide whether a finished IR ships as built (`None`, with the verdict
    /// recorded in `integration_reason`) or is rebuilt with backward Euler,
    /// by the ring predicate (`codegen::ring`) on the
    /// linearised trapezoidal system at its DC operating point.
    ///
    /// Only a default-trapezoidal build is decided: a flag, a directive or
    /// behavioral sources already fixed the integrator. The predicate failing
    /// to evaluate is an error, not a silent default.
    fn ring_promotion(
        ir: &mut CircuitIR,
        config: &CodegenConfig,
    ) -> Result<Option<Promotion>, CodegenError> {
        if ir.integrator_selection != IntegratorSelection::TrapDefault {
            return Ok(None);
        }
        let rate = ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
        let sys = match crate::codegen::ring::RingSystem::from_ir(ir) {
            Ok(sys) => sys,
            Err(e) => {
                crate::diag_warn!("Trapezoidal kept without a ring check: {e}");
                ir.integration_reason = format!("ring predicate not evaluated: {}", e.0);
                return Ok(None);
            }
        };
        let verdict = crate::codegen::ring::analyze(&sys)
            .map_err(|e| CodegenError::InvalidKernel(e.to_string()))?;
        let mut reason = verdict.reason(rate);
        if !ir.dc_op_converged {
            reason.push_str(" (linearised at a DC operating point that did not converge)");
        }
        if verdict.promote {
            // One plain sentence by default; the eigenvalue detail is for
            // whoever asks for it. A first-time user sees this on a five-line
            // diode clipper and needs to know whether to act (they do not).
            let cost = match (&verdict.be_error, &verdict.trap_error) {
                (Some(be), Some(tr)) => format!(
                    "It changes the in-band response by up to {:.2} % of the passband level at \
                     {:.0} Hz ({:.1} dB), where the trapezoidal rule would change it by {:.3} % \
                     ({:.1} dB)",
                    100.0 * be.rel,
                    be.hz,
                    be.db(),
                    100.0 * tr.rel,
                    tr.db()
                ),
                _ => "Its in-band cost was not evaluated".to_string(),
            };
            crate::diag_warn!(
                "Using backward Euler integration: the trapezoidal rule would ring or grow at \
                 Nyquist on this circuit, louder than backward Euler's own error. {cost}; \
                 oversampling reduces both. RUST_LOG=info for the numbers."
            );
            log::info!("Auto-enabling backward Euler: {reason}");
            let mut config_be = config.clone();
            if let Some(budget) = config.max_iterations_be_promoted {
                config_be.max_iterations = budget;
            }
            return Ok(Some(Promotion {
                verdict,
                config: config_be,
                reason,
            }));
        }
        log::info!("Trapezoidal: {reason}");
        ir.integration_reason = reason;
        ir.be_latch_reference = Some(BeLatchReference {
            passband_gain: verdict.passband_gain,
            ring_poles: verdict.ring_poles.iter().map(|z| (z.re, z.im)).collect(),
            hold: verdict.index2,
            be_cost_rel: verdict
                .be_error
                .as_ref()
                .map(|be| be.rel)
                .filter(|&rel| rel <= crate::codegen::ring::BE_COMPARISON_VALID_REL),
        });
        Ok(None)
    }

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
        let dc_op_result = match dc_op_result {
            Some(dc) => dc,
            None => solve_dc_op(mna, netlist, config)?,
        };
        let mut ir = Self::build_dk(kernel, mna, netlist, config, &dc_op_result, None)?;
        Self::refuse_self_starting_on_dk(&ir)?;
        let Some(p) = Self::ring_promotion(&mut ir, config)? else {
            return Ok(ir);
        };
        let mut ir = Self::build_dk(
            kernel,
            mna,
            netlist,
            &p.config,
            &dc_op_result,
            Some(&p.verdict),
        )?;
        ir.integration_reason = p.reason;
        Ok(ir)
    }

    /// Refuse a DK build whose DC operating point has a growing pole.
    ///
    /// Under the charge form trapezoidal integration is the bilinear map, so
    /// its propagator grows (`rho > TRAP_BE_PROMOTION_RHO`) exactly when the
    /// DC-OP-linearised continuous system has a right-half-plane pole: the
    /// circuit leaves its operating point on its own, a self-starting
    /// oscillator. Its switching folds are samples Newton cannot solve in one
    /// step; the nodal solver cuts the timestep there, DK cannot. Scope: the
    /// test sees only oscillators that start themselves. A kick-started or
    /// driven regenerative circuit (an astable seeded by an initial condition,
    /// a flip-flop) has a stable DC operating point; the runtime unsolved-
    /// sample counter catches it instead.
    fn refuse_self_starting_on_dk(ir: &CircuitIR) -> Result<(), CodegenError> {
        if !ir.dc_op_converged {
            return Ok(());
        }
        let Ok(sys) = crate::codegen::ring::RingSystem::from_ir(ir) else {
            return Ok(());
        };
        let verdict = crate::codegen::ring::analyze(&sys)
            .map_err(|e| CodegenError::InvalidKernel(e.to_string()))?;
        if verdict.growth {
            return Err(CodegenError::SelfStartingOscillator(format!(
                "the DC operating point has a growing pole (trapezoidal spectral radius \
                 {:.6}): the circuit oscillates on its own. The DK solver cannot contain \
                 its switching; it needs the nodal solver (--solver auto or nodal)",
                verdict.rho
            )));
        }
        Ok(())
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
        Self::from_mna_with_dc_op(mna, netlist, config, None)
    }

    /// [`Self::from_mna`] with the DC operating point the build ships; `None`
    /// solves it ([`solve_dc_op`]).
    pub fn from_mna_with_dc_op(
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        dc_op_result: Option<dc_op::DcOpResult>,
    ) -> Result<Self, CodegenError> {
        let dc_op_result = match dc_op_result {
            Some(dc) => dc,
            None => solve_dc_op(mna, netlist, config)?,
        };
        let mut ir = Self::build_nodal(mna, netlist, config, &dc_op_result, None)?;
        let Some(p) = Self::ring_promotion(&mut ir, config)? else {
            return Ok(ir);
        };
        let mut ir = Self::build_nodal(mna, netlist, &p.config, &dc_op_result, Some(&p.verdict))?;
        ir.integration_reason = p.reason;
        Ok(ir)
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
}

/// The rail-mode reason as recorded in `OPAMP_RAIL_MODE_REASON`, plus a
/// codegen-time notice when an explicit `hard` was chosen against the
/// resolver's verdict (see [`opamp_rail::hard_override_at_risk`]). Never a
/// runtime message: generated code does not print.
fn opamp_rail_reason_with_override(
    mna: &crate::mna::MnaSystem,
    requested: crate::codegen::OpampRailMode,
    resolved: &opamp_rail::ResolvedOpampRailMode,
) -> String {
    let base = resolved.reason.as_str().to_string();
    let at_risk = opamp_rail::hard_override_at_risk(mna, requested);
    if at_risk.is_empty() {
        return base;
    }
    let who = if at_risk.len() == 1 {
        format!("{} output is", at_risk[0])
    } else {
        format!("{} outputs are", at_risk.join(", "))
    };
    crate::diag_warn!(
        "hard rail mode requested; {who} AC-coupled downstream, where hard corrupts \
         capacitor history; auto would pick active-set"
    );
    format!(
        "{base} (auto: {})",
        opamp_rail::OpampRailModeReason::AcCoupledDownstream.as_str()
    )
}

#[cfg(test)]
mod opamp_rail_mode_tests;
