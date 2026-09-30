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

/// Gmin regularisation conductance added to every diagonal of the augmented
/// MNA matrix before NR. Prevents singular Jacobians on floating nodes;
/// matches the runtime `NodalSolver` gmin stamp.
const GMIN_REGULARISATION: f64 = 1e-12;

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
    /// See Oomox plugin roadmap P2 + P3.
    #[serde(default)]
    pub named_constants: NamedConstantsIR,
    /// `.runtime`-bound voltage sources. Codegen emits one `pub <field>: f64`
    /// on `CircuitState` per entry and stamps `rhs[vs_row] += state.<field>`
    /// in both trapezoidal and backward-Euler RHS builders. See Oomox P1.
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
    /// lookup so reading a node's operating point is a lookup, not N recompiles
    /// (openfarf thread 218). Unlike `nodes`, this includes solver-internal
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
    /// Silent samples to process after pot-triggered matrix rebuild (default 64).
    #[serde(default = "default_pot_settle_samples")]
    pub pot_settle_samples: usize,
    /// Use backward Euler integration (unconditionally stable, first-order).
    #[serde(default)]
    pub backward_euler: bool,
    /// Emit the runtime BE-latch safety net on a trapezoidal build.
    ///
    /// When `true`, the generated per-sample loop carries a cheap lag-1
    /// anti-correlation detector on the output; if the solver falls into a
    /// self-sustaining Nyquist (`(-1)^n`) limit cycle — the trapezoidal
    /// artifact that a *large-signal* operating point can reach even though
    /// the compile-time quiescent-OP spectral-radius analysis found trap
    /// stable — it latches to the L-stable backward-Euler path for the rest
    /// of the stream (cleared by `reset()`). Set `false` for backward-Euler
    /// builds (nothing to catch) and whenever trap is force-pinned
    /// (`--force-trap` / `.integrator trap`), which opts out of the net.
    /// Only the nodal codegen path emits it today.
    #[serde(default)]
    pub runtime_be_latch: bool,
    /// Emit event-triggered *breakpoint backward-Euler* on a trapezoidal build
    /// with `.switch`/`.pot` parameters.
    ///
    /// A mid-run component change (a `.switch` toggle or a `.pot` step) leaves
    /// the carried charge derivative `q_dot` built on the old values: after a
    /// capacitor change, `alpha·C_new·v_prev` meets a `q_dot` built on `C_old`.
    ///
    /// When `true`, `set_switch_*`/`set_pot_*` (and a lit glow device) arm a one-sample countdown
    /// (`BREAKPOINT_BE_SAMPLES = 1`) that routes the next sample through the
    /// L-stable backward-Euler matrices. The BE sample does not read `q_dot`,
    /// re-seeds it from its own capacitor currents, and damps the mode the step
    /// excited. Exactly one
    /// sample: a second BE sample over-damps and can knock a marginal
    /// self-oscillator (Farfisa G10 divider under `--force-trap`) into the wrong
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
    /// re-solve (Oomox roadmap P6 / Phase E). Default `false` → output is
    /// byte-identical to pre-Phase-E codegen. Threaded from
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

fn default_pot_settle_samples() -> usize {
    64
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
    /// Spectral radius of S * A_neg (trapezoidal feedback operator).
    /// Values > 1 mean the Schur path is unstable. Only computed for nodal path.
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

/// Build the DK trapezoidal (A, A_neg) pair from raw G/C at an arbitrary
/// rate. Used for the oversampled internal rate: the pair returned here is
/// both what the generated solver ships AND what the auto-BE discriminator
/// must evaluate (rho(S·A_neg) is strongly rate-dependent).
#[allow(clippy::too_many_arguments)]
fn build_dk_trap_matrices_at_rate(
    g_matrix: &[f64],
    c_matrix: &[f64],
    n: usize,
    n_nodes: usize,
    mna_n_aug: usize,
    bjt_internal: &[crate::mna::BjtTransientInternalNodes],
    rate: f64,
) -> (Vec<f64>, Vec<f64>) {
    let alpha = 2.0 * rate;

    // Build A = G + alpha*C, A_neg = alpha*C - G
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

    zero_augmented_history_rows(&mut a_neg_flat, n, n_nodes, mna_n_aug, bjt_internal);

    (a_flat, a_neg_flat)
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
/// spectral-radius promotion. `.integrator be` ⇒ `backward_euler = true`;
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
            crate::diag_warn!(
                "Using backward Euler integration: the trapezoidal rule would ring or grow at \
                 Nyquist on this circuit. BE is stable, at the cost of slightly damping the top \
                 octave (oversampling reduces that). No action needed; RUST_LOG=info for the \
                 numbers."
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

    /// The DK builder, from the operating point `dc_result` the build ships.
    /// `promoted` is the ring predicate's verdict when this
    /// build is the backward-Euler rebuild of a trapezoidal IR it promoted
    /// (see [`Self::ring_promotion`]); `None` builds the scheme the flags and
    /// directive select.
    fn build_dk(
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        dc_result: &dc_op::DcOpResult,
        promoted: Option<&crate::codegen::ring::RingVerdict>,
    ) -> Result<Self, CodegenError> {
        let n = kernel.n; // = n_aug (system dimension)
        let n_nodes = kernel.n_nodes; // original circuit node count
        let m = kernel.m;

        // Resolve the effective integration scheme: CLI flags override the
        // `.integrator` netlist directive, which overrides auto-promotion.
        let (cfg_backward_euler, _, mut integrator_selection) =
            resolve_integrator_pref(config, netlist.integrator);

        if m > crate::dk::MAX_M {
            return Err(CodegenError::UnsupportedTopology(crate::dk::max_m_refusal(
                m,
            )));
        }

        // `--subsample-fire on` is nodal-only: the DK kernel bakes S = A^-1 at
        // compile time, so a variable-dt sub-step (matrices at rate/alpha) has
        // no home here. Refuse loudly rather than emit a solver that silently
        // ignores the request. `auto` simply stays off on this route.
        if config.subsample_fire == crate::codegen::SubsampleFireMode::On {
            return Err(CodegenError::InvalidConfig(
                "--subsample-fire on requires the nodal route: the DK kernel bakes \
                 S = A^-1 at compile time and cannot carry the variable-dt glow-strike \
                 sub-step. Use --solver nodal (or --subsample-fire auto, which is \
                 inert on DK)."
                    .to_string(),
            ));
        }

        // Inductors are augmented-MNA branch rows (DkKernel::from_mna_augmented,
        // as the build makes every inductor kernel): L lives in C on rows
        // n_aug..n. A companion-model kernel (DkKernel::from_mna on an inductor
        // deck, the runtime LinearSolver's form) has no codegen path.
        if !kernel.inductors.is_empty()
            || !kernel.coupled_inductors.is_empty()
            || !kernel.transformer_groups.is_empty()
        {
            return Err(CodegenError::UnsupportedTopology(format!(
                "a DK kernel with {} companion-modelled inductor(s), {} coupled pair(s) \
                 and {} transformer group(s) cannot be code-generated: inductors are \
                 generated as augmented-MNA branch rows. Build the kernel with \
                 DkKernel::from_mna_augmented (as melange_solver::build::build does).",
                kernel.inductors.len(),
                kernel.coupled_inductors.len(),
                kernel.transformer_groups.len(),
            )));
        }
        let augmented_inductors = kernel.n > mna.n_aug;

        let topology = Topology {
            n,
            n_nodes,
            m,
            num_devices: kernel.num_devices,
            n_aug: mna.n_aug,
            augmented_inductors,
            num_linearized_devices: mna.linearized_triodes.len() + mna.linearized_bjts.len(),
            history_zero_rows: history_zero_rows(n, n_nodes, mna.n_aug, &mna.bjt_internal_nodes),
        };

        let os_factor = config.oversampling_factor;
        let internal_rate = config.sample_rate * os_factor as f64;

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

        // When oversampling, the shipped trap matrices are rebuilt at the
        // internal (oversampled) rate. Skipped when BE ships (forced, or this
        // is the promoted rebuild) — no trap pair is shipped then.
        let os_trap_pair = if os_factor > 1 && !cfg_backward_euler && promoted.is_none() {
            let (a_flat, a_neg_flat) = build_dk_trap_matrices_at_rate(
                &g_matrix,
                &c_matrix,
                n,
                n_nodes,
                mna.n_aug,
                &mna.bjt_internal_nodes,
                internal_rate,
            );
            let s = invert_flat_matrix(&a_flat, n)?;
            Some((s, a_neg_flat))
        } else {
            None
        };

        // Auto-promotion to backward Euler is decided on the finished
        // trapezoidal IR by the ring predicate (`codegen::ring`), which needs
        // the DC operating point; this build is then repeated with
        // `promoted` set. See `CircuitIR::ring_promotion`.
        let auto_be = promoted.is_some();
        let trap_discriminator_rho = promoted.map_or(0.0, |v| v.rho);
        let be = cfg_backward_euler || auto_be;

        if auto_be {
            integrator_selection = IntegratorSelection::BeAuto;
        }
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

        let rail_mode = resolve_opamp_rail_mode(mna, config.opamp_rail_mode);
        log::info!(
            "Op-amp rail mode: {} ({})",
            rail_mode.mode,
            rail_mode.reason.as_str()
        );
        let rail_mode_reason =
            opamp_rail_reason_with_override(mna, config.opamp_rail_mode, &rail_mode);

        let solver_config = SolverConfig {
            sample_rate: config.sample_rate,
            alpha,
            tolerance: config.tolerance,
            max_iterations: config.max_iterations,
            input_node: config.input_node,
            output_nodes: config.output_nodes.clone(),
            input_resistance: config.input_resistance,
            extra_input_nodes: config.extra_input_nodes.clone(),
            extra_input_resistances: config.extra_input_resistances.clone(),
            oversampling_factor: os_factor,
            output_scales: config.output_scales.clone(),
            output_clamp_v: config.output_clamp_v,
            pot_settle_samples: config.pot_settle_samples,
            backward_euler: be,
            // DK codegen path does not emit the runtime BE-latch net yet.
            runtime_be_latch: false,
            breakpoint_be: false,
            opamp_rail_mode: rail_mode.mode,
            opamp_rail_mode_reason: rail_mode_reason.clone(),
            emit_dc_op_recompute: config.emit_dc_op_recompute,
            nodal_sub_path_override: config.nodal_sub_path_override,
            allow_static_glow_on_full_lu: config.allow_static_glow_on_full_lu,
            injections: config.injections.clone(),
            taps: config.taps.clone(),
            subsample_fire_mode: config.subsample_fire,
            subsample_lit_factor: config.subsample_lit_factor,
            // DK bakes S = A^-1 at compile time and cannot carry a variable-dt
            // sub-step; `auto` is inert here, `on` is refused above.
            subsample_fire: false,
        };

        let metadata = CircuitMetadata {
            circuit_name: config.circuit_name.clone(),
            title: netlist.title.clone(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        let matrices = if os_factor > 1 && be {
            // BE + oversampling: build backward-Euler matrices at the
            // INTERNAL rate, mirroring the os=1 BE branch below. This branch
            // previously did not exist — the oversampled path unconditionally
            // baked trap matrices (alpha = 2·rate, A_neg = alpha·C − G,
            // rhs_const ×2) even when `be` was true, while
            // `solver_config.backward_euler` made the emitter use BE
            // semantics everywhere else. That mixed integrator shipped a
            // wrong DC fixed point (the ×2 trap rhs_const against BE update
            // equations halves the effective nonlinear bias current) and the
            // first runtime `rebuild_matrices()` silently swapped the
            // convention to consistent BE, stepping the operating point
            // mid-signal.
            let (s, a_neg_flat, rhs_const_be) = build_dk_be_matrices_at_rate(
                &g_matrix,
                &c_matrix,
                n,
                n_nodes,
                mna.n_aug,
                internal_rate,
                mna,
            )?;
            // Diagnostic parity with the nodal path's post-promotion check
            // (previously DK had none at all — a genuinely mis-stamped BE
            // build here would have shipped silently). See
            // `log_be_post_promotion_check` for the accurate/deflated
            // methodology and the "growing pole vs matrix defect" rationale.
            crate::codegen::stability::log_be_post_promotion_check(
                "DK",
                &s,
                &a_neg_flat,
                n,
                &config.input_node_indices(),
            );
            let k = compute_k_from_s(&s, &kernel.n_v, &kernel.n_i, n, m);
            Matrices {
                s,
                a_neg: a_neg_flat,
                k,
                n_v: kernel.n_v.clone(),
                n_i: kernel.n_i.clone(),
                rhs_const: rhs_const_be,
                g_matrix,
                c_matrix,
                a_matrix: Vec::new(),
                a_matrix_be: Vec::new(),
                // Primary integrator is already BE — no BE fallback set
                // (mirrors the os=1 BE branch).
                a_neg_be: Vec::new(),
                rhs_const_be: Vec::new(),
                s_be: Vec::new(),
                k_be: Vec::new(),
                spectral_radius_s_aneg: 0.0,
            }
        } else if os_factor > 1 {
            // Trapezoidal + oversampling: the internal-rate pair was already
            // built above.
            let (s, a_neg_flat) =
                os_trap_pair.expect("internal-rate trap pair built when os>1 and trap ships");

            // Compute K = N_v * S * N_i
            let k = compute_k_from_s(&s, &kernel.n_v, &kernel.n_i, n, m);

            // Compute BE fallback matrices for adaptive per-sample fallback
            let want_be_fallback = !config.backward_euler && !config.disable_be_fallback && m > 0;
            let (s_be, k_be, a_neg_be, rhs_const_be) = if want_be_fallback {
                compute_dk_be_fallback(
                    &g_matrix,
                    &c_matrix,
                    n,
                    m,
                    n_nodes,
                    &kernel.n_v,
                    &kernel.n_i,
                    internal_rate,
                    mna,
                )?
            } else {
                (Vec::new(), Vec::new(), Vec::new(), Vec::new())
            };

            // Charge form: history `alpha·C`, DC ×1. The whole-system
            // `a_neg_flat` was the discriminator's operator only.
            let _ = a_neg_flat;
            Matrices {
                s,
                a_neg: charge_form_history(
                    &c_matrix,
                    n,
                    2.0 * internal_rate,
                    &topology.history_zero_rows,
                ),
                k,
                n_v: kernel.n_v.clone(),
                n_i: kernel.n_i.clone(),
                rhs_const: rhs_const_1x(mna, n),
                g_matrix,
                c_matrix,
                a_matrix: Vec::new(),
                a_matrix_be: Vec::new(),
                a_neg_be,
                rhs_const_be,
                s_be,
                k_be,
                spectral_radius_s_aneg: 0.0,
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
            // #5: Blanket-zero ALL augmented algebraic rows (n_nodes..n_aug) —
            // VS/VCVS/ideal-transformer AND Boyle op-amp internal / current-mode
            // VCA / behavioral-V rows. The former per-type enumeration
            // (VS/VCVS/xfmr only) left stale trapezoidal history on the latter
            // three constraints. Routed through the shared helper, matching the
            // os>1 build_dk_be_matrices_at_rate path. Inductor branch rows
            // (n_aug..n) keep their history and are untouched.
            zero_augmented_history_rows(
                &mut a_neg_flat,
                n,
                mna.n,
                mna.n_aug,
                &mna.bjt_internal_nodes,
            );
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
            // Diagnostic parity with the nodal path's post-promotion check —
            // see `log_be_post_promotion_check` doc comment.
            crate::codegen::stability::log_be_post_promotion_check(
                "DK",
                &s_flat,
                &a_neg_flat,
                n,
                &config.input_node_indices(),
            );
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
            }
        } else {
            // Standard trapezoidal: use kernel matrices directly.
            // Also compute BE fallback matrices for adaptive per-sample fallback.
            let want_be_fallback = !config.disable_be_fallback && m > 0;
            let (s_be, k_be, a_neg_be, rhs_const_be) = if want_be_fallback {
                compute_dk_be_fallback(
                    &g_matrix,
                    &c_matrix,
                    n,
                    m,
                    n_nodes,
                    &kernel.n_v,
                    &kernel.n_i,
                    internal_rate,
                    mna,
                )?
            } else {
                (Vec::new(), Vec::new(), Vec::new(), Vec::new())
            };

            // Charge form: history `alpha·C`, DC ×1. The whole-system
            // `kernel.a_neg` was the discriminator's operator only.
            Matrices {
                s: kernel.s.clone(),
                a_neg: charge_form_history(
                    &c_matrix,
                    n,
                    2.0 * internal_rate,
                    &topology.history_zero_rows,
                ),
                k: kernel.k.clone(),
                n_v: kernel.n_v.clone(),
                n_i: kernel.n_i.clone(),
                rhs_const: rhs_const_1x(mna, n),
                g_matrix,
                c_matrix,
                a_matrix: Vec::new(),
                a_matrix_be: Vec::new(),
                a_neg_be,
                rhs_const_be,
                s_be,
                k_be,
                spectral_radius_s_aneg: 0.0,
            }
        };

        // BE fallback matrices are populated for nonlinear circuits (m>0) unless
        // config.disable_be_fallback is set. Linear circuits (m=0) skip BE
        // fallback since they don't have NR iteration that could diverge.

        let device_slots = Self::build_device_info_with_mna(netlist, Some(mna))?;
        let device_node_indices = Self::device_node_indices_for(&device_slots, mna);

        let pots = kernel
            .pots
            .iter()
            .map(|p| PotentiometerIR {
                g_nominal: p.g_nominal,
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

        // Build switches from MNA resolved info.
        //
        // Inductor branch rows: `rebuild_matrices()` stamps a switched L
        // delta into `c_eff[aug_row][aug_row]`, mirroring the nodal path. See
        // `dk_emitter::emit_switch_methods`.
        let inductor_aug_rows: std::collections::HashMap<String, usize> = if augmented_inductors {
            let mut map = std::collections::HashMap::new();
            let mut var_idx = mna.n_aug;
            for ind in &mna.inductors {
                map.insert(ind.name.to_ascii_uppercase(), var_idx);
                var_idx += 1;
            }
            for ci in &mna.coupled_inductors {
                map.insert(ci.l1_name.to_ascii_uppercase(), var_idx);
                map.insert(ci.l2_name.to_ascii_uppercase(), var_idx + 1);
                var_idx += 2;
            }
            for group in &mna.transformer_groups {
                for (widx, name) in group.winding_names.iter().enumerate() {
                    map.insert(name.to_ascii_uppercase(), var_idx + widx);
                }
                var_idx += group.num_windings;
            }
            map
        } else {
            std::collections::HashMap::new()
        };

        let switches: Vec<SwitchIR> = mna
            .switches
            .iter()
            .enumerate()
            .map(|(idx, sw)| {
                let components = sw
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
                            augmented_row,
                        }
                    })
                    .collect();
                SwitchIR {
                    index: idx,
                    label: sw.label.clone().unwrap_or_else(|| {
                        sw.components
                            .iter()
                            .map(|c| c.name.as_str())
                            .collect::<Vec<_>>()
                            .join("+")
                    }),
                    components,
                    positions: sw.positions.clone(),
                    num_positions: sw.positions.len(),
                    mutual_entries: Vec::new(),
                }
            })
            .collect();

        let has_dc_sources = kernel.rhs_const.iter().any(|&v| v != 0.0);

        let dc_config = dc_op_config(mna, config);
        // Check DC OP significance on the truncated vector (n_aug), not the full
        // n_dc vector which includes inductor branch currents.
        let dc_op_len = dc_result.v_node.len();
        let dc_op_truncated = &dc_result.v_node[..kernel.n.min(dc_op_len)];
        let has_dc_op = dc_op_truncated.iter().any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_op_method = format!("{:?}", dc_result.method);
        let dc_op_rail_pin = dc_result.rail_pin.label();
        let dc_op_iterations = dc_result.iterations;
        // Paired with `dc_operating_point` (plain, non-IC quiescent point) —
        // see the pairing note on `dc_operating_point` below, and the
        // per-sample magnitude/NaN-reset fallback (`process_sample.rs.tera`)
        // which resets `v_prev`/`i_nl_prev` back to this exact pair. Do NOT
        // repoint this at the IC-seeded solve — that was tried and created a
        // *different* v_prev/i_nl_prev mismatch at the reset fallback (which
        // always uses the plain `dc_operating_point`), producing the same
        // failure class it was meant to fix. See `dc_nl_currents_ic_seed`
        // below for the seed actually paired with `v_prev_ic_seed`.
        let dc_nl_currents = dc_result.i_nl.clone();

        // IC=-bearing capacitors: solve a second, independent initial-state
        // operating point (each such cap temporarily replaced by an ideal
        // voltage source of its IC value) used to seed `v_prev`. `None`
        // when the netlist has no `IC=` caps — see `mna.capacitor_ics`.
        //
        // `dc_nl_currents_ic_seed` (i_nl at that same IC-consistent point)
        // is captured alongside it and used ONLY to seed `i_nl_prev` at
        // construction/reset() time (paired with `v_prev_ic_seed`, NEVER
        // with the plain `dc_operating_point`). The IC constraint can move
        // node voltages far from the quiescent bias point (that's the whole
        // purpose of IC=), so `dc_result.i_nl` — device currents evaluated
        // at the *unperturbed* operating point — would be inconsistent with
        // an IC-seeded `v_prev`. Feeding mismatched (v_prev, i_nl_prev) into
        // the first trapezoidal step is the same class of bug documented
        // below at the nodal DC_OP resize comment ("pre-clamping... creates
        // a v_prev / i_nl_prev inconsistency... that slowly drifts the
        // state... before NR blows up").
        let mut dc_nl_currents_ic_seed: Option<Vec<f64>> = None;
        let v_prev_ic_seed = dc_op::solve_ic_seeded_operating_point(mna, &device_slots, &dc_config)
            .map(|ic_result| {
                if !ic_result.converged {
                    crate::diag_warn!(
                        "IC= initial-state solve did not converge (method: {:?}); v_prev seed uses best estimate",
                        ic_result.method
                    );
                }
                dc_nl_currents_ic_seed = Some(ic_result.i_nl.clone());
                let mut v = ic_result.v_node;
                v.resize(kernel.n, 0.0);
                v.truncate(kernel.n);
                v
            });
        let q_dot_ic_seed = match (&v_prev_ic_seed, &dc_nl_currents_ic_seed) {
            (Some(x), Some(i_nl)) if !solver_config.backward_euler => {
                Some(q_dot_at(&matrices, n, m, x, i_nl))
            }
            _ => None,
        };

        if !dc_result.converged && m > 0 {
            crate::diag_warn!(
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
            a_neg_be: if matrices.a_neg_be.is_empty() {
                MatrixSparsity {
                    rows: n,
                    cols: n,
                    nnz: 0,
                    nz_by_row: vec![Vec::new(); n],
                }
            } else {
                analyze_matrix_sparsity(&matrices.a_neg_be, n, n)
            },
            k: analyze_matrix_sparsity(&matrices.k, m, m),
            k_be: if matrices.k_be.len() == m * m && m > 0 {
                analyze_matrix_sparsity(&matrices.k_be, m, m)
            } else {
                MatrixSparsity {
                    rows: m,
                    cols: m,
                    nnz: 0,
                    nz_by_row: vec![Vec::new(); m],
                }
            },
            lu: None, // DK path doesn't use full LU
            g_aug_density: 0.0,
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
        let behavioral_sources = build_behavioral_sources_ir(mna);
        let behavioral_param_consts: Vec<(String, f64)> = netlist
            .params
            .iter()
            .map(|p| (p.name.clone(), p.value))
            .collect();
        let behavioral_scalar_runtimes: Vec<ScalarRuntimeIR> = netlist
            .runtime_scalars
            .iter()
            .map(|r| ScalarRuntimeIR {
                name: r.name.clone(),
                field_name: r.field_name.clone(),
                min: r.min_value,
                max: r.max_value,
                default: r.min_value.clamp(r.min_value, r.max_value),
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
            dc_operating_point: {
                // DC OP may return fewer nodes than kernel.n (e.g., when computed
                // on unexpanded MNA before internal node expansion). Pad with zeros.
                let mut dc = dc_result.v_node.clone();
                dc.resize(kernel.n, 0.0);
                dc.truncate(kernel.n);
                // Do NOT clamp op-amp output nodes to VCC/VEE supply rails
                // here — matching the nodal-path policy. Clamping v_prev
                // while `dc_nl_currents` (→ i_nl_prev) comes from the
                // UNCLAMPED solve leaves the state pair inconsistent: the
                // downstream device states encoded in i_nl no longer match
                // the clamped node voltages, and that inconsistency drifts
                // the state over thousands of samples before NR blows up
                // (observed on the nodal path as the 4kbuscomp failure:
                // ~2300 stable samples then 1e27 V explosion). A consistent
                // state pair beats prettier initial voltages; the emitted
                // per-sample rail handling plus the warmup samples cover
                // the transient from a beyond-rail DC seed.
                dc
            },
            v_prev_ic_seed,
            device_slots,
            device_node_indices,
            has_dc_sources,
            has_dc_op,
            dc_nl_currents,
            dc_nl_currents_ic_seed,
            q_dot_ic_seed,
            dc_op_converged,
            linearize_bias_unconverged: linearize_bias_unconverged(mna),
            dc_op_method,
            dc_op_rail_pin,
            dc_op_iterations,
            dc_block: config.dc_block,
            saturating_inductors: Vec::new(), // DK path: saturation routes to nodal
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
                .map(opamp_ir_from_info)
                .collect(),
            sparsity,
            noise: build_noise_ir(config, netlist, mna),
            named_constants,
            runtime_sources,
            behavioral_sources,
            behavioral_param_consts,
            behavioral_scalar_runtimes,
            trap_discriminator_rho,
            integrator_selection,
            integration_reason: String::new(),
            be_latch_reference: None,
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

    /// The nodal builder; `dc_result` and `promoted` as in [`Self::build_dk`].
    fn build_nodal(
        mna: &MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        dc_result: &dc_op::DcOpResult,
        promoted: Option<&crate::codegen::ring::RingVerdict>,
    ) -> Result<Self, CodegenError> {
        let n_nodes = mna.n;
        let n_aug = mna.n_aug;
        let m = mna.m;

        // Resolve the effective integration scheme: CLI flags override the
        // `.integrator` netlist directive, which overrides auto-promotion.
        let (cfg_backward_euler, cfg_force_trap, mut integrator_selection) =
            resolve_integrator_pref(config, netlist.integrator);

        if m > dk::MAX_M {
            return Err(CodegenError::InvalidConfig(dk::max_m_refusal(m)));
        }

        // Build augmented G/C matrices (includes inductor branch variables)
        let mut aug = mna.build_augmented_matrices();
        let n = aug.n_nodal;

        // Gmin regularization: prevent singular Jacobians on floating nodes.
        // Matches runtime NodalSolver (solver.rs Gmin stamping).
        for i in 0..n_nodes {
            aug.g[i][i] += GMIN_REGULARISATION;
        }

        let sample_rate = config.sample_rate;
        let internal_rate = sample_rate * config.oversampling_factor as f64;
        // Provisional integrator. If the nodal auto-detector decides the trap
        // propagation operator `S*A_neg` is unstable (spectral_radius > 1.002,
        // matching the `schur_unstable` gate in `nodal_emitter.rs`), the
        // promotion block below swaps in the BE matrices already built as the
        // transient fallback and flips `be`/`alpha`/`solver_config` in place.
        let mut alpha = if cfg_backward_euler {
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

        // Build A = G + alpha*C, A_neg = alpha*C - G (trapezoidal) or alpha*C (BE).
        // A/A_neg/A_be/A_neg_be are built once below, after an author's
        // AOL_TRANSIENT_CAP has possibly modified aug.g.
        //
        // Behavioral B-sources are stamped current-only (no trapezoidal history
        // term yet), which is exact under backward Euler (steady state G·v = i)
        // but only half-right under trapezoidal. BE is also the stable choice for
        // strongly-nonlinear sources (atan2 discriminators). Force it on.
        // (Trapezoidal + a behavioral i_prev history term is a future refinement
        // — see docs/aidocs/BEHAVIORAL_SOURCES.md.)
        let be = cfg_backward_euler || !mna.behavioral_sources.is_empty();
        if be && !cfg_backward_euler {
            // Behavioral forcing outranks a `.integrator trap` pin (the
            // current-only stamp is simply wrong under trap), so it also
            // overwrites a provisional trap selection.
            integrator_selection = IntegratorSelection::BeBehavioral;
        }
        // A/A_neg/A_be/A_neg_be are built ONCE below, after the selective
        // Rule-D' Gm cap has (possibly) modified aug.g. A former pre-cap build
        // here was immediately overwritten by that rebuild and never read in
        // between, so only the declarations remain.
        let mut a_flat = vec![0.0f64; n * n];
        let mut a_neg_flat = vec![0.0f64; n * n];
        let mut a_be_flat = vec![0.0f64; n * n];
        let mut a_neg_be_flat = vec![0.0f64; n * n];

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

        // Op-amp transient AOL cap, from the card's `AOL_TRANSIENT_CAP` only:
        // the VCCS stamp's excess Gm is removed from G for the transient
        // matrices (the DC operating point keeps the full AOL). Routing sends
        // such a card nodal, so DK never needs it.
        for oa in &mna.opamps {
            if oa.n_out_idx == 0 {
                continue;
            }
            let aol_cap = effective_aol_cap(oa);
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
            // Un-stamp direction must mirror the (corrected) VCCS stamp:
            // stamp is np -= gm / nm += gm, so the cap removes delta with
            // np += delta / nm -= delta.
            if oa.n_plus_idx > 0 && oa.n_plus_idx - 1 < n {
                aug.g[o][oa.n_plus_idx - 1] += delta;
            }
            if oa.n_minus_idx > 0 && oa.n_minus_idx - 1 < n {
                aug.g[o][oa.n_minus_idx - 1] -= delta;
            }
            log::info!(
                "Selective Gm cap on op-amp {}: AOL {:.0} → {:.0} (delta_Gm={:.1} S)",
                oa.name,
                oa.aol,
                aol_cap,
                delta,
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
        // Re-zero augmented (VS/VCVS/xfmr/VCA/behavioral) history rows in A_neg and
        // A_neg_be — but EXCLUDE parasitic-BJT internal nodes, which are physical
        // G/C nodes that must keep their trapezoidal history (zeroing them makes the
        // DC OP not a trap fixed point → a z=-1 collector-row ring). The helper masks
        // them out; this inline loop previously did not (design review root fix).
        zero_augmented_history_rows(&mut a_neg_flat, n, n_nodes, n_aug, &mna.bjt_internal_nodes);
        zero_augmented_history_rows(
            &mut a_neg_be_flat,
            n,
            n_nodes,
            n_aug,
            &mna.bjt_internal_nodes,
        );

        // Flatten G and C (with the selective Rule-D' Gm cap applied) for codegen constants
        let g_matrix = dk::flatten_matrix(&aug.g, n, n);
        let c_matrix = dk::flatten_matrix(&aug.c, n, n);

        let topology = Topology {
            n,
            n_nodes,
            m,
            num_devices: mna.num_devices,
            n_aug,
            augmented_inductors: true,
            num_linearized_devices: mna.linearized_triodes.len() + mna.linearized_bjts.len(),
            history_zero_rows: history_zero_rows(n, n_nodes, n_aug, &mna.bjt_internal_nodes),
        };

        let rail_mode = resolve_opamp_rail_mode(mna, config.opamp_rail_mode);
        log::info!(
            "Op-amp rail mode: {} ({})",
            rail_mode.mode,
            rail_mode.reason.as_str()
        );
        let rail_mode_reason =
            opamp_rail_reason_with_override(mna, config.opamp_rail_mode, &rail_mode);

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
            extra_input_nodes: config.extra_input_nodes.clone(),
            extra_input_resistances: config.extra_input_resistances.clone(),
            oversampling_factor: config.oversampling_factor,
            output_scales: config.output_scales.clone(),
            output_clamp_v: config.output_clamp_v,
            pot_settle_samples: config.pot_settle_samples,
            backward_euler: be,
            // Resolved after the auto-BE promotion block below (needs the
            // final `solver_config.backward_euler`).
            runtime_be_latch: false,
            breakpoint_be: false,
            opamp_rail_mode: rail_mode.mode,
            opamp_rail_mode_reason: rail_mode_reason.clone(),
            emit_dc_op_recompute: config.emit_dc_op_recompute,
            nodal_sub_path_override: config.nodal_sub_path_override,
            allow_static_glow_on_full_lu: config.allow_static_glow_on_full_lu,
            injections: config.injections.clone(),
            taps: config.taps.clone(),
            subsample_fire_mode: config.subsample_fire,
            subsample_lit_factor: config.subsample_lit_factor,
            // Resolved below next to `breakpoint_be` (needs `has_glow`).
            subsample_fire: false,
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

        // Compute spectral radius of S * A_neg to detect Schur instability.
        // When rho(S * A_neg) > 1, the trapezoidal feedback v_pred = S*(A_neg*v_prev + ...)
        // amplifies errors exponentially. Route to full LU NR instead.
        //
        // Discriminate Nyquist-marginal (eigenvalue near -1) from slow LF
        // (eigenvalue near +1) — see `crate::codegen::stability` for the
        // power-iteration sign analysis. The Nyquist case at rho ≈ 0.999
        // promotes to BE; the slow LF case stays on trap (bilinear
        // preserves those poles exactly).
        let trap_stability = crate::codegen::stability::analyze_trap_stability_deflated(
            &s_flat,
            &a_neg_flat,
            n,
            &config.input_node_indices(),
        );
        let mut spectral_radius_s_aneg = trap_stability.rho;
        // Diagnostic copy of the trap-side rho: `spectral_radius_s_aneg` is
        // overwritten with the post-promotion BE rho when auto-BE fires, but
        // CodegenMeta must report the value that TRIGGERED the promotion.
        // When the build is already BE (flag/directive/behavioral), the pair
        // analyzed above IS the BE pair — no trap discriminator ran, so the
        // field stays 0.0 (matching the DK path and the field contract).
        let trap_discriminator_rho = promoted.map_or(0.0, |v| v.rho);
        if spectral_radius_s_aneg > 0.99 {
            log::info!(
                "Nodal: spectral_radius(S*A_neg) = {:.4} ({} pair), dominant_sign = {:+.0}, \
                 max_abs_s = {:.4e} \
                 (marginally stable; Schur used when K well-conditioned)",
                spectral_radius_s_aneg,
                if be { "BE" } else { "trap" },
                trap_stability.dominant_sign,
                trap_stability.max_abs_s
            );
        }

        // Auto-BE promotion for the nodal path. The decision is the ring
        // predicate's (`codegen::ring`), taken on the finished trapezoidal IR
        // at its DC operating point; this build is repeated with `promoted`
        // set (see `CircuitIR::ring_promotion`). The BE matrices are already
        // built above as the transient fallback (`a_be_flat`,
        // `a_neg_be_flat`, `s_be_flat`, `k_be_flat`, `rhs_const_be`); a
        // promoted build clones them into the primary slot and flips
        // `alpha`/`solver_config.backward_euler` so every downstream emitter
        // picks BE formulas.
        if promoted.is_some() && !be {
            alpha = alpha_be;
            a_flat = a_be_flat.clone();
            a_neg_flat = a_neg_be_flat.clone();
            rhs_const = rhs_const_be.clone();
            s_flat = s_be_flat.clone();
            k_flat = k_be_flat.clone();
            solver_config.backward_euler = true;
            solver_config.alpha = alpha;
            integrator_selection = IntegratorSelection::BeAuto;
            // Recompute rho on BE matrices for the emitter's Schur-vs-full-LU
            // gate (`spectral_radius_s_aneg`, consumed by
            // `nodal_emitter.rs`'s `schur_unstable`). This power iteration is
            // intentionally the coarse, un-deflected, fixed-100-iteration
            // form — it is the historically-calibrated value the emitter
            // gate thresholds (1.002/1.05/1.0) were tuned against (see
            // wurli-power-amp notes below); do not swap it for the shared
            // `analyze_trap_stability_deflated` without re-validating every
            // threshold against the golden-audio suite.
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
            // `new_rho` (above) is the coarse un-deflected metric that drives
            // the emitter's Schur-vs-full-LU gate. Separately, cross-check
            // with the accurate/deflated analyzer and log a diagnostic that
            // correctly distinguishes "genuinely unstable circuit" from "BE
            // matrix-builder defect" — see `log_be_post_promotion_check` doc
            // comment for the full rationale and verification.
            if new_rho > crate::codegen::stability::BE_POST_PROMOTION_LIMIT {
                crate::codegen::stability::log_be_post_promotion_check(
                    "Nodal",
                    &s_flat,
                    &a_neg_flat,
                    n,
                    &config.input_node_indices(),
                );
            }
            spectral_radius_s_aneg = new_rho;
        }

        // Emit the runtime BE-latch safety net for genuine trapezoidal builds
        // with a nonlinear system only: nothing to catch once we are already on
        // backward Euler (by flag, `.integrator be`, behavioral sources, or the
        // promotion above), and `--force-trap` / `.integrator trap` opt out
        // entirely. The `m > 0` gate matches the auto-BE promotion: a passive
        // linear circuit has nothing to seed a Nyquist cycle, so it stays
        // byte-identical (no detector emitted).
        //
        // Saturating inductors make a circuit nonlinear with M = 0 (the flux
        // law lives on an augmented row, not in N_i), so they qualify on their
        // own. They were once excluded because the latch forces the BE fallback
        // every sample and the old decimated saturation path never updated the
        // BE matrices; that path is gone, and the flux device is stamped at
        // every Newton site at the site's own alpha.
        let has_saturating = mna.has_saturating_inductor();
        solver_config.runtime_be_latch =
            !solver_config.backward_euler && !cfg_force_trap && (m > 0 || has_saturating);

        // Event-triggered breakpoint backward-Euler for `.switch`/`.pot` swaps.
        // Independent of `cfg_force_trap` (a targeted correctness fix at an
        // explicit event, not the Nyquist-latch heuristic that force-trap
        // disables). Emitted only when the circuit actually has a discrete
        // conductance-swap parameter and runs on trap; the machinery is
        // byte-inert until a `set_switch_*`/`set_pot_*` call arms it, so golden
        // fixtures (which never toggle) are unaffected. Gated off for BE builds
        // (nothing to fix).
        // Armed only by discrete swaps (set_switch_*/set_pot_*), never by the
        // per-sample `.runtime R` setter — so a `.runtime R`-only circuit would
        // emit machinery nothing ever arms. Gate the flag on switches or a
        // *knob* pot (runtime_field == None), so runtime-R-only circuits stay
        // byte-identical.
        let has_knob_pot = mna.pots.iter().any(|p| p.runtime_field.is_none());
        // A glow-discharge device is a runtime conductance swap of the same
        // kind (RS lit <-> ROFF dark, ~1e5 step) that fires on its own latch
        // instead of on a setter. On the nodal route the lit phase is held on
        // the BE matrices (trap is A- but not L-stable: its damping factor on
        // the stiff lit mode tends to -1 and rings into the cathode diode's
        // breakdown at 44.1-96 kHz), so the machinery must be emitted for
        // glow decks too. DK is not re-armed by the glow (unchanged behaviour).
        let has_glow = mna
            .nonlinear_devices
            .iter()
            .any(|d| d.device_type == crate::mna::NonlinearDeviceType::Glow);
        // An op-amp rail pin or release is not a source: under the charge form
        // the pinned solve and the release commit a consistent q_dot, and a
        // backward-Euler sample there only re-seeds q_dot with a backward
        // difference across the edge (measured: it no longer lowers the
        // residual and costs output accuracy).
        solver_config.breakpoint_be =
            !solver_config.backward_euler && (!mna.switches.is_empty() || has_knob_pot || has_glow);

        // Sub-sample fire: variable-dt breakpoint re-solve at a glow strike.
        // Nodal route only (this builder), latched device required; the
        // emitter clears it again on the full-LU sub-path (Stage A = Schur).
        // `off` → false; `auto`/`on` → gated on a glow device being present.
        // Without a glow device nothing can fire, so the machinery is not
        // emitted and the generated source is byte-identical to pre-feature.
        solver_config.subsample_fire =
            config.subsample_fire != crate::codegen::SubsampleFireMode::Off && has_glow;
        if config.subsample_fire == crate::codegen::SubsampleFireMode::On && !has_glow {
            crate::diag_warn!(
                "--subsample-fire on: circuit has no latched (glow) device; the flag is inert."
            );
        }

        // Charge (companion) form: a trapezoidal build ships the history
        // matrix `alpha·C` and the DC sources at ×1 (they enter at n+1 only).
        // The whole-system `alpha·C − G` built above is what the stability
        // discriminators were calibrated against, so it is replaced only here,
        // after they ran.
        let (a_neg_flat, rhs_const) = if solver_config.backward_euler {
            (a_neg_flat, rhs_const)
        } else {
            (
                charge_form_history(&c_matrix, n, alpha, &topology.history_zero_rows),
                rhs_const_be.clone(),
            )
        };
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
        };

        // The DC OP was solved on this MNA (`mna.g`, which still has the full
        // op-amp Gm; only the `aug.g` copy was stripped).
        let dc_config = dc_op_config(mna, config);
        // Build device info with MNA so FA reductions are reflected in dimensions
        let device_slots = Self::build_device_info_with_mna(netlist, Some(mna))?;

        // Judge significance over exactly what the nodal path emits: all N rows,
        // inductor branch currents included (`dc_operating_point` is resized to
        // `n` below and baked whole). Judging only the first `n_aug` rows dropped
        // the operating point of a circuit whose only DC quantity is an inductor
        // current (a current-biased grounded inductor: every node at 0 V), which
        // then started from i_L = 0 and settled over L/R — seconds, for a
        // henry-class winding on the 1 Ω input.
        let has_dc_op = dc_result.v_node.iter().take(n).any(|&v| v.abs() > 1e-15);
        let dc_op_converged = dc_result.converged;
        let dc_op_method = format!("{:?}", dc_result.method);
        let dc_op_rail_pin = dc_result.rail_pin.label();
        let dc_op_iterations = dc_result.iterations;
        // Paired with `dc_operating_point` (plain, non-IC quiescent point).
        // Do NOT repoint this at the IC-seeded solve — see
        // `dc_nl_currents_ic_seed` below and its DK-path twin comment for
        // why: the plain (dc_operating_point, dc_nl_currents) pair is what
        // the per-sample magnitude/NaN-reset fallback resets state to, and
        // it must stay self-consistent independent of any IC= seed.
        let dc_nl_currents = dc_result.i_nl.clone();
        let has_dc_sources = !mna.voltage_sources.is_empty() || !mna.current_sources.is_empty();

        // IC=-bearing capacitors: solve a second, independent initial-state
        // operating point (each such cap temporarily replaced by an ideal
        // voltage source of its IC value) used to seed `v_prev`. `None` when
        // the netlist has no `IC=` caps — see `mna.capacitor_ics`.
        //
        // `dc_nl_currents_ic_seed` (i_nl at that same IC-consistent point) is
        // captured alongside it and used ONLY to seed `i_nl_prev` at
        // construction/reset() time, paired with `v_prev_ic_seed` — see the
        // identical reasoning in the DK-path twin of this block. It must
        // NEVER be paired with the plain `dc_operating_point`/`dc_nl_currents`
        // (that pairing belongs to the reset fallback, see the
        // "v_prev / i_nl_prev inconsistency" comment a few lines below).
        let mut dc_nl_currents_ic_seed: Option<Vec<f64>> = None;
        let v_prev_ic_seed = dc_op::solve_ic_seeded_operating_point(mna, &device_slots, &dc_config)
            .map(|ic_result| {
                if !ic_result.converged {
                    crate::diag_warn!(
                        "IC= initial-state solve did not converge (method: {:?}); v_prev seed uses best estimate",
                        ic_result.method
                    );
                }
                dc_nl_currents_ic_seed = Some(ic_result.i_nl.clone());
                let mut v = ic_result.v_node;
                v.resize(n, 0.0);
                v.truncate(n);
                v
            });
        let q_dot_ic_seed = match (&v_prev_ic_seed, &dc_nl_currents_ic_seed) {
            (Some(x), Some(i_nl)) if !solver_config.backward_euler => {
                Some(q_dot_at(&matrices, n, m, x, i_nl))
            }
            _ => None,
        };

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

        // Sparsity analysis (K is now computed for Schur complement NR)
        //
        // Behavioral B-source Jacobian stamps (`emit_behavioral_jacobian`) hit
        // positions outside the device N_i·J_dev·N_v envelope — the aug/terminal
        // rows × every referenced-node column. They MUST be part of the symbolic
        // pattern: a position absent from the pattern is never eliminated by the
        // straight-line sparse schedule, silently dropping the stamp and
        // converging to a wrong fixed point.
        let behavioral_stamp_patterns: Vec<lu::BehavioralStamp> = mna
            .behavioral_sources
            .iter()
            .map(|b| lu::BehavioralStamp {
                is_voltage: b.v_ext_idx.is_some(),
                aug_row: b.aug_row,
                n_plus_idx: b.n_plus_idx,
                n_minus_idx: b.n_minus_idx,
                referenced_node_indices: b.referenced_node_indices.values().copied().collect(),
            })
            .collect();
        // Belt-and-braces: if any V={} source somehow lacks its aug_row, the
        // stamp geometry cannot be guaranteed complete — prefer correctness and
        // route to the dense LU (which pivots at runtime and needs no pattern).
        let behavioral_pattern_complete = behavioral_stamp_patterns
            .iter()
            .all(|b| !b.is_voltage || b.aug_row.is_some());
        let mut g_aug_density = 0.0f64;
        let lu_sparsity = if m > 0 && behavioral_pattern_complete {
            // Compute G_aug = A - N_i*J_dev*N_v sparsity pattern
            // (+ behavioral B-source stamp positions)
            let g_aug_pattern = lu::compute_g_aug_pattern(
                &matrices.a_matrix,
                &matrices.n_i,
                &matrices.n_v,
                n,
                m,
                &device_slots,
                &behavioral_stamp_patterns,
            );
            let g_aug_nnz: usize = g_aug_pattern.iter().map(|r| r.len()).sum();
            let density = g_aug_nnz as f64 / (n * n) as f64;
            g_aug_density = density;
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
            a_neg_be: if matrices.a_neg_be.is_empty() {
                MatrixSparsity {
                    rows: n,
                    cols: n,
                    nnz: 0,
                    nz_by_row: vec![Vec::new(); n],
                }
            } else {
                analyze_matrix_sparsity(&matrices.a_neg_be, n, n)
            },
            k: analyze_matrix_sparsity(&matrices.k, m, m),
            k_be: if matrices.k_be.len() == m * m && m > 0 {
                analyze_matrix_sparsity(&matrices.k_be, m, m)
            } else {
                MatrixSparsity {
                    rows: m,
                    cols: m,
                    nnz: 0,
                    nz_by_row: vec![Vec::new(); m],
                }
            },
            lu: lu_sparsity,
            g_aug_density,
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
        let behavioral_sources = build_behavioral_sources_ir(mna);
        let behavioral_param_consts: Vec<(String, f64)> = netlist
            .params
            .iter()
            .map(|p| (p.name.clone(), p.value))
            .collect();
        let behavioral_scalar_runtimes: Vec<ScalarRuntimeIR> = netlist
            .runtime_scalars
            .iter()
            .map(|r| ScalarRuntimeIR {
                name: r.name.clone(),
                field_name: r.field_name.clone(),
                min: r.min_value,
                max: r.max_value,
                default: r.min_value.clamp(r.min_value, r.max_value),
            })
            .collect();

        let ir = CircuitIR {
            metadata,
            topology,
            solver_mode: SolverMode::Nodal,
            solver_config,
            matrices,
            dc_operating_point,
            v_prev_ic_seed,
            device_node_indices: Self::device_node_indices_for(&device_slots, mna),
            device_slots,
            has_dc_sources,
            has_dc_op,
            dc_nl_currents,
            dc_nl_currents_ic_seed,
            q_dot_ic_seed,
            dc_op_converged,
            linearize_bias_unconverged: linearize_bias_unconverged(mna),
            dc_op_method,
            dc_op_rail_pin,
            dc_op_iterations,
            dc_block: config.dc_block,
            pots: mna
                .pots
                .iter()
                .map(|p| PotentiometerIR {
                    g_nominal: p.g_nominal,
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
                            if switch_inductor_names.contains(&l1)
                                || switch_inductor_names.contains(&l2)
                            {
                                if let (Some(&ra), Some(&rb)) =
                                    (inductor_aug_rows.get(&l1), inductor_aug_rows.get(&l2))
                                {
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
                                    if switch_inductor_names.contains(&ni)
                                        || switch_inductor_names.contains(&nj)
                                    {
                                        if let (Some(&ra), Some(&rb)) =
                                            (inductor_aug_rows.get(&ni), inductor_aug_rows.get(&nj))
                                        {
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
                            label: sw.label.clone().unwrap_or_else(|| {
                                sw.components
                                    .iter()
                                    .map(|c| c.name.as_str())
                                    .collect::<Vec<_>>()
                                    .join("+")
                            }),
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
                                        augmented_row,
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
                // aug_row for inductor i is n_aug + i (one augmented row each,
                // matching the switch row mapping); no separate counter needed.
                for (i, ind) in mna.inductors.iter().enumerate() {
                    if let Some(isat) = ind.isat {
                        let (lair, lair_source) = match &ind.shared_core {
                            // Single inductor: the floor is a fraction of its own L.
                            None => {
                                let (lair, src) = crate::parser::resolve_air_floor(ind.air_floor);
                                (lair, src.to_string())
                            }
                            // Shared core: this is the magnetizing branch. Its
                            // floor was resolved against the whole core when the
                            // group was built (mna.rs), as a fraction of this branch.
                            Some(core) => {
                                if let Some(k) = core.implicit_k.filter(|&k| k < 0.9995) {
                                    let floor = core.floor_frac * k;
                                    let k_air = floor / ((1.0 - k) + floor);
                                    crate::diag_warn!(
                                        "Saturating shared core ({}): coupling k = {k} is looser than real \
                                         audio iron (1 - k ~ 1e-5..1e-4); the implied coupling in \
                                         deep saturation is k_air = {k_air:.3}.",
                                        ind.name
                                    );
                                }
                                (core.floor_frac, core.floor_reading.clone())
                            }
                        };
                        // Never silent: a default is announced, and so is an
                        // explicit zero floor.
                        if ind.air_floor.is_none() {
                            let d = crate::parser::DEFAULT_AIR_FLOOR;
                            let whose = if ind.shared_core.is_some() {
                                format!(
                                    "the core's magnetizing inductance floors at {d:e} of the \
                                     reference winding's inductance"
                                )
                            } else {
                                format!(
                                    "its saturated inductance floors at {d:e} of its inductance"
                                )
                            };
                            let what = if ind.shared_core.is_some() {
                                format!("shared core ({})", ind.name)
                            } else {
                                format!("inductor {}", ind.name)
                            };
                            crate::diag_warn!(
                                "Saturating {what}: no LAIR= or CORE= given, so {whose} \
                                 (rule-of-thumb for ungapped steel). Set LAIR=<fraction> from a \
                                 measured or core-data value, or CORE=gapped|steel|nickel."
                            );
                        } else if lair == 0.0 {
                            crate::diag_warn!(
                                "Saturating inductor {}: LAIR=0 gives a zero final slope; driven far \
                                 past ISAT (beyond ~10x) its current is set by a numerical, not a \
                                 physical, floor.",
                                ind.name
                            );
                        }
                        sat_inds.push(SaturatingInductorIR {
                            name: ind.name.clone(),
                            l0: ind.value,
                            isat,
                            aug_row: n_aug + i,
                            inductor_index: i,
                            lair,
                            lair_source,
                        });
                    }
                }
                sat_inds
            },
            opamps: mna
                .opamps
                .iter()
                .filter(|oa| {
                    (oa.vcc.is_finite() || oa.vee.is_finite() || oa.sr.is_finite())
                        && oa.n_out_idx > 0
                })
                .map(opamp_ir_from_info)
                .collect(),
            sparsity,
            noise: build_noise_ir(config, netlist, mna),
            named_constants,
            runtime_sources,
            behavioral_sources,
            behavioral_param_consts,
            behavioral_scalar_runtimes,
            trap_discriminator_rho,
            integrator_selection,
            integration_reason: String::new(),
            be_latch_reference: None,
        };
        // Measured, not a gate (design review): a railing op-amp driving a
        // saturating inductor crosses the core's knee within one sample with
        // the full rail across it, and trapezoidal integration at 1x
        // overshoots the inductor's internal current. The output is not
        // affected, and 4x is accurate.
        let rails_into_core = !ir.saturating_inductors.is_empty()
            && matches!(
                ir.solver_config.opamp_rail_mode,
                crate::codegen::OpampRailMode::ActiveSet
                    | crate::codegen::OpampRailMode::ActiveSetBe
            )
            && ir
                .opamps
                .iter()
                .any(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite());
        if rails_into_core && config.oversampling_factor < 4 {
            crate::diag_warn!(
                "An op-amp that can rail drives a saturating inductor: at 1x, the \
                 inductor's internal current can overshoot by up to ~13 % where the \
                 op-amp rails into the core (output H1 is unaffected); 4x is accurate. \
                 Consider --oversampling 4 (or `.oversampling 4` in the deck)."
            );
        }
        // Measured, not a gate (design review): a railing op-amp switches rail
        // to rail within a sample, and at 1x the harmonics of those edges fold
        // back below Nyquist. `active-set-be` shows less of it only because
        // backward Euler dissipates the edges; oversampling is the remedy.
        let railing_at_1x = rail_mode.reason == OpampRailModeReason::AcCoupledDownstream
            && ir.solver_config.opamp_rail_mode == crate::codegen::OpampRailMode::ActiveSet
            && ir
                .opamps
                .iter()
                .any(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
            && config.oversampling_factor == 1;
        if railing_at_1x {
            crate::diag_warn!(
                "An op-amp here can rail (rail mode active-set, chosen automatically). \
                 Rail clipping makes harmonics above Nyquist, which alias at 1x: on a \
                 single-supply overdrive, a 16 kHz tone at 48 kHz put a 66 Hz alias on \
                 the output at 18x the level of its fundamental; at 4x the alias was \
                 0.5 mV. Consider --oversampling 4 (or `.oversampling 4` in the deck)."
            );
        }
        // In augmented MNA every inductor has its own branch row; an L switch
        // component without one would be stamped into the node block as a
        // capacitance, which is a different circuit. Never emit that.
        for sw in &ir.switches {
            for comp in &sw.components {
                if comp.component_type == 'L' && comp.augmented_row.is_none() {
                    return Err(CodegenError::InvalidConfig(format!(
                        ".switch '{}': inductor {} has no branch row in this build, so \
                         the switch cannot change it.",
                        sw.label, comp.name
                    )));
                }
            }
        }
        Ok(ir)
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
    /// Only pure Ebers-Moll devices qualify (no Gummel-Poon VAF/VAR/IKF/IKR,
    /// no ISE leakage, no self-heating RTH, no ohmic parasitics RB/RC/RE) —
    /// for those the reduction is exact. GP/leaky BJTs are left full-2D even
    /// when forward-active, because the 1D emission has no qb and uses
    /// Ib = Ic/BF. Self-heating BJTs stay 2D because the thermal update reads
    /// the (Ic, Ib) slot pair at (s, s+1). Parasitic-carded BJTs stay 2D
    /// because the FA emission drops RB/RC/RE entirely (gm·RE is O(1) on
    /// power BJTs).
    ///
    /// Call this BEFORE building the final MNA/kernel. If non-empty, rebuild
    /// MNA with `from_netlist_forward_active()` and kernel before calling `from_kernel()`.
    pub fn detect_forward_active_bjts(
        mna: &crate::mna::MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
    ) -> std::collections::HashSet<String> {
        use crate::codegen::BjtFaMode;
        use crate::dc_op;

        // Netlist-shaped slots (this runs before any FA reduction), with the
        // MOSFET body-effect nodes resolved so this DC OP matches the others.
        let mut device_slots = Self::build_device_info(netlist).unwrap_or_default();
        if device_slots.is_empty() {
            return std::collections::HashSet::new();
        }
        Self::resolve_mosfet_nodes(&mut device_slots, mna);

        let dc_result =
            dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config(mna, config));

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
                // FA reduction is only exact for pure Ebers-Moll devices:
                // the 1D emission has no qb (drops Early/IKF) and uses
                // Ib = Ic/BF (drops ISE leakage). Gummel-Poon or ISE-carded
                // BJTs must route full-2D — slower but correct. A frozen-qb
                // 1D enhancement is a recorded follow-up.
                //
                // Self-heating (finite RTH) is also excluded: the thermal
                // update reads the 2D slot pair (Ic at s, Ib at s+1 and the
                // matching Vbe/Vbc rows). A 1D FA slot would make s+1 alias
                // the NEXT device's slot (or run out of bounds).
                //
                // Ohmic parasitics (RB/RC/RE) are also excluded: the 1D FA
                // emission drops them entirely, and gm·RE reaches O(1) on
                // power BJTs (RE=0.5Ω @ 100 mA → gm·RE ≈ 1.9 — a several-x
                // transconductance error on exactly the devices FA targets).
                //
                // Threshold -0.5V provides adequate margin for audio-level signals
                // (typical stage Vce margin is 0.85V in cascaded topologies).
                if vbc_eff < -0.5 {
                    let name = dev.name.to_ascii_uppercase();

                    // `--bjt-fa off`: never reduce — every BJT stays full-2D.
                    if config.bjt_fa_mode == BjtFaMode::Off {
                        log::info!(
                            "BJT '{}' forward-active (Vbc={:.3}V) but --bjt-fa=off — routing full-2D.",
                            name,
                            vbc_eff
                        );
                        continue;
                    }

                    // Self-heating is a STRUCTURAL exclusion, not an accuracy
                    // one: the thermal update reads the 2D (Ic,Ib) slot pair at
                    // (s, s+1); a 1D slot would alias the NEXT device's slot (or
                    // run out of bounds). It is NEVER reduced — not even under
                    // `--bjt-fa=force`.
                    if bp.has_self_heating() {
                        log::info!(
                            "BJT '{}' forward-active (Vbc={:.3}V) but self-heating (RTH finite) — the thermal update needs the 2D (Ic,Ib) slot pair; NOT 1D-reduced even under --bjt-fa=force. Routing full-2D.",
                            name,
                            vbc_eff
                        );
                        continue;
                    }

                    // GP / ISE / ohmic-parasitic devices are ACCURACY-excluded:
                    // the 1D FA emission is structurally valid (uses IS/NF/Vt,
                    // Ib=Ic/BF) but drops qb / leakage / RB-RC-RE. Exact only for
                    // pure Ebers-Moll.
                    if bp.is_gummel_poon() || bp.has_ise() || bp.has_parasitics() {
                        let mechanism = if bp.is_gummel_poon() {
                            "Gummel-Poon params present (VAF/VAR/IKF/IKR) — 1D FA emission has no qb (drops Early effect + high-level injection)"
                        } else if bp.has_ise() {
                            "ISE leakage present — 1D FA emission uses Ib=Ic/BF"
                        } else {
                            "ohmic parasitics present (RB/RC/RE) — 1D FA emission drops them; gm·RE reaches O(1) on power BJTs"
                        };
                        if config.bjt_fa_mode == BjtFaMode::Force {
                            // Explicit user opt-in. The reduction is accuracy-
                            // lossy and NOT safe under signal: the collector
                            // swing modulates qb, which this compile-time
                            // decision cannot see. Warn loudly, per device.
                            crate::diag_warn!(
                                "BJT '{}' FORCE-reduced to 1D by --bjt-fa=force despite: {}. Accuracy is NOT guaranteed under signal (~1-2 dB deviation under hard drive for GP/ISE, larger for parasitics). You requested this — remove --bjt-fa=force for the accuracy-exact full-2D model.",
                                name,
                                mechanism
                            );
                            forward_active.insert(name);
                        } else {
                            // Auto (default): leave full-2D — exact.
                            log::info!(
                                "BJT '{}' is forward-active (Vbc={:.3}V) but NOT 1D-reduced: {}. Routing full-2D (use --bjt-fa=force to override).",
                                name,
                                vbc_eff,
                                mechanism
                            );
                            continue;
                        }
                    } else {
                        // Pure Ebers-Moll: 1D reduction is EXACT (auto + force).
                        log::info!(
                            "BJT '{}' forward-active (Vbc={:.3}V). Using 1D model (exact).",
                            name,
                            vbc_eff
                        );
                        forward_active.insert(name);
                    }
                }
            }
        }
        forward_active
    }

    /// Per-device MNA node indices, parallel to `slots`.
    ///
    /// `build_device_info_with_mna` creates exactly one slot per entry of
    /// `mna.nonlinear_devices`, in the same order (linearized devices are
    /// absent from both). That 1:1 correspondence is what the FA / grid-off /
    /// LDR arms of that builder already rely on; this helper makes it
    /// explicit for the emitters. Returns an empty list (and warns) if the
    /// two ever disagree, so the region-exit characterization is simply
    /// not emitted rather than emitted against the wrong terminals.
    fn device_node_indices_for(
        slots: &[DeviceSlot],
        mna: &crate::mna::MnaSystem,
    ) -> Vec<Vec<usize>> {
        if slots.len() != mna.nonlinear_devices.len() {
            crate::diag_warn!(
                "device_slots ({}) and mna.nonlinear_devices ({}) differ in length; \
                 diag_region_exit_count will not be emitted for this circuit",
                slots.len(),
                mna.nonlinear_devices.len()
            );
            return Vec::new();
        }
        mna.nonlinear_devices
            .iter()
            .map(|d| d.node_indices.clone())
            .collect()
    }

    /// Phase 1b grid-off pentode reduction — selection.
    ///
    /// Returns a `HashMap<String, f64>` mapping pentode name (uppercased)
    /// to the DC-OP-converged `Vg2k = V[screen] - V[cathode]` that the
    /// reduced 2D device freezes. The caller passes the map to
    /// [`MnaSystem::from_netlist_with_grid_off`] (rebuilds with
    /// `dimension: 2` pentode slots); [`build_device_info_with_mna`] then
    /// sets `TubeParams.kind = SharpPentodeGridOff` and `vg2k_frozen` from
    /// the reduced MNA.
    ///
    /// **`force_all == false` (`--tube-grid-fa auto`) never reduces.** The
    /// reduction drops two things that the full 3D model carries:
    ///
    /// 1. `Vg2k` as a live NR dimension. It is frozen at its DC value, but
    ///    `Vg2k = V[screen] - V[cathode]` is cathode-referenced: every
    ///    cathode-biased stage without a bypass capacitor, and every stage
    ///    with a finite screen impedance, has a signal-dependent `Vg2k`, and
    ///    the local negative feedback through `dIp/dVg2k` is lost. Measured
    ///    against ngspice as a small-signal gain error of +2.2% (EF86,
    ///    Rk 4.7k unbypassed), +3.0% (EL84, Rk 150 unbypassed, screen
    ///    bypassed) and +12.3% (EL84, Rk 130 and a 1k screen stop, both
    ///    unbypassed); the linearized prediction from the DC-OP
    ///    sensitivities reproduces all three to four digits. See
    ///    `DEVICE_MODELS.md` "Grid-Off Reduction".
    /// 2. `Ig1`. Exact only while `Vgk <= 0`; the region is classified once
    ///    from the DC OP and never re-checked, so a stage driven into grid
    ///    conduction silently runs a model with no grid current.
    ///
    /// Neither can be bounded at compile time from a quiescent bias point,
    /// so there is no sound automatic selection; `auto` is reserved for a
    /// reduction that is provably neutral (none exists yet) and keeps the
    /// full 3D model. `force_all == true` (`--tube-grid-fa on`) reduces
    /// every non-variable-mu pentode and warns per device.
    ///
    /// Mirrors [`detect_forward_active_bjts`] for the BJT case.
    pub fn detect_grid_off_pentodes(
        mna: &crate::mna::MnaSystem,
        netlist: &Netlist,
        config: &CodegenConfig,
        force_all: bool,
    ) -> std::collections::HashMap<String, f64> {
        use crate::dc_op;

        // Must use build_device_info_with_mna so that FA-reduced BJTs (dim=1) produce
        // the correct start_idx values matching mna.m. Using build_device_info(netlist)
        // without MNA gives unreduced dimensions, causing v_nl OOB when FA reduction
        // has already happened (e.g. Q1 reduced 2D→1D shifts all subsequent start_idx).
        let device_slots = Self::build_device_info_with_mna(netlist, Some(mna)).unwrap_or_default();
        if device_slots.is_empty() {
            return std::collections::HashMap::new();
        }

        // Candidate pentodes: full-3D, sharp (non-variable-mu), with the
        // four terminals the reduction needs. Variable-mu pentodes (6K7,
        // EF89) are excluded outright — they exist for continuously varying
        // bias under sidechain control, the opposite of a frozen screen.
        // Schema `validate()` also rejects that combination.
        let candidates: Vec<(usize, &crate::device_types::TubeParams)> = device_slots
            .iter()
            .enumerate()
            .filter_map(|(slot_idx, slot)| {
                if slot.device_type != DeviceType::Tube || slot.dimension != 3 {
                    return None;
                }
                let tp = match &slot.params {
                    DeviceParams::Tube(tp) if tp.is_pentode() && !tp.is_variable_mu() => tp,
                    _ => return None,
                };
                let dev = mna.nonlinear_devices.get(slot_idx)?;
                // Pentode node order (from `categorize_element` in mna.rs):
                // [plate, grid, cathode, screen] with optional [, suppressor].
                if dev.node_indices.len() < 4 {
                    return None;
                }
                Some((slot_idx, tp))
            })
            .collect();
        if candidates.is_empty() {
            return std::collections::HashMap::new();
        }

        if !force_all {
            for (slot_idx, _) in &candidates {
                log::info!(
                    "Pentode '{}' keeps the full 3D model under --tube-grid-fa auto: the \
                     frozen-Vg2k reduction is not accuracy-neutral (it drops the \
                     Vg2k = V(screen) - V(cathode) feedback through the cathode and \
                     screen impedances, and the Ig1 grid current for Vgk > 0). \
                     `--tube-grid-fa on` opts in.",
                    mna.nonlinear_devices[*slot_idx].name.to_ascii_uppercase()
                );
            }
            return std::collections::HashMap::new();
        }

        let dc_result =
            dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config(mna, config));
        let v_at = |n: usize| -> f64 {
            if n > 0 && n - 1 < dc_result.v_node.len() {
                dc_result.v_node[n - 1]
            } else {
                0.0
            }
        };

        let mut grid_off = std::collections::HashMap::new();
        for (slot_idx, tp) in candidates {
            let dev = &mna.nonlinear_devices[slot_idx];
            let n_plate = dev.node_indices[0];
            let n_grid = dev.node_indices[1];
            let n_cathode = dev.node_indices[2];
            let n_screen = dev.node_indices[3];
            let v_cathode = v_at(n_cathode);
            let vgk = v_at(n_grid) - v_cathode;
            let vg2k = v_at(n_screen) - v_cathode;
            let vpk = v_at(n_plate) - v_cathode;
            let name = dev.name.to_ascii_uppercase();
            // Explicit user opt-in (`--tube-grid-fa on`). The reduction is
            // accuracy-lossy and NOT safe under signal: warn loudly, per
            // device, naming what is dropped. Mirrors `--bjt-fa force`.
            crate::diag_warn!(
                "Pentode '{}' FORCE-reduced to 2D by --tube-grid-fa on (Vgk={:.3}V, \
                 Vg2k={:.3}V frozen, Vpk={:.3}V). Dropped: (1) the live Vg2k = \
                 V(screen) - V(cathode) dimension — its feedback through the cathode \
                 and screen impedances is lost (measured +2% to +12% small-signal gain \
                 error on cathode-biased stages; exact only with an AC-grounded cathode \
                 AND screen); (2) the Ig1 grid current — wrong whenever Vgk > 0 (grid \
                 conduction; diag_region_exit_count counts those samples). Remove \
                 --tube-grid-fa on for the full 3D model.",
                name,
                vgk,
                vg2k,
                vpk
            );
            if vgk >= -(tp.vgk_onset + 0.5) {
                crate::diag_warn!(
                    "Pentode '{}' is NOT biased below grid cutoff at the DC OP \
                     (Vgk={:.3}V, onset {:.2}V): the forced grid-off model drops Ig1 \
                     at a bias where the grid already conducts.",
                    name,
                    vgk,
                    tp.vgk_onset
                );
            }
            grid_off.insert(name, vg2k);
        }
        grid_off
    }

    pub fn build_device_info(netlist: &Netlist) -> Result<Vec<DeviceSlot>, CodegenError> {
        Self::build_device_info_with_mna(netlist, None)
    }

    /// Deterministic per-(device, param) jitter draw.
    ///
    /// Hashes the netlist seed together with the device name and the
    /// uppercase param tag into an FNV-64 accumulator, then runs the
    /// SplitMix64 finalizer so well-correlated input bits can't produce
    /// correlated output bits. The high 53 bits become a uniform double
    /// in [0, 1) and the result is mapped to [-1, 1].
    fn mismatch_draw(seed: u64, device_name: &str, param_name: &str) -> f64 {
        const FNV_PRIME: u64 = 0x100000001b3;
        let mut h = seed ^ 0xcbf29ce484222325u64;
        for b in device_name.as_bytes() {
            h = (h ^ (b.to_ascii_uppercase() as u64)).wrapping_mul(FNV_PRIME);
        }
        // Null byte separator so "Q1" + "SA" can't collide with "Q" + "1SA".
        h = h.wrapping_mul(FNV_PRIME);
        for b in param_name.as_bytes() {
            h = (h ^ (b.to_ascii_uppercase() as u64)).wrapping_mul(FNV_PRIME);
        }
        // SplitMix64 finalizer
        h = (h ^ (h >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        h = (h ^ (h >> 27)).wrapping_mul(0x94d049bb133111eb);
        h ^= h >> 31;
        let u01 = (h >> 11) as f64 / (1u64 << 53) as f64;
        2.0 * u01 - 1.0
    }

    /// Look up the tolerance for `param_name` on `device_class` across all
    /// `.mismatch` specs in the netlist. Returns 0.0 when the directive is
    /// absent or the param isn't listed.
    fn mismatch_tol_for(netlist: &Netlist, device_class: char, param_name: &str) -> f64 {
        // Unit-variation kill switch (`ParseOptions::disable_unit_variation`,
        // set by `melange validate`). Reporting a zero tolerance here makes
        // `apply_mismatch` a bit-identical pass-through for every device and
        // every parameter, which is the `.mismatch` half of the same switch
        // that skips `.tolerance` in the parser. The specs stay on the netlist
        // so the caller can still name what it disabled.
        if netlist.unit_variation_disabled {
            return 0.0;
        }
        let upper = param_name;
        let mut tol = 0.0f64;
        for spec in &netlist.mismatch_specs {
            if spec.device_class != device_class {
                continue;
            }
            for (k, v) in &spec.params {
                if k == upper {
                    tol = *v; // last one wins (documented behavior)
                }
            }
        }
        tol
    }

    /// Multiply `nominal` by `(1 + tol · u)` where `u ∈ [-1, 1]` is drawn
    /// deterministically from the (seed, device, param) triple. When the
    /// tolerance is zero this is a pure pass-through — the return value
    /// is bit-identical to `nominal`.
    fn apply_mismatch(
        netlist: &Netlist,
        device_name: &str,
        param_name: &str,
        device_class: char,
        nominal: f64,
    ) -> f64 {
        let tol = Self::mismatch_tol_for(netlist, device_class, param_name);
        if tol == 0.0 {
            return nominal;
        }
        let seed = netlist.seed.unwrap_or(0);
        let u = Self::mismatch_draw(seed, device_name, param_name);
        nominal * (1.0 + tol * u)
    }

    /// Apply per-device `.mismatch T …` jitter to a tube's Koren parameters.
    ///
    /// Pushing mismatch to the *device* params (not the shared `.model` card)
    /// is what makes a push-pull tube pair audibly asymmetric — the dominant
    /// even-harmonic ("H2") source in an otherwise-balanced push-pull stage,
    /// where identical-model halves cancel even harmonics exactly. Shared by
    /// the `Triode` and `Pentode` arms (both carry `TubeParams`). Bit-identical
    /// pass-through when no `.mismatch T` directive lists the param (tol == 0).
    fn apply_tube_mismatch(netlist: &Netlist, name: &str, p: &mut crate::device_types::TubeParams) {
        p.mu = Self::apply_mismatch(netlist, name, "MU", 'T', p.mu);
        p.ex = Self::apply_mismatch(netlist, name, "EX", 'T', p.ex);
        p.kg1 = Self::apply_mismatch(netlist, name, "KG1", 'T', p.kg1);
        p.kp = Self::apply_mismatch(netlist, name, "KP", 'T', p.kp);
        p.kvb = Self::apply_mismatch(netlist, name, "KVB", 'T', p.kvb);
        // Pentode-only screen-current sensitivity; 0.0 on triodes → skip like
        // the DiodeParams.rs guard so triodes stay pure pass-through.
        if p.kg2 > 0.0 {
            p.kg2 = Self::apply_mismatch(netlist, name, "KG2", 'T', p.kg2);
        }
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
                Element::Diode { name, model, .. } => {
                    let mut params = Self::resolve_diode_params(netlist, model)?;
                    // Per-diode `.mismatch D …` jitter. No-op when the
                    // directive is absent or the param isn't listed.
                    params.is = Self::apply_mismatch(netlist, name, "IS", 'D', params.is);
                    params.n_vt = Self::apply_mismatch(netlist, name, "N", 'D', params.n_vt);
                    if params.rs > 0.0 {
                        params.rs = Self::apply_mismatch(netlist, name, "RS", 'D', params.rs);
                    }
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Diode,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Diode(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
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
                    let mut params = Self::resolve_bjt_params(netlist, model)?;
                    // Per-BJT `.mismatch Q …` jitter. Pushing mismatch to the
                    // *device* params (not the shared `.model`) is what makes
                    // push-pull pairs and antiparallel-style stages audibly
                    // asymmetric even when both transistors point at the same
                    // model card.
                    params.is = Self::apply_mismatch(netlist, name, "IS", 'Q', params.is);
                    params.beta_f = Self::apply_mismatch(netlist, name, "BF", 'Q', params.beta_f);
                    params.beta_r = Self::apply_mismatch(netlist, name, "BR", 'Q', params.beta_r);
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
                        stateful: None,
                    });
                    dim_offset += dim;
                    nl_dev_idx += 1;
                }
                Element::Jfet { name, model, .. } => {
                    let mut params = Self::resolve_jfet_params(netlist, model)?;
                    // Per-JFET `.mismatch J …` jitter on the core transfer
                    // parameters. No-op when the directive is absent.
                    params.idss = Self::apply_mismatch(netlist, name, "IDSS", 'J', params.idss);
                    params.vp = Self::apply_mismatch(netlist, name, "VP", 'J', params.vp);
                    params.lambda =
                        Self::apply_mismatch(netlist, name, "LAMBDA", 'J', params.lambda);
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Jfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Jfet(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
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
                    let mut params = Self::resolve_tube_params(netlist, model)?;
                    // Per-triode `.mismatch T …` jitter (see `apply_tube_mismatch`).
                    Self::apply_tube_mismatch(netlist, name, &mut params);
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Tube,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Tube(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Pentode { name, model, .. } => {
                    let mut params = Self::resolve_pentode_params(netlist, model)?;
                    // Per-pentode `.mismatch T …` jitter (see `apply_tube_mismatch`).
                    // Includes KG2 (screen-current sensitivity) for pentodes.
                    Self::apply_tube_mismatch(netlist, name, &mut params);
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
                                if v.abs() > 1e-15 {
                                    Some(v)
                                } else {
                                    None
                                }
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
                        stateful: None,
                    });
                    dim_offset += dim;
                    nl_dev_idx += 1;
                }
                Element::Mosfet { name, model, .. } => {
                    let mut params = Self::resolve_mosfet_params(netlist, model)?;
                    // Per-MOSFET `.mismatch M …` jitter on the core transfer
                    // parameters. No-op when the directive is absent.
                    params.kp = Self::apply_mismatch(netlist, name, "KP", 'M', params.kp);
                    params.vt = Self::apply_mismatch(netlist, name, "VT", 'M', params.vt);
                    params.lambda =
                        Self::apply_mismatch(netlist, name, "LAMBDA", 'M', params.lambda);
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Mosfet,
                        start_idx: dim_offset,
                        dimension: 2,
                        params: DeviceParams::Mosfet(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful: None,
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
                        stateful: None,
                    });
                    dim_offset += 2;
                    nl_dev_idx += 1;
                }
                Element::Ldr { model, .. } => {
                    let params = Self::resolve_ldr_params(netlist, model)?;
                    // The opaque state-block spec needs node indices (1-based,
                    // 0 = ground), which come from the MNA's built node map via
                    // this device's `nonlinear_devices` entry (node_indices =
                    // [r+, r-, ctrl+, ctrl-]). Present for every codegen path
                    // (all pass `Some(mna)`); `None` only in mna-less helper
                    // paths that never emit the stateful device.
                    let stateful = mna.and_then(|m| {
                        m.nonlinear_devices.get(nl_dev_idx).and_then(|d| {
                            if d.device_type == crate::mna::NonlinearDeviceType::Ldr
                                && d.node_indices.len() >= 4
                            {
                                let ni = &d.node_indices;
                                Some(crate::device_types::StatefulSpec {
                                    state_size: 1,
                                    state_seed: vec![params.r_max],
                                    terminal_nodes: ni.clone(),
                                    driving_nodes: vec![ni[2], ni[3]],
                                })
                            } else {
                                None
                            }
                        })
                    });
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Ldr,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Ldr(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful,
                    });
                    dim_offset += 1;
                    nl_dev_idx += 1;
                }
                Element::Glow { model, .. } => {
                    let params = Self::resolve_glow_params(netlist, model)?;
                    // Frozen-latch stateful spec: 1-element opaque state block
                    // (0.0 = dark, 1.0 = lit), seeded dark. terminal_nodes and
                    // driving_nodes are both the device's own [a, k] pair — the
                    // strike/extinguish thresholds are evaluated on the terminal
                    // voltage each sample. node_indices = [a, k] from the MNA.
                    let stateful = mna.and_then(|m| {
                        m.nonlinear_devices.get(nl_dev_idx).and_then(|d| {
                            if d.device_type == crate::mna::NonlinearDeviceType::Glow
                                && d.node_indices.len() >= 2
                            {
                                let ni = &d.node_indices;
                                // Fixed state layout, gated by INDEPENDENT flags
                                // so neither-on = today's 1-slot latch,
                                // byte-identical. Trailing slots are appended in
                                // a fixed order AFTER the Ī block so the eval-site
                                // Ī indices ([1..1+N]) never shift:
                                //   [0]              latch (0=dark, 1=lit) — always
                                //   [1..1+N]         Ī_i current-lags      — has_sections
                                //   [1+Nsec]         t_off since-extinction — has_d
                                //   [1+Nsec+has_d]   extinction-armed flag  — has_sections
                                //   [2+Nsec+has_d]   i_conv_prev (prev converged I) — has_sections
                                //   [3+Nsec+has_d]   pending_extinction debounce    — has_sections
                                // Ī_i seeded at IFLOOR (never 0 → no ln(i/0), eval
                                // starts on the R_T line with zero extra drop);
                                // t_off seeded LARGE so a cold device strikes at
                                // full VO (D≈0) until it first extinguishes; the
                                // armed flag seeds 0 (disarmed) — the strike
                                // (re)sets it and the extinction test is gated on
                                // it, so a still-forming discharge below IHOLD is
                                // not extinguished before it has ever sustained.
                                // i_conv_prev/pending seed 0: robust extinction
                                // needs a monotone converged crossing confirmed
                                // over a 1-sample debounce (rejects the near-V0
                                // integrator ring), so a single-sample dip in the
                                // through-current never extinguishes the tube.
                                let has_sec = params.has_sections();
                                let has_d = params.has_d();
                                let nsec = if has_sec {
                                    crate::device_types::GlowParams::MAX_SECTIONS
                                } else {
                                    0
                                };
                                // has_sections trailing slots: armed + i_conv_prev
                                // + pending_extinction = 3.
                                let n_sec_trailing = if has_sec { 3 } else { 0 };
                                let mut seed =
                                    vec![0.0f64; 1 + nsec + usize::from(has_d) + n_sec_trailing];
                                for s in seed.iter_mut().skip(1).take(nsec) {
                                    *s = params.ifloor;
                                }
                                if has_d {
                                    seed[1 + nsec] = crate::device_types::GlowParams::T_OFF_SEED;
                                }
                                // armed / i_conv_prev / pending (indices
                                // 1+nsec+has_d .. +2) stay at 0.0 (disarmed, no
                                // prior current, no pending crossing).
                                let state_size = seed.len();
                                let state_seed = seed;
                                Some(crate::device_types::StatefulSpec {
                                    state_size,
                                    state_seed,
                                    terminal_nodes: ni.clone(),
                                    driving_nodes: vec![ni[0], ni[1]],
                                })
                            } else {
                                None
                            }
                        })
                    });
                    slots.push(DeviceSlot {
                        device_type: DeviceType::Glow,
                        start_idx: dim_offset,
                        dimension: 1,
                        params: DeviceParams::Glow(params),
                        has_internal_mna_nodes: false,
                        vg2k_frozen: 0.0,
                        stateful,
                    });
                    dim_offset += 1;
                    nl_dev_idx += 1;
                }
                // An op-amp is a linear VCCS stamped in mna.rs, not a device
                // slot; its card is still checked here like every other
                // class's, so an unknown key is refused rather than warned.
                Element::Opamp { model, .. } => {
                    Self::check_model_params(netlist, model, ModelClass::Opamp)?;
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

        // MOSFET body effect reads V(source) − V(bulk), so the slots need the
        // node indices before ANY consumer evaluates them. Resolving here, not
        // at each call site, is the point: the nodal IR used to solve its DC
        // operating point first and resolve afterwards, so every nodal build
        // baked an operating point without body effect (measured: a
        // choke-loaded common-source stage at V(src) = 1.101 V, the GAMMA=0
        // answer, against ngspice's 0.903 V) and started each render with a
        // transient toward the body-effect bias.
        if let Some(mna) = mna {
            Self::resolve_mosfet_nodes(&mut slots, mna);
        }

        Ok(slots)
    }

    /// Resolve MOSFET source/bulk node indices from MNA nonlinear device info.
    ///
    /// Called after `build_device_info` to populate `source_node` and `bulk_node`
    /// fields in MosfetParams, which are needed for body effect (GAMMA/PHI).
    fn resolve_mosfet_nodes(slots: &mut [DeviceSlot], mna: &MnaSystem) {
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
            }
        }
    }

    /// Resolve diode model parameters from the netlist, with validation.
    ///
    /// Resolution order: explicit `.model` param → catalog → generic default.
    fn resolve_diode_params(netlist: &Netlist, model: &str) -> Result<DiodeParams, CodegenError> {
        let vt = melange_primitives::VT_ROOM;
        let cat = melange_devices::catalog::diodes::lookup(model);
        let is = match Self::lookup_model_param(netlist, model, "IS").or_else(|| cat.map(|c| c.is))
        {
            Some(v) => v,
            None => {
                // SPICE default diode (IS=1e-14, N=1.0) for ngspice parity.
                // The old fallback was a chimera: 1N4148's IS (2.52e-9) paired
                // with N=1.0 — matched neither the SPICE default nor a 1N4148.
                crate::diag_warn!(
                    "Diode model '{}' not in catalog and no IS given — falling back to the SPICE default diode (IS=1e-14, N=1.0)",
                    model
                );
                1e-14
            }
        };
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

        // Reverse breakdown current (optional; SPICE3f5 default IBV=1e-3).
        // A smaller default shifts the breakdown knee ~0.4·N V past BV.
        let ibv = Self::lookup_model_param(netlist, model, "IBV").unwrap_or(1e-3);
        if ibv.is_finite() {
            validate_positive_finite(ibv, "diode model IBV")?;
        }

        // Self-heating parameters (optional). Defaults match BjtParams so
        // `.model D(RTH=50)` with everything else implicit gives a sensible
        // silicon diode with 1 ms thermal memory.
        let rth = Self::lookup_model_param(netlist, model, "RTH").unwrap_or(f64::INFINITY);
        if rth.is_finite() && rth <= 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model RTH must be positive (or infinite to disable), got {rth}"
            )));
        }

        let cth = Self::lookup_model_param(netlist, model, "CTH").unwrap_or(1e-3);
        if cth < 0.0 || !cth.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model CTH must be non-negative and finite, got {cth}"
            )));
        }
        if cth == 0.0 {
            log::info!(
                "Diode model '{}': CTH=0 — thermal state has no memory; junction temperature tracks dissipation quasi-statically (Tj = TAMB + RTH·P each sample)",
                model
            );
        }

        let xti = Self::lookup_model_param(netlist, model, "XTI").unwrap_or(3.0);
        if !xti.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model XTI must be finite, got {xti}"
            )));
        }

        let eg = Self::lookup_model_param(netlist, model, "EG").unwrap_or(1.11);
        if eg <= 0.0 || !eg.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model EG must be positive and finite, got {eg}"
            )));
        }

        let tamb =
            Self::lookup_model_param(netlist, model, "TAMB").unwrap_or(melange_primitives::T_NOM);
        if tamb <= 0.0 || !tamb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "diode model TAMB must be positive and finite, got {tamb}"
            )));
        }

        // The card is SPICE's, extracted at TNOM; the device sits at TAMB.
        // Scale IS and N·Vt there with the SPICE3 diode law (ngspice
        // `diotemp.c`). Self-heating then moves Tj from TAMB with the same law
        // written relative to TAMB (`emit_self_heating_thermal_updates`); the
        // law composes, so a junction at Tj sees exactly IS(TNOM -> Tj). At
        // TAMB = TNOM every factor is exactly 1.
        let t = tamb / melange_primitives::T_NOM;
        let vt = vt * t;
        let is = is * t.powf(xti / n) * ((t - 1.0) * eg / (n * vt)).exp();
        validate_positive_finite(is, "diode model IS at TAMB")?;

        Self::check_model_params(netlist, model, ModelClass::Diode)?;
        // NOTE: no warn_unresolved_model() here — the diode resolver already
        // emits its own dedicated fallback warning in the IS-resolution arm
        // above ("not in catalog and no IS given — falling back to the SPICE
        // default diode"). Adding the general warning would double-warn.

        Ok(DiodeParams {
            is,
            n_vt: n * vt,
            cjo,
            rs,
            bv,
            ibv,
            rth,
            cth,
            xti,
            eg,
            tamb,
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
        // Card, then catalog part, then the SPICE / ngspice default (IS 1e-16,
        // BF 100, BR 1): a card that omits a parameter means what it means in
        // SPICE.
        let is = Self::lookup_model_param(netlist, model, "IS")
            .or_else(|| cat.map(|c| c.is))
            .unwrap_or(1e-16);
        let beta_f = Self::lookup_model_param(netlist, model, "BF")
            .or_else(|| cat.map(|c| c.beta_f))
            .unwrap_or(100.0);
        let beta_r = Self::lookup_model_param(netlist, model, "BR")
            .or_else(|| cat.map(|c| c.beta_r))
            .unwrap_or(1.0);

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

        // Depletion-cap parameters (SPICE defaults: VJ = 0.75 V, MJ = 0.33, FC = 0.5).
        // VJ must be strictly positive so `(1 - V/VJ)` is well-defined.
        // MJ is typically in [0.2, 0.5]; we accept anything finite and non-negative.
        // FC must be in [0, 0.95] — values at or above 1 would put the tangent
        // extension inside the singular region of the depletion formula.
        let vje = Self::lookup_model_param(netlist, model, "VJE").unwrap_or(0.75);
        if vje <= 0.0 || !vje.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model VJE must be positive and finite, got {vje}"
            )));
        }
        let mje = Self::lookup_model_param(netlist, model, "MJE").unwrap_or(0.33);
        if mje < 0.0 || !mje.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model MJE must be non-negative and finite, got {mje}"
            )));
        }
        let vjc = Self::lookup_model_param(netlist, model, "VJC").unwrap_or(0.75);
        if vjc <= 0.0 || !vjc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model VJC must be positive and finite, got {vjc}"
            )));
        }
        let mjc = Self::lookup_model_param(netlist, model, "MJC").unwrap_or(0.33);
        if mjc < 0.0 || !mjc.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model MJC must be non-negative and finite, got {mjc}"
            )));
        }
        let fc = Self::lookup_model_param(netlist, model, "FC").unwrap_or(0.5);
        if !fc.is_finite() || !(0.0..=0.95).contains(&fc) {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model FC must be in [0.0, 0.95], got {fc}"
            )));
        }

        // Forward transit time for diffusion capacitance (default 0 = disabled).
        let tf = Self::lookup_model_param(netlist, model, "TF").unwrap_or(0.0);
        if tf < 0.0 || !tf.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model TF must be non-negative and finite, got {tf}"
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
        if cth < 0.0 || !cth.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model CTH must be non-negative and finite, got {cth}"
            )));
        }
        if cth == 0.0 {
            log::info!(
                "BJT model '{}': CTH=0 — thermal state has no memory; junction temperature tracks dissipation quasi-statically (Tj = TAMB + RTH·P each sample)",
                model
            );
        }

        // SPICE `XTB`: forward/reverse beta temperature exponent, used by the
        // self-heating block as `BF(T) = BF·(Tj/Tnom)^XTB` (likewise `BR`).
        // Default 0.0 is SPICE's own, and makes the power term exactly 1.0 —
        // so a card without `XTB`, and any card at all when `Tj == Tnom`,
        // leaves beta untouched and the emitted DSP byte-identical.
        let xtb = Self::lookup_model_param(netlist, model, "XTB").unwrap_or(0.0);
        if !xtb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model XTB must be finite, got {xtb}"
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

        let tamb =
            Self::lookup_model_param(netlist, model, "TAMB").unwrap_or(melange_primitives::T_NOM);
        if tamb <= 0.0 || !tamb.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "BJT model TAMB must be positive and finite, got {tamb}"
            )));
        }

        // The card is SPICE's, extracted at TNOM; the device sits at TAMB.
        // Scale it there with the SPICE3 BJT law (ngspice `bjttemp.c`): IS
        // through XTI and EG, BF/BR through XTB, and the leakage currents
        // ISE/ISC through both. Self-heating then moves Tj from TAMB with the
        // IS/BF/BR law written relative to TAMB; the law composes. At TAMB =
        // TNOM every factor is exactly 1.
        let t = tamb / melange_primitives::T_NOM;
        let vt = vt * t;
        let factlog = (t - 1.0) * eg / vt + xti * t.ln();
        let bfactor = t.powf(xtb);
        let is = is * factlog.exp();
        let beta_f = beta_f * bfactor;
        let beta_r = beta_r * bfactor;
        let ise = ise * (factlog / ne).exp() / bfactor;
        let isc = isc * (factlog / nc).exp() / bfactor;
        validate_positive_finite(is, "BJT model IS at TAMB")?;

        Self::check_model_params(netlist, model, ModelClass::Bjt)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Bjt,
            "the built-in default BJT",
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
            tf,
            vje,
            mje,
            vjc,
            mjc,
            fc,
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
            xtb,
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
        // Melange's JFET device convention stores VP POSITIVE for P-channel
        // (the device model flips it internally: vp_eff = -vp for P). Every
        // vendor PJF card uses the SPICE convention VTO < 0, so copying VTO
        // verbatim would double-flip the pinch-off and leave the device dead.
        // Normalize SPICE-convention PJF cards (VTO < 0) to the melange
        // convention (vp = -VTO). N-channel VTO < 0 already matches — unchanged.
        let vp = match Self::lookup_model_param(netlist, model, "VTO") {
            Some(raw_vto) if is_p_channel && raw_vto < 0.0 => {
                crate::diag_warn!(
                    "P-channel JFET model '{}': SPICE-convention VTO={} normalized to melange convention vp={} (P-channel pinch-off stored positive; device model flips internally)",
                    model,
                    raw_vto,
                    -raw_vto
                );
                -raw_vto
            }
            Some(raw_vto) if is_p_channel => {
                log::info!(
                    "P-channel JFET model '{}': VTO={} > 0 accepted as already melange-convention (positive P-channel pinch-off)",
                    model,
                    raw_vto
                );
                raw_vto
            }
            Some(raw_vto) => raw_vto,
            None => default_vp,
        };
        // ngspice BETA = IDSS / VP^2, so IDSS = BETA * VP^2.
        // Uses the normalized vp — sign-safe regardless (vp is squared).
        let idss = if let Some(raw_idss) = Self::lookup_model_param(netlist, model, "IDSS") {
            raw_idss
        } else if let Some(beta) = Self::lookup_model_param(netlist, model, "BETA") {
            beta * vp * vp
        } else {
            // SPICE / ngspice default BETA = 1e-4 A/V^2 (IDSS = BETA * VTO^2).
            cat.map(|c| c.idss).unwrap_or(1e-4 * vp * vp)
        };
        // SPICE / ngspice default LAMBDA = 0.
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.0);

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

        // Gate junctions (SPICE IS, N; ngspice defaults). IS = 0 disables them.
        let is = Self::lookup_model_param(netlist, model, "IS").unwrap_or(1e-14);
        if is < 0.0 || !is.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model IS must be non-negative and finite, got {is}"
            )));
        }
        let n = Self::lookup_model_param(netlist, model, "N").unwrap_or(1.0);
        if n <= 0.0 || !n.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "JFET model N must be positive and finite, got {n}"
            )));
        }

        Self::check_model_params(netlist, model, ModelClass::Jfet)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Jfet,
            "the built-in default JFET",
        );

        Ok(JfetParams {
            idss,
            vp,
            lambda,
            is_p_channel,
            cgs,
            cgd,
            is,
            n,
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

        // Card, then catalog part, then the SPICE / ngspice level-1 default
        // (KP 2e-5 A/V^2 with W = L, VTO 0, LAMBDA 0).
        let kp = Self::lookup_model_param(netlist, model, "KP")
            .or_else(|| cat.map(|c| c.kp))
            .unwrap_or(2e-5);
        let default_vt = cat.map(|c| c.vt).unwrap_or(0.0);
        let vt = Self::lookup_model_param(netlist, model, "VTO")
            .or_else(|| Self::lookup_model_param(netlist, model, "VT"))
            .unwrap_or(default_vt);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.0);

        validate_positive_finite(kp, "MOSFET model KP")?;
        if !vt.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "MOSFET model VT must be finite, got {vt}"
            )));
        }
        // MOSFET VTO sign is PRESERVED (unlike the P-JFET normalization above):
        // the device math honors signed VTO, so NMOS with VTO < 0 is a valid
        // depletion-mode device (conducting at Vgs=0), not a convention clash.
        if !is_p_channel && vt < 0.0 {
            log::info!(
                "NMOS model '{}': VTO={} < 0 — depletion-mode device (conducts at Vgs=0); sign preserved",
                model,
                vt
            );
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

        Self::check_model_params(netlist, model, ModelClass::Mosfet)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Mosfet,
            "the built-in default MOSFET",
        );

        // source_node and bulk_node will be resolved later from the MNA system
        Ok(MosfetParams {
            kp,
            vt,
            lambda,
            is_p_channel,
            cgs,
            cgd,
            gamma,
            phi,
            source_node: 0,
            bulk_node: 0,
        })
    }

    /// Report the **grid current starting point** this triode's fitted grid law
    /// implies, and warn when it falls outside the manufacturer's per-type
    /// limit for that tube.
    ///
    /// This is a CHECK, deliberately not a parameter: a parameter would invite
    /// someone to fit it, and the whole value of the onset is that it is an
    /// *output* of `(Gg, xi, Cg)` that an independent datasheet row can score.
    ///
    /// The criterion is the manufacturers' own: `Ig = +0.3 µA`, positive, into
    /// the grid (Philips ECC82 1959 footnote, spelled out in full there; the
    /// ECC83 sheet carries the same row). The limit is read per tube type from
    /// that type's own sheet — ECC83 `max -0.9 V`, ECC82 `max -1.3 V` — never
    /// from one global constant, because the types genuinely differ. A type
    /// with no limit on file is reported and not checked.
    fn report_grid_start_point(model: &str, in_catalog: bool, gg: f64, xi: f64, cg: f64) {
        let tube = melange_devices::KorenTriode {
            mu: 100.0,
            ex: 1.4,
            kg1: 1060.0,
            kp: 600.0,
            kvb: 300.0,
            gg,
            xi,
            cg,
            lambda: 0.0,
            mu_b: 0.0,
            svar: 0.0,
            ex_b: 0.0,
        };
        let Some(onset) =
            tube.grid_voltage_at_current(melange_devices::tube::GRID_START_CRITERION_A)
        else {
            crate::diag_warn!(
                "Triode '{model}': grid law (Gg={gg:.4e}, xi={xi}, Cg={cg}) never reaches the \
                 0.3 uA grid-current starting point — the onset check cannot be evaluated."
            );
            return;
        };
        let ig_at_zero = tube.grid_current(0.0);
        let provenance = if in_catalog {
            ""
        } else {
            " [no catalog entry: grid law is the shipped 12AX7 default unless the deck set \
             GG/XI/CG]"
        };
        log::info!(
            "Triode '{model}': grid current starts (Ig = +0.3 uA) at Vgk = {onset:.3} V; \
             Ig(0 V) = {:.2} uA{provenance}",
            ig_at_zero * 1e6
        );
        match melange_devices::catalog::tubes::grid_start_limit_v(model) {
            Some(limit) if onset < limit => crate::diag_warn!(
                "Triode '{model}': derived grid-current starting point {onset:.3} V is BELOW the \
                 manufacturer limit for this type (Vg(Ig = +0.3 uA) max {limit:.1} V). The fitted \
                 grid law conducts further into the negative-grid region than the type is \
                 specified to."
            ),
            Some(limit) if onset >= 0.0 => crate::diag_warn!(
                "Triode '{model}': derived grid-current starting point {onset:.3} V is at or \
                 above 0 V, so this grid law has no negative-grid conduction at the 0.3 uA \
                 criterion at all. Every measured 12AX7 starts between -0.27 and -0.38 V, and \
                 the type's own limit is {limit:.1} V. Check GG/XI/CG."
            ),
            Some(_) => {}
            None => log::info!(
                "Triode '{model}': no manufacturer grid-current starting-point limit on file for \
                 this type — onset reported, not checked."
            ),
        }
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
        // Dempwolf & Zölzer DAFx-11 eq. (11) grid law. `IG_MAX`/`VGK_ONSET` are
        // RETIRED, not aliased and not repurposed — `check_model_params` below
        // refuses either key and prints the conversion. See `model_params.rs`.
        let gg = Self::lookup_model_param(netlist, model, "GG")
            .or_else(|| cat.map(|c| c.gg))
            .unwrap_or(melange_devices::tube::DEFAULT_GG);
        let xi = Self::lookup_model_param(netlist, model, "XI")
            .or_else(|| cat.map(|c| c.xi))
            .unwrap_or(melange_devices::tube::DEFAULT_XI);
        let cg = Self::lookup_model_param(netlist, model, "CG")
            .or_else(|| cat.map(|c| c.cg))
            .unwrap_or(melange_devices::tube::DEFAULT_CG);
        let lambda = Self::lookup_model_param(netlist, model, "LAMBDA")
            .or_else(|| cat.map(|c| c.lambda))
            .unwrap_or(0.0);

        // The triode is sharp-cutoff: its DC operating point and transient
        // evaluate the single-section Koren law. A card's MU_B/SVAR/EX_B is
        // refused (model_params TRIODE_REFUSED), and the fields stay 0, so every
        // estimate built from these params is of the tube as built.
        let (mu_b, svar, ex_b) = (0.0, 0.0, 0.0);

        validate_positive_finite(mu, "tube model MU")?;
        validate_positive_finite(ex, "tube model EX")?;
        validate_positive_finite(kg1, "tube model KG1")?;
        validate_positive_finite(kp, "tube model KP")?;
        validate_positive_finite(kvb, "tube model KVB")?;
        validate_positive_finite(gg, "tube model GG")?;
        validate_positive_finite(xi, "tube model XI")?;
        validate_positive_finite(cg, "tube model CG")?;

        // Validate optional lambda: must be non-negative and finite
        if !lambda.is_finite() || lambda < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model LAMBDA must be non-negative and finite, got {lambda}"
            )));
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

        // Self-heating thermal params (optional; disabled by default so the
        // generated solver is byte-identical for every non-thermal triode).
        // Mirrors the BJT/diode contract: `RTH` is the gate — any finite,
        // positive value activates the per-sample envelope-temperature update
        // and the `VBIAS_ALPHA · (Tp - TAMB)` Vgk drift baked into the Koren
        // call site. `CTH` sets the thermal time constant τ = RTH·CTH. Pentode
        // support is gated at the model layer (`has_self_heating`) — the
        // resolver still accepts the params so pentode circuits don't trip
        // the unrecognized-param warning, but the emitter won't use them
        // until pentode screen-dissipation is wired.
        let rth = Self::lookup_model_param(netlist, model, "RTH").unwrap_or(f64::INFINITY);
        if rth.is_finite() && rth <= 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model RTH must be positive (or infinite to disable), got {rth}"
            )));
        }
        let cth = Self::lookup_model_param(netlist, model, "CTH").unwrap_or(0.0);
        if !cth.is_finite() || cth < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model CTH must be non-negative and finite, got {cth}"
            )));
        }
        if rth.is_finite() && cth == 0.0 {
            log::info!(
                "Tube model '{}': CTH=0 with finite RTH — thermal time constant τ = RTH·CTH is zero; envelope temperature tracks dissipation quasi-statically (Tp = TAMB + RTH·P each sample)",
                model
            );
        }
        let vbias_alpha = Self::lookup_model_param(netlist, model, "VBIAS_ALPHA").unwrap_or(0.0);
        if !vbias_alpha.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model VBIAS_ALPHA must be finite, got {vbias_alpha}"
            )));
        }
        let tamb = Self::lookup_model_param(netlist, model, "TAMB").unwrap_or(300.15);
        if !tamb.is_finite() || tamb <= 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "tube model TAMB must be positive and finite, got {tamb}"
            )));
        }

        Self::check_model_params(netlist, model, ModelClass::Triode)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Triode,
            "a default 12AX7-class triode",
        );

        Self::report_grid_start_point(model, cat.is_some(), gg, xi, cg);

        Ok(TubeParams {
            kind: crate::device_types::TubeKind::SharpTriode,
            mu,
            ex,
            kg1,
            kp,
            kvb,
            // Leach fields, unused by the triode path (see `TubeParams::ig_max`).
            ig_max: 0.0,
            vgk_onset: 0.0,
            gg,
            xi,
            cg,
            lambda,
            ccg,
            cgp,
            ccp,
            rgi,
            kg2: 0.0,
            alpha_s: 0.0,
            a_factor: 0.0,
            beta_factor: 0.0,
            // Phase 5: partition noise is pentode-only. Triode plate-shot is
            // bare Schottky `2q·Ip`; PARTITION_F is unused and the field is
            // gated by `is_pentode()` at the codegen-collector layer.
            partition_f: 1.0,
            screen_form: crate::device_types::ScreenForm::Rational,
            mu_b,
            svar,
            ex_b,
            rth,
            cth,
            vbias_alpha,
            tamb,
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
        // Phase 5 partition-noise multiplier. Default 1.0 (textbook Schottky
        // partition statistics). Not in the catalog — it's a process-variation
        // knob applied at codegen, not a fitted device parameter.
        let partition_f = Self::lookup_model_param(netlist, model, "PARTITION_F").unwrap_or(1.0);
        let ig_max = Self::lookup_model_param(netlist, model, "IG_MAX")
            .or_else(|| cat.map(|c| c.ig_max))
            .unwrap_or(8e-3);
        let vgk_onset = Self::lookup_model_param(netlist, model, "VGK_ONSET")
            .or_else(|| cat.map(|c| c.vgk_onset))
            .unwrap_or(0.7);
        // The pentode plate law has no lambda term: a card's LAMBDA is refused
        // (model_params PENTODE_REFUSED), and the field stays 0.
        let lambda = 0.0;

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
                    Some(melange_devices::tube::ScreenForm::Exponential) => ScreenForm::Exponential,
                    Some(melange_devices::tube::ScreenForm::Classical) => ScreenForm::Classical,
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

        Self::check_model_params(netlist, model, ModelClass::Pentode)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Pentode,
            "a default EL84-class pentode",
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
            // D&Z triode grid fields, unused on the pentode path: a pentode's
            // control grid keeps the Leach law above (no published D&Z-form fit
            // exists for a power pentode, and melange does not invent one).
            gg: melange_devices::tube::DEFAULT_GG,
            xi: melange_devices::tube::DEFAULT_XI,
            cg: melange_devices::tube::DEFAULT_CG,
            lambda,
            ccg,
            cgp,
            ccp,
            rgi,
            kg2,
            alpha_s,
            a_factor,
            beta_factor,
            partition_f,
            screen_form,
            // Phase 1c variable-mu §5 params, resolved above from the `.model`
            // directive (explicit > catalog > default 0.0) and already
            // validated. Previously these were hardcoded to 0.0, which silently
            // discarded a variable-mu pentode card AFTER it passed validation —
            // the deck compiled as a sharp pentode with no diagnostic. For a
            // sharp card svar/mu_b/ex_b resolve to 0.0, so this is byte-identical
            // for every non-variable-mu pentode; it only changes svar>0 decks
            // (e.g. the 6K7 remote-cutoff stage).
            mu_b,
            svar,
            ex_b,
            // Pentode self-heating not wired yet — screen dissipation needs a
            // separate term (Ip·Vpk + Ig2·Vg2k). Triode path is live.
            rth: f64::INFINITY,
            cth: 0.0,
            vbias_alpha: 0.0,
            tamb: 300.15,
        };
        params.validate().map_err(CodegenError::InvalidConfig)?;
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

        Self::check_model_params(netlist, model, ModelClass::Vca)?;

        Ok(VcaParams { vscale, g0, thd })
    }

    /// Resolve opto/LDR model params. Resolution order per param: explicit
    /// `.model … LDR(RMIN=… …)` value → catalog entry (by model name, e.g.
    /// `.model VTL5C3 LDR()`) → generic default. Mirrors `ldr.rs` semantics.
    fn resolve_ldr_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<crate::device_types::LdrParams, CodegenError> {
        let cat = melange_devices::catalog::ldr::lookup(model);
        let r_min = Self::lookup_model_param(netlist, model, "RMIN")
            .or_else(|| cat.map(|c| c.r_min))
            .unwrap_or(75.0);
        let r_max = Self::lookup_model_param(netlist, model, "RMAX")
            .or_else(|| cat.map(|c| c.r_max))
            .unwrap_or(10e6);
        let gamma = Self::lookup_model_param(netlist, model, "GAMMA")
            .or_else(|| cat.map(|c| c.gamma))
            .unwrap_or(0.7);
        let attack_tau = Self::lookup_model_param(netlist, model, "TAU_A")
            .or_else(|| cat.map(|c| c.attack_tau))
            .unwrap_or(0.005);
        let release_tau = Self::lookup_model_param(netlist, model, "TAU_R")
            .or_else(|| cat.map(|c| c.release_tau))
            .unwrap_or(0.2);

        validate_positive_finite(r_min, "LDR model RMIN")?;
        validate_positive_finite(r_max, "LDR model RMAX")?;
        validate_positive_finite(gamma, "LDR model GAMMA")?;
        validate_positive_finite(attack_tau, "LDR model TAU_A")?;
        validate_positive_finite(release_tau, "LDR model TAU_R")?;
        if r_max <= r_min {
            return Err(CodegenError::InvalidConfig(format!(
                "LDR model '{model}': RMAX ({r_max}) must be greater than RMIN ({r_min})"
            )));
        }

        Self::check_model_params(netlist, model, ModelClass::Ldr)?;
        Self::warn_unresolved_model(
            netlist,
            model,
            cat.is_some(),
            ModelClass::Ldr,
            "the built-in default LDR",
        );

        Ok(crate::device_types::LdrParams {
            r_min,
            r_max,
            gamma,
            attack_tau,
            release_tau,
        })
    }

    /// Resolve glow-discharge / neon lamp model params (EXPERIMENTAL Phase 0c
    /// Stage 2a). Resolution per param: explicit `.model … NEON(VO=… …)` value
    /// → generic default. No catalog (throwaway experimental device).
    fn resolve_glow_params(
        netlist: &Netlist,
        model: &str,
    ) -> Result<crate::device_types::GlowParams, CodegenError> {
        // Option-A maintaining-line parameterisation (datasheet-sourced). The
        // lit branch is the affine maintaining line `V(a)−V(k) = v0 + rs·i`.
        // Its intercept `v0` is NOT authored directly — it is derived from the
        // datasheet static maintaining voltage `VM` (measured at the rated
        // current `IK`) and the slope `RS`, so the deck carries datasheet
        // numbers and melange computes the intercept:  v0 = VM − RS·IK.
        // ZA1001 anchors: VM = 93 V @ IK = 1.5 mA; RS ≈ 2.5–4.25 kΩ (ZA1004
        // form-transfer, mid 3 kΩ). The OLD model used VM directly as the
        // intercept (fixed VD = 93), parking the reset floor ~4–6 V too high.
        let vo = Self::lookup_model_param(netlist, model, "VO").unwrap_or(135.0);
        let vm = Self::lookup_model_param(netlist, model, "VM").unwrap_or(93.0);
        let ik = Self::lookup_model_param(netlist, model, "IK").unwrap_or(1.5e-3);
        let rs = Self::lookup_model_param(netlist, model, "RS").unwrap_or(3.0e3);
        let roff = Self::lookup_model_param(netlist, model, "ROFF").unwrap_or(300e6);
        // Holding current: the lit→dark extinction threshold on conduction
        // current. Default 2e-4 A (datasheet ZA1004 minimum-sustaining regime).
        // The reset floor lands at v0 + rs·ihold. Must exceed the lit
        // equilibrium sustaining current (Vb−v0)/(Rc+rs) for a relaxation
        // oscillator to extinguish; a physical small-neon value does.
        let ihold = Self::lookup_model_param(netlist, model, "IHOLD").unwrap_or(2e-4);

        // Relaxing-section lit branch (Benson & Bradshaw 1965; defaults-off).
        // RT = DC asymptote of the maintaining line (defaults to RS, so a deck
        // that authors no sections reduces exactly to the static model). K1..K4
        // = delayed-overvoltage coefficients [volts] (default 0 = section off),
        // TAU1..TAU4 = current-lag time constants [seconds]. A section is
        // "active" when its K is non-zero; a zero K disables the section with no
        // special-casing (both the log term and its Jacobian contribution
        // vanish). Never authored directly: the intercept v0 = VM − RS·IK below
        // is unchanged (R3 reconciliation of RS vs RT is a later authoring
        // concern, not this mechanism).
        let r_t = Self::lookup_model_param(netlist, model, "RT").unwrap_or(rs);
        let mut k = [0.0f64; 4];
        let mut tau = [0.0f64; 4];
        for i in 0..4 {
            k[i] = Self::lookup_model_param(netlist, model, &format!("K{}", i + 1)).unwrap_or(0.0);
            tau[i] =
                Self::lookup_model_param(netlist, model, &format!("TAU{}", i + 1)).unwrap_or(0.0);
        }
        let has_sections = k.iter().any(|&x| x != 0.0);

        // IFLOOR (A1): the log-domain current clamp / section-lag seed floor.
        // Authored key; defaults to IHOLD for continuity but is a live edge knob.
        let ifloor = Self::lookup_model_param(netlist, model, "IFLOOR").unwrap_or(ihold);

        // KSUB (part-a): static subnormal-branch slope κ [V per e-fold]; default
        // 0 = off (lit branch bit-identical to the no-KSUB form). Anchored at the
        // rated current IK (where g = VM). Only meaningful WITH sections.
        let ksub = Self::lookup_model_param(netlist, model, "KSUB").unwrap_or(0.0);

        // Ignition depression D(t_off) (Part B; default-off). D_AMP=0 → OFF and
        // no state slot / plain VO strike test (byte-identical). Curve:
        // D = clamp(D_AMP·ln(D_TKNEE/max(t_off, D_THOLD)), 0, VO−VM).
        let d_amp = Self::lookup_model_param(netlist, model, "D_AMP").unwrap_or(0.0);
        let d_tknee = Self::lookup_model_param(netlist, model, "D_TKNEE").unwrap_or(0.0);
        let d_thold = Self::lookup_model_param(netlist, model, "D_THOLD").unwrap_or(0.0);

        validate_positive_finite(vo, "NEON model VO")?;
        validate_positive_finite(vm, "NEON model VM")?;
        validate_positive_finite(ik, "NEON model IK")?;
        validate_positive_finite(rs, "NEON model RS")?;
        validate_positive_finite(roff, "NEON model ROFF")?;
        validate_positive_finite(ihold, "NEON model IHOLD")?;
        // r_t is a DC resistance that may be ~0 (normal glow) but never negative
        // or non-finite. Sections need a positive time constant only where the
        // coefficient is non-zero (an active section); a zero-K section is off
        // and its TAU is ignored.
        if !r_t.is_finite() || r_t < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': RT ({r_t}) must be finite and non-negative"
            )));
        }
        for i in 0..4 {
            if !k[i].is_finite() || !tau[i].is_finite() {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': K{}/TAU{} must be finite",
                    i + 1,
                    i + 1
                )));
            }
            if k[i] != 0.0 && tau[i] <= 0.0 {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': TAU{} ({}) must be > 0 when K{} ({}) is non-zero \
                     (active relaxing section needs a positive current-lag time constant)",
                    i + 1,
                    tau[i],
                    i + 1,
                    k[i]
                )));
            }
        }
        validate_positive_finite(ifloor, "NEON model IFLOOR")?;
        // KSUB (subnormal slope) validation. Non-negative; requires sections (the
        // static log term is folded into the section-branch g(I) — a KSUB-only
        // deck would emit the static linear path and silently drop it). The
        // R_T>0 && κ>ΣK corner makes the inner-Newton residual r'(x)=R_T·eˣ+(S−κ)
        // change sign (two roots / none) — reject it; R_T=0 (analog-EE authoring guidance)
        // or κ≤ΣK stays single-signed and globally convergent.
        if !ksub.is_finite() || ksub < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': KSUB ({ksub}) must be finite and non-negative"
            )));
        }
        if ksub != 0.0 {
            if !has_sections {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': KSUB ({ksub}) requires ≥1 active section (K1..K4); the \
                     subnormal term is folded into the relaxing-section lit branch, not the static path"
                )));
            }
            let k_sum: f64 = k.iter().sum();
            if r_t > 0.0 && ksub > k_sum {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': KSUB ({ksub}) > ΣK ({k_sum}) with RT ({r_t}) > 0 is a \
                     non-monotone lit branch (r'(x)=RT·eˣ+(ΣK−KSUB) changes sign → non-convergent); \
                     author RT=0 for a subnormal branch, or keep KSUB ≤ ΣK"
                )));
            }
        }
        // Ignition-depression validation (only meaningful when D_AMP ≠ 0).
        if !d_amp.is_finite() || d_amp < 0.0 {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': D_AMP ({d_amp}) must be finite and non-negative"
            )));
        }
        if d_amp != 0.0 {
            if !(d_tknee > 0.0 && d_tknee.is_finite()) {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': D_TKNEE ({d_tknee}) must be > 0 when D_AMP is non-zero"
                )));
            }
            if !(d_thold > 0.0 && d_thold.is_finite()) {
                return Err(CodegenError::InvalidConfig(format!(
                    "NEON model '{model}': D_THOLD ({d_thold}) must be > 0 when D_AMP is non-zero"
                )));
            }
        }

        // Derived maintaining-line intercept (A2). With relaxing sections the
        // lower reset floor comes from the section TAIL (Ī lags falling I →
        // negative overvoltage → cv_extinction < VM), so RS is retired from the
        // intercept and v0 = VM − RT·IK (→ VM at RT≈0). Without sections the
        // historical v0 = VM − RS·IK is kept EXACTLY (byte-identity).
        let v0 = if has_sections {
            vm - r_t * ik
        } else {
            vm - rs * ik
        };
        if v0 <= 0.0 {
            let (slope_name, slope_val) = if has_sections {
                ("RT", r_t)
            } else {
                ("RS", rs)
            };
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': derived maintaining-line intercept v0 = VM − {slope_name}·IK \
                 = {vm} − {slope_val}·{ik} = {v0} is non-positive; check VM/{slope_name}/IK"
            )));
        }
        if vo <= vm {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': VO ({vo}) must be greater than VM ({vm}) \
                 (ignition voltage above the maintaining voltage)"
            )));
        }
        if roff <= rs {
            return Err(CodegenError::InvalidConfig(format!(
                "NEON model '{model}': ROFF ({roff}) must be greater than RS ({rs})"
            )));
        }

        // main replaced warn_unrecognized_params with the stricter
        // check_model_params (unknown keys are a hard error; glow/NEON is a
        // melange-native device, so no recognized-but-unimplemented SPICE keys).
        Self::check_model_params(netlist, model, ModelClass::Glow)?;

        Ok(crate::device_types::GlowParams {
            vo,
            v0,
            rs,
            roff,
            ihold,
            r_t,
            k,
            tau,
            ifloor,
            ksub,
            // Subnormal anchor = rated current IK (g = VM there). Only used when ksub≠0.
            i_n: ik,
            d_amp,
            d_tknee,
            d_thold,
            // Hard cap on the ignition depression: V_s,eff never below VM.
            d_cap: vo - vm,
        })
    }

    /// Warn on unrecognized .model parameters (typo protection).
    /// Check every `.model` parameter against what melange does with it.
    ///
    /// Three outcomes, because a `.model` key can be wrong in two very different
    /// ways and collapsing them serves neither:
    ///
    /// * **Honored** — melange reads it. Silent.
    /// * **Recognized but unimplemented** — a real SPICE parameter melange does
    ///   not model yet (`unimplemented`). Warns, naming what the omission costs.
    ///   NOT an error: these arrive on authentic vendor model cards, and
    ///   refusing them would mean melange rejects genuine SPICE decks over a gap
    ///   of its own. The warning is the honest report of that gap.
    /// * **Unknown** — not a valid parameter for this device type at all. **Hard
    ///   error**, with the accepted keys and an alias hint where one is known.
    ///
    /// The third case used to warn and continue, which is how `VP=` on a JFET
    /// card (the datasheet spelling; SPICE uses `VTO`, and the sign convention
    /// differs) could be silently discarded while the deck still biased
    /// correctly off the built-in catalog — producing a right answer for the
    /// wrong reason, which is worse than a wrong answer.
    fn check_model_params(
        netlist: &Netlist,
        model_name: &str,
        class: ModelClass,
    ) -> Result<(), CodegenError> {
        let honored = class.honored();
        let Some(m) = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
        else {
            return Ok(());
        };
        for (key, value) in &m.params {
            let upper = key.to_ascii_uppercase();
            if honored.iter().any(|k| k.eq_ignore_ascii_case(&upper)) {
                continue;
            }
            if let Some(note) = class.refused_note(&upper) {
                if *value == 0.0 {
                    continue;
                }
                return Err(CodegenError::InvalidConfig(format!(
                    ".model {model_name}: {upper}={value} is refused: {note}."
                )));
            }
            if crate::model_params::notice_if_unimplemented(model_name, class, &upper) {
                continue;
            }
            // A RETIRED key is refused with its conversion, not reported as a
            // typo: the deck is not misspelled, it is written against a device
            // law melange no longer has.
            if let Some(note) = class.retired_note(&upper) {
                return Err(CodegenError::InvalidConfig(format!(
                    ".model {model_name}: parameter '{key}' is RETIRED — {note}. \
                     Accepted for this device: {}",
                    honored.join(", ")
                )));
            }
            let hint = crate::model_params::alias_hint(class, &upper);
            return Err(CodegenError::InvalidConfig(format!(
                ".model {model_name}: unknown parameter '{key}'.{hint} Accepted \
                 for this device: {}",
                honored.join(", ")
            )));
        }
        Ok(())
    }

    /// Warn when a `.model` card resolves entirely to the hardcoded default
    /// device because its name matches no built-in catalog part **and** it
    /// supplies none of its device-defining parameters.
    ///
    /// This is the "plausible numbers, wrong circuit" trap: a typo'd model name
    /// (`.model 12AX8 TRIODE()`) compiles silently as the default device (a
    /// 12AX7 triode, EL84 pentode, SPICE-default diode, …) with no diagnostic.
    ///
    /// It only fires for a *declared-but-underspecified* card. A device that
    /// references a **never-declared** model already hard-errors in the parser
    /// (`references model '…' which is not defined`), so that case never reaches
    /// here. Stays silent on a catalog hit and on any card that specifies a
    /// defining parameter — a fully custom off-catalog part is legitimate and
    /// common, so specifying even one defining key suppresses the warning.
    ///
    /// The class's electrical-identity parameter set (`ModelClass::defining()`
    /// — the recognized keys minus universal add-ons like KF/AF/RTH/CTH/TAMB,
    /// which do not define which device this is) decides "underspecified". A
    /// class with no such set never warns.
    fn warn_unresolved_model(
        netlist: &Netlist,
        model_name: &str,
        catalog_hit: bool,
        class: ModelClass,
        default_desc: &str,
    ) {
        let defining_keys = class.defining();
        if defining_keys.is_empty() || catalog_hit {
            return;
        }
        let Some(card) = netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
        else {
            return;
        };
        let supplied_defining = card.params.iter().any(|(key, _)| {
            let upper = key.to_ascii_uppercase();
            defining_keys.iter().any(|k| k.eq_ignore_ascii_case(&upper))
        });
        if !supplied_defining {
            crate::diag_warn!(
                ".model {}: name matches no built-in catalog part and no \
                 device-defining parameter was supplied — compiling as {}. A \
                 typo'd model name silently becomes the default device; use an \
                 exact catalog name or specify the device parameters.",
                model_name,
                default_desc,
            );
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
            r_sag: crate::mna::OPAMP_DEFAULT_R_SAG_OHM,
            vcc,
            vee,
            gbw: f64::INFINITY,
            sr: f64::INFINITY,
            ib: 0.0,
            rin: f64::INFINITY,
            aol_transient_cap: f64::INFINITY,
            n_internal_idx: 0,
            iir_c_dom: 0.0,
            n_int_idx: 0,
            en: 0.0,
            in_amps: 0.0,
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
            r_sag: crate::mna::OPAMP_DEFAULT_R_SAG_OHM,
            vcc,
            vee,
            gbw: f64::INFINITY,
            sr: f64::INFINITY,
            ib: 0.0,
            rin: f64::INFINITY,
            aol_transient_cap: f64::INFINITY,
            n_internal_idx: 0,
            iir_c_dom: 0.0,
            n_int_idx: 0,
            en: 0.0,
            in_amps: 0.0,
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
        let mna = mna_with_opamps_and_caps(3, vec![opamp_at_nodes(2, 1, 1, 9.0, 0.0)], &[]);
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
        assert!(
            (is_val - 1e-15).abs() < 1e-20,
            "Is should be 1e-15, got {is_val}"
        );
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
        assert_eq!(
            vs_added, 4,
            "Expected 4 rail-reference voltage sources (2 op-amps × 2 rails)"
        );
        assert_eq!(
            diodes_added, 4,
            "Expected 4 catch diodes (2 op-amps × 2 rails)"
        );
        assert_eq!(
            buffer_vcvs_added, 2,
            "Expected 2 output-buffer VCVS (1 per clamped op-amp)"
        );

        let r_ro_added = aug_netlist
            .elements
            .iter()
            .filter(|e| {
                matches!(e, crate::parser::Element::Resistor { name, .. } if name.starts_with("R_oa_ro_"))
            })
            .count();
        assert_eq!(
            r_ro_added, 2,
            "Expected 2 output-buffer series resistors (1 per clamped op-amp)"
        );

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
                assert_ne!(
                    oa.n_int_idx, 0,
                    "op-amp {} should be in BoyleDiodes mode",
                    oa.name
                );
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

    /// Audio-path (a feedback clipper, output cap-coupled) and control-path (the
    /// output drives a rectifier through a resistor) topologies used to resolve
    /// to different modes, because a backward-Euler step on every rail-engaged
    /// sample was thought to suit one and not the other. Both now resolve to
    /// `ActiveSet`: under the charge form a pin or release leaves no carried
    /// residual on capless rows in either topology, and `ActiveSetBe` is an
    /// explicit mode only.
    #[test]
    fn audio_and_control_path_topologies_both_resolve_to_active_set() {
        let feedback_clipper = "\
Feedback Clipper Test (overdrive pedal pattern)
R1 in sum 10k
R2 sum clip_out 100k
C1 clip_out ac_out 1u
R3 ac_out 0 100k
D1 clip_out sum DCLIP
D2 sum clip_out DCLIP
U1 0 sum clip_out OA1
.model DCLIP D(IS=1e-14 N=1.9)
.model OA1 OA(AOL=200k ROUT=75 VCC=4.5 VEE=-4.5)
";
        let sidechain = "\
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
.model OA1 OA(AOL=200k ROUT=75 VCC=9 VEE=-9)
";
        for spice in [feedback_clipper, sidechain] {
            let r = parse_and_resolve(spice);
            assert_eq!(r.mode, OpampRailMode::ActiveSet);
            assert_eq!(r.reason, OpampRailModeReason::AcCoupledDownstream);
        }
    }
}
