//! Code generation for specialized circuit solvers.
//!
//! This module generates zero-overhead Rust code from a compiled DkKernel.
//! The generated code is specialized for a specific circuit topology with
//! compile-time constant matrices and unrolled loops.
//!
//! ## Architecture
//!
//! ```text
//! Netlist → MNA → DkKernel → CircuitIR → Emitter → Source Code
//! ```
//!
//! - [`ir::CircuitIR`] — serializable, language-agnostic intermediate representation
//! - [`emitter::Emitter`] — trait that language backends implement
//! - [`rust_emitter::RustEmitter`] — Rust language backend

#[cfg(feature = "codegen")]
pub mod emitter;
#[cfg(feature = "codegen")]
pub mod fast_math;
#[cfg(feature = "codegen")]
pub mod ir;
#[cfg(feature = "codegen")]
pub mod policy;
#[cfg(feature = "codegen")]
pub mod ring;
pub mod routing;
#[cfg(feature = "codegen")]
pub mod rust_emitter;
#[cfg(feature = "codegen")]
pub mod stability;

#[cfg(feature = "codegen")]
use crate::dk::DkKernel;
#[cfg(feature = "codegen")]
use crate::mna::MnaSystem;
#[cfg(feature = "codegen")]
use crate::parser::Netlist;

#[cfg(feature = "codegen")]
use emitter::{EmitOutput, Emitter};
#[cfg(feature = "codegen")]
#[cfg(feature = "codegen")]
use ir::CircuitIR;
use rust_emitter::RustEmitter;

/// The single place a language backend is chosen.
///
/// `generate` goes through `dyn Emitter` rather than naming a concrete emitter,
/// so adding a target is a change here and nowhere else. Previously both
/// generate paths called `RustEmitter` directly, which meant nothing in melange
/// could be pointed at another backend no matter how language-agnostic the IR
/// was — the trait existed but carried no traffic.
#[cfg(feature = "codegen")]
fn select_emitter() -> Result<Box<dyn Emitter>, CodegenError> {
    Ok(Box::new(RustEmitter::new()?))
}

/// Strategy for modeling op-amp output saturation at the supply rails.
///
/// Real op-amps can't drive their output past their supply rails; in melange,
/// the linear VCCS model (`Gm = AOL/ROUT`) has unbounded output and requires
/// an explicit rail-saturation mechanism. Different circuits need different
/// mechanisms because they have different trade-offs between numerical fidelity,
/// harmonic fidelity, and runtime cost.
///
/// # Auto-selection (default)
///
/// When set to [`Auto`](OpampRailMode::Auto), codegen inspects the circuit and
/// picks the cheapest correct mode. The decision is logged at compile time so
/// users can see what was chosen and why. Override with one of the explicit
/// variants when bisecting issues or measuring.
///
/// # Variant semantics
///
/// - [`None`](OpampRailMode::None): no clamping at all. Op-amp output is
///   unbounded; circuit must never drive the op-amp into saturation. Suitable
///   only for verified-linear circuits (clean mixers, buffers, flat EQs).
///
/// - [`Hard`](OpampRailMode::Hard): post-NR `v[out].clamp(VEE, VCC)`. Cheapest,
///   matches pre-2026-04 behavior. **Breaks KCL** for any cap connecting the
///   clamped node to another node — downstream AC-coupled integrators will
///   drift to physically impossible values. Only safe when every op-amp
///   output is DC-coupled to its downstream load (no series coupling cap).
///
/// - [`ActiveSet`](OpampRailMode::ActiveSet): post-NR constrained re-solve.
///   After NR converges, any clamped node is pinned via row replacement and
///   the rest of the network is re-solved to match. KCL-consistent. Fixes
///   the cap-history corruption of an AC-coupled op-amp clipper. Cost: one extra LU back-solve
///   (O(N²)) on samples where clamping is active. Still produces hard-clip
///   harmonics — fine for utility clamping, not ideal for distortion pedals.
///
/// - [`BoyleDiodes`](OpampRailMode::BoyleDiodes): auto-inserted catch diodes
///   per op-amp, anchored to rail-offset voltage sources at `VCC − VOH_DROP`
///   and `VEE + VOL_DROP`. Matches the Boyle macromodel used by every
///   commercial SPICE and produces the soft exponential knee characteristic
///   of real op-amp output stages. Most accurate for distortion circuits
///   (op-amp diode-clipper overdrives and the like). Cost: +2 N and +2 M per op-amp, plus the
///   synthesized voltage sources' augmented rows.
/// A *request* for which nodal sub-path to emit — the user's override knob.
///
/// Distinct from [`NodalSubPath`], which reports what the emitter actually
/// generated. The report deliberately has no `Auto` variant: a finished build
/// is Schur or full-LU, never "auto". This type is the input, that one is the
/// outcome, and collapsing them would let an outcome claim to be undecided.
///
/// The nodal solver has two Newton implementations of the same circuit: the
///
/// **Schur** path predicts through `S = A⁻¹` and iterates only on the M coupled
/// device dimensions; the **full-LU** path factors the whole augmented N×N
/// system every iteration. Full-LU is slower and more robust; Schur is cheaper
/// and needs the reduction to be well conditioned.
///
/// [`Auto`](NodalSubPathOverride::Auto) is the shipping behaviour and is what every
/// production build should use: the emitter picks from measured conditioning
/// (`K` degeneracy, `max|S|`, `max|K|`, `ρ(S·A_neg)`) plus structural facts.
///
/// The two forcing modes are **diagnostic escape hatches**, in the same spirit
/// as `--force-trap`: they exist so the sub-path can be isolated as a variable
/// — A/B-ing the two implementations on one netlist, or reproducing a build
/// from before a routing decision moved. Neither is a production setting.
///
/// **Forcing is refused, not warned about, when the choice is structural rather
/// than heuristic.** Saturating inductors are stamped as nonlinear
/// devices on their augmented branch row inside the full-LU Newton loop, and
/// behavioral B-sources are stamped in node space only on the full-LU path — the
/// Schur reduction cannot express either. Asking for `Schur` on such a circuit
/// would emit code that silently drops the nonlinearity, so it is an error
/// instead. Where the auto choice was merely a conditioning *heuristic*,
/// forcing is allowed and warns.
#[cfg(feature = "codegen")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum NodalSubPathOverride {
    /// Pick from measured conditioning and structure. The shipping default.
    #[default]
    Auto,
    /// Force the Schur reduction. Refused when the circuit structurally
    /// requires full-LU. Diagnostic only.
    Schur,
    /// Force the full N×N LU Newton loop. Always structurally valid — full-LU
    /// is the general path — but slower. Diagnostic only.
    FullLu,
}

#[cfg(feature = "codegen")]
impl NodalSubPathOverride {
    /// Parse a mode name (case-insensitive) from a CLI flag or config string.
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "auto" => Some(Self::Auto),
            "schur" => Some(Self::Schur),
            "full-lu" | "full_lu" | "fulllu" | "lu" => Some(Self::FullLu),
            _ => None,
        }
    }

    /// Human-readable name for logging and the provenance header.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Schur => "schur",
            Self::FullLu => "full-lu",
        }
    }
}

#[cfg(feature = "codegen")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum OpampRailMode {
    /// Auto-select based on circuit topology. See type-level docs.
    Auto,
    /// No clamping — op-amp output is unbounded (only safe for linear circuits).
    None,
    /// Post-NR hard clamp (pre-2026-04 behavior). Breaks KCL for AC-coupled downstream.
    Hard,
    /// Post-NR constrained re-solve on trapezoidal `state.a`. KCL-consistent hard
    /// clip with square-wave harmonics. The auto-resolver's choice for any
    /// cap-coupled railing op-amp.
    ActiveSet,
    /// On rail engagement, fall through to the BE NR fallback and run the
    /// constrained re-solve against `state.a_be` (backward Euler).
    /// Trapezoidal with post-NR pin develops a Nyquist-rate limit cycle when
    /// the clamp is engaged across multiple samples (the cap-history term
    /// `(2/T)·C·v_prev` alternates sign every sample); BE damps this.
    ///
    /// Explicit mode only. It runs backward Euler on every rail-engaged sample,
    /// i.e. on whole rail plateaus (73-96 % of samples on a single-supply
    /// overdrive), which is first-order there: 2-4x the output-peak error of
    /// `ActiveSet`, at about the same CPU. See
    /// `docs/aidocs/OPAMP_RAIL_MODES.md`.
    ActiveSetBe,
    /// Auto-inserted Boyle catch diodes. Soft exponential knee, correct physics.
    BoyleDiodes,
}

#[cfg(feature = "codegen")]
impl OpampRailMode {
    /// Parse a mode name (case-insensitive) from a CLI flag or config string.
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "auto" => Some(Self::Auto),
            "none" | "off" => Some(Self::None),
            "hard" | "clamp" => Some(Self::Hard),
            "active-set" | "active_set" | "activeset" => Some(Self::ActiveSet),
            "active-set-be" | "active_set_be" | "activesetbe" | "be-on-clamp" => {
                Some(Self::ActiveSetBe)
            }
            "boyle-diodes" | "boyle_diodes" | "boylediodes" | "boyle" | "diodes" => {
                Some(Self::BoyleDiodes)
            }
            _ => None,
        }
    }

    /// Human-readable name for logging.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::None => "none",
            Self::Hard => "hard",
            Self::ActiveSet => "active-set",
            Self::ActiveSetBe => "active-set-be",
            Self::BoyleDiodes => "boyle-diodes",
        }
    }
}

#[cfg(feature = "codegen")]
impl std::fmt::Display for OpampRailMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Which nodal sub-path the emitter actually generated.
///
/// The DK-vs-nodal route is decided in [`routing::auto_route`], but the choice
/// *within* the nodal path is made by the emitter from the finished IR. It was
/// previously not reported anywhere, which made it unobservable: a netlist
/// author could not confirm a circuit reached full-LU, and neither could a
/// golden baseline. Returned from the emitter (rather than recomputed in the
/// pipeline) so there is exactly one source of truth — the emitter reports what
/// it did rather than the pipeline predicting what it should have done.
#[cfg(feature = "codegen")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum NodalSubPath {
    /// Schur-complement reduction: the M-dimensional nonlinear solve.
    Schur,
    /// Dense N x N LU over the full augmented system.
    FullLu,
}

#[cfg(feature = "codegen")]
impl NodalSubPath {
    pub fn as_str(&self) -> &'static str {
        match self {
            NodalSubPath::Schur => "schur",
            NodalSubPath::FullLu => "full-lu",
        }
    }
}

#[cfg(feature = "codegen")]
impl std::fmt::Display for NodalSubPath {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Circuit-noise injection mode.
///
/// Noise is stamped as Norton current sources into the MNA RHS, so the solver's
/// Jacobian shapes it through the full circuit transfer function automatically.
/// Runtime `set_noise_enabled(false)` branches around all RNG calls — zero CPU
/// cost when disabled. See `docs/aidocs/NOISE.md`.
#[cfg(feature = "codegen")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum NoiseMode {
    /// No noise code emitted. Byte-identical to pre-noise codegen.
    #[default]
    Off,
    /// Johnson-Nyquist thermal noise on every fixed resistor. Phase 1.
    Thermal,
    /// Thermal + shot noise on diode/BJT/JFET/MOSFET/tube junctions. Phase 2.
    Shot,
    /// Thermal + shot + 1/f (flicker) + op-amp en/in + pentode partition. Phases 3-5.
    Full,
}

#[cfg(feature = "codegen")]
impl NoiseMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "off" | "none" | "0" => Some(Self::Off),
            "thermal" | "johnson" | "johnson-nyquist" => Some(Self::Thermal),
            "shot" => Some(Self::Shot),
            "full" | "all" => Some(Self::Full),
            _ => None,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Off => "off",
            Self::Thermal => "thermal",
            Self::Shot => "shot",
            Self::Full => "full",
        }
    }

    /// True when any noise code should be emitted.
    pub fn is_enabled(&self) -> bool {
        !matches!(self, Self::Off)
    }

    /// True when thermal-noise code should be emitted.
    pub fn includes_thermal(&self) -> bool {
        matches!(self, Self::Thermal | Self::Shot | Self::Full)
    }

    /// True when shot-noise code should be emitted.
    pub fn includes_shot(&self) -> bool {
        matches!(self, Self::Shot | Self::Full)
    }

    /// True when flicker / op-amp / partition code should be emitted.
    pub fn includes_full(&self) -> bool {
        matches!(self, Self::Full)
    }
}

#[cfg(feature = "codegen")]
impl std::fmt::Display for NoiseMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Sub-sample fire mode (`--subsample-fire`): variable-dt breakpoint re-solve
/// for latched stateful devices (glow discharge) on the NODAL route.
///
/// A glow strike is detected AFTER the sample's solve, so conduction begins one
/// full inner sample late and the firing instant is quantised to the sample
/// grid. The re-solve splits the firing sample at the linear crossing fraction
/// `alpha`: a dark sub-step over `alpha*dt`, the latch flip, then a lit
/// (backward-Euler) sub-step over `(1-alpha)*dt`, each on matrices rebuilt at
/// the sub-step rate. Nodal-Schur only: the DK route bakes `S = A^-1` at
/// compile time and cannot carry a variable-dt step.
///
/// - [`Auto`](Self::Auto) (default): active when the circuit has a latched
///   device AND routes to nodal-Schur; otherwise inert (byte-identical).
/// - [`On`](Self::On): force; refused on the DK route and on the nodal full-LU
///   sub-path (not implemented there). Inert (warned) without a latched device.
/// - [`Off`](Self::Off): never emit the re-solve (today's whole-sample latch).
#[cfg(feature = "codegen")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SubsampleFireMode {
    /// Active on nodal-Schur with a latched device; inert otherwise. Default.
    #[default]
    Auto,
    /// Force on (nodal-Schur only; refused elsewhere).
    On,
    /// Disable (whole-sample latch flip, as before the feature).
    Off,
}

#[cfg(feature = "codegen")]
impl SubsampleFireMode {
    /// Parse a mode name (case-insensitive) from a CLI flag or config string.
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "auto" => Some(Self::Auto),
            "on" => Some(Self::On),
            "off" => Some(Self::Off),
            _ => None,
        }
    }

    /// Human-readable name for logging and the provenance header.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::On => "on",
            Self::Off => "off",
        }
    }
}

/// Forward-active (frozen-analysis) BJT reduction mode. Controls the 1-D
/// reduction applied to deep-forward-active BJTs (see
/// [`crate::codegen::ir::CircuitIR::detect_forward_active_bjts`]).
///
/// - [`Auto`](Self::Auto) (default): reduce only pure-Ebers-Moll BJTs, for
///   which the 1-D emission is EXACT. Gummel-Poon / ISE / self-heating /
///   parasitic-carded BJTs stay full-2-D. Historical behavior — byte-identical.
/// - [`Off`](Self::Off): no reduction — every BJT stays full-2-D (parity
///   escape hatch / bisecting).
/// - [`Force`](Self::Force): additionally reduce accuracy-excluded BJTs
///   (Gummel-Poon, ISE, parasitic) to 1-D with a per-device WARNING. This
///   DROPS the qb base-charge term (Early effect + high-level injection) and is
///   NOT accuracy-safe under signal — the collector swing modulates qb, which a
///   compile-time reduction cannot see (expect up to ~1-2 dB deviation under
///   hard drive, larger for parasitic-carded devices). Self-heating BJTs are
///   NEVER force-reduced (structural: a 1-D slot would alias the thermal
///   update's (Ic,Ib) slot pair).
#[cfg(feature = "codegen")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BjtFaMode {
    /// No FA reduction — all BJTs full-2-D. Default: the reduction's premise
    /// (the junction never leaves forward-active) is checked only at the DC
    /// operating point, and ordinary drive can break it.
    #[default]
    Off,
    /// Reduce pure-Ebers-Moll BJTs found forward-active at the DC operating
    /// point (exact while they stay there). On request only; a sample on which
    /// a reduced BJT leaves forward-active is counted unsolved.
    Auto,
    /// Also force-reduce GP/ISE/parasitic BJTs (warned, accuracy-lossy).
    Force,
}

/// Configuration for code generation
#[cfg(feature = "codegen")]
#[derive(Debug, Clone)]
pub struct CodegenConfig {
    /// Circuit name for generated code
    pub circuit_name: String,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Maximum iterations for Newton-Raphson
    pub max_iterations: usize,
    /// Convergence tolerance
    pub tolerance: f64,
    /// Input resistance (Thevenin equivalent) of the primary input port (port 0).
    pub input_resistance: f64,
    /// Input node index of the primary input port (port 0).
    pub input_node: usize,
    /// Additional input node indices for multi-input (M=0) circuits, one per
    /// extra port beyond the primary (port 0). Empty for the single-input case,
    /// in which the generated code is byte-identical to the pre-multi-input
    /// emitter. Parallel to [`Self::extra_input_resistances`]. See
    /// `local-docs/multi-input-ports-plan.md`. Multi-input is only valid for
    /// linear (M=0) circuits; M>0 is rejected before codegen.
    pub extra_input_nodes: Vec<usize>,
    /// Per-port Thevenin resistance for each entry in [`Self::extra_input_nodes`]
    /// (same length, same order). The primary port uses [`Self::input_resistance`].
    pub extra_input_resistances: Vec<f64>,
    /// Output node indices (one per output channel)
    pub output_nodes: Vec<usize>,
    /// Oversampling factor (1, 2, or 4). Default 1 (no oversampling).
    /// Factor > 1 reduces aliasing from nonlinearities.
    pub oversampling_factor: usize,
    /// Output scale factors applied after DC blocking (one per output, default [1.0])
    pub output_scales: Vec<f64>,
    /// Post-DC-block output limiter ceiling (volts). Generated code emits
    /// `scaled.clamp(-output_clamp_v, output_clamp_v)` and the diag_clamp_count
    /// increments above this threshold. Default 10.0 V preserves the historical
    /// "Signal Level Contract" (see `docs/aidocs/SIGNAL_LEVELS.md`). Raise for
    /// circuits whose rails exceed ±10 V (e.g. a Wurlitzer 200A power amp at
    /// ±22 V needs ±30 V or higher); leave at default for line-level circuits.
    /// Ignored when DC blocking is disabled — the scaled output is still
    /// NaN-guarded but not clamped.
    pub output_clamp_v: f64,
    /// Include DC blocking filter on outputs (default true).
    /// Set to false for circuits with output coupling caps or when the downstream
    /// pipeline handles DC offset. Removes the 5Hz HPF and its settling time.
    pub dc_block: bool,
    /// Use backward Euler integration instead of trapezoidal.
    /// Unconditionally stable (L-stable) — fixes divergence in high-gain feedback
    /// amplifiers where trapezoidal's imaginary-axis preservation causes oscillation.
    /// Trades second-order accuracy for first-order, giving slight HF rolloff.
    pub backward_euler: bool,
    /// Escape hatch: force trapezoidal integration even when the ring
    /// predicate (`codegen::ring`, `docs/aidocs/RING_PREDICATE.md`) would
    /// promote a default build to backward Euler. Also opts out of the
    /// runtime BE-latch. For bisecting regressions or reproducing legacy
    /// output only — auto-promoted BE is the correct default on circuits
    /// the predicate promotes.
    /// Ignored when `backward_euler` is already `true`.
    pub force_trap: bool,
    /// Which nodal sub-path to emit — see [`NodalSubPath`]. Default
    /// [`NodalSubPath::Auto`], which is the shipping behaviour.
    ///
    /// Tests that need to exercise the full-LU emitter historically abused a
    /// dummy behavioral source (`B_frc frc 0 V={0}`) as a routing lever; this
    /// replaces that idiom with an explicit, semantics-free switch, so the
    /// presence of a behavioral source means exactly one thing.
    pub nodal_sub_path_override: NodalSubPathOverride,
    /// Escape hatch for the fail-loud refusal of a relaxing-section / delayed-
    /// overvoltage / subnormal (KSUB) glow on the nodal full-LU sub-path
    /// (design review). By default `compile` REFUSES that combination: the
    /// full-LU device eval runs the static maintaining line for the lit branch
    /// while the strike seed and extinction test read the section model — a
    /// mixed, silently-wrong model that has masqueraded as a circuit failure.
    /// Set true to knowingly emit today's static line on full-LU (the section
    /// keys go inert); provenance then carries `glow_sections: inert (full-lu)`.
    /// Default **false**.
    pub allow_static_glow_on_full_lu: bool,
    /// Disable adaptive backward Euler fallback for the DK codegen path.
    /// When false (default), the generated code includes pre-computed BE matrices
    /// and can fall back to BE for individual samples where trapezoidal NR diverges.
    /// Set to true to save memory (~1.4KB for N=8, M=4) and compile time when
    /// the circuit is known to be well-conditioned.
    pub disable_be_fallback: bool,
    /// Strategy for op-amp supply rail saturation. Default [`OpampRailMode::Auto`],
    /// which inspects the circuit topology and picks the cheapest correct mode.
    /// See [`OpampRailMode`] for the full menu and trade-offs.
    pub opamp_rail_mode: OpampRailMode,
    /// Authentic circuit-noise mode. Default [`NoiseMode::Off`] — zero cost,
    /// byte-identical codegen. See [`NoiseMode`] and `docs/aidocs/NOISE.md`.
    pub noise_mode: NoiseMode,
    /// Master seed for deterministic noise. `0` (default) → seeded from system
    /// entropy at `CircuitState::default()`. Nonzero → every stream derived from
    /// this via SplitMix64, reproducible across runs. Ignored when
    /// `noise_mode == NoiseMode::Off`.
    pub noise_master_seed: u64,
    /// Emit `CircuitState::recompute_dc_op()` (runtime DC operating-point re-solve).
    ///
    /// When enabled, the generated code includes a runtime DC operating point
    /// solver so plugins can re-solve the bias after changing pot/switch values
    /// at arbitrary magnitudes — avoiding the warmup loop required to settle
    /// into a jittered equilibrium. Default `false` (no emission, byte-identical
    /// output to pre-Phase-E codegen). See `docs/aidocs/DC_OP.md` for scope.
    ///
    /// MVP scope: Direct-NR only, no source/Gmin stepping, no basin-trap handling.
    /// Not supported for DK circuits with parasitic-R BJTs (use nodal path).
    pub emit_dc_op_recompute: bool,
    /// TEST-ONLY: replace the DC operating-point Newton budget (iterations
    /// per solve attempt, [`crate::dc_op::DcOpConfig::max_iterations`]) of
    /// every DC solve the build makes. It exists so the unconverged-DC-OP
    /// refusal has a witness that does not depend on an open convergence bug
    /// staying open. It cannot ship a wrong operating point: a solve either
    /// converges within the budget (and passes the same step test and KCL
    /// residual gate as always), or it does not and the build is refused, or,
    /// under `--allow-unconverged-dc-op`, built with `DC_OP_CONVERGED = false`.
    /// `None` (every real build) keeps the default budget.
    #[doc(hidden)]
    pub dc_op_max_iterations: Option<usize>,
    /// The Newton iteration budget for a build the ring predicate promotes
    /// to backward Euler (`ir::CircuitIR::ring_promotion`), which replaces
    /// `max_iterations` in the rebuild. `None` keeps `max_iterations`. The
    /// CLI budgets a trapezoidal build and a promoted one differently
    /// (`pipeline::auto_tune_max_iter`), and only the finished trapezoidal IR
    /// can say which one ships.
    pub max_iterations_be_promoted: Option<usize>,
    /// Runtime feedback-injection sources (`.inject`), resolved by the caller
    /// (CLI) from node names to 0-indexed node rows. The caller must ALSO
    /// stamp each source's conductance (`1/resistance`) into
    /// `mna.g[node][node]` before building the kernel. Empty for decks without
    /// `.inject` → generated code is byte-identical to the pre-inject emitter.
    /// See `local-docs/inject-directive-plan.md`.
    pub injections: Vec<crate::codegen::ir::InjectionSpec>,
    /// Raw inner-rate tap probes (`.tap`). Empty when no `.tap` directive.
    pub taps: Vec<crate::codegen::ir::TapSpec>,
    /// Forward-active BJT reduction mode. Default [`BjtFaMode::Off`]. See
    /// [`BjtFaMode`] and the `--bjt-fa` CLI flag.
    pub bjt_fa_mode: BjtFaMode,
    /// Sub-sample fire (variable-dt glow-strike breakpoint re-solve) mode.
    /// Default [`SubsampleFireMode::Auto`] — active only on nodal-Schur decks
    /// with a latched device, byte-identical everywhere else. See
    /// [`SubsampleFireMode`] and the `--subsample-fire` CLI flag.
    pub subsample_fire: SubsampleFireMode,
    /// Diagnostic multiplier on the lit sub-step target length (`factor * tau`,
    /// where tau = `glow_lit_tau_min`). Default **1.0** — the last tested-safe
    /// point: the lock-margin sweep (design review) was flat from
    /// x = h/tau_true ≈ 0.017 to ≈ 1.1, and factor 1.0 keeps even a deck where
    /// the heuristic is EXACT (tau_min == tau_true) at x = 1.0, inside that
    /// ceiling; 2.0 would put it at x = 2, outside anything measured. Smaller =
    /// finer (0.5 buys nothing over 1.0 at ~36% more CPU). Diagnostic bisection
    /// tool (family of `--force-trap`), NOT a per-deck tuning knob.
    /// NOTE: if tau is ever DERIVED from the actual discharge loop, the factor
    /// becomes x directly and this default must be re-taken (design review).
    pub subsample_lit_factor: f64,
}

#[cfg(feature = "codegen")]
impl CodegenConfig {
    /// Number of input ports (1 for the single-input case).
    pub fn num_inputs(&self) -> usize {
        1 + self.extra_input_nodes.len()
    }

    /// All input node indices, port 0 first, then the extra ports in order.
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

    /// Validate configuration parameters.
    pub fn validate(&self) -> Result<(), CodegenError> {
        if !(self.sample_rate > 0.0 && self.sample_rate.is_finite()) {
            return Err(CodegenError::InvalidConfig(format!(
                "sample_rate must be positive and finite, got {}",
                self.sample_rate
            )));
        }
        if !(self.tolerance > 0.0 && self.tolerance.is_finite()) {
            return Err(CodegenError::InvalidConfig(format!(
                "tolerance must be positive and finite, got {}",
                self.tolerance
            )));
        }
        if self.max_iterations == 0 {
            return Err(CodegenError::InvalidConfig(
                "max_iterations must be > 0".to_string(),
            ));
        }
        if !(self.input_resistance > 0.0 && self.input_resistance.is_finite()) {
            return Err(CodegenError::InvalidConfig(format!(
                "input_resistance must be positive and finite, got {}",
                self.input_resistance
            )));
        }
        for (i, &r) in self.extra_input_resistances.iter().enumerate() {
            if !(r > 0.0 && r.is_finite()) {
                return Err(CodegenError::InvalidConfig(format!(
                    "extra_input_resistances[{}] must be positive and finite, got {}",
                    i, r
                )));
            }
        }
        if self.extra_input_nodes.len() != self.extra_input_resistances.len() {
            return Err(CodegenError::InvalidConfig(format!(
                "extra_input_nodes length ({}) must match extra_input_resistances length ({})",
                self.extra_input_nodes.len(),
                self.extra_input_resistances.len()
            )));
        }
        for (i, &scale) in self.output_scales.iter().enumerate() {
            if !scale.is_finite() {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_scales[{}] must be finite, got {}",
                    i, scale
                )));
            }
        }
        if !(self.output_clamp_v > 0.0 && self.output_clamp_v.is_finite()) {
            return Err(CodegenError::InvalidConfig(format!(
                "output_clamp_v must be positive and finite, got {}",
                self.output_clamp_v
            )));
        }
        if !self.output_scales.is_empty()
            && !self.output_nodes.is_empty()
            && self.output_scales.len() != self.output_nodes.len()
        {
            return Err(CodegenError::InvalidConfig(format!(
                "output_scales length ({}) must match output_nodes length ({})",
                self.output_scales.len(),
                self.output_nodes.len()
            )));
        }
        Ok(())
    }
}

#[cfg(feature = "codegen")]
impl Default for CodegenConfig {
    fn default() -> Self {
        Self {
            circuit_name: "unnamed_circuit".to_string(),
            sample_rate: 44100.0,
            max_iterations: 100,
            tolerance: 1e-9,
            input_resistance: 1.0, // 1Ω default (near-ideal voltage source)
            input_node: 0,
            extra_input_nodes: Vec::new(),
            extra_input_resistances: Vec::new(),
            output_nodes: vec![0],
            oversampling_factor: 1,
            output_scales: vec![1.0],
            output_clamp_v: 10.0,
            dc_block: true,
            backward_euler: false,
            force_trap: false,
            nodal_sub_path_override: NodalSubPathOverride::Auto,
            allow_static_glow_on_full_lu: false,
            disable_be_fallback: false,
            opamp_rail_mode: OpampRailMode::Auto,
            noise_mode: NoiseMode::Off,
            noise_master_seed: 0,
            emit_dc_op_recompute: false,
            max_iterations_be_promoted: None,
            dc_op_max_iterations: None,
            injections: Vec::new(),
            taps: Vec::new(),
            bjt_fa_mode: BjtFaMode::Off,
            subsample_fire: SubsampleFireMode::Auto,
            subsample_lit_factor: 1.0,
        }
    }
}

/// Error type for code generation failures
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum CodegenError {
    /// Invalid kernel configuration
    InvalidKernel(String),
    /// Unsupported circuit topology
    UnsupportedTopology(String),
    /// Invalid device model
    InvalidDevice(String),
    /// Invalid configuration parameter
    InvalidConfig(String),
    /// Template rendering error
    TemplateError(String),
    /// An upstream DK error
    Dk(crate::dk::DkError),
    /// An upstream MNA error
    Mna(crate::mna::MnaError),
    /// The DK path refuses a circuit whose DC operating point has a growing
    /// (right-half-plane) pole: a self-starting oscillator switches
    /// regeneratively, and DK cannot contain an unsolved fold sample the way
    /// the nodal solver does. An auto route rebuilds on nodal; a forced
    /// `--solver dk` fails with this reason.
    SelfStartingOscillator(String),
}

/// Classify whether the nodal emitter can currently stamp a behavioral source.
///
/// Wired today: `I={}` current sources AND `V={}` voltage sources (augmented
/// constraint row) whose expressions reference node voltages, `time`, `ddt`,
/// `idt`, and named `.param`/`.runtime` parameters (parameter resolution is
/// validated separately in `generate_nodal`). Deferred (errors loudly):
/// branch-current references.
/// See `docs/aidocs/BEHAVIORAL_SOURCES.md §Codegen integration plan`.
fn behavioral_emitter_supported(b: &crate::mna::BehavioralSourceInfo) -> Result<(), String> {
    if !b.expr.referenced_branches().is_empty() {
        return Err(format!(
            "branch-current references {:?} are not yet wired in codegen",
            b.expr.referenced_branches()
        ));
    }
    Ok(())
}

impl std::fmt::Display for CodegenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (label, msg) = match self {
            CodegenError::InvalidKernel(s) => ("Invalid kernel", s.as_str()),
            CodegenError::UnsupportedTopology(s) => ("Unsupported topology", s.as_str()),
            CodegenError::InvalidDevice(s) => ("Invalid device", s.as_str()),
            CodegenError::InvalidConfig(s) => ("Invalid config", s.as_str()),
            CodegenError::TemplateError(s) => ("Template error", s.as_str()),
            CodegenError::SelfStartingOscillator(s) => ("Self-starting oscillator", s.as_str()),
            CodegenError::Dk(e) => return write!(f, "Codegen error: {}", e),
            CodegenError::Mna(e) => return write!(f, "Codegen error: {}", e),
        };
        write!(f, "{label}: {msg}")
    }
}

impl std::error::Error for CodegenError {}

impl From<crate::dk::DkError> for CodegenError {
    fn from(e: crate::dk::DkError) -> Self {
        CodegenError::Dk(e)
    }
}

impl From<crate::mna::MnaError> for CodegenError {
    fn from(e: crate::mna::MnaError) -> Self {
        CodegenError::Mna(e)
    }
}

/// Generated circuit solver code.
#[cfg(feature = "codegen")]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct GeneratedCode {
    /// The generated Rust source code
    pub code: String,
    /// Number of circuit nodes (excluding ground)
    pub n: usize,
    /// Total nonlinear dimension (sum of device dimensions)
    pub m: usize,
    /// Metadata about auto-detected decisions made during codegen.
    pub meta: CodegenMeta,
}

/// A circuit's IR, ready for [`CodeGenerator::emit`].
#[derive(Debug, Clone)]
pub struct PreparedIr {
    /// The IR.
    pub ir: CircuitIR,
    /// The parasitic caps auto-inserted before the IR was built (empty when
    /// the circuit has capacitance of its own).
    pub parasitic_caps: Vec<crate::mna::ParasiticCap>,
}

/// Metadata about decisions made during code generation.
///
/// Every field here is actually populated by the codegen pipeline (the CLI
/// consumes a subset today; the rest are available for diagnostics without
/// `RUST_LOG=info`). The Schur-vs-full-LU sub-path decision IS now surfaced,
/// in `nodal_sub_path` — and it is surfaced the way this comment used to
/// prescribe: **returned from the emitter**, not recomputed here. Recomputing
/// it in the pipeline would create a second source of truth that could silently
/// diverge from what the emitter actually did.
#[cfg(feature = "codegen")]
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
pub struct CodegenMeta {
    /// Whether backward Euler was auto-selected by the ring predicate.
    /// `false` for every explicit selection (`--backward-euler`, `.integrator
    /// be`, behavioral-source forcing) — equivalent to
    /// `integrator_selection == IntegratorSelection::BeAuto`.
    pub backward_euler_auto: bool,
    /// Full record of how the shipped integrator was selected (CLI flag,
    /// `.integrator` directive, behavioral forcing, auto-promotion, or
    /// default trap). The CLI summary prints from this so a directive-pinned
    /// build is never reported as auto-promoted.
    pub integrator_selection: ir::IntegratorSelection,
    /// Why a default build integrates the way it does: the ring predicate's
    /// verdict in one line (`ir::CircuitIR::integration_reason`). Empty when
    /// the integrator was pinned.
    pub integration_reason: String,
    /// Spectral radius of the trapezoidal charge propagator at the DC
    /// operating point, as the ring predicate measured it, when auto-BE fired
    /// (0.0 otherwise). At the internal (oversampled) rate.
    pub backward_euler_spectral_radius: f64,
    /// DC operating point convergence method (e.g. "Direct NR", "Source Stepping").
    pub dc_op_method: String,
    /// DC operating point iteration count.
    pub dc_op_iterations: usize,
    /// Whether DC operating point converged.
    pub dc_op_converged: bool,
    /// Railed op-amp outputs at the DC operating point (`dc_op::RailPin::label`).
    pub dc_op_rail_pin: String,
    /// Whether a sparse LU elimination schedule was baked (nodal path;
    /// requires G_aug density < 40% and N >= 8). `false` on the DK path.
    pub sparse_lu_enabled: bool,
    /// G_aug sparsity-pattern density (0.0..1.0 fraction of nonzeros) that
    /// drove the sparse-LU decision. 0.0 when not applicable (DK path or
    /// M=0 circuits, where no pattern is computed).
    pub sparse_lu_density: f64,
    /// The 10 pF parasitic caps auto-inserted across device junctions of a
    /// capacitor-free nonlinear circuit, by device and node name; empty when
    /// the circuit has capacitance of its own. Part of the simulated circuit:
    /// `melange validate` adds them to its SPICE reference.
    pub parasitic_caps: Vec<crate::mna::ParasiticCap>,
    /// Which nodal sub-path the emitter actually generated, reported BY the
    /// emitter. `None` on the DK path (no sub-path applies).
    ///
    /// This used to be unreported, which made it unobservable: a netlist author
    /// could not confirm a circuit reached full-LU, and no golden baseline could
    /// pin it, so a change that silently moved a circuit between sub-paths was
    /// undetectable.
    pub nodal_sub_path: Option<NodalSubPath>,
    /// The nodal trap `spectral_radius_s_aneg` (`Some` sub-path only; 0.0 on the
    /// DK path). Distinct from [`Self::backward_euler_spectral_radius`] and from
    /// the DK-kernel `routing.spectral_radius` printed elsewhere.
    ///
    /// ⚠️ It is ONE of several full-LU triggers, not the decision. This doc and
    /// the CLI line built on it both used to say it "GOVERNED" the sub-path, and
    /// that was false wherever another predicate fired first — `steve-1073-preamp`
    /// routes full-LU on `s-ill-conditioned` (max|S| = 5.00e8) at rho = 0.9879,
    /// which is below every rho threshold and decided nothing. Read
    /// [`Self::nodal_full_lu_trigger`] for what actually fired; this is context.
    pub nodal_spectral_radius: f64,
    /// WHICH predicate sent this circuit to full-LU, named by the emitter's own
    /// chain (`s-ill-conditioned`, `positive-k`, `k-degenerate`, `override`, …).
    /// `None` on DK and on Schur. Emitter-reported, never re-derived here.
    pub nodal_full_lu_trigger: Option<&'static str>,
}

/// Build the codegen metadata block from the finished IR. Shared by the
/// DK-with-DC-OP and nodal generate paths, which built it identically.
#[cfg(feature = "codegen")]
fn build_codegen_meta(
    ir: &CircuitIR,
    parasitic_caps: &[crate::mna::ParasiticCap],
    nodal_sub_path: Option<NodalSubPath>,
    nodal_full_lu_trigger: Option<&'static str>,
) -> CodegenMeta {
    let backward_euler_auto = ir.integrator_selection == ir::IntegratorSelection::BeAuto;
    CodegenMeta {
        backward_euler_auto,
        integrator_selection: ir.integrator_selection,
        integration_reason: ir.integration_reason.clone(),
        // Report the discriminator's trap rho (internal-rate matrices under
        // oversampling) only when auto-BE fired, matching the field contract.
        backward_euler_spectral_radius: if backward_euler_auto {
            ir.trap_discriminator_rho
        } else {
            0.0
        },
        dc_op_method: ir.dc_op_method.clone(),
        dc_op_iterations: ir.dc_op_iterations,
        dc_op_converged: ir.dc_op_converged,
        dc_op_rail_pin: ir.dc_op_rail_pin.clone(),
        sparse_lu_enabled: ir.sparsity.lu.is_some(),
        sparse_lu_density: ir.sparsity.g_aug_density,
        parasitic_caps: parasitic_caps.to_vec(),
        nodal_sub_path,
        nodal_spectral_radius: if nodal_sub_path.is_some() {
            ir.matrices.spectral_radius_s_aneg
        } else {
            0.0
        },
        nodal_full_lu_trigger,
    }
}

/// Auto-insert parasitic caps when the C matrix is all zeros but the circuit
/// has nonlinear devices (A = G otherwise degenerates the trapezoidal
/// integrator — no energy storage, no dynamics). The caller owns `patched`
/// storage so the returned borrow can outlive this call. Returns the MNA to
/// use and the caps inserted.
///
/// The caps change the circuit, so the build says so: a SPICE run of the
/// same deck does not have them, and an author reading a difference between
/// the two would otherwise look for it in the solver.
#[cfg(feature = "codegen")]
fn maybe_insert_parasitic_caps<'a>(
    mna: &'a MnaSystem,
    patched: &'a mut Option<MnaSystem>,
) -> (&'a MnaSystem, Vec<crate::mna::ParasiticCap>) {
    if !mna.needs_parasitic_caps() {
        return (mna, Vec::new());
    }
    let caps = mna.parasitic_caps();
    let across: Vec<String> = caps
        .iter()
        .map(|c| format!("{} {}-{}", c.device, c.node_a, c.node_b))
        .collect();
    crate::diag_warn!(
        "Capacitor-free nonlinear circuit: melange adds a {:.0} pF capacitor across each device \
         junction ({}), since without capacitance the solver has no state. They are part of the \
         simulated circuit (10 pF is a pole at 1.6 MHz through 10 kOhm, 16 kHz through 1 MOhm), \
         and a SPICE run of this deck lacks them; `melange validate` adds them to its reference. \
         Put the circuit's own capacitances in the netlist to replace them.",
        crate::mna::PARASITIC_CAP * 1e12,
        across.join(", ")
    );
    let mut m = mna.clone();
    m.add_parasitic_caps();
    *patched = Some(m);
    (patched.as_ref().unwrap(), caps)
}

/// Code generator for circuit solvers
#[cfg(feature = "codegen")]
pub struct CodeGenerator {
    config: CodegenConfig,
}

#[cfg(feature = "codegen")]
impl CodeGenerator {
    /// Create a new code generator with the given configuration
    pub fn new(config: CodegenConfig) -> Self {
        Self { config }
    }

    /// Generate a complete circuit solver module.
    ///
    /// # Arguments
    /// * `kernel` - The compiled DK kernel
    /// * `mna` - MNA system for node mapping
    /// * `netlist` - Original netlist for extracting component values
    ///
    /// # Returns
    /// Generated Rust source code as a string
    pub fn generate(
        &self,
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
    ) -> Result<GeneratedCode, CodegenError> {
        self.generate_with_dc_op(kernel, mna, netlist, None)
    }

    /// Generate Rust solver code with a pre-computed DC operating point
    /// ([`Self::prepare_dk`], then [`Self::emit`]).
    pub fn generate_with_dc_op(
        &self,
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        dc_op: Option<crate::dc_op::DcOpResult>,
    ) -> Result<GeneratedCode, CodegenError> {
        let prepared = self.prepare_dk(kernel, mna, netlist, dc_op)?;
        self.emit(&prepared)
    }

    /// The DK IR of a circuit, every check but the output ports' (those are
    /// [`Self::emit`]'s): the IR fixes the route (a self-starting oscillator is
    /// refused here) and carries the operating point the build ships.
    ///
    /// `dc_op` is the DC operating point of `mna` (see
    /// [`ir::solve_dc_op`]); `None` solves it.
    pub fn prepare_dk(
        &self,
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        dc_op: Option<crate::dc_op::DcOpResult>,
    ) -> Result<PreparedIr, CodegenError> {
        // Behavioral B-sources route to the nodal path (see `routing.rs`); they
        // are not supported on this DK entry. `generate_nodal` is where the
        // node-space stamping lives.
        if !mna.behavioral_sources.is_empty() {
            return Err(CodegenError::UnsupportedTopology(
                "behavioral B-sources must use the nodal path (they route there \
                 automatically; call generate_nodal). See docs/aidocs/BEHAVIORAL_SOURCES.md."
                    .to_string(),
            ));
        }

        // Saturating inductors are flux devices solved by Newton on their
        // augmented row, and a saturating transformer is a T-model of ideal
        // couplings with (1 - k)·L leakage, which S = A^-1 cannot carry. This
        // entry would drop the saturation (the DK IR has no saturating list),
        // so it refuses, whoever calls it.
        if mna.has_saturating_inductor() {
            return Err(CodegenError::UnsupportedTopology(
                "saturating inductors (ISAT=) must use the nodal full-LU path (they route \
                 there automatically; call generate_nodal). See \
                 docs/aidocs/SATURATING_TRANSFORMERS.md."
                    .to_string(),
            ));
        }

        // Validate config
        self.config.validate()?;
        match self.config.oversampling_factor {
            1 | 2 | 4 => {}
            f => {
                return Err(CodegenError::InvalidConfig(format!(
                    "oversampling_factor must be 1, 2, or 4, got {f}"
                )));
            }
        }
        // Validate against n_nodes (original circuit nodes), not kernel.n:
        // kernel.n includes augmented rows (VS/VCVS branch constraints,
        // inductor branch currents). An input_node pointing at an aug row
        // would pass a `< kernel.n` check but stamp the input current into
        // an algebraic constraint row — same rule as output nodes below.
        for &in_node in &self.config.input_node_indices() {
            if in_node >= kernel.n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "input_node {} >= n_nodes={} (original circuit node count; \
                     augmented rows are not valid input nodes)",
                    in_node, kernel.n_nodes
                )));
            }
        }

        // BoyleDiodes mode auto-inserts catch diodes into the netlist, which
        // grows the MNA/kernel dimensions. Rebuilding the DkKernel from the
        // augmented netlist would also change the DK vs nodal routing decision
        // (new N, new spectral radius, etc.), so for now BoyleDiodes is only
        // supported on the nodal path. Explicit request on the DK path is a
        // user error with a clear remedy.
        let resolved = ir::resolve_opamp_rail_mode(mna, self.config.opamp_rail_mode);
        if resolved.mode == OpampRailMode::BoyleDiodes {
            return Err(CodegenError::UnsupportedTopology(
                "OpampRailMode::BoyleDiodes is not yet supported on the DK codegen \
                 path. Either rerun with `--solver nodal` or use a different rail \
                 mode (`--opamp-rail-mode {hard|active-set|active-set-be}`). The \
                 auto-detector never picks BoyleDiodes today, so this error only \
                 triggers on an explicit user override."
                    .to_string(),
            ));
        }
        // ActiveSet / ActiveSetBe pin a railed op-amp output and re-solve the
        // rest of the circuit; only the nodal solver implements that. DK could
        // only clamp the output after the solve, which corrupts the downstream
        // capacitor history — the very case the resolver picked an active-set
        // mode to avoid. `routing::auto_route` sends these circuits to nodal;
        // reaching here means DK was forced, so refuse rather than degrade.
        let has_clamped_opamp = mna
            .opamps
            .iter()
            .any(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()));
        if has_clamped_opamp
            && matches!(
                resolved.mode,
                OpampRailMode::ActiveSet | OpampRailMode::ActiveSetBe
            )
        {
            return Err(CodegenError::UnsupportedTopology(format!(
                "op-amp rail mode {} ({}) needs the pin-and-resolve at the rail, which only \
                 the nodal solver implements; the DK solver can only clamp the output, which \
                 corrupts downstream capacitor history. Use `--solver auto` or `--solver nodal`.",
                resolved.mode.as_str(),
                resolved.reason.as_str()
            )));
        }

        // Auto-insert parasitic caps if C matrix is all zeros and circuit has
        // nonlinear devices. The IR stores G/C for runtime sample rate recomputation,
        // so these must include the parasitic caps (matching the kernel, which also
        // auto-inserts them in from_mna/from_mna_augmented).
        let mut patched_mna = None;
        let (mna, parasitic_caps) = maybe_insert_parasitic_caps(mna, &mut patched_mna);

        let ir = CircuitIR::from_kernel_with_dc_op(kernel, mna, netlist, &self.config, dc_op)?;
        Ok(PreparedIr { ir, parasitic_caps })
    }

    /// Emit the code of a prepared IR, after checking its output ports.
    pub fn emit(&self, prepared: &PreparedIr) -> Result<GeneratedCode, CodegenError> {
        let ir = &prepared.ir;
        if self.config.output_nodes.is_empty() {
            return Err(CodegenError::InvalidConfig(
                "output_nodes must not be empty".to_string(),
            ));
        }
        // Validate against n_nodes (original circuit nodes), not the augmented
        // dimension.
        for (i, &node) in self.config.output_nodes.iter().enumerate() {
            if node >= ir.topology.n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (original circuit node count)",
                    i, node, ir.topology.n_nodes
                )));
            }
        }
        if self.config.output_scales.len() != self.config.output_nodes.len() {
            return Err(CodegenError::InvalidConfig(format!(
                "output_scales length ({}) must match output_nodes length ({})",
                self.config.output_scales.len(),
                self.config.output_nodes.len()
            )));
        }
        let emitted: EmitOutput = select_emitter()?.emit(ir)?;
        let nodal_sub_path = emitted.nodal_sub_path;
        let nodal_full_lu_trigger = emitted.nodal_full_lu_trigger;
        let code = emitted.primary().to_string();

        Ok(GeneratedCode {
            code,
            n: ir.topology.n,
            m: ir.topology.m,
            meta: build_codegen_meta(
                ir,
                &prepared.parasitic_caps,
                nodal_sub_path,
                nodal_full_lu_trigger,
            ),
        })
    }

    /// Generate code using the NodalSolver path (full N×N NR per sample).
    ///
    /// This bypasses the DkKernel entirely — no S=A⁻¹ precomputation, no K matrix.
    /// The generated code does LU factorization per NR iteration, which handles
    /// any circuit topology including transformer-coupled NFB with large inductors.
    ///
    /// # Errors
    /// Returns `CodegenError` if input/output node validation fails or if code
    /// generation encounters an error.
    pub fn generate_nodal(
        &self,
        mna: &MnaSystem,
        netlist: &Netlist,
    ) -> Result<GeneratedCode, CodegenError> {
        let prepared = self.prepare_nodal(mna, netlist, None)?;
        self.emit(&prepared)
    }

    /// The nodal IR of a circuit, every check but the output ports' (see
    /// [`Self::prepare_dk`]); `dc_op` is the DC operating point of `mna`,
    /// `None` solves it.
    pub fn prepare_nodal(
        &self,
        mna: &MnaSystem,
        netlist: &Netlist,
        dc_op: Option<crate::dc_op::DcOpResult>,
    ) -> Result<PreparedIr, CodegenError> {
        // Validate config (same checks as generate, but against MNA dimensions)
        self.config.validate()?;
        match self.config.oversampling_factor {
            1 | 2 | 4 => {}
            f => {
                return Err(CodegenError::InvalidConfig(format!(
                    "oversampling_factor must be 1, 2, or 4, got {f}"
                )));
            }
        }
        for &in_node in &self.config.input_node_indices() {
            if in_node >= mna.n {
                return Err(CodegenError::InvalidConfig(format!(
                    "input_node {} >= n_nodes={}",
                    in_node, mna.n
                )));
            }
        }

        // Behavioral B-sources: the nodal node-space stamping is being brought
        // up incrementally. The `I={}` class (current source over node /
        // time / ddt / idt) is wired; the rest error loudly rather than emit
        // code that silently ignores the source. See
        // docs/aidocs/BEHAVIORAL_SOURCES.md §Codegen integration plan.
        let resolvable_params: std::collections::BTreeSet<&str> = netlist
            .params
            .iter()
            .map(|p| p.name.as_str())
            .chain(netlist.runtime_scalars.iter().map(|r| r.name.as_str()))
            .collect();
        for b in &mna.behavioral_sources {
            if let Err(why) = behavioral_emitter_supported(b) {
                return Err(CodegenError::UnsupportedTopology(format!(
                    "behavioral source '{}': {why}",
                    b.name
                )));
            }
            for p in b.expr.referenced_params() {
                if !resolvable_params.contains(p.as_str()) {
                    return Err(CodegenError::UnsupportedTopology(format!(
                        "behavioral source '{}': unknown parameter '{}' (define it with \
                         `.param {} = <value>` or `.runtime {} <min> <max> as {}`)",
                        b.name, p, p, p, p
                    )));
                }
            }
        }

        // BoyleDiodes mode: augment the netlist with the internal-gain-node
        // scaffolding (catch diodes + output buffer + rail references),
        // rebuild MNA on the augmented netlist, then continue through the
        // rest of the codegen pipeline.
        //
        // CRITICAL: `MnaSystem::from_netlist` builds a fresh MNA from
        // scratch — it does NOT preserve any in-place mutations the caller
        // applied to the original `mna` reference. The CLI stamps the
        // input conductance and device junction caps on the original MNA
        // BEFORE calling `generate_nodal`, so we must re-apply both to the
        // rebuilt augmented MNA or its input row will be near-singular and
        // the LU produces garbage.
        //
        // The MNA-layer half of the BoyleDiodes refactor (auto-detecting
        // `_oa_int_{name}` in `node_map` and stamping Gm/Go at the high-
        // impedance internal node via `R_BOYLE_INT_LOAD`) is in
        // `mna::MnaSystem::from_netlist`. The augmented MNA naturally
        // inherits that behavior.
        let resolved = ir::resolve_opamp_rail_mode(mna, self.config.opamp_rail_mode);
        // The active-set pinned Newton stamps device Jacobians through N_i/N_v
        // and the saturating-inductor flux rows, and accepts an iterate on the
        // same step and flux-row residual checks as the main loop. Behavioral
        // sources (stamped in node space, with a non-diagonal Jacobian) are not
        // in that system, so it could converge to a point that is not a
        // solution. Refuse the combination rather than pin approximately; lift
        // it when their Jacobian is stamped in the pinned system and railing
        // acceptance covers it.
        let clamped_opamp = mna
            .opamps
            .iter()
            .any(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()));
        if clamped_opamp
            && !mna.behavioral_sources.is_empty()
            && matches!(
                resolved.mode,
                OpampRailMode::ActiveSet | OpampRailMode::ActiveSetBe
            )
        {
            return Err(CodegenError::UnsupportedTopology(format!(
                "op-amp rail mode {} cannot be solved on this circuit: it has a behavioral \
                 source, and the pinned solve at the rail does not include behavioral sources \
                 yet, so it would converge to a point that is not a solution. No rail handling \
                 is validated for this combination yet. The explicit modes are not a known \
                 workaround: neither has been measured with a behavioral source, and on a \
                 railing op-amp driving a saturating inductor `--opamp-rail-mode hard` measured \
                 2-290x the reference current (138 V out of a 9 V supply) and `boyle-diodes` \
                 27% low.",
                resolved.mode.as_str()
            )));
        }
        // BoyleDiodes: the catch diodes are circuit elements, added to the
        // netlist BEFORE the MNA is assembled (`build::build` does it) so every
        // pipeline step sees them. Generating from an un-augmented netlist would
        // either rebuild the MNA here — dropping whatever the caller stamped or
        // reduced (`.inject`, `.linearize`, internal-node expansion) — or ship
        // no diodes at all. Refuse instead.
        if resolved.mode == OpampRailMode::BoyleDiodes
            && mna
                .opamps
                .iter()
                .any(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()))
            && !netlist
                .models
                .iter()
                .any(|m| m.name == ir::BOYLE_CATCH_DIODE_MODEL)
        {
            return Err(CodegenError::UnsupportedTopology(
                "OpampRailMode::BoyleDiodes needs the catch diodes in the netlist before the \
                 MNA is assembled (ir::augment_netlist_with_boyle_diodes; \
                 melange_solver::build::build does it)."
                    .to_string(),
            ));
        }

        // Auto-insert parasitic caps if C matrix is all zeros and circuit has
        // nonlinear devices. Without capacitors, A = G and the trapezoidal
        // integrator degenerates (no energy storage → no dynamics).
        let mut patched_mna = None;
        let (mna, parasitic_caps) = maybe_insert_parasitic_caps(mna, &mut patched_mna);

        let ir = CircuitIR::from_mna_with_dc_op(mna, netlist, &self.config, dc_op)?;
        Ok(PreparedIr { ir, parasitic_caps })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_codegen_config_default() {
        let config = CodegenConfig::default();
        assert_eq!(config.sample_rate, 44100.0);
        assert_eq!(config.max_iterations, 100);
        assert!(config.tolerance > 0.0);
        // Default op-amp rail handling is auto-selection.
        assert_eq!(config.opamp_rail_mode, OpampRailMode::Auto);
    }

    #[test]
    fn test_opamp_rail_mode_parse_canonical() {
        assert_eq!(OpampRailMode::parse("auto"), Some(OpampRailMode::Auto));
        assert_eq!(OpampRailMode::parse("none"), Some(OpampRailMode::None));
        assert_eq!(OpampRailMode::parse("hard"), Some(OpampRailMode::Hard));
        assert_eq!(
            OpampRailMode::parse("active-set"),
            Some(OpampRailMode::ActiveSet)
        );
        assert_eq!(
            OpampRailMode::parse("boyle-diodes"),
            Some(OpampRailMode::BoyleDiodes)
        );
    }

    #[test]
    fn test_opamp_rail_mode_parse_aliases_and_case() {
        // Case-insensitive.
        assert_eq!(OpampRailMode::parse("AUTO"), Some(OpampRailMode::Auto));
        assert_eq!(OpampRailMode::parse("Hard"), Some(OpampRailMode::Hard));
        // Underscore variants.
        assert_eq!(
            OpampRailMode::parse("active_set"),
            Some(OpampRailMode::ActiveSet)
        );
        assert_eq!(
            OpampRailMode::parse("boyle_diodes"),
            Some(OpampRailMode::BoyleDiodes)
        );
        // Common short aliases.
        assert_eq!(OpampRailMode::parse("off"), Some(OpampRailMode::None));
        assert_eq!(OpampRailMode::parse("clamp"), Some(OpampRailMode::Hard));
        assert_eq!(
            OpampRailMode::parse("boyle"),
            Some(OpampRailMode::BoyleDiodes)
        );
        assert_eq!(
            OpampRailMode::parse("diodes"),
            Some(OpampRailMode::BoyleDiodes)
        );
    }

    #[test]
    fn test_opamp_rail_mode_parse_rejects_unknown() {
        assert_eq!(OpampRailMode::parse(""), None);
        assert_eq!(OpampRailMode::parse("soft"), None);
        assert_eq!(OpampRailMode::parse("tanh"), None);
        assert_eq!(OpampRailMode::parse("bogus"), None);
    }

    #[test]
    fn test_opamp_rail_mode_display_round_trips() {
        // Display format must parse back to the same variant so CLI logging stays stable.
        for mode in [
            OpampRailMode::Auto,
            OpampRailMode::None,
            OpampRailMode::Hard,
            OpampRailMode::ActiveSet,
            OpampRailMode::BoyleDiodes,
        ] {
            let text = mode.to_string();
            assert_eq!(
                OpampRailMode::parse(&text),
                Some(mode),
                "Display -> parse round-trip failed for {:?} (rendered as {:?})",
                mode,
                text
            );
        }
    }
}
