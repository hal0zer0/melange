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
pub mod ir;
#[cfg(feature = "codegen")]
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
use emitter::Emitter;
#[cfg(feature = "codegen")]
use ir::CircuitIR;
#[cfg(feature = "codegen")]
use rust_emitter::RustEmitter;

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
///   the Klon-class cap-history corruption. Cost: one extra LU back-solve
///   (O(N²)) on samples where clamping is active. Still produces hard-clip
///   harmonics — fine for utility clamping, not ideal for distortion pedals.
///
/// - [`BoyleDiodes`](OpampRailMode::BoyleDiodes): auto-inserted catch diodes
///   per op-amp, anchored to rail-offset voltage sources at `VCC − VOH_DROP`
///   and `VEE + VOL_DROP`. Matches the Boyle macromodel used by every
///   commercial SPICE and produces the soft exponential knee characteristic
///   of real op-amp output stages. Most accurate for distortion circuits
///   (Klon, Tube Screamer, etc.). Cost: +2 N and +2 M per op-amp, plus the
///   synthesized voltage sources' augmented rows.
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
    /// clip with square-wave harmonics. Preserves the steady DC rail value the
    /// op-amp converged to — required for circuits where the rail-clipped value
    /// IS the signal (e.g. control-path op-amps driving a VCA gain port).
    ActiveSet,
    /// On rail engagement, fall through to the BE NR fallback and run the
    /// constrained re-solve against `state.a_be` (backward Euler).
    /// Trapezoidal with post-NR pin develops a Nyquist-rate limit cycle when
    /// the clamp is engaged across multiple samples (the cap-history term
    /// `(2/T)·C·v_prev` alternates sign every sample); BE damps this.
    ///
    /// Required for audio-path op-amps whose output is cap-coupled to a
    /// downstream stage that integrates the op-amp's transient behavior —
    /// e.g. Klon Centaur's tone-out -> C15 -> output. Cost: BE NR runs every
    /// sample where the rail is engaged (~2x NR work for those samples).
    /// Compared to ActiveSet, produces cleaner clipped output but slightly
    /// different envelope dynamics (BE damps cap-coupled feedback more
    /// aggressively).
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
    /// Input resistance (Thevenin equivalent)
    pub input_resistance: f64,
    /// Input node index
    pub input_node: usize,
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
    /// Include DC operating point in generated code
    pub include_dc_op: bool,
    /// Maximum NR iterations for DC operating point solver
    pub dc_op_max_iterations: usize,
    /// Convergence tolerance for DC operating point solver
    pub dc_op_tolerance: f64,
    /// Include DC blocking filter on outputs (default true).
    /// Set to false for circuits with output coupling caps or when the downstream
    /// pipeline handles DC offset. Removes the 5Hz HPF and its settling time.
    pub dc_block: bool,
    /// Number of silent samples to process after pot-triggered matrix rebuild.
    /// Settles the NR to the new nonlinear DC operating point. Default 64.
    /// Set to 0 for zero-latency pot changes (may glitch on large pot swings).
    pub pot_settle_samples: usize,
    /// Use backward Euler integration instead of trapezoidal.
    /// Unconditionally stable (L-stable) — fixes divergence in high-gain feedback
    /// amplifiers where trapezoidal's imaginary-axis preservation causes oscillation.
    /// Trades second-order accuracy for first-order, giving slight HF rolloff.
    pub backward_euler: bool,
    /// Escape hatch: force trapezoidal integration even when the nodal
    /// auto-detector would promote to backward Euler (trap propagation
    /// operator `S*A_neg` has spectral radius > 1.002, which seeds a
    /// persistent Nyquist-rate limit cycle in `v_prev`). For bisecting
    /// regressions or reproducing legacy output only — auto-promoted BE
    /// is the correct default on circuits where trap is unstable.
    /// Ignored when `backward_euler` is already `true`.
    pub force_trap: bool,
    /// Per-sample gmin-stepping continuation for stiff hard-switching NR
    /// (full-LU path). When the trapezoidal Newton solve fails to converge at a
    /// switching transition — a positive-feedback junction pinned deep into
    /// saturation (v/vt≈300, i_dev at the safe_exp ceiling) leaves the Jacobian
    /// catastrophically ill-conditioned and the residual stuck flat — this
    /// retries with a Gmin homotopy (add `gmin·|N_v|` to the device-node
    /// diagonals, ramp 1e-2→1e-12, warm-starting each level) before the BE
    /// fallback, mirroring the DC-OP solver's Gmin stepping (DC_OP.md).
    ///
    /// Off by default: it only changes behaviour on samples that fail plain
    /// trap-NR (rare; byte-identical otherwise), but for those it converges the
    /// TRAP solution instead of the BE fallback and costs up to N_levels×MAX_ITER
    /// iterations on that sample — a robustness/latency trade best opted into for
    /// offline references (e.g. the Farfisa G10 calibration reference, where
    /// non-converged samples integrate into oscillator PITCH error). Full-LU
    /// nodal path only.
    pub gmin_continuation: bool,
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
    /// Emit `CircuitState::recompute_dc_op()` (Oomox plugin roadmap P6).
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
    /// Whether `routing::auto_route` selected the nodal path because the
    /// DK-kernel spectral radius exceeded the trap-instability threshold
    /// (`RoutingDecision::dk_unstable`). Threaded in by the caller
    /// (CLI/library routing call sites) so the nodal auto-BE promotion gate
    /// can use it as a corroborating (not unconditional) signal — see
    /// `stability::router_corroborates_marginal_instability`.
    ///
    /// The router measures spectral radius on the un-reduced DK-kernel
    /// `S·A_neg` via a fixed-iteration-count, non-deflected power iteration;
    /// the nodal IR build recomputes a more accurate (converged,
    /// input-deflated) estimate on its own matrices. The two can straddle
    /// the 1.002 promotion threshold on the same circuit (observed on
    /// `wurli-power-amp`: DK-kernel rho = 1.0040 vs nodal-deflated
    /// rho = 1.0005) — but the router's raw number is also demonstrably
    /// inaccurate on some circuits (verified on tungsten-thunder-horse:
    /// 1.1163 raw vs 0.8157 converged+deflated, comfortably trap-stable), so
    /// it is NOT trusted unconditionally. It only tips the balance when the
    /// nodal-local estimate independently corroborates a marginal
    /// (rho > 0.999, negative dominant eigenvalue) mode that the local
    /// gain-gate alone would otherwise decline to promote. Default `false`
    /// (nodal auto-BE promotion unchanged) when unset — e.g. library callers
    /// that construct `CodegenConfig` without running `routing::auto_route`
    /// first.
    pub router_dk_unstable: bool,
    /// Diagnostic companion to `router_dk_unstable`: the DK-kernel spectral
    /// radius that produced it (0.0 when `router_dk_unstable` is false).
    /// Used only for log messages when the router signal is the deciding
    /// factor (the nodal-local estimate alone did not cross the promotion
    /// threshold).
    pub router_dk_spectral_radius: f64,
}

#[cfg(feature = "codegen")]
impl CodegenConfig {
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
            output_nodes: vec![0],
            oversampling_factor: 1,
            output_scales: vec![1.0],
            output_clamp_v: 10.0,
            include_dc_op: true,
            dc_op_max_iterations: 200,
            dc_op_tolerance: 1e-9,
            dc_block: true,
            pot_settle_samples: 64,
            backward_euler: false,
            force_trap: false,
            gmin_continuation: false,
            disable_be_fallback: false,
            opamp_rail_mode: OpampRailMode::Auto,
            noise_mode: NoiseMode::Off,
            noise_master_seed: 0,
            emit_dc_op_recompute: false,
            router_dk_unstable: false,
            router_dk_spectral_radius: 0.0,
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

/// Metadata about decisions made during code generation.
///
/// Every field here is actually populated by the codegen pipeline (the CLI
/// consumes a subset today; the rest are available for diagnostics without
/// `RUST_LOG=info`). The Schur-vs-full-LU sub-path decision is NOT surfaced
/// here: it is made inside the nodal emitter from the finished IR, and
/// duplicating that gate in the pipeline would create a second source of
/// truth that could silently diverge from what the emitter actually did.
/// Surfacing it honestly means returning it from the emitter — future work.
#[cfg(feature = "codegen")]
#[derive(Debug, Clone, Default)]
#[non_exhaustive]
pub struct CodegenMeta {
    /// Whether backward Euler was auto-selected (spectral radius > threshold).
    /// `false` for every explicit selection (`--backward-euler`, `.integrator
    /// be`, behavioral-source forcing) — equivalent to
    /// `integrator_selection == IntegratorSelection::BeAuto`.
    pub backward_euler_auto: bool,
    /// Full record of how the shipped integrator was selected (CLI flag,
    /// `.integrator` directive, behavioral forcing, auto-promotion, or
    /// default trap). The CLI summary prints from this so a directive-pinned
    /// build is never reported as auto-promoted.
    pub integrator_selection: ir::IntegratorSelection,
    /// Spectral radius measured by the trap-stability discriminator that
    /// triggered auto-BE (0.0 when auto-BE did not fire). Evaluated on the
    /// trap `S·A_neg` pair at the rate the solver actually ships — under
    /// oversampling that is the internal (oversampled) rate.
    pub backward_euler_spectral_radius: f64,
    /// DC operating point convergence method (e.g. "Direct NR", "Source Stepping").
    pub dc_op_method: String,
    /// DC operating point iteration count.
    pub dc_op_iterations: usize,
    /// Whether DC operating point converged.
    pub dc_op_converged: bool,
    /// Whether a sparse LU elimination schedule was baked (nodal path;
    /// requires G_aug density < 40% and N >= 8). `false` on the DK path.
    pub sparse_lu_enabled: bool,
    /// G_aug sparsity-pattern density (0.0..1.0 fraction of nonzeros) that
    /// drove the sparse-LU decision. 0.0 when not applicable (DK path or
    /// M=0 circuits, where no pattern is computed).
    pub sparse_lu_density: f64,
    /// Whether parasitic caps were auto-inserted.
    pub parasitic_caps_inserted: bool,
}

/// Build the codegen metadata block from the finished IR. Shared by the
/// DK-with-DC-OP and nodal generate paths, which built it identically.
#[cfg(feature = "codegen")]
fn build_codegen_meta(ir: &CircuitIR, parasitic_caps_inserted: bool) -> CodegenMeta {
    let backward_euler_auto = ir.integrator_selection == ir::IntegratorSelection::BeAuto;
    CodegenMeta {
        backward_euler_auto,
        integrator_selection: ir.integrator_selection,
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
        sparse_lu_enabled: ir.sparsity.lu.is_some(),
        sparse_lu_density: ir.sparsity.g_aug_density,
        parasitic_caps_inserted,
    }
}

/// Auto-insert parasitic caps when the C matrix is all zeros but the circuit
/// has nonlinear devices (A = G otherwise degenerates the trapezoidal
/// integrator — no energy storage, no dynamics). The caller owns `patched`
/// storage so the returned borrow can outlive this call. Returns the MNA to
/// use and whether caps were inserted.
#[cfg(feature = "codegen")]
fn maybe_insert_parasitic_caps<'a>(
    mna: &'a MnaSystem,
    patched: &'a mut Option<MnaSystem>,
    ctx: &str,
) -> (&'a MnaSystem, bool) {
    let inserted = mna.m > 0 && !mna.c.iter().any(|row| row.iter().any(|&v| v != 0.0));
    if inserted {
        log::info!(
            "{ctx}: C matrix is all zeros with M={} nonlinear devices; auto-inserting parasitic caps",
            mna.m
        );
        let mut m = mna.clone();
        m.add_parasitic_caps();
        *patched = Some(m);
        (patched.as_ref().unwrap(), true)
    } else {
        (mna, false)
    }
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
        if self.config.input_node >= kernel.n_nodes {
            return Err(CodegenError::InvalidConfig(format!(
                "input_node {} >= n_nodes={} (original circuit node count; \
                 augmented rows are not valid input nodes)",
                self.config.input_node, kernel.n_nodes
            )));
        }
        if self.config.output_nodes.is_empty() {
            return Err(CodegenError::InvalidConfig(
                "output_nodes must not be empty".to_string(),
            ));
        }
        for (i, &node) in self.config.output_nodes.iter().enumerate() {
            // Validate against n_nodes (original circuit nodes), not n_aug
            let n_nodes = kernel.n_nodes;
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (original circuit node count)",
                    i, node, n_nodes
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

        self.generate_with_dc_op(kernel, mna, netlist, None)
    }

    /// Generate Rust solver code with a pre-computed DC operating point.
    ///
    /// When `dc_op` is provided, it is used instead of running the internal DC OP solver.
    /// This is useful when the MNA has been expanded (e.g., with internal nodes for
    /// parasitic BJTs) after the DC OP was computed on the original system.
    pub fn generate_with_dc_op(
        &self,
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        dc_op: Option<crate::dc_op::DcOpResult>,
    ) -> Result<GeneratedCode, CodegenError> {
        // BoyleDiodes mode auto-inserts catch diodes into the netlist, which
        // grows the MNA/kernel dimensions. Rebuilding the DkKernel from the
        // augmented netlist would also change the DK vs nodal routing decision
        // (new N, new spectral radius, etc.), so for now BoyleDiodes is only
        // supported on the nodal path. Explicit request on the DK path is a
        // user error with a clear remedy.
        let resolved = ir::refine_active_set_for_audio_path(
            ir::resolve_opamp_rail_mode(mna, self.config.opamp_rail_mode),
            mna,
            netlist,
        );
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

        // Auto-insert parasitic caps if C matrix is all zeros and circuit has
        // nonlinear devices. The IR stores G/C for runtime sample rate recomputation,
        // so these must include the parasitic caps (matching the kernel, which also
        // auto-inserts them in from_mna/from_mna_augmented).
        let mut patched_mna = None;
        let (mna, parasitic_caps_inserted) =
            maybe_insert_parasitic_caps(mna, &mut patched_mna, "Codegen");

        let ir = CircuitIR::from_kernel_with_dc_op(kernel, mna, netlist, &self.config, dc_op)?;
        let code = RustEmitter::new()?.emit(&ir)?;

        Ok(GeneratedCode {
            code,
            n: ir.topology.n,
            m: ir.topology.m,
            meta: build_codegen_meta(&ir, parasitic_caps_inserted),
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
        if self.config.input_node >= mna.n {
            return Err(CodegenError::InvalidConfig(format!(
                "input_node {} >= n_nodes={}",
                self.config.input_node, mna.n
            )));
        }
        if self.config.output_nodes.is_empty() {
            return Err(CodegenError::InvalidConfig(
                "output_nodes must not be empty".to_string(),
            ));
        }
        for (i, &node) in self.config.output_nodes.iter().enumerate() {
            if node >= mna.n {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={}",
                    i, node, mna.n
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
        let resolved = ir::refine_active_set_for_audio_path(
            ir::resolve_opamp_rail_mode(mna, self.config.opamp_rail_mode),
            mna,
            netlist,
        );
        let augmented_storage;
        let augmented_mna_storage;
        let (mna, netlist) = if resolved.mode == OpampRailMode::BoyleDiodes {
            log::info!(
                "Codegen nodal: BoyleDiodes mode active; augmenting netlist with \
                 internal-node catch diodes for {} clamped op-amps",
                mna.opamps
                    .iter()
                    .filter(|oa| oa.n_out_idx > 0 && (oa.vcc.is_finite() || oa.vee.is_finite()))
                    .count()
            );
            augmented_storage = ir::augment_netlist_with_boyle_diodes(netlist, mna);
            let mut rebuilt = MnaSystem::from_netlist(&augmented_storage).map_err(|e| {
                CodegenError::UnsupportedTopology(format!(
                    "BoyleDiodes augmented netlist failed to build MNA: {e}"
                ))
            })?;

            // Re-apply caller-side MNA mutations that the rebuild dropped.
            //
            // 1) Input conductance: the CLI stamps `g[in][in] += 1/R_in`
            //    into the original MNA before calling `generate_nodal`,
            //    but `from_netlist(&augmented_storage)` returns a clean
            //    MNA. Without this, the input node row only has the
            //    series cap conductance (~1e-4), the LU treats it as
            //    nearly singular, and the linear prediction blows up
            //    on the first non-zero input sample.
            if self.config.input_node < rebuilt.n {
                let g_in = 1.0 / self.config.input_resistance;
                rebuilt.g[self.config.input_node][self.config.input_node] += g_in;
            }

            // 2) Device junction caps: the CLI stamps these into the
            //    original MNA via `stamp_device_junction_caps`. The
            //    augmented netlist has new diodes (the catch diodes),
            //    so we need to rebuild `device_slots` on the augmented
            //    netlist + MNA before stamping.
            if let Ok(device_slots) =
                ir::CircuitIR::build_device_info_with_mna(&augmented_storage, Some(&rebuilt))
            {
                if !device_slots.is_empty() {
                    rebuilt.stamp_device_junction_caps(&device_slots);
                }
            }

            augmented_mna_storage = rebuilt;
            (&augmented_mna_storage, &augmented_storage)
        } else {
            (mna, netlist)
        };

        // Auto-insert parasitic caps if C matrix is all zeros and circuit has
        // nonlinear devices. Without capacitors, A = G and the trapezoidal
        // integrator degenerates (no energy storage → no dynamics).
        let mut patched_mna = None;
        let (mna, parasitic_caps_inserted) =
            maybe_insert_parasitic_caps(mna, &mut patched_mna, "Codegen nodal");

        let ir = CircuitIR::from_mna(mna, netlist, &self.config)?;
        let code = RustEmitter::new()?.emit(&ir)?;

        Ok(GeneratedCode {
            code,
            n: ir.topology.n,
            m: ir.topology.m,
            meta: build_codegen_meta(&ir, parasitic_caps_inserted),
        })
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
