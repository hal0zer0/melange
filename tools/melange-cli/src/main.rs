//! melange-cli - Command line tool for circuit modeling
//!
//! Usage:
//!   melange compile input.cir --output circuit.rs
//!   melange validate input.cir --output-node out
//!   melange simulate input.cir --input input.wav --output output.wav
//!   melange sources list
//!   melange builtins

// Doc comments use markdown lists whose continuations render fine.
#![allow(clippy::doc_lazy_continuation)]

mod builtins {
    // This module exists to include builtin circuit files
    // The actual content is embedded using include_str! in circuits.rs
}

pub mod cache;
pub mod circuits;
pub mod codegen_runner;
pub mod index_cmd;
pub mod plugin_template;
pub mod sources;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand, ValueEnum};
use melange_solver::build::format_system_size;
use std::path::{Path, PathBuf};

#[derive(Parser)]
#[command(name = "melange")]
#[command(about = "Circuit modeling toolkit - from SPICE to real-time DSP")]
// Version carries the build commit (`version_label`) so `melange --version`
// disambiguates a released tag, an unreleased main, and a local build that
// otherwise all print the same bare CARGO_PKG_VERSION.
#[command(version = version_label())]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

/// Help heading for flags that override a choice melange makes itself. Kept
/// out of the main list so `--help` leads with what a new user needs.
const EXPERT_HEADING: &str =
    "Solver overrides (melange chooses these; set them to pin or debug a route)";

#[derive(Subcommand)]
enum Commands {
    /// Compile a SPICE netlist to optimized Rust code
    Compile {
        /// Input SPICE netlist file or circuit reference
        /// (builtin:circuit, source:circuit, URL, or local path)
        input: String,

        /// Where to write the result. For `--format code` (the default) this is
        /// a FILE, e.g. `src/circuit.rs`. For `--format plugin` it is the
        /// DIRECTORY the generated Cargo project is created in, e.g.
        /// `my-pedal-plugin` — passing a file path there builds a project named
        /// after the file, nested wherever that file lives.
        #[arg(short, long)]
        output: PathBuf,

        /// Sample rate in Hz
        #[arg(short, long, default_value = "48000")]
        sample_rate: f64,

        /// Input node name
        #[arg(short, long, default_value = "in")]
        input_node: String,

        /// Output node name(s), comma-separated for multi-output (e.g., "out_l,out_r")
        #[arg(short = 'n', long, default_value = "out")]
        output_node: String,

        /// Maximum NR iterations
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "50")]
        max_iter: usize,

        /// Convergence tolerance
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "1e-9")]
        tolerance: f64,

        /// Output format
        #[arg(short = 'f', long, value_enum, default_value = "code")]
        format: OutputFormat,

        /// Output scale factor (default 1.0, use 0.1 to map ±10V to ±1.0 audio)
        #[arg(long, default_value = "1.0")]
        output_scale: f64,

        /// Post-DC-block output limiter ceiling in volts. The generated code emits
        /// `scaled.clamp(-V, V)` after DC blocking and `diag_clamp_count` increments
        /// above this threshold. Default 10.0 V preserves the historical "Signal Level
        /// Contract" (see docs/aidocs/SIGNAL_LEVELS.md). Raise for circuits with rails
        /// above ±10 V (e.g. a power amp at ±22 V rails needs 30 or higher). Ignored
        /// when DC blocking is disabled.
        #[arg(long, default_value = "10.0")]
        output_clamp: f64,

        /// Add Input Level and Output Level parameters to the plugin (default: true)
        #[arg(long, default_value = "true")]
        with_level_params: bool,

        /// Generate plugin without Input/Output Level parameters
        #[arg(long)]
        no_level_params: bool,

        /// Disable DC blocking filter on outputs. Use for circuits with output coupling
        /// caps or when downstream handles DC offset. Removes the 5Hz HPF settling time.
        #[arg(long)]
        no_dc_block: bool,

        /// Override input resistance (ohms). Default: 1Ω, or from .input_impedance directive.
        #[arg(long)]
        input_resistance: Option<f64>,

        /// Oversampling factor (1=none, 2=2x, 4=4x). Higher reduces aliasing and improves NR stability.
        /// Overrides the deck's `.oversampling` recommendation when set (even if lower); absent, the
        /// deck value is used, else 1.
        #[arg(long)]
        oversampling: Option<usize>,

        /// Solver type: auto (default), dk, nodal.
        /// Auto selects DK for most circuits, nodal for multi-transformer.
        /// Use nodal for large M circuits where DK NR doesn't converge.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        solver: String,

        /// Use backward Euler integration instead of trapezoidal.
        /// Unconditionally stable — fixes divergence in high-gain feedback amplifiers.
        /// Trades second-order accuracy for first-order (slight HF rolloff).
        #[arg(help_heading = EXPERT_HEADING, long)]
        backward_euler: bool,

        /// Force trapezoidal even when the nodal auto-detector would promote
        /// to backward Euler (trap propagation operator spectral_radius >
        /// 1.002 — persistent Nyquist-rate limit cycle in v_prev). Escape
        /// hatch for bisecting regressions or reproducing legacy output.
        /// Ignored when `--backward-euler` is already set.
        #[arg(help_heading = EXPERT_HEADING, long)]
        force_trap: bool,

        /// Pentode grid-off dimension reduction mode.
        ///
        /// When a pentode's grid is biased well below cutoff at DC-OP, the
        /// Ig1 NR dimension can be dropped and Vg2k frozen, reducing M by 1
        /// per grid-off tube. This enables DK Schur for circuits that would
        /// otherwise exceed the M=16 cap (e.g. 4×EL34 Plexi: M=18 → M=14).
        ///
        /// Valid values:{n}{n}
        /// * auto (default) — reserved for reductions that are provably
        ///   neutral; none exists today, so auto currently keeps the full 3D
        ///   model (== off).{n}{n}
        /// * on — reduce every non-variable-mu pentode to 2D (Vg2k frozen at
        ///   its DC value, Ig1 dropped). NOT accuracy-neutral: drops the
        ///   cathode/screen-referenced Vg2k feedback (measured +2% to +12%
        ///   small-signal gain error on cathode-biased stages) and all grid
        ///   current for Vgk > 0. Warns per device. Opt-in only.{n}{n}
        /// * off — never reduce; all pentodes keep their full 3D NR block.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        tube_grid_fa: String,

        /// Sub-sample fire: variable-dt breakpoint re-solve at a glow-discharge
        /// strike (nodal-Schur route only).
        ///
        /// A latched device flips AFTER the sample's solve, so conduction
        /// starts one inner sample late and the strike instant is quantised to
        /// the sample grid; on injection-locked neon divider chains that breaks
        /// lock at plugin rates. The re-solve splits the firing sample at the
        /// device's crossing fraction into a dark and a lit (BE) sub-step, each
        /// on matrices rebuilt at the sub-step rate. Stage A prototype: not
        /// real-time optimised (two O(N^3) inversions per firing sample).
        ///
        /// Valid values:{n}{n}
        /// * auto (default) — on when the circuit has a glow device AND routes
        ///   to nodal-Schur; otherwise inert (byte-identical output).{n}{n}
        /// * on — force; refused on the DK route and on the nodal full-LU
        ///   sub-path.{n}{n}
        /// * off — whole-sample latch flip (pre-feature behaviour).
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        subsample_fire: String,

        /// Diagnostic: lit sub-step multiplier (`factor * tau`) for the glow
        /// variable-dt re-solve. Unset → 1.0, the shipped last tested-safe point.
        /// A bisection tool in the family of `--force-trap` /
        /// `--nodal-subpath` — NOT a per-deck tuning knob (a deck author cannot
        /// honestly tune it without a lock-margin sweep). Recorded in the
        /// provenance Build: line and JSON.
        #[arg(help_heading = EXPERT_HEADING, long)]
        subsample_lit_factor: Option<f64>,

        /// BJT forward-active (frozen-analysis) reduction mode.
        ///
        /// * off — never reduce; all BJTs keep their full 2-D NR block. Default.{n}{n}
        /// * auto — reduce pure-Ebers-Moll BJTs found forward-active at the DC
        ///   operating point (Gummel-Poon / ISE / self-heating / parasitic BJTs
        ///   stay full-2-D). Exact only while a reduced BJT stays forward-active:
        ///   a sample on which one saturates is counted unsolved and refused.{n}{n}
        /// * force — also 1-D-reduce Gummel-Poon / ISE / parasitic BJTs, each
        ///   with a per-device WARNING. Drops the qb base-charge term (Early +
        ///   high-level injection); NOT accuracy-safe under signal (~1-2 dB
        ///   under hard drive, larger for parasitics). Self-heating BJTs are
        ///   never force-reduced (structural).
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "off")]
        bjt_fa: String,

        /// Op-amp supply rail saturation strategy.
        ///
        /// Controls how the generated solver models an op-amp's output hitting
        /// its supply rails. Different circuits need different trade-offs
        /// between numerical correctness, harmonic accuracy, and runtime cost.
        ///
        /// Valid values:{n}{n}
        /// * auto — inspect the circuit and pick the cheapest correct mode.
        ///   Logged at compile time so you can see what was chosen. Override
        ///   when bisecting issues.{n}{n}
        /// * none — no clamping at all (op-amp output is unbounded). Use only
        ///   for verified-linear circuits.{n}{n}
        /// * hard — post-NR v[out].clamp(VEE, VCC). Cheapest, matches
        ///   pre-2026-04 behavior. Breaks KCL for AC-coupled downstream caps
        ///   (see Klon investigation).{n}{n}
        /// * active-set — post-NR constrained re-solve. KCL-consistent hard
        ///   clip. Fixes Klon-class cap-history corruption. Still produces
        ///   square-wave harmonics.{n}{n}
        /// * active-set-be — active-set, plus a backward-Euler step on any
        ///   sample where the clamp engages. L-stable across the clamp
        ///   transition, so it does not ring the way trapezoidal does when a
        ///   rail is hit. Aliases: active_set_be, activesetbe, be-on-clamp.{n}{n}
        /// * boyle-diodes — auto-inserted physical catch diodes anchored to
        ///   rail-offset voltage sources. Matches commercial SPICE Boyle
        ///   macromodels. Produces soft exponential knee — best for
        ///   distortion pedals.
        #[arg(help_heading = EXPERT_HEADING, long, value_name = "MODE", default_value = "auto")]
        opamp_rail_mode: String,

        /// Which nodal sub-path to emit: auto (default), schur, full-lu.
        ///
        /// The nodal solver has TWO Newton implementations of the same circuit:
        /// `schur` predicts through S = A^-1 and iterates only the M coupled
        /// device dimensions; `full-lu` factors the whole augmented N x N system
        /// every iteration. `auto` picks from measured conditioning and is the
        /// shipping behaviour — leave it alone for production builds.
        ///
        /// The forcing modes are DIAGNOSTIC escape hatches, like --force-trap:
        /// they exist so the sub-path can be isolated as a variable (A/B the two
        /// implementations on one netlist, or reproduce a build from before a
        /// routing decision moved). They warn when they contradict the auto
        /// choice. `schur` is REFUSED outright on circuits that structurally
        /// require full-LU — saturating inductors and behavioral
        /// B-sources cannot be expressed by the Schur reduction, and forcing it
        /// would silently drop the nonlinearity. Ignored for DK-routed circuits.
        #[arg(help_heading = EXPERT_HEADING, long, value_name = "MODE", default_value = "auto")]
        nodal_subpath: String,

        /// Escape hatch: allow a relaxing-section / delayed-
        /// overvoltage / KSUB glow to compile on the nodal full-LU sub-path,
        /// which otherwise REFUSES. On full-LU the lit branch is the static line
        /// (v0+RS·i) while the strike seed/extinction read the section model — a
        /// mixed, silently-wrong model. With this flag the section keys go INERT
        /// on full-LU and provenance records `glow_sections: inert (full-lu)`.
        /// Diagnostic only; the Schur route honors the section model.
        #[arg(help_heading = EXPERT_HEADING, long, default_value_t = false)]
        allow_static_glow_on_full_lu: bool,

        /// Authentic circuit noise mode: off (default), thermal, shot, full.
        /// `thermal` emits Johnson-Nyquist noise on every resistor; `shot` adds
        /// junction shot; `full` adds 1/f flicker, pentode partition, and op-amp
        /// en/in. Off by default and byte-identical to a noiseless build when off.
        /// See docs/aidocs/NOISE.md. Runtime toggle via `set_noise_enabled(bool)`.
        #[arg(long, value_name = "MODE", default_value = "off")]
        noise: String,

        /// Master noise seed (u64). `0` → entropy from system clock at plugin
        /// init. Nonzero → deterministic noise (same seed → bit-identical output).
        #[arg(long, value_name = "SEED", default_value = "0")]
        noise_seed: u64,
        /// Build even when the DC operating point did not converge. By default
        /// the build is refused: its generated code would start from a state that
        /// is not a solution and slew or ring away from it. For deliberate use
        /// only; the build still warns.
        #[arg(help_heading = EXPERT_HEADING, long)]
        allow_unconverged_dc_op: bool,
        /// Test-only: the DC operating point's Newton budget per solve
        /// attempt, for the unconverged-DC-OP refusal's witness. Hidden.
        #[arg(long, hide = true)]
        dc_op_max_iterations: Option<usize>,

        /// Emit `CircuitState::recompute_dc_op()` for runtime DC operating
        /// point re-solve after pot/switch changes.
        ///
        /// Default OFF: generated code is byte-identical to pre-Phase-E output.
        /// When ON, plugins with per-instance component jitter can call
        /// `state.recompute_dc_op()` to jump to the jittered equilibrium
        /// without a warmup loop. See docs/aidocs/DC_OP.md for MVP scope
        /// (Direct-NR only, no basin-trap handling).
        #[arg(long)]
        emit_dc_op_recompute: bool,

        /// Plugin display name (defaults to capitalized circuit filename)
        #[arg(long)]
        name: Option<String>,

        /// Generate mono (1-channel) plugin instead of stereo
        #[arg(long)]
        mono: bool,

        /// Add wet/dry mix parameter to generated plugin
        #[arg(long)]
        wet_dry_mix: bool,

        /// Disable ear-protection output limiter. The limiter is a transparent soft
        /// clipper that engages near 0 dBFS to protect speakers and hearing.
        /// On by default — use this flag only for measurement or testing.
        #[arg(long)]
        no_ear_protection: bool,

        /// Plugin vendor name shown to the DAW (e.g., "Acme Audio").
        /// Defaults to "Melange" when omitted. Applies to `--format plugin`.
        #[arg(long, value_name = "STR")]
        vendor: Option<String>,

        /// Plugin vendor homepage URL (must start with http:// or https://).
        /// Defaults to "https://github.com/hal0zer0/melange". Applies to `--format plugin`.
        #[arg(long, value_name = "URL")]
        vendor_url: Option<String>,

        /// Plugin vendor contact email (e.g., "support@acme.example").
        /// Empty by default — a generated plugin should not name someone who
        /// has not agreed to answer its support mail. Applies to `--format plugin`.
        #[arg(long, value_name = "ADDR")]
        email: Option<String>,

        /// Override the VST3 class ID with exactly 16 printable ASCII bytes.
        /// Without this flag the ID is derived from the circuit filename, so
        /// renaming the .cir file breaks DAW sessions. Pin an explicit value
        /// for stable releases. Applies to `--format plugin`.
        #[arg(long, value_name = "16-CHAR")]
        vst3_id: Option<String>,

        /// Override the CLAP plugin ID (reverse-DNS style, e.g.,
        /// "com.acme.wurli"). Defaults to "com.melange.<circuit>" when omitted.
        /// Applies to `--format plugin`.
        #[arg(long, value_name = "STR")]
        clap_id: Option<String>,

        /// x86_64 instruction-set baseline for the plugin project
        /// (`--format plugin`). x86-64-v3 (default): AVX2, Haswell 2013+,
        /// fastest. x86-64-v2: SSE4.2, 2008+. x86-64: runs on every x86_64 CPU.
        /// A build above the machine's baseline crashes when the DAW loads it;
        /// results are bit-identical across all three. aarch64 is unaffected.
        #[arg(long, value_enum, value_name = "BASELINE", default_value = "x86-64-v3")]
        cpu_baseline: plugin_template::CpuBaseline,
    },

    /// Validate circuit against ngspice reference simulation
    ///
    /// Runs the melange solver and ngspice on the same circuit with a test input
    /// signal, then compares the outputs. Requires ngspice to be installed.
    Validate {
        /// Input SPICE netlist file or circuit reference
        /// (builtin:circuit, source:circuit, URL, or local path)
        input: String,

        /// Output node name to compare
        #[arg(short = 'n', long, default_value = "out")]
        output_node: String,

        /// Sample rate in Hz
        #[arg(short, long, default_value = "48000")]
        sample_rate: f64,

        /// Test signal duration in seconds
        #[arg(long, default_value = "1.0")]
        duration: f64,

        /// Test signal amplitude (volts)
        #[arg(long, default_value = "0.1")]
        amplitude: f64,

        /// Input node name (for informational display)
        #[arg(short = 'i', short_alias = 'I', long, default_value = "in")]
        input_node: String,

        /// Write comparison data to CSV file
        #[arg(long)]
        csv: Option<PathBuf>,

        /// Use relaxed tolerances (1% RMS, 0.999 correlation)
        #[arg(long)]
        relaxed: bool,

        /// Override RMS-error tolerance, in percent (e.g. 2 = 2%). Overrides the
        /// --relaxed/strict profile value for a node-class gate.
        #[arg(long, value_name = "PCT")]
        rms_tolerance: Option<f64>,

        /// Override peak-error tolerance, in volts (absolute). Without it the
        /// default profile's bound is 1 % of the reference's peak (floor 1 µV).
        #[arg(long, value_name = "V")]
        peak_tolerance: Option<f64>,

        /// Override max-relative-error tolerance, in percent (e.g. 5 = 5%).
        #[arg(long, value_name = "PCT")]
        max_rel_tolerance: Option<f64>,

        /// Override minimum correlation coefficient (0.0–1.0).
        #[arg(long, value_name = "X")]
        corr_min: Option<f64>,

        /// Override THD-error tolerance, in dB.
        #[arg(long, value_name = "DB")]
        thd_tolerance: Option<f64>,

        /// Forward-active BJT reduction: off (default), auto, force.
        ///
        /// Same mechanism as `melange compile --bjt-fa`.
        #[arg(long, default_value = "off")]
        bjt_fa: String,

        /// Grid-off pentode reduction: auto (default), on, off.
        ///
        /// Same mechanism as `melange compile --tube-grid-fa`. auto is
        /// reserved for reductions that are provably neutral; none exists
        /// today, so auto currently keeps the full 3D model (== off). `on`
        /// is the warned opt-in — use it to attribute a residual to the
        /// reduction.
        #[arg(long, default_value = "auto")]
        tube_grid_fa: String,

        /// Force Backward Euler on the melange side (diagnostic; mirrors
        /// `compile --backward-euler`). Attributes integrator error; the
        /// default keeps the shipped auto selection.
        #[arg(long)]
        backward_euler: bool,

        /// Force trapezoidal on the melange side (diagnostic; mirrors
        /// `compile --force-trap`). Ignored when --backward-euler is set.
        #[arg(long)]
        force_trap: bool,

        /// Oversampling factor (1=none, 2=2x, 4=4x) for the melange side —
        /// mirrors `melange compile --oversampling`.
        ///
        /// NOT a diagnostic. `--oversampling` is compile-time codegen: a build
        /// at 2x is different DSP (interpolator, solver at the internal rate,
        /// polyphase half-band decimator). Without this flag you would validate
        /// the 1x code and ship the 2x code.
        ///
        /// ngspice is untouched — it has its own timestep and knows nothing
        /// about melange's internal rate — and it is NOT filtered. The
        /// comparison is against the circuit: an unfiltered reference aligned
        /// to the melange output by one best-fit constant delay, the same
        /// alignment every mode gets. The half-bands' frequency-dependent phase
        /// therefore stays inside the number. No tolerance moves.
        ///
        /// Unlike `compile`, this does NOT read the deck's `.oversampling`
        /// recommendation: validate reports what it was asked to measure.
        #[arg(long, default_value = "1", value_name = "N")]
        oversampling: usize,

        /// Validate at the sample rate, twice and four times it (oversampling
        /// off) and classify the deck: PASS at the rate; CONVERGES (the
        /// error falls with the step toward ngspice: melange models the
        /// circuit and the rate is the cost; reports the asymptotic model
        /// error and the rate needed for 1 % and 0.1 %); PLATEAU or DIVERGES
        /// (the error stops falling or rises: a model or harness mismatch).
        #[arg(long)]
        rate_sweep: bool,
    },

    /// Simulate circuit with input signal
    Simulate {
        /// Input SPICE netlist file or circuit reference
        input: String,

        /// Input audio file (WAV). If omitted, generates a 1kHz sine wave.
        #[arg(short = 'a', long)]
        input_audio: Option<PathBuf>,

        /// Output audio file (WAV)
        #[arg(short, long)]
        output: PathBuf,

        /// Sample rate in Hz (used when no input audio provided)
        #[arg(short, long, default_value = "48000")]
        sample_rate: f64,

        /// Input node name
        #[arg(short = 'i', short_alias = 'I', long, default_value = "in")]
        input_node: String,

        /// Output node name
        #[arg(short = 'n', long, default_value = "out")]
        output_node: String,

        /// Duration in seconds (used when no input audio provided)
        #[arg(short, long, default_value = "1.0")]
        duration: f64,

        /// Test-tone amplitude in volts (peak) at the input node. Ignored with
        /// --input-audio, which is fed in as-is (full scale = 1 V)
        #[arg(long, default_value = "0.5")]
        amplitude: f64,

        /// Override input resistance (ohms). Default: 1Ω, or from .input_impedance directive.
        #[arg(long)]
        input_resistance: Option<f64>,

        /// Solver type: auto (default), dk (DK method), nodal (full-nodal NR).
        /// Auto selects nodal for nonlinear circuits with inductors, dk otherwise.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        solver: String,

        /// Nodal Schur-vs-full-LU sub-path: auto, schur, full-lu. Mirrors
        /// `compile --nodal-subpath`. A measuring verb must be able to pin its
        /// route — the auto route is per (deck, sample rate), so a measurement
        /// that cannot force the sub-path cannot reproduce or isolate it.
        #[arg(help_heading = EXPERT_HEADING, long, value_name = "MODE", default_value = "auto")]
        nodal_subpath: String,

        /// Op-amp rail saturation mode: auto, none, hard, active-set, active-set-be, boyle-diodes.
        /// Default 'auto' inspects the topology and picks the cheapest correct mode.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        opamp_rail_mode: String,

        /// Pentode grid-off dimension reduction mode: auto, on, off.
        /// `on` reduces every non-variable-mu pentode 3D→2D (Vg2k frozen, Ig1
        /// dropped; NOT accuracy-neutral, warned per device); `off` keeps full
        /// 3D blocks. auto is reserved for reductions that are provably
        /// neutral; none exists today, so auto currently keeps the full 3D
        /// model (== off). Mirrors `compile --tube-grid-fa`.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        tube_grid_fa: String,

        /// Sub-sample fire (glow-strike variable-dt re-solve): auto, on, off.
        /// Mirrors `compile --subsample-fire`. auto = on for glow decks on
        /// nodal-Schur, inert otherwise; on is refused on DK / full-LU.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        subsample_fire: String,

        /// Oversampling factor (1=none, 2=2x, 4=4x). Higher reduces aliasing
        /// and improves NR stability for circuits with diode switching.
        /// Overrides the deck's `.oversampling` recommendation when set (even
        /// if lower); absent, the deck value is used, else 1.
        #[arg(long)]
        oversampling: Option<usize>,

        /// Authentic circuit noise mode: off (default), thermal, shot, full.
        #[arg(long, value_name = "MODE", default_value = "off")]
        noise: String,

        /// Master noise seed (u64). `0` → entropy from system clock; nonzero → deterministic.
        #[arg(long, value_name = "SEED", default_value = "0")]
        noise_seed: u64,
        /// Build even when the DC operating point did not converge. By default
        /// the build is refused: its generated code would start from a state that
        /// is not a solution and slew or ring away from it. For deliberate use
        /// only; the build still warns.
        #[arg(help_heading = EXPERT_HEADING, long)]
        allow_unconverged_dc_op: bool,
        /// Test-only: the DC operating point's Newton budget per solve
        /// attempt, for the unconverged-DC-OP refusal's witness. Hidden.
        #[arg(long, hide = true)]
        dc_op_max_iterations: Option<usize>,

        /// Use backward Euler integration instead of trapezoidal.
        /// Unconditionally stable — fixes divergence in high-gain feedback
        /// amplifiers. Mirrors `compile --backward-euler`.
        #[arg(help_heading = EXPERT_HEADING, long)]
        backward_euler: bool,

        /// Force trapezoidal even when the nodal auto-detector would promote
        /// to backward Euler. Escape hatch for bisecting regressions.
        /// Ignored when `--backward-euler` is already set. Mirrors
        /// `compile --force-trap`.
        #[arg(help_heading = EXPERT_HEADING, long)]
        force_trap: bool,

        /// Maximum NR iterations per sample. Defaults to the same auto-tuned
        /// budget `compile` uses (scales with M, solver route, and trap
        /// spectral radius).
        #[arg(help_heading = EXPERT_HEADING, long)]
        max_iter: Option<usize>,

        /// Render even if samples were never solved.
        ///
        /// When every Newton path fails on a sample, the solver commits the
        /// PREVIOUS state as that sample's output. It is not a solution, and it
        /// is bounded and smooth, so the peak and the waveform both look
        /// healthy. Under constant input it is also a fixed point: the next
        /// sample re-poses the identical problem and fails the same way, so one
        /// hard sample can freeze the render to its end. `simulate` fails on
        /// this by default. Per-invocation only — a netlist cannot declare it,
        /// because whether a circuit trips depends on the input level, not the
        /// netlist.
        #[arg(help_heading = EXPERT_HEADING, long)]
        allow_nr_hold: bool,

        /// Render even when the circuit was not driven with the requested
        /// input: samples beyond the generated code's input limit
        /// (`INPUT_LIMIT_V`, 100 V) are clamped to it, and NaN/Inf samples are
        /// replaced by 0. Both are counted, and the command fails on either by
        /// default, because the output then answers a different question.
        #[arg(help_heading = EXPERT_HEADING, long)]
        allow_input_clamp: bool,

        /// Probe an internal node. May be repeated. Probe samples are written
        /// to a sidecar CSV (one column per probe) alongside the WAV; the
        /// primary `-n/--output-node` signal goes to the WAV unchanged.
        /// With --oversampling > 1, each row is one HOST-rate sample taken after
        /// the decimation filter, not a raw internal-rate value: on a node with
        /// sharp edges the filter's overshoot can exceed the raw peak, so do not
        /// gate peaks on probe rows at OS > 1.
        #[arg(long = "probe", value_name = "NODE")]
        probes: Vec<String>,

        /// Where to write the probe CSV. Defaults to the output name with
        /// `.wav` swapped for `.probes.csv` (`out.wav` -> `out.probes.csv`);
        /// ignored when no `--probe` is given.
        #[arg(long = "probe-csv", value_name = "PATH")]
        probe_csv: Option<PathBuf>,

        /// Write the output WAV as 16-bit signed PCM instead of the default
        /// IEEE float32. Float32 (WAVE format 3) carries melange's full range
        /// — the WAV holds VOLTS, which routinely exceed ±1 — but several
        /// common readers refuse it outright (Python's stdlib `wave` raises
        /// "unknown format: 3"). PCM16 cannot represent |v| > 1 V: those
        /// samples are CLAMPED, and the clamped count is reported.
        #[arg(long)]
        pcm16: bool,

        /// Set pot value: "Label=value" or "Rname=value" (e.g. "Drive=100k").
        /// May be repeated. Mirrors `analyze --pot`: the value is a RESISTANCE
        /// in ohms (engineering suffixes accepted) and is range-checked against
        /// the pot's declared `.pot` min..max — an out-of-range setting is
        /// refused, not clamped. For a `.wiper` label the value is a POSITION
        /// in 0.0..=1.0 instead. `melange nodes <circuit>` lists every control,
        /// its range and its default.
        #[arg(long = "pot", value_name = "NAME=VALUE")]
        pot_overrides: Vec<String>,

        /// Set switch position: "Label=pos" or index=pos (0-indexed, e.g.
        /// "Voice=3"). May be repeated. Applied at runtime via
        /// `state.set_switch_N(pos)` before the run — mirrors the generated
        /// plugin, so `simulate` reaches non-rest positions the plugin uses.
        #[arg(long = "switch", value_name = "NAME=POS")]
        switch_overrides: Vec<String>,

        /// Drive a `.inject` field: `--inject FIELD=sine:<freq_hz>:<amp_volts>`
        /// or `FIELD=dc:<volts>`. The value is CIRCUIT VOLTS injected at the
        /// `.inject` node through its declared impedance. May be repeated;
        /// `.inject` fields with no `--inject` default to 0 (undriven).
        #[arg(long = "inject", value_name = "FIELD=SPEC")]
        inject_drives: Vec<String>,
    },

    /// Analyze circuit frequency response
    Analyze {
        /// Input SPICE netlist file or circuit reference
        input: String,

        /// Input node name
        #[arg(short = 'i', short_alias = 'I', long, default_value = "in")]
        input_node: String,

        /// Output node name
        #[arg(short = 'n', long, default_value = "out")]
        output_node: String,

        /// Start frequency in Hz
        #[arg(long, default_value = "20.0")]
        start_freq: f64,

        /// End frequency in Hz
        #[arg(long, default_value = "20000.0")]
        end_freq: f64,

        /// Frequency points per decade
        #[arg(long, default_value = "10")]
        points_per_decade: usize,

        /// Input signal amplitude in volts (peak), applied to the input node
        #[arg(long, default_value = "0.1")]
        amplitude: f64,

        /// Sample rate in Hz
        #[arg(short, long, default_value = "96000")]
        sample_rate: f64,

        /// Override input resistance (ohms)
        #[arg(long)]
        input_resistance: Option<f64>,

        /// Write CSV output to file instead of stdout
        #[arg(short, long)]
        output: Option<PathBuf>,

        /// Set pot value: "Label=value" or "Rname=value" (e.g. "LF Boost=10k")
        #[arg(long = "pot", value_name = "NAME=VALUE")]
        pot_overrides: Vec<String>,

        /// Set switch position: "Label=pos" or index=pos (0-indexed, e.g. "LF Freq=3")
        #[arg(long = "switch", value_name = "NAME=POS")]
        switch_overrides: Vec<String>,

        /// Measure up to N harmonics per frequency point (0 = fundamental only,
        /// the legacy CSV). When >0, appends `thd_pct, h2_dbc, ..., hN_dbc`
        /// columns. Harmonics above Nyquist are reported as `nan`.
        #[arg(long, default_value = "0")]
        harmonics: usize,

        /// Pentode grid-off dimension reduction mode: auto, on, off.
        /// Mirrors `compile --tube-grid-fa`. See `simulate --tube-grid-fa` for
        /// the auto/on/off semantics. Defaults to `auto`, which is reserved
        /// for provably-neutral reductions and currently keeps the full 3D
        /// model (== off).
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        tube_grid_fa: String,

        /// Solver type: auto (default), dk (DK method), nodal (full-nodal NR).
        /// Mirrors `compile --solver`.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        solver: String,

        /// Oversampling factor (1=none, 2=2x, 4=4x). Mirrors
        /// `compile --oversampling` so the analyzed response matches the
        /// generated plugin. Overrides the deck's `.oversampling`
        /// recommendation when set (even if lower); absent, the deck value is
        /// used, else 1.
        #[arg(long)]
        oversampling: Option<usize>,

        /// Op-amp rail saturation mode: auto, none, hard, active-set,
        /// active-set-be, boyle-diodes. Mirrors `compile --opamp-rail-mode`.
        #[arg(help_heading = EXPERT_HEADING, long, value_name = "MODE", default_value = "auto")]
        opamp_rail_mode: String,

        /// Which nodal sub-path to emit: auto (default), schur, full-lu.
        ///
        /// The nodal solver has TWO Newton implementations of the same circuit:
        /// `schur` predicts through S = A^-1 and iterates only the M coupled
        /// device dimensions; `full-lu` factors the whole augmented N x N system
        /// every iteration. `auto` picks from measured conditioning and is the
        /// shipping behaviour — leave it alone for production builds.
        ///
        /// The forcing modes are DIAGNOSTIC escape hatches, like --force-trap:
        /// they exist so the sub-path can be isolated as a variable (A/B the two
        /// implementations on one netlist, or reproduce a build from before a
        /// routing decision moved). They warn when they contradict the auto
        /// choice. `schur` is REFUSED outright on circuits that structurally
        /// require full-LU — saturating inductors and behavioral
        /// B-sources cannot be expressed by the Schur reduction, and forcing it
        /// would silently drop the nonlinearity. Ignored for DK-routed circuits.
        #[arg(help_heading = EXPERT_HEADING, long, value_name = "MODE", default_value = "auto")]
        nodal_subpath: String,

        /// Authentic circuit noise mode: off (default), thermal, shot, full.
        /// Mirrors `compile --noise`.
        #[arg(long, value_name = "MODE", default_value = "off")]
        noise: String,

        /// Master noise seed (u64). `0` → entropy from system clock; nonzero → deterministic.
        /// Mirrors `compile --noise-seed`.
        #[arg(long, value_name = "SEED", default_value = "0")]
        noise_seed: u64,
        /// Build even when the DC operating point did not converge. By default
        /// the build is refused: its generated code would start from a state that
        /// is not a solution and slew or ring away from it. For deliberate use
        /// only; the build still warns.
        #[arg(help_heading = EXPERT_HEADING, long)]
        allow_unconverged_dc_op: bool,
        /// Test-only: the DC operating point's Newton budget per solve
        /// attempt, for the unconverged-DC-OP refusal's witness. Hidden.
        #[arg(long, hide = true)]
        dc_op_max_iterations: Option<usize>,

        /// Use backward Euler integration instead of trapezoidal.
        /// Mirrors `compile --backward-euler`.
        #[arg(help_heading = EXPERT_HEADING, long)]
        backward_euler: bool,

        /// Force trapezoidal even when auto-BE would fire. Ignored when
        /// `--backward-euler` is already set. Mirrors `compile --force-trap`.
        #[arg(help_heading = EXPERT_HEADING, long)]
        force_trap: bool,

        /// Maximum NR iterations per sample. Defaults to the same auto-tuned
        /// budget `compile` uses (scales with M, solver route, and trap
        /// spectral radius).
        #[arg(help_heading = EXPERT_HEADING, long)]
        max_iter: Option<usize>,

        /// Render even when the circuit was not driven with the requested
        /// input: samples beyond the generated code's input limit
        /// (`INPUT_LIMIT_V`, 100 V) are clamped to it, and NaN/Inf samples are
        /// replaced by 0. Both are counted, and the command fails on either by
        /// default, because the output then answers a different question.
        #[arg(help_heading = EXPERT_HEADING, long)]
        allow_input_clamp: bool,
    },

    /// Compute DC operating point and print node voltages
    ///
    /// The operating point `compile` ships with the same flags: the circuit is
    /// built exactly as `compile` builds it (ports, `.inject`, reductions,
    /// route) and the vector reported is the one its generated code embeds as
    /// `DC_OP`.
    DcOp {
        /// Input SPICE netlist file or circuit reference
        input: String,

        /// Input node name. As in `compile`: the input port's source
        /// conductance is part of the circuit the operating point is solved
        /// for, and a deck without the node is refused.
        #[arg(short, long, default_value = "in")]
        input_node: String,

        /// Override input resistance (ohms). Default: 1Ω, or from
        /// .input_impedance directive. An explicit flag beats the directive,
        /// matching compile/simulate/analyze.
        #[arg(long)]
        input_resistance: Option<f64>,

        /// Output format: "human" (default) or "json"
        #[arg(short = 'f', long, default_value = "human")]
        format: String,

        /// Sample rate in Hz, as for `compile` (the route can depend on it).
        #[arg(short, long, default_value = "48000")]
        sample_rate: f64,

        /// Oversampling factor, as for `compile` (the route can depend on it).
        #[arg(long)]
        oversampling: Option<usize>,

        /// Solver: auto (default), dk, nodal — as for `compile`.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        solver: String,

        /// Op-amp rail mode, as for `compile`.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        opamp_rail_mode: String,

        /// Forward-active BJT reduction, as for `compile` (auto|off|force).
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "off")]
        bjt_fa: String,

        /// Grid-off pentode reduction, as for `compile` (auto|on|off).
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        tube_grid_fa: String,

        /// Set a pot: "Label=value" or "Rname=value", as for `simulate` and
        /// `analyze`. May be repeated.
        #[arg(long = "pot", value_name = "NAME=VALUE")]
        pot_overrides: Vec<String>,

        /// Report the operating point even when it did not converge (every
        /// build refuses that by default).
        #[arg(help_heading = EXPERT_HEADING, long)]
        allow_unconverged_dc_op: bool,
        /// Test-only: the DC operating point's Newton budget per solve
        /// attempt, for the unconverged-DC-OP refusal's witness. Hidden.
        #[arg(long, hide = true)]
        dc_op_max_iterations: Option<usize>,
    },

    /// List available nodes in a netlist
    Nodes {
        /// Input SPICE netlist file or circuit reference
        input: String,
    },

    /// Generate or check a circuit repository's `circuits-index.json`.
    ///
    /// Publishing one lets anyone resolve your circuits by SHORT NAME
    /// (`yourrepo:big-muff`) instead of by path, which means you can reorganise
    /// — promote a deck between tiers, rename a directory — without breaking
    /// every consumer. Format spec: docs/CIRCUIT_INDEX.md.
    Index {
        /// Repository root to scan (default: current directory).
        #[arg(default_value = ".")]
        dir: PathBuf,

        /// Verify instead of writing: non-zero exit if the index is missing or
        /// no longer matches the tree. This is the CI form — a silently stale
        /// index is worse than none, because consumers trust it.
        #[arg(long)]
        check: bool,
    },

    /// Manage circuit sources (friendly source:circuit references)
    Sources {
        #[command(subcommand)]
        action: SourceAction,
    },

    /// List available builtin circuits
    Builtins,

    /// Manage the circuit cache
    Cache {
        #[command(subcommand)]
        action: CacheAction,
    },

    /// Import a KiCad netlist to Melange .cir format
    ///
    /// Supports KiCad XML intermediate netlist (full fidelity, preserves Melange.*
    /// custom fields), KiCad SPICE netlist (best-effort, standard components only),
    /// and .kicad_sch schematics (requires kicad-cli). Format is auto-detected.
    Import {
        /// Input file (KiCad XML .xml, SPICE .cir/.spice, or .kicad_sch schematic)
        input: PathBuf,

        /// Output Melange .cir file
        #[arg(short, long)]
        output: PathBuf,

        /// Input format override (auto-detected by default)
        #[arg(long, value_enum, default_value = "auto")]
        format: ImportFormat,

        /// Input is a .kicad_sch schematic file (shells out to kicad-cli)
        #[arg(long)]
        from_schematic: bool,
    },
}

#[derive(Subcommand)]
enum SourceAction {
    /// List configured sources
    List,

    /// Add a new source
    Add {
        /// Source name
        name: String,
        /// Where the circuits live: a local directory, or an http(s) base URL
        /// that serves raw files (on GitLab, `https://gitlab.com/<group>/<repo>/-/raw/main`)
        url: String,
        /// License identifier (optional)
        #[arg(short, long)]
        license: Option<String>,
        /// Attribution string (optional)
        #[arg(short, long)]
        attribution: Option<String>,
    },

    /// Remove a source
    Remove {
        /// Source name to remove
        name: String,
    },

    /// Show details for a source, and list the circuits its index publishes
    Show {
        /// Source name
        name: String,
    },
}

#[derive(Subcommand)]
enum CacheAction {
    /// Show cache contents
    List,

    /// Clear all cached files
    Clear,

    /// Show cache statistics
    Stats,
}

#[derive(ValueEnum, Clone, Debug, PartialEq)]
enum OutputFormat {
    /// Generate only the circuit code (default)
    // `rust` is accepted because the tool invites it: the generated output IS
    // Rust, the docs call it "Rust code", and `--format rust` was a first
    // user's first guess. Rejecting a guess your own wording produces is a
    // papercut with no upside.
    #[value(alias = "rust")]
    Code,
    /// Generate a complete plugin project
    #[value(alias = "project")]
    Plugin,
}

#[derive(ValueEnum, Clone, Debug)]
enum ImportFormat {
    /// Auto-detect from file content
    Auto,
    /// KiCad XML intermediate netlist (full fidelity)
    Xml,
    /// KiCad SPICE netlist (best-effort)
    Spice,
}

mod kicad_import;

/// `<crate version> (<commit>[-dirty])`: the commit label is the solver's
/// `build_identity::GIT_COMMIT`, the one the generated-code header carries, so
/// the two cannot disagree.
fn version_label() -> &'static str {
    static LABEL: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    LABEL.get_or_init(|| {
        format!(
            "{} ({})",
            env!("CARGO_PKG_VERSION"),
            melange_solver::build_identity::GIT_COMMIT
        )
    })
}

/// `--version` text: the clap static `<version> (<commit>[-dirty])` pointer plus
/// the exact runtime exe hash (`exe <16-hex FNV-1a-64>`). Matches the digest a
/// peer computes over the binary on disk, so a build can be identified from its
/// own output alone.
fn full_version_string() -> String {
    let base = version_label();
    match melange_solver::build_identity::current_exe_hash() {
        // Algorithm-qualified: `fnv1a64:` states the digest so a reader cannot
        // compare it against a different hash of the same file.
        Some(hash) => format!("melange {base} exe fnv1a64:{hash}"),
        None => format!("melange {base}"),
    }
}

fn main() -> Result<()> {
    // Exit quietly when the output pipe is closed early (e.g. `melange … | head`).
    // Rust ignores SIGPIPE by default, so a write to a closed stdout makes the
    // print machinery panic with "failed printing to stdout: Broken pipe"
    // (exit 101), which looks like a crash. Swallow exactly that panic and exit
    // cleanly; any other panic falls through to the default handler. A panic
    // hook is the std-only way to do this without pulling in a libc dependency.
    let default_panic_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let broken_pipe = info
            .payload()
            .downcast_ref::<String>()
            .is_some_and(|s| s.contains("Broken pipe"));
        if broken_pipe {
            std::process::exit(0);
        }
        default_panic_hook(info);
    }));

    // Initialize logger so log::info!/warn! from melange-solver are visible.
    // Default: only warnings. RUST_LOG=info or RUST_LOG=melange_solver=debug for more.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .format_target(false)
        .init();

    // Handle a top-level `--version`/`-V` ourselves so we can append the EXACT
    // build identity — a runtime hash of THIS executable — which clap's
    // build-time-static version string cannot carry. A version+commit is a
    // source-side pointer that cannot see a dirty tree or a different
    // feature/profile build; two binaries at one commit printed the same string
    // and a peer published from each (cross-project review). We honour the flag
    // the way clap would: only at the top level, before any subcommand.
    for arg in std::env::args().skip(1) {
        if arg == "-V" || arg == "--version" {
            println!("{}", full_version_string());
            return Ok(());
        }
        if !arg.starts_with('-') {
            break; // reached the subcommand — its own flags are not ours
        }
    }

    let cli = Cli::parse();

    match cli.command {
        Commands::Compile {
            input,
            output,
            sample_rate,
            input_node,
            output_node,
            max_iter,
            tolerance,
            output_scale,
            output_clamp,
            format,
            with_level_params,
            no_level_params,
            no_dc_block,
            input_resistance: input_resistance_flag,
            oversampling,
            solver,
            backward_euler,
            force_trap,
            tube_grid_fa,
            subsample_fire,
            subsample_lit_factor,
            bjt_fa,
            opamp_rail_mode,
            nodal_subpath,
            allow_static_glow_on_full_lu,
            noise,
            noise_seed,
            allow_unconverged_dc_op,
            dc_op_max_iterations,
            emit_dc_op_recompute,
            name,
            mono,
            wet_dry_mix,
            no_ear_protection,
            vendor,
            vendor_url,
            email,
            vst3_id,
            clap_id,
            cpu_baseline,
        } => {
            // Validate numeric CLI parameters
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!(
                    "Sample rate must be positive and finite, got {}",
                    sample_rate
                );
            }
            if tolerance <= 0.0 || !tolerance.is_finite() {
                anyhow::bail!("Tolerance must be positive and finite, got {}", tolerance);
            }
            if max_iter == 0 {
                anyhow::bail!("max-iter must be at least 1, got 0");
            }
            if let Some(n) = oversampling {
                if n != 1 && n != 2 && n != 4 {
                    anyhow::bail!("oversampling must be 1, 2, or 4, got {}", n);
                }
            }
            if output_clamp <= 0.0 || !output_clamp.is_finite() {
                anyhow::bail!(
                    "--output-clamp must be positive and finite, got {}",
                    output_clamp
                );
            }

            // Parse op-amp rail mode. Unknown values are user errors, not silent fallbacks.
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Unknown --opamp-rail-mode '{}'. Valid values: auto, none, hard, \
                         active-set, active-set-be, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;

            // Parse the nodal sub-path override. Unknown values are user errors.
            let nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::parse(
                &nodal_subpath,
            )
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "Unknown --nodal-subpath '{}'. Valid values: auto, schur, full-lu",
                    nodal_subpath
                )
            })?;

            let noise_mode =
                melange_solver::codegen::NoiseMode::parse(&noise).ok_or_else(|| {
                    anyhow::anyhow!(
                        "Unknown --noise '{}'. Valid values: off, thermal, shot, full",
                        noise
                    )
                })?;

            // Level params are on by default; --no-level-params disables them
            let level_params = with_level_params && !no_level_params;

            // Validate shipability flags (plugin branding).
            if let Some(url) = vendor_url.as_deref() {
                // Simple scheme check — we don't pull in a URL-parsing crate for this.
                if !(url.starts_with("http://") || url.starts_with("https://")) {
                    anyhow::bail!("--vendor-url '{}' must start with http:// or https://", url);
                }
            }
            if let Some(id) = vst3_id.as_deref() {
                plugin_template::validate_vst3_id(id)?;
            }
            if let Some(id) = clap_id.as_deref() {
                if id.is_empty() {
                    anyhow::bail!("--clap-id cannot be empty");
                }
            }

            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            // Validate tube-grid-fa mode.
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "Unknown --tube-grid-fa '{}'. Valid values: auto, on, off",
                    tube_grid_fa
                );
            }
            // Validate bjt-fa mode.
            if !matches!(bjt_fa.as_str(), "auto" | "off" | "force") {
                anyhow::bail!(
                    "Unknown --bjt-fa '{}'. Valid values: auto, off, force",
                    bjt_fa
                );
            }
            let subsample_fire_mode = parse_subsample_fire_mode(&subsample_fire)?;

            compile_circuit_source(
                &circuit_source,
                CompileOptions {
                    output: &output,
                    sample_rate,
                    input_node: &input_node,
                    output_node: &output_node,
                    max_iter,
                    tolerance,
                    output_scale,
                    output_clamp,
                    format,
                    with_level_params: level_params,
                    input_resistance_flag,
                    oversampling_cli: oversampling,
                    no_dc_block,
                    solver_override: &solver,
                    backward_euler,
                    force_trap,
                    tube_grid_fa: &tube_grid_fa,
                    subsample_fire: subsample_fire_mode,
                    subsample_lit_factor,
                    bjt_fa: &bjt_fa,
                    opamp_rail_mode: rail_mode,
                    nodal_sub_path_override,
                    allow_static_glow_on_full_lu,
                    noise_mode,
                    noise_seed,
                    emit_dc_op_recompute,
                    allow_unconverged_dc_op,
                    dc_op_max_iterations,
                    plugin_name: name.as_deref(),
                    mono,
                    wet_dry_mix,
                    ear_protection: !no_ear_protection,
                    vendor: vendor.as_deref(),
                    vendor_url: vendor_url.as_deref(),
                    email: email.as_deref(),
                    vst3_id_override: vst3_id.as_deref(),
                    clap_id_override: clap_id.as_deref(),
                    cpu_baseline,
                },
            )
        }
        Commands::Validate {
            input,
            output_node,
            sample_rate,
            duration,
            amplitude,
            input_node,
            csv,
            relaxed,
            rms_tolerance,
            peak_tolerance,
            max_rel_tolerance,
            corr_min,
            thd_tolerance,
            bjt_fa,
            tube_grid_fa,
            backward_euler,
            force_trap,
            oversampling,
            rate_sweep,
        } => {
            // Validate numeric CLI parameters
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!("sample-rate must be positive and finite");
            }
            if !matches!(oversampling, 1 | 2 | 4) {
                anyhow::bail!("oversampling must be 1, 2, or 4, got {}", oversampling);
            }
            if rate_sweep && oversampling != 1 {
                anyhow::bail!(
                    "--rate-sweep sets the rate itself with oversampling off; drop --oversampling"
                );
            }
            if !matches!(bjt_fa.as_str(), "auto" | "off" | "force") {
                anyhow::bail!(
                    "--bjt-fa must be one of: auto, off, force (got '{}')",
                    bjt_fa
                );
            }
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "--tube-grid-fa must be one of: auto, on, off (got '{}')",
                    tube_grid_fa
                );
            }
            if duration <= 0.0 || !duration.is_finite() {
                anyhow::bail!("duration must be positive and finite");
            }
            if amplitude <= 0.0 || !amplitude.is_finite() {
                anyhow::bail!("amplitude must be positive and finite");
            }

            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            validate_circuit_source(
                &circuit_source,
                ValidateOptions {
                    output_node: &output_node,
                    sample_rate,
                    duration,
                    amplitude,
                    input_node: &input_node,
                    csv_output: csv.as_ref(),
                    relaxed,
                    tol: ToleranceOverrides {
                        rms_pct: rms_tolerance,
                        peak_v: peak_tolerance,
                        max_rel_pct: max_rel_tolerance,
                        corr_min,
                        thd_db: thd_tolerance,
                    },
                    reductions: ReductionModes {
                        bjt_fa: &bjt_fa,
                        tube_grid_fa: &tube_grid_fa,
                        backward_euler,
                        force_trap,
                    },
                    oversampling,
                    rate_sweep,
                },
            )
        }
        Commands::Simulate {
            input,
            input_audio,
            output,
            sample_rate,
            input_node,
            output_node,
            duration,
            amplitude,
            input_resistance: input_resistance_flag,
            solver,
            nodal_subpath,
            opamp_rail_mode,
            tube_grid_fa,
            subsample_fire,
            oversampling,
            noise,
            noise_seed,
            allow_unconverged_dc_op,
            dc_op_max_iterations,
            backward_euler,
            force_trap,
            max_iter,
            allow_nr_hold,
            allow_input_clamp,
            probes,
            probe_csv,
            pcm16,
            pot_overrides,
            switch_overrides,
            inject_drives,
        } => {
            // Match parse-time node normalization (lowercase, gnd→0).
            let input_node = melange_solver::parser::normalize_node_name(&input_node);
            let output_node = melange_solver::parser::normalize_node_name(&output_node);
            // Probes are node names too — normalize them the same way so
            // `--probe GND`-style refs and mixed-case names resolve.
            let probes: Vec<String> = probes
                .iter()
                .map(|p| melange_solver::parser::normalize_node_name(p))
                .collect();
            if let Some(n) = oversampling {
                if n != 1 && n != 2 && n != 4 {
                    anyhow::bail!("oversampling must be 1, 2, or 4, got {}", n);
                }
            }
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "Unknown --tube-grid-fa '{}'. Valid values: auto, on, off",
                    tube_grid_fa
                );
            }
            let subsample_fire_mode = parse_subsample_fire_mode(&subsample_fire)?;
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Invalid --opamp-rail-mode '{}'. Valid: auto, none, hard, \
                         active-set, active-set-be, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;
            let noise_mode =
                melange_solver::codegen::NoiseMode::parse(&noise).ok_or_else(|| {
                    anyhow::anyhow!(
                        "Invalid --noise '{}'. Valid: off, thermal, shot, full",
                        noise
                    )
                })?;
            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            // Default probe CSV: derive from output WAV (foo.wav → foo.probes.csv).
            // Only consulted when --probe is non-empty.
            let probe_csv_path: Option<PathBuf> = if probes.is_empty() {
                None
            } else if let Some(p) = probe_csv {
                Some(p)
            } else {
                let mut p = output.clone();
                let stem = p
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| "output".to_string());
                p.set_file_name(format!("{}.probes.csv", stem));
                Some(p)
            };
            let nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::parse(
                &nodal_subpath,
            )
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "Unknown --nodal-subpath '{}'. Valid values: auto, schur, full-lu",
                    nodal_subpath
                )
            })?;
            simulate_circuit_source(
                &circuit_source,
                &SimulateOptions {
                    nodal_sub_path_override,
                    input_audio: input_audio.as_deref(),
                    output: &output,
                    sample_rate,
                    input_node: &input_node,
                    output_node: &output_node,
                    duration,
                    amplitude,
                    input_resistance_flag,
                    solver: &solver,
                    opamp_rail_mode: rail_mode,
                    tube_grid_fa: &tube_grid_fa,
                    subsample_fire: subsample_fire_mode,
                    oversampling,
                    noise_mode,
                    noise_seed,
                    allow_unconverged_dc_op,
                    dc_op_max_iterations,
                    backward_euler,
                    force_trap,
                    max_iter,
                    allow_nr_hold,
                    allow_input_clamp,
                    probes: &probes,
                    probe_csv: probe_csv_path.as_deref(),
                    pcm16,
                    pot_overrides: &pot_overrides,
                    switch_overrides: &switch_overrides,
                    inject_drives: &inject_drives,
                },
            )
        }
        Commands::Analyze {
            input,
            input_node,
            output_node,
            start_freq,
            end_freq,
            points_per_decade,
            amplitude,
            sample_rate,
            input_resistance,
            output,
            pot_overrides,
            switch_overrides,
            harmonics,
            tube_grid_fa,
            solver,
            oversampling,
            opamp_rail_mode,
            nodal_subpath,
            noise,
            noise_seed,
            allow_unconverged_dc_op,
            dc_op_max_iterations,
            backward_euler,
            force_trap,
            max_iter,
            allow_input_clamp,
        } => {
            // Validate numeric CLI parameters
            if start_freq <= 0.0 || !start_freq.is_finite() {
                anyhow::bail!("start-freq must be positive and finite");
            }
            if end_freq <= start_freq || !end_freq.is_finite() {
                anyhow::bail!("end-freq must be greater than start-freq and finite");
            }
            if amplitude <= 0.0 || !amplitude.is_finite() {
                anyhow::bail!("amplitude must be positive and finite");
            }
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!("sample-rate must be positive and finite");
            }
            if points_per_decade == 0 {
                anyhow::bail!("points-per-decade must be at least 1");
            }
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "Unknown --tube-grid-fa '{}'. Valid values: auto, on, off",
                    tube_grid_fa
                );
            }
            if let Some(n) = oversampling {
                if n != 1 && n != 2 && n != 4 {
                    anyhow::bail!("oversampling must be 1, 2, or 4, got {}", n);
                }
            }
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Invalid --opamp-rail-mode '{}'. Valid: auto, none, hard, \
                         active-set, active-set-be, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;
            // Parse the nodal sub-path override. Unknown values are user errors.
            let nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::parse(
                &nodal_subpath,
            )
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "Unknown --nodal-subpath '{}'. Valid values: auto, schur, full-lu",
                    nodal_subpath
                )
            })?;

            let noise_mode =
                melange_solver::codegen::NoiseMode::parse(&noise).ok_or_else(|| {
                    anyhow::anyhow!(
                        "Invalid --noise '{}'. Valid: off, thermal, shot, full",
                        noise
                    )
                })?;

            let circuit_source = circuits::resolve(&input)?;
            analyze_freq_response(
                &circuit_source,
                &AnalyzeOptions {
                    nodal_sub_path_override,
                    input_node: &input_node,
                    output_node: &output_node,
                    start_freq,
                    end_freq,
                    points_per_decade,
                    amplitude,
                    sample_rate,
                    input_resistance_flag: input_resistance,
                    output_file: output.as_ref(),
                    pot_overrides: &pot_overrides,
                    switch_overrides: &switch_overrides,
                    harmonics,
                    tube_grid_fa: &tube_grid_fa,
                    solver: &solver,
                    oversampling,
                    opamp_rail_mode: rail_mode,
                    noise_mode,
                    noise_seed,
                    allow_unconverged_dc_op,
                    dc_op_max_iterations,
                    backward_euler,
                    force_trap,
                    max_iter,
                    allow_input_clamp,
                },
            )
        }
        Commands::DcOp {
            input,
            input_node,
            input_resistance,
            format,
            sample_rate,
            oversampling,
            solver,
            opamp_rail_mode,
            bjt_fa,
            tube_grid_fa,
            pot_overrides,
            allow_unconverged_dc_op,
            dc_op_max_iterations,
        } => {
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!(
                    "Sample rate must be positive and finite, got {}",
                    sample_rate
                );
            }
            if let Some(n) = oversampling {
                if !matches!(n, 1 | 2 | 4) {
                    anyhow::bail!("oversampling must be 1, 2, or 4, got {}", n);
                }
            }
            if !matches!(solver.as_str(), "auto" | "dk" | "nodal") {
                anyhow::bail!(
                    "Unknown --solver '{}'. Valid values: auto, dk, nodal",
                    solver
                );
            }
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "Unknown --tube-grid-fa '{}'. Valid values: auto, on, off",
                    tube_grid_fa
                );
            }
            if !matches!(bjt_fa.as_str(), "auto" | "off" | "force") {
                anyhow::bail!(
                    "Unknown --bjt-fa '{}'. Valid values: auto, off, force",
                    bjt_fa
                );
            }
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Unknown --opamp-rail-mode '{}'. Valid values: auto, none, hard, \
                         active-set, active-set-be, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;
            let circuit_source = circuits::resolve(&input)?;
            eprintln!("Resolved circuit: {}", circuit_source.name());
            run_dc_op(
                &circuit_source,
                &DcOpOptions {
                    input_node: &input_node,
                    input_resistance,
                    format: &format,
                    sample_rate,
                    oversampling,
                    solver: &solver,
                    opamp_rail_mode: rail_mode,
                    bjt_fa: &bjt_fa,
                    tube_grid_fa: &tube_grid_fa,
                    pot_overrides: &pot_overrides,
                    allow_unconverged_dc_op,
                    dc_op_max_iterations,
                },
            )
        }
        Commands::Nodes { input } => {
            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            list_nodes_source(&circuit_source)
        }
        Commands::Index { dir, check } => index_cmd::run(&dir, check),
        Commands::Sources { action } => handle_sources(action),
        Commands::Builtins => list_builtins(),
        Commands::Cache { action } => handle_cache(action),
        Commands::Import {
            input,
            output,
            format,
            from_schematic,
        } => kicad_import::import_kicad(&input, &output, &format, from_schematic),
    }
}

/// Frame the normal-path routing decision as information rather than a warning.
///
/// The router's own reason strings are written for maintainers and contain the
/// words "unstable" and "ill-conditioned". A first-time user reading
/// `solver: multi-transformer circuit (3 groups, DK K matrix unstable)` on the
/// SHIPPED demo circuit reasonably concludes they broke something. They did
/// not: melange builds the DK kernel on every route, measures it, and picks
/// whichever of the two solvers can model that circuit correctly. Keep the
/// maintainer detail verbatim — just say out loud that this line is normal.
/// Mirrors the `info (normal):` prefix used in `melange_solver::pipeline`.
fn format_route_info(route_label: &str, reason: &str) -> String {
    // The "why not DK" gloss only makes sense when DK was not chosen.
    let why_not_dk = if route_label.eq_ignore_ascii_case("dk") {
        ")"
    } else {
        " \"unstable\" / \"ill-conditioned\" say\n\
         \x20                 why the DK route was not the fit here; they are not a verdict on the \
         netlist.)"
    };
    format!(
        "  info (normal): solver route = {route_label} \u{2014} {reason}\n\
         \x20                (normal routing output, not a warning: melange measures the DK kernel \
         it just built and\n\
         \x20                 picks the solver that models this circuit correctly.{why_not_dk}"
    )
}

/// Parse the `--bjt-fa` string into a [`melange_solver::codegen::BjtFaMode`].
/// Assumes the value was already validated (`auto` | `off` | `force`); an
/// unrecognized value falls back to `Off`, the default.
fn parse_bjt_fa_mode(s: &str) -> melange_solver::codegen::BjtFaMode {
    match s {
        "off" => melange_solver::codegen::BjtFaMode::Off,
        "force" => melange_solver::codegen::BjtFaMode::Force,
        "auto" => melange_solver::codegen::BjtFaMode::Auto,
        _ => melange_solver::codegen::BjtFaMode::Off,
    }
}

/// Whether the build summary names the DC operating point's railed op-amps:
/// always when they are pinned or the pin fell back.
fn meta_rail_pin_shown(label: &str) -> bool {
    !label.is_empty() && label != "none"
}

/// Diagnostic lit sub-step multiplier (`MELANGE_LIT_FACTOR` env var) for the
/// design review demand-3 sweep / lock-margin gate (cross-project review).
/// Deliberately an env var, not a CLI flag — a throwaway diagnostic knob. The
/// resolved value is recorded in the provenance manifest (`lit_factor`), so a
/// swept measurement carries its own build identity. Default 0.5 (= tau/2).
fn diag_lit_factor() -> f64 {
    std::env::var("MELANGE_LIT_FACTOR")
        .ok()
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|&f| f > 0.0)
        .unwrap_or(1.0)
}

/// A library build failure as the CLI reports it: the context line over the
/// underlying error, as `anyhow` chains them.
fn build_error(e: melange_solver::build::BuildError) -> anyhow::Error {
    let (context, source) = e.into_parts();
    let err = anyhow::anyhow!(source);
    match context {
        Some(c) => err.context(c),
        None => err,
    }
}

/// `melange compile`'s options (named, so two same-typed options cannot be
/// passed in each other's place).
struct CompileOptions<'a> {
    output: &'a PathBuf,
    sample_rate: f64,
    input_node: &'a str,
    output_node: &'a str,
    max_iter: usize,
    tolerance: f64,
    output_scale: f64,
    output_clamp: f64,
    format: OutputFormat,
    with_level_params: bool,
    input_resistance_flag: Option<f64>,
    oversampling_cli: Option<usize>,
    no_dc_block: bool,
    solver_override: &'a str,
    backward_euler: bool,
    force_trap: bool,
    tube_grid_fa: &'a str,
    subsample_fire: melange_solver::codegen::SubsampleFireMode,
    subsample_lit_factor: Option<f64>,
    bjt_fa: &'a str,
    opamp_rail_mode: melange_solver::codegen::OpampRailMode,
    nodal_sub_path_override: melange_solver::codegen::NodalSubPathOverride,
    allow_static_glow_on_full_lu: bool,
    noise_mode: melange_solver::codegen::NoiseMode,
    noise_seed: u64,
    emit_dc_op_recompute: bool,
    allow_unconverged_dc_op: bool,
    /// Test-only DC-OP Newton budget (hidden `--dc-op-max-iterations`).
    dc_op_max_iterations: Option<usize>,
    plugin_name: Option<&'a str>,
    mono: bool,
    wet_dry_mix: bool,
    ear_protection: bool,
    vendor: Option<&'a str>,
    vendor_url: Option<&'a str>,
    email: Option<&'a str>,
    vst3_id_override: Option<&'a str>,
    clap_id_override: Option<&'a str>,
    cpu_baseline: plugin_template::CpuBaseline,
}

fn compile_circuit_source(
    circuit_source: &circuits::CircuitSource,
    opts: CompileOptions<'_>,
) -> Result<()> {
    let CompileOptions {
        output,
        sample_rate,
        input_node,
        output_node,
        max_iter,
        tolerance,
        output_scale,
        output_clamp,
        format,
        with_level_params,
        input_resistance_flag,
        oversampling_cli,
        no_dc_block,
        solver_override,
        backward_euler,
        force_trap,
        tube_grid_fa,
        subsample_fire,
        subsample_lit_factor,
        bjt_fa,
        opamp_rail_mode,
        nodal_sub_path_override,
        allow_static_glow_on_full_lu,
        noise_mode,
        noise_seed,
        emit_dc_op_recompute,
        allow_unconverged_dc_op,
        dc_op_max_iterations,
        plugin_name,
        mono,
        wet_dry_mix,
        ear_protection,
        vendor,
        vendor_url,
        email,
        vst3_id_override,
        clap_id_override,
        cpu_baseline,
    } = opts;
    if format == OutputFormat::Plugin {
        refuse_existing_plugin_project(&plugin_project_dir(output), &circuit_source.name())?;
    }
    // Netlist node names are normalized (lowercase, gnd→0) at parse time;
    // fold the CLI-provided names the same way so lookups match.
    // Parse comma-separated input nodes (multi-input ports), mirroring the
    // comma-separated output-node handling below. Port 0 (the first name) is the
    // "primary" input; any extras drive additional ports for M=0 (linear)
    // circuits. A single input name collapses to the historical single-input
    // path (byte-identical generated code). See `local-docs/multi-input-ports-plan.md`.
    let input_node_names_owned: Vec<String> = input_node
        .split(',')
        .map(|s| melange_solver::parser::normalize_node_name(s.trim()))
        .filter(|s| !s.is_empty())
        .collect();
    if input_node_names_owned.is_empty() {
        anyhow::bail!("no input node specified");
    }
    let input_node_owned = input_node_names_owned[0].clone();
    let input_node = input_node_owned.as_str();
    let output_node_owned = melange_solver::parser::normalize_node_name(output_node);
    let output_node = output_node_owned.as_str();

    println!("melange compile");
    println!("  Source: {}", circuit_source.name());
    println!("  Output: {}", output.display());
    println!("  Sample rate: {} Hz", sample_rate);
    println!();

    // Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, name } => {
            println!("  Using builtin circuit: {}", name);
            content.clone()
        }
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read local file: {}", path.display()))?,
        src @ (circuits::CircuitSource::Url { .. } | circuits::CircuitSource::Friendly { .. }) => {
            fetch_remote_circuit(src)?
        }
    };

    // --mono is incompatible with multiple output nodes: a multi-output
    // plugin takes mono input and routes each output node to its own audio
    // channel, so a 1-channel layout can't represent it. Erroring beats
    // silently generating a plugin whose second output node is inaudible.
    let output_node_names: Vec<&str> = output_node.split(',').map(|s| s.trim()).collect();
    if mono && output_node_names.len() > 1 {
        anyhow::bail!(
            "--mono cannot be combined with multiple output nodes ({} given: \"{}\"). \
             Multi-output plugins route each output node to its own channel — \
             drop --mono, or pass a single --output-node.",
            output_node_names.len(),
            output_node_names.join(", ")
        );
    }

    let circuit_name = output
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("circuit")
        .to_string();
    let circuit_name: String = circuit_name
        .to_lowercase()
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    let circuit_name = if circuit_name.starts_with(|c: char| c.is_ascii_digit()) {
        format!("circuit_{circuit_name}")
    } else {
        circuit_name
    };

    // The one build every verb ships (melange_solver::build).
    let build_opts = melange_solver::build::BuildOptions {
        sample_rate,
        circuit_name,
        input_nodes: input_node_names_owned.clone(),
        output_nodes: output_node_names.iter().map(|s| s.to_string()).collect(),
        // `--max-iter 50` (the default value) means "not explicitly set".
        max_iter: if max_iter == 50 { None } else { Some(max_iter) },
        tolerance,
        output_scale,
        output_clamp,
        input_resistance: input_resistance_flag,
        oversampling: oversampling_cli,
        dc_block: !no_dc_block,
        solver: solver_override.to_string(),
        backward_euler,
        force_trap,
        tube_grid_fa: tube_grid_fa.to_string(),
        subsample_fire,
        // Flag wins; else the MELANGE_LIT_FACTOR env var (diagnostic); else 1.0.
        subsample_lit_factor: subsample_lit_factor.unwrap_or_else(diag_lit_factor),
        bjt_fa_mode: parse_bjt_fa_mode(bjt_fa),
        opamp_rail_mode,
        nodal_sub_path_override,
        allow_static_glow_on_full_lu,
        noise_mode,
        noise_seed,
        emit_dc_op_recompute,
        plugin_format: format != OutputFormat::Code,
        pot_overrides: None,
        resolve_taps: true,
        inject_runtime: true,
        disable_unit_variation: false,
        disable_self_heating: false,
        allow_unconverged_dc_op,
        dc_op_max_iterations,
        output_clamp_auto: false,
    };
    let melange_solver::build::Built {
        generated,
        netlist,
        mna,
        kernel,
        routing,
        solver_label,
        solver_reason,
        max_iter,
        oversampling,
        input_resistance,
        input_resistance_source: ir_source,
        output_node_indices,
        forward_active,
        grid_off_pentodes,
        linearize_outcome,
        ..
    } = melange_solver::build::build(&netlist_str, &build_opts, &|a| println!("{a}"), &|a| {
        eprintln!("{a}")
    })
    .map_err(build_error)?;

    // Auto-mono: single output node with single-channel circuit → default to mono.
    // Stereo duplicates the same mono circuit per channel, which is correct but
    // doubles CPU for no benefit unless the user has a stereo reason (e.g. wet/dry).
    let mono = if !mono && output_node_indices.len() == 1 && format == OutputFormat::Plugin {
        println!("  Auto-selecting mono (single output node). Use two output nodes for stereo.");
        true
    } else {
        mono
    };

    let line_count = generated.code.lines().count();
    println!("  ✓ Generated {} lines of Rust code", line_count);

    // Compilation summary: report all auto-detected decisions in one place.
    println!();
    println!("  Summary:");
    println!(
        "    System: {}",
        format_system_size(generated.n, mna.n, generated.m)
    );
    // "(normal)" because the reason strings carry maintainer words like
    // "unstable" / "ill-conditioned" that read as warnings — see
    // `format_route_info`.
    println!(
        "    Solver: {} \u{2014} normal routing output, not a warning ({})",
        solver_label, solver_reason
    );
    // Non-negative K diagonal note, printed ONCE here (the low-level kernel
    // builder logs it at debug only — it is rebuilt several times per compile).
    // Informative when a transformer-coupled NFB circuit routes to nodal for a
    // different primary reason (e.g. trap instability) but ALSO has a positive
    // K diagonal the author may want to know about.
    if routing.k_diag_unsafe {
        println!(
            "    Note: non-negative K diagonal (positive DK-Schur feedback, \
             expected for transformer-coupled NFB) — handled by nodal full-NR."
        );
    }
    // Which nodal sub-path the emitter actually took. Reported by the emitter,
    // not re-derived here. Without this a deck authored to reach full-LU could
    // silently sit on Schur with nothing to reveal it.
    if let Some(sp) = generated.meta.nodal_sub_path {
        // "Nodal NR sub-path" — explicitly the nodal Newton implementation
        // (Schur reduction vs full-LU), distinct from the DK kernel's BJT
        // internal-node expansion, which also says "full LU" (see pipeline.rs).
        //
        // Name the predicate that ACTUALLY fired, as the emitter reported it.
        // This line used to assert "route decided by nodal spectral radius
        // {rho}" for every full-LU route — which was false whenever a different
        // predicate fired first, and on `steve-1073-preamp` it named rho = 0.9879
        // as the deciding value when the trigger was `s-ill-conditioned`
        // (max|S| = 5.00e8) and 0.9879 is below every rho threshold. A confident
        // wrong reason is worse than no reason: the line was added to stop a
        // reader mis-attributing a route (design review) and then mis-attributed
        // one itself (design review). rho stays, as context, labelled as context.
        match generated.meta.nodal_full_lu_trigger {
            Some(trigger) => println!(
                "    Nodal NR sub-path: {sp} (nodal Newton; not DK node-expansion; \
                 trigger: {trigger}; nodal spectral radius {:.4})",
                generated.meta.nodal_spectral_radius
            ),
            None => println!(
                "    Nodal NR sub-path: {sp} (nodal Newton; not DK node-expansion; \
                 nodal spectral radius {:.4})",
                generated.meta.nodal_spectral_radius
            ),
        }
    }
    if routing.spectral_radius > 0.0 {
        // DK-kernel trap operator (routing::compute_spectral_radius) — this is
        // NOT the value that chose a nodal sub-path (see the line above).
        println!(
            "    DK-kernel spectral radius: {:.4}",
            routing.spectral_radius
        );
    }
    // Integration line: printed from the codegen-recorded selection so the
    // stated reason is the actual one (a `.integrator be` pin is NOT
    // "auto-selected").
    {
        use melange_solver::codegen::ir::IntegratorSelection as Sel;
        match generated.meta.integrator_selection {
            Sel::BeAuto => {
                println!("    Integration: Backward Euler (auto-selected)");
                println!("      ({})", generated.meta.integration_reason);
            }
            Sel::TrapDefault => {
                println!("    Integration: Trapezoidal");
                if !generated.meta.integration_reason.is_empty() {
                    println!("      ({})", generated.meta.integration_reason);
                }
            }
            Sel::TrapCliFlag => println!("    Integration: Trapezoidal (pinned by --force-trap)"),
            Sel::TrapDirective => {
                println!("    Integration: Trapezoidal (pinned by .integrator directive)");
            }
            Sel::BeCliFlag => println!("    Integration: Backward Euler (--backward-euler)"),
            Sel::BeDirective => {
                println!("    Integration: Backward Euler (pinned by .integrator directive)");
            }
            Sel::BeBehavioral => {
                println!("    Integration: Backward Euler (required by behavioral B-sources)");
            }
        }
    }
    if max_iter != 50 {
        println!(
            "    Max NR iterations: {} (auto-tuned from M={}, ρ={:.2})",
            max_iter, kernel.m, routing.spectral_radius
        );
    }
    if routing.k_ill_conditioned {
        println!("    K matrix: ill-conditioned (max|K| > 1e8, routed to nodal)");
    }
    if routing.s_ill_conditioned {
        println!("    S matrix: ill-conditioned (max|S| > 1e6, cap-only nodes)");
    }
    {
        let n_lin = mna.linearized_triodes.len() + mna.linearized_bjts.len();
        if n_lin > 0 {
            println!(
                "    Linearized devices: {} (K/S magnitude guards bypassed)",
                n_lin
            );
        }
    }
    println!("    Oversampling: {}×", oversampling);
    // A nonlinear circuit generates harmonics above Nyquist, and at 1x they
    // fold back into the audible band as inharmonic alias energy. The steady-
    // state frequency response does not show it, so a first-time author has no
    // way to discover the decision they just made by default.
    //
    // Fires ONLY when 1x was defaulted into — never when the author chose it on
    // the command line or in the deck. Telling someone about a decision they
    // already made is the noise that stops people reading notes at all.
    //
    // A NOTE, not a warning: 1x is correct for plenty of circuits (most of the
    // golden corpus runs there), and the amount that oversampling helps is
    // strongly circuit- and drive-dependent — so this points at the
    // measurement rather than prescribing a factor.
    if generated.m > 0
        && oversampling == 1
        && oversampling_cli.is_none()
        && netlist.recommended_oversampling.is_none()
    {
        println!(
            "    NOTE: {} nonlinear dimension(s) at 1× — harmonics above Nyquist fold",
            generated.m
        );
        println!("          back as aliasing, which the frequency response will not show.");
        println!(
            "          Measure it:  melange simulate <circuit> --input-audio <tone.wav> at 1× and"
        );
        println!(
            "                       --oversampling 4; compare the spectra (docs/OVERSAMPLING.md)"
        );
        println!(
            "          Set it:      --oversampling {{2|4}}, or `.oversampling N` in the deck."
        );
        println!("          Why it is not simply a quality dial: docs/OVERSAMPLING.md");
    }
    println!(
        "    Input: node \"{}\", resistance {}Ω ({})",
        input_node, input_resistance, ir_source
    );
    println!(
        "    Output: node \"{}\", scale {}, clamp ±{} V",
        output_node_names.join(", "),
        output_scale,
        output_clamp,
    );
    println!(
        "    DC block: {}",
        if !no_dc_block {
            "enabled (5 Hz HPF)"
        } else {
            "disabled"
        }
    );
    // DC operating point
    if generated.m > 0 {
        let meta = &generated.meta;
        if meta.dc_op_converged {
            println!(
                "    DC operating point: converged ({}, {} iterations)",
                meta.dc_op_method, meta.dc_op_iterations
            );
        } else {
            println!(
                "    DC operating point: *** DID NOT CONVERGE *** ({}, {} iterations)",
                meta.dc_op_method, meta.dc_op_iterations
            );
        }
        if !meta.parasitic_caps.is_empty() {
            println!(
                "    Parasitic caps: {} x 10 pF auto-inserted across device junctions (no \
                 capacitors in circuit)",
                meta.parasitic_caps.len()
            );
        }
    }
    if meta_rail_pin_shown(&generated.meta.dc_op_rail_pin) {
        println!(
            "    Railed op-amps at DC: {}",
            generated.meta.dc_op_rail_pin
        );
    }
    if !forward_active.is_empty() {
        println!(
            "    Forward-active reduction: {} BJTs linearized to 1D",
            forward_active.len()
        );
    }
    if !grid_off_pentodes.is_empty() {
        println!(
            "    Grid-off reduction: {} pentodes reduced to 2D",
            grid_off_pentodes.len()
        );
    }
    if linearize_outcome.bjts_linearized > 0 {
        println!(
            "    Linearized BJTs: {} (fully linear, M reduced by {})",
            linearize_outcome.bjts_linearized,
            linearize_outcome.bjts_linearized * 2
        );
    }
    if linearize_outcome.triodes_linearized > 0 {
        println!(
            "    Linearized triodes: {} (fully linear, M reduced by {})",
            linearize_outcome.triodes_linearized,
            linearize_outcome.triodes_linearized * 2
        );
    }
    println!("    Generated: {} lines of Rust", line_count);
    println!();

    // Step 5: Write output
    println!("Step 5: Writing output...");

    match format {
        OutputFormat::Code => {
            // Write single file (existing behavior)
            std::fs::write(output, &generated.code)
                .with_context(|| format!("Failed to write output file: {}", output.display()))?;

            println!("  ✓ Done!");
            println!();
            println!("Generated code written to: {}", output.display());
            println!("This code can be used with:");
            println!("  - melange-plugin for VST/AU/CLAP plugins");
            println!("  - Standalone integration in your own projects");
            println!();
            // The emitted file is the default output of `compile`, and its
            // caller-facing API (Default constructor, free `process_sample`,
            // pot/switch setters, output units) is not guessable from the
            // filename. Point at the doc here, where the reader actually is.
            println!("Its API is documented in docs/CODE_API.md:");
            println!("  let mut state = circuit::CircuitState::default();");
            println!("  state.set_sample_rate(48_000.0);");
            println!("  let out = circuit::process_sample(input, &mut state); // free fn -> [f64; NUM_OUTPUTS], volts");
            println!();
            // Suggest a DIRECTORY, not this file path. `--format plugin`
            // treats --output as the project root; echoing back
            // `.../src/circuit.rs` (which is what this hint used to do) sends
            // the reader to `cargo new` a project inside another project's
            // src/, named after a file. Derive a plain project name from the
            // circuit instead.
            let project_suggestion = suggest_plugin_project_dir(&circuit_source.name());
            println!("To generate a complete plugin project instead, use:");
            println!(
                "  melange compile {} --output {} --format plugin",
                circuit_source.name(),
                project_suggestion
            );
            println!("  (with --format plugin, --output is the project DIRECTORY, not a file)");
        }
        OutputFormat::Plugin => {
            // Generate complete plugin project
            let project_dir = plugin_project_dir(output);

            // Get a sanitized circuit name
            let circuit_name: String = project_dir
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("circuit")
                .to_lowercase()
                .chars()
                .map(|c| {
                    if c.is_ascii_alphanumeric() || c == '_' {
                        c
                    } else {
                        '_'
                    }
                })
                .collect();
            let circuit_name = if circuit_name.starts_with(|c: char| c.is_ascii_digit()) {
                format!("circuit_{circuit_name}")
            } else {
                circuit_name
            };

            // Build pot/switch parameter info from MNA data (works for both DK and nodal paths).
            // Skip entries declared via `.runtime R` — those are driven by plugin-side
            // envelope followers via `set_runtime_R_<field>` and do not get nih-plug
            // knobs. The netlist.pots index lookup stays aligned because `.pot`
            // directives are pushed into mna.pots BEFORE `.runtime R` directives.
            let pot_params: Vec<plugin_template::PotParamInfo> = mna
                .pots
                .iter()
                .enumerate()
                .filter(|(_, p)| p.runtime_field.is_none())
                .map(|(idx, p)| plugin_template::PotParamInfo {
                    index: idx,
                    name: netlist
                        .pots
                        .get(idx)
                        .map(|d| d.label.clone().unwrap_or_else(|| d.resistor_name.clone()))
                        .unwrap_or_else(|| format!("Pot {}", idx)),
                    min_resistance: p.min_resistance,
                    max_resistance: p.max_resistance,
                    default_resistance: netlist
                        .pots
                        .get(idx)
                        .and_then(|d| d.default_value)
                        .unwrap_or(1.0 / p.g_nominal),
                })
                .collect();
            let switch_params: Vec<plugin_template::SwitchParamInfo> = netlist
                .switches
                .iter()
                .enumerate()
                .map(|(idx, sw)| plugin_template::SwitchParamInfo {
                    index: idx,
                    name: sw.label.clone().unwrap_or_else(|| {
                        format!("Switch {} ({})", idx, sw.component_names.join("+"))
                    }),
                    num_positions: sw.positions.len(),
                })
                .collect();

            // Build wiper params from MNA wiper groups
            let wiper_params: Vec<plugin_template::WiperParamInfo> = mna
                .wiper_groups
                .iter()
                .enumerate()
                .map(|(idx, wg)| plugin_template::WiperParamInfo {
                    wiper_index: idx,
                    cw_pot_index: wg.cw_pot_index,
                    ccw_pot_index: wg.ccw_pot_index,
                    total_resistance: wg.total_resistance,
                    default_position: wg.default_position,
                    name: wg.label.clone().unwrap_or_else(|| format!("Wiper {}", idx)),
                })
                .collect();

            // Build gang params from MNA gang groups
            let gang_params: Vec<plugin_template::GangParamInfo> = mna
                .gang_groups
                .iter()
                .enumerate()
                .map(|(idx, gg)| plugin_template::GangParamInfo {
                    index: idx,
                    label: gg.label.clone(),
                    default_position: gg.default_position,
                    pot_members: gg
                        .pot_members
                        .iter()
                        .map(|&(pot_idx, inverted)| {
                            (
                                pot_idx,
                                mna.pots[pot_idx].min_resistance,
                                mna.pots[pot_idx].max_resistance,
                                inverted,
                            )
                        })
                        .collect(),
                    wiper_members: gg
                        .wiper_members
                        .iter()
                        .map(|&(wg_idx, inverted)| {
                            let wg = &mna.wiper_groups[wg_idx];
                            (
                                wg.cw_pot_index,
                                wg.ccw_pot_index,
                                wg.total_resistance,
                                inverted,
                            )
                        })
                        .collect(),
                })
                .collect();

            // Collect pot indices claimed by gangs
            let gang_claimed_pots: std::collections::HashSet<usize> = gang_params
                .iter()
                .flat_map(|g| g.pot_members.iter().map(|&(idx, _, _, _)| idx))
                .collect();
            let gang_claimed_wipers: std::collections::HashSet<usize> = gang_params
                .iter()
                .flat_map(|g| {
                    g.wiper_members
                        .iter()
                        .flat_map(|&(cw, ccw, _, _)| [cw, ccw])
                })
                .collect();

            // Filter out wiper-claimed and gang-claimed pots from individual pot params
            let wiper_claimed: std::collections::HashSet<usize> = wiper_params
                .iter()
                .flat_map(|w| [w.cw_pot_index, w.ccw_pot_index])
                .collect();
            let pot_params: Vec<_> = pot_params
                .into_iter()
                .filter(|p| {
                    !wiper_claimed.contains(&p.index) && !gang_claimed_pots.contains(&p.index)
                })
                .collect();

            // Filter out gang-claimed wipers from individual wiper params
            let wiper_params: Vec<_> = wiper_params
                .into_iter()
                .filter(|w| !gang_claimed_wipers.contains(&w.cw_pot_index))
                .collect();

            let plugin_options = plugin_template::PluginOptions {
                plugin_name,
                mono,
                wet_dry_mix,
                ear_protection,
                vendor,
                url: vendor_url,
                email,
                vst3_id: vst3_id_override,
                clap_id: clap_id_override,
                cpu_baseline,
            };
            plugin_template::generate_plugin_project_with_oversampling(
                &project_dir,
                &generated.code,
                &circuit_name,
                with_level_params,
                &pot_params,
                &wiper_params,
                &gang_params,
                &switch_params,
                output_node_indices.len(),
                oversampling,
                &plugin_options,
            )?;

            println!("  ✓ Done!");
            println!();
            println!("Generated plugin project at: {}", project_dir.display());
            println!();
            let dir_name = project_dir
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("<project-dir>");
            println!("Build the DSP library (works immediately):");
            println!("  cd {dir_name}");
            println!("  cargo build --release          # → raw plugin lib in target/release/");
            println!();
            println!("For a DAW-loadable CLAP + VST3 bundle (no nih-plug clone needed):");
            println!("  cd {dir_name}");
            println!("  bash build.sh                  # → CLAP + VST3 in target/bundled/");
            println!("  # (must be OUTSIDE any Cargo workspace — see the warning above if any)");
            println!();
            println!("See the generated README.md for full details.");
        }
    }

    Ok(())
}

/// `melange validate`'s stimulus: a sine at this frequency.
const VALIDATE_STIMULUS_HZ: f64 = 1000.0;
/// The rate sweep's stimulus. Incommensurate with every rate it renders: at a
/// frequency that divides the sample rate, a clipping deck's corners fall at the
/// same sub-sample phase every period, a different one at each rate, and that
/// fixed phase has twice forged a PLATEAU / DIVERGES verdict on a converging
/// deck. Stated, not derived.
const RATE_SWEEP_STIMULUS_HZ: f64 = 997.3;
/// Stimulus periods left out of every metric at the start of a validate render.
const VALIDATE_SETTLE_PERIODS: f64 = 20.0;
/// The default profile's peak bound, relative to the reference's peak over the
/// compared window. A stated bound (a 1 % peak error on the reference's own
/// scale), not a derived one. Measured 2026-09-29 over 85 local-corpus decks,
/// both sides with the settle window, against the absolute 20 mV: +2 passes, 0
/// regressions, the nearest passing deck at 0.90 %; at 0.5 %, +1 pass and 3
/// regressions (decks at 0.59-0.77 %).
const VALIDATE_PEAK_RELATIVE: f64 = 0.01;

/// Optional per-metric tolerance overrides from the CLI, applied on top of the
/// --relaxed/strict base profile. `None` fields keep the profile value.
#[derive(Default)]
struct ToleranceOverrides {
    /// RMS error tolerance, percent (2.0 = 2%).
    rms_pct: Option<f64>,
    /// Peak error tolerance, volts (absolute).
    peak_v: Option<f64>,
    /// Max relative error tolerance, percent (5.0 = 5%).
    max_rel_pct: Option<f64>,
    /// Minimum correlation coefficient (0.0–1.0).
    corr_min: Option<f64>,
    /// THD error tolerance, dB.
    thd_db: Option<f64>,
}

/// Dimension-reduction modes for the validation front end. Mirrors the
/// `melange compile` flags of the same names so that `validate` builds the
/// circuit the CLI ships — and so `--bjt-fa off` can attribute a residual to
/// the reduction.
struct ReductionModes<'a> {
    bjt_fa: &'a str,
    tube_grid_fa: &'a str,
    // Diagnostics (not reductions): melange-side integrator override, for
    // attributing integrator error against ngspice. Oversampling is NOT here —
    // it is not a diagnostic but part of the shipped build, and it rides its
    // own field on `ValidateOptions`.
    backward_euler: bool,
    force_trap: bool,
}

/// `melange validate`'s options (named, so two same-typed options cannot be
/// passed in each other's place).
struct ValidateOptions<'a> {
    output_node: &'a str,
    sample_rate: f64,
    duration: f64,
    amplitude: f64,
    input_node: &'a str,
    csv_output: Option<&'a PathBuf>,
    relaxed: bool,
    tol: ToleranceOverrides,
    reductions: ReductionModes<'a>,
    oversampling: usize,
    rate_sweep: bool,
}

fn validate_circuit_source(
    circuit_source: &circuits::CircuitSource,
    opts: ValidateOptions<'_>,
) -> Result<()> {
    let ValidateOptions {
        output_node,
        sample_rate,
        duration,
        amplitude,
        input_node,
        csv_output,
        relaxed,
        tol,
        reductions,
        oversampling,
        rate_sweep,
    } = opts;
    // Match parse-time node normalization (lowercase, gnd→0).
    let input_node_owned = melange_solver::parser::normalize_node_name(input_node);
    let input_node = input_node_owned.as_str();
    let output_node_owned = melange_solver::parser::normalize_node_name(output_node);
    let output_node = output_node_owned.as_str();
    use melange_validate::{
        comparison::ComparisonConfig, spice_runner::is_ngspice_available,
        validate_circuit_with_options, ValidationOptions,
    };

    println!("melange validate");
    println!("  Source: {}", circuit_source.name());
    println!("  Output node: {}", output_node);
    println!("  Input node: {}", input_node);
    println!("  Sample rate: {} Hz", sample_rate);
    println!("  Duration: {}s", duration);
    println!("  Amplitude: {}V", amplitude);
    println!(
        "  Tolerances: {}",
        if relaxed { "relaxed" } else { "strict" }
    );
    if oversampling > 1 {
        // Say what is being validated and what was done about the filters,
        // BEFORE the number appears. This run measures different DSP from the
        // 1x run above it in someone's scrollback.
        println!(
            "  Oversampling: {}\u{d7} (solver at {:.0} Hz internally)",
            oversampling,
            sample_rate * oversampling as f64
        );

        println!(
            "    The emitted code interpolates and decimates through polyphase IIR \
             half-band allpass chains,"
        );
        println!(
            "    whose frequency-dependent phase stays in the comparison. The reference is \
             NOT filtered:"
        );
        println!(
            "    it is aligned to the melange output by one best-fit constant delay \
             (analytic seed {:.2} samples",
            melange_validate::oversampling_round_trip_group_delay_samples(
                oversampling,
                sample_rate,
                1000.0
            )
        );
        println!("    at 1 kHz), the same alignment the 1\u{d7} run gets.");
        println!("    Tolerances are unchanged from the 1\u{d7} run.");
    }
    println!();

    // The first VALIDATE_SETTLE_PERIODS stimulus periods are left out of every
    // metric: the sine starts at t = 0 with a step in its derivative, and the
    // two engines' onset transients differ on a scale set by the stimulus.
    // Checked first: it depends only on the arguments, so a render too short to
    // compare is refused before anything about the environment is.
    let settle_time_s = VALIDATE_SETTLE_PERIODS / VALIDATE_STIMULUS_HZ;
    if settle_time_s >= duration {
        anyhow::bail!(
            "--duration {duration} s is inside the settle window ({VALIDATE_SETTLE_PERIODS} \
             periods of the {VALIDATE_STIMULUS_HZ} Hz stimulus = {settle_time_s} s), so nothing \
             would be compared; use a longer --duration"
        );
    }

    // Step 1: Check ngspice availability
    println!("Step 1: Checking ngspice...");
    if !is_ngspice_available() {
        anyhow::bail!(
            "ngspice is not installed or not found in PATH.\n\
             Install it with: sudo apt install ngspice (Debian/Ubuntu)\n\
             or: brew install ngspice (macOS)"
        );
    }
    println!("  ngspice found");

    // Step 2: Get circuit netlist as a file path
    // validate_circuit needs a file path. For local files, use directly.
    // For builtins/URLs, write to a secure temp file (random name, auto-cleanup on drop).
    // Uses tempfile::NamedTempFile to avoid TOCTOU/symlink clobber attacks from
    // predictable PID-based paths on shared hosts.
    println!("Step 2: Loading circuit...");
    use std::io::Write as _;
    let (netlist_path, _temp_file): (std::path::PathBuf, Option<tempfile::NamedTempFile>) =
        match circuit_source {
            circuits::CircuitSource::Local { path } => {
                // Verify the file exists
                if !path.exists() {
                    anyhow::bail!("Circuit file not found: {}", path.display());
                }
                (path.clone(), None)
            }
            circuits::CircuitSource::Builtin { content, name } => {
                println!("  Using builtin circuit: {}", name);
                let mut tmp = tempfile::Builder::new()
                    .prefix("melange_validate_")
                    .suffix(".cir")
                    .tempfile()
                    .context("Failed to create temp netlist file")?;
                tmp.write_all(content.as_bytes())
                    .context("Failed to write temp netlist")?;
                tmp.flush().context("Failed to flush temp netlist")?;
                let path = tmp.path().to_path_buf();
                (path, Some(tmp))
            }
            src @ (circuits::CircuitSource::Url { .. }
            | circuits::CircuitSource::Friendly { .. }) => {
                let content = fetch_remote_circuit(src)?;
                let mut tmp = tempfile::Builder::new()
                    .prefix("melange_validate_")
                    .suffix(".cir")
                    .tempfile()
                    .context("Failed to create temp netlist file")?;
                tmp.write_all(content.as_bytes())
                    .context("Failed to write temp netlist")?;
                tmp.flush().context("Failed to flush temp netlist")?;
                let path = tmp.path().to_path_buf();
                (path, Some(tmp))
            }
        };

    // Step 3: Generate test input signal (1kHz sine)
    println!(
        "Step 3: Generating test signal ({:.1}s, {:.3}V amplitude, 1kHz sine)...",
        duration, amplitude
    );
    let num_samples = (duration * sample_rate) as usize;
    let input_signal: Vec<f64> = (0..num_samples)
        .map(|i| {
            amplitude
                * (2.0 * std::f64::consts::PI * VALIDATE_STIMULUS_HZ * i as f64 / sample_rate).sin()
        })
        .collect();
    println!("  {} samples", input_signal.len());

    // Step 4: Configure comparison
    let mut config = if relaxed {
        ComparisonConfig::relaxed()
    } else {
        // Audio-grade default (see ComparisonConfig::default): correlation-
        // anchored, wide enough that a good-but-complex circuit passes. The
        // old default was strict() (0.01% RMS) — tighter than every per-circuit
        // CI tolerance, so it reported FAILED on genuinely-good circuits.
        ComparisonConfig::default()
    };
    // Apply per-metric overrides on top of the base profile. Percent inputs
    // (RMS, max-rel) are converted to the fractional form the comparator uses;
    // peak (V), correlation (0–1), and THD (dB) pass through directly.
    if let Some(pct) = tol.rms_pct {
        config.rms_error_tolerance = pct / 100.0;
    }
    config.settle_time_s = settle_time_s;
    // With no --peak-tolerance, the default profile's peak bound is relative
    // to the reference (an explicit tolerance, and --relaxed, stay absolute).
    match tol.peak_v {
        Some(v) => config.peak_error_tolerance = v,
        None if !relaxed => config.peak_error_relative = Some(VALIDATE_PEAK_RELATIVE),
        None => {}
    }
    if let Some(pct) = tol.max_rel_pct {
        config.max_relative_tolerance = pct / 100.0;
    }
    if let Some(x) = tol.corr_min {
        config.correlation_min = x;
    }
    if let Some(db) = tol.thd_db {
        config.thd_error_tolerance_db = db;
    }

    let options = ValidationOptions {
        generate_html_on_failure: false,
        generate_html_on_success: false,
        generate_csv: csv_output.is_some(),
        output_dir: csv_output.and_then(|p| p.parent().map(|d| d.to_path_buf())),
        circuit_name: Some(circuit_source.name()),
        input_node: input_node.to_string(),
        bjt_fa_mode: parse_bjt_fa_mode(reductions.bjt_fa),
        tube_grid_fa: reductions.tube_grid_fa.to_string(),
        backward_euler: reductions.backward_euler,
        force_trap: reductions.force_trap,
        oversampling,
        // The test signal's closed form: the reference is driven by it.
        analytic_stimulus: Some(melange_validate::AnalyticStimulus::Sine {
            amplitude,
            frequency: VALIDATE_STIMULUS_HZ,
        }),
        ..Default::default()
    };

    if rate_sweep {
        return run_rate_sweep(
            &netlist_path,
            amplitude,
            duration,
            sample_rate,
            output_node,
            &config,
            &options,
        );
    }

    // Step 5: Run validation
    println!("Step 4: Running validation (ngspice + melange solver)...");
    let result = validate_circuit_with_options(
        &netlist_path,
        &input_signal,
        sample_rate,
        output_node,
        &config,
        &options,
    );

    // _temp_file drops here, auto-cleaning the NamedTempFile on function exit.
    let result = result.with_context(|| "Validation failed")?;

    // Step 6: Print report
    println!();
    println!("{}", result.report.summary());

    // Step 7: Write CSV if requested
    if let Some(csv_path) = csv_output {
        // The validate library may have already written CSV if output_dir matched,
        // but if the user specified a specific path, write it explicitly
        if result.csv_path.as_ref() != Some(&csv_path.to_path_buf()) {
            // We need to reconstruct signals from the report info to write CSV.
            // Re-run would be expensive, so only rely on the library's CSV if it wrote one.
            if let Some(lib_csv) = &result.csv_path {
                // Move the library-generated CSV to the user-specified path (a
                // copy left a second full-size CSV beside it).
                std::fs::copy(lib_csv, csv_path)
                    .with_context(|| format!("Failed to copy CSV to {}", csv_path.display()))?;
                let _ = std::fs::remove_file(lib_csv);
                println!("CSV written to: {}", csv_path.display());
            }
        } else if result.csv_path.is_some() {
            println!("CSV written to: {}", csv_path.display());
        }
    }

    // Exit with error if validation failed.
    //
    // The unit-variation qualifier rides ON this line, both ways. The melange
    // side is built with `.tolerance`/`.mismatch` disabled so it compares
    // nominal against nominal (the ngspice deck has no other option); saying so
    // in a preamble would leave this line reading as a verdict on the unit the
    // deck describes, which it is not. Empty for a deck with no jitter
    // directive, which is every shipped validation deck.
    let qualifier = result.report.status_qualifier();
    if result.report.passed {
        println!("Validation PASSED{}", qualifier);
        Ok(())
    } else {
        // A near-perfect correlation next to a failed error gate is the
        // confusing case: the shapes agree, so the difference is in level,
        // offset or timing. Say which question to ask next.
        let shape_agrees = result.report.correlation_coefficient >= 0.999;
        let next_step = if shape_agrees {
            format!(
                "Correlation is {:.5}, so the two waveforms have the same shape: the difference \
                 is a gain, a DC offset, or a small time shift, not a different circuit. Rerun \
                 with --csv <file> and compare the spice_voltage and melange_voltage columns' \
                 peaks and means to see which. --relaxed loosens the error gates if that \
                 difference is acceptable for your use.",
                result.report.correlation_coefficient
            )
        } else {
            "Rerun with --csv <file> to see where the two engines part ways, or --relaxed for \
             looser tolerances."
                .to_string()
        };
        anyhow::bail!(
            "Validation FAILED{}: {} tolerance check(s) exceeded.\n{}",
            qualifier,
            result.report.failures.len(),
            next_step
        );
    }
}

/// `melange validate --rate-sweep`: the deck at `fs`, `2fs`, `4fs`, the
/// verdict, and the rates the fitted convergence needs for 1 % and 0.1 %.
/// PLATEAU and DIVERGES fail: melange converges to something other than the
/// reference there.
fn run_rate_sweep(
    netlist_path: &std::path::Path,
    amplitude: f64,
    duration: f64,
    sample_rate: f64,
    output_node: &str,
    config: &melange_validate::comparison::ComparisonConfig,
    options: &melange_validate::ValidationOptions,
) -> Result<()> {
    use melange_validate::rate_sweep::{
        rate_sweep, Fit, RateFor, SweepVerdict, Unresolved, EDGE_RATIO, PLATEAU_RATIO,
        RATIO_AGREEMENT,
    };
    println!(
        "Step 4: Rate sweep (oversampling off, the reference driven by the analytic sine at \
         {RATE_SWEEP_STIMULUS_HZ} Hz, incommensurate with the rates)..."
    );
    let sweep = rate_sweep(
        netlist_path,
        melange_validate::AnalyticStimulus::Sine {
            amplitude,
            frequency: RATE_SWEEP_STIMULUS_HZ,
        },
        duration,
        // validate's own settle window: 20 stimulus periods.
        20.0 / RATE_SWEEP_STIMULUS_HZ,
        sample_rate,
        output_node,
        config,
        options,
    )
    .with_context(|| "Rate sweep failed")?;
    println!(
        "  (error over every sample against the finest-rate reference; validate's own \
         per-rate number in brackets)"
    );
    for row in &sweep.rows {
        println!(
            "  {:>7.0} Hz  {:<40}  {:.4} %  [{:.4} % {}]",
            row.sample_rate,
            row.integrator,
            100.0 * row.error,
            100.0 * row.own_error,
            if row.passed { "PASS" } else { "FAIL" }
        );
    }
    println!(
        "  finest reference's self-check: {:.4} %{}",
        100.0 * sweep.reference_self_check,
        if sweep.reference_refined {
            " (refined to resolve the smallest error graded)"
        } else {
            ""
        }
    );
    if sweep.rows.len() > 3 {
        println!(
            "  (three rates were ambiguous: the fourth was added and the three finest decide)"
        );
    }
    let cross = match sweep.model_error_metric {
        Some(m) => format!("{:.4} % from the error metrics", 100.0 * m),
        None => "none from the error metrics (not monotone)".to_string(),
    };
    let rate = |tol: f64| match sweep.rate_for(tol) {
        RateFor::At(fs) => format!("{:.0} Hz", fs),
        RateFor::Unreachable => "not reachable (model error at or above it)".to_string(),
        RateFor::Unresolved => "not resolved".to_string(),
    };
    let fit_line = |order: f64, model_error: f64, fit: Fit| {
        let source = match fit {
            Fit::Waveform => format!("from the extrapolated waveform ({cross})"),
            Fit::ErrorMetric => "from the error metrics: the waveform fit has no answer, the \
                                 deck is not yet in its asymptotic range, so the order is not \
                                 the integrator's and the rates below are optimistic"
                .to_string(),
        };
        format!(
            "order {order:.2}, model error {:.4} % {source}: 1 % needs {}, 0.1 % needs {}",
            100.0 * model_error,
            rate(0.01),
            rate(0.001)
        )
    };
    let unresolved = |why: &Unresolved| match *why {
        Unresolved::ReferenceTooCoarse { self_check, error } => format!(
            "the finest reference's self-check ({:.4} %) is not below a third of the smallest \
             error it grades ({:.4} %), so how the error falls with the step is not measured",
            100.0 * self_check,
            100.0 * error
        ),
        Unresolved::EdgeDominated { grid, all } => format!(
            "edge-dominated: the finest render's error is {:.4} % over every sample but {:.4} % \
             at the instants the renders share (more than {EDGE_RATIO}x apart), so the error \
             lives between those instants and no fit is made on them. Per rate, unaligned \
             against validate's aligned figure: {}. On edges a sample or two wide the \
             alignment's fractional delay also ripples the reference (see SPICE_VALIDATION.md)",
            100.0 * all,
            100.0 * grid,
            sweep
                .rows
                .iter()
                .map(|r| format!(
                    "{:.0} Hz {:.4} % / {:.4} %",
                    r.sample_rate,
                    100.0 * r.error,
                    100.0 * r.own_error
                ))
                .collect::<Vec<_>>()
                .join(", ")
        ),
        Unresolved::PreAsymptotic { ratios } => format!(
            "pre-asymptotic: the error falls {:.2}x then {:.2}x per halving of the step (more \
             than {:.0} % apart), so the three finest rates do not describe one convergence",
            ratios[0],
            ratios[1],
            100.0 * RATIO_AGREEMENT
        ),
        Unresolved::FloorAmbiguous { model_error, error } => format!(
            "the error still falls (by at least {PLATEAU_RATIO}x per halving) but the fit puts \
             a floor of {:.4} % under a finest error of {:.4} %: the fits contradict each other",
            100.0 * model_error,
            100.0 * error
        ),
    };
    match sweep.verdict {
        SweepVerdict::Pass => {
            println!("Verdict: PASS at {:.0} Hz", sample_rate);
            match sweep.convergence {
                SweepVerdict::Converges {
                    order,
                    model_error,
                    fit,
                } => println!("  (converging: {})", fit_line(order, model_error, fit)),
                SweepVerdict::Unresolved(ref why) => println!("  ({})", unresolved(why)),
                _ => {}
            }
            Ok(())
        }
        SweepVerdict::Converges {
            order,
            model_error,
            fit,
        } => {
            println!("Verdict: CONVERGES ({})", fit_line(order, model_error, fit));
            Ok(())
        }
        SweepVerdict::Unresolved(ref why) => {
            anyhow::bail!("Verdict: UNRESOLVED: {}.", unresolved(why))
        }
        SweepVerdict::Plateau { order } => anyhow::bail!(
            "Verdict: PLATEAU (order {order:.2}; {cross}): the error has stopped falling with \
             the step (less than {PLATEAU_RATIO}x per halving at the finest rates), so melange \
             converges to something other than the reference. A model or harness mismatch."
        ),
        SweepVerdict::Diverges => anyhow::bail!(
            "Verdict: DIVERGES: the error against the finest reference rises with the rate. A \
             model or harness mismatch."
        ),
    }
}

struct SimulateOptions<'a> {
    input_audio: Option<&'a std::path::Path>,
    output: &'a PathBuf,
    sample_rate: f64,
    input_node: &'a str,
    output_node: &'a str,
    duration: f64,
    amplitude: f64,
    input_resistance_flag: Option<f64>,
    solver: &'a str,
    opamp_rail_mode: melange_solver::codegen::OpampRailMode,
    tube_grid_fa: &'a str,
    /// `--subsample-fire` mode (glow-strike variable-dt re-solve).
    subsample_fire: melange_solver::codegen::SubsampleFireMode,
    oversampling: Option<usize>,
    noise_mode: melange_solver::codegen::NoiseMode,
    noise_seed: u64,
    /// `--allow-unconverged-dc-op`: build even when the DC operating point did
    /// not converge.
    allow_unconverged_dc_op: bool,
    /// Test-only DC-OP Newton budget (hidden `--dc-op-max-iterations`).
    dc_op_max_iterations: Option<usize>,
    backward_euler: bool,
    force_trap: bool,
    /// Nodal sub-path override (`--nodal-subpath`). `Auto` for `simulate`,
    /// which does not expose the flag.
    nodal_sub_path_override: melange_solver::codegen::NodalSubPathOverride,
    /// Explicit `--max-iter` override; `None` → auto-tuned (see [`auto_tune_max_iter`]).
    max_iter: Option<usize>,
    /// `--allow-nr-hold`: render even when samples were never solved.
    allow_nr_hold: bool,
    /// `--allow-input-clamp`: render even when the input was clamped or NaN.
    allow_input_clamp: bool,
    probes: &'a [String],
    probe_csv: Option<&'a std::path::Path>,
    /// `--pcm16`: write the output WAV as 16-bit PCM instead of float32.
    /// File format only — the rendered samples and every reported figure are
    /// computed in f64 before encoding.
    pcm16: bool,
    /// `--pot NAME=VALUE` specs; baked into the netlist's resistor values
    /// before the MNA is built (same path `analyze` uses).
    pot_overrides: &'a [String],
    /// `--switch NAME=POS` specs; resolved and applied at runtime via
    /// `state.set_switch_N(pos)` before the run (mirrors the plugin).
    switch_overrides: &'a [String],
    /// `--inject FIELD=SPEC` specs; drive `.inject` fields from the CLI.
    inject_drives: &'a [String],
}

/// Options bundle for `melange analyze` — mirrors [`SimulateOptions`].
struct AnalyzeOptions<'a> {
    input_node: &'a str,
    output_node: &'a str,
    start_freq: f64,
    end_freq: f64,
    points_per_decade: usize,
    amplitude: f64,
    sample_rate: f64,
    input_resistance_flag: Option<f64>,
    output_file: Option<&'a PathBuf>,
    pot_overrides: &'a [String],
    switch_overrides: &'a [String],
    harmonics: usize,
    tube_grid_fa: &'a str,
    solver: &'a str,
    oversampling: Option<usize>,
    opamp_rail_mode: melange_solver::codegen::OpampRailMode,
    noise_mode: melange_solver::codegen::NoiseMode,
    noise_seed: u64,
    /// `--allow-unconverged-dc-op`: build even when the DC operating point did
    /// not converge.
    allow_unconverged_dc_op: bool,
    /// Test-only DC-OP Newton budget (hidden `--dc-op-max-iterations`).
    dc_op_max_iterations: Option<usize>,
    backward_euler: bool,
    force_trap: bool,
    /// Nodal sub-path override (`--nodal-subpath`).
    nodal_sub_path_override: melange_solver::codegen::NodalSubPathOverride,
    /// Explicit `--max-iter` override; `None` → auto-tuned (see [`auto_tune_max_iter`]).
    max_iter: Option<usize>,
    /// `--allow-input-clamp`: report even when the input was clamped or NaN.
    allow_input_clamp: bool,
}

/// Whether generated code DECLARES `field` on its state struct.
///
/// Matches the declaration, not the name: generated code can mention a counter
/// in a doc comment without declaring it (the full-LU sub-step ladder's docs
/// name `diag_nr_hold_count` on builds that have no hold), and a substring match
/// then made the simulate driver read a field that does not exist — it failed
/// to compile on every such circuit.
fn declares_state_field(code: &str, field: &str) -> bool {
    code.contains(&format!("pub {field}: "))
}

/// `CircuitState` diagnostic counters emitted only when sub-sample fire is
/// active; `simulate` prints them when the generated code declares them.
const SUBSAMPLE_FIRE_DIAG_FIELDS: [&str; 10] = [
    "diag_subsample_fire_count",
    "diag_subsample_fire_abandon_count",
    "diag_subsample_fire_detected",
    "diag_subsample_fire_resolved",
    // Reason-split of `detected - resolved` (must sum to it): a real miss is
    // ceiling+coincident; gridpoint is a bounded (<=1e-3·dt) extinction-timing
    // quantisation, expected to scale with the lit sub-step count.
    "diag_subsample_fire_unresolved_ceiling",
    "diag_subsample_fire_unresolved_gridpoint",
    "diag_subsample_fire_unresolved_coincident",
    "diag_subsample_fire_segments",
    // Memo effectiveness (a consumer's runtime check that the memo is active).
    "diag_subsample_fire_schur_builds",
    "diag_subsample_fire_schur_reuses",
];

/// Parse `--subsample-fire {auto|on|off}`. Unknown values are user errors.
fn parse_subsample_fire_mode(s: &str) -> Result<melange_solver::codegen::SubsampleFireMode> {
    melange_solver::codegen::SubsampleFireMode::parse(s).ok_or_else(|| {
        anyhow::anyhow!(
            "Unknown --subsample-fire '{}'. Valid values: auto, on, off",
            s
        )
    })
}

/// The project root `--format plugin` writes for `--output`: the path with
/// any extension dropped.
fn plugin_project_dir(output: &Path) -> PathBuf {
    if output.extension().is_some() {
        output.with_extension("")
    } else {
        output.to_path_buf()
    }
}

/// Refuse to generate a plugin project over an existing one. `src/lib.rs` is
/// the user's (README: "Yes — it's yours"), and regenerating the project
/// replaced it, and any `Cargo.toml` edits, without a word.
fn refuse_existing_plugin_project(project_dir: &Path, circuit: &str) -> Result<()> {
    let existing: Vec<&str> = ["src/lib.rs", "Cargo.toml"]
        .into_iter()
        .filter(|f| project_dir.join(f).exists())
        .collect();
    if existing.is_empty() {
        return Ok(());
    }
    anyhow::bail!(
        "refusing to overwrite the plugin project in {dir}: it already has {files}, which \
         are yours to edit, and generating the project again would replace them.\n\
         To update only the DSP, leaving your edits alone:\n  \
         melange compile {circuit} --format code -o {circuit_rs}\n\
         To start the project over, delete or move {dir} first.",
        dir = project_dir.display(),
        files = existing.join(" and "),
        circuit_rs = project_dir.join("src/circuit.rs").display(),
    )
}

/// A plausible project DIRECTORY name for the `--format plugin` hint.
///
/// Derived from the circuit, never from the `--format code` output path: that
/// path is a `.rs` FILE (`.../src/circuit.rs` in the common regenerate loop),
/// and `--format plugin` interprets `--output` as the project root. The old
/// hint echoed the file path straight back, so following it verbatim tried to
/// create a Cargo project inside another project's `src/`.
///
/// Falls back to `melange-plugin` when the source has no usable stem (a URL
/// with a trailing slash, a builtin reference, …).
fn suggest_plugin_project_dir(circuit_source_name: &str) -> String {
    let stem = std::path::Path::new(circuit_source_name)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("");
    // Builtin / source references look like "source:circuit" — take the tail.
    let stem = stem.rsplit(':').next().unwrap_or(stem);
    let cleaned: String = stem
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '-'
            }
        })
        .collect();
    let cleaned = cleaned.trim_matches('-').to_string();
    if cleaned.is_empty() {
        "melange-plugin".to_string()
    } else {
        format!("{cleaned}-plugin")
    }
}

/// Resolve `--switch NAME=POS` specs into `(switch_idx, position)` pairs.
///
/// `NAME` matches a switch label (case-insensitive) or a 0-based index; `POS`
/// is a 0-based position validated against the switch's `positions`. Shared by
/// `analyze` and `simulate`: both keep the netlist at position-0 element values
/// and apply the override at runtime via `state.set_switch_N(position)`, so the
/// emitted G/C constants stay consistent with `SwitchComponentIR.nominal_value`
/// (mutating `netlist.elements` up front would double-apply the delta).
fn resolve_switch_overrides(
    netlist: &melange_solver::parser::Netlist,
    switch_overrides: &[String],
) -> Result<Vec<(usize, usize)>> {
    let mut resolved = Vec::with_capacity(switch_overrides.len());
    for spec in switch_overrides {
        let (name, pos_str) = spec.split_once('=').ok_or_else(|| {
            anyhow::anyhow!("Invalid --switch format '{}', expected NAME=POS", spec)
        })?;
        let position: usize = pos_str.parse().map_err(|_| {
            anyhow::anyhow!("Invalid switch position '{}' in --switch {}", pos_str, spec)
        })?;

        // Match by label, by any controlled component name, or by index. Many
        // `.switch` directives carry no label (e.g. `.switch Rbyp30 1e9 1.0`),
        // so component-name matching is what makes those reachable by name
        // rather than forcing the caller to count indices.
        let switch_idx = if let Ok(idx) = name.parse::<usize>() {
            if idx >= netlist.switches.len() {
                anyhow::bail!(
                    "Switch index {} out of range (0..{})",
                    idx,
                    netlist.switches.len()
                );
            }
            idx
        } else {
            netlist
                .switches
                .iter()
                .position(|s| {
                    s.label
                        .as_deref()
                        .map(|l| l.eq_ignore_ascii_case(name))
                        .unwrap_or(false)
                        || s.component_names
                            .iter()
                            .any(|c| c.eq_ignore_ascii_case(name))
                })
                .ok_or_else(|| {
                    let available: Vec<String> = netlist
                        .switches
                        .iter()
                        .enumerate()
                        .map(|(i, s)| match s.label.as_deref() {
                            Some(l) => format!("{}: {} ({})", i, l, s.component_names.join("+")),
                            None => format!("{}: {}", i, s.component_names.join("+")),
                        })
                        .collect();
                    anyhow::anyhow!(
                        "Switch '{}' not found. Available: {}",
                        name,
                        available.join(", ")
                    )
                })?
        };

        let sw = &netlist.switches[switch_idx];
        if position >= sw.positions.len() {
            anyhow::bail!(
                "Switch '{}' position {} out of range (0..{})",
                name,
                position,
                sw.positions.len()
            );
        }

        resolved.push((switch_idx, position));
        eprintln!(
            "  Switch override: {} = position {}",
            sw.label.as_deref().unwrap_or(name),
            position
        );
    }
    Ok(resolved)
}

/// Output peak, in volts, below which `simulate` calls a rendered WAV silent.
///
/// Why a threshold rather than `peak == 0.0`: an output that lands on 1e-300 V
/// is exactly as broken as one that lands on 0.0 — the float merely happened to
/// underflow to a denormal instead of to zero — and an equality test would wave
/// it through. A floated stage, a dead bias point and a mistyped output node all
/// produce "some absurdly small number", not reliably a hard zero.
///
/// Why 1 uV specifically:
///   * melange writes volts into the WAV (1.0 == 1.0 V), so 1e-6 V is -120 dBFS.
///     Every real audio circuit's own thermal noise floor is orders of magnitude
///     above that, and no converter can reproduce it — there is no legitimately
///     quiet circuit sitting in this band, only broken ones.
///   * The generated binary reports `DIAG:peak` in scientific notation
///     (`{:.6e}`), so the real figure reaches this check however small it is —
///     the old `{:.6}` fixed format flattened everything under 5e-7 V to
///     `0.000000`, which made the threshold unmeasurable from here and the
///     warning unable to quote anything but zero.
///
/// A circuit that is quiet but working (a deep attenuator, a fader near the
/// bottom, a stage biased into cutoff) still lands far above this, so the
/// warning does not nag correct circuits. Genuinely intentional silence — a
/// zero-amplitude drive — is filtered out at the call site instead.
const SILENT_OUTPUT_PEAK_V: f64 = 1e-6;

/// True when a rendered output peak counts as digital silence.
///
/// NaN is deliberately NOT silence: a NaN peak means the solve blew up, which
/// the NaN/magnitude-reset counters report, and it must not be described to the
/// user as "nothing to hear".
fn output_is_silent(peak: f64) -> bool {
    peak.is_finite() && peak.abs() < SILENT_OUTPUT_PEAK_V
}

/// Build the warning shown when `simulate` renders digital silence.
///
/// `max_abs_node_v` is the run's largest internal node voltage (`DIAG:
/// max_abs_v_prev`); it splits the two very different failures the user cannot
/// otherwise tell apart — "the whole circuit is dead" versus "the circuit is
/// alive but the output tap is not connected to it".
fn silent_output_warning(
    peak: f64,
    output_node: &str,
    samples: Option<u64>,
    max_abs_node_v: Option<f64>,
) -> String {
    let sample_count = samples.map_or_else(|| "every".to_string(), |n| format!("all {n}"));
    let mut msg = format!(
        "WARNING: the rendered output is digital silence \u{2014} peak {peak:.6e} V across \
         {sample_count} samples of node '{output_node}', below the {SILENT_OUTPUT_PEAK_V:e} V \
         floor melange treats as silence. The run completed and the WAV was written, but there \
         is nothing in the file to hear.\n"
    );
    msg.push_str("  Nothing else in this report flags that, so check, roughly in order:\n");
    msg.push_str(
        "    1. A mistyped node name the topology check could not see. A typo that leaves a\n\
         \x20      node on one resistor or cap is refused, but one on a transistor, tube or\n\
         \x20      op-amp terminal only warns (scroll up), and one that happens to spell\n\
         \x20      another real node is invisible. Check every node name in the netlist\n\
         \x20      against `melange nodes <circuit>`.\n",
    );
    msg.push_str(
        "    2. A node with no DC path to ground (one reachable only through capacitors): that\n\
         \x20      section cannot bias, so no signal crosses it.\n",
    );
    msg.push_str(&format!(
        "    3. `--output-node {output_node}` is not where this circuit's signal actually comes\n\
         \x20      out; `melange nodes <circuit>` lists the node names.\n"
    ));
    msg.push_str(
        "    4. The input never reaches the output: a missing coupling cap, a pot or switch\n\
         \x20      sitting at its zero position, a supply rail that was never connected, or a\n\
         \x20      SPICE-style stimulus source left in the deck that still clamps the input\n\
         \x20      (melange drives the input node itself; the deck needs no source for it).\n",
    );
    match max_abs_node_v {
        Some(v) if v.is_finite() && v.abs() < SILENT_OUTPUT_PEAK_V => msg.push_str(
            "  Every internal node also stayed at 0 V for the whole run, so nothing in the\n\
             \x20 circuit moved at all \u{2014} suspect the drive or the supply (check the input node\n\
             \x20 and any DC sources) before the output tap.\n",
        ),
        Some(v) if v.is_finite() => msg.push_str(&format!(
            "  Internal nodes did move (max |v| = {v:.3} V), so at least part of the circuit is\n\
             \x20 live \u{2014} the break is between that live part and '{output_node}'. If the figure is\n\
             \x20 just the drive amplitude, nothing past the input node is moving at all.\n"
        )),
        _ => {}
    }
    msg.push_str(
        "  This is a warning, not an error: the file was written and the exit status is\n\
         \x20 unchanged. If you meant to render a muted state, ignore it.",
    );
    msg
}

fn simulate_circuit_source(
    circuit_source: &circuits::CircuitSource,
    opts: &SimulateOptions,
) -> Result<()> {
    println!("melange simulate");
    println!("  Source: {}", circuit_source.name());
    println!();

    // Step 1: Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, name } => {
            println!("  Using builtin circuit: {}", name);
            content.clone()
        }
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            println!("  Fetching: {}", url);
            cache::Cache::new()?.get_sync(url, false)?
        }
    };

    // The one build every verb ships (melange_solver::build). Output layout:
    // [primary, probe_1, probe_2, ...]. The generated `process_sample` returns
    // these in order; the simulate main routes index 0 to the WAV and 1.. to the
    // probe CSV.
    let mut output_nodes = vec![opts.output_node.to_string()];
    output_nodes.extend(opts.probes.iter().cloned());
    let build_opts = melange_solver::build::BuildOptions {
        sample_rate: opts.sample_rate,
        circuit_name: "simulate".to_string(),
        input_nodes: vec![opts.input_node.to_string()],
        output_nodes,
        max_iter: opts.max_iter,
        tolerance: 1e-9,
        output_scale: 1.0,
        output_clamp: 10.0,
        input_resistance: opts.input_resistance_flag,
        oversampling: opts.oversampling,
        dc_block: false, // preserve DC for accurate WAV output
        solver: opts.solver.to_string(),
        backward_euler: opts.backward_euler,
        force_trap: opts.force_trap,
        tube_grid_fa: opts.tube_grid_fa.to_string(),
        subsample_fire: opts.subsample_fire,
        subsample_lit_factor: diag_lit_factor(),
        bjt_fa_mode: melange_solver::codegen::BjtFaMode::Off,
        opamp_rail_mode: opts.opamp_rail_mode,
        nodal_sub_path_override: opts.nodal_sub_path_override,
        allow_static_glow_on_full_lu: false,
        noise_mode: opts.noise_mode,
        noise_seed: opts.noise_seed,
        emit_dc_op_recompute: false,
        plugin_format: false,
        // Knob settings mean the same thing here as in `analyze`.
        pot_overrides: Some(opts.pot_overrides.to_vec()),
        resolve_taps: false,
        inject_runtime: true,
        disable_unit_variation: false,
        disable_self_heating: false,
        allow_unconverged_dc_op: opts.allow_unconverged_dc_op,
        dc_op_max_iterations: opts.dc_op_max_iterations,
        output_clamp_auto: false,
    };
    let built =
        melange_solver::build::build(&netlist_str, &build_opts, &|a| println!("{a}"), &|a| {
            eprintln!("{a}")
        })
        .map_err(build_error)?;
    println!(
        "{}",
        format_route_info(built.solver_label, &built.solver_reason)
    );
    // Non-negative K diagonal note, printed ONCE (kernel builder logs it at
    // debug only — it is rebuilt several times per run). See compile summary.
    if built.routing.k_diag_unsafe {
        println!(
            "  Note: non-negative K diagonal (positive DK-Schur feedback, \
             expected for transformer-coupled NFB) — handled by nodal full-NR."
        );
    }
    if built.max_iter != 100 {
        println!("  Max NR iterations: {}", built.max_iter);
    }
    let injection_specs = built.injection_specs;
    let generated = built.generated;
    let netlist = built.netlist;
    let oversampling = built.oversampling;

    // Map `--inject FIELD=SPEC` to injection indices (by field name).
    let mut inject_driven: Vec<(usize, codegen_runner::InjectSource)> = Vec::new();
    for drive in opts.inject_drives {
        let (field, source) = codegen_runner::parse_inject_drive(drive)?;
        let idx = injection_specs
            .iter()
            .position(|s| s.name.eq_ignore_ascii_case(&field))
            .ok_or_else(|| {
                let names: Vec<&str> = injection_specs.iter().map(|s| s.name.as_str()).collect();
                anyhow::anyhow!(
                    "--inject field '{}' is not a `.inject` field in this deck. Available: {:?}",
                    field,
                    names
                )
            })?;
        inject_driven.push((idx, source));
    }
    // Warn on any `.inject` field left undriven (it injects 0 V) — only once the
    // user has supplied at least one `--inject`, so an unrelated run stays quiet.
    if !opts.inject_drives.is_empty() {
        for (si, spec) in injection_specs.iter().enumerate() {
            if !inject_driven.iter().any(|(i, _)| *i == si) {
                println!(
                    "warning: `.inject` field '{}' has no --inject drive; it will inject 0 V",
                    spec.name
                );
            }
        }
    }

    println!("  {} lines of code", generated.code.lines().count());

    // Step 6: Append simulate main, compile, run.
    // Probe names map 1:1 to `output_nodes[1..]`. The generated main uses
    // them for the CSV header; the runtime argv[3] supplies the CSV path.
    println!("Step 5: Compiling and running...");
    let probe_names: Vec<&str> = opts.probes.iter().map(|s| s.as_str()).collect();
    // Resolve --switch NAME=POS and apply at runtime via set_switch_N (same as
    // the generated plugin / `analyze`): keeps the netlist at position-0 values
    // so the emitted G/C constants stay consistent with the delta-from-nominal
    // stamp in set_switch_N.
    let switch_runtime_overrides = resolve_switch_overrides(&netlist, opts.switch_overrides)?;
    let switch_calls: Vec<String> = switch_runtime_overrides
        .iter()
        .map(|(idx, pos)| format!("state.set_switch_{idx}({pos})"))
        .collect();
    let extra_diag_counters = SUBSAMPLE_FIRE_DIAG_FIELDS
        .iter()
        .copied()
        // The unsolved-sample counters exist only where their mechanism
        // does (the hold on nodal builds with a Newton solve; the committed
        // count on DK and wherever a failed op-amp pin is committed).
        // Presence-filtered like the rest, so a build stays silent rather
        // than reporting a reassuring zero for a mechanism it does not have.
        .chain([
            "diag_unsolved_sample_count",
            "diag_nr_hold_count",
            "diag_nr_unconverged_commit_count",
            "diag_warm_start_fallback_count",
            "diag_reduced_model_exit_count",
        ])
        .chain(INPUT_DIAG_FIELDS)
        .filter(|f| declares_state_field(&generated.code, f))
        .collect::<Vec<&str>>();
    let simulate_main = codegen_runner::generate_simulate_main(codegen_runner::SimulateMain {
        sample_rate: opts.sample_rate,
        // `--pot` is baked into the netlist's R values before the MNA is built
        // (see `apply_pot_overrides`), so there is nothing to set at runtime.
        pot_calls: &[],
        switch_calls: &switch_calls,
        amplitude: if opts.input_audio.is_none() {
            Some(opts.amplitude)
        } else {
            None
        },
        freq: 1000.0, // test tone freq
        duration_secs: opts.duration,
        probe_names: &probe_names,
        noise_enabled: opts.noise_mode != melange_solver::codegen::NoiseMode::Off,
        inject_driven: &inject_driven,
        num_inject: injection_specs.len(),
        // Build-conditional counters: only present in the generated state when
        // the emitter resolved the feature active (auto = glow on nodal-Schur).
        extra_diag_counters: &extra_diag_counters,
        pcm16: opts.pcm16,
    });
    let full_source = format!("{}\n{}", generated.code, simulate_main);

    let binary_cache =
        codegen_runner::BinaryCache::new().with_context(|| "Failed to create binary cache")?;
    let compiled = binary_cache
        .compile(&full_source, "simulate")
        .with_context(|| "Compilation failed")?;
    if compiled.cached {
        println!("  Using cached binary");
    } else {
        println!("  Compiled successfully");
    }

    // Run the binary. argv layout:
    //   [1] input.wav | "--tone"
    //   [2] output.wav
    //   [3] probes.csv  (only present when probes were baked in)
    let mut cmd = std::process::Command::new(&compiled.path);
    if let Some(audio_path) = opts.input_audio {
        cmd.arg(audio_path.to_str().unwrap_or("input.wav"));
    } else {
        cmd.arg("--tone");
    }
    cmd.arg(opts.output.to_str().unwrap_or("output.wav"));
    if let Some(csv_path) = opts.probe_csv {
        cmd.arg(csv_path.to_str().unwrap_or("probes.csv"));
    }

    let result = cmd
        .output()
        .with_context(|| "Failed to run compiled binary")?;

    if !result.status.success() {
        let stderr = String::from_utf8_lossy(&result.stderr);
        anyhow::bail!("Simulate binary failed:\n{}", stderr);
    }

    // Parse diagnostics from stderr
    let stderr = String::from_utf8_lossy(&result.stderr);
    let mut nr_max_iter_count: Option<u64> = None;
    let mut diag_samples: Option<u64> = None;
    // Peak (V) and the largest node voltage seen anywhere in the circuit. Both
    // are already computed by the generated binary; the silent-output check
    // below reads them here rather than re-scanning the rendered buffer.
    let mut diag_peak: Option<f64> = None;
    let mut diag_max_abs_v_prev: Option<f64> = None;
    // The two unsolved-sample counters: the death-spiral hold, and a failed
    // op-amp pin (or a DK Newton) whose iterate was committed. A nodal build
    // with an active-set pin declares both, so they are kept apart.
    let mut nr_hold_count: Option<u64> = None;
    let mut nr_commit_count: Option<u64> = None;
    let mut reduced_exit_count: Option<u64> = None;
    // The unified count every build declares; it is what the verb refuses on.
    let mut unsolved_count: Option<u64> = None;
    // Counters that mean the solver had to WORK, not that anything is wrong.
    // Printed as a bare list they read as a hazard panel a newcomer cannot
    // interpret: is `region_exit_count: 0` good? is 5 bad? Nothing said.
    let mut recoveries: u64 = 0;
    let mut resets: u64 = 0;
    let mut printed_header = false;
    for line in stderr.lines() {
        if let Some(diag) = line.strip_prefix("DIAG:") {
            let parts: Vec<&str> = diag.splitn(2, '=').collect();
            if parts.len() == 2 {
                if !printed_header {
                    println!("  Solver diagnostics:");
                    printed_header = true;
                }
                let v: u64 = parts[1].trim().parse().unwrap_or(0);
                match parts[0] {
                    "substep_count" | "be_fallback_count" => recoveries += v,
                    "nan_reset_count" | "magnitude_reset_count" => resets += v,
                    _ => {}
                }
                println!("    {}: {}", parts[0], parts[1]);
                match parts[0] {
                    "nr_max_iter_count" => nr_max_iter_count = parts[1].trim().parse().ok(),
                    // Both mean "not a solution"; see nr_commit_count above.
                    "unsolved_sample_count" => unsolved_count = parts[1].trim().parse().ok(),
                    "nr_hold_count" => nr_hold_count = parts[1].trim().parse().ok(),
                    "nr_unconverged_commit_count" => nr_commit_count = parts[1].trim().parse().ok(),
                    "reduced_model_exit_count" => reduced_exit_count = parts[1].trim().parse().ok(),
                    "samples" => diag_samples = parts[1].trim().parse().ok(),
                    "peak" => diag_peak = parts[1].trim().parse().ok(),
                    "max_abs_v_prev" => diag_max_abs_v_prev = parts[1].trim().parse().ok(),
                    _ => {}
                }
            }
        }
    }

    // One line saying what the block above amounts to. The counters are
    // meaningful to a maintainer and opaque to everyone else, and a list of
    // numbers with no verdict trains people to skip it.
    //
    // `recoveries` counts retries RUN, not samples they saved: when any sample
    // stayed unsolved the line says so, rather than reassuring above the ERROR.
    let held = nr_hold_count.unwrap_or(0);
    let committed = nr_commit_count.unwrap_or(0);
    let total = unsolved_count.unwrap_or(held + committed);
    if printed_header {
        let capped = nr_max_iter_count.unwrap_or(0);
        if resets > 0 {
            println!(
                "    -> {resets} NaN/magnitude reset(s): the solve blew up and was reset. \
                 Treat this output as suspect."
            );
        } else if total > 0 {
            println!(
                "    -> {capped} sample(s) hit the iteration ceiling; the sub-step and \
                 backward-Euler retries ran {recoveries} time(s) and {total} sample(s) were \
                 still never solved (see the ERROR below)."
            );
        } else if capped > 0 && recoveries > 0 {
            println!(
                "    -> {capped} sample(s) hit the iteration ceiling and a sub-step or \
                 backward-Euler retry solved each of them. Normal on hard transients."
            );
        } else if capped > 0 {
            println!("    -> {capped} sample(s) hit the iteration ceiling.");
        } else {
            println!("    -> nothing to flag: no iteration-ceiling hits, no resets.");
        }
    }

    // Validity warning: if Newton-Raphson hit its iteration ceiling on a large
    // fraction of (internal-rate) samples, the solver never actually converged
    // there. A starved solve can latch every node at a DC-ish value that looks
    // exactly like a physical steady state while being numerically meaningless
    // — and nothing else in the normal output flags it as fatal-to-validity.
    // Surface it loudly. (Raised by melange-circuits 2026-08-15: a Ge astable
    // cascade starved at --max-iter 70 latched every node and read as real
    // physics for half a day; --max-iter 1000 converged.)
    //
    // Threshold is 20%, not the originally-suggested 5%: a healthy run of that
    // same circuit still shows ~5% onset-only max-iter samples (BE fallback
    // rescues them), so 5% would cry wolf. 20% cleanly separates onset
    // transients from a systematic latch (~100% in the failing case).
    if let (Some(nr_fail), Some(samples)) = (nr_max_iter_count, diag_samples) {
        let internal_samples = samples.saturating_mul(oversampling as u64).max(1);
        let frac = nr_fail as f64 / internal_samples as f64;
        if frac > 0.20 {
            let max_iter_disp = opts
                .max_iter
                .map_or_else(|| "default".to_string(), |m| m.to_string());
            eprintln!(
                "WARNING: Newton-Raphson hit its iteration ceiling {} times across {} internal \
                 samples ({:.0}%; a sample can fail both its trapezoidal and BE-fallback solve, so \
                 this can exceed 100%). The solver is failing to converge on a large fraction of \
                 samples — the output can latch at a DC-ish value that looks like a physical steady \
                 state while being numerically meaningless. Verify the result. A larger \
                 --max-iter (current {}) can let a slow but non-regenerative solve converge; on an \
                 oscillator or switching circuit it can instead let Newton settle on a spurious \
                 oscillation of the discrete step equations, with no unsolved sample to show it, \
                 so it is not a supported way past unsolved samples there (docs/limitations.md, \
                 \"Self-starting two-transistor astables\").",
                nr_fail,
                internal_samples,
                frac * 100.0,
                max_iter_disp,
            );
        }
    }

    println!();
    println!("Output written to: {}", opts.output.display());
    if let Some(csv_path) = opts.probe_csv {
        println!("Probes written to: {}", csv_path.display());
    }

    // Silent-output check. melange already computed the peak; a run that
    // renders digital silence must not slip out the door looking healthy just
    // because every counter is clean and the exit code is 0. See
    // `output_is_silent` for the threshold rationale.
    //
    // Deliberate silence is not a defect: a zero-amplitude test tone with noise
    // off renders a silent file BY REQUEST, so that combination stays quiet.
    let silence_requested = opts.input_audio.is_none()
        && opts.amplitude == 0.0
        && opts.noise_mode == melange_solver::codegen::NoiseMode::Off;
    if let Some(peak) = diag_peak {
        if output_is_silent(peak) && !silence_requested {
            eprintln!();
            eprintln!(
                "{}",
                silent_output_warning(peak, opts.output_node, diag_samples, diag_max_abs_v_prev)
            );
        }
    }

    // The death-spiral hold: samples where every Newton path failed and the
    // previous state was committed as the answer. This is NOT the iteration
    // ceiling warned about above — a capped sample that a fallback rescued is a
    // converged solution by another consistent scheme. A held sample is not a
    // solution at all, and nothing about the rendered audio can tell you: it is
    // bounded and smooth, so peak, RMS and the waveform all read healthy.
    //
    // Fail, do not warn. The whole failure mode is that it looks fine
    // (design review).
    if total > 0 {
        let of = diag_samples
            .map(|s| format!(" of {s} output samples"))
            .unwrap_or_default();
        if held > 0 {
            eprintln!();
            eprintln!(
                "ERROR: {held} sample(s){of} were never solved. Every Newton path failed there \
                 (the solve, the sub-step and, on a trapezoidal build, backward Euler), so the \
                 solver committed the PREVIOUS state as the output. Those samples are not a \
                 solution to this circuit.\n\
                 \n\
                 The rendered file looks healthy — a held value is bounded and smooth, so peak, \
                 RMS and the waveform cannot show it. Under a held input the hold is also a fixed \
                 point: the next sample re-poses the identical problem and fails identically, so \
                 one hard sample can freeze the render to its end."
            );
        }
        let reduced = reduced_exit_count.unwrap_or(0);
        if reduced > 0 {
            eprintln!();
            eprintln!(
                "ERROR: {reduced} sample(s){of} were solved on a REDUCED device model outside \
                 its region: a `.linearize`d device driven out of its small-signal region (a \
                 triode cut off or its grid past the conduction onset, a BJT cut off or \
                 saturated), a forward-active BJT that saturated, or a grid-off pentode whose \
                 grid conducted. The reduction assumes the device never goes there, so those \
                 samples are not a solution to this circuit. Remove `.linearize` for a stage \
                 that leaves its region at this drive, or lower the drive; for the other two, \
                 rebuild without the reduction (--bjt-fa off / --tube-grid-fa off)."
            );
        }
        if committed > 0 {
            eprintln!();
            eprintln!(
                "ERROR: {committed} sample(s){of} were never solved. The final Newton solve (an \
                 op-amp rail pin, or the DK solve) ended unconverged, and the solver committed \
                 that iterate as the output. Those samples are not a solution to this circuit, \
                 and a bounded, smooth render does not show it."
            );
        }
        eprintln!(
            "\nThe WAV was still written, so you can listen to what it did. Do not treat it as \
             this circuit's output. Re-run with --allow-nr-hold to accept it anyway."
        );
        if !opts.allow_nr_hold {
            anyhow::bail!("{total} sample(s) were never solved (--allow-nr-hold to override)");
        }
        eprintln!("(--allow-nr-hold given: continuing.)");
    }
    refuse_on_input_diag(&stderr, opts.allow_input_clamp)?;
    Ok(())
}

/// The generated code's input-sanitisation counters, printed as `DIAG:` lines
/// by the simulate and analyze harnesses (presence-filtered per build).
const INPUT_DIAG_FIELDS: [&str; 2] = ["diag_input_clamp_count", "diag_input_nan_count"];

/// Fail when the circuit was not driven with the requested input: samples
/// clamped to `INPUT_LIMIT_V`, or NaN/Inf samples replaced by 0. Like an NR
/// hold, the output then looks healthy while answering a different question
/// (design review). `allow` is the explicit override.
fn refuse_on_input_diag(stderr: &str, allow: bool) -> Result<()> {
    let count = |key: &str| -> u64 {
        stderr
            .lines()
            .filter_map(|l| l.strip_prefix("DIAG:"))
            .filter_map(|d| d.strip_prefix(key))
            .filter_map(|v| v.strip_prefix('='))
            .filter_map(|v| v.trim().parse::<u64>().ok())
            .next_back()
            .unwrap_or(0)
    };
    let (clamped, nan) = (count("input_clamp_count"), count("input_nan_count"));
    if clamped == 0 && nan == 0 {
        return Ok(());
    }
    eprintln!();
    if clamped > 0 {
        eprintln!(
            "ERROR: {clamped} input sample(s) exceeded the generated code's input limit \
             (INPUT_LIMIT_V = 100 V) and were clamped to it. The circuit was driven with a \
             clipped input, not the one requested, and nothing in the output shows it."
        );
    }
    if nan > 0 {
        eprintln!("ERROR: {nan} input sample(s) were NaN or infinite and were replaced by 0 V.");
    }
    if !allow {
        anyhow::bail!(
            "the circuit was not driven with the requested input ({clamped} clamped, {nan} \
             NaN/Inf; --allow-input-clamp to override)"
        );
    }
    eprintln!("(--allow-input-clamp given: continuing.)");
    Ok(())
}

// `LinearizeOutcome`, `apply_linearize_reductions` and `auto_tune_max_iter`
// moved to `melange_solver::pipeline` so `melange compile`, `simulate`,
// `analyze` and `melange-validate` share one front-end pipeline instead of
// four copies that drifted. See that module's docs for what the drift cost.
fn analyze_freq_response(
    circuit_source: &circuits::CircuitSource,
    opts: &AnalyzeOptions<'_>,
) -> Result<()> {
    let AnalyzeOptions {
        nodal_sub_path_override,
        input_node: input_node_name,
        output_node: output_node_name,
        start_freq,
        end_freq,
        points_per_decade,
        amplitude,
        sample_rate,
        input_resistance_flag,
        output_file,
        pot_overrides,
        switch_overrides,
        harmonics,
        tube_grid_fa,
        solver,
        oversampling: oversampling_cli,
        opamp_rail_mode,
        noise_mode,
        noise_seed,
        allow_unconverged_dc_op,
        dc_op_max_iterations,
        backward_euler,
        force_trap,
        max_iter,
        allow_input_clamp,
    } = *opts;
    // Match parse-time node normalization (lowercase, gnd→0).
    let input_node_owned = melange_solver::parser::normalize_node_name(input_node_name);
    let input_node_name = input_node_owned.as_str();
    let output_node_owned = melange_solver::parser::normalize_node_name(output_node_name);
    let output_node_name = output_node_owned.as_str();

    eprintln!("melange analyze (frequency response)");

    // Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, name } => {
            eprintln!("  Using builtin circuit: {}", name);
            content.clone()
        }
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            cache::Cache::new()?.get_sync(url, false)?
        }
    };

    // The one build every verb ships (melange_solver::build). Analyze writes
    // its CSV to stdout, so every build line goes to stderr.
    let build_opts = melange_solver::build::BuildOptions {
        sample_rate,
        circuit_name: "analyze".to_string(),
        input_nodes: vec![input_node_name.to_string()],
        output_nodes: vec![output_node_name.to_string()],
        max_iter,
        tolerance: 1e-9,
        output_scale: 1.0,
        output_clamp: 10.0,
        input_resistance: input_resistance_flag,
        oversampling: oversampling_cli,
        dc_block: false,
        solver: solver.to_string(),
        backward_euler,
        force_trap,
        tube_grid_fa: tube_grid_fa.to_string(),
        // `analyze` does not expose --subsample-fire; auto = active on glow
        // nodal-Schur decks, inert everywhere else.
        subsample_fire: melange_solver::codegen::SubsampleFireMode::Auto,
        subsample_lit_factor: diag_lit_factor(),
        bjt_fa_mode: melange_solver::codegen::BjtFaMode::Off,
        opamp_rail_mode,
        nodal_sub_path_override,
        allow_static_glow_on_full_lu: false,
        noise_mode,
        noise_seed,
        emit_dc_op_recompute: false,
        plugin_format: false,
        // One knob setting means the same thing here as in `simulate`.
        pot_overrides: Some(pot_overrides.to_vec()),
        resolve_taps: false,
        inject_runtime: false,
        disable_unit_variation: false,
        disable_self_heating: false,
        allow_unconverged_dc_op,
        dc_op_max_iterations,
        output_clamp_auto: false,
    };
    let built =
        melange_solver::build::build(&netlist_str, &build_opts, &|a| eprintln!("{a}"), &|a| {
            eprintln!("{a}")
        })
        .map_err(build_error)?;
    eprintln!(
        "{}",
        format_route_info(built.solver_label, &built.solver_reason)
    );
    if built.max_iter != 100 {
        eprintln!("  Max NR iterations: {}", built.max_iter);
    }
    let has_inductors = !built.mna.inductors.is_empty()
        || !built.mna.coupled_inductors.is_empty()
        || !built.mna.transformer_groups.is_empty();
    let generated = built.generated;
    let netlist = built.netlist;
    let oversampling = built.oversampling;

    // Keep the netlist at position-0 element values and apply overrides at
    // runtime via `state.set_switch_N(position)` (see `switch_calls` below).
    let switch_runtime_overrides = resolve_switch_overrides(&netlist, switch_overrides)?;

    // Generate frequency list
    let frequencies = generate_log_frequencies(start_freq, end_freq, points_per_decade);
    eprintln!(
        "  {} frequency points from {:.0} Hz to {:.0} Hz",
        frequencies.len(),
        start_freq,
        end_freq
    );
    // A constant delay is a phase slope, and the half-band filters add one:
    // tens of degrees by a few kHz. Someone comparing this sweep against a 1×
    // one sees phase_deg move a long way and reasonably concludes the circuit's
    // response changed. It did not — gain is unaffected. Said where the number
    // is produced, not only in a doc they may not have read.
    if oversampling > 1 {
        let added = if oversampling >= 4 { 3.47 } else { 2.65 };
        eprintln!(
            "  NOTE: at {}× the phase column includes ~{:.2} host samples of half-band",
            oversampling, added
        );
        eprintln!(
            "        filter delay (~{:.0}° at 3 kHz). Gain is unaffected; the circuit's",
            360.0 * 3000.0 * added / sample_rate
        );
        eprintln!("        response has not changed. See docs/OVERSAMPLING.md.");
    }

    // Determine settle time
    let settle_secs = if has_inductors { 5.0 } else { 0.5 };

    // Append analyze main, compile, run.
    //
    // Switch overrides are applied at runtime via `state.set_switch_N(position)`
    // before the settle loop; the settle loop then converges NR (if M>0) on
    // the new topology. Pot overrides remain netlist-baked (R-only, flows
    // into G at codegen time).
    let switch_calls: Vec<String> = switch_runtime_overrides
        .iter()
        .map(|(idx, pos)| format!("state.set_switch_{idx}({pos})"))
        .collect();
    let analyze_main = codegen_runner::generate_analyze_main(
        &frequencies,
        amplitude,
        sample_rate,
        settle_secs,
        &[], // pot calls (already baked into netlist)
        &switch_calls,
        harmonics,
        noise_mode != melange_solver::codegen::NoiseMode::Off,
        &INPUT_DIAG_FIELDS
            .iter()
            .copied()
            .filter(|f| declares_state_field(&generated.code, f))
            .collect::<Vec<&str>>(),
    );
    let full_source = format!("{}\n{}", generated.code, analyze_main);

    let binary_cache =
        codegen_runner::BinaryCache::new().with_context(|| "Failed to create binary cache")?;
    let compiled = binary_cache
        .compile(&full_source, "analyze")
        .with_context(|| "Compilation failed")?;
    if compiled.cached {
        eprintln!("  Using cached binary");
    }

    let result = std::process::Command::new(&compiled.path)
        .output()
        .with_context(|| "Failed to run analyze binary")?;

    if !result.status.success() {
        let stderr = String::from_utf8_lossy(&result.stderr);
        anyhow::bail!("Analyze binary failed:\n{}", stderr);
    }

    let stdout = String::from_utf8_lossy(&result.stdout);

    // Print per-frequency diagnostics from stderr
    let stderr = String::from_utf8_lossy(&result.stderr);
    for line in stderr.lines() {
        eprintln!("{}", line);
    }
    refuse_on_input_diag(&stderr, allow_input_clamp)?;

    // Output CSV.
    //
    // The two streams are already separated: every progress/diagnostic line
    // above is stderr, and stdout carries nothing but the CSV, so
    // `melange analyze ... > response.csv` yields a clean file with no
    // hand-stripping. On a terminal both land in the same scrollback and it
    // looks like one interleaved dump, which is what makes people reach for a
    // separator flag — so say where the CSV went instead.
    if let Some(out_path) = output_file {
        std::fs::write(out_path, stdout.as_bytes())
            .with_context(|| format!("Failed to write: {}", out_path.display()))?;
        eprintln!("  Results written to: {}", out_path.display());
    } else {
        eprintln!(
            "  {} CSV rows follow on stdout (everything above is stderr). \
             Capture them with `-o FILE`, or redirect: `melange analyze ... > response.csv`.",
            stdout.lines().count().saturating_sub(1)
        );
        print!("{}", stdout);
    }

    Ok(())
}

fn generate_log_frequencies(start: f64, end: f64, points_per_decade: usize) -> Vec<f64> {
    let log_start = start.log10();
    let log_end = end.log10();
    let decades = log_end - log_start;
    let total_points = (decades * points_per_decade as f64).ceil() as usize + 1;
    (0..total_points)
        .map(|i| {
            let log_f = log_start + i as f64 * decades / (total_points - 1).max(1) as f64;
            10.0_f64.powf(log_f)
        })
        .collect()
}

// build_device_entries and build_device_slots removed — runtime solvers replaced by codegen.
// write_wav removed — WAV I/O is now handled by compiled circuit binaries.
#[cfg(any())]
/// Build DeviceEntry list for CircuitSolver from netlist + MNA.
fn build_device_entries(
    netlist: &melange_solver::parser::Netlist,
    mna: &melange_solver::mna::MnaSystem,
) -> Vec<melange_solver::solver::DeviceEntry> {
    use melange_devices::bjt::{BjtEbersMoll, BjtPolarity};
    use melange_devices::diode::DiodeShockley;
    use melange_devices::jfet::{Jfet, JfetChannel};
    use melange_devices::mosfet::{ChannelType, Mosfet};
    use melange_devices::tube::KorenTriode;
    use melange_solver::solver::DeviceEntry;

    let find_model_param = |model_name: &str, param: &str| -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param))
                    .map(|(_, v)| *v)
            })
    };

    let mut devices = Vec::new();
    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Diode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-15);
                let n = find_model_param(model_name, "N").unwrap_or(1.0);
                devices.push(DeviceEntry::new_diode(
                    DiodeShockley::new_room_temp(is, n),
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-14);
                let bf = find_model_param(model_name, "BF").unwrap_or(200.0);
                let br = find_model_param(model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                let polarity = if is_pnp {
                    BjtPolarity::Pnp
                } else {
                    BjtPolarity::Npn
                };
                let nf = find_model_param(model_name, "NF").unwrap_or(1.0);
                devices.push(DeviceEntry::new_bjt(
                    BjtEbersMoll::new(is, melange_primitives::VT_ROOM, bf, br, polarity)
                        .with_nf(nf),
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Jfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Jfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
                    .unwrap_or(false);
                let channel = if is_p_channel {
                    JfetChannel::P
                } else {
                    JfetChannel::N
                };
                let default_vp = if is_p_channel { 2.0 } else { -2.0 };
                let vp = find_model_param(model_name, "VTO").unwrap_or(default_vp);
                let idss = if let Some(beta) = find_model_param(model_name, "BETA") {
                    beta * vp * vp
                } else {
                    find_model_param(model_name, "IDSS").unwrap_or(2e-3)
                };
                let mut jfet = Jfet::new(channel, vp, idss);
                jfet.lambda = find_model_param(model_name, "LAMBDA").unwrap_or(0.001);
                devices.push(DeviceEntry::new_jfet(jfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Mosfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Mosfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PM"))
                    .unwrap_or(false);
                let channel = if is_p_channel {
                    ChannelType::P
                } else {
                    ChannelType::N
                };
                let default_vt = if is_p_channel { -2.0 } else { 2.0 };
                let vt = find_model_param(model_name, "VTO").unwrap_or(default_vt);
                let kp = find_model_param(model_name, "KP").unwrap_or(0.1);
                let lambda = find_model_param(model_name, "LAMBDA").unwrap_or(0.01);
                let mosfet = Mosfet::new(channel, vt, kp, lambda);
                devices.push(DeviceEntry::new_mosfet(mosfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Tube => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Triode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let mu = find_model_param(model_name, "MU").unwrap_or(100.0);
                let ex = find_model_param(model_name, "EX").unwrap_or(1.4);
                let kg1 = find_model_param(model_name, "KG1").unwrap_or(1060.0);
                let kp = find_model_param(model_name, "KP").unwrap_or(600.0);
                let kvb = find_model_param(model_name, "KVB").unwrap_or(300.0);
                // D&Z eq. (11) grid law; IG_MAX/VGK_ONSET are retired keys and
                // a deck carrying either is refused before reaching here.
                let gg =
                    find_model_param(model_name, "GG").unwrap_or(melange_devices::tube::DEFAULT_GG);
                let xi =
                    find_model_param(model_name, "XI").unwrap_or(melange_devices::tube::DEFAULT_XI);
                let cg =
                    find_model_param(model_name, "CG").unwrap_or(melange_devices::tube::DEFAULT_CG);
                let lambda = find_model_param(model_name, "LAMBDA").unwrap_or(0.0);
                let tube = KorenTriode::with_all_params(mu, ex, kg1, kp, kvb, gg, xi, cg, lambda);
                devices.push(DeviceEntry::new_tube(tube, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Vca => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Vca { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let vscale = find_model_param(model_name, "VSCALE").unwrap_or(0.05298);
                let g0 = find_model_param(model_name, "G0").unwrap_or(1.0);
                let vca = melange_devices::Vca::new(vscale, g0);
                devices.push(DeviceEntry::new_vca(vca, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::BjtForwardActive => {
                // BjtForwardActive uses the same runtime 2D model as Bjt
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-14);
                let bf = find_model_param(model_name, "BF").unwrap_or(200.0);
                let br = find_model_param(model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                let polarity = if is_pnp {
                    BjtPolarity::Pnp
                } else {
                    BjtPolarity::Npn
                };
                let nf = find_model_param(model_name, "NF").unwrap_or(1.0);
                devices.push(DeviceEntry::new_bjt(
                    BjtEbersMoll::new(is, melange_primitives::VT_ROOM, bf, br, polarity)
                        .with_nf(nf),
                    dev_info.start_idx,
                ));
            }
        }
    }
    devices
}

#[cfg(any())]
fn build_device_slots(
    netlist: &melange_solver::parser::Netlist,
    mna: &melange_solver::mna::MnaSystem,
) -> Vec<melange_solver::codegen::ir::DeviceSlot> {
    use melange_solver::codegen::ir::{
        BjtParams, DeviceParams, DeviceSlot, DeviceType, DiodeParams, JfetParams, MosfetParams,
        TubeParams,
    };

    let find_param = |model_name: &str, param: &str| -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param))
                    .map(|(_, v)| *v)
            })
    };

    let mut slots = Vec::new();
    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Diode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is = find_param(&model_name, "IS").unwrap_or(1e-15);
                let n = find_param(&model_name, "N").unwrap_or(1.0);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Diode,
                    start_idx: dev_info.start_idx,
                    dimension: 1,
                    params: DeviceParams::Diode(DiodeParams {
                        is,
                        n_vt: n * melange_primitives::VT_ROOM,
                        cjo: find_param(&model_name, "CJO").unwrap_or(0.0),
                        rs: find_param(&model_name, "RS").unwrap_or(0.0),
                        bv: find_param(&model_name, "BV").unwrap_or(f64::INFINITY),
                        ibv: find_param(&model_name, "IBV").unwrap_or(1e-10),
                        rth: find_param(&model_name, "RTH").unwrap_or(f64::INFINITY),
                        cth: find_param(&model_name, "CTH").unwrap_or(1e-3),
                        xti: find_param(&model_name, "XTI").unwrap_or(3.0),
                        eg: find_param(&model_name, "EG").unwrap_or(1.11),
                        tamb: find_param(&model_name, "TAMB").unwrap_or(300.15),
                    }),
                    has_internal_mna_nodes: false,
                    vg2k_frozen: 0.0,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is = find_param(&model_name, "IS").unwrap_or(1e-14);
                let bf = find_param(&model_name, "BF").unwrap_or(200.0);
                let br = find_param(&model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Bjt,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Bjt(BjtParams {
                        is,
                        vt: melange_primitives::VT_ROOM,
                        beta_f: bf,
                        beta_r: br,
                        is_pnp,
                        vaf: f64::INFINITY,
                        var: f64::INFINITY,
                        ikf: f64::INFINITY,
                        ikr: f64::INFINITY,
                        cje: find_param(&model_name, "CJE").unwrap_or(0.0),
                        cjc: find_param(&model_name, "CJC").unwrap_or(0.0),
                        nf: find_param(&model_name, "NF").unwrap_or(1.0),
                        nr: find_param(&model_name, "NR").unwrap_or(1.0),
                        ise: find_param(&model_name, "ISE").unwrap_or(0.0),
                        ne: find_param(&model_name, "NE").unwrap_or(1.5),
                        isc: find_param(&model_name, "ISC").unwrap_or(0.0),
                        nc: find_param(&model_name, "NC").unwrap_or(2.0),
                        rb: find_param(&model_name, "RB").unwrap_or(0.0),
                        rc: find_param(&model_name, "RC").unwrap_or(0.0),
                        re: find_param(&model_name, "RE").unwrap_or(0.0),
                        rth: f64::INFINITY,
                        cth: 1e-3,
                        xti: 3.0,
                        eg: 1.11,
                        tamb: 300.15,
                    }),
                    has_internal_mna_nodes: false,
                    vg2k_frozen: 0.0,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Jfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Jfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
                    .unwrap_or(false);
                let default_vp = if is_p_channel { 2.0 } else { -2.0 };
                let vp = find_param(&model_name, "VTO").unwrap_or(default_vp);
                let idss = if let Some(beta) = find_param(&model_name, "BETA") {
                    beta * vp * vp
                } else {
                    find_param(&model_name, "IDSS").unwrap_or(2e-3)
                };
                let lambda = find_param(&model_name, "LAMBDA").unwrap_or(0.001);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Jfet,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Jfet(JfetParams {
                        idss,
                        vp,
                        lambda,
                        is_p_channel,
                        cgs: find_param(&model_name, "CGS").unwrap_or(0.0),
                        cgd: find_param(&model_name, "CGD").unwrap_or(0.0),
                        rd: find_param(&model_name, "RD").unwrap_or(0.0),
                        rs: find_param(&model_name, "RS").unwrap_or(0.0),
                    }),
                    has_internal_mna_nodes: false,
                    vg2k_frozen: 0.0,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Mosfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Mosfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PM"))
                    .unwrap_or(false);
                let default_vt = if is_p_channel { -2.0 } else { 2.0 };
                let vt = find_param(&model_name, "VTO").unwrap_or(default_vt);
                let kp = find_param(&model_name, "KP").unwrap_or(0.1);
                let lambda = find_param(&model_name, "LAMBDA").unwrap_or(0.01);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Mosfet,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Mosfet(MosfetParams {
                        kp,
                        vt,
                        lambda,
                        is_p_channel,
                        cgs: find_param(&model_name, "CGS").unwrap_or(0.0),
                        cgd: find_param(&model_name, "CGD").unwrap_or(0.0),
                        rd: find_param(&model_name, "RD").unwrap_or(0.0),
                        rs: find_param(&model_name, "RS").unwrap_or(0.0),
                        gamma: find_param(&model_name, "GAMMA").unwrap_or(0.0),
                        phi: find_param(&model_name, "PHI").unwrap_or(0.6),
                        source_node: 0,
                        bulk_node: 0,
                    }),
                    has_internal_mna_nodes: false,
                    vg2k_frozen: 0.0,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Tube => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Triode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let mu = find_param(&model_name, "MU").unwrap_or(100.0);
                let ex = find_param(&model_name, "EX").unwrap_or(1.4);
                let kg1 = find_param(&model_name, "KG1").unwrap_or(1060.0);
                let kp = find_param(&model_name, "KP").unwrap_or(600.0);
                let kvb = find_param(&model_name, "KVB").unwrap_or(300.0);
                let lambda = find_param(&model_name, "LAMBDA").unwrap_or(0.0);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Tube,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Tube(TubeParams {
                        kind: melange_solver::device_types::TubeKind::SharpTriode,
                        mu,
                        ex,
                        kg1,
                        kp,
                        kvb,
                        // Leach fields are pentode-only; a triode slot leaves
                        // them at zero and carries the D&Z grid law instead.
                        ig_max: 0.0,
                        vgk_onset: 0.0,
                        gg: find_param(&model_name, "GG")
                            .unwrap_or(melange_devices::tube::DEFAULT_GG),
                        xi: find_param(&model_name, "XI")
                            .unwrap_or(melange_devices::tube::DEFAULT_XI),
                        cg: find_param(&model_name, "CG")
                            .unwrap_or(melange_devices::tube::DEFAULT_CG),
                        lambda,
                        ccg: find_param(&model_name, "CCG").unwrap_or(0.0),
                        cgp: find_param(&model_name, "CGP").unwrap_or(0.0),
                        ccp: find_param(&model_name, "CCP").unwrap_or(0.0),
                        rgi: find_param(&model_name, "RGI").unwrap_or(0.0),
                        kg2: 0.0,
                        alpha_s: 0.0,
                        a_factor: 0.0,
                        beta_factor: 0.0,
                        screen_form: melange_solver::device_types::ScreenForm::Rational,
                        mu_b: 0.0,
                        svar: 0.0,
                        ex_b: 0.0,
                        rth: f64::INFINITY,
                        cth: 0.0,
                        vbias_alpha: 0.0,
                        tamb: 300.15,
                    }),
                    has_internal_mna_nodes: false,
                    vg2k_frozen: 0.0,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Vca => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Vca { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let vscale = find_param(&model_name, "VSCALE").unwrap_or(0.05298);
                let g0 = find_param(&model_name, "G0").unwrap_or(1.0);
                let thd = find_param(&model_name, "THD").unwrap_or(0.0);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Vca,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Vca(melange_solver::VcaParams { vscale, g0, thd }),
                    has_internal_mna_nodes: false,
                    vg2k_frozen: 0.0,
                });
            }
            melange_solver::mna::NonlinearDeviceType::BjtForwardActive => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is = find_param(&model_name, "IS").unwrap_or(1e-14);
                let bf = find_param(&model_name, "BF").unwrap_or(200.0);
                let br = find_param(&model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                slots.push(DeviceSlot {
                    device_type: DeviceType::BjtForwardActive,
                    start_idx: dev_info.start_idx,
                    dimension: 1,
                    params: DeviceParams::Bjt(BjtParams {
                        is,
                        vt: melange_primitives::VT_ROOM,
                        beta_f: bf,
                        beta_r: br,
                        is_pnp,
                        vaf: f64::INFINITY,
                        var: f64::INFINITY,
                        ikf: f64::INFINITY,
                        ikr: f64::INFINITY,
                        cje: find_param(&model_name, "CJE").unwrap_or(0.0),
                        cjc: find_param(&model_name, "CJC").unwrap_or(0.0),
                        nf: find_param(&model_name, "NF").unwrap_or(1.0),
                        nr: find_param(&model_name, "NR").unwrap_or(1.0),
                        ise: find_param(&model_name, "ISE").unwrap_or(0.0),
                        ne: find_param(&model_name, "NE").unwrap_or(1.5),
                        isc: find_param(&model_name, "ISC").unwrap_or(0.0),
                        nc: find_param(&model_name, "NC").unwrap_or(2.0),
                        rb: find_param(&model_name, "RB").unwrap_or(0.0),
                        rc: find_param(&model_name, "RC").unwrap_or(0.0),
                        re: find_param(&model_name, "RE").unwrap_or(0.0),
                        rth: f64::INFINITY,
                        cth: 1e-3,
                        xti: 3.0,
                        eg: 1.11,
                        tamb: 300.15,
                    }),
                    has_internal_mna_nodes: false,
                    vg2k_frozen: 0.0,
                });
            }
        }
    }
    slots
}

#[cfg(any())]
fn write_wav(output: &PathBuf, sample_rate: f64, samples: &[f64]) -> Result<()> {
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: sample_rate as u32,
        bits_per_sample: 32,
        sample_format: hound::SampleFormat::Float,
    };
    let mut writer = hound::WavWriter::create(output, spec)
        .with_context(|| format!("Failed to create WAV file: {}", output.display()))?;

    let mut peak = 0.0f64;
    for &s in samples {
        peak = peak.max(s.abs());
        writer
            .write_sample(s as f32)
            .with_context(|| "Failed to write WAV sample")?;
    }
    writer
        .finalize()
        .with_context(|| "Failed to finalize WAV file")?;

    println!();
    println!("Output written to: {}", output.display());
    println!("  Samples: {}", samples.len());
    println!("  Duration: {:.2}s", samples.len() as f64 / sample_rate);
    println!(
        "  Peak level: {:.4} ({:.1} dB)",
        peak,
        if peak > 0.0 {
            20.0 * peak.log10()
        } else {
            f64::NEG_INFINITY
        }
    );

    Ok(())
}

fn list_nodes_source(circuit_source: &circuits::CircuitSource) -> Result<()> {
    use melange_solver::{mna::MnaSystem, parser::Netlist};

    println!("melange nodes");
    println!("  Source: {}", circuit_source.name());
    println!();

    // Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, .. } => content.clone(),
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read local file: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            let cache = cache::Cache::new()?;
            cache.get_sync(url, false)?
        }
    };

    let mut netlist =
        Netlist::parse(&netlist_str).with_context(|| "Failed to parse SPICE netlist")?;

    // Expand subcircuit instances (X elements) before MNA
    if !netlist.subcircuits.is_empty() {
        netlist
            .expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
    }

    // Topology gate: the wiring defects a solver cannot see. A typo'd node
    // name invents a node and floats whatever it was on, and every number
    // melange prints afterwards is correct for the circuit it was handed. One
    // implementation for every verb — `melange_solver::topology`.
    // `nodes` REPORTS and never refuses, and says so by calling
    // `topology_report` rather than the gate: this is the command a user
    // reaches for to FIND the typo, so no finding at any severity may stop it
    // (a `.port` naming a node the deck lacks refuses everywhere else).
    // `Ports::inferred` guesses the input port from a node named `in` — enough
    // to keep the report free of input-coupling-cap islands the other verbs do
    // not see — and a `.port` declaration supersedes the guess about where the
    // deck's edges are, which is what drops the "no port was declared for this
    // run" hedge from the messages.
    melange_solver::pipeline::topology_report(
        &netlist,
        &melange_solver::topology::Ports::inferred(&netlist).with_deck_pins(&netlist),
        &|m| println!("{m}"),
    );

    let mna = MnaSystem::from_netlist(&netlist).with_context(|| "Failed to build MNA system")?;

    // Unrecognized `.model` keys. `compile`/`simulate`/`analyze` hard-error on
    // these; `nodes` was the one inspection command that stayed silent, so a
    // deck could be read here, look clean, and then be refused downstream.
    // Warn (never error) — `nodes` exists to show what a deck contains, and
    // refusing to list a pot range because a diode card has a typo'd key would
    // be the worse trade. See `model_params::warn_unknown_keys_on_referenced_models`.
    melange_solver::model_params::warn_unknown_keys_on_referenced_models(&netlist);

    // Say the count out loud AND say what it counts: `nodes` lists ground,
    // `dc-op`'s N does not, and `analyze`/`compile`'s N adds the augmented
    // constraint rows on top. Three legitimate numbers for one circuit —
    // see `format_system_size`.
    println!(
        "Nodes in circuit: {} entries (ground + {} circuit nodes)",
        mna.n + 1,
        mna.n
    );
    println!("  (0) GND - Ground reference");

    let mut nodes: Vec<_> = mna.node_map.iter().collect();
    nodes.sort_by(|a, b| a.1.cmp(b.1));

    for (name, &idx) in nodes {
        if name != "0" {
            println!("  ({}) {}", idx, name);
        }
    }

    if !mna.nonlinear_devices.is_empty() {
        println!();
        println!("Nonlinear devices:");
        for dev in &mna.nonlinear_devices {
            println!(
                "  {}: {:?} (dimension: {})",
                dev.name, dev.device_type, dev.dimension
            );
        }
    }

    // Controls: the names a user needs for --pot / --switch. Either the
    // human-readable label OR the component name is accepted, so print both.
    if !netlist.pots.is_empty()
        || !netlist.switches.is_empty()
        || !netlist.wipers.is_empty()
        || !netlist.gangs.is_empty()
    {
        println!();
        println!("Controls (name or label works with --pot / --switch):");
        // A `.wiper` emits two pots — the halves of its track — and they are
        // real setters in the generated API, so hiding them would mislead a
        // plugin author. Listing them as if they were two independent knobs
        // misleads everyone else. Name the relationship instead.
        let wiper_half = |r: &str| -> Option<String> {
            netlist.wipers.iter().find_map(|w| {
                let label = w.label.as_deref().unwrap_or(&w.resistor_cw);
                if w.resistor_cw == r {
                    Some(format!("cw half of wiper \"{label}\""))
                } else if w.resistor_ccw == r {
                    Some(format!("ccw half of wiper \"{label}\""))
                } else {
                    None
                }
            })
        };
        for pot in &netlist.pots {
            let label = pot.label.as_deref().unwrap_or(&pot.resistor_name);
            // No explicit default means the resistor's own declared value —
            // `mna.rs` resolves it with `default_value.unwrap_or(*value)`. The
            // number is knowable, so print it rather than the word "nominal",
            // which reads like a missing value next to every other pot's figure.
            let default = pot
                .default_value
                .or_else(|| {
                    netlist.elements.iter().find_map(|e| match e {
                        melange_solver::parser::Element::Resistor { name, value, .. }
                            if name.eq_ignore_ascii_case(&pot.resistor_name) =>
                        {
                            Some(*value)
                        }
                        _ => None,
                    })
                })
                .map(|d| format!("{d:.0}"))
                .unwrap_or_else(|| "nominal".to_string());
            let note = wiper_half(&pot.resistor_name)
                .map(|w| format!("  ({w})"))
                .unwrap_or_default();
            println!(
                "  pot     {:<26} [{}]  {:.0}..{:.0} ohm, default {}{}",
                format!("\"{label}\""),
                pot.resistor_name,
                pot.min_value,
                pot.max_value,
                default,
                note
            );
        }
        for wiper in &netlist.wipers {
            let label = wiper.label.as_deref().unwrap_or(&wiper.resistor_cw);
            println!(
                "  wiper   {:<26} [{}/{}]  total {:.0} ohm, position 0..1",
                format!("\"{label}\""),
                wiper.resistor_cw,
                wiper.resistor_ccw,
                wiper.total_resistance
            );
        }
        for sw in &netlist.switches {
            let label = sw
                .label
                .clone()
                .unwrap_or_else(|| sw.component_names.join(","));
            println!(
                "  switch  {:<26} {} positions (controls {})",
                format!("\"{label}\""),
                sw.positions.len(),
                sw.component_names.join(",")
            );
        }
        for gang in &netlist.gangs {
            println!(
                "  gang    {:<26} {} members, position 0..1",
                format!("\"{}\"", gang.label),
                gang.members.len()
            );
        }
    }

    Ok(())
}

/// Human-readable name for a DC-system row index reported by
/// `DcOpResult::kcl_worst_row`: the node name for circuit nodes, otherwise
/// the raw row (BJT internal nodes added by the DC solver).
fn dc_op_row_name(row: usize, idx_to_name: &[String], n: usize) -> String {
    if row < n && row + 1 < idx_to_name.len() {
        format!("v({})", idx_to_name[row + 1])
    } else {
        format!("dc row {} (internal node)", row)
    }
}

/// Options for `melange dc-op` (see `Commands::DcOp`).
struct DcOpOptions<'a> {
    input_node: &'a str,
    input_resistance: Option<f64>,
    format: &'a str,
    sample_rate: f64,
    oversampling: Option<usize>,
    solver: &'a str,
    opamp_rail_mode: melange_solver::codegen::OpampRailMode,
    bjt_fa: &'a str,
    tube_grid_fa: &'a str,
    pot_overrides: &'a [String],
    allow_unconverged_dc_op: bool,
    /// Test-only DC-OP Newton budget (hidden `--dc-op-max-iterations`).
    dc_op_max_iterations: Option<usize>,
}

/// `melange dc-op`: the operating point the build ships. The circuit is
/// assembled by the one build every verb uses ([`melange_solver::build::assemble`],
/// up to the IR, where the route and the operating point are settled), so
/// the vector printed is the one the generated code embeds as `DC_OP`.
fn run_dc_op(circuit_source: &circuits::CircuitSource, opts: &DcOpOptions<'_>) -> Result<()> {
    use melange_solver::codegen::ir::CircuitIR;

    // Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, .. } => content.clone(),
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read local file: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            let cache = cache::Cache::new()?;
            cache.get_sync(url, false)?
        }
    };

    let d = melange_solver::codegen::CodegenConfig::default();
    let build_opts = melange_solver::build::BuildOptions {
        sample_rate: opts.sample_rate,
        circuit_name: "dc_op".to_string(),
        // Match parse-time node normalization (lowercase, gnd→0).
        input_nodes: vec![melange_solver::parser::normalize_node_name(opts.input_node)],
        // An operating point has no output.
        output_nodes: Vec::new(),
        max_iter: None,
        tolerance: d.tolerance,
        output_scale: 1.0,
        output_clamp: d.output_clamp_v,
        input_resistance: opts.input_resistance,
        oversampling: opts.oversampling,
        dc_block: true,
        solver: opts.solver.to_string(),
        backward_euler: false,
        force_trap: false,
        tube_grid_fa: opts.tube_grid_fa.to_string(),
        subsample_fire: d.subsample_fire,
        subsample_lit_factor: diag_lit_factor(),
        bjt_fa_mode: parse_bjt_fa_mode(opts.bjt_fa),
        opamp_rail_mode: opts.opamp_rail_mode,
        nodal_sub_path_override: d.nodal_sub_path_override,
        allow_static_glow_on_full_lu: false,
        noise_mode: d.noise_mode,
        noise_seed: d.noise_master_seed,
        emit_dc_op_recompute: false,
        plugin_format: false,
        // Without --pot the deck's `.pot` defaults, as `compile` builds it.
        pot_overrides: if opts.pot_overrides.is_empty() {
            None
        } else {
            Some(opts.pot_overrides.to_vec())
        },
        resolve_taps: true,
        inject_runtime: true,
        disable_unit_variation: false,
        disable_self_heating: false,
        allow_unconverged_dc_op: opts.allow_unconverged_dc_op,
        dc_op_max_iterations: opts.dc_op_max_iterations,
        output_clamp_auto: false,
    };
    // The build's own lines go to stderr: stdout is the report (JSON-clean).
    let assembled =
        melange_solver::build::assemble(&netlist_str, &build_opts, &|a| eprintln!("{a}"), &|a| {
            eprintln!("{a}")
        })
        .map_err(build_error)?;
    let format = opts.format;
    let result = &assembled.dc_op;
    let mna = &assembled.mna;
    let device_slots =
        CircuitIR::build_device_info_with_mna(&assembled.netlist, Some(mna)).unwrap_or_default();
    let route = assembled.solver_label;

    // Build reverse node map (index → name)
    let mut idx_to_name: Vec<String> = vec!["0".to_string(); mna.n + 1];
    for (name, &idx) in &mna.node_map {
        if idx <= mna.n {
            idx_to_name[idx] = name.clone();
        }
    }

    if format == "json" {
        // JSON output for machine consumption, every number in its shortest
        // exact (round-trip) form: a consumer comparing two operating points
        // must not see a real difference rounded away.
        print!("{{");
        print!("\"converged\":{},", result.converged);
        print!("\"method\":\"{:?}\",", result.method);
        print!("\"iterations\":{},", result.iterations);
        print!("\"kcl_residual_max\":{:e},", result.kcl_residual_max);
        print!(
            "\"kcl_worst_row\":{},",
            match result.kcl_worst_row {
                Some(row) => format!("\"{}\"", dc_op_row_name(row, &idx_to_name, mna.n)),
                None => "null".to_string(),
            }
        );
        print!(
            "\"rail_pin\":\"{}\",",
            result
                .rail_pin
                .label()
                .replace('\\', "\\\\")
                .replace('"', "\\\"")
        );
        print!("\"solver\":\"{}\",", route);
        print!("\"n\":{},\"m\":{},", mna.n, mna.m);

        // Node voltages
        print!("\"nodes\":{{");
        let mut nodes: Vec<_> = mna.node_map.iter().collect();
        nodes.sort_by(|a, b| a.1.cmp(b.1));
        let mut first = true;
        for (name, &idx) in &nodes {
            if idx > 0 && idx <= result.v_node.len() {
                if !first {
                    print!(",");
                }
                first = false;
                print!("\"{}\":{:e}", name, result.v_node[idx - 1]);
            }
        }
        print!("}}");

        // Nonlinear device currents
        if !result.i_nl.is_empty() {
            print!(",\"devices\":{{");
            let mut first = true;
            for (i, slot) in device_slots.iter().enumerate() {
                let s = slot.start_idx;
                let dev_name = mna
                    .nonlinear_devices
                    .get(i)
                    .map(|d| d.name.as_str())
                    .unwrap_or("?");
                for d in 0..slot.dimension {
                    if s + d < result.i_nl.len() {
                        if !first {
                            print!(",");
                        }
                        first = false;
                        print!(
                            "\"{}[{}]\":{{\"v_nl\":{:e},\"i_nl\":{:e}}}",
                            dev_name,
                            d,
                            result.v_nl[s + d],
                            result.i_nl[s + d]
                        );
                    }
                }
            }
            print!("}}");
        }

        println!("}}");
    } else {
        // Human-readable output
        eprintln!("melange dc-op");
        eprintln!("  {}", format_system_size(mna.n, mna.n, mna.m));
        eprintln!("  Solver: {route} ({})", assembled.solver_reason);
        eprintln!(
            "  Converged: {} ({:?}, {} iterations)",
            result.converged, result.method, result.iterations
        );
        if result.rail_pin != melange_solver::dc_op::RailPin::None {
            eprintln!("  Railed op-amps: {}", result.rail_pin.label());
        }
        match result.kcl_worst_row {
            Some(row) => eprintln!(
                "  KCL residual: max |F| = {:.3e} A at {}",
                result.kcl_residual_max,
                dc_op_row_name(row, &idx_to_name, mna.n)
            ),
            None => eprintln!("  KCL residual: n/a (no voltage rows)"),
        }
        eprintln!();

        // Node voltages
        println!("Node voltages:");
        let mut nodes: Vec<_> = mna.node_map.iter().collect();
        nodes.sort_by(|a, b| a.1.cmp(b.1));
        for (name, &idx) in &nodes {
            if idx > 0 && idx <= result.v_node.len() {
                let v = result.v_node[idx - 1];
                if v.abs() > 0.1 {
                    println!("  v({}) = {:.4} V", name, v);
                } else if v.abs() > 1e-6 {
                    println!("  v({}) = {:.4} mV", name, v * 1e3);
                } else {
                    println!("  v({}) = {:.4e} V", name, v);
                }
            }
        }

        // Nonlinear device operating points
        if !result.i_nl.is_empty() {
            println!();
            println!("Device operating points:");
            for (i, slot) in device_slots.iter().enumerate() {
                let s = slot.start_idx;
                let dev_name = mna
                    .nonlinear_devices
                    .get(i)
                    .map(|d| d.name.as_str())
                    .unwrap_or("?");
                println!("  {} ({:?}):", dev_name, slot.device_type);
                for d in 0..slot.dimension {
                    if s + d < result.i_nl.len() {
                        let v = result.v_nl[s + d];
                        let i = result.i_nl[s + d];
                        if i.abs() > 1e-3 {
                            println!("    [{}] v_nl={:.4} V, i_nl={:.4} mA", d, v, i * 1e3);
                        } else if i.abs() > 1e-6 {
                            println!("    [{}] v_nl={:.4} V, i_nl={:.4} uA", d, v, i * 1e6);
                        } else {
                            println!("    [{}] v_nl={:.4} V, i_nl={:.4e} A", d, v, i);
                        }
                    }
                }
            }
        }
    }

    Ok(())
}

fn handle_sources(action: SourceAction) -> Result<()> {
    use crate::sources::{format_sources_list, SourcesConfig};

    match action {
        SourceAction::List => {
            let config = SourcesConfig::load()?;
            println!("{}", format_sources_list(&config));
            Ok(())
        }
        SourceAction::Add {
            name,
            url,
            license,
            attribution,
        } => {
            let mut config = SourcesConfig::load()?;

            // Refuse here rather than store something that only fails later.
            // A rejected source used to be accepted, listed as healthy by
            // `sources list`, and then die at first use with a url-crate
            // internal message telling the user their absolute path was a
            // "relative URL without a base".
            let looks_remote = url.starts_with("http://") || url.starts_with("https://");
            if !looks_remote && !std::path::Path::new(url.trim_end_matches('/')).is_dir() {
                let hint = if url.starts_with("file://") {
                    "For a local folder give the plain path, not a file:// URL."
                } else if std::path::Path::new(&url).exists() {
                    "That path exists but is not a directory. A source is the \
                     FOLDER circuits live in, not one .cir file — to compile a \
                     single file just pass it directly."
                } else {
                    "A source is either an http(s) base URL or a local directory \
                     that exists."
                };
                anyhow::bail!("Cannot use '{url}' as a source. {hint}");
            }

            // Store a local directory absolute: a relative path only resolves
            // from the directory it was added in.
            let url = if looks_remote {
                url
            } else {
                std::fs::canonicalize(url.trim_end_matches('/'))
                    .with_context(|| format!("Cannot resolve '{url}'"))?
                    .to_string_lossy()
                    .into_owned()
            };

            if config.has_source(&name) {
                println!("Warning: Source '{}' already exists. Overwriting.", name);
            }

            config.add_source(&name, &url, license.as_deref(), attribution.as_deref());
            config.save()?;

            println!("Added source '{}': {}", name, url);
            if let Some(lic) = license {
                println!("  License: {}", lic);
            }
            if let Some(attr) = attribution {
                println!("  Attribution: {}", attr);
            }

            Ok(())
        }
        SourceAction::Remove { name } => {
            let mut config = SourcesConfig::load()?;

            if config.remove_source(&name) {
                config.save()?;
                println!("Removed source '{}'", name);
            } else {
                anyhow::bail!("Source '{}' not found", name);
            }

            Ok(())
        }
        SourceAction::Show { name } => {
            let config = SourcesConfig::load()?;

            if let Some(source) = config.get_source(&name) {
                println!("Source: {}", name);
                println!("  URL: {}", source.url);
                if let Some(lic) = &source.license {
                    println!("  License: {}", lic);
                }
                if let Some(attr) = &source.attribution {
                    println!("  Attribution: {}", attr);
                }
                if let Some(subdir) = &source.subdirectory {
                    println!("  Subdirectory: {}", subdir);
                }
                let cache = crate::cache::Cache::new()?;
                match config.list_circuits(&name, &cache)? {
                    Some(circuits) => {
                        println!();
                        println!("  {} circuits:", circuits.len());
                        let width = circuits.iter().map(|(n, _)| n.len()).max().unwrap_or(0);
                        for (circuit, entry) in &circuits {
                            let meta: Vec<&str> =
                                [entry.category.as_deref(), entry.tier.as_deref()]
                                    .into_iter()
                                    .flatten()
                                    .collect();
                            println!("    {circuit:<width$}  {}", meta.join(", "));
                        }
                        if let Some((first, _)) = circuits.first() {
                            println!();
                            println!("  Use one as `{name}:<circuit>`, e.g. `melange nodes {name}:{first}`.");
                        }
                    }
                    None => {
                        println!();
                        println!(
                            "  No circuits-index.json published, so its circuits cannot be \
                             listed; `{name}:<file-name>` still resolves <base>/<file-name>.cir."
                        );
                    }
                }
            } else {
                anyhow::bail!("Source '{}' not found", name);
            }

            Ok(())
        }
    }
}

fn list_builtins() -> Result<()> {
    println!("Available builtin circuits:");
    println!();

    let builtins = circuits::list_builtins();

    for (name, description) in builtins {
        println!("  {:<15} - {}", name, description);
    }

    println!();
    println!("Usage examples:");
    println!("  melange compile passive-eq1a --format plugin -o passive-eq");
    println!("  melange simulate passive-eq1a --amplitude 0.1 -o drive.wav");
    println!("  melange nodes passive-eq1a");
    println!();
    println!("The full circuit library lives in a separate repo. Add it, list it, use it:");
    println!(
        "  melange sources add melange-circuits \
         https://gitlab.com/oomox-group/melange-circuits/-/raw/main"
    );
    println!("  melange sources show melange-circuits");
    println!("  melange nodes melange-circuits:<circuit>");

    Ok(())
}

fn handle_cache(action: CacheAction) -> Result<()> {
    use crate::cache::{format_cache_list, Cache};
    use crate::codegen_runner::BinaryCache;

    match action {
        CacheAction::List => {
            let cache = Cache::new()?;
            println!("{}", format_cache_list(&cache));
            let bin_cache = BinaryCache::new()?;
            let bin_stats = bin_cache.stats();
            println!();
            println!("Compiled binaries:");
            println!(
                "  {} files ({})",
                bin_stats.total_files,
                bin_stats.formatted_size()
            );
            Ok(())
        }
        CacheAction::Clear => {
            let cache = Cache::new()?;
            cache.clear()?;
            let bin_cache = BinaryCache::new()?;
            bin_cache.clear()?;
            println!("Cache cleared (circuits + compiled binaries).");
            Ok(())
        }
        CacheAction::Stats => {
            let cache = Cache::new()?;
            let stats = cache.stats();
            println!("Circuit cache:");
            println!("  Location: {}", cache.cache_dir().display());
            println!("  Files: {}", stats.total_files);
            println!("  Size: {}", stats.formatted_size());
            let bin_cache = BinaryCache::new()?;
            let bin_stats = bin_cache.stats();
            println!();
            println!("Binary cache (compiled simulate/analyze runs; `melange cache clear` empties both):");
            println!("  Location: {}", bin_cache.cache_dir().display());
            println!("  Files: {}", bin_stats.total_files);
            println!("  Size: {}", bin_stats.formatted_size());
            Ok(())
        }
    }
}

#[cfg(test)]
mod declares_state_field_tests {
    use super::declares_state_field;

    #[test]
    fn a_doc_comment_mention_is_not_a_declaration() {
        let code = "/// Past this depth the hold fires and `diag_nr_hold_count` counts it.\n\
                    pub struct CircuitState {\n    pub diag_be_fallback_count: u64,\n}\n";
        assert!(!declares_state_field(code, "diag_nr_hold_count"));
        assert!(declares_state_field(code, "diag_be_fallback_count"));
    }
}

#[cfg(test)]
mod silent_output_tests {
    use super::*;

    /// Exactly-zero is the case that shipped a 192 KB file of digital silence
    /// with a clean report and exit 0.
    #[test]
    fn hard_zero_is_silent() {
        assert!(output_is_silent(0.0));
        assert!(output_is_silent(-0.0));
    }

    /// The reason the rule is a threshold and not `== 0.0`: a denormal output
    /// is just as broken, and equality would wave it through.
    #[test]
    fn absurdly_small_is_silent() {
        assert!(output_is_silent(1e-300));
        assert!(output_is_silent(-1e-300));
        assert!(output_is_silent(f64::MIN_POSITIVE));
        // Under the old `{:.6}` DIAG format everything here arrived as
        // `0.000000`; with `{:.6e}` the real magnitude survives the round trip
        // and the threshold is what decides, not the printf width.
        assert!(output_is_silent(4.9e-7));
    }

    /// The other half of the rule: a quiet-but-working circuit must never be
    /// nagged. 1e-6 V is -120 dBFS; real outputs live many decades above it.
    #[test]
    fn quiet_but_working_is_not_silent() {
        assert!(!output_is_silent(SILENT_OUTPUT_PEAK_V));
        assert!(!output_is_silent(1e-5)); // -100 dBFS, still absurdly quiet
        assert!(!output_is_silent(1e-3)); // a heavily attenuated tap
        assert!(!output_is_silent(0.5)); // ordinary line level
        assert!(!output_is_silent(-0.5));
    }

    /// A blown-up solve is a different failure with its own counters; calling
    /// it "nothing to hear" would be wrong.
    #[test]
    fn nan_and_inf_are_not_silence() {
        assert!(!output_is_silent(f64::NAN));
        assert!(!output_is_silent(f64::INFINITY));
        assert!(!output_is_silent(f64::NEG_INFINITY));
    }

    /// The message has to name the causes a newcomer cannot tell apart, and
    /// echo back the output node they actually passed.
    #[test]
    fn warning_names_the_actionable_causes() {
        // A peak far below the old `{:.6}` print resolution must be REPORTED,
        // not flattened to "0.000000" — that was the whole defect.
        let tiny = silent_output_warning(3.7e-9, "out", Some(48000), Some(12.0));
        assert!(
            tiny.contains("3.700000e-9"),
            "warning must quote the real peak: {tiny}"
        );
        assert!(!tiny.contains("0.000000 V"), "{tiny}");

        let msg = silent_output_warning(0.0, "out", Some(48000), Some(12.0));
        for needle in [
            "digital silence",
            "mistyped node name",
            "DC path to ground",
            "--output-node out",
            "melange nodes",
            "warning, not an error",
            "48000",
        ] {
            assert!(
                msg.contains(needle),
                "message must mention {needle:?}: {msg}"
            );
        }
    }

    /// Live-but-disconnected and stone-dead are different bugs; the internal
    /// node peak is what separates them.
    #[test]
    fn warning_distinguishes_dead_circuit_from_dead_output_tap() {
        let alive = silent_output_warning(0.0, "out", Some(100), Some(12.0));
        assert!(alive.contains("Internal nodes did move"), "{alive}");
        assert!(alive.contains("12.000"), "{alive}");

        let dead = silent_output_warning(0.0, "out", Some(100), Some(0.0));
        assert!(
            dead.contains("nothing in the\n  circuit moved at all"),
            "{dead}"
        );

        // No `max_abs_v_prev` in the diagnostics: say nothing rather than guess.
        let unknown = silent_output_warning(0.0, "out", None, None);
        assert!(!unknown.contains("Internal nodes did move"), "{unknown}");
        assert!(!unknown.contains("moved at all"), "{unknown}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The `--format plugin` hint must name a DIRECTORY derived from the
    /// circuit, never echo back the `--format code` output FILE.
    #[test]
    fn plugin_project_suggestion_is_a_directory_name() {
        assert_eq!(suggest_plugin_project_dir("od.cir"), "od-plugin");
        assert_eq!(
            suggest_plugin_project_dir("/home/u/decks/passive-eq1a.cir"),
            "passive-eq1a-plugin"
        );
        assert_eq!(
            suggest_plugin_project_dir("builtin:passive-eq1a"),
            "passive-eq1a-plugin"
        );
        // Never a path, never an extension.
        let s = suggest_plugin_project_dir("/tmp/proj/src/circuit.rs");
        assert!(!s.contains('/'), "{s}");
        assert!(!s.contains('.'), "{s}");
        // Nothing usable -> a safe generic name rather than an empty --output.
        assert_eq!(suggest_plugin_project_dir("/"), "melange-plugin");
        assert_eq!(suggest_plugin_project_dir(""), "melange-plugin");
    }

    #[test]
    fn test_cli_parse() {
        // Test that CLI parsing works
        let cli = Cli::parse_from(["melange", "builtins"]);
        match cli.command {
            Commands::Builtins => {}
            _ => panic!("Expected Builtins command"),
        }
    }

    /// Fix 4 regression: `apply_grid_off_reduction` used to rebuild the MNA
    /// via `from_netlist_with_grid_off` with an EMPTY forward-active set,
    /// silently discarding the FA BJT reduction the caller had already
    /// applied (while the compile summary still printed the FA line).
    /// Plexi-class circuits need both reductions composed.
    #[test]
    fn grid_off_rebuild_preserves_forward_active_reduction() {
        use melange_solver::mna::MnaSystem;
        use melange_solver::parser::Netlist;

        let spice = "\
FA + grid-off composition
V1 vcc 0 DC 250
RB1 vcc b 470k
RB2 b 0 47k
RC vcc c 10k
RE e 0 1k
Q1 c b e QNPN
P1 plate grid cath screen EL84
RGL grid 0 1Meg
RSC vcc screen 1k
RPL vcc plate 5k
RK cath 0 130
.model QNPN NPN(IS=1e-14 BF=100)
.model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 \
              KP=152.4 KVB=4015.8 ALPHA_S=7.66 \
              A_FACTOR=4.344e-4 BETA_FACTOR=0.148)
";
        let netlist = Netlist::parse(spice).expect("parse");
        let full = MnaSystem::from_netlist(&netlist).expect("full MNA");
        assert_eq!(full.m, 5, "full system: BJT 2D + pentode 3D");

        // Simulate the caller's FA step (composition test — the FA set is
        // hand-supplied; detection is covered elsewhere).
        let forward_active: std::collections::HashSet<String> =
            ["Q1".to_string()].into_iter().collect();
        let mut mna = MnaSystem::from_netlist_forward_active(&netlist, &forward_active)
            .expect("FA-reduced MNA");
        assert_eq!(mna.m, 4, "after FA: BJT 1D + pentode 3D");

        let fa_config = melange_solver::codegen::CodegenConfig::default();
        // `--tube-grid-fa on` forces grid-off on every non-variable-mu
        // pentode regardless of DC bias, so detection is deterministic here.
        let grid_off = melange_solver::pipeline::apply_grid_off_reduction(
            &mut mna,
            &netlist,
            &fa_config,
            &forward_active,
            "on",
            "",
            48000.0,
            1,
            &[(0, 1.0)],
        )
        .expect("grid-off reduction");
        assert_eq!(grid_off.len(), 1, "pentode P1 must be grid-off reduced");
        assert_eq!(
            mna.m, 3,
            "after grid-off rebuild BOTH reductions must survive: BJT 1D + pentode 2D \
             (old bug: rebuild dropped FA, giving M=4 while the summary claimed FA)"
        );
    }
}

/// Fetch a resolved circuit's content, self-healing a stale index.
///
/// A 404 on a path an index gave us means the cached index is stale — the deck
/// moved tier since we last fetched it. Refetch the index once and retry, so a
/// promotion self-heals instead of needing `melange cache clear`. Only fires
/// for indexed sources: an unindexed one re-resolves to the same flat URL and
/// returns the same 404 without a wasted index request.
///
/// Shared by every command that reads a remote circuit, so `compile`,
/// `simulate` and `validate` cannot drift into disagreeing about it.
fn fetch_remote_circuit(src: &circuits::CircuitSource) -> Result<String> {
    let (url, indexed) = match src {
        circuits::CircuitSource::Url { url } => (url, None),
        circuits::CircuitSource::Friendly {
            url,
            source,
            circuit,
        } => (url, Some((source, circuit))),
        _ => anyhow::bail!("fetch_remote_circuit called on a non-remote source"),
    };
    println!("  Fetching from URL: {}", url);
    let cache = cache::Cache::new()?;
    match cache.get_sync(url, false) {
        Ok(c) => Ok(c),
        Err(e) if e.downcast_ref::<cache::NotFound>().is_some() => {
            let Some((source, circuit)) = indexed else {
                return Err(e);
            };
            let config = sources::SourcesConfig::load()?;
            let fresh = config.resolve_circuit_indexed(source, circuit, &cache, true)?;
            if &fresh == url {
                return Err(e);
            }
            println!("  Index was stale; refetched: {}", fresh);
            cache.get_sync(&fresh, false)
        }
        Err(e) => Err(e),
    }
}
