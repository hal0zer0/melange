use crate::plugin_template;
use clap::{Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "melange")]
#[command(about = "Circuit modeling toolkit - from SPICE to real-time DSP")]
// Version carries the build commit (`version_label`) so `melange --version`
// disambiguates a released tag, an unreleased main, and a local build that
// otherwise all print the same bare CARGO_PKG_VERSION.
#[command(version = version_label())]
pub(crate) struct Cli {
    /// Print the solver-routing detail (why the route and integrator were
    /// chosen, kernel measurements, iteration budget). Without it, `compile`,
    /// `simulate` and `analyze` say which route and integrator they used in
    /// one line; warnings and refusals print either way.
    #[arg(short, long, global = true)]
    pub(crate) verbose: bool,

    #[command(subcommand)]
    pub(crate) command: Commands,
}

/// Help heading for flags that override a choice melange makes itself. Kept
/// out of the main list so `--help` leads with what a new user needs.
const EXPERT_HEADING: &str =
    "Solver overrides (melange chooses these; set them to pin or debug a route)";

/// The values `--solver` accepts, checked at parse time. The build treats any
/// value other than `dk` or `nodal` as auto, so an unchecked typo (`--solver
/// nodel`) silently built the auto route.
pub(crate) const SOLVER_VALUES: [&str; 3] = ["auto", "dk", "nodal"];

#[derive(Subcommand)]
pub(crate) enum Commands {
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

        /// Output node name(s), comma-separated for multi-output (e.g., "out_l,out_r").
        /// `--format plugin` takes one (mono plugin) or two (stereo plugin, one
        /// node per channel); `--format code` takes any number.
        #[arg(short = 'n', long, default_value = "out")]
        output_node: String,

        /// Maximum NR iterations per sample. Defaults to an auto-tuned budget
        /// (scales with M, solver route, and trap spectral radius); setting it
        /// pins that exact budget. A nodal build never ships less than 100 and
        /// refuses a pin below it.
        #[arg(help_heading = EXPERT_HEADING, long)]
        max_iter: Option<usize>,

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

        /// Add Input Level and Output Level parameters to the plugin (default:
        /// true). `--with-level-params=false` leaves them out, as does
        /// `--no-level-params`.
        #[arg(
            long,
            value_name = "BOOL",
            default_value_t = true,
            action = clap::ArgAction::Set,
            num_args = 0..=1,
            require_equals = true,
            default_missing_value = "true"
        )]
        with_level_params: bool,

        /// Generate plugin without Input/Output Level parameters (same as
        /// `--with-level-params=false`)
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

        /// Solver: auto (default), dk, nodal.
        ///
        /// auto measures the circuit and picks the DK solver unless the
        /// circuit needs the nodal one: behavioral B-sources, saturating
        /// inductors, more than one transformer, M of 10 or more, positive
        /// feedback or a trapezoidal instability in the DK kernel, an
        /// ill-conditioned kernel, a self-starting oscillator, or an op-amp
        /// feature only nodal implements (active-set or boyle-diodes rail
        /// handling, AOL_TRANSIENT_CAP). `-v` prints the reason. dk and nodal
        /// force a route; dk is refused where it cannot represent the circuit.
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto", value_parser = SOLVER_VALUES)]
        solver: String,

        /// Use backward Euler integration instead of trapezoidal.
        /// Unconditionally stable — fixes divergence in high-gain feedback amplifiers.
        /// Trades second-order accuracy for first-order (slight HF rolloff).
        #[arg(help_heading = EXPERT_HEADING, long)]
        backward_euler: bool,

        /// Force trapezoidal even when the ring predicate would promote the
        /// build to backward Euler (a stiff mode that trapezoidal leaves
        /// ringing near Nyquist); also turns off the runtime BE-latch. Escape
        /// hatch for bisecting regressions or reproducing older output.
        /// Ignored when `--backward-euler` is already set.
        #[arg(help_heading = EXPERT_HEADING, long)]
        force_trap: bool,

        /// Pentode grid-off dimension reduction mode.
        ///
        /// When a pentode's grid is biased well below cutoff at DC-OP, the
        /// Ig1 NR dimension can be dropped and Vg2k frozen, reducing M by 1
        /// per grid-off tube. A smaller M is a cheaper Newton solve, and can
        /// bring a circuit under M=10 (from which auto routes nodal) or under
        /// the M=32 limit every route has.
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
        /// * hard — post-NR v[out].clamp(VEE, VCC). Cheapest. Breaks KCL for
        ///   AC-coupled downstream caps: the clamped node no longer agrees
        ///   with the charge the next cap holds.{n}{n}
        /// * active-set — post-NR constrained re-solve. KCL-consistent hard
        ///   clip, so a cap downstream of a railed op-amp keeps a correct
        ///   history. Still produces square-wave harmonics.{n}{n}
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
        /// Default OFF: the method is not emitted and the generated code is
        /// unchanged by this flag. When ON, plugins with per-instance
        /// component jitter can call
        /// `state.recompute_dc_op()` to jump to the jittered equilibrium
        /// without a warmup loop. See docs/aidocs/DC_OP.md for MVP scope
        /// (Direct-NR only, no basin-trap handling).
        #[arg(long)]
        emit_dc_op_recompute: bool,

        /// Plugin display name (defaults to capitalized circuit filename)
        #[arg(long)]
        name: Option<String>,

        /// Ask for a mono (1-in/1-out) plugin. This changes nothing today: a
        /// plugin built from one output node is mono unless `--stereo` is
        /// given, and one built from two output nodes is stereo (one node per
        /// channel), where `--mono` is refused rather than drop a node. Also
        /// refused with several output nodes under `--format code`, and with
        /// `--stereo`.
        #[arg(long)]
        mono: bool,

        /// Make a stereo (2-in/2-out) plugin from a circuit with ONE output
        /// node by running two independent copies of it, one per channel.
        /// Without it such a plugin is mono. Costs about twice the CPU.
        ///
        /// Both copies have the same component values, `.tolerance` and
        /// `.mismatch` included: it is one circuit duplicated, not two units.
        /// Every knob and switch moves both. Their circuit noise is
        /// independent, as two physical copies' is: the left channel uses the
        /// `--noise-seed`, the right a seed derived from it (with seed 0, from
        /// a clock read at every initialize/reset).
        ///
        /// `--format plugin` only. Refused with `--mono` and with two output
        /// nodes, which already make a stereo plugin (one node per channel).
        #[arg(long)]
        stereo: bool,

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

        /// Input node the test signal drives
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

        /// Sample rate in Hz the circuit is built for: the solver route,
        /// integrator and oversampling are chosen at this rate, and the test
        /// tone is rendered at it. Default 48000. With --input-audio and no
        /// --sample-rate, the circuit is built at the WAV's own rate; a
        /// --sample-rate that differs from the WAV's rate is refused (a build
        /// for one rate does not carry over to another).
        #[arg(short, long)]
        sample_rate: Option<f64>,

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

        /// Solver: auto (default), dk, nodal. Mirrors `compile --solver`
        /// (see `melange compile --help` for how auto chooses).
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto", value_parser = SOLVER_VALUES)]
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
        /// spectral radius). A nodal build never ships less than 100 and
        /// refuses a pin below it.
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
        /// columns. Harmonics above Nyquist are reported as `nan`. `thd_pct`
        /// sums H2..HN BELOW 20 kHz (and below Nyquist), the common audio
        /// definition (use 13 for H2..H13); it is `nan` for a point at or
        /// above 10 kHz, where no harmonic is in that band. The hN_dbc
        /// columns still report every harmonic up to Nyquist.
        #[arg(long, default_value = "0")]
        harmonics: usize,

        /// Drive-level pre-roll per point, in seconds. Each point is driven at
        /// its own frequency and amplitude for at least this long (whole
        /// 10-cycle DFT windows) before the window it measures, so the
        /// reading is the steady state at that drive, not the sweep's history
        /// (a sagging supply rail, bypass caps re-centring, the previous
        /// point's level). The point is then measured again after another
        /// stretch of this length, repeating until two successive
        /// measurements agree (fundamental gain and phase, and the harmonic
        /// vector, each within 0.1 %) or --preroll-max-secs is spent. The
        /// default 0.25 s covers time constants up to ~50 ms outright
        /// (5 tau); slower ones are caught by the agreement check. A time
        /// constant many times longer than this spacing can move less than
        /// the tolerance between two checks while still far from settled:
        /// for such a circuit raise --preroll-secs (and --preroll-max-secs).
        /// The zero-drive settle before the first point (0.5 s, 5 s with
        /// inductors) is separate and unchanged.
        #[arg(long, value_name = "SECS", default_value = "0.25")]
        preroll_secs: f64,

        /// Cap on drive-level time per point while the settle check repeats,
        /// in seconds. A point that has not settled by then is reported with
        /// a WARNING naming it (its row is the last measurement). 0 turns the
        /// check off: one measurement after --preroll-secs. Otherwise must be
        /// at least --preroll-secs. Off automatically with --noise.
        #[arg(long, value_name = "SECS", default_value = "2.0")]
        preroll_max_secs: f64,

        /// Pentode grid-off dimension reduction mode: auto, on, off.
        /// Mirrors `compile --tube-grid-fa`. See `simulate --tube-grid-fa` for
        /// the auto/on/off semantics. Defaults to `auto`, which is reserved
        /// for provably-neutral reductions and currently keeps the full 3D
        /// model (== off).
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto")]
        tube_grid_fa: String,

        /// Solver: auto (default), dk, nodal. Mirrors `compile --solver`
        /// (see `melange compile --help` for how auto chooses).
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto", value_parser = SOLVER_VALUES)]
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
        /// spectral radius). A nodal build never ships less than 100 and
        /// refuses a pin below it.
        #[arg(help_heading = EXPERT_HEADING, long)]
        max_iter: Option<usize>,

        /// Render even when the circuit was not driven with the requested
        /// input: samples beyond the generated code's input limit
        /// (`INPUT_LIMIT_V`, 100 V) are clamped to it, and NaN/Inf samples are
        /// replaced by 0. Both are counted, and the command fails on either by
        /// default, because the output then answers a different question.
        #[arg(help_heading = EXPERT_HEADING, long)]
        allow_input_clamp: bool,

        /// Report points whose render was not a solution. A point where any
        /// sample was held (every Newton path failed), committed unconverged,
        /// or solved on a reduced device model outside its region (a
        /// `.linearize`d stage driven out of its small-signal region) is
        /// refused by default, naming the point and the counter, the way
        /// `simulate` refuses the same render: its gain and THD describe the
        /// solver's fallback, not the circuit.
        #[arg(help_heading = EXPERT_HEADING, long)]
        allow_nr_hold: bool,
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
        #[arg(help_heading = EXPERT_HEADING, long, default_value = "auto", value_parser = SOLVER_VALUES)]
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
    /// (`yourrepo:fuzz-pedal`) instead of by path, which means you can reorganise
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
pub(crate) enum SourceAction {
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
pub(crate) enum CacheAction {
    /// Show cache contents
    List,

    /// Clear cached circuits and compiled binaries
    Clear {
        /// Clear only the compiled simulate/analyze binaries, keeping the
        /// downloaded circuits
        #[arg(long)]
        binaries: bool,
    },

    /// Show cache statistics
    Stats,
}

#[derive(ValueEnum, Clone, Debug, PartialEq)]
pub(crate) enum OutputFormat {
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
pub(crate) enum ImportFormat {
    /// Auto-detect from file content
    Auto,
    /// KiCad XML intermediate netlist (full fidelity)
    Xml,
    /// KiCad SPICE netlist (best-effort)
    Spice,
}

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
pub(crate) fn full_version_string() -> String {
    let base = version_label();
    match melange_solver::build_identity::current_exe_hash() {
        // Algorithm-qualified: `fnv1a64:` states the digest so a reader cannot
        // compare it against a different hash of the same file.
        Some(hash) => format!("melange {base} exe fnv1a64:{hash}"),
        None => format!("melange {base}"),
    }
}
