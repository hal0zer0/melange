//! SPICE-to-Rust validation pipeline.
//!
//! Layer 4 of the melange stack. Compares melange solver output against ngspice
//! reference simulations with multiple error metrics (correlation, RMS error,
//! peak error, spectral comparison).
//!
//! # Modules
//!
//! - [`spice_runner`] — invoke ngspice, parse output (raw/CSV), manage temp files
//! - [`comparison`] — signal comparison with configurable tolerances
//! - [`visualizer`] — generate CSV, HTML, and JSON reports from comparison results
//!
//! Requires ngspice to be installed (`apt install ngspice` / `brew install ngspice`).
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use std::path::Path;
//! use melange_validate::{
//!     validate_circuit,
//!     spice_runner::is_ngspice_available,
//!     comparison::ComparisonConfig,
//! };
//!
//! // Check if ngspice is available
//! if !is_ngspice_available() {
//!     println!("ngspice not installed, skipping validation");
//!     return;
//! }
//!
//! // Run validation
//! let input_signal = vec![0.0; 44100]; // 1 second of samples
//! let result = validate_circuit(
//!     Path::new("tests/data/rc_lowpass/circuit.cir"),
//!     &input_signal,
//!     44100.0,
//!     "out",
//!     &ComparisonConfig::default(),
//! ).expect("Validation failed");
//!
//! println!("{}", result.report.summary());
//! assert!(result.report.passed);
//! ```

use std::path::Path;
use thiserror::Error;

pub mod comparison;
pub mod deck_guard;
pub(crate) mod opamp_translate;
pub(crate) mod pentode_translate;
pub mod spice_runner;
pub(crate) mod tube_translate;
pub mod visualizer;

pub use comparison::{batch_compare, compare_signals, ComparisonConfig, ComparisonReport, Signal};
pub use deck_guard::{format_refusal, scan_deck, unit_variation_note, DeckHazard};
pub use spice_runner::{
    run_transient, run_transient_with_pwl, run_transient_with_thevenin_pwl, SpiceData, SpiceError,
};
pub use visualizer::{generate_csv, generate_html_report, generate_json_report};

/// Errors that can occur during validation
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ValidationError {
    /// SPICE simulation failed
    #[error("SPICE error: {0}")]
    Spice(#[source] SpiceError),

    /// The deck does not describe the same circuit to melange and to ngspice,
    /// so no comparison between them would mean anything.
    ///
    /// Raised before ngspice runs. Separate from [`ValidationError::Spice`]
    /// because it is not ngspice reporting a problem — it is melange declining
    /// to produce a number it could not stand behind. See
    /// [`crate::deck_guard`].
    #[error("{0}")]
    DeckNotComparable(String),

    /// Solver error from melange-solver
    #[error("Solver error: {0}")]
    Solver(String),

    /// IO error during file operations
    #[error("IO error: {0}")]
    Io(String),

    /// Invalid input parameters
    #[error("Invalid input: {0}")]
    InvalidInput(String),

    /// Comparison failed (signals differ beyond tolerance)
    #[error("Comparison failed: {0}")]
    ComparisonFailed(String),
}

impl From<SpiceError> for ValidationError {
    /// Keep the pre-flight refusal out of the `SPICE error:` bucket. The deck
    /// guard rejects before ngspice is invoked, so labeling its message as
    /// ngspice output would point the reader at the wrong engine — the same
    /// misdirection the guard exists to remove.
    fn from(e: SpiceError) -> Self {
        match e {
            SpiceError::DeckNotComparable(msg) => ValidationError::DeckNotComparable(msg),
            other => ValidationError::Spice(other),
        }
    }
}

impl From<std::io::Error> for ValidationError {
    fn from(e: std::io::Error) -> Self {
        ValidationError::Io(e.to_string())
    }
}

impl From<visualizer::VisualizerError> for ValidationError {
    fn from(e: visualizer::VisualizerError) -> Self {
        ValidationError::Io(e.to_string())
    }
}

/// Result of a validation run
#[derive(Debug, Clone)]
pub struct ValidationResult {
    /// The comparison report with all metrics
    pub report: ComparisonReport,
    /// Path to generated HTML report (if created)
    pub html_report_path: Option<std::path::PathBuf>,
    /// Path to generated CSV data (if created)
    pub csv_path: Option<std::path::PathBuf>,
    /// Path to generated JSON report (if created)
    pub json_path: Option<std::path::PathBuf>,
}

impl ValidationResult {
    /// Check if validation passed all tolerance checks
    pub fn passed(&self) -> bool {
        self.report.passed
    }

    /// Get a summary of the validation results
    pub fn summary(&self) -> String {
        self.report.summary()
    }
}

/// Options for validation runs
#[derive(Debug, Clone)]
pub struct ValidationOptions {
    /// Generate HTML report on failure
    pub generate_html_on_failure: bool,
    /// Generate HTML report on success
    pub generate_html_on_success: bool,
    /// Always generate CSV output
    pub generate_csv: bool,
    /// Always generate JSON output
    pub generate_json: bool,
    /// Output directory for generated files
    pub output_dir: Option<std::path::PathBuf>,
    /// Time step for SPICE transient analysis (auto if None)
    pub tstep: Option<f64>,
    /// Custom name for the circuit
    pub circuit_name: Option<String>,
    /// Additional nodes to capture (besides output_node)
    pub additional_nodes: Vec<String>,
    /// Input node name (default: "in")
    pub input_node: String,
    /// Forward-active BJT reduction mode — mirrors `melange compile --bjt-fa`.
    ///
    /// Defaults to `Auto`, matching the shipped build. Until 2026-09-03 this
    /// harness applied NO forward-active reduction, so it validated a
    /// full-2D system for circuits the CLI ships reduced (measured on
    /// `wurli_preamp`: shipped M=3, harness M=5).
    pub bjt_fa_mode: melange_solver::codegen::BjtFaMode,
    /// Grid-off pentode reduction mode — mirrors `melange compile
    /// --tube-grid-fa` (`auto` | `on` | `off`). Defaults to `auto`.
    pub tube_grid_fa: String,
    /// Force Backward Euler on the melange side — mirrors `melange compile
    /// --backward-euler`. DIAGNOSTIC only (attribute integrator error, like
    /// `--bjt-fa off`); default `false` keeps the shipped auto selection.
    pub backward_euler: bool,
    /// Force trapezoidal on the melange side — mirrors `melange compile
    /// --force-trap`. DIAGNOSTIC only; ignored when `backward_euler` is true.
    pub force_trap: bool,
    /// Oversampling factor for the melange side — mirrors `melange compile
    /// --oversampling` (1 | 2 | 4). Default 1.
    ///
    /// This is NOT a diagnostic: `--oversampling` is a compile-time codegen
    /// option, so a plugin built at 2x contains DIFFERENT DSP from the 1x
    /// build (interpolator, solver at the internal rate, polyphase half-band
    /// decimator). Without this knob the harness could only ever validate the
    /// 1x code for a build that ships at 2x or 4x.
    ///
    /// The ngspice side is unchanged — ngspice has its own timestep and knows
    /// nothing about melange's internal rate. Instead, the REFERENCE is passed
    /// through the same half-band round trip the shipped build applies (see
    /// [`apply_oversampling_round_trip`]), so the filters' known response is
    /// included in the comparison rather than absorbed by a widened tolerance.
    pub oversampling: usize,
}

impl Default for ValidationOptions {
    fn default() -> Self {
        Self {
            generate_html_on_failure: true,
            generate_html_on_success: false,
            generate_csv: false,
            generate_json: false,
            output_dir: None,
            tstep: None,
            circuit_name: None,
            additional_nodes: Vec::new(),
            input_node: "in".to_string(),
            bjt_fa_mode: melange_solver::codegen::BjtFaMode::Auto,
            tube_grid_fa: "auto".to_string(),
            backward_euler: false,
            force_trap: false,
            oversampling: 1,
        }
    }
}

/// High-level validation function
///
/// Runs ngspice on the provided netlist, runs the melange solver with the same input,
/// and compares the results against configurable tolerances.
///
/// # Arguments
///
/// * `netlist_path` - Path to the SPICE netlist file
/// * `input_signal` - Input signal samples (will be used as PWL source)
/// * `sample_rate` - Sample rate in Hz
/// * `output_node` - Name of the output node to compare
/// * `config` - Comparison configuration with tolerances
///
/// # Returns
///
/// Returns `ValidationResult` containing the comparison report and paths to any
/// generated files, or a `ValidationError` if validation fails.
///
/// # Example
///
/// ```rust,no_run
/// use std::path::Path;
/// use melange_validate::{validate_circuit, comparison::ComparisonConfig};
///
/// // Create a simple sine wave input
/// let input: Vec<f64> = (0..4410)
///     .map(|i| (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 44100.0).sin())
///     .collect();
///
/// let result = validate_circuit(
///     Path::new("tests/data/rc_lowpass/circuit.cir"),
///     &input,
///     44100.0,
///     "out",
///     &ComparisonConfig::default(),
/// ).expect("Validation failed");
///
/// assert!(result.passed());
/// ```
pub fn validate_circuit(
    netlist_path: &Path,
    input_signal: &[f64],
    sample_rate: f64,
    output_node: &str,
    config: &ComparisonConfig,
) -> Result<ValidationResult, ValidationError> {
    let options = ValidationOptions::default();
    validate_circuit_with_options(
        netlist_path,
        input_signal,
        sample_rate,
        output_node,
        config,
        &options,
    )
}

/// Validate a circuit with detailed options
///
/// This is the full-featured version of `validate_circuit` that allows
/// customizing the validation process.
///
/// # Example
///
/// ```rust,no_run
/// use std::path::Path;
/// use melange_validate::{
///     validate_circuit_with_options,
///     comparison::ComparisonConfig,
///     ValidationOptions,
/// };
///
/// let input = vec![0.0; 44100];
///
/// let options = ValidationOptions {
///     generate_html_on_failure: true,
///     generate_csv: true,
///     output_dir: Some(Path::new("./validation_output").to_path_buf()),
///     ..Default::default()
/// };
///
/// let result = validate_circuit_with_options(
///     Path::new("circuit.cir"),
///     &input,
///     44100.0,
///     "out",
///     &ComparisonConfig::strict(),
///     &options,
/// ).expect("Validation failed");
/// ```
pub fn validate_circuit_with_options(
    netlist_path: &Path,
    input_signal: &[f64],
    sample_rate: f64,
    output_node: &str,
    config: &ComparisonConfig,
    options: &ValidationOptions,
) -> Result<ValidationResult, ValidationError> {
    if input_signal.is_empty() {
        return Err(ValidationError::InvalidInput(
            "Input signal is empty".to_string(),
        ));
    }

    // Check if ngspice is available
    if !spice_runner::is_ngspice_available() {
        return Err(ValidationError::Spice(SpiceError::NgspiceNotFound));
    }

    let input_node = &options.input_node;

    // Read the netlist file once
    let netlist_str = std::fs::read_to_string(netlist_path)
        .map_err(|e| ValidationError::Io(format!("Failed to read netlist: {}", e)))?;

    // Strip VIN for melange (auto-detect and remove input voltage source)
    let (stripped_netlist, dc_offset) = strip_vin_source(&netlist_str, input_node);
    if let Some(dc) = dc_offset {
        if dc.abs() > 1e-12 {
            log::warn!(
                "VIN has DC offset of {:.3}V — melange will not reproduce this offset",
                dc
            );
        }
    }

    // Calculate timing parameters
    let duration = input_signal.len() as f64 / sample_rate;
    let tstep = options.tstep.unwrap_or(1.0 / sample_rate);

    // Build PWL data from input signal
    let pwl_data: Vec<(f64, f64)> = input_signal
        .iter()
        .enumerate()
        .map(|(i, &v)| (i as f64 / sample_rate, v))
        .collect();

    // Run SPICE simulation with Thevenin PWL (matched 1-ohm source impedance)
    let mut nodes_to_capture = vec![output_node.to_string()];
    nodes_to_capture.extend(options.additional_nodes.clone());

    let spice_data = spice_runner::run_transient_with_thevenin_pwl(
        &netlist_str,
        tstep,
        duration,
        input_node,
        &pwl_data,
        1.0, // 1 ohm series resistance matching melange's Thevenin model
        &nodes_to_capture,
    )?;

    // Extract the output signal from SPICE results
    let spice_output = spice_data
        .get_node_voltage(output_node)
        .map_err(ValidationError::from)?;

    // Run melange solver on stripped netlist (VIN removed)
    let melange_output = run_melange_solver_from_str(
        &stripped_netlist,
        input_signal,
        sample_rate,
        output_node,
        input_node,
        options.bjt_fa_mode,
        &options.tube_grid_fa,
        options.backward_euler,
        options.force_trap,
        options.oversampling,
        None,
    )?;

    // Apply DC blocking to SPICE output to match melange's internal DC blocker (5 Hz HPF)
    let mut spice_output_blocked = spice_output.to_vec();
    dc_block_signal(&mut spice_output_blocked, spice_data.sample_rate);

    // Oversampled build: put the reference through the SAME half-band round
    // trip the shipped code applies, so the filters' known (magnitude-flat,
    // all-phase) response is part of the comparison. Not a tolerance change —
    // see `apply_oversampling_round_trip` for what this does and does not
    // cover. No-op at factor 1, which is every default run.
    //
    // Order: after the DC blocker, so the blocker still sees the reference's
    // own first sample as its seed (the startup-transient fix documented on
    // `dc_block_signal`). Both are LTI, so they commute up to boundary effects.
    apply_oversampling_round_trip(
        &mut spice_output_blocked,
        options.oversampling,
        spice_data.sample_rate,
    );

    // Create signal objects for comparison
    let spice_signal = Signal::new(
        spice_output_blocked,
        spice_data.sample_rate,
        format!("spice_{}", output_node),
    );
    let melange_signal = Signal::new(
        melange_output,
        sample_rate,
        format!("melange_{}", output_node),
    );

    // Compare signals
    let mut report = compare_signals(&spice_signal, &melange_signal, config);
    report.circuit_name = options.circuit_name.clone().unwrap_or_else(|| {
        netlist_path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown")
            .to_string()
    });
    report.node_name = output_node.to_string();
    // The melange side was built with unit variation off (see
    // `run_melange_solver_from_str`). When the deck carries live `.tolerance` /
    // `.mismatch`, say so ON the result line, naming the directives and the
    // seed that was therefore not exercised. `None` — and so no added output —
    // for every deck without them.
    report.unit_variation_note = deck_guard::unit_variation_note(&netlist_str);
    // Say on the report which build was validated, and that the reference was
    // put through the same filters. A correlation number for an oversampled
    // build is not comparable to a 1x one, and a reader who is not told will
    // assume it is.
    report.oversampling_note = (options.oversampling > 1).then(|| {
        format!(
            "oversampling {}x (internal rate {:.0} Hz); reference passed through the same \
             half-band round trip (allpass, group delay {:.2} samples at 1 kHz)",
            options.oversampling,
            sample_rate * options.oversampling as f64,
            oversampling_round_trip_group_delay_samples(options.oversampling, sample_rate, 1000.0),
        )
    });

    // Generate output files if requested
    let mut html_report_path = None;
    let mut csv_path = None;
    let mut json_path = None;

    let should_generate_html = (report.passed && options.generate_html_on_success)
        || (!report.passed && options.generate_html_on_failure);

    if should_generate_html || options.generate_csv || options.generate_json {
        let output_dir = options
            .output_dir
            .clone()
            .unwrap_or_else(std::env::temp_dir);

        std::fs::create_dir_all(&output_dir)?;

        let base_name = format!(
            "{}_{}",
            report.circuit_name,
            if report.passed { "passed" } else { "failed" }
        );

        if should_generate_html {
            let html_path = output_dir.join(format!("{}.html", base_name));
            visualizer::generate_html_report(&report, &spice_signal, &melange_signal, &html_path)?;
            html_report_path = Some(html_path);
        }

        if options.generate_csv {
            let csv_file_path = output_dir.join(format!("{}.csv", base_name));
            visualizer::generate_csv(&spice_signal, &melange_signal, &csv_file_path)?;
            csv_path = Some(csv_file_path);
        }

        if options.generate_json {
            let json_file_path = output_dir.join(format!("{}.json", base_name));
            visualizer::generate_json_report(&report, &json_file_path)?;
            json_path = Some(json_file_path);
        }
    }

    Ok(ValidationResult {
        report,
        html_report_path,
        csv_path,
        json_path,
    })
}

/// Apply a 5 Hz DC blocking HPF to a signal.
///
/// Seeds `x_prev` with the signal's own first sample (rather than 0) so the
/// filter starts already settled at the reference's DC operating point,
/// instead of injecting an artificial startup transient that decays at the
/// 5 Hz pole (tau ~32ms). This mirrors the generated code's DC blocker,
/// which seeds `dc_block_x_prev` from the compile-time `DC_OP[out]` value
/// (`state.rs.tera`, "Seed the DC blocker's x[n-1] with the output-node DC
/// operating point") specifically to avoid that startup thump. Without this
/// seed, `dc_block_signal` and the generated code apply non-equivalent
/// filters whenever the raw output has a non-zero DC bias (e.g. an amplifier
/// stage with no output coupling cap): the reference exhibits a multi-volt
/// decaying transient over the comparison window while melange's output is
/// already settled, producing a spurious low-correlation "regression" that
/// is a test-harness artifact, not a solver bug. See wurli-preamp
/// (~9.1V raw DC bias, no output coupling cap).
///
/// This is the single implementation shared by the library validation path
/// and the `spice_validation` test harness — keep them unified.
pub fn dc_block_signal(signal: &mut [f64], sample_rate: f64) {
    let r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / sample_rate;
    let mut x_prev = signal.first().copied().unwrap_or(0.0);
    let mut y_prev = 0.0f64;
    for sample in signal.iter_mut() {
        let x = *sample;
        let y = x - x_prev + r * y_prev;
        x_prev = x;
        y_prev = y;
        *sample = y;
    }
}

/// Pass a reference signal through the SAME half-band round trip an
/// oversampled melange build applies, so the filters' known response is part
/// of the comparison instead of being charged to the solver.
///
/// # Why the reference is filtered and the tolerances are not
///
/// `--oversampling {2|4}` is a compile-time codegen option: the shipped build
/// upsamples, solves at the internal rate, and decimates through polyphase
/// half-band IIR allpass chains. Those filters have a real response, and it
/// ships, so it is a true difference between melange's output and the circuit.
/// The project's rule for it is to *include the known filter response in the
/// comparison*, never to widen an anchor around an effect whose size is known
/// analytically.
///
/// That is what this does. With the circuit replaced by an identity, the 2x
/// chain `up -> identity -> down` composes to the pure allpass
/// `A0(z) * A1(z)` in the HOST-rate `z` (each branch cell is clocked exactly
/// once per host sample) — magnitude-flat to the last bit, all response in the
/// phase. The 4x chain is the same statement nested. Applying it to the
/// ngspice output puts both signals through the same known linear response, so
/// what is left on the result line is the part that is not the filters.
///
/// The dominant term it removes is group delay. At 48 kHz the 2x round trip
/// delays by order a host sample; against a 1 kHz tone that alone costs
/// `1 - cos(2*pi*1000/48000) ~ 8.6e-3` of correlation, roughly four hundred
/// times the whole 48 kHz solver residual. Comparing an oversampled build
/// against an unfiltered reference measures that delay and almost nothing
/// else.
///
/// # What it does NOT cover
///
/// The round trip commutes with the circuit exactly only when the circuit is
/// linear. For a nonlinear circuit `D(C(U(x)))` is not `A(C(x))`, and the
/// difference is real and stays in the number: the residual imaging/aliasing
/// the oversampling exists to suppress, and the effect of the interpolator's
/// phase on the waveform that reaches the nonlinearity. Neither is removed
/// here, and neither should be — both ship.
///
/// Nor does it touch the other half of what `--oversampling` changes: the
/// solver runs at `factor * sample_rate`, a finer timestep, which moves the
/// answer on its own. That shows up as a genuine (small) improvement against
/// the reference, not as an artifact.
///
/// # Twin-drift
///
/// The chain used here is `melange_primitives::oversampling::Oversampler`,
/// the same implementation the codegen emitter mirrors. The coefficients the
/// emitter actually bakes into generated code are pinned to the primitives'
/// tables by `oversampling_reference_matches_emitted_coefficients`
/// (`tests/oversampling_reference.rs`); if the twins ever drift, that test
/// fails rather than this function quietly compensating with the wrong filter.
///
/// `factor == 1` is a no-op. Panics on an unsupported factor — the caller
/// validates it first, and silently comparing against an uncompensated
/// reference would be worse than stopping.
pub fn apply_oversampling_round_trip(signal: &mut [f64], factor: usize, sample_rate: f64) {
    if factor == 1 {
        return;
    }
    let mut os = melange_primitives::oversampling::Oversampler::new(factor, sample_rate)
        .unwrap_or_else(|e| panic!("oversampling reference filter: {e}"));
    for sample in signal.iter_mut() {
        *sample = os.process(*sample, |x| x);
    }
}

/// Group delay of the oversampling round trip at `freq_hz`, in host samples.
///
/// Measured, not asserted: an impulse is pushed through the same chain
/// [`apply_oversampling_round_trip`] uses and the phase of its response at
/// `freq_hz` is read off. Reported on the result line so the number the
/// harness compensated for is visible rather than implied.
///
/// Returns 0.0 for `factor == 1`.
pub fn oversampling_round_trip_group_delay_samples(
    factor: usize,
    sample_rate: f64,
    freq_hz: f64,
) -> f64 {
    if factor == 1 {
        return 0.0;
    }
    // Impulse response, long enough for these allpass chains to decay.
    let n = 4096;
    let mut h = vec![0.0f64; n];
    h[0] = 1.0;
    apply_oversampling_round_trip(&mut h, factor, sample_rate);

    // H(e^{jw}) at the test frequency; the round trip is allpass, so |H| = 1
    // and -phase/w is the phase delay, which for a single tone is exactly the
    // shift the comparison would otherwise have seen.
    let w = 2.0 * std::f64::consts::PI * freq_hz / sample_rate;
    let (mut re, mut im) = (0.0f64, 0.0f64);
    for (k, &hk) in h.iter().enumerate() {
        let a = w * k as f64;
        re += hk * a.cos();
        im -= hk * a.sin();
    }
    let phase = im.atan2(re); // in (-pi, pi]
    let mut delay = -phase / w;
    // Unwrap into the positive branch: these chains are causal and delay by
    // more than a sample at 4x, so a phase that has wrapped reads negative.
    let period_samples = sample_rate / freq_hz;
    while delay < 0.0 {
        delay += period_samples;
    }
    delay
}

/// Strip the input voltage source from a netlist string
///
/// Scans for a voltage source whose n+ terminal matches `input_node` (case-insensitive)
/// and removes it. Returns the modified netlist and any DC value found on the source.
///
/// This allows a single circuit file to work for both ngspice (which needs VIN)
/// and melange (which models input via conductance stamping).
pub fn strip_vin_source(netlist: &str, input_node: &str) -> (String, Option<f64>) {
    let input_upper = input_node.to_uppercase();
    let mut lines = Vec::new();
    let mut dc_value = None;
    let mut stripped = false;

    for (i, line) in netlist.lines().enumerate() {
        let trimmed = line.trim();

        // Line 0 is ALWAYS the free-text SPICE title, never an element. A title
        // whose 2nd token happens to equal the input node (e.g. "Valve in
        // preamp", "Voltage in stage") would otherwise be mistaken for VIN and
        // stripped, corrupting the deck. tube_translate.rs guards this same class.
        if i == 0 {
            lines.push(line.to_string());
            continue;
        }

        // Keep commented lines
        if trimmed.starts_with('*') {
            lines.push(line.to_string());
            continue;
        }

        let trimmed_upper = trimmed.to_uppercase();

        // Check if this is a voltage source with n+ matching input_node
        if trimmed_upper.starts_with('V') {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 3 && parts[1].to_uppercase() == input_upper {
                if stripped {
                    // Multiple voltage sources at input node — warn and keep extras
                    log::warn!(
                        "Multiple voltage sources at input node '{}'; keeping '{}'",
                        input_node,
                        parts[0]
                    );
                } else {
                    // Extract DC value if present (e.g., "VIN in 0 DC 5.0")
                    for (i, part) in parts.iter().enumerate() {
                        if part.to_uppercase() == "DC" && i + 1 < parts.len() {
                            dc_value = parts[i + 1].parse::<f64>().ok();
                            break;
                        }
                    }
                    // Strip first match only
                    stripped = true;
                    continue;
                }
            }
        }

        lines.push(line.to_string());
    }

    (lines.join("\n"), dc_value)
}

/// Run melange solver on a netlist string with the given input signal
///
/// Accepts a netlist string (e.g., after VIN stripping) and an explicit input node name.
/// Handles both linear and nonlinear circuits (with DC OP initialization for the latter).
/// Run the melange solver on a netlist string, through the SAME front-end the
/// shipped CLI uses.
///
/// `main_code` overrides the default stdin-samples-in / stdout-samples-out
/// driver; pass `None` for it. The SPICE test harness passes its own so it can
/// drive `set_pot_0(..)` per sample.
///
/// **This is the single implementation on purpose.** Until 2026-09-03 the
/// integration tests in `tests/spice_validation.rs` carried a second, silently
/// divergent copy of this routine — no `.linearize`, no forward-active or
/// grid-off reduction, unconditional internal-node expansion and the default
/// `MAX_ITER` of 100 rather than `auto_tune_max_iter`. That copy is what the
/// CI "SPICE validation" gate actually ran, so the gate was not measuring the
/// shipped build. Route new callers here rather than growing a third.
#[allow(clippy::too_many_arguments)]
pub fn run_melange_solver_from_str(
    netlist_str: &str,
    input_signal: &[f64],
    sample_rate: f64,
    output_node_name: &str,
    input_node_name: &str,
    bjt_fa_mode: melange_solver::codegen::BjtFaMode,
    tube_grid_fa: &str,
    backward_euler: bool,
    force_trap: bool,
    oversampling: usize,
    main_code: Option<&str>,
) -> Result<Vec<f64>, ValidationError> {
    use melange_solver::codegen::{routing, CodeGenerator, CodegenConfig};
    use std::io::Write;

    if !matches!(oversampling, 1 | 2 | 4) {
        return Err(ValidationError::InvalidInput(format!(
            "oversampling must be 1, 2, or 4, got {oversampling}"
        )));
    }
    // The solver runs at the INTERNAL rate under oversampling, so every
    // rate-dependent decision below — kernel build, routing, the reduction
    // gates — is made at `routing_rate`, exactly as `melange compile` and
    // `melange simulate` do it. Equals `sample_rate` when oversampling is off.
    let routing_rate = sample_rate * oversampling as f64;

    // Unit variation OFF on the melange side, unconditionally.
    //
    // `.tolerance` jitters fixed R/C/L values in the parser and `.mismatch`
    // jitters device model parameters in codegen — both on melange's side only.
    // The reference deck handed to ngspice carries the values as written, so a
    // jittered melange side puts a correlation between two DIFFERENT circuits
    // on the result line and attributes the gap to the solver. Measured on
    // `examples/passive-eq1a.cir` (`.seed 4142` / `.mismatch T MU=0.09
    // KG1=0.20`): 16 emitted device constants differ, `DEVICE_0_MU` by -7.6%
    // and `DEVICE_0_KG1` by -5.1%.
    //
    // This is what the docs already told authors to do by hand
    // (`docs/aidocs/UNIT_VARIATION.md`, `docs/limitations.md`); only the
    // automation was missing. It matches what validate measures: the solver
    // against a reference engine at the SAME component values. Jitter changes
    // values, not the solver, and whether the draw itself is correct is a
    // unit-test question ngspice cannot answer — see
    // `melange_solver::parser::tests::tolerance_draw_matches_nominal_times_one_plus_tol_u`.
    //
    // The caller names the disabled directives on the result line via
    // `deck_guard::unit_variation_note`; it is not a preamble, because a
    // footnote above a number does not retract the number.
    //
    // Unconditional here rather than an option on `ValidationOptions`: this
    // function IS the melange side of every comparison, so there is no caller
    // for whom the jittered answer would be the honest one.
    let parse_options = melange_solver::parser::ParseOptions {
        disable_unit_variation: true,
    };
    let netlist = melange_solver::parser::Netlist::parse_with_options(netlist_str, parse_options)
        .map_err(|e| {
        ValidationError::Solver(format!("Parse error at line {}: {}", e.line, e.message))
    })?;

    // Topology gate — the same one `melange compile` runs, on the same shared
    // implementation. A deck with an inert element validates a circuit the
    // author did not write, and a correlation number printed for the wrong
    // circuit is worse than no number. Warnings go to the log; refusals stop
    // the run here, before ngspice's answer can be compared to anything.
    melange_solver::pipeline::topology_gate(
        &netlist,
        &melange_solver::topology::Ports::declared(
            [input_node_name.to_string()],
            [output_node_name.to_string()],
        ),
        &|m| log::warn!("{m}"),
    )
    .map_err(|e| ValidationError::Solver(e.to_string()))?;

    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .map_err(|e| ValidationError::Solver(format!("MNA error: {}", e)))?;

    let input_node = mna
        .node_map
        .get(input_node_name)
        .copied()
        .ok_or_else(|| {
            ValidationError::Solver(format!(
                "Input node '{}' not found. Available: {:?}",
                input_node_name,
                mna.node_names_in_index_order()
            ))
        })?
        .saturating_sub(1);
    let output_node = mna
        .node_map
        .get(output_node_name)
        .copied()
        .ok_or_else(|| {
            ValidationError::Solver(format!(
                "Output node '{}' not found. Available: {:?}",
                output_node_name,
                mna.node_names_in_index_order()
            ))
        })?
        .saturating_sub(1);

    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }

    // Apply `.linearize` — the SAME shared pipeline step `melange compile` runs.
    //
    // This harness used to skip it entirely, which is how `wurli-power-amp` came
    // to "fail" validation at 1319% RMS and correlation 0.0002: without a
    // linearized device the emitter's `linearized_bypass` gate never fires, so
    // it chose Schur NR instead of full-LU and diverged on the first non-zero
    // sample. The shipped build validates at 0.228% RMS, correlation 1.000000.
    // A validator that builds a different circuit than the one it validates is
    // worse than no validator, because it is believed.
    //
    // Forward-active + grid-off reductions, then `.linearize` — the SAME three
    // shared pipeline steps, in the same order, that `melange compile` runs.
    //
    // Until 2026-09-03 this harness passed EMPTY forward-active and grid-off
    // sets, so it validated a full-2D system for any circuit the shipped build
    // reduces. Measured on the `wurli_preamp` validation deck: shipped M=3,
    // harness M=5 — a different circuit, and therefore not a statement about
    // what ships. `--bjt-fa` / `--tube-grid-fa` still select the mode, so
    // `off` remains available to attribute a residual to the reduction.
    let fa_config = melange_solver::codegen::CodegenConfig {
        circuit_name: "fa_detect".to_string(),
        sample_rate,
        input_resistance: 1.0,
        input_node,
        output_nodes: vec![output_node],
        bjt_fa_mode,
        ..melange_solver::codegen::CodegenConfig::default()
    };
    let forward_active = melange_solver::pipeline::apply_forward_active_reduction(
        &mut mna,
        &netlist,
        &fa_config,
        "auto",
        sample_rate,
        oversampling,
        input_node,
        1.0,
        &melange_solver::pipeline::silent,
    )
    .map_err(|e| ValidationError::Solver(format!("forward-active: {e}")))?;

    let grid_off_pentodes = melange_solver::pipeline::apply_grid_off_reduction(
        &mut mna,
        &netlist,
        &fa_config,
        &forward_active,
        tube_grid_fa,
        "auto",
        sample_rate,
        oversampling,
        input_node,
        1.0,
    )
    .map_err(|e| ValidationError::Solver(format!("grid-off: {e}")))?;

    melange_solver::pipeline::apply_linearize_reductions(
        &mut mna,
        &netlist,
        &forward_active,
        &grid_off_pentodes,
        input_node,
        1.0,
        1.0,
        &melange_solver::pipeline::silent,
    )
    .map_err(|e| ValidationError::Solver(format!("linearize: {e}")))?;

    // Stamp junction caps + pre-solve DC OP so BJT charge-storage caps are
    // linearized at the true operating point. When all BJTs use the default
    // CJE/CJC/TF parameters this is byte-identical to the zero-bias stamp;
    // when `.model` cards carry TF/VJE/etc. the kernel now sees the ngspice
    // depletion + diffusion cap values rather than the zero-bias shape.
    //
    // Uses `CircuitIR::build_device_info_with_mna` rather than the harness's
    // bare-minimum `build_device_slots_from_netlist` so the BJT `.model`
    // card's CJE/CJC/TF/VJE/MJE/VJC/MJC/FC are actually read. The original
    // harness builder hardcoded these to zero, which defeated the fix.
    //
    // The pre-solved DC OP is forwarded to `generate_with_dc_op` below so
    // we don't double-solve.
    let dc_preflight = {
        let device_slots = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
            &netlist,
            Some(&mna),
        )
        .unwrap_or_default();
        if device_slots.is_empty() {
            None
        } else {
            let dc_config = melange_solver::dc_op::DcOpConfig {
                input_node,
                input_resistance: 1.0,
                ..melange_solver::dc_op::DcOpConfig::default()
            };
            Some(mna.stamp_caps_and_solve_dc_op(&device_slots, &dc_config))
        }
    };

    // Build kernel and route
    let has_inductors = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();
    let mut dk_failed = false;
    let kernel = if has_inductors {
        melange_solver::dk::DkKernel::from_mna_augmented(&mna, routing_rate)
            .map_err(|e| ValidationError::Solver(format!("Augmented DK: {:?}", e)))?
    } else {
        match melange_solver::dk::DkKernel::from_mna(&mna, routing_rate) {
            Ok(k) => k,
            Err(_) => {
                dk_failed = true;
                melange_solver::dk::DkKernel::from_mna_augmented(&mna, routing_rate)
                    .map_err(|e| ValidationError::Solver(format!("DK fallback: {:?}", e)))?
            }
        }
    };

    let decision = routing::auto_route(&kernel, &mna, dk_failed);
    let use_nodal = decision.route == routing::SolverRoute::Nodal;

    if use_nodal {
        // K-gated, exactly as the CLI does it. Expanding unconditionally is what
        // pushed this harness onto Schur-with-expanded-parasitics, which
        // diverges where the shipped full-LU build converges.
        melange_solver::pipeline::expand_internal_nodes_if_conditioned(
            &mut mna,
            &netlist,
            &kernel,
            &melange_solver::pipeline::silent,
        );
    }

    // Post-DC-block output ceiling (default 10 V, see docs/aidocs/SIGNAL_LEVELS.md
    // "Signal Level Contract"). Sized for line-level circuits; a circuit whose DC
    // operating point carries a high-voltage rail (e.g. a 250 V tube B+) can
    // legitimately swing its output node tens of volts under large-signal drive.
    // A fixed 10 V ceiling silently hard-clips that into a square wave, which
    // then reads as a large melange-vs-ngspice divergence that is actually a
    // harness/config gap, not a solver bug (see triode_cc overdrive
    // investigation, 2026-08). Auto-scale from the DC operating point's node
    // voltage headroom (`dc_preflight`, already computed above) so any
    // high-rail circuit validated through this path gets a ceiling that won't
    // clip a legitimate large-signal swing; never lower it below the existing
    // 10 V default so line-level circuits keep their historical clamp
    // behavior byte-for-byte.
    let auto_clamp_v = dc_preflight
        .as_ref()
        .map(|dc| {
            dc.v_node
                .iter()
                .cloned()
                .fold(0.0_f64, |acc, v| acc.max(v.abs()))
                * 3.0
        })
        .unwrap_or(0.0)
        .max(CodegenConfig::default().output_clamp_v);

    let config = CodegenConfig {
        circuit_name: "validate".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        dc_block: true,
        router_dk_unstable: decision.dk_unstable,
        router_dk_spectral_radius: decision.spectral_radius,
        output_clamp_v: auto_clamp_v,
        // Same budget the shipped build gets; the default 100 is not what ships.
        max_iterations: melange_solver::pipeline::auto_tune_max_iter(
            None, &kernel, &decision, false, false, input_node,
        ),
        // Diagnostics (default: shipped behaviour — auto integrator).
        backward_euler,
        force_trap,
        // Compile-time DSP, NOT a diagnostic: at 2x/4x the emitted code
        // upsamples, solves at `routing_rate`, and decimates. The reference is
        // put through the same half-band round trip in
        // `validate_circuit_with_options` so the filters' known response is
        // included in the comparison rather than charged to the solver.
        oversampling_factor: oversampling,
        ..CodegenConfig::default()
    };
    let generator = CodeGenerator::new(config);
    let generated = if use_nodal {
        generator.generate_nodal(&mna, &netlist)
    } else {
        generator.generate_with_dc_op(&kernel, &mna, &netlist, dc_preflight)
    }
    .map_err(|e| ValidationError::Solver(format!("Codegen: {}", e)))?;

    // Append the driver main — caller-supplied, else stdin/stdout.
    // Diagnostics go to stderr as `DIAG:key=value` lines (same protocol as
    // the CLI's simulate driver) and are echoed below.
    let default_main = "fn main() {\n\
        let mut state = CircuitState::default();\n\
        let stdin = std::io::stdin();\n\
        let mut line = String::new();\n\
        loop {\n\
            line.clear();\n\
            if stdin.read_line(&mut line).unwrap() == 0 { break; }\n\
            if let Ok(input) = line.trim().parse::<f64>() {\n\
                let out = process_sample(input, &mut state);\n\
                println!(\"{:.15e}\", out[0]);\n\
            }\n\
        }\n\
        eprintln!(\"DIAG:nr_max_iter_count={}\", state.diag_nr_max_iter_count);\n\
        eprintln!(\"DIAG:region_exit_count={}\", state.diag_region_exit_count);\n\
    }\n";
    let full_source = format!("{}\n{}", generated.code, main_code.unwrap_or(default_main));

    // Compile
    static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let tmp_dir = std::env::temp_dir();
    let id = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let pid = std::process::id();
    let src = tmp_dir.join(format!("melange_val_{pid}_{id}.rs"));
    let bin = tmp_dir.join(format!("melange_val_{pid}_{id}"));

    std::fs::write(&src, &full_source)
        .map_err(|e| ValidationError::Solver(format!("Write: {}", e)))?;

    let compile = std::process::Command::new("rustc")
        .arg(&src)
        .arg("-o")
        .arg(&bin)
        .arg("--edition=2024")
        .arg("-O")
        .output()
        .map_err(|e| ValidationError::Solver(format!("rustc: {}", e)))?;
    let _ = std::fs::remove_file(&src);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin);
        return Err(ValidationError::Solver(format!(
            "Compilation failed:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        )));
    }

    // Run: pipe input via stdin
    let stdin_data: Vec<u8> = input_signal
        .iter()
        .map(|s| format!("{s:.15e}\n"))
        .collect::<String>()
        .into_bytes();
    let mut child = std::process::Command::new(&bin)
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| ValidationError::Solver(format!("Spawn: {}", e)))?;

    // Write stdin on a separate thread so `wait_with_output()` below drains
    // stdout/stderr concurrently. Otherwise, when both the input AND the
    // generated binary's output exceed the OS pipe buffer (~64 KB) and the
    // child interleaves reading stdin with writing stdout, `write_all` and the
    // child's stdout write deadlock against each other (the parent blocks
    // writing stdin while the child blocks writing a full stdout pipe that no
    // one is reading yet). Dropping the stdin handle at the end of the thread
    // closes the pipe, signalling EOF. A broken-pipe error here means the child
    // exited early — that surfaces via the exit-status check below, so it's
    // intentionally ignored.
    let stdin = child.stdin.take();
    let writer = std::thread::spawn(move || {
        if let Some(mut s) = stdin {
            let _ = s.write_all(&stdin_data);
        }
    });

    let result = child
        .wait_with_output()
        .map_err(|e| ValidationError::Solver(format!("Wait: {}", e)))?;
    let _ = writer.join();
    let _ = std::fs::remove_file(&bin);

    if !result.status.success() {
        return Err(ValidationError::Solver(format!(
            "Binary failed:\n{}",
            String::from_utf8_lossy(&result.stderr)
        )));
    }

    // Echo the driver's `DIAG:` lines so `melange validate` reports the
    // generated solver's counters (NR max-iter, region exits) next to the
    // comparison — a validation number without them hides a starved or
    // out-of-region solve.
    for line in String::from_utf8_lossy(&result.stderr).lines() {
        if let Some(diag) = line.strip_prefix("DIAG:") {
            eprintln!("  melange {}", diag.replacen('=', ": ", 1));
        }
    }

    Ok(String::from_utf8_lossy(&result.stdout)
        .lines()
        .filter_map(|l| l.trim().parse().ok())
        .collect())
}

/// Check if ngspice is available. Returns `true` if the test should be skipped.
///
/// Useful for tests that require ngspice to be installed.
///
/// # Example
///
/// ```rust,no_run
/// use melange_validate::should_skip_no_ngspice;
///
/// #[test]
/// fn test_validation() {
///     if should_skip_no_ngspice() {
///         return;
///     }
///     // ... rest of test
/// }
/// ```
pub fn should_skip_no_ngspice() -> bool {
    if !spice_runner::is_ngspice_available() {
        log::warn!("Skipping test: ngspice not available");
        return true;
    }
    false
}

/// Deprecated: use `should_skip_no_ngspice()` instead.
/// This function does NOT actually skip the test — it only prints a message.
#[deprecated(note = "Use should_skip_no_ngspice() which returns bool")]
pub fn skip_if_no_ngspice() {
    let _ = should_skip_no_ngspice();
}

/// Builder for validation runs
///
/// Provides a fluent API for configuring and running validations.
///
/// # Example
///
/// ```rust,no_run
/// use std::path::Path;
/// use melange_validate::ValidationBuilder;
///
/// let result = ValidationBuilder::new(Path::new("circuit.cir"))
///     .with_input(&vec![0.0; 44100])
///     .at_sample_rate(44100.0)
///     .measuring_node("out")
///     .with_strict_tolerances()
///     .generate_html_on_failure()
///     .run()
///     .expect("Validation failed");
/// ```
pub struct ValidationBuilder {
    netlist_path: std::path::PathBuf,
    input_signal: Option<Vec<f64>>,
    sample_rate: Option<f64>,
    output_node: Option<String>,
    config: ComparisonConfig,
    options: ValidationOptions,
}

impl ValidationBuilder {
    /// Create a new validation builder
    pub fn new(netlist_path: &Path) -> Self {
        Self {
            netlist_path: netlist_path.to_path_buf(),
            input_signal: None,
            sample_rate: None,
            output_node: None,
            config: ComparisonConfig::default(),
            options: ValidationOptions::default(),
        }
    }

    /// Set the input signal
    pub fn with_input(mut self, signal: &[f64]) -> Self {
        self.input_signal = Some(signal.to_vec());
        self
    }

    /// Set the sample rate
    pub fn at_sample_rate(mut self, rate: f64) -> Self {
        self.sample_rate = Some(rate);
        self
    }

    /// Set the output node to measure
    pub fn measuring_node(mut self, node: impl Into<String>) -> Self {
        self.output_node = Some(node.into());
        self
    }

    /// Use strict tolerances
    pub fn with_strict_tolerances(mut self) -> Self {
        self.config = ComparisonConfig::strict();
        self
    }

    /// Use relaxed tolerances
    pub fn with_relaxed_tolerances(mut self) -> Self {
        self.config = ComparisonConfig::relaxed();
        self
    }

    /// Generate HTML report on failure
    pub fn generate_html_on_failure(mut self) -> Self {
        self.options.generate_html_on_failure = true;
        self
    }

    /// Always generate HTML report
    pub fn always_generate_html(mut self) -> Self {
        self.options.generate_html_on_success = true;
        self.options.generate_html_on_failure = true;
        self
    }

    /// Generate CSV output
    pub fn generate_csv(mut self) -> Self {
        self.options.generate_csv = true;
        self
    }

    /// Set output directory for generated files
    pub fn output_to(mut self, dir: &Path) -> Self {
        self.options.output_dir = Some(dir.to_path_buf());
        self
    }

    /// Set custom circuit name
    pub fn named(mut self, name: impl Into<String>) -> Self {
        self.options.circuit_name = Some(name.into());
        self
    }

    /// Run the validation
    pub fn run(self) -> Result<ValidationResult, ValidationError> {
        let input_signal = self
            .input_signal
            .ok_or_else(|| ValidationError::InvalidInput("No input signal provided".to_string()))?;
        let sample_rate = self
            .sample_rate
            .ok_or_else(|| ValidationError::InvalidInput("No sample rate provided".to_string()))?;
        let output_node = self
            .output_node
            .ok_or_else(|| ValidationError::InvalidInput("No output node specified".to_string()))?;

        validate_circuit_with_options(
            &self.netlist_path,
            &input_signal,
            sample_rate,
            &output_node,
            &self.config,
            &self.options,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The "node not found" diagnostic used to list the available nodes by
    /// `HashMap` iteration, so the same failing run printed a different list
    /// every time. Ordering anything by hash iteration is a standing no
    /// (49ecaa4); `node_names_in_index_order()` gives the MNA index order,
    /// which is also the order the rest of the tooling reports nodes in.
    #[test]
    fn node_not_found_lists_nodes_in_a_stable_order() {
        let deck = "\
title
Rin in n1 1k
C1 n1 mid 100n
R2 mid out 4.7k
Rl out 0 10k
";
        let mut seen: Option<String> = None;
        for _ in 0..8 {
            let err = run_melange_solver_from_str(
                deck,
                &[0.0; 8],
                48000.0,
                "nosuchnode",
                "in",
                melange_solver::codegen::BjtFaMode::Auto,
                "auto",
                false,
                false,
                1,
                None,
            )
            .expect_err("missing output node must fail");
            let msg = err.to_string();
            assert!(msg.contains("nosuchnode"), "{msg}");
            match &seen {
                None => seen = Some(msg),
                Some(first) => assert_eq!(first, &msg, "node list order is not stable"),
            }
        }
        // MNA index order (ground first, then declaration order), not hash
        // order.
        let msg = seen.unwrap();
        assert!(msg.contains(r#"["0", "in", "n1", "mid", "out"]"#), "{msg}");
    }

    #[test]
    fn test_validation_builder() {
        // This test just verifies the builder compiles correctly
        // Actual validation would require a real netlist and ngspice
        let builder = ValidationBuilder::new(Path::new("test.cir"))
            .with_input(&vec![0.0; 100])
            .at_sample_rate(44100.0)
            .measuring_node("out")
            .with_strict_tolerances()
            .generate_html_on_failure()
            .generate_csv()
            .named("test_circuit");

        // Verify builder was constructed correctly
        assert_eq!(builder.config.rms_error_tolerance, 0.0001);
        assert!(builder.options.generate_html_on_failure);
        assert!(builder.options.generate_csv);
        assert_eq!(
            builder.options.circuit_name,
            Some("test_circuit".to_string())
        );
    }

    #[test]
    fn test_comparison_config_default() {
        let config = ComparisonConfig::default();
        assert!(config.rms_error_tolerance > 0.0);
        assert!(config.correlation_min > 0.99);
    }

    #[test]
    fn test_strip_vin_ignores_title_matching_input_node() {
        // The title's 2nd token is "in" (the input node). Before the line-0
        // guard, strip_vin_source mistook the title for VIN and removed it,
        // corrupting the deck. The title must survive; the real VIN on a later
        // line must still be stripped.
        let deck = "Valve in preamp\nVIN in 0 DC 0\nRin in n1 1k\nR1 n1 0 10k\n";
        let (stripped, _dc) = strip_vin_source(deck, "in");
        assert!(
            stripped.contains("Valve in preamp"),
            "title line must be preserved, got:\n{stripped}"
        );
        assert!(
            !stripped.lines().any(|l| l.trim_start().starts_with("VIN ")),
            "the real VIN element must still be stripped, got:\n{stripped}"
        );
    }
}
