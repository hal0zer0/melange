//! SPICE-to-Rust validation pipeline.
//!
//! Layer 4 of the melange stack. Compares melange solver output against ngspice
//! reference simulations with multiple error metrics (correlation, RMS error,
//! peak error, spectral comparison).
//!
//! # Modules
//!
//! - [`spice_runner`] — invoke ngspice, parse output (raw/CSV), manage temp files
//! - [`alignment`] — best-fit constant-delay alignment, applied before every comparison
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

pub mod alignment;
mod behavioral_translate;
pub mod comparison;
pub mod deck_guard;
pub(crate) mod opamp_translate;
pub(crate) mod pentode_translate;
mod sat_inductor_translate;
pub mod spice_runner;
pub(crate) mod tube_translate;
pub mod visualizer;

pub use alignment::{
    align_reference, apply_fractional_delay, dominant_frequency, fit_constant_delay,
    AlignmentRequest, DelayFit,
};
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
    /// nothing about melange's internal rate — and it is NOT filtered. The
    /// comparison is against the circuit: an unfiltered reference aligned to
    /// the melange output by one best-fit constant delay, the same alignment
    /// every mode gets (see [`alignment`]). The half-bands' frequency-dependent
    /// phase therefore stays inside the number, where it belongs; no tolerance
    /// widens for it.
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
            bjt_fa_mode: melange_solver::codegen::BjtFaMode::Off,
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

/// Refuse a port name the circuit does not have, listing the circuit's
/// nodes in index order (the message melange's own solve would give). A deck
/// melange cannot parse is left to the solve, which reports why.
fn check_ports_exist(netlist: &str, ports: &[(&str, &str)]) -> Result<(), ValidationError> {
    let Ok(mut parsed) = melange_solver::parser::Netlist::parse(netlist) else {
        return Ok(());
    };
    if parsed.expand_subcircuits().is_err() {
        return Ok(());
    }
    let Ok(mna) = melange_solver::mna::MnaSystem::from_netlist(&parsed) else {
        return Ok(());
    };
    for (role, node) in ports {
        if !mna.node_map.contains_key(*node) {
            let names: Vec<&str> = mna.node_names_in_index_order();
            return Err(ValidationError::Solver(format!(
                "{role} node '{node}' not found in circuit. Available: {names:?}. \
                 Name it with {}.",
                if *role == "Output" {
                    "-n/--output-node"
                } else {
                    "-i/--input-node"
                }
            )));
        }
    }
    Ok(())
}

/// The base name of the report files `validate` writes into `output_dir`:
/// the circuit's final path component and the verdict. The circuit name is
/// the deck's full path for a local file (or a URL), and joining an absolute
/// path onto `output_dir` replaces it: every report landed next to its deck,
/// in the user's circuit tree, whatever directory was asked for.
fn report_base_name(circuit_name: &str, passed: bool) -> String {
    let name = std::path::Path::new(circuit_name)
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| circuit_name.replace(['/', '\\'], "_"));
    format!("{}_{}", name, if passed { "passed" } else { "failed" })
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

    // A missing input or output node is refused here, before ngspice runs:
    // otherwise ngspice fails first ("can't parse 'out'", "no data saved"),
    // which reads as a simulator problem when it is a node name.
    check_ports_exist(
        &stripped_netlist,
        &[("Output", output_node), ("Input", input_node)],
    )?;

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

    // Run melange solver on stripped netlist (VIN removed). First, because
    // the reference must simulate the circuit melange built: a
    // capacitor-free nonlinear deck gets 10 pF parasitic caps across its
    // junctions, and the reference gets the same ones.
    let (melange_output, parasitic_caps) = run_melange_build(
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
    let reference_deck = with_parasitic_caps(&netlist_str, &parasitic_caps)?;

    let spice_data = spice_runner::run_transient_with_thevenin_pwl(
        &reference_deck,
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

    // Apply DC blocking to SPICE output to match melange's internal DC blocker (5 Hz HPF)
    let mut spice_output_blocked = spice_output.to_vec();
    dc_block_signal(&mut spice_output_blocked, spice_data.sample_rate);

    // Align the reference to the melange output by ONE best-fit constant
    // delay, in EVERY mode including 1x, and compare against that unfiltered
    // reference — i.e. against the circuit.
    //
    // Until 2026-09-23 an oversampled run instead pushed the reference through
    // the same half-band round trip with the circuit replaced by an identity.
    // The flag, the round trip's magnitude-flatness and the twin-drift guard
    // were all sound; the COMPARISON was not. On the single-tone stimulus
    // validate drives, an allpass is a pure time shift, so in the shipped
    // build harmonic `k` carries `tau_up(f0) + tau_down(k*f0)` (its harmonics
    // are generated after the up leg and never pass through it), while in a
    // round-tripped reference it carries `tau_up(k*f0) + tau_down(k*f0)`. The
    // down leg cancelled exactly and the up leg was billed at harmonic
    // frequencies it never saw: the numbers were real, the attribution was an
    // artifact. See `alignment` for the estimator and its constraints (delay
    // only, never gain; seeded at the analytic round-trip delay; bounded to
    // half a stimulus period).
    //
    // Order: after the DC blocker, so the blocker still sees the reference's
    // own first sample as its seed (the startup-transient fix documented on
    // `dc_block_signal`).
    //
    // `alignment::align_reference` is the ONE comparison method: the CI SPICE
    // gate in `tests/spice_validation.rs` calls exactly this, so the CLI's
    // numbers and CI's numbers are the same measurement.
    let (spice_output_aligned, delay_fit) =
        alignment::align_reference(alignment::AlignmentRequest {
            reference: &spice_output_blocked,
            actual: &melange_output,
            input_signal,
            reference_rate: spice_data.sample_rate,
            actual_rate: sample_rate,
            oversampling: options.oversampling,
            settle_time_s: config.settle_time_s,
        });

    // Create signal objects for comparison
    let spice_signal = Signal::new(
        spice_output_aligned,
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
    // The reference is the deck plus melange's parasitic caps when it had no
    // capacitance: say so, since a reader running ngspice on the deck alone
    // would get a different answer.
    report.parasitic_note = (!parasitic_caps.is_empty()).then(|| {
        format!(
            "the deck has no capacitors, so both engines carry the {} x 10 pF parasitic \
             junction capacitors melange adds ({})",
            parasitic_caps.len(),
            parasitic_caps
                .iter()
                .map(|c| format!("{} {}-{}", c.device, c.node_a, c.node_b))
                .collect::<Vec<_>>()
                .join(", ")
        )
    });
    // Say on the report which build was validated. A correlation number for an
    // oversampled build is not comparable to a 1x one, and a reader who is not
    // told will assume it is.
    report.oversampling_note = (options.oversampling > 1).then(|| {
        format!(
            "oversampling {}x (internal rate {:.0} Hz); the emitted code interpolates and \
             decimates through polyphase IIR half-band allpass chains, whose \
             frequency-dependent phase stays in the comparison",
            options.oversampling,
            sample_rate * options.oversampling as f64,
        )
    });
    // The alignment is part of how the number was obtained, in every mode, so
    // it is reported next to the number and names the analytic delay it was
    // seeded at — a fitted delay far from analytic is itself a finding.
    report.alignment_note = Some(delay_fit.note(spice_data.sample_rate));

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

        let base_name = report_base_name(&report.circuit_name, report.passed);

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

/// Pass a signal through the SAME half-band round trip an oversampled melange
/// build applies, with the circuit replaced by an identity.
///
/// # This is NOT the comparison method any more
///
/// Until 2026-09-23 `validate_circuit_with_options` ran the ngspice reference
/// through this so both sides carried the same filters. That compensation is
/// retired. On the single-tone stimulus validate drives, an allpass is a pure
/// time shift and a time-invariant circuit maps a delayed input to an
/// identically delayed output, so:
///
/// - shipped output: harmonic `k` carries `tau_up(f0) + tau_down(k*f0)` — the
///   harmonics are generated by the nonlinearity AFTER the up leg, so they
///   never pass through it;
/// - round-tripped reference: harmonic `k` carries
///   `tau_up(k*f0) + tau_down(k*f0)` — ngspice's harmonics already exist, then
///   go through BOTH legs.
///
/// The down leg cancels exactly and the up leg is billed at harmonic
/// frequencies it never saw, so the residual such a comparison reports is
/// `tau_up(f0) - tau_up(k*f0)`: a real number, attributed to the wrong place.
/// The comparison now runs against an UNFILTERED reference aligned by one
/// best-fit constant delay — see [`alignment`], and `docs/aidocs/OVERSAMPLING.md`.
///
/// # What it is still for
///
/// The twin-drift guard. The chain here is
/// `melange_primitives::oversampling::Oversampler`, the same implementation the
/// codegen emitter mirrors, and `tests/oversampling_reference.rs` pins the two
/// together by running this against the GENERATED, COMPILED oversampled code on
/// a pure-gain circuit. That test is the reason this function stays public: if
/// the coefficient tables, the branch split, the clocking or the stage
/// assignment ever drift apart, it fails. It is also how
/// [`oversampling_round_trip_group_delay_samples`] measures the analytic delay
/// the alignment search is seeded at.
///
/// `factor == 1` is a no-op. Panics on an unsupported factor — the caller
/// validates it first.
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

/// Delay of the oversampling round trip at `freq_hz`, in host samples.
///
/// PHASE delay, `-phase(w) / w`, despite the historical name — which is the
/// right quantity here: on a single tone the phase delay IS the time shift the
/// chain applies, and that is what the alignment search is seeded with.
///
/// Measured, not asserted: an impulse is pushed through the same chain
/// [`apply_oversampling_round_trip`] uses and the phase of its response at
/// `freq_hz` is read off. Printed next to the FITTED delay on the result line,
/// so a fit that disagrees with the filter design is visible rather than
/// silently accepted.
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
    run_melange_build(
        netlist_str,
        input_signal,
        sample_rate,
        output_node_name,
        input_node_name,
        bjt_fa_mode,
        tube_grid_fa,
        backward_euler,
        force_trap,
        oversampling,
        main_code,
    )
    .map(|(output, _)| output)
}

/// [`run_melange_solver_from_str`], also returning the parasitic caps the
/// build auto-inserted (empty unless the deck is capacitor-free and
/// nonlinear), which the reference needs to simulate the same circuit.
#[allow(clippy::too_many_arguments)]
fn run_melange_build(
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
) -> Result<(Vec<f64>, Vec<melange_solver::mna::ParasiticCap>), ValidationError> {
    use melange_solver::codegen::CodegenConfig;

    if !matches!(oversampling, 1 | 2 | 4) {
        return Err(ValidationError::InvalidInput(format!(
            "oversampling must be 1, 2, or 4, got {oversampling}"
        )));
    }

    // The melange side of every comparison is the build `melange compile`
    // ships: the one build function every verb calls. What validate chooses
    // differently is visible in these options:
    // * unit variation OFF, unconditionally. `.tolerance` jitters fixed R/C/L
    //   values in the parser and `.mismatch` jitters device model parameters,
    //   both on melange's side only; the reference deck handed to ngspice
    //   carries the values as written, so a jittered melange side would put a
    //   correlation between two DIFFERENT circuits on the result line (measured
    //   on `examples/passive-eq1a.cir`, `.seed 4142` / `.mismatch T MU=0.09
    //   KG1=0.20`: 16 emitted device constants differ, `DEVICE_0_MU` by -7.6 %).
    //   The caller names the disabled directives on the result line via
    //   `deck_guard::unit_variation_note`.
    // * the 1-ohm Thevenin input the harness drives the reference with;
    // * an output clamp raised to three times the largest DC operating-point
    //   node voltage (never below the 10 V default): a high-rail circuit (a
    //   250 V tube B+) swings its output tens of volts under large-signal
    //   drive, and a fixed 10 V ceiling would square it into a divergence that
    //   is a harness gap, not a solver bug (triode_cc overdrive, 2026-08);
    // * the base rate unless an oversampling factor is passed: at 2x/4x the
    //   emitted code upsamples, solves at the internal rate and decimates, and
    //   the reference goes through the same half-band round trip in
    //   `validate_circuit_with_options`.
    let build_opts = melange_solver::build::BuildOptions {
        sample_rate,
        circuit_name: "validate".to_string(),
        input_nodes: vec![input_node_name.to_string()],
        output_nodes: vec![output_node_name.to_string()],
        max_iter: None,
        tolerance: CodegenConfig::default().tolerance,
        output_scale: 1.0,
        output_clamp: CodegenConfig::default().output_clamp_v,
        input_resistance: Some(1.0),
        oversampling: Some(oversampling),
        dc_block: true,
        solver: "auto".to_string(),
        backward_euler,
        force_trap,
        tube_grid_fa: tube_grid_fa.to_string(),
        subsample_fire: CodegenConfig::default().subsample_fire,
        subsample_lit_factor: CodegenConfig::default().subsample_lit_factor,
        bjt_fa_mode,
        opamp_rail_mode: CodegenConfig::default().opamp_rail_mode,
        nodal_sub_path_override: CodegenConfig::default().nodal_sub_path_override,
        allow_static_glow_on_full_lu: false,
        noise_mode: CodegenConfig::default().noise_mode,
        noise_seed: CodegenConfig::default().noise_master_seed,
        emit_dc_op_recompute: false,
        plugin_format: false,
        pot_overrides: None,
        resolve_taps: false,
        inject_runtime: false,
        disable_unit_variation: true,
        // A build that starts from an unconverged operating point is refused:
        // comparing it against ngspice would measure the wrong start.
        allow_unconverged_dc_op: false,
        dc_op_max_iterations: None,
        output_clamp_auto: true,
    };
    let built =
        melange_solver::build::build(netlist_str, &build_opts, &|m| log::info!("{m}"), &|m| {
            log::warn!("{m}")
        })
        .map_err(|e| ValidationError::Solver(e.to_string()))?;
    let generated = built.generated;

    let output = run_generated_solver(&generated.code, input_signal, main_code)?;
    Ok((output, generated.meta.parasitic_caps))
}

/// `netlist` with melange's auto-inserted parasitic caps added as SPICE
/// capacitors, so a reference simulator runs the circuit melange built.
/// Inserted before `.end` (appended when there is none); unchanged when
/// `caps` is empty. Node names are the build's own, which for a subcircuit's
/// internal node is the `X1.node` form ngspice also resolves.
///
/// Refuses a cap whose node has no name: it could not be placed, and the
/// reference would silently be a different circuit.
pub fn with_parasitic_caps(
    netlist: &str,
    caps: &[melange_solver::mna::ParasiticCap],
) -> Result<String, ValidationError> {
    if caps.is_empty() {
        return Ok(netlist.to_string());
    }
    let mut lines = String::new();
    for (k, c) in caps.iter().enumerate() {
        if c.node_a.is_empty() || c.node_b.is_empty() {
            return Err(ValidationError::InvalidInput(format!(
                "melange added a parasitic capacitor across {} at a node with no netlist name; \
                 the reference cannot carry it. Put the circuit's capacitances in the netlist.",
                c.device
            )));
        }
        lines.push_str(&format!(
            "C_melange_parasitic_{} {} {} {:e}\n",
            k + 1,
            c.node_a,
            c.node_b,
            melange_solver::mna::PARASITIC_CAP
        ));
    }
    let mut out = String::with_capacity(netlist.len() + lines.len());
    let mut placed = false;
    for line in netlist.lines() {
        if !placed && line.trim().eq_ignore_ascii_case(".end") {
            out.push_str(&lines);
            placed = true;
        }
        out.push_str(line);
        out.push('\n');
    }
    if !placed {
        out.push_str(&lines);
    }
    Ok(out)
}

/// Compile generated circuit code with the validation driver, run it on
/// `input_signal`, and return the output, or refuse the render.
///
/// The second half of [`run_melange_solver_from_str`], public so a test can
/// run a deliberately broken build of real generated code (e.g. `MAX_ITER`
/// forced to 1) through the exact driver and refusals validate uses. Each
/// refusal (unsolved samples, clamped input, clamped output) needs a witness
/// that fails if it goes dead: they were once dead together.
pub fn run_generated_solver(
    code: &str,
    input_signal: &[f64],
    main_code: Option<&str>,
) -> Result<Vec<f64>, ValidationError> {
    use std::io::Write;

    // Append the driver main — caller-supplied, else stdin/stdout.
    // Diagnostics go to stderr as `DIAG:key=value` lines (same protocol as
    // the CLI's simulate driver) and are echoed below.
    //
    // Build-conditional counters are added by presence, so a build that does
    // not declare one still compiles:
    // - the unsolved-sample count (`diag_unsolved_sample_count`: the nodal
    //   death-spiral hold plus every committed unconverged iterate);
    // - the input sanitisation (clamp to INPUT_LIMIT_V, NaN -> 0) and the
    //   output clamp.
    // The lines are part of the template by construction. They used to be
    // spliced in with `str::replace` on an indented copy of the region-exit
    // line, which never matched (a `\` line continuation strips the next
    // line's indentation), so validate never saw these counters.
    // Matched on the field declaration: generated comments name the counters
    // on builds that do not declare them.
    // Every unsolved sample: the always-present unified counter, or on a
    // build that predates it the sum of the mechanism counters it declares.
    let unsolved_fields: Vec<&str> = if code.contains("pub diag_unsolved_sample_count: ") {
        vec!["diag_unsolved_sample_count"]
    } else {
        ["diag_nr_hold_count", "diag_nr_unconverged_commit_count"]
            .into_iter()
            .filter(|f| code.contains(&format!("pub {f}: ")))
            .collect()
    };
    let mut extra_diag: String = if unsolved_fields.is_empty() {
        String::new()
    } else {
        let sum = unsolved_fields
            .iter()
            .map(|f| format!("state.{f}"))
            .collect::<Vec<_>>()
            .join(" + ");
        format!("    eprintln!(\"DIAG:nr_hold_count={{}}\", {sum});\n")
    };
    for f in [
        "diag_input_clamp_count",
        "diag_input_nan_count",
        "diag_clamp_count",
    ] {
        if code.contains(&format!("pub {f}: ")) {
            let key = f.strip_prefix("diag_").unwrap_or(f);
            extra_diag.push_str(&format!("    eprintln!(\"DIAG:{key}={{}}\", state.{f});\n"));
        }
    }
    let default_main = format!(
        "fn main() {{\n\
         \x20   let mut state = CircuitState::default();\n\
         \x20   let stdin = std::io::stdin();\n\
         \x20   let mut line = String::new();\n\
         \x20   loop {{\n\
         \x20       line.clear();\n\
         \x20       if stdin.read_line(&mut line).unwrap() == 0 {{ break; }}\n\
         \x20       if let Ok(input) = line.trim().parse::<f64>() {{\n\
         \x20           let out = process_sample(input, &mut state);\n\
         \x20           println!(\"{{:.15e}}\", out[0]);\n\
         \x20       }}\n\
         \x20   }}\n\
         \x20   eprintln!(\"DIAG:nr_max_iter_count={{}}\", state.diag_nr_max_iter_count);\n\
         \x20   eprintln!(\"DIAG:region_exit_count={{}}\", state.diag_region_exit_count);\n\
         {extra_diag}}}\n"
    );
    let full_source = format!("{}\n{}", code, main_code.unwrap_or(default_main.as_str()));
    // Diagnostic: MELANGE_DUMP_SOURCE=<dir> writes the generated circuit code
    // to <dir>/validate.rs, so two builds of one deck can be diffed.
    if let Some(dir) = std::env::var_os("MELANGE_DUMP_SOURCE") {
        let dir = std::path::PathBuf::from(dir);
        let _ = std::fs::create_dir_all(&dir);
        let _ = std::fs::write(dir.join("validate.rs"), code);
    }

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
    let (mut held, mut clamped, mut nan, mut out_clamped) = (0u64, 0u64, 0u64, 0u64);
    for line in String::from_utf8_lossy(&result.stderr).lines() {
        if let Some(diag) = line.strip_prefix("DIAG:") {
            eprintln!("  melange {}", diag.replacen('=', ": ", 1));
            if let Some(v) = diag.strip_prefix("nr_hold_count=") {
                held = v.trim().parse().unwrap_or(0);
            }
            if let Some(v) = diag.strip_prefix("input_clamp_count=") {
                clamped = v.trim().parse().unwrap_or(0);
            }
            if let Some(v) = diag.strip_prefix("input_nan_count=") {
                nan = v.trim().parse().unwrap_or(0);
            }
            if let Some(v) = diag.strip_prefix("clamp_count=") {
                out_clamped = v.trim().parse().unwrap_or(0);
            }
        }
    }

    // ngspice was driven with the requested input; a melange render whose
    // input was clamped to INPUT_LIMIT_V (or had NaN replaced by 0) answers a
    // different question, and a correlation between the two measures nothing.
    if clamped > 0 || nan > 0 {
        return Err(ValidationError::Solver(format!(
            "melange was not driven with the requested input: {clamped} sample(s) exceeded \
             the generated code's input limit (INPUT_LIMIT_V = 100 V) and were clamped, \
             {nan} were NaN/Inf and replaced by 0. ngspice saw the unclamped input, so the \
             two renders are not comparable."
        )));
    }

    // The generated code clamps its output (after DC blocking) to the output
    // limit. ngspice has no such clamp, so on clamped samples the comparison
    // measures the clamp, not the circuit (design review).
    if out_clamped > 0 {
        return Err(ValidationError::Solver(format!(
            "{out_clamped} output sample(s) exceeded the generated code's output clamp and \
             were clipped to it. ngspice has no output clamp, so the two renders are not \
             comparable there. Lower the input level."
        )));
    }

    // A render containing samples that were never solved cannot validate
    // anything. Correlating it against ngspice produces a number, and the
    // number is meaningless: on those samples melange emitted the PREVIOUS
    // state, not an answer to the circuit. Fail here rather than let a
    // confident correlation be computed from a frozen render (design review).
    if held > 0 {
        return Err(ValidationError::Solver(format!(
            "{held} sample(s) were never solved: every Newton path failed and the previous \
             state was committed as the output. A correlation against this render does not \
             measure agreement with ngspice — it measures agreement with a held value. \
             Fix the convergence before trusting any number from this deck."
        )));
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

    /// A deck without the named port is refused before ngspice runs, with
    /// the circuit's nodes listed and the flag to use.
    #[test]
    fn a_missing_port_is_refused_by_name() {
        let deck = "bank\nR1 p3 p12 10k\nC1 p12 0 10n\n";
        assert!(check_ports_exist(deck, &[("Output", "p12"), ("Input", "p3")]).is_ok());
        let err = check_ports_exist(deck, &[("Output", "out"), ("Input", "p3")])
            .expect_err("no node out")
            .to_string();
        assert!(err.contains("Output node 'out' not found"), "{err}");
        assert!(
            err.contains("\"p12\"") && err.contains("-n/--output-node"),
            "{err}"
        );
    }

    /// A local deck's circuit name is its full path; the report files must
    /// land in `output_dir`, not next to the deck.
    #[test]
    fn report_files_are_named_inside_the_output_dir() {
        let dir = std::path::Path::new("/tmp/reports");
        for (name, want) in [
            ("/home/u/circuits/pedals/fuzz.cir", "fuzz.cir_failed"),
            ("https://example.org/decks/fuzz.cir", "fuzz.cir_failed"),
            ("builtin:fuzz", "builtin:fuzz_failed"),
        ] {
            let base = report_base_name(name, false);
            assert_eq!(base, want, "{name}");
            assert_eq!(
                dir.join(&base).parent(),
                Some(dir),
                "{name} escapes the output dir"
            );
        }
    }

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
                melange_solver::codegen::BjtFaMode::Off,
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
