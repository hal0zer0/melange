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
pub mod spice_runner;
pub mod visualizer;

pub use comparison::{batch_compare, compare_signals, ComparisonConfig, ComparisonReport, Signal};
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
    Spice(#[from] SpiceError),

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
        .map_err(ValidationError::Spice)?;

    // Run melange solver on stripped netlist (VIN removed)
    let melange_output = run_melange_solver_from_str(
        &stripped_netlist,
        input_signal,
        sample_rate,
        output_node,
        input_node,
    )?;

    // Apply DC blocking to SPICE output to match melange's internal DC blocker (5 Hz HPF)
    let mut spice_output_blocked = spice_output.to_vec();
    dc_block_signal(&mut spice_output_blocked, spice_data.sample_rate);

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

/// Apply a 5 Hz DC blocking high-pass filter to a signal.
///
/// Matches the internal DC blocker in melange's `CircuitSolver::process_sample()`.
fn dc_block_signal(signal: &mut [f64], sample_rate: f64) {
    let r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / sample_rate;
    let mut x_prev = 0.0f64;
    let mut y_prev = 0.0f64;
    for sample in signal.iter_mut() {
        let x = *sample;
        let y = x - x_prev + r * y_prev;
        x_prev = x;
        y_prev = y;
        *sample = y;
    }
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

    for line in netlist.lines() {
        let trimmed = line.trim();

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
fn run_melange_solver_from_str(
    netlist_str: &str,
    input_signal: &[f64],
    sample_rate: f64,
    output_node_name: &str,
    input_node_name: &str,
) -> Result<Vec<f64>, ValidationError> {
    use melange_solver::codegen::{CodeGenerator, CodegenConfig, routing};
    use std::io::Write;

    let netlist = melange_solver::parser::Netlist::parse(netlist_str).map_err(|e| {
        ValidationError::Solver(format!("Parse error at line {}: {}", e.line, e.message))
    })?;

    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .map_err(|e| ValidationError::Solver(format!("MNA error: {}", e)))?;

    let input_node = mna.node_map.get(input_node_name).copied()
        .ok_or_else(|| ValidationError::Solver(format!(
            "Input node '{}' not found. Available: {:?}",
            input_node_name, mna.node_map.keys().collect::<Vec<_>>()
        )))?.saturating_sub(1);
    let output_node = mna.node_map.get(output_node_name).copied()
        .ok_or_else(|| ValidationError::Solver(format!(
            "Output node '{}' not found. Available: {:?}",
            output_node_name, mna.node_map.keys().collect::<Vec<_>>()
        )))?.saturating_sub(1);

    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }

    // Stamp junction caps
    {
        let device_slots = build_device_slots_from_netlist(&netlist);
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    // Build kernel and route
    let has_inductors = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();
    let mut dk_failed = false;
    let kernel = if has_inductors {
        melange_solver::dk::DkKernel::from_mna_augmented(&mna, sample_rate)
            .map_err(|e| ValidationError::Solver(format!("Augmented DK: {:?}", e)))?
    } else {
        match melange_solver::dk::DkKernel::from_mna(&mna, sample_rate) {
            Ok(k) => k,
            Err(_) => {
                dk_failed = true;
                melange_solver::dk::DkKernel::from_mna_augmented(&mna, sample_rate)
                    .map_err(|e| ValidationError::Solver(format!("DK fallback: {:?}", e)))?
            }
        }
    };

    let decision = routing::auto_route(&kernel, &mna, dk_failed);
    let use_nodal = decision.route == routing::SolverRoute::Nodal;

    if use_nodal {
        let slots = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
            &netlist, Some(&mna),
        ).unwrap_or_default();
        if !slots.is_empty() {
            mna.expand_bjt_internal_nodes(&slots);
        }
    }

    let config = CodegenConfig {
        circuit_name: "validate".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        dc_block: true,
        ..CodegenConfig::default()
    };
    let generator = CodeGenerator::new(config);
    let generated = if use_nodal {
        generator.generate_nodal(&mna, &netlist)
    } else {
        generator.generate(&kernel, &mna, &netlist)
    }.map_err(|e| ValidationError::Solver(format!("Codegen: {}", e)))?;

    // Append stdin/stdout main
    let main_code = "fn main() {\n\
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
    }\n";
    let full_source = format!("{}\n{}", generated.code, main_code);

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
        .arg(&src).arg("-o").arg(&bin)
        .arg("--edition=2024").arg("-O")
        .output()
        .map_err(|e| ValidationError::Solver(format!("rustc: {}", e)))?;
    let _ = std::fs::remove_file(&src);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin);
        return Err(ValidationError::Solver(format!(
            "Compilation failed:\n{}", String::from_utf8_lossy(&compile.stderr)
        )));
    }

    // Run: pipe input via stdin
    let stdin_data: String = input_signal.iter().map(|s| format!("{s:.15e}\n")).collect();
    let mut child = std::process::Command::new(&bin)
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| ValidationError::Solver(format!("Spawn: {}", e)))?;

    if let Some(mut stdin) = child.stdin.take() {
        stdin.write_all(stdin_data.as_bytes())
            .map_err(|e| ValidationError::Solver(format!("Stdin: {}", e)))?;
    }

    let result = child.wait_with_output()
        .map_err(|e| ValidationError::Solver(format!("Wait: {}", e)))?;
    let _ = std::fs::remove_file(&bin);

    if !result.status.success() {
        return Err(ValidationError::Solver(format!(
            "Binary failed:\n{}", String::from_utf8_lossy(&result.stderr)
        )));
    }

    Ok(String::from_utf8_lossy(&result.stdout)
        .lines()
        .filter_map(|l| l.trim().parse().ok())
        .collect())
}

/// Build device list from parsed netlist
#[cfg(any())]
fn build_devices_from_netlist(
    netlist: &melange_solver::parser::Netlist,
    mna: &melange_solver::mna::MnaSystem,
) -> Result<Vec<melange_solver::solver::DeviceEntry>, ValidationError> {
    let mut devices = Vec::new();

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

    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                let model_params = find_diode_model(netlist, &dev_info.name);
                let is = model_params.0.unwrap_or(1e-15);
                let n = model_params.1.unwrap_or(1.0);
                let diode = melange_devices::DiodeShockley::new_room_temp(is, n);
                devices.push(melange_solver::solver::DeviceEntry::new_diode(
                    diode,
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Bjt
            | melange_solver::mna::NonlinearDeviceType::BjtForwardActive => {
                let model_params = find_bjt_model(netlist, &dev_info.name);
                let is = model_params.0.unwrap_or(1e-15);
                let beta_f = model_params.1.unwrap_or(200.0);
                let beta_r = model_params.2.unwrap_or(3.0);
                let nf = find_bjt_nf(netlist, &dev_info.name);
                let polarity = find_bjt_polarity(netlist, &dev_info.name);
                let bjt =
                    melange_devices::BjtEbersMoll::new_room_temp(is, beta_f, beta_r, polarity)
                        .with_nf(nf);
                devices.push(melange_solver::solver::DeviceEntry::new_bjt(
                    bjt,
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Jfet => {
                let model_name = find_device_model_name::<b'J'>(netlist, &dev_info.name);
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
                    .unwrap_or(false);
                let channel = if is_p_channel {
                    melange_devices::JfetChannel::P
                } else {
                    melange_devices::JfetChannel::N
                };
                let default_vp = if is_p_channel { 2.0 } else { -2.0 };
                let vp = find_model_param(&model_name, "VTO").unwrap_or(default_vp);
                // ngspice BETA = IDSS / VP^2, so IDSS = BETA * VP^2
                let idss = if let Some(beta) = find_model_param(&model_name, "BETA") {
                    beta * vp * vp
                } else {
                    find_model_param(&model_name, "IDSS").unwrap_or(2e-3)
                };
                let mut jfet = melange_devices::Jfet::new(channel, vp, idss);
                jfet.lambda = find_model_param(&model_name, "LAMBDA").unwrap_or(0.001);
                devices.push(melange_solver::solver::DeviceEntry::new_jfet(
                    jfet,
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Mosfet => {
                let model_name = find_device_model_name::<b'M'>(netlist, &dev_info.name);
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PM"))
                    .unwrap_or(false);
                let channel = if is_p_channel {
                    melange_devices::MosfetChannelType::P
                } else {
                    melange_devices::MosfetChannelType::N
                };
                let default_vt = if is_p_channel { -2.0 } else { 2.0 };
                let vt = find_model_param(&model_name, "VTO").unwrap_or(default_vt);
                let kp = find_model_param(&model_name, "KP").unwrap_or(0.1);
                let lambda = find_model_param(&model_name, "LAMBDA").unwrap_or(0.01);
                let mosfet = melange_devices::Mosfet::new(channel, vt, kp, lambda);
                devices.push(melange_solver::solver::DeviceEntry::new_mosfet(
                    mosfet,
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Tube => {
                let model_name = find_device_model_name::<b'T'>(netlist, &dev_info.name);
                let mu = find_model_param(&model_name, "MU").unwrap_or(100.0);
                let ex = find_model_param(&model_name, "EX").unwrap_or(1.4);
                let kg1 = find_model_param(&model_name, "KG1").unwrap_or(1060.0);
                let kp = find_model_param(&model_name, "KP").unwrap_or(600.0);
                let kvb = find_model_param(&model_name, "KVB").unwrap_or(300.0);
                let ig_max = find_model_param(&model_name, "IG_MAX").unwrap_or(2e-3);
                let vgk_onset = find_model_param(&model_name, "VGK_ONSET").unwrap_or(0.5);
                let lambda = find_model_param(&model_name, "LAMBDA").unwrap_or(0.0);
                let tube = melange_devices::KorenTriode::with_all_params(
                    mu, ex, kg1, kp, kvb, ig_max, vgk_onset, lambda,
                );
                devices.push(melange_solver::solver::DeviceEntry::new_tube(
                    tube,
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Vca => {
                let model_name = find_device_model_name::<b'Y'>(netlist, &dev_info.name);
                let vscale = find_model_param(&model_name, "VSCALE").unwrap_or(0.05298);
                let g0 = find_model_param(&model_name, "G0").unwrap_or(1.0);
                let vca = melange_devices::Vca::new(vscale, g0);
                devices.push(melange_solver::solver::DeviceEntry::new_vca(
                    vca,
                    dev_info.start_idx,
                ));
            }
        }
    }

    Ok(devices)
}

/// Find device model name by device name, using element prefix to match element type.
fn find_device_model_name<const PREFIX: u8>(
    netlist: &melange_solver::parser::Netlist,
    device_name: &str,
) -> String {
    netlist
        .elements
        .iter()
        .find_map(|e| match (PREFIX, e) {
            (b'J', melange_solver::parser::Element::Jfet { name, model, .. })
                if name.eq_ignore_ascii_case(device_name) =>
            {
                Some(model.clone())
            }
            (b'M', melange_solver::parser::Element::Mosfet { name, model, .. })
                if name.eq_ignore_ascii_case(device_name) =>
            {
                Some(model.clone())
            }
            (b'T', melange_solver::parser::Element::Triode { name, model, .. })
                if name.eq_ignore_ascii_case(device_name) =>
            {
                Some(model.clone())
            }
            _ => None,
        })
        .unwrap_or_default()
}

/// Find diode model parameters from netlist
/// Returns (IS, N) if found
fn find_diode_model(
    netlist: &melange_solver::parser::Netlist,
    device_name: &str,
) -> (Option<f64>, Option<f64>) {
    // First, find the device to get its model name
    let model_name = netlist.elements.iter().find_map(|e| {
        if let melange_solver::parser::Element::Diode { name, model, .. } = e {
            if name == device_name {
                return Some(model.clone());
            }
        }
        None
    });

    let model_name = match model_name {
        Some(name) => name,
        None => return (None, None),
    };

    // Find the model definition
    let mut is = None;
    let mut n = None;

    for model in &netlist.models {
        if model.name == model_name && model.model_type.to_uppercase().starts_with('D') {
            for (key, value) in &model.params {
                match key.to_uppercase().as_str() {
                    "IS" => is = Some(*value),
                    "N" => n = Some(*value),
                    _ => {}
                }
            }
            break;
        }
    }

    (is, n)
}

/// Find BJT model parameters from netlist
/// Returns (IS, BF, BR) if found
fn find_bjt_model(
    netlist: &melange_solver::parser::Netlist,
    device_name: &str,
) -> (Option<f64>, Option<f64>, Option<f64>) {
    // First, find the device to get its model name
    let model_name = netlist.elements.iter().find_map(|e| {
        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
            if name == device_name {
                return Some(model.clone());
            }
        }
        None
    });

    let model_name = match model_name {
        Some(name) => name,
        None => return (None, None, None),
    };

    // Find the model definition
    let mut is = None;
    let mut bf = None;
    let mut br = None;

    for model in &netlist.models {
        if model.name == model_name {
            let model_type_upper = model.model_type.to_uppercase();
            if model_type_upper.starts_with("NPN") || model_type_upper.starts_with("PNP") {
                for (key, value) in &model.params {
                    match key.to_uppercase().as_str() {
                        "IS" => is = Some(*value),
                        "BF" => bf = Some(*value),
                        "BR" => br = Some(*value),
                        _ => {}
                    }
                }
                break;
            }
        }
    }

    (is, bf, br)
}

/// Find BJT polarity from netlist model definition
fn find_bjt_polarity(
    netlist: &melange_solver::parser::Netlist,
    device_name: &str,
) -> melange_devices::BjtPolarity {
    // First, find the device to get its model name
    let model_name = netlist.elements.iter().find_map(|e| {
        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
            if name == device_name {
                return Some(model.clone());
            }
        }
        None
    });

    let model_name = match model_name {
        Some(name) => name,
        None => return melange_devices::BjtPolarity::Npn,
    };

    // Find the model definition and check type
    for model in &netlist.models {
        if model.name == model_name {
            let model_type_upper = model.model_type.to_uppercase();
            if model_type_upper.starts_with("PNP") {
                return melange_devices::BjtPolarity::Pnp;
            } else {
                // Default to NPN
                return melange_devices::BjtPolarity::Npn;
            }
        }
    }

    melange_devices::BjtPolarity::Npn
}

/// Look up BJT forward emission coefficient NF from model params. Returns 1.0 if not found.
fn find_bjt_nf(netlist: &melange_solver::parser::Netlist, device_name: &str) -> f64 {
    let model_name = netlist.elements.iter().find_map(|e| {
        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
            if name == device_name {
                return Some(model.clone());
            }
        }
        None
    });
    let model_name = match model_name {
        Some(name) => name,
        None => return 1.0,
    };
    for model in &netlist.models {
        if model.name == model_name {
            for (key, value) in &model.params {
                if key.eq_ignore_ascii_case("NF") {
                    return *value;
                }
            }
        }
    }
    1.0
}

/// Build device slots for the DC OP solver from a parsed netlist
fn build_device_slots_from_netlist(
    netlist: &melange_solver::parser::Netlist,
) -> Vec<melange_solver::device_types::DeviceSlot> {
    use melange_solver::device_types::{
        BjtParams, DeviceParams, DeviceSlot, DeviceType, DiodeParams,
    };

    let vt = melange_primitives::VT_ROOM;
    let mut slots = Vec::new();
    let mut dim_offset = 0;

    for elem in &netlist.elements {
        match elem {
            melange_solver::parser::Element::Diode { name, .. } => {
                let model_params = find_diode_model(netlist, name);
                let is = model_params.0.unwrap_or(1e-15);
                let n = model_params.1.unwrap_or(1.0);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Diode,
                    start_idx: dim_offset,
                    dimension: 1,
                    params: DeviceParams::Diode(DiodeParams {
                        is,
                        n_vt: n * vt,
                        cjo: 0.0,
                        rs: 0.0,
                        bv: f64::INFINITY,
                        ibv: 1e-10,
                    }),
                    has_internal_mna_nodes: false,
                });
                dim_offset += 1;
            }
            melange_solver::parser::Element::Bjt { model, .. } => {
                let bjt_params = find_bjt_model_by_name(netlist, model);
                let is = bjt_params.0.unwrap_or(1e-14);
                let beta_f = bjt_params.1.unwrap_or(200.0);
                let beta_r = bjt_params.2.unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model))
                    .map(|m| m.model_type.to_uppercase().starts_with("PNP"))
                    .unwrap_or(false);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Bjt,
                    start_idx: dim_offset,
                    dimension: 2,
                    params: DeviceParams::Bjt(BjtParams {
                        is,
                        vt,
                        beta_f,
                        beta_r,
                        is_pnp,
                        vaf: f64::INFINITY,
                        var: f64::INFINITY,
                        ikf: f64::INFINITY,
                        ikr: f64::INFINITY,
                        cje: 0.0,
                        cjc: 0.0,
                        nf: 1.0,
                        nr: 1.0,
                        ise: 0.0,
                        ne: 1.5,
                        isc: 0.0,
                        nc: 2.0,
                        rb: 0.0,
                        rc: 0.0,
                        re: 0.0,
                        rth: f64::INFINITY,
                        cth: 1e-3,
                        xti: 3.0,
                        eg: 1.11,
                        tamb: 300.15,
                    }),
                    has_internal_mna_nodes: false,
                });
                dim_offset += 2;
            }
            _ => {}
        }
    }

    slots
}

/// Find BJT model parameters by model name (not device name).
/// Returns (IS, BF, BR) if found.
fn find_bjt_model_by_name(
    netlist: &melange_solver::parser::Netlist,
    model_name: &str,
) -> (Option<f64>, Option<f64>, Option<f64>) {
    let mut is = None;
    let mut bf = None;
    let mut br = None;

    for model in &netlist.models {
        if model.name.eq_ignore_ascii_case(model_name) {
            for (key, value) in &model.params {
                match key.to_uppercase().as_str() {
                    "IS" => is = Some(*value),
                    "BF" => bf = Some(*value),
                    "BR" => br = Some(*value),
                    _ => {}
                }
            }
            break;
        }
    }

    (is, bf, br)
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
}
