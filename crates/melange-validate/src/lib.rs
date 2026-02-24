// melange-validate: SPICE-to-Rust validation pipeline
//
// Layer 4 of the melange stack. Bridges ngspice simulation and Rust DSP output:
// - ngspice runner (invoke simulation, parse raw/CSV output)
// - Automated comparison: DC bias, AC sweep, transient, THD
// - Tolerance-band assertions for integration into unit tests
// - "Bisect the signal chain" debugging helpers
// - Measurement routines (gain, frequency sweep, harmonics, spectral centroid)

//! # Melange Validate
//!
//! SPICE-to-Rust validation pipeline for the Melange circuit modeling toolkit.
//!
//! This crate provides infrastructure to compare melange solver output against
//! ngspice reference simulations with multiple error metrics.
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

pub use comparison::{
    batch_compare, compare_signals, ComparisonConfig, ComparisonReport, Signal,
};
pub use spice_runner::{run_transient, run_transient_with_pwl, SpiceData, SpiceError};
pub use visualizer::{generate_csv, generate_html_report, generate_json_report};

/// Errors that can occur during validation
#[derive(Debug, Error)]
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

    // Calculate timing parameters
    let duration = input_signal.len() as f64 / sample_rate;
    let tstep = options.tstep.unwrap_or(1.0 / sample_rate);

    // Build PWL data from input signal
    let pwl_data: Vec<(f64, f64)> = input_signal
        .iter()
        .enumerate()
        .map(|(i, &v)| (i as f64 / sample_rate, v))
        .collect();

    // Run SPICE simulation with PWL input
    let mut nodes_to_capture = vec![output_node.to_string()];
    nodes_to_capture.extend(options.additional_nodes.clone());

    let spice_data = run_transient_with_pwl(
        netlist_path,
        tstep,
        duration,
        "in", // Default PWL source name
        &pwl_data,
        &nodes_to_capture,
    )?;

    // Extract the output signal from SPICE results
    let spice_output = spice_data
        .get_node_voltage(output_node)
        .map_err(|e| ValidationError::Spice(e))?;

    // Run melange solver
    let melange_output = run_melange_solver(netlist_path, input_signal, sample_rate)?;

    // Create signal objects for comparison
    let spice_signal = Signal::new(
        spice_output.to_vec(),
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
    report.circuit_name = options
        .circuit_name
        .clone()
        .unwrap_or_else(|| {
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
            .unwrap_or_else(|| std::env::temp_dir());

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

/// Run melange solver on a netlist with the given input signal
///
/// This function integrates the full melange solver pipeline:
/// 1. Parse SPICE netlist
/// 2. Build MNA system
/// 3. Create DK kernel
/// 4. Build device list from netlist
/// 5. Run circuit solver with input signal
fn run_melange_solver(
    netlist_path: &Path,
    input_signal: &[f64],
    sample_rate: f64,
) -> Result<Vec<f64>, ValidationError> {
    // 1. Parse netlist
    let netlist_str = std::fs::read_to_string(netlist_path)
        .map_err(|e| ValidationError::Io(format!("Failed to read netlist: {}", e)))?;

    let netlist = melange_solver::parser::Netlist::parse(&netlist_str)
        .map_err(|e| ValidationError::Solver(format!("Parse error at line {}: {}", e.line, e.message)))?;

    // 2. Build MNA system
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .map_err(|e| ValidationError::Solver(format!("MNA error: {}", e)))?;

    // 3. Create DK kernel
    let kernel = melange_solver::dk::DkKernel::from_mna(&mna, sample_rate)
        .map_err(|e| ValidationError::Solver(format!("DK kernel error: {}", e)))?;

    // Get input/output node indices (default to "in" and "out")
    let input_node = mna.node_map.get("in").copied().unwrap_or(1).saturating_sub(1);
    let output_node = mna.node_map.get("out").copied().unwrap_or(2).saturating_sub(1);

    // 4. Build device list from netlist
    let devices = build_devices_from_netlist(&netlist, &mna)?;

    // 5. Create solver
    let mut solver = melange_solver::solver::CircuitSolver::new(kernel, devices, input_node, output_node);

    // 6. Run simulation
    let mut output = Vec::with_capacity(input_signal.len());
    for &sample in input_signal {
        let out = solver.process_sample(sample);
        output.push(out);
    }

    Ok(output)
}

/// Build device list from parsed netlist
fn build_devices_from_netlist(
    netlist: &melange_solver::parser::Netlist,
    mna: &melange_solver::mna::MnaSystem,
) -> Result<Vec<melange_solver::solver::DeviceEntry>, ValidationError> {
    let mut devices = Vec::new();

    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                // Find model parameters from netlist
                let model_params = find_diode_model(netlist, &dev_info.name);
                let is = model_params.0.unwrap_or(1e-15);
                let n = model_params.1.unwrap_or(1.0);

                let diode = melange_devices::DiodeShockley::new_room_temp(is, n);
                devices.push(melange_solver::solver::DeviceEntry::new_diode(
                    diode,
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                // Find model parameters from netlist
                let model_params = find_bjt_model(netlist, &dev_info.name);
                let is = model_params.0.unwrap_or(1e-15);
                let beta_f = model_params.1.unwrap_or(200.0);
                let beta_r = model_params.2.unwrap_or(3.0);

                // Determine polarity from model type
                let polarity = find_bjt_polarity(netlist, &dev_info.name);

                let bjt = melange_devices::BjtEbersMoll::new_room_temp(is, beta_f, beta_r, polarity);
                devices.push(melange_solver::solver::DeviceEntry::new_bjt(
                    bjt,
                    dev_info.start_idx,
                ));
            }
            _ => {
                // Other device types not yet supported
                return Err(ValidationError::Solver(format!(
                    "Device type {:?} not yet supported",
                    dev_info.device_type
                )));
            }
        }
    }

    Ok(devices)
}

/// Find diode model parameters from netlist
/// Returns (IS, N) if found
fn find_diode_model(netlist: &melange_solver::parser::Netlist, device_name: &str) -> (Option<f64>, Option<f64>) {
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
fn find_bjt_polarity(netlist: &melange_solver::parser::Netlist, device_name: &str) -> melange_devices::BjtPolarity {
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

/// Skip a test if ngspice is not available
///
/// Useful for tests that require ngspice to be installed.
///
/// # Example
///
/// ```rust,no_run
/// use melange_validate::skip_if_no_ngspice;
///
/// #[test]
/// fn test_validation() {
///     skip_if_no_ngspice();
///     // ... rest of test
/// }
/// ```
pub fn skip_if_no_ngspice() {
    if !spice_runner::is_ngspice_available() {
        eprintln!("Skipping test: ngspice not available");
        // In actual test code, you would use:
        // return;
        // or
        // panic!("ngspice not available");
    }
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
        assert_eq!(builder.options.circuit_name, Some("test_circuit".to_string()));
    }

    #[test]
    fn test_comparison_config_default() {
        let config = ComparisonConfig::default();
        assert!(config.rms_error_tolerance > 0.0);
        assert!(config.correlation_min > 0.99);
    }
}
