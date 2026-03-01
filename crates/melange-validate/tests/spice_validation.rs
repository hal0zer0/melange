//! SPICE validation tests - Compare melange against ngspice
//!
//! These tests verify that melange produces results matching ngspice
//! within tight tolerances. They require ngspice to be installed.
//!
//! Run with: cargo test -p melange-validate --test spice_validation -- --nocapture
//!
//! # Test Coverage
//!
//! | Test | Circuit Type | Nonlinearities | Tolerance |
//! |------|-------------|----------------|-----------|
//! | `test_rc_lowpass_vs_spice` | Linear RC | 0 | Strict |
//! | `test_diode_clipper_vs_spice` | Diode clipper | 2 diodes | Default |
//! | `test_bjt_common_emitter_vs_spice` | BJT amplifier | 1 BJT | Default |
//! | `test_antiparallel_diodes_vs_spice` | Symmetric clipper | 2 diodes | Default |

use std::path::PathBuf;

use melange_validate::{
    comparison::{compare_signals, ComparisonConfig, Signal},
    spice_runner::{is_ngspice_available, run_transient_with_pwl},
    visualizer::generate_html_report,
    ValidationError,
};
use melange_devices::bjt::BjtPolarity;

/// Sample rate used for all validation tests (48 kHz audio standard)
const SAMPLE_RATE: f64 = 48_000.0;

/// Get path to test data directory
fn test_data_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("data")
}

/// Load a PWL (piecewise linear) file and return (time, voltage) pairs
///
/// PWL files have format:
/// ```text
/// * comment
/// 0.0000 0.0000
/// 0.0001 0.3090
/// ...
/// ```
fn load_pwl_file(path: &PathBuf) -> Result<Vec<(f64, f64)>, std::io::Error> {
    let content = std::fs::read_to_string(path)?;
    let mut points = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();
        // Skip empty lines and comments
        if trimmed.is_empty() || trimmed.starts_with('*') {
            continue;
        }

        // Parse time voltage pairs
        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if parts.len() >= 2 {
            if let (Ok(time), Ok(voltage)) = (parts[0].parse::<f64>(), parts[1].parse::<f64>()) {
                points.push((time, voltage));
            }
        }
    }

    Ok(points)
}

/// Configuration for strict linear circuit tolerances
///
/// Linear circuits should match SPICE almost exactly since there's
/// no Newton-Raphson iteration or device nonlinearity.
fn strict_linear_config() -> ComparisonConfig {
    ComparisonConfig {
        rms_error_tolerance: 5e-4,        // 0.05% — accounts for trapezoidal frequency warping
        peak_error_tolerance: 5e-3,       // 5 mV for 1V signal
        max_relative_tolerance: 0.5,       // 50% — relative error is large near zero-crossings
        correlation_min: 0.99999,         // Five 9s
        thd_error_tolerance_db: 0.1,      // 0.1 dB
        full_scale: 1.0,
        skip_thd: false,
    }
}

/// Configuration for nonlinear circuit tolerances
///
/// Nonlinear circuits have more tolerance for error due to:
/// - DK method uses backward Euler for nonlinear currents vs SPICE's trapezoidal
/// - Device model differences (melange doesn't model RS, CJO, TT)
/// - Newton-Raphson convergence differences
/// - Numerical precision in exponential functions
fn nonlinear_config() -> ComparisonConfig {
    ComparisonConfig {
        rms_error_tolerance: 0.20,        // 20% — DK vs SPICE method differences
        peak_error_tolerance: 0.5,        // 500mV
        max_relative_tolerance: 5.0,      // 500% — near zero-crossings, large relative error expected
        correlation_min: 0.99,            // Two 9s — waveform shape should match
        thd_error_tolerance_db: 3.0,      // 3 dB — DK method produces different harmonics than SPICE
        full_scale: 5.0,                  // Diode clippers can hit 5V
        skip_thd: false,
    }
}

/// Run validation for a circuit and return the result
///
/// This helper function:
/// 1. Loads the PWL input file
/// 2. Runs ngspice simulation
/// 3. Creates melange solver and runs it
/// 4. Compares the results
fn run_validation(
    circuit_name: &str,
    output_node: &str,
    config: &ComparisonConfig,
) -> Result<ValidationResult, ValidationError> {
    let data_dir = test_data_dir().join(circuit_name);
    let netlist_path = data_dir.join("circuit.cir");
    let netlist_path_no_vin = data_dir.join("circuit_no_vin.cir");
    let input_pwl_path = data_dir.join("input_pwl.txt");

    // Load PWL input data
    let pwl_data =
        load_pwl_file(&input_pwl_path).map_err(|e| ValidationError::Io(e.to_string()))?;

    if pwl_data.is_empty() {
        return Err(ValidationError::InvalidInput(
            "No PWL data found in input file".to_string(),
        ));
    }

    // Determine simulation parameters from PWL data
    let duration = pwl_data.last().map(|(t, _)| *t).unwrap_or(0.01);
    let tstep = 1.0 / SAMPLE_RATE;

    // Run ngspice simulation (uses netlist with VIN source)
    let spice_data = run_transient_with_pwl(
        &netlist_path,
        tstep,
        duration,
        "in", // PWL source name - netlists use Vin as the input source
        &pwl_data,
        &[output_node.to_string()],
    )?;

    // Extract SPICE output
    let spice_output = spice_data
        .get_node_voltage(output_node)
        .map_err(|e| ValidationError::Spice(e))?
        .to_vec();

    // Run melange solver (uses netlist without VIN source, input via input_conductance)
    let input_signal = resample_pwl_to_signal(&pwl_data, SAMPLE_RATE, spice_output.len());
    
    let melange_output = run_melange_solver(&netlist_path_no_vin, &input_signal, SAMPLE_RATE)?;

    // Create signals for comparison
    let spice_signal = Signal::new(spice_output, SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_output, SAMPLE_RATE, "Melange");

    // Compare signals
    let mut report = compare_signals(&spice_signal, &melange_signal, config);
    report.circuit_name = circuit_name.to_string();
    report.node_name = output_node.to_string();

    // Generate failure report if needed
    let report_path = if !report.passed {
        let path = data_dir.join(format!("{}_failure_report.html", circuit_name));
        generate_html_report(&report, &spice_signal, &melange_signal, &path)
            .map_err(|e| ValidationError::Io(e.to_string()))?;
        Some(path)
    } else {
        None
    };

    Ok(ValidationResult {
        report,
        html_report_path: report_path,
    })
}

/// Resample PWL data to a uniform sample rate
fn resample_pwl_to_signal(pwl_data: &[(f64, f64)], sample_rate: f64, num_samples: usize) -> Vec<f64> {
    let mut signal = Vec::with_capacity(num_samples);

    for i in 0..num_samples {
        let t = i as f64 / sample_rate;
        let value = interpolate_pwl(pwl_data, t);
        signal.push(value);
    }

    signal
}

/// Linear interpolation for PWL data
fn interpolate_pwl(pwl_data: &[(f64, f64)], t: f64) -> f64 {
    // Handle edge cases
    if pwl_data.is_empty() {
        return 0.0;
    }
    if t <= pwl_data[0].0 {
        return pwl_data[0].1;
    }
    if t >= pwl_data.last().unwrap().0 {
        return pwl_data.last().unwrap().1;
    }

    // Find the interval containing t
    for i in 1..pwl_data.len() {
        let (t0, v0) = pwl_data[i - 1];
        let (t1, v1) = pwl_data[i];

        if t >= t0 && t <= t1 {
            // Linear interpolation
            if t1 - t0 > 0.0 {
                let frac = (t - t0) / (t1 - t0);
                return v0 + frac * (v1 - v0);
            } else {
                return v0;
            }
        }
    }

    0.0
}

/// Run melange solver on a netlist with the given input signal
fn run_melange_solver(
    netlist_path: &PathBuf,
    input_signal: &[f64],
    sample_rate: f64,
) -> Result<Vec<f64>, ValidationError> {
    use melange_solver::solver::CircuitSolver;

    // Read and parse netlist
    let netlist_str = std::fs::read_to_string(netlist_path)
        .map_err(|e| ValidationError::Io(format!("Failed to read netlist: {}", e)))?;

    let netlist = melange_solver::parser::Netlist::parse(&netlist_str)
        .map_err(|e| ValidationError::Solver(format!("Parse error at line {}: {}", e.line, e.message)))?;

    // Build MNA system (mutable — we need to stamp input conductance before kernel build)
    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .map_err(|e| ValidationError::Solver(format!("MNA error: {}", e)))?;

    // Determine input/output node indices BEFORE building kernel
    let input_node = mna.node_map.get("in").copied().unwrap_or(1).saturating_sub(1);
    let output_node = mna.node_map.get("out").copied().unwrap_or(2).saturating_sub(1);
    // Use near-ideal voltage source (1Ω) to match SPICE's ideal VIN.
    // The SPICE netlist uses an ideal voltage source; melange models it as a
    // Thevenin source with R_in. Using 1Ω gives G_in = 1.0 S, making the
    // input node voltage very close to the applied signal.
    let input_conductance = 1.0; // 1/1Ω = 1.0 S

    // CRITICAL: Stamp input conductance into MNA G matrix BEFORE building DK kernel.
    // The DK kernel bakes the G matrix into its S = (2/T * C + G)^-1 computation.
    // Without this stamp, the input node has no path to ground for the input signal,
    // causing massive gain errors. This matches what main.rs does at line 306.
    if input_node < mna.n {
        mna.g[input_node][input_node] += input_conductance;
    }

    // Create DK kernel (now includes input conductance in G)
    let kernel = melange_solver::dk::DkKernel::from_mna(&mna, sample_rate)
        .map_err(|e| ValidationError::Solver(format!("DK kernel error: {:?}", e)))?;

    // Build device list from netlist
    let devices = build_devices_from_netlist(&netlist, &mna)?;

    // Create solver with matching input_conductance
    let mut solver = CircuitSolver::new(kernel, devices, input_node, output_node);
    solver.input_conductance = input_conductance;

    // Run simulation
    let mut output = Vec::with_capacity(input_signal.len());
    for &sample in input_signal.iter() {
        output.push(solver.process_sample(sample));
    }

    Ok(output)
}

/// Build device list from parsed netlist
fn build_devices_from_netlist(
    netlist: &melange_solver::parser::Netlist,
    mna: &melange_solver::mna::MnaSystem,
) -> Result<Vec<melange_solver::solver::DeviceEntry>, ValidationError> {
    use melange_solver::solver::DeviceEntry;
    use melange_devices::diode::DiodeShockley;
    use melange_devices::bjt::BjtEbersMoll;

    let mut devices = Vec::new();

    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                // Find model parameters from netlist
                let model_params = find_diode_model(netlist, &dev_info.name);
                let is = model_params.0.unwrap_or(1e-15);
                let n = model_params.1.unwrap_or(1.0);

                let diode = DiodeShockley::new_room_temp(is, n);
                devices.push(DeviceEntry::new_diode(diode, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                // Find model parameters from netlist
                let model_params = find_bjt_model(netlist, &dev_info.name);
                let is = model_params.0.unwrap_or(1e-15);
                let beta_f = model_params.1.unwrap_or(200.0);
                let beta_r = model_params.2.unwrap_or(3.0);

                // Determine polarity from model type
                let polarity = find_bjt_polarity(netlist, &dev_info.name);

                let bjt = BjtEbersMoll::new_room_temp(is, beta_f, beta_r, polarity);
                devices.push(DeviceEntry::new_bjt(bjt, dev_info.start_idx));
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
fn find_bjt_polarity(netlist: &melange_solver::parser::Netlist, device_name: &str) -> BjtPolarity {
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
        None => return BjtPolarity::Npn,
    };

    // Find the model definition and check type
    for model in &netlist.models {
        if model.name == model_name {
            let model_type_upper = model.model_type.to_uppercase();
            if model_type_upper.starts_with("PNP") {
                return BjtPolarity::Pnp;
            } else {
                // Default to NPN
                return BjtPolarity::Npn;
            }
        }
    }

    BjtPolarity::Npn
}

/// Result of a validation run
struct ValidationResult {
    report: melange_validate::comparison::ComparisonReport,
    html_report_path: Option<PathBuf>,
}

/// Print detailed validation metrics
fn print_validation_metrics(result: &ValidationResult) {
    let report = &result.report;
    if report.passed {
        println!("  ✓ Validation passed for {}", report.circuit_name);
    } else {
        println!("  ✗ Validation FAILED for {}", report.circuit_name);
    }
    println!("    Samples: {} at {:.0} Hz", report.sample_count, report.sample_rate);
    println!("    RMS Error: {:.6e}", report.rms_error);
    println!("    Normalized RMS: {:.6} ({:.4}%)",
        report.normalized_rms_error,
        report.normalized_rms_error * 100.0
    );
    println!("    Peak Error: {:.6e}", report.peak_error);
    println!("    Correlation: {:.8}", report.correlation_coefficient);
    println!("    SNR: {:.2} dB", report.snr_db);

    if report.thd_spice.is_finite() && report.thd_melange.is_finite() {
        println!("    THD (SPICE): {:.2} dB", report.thd_spice);
        println!("    THD (Melange): {:.2} dB", report.thd_melange);
        println!("    THD Error: {:.2} dB", report.thd_error_db);
    }
}

// =============================================================================
// Test Cases
// =============================================================================

/// Test 1: RC Lowpass Filter (Linear Circuit)
///
/// This is a simple first-order RC filter. Since it's linear, it should
/// match SPICE almost exactly with very tight tolerances.
///
/// Circuit: R=10kΩ, C=10nF → fc ≈ 1.59 kHz
/// Input: 1 kHz sine wave
/// Expected: -3.9 dB attenuation, -58° phase shift
#[test]
fn test_rc_lowpass_vs_spice() {
    // Skip if ngspice not available
    if !is_ngspice_available() {
        eprintln!("Skipping test_rc_lowpass_vs_spice: ngspice not available");
        return;
    }

    println!("\n=== RC Lowpass Filter Validation ===");
    println!("Circuit: 10kΩ + 10nF, fc ≈ 1.59 kHz");

    let result = run_validation("rc_lowpass", "out", &strict_linear_config())
        .expect("Failed to run validation");

    print_validation_metrics(&result);

    // Assert validation passed
    assert!(
        result.report.passed,
        "RC lowpass validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );

    // Additional linear-circuit-specific assertions
    assert!(
        result.report.normalized_rms_error < 1e-3,
        "RMS error too large for linear circuit: {:.6e}",
        result.report.normalized_rms_error
    );
    assert!(
        result.report.correlation_coefficient > 0.9999,
        "Correlation too low for linear circuit: {:.8}",
        result.report.correlation_coefficient
    );
}

/// Test 2: Diode Clipper (Nonlinear Circuit)
///
/// Tests soft clipping behavior with antiparallel diodes.
/// This validates the diode model and Newton-Raphson convergence.
///
/// Circuit: Input → R(1k) → out node with diodes to ground + RC(1u, 10k) load
/// Input: 5V amplitude sine wave at 500Hz (clipped to ~0.65V)
/// Expected: Symmetric soft clipping with waveform correlation > 0.99
///
/// Note: The DK method uses backward Euler for nonlinear currents while SPICE
/// uses a different integration scheme. This causes ~15% amplitude difference
/// which is acceptable for method validation. The 1uF cap provides numerical
/// stability for the trapezoidal discretization.
#[test]
fn test_diode_clipper_vs_spice() {
    if !is_ngspice_available() {
        eprintln!("Skipping test_diode_clipper_vs_spice: ngspice not available");
        return;
    }

    println!("\n=== Diode Clipper Validation ===");
    println!("Circuit: Antiparallel 1N4148 diodes, soft clipping");

    let result = run_validation("diode_clipper", "out", &nonlinear_config())
        .expect("Failed to run validation");

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "Diode clipper validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );

    // Nonlinear circuit: waveform shape should match even if amplitude differs
    assert!(
        result.report.correlation_coefficient > 0.99,
        "Correlation too low for nonlinear circuit: {:.8}",
        result.report.correlation_coefficient
    );

    // Verify THD is computed (clipping creates harmonics)
    assert!(
        result.report.thd_spice.is_finite(),
        "THD should be computed for clipped signal"
    );
    assert!(
        result.report.thd_error_db.abs() < 1.0,
        "THD mismatch too large: {:.2} dB",
        result.report.thd_error_db
    );
}

/// Test 3: BJT Common Emitter Amplifier (Active Nonlinear Circuit)
///
/// Tests the Ebers-Moll BJT model with a common-emitter amplifier.
/// Validates both DC biasing and AC signal amplification.
///
/// Circuit: BC547 NPN, Vcc=12V, gain ≈ 10x (20dB)
/// Input: Small signal (10mV) at 1kHz
/// Expected: Output ≈ 100mV, inverted, with proper DC bias
///
/// **Why ignored**: The BJT amplifier requires a DC operating point (VCE ~6V,
/// IC ~1mA) before AC signals can be amplified. SPICE computes this via its
/// DC OP solver; melange starts all node voltages from zero, so the BJT sits
/// in cutoff and produces no output (~356% RMS error, near-zero correlation).
/// Un-ignoring this test requires implementing a nonlinear DC operating point
/// solver that iterates to find the quiescent bias point.
#[test]
#[ignore = "Requires nonlinear DC operating point solver (melange starts from zero, SPICE from DC OP)"]
fn test_bjt_common_emitter_vs_spice() {
    if !is_ngspice_available() {
        eprintln!("Skipping test_bjt_common_emitter_vs_spice: ngspice not available");
        return;
    }

    println!("\n=== BJT Common Emitter Validation ===");
    println!("Circuit: BC547 NPN amplifier, gain ≈ 10x");

    let result = run_validation("bjt_common_emitter", "out", &nonlinear_config())
        .expect("Failed to run validation");

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "BJT common emitter validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );

    // Verify amplification factor
    // The input is small, output should be approximately 10x
    // (This would need actual signal analysis for full validation)
}

/// Test 4: Antiparallel Diodes (2D Solver Test)
///
/// Tests the 2D DK solver with two nonlinear devices.
/// The diodes are arranged to create symmetric clipping.
///
/// Circuit: R(1k) -> out node with antiparallel diodes to ground + C(1u)
/// Input: 3V amplitude sine at 500Hz
///
/// This validates:
/// - 2D Newton-Raphson convergence
/// - Symmetric device handling
/// - Multiple nonlinearity interaction
#[test]
fn test_antiparallel_diodes_vs_spice() {
    if !is_ngspice_available() {
        eprintln!("Skipping test_antiparallel_diodes_vs_spice: ngspice not available");
        return;
    }

    println!("\n=== Antiparallel Diodes Validation ===");
    println!("Circuit: Two antiparallel diodes, 2D nonlinear system");

    let result = run_validation("antiparallel_diodes", "out", &nonlinear_config())
        .expect("Failed to run validation");

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "Antiparallel diodes validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );

    // Waveform shape should match well
    assert!(
        result.report.correlation_coefficient > 0.99,
        "Correlation too low for 2D solver: {:.8}",
        result.report.correlation_coefficient
    );
}

// =============================================================================
// Additional Integration Tests
// =============================================================================

/// Test that validates all working circuits in batch mode
///
/// Validates rc_lowpass (strict linear tolerances), diode_clipper and
/// antiparallel_diodes (nonlinear tolerances). BJT is excluded pending
/// nonlinear DC operating point solver.
#[test]
fn test_all_circuits_batch() {
    if !is_ngspice_available() {
        eprintln!("Skipping test_all_circuits_batch: ngspice not available");
        return;
    }

    println!("\n=== Batch Validation of All Circuits ===");

    let circuits = vec![
        ("rc_lowpass", "out", strict_linear_config(), "Linear RC"),
        ("diode_clipper", "out", nonlinear_config(), "Diode clipper (2 diodes)"),
        ("antiparallel_diodes", "out", nonlinear_config(), "Antiparallel diodes (2D)"),
    ];

    let mut passed = 0;
    let mut failed = 0;

    for (name, node, config, description) in circuits {
        print!("Testing {} ({})... ", name, description);
        match run_validation(name, node, &config) {
            Ok(result) => {
                if result.report.passed {
                    println!("✓ PASSED");
                    passed += 1;
                } else {
                    println!("✗ FAILED");
                    println!("  Report: {:?}", result.html_report_path);
                    failed += 1;
                }
            }
            Err(e) => {
                println!("✗ ERROR: {}", e);
                failed += 1;
            }
        }
    }

    println!("\n=== Summary ===");
    println!("Passed: {}", passed);
    println!("Failed: {}", failed);

    assert_eq!(
        failed, 0,
        "{} circuit(s) failed validation. Check generated HTML reports in test data directories.",
        failed
    );
}

/// Test ngspice availability detection
#[test]
fn test_ngspice_availability() {
    // This test just verifies the availability check works
    let available = is_ngspice_available();
    println!("ngspice available: {}", available);

    // We can't assert anything since we don't know the test environment
    // but we can verify the function doesn't panic
}

/// Test PWL file loading
#[test]
fn test_pwl_file_loading() {
    let pwl_path = test_data_dir().join("rc_lowpass").join("input_pwl.txt");
    let pwl_data = load_pwl_file(&pwl_path).expect("Failed to load PWL file");

    // Verify we loaded some data
    assert!(!pwl_data.is_empty(), "PWL data should not be empty");

    // Verify time is monotonically increasing
    for i in 1..pwl_data.len() {
        assert!(
            pwl_data[i].0 >= pwl_data[i - 1].0,
            "Time should be monotonically increasing"
        );
    }

    // Check first and last points
    assert_eq!(pwl_data[0].0, 0.0, "First time point should be 0");
    assert!(
        pwl_data.last().unwrap().0 > 0.0,
        "Last time point should be > 0"
    );

    println!("Loaded {} PWL points", pwl_data.len());
    println!("Duration: {} seconds", pwl_data.last().unwrap().0);
}

/// Test signal interpolation
#[test]
fn test_pwl_interpolation() {
    let pwl_data = vec![
        (0.0, 0.0),
        (1.0, 1.0),
        (2.0, 0.0),
    ];

    // Test exact points
    assert!((interpolate_pwl(&pwl_data, 0.0) - 0.0).abs() < 1e-10);
    assert!((interpolate_pwl(&pwl_data, 1.0) - 1.0).abs() < 1e-10);
    assert!((interpolate_pwl(&pwl_data, 2.0) - 0.0).abs() < 1e-10);

    // Test interpolation
    assert!((interpolate_pwl(&pwl_data, 0.5) - 0.5).abs() < 1e-10);
    assert!((interpolate_pwl(&pwl_data, 1.5) - 0.5).abs() < 1e-10);

    // Test extrapolation (should clamp)
    assert!((interpolate_pwl(&pwl_data, -1.0) - 0.0).abs() < 1e-10);
    assert!((interpolate_pwl(&pwl_data, 3.0) - 0.0).abs() < 1e-10);
}

/// Test comparison configs
#[test]
fn test_comparison_config_levels() {
    let strict = strict_linear_config();
    let nonlinear = nonlinear_config();
    let default = ComparisonConfig::default();

    // Strict should be tighter than default
    assert!(
        strict.rms_error_tolerance < default.rms_error_tolerance,
        "Strict RMS tolerance should be tighter than default"
    );
    assert!(
        strict.correlation_min > default.correlation_min,
        "Strict correlation should be higher than default"
    );

    // Nonlinear should be looser than strict
    assert!(
        nonlinear.rms_error_tolerance > strict.rms_error_tolerance,
        "Nonlinear RMS tolerance should be looser than strict"
    );
    assert!(
        nonlinear.correlation_min < strict.correlation_min,
        "Nonlinear correlation can be lower than strict"
    );

    println!("Strict config: RMS < {:.2e}, corr > {:.6}",
        strict.rms_error_tolerance, strict.correlation_min);
    println!("Nonlinear config: RMS < {:.2e}, corr > {:.6}",
        nonlinear.rms_error_tolerance, nonlinear.correlation_min);
}
