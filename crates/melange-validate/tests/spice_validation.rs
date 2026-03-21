//! SPICE validation tests - Compare melange against ngspice
//!
//! These tests verify that melange produces results matching ngspice
//! within tight tolerances. They require ngspice to be installed.
//!
//! All ngspice-dependent tests are marked `#[ignore]` so that `cargo test` clearly
//! shows them as "ignored" rather than silently passing. To run them:
//!
//! ```sh
//! cargo test -p melange-validate --test spice_validation -- --include-ignored --nocapture
//! ```
//!
//! # Test Coverage
//!
//! | Test | Circuit Type | Nonlinearities | Tolerance |
//! |------|-------------|----------------|-----------|
//! | `test_rc_lowpass_vs_spice` | Linear RC | 0 | Strict |
//! | `test_diode_clipper_vs_spice` | Diode clipper | 2 diodes | Default |
//! | `test_bjt_common_emitter_vs_spice` | BJT amplifier | 1 BJT | Default |
//! | `test_antiparallel_diodes_vs_spice` | Symmetric clipper | 2 diodes | Default |
//! | `test_rc_lowpass_step_response` | Linear RC | 0 | Strict |
//! | `test_rc_lowpass_chirp` | Linear RC | 0 | Strict |
//! | `test_diode_clipper_silence_to_signal` | Diode clipper | 2 diodes | Default |
//! | `test_wurli_preamp_vs_spice` | Wurli 200A preamp | 2 BJTs + 1 diode | BJT |

use std::path::PathBuf;

use melange_devices::bjt::BjtPolarity;
use melange_validate::{
    comparison::{compare_signals, ComparisonConfig, Signal},
    spice_runner::{is_ngspice_available, run_transient_with_thevenin_pwl},
    strip_vin_source, validate_circuit,
    visualizer::generate_html_report,
    ValidationError,
};

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
        rms_error_tolerance: 5e-4, // 0.05% — accounts for trapezoidal frequency warping
        peak_error_tolerance: 5e-3, // 5 mV for 1V signal
        max_relative_tolerance: 0.5, // 50% — relative error is large near zero-crossings
        correlation_min: 0.99999,  // Five 9s
        thd_error_tolerance_db: 0.1, // 0.1 dB
        full_scale: 1.0,
        skip_thd: false,
    }
}

/// Configuration for nonlinear circuit tolerances
///
/// Nonlinear circuits have more tolerance for error due to:
/// - Device model differences (melange doesn't model RS, CJO, TT)
/// - Newton-Raphson convergence differences
/// - Numerical precision in exponential functions
fn nonlinear_config() -> ComparisonConfig {
    ComparisonConfig {
        rms_error_tolerance: 0.20,   // 20% — DK vs SPICE method differences
        peak_error_tolerance: 0.5,   // 500mV
        max_relative_tolerance: 5.0, // 500% — near zero-crossings, large relative error expected
        correlation_min: 0.99,       // Two 9s — waveform shape should match
        thd_error_tolerance_db: 3.0, // 3 dB — DK method produces different harmonics than SPICE
        full_scale: 5.0,             // Diode clippers can hit 5V
        skip_thd: false,
    }
}

/// BJT-specific tolerances.
///
/// BJT circuits have larger error vs SPICE because:
/// - Gummel-Poon implementation differences (melange vs ngspice)
/// - ~0.5V DC offset from different model operating points
/// - ~10% gain mismatch (182x melange vs 201x ngspice)
///
/// Measured values (2026-03-19): RMS 12%, peak 0.38V, correlation 0.998
fn bjt_config() -> ComparisonConfig {
    ComparisonConfig {
        rms_error_tolerance: 0.15, // 15% — GP model with ISE/NE leakage (measured: 11.8%)
        peak_error_tolerance: 0.5, // 0.5V — startup transient (measured: 0.38V)
        max_relative_tolerance: 100.0, // near zero-crossings
        correlation_min: 0.99,     // waveform shape match (measured: 0.998)
        thd_error_tolerance_db: 5.0, // model differences in distortion
        full_scale: 10.0,          // BJT CE output can swing wider
        skip_thd: true,            // THD comparison not meaningful for nonlinear BJT
    }
}

/// Apply a 5Hz DC blocking HPF to a signal.
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
    let input_pwl_path = data_dir.join("input_pwl.txt");

    // Load PWL input data
    let pwl_data =
        load_pwl_file(&input_pwl_path).map_err(|e| ValidationError::Io(e.to_string()))?;

    if pwl_data.is_empty() {
        return Err(ValidationError::InvalidInput(
            "No PWL data found in input file".to_string(),
        ));
    }

    // Read the single netlist file
    let netlist_str =
        std::fs::read_to_string(&netlist_path).map_err(|e| ValidationError::Io(e.to_string()))?;

    // Determine simulation parameters from PWL data
    let duration = pwl_data.last().map(|(t, _)| *t).unwrap_or(0.01);
    let tstep = 1.0 / SAMPLE_RATE;

    // Run ngspice with Thevenin PWL (1-ohm series R matching melange)
    let spice_data = run_transient_with_thevenin_pwl(
        &netlist_str,
        tstep,
        duration,
        "in",
        &pwl_data,
        1.0,
        &[output_node.to_string()],
    )?;

    // Extract SPICE output
    let mut spice_output = spice_data
        .get_node_voltage(output_node)
        .map_err(|e| ValidationError::Spice(e))?
        .to_vec();

    // Strip VIN for melange solver (auto-detect and remove input voltage source)
    let (stripped_netlist, _dc_offset) = strip_vin_source(&netlist_str, "in");
    let input_signal = resample_pwl_to_signal(&pwl_data, SAMPLE_RATE, spice_output.len());

    let melange_output = run_melange_solver(&stripped_netlist, &input_signal, SAMPLE_RATE)?;

    // Apply DC blocking to SPICE output to match melange's internal DC blocker
    dc_block_signal(&mut spice_output, SAMPLE_RATE);

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
fn resample_pwl_to_signal(
    pwl_data: &[(f64, f64)],
    sample_rate: f64,
    num_samples: usize,
) -> Vec<f64> {
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

/// Run melange solver on a netlist string with the given input signal
fn run_melange_solver(
    netlist_str: &str,
    input_signal: &[f64],
    sample_rate: f64,
) -> Result<Vec<f64>, ValidationError> {
    use melange_solver::solver::CircuitSolver;

    let netlist = melange_solver::parser::Netlist::parse(netlist_str).map_err(|e| {
        ValidationError::Solver(format!("Parse error at line {}: {}", e.line, e.message))
    })?;

    // Build MNA system (mutable — we need to stamp input conductance before kernel build)
    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .map_err(|e| ValidationError::Solver(format!("MNA error: {}", e)))?;

    // Determine input/output node indices BEFORE building kernel
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    let output_node = mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);
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
    let mut solver = CircuitSolver::new(kernel, devices, input_node, output_node).unwrap();
    solver.set_input_conductance(input_conductance);

    // Initialize nonlinear DC operating point (essential for BJT circuits)
    if mna.m > 0 {
        let device_slots = build_device_slots_from_netlist(&netlist);
        solver.initialize_dc_op(&mna, &device_slots);
    }

    // Run simulation
    let mut output = Vec::with_capacity(input_signal.len());
    for &sample in input_signal.iter() {
        output.push(solver.process_sample(sample));
    }

    Ok(output)
}

#[allow(dead_code)]
/// Run melange solver on a netlist string using NodalSolver (full NR per sample).
///
/// This is more robust than `run_melange_solver` (DK/CircuitSolver) for multi-stage
/// BJT circuits where the DK method's reduced-dimension NR can diverge.
fn run_melange_solver_nodal(
    netlist_str: &str,
    input_signal: &[f64],
    sample_rate: f64,
) -> Result<Vec<f64>, ValidationError> {
    use melange_solver::solver::NodalSolver;

    let netlist = melange_solver::parser::Netlist::parse(netlist_str).map_err(|e| {
        ValidationError::Solver(format!("Parse error at line {}: {}", e.line, e.message))
    })?;

    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .map_err(|e| ValidationError::Solver(format!("MNA error: {}", e)))?;

    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    let output_node = mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);
    let input_conductance = 1.0;

    if input_node < mna.n {
        mna.g[input_node][input_node] += input_conductance;
    }

    let kernel = melange_solver::dk::DkKernel::from_mna(&mna, sample_rate)
        .map_err(|e| ValidationError::Solver(format!("DK kernel error: {:?}", e)))?;

    let device_slots = build_device_slots_from_netlist(&netlist);

    let mut solver = NodalSolver::new(
        kernel,
        &mna,
        &netlist,
        device_slots.clone(),
        input_node,
        output_node,
    )
    .expect("Failed to create nodal solver");
    solver.set_input_conductance(input_conductance);

    if mna.m > 0 {
        solver.initialize_dc_op(&mna, &device_slots);
    }

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
    use melange_devices::bjt::BjtEbersMoll;
    use melange_devices::diode::DiodeShockley;
    use melange_solver::solver::DeviceEntry;

    let mut devices = Vec::new();

    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                let model_params = find_diode_model(netlist, &dev_info.name);
                let is = model_params.0.unwrap_or(1e-15);
                let n = model_params.1.unwrap_or(1.0);

                let diode = DiodeShockley::new_room_temp(is, n);
                devices.push(DeviceEntry::new_diode(diode, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Bjt
            | melange_solver::mna::NonlinearDeviceType::BjtForwardActive => {
                let params = find_bjt_model(netlist, &dev_info.name);
                let is = params.is.unwrap_or(1e-15);
                let beta_f = params.bf.unwrap_or(200.0);
                let beta_r = params.br.unwrap_or(3.0);
                let polarity = find_bjt_polarity(netlist, &dev_info.name);
                let nf = params.nf.unwrap_or(1.0);
                let nr = params.nr.unwrap_or(1.0);
                let ise = params.ise.unwrap_or(0.0);
                let ne = params.ne.unwrap_or(1.5);
                let isc = params.isc.unwrap_or(0.0);
                let nc = params.nc.unwrap_or(1.0);
                let base_bjt = BjtEbersMoll::new_room_temp(is, beta_f, beta_r, polarity)
                    .with_nf(nf)
                    .with_nr(nr)
                    .with_leakage(ise, ne, isc, nc);

                // Use Gummel-Poon if any Early/high-injection params are finite
                let has_gp = params.vaf.is_some()
                    || params.var.is_some()
                    || params.ikf.is_some()
                    || params.ikr.is_some();
                if has_gp {
                    let gp = melange_devices::bjt::BjtGummelPoon::new(
                        base_bjt,
                        params.vaf.unwrap_or(f64::INFINITY),
                        params.var.unwrap_or(f64::INFINITY),
                        params.ikf.unwrap_or(f64::INFINITY),
                        params.ikr.unwrap_or(f64::INFINITY),
                    );
                    devices.push(DeviceEntry::new_bjt_gp(gp, dev_info.start_idx));
                } else {
                    devices.push(DeviceEntry::new_bjt(base_bjt, dev_info.start_idx));
                }
            }
            melange_solver::mna::NonlinearDeviceType::Jfet => {
                let jfet = find_jfet_device(netlist, &dev_info.name);
                devices.push(DeviceEntry::new_jfet(jfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Mosfet => {
                let mosfet = find_mosfet_device(netlist, &dev_info.name);
                devices.push(DeviceEntry::new_mosfet(mosfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Tube => {
                let tube = find_tube_device(netlist, &dev_info.name);
                devices.push(DeviceEntry::new_tube(tube, dev_info.start_idx));
            }
        }
    }

    Ok(devices)
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

/// Extracted BJT model parameters from a SPICE .model card.
#[derive(Debug, Default)]
struct BjtModelParams {
    is: Option<f64>,
    bf: Option<f64>,
    br: Option<f64>,
    nf: Option<f64>,
    nr: Option<f64>,
    /// Forward Early voltage (Gummel-Poon)
    vaf: Option<f64>,
    /// Reverse Early voltage (Gummel-Poon)
    var: Option<f64>,
    /// Forward high-injection knee current (Gummel-Poon)
    ikf: Option<f64>,
    /// Reverse high-injection knee current (Gummel-Poon)
    ikr: Option<f64>,
    /// Base-emitter leakage saturation current
    ise: Option<f64>,
    /// Base-emitter leakage emission coefficient
    ne: Option<f64>,
    /// Base-collector leakage saturation current
    isc: Option<f64>,
    /// Base-collector leakage emission coefficient
    nc: Option<f64>,
}

/// Find BJT model parameters from netlist.
///
/// Extracts Ebers-Moll params (IS, BF, BR, NF, NR) and Gummel-Poon params
/// (VAF, VAR, IKF, IKR) when present in the .model card.
fn find_bjt_model(netlist: &melange_solver::parser::Netlist, device_name: &str) -> BjtModelParams {
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
        None => return BjtModelParams::default(),
    };

    let mut params = BjtModelParams::default();

    for model in &netlist.models {
        if model.name == model_name {
            let model_type_upper = model.model_type.to_uppercase();
            if model_type_upper.starts_with("NPN") || model_type_upper.starts_with("PNP") {
                for (key, value) in &model.params {
                    match key.to_uppercase().as_str() {
                        "IS" => params.is = Some(*value),
                        "BF" => params.bf = Some(*value),
                        "BR" => params.br = Some(*value),
                        "NF" => params.nf = Some(*value),
                        "NR" => params.nr = Some(*value),
                        "VAF" | "VA" => params.vaf = Some(*value),
                        "VAR" | "VB" => params.var = Some(*value),
                        "IKF" | "JBF" => params.ikf = Some(*value),
                        "IKR" | "JBR" => params.ikr = Some(*value),
                        "ISE" => params.ise = Some(*value),
                        "NE" => params.ne = Some(*value),
                        "ISC" => params.isc = Some(*value),
                        "NC" => params.nc = Some(*value),
                        _ => {}
                    }
                }
                break;
            }
        }
    }

    params
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

/// Find JFET device from netlist, constructing a Jfet with model params.
fn find_jfet_device(
    netlist: &melange_solver::parser::Netlist,
    device_name: &str,
) -> melange_devices::Jfet {
    use melange_devices::jfet::{Jfet, JfetChannel};

    let model_name = netlist
        .elements
        .iter()
        .find_map(|e| {
            if let melange_solver::parser::Element::Jfet { name, model, .. } = e {
                if name.eq_ignore_ascii_case(device_name) {
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

    let channel = if is_p_channel {
        JfetChannel::P
    } else {
        JfetChannel::N
    };
    let default_vp = if is_p_channel { 2.0 } else { -2.0 };

    let find_param = |param: &str| -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(&model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param))
                    .map(|(_, v)| *v)
            })
    };

    let vp = find_param("VTO").unwrap_or(default_vp);
    // ngspice BETA = IDSS / VP^2, so IDSS = BETA * VP^2
    let idss = if let Some(beta) = find_param("BETA") {
        beta * vp * vp
    } else {
        find_param("IDSS").unwrap_or(2e-3)
    };
    let mut jfet = Jfet::new(channel, vp, idss);
    jfet.lambda = find_param("LAMBDA").unwrap_or(0.001);
    jfet
}

/// Find MOSFET device from netlist, constructing a Mosfet with model params.
fn find_mosfet_device(
    netlist: &melange_solver::parser::Netlist,
    device_name: &str,
) -> melange_devices::Mosfet {
    use melange_devices::mosfet::{ChannelType, Mosfet};

    let model_name = netlist
        .elements
        .iter()
        .find_map(|e| {
            if let melange_solver::parser::Element::Mosfet { name, model, .. } = e {
                if name.eq_ignore_ascii_case(device_name) {
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

    let channel = if is_p_channel {
        ChannelType::P
    } else {
        ChannelType::N
    };
    let default_vt = if is_p_channel { -2.0 } else { 2.0 };

    let find_param = |param: &str| -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(&model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param))
                    .map(|(_, v)| *v)
            })
    };

    let vt = find_param("VTO").unwrap_or(default_vt);
    let kp = find_param("KP").unwrap_or(0.1);
    let lambda = find_param("LAMBDA").unwrap_or(0.01);
    Mosfet::new(channel, vt, kp, lambda)
}

/// Find tube/triode device from netlist, constructing a KorenTriode with model params.
fn find_tube_device(
    netlist: &melange_solver::parser::Netlist,
    device_name: &str,
) -> melange_devices::KorenTriode {
    let model_name = netlist
        .elements
        .iter()
        .find_map(|e| {
            if let melange_solver::parser::Element::Triode { name, model, .. } = e {
                if name.eq_ignore_ascii_case(device_name) {
                    Some(model.clone())
                } else {
                    None
                }
            } else {
                None
            }
        })
        .unwrap_or_default();

    let find_param = |param: &str| -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(&model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param))
                    .map(|(_, v)| *v)
            })
    };

    let mu = find_param("MU").unwrap_or(100.0);
    let ex = find_param("EX").unwrap_or(1.4);
    let kg1 = find_param("KG1").unwrap_or(1060.0);
    let kp = find_param("KP").unwrap_or(600.0);
    let kvb = find_param("KVB").unwrap_or(300.0);
    let ig_max = find_param("IG_MAX").unwrap_or(2e-3);
    let vgk_onset = find_param("VGK_ONSET").unwrap_or(0.5);
    let lambda = find_param("LAMBDA").unwrap_or(0.0);
    melange_devices::KorenTriode::with_all_params(mu, ex, kg1, kp, kvb, ig_max, vgk_onset, lambda)
}

/// Build device slots for the DC OP solver from a parsed netlist.
fn build_device_slots_from_netlist(
    netlist: &melange_solver::parser::Netlist,
) -> Vec<melange_solver::device_types::DeviceSlot> {
    use melange_solver::device_types::{
        BjtParams, DeviceParams, DeviceSlot, DeviceType, DiodeParams, JfetParams, MosfetParams,
        TubeParams,
    };

    let vt = 0.02585;
    let mut slots = Vec::new();
    let mut dim_offset = 0;

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

    for elem in &netlist.elements {
        match elem {
            melange_solver::parser::Element::Diode { name, model: _, .. } => {
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
            melange_solver::parser::Element::Bjt { name: _, model, .. } => {
                let bjt_params = find_bjt_model_by_name(netlist, model);
                let is = bjt_params.is.unwrap_or(1e-14);
                let beta_f = bjt_params.bf.unwrap_or(200.0);
                let beta_r = bjt_params.br.unwrap_or(3.0);
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
                        vaf: bjt_params.vaf.unwrap_or(f64::INFINITY),
                        var: bjt_params.var.unwrap_or(f64::INFINITY),
                        ikf: bjt_params.ikf.unwrap_or(f64::INFINITY),
                        ikr: bjt_params.ikr.unwrap_or(f64::INFINITY),
                        cje: 0.0,
                        cjc: 0.0,
                        nf: bjt_params.nf.unwrap_or(1.0),
                        nr: bjt_params.nr.unwrap_or(1.0),
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
            melange_solver::parser::Element::Jfet { model, .. } => {
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model))
                    .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
                    .unwrap_or(false);
                let default_vp = if is_p_channel { 2.0 } else { -2.0 };
                slots.push(DeviceSlot {
                    device_type: DeviceType::Jfet,
                    start_idx: dim_offset,
                    dimension: 2,
                    params: DeviceParams::Jfet(JfetParams {
                        idss: {
                            let vp = find_model_param(model, "VTO").unwrap_or(default_vp);
                            if let Some(beta) = find_model_param(model, "BETA") {
                                beta * vp * vp
                            } else {
                                find_model_param(model, "IDSS").unwrap_or(2e-3)
                            }
                        },
                        vp: find_model_param(model, "VTO").unwrap_or(default_vp),
                        lambda: find_model_param(model, "LAMBDA").unwrap_or(0.001),
                        is_p_channel,
                        cgs: 0.0,
                        cgd: 0.0,
                        rd: 0.0,
                        rs_param: 0.0,
                    }),
                    has_internal_mna_nodes: false,
                });
                dim_offset += 2;
            }
            melange_solver::parser::Element::Mosfet { model, .. } => {
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model))
                    .map(|m| m.model_type.to_uppercase().starts_with("PM"))
                    .unwrap_or(false);
                let default_vt = if is_p_channel { -2.0 } else { 2.0 };
                slots.push(DeviceSlot {
                    device_type: DeviceType::Mosfet,
                    start_idx: dim_offset,
                    dimension: 2,
                    params: DeviceParams::Mosfet(MosfetParams {
                        kp: find_model_param(model, "KP").unwrap_or(0.1),
                        vt: find_model_param(model, "VTO").unwrap_or(default_vt),
                        lambda: find_model_param(model, "LAMBDA").unwrap_or(0.01),
                        is_p_channel,
                        cgs: 0.0,
                        cgd: 0.0,
                        rd: 0.0,
                        rs_param: 0.0,
                        gamma: 0.0,
                        phi: 0.6,
                        source_node: 0,
                        bulk_node: 0,
                    }),
                    has_internal_mna_nodes: false,
                });
                dim_offset += 2;
            }
            melange_solver::parser::Element::Triode { model, .. } => {
                slots.push(DeviceSlot {
                    device_type: DeviceType::Tube,
                    start_idx: dim_offset,
                    dimension: 2,
                    params: DeviceParams::Tube(TubeParams {
                        mu: find_model_param(model, "MU").unwrap_or(100.0),
                        ex: find_model_param(model, "EX").unwrap_or(1.4),
                        kg1: find_model_param(model, "KG1").unwrap_or(1060.0),
                        kp: find_model_param(model, "KP").unwrap_or(600.0),
                        kvb: find_model_param(model, "KVB").unwrap_or(300.0),
                        ig_max: find_model_param(model, "IG_MAX").unwrap_or(2e-3),
                        vgk_onset: find_model_param(model, "VGK_ONSET").unwrap_or(0.5),
                        lambda: find_model_param(model, "LAMBDA").unwrap_or(0.0),
                        ccg: 0.0,
                        cgp: 0.0,
                        ccp: 0.0,
                        rgi: 0.0,
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
///
/// Returns a `BjtModelParams` with all Ebers-Moll and Gummel-Poon params.
fn find_bjt_model_by_name(
    netlist: &melange_solver::parser::Netlist,
    model_name: &str,
) -> BjtModelParams {
    let mut params = BjtModelParams::default();

    for model in &netlist.models {
        if model.name.eq_ignore_ascii_case(model_name) {
            for (key, value) in &model.params {
                match key.to_uppercase().as_str() {
                    "IS" => params.is = Some(*value),
                    "BF" => params.bf = Some(*value),
                    "BR" => params.br = Some(*value),
                    "NF" => params.nf = Some(*value),
                    "NR" => params.nr = Some(*value),
                    "VAF" | "VA" => params.vaf = Some(*value),
                    "VAR" | "VB" => params.var = Some(*value),
                    "IKF" | "JBF" => params.ikf = Some(*value),
                    "IKR" | "JBR" => params.ikr = Some(*value),
                    "ISE" => params.ise = Some(*value),
                    "NE" => params.ne = Some(*value),
                    "ISC" => params.isc = Some(*value),
                    "NC" => params.nc = Some(*value),
                    _ => {}
                }
            }
            break;
        }
    }

    params
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
    println!(
        "    Samples: {} at {:.0} Hz",
        report.sample_count, report.sample_rate
    );
    println!("    RMS Error: {:.6e}", report.rms_error);
    println!(
        "    Normalized RMS: {:.6} ({:.4}%)",
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
#[ignore] // requires ngspice
fn test_rc_lowpass_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

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
#[ignore] // requires ngspice
fn test_diode_clipper_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

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
#[ignore] // requires ngspice
fn test_bjt_common_emitter_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== BJT Common Emitter Validation ===");
    println!("Circuit: BC547 NPN amplifier, gain ≈ 10x");

    // Run validation with signal access for gain analysis
    let data_dir = test_data_dir().join("bjt_common_emitter");
    let netlist_path = data_dir.join("circuit.cir");
    let input_pwl_path = data_dir.join("input_pwl.txt");

    let netlist_str = std::fs::read_to_string(&netlist_path).expect("Failed to read netlist");
    let pwl_data = load_pwl_file(&input_pwl_path).expect("Failed to load PWL");
    let duration = pwl_data.last().map(|(t, _)| *t).unwrap_or(0.01);
    let tstep = 1.0 / SAMPLE_RATE;

    let spice_data = run_transient_with_thevenin_pwl(
        &netlist_str,
        tstep,
        duration,
        "in",
        &pwl_data,
        1.0,
        &["out".to_string()],
    )
    .expect("ngspice failed");

    let mut spice_output = spice_data.get_node_voltage("out").unwrap().to_vec();
    // DC-block SPICE output to match melange's internal DC blocking filter.
    // The BJT CE output has ~6V DC bias at the collector; without blocking,
    // the ~0.5V model DC offset dominates the RMS error.
    dc_block_signal(&mut spice_output, SAMPLE_RATE);
    let input_signal = resample_pwl_to_signal(&pwl_data, SAMPLE_RATE, spice_output.len());
    let (stripped_netlist, _) = strip_vin_source(&netlist_str, "in");
    let melange_output = run_melange_solver(&stripped_netlist, &input_signal, SAMPLE_RATE)
        .expect("melange solver failed");

    let config = bjt_config();
    let spice_signal = Signal::new(spice_output.clone(), SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_output.clone(), SAMPLE_RATE, "Melange");
    let mut report = compare_signals(&spice_signal, &melange_signal, &config);
    report.circuit_name = "bjt_common_emitter".to_string();
    report.node_name = "out".to_string();

    let result = ValidationResult {
        report,
        html_report_path: None,
    };

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "BJT common emitter validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );

    // --- Gain verification ---
    // Input: 10mV sine (20mV peak-to-peak)
    // Expected: CE amplifier with gain ~100-200x
    let input_pp = input_signal
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        - input_signal.iter().cloned().fold(f64::INFINITY, f64::min);
    let melange_pp = melange_output
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        - melange_output.iter().cloned().fold(f64::INFINITY, f64::min);
    let spice_pp = spice_output
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        - spice_output.iter().cloned().fold(f64::INFINITY, f64::min);

    let melange_gain = melange_pp / input_pp;
    let spice_gain = spice_pp / input_pp;

    println!("    Input PP: {:.4} V", input_pp);
    println!(
        "    SPICE output PP: {:.4} V (gain: {:.1}x)",
        spice_pp, spice_gain
    );
    println!(
        "    Melange output PP: {:.4} V (gain: {:.1}x)",
        melange_pp, melange_gain
    );

    // Melange must actually amplify the signal (gain > 50x for a CE stage)
    assert!(
        melange_gain > 50.0,
        "BJT CE amplifier should have significant gain, got {:.1}x",
        melange_gain
    );
    // Melange gain should be within 2x of SPICE gain (allow for model differences)
    let gain_ratio = melange_gain / spice_gain;
    assert!(
        gain_ratio > 0.5 && gain_ratio < 2.0,
        "Melange gain ({:.1}x) should be within 2x of SPICE gain ({:.1}x), ratio={:.2}",
        melange_gain,
        spice_gain,
        gain_ratio
    );
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
#[ignore] // requires ngspice
fn test_antiparallel_diodes_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

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

/// Test 5: Op-Amp Inverting Amplifier (Linear, M=0)
///
/// Validates melange's VCCS-based op-amp model against ngspice.
/// Gain = -R2/R1 = -100k/10k = -10.
/// Uses VCCS (G element) + Rout to match melange's op-amp model exactly.
///
/// This is a linear circuit (M=0), so tolerances should be strict.
#[test]
#[ignore] // requires ngspice
fn test_opamp_inverting_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Op-Amp Inverting Amplifier Validation ===");
    println!("Circuit: Gain=-10, VCCS model (AOL=200k, ROUT=1)");

    let result = run_validation("opamp_inverting", "out", &strict_linear_config())
        .expect("Failed to run validation");

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "Op-amp inverting validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );
}

/// Test 6: JFET Common Source Amplifier
///
/// N-channel JFET with self-bias. Tests the 2D Shichman-Hodges model.
///
#[test]
#[ignore] // requires ngspice
fn test_jfet_common_source_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== JFET Common Source Amplifier Validation ===");
    println!("Circuit: N-channel JFET, Rd=2.2k, Rs=1k, VDD=12V");

    let config = ComparisonConfig {
        rms_error_tolerance: 0.10,
        peak_error_tolerance: 0.5,
        max_relative_tolerance: 5.0,
        correlation_min: 0.999,
        thd_error_tolerance_db: 5.0, // DK method produces different harmonics than SPICE
        full_scale: 5.0,
        skip_thd: false,
    };

    let result =
        run_validation("jfet_common_source", "out", &config).expect("Failed to run validation");

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "JFET common source validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );
}

/// Test 7: MOSFET Common Source Amplifier
///
/// N-channel MOSFET Level 1 with voltage divider bias. Tests the 2D model.
///
#[test]
#[ignore] // requires ngspice
fn test_mosfet_common_source_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== MOSFET Common Source Amplifier Validation ===");
    println!("Circuit: N-channel MOSFET, Rd=1k, VDD=5V");

    let config = ComparisonConfig {
        rms_error_tolerance: 0.10,
        peak_error_tolerance: 0.5,
        max_relative_tolerance: 5.0,
        correlation_min: 0.999,
        thd_error_tolerance_db: 5.0,
        full_scale: 5.0,
        skip_thd: true, // small-signal linear region: THD too low to measure reliably
    };

    let result =
        run_validation("mosfet_common_source", "out", &config).expect("Failed to run validation");

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "MOSFET common source validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );
}

/// Test 8: Wurlitzer 200A Preamp (Multi-Stage BJT + Diode, M=5)
///
/// Two-stage direct-coupled NPN CE amplifier (2N5089) with series-series
/// emitter NFB and a diode protection. 2 BJTs (4D) + 1 diode (1D) = M=5.
///
/// Circuit: R1(22k) → Cin(22nF) → Q1(150k Rc, 33k Re) → Q2(1.8k Rc) → R9(6.8k) → out
///          R10(56k) feedback from output to Q1 emitter, LDR(100k) shunt to ground
///          D1(1N4148) reverse-biased clamp at Q1 base
/// Input: 100mV sine at 1kHz
/// Expected: Output ≈ 9.1V DC bias, ~2.5x AC gain (7-8 dB)
///
/// **Note**: This circuit's runtime DK solver (CircuitSolver) currently fails to
/// converge because the nonlinear DC OP solver cannot find the correct bias point
/// for a two-stage direct-coupled BJT amplifier (M=5). The codegen path works
/// correctly. This test serves as a regression target for runtime solver improvements.
#[test]
#[ignore] // requires ngspice
fn test_wurli_preamp_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Wurlitzer 200A Preamp Validation ===");
    println!("Circuit: 2-stage NPN (2N5089), M=5, LDR=100k fixed");

    let data_dir = test_data_dir().join("wurli_preamp");
    let netlist_path = data_dir.join("circuit.cir");
    let input_pwl_path = data_dir.join("input_pwl.txt");

    let netlist_str = std::fs::read_to_string(&netlist_path).expect("Failed to read netlist");
    let pwl_data = load_pwl_file(&input_pwl_path).expect("Failed to load PWL");
    let duration = pwl_data.last().map(|(t, _)| *t).unwrap_or(0.01);
    let tstep = 1.0 / SAMPLE_RATE;

    // --- Run ngspice ---
    let spice_data = run_transient_with_thevenin_pwl(
        &netlist_str,
        tstep,
        duration,
        "in",
        &pwl_data,
        1.0,
        &["out".to_string()],
    )
    .expect("ngspice failed");

    let mut spice_output = spice_data.get_node_voltage("out").unwrap().to_vec();
    // DC-block SPICE output to match melange's internal DC blocking filter.
    // The wurli-preamp output has ~9.1V DC bias (no output coupling cap).
    dc_block_signal(&mut spice_output, SAMPLE_RATE);
    let input_signal = resample_pwl_to_signal(&pwl_data, SAMPLE_RATE, spice_output.len());

    // --- Run melange (DK/CircuitSolver) ---
    let (stripped_netlist, _) = strip_vin_source(&netlist_str, "in");
    let melange_output = run_melange_solver(&stripped_netlist, &input_signal, SAMPLE_RATE)
        .expect("melange solver failed");

    // --- Compare ---
    let config = bjt_config();
    let spice_signal = Signal::new(spice_output.clone(), SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_output.clone(), SAMPLE_RATE, "Melange");
    let mut report = compare_signals(&spice_signal, &melange_signal, &config);
    report.circuit_name = "wurli_preamp".to_string();
    report.node_name = "out".to_string();

    let result = ValidationResult {
        report,
        html_report_path: None,
    };

    print_validation_metrics(&result);

    // --- Gain verification ---
    let input_pp = input_signal
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        - input_signal.iter().cloned().fold(f64::INFINITY, f64::min);
    let melange_pp = melange_output
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        - melange_output.iter().cloned().fold(f64::INFINITY, f64::min);
    let spice_pp = spice_output
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        - spice_output.iter().cloned().fold(f64::INFINITY, f64::min);

    let spice_gain = spice_pp / input_pp;
    let melange_gain = melange_pp / input_pp;

    println!("    Input PP: {:.4} V", input_pp);
    println!(
        "    SPICE output PP: {:.4} V (gain: {:.1}x)",
        spice_pp, spice_gain
    );
    println!(
        "    Melange output PP: {:.4} V (gain: {:.1}x)",
        melange_pp, melange_gain
    );

    assert!(
        result.report.passed,
        "Wurli preamp validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );

    // Melange must actually amplify the signal (gain > 1x for a multi-stage preamp)
    assert!(
        melange_gain > 1.0,
        "Wurli preamp should amplify signal, got {:.2}x gain",
        melange_gain
    );
    // Melange gain should be within 4x of SPICE gain (allow for model differences)
    let gain_ratio = melange_gain / spice_gain;
    assert!(
        gain_ratio > 0.25 && gain_ratio < 4.0,
        "Melange gain ({:.1}x) should be within 4x of SPICE gain ({:.1}x), ratio={:.2}",
        melange_gain,
        spice_gain,
        gain_ratio
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
#[ignore] // requires ngspice
fn test_all_circuits_batch() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Batch Validation of All Circuits ===");

    let circuits = vec![
        ("rc_lowpass", "out", strict_linear_config(), "Linear RC"),
        (
            "diode_clipper",
            "out",
            nonlinear_config(),
            "Diode clipper (2 diodes)",
        ),
        (
            "antiparallel_diodes",
            "out",
            nonlinear_config(),
            "Antiparallel diodes (2D)",
        ),
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
    let pwl_data = vec![(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)];

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

// =============================================================================
// Enhanced Signal Tests (C.4)
// =============================================================================

/// Test: RC Lowpass Square Wave (Transient Response)
///
/// 500 Hz square wave at 1V tests transient step response repeatedly.
/// The RC lowpass with R=10kΩ, C=10nF has τ=100µs, so each half-cycle (1ms)
/// gives 10 time constants of settling. The square wave has zero DC on average,
/// avoiding DC blocker artifacts. 100ms tests error accumulation.
#[test]
#[ignore] // requires ngspice
fn test_rc_lowpass_step_response() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== RC Lowpass Square Wave (500 Hz, 100 ms) ===");

    let num_samples = (SAMPLE_RATE * 0.1) as usize; // 100ms
    let period_samples = (SAMPLE_RATE / 500.0) as usize; // 96 samples per cycle
    let input: Vec<f64> = (0..num_samples)
        .map(|i| {
            if (i % period_samples) < period_samples / 2 {
                1.0
            } else {
                -1.0
            }
        })
        .collect();

    let netlist_path = test_data_dir().join("rc_lowpass").join("circuit.cir");
    // Square wave tolerances: the bilinear transform can't track instantaneous
    // steps, so peak error is large (~1V) at transitions. RMS and correlation
    // still validate overall accuracy over 100ms of error accumulation.
    let config = ComparisonConfig {
        rms_error_tolerance: 0.05,   // 5% — step transitions dominate error
        peak_error_tolerance: 1.0,   // 1V — bilinear can't track instantaneous jump
        max_relative_tolerance: 1e4, // near zero-crossings
        correlation_min: 0.999,      // shape should match well despite peak errors
        thd_error_tolerance_db: 5.0,
        full_scale: 1.0,
        skip_thd: true, // square wave THD is not meaningful
    };

    let result = validate_circuit(&netlist_path, &input, SAMPLE_RATE, "out", &config)
        .expect("Square wave validation failed");

    println!("  Samples: {}", result.report.sample_count);
    println!("  RMS Error: {:.6e}", result.report.rms_error);
    println!(
        "  Correlation: {:.8}",
        result.report.correlation_coefficient
    );

    assert!(
        result.report.passed,
        "RC lowpass square wave failed:\n{}",
        result.report.summary()
    );
}

/// Test: RC Lowpass Chirp (Multi-Frequency)
///
/// Linear chirp from 100 Hz to 10 kHz over 100ms tests wideband accuracy
/// and error accumulation across 4800 samples. The chirp exercises the filter
/// from well below cutoff (1.59 kHz) to well above it.
#[test]
#[ignore] // requires ngspice
fn test_rc_lowpass_chirp() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== RC Lowpass Chirp (100 Hz → 10 kHz, 100 ms) ===");

    let duration = 0.1; // 100ms
    let num_samples = (SAMPLE_RATE * duration) as usize;
    let f_start = 100.0;
    let f_end = 10_000.0;

    // Linear chirp: phase = 2π * (f0*t + (f1-f0)*t²/(2*T))
    let input: Vec<f64> = (0..num_samples)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            let phase = 2.0
                * std::f64::consts::PI
                * (f_start * t + (f_end - f_start) * t * t / (2.0 * duration));
            phase.sin()
        })
        .collect();

    let netlist_path = test_data_dir().join("rc_lowpass").join("circuit.cir");

    // Relaxed vs pure-sine: chirp exercises high frequencies where trapezoidal
    // bilinear warping causes phase/amplitude differences vs SPICE's Gear method
    let config = ComparisonConfig {
        rms_error_tolerance: 0.02,   // 2% — trapezoidal warping at high freq
        peak_error_tolerance: 0.2,   // 200mV — instantaneous phase error near Nyquist
        max_relative_tolerance: 1e4, // near zero-crossings, relative error is huge
        correlation_min: 0.9999,     // waveform shape should still match well
        thd_error_tolerance_db: 5.0,
        full_scale: 1.0,
        skip_thd: true, // chirp has no meaningful THD
    };

    let result = validate_circuit(&netlist_path, &input, SAMPLE_RATE, "out", &config)
        .expect("Chirp validation failed");

    println!(
        "  Samples: {} ({:.0} ms)",
        result.report.sample_count,
        1000.0 * num_samples as f64 / SAMPLE_RATE
    );
    println!("  RMS Error: {:.6e}", result.report.rms_error);
    println!(
        "  Correlation: {:.8}",
        result.report.correlation_coefficient
    );

    assert!(
        result.report.passed,
        "RC lowpass chirp validation failed:\n{}",
        result.report.summary()
    );
}

/// Test: Diode Clipper Silence-to-Signal Transition
///
/// 5ms of silence followed by 15ms of 500Hz/5V sine onset. Tests NR solver
/// startup behavior when nonlinear devices transition from quiescent to active.
#[test]
#[ignore] // requires ngspice
fn test_diode_clipper_silence_to_signal() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Diode Clipper Silence-to-Signal ===");

    let silence_samples = (SAMPLE_RATE * 0.005) as usize; // 5ms
    let signal_samples = (SAMPLE_RATE * 0.015) as usize; // 15ms
    let mut input = vec![0.0; silence_samples];
    for i in 0..signal_samples {
        let t = i as f64 / SAMPLE_RATE;
        input.push(5.0 * (2.0 * std::f64::consts::PI * 500.0 * t).sin());
    }

    let netlist_path = test_data_dir().join("diode_clipper").join("circuit.cir");
    let result = validate_circuit(
        &netlist_path,
        &input,
        SAMPLE_RATE,
        "out",
        &nonlinear_config(),
    )
    .expect("Silence-to-signal validation failed");

    println!(
        "  Samples: {} ({}ms silence + {}ms signal)",
        result.report.sample_count,
        silence_samples * 1000 / SAMPLE_RATE as usize,
        signal_samples * 1000 / SAMPLE_RATE as usize
    );
    println!("  RMS Error: {:.6e}", result.report.rms_error);
    println!(
        "  Correlation: {:.8}",
        result.report.correlation_coefficient
    );

    assert!(
        result.report.passed,
        "Diode clipper silence-to-signal failed:\n{}",
        result.report.summary()
    );
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

    println!(
        "Strict config: RMS < {:.2e}, corr > {:.6}",
        strict.rms_error_tolerance, strict.correlation_min
    );
    println!(
        "Nonlinear config: RMS < {:.2e}, corr > {:.6}",
        nonlinear.rms_error_tolerance, nonlinear.correlation_min
    );
}

// =============================================================================
// Pot Modulation Tests (Trapezoidal backward-term correctness)
// =============================================================================

/// Compute the time-varying pot resistance for the tremolo LDR.
///
/// R(t) = R_center + R_depth * sin(2*pi*f_mod*t)
/// Sweeps between R_center-R_depth and R_center+R_depth.
fn pot_resistance_at(t: f64, f_mod: f64, r_center: f64, r_depth: f64) -> f64 {
    r_center + r_depth * (2.0 * std::f64::consts::PI * f_mod * t).sin()
}

/// Apply a pot resistance change to the DK kernel matrices.
///
/// Updates S (via Sherman-Morrison), K (recomputed from S), and a_neg
/// for a new pot conductance value. The `delta_g_for_a_neg` parameter
/// allows using a DIFFERENT delta_g for the backward term (A_neg) than
/// the forward term (S), which is critical for correct trapezoidal
/// discretization of time-varying resistances.
///
/// # Arguments
/// * `kernel` - The DK kernel to modify in place
/// * `pot_idx` - Index of the pot in kernel.pots
/// * `delta_g_for_s` - Conductance change for forward path (S, K): 1/R_new - g_nominal
/// * `delta_g_for_a_neg` - Conductance change for backward path (A_neg): 1/R_prev - g_nominal
/// * `s_ni_nodes` - Pre-allocated S*N_i buffer to update
fn apply_pot_to_kernel(
    kernel: &mut melange_solver::dk::DkKernel,
    pot_idx: usize,
    delta_g_for_s: f64,
    delta_g_for_a_neg: f64,
    s_ni_nodes: &mut [f64],
) {
    let n = kernel.n;
    let m = kernel.m;
    let pot = &kernel.pots[pot_idx];
    let np = pot.node_p;
    let nq = pot.node_q;

    // Step 1: Update S via Sherman-Morrison from nominal.
    // The caller resets S to S_nominal before each call, so we apply the
    // correction: S -= scale * su * su^T, where scale = delta_g / (1 + delta_g * usu).
    let denom = 1.0 + delta_g_for_s * pot.usu;
    let scale = if denom.abs() > 1e-15 {
        delta_g_for_s / denom
    } else {
        0.0
    };

    for i in 0..n {
        for j in 0..n {
            kernel.s[i * n + j] -= scale * pot.su[i] * pot.su[j];
        }
    }

    // Step 2: Recompute K = N_v * S * N_i from the corrected S.
    if m > 0 {
        for i in 0..m {
            for j in 0..m {
                let mut sum = 0.0;
                for ki in 0..n {
                    let nv_ik = kernel.n_v[i * n + ki];
                    if nv_ik == 0.0 {
                        continue;
                    }
                    for kj in 0..n {
                        sum += nv_ik * kernel.s[ki * n + kj] * kernel.n_i[kj * m + j];
                    }
                }
                kernel.k[i * m + j] = sum;
            }
        }
    }

    // Step 3: Recompute s_ni_nodes = S * N_i (for NR voltage damping).
    let n_nodes = kernel.n_nodes;
    for i in 0..n_nodes {
        for j in 0..m {
            let mut sum = 0.0;
            for k in 0..n {
                sum += kernel.s[i * n + k] * kernel.n_i[k * m + j];
            }
            s_ni_nodes[i * m + j] = sum;
        }
    }

    // Step 4: Stamp a_neg with delta_g_for_a_neg (relative to nominal).
    // A_neg = 2C/T - G. Increasing G by delta_g decreases A_neg by delta_g
    // at the pot's node entries (conductance stamp pattern).
    // The caller resets a_neg to nominal before each call.
    if np > 0 {
        kernel.a_neg[(np - 1) * n + (np - 1)] -= delta_g_for_a_neg;
    }
    if nq > 0 {
        kernel.a_neg[(nq - 1) * n + (nq - 1)] -= delta_g_for_a_neg;
    }
    if np > 0 && nq > 0 {
        kernel.a_neg[(np - 1) * n + (nq - 1)] += delta_g_for_a_neg;
        kernel.a_neg[(nq - 1) * n + (np - 1)] += delta_g_for_a_neg;
    }
}

/// Run the melange DK solver with per-sample pot modulation.
///
/// If `use_correct_backward_term` is true, the A_neg backward correction uses
/// the PREVIOUS timestep's pot resistance (correct trapezoidal discretization).
/// If false, it uses the CURRENT timestep's resistance (the bug).
fn run_melange_with_pot_modulation(
    netlist_str: &str,
    input_signal: &[f64],
    sample_rate: f64,
    pot_values: &[f64], // one R value per sample
    use_correct_backward_term: bool,
) -> Result<Vec<f64>, ValidationError> {
    use melange_solver::solver::CircuitSolver;

    let netlist = melange_solver::parser::Netlist::parse(netlist_str)
        .map_err(|e| ValidationError::Solver(format!("Parse error: {}", e.message)))?;

    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .map_err(|e| ValidationError::Solver(format!("MNA error: {}", e)))?;

    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    let output_node = mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);
    let input_conductance = 1.0;

    if input_node < mna.n {
        mna.g[input_node][input_node] += input_conductance;
    }

    let kernel = melange_solver::dk::DkKernel::from_mna(&mna, sample_rate)
        .map_err(|e| ValidationError::Solver(format!("DK kernel error: {:?}", e)))?;

    // Verify pot was parsed
    assert!(!kernel.pots.is_empty(), "No pots found in kernel");
    let pot = &kernel.pots[0];
    let g_nominal = pot.g_nominal;

    // Save nominal matrices for reset-and-reapply approach
    let s_nominal = kernel.s.clone();
    let a_neg_nominal = kernel.a_neg.clone();

    let devices = build_devices_from_netlist(&netlist, &mna)?;

    let mut solver = CircuitSolver::new(kernel, devices, input_node, output_node).unwrap();
    solver.set_input_conductance(input_conductance);

    // Initialize DC OP
    if mna.m > 0 {
        let device_slots = build_device_slots_from_netlist(&netlist);
        solver.initialize_dc_op(&mna, &device_slots);
    }

    let m = solver.m();
    let n_nodes = solver.n_nodes();

    // Pre-allocate s_ni_nodes buffer
    let mut s_ni_nodes = vec![0.0; n_nodes * m];

    let mut output = Vec::with_capacity(input_signal.len());
    let mut r_prev = 1.0 / g_nominal; // Start at nominal resistance

    for (sample_idx, &input) in input_signal.iter().enumerate() {
        let r_curr = pot_values[sample_idx];

        // Delta G from nominal for forward path (S, K) — always uses CURRENT R
        let delta_g_s = 1.0 / r_curr - g_nominal;

        // Delta G from nominal for backward path (A_neg):
        // CORRECT: uses PREVIOUS R (trapezoidal backward term)
        // BUGGY: uses CURRENT R (the error we're testing)
        let delta_g_a_neg = if use_correct_backward_term {
            1.0 / r_prev - g_nominal
        } else {
            1.0 / r_curr - g_nominal // BUG: uses current instead of previous
        };

        // Reset S and a_neg to nominal, then apply corrections
        solver.kernel_mut().s.copy_from_slice(&s_nominal);
        solver.kernel_mut().a_neg.copy_from_slice(&a_neg_nominal);

        apply_pot_to_kernel(
            solver.kernel_mut(),
            0,
            delta_g_s,
            delta_g_a_neg,
            &mut s_ni_nodes,
        );

        // Also update s_ni_nodes on the solver (it's a private field, so we
        // rely on the kernel's public S being correct for predict_into)
        // The NR loop uses kernel.k which we updated above.

        output.push(solver.process_sample(input));

        r_prev = r_curr;
    }

    Ok(output)
}

/// Test 9: Pot Modulation — Static Baseline
///
/// Validates a simple circuit with a fixed pot (R_ldr=10k) against ngspice.
/// This establishes that the basic circuit topology and device models work
/// correctly before testing time-varying pots.
///
/// Circuit: R1(1k) -> R_ldr(10k) -> C1(100n) || D1 || Rload(100k)
/// Input: 1kHz sine at 0.1V
#[test]
#[ignore] // requires ngspice
fn test_pot_modulation_static_baseline() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Pot Modulation: Static Baseline (R_ldr=10k fixed) ===");

    // Static ngspice netlist (fixed R_ldr, no B-source)
    let ngspice_netlist = r#"Pot Modulation Static Baseline
VIN in 0 DC 0 AC 1
R1 in mid 1k
R_ldr mid out 10k
C1 out 0 100n
D1 out 0 D1N4148
Rload out 0 100k
.MODEL D1N4148 D(IS=2.52e-9 N=1.752)
.TRAN 1u 10m
.PRINT TRAN V(out) V(in)
.END
"#;

    // Melange netlist (no VIN, with .pot)
    let melange_netlist = r#"Pot Modulation Static Baseline
R1 in mid 1k
R_ldr mid out 10k
C1 out 0 100n
D1 out 0 D1N4148
Rload out 0 100k
.MODEL D1N4148 D(IS=2.52e-9 N=1.752)
.pot R_ldr 1k 100k
"#;

    // Generate 10ms of 1kHz sine at 0.1V amplitude
    let duration = 0.01;
    let num_samples = (SAMPLE_RATE * duration) as usize;
    let input_signal: Vec<f64> = (0..num_samples)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            0.1 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin()
        })
        .collect();

    // Run ngspice with Thevenin PWL
    let tstep = 1.0 / SAMPLE_RATE;
    let pwl_data: Vec<(f64, f64)> = input_signal
        .iter()
        .enumerate()
        .map(|(i, &v)| (i as f64 / SAMPLE_RATE, v))
        .collect();

    let spice_data = run_transient_with_thevenin_pwl(
        ngspice_netlist,
        tstep,
        duration,
        "in",
        &pwl_data,
        1.0,
        &["out".to_string()],
    )
    .expect("ngspice failed");

    let mut spice_output = spice_data.get_node_voltage("out").unwrap().to_vec();
    dc_block_signal(&mut spice_output, SAMPLE_RATE);

    // Run melange with fixed pot at 10k (nominal value)
    let pot_values: Vec<f64> = vec![10_000.0; num_samples];
    let melange_output = run_melange_with_pot_modulation(
        melange_netlist,
        &input_signal,
        SAMPLE_RATE,
        &pot_values,
        true,
    )
    .expect("melange solver failed");

    // Trim to common length
    let len = spice_output.len().min(melange_output.len());
    let spice_signal = Signal::new(spice_output[..len].to_vec(), SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_output[..len].to_vec(), SAMPLE_RATE, "Melange");

    let config = nonlinear_config();
    let report = compare_signals(&spice_signal, &melange_signal, &config);

    println!("  Samples: {}", report.sample_count);
    println!("  RMS Error: {:.6e}", report.rms_error);
    println!(
        "  Normalized RMS: {:.4}%",
        report.normalized_rms_error * 100.0
    );
    println!("  Correlation: {:.8}", report.correlation_coefficient);

    assert!(
        report.correlation_coefficient > 0.99,
        "Static baseline should match ngspice closely (corr={:.6})",
        report.correlation_coefficient
    );
}

/// Test 10: Pot Modulation vs ngspice (time-varying resistance)
///
/// This is the main test for the trapezoidal backward-term bug.
///
/// The pot R_ldr is modulated sinusoidally at 5kHz:
///   R(t) = 5.5k + 4.5k * sin(2*pi*5000*t)
/// sweeping between 1k and 10k (~9.6 samples per modulation cycle at 48kHz).
///
/// The fast modulation rate causes large per-sample resistance changes,
/// making the A_neg backward-term error clearly visible. A small cap (1nF)
/// minimizes smoothing.
///
/// The test runs THREE simulations:
/// 1. ngspice with B-source (ground truth)
/// 2. melange with CORRECT backward term (delta_g_prev for A_neg)
/// 3. melange with BUGGY backward term (delta_g_current for A_neg)
///
/// The correct version achieves ~0.09% RMS vs SPICE, while the buggy
/// version gives ~1.5% RMS — a 17x degradation clearly attributable
/// to using current-timestep resistance in the backward term.
#[test]
#[ignore] // requires ngspice
fn test_pot_modulation_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Pot Modulation vs SPICE (5kHz LFO, 100ms) ===");

    let f_mod = 5000.0; // Fast modulation (5kHz) — ~9.6 samples/cycle at 48kHz
    let r_center = 5_500.0; // Center resistance (5.5k)
    let r_depth = 4_500.0; // Modulation depth (+/- 4.5k), sweeps 1k to 10k
    let f_signal = 1000.0; // Audio signal frequency (1kHz)
    let amplitude = 0.5; // Input amplitude (500mV) — enough to push diode into nonlinear
    let duration = 0.1; // 100ms — 500 modulation cycles

    let num_samples = (SAMPLE_RATE * duration) as usize;
    let tstep = 1.0 / SAMPLE_RATE;

    // Generate input signal: 1kHz sine at 0.1V
    let input_signal: Vec<f64> = (0..num_samples)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            amplitude * (2.0 * std::f64::consts::PI * f_signal * t).sin()
        })
        .collect();

    // Compute pot resistance for each sample
    let pot_values: Vec<f64> = (0..num_samples)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            pot_resistance_at(t, f_mod, r_center, r_depth)
        })
        .collect();

    // --- 1. Run ngspice with B-source for time-varying resistance ---
    // The B-source models a current-controlled time-varying resistor:
    //   B_ldr mid out I=V(mid,out) / R(t)
    // Small cap (1nF) minimizes smoothing so a_neg errors are visible.
    let ngspice_netlist = format!(
        "Pot Modulation Test\n\
         VIN in 0 DC 0 AC 1\n\
         R1 in mid 1k\n\
         B_ldr mid out I=V(mid,out)/({r_center}+{r_depth}*sin(2*3.14159265358979*{f_mod}*time))\n\
         C1 out 0 1n\n\
         D1 out 0 D1N4148\n\
         Rload out 0 100k\n\
         .MODEL D1N4148 D(IS=2.52e-9 N=1.752)\n\
         .TRAN 1u {duration}\n\
         .PRINT TRAN V(out) V(in)\n\
         .END\n",
    );

    let pwl_data: Vec<(f64, f64)> = input_signal
        .iter()
        .enumerate()
        .map(|(i, &v)| (i as f64 / SAMPLE_RATE, v))
        .collect();

    let spice_data = run_transient_with_thevenin_pwl(
        &ngspice_netlist,
        tstep,
        duration,
        "in",
        &pwl_data,
        1.0,
        &["out".to_string()],
    )
    .expect("ngspice failed for modulated pot");

    let mut spice_output = spice_data.get_node_voltage("out").unwrap().to_vec();
    dc_block_signal(&mut spice_output, SAMPLE_RATE);

    println!("  ngspice samples: {}", spice_output.len());

    // --- 2. Run melange with CORRECT backward term ---
    // Cap is 1nF to match ngspice netlist; pot range covers modulation sweep.
    let melange_netlist = r#"Pot Modulation Test
R1 in mid 1k
R_ldr mid out 5500
C1 out 0 1n
D1 out 0 D1N4148
Rload out 0 100k
.MODEL D1N4148 D(IS=2.52e-9 N=1.752)
.pot R_ldr 1k 100k
"#;

    // Pad pot_values to match output length if needed
    let len = spice_output.len().min(num_samples);
    let input_trimmed = &input_signal[..len];
    let pot_trimmed = &pot_values[..len];

    let melange_correct = run_melange_with_pot_modulation(
        melange_netlist,
        input_trimmed,
        SAMPLE_RATE,
        pot_trimmed,
        true,
    )
    .expect("melange solver failed (correct backward term)");

    // --- 3. Run melange with BUGGY backward term ---
    let melange_buggy = run_melange_with_pot_modulation(
        melange_netlist,
        input_trimmed,
        SAMPLE_RATE,
        pot_trimmed,
        false,
    )
    .expect("melange solver failed (buggy backward term)");

    // --- Compare all three ---
    let len = spice_output
        .len()
        .min(melange_correct.len())
        .min(melange_buggy.len());

    let spice_signal = Signal::new(spice_output[..len].to_vec(), SAMPLE_RATE, "SPICE");
    let correct_signal = Signal::new(
        melange_correct[..len].to_vec(),
        SAMPLE_RATE,
        "Melange_correct",
    );
    let buggy_signal = Signal::new(melange_buggy[..len].to_vec(), SAMPLE_RATE, "Melange_buggy");

    // Compare correct version vs SPICE
    let config_correct = ComparisonConfig {
        rms_error_tolerance: 0.30,
        peak_error_tolerance: 1.0,
        max_relative_tolerance: 10.0,
        correlation_min: 0.95,
        thd_error_tolerance_db: 5.0,
        full_scale: 1.0,
        skip_thd: true,
    };
    let report_correct = compare_signals(&spice_signal, &correct_signal, &config_correct);

    // Compare buggy version vs SPICE
    let report_buggy = compare_signals(&spice_signal, &buggy_signal, &config_correct);

    // Compare correct vs buggy directly
    let report_diff = compare_signals(&correct_signal, &buggy_signal, &config_correct);

    println!("\n  CORRECT backward term vs SPICE:");
    println!("    RMS Error: {:.6e}", report_correct.rms_error);
    println!(
        "    Normalized RMS: {:.4}%",
        report_correct.normalized_rms_error * 100.0
    );
    println!(
        "    Correlation: {:.8}",
        report_correct.correlation_coefficient
    );

    println!("\n  BUGGY backward term vs SPICE:");
    println!("    RMS Error: {:.6e}", report_buggy.rms_error);
    println!(
        "    Normalized RMS: {:.4}%",
        report_buggy.normalized_rms_error * 100.0
    );
    println!(
        "    Correlation: {:.8}",
        report_buggy.correlation_coefficient
    );

    println!("\n  CORRECT vs BUGGY (difference due to bug):");
    println!("    RMS Error: {:.6e}", report_diff.rms_error);
    println!(
        "    Normalized RMS: {:.4}%",
        report_diff.normalized_rms_error * 100.0
    );
    println!(
        "    Correlation: {:.8}",
        report_diff.correlation_coefficient
    );

    // The correct version should have better correlation than the buggy one
    // For a 5.63Hz tremolo with 20k depth, the error accumulates over each cycle.
    // Compute the improvement metric
    let corr_improvement =
        report_correct.correlation_coefficient - report_buggy.correlation_coefficient;
    let rms_improvement = report_buggy.normalized_rms_error - report_correct.normalized_rms_error;

    println!(
        "\n  Correlation improvement (correct - buggy): {:.6}",
        corr_improvement
    );
    println!(
        "  RMS improvement (buggy - correct): {:.6}",
        rms_improvement
    );

    // --- Assertions ---

    // 1. The correct version should pass our tolerances against SPICE
    assert!(
        report_correct.correlation_coefficient > 0.95,
        "Correct backward term should correlate well with SPICE (got {:.6})",
        report_correct.correlation_coefficient
    );

    // 2. The correct version should be STRICTLY better than the buggy version
    //    on at least one metric (correlation or RMS)
    assert!(
        report_correct.correlation_coefficient > report_buggy.correlation_coefficient
            || report_correct.normalized_rms_error < report_buggy.normalized_rms_error,
        "Correct backward term must outperform buggy version!\n\
         Correct: corr={:.8}, RMS={:.6e}\n\
         Buggy:   corr={:.8}, RMS={:.6e}",
        report_correct.correlation_coefficient,
        report_correct.normalized_rms_error,
        report_buggy.correlation_coefficient,
        report_buggy.normalized_rms_error,
    );

    // 3. The correct and buggy versions should be measurably different.
    //    The RMS difference between correct and buggy should exceed the
    //    noise floor (established by static baseline at ~0.05% RMS).
    //    A 2x noise floor margin (0.1%) ensures the difference is real.
    let rms_diff_between_versions = report_diff.normalized_rms_error;
    let noise_floor = 0.001; // 0.1% = 2x the static baseline noise
    assert!(
        rms_diff_between_versions > noise_floor,
        "Correct and buggy versions are indistinguishable.\n\
         RMS difference: {:.4}% (need > {:.4}% = 2x noise floor)\n\
         The test is not sensitive enough to detect the backward-term bug.",
        rms_diff_between_versions * 100.0,
        noise_floor * 100.0
    );

    println!(
        "\n  Test sensitivity: RMS diff {:.4}% is {:.1}x above noise floor {:.4}%",
        rms_diff_between_versions * 100.0,
        rms_diff_between_versions / noise_floor,
        noise_floor * 100.0
    );
}
