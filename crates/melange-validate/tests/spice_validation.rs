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

use melange_validate::{
    comparison::{compare_signals, ComparisonConfig, Signal},
    spice_runner::{is_ngspice_available, run_transient_with_thevenin_pwl},
    strip_vin_source, validate_circuit,
    visualizer::generate_html_report,
    ValidationError,
};

/// Sample rate used for all validation tests (48 kHz audio standard)
const SAMPLE_RATE: f64 = 48_000.0;

/// Atomic counter for unique temp file names in codegen compilation
static MELANGE_COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

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

    let melange_output = run_melange_codegen(&stripped_netlist, &input_signal, SAMPLE_RATE)?;

    // Apply DC blocking to SPICE output to match melange (codegen dc_block=false, so match raw)
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

/// Run melange via codegen: parse → MNA → DK kernel → route → generate code → compile → run.
///
/// This replaces all runtime solver functions (`run_melange_solver`,
/// `run_melange_nodal_solver`, `run_melange_nodal_solver_positive_k`).
/// The generated code is compiled to a temporary binary, input is piped via stdin,
/// and output is read from stdout.
fn run_melange_codegen(
    netlist_str: &str,
    input_signal: &[f64],
    sample_rate: f64,
) -> Result<Vec<f64>, ValidationError> {
    use melange_solver::codegen::routing;
    use melange_solver::codegen::{CodeGenerator, CodegenConfig};
    use melange_solver::dk::DkKernel;
    use std::io::Write;

    let netlist = melange_solver::parser::Netlist::parse(netlist_str)
        .map_err(|e| ValidationError::Solver(format!("Parse: {}", e.message)))?;

    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .map_err(|e| ValidationError::Solver(format!("MNA: {}", e)))?;

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

    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0; // G_in = 1.0 S
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
        DkKernel::from_mna_augmented(&mna, sample_rate)
            .map_err(|e| ValidationError::Solver(format!("Augmented DK: {:?}", e)))?
    } else {
        match DkKernel::from_mna(&mna, sample_rate) {
            Ok(k) => k,
            Err(_) => {
                dk_failed = true;
                DkKernel::from_mna_augmented(&mna, sample_rate)
                    .map_err(|e| ValidationError::Solver(format!("DK fallback: {:?}", e)))?
            }
        }
    };

    let decision = routing::auto_route(&kernel, &mna, dk_failed);
    let use_nodal = decision.route == routing::SolverRoute::Nodal;

    if use_nodal {
        let device_slots =
            melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(&netlist, Some(&mna))
                .unwrap_or_default();
        if !device_slots.is_empty() {
            mna.expand_bjt_internal_nodes(&device_slots);
        }
    }

    // Generate code — dc_block: true to match runtime solver's built-in DC blocker
    let config = CodegenConfig {
        circuit_name: "spice_val".to_string(),
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
    }
    .map_err(|e| ValidationError::Solver(format!("Codegen: {}", e)))?;

    // Append a main that reads input from stdin, processes, writes output to stdout
    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let stdin = std::io::stdin();
    let mut line = String::new();
    loop {
        line.clear();
        if stdin.read_line(&mut line).unwrap() == 0 { break; }
        if let Ok(input) = line.trim().parse::<f64>() {
            let out = process_sample(input, &mut state);
            println!("{:.15e}", out[0]);
        }
    }
}
"#;
    let full_source = format!("{}\n{}", generated.code, main_code);

    // Compile
    let tmp_dir = std::env::temp_dir();
    let counter = MELANGE_COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let pid = std::process::id();
    let src_path = tmp_dir.join(format!("melange_spval_{pid}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_spval_{pid}_{counter}"));

    std::fs::write(&src_path, &full_source)
        .map_err(|e| ValidationError::Solver(format!("Write source: {}", e)))?;

    let compile = std::process::Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2024")
        .arg("-O")
        .output()
        .map_err(|e| ValidationError::Solver(format!("rustc: {}", e)))?;

    let _ = std::fs::remove_file(&src_path);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        return Err(ValidationError::Solver(format!(
            "Compilation failed:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        )));
    }

    // Run: pipe input via stdin
    let stdin_data: String = input_signal
        .iter()
        .map(|s| format!("{s:.15e}\n"))
        .collect();

    let mut child = std::process::Command::new(&bin_path)
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| ValidationError::Solver(format!("Spawn: {}", e)))?;

    if let Some(mut stdin) = child.stdin.take() {
        stdin
            .write_all(stdin_data.as_bytes())
            .map_err(|e| ValidationError::Solver(format!("Stdin: {}", e)))?;
    }

    let output = child
        .wait_with_output()
        .map_err(|e| ValidationError::Solver(format!("Wait: {}", e)))?;

    let _ = std::fs::remove_file(&bin_path);

    if !output.status.success() {
        return Err(ValidationError::Solver(format!(
            "Binary failed:\n{}",
            String::from_utf8_lossy(&output.stderr)
        )));
    }

    let samples: Vec<f64> = String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|l| l.trim().parse().ok())
        .collect();

    Ok(samples)
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
            vg2k_frozen: 0.0,
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
                        tf: 0.0,
                        vje: 0.75,
                        mje: 0.33,
                        vjc: 0.75,
                        mjc: 0.33,
                        fc: 0.5,
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
            vg2k_frozen: 0.0,
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
                        rs: 0.0,
                    }),
                    has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
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
                        rs: 0.0,
                        gamma: 0.0,
                        phi: 0.6,
                        source_node: 0,
                        bulk_node: 0,
                    }),
                    has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
                });
                dim_offset += 2;
            }
            melange_solver::parser::Element::Triode { model, .. } => {
                slots.push(DeviceSlot {
                    device_type: DeviceType::Tube,
                    start_idx: dim_offset,
                    dimension: 2,
                    params: DeviceParams::Tube(TubeParams {
                        kind: melange_solver::device_types::TubeKind::SharpTriode,
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
                        kg2: 0.0,
                        alpha_s: 0.0,
                        a_factor: 0.0,
                        beta_factor: 0.0,
                        screen_form: melange_solver::device_types::ScreenForm::Rational,
                        mu_b: 0.0,
                        svar: 0.0,
                        ex_b: 0.0,
                    }),
                    has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
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
    let melange_output = run_melange_codegen(&stripped_netlist, &input_signal, SAMPLE_RATE)
        .expect("melange codegen failed");

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

    // --- Run melange (codegen, auto-routes DK vs nodal) ---
    let (stripped_netlist, _) = strip_vin_source(&netlist_str, "in");
    let melange_output = run_melange_codegen(&stripped_netlist, &input_signal, SAMPLE_RATE)
        .expect("melange codegen failed");

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


/// Neve 1073 Output Amplifier (BA283 AM) vs ngspice
///
/// Class A output stage: 3 BJTs (2× BC184C + 1× 2N3055), LO1166 output transformer.
/// CE input → Darlington CE → transformer. NFB via C4 through R1 to emitter.
/// Uses NodalSolver (augmented MNA for coupled inductors).
#[test]
#[ignore] // requires ngspice
fn test_neve_1073_output_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Neve 1073 Output Amplifier Validation ===");
    println!("Circuit: BA283 AM, 3 BJTs + LO1166 transformer, N=14, M=6");

    let data_dir = test_data_dir().join("neve_1073_output");
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
    // DC-block: output has large DC offset from Class A bias through transformer
    dc_block_signal(&mut spice_output, SAMPLE_RATE);
    let input_signal = resample_pwl_to_signal(&pwl_data, SAMPLE_RATE, spice_output.len());

    // --- Run melange (codegen, auto-routes to nodal for inductor circuit) ---
    let (stripped_netlist, _) = strip_vin_source(&netlist_str, "in");
    let melange_output = run_melange_codegen(&stripped_netlist, &input_signal, SAMPLE_RATE)
        .expect("melange codegen failed");

    // --- Compare ---
    let config = bjt_config();
    let spice_signal = Signal::new(spice_output.clone(), SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_output.clone(), SAMPLE_RATE, "Melange");
    let mut report = compare_signals(&spice_signal, &melange_signal, &config);
    report.circuit_name = "neve_1073_output".to_string();
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
        "    SPICE output PP: {:.4} V (gain: {:.1}x / {:.1} dB)",
        spice_pp,
        spice_gain,
        20.0 * spice_gain.log10()
    );
    println!(
        "    Melange output PP: {:.4} V (gain: {:.1}x / {:.1} dB)",
        melange_pp,
        melange_gain,
        20.0 * melange_gain.log10()
    );
}

/// Neve 1073 Preamp (BA283 AV) vs ngspice
///
/// Class A preamp: 3× BC184C. CE(TR4) → DC-coupled CE(TR5) → EF(TR6).
/// R11+R12 series collector load, R10 inter-stage DC feedback, R17 AC NFB.
/// Uses CircuitSolver (DK path, no inductors). N=14, M=6.
#[test]
#[ignore] // requires ngspice
fn test_neve_1073_preamp_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Neve 1073 Preamp (BA283 AV) Validation ===");
    println!("Circuit: 3× BC184C, CE→CE→EF, N=14, M=6");

    let data_dir = test_data_dir().join("neve_1073_preamp");
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
    // DC-block: output has DC offset through C15/R20
    dc_block_signal(&mut spice_output, SAMPLE_RATE);
    let input_signal = resample_pwl_to_signal(&pwl_data, SAMPLE_RATE, spice_output.len());

    // --- Run melange (codegen, auto-routes DK vs nodal) ---
    let (stripped_netlist, _) = strip_vin_source(&netlist_str, "in");
    let melange_output = run_melange_codegen(&stripped_netlist, &input_signal, SAMPLE_RATE)
        .expect("melange codegen failed");

    // DC-block melange output too
    let mut melange_dc_blocked = melange_output.clone();
    dc_block_signal(&mut melange_dc_blocked, SAMPLE_RATE);

    // --- Compare ---
    let config = bjt_config();
    let spice_signal = Signal::new(spice_output.clone(), SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_dc_blocked.clone(), SAMPLE_RATE, "Melange");
    let mut report = compare_signals(&spice_signal, &melange_signal, &config);
    report.circuit_name = "neve_1073_preamp".to_string();
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
    let melange_pp = melange_dc_blocked
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        - melange_dc_blocked
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
    let spice_pp = spice_output
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        - spice_output.iter().cloned().fold(f64::INFINITY, f64::min);

    let spice_gain = spice_pp / input_pp;
    let melange_gain = melange_pp / input_pp;

    println!("    Input PP: {:.4} V", input_pp);
    println!(
        "    SPICE output PP: {:.4} V (gain: {:.1}x / {:.1} dB)",
        spice_pp,
        spice_gain,
        20.0 * spice_gain.log10()
    );
    println!(
        "    Melange output PP: {:.4} V (gain: {:.1}x / {:.1} dB)",
        melange_pp,
        melange_gain,
        20.0 * melange_gain.log10()
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




/// Test: Tube Screamer TS808 (Op-amp + Diode Clipping)
///
/// Tests the classic TS808 inverting op-amp with antiparallel diode
/// feedback clipping. Op-amp modeled as VCCS + Rout (same model in
/// both ngspice and melange). 0.5V input drives diodes into soft
/// clipping at ~0.6V.
///
/// Circuit: JRC4558 op-amp (AOL=200000, ROUT=75), 2x 1N4148,
///          51k drive/feedback, RC tone stack, output coupling
/// Input: 1kHz sine, 0.5V amplitude
/// Expected: Soft-clipped output ~0.17V peak
#[test]
#[ignore] // requires ngspice
fn test_tube_screamer_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Tube Screamer TS808 Validation ===");
    println!("Circuit: Op-amp inverting + antiparallel diode clipping");

    let result = run_validation("tube_screamer", "out", &nonlinear_config())
        .expect("Failed to run validation");

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "Tube Screamer validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );

    assert!(
        result.report.correlation_coefficient > 0.99,
        "Correlation too low: {:.8}",
        result.report.correlation_coefficient
    );
}

#[test]
#[ignore] // Requires ngspice
fn test_tube_screamer_wiper_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Tube Screamer TS808 (Wiper Volume) Validation ===");
    println!("Circuit: Same clipping + tone as original, with 100K wiper volume at pos=0.85");

    // Volume pot attenuation (pos=0.85 → 15% loss) pushes zero-crossings closer to zero,
    // where relative error spikes. Relax max_relative_tolerance vs base nonlinear_config.
    let config = ComparisonConfig {
        max_relative_tolerance: 5000.0, // near-zero relative error from volume divider
        ..nonlinear_config()
    };

    let result = run_validation("tube_screamer_wiper", "out", &config)
        .expect("Failed to run validation");

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "Tube Screamer Wiper validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );

    // 5 nines correlation — better than required 2 nines
    assert!(
        result.report.correlation_coefficient > 0.9999,
        "Correlation too low: {:.8}",
        result.report.correlation_coefficient
    );
}

// =============================================================================
// Klon Centaur Validation
// =============================================================================


/// Klon Centaur (Gold Edition) vs ngspice
///
/// 4 op-amp sections (2× TL072), 2× 1N34A germanium diodes, 3 pots.
/// Shunt diode topology → positive K diagonal → uses NodalSolver (full N×N LU).
#[test]
#[ignore] // requires ngspice
fn test_klon_centaur_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Klon Centaur Validation ===");
    println!("Circuit: 4 op-amps + 2 Ge diodes, N≈40, M=2 (nodal full LU)");

    let data_dir = test_data_dir().join("klon_centaur");
    let netlist_path = data_dir.join("circuit.cir");
    let input_pwl_path = data_dir.join("input_pwl.txt");

    let netlist_str = std::fs::read_to_string(&netlist_path).expect("read netlist");
    let pwl_data = load_pwl_file(&input_pwl_path).expect("read PWL");
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
    dc_block_signal(&mut spice_output, SAMPLE_RATE);
    let input_signal = resample_pwl_to_signal(&pwl_data, SAMPLE_RATE, spice_output.len());

    // --- Run melange (codegen, auto-routes to nodal for positive K / shunt diodes) ---
    let (stripped_netlist, _) = strip_vin_source(&netlist_str, "in");
    let melange_output = run_melange_codegen(&stripped_netlist, &input_signal, SAMPLE_RATE)
        .expect("melange codegen failed");

    // --- Compare ---
    let spice_signal = Signal::new(spice_output, SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_output, SAMPLE_RATE, "Melange");

    // Op-amp + germanium diode circuit. At 100mV input, diodes are barely active
    // (Ge Vf ~ 300mV). THD difference is large in absolute dB because both are very
    // low distortion at this level — skip THD comparison.
    let config = ComparisonConfig {
        max_relative_tolerance: 5000.0, // near-zero relative error from volume divider
        skip_thd: true,
        ..nonlinear_config()
    };

    let mut report = compare_signals(&spice_signal, &melange_signal, &config);
    report.circuit_name = "klon_centaur".to_string();
    report.node_name = "out".to_string();

    if !report.passed {
        let path = data_dir.join("klon_centaur_failure_report.html");
        let _ = generate_html_report(&report, &spice_signal, &melange_signal, &path);
        println!("Failure report: {}", path.display());
    }

    print_validation_metrics(&ValidationResult {
        report: report.clone(),
        html_report_path: None,
    });

    assert!(
        report.passed,
        "Klon Centaur validation failed:\n{}",
        report.summary()
    );

    // Nearly 4 nines correlation — well above the required 2 nines
    assert!(
        report.correlation_coefficient > 0.999,
        "Correlation too low: {:.8}",
        report.correlation_coefficient
    );
}
