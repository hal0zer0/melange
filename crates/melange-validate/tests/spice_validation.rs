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
//! | `test_wurli_preamp_vs_spice` | Wurli 200A preamp | 2 BJTs + 1 diode | Wurli |
//! | `test_neve_1073_output_vs_spice` | Neve BA283 AM | 3 BJTs + transformer | Neve output |
//! | `test_neve_1073_preamp_vs_spice` | Neve BA283 AV | 3 BJTs | Neve preamp |
//! | `test_pot_static_offnominal_vs_spice` | Off-nominal `.pot` | 1 diode | Nonlinear |
//! | `test_pot_modulation_vs_spice` | Audio-rate `.pot` vs B-source | 1 diode + R(t) | Custom |

use std::path::PathBuf;

use melange_validate::{
    comparison::{compare_signals, ComparisonConfig, Signal},
    dc_block_signal,
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
        settle_time_s: 0.0,
    }
}

/// Configuration for nonlinear circuit tolerances
///
/// Nonlinear circuits have more tolerance for error due to:
/// - Device model differences (melange doesn't model RS, CJO, TT)
/// - Newton-Raphson convergence differences
/// - Numerical precision in exponential functions
///
/// Measured 2026-07-18 (HEAD b421358, ngspice-42, reltol=1e-4 reference)
/// across the direct users of this config:
///   diode_clipper        rms 0.069%  peak 1.6e-3 V  corr 0.99999991  THD err 0.00 dB
///   antiparallel_diodes  rms 0.059%  peak 1.2e-3 V  corr 0.99999983  THD err 0.03 dB
///   tube_screamer        rms 0.442%  peak 9.9e-3 V  corr 0.99999032  THD err 0.04 dB
///   diode silence→signal rms 0.125%  peak 1.43e-2 V corr 0.99999934  THD err <0.1 dB
/// Gates below sit ~3.5-10x above the worst measured user per metric
/// (rms: tube_screamer; peak: silence→signal onset; corr: tube_screamer).
/// tube_screamer_wiper overrides rms/corr with its own cited numbers.
fn nonlinear_config() -> ComparisonConfig {
    ComparisonConfig {
        rms_error_tolerance: 0.02, // was 0.20; worst measured 0.442% → 4.5x headroom
        peak_error_tolerance: 0.05, // was 0.5 V; worst measured 1.43e-2 V → 3.5x headroom
        max_relative_tolerance: 5.0, // 500% — near zero-crossings, large relative error expected
        correlation_min: 0.9999,   // was 0.99; worst measured 1-corr = 9.7e-6 → 10.3x headroom
        thd_error_tolerance_db: 1.0, // was 3.0; worst measured 0.04 dB → 25x headroom
        full_scale: 5.0,           // Diode clippers can hit 5V
        skip_thd: false,
        settle_time_s: 0.0,
    }
}

/// BJT common-emitter tolerances (test_bjt_common_emitter_vs_spice).
///
/// BJT circuits have larger error vs SPICE because:
/// - Gummel-Poon implementation differences (melange vs ngspice)
/// - small gain mismatch (measured 213.2x melange vs 208.3x ngspice, ratio 1.024)
///
/// Measured 2026-07-18 (HEAD b421358, ngspice-42, reltol=1e-4 reference,
/// 3 ms settle window): rms 4.26%, peak 0.164 V, corr 0.99964880.
/// The old peak gate (0.5 V) was documented as widened for a startup
/// transient; today's measurement shows the settled peak (0.164 V) is
/// steady-state model mismatch (gain ratio 1.024 on a ~4.2 V p-p output),
/// not startup — the settle window removes the startup contribution and
/// the peak gate is tightened to 0.30 V (1.8x over the settled peak).
fn bjt_config() -> ComparisonConfig {
    ComparisonConfig {
        rms_error_tolerance: 0.10,     // was 0.15; measured 4.26% → 2.3x headroom
        peak_error_tolerance: 0.30,    // was 0.5 V; measured settled peak 0.164 V → 1.8x
        max_relative_tolerance: 100.0, // near zero-crossings
        correlation_min: 0.995,        // was 0.99; measured 1-corr = 3.5e-4 → 14x headroom
        thd_error_tolerance_db: 5.0,   // model differences in distortion
        full_scale: 10.0,              // BJT CE output can swing wider
        skip_thd: true,                // THD comparison not meaningful for nonlinear BJT
        // 3 ms settle on the 10 ms signal: excludes the DC-blocker/DC-OP
        // startup region while keeping 70% of the window. (The 5 Hz blocker
        // tau is ~32 ms, but both sides use first-sample/DC-OP-seeded
        // blockers, so only a short residual remains.)
        settle_time_s: 0.003,
    }
}

/// Wurlitzer 200A preamp tolerances (test_wurli_preamp_vs_spice).
///
/// Split from bjt_config 2026-07-18 — the wurli circuit measures far tighter
/// than the BJT CE test, so sharing the CE gates hid ~17x of margin.
/// Measured 2026-07-18 (HEAD b421358, ngspice-42, reltol=1e-4 reference,
/// 10 ms settle on the 50 ms signal): rms 0.235%, peak 1.28e-3 V,
/// corr 0.99999734, gain ratio 1.0006.
fn wurli_config() -> ComparisonConfig {
    ComparisonConfig {
        rms_error_tolerance: 0.01,     // was 0.15; measured 0.235% → 4.3x headroom
        peak_error_tolerance: 0.01,    // was 0.5 V; measured settled peak 1.28 mV → 7.8x headroom
        max_relative_tolerance: 100.0, // near zero-crossings
        correlation_min: 0.9999,       // was 0.99; measured 1-corr = 2.7e-6 → 37x headroom
        thd_error_tolerance_db: 5.0,
        full_scale: 10.0,
        skip_thd: true,
        // 10 ms settle on a 50 ms signal: comfortably excludes the residual
        // startup region while keeping 80% of the window.
        settle_time_s: 0.010,
    }
}

/// Neve 1073 output amp (BA283 AM) tolerances (test_neve_1073_output_vs_spice).
///
/// Armed 2026-07-18 — this test previously ran everything and asserted
/// nothing. STATUS.md recorded corr 0.9961 / rms 14.4% ("marginal") from
/// 2026-04-08; measured today (HEAD b421358, ngspice-42, reltol=1e-4,
/// corrected single-VIN deck, 10 ms settle on the 50 ms signal):
/// rms 0.107%, peak 1.06e-4 V, corr 0.99999952, gain 6.7x with
/// gain ratio 1.0000 — the emitter-NR campaign (and interim BJT/transformer
/// fixes since April) fixed the marginal correlation.
///
/// Corrected drive level: the deck previously baked in a `VIN in_src` +
/// `R_src 1` Thevenin pair that escaped both the harness strip and the
/// Thevenin inject (n+ was "in_src", not "in"), leaving a second 1-ohm
/// shunt at the input node on both sides. Removing it doubled the drive:
/// measured gain went 3.4x → 6.7x (16.5 dB) at identical correlation.
fn neve_output_config() -> ComparisonConfig {
    ComparisonConfig {
        rms_error_tolerance: 0.01,     // measured 0.107% → 9.3x headroom
        peak_error_tolerance: 0.005,   // measured 1.06e-4 V → 47x headroom
        max_relative_tolerance: 100.0, // near zero-crossings
        correlation_min: 0.9999,       // measured 1-corr = 4.8e-7 → 208x headroom
        thd_error_tolerance_db: 5.0,
        full_scale: 10.0,
        skip_thd: true,
        settle_time_s: 0.010,
    }
}

/// Neve 1073 preamp (BA283 AV) tolerances (test_neve_1073_preamp_vs_spice).
///
/// Armed 2026-07-18 — this test previously ran everything and asserted
/// nothing. Historical record: corr 0.99999 / rms 0.53% (STATUS "5-nines").
/// Measured today (HEAD b421358, ngspice-42, reltol=1e-4, 64 ms settle —
/// 2x the 5 Hz blocker tau of ~32 ms — on the 500 ms signal):
/// rms 0.0346%, peak 1.12e-4 V, corr 1.00000000 (printed at 8 decimals),
/// gain 26.0x with gain ratio 0.9996.
///
/// Most of the historical 0.53% rms was a harness artifact: the melange
/// output was DC-blocked TWICE (once inside the generated code, once in
/// the test) while the SPICE side was blocked once. Removing the second
/// application dropped rms 0.508% → 0.0346% and corr 0.99998710 → ~1.0.
fn neve_preamp_config() -> ComparisonConfig {
    ComparisonConfig {
        rms_error_tolerance: 0.005,    // measured 0.0346% → 14x headroom
        peak_error_tolerance: 0.002,   // measured 1.12e-4 V → 18x headroom
        max_relative_tolerance: 100.0, // near zero-crossings
        correlation_min: 0.99999,      // measured 1-corr < 5e-9 → >2000x headroom
        thd_error_tolerance_db: 5.0,
        full_scale: 10.0,
        skip_thd: true,
        settle_time_s: 0.064,
    }
}

// NOTE: `dc_block_signal` is imported from the melange_validate library —
// there is intentionally NO test-local copy. The library implementation
// seeds `x_prev` from the signal's first sample (mirroring the generated
// code's DC-OP-seeded blocker); see its doc comment in `src/lib.rs`.

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
        .map_err(ValidationError::Spice)?
        .to_vec();

    // Strip VIN for melange solver (auto-detect and remove input voltage source)
    let (stripped_netlist, _dc_offset) = strip_vin_source(&netlist_str, "in");
    let input_signal = resample_pwl_to_signal(&pwl_data, SAMPLE_RATE, spice_output.len());

    let melange_output = run_melange_codegen(&stripped_netlist, &input_signal, SAMPLE_RATE)?;

    // Apply DC blocking to SPICE output to match melange's built-in blocker
    // (run_melange_codegen generates with dc_block: true, so the melange
    // output is already 5 Hz-blocked; the reference must get the same filter).
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
    // Standard main: read a sample per stdin line, process, print.
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
    run_melange_codegen_with_main(netlist_str, input_signal, sample_rate, main_code)
}

/// Like [`run_melange_codegen`], but with a caller-supplied `fn main()` body
/// appended to the generated code. Used by the pot-modulation tests to drive
/// `set_pot_0(...)` per sample (or once, off-nominal) — something the standard
/// stdin/stdout main has no hook for.
fn run_melange_codegen_with_main(
    netlist_str: &str,
    input_signal: &[f64],
    sample_rate: f64,
    main_code: &str,
) -> Result<Vec<f64>, ValidationError> {
    use melange_solver::codegen::routing;
    use melange_solver::codegen::{CodeGenerator, CodegenConfig};
    use melange_solver::dk::DkKernel;
    use std::io::Write;

    let netlist = melange_solver::parser::Netlist::parse(netlist_str)
        .map_err(|e| ValidationError::Solver(format!("Parse: {}", e.message)))?;

    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .map_err(|e| ValidationError::Solver(format!("MNA: {}", e)))?;

    // Hard-error on missing nodes (like the library twin in src/lib.rs).
    // The old `.unwrap_or(1)` / `.unwrap_or(2)` fallback silently compared
    // against an arbitrary node when a deck renamed in/out — a wrong-node
    // comparison must fail loudly, not produce plausible garbage.
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .ok_or_else(|| {
            ValidationError::Solver(format!(
                "Input node 'in' not found. Available: {:?}",
                mna.node_map.keys().collect::<Vec<_>>()
            ))
        })?
        .saturating_sub(1);
    let output_node = mna
        .node_map
        .get("out")
        .copied()
        .ok_or_else(|| {
            ValidationError::Solver(format!(
                "Output node 'out' not found. Available: {:?}",
                mna.node_map.keys().collect::<Vec<_>>()
            ))
        })?
        .saturating_sub(1);

    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0; // G_in = 1.0 S
    }

    // Stamp junction caps + pre-solve DC OP so BJT charge-storage caps are
    // linearized at the real operating point. Uses the IR's
    // `build_device_info_with_mna` rather than the harness's
    // `build_device_slots_from_netlist` stub — the latter hardcoded
    // CJE/CJC/TF to zero regardless of the `.model` card and would defeat
    // the re-linearization. When all BJTs use the SPICE defaults this is
    // byte-identical to the zero-bias stamp.
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
        let device_slots = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
            &netlist,
            Some(&mna),
        )
        .unwrap_or_default();
        if !device_slots.is_empty() {
            mna.expand_bjt_internal_nodes(&device_slots);
        }
    }

    // Post-DC-block output ceiling (CodegenConfig::output_clamp_v, default 10 V
    // per docs/aidocs/SIGNAL_LEVELS.md's "Signal Level Contract"). The default
    // is sized for line-level circuits; a circuit whose DC operating point
    // carries a high-voltage rail (e.g. a 250 V tube B+) can legitimately swing
    // its output node tens of volts under normal large-signal drive, and a
    // fixed 10 V ceiling silently hard-clips that into a square wave — which
    // then reads as a huge melange-vs-ngspice divergence that is actually a
    // harness/config gap, not a solver bug (see triode_cc overdrive
    // investigation). Auto-scale the ceiling from the DC operating point's
    // node-voltage headroom (already computed above as `dc_preflight`) so any
    // future high-rail circuit validated through this harness gets a ceiling
    // that won't clip a legitimate large-signal swing; never lower it below
    // the existing 10 V default so all line-level circuits keep their
    // historical clamp behavior byte-for-byte.
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

    // Generate code — dc_block: true to match runtime solver's built-in DC blocker
    let config = CodegenConfig {
        circuit_name: "spice_val".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        dc_block: true,
        output_clamp_v: auto_clamp_v,
        ..CodegenConfig::default()
    };
    let generator = CodeGenerator::new(config);
    let generated = if use_nodal {
        generator.generate_nodal(&mna, &netlist)
    } else {
        generator.generate_with_dc_op(&kernel, &mna, &netlist, dc_preflight)
    }
    .map_err(|e| ValidationError::Solver(format!("Codegen: {}", e)))?;

    // Append the caller-supplied main (stdin samples in, stdout samples out)
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

    // Run: pipe input via stdin.
    let stdin_data: Vec<u8> = input_signal
        .iter()
        .map(|s| format!("{s:.15e}\n"))
        .collect::<String>()
        .into_bytes();

    let mut child = std::process::Command::new(&bin_path)
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

    let output = child
        .wait_with_output()
        .map_err(|e| ValidationError::Solver(format!("Wait: {}", e)))?;
    let _ = writer.join();

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

    // Measured 2026-07-18 (HEAD b421358, ngspice-42, reltol=1e-4):
    // rms 3.47%, peak 4.6e-6 V, corr 0.99940687, THD err 4.70 dB.
    // rms margin is only 2.9x — left at 0.10 (tightening would leave <10x).
    // Peak tightened 0.5 → 1e-4 (measured 4.6e-6 → 22x headroom; the output
    // signal is micro-volt scale, so a 0.5 V peak gate was vacuous).
    let config = ComparisonConfig {
        rms_error_tolerance: 0.10,
        peak_error_tolerance: 1e-4,
        max_relative_tolerance: 5.0,
        correlation_min: 0.999,
        thd_error_tolerance_db: 5.0, // DK method produces different harmonics than SPICE
        full_scale: 5.0,
        skip_thd: false,
        settle_time_s: 0.0,
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

    // Measured 2026-07-18 (HEAD b421358, ngspice-42, reltol=1e-4):
    // rms 0.0287%, peak 2.7e-6 V, corr 0.99999997.
    // rms tightened 0.10 → 0.005 (17x headroom); peak 0.5 → 1e-4 (37x —
    // micro-volt-scale output made the 0.5 V gate vacuous); corr 0.999 →
    // 0.9999 (measured 1-corr = 3e-8 → >1000x headroom even after tightening).
    let config = ComparisonConfig {
        rms_error_tolerance: 0.005,
        peak_error_tolerance: 1e-4,
        max_relative_tolerance: 5.0,
        correlation_min: 0.9999,
        thd_error_tolerance_db: 5.0,
        full_scale: 5.0,
        skip_thd: true, // small-signal linear region: THD too low to measure reliably
        settle_time_s: 0.0,
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
    let config = wurli_config();
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
    // Tightened 2026-07-18 from [0.25..4.0] to [0.8..1.25]:
    // measured gain ratio 1.0006 (melange 2.4x vs SPICE 2.4x).
    let gain_ratio = melange_gain / spice_gain;
    assert!(
        gain_ratio > 0.8 && gain_ratio < 1.25,
        "Melange gain ({:.1}x) should be within [0.8..1.25] of SPICE gain ({:.1}x), ratio={:.4}",
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
    let config = neve_output_config();
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

    // --- Gates (armed 2026-07-18; this test previously asserted nothing) ---
    // Measured: corr 0.99999952, rms 0.107%, gain ratio 1.0000 — see
    // neve_output_config() for the full measurement citation.
    assert!(
        result.report.passed,
        "Neve 1073 output validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );
    let gain_ratio = melange_gain / spice_gain;
    assert!(
        gain_ratio > 0.8 && gain_ratio < 1.25,
        "Melange gain ({:.2}x) should be within [0.8..1.25] of SPICE gain ({:.2}x), ratio={:.4} \
         (measured ratio 1.0002 on 2026-07-18)",
        melange_gain,
        spice_gain,
        gain_ratio
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

    // NOTE: melange_output is NOT filtered again here — run_melange_codegen
    // generates with dc_block: true, so the generated binary already applied
    // the 5 Hz blocker. A second application (removed 2026-07-18) rolled off
    // the melange side twice while the SPICE side was blocked once,
    // introducing an asymmetric LF error that was a harness artifact.

    // --- Compare ---
    let config = neve_preamp_config();
    let spice_signal = Signal::new(spice_output.clone(), SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_output.clone(), SAMPLE_RATE, "Melange");
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

    // --- Gates (armed 2026-07-18; this test previously asserted nothing) ---
    // Measured: corr 1.00000000, rms 0.0346%, gain ratio 0.9996 — see
    // neve_preamp_config() for the full measurement citation.
    assert!(
        result.report.passed,
        "Neve 1073 preamp validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );
    let gain_ratio = melange_gain / spice_gain;
    assert!(
        gain_ratio > 0.8 && gain_ratio < 1.25,
        "Melange gain ({:.2}x) should be within [0.8..1.25] of SPICE gain ({:.2}x), ratio={:.4} \
         (measured ratio 0.9996 on 2026-07-18)",
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
    let mut input: Vec<f64> = (0..num_samples)
        .map(|i| {
            if (i % period_samples) < period_samples / 2 {
                1.0
            } else {
                -1.0
            }
        })
        .collect();
    // Start the input at 0 so both engines see the same onset. ngspice
    // pre-settles its DC operating point at PWL(t=0): a square wave whose
    // first sample is +1.0 starts ngspice pre-charged at +1 V while melange
    // starts from silence and genuinely sees the 0->+1 onset step. That
    // asymmetry puts a 5 Hz-blocker droop (r^n, RMS ~0.4 over 100 ms) in the
    // melange output with no counterpart in the reference. (It was
    // historically masked by a zero-seeded reference-side DC blocker that
    // injected the same r^n term by accident; the unified first-sample-seeded
    // blocker exposed it.) With input[0] = 0, both engines settle at 0 V and
    // both see the identical step one sample later.
    input[0] = 0.0;

    let netlist_path = test_data_dir().join("rc_lowpass").join("circuit.cir");
    // Measured 2026-07-18 (after the input[0] = 0 onset fix and reltol=1e-4):
    // rms 0.132%, peak 6.9e-3 V, corr 0.99999924. The historical 5% / 1 V
    // gates were covering the onset-asymmetry artifact (rms was 2.24e-2),
    // not transition error — with matched onsets the bilinear step response
    // tracks ngspice to millivolts.
    let config = ComparisonConfig {
        rms_error_tolerance: 0.01,   // was 0.05; measured 0.132% → 7.6x headroom
        peak_error_tolerance: 0.05,  // was 1.0 V; measured 6.9e-3 V → 7.3x headroom
        max_relative_tolerance: 1e4, // near zero-crossings
        correlation_min: 0.9999,     // was 0.999; measured 1-corr = 7.6e-7 → 131x headroom
        thd_error_tolerance_db: 5.0,
        full_scale: 1.0,
        skip_thd: true, // square wave THD is not meaningful
        settle_time_s: 0.0,
    };

    let result = validate_circuit(&netlist_path, &input, SAMPLE_RATE, "out", &config)
        .expect("Square wave validation failed");

    println!("  Samples: {}", result.report.sample_count);
    println!("  RMS Error: {:.6e}", result.report.rms_error);
    println!(
        "  Normalized RMS: {:.6} ({:.4}%)",
        result.report.normalized_rms_error,
        result.report.normalized_rms_error * 100.0
    );
    println!("  Peak Error: {:.6e}", result.report.peak_error);
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
        settle_time_s: 0.0,
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
        "  Normalized RMS: {:.6} ({:.4}%)",
        result.report.normalized_rms_error,
        result.report.normalized_rms_error * 100.0
    );
    println!("  Peak Error: {:.6e}", result.report.peak_error);
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
        "  Normalized RMS: {:.6} ({:.4}%)",
        result.report.normalized_rms_error,
        result.report.normalized_rms_error * 100.0
    );
    println!("  Peak Error: {:.6e}", result.report.peak_error);
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
    //
    // rms/corr overrides vs the (tightened) base nonlinear_config, measured
    // 2026-07-18: rms 5.71%, corr 0.99838696. The wiper variant's divider +
    // deliberately simplified tone network give it a genuinely larger
    // time-domain offset than the clipping-only tube_screamer (see the
    // correlation assert comment below); the THD match (0.11 dB) is the
    // sonically meaningful gate. rms 0.10 = 1.75x over measured (was
    // effectively 0.20 before the base config tightening — this is still a
    // tightening, not a widening).
    let config = ComparisonConfig {
        max_relative_tolerance: 5000.0, // near-zero relative error from volume divider
        rms_error_tolerance: 0.10,      // measured 5.71% → 1.75x headroom
        correlation_min: 0.997,         // measured 0.99838696; matches the assert below
        ..nonlinear_config()
    };

    let result =
        run_validation("tube_screamer_wiper", "out", &config).expect("Failed to run validation");

    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "Tube Screamer Wiper validation failed:\n{}\nReport saved to: {:?}",
        result.report.summary(),
        result.html_report_path
    );

    // 3 nines correlation. The 5-nines figure in ts808 validation history is for
    // the clipping-stage-only `tube_screamer`; this wiper variant adds a volume
    // divider + a deliberately simplified tone network, so its time-domain match
    // is ~0.998 (vs the clipping stage's 0.99999). The harmonic content — what the
    // pedal actually sounds like — still matches ngspice to 0.11 dB THD, so the
    // sound is validated; the lower correlation is the divider/tone offset, not a
    // clipping error. Threshold set below the genuine ~0.9984 with margin.
    assert!(
        result.report.correlation_coefficient > 0.997,
        "Correlation too low: {:.8}",
        result.report.correlation_coefficient
    );
}

// =============================================================================
// Dynamic Potentiometer Validation (tests/data/pot_modulation)
// =============================================================================

/// Shared harness for the pot_modulation deck pair.
///
/// `ngspice_deck` is run through the standard Thevenin-PWL path;
/// `circuit_melange.cir` (no VIN, `.pot R_ldr 1k 100k`, nominal 5.5k) is run
/// through codegen with a caller-supplied main so the test can drive
/// `set_pot_0(...)`.
fn run_pot_validation(
    ngspice_deck: &str,
    input_signal: &[f64],
    main_code: &str,
) -> (Vec<f64>, Vec<f64>) {
    let data_dir = test_data_dir().join("pot_modulation");
    let ngspice_netlist = std::fs::read_to_string(data_dir.join(ngspice_deck))
        .expect("Failed to read ngspice pot deck");
    let melange_netlist = std::fs::read_to_string(data_dir.join("circuit_melange.cir"))
        .expect("Failed to read melange pot deck");

    let duration = input_signal.len() as f64 / SAMPLE_RATE;
    let tstep = 1.0 / SAMPLE_RATE;
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
    .expect("ngspice failed on pot deck");

    let mut spice_output = spice_data.get_node_voltage("out").unwrap().to_vec();
    dc_block_signal(&mut spice_output, SAMPLE_RATE);

    // circuit_melange.cir carries no VIN; strip is a no-op but kept for symmetry
    let (stripped, _) = strip_vin_source(&melange_netlist, "in");
    let melange_output =
        run_melange_codegen_with_main(&stripped, input_signal, SAMPLE_RATE, main_code)
            .expect("melange codegen failed on pot deck");

    (spice_output, melange_output)
}

/// Static off-nominal pot position vs ngspice.
///
/// First validation of a NON-nominal pot position through the `.pot`
/// mechanism: the melange deck's R_ldr is nominally 5.5k, and the test moves
/// it to 10k via `set_pot_0(10_000.0)` before the first sample (triggering
/// the lazy matrix rebuild path), then compares against ngspice running the
/// same topology with a fixed 10k resistor (`circuit_static.cir`).
/// This isolates the pot re-stamp/rebuild machinery from any R(t)
/// zero-order-hold considerations — if this is tight and the dynamic test
/// below is loose, the looseness is R(t) discretization, not the mechanism.
#[test]
#[ignore] // requires ngspice
fn test_pot_static_offnominal_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Pot Static Off-Nominal (R_ldr 5.5k -> 10k) Validation ===");

    // 500 Hz, 1 V, 20 ms — clips positive half on D1, divider on negative half
    let num_samples = (SAMPLE_RATE * 0.020) as usize;
    let input: Vec<f64> = (0..num_samples)
        .map(|i| (2.0 * std::f64::consts::PI * 500.0 * i as f64 / SAMPLE_RATE).sin())
        .collect();

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    // Off-nominal pot position: nominal is 5.5k, move to 10k before the
    // first sample. Rebuild happens lazily inside process_sample().
    state.set_pot_0(10_000.0);
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

    let (spice_output, melange_output) =
        run_pot_validation("circuit_static.cir", &input, main_code);

    // Measured 2026-07-18 (first arming): rms 0.0374%, peak 4.06e-3 V,
    // corr 0.99999995, THD err 0.01 dB — the off-nominal rebuild is exactly
    // as tight as the nominal-position diode tests, so nonlinear_config
    // (rms 2%, peak 0.05 V, corr 0.9999, THD 1 dB) applies unchanged.
    let config = nonlinear_config();
    let spice_signal = Signal::new(spice_output, SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_output, SAMPLE_RATE, "Melange");
    let mut report = compare_signals(&spice_signal, &melange_signal, &config);
    report.circuit_name = "pot_static_offnominal".to_string();
    report.node_name = "out".to_string();

    let result = ValidationResult {
        report,
        html_report_path: None,
    };
    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "Pot static off-nominal validation failed:\n{}",
        result.report.summary()
    );
}

/// Audio-rate pot modulation vs a native ngspice B-source reference.
///
/// The ngspice side (`circuit.cir`) models the time-varying resistance with
/// a behavioral current source — `I = V(mid,out) / (5500 + 4500*sin(2*pi*
/// 5000*time))` — an INDEPENDENT reference, not a melange mirror: ngspice
/// evaluates R(t) continuously inside its own adaptive integration. The
/// melange side drives the same law through the `.pot` mechanism with one
/// `set_pot_0()` call per sample (full per-sample O(N^3) rebuild path).
///
/// R(t) on the melange side is evaluated at the trapezoidal step midpoint
/// (t_n - T/2): melange holds R constant across each step while ngspice
/// integrates through the continuously-varying R, so the midpoint value is
/// the natural zero-order-hold representative. Measured 2026-07-18:
///   midpoint  (t_n - T/2): rms 1.28%, peak 3.93e-2 V, corr 0.99991763
///   end-point (t_n):       rms 1.70%, peak 3.04e-2 V, corr 0.99985445
/// Midpoint wins on rms and correlation and is what the test uses. The
/// static off-nominal test above sits at the 0.037% floor, confirming the
/// residual here is R(t) discretization, not the pot mechanism.
#[test]
#[ignore] // requires ngspice
fn test_pot_modulation_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== Pot Modulation (5 kHz R sweep 1k-10k, B-source ref) Validation ===");

    // 500 Hz, 1 V, 50 ms signal; R modulated 1k..10k at 5 kHz (per the deck)
    let num_samples = (SAMPLE_RATE * 0.050) as usize;
    let input: Vec<f64> = (0..num_samples)
        .map(|i| (2.0 * std::f64::consts::PI * 500.0 * i as f64 / SAMPLE_RATE).sin())
        .collect();

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let stdin = std::io::stdin();
    let mut line = String::new();
    let mut n: u64 = 0;
    loop {
        line.clear();
        if stdin.read_line(&mut line).unwrap() == 0 { break; }
        if let Ok(input) = line.trim().parse::<f64>() {
            // R(t) evaluated at the step midpoint (t_n - T/2); melange holds
            // R constant over the step [t_{n-1}, t_n] that ngspice integrates
            // with continuously-varying R. Matches the deck's B-source law:
            // R(t) = 5500 + 4500*sin(2*pi*5000*t).
            let t = (n as f64 - 0.5) / 48000.0;
            let r = 5500.0 + 4500.0 * (2.0 * std::f64::consts::PI * 5000.0 * t).sin();
            state.set_pot_0(r);
            let out = process_sample(input, &mut state);
            println!("{:.15e}", out[0]);
            n += 1;
        }
    }
}
"#;

    let (spice_output, melange_output) = run_pot_validation("circuit.cir", &input, main_code);

    // Gates measured 2026-07-18 (midpoint R evaluation, see doc comment):
    // rms 1.28%, peak 3.93e-2 V, corr 0.99991763. Dynamic-R comparison is
    // inherently looser than static: melange's per-sample zero-order hold of
    // R(t) vs ngspice's continuous B-source evaluation leaves a genuine
    // discretization residual at 5 kHz mod / 48 kHz fs (9.6 samples per
    // modulation period). Gates sit ~2.5-12x over measured.
    let config = ComparisonConfig {
        rms_error_tolerance: 0.05,    // measured 1.28% → 3.9x headroom
        peak_error_tolerance: 0.10,   // measured 3.93e-2 V → 2.5x headroom
        max_relative_tolerance: 50.0, // near zero-crossings under modulation
        correlation_min: 0.999,       // measured 1-corr = 8.2e-5 → 12x headroom
        thd_error_tolerance_db: 5.0,
        full_scale: 1.0,
        skip_thd: true, // modulation sidebands, not harmonics — THD is meaningless
        settle_time_s: 0.0,
    };
    let spice_signal = Signal::new(spice_output, SAMPLE_RATE, "SPICE");
    let melange_signal = Signal::new(melange_output, SAMPLE_RATE, "Melange");
    let mut report = compare_signals(&spice_signal, &melange_signal, &config);
    report.circuit_name = "pot_modulation".to_string();
    report.node_name = "out".to_string();

    let result = ValidationResult {
        report,
        html_report_path: None,
    };
    print_validation_metrics(&result);

    assert!(
        result.report.passed,
        "Pot modulation validation failed:\n{}",
        result.report.summary()
    );
}

// Klon Centaur validation removed: the circuit is deferred (circuits/unstable/,
// never reached a working SPICE-validated state) and its test-data dir
// (data/klon_centaur/) no longer exists, so the test only ever panicked on the
// missing netlist. Restore both the data and the test together if klon is revived.

/// Test: 12AX7 common-cathode triode gain stage vs ngspice.
///
/// Proves the tube-translation path (melange-validate `tube_translate`): the
/// ngspice reference deck's `T` element is auto-rewritten into a Koren B-source
/// `.subckt` that reproduces melange's own triode equation, so this is a
/// self-consistent cross-check of melange's transient SOLVER on a tube circuit
/// (NR + integration + timestep), not an independent tube-physics test. Before
/// this path existed, ngspice parsed `T` as a transmission line and the deck
/// could not run at all.
#[test]
#[ignore] // requires ngspice
fn test_triode_cc_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== 12AX7 Common-Cathode Triode Validation ===");

    // 500 Hz, 40 mV small-signal drive (grid stays below cutoff → linear-ish
    // plate operation, matching the golden reference conditions). input[0] = 0
    // so both engines settle from the same DC operating point at onset.
    let n = (SAMPLE_RATE * 0.1) as usize;
    let input: Vec<f64> = (0..n)
        .map(|i| 0.04 * (2.0 * std::f64::consts::PI * 500.0 * i as f64 / SAMPLE_RATE).sin())
        .collect();

    let netlist_path = test_data_dir().join("triode_cc").join("circuit.cir");

    let config = ComparisonConfig {
        // Measured 0.0035% RMS / corr 1.0 after the conditional-parasitic fix
        // (the twin no longer adds 10 pF caps this cap-loaded circuit doesn't
        // have; was 0.1025% with them). 0.05% gate ~14x over measured, and low
        // enough to catch a regression that re-adds unconditional parasitics
        // (which pushed this back to ~0.10%).
        rms_error_tolerance: 5e-4,
        peak_error_tolerance: 0.05,
        max_relative_tolerance: 0.05,
        correlation_min: 0.999,
        thd_error_tolerance_db: 5.0,
        full_scale: 1.0,
        skip_thd: true, // small-signal triode: solver-parity test, not distortion
        settle_time_s: 0.02,
    };

    let result = validate_circuit(&netlist_path, &input, SAMPLE_RATE, "out", &config)
        .expect("triode CC validation failed to run");

    println!("  Samples:     {}", result.report.sample_count);
    println!(
        "  RMS Error:   {:.6e} ({:.4}%)",
        result.report.rms_error,
        result.report.normalized_rms_error * 100.0
    );
    println!("  Peak Error:  {:.6e}", result.report.peak_error);
    println!("  Max Rel Err: {:.6e}", result.report.max_relative_error);
    println!(
        "  Correlation: {:.8}",
        result.report.correlation_coefficient
    );

    assert!(
        result.report.passed,
        "Triode CC validation failed:\n{}",
        result.report.summary()
    );
}

/// Test: 12AX7 common-cathode triode gain stage vs ngspice, PLATE OVERDRIVE.
///
/// Drives the same fixture as `test_triode_cc_vs_spice` at 1.0V/500Hz (25x
/// the small-signal amplitude): the grid swings roughly -2.2V..-0.2V (stays
/// below cutoff, no grid conduction) but the plate node swings tens of volts
/// as the triode is driven hard into its nonlinear knee. This previously
/// diverged ~78% RMS / ~50V peak / corr 0.933 from ngspice.
///
/// Root cause (investigated 2026-08, dr-debuggenshmirtz): NOT a solver/NR/
/// device-model bug. Per-sample dumps of melange's raw (pre-DC-block,
/// pre-clamp) node voltages against ngspice's raw node voltages agree to
/// within ~0.1V even at 200+V swings (diag_nr_max_iter_count/be_fallback/
/// voltage_damp/nan_reset all stayed 0 throughout) — the physics and the NR
/// solve are correct. The divergence was entirely the harness's hardcoded
/// `output_clamp_v` default of 10V (a line-level-circuit safety ceiling,
/// see docs/aidocs/SIGNAL_LEVELS.md) silently hard-clipping this circuit's
/// legitimate ~50-60V raw output swing (this fixture taps the triode plate
/// directly through a coupling cap, with the B+ rail at 250V) into a flat
/// +/-10V square wave every sample. Fixed by auto-scaling the ceiling from
/// the DC operating point's node-voltage headroom in
/// `run_melange_solver_from_str` (src/lib.rs) — never lowers below the
/// historical 10V default, so line-level circuits are unaffected.
#[test]
#[ignore] // requires ngspice
fn test_triode_cc_overdrive_vs_spice() {
    assert!(is_ngspice_available(), "ngspice not found");

    println!("\n=== 12AX7 Common-Cathode Triode Overdrive Validation ===");

    // 500 Hz, 1.0V drive: grid swings well below cutoff (no grid conduction)
    // but the plate is driven hard into overdrive.
    let n = (SAMPLE_RATE * 0.5) as usize;
    let input: Vec<f64> = (0..n)
        .map(|i| 1.0 * (2.0 * std::f64::consts::PI * 500.0 * i as f64 / SAMPLE_RATE).sin())
        .collect();

    let netlist_path = test_data_dir().join("triode_cc").join("circuit.cir");

    // Gates measured 2026-08 after the output_clamp_v auto-scale fix AND the
    // conditional-parasitic fix: RMS 0.0164%, peak err 1.58e-2 V, max rel err
    // 2.63e-4, corr 0.99999999 (was 0.0935% before the parasitic fix). 0.1%
    // gate ~6x over measured, and catches a regression re-adding unconditional
    // parasitics.
    let config = ComparisonConfig {
        rms_error_tolerance: 0.001,
        peak_error_tolerance: 0.15,
        max_relative_tolerance: 0.02,
        correlation_min: 0.999,
        thd_error_tolerance_db: 5.0,
        full_scale: 1.0,
        skip_thd: true, // overdriven triode: solver-parity test, not distortion
        settle_time_s: 0.1,
    };

    let result = validate_circuit(&netlist_path, &input, SAMPLE_RATE, "out", &config)
        .expect("triode CC overdrive validation failed to run");

    println!("  Samples:     {}", result.report.sample_count);
    println!(
        "  RMS Error:   {:.6e} ({:.4}%)",
        result.report.rms_error,
        result.report.normalized_rms_error * 100.0
    );
    println!("  Peak Error:  {:.6e}", result.report.peak_error);
    println!("  Max Rel Err: {:.6e}", result.report.max_relative_error);
    println!(
        "  Correlation: {:.8}",
        result.report.correlation_coefficient
    );

    assert!(
        result.report.passed,
        "Triode CC overdrive validation failed:\n{}",
        result.report.summary()
    );
}
