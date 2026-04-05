//! Smoke test for the shared test harness.
//!
//! Validates that the support module compiles and the basic APIs work.

mod support;


const RC_LOWPASS: &str = "\
RC Lowpass
R1 in out 1k
C1 out 0 1u
";

const DIODE_CLIPPER: &str = "\
Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
";

#[test]
fn smoke_build_linear_circuit() {
    let config = support::config_for_spice(RC_LOWPASS, 44100.0);
    let circuit = support::build_circuit(RC_LOWPASS, &config, "smoke_linear");
    assert_eq!(circuit.m, 0, "RC lowpass should have m=0");
    assert!(circuit.n > 0, "should have nodes");
}

#[test]
fn smoke_run_sine_linear() {
    let config = support::config_for_spice(RC_LOWPASS, 44100.0);
    let circuit = support::build_circuit(RC_LOWPASS, &config, "smoke_sine");
    let samples = support::run_sine(&circuit, 1000.0, 0.5, 441, 44100.0);
    assert_eq!(samples.len(), 441);
    support::assert_finite(&samples);
}

#[test]
fn smoke_run_step_linear() {
    let config = support::config_for_spice(RC_LOWPASS, 44100.0);
    let circuit = support::build_circuit(RC_LOWPASS, &config, "smoke_step");
    let samples = support::run_step(&circuit, 1.0, 100, 44100.0);
    assert_eq!(samples.len(), 100);
    support::assert_finite(&samples);
    // RC lowpass step response should be positive and bounded
    support::assert_bounded(&samples, -0.01, 1.01);
}

#[test]
fn smoke_run_signal_linear() {
    let config = support::config_for_spice(RC_LOWPASS, 44100.0);
    let circuit = support::build_circuit(RC_LOWPASS, &config, "smoke_signal");
    let input: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
    let samples = support::run_signal(&circuit, &input, 44100.0);
    assert_eq!(samples.len(), 100);
    support::assert_finite(&samples);
}

#[test]
fn smoke_build_nonlinear_circuit() {
    let config = support::config_for_spice(DIODE_CLIPPER, 44100.0);
    let circuit = support::build_circuit(DIODE_CLIPPER, &config, "smoke_diode");
    assert!(circuit.m > 0, "diode clipper should have nonlinear devices");
}

#[test]
fn smoke_run_sine_nonlinear() {
    let config = support::config_for_spice(DIODE_CLIPPER, 44100.0);
    let circuit = support::build_circuit(DIODE_CLIPPER, &config, "smoke_diode_sine");
    let samples = support::run_sine(&circuit, 500.0, 1.0, 441, 44100.0);
    assert_eq!(samples.len(), 441);
    support::assert_finite(&samples);
    // Diode clipper should produce some output
    support::assert_peak_above(&samples, 0.01);
}

#[test]
fn smoke_compile_and_run_custom_main() {
    let config = support::config_for_spice(RC_LOWPASS, 44100.0);
    let (code, _, _) = support::generate_circuit_code(RC_LOWPASS, &config);

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let out = process_sample(0.5, &mut state);
    println!("result={:.6e}", out[0]);
}
"#;
    let output = support::compile_and_run(&code, main_code, "smoke_custom");
    let result = output.parse_kv("result").expect("should have result");
    assert!(result.is_finite(), "result should be finite");
}

#[test]
fn smoke_diagnostics() {
    let config = support::config_for_spice(DIODE_CLIPPER, 44100.0);
    let circuit = support::build_circuit(DIODE_CLIPPER, &config, "smoke_diag");
    let output = support::run_sine_full(&circuit, 500.0, 1.0, 441, 44100.0);
    // Should have diagnostic info on stderr
    let nr_max = output.diag("nr_max_iter_count");
    assert!(nr_max.is_some(), "should have nr_max_iter_count diagnostic");
}

/// Use a unique circuit that no other test uses, to verify cache hit.
const CACHE_TEST_CIRCUIT: &str = "\
Cache Test RC
R1 in out 4.7k
C1 out 0 470n
";

#[test]
fn smoke_compilation_cache_reuse() {
    let config = support::config_for_spice(CACHE_TEST_CIRCUIT, 44100.0);

    // Build twice with same circuit — second should hit cache
    let circuit1 = support::build_circuit(CACHE_TEST_CIRCUIT, &config, "cache1");
    let circuit2 = support::build_circuit(CACHE_TEST_CIRCUIT, &config, "cache2");

    assert_eq!(
        circuit1.binary_path, circuit2.binary_path,
        "same circuit should reuse cached binary"
    );
}
