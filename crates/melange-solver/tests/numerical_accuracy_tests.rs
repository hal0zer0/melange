//! Numerical accuracy and edge case tests for the DK solver.
//!
//! These tests verify that the solver produces physically correct results
//! for known circuits, matches analytical solutions where possible, and
//! remains stable under extreme inputs.
//!
//! All tests use the codegen compile-and-run pipeline via the shared
//! test harness (support module).

mod support;

use melange_solver::codegen::CodegenConfig;
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const SAMPLE_RATE: f64 = 44100.0;

// ============================================================================
// Circuit definitions (shared across tests for compilation caching)
// ============================================================================

/// Standard two-node RC lowpass. G_in at "in" node doesn't affect the
/// output time constant tau = R1*C1 = 1ms = 44.1 samples at 44100 Hz.
const RC_LOWPASS: &str = "RC Lowpass\nR1 in out 1k\nC1 out 0 1u\n";

const SHUNT_CLIPPER: &str =
    "Shunt Clipper\nRin in out 1k\nD1 out 0 D1N4148\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";

const SHUNT_CLIPPER_NO_CAP: &str =
    "No Cap\nRin in 0 1k\nD1 in out D1N4148\nR1 out 0 1k\n.model D1N4148 D(IS=1e-15)\n";

const SHUNT_CLIPPER_WITH_CAP: &str =
    "With Cap\nRin in 0 1k\nD1 in out D1N4148\nR1 out 0 1k\nC1 out 0 100p\n.model D1N4148 D(IS=1e-15)\n";

const TWO_DIODES: &str =
    "Two Diodes\nRin in 0 1k\nD1 in mid D1N4148\nD2 mid out D1N4148\nR1 mid 0 10k\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";

const THREE_DIODES: &str =
    "Three Diodes\nRin in 0 1k\nD1 in m1 D1N4148\nD2 m1 m2 D1N4148\nD3 m2 out D1N4148\nR1 m1 0 10k\nR2 m2 0 10k\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";

const FOUR_DIODES: &str =
    "Four Diodes\nRin in 0 1k\nD1 in m1 D1N4148\nD2 m1 m2 D1N4148\nD3 m2 m3 D1N4148\nD4 m3 out D1N4148\nR1 m1 0 10k\nR2 m2 0 10k\nR3 m3 0 10k\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";

const BJT_CE_SIMPLE: &str =
    "BJT CE\nQ1 c b e 2N2222\nRc c 0 1k\nRb b 0 100k\nRe e 0 100\n.model 2N2222 NPN(IS=1e-15 BF=200)\n";

const RL_CIRCUIT: &str = "RL Circuit\nR1 in out 1k\nL1 out 0 10m\nC1 out 0 100p\n";

const PI_FILTER: &str =
    "Pi Filter\nR1 in n1 100\nL1 n1 mid 10m\nC1 mid 0 1u\nL2 mid out 10m\nR2 out 0 1k\nC2 out 0 100p\nC3 n1 0 100p\n";

const ZERO_TEST: &str =
    "Zero Test\nRin in 0 1k\nD1 in out D1N4148\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";

// ============================================================================
// Helpers
// ============================================================================

/// Config for two-node circuits with in/out nodes and custom input resistance.
fn two_node_config(spice: &str, input_resistance: f64) -> CodegenConfig {
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let in_node = mna.node_map.get("in").copied().unwrap_or(1).saturating_sub(1);
    let out_node = mna.node_map.get("out").copied().unwrap_or(2).saturating_sub(1);
    CodegenConfig {
        circuit_name: "test".to_string(),
        sample_rate: SAMPLE_RATE,
        input_node: in_node,
        output_nodes: vec![out_node],
        input_resistance,
        ..CodegenConfig::default()
    }
}

/// Config for BJT CE circuit (input=base, output=collector).
fn bjt_ce_config() -> CodegenConfig {
    let netlist = Netlist::parse(BJT_CE_SIMPLE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let b_node = mna.node_map.get("b").copied().unwrap_or(1).saturating_sub(1);
    let c_node = mna.node_map.get("c").copied().unwrap_or(2).saturating_sub(1);
    CodegenConfig {
        circuit_name: "test_bjt".to_string(),
        sample_rate: SAMPLE_RATE,
        input_node: b_node,
        output_nodes: vec![c_node],
        input_resistance: 100000.0, // Rb = 100k
        ..CodegenConfig::default()
    }
}

// ============================================================================
// Test 1: RC Lowpass Exact Time Constant
// ============================================================================

#[test]
fn test_rc_lowpass_exact_time_constant() {
    let config = support::config_for_spice(RC_LOWPASS, SAMPLE_RATE);
    let circuit = support::build_circuit(RC_LOWPASS, &config, "rc_tau");
    let output = support::run_step(&circuit, 1.0, 300, SAMPLE_RATE);

    // tau = R1*C1 = 1k * 1u = 1ms = 44.1 samples at 44100 Hz
    let tau_samples: f64 = 44.1;

    // At 1 tau (sample 44): v ~ 1 - exp(-1) ~ 0.632
    let analytical_1tau = 1.0 - (-44.0_f64 / tau_samples).exp();
    assert!(
        (output[44] - analytical_1tau).abs() < 0.05,
        "At 1 tau: output[44] = {:.4}, expected ~{:.4} (tolerance 0.05)",
        output[44], analytical_1tau
    );

    // Peak output should exceed 0.8V
    let peak = output.iter().cloned().fold(0.0f64, f64::max);
    assert!(peak > 0.8, "Peak output = {:.4}, expected > 0.8", peak);

    // Monotonically increasing in first 100 samples
    for i in 1..100 {
        assert!(
            output[i] >= output[i - 1] - 1e-10,
            "Output should increase: output[{}]={:.6} < output[{}]={:.6}",
            i, output[i], i - 1, output[i - 1]
        );
    }
}

// ============================================================================
// Test 2: RC Highpass Impulse Response (Discharge Test)
// ============================================================================

#[test]
fn test_rc_highpass_impulse_response() {
    let config = support::config_for_spice(RC_LOWPASS, SAMPLE_RATE);
    let (code, _, _) = support::generate_circuit_code(RC_LOWPASS, &config);

    // Custom main: impulse (1.0 for first sample, then 0.0)
    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    // Impulse: 1V for first sample, then 0V
    let out0 = process_sample(1.0, &mut state);
    println!("{:.15e}", out0[0]);
    for _ in 1..300 {
        let out = process_sample(0.0, &mut state);
        println!("{:.15e}", out[0]);
    }
}
"#;
    let result = support::compile_and_run(&code, main_code, "rc_impulse");
    let output = result.parse_samples();
    assert_eq!(output.len(), 300);

    // Initial response should be positive (capacitor charges on impulse)
    assert!(output[0] > 0.0, "Initial impulse response should be positive, got {:.6}", output[0]);

    // Output decays toward zero after impulse (RC discharge through R1)
    // tau = R1*C1 = 44.1 samples; at ~1 tau, ratio ≈ exp(-1)
    if output[1] > 1e-12 {
        let ratio = output[45] / output[1];
        let expected = (-1.0_f64).exp(); // ~0.368
        assert!(
            (ratio - expected).abs() < 0.15,
            "Decay ratio output[45]/output[1] = {:.4}, expected ~{:.4}", ratio, expected
        );
    }

    // After ~5 tau: near zero
    assert!(output[220].abs() < 0.01, "After 220 samples: {:.6}", output[220]);
}

// ============================================================================
// Test 3: Diode Forward Voltage Shunt Clipper
// ============================================================================

#[test]
fn test_diode_forward_voltage_shunt_clipper() {
    let config = two_node_config(SHUNT_CLIPPER, 1000.0);
    let circuit = support::build_circuit(SHUNT_CLIPPER, &config, "shunt_clip");
    let output = support::run_step(&circuit, 5.0, 1000, SAMPLE_RATE);

    let peak = output.iter().cloned().fold(0.0f64, f64::max);
    assert!(peak > 0.5, "Peak {:.4}V should be > 0.5V", peak);
    assert!(peak < 1.0, "Peak {:.4}V should be < 1.0V", peak);

    assert!(output[50] > output[5], "Output should increase during transient");

    let last_50 = &output[950..1000];
    let min_last = last_50.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_last = last_50.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    assert!((max_last - min_last) < 0.15, "Last 50 should settle: range={:.4}", max_last - min_last);
}

// ============================================================================
// Test 4: Diode Logarithmic Voltage Scaling
// ============================================================================

#[test]
fn test_diode_logarithmic_voltage_scaling() {
    let config = two_node_config(SHUNT_CLIPPER, 1000.0);

    let run_to_steady = |amp: f64| -> f64 {
        let circuit = support::build_circuit(SHUNT_CLIPPER, &config, "diode_log");
        let output = support::run_step(&circuit, amp, 500, SAMPLE_RATE);
        *output.last().unwrap()
    };

    let v1 = run_to_steady(1.0);
    let v2 = run_to_steady(2.0);
    let v5 = run_to_steady(5.0);

    assert!(v1 > 0.4, "v_out(1V) = {:.4}, expected > 0.4", v1);
    assert!(v5 < 1.0, "v_out(5V) = {:.4}, expected < 1.0", v5);
    assert!((v5 - v1) < 0.2, "v_out(5V)-v_out(1V) = {:.4}, expected < 0.2", v5 - v1);

    let inc = v2 - v1;
    assert!(inc < v2 * 0.5, "Increment 1->2V ({:.4}) should be << v_out(2V) ({:.4})", inc, v2);
}

// ============================================================================
// Test 5: Multi-Device Two Diodes Converges
// ============================================================================

#[test]
fn test_multi_device_two_diodes_converges() {
    let config = two_node_config(TWO_DIODES, 1000.0);
    let circuit = support::build_circuit(TWO_DIODES, &config, "two_diodes");
    assert_eq!(circuit.m, 2, "Two diodes should produce m=2, got m={}", circuit.m);

    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);

    let final_out = *output.last().unwrap();
    assert!(final_out > 0.0, "Final should be positive, got {:.6}", final_out);
    assert!(final_out < 1.0, "Final {:.4} should be < 1V", final_out);
}

// ============================================================================
// Test 6: Multi-Device Three Devices Converges
// ============================================================================

#[test]
fn test_multi_device_three_devices_converges() {
    let config = two_node_config(THREE_DIODES, 1000.0);
    let circuit = support::build_circuit(THREE_DIODES, &config, "three_diodes");
    assert_eq!(circuit.m, 3, "Three diodes should produce m=3, got m={}", circuit.m);

    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);

    let final_out = *output.last().unwrap();
    assert!(final_out >= 0.0, "Final should be non-negative, got {:.6}", final_out);
    assert!(final_out < 1.0, "Final {:.4} should be < 1V", final_out);
}

// ============================================================================
// Test 7: BJT Common Emitter Bias Point
// ============================================================================

#[test]
fn test_bjt_common_emitter_bias_point() {
    let config = bjt_ce_config();
    let circuit = support::build_circuit(BJT_CE_SIMPLE, &config, "bjt_ce");
    assert_eq!(circuit.m, 2, "One BJT should produce m=2, got m={}", circuit.m);

    let output = support::run_step(&circuit, 0.7, 200, SAMPLE_RATE);
    support::assert_finite(&output);

    let final_out = *output.last().unwrap();
    assert!(final_out.abs() < 5.0, "Collector voltage {:.4}V should be < 5V", final_out);
}

// ============================================================================
// Test 8: Purely Resistive Circuit With Cap Stable
// ============================================================================

#[test]
fn test_purely_resistive_circuit_with_cap_stable() {
    let config_no = two_node_config(SHUNT_CLIPPER_NO_CAP, 1000.0);
    let config_with = two_node_config(SHUNT_CLIPPER_WITH_CAP, 1000.0);

    let circuit_no = support::build_circuit(SHUNT_CLIPPER_NO_CAP, &config_no, "no_cap");
    let circuit_with = support::build_circuit(SHUNT_CLIPPER_WITH_CAP, &config_with, "with_cap");

    let out_no = support::run_step(&circuit_no, 1.0, 100, SAMPLE_RATE);
    let out_with = support::run_step(&circuit_with, 1.0, 100, SAMPLE_RATE);

    let pp = |output: &[f64]| -> f64 {
        let last20 = &output[80..100];
        let min_v = last20.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_v = last20.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        max_v - min_v
    };

    assert!(pp(&out_no) < 0.005, "No-cap pp={:.6e}", pp(&out_no));
    assert!(pp(&out_with) < 0.005, "With-cap pp={:.6e}", pp(&out_with));
    assert!((out_no[99] - out_with[99]).abs() < 0.01, "Steady states: no={:.6}, with={:.6}", out_no[99], out_with[99]);
    support::assert_finite(&out_no);
    support::assert_finite(&out_with);
}

// ============================================================================
// Test 9: Solver Output Never Exceeds 10V For Any Input
// ============================================================================

#[test]
fn test_solver_output_never_exceeds_10v_for_any_input() {
    let config = two_node_config(SHUNT_CLIPPER, 1000.0);
    let (code, _, _) = support::generate_circuit_code(SHUNT_CLIPPER, &config);

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let extreme_inputs: &[f64] = &[0.0, 1.0, -1.0, 5.0, -5.0, 10.0, -10.0, 100.0, -100.0, 0.001, -0.001];
    for &input_v in extreme_inputs {
        for _ in 0..50 {
            let out = process_sample(input_v, &mut state);
            if !out[0].is_finite() || out[0].abs() > 10.0 {
                println!("FAIL:input={},output={:.6e}", input_v, out[0]);
                std::process::exit(1);
            }
        }
    }
    println!("PASS");
}
"#;
    let result = support::compile_and_run(&code, main_code, "extreme_input");
    assert!(result.stdout.contains("PASS"), "Extreme input test failed: {}", result.stdout);
}

// ============================================================================
// Test 10: Zero Input Produces Zero Output
// ============================================================================

#[test]
fn test_zero_input_produces_zero_output() {
    let config = two_node_config(ZERO_TEST, 1000.0);
    let circuit = support::build_circuit(ZERO_TEST, &config, "zero_test");
    let output = support::run_step(&circuit, 0.0, 100, SAMPLE_RATE);

    for (i, &v) in output.iter().enumerate() {
        assert!(v.abs() < 1e-10, "Zero input: sample {} = {:.2e}", i, v);
    }
}

// ============================================================================
// Test 11: NaN Input Handling
// ============================================================================

#[test]
fn test_solver_handles_nan_input() {
    let config = two_node_config(SHUNT_CLIPPER, 1000.0);
    let (code, _, _) = support::generate_circuit_code(SHUNT_CLIPPER, &config);

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();

    // Normal operation
    for _ in 0..10 {
        let out = process_sample(1.0, &mut state);
        assert!(out[0].is_finite(), "Normal input should be finite");
    }

    // Feed NaN — codegen should produce finite output
    let out_nan = process_sample(f64::NAN, &mut state);
    assert!(out_nan[0].is_finite(), "NaN input should produce finite output");

    // Feed Inf
    let out_inf = process_sample(f64::INFINITY, &mut state);
    assert!(out_inf[0].is_finite(), "Inf input should produce finite output");

    // Recovery
    for _ in 0..50 {
        let out = process_sample(0.0, &mut state);
        assert!(out[0].is_finite(), "Recovery should be finite");
    }
    println!("PASS");
}
"#;
    let result = support::compile_and_run(&code, main_code, "nan_input");
    assert!(result.stdout.contains("PASS"), "NaN test failed:\nstdout: {}\nstderr: {}", result.stdout, result.stderr);
}

// ============================================================================
// Error-path tests (no runtime solver needed — kernel validation only)
// ============================================================================

#[test]
fn test_dk_kernel_rejects_zero_sample_rate() {
    let spice = "RC\nR1 in 0 1k\nC1 in 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert!(DkKernel::from_mna(&mna, 0.0).is_err(), "Zero sample rate should be rejected");
}

#[test]
fn test_dk_kernel_rejects_negative_sample_rate() {
    let spice = "RC\nR1 in 0 1k\nC1 in 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert!(DkKernel::from_mna(&mna, -44100.0).is_err(), "Negative sample rate should be rejected");
}

#[test]
fn test_dk_kernel_rejects_nan_sample_rate() {
    let spice = "RC\nR1 in 0 1k\nC1 in 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert!(DkKernel::from_mna(&mna, f64::NAN).is_err(), "NaN sample rate should be rejected");
}

// ============================================================================
// Inductor tests
// ============================================================================

#[test]
fn test_inductor_rl_circuit_finite_output() {
    let config = support::config_for_spice(RL_CIRCUIT, SAMPLE_RATE);
    let circuit = support::build_circuit(RL_CIRCUIT, &config, "rl_finite");
    let output = support::run_step(&circuit, 1.0, 500, SAMPLE_RATE);
    support::assert_finite(&output);
    support::assert_bounded(&output, -10.0, 10.0);
}

// ============================================================================
// M>2 convergence tests
// ============================================================================

#[test]
fn test_m3_three_diodes_converges() {
    let config = two_node_config(THREE_DIODES, 1000.0);
    let circuit = support::build_circuit(THREE_DIODES, &config, "m3_diodes");
    assert_eq!(circuit.m, 3, "Three diodes should produce m=3");

    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);
}

#[test]
fn test_m4_four_diodes_converges() {
    let config = two_node_config(FOUR_DIODES, 1000.0);
    let circuit = support::build_circuit(FOUR_DIODES, &config, "m4_diodes");
    assert_eq!(circuit.m, 4, "Four diodes should produce m=4");

    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);
}

// ============================================================================
// Multi-inductor pi-filter
// ============================================================================

#[test]
fn test_multi_inductor_pi_filter() {
    let config = support::config_for_spice(PI_FILTER, SAMPLE_RATE);
    let circuit = support::build_circuit(PI_FILTER, &config, "pi_filter");
    let output = support::run_step(&circuit, 1.0, 500, SAMPLE_RATE);
    support::assert_finite(&output);
    support::assert_bounded(&output, -10.0, 10.0);
}

// ============================================================================
// NR fallback — MaxIterations graceful degradation
// ============================================================================

#[test]
fn test_nr_max_iterations_graceful_degradation() {
    // Use max_iterations=2 to force early NR termination
    let mut config = two_node_config(SHUNT_CLIPPER, 1000.0);
    config.max_iterations = 2;
    config.circuit_name = "test_max_iter".to_string();

    let circuit = support::build_circuit(SHUNT_CLIPPER, &config, "nr_maxiter");
    let output = support::run_step(&circuit, 10.0, 100, SAMPLE_RATE);
    support::assert_finite(&output);
}

// ============================================================================
// NR NaN/Inf recovery
// ============================================================================

#[test]
fn test_nr_nan_recovery() {
    let config = two_node_config(SHUNT_CLIPPER, 1000.0);
    let (code, _, _) = support::generate_circuit_code(SHUNT_CLIPPER, &config);

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let mut all_ok = true;

    // Normal operation
    for _ in 0..20 {
        let out = process_sample(1.0, &mut state);
        if !out[0].is_finite() { all_ok = false; }
    }

    // Feed NaN for several samples
    for _ in 0..5 {
        let out = process_sample(f64::NAN, &mut state);
        if !out[0].is_finite() { all_ok = false; }
    }

    // Recovery
    for _ in 0..200 {
        let out = process_sample(0.0, &mut state);
        if !out[0].is_finite() || out[0].abs() > 10.0 { all_ok = false; }
    }

    // Final should be small
    let final_out = process_sample(0.0, &mut state);
    if !final_out[0].is_finite() || final_out[0].abs() >= 1.0 { all_ok = false; }

    if all_ok { println!("PASS"); } else { println!("FAIL"); }
}
"#;
    let result = support::compile_and_run(&code, main_code, "nan_recovery");
    assert!(result.stdout.contains("PASS"), "NaN recovery failed:\n{}\n{}", result.stdout, result.stderr);
}

// ============================================================================
// Inductor NaN reset / recovery
// ============================================================================

#[test]
fn test_inductor_nan_reset_recovery() {
    let config = support::config_for_spice(RL_CIRCUIT, SAMPLE_RATE);
    let (code, _, _) = support::generate_circuit_code(RL_CIRCUIT, &config);

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let mut ok = true;

    // Normal operation
    for _ in 0..50 {
        let out = process_sample(1.0, &mut state);
        if !out[0].is_finite() { ok = false; }
    }

    // Feed NaN
    let _ = process_sample(f64::NAN, &mut state);

    // Recovery
    for _ in 0..100 {
        let out = process_sample(0.0, &mut state);
        if !out[0].is_finite() { ok = false; }
    }

    if ok { println!("PASS"); } else { println!("FAIL"); }
}
"#;
    let result = support::compile_and_run(&code, main_code, "rl_nan");
    assert!(result.stdout.contains("PASS"), "RL NaN recovery failed:\n{}\n{}", result.stdout, result.stderr);
}
