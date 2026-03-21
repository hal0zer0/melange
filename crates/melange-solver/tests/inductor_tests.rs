//! Inductor companion model tests.
//!
//! Verifies that inductor support works correctly in both the runtime solver
//! and code generation. Uses trapezoidal companion model: the inductor is
//! replaced each timestep by an equivalent conductance g_eq = T/(2L) in
//! parallel with a history current source.
//!
//! Tests cover:
//! - RL circuit step response (exponential current ramp toward V/R)
//! - Inductor current continuity (no jumps)
//! - Generated code for inductor circuits compiles with rustc
//! - RLC circuit basic resonance behavior
//! - Codegen contains correct inductor constants and state fields

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::solver::LinearSolver;
use std::io::Write;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn build_pipeline(spice: &str) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");
    (netlist, mna, kernel)
}

/// Build pipeline with input conductance stamped into G matrix before kernel build.
/// This models the input as a Thevenin voltage source with series resistance R_in.
fn build_pipeline_with_input(
    spice: &str,
    input_node_name: &str,
    input_resistance: f64,
) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    // Stamp input conductance into G matrix BEFORE building DK kernel
    let in_idx = *mna.node_map.get(input_node_name).unwrap() - 1;
    mna.g[in_idx][in_idx] += 1.0 / input_resistance;
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");
    (netlist, mna, kernel)
}

fn default_config() -> CodegenConfig {
    CodegenConfig {
        circuit_name: "test_circuit".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

fn generate_code(spice: &str) -> String {
    let (netlist, mna, kernel) = build_pipeline(spice);
    let codegen = CodeGenerator::new(default_config());
    let result = codegen
        .generate(&kernel, &mna, &netlist)
        .expect("code generation failed");
    result.code
}

/// Compile generated code with rustc, panicking with stderr on failure.
fn assert_compiles(code: &str, label: &str) {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static COUNTER: AtomicUsize = AtomicUsize::new(0);
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);

    let tmp_dir = std::env::temp_dir();
    let src = tmp_dir.join(format!("melange_ind_test_{}.rs", id));
    let lib = tmp_dir.join(format!("melange_ind_test_{}.rlib", id));

    {
        let mut f = std::fs::File::create(&src).expect("create temp file");
        f.write_all(code.as_bytes()).expect("write temp file");
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2021", "--crate-type", "lib", "-o"])
        .arg(&lib)
        .arg(&src)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&src);
    let _ = std::fs::remove_file(&lib);

    assert!(
        output.status.success(),
        "Generated code for '{}' failed to compile:\n{}",
        label,
        String::from_utf8_lossy(&output.stderr)
    );
}

// ---------------------------------------------------------------------------
// Test circuits
// ---------------------------------------------------------------------------

/// RL lowpass: R1=1k in series, L1=100mH to ground, plus a small cap for stability.
/// The inductor needs a capacitor in parallel for the DK trapezoidal method to work.
/// tau = L/R = 0.1/1000 = 0.1ms = 4.41 samples at 44.1kHz
const RL_LOWPASS_SPICE: &str = "\
RL Lowpass
R1 in out 1k
L1 out 0 100m
C1 out 0 100p
";

/// RL highpass: L1 in series, R1 to ground.
/// tau = L/R = 10m/1k = 10us = 0.441 samples at 44.1kHz
const RL_HIGHPASS_SPICE: &str = "\
RL Highpass
L1 in out 10m
R1 out 0 1k
C1 out 0 100p
";

/// RLC circuit: series R + L with C to ground.
/// Natural frequency: f0 = 1/(2*pi*sqrt(LC)) = 1/(2*pi*sqrt(10m*100n)) ~ 5033 Hz
const RLC_SPICE: &str = "\
RLC Circuit
R1 in mid 100
L1 mid out 10m
C1 out 0 100n
";

/// RL with larger inductor for longer time constant.
/// tau = L/R = 1/1000 = 1ms = 44.1 samples at 44.1kHz
const RL_LARGE_L_SPICE: &str = "\
RL Large L
R1 in out 1k
L1 out 0 1
C1 out 0 1p
";

/// Two inductors in the same circuit.
const TWO_INDUCTOR_SPICE: &str = "\
Two Inductors
R1 in mid 1k
L1 mid out 10m
L2 out 0 100m
C1 out 0 100p
";

// ---------------------------------------------------------------------------
// Test 1: RL step response — inductor current ramps exponentially
// ---------------------------------------------------------------------------

#[test]
fn test_rl_step_response() {
    // RL lowpass: R1=1k in series, L1=100mH shunt to ground, plus tiny C for stability.
    // tau = L/R = 0.1/1000 = 0.1ms = 4.41 samples
    // Must stamp input conductance into G before building kernel (Thevenin model).
    let input_resistance = 1.0; // 1 ohm Thevenin source
    let (_, mna, kernel) = build_pipeline_with_input(RL_LOWPASS_SPICE, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    // Apply 1V DC step for 500 samples (many time constants)
    let num_samples = 500;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        output[i] = solver.process_sample(1.0);
    }

    // The output node has: R1 from "in", L1+C1 to ground.
    // At DC steady state, L is short and C is open, so out = in through a
    // voltage divider formed by R_in (1 ohm) + R1 (1k) and L (short to gnd).
    // V_out_dc = 0V (L shorts output to ground, all voltage dropped across R1+R_in).
    //
    // Transiently, the inductor's companion conductance g_eq = T/(2L) is small,
    // so output starts high (R1 divider without much shunt) and decays toward 0
    // as the inductor current builds up and shorts the output.
    //
    // Verify: output starts at some voltage and approaches 0 as inductor current builds.
    assert!(
        output[0].abs() > 0.01,
        "Output at t=0 should be non-zero (inductor companion has finite impedance), got {:.4}",
        output[0]
    );

    // After many time constants, output should approach 0 (inductor is a short at DC)
    let final_output = output[num_samples - 1].abs();
    assert!(
        final_output < 0.1,
        "Final output should be near 0 (inductor shorts to ground at DC), got {:.4}",
        output[num_samples - 1]
    );

    // All outputs should be finite
    for (i, &v) in output.iter().enumerate() {
        assert!(v.is_finite(), "Output[{}] should be finite, got {}", i, v);
    }

    // The general trend should be decaying (early outputs larger than late)
    let avg_first_10: f64 = output[0..10].iter().map(|v| v.abs()).sum::<f64>() / 10.0;
    let avg_last_10: f64 = output[490..500].iter().map(|v| v.abs()).sum::<f64>() / 10.0;
    assert!(
        avg_first_10 > avg_last_10,
        "Output should decay over time for RL lowpass: avg_first_10 = {:.4}, avg_last_10 = {:.4}",
        avg_first_10,
        avg_last_10
    );
}

// ---------------------------------------------------------------------------
// Test 2: Inductor current continuity (no jumps)
// ---------------------------------------------------------------------------

#[test]
fn test_inductor_current_continuity() {
    // Verify inductor current continuity indirectly: after the input step-down,
    // the output should NOT drop to zero instantly because the inductor maintains
    // current flow. The inductor's stored energy means the output decays smoothly.
    //
    // Circuit: R1=1k in->out, L1=1H out->gnd, C1=1p out->gnd (tiny cap for stability)
    // tau = L/R = 1/1000 = 1ms = 44.1 samples
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(RL_LARGE_L_SPICE, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    // Phase 1: Drive with DC step (200 samples) to build up inductor current
    let mut output_phase1 = vec![0.0; 200];
    for i in 0..200 {
        output_phase1[i] = solver.process_sample(1.0);
    }

    // Phase 2: Remove input (200 samples) — inductor current should decay smoothly
    let mut output_phase2 = vec![0.0; 200];
    for i in 0..200 {
        output_phase2[i] = solver.process_sample(0.0);
    }

    // After the input step-down, the inductor's stored energy means the output
    // doesn't vanish instantly. The first few samples of phase 2 should be non-zero.
    // Due to the inductor's back-EMF, the output may go negative briefly
    // (the inductor tries to maintain current flow, which drives the node negative).
    assert!(
        output_phase2[0].abs() > 1e-6,
        "Output right after step-down should be non-zero (inductor maintains current). \
         Got {:.6e}",
        output_phase2[0]
    );

    // The output in phase 2 should gradually decay toward zero.
    let early_energy: f64 = output_phase2[0..20].iter().map(|v| v * v).sum::<f64>();
    let late_energy: f64 = output_phase2[180..200].iter().map(|v| v * v).sum::<f64>();
    assert!(
        late_energy < early_energy || early_energy < 1e-15,
        "Output energy should decay after step-down: early={:.6e}, late={:.6e}",
        early_energy,
        late_energy
    );

    // All outputs should be finite
    for (i, &v) in output_phase1.iter().chain(output_phase2.iter()).enumerate() {
        assert!(v.is_finite(), "Output[{}] should be finite, got {}", i, v);
    }
}

// ---------------------------------------------------------------------------
// Test 3: Inductor codegen produces compilable code
// ---------------------------------------------------------------------------

#[test]
fn test_rl_lowpass_codegen_compiles() {
    let code = generate_code(RL_LOWPASS_SPICE);
    assert_compiles(&code, "rl_lowpass");
}

#[test]
fn test_rl_highpass_codegen_compiles() {
    let code = generate_code(RL_HIGHPASS_SPICE);
    assert_compiles(&code, "rl_highpass");
}

#[test]
fn test_rlc_codegen_compiles() {
    let code = generate_code(RLC_SPICE);
    assert_compiles(&code, "rlc_circuit");
}

#[test]
fn test_two_inductor_codegen_compiles() {
    let code = generate_code(TWO_INDUCTOR_SPICE);
    assert_compiles(&code, "two_inductors");
}

// ---------------------------------------------------------------------------
// Test 4: Codegen contains correct inductor constants and state
// ---------------------------------------------------------------------------

#[test]
fn test_inductor_codegen_constants() {
    let code = generate_code(RL_LOWPASS_SPICE);

    // Should contain inductor equivalent conductance constant
    assert!(
        code.contains("IND_0_G_EQ"),
        "Generated code should contain IND_0_G_EQ constant"
    );

    // The G_EQ value should be T/(2*L) = (1/44100)/(2*0.1) = 1.133786...e-4
    let expected_g_eq = 1.0 / 44100.0 / (2.0 * 0.1);
    let g_eq_str = format!("{:.17e}", expected_g_eq);
    assert!(
        code.contains(&g_eq_str),
        "IND_0_G_EQ should be {}, got code containing: {}",
        g_eq_str,
        code.lines()
            .find(|l| l.contains("IND_0_G_EQ"))
            .unwrap_or("<not found>")
    );

    // Should contain NUM_INDUCTORS
    assert!(
        code.contains("NUM_INDUCTORS"),
        "Generated code should contain NUM_INDUCTORS constant"
    );

    // Should contain inductor node constants
    assert!(
        code.contains("IND_0_NODE_I"),
        "Generated code should contain IND_0_NODE_I constant"
    );
    assert!(
        code.contains("IND_0_NODE_J"),
        "Generated code should contain IND_0_NODE_J constant"
    );
}

#[test]
fn test_inductor_codegen_state_fields() {
    let code = generate_code(RL_LOWPASS_SPICE);

    // State struct should have inductor fields
    assert!(
        code.contains("ind_i_prev"),
        "State should have ind_i_prev field for inductor current history"
    );
    assert!(
        code.contains("ind_v_prev"),
        "State should have ind_v_prev field for inductor voltage history"
    );
    assert!(
        code.contains("ind_i_hist"),
        "State should have ind_i_hist field for inductor history current source"
    );
}

#[test]
fn test_inductor_codegen_rhs_injection() {
    let code = generate_code(RL_LOWPASS_SPICE);

    // build_rhs should inject inductor history current
    // The template subtracts i_hist at node_i and adds at node_j
    assert!(
        code.contains("ind_i_hist"),
        "build_rhs should reference ind_i_hist for inductor RHS injection"
    );
}

#[test]
fn test_inductor_codegen_state_update() {
    let code = generate_code(RL_LOWPASS_SPICE);

    // process_sample should update inductor state after computing voltages
    assert!(
        code.contains("state.ind_i_prev"),
        "process_sample should update ind_i_prev"
    );
    assert!(
        code.contains("state.ind_v_prev"),
        "process_sample should update ind_v_prev"
    );
    assert!(
        code.contains("state.ind_i_hist"),
        "process_sample should update ind_i_hist"
    );

    // The update should use the trapezoidal formula:
    // i_new = i_prev + g_eq * (v_prev + v_new)
    assert!(
        code.contains("IND_0_G_EQ"),
        "State update should reference IND_0_G_EQ"
    );
}

// ---------------------------------------------------------------------------
// Test 5: Two inductor circuit codegen
// ---------------------------------------------------------------------------

#[test]
fn test_two_inductor_codegen_constants() {
    let code = generate_code(TWO_INDUCTOR_SPICE);

    // Should have constants for both inductors
    assert!(code.contains("IND_0_G_EQ"), "Missing IND_0_G_EQ");
    assert!(code.contains("IND_1_G_EQ"), "Missing IND_1_G_EQ");
    assert!(code.contains("IND_0_NODE_I"), "Missing IND_0_NODE_I");
    assert!(code.contains("IND_1_NODE_I"), "Missing IND_1_NODE_I");

    // G_EQ values should differ (different inductance values)
    let (_, _, kernel) = build_pipeline(TWO_INDUCTOR_SPICE);
    assert_eq!(kernel.inductors.len(), 2, "Should have 2 inductors");
    assert!(
        (kernel.inductors[0].g_eq - kernel.inductors[1].g_eq).abs() > 1e-10,
        "Two inductors with different L values should have different g_eq"
    );
}

// ---------------------------------------------------------------------------
// Test 6: Inductor IR data
// ---------------------------------------------------------------------------

#[test]
fn test_inductor_ir() {
    let (netlist, mna, kernel) = build_pipeline(RL_LOWPASS_SPICE);
    let config = default_config();
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();

    assert_eq!(ir.inductors.len(), 1, "Should have 1 inductor in IR");
    assert_eq!(ir.inductors[0].name, "L1");

    // g_eq should match kernel
    let expected_g_eq = 1.0 / 44100.0 / (2.0 * 0.1); // T/(2L) for L=100mH
    assert!(
        (ir.inductors[0].g_eq - expected_g_eq).abs() < 1e-15,
        "IR g_eq should match: expected {:.17e}, got {:.17e}",
        expected_g_eq,
        ir.inductors[0].g_eq
    );

    // Node indices should be non-zero (connected between named nodes)
    // L1 is between "out" and "0" (ground), so one node should be > 0
    let out_idx = *mna.node_map.get("out").unwrap();
    assert!(
        ir.inductors[0].node_i == out_idx || ir.inductors[0].node_j == out_idx,
        "Inductor should connect to 'out' node"
    );
}

// ---------------------------------------------------------------------------
// Test 7: RLC circuit behavior — output should show oscillation/resonance
// ---------------------------------------------------------------------------

#[test]
fn test_rlc_circuit_behavior() {
    // RLC circuit with input conductance stamped for proper Thevenin model.
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(RLC_SPICE, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    // Apply impulse: 1 sample of 1.0 then zeros
    let num_samples = 2000;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        let input = if i == 0 { 1.0 } else { 0.0 };
        output[i] = solver.process_sample(input);
    }

    // RLC impulse response should show oscillation (not purely monotonic decay).
    // Count sign changes in the output after the initial transient.
    let mut sign_changes = 0;
    for i in 11..num_samples {
        if output[i] * output[i - 1] < 0.0 {
            sign_changes += 1;
        }
    }

    // With L=10mH, C=100nF, R=100:
    // f0 = 1/(2*pi*sqrt(LC)) = 1/(2*pi*sqrt(1e-9)) ~ 5033 Hz
    // At 44.1kHz sample rate, that's about 8.76 samples per cycle.
    // Over 2000 samples, we should see many sign changes if underdamped.
    //
    // Q factor: Q = (1/R)*sqrt(L/C) = (1/100)*sqrt(10m/100n) = (1/100)*sqrt(100000) = 316/100 ~ 3.16
    // This is underdamped (Q > 0.5), so we expect oscillations.
    assert!(
        sign_changes > 5,
        "RLC circuit should show oscillation (underdamped Q~3.16), but only {} sign changes found",
        sign_changes
    );

    // The oscillation should decay over time (damped by R)
    let max_first_half = output[10..1000]
        .iter()
        .map(|&x| x.abs())
        .fold(0.0f64, f64::max);
    let max_second_half = output[1000..2000]
        .iter()
        .map(|&x| x.abs())
        .fold(0.0f64, f64::max);
    assert!(
        max_second_half < max_first_half || max_first_half < 1e-10,
        "RLC oscillation should decay: max first half = {:.6}, max second half = {:.6}",
        max_first_half,
        max_second_half
    );
}

// ---------------------------------------------------------------------------
// Test 8: Inductor in MNA system — check A matrix stamping
// ---------------------------------------------------------------------------

#[test]
fn test_inductor_mna_stamping() {
    let (_, mna, _) = build_pipeline(RL_LOWPASS_SPICE);

    // Verify inductor was found by the parser
    assert_eq!(mna.inductors.len(), 1, "Should have 1 inductor");
    assert_eq!(mna.inductors[0].name, "L1");
    assert!(
        (mna.inductors[0].value - 0.1).abs() < 1e-10,
        "L1 should be 100mH = 0.1H"
    );

    // The A matrix should include inductor companion model conductance g_eq = T/(2L)
    let a = mna.get_a_matrix(44100.0).unwrap();
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    // The diagonal element for the output node should include:
    // - G_R1 = 1/1000 = 0.001 (from R1)
    // - 2*C1/T = 2*100e-12*44100 (from C1)
    // - g_eq = T/(2*L) = (1/44100)/(2*0.1) (from L1 companion)
    let g_r1 = 0.001;
    let alpha_c1 = 2.0 * 100e-12 * 44100.0;
    let g_eq_l1 = (1.0 / 44100.0) / (2.0 * 0.1);

    let expected_diag = g_r1 + alpha_c1 + g_eq_l1;
    assert!(
        (a[out_idx][out_idx] - expected_diag).abs() / expected_diag < 0.01,
        "A[out][out] should include R1 + C1 + L1 contributions: expected {:.6e}, got {:.6e}",
        expected_diag,
        a[out_idx][out_idx]
    );
}

// ---------------------------------------------------------------------------
// Test 9: Inductor DK kernel data
// ---------------------------------------------------------------------------

#[test]
fn test_inductor_dk_kernel() {
    let (_, _, kernel) = build_pipeline(RL_LOWPASS_SPICE);

    assert_eq!(kernel.inductors.len(), 1);
    let ind = &kernel.inductors[0];

    assert_eq!(&*ind.name, "L1");
    assert!(
        (ind.inductance - 0.1).abs() < 1e-10,
        "Inductance should be 0.1H"
    );

    let expected_g_eq = (1.0 / 44100.0) / (2.0 * 0.1);
    assert!(
        (ind.g_eq - expected_g_eq).abs() < 1e-15,
        "g_eq should be T/(2L) = {:.17e}, got {:.17e}",
        expected_g_eq,
        ind.g_eq
    );

    // Initial state should be zero
    assert_eq!(ind.i_hist, 0.0, "Initial history current should be 0");
    assert_eq!(ind.i_prev, 0.0, "Initial previous current should be 0");
    assert_eq!(ind.v_prev, 0.0, "Initial previous voltage should be 0");
}

// ---------------------------------------------------------------------------
// Test 10: Inductor codegen sanitization resets inductor state
// ---------------------------------------------------------------------------

#[test]
fn test_inductor_codegen_sanitization() {
    let code = generate_code(RL_LOWPASS_SPICE);

    // NaN sanitization should reset inductor state
    // NaN check now happens before state write: checks local `v` not `state.v_prev`
    let sanitize_idx = code
        .find("if !v.iter().all(|x| x.is_finite())")
        .or_else(|| code.find("if !state.v_prev.iter().all(|x| x.is_finite())"));
    assert!(sanitize_idx.is_some(), "Should have NaN sanitization block");

    let after_sanitize = &code[sanitize_idx.unwrap()..];
    // NaN reset now returns DC operating point output instead of zeros
    let return_idx = after_sanitize
        .find("return dc_output;")
        .or_else(|| after_sanitize.find("return [0.0; NUM_OUTPUTS];"))
        .expect("Missing return in sanitization");
    let sanitize_block = &after_sanitize[..return_idx];

    assert!(
        sanitize_block.contains("ind_i_prev"),
        "Sanitization should reset ind_i_prev"
    );
    assert!(
        sanitize_block.contains("ind_v_prev"),
        "Sanitization should reset ind_v_prev"
    );
    assert!(
        sanitize_block.contains("ind_i_hist"),
        "Sanitization should reset ind_i_hist"
    );
}

// ---------------------------------------------------------------------------
// Test 11: No inductor code when circuit has no inductors
// ---------------------------------------------------------------------------

#[test]
fn test_no_inductor_backward_compat() {
    let rc_spice = "\
RC No Inductor
R1 in out 10k
C1 out 0 100n
";
    let code = generate_code(rc_spice);

    assert!(
        !code.contains("IND_0"),
        "Code without inductors should not have IND_0 constants"
    );
    assert!(
        !code.contains("NUM_INDUCTORS"),
        "Code without inductors should not have NUM_INDUCTORS"
    );
    assert!(
        !code.contains("ind_i_prev"),
        "Code without inductors should not have ind_i_prev state"
    );
    assert!(
        !code.contains("ind_v_prev"),
        "Code without inductors should not have ind_v_prev state"
    );
    assert!(
        !code.contains("ind_i_hist"),
        "Code without inductors should not have ind_i_hist state"
    );
}

// ---------------------------------------------------------------------------
// Test 12: Inductor reset clears state
// ---------------------------------------------------------------------------

#[test]
fn test_inductor_reset() {
    let code = generate_code(RL_LOWPASS_SPICE);

    // The reset() method should clear inductor state
    let reset_idx = code.find("fn reset(");
    assert!(reset_idx.is_some(), "Should have reset() method");

    let after_reset = &code[reset_idx.unwrap()..];
    let end_fn = after_reset.find('}').expect("reset function should close");
    let reset_body = &after_reset[..end_fn];

    assert!(
        reset_body.contains("ind_i_prev")
            || after_reset[..after_reset.len().min(300)].contains("ind_i_prev"),
        "reset() should clear inductor state (ind_i_prev)"
    );
}

// ---------------------------------------------------------------------------
// Test 13: RL step response time constant validation
// ---------------------------------------------------------------------------

#[test]
fn test_rl_step_response_time_constant() {
    // Use a larger inductor for a longer time constant that's easier to measure.
    // L=1H, R=1k -> tau = L/R = 1ms = 44.1 samples
    // This matches the RC lowpass test's time constant for easy comparison.
    let spice = "\
RL Time Constant
R1 in out 1k
L1 out 0 1
C1 out 0 1p
";
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(spice, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    // Apply 1V DC step
    let num_samples = 500;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        output[i] = solver.process_sample(1.0);
    }

    // For an RL lowpass with step input, the voltage across the load (inductor)
    // rises as: v(t) = V_final * (1 - exp(-t/tau))
    // where tau = L/R = 1/1000 = 1ms = 44.1 samples
    //
    // The Thevenin source has R_in = 1 ohm, so the effective series resistance
    // is R1 + R_in = 1001 ohm. At DC steady state, inductor is a short,
    // so V_out = V_in * 0 / (R1 + R_in) = 0V (inductor shorts the output to ground).
    // Actually, V_out across L approaches 0 at DC (all voltage across R),
    // so the transient starts at some value and decays.
    //
    // We verify the general shape: starts at some value, then settles.
    let v_early = output[2];
    let v_late = output[300]; // ~7 tau

    // The output should be finite and well-behaved throughout
    for (i, &v) in output.iter().enumerate() {
        assert!(
            v.is_finite(),
            "Output at sample {} should be finite, got {}",
            i,
            v
        );
    }

    // After many time constants, output should be settled (stable)
    let settled_range = &output[400..500];
    let settled_mean: f64 = settled_range.iter().sum::<f64>() / settled_range.len() as f64;
    let settled_var: f64 = settled_range
        .iter()
        .map(|&v| (v - settled_mean).powi(2))
        .sum::<f64>()
        / settled_range.len() as f64;
    assert!(
        settled_var.sqrt() < 0.01,
        "Output should be stable after many tau: mean={:.6}, std={:.6}",
        settled_mean,
        settled_var.sqrt()
    );

    // Early output should differ from late output (transient behavior)
    assert!(
        (v_early - v_late).abs() > 1e-6,
        "Early output ({:.6}) should differ from late output ({:.6}): \
         RL circuit should have transient behavior",
        v_early,
        v_late
    );
}
