//! Numerical accuracy and edge case tests for the DK solver.
//!
//! These tests verify that the solver produces physically correct results
//! for known circuits, matches analytical solutions where possible, and
//! remains stable under extreme inputs.

use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
use melange_solver::solver::{CircuitSolver, LinearSolver, DeviceEntry};
use melange_devices::{DiodeShockley, BjtEbersMoll};

const SAMPLE_RATE: f64 = 44100.0;

// ============================================================================
// Test 1: RC Lowpass Exact Time Constant
// ============================================================================

/// Verify that the RC lowpass step response matches the analytical
/// exponential charging curve v(t) = 1 - exp(-t/tau).
///
/// Single-node topology: R1=1k from node to ground, C1=1u from node to ground.
/// The solver's Thevenin model drives the node through input_conductance = 1/R1.
/// The total conductance at the node is G_R1 = 0.001 S, and the capacitor is C = 1uF.
/// The effective time constant is C / G_total = 1u / 0.001 = 1ms = 44.1 samples.
///
/// However, since the Thevenin model only injects current (does not stamp conductance
/// into the A matrix), the actual tau depends on R1 alone: tau = R1*C = 1ms.
#[test]
fn test_rc_lowpass_exact_time_constant() {
    let spice = "RC Lowpass\nR1 in 0 1k\nC1 in 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // Single-node circuit: input and output are both node 0 (the "in" node)
    let in_idx = *mna.node_map.get("in").unwrap() - 1;

    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();
    let mut solver = LinearSolver::new(kernel, in_idx, in_idx);
    solver.input_conductance = 0.001; // 1/R1 = 1/1k

    // Apply 1V DC step for 300 samples
    let num_samples = 300;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        output[i] = solver.process_sample(1.0);
    }

    // The time constant tau = R*C = 1k * 1u = 1ms = 44.1 samples
    let tau_samples: f64 = 44.1;

    // At 1 tau (sample 44): v ~ 1 - exp(-1) ~ 0.632
    let analytical_1tau = 1.0 - (-44.0_f64 / tau_samples).exp();
    assert!(
        (output[44] - analytical_1tau).abs() < 0.02,
        "At 1 tau: output[44] = {:.4}, expected ~{:.4} (tolerance 0.02)",
        output[44], analytical_1tau
    );

    // At 3 tau (sample 132): v ~ 1 - exp(-3) ~ 0.950
    let analytical_3tau = 1.0 - (-132.0_f64 / tau_samples).exp();
    assert!(
        (output[132] - analytical_3tau).abs() < 0.02,
        "At 3 tau: output[132] = {:.4}, expected ~{:.4} (tolerance 0.02)",
        output[132], analytical_3tau
    );

    // At 5 tau (sample 220): v ~ 1 - exp(-5) ~ 0.993
    let analytical_5tau = 1.0 - (-220.0_f64 / tau_samples).exp();
    assert!(
        (output[220] - analytical_5tau).abs() < 0.01,
        "At 5 tau: output[220] = {:.4}, expected ~{:.4} (tolerance 0.01)",
        output[220], analytical_5tau
    );

    // Final output should be very close to 1.0
    let final_output = output[num_samples - 1];
    assert!(
        (final_output - 1.0).abs() < 0.01,
        "Final output = {:.4}, expected ~1.0 (tolerance 0.01)",
        final_output
    );
}

// ============================================================================
// Test 2: RC Highpass Impulse Response (Discharge Test)
// ============================================================================

/// Verify that after a single-sample impulse, the RC circuit discharges
/// exponentially toward zero.
///
/// Single-node topology: R1=1k from node to ground, C1=1u from node to ground.
/// Apply 1V for a single sample, then 0V for the rest.
/// After the impulse, the capacitor discharges through R1.
/// The decay should follow v(n) ~ A * exp(-n / tau) where tau = R*C = 44.1 samples.
///
/// This tests the same physics as an RC highpass impulse response:
/// the output rises on the transient and then decays back to zero.
#[test]
fn test_rc_highpass_impulse_response() {
    let spice = "RC Discharge\nR1 in 0 1k\nC1 in 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let in_idx = *mna.node_map.get("in").unwrap() - 1;

    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();
    let mut solver = LinearSolver::new(kernel, in_idx, in_idx);
    solver.input_conductance = 0.001; // 1/R1 = 1/1k

    // Apply single impulse: 1V for one sample, then 0V
    let num_samples = 300;
    let mut output = vec![0.0; num_samples];
    output[0] = solver.process_sample(1.0);
    for i in 1..num_samples {
        output[i] = solver.process_sample(0.0);
    }

    // Initial response should be positive
    assert!(
        output[0] > 0.0,
        "Initial impulse response should be positive, got {:.6}",
        output[0]
    );

    // The decay ratio at ~1 tau (sample 44) relative to sample 1 should
    // approximate exp(-1) ~ 0.368.
    // We compare output[45] / output[1] since sample 0 includes the impulse injection.
    if output[1] > 1e-12 {
        let ratio = output[45] / output[1];
        let expected = (-1.0_f64).exp(); // ~0.368
        assert!(
            (ratio - expected).abs() < 0.15,
            "Decay ratio output[45]/output[1] = {:.4}, expected ~{:.4} (tolerance 0.15)",
            ratio, expected
        );
    }

    // After ~5 tau (220 samples), output should be very close to zero
    assert!(
        output[220].abs() < 0.01,
        "After 220 samples (~5 tau), output should be near zero, got {:.6}",
        output[220]
    );

    // Verify monotonic decay from sample 1 onward (after the impulse)
    for i in 1..num_samples - 1 {
        assert!(
            output[i] >= output[i + 1] - 1e-10,
            "Output should decay monotonically: output[{}]={:.6} < output[{}]={:.6}",
            i, output[i], i + 1, output[i + 1]
        );
    }
}

// ============================================================================
// Test 3: Diode Forward Voltage Shunt Clipper
// ============================================================================

/// Verify that a shunt diode clipper limits the output to the diode
/// forward voltage (~0.65-0.80V) when driven with 5V DC.
///
/// Circuit: Rin=1k from in to out, D1 from out to ground, C1=1u from out to ground.
#[test]
fn test_diode_forward_voltage_shunt_clipper() {
    let spice = "Shunt Clipper\nRin in out 1k\nD1 out 0 D1N4148\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    // Create diode device: IS=1e-15, n=1.0
    let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
    let devices = vec![DeviceEntry::new_diode(diode, 0)];

    let mut solver = CircuitSolver::new(kernel, devices, in_idx, out_idx);
    solver.input_conductance = 0.001; // 1/Rin = 1/1k

    // Apply 5V DC step for 1000 samples (allow ample settling time)
    let num_samples = 1000;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        output[i] = solver.process_sample(5.0);
    }

    let final_output = output[num_samples - 1];

    // Diode forward voltage should limit output.
    // With IS=1e-15, n=1.0 and 5V input through 1k, the diode forward voltage
    // is V_d = n*Vt * ln(I/Is) where I ~ (5-Vd)/1k ~ 4.3mA.
    // This gives V_d ~ 0.026 * ln(4.3e-3 / 1e-15) ~ 0.74V.
    // The Thevenin model and trapezoidal discretization may shift this slightly.
    assert!(
        final_output > 0.5,
        "Final output {:.4}V should be above 0.5V (diode is conducting)",
        final_output
    );
    assert!(
        final_output < 1.0,
        "Final output {:.4}V should be below 1.0V (diode clips)",
        final_output
    );

    // Output should generally increase over time as the capacitor charges.
    // We verify this by checking that the average output in the second half
    // is higher than (or equal to) the first quarter average.
    let avg_first_quarter: f64 = output[0..250].iter().sum::<f64>() / 250.0;
    let avg_second_half: f64 = output[500..1000].iter().sum::<f64>() / 500.0;
    assert!(
        avg_second_half >= avg_first_quarter,
        "Output should increase over time: avg first quarter = {:.4}, avg second half = {:.4}",
        avg_first_quarter, avg_second_half
    );

    // The last 50 samples should have approximately settled.
    // With NR clamping and trapezoidal rule, some residual oscillation is acceptable.
    let last_50 = &output[950..1000];
    let min_last = last_50.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_last = last_50.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    assert!(
        (max_last - min_last) < 0.15,
        "Last 50 samples should be approximately settled: min={:.4}, max={:.4}, range={:.4}",
        min_last, max_last, max_last - min_last
    );
}

// ============================================================================
// Test 4: Diode Logarithmic Voltage Scaling
// ============================================================================

/// Verify that the diode forward voltage scales logarithmically with
/// input current (i.e., input voltage through a resistor).
///
/// V_diode = n*Vt * ln(I/Is + 1), so doubling the current adds only
/// n*Vt*ln(2) ~ 18mV to the diode voltage.
#[test]
fn test_diode_logarithmic_voltage_scaling() {
    let spice = "Shunt Clipper\nRin in out 1k\nD1 out 0 D1N4148\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";

    let run_to_steady_state = |input_v: f64| -> f64 {
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();
        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];
        let in_idx = *mna.node_map.get("in").unwrap() - 1;
        let out_idx = *mna.node_map.get("out").unwrap() - 1;
        let mut solver = CircuitSolver::new(kernel, devices, in_idx, out_idx);
        solver.input_conductance = 0.001;
        let mut last = 0.0;
        for _ in 0..500 {
            last = solver.process_sample(input_v);
        }
        last
    };

    let v_out_1v = run_to_steady_state(1.0);
    let v_out_2v = run_to_steady_state(2.0);
    let v_out_5v = run_to_steady_state(5.0);

    // Basic sanity: diode voltage should be positive for positive input
    assert!(
        v_out_1v > 0.4,
        "v_out at 1V input = {:.4}V, expected > 0.4V",
        v_out_1v
    );

    // Diode should limit even at 5V input
    assert!(
        v_out_5v < 1.0,
        "v_out at 5V input = {:.4}V, expected < 1.0V (logarithmic, not linear)",
        v_out_5v
    );

    // The difference between 5V and 1V input should be small (logarithmic scaling)
    let delta = v_out_5v - v_out_1v;
    assert!(
        delta < 0.2,
        "v_out(5V) - v_out(1V) = {:.4}V, expected < 0.2V (logarithmic, not linear)",
        delta
    );

    // Verify non-linear scaling: the increment from 1V to 2V should be
    // much smaller than v_out_2v itself (not linear)
    let increment_1_to_2 = v_out_2v - v_out_1v;
    assert!(
        increment_1_to_2 < v_out_2v * 0.5,
        "Increment from 1V to 2V ({:.4}V) should be much less than v_out_2v ({:.4}V)",
        increment_1_to_2, v_out_2v
    );
}

// ============================================================================
// Test 5: Multi-Device Two Diodes Converges
// ============================================================================

/// Verify that a circuit with two series diodes converges and produces
/// bounded, finite output.
///
/// Circuit: Rin to ground, D1 from in to mid, D2 from mid to out,
/// R1 from mid to ground, C1 from out to ground.
#[test]
fn test_multi_device_two_diodes_converges() {
    let spice = "Two Diodes\nRin in 0 1k\nD1 in mid D1N4148\nD2 mid out D1N4148\nR1 mid 0 10k\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    assert_eq!(
        kernel.m, 2,
        "Two diodes should produce m=2, got m={}",
        kernel.m
    );

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let diode1 = DiodeShockley::new_room_temp(1e-15, 1.0);
    let diode2 = DiodeShockley::new_room_temp(1e-15, 1.0);
    let devices = vec![
        DeviceEntry::new_diode(diode1, 0),
        DeviceEntry::new_diode(diode2, 1),
    ];

    let mut solver = CircuitSolver::new(kernel, devices, in_idx, out_idx);
    solver.input_conductance = 0.001; // 1/Rin = 1/1k

    // Process 200 samples of 1V DC step
    let num_samples = 200;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        output[i] = solver.process_sample(1.0);
    }

    // All outputs must be finite
    for (i, &v) in output.iter().enumerate() {
        assert!(
            v.is_finite(),
            "Output[{}] = {} is not finite",
            i, v
        );
    }

    // Final output should be positive (positive input driving through diodes)
    let final_output = output[num_samples - 1];
    assert!(
        final_output > 0.0,
        "Final output should be positive for 1V input, got {:.6}",
        final_output
    );

    // Final output should be less than the input (diodes limit voltage)
    assert!(
        final_output < 1.0,
        "Final output {:.4}V should be less than 1V input (diodes limit)",
        final_output
    );
}

// ============================================================================
// Test 6: Multi-Device Three Devices Converges
// ============================================================================

/// Verify that a circuit with three series diodes converges.
///
/// Three diodes in series from in through m1, m2 to out, with resistive
/// loads and a capacitor at the output.
#[test]
fn test_multi_device_three_devices_converges() {
    let spice = "Three Diodes\nRin in 0 1k\nD1 in m1 D1N4148\nD2 m1 m2 D1N4148\nD3 m2 out D1N4148\nR1 m1 0 10k\nR2 m2 0 10k\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    assert_eq!(
        kernel.m, 3,
        "Three diodes should produce m=3, got m={}",
        kernel.m
    );

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let diode1 = DiodeShockley::new_room_temp(1e-15, 1.0);
    let diode2 = DiodeShockley::new_room_temp(1e-15, 1.0);
    let diode3 = DiodeShockley::new_room_temp(1e-15, 1.0);
    let devices = vec![
        DeviceEntry::new_diode(diode1, 0),
        DeviceEntry::new_diode(diode2, 1),
        DeviceEntry::new_diode(diode3, 2),
    ];

    let mut solver = CircuitSolver::new(kernel, devices, in_idx, out_idx);
    solver.input_conductance = 0.001;

    // Process 200 samples of 1V DC step
    let num_samples = 200;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        output[i] = solver.process_sample(1.0);
    }

    // All outputs must be finite
    for (i, &v) in output.iter().enumerate() {
        assert!(
            v.is_finite(),
            "Output[{}] = {} is not finite",
            i, v
        );
    }

    // Output should be bounded: positive and less than input
    let final_output = output[num_samples - 1];
    assert!(
        final_output >= 0.0,
        "Final output should be non-negative, got {:.6}",
        final_output
    );
    assert!(
        final_output < 1.0,
        "Final output {:.4}V should be less than 1V (three diodes limit)",
        final_output
    );
}

// ============================================================================
// Test 7: BJT Common Emitter Bias Point
// ============================================================================

/// Verify that a BJT common-emitter amplifier produces finite, bounded
/// output when biased into the active region.
///
/// Circuit: Q1 (c, b, e), Rc=1k from c to ground, Rb=100k from b to ground,
/// Re=100 from e to ground.
#[test]
fn test_bjt_common_emitter_bias_point() {
    let spice = "BJT CE\nQ1 c b e 2N2222\nRc c 0 1k\nRb b 0 100k\nRe e 0 100\n.model 2N2222 NPN(IS=1e-15 BF=200)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    assert_eq!(
        kernel.m, 2,
        "One BJT should produce m=2, got m={}",
        kernel.m
    );

    let b_idx = *mna.node_map.get("b").unwrap() - 1;
    let c_idx = *mna.node_map.get("c").unwrap() - 1;

    let bjt = BjtEbersMoll::new_room_temp(1e-15, 200.0, 3.0, melange_devices::BjtPolarity::Npn);
    let devices = vec![DeviceEntry::new_bjt(bjt, 0)];

    let mut solver = CircuitSolver::new(kernel, devices, b_idx, c_idx);
    solver.input_conductance = 1.0 / 100000.0; // Rb = 100k

    // Apply 0.7V DC bias for 200 samples to forward-bias the base
    let num_samples = 200;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        output[i] = solver.process_sample(0.7);
    }

    // All outputs must be finite
    for (i, &v) in output.iter().enumerate() {
        assert!(
            v.is_finite(),
            "Output[{}] = {} is not finite",
            i, v
        );
    }

    // Collector voltage should be positive (resistor to ground,
    // current flowing through Rc creates a voltage drop)
    let final_output = output[num_samples - 1];
    assert!(
        final_output.is_finite(),
        "Final collector voltage must be finite, got {}",
        final_output
    );

    // Output should be bounded within reasonable range
    assert!(
        final_output.abs() < 5.0,
        "Collector voltage {:.4}V should be bounded < 5V",
        final_output
    );
}

// ============================================================================
// Test 8: Purely Resistive Circuit With Cap Stable
// ============================================================================

/// Verify that adding a small capacitor stabilizes a nonlinear circuit.
///
/// Compare a diode circuit without any capacitor to one with a 100pF cap.
/// The version with the capacitor should have smaller peak-to-peak variation
/// in the last 20 samples (more stable steady state).
#[test]
fn test_purely_resistive_circuit_with_cap_stable() {
    let spice_no_cap = "No Cap\nRin in 0 1k\nD1 in out D1N4148\nR1 out 0 1k\n.model D1N4148 D(IS=1e-15)\n";
    let spice_with_cap = "With Cap\nRin in 0 1k\nD1 in out D1N4148\nR1 out 0 1k\nC1 out 0 100p\n.model D1N4148 D(IS=1e-15)\n";

    // Run without cap
    let run_circuit = |spice: &str| -> Vec<f64> {
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();
        let in_idx = *mna.node_map.get("in").unwrap() - 1;
        let out_idx = *mna.node_map.get("out").unwrap() - 1;
        let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
        let devices = vec![DeviceEntry::new_diode(diode, 0)];
        let mut solver = CircuitSolver::new(kernel, devices, in_idx, out_idx);
        solver.input_conductance = 0.001;
        let mut output = vec![0.0; 100];
        for i in 0..100 {
            output[i] = solver.process_sample(1.0);
        }
        output
    };

    let output_no_cap = run_circuit(spice_no_cap);
    let output_with_cap = run_circuit(spice_with_cap);

    // Compute peak-to-peak variation in last 20 samples
    let pp_variation = |output: &[f64]| -> f64 {
        let last20 = &output[80..100];
        let min_v = last20.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_v = last20.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        max_v - min_v
    };

    let pp_no_cap = pp_variation(&output_no_cap);
    let pp_with_cap = pp_variation(&output_with_cap);

    // With cap: should have smaller or equal peak-to-peak variation
    assert!(
        pp_with_cap <= pp_no_cap + 1e-10,
        "With cap pp={:.6}, without cap pp={:.6}. Cap should stabilize output.",
        pp_with_cap, pp_no_cap
    );

    // Both should produce finite output
    assert!(
        output_no_cap.iter().all(|v| v.is_finite()),
        "No-cap outputs should all be finite"
    );
    assert!(
        output_with_cap.iter().all(|v| v.is_finite()),
        "With-cap outputs should all be finite"
    );
}

// ============================================================================
// Test 9: Solver Output Never Exceeds 10V For Any Input
// ============================================================================

/// Stress test: feed extreme inputs to a diode clipper circuit and verify
/// that all outputs remain finite and within [-10, 10]V.
///
/// The solver internally clamps output to [-10, 10]V for audio safety,
/// so this test verifies that the safety mechanism works correctly.
#[test]
fn test_solver_output_never_exceeds_10v_for_any_input() {
    let spice = "Diode Clipper\nRin in out 1k\nD1 out 0 D1N4148\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
    let devices = vec![DeviceEntry::new_diode(diode, 0)];

    let mut solver = CircuitSolver::new(kernel, devices, in_idx, out_idx);
    solver.input_conductance = 0.001;

    let extreme_inputs = [0.0, 1.0, -1.0, 5.0, -5.0, 10.0, -10.0, 100.0, -100.0, 0.001, -0.001];

    for &input_v in &extreme_inputs {
        // Process 50 samples at each input level
        for sample_idx in 0..50 {
            let output = solver.process_sample(input_v);
            assert!(
                output.is_finite(),
                "Output must be finite for input={}, sample={}, got {}",
                input_v, sample_idx, output
            );
            assert!(
                output.abs() <= 10.0,
                "Output must be within [-10, 10]V for input={}, sample={}, got {:.4}",
                input_v, sample_idx, output
            );
        }
    }
}

// ============================================================================
// Test 10: Zero Input Produces Zero Output
// ============================================================================

/// Verify that a circuit with no DC bias produces zero (or near-zero)
/// output when driven with zero input.
///
/// With zero input and no stored energy, the diode sees no forward bias
/// and the output should remain at zero.
#[test]
fn test_zero_input_produces_zero_output() {
    let spice = "Zero Test\nRin in 0 1k\nD1 in out D1N4148\nC1 out 0 1u\n.model D1N4148 D(IS=1e-15)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
    let devices = vec![DeviceEntry::new_diode(diode, 0)];

    let mut solver = CircuitSolver::new(kernel, devices, in_idx, out_idx);
    solver.input_conductance = 0.001;

    // Process 100 samples of 0V input
    for i in 0..100 {
        let output = solver.process_sample(0.0);
        assert!(
            output.abs() < 1e-10,
            "Zero input should produce zero output, but sample {} = {:.2e}",
            i, output
        );
    }
}
