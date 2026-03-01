//! Minimal end-to-end test - Parse netlist → Build solver → Process samples
//!
//! This test verifies the basic workflow works without requiring ngspice.

use melange_devices::diode::DiodeShockley;
use melange_solver::solver::DeviceEntry;

/// Simple RC lowpass test circuit.
/// R1 and C1 in parallel from input node to ground.
/// The Thevenin input model requires a ground-referenced source impedance.
const RC_LOWPASS_SPICE: &str = r#"RC Circuit
R1 in 0 1k
C1 in 0 1u
"#;

/// Diode clipper test circuit (with cap to prevent singular matrix)
const DIODE_CLIPPER_SPICE: &str = r#"Diode Clipper
Rin in 0 1k
D1 in out D1N4148
C1 out 0 1u
.model D1N4148 D(IS=2.52e-9 N=1.752)
"#;

#[test]
fn test_parse_rc_lowpass() {
    let netlist = melange_solver::parser::Netlist::parse(RC_LOWPASS_SPICE).unwrap();

    // Should have 2 elements: R1, C1
    assert_eq!(netlist.elements.len(), 2, "Should parse 2 elements");

    // Check for resistor
    let has_r1 = netlist.elements.iter().any(|e| {
        if let melange_solver::parser::Element::Resistor { name, .. } = e {
            name == "R1"
        } else {
            false
        }
    });
    assert!(has_r1, "Should have R1 resistor");
}

#[test]
fn test_build_mna_rc_lowpass() {
    let netlist = melange_solver::parser::Netlist::parse(RC_LOWPASS_SPICE).unwrap();
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).unwrap();

    // Single-node RC (R1 in 0, C1 in 0): one non-ground node "in"
    assert!(mna.n >= 1, "Should have at least 1 non-ground node");
}

#[test]
fn test_create_dk_kernel_rc() {
    let netlist = melange_solver::parser::Netlist::parse(RC_LOWPASS_SPICE).unwrap();
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = melange_solver::dk::DkKernel::from_mna(&mna, 48000.0).unwrap();

    // Linear circuit should have m=0 (no nonlinear devices)
    assert_eq!(kernel.m, 0, "RC circuit should have no nonlinear ports");
    assert!(kernel.n >= 1, "Should have at least 1 node");
}

/// End-to-end RC lowpass: verify output converges to 1V with correct dynamics.
#[test]
fn test_full_solver_rc_lowpass() {
    use melange_solver::solver::LinearSolver;

    let netlist = melange_solver::parser::Netlist::parse(RC_LOWPASS_SPICE).unwrap();
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = melange_solver::dk::DkKernel::from_mna(&mna, 44100.0).unwrap();

    // Single-node RC: input and output are both node 0
    let mut solver = LinearSolver::new(kernel, 0, 0);
    solver.input_conductance = 0.001; // G_in = 1/R1 = 1/1k

    // Process 500 samples of 1V DC step
    let num_samples = 500;
    let mut output = vec![0.0; num_samples];

    for i in 0..num_samples {
        output[i] = solver.process_sample(1.0);
    }

    // After ~11 time constants, output should converge to ~1V
    let final_output = output[num_samples - 1];
    assert!((final_output - 1.0).abs() < 0.05,
        "RC lowpass should converge to 1V, got {:.4}V", final_output);

    // Output should be monotonically increasing
    for i in 1..num_samples {
        assert!(output[i] >= output[i-1] - 1e-10,
            "Output should be monotonic: output[{}]={:.6} < output[{}]={:.6}",
            i, output[i], i-1, output[i-1]);
    }

    // First sample should be positive (step response starts rising)
    assert!(output[0] > 0.0, "First sample should be positive, got {:.6}", output[0]);
}

#[test]
fn test_diode_clipper_has_nonlinearity() {
    let netlist = melange_solver::parser::Netlist::parse(DIODE_CLIPPER_SPICE).unwrap();
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = melange_solver::dk::DkKernel::from_mna(&mna, 48000.0).unwrap();
    
    // Should have 1 nonlinear port (the diode)
    assert_eq!(kernel.m, 1, "Diode clipper should have 1 nonlinear port");
}

/// End-to-end diode clipper: verify clipping behavior.
#[test]
fn test_diode_clipper_solver() {
    let netlist = melange_solver::parser::Netlist::parse(DIODE_CLIPPER_SPICE).unwrap();
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = melange_solver::dk::DkKernel::from_mna(&mna, 44100.0).unwrap();

    // Create diode device
    let diode = DiodeShockley::new(1e-15, 0.02585, 1.0);
    let devices = vec![DeviceEntry::new_diode(diode, 0)];

    let input_node = 0;
    let output_node = 1;
    let mut solver = melange_solver::solver::CircuitSolver::new(kernel, devices, input_node, output_node);
    solver.input_conductance = 0.001; // Rin = 1k

    // Small signal: output should be proportional to input (not clipped)
    let small_out = solver.process_sample(0.01);
    assert!(small_out.abs() < 0.1, "Small signal output should be small, got {}", small_out);

    // Large signal: output should be bounded by diode forward voltage
    solver.reset();
    let mut outputs = vec![];
    for _ in 0..200 {
        let output = solver.process_sample(2.0);
        outputs.push(output);
        assert!(output.is_finite(), "Output should be finite");
    }

    let final_output = *outputs.last().unwrap();
    assert!(final_output > 0.0, "Output should be positive for positive input");
    assert!(final_output < 1.5,
        "Diode clipper should limit output below ~1V, got {:.4}V", final_output);
}

