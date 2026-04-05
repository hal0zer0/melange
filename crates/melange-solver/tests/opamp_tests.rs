//! Op-amp integration tests.
//!
//! Tests verify that the ideal op-amp model (VCCS stamping in G matrix)
//! produces correct closed-loop gains for standard amplifier topologies.
//!
//! The op-amp is modeled as a voltage-controlled current source:
//!   G[out, n_plus] += Gm     (where Gm = AOL / ROUT)
//!   G[out, n_minus] -= Gm
//!   G[out, out] += Go        (where Go = 1 / ROUT)
//!
//! This is purely linear -- no nonlinear dimensions (M stays 0 for pure op-amp circuits).

use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::solver::LinearSolver;

/// Helper: build a LinearSolver from a SPICE netlist string and return the output node index.
fn build_linear_solver(spice: &str, sample_rate: f64) -> (LinearSolver, usize, usize) {
    let netlist = Netlist::parse(spice).expect("Failed to parse netlist");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("Failed to build MNA");
    let node_map = mna.node_map.clone();

    let input_idx = *node_map.get("in").expect("Input node 'in' not found");
    let output_idx = *node_map.get("out").expect("Output node 'out' not found");
    assert!(input_idx > 0, "Input node cannot be ground");
    assert!(output_idx > 0, "Output node cannot be ground");
    let input_node_0 = input_idx - 1;
    let output_node_0 = output_idx - 1;

    // Stamp input conductance BEFORE building kernel (critical!)
    let input_conductance = 1.0; // 1 ohm Thevenin source
    mna.stamp_input_conductance(input_node_0, input_conductance);

    let kernel = DkKernel::from_mna(&mna, sample_rate).expect("Failed to build DK kernel");
    let solver = LinearSolver::new(kernel, input_node_0, output_node_0);

    (solver, input_node_0, output_node_0)
}

/// Helper: measure DC gain by processing a step input and returning peak absolute output
/// (preserving sign). With a DC blocker in the solver, the final sample drools toward zero,
/// so we track the peak instead.
fn measure_dc_gain(solver: &mut LinearSolver, input_voltage: f64, num_samples: usize) -> f64 {
    let mut peak_pos = 0.0f64;
    let mut peak_neg = 0.0f64;
    for _ in 0..num_samples {
        let out = solver.process_sample(input_voltage);
        if out > peak_pos {
            peak_pos = out;
        }
        if out < peak_neg {
            peak_neg = out;
        }
    }
    let peak = if peak_pos.abs() > peak_neg.abs() {
        peak_pos
    } else {
        peak_neg
    };
    peak / input_voltage
}

// =============================================================================
// Inverting Amplifier: Gain = -R2/R1
// =============================================================================

#[test]
fn test_opamp_inverting_amplifier_gain() {
    let spice = r#"Inverting Amplifier
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out opamp
.model opamp OA(AOL=200000)
"#;
    let (mut solver, _, _) = build_linear_solver(spice, 44100.0);

    let gain = measure_dc_gain(&mut solver, 0.1, 2000);

    assert!(
        (gain - (-10.0)).abs() < 0.5,
        "Inverting amplifier gain should be ~-10, got {:.3}",
        gain
    );
}

#[test]
fn test_opamp_inverting_amplifier_unity_gain() {
    let spice = r#"Unity Inverting
R1 in inv 10k
R2 inv out 10k
C1 out 0 100n
U1 0 inv out opamp
.model opamp OA(AOL=200000)
"#;
    let (mut solver, _, _) = build_linear_solver(spice, 44100.0);

    let gain = measure_dc_gain(&mut solver, 0.5, 2000);

    assert!(
        (gain - (-1.0)).abs() < 0.1,
        "Unity inverting gain should be ~-1, got {:.3}",
        gain
    );
}

// =============================================================================
// Non-Inverting Amplifier: Gain = 1 + R2/R1
// =============================================================================

#[test]
fn test_opamp_noninverting_amplifier_gain() {
    let spice = r#"Non-Inverting Amplifier
R1 inv 0 10k
R2 inv out 100k
C1 out 0 100n
U1 in inv out opamp
.model opamp OA(AOL=200000)
"#;
    let (mut solver, _, _) = build_linear_solver(spice, 44100.0);

    let gain = measure_dc_gain(&mut solver, 0.1, 2000);

    assert!(
        (gain - 11.0).abs() < 0.5,
        "Non-inverting amplifier gain should be ~11, got {:.3}",
        gain
    );
}

// =============================================================================
// Voltage Follower (Unity Gain Buffer): Gain = 1
// =============================================================================

#[test]
fn test_opamp_voltage_follower() {
    let spice = r#"Voltage Follower
C1 out 0 100n
U1 in out out opamp
.model opamp OA(AOL=200000)
"#;
    let (mut solver, _, _) = build_linear_solver(spice, 44100.0);

    let gain = measure_dc_gain(&mut solver, 0.5, 2000);

    assert!(
        (gain - 1.0).abs() < 0.05,
        "Voltage follower gain should be ~1.0, got {:.4}",
        gain
    );
}

// =============================================================================
// Summing Amplifier
// =============================================================================

#[test]
fn test_opamp_summing_amplifier() {
    let spice = r#"Summing Amplifier
R1 in inv 10k
R2 inv out 10k
Rload out 0 100k
C1 out 0 100n
U1 0 inv out opamp
.model opamp OA(AOL=200000)
"#;
    let (mut solver, _, _) = build_linear_solver(spice, 44100.0);

    let gain = measure_dc_gain(&mut solver, 0.3, 2000);

    assert!(
        (gain - (-1.0)).abs() < 0.1,
        "Summing amplifier gain should be ~-1, got {:.3}",
        gain
    );
}

// =============================================================================
// Codegen tests
// =============================================================================

#[test]
fn test_opamp_codegen_inverting_amplifier() {
    use melange_solver::codegen::{CodeGenerator, CodegenConfig};

    let spice = r#"Inverting Amplifier
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out opamp
.model opamp OA(AOL=200000)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let node_map = mna.node_map.clone();

    let input_node_0 = node_map["in"] - 1;
    let output_node_0 = node_map["out"] - 1;

    mna.stamp_input_conductance(input_node_0, 1.0);
    let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

    let config = CodegenConfig {
        circuit_name: "inverting_opamp".to_string(),
        sample_rate: 44100.0,
        input_node: input_node_0,
        output_nodes: vec![output_node_0],
        ..CodegenConfig::default()
    };

    let generator = CodeGenerator::new(config);
    let result = generator.generate(&kernel, &mna, &netlist);
    assert!(result.is_ok(), "Codegen failed: {:?}", result.err());

    let generated = result.unwrap();
    assert_eq!(generated.m, 0, "M should be 0 for linear op-amp circuit");
    assert!(generated.code.contains("pub const M: usize = 0"));
}

#[test]
fn test_opamp_with_diode_codegen() {
    use melange_solver::codegen::{CodeGenerator, CodegenConfig};

    let spice = r#"Op-Amp Then Diode Clipper
R1 in inv 10k
R2 inv opout 100k
C1 opout 0 100n
U1 0 inv opout opamp
Rcouple opout out 1k
D1 out 0 D1N4148
Rload out 0 10k
C2 out 0 100n
.model opamp OA(AOL=10000)
.model D1N4148 D(IS=2.52e-9 N=1.752)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let node_map = mna.node_map.clone();

    let input_node_0 = node_map["in"] - 1;
    let output_node_0 = node_map["out"] - 1;

    mna.stamp_input_conductance(input_node_0, 1.0);
    let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

    let config = CodegenConfig {
        circuit_name: "opamp_then_diode".to_string(),
        sample_rate: 44100.0,
        input_node: input_node_0,
        output_nodes: vec![output_node_0],
        ..CodegenConfig::default()
    };

    let generator = CodeGenerator::new(config);
    let result = generator.generate(&kernel, &mna, &netlist);
    assert!(result.is_ok(), "Codegen failed: {:?}", result.err());

    let generated = result.unwrap();
    assert_eq!(generated.m, 1, "M should be 1 for one diode");
}

// =============================================================================
// AC signal processing
// =============================================================================

#[test]
fn test_opamp_inverting_ac_signal() {
    let spice = r#"Inverting AC
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out opamp
.model opamp OA(AOL=200000)
"#;
    let (mut solver, _, _) = build_linear_solver(spice, 44100.0);

    let sample_rate = 44100.0;
    let freq = 1000.0;
    let amplitude = 0.1;

    for i in 0..500 {
        let t = i as f64 / sample_rate;
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
        solver.process_sample(input);
    }

    let samples_per_cycle = (sample_rate / freq) as usize;
    let mut max_output = f64::NEG_INFINITY;
    let mut min_output = f64::INFINITY;

    for i in 0..samples_per_cycle {
        let t = (500 + i) as f64 / sample_rate;
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
        let output = solver.process_sample(input);
        max_output = max_output.max(output);
        min_output = min_output.min(output);
    }

    let output_amplitude = (max_output - min_output) / 2.0;
    let expected_amplitude = amplitude * 10.0;

    assert!(
        (output_amplitude - expected_amplitude).abs() / expected_amplitude < 0.15,
        "Output amplitude should be ~{:.3}V, got {:.3}V",
        expected_amplitude,
        output_amplitude
    );
}

// =============================================================================
// Model parameter tests
// =============================================================================

#[test]
fn test_opamp_default_model_params() {
    let spice = r#"Default Params
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out myopamp
.model myopamp OA()
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.opamps.len(), 1);
    assert_eq!(mna.opamps[0].aol, 200_000.0);
    assert_eq!(mna.opamps[0].r_out, 1.0);
}

#[test]
fn test_opamp_custom_rout() {
    let spice = r#"Custom ROUT
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out myoa
.model myoa OA(AOL=100000 ROUT=75)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.opamps.len(), 1);
    assert_eq!(mna.opamps[0].aol, 100_000.0);
    assert_eq!(mna.opamps[0].r_out, 75.0);
}

#[test]
fn test_opamp_gain_improves_with_higher_aol() {
    let make_spice = |aol: f64| -> String {
        format!(
            r#"Inverting
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out oa
.model oa OA(AOL={})
"#,
            aol
        )
    };

    let (mut solver_low, _, _) = build_linear_solver(&make_spice(1000.0), 44100.0);
    let gain_low = measure_dc_gain(&mut solver_low, 0.1, 2000);

    let (mut solver_high, _, _) = build_linear_solver(&make_spice(200_000.0), 44100.0);
    let gain_high = measure_dc_gain(&mut solver_high, 0.1, 2000);

    let error_low = (gain_low - (-10.0)).abs();
    let error_high = (gain_high - (-10.0)).abs();

    assert!(
        error_high < error_low,
        "Higher AOL should give more accurate gain: error_low={:.4}, error_high={:.4}",
        error_low,
        error_high
    );
}

// =============================================================================
// VCC/VEE asymmetric supply rail tests
// =============================================================================

#[test]
fn test_opamp_vcc_vee_parsed() {
    let spice = r#"VCC/VEE Test
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out oa
.model oa OA(AOL=200000 VCC=9 VEE=0)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.opamps.len(), 1);
    assert_eq!(mna.opamps[0].vcc, 9.0, "VCC should be 9.0");
    assert_eq!(mna.opamps[0].vee, 0.0, "VEE should be 0.0");
}

#[test]
fn test_opamp_vcc_vee_negative_rail() {
    let spice = r#"Negative Rail Test
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out oa
.model oa OA(AOL=200000 VCC=18 VEE=-9)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert_eq!(mna.opamps[0].vcc, 18.0, "VCC should be 18.0");
    assert_eq!(mna.opamps[0].vee, -9.0, "VEE should be -9.0");
}

#[test]
fn test_opamp_vsat_backward_compat() {
    let spice = r#"VSAT Backward Compat
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out oa
.model oa OA(AOL=200000 VSAT=13)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // VSAT=13 should resolve to VCC=13, VEE=-13
    assert_eq!(mna.opamps[0].vcc, 13.0, "VCC should be 13.0 from VSAT");
    assert_eq!(mna.opamps[0].vee, -13.0, "VEE should be -13.0 from VSAT");
}

#[test]
fn test_opamp_vcc_vee_overrides_vsat() {
    let spice = r#"VCC/VEE Override
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out oa
.model oa OA(AOL=200000 VSAT=13 VCC=9 VEE=0)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // VCC/VEE should take priority over VSAT
    assert_eq!(mna.opamps[0].vcc, 9.0, "VCC should override VSAT");
    assert_eq!(mna.opamps[0].vee, 0.0, "VEE should override VSAT");
}

#[test]
fn test_opamp_gbw_auto_default_preserves_symmetric() {
    let spice = r#"GBW Auto Default
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out oa
.model oa OA(AOL=200000 GBW=3e6)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // GBW finite + no explicit rails → auto-default ±13V
    assert_eq!(mna.opamps[0].vcc, 13.0, "VCC should auto-default to 13V");
    assert_eq!(mna.opamps[0].vee, -13.0, "VEE should auto-default to -13V");
}

#[test]
fn test_opamp_no_rails_no_clamping() {
    let spice = r#"No Rails
R1 in inv 10k
R2 inv out 100k
C1 out 0 100n
U1 0 inv out oa
.model oa OA(AOL=200000)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // No VSAT, no VCC/VEE, no GBW → no clamping
    assert!(mna.opamps[0].vcc.is_infinite(), "VCC should be infinity");
    assert!(mna.opamps[0].vee.is_infinite(), "VEE should be -infinity");
}

#[test]
fn test_opamp_vcc_vee_codegen_asymmetric_clamp() {
    use melange_solver::codegen::{CodeGenerator, CodegenConfig};

    // Op-amp with asymmetric rails + diode (separated by coupling R to avoid +K diagonal)
    let spice = r#"Asymmetric Clamp Codegen
R1 in inv 10k
R2 inv opout 100k
C1 opout 0 100n
U1 0 inv opout oa
Rcouple opout out 1k
D1 out 0 D1N4148
Rload out 0 10k
C2 out 0 100n
.model oa OA(AOL=200000 VCC=9 VEE=0)
.model D1N4148 D(IS=2.52e-9 N=1.752)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let node_map = mna.node_map.clone();

    let input_node_0 = node_map["in"] - 1;
    let output_node_0 = node_map["out"] - 1;

    mna.stamp_input_conductance(input_node_0, 1.0);
    let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

    let config = CodegenConfig {
        circuit_name: "vcc_vee_test".to_string(),
        sample_rate: 44100.0,
        input_node: input_node_0,
        output_nodes: vec![output_node_0],
        ..CodegenConfig::default()
    };

    let generator = CodeGenerator::new(config);
    let result = generator.generate(&kernel, &mna, &netlist);
    assert!(result.is_ok(), "Codegen failed: {:?}", result.err());

    let generated = result.unwrap();
    // Generated code should contain asymmetric clamp with VCC=9 and VEE=0
    let has_clamp = generated.code.lines().any(|line| {
        line.contains(".clamp(") && line.contains("9.0") && line.contains("0.0")
    });
    assert!(has_clamp, "Generated code should contain .clamp(0.0..., 9.0...) for VEE=0/VCC=9");
}

#[test]
fn test_opamp_dk_path_has_clamping() {
    use melange_solver::codegen::{CodeGenerator, CodegenConfig};

    // Pure opamp with diode (M=1, DK Schur path) should have clamping in generated code
    let spice = r#"DK Path Clamping
R1 in inv 10k
R2 inv opout 100k
C1 opout 0 100n
U1 0 inv opout oa
Rcouple opout dout 1k
D1 dout 0 D1N4148
Rload dout 0 10k
C2 dout 0 100n
.model oa OA(AOL=200000 VSAT=5)
.model D1N4148 D(IS=2.52e-9 N=1.752)
"#;
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let node_map = mna.node_map.clone();

    let input_node_0 = node_map["in"] - 1;
    let output_node_0 = node_map["dout"] - 1;

    mna.stamp_input_conductance(input_node_0, 1.0);
    let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();

    let config = CodegenConfig {
        circuit_name: "dk_vsat_test".to_string(),
        sample_rate: 44100.0,
        input_node: input_node_0,
        output_nodes: vec![output_node_0],
        ..CodegenConfig::default()
    };

    let generator = CodeGenerator::new(config);
    let result = generator.generate(&kernel, &mna, &netlist);
    assert!(result.is_ok(), "Codegen failed: {:?}", result.err());

    let generated = result.unwrap();
    // M=1 should route to DK path, and the generated code should have VSAT clamping
    assert_eq!(generated.m, 1, "Should have M=1 for DK path");
    assert!(
        generated.code.contains("Op-amp output voltage clamping"),
        "DK path should have op-amp clamping comment"
    );
}
