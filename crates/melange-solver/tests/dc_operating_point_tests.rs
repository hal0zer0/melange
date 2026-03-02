//! DC Operating Point tests.
//!
//! Verifies that the DC operating point calculation correctly computes
//! linear steady-state node voltages for various circuit topologies.
//!
//! The DC operating point treats capacitors as open circuits and nonlinear
//! devices as open, solving G * v_dc = b_dc (Norton currents from DC sources).
//!
//! Tests cover:
//! - Simple resistive voltage divider (exact analytical result)
//! - RC circuit DC OP (capacitor open at DC)
//! - Circuit with DC voltage source (VCC bias voltages)
//! - Verification that `set_dc_operating_point()` properly initializes `v_prev`
//! - DC OP with input conductance affecting the result

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::codegen::ir::CircuitIR;
use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
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

fn build_ir(spice: &str, config: &CodegenConfig) -> CircuitIR {
    let (netlist, mna, kernel) = build_pipeline(spice);
    CircuitIR::from_kernel(&kernel, &mna, &netlist, config).unwrap()
}

fn default_config() -> CodegenConfig {
    CodegenConfig {
        circuit_name: "test_circuit".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

fn generate_code(spice: &str, config: &CodegenConfig) -> String {
    let (netlist, mna, kernel) = build_pipeline(spice);
    let codegen = CodeGenerator::new(config.clone());
    let result = codegen.generate(&kernel, &mna, &netlist)
        .expect("code generation failed");
    result.code
}

/// Compile generated code with rustc, panicking with stderr on failure.
fn assert_compiles(code: &str, label: &str) {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static COUNTER: AtomicUsize = AtomicUsize::new(0);
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);

    let tmp_dir = std::env::temp_dir();
    let src = tmp_dir.join(format!("melange_dcop_test_{}.rs", id));
    let lib = tmp_dir.join(format!("melange_dcop_test_{}.rlib", id));

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

/// Resistive voltage divider: V1=12V through R1=10k, R2=10k to ground.
/// DC OP should give V_out = V1 * R2/(R1+R2) = 6V at the output node.
///
/// Circuit: VCC (12V) -> R1 (10k) -> mid -> R2 (10k) -> GND
/// The voltage source is modeled as a Norton equivalent with VS_CONDUCTANCE=1e6.
const VOLTAGE_DIVIDER_SPICE: &str = "\
Voltage Divider
V1 vcc 0 12
R1 vcc mid 10k
R2 mid 0 10k
C1 mid 0 1n
";

/// RC circuit: R1=10k, C1=100n. At DC, capacitor is open, so
/// the output node voltage equals the input through the resistor path.
const RC_DCOP_SPICE: &str = "\
RC DC OP
R1 in out 10k
C1 out 0 100n
";

/// Circuit with VCC bias: common BJT-style bias network (without BJT, just resistors).
/// VCC=9V, R1=100k, R2=47k to ground.
/// V_base = VCC * R2/(R1+R2) = 9 * 47k/147k ~ 2.878V
const VCC_BIAS_SPICE: &str = "\
VCC Bias Network
V1 vcc 0 9
R1 vcc base 100k
R2 base 0 47k
C1 base 0 1n
";

/// Two voltage sources: VCC=12V and VEE=-12V (dual supply).
/// R1=10k from VCC to mid, R2=10k from mid to VEE.
/// V_mid = (VCC + VEE) / 2 = 0V
const DUAL_SUPPLY_SPICE: &str = "\
Dual Supply
V1 vcc 0 12
V2 vee 0 -12
R1 vcc mid 10k
R2 mid vee 10k
C1 mid 0 1n
";

// ---------------------------------------------------------------------------
// Test 1: Resistive voltage divider DC operating point
// ---------------------------------------------------------------------------

#[test]
fn test_dc_op_voltage_divider() {
    let config = CodegenConfig {
        circuit_name: "voltage_divider".to_string(),
        sample_rate: 44100.0,
        input_node: 0, // not used for DC OP, but needed
        output_node: 1,
        input_resistance: 1e6, // very high so it doesn't affect DC OP
        ..CodegenConfig::default()
    };

    let ir = build_ir(VOLTAGE_DIVIDER_SPICE, &config);

    // There should be DC sources (V1)
    assert!(ir.has_dc_sources, "Circuit with V1 should have DC sources");
    assert!(ir.has_dc_op, "Voltage divider should produce non-zero DC OP");

    // Get node indices
    let (_netlist, mna, _kernel) = build_pipeline(VOLTAGE_DIVIDER_SPICE);
    let vcc_idx = *mna.node_map.get("vcc").expect("vcc node") - 1;
    let mid_idx = *mna.node_map.get("mid").expect("mid node") - 1;

    let dc_op = &ir.dc_operating_point;

    // VCC node should be very close to 12V (voltage source modeled with large conductance)
    assert!(
        (dc_op[vcc_idx] - 12.0).abs() < 0.01,
        "VCC node DC OP should be ~12V, got {:.6}",
        dc_op[vcc_idx]
    );

    // Mid node: V_mid = V1 * R2/(R1+R2) = 12 * 10k/20k = 6V
    // Due to the Norton-equivalent modeling (VS_CONDUCTANCE=1e6), the voltage source
    // has a tiny internal impedance (1e-6 ohms), so the divider should be very accurate.
    assert!(
        (dc_op[mid_idx] - 6.0).abs() < 0.01,
        "Mid node DC OP should be ~6V (R1=R2 divider), got {:.6}",
        dc_op[mid_idx]
    );
}

// ---------------------------------------------------------------------------
// Test 2: RC circuit DC OP (capacitor open at DC)
// ---------------------------------------------------------------------------

#[test]
fn test_dc_op_rc_capacitor_open() {
    // For an RC circuit with no DC voltage source, the DC OP should be all zeros
    // (no DC sources means no steady-state voltage).
    let config = CodegenConfig {
        circuit_name: "rc_dcop".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };

    let ir = build_ir(RC_DCOP_SPICE, &config);

    // No voltage sources, so no DC bias
    assert!(!ir.has_dc_sources, "RC circuit without V source should have no DC sources");

    // DC OP should be all zeros (no sources to bias the circuit)
    for (i, &v) in ir.dc_operating_point.iter().enumerate() {
        assert!(
            v.abs() < 1e-12,
            "DC OP node {} should be 0 with no DC sources, got {:.6e}",
            i, v
        );
    }
}

#[test]
fn test_dc_op_rc_with_vcc() {
    // RC circuit WITH a voltage source: DC OP should show the bias.
    // V1=5V -> R1=10k -> out, C1=100n from out to ground.
    // At DC, capacitor is open, so no current flows through R1.
    // With input conductance stamped, the output node sees VCC through the divider
    // formed by R1 and input resistance.
    let spice = "\
RC with VCC
V1 vcc 0 5
R1 vcc out 10k
C1 out 0 100n
";
    let config = CodegenConfig {
        circuit_name: "rc_vcc".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1e6,
        ..CodegenConfig::default()
    };

    let ir = build_ir(spice, &config);
    let (_, mna, _) = build_pipeline(spice);

    assert!(ir.has_dc_sources, "Should have DC sources");
    assert!(ir.has_dc_op, "Should have non-zero DC OP");

    let vcc_idx = *mna.node_map.get("vcc").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    // VCC should be ~5V
    assert!(
        (ir.dc_operating_point[vcc_idx] - 5.0).abs() < 0.01,
        "VCC should be ~5V, got {:.6}",
        ir.dc_operating_point[vcc_idx]
    );

    // At DC, no current flows through C1 (open), but the G matrix only has R1
    // and input conductance. The output node is connected to VCC through R1 (10k)
    // and to ground through the input conductance (1/1e6 S). So:
    // V_out = VCC * G_vs * G_R1 / (G_R1 + G_in) ~ VCC (since G_in << G_R1)
    // Actually, the DC OP computation uses the G matrix directly. Capacitors are open,
    // so the "out" node only sees R1 connecting it to VCC and maybe input conductance.
    // With VS_CONDUCTANCE=1e6 for the voltage source:
    // V_vcc ~ 5V, and V_out ~ 5V (through R1 with negligible input conductance load)
    assert!(
        (ir.dc_operating_point[out_idx] - 5.0).abs() < 0.1,
        "Output node should be ~5V at DC (cap is open, R1 connects to VCC), got {:.6}",
        ir.dc_operating_point[out_idx]
    );
}

// ---------------------------------------------------------------------------
// Test 3: VCC bias network
// ---------------------------------------------------------------------------

#[test]
fn test_dc_op_vcc_bias_network() {
    let config = CodegenConfig {
        circuit_name: "vcc_bias".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1e6,
        ..CodegenConfig::default()
    };

    let ir = build_ir(VCC_BIAS_SPICE, &config);
    let (_, mna, _) = build_pipeline(VCC_BIAS_SPICE);

    assert!(ir.has_dc_sources);
    assert!(ir.has_dc_op);

    let vcc_idx = *mna.node_map.get("vcc").unwrap() - 1;
    let base_idx = *mna.node_map.get("base").unwrap() - 1;

    // VCC = 9V
    assert!(
        (ir.dc_operating_point[vcc_idx] - 9.0).abs() < 0.01,
        "VCC should be ~9V, got {:.6}",
        ir.dc_operating_point[vcc_idx]
    );

    // V_base = VCC * R2/(R1+R2) = 9 * 47k/147k = 2.8776...V
    let expected_base = 9.0 * 47000.0 / (100000.0 + 47000.0);
    assert!(
        (ir.dc_operating_point[base_idx] - expected_base).abs() < 0.05,
        "Base node should be ~{:.4}V (R1-R2 divider from VCC), got {:.6}",
        expected_base, ir.dc_operating_point[base_idx]
    );
}

// ---------------------------------------------------------------------------
// Test 4: Dual supply cancellation
// ---------------------------------------------------------------------------

#[test]
fn test_dc_op_dual_supply() {
    let config = CodegenConfig {
        circuit_name: "dual_supply".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1e6,
        ..CodegenConfig::default()
    };

    let ir = build_ir(DUAL_SUPPLY_SPICE, &config);
    let (_, mna, _) = build_pipeline(DUAL_SUPPLY_SPICE);

    assert!(ir.has_dc_sources);
    assert!(ir.has_dc_op);

    let vcc_idx = *mna.node_map.get("vcc").unwrap() - 1;
    let vee_idx = *mna.node_map.get("vee").unwrap() - 1;
    let mid_idx = *mna.node_map.get("mid").unwrap() - 1;

    // VCC ~ 12V
    assert!(
        (ir.dc_operating_point[vcc_idx] - 12.0).abs() < 0.01,
        "VCC should be ~12V, got {:.6}",
        ir.dc_operating_point[vcc_idx]
    );

    // VEE ~ -12V
    assert!(
        (ir.dc_operating_point[vee_idx] - (-12.0)).abs() < 0.01,
        "VEE should be ~-12V, got {:.6}",
        ir.dc_operating_point[vee_idx]
    );

    // Mid point: (12 + (-12)) / 2 = 0V (symmetric divider)
    assert!(
        ir.dc_operating_point[mid_idx].abs() < 0.1,
        "Mid node should be ~0V (symmetric dual supply), got {:.6}",
        ir.dc_operating_point[mid_idx]
    );
}

// ---------------------------------------------------------------------------
// Test 5: set_dc_operating_point initializes v_prev in generated code
// ---------------------------------------------------------------------------

#[test]
fn test_dc_op_initializes_v_prev_in_codegen() {
    let config = CodegenConfig {
        circuit_name: "dcop_init".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1e6,
        ..CodegenConfig::default()
    };

    let code = generate_code(VOLTAGE_DIVIDER_SPICE, &config);

    // The generated code should contain DC_OP constant
    assert!(
        code.contains("DC_OP"),
        "Generated code should contain DC_OP constant for circuit with voltage source"
    );

    // v_prev should be initialized to DC_OP in Default impl
    assert!(
        code.contains("v_prev: DC_OP"),
        "v_prev should be initialized to DC_OP in Default impl.\nCode snippet: {}",
        &code[code.find("impl Default").unwrap_or(0)..code.len().min(code.find("impl Default").unwrap_or(0) + 500)]
    );

    // dc_operating_point field should also be initialized to DC_OP
    assert!(
        code.contains("dc_operating_point: DC_OP"),
        "dc_operating_point should be initialized to DC_OP in Default impl"
    );

    // set_dc_operating_point should set both dc_operating_point and v_prev
    assert!(
        code.contains("fn set_dc_operating_point"),
        "Should have set_dc_operating_point method"
    );

    // reset() should restore v_prev from dc_operating_point
    assert!(
        code.contains("self.v_prev = self.dc_operating_point"),
        "reset() should restore v_prev from stored dc_operating_point"
    );
}

#[test]
fn test_dc_op_codegen_compiles_with_vcc() {
    let config = CodegenConfig {
        circuit_name: "dcop_compile".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1e6,
        ..CodegenConfig::default()
    };

    let code = generate_code(VOLTAGE_DIVIDER_SPICE, &config);
    assert_compiles(&code, "voltage_divider_dcop");
}

// ---------------------------------------------------------------------------
// Test 6: DC OP values are correctly embedded in generated code constants
// ---------------------------------------------------------------------------

#[test]
fn test_dc_op_values_embedded_correctly() {
    let config = CodegenConfig {
        circuit_name: "dcop_values".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1e6,
        ..CodegenConfig::default()
    };

    let ir = build_ir(VOLTAGE_DIVIDER_SPICE, &config);
    let code = generate_code(VOLTAGE_DIVIDER_SPICE, &config);

    // The DC_OP constant values should be present in the code
    // and should match the computed IR values
    assert!(code.contains("DC_OP: [f64; N]"), "Should have DC_OP constant declaration");

    // Each DC OP value should appear in the code (as scientific notation)
    for &v in &ir.dc_operating_point {
        if v.abs() > 1e-15 {
            // The value should appear somewhere in the DC_OP array
            // We just check that the code contains a value close to this
            let formatted = format!("{:.17e}", v);
            assert!(
                code.contains(&formatted),
                "DC_OP should contain value {:.6} (formatted as {})",
                v, formatted
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Test 7: No DC OP for circuit without sources
// ---------------------------------------------------------------------------

#[test]
fn test_no_dc_op_without_sources() {
    let config = CodegenConfig {
        circuit_name: "no_sources".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };

    let ir = build_ir(RC_DCOP_SPICE, &config);

    assert!(!ir.has_dc_sources, "RC-only circuit has no DC sources");
    assert!(!ir.has_dc_op, "RC-only circuit should have zero DC OP");

    // Code should NOT have DC_OP constant
    let code = generate_code(RC_DCOP_SPICE, &config);
    assert!(
        !code.contains("pub const DC_OP: [f64"),
        "Code without DC sources should not have DC_OP constant"
    );

    // v_prev should be initialized to zeros instead
    assert!(
        code.contains("v_prev: [0.0; N]"),
        "v_prev should be initialized to zeros when no DC OP"
    );
}

// ---------------------------------------------------------------------------
// Test 8: Input resistance affects DC OP calculation
// ---------------------------------------------------------------------------

#[test]
fn test_dc_op_input_resistance_effect() {
    // The DC OP includes input conductance in the G matrix.
    // With different input_resistance on the output node, the DC OP should change.
    // We stamp G_in on the output node, where it acts as a shunt to ground.
    let spice = "\
Input R effect
V1 vcc 0 5
R1 vcc out 10k
C1 out 0 100n
";

    let (_, mna, _) = build_pipeline(spice);
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    // Use input_node = out_idx (the "out" node) so that input conductance
    // acts as a load on the output, pulling voltage toward ground.
    let config_low_r = CodegenConfig {
        circuit_name: "dcop_low_r".to_string(),
        sample_rate: 44100.0,
        input_node: out_idx,
        output_node: out_idx,
        input_resistance: 100.0, // 100 ohm => G_in = 0.01 S (huge load vs R1's 0.0001 S)
        ..CodegenConfig::default()
    };

    let config_high_r = CodegenConfig {
        circuit_name: "dcop_high_r".to_string(),
        sample_rate: 44100.0,
        input_node: out_idx,
        output_node: out_idx,
        input_resistance: 1e6, // 1M ohm => G_in = 1e-6 S (negligible)
        ..CodegenConfig::default()
    };

    let ir_low = build_ir(spice, &config_low_r);
    let ir_high = build_ir(spice, &config_high_r);

    // Both should have DC OP
    assert!(ir_low.has_dc_op);
    assert!(ir_high.has_dc_op);

    let v_low = ir_low.dc_operating_point[out_idx];
    let v_high = ir_high.dc_operating_point[out_idx];

    // With low input resistance (100 ohm), G_in = 0.01 S is much larger than
    // G_R1 = 1/10k = 0.0001 S, so V_out is pulled heavily toward 0.
    // With high input resistance (1M ohm), G_in = 1e-6 S is negligible,
    // so V_out ~ 5V (VCC through R1 with minimal loading).
    assert!(
        v_high > v_low,
        "Higher input resistance should give higher output voltage: low_r={:.6}, high_r={:.6}",
        v_low, v_high
    );
    assert!(
        (v_low - v_high).abs() > 0.01,
        "DC OP should differ significantly with different input resistance: low_r={:.6}, high_r={:.6}",
        v_low, v_high
    );
}

// ---------------------------------------------------------------------------
// Test 9: DC OP vector length matches node count
// ---------------------------------------------------------------------------

#[test]
fn test_dc_op_vector_length() {
    let config = default_config();

    // 2-node circuit
    let ir2 = build_ir(RC_DCOP_SPICE, &config);
    assert_eq!(
        ir2.dc_operating_point.len(), ir2.topology.n,
        "DC OP length should match node count N={}", ir2.topology.n
    );

    // 3-node circuit (VCC bias)
    let ir3 = build_ir(VCC_BIAS_SPICE, &CodegenConfig {
        circuit_name: "len_test".to_string(),
        input_resistance: 1e6,
        ..config.clone()
    });
    assert_eq!(
        ir3.dc_operating_point.len(), ir3.topology.n,
        "DC OP length should match node count N={}", ir3.topology.n
    );

    // 3-node dual supply
    let ir4 = build_ir(DUAL_SUPPLY_SPICE, &CodegenConfig {
        circuit_name: "len_test2".to_string(),
        input_resistance: 1e6,
        ..config.clone()
    });
    assert_eq!(
        ir4.dc_operating_point.len(), ir4.topology.n,
        "DC OP length should match node count N={}", ir4.topology.n
    );
}

// ---------------------------------------------------------------------------
// Test 10: DC OP all finite
// ---------------------------------------------------------------------------

#[test]
fn test_dc_op_all_finite() {
    let config = CodegenConfig {
        circuit_name: "finite_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1e6,
        ..CodegenConfig::default()
    };

    for (name, spice) in [
        ("voltage_divider", VOLTAGE_DIVIDER_SPICE),
        ("rc", RC_DCOP_SPICE),
        ("vcc_bias", VCC_BIAS_SPICE),
        ("dual_supply", DUAL_SUPPLY_SPICE),
    ] {
        let ir = build_ir(spice, &config);
        for (i, &v) in ir.dc_operating_point.iter().enumerate() {
            assert!(
                v.is_finite(),
                "DC OP[{}] for {} should be finite, got {}",
                i, name, v
            );
        }
    }
}
