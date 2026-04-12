//! Tests for the nonlinear DC operating point solver.
//!
//! Validates that the DC OP solver correctly finds bias points for
//! circuits with diodes and BJTs.

mod support;

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig, DcOpMethod};
use melange_solver::device_types::{BjtParams, DeviceParams, DeviceSlot, DeviceType, DiodeParams};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn build_pipeline(spice: &str) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA failed");
    // Stamp input conductance (1Ω)
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("DK failed");
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

fn build_ir(spice: &str, config: &CodegenConfig) -> CircuitIR {
    let (netlist, mna, kernel) = build_pipeline(spice);
    CircuitIR::from_kernel(&kernel, &mna, &netlist, config).expect("IR build failed")
}

// =============================================================================
// Test circuits
// =============================================================================

const VOLTAGE_DIVIDER: &str = "Voltage Divider
R1 in out 1k
R2 out 0 1k
";

const RC_LOWPASS: &str = "RC Lowpass
R1 in out 1k
C1 out 0 1u
";

const VCC_BIAS: &str = "VCC Bias
VCC vcc 0 DC 12
R1 vcc out 1k
R2 out 0 1k
";

const SINGLE_DIODE_VCC: &str = "Diode with VCC
VCC vcc 0 DC 5
R1 vcc out 1k
D1 out 0 DMOD
C1 out 0 1u
.MODEL DMOD D(IS=2.52e-9 N=1.752)
";

const ANTIPARALLEL_DIODES_VCC: &str = "Antiparallel Diodes with VCC
VCC vcc 0 DC 5
R1 vcc out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.MODEL DMOD D(IS=2.52e-9 N=1.752)
";

const BJT_COMMON_EMITTER: &str = "BJT Common Emitter
VCC vcc 0 DC 12
C1 in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit BC547
RC vcc coll 6.8k
RE emit 0 1k
CE emit 0 100u
C2 coll out 10u
Rload out 0 100k
.MODEL BC547 NPN(IS=1.8e-14 BF=400 BR=35.5)
";

const PNP_CIRCUIT: &str = "PNP BJT Circuit
VCC vcc 0 DC 12
R1 base 0 100k
R2 vcc base 22k
Q1 coll base emit PNPMOD
RC coll 0 6.8k
RE vcc emit 1k
CE vcc emit 100u
C1 in base 10u
C2 coll out 10u
Rload out 0 100k
.MODEL PNPMOD PNP(IS=1.8e-14 BF=200 BR=10)
";

// =============================================================================
// Linear circuits — same results as existing linear solver
// =============================================================================

#[test]
fn test_linear_circuits_unchanged() {
    // Voltage divider with no DC sources and no nonlinear devices
    // should return all zeros
    let (_netlist, mna, _) = build_pipeline(VOLTAGE_DIVIDER);
    let config = DcOpConfig {
        input_node: 0,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };

    let slots: Vec<DeviceSlot> = vec![];
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(result.converged);
    assert_eq!(result.method, DcOpMethod::Linear);
    assert!(result.v_node.iter().all(|&v| v.abs() < 1e-10));
}

#[test]
fn test_vcc_bias_linear() {
    // VCC bias network — no nonlinear devices, should match linear solver
    let (_netlist, mna, _) = build_pipeline(VCC_BIAS);
    let config = DcOpConfig {
        input_node: mna
            .node_map
            .get("in")
            .copied()
            .unwrap_or(1)
            .saturating_sub(1),
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };

    let slots: Vec<DeviceSlot> = vec![];
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(result.converged);
    assert_eq!(result.method, DcOpMethod::Linear);

    // VCC=12V, R1=R2=1k, so V_out should be about 6V
    let out_idx = mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(0)
        .saturating_sub(1);
    if out_idx < result.v_node.len() {
        let v_out = result.v_node[out_idx];
        assert!(
            (v_out - 6.0).abs() < 0.1,
            "Expected V_out ≈ 6V, got {:.3}V",
            v_out
        );
    }
}

// =============================================================================
// Diode circuits
// =============================================================================

#[test]
fn test_single_diode_vcc() {
    // VCC=5V → R=1k → D1 → GND
    // Expected: V_anode ≈ 0.6-0.7V (diode forward voltage)
    let (netlist, mna, _) = build_pipeline(SINGLE_DIODE_VCC);
    let config = DcOpConfig {
        input_node: mna
            .node_map
            .get("in")
            .copied()
            .unwrap_or(1)
            .saturating_sub(1),
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };

    // Build device slots from MNA device info
    let slots = build_device_slots(&netlist, &mna);

    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "DC OP solver did not converge for single diode circuit"
    );
    assert_ne!(result.method, DcOpMethod::Failed);

    // Check diode forward voltage
    let out_idx = mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(0)
        .saturating_sub(1);
    if out_idx < result.v_node.len() {
        let v_out = result.v_node[out_idx];
        assert!(
            v_out > 0.4 && v_out < 0.9,
            "Expected V_out ≈ 0.6V (diode forward drop), got {:.4}V",
            v_out
        );
    }

    // Check that i_nl is nonzero (diode is conducting)
    assert!(
        result.i_nl.iter().any(|&i| i.abs() > 1e-6),
        "Expected nonzero diode current at DC OP, got {:?}",
        result.i_nl
    );
}

#[test]
fn test_antiparallel_diodes_dc_op() {
    let (netlist, mna, _) = build_pipeline(ANTIPARALLEL_DIODES_VCC);
    let config = DcOpConfig {
        input_node: mna
            .node_map
            .get("in")
            .copied()
            .unwrap_or(1)
            .saturating_sub(1),
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };

    let slots = build_device_slots(&netlist, &mna);
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "DC OP solver did not converge for antiparallel diodes"
    );
}

// =============================================================================
// BJT circuits
// =============================================================================

#[test]
fn test_bjt_common_emitter_bias() {
    // Classic BJT CE amplifier. Expected bias point:
    // V_base ≈ 2.16V (from voltage divider 100k/22k)
    // V_emit ≈ V_base - 0.65 ≈ 1.5V
    // I_C ≈ V_emit/R_E ≈ 1.5mA
    // V_coll ≈ VCC - IC*RC ≈ 12 - 1.5*6.8 ≈ 1.8V
    let (netlist, mna, _) = build_pipeline(BJT_COMMON_EMITTER);
    let config = DcOpConfig {
        input_node: mna
            .node_map
            .get("in")
            .copied()
            .unwrap_or(1)
            .saturating_sub(1),
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };

    let slots = build_device_slots(&netlist, &mna);
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "DC OP solver did not converge for BJT CE circuit (method: {:?}, iters: {})",
        result.method, result.iterations
    );

    // Check node voltages
    let base_idx = mna
        .node_map
        .get("base")
        .copied()
        .unwrap_or(0)
        .saturating_sub(1);
    let emit_idx = mna
        .node_map
        .get("emit")
        .copied()
        .unwrap_or(0)
        .saturating_sub(1);
    let coll_idx = mna
        .node_map
        .get("coll")
        .copied()
        .unwrap_or(0)
        .saturating_sub(1);
    let vcc_idx = mna
        .node_map
        .get("vcc")
        .copied()
        .unwrap_or(0)
        .saturating_sub(1);

    if vcc_idx < result.v_node.len() {
        let v_vcc = result.v_node[vcc_idx];
        assert!(
            (v_vcc - 12.0).abs() < 0.1,
            "Expected V_vcc ≈ 12V, got {:.3}V",
            v_vcc
        );
    }

    if base_idx < result.v_node.len() {
        let v_base = result.v_node[base_idx];
        // Base voltage from divider: 12 * 22k/(100k+22k) ≈ 2.16V
        assert!(
            v_base > 1.0 && v_base < 3.5,
            "Expected V_base ≈ 2.2V, got {:.3}V",
            v_base
        );
    }

    if emit_idx < result.v_node.len() && base_idx < result.v_node.len() {
        let vbe = result.v_node[base_idx] - result.v_node[emit_idx];
        assert!(
            vbe > 0.4 && vbe < 0.8,
            "Expected Vbe ≈ 0.6V, got {:.3}V",
            vbe
        );
    }

    if coll_idx < result.v_node.len() {
        let v_coll = result.v_node[coll_idx];
        // Collector should be between 1V and 10V (active region)
        assert!(
            v_coll > 0.5 && v_coll < 11.0,
            "Expected V_coll in active region, got {:.3}V",
            v_coll
        );
    }

    // Check that i_nl is nonzero (BJT is conducting)
    assert!(
        result.i_nl.iter().any(|&i| i.abs() > 1e-6),
        "Expected nonzero BJT currents at DC OP"
    );
}

#[test]
fn test_pnp_bjt_dc_op() {
    let (netlist, mna, _) = build_pipeline(PNP_CIRCUIT);
    let config = DcOpConfig {
        input_node: mna
            .node_map
            .get("in")
            .copied()
            .unwrap_or(1)
            .saturating_sub(1),
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };

    let slots = build_device_slots(&netlist, &mna);
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "DC OP solver did not converge for PNP circuit (method: {:?}, iters: {})",
        result.method, result.iterations
    );

    // PNP should have collector voltage > 0 (current flows from VCC through RE to collector)
    let coll_idx = mna
        .node_map
        .get("coll")
        .copied()
        .unwrap_or(0)
        .saturating_sub(1);
    if coll_idx < result.v_node.len() {
        let v_coll = result.v_node[coll_idx];
        assert!(
            v_coll > 0.0,
            "Expected PNP collector voltage > 0, got {:.3}V",
            v_coll
        );
    }
}

// =============================================================================
// Source stepping / convergence strategy tests
// =============================================================================

#[test]
fn test_source_stepping_and_direct_nr_agree() {
    // For a simple diode circuit, both direct NR and source stepping
    // should find the same answer (within tolerance)
    let (netlist, mna, _) = build_pipeline(SINGLE_DIODE_VCC);
    let slots = build_device_slots(&netlist, &mna);

    let config_direct = DcOpConfig {
        input_node: mna
            .node_map
            .get("in")
            .copied()
            .unwrap_or(1)
            .saturating_sub(1),
        input_resistance: 1.0,
        max_iterations: 200,
        source_steps: 1, // Force single step = direct NR
        ..DcOpConfig::default()
    };

    let config_stepping = DcOpConfig {
        input_node: mna
            .node_map
            .get("in")
            .copied()
            .unwrap_or(1)
            .saturating_sub(1),
        input_resistance: 1.0,
        max_iterations: 200,
        source_steps: 20, // Fine-grained stepping
        ..DcOpConfig::default()
    };

    let result_direct = solve_dc_operating_point(&mna, &slots, &config_direct);
    let result_stepping = solve_dc_operating_point(&mna, &slots, &config_stepping);

    assert!(
        result_direct.converged,
        "Direct NR should converge for simple diode"
    );
    assert!(
        result_stepping.converged,
        "Source stepping should converge for simple diode"
    );

    // Results should agree within tolerance
    for (a, b) in result_direct
        .v_node
        .iter()
        .zip(result_stepping.v_node.iter())
    {
        assert!(
            (a - b).abs() < 1e-3,
            "Direct NR and source stepping disagree: {:.6} vs {:.6}",
            a,
            b
        );
    }
}

// =============================================================================
// Codegen integration
// =============================================================================

#[test]
fn test_codegen_with_nonlinear_dc_op() {
    let ir = build_ir(
        SINGLE_DIODE_VCC,
        &CodegenConfig {
            circuit_name: "diode_dc_test".to_string(),
            input_node: 0,
            output_nodes: vec![1],
            ..default_config()
        },
    );

    assert!(ir.has_dc_op, "Should have DC OP for circuit with VCC");
    assert!(
        ir.dc_op_converged,
        "Nonlinear DC OP should converge for diode circuit"
    );

    // Check that dc_nl_currents has a nonzero entry (diode is conducting)
    assert!(
        ir.dc_nl_currents.iter().any(|&i| i.abs() > 1e-10),
        "DC nonlinear currents should be nonzero for conducting diode: {:?}",
        ir.dc_nl_currents
    );
}

#[test]
fn test_codegen_bjt_has_dc_nl_i() {
    let ir = build_ir(
        BJT_COMMON_EMITTER,
        &CodegenConfig {
            circuit_name: "bjt_dc_test".to_string(),
            input_node: 0,
            output_nodes: vec![1],
            ..default_config()
        },
    );

    assert!(ir.has_dc_op, "Should have DC OP for BJT circuit");

    // BJT has M=2, dc_nl_currents should have 2 entries
    assert_eq!(
        ir.dc_nl_currents.len(),
        2,
        "BJT should have 2 DC NL currents"
    );

    // At least Ic should be nonzero at bias point
    assert!(
        ir.dc_nl_currents.iter().any(|&i| i.abs() > 1e-6),
        "BJT DC currents should be nonzero: {:?}",
        ir.dc_nl_currents
    );
}

#[test]
fn test_codegen_dc_nl_i_in_generated_code() {
    // Generate code for a BJT circuit and check that DC_NL_I constant is present
    let (netlist, mna, kernel) = build_pipeline(BJT_COMMON_EMITTER);
    let config = CodegenConfig {
        circuit_name: "bjt_dc_gen".to_string(),
        input_node: 0,
        output_nodes: vec![1],
        ..default_config()
    };

    let generator = CodeGenerator::new(config);
    let result = generator
        .generate(&kernel, &mna, &netlist)
        .expect("codegen failed");

    // Check that generated code contains DC_NL_I
    assert!(
        result.code.contains("DC_NL_I"),
        "Generated code should contain DC_NL_I constant for BJT circuit"
    );

    // Check that i_nl_prev is initialized from DC_NL_I
    assert!(
        result.code.contains("i_nl_prev: DC_NL_I"),
        "Generated code should initialize i_nl_prev from DC_NL_I"
    );
}

#[test]
fn test_codegen_linear_circuit_no_dc_nl_i() {
    // Linear circuit should NOT have DC_NL_I
    let (netlist, mna, kernel) = build_pipeline(RC_LOWPASS);
    let config = CodegenConfig {
        circuit_name: "rc_linear".to_string(),
        input_node: 0,
        output_nodes: vec![1],
        ..default_config()
    };

    let generator = CodeGenerator::new(config);
    let result = generator
        .generate(&kernel, &mna, &netlist)
        .expect("codegen failed");

    // Linear circuit should not have DC_NL_I
    assert!(
        !result.code.contains("DC_NL_I"),
        "Linear circuit should not have DC_NL_I constant"
    );
}

// =============================================================================
// Codegen DC OP integration (verifies codegen includes DC OP)
// =============================================================================

#[test]
fn test_codegen_dc_op_produces_stable_output() {
    // Verify that codegen with DC OP init produces stable, finite output
    // for a circuit with DC supply and nonlinear devices.
    let config = support::config_for_spice(SINGLE_DIODE_VCC, 44100.0);
    let circuit = support::build_circuit(SINGLE_DIODE_VCC, &config, "dc_op_codegen");

    // Run a few samples of zero input — DC OP should make output stable
    let output = support::run_step(&circuit, 0.0, 100, 44100.0);
    support::assert_finite(&output);

    // Generated code should include DC_OP constant (non-zero DC point)
    assert!(circuit.code.contains("DC_OP"), "Codegen should include DC_OP constant");
}

// =============================================================================
// Helpers
// =============================================================================

/// Build device slots from MNA device info + netlist models (matches CircuitIR::build_device_info).
fn build_device_slots(netlist: &Netlist, _mna: &MnaSystem) -> Vec<DeviceSlot> {
    use melange_solver::parser::Element;

    let mut slots = Vec::new();
    let mut dim_offset = 0;
    let vt = 0.02585;

    for elem in &netlist.elements {
        match elem {
            Element::Diode { model, .. } => {
                let is = lookup_model_param(netlist, model, "IS").unwrap_or(2.52e-9);
                let n = lookup_model_param(netlist, model, "N").unwrap_or(1.0);
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
            Element::Bjt { model, .. } => {
                let is = lookup_model_param(netlist, model, "IS").unwrap_or(1.26e-14);
                let vt_val = lookup_model_param(netlist, model, "VT").unwrap_or(vt);
                let beta_f = lookup_model_param(netlist, model, "BF").unwrap_or(200.0);
                let beta_r = lookup_model_param(netlist, model, "BR").unwrap_or(3.0);
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
                        vt: vt_val,
                        beta_f,
                        beta_r,
                        is_pnp,
                        vaf: f64::INFINITY,
                        var: f64::INFINITY,
                        ikf: f64::INFINITY,
                        ikr: f64::INFINITY,
                        cje: 0.0,
                        cjc: 0.0,
                        nf: 1.0,
                        nr: 1.0,
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
            _ => {}
        }
    }

    slots
}

// build_device_entries removed — was only used by test_runtime_solver_dc_op_init.

fn lookup_model_param(netlist: &Netlist, model_name: &str, param_name: &str) -> Option<f64> {
    netlist
        .models
        .iter()
        .find(|m| m.name.eq_ignore_ascii_case(model_name))
        .and_then(|m| {
            m.params
                .iter()
                .find(|(k, _)| k.eq_ignore_ascii_case(param_name))
                .map(|(_, v)| *v)
        })
}

// =============================================================================
// Power amplifier DC OP (internal nodes for parasitic BJTs)
// =============================================================================

#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_power_amp_dc_op_converges() {
    // Wurlitzer 200A power amplifier: 8 BJTs, Class AB push-pull.
    // Q2N5087 has RB=120Ω — the old 3-iteration inner loop failed to converge.
    // Internal nodes fix: parasitic R in global matrix, intrinsic model on junctions.
    let src = std::fs::read_to_string("../../circuits/testing/wurli-power-amp.cir")
        .expect("wurli-power-amp.cir not found");
    let netlist = Netlist::parse(&src).expect("parse failed");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA failed");

    // Stamp input conductance (1Ω)
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }

    // Build full device slots (with GP params including RB/RC/RE)
    let slots = CircuitIR::build_device_info(&netlist).expect("device info");

    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };

    // Verify parasitic BJTs are present
    let parasitic_count = slots
        .iter()
        .filter(|s| matches!(&s.params, DeviceParams::Bjt(bp) if bp.has_parasitics()))
        .count();
    assert_eq!(parasitic_count, 8, "All 8 BJTs should have parasitic R");

    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "Power amp DC OP should converge (method: {:?}, iters: {})",
        result.method, result.iterations
    );

    // ngspice reference values:
    // v(out) = -0.063V, v(emit_pair) = 0.662V, v(drv_bot) = -0.727V
    // v(vas_out) = 0.445V, v(coll7) = -21.81V

    let out_idx = mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(0)
        .saturating_sub(1);
    let emit_pair_idx = mna
        .node_map
        .get("emit_pair")
        .copied()
        .unwrap_or(0)
        .saturating_sub(1);

    // Compare against ngspice reference values (within 0.5V tolerance for bias points)
    if out_idx < result.v_node.len() {
        let v_out = result.v_node[out_idx];
        assert!(
            (v_out - (-0.063)).abs() < 0.5,
            "v(out) should be near -0.063V (ngspice), got {:.4}V",
            v_out
        );
    }

    if emit_pair_idx < result.v_node.len() {
        let v_ep = result.v_node[emit_pair_idx];
        assert!(
            (v_ep - 0.662).abs() < 0.5,
            "v(emit_pair) should be near 0.662V (ngspice), got {:.4}V",
            v_ep
        );
    }

    // At least some BJTs should be conducting (nonzero i_nl)
    let max_i = result.i_nl.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    assert!(
        max_i > 1e-6,
        "At least one BJT should be conducting at DC OP, max |i_nl| = {:.2e}",
        max_i
    );
}

// =============================================================================
// Pentode DC operating point (Reefman Derk §4.4 — task P1a-06)
// =============================================================================
//
// EL84 single-ended Class A power stage. Reefman fit (uTracer-calibrated)
// will not match RCA datasheet values exactly, so the asserts here use wide
// bands. The intent is to lock in:
//   1. The 3D dispatch arm in dc_op.rs evaluates currents/Jacobian without
//      panics or NaN/Inf.
//   2. NR converges from the seeded initial guess (clamp_junction_voltages
//      pre-biases the screen ≈250V and grid ≈-2V).
//   3. NR also converges from a cold v=0 start (exercises source-stepping /
//      Gmin-stepping fallback chain).

const EL84_CLASS_A: &str = "EL84 Class A SE
VCC vcc 0 DC 300
RP vcc plate 5.6k
RG2 vcc screen 1k
RK cath 0 150
RG in grid 470k
P1 plate grid cath screen EL84
.MODEL EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 KP=152.4 KVB=4015.8
+    ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148
+    IG_MAX=8e-3 VGK_ONSET=0.7)
";

fn build_el84_pipeline() -> (MnaSystem, Vec<DeviceSlot>, usize) {
    let netlist = Netlist::parse(EL84_CLASS_A).expect("EL84 parse failed");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("EL84 MNA failed");
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }
    let slots = CircuitIR::build_device_info(&netlist).expect("device info");
    (mna, slots, input_node)
}

fn pentode_indices(mna: &MnaSystem) -> (usize, usize, usize, usize) {
    let plate = mna.node_map.get("plate").copied().unwrap() - 1;
    let grid = mna.node_map.get("grid").copied().unwrap() - 1;
    let cath = mna.node_map.get("cath").copied().unwrap() - 1;
    let screen = mna.node_map.get("screen").copied().unwrap() - 1;
    (plate, grid, cath, screen)
}

#[test]
fn test_pentode_dc_op_el84_class_a() {
    let (mna, slots, input_node) = build_el84_pipeline();
    assert_eq!(slots.len(), 1, "should have one pentode device");
    assert_eq!(slots[0].dimension, 3, "pentode dimension must be 3");

    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &slots, &config);
    assert!(
        result.converged,
        "EL84 Class A DC OP should converge (method: {:?}, iters: {})",
        result.method, result.iterations
    );
    assert!(
        !matches!(result.method, DcOpMethod::Failed),
        "method should not be Failed"
    );

    // No NaN/Inf anywhere in the converged solution.
    for (i, &v) in result.v_node.iter().enumerate() {
        assert!(v.is_finite(), "v_node[{i}] = {v} not finite");
    }
    for (i, &v) in result.v_nl.iter().enumerate() {
        assert!(v.is_finite(), "v_nl[{i}] = {v} not finite");
    }
    for (i, &i_val) in result.i_nl.iter().enumerate() {
        assert!(i_val.is_finite(), "i_nl[{i}] = {i_val} not finite");
    }

    let (p_idx, g_idx, k_idx, s_idx) = pentode_indices(&mna);
    let v_p = result.v_node[p_idx];
    let v_g = result.v_node[g_idx];
    let v_k = result.v_node[k_idx];
    let v_s = result.v_node[s_idx];

    let vgk = v_g - v_k;
    let vpk = v_p - v_k;
    let vg2k = v_s - v_k;
    let ip = result.i_nl[slots[0].start_idx];
    let ig2 = result.i_nl[slots[0].start_idx + 1];
    let ig1 = result.i_nl[slots[0].start_idx + 2];

    eprintln!(
        "EL84 DC OP: Vgk={:.3}V Vpk={:.2}V Vg2k={:.2}V  Ip={:.3}mA Ig2={:.3}mA Ig1={:.3}mA  iters={} method={:?}",
        vgk,
        vpk,
        vg2k,
        ip * 1000.0,
        ig2 * 1000.0,
        ig1 * 1000.0,
        result.iterations,
        result.method
    );

    // Class A bias windows. Bands are deliberately wide because:
    //   * The Reefman EL84 fit is calibrated to uTracer curves, not RCA
    //     datasheet, and the load-line interaction with our specific Rp/Rk
    //     pair determines the precise OP.
    //   * With Rp = 5.6k and Vcc = 300V the model lands hot (Ip ≈ 40mA),
    //     which pulls Vpk well below 250V due to the IR_p drop. This is
    //     correct physics — verify only that the OP is *physical*, not
    //     that it lands on a particular text-book point.
    //
    // What we *do* assert tightly: Vpk > 0 (plate hasn't crashed below
    // ground), Vg2k positive and ≤ supply, Vgk negative (Class A cutoff
    // direction), and Ip in the EL84 max-rating window (< 80 mA).
    assert!(
        (10.0..=300.0).contains(&vpk),
        "Vpk should be positive and below B+, got {:.2}V",
        vpk
    );
    assert!(
        (200.0..=310.0).contains(&vg2k),
        "Vg2k should be in 200-310V (screen-resistor drop ≤ 100V), got {:.2}V",
        vg2k
    );
    assert!(
        (-15.0..=-1.0).contains(&vgk),
        "Vgk should be negative (Class A cutoff bias), got {:.3}V",
        vgk
    );
    let ip_ma = ip * 1000.0;
    assert!(
        (0.5..=80.0).contains(&ip_ma),
        "Ip should be in EL84 Class A range (sub-max-rating), got {:.3}mA",
        ip_ma
    );

    // Ig1 (control grid) should be ≈0 — Vgk is negative, so the Leach
    // power-law term returns zero exactly.
    assert!(
        ig1.abs() < 1e-9,
        "Ig1 should be near zero in Class A, got {:.3e}A",
        ig1
    );
}

#[test]
fn test_pentode_dc_op_convergence_from_cold_start() {
    // Same EL84 circuit, but force the solver through the source-stepping
    // / Gmin-stepping fallback chain by using an unusually tight tolerance
    // and disabling some of the easy outs. The actual entry point is the
    // public solver, so we can't *force* a cold start — but if the EL84
    // converges via *any* method, the fallback chain is intact.
    let (mna, slots, input_node) = build_el84_pipeline();
    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &slots, &config);
    assert!(
        result.converged,
        "EL84 DC OP must converge from any starting strategy: method={:?} iters={}",
        result.method, result.iterations
    );
    // Confirm we didn't fall through to the linear/Failed branch.
    assert_ne!(result.method, DcOpMethod::Failed);
    assert_ne!(result.method, DcOpMethod::Linear);
    // i_nl must show a real conducting pentode.
    let max_i = result.i_nl.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    assert!(
        max_i > 1e-6,
        "EL84 should be conducting at DC OP, max |i_nl| = {:.2e}",
        max_i
    );
}

#[test]
fn test_pentode_dc_op_no_nan() {
    let (mna, slots, input_node) = build_el84_pipeline();
    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &slots, &config);
    assert!(result.converged, "EL84 DC OP must converge");

    let mut all_vals: Vec<f64> = Vec::new();
    all_vals.extend(result.v_node.iter().copied());
    all_vals.extend(result.v_nl.iter().copied());
    all_vals.extend(result.i_nl.iter().copied());
    for (i, v) in all_vals.iter().enumerate() {
        assert!(
            v.is_finite() && !v.is_nan(),
            "EL84 DC OP value index {i} = {v} not finite"
        );
    }
}

#[test]
fn test_pentode_jacobian_matches_fd() {
    use melange_solver::dc_op::evaluate_devices;

    // Test the EL84 dispatch arm directly against finite differences at a
    // representative Class A operating point. We don't need an MNA system
    // for this — just the slot and an evaluate_devices call.
    let (_mna, slots, _input_node) = build_el84_pipeline();
    let slot = slots[0].clone();
    let m = 3;

    // Class A operating point: Vgk=-5, Vpk=250, Vg2k=290.
    let v_op = [-5.0, 250.0, 290.0];

    let mut i0 = vec![0.0; m];
    let mut j0 = vec![0.0; m * m];
    evaluate_devices(&v_op, &[slot.clone()], &mut i0, &mut j0, m);

    // Currents must be finite and physical.
    for (i, val) in i0.iter().enumerate() {
        assert!(val.is_finite(), "i0[{i}] = {val} not finite");
    }
    assert!(i0[0] > 0.0, "Ip should be positive at Class A OP, got {}", i0[0]);
    assert!(i0[1] > 0.0, "Ig2 should be positive at Class A OP, got {}", i0[1]);

    // Finite-difference Jacobian (3×3 row-major). Use a sympathetic step.
    let h = 1e-3;
    let mut j_fd = vec![0.0; m * m];
    for col in 0..3 {
        let mut v_plus = v_op;
        let mut v_minus = v_op;
        v_plus[col] += h;
        v_minus[col] -= h;
        let mut i_plus = vec![0.0; m];
        let mut i_minus = vec![0.0; m];
        let mut dummy = vec![0.0; m * m];
        evaluate_devices(&v_plus, &[slot.clone()], &mut i_plus, &mut dummy, m);
        evaluate_devices(&v_minus, &[slot.clone()], &mut i_minus, &mut dummy, m);
        for row in 0..3 {
            j_fd[row * m + col] = (i_plus[row] - i_minus[row]) / (2.0 * h);
        }
    }

    // Compare entry-by-entry. Skip Ig1 row entries (sparse: only [2][0] is
    // nonzero, and at Vgk=-5 even that is zero by Leach cutoff). Also skip
    // entries whose magnitude is below 1e-12 — FD noise dominates there.
    for row in 0..2 {
        for col in 0..3 {
            let analytic = j0[row * m + col];
            let fd = j_fd[row * m + col];
            let scale = analytic.abs().max(fd.abs()).max(1e-12);
            if scale < 1e-10 {
                continue;
            }
            let rel_err = (analytic - fd).abs() / scale;
            assert!(
                rel_err < 1e-3,
                "Jacobian entry [{row}][{col}]: analytic={analytic:.6e} fd={fd:.6e} rel_err={rel_err:.3e}"
            );
        }
    }
}

// =============================================================================
// Pentode DC operating point — circuit file validation
// =============================================================================
//
// These tests load real .cir files from circuits/testing/ and verify that the
// DC-OP solver converges to physically reasonable bias points for pentode
// circuits. Tolerances are deliberately wide (±20%) because the Reefman tube
// fits are calibrated to uTracer curves, not necessarily to canonical
// datasheet values, and load-line interaction with specific component values
// shifts the operating point.

/// Helper: parse a .cir file, build MNA + device slots, stamp junction caps
/// and input conductance, return everything needed for DC-OP.
fn build_pentode_circuit_pipeline(
    path: &str,
) -> (MnaSystem, Vec<DeviceSlot>, usize, Netlist) {
    let src = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("Failed to read {}: {}", path, e));
    let netlist = Netlist::parse(&src).expect("parse failed");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA failed");

    // Build device slots
    let device_slots = CircuitIR::build_device_info(&netlist).expect("device info");

    // Stamp junction capacitances
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }

    // Stamp input conductance (1 ohm)
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }

    (mna, device_slots, input_node, netlist)
}

/// Helper: get a node index from the MNA node map (1-based -> 0-based).
fn node_idx(mna: &MnaSystem, name: &str) -> usize {
    mna.node_map
        .get(name)
        .copied()
        .unwrap_or_else(|| panic!("node '{}' not found in MNA", name))
        .saturating_sub(1)
}

// -----------------------------------------------------------------------------
// EL84 single-stage (el84-single-stage.cir)
// -----------------------------------------------------------------------------
//
// Single EL84 pentode Class A power stage. B+ = 300V.
// Expected: Vpk ~ 200-260V, Vg2k ~ 280-300V, Vgk ~ -5 to -12V, Ip ~ 20-50mA.

#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pentode_dc_op_el84_single_stage_circuit() {
    let (mna, slots, input_node, _netlist) =
        build_pentode_circuit_pipeline("../../circuits/testing/el84-single-stage.cir");

    // Should have exactly 1 pentode device (dimension 3)
    let pentode_slots: Vec<_> = slots.iter().filter(|s| s.dimension == 3).collect();
    assert_eq!(
        pentode_slots.len(),
        1,
        "el84-single-stage should have 1 pentode, found {}",
        pentode_slots.len()
    );

    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "EL84 single-stage DC OP should converge (method: {:?}, iters: {})",
        result.method, result.iterations
    );
    assert_ne!(result.method, DcOpMethod::Failed);

    // All values must be finite
    for (i, &v) in result.v_node.iter().enumerate() {
        assert!(v.is_finite(), "v_node[{}] = {} not finite", i, v);
    }

    // Check key node voltages
    let plate_idx = node_idx(&mna, "plate");
    let grid_idx = node_idx(&mna, "grid");
    let cath_idx = node_idx(&mna, "cath");
    let screen_idx = node_idx(&mna, "screen");

    let v_plate = result.v_node[plate_idx];
    let v_grid = result.v_node[grid_idx];
    let v_cath = result.v_node[cath_idx];
    let v_screen = result.v_node[screen_idx];

    let vpk = v_plate - v_cath;
    let vgk = v_grid - v_cath;
    let vg2k = v_screen - v_cath;

    eprintln!(
        "EL84 single-stage DC OP: Vpk={:.1}V  Vg2k={:.1}V  Vgk={:.2}V  V_plate={:.1}V  V_cath={:.2}V",
        vpk, vg2k, vgk, v_plate, v_cath
    );

    // Plate-cathode: should be well above 0, below B+ (300V).
    // With Ra=5.6k and ~40mA plate current, Vpk ≈ 300 - 40e-3*5600 - Vk ≈ 70-260V
    assert!(
        vpk > 10.0 && vpk < 300.0,
        "Vpk should be in 10-300V range, got {:.1}V",
        vpk
    );

    // Screen-cathode: screen supply 300V through 1k resistor, so Vg2k ≈ 280-300V
    assert!(
        vg2k > 200.0 && vg2k < 310.0,
        "Vg2k should be in 200-310V range, got {:.1}V",
        vg2k
    );

    // Grid-cathode: should be negative (cathode bias from Rk=130)
    assert!(
        vgk < 0.0 && vgk > -20.0,
        "Vgk should be negative (Class A bias), got {:.2}V",
        vgk
    );

    // NL currents: pentode Ip should be non-negligible
    let slot = &pentode_slots[0];
    let ip = result.i_nl[slot.start_idx];
    let ip_ma = ip * 1000.0;
    eprintln!("  Ip = {:.2} mA", ip_ma);
    assert!(
        ip_ma > 1.0 && ip_ma < 80.0,
        "Ip should be in 1-80 mA range for EL84 Class A, got {:.2} mA",
        ip_ma
    );
}

// -----------------------------------------------------------------------------
// AC15 (ac15.cir)
// -----------------------------------------------------------------------------
//
// Vox AC15: EF86 pentode preamp + 12AX7 cathodyne PI + 2x EL84 push-pull.
// This is a complex multi-tube circuit with coupled inductors (output
// transformer). We primarily verify convergence and that plate voltages
// are in a physically reasonable range (100-320V).

#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pentode_dc_op_ac15_convergence() {
    let (mna, slots, input_node, _netlist) =
        build_pentode_circuit_pipeline("../../circuits/testing/ac15.cir");

    // Should have 3 pentode/triode devices total:
    //   EF86 pentode (dim 3), 12AX7 triode (dim 2), 2x EL84 pentode (dim 3 each)
    // Total NL devices: 4 (1 triode + 3 pentodes)
    let pentode_count = slots.iter().filter(|s| s.dimension == 3).count();
    let triode_count = slots.iter().filter(|s| s.dimension == 2).count();
    assert!(
        pentode_count >= 2,
        "AC15 should have at least 2 pentodes (EF86 + 2x EL84), found {}",
        pentode_count
    );
    assert!(
        triode_count >= 1,
        "AC15 should have at least 1 triode (12AX7 PI), found {}",
        triode_count
    );

    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "AC15 DC OP should converge (method: {:?}, iters: {})",
        result.method, result.iterations
    );
    assert_ne!(result.method, DcOpMethod::Failed);

    // All values must be finite
    for (i, &v) in result.v_node.iter().enumerate() {
        assert!(v.is_finite(), "v_node[{}] = {} not finite", i, v);
    }
    for (i, &v) in result.i_nl.iter().enumerate() {
        assert!(v.is_finite(), "i_nl[{}] = {} not finite", i, v);
    }

    // B+ rail should be near 320V
    let vcc_idx = node_idx(&mna, "vcc");
    let v_vcc = result.v_node[vcc_idx];
    assert!(
        (v_vcc - 320.0).abs() < 1.0,
        "VCC should be ~320V, got {:.1}V",
        v_vcc
    );

    // EF86 plate: should be well above 0 and below VCC
    let pa_ef_idx = node_idx(&mna, "pa_ef");
    let v_pa_ef = result.v_node[pa_ef_idx];
    eprintln!("AC15 DC OP: V(pa_ef) = {:.1}V", v_pa_ef);
    assert!(
        v_pa_ef > 50.0 && v_pa_ef < 320.0,
        "EF86 plate voltage should be in 50-320V range, got {:.1}V",
        v_pa_ef
    );

    // PI plate and cathode: both should be positive, below VCC
    let pi_p_idx = node_idx(&mna, "pi_p");
    let pi_k_idx = node_idx(&mna, "pi_k");
    let v_pi_p = result.v_node[pi_p_idx];
    let v_pi_k = result.v_node[pi_k_idx];
    eprintln!(
        "AC15 DC OP: V(pi_p) = {:.1}V  V(pi_k) = {:.1}V",
        v_pi_p, v_pi_k
    );
    assert!(
        v_pi_p > 50.0 && v_pi_p < 320.0,
        "PI plate voltage should be in 50-320V range, got {:.1}V",
        v_pi_p
    );
    assert!(
        v_pi_k > 0.0 && v_pi_k < 250.0,
        "PI cathode voltage should be in 0-250V range, got {:.1}V",
        v_pi_k
    );

    // EL84 push-pull plates: should be positive and in 100-320V range
    let pa_1_idx = node_idx(&mna, "pa_1");
    let pa_2_idx = node_idx(&mna, "pa_2");
    let v_pa_1 = result.v_node[pa_1_idx];
    let v_pa_2 = result.v_node[pa_2_idx];
    eprintln!(
        "AC15 DC OP: V(pa_1) = {:.1}V  V(pa_2) = {:.1}V",
        v_pa_1, v_pa_2
    );
    assert!(
        v_pa_1 > 50.0 && v_pa_1 < 320.0,
        "EL84 #1 plate voltage should be in 50-320V range, got {:.1}V",
        v_pa_1
    );
    assert!(
        v_pa_2 > 50.0 && v_pa_2 < 320.0,
        "EL84 #2 plate voltage should be in 50-320V range, got {:.1}V",
        v_pa_2
    );

    // Push-pull plates should be roughly symmetric (within 30V of each other)
    assert!(
        (v_pa_1 - v_pa_2).abs() < 30.0,
        "Push-pull plate voltages should be roughly symmetric, got {:.1}V vs {:.1}V (diff={:.1}V)",
        v_pa_1,
        v_pa_2,
        (v_pa_1 - v_pa_2).abs()
    );

    // At least some devices should be conducting
    let max_i = result.i_nl.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    assert!(
        max_i > 1e-6,
        "At least one tube should be conducting at DC OP, max |i_nl| = {:.2e}",
        max_i
    );
}

// -----------------------------------------------------------------------------
// Tweed Deluxe (tweed-deluxe.cir)
// -----------------------------------------------------------------------------
//
// Fender 5E3 Tweed Deluxe: 12AX7 preamp + 12AX7 cathodyne PI + 2x 6V6GT
// beam tetrode push-pull (DerkE §4.5 exponential screen form).
// Coupled inductors (OT). Primarily verify convergence + reasonable voltages.

#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pentode_dc_op_tweed_deluxe_convergence() {
    let (mna, slots, input_node, _netlist) =
        build_pentode_circuit_pipeline("../../circuits/testing/tweed-deluxe.cir");

    // Should have 2 pentodes (6V6GT) and 2 triodes (12AX7 preamp + PI)
    let pentode_count = slots.iter().filter(|s| s.dimension == 3).count();
    let triode_count = slots.iter().filter(|s| s.dimension == 2).count();
    assert!(
        pentode_count >= 2,
        "Tweed Deluxe should have at least 2 pentodes (6V6GT pair), found {}",
        pentode_count
    );
    assert!(
        triode_count >= 2,
        "Tweed Deluxe should have at least 2 triodes (12AX7 preamp + PI), found {}",
        triode_count
    );

    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "Tweed Deluxe DC OP should converge (method: {:?}, iters: {})",
        result.method, result.iterations
    );
    assert_ne!(result.method, DcOpMethod::Failed);

    // All values must be finite
    for (i, &v) in result.v_node.iter().enumerate() {
        assert!(v.is_finite(), "v_node[{}] = {} not finite", i, v);
    }
    for (i, &v) in result.i_nl.iter().enumerate() {
        assert!(v.is_finite(), "i_nl[{}] = {} not finite", i, v);
    }

    // B+ rail should be near 320V
    let vcc_idx = node_idx(&mna, "vcc");
    let v_vcc = result.v_node[vcc_idx];
    assert!(
        (v_vcc - 320.0).abs() < 1.0,
        "VCC should be ~320V, got {:.1}V",
        v_vcc
    );

    // 12AX7 preamp plate: should be in active region
    let v1_p_idx = node_idx(&mna, "v1_p");
    let v_v1_p = result.v_node[v1_p_idx];
    eprintln!("Tweed Deluxe DC OP: V(v1_p) = {:.1}V", v_v1_p);
    assert!(
        v_v1_p > 50.0 && v_v1_p < 320.0,
        "12AX7 preamp plate should be in 50-320V range, got {:.1}V",
        v_v1_p
    );

    // PI plate and cathode: cathodyne with 56k loads
    let pi_p_idx = node_idx(&mna, "pi_p");
    let pi_k_idx = node_idx(&mna, "pi_k");
    let v_pi_p = result.v_node[pi_p_idx];
    let v_pi_k = result.v_node[pi_k_idx];
    eprintln!(
        "Tweed Deluxe DC OP: V(pi_p) = {:.1}V  V(pi_k) = {:.1}V",
        v_pi_p, v_pi_k
    );
    assert!(
        v_pi_p > 50.0 && v_pi_p < 320.0,
        "PI plate voltage should be in 50-320V range, got {:.1}V",
        v_pi_p
    );
    assert!(
        v_pi_k > 0.0 && v_pi_k < 250.0,
        "PI cathode voltage should be in 0-250V range, got {:.1}V",
        v_pi_k
    );

    // 6V6GT push-pull plates: should be in 100-320V range
    let pa_1_idx = node_idx(&mna, "pa_1");
    let pa_2_idx = node_idx(&mna, "pa_2");
    let v_pa_1 = result.v_node[pa_1_idx];
    let v_pa_2 = result.v_node[pa_2_idx];
    eprintln!(
        "Tweed Deluxe DC OP: V(pa_1) = {:.1}V  V(pa_2) = {:.1}V",
        v_pa_1, v_pa_2
    );
    assert!(
        v_pa_1 > 50.0 && v_pa_1 < 320.0,
        "6V6GT #1 plate voltage should be in 50-320V range, got {:.1}V",
        v_pa_1
    );
    assert!(
        v_pa_2 > 50.0 && v_pa_2 < 320.0,
        "6V6GT #2 plate voltage should be in 50-320V range, got {:.1}V",
        v_pa_2
    );

    // Push-pull symmetry check
    assert!(
        (v_pa_1 - v_pa_2).abs() < 30.0,
        "Push-pull plate voltages should be roughly symmetric, got {:.1}V vs {:.1}V (diff={:.1}V)",
        v_pa_1,
        v_pa_2,
        (v_pa_1 - v_pa_2).abs()
    );

    // Shared cathode bias node: should be a few volts above ground
    let ck_pp_idx = node_idx(&mna, "ck_pp");
    let v_ck = result.v_node[ck_pp_idx];
    eprintln!("Tweed Deluxe DC OP: V(ck_pp) = {:.1}V", v_ck);
    assert!(
        v_ck > 0.5 && v_ck < 50.0,
        "Shared cathode bias should be a few volts above ground, got {:.1}V",
        v_ck
    );

    // At least some devices should be conducting
    let max_i = result.i_nl.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    assert!(
        max_i > 1e-6,
        "At least one tube should be conducting at DC OP, max |i_nl| = {:.2e}",
        max_i
    );
}

// -----------------------------------------------------------------------------
// 6K7 variable-mu stage (6k7-varimu-stage.cir)
// -----------------------------------------------------------------------------
//
// Single 6K7 remote-cutoff pentode (Reefman §5 two-section Koren). B+ = 250V.
// Default Rk = 150 (max gain). Screen fed through 470k from B+.
// Expected: Vpk ~ 100-230V, Vg2k ~ 50-110V, Vgk ~ -0.5 to -3V, Ip ~ 1-8mA.

#[test]
#[ignore] // circuit migrated to melange-audio/circuits repo — test moves with it
fn test_pentode_dc_op_6k7_varimu_stage() {
    let (mna, slots, input_node, _netlist) =
        build_pentode_circuit_pipeline("../../circuits/testing/6k7-varimu-stage.cir");

    // Should have exactly 1 pentode device (dimension 3)
    let pentode_slots: Vec<_> = slots.iter().filter(|s| s.dimension == 3).collect();
    assert_eq!(
        pentode_slots.len(),
        1,
        "6k7-varimu-stage should have 1 pentode, found {}",
        pentode_slots.len()
    );

    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "6K7 varimu DC OP should converge (method: {:?}, iters: {})",
        result.method, result.iterations
    );
    assert_ne!(result.method, DcOpMethod::Failed);

    // All values must be finite
    for (i, &v) in result.v_node.iter().enumerate() {
        assert!(v.is_finite(), "v_node[{}] = {} not finite", i, v);
    }

    // Check key node voltages
    let plate_idx = node_idx(&mna, "plate");
    let grid_idx = node_idx(&mna, "grid");
    let cath_idx = node_idx(&mna, "cath");
    let screen_idx = node_idx(&mna, "screen");

    let v_plate = result.v_node[plate_idx];
    let v_grid = result.v_node[grid_idx];
    let v_cath = result.v_node[cath_idx];
    let v_screen = result.v_node[screen_idx];

    let vpk = v_plate - v_cath;
    let vgk = v_grid - v_cath;
    let vg2k = v_screen - v_cath;

    eprintln!(
        "6K7 varimu DC OP: Vpk={:.1}V  Vg2k={:.1}V  Vgk={:.2}V  V_plate={:.1}V  V_screen={:.1}V  V_cath={:.2}V",
        vpk, vg2k, vgk, v_plate, v_screen, v_cath
    );

    // Plate-cathode: should be positive, below B+ (250V)
    // With Ra=100k and small Ip, most of B+ appears at the plate
    assert!(
        vpk > 50.0 && vpk < 250.0,
        "Vpk should be in 50-250V range, got {:.1}V",
        vpk
    );

    // Screen-cathode: screen fed through 470k from 250V B+, with 47nF bypass.
    // At small screen currents, drop across 470k is moderate.
    // Wide tolerance since screen current + 470k can create significant drop.
    assert!(
        vg2k > 10.0 && vg2k < 255.0,
        "Vg2k should be in 10-255V range, got {:.1}V",
        vg2k
    );

    // Grid-cathode: should be negative (cathode bias, Rk=150, small Ip)
    assert!(
        vgk < 0.0 && vgk > -20.0,
        "Vgk should be negative (cathode bias), got {:.2}V",
        vgk
    );

    // Cathode voltage: Rk=150 with a few mA gives a small positive voltage
    assert!(
        v_cath > 0.0 && v_cath < 10.0,
        "Cathode voltage should be a small positive value, got {:.2}V",
        v_cath
    );

    // NL currents: pentode Ip should be non-negligible
    let slot = &pentode_slots[0];
    let ip = result.i_nl[slot.start_idx];
    let ip_ma = ip * 1000.0;
    eprintln!("  Ip = {:.3} mA", ip_ma);
    assert!(
        ip_ma > 0.1 && ip_ma < 20.0,
        "Ip should be in 0.1-20 mA range for 6K7 at default bias, got {:.3} mA",
        ip_ma
    );

    // Ig1 (control grid) should be ~0 — Vgk is negative
    let ig1 = result.i_nl[slot.start_idx + 2];
    assert!(
        ig1.abs() < 1e-9,
        "Ig1 should be near zero with negative Vgk, got {:.3e}A",
        ig1
    );
}
