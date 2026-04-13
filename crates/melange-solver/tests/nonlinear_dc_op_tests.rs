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

// Power amp and pentode DC OP circuit-file tests removed — circuits migrated
// to melange-audio/circuits repo.
//
// Removed tests:
//   - test_power_amp_dc_op_converges
//   - test_pentode_dc_op_el84_single_stage_circuit
//   - test_pentode_dc_op_ac15_convergence
//   - test_pentode_dc_op_tweed_deluxe_convergence
//   - test_pentode_dc_op_6k7_varimu_stage
// Removed helpers (only used by those tests):
//   - build_pentode_circuit_pipeline()
//   - node_idx()

