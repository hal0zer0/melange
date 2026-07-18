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
    assert!(
        circuit.code.contains("DC_OP"),
        "Codegen should include DC_OP constant"
    );
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
                let rs = lookup_model_param(netlist, model, "RS").unwrap_or(0.0);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Diode,
                    start_idx: dim_offset,
                    dimension: 1,
                    params: DeviceParams::Diode(DiodeParams {
                        is,
                        n_vt: n * vt,
                        cjo: 0.0,
                        rs,
                        bv: f64::INFINITY,
                        ibv: 1e-10,
                        rth: f64::INFINITY,
                        cth: 1e-3,
                        xti: 3.0,
                        eg: 1.11,
                        tamb: 300.15,
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
                let ise = lookup_model_param(netlist, model, "ISE").unwrap_or(0.0);
                let ne = lookup_model_param(netlist, model, "NE").unwrap_or(1.5);
                let isc = lookup_model_param(netlist, model, "ISC").unwrap_or(0.0);
                let nc = lookup_model_param(netlist, model, "NC").unwrap_or(2.0);
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
                        tf: 0.0,
                        vje: 0.75,
                        mje: 0.33,
                        vjc: 0.75,
                        mjc: 0.33,
                        fc: 0.5,
                        nf: 1.0,
                        nr: 1.0,
                        ise,
                        ne,
                        isc,
                        nc,
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

// =============================================================================
// Regression tests: DC OP must solve the SAME device models as the transient
// runtime (2026-07-18 fixes: BJT ISE/ISC leakage, diode RS junction solve,
// damped parasitic-BJT internal Newton, JFET Ig ≡ 0).
// =============================================================================

/// ISE-significant bias network. At the converged point the leakage base
/// current (ISE=1e-11, NE=2) is comparable to the ideal IS/BF term, so
/// omitting it (pre-fix behavior) shifted Vbe by ~15 mV and the collector
/// bias by ~2 V through RC=1k:
///   before (no leakage): Vbe=0.694, Ic=4.52 mA, V(coll)=7.48 V
///   after  (leakage):    Vbe=0.679, Ic=2.52 mA, V(coll)=9.48 V
const BJT_ISE_BIAS: &str = "BJT ISE Bias
VCC vcc 0 DC 12
RB vcc base 1meg
Q1 coll base 0 QLEAK
RC vcc coll 1k
C1 in base 10u
C2 coll 0 1u
.MODEL QLEAK NPN(IS=1e-14 BF=400 BR=3 ISE=1e-11 NE=2.0)
";

#[test]
fn test_bjt_ise_leakage_in_dc_op() {
    use melange_devices::bjt::{BjtEbersMoll, BjtPolarity};

    // DC-OP-only pipeline (no DK kernel — this bias network is a DC OP
    // regression fixture, not a transient circuit).
    let netlist = Netlist::parse(BJT_ISE_BIAS).expect("parse failed");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA failed");
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0;
    }
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
    // Sanity: the harness must have picked up ISE from the model card.
    match &slots[0].params {
        DeviceParams::Bjt(bp) => {
            assert!(bp.ise > 0.0, "harness failed to parse ISE from model card");
            assert!((bp.ne - 2.0).abs() < 1e-12, "harness failed to parse NE");
        }
        _ => panic!("expected BJT slot"),
    }

    let result = solve_dc_operating_point(&mna, &slots, &config);
    assert!(
        result.converged,
        "DC OP failed to converge: {:?}",
        result.method
    );

    let vbe = result.v_nl[0];
    let vbc = result.v_nl[1];
    let ib = result.i_nl[1];

    // KCL at the base: everything flowing through RB=1M enters the base.
    let base_idx = mna.node_map["base"] - 1;
    let v_base = result.v_node[base_idx];
    let ib_kcl = (12.0 - v_base) / 1.0e6;
    assert!(
        (ib - ib_kcl).abs() / ib_kcl.abs() < 0.02,
        "Ib={ib:.3e} violates base KCL (RB current {ib_kcl:.3e})"
    );

    // The solved Ib must match the leakage-carrying devices-crate model.
    let em_leak = BjtEbersMoll::new(1e-14, 0.02585, 400.0, 3.0, BjtPolarity::Npn)
        .with_leakage(1e-11, 2.0, 0.0, 2.0);
    let ib_model = em_leak.base_current(vbe, vbc);
    assert!(
        (ib - ib_model).abs() / ib_model.abs() < 1e-6,
        "Ib={ib:.6e} disagrees with leakage model Ib={ib_model:.6e}"
    );

    // Prove the card is ISE-significant: the ideal-only model at the same
    // junction voltages accounts for well under the total base current.
    let em_ideal = BjtEbersMoll::new(1e-14, 0.02585, 400.0, 3.0, BjtPolarity::Npn);
    let ib_ideal = em_ideal.base_current(vbe, vbc);
    assert!(
        ib_ideal < 0.75 * ib,
        "test card not ISE-significant: ideal Ib={ib_ideal:.3e} vs total {ib:.3e}"
    );

    // Bias-point regression: leakage steals base drive, so Vbe (and Ic) are
    // LOWER than the leakage-free solve. Pre-fix values: Vbe=0.694,
    // V(coll)=7.48 V. Post-fix: Vbe=0.679, V(coll)=9.48 V.
    assert!(
        vbe < 0.687,
        "Vbe={vbe:.4} — leakage omitted? (pre-fix converged to 0.694)"
    );
    let coll_idx = mna.node_map["coll"] - 1;
    let v_coll = result.v_node[coll_idx];
    assert!(
        (v_coll - 9.48).abs() < 0.3,
        "V(coll)={v_coll:.3}, expected ~9.48 V (pre-fix: 7.48 V)"
    );
}

/// Power-BJT parasitic emitter resistance: the pre-fix 3-iteration undamped
/// fixed point oscillates between cutoff and saturation whenever
/// gm·(RE + RB/β) ≳ 1 (Ic > VT/RE — 2.6 mA at RE=10 Ω). Trigger case
/// (IS=1e-14, RE=10 Ω, Vbe_ext=0.75 V) bounced vbe_int 0.750 ↔ 0.350 and
/// returned Ic=7.4e-9 A (cutoff) where the true solution is ~9 mA.
/// The fix mirrors the runtime's damped 2D Newton (`bjt_with_parasitics`).
#[test]
fn test_parasitic_bjt_internal_newton_converges() {
    use melange_devices::bjt::{BjtEbersMoll, BjtPolarity};
    use melange_solver::dc_op::evaluate_devices;

    let re = 10.0;
    let vt = 0.02585;
    let slot = DeviceSlot {
        device_type: DeviceType::Bjt,
        start_idx: 0,
        dimension: 2,
        params: DeviceParams::Bjt(BjtParams {
            is: 1e-14,
            vt,
            beta_f: 200.0,
            beta_r: 3.0,
            is_pnp: false,
            vaf: f64::INFINITY,
            var: f64::INFINITY,
            ikf: f64::INFINITY,
            ikr: f64::INFINITY,
            cje: 0.0,
            cjc: 0.0,
            tf: 0.0,
            vje: 0.75,
            mje: 0.33,
            vjc: 0.75,
            mjc: 0.33,
            fc: 0.5,
            nf: 1.0,
            nr: 1.0,
            ise: 0.0,
            ne: 1.5,
            isc: 0.0,
            nc: 2.0,
            rb: 0.0,
            rc: 0.0,
            re,
            rth: f64::INFINITY,
            cth: 1e-3,
            xti: 3.0,
            eg: 1.11,
            tamb: 300.15,
        }),
        has_internal_mna_nodes: false,
        vg2k_frozen: 0.0,
    };

    let m = 2;
    let eval = |vbe_ext: f64, vbc_ext: f64| -> (f64, f64, Vec<f64>) {
        let v_nl = vec![vbe_ext, vbc_ext];
        let mut i_nl = vec![0.0; m];
        let mut j_dev = vec![0.0; m * m];
        evaluate_devices(&v_nl, std::slice::from_ref(&slot), &mut i_nl, &mut j_dev, m);
        (i_nl[0], i_nl[1], j_dev)
    };

    let (ic, ib, j_dev) = eval(0.75, -5.0);
    assert!(ic.is_finite() && ib.is_finite());

    // Beyond the old divergence threshold (VT/RE = 2.6 mA), in the multi-mA
    // range the fixed point could never reach. True solution: vbe_int ≈ 0.66,
    // Ic ≈ 9 mA. Pre-fix returned Ic = 7.4e-9 A.
    assert!(
        ic > 2.6e-3 && ic < 2.0e-2,
        "Ic={ic:.3e} implausible (expected ~9 mA; pre-fix oscillation gave 7.4e-9 A)"
    );

    // Self-consistency of the internal solve: reconstruct the internal
    // junction voltages the converged currents imply, and re-evaluate the
    // intrinsic model there. Under the old oscillating fixed point this
    // mismatches by many orders of magnitude.
    let vbe_int = 0.75 - (ic + ib) * re;
    let vbc_int = -5.0;
    let em = BjtEbersMoll::new(1e-14, vt, 200.0, 3.0, BjtPolarity::Npn);
    let ic_check = em.collector_current(vbe_int, vbc_int);
    let ib_check = em.base_current(vbe_int, vbc_int);
    assert!(
        (ic - ic_check).abs() / ic_check.abs() < 1e-6,
        "internal solve inconsistent: Ic={ic:.6e} vs intrinsic({vbe_int:.4}) = {ic_check:.6e}"
    );
    assert!(
        (ib - ib_check).abs() / ib_check.abs() < 1e-6,
        "internal solve inconsistent: Ib={ib:.6e} vs intrinsic = {ib_check:.6e}"
    );

    // External Jacobian must be the chain-rule (J_dev · J_F⁻¹) derivative of
    // the composed device — verify dIc/dVbe_ext against a central difference.
    let h = 1e-7;
    let (ic_p, _, _) = eval(0.75 + h, -5.0);
    let (ic_m, _, _) = eval(0.75 - h, -5.0);
    let dic_fd = (ic_p - ic_m) / (2.0 * h);
    let dic_jac = j_dev[0]; // j_dev[0*m + 0] = dIc/dVbe
    assert!(
        (dic_jac - dic_fd).abs() / dic_fd.abs() < 1e-4,
        "external Jacobian dIc/dVbe={dic_jac:.6e} vs finite-diff {dic_fd:.6e} \
         (raw internal Jacobian would be ~{:.0}x larger)",
        1.0 + em.collector_jacobian(vbe_int, vbc_int).0 * re
    );
}

/// Diode with RS: DC OP must use the same junction solve as the transient
/// runtime (V_j + I·RS = V), not the bare Shockley law at the terminal
/// voltage. VCC=5 V through 1k into a 1N4148-like diode with RS=100:
///   before (RS ignored): V(out)=0.651 V, I=4.35 mA
///   after  (junction solve): V(out)=1.042 V, I=3.96 mA
const DIODE_RS_VCC: &str = "Diode RS with VCC
VCC vcc 0 DC 5
R1 vcc out 1k
D1 out 0 DRS
C1 in out 10u
C2 out 0 1u
.MODEL DRS D(IS=2.52e-9 N=1.752 RS=100)
";

#[test]
fn test_diode_rs_dc_op_matches_devices_crate() {
    use melange_devices::diode::{DiodeShockley, DiodeWithRs};

    let (netlist, mna, _) = build_pipeline(DIODE_RS_VCC);
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
    match &slots[0].params {
        DeviceParams::Diode(dp) => {
            assert!((dp.rs - 100.0).abs() < 1e-9, "harness failed to parse RS");
        }
        _ => panic!("expected diode slot"),
    }

    let result = solve_dc_operating_point(&mna, &slots, &config);
    assert!(
        result.converged,
        "DC OP failed to converge: {:?}",
        result.method
    );

    let out_idx = mna.node_map["out"] - 1;
    let v_out = result.v_node[out_idx];
    let i_d = result.i_nl[0];

    // KCL: the 1k feed delivers exactly the diode current (device Gmin is
    // 1e-12 S — negligible at mA scale).
    let i_kcl = (5.0 - v_out) / 1000.0;
    assert!(
        (i_d - i_kcl).abs() / i_kcl.abs() < 1e-3,
        "diode current {i_d:.4e} violates KCL ({i_kcl:.4e})"
    );

    // The converged point must satisfy the devices-crate DiodeWithRs law —
    // the same junction solve the generated runtime uses.
    let n_vt = 1.752 * 0.02585;
    let drs = DiodeWithRs::new(DiodeShockley::new(2.52e-9, 1.0, n_vt), 100.0);
    let i_model = drs.current_at(v_out);
    assert!(
        (i_d - i_model).abs() / i_model.abs() < 1e-6,
        "i_nl={i_d:.6e} disagrees with DiodeWithRs({v_out:.4}) = {i_model:.6e}"
    );

    // Bias-point regression: RS drops ~0.4 V at 4 mA. Pre-fix (RS ignored)
    // converged to V(out)=0.651 V; the correct point is 1.042 V.
    assert!(
        (v_out - 1.042).abs() < 0.02,
        "V(out)={v_out:.4} V, expected ~1.042 V (pre-fix: 0.651 V)"
    );
}

// ── DC/runtime parity: MOSFET body effect ────────────────────────────

/// The generated runtime shifts VT per-sample from Vsb (dk_emitter.rs
/// body_effect_update, nodal_emitter.rs vt_eff — channel-signed
/// magnitude-space GAMMA/PHI formula). The DC OP must evaluate the SAME
/// body-shifted device, not the nominal-VT one.
#[test]
fn test_dc_op_mosfet_body_effect_parity() {
    use melange_devices::mosfet::{ChannelType, Mosfet};
    use melange_solver::dc_op::{evaluate_devices, evaluate_devices_with_nodes};
    use melange_solver::device_types::MosfetParams;

    let make_slot = |gamma: f64| DeviceSlot {
        device_type: DeviceType::Mosfet,
        start_idx: 0,
        dimension: 2,
        params: DeviceParams::Mosfet(MosfetParams {
            kp: 2e-3,
            vt: 1.0,
            lambda: 0.01,
            is_p_channel: false,
            cgs: 0.0,
            cgd: 0.0,
            rd: 0.0,
            rs: 0.0,
            gamma,
            phi: 0.7,
            // 1-based MNA node indices: source = node 1, bulk = node 2.
            source_node: 1,
            bulk_node: 2,
        }),
        has_internal_mna_nodes: false,
        vg2k_frozen: 0.0,
    };

    let m = 2;
    let vds = 3.0;
    let vgs = 2.5;
    let v_nl = vec![vds, vgs]; // dim 0 = Vds, dim 1 = Vgs
    // Source elevated 1.5 V above grounded bulk → Vsb = 1.5 V.
    let v_node = vec![1.5, 0.0, 4.5];

    let mut i_nl = vec![0.0; m];
    let mut j_dev = vec![0.0; m * m];
    evaluate_devices_with_nodes(
        &v_nl,
        &[make_slot(0.5)],
        &mut i_nl,
        &mut j_dev,
        m,
        &v_node,
    );

    // Hand-computed body-shifted evaluation (same formula the emitters bake):
    //   vt_eff = VT + GAMMA·(√(PHI + Vsb) − √PHI)   (NMOS: sign = +1)
    let vt_eff = 1.0 + 0.5 * ((0.7f64 + 1.5).sqrt() - 0.7f64.sqrt());
    let shifted = Mosfet::new(ChannelType::N, vt_eff, 2e-3, 0.01);
    let id_expected = shifted.drain_current(vgs, vds);
    let (gm_expected, gds_expected) = shifted.jacobian_partial(vgs, vds);
    assert!(
        (i_nl[0] - id_expected).abs() < 1e-15,
        "DC Id={:.9e} must match body-shifted device {:.9e}",
        i_nl[0],
        id_expected
    );
    assert!(
        (j_dev[0] - gds_expected).abs() < 1e-15 && (j_dev[1] - gm_expected).abs() < 1e-15,
        "DC Jacobian must come from the body-shifted device"
    );
    assert!(i_nl[1].abs() < 1e-30, "insulated gate must carry no current");

    // And it must DIFFER from the GAMMA-ignored evaluation (the pre-fix bug).
    let nominal = Mosfet::new(ChannelType::N, 1.0, 2e-3, 0.01);
    let id_nominal = nominal.drain_current(vgs, vds);
    assert!(
        (i_nl[0] - id_nominal).abs() > 0.05 * id_nominal.abs(),
        "body-shifted Id={:.6e} should differ measurably from nominal {:.6e}",
        i_nl[0],
        id_nominal
    );

    // GAMMA=0 with nodes ≡ no-body-effect path: exact no-op.
    let mut i_g0 = vec![0.0; m];
    let mut j_g0 = vec![0.0; m * m];
    evaluate_devices_with_nodes(&v_nl, &[make_slot(0.0)], &mut i_g0, &mut j_g0, m, &v_node);
    assert!(
        (i_g0[0] - id_nominal).abs() < 1e-15,
        "GAMMA=0 must reproduce the nominal-VT device exactly"
    );

    // Legacy no-node-vector entry point falls back to nominal VT (Vsb = 0
    // is the only case where that is exact — callers with elevated sources
    // must use evaluate_devices_with_nodes).
    let mut i_legacy = vec![0.0; m];
    let mut j_legacy = vec![0.0; m * m];
    evaluate_devices(&v_nl, &[make_slot(0.5)], &mut i_legacy, &mut j_legacy, m);
    assert!(
        (i_legacy[0] - id_nominal).abs() < 1e-15,
        "no-node-vector fallback must be the nominal-VT evaluation"
    );
}

// ── DC/runtime parity: VCA THD coefficient ───────────────────────────

/// The generated runtime evaluates the VCA with DEVICE_THD (gain-dependent
/// cubic distortion). `Vca::new` silently zeroes thd; the DC OP must use
/// the thd-carrying constructor so a DC-biased signal port lands on the
/// same fixed point the transient converges to.
#[test]
fn test_dc_op_vca_thd_parity() {
    use melange_devices::Vca;
    use melange_solver::dc_op::evaluate_devices;
    use melange_solver::device_types::VcaParams;

    let vscale = 0.1;
    let g0 = 1e-3;
    let thd = 0.5;
    let slot = DeviceSlot {
        device_type: DeviceType::Vca,
        start_idx: 0,
        dimension: 2,
        params: DeviceParams::Vca(VcaParams { vscale, g0, thd }),
        has_internal_mna_nodes: false,
        vg2k_frozen: 0.0,
    };

    let m = 2;
    // DC-biased signal port + control voltage that puts gain below unity
    // (thd_factor = thd·(1 − gain) is only non-zero there).
    let v_sig = 0.8;
    let v_ctrl = 0.05;
    let v_nl = vec![v_sig, v_ctrl];
    let mut i_nl = vec![0.0; m];
    let mut j_dev = vec![0.0; m * m];
    evaluate_devices(&v_nl, &[slot], &mut i_nl, &mut j_dev, m);

    // Must match the devices-crate evaluation WITH thd (what the runtime does)…
    let vca_thd = Vca::new_with_thd(vscale, g0, thd);
    let i_expected = vca_thd.current(v_sig, v_ctrl);
    let jac_expected = vca_thd.jacobian(v_sig, v_ctrl);
    assert!(
        (i_nl[0] - i_expected).abs() < 1e-18,
        "DC I_sig={:.9e} must match thd-carrying Vca {:.9e}",
        i_nl[0],
        i_expected
    );
    assert!(
        (j_dev[0] - jac_expected[0]).abs() < 1e-15
            && (j_dev[1] - jac_expected[1]).abs() < 1e-15,
        "DC Jacobian must match thd-carrying Vca"
    );

    // …and differ from the thd-less evaluation (the pre-fix bug).
    let i_ideal = Vca::new(vscale, g0).current(v_sig, v_ctrl);
    assert!(
        (i_nl[0] - i_ideal).abs() > 1e-3 * i_ideal.abs(),
        "thd={thd} at gain<1 must shift the DC current (got {:.6e} vs ideal {:.6e})",
        i_nl[0],
        i_ideal
    );
}

// ── Behavioral B-sources: honest DC-OP limitation warning ────────────

/// Behavioral sources have no DC model yet (BEHAVIORAL_SOURCES.md §6
/// follow-up): I={} is treated as open, V={} rows enforce V=0. The DC OP
/// must say so — one warning naming every source — instead of silently
/// biasing the operating point.
#[test]
fn test_dc_op_behavioral_source_warning() {
    use melange_solver::dc_op::behavioral_dc_op_warning;

    // I={} source.
    let spice = "\
Behavioral DC warning test
Va a 0 DC 0.6
B1 out 0 I={ tanh(V(a)) }
Rout out 0 1
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA failed");
    let msg = behavioral_dc_op_warning(&mna).expect("must warn when B-sources are present");
    assert!(msg.contains("B1"), "warning must name the source: {msg}");
    assert!(
        msg.contains("I=0"),
        "warning must state the I={{}} DC treatment: {msg}"
    );
    assert!(
        msg.contains("BEHAVIORAL_SOURCES.md"),
        "warning must point at the recorded follow-up: {msg}"
    );

    // V={} source gets the V=0 wording.
    let spice_v = "\
Behavioral DC warning test (V)
Va a 0 DC 0.6
B2 out 0 V={ tanh(V(a)) }
Rout out 0 1
";
    let netlist_v = Netlist::parse(spice_v).expect("parse failed");
    let mna_v = MnaSystem::from_netlist(&netlist_v).expect("MNA failed");
    let msg_v = behavioral_dc_op_warning(&mna_v).expect("must warn for V={} too");
    assert!(msg_v.contains("B2"), "warning must name the source: {msg_v}");
    assert!(
        msg_v.contains("V=0"),
        "warning must state the V={{}} DC treatment: {msg_v}"
    );

    // The warning path must not disturb the solve itself (linear circuit:
    // solver still returns a well-formed result).
    let result = solve_dc_operating_point(&mna, &[], &DcOpConfig::default());
    assert!(result.converged, "linear DC OP must still converge");
    assert!(result.v_node.iter().all(|x| x.is_finite()));

    // No behavioral sources → no warning.
    let spice_plain = "\
Plain RC
Vin in 0 DC 1
R1 in out 1k
R2 out 0 1k
";
    let netlist_p = Netlist::parse(spice_plain).expect("parse failed");
    let mna_p = MnaSystem::from_netlist(&netlist_p).expect("MNA failed");
    assert!(
        behavioral_dc_op_warning(&mna_p).is_none(),
        "must not warn on circuits without behavioral sources"
    );
}
