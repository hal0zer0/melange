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
                    stateful: None,
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
                        xtb: 0.0,
                        eg: 1.11,
                        tamb: 300.15,
                    }),
                    has_internal_mna_nodes: false,
                    vg2k_frozen: 0.0,
                    stateful: None,
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
            xtb: 0.0,
            eg: 1.11,
            tamb: 300.15,
        }),
        has_internal_mna_nodes: false,
        vg2k_frozen: 0.0,
        stateful: None,
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
        stateful: None,
    };

    let m = 2;
    let vds = 3.0;
    let vgs = 2.5;
    let v_nl = vec![vds, vgs]; // dim 0 = Vds, dim 1 = Vgs
                               // Source elevated 1.5 V above grounded bulk → Vsb = 1.5 V.
    let v_node = vec![1.5, 0.0, 4.5];

    let mut i_nl = vec![0.0; m];
    let mut j_dev = vec![0.0; m * m];
    evaluate_devices_with_nodes(&v_nl, &[make_slot(0.5)], &mut i_nl, &mut j_dev, m, &v_node);

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
    assert!(
        i_nl[1].abs() < 1e-30,
        "insulated gate must carry no current"
    );

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
        stateful: None,
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
        (j_dev[0] - jac_expected[0]).abs() < 1e-15 && (j_dev[1] - jac_expected[1]).abs() < 1e-15,
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
    assert!(
        msg_v.contains("B2"),
        "warning must name the source: {msg_v}"
    );
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

// =============================================================================
// Regression tests: DC OP NR machinery fixes (2026-07-18)
//   1. pnjlim correction distribution pseudo-inverse normalization
//   2. SPICE-style RELTOL/ABSTOL convergence (300 V circuits)
//   3. is_voltage_row: inductor branch currents excluded from clamp/damping
//   4. fetlim dispatch for JFET/MOSFET at DC
//   5. unified degeneracy predicate + no-DC-source skip
//   6. singular linear system reported honestly
// =============================================================================

/// Fix 1: floating series diode pair. D1 (a→b) has a floating N_v row
/// ([+1, −1], ‖row‖² = 2), so the pre-fix raw-transpose pnjlim distribution
/// applied DOUBLE the limiting correction to that junction — a strong limiting
/// event became a REVERSED step. Measured pre-fix behavior on this circuit:
/// the reversed step flips D1 into reverse bias and NR "converges" DirectNr
/// to a NON-PHYSICAL equilibrium with Vf(D1) ≈ −96.5 V (wrong basin — would
/// have poisoned DC_NL_I). With the pseudo-inverse normalization, Direct NR
/// must land on the physical series-pair OP; the Vf/current assertions below
/// are the discriminator.
#[test]
fn test_floating_series_diode_pair_direct_nr() {
    let spice = "Floating Series Diode Pair
VCC vcc 0 DC 100
R1 vcc a 100
D1 a b DMOD
D2 b 0 DMOD
C1 b 0 1u
.MODEL DMOD D(IS=2.52e-9 N=1.752)
";
    let (netlist, mna, _) = build_pipeline(spice);
    let config = DcOpConfig {
        input_node: mna
            .node_map
            .get("in")
            .copied()
            .unwrap_or(1)
            .saturating_sub(1),
        input_resistance: 1.0,
        // Iteration budget: the normalized distribution needs ~34 pnjlim-paced
        // iterations (each iteration multiplies the diode current by ~e while
        // climbing from the clamped 0.6 V seed to the 0.9 V high-current Vf).
        max_iterations: 60,
        ..DcOpConfig::default()
    };
    let slots = build_device_slots(&netlist, &mna);
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(result.converged, "series diode pair DC OP must converge");
    assert_eq!(
        result.method,
        DcOpMethod::DirectNr,
        "must converge via Direct NR (not fall through to stepping), got {:?} in {} iters",
        result.method,
        result.iterations
    );

    // Physics: I ≈ (100 − 2·Vf)/100 ≈ 0.98 A, Vf(≈1 A, IS=2.52e-9, n=1.752)
    // ≈ n·Vt·ln(I/IS) ≈ 0.045·19.8 ≈ 0.89 V per diode.
    let node = |name: &str| -> f64 {
        let idx = mna.node_map.get(name).copied().unwrap() - 1;
        result.v_node[idx]
    };
    let (v_a, v_b) = (node("a"), node("b"));
    let vf1 = v_a - v_b;
    let vf2 = v_b;
    assert!(
        vf1 > 0.7 && vf1 < 1.1,
        "floating diode Vf out of range: {vf1}"
    );
    assert!(
        vf2 > 0.7 && vf2 < 1.1,
        "grounded diode Vf out of range: {vf2}"
    );
    // Both diodes carry the same series current: I = (100 − V(a))/100.
    let i_series = (100.0 - v_a) / 100.0;
    for (k, &i) in result.i_nl.iter().enumerate() {
        assert!(
            (i - i_series).abs() < 1e-6 * i_series.abs().max(1.0),
            "diode {k} current {i} != series current {i_series}"
        );
    }
}

/// Fix 2: 300 V rails. The pre-fix convergence check was absolute-only
/// (|delta| < 1e-9 for every row), which demands ~3 ppt relative precision on
/// a 300 V node — at or below the LU round-off floor for ill-scaled systems.
/// The SPICE-style `|delta_i| < reltol·|v_i| + abstol` check (reltol = 1e-6)
/// must let the scaled circuit converge via Direct NR with the same junction
/// physics as the 5 V version.
#[test]
fn test_high_voltage_divider_diode_direct_nr() {
    // Wide resistance spread (1meg : 10) on a 300 V rail pushes the LU
    // round-off floor on the high-voltage nodes above the old 1e-9 absolute
    // tolerance — the pre-fix absolute-only check false-fails here.
    let spice = "HV Diode with 300V rail
VCC vcc 0 DC 300
R1 vcc mid 1meg
R2 mid out 10
D1 out 0 DMOD
C1 out 0 1u
.MODEL DMOD D(IS=2.52e-9 N=1.752)
";
    let (netlist, mna, _) = build_pipeline(spice);
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

    assert!(result.converged, "300 V circuit DC OP must converge");
    assert_eq!(
        result.method,
        DcOpMethod::DirectNr,
        "300 V circuit must converge via Direct NR, got {:?} in {} iters",
        result.method,
        result.iterations
    );
    let node = |name: &str| -> f64 {
        let idx = mna.node_map.get(name).copied().unwrap() - 1;
        result.v_node[idx]
    };
    assert!(
        (node("vcc") - 300.0).abs() < 1e-3,
        "VCC node must sit at 300 V, got {}",
        node("vcc")
    );
    let v_out = node("out");
    assert!(
        v_out > 0.4 && v_out < 1.0,
        "diode forward drop out of range at 300 V rails: {v_out}"
    );
    // Diode current ≈ (300 − Vf)/1meg ≈ 0.3 mA
    assert!(
        result.i_nl.iter().any(|&i| (i - 0.3e-3).abs() < 0.05e-3),
        "expected ≈0.3 mA diode current, got {:?}",
        result.i_nl
    );
}

/// Fix 3: when MNA-level BJT internal nodes exist, the pre-fix damping loop
/// classified every row `i >= internal_node_start` as a voltage row — sweeping
/// the inductor DC-short branch-current rows (amperes!) into the ±50 "V" flat
/// clamp and the 10 V damping scan. A branch current that legitimately exceeds
/// 50 A then needed |I|/50 clamped iterations (with 0.1× damping poisoning all
/// node updates on the way), blowing the iteration budget. With the explicit
/// is_voltage_row classification, Direct NR must converge with the branch
/// current landing at its physical value in one clean Newton trajectory.
#[test]
fn test_inductor_branch_current_not_clamped_with_internal_nodes() {
    use melange_solver::device_types::BjtParams;

    let spice = "Inductor big branch current + parasitic BJT
VCC vcc 0 DC 300
L1 vcc a 10m
D1 a b DMOD
RS b 0 0.05
R1 vcc base 300k
R2 base 0 22k
Q1 coll base emit QPAR
RC vcc coll 6.8k
RE emit 0 1k
C1 in base 10u
.MODEL DMOD D(IS=2.52e-9 N=1.752)
.MODEL QPAR NPN(IS=1.8e-14 BF=200 BR=10)
";
    let netlist = Netlist::parse(spice).expect("parse failed");
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

    // Build slots, then give the BJT parasitic resistances and expand
    // MNA-level internal nodes (the transient/nodal pipeline does this via
    // expand_bjt_internal_nodes) so the DC system sees mna.bjt_internal_nodes.
    let mut slots = build_device_slots(&netlist, &mna);
    for slot in slots.iter_mut() {
        if let DeviceParams::Bjt(BjtParams { rb, rc, re, .. }) = &mut slot.params {
            *rb = 100.0;
            *rc = 10.0;
            *re = 1.0;
        }
    }
    mna.expand_bjt_internal_nodes(&slots);
    assert!(
        !mna.bjt_internal_nodes.is_empty(),
        "test setup: MNA internal nodes must exist to exercise the conflation"
    );
    for slot in slots.iter_mut() {
        if matches!(slot.device_type, DeviceType::Bjt) {
            slot.has_internal_mna_nodes = true;
        }
    }

    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        // Pre-fix, the ~5900 A branch current is clamped to 50 per iteration:
        // ≥118 iterations just for the branch row. 80 iterations is generous
        // for the fixed solver and unreachable for the clamped one.
        max_iterations: 80,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(
        result.converged,
        "inductor + parasitic-BJT DC OP must converge (method {:?}, {} iters)",
        result.method, result.iterations
    );
    assert_eq!(
        result.method,
        DcOpMethod::DirectNr,
        "must converge via Direct NR, got {:?} in {} iters",
        result.method,
        result.iterations
    );

    // The inductor is a DC short: I ≈ (300 − Vf(D1)) / 0.05 Ω ≈ 5.97 kA.
    // Its branch-current row is the first row after mna.n_aug (post-expansion).
    let branch_idx = mna.n_aug;
    assert!(
        branch_idx < result.v_node.len(),
        "branch row {branch_idx} must be inside the DC solution vector ({})",
        result.v_node.len()
    );
    let i_branch = result.v_node[branch_idx].abs();
    assert!(
        i_branch > 50.0,
        "branch current must legitimately exceed the old 50 'V' clamp, got {i_branch}"
    );
    assert!(
        (i_branch - 5.97e3).abs() < 0.05 * 5.97e3,
        "branch current must land near (300 − Vf)/0.05 ≈ 5.97 kA, got {i_branch}"
    );

    // The BJT bias must be sane too (base divider ≈ 20.5 V, Vbe ≈ 0.7).
    let vbe = result.v_nl[slots
        .iter()
        .find(|s| matches!(s.device_type, DeviceType::Bjt))
        .unwrap()
        .start_idx];
    assert!(
        vbe > 0.5 && vbe < 0.9,
        "BJT Vbe out of range with internal nodes: {vbe}"
    );
}

/// Fix 4: JFET fetlim dispatch at DC. A self-biased N-JFET source follower
/// whose linear guess starts far from the operating point must converge via
/// Direct NR with the fetlim-limited trajectory, landing on the physical
/// self-bias point (Vgs between Vp and 0, Id between 0 and IDSS).
#[test]
fn test_jfet_self_bias_dc_op_with_fetlim() {
    use melange_solver::device_types::JfetParams;

    let spice = "JFET self bias
VCC vcc 0 DC 24
J1 drain gate src JMOD
RD vcc drain 2.2k
RG gate 0 1meg
RSRC src 0 1k
C1 in gate 10u
.MODEL JMOD NJF(VTO=-2.0 BETA=1.25e-3 LAMBDA=0)
";
    let netlist = Netlist::parse(spice).expect("parse failed");
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

    // Manual slot: build_device_slots doesn't cover JFETs.
    // IDSS = BETA·VTO² = 1.25e-3·4 = 5 mA.
    let slots = vec![DeviceSlot {
        device_type: DeviceType::Jfet,
        start_idx: 0,
        dimension: 2,
        params: DeviceParams::Jfet(JfetParams {
            idss: 5e-3,
            vp: -2.0,
            lambda: 0.0,
            is_p_channel: false,
            cgs: 0.0,
            cgd: 0.0,
            rd: 0.0,
            rs: 0.0,
        }),
        has_internal_mna_nodes: false,
        vg2k_frozen: 0.0,
        stateful: None,
    }];

    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &slots, &config);

    assert!(result.converged, "JFET self-bias DC OP must converge");
    assert_eq!(
        result.method,
        DcOpMethod::DirectNr,
        "JFET self-bias must converge via Direct NR, got {:?} in {} iters",
        result.method,
        result.iterations
    );
    // v_nl layout: dim 0 = Vds, dim 1 = Vgs.
    let (vds, vgs) = (result.v_nl[0], result.v_nl[1]);
    assert!(
        vgs > -2.0 && vgs < 0.0,
        "self-bias Vgs must sit between Vp and 0, got {vgs}"
    );
    assert!(vds > 0.0, "N-JFET Vds must be positive, got {vds}");
    // Self-bias: Id·RSRC = −Vgs → Id = −Vgs/1k, and Id = IDSS·(1 − Vgs/Vp)².
    let id = result.i_nl[0];
    assert!(
        (id - (-vgs / 1e3)).abs() < 1e-6,
        "Id {id} must satisfy the self-bias constraint −Vgs/RSRC = {}",
        -vgs / 1e3
    );
    assert!(
        id > 0.0 && id < 5e-3,
        "Id must be between 0 and IDSS, got {id}"
    );
}

/// Fix 5: a diode circuit with NO DC sources (b_dc all zero). The all-off
/// solution IS the true operating point, so the degeneracy check must be
/// skipped and Direct NR must be accepted immediately — pre-fix, Strategy 1
/// rejected the (correct) all-off solution as degenerate and burned source
/// stepping iterations to arrive at the same answer.
#[test]
fn test_source_free_diode_clipper_accepts_all_off() {
    let spice = "Source-free diode clipper
R1 in a 10k
D1 a 0 DMOD
D2 0 a DMOD
C1 a 0 10n
.MODEL DMOD D(IS=2.52e-9 N=1.752)
";
    let (netlist, mna, _) = build_pipeline(spice);
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

    assert!(result.converged, "source-free clipper DC OP must converge");
    assert_eq!(
        result.method,
        DcOpMethod::DirectNr,
        "all-off is the true OP for a source-free circuit — Direct NR must be \
         accepted, not rejected as degenerate (got {:?})",
        result.method
    );
    assert!(
        result.v_node.iter().all(|&v| v.abs() < 1e-6),
        "source-free OP must be ~0 V everywhere, got {:?}",
        result.v_node
    );
    assert!(
        result.i_nl.iter().all(|&i| i.abs() < 1e-9),
        "source-free OP must have ~zero device currents, got {:?}",
        result.i_nl
    );
}

/// Fix 6: a structurally singular linear DC system (two parallel voltage
/// sources with conflicting values → linearly dependent constraint rows) must
/// be reported as converged = false with the distinct SingularLinear method —
/// pre-fix it silently returned all-zeros with converged = true.
#[test]
fn test_singular_linear_system_reported_not_converged() {
    let spice = "Conflicting parallel voltage sources
V1 a 0 DC 5
V2 a 0 DC 3
R1 a 0 1k
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA failed");

    let config = DcOpConfig::default();
    let result = solve_dc_operating_point(&mna, &[], &config);

    assert!(
        !result.converged,
        "singular linear system must NOT be reported as converged"
    );
    assert_eq!(
        result.method,
        DcOpMethod::SingularLinear,
        "singular linear system must use the distinct SingularLinear method, got {:?}",
        result.method
    );
    assert!(
        result.v_node.iter().all(|&v| v == 0.0),
        "singular fallback returns zeros"
    );
}

/// Fix-point invariance guard: the pnjlim normalization (fix 1) and the
/// RELTOL convergence check (fix 2) change NR trajectories, not fixed points.
/// The canonical BJT common-emitter bias must land on the SAME operating point
/// as before (values from the pre-fix solver / DC_OP.md verification table).
#[test]
fn test_bjt_ce_fixed_point_unchanged_by_nr_fixes() {
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
    assert!(result.converged);

    let node = |name: &str| -> f64 {
        let idx = mna.node_map.get(name).copied().unwrap() - 1;
        result.v_node[idx]
    };
    // DC_OP.md expected values: V(base) ≈ 2.1 V (divider loaded by Ib),
    // V(emit) ≈ V(base) − 0.65, V(coll) ≈ 12 − Ic·6.8k.
    let (v_base, v_emit, v_coll) = (node("base"), node("emit"), node("coll"));
    assert!(
        (v_base - 2.1).abs() < 0.2,
        "V(base) moved: expected ≈2.1 V, got {v_base}"
    );
    assert!(
        (v_base - v_emit - 0.65).abs() < 0.08,
        "Vbe moved: expected ≈0.65 V, got {}",
        v_base - v_emit
    );
    let ic = v_emit / 1000.0; // I_E ≈ I_C through RE=1k
    let v_coll_expected = 12.0 - ic * 6800.0;
    assert!(
        (v_coll - v_coll_expected).abs() < 0.3,
        "V(coll) inconsistent with Ic·RC: got {v_coll}, expected ≈{v_coll_expected}"
    );
}

// =============================================================================
// Germanium PNP common-emitter: Direct-NR degeneracy-threshold regression
// =============================================================================
//
// Bug: `solution_has_active_junction` used a hardcoded 0.3V "is this
// junction on" threshold, tuned for silicon (Vf ~ 0.6-0.7V). Germanium
// devices (IS ~ 1e-7 to 1e-6) turn on at a much lower forward voltage
// (~0.15-0.3V), so Direct NR's correct, converged answer was misclassified
// as "degenerate" for EVERY R_E in this circuit and discarded in favor of
// source/Gmin stepping. Gmin stepping then diverged outright (raw internal
// junction voltage overshoots the exponential knee with no pnjlim
// protection for internal-node BJTs, driving i_nl to ~1e11 mA) for R_E in
// roughly [1.4k, 2.5k], while happening to land back on the correct answer
// via Gmin stepping outside that band — a convergence "window" that fails
// with success on both sides, the classic homotopy-branch-loss signature.
//
// Fix: scale the threshold by the device's own `pn_vcrit` (same quantity
// pnjlim uses) instead of a fixed silicon-typical voltage.
const GE_PNP_STAGE_TEMPLATE: &str = "TN2-clone minimal stage
C_dummy in b1 1n
Vrail rail 0 DC 8
R_e rail e1 {RE}
Q1 c1 b1 e1 OC74
R_b_up rail b1 5.8k
R_b_dn b1 0 47k
R_c c1 0 1k
R_out c1 out 100k
R_outref out 0 1Meg
.model OC74 PNP(IS=3e-7 BF=90 VAF=50 RB=40 RC=4 RE=1 CJE=80p CJC=30p TF=2n)
.end
";

fn solve_ge_pnp_stage(re_ohms: f64) -> (MnaSystem, melange_solver::dc_op::DcOpResult) {
    let spice = GE_PNP_STAGE_TEMPLATE.replace("{RE}", &format!("{re_ohms}"));
    let netlist = Netlist::parse(&spice).expect("parse failed");
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
    let device_slots =
        CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap_or_default();
    let config = DcOpConfig {
        input_node,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    let result = solve_dc_operating_point(&mna, &device_slots, &config);
    (mna, result)
}

/// Direct repro of the confirmed bug: R_E = 1.5k sat squarely in the
/// pre-fix failure window (raw v(e1) pinned to the 8V rail, i_nl runaway
/// to ~1e11 mA). This must converge to the textbook Ge bias point.
#[test]
fn test_ge_pnp_common_emitter_re_1500_converges() {
    let (mna, result) = solve_ge_pnp_stage(1500.0);
    assert!(
        result.converged,
        "DC OP failed to converge at R_E=1500 ohms (method={:?}, iters={})",
        result.method, result.iterations
    );
    let e1_idx = mna.node_map["e1"] - 1;
    let v_e1 = result.v_node[e1_idx];
    // Pre-fix failure state: v(e1) pinned to the 8V rail (passive network
    // solution with the transistor electrically absent).
    assert!(
        v_e1 < 7.9,
        "v(e1)={v_e1:.4} — looks like the pre-fix rail-pinned failure state"
    );
    let i_e = (8.0 - v_e1) / 1500.0;
    // Textbook Ge PNP bias at this R_E: Ie in the few-hundred-uA range.
    assert!(
        i_e > 1e-5 && i_e < 1e-2,
        "I_E={i_e:.3e} A implausible (pre-fix runaway gave i_nl ~ 1e11 mA)"
    );
    assert!(
        result.i_nl.iter().all(|x| x.abs() < 1.0),
        "device current runaway: i_nl={:?}",
        result.i_nl
    );
}

/// Full R_E sweep spanning (and centered inside) the pre-fix failure window
/// [1.4k, 2.5k], confirming convergence to a physically consistent,
/// monotonically decreasing emitter voltage as R_E increases (more emitter
/// degeneration -> less bias current -> v(e1) closer to the rail is WRONG;
/// more R_E -> less current -> smaller (rail - v(e1)) drop -> v(e1) closer
/// to the rail is actually correct, so v(e1) should monotonically increase
/// toward 8V... but the dominant effect here is the reduced Vbe forced by
/// lower current, so verify monotonic behavior empirically rather than
/// asserting a direction, and pin the known-good textbook point).
#[test]
fn test_ge_pnp_common_emitter_re_sweep_converges() {
    let re_values = [
        1000.0, 1200.0, 1300.0, 1400.0, 1500.0, 1600.0, 1800.0, 1980.0, 2000.0, 2500.0, 2600.0,
        2700.0, 3000.0, 4700.0, 10000.0,
    ];

    let mut prev_i_e: Option<f64> = None;
    for &re in &re_values {
        let (mna, result) = solve_ge_pnp_stage(re);
        assert!(
            result.converged,
            "DC OP failed to converge at R_E={re} ohms (method={:?}, iters={})",
            result.method, result.iterations
        );
        let e1_idx = mna.node_map["e1"] - 1;
        let v_e1 = result.v_node[e1_idx];
        let i_e = (8.0 - v_e1) / re;
        assert!(
            i_e > 0.0 && i_e < 1e-2,
            "R_E={re}: I_E={i_e:.3e} A implausible"
        );
        if let Some(prev) = prev_i_e {
            assert!(
                i_e < prev,
                "R_E={re}: I_E={i_e:.3e} A not monotonically decreasing \
                 (prev={prev:.3e} A) — solver landed on a different branch"
            );
        }
        prev_i_e = Some(i_e);
    }

    // Textbook reference point cited for this stage: at R_E ~ 1.98k, Ie is
    // in the few-hundred-uA range (~0.3-0.4 mA).
    let (mna, result) = solve_ge_pnp_stage(1980.0);
    let e1_idx = mna.node_map["e1"] - 1;
    let v_e1 = result.v_node[e1_idx];
    let i_e = (8.0 - v_e1) / 1980.0;
    assert!(
        (i_e - 3.45e-4).abs() < 1e-4,
        "R_E=1980: I_E={i_e:.3e} A, expected ~0.345 mA (textbook ~0.3-0.4 mA)"
    );
}

/// Candidate-retention regression (arbiter thread 247). At R_E=10k this germanium
/// PNP has a genuine forward-active operating point — ngspice-42: v(e1)=7.262 V,
/// Ic=73 µA — but |Veb|=0.139 V sits just below the 0.5·vcrit≈0.142 V
/// active-junction threshold, so ALL three general strategies (Direct NR, source
/// stepping, Gmin) flag it "degenerate". Candidate retention is what returns
/// Strategy 1's real solution instead of failing: with Gmin now re-gated on the
/// same predicate, WITHOUT retention this circuit would be non-convergent. This
/// pins the ngspice-matching result so a refactor that drops retention (or the
/// re-gate that makes it load-bearing) regresses loudly, distinctly from the
/// broader sweep test.
#[test]
fn test_ge_pnp_low_bias_recovered_by_retention() {
    let (mna, result) = solve_ge_pnp_stage(10_000.0);
    assert!(
        result.converged,
        "retention must recover the low-bias OP (method={:?}, iters={})",
        result.method, result.iterations
    );
    let v_e1 = result.v_node[mna.node_map["e1"] - 1];
    // ngspice-42: v(e1)=7.262 V. A value near the 8 V rail would mean a degenerate
    // all-off solution (transistor electrically absent) was returned instead.
    assert!(
        (v_e1 - 7.262).abs() < 0.02,
        "v(e1) must match ngspice (7.262 V), got {v_e1:.4} — near 8 V would be the \
         degenerate all-off solution"
    );
    let i_e = (8.0 - v_e1) / 10_000.0;
    assert!(
        (i_e - 73e-6).abs() < 10e-6,
        "I_E must be ~73 µA forward-active (ngspice), got {i_e:.3e} A"
    );
}
