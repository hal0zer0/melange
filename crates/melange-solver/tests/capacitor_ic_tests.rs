//! `IC=` (capacitor initial condition) end-to-end regression tests.
//!
//! Covers the full parser → MNA → DC-OP-style initial-state solve → IR →
//! codegen (DK and nodal) → compiled-and-run pipeline for SPICE `IC=` on
//! capacitors (SPICE `.IC`/UIC semantics — see docs/aidocs/DC_OP.md and
//! CLAUDE.md's accuracy doctrine).
//!
//! Test groups:
//! 1. IR-level: `CircuitIR::v_prev_ic_seed` populated/omitted correctly,
//!    numeric values pinned against an independently hand-derived solve.
//! 2. Codegen string-level: `V_PREV_IC_SEED` constant + `v_prev`/reset()
//!    wiring present only when the circuit has IC= caps (DK and nodal).
//! 3. Compiled-and-run numeric: the RC-discharge and two-node-cap
//!    circuits actually decay from the prescribed IC in the compiled
//!    binary.
//! 4. No-IC circuits are unaffected (same codegen strings as before this
//!    feature existed).

mod support;

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::CodegenConfig;
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ---------------------------------------------------------------------------
// Circuits
// ---------------------------------------------------------------------------

/// RC discharge: cap-to-ground IC. R1=1k, C1=10u (tau=10ms), IC=5V.
/// No DC source (V1=0), so the plain (non-IC) DC OP is all zero — this
/// circuit exercises the `has_dc_op == false && has_cap_ic == true` branch.
const RC_DISCHARGE_IC: &str = "\
RC discharge IC
V1 in 0 DC 0
R1 in out 1k
C1 out 0 10u IC=5
";

/// Same topology, no IC= (control circuit for byte-for-byte regression).
const RC_DISCHARGE_NO_IC: &str = "\
RC discharge no IC
V1 in 0 DC 0
R1 in out 1k
C1 out 0 10u
";

/// Cap between two non-ground nodes in a resistive divider. IC=-4V.
const TWO_NODE_IC: &str = "\
Two-node IC test
V1 in 0 DC 0
R1 in c3 1k
R2 c3 0 10k
R3 c3 b4 5k
R4 b4 0 10k
Cx c3 b4 6n IC=-4
";

// Hand-derived (independently, via numpy) solution of the augmented linear
// system for TWO_NODE_IC: v(c3) - v(b4) = -4 exactly, with the rest of the
// network fully KCL-consistent. V1 pins v(in) to exactly 0 V (an augmented
// algebraic constraint, not a resistor), so R1 effectively becomes a 1k leg
// from c3 to ground: v(c3) = -1/3, v(b4) = 11/3. See the corresponding
// dc_op.rs unit tests for the derivation.
const TWO_NODE_V_C3: f64 = -1.0 / 3.0;
const TWO_NODE_V_B4: f64 = 11.0 / 3.0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn build_pipeline(spice: &str) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA build failed");
    let kernel = DkKernel::from_mna(&mna, 48000.0).expect("DK kernel build failed");
    (netlist, mna, kernel)
}

fn build_ir(spice: &str, config: &CodegenConfig) -> (CircuitIR, MnaSystem) {
    let (netlist, mna, kernel) = build_pipeline(spice);
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, config).unwrap();
    (ir, mna)
}

fn ic_config(output_node: usize) -> CodegenConfig {
    CodegenConfig {
        circuit_name: "ic_test".to_string(),
        sample_rate: 48000.0,
        input_node: 0,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        dc_block: false,
        ..CodegenConfig::default()
    }
}

// ---------------------------------------------------------------------------
// 1. IR-level: v_prev_ic_seed presence + numeric correctness
// ---------------------------------------------------------------------------

#[test]
fn ir_v_prev_ic_seed_absent_without_ic() {
    let (ir, _mna) = build_ir(RC_DISCHARGE_NO_IC, &ic_config(1));
    assert!(
        ir.v_prev_ic_seed.is_none(),
        "circuit without IC= caps must not populate v_prev_ic_seed"
    );
}

#[test]
fn ir_v_prev_ic_seed_present_cap_to_ground() {
    let (ir, mna) = build_ir(RC_DISCHARGE_IC, &ic_config(1));
    let seed = ir
        .v_prev_ic_seed
        .as_ref()
        .expect("IC= circuit must populate v_prev_ic_seed");
    let out_idx = mna.node_map["out"] - 1;
    assert!(
        (seed[out_idx] - 5.0).abs() < 1e-9,
        "v_prev_ic_seed[out] = {}, expected exactly 5.0",
        seed[out_idx]
    );
    // The plain (IC-free) DC OP for this circuit is all zero (no DC
    // sources) — dc_operating_point must be UNAFFECTED by the IC seed.
    assert!(
        ir.dc_operating_point.iter().all(|&v| v.abs() < 1e-12),
        "dc_operating_point must stay the pure IC-free DC bias point: {:?}",
        ir.dc_operating_point
    );
}

#[test]
fn ir_v_prev_ic_seed_two_node_cap() {
    let (netlist, mna, kernel) = build_pipeline(TWO_NODE_IC);
    let config = ic_config(mna.node_map["c3"] - 1);
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();
    let seed = ir
        .v_prev_ic_seed
        .as_ref()
        .expect("IC= circuit must populate v_prev_ic_seed");

    let c3 = mna.node_map["c3"] - 1;
    let b4 = mna.node_map["b4"] - 1;
    assert!(
        (seed[c3] - seed[b4] - (-4.0)).abs() < 1e-9,
        "v_prev_ic_seed[c3] - v_prev_ic_seed[b4] = {}, expected -4.0",
        seed[c3] - seed[b4]
    );
    assert!(
        (seed[c3] - TWO_NODE_V_C3).abs() < 1e-6,
        "v_prev_ic_seed[c3] = {}, expected {}",
        seed[c3],
        TWO_NODE_V_C3
    );
    assert!(
        (seed[b4] - TWO_NODE_V_B4).abs() < 1e-6,
        "v_prev_ic_seed[b4] = {}, expected {}",
        seed[b4],
        TWO_NODE_V_B4
    );
}

// ---------------------------------------------------------------------------
// 2. Codegen string-level: DK path
// ---------------------------------------------------------------------------

#[test]
fn codegen_dk_emits_v_prev_ic_seed_const() {
    let (code, _n, _m) = support::generate_circuit_code(RC_DISCHARGE_IC, &ic_config(1));
    assert!(
        code.contains("pub const V_PREV_IC_SEED: [f64; N]"),
        "generated code must declare V_PREV_IC_SEED"
    );
    assert!(
        code.contains("v_prev: V_PREV_IC_SEED"),
        "Default impl must seed v_prev from V_PREV_IC_SEED"
    );
    assert!(
        code.contains("self.v_prev = V_PREV_IC_SEED;"),
        "reset() must restore v_prev from V_PREV_IC_SEED"
    );
    // has_dc_op is false for this circuit (no DC sources) — DC_OP itself
    // must NOT be emitted, but V_PREV_IC_SEED must be emitted regardless.
    assert!(
        !code.contains("pub const DC_OP: [f64"),
        "circuit has no DC sources — DC_OP must not be emitted"
    );
    // dc_operating_point stays tied to DC_OP semantics (zeros here), never
    // to the IC seed.
    assert!(
        code.contains("dc_operating_point: [0.0; N]"),
        "dc_operating_point must stay IC-free"
    );
    // The IC value itself must appear in the constant array, formatted the
    // same way as every other emitted f64 constant (`{:.17e}`).
    let formatted = format!("{:.17e}", 5.0_f64);
    assert!(
        code.contains(&formatted),
        "V_PREV_IC_SEED array must contain the baked 5.0 V initial condition \
         (formatted as {formatted})"
    );
}

#[test]
fn codegen_dk_no_ic_omits_v_prev_ic_seed_const() {
    let (code, _n, _m) = support::generate_circuit_code(RC_DISCHARGE_NO_IC, &ic_config(1));
    assert!(
        !code.contains("V_PREV_IC_SEED"),
        "circuit without IC= must not emit V_PREV_IC_SEED anywhere"
    );
    // Matches pre-existing (pre-IC-feature) codegen exactly for the no-DC-op
    // case: v_prev zeroed, reset() restores from dc_operating_point.
    assert!(code.contains("v_prev: [0.0; N]"));
    assert!(code.contains("self.v_prev = self.dc_operating_point;"));
}

// ---------------------------------------------------------------------------
// 2b. Codegen string-level: nodal path
// ---------------------------------------------------------------------------

#[test]
fn codegen_nodal_emits_v_prev_ic_seed_const() {
    let (code, _n, _m) = support::generate_circuit_code_nodal(RC_DISCHARGE_IC, &ic_config(1));
    assert!(
        code.contains("pub const V_PREV_IC_SEED: [f64; N]"),
        "nodal codegen must declare V_PREV_IC_SEED"
    );
    assert!(
        code.contains("v_prev: V_PREV_IC_SEED"),
        "nodal Default impl must seed v_prev from V_PREV_IC_SEED"
    );
    assert!(
        code.contains("self.v_prev = V_PREV_IC_SEED;"),
        "nodal reset() must restore v_prev from V_PREV_IC_SEED"
    );
}

#[test]
fn codegen_nodal_no_ic_omits_v_prev_ic_seed_const() {
    let (code, _n, _m) = support::generate_circuit_code_nodal(RC_DISCHARGE_NO_IC, &ic_config(1));
    assert!(
        !code.contains("V_PREV_IC_SEED"),
        "nodal circuit without IC= must not emit V_PREV_IC_SEED anywhere"
    );
}

// ---------------------------------------------------------------------------
// 3. Compiled-and-run numeric verification
// ---------------------------------------------------------------------------

#[test]
fn compiled_rc_discharge_matches_analytic_decay() {
    let circuit = support::build_circuit(RC_DISCHARGE_IC, &ic_config(1), "ic_rc_discharge");
    let sample_rate = 48000.0_f64;
    let num_samples = 2000; // ~42 ms, several time constants
    let input = vec![0.0; num_samples];
    let out = support::run_signal(&circuit, &input, sample_rate);
    assert_eq!(out.len(), num_samples);

    let tau = 1e-3 * 10.0; // R=1k, C=10u -> tau = 10ms
    for &idx in &[0usize, 100, 480, 960, 1440] {
        let t = (idx + 1) as f64 / sample_rate; // process_sample output is one step past v_prev
        let expected = 5.0 * (-t / tau).exp();
        let got = out[idx];
        let rel_err = (got - expected).abs() / expected.max(1e-6);
        assert!(
            rel_err < 0.01,
            "sample[{idx}]: got {got}, expected ~{expected} (analytic RC decay), rel_err={rel_err}"
        );
    }
}

#[test]
fn compiled_rc_no_ic_stays_at_zero() {
    // Control: without IC=, the same topology with V1=0 must stay at zero
    // (no DC source, no charge) — proves the IC feature is opt-in.
    let circuit = support::build_circuit(RC_DISCHARGE_NO_IC, &ic_config(1), "ic_rc_no_ic");
    let sample_rate = 48000.0_f64;
    let input = vec![0.0; 100];
    let out = support::run_signal(&circuit, &input, sample_rate);
    for (i, &v) in out.iter().enumerate() {
        assert!(
            v.abs() < 1e-9,
            "sample[{i}] = {v}, expected 0.0 (no IC, no DC source)"
        );
    }
}

#[test]
fn compiled_two_node_ic_constraint_holds_at_first_sample() {
    let (netlist, mna, _kernel) = build_pipeline(TWO_NODE_IC);
    let c3 = mna.node_map["c3"] - 1;
    let b4 = mna.node_map["b4"] - 1;
    let mut config = ic_config(c3);
    config.circuit_name = "ic_two_node".to_string();
    let _ = netlist; // node maps already resolved above

    let circuit = support::build_circuit_nodal(TWO_NODE_IC, &config, "ic_two_node_nodal");
    let sample_rate = 48000.0_f64;
    let input = vec![0.0; 5];
    let out_c3 = support::run_signal(&circuit, &input, sample_rate);

    // Also probe b4 via a second compile with output_nodes=[b4] (same
    // circuit/topology, cheap to compile again — cached by content hash).
    let mut config_b4 = ic_config(b4);
    config_b4.circuit_name = "ic_two_node".to_string();
    let circuit_b4 = support::build_circuit_nodal(TWO_NODE_IC, &config_b4, "ic_two_node_nodal_b4");
    let out_b4 = support::run_signal(&circuit_b4, &input, sample_rate);

    // With a 6nF cap and 5k series R, tau ~ 30us — comparable to the 20.8us
    // sample period, so even sample 0 (one full trapezoidal step past the
    // raw IC seed) has already decayed substantially. Assert the two probed
    // traces stay finite and bounded by the seed magnitude (loose physical
    // sanity check — the exact seed value is pinned by the IR-level test
    // above, which inspects v_prev_ic_seed directly before any stepping).
    for (i, (&c3v, &b4v)) in out_c3.iter().zip(out_b4.iter()).enumerate() {
        assert!(c3v.is_finite() && b4v.is_finite(), "sample {i} not finite");
        assert!(
            c3v.abs() <= TWO_NODE_V_C3.abs() + 1.0,
            "c3[{i}]={c3v} exceeds seed-consistent bound"
        );
        assert!(
            b4v.abs() <= TWO_NODE_V_B4.abs() + 1.0,
            "b4[{i}]={b4v} exceeds seed-consistent bound"
        );
    }
}

#[test]
fn compiled_ic_circuit_compiles_dk_and_nodal() {
    let (code_dk, _, _) = support::generate_circuit_code(RC_DISCHARGE_IC, &ic_config(1));
    let (code_nodal, _, _) = support::generate_circuit_code_nodal(RC_DISCHARGE_IC, &ic_config(1));
    // build_circuit / build_circuit_nodal already assert_compiles internally
    // via compile_circuit_code (panics on failure); calling them here is the
    // actual regression pin, but also sanity-check the raw strings compile
    // conceptually by construction, not just contain the right substrings.
    let _ = support::build_circuit(RC_DISCHARGE_IC, &ic_config(1), "ic_compile_dk");
    let _ = support::build_circuit_nodal(RC_DISCHARGE_IC, &ic_config(1), "ic_compile_nodal");
    assert!(code_dk.contains("V_PREV_IC_SEED"));
    assert!(code_nodal.contains("V_PREV_IC_SEED"));
}
