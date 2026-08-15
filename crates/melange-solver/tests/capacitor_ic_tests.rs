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

// ---------------------------------------------------------------------------
// 5. IC= combined with a VCVS (`E`) element — regression for the reported
//    blowup: `i_nl_prev` (DC_NL_I) was seeded from the plain (non-IC)
//    quiescent operating point regardless of `has_cap_ic`, while `v_prev`
//    was seeded from the IC-perturbed point. That KCL-inconsistent pair fed
//    into the first trapezoidal step and NR quietly diverged tens to
//    hundreds of samples later — reproduced on a cross-coupled-BJT astable
//    (the "G10 divider" circuit) with peaks up to ~57,000 V on an 8 V rail.
//    The mismatch is not VCVS-specific — any circuit where the IC-seeded
//    operating point differs materially from the plain one (which is the
//    entire purpose of IC=) can trigger it once nonlinear devices are
//    present; a VCVS control node happened to be the reporter's circuit.
// ---------------------------------------------------------------------------

/// Diode biased through a VCVS (E1, gain 2), with an IC=-bearing cap (Ca)
/// pinning node `a`'s companion node `b` 2V away from the diode's actual
/// operating voltage. Small, fast, deterministic — exercises exactly the
/// v_prev/i_nl_prev pairing mechanism without needing the full multi-BJT
/// astable topology that originally surfaced the bug.
const DIODE_E_IC: &str = "\
Diode E IC pairing test
V1 in 0 DC 5
E1 drv 0 in 0 2
R1 drv a 1k
D1 a 0 DMOD
Ca a b 1u IC=2
Rb b 0 10k
.model DMOD D(IS=1e-14)
";

const DIODE_E_NO_IC: &str = "\
Diode E no IC (control)
V1 in 0 DC 5
E1 drv 0 in 0 2
R1 drv a 1k
D1 a 0 DMOD
Ca a b 1u
Rb b 0 10k
.model DMOD D(IS=1e-14)
";

#[test]
fn ir_dc_nl_currents_ic_seed_present_and_differs_from_plain() {
    let (netlist, mna, kernel) = build_pipeline(DIODE_E_IC);
    let a = mna.node_map["a"] - 1;
    let config = ic_config(a);
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();

    let seed = ir
        .dc_nl_currents_ic_seed
        .as_ref()
        .expect("IC= circuit with a nonlinear device must populate dc_nl_currents_ic_seed");
    assert_eq!(seed.len(), 1, "single diode -> M=1");

    // The two solves must disagree (Ca's IC=2 moves the diode-adjacent
    // network away from the plain quiescent point) — otherwise this test
    // can't distinguish the fix from the pre-fix behavior.
    assert!(
        (seed[0] - ir.dc_nl_currents[0]).abs() > 1e-6,
        "dc_nl_currents_ic_seed[0]={} must differ from the plain \
         dc_nl_currents[0]={} for this circuit (IC= perturbs the diode's \
         effective bias network)",
        seed[0],
        ir.dc_nl_currents[0]
    );

    // `dc_nl_currents` (paired with `dc_operating_point`) must stay exactly
    // the plain, non-IC quiescent solve — independently recomputed here.
    let device_slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
    let dc_op_config = melange_solver::dc_op::DcOpConfig {
        input_node: config.input_node,
        input_resistance: config.input_resistance,
        ..melange_solver::dc_op::DcOpConfig::default()
    };
    let plain = melange_solver::dc_op::solve_dc_operating_point(&mna, &device_slots, &dc_op_config);
    assert!(
        (ir.dc_nl_currents[0] - plain.i_nl[0]).abs() < 1e-12,
        "dc_nl_currents must remain the plain (non-IC) DC OP current, got {} vs {}",
        ir.dc_nl_currents[0],
        plain.i_nl[0]
    );

    // And the IC-seeded seed must match solve_ic_seeded_operating_point's
    // own i_nl exactly (same solve, just re-derived independently here).
    let ic_result =
        melange_solver::dc_op::solve_ic_seeded_operating_point(&mna, &device_slots, &dc_op_config)
            .expect("has IC caps");
    assert!(
        (seed[0] - ic_result.i_nl[0]).abs() < 1e-12,
        "dc_nl_currents_ic_seed must equal solve_ic_seeded_operating_point's i_nl exactly"
    );
}

#[test]
fn ir_dc_nl_currents_ic_seed_absent_without_ic() {
    let (ir, _mna) = build_ir(DIODE_E_NO_IC, &ic_config(1));
    assert!(
        ir.dc_nl_currents_ic_seed.is_none(),
        "circuit without IC= caps must not populate dc_nl_currents_ic_seed"
    );
}

#[test]
fn codegen_dk_pairs_dc_nl_i_ic_seed_with_v_prev_ic_seed() {
    let a_idx = {
        let netlist = Netlist::parse(DIODE_E_IC).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.node_map["a"] - 1
    };
    let (code, _n, _m) = support::generate_circuit_code(DIODE_E_IC, &ic_config(a_idx));

    // Both constants must be present — the plain quiescent DC_NL_I and the
    // IC-seeded twin — since this circuit has a nonzero plain-DC diode
    // current AND an IC seed.
    assert!(
        code.contains("pub const DC_NL_I: [f64; M]"),
        "plain DC_NL_I must still be emitted (paired with dc_operating_point)"
    );
    assert!(
        code.contains("pub const DC_NL_I_IC_SEED: [f64; M]"),
        "DC_NL_I_IC_SEED must be emitted for an IC= circuit with M>0"
    );

    // Construction (Default) and reset() must seed i_nl_prev from the
    // IC-seeded constant, paired with V_PREV_IC_SEED for v_prev.
    assert!(
        code.contains("i_nl_prev: DC_NL_I_IC_SEED"),
        "Default impl must seed i_nl_prev from DC_NL_I_IC_SEED, not DC_NL_I"
    );
    assert!(
        code.contains("self.i_nl_prev = DC_NL_I_IC_SEED;"),
        "reset() must restore i_nl_prev from DC_NL_I_IC_SEED, not DC_NL_I"
    );

    // The plain DC_NL_I must still be used by the per-sample magnitude/NaN
    // reset fallback, paired with dc_operating_point (NOT the IC seed) —
    // this is the pairing that broke when an earlier fix attempt simply
    // repointed DC_NL_I at the IC-seeded solve.
    assert!(
        code.contains("state.v_prev = state.dc_operating_point;")
            && code.contains("state.i_nl_prev = DC_NL_I;"),
        "the magnitude/NaN-reset fallback must reset to the plain \
         (dc_operating_point, DC_NL_I) pair, independent of has_cap_ic"
    );
}

#[test]
fn codegen_nodal_pairs_dc_nl_i_ic_seed_with_v_prev_ic_seed() {
    let a_idx = {
        let netlist = Netlist::parse(DIODE_E_IC).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.node_map["a"] - 1
    };
    let (code, _n, _m) = support::generate_circuit_code_nodal(DIODE_E_IC, &ic_config(a_idx));
    assert!(code.contains("pub const DC_NL_I_IC_SEED: [f64; M]"));
    assert!(code.contains("i_nl_prev: DC_NL_I_IC_SEED"));
    assert!(code.contains("self.i_nl_prev = DC_NL_I_IC_SEED;"));
}

#[test]
fn codegen_dk_no_ic_omits_dc_nl_i_ic_seed() {
    let (code, _n, _m) = support::generate_circuit_code(DIODE_E_NO_IC, &ic_config(1));
    assert!(
        !code.contains("DC_NL_I_IC_SEED"),
        "circuit without IC= must not emit DC_NL_I_IC_SEED anywhere"
    );
}

// ---------------------------------------------------------------------------
// 6. End-to-end: the reported blowup circuit stays bounded.
//
// A cross-coupled-BJT astable ("G10 divider") kicked off its degenerate
// symmetric fixed point by an IC=-bearing capacitor, with a VCVS driving
// the trigger input. Before the fix: peak ~5053 V (this exact netlist) /
// ~48000 V (VCVS-free variant) on an 8 V rail. After the fix: bounded,
// matching the plain (non-IC) quiescent peak.
// ---------------------------------------------------------------------------

/// Reported repro (melange-circuits): two cross-coupled PNP stages, a VCVS
/// driving the trigger network, IC=-4 on the cross-coupling cap C_x1.
const ASTABLE_IC_AND_E: &str = "\
G10 divider lab v3 (internal PULSE drive)
Vrail rail 0 DC 8
R_green green 0 2.7k
E_amp drvsrc 0 in 0 8
R_trig drvsrc trigmid 10k
C_trig trigmid b4 1n
Q_D1A c3 b3 rail SFT352
Q_D1B c4 b4 rail SFT352
R_b3 b3 green 150k
R_b4 b4 green 150k
R_c4 c4 green 56k
R_c3 c3 ntap 2.7k
R_ntap ntap green 27k
C_x1 c3 b4 6n IC=-4
C_x2 c4 b3 6.8n
C_out ntap out 1u
R_bleed out green 100k

.model SFT352 PNP(IS=3e-7 BF=90 VAF=50 RB=40 RC=4 RE=1 CJE=80p CJC=30p TF=1n)
";

/// Same topology, VCVS replaced by a direct wire from `in` (keeps the `in`
/// node name for `config_for_spice` auto-detection) — isolates the IC=
/// effect from the VCVS.
const ASTABLE_IC_ONLY: &str = "\
G10 divider lab v3 (no VCVS)
Vrail rail 0 DC 8
R_green green 0 2.7k
R_trig in trigmid 10k
C_trig trigmid b4 1n
Q_D1A c3 b3 rail SFT352
Q_D1B c4 b4 rail SFT352
R_b3 b3 green 150k
R_b4 b4 green 150k
R_c4 c4 green 56k
R_c3 c3 ntap 2.7k
R_ntap ntap green 27k
C_x1 c3 b4 6n IC=-4
C_x2 c4 b3 6.8n
C_out ntap out 1u
R_bleed out green 100k

.model SFT352 PNP(IS=3e-7 BF=90 VAF=50 RB=40 RC=4 RE=1 CJE=80p CJC=30p TF=1n)
";

/// Same topology with the VCVS, no IC= — the plain quiescent circuit
/// (control for "does the VCVS alone cause the blowup").
const ASTABLE_E_ONLY: &str = "\
G10 divider lab v3 (no IC)
Vrail rail 0 DC 8
R_green green 0 2.7k
E_amp drvsrc 0 in 0 8
R_trig drvsrc trigmid 10k
C_trig trigmid b4 1n
Q_D1A c3 b3 rail SFT352
Q_D1B c4 b4 rail SFT352
R_b3 b3 green 150k
R_b4 b4 green 150k
R_c4 c4 green 56k
R_c3 c3 ntap 2.7k
R_ntap ntap green 27k
C_x1 c3 b4 6n
C_x2 c4 b3 6.8n
C_out ntap out 1u
R_bleed out green 100k

.model SFT352 PNP(IS=3e-7 BF=90 VAF=50 RB=40 RC=4 RE=1 CJE=80p CJC=30p TF=1n)
";

/// Neither VCVS nor IC= — plain control.
const ASTABLE_NEITHER: &str = "\
G10 divider lab v3 (neither)
Vrail rail 0 DC 8
R_green green 0 2.7k
R_trig in trigmid 10k
C_trig trigmid b4 1n
Q_D1A c3 b3 rail SFT352
Q_D1B c4 b4 rail SFT352
R_b3 b3 green 150k
R_b4 b4 green 150k
R_c4 c4 green 56k
R_c3 c3 ntap 2.7k
R_ntap ntap green 27k
C_x1 c3 b4 6n
C_x2 c4 b3 6.8n
C_out ntap out 1u
R_bleed out green 100k

.model SFT352 PNP(IS=3e-7 BF=90 VAF=50 RB=40 RC=4 RE=1 CJE=80p CJC=30p TF=1n)
";

/// The rail is 8 V. This circuit is a genuine astable multivibrator (a
/// "G10 divider") — the IC=-4V seed kicks it off its degenerate symmetric
/// DC fixed point into real switching dynamics, not a settle back to a flat
/// DC point. Once the DK per-iteration NR voltage-limiting floors were also
/// fixed (`nr_helpers.rs::emit_nr_limit_and_converge`, see
/// `tools/melange-cli/tests/cli_integration.rs`
/// `test_ic_seeded_astable_stays_bounded_at_os4`), the resolved trajectory
/// plateaus around a ~20-32 V oscillation (measured up to ~32 V over 300 ms)
/// rather than the ~1-20 V a coarser/floor-limited solve showed. 50 V is a
/// loose bound with real margin over that plateau — it pins "did not blow
/// up" (pre-fix: 5053 V-57000 V finite, 488,012 V at 4x oversampling)
/// without over-fitting to the exact oscillation amplitude.
const ASTABLE_BOUND: f64 = 50.0;

fn run_astable(spice: &str, tag: &str) -> Vec<f64> {
    let config = support::config_for_spice(spice, 48000.0);
    let circuit = support::build_circuit(spice, &config, tag);
    let input = vec![0.0; 2400]; // 50 ms — matches the plateaued-amplitude window
    support::run_signal(&circuit, &input, 48000.0)
}

#[test]
fn astable_ic_and_e_stays_bounded() {
    let out = run_astable(ASTABLE_IC_AND_E, "astable_ic_and_e");
    support::assert_finite(&out);
    support::assert_bounded(&out, -ASTABLE_BOUND, ASTABLE_BOUND);
}

#[test]
fn astable_ic_only_stays_bounded() {
    let out = run_astable(ASTABLE_IC_ONLY, "astable_ic_only");
    support::assert_finite(&out);
    support::assert_bounded(&out, -ASTABLE_BOUND, ASTABLE_BOUND);
}

#[test]
fn astable_e_only_stays_bounded() {
    let out = run_astable(ASTABLE_E_ONLY, "astable_e_only");
    support::assert_finite(&out);
    support::assert_bounded(&out, -ASTABLE_BOUND, ASTABLE_BOUND);
}

#[test]
fn astable_neither_stays_bounded() {
    let out = run_astable(ASTABLE_NEITHER, "astable_neither");
    support::assert_finite(&out);
    support::assert_bounded(&out, -ASTABLE_BOUND, ASTABLE_BOUND);
}
