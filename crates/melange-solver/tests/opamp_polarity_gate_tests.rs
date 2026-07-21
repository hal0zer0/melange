//! Op-amp differential-polarity gates (review-round-2 H2).
//!
//! Every closed-loop op-amp configuration solves to the same gain under
//! EITHER sign of the VCCS transconductance (for an inverting amp with a
//! flipped Gm, `Vout(1 − AOL·R1/(R1+R2)) = AOL·R2/(R1+R2)·Vin` still gives
//! `−R2/R1·Vin` for large AOL; a follower gives +1 both ways). That is
//! exactly how the July 2026 inverted-VCCS bug (fixed in `3e246cb`)
//! survived every closed-loop gate, the SPICE suite (whose deck
//! hand-encoded the same flipped G element), and the goldens.
//!
//! These gates are OPEN-LOOP, where polarity is directly observable:
//! a +1 mV differential (V+ > V−) must drive the output POSITIVE with
//! magnitude ≈ Gm·v_diff·(Rout ∥ Rload), on
//!   1. the raw MNA stamps,
//!   2. the runtime DC operating-point path (`solve_dc_operating_point`),
//!   3. generated code (compiled and executed),
//!   4. generated code with asymmetric rails (VCC=9/VEE=0): a positive
//!      differential overdrive must saturate at the TOP rail — the
//!      observable that actually shipped wrong downstream in July 2026.
//!
//! Failure-mode verification (2026-07-21): with the Gm stamp signs flipped
//! back (`g[o][np-1] += gm; g[o][nm-1] -= gm` in a scratch copy of
//! `mna.rs`, never in-tree), all four tests fail: the stamp signs invert,
//! the DC OP and codegen outputs come out at −0.999 V instead of +0.999 V,
//! and the rail test saturates at 0 V (VEE) instead of 9 V (VCC).

mod support;

use melange_solver::codegen::CodegenConfig;
use melange_solver::dc_op::{self, DcOpConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const SR: f64 = 48_000.0;

/// 1. Raw MNA stamp signs, with BOTH inputs off ground.
///
/// Convention (mna.rs, "Convention: G[k][j]·Vj = current LEAVING node k"):
/// the op-amp injects +Gm·(V+ − V−) INTO the output node, so
///   G[out][V+] = −Gm,  G[out][V−] = +Gm,  G[out][out] += Go = 1/ROUT.
/// The existing unit test in mna.rs only pins the V− entry (V+ is
/// grounded there); this pins both, so flipping the pair back fails here
/// even if a compensating deck/golden change slips through elsewhere.
#[test]
fn opamp_vccs_stamp_signs_both_inputs() {
    let spice = "\
Opamp Stamp Sign Gate
R1 in vp 1k
R2 in vm 1k
R3 vp 0 1k
R4 vm 0 1k
U1 vp vm out OPA
Rload out 0 1k
.model OPA OA(AOL=1000 ROUT=1)
";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let o = mna.node_map["out"] - 1;
    let p = mna.node_map["vp"] - 1;
    let m = mna.node_map["vm"] - 1;

    let gm = 1000.0; // AOL/ROUT
                     // No other element connects out↔vp or out↔vm, so the entries are pure.
    assert!(
        (mna.g[o][p] - (-gm)).abs() < 1e-9,
        "G[out][V+] must be -Gm = {}, got {} (op-amp differential polarity!)",
        -gm,
        mna.g[o][p]
    );
    assert!(
        (mna.g[o][m] - gm).abs() < 1e-9,
        "G[out][V-] must be +Gm = {}, got {} (op-amp differential polarity!)",
        gm,
        mna.g[o][m]
    );
    // Go = 1/ROUT = 1.0 plus Rload's 1e-3.
    assert!(
        (mna.g[o][o] - 1.001).abs() < 1e-9,
        "G[out][out] must be Go + 1/Rload = 1.001, got {}",
        mna.g[o][o]
    );
}

/// 2. Runtime MNA path: DC operating point of an open-loop stage.
///
/// V1 pins V+ at +1 mV (augmented-MNA constraint row), V− is grounded,
/// AOL=1000/ROUT=1 keeps the open-loop output inside any rail:
///   V(out) = Gm·v_diff / (Go + 1/Rload) = 1000·0.001/1.001 = +0.999 V.
/// Sign AND magnitude asserted — a flipped Gm gives −0.999 V, a wrong
/// magnitude means AOL/ROUT→Gm mapping drifted.
#[test]
fn opamp_open_loop_positive_differential_runtime_dc_op() {
    let spice = "\
Opamp Open Loop DC
V1 vp 0 DC 0.001
U1 vp 0 out OPA
Rload out 0 1k
.model OPA OA(AOL=1000 ROUT=1)
";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let result = dc_op::solve_dc_operating_point(&mna, &[], &DcOpConfig::default());
    assert!(result.converged, "linear DC OP must converge");

    let v_out = result.v_node[mna.node_map["out"] - 1];
    assert!(
        v_out > 0.0,
        "open-loop op-amp with V+ = +1 mV, V- = 0 must drive output POSITIVE, \
         got {v_out:.6} V — differential polarity is inverted"
    );
    assert!(
        (v_out - 0.999).abs() < 0.01,
        "open-loop DC output magnitude: expected Gm·v_diff/(Go+1/Rload) = +0.999 V, \
         got {v_out:.6} V"
    );
}

/// 3. Generated code: same open-loop stage driven from the audio input.
///
/// The input Thevenin (1 Ω vs the 1 MΩ leak) puts ≈ +1 mV on V+; expected
/// settled output +0.999 V as above. `dc_block: false` so the DC observable
/// survives to the output. A flipped Gm gives −0.999 V.
#[test]
fn opamp_open_loop_positive_differential_codegen() {
    let spice = "\
Opamp Open Loop Codegen
Rleak in 0 1Meg
U1 in 0 out OPA
Rload out 0 1k
.model OPA OA(AOL=1000 ROUT=1)
";
    let config = CodegenConfig {
        circuit_name: "opamp_ol_gate".to_string(),
        sample_rate: SR,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        dc_block: false,
        ..CodegenConfig::default()
    };
    // Resolve node indices from the netlist instead of guessing.
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let config = CodegenConfig {
        input_node: mna.node_map["in"] - 1,
        output_nodes: vec![mna.node_map["out"] - 1],
        ..config
    };

    let circuit = support::build_circuit(spice, &config, "opamp_ol_gate");
    let out = support::run_step(&circuit, 0.001, 64, SR);
    let settled = *out.last().unwrap();
    assert!(
        settled > 0.0,
        "codegen open-loop op-amp with +1 mV on V+ must output POSITIVE, \
         got {settled:.6} V — differential polarity is inverted in generated code"
    );
    assert!(
        (settled - 0.999).abs() < 0.01,
        "codegen open-loop output magnitude: expected +0.999 V, got {settled:.6} V"
    );
}

/// 4. Generated code, asymmetric rails: overdrive lands on the CORRECT rail.
///
/// VCC=9/VEE=0 with a +0.1 V differential overdrive (raw VCCS output would
/// be ~+20 kV) must saturate at the TOP rail (9 V). This is the observable
/// that shipped wrong downstream in July 2026 — asymmetric clipping landed
/// on the wrong rail — and the only pre-existing test for it merely greps
/// the generated text for `.clamp(0.0, 9.0)`, which is polarity-blind.
/// A flipped Gm saturates at 0 V (VEE) and fails loudly here.
#[test]
fn opamp_open_loop_overdrive_saturates_at_top_rail_codegen() {
    let spice = "\
Opamp Rail Polarity
Rleak in 0 1Meg
U1 in 0 out OPA
Rload out 0 1k
.model OPA OA(AOL=200000 ROUT=1 VCC=9 VEE=0)
";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let config = CodegenConfig {
        circuit_name: "opamp_rail_gate".to_string(),
        sample_rate: SR,
        input_node: mna.node_map["in"] - 1,
        output_nodes: vec![mna.node_map["out"] - 1],
        input_resistance: 1.0,
        dc_block: false,
        ..CodegenConfig::default()
    };

    let circuit = support::build_circuit(spice, &config, "opamp_rail_gate");
    let out = support::run_step(&circuit, 0.1, 64, SR);
    let settled = *out.last().unwrap();
    assert!(
        settled > 8.5 && settled < 9.1,
        "positive differential overdrive must saturate at VCC = 9 V \
         (top rail), got {settled:.4} V — {}",
        if settled < 0.5 {
            "output sits at VEE: differential polarity is inverted"
        } else {
            "rail clamp magnitude drifted"
        }
    );
}
