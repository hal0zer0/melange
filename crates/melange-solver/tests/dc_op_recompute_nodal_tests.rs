//! The runtime DC operating-point solve on the nodal route
//! (`recompute_dc_op` behind `--emit-dc-op-recompute`), the full
//! `set_dc_operating_point` writeback with its KCL residual on both routes,
//! and the live-point / own-currents pairing `reset()` and the NaN recovery
//! restart from.
//!
//! Acceptance (ruled 2026-11-03):
//! - (i) on nodal fixtures (diode + BJT, a transformer with a DC-carrying
//!   winding, a JFET, a switch, a pot), `v_prev` perturbed by ×1.1 and by
//!   +1 V on the node rows recomputes onto the baked `DC_OP` within 1e-9 V
//!   and 1e-9 A of KCL, in a bounded number of iterations;
//! - (ii) after a `set_switch` / `set_pot` change the recompute equals the
//!   `DC_OP` of the same deck built at that value;
//! - (iii) the first `process_sample` after a recompute converges in at most
//!   two iterations with no hold and no sub-step;
//! - (v) a behavioral deck, a clampable-op-amp deck and a glow deck refuse:
//!   the counter advances and the state is untouched;
//! - (vi) `set_dc_operating_point` on a node-voltages-only vector of a
//!   transformer deck returns a residual that names the node the winding
//!   feeds;
//! - (viii) after a recompute at moved pots, a NaN sample restarts from the
//!   live point and its own currents, not the baked `DC_NL_I`.

mod support;

use melange_solver::codegen::{CodegenConfig, OpampRailMode};

fn config(spice: &str) -> CodegenConfig {
    let mut cfg = support::config_in_out_or_node1(spice, 48000.0);
    cfg.circuit_name = "dc_op_recompute_nodal".to_string();
    cfg.emit_dc_op_recompute = true;
    cfg
}

fn nodal(spice: &str) -> String {
    support::build_as_shipped(spice, &config(spice), "nodal").0
}

fn dk(spice: &str) -> String {
    support::build_as_shipped(spice, &config(spice), "dk").0
}

/// Compile `code` with `main` (which prints `ok` on success) and run it.
fn run(code: &str, main: &str, tag: &str) -> support::RunOutput {
    let out = support::compile_and_run(code, main, tag);
    assert!(out.stdout.contains("ok"), "{tag}: {}", out.stdout);
    out
}

// --- fixtures -----------------------------------------------------------------

/// A BJT common-emitter stage with a diode clamp on its output.
const DIODE_BJT: &str = "\
bjt stage with diode clamp
VCC vcc 0 DC 12
R1 vcc b 100k
R2 b 0 22k
Rc vcc c 6.8k
Re e 0 1k
Q1 c b e QX
Cin in b 1u
Cout c out 1u
Rl out 0 100k
D1 out 0 DX
.model QX NPN(IS=1e-14 BF=200)
.model DX D(IS=1e-14)
";

/// A transformer whose primary carries 10 mA of DC: the winding current is a
/// row of the solution.
const TRANSFORMER: &str = "\
transformer with a DC-carrying primary
VCC vcc 0 DC 10
L1 vcc mid 100m
L2 sec 0 10m
K1 L1 L2 0.9
Rload mid 0 1k
Rsec sec out 1k
C1 mid 0 100n
C2 out 0 100n
Rin in mid 1Meg
";

/// A JFET common-source stage.
const JFET: &str = "\
jfet stage
VCC vcc 0 DC 12
Rd vcc d 2.2k
Rg g 0 1Meg
Rs s 0 470
J1 d g s JX
Cin in g 100n
Cout d out 1u
Rl out 0 100k
.model JX NJF(VTO=-2 BETA=1e-3)
";

/// A diode bias network whose feed resistor is a 3-position switch.
const SWITCH: &str = "\
switched diode bias
VCC vcc 0 DC 5
R1 vcc a 1k
.switch R1 1k 2.2k 4.7k
D1 a 0 DX
C1 a out 1u
Rl out 0 100k
Rin in a 1Meg
.model DX D(IS=1e-14)
";

/// The same bias network with the feed resistor as a pot.
const POT: &str = "\
pot diode bias
VCC vcc 0 DC 5
R1 vcc a 1k
.pot R1 500 10k
D1 a 0 DX
C1 a out 1u
Rl out 0 100k
Rin in a 1Meg
.model DX D(IS=1e-14)
";

/// Recompute from a perturbed start and compare with the baked point: every
/// row within 1e-9 V (the branch-current rows within 1e-9 A), the KCL residual
/// of the installed point within 1e-9 A, the iteration count bounded, and
/// the first sample afterwards converged at once with no hold or sub-step.
const PERTURB_MAIN: &str = "
fn main() {
    for (label, scale, shift) in [(\"x1.1\", 1.1f64, 0.0f64), (\"+1V\", 1.0, 1.0)] {
        let mut s = CircuitState::default();
        s.set_sample_rate(48000.0);
        let before = s.diag_nr_max_iter_count;
        for i in 0..N_NODES { s.v_prev[i] = s.v_prev[i] * scale + shift; }
        s.recompute_dc_op();
        assert_eq!(s.diag_nr_max_iter_count, before, \"{label}: the recompute refused\");
        let live: [f64; N] = *s.dc_op();
        for i in 0..N {
            assert!((live[i] - DC_OP[i]).abs() < 1e-9,
                \"{label}: row {i}: recomputed {} vs baked {}\", live[i], DC_OP[i]);
            assert_eq!(s.v_prev[i], live[i], \"{label}: v_prev seeded at row {i}\");
        }
        println!(\"{label}: iters={}\", s.last_nr_iterations);
        assert!(s.last_nr_iterations < 60, \"{label}: {} iterations\", s.last_nr_iterations);
        // The installed point is a KCL root (set_dc_operating_point re-installs
        // it and reports its residual).
        let (residual, row) = s.set_dc_operating_point(live);
        assert!(residual <= 1e-9, \"{label}: KCL residual {residual:e} A at row {row}\");
        let _ = process_sample(0.0, &mut s);
        assert!(s.last_nr_iterations <= 1, \"{label}: first sample took {} iterations\", s.last_nr_iterations + 1);
        assert_eq!(s.diag_substep_count, 0, \"{label}: sub-steps on the first sample\");
        assert_eq!(HOLDS, 0, \"{label}: holds on the first sample\");
    }
    println!(\"ok\");
}
";

fn perturb_main(code: &str) -> String {
    let holds = if code.contains("pub diag_nr_hold_count: ") {
        "s.diag_nr_hold_count"
    } else {
        "0u64"
    };
    PERTURB_MAIN.replace("HOLDS", holds)
}

#[test]
fn nodal_fixtures_recompute_onto_the_baked_point_from_a_perturbed_start() {
    for (deck, tag) in [
        (DIODE_BJT, "diode_bjt"),
        (TRANSFORMER, "transformer"),
        (JFET, "jfet"),
        (SWITCH, "switch"),
        (POT, "pot"),
    ] {
        let code = nodal(deck);
        assert!(
            code.contains("/// This build: solved."),
            "{tag}: the build must not refuse"
        );
        let out = run(&code, &perturb_main(&code), &format!("recompute_{tag}"));
        eprintln!("{tag}: {}", out.stdout.trim().replace('\n', " | "));
    }
}

/// After `set_switch_0(1)` / `set_pot_0(2200)`, the recompute lands on the
/// `DC_OP` of the same deck built at that position / value.
#[test]
fn recompute_after_a_control_change_equals_the_deck_built_at_that_value() {
    for (deck, tag, setter, spec) in [
        (SWITCH, "switch", "s.set_switch_0(1);", "R1=1"),
        (POT, "pot", "s.set_pot_0(2200.0);", "R1=2200"),
    ] {
        let overridden = support::try_build_shipped_with(deck, &config(deck), "nodal", |o| {
            if tag == "switch" {
                o.switch_overrides = Some(vec![spec.to_string()]);
            } else {
                o.pot_overrides = Some(vec![spec.to_string()]);
            }
        })
        .unwrap_or_else(|e| panic!("{tag}: override build failed: {e}"));
        let target: Vec<String> = overridden
            .dc_op
            .v_node
            .iter()
            .map(|v| format!("{v:.17e}"))
            .collect();
        let code = nodal(deck);
        let main = format!(
            "fn main() {{
    let target: [f64; N] = [{}];
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let before = s.diag_nr_max_iter_count;
    {setter}
    s.recompute_dc_op();
    assert_eq!(s.diag_nr_max_iter_count, before, \"the recompute refused\");
    for i in 0..N {{
        assert!((s.dc_op()[i] - target[i]).abs() < 1e-9,
            \"row {{i}}: recomputed {{}} vs the deck built at the value {{}}\", s.dc_op()[i], target[i]);
    }}
    let _ = process_sample(0.0, &mut s);
    assert!(s.last_nr_iterations <= 1, \"first sample took {{}} iterations\", s.last_nr_iterations + 1);
    println!(\"ok\");
}}",
            target.join(", ")
        );
        run(&code, &main, &format!("recompute_control_{tag}"));
    }
}

/// Builds that refuse: the counter advances, nothing else moves.
#[test]
fn refused_builds_bump_the_counter_and_leave_state_untouched() {
    let behavioral =
        "behavioral\nR1 in a 10k\nC1 a 0 100n\nB1 out 0 I={ tanh(V(a)) }\nR2 out 0 1k\n";
    let opamp = "comparator railed at rest\nVref ref 0 DC 0.1\nRin in 0 10k\nU1 ref in out OX\n\
                 Rl out 0 1k\n.model OX OA(AOL=100000 VCC=15 VEE=-15)\n";
    let glow =
        "neon relaxation\n.model NE1 NEON(VO=135 VM=93 IK=1.5e-3 RS=3000 IHOLD=2e-4 ROFF=300e6)\n\
                Vb rail 0 DC 170\nRc rail osc 1MEG\nCosc osc 0 10N\nN1 osc 0 NE1\nRin in 0 1G\n";
    let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let before = s.clone();
    s.recompute_dc_op();
    assert_eq!(s.diag_nr_max_iter_count, before.diag_nr_max_iter_count + 1);
    assert_eq!(s.v_prev, before.v_prev);
    assert_eq!(s.dc_operating_point, before.dc_operating_point);
    assert_eq!(s.i_nl_prev, before.i_nl_prev);
    assert_eq!(s.dc_op_i_nl, before.dc_op_i_nl);
    println!(\"ok\");
}";
    for (deck, tag, why) in [
        (behavioral, "behavioral", "behavioral B-source"),
        (opamp, "opamp", "clampable op-amp"),
        (glow, "glow", "latched (glow) device"),
    ] {
        let mut cfg = config(deck);
        if tag == "glow" {
            cfg.output_nodes = vec![support::node_index(deck, "osc")];
        }
        if tag == "opamp" {
            cfg.opamp_rail_mode = OpampRailMode::ActiveSet;
        }
        let code = support::build_as_shipped(deck, &cfg, "nodal").0;
        assert!(
            code.contains(&format!("/// This build: REFUSED ({why}).")),
            "{tag}: the method doc must name the refusal"
        );
        run(&code, main, &format!("refused_{tag}"));
    }
}

/// `set_dc_operating_point` reports a non-equilibrium vector: the baked point
/// with its winding and source currents zeroed leaves the primary's 10 mA
/// unaccounted at the node it feeds.
#[test]
fn set_dc_operating_point_names_the_node_a_zeroed_winding_feeds() {
    let code = nodal(TRANSFORMER);
    let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let (res_eq, row_eq) = s.set_dc_operating_point(DC_OP);
    assert!(res_eq <= 1e-9, \"the baked point is an equilibrium: {res_eq:e} A at row {row_eq}\");
    let mut nodes_only = DC_OP;
    for i in N_NODES..N { nodes_only[i] = 0.0; }
    let (res, row) = s.set_dc_operating_point(nodes_only);
    println!(\"residual={res:e} row={row} mid={}\", NODE_MID);
    assert_eq!(row, NODE_MID, \"the worst row is the node the primary feeds\");
    assert!((res - 10.0 / 1000.0).abs() < 1e-4, \"the residual is the primary's DC current, got {res:e} A\");
    assert_eq!(s.v_prev, nodes_only, \"the vector is installed regardless\");
    println!(\"ok\");
}";
    run(&code, main, "set_dc_op_winding");
}

/// `set_dc_operating_point` on both routes: an equilibrium reads within the
/// gate, a non-equilibrium reads its KCL error, and the writeback pairs the
/// installed point with the currents evaluated at it.
#[test]
fn set_dc_operating_point_writes_the_pair_on_both_routes() {
    let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let (res, row) = s.set_dc_operating_point(DC_OP);
    assert!(res <= 1e-9, \"baked point: {res:e} A at row {row}\");
    assert_eq!(s.i_nl_prev, s.dc_op_i_nl);
    assert_eq!(s.i_nl_prev_prev, s.dc_op_i_nl);
    assert_eq!(s.v_prev, DC_OP);
    let mut off = DC_OP;
    off[NODE_A] += 0.2;
    let (res_off, row_off) = s.set_dc_operating_point(off);
    assert!(res_off > 1e-4, \"0.2 V on the diode node must read as a large residual, got {res_off:e}\");
    assert_eq!(row_off, NODE_A);
    assert_eq!(s.dc_operating_point, off);
    assert_eq!(s.i_nl_prev, s.dc_op_i_nl);
    assert!(s.dc_op_i_nl[0] > DC_NL_I[0] * 100.0, \"the currents are the diode's at the installed point\");
    println!(\"ok\");
}";
    run(&nodal(POT), main, "set_dc_op_pair_nodal");
    run(&dk(POT), main, "set_dc_op_pair_dk");
}

/// After a recompute at moved pots, a sample that diverges restarts from the
/// live point WITH its own currents (`dc_op_i_nl`), and `reset()` returns to
/// the factory pair. Both routes.
#[test]
fn recovery_restarts_from_the_live_pair_and_reset_from_the_factory_pair() {
    let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    s.set_pot_0(5000.0);
    let before = s.diag_nr_max_iter_count;
    s.recompute_dc_op();
    assert_eq!(s.diag_nr_max_iter_count, before, \"the recompute refused\");
    let live = s.dc_operating_point;
    let live_i = s.dc_op_i_nl;
    assert!((live_i[0] - DC_NL_I[0]).abs() > 1e-6, \"the pot move must change the diode current\");
    // Every row: a NaN on a row no capacitor reads would never enter a solve.
    for i in 0..N { s.v_prev[i] = f64::NAN; }
    let _ = process_sample(0.0, &mut s);
    assert_eq!(s.diag_nan_reset_count, 1, \"the NaN recovery must have fired\");
    assert_eq!(s.v_prev, live, \"recovery restarts from the live point\");
    assert_eq!(s.i_nl_prev, live_i, \"...with its own currents\");
    assert_eq!(s.i_nl_prev_prev, live_i);
    s.reset();
    assert_eq!(s.dc_operating_point, DC_OP, \"reset returns to the factory point\");
    assert_eq!(s.dc_op_i_nl, DC_NL_I, \"...with the factory currents\");
    // reset() runs the warmup on the nodal route, so v_prev is the factory
    // point to within the warmup's own roundoff.
    for i in 0..N { assert!((s.v_prev[i] - DC_OP[i]).abs() < 1e-9, \"row {i} after reset\"); }
    for i in 0..M { assert!((s.i_nl_prev[i] - DC_NL_I[i]).abs() < 1e-12, \"current {i} after reset\"); }
    println!(\"ok\");
}";
    run(&nodal(POT), main, "recovery_pair_nodal");
    run(&dk(POT), main, "recovery_pair_dk");
}
