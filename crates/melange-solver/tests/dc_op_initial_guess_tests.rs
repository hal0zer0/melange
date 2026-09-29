//! The DC operating point's initial guess clamps each junction, not a node.
//!
//! The linear solve that seeds Newton has no device currents, so it can hold
//! a junction several volts into forward bias. The guess clamps the junction
//! by moving its dependent node (cathode, emitter), which is also right when a
//! voltage source fixes that node (the source's own row restores it on the
//! first Newton step), and by moving the other node (anode, base) when the
//! dependent node is ground. A grounded junction the guess cannot move
//! otherwise starts where the linear solve put it, and an exponential
//! junction descends one thermal voltage per Newton iteration from there:
//! from a 5.4 V start that is ~190 iterations.
//!
//! References are ngspice `.op` (reltol 1e-9) of the same decks, with the
//! build's 1 Ω input source at `in`.

mod support;

use melange_solver::codegen::OpampRailMode;
use melange_solver::dc_op::DcOpMethod;

fn dc_op(deck: &str, mode: OpampRailMode) -> melange_solver::build::Built {
    let mut config = support::config_for_spice(deck, 48000.0);
    config.opamp_rail_mode = mode;
    support::build_shipped(deck, &config, "auto")
}

fn node(built: &melange_solver::build::Built, name: &str) -> f64 {
    built.dc_op.v_node[built.mna.node_map[name] - 1]
}

/// An op-amp (linearly 5.5 V, inside its rail) driving a grounded-emitter BJT
/// base through 1 MΩ: the linear guess holds the base at the op-amp output.
const GROUNDED_EMITTER: &str = "\
Op-amp into a grounded-emitter BJT
Vref ref 0 DC 0.5
R1 in inv 10k
R2 inv oa 100k
U1 ref inv oa OA1
Rload oa 0 1k
Rb oa b 1meg
Q1 c b 0 QN
Rc vcc c 4.7k
Vcc vcc 0 DC 12
Co c out 1u
Rl out 0 100k
.model OA1 OA(AOL=200000 VCC=9 VEE=-9)
.model QN NPN(IS=1e-14 BF=100)
";

#[test]
fn a_grounded_emitter_junction_starts_clamped() {
    for mode in [
        OpampRailMode::Hard,
        OpampRailMode::ActiveSet,
        OpampRailMode::None,
    ] {
        let built = dc_op(GROUNDED_EMITTER, mode);
        let r = &built.dc_op;
        assert_eq!(r.method, DcOpMethod::DirectNr, "{mode:?}");
        assert!(r.iterations <= 20, "{mode:?}: {} iterations", r.iterations);
        let (b, c) = (node(&built, "b"), node(&built, "c"));
        assert!((b - 0.6364696).abs() < 1e-6, "{mode:?}: v(b) = {b}");
        assert!((c - 9.714525).abs() < 1e-5, "{mode:?}: v(c) = {c}");
    }
}

/// The same op-amp railed at rest (linearly 10.87 V, 8 V limit) into the
/// grounded-emitter BJT: Newton converges directly under every rail law
/// instead of falling to source stepping.
const RAILED_INTO_A_BJT: &str = "\
Railed op-amp into a BJT base
Vref ref 0 DC 1
R1 in inv 10k
R2 inv oa 100k
U1 ref inv oa OA1
Rload oa 0 1k
Rb oa b 1meg
Q1 c b 0 QN
Rc vcc c 4.7k
Vcc vcc 0 DC 12
Co c out 1u
Rl out 0 100k
.model OA1 OA(AOL=200000 VCC=9 VEE=-9)
.model QN NPN(IS=1e-14 BF=100)
";

#[test]
fn a_railed_op_amp_into_a_grounded_emitter_converges_directly() {
    for mode in [
        OpampRailMode::Hard,
        OpampRailMode::ActiveSet,
        OpampRailMode::None,
    ] {
        let r = dc_op(RAILED_INTO_A_BJT, mode).dc_op;
        assert_eq!(r.method, DcOpMethod::DirectNr, "{mode:?}");
        assert!(r.iterations <= 20, "{mode:?}: {} iterations", r.iterations);
    }
}

/// Under `BoyleDiodes` (the rail is catch diodes on an internal gain node)
/// both BJT decks have an operating point. Neither converged while the guess
/// left the grounded-emitter junction at the op-amp's output voltage.
#[test]
fn a_catch_diode_rail_into_a_grounded_emitter_has_an_operating_point() {
    for (deck, b_want) in [
        (GROUNDED_EMITTER, 0.6364698),
        (RAILED_INTO_A_BJT, 0.6457199),
    ] {
        let title = deck.lines().next().unwrap();
        let built = dc_op(deck, OpampRailMode::BoyleDiodes);
        assert!(built.dc_op.converged, "{title}: {:?}", built.dc_op.method);
        let b = node(&built, "b");
        assert!((b - b_want).abs() < 1e-6, "{title}: v(b) = {b}");
    }
}

/// Junctions on supply rails: an NPN whose emitter sits on −12 V with its base
/// held at 0 V by the guess (Vbe +12 V), and a diode from a 24 V divider into
/// a +12 V rail (forward 9.8 V in the guess). The guess moves the source-fixed
/// node and the rail's own row restores it. ngspice: v(b) −11.3416 V,
/// v(c) 6.669428 V; v(x) 12.65532 V.
const EMITTER_ON_A_NEGATIVE_RAIL: &str = "\
NPN with its emitter on a negative rail
Vee vee 0 DC -12
Vcc vcc 0 DC 12
Rb b 0 1meg
Q1 c b vee QN
Rc vcc c 4.7k
Cin in b 1u
Co c out 1u
Rl out 0 100k
.model QN NPN(IS=1e-14 BF=100)
";

const DIODE_INTO_A_POSITIVE_RAIL: &str = "\
Diode into a positive rail
V24 v24 0 DC 24
Vcc vcc 0 DC 12
R1 v24 x 10k
Rx x 0 100k
D1 x vcc DX
Cin in x 1u
Co x out 1u
Rl out 0 100k
.model DX D(IS=1e-14)
";

#[test]
fn junctions_on_supply_rails_converge_from_a_deep_forward_guess() {
    let built = dc_op(EMITTER_ON_A_NEGATIVE_RAIL, OpampRailMode::Auto);
    assert!(built.dc_op.converged);
    assert!(built.dc_op.iterations <= 10, "{}", built.dc_op.iterations);
    let (b, c) = (node(&built, "b"), node(&built, "c"));
    assert!((b - -11.3416).abs() < 1e-4, "v(b) = {b}");
    assert!((c - 6.669428).abs() < 1e-5, "v(c) = {c}");

    let built = dc_op(DIODE_INTO_A_POSITIVE_RAIL, OpampRailMode::Auto);
    assert!(built.dc_op.converged);
    assert!(built.dc_op.iterations <= 10, "{}", built.dc_op.iterations);
    let x = node(&built, "x");
    assert!((x - 12.65532).abs() < 1e-5, "v(x) = {x}");
}
