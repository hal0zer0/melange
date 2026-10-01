//! The DC operating point's op-amp rail is an active set inside Newton.
//!
//! Every Newton iteration of every ladder stage solves with the railed outputs
//! pinned the rail mode's way (at the terminal under `Hard`, on the load line
//! `limit − R_SAG·I_load` under the active-set modes), reads the pin set of the
//! next iteration from the raw solution, and treats a pinned row as its own
//! equation. A post-step clamp was a projection instead: the step test read
//! the pre-clamp step, so a high-gain output linearly beyond its rail beside a
//! junction never converged, under any rail mode.
//!
//! References are ngspice `.op` (reltol 1e-9) with the op-amp clamped the same
//! way: an 8 V source at the output (`Hard`), 8 V behind 200 Ω (load line), or
//! a VCCS `AOL/ROUT` with `ROUT` 75 Ω (`None`), and the build's 1 Ω input
//! source at `in`. Rails: VCC/VEE ±9 V, default 1.0 V drop (limit 8 V).

mod support;

use melange_solver::codegen::OpampRailMode;

/// `v(oa)` linearly 10.87 V, beyond the 8 V limit, with a diode at `b`.
const RAILED_BESIDE_A_DIODE: &str = "\
Railed op-amp beside a diode
Vref ref 0 DC 1
R1 in inv 10k
R2 inv oa 100k
U1 ref inv oa OA1
Rload oa 0 1k
Rb oa b 1meg
D1 b 0 DX
Co oa out 1u
Rl out 0 100k
.model DX D(IS=1e-14)
.model OA1 OA(AOL=200000 VCC=9 VEE=-9)
";

/// The same op-amp driving a BJT base through 1 MΩ.
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

fn dc_op(deck: &str, mode: OpampRailMode) -> melange_solver::build::Built {
    let mut config = support::config_for_spice(deck, 48000.0);
    config.opamp_rail_mode = mode;
    support::build_shipped(deck, &config, "auto")
}

fn node(built: &melange_solver::build::Built, name: &str) -> f64 {
    built.dc_op.v_node[built.mna.node_map[name] - 1]
}

/// (deck, mode, [(node, ngspice volts, tolerance)])
type RailCase<'a> = (&'a str, OpampRailMode, &'a [(&'a str, f64, f64)]);

#[test]
fn a_railed_output_beside_a_junction_settles_on_each_rail_law() {
    // (deck, mode, [(node, ngspice volts, tolerance)])
    let cases: [RailCase<'_>; 6] = [
        (
            RAILED_BESIDE_A_DIODE,
            OpampRailMode::Hard,
            &[("oa", 8.0, 1e-9), ("b", 0.5284663, 1e-6)],
        ),
        (
            RAILED_BESIDE_A_DIODE,
            OpampRailMode::ActiveSet,
            &[("oa", 6.655561, 2e-6), ("b", 0.5233568, 1e-6)],
        ),
        (
            RAILED_BESIDE_A_DIODE,
            OpampRailMode::None,
            &[("oa", 10.99835, 2e-5), ("b", 0.5371717, 1e-6)],
        ),
        (
            RAILED_INTO_A_BJT,
            OpampRailMode::Hard,
            &[
                ("oa", 8.0, 1e-9),
                ("b", 0.6471645, 1e-6),
                ("c", 8.544164, 1e-5),
            ],
        ),
        (
            RAILED_INTO_A_BJT,
            OpampRailMode::ActiveSet,
            &[
                ("oa", 6.655580, 2e-6),
                ("b", 0.6419641, 1e-6),
                ("c", 9.173597, 1e-5),
            ],
        ),
        (
            RAILED_INTO_A_BJT,
            OpampRailMode::None,
            &[
                ("oa", 10.99835, 2e-5),
                ("b", 0.6559886, 1e-6),
                ("c", 7.139088, 1e-5),
            ],
        ),
    ];
    for (deck, mode, expect) in cases {
        let title = deck.lines().next().unwrap();
        let built = dc_op(deck, mode);
        assert!(
            built.dc_op.converged,
            "{title}, {mode:?}: {:?} after {} iterations",
            built.dc_op.method, built.dc_op.iterations
        );
        for &(name, want, tol) in expect {
            let got = node(&built, name);
            assert!(
                (got - want).abs() <= tol,
                "{title}, {mode:?}: v({name}) = {got:.7} V, ngspice {want:.7} V"
            );
        }
    }
}

/// A unity follower of a diode-clamped node fed from 15 V. The linear start
/// puts `v+` at the supply, past the 13.5 V limit, so the follower starts
/// pinned high; the first Newton step brings the clamp on (`v+` 0.66 V). With
/// the output pinned the loop is open, so `v+ − v−` points at the opposite
/// rail: a test that moved the pin there alternated between the rails for the
/// whole budget. A held pin only stays or releases, and released, the next
/// solve decides the side with the loop closed. ngspice `v(out)` 0.6644267 V.
const FOLLOWER_OF_A_CLAMPED_NODE: &str = "\
Follower of a clamped node
Rin in 0 10k
Vs vs 0 DC 15
Rs vs x 10k
D1 x 0 DX
U1 x fb out OA
Rf out fb 10k
.model OA OA(AOL=100000 VSAT=13.5)
.model DX D(IS=1e-14)
";

/// A unity follower after a clamped threshold stage and an attack diode, cut
/// down from a compressor sidechain: nothing is railed at the operating point
/// (`v(out)` −0.83 mV; ngspice −0.8306 mV, the 1 µV gap is its reverse-bias
/// diode polynomial). The first Newton step is damped, and the damped iterate
/// puts `AOL·(v+ − v−)` past the rail. Reading the pin set from the raw
/// solution never pins it; reading it from the damped iterate pins it once,
/// and then only the release rule brings it back. Without both the follower
/// flipped between the rails until the budget ran out.
const FOLLOWER_AFTER_A_THRESHOLD_CLAMP: &str = "\
Follower after a clamped threshold stage
Vpos vcc15 0 DC 15
Rthra vcc15 th_ref 10K
Rthrb th_ref 0 1.5K
Rin in thr_vg 10K
Rt2 th_ref thr_vg 40K
U_thr 0 thr_vg over OA
Rtfb over thr_vg 400K
Dthrc thr_vg over DX
Ratt over att_a 10K
Datt att_a cv DX
Ctime cv 0 2.2U
Rrel cv 0 330K
U_cvb cv cvb_i out OA
Rcvbf out cvb_i 10K
.model OA OA(AOL=100000 ROUT=100 VSAT=13.5)
.model DX D(IS=2.52N N=1.752)
";

/// `deck` converges under `Hard` and `ActiveSet` to its unrailed (`None`)
/// operating point, with no output pinned; the `None` point's `v(out)` is
/// `ngspice` within 5 µV.
fn settles_unrailed(deck: &str, nodes: &[&str], ngspice: f64) {
    let free = dc_op(deck, OpampRailMode::None);
    assert!(free.dc_op.converged);
    let out = node(&free, "out");
    assert!((out - ngspice).abs() < 5e-6, "free v(out) = {out} V");
    for mode in [OpampRailMode::Hard, OpampRailMode::ActiveSet] {
        let built = dc_op(deck, mode);
        assert!(
            built.dc_op.converged,
            "{mode:?}: {:?} after {} iterations",
            built.dc_op.method, built.dc_op.iterations
        );
        assert_eq!(built.dc_op.rail_pin.label(), "none", "{mode:?}");
        for &name in nodes {
            let (got, want) = (node(&built, name), node(&free, name));
            assert!(
                (got - want).abs() < 1e-9,
                "{mode:?}: v({name}) = {got} V, unrailed {want} V"
            );
        }
    }
}

#[test]
fn a_pinned_follower_releases_instead_of_flipping_rails() {
    settles_unrailed(FOLLOWER_OF_A_CLAMPED_NODE, &["out", "x"], 0.6644267);
}

#[test]
fn a_damped_first_step_does_not_strand_a_follower_on_a_rail() {
    settles_unrailed(
        FOLLOWER_AFTER_A_THRESHOLD_CLAMP,
        &["out", "cv", "over", "att_a"],
        -0.000830550,
    );
}
