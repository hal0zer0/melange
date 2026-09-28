//! Saturating coupled groups the shared-core model does not cover are refused.
//!
//! Each case below compiled silently before, and each was wrong:
//! - k <= 0.8: the old per-winding path was a no-op without a `.pot`/`.switch`
//!   (a K=0.5 pair at 2.5x Isat rendered bit-identical to the linear deck),
//!   per-winding saturation with one (physically wrong: under load the winding
//!   MMFs cancel), and absent altogether on the full-LU path. A closed iron core
//!   has k > 0.99; k <= 0.8 is no shared core or a shunted (ballast-type) core
//!   whose leakage flux itself saturates — out of scope, permanently.
//! - W >= 3 windings: the T-model used a per-winding average coupling, a 4 dB
//!   linear error at 20 Hz with k = (0.95, 0.6, 0.6).
//! - Several ISATs on one core: the first silently won.

use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn mna_err(spice: &str) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    match MnaSystem::from_netlist(&netlist) {
        Ok(_) => panic!("expected a refusal for:\n{spice}"),
        Err(e) => e.to_string(),
    }
}

fn mna_ok(spice: &str) {
    let netlist = Netlist::parse(spice).expect("parse");
    MnaSystem::from_netlist(&netlist).expect("must build");
}

#[test]
fn loose_coupled_saturating_pair_is_refused() {
    for k in ["0.5", "0.8"] {
        let e = mna_err(&format!(
            "loose\nR1 in a 100\nL1 a 0 100m ISAT=20m\nL2 b 0 100m\nK1 L1 L2 {k}\nR2 b 0 1k\n"
        ));
        assert!(e.contains("carry ISAT") && e.contains("ballast"), "{e}");
    }
}

#[test]
fn three_winding_saturating_transformer_is_refused() {
    let e = mna_err(
        "three\nR1 in a 100\nL1 a 0 100m ISAT=20m\nL2 b 0 100m\nL3 c 0 100m\n\
         K1 L1 L2 0.95\nK2 L1 L3 0.95\nK3 L2 L3 0.95\nR2 b 0 1k\nR3 c 0 1k\n",
    );
    assert!(e.contains("3 windings"), "{e}");
}

#[test]
fn conflicting_saturation_currents_on_one_core_are_refused() {
    let e = mna_err(
        "conflict\nR1 in a 100\nL1 a 0 100m ISAT=20m\nL2 b 0 100m ISAT=30m\n\
         K1 L1 L2 0.99\nR2 b 0 1k\n",
    );
    assert!(e.contains("different saturation currents"), "{e}");
}

#[test]
fn covered_saturating_groups_still_build() {
    // Tight pair, ISAT on one winding.
    mna_ok("tight\nR1 in a 100\nL1 a 0 100m ISAT=20m\nL2 b 0 100m\nK1 L1 L2 0.95\nR2 b 0 1k\n");
    // Two ISATs that agree once referred to the larger winding
    // (20 mA * sqrt(100m/400m) = 10 mA).
    mna_ok(
        "agree\nR1 in a 100\nL1 a 0 100m ISAT=20m\nL2 b 0 400m ISAT=10m\n\
         K1 L1 L2 0.99\nR2 b 0 1k\n",
    );
    // A non-saturating loose pair is untouched.
    mna_ok("linear\nR1 in a 100\nL1 a 0 100m\nL2 b 0 100m\nK1 L1 L2 0.5\nR2 b 0 1k\n");
}
