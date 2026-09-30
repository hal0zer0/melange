//! Saturating coupled groups the shared-core model does not cover are refused.
//!
//! Each case below compiled silently before, and each was wrong:
//! - k <= 0.8: the old per-winding path was a no-op without a `.pot`/`.switch`
//!   (a K=0.5 pair at 2.5x Isat rendered bit-identical to the linear deck),
//!   per-winding saturation with one (physically wrong: under load the winding
//!   MMFs cancel), and absent altogether on the full-LU path. A closed iron core
//!   has k > 0.99; k <= 0.8 is no shared core or a shunted (ballast-type) core
//!   whose leakage flux itself saturates — out of scope, permanently.
//! - W >= 3 windings with no stated core: the T-model used a per-winding
//!   average coupling, a 4 dB linear error at 20 Hz with k = (0.95, 0.6, 0.6).
//!   The linear [L] does not fix the split into core and leakage, so the deck
//!   states it (TURNS= on every winding, LM= on one).
//! - Several ISATs on one core: the first silently won.

mod support;

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

/// On a shared core an authored LAIR is the winding's TOTAL air-core
/// self-inductance, of which the leakage (1 - k) is already declared by K; a
/// CORE= class (or the default) is the core's MAGNETIZING floor directly.
/// Declarations on both windings each imply a magnetizing floor. They are
/// estimates good to a factor of 3 (the class estimate's own band): within
/// it the least is used, with a notice; beyond it the deck is refused.
#[test]
fn conflicting_air_core_floors_on_one_core_are_refused() {
    // k = 0.9999, L_ref = 400m: LAIR=2e-3 leaves 1.9e-3, CORE=steel gives 3e-4,
    // 6.3x apart.
    let e = mna_err(
        "conflict\nR1 in a 100\nL1 a 0 100m ISAT=20m LAIR=2e-3\nL2 b 0 400m ISAT=10m CORE=steel\n\
         K1 L1 L2 0.9999\nR2 b 0 1k\n",
    );
    assert!(e.contains("more than 3x apart"), "{e}");
    // LAIR=6e-4 leaves 5e-4 against 3e-4: within the band, the least is used.
    let floor = mag_floor_frac(
        "within\nR1 in a 100\nL1 a 0 100m ISAT=20m LAIR=6e-4\nL2 b 0 400m ISAT=10m CORE=steel\n\
         K1 L1 L2 0.9999\nR2 b 0 1k\n",
    );
    assert!((floor - 3e-4 / 0.9999).abs() < 1e-12, "{floor}");
    // LAIR=4e-4 leaves 3e-4 = CORE=steel: one core, one floor.
    mna_ok(
        "same\nR1 in a 100\nL1 a 0 100m ISAT=20m LAIR=4e-4\nL2 b 0 400m ISAT=10m CORE=steel\n\
         K1 L1 L2 0.9999\nR2 b 0 1k\n",
    );
    mna_ok(
        "one\nR1 in a 100\nL1 a 0 100m ISAT=20m CORE=gapped\nL2 b 0 400m ISAT=10m\n\
         K1 L1 L2 0.99\nR2 b 0 1k\n",
    );
}

/// The magnetizing branch's air floor, as a fraction of that branch.
fn mag_floor_frac(spice: &str) -> f64 {
    let netlist = Netlist::parse(spice).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("must build");
    mna.inductors
        .iter()
        .find_map(|l| l.shared_core.as_ref().map(|c| c.floor_frac))
        .expect("a magnetizing branch")
}

/// An authored LAIR no larger than the leakage 1 - k contradicts the deck's
/// own K: there is no magnetizing air floor left. A class or the default never
/// refuses.
#[test]
fn authored_air_floor_inside_the_leakage_is_refused() {
    let e = mna_err(
        "inside\nR1 in a 100\nL1 a 0 1 ISAT=10m LAIR=3e-4\nL2 b 0 1\nK1 L1 L2 0.99\nR2 b 0 1k\n",
    );
    assert!(
        e.contains("leaving no magnetizing") && e.contains("CORE="),
        "{e}"
    );
    mna_ok("default\nR1 in a 100\nL1 a 0 1 ISAT=10m\nL2 b 0 1\nK1 L1 L2 0.99\nR2 b 0 1k\n");
    mna_ok(
        "class\nR1 in a 100\nL1 a 0 1 ISAT=10m CORE=nickel\nL2 b 0 1\nK1 L1 L2 0.99\nR2 b 0 1k\n",
    );
    mna_ok("above\nR1 in a 100\nL1 a 0 1 ISAT=10m LAIR=2e-2\nL2 b 0 1\nK1 L1 L2 0.99\nR2 b 0 1k\n");
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

/// A `.switch` cannot change a saturating inductor. Its flux law (L0, ISAT,
/// floor) is baked at compile time while the switch moves only the linear L,
/// so a switched position solved Φ_L0(i) + (L_new − L0)·i: a 1 H → 0.1 H
/// switch ran away to 740 V from a 10 mV input. A winding of a saturating
/// core has no branch row of its own after the T-model split, so the switch
/// was stamped as a capacitance on the winding's node: output ~1e-6 of normal.
/// Both compiled silently.
#[test]
fn switch_on_saturating_iron_is_refused() {
    let single =
        mna_err("sw\nR1 in out 100\nL1 out 0 1 ISAT=0.1m LAIR=1e-3\n.switch L1 1 0.1 \"Tap\"\n");
    assert!(
        single.contains("L1") && single.contains("saturating inductor"),
        "{single}"
    );
    let core = "xf\nR1 in a 100\nL1 a 0 1 ISAT=10m CORE=steel\nL2 b 0 1\nK1 L1 L2 0.999\n\
                R2 b 0 1k\n";
    for (winding, pos, why) in [
        ("L1", "1 0.5", "saturating inductor"),
        (
            "L2",
            "1 4",
            "winding of a saturating core (L1 carries ISAT)",
        ),
    ] {
        let e = mna_err(&format!("{core}.switch {winding} {pos} \"Tap\"\n"));
        assert!(e.contains(winding) && e.contains(why), "{winding}: {e}");
    }
    // A linear inductor, and a linear coupled pair, still switch.
    mna_ok("lin\nR1 in out 100\nL1 out 0 1\n.switch L1 1 0.1 \"Tap\"\n");
    mna_ok(
        "linx\nR1 in a 100\nL1 a 0 1\nL2 b 0 1\nK1 L1 L2 0.999\nR2 b 0 1k\n\
         .switch L1 1 0.5 \"Tap\"\n",
    );
}

/// Every build with a Newton loop counts a trapezoidal MAX_ITER exhaustion,
/// M = 0 included: a saturating inductor iterates without adding to M, and
/// the counter used to be emitted only for M > 0, so a passive saturating
/// deck could fail Newton on every edge with `diag_nr_max_iter_count == 0`.
#[test]
fn max_iter_counter_is_emitted_for_an_m0_saturating_build() {
    use melange_solver::codegen::CodegenConfig;
    let spice = "rl\nR1 in out 30\nL1 out 0 1 ISAT=10m LAIR=3e-4\n";
    let netlist = Netlist::parse(spice).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");
    assert_eq!(mna.m, 0);
    let config = CodegenConfig {
        circuit_name: "rl".to_string(),
        sample_rate: 48000.0,
        input_node: mna.node_map["in"] - 1,
        output_nodes: vec![mna.node_map["out"] - 1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let code = support::build_as_shipped(spice, &config, "nodal").0;
    assert!(
        code.contains("state.diag_nr_max_iter_count += 1;"),
        "M = 0 saturating build does not count MAX_ITER exhaustion"
    );
}

/// No DK path can carry saturation: the DK entry refuses any MNA with a
/// saturating inductor (the DK IR has no saturating list, so it would have
/// run the core linear), and with it the saturating T-model, whose ideal
/// couplings and (1 - k)·L leakage belong to the nodal full-LU path.
#[test]
fn dk_codegen_refuses_saturating_inductors() {
    use melange_solver::codegen::{CodeGenerator, CodegenConfig};
    use melange_solver::dk::DkKernel;
    let mut reached = 0;
    for spice in [
        "rl\nR1 in out 30\nL1 out 0 1 ISAT=10m\n",
        "xf\nR1 in p 100\nL1 p 0 1 ISAT=10m\nL2 out 0 1\nK1 L1 L2 0.9999\nR2 out 0 1k\n",
    ] {
        let netlist = Netlist::parse(spice).expect("parse");
        let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
        let config = CodegenConfig {
            circuit_name: "dk".to_string(),
            sample_rate: 48000.0,
            input_node: mna.node_map["in"] - 1,
            output_nodes: vec![mna.node_map["out"] - 1],
            input_resistance: 1.0,
            ..CodegenConfig::default()
        };
        // The shipped build refuses `--solver dk` on a saturating circuit.
        assert!(
            support::try_build_shipped(spice, &config, "dk").is_err(),
            "the shipped DK build must refuse a saturating circuit"
        );
        // Bypasses the production pipeline on purpose: tests the DK
        // generator's own refusal, the backstop behind the build's.
        mna.g[config.input_node][config.input_node] += 1.0;
        let Ok(kernel) = DkKernel::from_mna(&mna, 48000.0) else {
            continue; // no kernel, no DK code either
        };
        let err = CodeGenerator::new(config)
            .generate(&kernel, &mna, &netlist)
            .expect_err("DK codegen must refuse a saturating circuit");
        assert!(err.to_string().contains("saturating inductors"), "{err}");
        reached += 1;
    }
    assert!(
        reached > 0,
        "no case reached the DK generator; the test is void"
    );
}

/// The T-model realizes the authored coupling exactly, at any k < 1. Its
/// leakage was floored at 1e-4·L, so every k above 0.9999 silently realized
/// k_eff = k/(k + 1e-4) ≈ 0.9999 (0.15 dB high at 20 kHz on a 1:4 step-up
/// into 1 nF, and the same response at k = 0.99999 and 0.999999). Real audio
/// iron sits at 1 - k ~ 1e-5..1e-4.
#[test]
fn tmodel_realizes_tight_coupling_exactly() {
    for k in [0.99999f64, 0.999999] {
        let spice = format!(
            "xf\nR1 in p 600\nL1 p 0 1 ISAT=10m\nL2 out 0 16\nK1 L1 L2 {k}\nR2 out 0 100k\n"
        );
        let mna = MnaSystem::from_netlist(&Netlist::parse(&spice).unwrap()).unwrap();
        let l = |name: &str| {
            mna.inductors
                .iter()
                .find(|i| i.name.eq_ignore_ascii_case(name))
                .unwrap_or_else(|| panic!("{name} missing"))
                .value
        };
        // Reference winding is the larger one (L2 = 16 H).
        let (leak1, leak2, mag) = (l("L1_leak"), l("L2_leak"), l("L2_mag"));
        assert_eq!(leak1, (1.0 - k) * 1.0, "k={k}: L1 leakage");
        assert_eq!(leak2, (1.0 - k) * 16.0, "k={k}: L2 leakage");
        // Realized coupling: mutual n·mag over sqrt(self1·self2), n = sqrt(L1/L_ref).
        let n = (1.0f64 / 16.0).sqrt();
        let (self1, self2) = (leak1 + n * n * mag, leak2 + mag);
        let k_eff = n * mag / (self1 * self2).sqrt();
        assert!((k_eff - k).abs() < 1e-14, "k={k}: realized {k_eff}");
    }
}
