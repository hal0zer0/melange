//! The nodal Schur-vs-full-LU COST rule (`codegen::route_cost`): the last
//! trigger in the sub-path chain, consulted only for a circuit every
//! structural and conditioning guard has left on Schur.
//!
//! Pinned here, on synthetic decks the rule's inputs (N, M, packed back-solve
//! flops) place well inside each side:
//!
//! 1. a 30-stage diode ladder (N = 31, M = 30) routes full-LU with trigger
//!    `cost` — the dense Schur matvec and the dense 30×30 elimination against
//!    a tridiagonal packed factor. This deck sits off the corpus's (M,
//!    solve_flops) trend, where the fit's two full-LU terms are collinear, so
//!    its outcome was MEASURED, not only predicted: `bench.sh` 2026-11-04, one
//!    core, 0.3 V 220 Hz, Schur 6692 ns vs full-LU 1649 ns per sample (ratio
//!    4.06; 1.98 Newton iterations per sample on both routes, no ceiling,
//!    sub-step or unsolved sample on either) against the emitter's predicted
//!    8278 vs 4251 ns (1.95): the fit over-prices full-LU's M term off-trend
//!    (3·37·30 ≈ 3330 of the 4251 ns; Schur itself is over-predicted 1.24×,
//!    full-LU 2.58×), so it under-predicts the RATIO. That direction cannot
//!    cause a false Schur→full-LU flip, only a missed one (a high-M deck left
//!    on Schur when full-LU is cheaper: a speed loss, not a model change); if
//!    that ever matters, a ladder family in the fit separates the M and
//!    solve_flops terms;
//! 2. a 5-stage ladder of the same kind stays on Schur (ratio well under
//!    `COST_RATIO_MAX`);
//! 3. the 30-stage ladder with a relaxing-section glow lamp on it stays on
//!    Schur: full-LU is not the same model for a sectioned glow (it is refused
//!    there), and a cost heuristic must never turn a deck that builds today
//!    into a refusal;
//! 4. the 30-stage ladder with a linearized BJT stage stays on Schur: under
//!    the linearized bypass the full-LU Newton is documented to degrade on
//!    long chains, which the fit never measured.
//!
//! The corpus decision (the rule moves exactly one corpus deck, the revised
//! FET limiter) is the ignored sweep at the bottom, which reads the sibling
//! `melange-circuits` checkout like `f7_routing_predicate_reachability_tests`.

mod support;

use melange_solver::codegen::route_cost::{predict, COST_RATIO_MAX};
use melange_solver::codegen::NodalSubPath;

const SR: f64 = 48000.0;

/// `stages` cascaded RC sections, each shunted by a diode and 100 Ω to ground
/// (the shunt keeps every |K| entry under 1e3 so no magnitude guard can fire): a path
/// graph whose packed factor is tridiagonal, with one Newton dimension per
/// stage. `extra` is appended before `.end`.
fn ladder(stages: usize, extra: &str) -> String {
    let mut s =
        String::from("diode clipper ladder (route cost rule witness)\n.model DX D(IS=1e-14 N=1)\n");
    let mut prev = "in".to_string();
    for k in 1..=stages {
        let node = if k == stages {
            "out".to_string()
        } else {
            format!("n{k}")
        };
        s.push_str(&format!(
            "R{k} {prev} {node} 1k\nD{k} {node} 0 DX\nC{k} {node} 0 1n\nRs{k} {node} 0 100\n"
        ));
        prev = node;
    }
    s.push_str(extra);
    s.push_str(".end\n");
    s
}

fn build(spice: &str) -> melange_solver::build::Built {
    let config = support::config_for_spice(spice, SR);
    support::build_shipped(spice, &config, "nodal")
}

#[test]
fn thirty_stage_ladder_routes_full_lu_on_cost() {
    let spice = ladder(30, "");
    let built = build(&spice);
    assert_eq!(built.generated.m, 30);
    assert_eq!(
        built.generated.meta.nodal_sub_path,
        Some(NodalSubPath::FullLu),
        "{:?}",
        built.generated.meta.nodal_full_lu_trigger
    );
    assert_eq!(built.generated.meta.nodal_full_lu_trigger, Some("cost"));
    // The prediction, re-evaluated here with 4·N as an ESTIMATE of the
    // tridiagonal factor's back-solve flops (the emitter read 182 from the
    // IR): well above the threshold, not a knife-edge, and under the measured
    // ratio of 4.06 (module doc).
    let p = predict(built.generated.n, 30, Some(4 * built.generated.n)).unwrap();
    assert!(p.ratio() > 1.25 * COST_RATIO_MAX, "{p:?}");
}

#[test]
fn five_stage_ladder_stays_on_schur() {
    let spice = ladder(5, "");
    let built = build(&spice);
    assert_eq!(built.generated.m, 5);
    assert_eq!(
        built.generated.meta.nodal_sub_path,
        Some(NodalSubPath::Schur)
    );
    assert_eq!(built.generated.meta.nodal_full_lu_trigger, None);
}

#[test]
fn cost_rule_is_inert_on_a_sectioned_glow_deck() {
    // An unlit relaxing-section neon across one ladder node: a device whose
    // lit branch full-LU cannot run (it would be refused there).
    let spice = ladder(
        30,
        ".model NE1 NEON(VO=135 VM=93 IK=1.5e-3 RS=3000 IHOLD=2e-4 ROFF=300e6 RT=0 K1=1.0 TAU1=300e-6 KSUB=2.0)\n\
         N1 n15 0 NE1\n",
    );
    let built = build(&spice);
    assert_ne!(
        built.generated.meta.nodal_full_lu_trigger,
        Some("cost"),
        "the cost rule must not route a sectioned-glow deck into the full-LU refusal"
    );
    assert_eq!(
        built.generated.meta.nodal_sub_path,
        Some(NodalSubPath::Schur),
        "{:?}",
        built.generated.meta.nodal_full_lu_trigger
    );
}

#[test]
fn cost_rule_is_inert_under_the_linearized_bypass() {
    // One biased BJT stage hung off the ladder, linearized at its DC OP.
    let spice = ladder(
        30,
        ".model QN NPN(IS=1e-14 BF=100)\n\
         Vcc vcc 0 DC 9\n\
         Rb1 vcc b1 470k\n\
         Rc1 vcc c1 1k\n\
         Q1 c1 b1 0 QN\n\
         Rq n15 b1 100k\n\
         .linearize Q1\n",
    );
    let built = build(&spice);
    assert_eq!(built.generated.m, 30, "the BJT must be linearized out of M");
    assert_ne!(built.generated.meta.nodal_full_lu_trigger, Some("cost"));
    assert_eq!(
        built.generated.meta.nodal_sub_path,
        Some(NodalSubPath::Schur),
        "{:?}",
        built.generated.meta.nodal_full_lu_trigger
    );
}

/// The corpus decision: over the sibling `melange-circuits` checkout
/// (unstable/ and testing/) and the golden decks, the cost rule moves exactly
/// the revised FET limiter and nothing else. Ignored by default: the sibling
/// checkout is absent in CI. Run with:
///
/// ```text
/// cargo test -p melange-solver --test route_cost_rule_tests -- --include-ignored --nocapture
/// ```
///
/// RESULT (2026-11-04, every deck at its default flags): one deck, the FET
/// limiter whose working copy is the Q6 revision (N = 84, M = 28, ratio 2.33);
/// the committed form of that deck routes full-LU on `k-diag-negative` before
/// the rule is consulted. Next-highest auto-Schur ratios: rexi-mockup 1.07,
/// passive-eq1a 0.99.
#[test]
#[ignore = "sweeps the melange-circuits sibling checkout; absent in CI"]
fn corpus_sweep_moves_only_the_revised_fet_limiter() {
    let home = std::env::var_os("HOME")
        .map(std::path::PathBuf::from)
        .unwrap();
    let circuits = home.join("dev/melange-circuits");
    if !circuits.is_dir() {
        eprintln!("SKIP: ~/dev/melange-circuits not present");
        return;
    }
    let mut files = Vec::new();
    for sub in ["unstable", "testing"] {
        collect_cir(&circuits.join(sub), &mut files);
    }
    collect_cir(
        &std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../tools/golden-harness/decks"),
        &mut files,
    );
    files.sort();
    let mut moved = Vec::new();
    let mut built = 0usize;
    for f in &files {
        let Ok(text) = std::fs::read_to_string(f) else {
            continue;
        };
        // Only decks with named `in` and `out` nodes build standalone (the
        // rest are modules); skip the others like the census does.
        let Ok(netlist) = melange_solver::parser::Netlist::parse(&text) else {
            continue;
        };
        let Ok(mna) = melange_solver::mna::MnaSystem::from_netlist(&netlist) else {
            continue;
        };
        if !mna.node_map.contains_key("in") || !mna.node_map.contains_key("out") {
            continue;
        }
        let config = support::config_for_spice(&text, SR);
        let Ok(b) = support::try_build_shipped(&text, &config, "auto") else {
            continue;
        };
        built += 1;
        if b.generated.meta.nodal_full_lu_trigger == Some("cost") {
            moved.push(f.file_stem().unwrap().to_string_lossy().to_string());
        }
    }
    eprintln!(
        "route cost sweep: {} decks built, moved by the cost rule: {moved:?}",
        built
    );
    assert!(built > 50, "the sweep built only {built} decks");
    for name in &moved {
        assert_eq!(
            name, "compressor76",
            "the cost rule moved an unexpected deck: {moved:?}"
        );
    }
}

fn collect_cir(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            collect_cir(&p, out);
        } else if p.extension().is_some_and(|x| x == "cir") {
            out.push(p);
        }
    }
}
