//! The DC operating point's start for BJTs whose parasitic RB/RC/RE are
//! expanded into internal nodes b'/c'/e' (every nodal build expands them).
//!
//! The expanded device's N_v rows name the internal nodes. The start acts on
//! the terminals as the circuit sees them: the junction clamp of the linear
//! guess moves the EXTERNAL emitter/base/collector, and the internal nodes
//! are then placed from the externals. From a seeded start (the `.linearize`
//! bias point) every internal node starts at its external terminal, not one
//! junction drop from b' — the seed is a solution at the external nodes.
//!
//! Witness: the Wurlitzer 200A power amplifier as synced 2026-08-03
//! (`data/wurli_power_amp_2026_08_03.cir`, `.linearize Q9`). Its reference is
//! ngspice-42 `.op` of the same deck with Q9 left nonlinear and the build's
//! 1 Ω input source at `in` (`.runtime` lines dropped); tightening ngspice to
//! reltol 1e-7 / vntol 1e-12 leaves every digit below unchanged. The
//! linearized device is rebuilt at the bias point, so the linearized
//! circuit's operating point is that same point.

mod support;

use melange_solver::build::Built;

const WPA_2026_08_03: &str = include_str!("data/wurli_power_amp_2026_08_03.cir");

/// ngspice-42 `.op` node voltages of the deck (see the module docs).
const NGSPICE_OP: [(&str, f64); 15] = [
    ("in_ac", 4.4153478121e-02),
    ("emit_pair", 6.6183441397e-01),
    ("coll7", -2.180955463e+01),
    ("coll8", -2.249999857e+01),
    ("fb_inv", 2.5125668024e-02),
    ("out", -6.295704653e-02),
    ("c10_node", 2.5125668024e-02),
    ("drv_bot", -7.270938762e-01),
    ("boot", 1.1472636730e+01),
    ("vas_out", 4.4527345966e-01),
    ("bias_mid", -5.145774960e-02),
    ("base11", 2.2498346646e+01),
    ("nodec", -6.295404161e-02),
    ("base13", -2.196577722e+01),
    ("noded", -6.666153785e-02),
];

/// Absolute agreement with ngspice, volts. melange's operating point sits
/// within 6e-7 V of ngspice's at every node (largest at `emit_pair`); ngspice's
/// own digits do not move when its tolerances are tightened 1e4-fold. 1e-5 V
/// leaves 15x margin over that model-level difference and is well under any
/// bias error the start defect produced (no convergence at all, KCL 87 A).
const TOL_V: f64 = 1e-5;

fn build(deck: &str) -> Built {
    let config = support::config_for_spice(deck, 48000.0);
    support::try_build_shipped(deck, &config, "auto")
        .unwrap_or_else(|e| panic!("the build refused the deck: {e}"))
}

fn node(built: &Built, name: &str) -> f64 {
    built.dc_op.v_node[built.mna.node_map[name] - 1]
}

fn assert_matches_ngspice(built: &Built, label: &str) {
    assert!(
        built.dc_op.converged,
        "{label}: DC OP did not converge ({:?}, {} iterations, KCL {:.3e} A)",
        built.dc_op.method, built.dc_op.iterations, built.dc_op.kcl_residual_max
    );
    for (name, want) in NGSPICE_OP {
        let got = node(built, name);
        assert!(
            (got - want).abs() < TOL_V,
            "{label}: v({name}) = {got}, ngspice {want} (|diff| {:.3e} V)",
            (got - want).abs()
        );
    }
}

/// The `.linearize Q9` deck: Direct NR starts at the bias point with the
/// parasitic BJTs' internal nodes at their external terminals.
#[test]
fn seeded_start_puts_internal_nodes_at_their_terminals() {
    let built = build(WPA_2026_08_03);
    assert_eq!(
        built.solver_label, "nodal",
        "the witness needs the nodal route"
    );
    assert!(
        !built.mna.bjt_internal_nodes.is_empty(),
        "the witness needs expanded parasitic BJTs"
    );
    assert!(
        built.mna.linearize_bias_nodes.is_some(),
        "the witness needs a seeded start (.linearize Q9 with a converged bias solve)"
    );
    assert_matches_ngspice(&built, ".linearize Q9");
    // From the bias point Newton has almost nothing to do.
    assert!(
        built.dc_op.iterations <= 10,
        "{} iterations from the seed",
        built.dc_op.iterations
    );
}

/// The same circuit without `.linearize`: an unseeded start, so this pins the
/// operating point independently of the seed.
#[test]
fn unseeded_power_amp_matches_ngspice() {
    let deck: String = WPA_2026_08_03
        .lines()
        .filter(|l| !l.starts_with(".linearize"))
        .map(|l| format!("{l}\n"))
        .collect();
    let built = build(&deck);
    assert_eq!(built.solver_label, "nodal");
    assert!(!built.mna.bjt_internal_nodes.is_empty());
    assert!(built.mna.linearize_bias_nodes.is_none());
    assert_matches_ngspice(&built, "no .linearize");
}

/// A grounded-emitter NPN with RB only, biased from 12 V through 1 MΩ: the
/// linear guess holds the external base at 12 V. The clamp moves the external
/// base to 0.65 V (the emitter is ground) and b' starts there. Read from the
/// expanded N_v row it moved b' instead, which the internal-node init then
/// put back at the external base: the intrinsic junction started 12 V into
/// forward bias and Newton walked it down (20 iterations against 4).
/// ngspice-42 `.op` (reltol 1e-9, 1 Ω at `in`): v(b) 0.65950562938 V,
/// v(c) 6.6699650654 V.
const GROUNDED_EMITTER_RB: &str = "\
Grounded-emitter BJT with base resistance
Vcc vcc 0 DC 12
Vb vb 0 DC 12
Rb vb b 1meg
Q1 c b 0 QN
Rc vcc c 4.7k
Cin in b 1u
Co c out 1u
Rl out 0 100k
.model QN NPN(IS=1e-14 BF=100 RB=100)
";

#[test]
fn unseeded_clamp_starts_the_intrinsic_junction_from_the_clamped_terminals() {
    let config = support::config_for_spice(GROUNDED_EMITTER_RB, 48000.0);
    let built = support::try_build_shipped(GROUNDED_EMITTER_RB, &config, "nodal")
        .unwrap_or_else(|e| panic!("the build refused the deck: {e}"));
    assert_eq!(built.mna.bjt_internal_nodes.len(), 1, "Q1's RB is expanded");
    let r = &built.dc_op;
    assert!(r.converged, "{:?}", r.method);
    assert_eq!(r.method, melange_solver::dc_op::DcOpMethod::DirectNr);
    assert!(r.iterations <= 10, "{} iterations", r.iterations);
    let (b, c) = (node(&built, "b"), node(&built, "c"));
    assert!((b - 0.65950562938).abs() < TOL_V, "v(b) = {b}");
    assert!((c - 6.6699650654).abs() < TOL_V, "v(c) = {c}");
}
