//! Regression test for the routing-rate bug (2026-08-14).
//!
//! `routing::auto_route`'s spectral-radius / K-conditioning checks are
//! computed on whatever `DkKernel` they are handed. Before this fix, the
//! CLI (`tools/melange-cli/src/main.rs`) built that kernel at the BASE
//! (host) sample rate even when `--oversampling N` was requested, while
//! codegen ships matrices built at the INTERNAL rate
//! (`sample_rate * oversampling_factor`). `rho(S*A_neg)` is strongly rate-
//! dependent (see `memory/passive_lc_auto_be_fix.md`), so a circuit that
//! looks trap-stable at the base rate can be genuinely trap-unstable at the
//! internal rate the generated solver actually runs at — and the router,
//! never having looked at that rate, ships it on DK Schur anyway.
//!
//! This was diagnosed on a Ge regenerative-LC-oscillator circuit
//! (`melange-circuits/local-docs/g10-osc-sq2-repro.cir`): at 48 kHz the
//! un-reduced-MNA kernel reads comfortably trap-stable, but at the 192 kHz
//! internal rate used by `--oversampling 4` it reads
//! `spectral_radius(S*A_neg) = 1.315` (genuinely trap-unstable, dominant
//! real growing mode — the circuit is a regenerative oscillator, unstable
//! at its DC-OP linearization by design). DK Schur has no full-LU NR rescue
//! path; the generated solver diverged to a nonphysical peak of ~40 kV.
//! Routing to nodal (which the router SHOULD have done, given the correct
//! rate) converges cleanly.
//!
//! This test pins the underlying mechanism `routing::auto_route` depends
//! on: the SAME topology routes differently depending on which rate its
//! kernel was built at. `tools/melange-cli/tests/cli_integration.rs` pins
//! the actual CLI fix (build the routing kernel at the internal rate)
//! end-to-end.

use melange_solver::codegen::routing::{self, SolverRoute};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

/// The exact g10 repro netlist (verbatim copy of
/// `melange-circuits/local-docs/g10-osc-sq2-repro.cir` as of 2026-08-14).
/// Embedded rather than read from the sibling repo so this test is
/// self-contained (the melange-circuits checkout may not be present, e.g.
/// in CI). A Farfisa Compact G10 master LC oscillator (Ge PNP,
/// transformer-coupled regenerative feedback via `K1`) plus one squarer
/// stage. This topology is genuinely unstable at its DC-OP linearization by
/// design (that's what makes it oscillate) — see the netlist's own hazard
/// notes.
const G10_OSCILLATOR: &str = "\
Farfisa Compact G10 reference chain (master osc + squarer + divider + keying)
Vrail rail 0 DC 8
Vvib vterm 0 DC 8
.runtime Vvib as v_g10_vterm
C_kick in b1 1n
R_e18 rail node_a 1.8k
C_e25 rail node_a 25u
L_fb node_b node_a 8.35m
R_180 node_b e1 180
Q_TN1G c1 b1 e1 SFT307
L_tank c1 tanklo 1.36
K1 L_tank L_fb 0.3
C_tank c1 tanklo 13.5n
R_47k tanklo b1 47k
R_27k2 tanklo 0 2.7k
R_10k rail b1 10k
R_27k vterm b1 27k
C_sq c1 sq_n 10n
R_sqb sq_n b2 47k
R_470k b2 0 470k
Q_TN2G c2 b2 rail SFT307
R_c2 c2 0 10k
C_fout c2 term_f 1u
R_fbleed term_f 0 100k
.model SFT307 PNP(IS=2e-7 BF=110 VAF=60 RB=50 RC=5 RE=1 CJE=60p CJC=25p TF=1n)
.model SFT352 PNP(IS=3e-7 BF=90 VAF=50 RB=40 RC=4 RE=1 CJE=80p CJC=30p TF=1n)
";

fn build_mna(input_node_name: &str) -> (Netlist, MnaSystem, usize) {
    let netlist = Netlist::parse(G10_OSCILLATOR).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map[input_node_name] - 1;
    mna.g[input_node][input_node] += 1.0;
    // Mirror the CLI pipeline's junction-cap auto-stamp (`compile`/`simulate`
    // step 2, before kernel construction) — without it the parasitic BJT
    // junction caps are missing from C, which materially shifts the trap
    // spectral radius and desyncs this fixture from the CLI's actual
    // pre-route kernel.
    let device_slots =
        melange_solver::codegen::ir::CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }
    (netlist, mna, input_node)
}

fn route_at_rate(rate: f64) -> routing::RoutingDecision {
    let (_netlist, mna, _input_node) = build_mna("in");
    let kernel = DkKernel::from_mna_augmented(&mna, rate).expect("augmented kernel");
    routing::auto_route(&kernel, &mna, false)
}

/// At a plausible host rate (48 kHz), this oscillator's trap propagation
/// operator must NOT look unstable to the router — establishing the
/// baseline the internal-rate test below contrasts against. If this ever
/// starts failing, the fixture stopped reproducing the "looks fine at base
/// rate" half of the bug and the fixture needs revisiting.
#[test]
fn base_rate_kernel_under_reports_instability() {
    let decision = route_at_rate(48_000.0);
    assert!(
        !decision.dk_unstable,
        "fixture regression: base-rate (48 kHz) kernel must read trap-stable \
         (dk_unstable=false) for this test to isolate the rate-dependence bug; \
         got spectral_radius={:.4}",
        decision.spectral_radius
    );
    assert_eq!(decision.route, SolverRoute::DkSchur);
}

/// At the internal rate a `--oversampling 4` build of this circuit would
/// actually ship (192 kHz), the SAME topology must read genuinely
/// trap-unstable and route to nodal. This is the rate the CLI must build
/// the routing kernel at — building at the base rate (the pre-fix
/// behavior) would silently ship this on DK Schur, which has no full-LU NR
/// rescue for the ensuing device-state-transition NR failure.
#[test]
fn internal_rate_kernel_detects_instability_and_routes_nodal() {
    let decision = route_at_rate(192_000.0);
    assert!(
        decision.dk_unstable,
        "internal-rate (192 kHz) kernel must detect trap instability \
         (spectral_radius={:.4}); the base-rate kernel missed it entirely",
        decision.spectral_radius
    );
    assert_eq!(
        decision.route,
        SolverRoute::Nodal,
        "trap-unstable-at-internal-rate circuit must route to nodal, not DK Schur \
         (DK Schur has no full-LU NR fallback for the resulting divergence)"
    );
    assert!(
        decision.spectral_radius > 1.002,
        "expected spectral radius clearly above the promotion threshold, got {:.4}",
        decision.spectral_radius
    );
}
