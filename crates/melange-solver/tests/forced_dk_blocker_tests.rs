//! Structural-blocker routing fields used to reject a forced `--solver dk`.
//!
//! `RoutingDecision` exposes `behavioral`, `saturating_inductor`,
//! `multi_transformer`, and `k_diag_unsafe`. The CLI rejects `--solver dk` when
//! any of these is set, because DK structurally cannot represent the circuit and
//! would emit a silently-wrong solver (behavioral source dropped, saturating
//! inductor linearized, multi-transformer DC-OP singular, transformer-NFB
//! divergence). These are computed from the CIRCUIT, independent of which
//! first-match routing reason wins — so a circuit that routes nodal for a SOFT
//! reason (e.g. trapezoidal instability) still reports its hard structural
//! blocker. Soft-only circuits must report none, or the override is over-blocked.

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::routing::{self, SolverRoute};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn route(spice: &str) -> routing::RoutingDecision {
    route_with(spice, melange_solver::codegen::OpampRailMode::Auto)
}

fn route_with(
    spice: &str,
    rail: melange_solver::codegen::OpampRailMode,
) -> routing::RoutingDecision {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let device_slots = CircuitIR::build_device_info(&netlist).unwrap_or_default();
    if !device_slots.is_empty() {
        mna.stamp_device_junction_caps(&device_slots);
    }
    let (kernel, dk_failed) = match DkKernel::from_mna(&mna, 48_000.0) {
        Ok(k) => (k, false),
        Err(_) => (
            DkKernel::from_mna_augmented(&mna, 48_000.0).expect("augmented kernel"),
            true,
        ),
    };
    routing::auto_route(&kernel, &mna, dk_failed, rail)
}

#[test]
fn behavioral_source_sets_behavioral_blocker() {
    let d = route(
        "Behavioral clipper\n\
         Rin in n1 1k\n\
         B1 n1 0 I={tanh(5.0*V(in)) * 1.0e-3}\n\
         R1 n1 0 1k\n",
    );
    assert!(
        d.behavioral,
        "behavioral B-source must set routing.behavioral"
    );
    assert_eq!(d.route, SolverRoute::Nodal);
}

#[test]
fn saturating_inductor_sets_blocker_even_under_soft_route_reason() {
    // This deck's first-match routing reason is the SOFT `dk_unstable`
    // (spectral radius > 1.002 from the marginal integrator), not the
    // saturating-inductor branch — yet the hard structural field must still be
    // set, so a forced --solver dk is correctly rejected.
    let d = route(
        "Saturating inductor\n\
         Rin in n1 1k\n\
         L1 n1 out 100m isat=0.5\n\
         Rout out 0 100k\n\
         D1 out 0 D1\n\
         .model D1 D(IS=1e-14)\n",
    );
    assert!(
        d.saturating_inductor,
        "isat inductor must set routing.saturating_inductor regardless of the winning route reason"
    );
    assert_eq!(d.route, SolverRoute::Nodal);
}

#[test]
fn plain_dk_capable_circuit_sets_no_hard_blocker() {
    // A symmetric diode clipper: DK Schur is the correct route, and forcing
    // --solver dk must remain valid — so NONE of the hard blockers may fire.
    let d = route(
        "Diode clipper\n\
         Rin in n1 1k\n\
         D1 n1 0 D1\n\
         D2 0 n1 D1\n\
         Rout n1 out 1k\n\
         Rl out 0 100k\n\
         .model D1 D(IS=2.52e-9 N=1.752)\n",
    );
    assert_eq!(d.route, SolverRoute::DkSchur);
    assert!(!d.behavioral, "plain clipper must not report behavioral");
    assert!(
        !d.saturating_inductor,
        "plain clipper must not report saturating_inductor"
    );
    assert!(
        !d.multi_transformer,
        "plain clipper must not report multi_transformer"
    );
    assert!(
        !d.k_diag_unsafe,
        "plain clipper must not report k_diag_unsafe"
    );
}

/// A single-supply op-amp gain stage whose output is capacitor-coupled on:
/// the auto-resolver picks an active-set mode, which only nodal implements.
const CAP_COUPLED_OPAMP: &str = "Single-supply op-amp stage\n\
    Vcc vcc 0 DC 9\n\
    Rb1 vcc bias 100k\n\
    Rb2 bias 0 100k\n\
    Cin in inp 100n\n\
    Rinb inp bias 1meg\n\
    U1 inp inv out OA9\n\
    Rf out inv 100k\n\
    Rg inv mid 4.7k\n\
    Cg mid 0 1u\n\
    Cout out outac 1u\n\
    Rl outac 0 100k\n\
    .model OA9 OA(AOL=200000 ROUT=75 VCC=9 VEE=0)\n";

#[test]
fn active_set_rail_mode_routes_nodal_and_blocks_forced_dk() {
    let d = route(CAP_COUPLED_OPAMP);
    assert!(
        d.opamp_active_set,
        "a capacitor-coupled clamped op-amp must resolve to an active-set mode ({:?})",
        d.opamp_rail_mode
    );
    assert_eq!(d.route, SolverRoute::Nodal, "{}", d.reason);
}

/// An op-amp stage that routes DK on its own merits (no inductors, no
/// conditioning trouble). Asking for an active-set rail mode is the only thing
/// that can move it, so the routing reason must name it.
const DK_OPAMP_STAGE: &str = "Inverting stage\n\
    R1 in inv 10k\n\
    R2 inv out 100k\n\
    U1 0 inv out OA13\n\
    Rl out 0 10k\n\
    Cl out 0 1n\n\
    .model OA13 OA(AOL=200000 ROUT=75 VCC=13 VEE=-13)\n";

#[test]
fn explicit_active_set_moves_an_otherwise_dk_circuit_to_nodal() {
    use melange_solver::codegen::OpampRailMode;
    let baseline = route_with(DK_OPAMP_STAGE, OpampRailMode::Hard);
    assert_eq!(
        baseline.route,
        SolverRoute::DkSchur,
        "test premise: this deck routes DK under hard ({})",
        baseline.reason
    );
    for mode in [OpampRailMode::ActiveSet, OpampRailMode::ActiveSetBe] {
        let d = route_with(DK_OPAMP_STAGE, mode);
        assert!(d.opamp_active_set);
        assert_eq!(d.route, SolverRoute::Nodal);
        assert!(
            d.reason.contains("active-set rail handling"),
            "{}",
            d.reason
        );
    }
}

#[test]
fn explicit_hard_rail_mode_stays_allowed_on_dk() {
    let d = route_with(
        CAP_COUPLED_OPAMP,
        melange_solver::codegen::OpampRailMode::Hard,
    );
    assert!(
        !d.opamp_active_set,
        "an explicit hard mode is not active-set"
    );
    assert_ne!(
        d.reason.split(' ').next(),
        Some("op-amp"),
        "hard must not be routed for the rail-mode reason: {}",
        d.reason
    );
}

/// `AOL_TRANSIENT_CAP` is applied to the transient matrices by the nodal IR
/// builder only; the DK kernel is built from the uncapped `G`. A card that sets
/// it routes nodal, and the flag lets the CLI refuse a forced `--solver dk`
/// rather than ship the cap silently dropped.
#[test]
fn an_author_transient_aol_cap_routes_nodal() {
    use melange_solver::codegen::OpampRailMode;
    let baseline = route_with(DK_OPAMP_STAGE, OpampRailMode::Hard);
    assert_eq!(baseline.route, SolverRoute::DkSchur, "{}", baseline.reason);
    assert!(!baseline.opamp_transient_aol_cap);
    let capped = DK_OPAMP_STAGE.replace(
        "OA(AOL=200000 ROUT=75 VCC=13 VEE=-13)",
        "OA(AOL=200000 ROUT=75 VCC=13 VEE=-13 AOL_TRANSIENT_CAP=1000)",
    );
    assert_ne!(capped, DK_OPAMP_STAGE);
    let d = route_with(&capped, OpampRailMode::Hard);
    assert!(d.opamp_transient_aol_cap);
    assert_eq!(d.route, SolverRoute::Nodal);
    assert!(d.reason.contains("AOL_TRANSIENT_CAP"), "{}", d.reason);
}
