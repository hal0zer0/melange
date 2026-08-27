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
    routing::auto_route(&kernel, &mna, dk_failed)
}

#[test]
fn behavioral_source_sets_behavioral_blocker() {
    let d = route(
        "Behavioral clipper\n\
         Rin in n1 1k\n\
         B1 n1 0 I={tanh(5.0*V(in)) * 1.0e-3}\n\
         R1 n1 0 1k\n",
    );
    assert!(d.behavioral, "behavioral B-source must set routing.behavioral");
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
