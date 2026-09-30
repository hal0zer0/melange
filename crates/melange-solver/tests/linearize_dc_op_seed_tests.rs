//! The DC operating point of a `.linearize`d circuit starts from the point the
//! devices were linearized at.
//!
//! `.linearize` solves the full circuit's DC operating point, extracts each
//! flagged device's small-signal parameters there, and rebuilds the system with
//! those devices linear (their Norton constants taken from that point). That
//! point therefore satisfies the linearized system's DC equations exactly, and
//! its DC solve starts there. It used to restart from the junction-clamped
//! linear guess and, on a FET limiter's output stage (a Darlington-equivalent
//! NF = 2 transistor), failed every strategy while the bias solve had
//! converged.
//!
//! Witness: a high-Vbe (NF = 2) common-emitter stage into a linearized
//! emitter follower. With a budget of 3 Newton iterations per attempt, the
//! seeded solve converges and the unseeded one (the linear guess) does not.

use melange_solver::codegen::ir::{dc_op_config, CircuitIR};
use melange_solver::codegen::OpampRailMode;
use melange_solver::dc_op::solve_dc_operating_point;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::pipeline::apply_linearize_reductions;

const DECK: &str = "high-Vbe stage into a linearized follower
.model QHI NPN(IS=2.2e-12 NF=2 BF=1000 VAF=100)
.model QF NPN(IS=1e-14 BF=200 VAF=100)
VCC vcc 0 DC 30
Rin in b1 10k
Rb1 vcc b1 220k
Rb2 b1 0 47k
Q1 c1 b1 e1 QHI
Rc1 vcc c1 4.7k
Re1 e1 0 470
Q2 vcc c1 e2 QF
Re2 e2 0 2.2k
Co e2 out 10u
Rl out 0 10k
.linearize Q2
";

fn linearized() -> (Netlist, MnaSystem) {
    let netlist = Netlist::parse(DECK).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    apply_linearize_reductions(
        &mut mna,
        &netlist,
        &Default::default(),
        &Default::default(),
        &[],
        OpampRailMode::Auto.into(),
        &|_| {},
    )
    .unwrap();
    (netlist, mna)
}

#[test]
fn a_linearized_circuit_s_dc_op_starts_where_it_was_linearized() {
    let (netlist, mna) = linearized();
    assert_eq!(mna.linearized_bjts.len(), 1);
    let seed = mna
        .linearize_bias_nodes
        .clone()
        .expect("the bias solve converged, so its point is recorded");
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();

    let mut config = dc_op_config(&mna, OpampRailMode::Auto);
    config.max_iterations = 3;
    assert!(config.seed_nodes.is_some(), "dc_op_config carries the seed");
    let seeded = solve_dc_operating_point(&mna, &slots, &config);
    assert!(seeded.converged, "seeded: {:?}", seeded.method);
    assert!(seeded.iterations <= 2, "seeded took {}", seeded.iterations);

    // It converges to the point it was seeded with.
    for (name, &x) in &seed {
        let idx = mna.node_map[name];
        let v = seeded.v_node[idx - 1];
        assert!(
            (v - x).abs() <= 1e-6 * x.abs().max(1.0),
            "{name}: {v} vs bias point {x}"
        );
    }

    config.seed_nodes = None;
    let unseeded = solve_dc_operating_point(&mna, &slots, &config);
    assert!(
        !unseeded.converged,
        "from the linear guess the same budget must not suffice, or this is no witness"
    );
}
