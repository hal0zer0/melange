//! A `.linearize`d triode keeps its inter-electrode capacitances.
//!
//! The cap re-stamp after the linearize rebuild covers only the devices left
//! in the nonlinear system, so a linearized triode's `CCG`/`CGP`/`CCP` were
//! dropped: a common-cathode stage with CGP = 100 pF behind 100 kOhm read
//! flat to 20 kHz (+34.5 dB) where the unlinearized deck rolls off through
//! its Miller pole (-3.5 dB at 20 kHz). The capacitances are linear, so the
//! oracle is exact: the linearized circuit's capacitance matrix equals the
//! full circuit's.

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::OpampRailMode;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::pipeline::apply_linearize_reductions;

const CC: &str = "triode common cathode with inter-electrode caps
VCC vcc 0 DC 250
VD d 0 DC 0
Rs d g 100k
Rg g 0 1Meg
T1 g p k TX
Ra vcc p 100k
Rk k 0 1.5k
Ck k 0 100u
.model TX TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300 CCG=2.3p CGP=100p CCP=0.9p)
.linearize T1
";

#[test]
fn a_linearized_triode_keeps_its_inter_electrode_caps() {
    let netlist = Netlist::parse(CC).unwrap();

    let mut full = MnaSystem::from_netlist(&netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&full)).unwrap();
    full.stamp_device_junction_caps(&slots);

    let mut lin = MnaSystem::from_netlist(&netlist).unwrap();
    apply_linearize_reductions(
        &mut lin,
        &netlist,
        &Default::default(),
        &Default::default(),
        &[],
        OpampRailMode::Auto.into(),
        &|_| {},
    )
    .unwrap();
    assert_eq!(lin.m, 0, "the triode must be linearized");
    assert_eq!(full.node_map, lin.node_map);

    let g = full.node_map["g"] - 1;
    let p = full.node_map["p"] - 1;
    assert!(-full.c[g][p] > 50e-12, "the fixture's CGP is not in C");
    for (i, (rf, rl)) in full.c.iter().zip(&lin.c).enumerate() {
        for (j, (&f, &l)) in rf.iter().zip(rl).enumerate() {
            assert!(
                (f - l).abs() <= 1e-9 * f.abs().max(1e-15),
                "C[{i}][{j}]: full {f:e}, linearized {l:e}"
            );
        }
    }
}
