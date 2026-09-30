//! The DC operating point is the diode law's own fixed point, the one the
//! transient solves.
//!
//! The DC solve added its 1e-12 S diode GMIN to the current as well as to
//! the Jacobian, so it solved a leakier diode than the transient: a reverse
//! diode fed from 10 V through 1 GΩ read 9.9800 V (that GMIN plus the node
//! Gmin) where ngspice with GMIN off gives 9.99999 V, and the transient then
//! relaxed away from the point it started at. The GMIN stays in the
//! Jacobian only, as Newton conditioning.
//!
//! Reference: ngspice .op with GMIN off and an explicit 1 TΩ from the node to
//! ground, which is the DC solve's own node Gmin (a separate open item,
//! STATUS "Node Gmin moves the fixed point"): 9.990000000000 V.

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

#[test]
fn a_reverse_diode_s_leak_is_the_diode_law() {
    let netlist = Netlist::parse(
        "reverse diode behind 1G\nVCC vcc 0 DC 10\nR1 vcc a 1G\nD1 0 a DX\n.model DX D(IS=1e-14)\n",
    )
    .unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
    let dc = solve_dc_operating_point(&mna, &slots, &DcOpConfig::default());
    assert!(dc.converged);
    let v = dc.v_node[mna.node_map["a"] - 1];
    assert!(
        (v - 9.990000000000).abs() < 2e-8,
        "v(a) {v:.12} vs ngspice (GMIN off, node Gmin explicit) 9.990000000000"
    );
}
