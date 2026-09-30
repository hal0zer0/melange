//! A `.linearize`d BJT is its own small-signal model: the linearized circuit's
//! response to a small input step equals the derivative of the full circuit's
//! DC operating point.
//!
//! The linearized stamp used to be rebuilt from bare Ebers-Moll formulas: the
//! B-C junction at Vt instead of NR·Vt, no Gummel-Poon qb (so no Early output
//! resistance), no ISE/ISC leakage, no RB/RC/RE, and B-C conductances stamped
//! into rows the device does not draw them from. Measured at 1 kHz against
//! ngspice `.ac`, a bypassed common-emitter stage with a Gummel-Poon card read
//! +0.71 dB hot, and an NR = 2 stage with its B-C junction forward read
//! 16.7 dB low.
//!
//! Oracle: the central difference of the nonlinear DC operating point in the
//! drive voltage, against the exact slope of the linearized circuit. No
//! reference simulator involved: a linearization is a derivative.

use melange_solver::codegen::ir::{dc_op_config, CircuitIR};
use melange_solver::codegen::OpampRailMode;
use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::pipeline::apply_linearize_reductions;

/// Gummel-Poon NPN with Early, high injection, B-E leakage and all three
/// ohmic parasitics; forward active, emitter lightly degenerated so the
/// output resistance moves the gain.
const GP_NPN: &str = "gummel-poon common emitter
.model QGP NPN(IS=1.5e-14 BF=200 NF=1 VAF=50 IKF=0.2 ISE=5e-13 NE=1.5 BR=4 NR=1 VAR=20 IKR=0.1 RB=10 RE=0.3 RC=0.5)
VCC vcc 0 DC 12
VD d 0 DC 0
Rs d b 10k
R1 vcc b 100k
R2 b 0 22k
RC vcc c 4.7k
Q1 c b e QGP
RE e 0 100
.linearize Q1
";

/// NR = 2 with the base overdriven, so the B-C junction is forward biased
/// and its slope carries the stage.
const NR2_SATURATED: &str = "nr=2 overdriven stage
.model QNR NPN(IS=1e-14 BF=100 NF=1 BR=5 NR=2)
VCC vcc 0 DC 5
VD d 0 DC 0
Rs d b 100k
RB vcc b 100k
RC vcc c 10k
Q1 c b 0 QNR
.linearize Q1
";

/// The Gummel-Poon stage mirrored to PNP, with B-C leakage and NR != 1.
const GP_PNP: &str = "gummel-poon pnp common emitter
.model QGPP PNP(IS=3e-14 BF=250 NF=1.003 VAF=115 IKF=0.01 ISE=5e-15 NE=1.34 BR=3.5 NR=1.2 VAR=26 IKR=0.01 ISC=1.7e-13 NC=1.2 RB=120 RE=0.5 RC=2)
VCC vcc 0 DC -12
VD d 0 DC 0
Rs d b 10k
R1 vcc b 100k
R2 b 0 22k
RC vcc c 4.7k
Q1 c b e QGPP
RE e 0 100
.linearize Q1
";

fn set_drive(mna: &mut MnaSystem, v: f64) {
    let vs = mna
        .voltage_sources
        .iter_mut()
        .find(|vs| vs.name.eq_ignore_ascii_case("VD"))
        .expect("drive source VD");
    vs.dc_value = v;
}

/// d v(node) / d V(VD) at VD = 0 through the full nonlinear circuit.
fn nonlinear_slope(netlist: &Netlist, node: &str, h: f64) -> f64 {
    let mut mna = MnaSystem::from_netlist(netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(netlist, Some(&mna)).unwrap();
    let idx = mna.node_map[node] - 1;
    let mut at = |v: f64| {
        set_drive(&mut mna, v);
        let dc = solve_dc_operating_point(&mna, &slots, &DcOpConfig::default());
        assert!(dc.converged, "nonlinear DC OP at VD = {v}: {:?}", dc.method);
        dc.v_node[idx]
    };
    (at(h) - at(-h)) / (2.0 * h)
}

/// d v(node) / d V(VD) through the circuit with Q1 linearized at VD = 0.
fn linearized_slope(netlist: &Netlist, node: &str) -> f64 {
    let mut mna = MnaSystem::from_netlist(netlist).unwrap();
    let outcome = apply_linearize_reductions(
        &mut mna,
        netlist,
        &Default::default(),
        &Default::default(),
        &[],
        OpampRailMode::Auto.into(),
        &|_| {},
    )
    .unwrap();
    assert!(outcome.bias_unconverged.is_none());
    assert_eq!(mna.linearized_bjts.len(), 1);
    assert_eq!(mna.m, 0, "Q1 is the only nonlinear device");
    let idx = mna.node_map[node] - 1;
    let config = dc_op_config(&mna, OpampRailMode::Auto);
    let mut at = |v: f64| {
        set_drive(&mut mna, v);
        let dc = solve_dc_operating_point(&mna, &[], &config);
        assert!(dc.converged, "linearized DC solve at VD = {v}");
        dc.v_node[idx]
    };
    // Linear, so any step gives the exact slope.
    at(1.0) - at(0.0)
}

fn assert_small_signal_matches(deck: &str) {
    let netlist = Netlist::parse(deck).unwrap();
    for node in ["b", "c"] {
        let reference = nonlinear_slope(&netlist, node, 1e-4);
        let linear = linearized_slope(&netlist, node);
        let rel = (linear - reference).abs() / reference.abs();
        assert!(
            rel < 1e-4,
            "{node}: linearized dV/dVD = {linear:.6e}, nonlinear = {reference:.6e} (rel {rel:.2e})"
        );
    }
}

#[test]
fn gummel_poon_npn_with_parasitics_linearizes_to_its_own_derivative() {
    assert_small_signal_matches(GP_NPN);
}

#[test]
fn nr_2_forward_bc_junction_linearizes_to_its_own_derivative() {
    assert_small_signal_matches(NR2_SATURATED);
}

#[test]
fn gummel_poon_pnp_linearizes_to_its_own_derivative() {
    assert_small_signal_matches(GP_PNP);
}
