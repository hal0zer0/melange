//! A reduction that rebuilds the MNA keeps every port the build stamped.
//!
//! `.linearize`, `--bjt-fa` and `--tube-grid-fa on` rebuild the MNA from the
//! netlist. The build had already stamped each input port's and each
//! `.inject` source's conductance into G; the rebuild re-stamped only the
//! primary input, so a `.inject` deck with a reduction shipped a circuit with
//! the injection's impedance missing — kernel and DC operating point alike.
//! (A multi-input deck never reaches a rebuild: it is refused while any
//! nonlinear device remains, and the rebuild restamps its extra ports anyway.)
//! Witness: the collector below, with a 10 kΩ injection to ground, sits at
//! 3.476 V; with `.linearize Q1` or `--bjt-fa force` the build put it at
//! 6.953 V, the collector with no injection at all.

mod support;

use melange_solver::codegen::{BjtFaMode, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

/// A common-emitter stage, forward-active at rest, with a 10 kΩ injection at
/// its collector. Emitter degeneration keeps it on the DK route, where the
/// forward-active reduction applies.
const INJECTED_CE: &str = "\
inject at a collector
V1 vcc 0 9
Rb1 vcc a 100k
Rb2 a 0 22k
C1 in a 1u
Q1 c a e QN
Re e 0 4.7k
Rc vcc c 10k
C2 c out 1u
Rl out 0 100k
.inject c fb R=10k
.model QN NPN(IS=1e-14 BF=100)
";

fn config(spice: &str, bjt_fa_mode: BjtFaMode) -> CodegenConfig {
    let mna = MnaSystem::from_netlist(&Netlist::parse(spice).unwrap()).unwrap();
    CodegenConfig {
        circuit_name: "rebuild_ports".to_string(),
        sample_rate: 48000.0,
        input_node: mna.node_map["in"] - 1,
        output_nodes: vec![mna.node_map["out"] - 1],
        bjt_fa_mode,
        ..CodegenConfig::default()
    }
}

/// `DC_OP[NODE_C]` of the shipped build.
fn v_collector(spice: &str, bjt_fa_mode: BjtFaMode) -> f64 {
    let code = support::build_as_shipped(spice, &config(spice, bjt_fa_mode), "auto").0;
    let idx: usize = code
        .lines()
        .find_map(|l| l.strip_prefix("pub const NODE_C: usize = "))
        .and_then(|v| v.trim_end_matches(';').parse().ok())
        .expect("NODE_C");
    let dc_op = code
        .lines()
        .find_map(|l| l.strip_prefix("pub const DC_OP: [f64; N] = ["))
        .expect("DC_OP");
    dc_op
        .trim_end_matches("];")
        .split(',')
        .nth(idx)
        .and_then(|v| v.trim().parse().ok())
        .expect("DC_OP entry")
}

#[test]
fn reductions_keep_the_injection_conductance() {
    let full = v_collector(INJECTED_CE, BjtFaMode::Off);
    let no_injection = v_collector(
        &INJECTED_CE.replace(".inject c fb R=10k\n", ""),
        BjtFaMode::Off,
    );
    // Precondition: the injection's 10 kΩ to ground moves the collector.
    assert!(
        (full - no_injection).abs() > 1.0,
        "collector {full:.4} V with the injection, {no_injection:.4} V without"
    );

    let linearized = format!("{INJECTED_CE}.linearize Q1\n");
    for (what, v) in [
        (".linearize", v_collector(&linearized, BjtFaMode::Off)),
        ("--bjt-fa force", v_collector(INJECTED_CE, BjtFaMode::Force)),
    ] {
        assert!(
            (v - full).abs() < 1e-3,
            "{what}: collector {v:.4} V, unreduced {full:.4} V, with no injection \
             {no_injection:.4} V"
        );
    }
}
