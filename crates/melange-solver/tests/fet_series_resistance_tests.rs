//! A JFET or MOSFET card's series resistance (`RD=`/`RS=`) is refused, not
//! ignored.
//!
//! It used to be accepted and reach only the Newton Jacobian: the currents
//! were evaluated at the external terminal voltages, so the converged answer
//! was the device WITHOUT the resistance. A JFET with RS = 1k biased at
//! v(d) = 4.5717 V, exactly the no-RS deck, where ngspice gives 8.4026 V; an
//! NMOS with RS = 1k, 10.4214 V against 10.9806 V. Until it is in the
//! solution the card value is refused, naming the explicit resistor that
//! models it, and that resistor matches ngspice.

mod support;

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn jfet(card: &str, series: bool) -> String {
    let (source, rs) = if series {
        ("si", "Rsint si s 1k\n")
    } else {
        ("s", "")
    };
    format!(
        "jfet stage\nVCC vcc 0 DC 12\nRG in 0 1Meg\nRD1 vcc d 4.7k\nJ1 d in {source} JX\n{rs}\
         RSx s 0 470\n.model JX NJF(VTO=-2 BETA=1e-3{card})\n"
    )
}

fn nmos(card: &str, series: bool) -> String {
    let (source, rs) = if series {
        ("si", "Rsint si s 1k\n")
    } else {
        ("s", "")
    };
    format!(
        "nmos stage\nVCC vcc 0 DC 12\nRdummy in 0 1k\nRG1 vcc g 1Meg\nRG2 g 0 330k\n\
         RD1 vcc d 4.7k\nM1 d g {source} {source} MX\n{rs}RSx s 0 470\n\
         .model MX NMOS(VTO=2 KP=1e-3{card})\n"
    )
}

fn refusal(spice: &str) -> String {
    let config = support::config_for_spice(spice, 48000.0);
    match support::try_build_shipped(spice, &config, "auto") {
        Ok(_) => panic!("expected a refusal for:\n{spice}"),
        Err(e) => e,
    }
}

#[test]
fn a_fet_card_s_series_resistance_is_refused() {
    for (deck, name) in [
        (jfet(" RS=1k", false), "RS=1000 is refused: JFET source"),
        (jfet(" RD=220", false), "RD=220 is refused: JFET drain"),
        (nmos(" RS=1k", false), "RS=1000 is refused: MOSFET source"),
        (nmos(" RD=10", false), "RD=10 is refused: MOSFET drain"),
    ] {
        let e = refusal(&deck);
        assert!(e.contains(name) && e.contains("explicit resistor"), "{e}");
    }
    // Zero is the model without it, which is what the solution computes.
    for deck in [jfet(" RS=0 RD=0", false), nmos(" RS=0 RD=0", false)] {
        let mut config = support::config_for_spice(&deck, 48000.0);
        config.output_nodes = vec![support::node_index(&deck, "d")];
        support::try_build_shipped(&deck, &config, "auto").expect("zero builds");
    }
}

/// The resistor the refusal names gives ngspice's operating point for the
/// card that set it: ngspice .op on the RS = 1k card, converged tight
/// (reltol 1e-12) with GMIN off. At ngspice's default GMIN (1e-12 S across
/// the JFET's gate junctions, which melange does not model yet) the JFET
/// reads 2.2e-5 V lower. The MOSFET is held to 5e-6 V rather than 1e-6: its
/// gate sits at ~3 V behind a 250k divider, where the DC solve's own 1e-12 S
/// node Gmin moves it (STATUS.md, "Node Gmin moves the fixed point"); the
/// JFET's gate sits at 0 V, where it does not.
#[test]
fn the_explicit_resistor_matches_ngspice() {
    for (deck, v_d, v_s, tol) in [
        (jfet("", true), 8.402641638711, 0.3597358361195, 1e-6),
        (nmos("", true), 10.98062401439, 0.1019375985609, 5e-6),
    ] {
        let netlist = Netlist::parse(&deck).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
        let dc = solve_dc_operating_point(&mna, &slots, &DcOpConfig::default());
        assert!(dc.converged);
        let at = |n: &str| dc.v_node[mna.node_map[n] - 1];
        assert!(
            (at("d") - v_d).abs() < tol,
            "v(d) {} vs ngspice {v_d}",
            at("d")
        );
        assert!(
            (at("s") - v_s).abs() < tol,
            "v(s) {} vs ngspice {v_s}",
            at("s")
        );
    }
}
