//! A `.linearize` that cannot hold is refused at compile time, naming the
//! evidence: a device already outside the region its small-signal model
//! assumes at its own operating point, or a name that is not a BJT or triode.
//! A triode at or past its grid's conduction onset used to be silently kept
//! nonlinear against the directive, and a saturated BJT was linearized and
//! then out of its region on every sample.

use melange_solver::codegen::OpampRailMode;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::pipeline::apply_linearize_reductions;

/// The refusal message for `deck`, or a panic if it was accepted.
fn refusal(deck: &str) -> String {
    let netlist = Netlist::parse(deck).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    match apply_linearize_reductions(
        &mut mna,
        &netlist,
        &Default::default(),
        &Default::default(),
        &[],
        OpampRailMode::Auto.into(),
        &|_| {},
    ) {
        Ok(_) => panic!("accepted:\n{deck}"),
        Err(e) => e.to_string(),
    }
}

const BJT: &str = ".model QX NPN(IS=1e-14 BF=200)\nQ1 c b e QX\nRC vcc c 4.7k\nVCC vcc 0 DC 12\n\
                   .linearize Q1\n";

#[test]
fn a_bjt_saturated_at_its_operating_point_is_refused() {
    // 2.2 mA through 470 Ohm of emitter puts the collector below the base.
    let deck = format!("sat\n{BJT}R1 vcc b 100k\nR2 b 0 22k\nRE e 0 470\n");
    let e = refusal(&deck);
    assert!(
        e.contains("Q1 is saturated at its own operating point (Vbc = +"),
        "{e}"
    );
}

#[test]
fn a_bjt_cut_off_at_its_operating_point_is_refused() {
    // Base held at -1 V: the B-E junction is reverse biased.
    let deck = format!("off\n{BJT}VB b 0 DC -1\nRE e 0 1k\n");
    let e = refusal(&deck);
    assert!(
        e.contains("Q1 is cut off at its own operating point"),
        "{e}"
    );
}

const TRIODE: &str = ".model TX TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)\n\
                      T1 g p 0 TX\nRa vcc p 100k\nVCC vcc 0 DC 250\n.linearize T1\n";

#[test]
fn a_triode_grid_past_its_onset_at_its_operating_point_is_refused() {
    let deck = format!("grid on\n{TRIODE}VG gd 0 DC 1\nRG gd g 10k\n");
    let e = refusal(&deck);
    assert!(e.contains("T1's grid is past its conduction onset"), "{e}");
}

#[test]
fn a_triode_cut_off_at_its_operating_point_is_refused() {
    let deck = format!("cut off\n{TRIODE}VG g 0 DC -30\n");
    let e = refusal(&deck);
    assert!(
        e.contains("T1 is cut off at its own operating point"),
        "{e}"
    );
}

#[test]
fn a_name_that_is_not_a_bjt_or_triode_is_refused() {
    let deck = "not a device\nR1 a 0 1k\nV1 a 0 DC 1\nD1 a 0 DX\n.model DX D\n.linearize D1\n";
    let e = refusal(deck);
    assert!(e.contains("'D1' is not a BJT or triode"), "{e}");
}
