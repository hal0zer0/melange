//! `ParseOptions::disable_self_heating` (set by `melange validate`) resolves
//! every device isothermal: a card's `RTH` is not applied, while `TAMB`
//! still sets the device's static temperature (its `IS` and thermal voltage
//! scaled from TNOM).

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::device_types::DeviceParams;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::{Netlist, ParseOptions};

const DECK: &str = "diode\nR1 in a 1k\nD1 a 0 DX\n\
                    .model DX D(IS=2.52e-9 N=1.752 RTH=50 CTH=2e-3 TAMB=320)\n";

fn diode(options: ParseOptions) -> melange_solver::device_types::DiodeParams {
    let netlist = Netlist::parse_with_options(DECK, options).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
    match &slots[0].params {
        DeviceParams::Diode(d) => d.clone(),
        other => panic!("{other:?}"),
    }
}

#[test]
fn isothermal_drops_rth_and_keeps_the_static_temperature() {
    let shipped = diode(ParseOptions::default());
    let iso = diode(ParseOptions {
        disable_self_heating: true,
        ..ParseOptions::default()
    });
    assert_eq!(shipped.rth, 50.0);
    assert!(iso.rth.is_infinite(), "RTH must not be applied");
    assert_eq!(iso.tamb, 320.0);
    // TAMB's static scaling is identical: the same IS and N*Vt at 320 K.
    assert_eq!(iso.is, shipped.is);
    assert_eq!(iso.n_vt, shipped.n_vt);
    assert!(iso.is > 2.52e-9 * 2.0, "IS at 320 K is scaled up from TNOM");
}
