//! `TAMB` is the device temperature, as SPICE's `.temp`.
//!
//! A card's parameters are SPICE's, extracted at TNOM = 27 °C; a device on a
//! card with `TAMB` sits at that temperature, with or without self-heating.
//! The operating points below are compared with ngspice-42 `.op` on the same
//! deck (the `.model` line without `TAMB`, and `.temp 60` / `.temp 27`),
//! recorded 2026-09-29. The decks have no input node, so melange's 1 Ω input
//! stamp does not load them and the two engines agree to about 1 µV.

use std::path::PathBuf;
use std::process::Command;

const DIODE: &str = "diode\nVcc vcc 0 DC 5\nR1 vcc a 4.7k\nD1 a 0 DX\n\
                     .model DX D(IS=2.52n N=1.752 XTI=3 EG=1.11{EXTRA})\n";

const BJT: &str = "bjt\nVcc vcc 0 DC 12\nR1 vcc base 100k\nR2 base 0 22k\nQ1 coll base emit QX\n\
                   Rc vcc coll 4.7k\nRe emit 0 1k\n\
                   .model QX NPN(IS=1e-14 BF=200 BR=3 XTI=3 XTB=1.5 EG=1.11 ISE=1e-13 NE=1.5 \
                   ISC=1e-14 NC=2{EXTRA})\n";

/// ngspice-42 `.op`, node → volts.
const DIODE_60C: &[(&str, f64)] = &[("a", 5.150217e-1)];
const DIODE_27C: &[(&str, f64)] = &[("a", 5.813740e-1)];
const BJT_60C: &[(&str, f64)] = &[("base", 2.014381), ("coll", 5.418591), ("emit", 1.408593)];
const BJT_27C: &[(&str, f64)] = &[("base", 1.997502), ("coll", 5.768462), ("emit", 1.335089)];

const TOL_V: f64 = 5e-6;

fn dc_op(deck: &str, extra: &str, tag: &str) -> serde_json::Value {
    let path: PathBuf = std::env::temp_dir().join(format!(
        "melange_device_temperature_{}_{tag}.cir",
        std::process::id()
    ));
    std::fs::write(&path, deck.replace("{EXTRA}", extra)).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_melange"))
        .args(["dc-op", "--format", "json"])
        .arg(&path)
        .output()
        .expect("run melange");
    let _ = std::fs::remove_file(&path);
    assert!(
        out.status.success(),
        "dc-op failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    let json = stdout
        .lines()
        .find(|l| l.trim_start().starts_with('{'))
        .expect("a JSON line on stdout");
    serde_json::from_str(json).expect("dc-op JSON")
}

fn assert_nodes(v: &serde_json::Value, want: &[(&str, f64)], what: &str) {
    assert_eq!(
        v["converged"], true,
        "{what}: DC operating point did not converge"
    );
    for (node, volts) in want {
        let got = v["nodes"][node]
            .as_f64()
            .unwrap_or_else(|| panic!("{what}: no node {node}"));
        assert!(
            (got - volts).abs() < TOL_V,
            "{what}: v({node}) = {got:.7} V, ngspice {volts:.7} V"
        );
    }
}

#[test]
fn tamb_is_the_device_temperature() {
    for (deck, at_60, at_27, name) in [
        (DIODE, DIODE_60C, DIODE_27C, "diode"),
        (BJT, BJT_60C, BJT_27C, "BJT"),
    ] {
        assert_nodes(
            &dc_op(deck, "", &format!("{name}_27")),
            at_27,
            &format!("{name}, no TAMB"),
        );
        assert_nodes(
            &dc_op(deck, " TAMB=333.15", &format!("{name}_60")),
            at_60,
            &format!("{name}, TAMB=333.15"),
        );
        // Self-heating starts from the same device temperature: the DC
        // operating point is solved with the junction at TAMB.
        assert_nodes(
            &dc_op(deck, " TAMB=333.15 RTH=100", &format!("{name}_60_rth")),
            at_60,
            &format!("{name}, TAMB=333.15 with RTH"),
        );
    }
}
