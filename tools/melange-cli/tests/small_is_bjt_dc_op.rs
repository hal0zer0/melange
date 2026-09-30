//! A small-IS BJT carries the current its law gives.
//!
//! Every BJT exponential used to clamp at `x = 40`, capping the junction's
//! current at `IS·e^40`: 23.5 mA at IS = 1e-19 and 68 µA at IS = 2.9e-22 (the
//! scale of a single-transistor Darlington equivalent). An emitter follower
//! that needs 30-40 mA then either failed its DC operating point or
//! "converged" with the transistor nearly off. The junction exponential now
//! continues the law (`melange_devices::safeguards::junction_exp`).
//!
//! References: ngspice-42 `.op` on the same deck plus `Rsrc in 0 1` (melange's
//! input-port impedance), `.options reltol=1e-6 abstol=1e-15 vntol=1e-9`.

use std::process::Command;

fn deck(is: &str) -> String {
    format!(
        "small-IS emitter follower\n\
         .model QS NPN(IS={is} BF=1000 VAF=100)\n\
         VCC vcc 0 DC 30\n\
         Rb1 vcc b 27k\n\
         Rb2 b 0 2.2k\n\
         Rin in b 10k\n\
         Q1 vcc b e QS\n\
         Re e 0 22\n"
    )
}

fn dc_op(is: &str) -> serde_json::Value {
    let path = std::env::temp_dir().join(format!(
        "melange_small_is_{}_{}.cir",
        is.replace('.', "p"),
        std::process::id()
    ));
    std::fs::write(&path, deck(is)).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_melange"))
        .args(["dc-op", "--format", "json"])
        .arg(&path)
        .output()
        .expect("run melange");
    let _ = std::fs::remove_file(&path);
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    serde_json::from_str(
        stdout
            .lines()
            .find(|l| l.trim_start().starts_with('{'))
            .unwrap(),
    )
    .unwrap()
}

#[test]
fn a_small_is_follower_matches_ngspice() {
    // (IS, ngspice v(b), ngspice v(e))
    for (is, vb, ve) in [
        ("1e-19", 1.830810, 0.7916737),
        ("2.9e-22", 1.839069, 0.6537525),
    ] {
        let j = dc_op(is);
        assert_eq!(j["converged"], true, "IS={is}");
        let (b, e) = (
            j["nodes"]["b"].as_f64().unwrap(),
            j["nodes"]["e"].as_f64().unwrap(),
        );
        assert!((b - vb).abs() < 1e-5, "IS={is}: v(b) {b} vs ngspice {vb}");
        assert!((e - ve).abs() < 1e-5, "IS={is}: v(e) {e} vs ngspice {ve}");
        // Past the old ceiling IS·e^40 (23.5 mA and 68 µA).
        let ceiling = is.parse::<f64>().unwrap() * 40f64.exp();
        assert!(
            e / 22.0 > ceiling,
            "IS={is}: Ie {} A, old ceiling {ceiling} A",
            e / 22.0
        );
    }
}
