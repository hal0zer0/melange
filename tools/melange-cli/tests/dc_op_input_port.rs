//! The `dc-op` verb counts the input port's conductance once.
//!
//! A divider with DC at its input node, driven through a 1 MΩ input port, is
//! compared with ngspice `.op` on the same circuit with the port as a 0 V
//! source behind 1 MΩ (`Vin src 0 0` + `Rsrc src in 1Meg`). The verb used to
//! stamp the port conductance into G and then again inside the DC solve,
//! which put `v(in)` at 1.9934 V — and its KCL residual, checked against the
//! doubled system, read 1e-19 A.

use std::process::Command;

const TAPPED_DIVIDER: &str = "tapped divider\n\
V1 vcc 0 12\n\
R1 vcc mid 10k\n\
R2 mid 0 10k\n\
C1 mid 0 1n\n\
Rin in mid 1Meg\n";

/// ngspice `.op`, node → volts.
const NGSPICE: &[(&str, f64)] = &[("in", 2.992519), ("mid", 5.985037)];

#[test]
fn dc_op_verb_counts_the_input_port_once() {
    let path = std::env::temp_dir().join(format!(
        "melange_dc_op_input_port_{}.cir",
        std::process::id()
    ));
    std::fs::write(&path, TAPPED_DIVIDER).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_melange"))
        .args([
            "dc-op",
            "--format",
            "json",
            "-i",
            "in",
            "--input-resistance",
            "1e6",
        ])
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
    let v: serde_json::Value = serde_json::from_str(json).expect("dc-op JSON");
    assert_eq!(v["converged"], true, "DC operating point did not converge");
    // The DC solve's node Gmin (1e-12 S) against the 1 MΩ port moves v(in) by
    // ~3 µV; 1e-5 V covers it and nothing else.
    for (node, volts) in NGSPICE {
        let got = v["nodes"][node]
            .as_f64()
            .unwrap_or_else(|| panic!("no node {node}"));
        assert!(
            (got - volts).abs() < 1e-5,
            "v({node}) = {got:.7} V, ngspice {volts:.7} V"
        );
    }
}
