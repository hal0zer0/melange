//! A capacitor-free nonlinear deck is simulated with 10 pF parasitic
//! capacitors across its device junctions (without capacitance the solver
//! has no state). They are part of the circuit melange builds, so the
//! validate reference carries the same ones: without them the two engines
//! simulate different circuits and the difference is charged to the solver.
//!
//! Measured on a JFET variable resistor with its gate held through 1 MOhm,
//! where the parasitic gate caps put a pole near 16 kHz on the gate: against
//! a reference without them the normalized rms error is 1.9e-2, with them
//! 3.9e-4 (the remainder is melange's 48 kHz step on that pole).

use melange_solver::mna::ParasiticCap;
use melange_validate::spice_runner::is_ngspice_available;
use melange_validate::{
    validate_circuit_with_options, with_parasitic_caps, ComparisonConfig, ValidationOptions,
};

fn cap(device: &str, a: &str, b: &str) -> ParasiticCap {
    ParasiticCap {
        device: device.to_string(),
        node_a: a.to_string(),
        node_b: b.to_string(),
    }
}

#[test]
fn the_reference_deck_gets_the_caps_before_end() {
    let deck = "t\nR1 in out 1k\nD1 out 0 DX\n.model DX D\n.END\n";
    let twin = with_parasitic_caps(deck, &[cap("D1", "out", "0")]).unwrap();
    assert_eq!(
        twin,
        "t\nR1 in out 1k\nD1 out 0 DX\n.model DX D\nC_melange_parasitic_1 out 0 1e-11\n.END\n"
    );
    // No `.end`: appended.
    let twin = with_parasitic_caps("t\nD1 a b DX\n", &[cap("D1", "a", "b")]).unwrap();
    assert!(
        twin.ends_with("C_melange_parasitic_1 a b 1e-11\n"),
        "{twin}"
    );
    // No caps: the deck as written.
    assert_eq!(with_parasitic_caps(deck, &[]).unwrap(), deck);
    // A cap at a node with no name cannot be placed: refused, not dropped.
    assert!(with_parasitic_caps(deck, &[cap("D1", "", "0")]).is_err());
}

const VVR: &str = "jfet vvr, no capacitors\nR1 in out 1k\nJ1 out g 0 JX\nRG g 0 1Meg\n\
                   .model JX NJF(VTO=-2 BETA=1e-3)\n.end\n";

#[test]
#[ignore = "requires ngspice"]
fn a_capacitor_free_deck_validates_against_the_same_circuit() {
    assert!(
        is_ngspice_available(),
        "ngspice not found: this test needs it (run without --include-ignored to skip)"
    );
    const FS: f64 = 48000.0;
    let dir = std::env::temp_dir().join(format!("melange_parasitic_twin_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("vvr.cir");
    std::fs::write(&path, VVR).unwrap();
    // 5 V at 1 kHz: the drain swings below the gate, the gate-drain junction
    // conducts, and the gate's 1 MOhm node carries signal through the caps.
    let input: Vec<f64> = (0..(0.1 * FS) as usize)
        .map(|k| 5.0 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / FS).sin())
        .collect();
    let options = ValidationOptions {
        generate_html_on_failure: false,
        output_dir: Some(dir.clone()),
        ..ValidationOptions::default()
    };
    let result = validate_circuit_with_options(
        &path,
        &input,
        FS,
        "out",
        &ComparisonConfig::strict(),
        &options,
    )
    .unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    let report = &result.report;
    let note = report.parasitic_note.as_deref().unwrap_or("");
    assert!(
        note.contains("J1 g-0") && note.contains("J1 g-out"),
        "the report must name the caps both engines carry: {note:?}"
    );
    eprintln!(
        "normalized rms error {:.3e}, correlation {:.9}",
        report.normalized_rms_error, report.correlation_coefficient
    );
    assert!(
        report.normalized_rms_error < 1e-3,
        "normalized rms error {:.3e}:\n{}",
        report.normalized_rms_error,
        report.summary()
    );
}
