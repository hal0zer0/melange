//! A `.linearize`d device is simulated as its small-signal model, and the
//! validate reference runs the same model (`linearize_twin`), so validate
//! measures the solver against the circuit melange built. Against the full
//! device instead, a linearized common-emitter stage at 0.3 V failed on the
//! real transistor's distortion (THD -59.9 dB against melange's -189 dB,
//! 0.107 % RMS); with the twin it matches to 0.029 %, the step-size residue
//! of a linear circuit.

use melange_validate::spice_runner::is_ngspice_available;
use melange_validate::{validate_circuit_with_options, ComparisonConfig, ValidationOptions};

/// Forward active at rest (Vc about 5 V) and at the drive used.
const CE: &str = "linearized common emitter
VCC vcc 0 DC 12
Cin in b 10u
R1 vcc b 100k
R2 b 0 22k
Q1 c b e QX
RC vcc c 4.7k
RE e 0 1k
Cout c out 10u
Rl out 0 100k
.model QX NPN(IS=1e-14 BF=200)
.linearize Q1
.end
";

#[test]
fn a_linearized_stage_validates_against_the_same_model() {
    if !is_ngspice_available() {
        eprintln!("ngspice not available; skipping");
        return;
    }
    const FS: f64 = 48000.0;
    let dir = std::env::temp_dir().join(format!("melange_linearize_twin_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("ce.cir");
    std::fs::write(&path, CE).unwrap();
    let input: Vec<f64> = (0..(0.2 * FS) as usize)
        .map(|k| 0.3 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / FS).sin())
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
    assert!(
        report
            .linearize_note
            .as_deref()
            .unwrap_or("")
            .contains("Q1"),
        "the report must say the reference runs Q1's small-signal model"
    );
    assert!(
        report.normalized_rms_error < 5e-4,
        "normalized rms error {:.3e}:\n{}",
        report.normalized_rms_error,
        report.summary()
    );
}

/// A linearized cathode follower at ~25 uA idle, driven 10 V: cut off on
/// the negative swings. validate refuses it, naming the reduced model, not a
/// Newton failure (the Newton solve succeeded on those samples).
#[test]
fn a_linearized_stage_out_of_its_region_is_refused_as_such() {
    if !is_ngspice_available() {
        eprintln!("ngspice not available; skipping");
        return;
    }
    const FS: f64 = 48000.0;
    let deck = "linearized cathode follower
VCC vcc 0 DC 250
Cin in g 1u
Rg g 0 1Meg
T1 g vcc k TX
Rk k 0 100k
Cout k out 1u
Rl out 0 1Meg
.model TX TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
.linearize T1
.end
";
    let dir = std::env::temp_dir().join(format!("melange_linearize_exit_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("cf.cir");
    std::fs::write(&path, deck).unwrap();
    let input: Vec<f64> = (0..(0.1 * FS) as usize)
        .map(|k| 10.0 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / FS).sin())
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
    );
    let _ = std::fs::remove_dir_all(&dir);
    let err = match result {
        Ok(_) => panic!("a follower cut off on every negative swing must be refused"),
        Err(e) => e.to_string(),
    };
    assert!(
        err.contains("REDUCED device model outside its region") && err.contains(".linearize"),
        "{err}"
    );
}
