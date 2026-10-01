//! A self-heating deck validates isothermal: melange is built without its
//! devices' `RTH` (ngspice's diode and BJT have no thermal model; the
//! reference used to ignore the keys with a warning), and the reference puts
//! each device at its card's `TAMB`. The result line says so.

use melange_validate::spice_runner::is_ngspice_available;
use melange_validate::{validate_circuit_with_options, ComparisonConfig, ValidationOptions};

/// A diode clipper at 320 K with a fast, strong thermal loop (tau = 2 ms,
/// several kelvin of rise), so both halves show: with melange self-heating
/// the run reads 1.5 % NRMSE, and without the reference's instance
/// temperature (its IS at 300.15 K instead of 320 K) 6.5 %.
const CLIPPER: &str = "self-heating clipper
R1 in a 1k
D1 a 0 DX
D2 0 a DX
C1 a 0 10n
R2 a out 10k
Rl out 0 100k
.model DX D(IS=2.52e-9 N=1.752 RS=0.6 RTH=20000 CTH=1e-7 TAMB=320)
.end
";

#[test]
#[ignore = "requires ngspice"]
fn a_self_heating_deck_validates_isothermal_at_its_tamb() {
    assert!(
        is_ngspice_available(),
        "ngspice not found: this test needs it (run without --include-ignored to skip)"
    );
    const FS: f64 = 48000.0;
    let dir = std::env::temp_dir().join(format!("melange_thermal_twin_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("clip.cir");
    std::fs::write(&path, CLIPPER).unwrap();
    let input: Vec<f64> = (0..(0.1 * FS) as usize)
        .map(|k| 2.0 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / FS).sin())
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
    let q = report.status_qualifier();
    assert!(
        q.contains("self-heating disabled for comparison (DX RTH=20000)") && q.contains("TAMB"),
        "{q}"
    );
    assert!(
        report.normalized_rms_error < 5e-3,
        "normalized rms error {:.3e}:\n{}",
        report.normalized_rms_error,
        report.summary()
    );
}
