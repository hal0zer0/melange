//! The rate sweep separates discretization from model error on a deck with
//! a known answer: an RC lowpass is modelled exactly, so under strict gates
//! it fails at 48 kHz on step size alone and must CONVERGE toward ngspice,
//! with an asymptotic (model) error near zero.

use melange_validate::rate_sweep::{rate_sweep, SweepVerdict};
use melange_validate::spice_runner::is_ngspice_available;
use melange_validate::{AnalyticStimulus, ComparisonConfig, ValidationOptions};

#[test]
#[ignore = "requires ngspice"]
fn an_exactly_modelled_deck_converges_to_no_model_error() {
    assert!(
        is_ngspice_available(),
        "ngspice not found: this test needs it (run without --include-ignored to skip)"
    );
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/rc_lowpass/circuit.cir");
    let sweep = rate_sweep(
        &path,
        AnalyticStimulus::Sine {
            amplitude: 0.1,
            frequency: 1000.0,
        },
        0.2,
        0.02,
        48000.0,
        "out",
        &ComparisonConfig::strict(),
        &ValidationOptions::default(),
    )
    .unwrap();
    for r in &sweep.rows {
        eprintln!(
            "{:.0} Hz {} {:.4} %",
            r.sample_rate,
            r.integrator,
            100.0 * r.error
        );
    }
    match sweep.verdict {
        SweepVerdict::Converges { model_error, .. } => {
            assert!(model_error < 1e-4, "model error {model_error:e}")
        }
        v => panic!("an RC lowpass must converge: {v:?}"),
    }
    assert!(sweep.rows[0].error > sweep.rows[2].error * 4.0);
}
