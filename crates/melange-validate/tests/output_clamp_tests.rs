//! validate refuses a melange render whose generated output clamp engaged.
//!
//! The generated code clips its output (after DC blocking) to the output
//! limit. ngspice has no such clamp, so on clipped samples a comparison
//! measures the clamp, not the circuit. The melange-side runner fails the
//! render instead of returning samples to correlate.

use melange_solver::codegen::BjtFaMode;
use melange_validate::run_melange_solver_from_str;

/// A 1:1 divider with a small cap: out = in / 2.
const DIVIDER: &str = "divider\nR1 in out 1k\nR2 out 0 1k\nC1 out 0 1n\n";

fn render(peak: f64) -> Result<Vec<f64>, String> {
    let fs = 48000.0;
    let input: Vec<f64> = (0..4800)
        .map(|i| peak * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / fs).sin())
        .collect();
    run_melange_solver_from_str(
        DIVIDER,
        &input,
        fs,
        "out",
        "in",
        BjtFaMode::Auto,
        "auto",
        false,
        false,
        1,
        None,
    )
    .map_err(|e| e.to_string())
}

#[test]
fn a_render_past_the_output_clamp_is_refused() {
    // 40 V in -> 20 V at the output, past the 10 V output clamp.
    let err = render(40.0).expect_err("a clamped render must not validate");
    assert!(err.contains("output clamp"), "{err}");
    // 5 V in -> 2.5 V out: nothing clipped.
    let ok = render(5.0).expect("an unclamped render validates");
    assert!(ok.iter().any(|v| v.abs() > 2.0));
}

/// The input clamp (INPUT_LIMIT_V = 100 V) is refused the same way: ngspice
/// saw the unclamped input.
#[test]
fn a_render_with_a_clamped_input_is_refused() {
    let err = render(150.0).expect_err("a clamped input must not validate");
    assert!(err.contains("requested input"), "{err}");
}
