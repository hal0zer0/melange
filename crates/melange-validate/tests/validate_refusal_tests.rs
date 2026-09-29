//! Every render validate claims to refuse has a witness here.
//!
//! validate refuses a melange render when its output passed the generated
//! output clamp, when its input was clamped or NaN-scrubbed, and when any
//! sample was never solved: in each case a comparison against ngspice would
//! measure something other than the circuit. The counters those refusals
//! read were once never printed by validate's driver, so every refusal was
//! dead at once. Each test here fails if its refusal goes dead again.

mod support;
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

/// The unsolved-sample refusal. A full-LU diode clipper driven hard, built from
/// real generated code, validates as is; the same build with `MAX_ITER` forced
/// to 1 cannot converge and commits held samples, and validate must refuse it.
#[test]
fn a_render_with_unsolved_samples_is_refused() {
    use melange_validate::run_generated_solver;

    let spice =
        "hard clipper\nR1 in a 1k\nD1 a 0 DX\nD2 0 a DX\nC1 a 0 10n\nR2 a out 1k\nR3 out 0 100k\n\
                 .model DX D(IS=2.52n N=1.752)\n";
    let opts = melange_solver::build::BuildOptions {
        circuit_name: "hold_witness".to_string(),
        input_resistance: Some(1.0),
        solver: "nodal".to_string(),
        nodal_sub_path_override: melange_solver::codegen::NodalSubPathOverride::FullLu,
        ..support::options(48000.0, "in", &["out"])
    };
    let code = support::build(spice, &opts).generated.code;
    assert!(
        code.contains("pub diag_nr_hold_count"),
        "a full-LU build declares the hold"
    );
    let input: Vec<f64> = (0..4800)
        .map(|i| 20.0 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin())
        .collect();

    run_generated_solver(&code, &input, None).expect("the unmodified build validates");

    let max_iter_line = code
        .lines()
        .find(|l| l.starts_with("pub const MAX_ITER: usize ="))
        .expect("MAX_ITER constant");
    let starved = code.replace(max_iter_line, "pub const MAX_ITER: usize = 1;");
    let err = run_generated_solver(&starved, &input, None)
        .expect_err("held samples must not validate")
        .to_string();
    assert!(err.contains("never solved"), "{err}");
}

/// The same refusal on the DK path, which has no hold: an unsolved final
/// solve (the trapezoidal one, or the BE fallback when it ran) is committed and
/// counted in `diag_nr_unconverged_commit_count`. With `MAX_ITER` forced to 1
/// every solve ends unsolved, and validate must refuse the render.
#[test]
fn a_dk_render_with_unsolved_samples_is_refused() {
    use melange_validate::run_generated_solver;

    let spice =
        "hard clipper\nR1 in a 1k\nD1 a 0 DX\nD2 0 a DX\nC1 a 0 10n\nR2 a out 1k\nR3 out 0 100k\n\
                 .model DX D(IS=2.52n N=1.752)\n";
    let opts = melange_solver::build::BuildOptions {
        circuit_name: "dk_unsolved_witness".to_string(),
        input_resistance: Some(1.0),
        solver: "dk".to_string(),
        ..support::options(48000.0, "in", &["out"])
    };
    let code = support::build(spice, &opts).generated.code;
    assert!(
        code.contains("pub diag_nr_unconverged_commit_count")
            && !code.contains("pub diag_nr_hold_count"),
        "a DK build counts its unsolved commits and has no hold"
    );
    let input: Vec<f64> = (0..4800)
        .map(|i| 20.0 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin())
        .collect();
    run_generated_solver(&code, &input, None).expect("the unmodified build validates");

    let max_iter_line = code
        .lines()
        .find(|l| l.starts_with("pub const MAX_ITER: usize ="))
        .expect("MAX_ITER constant");
    let starved = code.replace(max_iter_line, "pub const MAX_ITER: usize = 1;");
    let err = run_generated_solver(&starved, &input, None)
        .expect_err("unsolved samples must not validate")
        .to_string();
    assert!(err.contains("never solved"), "{err}");
}
