//! A behavioral current source's current is not a Newton unknown: it enters the
//! per-iteration solve as a companion in the working right-hand side and a
//! Jacobian stamp in the chord factor. The KCL residual helpers that decide
//! convergence (and steer the Armijo line search) read the base `A` and `rhs`,
//! so they must evaluate that current themselves. Without it the residual at
//! the true solution is off by `f(v)` on the terminal rows and an `M > 0` build
//! (one `I={}` source beside one diode) held 2300 of 2400 samples at every
//! drive, while the same deck without the diode (`M = 0`, no residual gate)
//! rendered cleanly.

mod support;

/// One `I={}` source into a 1 kΩ ‖ 1 µF load, with a diode beside it so the
/// build has nonlinear devices (`M > 0`) and emits the residual gate.
const BSRC_CURRENT_WITH_DIODE: &str = "Behavioral I source with a diode (M>0)
.model DS D(IS=1e-12 N=1.2)
Rin in a 1k
B1 out 0 I={ -1e-3*tanh(V(a)) }
Rout out 0 1k
D1 out k DS
Rk k 0 10k
Cout out 0 1u
.end
";

/// The same deck without the diode and its return resistor (`M = 0`): the
/// control render. In the first deck the diode sits reverse-biased or far below
/// its knee at these levels, so the two decks are the same circuit to well under
/// a nanovolt.
const BSRC_CURRENT_CONTROL: &str = "Behavioral I source, no diode (M=0 control)
Rin in a 1k
B1 out 0 I={ -1e-3*tanh(V(a)) }
Rout out 0 1k
Cout out 0 1u
.end
";

fn build(spice: &str) -> melange_solver::build::Built {
    let config = support::config_in_out_or_node1(spice, 48000.0);
    support::try_build_shipped_with(spice, &config, "auto", |_| {})
        .unwrap_or_else(|e| panic!("build refused: {e}"))
}

#[test]
fn both_kcl_residual_helpers_evaluate_the_behavioral_current() {
    let code = build(BSRC_CURRENT_WITH_DIODE).generated.code;
    assert!(
        code.contains("fn kcl_residual_inl(") && code.contains("fn kcl_residual("),
        "residual helpers are emitted for an M > 0 build"
    );
    // The term appears once per helper, folded into `q` at the source's
    // terminal rows (n− is ground here, so only n+ is stamped).
    assert_eq!(
        code.matches("let bsrc_0_f_resid = ").count(),
        2,
        "the I={{}} current is evaluated in both residual helpers"
    );
    assert_eq!(code.matches("-= bsrc_0_f_resid;").count(), 2);
    // The reuse helper reads `state` for the expression, so it takes it.
    assert!(
        code.contains("i_nl: &[f64; M], state: &CircuitState) -> (f64, bool, f64)"),
        "kcl_residual_inl takes state when a behavioral current source exists"
    );
}

#[test]
fn a_current_source_beside_a_diode_renders_as_its_diodeless_control() {
    let with_diode = build(BSRC_CURRENT_WITH_DIODE).generated.code;
    let control = build(BSRC_CURRENT_CONTROL).generated.code;
    // 0.5 V at 1 kHz for 0.1 s; print the unsolved count and the last sample.
    let main = "
fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let mut last = 0.0f64;
    for k in 0..4800usize {
        let x = 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / 48000.0).sin();
        last = process_sample(x, &mut s)[0];
    }
    println!(\"{} {:.12e}\", s.diag_unsolved_sample_count, last);
}
";
    let a = support::compile_and_run(&with_diode, main, "bsrc_i_with_diode");
    let b = support::compile_and_run(&control, main, "bsrc_i_control");
    let parse = |o: &support::RunOutput| -> (u64, f64) {
        let mut it = o.stdout.split_whitespace();
        (
            it.next().unwrap().parse().unwrap(),
            it.next().unwrap().parse().unwrap(),
        )
    };
    let (unsolved_a, last_a) = parse(&a);
    let (unsolved_b, last_b) = parse(&b);
    assert_eq!(unsolved_b, 0, "control:\n{}", b.stderr);
    assert_eq!(
        unsolved_a, 0,
        "the M > 0 build held samples (was 2300 of 2400):\n{}",
        a.stderr
    );
    assert!(
        (last_a - last_b).abs() < 1e-9,
        "last sample {last_a:e} vs control {last_b:e}"
    );
}
