//! The runtime BE-latch engages on a stiff alternating mode that trapezoidal
//! integration keeps ringing, and on nothing else.
//!
//! Entry tracks the lag-1 ratio of the mean-removed output over its window.
//! For one mode x = A*z^n that ratio is z; for a mixture it is the
//! power-weighted mean of the components' factors. It engages at
//! ratio <= -exp(-alpha): an alternating mode that outlives the estimator's
//! own window and dominates the output.
//!
//! The positive witness is a stiff node (1 kOhm into 10 pF, tau = 10 ns):
//! trapezoidal integration is A-stable but not L-stable, so after a program
//! stop the node rings at z = -0.998 per sample at 48 kHz. The negative
//! witnesses: the in-repo saturating transformer with an open secondary, whose
//! ring after a stop was the whole-system trapezoidal form's z = -1 memory on
//! the algebraic rows and is gone under the charge form, and a corpus
//! mastering deck the previous detector latched on a 2-sample impulse tail
//! (its minimum ratio over a 60 s hostile program is -0.32 against a -0.99
//! threshold; it lives outside this repository).

mod support;

const SAT_CORE_OPEN: &str = include_str!("../../../tools/golden-harness/decks/sat-core-open.cir");

const STIFF: &str = "stiff node witness\nR_s in out 1k\nC_p out 0 10p\nD_1 out 0 DX\n\
D_2 0 out DX\nR_l out 0 100k\n.model DX D(IS=2.52n N=1.752)\n";

/// Play 1 s of 220 Hz + 1760 Hz at `amp`, stop, and return the sample (after
/// the stop) at which the latch engaged.
fn latch_after_stop(deck: &str, amp: f64, oversampling: usize, tag: &str) -> Option<usize> {
    let mut config = support::config_for_spice(deck, 48000.0);
    config.oversampling_factor = oversampling;
    let code = support::generate_circuit_code_nodal(deck, &config).0;
    assert!(
        code.contains("pub be_latched"),
        "a trapezoidal build carries the latch"
    );
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let n = 48000usize;
    for i in 0..n {{
        let t = i as f64 / 48000.0;
        let u = {amp:?} * (0.6 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.4 * (2.0 * std::f64::consts::PI * 1760.0 * t).sin());
        let _ = process_sample(u, &mut s);
    }}
    let before = s.be_latched;
    let mut at = -1i64;
    for i in 0..(n / 2) {{
        let _ = process_sample(0.0, &mut s);
        if s.be_latched {{ at = i as i64; break; }}
    }}
    println!(\"before={{}} at={{}}\", before, at);
}}"
    );
    let out = support::compile_and_run(&code, &main, &format!("{tag}_{oversampling}"));
    let line = out
        .stdout
        .lines()
        .find(|l| l.starts_with("before="))
        .unwrap()
        .to_string();
    assert!(
        line.starts_with("before=false"),
        "{tag} {oversampling}x: must not latch while playing: {line}"
    );
    let at: i64 = line.rsplit('=').next().unwrap().trim().parse().unwrap();
    (at >= 0).then_some(at as usize)
}

#[test]
fn the_latch_engages_on_a_stiff_trapezoidal_ring() {
    let at = latch_after_stop(STIFF, 0.3, 1, "latch_stiff");
    eprintln!("stiff node: latch engaged {at:?} samples after the stop");
    let at = at.expect("the stiff node's ring must engage the latch");
    assert!(at < 4800, "engaged {at} samples after the stop (> 100 ms)");
}

/// Under the whole-system trapezoidal form this deck's leakage mode rang for
/// seconds after a stop and engaged the latch (it was the witness here). That
/// ring was the accepted-residual walk on the algebraic rows, not a physical
/// or integrator mode, and the charge form removes it.
#[test]
fn the_open_secondary_no_longer_rings_after_a_stop() {
    for os in [1, 4] {
        let at = latch_after_stop(SAT_CORE_OPEN, 5.0, os, "latch_sat_core_open");
        assert_eq!(
            at, None,
            "{os}x: latch engaged {at:?} samples after the stop"
        );
    }
}

/// An output resting at ~2 V behind a DC bias, with a stiff 10 pF node
/// (tau = 4 ns). A kick of the capacitor current `q_dot` rings that mode, of
/// whatever size the kick was.
const BIASED_REST: &str =
    "biased rest alternation witness\nCin in 0 1u\nD1 in 0 DX\nRin in out 1k\n\
Vb vb 0 DC 10\nRb vb out 2k\nRload out 0 1k\nCp out 0 10p\n.model DX D(IS=2.52n N=1.752)\n";

const TOL_LINE: &str = "let be_tol = 1e-3 * state.be_x_mean.abs() + 1e-6;";

/// Settle at rest, kick the output node's capacitor current so the stiff mode
/// rings with amplitude `ring` volts, and report whether the latch engaged over
/// the next second. The node's total conductance is 2.5 mS, so a kick of
/// `ring * 2.5e-3` A rings it by `ring` V.
fn latches_on_rest_ring(code: &str, ring: f64, tag: &str) -> bool {
    let kick = ring * 2.5e-3;
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for _ in 0..4800 {{ let _ = process_sample(0.0, &mut s); }}
    s.q_dot[OUTPUT_NODES[0]] += {kick:?};
    for _ in 0..48000 {{ let _ = process_sample(0.0, &mut s); if s.be_latched {{ break; }} }}
    println!(\"latched={{}}\", s.be_latched as u8);
}}"
    );
    support::compile_and_run(code, &main, tag)
        .parse_kv("latched")
        .unwrap()
        == 1.0
}

/// A ring inside the solver's own node tolerance (1e-3*|v| + 1e-6 V, the
/// Newton node-step test) cannot be told apart from convergence noise and
/// must not engage the latch; the same deck with a larger ring must. Both at
/// 1x and 4x oversampling (the window is in internal samples).
#[test]
fn a_rest_alternation_inside_the_node_tolerance_does_not_latch() {
    for os in [1, 4] {
        let mut config = support::config_for_spice(BIASED_REST, 48000.0);
        config.oversampling_factor = os;
        let code = support::generate_circuit_code_nodal(BIASED_REST, &config).0;
        assert!(code.contains("pub be_latched") && code.contains(TOL_LINE));
        // Output at ~2 V: tolerance ~2 mV.
        assert!(
            !latches_on_rest_ring(&code, 1e-4, &format!("rest_100uV_{os}")),
            "{os}x: 100 uV at 2 V latched"
        );
        assert!(
            latches_on_rest_ring(&code, 1e-2, &format!("rest_10mV_{os}")),
            "{os}x: 10 mV at 2 V did not latch"
        );
        if os == 1 {
            // A fixed 1 uV floor (the old power floor) would take the 100 uV ring as evidence.
            let fixed = code.replace(TOL_LINE, "let be_tol = 1e-6;");
            assert!(
                latches_on_rest_ring(&fixed, 1e-4, "rest_fixed_floor"),
                "the witness no longer distinguishes the tolerance floor from a fixed one"
            );
        }
    }
}
