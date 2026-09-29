//! The runtime BE-latch engages on a stiff alternating mode that trapezoidal
//! integration keeps ringing, and on nothing else.
//!
//! Entry tracks the lag-1 ratio of the mean-removed output over its window.
//! For one mode x = A*z^n that ratio is z; for a mixture it is the
//! power-weighted mean of the components' factors. It engages at
//! ratio <= -exp(-alpha): an alternating mode that outlives the estimator's
//! own window and dominates the output. The positive witness is the in-repo
//! saturating transformer with an open secondary: after a program stop its
//! leakage mode rings under trap for seconds. The negative witnesses are a
//! corpus mastering deck the previous detector latched on a 2-sample impulse
//! tail (its minimum ratio over a 60 s hostile program is -0.32 against a
//! -0.99 threshold; it lives outside this repository), and the small rings in
//! the regression suite that must not latch: the railing choke's 1 uV rest
//! ring and the open-secondary transformer's ring under 30 Hz program.

mod support;

const DECK: &str = include_str!("../../../tools/golden-harness/decks/sat-core-open.cir");

/// Play 1 s of 220 Hz + 1760 Hz at 5 V, stop, and return the sample (after
/// the stop) at which the latch engaged.
fn latch_after_stop(oversampling: usize) -> Option<usize> {
    let mut config = support::config_for_spice(DECK, 48000.0);
    config.oversampling_factor = oversampling;
    let code = support::generate_circuit_code_nodal(DECK, &config).0;
    assert!(
        code.contains("pub be_latched"),
        "a trapezoidal build carries the latch"
    );
    let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let n = 48000usize;
    for i in 0..n {
        let t = i as f64 / 48000.0;
        let u = 5.0 * (0.6 * (2.0 * std::f64::consts::PI * 220.0 * t).sin()
            + 0.4 * (2.0 * std::f64::consts::PI * 1760.0 * t).sin());
        let _ = process_sample(u, &mut s);
    }
    let before = s.be_latched;
    let mut at = -1i64;
    for i in 0..(n / 2) {
        let _ = process_sample(0.0, &mut s);
        if s.be_latched { at = i as i64; break; }
    }
    println!(\"before={} at={}\", before, at);
}";
    let out = support::compile_and_run(&code, main, &format!("latch_entry_{oversampling}"));
    let line = out
        .stdout
        .lines()
        .find(|l| l.starts_with("before="))
        .unwrap()
        .to_string();
    assert!(
        line.starts_with("before=false"),
        "must not latch while playing: {line}"
    );
    let at: i64 = line.rsplit('=').next().unwrap().trim().parse().unwrap();
    (at >= 0).then_some(at as usize)
}

#[test]
fn the_latch_engages_on_the_open_secondary_ring() {
    for os in [1, 4] {
        let at = latch_after_stop(os);
        eprintln!("{os}x: latch engaged {at:?} samples after the stop");
        let at = at.unwrap_or_else(|| panic!("{os}x: the leakage ring must engage the latch"));
        assert!(
            at < 4800,
            "{os}x: engaged {at} samples after the stop (> 100 ms)"
        );
    }
}

/// A capless output resting at ~2 V behind a DC bias. Poking its state leaves a
/// persistent trapezoidal alternation (the capless row holds KCL only on
/// average), of whatever size the poke was.
const BIASED_REST: &str =
    "biased rest alternation witness\nCin in 0 1u\nD1 in 0 DX\nRin in out 1k\n\
Vb vb 0 DC 10\nRb vb out 2k\nRload out 0 1k\n.model DX D(IS=2.52n N=1.752)\n";

const TOL_LINE: &str = "let be_tol = 1e-3 * state.be_x_mean.abs() + 1e-6;";

/// Settle at rest, poke the output node by `poke` volts, and report whether the
/// latch engaged over the next second.
fn latches_on_rest_alternation(code: &str, poke: f64, tag: &str) -> bool {
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for _ in 0..4800 {{ let _ = process_sample(0.0, &mut s); }}
    s.v_prev[OUTPUT_NODES[0]] += {poke:?};
    for _ in 0..48000 {{ let _ = process_sample(0.0, &mut s); if s.be_latched {{ break; }} }}
    println!(\"latched={{}}\", s.be_latched as u8);
}}"
    );
    support::compile_and_run(code, &main, tag)
        .parse_kv("latched")
        .unwrap()
        == 1.0
}

/// An alternation inside the solver's own node tolerance (1e-3*|v| + 1e-6 V,
/// the Newton node-step test) cannot be told apart from convergence noise and
/// must not engage the latch; the same deck with a larger alternation must.
#[test]
fn a_rest_alternation_inside_the_node_tolerance_does_not_latch() {
    let config = support::config_for_spice(BIASED_REST, 48000.0);
    let code = support::generate_circuit_code_nodal(BIASED_REST, &config).0;
    assert!(code.contains("pub be_latched") && code.contains(TOL_LINE));
    // Output at ~2 V: tolerance ~2 mV.
    assert!(
        !latches_on_rest_alternation(&code, 1e-4, "rest_100uV"),
        "100 uV at 2 V latched"
    );
    assert!(
        latches_on_rest_alternation(&code, 1e-2, "rest_10mV"),
        "10 mV at 2 V did not latch"
    );
    // A fixed 1 uV floor (the old power floor) would take the 100 uV alternation as evidence.
    let fixed = code.replace(TOL_LINE, "let be_tol = 1e-6;");
    assert!(
        latches_on_rest_alternation(&fixed, 1e-4, "rest_fixed_floor"),
        "the witness no longer distinguishes the tolerance floor from a fixed one"
    );
}
