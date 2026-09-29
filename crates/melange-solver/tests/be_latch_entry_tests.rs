//! The runtime BE-latch engages on a stiff alternating mode that trapezoidal
//! integration keeps ringing, when that ring is at least −60 dB of the program
//! that excited it — the compile-time ring predicate's own threshold
//! (`docs/aidocs/RING_PREDICATE.md`), so the two agree on what a ring worth
//! backward Euler is.
//!
//! Entry tracks the lag-1 ratio of the mean-removed output over its window.
//! For one mode x = A*z^n that ratio is z; for a mixture it is the
//! power-weighted mean of the components' factors. It engages at
//! ratio <= -exp(-alpha) (an alternating mode that outlives the window and
//! dominates the output) AND amplitude >= the larger of the node tolerance and
//! 1e-3 x the program reference. The reference is passband gain x input
//! amplitude, decaying no faster than the slowest ring the circuit can carry
//! at the running rate, so a ring cannot "dominate" merely because the
//! program stopped.
//!
//! Witness: a stiff node, R_s into C_p (tau = R*C in ns) at the output; trap
//! maps its pole to z ≈ −1 + 4·fs·tau. Its input residue relative to the
//! passband is 2·alpha·tau: −74 dB at 1 pF, −62.4 dB at 4 pF (48 kHz), so both
//! builds stay trapezoidal. A diode on a side branch makes the build nonlinear
//! (the latch is emitted on nonlinear builds) without touching the ring node.

mod support;

const SAT_CORE_LOADED: &str =
    include_str!("../../../tools/golden-harness/decks/sat-core-loaded.cir");

fn stiff(cp: &str) -> String {
    format!(
        "stiff node witness\nR_s in out 1k\nC_p out 0 {cp}\nR_l out 0 100k\nR_x in x 1Meg\n\
         D_x x 0 DX\n.model DX D(IS=2.52n N=1.752)\n"
    )
}

const FLOOR_LINE: &str = "let be_floor = f64::max(be_tol, BE_LATCH_RING_REL * state.be_ref);";
const DECAY_LINE: &str =
    "self.be_ref_decay = be_latch_ref_decay(sample_rate * OVERSAMPLING_FACTOR as f64);";

fn code_for(deck: &str, oversampling: usize) -> String {
    let mut config = support::config_for_spice(deck, 48000.0);
    config.oversampling_factor = oversampling;
    let code = support::generate_circuit_code_nodal(deck, &config).0;
    assert!(
        code.contains("pub be_latched") && code.contains(FLOOR_LINE),
        "a trapezoidal build carries the latch and its program floor"
    );
    code
}

/// Play 1 s of 220 Hz + 1760 Hz at `amp`, stop, and return the sample (after
/// the stop) at which the latch engaged.
fn latch_after_stop(code: &str, amp: f64, tag: &str) -> Option<usize> {
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
    let out = support::compile_and_run(code, &main, tag);
    let line = out
        .stdout
        .lines()
        .find(|l| l.starts_with("before="))
        .unwrap()
        .to_string();
    assert!(
        line.starts_with("before=false"),
        "{tag}: must not latch while playing: {line}"
    );
    let at: i64 = line.rsplit('=').next().unwrap().trim().parse().unwrap();
    (at >= 0).then_some(at as usize)
}

/// Settle, play 0.2 s of a 1 V sine, then 0.1 s of silence, then single clicks
/// of 1 V every `period` samples (`0` = one click), all at host rate `fs`;
/// report the sample after the first click at which the latch engaged.
fn latch_after_clicks(code: &str, fs: f64, period: usize, tag: &str) -> Option<usize> {
    let main = format!(
        "fn main() {{
    let fs: f64 = {fs:?};
    let mut s = CircuitState::default();
    s.set_sample_rate(fs);
    for _ in 0..(0.1 * fs) as usize {{ let _ = process_sample(0.0, &mut s); }}
    for i in 0..(0.2 * fs) as usize {{
        let u = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / fs).sin();
        let _ = process_sample(u, &mut s);
    }}
    for _ in 0..(0.1 * fs) as usize {{ let _ = process_sample(0.0, &mut s); }}
    let period = {period}usize;
    let mut at = -1i64;
    for i in 0..(0.5 * fs) as usize {{
        let click = if (period == 0 && i == 0) || (period > 0 && i % period == 0) {{ 1.0 }} else {{ 0.0 }};
        let _ = process_sample(click, &mut s);
        if s.be_latched && at < 0 {{ at = i as i64; }}
    }}
    println!(\"at={{}}\", at);
}}"
    );
    let at = support::compile_and_run(code, &main, tag)
        .parse_kv("at")
        .unwrap();
    (at >= 0.0).then_some(at as usize)
}

/// A stiff ring below −60 dB of the program is one the ring predicate left on
/// trapezoidal: it must not engage the latch, after a stop or after a click.
/// The instantaneous floor (the node tolerance alone) took it as evidence the
/// moment the program went quiet.
#[test]
fn a_ring_below_the_threshold_does_not_latch_after_the_program_stops() {
    for (cp, tag) in [("1p", "stiff_1p"), ("4p", "stiff_4p")] {
        let code = code_for(&stiff(cp), 1);
        assert_eq!(
            latch_after_stop(&code, 0.3, &format!("{tag}_stop")),
            None,
            "{tag}: stop"
        );
        assert_eq!(
            latch_after_clicks(&code, 48000.0, 0, &format!("{tag}_click")),
            None,
            "{tag}: click"
        );
        let old = code.replace(FLOOR_LINE, "let be_floor = be_tol;");
        let at = latch_after_stop(&old, 0.3, &format!("{tag}_stop_old"));
        eprintln!("{tag}: instantaneous-floor mutant engaged {at:?} samples after the stop");
        assert!(
            at.is_some(),
            "{tag}: the witness no longer tells the floors apart"
        );
    }
}

/// The reference's memory is remapped at the running rate: at 44.1 kHz a
/// build compiled at 48 kHz rings more slowly (trap's stiff-mode decay scales
/// as fs^2), and a memory frozen at 48 kHz would be outlived by the ring.
#[test]
fn a_ring_below_the_threshold_does_not_latch_at_a_lower_host_rate() {
    let code = code_for(&stiff("4p"), 1);
    assert_eq!(
        latch_after_clicks(&code, 44100.0, 0, "stiff_4p_44k"),
        None,
        "44.1 kHz: a -63 dB ring latched"
    );
    let frozen = code.replace(DECAY_LINE, "");
    let at = latch_after_clicks(&frozen, 44100.0, 0, "stiff_4p_44k_frozen");
    eprintln!("44.1 kHz, memory frozen at 48 kHz: engaged {at:?} samples after the click");
    assert!(
        at.is_some(),
        "the witness no longer tells a frozen memory apart"
    );
}

/// Phase-coherent even-period clicks add each click's ring in phase. At a
/// 100 Hz click rate (480 samples) the -62.4 dB single-event ring accumulates
/// toward 1/(1-|z|^480) ≈ +10 dB and crosses -60 dB of the program at the
/// second click, where the latch engages — by the same rule that leaves the
/// single event alone. (Between
/// clicks the output is the ring alone, as after each impulse of a sparse
/// click track; a denser train keeps the clicks themselves in the latch's
/// window, and they, not the ring, dominate it.)
#[test]
fn coherent_even_period_clicks_accumulate_and_latch() {
    let code = code_for(&stiff("4p"), 1);
    let at = latch_after_clicks(&code, 48000.0, 480, "stiff_4p_train");
    eprintln!("100 Hz click train: engaged {at:?} samples after the first click");
    assert!(
        at.is_some(),
        "the accumulated ring did not engage the latch"
    );
}

/// A ring injected at -40 dB of the program reference (a capacitor-current
/// kick after a loud program and a stop) engages the latch; the same kick at
/// -80 dB does not.
#[test]
fn a_ring_40_db_under_the_program_latches() {
    let code = code_for(&stiff("1p"), 1);
    let run = |rel: f64, tag: &str| -> bool {
        // Node conductance 1/1k + 1/100k: a kick of `a * g` A rings the node by `a` V.
        let main = format!(
            "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for i in 0..9600usize {{
        let _ = process_sample((2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin(), &mut s);
    }}
    for _ in 0..480usize {{ let _ = process_sample(0.0, &mut s); }}
    let latched_before = s.be_latched;
    let ring = {rel:?} * s.be_ref;
    s.q_dot[OUTPUT_NODES[0]] += ring * (1.0 / 1000.0 + 1.0 / 100000.0);
    for _ in 0..4800usize {{ let _ = process_sample(0.0, &mut s); if s.be_latched {{ break; }} }}
    println!(\"latched={{}}\", (s.be_latched && !latched_before) as u8);
}}"
        );
        support::compile_and_run(&code, &main, tag)
            .parse_kv("latched")
            .unwrap()
            == 1.0
    };
    assert!(run(1e-2, "kick_minus40"), "a -40 dB ring did not latch");
    assert!(!run(1e-4, "kick_minus80"), "a -80 dB ring latched");
}

/// The loaded secondary has no lasting Nyquist-side mode; nothing rings after
/// a stop, at 1x or 4x.
#[test]
fn the_loaded_secondary_does_not_ring_after_a_stop() {
    for os in [1, 4] {
        let code = code_for(SAT_CORE_LOADED, os);
        let at = latch_after_stop(&code, 5.0, &format!("latch_sat_core_loaded_{os}"));
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
