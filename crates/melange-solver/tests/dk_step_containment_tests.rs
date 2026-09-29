//! A converged DK sample is the circuit's answer: nothing edits it.
//!
//! The DK path contains an UNSOLVED sample (its final Newton solve ended at
//! MAX_ITER) by scaling a step larger than max(2 V, 5 % of max |DC_OP|) back
//! to that bound, and counts it. It used to apply the same scaling to every
//! sample, solved or not, so a legitimately large step — a hard-driven input
//! node, a high-gain stage at high frequency — was pulled back toward the
//! previous sample, silently. On a corpus amplifier that made the golden
//! sweep 78 % wrong against an 8x render.
//!
//! The BE fallback likewise runs only for an unsolved trapezoidal sample. It
//! used to run as well whenever a converged node exceeded 3 x max |DC_OP| +
//! 10 V ("ringing"), but signals legitimately swing past their bias point.
//!
//! Witness: a diode clipper driven at 20 V, 1 kHz; its input node moves
//! 2.6 V per sample at 48 kHz and reaches 20 V (past the old 13 V "ringing"
//! bound), every sample converged. The shipped build never damps, never falls
//! back, and tracks an 8x render of itself; a mutant that damps converged
//! samples again is caught by both the counter and the error.

mod support;

const CLIPPER: &str = "hard clipper\nR1 in a 1k\nD1 a 0 DX\nD2 0 a DX\nC1 a 0 10n\nR2 a out 1k\n\
                       R3 out 0 100k\n.model DX D(IS=2.52n N=1.752)\n";

const GATE: &str = "if unsolved && max_delta > damp_thresh {";

/// 0.1 s of the drive at `fs`, output at the 48 kHz instants, plus the damp,
/// unsolved and BE-fallback counters.
fn render(code: &str, fs: f64, tag: &str) -> (Vec<f64>, f64, f64, f64) {
    let main = format!(
        "fn main() {{
    let fs: f64 = {fs:?};
    let k = (fs / 48000.0).round() as usize;
    let mut s = CircuitState::default();
    s.set_sample_rate(fs);
    for i in 0..(0.1 * fs) as usize {{
        let u = 20.0 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / fs).sin();
        let y = process_sample(u, &mut s)[0];
        if i % k == 0 {{ println!(\"{{:.17e}}\", y); }}
    }}
    eprintln!(\"damp={{}}\", s.diag_voltage_damp_count);
    eprintln!(\"unsolved={{}}\", s.diag_nr_unconverged_commit_count);
    eprintln!(\"fallback={{}}\", s.diag_be_fallback_count);
}}"
    );
    let out = support::compile_and_run(code, &main, tag);
    let y = out.parse_samples();
    let damp = out.parse_kv("damp").unwrap();
    let unsolved = out.parse_kv("unsolved").unwrap();
    let fallback = out.parse_kv("fallback").unwrap();
    (y, damp, unsolved, fallback)
}

fn code_at(fs: f64) -> String {
    let mut config = support::config_for_spice(CLIPPER, fs);
    config.dc_block = false;
    let code = support::generate_circuit_code(CLIPPER, &config).0;
    assert!(
        code.contains(GATE),
        "the DK build contains only unsolved samples"
    );
    code
}

fn rms_err(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    let tail = n / 2..n;
    (tail.clone().map(|i| (a[i] - b[i]).powi(2)).sum::<f64>() / tail.len() as f64).sqrt()
}

#[test]
fn a_converged_dk_sample_is_never_damped() {
    let (reference, ref_damp, _, _) = render(&code_at(384000.0), 384000.0, "dk_contain_ref");
    assert_eq!(ref_damp, 0.0);
    let code = code_at(48000.0);
    let (y, damp, unsolved, fallback) = render(&code, 48000.0, "dk_contain_48k");
    assert_eq!(unsolved, 0.0, "every sample converges");
    assert_eq!(
        fallback, 0.0,
        "a converged sample was sent to the BE fallback"
    );
    assert_eq!(damp, 0.0, "a converged sample was damped");
    let err = rms_err(&y, &reference);

    let mutant = code.replace(GATE, "if max_delta > damp_thresh {");
    let (ym, damp_m, _, _) = render(&mutant, 48000.0, "dk_contain_mutant");
    let err_m = rms_err(&ym, &reference);
    eprintln!(
        "vs 8x render: shipped {err:.3e} V rms (0 damped); converged-damping mutant \
         {err_m:.3e} V rms ({damp_m} damped)"
    );
    assert!(damp_m > 0.0, "the witness no longer steps past the bound");
    assert!(
        err_m > 10.0 * err,
        "the mutant's error {err_m:e} is not clearly worse than {err:e}"
    );
}
