//! Which oversampling leg does the residual belong to?
//!
//! `melange validate --oversampling` reports a residual against the circuit.
//! The half-band chain has two legs — an interpolator before the solver and a
//! decimator after it — and a single-tone comparison cannot, on its own, say
//! which one the residual came from. This harness answers that by SWAPPING one
//! leg at a time for a linear-phase (dispersion-free) equivalent and measuring
//! what moves.
//!
//! Run it (needs ngspice on PATH):
//!
//! ```text
//! cargo run -p melange-validate --release --example os_leg_attribution
//! ```
//!
//! # The two experiments
//!
//! **Test A — per-leg identity swap, single tone.** Four builds of the same
//! 2x circuit: shipped/shipped (which must reproduce the shipped
//! `process_sample` bit for bit, and is checked to), ideal-up/shipped-down,
//! shipped-up/ideal-down, ideal/ideal. Each is compared against the SAME
//! unfiltered ngspice reference with the same best-fit constant delay
//! estimator `melange validate` uses. The leg whose swap moves the residual is
//! the leg the residual belongs to.
//!
//! **Test B — two-tone IMD, up leg only.** A single tone cannot show what the
//! up leg does, because on one tone an allpass is a pure time shift and the
//! circuit is time-invariant. On a multi-tone input the up leg genuinely
//! reshapes the waveform that reaches the clipper, so the intermodulation
//! products change. This is the only measurement that can see it, and it is
//! melange-against-melange (shipped up leg vs ideal up leg): no reference
//! engine is involved or needed.
//!
//! # What "ideal" means here
//!
//! A linear-phase FIR half-band (Kaiser-windowed sinc, 129 taps, cutoff at
//! half the internal Nyquist) in place of the shipped polyphase IIR allpass
//! chain. Same job, same magnitude response class, constant group delay — so
//! the ONLY thing removed by the swap is the leg's dispersion, which the
//! best-fit constant delay cannot absorb. A literal identity (sample-and-hold
//! up, drop-sample down) would have confounded the answer with imaging and
//! aliasing that the shipped leg is there to suppress.
//!
//! Each ideal leg delays by 32 host samples, which is outside the +/- half
//! period the shipped alignment searches, so this harness seeds its fit from a
//! coarse integer scan instead and reports the fitted delay for each variant.

use melange_validate::{alignment, dc_block_signal, run_melange_solver_from_str};

const DECK: &str = "crates/melange-validate/tests/data/tube_screamer_u/circuit.cir";
const SAMPLE_RATE: f64 = 48_000.0;
const OVERSAMPLING: usize = 2;
/// Samples dropped from the front of every metric window. The ideal legs delay
/// by 32 host samples each, so the aligned reference's first ~100 samples are
/// built from the constant extension; 256 clears that for every variant and
/// keeps the four rows commensurable.
const WINDOW_SKIP: usize = 256;

fn main() {
    let duration: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.5);
    let amplitude: f64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0.3);

    let netlist = std::fs::read_to_string(DECK).expect("read deck");
    let (stripped, _) = melange_validate::strip_vin_source(&netlist, "in");
    let n = (duration * SAMPLE_RATE) as usize;

    println!("os_leg_attribution — {DECK}");
    println!("  {SAMPLE_RATE} Hz, {duration} s ({n} samples), {amplitude} V, {OVERSAMPLING}x\n");

    test_a(&netlist, &stripped, n, amplitude);
    test_b(&stripped, n, amplitude);
}

// ---------------------------------------------------------------------------
// Test A — per-leg swap against an unfiltered ngspice reference
// ---------------------------------------------------------------------------

fn test_a(netlist: &str, stripped: &str, n: usize, amplitude: f64) {
    println!("== Test A: per-leg identity swap, single 1 kHz tone ==\n");

    let input: Vec<f64> = (0..n)
        .map(|i| amplitude * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / SAMPLE_RATE).sin())
        .collect();

    // ngspice reference — unfiltered, exactly as `melange validate` now uses it.
    let pwl: Vec<(f64, f64)> = input
        .iter()
        .enumerate()
        .map(|(i, &v)| (i as f64 / SAMPLE_RATE, v))
        .collect();
    let spice = melange_validate::run_transient_with_thevenin_pwl(
        netlist,
        1.0 / SAMPLE_RATE,
        n as f64 / SAMPLE_RATE,
        "in",
        &pwl,
        1.0,
        &["out".to_string()],
    )
    .expect("ngspice");
    let mut reference = spice.get_node_voltage("out").expect("out node").to_vec();
    dc_block_signal(&mut reference, spice.sample_rate);

    // The 1x build, for the floor: what the solver costs against ngspice with
    // no oversampling filters in the path at all.
    let out_1x = run(stripped, &input, 1, None);
    report_row("1x (no oversampling)", &reference, &out_1x, 0);

    // The shipped 2x build, through the library's own entry point.
    let shipped = run(stripped, &input, OVERSAMPLING, None);

    // Sanity: the custom driver with BOTH legs shipped must reproduce it
    // exactly, or nothing below is a measurement of a leg swap.
    let both_shipped = run(stripped, &input, OVERSAMPLING, Some(&driver(false, false)));
    let worst = shipped
        .iter()
        .zip(both_shipped.iter())
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);
    println!(
        "  driver check: custom wrapper with both legs shipped vs process_sample: \
         worst |diff| = {worst:e}"
    );
    assert!(
        worst == 0.0,
        "the custom driver is not the shipped path (worst {worst:e}); the leg-swap \
         rows below would not mean anything"
    );
    println!();

    report_row("2x shipped up + shipped down", &reference, &shipped, 0);
    let ideal_up = run(stripped, &input, OVERSAMPLING, Some(&driver(true, false)));
    report_row("2x IDEAL  up + shipped down", &reference, &ideal_up, 32);
    let ideal_dn = run(stripped, &input, OVERSAMPLING, Some(&driver(false, true)));
    report_row("2x shipped up + IDEAL  down", &reference, &ideal_dn, 32);
    let ideal_both = run(stripped, &input, OVERSAMPLING, Some(&driver(true, true)));
    report_row("2x IDEAL  up + IDEAL  down", &reference, &ideal_both, 64);

    println!();
    // Both tables, so the reader can separate what the SOLVER costs per
    // harmonic from what the oversampling filters add on top of it.
    harmonic_table(&reference, &out_1x, "1x");
    println!();
    harmonic_table(&reference, &shipped, "2x shipped");
}

/// Metrics for one variant, against the reference, with a best-fit constant
/// delay seeded from a coarse integer scan around `hint`.
///
/// `hint` is 32 host samples per IDEAL leg in the variant and 0 otherwise:
/// each linear-phase leg delays by exactly `HB_M / 2` host samples by
/// construction, while a shipped leg delays by order one. The scan is then
/// bounded to half a stimulus period either side of that, for the same reason
/// the shipped estimator is: on a periodic tone the residual is nearly
/// periodic in the delay, so an unbounded scan can and does settle a whole
/// cycle away (measured: a blind 0..96 scan put the shipped 2x variant at
/// 50.66 samples, exactly 48 — one period — past its 2.66).
fn report_row(label: &str, reference: &[f64], actual: &[f64], hint: isize) {
    let len = reference.len().min(actual.len());
    let window = WINDOW_SKIP..len;

    // Coarse integer seed, bounded to half a period either side of the hint.
    let mut best = (hint, f64::INFINITY);
    for d in (hint - 24)..=(hint + 24) {
        let e: f64 = (window.start..window.end)
            .map(|i| {
                let r = reference[(i as isize - d).clamp(0, len as isize - 1) as usize];
                (actual[i] - r).powi(2)
            })
            .sum();
        if e < best.1 {
            best = (d, e);
        }
    }
    let fit = alignment::fit_constant_delay(
        reference,
        actual,
        window.clone(),
        best.0 as f64,
        1000.0,
        SAMPLE_RATE,
    );
    let aligned = alignment::apply_fractional_delay(reference, fit.delay_samples);

    let r = &aligned[window.clone()];
    let a = &actual[window.clone()];
    let m = len - WINDOW_SKIP;
    let ref_rms = (r.iter().map(|v| v * v).sum::<f64>() / m as f64).sqrt();
    let rms_err = (r
        .iter()
        .zip(a.iter())
        .map(|(x, y)| (y - x).powi(2))
        .sum::<f64>()
        / m as f64)
        .sqrt();
    let peak_err = r
        .iter()
        .zip(a.iter())
        .map(|(x, y)| (y - x).abs())
        .fold(0.0f64, f64::max);
    let rho = pearson(r, a);

    println!(
        "  {label:<28} delay {:8.4} sp | nRMS {:8.4}% | 1-rho {:10.3e} | peak err {:.4e} V",
        fit.delay_samples,
        100.0 * rms_err / ref_rms,
        1.0 - rho,
        peak_err
    );
}

fn pearson(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len() as f64;
    let mx = x.iter().sum::<f64>() / n;
    let my = y.iter().sum::<f64>() / n;
    let (mut sxy, mut sxx, mut syy) = (0.0, 0.0, 0.0);
    for (a, b) in x.iter().zip(y.iter()) {
        let (da, db) = (a - mx, b - my);
        sxy += da * db;
        sxx += da * da;
        syy += db * db;
    }
    sxy / (sxx.sqrt() * syy.sqrt())
}

/// Per-harmonic diagnostic table, aligned on the FUNDAMENTAL's phase.
///
/// Fundamental-phase alignment is the wrong choice for a gate — it zeroes the
/// fundamental by construction and bills everything to the harmonics — but it
/// is the right frame for a dispersion read-out: with the fundamental pinned,
/// each harmonic's residual phase IS the dispersion at that frequency.
fn harmonic_table(reference: &[f64], actual: &[f64], label: &str) {
    println!("  per-harmonic diagnostic ({label}), phase relative to the fundamental:");
    println!("    (a DIAGNOSTIC frame, not the gate: the fundamental is zeroed by construction)");
    let len = reference.len().min(actual.len());
    let start = WINDOW_SKIP;
    let f0 = 1000.0;
    // A whole number of fundamental periods, or the fundamental leaks into
    // every harmonic bin. Measured on a 704-sample window (14.67 periods) the
    // leak was order 1e-3 V — the size of the harmonics being read.
    let period = (SAMPLE_RATE / f0).round() as usize;
    let n = ((len - start) / period) * period;
    if n == 0 {
        println!("    (window too short for a whole fundamental period)");
        return;
    }
    let (rf, af) = (
        goertzel(&reference[start..start + n], f0, SAMPLE_RATE),
        goertzel(&actual[start..start + n], f0, SAMPLE_RATE),
    );
    let ref_phase0 = rf.1;
    let act_phase0 = af.1;
    println!("    k   f (Hz)   |ref| (V)    |mel| (V)   mag err     phase err (deg)");
    for k in 1..=9 {
        let f = f0 * k as f64;
        if f >= SAMPLE_RATE * 0.5 {
            break;
        }
        let (rm, rp) = goertzel(&reference[start..start + n], f, SAMPLE_RATE);
        let (am, ap) = goertzel(&actual[start..start + n], f, SAMPLE_RATE);
        let dphi =
            wrap_deg(((ap - act_phase0 * k as f64) - (rp - ref_phase0 * k as f64)).to_degrees());
        println!(
            "    {k:<3} {f:>7.0} {rm:>11.3e} {am:>12.3e} {:>10.3e} {dphi:>14.3}",
            am - rm
        );
    }
}

fn wrap_deg(mut d: f64) -> f64 {
    while d > 180.0 {
        d -= 360.0;
    }
    while d < -180.0 {
        d += 360.0;
    }
    d
}

/// Magnitude and phase of `signal` at `freq`, by direct correlation.
fn goertzel(signal: &[f64], freq: f64, sample_rate: f64) -> (f64, f64) {
    let w = 2.0 * std::f64::consts::PI * freq / sample_rate;
    let (mut re, mut im) = (0.0f64, 0.0f64);
    for (i, &x) in signal.iter().enumerate() {
        let a = w * i as f64;
        re += x * a.cos();
        im -= x * a.sin();
    }
    let n = signal.len() as f64;
    (2.0 * (re * re + im * im).sqrt() / n, im.atan2(re))
}

// ---------------------------------------------------------------------------
// Test B — two-tone IMD, shipped up leg vs ideal up leg
// ---------------------------------------------------------------------------

fn test_b(stripped: &str, n: usize, amplitude: f64) {
    println!("\n== Test B: two-tone IMD, up leg swapped (melange vs melange) ==\n");

    // Two pairs, and the reason for each:
    //
    // 1 kHz + 1.1 kHz is where a Tube Screamer is actually played, but it sits
    // deep inside the half-band's flat, near-linear-phase region, so if the up
    // leg's dispersion mattered anywhere it would matter least here. It is the
    // musically relevant control.
    //
    // 19 kHz + 20 kHz is the conventional aliasing pair, and it is the
    // informative one for THIS question: it sits at the top of the passband
    // where the allpass chain's phase varies fastest, and its products are
    // exactly what oversampling exists to keep out of the band. If swapping the
    // up leg changes anything, it changes it here.
    for (f1, f2) in [(1000.0f64, 1100.0f64), (19_000.0, 20_000.0)] {
        let a = amplitude / 2.0; // equal tones, same total peak drive
        let input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / SAMPLE_RATE;
                a * (2.0 * std::f64::consts::PI * f1 * t).sin()
                    + a * (2.0 * std::f64::consts::PI * f2 * t).sin()
            })
            .collect();

        let shipped = run(stripped, &input, OVERSAMPLING, Some(&driver(false, false)));
        let ideal = run(stripped, &input, OVERSAMPLING, Some(&driver(true, false)));

        println!("  {f1:.0} Hz + {f2:.0} Hz, {a:.3} V each:");
        // Spectra over the SECOND HALF of the run only. `C_out` (0.1 uF) into
        // 1 MOhm gives tau = 0.1 s and the DC blocker's tau is 32 ms, so the
        // front of the run is an envelope, not a level: measured on a 20 ms
        // run the two variants' fundamentals differed by 0.8 dB purely because
        // their 32-sample delay difference lands on a different point of that
        // envelope. Nothing before steady state says anything about IMD.
        let total = shipped.len().min(ideal.len());
        let start = (total / 2).max(WINDOW_SKIP);
        // A whole number of periods of the 100 Hz grid every product below
        // sits on, so nothing leaks between bins.
        let m = ((total - start) / 480) * 480;
        assert!(m > 0, "run too short for the spectral window");
        let probes = [
            ("f2 - f1     ", f2 - f1),
            ("2f1 - f2    ", 2.0 * f1 - f2),
            ("2f2 - f1    ", 2.0 * f2 - f1),
            ("3f1 - 2f2   ", (3.0 * f1 - 2.0 * f2).abs()),
            ("3f2 - 2f1   ", 3.0 * f2 - 2.0 * f1),
            ("f1          ", f1),
            ("f2          ", f2),
        ];
        println!("    product        f (Hz)   shipped-up (V)   ideal-up (V)      delta (dB)");
        for (name, f) in probes {
            if f <= 0.0 || f >= SAMPLE_RATE * 0.5 {
                continue;
            }
            let (sm, _) = goertzel(&shipped[start..start + m], f, SAMPLE_RATE);
            let (im, _) = goertzel(&ideal[start..start + m], f, SAMPLE_RATE);
            let db = if sm > 0.0 && im > 0.0 {
                20.0 * (sm / im).log10()
            } else {
                f64::NAN
            };
            println!("    {name}  {f:>8.0}   {sm:>13.5e}   {im:>12.5e}   {db:>13.4}");
        }
        // Whole-waveform difference between the two up legs, delay-fitted so a
        // pure time shift is not counted. Read it as an upper bound, NOT as
        // IMD: the two up legs differ in PHASE across the band, and a single
        // constant delay cannot remove a frequency-dependent phase, so this
        // number is dominated by that linear difference. The product levels
        // above are the IMD measurement; this is context for them.
        //
        // The SHIPPED output is the one delayed into place: the ideal leg
        // delays by 32 host samples and the shipped one by order one, so the
        // shift runs shipped -> ideal, seeded at 32. The search width is fixed
        // at +/-24 samples (half a 1 kHz period) rather than derived from the
        // tone pair — at 19/20 kHz half a period is 1.3 samples, which would
        // pin the fit to its own seed and report that as a result.
        let len = start + m;
        let fit =
            alignment::fit_constant_delay(&shipped, &ideal, start..len, 32.0, 1000.0, SAMPLE_RATE);
        let al = alignment::apply_fractional_delay(&shipped, fit.delay_samples);
        let rms_ref = (ideal[start..len].iter().map(|v| v * v).sum::<f64>() / m as f64).sqrt();
        let rms_err = (al[start..len]
            .iter()
            .zip(ideal[start..len].iter())
            .map(|(x, y)| (y - x).powi(2))
            .sum::<f64>()
            / m as f64)
            .sqrt();
        println!(
            "    whole waveform (upper bound, phase-dominated), delay-fitted ({:.3} sp): \
             nRMS {:.4}%\n",
            fit.delay_samples,
            100.0 * rms_err / rms_ref
        );
    }
}

// ---------------------------------------------------------------------------
// Plumbing
// ---------------------------------------------------------------------------

fn run(stripped: &str, input: &[f64], oversampling: usize, main_code: Option<&str>) -> Vec<f64> {
    run_melange_solver_from_str(
        stripped,
        input,
        SAMPLE_RATE,
        "out",
        "in",
        melange_solver::codegen::BjtFaMode::Auto,
        "auto",
        false,
        false,
        oversampling,
        main_code,
    )
    .expect("melange run")
}

/// The custom `fn main` appended to the generated source: the shipped 2x
/// wrapper, re-spelled, with either leg replaceable by a linear-phase FIR
/// half-band. `os_halfband`, `os_halfband_down`, `OS_COEFFS` and
/// `process_sample_inner` are the generated module's own items.
fn driver(ideal_up: bool, ideal_down: bool) -> String {
    let up = if ideal_up {
        "ideal_up.process(x)"
    } else {
        "os_halfband(x, &OS_COEFFS, &mut state.os_up_state)"
    };
    let down = if ideal_down {
        "ideal_down.process(oe[0], oo[0])"
    } else {
        "os_halfband_down(oe[0], oo[0], &OS_COEFFS, &mut state.os_dn_state[0])"
    };
    format!(
        r#"
// ---- diagnostic: linear-phase FIR half-band legs (os_leg_attribution) ----
const HB_M: usize = 64; // even, so the delay is a whole number of host samples

fn hb_bessel_i0(x: f64) -> f64 {{
    let half = x / 2.0;
    let mut term = 1.0f64;
    let mut sum = 1.0f64;
    for k in 1..80 {{
        let f = half / k as f64;
        term *= f * f;
        sum += term;
        if term < 1e-18 * sum {{ break; }}
    }}
    sum
}}

/// Kaiser-windowed half-band lowpass, 2*HB_M+1 taps, cutoff at half the
/// internal Nyquist. Symmetric, so its group delay is exactly HB_M internal
/// samples at every frequency — that is the whole point of using it here.
fn hb_taps() -> Vec<f64> {{
    let beta = 9.0f64;
    let i0b = hb_bessel_i0(beta);
    let mut h: Vec<f64> = (0..(2 * HB_M + 1))
        .map(|k| {{
            let u = k as f64 - HB_M as f64;
            let s = if u.abs() < 1e-12 {{
                1.0
            }} else {{
                let p = std::f64::consts::PI * 0.5 * u;
                p.sin() / p
            }};
            let r = u / HB_M as f64;
            let w = if r.abs() >= 1.0 {{
                0.0
            }} else {{
                hb_bessel_i0(beta * (1.0 - r * r).sqrt()) / i0b
            }};
            0.5 * s * w
        }})
        .collect();
    let sum: f64 = h.iter().sum();
    for t in h.iter_mut() {{ *t /= sum; }}
    h
}}

struct IdealUp {{ line: Vec<f64>, h: Vec<f64> }}
impl IdealUp {{
    fn new() -> Self {{ Self {{ line: vec![0.0; HB_M], h: hb_taps() }} }}
    /// One host sample in, the two internal-rate samples out.
    fn process(&mut self, x: f64) -> (f64, f64) {{
        self.line.pop();
        self.line.insert(0, x);
        // Even phase: the half-band's only non-zero even tap is the centre one.
        let even = self.line[HB_M / 2];
        // Odd phase: the odd taps, at twice the gain (zero-stuffing).
        let mut odd = 0.0;
        let mut q: isize = -(HB_M as isize) + 1;
        while q <= HB_M as isize - 1 {{
            let idx = ((HB_M as isize + q - 1) / 2) as usize;
            odd += self.h[(HB_M as isize + q) as usize] * self.line[idx];
            q += 2;
        }}
        (even, 2.0 * odd)
    }}
}}

struct IdealDown {{ buf: Vec<f64>, h: Vec<f64> }}
impl IdealDown {{
    fn new() -> Self {{ Self {{ buf: vec![0.0; 2 * HB_M + 2], h: hb_taps() }} }}
    /// A pair of internal-rate samples in (x0 earlier), one host sample out.
    fn process(&mut self, x0: f64, x1: f64) -> f64 {{
        self.buf.pop();
        self.buf.insert(0, x0);
        self.buf.pop();
        self.buf.insert(0, x1);
        // buf[1 + k] is v[2n - k]; the FIR's centre lands on v[2n - HB_M].
        (0..=(2 * HB_M)).map(|k| self.h[k] * self.buf[1 + k]).sum()
    }}
}}

fn main() {{
    let mut state = CircuitState::default();
    let mut ideal_up = IdealUp::new();
    let mut ideal_down = IdealDown::new();
    let _ = (&ideal_up.h, &ideal_down.h);
    let stdin = std::io::stdin();
    let mut line = String::new();
    loop {{
        line.clear();
        if std::io::BufRead::read_line(&mut stdin.lock(), &mut line).unwrap() == 0 {{ break; }}
        if let Ok(input) = line.trim().parse::<f64>() {{
            let x = if input.is_finite() {{ input.clamp(-100.0, 100.0) }} else {{ 0.0 }};
            let (e, o) = {up};
            let oe = process_sample_inner(e, &mut state);
            let oo = process_sample_inner(o, &mut state);
            let v = {down};
            let out = if v.is_finite() {{ v }} else {{ 0.0 }};
            println!("{{:.15e}}", out);
        }}
    }}
}}
"#
    )
}
