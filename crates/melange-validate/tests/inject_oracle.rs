//! Verification oracle for the `.inject` / `.tap` runtime-feedback directives.
//!
//! `.inject <node> <field> R=<ohms>` stamps a Thevenin (or `RSHUNT=` Norton)
//! source at a circuit node, driven per (inner) sample by a `process_sample`
//! argument. `.tap <node>` returns the RAW inner-rate node voltage.
//!
//! These tests compile generated code with `rustc` and run it. They need only
//! `rustc` (no ngspice) and take well under a second, so they run in the
//! default `cargo test`.
//!
//! Gates (from the plan's §Gates):
//!   * (a) a constant injection behind `R` == a literal DC source behind the
//!     same `R` (the Thevenin stamp is the exact same machinery as the
//!     ngspice-validated audio input port).
//!   * (b, Norton) a VARYING Norton (`RSHUNT=`) injection == a real time-varying
//!     current source behind the same shunt `R` (expressed as its exact Thevenin
//!     equivalent, the validated input port). Locks in the Norton trapezoidal
//!     `(val+val_prev)` discretization — instantaneous `val` is half-amplitude.
//!   * (c) a synthetic near-unity closed loop tracks the analytic steady state
//!     `H·c/(1 − k·H)` with `H = Rload/(Rload + R_inj)` across loop gain — a
//!     stamp with a flipped SIGN or wrong `R` changes `H` and is CAUGHT,
//!     most visibly at the marginal (near-unity) operating point.
//!   * (5) an inner-rate injection tone appears at the tap UN-band-limited
//!     (asserts a `rate=inner` injection bypasses the anti-alias up-filter
//!     and the tap bypasses the decimator).
//!   * (host) a `rate=host` injection IS the audio input: two identical
//!     nonlinear paths, one driven through `input`, one through a `rate=host`
//!     injection behind the same 1 Ω, agree to the last bit at 1x, 2x and 4x
//!     on every route (Thevenin and Norton); the same drive declared
//!     `rate=inner` and fed per-sub-step repeats does not.

mod support;

use std::sync::atomic::{AtomicU32, Ordering};

static COUNTER: AtomicU32 = AtomicU32::new(0);

/// A resolved injection for the test harness: (node name, field, ohms,
/// norton, declared rate — `"host"` or `"inner"`).
struct Inj<'a> {
    node: &'a str,
    field: &'a str,
    ohms: f64,
    norton: bool,
    rate: &'a str,
}

/// Generate code for `netlist_str` with the given injections + taps, append
/// `main_body`, compile with `rustc`, run (no stdin), and return stdout.
///
/// Builds through `melange_solver::build::build`, the build `melange compile`
/// ships: the injections and taps are written into the deck as `.inject` /
/// `.tap` directives (in `injects` order, each with its declared `rate=`), and the build stamps `G_in` and every injection conductance, routes
/// DK/nodal, and resolves the specs itself.
#[allow(clippy::too_many_arguments)]
fn gen_and_run(
    netlist_str: &str,
    input_name: &str,
    input_resistance: f64,
    output_node: &str,
    injects: &[Inj],
    taps: &[&str],
    oversampling: usize,
    main_body: &str,
) -> String {
    gen_and_run_with(
        netlist_str,
        input_name,
        input_resistance,
        &[output_node],
        injects,
        taps,
        oversampling,
        main_body,
        |_| {},
    )
}

/// [`gen_and_run`] with several output nodes and a hook that adjusts the
/// build options (solver route) before the build.
#[allow(clippy::too_many_arguments)]
fn gen_and_run_with(
    netlist_str: &str,
    input_name: &str,
    input_resistance: f64,
    output_nodes: &[&str],
    injects: &[Inj],
    taps: &[&str],
    oversampling: usize,
    main_body: &str,
    tweak: impl FnOnce(&mut melange_solver::build::BuildOptions),
) -> String {
    let mut deck: String = netlist_str
        .lines()
        .filter(|l| !l.trim().eq_ignore_ascii_case(".end"))
        .map(|l| format!("{l}\n"))
        .collect();
    for inj in injects {
        let kind = if inj.norton { "RSHUNT" } else { "R" };
        deck.push_str(&format!(
            ".inject {} {} {kind}={} rate={}\n",
            inj.node, inj.field, inj.ohms, inj.rate
        ));
    }
    for t in taps {
        deck.push_str(&format!(".tap {t}\n"));
    }
    deck.push_str(".end\n");

    let mut opts = melange_solver::build::BuildOptions {
        circuit_name: "inject_oracle".to_string(),
        input_resistance: Some(input_resistance),
        oversampling: Some(oversampling),
        oversampling_set: melange_solver::build::OversamplingSet::Deck,
        dc_block: false, // raw DC comparison — no 5 Hz HPF on the output
        ..support::options(48000.0, input_name, output_nodes)
    };
    tweak(&mut opts);
    let built = support::build(&deck, &opts);
    let names: Vec<&str> = built
        .injection_specs
        .iter()
        .map(|s| s.name.as_str())
        .collect();
    let fields: Vec<&str> = injects.iter().map(|i| i.field).collect();
    assert_eq!(names, fields, "injection order");
    let generated = built.generated;

    let full_source = format!("{}\n{}", generated.code, main_body);
    let tmp = std::env::temp_dir();
    let n = COUNTER.fetch_add(1, Ordering::SeqCst);
    let pid = std::process::id();
    let src = tmp.join(format!("inject_oracle_{pid}_{n}.rs"));
    let bin = tmp.join(format!("inject_oracle_{pid}_{n}"));
    std::fs::write(&src, &full_source).expect("write src");
    let compile = std::process::Command::new("rustc")
        .arg(&src)
        .arg("-o")
        .arg(&bin)
        .arg("--edition=2021")
        .arg("-O")
        .output()
        .expect("rustc spawn");
    let _ = std::fs::remove_file(&src);
    assert!(
        compile.status.success(),
        "generated code failed to compile:\n{}",
        String::from_utf8_lossy(&compile.stderr)
    );
    let out = std::process::Command::new(&bin).output().expect("run");
    let _ = std::fs::remove_file(&bin);
    assert!(out.status.success(), "generated binary exited nonzero");
    String::from_utf8_lossy(&out.stdout).into_owned()
}

const RC_NODE: &str = "\
* single node with a load resistor and a small parasitic cap
Rin in nx 100meg
Rload nx 0 1k
Cpar nx 0 1p
.end
";

/// (a) A constant injection behind `R` produces the same raw node voltage as a
/// literal DC voltage source behind the same `R`.
#[test]
fn inject_constant_equals_literal_source_behind_r() {
    // Deck A: `.inject nx fb R=1k`, driven at constant 1.0 V.
    let deck_a = RC_NODE.to_string();
    let main = "
fn main() {
    let mut state = CircuitState::default();
    let inj = [[1.0f64; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR];
    let mut tap = 0.0;
    for _ in 0..300_000 { let (_o, t) = process_sample(0.0, &[], &inj, &mut state); tap = t[0][0]; }
    println!(\"{tap:.6}\");
}";
    let a = gen_and_run(
        &deck_a,
        "in",
        1.0,
        "nx",
        &[Inj {
            node: "nx",
            field: "fb",
            ohms: 1000.0,
            norton: false,
            rate: "inner",
        }],
        &["nx"],
        1,
        main,
    );

    // Deck B: a real DC source `Vfb` behind an identical 1k resistor `Rfb`.
    let deck_b = "\
* literal DC source behind R at nx
Rin in nx 100meg
Rload nx 0 1k
Cpar nx 0 1p
Vfb fsrc 0 1.0
Rfb fsrc nx 1k
.end
";
    let main_b = "
fn main() {
    let mut state = CircuitState::default();
    let inj = [[0.0f64; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR]; // NUM_INJECT == 0
    let mut tap = 0.0;
    for _ in 0..300_000 { let (_o, t) = process_sample(0.0, &[], &inj, &mut state); tap = t[0][0]; }
    println!(\"{tap:.6}\");
}";
    let b = gen_and_run(deck_b, "in", 1.0, "nx", &[], &["nx"], 1, main_b);

    let va: f64 = a.trim().parse().expect("parse A");
    let vb: f64 = b.trim().parse().expect("parse B");
    assert!(
        (va - vb).abs() < 1e-5,
        "injection behind R ({va}) must equal literal source behind R ({vb})"
    );
    // Sanity: both near the 1 k / (1 k + 1 k) divider (loaded slightly by the
    // 100 MΩ input path + 1 Ω input shunt).
    assert!((va - 0.5).abs() < 0.02, "unexpected divider value {va}");
}

/// (c) A synthetic closed loop (`inj = k·tap_prev + c`) tracks the analytic
/// steady state `H·c/(1 − k·H)` with `H = Rload/(Rload+R_inj) = 0.5` across
/// loop gain — including the sign of `k` (negative feedback) and the marginal
/// near-unity case. A flipped stamp sign or wrong `R` would fail this.
#[test]
fn inject_closed_loop_tracks_analytic_sign_and_impedance() {
    let main = "
fn run(k: f64, c: f64) -> f64 {
    let mut state = CircuitState::default();
    let mut fb = 0.0f64; let mut tap = 0.0;
    for _ in 0..500_000 {
        let val = k * fb + c;
        let inj = [[val; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR];
        let (_o, t) = process_sample(0.0, &[], &inj, &mut state);
        tap = t[0][0]; fb = tap;
    }
    tap
}
fn main() {
    for &k in &[0.0f64, 0.8, -0.8, 1.8] { println!(\"{:.6}\", run(k, 1.0)); }
}";
    let out = gen_and_run(
        RC_NODE,
        "in",
        1.0,
        "nx",
        &[Inj {
            node: "nx",
            field: "fb",
            ohms: 1000.0,
            norton: false,
            rate: "inner",
        }],
        &["nx"],
        1,
        main,
    );
    let vals: Vec<f64> = out.lines().map(|l| l.trim().parse().unwrap()).collect();
    let h = 0.5_f64;
    for (i, &k) in [0.0, 0.8, -0.8, 1.8].iter().enumerate() {
        let analytic = h * 1.0 / (1.0 - k * h);
        assert!(
            (vals[i] - analytic).abs() < 5e-3,
            "loop gain k={k}: measured {} vs analytic {analytic} (sign/R regression?)",
            vals[i]
        );
    }
    // Negative feedback must move the OPPOSITE way from positive feedback.
    assert!(
        vals[2] < vals[0] && vals[0] < vals[1],
        "feedback sign not load-bearing"
    );
}

/// (5) An inner-rate injection tone (±I alternating per internal sample at 2×
/// oversampling) appears at the tap UN-band-limited: the injection bypasses the
/// anti-alias up-filter and the tap bypasses the decimator. Through the filter
/// this above-host-Nyquist content would collapse toward zero.
///
/// At 4× oversampling, a period-4 inner pattern `[+A,+A,−A,−A]` is a 48 kHz
/// tone (host rate 48 kHz → host Nyquist 24 kHz → this sits deep in the up-
/// filter's STOPBAND) and — critically — is NOT the trapezoidal null: the
/// proper-trap `(val+val_prev)` history turns it into `[0,2A,0,−2A]`, still a
/// 48 kHz tone, not zero (unlike a pure inner-Nyquist alternation, which trap
/// nulls for BOTH source kinds). The `Cpar` pole (~80 kHz) sits above the tone
/// but below the 96 kHz inner-Nyquist, so the tone passes and there is no stiff-
/// cap z≈−1 artifact. If the injection went through the up-filter this stopband
/// tone would be ~−60 dB; the raw tap shows it at full amplitude.
#[test]
fn inject_inner_rate_tone_reaches_tap_unfiltered() {
    const TONE_NODE: &str = "\
* node whose RC pole (~80 kHz) passes 48 kHz but is below 96 kHz inner-Nyquist
Rin in nx 100meg
Rload nx 0 1k
Cpar nx 0 4n
.end
";
    let main = "
fn main() {
    let mut state = CircuitState::default();
    // OVERSAMPLING_FACTOR == 4: period-4 inner tone at 48 kHz (up-filter stopband).
    let inj: [[f64; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR] = [[1.0],[1.0],[-1.0],[-1.0]];
    let (mut mn, mut mx) = (f64::MAX, f64::MIN);
    for n in 0..40_000 {
        let (_o, t) = process_sample(0.0, &[], &inj, &mut state);
        if n > 30_000 { for k in 0..OVERSAMPLING_FACTOR { let v = t[k][0]; mn = mn.min(v); mx = mx.max(v); } }
    }
    println!(\"{:.6}\", mx - mn);
}";
    let out = gen_and_run(
        TONE_NODE,
        "in",
        1.0,
        "nx",
        &[Inj {
            node: "nx",
            field: "fb",
            ohms: 1000.0,
            norton: false,
            rate: "inner",
        }],
        &["nx"],
        4,
        main,
    );
    let spread: f64 = out.trim().parse().expect("parse");
    // Bypass ⇒ the 48 kHz tone reaches the tap (spread ~1.1 V). Through the
    // up-filter this stopband tone would be crushed toward 0.
    assert!(
        spread > 0.5,
        "inner-rate tone spread {spread} too small — injection is being band-limited?"
    );
}

/// (b, Norton) The direct analog of the Thevenin equivalence test, for the
/// Norton (`RSHUNT=`) source kind, driven by a NON-trivial VARYING sequence.
///
/// Reference = melange's own audio input port, the canonical ngspice-validated
/// `(V+V_prev)·G` Thevenin source. A Norton current source `I(t)` with shunt
/// `R0` is *exactly* a Thevenin `V(t)=I(t)·R0` behind `R0` (identical topology:
/// `1/R0` to ground + the source), so the two decks must produce the same node
/// voltage sample-for-sample. This locks in the Norton discretization:
/// instantaneous `rhs += val` produces HALF the correct amplitude (empirically
/// 0.2524 vs 0.4955); the trapezoidal `rhs += val + val_prev` matches exactly.
#[test]
fn inject_norton_varying_equals_current_source_behind_shunt() {
    // Deck N: Norton current inject I(t) at nx, shunt R0 = 1 kΩ.
    const NORTON_DECK: &str = "\
* Norton current inject at nx, shunt 1k
Rin in nx 1g
Rload nx 0 1k
Cpar nx 0 10n
.end
";
    // Reference deck: nx IS the input node; R_in = R0 = 1 kΩ supplied via the
    // harness. Same load + cap, so identical topology. Drive V(t) = I(t)·R0.
    const REF_DECK: &str = "\
* input-port Thevenin reference at nx (R_in = R0 = 1k)
Rload nx 0 1k
Cpar nx 0 10n
.end
";
    // A 3 kHz sine at 48 kHz — a decent fraction of fs where instantaneous `val`
    // and averaged `(val+val_prev)` differ substantially. Print the settled tap.
    const NSAMP: usize = 48_000;
    const AMP: f64 = 1e-3; // 1 mA
    const R0: f64 = 1000.0;
    let n_main = format!(
        "
fn main() {{
    let mut state = CircuitState::default();
    let mut out = String::new();
    for n in 0..{NSAMP} {{
        let i = {AMP} * (2.0*std::f64::consts::PI*3000.0*(n as f64)/48000.0).sin();
        let inj = [[i; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR];
        let (_o, t) = process_sample(0.0, &[], &inj, &mut state);
        if n >= {NSAMP} - 400 {{ out.push_str(&format!(\"{{:.9}} \", t[0][0])); }}
    }}
    println!(\"{{}}\", out.trim());
}}"
    );
    let ref_main = format!(
        "
fn main() {{
    let mut state = CircuitState::default();
    let mut out = String::new();
    for n in 0..{NSAMP} {{
        let v = {AMP} * {R0}_f64 * (2.0*std::f64::consts::PI*3000.0*(n as f64)/48000.0).sin();
        let inj = [[0.0; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR];
        let (_o, t) = process_sample(v, &[], &inj, &mut state);
        if n >= {NSAMP} - 400 {{ out.push_str(&format!(\"{{:.9}} \", t[0][0])); }}
    }}
    println!(\"{{}}\", out.trim());
}}"
    );
    let n_out = gen_and_run(
        NORTON_DECK,
        "in",
        1.0,
        "nx",
        &[Inj {
            node: "nx",
            field: "fb",
            ohms: R0,
            norton: true,
            rate: "inner",
        }],
        &["nx"],
        1,
        &n_main,
    );
    let ref_out = gen_and_run(REF_DECK, "nx", R0, "nx", &[], &["nx"], 1, &ref_main);

    let nv: Vec<f64> = n_out
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    let rv: Vec<f64> = ref_out
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    assert_eq!(nv.len(), rv.len(), "sample count mismatch");
    assert!(nv.len() >= 300, "not enough settled samples");
    let max_abs_diff = nv
        .iter()
        .zip(&rv)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f64, f64::max);
    let peak = rv.iter().fold(0.0_f64, |m, &x| m.max(x.abs()));
    assert!(peak > 0.1, "reference produced no signal (peak {peak})");
    // Same tolerance class as the Thevenin equivalence test: a factor-of-2
    // discretization error would show max_abs_diff ~= peak/2 ~= 0.25 V.
    assert!(
        max_abs_diff < 1e-4,
        "Norton varying inject deviates from current-source-behind-shunt reference: \
         max_abs_diff={max_abs_diff} (peak {peak}) — Norton trap form wrong?"
    );
}

/// Two identical antiparallel-diode clippers. Path A hangs off the audio input
/// `in` (melange stamps its 1 Ω Thevenin); path B hangs off `in2`, driven by a
/// `.inject in2 drv R=1` (or `RSHUNT=1`) — the same 1 Ω source. Nothing
/// couples the two paths, so with the same signal into both they must agree.
const TWIN_CLIPPERS: &str = "\
* twin diode clippers: A via the input, B via .inject
R1 in a 1k
D1 a 0 DM
D2 0 a DM
C1 a 0 10n
R2 in2 b 1k
D3 b 0 DM
D4 0 b DM
C2 b 0 10n
.model DM D(IS=2.52n N=1.752 RS=0.568)
.end
";

/// Drive `x` into the input and into the injection (host-rate argument, or
/// the same `x` repeated across every inner sub-step), clipping hard (2 V
/// peak, 5 kHz). Prints `max|A − B|` and A's peak.
fn twin_main(host: bool) -> String {
    let (host_arg, inner_arg) = if host {
        (
            "&[x; NUM_INJECT_HOST]",
            "&[[0.0; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR]",
        )
    } else {
        (
            "&[0.0; NUM_INJECT_HOST]",
            "&[[x; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR]",
        )
    };
    format!(
        "
fn main() {{
    let mut state = CircuitState::default();
    let (mut maxd, mut peak) = (0.0f64, 0.0f64);
    for n in 0..24_000 {{
        let x = 2.0 * (2.0 * std::f64::consts::PI * 5000.0 * (n as f64) / 48000.0).sin();
        let (o, _t) = process_sample(x, {host_arg}, {inner_arg}, &mut state);
        maxd = maxd.max((o[0] - o[1]).abs());
        peak = peak.max(o[0].abs());
    }}
    println!(\"{{maxd:e}} {{peak:e}}\");
}}"
    )
}

/// Run [`TWIN_CLIPPERS`] with path B driven by an injection of the given kind
/// and rate; returns `(max|A − B|, peak of A)`.
fn run_twin(oversampling: usize, route: &str, norton: bool, rate: &str) -> (f64, f64) {
    let out = gen_and_run_with(
        TWIN_CLIPPERS,
        "in",
        1.0,
        &["a", "b"],
        &[Inj {
            node: "in2",
            field: "drv",
            ohms: 1.0,
            norton,
            rate,
        }],
        &[],
        oversampling,
        &twin_main(rate == "host"),
        |o| match route {
            "dk" => o.solver = "dk".to_string(),
            "schur" => {
                o.solver = "nodal".to_string();
                o.nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::Schur;
            }
            "full-lu" => {
                o.solver = "nodal".to_string();
                o.nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::FullLu;
            }
            other => panic!("unknown route {other}"),
        },
    );
    let v: Vec<f64> = out
        .split_whitespace()
        .map(|t| t.parse().expect("parse"))
        .collect();
    (v[0], v[1])
}

/// (host) A `rate=host` injection behind 1 Ω is the audio input: the twin
/// clippers agree to the last bit at 1x, 2x and 4x, on DK, nodal Schur and
/// nodal full-LU, for both source kinds (a Norton current `I` behind
/// `RSHUNT=1` is the Thevenin `V = I` behind 1 Ω). Before `rate=host` an
/// injection skipped the input's up-filter, so B led A by the filter's group
/// delay and was not band-limited.
#[test]
fn inject_host_rate_matches_audio_input_bit_for_bit() {
    for route in ["dk", "schur", "full-lu"] {
        for os in [1, 2, 4] {
            for norton in [false, true] {
                let (maxd, peak) = run_twin(os, route, norton, "host");
                assert!(peak > 0.3, "{route} {os}x: no signal (peak {peak})");
                assert!(
                    maxd <= 1e-12 * peak,
                    "{route} {os}x norton={norton}: host-rate injection differs from the \
                     audio input by {maxd:e} (peak {peak})"
                );
            }
        }
    }
}

/// The witness above has teeth: the same drive declared `rate=inner` and fed
/// naive per-sub-step repeats (no up-filter) differs from the audio input by a
/// large fraction of the signal at 2x and 4x, and is identical only at 1x,
/// where the two rates are the same thing.
#[test]
fn inject_inner_rate_naive_repeats_differ_from_audio_input() {
    let (maxd1, _) = run_twin(1, "dk", false, "inner");
    assert_eq!(maxd1, 0.0, "1x: rate=inner must equal rate=host");
    for os in [2, 4] {
        let (maxd, peak) = run_twin(os, "dk", false, "inner");
        assert!(
            maxd > 0.1 * peak,
            "{os}x: naive inner repeats should NOT match the up-filtered input \
             (max diff {maxd:e}, peak {peak})"
        );
    }
}
