//! Verification oracle for the `.inject` / `.tap` runtime-feedback directives.
//!
//! `.inject <node> <field> R=<ohms>` stamps a Thevenin (or `RSHUNT=` Norton)
//! source at a circuit node, driven per (inner) sample by a `process_sample`
//! argument. `.tap <node>` returns the RAW inner-rate node voltage. See
//! `local-docs/inject-directive-plan.md`.
//!
//! These tests compile generated code with `rustc` and run it, so — like the
//! ngspice tests in `spice_validation.rs` — they are `#[ignore]`d from the
//! default `cargo test`. Run with:
//!
//! ```text
//! cargo test -p melange-validate --test inject_oracle -- --include-ignored
//! ```
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
//!     (asserts the injection bypasses the anti-alias up-filter and the tap
//!     bypasses the decimator).

use std::sync::atomic::{AtomicU32, Ordering};

static COUNTER: AtomicU32 = AtomicU32::new(0);

/// A resolved injection for the test harness: (node name, field, ohms, norton).
struct Inj<'a> {
    node: &'a str,
    field: &'a str,
    ohms: f64,
    norton: bool,
}

/// Generate code for `netlist_str` with the given injections + taps, append
/// `main_body`, compile with `rustc`, run (no stdin), and return stdout.
///
/// Mirrors the CLI compile path: stamps `G_in` at `in` and `1/ohms` at each
/// injection node BEFORE building the kernel, routes DK/nodal automatically,
/// and threads the resolved `InjectionSpec`/`TapSpec` into `CodegenConfig`.
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
    use melange_solver::codegen::ir::{InjectionSpec, TapSpec};
    use melange_solver::codegen::routing;
    use melange_solver::codegen::{CodeGenerator, CodegenConfig};
    use melange_solver::dk::DkKernel;

    let netlist = melange_solver::parser::Netlist::parse(netlist_str)
        .unwrap_or_else(|e| panic!("parse: {}", e.message));
    let mut mna = melange_solver::mna::MnaSystem::from_netlist(&netlist)
        .unwrap_or_else(|e| panic!("mna: {e}"));

    let resolve = |name: &str| -> usize {
        mna.node_map
            .get(name)
            .copied()
            .unwrap_or_else(|| panic!("node '{name}' not found"))
    };

    let input_node = resolve(input_name).saturating_sub(1);
    let out_idx = resolve(output_node).saturating_sub(1);
    if input_node < mna.n {
        mna.g[input_node][input_node] += 1.0 / input_resistance; // G_in = 1/R_in
    }

    // Stamp each injection conductance BEFORE the kernel (baked into S / DC-OP).
    let mut inj_specs = Vec::new();
    for inj in injects {
        let idx = resolve(inj.node) - 1;
        mna.g[idx][idx] += 1.0 / inj.ohms;
        inj_specs.push(InjectionSpec {
            node: idx,
            name: inj.field.to_string(),
            resistance: inj.ohms,
            norton: inj.norton,
        });
    }
    let tap_specs: Vec<TapSpec> = taps
        .iter()
        .map(|t| TapSpec {
            node: resolve(t) - 1,
            name: (*t).to_string(),
        })
        .collect();

    let has_inductors = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();
    let mut dk_failed = false;
    let kernel = if has_inductors {
        DkKernel::from_mna_augmented(&mna, 48000.0).expect("aug DK")
    } else {
        match DkKernel::from_mna(&mna, 48000.0) {
            Ok(k) => k,
            Err(_) => {
                dk_failed = true;
                DkKernel::from_mna_augmented(&mna, 48000.0).expect("DK fallback")
            }
        }
    };
    let decision = routing::auto_route(&kernel, &mna, dk_failed);
    let use_nodal = decision.route == routing::SolverRoute::Nodal;
    if use_nodal {
        let slots = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
            &netlist,
            Some(&mna),
        )
        .unwrap_or_default();
        if !slots.is_empty() {
            mna.expand_bjt_internal_nodes(&slots);
        }
    }

    let config = CodegenConfig {
        circuit_name: "inject_oracle".to_string(),
        sample_rate: 48000.0,
        input_node,
        output_nodes: vec![out_idx],
        input_resistance,
        dc_block: false, // raw DC comparison — no 5 Hz HPF on the output
        oversampling_factor: oversampling,
        injections: inj_specs,
        taps: tap_specs,
        ..CodegenConfig::default()
    };
    let generator = CodeGenerator::new(config);
    let generated = if use_nodal {
        generator.generate_nodal(&mna, &netlist)
    } else {
        generator.generate(&kernel, &mna, &netlist)
    }
    .unwrap_or_else(|e| panic!("codegen: {e}"));

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
#[ignore] // compiles + runs generated code (needs rustc); heavy
fn inject_constant_equals_literal_source_behind_r() {
    // Deck A: `.inject nx fb R=1k`, driven at constant 1.0 V.
    let deck_a = RC_NODE.to_string();
    let main = "
fn main() {
    let mut state = CircuitState::default();
    let inj = [[1.0f64; NUM_INJECT]; OVERSAMPLING_FACTOR];
    let mut tap = 0.0;
    for _ in 0..300_000 { let (_o, t) = process_sample(0.0, &inj, &mut state); tap = t[0][0]; }
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
    let inj = [[0.0f64; NUM_INJECT]; OVERSAMPLING_FACTOR]; // NUM_INJECT == 0
    let mut tap = 0.0;
    for _ in 0..300_000 { let (_o, t) = process_sample(0.0, &inj, &mut state); tap = t[0][0]; }
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
#[ignore]
fn inject_closed_loop_tracks_analytic_sign_and_impedance() {
    let main = "
fn run(k: f64, c: f64) -> f64 {
    let mut state = CircuitState::default();
    let mut fb = 0.0f64; let mut tap = 0.0;
    for _ in 0..500_000 {
        let val = k * fb + c;
        let inj = [[val; NUM_INJECT]; OVERSAMPLING_FACTOR];
        let (_o, t) = process_sample(0.0, &inj, &mut state);
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
#[ignore]
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
    let inj: [[f64; NUM_INJECT]; OVERSAMPLING_FACTOR] = [[1.0],[1.0],[-1.0],[-1.0]];
    let (mut mn, mut mx) = (f64::MAX, f64::MIN);
    for n in 0..40_000 {
        let (_o, t) = process_sample(0.0, &inj, &mut state);
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
#[ignore]
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
        let inj = [[i; NUM_INJECT]; OVERSAMPLING_FACTOR];
        let (_o, t) = process_sample(0.0, &inj, &mut state);
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
        let inj = [[0.0; NUM_INJECT]; OVERSAMPLING_FACTOR];
        let (_o, t) = process_sample(v, &inj, &mut state);
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
