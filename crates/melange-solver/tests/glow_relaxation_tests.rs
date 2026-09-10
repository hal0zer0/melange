//! Glow-discharge / neon relaxation-oscillator tests (Phase 0c Stage 2a).
//!
//! The `N<name> a k NEON(VO VM IK RS IHOLD ROFF)` element is the ZA1001 relaxation
//! divider — the heart of the Philicorda AG7500 (70× of them). Coverage here:
//!  - free-running relaxation: strikes at VO, extinguishes at the emergent reset
//!    floor v0+RS·IHOLD≈89 V (Option-A maintaining-LINE model, intercept
//!    v0=VM−RS·IK decoupled from the static VM), analytic
//!    T ≈ RC·ln((Vb−V_floor)/(Vb−Vo)) period, on BOTH the nodal and DK-Schur
//!    routes (`assert_relax_fixed`);
//!  - supply-sensitivity (the ZA1001 divider-frequency dependence Philips
//!    regulated for — schemer's falsification criterion);
//!  - oversampling: preserves the oscillator physics, plus base-rate aliasing /
//!    anti-alias validation (arbiter ruling on sub-sample edges — OS is the
//!    anti-alias; output BLEP rejected; breakpoint re-solve deferred).

use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};

use melange_solver::codegen::{CodeGenerator, CodegenConfig, SubsampleFireMode};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// Rc = 1 MΩ, C = 10 nF → τ_charge = 10 ms. Vb = 170 V, VO = 135, VM = 93 @
// IK = 1.5 mA, RS = 3 kΩ → derived intercept v0 = VM−RS·IK = 88.5 V, emergent
// reset floor V_floor = v0+RS·IHOLD = 89.1 V.
// Analytic free-running period T = RC·ln((Vb−V_floor)/(Vb−Vo))
//                                = 0.01·ln(80.9/35) = 8.40 ms  (~119 Hz).
// Rail voltage `vb` is parametrized so the supply-sensitivity test can vary it.
fn relax_deck(vb: f64) -> String {
    format!(
        "\
Neon Relaxation Oscillator
.model NE1 NEON(VO=135 VM=93 IK=1.5e-3 RS=3000 IHOLD=2e-4 ROFF=300e6)
Vb rail 0 DC {vb}
Rc rail osc 1MEG
Cosc osc 0 10N
N1 osc 0 NE1
Rin in 0 1G
.END
"
    )
}

static COUNTER: AtomicU32 = AtomicU32::new(0);

fn generate_nodal_code(spice: &str, sample_rate: f64) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["osc"] - 1;
    mna.g[input_node][input_node] += 1.0;

    // Pinned to the WHOLE-SAMPLE latch: these tests document the pre-feature
    // glow behaviour (strike/extinguish on the grid, lit-hold BE, OS ASR). The
    // sub-sample fire re-solve (default `auto` on nodal-Schur glow decks) is
    // covered by `subsample_fire_tests.rs`.
    let config = CodegenConfig {
        circuit_name: "glow_relax_test".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        subsample_fire: SubsampleFireMode::Off,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

/// Generate with a given whole-circuit oversampling factor and NO output clamp,
/// so the raw reservoir swing reaches the output for spectral analysis. Used to
/// measure whether oversampling band-limits the glow strike/extinguish edge
/// (the arbiter-ruled anti-alias for this Stage-2a device — output BLEP was
/// rejected as ill-posed/cosmetic; OS is the physics-faithful mitigation).
fn generate_glow_code_os(spice: &str, sample_rate: f64, os: usize) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["osc"] - 1;
    mna.g[input_node][input_node] += 1.0;

    let config = CodegenConfig {
        circuit_name: "glow_os_test".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        oversampling_factor: os,
        output_clamp_v: 1.0e9, // don't clip the ~40 V reservoir AC swing
        subsample_fire: SubsampleFireMode::Off, // whole-sample latch (see generate_nodal_code)
        ..CodegenConfig::default()
    };
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen")
        .code
}

fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_glow_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_glow_{tag}_{id}_{counter}"));

    let full_code = format!("{code}\n\n{main_code}\n");
    std::fs::File::create(&src_path)
        .unwrap()
        .write_all(full_code.as_bytes())
        .unwrap();

    let compile = std::process::Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2024")
        .arg("-O")
        .output()
        .expect("rustc");
    let _ = std::fs::remove_file(&src_path);
    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        panic!(
            "Compilation failed for {tag}:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin_path).output().expect("run");
    let _ = std::fs::remove_file(&bin_path);
    if !run.status.success() {
        panic!(
            "Binary failed for {tag}:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }
    String::from_utf8_lossy(&run.stdout).to_string()
}

fn parse_kv(output: &str, key: &str) -> f64 {
    output
        .lines()
        .find(|l| l.starts_with(key))
        .unwrap_or_else(|| panic!("key '{key}' not found in:\n{output}"))
        .split('=')
        .nth(1)
        .unwrap()
        .trim()
        .parse()
        .unwrap()
}

/// Generate DK-route (`generate`) code for the glow deck — mirrors the CLI's
/// default routing for this circuit (DK Schur). Stamps the input conductance
/// before building the kernel (S = A⁻¹ bakes G).
fn generate_dk_code(spice: &str, sample_rate: f64) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["osc"] - 1;
    mna.g[input_node][input_node] += 1.0;
    let kernel = DkKernel::from_mna(&mna, sample_rate).expect("dk kernel");
    let config = CodegenConfig {
        circuit_name: "glow_relax_dk".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(config)
        .generate(&kernel, &mna, &netlist)
        .expect("dk codegen")
        .code
}

/// Observation `main`: free-runs the oscillator for 0.2 s and reports the raw
/// reservoir node voltage V(osc)=state.v_prev[OUTPUT_NODES[0]] and neon latch
/// state.device_0_state[0] directly (the returned output[] is DC-blocked/scaled/
/// clamped and hides the absolute reservoir voltage). Shared by both route tests.
const OBSERVE_MAIN: &str = r#"
fn main() {
    // No warmup(): the oscillator self-starts from the DC-OP seed and the 0.05 s
    // settle window below discards startup. (warmup() isn't emitted on the DK
    // generate() entry, so avoiding it keeps this main route-agnostic.)
    let mut state = CircuitState::default();

    let sr = 48000.0f64;
    let n = (sr * 0.2) as usize;
    let settle = (sr * 0.05) as usize;
    let osc = OUTPUT_NODES[0];

    let mut vmin = f64::INFINITY;
    let mut vmax = f64::NEG_INFINITY;
    let mut strikes = 0u32;      // dark->lit latch transitions
    let mut prev_lit = false;
    let mut extinguish_v_sum = 0.0f64;
    let mut extinguish_n = 0u32;
    let mut first_strike = -1i64;
    let mut last_strike = -1i64;

    for i in 0..n {
        let _ = process_sample(0.0, &mut state);
        let v = state.v_prev[osc];
        let lit = state.device_0_state[0] >= 0.5;
        if i >= settle {
            if v < vmin { vmin = v; }
            if v > vmax { vmax = v; }
            if !prev_lit && lit {
                strikes += 1;
                if first_strike < 0 { first_strike = i as i64; }
                last_strike = i as i64;
            }
            if prev_lit && !lit {
                extinguish_v_sum += v;
                extinguish_n += 1;
            }
        }
        prev_lit = lit;
    }
    let mean_extinguish_v = if extinguish_n > 0 { extinguish_v_sum / extinguish_n as f64 } else { -999.0 };
    let period_ms = if strikes >= 2 {
        (last_strike - first_strike) as f64 / (strikes - 1) as f64 / sr * 1000.0
    } else { -1.0 };
    println!("mean_extinguish_v={:.4}", mean_extinguish_v);
    println!("vmin={:.4}", vmin);
    println!("vmax={:.4}", vmax);
    println!("strikes={}", strikes);
    println!("period_ms={:.4}", period_ms);
    println!("nan_reset={}", state.diag_nan_reset_count);
}
"#;

/// Assert the Option-A maintaining-LINE lit model produces a correct relaxation
/// oscillation: strikes at VO, extinguishes at the emergent reset floor
/// v0+RS·IHOLD≈89 V (NOT the old fixed-VD=93 floor, and NOT the older
/// deep-discharge-to-0 bug), VO−floor swing, and the analytic
/// RC·ln((Vb−V_floor)/(Vb−Vo)) period. Shared by the nodal and DK route tests
/// so the fix is proven on BOTH paths — and, per the arbiter's "measure the
/// emergent floor" requirement, the reset floor is asserted directly.
///
/// Params: VO=135, VM=93@IK=1.5mA, RS=3000 → derived intercept v0=VM−RS·IK=88.5;
/// reset floor = v0+RS·IHOLD = 88.5+3000·2e-4 = 89.1 V. Measured floor lands at
/// ~88.95 (the whole-sample discharge steps just past the 89.1 extinction
/// threshold toward v0). This is the Option-A reset-floor drop: 93 → ~88.95,
/// −4 V, decoupling the intercept from the static maintaining point.
fn assert_relax_fixed(route: &str, out: &str) {
    let vmin = parse_kv(out, "vmin");
    let vmax = parse_kv(out, "vmax");
    let strikes = parse_kv(out, "strikes") as u32;
    let period_ms = parse_kv(out, "period_ms");
    let nan_reset = parse_kv(out, "nan_reset") as u32;
    let mean_extinguish_v = parse_kv(out, "mean_extinguish_v");
    eprintln!(
        "GLOW RELAX [{route}]: vmin={vmin:.3} V, vmax={vmax:.3} V, swing={:.3} V, \
         mean_extinguish_v={mean_extinguish_v:.2} V (emergent floor, target ~89 V), \
         strikes={strikes}, period={period_ms:.4} ms (analytic 8.40 ms + discretization), \
         nan_reset={nan_reset}",
        vmax - vmin
    );
    assert_eq!(nan_reset, 0, "[{route}] NaN resets in glow oscillator");
    assert!(
        strikes >= 5,
        "[{route}] expected sustained oscillation, got {strikes} strikes"
    );
    assert!(
        vmax >= 133.0 && vmax <= 137.0,
        "[{route}] peak should strike near VO=135, got vmax={vmax}"
    );
    // The emergent reset floor lands at the maintaining-line intercept region
    // v0+RS·IHOLD≈89 V — the Option-A fix. Regression guards BOTH ways: vs the
    // old fixed-VD=93 floor (too high, mis-centers the divider window) AND vs the
    // ancient fixed-RON deep-discharge bug (vmin≈-2.5 V, 137 V swing).
    assert!(
        (mean_extinguish_v - 89.0).abs() < 1.5 && vmin > 87.0 && vmin < 90.5,
        "[{route}] extinction must land at the emergent floor ≈89 V \
         (v0+RS·IHOLD), not the old 93 V: mean_extinguish_v={mean_extinguish_v} V, vmin={vmin} V"
    );
    let swing = vmax - vmin;
    assert!(
        (swing - 46.0).abs() < 3.0,
        "[{route}] sawtooth p-p should be VO−floor≈46 V, got {swing} V"
    );
    // Analytic (instantaneous-discharge) period is RC·ln((170−89.1)/(170−135))
    // = 8.40 ms. Measured runs ~3% high (8.67 ms): the finite RS·C=30 µs
    // discharge flank (~6 samples at 48k) plus whole-sample strike latency add
    // real time the instantaneous formula omits. Tolerance covers that bias.
    assert!(
        (period_ms - 8.40).abs() / 8.40 < 0.06,
        "[{route}] relaxation period should match analytic 8.40 ms + discretization (±6%), got {period_ms} ms"
    );
}

#[test]
fn test_glow_relaxation_oscillates() {
    // Nodal route (generate_nodal). See assert_relax_fixed for the checks.
    let code = generate_nodal_code(&relax_deck(170.0), 48000.0);
    let out = compile_and_run(&code, OBSERVE_MAIN, "relax_nodal");
    assert_relax_fixed("nodal full-LU/Schur", &out);
}

/// Same relaxation deck via the DK-Schur route (the CLI's default route for this
/// circuit — `melange compile` picks DK Schur, N=4 M=1). The in-solve glow eval
/// for DK lives in a SEPARATE emit site (`nr_helpers.rs`) from the nodal ones,
/// so this guards against the maintaining-voltage fix being applied to only one
/// route (which it initially was — the nodal test passed while the shipped DK
/// path still had the deep-discharge bug).
#[test]
fn test_glow_relaxation_oscillates_dk_route() {
    let out = compile_and_run(
        &generate_dk_code(&relax_deck(170.0), 48000.0),
        OBSERVE_MAIN,
        "relax_dk",
    );
    assert_relax_fixed("DK Schur", &out);
}

/// Measure the free-running relaxation period (ms) at a given rail voltage.
fn measure_period_ms(vb: f64, tag: &str) -> f64 {
    let code = generate_nodal_code(&relax_deck(vb), 48000.0);
    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    state.warmup();
    let sr = 48000.0f64;
    let n = (sr * 0.35) as usize;
    let settle = (sr * 0.05) as usize;
    let mut strikes = 0u32;
    let mut prev_lit = false;
    let mut first = -1i64;
    let mut last = -1i64;
    for i in 0..n {
        let _ = process_sample(0.0, &mut state);
        let lit = state.device_0_state[0] >= 0.5;
        if i >= settle && !prev_lit && lit {
            strikes += 1;
            if first < 0 { first = i as i64; }
            last = i as i64;
        }
        prev_lit = lit;
    }
    let period_ms = if strikes >= 2 {
        (last - first) as f64 / (strikes - 1) as f64 / sr * 1000.0
    } else { -1.0 };
    println!("period_ms={:.5}", period_ms);
    println!("strikes={}", strikes);
}
"#;
    let out = compile_and_run(&code, main_code, tag);
    assert!(
        parse_kv(&out, "strikes") as u32 >= 3,
        "too few cycles at Vb={vb}"
    );
    parse_kv(&out, "period_ms")
}

/// Supply-sensitivity: the relaxation period MUST depend on the rail voltage.
/// This is schemer's falsification criterion for the ZA1001 divider bracket
/// (thread 162, off the AG7500 service manual §6): Philips regulated the +1
/// rail specifically because "the correct oscillation frequency of the divider
/// sections depends on the supply voltage." A model whose dividers are
/// insensitive to Vb is wrong regardless of how well any single point locks.
/// T = RC·ln((Vb−V_floor)/(Vb−Vo)) with the emergent floor V_floor≈89.1 predicts
/// T(170 V)=8.40 ms, T(150 V)=14.01 ms (×1.67) — sensitivity rises sharply as Vo
/// approaches Vb. (Measured runs ~3% high on both from the finite-discharge/
/// whole-sample overhead; the ratio is robust to it since the overhead is a
/// near-constant absolute time.)
#[test]
fn test_glow_period_is_supply_sensitive() {
    let t_hi = measure_period_ms(170.0, "supply_170");
    let t_lo = measure_period_ms(150.0, "supply_150");
    eprintln!(
        "GLOW SUPPLY SENSITIVITY: T(170V)={t_hi:.4} ms, T(150V)={t_lo:.4} ms, \
         ratio={:.3} (analytic 14.01/8.40 = 1.67)",
        t_lo / t_hi
    );
    // Lowering the rail 170→150 V must lengthen the period substantially
    // (analytic ×1.67). A supply-insensitive model would give ratio ≈ 1.
    assert!(
        t_lo > t_hi * 1.4,
        "divider period must be supply-sensitive (schemer's ZA1001 falsification \
         test): T(150V)={t_lo} ms should be ≫ T(170V)={t_hi} ms"
    );
    // And each should track its own analytic prediction + discretization (±6%).
    assert!(
        (t_hi - 8.40).abs() / 8.40 < 0.06,
        "T(170V)={t_hi} vs analytic 8.40 ms"
    );
    assert!(
        (t_lo - 14.01).abs() / 14.01 < 0.06,
        "T(150V)={t_lo} vs analytic 14.01 ms"
    );
}

// ── Oversampling anti-alias validation (arbiter ruling on sub-sample edge
// handling: OS is the physics-faithful anti-alias for the sharp glow strike/
// extinguish edge; output BLEP was REJECTED as ill-posed for the mixed
// multi-oscillator target and doctrinally a cosmetic output filter per
// CLAUDE.md; the sub-sample breakpoint re-solve (option B) is deferred, gated on
// these numbers + a listening pass). A naive relaxation oscillator's edge
// (τ_discharge ≈ 10 µs at RON=1k/C=10n, well under the 20.8 µs base sample
// period) aliases at 48 kHz: its ultra-harmonics fold DOWN to inharmonic bins.
// Whole-circuit OS runs the solver at the internal rate and the half-band
// decimator removes the foldover before dropping to 48 kHz.
//
// Metric: aliasing-to-signal ratio (ASR) = energy in bins that are NOT
// harmonics of f_osc (below Nyquist), relative to harmonic energy. Hann window +
// harmonic-cluster attribution around the measured f0 (a bare near-Nyquist
// proxy is INVALID here — foldover lands at low freqs, and OS4 legitimately puts
// MORE real energy near Nyquist by resolving the edge). Lower ASR = less
// aliasing. This is spectral evidence only; the final D-vs-B call also needs a
// listening pass (SPICE/spectral necessary-but-not-sufficient).

/// Measurement main: free-run the oscillator, capture the decimated 48 kHz
/// output, Hann-window + radix-2 FFT, and report ASR = inharmonic/harmonic
/// energy (dB). Route-agnostic; no warmup().
const ASR_MAIN: &str = r#"
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 { j ^= bit; bit >>= 1; }
        j ^= bit;
        if i < j { re.swap(i, j); im.swap(i, j); }
    }
    let mut len = 2usize;
    while len <= n {
        let ang = -2.0 * std::f64::consts::PI / len as f64;
        let (wr, wi) = (ang.cos(), ang.sin());
        let mut i = 0usize;
        while i < n {
            let (mut cr, mut ci) = (1.0f64, 0.0f64);
            for k in 0..len / 2 {
                let a = i + k; let b = i + k + len / 2;
                let tr = cr * re[b] - ci * im[b];
                let ti = cr * im[b] + ci * re[b];
                re[b] = re[a] - tr; im[b] = im[a] - ti;
                re[a] += tr; im[a] += ti;
                let ncr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr; cr = ncr;
            }
            i += len;
        }
        len <<= 1;
    }
}
fn main() {
    let mut state = CircuitState::default();
    let sr = 48000.0f64;
    let settle = (sr * 0.05) as usize;
    let n = 16384usize; // 2^14
    for _ in 0..settle { let _ = process_sample(0.0, &mut state); }
    let mut buf = Vec::with_capacity(n);
    let mut strikes = 0u32; let mut prev_lit = false;
    let mut first = -1i64; let mut last = -1i64;
    for i in 0..n {
        let y = process_sample(0.0, &mut state)[0];
        buf.push(y);
        let lit = state.device_0_state[0] >= 0.5;
        if !prev_lit && lit { strikes += 1; if first < 0 { first = i as i64; } last = i as i64; }
        prev_lit = lit;
    }
    let period = if strikes >= 2 { (last - first) as f64 / (strikes - 1) as f64 } else { -1.0 };
    let f0 = sr / period;
    let mean = buf.iter().sum::<f64>() / n as f64;
    let mut re = vec![0.0f64; n];
    let mut im = vec![0.0f64; n];
    for i in 0..n {
        let w = 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / (n as f64 - 1.0)).cos();
        re[i] = (buf[i] - mean) * w;
    }
    fft(&mut re, &mut im);
    let half = n / 2;
    let bin_hz = sr / n as f64;
    let mut is_harm = vec![false; half];
    let kmax = (sr / 2.0 / f0) as usize;
    for k in 1..=kmax {
        let center = (k as f64 * f0 / bin_hz).round() as i64;
        for d in -2i64..=2 {
            let b = center + d;
            if b >= 4 && (b as usize) < half { is_harm[b as usize] = true; }
        }
    }
    let (mut e_harm, mut e_inharm) = (0.0f64, 0.0f64);
    for b in 4..half {
        let p = re[b] * re[b] + im[b] * im[b];
        if is_harm[b] { e_harm += p; } else { e_inharm += p; }
    }
    let asr_db = 10.0 * (e_inharm / e_harm.max(1e-300)).max(1e-300).log10();
    println!("f0_hz={:.3}", f0);
    println!("asr_db={:.3}", asr_db);
    println!("strikes={}", strikes);
}
"#;

fn measure_asr_db(spice: &str, os: usize, tag: &str) -> (f64, f64) {
    let code = generate_glow_code_os(spice, 48000.0, os);
    let out = compile_and_run(&code, ASR_MAIN, &format!("asr_{tag}_os{os}"));
    assert!(
        parse_kv(&out, "strikes") as u32 >= 5,
        "{tag} OS={os}: oscillator didn't run"
    );
    (parse_kv(&out, "asr_db"), parse_kv(&out, "f0_hz"))
}

/// Quantifies the arbiter's option-D claim on a LOW divider (f0≈126 Hz), where
/// the period is many samples so the ASR estimate is stable.
///
/// FINDING (recorded): base-rate aliasing is already modest (ASR ≈ −39 dB) and
/// 4× oversampling only nudges it ~1 dB — the reservoir cap band-limits the
/// discharge (τ=RON·C=10 µs ≈ one sample → a fast ramp, not an ideal step), so
/// there is little foldover to remove. This is evidence that the expensive
/// sub-sample breakpoint re-solve (option B) is NOT justified for this regime.
///
/// NOTE ON HIGH DIVIDERS: a naive FFT ASR on a self-oscillator whose period is
/// only a handful of samples is dominated by f0-vs-fs bin alignment and period
/// jitter, not aliasing — measured ASR there swings ±20 dB with tiny f0 shifts
/// (e.g. f0≈fs/8 folds foldover exactly onto harmonics). A trustworthy
/// cross-divider aliasing study needs non-coherent analysis + f0 chosen well off
/// fs sub-multiples, and ultimately a listening pass — both DEFERRED (they are
/// the gate on ever building option B). So this test asserts only the stable
/// low-divider facts; see `test_glow_oversampling_preserves_physics` for the OS
/// correctness guard that DOES generalize.
#[test]
fn test_glow_oversampling_low_divider_aliasing_modest() {
    let (lo1, flo) = measure_asr_db(&relax_deck(170.0), 1, "lo");
    let (lo2, _) = measure_asr_db(&relax_deck(170.0), 2, "lo");
    let (lo4, _) = measure_asr_db(&relax_deck(170.0), 4, "lo");
    eprintln!(
        "GLOW ASR low divider f0≈{flo:.0} Hz: OS1={lo1:.2} dB OS2={lo2:.2} dB OS4={lo4:.2} dB \
         (inharmonic/harmonic; lower = less aliasing)"
    );
    assert!(
        lo1 < -25.0,
        "base-rate aliasing should be modest (cap band-limits), ASR={lo1} dB"
    );
    assert!(
        lo4 <= lo1 + 1.0,
        "OS must not worsen aliasing: OS1={lo1} dB OS4={lo4} dB"
    );
}

/// Oversampling must PRESERVE the oscillator physics (this generalizes across
/// dividers, unlike the spectral ASR). At OS=4 the solver runs at 192 kHz; the
/// glow must still strike at VO, extinguish at ~VD, and hold the analytic period.
#[test]
fn test_glow_oversampling_preserves_physics() {
    let code = generate_glow_code_os(&relax_deck(170.0), 48000.0, 4);
    let out = compile_and_run(&code, OBSERVE_MAIN, "os4_physics");
    assert_relax_fixed("nodal OS=4", &out);
}

// ---------------------------------------------------------------------------
// Multi-stage chain divergence regression (nodal Schur NR false-convergence).
//
// A 2-stage ZA1001 divider (the minimal case that reproduces the failure).
// Pre-fix, the first-order NR warm-start predictor `2·i_prev − i_prev_prev`
// extrapolated the stiff lit-discharge current (the glow RS↔ROFF is a ~1e5
// conductance step) into the BA100 cathode diode's reverse breakdown, and the
// voltage-step-only Schur convergence test ACCEPTED the non-physical state
// (residual ~2.35e14 A) → runaway to ~1e6 V (149 magnitude-resets on nodal-trap;
// DK survived only via Step-6c damping masking the same overshoot). The fix is a
// zero-order warm start for glow circuits. This deck is openphilicorda's minimal
// repro (`nodal-trap-divergence.cir`), output node renamed `out`→`osc` for the
// harness. Output node "osc", input node "in", undriven.
fn chain_deck() -> String {
    "\
Glow 2-stage divider chain — nodal-trap divergence regression
R35 ht a5 1.5meg
R36 k5 0 47k
N5 a5 k5 ZA1001
C11 a5 m5 470p
C12 m5 0 5.6n
C36 a5 k6 22p
C35 k6 0 15p
R16 ht r16w 650k
R22 r16w a6 1.5meg
N6 a6 k6 ZA1001
D_GR1 k6 0 BA100
C13 a6 m6 1n
C14 m6 0 10n
R6 in k5 100k
R_out m6 osc 1k
R_load osc 0 1meg
VHT ht 0 DC 175
.model BA100 D(IS=2e-9 N=1.9 RS=8 CJO=1.5p BV=60)
.model ZA1001 NEON(VO=135 VM=93 IK=1.5m RS=3000 IHOLD=2e-4 ROFF=1e9)
.END
"
    .to_string()
}

const CHAIN_MAIN: &str = r#"
fn main() {
    let mut state = CircuitState::default();
    // ~0.15 s at 192 kHz. The divergence manifests within ~60 samples pre-fix.
    let n = 28800usize;
    let mut maxabs = 0.0f64;
    for _ in 0..n {
        let _ = process_sample(0.0, &mut state);
        for &v in state.v_prev.iter() {
            let a = v.abs();
            if a > maxabs { maxabs = a; }
        }
    }
    println!("magnitude_reset={}", state.diag_magnitude_reset_count);
    println!("nan_reset={}", state.diag_nan_reset_count);
    println!("maxabs={:.3}", maxabs);
}
"#;

fn assert_chain_no_divergence(route: &str, out: &str) {
    let mr = parse_kv(out, "magnitude_reset") as u64;
    let nr = parse_kv(out, "nan_reset") as u64;
    let maxabs = parse_kv(out, "maxabs");
    eprintln!("GLOW CHAIN [{route}]: magnitude_reset={mr}, nan_reset={nr}, maxabs={maxabs:.1} V (rail 175 V)");
    // Pre-fix nodal-trap gave 149 magnitude-resets and ~1e6 V. The fix must hold
    // every node within a few times the 175 V rail with zero resets, both routes.
    assert_eq!(
        mr, 0,
        "[{route}] glow chain DIVERGED ({mr} magnitude-resets) — nodal-Schur NR false-convergence regression"
    );
    assert_eq!(nr, 0, "[{route}] glow chain produced {nr} NaN resets");
    assert!(
        maxabs < 250.0,
        "[{route}] glow chain node voltage {maxabs} V >> 175 V rail — divergence"
    );
}

/// REGRESSION: the multi-stage glow divider chain must not diverge on the nodal
/// Schur route (the failure was SILENT — it looked like a collapsed single
/// frequency, but was a numerical runaway masked by the magnitude-reset cadence).
#[test]
fn test_glow_chain_no_divergence_nodal() {
    let code = generate_nodal_code(&chain_deck(), 192000.0);
    let out = compile_and_run(&code, CHAIN_MAIN, "chain_nodal");
    assert_chain_no_divergence("nodal Schur/trap", &out);
}

/// Same chain on the DK route. Pre-fix it survived only because Step-6c damping
/// masked the same overshoot; the fix un-masks it and it must stay clean.
#[test]
fn test_glow_chain_no_divergence_dk() {
    let code = generate_dk_code(&chain_deck(), 192000.0);
    let out = compile_and_run(&code, CHAIN_MAIN, "chain_dk");
    assert_chain_no_divergence("DK Schur", &out);
}

/// The 5-stage divider chain (Philicorda B5..B9 topology, undriven). Two
/// stages survive at audio rates on the zero-order warm start alone; five do
/// not: pre-fix the nodal route reaches ~4e4 V at 44.1/48/96 kHz (clean only
/// at 192 kHz), with no magnitude reset — a silent mis-divide.
fn chain5_deck() -> String {
    "\
Glow 5-stage divider chain — audio-rate nodal lit-hold regression
R35 ht a5 1.5meg
R36 k5 0 47k
N5 a5 k5 ZA1001
C11 a5 m5 470p
C12 m5 0 5.6n
C36 a5 k6 22p
C35 k6 0 15p
R16 ht r16w 620k
R22 r16w a6 1.5meg
N6 a6 k6 ZA1001
D_GR1 k6 0 BA100
C13 a6 m6 1n
C14 m6 0 10n
C15 a6 k7 22p
C16 k7 0 15p
R17 ht r17w 1300k
R23 r17w a7 1.8meg
N7 a7 k7 ZA1001
D_GR2 k7 0 BA100
C17 a7 m7 1n
C18 m7 0 18n
C19 a7 k8 22p
C20 k8 0 15p
R18 ht r18w 1340k
R24 r18w a8 1.8meg
N8 a8 k8 ZA1001
D_GR3 k8 0 BA100
C21 a8 m8 2.2n
C22 m8 0 68n
C23 a8 k9 47p
C24 k9 0 47p
R19 ht r19w 1100k
R25 r19w a9 1.8meg
N9 a9 k9 ZA1001
D_GR4 k9 0 BA100
C25 a9 m9 4.7n
C26 m9 0 150n
R6 in k5 100k
R_out m9 osc 1k
R_load osc 0 1meg
VHT ht 0 DC 175
.model BA100 D(IS=2e-9 N=1.9 RS=8 CJO=1.5p BV=60)
.model ZA1001 NEON(VO=135 VM=93 IK=1.5m RS=3000 IHOLD=2e-4 ROFF=1e9)
.END
"
    .to_string()
}

/// 0.5 s at 44.1 kHz. Pre-fix the 5-stage chain leaves the 175 V rail within
/// the first few ms; 0.5 s covers ~70 periods of the slowest (÷16) stage.
const CHAIN5_MAIN_44K1: &str = r#"
fn main() {
    let mut state = CircuitState::default();
    let n = 22050usize;
    let mut maxabs = 0.0f64;
    for _ in 0..n {
        let _ = process_sample(0.0, &mut state);
        for &v in state.v_prev.iter() {
            let a = v.abs();
            if a > maxabs { maxabs = a; }
        }
    }
    println!("magnitude_reset={}", state.diag_magnitude_reset_count);
    println!("nan_reset={}", state.diag_nan_reset_count);
    println!("nr_max_iter={}", state.diag_nr_max_iter_count);
    println!("maxabs={:.3}", maxabs);
}
"#;

/// REGRESSION (nodal, 44.1 kHz — the lowest common DAW rate and the worst
/// case: the trap ring on the glow's RS/ROFF conductance step grows with dt).
/// The lit-hold BE (breakpoint-BE countdown re-armed while any glow is lit)
/// plus a fallback RHS without the trap-midpoint `N_I * i_nl_prev` stamp must
/// hold every node at the rail with zero resets and zero trap max-iter hits.
#[test]
fn test_glow_chain5_no_divergence_nodal_44k1() {
    let code = generate_nodal_code(&chain5_deck(), 44100.0);
    assert!(
        code.contains("pub const GLOW_LIT_BE_SAMPLES: u32 = 1;"),
        "glow nodal code must emit the lit-hold const"
    );
    assert!(
        code.contains("state.breakpoint_be = state.breakpoint_be.max(GLOW_LIT_BE_SAMPLES);"),
        "glow nodal code must re-arm breakpoint-BE while lit"
    );
    assert!(
        !code.contains("N_I[i][j] * state.i_nl_prev[j]"),
        "glow nodal BE fallback must not carry the trap-midpoint i_nl_prev stamp"
    );
    let out = compile_and_run(&code, CHAIN5_MAIN_44K1, "chain5_nodal_44k1");
    assert_chain_no_divergence("nodal Schur/trap 44.1k 5-stage", &out);
    let nr_max = parse_kv(&out, "nr_max_iter") as u64;
    assert_eq!(
        nr_max, 0,
        "5-stage chain hit the trap NR wall {nr_max} times — lit samples must be solved on BE proactively"
    );
}

/// Byte-neutrality guard for the lit-hold machinery: a circuit without a
/// glow device (here a diode-only deck with the same cathode diode) must emit
/// neither the lit-hold const/re-arm nor the breakpoint-BE state, and must
/// keep the trap-midpoint `N_I * i_nl_prev` stamp in its BE fallback.
#[test]
fn test_glow_lit_hold_absent_without_glow() {
    let deck = "\
Diode clamp — no glow device
Vb rail 0 DC 175
Rc rail osc 1MEG
Cosc osc 0 10N
D1 osc k BA100
Rk k 0 47k
Rin in k 100k
.model BA100 D(IS=2e-9 N=1.9 RS=8 CJO=1.5p BV=60)
.END
";
    let code = generate_nodal_code(deck, 44100.0);
    assert!(!code.contains("GLOW_LIT_BE_SAMPLES"));
    assert!(!code.contains("breakpoint_be"));
    assert!(
        code.contains("N_I[i][j] * state.i_nl_prev[j]"),
        "non-glow BE fallback must keep the trap-midpoint i_nl_prev stamp (unchanged behaviour)"
    );
}
