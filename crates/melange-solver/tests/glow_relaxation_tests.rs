//! Glow-discharge / neon relaxation-oscillator tests (Phase 0c Stage 2a).
//!
//! The `N<name> a k NEON(VO VD RON ROFF)` element has ZERO test coverage — it
//! builds, but nothing proves it actually oscillates. The ZA1001 relaxation
//! divider is the heart of the Philicorda AG7500 (70× of them), so "does the
//! reservoir cap strike at VO and extinguish at VD, producing a sawtooth at
//! T ≈ RC·ln((Vb−Vd)/(Vb−Vo))?" is the gating question for that whole circuit.
//!
//! This first test is OBSERVATIONAL: it drives the classic RC relaxation
//! topology (rail → Rc → cap+neon to ground) and prints the steady-state
//! min/max node voltage, cycle count, and period so we can compare against the
//! analytic prediction and check the discharge flank (does it extinguish near
//! VD, or overshoot toward ground because the once-per-sample latch keeps RON
//! engaged for a whole sample?).

use std::io::Write;
use std::sync::atomic::{AtomicU32, Ordering};

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// Rc = 1 MΩ, C = 10 nF → τ_charge = 10 ms. Vb = 170 V, VO = 135, VD = 93.
// Analytic free-running period T = RC·ln((Vb−Vd)/(Vb−Vo))
//                                = 0.01·ln(77/35) = 7.885 ms  (~126.8 Hz).
// Rail voltage `vb` is parametrized so the supply-sensitivity test can vary it.
fn relax_deck(vb: f64) -> String {
    format!(
        "\
Neon Relaxation Oscillator
.model NE1 NEON(VO=135 VD=93 RON=1000 ROFF=1e9)
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

    let config = CodegenConfig {
        circuit_name: "glow_relax_test".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
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

/// Assert the maintaining-voltage lit model produces a correct relaxation
/// oscillation: strikes at VO, extinguishes at ~VD (NOT the old deep-discharge
/// to ~0 V), VO−VD swing, and the analytic RC·ln((Vb−Vd)/(Vb−Vo)) period.
/// Shared by the nodal and DK route tests so the fix is proven on BOTH paths.
fn assert_relax_fixed(route: &str, out: &str) {
    let vmin = parse_kv(out, "vmin");
    let vmax = parse_kv(out, "vmax");
    let strikes = parse_kv(out, "strikes") as u32;
    let period_ms = parse_kv(out, "period_ms");
    let nan_reset = parse_kv(out, "nan_reset") as u32;
    let mean_extinguish_v = parse_kv(out, "mean_extinguish_v");
    eprintln!(
        "GLOW RELAX [{route}]: vmin={vmin:.3} V, vmax={vmax:.3} V, swing={:.3} V, \
         mean_extinguish_v={mean_extinguish_v:.2} V (target ~VD=93), strikes={strikes}, \
         period={period_ms:.4} ms (analytic ~7.885 ms), nan_reset={nan_reset}",
        vmax - vmin
    );
    assert_eq!(nan_reset, 0, "[{route}] NaN resets in glow oscillator");
    assert!(strikes >= 5, "[{route}] expected sustained oscillation, got {strikes} strikes");
    assert!(
        vmax >= 133.0 && vmax <= 137.0,
        "[{route}] peak should strike near VO=135, got vmax={vmax}"
    );
    // Extinction lands at the maintaining voltage VD≈93, not overshooting to ~0.
    // Regression guard vs the old fixed-RON bug (vmin≈-2.5 V, 137 V swing, 62 Hz).
    assert!(
        (mean_extinguish_v - 93.0).abs() < 2.0 && vmin > 90.0,
        "[{route}] extinction must land at VD≈93 V, not overshoot: \
         mean_extinguish_v={mean_extinguish_v} V, vmin={vmin} V"
    );
    let swing = vmax - vmin;
    assert!(
        (swing - 42.0).abs() < 4.0,
        "[{route}] sawtooth p-p should be VO−VD≈42 V (not the ~137 V bug), got {swing} V"
    );
    assert!(
        (period_ms - 7.885).abs() / 7.885 < 0.05,
        "[{route}] relaxation period should match analytic 7.885 ms (±5%), got {period_ms} ms"
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
    let out = compile_and_run(&generate_dk_code(&relax_deck(170.0), 48000.0), OBSERVE_MAIN, "relax_dk");
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
    assert!(parse_kv(&out, "strikes") as u32 >= 3, "too few cycles at Vb={vb}");
    parse_kv(&out, "period_ms")
}

/// Supply-sensitivity: the relaxation period MUST depend on the rail voltage.
/// This is schemer's falsification criterion for the ZA1001 divider bracket
/// (thread 162, off the AG7500 service manual §6): Philips regulated the +1
/// rail specifically because "the correct oscillation frequency of the divider
/// sections depends on the supply voltage." A model whose dividers are
/// insensitive to Vb is wrong regardless of how well any single point locks.
/// T = RC·ln((Vb−Vd)/(Vb−Vo)) predicts T(170 V)=7.885 ms, T(150 V)=13.35 ms
/// (×1.69) — sensitivity rises sharply as Vo approaches Vb.
#[test]
fn test_glow_period_is_supply_sensitive() {
    let t_hi = measure_period_ms(170.0, "supply_170");
    let t_lo = measure_period_ms(150.0, "supply_150");
    eprintln!(
        "GLOW SUPPLY SENSITIVITY: T(170V)={t_hi:.4} ms, T(150V)={t_lo:.4} ms, \
         ratio={:.3} (analytic 13.35/7.885 = 1.69)",
        t_lo / t_hi
    );
    // Lowering the rail 170→150 V must lengthen the period substantially
    // (analytic ×1.69). A supply-insensitive model would give ratio ≈ 1.
    assert!(
        t_lo > t_hi * 1.4,
        "divider period must be supply-sensitive (schemer's ZA1001 falsification \
         test): T(150V)={t_lo} ms should be ≫ T(170V)={t_hi} ms"
    );
    // And each should track its own analytic prediction (±6%).
    assert!((t_hi - 7.885).abs() / 7.885 < 0.06, "T(170V)={t_hi} vs analytic 7.885 ms");
    assert!((t_lo - 13.35).abs() / 13.35 < 0.06, "T(150V)={t_lo} vs analytic 13.35 ms");
}
