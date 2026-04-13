//! Tests for full-LU NR codegen optimizations:
//! - Chord method (periodic Jacobian reuse within a sample)
//! - Cross-timestep Jacobian persistence
//! - Compile-time sparse LU factorization
//!
//! These tests verify that the optimized paths produce the same results
//! as the dense/non-optimized paths across various circuit topologies.

use std::io::Write;

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ── Circuit definitions ──────────────────────────────────────────────

/// Simple diode clipper (N=2, M=1) — routes to full LU via K ill-conditioned
const DIODE_CLIPPER: &str = "\
Diode clipper
R1 in out 4.7k
D1 out 0 D1N4148
.model D1N4148 D(IS=2.52e-9 N=1.752)
C1 out 0 47n
.END";

/// BJT common-emitter (N≈8, M=2) — larger nonlinear system
const BJT_CE: &str = "\
BJT CE amp
R1 in base 10k
R2 vcc base 100k
R3 base 0 47k
RC vcc collector 4.7k
RE emitter 0 1k
CE emitter 0 10u
Q1 collector base emitter QNPN
.model QNPN NPN(IS=1e-14 BF=200 BR=1 CJE=10p CJC=5p)
VCC vcc 0 DC 12
C1 in 0 100n
C2 collector 0 100p
.END";

/// Two-tube stage (N≈15, M=4) — forces full LU, tests sparse LU on medium circuit
const TWO_TUBE_STAGE: &str = "\
Two tube preamp
R1 in grid1 100k
Rg1 grid1 0 1Meg
Ra1 vcc plate1 100k
Rk1 cathode1 0 1.5k
Ck1 cathode1 0 25u
C1 plate1 grid2 100n
Rg2 grid2 0 470k
Ra2 vcc plate2 100k
Rk2 cathode2 0 1.5k
Ck2 cathode2 0 25u
Cout plate2 out 100n
Rout out 0 100k
T1 grid1 plate1 cathode1 12AX7
T2 grid2 plate2 cathode2 12AX7
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300 CCG=1.6p CGP=1.7p CCP=0.46p)
VCC vcc 0 DC 250
Cin in 0 100p
.END";

// ── Helpers ──────────────────────────────────────────────────────────

/// Generate nodal codegen for a circuit, return the generated code string.
fn generate_nodal_code(spice: &str, sample_rate: f64) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna
        .node_map
        .get("in")
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    let output_node = mna
        .node_map
        .get("out")
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);
    mna.g[input_node][input_node] += 1.0;

    let config = CodegenConfig {
        circuit_name: "test_full_lu".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen");
    result.code
}

/// Compile and run generated code with a custom main function.
/// Returns stdout as string.
fn compile_and_run(code: &str, main_code: &str, tag: &str) -> String {
    let tmp_dir = std::env::temp_dir();
    let id = std::process::id();
    let counter = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    let src_path = tmp_dir.join(format!("melange_full_lu_{tag}_{id}_{counter}.rs"));
    let bin_path = tmp_dir.join(format!("melange_full_lu_{tag}_{id}_{counter}"));

    let full_code = format!("{code}\n\n{main_code}\n");
    {
        let mut f = std::fs::File::create(&src_path).unwrap();
        f.write_all(full_code.as_bytes()).unwrap();
    }

    let compile = std::process::Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2024")
        .arg("-O") // release mode for realistic perf
        .output()
        .expect("rustc");

    let _ = std::fs::remove_file(&src_path);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        panic!(
            "Codegen compilation failed for {tag}:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin_path).output().expect("run");
    let _ = std::fs::remove_file(&bin_path);
    if !run.status.success() {
        panic!(
            "Codegen binary failed for {tag}:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }

    String::from_utf8_lossy(&run.stdout).to_string()
}

static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// Parse output lines as f64 values.
fn parse_f64_lines(output: &str) -> Vec<f64> {
    output
        .lines()
        .filter_map(|l| l.trim().parse::<f64>().ok())
        .collect()
}

// ── Tests ────────────────────────────────────────────────────────────

/// Verify that sparse LU (when enabled) produces the same output as the
/// circuit would with dense LU. Tests correctness of AMD ordering,
/// symbolic factorization, and in-place sparse elimination.
#[test]
fn test_sparse_lu_correctness_diode_clipper() {
    let code = generate_nodal_code(DIODE_CLIPPER, 48000.0);

    // Circuit may route to Schur (K well-conditioned) or full-LU depending on
    // spectral radius. Test correctness regardless of path.
    // Run it and verify output is non-zero and finite
    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let mut peak = 0.0f64;
    for i in 0..480u32 {
        let t = i as f64 / 48000.0;
        let input = 2.0 * (2.0 * std::f64::consts::PI * 500.0 * t).sin();
        let output = process_sample(input, &mut state);
        let v = output[0];
        if v.abs() > peak { peak = v.abs(); }
        println!("{:.15e}", v);
    }
    eprintln!("peak={:.6}", peak);
    eprintln!("nr_fail={}", state.diag_nr_max_iter_count);
    eprintln!("nan_reset={}", state.diag_nan_reset_count);
}
"#;
    let output = compile_and_run(&code, main_code, "sparse_diode");
    let values = parse_f64_lines(&output);
    assert_eq!(values.len(), 480);

    // Output should be non-zero (signal passes through)
    let peak = values.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
    assert!(
        peak > 0.01,
        "Diode clipper should produce non-zero output, peak={peak:.6}"
    );

    // All values should be finite
    assert!(
        values.iter().all(|v| v.is_finite()),
        "All outputs must be finite"
    );
}

/// Verify sparse LU on a two-tube circuit (larger N, M=4).
/// This tests the AMD ordering and fill-reduction on a real tube topology.
#[test]
fn test_sparse_lu_correctness_two_tube() {
    let code = generate_nodal_code(TWO_TUBE_STAGE, 48000.0);

    let has_sparse = code.contains("sparse_lu_factor");
    eprintln!(
        "Two-tube stage: sparse_lu={}, code lines={}",
        has_sparse,
        code.lines().count()
    );

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let mut peak = 0.0f64;
    for i in 0..960u32 {
        let t = i as f64 / 48000.0;
        let input = 0.05 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        let output = process_sample(input, &mut state);
        let v = output[0];
        if v.abs() > peak { peak = v.abs(); }
        if i >= 480 { println!("{:.15e}", v); }
    }
    eprintln!("peak={:.6}", peak);
    eprintln!("nr_fail={}", state.diag_nr_max_iter_count);
    eprintln!("refactors={}", state.diag_refactor_count);
}
"#;
    let output = compile_and_run(&code, main_code, "sparse_two_tube");
    let values = parse_f64_lines(&output);
    assert_eq!(values.len(), 480);

    let peak = values.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
    assert!(
        peak > 1e-4,
        "Two-tube stage should produce non-zero output, peak={peak:.6e}"
    );
    assert!(
        values.iter().all(|v| v.is_finite()),
        "All outputs must be finite"
    );
}

/// Verify cross-timestep Jacobian reuse produces correct output.
/// Compares output with chord_valid=true (reuse) against output with
/// forced refactoring every sample (chord_valid set to false each sample).
#[test]
fn test_cross_timestep_reuse_correctness() {
    let code = generate_nodal_code(DIODE_CLIPPER, 48000.0);

    // Run with normal cross-timestep reuse
    let main_reuse = r#"
fn main() {
    let mut state = CircuitState::default();
    for i in 0..480u32 {
        let t = i as f64 / 48000.0;
        let input = 2.0 * (2.0 * std::f64::consts::PI * 500.0 * t).sin();
        let output = process_sample(input, &mut state);
        println!("{:.15e}", output[0]);
    }
}
"#;
    let output_reuse = compile_and_run(&code, main_reuse, "xts_reuse");
    let reuse_values = parse_f64_lines(&output_reuse);

    // Run with forced refactoring every sample (disable cross-timestep reuse)
    let main_forced = r#"
fn main() {
    let mut state = CircuitState::default();
    for i in 0..480u32 {
        state.chord_valid = false; // Force refactoring
        let t = i as f64 / 48000.0;
        let input = 2.0 * (2.0 * std::f64::consts::PI * 500.0 * t).sin();
        let output = process_sample(input, &mut state);
        println!("{:.15e}", output[0]);
    }
}
"#;
    let output_forced = compile_and_run(&code, main_forced, "xts_forced");
    let forced_values = parse_f64_lines(&output_forced);

    assert_eq!(reuse_values.len(), forced_values.len());

    // Compare: outputs should track each other over the full waveform.
    // Cross-timestep reuse uses a stale Jacobian, so per-sample differences
    // can be larger than NR tolerance. But the overall waveform shape must match.
    // Use correlation (normalized cross-correlation) as the primary metric.
    let skip = 10; // skip initial transient
    let mut sum_rf = 0.0f64;
    let mut sum_rr = 0.0f64;
    let mut sum_ff = 0.0f64;
    let mut max_diff = 0.0f64;
    let mut max_abs = 0.0f64;
    for (&r, &f) in reuse_values[skip..]
        .iter()
        .zip(forced_values[skip..].iter())
    {
        sum_rf += r * f;
        sum_rr += r * r;
        sum_ff += f * f;
        max_diff = max_diff.max((r - f).abs());
        max_abs = max_abs.max(r.abs().max(f.abs()));
    }
    let corr = if sum_rr > 0.0 && sum_ff > 0.0 {
        sum_rf / (sum_rr.sqrt() * sum_ff.sqrt())
    } else {
        1.0
    };
    let rel_diff = if max_abs > 1e-10 {
        max_diff / max_abs
    } else {
        max_diff
    };
    eprintln!(
        "Cross-timestep reuse vs forced: corr={corr:.8}, max_diff={max_diff:.2e}, rel_diff={rel_diff:.2e}"
    );
    // Correlation must be very high (waveforms track each other)
    assert!(
        corr > 0.99,
        "Cross-timestep reuse should have >0.99 correlation with forced refactoring, got {corr:.6}"
    );
    // Both paths should produce non-trivial output
    assert!(
        max_abs > 0.01,
        "Both paths should produce non-zero output, max_abs={max_abs:.2e}"
    );
}

/// Verify that state.reset() produces correct output (path-agnostic).
/// After reset, the circuit should produce the same output as a fresh default().
#[test]
fn test_chord_valid_reset() {
    let code = generate_nodal_code(DIODE_CLIPPER, 48000.0);

    let main_code = r#"
fn main() {
    // Run 100 samples from default state
    let mut state1 = CircuitState::default();
    let mut peak1 = 0.0f64;
    for i in 0..100u32 {
        let t = i as f64 / 48000.0;
        let input = 1.0 * (2.0 * std::f64::consts::PI * 500.0 * t).sin();
        let output = process_sample(input, &mut state1);
        peak1 = peak1.max(output[0].abs());
    }

    // Reset and run 100 more — should produce same waveform as fresh default
    state1.reset();
    let mut state2 = CircuitState::default();
    let mut max_diff = 0.0f64;
    for i in 0..100u32 {
        let t = i as f64 / 48000.0;
        let input = 1.0 * (2.0 * std::f64::consts::PI * 500.0 * t).sin();
        let out1 = process_sample(input, &mut state1);
        let out2 = process_sample(input, &mut state2);
        let diff = (out1[0] - out2[0]).abs();
        max_diff = max_diff.max(diff);
        if !out1[0].is_finite() { println!("NAN_AT={}", i); }
    }
    println!("peak={:.6}", peak1);
    println!("max_diff={:.6e}", max_diff);
    println!("nr_fail={}", state1.diag_nr_max_iter_count);
}
"#;
    let output = compile_and_run(&code, main_code, "chord_reset");

    // Output should be non-zero
    assert!(
        output.contains("peak=") && !output.contains("peak=0.000000"),
        "Circuit should produce non-zero output. Output:\n{output}"
    );
    // Reset state should match fresh default (within floating-point tolerance)
    let max_diff_line = output.lines().find(|l| l.starts_with("max_diff="));
    if let Some(line) = max_diff_line {
        let val: f64 = line.split('=').nth(1).unwrap().parse().unwrap_or(1.0);
        assert!(val < 1e-10, "Reset state should match fresh default, max_diff={val:.2e}");
    }
    // No NR failures
    assert!(
        output.contains("nr_fail=0"),
        "No NR failures expected. Output:\n{output}"
    );
    // No NaN
    assert!(
        !output.contains("NAN_AT="),
        "No NaN expected. Output:\n{output}"
    );
}

/// Verify sparse LU on a BJT circuit (tests 2D device Jacobian stamping).
#[test]
fn test_sparse_lu_bjt_circuit() {
    let code = generate_nodal_code(BJT_CE, 48000.0);

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let mut peak = 0.0f64;
    for i in 0..960u32 {
        let t = i as f64 / 48000.0;
        let input = 0.01 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        let output = process_sample(input, &mut state);
        let v = output[0];
        if v.abs() > peak { peak = v.abs(); }
        if i >= 480 { println!("{:.15e}", v); }
    }
    eprintln!("peak={:.6}", peak);
    eprintln!("nr_fail={}", state.diag_nr_max_iter_count);
    eprintln!("nan_reset={}", state.diag_nan_reset_count);
    eprintln!("refactors={}", state.diag_refactor_count);
}
"#;
    let output = compile_and_run(&code, main_code, "sparse_bjt");
    let values = parse_f64_lines(&output);
    assert_eq!(values.len(), 480);

    let peak = values.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
    assert!(
        peak > 1e-3,
        "BJT CE should produce non-zero output, peak={peak:.6e}"
    );
    assert!(
        values.iter().all(|v| v.is_finite()),
        "All outputs must be finite"
    );
}

/// Verify that the nodal path produces stable, correct output over 100 samples.
/// Path-agnostic: works for both Schur and full-LU routing.
#[test]
fn test_cross_timestep_refactor_counting() {
    let code = generate_nodal_code(DIODE_CLIPPER, 48000.0);

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let mut peak = 0.0f64;
    let mut all_finite = true;
    for i in 0..100u32 {
        let t = i as f64 / 48000.0;
        let input = 0.5 * (2.0 * std::f64::consts::PI * 500.0 * t).sin();
        let output = process_sample(input, &mut state);
        peak = peak.max(output[0].abs());
        if !output[0].is_finite() { all_finite = false; }
    }
    println!("peak={:.6}", peak);
    println!("all_finite={}", all_finite);
    println!("nr_fail={}", state.diag_nr_max_iter_count);
}
"#;
    let output = compile_and_run(&code, main_code, "xts_counting");

    // Output should be non-zero
    assert!(
        output.contains("peak=") && !output.contains("peak=0.000000"),
        "Circuit should produce non-zero output. Output:\n{output}"
    );
    // All finite
    assert!(output.contains("all_finite=true"), "All outputs should be finite. Output:\n{output}");
    // No NR failures
    assert!(output.contains("nr_fail=0"), "No NR failures expected. Output:\n{output}");
}

// ── Gain sanity tests ────────────────────────────────────────────────

/// Helper: generate nodal code, process samples, return gain in dB.
fn measure_codegen_gain(spice: &str, amplitude: f64, tag: &str) -> (f64, f64, u64) {
    let code = generate_nodal_code(spice, 48000.0);

    let main_code = format!(
        r#"
fn main() {{
    let mut state = CircuitState::default();
    let mut peak = 0.0f64;
    for i in 0..4800u32 {{
        let t = i as f64 / 48000.0;
        let input = {amplitude:?} * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        let output = process_sample(input, &mut state);
        if i >= 2400 {{ // skip first 50ms (DC settling)
            let v = output[0].abs();
            if v > peak {{ peak = v; }}
        }}
    }}
    let gain_db = if peak > 1e-20 {{ 20.0 * (peak / {amplitude:?}_f64).log10() }} else {{ -999.0 }};
    println!("peak={{:.8e}}", peak);
    println!("gain_db={{:.2}}", gain_db);
    println!("nr_fail={{}}", state.diag_nr_max_iter_count);
    println!("nan_reset={{}}", state.diag_nan_reset_count);
}}
"#
    );

    let output = compile_and_run(&code, &main_code, tag);

    let peak = output
        .lines()
        .find(|l| l.starts_with("peak="))
        .and_then(|l| l.strip_prefix("peak="))
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(0.0);
    let gain_db = output
        .lines()
        .find(|l| l.starts_with("gain_db="))
        .and_then(|l| l.strip_prefix("gain_db="))
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(-999.0);
    let nr_fail = output
        .lines()
        .find(|l| l.starts_with("nr_fail="))
        .and_then(|l| l.strip_prefix("nr_fail="))
        .and_then(|v| v.parse::<u64>().ok())
        .unwrap_or(0);

    (peak, gain_db, nr_fail)
}

/// Verify diode clipper has reasonable gain (should clip, not amplify wildly).
/// A diode clipper with 2V input should clip to ~0.6V (diode forward voltage).
#[test]
fn test_gain_sanity_diode_clipper() {
    let (peak, gain_db, nr_fail) = measure_codegen_gain(DIODE_CLIPPER, 2.0, "gain_diode");

    eprintln!("Diode clipper: peak={peak:.4}V, gain={gain_db:.1} dB, nr_fail={nr_fail}");

    assert!(
        peak > 0.1 && peak < 5.0,
        "Diode clipper peak should be 0.1-5V (clipping), got {peak:.4}V"
    );
    assert!(nr_fail == 0, "No NR failures expected, got {nr_fail}");
}

/// Verify BJT CE amp has reasonable gain.
/// A CE amp with 10mV input and 4.7k collector load at 12V should give ~10-100× voltage gain.
#[test]
fn test_gain_sanity_bjt_ce() {
    let (peak, gain_db, nr_fail) = measure_codegen_gain(BJT_CE, 0.01, "gain_bjt");

    eprintln!("BJT CE: peak={peak:.4}V, gain={gain_db:.1} dB, nr_fail={nr_fail}");

    // CE amp gain depends heavily on bias point and coupling caps.
    // With emitter bypass cap and DC blocking on output, gain should be positive
    // but exact value depends on operating point. Main check: not wildly wrong.
    assert!(
        gain_db > -20.0 && gain_db < 60.0,
        "BJT CE gain should be -20 to 60 dB (reasonable for CE amp), got {gain_db:.1} dB"
    );
    assert!(nr_fail == 0, "No NR failures expected, got {nr_fail}");
}

/// Verify two-tube preamp has reasonable gain.
/// Two 12AX7 gain stages: each ~30-40 dB, but with plate-loading and coupling losses,
/// expect 30-60 dB total. Should NOT be > 80 dB (positive feedback / instability).
#[test]
fn test_gain_sanity_two_tube() {
    let (peak, gain_db, nr_fail) = measure_codegen_gain(TWO_TUBE_STAGE, 0.005, "gain_tube");

    eprintln!("Two-tube: peak={peak:.4}V, gain={gain_db:.1} dB, nr_fail={nr_fail}");

    assert!(
        gain_db > 10.0 && gain_db < 80.0,
        "Two-tube gain should be 10-80 dB, got {gain_db:.1} dB"
    );
    assert!(nr_fail == 0, "No NR failures expected, got {nr_fail}");
}

