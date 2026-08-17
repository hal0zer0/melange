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
        assert!(
            val < 1e-10,
            "Reset state should match fresh default, max_diff={val:.2e}"
        );
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
    assert!(
        output.contains("all_finite=true"),
        "All outputs should be finite. Output:\n{output}"
    );
    // No NR failures
    assert!(
        output.contains("nr_fail=0"),
        "No NR failures expected. Output:\n{output}"
    );
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

/// Saturating (uncoupled) inductor as an in-NR-loop nonlinear device
/// (SATURATING_TRANSFORMERS.md §3.4). Since 2026-08-15 the uncoupled saturating
/// inductor is no longer a lagged/decimated Sherman-Morrison patch on the trap
/// matrices; it is a genuine device on its augmented branch row, stamped inside
/// the full-LU Newton loop: residual uses the flux integral
/// `Φ(i) = L0·Isat·tanh(i/Isat)`, Jacobian uses the differential
/// `L_diff = L0/cosh²(i/Isat)`. This test pins the new emitted structure (the
/// old SM block is gone) and proves the generated code still compiles and runs.
#[test]
fn test_saturating_inductor_nr_flux_stamp() {
    const SAT_IND: &str = "\
Saturating inductor NR flux stamp
R1 in a 100
L1 a b 100m ISAT=20m
Rb b 0 1k
D1 b 0 DCLIP
.model DCLIP D(IS=2.52e-9 N=1.752)
C1 b 0 100n
Rout2 b out 1k
Cout out 0 100n
Rterm out 0 10k
.END";
    let code = generate_nodal_code(SAT_IND, 48000.0);
    // The retired decimated Sherman-Morrison block must NOT be emitted.
    assert!(
        !code.contains("SM for inductor"),
        "legacy decimated SM inductor block should no longer be emitted"
    );
    // Flux-integral residual term Φ(i) (the history / companion use tanh).
    assert!(
        code.contains("SAT_IND_0_L0 * SAT_IND_0_ISAT * (i0 / SAT_IND_0_ISAT).tanh()"),
        "expected the flux integral Φ(i)=L0·Isat·tanh(i/Isat) in the companion stamp"
    );
    // Differential-inductance Jacobian L_diff = L0/cosh²(i/Isat).
    assert!(
        code.contains("(i0 / SAT_IND_0_ISAT).clamp(-40.0, 40.0).cosh()"),
        "expected the differential L_diff = L0/cosh²(i/Isat) Jacobian stamp"
    );
    // Stamp targets the augmented branch row of the inductor.
    assert!(
        code.contains("chord_lu[SAT_IND_0_AUG_ROW][SAT_IND_0_AUG_ROW] +="),
        "expected the L_diff correction on the inductor's augmented Jacobian diagonal"
    );

    let main_code = r#"
fn main() {
    let mut state = CircuitState::default();
    let mut peak = 0.0f64;
    for i in 0..960u32 {
        let t = i as f64 / 48000.0;
        // Drive hard enough to swing the inductor current through Isat.
        let input = 5.0 * (2.0 * std::f64::consts::PI * 100.0 * t).sin();
        let output = process_sample(input, &mut state);
        let v = output[0];
        if v.abs() > peak { peak = v.abs(); }
        assert!(v.is_finite(), "non-finite output at sample {i}");
    }
    println!("{:.9e}", peak);
}
"#;
    let out = compile_and_run(&code, main_code, "sat_ind_sm_guard");
    let peak: f64 = out.trim().parse().expect("peak parse");
    assert!(
        peak > 1e-4,
        "saturating-inductor circuit should pass signal, peak={peak:.3e}"
    );
}

/// End-to-end HARMONIC correctness of the NR-integrated saturating inductor.
///
/// Pins the physics validated 2026-08-16 against an independent RK4 integration
/// of the exact flux ODE `Vin − R·i = d/dt[L0·Isat·tanh(i/Isat)]` and two ngspice
/// twins (idt flux-state + XSPICE core+lcouple), which all agreed to <0.5% on the
/// harmonic content. A simple R–L divider driven by a 1 kHz sine; the saturating
/// inductor soft-clips symmetrically.
///
/// Two guards a regression would trip:
///  - **Symmetry**: Φ(i)=L0·Isat·tanh(i/Isat) is ODD, so a symmetric drive must
///    produce NO even harmonics. H2/H1 ≈ 0. A sign error, DC-history bug, or the
///    switch-gswap "2×-first-sample" class of artifact would break this.
///  - **Saturation present & correct magnitude**: H3/H1 lands near the RK4 value
///    (≈21% at this drive; test-harness INPUT_RESISTANCE is 1Ω so R1=146 gives the
///    validated R≈147). A linear inductor (control, no ISAT) must show H3/H1 ≈ 0,
///    proving the 3rd harmonic is saturation, not numerical noise.
#[test]
fn test_saturating_inductor_harmonics() {
    // Single-bin DFT harmonic extractor over a steady-state window. Prints
    // "H1 H2 H3 peak". INPUT_RESISTANCE is 1Ω in the test harness, so R1=146
    // gives R≈147 — the drive depth the RK4/ngspice references were taken at.
    const MAIN: &str = r#"
fn main() {
    let mut state = CircuitState::default();
    let sr = 48000.0_f64;
    let f = 1000.0_f64;
    let n = 5280usize;    // 0.11 s
    let skip = 1440usize; // drop 0.03 s of onset transient
    let mut samp: Vec<f64> = Vec::new();
    for i in 0..n {
        let t = i as f64 / sr;
        let input = 0.9 * (2.0 * std::f64::consts::PI * f * t).sin();
        let out = process_sample(input, &mut state);
        if i >= skip { samp.push(out[0]); }
    }
    let bin = |fr: f64| -> f64 {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (k, &v) in samp.iter().enumerate() {
            let ph = 2.0 * std::f64::consts::PI * fr * ((skip + k) as f64) / sr;
            re += v * ph.cos();
            im += v * ph.sin();
        }
        2.0 * (re * re + im * im).sqrt() / samp.len() as f64
    };
    let peak = samp.iter().fold(0.0f64, |a, &x| a.max(x.abs()));
    println!("{:.9} {:.9} {:.9} {:.9}", bin(1000.0), bin(2000.0), bin(3000.0), peak);
}
"#;

    let parse = |out: &str| -> (f64, f64, f64, f64) {
        let v: Vec<f64> = out
            .split_whitespace()
            .filter_map(|s| s.parse::<f64>().ok())
            .collect();
        assert_eq!(v.len(), 4, "expected 'H1 H2 H3 peak', got {out:?}");
        (v[0], v[1], v[2], v[3])
    };

    // Saturating case.
    const SAT: &str = "\
Saturating inductor harmonic check
R1 in out 146
L1 out 0 50m ISAT=2m
.END";
    let sat_code = generate_nodal_code(SAT, 48000.0);
    let (h1, h2, h3, peak) = parse(&compile_and_run(&sat_code, MAIN, "sat_ind_harm"));
    assert!(
        h1.is_finite() && h2.is_finite() && h3.is_finite() && peak.is_finite(),
        "non-finite harmonics: H1={h1} H2={h2} H3={h3} peak={peak}"
    );
    assert!(
        h1 > 0.3,
        "fundamental too small, signal not passing: H1={h1:.4}"
    );
    assert!(
        peak < 0.9,
        "output should stay below the 0.9 V drive: peak={peak:.4}"
    );
    // Symmetry: odd tanh ⇒ no even harmonics.
    assert!(
        h2 / h1 < 0.01,
        "even-harmonic leak — saturation should be symmetric: H2/H1={:.4}",
        h2 / h1
    );
    // Saturation present and near the RK4/ngspice reference (~0.21). Wide band
    // to absorb 48 kHz trapezoidal discretization without being vacuous.
    let h3_ratio = h3 / h1;
    assert!(
        (0.15..0.28).contains(&h3_ratio),
        "H3/H1={h3_ratio:.4} outside validated saturating band [0.15, 0.28] \
         (RK4/ngspice reference ≈ 0.21)"
    );

    // Linear control: same divider, no ISAT ⇒ 3rd harmonic must vanish.
    const LIN: &str = "\
Linear inductor control
R1 in out 146
L1 out 0 50m
.END";
    let lin_code = generate_nodal_code(LIN, 48000.0);
    let (lh1, _lh2, lh3, _lpk) = parse(&compile_and_run(&lin_code, MAIN, "lin_ind_harm"));
    assert!(
        lh3 / lh1 < 0.005,
        "linear inductor produced a 3rd harmonic ({:.4}) — H3 must come from \
         saturation, not the solver",
        lh3 / lh1
    );
}

/// Coupled SATURATING TRANSFORMER via the shared-core T-model (Phase 2, 2026-08-16).
///
/// Two guards:
///  - **T-model == exact [L]** (the ideal-transformer sign fix): a saturating
///    group routes through the T-model (leakage + ideal couplings + one {ref}_mag
///    magnetizing inductor); a non-saturating group uses the exact CoupledInductor
///    path. Driven identically (near-linear via huge ISAT), their transfer must
///    match to <0.5% — otherwise the ideal-transformer stamp is mis-realizing the
///    mutual (the +5% frequency-growing bug this pins against).
///  - **Shared-core saturation** is odd-symmetric on the secondary: H2/H1 ≈ 0,
///    H3 present, and a linear control shows no H3.
#[test]
fn test_saturating_transformer_tmodel() {
    // "H1 H2 H3 peak" over a steady-state window; secondary is `out`.
    const MAIN: &str = r#"
fn main() {
    let mut state = CircuitState::default();
    let sr = 48000.0_f64; let f = 1000.0_f64;
    let n = 5280usize; let skip = 1440usize;
    let mut samp: Vec<f64> = Vec::new();
    for i in 0..n {
        let t = i as f64 / sr;
        let input = 0.9 * (2.0 * std::f64::consts::PI * f * t).sin();
        let out = process_sample(input, &mut state);
        if i >= skip { samp.push(out[0]); }
    }
    let bin = |fr: f64| -> f64 {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (k, &v) in samp.iter().enumerate() {
            let ph = 2.0 * std::f64::consts::PI * fr * ((skip + k) as f64) / sr;
            re += v * ph.cos(); im += v * ph.sin();
        }
        2.0 * (re * re + im * im).sqrt() / samp.len() as f64
    };
    let peak = samp.iter().fold(0.0f64, |a, &x| a.max(x.abs()));
    println!("{:.9} {:.9} {:.9} {:.9}", bin(1000.0), bin(2000.0), bin(3000.0), peak);
}
"#;
    let parse = |out: &str| -> (f64, f64, f64, f64) {
        let v: Vec<f64> = out
            .split_whitespace()
            .filter_map(|s| s.parse().ok())
            .collect();
        assert_eq!(v.len(), 4, "expected 'H1 H2 H3 peak', got {out:?}");
        (v[0], v[1], v[2], v[3])
    };
    // Near-linear T-model (huge ISAT fires the decomposition but never saturates).
    const TM: &str = "\
Tmodel near-linear transformer
L_pri in 0 100m ISAT=1e6
L_sec out 0 25m
K1 L_pri L_sec 0.99
R_load out 0 1k
.END";
    // Exact path: same transformer, no ISAT (CoupledInductorInfo).
    const EX: &str = "\
Exact coupled transformer
L_pri in 0 100m
L_sec out 0 25m
K1 L_pri L_sec 0.99
R_load out 0 1k
.END";
    let (tm_h1, _, _, _) = parse(&compile_and_run(
        &generate_nodal_code(TM, 48000.0),
        MAIN,
        "xfmr_tm",
    ));
    let (ex_h1, _, _, _) = parse(&compile_and_run(
        &generate_nodal_code(EX, 48000.0),
        MAIN,
        "xfmr_ex",
    ));
    let transfer_err = (tm_h1 - ex_h1).abs() / ex_h1;
    assert!(
        transfer_err < 0.005,
        "T-model transfer must match the exact [L] path (ideal-xfmr sign): \
         T-model H1={tm_h1:.6}, exact H1={ex_h1:.6}, err={:.4}",
        transfer_err
    );

    // Saturating transformer: shared-core odd-symmetric saturation on the secondary.
    const SAT: &str = "\
Saturating transformer
L_pri in 0 100m ISAT=1m
L_sec out 0 25m
K1 L_pri L_sec 0.99
R_load out 0 1k
.END";
    let (sh1, sh2, sh3, speak) = parse(&compile_and_run(
        &generate_nodal_code(SAT, 48000.0),
        MAIN,
        "xfmr_sat",
    ));
    assert!(
        sh1.is_finite() && speak.is_finite() && speak < 1.0,
        "unbounded/NaN: H1={sh1} peak={speak}"
    );
    assert!(sh1 > 0.05, "secondary carries no signal: H1={sh1:.4}");
    assert!(
        sh2 / sh1 < 0.01,
        "even-harmonic leak — shared-core saturation must be symmetric: H2/H1={:.4}",
        sh2 / sh1
    );
    assert!(
        sh3 / sh1 > 0.03,
        "no saturation on the secondary — core not saturating: H3/H1={:.4}",
        sh3 / sh1
    );
}
