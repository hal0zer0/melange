//! CLI integration tests — invoke the melange binary and verify behavior.
//!
//! These tests use inline synthetic circuits to test CLI commands
//! without depending on external circuit files.

use std::path::PathBuf;
use std::process::Command;

/// Minimal RC lowpass for testing CLI commands (linear, no nonlinear devices).
const TEST_RC_LOWPASS: &str = "\
RC Lowpass Test Fixture
R1 in out 10k
C1 out 0 10n
";

/// Minimal diode clipper for testing nonlinear CLI paths.
const TEST_DIODE_CLIPPER: &str = "\
Diode Clipper Test Fixture
R1 in out 4.7k
D1 out 0 1N4148
D2 0 out 1N4148
C1 out 0 100n
.model 1N4148 D(IS=2.52e-9 N=1.752 BV=100 IBV=100u)
";

/// Write a test circuit to a temp file and return the path.
fn write_test_circuit(content: &str, name: &str) -> PathBuf {
    let path = std::env::temp_dir().join(format!("melange_cli_test_{}.cir", name));
    std::fs::write(&path, content).expect("write test circuit");
    path
}

/// Get the path to the built melange binary.
fn melange_bin() -> PathBuf {
    // cargo test builds the binary in target/debug/
    let mut path = std::env::current_exe()
        .unwrap()
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf();
    path.push("melange");
    path
}

/// Get the project root (workspace root).
fn project_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

/// Run melange with args, assert success, return stdout.
fn run_melange(args: &[&str]) -> String {
    let output = Command::new(melange_bin())
        .args(args)
        .current_dir(project_root())
        .output()
        .expect("failed to run melange binary");

    if !output.status.success() {
        panic!(
            "melange {} failed (exit {}):\nstdout: {}\nstderr: {}",
            args.join(" "),
            output.status,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr),
        );
    }
    String::from_utf8_lossy(&output.stdout).to_string()
}

/// Run melange expecting failure, return stderr.
fn run_melange_fail(args: &[&str]) -> String {
    let output = Command::new(melange_bin())
        .args(args)
        .current_dir(project_root())
        .output()
        .expect("failed to run melange binary");

    assert!(
        !output.status.success(),
        "melange {} should have failed but succeeded:\n{}",
        args.join(" "),
        String::from_utf8_lossy(&output.stdout),
    );
    String::from_utf8_lossy(&output.stderr).to_string()
}

// ============================================================================
// builtins command
// ============================================================================

#[test]
fn test_builtins_lists_circuits() {
    let stdout = run_melange(&["builtins"]);
    // Builtins migrated to melange-audio/circuits repo — list is now empty
    assert!(
        stdout.contains("Available builtin circuits") || stdout.contains("No builtin"),
        "Should have header or empty message"
    );
}

// ============================================================================
// nodes command
// ============================================================================

#[test]
fn test_nodes_circuit_file() {
    let cir = write_test_circuit(TEST_RC_LOWPASS, "nodes");
    let stdout = run_melange(&["nodes", cir.to_str().unwrap()]);
    assert!(
        stdout.contains("in") || stdout.contains("out"),
        "Should list circuit nodes, got: {}",
        stdout
    );
    let _ = std::fs::remove_file(&cir);
}

// ============================================================================
// compile command
// ============================================================================

#[test]
fn test_compile_circuit_file() {
    let cir = write_test_circuit(TEST_DIODE_CLIPPER, "compile");
    let tmp = std::env::temp_dir().join("melange_cli_test_compile.rs");
    let tmp_str = tmp.to_str().unwrap();

    let stdout = run_melange(&[
        "compile",
        cir.to_str().unwrap(),
        "--output",
        tmp_str,
        "--input-node",
        "in",
        "--output-node",
        "out",
    ]);

    assert!(
        stdout.contains("Done") || stdout.contains("Generated"),
        "Should indicate success: {}",
        stdout
    );

    // Verify output file exists and contains Rust code
    let code = std::fs::read_to_string(&tmp).expect("should read generated file");
    assert!(
        code.contains("process_sample"),
        "Generated code should contain process_sample function"
    );
    assert!(
        code.contains("CircuitState"),
        "Generated code should contain CircuitState struct"
    );

    let _ = std::fs::remove_file(&tmp);
    let _ = std::fs::remove_file(&cir);
}

#[test]
fn test_compile_produces_compilable_rust() {
    let cir = write_test_circuit(TEST_DIODE_CLIPPER, "compiles");
    let tmp_rs = std::env::temp_dir().join("melange_cli_test_compiles.rs");
    let tmp_rlib = std::env::temp_dir().join("melange_cli_test_compiles.rlib");

    run_melange(&[
        "compile",
        cir.to_str().unwrap(),
        "--output",
        tmp_rs.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "out",
    ]);

    // Compile the generated Rust code as a library
    let compile = Command::new("rustc")
        .arg(&tmp_rs)
        .arg("--edition=2024")
        .arg("--crate-type=lib")
        .arg("-o")
        .arg(&tmp_rlib)
        .output()
        .expect("rustc");

    let _ = std::fs::remove_file(&tmp_rs);
    let _ = std::fs::remove_file(&tmp_rlib);
    let _ = std::fs::remove_file(&cir);

    assert!(
        compile.status.success(),
        "Generated code should compile:\n{}",
        String::from_utf8_lossy(&compile.stderr)
    );
}

// ============================================================================
// simulate command
// ============================================================================

#[test]
fn test_simulate_sine_tone() {
    let cir = write_test_circuit(TEST_RC_LOWPASS, "sim");
    let tmp_wav = std::env::temp_dir().join("melange_cli_test_sim.wav");

    run_melange(&[
        "simulate",
        cir.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "out",
        "--amplitude",
        "0.1",
        "--output",
        tmp_wav.to_str().unwrap(),
        "--duration",
        "0.1",
    ]);

    // Verify WAV file exists and has content
    let metadata = std::fs::metadata(&tmp_wav).expect("WAV file should exist");
    assert!(
        metadata.len() > 100,
        "WAV file should have content, got {} bytes",
        metadata.len()
    );

    let _ = std::fs::remove_file(&tmp_wav);
    let _ = std::fs::remove_file(&cir);
}

#[test]
fn test_analyze_integral_amplitude() {
    // Regression: `--amplitude 1.0` used to emit `let amplitude = 1;` in the
    // generated analyze main (f64 Display drops the trailing `.0`), producing
    // an integer literal that fails rustc with E0277 (cannot multiply
    // {integer} by f64). Reported by the velvet-elvis agent 2026-06-10.
    let cir = write_test_circuit(TEST_RC_LOWPASS, "analyze_amp");

    let stdout = run_melange(&[
        "analyze",
        cir.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "out",
        "--amplitude",
        "1.0",
        "--start-freq",
        "1000",
        "--end-freq",
        "2000",
        "--points-per-decade",
        "1",
    ]);

    assert!(
        stdout.contains("frequency_hz,gain_db,phase_deg"),
        "expected analyze CSV header, got:\n{stdout}"
    );

    let _ = std::fs::remove_file(&cir);
}

/// Read the raw PCM payload (bytes after the `data` chunk header) of a WAV
/// file written by melange's embedded writer. Byte-level compare is enough —
/// this test only needs "did the output change," not decoded sample values.
fn wav_pcm_bytes(path: &std::path::Path) -> Vec<u8> {
    let data = std::fs::read(path).expect("read wav");
    let idx = data
        .windows(4)
        .position(|w| w == b"data")
        .expect("wav data chunk");
    data[idx + 8..].to_vec()
}

#[test]
fn test_simulate_noise_full_actually_injects_noise() {
    // Regression: `simulate --noise <mode>` used to compile in the full
    // noise machinery but leave the runtime `noise_enabled` master switch at
    // its default (false), so output was byte-identical and seed-invariant
    // to `--noise off` — a silent no-op with no warning. Reported by
    // melange-circuits 2026-07-25
    // (local-docs/melange-bug-simulate-noise-noop-2026-07-25.md in their
    // repo). Fixed by emitting `state.set_noise_enabled(true)` in the
    // generated `main()` when the caller passed `--noise <mode>` — passing
    // the flag IS the opt-in for `simulate`/`analyze`, unlike the plugin
    // seam where noise defaults off until the host UI enables it.
    let cir = write_test_circuit(TEST_RC_LOWPASS, "noise_noop");
    let wav_off = std::env::temp_dir().join("melange_cli_test_noise_off.wav");
    let wav_full_a = std::env::temp_dir().join("melange_cli_test_noise_full_a.wav");
    let wav_full_b = std::env::temp_dir().join("melange_cli_test_noise_full_b.wav");

    let base_args = |out: &std::path::Path, extra: &[&str]| -> Vec<String> {
        let mut args = vec![
            "simulate".to_string(),
            cir.to_str().unwrap().to_string(),
            "--input-node".to_string(),
            "in".to_string(),
            "--output-node".to_string(),
            "out".to_string(),
            "--amplitude".to_string(),
            "0.0".to_string(),
            "--duration".to_string(),
            "0.1".to_string(),
            "--output".to_string(),
            out.to_str().unwrap().to_string(),
        ];
        args.extend(extra.iter().map(|s| s.to_string()));
        args
    };

    run_melange(&base_args(&wav_off, &["--noise", "off"])
        .iter()
        .map(String::as_str)
        .collect::<Vec<_>>());
    run_melange(
        &base_args(&wav_full_a, &["--noise", "full", "--noise-seed", "12345"])
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>(),
    );
    run_melange(
        &base_args(&wav_full_b, &["--noise", "full", "--noise-seed", "999"])
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>(),
    );

    let off = wav_pcm_bytes(&wav_off);
    let full_a = wav_pcm_bytes(&wav_full_a);
    let full_b = wav_pcm_bytes(&wav_full_b);

    assert_ne!(
        off, full_a,
        "--noise full must differ from --noise off (silent no-op regression)"
    );
    assert_ne!(
        full_a, full_b,
        "different --noise-seed values must produce different output \
         (seed-invariance means the RNG master switch is still off)"
    );

    let _ = std::fs::remove_file(&cir);
    let _ = std::fs::remove_file(&wav_off);
    let _ = std::fs::remove_file(&wav_full_a);
    let _ = std::fs::remove_file(&wav_full_b);
}

#[test]
fn test_analyze_linear_circuit_harmonics_at_numerical_floor() {
    // Regression for the DFT leakage bug: the measurement window rounded to N
    // samples while the drive frequency stayed exact, leaving up to 0.5
    // samples of cycle mismatch. The rectangular-window leakage floored
    // h2..hN at −50..−75 dBc even on a perfectly linear circuit. With the
    // drive snapped to the exact bin DFT_CYCLES·sr/N, a linear RC must show
    // every harmonic at the numerical floor (below −120 dBc) or nan
    // (above Nyquist).
    //
    // Frequencies are chosen so DFT_CYCLES·sr/f is NOT an integer — the
    // pre-fix leakage regime.
    let cir = write_test_circuit(TEST_RC_LOWPASS, "analyze_leakage");

    let stdout = run_melange(&[
        "analyze",
        cir.to_str().unwrap(),
        "--input-node",
        "in",
        "--output-node",
        "out",
        "--amplitude",
        "0.5",
        "--start-freq",
        "313",
        "--end-freq",
        "7919",
        "--points-per-decade",
        "2",
        "--harmonics",
        "5",
    ]);
    let _ = std::fs::remove_file(&cir);

    // CSV: frequency_hz,gain_db,phase_deg,thd_pct,h2_dbc..h5_dbc,nyquist_dbc
    let mut data_rows = 0;
    for line in stdout
        .lines()
        .skip_while(|l| !l.starts_with("frequency_hz"))
    {
        if line.starts_with("frequency_hz") || line.trim().is_empty() {
            continue;
        }
        let cols: Vec<&str> = line.split(',').collect();
        assert!(
            cols.len() >= 9,
            "expected >= 9 CSV columns with --harmonics 5, got {}: {line}",
            cols.len()
        );
        data_rows += 1;
        for (i, col) in cols[4..8].iter().enumerate() {
            if *col == "nan" {
                continue; // harmonic above Nyquist
            }
            let dbc: f64 = col
                .parse()
                .unwrap_or_else(|_| panic!("unparseable h{}_dbc {col:?} in {line}", i + 2));
            assert!(
                dbc < -120.0,
                "h{}_dbc = {dbc} dBc on a pure linear RC — DFT leakage floor is back \
                 (was −50..−75 dBc before the bin snap). Row: {line}",
                i + 2
            );
        }
    }
    assert!(data_rows >= 3, "expected >= 3 data rows, got:\n{stdout}");
}

// ============================================================================
// error cases
// ============================================================================

#[test]
fn test_compile_mono_with_multiple_outputs_errors() {
    // --mono + multiple output nodes cannot be represented (multi-output
    // plugins route one node per channel) and used to generate a broken
    // plugin silently.
    let cir = write_test_circuit(
        "Two Output Test\nR1 in out 10k\nC1 out 0 10n\nR2 out out2 1k\nC2 out2 0 10n\n",
        "mono_multi",
    );
    let stderr = run_melange_fail(&[
        "compile",
        cir.to_str().unwrap(),
        "--output",
        "/tmp/melange_cli_mono_multi.rs",
        "--input-node",
        "in",
        "--output-node",
        "out,out2",
        "--mono",
    ]);
    let _ = std::fs::remove_file(&cir);
    assert!(
        stderr.contains("--mono") && stderr.contains("output node"),
        "should explain the --mono / multi-output conflict: {stderr}"
    );
}

#[test]
fn test_compile_missing_file() {
    let stderr = run_melange_fail(&[
        "compile",
        "nonexistent_circuit.cir",
        "--output",
        "/tmp/melange_cli_fail.rs",
        "--input-node",
        "in",
        "--output-node",
        "out",
    ]);
    assert!(
        stderr.contains("not found")
            || stderr.contains("No such file")
            || stderr.contains("error")
            || stderr.contains("Error"),
        "Should report missing file error: {}",
        stderr
    );
}

#[test]
fn test_compile_missing_node() {
    let cir = write_test_circuit(TEST_RC_LOWPASS, "missing_node");
    let stderr = run_melange_fail(&[
        "compile",
        cir.to_str().unwrap(),
        "--output",
        "/tmp/melange_cli_fail.rs",
        "--input-node",
        "nonexistent_node",
        "--output-node",
        "out",
    ]);
    let _ = std::fs::remove_file(&cir);
    assert!(
        stderr.contains("node")
            || stderr.contains("Node")
            || stderr.contains("error")
            || stderr.contains("Error"),
        "Should report missing node error: {}",
        stderr
    );
}
