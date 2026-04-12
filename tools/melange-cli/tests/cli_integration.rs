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

// ============================================================================
// error cases
// ============================================================================

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
