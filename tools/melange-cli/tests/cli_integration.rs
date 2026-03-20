//! CLI integration tests — invoke the melange binary and verify behavior.
//!
//! These tests build and run the CLI binary against real circuit files,
//! verifying end-to-end pipeline correctness.

use std::path::PathBuf;
use std::process::Command;

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
    assert!(stdout.contains("rc-lowpass"), "Should list rc-lowpass builtin");
    assert!(
        stdout.contains("Available builtin circuits"),
        "Should have header"
    );
}

// ============================================================================
// nodes command
// ============================================================================

#[test]
fn test_nodes_circuit_file() {
    let stdout = run_melange(&["nodes", "circuits/wurli-preamp.cir"]);
    // Should list node names from the circuit
    assert!(
        stdout.contains("out") || stdout.contains("base") || stdout.contains("coll"),
        "Should list circuit nodes, got: {}",
        stdout
    );
}

// ============================================================================
// compile command
// ============================================================================

#[test]
fn test_compile_circuit_file() {
    let tmp = std::env::temp_dir().join("melange_cli_test_compile.rs");
    let tmp_str = tmp.to_str().unwrap();

    let stdout = run_melange(&[
        "compile",
        "circuits/wurli-preamp.cir",
        "--output",
        tmp_str,
        "--input-node",
        "mid_in",
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
}

#[test]
fn test_compile_produces_compilable_rust() {
    let tmp_rs = std::env::temp_dir().join("melange_cli_test_compiles.rs");
    let tmp_rlib = std::env::temp_dir().join("melange_cli_test_compiles.rlib");

    run_melange(&[
        "compile",
        "circuits/tweed-preamp.cir",
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
    let tmp_wav = std::env::temp_dir().join("melange_cli_test_sim.wav");

    run_melange(&[
        "simulate",
        "circuits/wurli-preamp.cir",
        "--input-node",
        "mid_in",
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
    let stderr = run_melange_fail(&[
        "compile",
        "circuits/wurli-preamp.cir",
        "--output",
        "/tmp/melange_cli_fail.rs",
        "--input-node",
        "nonexistent_node",
        "--output-node",
        "out",
    ]);
    assert!(
        stderr.contains("node")
            || stderr.contains("Node")
            || stderr.contains("error")
            || stderr.contains("Error"),
        "Should report missing node error: {}",
        stderr
    );
}
