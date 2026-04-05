//! Shared test harness for compile-and-run codegen tests.
//!
//! Provides centralized compilation, caching, and execution infrastructure
//! so that individual test files don't need to duplicate boilerplate.
//!
//! ## Two-tier API
//!
//! **Low-level** — for tests needing custom `main()` functions:
//! ```ignore
//! let output = support::compile_and_run(&code, &main_code, "my_test");
//! let samples = output.parse_samples();
//! ```
//!
//! **High-level** — for standard circuit tests (cached compilation):
//! ```ignore
//! let circuit = support::build_circuit(SPICE, &config, "diode_clip");
//! let samples = support::run_sine(&circuit, 1000.0, 0.5, 4410, 44100.0);
//! ```

#![allow(dead_code)]

use std::collections::HashMap;
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;
use std::sync::{LazyLock, Mutex};

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ── Compilation cache ──────────────────────────────────────────────────

/// Global compilation cache: source code hash → binary path on disk.
/// Tests sharing the same generated circuit code compile once.
static BINARY_CACHE: LazyLock<Mutex<HashMap<u64, PathBuf>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// Atomic counter for unique temp file names (thread-safe).
static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

fn next_id() -> u32 {
    COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst)
}

/// Hash source code for cache key. Not cryptographic — just needs uniqueness.
fn hash_source(source: &str) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    source.hash(&mut hasher);
    hasher.finish()
}

// ── Output types ───────────────────────────────────────────────────────

/// Output from running a compiled circuit binary.
#[derive(Debug, Clone)]
pub struct RunOutput {
    pub stdout: String,
    pub stderr: String,
}

impl RunOutput {
    /// Parse stdout lines as f64 samples.
    pub fn parse_samples(&self) -> Vec<f64> {
        self.stdout
            .lines()
            .filter_map(|l| l.trim().parse::<f64>().ok())
            .collect()
    }

    /// Parse a key=value pair from stdout (e.g., "peak=1.234").
    pub fn parse_kv(&self, key: &str) -> Option<f64> {
        let prefix = format!("{key}=");
        self.stdout
            .lines()
            .chain(self.stderr.lines())
            .find(|l| l.contains(&prefix))
            .and_then(|l| {
                l.split(&prefix)
                    .nth(1)
                    .and_then(|v| v.trim().parse().ok())
            })
    }

    /// Parse a DIAG:key=value pair from stderr.
    pub fn diag(&self, key: &str) -> Option<f64> {
        let prefix = format!("DIAG:{key}=");
        self.stderr
            .lines()
            .find(|l| l.starts_with(&prefix))
            .and_then(|l| l[prefix.len()..].trim().parse().ok())
    }
}

/// A compiled circuit binary ready to run with different inputs.
pub struct CompiledCircuit {
    /// Path to the compiled binary on disk.
    pub binary_path: PathBuf,
    /// The generated circuit code (without main), for recompilation with custom mains.
    pub code: String,
    /// Number of circuit nodes (from codegen).
    pub n: usize,
    /// Nonlinear dimension (from codegen).
    pub m: usize,
}

// ── Low-level: compile_and_run ─────────────────────────────────────────

/// Compile circuit code + custom main, run the binary, return output.
///
/// This is the general-purpose function for tests that need full control
/// over the `main()` function. For standard tests, prefer `build_circuit()`
/// + `run_sine()` which caches the compilation.
///
/// # Arguments
/// * `code` — Generated circuit code (struct, process_sample, etc.)
/// * `main_code` — Custom `fn main() { ... }` source
/// * `tag` — Short identifier for temp file naming
pub fn compile_and_run(code: &str, main_code: &str, tag: &str) -> RunOutput {
    let full_source = format!("{code}\n\n{main_code}\n");
    compile_source_and_run(&full_source, tag, &[])
}

/// Compile full source (with main) and run, passing optional argv to the binary.
fn compile_source_and_run(source: &str, tag: &str, args: &[&str]) -> RunOutput {
    let tmp_dir = std::env::temp_dir();
    let pid = std::process::id();
    let id = next_id();
    let src_path = tmp_dir.join(format!("melange_test_{tag}_{pid}_{id}.rs"));
    let bin_path = tmp_dir.join(format!("melange_test_{tag}_{pid}_{id}"));

    // Write source
    {
        let mut f = std::fs::File::create(&src_path).expect("create source file");
        f.write_all(source.as_bytes()).expect("write source");
    }

    // Compile with optimization (debug builds are 10-50x slower)
    let compile = Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2024")
        .arg("-O")
        .output()
        .expect("failed to invoke rustc");

    let _ = std::fs::remove_file(&src_path);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        panic!(
            "Compilation failed for '{tag}':\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    // Run
    let run = Command::new(&bin_path)
        .args(args)
        .output()
        .expect("failed to run binary");

    let _ = std::fs::remove_file(&bin_path);

    if !run.status.success() {
        panic!(
            "Binary execution failed for '{tag}':\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }

    RunOutput {
        stdout: String::from_utf8_lossy(&run.stdout).to_string(),
        stderr: String::from_utf8_lossy(&run.stderr).to_string(),
    }
}

/// Compile full source with main, run the binary, and pipe input data via stdin.
fn compile_and_run_with_stdin(source: &str, tag: &str, args: &[&str], stdin_data: &str) -> RunOutput {
    let tmp_dir = std::env::temp_dir();
    let pid = std::process::id();
    let id = next_id();
    let src_path = tmp_dir.join(format!("melange_test_{tag}_{pid}_{id}.rs"));
    let bin_path = tmp_dir.join(format!("melange_test_{tag}_{pid}_{id}"));

    {
        let mut f = std::fs::File::create(&src_path).expect("create source file");
        f.write_all(source.as_bytes()).expect("write source");
    }

    let compile = Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2024")
        .arg("-O")
        .output()
        .expect("failed to invoke rustc");

    let _ = std::fs::remove_file(&src_path);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        panic!(
            "Compilation failed for '{tag}':\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let mut child = Command::new(&bin_path)
        .args(args)
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("failed to spawn binary");

    if let Some(mut stdin) = child.stdin.take() {
        stdin.write_all(stdin_data.as_bytes()).expect("write stdin");
    }

    let output = child.wait_with_output().expect("wait for binary");
    let _ = std::fs::remove_file(&bin_path);

    if !output.status.success() {
        panic!(
            "Binary execution failed for '{tag}':\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    RunOutput {
        stdout: String::from_utf8_lossy(&output.stdout).to_string(),
        stderr: String::from_utf8_lossy(&output.stderr).to_string(),
    }
}

// ── High-level: build_circuit + run ────────────────────────────────────

/// Standard test main template.
///
/// The generated binary supports three modes via argv:
/// - `binary sine <freq> <amplitude> <num_samples> <sample_rate>`
/// - `binary step <amplitude> <num_samples> <sample_rate>`
/// - `binary signal <sample_rate>` (reads samples from stdin, one per line)
///
/// Output: one f64 per line on stdout (scientific notation).
/// Diagnostics: DIAG:key=value on stderr.
const STANDARD_MAIN: &str = r#"
fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: binary <mode> [args...]");
        eprintln!("Modes: sine <freq> <amp> <n> <sr>, step <amp> <n> <sr>, signal <sr>");
        std::process::exit(1);
    }

    // CircuitState::default() handles DC OP initialization (DK path)
    // or warmup (nodal path) automatically — no explicit warmup needed.
    let mut state = CircuitState::default();

    let mode = args[1].as_str();
    match mode {
        "sine" => {
            let freq: f64 = args[2].parse().unwrap();
            let amplitude: f64 = args[3].parse().unwrap();
            let num_samples: usize = args[4].parse().unwrap();
            let sr: f64 = args[5].parse().unwrap();
            state.set_sample_rate(sr);
            for i in 0..num_samples {
                let t = i as f64 / sr;
                let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
                let out = process_sample(input, &mut state);
                println!("{:.15e}", out[0]);
            }
        }
        "step" => {
            let amplitude: f64 = args[2].parse().unwrap();
            let num_samples: usize = args[3].parse().unwrap();
            let sr: f64 = args[4].parse().unwrap();
            state.set_sample_rate(sr);
            for _ in 0..num_samples {
                let out = process_sample(amplitude, &mut state);
                println!("{:.15e}", out[0]);
            }
        }
        "signal" => {
            let sr: f64 = args[2].parse().unwrap();
            state.set_sample_rate(sr);
            let stdin = std::io::stdin();
            let mut line = String::new();
            loop {
                line.clear();
                if stdin.read_line(&mut line).unwrap() == 0 { break; }
                if let Ok(input) = line.trim().parse::<f64>() {
                    let out = process_sample(input, &mut state);
                    println!("{:.15e}", out[0]);
                }
            }
        }
        _ => {
            eprintln!("Unknown mode: {mode}");
            std::process::exit(1);
        }
    }
    eprintln!("DIAG:nr_max_iter_count={}", state.diag_nr_max_iter_count);
    eprintln!("DIAG:nan_reset_count={}", state.diag_nan_reset_count);
}
"#;

/// Build a circuit binary from SPICE source through the full codegen pipeline.
///
/// Pipeline: parse → MNA → stamp G_in → DK kernel → codegen → compile.
/// The compiled binary is cached — subsequent calls with the same circuit
/// and config return the cached binary without recompilation.
///
/// # Arguments
/// * `spice` — SPICE netlist source string
/// * `config` — Codegen configuration (sample rate, tolerances, etc.)
/// * `tag` — Short identifier for diagnostics and temp file naming
///
/// # Panics
/// Panics if any pipeline step fails (parse, MNA, DK, codegen, compile).
pub fn build_circuit(spice: &str, config: &CodegenConfig, tag: &str) -> CompiledCircuit {
    let (code, n, m) = generate_circuit_code(spice, config);
    let binary_path = compile_circuit_code(&code, tag);
    CompiledCircuit {
        binary_path,
        code,
        n,
        m,
    }
}

/// Build a circuit binary using the nodal solver codegen path.
///
/// Use this for circuits with inductors, transformers, or when the DK path
/// is unstable (K≈0, positive K diagonal, spectral radius > 1).
pub fn build_circuit_nodal(spice: &str, config: &CodegenConfig, tag: &str) -> CompiledCircuit {
    let (code, n, m) = generate_circuit_code_nodal(spice, config);
    let binary_path = compile_circuit_code(&code, tag);
    CompiledCircuit {
        binary_path,
        code,
        n,
        m,
    }
}

/// Generate circuit code through the DK codegen pipeline (no compilation).
///
/// Returns (code, n, m). Useful when you need the code for a custom main.
pub fn generate_circuit_code(spice: &str, config: &CodegenConfig) -> (String, usize, usize) {
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA build failed");

    let input_node = config.input_node;
    let input_conductance = 1.0 / config.input_resistance;
    if input_node < mna.n {
        mna.g[input_node][input_node] += input_conductance;
    }

    let kernel = DkKernel::from_mna(&mna, config.sample_rate).expect("DK kernel build failed");
    let generator = CodeGenerator::new(config.clone());
    let result = generator
        .generate(&kernel, &mna, &netlist)
        .expect("codegen failed");

    (result.code, result.n, result.m)
}

/// Generate circuit code through the nodal codegen pipeline (no compilation).
pub fn generate_circuit_code_nodal(spice: &str, config: &CodegenConfig) -> (String, usize, usize) {
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("MNA build failed");

    let input_node = config.input_node;
    let input_conductance = 1.0 / config.input_resistance;
    if input_node < mna.n {
        mna.g[input_node][input_node] += input_conductance;
    }

    let generator = CodeGenerator::new(config.clone());
    let result = generator
        .generate_nodal(&mna, &netlist)
        .expect("nodal codegen failed");

    (result.code, result.n, result.m)
}

/// Compile circuit code into a cached binary. Returns the binary path.
///
/// The standard test main is appended to the circuit code. The resulting
/// binary supports sine/step/signal modes via argv.
fn compile_circuit_code(code: &str, tag: &str) -> PathBuf {
    let full_source = format!("{code}\n{STANDARD_MAIN}");
    let key = hash_source(&full_source);

    // Check cache
    {
        let cache = BINARY_CACHE.lock().unwrap();
        if let Some(path) = cache.get(&key) {
            if path.exists() {
                return path.clone();
            }
        }
    }

    // Compile
    let tmp_dir = std::env::temp_dir();
    let pid = std::process::id();
    let id = next_id();
    let src_path = tmp_dir.join(format!("melange_cached_{tag}_{pid}_{id}.rs"));
    let bin_path = tmp_dir.join(format!("melange_cached_{tag}_{pid}_{id}"));

    {
        let mut f = std::fs::File::create(&src_path).expect("create source file");
        f.write_all(full_source.as_bytes()).expect("write source");
    }

    let compile = Command::new("rustc")
        .arg(&src_path)
        .arg("-o")
        .arg(&bin_path)
        .arg("--edition=2024")
        .arg("-O")
        .output()
        .expect("failed to invoke rustc");

    let _ = std::fs::remove_file(&src_path);

    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin_path);
        panic!(
            "Compilation failed for '{tag}':\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    // Store in cache
    {
        let mut cache = BINARY_CACHE.lock().unwrap();
        cache.insert(key, bin_path.clone());
    }

    bin_path
}

// ── Run methods for CompiledCircuit ────────────────────────────────────

/// Run a cached circuit binary with a sine input.
pub fn run_sine(
    circuit: &CompiledCircuit,
    freq: f64,
    amplitude: f64,
    num_samples: usize,
    sample_rate: f64,
) -> Vec<f64> {
    let output = Command::new(&circuit.binary_path)
        .args([
            "sine",
            &freq.to_string(),
            &amplitude.to_string(),
            &num_samples.to_string(),
            &sample_rate.to_string(),
        ])
        .output()
        .expect("failed to run circuit binary");

    if !output.status.success() {
        panic!(
            "Circuit binary failed (sine {freq}Hz):\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|l| l.trim().parse::<f64>().ok())
        .collect()
}

/// Run a cached circuit binary with a DC step input.
pub fn run_step(
    circuit: &CompiledCircuit,
    amplitude: f64,
    num_samples: usize,
    sample_rate: f64,
) -> Vec<f64> {
    let output = Command::new(&circuit.binary_path)
        .args([
            "step",
            &amplitude.to_string(),
            &num_samples.to_string(),
            &sample_rate.to_string(),
        ])
        .output()
        .expect("failed to run circuit binary");

    if !output.status.success() {
        panic!(
            "Circuit binary failed (step {amplitude}V):\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|l| l.trim().parse::<f64>().ok())
        .collect()
}

/// Run a cached circuit binary with a DC step input and return full output (including diagnostics).
pub fn run_step_full(
    circuit: &CompiledCircuit,
    amplitude: f64,
    num_samples: usize,
    sample_rate: f64,
) -> RunOutput {
    let output = Command::new(&circuit.binary_path)
        .args([
            "step",
            &amplitude.to_string(),
            &num_samples.to_string(),
            &sample_rate.to_string(),
        ])
        .output()
        .expect("failed to run circuit binary");

    if !output.status.success() {
        panic!(
            "Circuit binary failed (step {amplitude}V):\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    RunOutput {
        stdout: String::from_utf8_lossy(&output.stdout).to_string(),
        stderr: String::from_utf8_lossy(&output.stderr).to_string(),
    }
}

/// Run a cached circuit binary with an arbitrary input signal.
///
/// Input samples are piped via stdin, one per line.
pub fn run_signal(
    circuit: &CompiledCircuit,
    input: &[f64],
    sample_rate: f64,
) -> Vec<f64> {
    let stdin_data: String = input.iter().map(|s| format!("{s:.15e}\n")).collect();

    let mut child = Command::new(&circuit.binary_path)
        .args(["signal", &sample_rate.to_string()])
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("failed to spawn circuit binary");

    if let Some(mut stdin) = child.stdin.take() {
        stdin.write_all(stdin_data.as_bytes()).expect("write stdin");
    }

    let output = child.wait_with_output().expect("wait for binary");

    if !output.status.success() {
        panic!(
            "Circuit binary failed (signal mode):\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|l| l.trim().parse::<f64>().ok())
        .collect()
}

/// Run a cached circuit binary with a sine input and return full output (including diagnostics).
pub fn run_sine_full(
    circuit: &CompiledCircuit,
    freq: f64,
    amplitude: f64,
    num_samples: usize,
    sample_rate: f64,
) -> RunOutput {
    let output = Command::new(&circuit.binary_path)
        .args([
            "sine",
            &freq.to_string(),
            &amplitude.to_string(),
            &num_samples.to_string(),
            &sample_rate.to_string(),
        ])
        .output()
        .expect("failed to run circuit binary");

    if !output.status.success() {
        panic!(
            "Circuit binary failed (sine {freq}Hz):\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    RunOutput {
        stdout: String::from_utf8_lossy(&output.stdout).to_string(),
        stderr: String::from_utf8_lossy(&output.stderr).to_string(),
    }
}

// ── Codegen config helpers ─────────────────────────────────────────────

/// Create a default codegen config for a circuit with "in" and "out" nodes.
///
/// Resolves input/output node indices from the SPICE netlist.
/// Uses sensible test defaults (44100 Hz, 1Ω input, dc_block on).
pub fn config_for_spice(spice: &str, sample_rate: f64) -> CodegenConfig {
    let netlist = Netlist::parse(spice).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");

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

    CodegenConfig {
        circuit_name: "test_circuit".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

// ── Assertion helpers ──────────────────────────────────────────────────

/// Assert all samples are finite (no NaN or Inf).
pub fn assert_finite(samples: &[f64]) {
    for (i, &s) in samples.iter().enumerate() {
        assert!(
            s.is_finite(),
            "sample[{i}] is not finite: {s} (total samples: {})",
            samples.len()
        );
    }
}

/// Assert all samples are within [min, max].
pub fn assert_bounded(samples: &[f64], min: f64, max: f64) {
    for (i, &s) in samples.iter().enumerate() {
        assert!(
            s >= min && s <= max,
            "sample[{i}] = {s} outside [{min}, {max}] (total samples: {})",
            samples.len()
        );
    }
}

/// Assert peak absolute value exceeds threshold (circuit is producing output).
pub fn assert_peak_above(samples: &[f64], threshold: f64) {
    let peak = samples.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    assert!(
        peak > threshold,
        "peak output {peak:.6e} below threshold {threshold:.6e} (circuit may not be producing signal)"
    );
}

/// Assert that two sample vectors match within tolerance (absolute).
pub fn assert_samples_match(a: &[f64], b: &[f64], tolerance: f64, label: &str) {
    assert_eq!(
        a.len(),
        b.len(),
        "{label}: length mismatch: {} vs {}",
        a.len(),
        b.len()
    );
    let mut max_diff = 0.0_f64;
    let mut max_idx = 0;
    for (i, (&va, &vb)) in a.iter().zip(b.iter()).enumerate() {
        let diff = (va - vb).abs();
        if diff > max_diff {
            max_diff = diff;
            max_idx = i;
        }
    }
    assert!(
        max_diff <= tolerance,
        "{label}: max diff {max_diff:.6e} at sample[{max_idx}] exceeds tolerance {tolerance:.6e} \
         (a={:.6e}, b={:.6e})",
        a[max_idx],
        b[max_idx]
    );
}

/// Assert codegen output matches a golden reference file.
///
/// Golden files are JSON with format:
/// ```json
/// {
///   "test_name": "...",
///   "output": [0.0, 0.0221, ...],
///   "tolerance": 1e-10
/// }
/// ```
pub fn assert_matches_golden(output: &[f64], golden_path: &str, tolerance: f64) {
    let golden_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden");
    let path = golden_dir.join(golden_path);

    let content = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("Failed to read golden file {}: {e}", path.display()));

    let json: serde_json::Value = serde_json::from_str(&content)
        .unwrap_or_else(|e| panic!("Failed to parse golden file {}: {e}", path.display()));

    let golden: Vec<f64> = json["output"]
        .as_array()
        .expect("golden file missing 'output' array")
        .iter()
        .map(|v| v.as_f64().expect("non-numeric value in golden output"))
        .collect();

    let file_tolerance = json["tolerance"].as_f64().unwrap_or(tolerance);
    let effective_tolerance = tolerance.max(file_tolerance);

    assert_samples_match(output, &golden, effective_tolerance, golden_path);
}

// ── Golden reference recording ─────────────────────────────────────────

/// Record a golden reference file from current output.
///
/// Only writes if the file doesn't already exist (won't overwrite).
/// Call this from a dedicated recording test, not from regular tests.
pub fn record_golden(
    golden_path: &str,
    test_name: &str,
    spice: &str,
    output: &[f64],
    tolerance: f64,
    generator: &str,
) {
    let golden_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden");
    std::fs::create_dir_all(&golden_dir).expect("create golden dir");

    let path = golden_dir.join(golden_path);
    if path.exists() {
        return; // Don't overwrite existing golden files
    }

    let json = serde_json::json!({
        "test_name": test_name,
        "spice": spice,
        "output": output,
        "tolerance": tolerance,
        "generator": generator,
        "generated_at": chrono_now_iso(),
    });

    let content = serde_json::to_string_pretty(&json).expect("serialize golden");
    std::fs::write(&path, content)
        .unwrap_or_else(|e| panic!("Failed to write golden file {}: {e}", path.display()));
}

/// Current timestamp in ISO 8601 format (no chrono dependency).
fn chrono_now_iso() -> String {
    // Use a simple approach: read from `date` command
    let output = Command::new("date")
        .arg("-u")
        .arg("+%Y-%m-%dT%H:%M:%SZ")
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .unwrap_or_else(|| "unknown".to_string());
    output.trim().to_string()
}
