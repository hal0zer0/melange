//! Compilation and execution of generated circuit code.
//!
//! The run mechanism is a direct port of melange-validate's
//! `run_melange_codegen_with_main` (crates/melange-validate/tests/
//! spice_validation.rs): append a `fn main()` driver to the generated
//! module, compile the single file with `rustc --edition=2024 -O`, then
//! pipe text samples through stdin/stdout with the stdin write on a
//! separate thread (the documented pipe-deadlock avoidance). We add a
//! poll-based timeout so an unattended run can never hang on one circuit.

use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

/// Run a shell command line, capturing output.
pub fn sh(cmd: &str) -> Result<std::process::Output, String> {
    Command::new("sh")
        .arg("-c")
        .arg(cmd)
        .output()
        .map_err(|e| format!("spawn `sh -c {cmd}`: {e}"))
}

/// Run the manifest's compile command (default: `melange compile {cir} -o
/// {out} -f code`), substituting `{cir}` / `{out}` placeholders.
///
/// Manifest compile commands may carry paths relative to some other repo's
/// root (the real manifest uses oomox-relative `-o plugins/<x>/src/circuit.rs`
/// and `../melange-circuits/<cir>` inputs). The harness must never write
/// into those trees, so the command is tokenized and rewritten:
/// - the `-o` / `--output` target is replaced with our work path
///   (appended if absent),
/// - any token ending in `.cir` is replaced with the RESOLVED absolute
///   netlist path (appended if absent).
///
/// Paths containing spaces are not supported (none exist in this ecosystem).
///
/// Returns the path of the generated Rust module (searching inside `out`
/// if the command produced a directory, e.g. a plugin project).
/// Result of invoking `melange compile` for one circuit.
pub struct CompileOutcome {
    pub generated: PathBuf,
    /// Nodal sub-path reported by the compiler ("schur" / "full-lu"), or None
    /// on the DK path where no sub-path applies.
    pub nodal_sub_path: Option<String>,
}

pub fn compile_circuit(
    compile_cmd: Option<&str>,
    cir_abs: &str,
    out_rs: &Path,
) -> Result<CompileOutcome, String> {
    let out_str = out_rs.to_string_lossy().to_string();
    let raw = compile_cmd
        .unwrap_or("melange compile {cir} -o {out} -f code")
        .replace("{cir}", cir_abs)
        .replace("{out}", &out_str);
    let mut toks: Vec<String> = raw.split_whitespace().map(String::from).collect();
    let mut had_out = false;
    let mut i = 0;
    while i < toks.len() {
        if (toks[i] == "-o" || toks[i] == "--output") && i + 1 < toks.len() {
            toks[i + 1] = out_str.clone();
            had_out = true;
        }
        i += 1;
    }
    if !had_out {
        toks.push("-o".into());
        toks.push(out_str.clone());
    }
    let mut had_cir = false;
    for t in toks.iter_mut() {
        if t.ends_with(".cir") {
            *t = cir_abs.to_string();
            had_cir = true;
        }
    }
    if !had_cir {
        toks.push(cir_abs.to_string());
    }
    let cmd = toks.join(" ");

    let output = sh(&cmd)?;
    if !output.status.success() {
        return Err(format!(
            "compile command failed ({cmd}):\n{}",
            String::from_utf8_lossy(&output.stderr)
        ));
    }

    // Which nodal sub-path the emitter took, straight from the compiler's own
    // summary. Recorded so a sub-path move is named explicitly rather than
    // showing up only as a large unexplained circuit.rs diff.
    let stdout = String::from_utf8_lossy(&output.stdout);
    let nodal_sub_path = stdout.lines().find_map(|l| {
        l.trim()
            .strip_prefix("Nodal sub-path:")
            .map(|v| v.trim().to_string())
    });

    // Locate the generated module.
    if out_rs.is_file() {
        return Ok(CompileOutcome {
            generated: out_rs.to_path_buf(),
            nodal_sub_path,
        });
    }
    if out_rs.is_dir() {
        if let Some(p) = find_circuit_rs(out_rs, 3) {
            return Ok(CompileOutcome {
                generated: p,
                nodal_sub_path,
            });
        }
    }
    Err(format!(
        "compile command succeeded but no generated .rs found at {}",
        out_rs.display()
    ))
}

/// Recursively search (bounded depth) for a .rs file containing
/// `pub fn process_sample` — the circuit module inside a plugin project.
fn find_circuit_rs(dir: &Path, depth: usize) -> Option<PathBuf> {
    if depth == 0 {
        return None;
    }
    let entries = std::fs::read_dir(dir).ok()?;
    let mut subdirs = Vec::new();
    for e in entries.flatten() {
        let p = e.path();
        if p.is_file() && p.extension().is_some_and(|x| x == "rs") {
            if let Ok(txt) = std::fs::read_to_string(&p) {
                if txt.contains("pub fn process_sample") {
                    return Some(p);
                }
            }
        } else if p.is_dir() {
            subdirs.push(p);
        }
    }
    for d in subdirs {
        if let Some(p) = find_circuit_rs(&d, depth - 1) {
            return Some(p);
        }
    }
    None
}

/// Compile a standalone .rs (generated module + driver main) to a binary.
pub fn rustc(src: &Path, bin: &Path) -> Result<(), String> {
    let output = Command::new("rustc")
        .arg(src)
        .arg("-o")
        .arg(bin)
        .arg("--edition=2024")
        .arg("-O")
        .output()
        .map_err(|e| format!("spawn rustc: {e}"))?;
    if !output.status.success() {
        return Err(format!(
            "rustc failed on {}:\n{}",
            src.display(),
            String::from_utf8_lossy(&output.stderr)
        ));
    }
    Ok(())
}

pub struct RenderOutput {
    pub channels: usize,
    /// Interleaved samples, frames * channels.
    ///
    /// Held at f64. The driver prints full `{:.17e}` f64 text and this used to
    /// be downcast to f32 on the way in, which put a ~6e-8 relative floor under
    /// every comparison — far above the ~1-ulp-of-f64 differences that FMA
    /// contraction and libm divergence produce. A cross-language or refactor
    /// gate built on that floor cannot see the defects it exists to catch.
    pub interleaved: Vec<f64>,
    pub frames: usize,
    /// Diagnostic counters reported by the driver on stderr as a single
    /// `MELANGE_DIAG k=v ...` line. Empty when the generated module declares
    /// none.
    ///
    /// These are the only visibility the gate has into the NR recovery
    /// ladders. The renders cannot see them: measured across the corpus, only
    /// 7 of 41 circuits enter any ladder, and several ladders never fire on any
    /// deck — so emitted recovery code can change with every sample unchanged.
    pub diagnostics: std::collections::BTreeMap<String, f64>,
}

/// Parse the driver's `MELANGE_DIAG k=v k=v ...` stderr line.
fn parse_diagnostics(stderr: &[u8]) -> std::collections::BTreeMap<String, f64> {
    let mut out = std::collections::BTreeMap::new();
    let text = String::from_utf8_lossy(stderr);
    for line in text.lines() {
        let Some(rest) = line.trim().strip_prefix("MELANGE_DIAG") else {
            continue;
        };
        for tok in rest.split_whitespace() {
            if let Some((k, v)) = tok.split_once('=') {
                if let Ok(n) = v.parse::<f64>() {
                    out.insert(k.to_string(), n);
                }
            }
        }
    }
    out
}

/// Pipe `input` (one `{:.17e}` f64 per line) through the compiled driver
/// binary and collect its multi-channel output (space-separated per line).
pub fn run_render(bin: &Path, input: &[f64], timeout: Duration) -> Result<RenderOutput, String> {
    let stdin_data: Vec<u8> = input
        .iter()
        .map(|s| format!("{s:.17e}\n"))
        .collect::<String>()
        .into_bytes();

    let mut child = Command::new(bin)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("spawn {}: {e}", bin.display()))?;

    // Writer thread: see melange-validate for the deadlock rationale.
    let stdin = child.stdin.take();
    let writer = std::thread::spawn(move || {
        if let Some(mut s) = stdin {
            let _ = s.write_all(&stdin_data);
        }
    });
    let mut stdout_pipe = child.stdout.take().ok_or("no stdout pipe")?;
    let out_reader = std::thread::spawn(move || {
        let mut buf = Vec::new();
        let _ = stdout_pipe.read_to_end(&mut buf);
        buf
    });
    let mut stderr_pipe = child.stderr.take().ok_or("no stderr pipe")?;
    let err_reader = std::thread::spawn(move || {
        let mut buf = Vec::new();
        let _ = stderr_pipe.read_to_end(&mut buf);
        buf
    });

    let deadline = Instant::now() + timeout;
    let status = loop {
        match child.try_wait() {
            Ok(Some(st)) => break st,
            Ok(None) => {
                if Instant::now() > deadline {
                    let _ = child.kill();
                    let _ = child.wait();
                    let _ = writer.join();
                    let _ = out_reader.join();
                    let _ = err_reader.join();
                    return Err(format!("render timed out after {:?}", timeout));
                }
                std::thread::sleep(Duration::from_millis(20));
            }
            Err(e) => return Err(format!("wait: {e}")),
        }
    };
    let _ = writer.join();
    let stdout_buf = out_reader.join().map_err(|_| "stdout reader panicked")?;
    let stderr_buf = err_reader.join().map_err(|_| "stderr reader panicked")?;

    if !status.success() {
        return Err(format!(
            "render binary exited with {status}:\n{}",
            String::from_utf8_lossy(&stderr_buf)
        ));
    }

    let text = String::from_utf8_lossy(&stdout_buf);
    let mut channels = 0usize;
    let mut interleaved: Vec<f64> = Vec::new();
    let mut frames = 0usize;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let vals: Vec<f64> = line
            .split_whitespace()
            .filter_map(|t| t.parse::<f64>().ok())
            .collect();
        if vals.is_empty() {
            continue;
        }
        if channels == 0 {
            channels = vals.len();
        } else if vals.len() != channels {
            return Err(format!(
                "inconsistent channel count in render output: {} then {}",
                channels,
                vals.len()
            ));
        }
        for v in &vals {
            interleaved.push(*v);
        }
        frames += 1;
    }
    if frames == 0 {
        return Err("render produced no samples".into());
    }
    Ok(RenderOutput {
        channels,
        interleaved,
        frames,
        diagnostics: parse_diagnostics(&stderr_buf),
    })
}
