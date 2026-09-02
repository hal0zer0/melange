//! Capture mode: compile each manifest circuit with the installed `melange`
//! CLI, build stdin/stdout driver binaries (melange-validate mechanism),
//! render the deterministic programs, and write PCM + stats + metadata
//! into the baseline directory.

use crate::{manifest, programs, runner, stats};
use serde::Serialize;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Duration;

/// Fixed master noise seed for every capture. MUST be nonzero: generated
/// code maps seed 0 to SystemTime nanos (see `seed_noise_rngs_salted` in
/// the emitted module), which would destroy bit-reproducibility.
pub const GOLDEN_NOISE_SEED: u64 = 0x474F4C44454E31; // "GOLDEN1"-ish

#[derive(Serialize)]
struct ProgramCapture {
    name: String,
    ok: bool,
    frames: usize,
    channels: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
    render_secs: f64,
}

#[derive(Serialize)]
struct CircuitCapture {
    /// Unique baseline-directory key. Equals `plugin` when the plugin name
    /// is unique in the manifest; otherwise `plugin--<cir-stem>` (the real
    /// manifest maps several .cir files to one plugin, e.g. noyce x12).
    id: String,
    plugin: String,
    cir: String,
    /// Absolute path of the netlist actually compiled.
    cir_resolved: String,
    /// sha256 of the netlist content as compiled. The melange-circuits
    /// working tree (not its git HEAD) is the authoritative source for
    /// several shipped circuits, so a git rev cannot pin the input — the
    /// content hash can. Compare mode uses this to distinguish "the input
    /// changed" from "the compiler changed".
    cir_sha256: String,
    compile_ok: bool,
    noise_api_detected: bool,
    seed_api_detected: bool,
    pot_setters_detected: Vec<usize>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    warnings: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
    programs: Vec<ProgramCapture>,
}

/// Validate a manifest without capturing: parse it, resolve every netlist
/// path, and print the plan. Returns the number of unresolvable entries.
pub fn dry_run(manifest_path: &Path) -> Result<usize, String> {
    let entries = manifest::load(manifest_path)?;
    let mut bad = 0usize;
    println!("manifest OK: {} circuits", entries.len());
    let ids = unique_ids(&entries);
    for (i, e) in entries.iter().enumerate() {
        match resolve_cir(&e.cir) {
            Some(p) => {
                let sha = sh_line(&format!("sha256sum {} | cut -d' ' -f1", p.display()));
                println!(
                    "  {:36} {}  sha {}..  pots={} noise={} level={}",
                    ids[i],
                    p.display(),
                    &sha[..sha.len().min(12)],
                    e.has_pots,
                    e.has_noise,
                    e.input_level
                );
            }
            None => {
                bad += 1;
                println!("  {:36} UNRESOLVED netlist: {}", ids[i], e.cir);
            }
        }
    }
    if bad > 0 {
        println!("{bad} entries have unresolvable netlist paths");
    }
    Ok(bad)
}

pub fn run(
    manifest_path: &Path,
    out_dir: &Path,
    fs: f64,
    timeout_s: u64,
    keep_work: bool,
) -> Result<usize, String> {
    let entries = manifest::load(manifest_path)?;
    std::fs::create_dir_all(out_dir).map_err(|e| format!("create {}: {e}", out_dir.display()))?;

    let work = std::env::temp_dir().join(format!("golden-harness-{}", std::process::id()));
    std::fs::create_dir_all(&work).map_err(|e| format!("create workdir: {e}"))?;

    let timeout = Duration::from_secs(timeout_s);
    let mut reports: Vec<CircuitCapture> = Vec::new();
    let mut failed_circuits = 0usize;

    println!(
        "golden-harness capture: {} circuits -> {}",
        entries.len(),
        out_dir.display()
    );

    let ids = unique_ids(&entries);
    for (i, entry) in entries.iter().enumerate() {
        println!("[{}/{}] {}", i + 1, entries.len(), ids[i]);
        let rep = capture_circuit(entry, &ids[i], out_dir, &work, fs, timeout);
        let circuit_failed =
            !rep.compile_ok || rep.programs.iter().any(|p| !p.ok) || rep.error.is_some();
        if circuit_failed {
            failed_circuits += 1;
            println!("      FAILED (see capture_report.json)");
        }
        reports.push(rep);
    }

    write_metadata(manifest_path, out_dir, fs)?;
    let report_json = serde_json::json!({ "circuits": reports });
    std::fs::write(
        out_dir.join("capture_report.json"),
        serde_json::to_string_pretty(&report_json).unwrap(),
    )
    .map_err(|e| format!("write capture_report.json: {e}"))?;

    if !keep_work {
        let _ = std::fs::remove_dir_all(&work);
    } else {
        println!("work dir kept: {}", work.display());
    }

    println!(
        "capture done: {}/{} circuits ok",
        entries.len() - failed_circuits,
        entries.len()
    );
    Ok(failed_circuits)
}

/// Unique per-entry directory keys: `plugin`, or `plugin--<cir-stem>` when
/// several manifest entries share one plugin name.
fn unique_ids(entries: &[manifest::Entry]) -> Vec<String> {
    let mut counts: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
    for e in entries {
        *counts.entry(e.plugin.as_str()).or_default() += 1;
    }
    entries
        .iter()
        .map(|e| {
            if counts[e.plugin.as_str()] > 1 {
                let stem = Path::new(&e.cir)
                    .file_stem()
                    .map(|s| s.to_string_lossy().to_string())
                    .unwrap_or_else(|| "unknown".into());
                format!("{}--{}", e.plugin, stem)
            } else {
                e.plugin.clone()
            }
        })
        .collect()
}

fn capture_circuit(
    entry: &manifest::Entry,
    id: &str,
    out_dir: &Path,
    work: &Path,
    fs: f64,
    timeout: Duration,
) -> CircuitCapture {
    let mut rep = CircuitCapture {
        id: id.to_string(),
        plugin: entry.plugin.clone(),
        cir: entry.cir.clone(),
        cir_resolved: String::new(),
        cir_sha256: String::new(),
        compile_ok: false,
        noise_api_detected: false,
        seed_api_detected: false,
        pot_setters_detected: Vec::new(),
        warnings: Vec::new(),
        error: None,
        programs: Vec::new(),
    };

    let circuit_dir = out_dir.join(id);
    if let Err(e) = std::fs::create_dir_all(&circuit_dir) {
        rep.error = Some(format!("create circuit dir: {e}"));
        return rep;
    }

    // 0. Resolve the netlist path and hash its content. The working tree
    //    is authoritative (melange-circuits HEAD is stale), so the hash —
    //    not any git rev — pins what was compiled.
    let cir_abs = match resolve_cir(&entry.cir) {
        Some(p) => p,
        None => {
            rep.error = Some(format!("netlist not found: {}", entry.cir));
            return rep;
        }
    };
    rep.cir_resolved = cir_abs.display().to_string();
    rep.cir_sha256 = sh_line(&format!("sha256sum {} | cut -d' ' -f1", cir_abs.display()));
    if rep.cir_sha256.is_empty() {
        rep.warnings.push("sha256sum failed on netlist".into());
    }

    // 1. melange CLI compile (the same path oomox uses).
    let gen_rs = work.join(format!("{id}.rs"));
    let rs_path =
        match runner::compile_circuit(entry.compile_cmd.as_deref(), &rep.cir_resolved, &gen_rs) {
            Ok(p) => p,
            Err(e) => {
                rep.error = Some(e);
                return rep;
            }
        };
    let code = match std::fs::read_to_string(&rs_path) {
        Ok(c) => c,
        Err(e) => {
            rep.error = Some(format!("read generated module: {e}"));
            return rep;
        }
    };
    rep.compile_ok = true;

    // Provenance copy: lets a compare-time delta be traced to a codegen diff.
    // The `// melange: <version> (<commit>)` + `// provenance:` JSON header lines
    // carry the build's version/commit, which change every commit and would make
    // a byte-diff of two revs' circuit.rs noisy. Mask ONLY those two identity
    // fields (the resolved DSP-contract fields in the provenance JSON are
    // deterministic and stay, so real codegen changes still show). Byte-neutrality
    // is a verification tool, not a goal — nothing else is touched. `code` itself
    // is left intact for the feature detection below.
    let _ = std::fs::write(
        circuit_dir.join("circuit.rs"),
        normalize_provenance_header(&code),
    );

    // 2. Feature detection on the generated module (authoritative — the
    //    manifest's has_noise/has_pots only set expectations).
    rep.noise_api_detected = code.contains("pub fn set_noise_enabled");
    rep.seed_api_detected = code.contains("pub fn set_seed");
    rep.pot_setters_detected = detect_pot_setters(&code);
    if entry.has_noise && !rep.noise_api_detected {
        rep.warnings.push(
            "manifest says has_noise but generated code has no noise API \
             (compile_cmd missing --noise?)"
                .into(),
        );
    }
    if entry.has_pots && rep.pot_setters_detected.is_empty() {
        rep.warnings
            .push("manifest says has_pots but no set_pot_N found in generated code".into());
    }
    if rep.noise_api_detected && !rep.seed_api_detected {
        rep.warnings.push(
            "noise API present but no set_seed — noise renders may be nondeterministic".into(),
        );
    }

    // 3. Build driver binaries.
    let std_bin = work.join(format!("{id}_std"));
    if let Err(e) = build_driver(
        &code,
        &std_main(rep.noise_api_detected && rep.seed_api_detected),
        work,
        &format!("{id}_std"),
        &std_bin,
    ) {
        rep.error = Some(format!("std driver: {e}"));
        return rep;
    }
    let use_pot_sweep = !rep.pot_setters_detected.is_empty();
    let pot_bin = work.join(format!("{id}_pot"));
    if use_pot_sweep {
        let total = programs::frames(&programs::POTSWEEP, fs);
        if let Err(e) = build_driver(
            &code,
            &pot_main(
                rep.noise_api_detected && rep.seed_api_detected,
                &rep.pot_setters_detected,
                total,
            ),
            work,
            &format!("{id}_pot"),
            &pot_bin,
        ) {
            rep.error = Some(format!("pot-sweep driver: {e}"));
            return rep;
        }
    }

    // 4. Render each program. Failures are recorded, never abort.
    for prog in programs::programs(use_pot_sweep) {
        let started = std::time::Instant::now();
        let input = programs::gen_input(&prog, fs, entry.input_level);
        let bin = if prog.pot_sweep { &pot_bin } else { &std_bin };
        let mut pc = ProgramCapture {
            name: prog.name.to_string(),
            ok: false,
            frames: 0,
            channels: 0,
            error: None,
            render_secs: 0.0,
        };
        match runner::run_render(bin, &input, timeout) {
            Ok(out) => {
                pc.frames = out.frames;
                pc.channels = out.channels;
                if out.frames != input.len() {
                    pc.error = Some(format!(
                        "sample count mismatch: sent {}, got {}",
                        input.len(),
                        out.frames
                    ));
                } else {
                    match write_outputs(&circuit_dir, prog.name, &out, fs) {
                        Ok(()) => pc.ok = true,
                        Err(e) => pc.error = Some(e),
                    }
                }
            }
            Err(e) => pc.error = Some(e),
        }
        pc.render_secs = started.elapsed().as_secs_f64();
        println!(
            "      {:9} {}  ({:.1}s)",
            prog.name,
            if pc.ok { "ok" } else { "FAIL" },
            pc.render_secs
        );
        rep.programs.push(pc);
    }
    rep
}

fn write_outputs(
    circuit_dir: &Path,
    prog_name: &str,
    out: &runner::RenderOutput,
    fs: f64,
) -> Result<(), String> {
    // Raw PCM: interleaved f64 little-endian.
    //
    // f64, not f32. The renders are the substrate of every equivalence claim
    // the project makes — refactor neutrality and (eventually) Rust-vs-C++
    // parity. Storing them at f32 put a ~6e-8 relative floor under all of it,
    // which is orders of magnitude coarser than the differences that actually
    // matter here (FP contraction, libm divergence: ~1 ulp of f64). Baselines
    // written before this change carry the `.f32le` extension and are readable
    // by `compare`, but cannot support a bit-exactness claim.
    let pcm_path = circuit_dir.join(format!("{prog_name}.f64le"));
    let f = std::fs::File::create(&pcm_path).map_err(|e| format!("create pcm: {e}"))?;
    let mut w = std::io::BufWriter::new(f);
    for v in &out.interleaved {
        w.write_all(&v.to_le_bytes())
            .map_err(|e| format!("write pcm: {e}"))?;
    }
    w.flush().map_err(|e| format!("flush pcm: {e}"))?;

    let st = stats::compute(&out.interleaved, out.channels, out.frames, fs);
    std::fs::write(
        circuit_dir.join(format!("{prog_name}.stats.json")),
        serde_json::to_string_pretty(&st).unwrap(),
    )
    .map_err(|e| format!("write stats: {e}"))?;
    Ok(())
}

fn build_driver(
    code: &str,
    main_src: &str,
    work: &Path,
    stem: &str,
    bin: &Path,
) -> Result<(), String> {
    let src = work.join(format!("{stem}_driver.rs"));
    let full = format!("{code}\n{main_src}");
    std::fs::write(&src, full).map_err(|e| format!("write driver source: {e}"))?;
    runner::rustc(&src, bin)
}

/// Detect `pub fn set_pot_N(` setters in the generated module. Runtime-R
/// setters (`set_runtime_R_*`) are deliberately excluded — those are
/// audio-rate envelope hooks, not knobs.
fn detect_pot_setters(code: &str) -> Vec<usize> {
    let mut out = Vec::new();
    let needle = "pub fn set_pot_";
    let mut rest = code;
    while let Some(pos) = rest.find(needle) {
        rest = &rest[pos + needle.len()..];
        let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
        if let Ok(idx) = digits.parse::<usize>() {
            if !out.contains(&idx) {
                out.push(idx);
            }
        }
    }
    out.sort_unstable();
    out
}

fn seed_block(pin_noise: bool) -> &'static str {
    if pin_noise {
        "    state.set_noise_enabled(true);\n    state.set_seed(GOLDEN_SEED);\n"
    } else {
        ""
    }
}

/// Standard driver: one f64 sample per stdin line, all output channels per
/// stdout line. Mirrors melange-validate's driver (buffered IO added for
/// throughput; formatting `{:.17e}` is an exact f64 round-trip).
fn std_main(pin_noise: bool) -> String {
    format!(
        r#"
#[allow(dead_code)]
const GOLDEN_SEED: u64 = {GOLDEN_NOISE_SEED};
fn main() {{
    use std::io::{{BufRead, Write}};
    let mut state = CircuitState::default();
{seed}    let stdin = std::io::stdin();
    let mut reader = std::io::BufReader::new(stdin.lock());
    let stdout = std::io::stdout();
    let mut w = std::io::BufWriter::new(stdout.lock());
    let mut line = String::new();
    loop {{
        line.clear();
        if reader.read_line(&mut line).unwrap() == 0 {{ break; }}
        if let Ok(input) = line.trim().parse::<f64>() {{
            let out = process_sample(input, &mut state);
            for (i, v) in out.iter().enumerate() {{
                if i > 0 {{ w.write_all(b" ").unwrap(); }}
                write!(w, "{{:.17e}}", v).unwrap();
            }}
            w.write_all(b"\n").unwrap();
        }}
    }}
    w.flush().unwrap();
}}
"#,
        seed = seed_block(pin_noise),
    )
}

/// Pot-sweep driver: like the standard driver, but every
/// `POT_UPDATE_INTERVAL` samples all detected pots are driven along a
/// triangle 0 -> 1 -> 0 position profile over the whole render, mapped
/// linearly into each pot's [MIN_R, MAX_R]. Exercises the setter /
/// matrices_dirty / rebuild lifecycle the way a host automation pass does.
fn pot_main(pin_noise: bool, pots: &[usize], total_frames: usize) -> String {
    let interval = programs::POT_UPDATE_INTERVAL;
    let mut setters = String::new();
    for idx in pots {
        setters.push_str(&format!(
            "            state.set_pot_{idx}(POT_{idx}_MIN_R + pos * (POT_{idx}_MAX_R - POT_{idx}_MIN_R));\n"
        ));
    }
    format!(
        r#"
#[allow(dead_code)]
const GOLDEN_SEED: u64 = {GOLDEN_NOISE_SEED};
const SWEEP_TOTAL_FRAMES: u64 = {total_frames};
const POT_UPDATE_INTERVAL: u64 = {interval};
fn main() {{
    use std::io::{{BufRead, Write}};
    let mut state = CircuitState::default();
{seed}    let stdin = std::io::stdin();
    let mut reader = std::io::BufReader::new(stdin.lock());
    let stdout = std::io::stdout();
    let mut w = std::io::BufWriter::new(stdout.lock());
    let mut line = String::new();
    let mut n: u64 = 0;
    loop {{
        line.clear();
        if reader.read_line(&mut line).unwrap() == 0 {{ break; }}
        if let Ok(input) = line.trim().parse::<f64>() {{
            if n % POT_UPDATE_INTERVAL == 0 {{
                let t = n as f64 / SWEEP_TOTAL_FRAMES as f64;
                let pos = if t < 0.5 {{ 2.0 * t }} else {{ 2.0 * (1.0 - t) }};
{setters}            }}
            let out = process_sample(input, &mut state);
            for (i, v) in out.iter().enumerate() {{
                if i > 0 {{ w.write_all(b" ").unwrap(); }}
                write!(w, "{{:.17e}}", v).unwrap();
            }}
            w.write_all(b"\n").unwrap();
            n += 1;
        }}
    }}
    w.flush().unwrap();
}}
"#,
        seed = seed_block(pin_noise),
    )
}

/// Resolve a manifest netlist path to an existing file. Tries, in order:
/// the path as given (tilde-expanded by the manifest loader), relative to
/// `~/dev/melange-circuits` (the real manifest uses repo-relative paths
/// like `unstable/pedals/x.cir`), and relative to `~/dev/oomox` (covers
/// `../melange-circuits/<cir>` forms).
fn resolve_cir(cir: &str) -> Option<PathBuf> {
    let as_given = PathBuf::from(cir);
    if as_given.is_file() {
        return as_given.canonicalize().ok().or(Some(as_given));
    }
    if let Ok(home) = std::env::var("HOME") {
        for base in ["dev/melange-circuits", "dev/oomox"] {
            let p = Path::new(&home).join(base).join(cir);
            if p.is_file() {
                return p.canonicalize().ok().or(Some(p));
            }
        }
    }
    None
}

fn sh_line(cmd: &str) -> String {
    runner::sh(cmd)
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_default()
}

fn write_metadata(manifest_path: &Path, out_dir: &Path, fs: f64) -> Result<(), String> {
    // Melange repo root: this tool lives at <root>/tools/golden-harness.
    let melange_root: PathBuf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."));
    let root = melange_root.display();
    let rev = sh_line(&format!("git -C {root} rev-parse HEAD"));
    let dirty = !sh_line(&format!("git -C {root} status --porcelain")).is_empty();
    let manifest_sha = sh_line(&format!(
        "sha256sum {} | cut -d' ' -f1",
        manifest_path.display()
    ));

    let meta = serde_json::json!({
        "melange_rev": rev,
        "melange_worktree_dirty": dirty,
        "melange_version": sh_line("melange --version"),
        "melange_bin": sh_line("command -v melange"),
        "rustc_version": sh_line("rustc --version"),
        "captured_at": sh_line("date -u +%Y-%m-%dT%H:%M:%SZ"),
        "sample_rate": fs,
        "noise_seed": GOLDEN_NOISE_SEED,
        "manifest_path": manifest_path.display().to_string(),
        "manifest_sha256": manifest_sha,
        "harness": "golden-harness 0.1",
        "programs": {
            "silence": {"seconds": programs::SILENCE.seconds},
            "sine1k": {"seconds": programs::SINE1K.seconds, "freq_hz": 1000.0},
            "sweep": {"seconds": programs::SWEEP.seconds, "f0_hz": 20.0, "f1_hz": 20000.0},
            "step": {"seconds": programs::STEP.seconds, "edge_at_s": 0.1},
            "potsweep": {"seconds": programs::POTSWEEP.seconds,
                          "update_interval": programs::POT_UPDATE_INTERVAL},
        },
    });
    std::fs::write(
        out_dir.join("metadata.json"),
        serde_json::to_string_pretty(&meta).unwrap(),
    )
    .map_err(|e| format!("write metadata.json: {e}"))
}

/// Mask the per-commit build-identity fields in a generated module's provenance
/// header so a byte-diff of two revs' `circuit.rs` reflects only real codegen
/// changes, not the version/commit stamp.
///
/// Touches exactly two lines and nothing else:
/// - `// melange: <v> (<c>)`  → `// melange: <version> (<commit>)`
/// - the `// provenance: {...}` JSON: the `"melange"` and `"commit"` values are
///   replaced with placeholders; every other (deterministic, DSP-contract)
///   field is preserved verbatim.
fn normalize_provenance_header(code: &str) -> String {
    let mut out = String::with_capacity(code.len());
    for line in code.lines() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("// melange:") {
            out.push_str("// melange: <version> (<commit>)");
        } else if trimmed.starts_with("// provenance:") {
            let masked = mask_json_field(line, "melange", "<version>");
            let masked = mask_json_field(&masked, "commit", "<commit>");
            out.push_str(&masked);
        } else {
            out.push_str(line);
        }
        out.push('\n');
    }
    out
}

/// Replace the string value of `"key":"..."` in `s` with `placeholder`, leaving
/// the rest of the line untouched. No-op if the key is absent or malformed.
fn mask_json_field(s: &str, key: &str, placeholder: &str) -> String {
    let needle = format!("\"{key}\":\"");
    let Some(start) = s.find(&needle) else {
        return s.to_string();
    };
    let val_start = start + needle.len();
    let Some(rel_end) = s[val_start..].find('"') else {
        return s.to_string();
    };
    let end = val_start + rel_end;
    let mut out = String::with_capacity(s.len());
    out.push_str(&s[..val_start]);
    out.push_str(placeholder);
    out.push_str(&s[end..]);
    out
}

#[cfg(test)]
mod provenance_norm_tests {
    use super::*;

    #[test]
    fn masks_melange_and_provenance_lines_only() {
        let code = "// Generated by melange-solver\n\
                    // melange: 0.1.0 (e2f6d62)\n\
                    // Circuit: \"X\"\n\
                    // Build: integration=trapezoidal, dc-block=on\n\
                    // provenance: {\"melange\":\"0.1.0\",\"commit\":\"e2f6d62\",\"dc_block\":true}\n\
                    pub const N: usize = 2;\n";
        let out = normalize_provenance_header(code);
        assert!(out.contains("// melange: <version> (<commit>)"));
        assert!(out.contains("\"melange\":\"<version>\""));
        assert!(out.contains("\"commit\":\"<commit>\""));
        // Deterministic DSP-contract field preserved.
        assert!(out.contains("\"dc_block\":true"));
        // Untouched lines survive verbatim.
        assert!(out.contains("// Circuit: \"X\""));
        assert!(out.contains("pub const N: usize = 2;"));
        // Two revs with different version/commit normalize identically.
        let code2 = code.replace("0.1.0 (e2f6d62)", "0.2.0 (abc1234)").replace(
            "\"melange\":\"0.1.0\",\"commit\":\"e2f6d62\"",
            "\"melange\":\"0.2.0\",\"commit\":\"abc1234\"",
        );
        assert_eq!(out, normalize_provenance_header(&code2));
    }
}
