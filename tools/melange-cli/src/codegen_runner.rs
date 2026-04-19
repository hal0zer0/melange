//! Codegen runner: compile circuit code to binary and execute.
//!
//! Shared orchestration for `melange simulate` and `melange analyze`.
//! Generates circuit code, compiles to an optimized binary via `rustc`,
//! and runs it with the specified input. Binaries are cached by source
//! hash to avoid recompilation for the same circuit+config.

use anyhow::{Context, Result};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;

/// A compiled circuit binary ready to run.
pub struct CompiledBinary {
    /// Path to the compiled binary.
    pub path: PathBuf,
    /// Whether this binary was loaded from cache (vs freshly compiled).
    pub cached: bool,
}

/// Binary cache for compiled circuit code.
///
/// Stores compiled binaries in `~/.cache/melange/binaries/` keyed by
/// hash of the source code. Same circuit+config = skip compilation.
pub struct BinaryCache {
    cache_dir: PathBuf,
}

impl BinaryCache {
    /// Create a new binary cache.
    pub fn new() -> Result<Self> {
        let cache_dir = dirs::cache_dir()
            .map(|p| p.join("melange").join("binaries"))
            .ok_or_else(|| anyhow::anyhow!("Cannot find cache directory"))?;

        std::fs::create_dir_all(&cache_dir).with_context(|| {
            format!(
                "Failed to create binary cache dir: {}",
                cache_dir.display()
            )
        })?;

        Ok(Self { cache_dir })
    }

    /// Compile source code to a binary, using cache if available.
    ///
    /// # Arguments
    /// * `source` — Complete Rust source code (circuit + main)
    /// * `name` — Human-readable name for diagnostics
    ///
    /// # Returns
    /// Path to the compiled binary. Cached binaries are reused without recompilation.
    pub fn compile(&self, source: &str, name: &str) -> Result<CompiledBinary> {
        let hash = hash_source(source);
        let bin_name = format!("melange_{name}_{hash:016x}");
        let bin_path = self.cache_dir.join(&bin_name);

        // Check cache
        if bin_path.exists() {
            return Ok(CompiledBinary {
                path: bin_path,
                cached: true,
            });
        }

        // Use PID-unique temp paths to avoid races between concurrent processes
        let pid = std::process::id();
        let src_path = self.cache_dir.join(format!("{bin_name}_tmp_{pid}.rs"));
        let tmp_bin = self.cache_dir.join(format!("{bin_name}_tmp_{pid}"));

        // Write source to temp file
        {
            let mut f = std::fs::File::create(&src_path)
                .with_context(|| format!("Failed to create source file: {}", src_path.display()))?;
            f.write_all(source.as_bytes())?;
        }

        // Compile to temp binary with optimization
        let compile = Command::new("rustc")
            .arg(&src_path)
            .arg("-o")
            .arg(&tmp_bin)
            .arg("--edition=2024")
            .arg("-O")
            .output()
            .context("Failed to invoke rustc. Is the Rust toolchain installed?")?;

        // Clean up source file regardless of outcome
        let _ = std::fs::remove_file(&src_path);

        if !compile.status.success() {
            // Clean up failed temp binary (rustc may leave a partial file)
            let _ = std::fs::remove_file(&tmp_bin);
            let stderr = String::from_utf8_lossy(&compile.stderr);
            anyhow::bail!("Compilation failed for '{name}':\n{stderr}");
        }

        // Atomic placement — rename to final path on same filesystem
        match std::fs::rename(&tmp_bin, &bin_path) {
            Ok(_) => {}
            Err(_) => {
                // Another process won the race — use their binary, discard ours
                let _ = std::fs::remove_file(&tmp_bin);
            }
        }

        Ok(CompiledBinary {
            path: bin_path,
            cached: false,
        })
    }

    /// Remove all cached binaries.
    pub fn clear(&self) -> Result<()> {
        for entry in std::fs::read_dir(&self.cache_dir)? {
            let entry = entry?;
            let path = entry.path();
            if path.is_file() && path.extension().is_none_or(|e| e != "rs") {
                let _ = std::fs::remove_file(&path);
            }
        }
        Ok(())
    }

    /// Get cache statistics.
    pub fn stats(&self) -> CacheStats {
        let mut total_files = 0;
        let mut total_bytes = 0u64;
        if let Ok(entries) = std::fs::read_dir(&self.cache_dir) {
            for entry in entries.flatten() {
                if let Ok(meta) = entry.metadata() {
                    if meta.is_file() {
                        total_files += 1;
                        total_bytes += meta.len();
                    }
                }
            }
        }
        CacheStats {
            total_files,
            total_bytes,
        }
    }
}

/// Binary cache statistics.
pub struct CacheStats {
    pub total_files: usize,
    pub total_bytes: u64,
}

impl CacheStats {
    pub fn formatted_size(&self) -> String {
        if self.total_bytes < 1024 {
            format!("{} B", self.total_bytes)
        } else if self.total_bytes < 1024 * 1024 {
            format!("{:.1} KB", self.total_bytes as f64 / 1024.0)
        } else {
            format!("{:.1} MB", self.total_bytes as f64 / (1024.0 * 1024.0))
        }
    }
}

/// Hash source code for cache key.
fn hash_source(source: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    source.hash(&mut hasher);
    hasher.finish()
}

// ── Main generation templates ──────────────────────────────────────────

/// Generate a `fn main()` for the `simulate` command.
///
/// The binary reads input WAV from argv[1], writes output WAV to argv[2].
/// Supports both WAV file input and sine test tone generation.
pub fn generate_simulate_main(
    sample_rate: f64,
    pot_calls: &[String],
    switch_calls: &[String],
    amplitude: Option<f64>,
    freq: f64,
    duration_secs: f64,
    probe_names: &[&str],
) -> String {
    let pot_lines: String = pot_calls
        .iter()
        .map(|c| format!("    {c};\n"))
        .collect();
    let switch_lines: String = switch_calls
        .iter()
        .map(|c| format!("    {c};\n"))
        .collect();

    // Embed minimal WAV reader/writer
    let wav_code = include_str!("wav_embed.rs.inc");

    // Probe plumbing. When probe_names is empty the generated body is
    // byte-identical to the pre-feature version (no CSV writer, no argv[3]).
    // Probes live at `out[1..=N]`; `out[0]` is always the primary output.
    let has_probes = !probe_names.is_empty();
    let probe_header: String = if has_probes {
        // Node names are CLI-resolved netlist identifiers (no commas,
        // no quotes, no newlines) — safe to embed bare.
        let cols = probe_names
            .iter()
            .map(|n| (*n).to_string())
            .collect::<Vec<_>>()
            .join(",");
        format!("sample_idx,time_s,{cols}")
    } else {
        String::new()
    };
    let probe_count = probe_names.len();
    let probe_open: String = if has_probes {
        r#"
    let probe_csv_path = args.get(3).cloned().unwrap_or_else(|| {
        eprintln!("Probes compiled in but argv[3] (probe CSV path) missing");
        std::process::exit(1);
    });
    let probe_file = std::fs::File::create(&probe_csv_path).unwrap_or_else(|e| {
        eprintln!("Failed to open probe CSV {}: {}", probe_csv_path, e);
        std::process::exit(1);
    });
    let mut probe_writer = std::io::BufWriter::new(probe_file);
    use std::io::Write as _;
    writeln!(probe_writer, "{PROBE_HEADER}").ok();
"#
        .replace("{PROBE_HEADER}", &probe_header)
    } else {
        String::new()
    };
    // Per-sample probe emit — writes `out[1..=N]` as CSV row.
    let probe_emit: String = if has_probes {
        r#"
        {
            let mut row = format!("{},{:.9}", i, (i as f64) / sr);
            for k in 1..=PROBE_COUNT {
                row.push_str(&format!(",{:.9}", out[k]));
            }
            writeln!(probe_writer, "{}", row).ok();
        }
"#
        .replace("PROBE_COUNT", &probe_count.to_string())
    } else {
        String::new()
    };
    let probe_close: String = if has_probes {
        String::from(r#"    probe_writer.flush().ok();
    eprintln!("DIAG:probes_written={}", samples.len());
"#)
    } else {
        String::new()
    };

    format!(
        r#"{wav_code}

fn main() {{
    let args: Vec<String> = std::env::args().collect();

    let mut state = CircuitState::default();
{pot_lines}{switch_lines}
    // Determine input source
    let (samples, sr) = if let Some(input_path) = args.get(1) {{
        if input_path == "--tone" {{
            // Test tone mode
            let sr: f64 = {sample_rate:.6};
            let dur_s: f64 = {duration_secs:.6};
            let n = (sr * dur_s) as usize;
            let amp: f64 = {amp:.17e};
            let freq: f64 = {freq:.6};
            let samples: Vec<f64> = (0..n)
                .map(|i| amp * (2.0 * std::f64::consts::PI * freq * (i as f64) / sr).sin())
                .collect();
            (samples, sr)
        }} else {{
            read_wav(input_path)
        }}
    }} else {{
        eprintln!("Usage: binary <input.wav|--tone> <output.wav> [probes.csv]");
        std::process::exit(1);
    }};

    state.set_sample_rate(sr);

    let output_path = args.get(2).map(|s| s.as_str()).unwrap_or("output.wav");
{probe_open}
    let mut output = Vec::with_capacity(samples.len());
    let mut max_abs_v_prev = 0.0f64;
    let trace_nodes: Vec<(&str, usize)> = std::env::var("MELANGE_TRACE_NODES")
        .ok()
        .map(|s| s.split(',').filter_map(|tok| {{
            let mut parts = tok.splitn(2, '=');
            let name = parts.next()?.to_string();
            let idx: usize = parts.next()?.parse().ok()?;
            Some((name, idx))
        }}).collect::<Vec<_>>())
        .unwrap_or_default()
        .into_iter()
        .map(|(n, i)| (Box::leak(n.into_boxed_str()) as &str, i))
        .collect();
    let trace_every: usize = std::env::var("MELANGE_TRACE_EVERY").ok().and_then(|s| s.parse().ok()).unwrap_or(500);
    for (i, &s) in samples.iter().enumerate() {{
        let out = process_sample(s, &mut state);
        output.push(out[0]);
{probe_emit}        for &v in &state.v_prev {{
            if v.abs() > max_abs_v_prev {{ max_abs_v_prev = v.abs(); }}
        }}
        if !trace_nodes.is_empty() && (i % trace_every == 0 || out[0].abs() > 1e3) {{
            let mut buf = format!("DIAG:TRACE s={{}} ", i);
            for (name, idx) in &trace_nodes {{
                buf.push_str(&format!("{{}}[{{}}]={{:.4}} ", name, idx, state.v_prev[*idx]));
            }}
            for k in 0..state.i_nl_prev.len() {{
                buf.push_str(&format!("i[{{}}]={{:.4e}} ", k, state.i_nl_prev[k]));
            }}
            eprintln!("{{}}", buf);
            if out[0].abs() > 1e3 {{ break; }}
        }}
    }}

    write_wav(output_path, sr as u32, &output);
{probe_close}
    // Diagnostics
    let peak = output.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
    eprintln!("DIAG:samples={{}}", output.len());
    eprintln!("DIAG:peak={{:.6}}", peak);
    eprintln!("DIAG:nr_max_iter_count={{}}", state.diag_nr_max_iter_count);
    eprintln!("DIAG:substep_count={{}}", state.diag_substep_count);
    eprintln!("DIAG:nan_reset_count={{}}", state.diag_nan_reset_count);
    eprintln!("DIAG:be_fallback_count={{}}", state.diag_be_fallback_count);
    eprintln!("DIAG:max_abs_v_prev={{:.6}}", max_abs_v_prev);
}}
"#,
        amp = amplitude.unwrap_or(0.5),
    )
}

/// Generate a `fn main()` for the `analyze` command.
///
/// The binary runs a frequency sweep internally and outputs CSV to stdout.
///
/// `harmonics`: 0 = fundamental only (legacy 3-column CSV). N>0 measures the
/// fundamental plus H2..HN on the same sample run — the window is already an
/// integer number of fundamental cycles, so harmonic k also sits on an integer
/// bin and uses the same single-bin DFT. Bins at or above Nyquist are reported
/// as `nan` rather than aliased.
///
/// When `harmonics>0` an extra `nyquist_dbc` column is appended, reporting the
/// peak amplitude at exactly SR/2 (correlated against `(-1)^n`) in dB relative
/// to the fundamental. This catches trap-rule numerical limit cycles and any
/// other persistent sample-rate alternation that sits above every usable
/// harmonic bin. `nan` means the fundamental is too small to make a ratio
/// meaningful.
pub fn generate_analyze_main(
    frequencies: &[f64],
    amplitude: f64,
    sample_rate: f64,
    settle_secs: f64,
    pot_calls: &[String],
    switch_calls: &[String],
    harmonics: usize,
) -> String {
    let pot_lines: String = pot_calls
        .iter()
        .map(|c| format!("    {c};\n"))
        .collect();
    let switch_lines: String = switch_calls
        .iter()
        .map(|c| format!("    {c};\n"))
        .collect();

    let freq_list: String = frequencies
        .iter()
        .map(|f| format!("{f:.6}"))
        .collect::<Vec<_>>()
        .join(", ");

    // Header extras. When harmonics==0 the CSV is byte-identical
    // to the pre-feature version.
    let extra_header: String = if harmonics == 0 {
        String::new()
    } else {
        let mut header = String::from(",thd_pct");
        for k in 2..=harmonics {
            header.push_str(&format!(",h{k}_dbc"));
        }
        header.push_str(",nyquist_dbc");
        header
    };

    format!(
        r#"
fn main() {{
    // Number of sine cycles integrated per frequency point for the 1-bin
    // DFT. 10 gives ~40 dB SNR on a settled linear response — plenty for
    // the passband / rolloff / linearity checks this tool ships. Raise
    // here (not via a magic number anywhere else) if a specific circuit
    // needs lower noise floor at a particular frequency band.
    const DFT_CYCLES: usize = 10;
    const HARMONICS: usize = {harmonics};

    let mut state = CircuitState::default();
{pot_lines}{switch_lines}    state.set_sample_rate({sample_rate:.1});

    let freqs: &[f64] = &[{freq_list}];
    let amplitude = {amplitude};
    let sr = {sample_rate:.1};
    let settle_samples = ({settle_secs:.1} * sr) as usize;

    // Settle once, outside the frequency loop. The state carries over
    // between frequency points — the DFT integrates over integer cycles
    // of the new drive, which rejects both DC and any residual component
    // at the previous frequency. Circuits with time constants longer
    // than the DFT window would bias the result, so if a future circuit
    // needs per-point settle back, expose a `--cold-start` flag on
    // `melange analyze` and re-introduce `state = base_state.clone()`
    // plus an inner settle loop here.
    for _ in 0..settle_samples {{
        process_sample(0.0, &mut state);
    }}

    println!("frequency_hz,gain_db,phase_deg{extra_header}");
    // Per-harmonic DFT accumulators; index 0 = fundamental, k-1 = Hk.
    // Allocated once, zeroed per frequency point.
    let max_bins = if HARMONICS == 0 {{ 1 }} else {{ HARMONICS }};
    let mut sum_cos = vec![0.0f64; max_bins];
    let mut sum_sin = vec![0.0f64; max_bins];
    for &freq in freqs {{
        // Measure: single-bin DFT over `DFT_CYCLES` integer cycles of the
        // fundamental. Harmonic k has k*DFT_CYCLES cycles in that window —
        // still integer, so the k-th bin is clean against the others.
        let measure_samples = ((DFT_CYCLES as f64) * sr / freq).round() as usize;

        for s in sum_cos.iter_mut() {{ *s = 0.0; }}
        for s in sum_sin.iter_mut() {{ *s = 0.0; }}
        let mut sum_in_cos = 0.0f64;
        let mut sum_in_sin = 0.0f64;
        // Correlation of output against (-1)^n — the exact DFT bin at SR/2.
        // Used only when HARMONICS>0 to report `nyquist_dbc`, but accumulated
        // unconditionally since a branch inside the sample loop is worse than
        // an unused f64 add.
        let mut sum_nyquist = 0.0f64;
        for i in 0..measure_samples {{
            let t = i as f64 / sr;
            let phase = 2.0 * std::f64::consts::PI * freq * t;
            let input = amplitude * phase.sin();
            let out = process_sample(input, &mut state);
            let output = out[0];

            sum_in_cos += input * phase.cos();
            sum_in_sin += input * phase.sin();

            let sign = if i & 1 == 0 {{ 1.0 }} else {{ -1.0 }};
            sum_nyquist += output * sign;

            for k in 1..=max_bins {{
                let kphase = (k as f64) * phase;
                sum_cos[k - 1] += output * kphase.cos();
                sum_sin[k - 1] += output * kphase.sin();
            }}
        }}

        let n = measure_samples as f64;
        // Fundamental = bin 0.
        let out_mag = ((2.0 * sum_cos[0] / n).powi(2) + (2.0 * sum_sin[0] / n).powi(2)).sqrt();
        let in_mag = ((2.0 * sum_in_cos / n).powi(2) + (2.0 * sum_in_sin / n).powi(2)).sqrt();

        let gain = if in_mag > 1e-30 {{ out_mag / in_mag }} else {{ 0.0 }};
        let gain_db = if gain > 1e-30 {{ 20.0 * gain.log10() }} else {{ -200.0 }};

        let out_phase = (2.0 * sum_sin[0] / n).atan2(2.0 * sum_cos[0] / n);
        let in_phase = (2.0 * sum_in_sin / n).atan2(2.0 * sum_in_cos / n);
        let phase_diff = (out_phase - in_phase).to_degrees();

        if HARMONICS == 0 {{
            println!("{{:.2}},{{:.4}},{{:.2}}", freq, gain_db, phase_diff);
            eprintln!("  {{:.1}} Hz: {{:.2}} dB, {{:.1}}°", freq, gain_db, phase_diff);
        }} else {{
            // Magnitudes per harmonic. `h1_mag` is the fundamental (bin 0).
            // A harmonic above Nyquist has no physical content — the discrete
            // cosine/sine at (k*f) hits aliased bins and the computed "mag" is
            // meaningless. Report `nan` so downstream tooling can skip it.
            let nyquist = sr * 0.5;
            let mut mags = vec![0.0f64; HARMONICS];
            for k in 1..=HARMONICS {{
                let fk = (k as f64) * freq;
                if fk >= nyquist {{
                    mags[k - 1] = f64::NAN;
                }} else {{
                    let c = 2.0 * sum_cos[k - 1] / n;
                    let s = 2.0 * sum_sin[k - 1] / n;
                    mags[k - 1] = (c * c + s * s).sqrt();
                }}
            }}
            let h1 = mags[0];
            // THD = sqrt(sum k>=2 of Hk^2) / H1; NaN-valued harmonics drop out.
            let mut sq_sum = 0.0f64;
            for k in 2..=HARMONICS {{
                let m = mags[k - 1];
                if m.is_finite() {{ sq_sum += m * m; }}
            }}
            let thd_pct = if h1 > 1e-30 {{ (sq_sum.sqrt() / h1) * 100.0 }} else {{ f64::NAN }};

            // Nyquist bin — correlation of output with (-1)^n recovers the
            // peak amplitude of any component at exactly SR/2. Unlike interior
            // DFT bins there is no matching sine term (sin(π·n) ≡ 0), so the
            // peak-amplitude scaling is |sum|/N rather than 2·|sum|/N.
            let nyq_mag = sum_nyquist.abs() / n;
            let nyquist_dbc = if h1 > 1e-30 && nyq_mag > 1e-30 {{
                20.0 * (nyq_mag / h1).log10()
            }} else {{
                f64::NEG_INFINITY
            }};

            print!("{{:.2}},{{:.4}},{{:.2}}", freq, gain_db, phase_diff);
            print!(",{{:.4}}", thd_pct);
            for k in 2..=HARMONICS {{
                let m = mags[k - 1];
                if !m.is_finite() || h1 <= 1e-30 {{
                    print!(",nan");
                }} else {{
                    let dbc = 20.0 * (m / h1).log10();
                    print!(",{{:.2}}", dbc);
                }}
            }}
            if nyquist_dbc.is_finite() {{
                print!(",{{:.2}}", nyquist_dbc);
            }} else {{
                print!(",nan");
            }}
            println!();
            eprintln!(
                "  {{:.1}} Hz: {{:.2}} dB, {{:.1}}°, THD={{:.3}}%",
                freq, gain_db, phase_diff, thd_pct
            );
        }}
    }}
}}
"#,
    )
}
