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
        eprintln!("Usage: binary <input.wav|--tone> <output.wav>");
        std::process::exit(1);
    }};

    state.set_sample_rate(sr);

    let output_path = args.get(2).map(|s| s.as_str()).unwrap_or("output.wav");

    let mut output = Vec::with_capacity(samples.len());
    for &s in &samples {{
        let out = process_sample(s, &mut state);
        output.push(out[0]);
    }}

    write_wav(output_path, sr as u32, &output);

    // Diagnostics
    let peak = output.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
    eprintln!("DIAG:samples={{}}", output.len());
    eprintln!("DIAG:peak={{:.6}}", peak);
    eprintln!("DIAG:nr_max_iter_count={{}}", state.diag_nr_max_iter_count);
    eprintln!("DIAG:nan_reset_count={{}}", state.diag_nan_reset_count);
}}
"#,
        amp = amplitude.unwrap_or(0.5),
    )
}

/// Generate a `fn main()` for the `analyze` command.
///
/// The binary runs a frequency sweep internally and outputs CSV to stdout.
pub fn generate_analyze_main(
    frequencies: &[f64],
    amplitude: f64,
    sample_rate: f64,
    settle_secs: f64,
    pot_calls: &[String],
    switch_calls: &[String],
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

    format!(
        r#"
fn main() {{
    let mut base_state = CircuitState::default();
{pot_lines}{switch_lines}    base_state.set_sample_rate({sample_rate:.1});

    let freqs: &[f64] = &[{freq_list}];
    let amplitude = {amplitude};
    let sr = {sample_rate:.1};
    let settle_samples = ({settle_secs:.1} * sr) as usize;

    println!("frequency_hz,gain_db,phase_deg");
    for &freq in freqs {{
        let mut state = base_state.clone();

        // Settle: run silent samples then sine to reach steady state
        for _ in 0..settle_samples {{
            process_sample(0.0, &mut state);
        }}

        // Measure: single-bin DFT over integer number of cycles
        let cycles = ((sr / freq).ceil() as usize).max(10);
        let measure_samples = ((cycles as f64) * sr / freq).round() as usize;

        let mut sum_cos = 0.0f64;
        let mut sum_sin = 0.0f64;
        let mut sum_in_cos = 0.0f64;
        let mut sum_in_sin = 0.0f64;
        for i in 0..measure_samples {{
            let t = i as f64 / sr;
            let phase = 2.0 * std::f64::consts::PI * freq * t;
            let input = amplitude * phase.sin();
            let out = process_sample(input, &mut state);
            let output = out[0];

            sum_cos += output * phase.cos();
            sum_sin += output * phase.sin();
            sum_in_cos += input * phase.cos();
            sum_in_sin += input * phase.sin();
        }}

        let n = measure_samples as f64;
        let out_mag = ((2.0 * sum_cos / n).powi(2) + (2.0 * sum_sin / n).powi(2)).sqrt();
        let in_mag = ((2.0 * sum_in_cos / n).powi(2) + (2.0 * sum_in_sin / n).powi(2)).sqrt();

        let gain = if in_mag > 1e-30 {{ out_mag / in_mag }} else {{ 0.0 }};
        let gain_db = if gain > 1e-30 {{ 20.0 * gain.log10() }} else {{ -200.0 }};

        let out_phase = (2.0 * sum_sin / n).atan2(2.0 * sum_cos / n);
        let in_phase = (2.0 * sum_in_sin / n).atan2(2.0 * sum_in_cos / n);
        let phase_diff = (out_phase - in_phase).to_degrees();

        println!("{{:.2}},{{:.4}},{{:.2}}", freq, gain_db, phase_diff);
        eprintln!("  {{:.1}} Hz: {{:.2}} dB, {{:.1}}°", freq, gain_db, phase_diff);
    }}
}}
"#,
    )
}
