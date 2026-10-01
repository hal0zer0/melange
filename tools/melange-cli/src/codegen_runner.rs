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
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, SystemTime};

/// A CLI-supplied drive source for a `.inject` field (`--inject FIELD=SPEC`).
/// The value produced is CIRCUIT VOLTS injected at the `.inject` node through
/// its declared impedance.
#[derive(Debug, Clone, Copy)]
pub enum InjectSource {
    /// `sine:<freq_hz>:<amp_volts>` — a sine at circuit volts.
    Sine { freq: f64, amp: f64 },
    /// `dc:<volts>` — a constant voltage.
    Dc { v: f64 },
}

/// Parse one `--inject FIELD=SPEC` value into `(field_name, source)`.
/// SPEC is `sine:<freq>:<amp>` or `dc:<v>`.
pub fn parse_inject_drive(s: &str) -> Result<(String, InjectSource)> {
    let (field, spec) = s.split_once('=').ok_or_else(|| {
        anyhow::anyhow!("--inject '{s}' must be FIELD=SPEC, e.g. FIELD=sine:1000:5 or FIELD=dc:2.5")
    })?;
    let field = field.trim();
    if field.is_empty() {
        anyhow::bail!("--inject '{s}': empty field name before '='");
    }
    let parts: Vec<&str> = spec.split(':').collect();
    let source = match parts.as_slice() {
        ["sine", f, a] => {
            let freq: f64 = f
                .parse()
                .map_err(|_| anyhow::anyhow!("--inject '{s}': invalid sine frequency '{f}'"))?;
            let amp: f64 = a
                .parse()
                .map_err(|_| anyhow::anyhow!("--inject '{s}': invalid sine amplitude '{a}'"))?;
            if !(freq > 0.0 && freq.is_finite()) || !amp.is_finite() {
                anyhow::bail!("--inject '{s}': sine freq must be > 0 and amp finite");
            }
            InjectSource::Sine { freq, amp }
        }
        ["dc", v] => {
            let v: f64 = v
                .parse()
                .map_err(|_| anyhow::anyhow!("--inject '{s}': invalid dc value '{v}'"))?;
            if !v.is_finite() {
                anyhow::bail!("--inject '{s}': dc value must be finite");
            }
            InjectSource::Dc { v }
        }
        _ => anyhow::bail!(
            "--inject '{s}': SPEC must be sine:<freq>:<amp> or dc:<volts>, got '{spec}'"
        ),
    };
    Ok((field.to_string(), source))
}

/// A compiled circuit binary ready to run.
pub struct CompiledBinary {
    /// Path to the compiled binary.
    pub path: PathBuf,
    /// Whether this binary was loaded from cache (vs freshly compiled).
    pub cached: bool,
}

/// Environment variable capping the binary cache, in MiB; `0` = no limit.
pub const BINARY_CACHE_MAX_ENV: &str = "MELANGE_BINARY_CACHE_MAX_MB";

/// The binary cache's default cap: 2 GiB. Each binary is a few MB, and every
/// distinct deck × flags × pot setting is a new one, so without a cap the
/// directory only grows (one machine reached 23-27 GB).
pub const DEFAULT_BINARY_CACHE_MAX_BYTES: u64 = 2 * 1024 * 1024 * 1024;

/// A `_tmp_` file older than this is a crashed compile's leftover, not one in
/// flight in another process, and eviction may remove it.
const STALE_TMP_AGE: Duration = Duration::from_secs(60 * 60);

/// The binary cache's cap in bytes from [`BINARY_CACHE_MAX_ENV`]; `None` = no
/// limit. An unparseable value warns and keeps the default.
pub fn binary_cache_cap() -> Option<u64> {
    let raw = std::env::var(BINARY_CACHE_MAX_ENV).ok();
    let (cap, warning) = parse_binary_cache_cap(raw.as_deref());
    if let Some(w) = warning {
        eprintln!("warning: {w}");
    }
    cap
}

/// [`binary_cache_cap`] without the environment: the cap for a raw env value,
/// and the warning to print for a value that is not a whole number of MiB.
fn parse_binary_cache_cap(raw: Option<&str>) -> (Option<u64>, Option<String>) {
    let Some(raw) = raw else {
        return (Some(DEFAULT_BINARY_CACHE_MAX_BYTES), None);
    };
    match raw.trim().parse::<u64>() {
        Ok(0) => (None, None),
        Ok(mib) => (Some(mib.saturating_mul(1024 * 1024)), None),
        Err(_) => (
            Some(DEFAULT_BINARY_CACHE_MAX_BYTES),
            Some(format!(
                "{BINARY_CACHE_MAX_ENV}='{raw}' is not a whole number of MiB; using the \
                 default {} MiB",
                DEFAULT_BINARY_CACHE_MAX_BYTES / (1024 * 1024)
            )),
        ),
    }
}

/// A cap for display: its size, or "none".
pub fn format_cap(cap: Option<u64>) -> String {
    match cap {
        Some(bytes) => CacheStats {
            total_files: 0,
            total_bytes: bytes,
        }
        .formatted_size(),
        None => "none".to_string(),
    }
}

/// What one eviction pass removed.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct Evicted {
    pub files: usize,
    pub bytes: u64,
}

/// Binary cache for compiled circuit code.
///
/// Stores compiled binaries in `~/.cache/melange/binaries/` keyed by
/// hash of the source code. Same circuit+config = skip compilation.
///
/// Size-capped, least recently used first: a cache hit refreshes the binary's
/// mtime, and after each new binary is placed the oldest are removed until the
/// directory is under [`binary_cache_cap`]. Eviction is best-effort — a
/// failure warns and never fails the command — and never removes the binary
/// just built.
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
            format!("Failed to create binary cache dir: {}", cache_dir.display())
        })?;

        Ok(Self { cache_dir })
    }

    /// A cache rooted at `cache_dir` (tests).
    #[cfg(test)]
    fn with_dir(cache_dir: PathBuf) -> Self {
        Self { cache_dir }
    }

    /// Where compiled simulate/analyze binaries are kept.
    pub fn cache_dir(&self) -> &std::path::Path {
        &self.cache_dir
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
        // Diagnostic: MELANGE_DUMP_SOURCE=<dir> writes the exact source a verb
        // compiles to <dir>/<name>.rs, so two builds of one deck can be diffed
        // (the cache itself keeps only binaries).
        if let Some(dir) = std::env::var_os("MELANGE_DUMP_SOURCE") {
            let dir = std::path::PathBuf::from(dir);
            let _ = std::fs::create_dir_all(&dir);
            let _ = std::fs::write(dir.join(format!("{name}.rs")), source);
        }
        let hash = hash_source(source);
        let bin_name = format!("melange_{name}_{hash:016x}");
        let bin_path = self.cache_dir.join(&bin_name);

        // Check cache
        if bin_path.exists() {
            // Mark it used, so eviction keeps the binaries people run.
            touch(&bin_path);
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

        if let Some(cap) = binary_cache_cap() {
            if let Err(e) = self.evict_to(cap, &bin_path) {
                eprintln!(
                    "warning: could not trim the binary cache at {} ({e}); `melange cache \
                     clear --binaries` empties it",
                    self.cache_dir.display()
                );
            }
        }

        Ok(CompiledBinary {
            path: bin_path,
            cached: false,
        })
    }

    /// Remove the least recently used binaries (oldest mtime first) until the
    /// cache holds at most `cap` bytes. `keep` is never removed, even when it
    /// alone exceeds the cap. A `_tmp_` file is removed only once it is older
    /// than [`STALE_TMP_AGE`]: a younger one may be another process's compile
    /// in flight. Only `melange_*` files are touched.
    fn evict_to(&self, cap: u64, keep: &Path) -> std::io::Result<Evicted> {
        let now = SystemTime::now();
        let mut total = 0u64;
        let mut candidates: Vec<(SystemTime, u64, PathBuf)> = Vec::new();
        for entry in std::fs::read_dir(&self.cache_dir)? {
            let Ok(entry) = entry else { continue };
            let name = entry.file_name();
            let Some(name) = name.to_str() else { continue };
            if !name.starts_with("melange_") {
                continue;
            }
            let Ok(meta) = entry.metadata() else { continue };
            if !meta.is_file() {
                continue;
            }
            total += meta.len();
            let path = entry.path();
            if path == keep {
                continue;
            }
            let mtime = meta.modified().unwrap_or(SystemTime::UNIX_EPOCH);
            if name.contains("_tmp_")
                && now.duration_since(mtime).unwrap_or(Duration::ZERO) < STALE_TMP_AGE
            {
                continue;
            }
            candidates.push((mtime, meta.len(), path));
        }
        let mut evicted = Evicted::default();
        if total <= cap {
            return Ok(evicted);
        }
        candidates.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.2.cmp(&b.2)));
        for (_, len, path) in candidates {
            if total <= cap {
                break;
            }
            match std::fs::remove_file(&path) {
                Ok(()) => {
                    evicted.files += 1;
                    evicted.bytes += len;
                    total = total.saturating_sub(len);
                }
                // Another process removed it first: the space is free either way.
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                    total = total.saturating_sub(len);
                }
                Err(e) => return Err(e),
            }
        }
        Ok(evicted)
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

/// Refresh a cached binary's mtime (its "last used" time for eviction).
/// Best-effort: opened read-only, so a binary another process is executing
/// can still be touched; a failure only makes the binary look older.
fn touch(path: &Path) {
    if let Ok(f) = std::fs::File::open(path) {
        let _ = f.set_modified(SystemTime::now());
    }
}

/// Hash source code for cache key.
fn hash_source(source: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    source.hash(&mut hasher);
    hasher.finish()
}

// ── Main generation templates ──────────────────────────────────────────

/// What [`generate_simulate_main`] emits (named, so two same-typed options
/// cannot be passed in each other's place).
pub struct SimulateMain<'a> {
    pub sample_rate: f64,
    pub pot_calls: &'a [String],
    pub switch_calls: &'a [String],
    pub amplitude: Option<f64>,
    pub freq: f64,
    pub duration_secs: f64,
    pub probe_names: &'a [&'a str],
    pub noise_enabled: bool,
    pub inject_driven: &'a [(usize, InjectSource)],
    pub num_inject: usize,
    pub extra_diag_counters: &'a [&'a str],
    /// `--pcm16`: write 16-bit signed PCM instead of the default IEEE float32.
    /// Affects the FILE FORMAT only; the rendered samples are identical, and
    /// the DIAG figures are computed from the f64 buffer before encoding.
    pub pcm16: bool,
}

/// Generate a `fn main()` for the `simulate` command.
///
/// The binary reads input WAV from argv[1], writes output WAV to argv[2].
/// Supports both WAV file input and sine test tone generation.
pub fn generate_simulate_main(main: SimulateMain<'_>) -> String {
    let SimulateMain {
        sample_rate,
        pot_calls,
        switch_calls,
        amplitude,
        freq,
        duration_secs,
        probe_names,
        noise_enabled,
        inject_driven,
        num_inject,
        extra_diag_counters,
        pcm16,
    } = main;
    // Optional `CircuitState` u64 diagnostic counters that exist only on some
    // builds (e.g. `diag_subsample_fire_count` on glow nodal-Schur decks).
    // Printed as `DIAG:<name without diag_>=<value>` after the fixed set.
    let extra_diag_lines: String = extra_diag_counters
        .iter()
        .map(|f| {
            let key = f.strip_prefix("diag_").unwrap_or(f);
            format!("    eprintln!(\"DIAG:{key}={{}}\", state.{f});\n")
        })
        .collect();
    let pot_lines: String = pot_calls.iter().map(|c| format!("    {c};\n")).collect();
    let switch_lines: String = switch_calls.iter().map(|c| format!("    {c};\n")).collect();
    // `--noise <mode>` bakes the noise machinery into codegen (state fields,
    // RNG seeding, per-sample injection, `set_noise_enabled` method — see
    // `noise.enabled` gating in dk_emitter.rs/nodal_emitter.rs), but the
    // runtime master switch defaults OFF (`noise_enabled: false` in
    // `CircuitState::default()`) so a shipped plugin starts silent until the
    // host UI opts in. For `simulate`/`analyze`, passing `--noise <mode>` IS
    // the opt-in — without this call the CLI compiles in the full noise
    // machinery and then runs it switched off, producing output that is
    // byte-identical and seed-invariant to `--noise off` with no warning.
    let noise_enable_line = if noise_enabled {
        "    state.set_noise_enabled(true);\n"
    } else {
        ""
    };

    // Embed minimal WAV reader/writer
    let wav_code = include_str!("wav_embed.rs.inc");
    // Baked into the generated source (not passed at runtime) so the binary
    // cache keys on it — a float32 build and a PCM16 build are different
    // binaries, not one binary reused with the wrong writer.
    let pcm16_literal = if pcm16 { "true" } else { "false" };

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
        String::from(
            r#"    probe_writer.flush().ok();
    eprintln!("DIAG:probes_written={}", samples.len());
"#,
        )
    } else {
        String::new()
    };

    // `.inject` drive: when the deck has injections (NUM_INJECT>0), every
    // process_sample call takes an injection array. Fill it per inner
    // (oversampled) sample from the `--inject` sources; undriven fields stay 0.
    let driven_lines: String = inject_driven
        .iter()
        .map(|(idx, src)| match src {
            InjectSource::Sine { freq, amp } => format!(
                "            melange_inj[j][{idx}] = {amp:.17e} * (2.0 * std::f64::consts::PI * {freq:.17e} * ti).sin();\n"
            ),
            InjectSource::Dc { v } => format!("            melange_inj[j][{idx}] = {v:.17e};\n"),
        })
        .collect();
    let uses_ti = inject_driven
        .iter()
        .any(|(_, s)| matches!(s, InjectSource::Sine { .. }));
    let inject_fill: String = if num_inject > 0 {
        let ti = if uses_ti { "ti" } else { "_ti" };
        format!(
            "        let mut melange_inj = [[0.0f64; NUM_INJECT]; OVERSAMPLING_FACTOR];\n\
             \x20       for j in 0..OVERSAMPLING_FACTOR {{\n\
             \x20           let {ti} = (i as f64) / sr + (j as f64) / (sr * OVERSAMPLING_FACTOR as f64);\n\
             {driven_lines}        }}\n"
        )
    } else {
        String::new()
    };
    // An `.inject`/`.tap` deck's process_sample takes the injection array and
    // returns `(outputs, taps)`; a plain deck takes only the input and returns
    // the outputs array. Bind `out` to the outputs either way.
    let process_stmt: &str = if num_inject > 0 {
        "let (out, _melange_taps) = process_sample(s, &melange_inj, &mut state);"
    } else {
        "let out = process_sample(s, &mut state);"
    };

    format!(
        r#"{wav_code}

fn main() {{
    let args: Vec<String> = std::env::args().collect();

    let mut state = CircuitState::default();
{noise_enable_line}{pot_lines}{switch_lines}
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
{inject_fill}        {process_stmt}
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

    write_wav(output_path, sr as u32, &output, {pcm16_literal});
{probe_close}
    // Diagnostics
    let peak = output.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
    eprintln!("DIAG:samples={{}}", output.len());
    // Scientific notation, not {{:.6}}: fixed 6 decimals cannot express a peak
    // below ~5e-7 V, so every genuinely tiny output arrived at the parent as
    // "0.000000" and the silence warning could only ever quote that. The real
    // figure is what tells a broken-wiring zero apart from a very quiet stage.
    eprintln!("DIAG:peak={{:.6e}}", peak);
    eprintln!("DIAG:nr_max_iter_count={{}}", state.diag_nr_max_iter_count);
    eprintln!("DIAG:substep_count={{}}", state.diag_substep_count);
    eprintln!("DIAG:nan_reset_count={{}}", state.diag_nan_reset_count);
    eprintln!("DIAG:magnitude_reset_count={{}}", state.diag_magnitude_reset_count);
    eprintln!("DIAG:be_fallback_count={{}}", state.diag_be_fallback_count);
    eprintln!("DIAG:region_exit_count={{}}", state.diag_region_exit_count);
    eprintln!("DIAG:max_abs_v_prev={{:.6}}", max_abs_v_prev);
{extra_diag_lines}}}
"#,
        amp = amplitude.unwrap_or(0.5),
    )
}

/// Upper edge of the band `analyze`'s `thd_pct` sums over: harmonic k of a
/// point at f enters THD only when k·f is below this AND below Nyquist. The
/// common audio definition (H2..H13 below 20 kHz) that Sensor Array and
/// melange-circuits use; the per-harmonic `hN_dbc` columns are still reported
/// up to Nyquist.
pub const ANALYZE_THD_BAND_HZ: f64 = 20_000.0;

/// The depth below which `analyze` prints a dBc column (`hN_dbc`,
/// `nyquist_dbc`) as `-inf`: floating-point rounding residue, not circuit
/// content. Stated in `analyze --help`.
pub const ANALYZE_DBC_FLOOR: f64 = -200.0;

/// Relative agreement two successive measurements of one point must reach
/// before `analyze` calls it steady state: the complex fundamental (gain AND
/// phase) within 0.1 % of itself (~0.009 dB, ~0.06°), and the harmonic vector
/// H2..HN within 0.1 % of its own magnitude.
pub const ANALYZE_SETTLE_TOL: f64 = 1e-3;

/// Absolute floor of the harmonic-vector agreement, relative to the
/// fundamental (-100 dBc, a THD resolution of 0.001 percentage points). Keeps
/// a near-linear point from chasing numerical noise in harmonics that sit
/// at the solver's tolerance.
pub const ANALYZE_SETTLE_FLOOR: f64 = 1e-5;

/// What [`generate_analyze_main`] emits (named, so two same-typed options
/// cannot be passed in each other's place).
pub struct AnalyzeMain<'a> {
    pub frequencies: &'a [f64],
    pub amplitude: f64,
    pub sample_rate: f64,
    /// Zero-drive settle, once, before the first point (seconds).
    pub settle_secs: f64,
    /// Minimum drive-level pre-roll per point, and the spacing between the
    /// point's successive settle-check measurements (seconds).
    pub preroll_secs: f64,
    /// Cap on drive-level time per point while the settle check repeats.
    /// `0` = no settle check: one measurement after the pre-roll.
    pub preroll_max_secs: f64,
    pub pot_calls: &'a [String],
    pub switch_calls: &'a [String],
    pub harmonics: usize,
    pub noise_enabled: bool,
    /// `CircuitState` u64 counters printed once as `DIAG:<name>=<v>` at the
    /// end of the run (the caller presence-filters them per build).
    pub diag_counters: &'a [&'a str],
    /// `CircuitState` u64 counters whose per-point increments are printed as
    /// `DIAGPT:<freq>:<name>=<delta>` (only nonzero ones), so the caller can
    /// refuse a point whose render was not a solution. Presence-filtered.
    pub point_counters: &'a [&'a str],
}

/// Generate a `fn main()` for the `analyze` command.
///
/// The binary runs a frequency sweep internally and outputs CSV to stdout.
///
/// `harmonics`: 0 = fundamental only (legacy 3-column CSV). N>0 measures the
/// fundamental plus H2..HN on the same sample run — the drive frequency is
/// snapped to the exact bin `DFT_CYCLES·sr/N` so the window is an integer
/// number of fundamental cycles, and harmonic k also sits on an integer bin
/// using the same single-bin DFT. The CSV reports the snapped frequency (it
/// can differ from the requested log-spaced value by up to half a sample's
/// worth of period). Bins at or above Nyquist are reported as `nan` rather
/// than aliased. `thd_pct` sums H2..HN below [`ANALYZE_THD_BAND_HZ`] (and
/// Nyquist); `nan` when no harmonic lies in that band.
///
/// When `harmonics>0` an extra `nyquist_dbc` column is appended, reporting the
/// peak amplitude at exactly SR/2 (correlated against `(-1)^n`) in dB relative
/// to the fundamental. This catches trap-rule numerical limit cycles and any
/// other persistent sample-rate alternation that sits above every usable
/// harmonic bin. `nan` means the fundamental is too small to make a ratio
/// meaningful. Any dBc column below [`ANALYZE_DBC_FLOOR`] prints as `-inf`.
///
/// Steady state: each point is driven at its own frequency and amplitude for
/// at least `preroll_secs` (whole DFT windows) before the window it
/// measures, then measured again after another such stretch, until two
/// successive measurements agree ([`ANALYZE_SETTLE_TOL`]) or
/// `preroll_max_secs` is reached. Each point prints
/// `SETTLEPT:<freq>:<secs at drive>:<status>:<fund rel diff>:<harm rel diff>`
/// (status 1 = settled, 0 = cap reached, 2 = unchecked) for the caller.
pub fn generate_analyze_main(main: AnalyzeMain<'_>) -> String {
    let AnalyzeMain {
        frequencies,
        amplitude,
        sample_rate,
        settle_secs,
        preroll_secs,
        preroll_max_secs,
        pot_calls,
        switch_calls,
        harmonics,
        noise_enabled,
        diag_counters,
        point_counters,
    } = main;
    let diag_lines: String = diag_counters
        .iter()
        .map(|f| {
            let key = f.strip_prefix("diag_").unwrap_or(f);
            format!("    eprintln!(\"DIAG:{key}={{}}\", state.{f});\n")
        })
        .collect();
    let npc = point_counters.len();
    let point_counter_names: String = point_counters
        .iter()
        .map(|f| format!("\"{}\"", f.strip_prefix("diag_").unwrap_or(f)))
        .collect::<Vec<_>>()
        .join(", ");
    let point_counter_reads: String = point_counters
        .iter()
        .map(|f| format!("state.{f}"))
        .collect::<Vec<_>>()
        .join(", ");
    let pot_lines: String = pot_calls.iter().map(|c| format!("    {c};\n")).collect();
    let switch_lines: String = switch_calls.iter().map(|c| format!("    {c};\n")).collect();
    // See `generate_simulate_main`'s comment: `--noise <mode>` bakes in the
    // noise machinery, but the runtime master switch defaults OFF. Without
    // this call, `analyze --noise <mode>` silently measures a noise-free
    // circuit and reports success.
    let noise_enable_line = if noise_enabled {
        "    state.set_noise_enabled(true);\n"
    } else {
        ""
    };

    let freq_list: String = frequencies
        .iter()
        .map(|f| format!("{f:.6}"))
        .collect::<Vec<_>>()
        .join(", ");

    // Header extras. When harmonics==0 the CSV header is the legacy one.
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
// Per-point counter increments, printed as `DIAGPT:<label>:<name>=<delta>`
// (nonzero only). Returns a human summary of them ("" when all zero).
fn analyze_report_point_counters(label: &str, before: &[u64], after: &[u64]) -> String {{
    const NAMES: [&str; {npc}] = [{point_counter_names}];
    let mut summary = String::new();
    for (k, name) in NAMES.iter().enumerate() {{
        let delta = after[k].saturating_sub(before[k]);
        if delta > 0 {{
            eprintln!("DIAGPT:{{}}:{{}}={{}}", label, name, delta);
            if !summary.is_empty() {{ summary.push_str(", "); }}
            summary.push_str(&format!("{{}} +{{}}", name, delta));
        }}
    }}
    summary
}}

fn analyze_point_counters(state: &CircuitState) -> [u64; {npc}] {{
    let _ = state;
    [{point_counter_reads}]
}}

fn main() {{
    // Number of sine cycles integrated per frequency point for the 1-bin
    // DFT. 10 gives ~40 dB SNR on a settled linear response — plenty for
    // the passband / rolloff / linearity checks this tool ships. Raise
    // here (not via a magic number anywhere else) if a specific circuit
    // needs lower noise floor at a particular frequency band.
    const DFT_CYCLES: usize = 10;
    const HARMONICS: usize = {harmonics};
    // Steady-state check (see `ANALYZE_SETTLE_TOL` / `_FLOOR` in the CLI).
    const SETTLE_TOL: f64 = {settle_tol:e};
    const SETTLE_FLOOR: f64 = {settle_floor:e};
    // THD sums harmonics below this AND below Nyquist.
    const THD_BAND_HZ: f64 = {thd_band:.1};
    // A dBc column below this prints as `-inf` (rounding residue, not content).
    const DBC_FLOOR: f64 = {dbc_floor:.1};

    let mut state = CircuitState::default();
{noise_enable_line}{pot_lines}{switch_lines}    state.set_sample_rate({sample_rate:.1});

    let freqs: &[f64] = &[{freq_list}];
    let amplitude: f64 = {amplitude:.17e};
    let sr = {sample_rate:.1};
    let settle_samples = ({settle_secs:.6} * sr) as usize;
    let preroll_secs: f64 = {preroll_secs:.17e};
    let preroll_max_secs: f64 = {preroll_max_secs:.17e};

    // Zero-drive settle, once, before the first point: lets the bias network
    // move off the embedded DC operating point (and inductor currents
    // settle) before any drive is applied. Its unsolved samples are reported
    // like a point's, under the label `settle`.
    let c0 = analyze_point_counters(&state);
    for _ in 0..settle_samples {{
        process_sample(0.0, &mut state);
    }}
    let _ = analyze_report_point_counters("settle", &c0, &analyze_point_counters(&state));

    println!("frequency_hz,gain_db,phase_deg{extra_header}");
    // Per-harmonic DFT accumulators; index 0 = fundamental, k-1 = Hk.
    // `prev_*` holds the point's previous measurement for the settle check.
    let max_bins = if HARMONICS == 0 {{ 1 }} else {{ HARMONICS }};
    let mut sum_cos = vec![0.0f64; max_bins];
    let mut sum_sin = vec![0.0f64; max_bins];
    let mut prev_cos = vec![0.0f64; max_bins];
    let mut prev_sin = vec![0.0f64; max_bins];
    for &freq in freqs {{
        // Measure: single-bin DFT over `DFT_CYCLES` integer cycles of the
        // fundamental. Harmonic k has k*DFT_CYCLES cycles in that window —
        // still integer, so the k-th bin is clean against the others.
        // N is kept EVEN: the Nyquist kernel (-1)^n is orthogonal to every
        // integer bin only when N/2 is an integer. An odd N leaked the
        // fundamental into `nyquist_dbc` (a linear RC read -43.7 dBc).
        let measure_samples =
            (2.0 * ((DFT_CYCLES as f64) * sr / freq / 2.0).round()).max(2.0) as usize;
        // Snap the drive frequency to the exact DFT bin DFT_CYCLES·sr/N.
        // Rounding N while keeping the requested frequency leaves up to 0.5
        // samples of cycle mismatch — rectangular-window leakage that floors
        // h2..hN at −50..−75 dBc (and pollutes thd/nyquist) even on perfectly
        // linear circuits. The snapped value is used everywhere below: drive,
        // correlation kernels, and the CSV frequency column.
        let freq = (DFT_CYCLES as f64) * sr / (measure_samples as f64);
        let nyquist = sr * 0.5;
        let n = measure_samples as f64;

        // Steady state at THIS drive. A point is measured only after the
        // circuit has been driven at its own frequency and amplitude for at
        // least `preroll_secs`: the previous point's drive, the frequency
        // switch's broadband transient, and slow bias movement under drive
        // (a sagging RC rail, bypass caps re-centring) all decay first. The
        // drive is run in chunks of whole DFT windows (so its phase is
        // continuous: i and i+measure_samples share the same phase mod 2π),
        // the last window of each chunk is measured, and chunks repeat until
        // two successive measurements agree or `preroll_max_secs` is spent.
        // The reported values are the last measurement's.
        let windows_per_chunk =
            ((preroll_secs * sr / measure_samples as f64).ceil() as usize).max(1);
        let chunk_secs = (windows_per_chunk * measure_samples) as f64 / sr;
        let max_chunks = if preroll_max_secs <= 0.0 {{
            1
        }} else {{
            ((preroll_max_secs / chunk_secs - 1e-9).ceil() as usize).max(2)
        }};
        let counters_before = analyze_point_counters(&state);
        let mut chunks = 0usize;
        // 2 = unchecked (single measurement), 1 = settled, 0 = cap reached.
        let mut status = if max_chunks == 1 {{ 2 }} else {{ 0 }};
        let mut fund_rel = f64::NAN;
        let mut harm_rel = f64::NAN;
        let mut sum_in_cos: f64;
        let mut sum_in_sin: f64;
        // Correlation of output against (-1)^n — the exact DFT bin at SR/2.
        let mut sum_nyquist: f64;
        loop {{
            for _ in 1..windows_per_chunk {{
                for i in 0..measure_samples {{
                    let t = i as f64 / sr;
                    let phase = 2.0 * std::f64::consts::PI * freq * t;
                    process_sample(amplitude * phase.sin(), &mut state);
                }}
            }}

            for s in sum_cos.iter_mut() {{ *s = 0.0; }}
            for s in sum_sin.iter_mut() {{ *s = 0.0; }}
            sum_in_cos = 0.0;
            sum_in_sin = 0.0;
            sum_nyquist = 0.0;
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
            chunks += 1;

            if chunks > 1 {{
                // Complex differences: the windows start at the same drive
                // phase, so a settled circuit repeats each bin exactly.
                let h1 = (sum_cos[0] * sum_cos[0] + sum_sin[0] * sum_sin[0]).sqrt();
                let d1 = ((sum_cos[0] - prev_cos[0]).powi(2)
                    + (sum_sin[0] - prev_sin[0]).powi(2))
                .sqrt();
                let mut dist = 0.0f64;
                let mut dd = 0.0f64;
                for k in 2..=HARMONICS {{
                    if (k as f64) * freq < nyquist {{
                        dist += sum_cos[k - 1] * sum_cos[k - 1] + sum_sin[k - 1] * sum_sin[k - 1];
                        dd += (sum_cos[k - 1] - prev_cos[k - 1]).powi(2)
                            + (sum_sin[k - 1] - prev_sin[k - 1]).powi(2);
                    }}
                }}
                let (dist, dd) = (dist.sqrt(), dd.sqrt());
                fund_rel = if h1 > 0.0 {{ d1 / h1 }} else if d1 == 0.0 {{ 0.0 }} else {{ f64::INFINITY }};
                harm_rel = if dist > 0.0 {{ dd / dist }} else if dd == 0.0 {{ 0.0 }} else {{ f64::INFINITY }};
                if d1 <= SETTLE_TOL * h1 && dd <= SETTLE_TOL * dist + SETTLE_FLOOR * h1 {{
                    status = 1;
                    break;
                }}
            }}
            if chunks >= max_chunks {{
                break;
            }}
            prev_cos.copy_from_slice(&sum_cos);
            prev_sin.copy_from_slice(&sum_sin);
        }}
        let drive_secs = chunks as f64 * chunk_secs;
        eprintln!(
            "SETTLEPT:{{:.2}}:{{:.6}}:{{}}:{{:e}}:{{:e}}",
            freq, drive_secs, status, fund_rel, harm_rel
        );
        let point_label = format!("{{:.2}}", freq);
        let unsolved =
            analyze_report_point_counters(&point_label, &counters_before, &analyze_point_counters(&state));
        let mut note = String::new();
        if !unsolved.is_empty() {{
            note.push_str(&format!("  <-- counters: {{}}", unsolved));
        }}
        if status == 0 {{
            note.push_str(&format!("  <-- not settled after {{:.2}} s", drive_secs));
        }}

        // Fundamental = bin 0.
        let out_mag = ((2.0 * sum_cos[0] / n).powi(2) + (2.0 * sum_sin[0] / n).powi(2)).sqrt();
        let in_mag = ((2.0 * sum_in_cos / n).powi(2) + (2.0 * sum_in_sin / n).powi(2)).sqrt();

        let gain = if in_mag > 1e-30 {{ out_mag / in_mag }} else {{ 0.0 }};
        let gain_db = if gain > 1e-30 {{ 20.0 * gain.log10() }} else {{ -200.0 }};

        // Correlating x(t) = A*sin(wt + phi) against sin(wt) gives (A/2)*cos(phi)
        // and against cos(wt) gives (A/2)*sin(phi). So sum_sin carries the COSINE
        // part and sum_cos the SINE part, and atan2 must be called as
        // atan2(sine_part, cosine_part) = atan2(sum_cos, sum_sin) to recover phi.
        // Passing them the other way round yields (pi/2 - phi), whose difference
        // between output and input is -(phi_out - phi_in) — a sign-inverted phase.
        // That was the behaviour up to and including 0.1.9: an RC low-pass read
        // +45 deg at its cutoff where every other tool reads -45. Reported by
        // melange-circuits with a known-answer probe (cross-project review).
        let out_phase = (2.0 * sum_cos[0] / n).atan2(2.0 * sum_sin[0] / n);
        let in_phase = (2.0 * sum_in_cos / n).atan2(2.0 * sum_in_sin / n);
        let phase_diff = (out_phase - in_phase).to_degrees();

        if HARMONICS == 0 {{
            println!("{{:.2}},{{:.4}},{{:.2}}", freq, gain_db, phase_diff);
            eprintln!("  {{:.1}} Hz: {{:.2}} dB, {{:.1}}°{{}}", freq, gain_db, phase_diff, note);
        }} else {{
            // Magnitudes per harmonic. `h1_mag` is the fundamental (bin 0).
            // A harmonic above Nyquist has no physical content — the discrete
            // cosine/sine at (k*f) hits aliased bins and the computed "mag" is
            // meaningless. Report `nan` so downstream tooling can skip it.
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
            // THD = sqrt(sum of Hk^2, k>=2, k*f below THD_BAND_HZ and Nyquist)
            // / H1. No harmonic in that band (f >= THD_BAND_HZ/2) = no THD:
            // `nan`, not a reassuring 0.
            let mut sq_sum = 0.0f64;
            let mut in_band = 0usize;
            for k in 2..=HARMONICS {{
                let m = mags[k - 1];
                if (k as f64) * freq < THD_BAND_HZ && m.is_finite() {{
                    sq_sum += m * m;
                    in_band += 1;
                }}
            }}
            let thd_pct = if h1 > 1e-30 && in_band > 0 {{
                (sq_sum.sqrt() / h1) * 100.0
            }} else {{
                f64::NAN
            }};

            // Nyquist bin — correlation of output with (-1)^n recovers the
            // peak amplitude of any component at exactly SR/2. Unlike interior
            // DFT bins there is no matching sine term (sin(π·n) ≡ 0), so the
            // peak-amplitude scaling is |sum|/N rather than 2·|sum|/N.
            let nyq_mag = sum_nyquist.abs() / n;
            let nyquist_dbc = if h1 > 1e-30 {{
                20.0 * (nyq_mag / h1).log10()
            }} else {{
                f64::NAN
            }};

            print!("{{:.2}},{{:.4}},{{:.2}}", freq, gain_db, phase_diff);
            if thd_pct.is_finite() {{
                print!(",{{:.4}}", thd_pct);
            }} else {{
                print!(",nan");
            }}
            // A dBc figure: `nan` = not measurable, `-inf` = below DBC_FLOOR.
            let print_dbc = |dbc: f64| {{
                if dbc.is_nan() {{
                    print!(",nan");
                }} else if dbc < DBC_FLOOR {{
                    print!(",-inf");
                }} else {{
                    print!(",{{:.2}}", dbc);
                }}
            }};
            for k in 2..=HARMONICS {{
                let m = mags[k - 1];
                if !m.is_finite() || h1 <= 1e-30 {{
                    print_dbc(f64::NAN);
                }} else {{
                    print_dbc(20.0 * (m / h1).log10());
                }}
            }}
            print_dbc(nyquist_dbc);
            println!();
            let thd_text = if thd_pct.is_finite() {{
                format!("{{:.3}}%", thd_pct)
            }} else {{
                "n/a".to_string()
            }};
            eprintln!(
                "  {{:.1}} Hz: {{:.2}} dB, {{:.1}}°, THD={{}}{{}}",
                freq, gain_db, phase_diff, thd_text, note
            );
        }}
    }}
{diag_lines}}}
"#,
        settle_tol = ANALYZE_SETTLE_TOL,
        settle_floor = ANALYZE_SETTLE_FLOOR,
        thd_band = ANALYZE_THD_BAND_HZ,
        dbc_floor = ANALYZE_DBC_FLOOR,
    )
}

#[cfg(test)]
mod binary_cache_eviction_tests {
    use super::*;

    /// A file of `len` bytes whose mtime is `age_s` seconds in the past.
    fn put(dir: &Path, name: &str, len: usize, age_s: u64) -> PathBuf {
        let path = dir.join(name);
        std::fs::write(&path, vec![0u8; len]).unwrap();
        let f = std::fs::File::options().write(true).open(&path).unwrap();
        f.set_modified(SystemTime::now() - Duration::from_secs(age_s))
            .unwrap();
        path
    }

    #[test]
    fn evicts_least_recently_used_until_under_cap() {
        let tmp = tempfile::tempdir().unwrap();
        let cache = BinaryCache::with_dir(tmp.path().to_path_buf());
        let oldest = put(tmp.path(), "melange_simulate_a", 100, 300);
        let middle = put(tmp.path(), "melange_simulate_b", 100, 200);
        let newest = put(tmp.path(), "melange_simulate_c", 100, 100);
        let just_built = put(tmp.path(), "melange_simulate_d", 100, 0);

        let evicted = cache.evict_to(250, &just_built).unwrap();
        assert_eq!(
            evicted,
            Evicted {
                files: 2,
                bytes: 200
            }
        );
        assert!(!oldest.exists() && !middle.exists());
        assert!(newest.exists() && just_built.exists());
    }

    #[test]
    fn under_cap_removes_nothing() {
        let tmp = tempfile::tempdir().unwrap();
        let cache = BinaryCache::with_dir(tmp.path().to_path_buf());
        let a = put(tmp.path(), "melange_analyze_a", 100, 300);
        let b = put(tmp.path(), "melange_analyze_b", 100, 0);
        assert_eq!(cache.evict_to(200, &b).unwrap(), Evicted::default());
        assert!(a.exists() && b.exists());
    }

    /// The binary just built survives even when it is the oldest file and
    /// alone exceeds the cap: the command is about to run it.
    #[test]
    fn never_evicts_the_just_built_binary() {
        let tmp = tempfile::tempdir().unwrap();
        let cache = BinaryCache::with_dir(tmp.path().to_path_buf());
        let keep = put(tmp.path(), "melange_simulate_keep", 500, 1000);
        let other = put(tmp.path(), "melange_simulate_other", 100, 10);
        cache.evict_to(50, &keep).unwrap();
        assert!(keep.exists());
        assert!(!other.exists());
    }

    /// A young `_tmp_` file may be another process's compile in flight; an old
    /// one is a crash leftover. Files that are not melange's are never touched.
    #[test]
    fn spares_in_flight_temp_files_and_foreign_files() {
        let tmp = tempfile::tempdir().unwrap();
        let cache = BinaryCache::with_dir(tmp.path().to_path_buf());
        let in_flight = put(tmp.path(), "melange_simulate_x_tmp_42.rs", 100, 60);
        let stale = put(tmp.path(), "melange_simulate_y_tmp_7", 100, 2 * 3600);
        let foreign = put(tmp.path(), "notes.txt", 100, 5000);
        let keep = put(tmp.path(), "melange_simulate_z", 100, 0);
        cache.evict_to(0, &keep).unwrap();
        assert!(in_flight.exists(), "an in-flight compile must survive");
        assert!(!stale.exists(), "a crash leftover is evictable");
        assert!(foreign.exists(), "only melange_* files are the cache's");
        assert!(keep.exists());
    }

    /// A cache hit refreshes the mtime, so a binary in use is not the next
    /// one evicted.
    #[test]
    fn touch_marks_a_binary_recently_used() {
        let tmp = tempfile::tempdir().unwrap();
        let cache = BinaryCache::with_dir(tmp.path().to_path_buf());
        let hit = put(tmp.path(), "melange_simulate_hit", 100, 500);
        let cold = put(tmp.path(), "melange_simulate_cold", 100, 100);
        touch(&hit);
        let keep = put(tmp.path(), "melange_simulate_new", 100, 0);
        cache.evict_to(200, &keep).unwrap();
        assert!(hit.exists(), "the touched binary is the most recently used");
        assert!(!cold.exists());
    }

    #[test]
    fn cap_from_env_value() {
        let default = Some(DEFAULT_BINARY_CACHE_MAX_BYTES);
        assert_eq!(parse_binary_cache_cap(None), (default, None));
        assert_eq!(parse_binary_cache_cap(Some("0")), (None, None));
        assert_eq!(
            parse_binary_cache_cap(Some(" 512 ")),
            (Some(512 * 1024 * 1024), None)
        );
        let (cap, warning) = parse_binary_cache_cap(Some("2GB"));
        assert_eq!(cap, default);
        assert!(warning.unwrap().contains(BINARY_CACHE_MAX_ENV));
        assert_eq!(format_cap(None), "none");
        assert_eq!(format_cap(Some(2 * 1024 * 1024 * 1024)), "2048.0 MB");
    }
}
