//! Compare mode: diff two baseline directories, per circuit per program.
//!
//! Classification:
//! - IDENTICAL — PCM files byte-equal.
//! - NEGLIGIBLE — |RMS level delta| < 0.01 dB AND correlation > 0.99999,
//!   same length, no NaN/Inf introduced. Also applied when both renders sit
//!   below the structural residue floor (peak < -120 dBFS both sides AND
//!   max sample delta < 1e-6): differences there are far below audibility
//!   and below the solver's documented silence-residue floor.
//! - CHANGED — everything else, sorted by severity, loud.
//! - INPUT-CHANGED — changed, but the netlist content hash also differs
//!   between baselines: the .cir input moved, not the compiler.
//! - MISSING — present on one side only, or failed to capture.

use crate::stats::{Stats, SILENT_DB};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

#[derive(Serialize, Clone)]
struct PairResult {
    plugin: String,
    program: String,
    class: String,
    severity: f64,
    bit_identical: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    level_delta_db: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_sample_delta: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    correlation: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    band_deltas_db: Option<BTreeMap<String, f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    frames_a: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    frames_b: Option<usize>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    notes: Vec<String>,
}

fn read_pcm(path: &Path) -> Option<Vec<u8>> {
    std::fs::read(path).ok()
}

/// PCM sample width of a baseline render, inferred from the file extension.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PcmFormat {
    /// `.f64le` — full f64. The only format that can support a bit-exactness claim.
    F64,
    /// `.f32le` — legacy. Carries a ~6e-8 relative floor; retained so pre-existing
    /// baselines stay readable, but a byte-compare of two f32 renders proves much
    /// less than it appears to.
    F32,
}

impl PcmFormat {
    fn ext(self) -> &'static str {
        match self {
            PcmFormat::F64 => "f64le",
            PcmFormat::F32 => "f32le",
        }
    }
}

/// Locate a render for `plugin`/`program` in `dir`, preferring f64.
fn find_pcm(dir: &Path, plugin: &str, program: &str) -> Option<(Vec<u8>, PcmFormat)> {
    for fmt in [PcmFormat::F64, PcmFormat::F32] {
        let p = dir.join(plugin).join(format!("{program}.{}", fmt.ext()));
        if let Some(b) = read_pcm(&p) {
            return Some((b, fmt));
        }
    }
    None
}

/// Decode little-endian PCM to f64. f32 input widens exactly (every f32 is
/// representable in f64), so downstream metrics are uniform across formats.
fn bytes_to_samples(bytes: &[u8], fmt: PcmFormat) -> Vec<f64> {
    match fmt {
        PcmFormat::F64 => bytes
            .chunks_exact(8)
            .map(|c| f64::from_le_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]))
            .collect(),
        PcmFormat::F32 => bytes
            .chunks_exact(4)
            .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]) as f64)
            .collect(),
    }
}

/// Compare the generated `circuit.rs` stored alongside each capture.
///
/// The audio renders alone cannot gate a refactor: identical audio is a
/// necessary but not sufficient condition for identical codegen, and large
/// swathes of emitted code (recovery ladders, diagnostics) are not exercised by
/// the five programs at all. Diffing the source closes that hole cheaply.
/// Returns `None` when either side is absent.
fn generated_source_differs(dir_a: &Path, dir_b: &Path, plugin: &str) -> Option<bool> {
    let a = std::fs::read_to_string(dir_a.join(plugin).join("circuit.rs")).ok()?;
    let b = std::fs::read_to_string(dir_b.join(plugin).join("circuit.rs")).ok()?;
    Some(a != b)
}

fn read_stats(dir: &Path, plugin: &str, program: &str) -> Option<Stats> {
    let p = dir.join(plugin).join(format!("{program}.stats.json"));
    let txt = std::fs::read_to_string(p).ok()?;
    serde_json::from_str(&txt).ok()
}

/// Pearson correlation over the common prefix, with a zero-variance guard:
/// two signals with (near-)zero variance correlate 1.0 when their samples
/// match closely, 0.0 otherwise.
fn correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    if n == 0 {
        return 0.0;
    }
    let (mut sa, mut sb) = (0.0f64, 0.0f64);
    for i in 0..n {
        sa += a[i];
        sb += b[i];
    }
    let (ma, mb) = (sa / n as f64, sb / n as f64);
    let (mut num, mut va, mut vb) = (0.0f64, 0.0f64, 0.0f64);
    for i in 0..n {
        let da = a[i] - ma;
        let db_ = b[i] - mb;
        num += da * db_;
        va += da * da;
        vb += db_ * db_;
    }
    if va <= 0.0 || vb <= 0.0 {
        let max_delta = max_sample_delta(a, b);
        return if max_delta < 1e-12 { 1.0 } else { 0.0 };
    }
    num / (va.sqrt() * vb.sqrt())
}

fn max_sample_delta(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    let mut m = 0.0f64;
    for i in 0..n {
        let d = (a[i] - b[i]).abs();
        if d.is_nan() {
            return f64::NAN;
        }
        m = m.max(d);
    }
    m
}

fn list_programs(dir: &Path, plugin: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    if let Ok(entries) = std::fs::read_dir(dir.join(plugin)) {
        for e in entries.flatten() {
            let p = e.path();
            if p.extension().is_some_and(|x| x == "f64le" || x == "f32le") {
                if let Some(stem) = p.file_stem().and_then(|s| s.to_str()) {
                    out.insert(stem.to_string());
                }
            }
        }
    }
    out
}

fn list_plugins(dir: &Path) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    if let Ok(entries) = std::fs::read_dir(dir) {
        for e in entries.flatten() {
            let p = e.path();
            if p.is_dir() {
                if let Some(name) = p.file_name().and_then(|s| s.to_str()) {
                    out.insert(name.to_string());
                }
            }
        }
    }
    out
}

/// plugin -> netlist content sha256, from the baseline's capture_report.json.
fn load_cir_hashes(dir: &Path) -> BTreeMap<String, String> {
    let mut out = BTreeMap::new();
    let Ok(txt) = std::fs::read_to_string(dir.join("capture_report.json")) else {
        return out;
    };
    let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) else {
        return out;
    };
    if let Some(circuits) = v.get("circuits").and_then(|c| c.as_array()) {
        for c in circuits {
            // `id` is the baseline-directory key (unique even when several
            // netlists share one plugin name); older reports only had `plugin`.
            let key = c
                .get("id")
                .and_then(|p| p.as_str())
                .or_else(|| c.get("plugin").and_then(|p| p.as_str()));
            if let (Some(plugin), Some(sha)) = (key, c.get("cir_sha256").and_then(|s| s.as_str())) {
                if !sha.is_empty() {
                    out.insert(plugin.to_string(), sha.to_string());
                }
            }
        }
    }
    out
}

fn meta_rev(dir: &Path) -> String {
    std::fs::read_to_string(dir.join("metadata.json"))
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|v| {
            v.get("melange_rev")
                .and_then(|r| r.as_str())
                .map(|s| s.chars().take(12).collect())
        })
        .unwrap_or_else(|| "unknown-rev".to_string())
}

fn compare_pair(dir_a: &Path, dir_b: &Path, plugin: &str, program: &str) -> PairResult {
    let mut res = PairResult {
        plugin: plugin.to_string(),
        program: program.to_string(),
        class: "CHANGED".into(),
        severity: 0.0,
        bit_identical: false,
        level_delta_db: None,
        max_sample_delta: None,
        correlation: None,
        band_deltas_db: None,
        frames_a: None,
        frames_b: None,
        notes: Vec::new(),
    };

    let fa = find_pcm(dir_a, plugin, program);
    let fb = find_pcm(dir_b, plugin, program);
    let ba = fa.as_ref().map(|(b, _)| b.clone());
    let bb = fb.as_ref().map(|(b, _)| b.clone());
    match (&ba, &bb) {
        (None, None) => {
            res.class = "MISSING".into();
            res.severity = 1e9;
            res.notes.push("render missing in BOTH baselines".into());
            return res;
        }
        (None, Some(_)) => {
            res.class = "MISSING".into();
            res.severity = 1e9;
            res.notes
                .push(format!("missing in A ({})", dir_a.display()));
            return res;
        }
        (Some(_), None) => {
            res.class = "MISSING".into();
            res.severity = 1e9;
            res.notes
                .push(format!("missing in B ({})", dir_b.display()));
            return res;
        }
        (Some(a), Some(b)) => {
            if a == b {
                res.class = "IDENTICAL".into();
                res.bit_identical = true;
                // An f32-vs-f32 byte match is NOT evidence of bit-exactness at
                // f64: the stored values were rounded before they were written.
                // Say so, so a green run is not over-read.
                if fa.as_ref().map(|(_, f)| *f) == Some(PcmFormat::F32) {
                    res.notes.push(
                        "byte-equal at f32 precision — cannot support an f64                          bit-exactness claim; re-capture as .f64le"
                            .into(),
                    );
                }
                return res;
            }
        }
    }

    let (raw_a, fmt_a) = fa.unwrap();
    let (raw_b, fmt_b) = fb.unwrap();
    if fmt_a != fmt_b {
        res.notes.push(format!(
            "PCM format differs between baselines: A={} B={} — comparison is              limited to the coarser precision",
            fmt_a.ext(),
            fmt_b.ext()
        ));
    }
    let a = bytes_to_samples(&raw_a, fmt_a);
    let b = bytes_to_samples(&raw_b, fmt_b);
    let sa = read_stats(dir_a, plugin, program);
    let sb = read_stats(dir_b, plugin, program);

    res.frames_a = Some(a.len());
    res.frames_b = Some(b.len());
    let same_len = a.len() == b.len();
    if !same_len {
        res.notes.push(format!(
            "length mismatch: {} vs {} samples",
            a.len(),
            b.len()
        ));
    }

    let corr = correlation(&a, &b);
    let max_delta = max_sample_delta(&a, &b);
    res.correlation = Some(corr);
    res.max_sample_delta = Some(max_delta);

    let mut nan_introduced = false;
    let mut level_delta = 0.0f64;
    let mut max_band_delta = 0.0f64;
    let mut both_below_floor = false;
    if let (Some(sa), Some(sb)) = (&sa, &sb) {
        nan_introduced = sb.nan_count > sa.nan_count || sb.inf_count > sa.inf_count;
        if nan_introduced {
            res.notes.push(format!(
                "NaN/Inf introduced: {}+{} -> {}+{}",
                sa.nan_count, sa.inf_count, sb.nan_count, sb.inf_count
            ));
        }
        level_delta = delta_db(sa.rms_dbfs, sb.rms_dbfs);
        res.level_delta_db = Some(level_delta);
        let mut bands = BTreeMap::new();
        for (k, va) in &sa.bands_dbfs {
            if let Some(vb) = sb.bands_dbfs.get(k) {
                let d = delta_db(*va, *vb);
                max_band_delta = max_band_delta.max(d.abs());
                bands.insert(k.clone(), d);
            }
        }
        res.band_deltas_db = Some(bands);
        both_below_floor = sa.peak_dbfs < -120.0 && sb.peak_dbfs < -120.0;
    } else {
        res.notes.push("stats JSON missing on one side".into());
    }

    let negligible = same_len
        && !nan_introduced
        && ((level_delta.abs() < 0.01 && corr > 0.99999) || (both_below_floor && max_delta < 1e-6));
    if negligible {
        res.class = "NEGLIGIBLE".into();
        if both_below_floor && !(level_delta.abs() < 0.01 && corr > 0.99999) {
            res.notes
                .push("both renders below -120 dBFS residue floor".into());
        }
        return res;
    }

    // Severity: NaN dominates, then the loudest of level/band shift and
    // decorrelation (scaled so corr 0.9 ~ 10).
    res.severity = if nan_introduced || max_delta.is_nan() {
        1e12
    } else if !same_len {
        1e6
    } else {
        level_delta
            .abs()
            .max(max_band_delta)
            .max((1.0 - corr) * 100.0)
    };
    res
}

/// Delta between two dB values where either may be the -400 dB silence
/// sentinel. Sentinel-to-sentinel is 0 (both silent); otherwise the raw
/// difference (a silent side going live IS a huge, real delta).
fn delta_db(a: f64, b: f64) -> f64 {
    if a <= SILENT_DB && b <= SILENT_DB {
        0.0
    } else {
        b - a
    }
}

/// Compare two baselines.
///
/// `strict` selects the gate semantics:
/// - `false` (default) — a *release* comparison. NEGLIGIBLE passes: the question
///   is "did anything audibly change?".
/// - `true` — a *refactor* comparison. Only IDENTICAL passes, and every
///   generated `circuit.rs` must match too. The question is "did anything
///   change at all?", which is the only gate a behaviour-preserving refactor
///   can be held to. These are different questions and must not share a
///   threshold: NEGLIGIBLE is exactly the band in which a refactor bug hides.
pub fn run(dir_a: &Path, dir_b: &Path, json_out: &Path, strict: bool) -> Result<usize, String> {
    if !dir_a.is_dir() {
        return Err(format!("{} is not a directory", dir_a.display()));
    }
    if !dir_b.is_dir() {
        return Err(format!("{} is not a directory", dir_b.display()));
    }

    let plugins: BTreeSet<String> = list_plugins(dir_a)
        .union(&list_plugins(dir_b))
        .cloned()
        .collect();
    if plugins.is_empty() {
        return Err("no circuit directories found in either baseline".into());
    }

    // Netlist content hashes: when they differ between baselines, output
    // deltas for that circuit are "the input changed", NOT a compiler
    // regression — never silently classify them as one.
    let hashes_a = load_cir_hashes(dir_a);
    let hashes_b = load_cir_hashes(dir_b);
    let mut netlist_changed: BTreeMap<String, (String, String)> = BTreeMap::new();
    for (plugin, ha) in &hashes_a {
        if let Some(hb) = hashes_b.get(plugin) {
            if ha != hb {
                netlist_changed.insert(plugin.clone(), (ha.clone(), hb.clone()));
            }
        }
    }

    let mut results: Vec<PairResult> = Vec::new();
    for plugin in &plugins {
        let progs: BTreeSet<String> = list_programs(dir_a, plugin)
            .union(&list_programs(dir_b, plugin))
            .cloned()
            .collect();
        if progs.is_empty() {
            results.push(PairResult {
                plugin: plugin.clone(),
                program: "(none)".into(),
                class: "MISSING".into(),
                severity: 1e9,
                bit_identical: false,
                level_delta_db: None,
                max_sample_delta: None,
                correlation: None,
                band_deltas_db: None,
                frames_a: None,
                frames_b: None,
                notes: vec!["no renders captured for this circuit in either baseline".into()],
            });
            continue;
        }
        for program in &progs {
            let mut r = compare_pair(dir_a, dir_b, plugin, program);
            if let Some((ha, hb)) = netlist_changed.get(plugin) {
                r.notes.push(format!(
                    "netlist content differs between baselines ({}.. -> {}..)",
                    &ha[..ha.len().min(12)],
                    &hb[..hb.len().min(12)]
                ));
                if r.class == "CHANGED" {
                    r.class = "INPUT-CHANGED".into();
                }
            }
            results.push(r);
        }
    }

    let count = |c: &str| results.iter().filter(|r| r.class == c).count();
    let n_identical = count("IDENTICAL");
    let n_negligible = count("NEGLIGIBLE");
    let n_changed = count("CHANGED");
    let n_missing = count("MISSING");
    let n_input_changed = count("INPUT-CHANGED");

    // ---- text report ----
    let rev_a = meta_rev(dir_a);
    let rev_b = meta_rev(dir_b);
    println!("================ GOLDEN AUDIO COMPARE ================");
    println!("A: {}  (melange {rev_a})", dir_a.display());
    println!("B: {}  (melange {rev_b})", dir_b.display());
    println!("======================================================");

    if !netlist_changed.is_empty() {
        println!("\n######################################################");
        println!(
            "##  WARNING: NETLIST INPUT CHANGED for {} circuit(s)",
            netlist_changed.len()
        );
        println!("##  Deltas below for these circuits reflect a changed");
        println!("##  .cir INPUT, NOT a compiler regression:");
        for (plugin, (ha, hb)) in &netlist_changed {
            println!(
                "##    {plugin}: {}.. -> {}..",
                &ha[..ha.len().min(12)],
                &hb[..hb.len().min(12)]
            );
        }
        println!("######################################################");
    }

    let mut changed: Vec<&PairResult> = results
        .iter()
        .filter(|r| r.class == "CHANGED" || r.class == "MISSING" || r.class == "INPUT-CHANGED")
        .collect();
    changed.sort_by(|x, y| y.severity.total_cmp(&x.severity));

    if changed.is_empty() {
        println!("\nNO AUDIBLE CHANGES.");
    } else {
        println!("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        println!("!!  {} RENDER(S) CHANGED OR MISSING  !!", changed.len());
        println!("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        for r in &changed {
            // Level delta (dB) leads the line — it is the number the oomox
            // change-risk checklist consumes.
            let level = r
                .level_delta_db
                .map(|l| format!("{l:+8.3} dB"))
                .unwrap_or_else(|| "   n/a   ".into());
            let mut line = format!(
                "  {level}  {:26} {:13}",
                format!("{}/{}", r.plugin, r.program),
                r.class
            );
            if let Some(c) = r.correlation {
                line.push_str(&format!("  corr {c:.6}"));
            }
            if let Some(m) = r.max_sample_delta {
                line.push_str(&format!("  maxΔ {m:.3e}"));
            }
            line.push_str(&format!("  sev {:.3}", r.severity));
            println!("{line}");
            if let Some(bands) = &r.band_deltas_db {
                let s: Vec<String> = bands
                    .iter()
                    .map(|(k, v)| {
                        format!("{}:{:+.2}", k.split_once('_').map(|x| x.1).unwrap_or(k), v)
                    })
                    .collect();
                println!("             bands dB: {}", s.join("  "));
            }
            for n in &r.notes {
                println!("             note: {n}");
            }
        }
    }

    let negligible: Vec<&PairResult> = results.iter().filter(|r| r.class == "NEGLIGIBLE").collect();
    if !negligible.is_empty() {
        println!("\nNEGLIGIBLE ({}):", negligible.len());
        for r in &negligible {
            println!(
                "  {:24} level {:+.5} dB  corr {:.7}{}",
                format!("{}/{}", r.plugin, r.program),
                r.level_delta_db.unwrap_or(0.0),
                r.correlation.unwrap_or(1.0),
                if r.notes.is_empty() {
                    String::new()
                } else {
                    format!("  ({})", r.notes.join("; "))
                }
            );
        }
    }
    // ---- generated-source diff (strict gate) ----
    // Audio equality does not imply codegen equality: the five programs never
    // enter most of the recovery ladders, so a refactor can alter emitted code
    // that the renders simply never execute.
    let mut source_differs: Vec<String> = Vec::new();
    let mut source_missing: Vec<String> = Vec::new();
    for plugin in &plugins {
        match generated_source_differs(dir_a, dir_b, plugin) {
            Some(true) => source_differs.push(plugin.clone()),
            Some(false) => {}
            None => source_missing.push(plugin.clone()),
        }
    }
    if !source_differs.is_empty() {
        println!("\nGENERATED SOURCE DIFFERS ({}):", source_differs.len());
        for p in &source_differs {
            println!("  {p}/circuit.rs");
        }
        println!(
            "  (provenance header lines are masked at capture time, so these are \
             real codegen differences)"
        );
    }
    if !source_missing.is_empty() {
        println!(
            "\nGENERATED SOURCE UNAVAILABLE for {} circuit(s) — cannot verify codegen equality",
            source_missing.len()
        );
    }

    println!(
        "\nSUMMARY: {n_identical} identical, {n_negligible} negligible, {n_changed} changed, \
         {n_input_changed} input-changed, {n_missing} missing"
    );
    if strict {
        println!(
            "STRICT: {} source-differs, {} source-unavailable",
            source_differs.len(),
            source_missing.len()
        );
    }

    // ---- JSON report ----
    let json = serde_json::json!({
        "baseline_a": dir_a.display().to_string(),
        "baseline_b": dir_b.display().to_string(),
        "melange_rev_a": rev_a,
        "melange_rev_b": rev_b,
        "summary": {
            "identical": n_identical,
            "negligible": n_negligible,
            "changed": n_changed,
            "input_changed": n_input_changed,
            "missing": n_missing,
        },
        "netlist_input_changed": netlist_changed.iter().map(|(p, (a, b))| {
            serde_json::json!({"plugin": p, "sha256_a": a, "sha256_b": b})
        }).collect::<Vec<_>>(),
        "strict": strict,
        "generated_source_differs": source_differs,
        "generated_source_unavailable": source_missing,
        "results": results,
    });
    std::fs::write(json_out, serde_json::to_string_pretty(&json).unwrap())
        .map_err(|e| format!("write {}: {e}", json_out.display()))?;
    println!("JSON report: {}", json_out.display());

    if strict {
        // Only IDENTICAL passes, and codegen must match. NEGLIGIBLE is a
        // failure here by design — see the doc comment on `run`.
        return Ok(n_changed
            + n_missing
            + n_input_changed
            + n_negligible
            + source_differs.len()
            + source_missing.len());
    }
    Ok(n_changed + n_missing + n_input_changed)
}
