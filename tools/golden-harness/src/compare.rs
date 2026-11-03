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
//!
//! None of those classes can answer "was either render ever a solution?": a
//! Newton-Raphson solve that exhausts its iteration ceiling emits the capped
//! iterate, which is bounded and smooth and therefore IDENTICAL to itself run
//! after run. `convergence` assesses that separately and prints it as its own
//! loud category; see that module for why it is deliberately report-only.
//!
//! # Solver counter gate
//!
//! The audio classes are also blind to HOW an output was produced. A build
//! that dropped the saturating-inductor stamps rendered `funkyinduct` within
//! 1.85e-4 of the baseline — NEGLIGIBLE — while its counters showed 38 287
//! Newton ceilings and 9.5 M line-search failures where the baseline had 35
//! and 59 408: the rescue paths produced a near-identical output from a wrong
//! Jacobian. So every counter the capture records (`stats.json`
//! `diagnostics`) is judged per render, per counter, by its class in
//! [`COUNTER_CLASSES`]:
//!
//! - **failure-path** (a rescue or a refusal fired): ANY increase fails.
//! - **effort** (how hard the solve was): a move beyond
//!   [`EFFORT_RELATIVE_BOUND`] of the baseline count, floored at
//!   [`EFFORT_ABSOLUTE_FLOOR`] counts, fails.
//! - **informational** (not a count): reported, never gated.
//! - a key in neither list fails: an unclassified counter is a gate hole.
//!
//! The rule is per render, per counter, because totals over cascaded renders
//! are not monotone (one render's loss hides another's gain). A failing
//! counter is never waved through on the audio verdict: the change needs a
//! first-divergence trace (the first sample at which the two builds' Newton
//! paths part, and why) before it can pass. See `COUNTER GATE` in the
//! report and `golden-baseline-manifest.md`.

use crate::convergence;
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
    #[serde(skip_serializing_if = "std::ops::Not::not", default)]
    diagnostics_differ: bool,
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
    /// Solver-counter findings for this render (see [`judge_counters`]).
    /// Non-empty means the render FAILS the counter gate.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    counter_findings: Vec<CounterFinding>,
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

/// Nodal sub-path per circuit, from a baseline's capture_report.json.
fn load_sub_paths(dir: &Path) -> BTreeMap<String, String> {
    let mut out = BTreeMap::new();
    let Ok(txt) = std::fs::read_to_string(dir.join("capture_report.json")) else {
        return out;
    };
    let Ok(v) = serde_json::from_str::<serde_json::Value>(&txt) else {
        return out;
    };
    if let Some(arr) = v.get("circuits").and_then(|x| x.as_array()) {
        for c in arr {
            if let (Some(p), Some(sp)) = (
                c.get("plugin").and_then(|x| x.as_str()),
                c.get("nodal_sub_path").and_then(|x| x.as_str()),
            ) {
                out.insert(p.to_string(), sp.to_string());
            }
        }
    }
    out
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

/// Compare the solver diagnostic counters recorded with each render.
///
/// Deliberately checked even when the PCM is byte-identical: a change that
/// alters how often the BE fallback, NaN reset or line search fires without
/// altering the output samples is exactly the class of regression the audio
/// gate cannot see, and exactly what a codegen refactor is most likely to do.
/// Returns the differing keys as `(name, a, b)`.
fn diagnostic_deltas(a: &Option<Stats>, b: &Option<Stats>) -> Vec<(String, f64, f64)> {
    let (Some(a), Some(b)) = (a, b) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    let keys: BTreeSet<&String> = a.diagnostics.keys().chain(b.diagnostics.keys()).collect();
    for k in keys {
        let va = a.diagnostics.get(k).copied().unwrap_or(0.0);
        let vb = b.diagnostics.get(k).copied().unwrap_or(0.0);
        if va != vb {
            out.push((k.clone(), va, vb));
        }
    }
    out
}

/// What a recorded solver counter says about a render, which decides how the
/// gate judges a change in it.
#[derive(Serialize, Clone, Copy, PartialEq, Eq, Debug)]
#[serde(rename_all = "kebab-case")]
pub enum CounterClass {
    /// The counter records a rescue or a refusal: a Newton ceiling, a
    /// sub-step, a backward-Euler fallback, a hold, a reset, a singular
    /// matrix, a clamp, an abandoned sub-sample solve. The output after such
    /// an event is bounded and smooth, so the audio gate cannot see it fire;
    /// ANY per-render increase fails the gate.
    FailurePath,
    /// The counter records how hard a converged solve was (iterations,
    /// refactors, line-search failures) or a region the circuit entered
    /// (an op-amp pinned, a device outside its reduction's region). Legitimate
    /// roundoff-class changes move these by a handful of samples; a change in
    /// the solver's path moves them by orders of magnitude. A per-render move
    /// beyond [`EFFORT_RELATIVE_BOUND`] (floored at [`EFFORT_ABSOLUTE_FLOOR`])
    /// fails the gate.
    Effort,
    /// Not a count: a level, or the last sample's iteration index. Reported
    /// with the other deltas, never gated here (the level is the audio gate's
    /// measurand; a single sample's iteration index is noise by construction).
    Informational,
}

impl CounterClass {
    pub fn as_str(self) -> &'static str {
        match self {
            CounterClass::FailurePath => "failure-path",
            CounterClass::Effort => "effort",
            CounterClass::Informational => "informational",
        }
    }
}

/// THE classification of every solver counter the capture can record
/// (`capture::detect_diag_fields`: every `pub diag_*` field and
/// `last_nr_iterations` on the generated `CircuitState`). One table; the test
/// `every_emitted_counter_is_classified` scans the emitters and templates in
/// `crates/melange-solver` and every stored baseline so a counter added to
/// codegen cannot reach a compare unclassified — and if one does, the gate
/// fails on it loudly rather than letting it through.
///
/// The meaning of each field is its doc comment on the generated
/// `CircuitState` (`docs/CODE_API.md` "Did the solver actually solve it?").
pub const COUNTER_CLASSES: &[(&str, CounterClass)] = &[
    // ---- failure-path: a rescue or a refusal fired ----
    // Newton exhausted its iteration ceiling; the capped iterate went on to
    // the sub-step / BE ladder.
    ("diag_nr_max_iter_count", CounterClass::FailurePath),
    // The adaptive sub-step ladder ran on a sample.
    ("diag_substep_count", CounterClass::FailurePath),
    // The backward-Euler fallback solve ran (also counts every sample after
    // the runtime BE latch engaged).
    ("diag_be_fallback_count", CounterClass::FailurePath),
    // The runtime BE latch engaged: a self-sustaining Nyquist limit cycle was
    // detected and the instance switched to backward Euler for good.
    ("diag_be_latch_count", CounterClass::FailurePath),
    // Every Newton path failed and the previous state was committed.
    ("diag_nr_hold_count", CounterClass::FailurePath),
    // DK: an unconverged final iterate was committed as output.
    (
        "diag_nr_unconverged_commit_count",
        CounterClass::FailurePath,
    ),
    // Every sample that was never solved, whatever the mechanism.
    ("diag_unsolved_sample_count", CounterClass::FailurePath),
    // A reduced device model left the region its reduction assumes.
    ("diag_reduced_model_exit_count", CounterClass::FailurePath),
    // State reset on a NaN/Inf iterate.
    ("diag_nan_reset_count", CounterClass::FailurePath),
    // State reset on a finite but physically impossible iterate.
    ("diag_magnitude_reset_count", CounterClass::FailurePath),
    // Matrix inversion found a singular matrix.
    ("diag_singular_matrix_count", CounterClass::FailurePath),
    // Full-LU's exact warm start was unreachable; the predictor was used.
    ("diag_warm_start_fallback_count", CounterClass::FailurePath),
    // The legacy global voltage-damping net scaled a step.
    ("diag_voltage_damp_count", CounterClass::FailurePath),
    // The generated output clamp engaged: on those samples the audio IS the
    // clamp, so growth beyond the ceiling is invisible to the audio gate.
    ("diag_clamp_count", CounterClass::FailurePath),
    // Host-side guards: the circuit was driven with something other than
    // what the program sent. The programs are fixed, so an increase means
    // the build accepted less of them.
    ("diag_input_clamp_count", CounterClass::FailurePath),
    ("diag_input_nan_count", CounterClass::FailurePath),
    ("diag_inject_clamp_count", CounterClass::FailurePath),
    ("diag_runtime_nan_count", CounterClass::FailurePath),
    // Linear DK with pots: a rank-1 update's denominator was non-finite or
    // below 1e-6 and a full rebuild was forced.
    ("diag_rank1_fallback_count", CounterClass::FailurePath),
    // Sub-sample fire: the sub-step solve was abandoned and the whole-sample
    // solution kept; a flip left grid-quantised because the breakpoint
    // budget was spent; a second device latched at another device's
    // breakpoint instead of its own crossing.
    (
        "diag_subsample_fire_abandon_count",
        CounterClass::FailurePath,
    ),
    (
        "diag_subsample_fire_unresolved_ceiling",
        CounterClass::FailurePath,
    ),
    (
        "diag_subsample_fire_unresolved_coincident",
        CounterClass::FailurePath,
    ),
    // ---- effort: how hard a converged solve was, or a region entered ----
    ("diag_refactor_count", CounterClass::Effort),
    ("diag_ls_fail_count", CounterClass::Effort),
    // Samples on which the active-set rail resolve pinned an op-amp output:
    // the op-amp railing, which is physics, not a rescue.
    ("diag_active_set_pin_count", CounterClass::Effort),
    // Device-samples in a region a reduction would not cover; counted on the
    // full models too (characterization, not a guard).
    ("diag_region_exit_count", CounterClass::Effort),
    // Linear DK with pots: rank-1 updates taken, and the full rebuilds a wide
    // downward sweep is expected to force.
    ("diag_rank1_update_count", CounterClass::Effort),
    ("diag_rank1_trigger_rebuild_count", CounterClass::Effort),
    // Sub-sample fire working: samples re-solved, flips detected and
    // resolved, extra segments solved, Schur triples built or reused, and
    // flips left on a segment end within the minimum-segment guard (benign).
    ("diag_subsample_fire_count", CounterClass::Effort),
    ("diag_subsample_fire_detected", CounterClass::Effort),
    ("diag_subsample_fire_resolved", CounterClass::Effort),
    ("diag_subsample_fire_segments", CounterClass::Effort),
    ("diag_subsample_fire_schur_builds", CounterClass::Effort),
    ("diag_subsample_fire_schur_reuses", CounterClass::Effort),
    (
        "diag_subsample_fire_unresolved_gridpoint",
        CounterClass::Effort,
    ),
    // ---- informational: not counts ----
    // Peak absolute output before the clamp: a level, the audio gate's job.
    ("diag_peak_output", CounterClass::Informational),
    // The LAST sample's 0-indexed converging iteration: one sample's reading,
    // not a render-wide count. `convergence` prints it as context.
    ("last_nr_iterations", CounterClass::Informational),
];

/// Class of a recorded counter key, or `None` if the table does not know it.
pub fn counter_class(key: &str) -> Option<CounterClass> {
    COUNTER_CLASSES
        .iter()
        .find(|(k, _)| *k == key)
        .map(|(_, c)| *c)
}

/// Relative bound on an effort counter's per-render move: `|B − A| / A`.
///
/// Set from measured data, not taste. The reciprocal-pivot experiment
/// (2026-11-03: multiply by a stored 1/pivot in the sparse LU instead of
/// dividing, one extra unit roundoff per operation, ruled a roundoff-class
/// change) moved effort counters on 5 of the 192 golden renders, against the
/// same binary without it (`golden/A` vs `golden/R` of that run, corpus totals
/// refactors 12 633 376 → 12 633 380, line-search failures 59 408 → 59 413):
///
/// ```text
///   warpony/silence          diag_ls_fail_count    1125 ->   1129   +0.356 %
///   warpony/silence          diag_refactor_count  23276 ->  23293   +0.073 %
///   warpony/step             diag_refactor_count  11791 ->  11781   -0.085 %
///   basic-bitch/sweep        diag_refactor_count 163544 -> 163543   -0.0006 %
///   warpony/sweep            diag_refactor_count 551937 -> 551936   -0.0002 %
///   wurli-power-amp/sine1k   diag_ls_fail_count   24173 ->  24174   +0.004 %
///   wurli-power-amp/sine1k   diag_refactor_count 477820 -> 477819   -0.0002 %
/// ```
///
/// The largest roundoff-class move is +0.36 %. The regression this gate
/// exists to catch moved `funkyinduct` line-search failures 59 408 → 9.5 M
/// (+16 000 %) with the audio NEGLIGIBLE. 10 % is 28× the largest legitimate
/// move and 1/1600 of the regression: a change that only reshuffles rounding
/// cannot reach it, and a change in the solver's path cannot hide under it.
///
/// Where it stands for a change that is NOT roundoff-class: the 0.1.20 cycle
/// changed the certification itself (the node-step test on the undamped
/// correction, the libm device laws) and moved `warpony/step` line-search
/// failures 323 → 441 (+36.5 %) and `warpony/silence` 828 → 1125 (+35.9 %)
/// between `v0.1.19-rc` and `v0.1.20-rc`. Under this gate such a move fails
/// until it carries its first-divergence trace, which is the rule: a
/// certification change is exactly the kind that must be traced, not waved
/// through on a NEGLIGIBLE audio verdict. (The libm change alone recorded
/// "Newton iteration totals identical" on the bench decks, CHANGELOG 0.1.20;
/// no per-render capture of it alone survives, so its own effort moves are
/// not separable from that cycle's.)
pub const EFFORT_RELATIVE_BOUND: f64 = 0.10;

/// Absolute floor under [`EFFORT_RELATIVE_BOUND`]: an effort counter may move
/// by this many counts in either direction whatever its baseline, so a render
/// whose baseline count is 0 (or a few) is judged on the same count scale as
/// the rest rather than failing on `1 → 2`. The bound on a render is
/// `max(EFFORT_RELATIVE_BOUND × A, EFFORT_ABSOLUTE_FLOOR)`.
///
/// From the same roundoff-class data: the largest absolute move any effort
/// counter made was +17 refactors (`warpony/silence`, 23 276 → 23 293) and +4
/// line-search failures on the same render; 20 sits above both. No
/// roundoff-class change moved an effort counter from 0 at all, nor did the
/// `v0.1.18-rc → v0.1.19-rc` or `v0.1.19-rc → v0.1.20-rc` release pairs over
/// 192 renders each (the smallest baseline that moved was
/// `noyce-transformer-triode/step` refactors 5 → 4), so the floor is bounded by
/// the largest small move seen, not by a from-zero datum. Against the
/// `funkyinduct` regression (+9.4 M) it is five orders of magnitude under.
pub const EFFORT_ABSOLUTE_FLOOR: f64 = 20.0;

/// Why a counter failed the gate.
#[derive(Serialize, Clone, Copy, PartialEq, Eq, Debug)]
#[serde(rename_all = "kebab-case")]
pub enum CounterFindingKind {
    /// A failure-path counter is higher in B than in A.
    FailurePathIncrease,
    /// An effort counter moved beyond the relative bound (or the floor).
    EffortOutOfBound,
    /// The key is in neither list of [`COUNTER_CLASSES`].
    Unclassified,
}

/// One failing counter on one render.
#[derive(Serialize, Clone, Debug, PartialEq)]
pub struct CounterFinding {
    pub counter: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub class: Option<CounterClass>,
    pub kind: CounterFindingKind,
    /// Recorded value in A; `None` when A's build declared no such counter.
    pub a: Option<f64>,
    /// Recorded value in B; `None` when B's build declared no such counter.
    pub b: Option<f64>,
    /// The sentence the report prints. Names the counter, both values and
    /// what has to happen before the change can pass.
    pub message: String,
}

fn fmt_val(v: Option<f64>) -> String {
    match v {
        Some(x) => format!("{x}"),
        None => "(absent)".into(),
    }
}

/// Judge one render's recorded counters, A (reference) against B.
///
/// A key absent on one side reads as 0 on that side: the build did not
/// declare the mechanism, so it cannot have fired. A failure counter that
/// appears with a nonzero value is therefore an increase (the route moved AND
/// the rescue fired), which is exactly a change to trace.
///
/// Returns one finding per failing counter; empty means the render passes.
pub fn judge_counters(a: &BTreeMap<String, f64>, b: &BTreeMap<String, f64>) -> Vec<CounterFinding> {
    const TRACE: &str = "a first-divergence trace is required before this change can pass";
    let keys: BTreeSet<&String> = a.keys().chain(b.keys()).collect();
    let mut out = Vec::new();
    for k in keys {
        let va = a.get(k).copied();
        let vb = b.get(k).copied();
        let (x, y) = (va.unwrap_or(0.0), vb.unwrap_or(0.0));
        match counter_class(k) {
            None => out.push(CounterFinding {
                counter: k.clone(),
                class: None,
                kind: CounterFindingKind::Unclassified,
                a: va,
                b: vb,
                message: format!(
                    "{k} {} -> {}: counter not in COUNTER_CLASSES (compare.rs); classify it \
                     as failure-path, effort or informational before this change can pass",
                    fmt_val(va),
                    fmt_val(vb)
                ),
            }),
            Some(CounterClass::FailurePath) => {
                if y > x {
                    out.push(CounterFinding {
                        counter: k.clone(),
                        class: Some(CounterClass::FailurePath),
                        kind: CounterFindingKind::FailurePathIncrease,
                        a: va,
                        b: vb,
                        message: format!(
                            "{k} {} -> {} (+{}): a failure-path counter increased; {TRACE}",
                            fmt_val(va),
                            fmt_val(vb),
                            y - x
                        ),
                    });
                }
            }
            Some(CounterClass::Effort) => {
                let delta = y - x;
                let allowed = (EFFORT_RELATIVE_BOUND * x.abs()).max(EFFORT_ABSOLUTE_FLOOR);
                if delta.abs() > allowed {
                    let pct = if x != 0.0 {
                        format!("{:+.2} %", delta / x * 100.0)
                    } else {
                        "from zero".to_string()
                    };
                    out.push(CounterFinding {
                        counter: k.clone(),
                        class: Some(CounterClass::Effort),
                        kind: CounterFindingKind::EffortOutOfBound,
                        a: va,
                        b: vb,
                        message: format!(
                            "{k} {} -> {} ({pct}; |delta| {} > bound {:.1} = max({:.0} % of {}, \
                             {})): an effort counter moved beyond the bound; {TRACE}",
                            fmt_val(va),
                            fmt_val(vb),
                            delta.abs(),
                            allowed,
                            EFFORT_RELATIVE_BOUND * 100.0,
                            x,
                            EFFORT_ABSOLUTE_FLOOR
                        ),
                    });
                }
            }
            Some(CounterClass::Informational) => {}
        }
    }
    out
}

/// Counter findings for a render pair from its stats; a side with no stats
/// contributes an empty map (nothing recorded, nothing declared).
fn judge_render_counters(a: &Option<Stats>, b: &Option<Stats>) -> Vec<CounterFinding> {
    let empty = BTreeMap::new();
    let da = a.as_ref().map(|s| &s.diagnostics).unwrap_or(&empty);
    let db = b.as_ref().map(|s| &s.diagnostics).unwrap_or(&empty);
    judge_counters(da, db)
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
        diagnostics_differ: false,
        level_delta_db: None,
        max_sample_delta: None,
        correlation: None,
        band_deltas_db: None,
        frames_a: None,
        frames_b: None,
        notes: Vec::new(),
        counter_findings: Vec::new(),
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
                // Byte-identical audio does NOT imply identical execution.
                let sa = read_stats(dir_a, plugin, program);
                let sb = read_stats(dir_b, plugin, program);
                // The counter gate judges byte-identical renders too: this is
                // precisely the case the audio classes cannot see.
                res.counter_findings = judge_render_counters(&sa, &sb);
                let dd = diagnostic_deltas(&sa, &sb);
                if !dd.is_empty() {
                    res.diagnostics_differ = true;
                    res.notes.push(format!(
                        "renders byte-identical but solver diagnostics differ: {}",
                        dd.iter()
                            .map(|(k, x, y)| format!("{k} {x}->{y}"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ));
                }
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

    res.counter_findings = judge_render_counters(&sa, &sb);
    let dd = diagnostic_deltas(&sa, &sb);
    if !dd.is_empty() {
        res.diagnostics_differ = true;
        res.notes.push(format!(
            "solver diagnostics differ: {}",
            dd.iter()
                .map(|(k, x, y)| format!("{k} {x}->{y}"))
                .collect::<Vec<_>>()
                .join(", ")
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

/// Gate failures and held renders, kept separate: "the output changed" and
/// "the output was never a solution" are different findings and get different
/// exit codes.
pub struct Outcome {
    pub failures: usize,
    pub held: usize,
    /// Renders that fail the solver counter gate (a failure-path counter
    /// increased, an effort counter left its bound, or a counter is
    /// unclassified). Its own failing condition: "the solver took a different
    /// path to this output" is a third finding, independent of the audio
    /// classes and of the hold.
    pub counter_failures: usize,
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
pub fn run(dir_a: &Path, dir_b: &Path, json_out: &Path, strict: bool) -> Result<Outcome, String> {
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
                diagnostics_differ: false,
                level_delta_db: None,
                max_sample_delta: None,
                correlation: None,
                band_deltas_db: None,
                frames_a: None,
                frames_b: None,
                notes: vec!["no renders captured for this circuit in either baseline".into()],
                counter_findings: Vec::new(),
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
    // ---- nodal sub-path (diagnosis, not detection) ----
    // A sub-path move necessarily changes circuit.rs wholesale, so the source
    // diff below already CATCHES it. Naming it turns a 3000-line unexplained
    // diff into one line that says what happened.
    let sp_a = load_sub_paths(dir_a);
    let sp_b = load_sub_paths(dir_b);
    let mut sub_path_moved: Vec<(String, String, String)> = Vec::new();
    for (plugin, a) in &sp_a {
        if let Some(b) = sp_b.get(plugin) {
            if a != b {
                sub_path_moved.push((plugin.clone(), a.clone(), b.clone()));
            }
        }
    }
    if !sub_path_moved.is_empty() {
        println!("\nNODAL SUB-PATH MOVED ({}):", sub_path_moved.len());
        for (p, a, b) in &sub_path_moved {
            println!("  {p}: {a} -> {b}");
        }
        println!(
            "  (the emitter chose a different nodal solve for these circuits; \
             expect a wholesale circuit.rs diff)"
        );
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
    let n_diag_differ = results.iter().filter(|r| r.diagnostics_differ).count();
    if n_diag_differ > 0 {
        println!(
            "\nSOLVER DIAGNOSTICS DIFFER ({n_diag_differ} render(s)) — recovery-ladder \
             behaviour changed even where audio did not"
        );
        for r in results.iter().filter(|r| r.diagnostics_differ) {
            for n in &r.notes {
                if n.contains("diagnostics differ") {
                    println!("  {}/{}: {n}", r.plugin, r.program);
                }
            }
        }
    }
    if strict {
        println!(
            "STRICT: {} source-differs, {} source-unavailable, {n_diag_differ} diagnostics-differ",
            source_differs.len(),
            source_missing.len()
        );
    }

    // ---- solver counter gate ----
    // Per render, per counter, by class (see COUNTER_CLASSES). Prints
    // unconditionally so a run in which the section is missing is visibly
    // different from one in which it passed.
    let counter_failed: Vec<&PairResult> = results
        .iter()
        .filter(|r| !r.counter_findings.is_empty())
        .collect();
    let n_counter_failed = counter_failed.len();
    println!("\n---- SOLVER COUNTER GATE ----");
    println!(
        "  rule: failure-path counters may not increase; effort counters may move by at most \
         max({:.0} % of the baseline, {} counts); unclassified counters fail. Per render, per \
         counter.",
        EFFORT_RELATIVE_BOUND * 100.0,
        EFFORT_ABSOLUTE_FLOOR
    );
    if counter_failed.is_empty() {
        println!("  No render's solver counters left their class's bound.");
    } else {
        let n_findings: usize = counter_failed
            .iter()
            .map(|r| r.counter_findings.len())
            .sum();
        println!("\n  ######################################################");
        println!(
            "  ##  {n_counter_failed} RENDER(S) FAIL ON SOLVER COUNTERS ({n_findings} counter(s))"
        );
        println!("  ##  The output gate is blind to the rescue paths that produced");
        println!("  ##  the output. A near-identical render from a different Newton");
        println!("  ##  path is a regression until it is traced to its first");
        println!("  ##  divergence. THIS FAILS THE GATE; do not wave it through on");
        println!("  ##  the audio verdict.");
        println!("  ######################################################");
        for r in &counter_failed {
            println!("  {}/{}  [{}]", r.plugin, r.program, r.class);
            for f in &r.counter_findings {
                println!(
                    "      FAIL  [{}] {}",
                    f.class.map(CounterClass::as_str).unwrap_or("unclassified"),
                    f.message
                );
            }
        }
    }

    // ---- convergence health (report-only, per baseline) ----
    // Asks a question the A-vs-B diff structurally cannot: not "did the output
    // change?" but "was the output ever a solution?". Printed for BOTH sides,
    // because a non-converged render is a property of one baseline and is
    // equally present in the reference.
    let conv_a = convergence::scan(dir_a);
    let conv_b = convergence::scan(dir_b);
    let conv_sets = [
        (format!("A {}", dir_a.display()), conv_a.as_slice()),
        (format!("B {}", dir_b.display()), conv_b.as_slice()),
    ];
    convergence::print_section(&conv_sets);
    // Held renders fail the compare from EITHER side: a delta measured against a
    // frozen-circuit golden is not a measurement of the change, and a new
    // baseline that ships non-solutions is not a reference. Baselines captured
    // before `diag_nr_hold_count` existed carry no hold key, so they cannot be
    // Held and this stays backward-compatible by construction.
    let held = convergence::held_count(&conv_sets);

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
        "diagnostics_differ_count": n_diag_differ,
        "counter_gate": {
            "effort_relative_bound": EFFORT_RELATIVE_BOUND,
            "effort_absolute_floor": EFFORT_ABSOLUTE_FLOOR,
            "gates_exit_code": true,
            "failed_render_count": n_counter_failed,
            "failed": counter_failed.iter().map(|r| serde_json::json!({
                "plugin": r.plugin,
                "program": r.program,
                "class": r.class,
                "findings": r.counter_findings,
            })).collect::<Vec<_>>(),
        },
        "convergence": {
            "nonconverged_fraction_threshold": convergence::NONCONVERGED_FRACTION,
            "gates_exit_code": false,
            "a": {
                "tally": convergence::tally(&conv_a),
                "not_clean": conv_a.iter()
                    .filter(|h| h.class != convergence::Class::Clean)
                    .collect::<Vec<_>>(),
            },
            "b": {
                "tally": convergence::tally(&conv_b),
                "not_clean": conv_b.iter()
                    .filter(|h| h.class != convergence::Class::Clean)
                    .collect::<Vec<_>>(),
            },
        },
        "nodal_sub_path_moved": sub_path_moved
            .iter()
            .map(|(p, a, b)| serde_json::json!({"plugin": p, "from": a, "to": b}))
            .collect::<Vec<_>>(),
        "results": results,
    });
    std::fs::write(json_out, serde_json::to_string_pretty(&json).unwrap())
        .map_err(|e| format!("write {}: {e}", json_out.display()))?;
    println!("JSON report: {}", json_out.display());

    if strict {
        // Only IDENTICAL passes, and codegen must match. NEGLIGIBLE is a
        // failure here by design — see the doc comment on `run`.
        return Ok(Outcome {
            failures: n_changed
                + n_missing
                + n_input_changed
                + n_negligible
                + source_differs.len()
                + source_missing.len()
                + n_diag_differ,
            held,
            counter_failures: n_counter_failed,
        });
    }
    Ok(Outcome {
        failures: n_changed + n_missing + n_input_changed,
        held,
        counter_failures: n_counter_failed,
    })
}

#[cfg(test)]
mod counter_gate_tests {
    use super::*;
    use std::path::PathBuf;

    fn diags(pairs: &[(&str, f64)]) -> BTreeMap<String, f64> {
        pairs.iter().map(|(k, v)| ((*k).to_string(), *v)).collect()
    }

    fn repo_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
    }

    fn walk(dir: &Path, ext: &str, out: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                walk(&p, ext, out);
            } else if p.extension().is_some_and(|x| x == ext) {
                out.push(p);
            }
        }
    }

    /// Every `pub diag_*` / `last_nr_iterations` declaration in a source or
    /// template file, the way the emitters write them (plain, or inside a
    /// string literal with `\x20` indentation — hence a `find`, not a prefix).
    fn declared_counters(text: &str) -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        for line in text.lines() {
            let mut rest = line;
            while let Some(pos) = rest.find("pub ") {
                rest = &rest[pos + 4..];
                let name: String = rest
                    .chars()
                    .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
                    .collect();
                let after = rest[name.len()..].trim_start();
                if after.starts_with(':')
                    && (name.starts_with("diag_") || name == "last_nr_iterations")
                {
                    out.insert(name);
                }
            }
        }
        out
    }

    /// The gate's whole premise: no counter the capture can write is
    /// unknown to the table. Scans the live emitters and templates (so a
    /// counter added to codegen fails here before it reaches a baseline) and
    /// every stored baseline's stats (so keys that older builds recorded stay
    /// classified too).
    #[test]
    fn every_emitted_counter_is_classified() {
        let root = repo_root();
        let mut files = Vec::new();
        walk(
            &root.join("crates/melange-solver/src/codegen"),
            "rs",
            &mut files,
        );
        walk(
            &root.join("crates/melange-solver/templates"),
            "tera",
            &mut files,
        );
        assert!(
            files.len() > 10,
            "codegen scan found {} files under {} — wrong root?",
            files.len(),
            root.display()
        );
        let mut found = BTreeSet::new();
        for f in &files {
            let text = std::fs::read_to_string(f).unwrap();
            found.extend(declared_counters(&text));
        }
        let mut stats_files = Vec::new();
        walk(&root.join("golden-baselines"), "json", &mut stats_files);
        for f in stats_files
            .iter()
            .filter(|p| p.to_string_lossy().ends_with(".stats.json"))
        {
            let Ok(txt) = std::fs::read_to_string(f) else {
                continue;
            };
            if let Ok(st) = serde_json::from_str::<Stats>(&txt) {
                found.extend(st.diagnostics.keys().cloned());
            }
        }
        assert!(
            found.len() >= 20,
            "scan found only {} counters: {found:?}",
            found.len()
        );
        eprintln!(
            "scan: {} counters declared by codegen/templates or recorded in baselines, \
             {} entries in COUNTER_CLASSES",
            found.len(),
            COUNTER_CLASSES.len()
        );
        let unclassified: Vec<&String> = found
            .iter()
            .filter(|k| counter_class(k).is_none())
            .collect();
        assert!(
            unclassified.is_empty(),
            "counters emitted by codegen (or recorded in a baseline) with no class in \
             COUNTER_CLASSES: {unclassified:?}. Add each to the table in compare.rs as \
             failure-path, effort or informational."
        );
        // The table is one table: no key twice.
        let mut seen = BTreeSet::new();
        for (k, _) in COUNTER_CLASSES {
            assert!(seen.insert(*k), "{k} appears twice in COUNTER_CLASSES");
        }
    }

    #[test]
    fn failure_path_increase_fails_decrease_passes() {
        let a = diags(&[("diag_nr_max_iter_count", 35.0)]);
        let b = diags(&[("diag_nr_max_iter_count", 38287.0)]);
        let f = judge_counters(&a, &b);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].kind, CounterFindingKind::FailurePathIncrease);
        assert_eq!(f[0].class, Some(CounterClass::FailurePath));
        assert!(f[0].message.contains("diag_nr_max_iter_count 35 -> 38287"));
        assert!(f[0].message.contains("first-divergence trace"));
        // Even one more counts.
        let b1 = diags(&[("diag_nr_max_iter_count", 36.0)]);
        assert_eq!(judge_counters(&a, &b1).len(), 1);
        // A decrease passes; so does equality.
        let b2 = diags(&[("diag_nr_max_iter_count", 0.0)]);
        assert!(judge_counters(&a, &b2).is_empty());
        assert!(judge_counters(&a, &a).is_empty());
    }

    /// A build that gained the counter AND fired it: an increase from an
    /// absent key. A build that lost the counter (absent in B) is not.
    #[test]
    fn failure_path_absent_counts_as_zero() {
        let a = diags(&[]);
        let b = diags(&[("diag_nr_hold_count", 1.0)]);
        let f = judge_counters(&a, &b);
        assert_eq!(f.len(), 1);
        assert!(f[0].message.contains("(absent) -> 1"));
        let b0 = diags(&[("diag_nr_hold_count", 0.0)]);
        assert!(judge_counters(&a, &b0).is_empty());
        assert!(judge_counters(&b, &a).is_empty());
    }

    /// The measured roundoff-class moves pass; the funkyinduct regression
    /// and a 20 % move fail; a 9 % move on a large count passes.
    #[test]
    fn effort_within_bound_passes_beyond_fails() {
        let a = diags(&[
            ("diag_ls_fail_count", 1125.0),
            ("diag_refactor_count", 23276.0),
        ]);
        let b = diags(&[
            ("diag_ls_fail_count", 1129.0),
            ("diag_refactor_count", 23293.0),
        ]);
        assert!(judge_counters(&a, &b).is_empty());

        let a = diags(&[("diag_ls_fail_count", 59408.0)]);
        let b = diags(&[("diag_ls_fail_count", 9_500_000.0)]);
        let f = judge_counters(&a, &b);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].kind, CounterFindingKind::EffortOutOfBound);
        assert!(f[0].message.contains("diag_ls_fail_count 59408 -> 9500000"));
        assert!(f[0].message.contains("first-divergence trace"));

        let a = diags(&[("diag_refactor_count", 1_000_000.0)]);
        assert!(judge_counters(&a, &diags(&[("diag_refactor_count", 1_090_000.0)])).is_empty());
        assert_eq!(
            judge_counters(&a, &diags(&[("diag_refactor_count", 1_200_000.0)])).len(),
            1
        );
        // Both directions: a solve that stopped working is also a move.
        assert_eq!(
            judge_counters(&a, &diags(&[("diag_refactor_count", 800_000.0)])).len(),
            1
        );
        // Exactly at the bound passes (the bound is inclusive).
        assert!(judge_counters(&a, &diags(&[("diag_refactor_count", 1_100_000.0)])).is_empty());
    }

    /// Baseline zero (or a few): the absolute floor decides.
    #[test]
    fn effort_from_zero_uses_absolute_floor() {
        let z = diags(&[("diag_ls_fail_count", 0.0)]);
        assert!(judge_counters(&z, &diags(&[("diag_ls_fail_count", 20.0)])).is_empty());
        let f = judge_counters(&z, &diags(&[("diag_ls_fail_count", 21.0)]));
        assert_eq!(f.len(), 1);
        assert!(f[0].message.contains("from zero"));
        // Absent in A reads as zero too.
        assert!(judge_counters(&diags(&[]), &diags(&[("diag_ls_fail_count", 20.0)])).is_empty());
        assert_eq!(
            judge_counters(&diags(&[]), &diags(&[("diag_ls_fail_count", 21.0)])).len(),
            1
        );
        // The recorded small move: noyce-transformer-triode/step refactors 5 -> 4.
        let a = diags(&[("diag_refactor_count", 5.0)]);
        assert!(judge_counters(&a, &diags(&[("diag_refactor_count", 4.0)])).is_empty());
        // The floor stops applying once 10 % of the baseline exceeds it.
        let a = diags(&[("diag_refactor_count", 300.0)]);
        assert!(judge_counters(&a, &diags(&[("diag_refactor_count", 330.0)])).is_empty());
        assert_eq!(
            judge_counters(&a, &diags(&[("diag_refactor_count", 331.0)])).len(),
            1
        );
    }

    /// An unclassified key fails the gate even when it did not move: the
    /// table is the gate, and a hole in it is a hole in the gate.
    #[test]
    fn unknown_key_fails_even_when_equal() {
        let a = diags(&[("diag_new_mechanism_count", 3.0)]);
        let f = judge_counters(&a, &a);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].kind, CounterFindingKind::Unclassified);
        assert_eq!(f[0].class, None);
        assert!(f[0].message.contains("COUNTER_CLASSES"));
    }

    /// A level and a last-sample reading are not counts.
    #[test]
    fn informational_never_fails() {
        let a = diags(&[("diag_peak_output", 0.8235), ("last_nr_iterations", 2.0)]);
        let b = diags(&[("diag_peak_output", 99.0), ("last_nr_iterations", 100.0)]);
        assert!(judge_counters(&a, &b).is_empty());
    }

    /// One render, several counters: one finding each, in key order.
    #[test]
    fn several_counters_on_one_render_each_report() {
        let a = diags(&[
            ("diag_nr_max_iter_count", 35.0),
            ("diag_ls_fail_count", 59408.0),
            ("diag_refactor_count", 100.0),
        ]);
        let b = diags(&[
            ("diag_nr_max_iter_count", 38287.0),
            ("diag_ls_fail_count", 9_500_000.0),
            ("diag_refactor_count", 101.0),
        ]);
        let f = judge_counters(&a, &b);
        assert_eq!(f.len(), 2);
        assert_eq!(f[0].counter, "diag_ls_fail_count");
        assert_eq!(f[1].counter, "diag_nr_max_iter_count");
    }

    fn release_baselines() -> Vec<PathBuf> {
        let mut dirs: Vec<PathBuf> = std::fs::read_dir(repo_root().join("golden-baselines"))
            .map(|rd| {
                rd.flatten()
                    .map(|e| e.path())
                    .filter(|p| {
                        p.is_dir()
                            && p.file_name()
                                .and_then(|n| n.to_str())
                                .is_some_and(|n| n.starts_with('v') && n.ends_with("-rc"))
                    })
                    .collect()
            })
            .unwrap_or_default();
        dirs.sort();
        dirs
    }

    /// End to end on the real reference: a baseline against itself passes
    /// every gate, counter gate included. Skips (loudly) when the gitignored
    /// baselines are not on this machine.
    #[test]
    fn real_baseline_self_compare_passes_counter_gate() {
        let dirs = release_baselines();
        let Some(dir) = dirs.last() else {
            eprintln!("no golden-baselines/v*-rc directory here; self-compare check skipped");
            return;
        };
        let json = std::env::temp_dir().join(format!(
            "golden-harness-self-compare-{}.json",
            std::process::id()
        ));
        let out = run(dir, dir, &json, false).expect("compare runs");
        let _ = std::fs::remove_file(&json);
        assert_eq!(out.failures, 0, "{}", dir.display());
        assert_eq!(out.held, 0, "{}", dir.display());
        assert_eq!(out.counter_failures, 0, "{}", dir.display());
    }

    /// Two different real baselines: the section is produced and the report
    /// carries it, whatever the verdict (a release pair may legitimately fail
    /// this gate — that is the finding to trace, not a panic).
    #[test]
    fn real_baselines_cross_compare_produces_counter_section() {
        let dirs = release_baselines();
        if dirs.len() < 2 {
            eprintln!("fewer than two golden-baselines/v*-rc directories; cross-compare skipped");
            return;
        }
        let (a, b) = (&dirs[dirs.len() - 2], &dirs[dirs.len() - 1]);
        let json = std::env::temp_dir().join(format!(
            "golden-harness-cross-compare-{}.json",
            std::process::id()
        ));
        let out = run(a, b, &json, false).expect("compare runs");
        let report: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&json).unwrap()).unwrap();
        let _ = std::fs::remove_file(&json);
        let gate = &report["counter_gate"];
        assert_eq!(gate["gates_exit_code"], true);
        assert_eq!(gate["failed_render_count"], out.counter_failures);
        assert_eq!(
            gate["failed"].as_array().map(|v| v.len()),
            Some(out.counter_failures)
        );
    }
}
