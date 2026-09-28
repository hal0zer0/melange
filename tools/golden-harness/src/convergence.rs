//! Convergence health of a recorded render.
//!
//! The audio comparison in `compare.rs` asks "did the output change?". It
//! cannot ask "was the output ever a solution?" — and those are different
//! questions. A Newton-Raphson solve that exhausts its iteration ceiling emits
//! the capped iterate anyway: a number that is bounded, smooth and entirely
//! plausible, but that does not satisfy the circuit equations. It correlates
//! 1.0 with itself run to run, so the golden gate passes it forever.
//!
//! This is not hypothetical. `steve-1073-preamp/step` has recorded
//! `diag_nr_max_iter_count = 43200` over 48000 frames — every single
//! post-step-edge sample — since the v0.1.5 baseline, at a peak of
//! -0.50 dBFS. Four baselines and three releases read that render as healthy
//! because nothing ever divided one recorded number by the other.
//!
//! # The rule
//!
//! `diag_nr_max_iter_count / internal_samples`, where
//! `internal_samples = frames * OVERSAMPLING_FACTOR`. At or above
//! [`NONCONVERGED_FRACTION`] the render is reported as NOT-CONVERGED; any
//! nonzero count below it is reported as MARGINAL. Both tiers print their
//! numbers, every run — the fraction is the measurand, the threshold only
//! decides how loudly it is said.
//!
//! # Why 20%, and why it is not a new number
//!
//! `melange simulate` has warned at exactly this fraction since 2026-08-15,
//! when a Ge astable cascade starved at `--max-iter 70`, latched every node,
//! and read as real physics for half a day (see `tools/melange-cli/src/main.rs`).
//! That threshold was set where it is because a *healthy* run of that same
//! circuit still shows ~5% onset-only max-iter samples, so 5% cries wolf.
//! Reusing it is deliberate: a second, differently-tuned line in a sibling tool
//! would itself be a way for the two to disagree silently.
//!
//! It also survives contact with this corpus. `golden-baselines/` holds 2192
//! recorded renders; 568 of them predate diagnostic recording and are
//! UNMEASURED. Across the 1624 that are measured, the per-internal-sample
//! max-iter fraction lands in exactly four places:
//!
//! ```text
//!   0.000              1586 renders  (97.7% of measured)
//!   0.0094% - 0.041%     25 renders  qapla-1a, steve sine1k/sweep, tungsten, uniquorn
//!   0.44%                 9 renders  gold-press-riaa/sweep (OS=4)
//!   90.0%                 4 renders  steve-1073-preamp/step
//! ```
//!
//! The band from 0.44% to 90% — a factor of 206 — is empty. Any threshold
//! placed anywhere inside it partitions this corpus identically, so the rule is
//! insensitive to the exact value and 20% is not fitted to any deck.
//!
//! # What is deliberately NOT the rule
//!
//! - `diag_be_fallback_count` is not a failure signal. `qapla-1a/potsweep`
//!   runs 95.7% BE fallback with a max-iter fraction of 0.0094% and a final
//!   solve of 1 iteration: that is the runtime BE-latch working as designed.
//!   Gating on it would flag a healthy circuit and teach everyone to ignore
//!   the banner.
//! - `diag_ls_fail_count` and `diag_refactor_count` are recoverable events, not
//!   failures; 56 healthy renders carry nonzero counts. They are reported as
//!   corroborating context on a flagged render and never decide the class.
//!
//! # Report-only, by design
//!
//! Nothing here changes an exit code. "Did not converge" is a property of ONE
//! baseline, equally true of the reference side, so failing a comparison on it
//! would fail every future comparison against a baseline that contains such a
//! render — including release gates for changes that had nothing to do with it.
//! A permanently-red gate is a gate that gets ignored, which is how the
//! original evidence went unread for four baselines. It is a distinct, loud
//! category instead: "the output changed" and "the output was never a
//! solution" are different facts and stay separately legible.

use crate::stats::Stats;
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

/// Fraction of internal samples at which NR max-iter exhaustion is called a
/// non-convergence rather than a transient. Same line `melange simulate`
/// warns at; see the module docs for the derivation and the corpus evidence.
pub const NONCONVERGED_FRACTION: f64 = 0.20;

/// Diagnostic keys this module reads. A render whose stats lack
/// [`MAX_ITER_KEY`] is UNKNOWN, never "clean" — absence of evidence is not
/// evidence of convergence.
const MAX_ITER_KEY: &str = "diag_nr_max_iter_count";
const LS_FAIL_KEY: &str = "diag_ls_fail_count";
const REFACTOR_KEY: &str = "diag_refactor_count";
const BE_FALLBACK_KEY: &str = "diag_be_fallback_count";
const LAST_ITERS_KEY: &str = "last_nr_iterations";
/// The death-spiral hold: samples where EVERY Newton path failed and the
/// previous state was committed as the answer. Unlike the cap, this is not a
/// degree of difficulty — it is a non-solution shipped as output.
const HOLD_KEY: &str = "diag_nr_hold_count";
/// Schur's equivalent: no hold, so an unconverged sample is committed as
/// output instead of frozen. Same silent-wrong class, so it is ranked with the
/// hold and fails the same gates (design review).
const UNCONVERGED_COMMIT_KEY: &str = "diag_nr_unconverged_commit_count";

#[derive(Serialize, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[serde(rename_all = "kebab-case")]
pub enum Class {
    /// Baseline predates diagnostic recording — convergence is unmeasured.
    Unknown,
    /// NR never hit its ceiling.
    Clean,
    /// Nonzero but below [`NONCONVERGED_FRACTION`]: isolated capped samples,
    /// typically an onset transient. Reported with its numbers, not flagged.
    Marginal,
    /// At or above [`NONCONVERGED_FRACTION`]: the emitted samples on that
    /// fraction of the render are capped iterates, not solutions.
    NotConverged,
    /// The death-spiral hold fired: on `hold_count` samples every Newton path
    /// failed and the PREVIOUS state was committed as this sample's output.
    ///
    /// Ranked above [`Class::NotConverged`] because it is a different KIND of
    /// claim, not a worse degree of the same one. A capped-but-recovered sample
    /// is a converged solution reached by another consistent scheme; a held
    /// sample is not a solution at all. Under constant input the hold is also a
    /// fixed point — the next sample re-poses the identical problem and fails
    /// identically — so one hard sample can freeze a render to its end
    /// (design review).
    Held,
}

impl Class {
    pub fn as_str(self) -> &'static str {
        match self {
            Class::Unknown => "UNKNOWN",
            Class::Clean => "CLEAN",
            Class::Marginal => "MARGINAL",
            Class::NotConverged => "NOT-CONVERGED",
            Class::Held => "HELD",
        }
    }
}

#[derive(Serialize, Clone)]
pub struct Health {
    pub plugin: String,
    pub program: String,
    pub class: Class,
    pub frames: usize,
    /// `OVERSAMPLING_FACTOR` read from the baseline's own stored
    /// `circuit.rs`. `None` when that file is absent, in which case
    /// `internal_samples` falls back to `frames` and the reported fraction is
    /// an OVER-estimate by exactly the true factor — the safe direction for a
    /// health check, but say so rather than imply precision.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub oversampling: Option<u32>,
    pub oversampling_known: bool,
    pub internal_samples: u64,
    pub max_iter_count: f64,
    /// `max_iter_count / internal_samples`. Can exceed 1.0: a sample may
    /// exhaust both its trapezoidal solve and its BE-fallback solve, and
    /// sub-sample-fire steps add solves the frame count does not see.
    pub fraction: f64,
    /// `max_iter` from the generated module's provenance header, i.e. the
    /// ceiling that was hit.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_iter: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_nr_iterations: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ls_fail_per_sample: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refactor_per_sample: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub be_fallback_fraction: Option<f64>,
    /// Samples the death-spiral hold committed. `None` means no count was
    /// recorded — see [`Self::hold_instrumented`] for whether that is a true
    /// zero or genuinely unmeasured.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hold_count: Option<f64>,
    /// Whether this render's build has a hold path at all. `Some(false)` is a
    /// TRUE ZERO — DK and M = 0 nodal builds cannot hold, and re-capturing them
    /// will never produce a counter. `None` is genuinely unmeasured. Keeping
    /// these apart matters: a report that tells someone to "re-capture to find
    /// out" about a build with no hold mechanism is confidently wrong, and a
    /// wrong reason ends the reader's search at the wrong place.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hold_instrumented: Option<bool>,
    /// Which mechanism this build has, named in the report so a reader is not
    /// left to infer it from the route.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hold_mechanism: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hold_fraction: Option<f64>,
    pub peak_dbfs: f64,
}

impl Health {
    fn key(&self) -> String {
        format!("{}/{}", self.plugin, self.program)
    }
}

/// Classify one recorded render.
pub fn assess(
    plugin: &str,
    program: &str,
    st: &Stats,
    oversampling: Option<u32>,
    max_iter: Option<u32>,
    hold_instrumented: Option<bool>,
) -> Health {
    let d = &st.diagnostics;
    let os = oversampling.unwrap_or(1).max(1) as u64;
    let internal = (st.frames as u64).saturating_mul(os).max(1);
    let rate = |k: &str| d.get(k).map(|v| v / internal as f64);

    let (mut class, count, fraction) = match d.get(MAX_ITER_KEY) {
        None => (Class::Unknown, 0.0, 0.0),
        Some(&c) => {
            let f = c / internal as f64;
            let cls = if c <= 0.0 {
                Class::Clean
            } else if f >= NONCONVERGED_FRACTION {
                Class::NotConverged
            } else {
                Class::Marginal
            };
            (cls, c, f)
        }
    };

    // The hold outranks every cap-fraction verdict, at ANY count. One held
    // sample is one sample of output that was never solved; there is no
    // fraction of that which is acceptable, so there is no threshold here.
    // A baseline with no hold key is left on its cap-based class: absence of
    // the counter is absence of evidence, not evidence of zero.
    // Both mechanisms ship a sample that was never solved. full-LU freezes the
    // previous state, Schur commits the diverged iterate; neither is a solution,
    // so they share a class and a gate. A build declares only the one it has.
    let hold_count = match (d.get(HOLD_KEY), d.get(UNCONVERGED_COMMIT_KEY)) {
        (None, None) => None,
        (a, b) => Some(a.copied().unwrap_or(0.0) + b.copied().unwrap_or(0.0)),
    };
    let hold_mechanism = if d.contains_key(UNCONVERGED_COMMIT_KEY) {
        Some("unconverged-commit (schur)")
    } else if d.contains_key(HOLD_KEY) {
        Some("death-spiral hold (full-lu)")
    } else {
        None
    };
    let hold_fraction = hold_count.map(|h| h / internal as f64);
    if hold_count.is_some_and(|h| h > 0.0) {
        class = Class::Held;
    }

    Health {
        plugin: plugin.to_string(),
        program: program.to_string(),
        class,
        frames: st.frames,
        oversampling,
        oversampling_known: oversampling.is_some(),
        internal_samples: internal,
        max_iter_count: count,
        fraction,
        max_iter,
        last_nr_iterations: d.get(LAST_ITERS_KEY).copied(),
        ls_fail_per_sample: rate(LS_FAIL_KEY),
        refactor_per_sample: rate(REFACTOR_KEY),
        be_fallback_fraction: rate(BE_FALLBACK_KEY),
        hold_count,
        hold_fraction,
        hold_instrumented,
        hold_mechanism,
        peak_dbfs: st.peak_dbfs,
    }
}

/// `OVERSAMPLING_FACTOR` and provenance `max_iter` from a baseline's stored
/// generated module. Read from the baseline itself rather than from the
/// manifest, so the numbers describe the code that actually ran and older
/// baselines stay readable without their original manifest.
/// Returns `(oversampling, max_iter, hold_instrumented)`.
///
/// `hold_instrumented` distinguishes two very different reasons a render can
/// have no hold count, which must never be reported as the same thing:
/// `Some(false)` — this build has NO hold path (DK, or nodal with M = 0 and no
/// behavioral source or saturating inductor), so it structurally cannot hold and
/// zero is the true answer; `None` — the baseline has no stored `circuit.rs` at
/// all, so nothing is known. Only the latter is unmeasured.
fn build_constants(dir: &Path, plugin: &str) -> (Option<u32>, Option<u32>, Option<bool>) {
    let Ok(code) = std::fs::read_to_string(dir.join(plugin).join("circuit.rs")) else {
        return (None, None, None);
    };
    let hold_instrumented = Some(
        code.contains("diag_nr_hold_count") || code.contains("diag_nr_unconverged_commit_count"),
    );
    let mut os = None;
    let mut max_iter = None;
    for line in code.lines().take(200) {
        let t = line.trim();
        if os.is_none() {
            if let Some(rest) = t.strip_prefix("pub const OVERSAMPLING_FACTOR: usize =") {
                os = rest.trim().trim_end_matches(';').trim().parse::<u32>().ok();
            }
        }
        if max_iter.is_none() {
            if let Some(i) = t.find("\"max_iter\":") {
                let tail = &t[i + "\"max_iter\":".len()..];
                let digits: String = tail.chars().take_while(|c| c.is_ascii_digit()).collect();
                max_iter = digits.parse::<u32>().ok();
            }
        }
        if os.is_some() && max_iter.is_some() {
            break;
        }
    }
    (os, max_iter, hold_instrumented)
}

fn list_plugins(dir: &Path) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    if let Ok(entries) = std::fs::read_dir(dir) {
        for e in entries.flatten() {
            let p = e.path();
            if p.is_dir() {
                if let Some(n) = p.file_name().and_then(|s| s.to_str()) {
                    out.insert(n.to_string());
                }
            }
        }
    }
    out
}

/// Assess every render recorded in a baseline directory.
pub fn scan(dir: &Path) -> Vec<Health> {
    let mut out = Vec::new();
    for plugin in list_plugins(dir) {
        let (os, max_iter, hold_instrumented) = build_constants(dir, &plugin);
        let Ok(entries) = std::fs::read_dir(dir.join(&plugin)) else {
            continue;
        };
        let mut names: Vec<String> = Vec::new();
        for e in entries.flatten() {
            let p = e.path();
            let Some(name) = p.file_name().and_then(|s| s.to_str()) else {
                continue;
            };
            if let Some(stem) = name.strip_suffix(".stats.json") {
                names.push(stem.to_string());
            }
        }
        names.sort();
        for program in names {
            let p = dir.join(&plugin).join(format!("{program}.stats.json"));
            let Ok(txt) = std::fs::read_to_string(&p) else {
                continue;
            };
            let Ok(st) = serde_json::from_str::<Stats>(&txt) else {
                continue;
            };
            out.push(assess(
                &plugin,
                &program,
                &st,
                os,
                max_iter,
                hold_instrumented,
            ));
        }
    }
    out.sort_by(|a, b| {
        b.fraction
            .total_cmp(&a.fraction)
            .then_with(|| a.key().cmp(&b.key()))
    });
    out
}

/// Per-class render counts.
pub fn tally(healths: &[Health]) -> BTreeMap<&'static str, usize> {
    let mut m = BTreeMap::new();
    for c in [
        Class::NotConverged,
        Class::Marginal,
        Class::Clean,
        Class::Unknown,
    ] {
        m.insert(c.as_str(), 0usize);
    }
    for h in healths {
        *m.entry(h.class.as_str()).or_default() += 1;
    }
    m
}

fn summary_line(healths: &[Health]) -> String {
    let t = tally(healths);
    format!(
        "{} not-converged, {} marginal, {} clean, {} unknown",
        t["NOT-CONVERGED"], t["MARGINAL"], t["CLEAN"], t["UNKNOWN"]
    )
}

fn detail_lines(h: &Health, indent: &str) -> Vec<String> {
    let mut out = Vec::new();
    let cap = h
        .max_iter
        .map(|m| format!("{m}-iter cap"))
        .unwrap_or_else(|| "iteration cap".into());
    let denom = if h.oversampling_known {
        format!(
            "{} internal samples ({} frames x{})",
            h.internal_samples,
            h.frames,
            h.oversampling.unwrap_or(1)
        )
    } else {
        format!(
            "{} frames (oversampling unknown — fraction is an upper bound)",
            h.frames
        )
    };
    out.push(format!(
        "{indent}{:34} {:>7.3}%  {} of {} hit the {cap}",
        h.key(),
        h.fraction * 100.0,
        h.max_iter_count,
        denom
    ));
    let mut ctx = format!("{indent}  peak {:+.2} dBFS", h.peak_dbfs);
    if let Some(l) = h.last_nr_iterations {
        ctx.push_str(&format!(", last solve {l:.0} iters"));
    }
    if let Some(v) = h.ls_fail_per_sample {
        if v > 0.0 {
            ctx.push_str(&format!(", line-search fails {v:.2}/sample"));
        }
    }
    if let Some(v) = h.refactor_per_sample {
        if v > 0.0 {
            ctx.push_str(&format!(", LU refactors {v:.2}/sample"));
        }
    }
    if let Some(v) = h.be_fallback_fraction {
        if v > 0.0 {
            ctx.push_str(&format!(", BE fallback {:.1}%", v * 100.0));
        }
    }
    out.push(ctx);
    out
}

/// Print the convergence section for one or more labelled baselines.
///
/// Prints unconditionally — a clean corpus says so out loud, so that a run in
/// which the section is missing is visibly different from one in which it
/// passed.
pub fn print_section(sets: &[(String, &[Health])]) {
    println!("\n---- CONVERGENCE HEALTH (Newton-Raphson) ----");
    for (label, hs) in sets {
        println!("  {label}: {}", summary_line(hs));
    }

    // HELD first: it is the only class that fails a gate.
    let held: Vec<(&str, &Health)> = sets
        .iter()
        .flat_map(|(l, hs)| {
            hs.iter()
                .filter(|h| h.class == Class::Held)
                .map(move |h| (l.as_str(), h))
        })
        .collect();
    if !held.is_empty() {
        println!("\n  ######################################################");
        println!(
            "  ##  {} RENDER(S) SHIPPED SAMPLES THAT ARE NOT SOLUTIONS",
            held.len()
        );
        println!("  ##  Every Newton path failed on those samples and the");
        println!("  ##  PREVIOUS state was committed as the answer. Bounded and");
        println!("  ##  smooth, so peak/RMS/clamp/correlation all read healthy.");
        println!("  ##  Under constant input the hold is a FIXED POINT: the next");
        println!("  ##  sample re-poses the identical problem and fails the same");
        println!("  ##  way, so one hard sample can freeze a render to its end.");
        println!("  ##  THIS FAILS THE GATE. (design review)");
        println!("  ######################################################");
        for (label, h) in &held {
            println!(
                "    [{label}] {:34} {} of {} internal samples ({:.3}%) via {}, peak {:.2} dBFS",
                h.key(),
                h.hold_count.unwrap_or(0.0),
                h.internal_samples,
                h.hold_fraction.unwrap_or(0.0) * 100.0,
                h.hold_mechanism.unwrap_or("unknown mechanism"),
                h.peak_dbfs
            );
            for l in detail_lines(h, "      ") {
                println!("{l}");
            }
        }
    }

    let flagged: Vec<(&str, &Health)> = sets
        .iter()
        .flat_map(|(l, hs)| {
            hs.iter()
                .filter(|h| h.class == Class::NotConverged)
                .map(move |h| (l.as_str(), h))
        })
        .collect();
    if !flagged.is_empty() {
        println!("\n  ######################################################");
        println!("  ##  {} RENDER(S) DID NOT CONVERGE", flagged.len());
        println!("  ##  The emitted samples on that fraction of the render are");
        println!("  ##  CAPPED NR ITERATES, NOT SOLUTIONS. A capped iterate is");
        println!("  ##  bounded and smooth, so peak/RMS/correlation all look");
        println!("  ##  healthy and the audio gate cannot see it.");
        println!("  ######################################################");
        for (label, h) in &flagged {
            println!("    [{label}]");
            for l in detail_lines(h, "      ") {
                println!("{l}");
            }
        }
    }

    let marginal: Vec<(&str, &Health)> = sets
        .iter()
        .flat_map(|(l, hs)| {
            hs.iter()
                .filter(|h| h.class == Class::Marginal)
                .map(move |h| (l.as_str(), h))
        })
        .collect();
    if !marginal.is_empty() {
        println!(
            "\n  MARGINAL ({} render(s), nonzero but under {:.0}%):",
            marginal.len(),
            NONCONVERGED_FRACTION * 100.0
        );
        for (label, h) in &marginal {
            println!(
                "    [{label}] {:34} {:>7.3}%  ({} of {})",
                h.key(),
                h.fraction * 100.0,
                h.max_iter_count,
                h.internal_samples
            );
        }
    }

    let unknown: usize = sets
        .iter()
        .map(|(_, hs)| hs.iter().filter(|h| h.class == Class::Unknown).count())
        .sum();
    if unknown > 0 {
        println!(
            "\n  {unknown} render(s) UNMEASURED — captured before solver diagnostics were \
             recorded; convergence is unknown, not clean."
        );
    }
    // Two different reasons for a missing hold count, reported separately. A
    // build with no hold path CANNOT hold, so zero is the true answer and
    // telling anyone to re-capture it would be a confidently wrong instruction.
    let no_hold_path: usize = sets
        .iter()
        .map(|(_, hs)| {
            hs.iter()
                .filter(|h| h.hold_instrumented == Some(false))
                .count()
        })
        .sum();
    let unmeasured_hold: usize = sets
        .iter()
        .map(|(_, hs)| {
            hs.iter()
                .filter(|h| h.hold_instrumented.is_none() && h.hold_count.is_none())
                .count()
        })
        .sum();
    if no_hold_path > 0 {
        println!(
            "\n  {no_hold_path} render(s) have NO HOLD PATH in their build (DK, or nodal at \
             M = 0 with no behavioral source or saturating inductor). They cannot hold; \
             zero is the true answer, not a gap."
        );
    }
    if unmeasured_hold > 0 {
        println!(
            "\n  {unmeasured_hold} render(s) are UNMEASURED for the hold — no stored \
             `circuit.rs`, so whether their build even has a hold path is unknown. \
             Re-capture to find out."
        );
    }

    if held.is_empty() {
        if flagged.is_empty() {
            println!(
                "\n  No render was held, and none exceeded the {:.0}% non-convergence line. \
                 The capped-sample classes below HELD are report-only; HELD is the one \
                 that fails a gate.",
                NONCONVERGED_FRACTION * 100.0
            );
        } else {
            println!(
                "\n  No render was held. Capped-but-recovered samples are a converged \
                 solution reached by another consistent scheme, so NOT-CONVERGED and \
                 MARGINAL are REPORT-ONLY and do not change the exit code."
            );
        }
    } else {
        println!(
            "\n  {} HELD render(s) FAIL this run. Capped-but-recovered counts above \
             remain report-only — they are a different class (design review).",
            held.len()
        );
    }
}

/// How many renders in these sets shipped non-solutions. The gate's number.
pub fn held_count(sets: &[(String, &[Health])]) -> usize {
    sets.iter()
        .map(|(_, hs)| hs.iter().filter(|h| h.class == Class::Held).count())
        .sum()
}

/// Standalone `convergence` subcommand: assess each baseline directory given.
///
/// Returns how many renders were HELD, so the caller can fail on them. The
/// cap-based classes stay report-only; see [`Class::Held`].
pub fn run(dirs: &[std::path::PathBuf]) -> Result<usize, String> {
    let mut owned: Vec<(String, Vec<Health>)> = Vec::new();
    for d in dirs {
        if !d.is_dir() {
            return Err(format!("{} is not a directory", d.display()));
        }
        owned.push((d.display().to_string(), scan(d)));
    }
    let sets: Vec<(String, &[Health])> = owned
        .iter()
        .map(|(l, h)| (l.clone(), h.as_slice()))
        .collect();
    print_section(&sets);
    Ok(held_count(&sets))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn stats(frames: usize, diags: &[(&str, f64)]) -> Stats {
        let mut d = BTreeMap::new();
        for (k, v) in diags {
            d.insert((*k).to_string(), *v);
        }
        Stats {
            channels: 1,
            frames,
            sample_rate: 48000.0,
            peak_dbfs: -0.5,
            rms_dbfs: -20.0,
            dc_mean: 0.0,
            nan_count: 0,
            inf_count: 0,
            bands_dbfs: BTreeMap::new(),
            diagnostics: d,
        }
    }

    /// A baseline with no diagnostics recorded is UNMEASURED, never CLEAN.
    /// Absence of evidence is not evidence of convergence — 568 renders in
    /// `golden-baselines/` are in exactly this state.
    #[test]
    fn missing_counter_is_unknown_not_clean() {
        let h = assess(
            "p",
            "step",
            &stats(48000, &[]),
            Some(1),
            Some(100),
            Some(true),
        );
        assert_eq!(h.class, Class::Unknown);
    }

    #[test]
    fn zero_count_is_clean() {
        let h = assess(
            "p",
            "step",
            &stats(48000, &[(MAX_ITER_KEY, 0.0)]),
            Some(1),
            Some(100),
            Some(true),
        );
        assert_eq!(h.class, Class::Clean);
        assert_eq!(h.fraction, 0.0);
    }

    /// The recorded `steve-1073-preamp/step` numbers, verbatim.
    #[test]
    fn steve_step_is_not_converged() {
        let h = assess(
            "steve-1073-preamp",
            "step",
            &stats(
                48000,
                &[
                    (MAX_ITER_KEY, 43200.0),
                    (BE_FALLBACK_KEY, 43199.0),
                    (LS_FAIL_KEY, 2246353.0),
                    (REFACTOR_KEY, 4190402.0),
                    (LAST_ITERS_KEY, 100.0),
                ],
            ),
            Some(1),
            Some(100),
            Some(true),
        );
        assert_eq!(h.class, Class::NotConverged);
        assert!((h.fraction - 0.9).abs() < 1e-12);
        assert!((h.ls_fail_per_sample.unwrap() - 46.799).abs() < 1e-3);
    }

    /// The recorded `gold-press-riaa/sweep` numbers. At OS=4 this is 0.44%,
    /// not the 1.75% a naive divide-by-frames would report — the denominator
    /// has to be internal samples or the corpus' cleanest gap moves.
    #[test]
    fn oversampling_scales_the_denominator() {
        let st = stats(192000, &[(MAX_ITER_KEY, 3352.0)]);
        let os4 = assess("gold-press", "sweep", &st, Some(4), Some(100), Some(true));
        assert_eq!(os4.internal_samples, 768000);
        assert!((os4.fraction - 0.004365).abs() < 1e-6);
        assert_eq!(os4.class, Class::Marginal);

        // Unknown oversampling falls back to frames: an OVER-estimate, which
        // is the safe direction, and the flag says so.
        let unknown = assess("gold-press", "sweep", &st, None, Some(100), Some(true));
        assert!(!unknown.oversampling_known);
        assert!(unknown.fraction > os4.fraction);
    }

    /// The threshold is inclusive, and a hair under it is still MARGINAL.
    #[test]
    fn threshold_boundary_is_inclusive() {
        let at = assess(
            "p",
            "step",
            &stats(1000, &[(MAX_ITER_KEY, 200.0)]),
            Some(1),
            None,
            Some(true),
        );
        assert_eq!(at.class, Class::NotConverged);
        let under = assess(
            "p",
            "step",
            &stats(1000, &[(MAX_ITER_KEY, 199.0)]),
            Some(1),
            None,
            Some(true),
        );
        assert_eq!(under.class, Class::Marginal);
    }

    /// Heavy BE fallback is a working recovery ladder, not a failure.
    /// `qapla-1a/potsweep` runs 95.7% BE fallback and converges: if this ever
    /// classifies as NOT-CONVERGED the rule has started crying wolf.
    #[test]
    fn heavy_be_fallback_alone_is_not_a_failure() {
        let h = assess(
            "qapla-1a",
            "potsweep",
            &stats(
                96000,
                &[
                    (MAX_ITER_KEY, 9.0),
                    (BE_FALLBACK_KEY, 91909.0),
                    (LAST_ITERS_KEY, 1.0),
                ],
            ),
            Some(1),
            Some(100),
            Some(true),
        );
        assert_eq!(h.class, Class::Marginal);
        assert!(h.be_fallback_fraction.unwrap() > 0.95);
    }

    /// Heavy line-search failure alone is likewise not a failure:
    /// `wurli-power-amp/sine1k` records 14009 line-search fails with a
    /// max-iter count of zero and is CLEAN.
    #[test]
    fn line_search_failures_alone_are_not_a_failure() {
        let h = assess(
            "wurli-power-amp",
            "sine1k",
            &stats(96000, &[(MAX_ITER_KEY, 0.0), (LS_FAIL_KEY, 14009.0)]),
            Some(1),
            Some(100),
            Some(true),
        );
        assert_eq!(h.class, Class::Clean);
    }
}
