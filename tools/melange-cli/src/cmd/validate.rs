use crate::args::parse_bjt_fa_mode;
use crate::circuits;
use crate::common::{is_build_detail, load_circuit_text};
use anyhow::{Context, Result};
use std::path::PathBuf;

/// `melange validate`'s stimulus: a sine at this frequency.
const VALIDATE_STIMULUS_HZ: f64 = 1000.0;
/// The rate sweep's stimulus. Incommensurate with every rate it renders: at a
/// frequency that divides the sample rate, a clipping deck's corners fall at the
/// same sub-sample phase every period, a different one at each rate, and that
/// fixed phase has twice forged a PLATEAU / DIVERGES verdict on a converging
/// deck. Stated, not derived.
const RATE_SWEEP_STIMULUS_HZ: f64 = 997.3;
/// Stimulus periods left out of every metric at the start of a validate render.
const VALIDATE_SETTLE_PERIODS: f64 = 20.0;
/// The default profile's peak bound, relative to the reference's peak over the
/// compared window. A stated bound (a 1 % peak error on the reference's own
/// scale), not a derived one. Measured 2026-09-29 over 85 local-corpus decks,
/// both sides with the settle window, against the absolute 20 mV: +2 passes, 0
/// regressions, the nearest passing deck at 0.90 %; at 0.5 %, +1 pass and 3
/// regressions (decks at 0.59-0.77 %).
const VALIDATE_PEAK_RELATIVE: f64 = 0.01;

/// Optional per-metric tolerance overrides from the CLI, applied on top of the
/// --relaxed/strict base profile. `None` fields keep the profile value.
#[derive(Default)]
pub(crate) struct ToleranceOverrides {
    /// RMS error tolerance, percent (2.0 = 2%).
    pub(crate) rms_pct: Option<f64>,
    /// Peak error tolerance, volts (absolute).
    pub(crate) peak_v: Option<f64>,
    /// Max relative error tolerance, percent (5.0 = 5%).
    pub(crate) max_rel_pct: Option<f64>,
    /// Minimum correlation coefficient (0.0–1.0).
    pub(crate) corr_min: Option<f64>,
    /// THD error tolerance, dB.
    pub(crate) thd_db: Option<f64>,
}

/// Dimension-reduction modes for the validation front end. Mirrors the
/// `melange compile` flags of the same names so that `validate` builds the
/// circuit the CLI ships — and so `--bjt-fa off` can attribute a residual to
/// the reduction.
pub(crate) struct ReductionModes<'a> {
    pub(crate) bjt_fa: &'a str,
    pub(crate) tube_grid_fa: &'a str,
    // Diagnostics (not reductions): melange-side integrator override, for
    // attributing integrator error against ngspice. Oversampling is NOT here —
    // it is not a diagnostic but part of the shipped build, and it rides its
    // own field on `ValidateOptions`.
    pub(crate) backward_euler: bool,
    pub(crate) force_trap: bool,
}

/// `melange validate`'s options (named, so two same-typed options cannot be
/// passed in each other's place).
pub(crate) struct ValidateOptions<'a> {
    pub(crate) output_node: &'a str,
    pub(crate) sample_rate: f64,
    pub(crate) duration: f64,
    pub(crate) amplitude: f64,
    pub(crate) input_node: &'a str,
    pub(crate) csv_output: Option<&'a PathBuf>,
    pub(crate) relaxed: bool,
    pub(crate) tol: ToleranceOverrides,
    pub(crate) reductions: ReductionModes<'a>,
    pub(crate) oversampling: usize,
    pub(crate) rate_sweep: bool,
    /// `-v/--verbose`: print the progress steps and every solver counter.
    pub(crate) verbose: bool,
}

pub(crate) fn validate_circuit_source(
    circuit_source: &circuits::CircuitSource,
    opts: ValidateOptions<'_>,
) -> Result<()> {
    let ValidateOptions {
        output_node,
        sample_rate,
        duration,
        amplitude,
        input_node,
        csv_output,
        relaxed,
        tol,
        reductions,
        oversampling,
        rate_sweep,
        verbose,
    } = opts;
    // Match parse-time node normalization (lowercase, gnd→0).
    let input_node_owned = melange_solver::parser::normalize_node_name(input_node);
    let input_node = input_node_owned.as_str();
    let output_node_owned = melange_solver::parser::normalize_node_name(output_node);
    let output_node = output_node_owned.as_str();
    use melange_validate::{
        comparison::ComparisonConfig, spice_runner::is_ngspice_available,
        validate_circuit_with_options, ValidationOptions,
    };

    println!("melange validate");
    println!("  Source: {}", circuit_source.name());
    println!("  Output node: {}", output_node);
    println!("  Input node: {}", input_node);
    println!("  Sample rate: {} Hz", sample_rate);
    println!("  Duration: {}s", duration);
    println!("  Amplitude: {}V", amplitude);
    println!(
        "  Tolerances: {}",
        if relaxed { "relaxed" } else { "strict" }
    );
    if oversampling > 1 {
        // Say what is being validated and what was done about the filters,
        // BEFORE the number appears. This run measures different DSP from the
        // 1x run above it in someone's scrollback.
        println!(
            "  Oversampling: {}\u{d7} (solver at {:.0} Hz internally)",
            oversampling,
            sample_rate * oversampling as f64
        );

        println!(
            "    The emitted code interpolates and decimates through polyphase IIR \
             half-band allpass chains,"
        );
        println!(
            "    whose frequency-dependent phase stays in the comparison. The reference is \
             NOT filtered:"
        );
        println!(
            "    it is aligned to the melange output by one best-fit constant delay \
             (analytic seed {:.2} samples",
            melange_validate::oversampling_round_trip_group_delay_samples(
                oversampling,
                sample_rate,
                1000.0
            )
        );
        println!("    at 1 kHz), the same alignment the 1\u{d7} run gets.");
        println!("    Tolerances are unchanged from the 1\u{d7} run.");
    }
    println!();

    // The first VALIDATE_SETTLE_PERIODS stimulus periods are left out of every
    // metric: the sine starts at t = 0 with a step in its derivative, and the
    // two engines' onset transients differ on a scale set by the stimulus.
    // Checked first: it depends only on the arguments, so a render too short to
    // compare is refused before anything about the environment is.
    let settle_time_s = VALIDATE_SETTLE_PERIODS / VALIDATE_STIMULUS_HZ;
    if settle_time_s >= duration {
        anyhow::bail!(
            "--duration {duration} s is inside the settle window ({VALIDATE_SETTLE_PERIODS} \
             periods of the {VALIDATE_STIMULUS_HZ} Hz stimulus = {settle_time_s} s), so nothing \
             would be compared; use a longer --duration"
        );
    }

    // Step 1: Check ngspice availability
    if verbose {
        println!("Step 1: Checking ngspice...");
    }
    if !is_ngspice_available() {
        anyhow::bail!(
            "ngspice is not installed or not found in PATH.\n\
             Install it with: sudo apt install ngspice (Debian/Ubuntu)\n\
             or: brew install ngspice (macOS)"
        );
    }
    if verbose {
        println!("  ngspice found");
    }

    // Step 2: Get circuit netlist as a file path
    // validate_circuit needs a file path. For local files, use directly.
    // For builtins/URLs, write to a secure temp file (random name, auto-cleanup on drop).
    // Uses tempfile::NamedTempFile to avoid TOCTOU/symlink clobber attacks from
    // predictable PID-based paths on shared hosts.
    if verbose {
        println!("Step 2: Loading circuit...");
    }
    use std::io::Write as _;
    let (netlist_path, _temp_file): (std::path::PathBuf, Option<tempfile::NamedTempFile>) =
        match circuit_source {
            circuits::CircuitSource::Local { path } => {
                // Verify the file exists
                if !path.exists() {
                    anyhow::bail!("Circuit file not found: {}", path.display());
                }
                (path.clone(), None)
            }
            src @ (circuits::CircuitSource::Builtin { .. }
            | circuits::CircuitSource::Url { .. }
            | circuits::CircuitSource::Friendly { .. }) => {
                let content = load_circuit_text(src, &|l| {
                    if verbose || !is_build_detail(l) {
                        println!("{l}")
                    }
                })?;
                // melange-validate reads the deck from a path. The file is
                // removed when `tmp` drops; a killed run cannot drop it, so
                // it lives in a scratch dir of its own that every run sweeps
                // of leftovers (see `sweep_stale_netlists`).
                let scratch = std::env::temp_dir().join("melange-validate-netlists");
                std::fs::create_dir_all(&scratch).with_context(|| {
                    format!("Failed to create scratch dir {}", scratch.display())
                })?;
                sweep_stale_netlists(&scratch, STALE_NETLIST_AGE);
                let mut tmp = tempfile::Builder::new()
                    .prefix("melange_validate_")
                    .suffix(".cir")
                    .tempfile_in(&scratch)
                    .context("Failed to create temp netlist file")?;
                tmp.write_all(content.as_bytes())
                    .context("Failed to write temp netlist")?;
                tmp.flush().context("Failed to flush temp netlist")?;
                let path = tmp.path().to_path_buf();
                (path, Some(tmp))
            }
        };

    // Step 3: Generate test input signal (1kHz sine)
    if verbose {
        println!(
            "Step 3: Generating test signal ({}s, {:.3}V amplitude, 1kHz sine)...",
            duration, amplitude
        );
    }
    let num_samples = (duration * sample_rate) as usize;
    let input_signal: Vec<f64> = (0..num_samples)
        .map(|i| {
            amplitude
                * (2.0 * std::f64::consts::PI * VALIDATE_STIMULUS_HZ * i as f64 / sample_rate).sin()
        })
        .collect();
    if verbose {
        println!("  {} samples", input_signal.len());
    }

    // Configure comparison
    let mut config = if relaxed {
        ComparisonConfig::relaxed()
    } else {
        // Audio-grade default (see ComparisonConfig::default): correlation-
        // anchored, wide enough that a good-but-complex circuit passes. The
        // old default was strict() (0.01% RMS) — tighter than every per-circuit
        // CI tolerance, so it reported FAILED on genuinely-good circuits.
        ComparisonConfig::default()
    };
    // Apply per-metric overrides on top of the base profile. Percent inputs
    // (RMS, max-rel) are converted to the fractional form the comparator uses;
    // peak (V), correlation (0–1), and THD (dB) pass through directly.
    if let Some(pct) = tol.rms_pct {
        config.rms_error_tolerance = pct / 100.0;
    }
    config.settle_time_s = settle_time_s;
    // With no --peak-tolerance, the default profile's peak bound is relative
    // to the reference (an explicit tolerance, and --relaxed, stay absolute).
    match tol.peak_v {
        Some(v) => config.peak_error_tolerance = v,
        None if !relaxed => config.peak_error_relative = Some(VALIDATE_PEAK_RELATIVE),
        None => {}
    }
    if let Some(pct) = tol.max_rel_pct {
        config.max_relative_tolerance = pct / 100.0;
    }
    if let Some(x) = tol.corr_min {
        config.correlation_min = x;
    }
    if let Some(db) = tol.thd_db {
        config.thd_error_tolerance_db = db;
    }

    let options = ValidationOptions {
        generate_html_on_failure: false,
        generate_html_on_success: false,
        generate_csv: csv_output.is_some(),
        output_dir: csv_output.and_then(|p| p.parent().map(|d| d.to_path_buf())),
        circuit_name: Some(circuit_source.name()),
        input_node: input_node.to_string(),
        bjt_fa_mode: parse_bjt_fa_mode(reductions.bjt_fa),
        tube_grid_fa: reductions.tube_grid_fa.to_string(),
        backward_euler: reductions.backward_euler,
        force_trap: reductions.force_trap,
        oversampling,
        // The test signal's closed form: the reference is driven by it.
        analytic_stimulus: Some(melange_validate::AnalyticStimulus::Sine {
            amplitude,
            frequency: VALIDATE_STIMULUS_HZ,
        }),
        // Every solver counter with -v; only the ones that need attention
        // without it.
        verbose_diagnostics: verbose,
        ..Default::default()
    };

    if rate_sweep {
        return run_rate_sweep(
            &netlist_path,
            amplitude,
            duration,
            sample_rate,
            output_node,
            &config,
            &options,
        );
    }

    // Step 4: Run validation
    if verbose {
        println!("Step 4: Running validation (ngspice + melange solver)...");
    }
    let result = validate_circuit_with_options(
        &netlist_path,
        &input_signal,
        sample_rate,
        output_node,
        &config,
        &options,
    );

    // _temp_file drops here, auto-cleaning the NamedTempFile on function exit.
    let result = result.with_context(|| "Validation failed")?;

    // Print report. The solver counters printed above it, if any, end with
    // their own blank line; the header's closes the block otherwise.
    println!("{}", result.report.summary());

    // Write CSV if requested
    if let Some(csv_path) = csv_output {
        // The validate library may have already written CSV if output_dir matched,
        // but if the user specified a specific path, write it explicitly
        if result.csv_path.as_ref() != Some(&csv_path.to_path_buf()) {
            // We need to reconstruct signals from the report info to write CSV.
            // Re-run would be expensive, so only rely on the library's CSV if it wrote one.
            if let Some(lib_csv) = &result.csv_path {
                // Move the library-generated CSV to the user-specified path (a
                // copy left a second full-size CSV beside it).
                std::fs::copy(lib_csv, csv_path)
                    .with_context(|| format!("Failed to copy CSV to {}", csv_path.display()))?;
                let _ = std::fs::remove_file(lib_csv);
                println!("CSV written to: {}", csv_path.display());
            }
        } else if result.csv_path.is_some() {
            println!("CSV written to: {}", csv_path.display());
        }
    }

    // Exit with error if validation failed.
    //
    // The unit-variation qualifier rides ON this line, both ways. The melange
    // side is built with `.tolerance`/`.mismatch` disabled so it compares
    // nominal against nominal (the ngspice deck has no other option); saying so
    // in a preamble would leave this line reading as a verdict on the unit the
    // deck describes, which it is not. Empty for a deck with no jitter
    // directive, which is every shipped validation deck.
    let qualifier = result.report.status_qualifier();
    if result.report.passed {
        println!("Validation PASSED{}", qualifier);
        Ok(())
    } else {
        // A near-perfect correlation next to a failed error gate is the
        // confusing case: the shapes agree, so the difference is in level,
        // offset or timing. Say which question to ask next.
        let shape_agrees = result.report.correlation_coefficient >= 0.999;
        let next_step = if shape_agrees {
            format!(
                "Correlation is {:.5}, so the two waveforms have the same shape: the difference \
                 is a gain, a DC offset, or a small time shift, not a different circuit. Rerun \
                 with --csv <file> and compare the spice_voltage and melange_voltage columns' \
                 peaks and means to see which. --relaxed loosens the error gates if that \
                 difference is acceptable for your use.",
                result.report.correlation_coefficient
            )
        } else {
            "Rerun with --csv <file> to see where the two engines part ways, or --relaxed for \
             looser tolerances."
                .to_string()
        };
        anyhow::bail!(
            "Validation FAILED{}: {} tolerance check(s) exceeded.\n{}",
            qualifier,
            result.report.failures.len(),
            next_step
        );
    }
}

/// `melange validate --rate-sweep`: the deck at `fs`, `2fs`, `4fs`, the
/// verdict, and the rates the fitted convergence needs for 1 % and 0.1 %.
/// PLATEAU and DIVERGES fail: melange converges to something other than the
/// reference there.
fn run_rate_sweep(
    netlist_path: &std::path::Path,
    amplitude: f64,
    duration: f64,
    sample_rate: f64,
    output_node: &str,
    config: &melange_validate::comparison::ComparisonConfig,
    options: &melange_validate::ValidationOptions,
) -> Result<()> {
    use melange_validate::rate_sweep::{
        rate_sweep, Fit, RateFor, SweepVerdict, Unresolved, EDGE_RATIO, PLATEAU_RATIO,
        RATIO_AGREEMENT,
    };
    println!(
        "Step 4: Rate sweep (oversampling off, the reference driven by the analytic sine at \
         {RATE_SWEEP_STIMULUS_HZ} Hz, incommensurate with the rates)..."
    );
    let sweep = rate_sweep(
        netlist_path,
        melange_validate::AnalyticStimulus::Sine {
            amplitude,
            frequency: RATE_SWEEP_STIMULUS_HZ,
        },
        duration,
        // validate's own settle window: 20 stimulus periods.
        20.0 / RATE_SWEEP_STIMULUS_HZ,
        sample_rate,
        output_node,
        config,
        options,
    )
    .with_context(|| "Rate sweep failed")?;
    println!(
        "  (error over every sample against the finest-rate reference; validate's own \
         per-rate number in brackets)"
    );
    for row in &sweep.rows {
        println!(
            "  {:>7.0} Hz  {:<40}  {:.4} %  [{:.4} % {}]",
            row.sample_rate,
            row.integrator,
            100.0 * row.error,
            100.0 * row.own_error,
            if row.passed { "PASS" } else { "FAIL" }
        );
    }
    println!(
        "  finest reference's self-check: {:.4} %{}",
        100.0 * sweep.reference_self_check,
        if sweep.reference_refined {
            " (refined to resolve the smallest error graded)"
        } else {
            ""
        }
    );
    if sweep.rows.len() > 3 {
        println!(
            "  (three rates were ambiguous: the fourth was added and the three finest decide)"
        );
    }
    let cross = match sweep.model_error_metric {
        Some(m) => format!("{:.4} % from the error metrics", 100.0 * m),
        None => "none from the error metrics (not monotone)".to_string(),
    };
    let rate = |tol: f64| match sweep.rate_for(tol) {
        RateFor::At(fs) => format!("{:.0} Hz", fs),
        RateFor::Unreachable => "not reachable (model error at or above it)".to_string(),
        RateFor::Unresolved => "not resolved".to_string(),
    };
    let fit_line = |order: f64, model_error: f64, fit: Fit| {
        let source = match fit {
            Fit::Waveform => format!("from the extrapolated waveform ({cross})"),
            Fit::ErrorMetric => "from the error metrics: the waveform fit has no answer, the \
                                 deck is not yet in its asymptotic range, so the order is not \
                                 the integrator's and the rates below are optimistic"
                .to_string(),
        };
        format!(
            "order {order:.2}, model error {:.4} % {source}: 1 % needs {}, 0.1 % needs {}",
            100.0 * model_error,
            rate(0.01),
            rate(0.001)
        )
    };
    let unresolved = |why: &Unresolved| match *why {
        Unresolved::ReferenceTooCoarse { self_check, error } => format!(
            "the finest reference's self-check ({:.4} %) is not below a third of the smallest \
             error it grades ({:.4} %), so how the error falls with the step is not measured",
            100.0 * self_check,
            100.0 * error
        ),
        Unresolved::EdgeDominated { grid, all } => format!(
            "edge-dominated: the finest render's error is {:.4} % over every sample but {:.4} % \
             at the instants the renders share (more than {EDGE_RATIO}x apart), so the error \
             lives between those instants and no fit is made on them. Per rate, unaligned \
             against validate's aligned figure: {}. On edges a sample or two wide the \
             alignment's fractional delay also ripples the reference (see SPICE_VALIDATION.md)",
            100.0 * all,
            100.0 * grid,
            sweep
                .rows
                .iter()
                .map(|r| format!(
                    "{:.0} Hz {:.4} % / {:.4} %",
                    r.sample_rate,
                    100.0 * r.error,
                    100.0 * r.own_error
                ))
                .collect::<Vec<_>>()
                .join(", ")
        ),
        Unresolved::PreAsymptotic { ratios } => format!(
            "pre-asymptotic: the error falls {:.2}x then {:.2}x per halving of the step (more \
             than {:.0} % apart), so the three finest rates do not describe one convergence",
            ratios[0],
            ratios[1],
            100.0 * RATIO_AGREEMENT
        ),
        Unresolved::FloorAmbiguous { model_error, error } => format!(
            "the error still falls (by at least {PLATEAU_RATIO}x per halving) but the fit puts \
             a floor of {:.4} % under a finest error of {:.4} %: the fits contradict each other",
            100.0 * model_error,
            100.0 * error
        ),
    };
    match sweep.verdict {
        SweepVerdict::Pass => {
            println!("Verdict: PASS at {:.0} Hz", sample_rate);
            match sweep.convergence {
                SweepVerdict::Converges {
                    order,
                    model_error,
                    fit,
                } => println!("  (converging: {})", fit_line(order, model_error, fit)),
                SweepVerdict::Unresolved(ref why) => println!("  ({})", unresolved(why)),
                _ => {}
            }
            Ok(())
        }
        SweepVerdict::Converges {
            order,
            model_error,
            fit,
        } => {
            println!("Verdict: CONVERGES ({})", fit_line(order, model_error, fit));
            Ok(())
        }
        SweepVerdict::Unresolved(ref why) => {
            anyhow::bail!("Verdict: UNRESOLVED: {}.", unresolved(why))
        }
        SweepVerdict::Plateau { order } => anyhow::bail!(
            "Verdict: PLATEAU (order {order:.2}; {cross}): the error has stopped falling with \
             the step (less than {PLATEAU_RATIO}x per halving at the finest rates), so melange \
             converges to something other than the reference. A model or harness mismatch."
        ),
        SweepVerdict::Diverges => anyhow::bail!(
            "Verdict: DIVERGES: the error against the finest reference rises with the rate. A \
             model or harness mismatch."
        ),
    }
}

/// Age past which a netlist left in the validate scratch dir is a killed
/// run's leftover. A live run reads its netlist within seconds of writing it.
const STALE_NETLIST_AGE: std::time::Duration = std::time::Duration::from_secs(3600);

/// Remove `melange_validate_*.cir` files older than `age` from `dir`: the
/// netlists of validate runs killed before their temp file could drop.
/// Best effort; a file another process is still using is never that old.
fn sweep_stale_netlists(dir: &std::path::Path, age: std::time::Duration) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    let now = std::time::SystemTime::now();
    for entry in entries.flatten() {
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if !(name.starts_with("melange_validate_") && name.ends_with(".cir")) {
            continue;
        }
        let stale = entry
            .metadata()
            .and_then(|m| m.modified())
            .ok()
            .and_then(|t| now.duration_since(t).ok())
            .is_some_and(|d| d > age);
        if stale {
            let _ = std::fs::remove_file(entry.path());
        }
    }
}

#[cfg(test)]
mod sweep_tests {
    use super::sweep_stale_netlists;
    use std::time::{Duration, SystemTime};

    #[test]
    fn removes_only_old_validate_netlists() {
        let dir = tempfile::tempdir().unwrap();
        let put = |name: &str, age_s: u64| {
            let p = dir.path().join(name);
            std::fs::write(&p, "x").unwrap();
            let f = std::fs::File::options().write(true).open(&p).unwrap();
            f.set_modified(SystemTime::now() - Duration::from_secs(age_s))
                .unwrap();
            p
        };
        let old = put("melange_validate_a.cir", 7200);
        let young = put("melange_validate_b.cir", 10);
        let foreign = put("other.cir", 7200);
        sweep_stale_netlists(dir.path(), Duration::from_secs(3600));
        assert!(!old.exists());
        assert!(young.exists());
        assert!(foreign.exists());
    }
}
