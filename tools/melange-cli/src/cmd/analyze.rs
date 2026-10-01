use crate::common::{
    build_error_in, declares_state_field, diag_lit_factor, dk_unsolved_remedy, has_opamps,
    is_build_detail, load_circuit_text, print_run_route_detail, refuse_on_input_diag,
    report_build_line, resolve_switch_overrides, route_summary, unconverged_commit_source,
    INPUT_DIAG_FIELDS,
};
use crate::{circuits, codegen_runner};
use anyhow::{Context, Result};
use std::path::PathBuf;

/// Options bundle for `melange analyze` — mirrors [`SimulateOptions`](crate::cmd::simulate::SimulateOptions).
pub(crate) struct AnalyzeOptions<'a> {
    pub(crate) input_node: &'a str,
    pub(crate) output_node: &'a str,
    pub(crate) start_freq: f64,
    pub(crate) end_freq: f64,
    pub(crate) points_per_decade: usize,
    /// `--freq`: measure this one frequency instead of the log sweep.
    pub(crate) single_freq: Option<f64>,
    pub(crate) amplitude: f64,
    pub(crate) sample_rate: f64,
    pub(crate) input_resistance_flag: Option<f64>,
    pub(crate) output_file: Option<&'a PathBuf>,
    pub(crate) pot_overrides: &'a [String],
    pub(crate) switch_overrides: &'a [String],
    pub(crate) harmonics: usize,
    pub(crate) tube_grid_fa: &'a str,
    pub(crate) solver: &'a str,
    pub(crate) oversampling: Option<usize>,
    pub(crate) opamp_rail_mode: melange_solver::codegen::OpampRailMode,
    pub(crate) noise_mode: melange_solver::codegen::NoiseMode,
    pub(crate) noise_seed: u64,
    /// `--allow-unconverged-dc-op`: build even when the DC operating point did
    /// not converge.
    pub(crate) allow_unconverged_dc_op: bool,
    /// Test-only DC-OP Newton budget (hidden `--dc-op-max-iterations`).
    pub(crate) dc_op_max_iterations: Option<usize>,
    pub(crate) backward_euler: bool,
    pub(crate) force_trap: bool,
    /// Nodal sub-path override (`--nodal-subpath`).
    pub(crate) nodal_sub_path_override: melange_solver::codegen::NodalSubPathOverride,
    /// Explicit `--max-iter` override; `None` → auto-tuned (see [`auto_tune_max_iter`]).
    pub(crate) max_iter: Option<usize>,
    /// `--allow-input-clamp`: report even when the input was clamped or NaN.
    pub(crate) allow_input_clamp: bool,
    /// `--allow-nr-hold`: report even when a point's render was not a
    /// solution (held, unconverged-committed, or reduced-model-exit samples).
    pub(crate) allow_nr_hold: bool,
    /// `--preroll-secs`: minimum drive-level pre-roll per point, and the
    /// spacing between its settle-check measurements.
    pub(crate) preroll_secs: f64,
    /// `--preroll-max-secs`: cap on drive-level time per point; 0 = no
    /// settle check.
    pub(crate) preroll_max_secs: f64,
    /// `-v/--verbose`: print the routing detail.
    pub(crate) verbose: bool,
}

// `LinearizeOutcome`, `apply_linearize_reductions` and `auto_tune_max_iter`
// moved to `melange_solver::pipeline` so `melange compile`, `simulate`,
// `analyze` and `melange-validate` share one front-end pipeline instead of
// four copies that drifted. See that module's docs for what the drift cost.
pub(crate) fn analyze_freq_response(
    circuit_source: &circuits::CircuitSource,
    opts: &AnalyzeOptions<'_>,
) -> Result<()> {
    let AnalyzeOptions {
        nodal_sub_path_override,
        input_node: input_node_name,
        output_node: output_node_name,
        start_freq,
        end_freq,
        points_per_decade,
        single_freq,
        amplitude,
        sample_rate,
        input_resistance_flag,
        output_file,
        pot_overrides,
        switch_overrides,
        harmonics,
        tube_grid_fa,
        solver,
        oversampling: oversampling_cli,
        opamp_rail_mode,
        noise_mode,
        noise_seed,
        allow_unconverged_dc_op,
        dc_op_max_iterations,
        backward_euler,
        force_trap,
        max_iter,
        allow_input_clamp,
        allow_nr_hold,
        preroll_secs,
        preroll_max_secs,
        verbose,
    } = *opts;
    // Match parse-time node normalization (lowercase, gnd→0).
    let input_node_owned = melange_solver::parser::normalize_node_name(input_node_name);
    let input_node_name = input_node_owned.as_str();
    let output_node_owned = melange_solver::parser::normalize_node_name(output_node_name);
    let output_node_name = output_node_owned.as_str();

    eprintln!("melange analyze (frequency response)");

    // Analyze writes its CSV to stdout, so the loader's lines go to stderr.
    let netlist_str = load_circuit_text(circuit_source, &|l| {
        if verbose || !is_build_detail(l) {
            eprintln!("{l}")
        }
    })?;

    // The one build every verb ships (melange_solver::build). Analyze writes
    // its CSV to stdout, so every build line goes to stderr.
    let build_opts = melange_solver::build::BuildOptions {
        sample_rate,
        circuit_name: "analyze".to_string(),
        input_nodes: vec![input_node_name.to_string()],
        output_nodes: vec![output_node_name.to_string()],
        max_iter,
        tolerance: 1e-9,
        output_scale: 1.0,
        output_clamp: 10.0,
        input_resistance: input_resistance_flag,
        oversampling: oversampling_cli,
        dc_block: false,
        solver: solver.to_string(),
        backward_euler,
        force_trap,
        tube_grid_fa: tube_grid_fa.to_string(),
        // `analyze` does not expose --subsample-fire; auto = active on glow
        // nodal-Schur decks, inert everywhere else.
        subsample_fire: melange_solver::codegen::SubsampleFireMode::Auto,
        subsample_lit_factor: diag_lit_factor(),
        bjt_fa_mode: melange_solver::codegen::BjtFaMode::Off,
        opamp_rail_mode,
        nodal_sub_path_override,
        allow_static_glow_on_full_lu: false,
        noise_mode,
        noise_seed,
        emit_dc_op_recompute: false,
        plugin_format: false,
        // One knob setting means the same thing here as in `simulate`.
        pot_overrides: Some(pot_overrides.to_vec()),
        resolve_taps: false,
        inject_runtime: false,
        disable_unit_variation: false,
        disable_self_heating: false,
        allow_unconverged_dc_op,
        dc_op_max_iterations,
        output_clamp_auto: false,
    };
    let built = melange_solver::build::build(
        &netlist_str,
        &build_opts,
        &|a| report_build_line(verbose, a, |l| eprintln!("{l}")),
        &|a| eprintln!("{a}"),
    )
    .map_err(|e| build_error_in(e, &netlist_str))?;
    let route_is_dk = built.solver_label == "DK";
    let circuit_has_opamps = has_opamps(&built.netlist);
    let summary = route_summary(
        built.solver_label,
        solver,
        built.generated.meta.nodal_sub_path,
        built.generated.meta.integrator_selection,
    );
    if verbose {
        // The one-line summary, then the detail behind it.
        eprintln!("  {}", summary.trim_end_matches(" (-v for why)"));
        print_run_route_detail(&built, max_iter.is_some(), &|l| eprintln!("{l}"));
    } else {
        eprintln!("  {summary}");
    }
    let has_inductors = !built.mna.inductors.is_empty()
        || !built.mna.coupled_inductors.is_empty()
        || !built.mna.transformer_groups.is_empty();
    let generated = built.generated;
    let netlist = built.netlist;
    let oversampling = built.oversampling;

    // Keep the netlist at position-0 element values and apply overrides at
    // runtime via `state.set_switch_N(position)` (see `switch_calls` below).
    let switch_runtime_overrides = resolve_switch_overrides(&netlist, switch_overrides)?;

    // Generate frequency list
    let frequencies = match single_freq {
        Some(f) => {
            eprintln!("  1 frequency point: {f} Hz, at a {sample_rate} Hz sample rate");
            vec![f]
        }
        None => {
            let frequencies = generate_log_frequencies(start_freq, end_freq, points_per_decade);
            eprintln!(
                "  {} frequency points from {:.0} Hz to {:.0} Hz, at a {} Hz sample rate",
                frequencies.len(),
                start_freq,
                end_freq,
                sample_rate
            );
            frequencies
        }
    };
    // A constant delay is a phase slope, and the half-band filters add one:
    // tens of degrees by a few kHz. Someone comparing this sweep against a 1×
    // one sees phase_deg move a long way and reasonably concludes the circuit's
    // response changed. It did not — gain is unaffected. Said where the number
    // is produced, not only in a doc they may not have read.
    if oversampling > 1 {
        let added = if oversampling >= 4 { 3.47 } else { 2.65 };
        eprintln!(
            "  NOTE: at {}× the phase column includes ~{:.2} host samples of half-band",
            oversampling, added
        );
        eprintln!(
            "        filter delay (~{:.0}° at 3 kHz). Gain is unaffected; the circuit's",
            360.0 * 3000.0 * added / sample_rate
        );
        eprintln!("        response has not changed. See docs/OVERSAMPLING.md.");
    }

    // Zero-drive settle, once, before the first point: moves the bias off the
    // embedded DC operating point (and lets inductor currents settle) before
    // any drive. It does NOT make a point steady state — each point's own
    // drive-level pre-roll below does that.
    let settle_secs = if has_inductors { 5.0 } else { 0.5 };

    // `--noise` makes successive windows differ by the noise itself, so the
    // agreement check would chase it to the cap on every point. Keep the
    // fixed pre-roll, drop the check, and say so.
    let preroll_max_secs =
        if noise_mode != melange_solver::codegen::NoiseMode::Off && preroll_max_secs > 0.0 {
            eprintln!(
                "  NOTE: --noise is on, so the per-point settle check is off (noisy windows never \
             agree); each point gets the fixed {preroll_secs} s pre-roll only."
            );
            0.0
        } else {
            preroll_max_secs
        };
    // The settle method is `-v` detail while the check runs (the summary after
    // the sweep says whether every point settled); with the check off, say so,
    // since nothing after the sweep will.
    if preroll_max_secs > 0.0 {
        if verbose {
            eprintln!(
                "  Each point: driven at its own frequency and amplitude for >= {preroll_secs} s, \
                 then re-measured every {preroll_secs} s until two measurements agree to {:.1} % \
                 (or {preroll_max_secs} s at drive).",
                codegen_runner::ANALYZE_SETTLE_TOL * 100.0
            );
        }
    } else {
        eprintln!(
            "  Each point: driven at its own frequency and amplitude for >= {preroll_secs} s \
             before it is measured (settle check off)."
        );
    }
    if harmonics > 0 {
        eprintln!(
            "  thd_pct band: H2..H{harmonics} below {:.0} kHz (and below Nyquist); hN_dbc \
             columns are reported up to Nyquist.",
            codegen_runner::ANALYZE_THD_BAND_HZ / 1000.0
        );
    }

    // Append analyze main, compile, run.
    //
    // Switch overrides are applied at runtime via `state.set_switch_N(position)`
    // before the settle loop; the settle loop then converges NR (if M>0) on
    // the new topology. Pot overrides remain netlist-baked (R-only, flows
    // into G at codegen time).
    let switch_calls: Vec<String> = switch_runtime_overrides
        .iter()
        .map(|(idx, pos)| format!("state.set_switch_{idx}({pos})"))
        .collect();
    let declared = |fields: &[&'static str]| -> Vec<&'static str> {
        fields
            .iter()
            .copied()
            .filter(|f| declares_state_field(&generated.code, f))
            .collect()
    };
    let diag_counters = declared(&INPUT_DIAG_FIELDS);
    let point_counters = declared(&POINT_DIAG_FIELDS);
    let analyze_main = codegen_runner::generate_analyze_main(codegen_runner::AnalyzeMain {
        frequencies: &frequencies,
        amplitude,
        sample_rate,
        settle_secs,
        preroll_secs,
        preroll_max_secs,
        pot_calls: &[], // already baked into netlist
        switch_calls: &switch_calls,
        harmonics,
        noise_enabled: noise_mode != melange_solver::codegen::NoiseMode::Off,
        diag_counters: &diag_counters,
        point_counters: &point_counters,
    });
    let full_source = format!("{}\n{}", generated.code, analyze_main);

    let binary_cache =
        codegen_runner::BinaryCache::new().with_context(|| "Failed to create binary cache")?;
    let compiled = binary_cache
        .compile(&full_source, "analyze")
        .with_context(|| "Compilation failed")?;
    if compiled.cached && verbose {
        eprintln!("  Using cached binary");
    }

    let result = std::process::Command::new(&compiled.path)
        .output()
        .with_context(|| "Failed to run analyze binary")?;

    if !result.status.success() {
        let stderr = String::from_utf8_lossy(&result.stderr);
        anyhow::bail!("Analyze binary failed:\n{}", stderr);
    }

    let stdout = String::from_utf8_lossy(&result.stdout);

    // Print per-frequency lines from stderr; the machine lines (`DIAGPT:`,
    // `SETTLEPT:`) are read below and reported in words instead, and the
    // end-of-run `DIAG:` counters are `-v` detail (a nonzero input counter is
    // refused below in words).
    let stderr = String::from_utf8_lossy(&result.stderr);
    for line in stderr.lines() {
        if line.starts_with("DIAGPT:") || line.starts_with("SETTLEPT:") {
            continue;
        }
        if !verbose && is_numeric_diag_line(line) {
            continue;
        }
        eprintln!("{}", line);
    }
    report_settle(&stderr, preroll_max_secs);
    refuse_on_unsolved_points(
        &stderr,
        &PointRun {
            sample_rate,
            oversampling,
            settle_secs,
            route_is_dk,
            has_opamps: circuit_has_opamps,
        },
        allow_nr_hold,
    )?;
    refuse_on_input_diag(&stderr, allow_input_clamp)?;

    // Output CSV.
    //
    // The two streams are already separated: every progress/diagnostic line
    // above is stderr, and stdout carries nothing but the CSV, so
    // `melange analyze ... > response.csv` yields a clean file with no
    // hand-stripping. On a terminal both land in the same scrollback and it
    // looks like one interleaved dump, which is what makes people reach for a
    // separator flag — so say where the CSV went instead.
    if let Some(out_path) = output_file {
        std::fs::write(out_path, stdout.as_bytes())
            .with_context(|| format!("Failed to write: {}", out_path.display()))?;
        eprintln!("  Results written to: {}", out_path.display());
    } else {
        eprintln!(
            "  {} CSV rows follow on stdout (everything above is stderr). \
             Capture them with `-o FILE`, or redirect: `melange analyze ... > response.csv`.",
            stdout.lines().count().saturating_sub(1)
        );
        print!("{}", stdout);
    }

    Ok(())
}

/// A `DIAG:<key>=<number>` line (a counter, as opposed to a `DIAG:note=` text).
fn is_numeric_diag_line(line: &str) -> bool {
    line.strip_prefix("DIAG:")
        .and_then(|d| d.split_once('='))
        .is_some_and(|(_, v)| v.trim().parse::<f64>().is_ok())
}

fn generate_log_frequencies(start: f64, end: f64, points_per_decade: usize) -> Vec<f64> {
    let log_start = start.log10();
    let log_end = end.log10();
    let decades = log_end - log_start;
    let total_points = (decades * points_per_decade as f64).ceil() as usize + 1;
    (0..total_points)
        .map(|i| {
            let log_f = log_start + i as f64 * decades / (total_points - 1).max(1) as f64;
            10.0_f64.powf(log_f)
        })
        .collect()
}

/// `CircuitState` counters whose per-point increments `analyze` reads
/// (presence-filtered per build): the unsolved-sample family `simulate`
/// refuses on, plus the NaN/magnitude resets it calls suspect.
const POINT_DIAG_FIELDS: [&str; 6] = [
    "diag_unsolved_sample_count",
    "diag_nr_hold_count",
    "diag_nr_unconverged_commit_count",
    "diag_reduced_model_exit_count",
    "diag_nan_reset_count",
    "diag_magnitude_reset_count",
];

/// One `SETTLEPT:` line: a point's drive-level time and settle verdict.
#[derive(Debug, PartialEq)]
struct SettlePoint {
    label: String,
    drive_secs: f64,
    /// 1 = settled, 0 = cap reached, 2 = unchecked.
    status: u8,
    fund_rel: f64,
    harm_rel: f64,
}

fn parse_settle_points(stderr: &str) -> Vec<SettlePoint> {
    stderr
        .lines()
        .filter_map(|l| l.strip_prefix("SETTLEPT:"))
        .filter_map(|rest| {
            let f: Vec<&str> = rest.split(':').collect();
            if f.len() != 5 {
                return None;
            }
            Some(SettlePoint {
                label: f[0].to_string(),
                drive_secs: f[1].parse().ok()?,
                status: f[2].parse().ok()?,
                fund_rel: f[3].parse().unwrap_or(f64::NAN),
                harm_rel: f[4].parse().unwrap_or(f64::NAN),
            })
        })
        .collect()
}

/// `DIAGPT:<label>:<counter>=<delta>` lines grouped by label, in the order
/// the labels first appear (`settle`, then each point's frequency).
fn parse_point_counts(stderr: &str) -> Vec<(String, Vec<(String, u64)>)> {
    let mut out: Vec<(String, Vec<(String, u64)>)> = Vec::new();
    for rest in stderr.lines().filter_map(|l| l.strip_prefix("DIAGPT:")) {
        let Some((label, kv)) = rest.split_once(':') else {
            continue;
        };
        let Some((name, v)) = kv.split_once('=') else {
            continue;
        };
        let Ok(v) = v.trim().parse::<u64>() else {
            continue;
        };
        match out.iter_mut().find(|(l, _)| l == label) {
            Some((_, counts)) => counts.push((name.to_string(), v)),
            None => out.push((label.to_string(), vec![(name.to_string(), v)])),
        }
    }
    out
}

/// The one-line steady-state verdict: whether every point settled before the
/// `--preroll-max-secs` cap, and if not, which points reached the cap still
/// moving. A point that settled on the last check the cap allows says so, so a
/// drive time equal to the cap never reads as either outcome.
fn settle_summary(points: &[SettlePoint], preroll_max_secs: f64) -> String {
    let unsettled: Vec<&str> = points
        .iter()
        .filter(|p| p.status == 0)
        .map(|p| p.label.as_str())
        .collect();
    let cap = format!("the --preroll-max-secs cap of {preroll_max_secs} s");
    if !unsettled.is_empty() {
        return format!(
            "Steady state: {}/{} point(s) settled before {cap}; {} reached the cap still \
             moving: {} Hz (see the WARNING below).",
            points.len() - unsettled.len(),
            points.len(),
            unsettled.len(),
            unsettled.join(" Hz, ")
        );
    }
    let slowest = points
        .iter()
        .max_by(|a, b| a.drive_secs.total_cmp(&b.drive_secs))
        .expect("non-empty");
    let at_cap = slowest.drive_secs >= preroll_max_secs * (1.0 - 1e-9);
    let when = if at_cap {
        format!(
            "settled on the last check the cap allows ({:.2} s at drive)",
            slowest.drive_secs
        )
    } else {
        format!("settled after {:.2} s at drive", slowest.drive_secs)
    };
    if points.len() == 1 {
        return format!(
            "Steady state: the {} Hz point {when}, within {cap}.",
            slowest.label
        );
    }
    format!(
        "Steady state: all {} points settled within {cap}; the slowest, {} Hz, {when}.",
        points.len(),
        slowest.label
    )
}

/// Report points that did not reach steady state within the cap. A warning,
/// not a refusal: the samples are solutions, and some circuits have no steady
/// state at a given drive at all (an oscillator, a limit cycle).
fn report_settle(stderr: &str, preroll_max_secs: f64) {
    let points = parse_settle_points(stderr);
    if points.is_empty() || preroll_max_secs <= 0.0 {
        return;
    }
    let unsettled: Vec<&SettlePoint> = points.iter().filter(|p| p.status == 0).collect();
    eprintln!("  {}", settle_summary(&points, preroll_max_secs));
    if unsettled.is_empty() {
        return;
    }
    eprintln!();
    eprintln!(
        "WARNING: {} point(s) did not reach steady state within --preroll-max-secs \
         ({preroll_max_secs} s at drive). Their rows report the last measurement, which is \
         still moving:",
        unsettled.len()
    );
    for p in &unsettled {
        eprintln!(
            "  {} Hz: the last two measurements differ by {:.3} % (fundamental) and {:.3} % \
             (harmonics)",
            p.label,
            p.fund_rel * 100.0,
            p.harm_rel * 100.0
        );
    }
    eprintln!(
        "  Raise --preroll-max-secs if the circuit is still settling, or the circuit has no \
         steady state at this drive (an oscillator, a limit cycle, a subharmonic response)."
    );
}

/// Refuse any point whose render was not a solution, the way `simulate`
/// refuses a render: held samples, unconverged commits, and samples solved on
/// a reduced device model outside its region (`unsolved_sample_count`
/// covers all three where the build declares it). `allow` is
/// `--allow-nr-hold`. NaN/magnitude resets are reported as suspect, as
/// `simulate` reports them.
/// What [`refuse_on_unsolved_points`] needs to know about the run and build.
struct PointRun {
    sample_rate: f64,
    oversampling: usize,
    /// The zero-drive settle before the first point (seconds).
    settle_secs: f64,
    /// The build is on the DK route (its remedy for unsolved samples is the
    /// nodal route).
    route_is_dk: bool,
    /// The circuit has op-amps (an unconverged commit can be a rail pin).
    has_opamps: bool,
}

fn refuse_on_unsolved_points(stderr: &str, run: &PointRun, allow: bool) -> Result<()> {
    let PointRun {
        sample_rate,
        oversampling,
        settle_secs,
        route_is_dk,
        has_opamps,
    } = *run;
    let points = parse_point_counts(stderr);
    let settle = parse_settle_points(stderr);
    // Internal solves a label's counters were counted over.
    let solves = |label: &str| -> Option<u64> {
        let secs = if label == "settle" {
            Some(settle_secs)
        } else {
            settle
                .iter()
                .find(|p| p.label == label)
                .map(|p| p.drive_secs)
        }?;
        Some((secs * sample_rate).round() as u64 * oversampling as u64)
    };
    let get = |counts: &[(String, u64)], key: &str| -> Option<u64> {
        counts.iter().find(|(n, _)| n == key).map(|(_, v)| *v)
    };
    let where_ = |label: &str| -> String {
        if label == "settle" {
            "the zero-drive settle before the first point".to_string()
        } else {
            format!("{label} Hz")
        }
    };
    let mut bad: Vec<(String, String, String)> = Vec::new();
    let (mut any_reduced, mut any_held, mut any_committed) = (false, false, false);
    for (label, counts) in &points {
        let resets = get(counts, "nan_reset_count").unwrap_or(0)
            + get(counts, "magnitude_reset_count").unwrap_or(0);
        if resets > 0 {
            eprintln!(
                "WARNING: {}: {resets} NaN/magnitude reset(s): the solve blew up and was reset. \
                 Treat this point as suspect.",
                where_(label)
            );
        }
        let held = get(counts, "nr_hold_count").unwrap_or(0);
        let committed = get(counts, "nr_unconverged_commit_count").unwrap_or(0);
        let reduced = get(counts, "reduced_model_exit_count").unwrap_or(0);
        let total = get(counts, "unsolved_sample_count").unwrap_or(held + committed);
        if total == 0 {
            continue;
        }
        any_reduced |= reduced > 0;
        any_held |= held > 0;
        any_committed |= committed > 0;
        let detail = counts
            .iter()
            .filter(|(n, _)| !n.ends_with("reset_count"))
            .map(|(n, v)| format!("{n} {v}"))
            .collect::<Vec<_>>()
            .join(", ");
        let of = solves(label)
            .map(|n| format!(" of {n}"))
            .unwrap_or_default();
        bad.push((where_(label), format!("{total}{of}"), detail));
    }
    if bad.is_empty() {
        return Ok(());
    }
    let os_note = if oversampling > 1 {
        format!(" (counted per internal solve, {oversampling} per host sample)")
    } else {
        String::new()
    };
    eprintln!();
    eprintln!(
        "ERROR: {} point(s) measured samples that are not a solution to this circuit{os_note}:",
        bad.len()
    );
    for (at, total, detail) in &bad {
        eprintln!("  {at}: {total} sample(s) unsolved [{detail}]");
    }
    if any_reduced {
        eprintln!(
            "  reduced_model_exit_count: samples solved on a REDUCED device model outside its \
             region: a `.linearize`d device driven out of its small-signal region (a triode cut \
             off or its grid past the conduction onset, a BJT cut off or saturated), or a \
             grid-off pentode (--tube-grid-fa on) whose grid conducted. Remove `.linearize` for \
             a stage that leaves its region at this drive, or lower --amplitude."
        );
    }
    if any_held {
        eprintln!(
            "  nr_hold_count: every Newton path failed and the solver committed the PREVIOUS \
             state as the output."
        );
    }
    if any_committed {
        eprintln!(
            "  nr_unconverged_commit_count: the final Newton solve ({}) ended unconverged and \
             that iterate was committed.",
            unconverged_commit_source(route_is_dk, has_opamps)
        );
    }
    eprintln!(
        "  A gain or THD read from those samples describes the solver's fallback, not the \
         circuit; the waveform stays bounded and smooth, so nothing in the numbers shows it. \
         `melange simulate` refuses the same render."
    );
    let dk_remedy = if any_committed {
        dk_unsolved_remedy(route_is_dk)
    } else {
        None
    };
    if let Some(remedy) = dk_remedy {
        eprintln!("  {remedy}");
    }
    if !allow {
        if dk_remedy.is_some() {
            anyhow::bail!(
                "{} analyze point(s) were not a solution on the DK route (try --solver nodal; \
                 --allow-nr-hold to report them anyway)",
                bad.len()
            );
        }
        anyhow::bail!(
            "{} analyze point(s) were not a solution (--allow-nr-hold to report them anyway)",
            bad.len()
        );
    }
    eprintln!("(--allow-nr-hold given: reporting them anyway.)");
    Ok(())
}

#[cfg(test)]
mod point_diag_tests {
    use super::*;

    const STDERR: &str = "\
  1000.0 Hz: 12.88 dB, 13.4°, THD=5.972%
DIAGPT:settle:nr_hold_count=0
SETTLEPT:100.00:0.500000:1:2.1e-5:3.4e-4
DIAGPT:1002.09:reduced_model_exit_count=6497
DIAGPT:1002.09:unsolved_sample_count=6497
SETTLEPT:1002.09:2.000000:0:1.5e-2:NaN
DIAGPT:2000.00:nan_reset_count=1
";

    const RUN: PointRun = PointRun {
        sample_rate: 48000.0,
        oversampling: 1,
        settle_secs: 0.5,
        route_is_dk: false,
        has_opamps: false,
    };

    /// An unconverged commit on the DK route is refused with the nodal route
    /// as the first remedy; off DK the refusal does not suggest it.
    #[test]
    fn a_dk_unconverged_commit_points_at_the_nodal_route() {
        let stderr = "DIAGPT:1000.00:nr_unconverged_commit_count=4\n\
                      DIAGPT:1000.00:unsolved_sample_count=4\n";
        let dk = PointRun {
            route_is_dk: true,
            ..RUN
        };
        let err = refuse_on_unsolved_points(stderr, &dk, false).unwrap_err();
        assert!(err.to_string().contains("--solver nodal"), "{err}");
        assert!(err.to_string().contains("--allow-nr-hold"), "{err}");
        let err = refuse_on_unsolved_points(stderr, &RUN, false).unwrap_err();
        assert!(!err.to_string().contains("--solver nodal"), "{err}");
    }

    #[test]
    fn settle_summary_says_whether_every_point_beat_the_cap() {
        let p = |label: &str, drive_secs: f64, status: u8| SettlePoint {
            label: label.to_string(),
            drive_secs,
            status,
            fund_rel: 0.0,
            harm_rel: 0.0,
        };
        let all = settle_summary(&[p("20.00", 1.5, 1), p("200.00", 0.5, 1)], 2.0);
        assert!(all.contains("all 2 points settled within"), "{all}");
        assert!(all.contains("20.00 Hz, settled after 1.50 s"), "{all}");
        // Settled on the final check: drive time == cap, and it says so.
        let edge = settle_summary(&[p("20.00", 2.0, 1)], 2.0);
        assert!(edge.contains("last check the cap allows (2.00 s"), "{edge}");
        assert!(edge.contains("the 20.00 Hz point settled"), "{edge}");
        // Hit the cap: named, and not called settled.
        let hit = settle_summary(
            &[p("20.00", 2.0, 0), p("30.00", 2.0, 0), p("1000.00", 0.5, 1)],
            2.0,
        );
        assert!(hit.contains("1/3 point(s) settled"), "{hit}");
        assert!(
            hit.contains("2 reached the cap still moving: 20.00 Hz, 30.00 Hz"),
            "{hit}"
        );
    }

    #[test]
    fn settle_lines_parse() {
        let p = parse_settle_points(STDERR);
        assert_eq!(p.len(), 2);
        assert_eq!(p[0].label, "100.00");
        assert_eq!(p[0].status, 1);
        assert_eq!(p[1].status, 0);
        assert!((p[1].drive_secs - 2.0).abs() < 1e-12);
        assert!(p[1].harm_rel.is_nan());
    }

    #[test]
    fn point_counts_group_by_label_in_order() {
        let c = parse_point_counts(STDERR);
        let labels: Vec<&str> = c.iter().map(|(l, _)| l.as_str()).collect();
        assert_eq!(labels, ["settle", "1002.09", "2000.00"]);
        assert_eq!(c[1].1.len(), 2);
    }

    #[test]
    fn an_unsolved_point_is_refused_unless_allowed() {
        let err = refuse_on_unsolved_points(STDERR, &RUN, false).unwrap_err();
        assert!(err.to_string().contains("--allow-nr-hold"), "{err}");
        refuse_on_unsolved_points(STDERR, &RUN, true).unwrap();
    }

    #[test]
    fn resets_alone_warn_but_do_not_refuse() {
        refuse_on_unsolved_points("DIAGPT:50.00:nan_reset_count=3\n", &RUN, false).unwrap();
    }

    /// A build without the unified counter still refuses on its parts.
    #[test]
    fn hold_without_the_unified_counter_is_refused() {
        assert!(refuse_on_unsolved_points("DIAGPT:50.00:nr_hold_count=2\n", &RUN, false).is_err());
    }
}
