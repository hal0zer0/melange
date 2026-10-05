use crate::args::utf8_path_arg;
use crate::common::{
    build_error_in, declares_state_field, diag_lit_factor, dk_unsolved_remedy, has_opamps,
    is_build_detail, load_circuit_text, print_run_route_detail, refuse_on_input_diag,
    report_build_line, resolve_switch_overrides, route_summary, unconverged_commit_source,
    INPUT_DIAG_FIELDS,
};
use crate::{circuits, codegen_runner};
use anyhow::{Context, Result};
use std::path::PathBuf;

pub(crate) struct SimulateOptions<'a> {
    pub(crate) input_audio: Option<&'a std::path::Path>,
    pub(crate) output: &'a PathBuf,
    pub(crate) sample_rate: f64,
    /// Where `sample_rate` came from (see [`resolve_simulate_sample_rate`](crate::args::resolve_simulate_sample_rate));
    /// printed under `-v`.
    pub(crate) sample_rate_source: &'static str,
    pub(crate) input_node: &'a str,
    pub(crate) output_node: &'a str,
    pub(crate) duration: f64,
    pub(crate) amplitude: f64,
    pub(crate) input_resistance_flag: Option<f64>,
    pub(crate) solver: &'a str,
    pub(crate) opamp_rail_mode: melange_solver::codegen::OpampRailMode,
    pub(crate) tube_grid_fa: &'a str,
    /// `--subsample-fire` mode (glow-strike variable-dt re-solve).
    pub(crate) subsample_fire: melange_solver::codegen::SubsampleFireMode,
    pub(crate) oversampling: Option<usize>,
    pub(crate) noise_mode: melange_solver::codegen::NoiseMode,
    pub(crate) noise_seed: u64,
    /// `--allow-unconverged-dc-op`: build even when the DC operating point did
    /// not converge.
    pub(crate) allow_unconverged_dc_op: bool,
    /// Test-only DC-OP Newton budget (hidden `--dc-op-max-iterations`).
    pub(crate) dc_op_max_iterations: Option<usize>,
    pub(crate) backward_euler: bool,
    pub(crate) force_trap: bool,
    /// Nodal sub-path override (`--nodal-subpath`). `Auto` for `simulate`,
    /// which does not expose the flag.
    pub(crate) nodal_sub_path_override: melange_solver::codegen::NodalSubPathOverride,
    /// Explicit `--max-iter` override; `None` → auto-tuned (see [`auto_tune_max_iter`]).
    pub(crate) max_iter: Option<usize>,
    /// `--allow-nr-hold`: render even when samples were never solved.
    pub(crate) allow_nr_hold: bool,
    /// `--allow-input-clamp`: render even when the input was clamped or NaN.
    pub(crate) allow_input_clamp: bool,
    pub(crate) probes: &'a [String],
    pub(crate) probe_csv: Option<&'a std::path::Path>,
    /// `--pcm16`: write the output WAV as 16-bit PCM instead of float32.
    /// File format only — the rendered samples and every reported figure are
    /// computed in f64 before encoding.
    pub(crate) pcm16: bool,
    /// `--pot NAME=VALUE` specs; baked into the netlist's resistor values
    /// before the MNA is built (same path `analyze` uses).
    pub(crate) pot_overrides: &'a [String],
    /// `--switch NAME=POS` specs; resolved and applied at runtime via
    /// `state.set_switch_N(pos)` before the run (mirrors the plugin).
    pub(crate) switch_overrides: &'a [String],
    /// `--inject FIELD=SPEC` specs; drive `.inject` fields from the CLI.
    pub(crate) inject_drives: &'a [String],
    /// `-v/--verbose`: print the routing detail.
    pub(crate) verbose: bool,
}

/// `CircuitState` diagnostic counters emitted only when sub-sample fire is
/// active; `simulate` prints them when the generated code declares them.
const SUBSAMPLE_FIRE_DIAG_FIELDS: [&str; 10] = [
    "diag_subsample_fire_count",
    "diag_subsample_fire_abandon_count",
    "diag_subsample_fire_detected",
    "diag_subsample_fire_resolved",
    // Reason-split of `detected - resolved` (must sum to it): a real miss is
    // ceiling+coincident; gridpoint is a bounded (<=1e-3·dt) extinction-timing
    // quantisation, expected to scale with the lit sub-step count.
    "diag_subsample_fire_unresolved_ceiling",
    "diag_subsample_fire_unresolved_gridpoint",
    "diag_subsample_fire_unresolved_coincident",
    "diag_subsample_fire_segments",
    // Memo effectiveness (a consumer's runtime check that the memo is active).
    "diag_subsample_fire_schur_builds",
    "diag_subsample_fire_schur_reuses",
];

/// Output peak, in volts, below which `simulate` calls a rendered WAV silent.
///
/// Why a threshold rather than `peak == 0.0`: an output that lands on 1e-300 V
/// is exactly as broken as one that lands on 0.0 — the float merely happened to
/// underflow to a denormal instead of to zero — and an equality test would wave
/// it through. A floated stage, a dead bias point and a mistyped output node all
/// produce "some absurdly small number", not reliably a hard zero.
///
/// Why 1 uV specifically:
///   * melange writes volts into the WAV (1.0 == 1.0 V), so 1e-6 V is -120 dBFS.
///     Every real audio circuit's own thermal noise floor is orders of magnitude
///     above that, and no converter can reproduce it — there is no legitimately
///     quiet circuit sitting in this band, only broken ones.
///   * The generated binary reports `DIAG:peak` in scientific notation
///     (`{:.6e}`), so the real figure reaches this check however small it is —
///     the old `{:.6}` fixed format flattened everything under 5e-7 V to
///     `0.000000`, which made the threshold unmeasurable from here and the
///     warning unable to quote anything but zero.
///
/// A circuit that is quiet but working (a deep attenuator, a fader near the
/// bottom, a stage biased into cutoff) still lands far above this, so the
/// warning does not nag correct circuits. Genuinely intentional silence — a
/// zero-amplitude drive — is filtered out at the call site instead.
const SILENT_OUTPUT_PEAK_V: f64 = 1e-6;

/// True when a rendered output peak counts as digital silence.
///
/// NaN is deliberately NOT silence: a NaN peak means the solve blew up, which
/// the NaN/magnitude-reset counters report, and it must not be described to the
/// user as "nothing to hear".
fn output_is_silent(peak: f64) -> bool {
    peak.is_finite() && peak.abs() < SILENT_OUTPUT_PEAK_V
}

/// Build the warning shown when `simulate` renders digital silence.
///
/// `max_abs_node_v` is the run's largest internal node voltage (`DIAG:
/// max_abs_v_prev`); it splits the two very different failures the user cannot
/// otherwise tell apart — "the whole circuit is dead" versus "the circuit is
/// alive but the output tap is not connected to it".
fn silent_output_warning(
    peak: f64,
    output_node: &str,
    samples: Option<u64>,
    max_abs_node_v: Option<f64>,
) -> String {
    let sample_count = samples.map_or_else(|| "every".to_string(), |n| format!("all {n}"));
    let mut msg = format!(
        "WARNING: the rendered output is digital silence \u{2014} peak {peak:.6e} V across \
         {sample_count} samples of node '{output_node}', below the {SILENT_OUTPUT_PEAK_V:e} V \
         floor melange treats as silence. The run completed and the WAV was written, but there \
         is nothing in the file to hear.\n"
    );
    msg.push_str("  Nothing else in this report flags that, so check, roughly in order:\n");
    msg.push_str(
        "    1. A mistyped node name the topology check could not see. A typo that leaves a\n\
         \x20      node on one resistor or cap is refused, but one on a transistor, tube or\n\
         \x20      op-amp terminal only warns (scroll up), and one that happens to spell\n\
         \x20      another real node is invisible. Check every node name in the netlist\n\
         \x20      against `melange nodes <circuit>`.\n",
    );
    msg.push_str(
        "    2. A node with no DC path to ground (one reachable only through capacitors): that\n\
         \x20      section cannot bias, so no signal crosses it.\n",
    );
    msg.push_str(&format!(
        "    3. `--output-node {output_node}` is not where this circuit's signal actually comes\n\
         \x20      out; `melange nodes <circuit>` lists the node names.\n"
    ));
    msg.push_str(
        "    4. The input never reaches the output: a missing coupling cap, a pot or switch\n\
         \x20      sitting at its zero position, a supply rail that was never connected, or a\n\
         \x20      SPICE-style stimulus source left in the deck that still clamps the input\n\
         \x20      (melange drives the input node itself; the deck needs no source for it).\n",
    );
    match max_abs_node_v {
        Some(v) if v.is_finite() && v.abs() < SILENT_OUTPUT_PEAK_V => msg.push_str(
            "  Every internal node also stayed at 0 V for the whole run, so nothing in the\n\
             \x20 circuit moved at all \u{2014} suspect the drive or the supply (check the input node\n\
             \x20 and any DC sources) before the output tap.\n",
        ),
        Some(v) if v.is_finite() => msg.push_str(&format!(
            "  Internal nodes did move (max |v| = {v:.3} V), so at least part of the circuit is\n\
             \x20 live \u{2014} the break is between that live part and '{output_node}'. If the figure is\n\
             \x20 just the drive amplitude, nothing past the input node is moving at all.\n"
        )),
        _ => {}
    }
    msg.push_str(
        "  This is a warning, not an error: the file was written and the exit status is\n\
         \x20 unchanged. If you meant to render a muted state, ignore it.",
    );
    msg
}

/// Counters that describe how hard the solve worked, not whether its result
/// can be trusted: the default output leaves them to `-v` and lets the verdict
/// line summarize them. Every other counter prints by default when nonzero
/// (an unsolved sample, a reset, a clamp, a missed sub-sample fire, and any
/// counter added later until it is classified here).
const EXPLANATORY_DIAGS: [&str; 16] = [
    "samples",
    "probes_written",
    "peak",
    "max_abs_v_prev",
    "nr_max_iter_count",
    "substep_count",
    "be_fallback_count",
    "region_exit_count",
    "warm_start_fallback_count",
    "subsample_fire_count",
    "subsample_fire_detected",
    "subsample_fire_resolved",
    "subsample_fire_unresolved_gridpoint",
    "subsample_fire_segments",
    "subsample_fire_schur_builds",
    "subsample_fire_schur_reuses",
];

/// Whether a `DIAG:` line prints without `-v`: a note (non-numeric value)
/// always does; a counter does when it is nonzero and not explanatory.
fn diag_shown_by_default(key: &str, value: &str) -> bool {
    match value.trim().parse::<f64>() {
        Ok(v) => v != 0.0 && !EXPLANATORY_DIAGS.contains(&key),
        Err(_) => true,
    }
}

/// `s` with its first letter upper-cased.
fn capitalize(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().chain(c).collect(),
        None => String::new(),
    }
}

/// The rendered output's level, in volts and in dBFS (the WAV maps 1 V to
/// full scale).
fn output_level_line(peak: f64) -> String {
    if !peak.is_finite() {
        return format!("Output peak: {peak} (not a number: see the counters above)");
    }
    let a = peak.abs();
    let volts = if (1e-3..1e4).contains(&a) {
        format!("{a:.4} V")
    } else {
        format!("{a:.3e} V")
    };
    if a > 0.0 {
        format!(
            "Output peak: {volts} ({:.1} dBFS; the WAV's full scale is 1 V)",
            20.0 * a.log10()
        )
    } else {
        format!("Output peak: {volts} (the WAV's full scale is 1 V)")
    }
}

pub(crate) fn simulate_circuit_source(
    circuit_source: &circuits::CircuitSource,
    opts: &SimulateOptions,
) -> Result<()> {
    // The rendering binary takes its paths as UTF-8 arguments. Checked before
    // the build, so a path it cannot be handed fails here, not after the
    // compile — and never silently becomes `output.wav` in the working dir.
    let input_audio_arg = opts
        .input_audio
        .map(|p| utf8_path_arg(p, "--input-audio"))
        .transpose()?;
    let output_arg = utf8_path_arg(opts.output, "--output")?;
    let probe_csv_arg = opts
        .probe_csv
        .map(|p| utf8_path_arg(p, "--probe-csv"))
        .transpose()?;

    println!("melange simulate");
    println!("  Source: {}", circuit_source.name());
    println!();

    let verbose = opts.verbose;
    let netlist_str = load_circuit_text(circuit_source, &|l| {
        if verbose || !is_build_detail(l) {
            println!("{l}")
        }
    })?;

    // The one build every verb ships (melange_solver::build). Output layout:
    // [primary, probe_1, probe_2, ...]. The generated `process_sample` returns
    // these in order; the simulate main routes index 0 to the WAV and 1.. to the
    // probe CSV.
    let mut output_nodes = vec![opts.output_node.to_string()];
    output_nodes.extend(opts.probes.iter().cloned());
    let build_opts = melange_solver::build::BuildOptions {
        sample_rate: opts.sample_rate,
        circuit_name: "simulate".to_string(),
        input_nodes: vec![opts.input_node.to_string()],
        output_nodes,
        max_iter: opts.max_iter,
        tolerance: 1e-9,
        output_scale: 1.0,
        output_clamp: 10.0,
        input_resistance: opts.input_resistance_flag,
        oversampling: opts.oversampling,
        oversampling_set: melange_solver::build::OversamplingSet::Off,
        dc_block: false, // preserve DC for accurate WAV output
        solver: opts.solver.to_string(),
        backward_euler: opts.backward_euler,
        force_trap: opts.force_trap,
        tube_grid_fa: opts.tube_grid_fa.to_string(),
        subsample_fire: opts.subsample_fire,
        subsample_lit_factor: diag_lit_factor(),
        bjt_fa_mode: melange_solver::codegen::BjtFaMode::Off,
        opamp_rail_mode: opts.opamp_rail_mode,
        nodal_sub_path_override: opts.nodal_sub_path_override,
        allow_static_glow_on_full_lu: false,
        noise_mode: opts.noise_mode,
        noise_seed: opts.noise_seed,
        emit_dc_op_recompute: false,
        plugin_format: false,
        // Knob settings mean the same thing here as in `analyze`.
        pot_overrides: Some(opts.pot_overrides.to_vec()),
        resolve_taps: false,
        inject_runtime: true,
        disable_unit_variation: false,
        disable_self_heating: false,
        allow_unconverged_dc_op: opts.allow_unconverged_dc_op,
        dc_op_max_iterations: opts.dc_op_max_iterations,
        output_clamp_auto: false,
    };
    let built = melange_solver::build::build(
        &netlist_str,
        &build_opts,
        &|a| report_build_line(verbose, a, |l| println!("{l}")),
        &|a| eprintln!("{a}"),
    )
    .map_err(|e| build_error_in(e, &netlist_str))?;
    let route_is_dk = built.solver_label == "DK";
    let circuit_has_opamps = has_opamps(&built.netlist);
    let summary = route_summary(
        built.solver_label,
        opts.solver,
        built.generated.meta.nodal_sub_path,
        built.generated.meta.integrator_selection,
    );
    if verbose {
        // The one-line summary, then the detail behind it.
        println!("  {}", summary.trim_end_matches(" (-v for why)"));
        println!(
            "  Sample rate: {} Hz ({})",
            opts.sample_rate, opts.sample_rate_source
        );
        print_run_route_detail(&built, opts.max_iter.is_some(), &|l| println!("{l}"));
    } else {
        println!("  {summary}");
    }
    let injection_specs = built.injection_specs;
    let generated = built.generated;
    let netlist = built.netlist;
    let oversampling = built.oversampling;
    // The budget the generated `MAX_ITER` carries.
    let max_iter = built.max_iter;

    // Map `--inject FIELD=SPEC` to injection indices (by field name).
    let mut inject_driven: Vec<(usize, codegen_runner::InjectSource)> = Vec::new();
    for drive in opts.inject_drives {
        let (field, source) = codegen_runner::parse_inject_drive(drive)?;
        let idx = injection_specs
            .iter()
            .position(|s| s.name.eq_ignore_ascii_case(&field))
            .ok_or_else(|| {
                let names: Vec<&str> = injection_specs.iter().map(|s| s.name.as_str()).collect();
                anyhow::anyhow!(
                    "--inject field '{}' is not a `.inject` field in this deck. Available: {:?}",
                    field,
                    names
                )
            })?;
        inject_driven.push((idx, source));
    }
    // Each driven field's position within its rate's `process_sample`
    // argument (directive order within the rate, as the generated
    // `INJECT_HOST_*` / `INJECT_INNER_*` constants list them).
    let inject_drives: Vec<codegen_runner::InjectDrive> = inject_driven
        .iter()
        .map(|&(idx, source)| {
            let host_rate = injection_specs[idx].host_rate;
            let index = injection_specs[..idx]
                .iter()
                .filter(|s| s.host_rate == host_rate)
                .count();
            codegen_runner::InjectDrive {
                host_rate,
                index,
                source,
            }
        })
        .collect();
    // Warn on any `.inject` field left undriven (it injects 0 V) — only once the
    // user has supplied at least one `--inject`, so an unrelated run stays quiet.
    if !opts.inject_drives.is_empty() {
        for (si, spec) in injection_specs.iter().enumerate() {
            if !inject_driven.iter().any(|(i, _)| *i == si) {
                println!(
                    "warning: `.inject` field '{}' has no --inject drive; it will inject 0 V",
                    spec.name
                );
            }
        }
    }

    if verbose {
        println!("  {} lines of code", generated.code.lines().count());
    }

    // Step 5 (the build printed 1-4 under -v): append simulate main, compile,
    // run. Probe names map 1:1 to `output_nodes[1..]`. The generated main uses
    // them for the CSV header; the runtime argv[3] supplies the CSV path.
    if verbose {
        println!("Step 5: Compiling and running...");
    }
    let probe_names: Vec<&str> = opts.probes.iter().map(|s| s.as_str()).collect();
    // Resolve --switch NAME=POS and apply at runtime via set_switch_N (same as
    // the generated plugin / `analyze`): keeps the netlist at position-0 values
    // so the emitted G/C constants stay consistent with the delta-from-nominal
    // stamp in set_switch_N.
    let switch_runtime_overrides = resolve_switch_overrides(&netlist, opts.switch_overrides)?;
    let switch_calls: Vec<String> = switch_runtime_overrides
        .iter()
        .map(|(idx, pos)| format!("state.set_switch_{idx}({pos})"))
        .collect();
    let extra_diag_counters = SUBSAMPLE_FIRE_DIAG_FIELDS
        .iter()
        .copied()
        // The unsolved-sample counters exist only where their mechanism
        // does (the hold on nodal builds with a Newton solve; the committed
        // count on DK and wherever a failed op-amp pin is committed).
        // Presence-filtered like the rest, so a build stays silent rather
        // than reporting a reassuring zero for a mechanism it does not have.
        .chain([
            "diag_unsolved_sample_count",
            "diag_nr_hold_count",
            "diag_nr_unconverged_commit_count",
            "diag_warm_start_fallback_count",
            "diag_reduced_model_exit_count",
        ])
        .chain(INPUT_DIAG_FIELDS)
        .filter(|f| declares_state_field(&generated.code, f))
        .collect::<Vec<&str>>();
    let simulate_main = codegen_runner::generate_simulate_main(codegen_runner::SimulateMain {
        sample_rate: opts.sample_rate,
        // `--pot` is baked into the netlist's R values before the MNA is built
        // (see `apply_pot_overrides`), so there is nothing to set at runtime.
        pot_calls: &[],
        switch_calls: &switch_calls,
        amplitude: if opts.input_audio.is_none() {
            Some(opts.amplitude)
        } else {
            None
        },
        freq: 1000.0, // test tone freq
        duration_secs: opts.duration,
        probe_names: &probe_names,
        noise_enabled: opts.noise_mode != melange_solver::codegen::NoiseMode::Off,
        inject_driven: &inject_drives,
        num_inject: injection_specs.len(),
        // Build-conditional counters: only present in the generated state when
        // the emitter resolved the feature active (auto = glow on nodal-Schur).
        extra_diag_counters: &extra_diag_counters,
        pcm16: opts.pcm16,
    });
    let full_source = format!("{}\n{}", generated.code, simulate_main);

    let binary_cache =
        codegen_runner::BinaryCache::new().with_context(|| "Failed to create binary cache")?;
    let compiled = binary_cache
        .compile(&full_source, "simulate")
        .with_context(|| "Compilation failed")?;
    if verbose {
        if compiled.cached {
            println!("  Using cached binary");
        } else {
            println!("  Compiled successfully");
        }
    }

    // Run the binary. argv layout:
    //   [1] input.wav | "--tone"
    //   [2] output.wav
    //   [3] probes.csv  (only present when probes were baked in)
    let mut cmd = std::process::Command::new(&compiled.path);
    cmd.arg(input_audio_arg.unwrap_or("--tone"));
    cmd.arg(output_arg);
    if let Some(csv_path) = probe_csv_arg {
        cmd.arg(csv_path);
    }

    let result = cmd
        .output()
        .with_context(|| "Failed to run compiled binary")?;

    if !result.status.success() {
        let stderr = String::from_utf8_lossy(&result.stderr);
        anyhow::bail!("Simulate binary failed:\n{}", stderr);
    }

    // Parse diagnostics from stderr
    let stderr = String::from_utf8_lossy(&result.stderr);
    let mut nr_max_iter_count: Option<u64> = None;
    let mut diag_samples: Option<u64> = None;
    // Peak (V) and the largest node voltage seen anywhere in the circuit. Both
    // are already computed by the generated binary; the silent-output check
    // below reads them here rather than re-scanning the rendered buffer.
    let mut diag_peak: Option<f64> = None;
    let mut diag_max_abs_v_prev: Option<f64> = None;
    // The two unsolved-sample counters: the death-spiral hold, and a failed
    // op-amp pin (or a DK Newton) whose iterate was committed. A nodal build
    // with an active-set pin declares both, so they are kept apart.
    let mut nr_hold_count: Option<u64> = None;
    let mut nr_commit_count: Option<u64> = None;
    let mut reduced_exit_count: Option<u64> = None;
    // The unified count every build declares; it is what the verb refuses on.
    let mut unsolved_count: Option<u64> = None;
    // Counters that mean the solver had to WORK, not that anything is wrong.
    // Printed as a bare list they read as a hazard panel a newcomer cannot
    // interpret: is `region_exit_count: 0` good? is 5 bad? Nothing said.
    let mut recoveries: u64 = 0;
    let mut resets: u64 = 0;
    // Every `DIAG:<key>=<value>` line, in order. `-v` prints them all; the
    // default prints only the ones that need attention (see
    // `diag_shown_by_default`) under the verdict.
    let mut diag_lines: Vec<(&str, &str)> = Vec::new();
    for line in stderr.lines() {
        if let Some(diag) = line.strip_prefix("DIAG:") {
            let parts: Vec<&str> = diag.splitn(2, '=').collect();
            if parts.len() == 2 {
                let v: u64 = parts[1].trim().parse().unwrap_or(0);
                match parts[0] {
                    "substep_count" | "be_fallback_count" => recoveries += v,
                    "nan_reset_count" | "magnitude_reset_count" => resets += v,
                    _ => {}
                }
                diag_lines.push((parts[0], parts[1]));
                match parts[0] {
                    "nr_max_iter_count" => nr_max_iter_count = parts[1].trim().parse().ok(),
                    // Both mean "not a solution"; see nr_commit_count above.
                    "unsolved_sample_count" => unsolved_count = parts[1].trim().parse().ok(),
                    "nr_hold_count" => nr_hold_count = parts[1].trim().parse().ok(),
                    "nr_unconverged_commit_count" => nr_commit_count = parts[1].trim().parse().ok(),
                    "reduced_model_exit_count" => reduced_exit_count = parts[1].trim().parse().ok(),
                    "samples" => diag_samples = parts[1].trim().parse().ok(),
                    "peak" => diag_peak = parts[1].trim().parse().ok(),
                    "max_abs_v_prev" => diag_max_abs_v_prev = parts[1].trim().parse().ok(),
                    _ => {}
                }
            }
        }
    }
    let printed_header = !diag_lines.is_empty();
    if printed_header {
        if verbose {
            println!("  Solver diagnostics:");
            for (key, value) in &diag_lines {
                println!("    {key}: {value}");
            }
            println!(
                "    (peak and max_abs_v_prev are volts: the output's largest |sample|, and the \
                 largest |node voltage| anywhere in the circuit)"
            );
        } else {
            println!("  Solver diagnostics (-v lists every counter):");
            for (key, value) in diag_lines
                .iter()
                .filter(|(k, v)| diag_shown_by_default(k, v))
            {
                println!("    {key}: {value}");
            }
        }
    }

    // One line saying what the counters amount to. They are meaningful to a
    // maintainer and opaque to everyone else, and a list of numbers with no
    // verdict trains people to skip it.
    //
    // `recoveries` counts retries RUN, not samples they saved: when any sample
    // stayed unsolved the line says so, rather than reassuring above the ERROR.
    let held = nr_hold_count.unwrap_or(0);
    let committed = nr_commit_count.unwrap_or(0);
    let total = unsolved_count.unwrap_or(held + committed);
    // DK has no sub-step ladder; its only retry is backward Euler.
    let retries = if route_is_dk {
        "backward-Euler retries"
    } else {
        "sub-step and backward-Euler retries"
    };
    let a_retry = if route_is_dk {
        "a backward-Euler retry"
    } else {
        "a sub-step or backward-Euler retry"
    };
    if printed_header {
        let capped = nr_max_iter_count.unwrap_or(0);
        if resets > 0 {
            println!(
                "    -> {resets} NaN/magnitude reset(s): the solve blew up and was reset. \
                 Treat this output as suspect."
            );
        } else if total > 0 {
            println!(
                "    -> {capped} sample(s) hit the iteration ceiling; the {retries} ran \
                 {recoveries} time(s) and {total} sample(s) were still never solved (see the \
                 ERROR below)."
            );
        } else if capped > 0 && recoveries > 0 {
            println!(
                "    -> {capped} sample(s) hit the iteration ceiling and {a_retry} solved each \
                 of them. Normal on hard transients."
            );
        } else if capped > 0 {
            println!("    -> {capped} sample(s) hit the iteration ceiling.");
        } else {
            println!("    -> nothing to flag: no iteration-ceiling hits, no resets.");
        }
    }
    if let Some(peak) = diag_peak {
        println!("  {}", output_level_line(peak));
    }

    // Validity warning: if Newton-Raphson hit its iteration ceiling on a large
    // fraction of (internal-rate) samples, the solver never actually converged
    // there. A starved solve can latch every node at a DC-ish value that looks
    // exactly like a physical steady state while being numerically meaningless
    // — and nothing else in the normal output flags it as fatal-to-validity.
    // Surface it loudly. (Raised by melange-circuits 2026-08-15: a Ge astable
    // cascade starved at --max-iter 70 latched every node and read as real
    // physics for half a day; --max-iter 1000 converged.)
    //
    // Threshold is 20%, not the originally-suggested 5%: a healthy run of that
    // same circuit still shows ~5% onset-only max-iter samples (BE fallback
    // rescues them), so 5% would cry wolf. 20% cleanly separates onset
    // transients from a systematic latch (~100% in the failing case).
    if let (Some(nr_fail), Some(samples)) = (nr_max_iter_count, diag_samples) {
        let internal_samples = samples.saturating_mul(oversampling as u64).max(1);
        let frac = nr_fail as f64 / internal_samples as f64;
        if frac > 0.20 {
            eprintln!(
                "WARNING: Newton-Raphson hit its iteration ceiling {} times across {} internal \
                 samples ({:.0}%; a sample can fail both its trapezoidal and BE-fallback solve, so \
                 this can exceed 100%). The solver is failing to converge on a large fraction of \
                 samples — the output can latch at a DC-ish value that looks like a physical steady \
                 state while being numerically meaningless. Verify the result. A larger \
                 --max-iter (current {max_iter}) can let a slow but non-regenerative solve converge; on an \
                 oscillator or switching circuit it can instead let Newton settle on a spurious \
                 oscillation of the discrete step equations, with no unsolved sample to show it, \
                 so it is not a supported way past unsolved samples there (docs/limitations.md, \
                 \"Self-starting two-transistor astables\").",
                nr_fail,
                internal_samples,
                frac * 100.0,
            );
        }
    }

    println!();
    println!("Output written to: {}", opts.output.display());
    if let Some(csv_path) = opts.probe_csv {
        println!("Probes written to: {}", csv_path.display());
    }

    // Silent-output check. melange already computed the peak; a run that
    // renders digital silence must not slip out the door looking healthy just
    // because every counter is clean and the exit code is 0. See
    // `output_is_silent` for the threshold rationale.
    //
    // Deliberate silence is not a defect: a zero-amplitude test tone with noise
    // off renders a silent file BY REQUEST, so that combination stays quiet.
    let silence_requested = opts.input_audio.is_none()
        && opts.amplitude == 0.0
        && opts.noise_mode == melange_solver::codegen::NoiseMode::Off;
    if let Some(peak) = diag_peak {
        if output_is_silent(peak) && !silence_requested {
            eprintln!();
            eprintln!(
                "{}",
                silent_output_warning(peak, opts.output_node, diag_samples, diag_max_abs_v_prev)
            );
        }
    }

    // The death-spiral hold: samples where every Newton path failed and the
    // previous state was committed as the answer. This is NOT the iteration
    // ceiling warned about above — a capped sample that a fallback rescued is a
    // converged solution by another consistent scheme. A held sample is not a
    // solution at all, and nothing about the rendered audio can tell you: it is
    // bounded and smooth, so peak, RMS and the waveform all read healthy.
    //
    // Fail, do not warn. The whole failure mode is that it looks fine
    // (design review).
    if total > 0 {
        let of = diag_samples
            .map(|s| format!(" of {s} output samples"))
            .unwrap_or_default();
        if held > 0 {
            eprintln!();
            eprintln!(
                "ERROR: {held} sample(s){of} were never solved. Every Newton path failed there \
                 (the solve, the sub-step and, on a trapezoidal build, backward Euler), so the \
                 solver committed the PREVIOUS state as the output. Those samples are not a \
                 solution to this circuit.\n\
                 \n\
                 The rendered file looks healthy — a held value is bounded and smooth, so peak, \
                 RMS and the waveform cannot show it. Under a held input the hold is also a fixed \
                 point: the next sample re-poses the identical problem and fails identically, so \
                 one hard sample can freeze the render to its end."
            );
        }
        let reduced = reduced_exit_count.unwrap_or(0);
        if reduced > 0 {
            eprintln!();
            eprintln!(
                "ERROR: {reduced} sample(s){of} were solved on a REDUCED device model outside \
                 its region: a `.linearize`d device driven out of its small-signal region (a \
                 triode cut off or its grid past the conduction onset, a BJT cut off or \
                 saturated), or a grid-off pentode (--tube-grid-fa on) whose grid conducted. The reduction assumes the device never goes there, so those \
                 samples are not a solution to this circuit. Remove `.linearize` for a stage \
                 that leaves its region at this drive, or lower the drive; for a grid-off \
                 pentode, run without the reduction (--tube-grid-fa off)."
            );
        }
        if committed > 0 {
            eprintln!();
            eprintln!(
                "ERROR: {committed} sample(s){of} were never solved. {} ended unconverged, and \
                 the solver committed that iterate as the output. Those samples are not a \
                 solution to this circuit, and a bounded, smooth render does not show it.",
                capitalize(unconverged_commit_source(route_is_dk, circuit_has_opamps))
            );
        }
        // On DK, the route itself is the first thing to change: the nodal
        // route has the sub-step rescue DK lacks.
        let dk_remedy = if committed > 0 {
            dk_unsolved_remedy(route_is_dk)
        } else {
            None
        };
        if let Some(remedy) = dk_remedy {
            eprintln!("\n{remedy}");
        }
        eprintln!(
            "\nThe WAV was still written, so you can listen to what it did. Do not treat it as \
             this circuit's output. Re-run with --allow-nr-hold to accept it anyway."
        );
        if !opts.allow_nr_hold {
            if dk_remedy.is_some() {
                anyhow::bail!(
                    "{total} sample(s) were never solved on the DK route (try --solver nodal; \
                     --allow-nr-hold to accept the render anyway)"
                );
            }
            anyhow::bail!("{total} sample(s) were never solved (--allow-nr-hold to override)");
        }
        eprintln!("(--allow-nr-hold given: continuing.)");
    }
    refuse_on_input_diag(&stderr, opts.allow_input_clamp)?;
    Ok(())
}

#[cfg(test)]
mod silent_output_tests {
    use super::*;

    /// Exactly-zero is the case that shipped a 192 KB file of digital silence
    /// with a clean report and exit 0.
    #[test]
    fn hard_zero_is_silent() {
        assert!(output_is_silent(0.0));
        assert!(output_is_silent(-0.0));
    }

    /// The reason the rule is a threshold and not `== 0.0`: a denormal output
    /// is just as broken, and equality would wave it through.
    #[test]
    fn absurdly_small_is_silent() {
        assert!(output_is_silent(1e-300));
        assert!(output_is_silent(-1e-300));
        assert!(output_is_silent(f64::MIN_POSITIVE));
        // Under the old `{:.6}` DIAG format everything here arrived as
        // `0.000000`; with `{:.6e}` the real magnitude survives the round trip
        // and the threshold is what decides, not the printf width.
        assert!(output_is_silent(4.9e-7));
    }

    /// The other half of the rule: a quiet-but-working circuit must never be
    /// nagged. 1e-6 V is -120 dBFS; real outputs live many decades above it.
    #[test]
    fn quiet_but_working_is_not_silent() {
        assert!(!output_is_silent(SILENT_OUTPUT_PEAK_V));
        assert!(!output_is_silent(1e-5)); // -100 dBFS, still absurdly quiet
        assert!(!output_is_silent(1e-3)); // a heavily attenuated tap
        assert!(!output_is_silent(0.5)); // ordinary line level
        assert!(!output_is_silent(-0.5));
    }

    /// A blown-up solve is a different failure with its own counters; calling
    /// it "nothing to hear" would be wrong.
    #[test]
    fn nan_and_inf_are_not_silence() {
        assert!(!output_is_silent(f64::NAN));
        assert!(!output_is_silent(f64::INFINITY));
        assert!(!output_is_silent(f64::NEG_INFINITY));
    }

    /// The message has to name the causes a newcomer cannot tell apart, and
    /// echo back the output node they actually passed.
    #[test]
    fn warning_names_the_actionable_causes() {
        // A peak far below the old `{:.6}` print resolution must be REPORTED,
        // not flattened to "0.000000" — that was the whole defect.
        let tiny = silent_output_warning(3.7e-9, "out", Some(48000), Some(12.0));
        assert!(
            tiny.contains("3.700000e-9"),
            "warning must quote the real peak: {tiny}"
        );
        assert!(!tiny.contains("0.000000 V"), "{tiny}");

        let msg = silent_output_warning(0.0, "out", Some(48000), Some(12.0));
        for needle in [
            "digital silence",
            "mistyped node name",
            "DC path to ground",
            "--output-node out",
            "melange nodes",
            "warning, not an error",
            "48000",
        ] {
            assert!(
                msg.contains(needle),
                "message must mention {needle:?}: {msg}"
            );
        }
    }

    /// Live-but-disconnected and stone-dead are different bugs; the internal
    /// node peak is what separates them.
    #[test]
    fn warning_distinguishes_dead_circuit_from_dead_output_tap() {
        let alive = silent_output_warning(0.0, "out", Some(100), Some(12.0));
        assert!(alive.contains("Internal nodes did move"), "{alive}");
        assert!(alive.contains("12.000"), "{alive}");

        let dead = silent_output_warning(0.0, "out", Some(100), Some(0.0));
        assert!(
            dead.contains("nothing in the\n  circuit moved at all"),
            "{dead}"
        );

        // No `max_abs_v_prev` in the diagnostics: say nothing rather than guess.
        let unknown = silent_output_warning(0.0, "out", None, None);
        assert!(!unknown.contains("Internal nodes did move"), "{unknown}");
        assert!(!unknown.contains("moved at all"), "{unknown}");
    }
}
