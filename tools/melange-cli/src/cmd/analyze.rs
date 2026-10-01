use crate::common::{
    build_error, declares_state_field, diag_lit_factor, load_circuit_text, print_run_route_detail,
    refuse_on_input_diag, resolve_switch_overrides, route_summary, INPUT_DIAG_FIELDS,
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
        verbose,
    } = *opts;
    // Match parse-time node normalization (lowercase, gnd→0).
    let input_node_owned = melange_solver::parser::normalize_node_name(input_node_name);
    let input_node_name = input_node_owned.as_str();
    let output_node_owned = melange_solver::parser::normalize_node_name(output_node_name);
    let output_node_name = output_node_owned.as_str();

    eprintln!("melange analyze (frequency response)");

    // Analyze writes its CSV to stdout, so the loader's lines go to stderr.
    let netlist_str = load_circuit_text(circuit_source, &|l| eprintln!("{l}"))?;

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
    let built =
        melange_solver::build::build(&netlist_str, &build_opts, &|a| eprintln!("{a}"), &|a| {
            eprintln!("{a}")
        })
        .map_err(build_error)?;
    if verbose {
        print_run_route_detail(&built, max_iter.is_some(), &|l| eprintln!("{l}"));
    } else {
        eprintln!(
            "  {}",
            route_summary(
                built.solver_label,
                solver,
                built.generated.meta.nodal_sub_path,
                built.generated.meta.integrator_selection,
            )
        );
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
    let frequencies = generate_log_frequencies(start_freq, end_freq, points_per_decade);
    eprintln!(
        "  {} frequency points from {:.0} Hz to {:.0} Hz",
        frequencies.len(),
        start_freq,
        end_freq
    );
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

    // Determine settle time
    let settle_secs = if has_inductors { 5.0 } else { 0.5 };

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
    let analyze_main = codegen_runner::generate_analyze_main(
        &frequencies,
        amplitude,
        sample_rate,
        settle_secs,
        &[], // pot calls (already baked into netlist)
        &switch_calls,
        harmonics,
        noise_mode != melange_solver::codegen::NoiseMode::Off,
        &INPUT_DIAG_FIELDS
            .iter()
            .copied()
            .filter(|f| declares_state_field(&generated.code, f))
            .collect::<Vec<&str>>(),
    );
    let full_source = format!("{}\n{}", generated.code, analyze_main);

    let binary_cache =
        codegen_runner::BinaryCache::new().with_context(|| "Failed to create binary cache")?;
    let compiled = binary_cache
        .compile(&full_source, "analyze")
        .with_context(|| "Compilation failed")?;
    if compiled.cached {
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

    // Print per-frequency diagnostics from stderr
    let stderr = String::from_utf8_lossy(&result.stderr);
    for line in stderr.lines() {
        eprintln!("{}", line);
    }
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
