//! melange-cli - Command line tool for circuit modeling
//!
//! Usage:
//!   melange compile input.cir --output circuit.rs
//!   melange validate input.cir --output-node out
//!   melange simulate input.cir --input-audio input.wav --output output.wav
//!   melange sources list
//!   melange builtins

// Doc comments use markdown lists whose continuations render fine.
#![allow(clippy::doc_lazy_continuation)]

pub mod cache;
pub mod circuits;
pub mod codegen_runner;
pub mod index_cmd;
pub mod plugin_template;
pub mod sources;

mod args;
mod cli;
mod cmd;
mod common;

use anyhow::{Context, Result};
use args::{parse_subsample_fire_mode, resolve_simulate_sample_rate};
use clap::Parser;
use cli::{full_version_string, Cli, Commands, ImportFormat};
use cmd::analyze::{analyze_freq_response, AnalyzeOptions};
use cmd::compile::{compile_circuit_source, CompileOptions};
use cmd::dc_op::{run_dc_op, DcOpOptions};
use cmd::manage::{handle_cache, handle_sources, list_builtins};
use cmd::nodes::list_nodes_source;
use cmd::simulate::{simulate_circuit_source, SimulateOptions};
use cmd::validate::{validate_circuit_source, ReductionModes, ToleranceOverrides, ValidateOptions};
use std::path::PathBuf;
#[cfg(test)]
use {
    args::utf8_path_arg, cli::CacheAction, cli::SOLVER_VALUES,
    cmd::compile::suggest_plugin_project_dir, common::route_summary, std::path::Path,
};

mod kicad_import;

fn main() -> Result<()> {
    // Exit quietly when the output pipe is closed early (e.g. `melange … | head`).
    // Rust ignores SIGPIPE by default, so a write to a closed stdout makes the
    // print machinery panic with "failed printing to stdout: Broken pipe"
    // (exit 101), which looks like a crash. Swallow exactly that panic and exit
    // cleanly; any other panic falls through to the default handler. A panic
    // hook is the std-only way to do this without pulling in a libc dependency.
    let default_panic_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let broken_pipe = info
            .payload()
            .downcast_ref::<String>()
            .is_some_and(|s| s.contains("Broken pipe"));
        if broken_pipe {
            std::process::exit(0);
        }
        default_panic_hook(info);
    }));

    // Initialize logger so log::info!/warn! from melange-solver are visible.
    // Default: only warnings. RUST_LOG=info or RUST_LOG=melange_solver=debug for more.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .format_target(false)
        .init();

    // Handle a top-level `--version`/`-V` ourselves so we can append the EXACT
    // build identity — a runtime hash of THIS executable — which clap's
    // build-time-static version string cannot carry. A version+commit is a
    // source-side pointer that cannot see a dirty tree or a different
    // feature/profile build; two binaries at one commit printed the same string
    // and a peer published from each (cross-project review). We honour the flag
    // the way clap would: only at the top level, before any subcommand.
    for arg in std::env::args().skip(1) {
        if arg == "-V" || arg == "--version" {
            println!("{}", full_version_string());
            return Ok(());
        }
        if !arg.starts_with('-') {
            break; // reached the subcommand — its own flags are not ours
        }
    }

    let cli = Cli::parse();
    let verbose = cli.verbose;

    match cli.command {
        Commands::Compile {
            input,
            output,
            sample_rate,
            input_node,
            output_node,
            max_iter,
            tolerance,
            output_scale,
            output_clamp,
            format,
            with_level_params,
            no_level_params,
            no_dc_block,
            input_resistance: input_resistance_flag,
            oversampling,
            solver,
            backward_euler,
            force_trap,
            tube_grid_fa,
            subsample_fire,
            subsample_lit_factor,
            bjt_fa,
            opamp_rail_mode,
            nodal_subpath,
            allow_static_glow_on_full_lu,
            noise,
            noise_seed,
            allow_unconverged_dc_op,
            dc_op_max_iterations,
            emit_dc_op_recompute,
            name,
            mono,
            wet_dry_mix,
            no_ear_protection,
            vendor,
            vendor_url,
            email,
            vst3_id,
            clap_id,
            cpu_baseline,
        } => {
            // Validate numeric CLI parameters
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!(
                    "Sample rate must be positive and finite, got {}",
                    sample_rate
                );
            }
            if tolerance <= 0.0 || !tolerance.is_finite() {
                anyhow::bail!("Tolerance must be positive and finite, got {}", tolerance);
            }
            if max_iter == Some(0) {
                anyhow::bail!("max-iter must be at least 1, got 0");
            }
            if let Some(n) = oversampling {
                if n != 1 && n != 2 && n != 4 {
                    anyhow::bail!("oversampling must be 1, 2, or 4, got {}", n);
                }
            }
            if output_clamp <= 0.0 || !output_clamp.is_finite() {
                anyhow::bail!(
                    "--output-clamp must be positive and finite, got {}",
                    output_clamp
                );
            }

            // Parse op-amp rail mode. Unknown values are user errors, not silent fallbacks.
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Unknown --opamp-rail-mode '{}'. Valid values: auto, none, hard, \
                         active-set, active-set-be, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;

            // Parse the nodal sub-path override. Unknown values are user errors.
            let nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::parse(
                &nodal_subpath,
            )
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "Unknown --nodal-subpath '{}'. Valid values: auto, schur, full-lu",
                    nodal_subpath
                )
            })?;

            let noise_mode =
                melange_solver::codegen::NoiseMode::parse(&noise).ok_or_else(|| {
                    anyhow::anyhow!(
                        "Unknown --noise '{}'. Valid values: off, thermal, shot, full",
                        noise
                    )
                })?;

            // Level params are on by default; --no-level-params disables them
            let level_params = with_level_params && !no_level_params;

            // Validate shipability flags (plugin branding).
            if let Some(url) = vendor_url.as_deref() {
                // Simple scheme check — we don't pull in a URL-parsing crate for this.
                if !(url.starts_with("http://") || url.starts_with("https://")) {
                    anyhow::bail!("--vendor-url '{}' must start with http:// or https://", url);
                }
            }
            if let Some(id) = vst3_id.as_deref() {
                plugin_template::validate_vst3_id(id)?;
            }
            if let Some(id) = clap_id.as_deref() {
                if id.is_empty() {
                    anyhow::bail!("--clap-id cannot be empty");
                }
            }

            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            // Validate tube-grid-fa mode.
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "Unknown --tube-grid-fa '{}'. Valid values: auto, on, off",
                    tube_grid_fa
                );
            }
            // Validate bjt-fa mode.
            if !matches!(bjt_fa.as_str(), "auto" | "off" | "force") {
                anyhow::bail!(
                    "Unknown --bjt-fa '{}'. Valid values: auto, off, force",
                    bjt_fa
                );
            }
            let subsample_fire_mode = parse_subsample_fire_mode(&subsample_fire)?;

            compile_circuit_source(
                &circuit_source,
                CompileOptions {
                    output: &output,
                    sample_rate,
                    input_node: &input_node,
                    output_node: &output_node,
                    max_iter,
                    tolerance,
                    output_scale,
                    output_clamp,
                    format,
                    with_level_params: level_params,
                    input_resistance_flag,
                    oversampling_cli: oversampling,
                    no_dc_block,
                    solver_override: &solver,
                    backward_euler,
                    force_trap,
                    tube_grid_fa: &tube_grid_fa,
                    subsample_fire: subsample_fire_mode,
                    subsample_lit_factor,
                    bjt_fa: &bjt_fa,
                    opamp_rail_mode: rail_mode,
                    nodal_sub_path_override,
                    allow_static_glow_on_full_lu,
                    noise_mode,
                    noise_seed,
                    emit_dc_op_recompute,
                    allow_unconverged_dc_op,
                    dc_op_max_iterations,
                    plugin_name: name.as_deref(),
                    mono,
                    wet_dry_mix,
                    ear_protection: !no_ear_protection,
                    vendor: vendor.as_deref(),
                    vendor_url: vendor_url.as_deref(),
                    email: email.as_deref(),
                    vst3_id_override: vst3_id.as_deref(),
                    clap_id_override: clap_id.as_deref(),
                    cpu_baseline,
                    verbose,
                },
            )
        }
        Commands::Validate {
            input,
            output_node,
            sample_rate,
            duration,
            amplitude,
            input_node,
            csv,
            relaxed,
            rms_tolerance,
            peak_tolerance,
            max_rel_tolerance,
            corr_min,
            thd_tolerance,
            bjt_fa,
            tube_grid_fa,
            backward_euler,
            force_trap,
            oversampling,
            rate_sweep,
        } => {
            // Validate numeric CLI parameters
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!("sample-rate must be positive and finite");
            }
            if !matches!(oversampling, 1 | 2 | 4) {
                anyhow::bail!("oversampling must be 1, 2, or 4, got {}", oversampling);
            }
            if rate_sweep && oversampling != 1 {
                anyhow::bail!(
                    "--rate-sweep sets the rate itself with oversampling off; drop --oversampling"
                );
            }
            if !matches!(bjt_fa.as_str(), "auto" | "off" | "force") {
                anyhow::bail!(
                    "--bjt-fa must be one of: auto, off, force (got '{}')",
                    bjt_fa
                );
            }
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "--tube-grid-fa must be one of: auto, on, off (got '{}')",
                    tube_grid_fa
                );
            }
            if duration <= 0.0 || !duration.is_finite() {
                anyhow::bail!("duration must be positive and finite");
            }
            if amplitude <= 0.0 || !amplitude.is_finite() {
                anyhow::bail!("amplitude must be positive and finite");
            }

            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            validate_circuit_source(
                &circuit_source,
                ValidateOptions {
                    output_node: &output_node,
                    sample_rate,
                    duration,
                    amplitude,
                    input_node: &input_node,
                    csv_output: csv.as_ref(),
                    relaxed,
                    tol: ToleranceOverrides {
                        rms_pct: rms_tolerance,
                        peak_v: peak_tolerance,
                        max_rel_pct: max_rel_tolerance,
                        corr_min,
                        thd_db: thd_tolerance,
                    },
                    reductions: ReductionModes {
                        bjt_fa: &bjt_fa,
                        tube_grid_fa: &tube_grid_fa,
                        backward_euler,
                        force_trap,
                    },
                    oversampling,
                    rate_sweep,
                },
            )
        }
        Commands::Simulate {
            input,
            input_audio,
            output,
            sample_rate,
            input_node,
            output_node,
            duration,
            amplitude,
            input_resistance: input_resistance_flag,
            solver,
            nodal_subpath,
            opamp_rail_mode,
            tube_grid_fa,
            subsample_fire,
            oversampling,
            noise,
            noise_seed,
            allow_unconverged_dc_op,
            dc_op_max_iterations,
            backward_euler,
            force_trap,
            max_iter,
            allow_nr_hold,
            allow_input_clamp,
            probes,
            probe_csv,
            pcm16,
            pot_overrides,
            switch_overrides,
            inject_drives,
        } => {
            // Match parse-time node normalization (lowercase, gnd→0).
            let input_node = melange_solver::parser::normalize_node_name(&input_node);
            let output_node = melange_solver::parser::normalize_node_name(&output_node);
            // Probes are node names too — normalize them the same way so
            // `--probe GND`-style refs and mixed-case names resolve.
            let probes: Vec<String> = probes
                .iter()
                .map(|p| melange_solver::parser::normalize_node_name(p))
                .collect();
            if let Some(n) = oversampling {
                if n != 1 && n != 2 && n != 4 {
                    anyhow::bail!("oversampling must be 1, 2, or 4, got {}", n);
                }
            }
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "Unknown --tube-grid-fa '{}'. Valid values: auto, on, off",
                    tube_grid_fa
                );
            }
            let wav_rate = input_audio
                .as_deref()
                .map(|p| {
                    hound::WavReader::open(p)
                        .map(|r| r.spec().sample_rate)
                        .with_context(|| {
                            format!(
                                "Failed to read the --input-audio WAV header: {}",
                                p.display()
                            )
                        })
                })
                .transpose()?;
            let (sample_rate, sample_rate_source) =
                resolve_simulate_sample_rate(sample_rate, wav_rate)?;
            let subsample_fire_mode = parse_subsample_fire_mode(&subsample_fire)?;
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Invalid --opamp-rail-mode '{}'. Valid: auto, none, hard, \
                         active-set, active-set-be, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;
            let noise_mode =
                melange_solver::codegen::NoiseMode::parse(&noise).ok_or_else(|| {
                    anyhow::anyhow!(
                        "Invalid --noise '{}'. Valid: off, thermal, shot, full",
                        noise
                    )
                })?;
            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            // Default probe CSV: derive from output WAV (foo.wav → foo.probes.csv).
            // Only consulted when --probe is non-empty.
            let probe_csv_path: Option<PathBuf> = if probes.is_empty() {
                None
            } else if let Some(p) = probe_csv {
                Some(p)
            } else {
                let mut p = output.clone();
                let stem = p
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| "output".to_string());
                p.set_file_name(format!("{}.probes.csv", stem));
                Some(p)
            };
            let nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::parse(
                &nodal_subpath,
            )
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "Unknown --nodal-subpath '{}'. Valid values: auto, schur, full-lu",
                    nodal_subpath
                )
            })?;
            simulate_circuit_source(
                &circuit_source,
                &SimulateOptions {
                    nodal_sub_path_override,
                    input_audio: input_audio.as_deref(),
                    output: &output,
                    sample_rate,
                    sample_rate_source,
                    input_node: &input_node,
                    output_node: &output_node,
                    duration,
                    amplitude,
                    input_resistance_flag,
                    solver: &solver,
                    opamp_rail_mode: rail_mode,
                    tube_grid_fa: &tube_grid_fa,
                    subsample_fire: subsample_fire_mode,
                    oversampling,
                    noise_mode,
                    noise_seed,
                    allow_unconverged_dc_op,
                    dc_op_max_iterations,
                    backward_euler,
                    force_trap,
                    max_iter,
                    allow_nr_hold,
                    allow_input_clamp,
                    probes: &probes,
                    probe_csv: probe_csv_path.as_deref(),
                    pcm16,
                    pot_overrides: &pot_overrides,
                    switch_overrides: &switch_overrides,
                    inject_drives: &inject_drives,
                    verbose,
                },
            )
        }
        Commands::Analyze {
            input,
            input_node,
            output_node,
            start_freq,
            end_freq,
            points_per_decade,
            amplitude,
            sample_rate,
            input_resistance,
            output,
            pot_overrides,
            switch_overrides,
            harmonics,
            tube_grid_fa,
            solver,
            oversampling,
            opamp_rail_mode,
            nodal_subpath,
            noise,
            noise_seed,
            allow_unconverged_dc_op,
            dc_op_max_iterations,
            backward_euler,
            force_trap,
            max_iter,
            allow_input_clamp,
        } => {
            // Validate numeric CLI parameters
            if start_freq <= 0.0 || !start_freq.is_finite() {
                anyhow::bail!("start-freq must be positive and finite");
            }
            if end_freq <= start_freq || !end_freq.is_finite() {
                anyhow::bail!("end-freq must be greater than start-freq and finite");
            }
            if amplitude <= 0.0 || !amplitude.is_finite() {
                anyhow::bail!("amplitude must be positive and finite");
            }
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!("sample-rate must be positive and finite");
            }
            if points_per_decade == 0 {
                anyhow::bail!("points-per-decade must be at least 1");
            }
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "Unknown --tube-grid-fa '{}'. Valid values: auto, on, off",
                    tube_grid_fa
                );
            }
            if let Some(n) = oversampling {
                if n != 1 && n != 2 && n != 4 {
                    anyhow::bail!("oversampling must be 1, 2, or 4, got {}", n);
                }
            }
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Invalid --opamp-rail-mode '{}'. Valid: auto, none, hard, \
                         active-set, active-set-be, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;
            // Parse the nodal sub-path override. Unknown values are user errors.
            let nodal_sub_path_override = melange_solver::codegen::NodalSubPathOverride::parse(
                &nodal_subpath,
            )
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "Unknown --nodal-subpath '{}'. Valid values: auto, schur, full-lu",
                    nodal_subpath
                )
            })?;

            let noise_mode =
                melange_solver::codegen::NoiseMode::parse(&noise).ok_or_else(|| {
                    anyhow::anyhow!(
                        "Invalid --noise '{}'. Valid: off, thermal, shot, full",
                        noise
                    )
                })?;

            let circuit_source = circuits::resolve(&input)?;
            analyze_freq_response(
                &circuit_source,
                &AnalyzeOptions {
                    nodal_sub_path_override,
                    input_node: &input_node,
                    output_node: &output_node,
                    start_freq,
                    end_freq,
                    points_per_decade,
                    amplitude,
                    sample_rate,
                    input_resistance_flag: input_resistance,
                    output_file: output.as_ref(),
                    pot_overrides: &pot_overrides,
                    switch_overrides: &switch_overrides,
                    harmonics,
                    tube_grid_fa: &tube_grid_fa,
                    solver: &solver,
                    oversampling,
                    opamp_rail_mode: rail_mode,
                    noise_mode,
                    noise_seed,
                    allow_unconverged_dc_op,
                    dc_op_max_iterations,
                    backward_euler,
                    force_trap,
                    max_iter,
                    allow_input_clamp,
                    verbose,
                },
            )
        }
        Commands::DcOp {
            input,
            input_node,
            input_resistance,
            format,
            sample_rate,
            oversampling,
            solver,
            opamp_rail_mode,
            bjt_fa,
            tube_grid_fa,
            pot_overrides,
            allow_unconverged_dc_op,
            dc_op_max_iterations,
        } => {
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!(
                    "Sample rate must be positive and finite, got {}",
                    sample_rate
                );
            }
            if let Some(n) = oversampling {
                if !matches!(n, 1 | 2 | 4) {
                    anyhow::bail!("oversampling must be 1, 2, or 4, got {}", n);
                }
            }
            if !matches!(solver.as_str(), "auto" | "dk" | "nodal") {
                anyhow::bail!(
                    "Unknown --solver '{}'. Valid values: auto, dk, nodal",
                    solver
                );
            }
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "Unknown --tube-grid-fa '{}'. Valid values: auto, on, off",
                    tube_grid_fa
                );
            }
            if !matches!(bjt_fa.as_str(), "auto" | "off" | "force") {
                anyhow::bail!(
                    "Unknown --bjt-fa '{}'. Valid values: auto, off, force",
                    bjt_fa
                );
            }
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Unknown --opamp-rail-mode '{}'. Valid values: auto, none, hard, \
                         active-set, active-set-be, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;
            let circuit_source = circuits::resolve(&input)?;
            eprintln!("Resolved circuit: {}", circuit_source.name());
            run_dc_op(
                &circuit_source,
                &DcOpOptions {
                    input_node: &input_node,
                    input_resistance,
                    format: &format,
                    sample_rate,
                    oversampling,
                    solver: &solver,
                    opamp_rail_mode: rail_mode,
                    bjt_fa: &bjt_fa,
                    tube_grid_fa: &tube_grid_fa,
                    pot_overrides: &pot_overrides,
                    allow_unconverged_dc_op,
                    dc_op_max_iterations,
                },
            )
        }
        Commands::Nodes { input } => {
            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            list_nodes_source(&circuit_source)
        }
        Commands::Index { dir, check } => index_cmd::run(&dir, check),
        Commands::Sources { action } => handle_sources(action),
        Commands::Builtins => list_builtins(),
        Commands::Cache { action } => handle_cache(action),
        Commands::Import {
            input,
            output,
            format,
            from_schematic,
        } => kicad_import::import_kicad(&input, &output, &format, from_schematic),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simulate_sample_rate_follows_the_input_wav() {
        // No WAV: the flag, else 48 kHz.
        assert_eq!(
            resolve_simulate_sample_rate(None, None).unwrap(),
            (48_000.0, "default")
        );
        assert_eq!(
            resolve_simulate_sample_rate(Some(96_000.0), None).unwrap(),
            (96_000.0, "--sample-rate")
        );
        // WAV, no flag: the WAV's rate.
        assert_eq!(
            resolve_simulate_sample_rate(None, Some(44_100)).unwrap(),
            (44_100.0, "from the --input-audio WAV")
        );
        // WAV and a matching flag: fine.
        assert_eq!(
            resolve_simulate_sample_rate(Some(44_100.0), Some(44_100)).unwrap(),
            (44_100.0, "--sample-rate")
        );
        // WAV and a mismatching flag: refused, naming both rates.
        let err = resolve_simulate_sample_rate(Some(48_000.0), Some(44_100))
            .unwrap_err()
            .to_string();
        assert!(err.contains("48000 Hz"), "{err}");
        assert!(err.contains("44100 Hz"), "{err}");
        // Nonsense rates are refused rather than built.
        assert!(resolve_simulate_sample_rate(Some(0.0), None).is_err());
        assert!(resolve_simulate_sample_rate(None, Some(0)).is_err());
    }

    /// The `--format plugin` hint must name a DIRECTORY derived from the
    /// circuit, never echo back the `--format code` output FILE.
    #[test]
    fn plugin_project_suggestion_is_a_directory_name() {
        assert_eq!(suggest_plugin_project_dir("od.cir"), "od-plugin");
        assert_eq!(
            suggest_plugin_project_dir("/home/u/decks/passive-eq1a.cir"),
            "passive-eq1a-plugin"
        );
        assert_eq!(
            suggest_plugin_project_dir("builtin:passive-eq1a"),
            "passive-eq1a-plugin"
        );
        // Never a path, never an extension.
        let s = suggest_plugin_project_dir("/tmp/proj/src/circuit.rs");
        assert!(!s.contains('/'), "{s}");
        assert!(!s.contains('.'), "{s}");
        // Nothing usable -> a safe generic name rather than an empty --output.
        assert_eq!(suggest_plugin_project_dir("/"), "melange-plugin");
        assert_eq!(suggest_plugin_project_dir(""), "melange-plugin");
    }

    /// An unknown `--solver` value is a parse error on every verb that takes
    /// it. The build reads anything but `dk`/`nodal` as auto, so a typo used
    /// to build the auto route without a word.
    #[test]
    fn solver_typo_is_rejected_on_every_verb() {
        let verbs: [&[&str]; 4] = [
            &["melange", "compile", "x.cir", "-o", "x.rs"],
            &["melange", "simulate", "x.cir", "-o", "x.wav"],
            &["melange", "analyze", "x.cir"],
            &["melange", "dc-op", "x.cir"],
        ];
        for verb in verbs {
            for bad in ["nodel", "DK", "Auto", ""] {
                let mut args = verb.to_vec();
                args.extend(["--solver", bad]);
                let err = match Cli::try_parse_from(&args) {
                    Ok(_) => panic!("{args:?} must be refused"),
                    Err(e) => e,
                };
                assert_eq!(
                    err.kind(),
                    clap::error::ErrorKind::InvalidValue,
                    "{args:?}: {err}"
                );
            }
            for good in SOLVER_VALUES {
                let mut args = verb.to_vec();
                args.extend(["--solver", good]);
                assert!(Cli::try_parse_from(&args).is_ok(), "{args:?}");
            }
        }
    }

    /// `--with-level-params` was a flag that could only ever say true.
    #[test]
    fn with_level_params_can_be_set_false() {
        let level = |extra: &[&str]| -> (bool, bool) {
            let mut args = vec!["melange", "compile", "x.cir", "-o", "x.rs"];
            args.extend_from_slice(extra);
            match Cli::try_parse_from(&args).unwrap().command {
                Commands::Compile {
                    with_level_params,
                    no_level_params,
                    ..
                } => (with_level_params, no_level_params),
                _ => unreachable!(),
            }
        };
        assert_eq!(level(&[]), (true, false), "default on");
        assert_eq!(level(&["--with-level-params"]), (true, false));
        assert_eq!(level(&["--with-level-params=true"]), (true, false));
        assert_eq!(level(&["--with-level-params=false"]), (false, false));
        assert_eq!(level(&["--no-level-params"]), (true, true));
        // A bare flag must not swallow the next argument as its value.
        let mut args = vec!["melange", "compile", "--with-level-params", "x.cir"];
        args.extend(["-o", "x.rs"]);
        assert!(Cli::try_parse_from(&args).is_ok());
    }

    #[test]
    fn verbose_is_global_and_off_by_default() {
        let parse = |args: &[&str]| Cli::try_parse_from(args).unwrap().verbose;
        assert!(!parse(&["melange", "analyze", "x.cir"]));
        assert!(parse(&["melange", "-v", "analyze", "x.cir"]));
        assert!(parse(&["melange", "analyze", "x.cir", "--verbose"]));
    }

    /// Without `--max-iter` the build auto-tunes; with it, the value is pinned
    /// exactly — including 50, which used to be read as "not set".
    #[test]
    fn compile_max_iter_50_is_a_real_value() {
        let max_iter = |extra: &[&str]| -> Option<usize> {
            let mut args = vec!["melange", "compile", "x.cir", "-o", "x.rs"];
            args.extend_from_slice(extra);
            match Cli::try_parse_from(&args).unwrap().command {
                Commands::Compile { max_iter, .. } => max_iter,
                _ => unreachable!(),
            }
        };
        assert_eq!(max_iter(&[]), None);
        assert_eq!(max_iter(&["--max-iter", "50"]), Some(50));
    }

    #[test]
    fn cache_clear_binaries_flag() {
        let binaries = |args: &[&str]| match Cli::try_parse_from(args).unwrap().command {
            Commands::Cache {
                action: CacheAction::Clear { binaries },
            } => binaries,
            _ => unreachable!(),
        };
        assert!(!binaries(&["melange", "cache", "clear"]));
        assert!(binaries(&["melange", "cache", "clear", "--binaries"]));
    }

    /// A path the rendering binary cannot receive is refused, naming the flag,
    /// instead of silently becoming `output.wav`.
    #[cfg(unix)]
    #[test]
    fn non_utf8_simulate_path_is_refused() {
        use std::os::unix::ffi::OsStrExt;
        let bad = Path::new(std::ffi::OsStr::from_bytes(b"/tmp/out\xff.wav"));
        let err = utf8_path_arg(bad, "--output").unwrap_err().to_string();
        assert!(err.contains("--output") && err.contains("UTF-8"), "{err}");
        assert_eq!(
            utf8_path_arg(Path::new("/tmp/out.wav"), "--output").unwrap(),
            "/tmp/out.wav"
        );
    }

    #[test]
    fn route_summary_is_one_plain_line() {
        use melange_solver::codegen::ir::IntegratorSelection as Sel;
        use melange_solver::codegen::NodalSubPath;
        let dk = route_summary("DK", "auto", None, Sel::TrapDefault);
        assert_eq!(
            dk,
            "Solver: DK, chosen automatically; trapezoidal integration. (-v for why)"
        );
        let nodal = route_summary("nodal", "nodal", Some(NodalSubPath::FullLu), Sel::BeAuto);
        assert!(nodal.starts_with("Solver: nodal (full-lu sub-path), forced by --solver nodal;"));
        assert!(nodal.contains("backward-Euler integration (chosen automatically)"));
        for line in [&dk, &nodal] {
            assert!(!line.contains('\n'));
            for jargon in ["unstable", "ill-conditioned", "spectral", "K matrix"] {
                assert!(!line.contains(jargon), "{line}");
            }
        }
    }

    #[test]
    fn test_cli_parse() {
        // Test that CLI parsing works
        let cli = Cli::parse_from(["melange", "builtins"]);
        match cli.command {
            Commands::Builtins => {}
            _ => panic!("Expected Builtins command"),
        }
    }

    /// Fix 4 regression: `apply_grid_off_reduction` used to rebuild the MNA
    /// via `from_netlist_with_grid_off` with an EMPTY forward-active set,
    /// silently discarding the FA BJT reduction the caller had already
    /// applied (while the compile summary still printed the FA line).
    /// Circuits with both pentodes and forward-active BJTs need both reductions composed.
    #[test]
    fn grid_off_rebuild_preserves_forward_active_reduction() {
        use melange_solver::mna::MnaSystem;
        use melange_solver::parser::Netlist;

        let spice = "\
FA + grid-off composition
V1 vcc 0 DC 250
RB1 vcc b 470k
RB2 b 0 47k
RC vcc c 10k
RE e 0 1k
Q1 c b e QNPN
P1 plate grid cath screen EL84
RGL grid 0 1Meg
RSC vcc screen 1k
RPL vcc plate 5k
RK cath 0 130
.model QNPN NPN(IS=1e-14 BF=100)
.model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 \
              KP=152.4 KVB=4015.8 ALPHA_S=7.66 \
              A_FACTOR=4.344e-4 BETA_FACTOR=0.148)
";
        let netlist = Netlist::parse(spice).expect("parse");
        let full = MnaSystem::from_netlist(&netlist).expect("full MNA");
        assert_eq!(full.m, 5, "full system: BJT 2D + pentode 3D");

        // Simulate the caller's FA step (composition test — the FA set is
        // hand-supplied; detection is covered elsewhere).
        let forward_active: std::collections::HashSet<String> =
            ["Q1".to_string()].into_iter().collect();
        let mut mna = MnaSystem::from_netlist_forward_active(&netlist, &forward_active)
            .expect("FA-reduced MNA");
        assert_eq!(mna.m, 4, "after FA: BJT 1D + pentode 3D");

        let fa_config = melange_solver::codegen::CodegenConfig::default();
        // `--tube-grid-fa on` forces grid-off on every non-variable-mu
        // pentode regardless of DC bias, so detection is deterministic here.
        let grid_off = melange_solver::pipeline::apply_grid_off_reduction(
            &mut mna,
            &netlist,
            &fa_config,
            &forward_active,
            "on",
            "",
            48000.0,
            1,
            &[(0, 1.0)],
        )
        .expect("grid-off reduction");
        assert_eq!(grid_off.len(), 1, "pentode P1 must be grid-off reduced");
        assert_eq!(
            mna.m, 3,
            "after grid-off rebuild BOTH reductions must survive: BJT 1D + pentode 2D \
             (old bug: rebuild dropped FA, giving M=4 while the summary claimed FA)"
        );
    }
}
