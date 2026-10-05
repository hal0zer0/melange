use crate::args::parse_bjt_fa_mode;
use crate::cli::OutputFormat;
use crate::common::{
    build_error_in, diag_lit_factor, is_build_detail, load_circuit_text, route_summary,
};
use crate::{circuits, plugin_template};
use anyhow::{Context, Result};
use melange_solver::build::format_system_size;
use std::path::{Path, PathBuf};

/// What `compile -v` prints about the route (see [`print_compile_route_detail`]).
struct RouteDetail<'a> {
    solver_label: &'a str,
    solver_reason: &'a str,
    routing: &'a melange_solver::codegen::routing::RoutingDecision,
    meta: &'a melange_solver::codegen::CodegenMeta,
    max_iter: usize,
    /// `--max-iter` was given (the budget is the user's, not auto-tuned).
    max_iter_pinned: bool,
    m: usize,
    n_linearized: usize,
}

/// The `compile -v` routing detail: the router's reason, the kernel
/// measurements behind it, the integrator's reason and the Newton budget.
fn print_compile_route_detail(d: &RouteDetail<'_>) {
    // "(normal)" because the reason strings carry maintainer words like
    // "unstable" / "ill-conditioned" that read as warnings — see
    // `format_route_info`.
    println!(
        "    Solver: {} \u{2014} normal routing output, not a warning ({})",
        d.solver_label, d.solver_reason
    );
    // Non-negative K diagonal note, printed ONCE here (the low-level kernel
    // builder logs it at debug only — it is rebuilt several times per compile).
    // Informative when a transformer-coupled NFB circuit routes to nodal for a
    // different primary reason (e.g. trap instability) but ALSO has a positive
    // K diagonal the author may want to know about.
    if d.routing.k_diag_unsafe {
        println!(
            "    Note: non-negative K diagonal (positive DK-Schur feedback, \
             expected for transformer-coupled NFB) — handled by nodal full-NR."
        );
    }
    // Which nodal sub-path the emitter actually took. Reported by the emitter,
    // not re-derived here.
    //
    // "Nodal NR sub-path" — explicitly the nodal Newton implementation (Schur
    // reduction vs full-LU), distinct from the DK kernel's BJT internal-node
    // expansion, which also says "full LU" (see pipeline.rs). Name the
    // predicate that ACTUALLY fired, as the emitter reported it: rho is
    // context, labelled as context, not the deciding value.
    if let Some(sp) = d.meta.nodal_sub_path {
        match d.meta.nodal_full_lu_trigger {
            Some(trigger) => println!(
                "    Nodal NR sub-path: {sp} (nodal Newton; not DK node-expansion; \
                 trigger: {trigger}; nodal spectral radius {:.4})",
                d.meta.nodal_spectral_radius
            ),
            None => println!(
                "    Nodal NR sub-path: {sp} (nodal Newton; not DK node-expansion; \
                 nodal spectral radius {:.4})",
                d.meta.nodal_spectral_radius
            ),
        }
    }
    if d.routing.spectral_radius > 0.0 {
        // DK-kernel trap operator (routing::compute_spectral_radius) — this is
        // NOT the value that chose a nodal sub-path (see the line above).
        println!(
            "    DK-kernel spectral radius: {:.4}",
            d.routing.spectral_radius
        );
    }
    // Integration line: printed from the codegen-recorded selection so the
    // stated reason is the actual one (a `.integrator be` pin is NOT
    // "auto-selected").
    {
        use melange_solver::codegen::ir::IntegratorSelection as Sel;
        match d.meta.integrator_selection {
            Sel::BeAuto => {
                println!("    Integration: Backward Euler (auto-selected)");
                println!("      ({})", d.meta.integration_reason);
            }
            Sel::TrapDefault => {
                println!("    Integration: Trapezoidal");
                if !d.meta.integration_reason.is_empty() {
                    println!("      ({})", d.meta.integration_reason);
                }
            }
            Sel::TrapCliFlag => println!("    Integration: Trapezoidal (pinned by --force-trap)"),
            Sel::TrapDirective => {
                println!("    Integration: Trapezoidal (pinned by .integrator directive)");
            }
            Sel::BeCliFlag => println!("    Integration: Backward Euler (--backward-euler)"),
            Sel::BeDirective => {
                println!("    Integration: Backward Euler (pinned by .integrator directive)");
            }
            Sel::BeBehavioral => {
                println!("    Integration: Backward Euler (required by behavioral B-sources)");
            }
        }
    }
    // The budget the emitted `MAX_ITER` carries (the provenance `Build:`
    // line's `max_iter`), not the requested one.
    if d.max_iter_pinned {
        println!("    Max NR iterations: {} (--max-iter)", d.max_iter);
    } else {
        println!(
            "    Max NR iterations: {} (auto-tuned from M={}, ρ={:.2}{})",
            d.max_iter,
            d.m,
            d.routing.spectral_radius,
            nodal_floor_note(d.solver_label)
        );
    }
    if d.routing.k_ill_conditioned {
        println!("    K matrix: ill-conditioned (max|K| > 1e8, routed to nodal)");
    }
    if d.routing.s_ill_conditioned {
        println!("    S matrix: ill-conditioned (max|S| > 1e6, cap-only nodes)");
    }
    if d.n_linearized > 0 {
        println!(
            "    Linearized devices: {} (K/S magnitude guards bypassed)",
            d.n_linearized
        );
    }
}

/// The auto-tuned budget's note on the nodal route, which never ships less
/// than its floor.
fn nodal_floor_note(solver_label: &str) -> String {
    if solver_label == "nodal" {
        format!(
            "; nodal builds ship at least {}",
            melange_solver::codegen::policy::NODAL_MAX_ITER_FLOOR
        )
    } else {
        String::new()
    }
}

/// Whether the build summary names the DC operating point's railed op-amps:
/// always when they are pinned or the pin fell back.
fn meta_rail_pin_shown(label: &str) -> bool {
    !label.is_empty() && label != "none"
}

/// `melange compile`'s options (named, so two same-typed options cannot be
/// passed in each other's place).
pub(crate) struct CompileOptions<'a> {
    pub(crate) output: &'a PathBuf,
    pub(crate) sample_rate: f64,
    pub(crate) input_node: &'a str,
    pub(crate) output_node: &'a str,
    pub(crate) max_iter: Option<usize>,
    pub(crate) tolerance: f64,
    pub(crate) output_scale: f64,
    pub(crate) output_clamp: f64,
    pub(crate) format: OutputFormat,
    pub(crate) with_level_params: bool,
    pub(crate) input_resistance_flag: Option<f64>,
    pub(crate) oversampling_cli: Option<usize>,
    pub(crate) oversampling_set: melange_solver::build::OversamplingSet,
    pub(crate) no_dc_block: bool,
    pub(crate) solver_override: &'a str,
    pub(crate) backward_euler: bool,
    pub(crate) force_trap: bool,
    pub(crate) tube_grid_fa: &'a str,
    pub(crate) subsample_fire: melange_solver::codegen::SubsampleFireMode,
    pub(crate) subsample_lit_factor: Option<f64>,
    pub(crate) bjt_fa: &'a str,
    pub(crate) opamp_rail_mode: melange_solver::codegen::OpampRailMode,
    pub(crate) nodal_sub_path_override: melange_solver::codegen::NodalSubPathOverride,
    pub(crate) allow_static_glow_on_full_lu: bool,
    pub(crate) noise_mode: melange_solver::codegen::NoiseMode,
    pub(crate) noise_seed: u64,
    pub(crate) emit_dc_op_recompute: bool,
    pub(crate) allow_unconverged_dc_op: bool,
    /// Test-only DC-OP Newton budget (hidden `--dc-op-max-iterations`).
    pub(crate) dc_op_max_iterations: Option<usize>,
    pub(crate) plugin_name: Option<&'a str>,
    pub(crate) mono: bool,
    /// `--stereo`: a 1-output circuit as a stereo plugin, one circuit
    /// instance per channel.
    pub(crate) stereo: bool,
    pub(crate) wet_dry_mix: bool,
    pub(crate) ear_protection: bool,
    pub(crate) vendor: Option<&'a str>,
    pub(crate) vendor_url: Option<&'a str>,
    pub(crate) email: Option<&'a str>,
    pub(crate) vst3_id_override: Option<&'a str>,
    pub(crate) clap_id_override: Option<&'a str>,
    pub(crate) cpu_baseline: plugin_template::CpuBaseline,
    /// `-v/--verbose`: print the routing detail in the summary.
    pub(crate) verbose: bool,
}

pub(crate) fn compile_circuit_source(
    circuit_source: &circuits::CircuitSource,
    opts: CompileOptions<'_>,
) -> Result<()> {
    let CompileOptions {
        output,
        sample_rate,
        input_node,
        output_node,
        max_iter: max_iter_flag,
        tolerance,
        output_scale,
        output_clamp,
        format,
        with_level_params,
        input_resistance_flag,
        oversampling_cli,
        oversampling_set,
        no_dc_block,
        solver_override,
        backward_euler,
        force_trap,
        tube_grid_fa,
        subsample_fire,
        subsample_lit_factor,
        bjt_fa,
        opamp_rail_mode,
        nodal_sub_path_override,
        allow_static_glow_on_full_lu,
        noise_mode,
        noise_seed,
        emit_dc_op_recompute,
        allow_unconverged_dc_op,
        dc_op_max_iterations,
        plugin_name,
        mono,
        stereo,
        wet_dry_mix,
        ear_protection,
        vendor,
        vendor_url,
        email,
        vst3_id_override,
        clap_id_override,
        cpu_baseline,
        verbose,
    } = opts;
    if format == OutputFormat::Plugin {
        refuse_existing_plugin_project(&plugin_project_dir(output), &circuit_source.name())?;
    }
    // Netlist node names are normalized (lowercase, gnd→0) at parse time;
    // fold the CLI-provided names the same way so lookups match.
    // Parse comma-separated input nodes (multi-input ports), mirroring the
    // comma-separated output-node handling below. Port 0 (the first name) is the
    // "primary" input; any extras drive additional ports for M=0 (linear)
    // circuits. A single input name collapses to the historical single-input
    // path (byte-identical generated code).
    let input_node_names_owned: Vec<String> = input_node
        .split(',')
        .map(|s| melange_solver::parser::normalize_node_name(s.trim()))
        .filter(|s| !s.is_empty())
        .collect();
    if input_node_names_owned.is_empty() {
        anyhow::bail!("no input node specified");
    }
    let input_node_owned = input_node_names_owned[0].clone();
    let input_node = input_node_owned.as_str();
    let output_node_owned = melange_solver::parser::normalize_node_name(output_node);
    let output_node = output_node_owned.as_str();

    println!("melange compile");
    println!("  Source: {}", circuit_source.name());
    println!("  Output: {}", output.display());
    println!("  Sample rate: {} Hz", sample_rate);
    println!();

    let netlist_str = load_circuit_text(circuit_source, &|l| {
        if verbose || !is_build_detail(l) {
            println!("{l}")
        }
    })?;

    // --mono is incompatible with multiple output nodes: a multi-output
    // plugin takes mono input and routes each output node to its own audio
    // channel, so a 1-channel layout can't represent it. Erroring beats
    // silently generating a plugin whose second output node is inaudible.
    let output_node_names: Vec<&str> = output_node.split(',').map(|s| s.trim()).collect();
    if stereo {
        refuse_stereo_misuse(
            format == OutputFormat::Code,
            mono,
            &output_node_names,
            &circuit_source.name(),
        )?;
    }
    if mono && output_node_names.len() > 1 {
        anyhow::bail!(
            "--mono cannot be combined with multiple output nodes ({} given: \"{}\"). \
             Multi-output plugins route each output node to its own channel — \
             drop --mono, or pass a single --output-node.",
            output_node_names.len(),
            output_node_names.join(", ")
        );
    }
    // A plugin's audio layout is 1 channel (one output node) or 2 (two output
    // nodes, one per channel). With more nodes every one past the second had
    // no channel and was dropped without a word.
    if format == OutputFormat::Plugin && output_node_names.len() > 2 {
        anyhow::bail!(
            "{}",
            plugin_output_count_refusal(&output_node_names, &circuit_source.name())
        );
    }

    let circuit_name = output
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("circuit")
        .to_string();
    let circuit_name: String = circuit_name
        .to_lowercase()
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    let circuit_name = if circuit_name.starts_with(|c: char| c.is_ascii_digit()) {
        format!("circuit_{circuit_name}")
    } else {
        circuit_name
    };

    let build_printed = std::cell::Cell::new(false);
    // The one build every verb ships (melange_solver::build).
    let build_opts = melange_solver::build::BuildOptions {
        sample_rate,
        circuit_name,
        input_nodes: input_node_names_owned.clone(),
        output_nodes: output_node_names.iter().map(|s| s.to_string()).collect(),
        // `None` (no `--max-iter`) lets the build auto-tune the budget.
        max_iter: max_iter_flag,
        tolerance,
        output_scale,
        output_clamp,
        input_resistance: input_resistance_flag,
        oversampling: oversampling_cli,
        oversampling_set: oversampling_set.clone(),
        dc_block: !no_dc_block,
        solver: solver_override.to_string(),
        backward_euler,
        force_trap,
        tube_grid_fa: tube_grid_fa.to_string(),
        subsample_fire,
        // Flag wins; else the MELANGE_LIT_FACTOR env var (diagnostic); else 1.0.
        subsample_lit_factor: subsample_lit_factor.unwrap_or_else(diag_lit_factor),
        bjt_fa_mode: parse_bjt_fa_mode(bjt_fa),
        opamp_rail_mode,
        nodal_sub_path_override,
        allow_static_glow_on_full_lu,
        noise_mode,
        noise_seed,
        emit_dc_op_recompute,
        plugin_format: format != OutputFormat::Code,
        pot_overrides: None,
        resolve_taps: true,
        inject_runtime: true,
        disable_unit_variation: false,
        disable_self_heating: false,
        allow_unconverged_dc_op,
        dc_op_max_iterations,
        output_clamp_auto: false,
    };
    let melange_solver::build::Built {
        generated,
        netlist,
        mna,
        kernel,
        routing,
        solver_label,
        solver_reason,
        max_iter,
        oversampling,
        oversampling_set: runtime_set,
        input_resistance,
        input_resistance_source: ir_source,
        output_node_indices,
        forward_active,
        grid_off_pentodes,
        linearize_outcome,
        ..
    } = melange_solver::build::build(
        &netlist_str,
        &build_opts,
        &|a| {
            // Whether anything printed between the header and the summary,
            // which then gets a blank line of its own.
            let line = a.to_string();
            if verbose || !is_build_detail(&line) {
                build_printed.set(true);
                println!("{line}");
            }
        },
        &|a| {
            build_printed.set(true);
            eprintln!("{a}")
        },
    )
    .map_err(|e| build_error_in(e, &netlist_str))?;

    // A single output node makes a mono plugin, `--mono` or not, unless
    // `--stereo` asks for the template's other single-output layout: one
    // circuit instance per stereo channel, at twice the CPU. Guitar pedals are
    // mono, so stereo is opt-in.
    let mono =
        if !mono && !stereo && output_node_indices.len() == 1 && format == OutputFormat::Plugin {
            println!(
                "  Auto-selecting mono (single output node). For stereo, pass two output \
             nodes or --stereo."
            );
            build_printed.set(true);
            true
        } else {
            mono
        };
    // `--stereo` was refused above unless this is a 1-output plugin build.
    if stereo {
        println!(
            "  Stereo: two independent copies of the circuit, one per channel \
             (about twice the CPU of the mono plugin)."
        );
        build_printed.set(true);
    }

    let line_count = generated.code.lines().count();
    if verbose {
        println!("  ✓ Generated {} lines of Rust code", line_count);
    }

    // Compilation summary: report all auto-detected decisions in one place.
    if build_printed.get() {
        println!();
    }
    println!("  Summary:");
    if verbose {
        println!(
            "    System: {}",
            format_system_size(generated.n, mna.n, generated.m)
        );
    }
    if verbose {
        print_compile_route_detail(&RouteDetail {
            solver_label,
            solver_reason: &solver_reason,
            routing: &routing,
            meta: &generated.meta,
            max_iter,
            max_iter_pinned: max_iter_flag.is_some(),
            m: kernel.m,
            n_linearized: mna.linearized_triodes.len() + mna.linearized_bjts.len(),
        });
    } else {
        println!(
            "    {}",
            route_summary(
                solver_label,
                solver_override,
                generated.meta.nodal_sub_path,
                generated.meta.integrator_selection,
            )
        );
    }
    match &runtime_set {
        Some(set) => println!(
            "    Oversampling: {}× default, runtime-selectable {}",
            oversampling,
            set.factors
                .iter()
                .map(|f| format!("{f}×"))
                .collect::<Vec<_>>()
                .join("/")
        ),
        None => println!("    Oversampling: {}×", oversampling),
    }
    // A nonlinear circuit generates harmonics above Nyquist, and at 1x they
    // fold back into the audible band as inharmonic alias energy. The steady-
    // state frequency response does not show it, so a first-time author has no
    // way to discover the decision they just made by default.
    //
    // Fires ONLY when 1x was defaulted into — never when the author chose it on
    // the command line or in the deck. Telling someone about a decision they
    // already made is the noise that stops people reading notes at all.
    //
    // A NOTE, not a warning: 1x is correct for plenty of circuits (most of the
    // golden corpus runs there), and the amount that oversampling helps is
    // strongly circuit- and drive-dependent — so this points at the
    // measurement rather than prescribing a factor.
    if generated.m > 0
        && oversampling == 1
        && oversampling_cli.is_none()
        && netlist.recommended_oversampling.is_none()
    {
        println!(
            "    NOTE: {} nonlinear dimension(s) at 1× — harmonics above Nyquist fold",
            generated.m
        );
        println!("          back as aliasing, which the frequency response will not show.");
        println!(
            "          Measure it:  melange simulate <circuit> --input-audio <tone.wav> at 1× and"
        );
        println!(
            "                       --oversampling 4; compare the spectra (docs/OVERSAMPLING.md)"
        );
        println!(
            "          Set it:      --oversampling {{2|4}}, or `.oversampling N` in the deck."
        );
        println!("          Why it is not simply a quality dial: docs/OVERSAMPLING.md");
    }
    println!(
        "    Input: node \"{}\", resistance {}Ω ({})",
        input_node, input_resistance, ir_source
    );
    println!(
        "    Output: node \"{}\", scale {}, clamp ±{} V",
        output_node_names.join(", "),
        output_scale,
        output_clamp,
    );
    println!(
        "    DC block: {}",
        if !no_dc_block {
            "enabled (5 Hz HPF)"
        } else {
            "disabled"
        }
    );
    // DC operating point
    if generated.m > 0 {
        let meta = &generated.meta;
        if meta.dc_op_converged {
            println!(
                "    DC operating point: converged ({}, {} iterations)",
                meta.dc_op_method, meta.dc_op_iterations
            );
        } else {
            println!(
                "    DC operating point: *** DID NOT CONVERGE *** ({}, {} iterations)",
                meta.dc_op_method, meta.dc_op_iterations
            );
        }
        if !meta.parasitic_caps.is_empty() {
            println!(
                "    Parasitic caps: {} x 10 pF auto-inserted across device junctions (no \
                 capacitors in circuit)",
                meta.parasitic_caps.len()
            );
        }
    }
    if meta_rail_pin_shown(&generated.meta.dc_op_rail_pin) {
        println!(
            "    Railed op-amps at DC: {}",
            generated.meta.dc_op_rail_pin
        );
    }
    if !forward_active.is_empty() {
        println!(
            "    Forward-active reduction: {} BJTs linearized to 1D",
            forward_active.len()
        );
    }
    if !grid_off_pentodes.is_empty() {
        println!(
            "    Grid-off reduction: {} pentodes reduced to 2D",
            grid_off_pentodes.len()
        );
    }
    if linearize_outcome.bjts_linearized > 0 {
        println!(
            "    Linearized BJTs: {} (fully linear, M reduced by {})",
            linearize_outcome.bjts_linearized,
            linearize_outcome.bjts_linearized * 2
        );
    }
    if linearize_outcome.triodes_linearized > 0 {
        println!(
            "    Linearized triodes: {} (fully linear, M reduced by {})",
            linearize_outcome.triodes_linearized,
            linearize_outcome.triodes_linearized * 2
        );
    }
    println!("    Generated: {} lines of Rust", line_count);
    println!();

    // Step 5 (the build printed 1-4 under -v): write output
    if verbose {
        println!("Step 5: Writing output...");
    }

    match format {
        OutputFormat::Code => {
            // Write single file (existing behavior)
            std::fs::write(output, &generated.code)
                .with_context(|| format!("Failed to write output file: {}", output.display()))?;

            println!("  ✓ Done!");
            println!();
            println!("Generated code written to: {}", output.display());
            println!("This code can be used with:");
            println!("  - Standalone integration in your own projects");
            println!("  - A CLAP/VST3 plugin project, generated with `--format plugin` (below)");
            println!();
            // The emitted file is the default output of `compile`, and its
            // caller-facing API (Default constructor, free `process_sample`,
            // pot/switch setters, output units) is not guessable from the
            // filename. Point at the doc here, where the reader actually is.
            println!("Its API is documented in docs/CODE_API.md:");
            println!("  let mut state = circuit::CircuitState::default();");
            // The rate the code was compiled at, as a Rust f64 literal.
            println!("  state.set_sample_rate({sample_rate:?});");
            if generated
                .code
                .contains("pub fn process_sample(input: f64, injections_host:")
            {
                // `.inject`/`.tap` decks: host-rate and inner-rate injection
                // arrays in, `(outputs, raw taps)` out.
                println!("  let (out, taps) = circuit::process_sample(input, &injections_host, &injections_inner, &mut state); // .inject/.tap API: see its doc comment");
            } else {
                println!("  let out = circuit::process_sample(input, &mut state); // free fn -> [f64; NUM_OUTPUTS], volts");
            }
            println!();
            // Suggest a DIRECTORY, not this file path. `--format plugin`
            // treats --output as the project root; echoing back
            // `.../src/circuit.rs` (which is what this hint used to do) sends
            // the reader to `cargo new` a project inside another project's
            // src/, named after a file. Derive a plain project name from the
            // circuit instead.
            let project_suggestion = suggest_plugin_project_dir(&circuit_source.name());
            println!("To generate a complete plugin project instead, use:");
            println!(
                "  melange compile {} --output {} --format plugin",
                circuit_source.name(),
                project_suggestion
            );
            println!("  (with --format plugin, --output is the project DIRECTORY, not a file)");
        }
        OutputFormat::Plugin => {
            // Generate complete plugin project
            let project_dir = plugin_project_dir(output);

            // Get a sanitized circuit name
            let circuit_name: String = project_dir
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("circuit")
                .to_lowercase()
                .chars()
                .map(|c| {
                    if c.is_ascii_alphanumeric() || c == '_' {
                        c
                    } else {
                        '_'
                    }
                })
                .collect();
            let circuit_name = if circuit_name.starts_with(|c: char| c.is_ascii_digit()) {
                format!("circuit_{circuit_name}")
            } else {
                circuit_name
            };

            // Build pot/switch parameter info from MNA data (works for both DK and nodal paths).
            // Skip entries declared via `.runtime R` — those are driven by plugin-side
            // envelope followers via `set_runtime_R_<field>` and do not get nih-plug
            // knobs. The netlist.pots index lookup stays aligned because `.pot`
            // directives are pushed into mna.pots BEFORE `.runtime R` directives.
            let pot_params: Vec<plugin_template::PotParamInfo> = mna
                .pots
                .iter()
                .enumerate()
                .filter(|(_, p)| p.runtime_field.is_none())
                .map(|(idx, p)| plugin_template::PotParamInfo {
                    index: idx,
                    name: netlist
                        .pots
                        .get(idx)
                        .map(|d| d.label.clone().unwrap_or_else(|| d.resistor_name.clone()))
                        .unwrap_or_else(|| format!("Pot {}", idx)),
                    min_resistance: p.min_resistance,
                    max_resistance: p.max_resistance,
                    default_resistance: netlist
                        .pots
                        .get(idx)
                        .and_then(|d| d.default_value)
                        .unwrap_or(1.0 / p.g_nominal),
                })
                .collect();
            let switch_params: Vec<plugin_template::SwitchParamInfo> = netlist
                .switches
                .iter()
                .enumerate()
                .map(|(idx, sw)| plugin_template::SwitchParamInfo {
                    index: idx,
                    name: sw.label.clone().unwrap_or_else(|| {
                        format!("Switch {} ({})", idx, sw.component_names.join("+"))
                    }),
                    num_positions: sw.positions.len(),
                })
                .collect();

            // Build wiper params from MNA wiper groups
            let wiper_params: Vec<plugin_template::WiperParamInfo> = mna
                .wiper_groups
                .iter()
                .enumerate()
                .map(|(idx, wg)| plugin_template::WiperParamInfo {
                    wiper_index: idx,
                    cw_pot_index: wg.cw_pot_index,
                    ccw_pot_index: wg.ccw_pot_index,
                    total_resistance: wg.total_resistance,
                    default_position: wg.default_position,
                    name: wg.label.clone().unwrap_or_else(|| format!("Wiper {}", idx)),
                })
                .collect();

            // Build gang params from MNA gang groups
            let gang_params: Vec<plugin_template::GangParamInfo> = mna
                .gang_groups
                .iter()
                .enumerate()
                .map(|(idx, gg)| plugin_template::GangParamInfo {
                    index: idx,
                    label: gg.label.clone(),
                    default_position: gg.default_position,
                    pot_members: gg
                        .pot_members
                        .iter()
                        .map(|&(pot_idx, inverted)| {
                            (
                                pot_idx,
                                mna.pots[pot_idx].min_resistance,
                                mna.pots[pot_idx].max_resistance,
                                inverted,
                            )
                        })
                        .collect(),
                    wiper_members: gg
                        .wiper_members
                        .iter()
                        .map(|&(wg_idx, inverted)| {
                            let wg = &mna.wiper_groups[wg_idx];
                            (
                                wg.cw_pot_index,
                                wg.ccw_pot_index,
                                wg.total_resistance,
                                inverted,
                            )
                        })
                        .collect(),
                })
                .collect();

            // Collect pot indices claimed by gangs
            let gang_claimed_pots: std::collections::HashSet<usize> = gang_params
                .iter()
                .flat_map(|g| g.pot_members.iter().map(|&(idx, _, _, _)| idx))
                .collect();
            let gang_claimed_wipers: std::collections::HashSet<usize> = gang_params
                .iter()
                .flat_map(|g| {
                    g.wiper_members
                        .iter()
                        .flat_map(|&(cw, ccw, _, _)| [cw, ccw])
                })
                .collect();

            // Filter out wiper-claimed and gang-claimed pots from individual pot params
            let wiper_claimed: std::collections::HashSet<usize> = wiper_params
                .iter()
                .flat_map(|w| [w.cw_pot_index, w.ccw_pot_index])
                .collect();
            let pot_params: Vec<_> = pot_params
                .into_iter()
                .filter(|p| {
                    !wiper_claimed.contains(&p.index) && !gang_claimed_pots.contains(&p.index)
                })
                .collect();

            // Filter out gang-claimed wipers from individual wiper params
            let wiper_params: Vec<_> = wiper_params
                .into_iter()
                .filter(|w| !gang_claimed_wipers.contains(&w.cw_pot_index))
                .collect();

            let plugin_options = plugin_template::PluginOptions {
                plugin_name,
                mono,
                // Two instances of one circuit need their own noise streams;
                // only a circuit with runtime noise has `set_seed` to call.
                per_channel_noise_seeds: stereo && circuit_has_runtime_noise(&generated.code),
                // `--noise` asked for noise: give the plugin its on/off switch
                // (otherwise the circuit's noise stays off, silent).
                circuit_noise_param: circuit_has_runtime_noise(&generated.code),
                wet_dry_mix,
                ear_protection,
                vendor,
                url: vendor_url,
                email,
                vst3_id: vst3_id_override,
                clap_id: clap_id_override,
                cpu_baseline,
            };
            plugin_template::generate_plugin_project_with_oversampling(
                &project_dir,
                &generated.code,
                &circuit_name,
                with_level_params,
                &pot_params,
                &wiper_params,
                &gang_params,
                &switch_params,
                output_node_indices.len(),
                oversampling,
                &plugin_options,
            )?;

            println!("  ✓ Done!");
            println!();
            println!("Generated plugin project at: {}", project_dir.display());
            println!();
            let dir_name = project_dir
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("<project-dir>");
            println!("Build the DSP library (works immediately):");
            println!("  cd {dir_name}");
            println!("  cargo build --release          # → raw plugin lib in target/release/");
            println!();
            println!("For a DAW-loadable CLAP + VST3 bundle (no nih-plug clone needed):");
            println!("  cd {dir_name}");
            println!("  bash build.sh                  # → CLAP + VST3 in target/bundled/");
            println!("  # (must be OUTSIDE any Cargo workspace — see the warning above if any)");
            println!();
            println!("See the generated README.md for full details.");
        }
    }

    Ok(())
}

/// Refuse `--stereo` anywhere but a one-output `--format plugin` build, saying
/// what to do instead.
fn refuse_stereo_misuse(
    code_format: bool,
    mono: bool,
    output_node_names: &[&str],
    circuit: &str,
) -> Result<()> {
    if code_format {
        anyhow::bail!(
            "--stereo applies to --format plugin only: it makes the plugin run two copies \
             of the circuit, one per channel. Generated code has no channels; for stereo, \
             create one CircuitState per channel yourself (with --noise, give each its own \
             nonzero set_seed so their noise is independent), or generate the plugin:\n  \
             melange compile {circuit} --format plugin --stereo -o <project-dir>"
        );
    }
    if mono {
        anyhow::bail!(
            "--stereo and --mono contradict each other: --stereo makes a 2-channel plugin \
             by running two copies of the circuit, --mono a 1-channel one. Pass one of them."
        );
    }
    if output_node_names.len() > 1 {
        anyhow::bail!(
            "--stereo cannot be combined with multiple output nodes ({} given: \"{}\"). \
             Two output nodes already make a stereo plugin, one node per channel; \
             --stereo is for a circuit with ONE output node, which it runs twice. \
             Drop --stereo, or pass a single --output-node.",
            output_node_names.len(),
            output_node_names.join(", ")
        );
    }
    Ok(())
}

/// Whether generated circuit code carries runtime noise, i.e. has the
/// `CircuitState::set_seed` that `--stereo` calls to give each channel its own
/// noise and the `set_noise_enabled` the plugin's "Circuit Noise" parameter
/// calls. Codegen emits both exactly when `--noise` is on and the circuit has
/// a noise source.
fn circuit_has_runtime_noise(code: &str) -> bool {
    code.contains("pub const NOISE_MASTER_SEED_DEFAULT: u64")
        && code.contains("pub fn set_seed(&mut self, master: u64)")
        && code.contains("pub fn set_noise_enabled(&mut self, on: bool)")
}

/// The refusal for `--format plugin` with more than two output nodes.
fn plugin_output_count_refusal(output_node_names: &[&str], circuit: &str) -> String {
    let n = output_node_names.len();
    format!(
        "--format plugin supports 1 output node (a mono plugin) or 2 (a stereo plugin, \
         one node per channel); {n} given: {nodes}. A plugin has no channel for the \
         nodes past the second, so they would be inaudible.\n\
         Choose two of them:\n  \
         melange compile {circuit} --format plugin -n <node_a>,<node_b> -o <project-dir>\n\
         or generate the code, whose process_sample returns all {n} outputs:\n  \
         melange compile {circuit} --format code -n {joined} -o <file.rs>",
        nodes = output_node_names
            .iter()
            .map(|s| format!("\"{s}\""))
            .collect::<Vec<_>>()
            .join(", "),
        joined = output_node_names.join(","),
    )
}

/// The project root `--format plugin` writes for `--output`: the path with
/// any extension dropped.
fn plugin_project_dir(output: &Path) -> PathBuf {
    if output.extension().is_some() {
        output.with_extension("")
    } else {
        output.to_path_buf()
    }
}

/// Refuse to generate a plugin project over an existing one. `src/lib.rs` is
/// the user's (README: "Yes — it's yours"), and regenerating the project
/// replaced it, and any `Cargo.toml` edits, without a word.
fn refuse_existing_plugin_project(project_dir: &Path, circuit: &str) -> Result<()> {
    let existing: Vec<&str> = ["src/lib.rs", "Cargo.toml"]
        .into_iter()
        .filter(|f| project_dir.join(f).exists())
        .collect();
    if existing.is_empty() {
        return Ok(());
    }
    anyhow::bail!(
        "refusing to overwrite the plugin project in {dir}: it already has {files}, which \
         are yours to edit, and generating the project again would replace them.\n\
         To update only the DSP, leaving your edits alone:\n  \
         melange compile {circuit} --format code -o {circuit_rs}\n\
         To start the project over, delete or move {dir} first.",
        dir = project_dir.display(),
        files = existing.join(" and "),
        circuit_rs = project_dir.join("src/circuit.rs").display(),
    )
}

/// A plausible project DIRECTORY name for the `--format plugin` hint.
///
/// Derived from the circuit, never from the `--format code` output path: that
/// path is a `.rs` FILE (`.../src/circuit.rs` in the common regenerate loop),
/// and `--format plugin` interprets `--output` as the project root. The old
/// hint echoed the file path straight back, so following it verbatim tried to
/// create a Cargo project inside another project's `src/`.
///
/// Falls back to `melange-plugin` when the source has no usable stem (a URL
/// with a trailing slash, a builtin reference, …).
pub(crate) fn suggest_plugin_project_dir(circuit_source_name: &str) -> String {
    let stem = std::path::Path::new(circuit_source_name)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("");
    // Builtin / source references look like "source:circuit" — take the tail.
    let stem = stem.rsplit(':').next().unwrap_or(stem);
    let cleaned: String = stem
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '-'
            }
        })
        .collect();
    let cleaned = cleaned.trim_matches('-').to_string();
    if cleaned.is_empty() {
        "melange-plugin".to_string()
    } else {
        format!("{cleaned}-plugin")
    }
}
