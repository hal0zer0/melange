use crate::args::parse_bjt_fa_mode;
use crate::circuits;
use crate::common::{
    build_error_in, diag_lit_factor, is_build_detail, load_circuit_text, report_build_line,
};
use anyhow::Result;
use melange_solver::build::format_system_size;

/// Human-readable name for a DC-system row index reported by
/// `DcOpResult::kcl_worst_row`: the node name for circuit nodes, otherwise
/// the raw row (BJT internal nodes added by the DC solver).
fn dc_op_row_name(row: usize, idx_to_name: &[String], n: usize) -> String {
    if row < n && row + 1 < idx_to_name.len() {
        format!("v({})", idx_to_name[row + 1])
    } else {
        format!("dc row {} (internal node)", row)
    }
}

/// Options for `melange dc-op` (see `Commands::DcOp`).
pub(crate) struct DcOpOptions<'a> {
    pub(crate) input_node: &'a str,
    pub(crate) input_resistance: Option<f64>,
    pub(crate) format: &'a str,
    pub(crate) sample_rate: f64,
    pub(crate) oversampling: Option<usize>,
    pub(crate) solver: &'a str,
    pub(crate) opamp_rail_mode: melange_solver::codegen::OpampRailMode,
    pub(crate) bjt_fa: &'a str,
    pub(crate) tube_grid_fa: &'a str,
    pub(crate) pot_overrides: &'a [String],
    pub(crate) allow_unconverged_dc_op: bool,
    /// Test-only DC-OP Newton budget (hidden `--dc-op-max-iterations`).
    pub(crate) dc_op_max_iterations: Option<usize>,
    /// `-v/--verbose`: print the build steps, system size and routing reason.
    pub(crate) verbose: bool,
}

/// `melange dc-op`: the operating point the build ships. The circuit is
/// assembled by the one build every verb uses ([`melange_solver::build::assemble`],
/// up to the IR, where the route and the operating point are settled), so
/// the vector printed is the one the generated code embeds as `DC_OP`.
pub(crate) fn run_dc_op(
    circuit_source: &circuits::CircuitSource,
    opts: &DcOpOptions<'_>,
) -> Result<()> {
    use melange_solver::codegen::ir::CircuitIR;

    let verbose = opts.verbose;
    let human = opts.format != "json";
    if human {
        eprintln!("melange dc-op");
        eprintln!("  Source: {}", circuit_source.name());
    }
    // stdout is the report (JSON-clean), so the loader's lines go to stderr.
    let netlist_str = load_circuit_text(circuit_source, &|l| {
        if verbose || !is_build_detail(l) {
            eprintln!("{l}")
        }
    })?;

    let d = melange_solver::codegen::CodegenConfig::default();
    let build_opts = melange_solver::build::BuildOptions {
        sample_rate: opts.sample_rate,
        circuit_name: "dc_op".to_string(),
        // Match parse-time node normalization (lowercase, gnd→0).
        input_nodes: vec![melange_solver::parser::normalize_node_name(opts.input_node)],
        // An operating point has no output.
        output_nodes: Vec::new(),
        max_iter: None,
        tolerance: d.tolerance,
        output_scale: 1.0,
        output_clamp: d.output_clamp_v,
        input_resistance: opts.input_resistance,
        oversampling: opts.oversampling,
        oversampling_set: melange_solver::build::OversamplingSet::Off,
        dc_block: true,
        solver: opts.solver.to_string(),
        backward_euler: false,
        force_trap: false,
        tube_grid_fa: opts.tube_grid_fa.to_string(),
        subsample_fire: d.subsample_fire,
        subsample_lit_factor: diag_lit_factor(),
        bjt_fa_mode: parse_bjt_fa_mode(opts.bjt_fa),
        opamp_rail_mode: opts.opamp_rail_mode,
        nodal_sub_path_override: d.nodal_sub_path_override,
        allow_static_glow_on_full_lu: false,
        noise_mode: d.noise_mode,
        noise_seed: d.noise_master_seed,
        emit_dc_op_recompute: false,
        plugin_format: false,
        // Without --pot the deck's `.pot` defaults, as `compile` builds it.
        pot_overrides: if opts.pot_overrides.is_empty() {
            None
        } else {
            Some(opts.pot_overrides.to_vec())
        },
        resolve_taps: true,
        inject_runtime: true,
        disable_unit_variation: false,
        disable_self_heating: false,
        allow_unconverged_dc_op: opts.allow_unconverged_dc_op,
        dc_op_max_iterations: opts.dc_op_max_iterations,
        output_clamp_auto: false,
    };
    // The build's own lines go to stderr: stdout is the report (JSON-clean).
    let assembled = melange_solver::build::assemble(
        &netlist_str,
        &build_opts,
        &|a| report_build_line(verbose, a, |l| eprintln!("{l}")),
        &|a| eprintln!("{a}"),
    )
    .map_err(|e| build_error_in(e, &netlist_str))?;
    let format = opts.format;
    let result = &assembled.dc_op;
    let mna = &assembled.mna;
    let device_slots =
        CircuitIR::build_device_info_with_mna(&assembled.netlist, Some(mna)).unwrap_or_default();
    let route = assembled.solver_label;

    // Build reverse node map (index → name)
    let mut idx_to_name: Vec<String> = vec!["0".to_string(); mna.n + 1];
    for (name, &idx) in &mna.node_map {
        if idx <= mna.n {
            idx_to_name[idx] = name.clone();
        }
    }

    if format == "json" {
        // JSON output for machine consumption, every number in its shortest
        // exact (round-trip) form: a consumer comparing two operating points
        // must not see a real difference rounded away.
        print!("{{");
        print!("\"converged\":{},", result.converged);
        print!("\"method\":\"{:?}\",", result.method);
        print!("\"iterations\":{},", result.iterations);
        print!("\"kcl_residual_max\":{:e},", result.kcl_residual_max);
        print!(
            "\"kcl_worst_row\":{},",
            match result.kcl_worst_row {
                Some(row) => format!("\"{}\"", dc_op_row_name(row, &idx_to_name, mna.n)),
                None => "null".to_string(),
            }
        );
        print!(
            "\"rail_pin\":\"{}\",",
            result
                .rail_pin
                .label()
                .replace('\\', "\\\\")
                .replace('"', "\\\"")
        );
        print!("\"solver\":\"{}\",", route);
        print!("\"n\":{},\"m\":{},", mna.n, mna.m);

        // Node voltages
        print!("\"nodes\":{{");
        let mut nodes: Vec<_> = mna.node_map.iter().collect();
        nodes.sort_by(|a, b| a.1.cmp(b.1));
        let mut first = true;
        for (name, &idx) in &nodes {
            if idx > 0 && idx <= result.v_node.len() {
                if !first {
                    print!(",");
                }
                first = false;
                print!("\"{}\":{:e}", name, result.v_node[idx - 1]);
            }
        }
        print!("}}");

        // Nonlinear device currents
        if !result.i_nl.is_empty() {
            print!(",\"devices\":{{");
            let mut first = true;
            for (i, slot) in device_slots.iter().enumerate() {
                let s = slot.start_idx;
                let dev_name = mna
                    .nonlinear_devices
                    .get(i)
                    .map(|d| d.name.as_str())
                    .unwrap_or("?");
                for d in 0..slot.dimension {
                    if s + d < result.i_nl.len() {
                        if !first {
                            print!(",");
                        }
                        first = false;
                        print!(
                            "\"{}[{}]\":{{\"v_nl\":{:e},\"i_nl\":{:e}}}",
                            dev_name,
                            d,
                            result.v_nl[s + d],
                            result.i_nl[s + d]
                        );
                    }
                }
            }
            print!("}}");
        }

        println!("}}");
    } else {
        // Human-readable output (the banner printed before the build).
        if verbose {
            eprintln!("  {}", format_system_size(mna.n, mna.n, mna.m));
            eprintln!("  Solver: {route} ({})", assembled.solver_reason);
            if route != "DK" {
                // The router's reasons are written for maintainers ("DK K
                // matrix unstable"); on the default route they explain the
                // choice, they do not report a fault.
                eprintln!(
                    "    (routing information, not a warning: the reason says why the DK \
                     route was not the fit for this circuit)"
                );
            }
        } else {
            let chosen_by = if opts.solver == "auto" {
                "chosen automatically".to_string()
            } else {
                format!("forced by --solver {}", opts.solver)
            };
            eprintln!("  Solver: {route}, {chosen_by}. (-v for why)");
        }
        eprintln!(
            "  Converged: {} ({:?}, {} iterations)",
            result.converged, result.method, result.iterations
        );
        if result.rail_pin != melange_solver::dc_op::RailPin::None {
            eprintln!("  Railed op-amps: {}", result.rail_pin.label());
        }
        match result.kcl_worst_row {
            Some(row) => eprintln!(
                "  KCL residual: max |F| = {:.3e} A at {}",
                result.kcl_residual_max,
                dc_op_row_name(row, &idx_to_name, mna.n)
            ),
            None => eprintln!("  KCL residual: n/a (no voltage rows)"),
        }
        eprintln!();

        // Node voltages
        println!("Node voltages:");
        let mut nodes: Vec<_> = mna.node_map.iter().collect();
        nodes.sort_by(|a, b| a.1.cmp(b.1));
        for (name, &idx) in &nodes {
            if idx > 0 && idx <= result.v_node.len() {
                let v = result.v_node[idx - 1];
                if v.abs() > 0.1 {
                    println!("  v({}) = {:.4} V", name, v);
                } else if v.abs() > 1e-6 {
                    println!("  v({}) = {:.4} mV", name, v * 1e3);
                } else {
                    println!("  v({}) = {:.4e} V", name, v);
                }
            }
        }

        // Nonlinear device operating points
        if !result.i_nl.is_empty() {
            println!();
            println!("Device operating points:");
            for (i, slot) in device_slots.iter().enumerate() {
                let s = slot.start_idx;
                let dev_name = mna
                    .nonlinear_devices
                    .get(i)
                    .map(|d| d.name.as_str())
                    .unwrap_or("?");
                println!("  {} ({:?}):", dev_name, slot.device_type);
                for d in 0..slot.dimension {
                    if s + d < result.i_nl.len() {
                        let v = result.v_nl[s + d];
                        let i = result.i_nl[s + d];
                        if i.abs() > 1e-3 {
                            println!("    [{}] v_nl={:.4} V, i_nl={:.4} mA", d, v, i * 1e3);
                        } else if i.abs() > 1e-6 {
                            println!("    [{}] v_nl={:.4} V, i_nl={:.4} uA", d, v, i * 1e6);
                        } else {
                            println!("    [{}] v_nl={:.4} V, i_nl={:.4e} A", d, v, i);
                        }
                    }
                }
            }
        }
    }

    Ok(())
}
