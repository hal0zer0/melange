//! The one build: netlist text → generated code, as every verb ships it.
//!
//! `melange compile`, `simulate`, `analyze` and `melange-validate` each used to
//! assemble the [`crate::pipeline`] steps in their own sequence, and the four
//! sequences disagreed (the DK-kernel failure fallback, the Newton budget,
//! `.inject` stamping, stamp order, junction caps versus forward-active
//! detection), so a verb other than `compile` could check a different circuit
//! from the one `compile` ships. This module is the single sequence. What a verb
//! legitimately varies is a field of [`BuildOptions`], visible in one signature.
//!
//! Diagnostics go through two [`Reporter`]s (the lines `compile` prints on
//! stdout and on stderr) so a library caller can stay silent.

use crate::codegen::{CodeGenerator, CodegenConfig};
use crate::dk::DkKernel;
use crate::mna::MnaSystem;
use crate::parser::Netlist;
use crate::pipeline::Reporter;

/// Route a diagnostic line to a reporter (same shape as `pipeline`'s macro).
macro_rules! report {
    ($rep:expr, $($arg:tt)*) => { ($rep)(format_args!($($arg)*)) };
}
/// Build a [`BuildError`] from a format string.
macro_rules! err {
    ($($arg:tt)*) => { BuildError::msg(format!($($arg)*)) };
}
/// Return a [`BuildError`] from a format string.
macro_rules! bail {
    ($($arg:tt)*) => { return Err(err!($($arg)*)) };
}

/// Why a build failed: an optional context line over the underlying error,
/// the shape `anyhow` shows as "context / Caused by: source".
#[derive(Debug)]
pub struct BuildError {
    context: Option<String>,
    source: Box<dyn std::error::Error + Send + Sync + 'static>,
}

impl BuildError {
    fn msg(message: String) -> Self {
        BuildError {
            context: None,
            source: message.into(),
        }
    }
    /// The context line, if the failing step added one.
    pub fn context(&self) -> Option<&str> {
        self.context.as_deref()
    }
    /// The underlying error.
    pub fn into_parts(
        self,
    ) -> (
        Option<String>,
        Box<dyn std::error::Error + Send + Sync + 'static>,
    ) {
        (self.context, self.source)
    }
}

impl std::fmt::Display for BuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.context {
            Some(c) => write!(f, "{c}: {}", self.source),
            None => write!(f, "{}", self.source),
        }
    }
}

impl<E: std::error::Error + Send + Sync + 'static> From<E> for BuildError {
    fn from(e: E) -> Self {
        BuildError {
            context: None,
            source: Box::new(e),
        }
    }
}

/// `anyhow`-style `.with_context` for the steps of [`build`].
trait WithContext<T> {
    fn with_context<C: std::fmt::Display, F: FnOnce() -> C>(self, f: F) -> Result<T, BuildError>;
}

impl<T, E: std::error::Error + Send + Sync + 'static> WithContext<T> for Result<T, E> {
    fn with_context<C: std::fmt::Display, F: FnOnce() -> C>(self, f: F) -> Result<T, BuildError> {
        self.map_err(|e| BuildError {
            context: Some(f().to_string()),
            source: Box::new(e),
        })
    }
}

/// Everything a verb chooses about a build. The defaults are `compile`'s.
#[derive(Debug, Clone)]
pub struct BuildOptions {
    /// Host sample rate (Hz).
    pub sample_rate: f64,
    /// Generated module's circuit name.
    pub circuit_name: String,
    /// Input nodes, normalized; the first is the primary audio input, any
    /// others are extra input ports (linear circuits only).
    pub input_nodes: Vec<String>,
    /// Output nodes, normalized, in `process_sample` output order.
    pub output_nodes: Vec<String>,
    /// The user's Newton budget, or `None` to auto-tune.
    pub max_iter: Option<usize>,
    /// Newton tolerance.
    pub tolerance: f64,
    /// Output scale, broadcast to every output.
    pub output_scale: f64,
    /// Output clamp (V).
    pub output_clamp: f64,
    /// Input source resistance; `None` = the deck's `.input_impedance`, else 1 Ω.
    pub input_resistance: Option<f64>,
    /// Explicit oversampling; `None` = the deck's `.oversampling`, else 1.
    pub oversampling: Option<usize>,
    /// The generated 5 Hz DC blocker.
    pub dc_block: bool,
    /// `"auto"`, `"dk"` or `"nodal"`.
    pub solver: String,
    /// `--backward-euler`.
    pub backward_euler: bool,
    /// `--force-trap`.
    pub force_trap: bool,
    /// `--tube-grid-fa`: `"auto"`, `"on"` or `"off"`.
    pub tube_grid_fa: String,
    /// `--subsample-fire`.
    pub subsample_fire: crate::codegen::SubsampleFireMode,
    /// Lit sub-step multiplier (resolved by the caller).
    pub subsample_lit_factor: f64,
    /// `--bjt-fa`.
    pub bjt_fa_mode: crate::codegen::BjtFaMode,
    /// `--opamp-rail-mode`.
    pub opamp_rail_mode: crate::codegen::OpampRailMode,
    /// `--nodal-subpath`.
    pub nodal_sub_path_override: crate::codegen::NodalSubPathOverride,
    /// `--allow-static-glow-on-full-lu`.
    pub allow_static_glow_on_full_lu: bool,
    /// Noise mode.
    pub noise_mode: crate::codegen::NoiseMode,
    /// Noise master seed.
    pub noise_seed: u64,
    /// Emit the runtime DC-OP recompute.
    pub emit_dc_op_recompute: bool,
    /// The build feeds a plugin project (not `--format code`): refuses the
    /// features only `--format code` carries (multi-input, `.inject`/`.tap`).
    pub plugin_format: bool,
    /// `--pot NAME=VALUE` overrides, applied to the netlist before the MNA is
    /// built, which also settles every other pot onto its `.pot` default
    /// ([`apply_pot_overrides`]). `None` leaves the netlist as written (the MNA
    /// applies `.pot` defaults itself).
    pub pot_overrides: Option<Vec<String>>,
    /// Resolve `.tap` probes (they change the generated `process_sample` API).
    pub resolve_taps: bool,
    /// Give the generated code the `.inject` runtime API. Either way every
    /// `.inject` source's conductance is stamped (it is part of the circuit);
    /// without the API each injection holds 0 (a verb whose driver calls the
    /// plain `process_sample`, e.g. `analyze`).
    pub inject_runtime: bool,
    /// Parse with `.tolerance` / `.mismatch` jitter off (validate: the reference
    /// deck carries the values as written).
    pub disable_unit_variation: bool,
    /// Build even when the DC operating point did not converge
    /// (`--allow-unconverged-dc-op`). By default such a build is refused: its
    /// generated code would start from a state that is not a solution.
    pub allow_unconverged_dc_op: bool,
    /// TEST-ONLY DC-OP Newton budget: see
    /// [`crate::codegen::CodegenConfig::dc_op_max_iterations`].
    #[doc(hidden)]
    pub dc_op_max_iterations: Option<usize>,
    /// Raise the output clamp to three times the largest DC operating-point
    /// node voltage when that is higher (validate: a high-rail circuit swings
    /// its output tens of volts legitimately; a 10 V clamp would square it).
    pub output_clamp_auto: bool,
}

/// An assembled build: everything [`build`] ships but the emitted code.
pub struct Assembled {
    /// The codegen configuration the IR was built with.
    pub config: CodegenConfig,
    /// The IR, route settled.
    pub prepared: crate::codegen::PreparedIr,
    /// The DC operating point the build ships (`DC_OP`), solved on `mna`.
    pub dc_op: crate::dc_op::DcOpResult,
    pub netlist: Netlist,
    /// The MNA the IR was built from (on the nodal route, internal nodes
    /// expanded).
    pub mna: MnaSystem,
    pub kernel: DkKernel,
    pub routing: crate::codegen::routing::RoutingDecision,
    /// `"nodal"` or `"DK"`.
    pub solver_label: &'static str,
    pub solver_reason: String,
    /// The Newton budget that ships.
    pub max_iter: usize,
    pub oversampling: usize,
    pub input_node_idx: usize,
    pub input_resistance: f64,
    /// Where the input resistance came from.
    pub input_resistance_source: &'static str,
    pub output_node_indices: Vec<usize>,
    pub forward_active: std::collections::HashSet<String>,
    pub grid_off_pentodes: std::collections::HashMap<String, f64>,
    pub linearize_outcome: crate::pipeline::LinearizeOutcome,
    /// The resolved `.inject` sources, in generated-code order.
    pub injection_specs: Vec<crate::codegen::ir::InjectionSpec>,
}

/// A finished build and what the caller reports about it.
pub struct Built {
    pub generated: crate::codegen::GeneratedCode,
    /// The DC operating point the generated code embeds (`DC_OP`).
    pub dc_op: crate::dc_op::DcOpResult,
    pub netlist: Netlist,
    pub mna: MnaSystem,
    pub kernel: DkKernel,
    pub routing: crate::codegen::routing::RoutingDecision,
    /// `"nodal"` or `"DK"`.
    pub solver_label: &'static str,
    pub solver_reason: String,
    /// The Newton budget that ships.
    pub max_iter: usize,
    pub oversampling: usize,
    pub input_node_idx: usize,
    pub input_resistance: f64,
    /// Where the input resistance came from.
    pub input_resistance_source: &'static str,
    pub output_node_indices: Vec<usize>,
    pub forward_active: std::collections::HashSet<String>,
    pub grid_off_pentodes: std::collections::HashMap<String, f64>,
    pub linearize_outcome: crate::pipeline::LinearizeOutcome,
    /// The resolved `.inject` sources, in generated-code order.
    pub injection_specs: Vec<crate::codegen::ir::InjectionSpec>,
}

/// Assemble `netlist_str` as every verb ships it, up to the IR: the circuit,
/// its route and the DC operating point it ships. [`build`] emits the code;
/// `melange dc-op` reports the operating point. `opts.output_nodes` may be
/// empty here (no output is needed to reach the IR).
pub fn assemble(
    netlist_str: &str,
    opts: &BuildOptions,
    out: Reporter<'_>,
    err: Reporter<'_>,
) -> Result<Assembled, BuildError> {
    // Each model warning once per build, however many steps resolve the model.
    let _warnings = crate::diag::BuildScope::begin();
    let sample_rate = opts.sample_rate;
    let tolerance = opts.tolerance;
    let output_scale = opts.output_scale;
    let output_clamp = opts.output_clamp;
    let input_resistance_flag = opts.input_resistance;
    let oversampling_cli = opts.oversampling;
    let no_dc_block = !opts.dc_block;
    let solver_override = opts.solver.as_str();
    let backward_euler = opts.backward_euler;
    let force_trap = opts.force_trap;
    let tube_grid_fa = opts.tube_grid_fa.as_str();
    let subsample_fire = opts.subsample_fire;
    let subsample_lit_factor = opts.subsample_lit_factor;
    let opamp_rail_mode = opts.opamp_rail_mode;
    // Every DC solve this build makes solves the same problem.
    let dc_request = crate::codegen::ir::DcOpRequest {
        opamp_rail_mode,
        max_iterations: opts.dc_op_max_iterations,
    };
    let nodal_sub_path_override = opts.nodal_sub_path_override;
    let allow_static_glow_on_full_lu = opts.allow_static_glow_on_full_lu;
    let noise_mode = opts.noise_mode;
    let noise_seed = opts.noise_seed;
    let emit_dc_op_recompute = opts.emit_dc_op_recompute;
    let input_node = opts
        .input_nodes
        .first()
        .map(|s| s.as_str())
        .ok_or_else(|| err!("no input node specified"))?;

    // Step 1: Parse netlist
    report!(out, "Step 1: Parsing SPICE netlist...");
    let parse_options = crate::parser::ParseOptions {
        disable_unit_variation: opts.disable_unit_variation,
    };
    let mut netlist = Netlist::parse_with_options(netlist_str, parse_options)
        .with_context(|| "Failed to parse SPICE netlist")?;

    // Resolve the effective oversampling factor: explicit --oversampling wins,
    // else the deck's `.oversampling` recommendation, else 1.
    let oversampling = resolve_oversampling(oversampling_cli, netlist.recommended_oversampling);

    // Expand subcircuit instances (X elements) before MNA
    if !netlist.subcircuits.is_empty() {
        let num_subcircuits = netlist.subcircuits.len();
        netlist
            .expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
        report!(
            out,
            "  ✓ Expanded {} subcircuit definition(s)",
            num_subcircuits
        );
    }

    // Topology gate: the wiring defects a solver cannot see. A typo'd node
    // name invents a node and floats whatever it was on, and every number
    // melange prints afterwards is correct for the circuit it was handed. One
    // implementation for every verb — `crate::topology`.
    // With no output named (`melange dc-op`), the build knows only its input
    // ports, plus any `.port` the deck declares.
    let ports = if opts.output_nodes.is_empty() {
        crate::topology::Ports::inputs_only(opts.input_nodes.clone()).with_deck_pins(&netlist)
    } else {
        crate::topology::Ports::declared(opts.input_nodes.clone(), opts.output_nodes.clone())
    };
    crate::pipeline::topology_gate(&netlist, &ports, out)?;

    report!(out, "  ✓ Parsed {} elements", netlist.elements.len());

    // `--pot` overrides (and every other pot settled onto its `.pot` default),
    // BEFORE the MNA: pot values are R values that flow into G at codegen time.
    if let Some(overrides) = &opts.pot_overrides {
        apply_pot_overrides(&mut netlist, overrides, &|m| report!(out, "{m}"))?;
    }

    // `--opamp-rail-mode boyle-diodes`: the catch diodes (and each op-amp's
    // internal gain node and output buffer) are part of the circuit, so they
    // join the netlist here, before the MNA is assembled; every later step
    // (ports, `.inject`, reductions, DC OP, internal-node expansion) sees them
    // once. The rail mode is read on the circuit WITHOUT the catch diodes on
    // purpose: they exist because of that choice, so resolving it on the
    // augmented circuit would be circular. Do not move the resolution after
    // the augmentation.
    let probe = MnaSystem::from_netlist(&netlist).with_context(|| "Failed to build MNA system")?;
    if crate::codegen::ir::resolve_opamp_rail_mode(&probe, opamp_rail_mode).mode
        == crate::codegen::OpampRailMode::BoyleDiodes
    {
        netlist = crate::codegen::ir::augment_netlist_with_boyle_diodes(&netlist, &probe);
        report!(
            out,
            "  Op-amp rail mode boyle-diodes: catch diodes added to the netlist"
        );
    }

    // Step 2: Build MNA system
    report!(out, "Step 2: Building MNA system...");
    let mut mna =
        MnaSystem::from_netlist(&netlist).with_context(|| "Failed to build MNA system")?;

    report!(
        out,
        "  ✓ {} nodes, {} nonlinear devices",
        mna.n,
        mna.nonlinear_devices.len()
    );

    // Get input node index and add input conductance to G matrix
    // This models the source impedance of the input voltage source
    let input_node_raw = mna.node_map.get(input_node).copied().ok_or_else(|| {
        let suggestions =
            suggest_node_names(input_node, mna.node_names_in_index_order().into_iter());
        let hint = if suggestions.is_empty() {
            format!("Available: {:?}", mna.node_names_in_index_order())
        } else {
            format!("Did you mean: {}?", suggestions.join(", "))
        };
        err!("Input node '{}' not found in circuit. {}", input_node, hint)
    })?;
    if input_node_raw == 0 {
        bail!("Input node cannot be ground (0). Please specify a non-ground node.");
    }
    let input_node_idx = input_node_raw - 1;
    // Resolve input resistance: CLI flag > .input_impedance directive > default 1Ω
    let (input_resistance, ir_source) = if let Some(r) = input_resistance_flag {
        (r, "from --input-resistance flag")
    } else if let Some(r) = netlist.input_impedance {
        (r, "from .input_impedance directive")
    } else {
        (1.0, "default")
    };
    if !(input_resistance > 0.0 && input_resistance.is_finite()) {
        bail!(
            "input resistance must be positive and finite, got {}",
            input_resistance
        );
    }
    report!(
        out,
        "  Input resistance: {} ohm ({})",
        input_resistance,
        ir_source
    );
    let input_conductance = 1.0 / input_resistance;
    if input_node_idx < mna.n {
        mna.g[input_node_idx][input_node_idx] += input_conductance;
    }

    // Resolve any EXTRA input ports (multi-input). Each shares the single
    // `--input-resistance` value for now (per-port resistance is a later CLI
    // follow-up; the field shape is already a Vec). Stamp each extra port's
    // Thevenin conductance into G before the DK kernel is built — exactly like
    // the primary above — so S = A^{-1} bakes in every input port.
    let mut extra_input_nodes: Vec<usize> = Vec::new();
    let mut extra_input_resistances: Vec<f64> = Vec::new();
    for name in opts.input_nodes.iter().skip(1) {
        let raw = mna.node_map.get(name.as_str()).copied().ok_or_else(|| {
            let suggestions = suggest_node_names(name, mna.node_names_in_index_order().into_iter());
            let hint = if suggestions.is_empty() {
                format!("Available: {:?}", mna.node_names_in_index_order())
            } else {
                format!("Did you mean: {}?", suggestions.join(", "))
            };
            err!("Input node '{}' not found in circuit. {}", name, hint)
        })?;
        if raw == 0 {
            bail!("Input node cannot be ground (0). Please specify a non-ground node.");
        }
        let idx = raw - 1;
        if idx == input_node_idx || extra_input_nodes.contains(&idx) {
            report!(
                err,
                "  WARNING: input node '{}' listed more than once; ignoring the duplicate.",
                name
            );
            continue;
        }
        if idx < mna.n {
            mna.g[idx][idx] += input_conductance;
        }
        extra_input_nodes.push(idx);
        extra_input_resistances.push(input_resistance);
    }
    let num_input_ports = 1 + extra_input_nodes.len();
    if num_input_ports > 1 {
        report!(out, "  Input ports: {} (multi-input)", num_input_ports);
        // GUARDRAIL: multi-input superposition is only exact for linear (M=0)
        // circuits. A nonlinear device makes the combined solve non-additive, so
        // reject rather than silently emit a wrong plugin. `mna.m` is the number
        // of nonlinear device dimensions of the ORIGINAL (un-reduced) system.
        if mna.m > 0 {
            bail!(
                "multi-input decks are supported for linear (M=0) circuits only; \
                 found {} nonlinear device dimension(s) ({} nonlinear device(s)). \
                 Multi-input superposition is exact only when nothing multiplicative \
                 or nonlinear touches the inputs. Compile with a single --input-node, \
                 or remove the nonlinear devices.",
                mna.m,
                mna.nonlinear_devices.len()
            );
        }
        if opts.plugin_format {
            bail!(
                "multi-input decks are supported for `--format code` only; the \
                 nih-plug plugin wrapper does not yet route multiple input channels."
            );
        }
        if oversampling > 1 {
            bail!(
                "multi-input decks do not yet support oversampling (--oversampling {}). \
                 Linear (M=0) circuits do not alias, so oversampling is not needed. \
                 Recompile with --oversampling 1.",
                oversampling
            );
        }
        if emit_dc_op_recompute {
            bail!("multi-input decks do not yet support --emit-dc-op-recompute.");
        }
    }

    // Resolve `.inject` runtime feedback sources and `.tap` raw probes.
    //
    // Each `.inject` stamps its Thevenin/Norton conductance (1/impedance) into
    // `mna.g[node][node]` BEFORE the kernel builds — exactly like an input port
    // — so the source is baked into S and present at the DC operating point
    // (Gate 5). The runtime value enters the per-sample RHS as an ordinary
    // constant; the NR loop never sees it (see inject-directive-plan.md).
    // `.tap` is a read-only probe: node resolution only, no stamp.
    let mut injection_specs: Vec<crate::codegen::ir::InjectionSpec> = Vec::new();
    let mut tap_specs: Vec<crate::codegen::ir::TapSpec> = Vec::new();
    if !netlist.injections.is_empty() || !netlist.taps.is_empty() {
        // Scope guardrails (Stage-1). The plan's scope is a single audio `-i`
        // input + N injections, `--format code`.
        if num_input_ports > 1 {
            bail!(
                "`.inject`/`.tap` decks use a single --input-node (plus N injections); \
                 multi-input ports (multiple --input-node entries) are out of scope. \
                 Found {} input ports.",
                num_input_ports
            );
        }
        if opts.plugin_format {
            bail!(
                "`.inject`/`.tap` decks are supported for `--format code` only; the \
                 nih-plug plugin wrapper does not route runtime injections or raw taps."
            );
        }
        for inj in &netlist.injections {
            let raw = mna
                .node_map
                .get(inj.node.as_str())
                .copied()
                .ok_or_else(|| {
                    let suggestions =
                        suggest_node_names(&inj.node, mna.node_names_in_index_order().into_iter());
                    let hint = if suggestions.is_empty() {
                        format!("Available: {:?}", mna.node_names_in_index_order())
                    } else {
                        format!("Did you mean: {}?", suggestions.join(", "))
                    };
                    err!(".inject node '{}' not found in circuit. {}", inj.node, hint)
                })?;
            if raw == 0 {
                bail!(
                    ".inject node '{}' resolves to ground; injection is single-ended \
                     (node-to-ground) and must target a non-ground node.",
                    inj.node
                );
            }
            let idx = raw - 1;
            let (resistance, norton) = match inj.impedance {
                crate::parser::InjectImpedance::Thevenin(r) => (r, false),
                crate::parser::InjectImpedance::Norton(r) => (r, true),
            };
            if !(resistance > 0.0 && resistance.is_finite()) {
                bail!(
                    ".inject '{}' impedance must be positive and finite, got {}",
                    inj.field_name,
                    resistance
                );
            }
            if idx < mna.n {
                mna.g[idx][idx] += 1.0 / resistance;
            }
            let kind = if norton {
                "Norton RSHUNT"
            } else {
                "Thevenin R"
            };
            report!(
                out,
                "  Injection '{}' at node '{}' ({}={} ohm)",
                inj.field_name,
                inj.node,
                kind,
                resistance
            );
            injection_specs.push(crate::codegen::ir::InjectionSpec {
                node: idx,
                name: inj.field_name.clone(),
                resistance,
                norton,
            });
        }
        for tap in netlist.taps.iter().filter(|_| opts.resolve_taps) {
            let raw = mna
                .node_map
                .get(tap.node.as_str())
                .copied()
                .ok_or_else(|| {
                    let suggestions =
                        suggest_node_names(&tap.node, mna.node_names_in_index_order().into_iter());
                    let hint = if suggestions.is_empty() {
                        format!("Available: {:?}", mna.node_names_in_index_order())
                    } else {
                        format!("Did you mean: {}?", suggestions.join(", "))
                    };
                    err!(".tap node '{}' not found in circuit. {}", tap.node, hint)
                })?;
            if raw == 0 {
                bail!(".tap node '{}' resolves to ground.", tap.node);
            }
            report!(
                out,
                "  Tap '{}' at node '{}' (raw inner-rate)",
                tap.name,
                tap.node
            );
            tap_specs.push(crate::codegen::ir::TapSpec {
                node: raw - 1,
                name: tap.name.clone(),
            });
        }
    }

    // Warn if passive EQ topology detected with low source impedance
    if input_resistance < 10.0 && !mna.pots.is_empty() {
        // Check if any pot is connected to the input node
        let mut connected_pots = Vec::new();
        for pot in &mna.pots {
            // Direct connection: pot node matches input node (both 1-indexed)
            let direct = pot.node_p == input_node_raw || pot.node_q == input_node_raw;
            // One-hop: connected through another component (check G matrix)
            let pot_p_0 = if pot.node_p > 0 {
                pot.node_p - 1
            } else {
                usize::MAX
            };
            let pot_q_0 = if pot.node_q > 0 {
                pot.node_q - 1
            } else {
                usize::MAX
            };
            let one_hop = (pot_p_0 < mna.n && mna.g[input_node_idx][pot_p_0] != 0.0)
                || (pot_q_0 < mna.n && mna.g[input_node_idx][pot_q_0] != 0.0);
            if direct || one_hop {
                connected_pots.push(pot.name.clone());
            }
        }
        if !connected_pots.is_empty() {
            report!(err, "");
            report!(
                err,
                "  WARNING: Passive EQ topology detected with {:.0}Ω source impedance.",
                input_resistance
            );
            report!(
                err,
                "  Pots connected to input: {}",
                connected_pots.join(", ")
            );
            report!(
                err,
                "  A low source impedance overwhelms passive EQ networks, making pots inert."
            );
            report!(
                err,
                "  Consider adding to your netlist:  .input_impedance 600"
            );
            report!(
                err,
                "  Or use the CLI flag:              --input-resistance 600"
            );
            report!(err, "");
        }
    }

    // Tier 3c: Input impedance suggestion based on topology.
    // When using the 1Ω default (no --input-resistance, no .input_impedance directive),
    // check what devices connect to the input node and suggest a realistic impedance.
    if ir_source == "default" && input_resistance < 10.0 {
        use crate::parser::Element;
        let input_name = input_node;
        let has_tube_input = netlist.elements.iter().any(|e| matches!(e,
            Element::Triode { n_grid, .. } | Element::Pentode { n_grid, .. } if n_grid == input_name
        ));
        let has_jfet_input = netlist.elements.iter().any(|e| {
            matches!(e,
                Element::Jfet { ng, .. } if ng == input_name
            )
        });
        let has_mosfet_input = netlist.elements.iter().any(|e| {
            matches!(e,
                Element::Mosfet { ng, .. } if ng == input_name
            )
        });
        // Check if input connects through a large resistor (>10k) suggesting high-Z input
        let has_large_input_r = netlist.elements.iter().any(|e| {
            if let Element::Resistor {
                n_plus,
                n_minus,
                value,
                ..
            } = e
            {
                (n_plus == input_name || n_minus == input_name) && *value > 10_000.0
            } else {
                false
            }
        });
        if has_tube_input || has_jfet_input || has_mosfet_input {
            let device = if has_tube_input {
                "tube grid"
            } else if has_jfet_input {
                "JFET gate"
            } else {
                "MOSFET gate"
            };
            report!(out,
                "  Hint: Input connects to {} (high impedance). Consider --input-resistance 1M or .input_impedance 1M",
                device
            );
        } else if has_large_input_r {
            report!(out,
                "  Hint: Large resistor on input node. Consider --input-resistance 10k or .input_impedance 10k"
            );
        }
    }

    // Every conductance to ground stamped above: the input ports and the
    // `.inject` sources. A reduction that rebuilds the MNA from the netlist
    // restamps all of them.
    let port_stamps: Vec<(usize, f64)> = std::iter::once(input_node_idx)
        .chain(extra_input_nodes.iter().copied())
        .map(|node| (node, input_conductance))
        .chain(injection_specs.iter().map(|s| (s.node, 1.0 / s.resistance)))
        .collect();

    // Stamp junction capacitances BEFORE FA detection (caps affect DC OP).
    // Internal node expansion happens AFTER FA detection to avoid disrupting it.
    {
        let device_slots =
            crate::codegen::ir::CircuitIR::build_device_info(&netlist).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    // Detect forward-active BJTs (runs DC OP with 2D model, checks Vbc < -1V)
    let fa_config = crate::codegen::CodegenConfig {
        input_node: input_node_idx,
        input_resistance,
        bjt_fa_mode: opts.bjt_fa_mode,
        dc_op_max_iterations: opts.dc_op_max_iterations,
        ..crate::codegen::CodegenConfig::default()
    };
    // Skip FA detection when the final solver will be Nodal:
    //   (a) user explicitly overrode with --solver nodal
    //   (b) auto-routing will send this circuit to Nodal because the
    //       un-reduced DK kernel is trap-unstable or fails to build.
    // Motivation: the FA-reduced DC-OP can converge to a parasitic
    // equilibrium on push-pull topologies (see memory/wurli_power_amp_...).
    // Since Nodal handles full-dim BJTs natively, FA is unnecessary there.
    let forward_active = crate::pipeline::apply_forward_active_reduction(
        &mut mna,
        &netlist,
        &fa_config,
        solver_override,
        sample_rate,
        oversampling,
        &port_stamps,
        out,
    )?;

    // Grid-off pentode reduction (3D → 2D NR block with Vg2k frozen and
    // Ig1 dropped). Only `--tube-grid-fa on` reduces (warned per device);
    // `auto` and `off` keep the full 3D model. Skipped on the nodal route,
    // including the auto-router's pre-route verdict, so a reduction can
    // never move a circuit from nodal to DK.
    let grid_off_pentodes = crate::pipeline::apply_grid_off_reduction(
        &mut mna,
        &netlist,
        &fa_config,
        &forward_active,
        tube_grid_fa,
        solver_override,
        sample_rate,
        oversampling,
        &port_stamps,
    )?;
    if let Some(msg) = crate::pipeline::format_grid_off_log(&grid_off_pentodes) {
        report!(out, "{msg}");
    }

    // Apply `.linearize` directives: DC-OP → extract g-params → rebuild
    // MNA with FA + linearized + grid-off reductions fused via
    // `from_netlist_with_all_reductions`, then re-stamp junction caps.
    // Shared with simulate/analyze — see `apply_linearize_reductions`.
    let linearize_outcome = crate::pipeline::apply_linearize_reductions(
        &mut mna,
        &netlist,
        &forward_active,
        &grid_off_pentodes,
        &port_stamps,
        dc_request,
        out,
    )?;

    // NOTE: Internal node expansion for parasitic BJTs is deferred until after
    // solver routing. The DK path handles parasitics via bjt_with_parasitics() inner NR.
    // Only the nodal path benefits from MNA-level internal nodes (eliminates inner NR).

    // BJT junction-cap preflight: solve DC OP on the fully-reduced MNA and
    // re-linearize CJE/CJC at Vbe_op/Vbc_op, then add the diffusion-cap
    // contribution `TF · |Ic| / Vt`. No-op when every BJT uses the SPICE
    // defaults (TF = 0, CJE = CJC = 0, etc.). See the SPICE validation
    // harness for the matching call site — the two paths must agree so a
    // plugin built from `melange compile` behaves like the validated one.
    let dc_preflight = preflight_relinearize_bjt_caps(&mut mna, &netlist, dc_request);
    let output_clamp = if opts.output_clamp_auto {
        dc_preflight
            .as_ref()
            .map(|dc| {
                dc.v_node
                    .iter()
                    .cloned()
                    .fold(0.0_f64, |acc, v| acc.max(v.abs()))
                    * 3.0
            })
            .unwrap_or(0.0)
            .max(output_clamp)
    } else {
        output_clamp
    };

    // Step 3: Create DK kernel
    // Use augmented MNA for inductor circuits (well-conditioned for large L)
    let has_inductors_compile = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();
    // Built on EVERY route, nodal included: the routing decision is read off
    // this kernel. See the `simulate` path for the same note.
    report!(
        out,
        "Step 3: Creating DK kernel (routing analysis \u{2014} built on every route)..."
    );
    // Build at the INTERNAL (oversampled) rate — the routing decision below
    // must see the same S/A_neg the generated solver will actually ship.
    // Building at the base host rate can miss trap/BE instability that only
    // appears once oversampling raises alpha (see
    // memory/dk_backward_euler_ignored_trap_unstable.md). For os=1 this is
    // identical to `sample_rate` (no behavior change).
    let routing_rate = sample_rate * oversampling as f64;
    let kernel_result = if has_inductors_compile {
        report!(out, "  Using augmented MNA for inductors");
        DkKernel::from_mna_augmented(&mna, routing_rate)
    } else {
        DkKernel::from_mna(&mna, routing_rate)
    };
    // If DK kernel fails (e.g., positive feedback / oscillator circuit),
    // auto-fall back to nodal solver which has no K diagonal constraint.
    let (kernel, dk_failed) = match kernel_result {
        Ok(k) => {
            report!(
                out,
                "  ✓ Matrix dimensions: {}",
                format_system_size(k.n, k.n_nodes, k.m)
            );
            (k, false)
        }
        Err(ref e) => {
            if solver_override == "dk" {
                // User explicitly requested DK — propagate the error
                kernel_result.with_context(|| "Failed to create DK kernel")?;
                unreachable!()
            } else {
                report!(out, "  DK kernel failed: {}", e);
                report!(
                    out,
                    "  Auto-selecting nodal solver (handles positive feedback / oscillators)"
                );
                // The kernel still feeds the internal-node expansion gate (its K
                // diagonal) and the Newton-budget tuner, so a real kernel beats a
                // zero one whenever it exists: try the augmented form first. Only
                // if that fails too does a zero kernel stand in for dimensions.
                let augmented = if has_inductors_compile {
                    None
                } else {
                    DkKernel::from_mna_augmented(&mna, routing_rate).ok()
                };
                if let Some(k) = augmented {
                    report!(
                        out,
                        "  Routing analysis uses the augmented kernel: {}",
                        format_system_size(k.n, k.n_nodes, k.m)
                    );
                    (k, true)
                } else {
                    let m = mna.m;
                    let n = mna.n_aug;
                    let dummy = DkKernel {
                        n,
                        m,
                        n_nodes: mna.n,
                        num_devices: mna.num_devices,
                        sample_rate,
                        s: vec![0.0; n * n],
                        a_neg: vec![0.0; n * n],
                        k: vec![0.0; m * m],
                        n_v: vec![0.0; m * n],
                        n_i: vec![0.0; n * m],
                        rhs_const: vec![0.0; n],
                        inductors: vec![],
                        coupled_inductors: vec![],
                        transformer_groups: vec![],
                        pots: vec![],
                        wiper_groups: vec![],
                        gang_groups: vec![],
                    };
                    (dummy, true)
                }
            }
        }
    };

    // Step 4: Generate code
    report!(out, "Step 4: Generating Rust code...");

    // Resolve the op-amp rail mode so we can print the auto-decision before
    // the emitter runs its own logs. This is a side-effect-free inspection —
    // the actual mode baked into the IR happens inside codegen (which does
    // the same resolution deterministically).
    {
        use crate::codegen::ir::resolve_opamp_rail_mode;
        let resolved = resolve_opamp_rail_mode(&mna, opamp_rail_mode);
        report!(
            out,
            "  Op-amp rail mode: {} ({})",
            resolved.mode,
            resolved.reason.as_str()
        );
    }

    // Parse comma-separated output nodes
    let output_node_names: Vec<&str> = opts.output_nodes.iter().map(|s| s.as_str()).collect();
    let mut output_node_indices = Vec::new();
    for name in &output_node_names {
        let raw = mna.node_map.get(*name).copied().ok_or_else(|| {
            let suggestions = suggest_node_names(name, mna.node_names_in_index_order().into_iter());
            let hint = if suggestions.is_empty() {
                format!("Available: {:?}", mna.node_names_in_index_order())
            } else {
                format!("Did you mean: {}?", suggestions.join(", "))
            };
            err!("Output node '{}' not found in circuit. {}", name, hint)
        })?;
        if raw == 0 {
            bail!(
                "Output node '{}' cannot be ground (0). Please specify a non-ground node.",
                name
            );
        }
        output_node_indices.push(raw - 1);
    }

    // DC-block auto-detection: if the circuit already has a coupling cap on the
    // output node, the built-in 5 Hz DC blocker is redundant (double-filtering
    // and adds 200ms settle time). Suggest --no-dc-block.
    let dc_block_auto_skip = !no_dc_block
        && !output_node_names.is_empty()
        && output_node_names
            .iter()
            .all(|name| has_output_coupling_cap(&netlist, name));
    if dc_block_auto_skip {
        report!(out,
            "  Output coupling cap detected on \"{}\". Consider --no-dc-block to avoid double filtering.",
            output_node_names.join(", ")
        );
    }

    // Route solver first — routing info feeds into config auto-tuning.
    let routing = crate::codegen::routing::auto_route(&kernel, &mna, dk_failed, opamp_rail_mode);

    // Tier 3b: Auto-tune max_iter based on M and solver path (see
    // `auto_tune_max_iter`); `opts.max_iter` is the user's explicit budget, if
    // any.
    //
    // Resolve the `.integrator` directive with the same precedence codegen
    // uses so the iteration budget matches the integrator that ships, on every
    // verb: they all compile and run this same generated code.
    let (effective_backward_euler, _, _) = crate::codegen::ir::resolve_integrator_flags(
        backward_euler,
        force_trap,
        netlist.integrator,
    );
    let user_max_iter = opts.max_iter;
    let max_iter = crate::pipeline::auto_tune_max_iter(
        user_max_iter,
        &kernel,
        &routing,
        !effective_backward_euler,
    );
    let max_iter_be_promoted =
        crate::pipeline::auto_tune_max_iter(user_max_iter, &kernel, &routing, false);

    // Broadcast single output_scale to all outputs
    let output_scales = vec![output_scale; output_node_indices.len()];

    let config = CodegenConfig {
        circuit_name: opts.circuit_name.clone(),
        input_node: input_node_idx,
        extra_input_nodes: extra_input_nodes.clone(),
        extra_input_resistances: extra_input_resistances.clone(),
        output_nodes: output_node_indices.clone(),
        sample_rate,
        max_iterations: max_iter,
        tolerance,
        output_scales,
        output_clamp_v: output_clamp,
        include_dc_op: true,
        input_resistance,
        oversampling_factor: oversampling,
        dc_block: !no_dc_block,
        backward_euler,
        force_trap,
        opamp_rail_mode,
        nodal_sub_path_override,
        noise_mode,
        noise_master_seed: noise_seed,
        emit_dc_op_recompute,
        max_iterations_be_promoted: Some(max_iter_be_promoted),
        injections: if opts.inject_runtime {
            injection_specs.clone()
        } else {
            Vec::new()
        },
        taps: tap_specs.clone(),
        subsample_fire,
        subsample_lit_factor,
        allow_static_glow_on_full_lu,
        dc_op_max_iterations: opts.dc_op_max_iterations,
        ..CodegenConfig::default()
    };

    let generator = CodeGenerator::new(config.clone());
    if solver_override == "dk" {
        if let Some(blocker) = forced_dk_hard_blocker(&routing) {
            bail!(
                "--solver dk cannot be forced on this circuit: {blocker}.\n\
                 The DK solver structurally cannot represent it and would emit a \
                 silently-wrong solver. Use --solver auto (recommended) or --solver nodal."
            );
        }
    }
    let use_nodal_codegen = match solver_override {
        "nodal" => true,
        "dk" => false,
        _ => routing.route == crate::codegen::routing::SolverRoute::Nodal,
    };
    let mut solver_label = if use_nodal_codegen { "nodal" } else { "DK" };
    let mut solver_reason = if solver_override == "nodal" || solver_override == "dk" {
        // Forced route: still surface what the auto-router decided so the pinned
        // sub-path is visible, not masked by the override reason (melange-circuits
        // t469 — a measuring verb must see the route it is actually on).
        format!(
            "--solver {} (user override; auto would pick: {})",
            solver_override, routing.reason
        )
    } else {
        routing.reason.clone()
    };

    // FA reduction is kept for nodal path when the DC OP confirms deeply
    // reverse-biased B-C junctions (Vbc < -0.5V). The FA detection threshold
    // is conservative enough that BJTs passing it have adequate margin for
    // audio-level transients. Previously this blanket-undid all FA for nodal,
    // but that forces M=16 for ladder filters where all BJTs are clearly FA.

    // The IR, and with it the route: the DK IR refuses a self-starting
    // oscillator, which then builds on nodal. The operating point the build
    // ships is solved once, here, on the MNA its route generates from, and
    // handed to the IR.
    let (prepared, dc_op) = if use_nodal_codegen {
        report!(out, "  Using nodal solver codegen");
        prepare_nodal_route(&generator, &mut mna, &netlist, dc_request)?
    } else {
        // DK path: do NOT expand internal nodes. The DK kernel is ill-conditioned
        // with high-conductance parasitic nodes. Instead, bjt_with_parasitics()
        // inner NR handles parasitics in the generated code.
        if has_inductors_compile {
            report!(out, "  Using DK codegen with augmented MNA for inductors");
        }
        // The preflight solved this MNA's operating point (a circuit without
        // devices had no preflight).
        let dc_op = match dc_preflight {
            Some(dc) => dc,
            None => crate::codegen::ir::solve_dc_op(&mna, &netlist, dc_request)
                .with_context(|| "Code generation failed")?,
        };
        match generator.prepare_dk(&kernel, &mna, &netlist, Some(dc_op.clone())) {
            Err(crate::codegen::CodegenError::SelfStartingOscillator(why))
                if solver_override != "dk" =>
            {
                report!(out, "  Using nodal solver codegen: {why}");
                solver_label = "nodal";
                solver_reason = format!("self-starting oscillator: {why}");
                prepare_nodal_route(&generator, &mut mna, &netlist, dc_request)?
            }
            other => (other.with_context(|| "Code generation failed")?, dc_op),
        }
    };

    // An operating point that is not a solution is not a place to start from:
    // the generated code would begin there and slew or ring away from it, with
    // only a WARN to say so.
    if !dc_op.converged {
        let names = mna.node_names_in_index_order();
        let worst = dc_op
            .kcl_worst_row
            .map(|row| {
                let name = names.get(row + 1).copied().unwrap_or("");
                if name.is_empty() {
                    format!(" at row {row}")
                } else {
                    format!(" at v({name})")
                }
            })
            .unwrap_or_default();
        let what = format!(
            "the DC operating point did not converge ({:?}, {} iterations; KCL residual \
             {:.3e} A{worst})",
            dc_op.method, dc_op.iterations, dc_op.kcl_residual_max
        );
        if opts.allow_unconverged_dc_op {
            report!(
                err,
                "  WARNING: {what}. Building anyway (--allow-unconverged-dc-op): the \
                 generated code starts from a state that is not a solution."
            );
        } else {
            bail!(
                "{what}: the generated code would start from a state that is not a \
                 solution. --allow-unconverged-dc-op builds it anyway."
            );
        }
    }

    Ok(Assembled {
        config,
        prepared,
        dc_op,
        netlist,
        mna,
        kernel,
        routing,
        solver_label,
        solver_reason,
        max_iter,
        oversampling,
        input_node_idx,
        input_resistance,
        input_resistance_source: ir_source,
        output_node_indices,
        forward_active,
        grid_off_pentodes,
        linearize_outcome,
        injection_specs,
    })
}

/// The nodal route's tail: expand the parasitic-BJT internal nodes, solve the
/// operating point the build ships on the result, and build the nodal IR from
/// it.
fn prepare_nodal_route(
    generator: &CodeGenerator,
    mna: &mut MnaSystem,
    netlist: &Netlist,
    dc_request: crate::codegen::ir::DcOpRequest,
) -> Result<(crate::codegen::PreparedIr, crate::dc_op::DcOpResult), BuildError> {
    crate::pipeline::expand_internal_nodes(mna, netlist);
    let dc_op = crate::codegen::ir::solve_dc_op(mna, netlist, dc_request)
        .with_context(|| "Nodal code generation failed")?;
    let prepared = generator
        .prepare_nodal(mna, netlist, Some(dc_op.clone()))
        .with_context(|| "Nodal code generation failed")?;
    Ok((prepared, dc_op))
}

/// Build `netlist_str` as every verb ships it: [`assemble`], then emit.
pub fn build(
    netlist_str: &str,
    opts: &BuildOptions,
    out: Reporter<'_>,
    err: Reporter<'_>,
) -> Result<Built, BuildError> {
    // Each model warning once per build, emission included.
    let _warnings = crate::diag::BuildScope::begin();
    let a = assemble(netlist_str, opts, out, err)?;
    let context = if a.solver_label == "nodal" {
        "Nodal code generation failed"
    } else {
        "Code generation failed"
    };
    let generated = CodeGenerator::new(a.config)
        .emit(&a.prepared)
        .with_context(|| context)?;
    Ok(Built {
        generated,
        dc_op: a.dc_op,
        netlist: a.netlist,
        mna: a.mna,
        kernel: a.kernel,
        routing: a.routing,
        solver_label: a.solver_label,
        solver_reason: a.solver_reason,
        max_iter: a.max_iter,
        oversampling: a.oversampling,
        input_node_idx: a.input_node_idx,
        input_resistance: a.input_resistance,
        input_resistance_source: a.input_resistance_source,
        output_node_indices: a.output_node_indices,
        forward_active: a.forward_active,
        grid_off_pentodes: a.grid_off_pentodes,
        linearize_outcome: a.linearize_outcome,
        injection_specs: a.injection_specs,
    })
}

#[allow(clippy::too_many_arguments)]
/// Suggest similar node names when a lookup fails.
/// Returns names that share a common prefix or contain the query as a substring.
pub fn suggest_node_names<'a>(
    query: &str,
    available: impl Iterator<Item = &'a str>,
) -> Vec<String> {
    let q = query.to_ascii_lowercase();
    let mut suggestions: Vec<(usize, String)> = Vec::new();
    for name in available {
        let n = name.to_ascii_lowercase();
        // Exact case-insensitive match
        if n == q {
            suggestions.push((0, name.to_string()));
        }
        // One is a prefix of the other
        else if n.starts_with(&q) || q.starts_with(&n) {
            suggestions.push((1, name.to_string()));
        }
        // Substring match
        else if n.contains(&q) || q.contains(&n) {
            suggestions.push((2, name.to_string()));
        }
        // Common audio I/O aliases
        else {
            let input_aliases = ["in", "input", "vin", "audio_in", "sig_in", "mid_in"];
            let output_aliases = ["out", "output", "vout", "audio_out", "sig_out"];
            let q_is_input = input_aliases.contains(&q.as_str());
            let n_is_input = input_aliases.contains(&n.as_str());
            let q_is_output = output_aliases.contains(&q.as_str());
            let n_is_output = output_aliases.contains(&n.as_str());
            if (q_is_input && n_is_input) || (q_is_output && n_is_output) {
                suggestions.push((1, name.to_string()));
            }
        }
    }
    suggestions.sort_by_key(|(score, _)| *score);
    suggestions.into_iter().map(|(_, name)| name).collect()
}

/// One consistent rendering of the two system sizes every verb reports.
///
/// The same circuit legitimately has more than one "N", and printing them bare
/// reads as a contradiction. On `examples/passive-eq1a.cir`:
///   * `nodes`  lists 41 entries — ground plus 40 circuit nodes.
///   * `dc-op`  reports N=40 — `MnaSystem::n`, circuit nodes with ground excluded.
///   * `analyze` / `compile` / `simulate` report N=52 — `MnaSystem::n_aug`
///     (== `DkKernel::n`): the 40 circuit nodes plus one augmented row per
///     voltage source, VCVS, transformer coupling and inductor winding. That
///     deck declares 1 voltage source and 11 inductors, so 40 + 12 = 52. Those
///     rows carry algebraic constraints, not node voltages.
/// M is the same number everywhere: the total nonlinear device dimension
/// (1 per diode, 2 per BJT/JFET/MOSFET/triode/VCA, 3 per pentode).
pub fn format_system_size(n_total: usize, n_circuit_nodes: usize, m: usize) -> String {
    let extra = n_total.saturating_sub(n_circuit_nodes);
    if extra == 0 {
        format!("N={n_total} ({n_circuit_nodes} circuit nodes, ground excluded), M={m} (nonlinear device dimensions)")
    } else {
        format!(
            "N={n_total} ({n_circuit_nodes} circuit nodes + {extra} constraint rows: voltage \
             sources, inductor windings, transformer couplings), M={m} (nonlinear device dimensions)"
        )
    }
}

/// If `--solver dk` is forced on a circuit that STRUCTURALLY requires the nodal
/// path, DK codegen builds without error but emits a silently-wrong solver
/// (behavioral source dropped → linear passthrough; saturating inductor
/// linearized; multi-transformer DC-OP singular; transformer-NFB K-diagonal
/// divergence). Return the blocker description for those cases so the caller can
/// fail loud. Soft router preferences (trapezoidal instability, large M, K/S
/// conditioning) are NOT blockers — DK can still represent those circuits, so a
/// forced override stays valid for them. `dk_failed` is already hard-errored at
/// kernel construction, so it is not repeated here.
pub fn forced_dk_hard_blocker(
    routing: &crate::codegen::routing::RoutingDecision,
) -> Option<String> {
    if routing.behavioral {
        Some("a behavioral B-source is present — DK cannot stamp it, so it is dropped (linear passthrough)".to_string())
    } else if routing.saturating_inductor {
        Some("a saturating inductor is present — its flux law is solved by Newton on an augmented row each sample, which DK cannot do, so it would run linear (no saturation)".to_string())
    } else if routing.multi_transformer {
        Some("multiple transformer groups are present — the DK K matrix is singular (the build ships converged=false)".to_string())
    } else if routing.k_diag_unsafe {
        Some("a non-negative K diagonal with live current injection (e.g. transformer-coupled negative feedback) — the DK Schur Newton iteration diverges".to_string())
    } else if routing.opamp_active_set {
        Some(
            "an op-amp needs active-set rail handling (asked for with --opamp-rail-mode, or \
             picked because an op-amp output is capacitor-coupled downstream) — pinning a \
             railed output and re-solving the circuit is nodal-only, and DK can only clamp \
             the output, which corrupts the downstream capacitor history"
                .to_string(),
        )
    } else if routing.opamp_boyle_diodes {
        Some(
            "the op-amp rail mode is boyle-diodes — the catch diodes hang off each op-amp's \
             internal gain node, which only the nodal solver builds"
                .to_string(),
        )
    } else if routing.opamp_transient_aol_cap {
        Some(
            "an op-amp card sets AOL_TRANSIENT_CAP — the cap is applied to the transient \
             matrices by the nodal solver only, and DK would ignore it"
                .to_string(),
        )
    } else {
        None
    }
}

/// Resolve the effective oversampling factor for the shipping path
/// (`compile` / `simulate` / `analyze`).
///
/// `.oversampling N` in a deck is an accuracy MINIMUM / recommendation, not a
/// mandate: rate costs CPU and latency, which is the downstream plugin author's
/// product decision. So an explicit `--oversampling` on the command line always
/// wins — even when it is LOWER than the deck value, in which case a warning is
/// logged. Absent an explicit flag, the deck's `.oversampling` value is used;
/// absent both, 1. `validate` never calls this — it stays at the base rate
/// regardless of the directive (an oversampled comparison is confounded by
/// anti-alias-filter group delay).
pub fn resolve_oversampling(explicit_cli: Option<usize>, recommended: Option<usize>) -> usize {
    match (explicit_cli, recommended) {
        (Some(cli), Some(rec)) => {
            if cli < rec {
                crate::diag_warn!(
                    "deck recommends .oversampling >= {rec} for accuracy; building at {cli} < {rec} \
                     by request (--oversampling wins)"
                );
            }
            cli
        }
        (Some(cli), None) => cli,
        (None, Some(rec)) => rec,
        (None, None) => 1,
    }
}

/// Check if a capacitor is directly connected to the given output node.
/// Returns true if the circuit already has an output coupling cap, meaning
/// melange's built-in DC blocker is redundant.
/// Pre-kernel preflight: solve the DC operating point on the fully-reduced
/// MNA and re-linearize BJT junction capacitances at that bias point.
///
/// Matches the SPICE validation harness so a plugin built with
/// `melange compile` sees the same ngspice-parity cap stamps that the
/// validation tests exercise. Without this call, BJT `.model` cards
/// carrying `TF` / `VJE` / `MJE` / `VJC` / `MJC` / `FC` parameters ship
/// with zero-bias linear caps and the diffusion-cap contribution is
/// silently dropped.
///
/// Must be called AFTER all MNA reductions (FA / grid-off / linearize)
/// have produced their final mna and stamped zero-bias caps, but BEFORE
/// `DkKernel::from_mna` — the kernel's precomputed `S = A⁻¹` must see
/// the re-linearized `C`.
///
/// Returns the converged `DcOpResult` so the caller can thread it into
/// `CodeGenerator::generate_with_dc_op` and avoid a redundant solve
/// inside codegen. Returns `None` when the circuit has no nonlinear
/// devices (nothing to re-linearize).
pub fn preflight_relinearize_bjt_caps(
    mna: &mut crate::mna::MnaSystem,
    netlist: &crate::parser::Netlist,
    dc_request: crate::codegen::ir::DcOpRequest,
) -> Option<crate::dc_op::DcOpResult> {
    let device_slots =
        crate::codegen::ir::CircuitIR::build_device_info_with_mna(netlist, Some(mna))
            .unwrap_or_default();
    if device_slots.is_empty() {
        return None;
    }
    let dc = crate::dc_op::solve_dc_operating_point(
        mna,
        &device_slots,
        &crate::codegen::ir::dc_op_config(mna, dc_request),
    );
    if dc.converged {
        mna.relinearize_bjt_caps_at_dc_op(&device_slots, &dc.v_nl, &dc.i_nl);
    }
    Some(dc)
}

pub fn has_output_coupling_cap(netlist: &crate::parser::Netlist, output_node_name: &str) -> bool {
    use crate::parser::Element;
    netlist.elements.iter().any(|elem| {
        matches!(elem, Element::Capacitor { n_plus, n_minus, .. }
            if n_plus == output_node_name || n_minus == output_node_name)
    })
}

/// Apply `--pot NAME=VALUE` overrides to a parsed netlist, then settle every
/// pot that was NOT overridden onto its `.pot` default.
///
/// Shared by `analyze` and `simulate` so a knob position means the same thing
/// in both. `simulate` gained `--pot` late: it had `--switch` but no `--pot`,
/// so a distortion pedal's Drive control — which IS the circuit — could be
/// characterised at any setting by `analyze` and heard at exactly one.
///
/// Name resolution for each SPEC:
///   1. Try `.wiper` labels first. A wiper has two legs (cw/ccw) that co-vary
///      with a position in 0.0..=1.0; the value is a POSITION and both legs are
///      set accordingly.
///   2. Fall back to `.pot` (by label or resistor name). The value is a
///      RESISTANCE in ohms (engineering suffixes accepted, e.g. "200k"), and
///      is range-checked against that pot's declared min..max.
///
/// `log` receives one line per applied override; callers route it to whichever
/// stream carries their progress messages (`analyze` writes CSV on stdout, so
/// it logs to stderr).
///
/// Minimum leg resistance (WIPER_MIN_LEG_R) must track the constant in
/// `parser::expand_wipers`.
pub fn apply_pot_overrides(
    netlist: &mut crate::parser::Netlist,
    pot_overrides: &[String],
    log: &dyn Fn(&str),
) -> Result<(), BuildError> {
    const WIPER_MIN_LEG_R: f64 = 10.0;

    let mut overridden_resistors: std::collections::HashSet<String> =
        std::collections::HashSet::new();

    // First, apply explicit --pot overrides
    for spec in pot_overrides {
        let (name, val_str) = spec
            .split_once('=')
            .ok_or_else(|| err!("Invalid --pot format '{}', expected NAME=VALUE", spec))?;

        // Try wiper label first (the label on `.wiper` lives on the directive, not
        // on the expanded PotDirective entries — those have label:None).
        let wiper_match = netlist.wipers.iter().find(|w| {
            w.label
                .as_deref()
                .map(|l| l.eq_ignore_ascii_case(name))
                .unwrap_or(false)
        });

        if let Some(wiper) = wiper_match {
            // Wiper: interpret value as position 0.0..=1.0
            let pos = val_str.parse::<f64>().map_err(|_| {
                err!(
                    "Invalid wiper position '{}' in --pot {} (expected 0.0..=1.0)",
                    val_str,
                    spec
                )
            })?;
            if !pos.is_finite() || !(0.0..=1.0).contains(&pos) {
                bail!(
                    "Wiper position must be in 0.0..=1.0: {} (got {})",
                    spec,
                    pos
                );
            }
            let r_total = wiper.total_resistance;
            let range = r_total - 2.0 * WIPER_MIN_LEG_R;
            // Matches parser::expand_wipers and plugin_template wiper_assignments.
            let r_cw = (1.0 - pos) * range + WIPER_MIN_LEG_R;
            let r_ccw = pos * range + WIPER_MIN_LEG_R;
            let cw_name = wiper.resistor_cw.clone();
            let ccw_name = wiper.resistor_ccw.clone();

            for (resistor_name, r_val) in [(&cw_name, r_cw), (&ccw_name, r_ccw)] {
                let found = netlist.elements.iter_mut().any(|e| {
                    if let crate::parser::Element::Resistor {
                        name: n, value: v, ..
                    } = e
                    {
                        if n.eq_ignore_ascii_case(resistor_name) {
                            *v = r_val;
                            return true;
                        }
                    }
                    false
                });
                if !found {
                    bail!("Wiper resistor '{}' not found in netlist", resistor_name);
                }
                for p in netlist.pots.iter_mut() {
                    if p.resistor_name.eq_ignore_ascii_case(resistor_name) {
                        p.default_value = Some(r_val);
                        break;
                    }
                }
                overridden_resistors.insert(resistor_name.to_ascii_uppercase());
            }
            log(&format!(
                "  Wiper override: {} pos={:.3} ({} = {:.1}Ω, {} = {:.1}Ω)",
                name, pos, cw_name, r_cw, ccw_name, r_ccw,
            ));
            continue;
        }

        // Pot: interpret value as a resistance in ohms
        let value = crate::parser::parse_value(val_str)
            .map_err(|_| err!("Invalid resistance value '{}' in --pot {}", val_str, spec))?;
        if value <= 0.0 || !value.is_finite() {
            bail!("Pot value must be positive and finite: {}", spec);
        }

        // Match by pot label or resistor name
        let matched = netlist
            .pots
            .iter()
            .find(|p| {
                p.label
                    .as_deref()
                    .map(|l| l.eq_ignore_ascii_case(name))
                    .unwrap_or(false)
                    || p.resistor_name.eq_ignore_ascii_case(name)
            })
            .ok_or_else(|| {
                let mut available: Vec<String> = netlist
                    .pots
                    .iter()
                    .map(|p| {
                        format!(
                            "{} ({})",
                            p.resistor_name,
                            p.label.as_deref().unwrap_or("no label")
                        )
                    })
                    .collect();
                for w in &netlist.wipers {
                    if let Some(label) = &w.label {
                        available.push(format!("[wiper] {} (pos 0.0..=1.0)", label));
                    }
                }
                err!(
                    "Pot '{}' not found. Available: {}",
                    name,
                    available.join(", ")
                )
            })?;
        let resistor_name = matched.resistor_name.clone();
        let (r_min, r_max) = (matched.min_value, matched.max_value);
        let display_name = matched
            .label
            .clone()
            .unwrap_or_else(|| matched.resistor_name.clone());

        // REFUSE out of range — do not clamp. The `.pot` min..max IS the real
        // knob's travel, and it is what the generated plugin's parameter maps
        // onto. Accepting a value outside it silently characterises a position
        // the plugin can never produce: on examples/passive-eq1a.cir,
        // `--pot "LF Boost=1e9"` used to report +14.93 dB at 20 Hz, about 4 dB
        // past anything the physical control can reach. Switch positions were
        // already range-checked; pots were not. Refusing (rather than clamping)
        // keeps the reported response and the requested setting the same thing.
        if value < r_min || value > r_max {
            bail!(
                "Pot '{}' value {} out of range ({}..{} ohm). \
                 The range comes from the `.pot` directive for {} and is the travel of the \
                 real control, so a value outside it describes a setting the generated plugin \
                 cannot reach. Pick a value inside the range \
                 (`melange nodes <circuit>` lists every pot's range and default).",
                display_name,
                format_ohms(value),
                format_ohms(r_min),
                format_ohms(r_max),
                resistor_name,
            );
        }

        // Update element value
        let found = netlist.elements.iter_mut().any(|e| {
            if let crate::parser::Element::Resistor {
                name: n, value: v, ..
            } = e
            {
                if n.eq_ignore_ascii_case(&resistor_name) {
                    *v = value;
                    return true;
                }
            }
            false
        });
        if !found {
            bail!(
                "Resistor '{}' referenced by pot not found in netlist",
                resistor_name
            );
        }
        // Also update PotDirective.default_value so MNA pot_default_overrides
        // doesn't override the element value we just set.
        for p in netlist.pots.iter_mut() {
            if p.resistor_name.eq_ignore_ascii_case(&resistor_name) {
                p.default_value = Some(value);
                break;
            }
        }
        overridden_resistors.insert(resistor_name.to_ascii_uppercase());
        log(&format!("  Pot override: {} = {:.1}", resistor_name, value));
    }

    // Apply .pot defaults for pots not explicitly overridden
    for pot in &netlist.pots {
        if overridden_resistors.contains(&pot.resistor_name.to_ascii_uppercase()) {
            continue;
        }
        if let Some(default) = pot.default_value {
            for e in netlist.elements.iter_mut() {
                if let crate::parser::Element::Resistor {
                    name: n, value: v, ..
                } = e
                {
                    if n.eq_ignore_ascii_case(&pot.resistor_name) {
                        *v = default;
                        break;
                    }
                }
            }
        }
    }

    Ok(())
}

/// Render a resistance the way a netlist author writes it, so a range message
/// reads like the `.pot` line it came from (`100..10000 ohm`, not `1e2..1e4`).
pub fn format_ohms(r: f64) -> String {
    if r >= 1.0 && r.fract() == 0.0 && r < 1e15 {
        format!("{}", r as i64)
    } else {
        format!("{r}")
    }
}
