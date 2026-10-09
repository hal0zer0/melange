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
    /// With a runtime set, this is the default factor.
    pub oversampling: Option<usize>,
    /// The runtime-selectable factor set: the deck's `.oversampling ... allow=`,
    /// none, or one given on the command line.
    pub oversampling_set: OversamplingSet,
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
    /// Parse isothermal: no device self-heating (validate: the reference has
    /// no thermal model). `TAMB` still sets static device temperatures.
    pub disable_self_heating: bool,
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

/// Which runtime oversampling set a build uses (`set_oversampling`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum OversamplingSet {
    /// The deck's `.oversampling N allow=...`, if it has one (`compile`).
    #[default]
    Deck,
    /// None: the factor is fixed at build time, whatever the deck declares
    /// (`--oversampling-set off`; the verbs that run one fixed factor).
    Off,
    /// This set, overriding the deck's (`--oversampling-set 1,2,4`).
    Set(Vec<usize>),
    /// A runtime build's internal fixed build of one factor of its set: as
    /// `Off`, and the factor is the set's, not a user's `--oversampling`
    /// (no "below the recommendation by request" warning).
    #[doc(hidden)]
    FactorOfSet,
}

/// A resolved runtime oversampling set.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedOversamplingSet {
    /// Ascending, at least two.
    pub factors: Vec<usize>,
    /// The factor a fresh state runs at; in `factors`.
    pub default: usize,
    /// `"directive"` or `"cli"`.
    pub source: &'static str,
    /// The deck's `.oversampling` recommendation, if any.
    pub recommended: Option<usize>,
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
    /// The Newton budget that ships: the emitted `MAX_ITER`
    /// ([`crate::codegen::ir::CircuitIR::effective_max_iter`]).
    pub max_iter: usize,
    pub oversampling: usize,
    /// The runtime oversampling set, when the build has one (`oversampling`
    /// is then its default factor).
    pub oversampling_set: Option<ResolvedOversamplingSet>,
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
    /// The Newton budget that ships: the emitted `MAX_ITER`
    /// ([`crate::codegen::ir::CircuitIR::effective_max_iter`]).
    pub max_iter: usize,
    pub oversampling: usize,
    /// The runtime oversampling set, when the build has one (`oversampling`
    /// is then its default factor).
    pub oversampling_set: Option<ResolvedOversamplingSet>,
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
        disable_self_heating: opts.disable_self_heating,
    };
    let mut netlist = Netlist::parse_with_options(netlist_str, parse_options)
        .with_context(|| "Failed to parse SPICE netlist")?;

    // Resolve the effective oversampling factor: explicit --oversampling wins,
    // else the deck's `.oversampling` recommendation, else 1.
    let oversampling = if opts.oversampling_set == OversamplingSet::FactorOfSet {
        // One factor of a runtime set: the set chose it, not a user flag.
        oversampling_cli.unwrap_or(1)
    } else {
        resolve_oversampling(oversampling_cli, netlist.recommended_oversampling)
    };
    let oversampling_set = resolve_oversampling_set(
        &opts.oversampling_set,
        netlist.oversampling_set.as_deref(),
        oversampling,
        netlist.recommended_oversampling,
    )?;

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
        if let Some(set) = &oversampling_set {
            bail!(
                "multi-input decks do not yet support oversampling, and this build asks for \
                 runtime factors {:?} (`.oversampling ... allow=` or --oversampling-set). \
                 Linear (M=0) circuits do not alias; build with --oversampling-set off.",
                set.factors
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
            let host_rate = inj.rate == crate::parser::InjectRate::Host;
            report!(
                out,
                "  Injection '{}' at node '{}' ({}={} ohm, rate={})",
                inj.field_name,
                inj.node,
                kind,
                resistance,
                if host_rate { "host" } else { "inner" }
            );
            injection_specs.push(crate::codegen::ir::InjectionSpec {
                node: idx,
                name: inj.field_name.clone(),
                resistance,
                norton,
                host_rate,
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
                "  WARNING: pot(s) {} sit within one component of the input, which is driven \
                 from a {:.0}Ω source.",
                connected_pots.join(", "),
                input_resistance
            );
            report!(
                err,
                "  A passive network's response depends on the impedance driving it, and a \
                 near-ideal source can leave its pots with little effect."
            );
            report!(
                err,
                "  If the real circuit is driven from something else (a 600Ω line output, a \
                 tube plate of tens of kΩ), set that impedance:"
            );
            report!(
                err,
                "    .input_impedance <ohms>   (in the netlist)   or   --input-resistance <ohms>"
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
                "  Hint: a resistor over 10kΩ sits on the input node, driven from a near-ideal source. If the real circuit is driven from a higher impedance, set it to the driver's output impedance: .input_impedance <ohms> or --input-resistance <ohms>"
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
        let device_slots = crate::codegen::ir::CircuitIR::build_device_info(&netlist)
            .with_context(|| "Code generation failed")?;
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
    // equilibrium on push-pull topologies (observed on a push-pull power amp).
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
    if let Some(what) = &linearize_outcome.bias_unconverged {
        if opts.allow_unconverged_dc_op {
            report!(
                err,
                "  WARNING: {what}. Building anyway (--allow-unconverged-dc-op): the linearized \
                 devices' small-signal parameters come from a point that is not a solution."
            );
        } else {
            bail!(
                "{what}: the linearized devices' small-signal parameters would come from a \
                 point that is not a solution. --allow-unconverged-dc-op builds it anyway."
            );
        }
    }

    // NOTE: Internal node expansion for parasitic BJTs is deferred until after
    // solver routing. The DK path handles parasitics via bjt_with_parasitics() inner NR.
    // Only the nodal path benefits from MNA-level internal nodes (eliminates inner NR).

    // BJT junction-cap preflight: solve DC OP on the fully-reduced MNA and
    // re-linearize CJE/CJC at Vbe_op/Vbc_op, then add the diffusion-cap
    // contribution `TF · d(I_F/qb)/dVbe`. No-op when every BJT uses the SPICE
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
    // appears once oversampling raises alpha. For os=1 this is
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
                    #[allow(deprecated)] // the companion-inductor fields stay empty
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
    if use_nodal_codegen {
        refuse_max_iter_below_nodal_floor(user_max_iter)?;
    }
    let mut solver_label = if use_nodal_codegen { "nodal" } else { "DK" };
    let mut solver_reason = if solver_override == "nodal" || solver_override == "dk" {
        // Forced route: still surface what the auto-router decided so the pinned
        // sub-path is visible, not masked by the override reason (a measuring
        // verb must see the route it is actually on).
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
                refuse_max_iter_below_nodal_floor(user_max_iter)?;
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

    // The budget the emitted `MAX_ITER` carries (the nodal floor and a
    // backward-Euler promotion's budget applied), so the console reports what
    // the provenance `Build:` line and the code say.
    let max_iter = prepared.ir.effective_max_iter();

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
        oversampling_set,
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

/// Refuse a `--max-iter` pin below the nodal Newton budget floor
/// ([`crate::codegen::policy::NODAL_MAX_ITER_FLOOR`]). The nodal route would
/// raise it to the floor anyway, so the pin would not be the budget that ships;
/// an auto-tuned budget is raised silently, as it always was.
fn refuse_max_iter_below_nodal_floor(user_max_iter: Option<usize>) -> Result<(), BuildError> {
    let floor = crate::codegen::policy::NODAL_MAX_ITER_FLOOR;
    match user_max_iter {
        // 0 is refused on every route by `CodegenConfig::validate`.
        Some(n) if (1..floor).contains(&n) => bail!(
            "--max-iter {n} is below the nodal solver's Newton budget floor of {floor} \
             iterations per sample. The nodal Newton is globalized by an Armijo line \
             search, which crosses a device's saturation knee in many short steps; \
             with fewer, a full-scale transient can end a sample unsolved, so a nodal \
             build never ships less. Pass --max-iter {floor} or more, or omit it \
             (auto-tuned)."
        ),
        _ => Ok(()),
    }
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
    if let Some(set) = a.oversampling_set.clone() {
        return build_runtime_oversampling(netlist_str, opts, &set, out, err);
    }
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
        oversampling_set: None,
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

/// Provenance keys that may differ between the factors of a runtime set:
/// the factor itself and the ring predicate's per-rate verdict text. Every
/// other key describes the solver the code is (route, sub-path, integrator,
/// latch, rail mode, reductions, budget), so it must agree across the set;
/// a key added later is held to that by default.
const PER_FACTOR_PROVENANCE_KEYS: [&str; 3] = [
    "oversampling",
    "integration_reason",
    // The largest rounding-noise ratio met while settling the structural
    // sparsity (LINEAR_ALGEBRA.md "Structural Sparsity"): a property of each
    // factor's inversion, not of the solver's structure.
    "sparsity_noise_ratio",
];

/// The top-level `(key, raw value text)` pairs of generated code's
/// `// provenance: {...}` line, in order. The line is melange's own flat JSON
/// object (values: strings, numbers, booleans, or nested objects compared as
/// text), so a quote-and-depth-aware split is exact for it.
fn provenance_of(code: &str) -> Result<Vec<(String, String)>, BuildError> {
    let line = code
        .lines()
        .find_map(|l| l.strip_prefix("// provenance: "))
        .ok_or_else(|| err!("generated code carries no provenance line"))?
        .trim();
    let body = line
        .strip_prefix('{')
        .and_then(|b| b.strip_suffix('}'))
        .ok_or_else(|| err!("generated provenance is not a JSON object: {line}"))?;
    // Split at top-level commas: outside strings and nested braces/brackets.
    let (mut fields, mut cur) = (Vec::new(), String::new());
    let (mut depth, mut in_str, mut escaped) = (0i32, false, false);
    for c in body.chars() {
        if in_str {
            in_str = !(c == '"' && !escaped);
            escaped = c == '\\' && !escaped;
        } else {
            match c {
                '"' => in_str = true,
                '{' | '[' => depth += 1,
                '}' | ']' => depth -= 1,
                ',' if depth == 0 => {
                    fields.push(std::mem::take(&mut cur));
                    continue;
                }
                _ => {}
            }
        }
        cur.push(c);
    }
    if !cur.trim().is_empty() {
        fields.push(cur);
    }
    fields
        .into_iter()
        .map(|f| {
            let (k, v) = f
                .split_once(':')
                .ok_or_else(|| err!("malformed provenance field: {f}"))?;
            Ok((k.trim().trim_matches('"').to_string(), v.trim().to_string()))
        })
        .collect()
}

/// Generated constants the runtime code switches per factor: the baked
/// matrices and DC-blocker coefficient a same-rate restore loads, and the
/// runtime BE latch's reference. Each is read through a per-factor accessor
/// wherever the running factor matters.
pub const SWITCHED_PER_FACTOR: [&str; 15] = [
    "S_DEFAULT",
    "A_DEFAULT",
    "A_NEG_DEFAULT",
    "K_DEFAULT",
    "S_NI_DEFAULT",
    "S_BE_DEFAULT",
    "A_BE_DEFAULT",
    "A_NEG_BE_DEFAULT",
    "K_BE_DEFAULT",
    "S_NI_BE_DEFAULT",
    "DC_BLOCK_R",
    "BE_LATCH_RING_POLES",
    "BE_LATCH_BE_COST_REL",
    "BE_LATCH_PASSBAND_GAIN",
    "BE_LATCH_RING_HOLD",
];

/// Generated constants that ARE the factor or its filters: the runtime code
/// replaces them with the running factor and a cascade per factor. The
/// public `INTERNAL_SAMPLE_RATE` and `ALPHA` (2/T at it) are informational,
/// read by no generated code; a runtime build's describe its default factor,
/// the one a fresh state runs at.
const FACTOR_ITSELF: [&str; 5] = [
    "OVERSAMPLING_FACTOR",
    "INTERNAL_SAMPLE_RATE",
    "ALPHA",
    "OS_COEFFS",
    "OS_COEFFS_OUTER",
];

/// Every `const NAME: TYPE = VALUE;` of generated code, in order, as
/// `(name, type, value text)`.
fn generated_consts(code: &str) -> Vec<(String, String, String)> {
    crate::codegen::const_text::const_items(code)
        .into_iter()
        .map(|c| (c.name, c.ty, c.value))
        .collect()
}

/// Generated code reduced to what executes, for comparing two factors'
/// runtime emissions: blank and comment lines dropped (the header carries the factor
/// and the ring predicate's per-rate verdict) and every [`FACTOR_ITSELF`]
/// constant dropped (each factor's build describes itself there, and a 1x
/// build emits no `INTERNAL_SAMPLE_RATE`). Every other byte is the solver.
fn executable_text(code: &str) -> String {
    let mut masked = String::with_capacity(code.len());
    let mut at = 0;
    for c in crate::codegen::const_text::const_items(code) {
        if FACTOR_ITSELF.contains(&c.name.as_str()) {
            masked.push_str(&code[at..c.ty_range.start]);
            masked.push('_');
            masked.push_str(&code[c.ty_range.end..c.value_range.start]);
            masked.push('_');
            at = c.value_range.end;
        }
    }
    masked.push_str(&code[at..]);
    masked
        .lines()
        .filter(|l| {
            let item = l.trim_start();
            let item = item.strip_prefix("pub ").unwrap_or(item);
            !item.is_empty()
                && !item.starts_with("//")
                && !FACTOR_ITSELF
                    .iter()
                    .any(|n| item.starts_with(&format!("const {n}: _ = _;")))
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// The first line at which two factors' executable texts differ: its 1-based
/// line number and the two lines (`<end>` for the text that has run out).
/// `None` when they are the same solver.
fn first_executable_difference<'a>(
    reference: &'a str,
    text: &'a str,
) -> Option<(usize, &'a str, &'a str)> {
    let (mut r, mut t) = (reference.lines(), text.lines());
    let mut line = 0;
    loop {
        line += 1;
        match (r.next(), t.next()) {
            (None, None) => return None,
            (Some(x), Some(y)) if x == y => {}
            (x, y) => return Some((line, x.unwrap_or("<end>"), y.unwrap_or("<end>"))),
        }
    }
}

/// The constants whose value differs between the factors' fixed builds
/// (`codes`, in `factors` order), each with its literal per factor. Refuses a
/// differing constant the runtime code does not switch per factor: it would
/// run at the default factor's value at every factor.
fn per_factor_consts(
    factors: &[usize],
    codes: &[String],
) -> Result<Vec<crate::codegen::ir::PerFactorConst>, BuildError> {
    let tables: Vec<Vec<(String, String, String)>> =
        codes.iter().map(|c| generated_consts(c)).collect();
    let mut names: Vec<String> = Vec::new();
    for t in &tables {
        for (n, _, _) in t {
            if !names.contains(n) {
                names.push(n.clone());
            }
        }
    }
    let mut out = Vec::new();
    for name in names {
        if FACTOR_ITSELF.contains(&name.as_str()) {
            continue;
        }
        let entries: Vec<Option<&(String, String, String)>> = tables
            .iter()
            .map(|t| t.iter().find(|(n, _, _)| *n == name))
            .collect();
        let present: Vec<&(String, String, String)> = entries.iter().flatten().copied().collect();
        let same = present.len() == entries.len()
            && present
                .windows(2)
                .all(|w| w[0].1 == w[1].1 && w[0].2 == w[1].2);
        if same {
            continue;
        }
        if present.len() != entries.len() || !SWITCHED_PER_FACTOR.contains(&name.as_str()) {
            let at: Vec<String> = factors
                .iter()
                .zip(&entries)
                .map(|(f, e)| match e {
                    Some((_, _, v)) if v.len() <= 40 => format!("{f}x: {v}"),
                    Some((_, _, v)) => format!("{f}x: <{}-char literal>", v.len()),
                    None => format!("{f}x: absent"),
                })
                .collect();
            bail!(
                "the runtime oversampling set {factors:?} is refused: the generated constant \
                 `{name}` differs between factors ({}) and the runtime code does not switch it \
                 per factor, so every factor would run with the default factor's value",
                at.join(", ")
            );
        }
        out.push(crate::codegen::ir::PerFactorConst {
            name: name.clone(),
            types: present.iter().map(|(_, t, _)| t.clone()).collect(),
            values: present.iter().map(|(_, _, v)| v.clone()).collect(),
        });
    }
    Ok(out)
}

/// Build a deck whose oversampling factor is selectable at runtime.
///
/// Each factor of the set is assembled and emitted as the fixed build it would
/// be, with the Newton budget pinned to the largest any factor auto-tunes. The
/// set is refused unless those builds are the same solver (their provenance
/// agrees on every key but [`PER_FACTOR_PROVENANCE_KEYS`]): the runtime code
/// is one solver whose rate-dependent values are switched per factor, so a
/// factor that would route or integrate differently cannot be one of its
/// settings. The runtime code is emitted from the default factor's build,
/// carrying each factor's settled values.
fn build_runtime_oversampling(
    netlist_str: &str,
    opts: &BuildOptions,
    set: &ResolvedOversamplingSet,
    out: Reporter<'_>,
    err: Reporter<'_>,
) -> Result<Built, BuildError> {
    let quiet: Reporter<'_> = &|_| {};
    let fixed = |n: usize, max_iter: Option<usize>| BuildOptions {
        oversampling: Some(n),
        oversampling_set: OversamplingSet::FactorOfSet,
        max_iter,
        ..opts.clone()
    };
    let assemble_all = |max_iter: Option<usize>| -> Result<Vec<Assembled>, BuildError> {
        set.factors
            .iter()
            .map(|&n| assemble(netlist_str, &fixed(n, max_iter), quiet, quiet))
            .collect()
    };
    let mut factors = assemble_all(opts.max_iter)?;
    let budget = factors.iter().map(|a| a.max_iter).max().unwrap_or(0);
    if factors.iter().any(|a| a.max_iter != budget) {
        factors = assemble_all(Some(budget))?;
    }
    // Each factor as the fixed build it would be; their provenance must agree.
    let mut provenance = Vec::with_capacity(factors.len());
    let mut codes = Vec::with_capacity(factors.len());
    for a in &factors {
        let code = CodeGenerator::new(a.config.clone())
            .emit(&a.prepared)
            .with_context(|| format!("code generation at {}x failed", a.oversampling))?
            .code;
        provenance.push(provenance_of(&code)?);
        codes.push(code);
    }
    let get = |p: &[(String, String)], key: &str| {
        p.iter().find(|(k, _)| k == key).map(|(_, v)| v.clone())
    };
    let reference = &provenance[0];
    for (a, p) in factors.iter().zip(&provenance).skip(1) {
        let keys: std::collections::BTreeSet<&String> =
            reference.iter().chain(p.iter()).map(|(k, _)| k).collect();
        let show = |v: Option<String>| v.unwrap_or_else(|| "absent".to_string());
        let differences: Vec<String> = keys
            .into_iter()
            .filter(|key| !PER_FACTOR_PROVENANCE_KEYS.contains(&key.as_str()))
            .filter_map(|key| {
                let (x, y) = (get(reference, key), get(p, key));
                (x != y).then(|| {
                    format!(
                        "{key}: {} at {}x, {} at {}x",
                        show(x),
                        set.factors[0],
                        show(y),
                        a.oversampling
                    )
                })
            })
            .collect();
        if !differences.is_empty() {
            bail!(
                "the runtime oversampling set {:?} is refused: one solver cannot serve it, \
                 because at {}x the build differs from {}x in {}. Drop the factor from \
                 allow= (or --oversampling-set), or build each factor as its own deck.",
                set.factors,
                a.oversampling,
                set.factors[0],
                differences.join("; ")
            );
        }
    }
    let per_factor = factors
        .iter()
        .map(|a| crate::codegen::ir::FactorSettlement {
            factor: a.oversampling,
            integration_reason: a.prepared.ir.integration_reason.clone(),
            sparsity_noise_ratio: a.prepared.ir.sparsity.noise_ratio,
        })
        .collect();
    let per_factor_consts = per_factor_consts(&set.factors, &codes)?;
    let runtime = crate::codegen::ir::RuntimeOversampling {
        factors: set.factors.clone(),
        default: set.default,
        source: set.source.to_string(),
        recommended: set.recommended,
        per_factor,
        per_factor_consts,
    };
    // The runtime code is emitted from one factor's IR, so the emission must
    // not depend on which: an IR whose structure differed by factor would run
    // the default factor's structure at every factor. The emitted matrix
    // patterns are structural (the same at every rate), so a difference here
    // is a genuine one that provenance and the switched constants do not see.
    // Emit it from every factor's IR and require the same executable text.
    let mut emissions = Vec::with_capacity(factors.len());
    for a in &factors {
        let mut prepared = a.prepared.clone();
        prepared.ir.solver_config.runtime_oversampling = Some(runtime.clone());
        let context = if a.solver_label == "nodal" {
            "Nodal code generation failed"
        } else {
            "Code generation failed"
        };
        emissions.push(
            CodeGenerator::new(a.config.clone())
                .emit(&prepared)
                .with_context(|| {
                    format!("{context} (runtime oversampling, {}x)", a.oversampling)
                })?,
        );
    }
    let default_idx = set
        .factors
        .iter()
        .position(|&n| n == set.default)
        .expect("resolve_oversampling_set: the default is in the set");
    let reference = executable_text(&emissions[default_idx].code);
    for (a, e) in factors.iter().zip(&emissions) {
        let text = executable_text(&e.code);
        if let Some((line, x, y)) = first_executable_difference(&reference, &text) {
            bail!(
                "the runtime oversampling set {:?} is refused: the solver's structure differs \
                 between {}x and {}x (the code emitted from each factor's build first differs \
                 at executable line {}: `{}` against `{}`), so one solver cannot serve both. \
                 Drop the factor from allow= (or --oversampling-set), or build each factor as \
                 its own deck.",
                set.factors,
                set.default,
                a.oversampling,
                line,
                x.trim(),
                y.trim()
            );
        }
    }
    let generated = emissions.swap_remove(default_idx);
    let a = factors.swap_remove(default_idx);
    report!(
        out,
        "  Oversampling: runtime-selectable {:?} (default {}x, from the {}), one solver at every factor",
        set.factors,
        set.default,
        set.source
    );
    let _ = err;
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
        oversampling_set: Some(set.clone()),
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

/// Resolve the runtime oversampling set (`set_oversampling`): the command
/// line's, else the deck's `.oversampling N allow=...`, else none. The default
/// factor must be in the set: a set without a resolvable default is refused,
/// not guessed.
pub fn resolve_oversampling_set(
    choice: &OversamplingSet,
    deck: Option<&[usize]>,
    default: usize,
    recommended: Option<usize>,
) -> Result<Option<ResolvedOversamplingSet>, BuildError> {
    let (factors, source) = match (choice, deck) {
        (OversamplingSet::Off | OversamplingSet::FactorOfSet, _)
        | (OversamplingSet::Deck, None) => return Ok(None),
        (OversamplingSet::Deck, Some(d)) => (d.to_vec(), "directive"),
        (OversamplingSet::Set(cli), deck) => {
            let mut f = cli.clone();
            f.sort_unstable();
            f.dedup();
            if f.len() < 2 || f.len() != cli.len() || f.iter().any(|n| !matches!(n, 1 | 2 | 4)) {
                bail!(
                    "--oversampling-set takes two or three distinct factors from 1, 2, 4, \
                     got {cli:?}"
                );
            }
            if let Some(d) = deck {
                if d != f.as_slice() {
                    crate::diag_warn!(
                        "--oversampling-set {f:?} overrides the deck's .oversampling allow={d:?}"
                    );
                }
            }
            (f, "cli")
        }
    };
    if !factors.contains(&default) {
        bail!(
            "the default oversampling factor {default} (the deck's .oversampling, or \
             --oversampling) is not in the runtime set {factors:?}"
        );
    }
    Ok(Some(ResolvedOversamplingSet {
        factors,
        default,
        source,
        recommended,
    }))
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
        mna.relinearize_bjt_caps_at_dc_op(&device_slots, &dc.v_nl, &dc.v_node);
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

#[cfg(test)]
mod runtime_oversampling_guard_tests {
    //! The one-solver check on synthetic emissions. With the emitted matrix
    //! patterns chosen by the circuit's structure, no in-repo deck emits a
    //! different solver at a different rate, so the check is exercised here on
    //! text shaped like two factors' builds.
    use super::{executable_text, first_executable_difference};

    /// A fixed build at `factor`, reduced to the lines that matter: it
    /// describes itself in comments and the factor-itself constants, then
    /// carries the solver (`terms`).
    fn emission(factor: usize, terms: &[&str]) -> String {
        let mut s = format!(
            "// melange: generated\n// provenance: {{\"oversampling\":{factor}}}\n\n\
             pub const OVERSAMPLING_FACTOR: usize = {factor};\n"
        );
        if factor > 1 {
            s.push_str(&format!(
                "pub const INTERNAL_SAMPLE_RATE: f64 = {}.0;\n\n\
                 pub const OS_COEFFS: [f64; 2] = [0.1, 0.2];\n",
                48000 * factor
            ));
        }
        s.push_str("pub const MAX_ITER: usize = 40;\n    // the Newton step\n");
        for t in terms {
            s.push_str(&format!("        {t}\n"));
        }
        s
    }

    #[test]
    fn executable_text_keeps_only_the_solver() {
        let text = executable_text(&emission(2, &["v_d[0] += k_be[0][5] * i_nl[5];"]));
        assert_eq!(
            text,
            "pub const MAX_ITER: usize = 40;\n        v_d[0] += k_be[0][5] * i_nl[5];"
        );
    }

    #[test]
    fn factors_that_differ_only_in_describing_themselves_are_one_solver() {
        let terms = [
            "v_d[0] += k_be[0][5] * i_nl[5];",
            "v_d[1] += k_be[1][2] * i_nl[2];",
        ];
        let one = executable_text(&emission(1, &terms));
        for f in [2, 4] {
            let other = executable_text(&emission(f, &terms));
            assert_eq!(first_executable_difference(&one, &other), None, "{f}x");
        }
    }

    #[test]
    fn a_term_present_at_one_factor_only_is_named() {
        let both = [
            "v_d[0] += k_be[0][5] * i_nl[5];",
            "v_d[1] += k_be[1][2] * i_nl[2];",
        ];
        let one = ["v_d[1] += k_be[1][2] * i_nl[2];"];
        let a = executable_text(&emission(2, &both));
        let b = executable_text(&emission(1, &one));
        assert_eq!(
            first_executable_difference(&a, &b),
            Some((
                2,
                "        v_d[0] += k_be[0][5] * i_nl[5];",
                "        v_d[1] += k_be[1][2] * i_nl[2];"
            ))
        );
        // A text that runs out is reported against the other's extra line.
        let extra = [
            "v_d[1] += k_be[1][2] * i_nl[2];",
            "v_d[0] += k_be[0][5] * i_nl[5];",
        ];
        let c = executable_text(&emission(1, &extra));
        assert_eq!(
            first_executable_difference(&b, &c),
            Some((3, "<end>", "        v_d[0] += k_be[0][5] * i_nl[5];"))
        );
    }
}
