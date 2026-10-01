use crate::{cache, circuits, sources};
use anyhow::{Context, Result};

/// Frame the normal-path routing decision as information rather than a warning.
///
/// The router's own reason strings are written for maintainers and contain the
/// words "unstable" and "ill-conditioned". A first-time user reading
/// `solver: multi-transformer circuit (3 groups, DK K matrix unstable)` on the
/// SHIPPED demo circuit reasonably concludes they broke something. They did
/// not: melange builds the DK kernel on every route, measures it, and picks
/// whichever of the two solvers can model that circuit correctly. Keep the
/// maintainer detail verbatim — just say out loud that this line is normal.
/// Mirrors the `info (normal):` prefix used in `melange_solver::pipeline`.
fn format_route_info(route_label: &str, reason: &str) -> String {
    // The "why not DK" gloss only makes sense when DK was not chosen.
    let why_not_dk = if route_label.eq_ignore_ascii_case("dk") {
        ")"
    } else {
        " \"unstable\" / \"ill-conditioned\" say\n\
         \x20                 why the DK route was not the fit here; they are not a verdict on the \
         netlist.)"
    };
    format!(
        "  info (normal): solver route = {route_label} \u{2014} {reason}\n\
         \x20                (normal routing output, not a warning: melange measures the DK kernel \
         it just built and\n\
         \x20                 picks the solver that models this circuit correctly.{why_not_dk}"
    )
}

/// The integrator a build used, in words (with what pinned it, if anything).
fn integration_words(integrator: melange_solver::codegen::ir::IntegratorSelection) -> &'static str {
    use melange_solver::codegen::ir::IntegratorSelection as Sel;
    match integrator {
        Sel::TrapDefault => "trapezoidal integration",
        Sel::TrapCliFlag => "trapezoidal integration (--force-trap)",
        Sel::TrapDirective => "trapezoidal integration (.integrator directive)",
        Sel::BeCliFlag => "backward-Euler integration (--backward-euler)",
        Sel::BeDirective => "backward-Euler integration (.integrator directive)",
        Sel::BeBehavioral => "backward-Euler integration (required by behavioral sources)",
        Sel::BeAuto => "backward-Euler integration (chosen automatically)",
    }
}

/// The default (non-verbose) route line: which solver and which integrator the
/// build used, in words a newcomer can read. The reasons and the kernel
/// measurements behind them are `-v` detail; see [`format_route_info`].
pub(crate) fn route_summary(
    solver_label: &str,
    solver_flag: &str,
    sub_path: Option<melange_solver::codegen::NodalSubPath>,
    integrator: melange_solver::codegen::ir::IntegratorSelection,
) -> String {
    let mut solver = solver_label.to_string();
    if let Some(sp) = sub_path {
        solver.push_str(&format!(" ({sp} sub-path)"));
    }
    let chosen_by = if solver_flag == "auto" {
        "chosen automatically".to_string()
    } else {
        format!("forced by --solver {solver_flag}")
    };
    let integration = integration_words(integrator);
    format!("Solver: {solver}, {chosen_by}; {integration}. (-v for why)")
}

/// The build's own progress lines that are solver internals: the numbered
/// steps, matrix sizes, the op-amp rail-mode resolution, which codegen ran,
/// automatic reductions. They explain how melange built the circuit, not
/// anything the user has to act on, so the verbs print them only under `-v`.
/// Matched by exact prefix against the lines `melange_solver::build` and
/// `melange_solver::pipeline` emit; anything not listed here (warnings,
/// hints, refusals, `--pot` echoes, any line added later) prints by default.
const BUILD_DETAIL_PREFIXES: &[&str] = &[
    "Step 1: Parsing SPICE netlist",
    "Step 2: Building MNA system",
    "Step 3: Creating DK kernel",
    "Step 4: Generating Rust code",
    "  \u{2713} Parsed ",
    "  \u{2713} Expanded ",
    "  \u{2713} Matrix dimensions: ",
    "  Input resistance: ",
    "  Input ports: ",
    "  Injection '",
    "  Tap '",
    "  Using augmented MNA for inductors",
    "  DK kernel failed: ",
    "  Auto-selecting nodal solver",
    "  Routing analysis uses the augmented kernel",
    "  Op-amp rail mode",
    "  Using DK codegen with augmented MNA",
    "  Forward-active BJTs: ",
    "  Grid-off pentodes: ",
    "  Linearized ",
    "  Using builtin circuit: ",
];

/// Whether a build/loader line is `-v` detail (see [`BUILD_DETAIL_PREFIXES`]).
pub(crate) fn is_build_detail(line: &str) -> bool {
    if BUILD_DETAIL_PREFIXES.iter().any(|p| line.starts_with(p)) {
        return true;
    }
    // The bare route line is detail; the self-starting-oscillator fallback
    // ("  Using nodal solver codegen: <why>") says why in plain words and
    // prints by default.
    if line == "  Using nodal solver codegen" {
        return true;
    }
    // Step 2's "  ✓ 10 nodes, 2 nonlinear devices".
    line.strip_prefix("  \u{2713} ")
        .is_some_and(|rest| rest.contains(" nodes, ") && rest.ends_with(" nonlinear devices"))
}

/// Route one build line to `emit`, dropping `-v` detail unless `verbose`.
pub(crate) fn report_build_line(verbose: bool, line: std::fmt::Arguments<'_>, emit: fn(&str)) {
    let line = line.to_string();
    if verbose || !is_build_detail(&line) {
        emit(&line);
    }
}

/// What `nr_unconverged_commit_count` counted on this build: a DK Newton solve
/// that ended unconverged, and/or an op-amp rail pin that did. Names the op-amp
/// pin only when the circuit has op-amps.
pub(crate) fn unconverged_commit_source(route_is_dk: bool, has_opamps: bool) -> &'static str {
    match (route_is_dk, has_opamps) {
        (true, false) => "the DK Newton solve",
        (true, true) => "the DK Newton solve (or an op-amp rail pin)",
        (false, true) => "an op-amp rail-pin solve",
        (false, false) => "the final Newton solve",
    }
}

/// The remedy to offer first when a DK-route build left samples unsolved: the
/// nodal route, whose sub-step ladder (DK has only a backward-Euler retry)
/// crosses the regenerative switching edges of oscillators and hard-switching
/// stages. `None` off the DK route.
pub(crate) fn dk_unsolved_remedy(route_is_dk: bool) -> Option<&'static str> {
    route_is_dk.then_some(
        "This build is on the DK solver route, whose only retry is a backward-Euler \
         re-solve. Try `--solver nodal` first: the nodal route adds a sub-step ladder that \
         crosses regenerative switching edges (oscillators, hard-switching stages), which is \
         what usually leaves DK samples unsolved. See \"Circuits with no audio input \
         (oscillators)\" in docs/NETLIST_GUIDE.md.",
    )
}

/// Whether the netlist has an op-amp element.
pub(crate) fn has_opamps(netlist: &melange_solver::parser::Netlist) -> bool {
    netlist
        .elements
        .iter()
        .any(|e| matches!(e, melange_solver::parser::Element::Opamp { .. }))
}

/// The `simulate -v` / `analyze -v` routing detail, through `emit` (stdout for
/// `simulate`, stderr for `analyze`, whose stdout is the CSV).
pub(crate) fn print_run_route_detail(
    built: &melange_solver::build::Built,
    max_iter_pinned: bool,
    emit: &dyn Fn(&str),
) {
    emit(&format_route_info(built.solver_label, &built.solver_reason));
    // Non-negative K diagonal note, printed ONCE (kernel builder logs it at
    // debug only — it is rebuilt several times per run). See compile summary.
    if built.routing.k_diag_unsafe {
        emit(
            "  Note: non-negative K diagonal (positive DK-Schur feedback, \
             expected for transformer-coupled NFB) — handled by nodal full-NR.",
        );
    }
    let meta = &built.generated.meta;
    emit(&format!(
        "  Integration: {}",
        integration_words(meta.integrator_selection)
    ));
    if !meta.integration_reason.is_empty() {
        emit(&format!("    ({})", meta.integration_reason));
    }
    // The budget the emitted `MAX_ITER` carries (the provenance `Build:`
    // line's `max_iter`), not the requested one.
    let how = if max_iter_pinned {
        " (--max-iter)".to_string()
    } else if built.solver_label == "nodal" {
        format!(
            " (auto-tuned; nodal builds ship at least {})",
            melange_solver::codegen::policy::NODAL_MAX_ITER_FLOOR
        )
    } else {
        " (auto-tuned)".to_string()
    };
    emit(&format!("  Max NR iterations: {}{how}", built.max_iter));
}

/// Diagnostic lit sub-step multiplier (`MELANGE_LIT_FACTOR` env var) for the
/// design review demand-3 sweep / lock-margin gate (cross-project review).
/// Deliberately an env var, not a CLI flag — a throwaway diagnostic knob. The
/// resolved value is recorded in the provenance manifest (`lit_factor`), so a
/// swept measurement carries its own build identity. Default 1.0 (= tau, the
/// last tested-safe point; `--subsample-lit-factor`'s help says the same).
pub(crate) fn diag_lit_factor() -> f64 {
    std::env::var("MELANGE_LIT_FACTOR")
        .ok()
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|&f| f > 0.0)
        .unwrap_or(1.0)
}

/// A library build failure as the CLI reports it: the context line over the
/// underlying error, as `anyhow` chains them.
pub(crate) fn build_error(e: melange_solver::build::BuildError) -> anyhow::Error {
    let (context, source) = e.into_parts();
    let err = anyhow::anyhow!(source);
    match context {
        Some(c) => err.context(c),
        None => err,
    }
}

/// [`build_error`] for a build of `netlist`. A `.model` card refusal (an
/// unknown, retired or refused parameter) is raised after parsing, where line
/// numbers are no longer carried, and its context read "Code generation
/// failed"; here the context names the card and the netlist line it starts on
/// instead, the way a parse error does.
pub(crate) fn build_error_in(e: melange_solver::build::BuildError, netlist: &str) -> anyhow::Error {
    // `e` displays as "<context>: <source>".
    let card = model_card_in_error(&e.to_string())
        .and_then(|name| model_card_line(netlist, &name).map(|line| (name, line)));
    let Some((name, line)) = card else {
        return build_error(e);
    };
    let (_, source) = e.into_parts();
    anyhow::anyhow!(source).context(format!(
        "Invalid .model card '{name}' at line {line} of the netlist"
    ))
}

/// The card a `.model` refusal names: the message (after an optional
/// `Label: ` prefix) starts `.model <NAME>`.
fn model_card_in_error(msg: &str) -> Option<String> {
    let at = msg.find(".model ")?;
    if at != 0 && !msg[..at].ends_with(": ") {
        return None;
    }
    let name: String = msg[at + ".model ".len()..]
        .chars()
        .take_while(|c| !c.is_whitespace() && *c != ':')
        .collect();
    (!name.is_empty()).then_some(name)
}

/// The 1-based netlist line a `.model <name>` card starts on (first match,
/// case-insensitive, as the parser resolves model names).
fn model_card_line(netlist: &str, name: &str) -> Option<usize> {
    netlist
        .lines()
        .position(|line| {
            let mut words = line.split_whitespace();
            words
                .next()
                .is_some_and(|w| w.eq_ignore_ascii_case(".model"))
                && words.next().is_some_and(|n| {
                    // `.model NAME D(...)` or `.model NAME(...)`-less forms alike.
                    n.split('(')
                        .next()
                        .is_some_and(|n| n.eq_ignore_ascii_case(name))
                })
        })
        .map(|i| i + 1)
}

/// Whether generated code DECLARES `field` on its state struct.
///
/// Matches the declaration, not the name: generated code can mention a counter
/// in a doc comment without declaring it (the full-LU sub-step ladder's docs
/// name `diag_nr_hold_count` on builds that have no hold), and a substring match
/// then made the simulate driver read a field that does not exist — it failed
/// to compile on every such circuit.
pub(crate) fn declares_state_field(code: &str, field: &str) -> bool {
    code.contains(&format!("pub {field}: "))
}

/// Resolve `--switch NAME=POS` specs into `(switch_idx, position)` pairs.
///
/// `NAME` matches a switch label (case-insensitive) or a 0-based index; `POS`
/// is a 0-based position validated against the switch's `positions`. Shared by
/// `analyze` and `simulate`: both keep the netlist at position-0 element values
/// and apply the override at runtime via `state.set_switch_N(position)`, so the
/// emitted G/C constants stay consistent with `SwitchComponentIR.nominal_value`
/// (mutating `netlist.elements` up front would double-apply the delta).
pub(crate) fn resolve_switch_overrides(
    netlist: &melange_solver::parser::Netlist,
    switch_overrides: &[String],
) -> Result<Vec<(usize, usize)>> {
    let mut resolved = Vec::with_capacity(switch_overrides.len());
    for spec in switch_overrides {
        let (name, pos_str) = spec.split_once('=').ok_or_else(|| {
            anyhow::anyhow!("Invalid --switch format '{}', expected NAME=POS", spec)
        })?;
        let position: usize = pos_str.parse().map_err(|_| {
            anyhow::anyhow!("Invalid switch position '{}' in --switch {}", pos_str, spec)
        })?;

        // Match by label, by any controlled component name, or by index. Many
        // `.switch` directives carry no label (e.g. `.switch Rbyp30 1e9 1.0`),
        // so component-name matching is what makes those reachable by name
        // rather than forcing the caller to count indices.
        let switch_idx = if let Ok(idx) = name.parse::<usize>() {
            if idx >= netlist.switches.len() {
                anyhow::bail!(
                    "Switch index {} out of range (0..{})",
                    idx,
                    netlist.switches.len()
                );
            }
            idx
        } else {
            netlist
                .switches
                .iter()
                .position(|s| {
                    s.label
                        .as_deref()
                        .map(|l| l.eq_ignore_ascii_case(name))
                        .unwrap_or(false)
                        || s.component_names
                            .iter()
                            .any(|c| c.eq_ignore_ascii_case(name))
                })
                .ok_or_else(|| {
                    let available: Vec<String> = netlist
                        .switches
                        .iter()
                        .enumerate()
                        .map(|(i, s)| match s.label.as_deref() {
                            Some(l) => format!("{}: {} ({})", i, l, s.component_names.join("+")),
                            None => format!("{}: {}", i, s.component_names.join("+")),
                        })
                        .collect();
                    anyhow::anyhow!(
                        "Switch '{}' not found. Available: {}",
                        name,
                        available.join(", ")
                    )
                })?
        };

        let sw = &netlist.switches[switch_idx];
        if position >= sw.positions.len() {
            anyhow::bail!(
                "Switch '{}' position {} out of range (0..{})",
                name,
                position,
                sw.positions.len()
            );
        }

        resolved.push((switch_idx, position));
        eprintln!(
            "  Switch override: {} = position {}",
            sw.label.as_deref().unwrap_or(name),
            position
        );
    }
    Ok(resolved)
}

/// The generated code's input-sanitisation counters, printed as `DIAG:` lines
/// by the simulate and analyze harnesses (presence-filtered per build).
pub(crate) const INPUT_DIAG_FIELDS: [&str; 2] = ["diag_input_clamp_count", "diag_input_nan_count"];

/// Fail when the circuit was not driven with the requested input: samples
/// clamped to `INPUT_LIMIT_V`, or NaN/Inf samples replaced by 0. Like an NR
/// hold, the output then looks healthy while answering a different question
/// (design review). `allow` is the explicit override.
pub(crate) fn refuse_on_input_diag(stderr: &str, allow: bool) -> Result<()> {
    let count = |key: &str| -> u64 {
        stderr
            .lines()
            .filter_map(|l| l.strip_prefix("DIAG:"))
            .filter_map(|d| d.strip_prefix(key))
            .filter_map(|v| v.strip_prefix('='))
            .filter_map(|v| v.trim().parse::<u64>().ok())
            .next_back()
            .unwrap_or(0)
    };
    let (clamped, nan) = (count("input_clamp_count"), count("input_nan_count"));
    if clamped == 0 && nan == 0 {
        return Ok(());
    }
    eprintln!();
    if clamped > 0 {
        eprintln!(
            "ERROR: {clamped} input sample(s) exceeded the generated code's input limit \
             (INPUT_LIMIT_V = 100 V) and were clamped to it. The circuit was driven with a \
             clipped input, not the one requested, and nothing in the output shows it."
        );
    }
    if nan > 0 {
        eprintln!("ERROR: {nan} input sample(s) were NaN or infinite and were replaced by 0 V.");
    }
    if !allow {
        anyhow::bail!(
            "the circuit was not driven with the requested input ({clamped} clamped, {nan} \
             NaN/Inf; --allow-input-clamp to override)"
        );
    }
    eprintln!("(--allow-input-clamp given: continuing.)");
    Ok(())
}

/// A resolved circuit's netlist text, however it was referenced: builtin,
/// local file, or remote (with [`fetch_remote_circuit`]'s stale-index
/// self-heal). The one loader every verb uses.
pub(crate) fn load_circuit_text(
    src: &circuits::CircuitSource,
    report: &dyn Fn(&str),
) -> Result<String> {
    match src {
        circuits::CircuitSource::Builtin { content, name } => {
            report(&format!("  Using builtin circuit: {}", name));
            Ok(content.clone())
        }
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read local file: {}", path.display())),
        circuits::CircuitSource::Url { .. } | circuits::CircuitSource::Friendly { .. } => {
            fetch_remote_circuit(src, report)
        }
    }
}

/// Fetch a resolved circuit's content, self-healing a stale index.
///
/// A 404 on a path an index gave us means the cached index is stale — the deck
/// moved tier since we last fetched it. Refetch the index once and retry, so a
/// promotion self-heals instead of needing `melange cache clear`. Only fires
/// for indexed sources: an unindexed one re-resolves to the same flat URL and
/// returns the same 404 without a wasted index request.
///
/// Reached only through [`load_circuit_text`], which every verb reads its
/// circuit with, so no verb can drift into skipping the self-heal. `report`
/// carries the progress lines (stderr for verbs whose stdout is data).
fn fetch_remote_circuit(src: &circuits::CircuitSource, report: &dyn Fn(&str)) -> Result<String> {
    let (url, indexed) = match src {
        circuits::CircuitSource::Url { url } => (url, None),
        circuits::CircuitSource::Friendly {
            url,
            source,
            circuit,
        } => (url, Some((source, circuit))),
        _ => anyhow::bail!("fetch_remote_circuit called on a non-remote source"),
    };
    report(&format!("  Fetching from URL: {}", url));
    let cache = cache::Cache::new()?;
    match cache.get_sync(url, false) {
        Ok(c) => Ok(c),
        Err(e) if e.downcast_ref::<cache::NotFound>().is_some() => {
            let Some((source, circuit)) = indexed else {
                return Err(e);
            };
            let config = sources::SourcesConfig::load()?;
            let fresh = config.resolve_circuit_indexed(source, circuit, &cache, true)?;
            if &fresh == url {
                return Err(e);
            }
            report(&format!("  Index was stale; refetched: {}", fresh));
            cache.get_sync(&fresh, false)
        }
        Err(e) => Err(e),
    }
}

#[cfg(test)]
mod model_card_error_tests {
    use super::{model_card_in_error, model_card_line};

    #[test]
    fn a_model_card_refusal_is_located_in_the_netlist() {
        let msg = "Invalid config: .model 1N4148 (diode card): unknown parameter 'BOGUS'.";
        assert_eq!(model_card_in_error(msg).as_deref(), Some("1N4148"));
        let deck = "* t\nD1 in out 1N4148\nR1 out 0 1k\n.MODEL 1n4148 D(IS=1e-14\n+ BOGUS=3)\n";
        assert_eq!(model_card_line(deck, "1N4148"), Some(4));
        assert_eq!(model_card_line(deck, "1N914"), None);
        // A message that only mentions a card mid-sentence is not a card refusal.
        assert_eq!(model_card_in_error("see the .model card"), None);
    }
}

#[cfg(test)]
mod declares_state_field_tests {
    use super::declares_state_field;

    #[test]
    fn a_doc_comment_mention_is_not_a_declaration() {
        let code = "/// Past this depth the hold fires and `diag_nr_hold_count` counts it.\n\
                    pub struct CircuitState {\n    pub diag_be_fallback_count: u64,\n}\n";
        assert!(!declares_state_field(code, "diag_nr_hold_count"));
        assert!(declares_state_field(code, "diag_be_fallback_count"));
    }
}
