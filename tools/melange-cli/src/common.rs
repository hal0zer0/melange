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
    emit(&format!(
        "  Max NR iterations: {}{}",
        built.max_iter,
        if max_iter_pinned {
            " (--max-iter)"
        } else {
            " (auto-tuned)"
        }
    ));
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
