//! `.inject` / `.tap` emission plumbing shared by the DK and nodal paths.

use tera::Context;

use super::helpers::fmt_f64;
use crate::codegen::ir::CircuitIR;

/// Insert the `.inject` / `.tap` Tera variables (shared by both DK and nodal
/// template consumers: constants, state, build_rhs, process_sample).
///
/// `inject_or_tap` gates every API-shape divergence: when it is false (no
/// `.inject` and no `.tap`), the emitted `process_sample(input, state)` and
/// every related fragment are byte-identical to the pre-inject path. `.inject`
/// and multi-input are mutually exclusive (rejected at the CLI), so the two
/// context helpers never both activate their divergent branches.
pub(super) fn insert_inject_ctx(ctx: &mut Context, ir: &CircuitIR) {
    let inj = &ir.solver_config.injections;
    let taps = &ir.solver_config.taps;
    let has_inject = !inj.is_empty();
    let has_tap = !taps.is_empty();
    ctx.insert("has_inject", &has_inject);
    ctx.insert("has_tap", &has_tap);
    ctx.insert("inject_or_tap", &(has_inject || has_tap));
    ctx.insert("num_inject", &inj.len());
    ctx.insert("num_tap", &taps.len());
    ctx.insert(
        "inject_nodes_values",
        &inj.iter()
            .map(|i| i.node.to_string())
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "inject_names_values",
        &inj.iter()
            .map(|i| format!("{:?}", i.name))
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "inject_resistances_values",
        &inj.iter()
            .map(|i| fmt_f64(i.resistance))
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "inject_is_norton_values",
        &inj.iter()
            .map(|i| i.norton.to_string())
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "tap_nodes_values",
        &taps
            .iter()
            .map(|t| t.node.to_string())
            .collect::<Vec<_>>()
            .join(", "),
    );
    ctx.insert(
        "tap_names_values",
        &taps
            .iter()
            .map(|t| format!("{:?}", t.name))
            .collect::<Vec<_>>()
            .join(", "),
    );
}

/// Emit the `.inject` / `.tap` constant block for the NODAL path (which builds
/// constants inline rather than via `constants.rs.tera`). Byte-for-byte the
/// same text the template emits under `{% if inject_or_tap %}`, so the two
/// paths stay in lockstep. Empty when there is no `.inject`/`.tap`.
pub(super) fn emit_inject_tap_constants(ir: &CircuitIR) -> String {
    let inj = &ir.solver_config.injections;
    let taps = &ir.solver_config.taps;
    if inj.is_empty() && taps.is_empty() {
        return String::new();
    }
    let inject_nodes = inj
        .iter()
        .map(|i| i.node.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let inject_names = inj
        .iter()
        .map(|i| format!("{:?}", i.name))
        .collect::<Vec<_>>()
        .join(", ");
    let inject_res = inj
        .iter()
        .map(|i| fmt_f64(i.resistance))
        .collect::<Vec<_>>()
        .join(", ");
    let inject_norton = inj
        .iter()
        .map(|i| i.norton.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let tap_nodes = taps
        .iter()
        .map(|t| t.node.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let tap_names = taps
        .iter()
        .map(|t| format!("{:?}", t.name))
        .collect::<Vec<_>>()
        .join(", ");
    format!(
        "\n// -----------------------------------------------------------------------------\n\
         // Runtime feedback injection (`.inject`) + raw inner-rate taps (`.tap`).\n\
         //\n\
         // When either directive is present `process_sample` takes per-inner-sample\n\
         // injection arrays and returns per-inner-sample raw taps (see its doc-comment).\n\
         // Both counts are always emitted so the API shape is uniform; one may be 0.\n\
         // -----------------------------------------------------------------------------\n\n\
         /// Number of runtime feedback-injection sources (`.inject`). May be 0.\n\
         pub const NUM_INJECT: usize = {num_inject};\n\n\
         /// Injection node indices (0-indexed), one per `.inject` source, in directive\n\
         /// order — the SAME order as the `injections` argument to `process_sample`.\n\
         pub const INJECT_NODES: [usize; NUM_INJECT] = [{inject_nodes}];\n\n\
         /// Injection source names (INJECT order), for the caller's index mapping.\n\
         pub const INJECT_NAMES: [&str; NUM_INJECT] = [{inject_names}];\n\n\
         /// Injection source impedances in ohms (series R for Thevenin, shunt R for\n\
         /// Norton). The conductance `1/INJECT_RESISTANCES[k]` is already baked into\n\
         /// the G matrix (stamped before the kernel), so it never enters the NR loop.\n\
         pub const INJECT_RESISTANCES: [f64; NUM_INJECT] = [{inject_res}];\n\n\
         /// Per-injection Norton flag: `true` = the runtime value is a CURRENT\n\
         /// (`rhs[node] += val`); `false` = a VOLTAGE behind R\n\
         /// (`rhs[node] += (val + val_prev) / R` for trap, `val / R` for BE).\n\
         pub const INJECT_IS_NORTON: [bool; NUM_INJECT] = [{inject_norton}];\n\n\
         /// Number of raw inner-rate taps (`.tap`). May be 0.\n\
         pub const NUM_TAP: usize = {num_tap};\n\n\
         /// Tap node indices (0-indexed), read RAW (pre-decimation, pre-DC-block,\n\
         /// pre-scale) each inner sample — see the `taps_inner` return of\n\
         /// `process_sample`. Emitted separately from OUTPUT_NODES even when a node\n\
         /// coincides (different semantics).\n\
         pub const TAP_NODES: [usize; NUM_TAP] = [{tap_nodes}];\n\n\
         /// Tap names (TAP order), for the caller's index mapping.\n\
         pub const TAP_NAMES: [&str; NUM_TAP] = [{tap_names}];\n",
        num_inject = inj.len(),
        num_tap = taps.len(),
    )
}

/// Emit the `.inject` RHS stamp loop for the NODAL path (inline RHS builders).
///
/// `rhs_var` is the target RHS array (`rhs`, `rhs_s`, …); `indent` is the
/// leading whitespace. Empty when there is no `.inject`. Mirrors the
/// audio-input stamp: the source value is known at sample start, so it enters
/// the RHS as a constant and the NR loop never sees it. Under the charge form
/// both integrators stamp the value at `n+1` only: a Norton source as the
/// current, a Thevenin source as `V/R` (a Norton current `I` is the exact
/// equivalent of `V = I/G_sh` behind `R = 1/G_sh`).
pub(super) fn emit_inject_rhs_stamp(ir: &CircuitIR, rhs_var: &str, indent: &str) -> String {
    if ir.solver_config.injections.is_empty() {
        return String::new();
    }
    format!(
        "{indent}// Runtime feedback injections (.inject): source value known at sample\n\
         {indent}// start enters the RHS as a constant at n+1 (NR never sees it).\n\
         {indent}for k in 0..NUM_INJECT {{\n\
         {indent}    if INJECT_IS_NORTON[k] {{\n\
         {indent}        {rhs_var}[INJECT_NODES[k]] += injections[k];\n\
         {indent}    }} else {{\n\
         {indent}        {rhs_var}[INJECT_NODES[k]] += injections[k] / INJECT_RESISTANCES[k];\n\
         {indent}    }}\n\
         {indent}}}\n"
    )
}

/// Emit an INTERNAL zero-input `process_sample(...)` call line (warmup /
/// DC-OP fast-forward loops), matching the public signature: multi-input
/// takes `[0.0; NUM_INPUTS]`; `.inject`/`.tap` decks take the extra
/// all-zero per-inner-sample injection array and return a tuple (discarded in
/// statement position). `indent` is the emitted-code leading whitespace.
pub(super) fn emit_warmup_call(ir: &CircuitIR, indent: &str, let_bind: bool) -> String {
    // `let_bind` reproduces the exact pre-inject statement form at each call
    // site (the DC-OP settle loop used `let _ = …`; the nodal warmup loops a
    // bare call) so no-inject decks stay byte-identical.
    let lhs = if let_bind { "let _ = " } else { "" };
    if ir.solver_config.has_inject_or_tap() {
        format!(
            "{indent}{lhs}process_sample(0.0, &[[0.0; NUM_INJECT]; OVERSAMPLING_FACTOR], self);\n"
        )
    } else if ir.solver_config.num_inputs() > 1 {
        format!("{indent}{lhs}process_sample([0.0; NUM_INPUTS], self);\n")
    } else {
        format!("{indent}{lhs}process_sample(0.0, self);\n")
    }
}

/// Emit the `.inject` RHS stamp for a nodal micro-SUB-STEP (stiff-sample
/// recovery). Both source kinds interpolate their injection across the
/// sub-step ramp exactly like the audio input and stamp the value at the
/// sub-step's end (the charge form's `n+1`): Norton as the current, Thevenin
/// as `V/R`. `ndiv` is the sub-division count variable in scope; `step` is the
/// loop index in scope. Empty when there is no `.inject`.
pub(super) fn emit_inject_substep_stamp(
    ir: &CircuitIR,
    rhs_var: &str,
    indent: &str,
    ndiv: &str,
) -> String {
    if ir.solver_config.injections.is_empty() {
        return String::new();
    }
    format!(
        "{indent}for k in 0..NUM_INJECT {{\n\
         {indent}    let inj_step_k = (injections[k] - state.injections_prev[k]) / {ndiv} as f64;\n\
         {indent}    let inj_s = state.injections_prev[k] + inj_step_k * (step + 1) as f64;\n\
         {indent}    if INJECT_IS_NORTON[k] {{\n\
         {indent}        {rhs_var}[INJECT_NODES[k]] += inj_s;\n\
         {indent}    }} else {{\n\
         {indent}        {rhs_var}[INJECT_NODES[k]] += inj_s / INJECT_RESISTANCES[k];\n\
         {indent}    }}\n\
         {indent}}}\n"
    )
}
