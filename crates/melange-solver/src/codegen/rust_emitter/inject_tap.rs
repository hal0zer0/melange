//! `.inject` / `.tap` emission plumbing shared by the DK and nodal paths.

use tera::Context;

use super::helpers::{fmt_f64, oversampling_info};
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
    // The whole constant block, single-sourced with the nodal path.
    ctx.insert("inject_tap_constants", &emit_inject_tap_constants(ir));
    // Host-rate injection up-filter state: field declarations, Default
    // initializers, and the resets that mirror every `os_up_state` reset.
    ctx.insert("inject_os_fields", &emit_inject_os_state_fields(ir));
    ctx.insert("inject_os_init", &emit_inject_os_state_init(ir));
    ctx.insert(
        "inject_os_reset_self",
        &emit_inject_os_state_reset(ir, "self", "        "),
    );
    ctx.insert(
        "inject_os_reset_state",
        &emit_inject_os_state_reset(ir, "state", "        "),
    );
}

/// Comma-joined emission helper for one per-injection column.
fn join_col<T>(items: &[T], f: impl Fn(&T) -> String) -> String {
    items.iter().map(f).collect::<Vec<_>>().join(", ")
}

/// Emit the `.inject` / `.tap` constant block (both the DK template, via the
/// `inject_tap_constants` context variable, and the nodal path, which builds
/// constants inline). Empty when there is no `.inject`/`.tap`.
///
/// `NUM_INJECT` / `INJECT_*` describe EVERY injection in directive order (the
/// inner solve's view); the `_HOST` / `_INNER` families describe each kind in
/// the order `process_sample` takes it.
pub(super) fn emit_inject_tap_constants(ir: &CircuitIR) -> String {
    let inj = &ir.solver_config.injections;
    let taps = &ir.solver_config.taps;
    if inj.is_empty() && taps.is_empty() {
        return String::new();
    }
    let host_idx = ir.solver_config.inject_host_indices();
    let inner_idx = ir.solver_config.inject_inner_indices();
    let inject_nodes = join_col(inj, |i| i.node.to_string());
    let inject_names = join_col(inj, |i| format!("{:?}", i.name));
    let inject_res = join_col(inj, |i| fmt_f64(i.resistance));
    let inject_norton = join_col(inj, |i| i.norton.to_string());
    let inject_is_host = join_col(inj, |i| i.host_rate.to_string());
    let kind_block = |kind: &str, count_doc: &str, idx: &[usize]| -> String {
        let index = join_col(idx, |k| k.to_string());
        let names = join_col(idx, |&k| format!("{:?}", inj[k].name));
        let nodes = join_col(idx, |&k| inj[k].node.to_string());
        let res = join_col(idx, |&k| fmt_f64(inj[k].resistance));
        let norton = join_col(idx, |&k| inj[k].norton.to_string());
        let lower = kind.to_ascii_lowercase();
        format!(
            "{count_doc}\
             pub const NUM_INJECT_{kind}: usize = {n};\n\n\
             /// Position of each `rate={lower}` injection in the `INJECT_*` arrays, in the\n\
             /// order of `process_sample`'s `injections_{lower}` argument.\n\
             pub const INJECT_{kind}_INDEX: [usize; NUM_INJECT_{kind}] = [{index}];\n\n\
             /// `rate={lower}` injection names (`injections_{lower}` order).\n\
             pub const INJECT_{kind}_NAMES: [&str; NUM_INJECT_{kind}] = [{names}];\n\n\
             /// `rate={lower}` injection node indices (0-indexed, `injections_{lower}` order).\n\
             pub const INJECT_{kind}_NODES: [usize; NUM_INJECT_{kind}] = [{nodes}];\n\n\
             /// `rate={lower}` injection impedances in ohms (`injections_{lower}` order).\n\
             pub const INJECT_{kind}_RESISTANCES: [f64; NUM_INJECT_{kind}] = [{res}];\n\n\
             /// `rate={lower}` Norton flags (`injections_{lower}` order).\n\
             pub const INJECT_{kind}_IS_NORTON: [bool; NUM_INJECT_{kind}] = [{norton}];\n\n",
            n = idx.len(),
        )
    };
    let host_block = kind_block(
        "HOST",
        "/// Number of `rate=host` injections (may be 0): audio-rate inputs, one value\n\
         /// per host sample, upsampled through the audio input's half-band up-filter.\n",
        &host_idx,
    );
    let inner_block = kind_block(
        "INNER",
        "/// Number of `rate=inner` injections (may be 0): one value per inner\n\
         /// (oversampled) sub-step, straight to the inner solve.\n",
        &inner_idx,
    );
    let tap_nodes = join_col(taps, |t| t.node.to_string());
    let tap_names = join_col(taps, |t| format!("{:?}", t.name));
    format!(
        "\n// -----------------------------------------------------------------------------\n\
         // Runtime injection (`.inject`) + raw inner-rate taps (`.tap`).\n\
         //\n\
         // When either directive is present `process_sample` takes a host-rate and an\n\
         // inner-rate injection array and returns per-inner-sample raw taps (see its\n\
         // doc-comment). Every count is always emitted so the API shape is uniform; any\n\
         // of them may be 0.\n\
         // -----------------------------------------------------------------------------\n\n\
         /// Number of runtime injection sources (`.inject`), both rates. May be 0.\n\
         pub const NUM_INJECT: usize = {num_inject};\n\n\
         /// Injection node indices (0-indexed), one per `.inject` source, in directive\n\
         /// order (both rates). `process_sample` takes the two rates as separate\n\
         /// arguments; `INJECT_HOST_INDEX` / `INJECT_INNER_INDEX` map them here.\n\
         pub const INJECT_NODES: [usize; NUM_INJECT] = [{inject_nodes}];\n\n\
         /// Injection source names (INJECT order), for the caller's index mapping.\n\
         pub const INJECT_NAMES: [&str; NUM_INJECT] = [{inject_names}];\n\n\
         /// Injection source impedances in ohms (series R for Thevenin, shunt R for\n\
         /// Norton). The conductance `1/INJECT_RESISTANCES[k]` is already baked into\n\
         /// the G matrix (stamped before the kernel), so it never enters the NR loop.\n\
         pub const INJECT_RESISTANCES: [f64; NUM_INJECT] = [{inject_res}];\n\n\
         /// Per-injection Norton flag: `true` = the runtime value is a CURRENT\n\
         /// (`rhs[node] += val`); `false` = a VOLTAGE behind R (`rhs[node] += val / R`).\n\
         /// Both enter at n+1 under either integrator.\n\
         pub const INJECT_IS_NORTON: [bool; NUM_INJECT] = [{inject_norton}];\n\n\
         /// Per-injection rate: `true` = `rate=host`, `false` = `rate=inner`.\n\
         pub const INJECT_IS_HOST: [bool; NUM_INJECT] = [{inject_is_host}];\n\n\
         {host_block}\
         {inner_block}\
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

/// Whether this build carries per-injection up-filter state: an
/// `.inject`/`.tap` API at `OVERSAMPLING_FACTOR > 1`. The arrays are sized
/// `NUM_INJECT_HOST` (possibly 0) so the state layout does not depend on the
/// rate mix.
fn has_inject_os_state(ir: &CircuitIR) -> bool {
    ir.solver_config.has_inject_or_tap() && ir.solver_config.oversampling_factor > 1
}

/// `CircuitState` fields for the host-rate injection up-filters (4-space
/// indent, leading-newline-free). Empty when [`has_inject_os_state`] is false.
pub(super) fn emit_inject_os_state_fields(ir: &CircuitIR) -> String {
    if !has_inject_os_state(ir) {
        return String::new();
    }
    let info = oversampling_info(ir.solver_config.oversampling_factor);
    let mut s = format!(
        "    /// Half-band up-filter state per `rate=host` injection (the same filter as\n\
         \x20   /// `os_up_state`, one copy per injection)\n\
         \x20   pub os_inj_up_state: [[f64; {}]; NUM_INJECT_HOST],\n",
        info.state_size
    );
    if ir.solver_config.oversampling_factor == 4 {
        s.push_str(&format!(
            "    /// 4x outer up-filter state per `rate=host` injection (as `os_up_state_outer`)\n\
             \x20   pub os_inj_up_state_outer: [[f64; {}]; NUM_INJECT_HOST],\n",
            info.state_size_outer
        ));
    }
    s
}

/// `Default` initializers for [`emit_inject_os_state_fields`] (12-space indent).
pub(super) fn emit_inject_os_state_init(ir: &CircuitIR) -> String {
    if !has_inject_os_state(ir) {
        return String::new();
    }
    let info = oversampling_info(ir.solver_config.oversampling_factor);
    let mut s = format!(
        "            os_inj_up_state: [[0.0; {}]; NUM_INJECT_HOST],\n",
        info.state_size
    );
    if ir.solver_config.oversampling_factor == 4 {
        s.push_str(&format!(
            "            os_inj_up_state_outer: [[0.0; {}]; NUM_INJECT_HOST],\n",
            info.state_size_outer
        ));
    }
    s
}

/// Zero the host-rate injection up-filter state. Emitted beside every reset
/// of `os_up_state` (reset, set_sample_rate, NaN recovery, DC-OP recompute)
/// so an injection and the audio input always restart from the same filter
/// state. `owner` is `self` or `state`. Empty when [`has_inject_os_state`] is
/// false.
pub(super) fn emit_inject_os_state_reset(ir: &CircuitIR, owner: &str, indent: &str) -> String {
    if !has_inject_os_state(ir) {
        return String::new();
    }
    let info = oversampling_info(ir.solver_config.oversampling_factor);
    let mut s = format!(
        "{indent}{owner}.os_inj_up_state = [[0.0; {}]; NUM_INJECT_HOST];\n",
        info.state_size
    );
    if ir.solver_config.oversampling_factor == 4 {
        s.push_str(&format!(
            "{indent}{owner}.os_inj_up_state_outer = [[0.0; {}]; NUM_INJECT_HOST];\n",
            info.state_size_outer
        ));
    }
    s
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
/// takes `[0.0; NUM_INPUTS]`; `.inject`/`.tap` decks take the extra all-zero
/// host-rate and per-inner-sample injection arrays and return a tuple
/// (discarded in statement position). `indent` is the emitted-code leading whitespace.
pub(super) fn emit_warmup_call(ir: &CircuitIR, indent: &str, let_bind: bool) -> String {
    // `let_bind` reproduces the exact pre-inject statement form at each call
    // site (the DC-OP settle loop used `let _ = …`; the nodal warmup loops a
    // bare call) so no-inject decks stay byte-identical.
    let lhs = if let_bind { "let _ = " } else { "" };
    if ir.solver_config.has_inject_or_tap() {
        format!(
            "{indent}{lhs}process_sample(0.0, &[0.0; NUM_INJECT_HOST], &[[0.0; NUM_INJECT_INNER]; OVERSAMPLING_FACTOR], self);\n"
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
