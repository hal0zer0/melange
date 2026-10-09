//! Guards on the values a plugin writes into generated state.
//!
//! A non-finite runtime input never reaches the solver and is always counted
//! in `diag_runtime_nan_count`: a per-sample source (`.runtime V`, `.inject`)
//! reads as 0 for that sample; a ranged setter clamps ±inf to its range end;
//! any other non-finite setter argument leaves the value unchanged. Without
//! this, one non-finite control voltage made every later sample exhaust the
//! Newton and sub-step budgets and reset on NaN (measured: 100× slower per
//! sample on DK, 17,000× on nodal), with no way back until the plugin wrote a
//! finite value.
//!
//! `.runtime V` fields are public and read by every RHS stamp, so they are
//! sanitised once per HOST sample into a private copy that the stamps use:
//! at the top of the host-level `process_sample` (the fixed 1× entry, the
//! `.inject` wrapper, the oversampling wrappers and the runtime-selectable
//! dispatcher), never inside the inner-rate function. The public field is
//! left as the caller wrote it, so a stuck value keeps counting on every host
//! sample until it is replaced, and the count does not scale with the
//! oversampling factor.

use crate::codegen::ir::CircuitIR;

/// The suffix of the private per-source copy every RHS stamp reads.
pub(super) const SANITIZED_SUFFIX: &str = "_sanitized";

/// The statements that sanitise every `.runtime V` field into its private
/// copy, counting a non-finite value once; empty when the deck has none.
pub(super) fn sanitize_block(ir: &CircuitIR, indent: &str) -> String {
    if ir.runtime_sources.is_empty() {
        return String::new();
    }
    let mut code = format!(
        "{indent}// `.runtime` sources: a non-finite value never reaches the solver. Each\n\
         {indent}// field is read once per host sample into the private copy the RHS stamps\n\
         {indent}// use; 0 (the deck's own DC value) replaces it and the write is counted. The\n\
         {indent}// public field is left as written, so a stuck value keeps counting.\n"
    );
    for rt in &ir.runtime_sources {
        let f = &rt.field_name;
        code.push_str(&format!(
            "{indent}state.{f}{SANITIZED_SUFFIX} = if state.{f}.is_finite() {{ state.{f} }} else {{ state.diag_runtime_nan_count += 1; 0.0 }};\n"
        ));
    }
    code.push('\n');
    code
}

/// The doc comment and declaration of `diag_runtime_nan_count`, at `indent`.
pub(super) fn counter_field(indent: &str) -> String {
    format!(
        "{indent}/// Diagnostic: non-finite (NaN or ±inf) values the plugin wrote: a\n\
         {indent}/// `.runtime` voltage-source field (counted once per host sample while it\n\
         {indent}/// stays non-finite; the source reads as 0 for that sample), a `.inject`\n\
         {indent}/// value (replaced by 0), or a setter argument (`set_pot_*`,\n\
         {indent}/// `set_runtime_*`, the noise setters, `set_temperature_k`,\n\
         {indent}/// `set_sample_rate`: ±inf clamps to a declared range, anything else\n\
         {indent}/// leaves the value unchanged). Cleared by `reset()`.\n\
         {indent}pub diag_runtime_nan_count: u64,\n"
    )
}
