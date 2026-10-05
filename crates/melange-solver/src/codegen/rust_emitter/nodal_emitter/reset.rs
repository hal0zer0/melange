//! State-reset emitters: the DC-blocker history reseed and the NaN/Inf reset.

use super::sites::has_be_instance;
use crate::codegen::ir::{CircuitIR, DeviceParams};
use crate::codegen::rust_emitter::dk_emitter::NoiseEmission;
use crate::codegen::rust_emitter::helpers::{
    carries_q_dot, emit_stateful_state_restore, oversampling_info, stateful_device_data,
};
use crate::codegen::rust_emitter::inject_tap::emit_inject_os_state_reset;

/// Emit the DC-blocker history reseed shared by `reset()`, `set_sample_rate`
/// (both the same-rate fast path and the full-rebuild path), and the NaN
/// recovery block:
///
/// - `dc_block_x_prev[k]` ← `dc_operating_point[OUTPUT_NODES[k]]` (baked
///   index; `0.0` for outputs without a DC OP entry)
/// - `dc_block_y_prev` ← zeros
///
/// Zeroing `x_prev` instead would make the first sample after the reseed see
/// the output's full DC bias as a step through the differentiator — an
/// audible click. Matches the DK template's reseed.
///
/// `recv` is the receiver expression (`"self"` or `"state"`).
///
/// `use_ic_seed`: when true and the circuit has `IC=`-bearing capacitors,
/// seeds from the compile-time `V_PREV_IC_SEED` constant instead of the
/// live `dc_operating_point` field. Only `reset()` passes `true` — "factory
/// state" restores the same t=0 IC-seeded state as `Default`. The
/// mid-session NaN-recovery and `set_sample_rate` call sites pass `false`
/// deliberately: those are "return to the designed nominal bias" recoveries,
/// not "replay the historical startup charge."
pub(super) fn emit_dc_block_history_reseed(
    code: &mut String,
    ir: &CircuitIR,
    indent: &str,
    recv: &str,
    use_ic_seed: bool,
) {
    let has_dc_op = ir.has_dc_op;
    let ic_seed = if use_ic_seed {
        &ir.v_prev_ic_seed
    } else {
        &None
    };
    for (oi, &node) in ir.solver_config.output_nodes.iter().enumerate() {
        if let Some(v_prev_ic) = ic_seed {
            if node < v_prev_ic.len() {
                code.push_str(&format!(
                    "{indent}{recv}.dc_block_x_prev[{oi}] = V_PREV_IC_SEED[{node}];\n"
                ));
                continue;
            }
        }
        if has_dc_op && node < ir.dc_operating_point.len() {
            code.push_str(&format!(
                "{indent}{recv}.dc_block_x_prev[{oi}] = {recv}.dc_operating_point[{node}];\n"
            ));
        } else {
            code.push_str(&format!("{indent}{recv}.dc_block_x_prev[{oi}] = 0.0;\n"));
        }
    }
    code.push_str(&format!(
        "{indent}{recv}.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n"
    ));
}

// ============================================================================
// Shared NaN/Inf state-reset emitter (nodal Schur + full-LU)
// ============================================================================

/// Emit the `if !v.iter().all(|x| x.is_finite()) { ... }` recovery block that
/// clears persistent solver state after a numerical blow-up, then returns the
/// DC-operating-point output clamped to ±10 V.
///
/// Must stay in sync with the DK-path equivalent in
/// `templates/rust/process_sample.rs.tera` (Step 7). Reset list:
///
/// - `v_prev`  ← `dc_operating_point`
/// - `i_nl_prev` / `i_nl_prev_prev` ← `DC_NL_I` or zeros
/// - `input_prev` ← 0
/// - `dc_block_x_prev` ← DC OP at the output nodes, `dc_block_y_prev` ← 0
///   (when `ir.dc_block`)
/// - BJT self-heating thermal state (per device with `RTH < ∞`)
/// - Pot fields are NOT touched (matrices already match them; a nominal
///   snap without a rebuild would desync fields from matrices)
/// - Oversampler filter state (inner + outer 4× when applicable)
/// - `chord_valid` ← false (full-LU path only)
/// - `diag_nan_reset_count` += 1
///
/// `indent` is the indent prefix of the outer `if` line (usually `"    "`).
/// `is_full_lu` selects the full-LU-only chord-LU invalidation.
///
/// Augmented MNA stores inductor branch currents in `v_prev` (their history
/// is in `q_dot`), so there is no separate inductor state to reset.
pub(super) fn emit_nodal_nan_reset(
    code: &mut String,
    ir: &CircuitIR,
    indent: &str,
    is_full_lu: bool,
    noise: &NoiseEmission,
) {
    let body = format!("{indent}    ");
    let has_dc_nl = ir.dc_nl_currents.iter().any(|&v| v != 0.0);
    let m = ir.topology.m;

    code.push_str(&format!(
        "{indent}// NaN/Inf AND finite-but-implausible-magnitude check BEFORE state\n\
         {indent}// update — prevents corruption of v_prev/i_nl_prev. A saturated solve\n\
         {indent}// (exp() clamp, near-singular LU) can produce a huge-but-finite value\n\
         {indent}// that bypasses is_finite() and would otherwise propagate forever.\n"
    ));
    code.push_str(&format!(
        "{indent}// Mirrors the DK-path reset in templates/rust/process_sample.rs.tera (Step 7).\n"
    ));
    code.push_str(&format!(
        "{indent}let v_is_finite = v.iter().all(|x| x.is_finite());\n\
         {indent}if !v_is_finite || v.iter().any(|x| x.abs() > STATE_MAX_PLAUSIBLE_MAGNITUDE) {{\n"
    ));

    // Core NR state
    code.push_str(&format!("{body}state.v_prev = state.dc_operating_point;\n"));
    if carries_q_dot(ir) {
        code.push_str(&format!("{body}state.q_dot = [0.0; N];\n"));
    }
    if has_dc_nl {
        code.push_str(&format!("{body}state.i_nl_prev = DC_NL_I;\n"));
        code.push_str(&format!("{body}state.i_nl_prev_prev = DC_NL_I;\n"));
    } else {
        code.push_str(&format!("{body}state.i_nl_prev = [0.0; M];\n"));
        code.push_str(&format!("{body}state.i_nl_prev_prev = [0.0; M];\n"));
    }
    if ir.solver_config.num_inputs() > 1 {
        code.push_str(&format!("{body}state.inputs_prev = [0.0; NUM_INPUTS];\n"));
    } else {
        code.push_str(&format!("{body}state.input_prev = 0.0;\n"));
    }
    if ir.solver_config.has_inject_or_tap() {
        code.push_str(&format!(
            "{body}state.injections_prev = [0.0; NUM_INJECT];\n"
        ));
    }

    // DC blocker history: reseed x_prev from the DC operating point (matches
    // reset() and the DK template). Zeroing x_prev would make the first
    // post-recovery sample see the full output DC bias as a step through the
    // differentiator — a second click right after the NaN event.
    if ir.dc_block {
        emit_dc_block_history_reseed(code, ir, &body, "state", false);
    }

    // Device self-heating thermal state (BJT, diode, and triode)
    for (dev_num, slot) in ir.device_slots.iter().enumerate() {
        match &slot.params {
            DeviceParams::Bjt(bp) if bp.has_self_heating() => {
                code.push_str(&format!(
                    "{body}state.device_{dev_num}_tj = DEVICE_{dev_num}_TAMB;\n\
                     {body}state.device_{dev_num}_is = DEVICE_{dev_num}_IS;\n\
                     {body}state.device_{dev_num}_vt = DEVICE_{dev_num}_VT;\n"
                ));
            }
            DeviceParams::Diode(dp) if dp.has_self_heating() => {
                code.push_str(&format!(
                    "{body}state.device_{dev_num}_tj = DEVICE_{dev_num}_TAMB;\n\
                     {body}state.device_{dev_num}_is = DEVICE_{dev_num}_IS;\n\
                     {body}state.device_{dev_num}_n_vt = DEVICE_{dev_num}_N_VT;\n"
                ));
            }
            DeviceParams::Tube(tp) if tp.has_self_heating() => {
                // Triode: only Tj is carried as runtime state. The Vgk
                // bias shift is computed on the fly at the NR call site.
                code.push_str(&format!(
                    "{body}state.device_{dev_num}_tj = DEVICE_{dev_num}_TAMB;\n"
                ));
            }
            _ => {}
        }
    }

    // Stateful-device opaque state blocks: restore to seed. `body` here is
    // 8-space (indent="    " + 4), matching the shared helper's 8-space `state.`
    // lines, so this block is byte-identical to the DK NaN-recovery path.
    code.push_str(&emit_stateful_state_restore(
        &stateful_device_data(ir),
        "state.",
    ));

    // Pots are deliberately NOT reset here. The matrices already reflect the
    // current pot fields; snapping the fields to nominal without setting
    // matrices_dirty would leave fields and matrices disagreeing until the
    // host next moves a knob. (Pot rebuilds are absolute-from-nominal, so
    // leaving both alone is coherent.)

    // Oversampler filter state (inner polyphase + outer 4× halfband)
    let os_factor = ir.solver_config.oversampling_factor;
    if super::super::runtime_os::runtime(ir).is_some() {
        code.push_str(&format!("{body}state.reset_oversampler();\n"));
    } else if os_factor > 1 {
        let os_info = oversampling_info(os_factor);
        code.push_str(&format!(
            "{body}state.os_up_state = [0.0; {}];\n\
             {body}state.os_dn_state = [[0.0; {}]; NUM_OUTPUTS];\n",
            os_info.state_size, os_info.state_size
        ));
        if os_factor == 4 {
            code.push_str(&format!(
                "{body}state.os_up_state_outer = [0.0; {}];\n\
                 {body}state.os_dn_state_outer = [[0.0; {}]; NUM_OUTPUTS];\n",
                os_info.state_size_outer, os_info.state_size_outer
            ));
        }
        code.push_str(&emit_inject_os_state_reset(ir, "state", &body));
    }

    // Full-LU-only: invalidate the cross-timestep chord LU factorization.
    if is_full_lu && m > 0 {
        code.push_str(&format!("{body}state.chord_valid = false;\n"));
        if has_be_instance(ir) {
            code.push_str(&format!("{body}state.chord_be_valid = false;\n"));
        }
    }

    // Noise NaN-recovery: clear two-draw lag + BE-replay caches if noise
    // codegen is enabled. Empty string when noise mode is Off — the
    // template-style `{% if %}` gating in dk_emitter.rs.tera is handled
    // here by the empty-fragment guarantee on `NoiseEmission`. Indent is
    // already baked into the body (8-space DK convention); re-indent to
    // match the local `body` prefix.
    if noise.enabled && !noise.nan_recovery_body.is_empty() {
        for line in noise.nan_recovery_body.lines() {
            let trimmed = line.trim_start();
            if trimmed.is_empty() {
                code.push('\n');
            } else {
                code.push_str(&format!("{body}{trimmed}\n"));
            }
        }
    }

    code.push_str(&format!(
        "{body}if v_is_finite {{ state.diag_magnitude_reset_count += 1; }} else {{ state.diag_nan_reset_count += 1; }}\n"
    ));

    // Return DC-OP output (clamped to ±10 V) instead of zero, to minimize the
    // discontinuity at the recovery edge. Values baked at codegen time.
    code.push_str(&format!("{body}let mut nan_out = [0.0f64; NUM_OUTPUTS];\n"));
    for (oi, &node) in ir.solver_config.output_nodes.iter().enumerate() {
        if node < ir.dc_operating_point.len() {
            let dc_val = ir.dc_operating_point[node];
            let scale = ir
                .solver_config
                .output_scales
                .get(oi)
                .copied()
                .unwrap_or(1.0);
            let clamp_v = ir.solver_config.output_clamp_v;
            let out_val = (dc_val * scale).clamp(-clamp_v, clamp_v);
            code.push_str(&format!("{body}nan_out[{oi}] = {out_val:.17e};\n"));
        }
    }
    if ir.solver_config.has_inject_or_tap() {
        // Raw taps fall back to the DC operating point (v is invalid here).
        code.push_str(&format!("{body}let mut nan_taps = [0.0f64; NUM_TAP];\n"));
        for (ti, tap) in ir.solver_config.taps.iter().enumerate() {
            let dc_val = ir.dc_operating_point.get(tap.node).copied().unwrap_or(0.0);
            code.push_str(&format!("{body}nan_taps[{ti}] = {dc_val:.17e};\n"));
        }
        code.push_str(&format!("{body}return (nan_out, nan_taps);\n"));
    } else {
        code.push_str(&format!("{body}return nan_out;\n"));
    }
    code.push_str(&format!("{indent}}}\n\n"));
}
