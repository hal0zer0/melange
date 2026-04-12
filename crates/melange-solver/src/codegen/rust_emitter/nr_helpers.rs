//! Newton-Raphson helper functions for code generation.
//!
//! Free functions that emit NR singular fallback, voltage limiting, and
//! convergence checking code. Used by both DK and nodal solver paths.

use crate::codegen::ir::{CircuitIR, DeviceType};

// ============================================================================
// Procedural NR solver generation (too complex for templates)
// ============================================================================

/// Emit damped fallback for singular Jacobian: half-step on residual, clamped.
///
/// Clamp scales with current magnitude to handle both small-signal (diodes at µA)
/// and large-signal (BJTs at mA) operating points.
pub(super) fn emit_nr_singular_fallback(code: &mut String, dim: usize, indent: &str) {
    code.push_str(&format!(
        "{indent}// Singular Jacobian — damped fallback (0.5 * residual, adaptive clamp)\n"
    ));
    for i in 0..dim {
        code.push_str(&format!(
            "{indent}{{ let clamp = (i_nl[{i}].abs() * 0.1).max(0.01); i_nl[{i}] -= (f{i} * 0.5).clamp(-clamp, clamp); }}\n"
        ));
    }
}

/// Emit SPICE-style voltage-space limiting and convergence check for NR.
///
/// After computing Newton deltas (delta0..delta{dim-1}), converts to voltage
/// space via K matrix, applies per-device pnjlim/fetlim, and uses scalar
/// damping to maintain current-space NR consistency.
///
/// Assumes `delta0..delta{dim-1}` and `v_d0..v_d{dim-1}` are in scope.
pub(super) fn emit_nr_limit_and_converge(code: &mut String, ir: &CircuitIR, dim: usize, indent: &str) {
    // Compute implied voltage changes: dv[i] = -sum_j K[i][j] * delta[j]
    code.push_str(&format!(
        "{indent}// Voltage-space limiting (SPICE pnjlim/fetlim through K matrix)\n"
    ));
    for i in 0..dim {
        code.push_str(&format!("{indent}let dv{i} = -("));
        let mut first = true;
        {
            // state.k is exact (per-block rebuild), no k_eff needed
            for j in 0..dim {
                if !first {
                    code.push_str(" + ");
                }
                code.push_str(&format!("state.k[{i}][{j}] * delta{j}"));
                first = false;
            }
        }
        code.push_str(");\n");
    }

    // Compute per-dimension damping factor from per-device limiting
    code.push_str(&format!("{indent}let mut alpha = [1.0_f64; {dim}];\n"));
    code.push_str(&format!("{indent}let mut any_limited = false;\n"));
    for (dev_num, slot) in ir.device_slots.iter().enumerate() {
        for d in 0..slot.dimension {
            let i = slot.start_idx + d;
            code.push_str(&format!("{indent}if dv{i}.abs() > 1e-15 {{\n"));
            // Emit per-device limiter call based on device type and dimension
            match (&slot.device_type, d) {
                (DeviceType::Diode, _) => {
                    code.push_str(&format!(
                        "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_VCRIT);\n"
                    ));
                }
                (DeviceType::Bjt, _) | (DeviceType::BjtForwardActive, _) => {
                    // PN junction limiting (Vbe for forward-active, both Vbe/Vbc for full)
                    code.push_str(&format!(
                        "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vt, DEVICE_{dev_num}_VCRIT);\n"
                    ));
                }
                (DeviceType::Jfet, 0) => {
                    // dim 0 = Vds — generous limiting
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, 0.0);\n"
                    ));
                }
                (DeviceType::Jfet, _) => {
                    // dim 1 = Vgs — limit around pinch-off voltage
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vp);\n"
                    ));
                }
                (DeviceType::Mosfet, 0) => {
                    // dim 0 = Vds — generous limiting
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, 0.0);\n"
                    ));
                }
                (DeviceType::Mosfet, _) => {
                    // dim 1 = Vgs — limit around threshold voltage
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vt);\n"
                    ));
                }
                (DeviceType::Tube, 0) => {
                    code.push_str(&format!(
                        "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vgk_onset / 3.0, DEVICE_{dev_num}_VCRIT);\n"
                    ));
                }
                (DeviceType::Tube, 2) => {
                    // Pentode dim 2 = Vg2k — same softplus knee behavior as Vgk;
                    // log-junction limiting prevents large NR steps from skipping
                    // the E1 knee. Triodes never reach dim==2 (they are 2D).
                    code.push_str(&format!(
                        "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vgk_onset / 3.0, DEVICE_{dev_num}_VCRIT);\n"
                    ));
                }
                (DeviceType::Tube, _) => {
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, 0.0);\n"
                    ));
                }
                (DeviceType::Vca, _) => {
                    // VCA: no junction limiting needed — fast_exp already clamps
                    code.push_str(&format!("{indent}    let v_lim = v_d{i} + dv{i};\n"));
                }
            }
            code.push_str(&format!(
                "{indent}    let ratio = ((v_lim - v_d{i}) / dv{i}).max(0.01);\n"
            ));
            code.push_str(&format!(
                "{indent}    if ratio < alpha[{i}] {{ alpha[{i}] = ratio; if ratio < 1.0 {{ any_limited = true; }} }}\n"
            ));
            code.push_str(&format!("{indent}}}\n"));
        }
    }

    // Scalar alpha: use the global minimum across ALL devices.
    // Per-device alpha breaks the coupled Newton direction for multi-device
    // systems (e.g., anti-parallel diodes), causing oscillation when limiting
    // is asymmetric. Scalar alpha preserves the descent direction.
    code.push_str(&format!("{indent}let alpha_scalar = alpha.iter().copied().fold(1.0_f64, f64::min);\n"));
    code.push_str(&format!("{indent}if alpha_scalar < 1.0 {{ any_limited = true; }}\n"));

    // Current-space backstop: limit maximum current step per iteration.
    // Unlike the old voltage-space backstop (3.5V), this is K-independent.
    // When SM corrections weaken K, voltage backstops become pathologically
    // restrictive (dv = K*delta shrinks, backstop ratio → 0.006). Current
    // backstop avoids this because it operates directly on delta (current).
    code.push_str(&format!("{indent}let max_di = "));
    for i in 0..dim {
        if i > 0 {
            code.push_str(&format!(".max(delta{i}.abs())"));
        } else {
            code.push_str(&format!("delta{i}.abs()"));
        }
    }
    code.push_str(";\n");
    code.push_str(&format!(
        "{indent}let alpha_scalar = if max_di * alpha_scalar > 0.1 {{ (0.1 / max_di).max(0.01).min(alpha_scalar) }} else {{ alpha_scalar }};\n"
    ));

    // Apply scalar-damped step
    for i in 0..dim {
        code.push_str(&format!("{indent}i_nl[{i}] -= alpha_scalar * delta{i};\n"));
    }

    // Dual convergence check:
    // 1. Voltage-step (RELTOL): only checked when no limiting occurred
    // 2. Current-residual (ABSTOL): ALWAYS checked, even after limiting
    //
    // The residual check is K-independent — if |f_i| is small, the solution
    // is correct regardless of what pnjlim did to the step. This prevents
    // the solver from burning iterations when limiting triggers frequently
    // (common with weak K from SM pot corrections).
    code.push_str(&format!(
        "\n{indent}// Convergence: voltage-step (RELTOL, ungated) + current-residual (always)\n"
    ));
    code.push_str(&format!("{indent}{{\n"));
    code.push_str(&format!("{indent}    let mut nr_converged = true;\n"));
    // Voltage-step check: only when not limited (step direction is reliable)
    code.push_str(&format!("{indent}    if !any_limited {{\n"));
    for i in 0..dim {
        code.push_str(&format!(
            "{indent}        {{ let step = dv{i} * alpha_scalar; let v_new = v_d{i} + step; let v_thr = 1e-3 * v_d{i}.abs().max(v_new.abs()) + 1e-6; if step.abs() > v_thr {{ nr_converged = false; }} }}\n"
        ));
    }
    code.push_str(&format!("{indent}    }}\n"));
    // Current residual check: always (K-independent, catches cancellation artifacts)
    for i in 0..dim {
        code.push_str(&format!(
            "{indent}    {{ let i_thr = 1e-3 * i_nl[{i}].abs().max(i_dev{i}.abs()).max(1e-9) + 1e-12; if f{i}.abs() > i_thr {{ nr_converged = false; }} }}\n"
        ));
    }
    code.push_str(&format!("{indent}    if nr_converged {{\n"));
    code.push_str(&format!(
        "{indent}        state.last_nr_iterations = iter as u32;\n"
    ));
    code.push_str(&format!("{indent}        return i_nl;\n"));
    code.push_str(&format!("{indent}    }}\n"));
    code.push_str(&format!("{indent}}}\n"));
}

/// Emit NR limit/converge for Schur complement process_sample (uses `break` not `return`).
///
/// Same as `emit_nr_limit_and_converge` but ends the NR loop with `break` instead of
/// `return i_nl`, since we're inside `process_sample` not a standalone `solve_nonlinear`.
pub(super) fn emit_schur_nr_limit_and_converge(
    code: &mut String,
    ir: &CircuitIR,
    dim: usize,
    indent: &str,
    k_matrix_expr: &str,
) {
    // Trial-voltage approach: compute new device voltages after full NR step,
    // then limit those directly. This avoids the K-amplification problem where
    // large K entries cause dv = -K*delta to be huge even for small delta,
    // leading to extreme damping ratios that prevent convergence.
    code.push_str(&format!(
        "{indent}// Trial voltages: apply full NR step, then limit\n"
    ));
    // i_nl_trial = i_nl - delta
    for i in 0..dim {
        code.push_str(&format!("{indent}let i_trial{i} = i_nl[{i}] - delta{i};\n"));
    }
    // v_d_trial = p + K * i_nl_trial (new device voltages after full step)
    for i in 0..dim {
        code.push_str(&format!("{indent}let v_trial{i} = p[{i}]"));
        for j in 0..dim {
            code.push_str(&format!(" + {k_matrix_expr}[{i}][{j}] * i_trial{j}"));
        }
        code.push_str(";\n");
    }

    // Per-device voltage limiting on trial voltages
    code.push_str(&format!("{indent}let mut any_limited = false;\n"));
    for (dev_num, slot) in ir.device_slots.iter().enumerate() {
        for d in 0..slot.dimension {
            let i = slot.start_idx + d;
            // Apply pnjlim/fetlim: limit v_trial relative to v_d (current)
            let lim_expr = match (&slot.device_type, d) {
                (DeviceType::Diode, _) => format!(
                    "pnjlim(v_trial{i}, v_d{i}, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_VCRIT)"
                ),
                (DeviceType::Bjt, _) | (DeviceType::BjtForwardActive, _) => format!(
                    "pnjlim(v_trial{i}, v_d{i}, state.device_{dev_num}_vt, DEVICE_{dev_num}_VCRIT)"
                ),
                (DeviceType::Jfet, 0) => format!("fetlim(v_trial{i}, v_d{i}, 0.0)"),
                (DeviceType::Jfet, _) => format!("fetlim(v_trial{i}, v_d{i}, state.device_{dev_num}_vp)"),
                (DeviceType::Mosfet, 0) => format!("fetlim(v_trial{i}, v_d{i}, 0.0)"),
                (DeviceType::Mosfet, _) => format!("fetlim(v_trial{i}, v_d{i}, state.device_{dev_num}_vt)"),
                (DeviceType::Tube, 0) => format!(
                    "pnjlim(v_trial{i}, v_d{i}, state.device_{dev_num}_vgk_onset / 3.0, DEVICE_{dev_num}_VCRIT)"
                ),
                (DeviceType::Tube, _) => format!("fetlim(v_trial{i}, v_d{i}, 0.0)"),
                (DeviceType::Vca, _) => format!("v_trial{i}"), // No limiting needed
            };
            code.push_str(&format!("{indent}let v_lim{i} = {lim_expr};\n"));
        }
    }

    // Compute global damping: ratio of limited voltage change to full voltage change
    // Use the minimum ratio across all dimensions (most conservative).
    // When dv_lim and dv_full have opposite signs (limiter reversed direction,
    // common with positive K diagonal), clamp to 0 — don't take the step.
    code.push_str(&format!("{indent}let mut global_alpha = 1.0_f64;\n"));
    for i in 0..dim {
        code.push_str(&format!(
            "{indent}{{ let dv_full = v_trial{i} - v_d{i}; let dv_lim = v_lim{i} - v_d{i}; \
             if dv_full.abs() > 1e-15 {{ let r = if dv_full * dv_lim < 0.0 {{ 0.0 }} \
             else {{ (dv_lim / dv_full).clamp(0.0, 1.0) }}; \
             if r < global_alpha {{ global_alpha = r; any_limited = true; }} }} }}\n"
        ));
    }

    // Global voltage backstop: adaptive limit based on DC operating point voltages
    let max_dc_v = ir
        .dc_operating_point
        .iter()
        .map(|v| v.abs())
        .fold(0.0_f64, f64::max);
    let dv_limit = if max_dc_v > 20.0 {
        (max_dc_v * 0.15).max(3.5)
    } else {
        3.5
    };
    code.push_str(&format!("{indent}{{ let max_dv = "));
    for i in 0..dim {
        if i > 0 {
            code.push_str(&format!(
                ".max(((v_trial{i} - v_d{i}) * global_alpha).abs())"
            ));
        } else {
            code.push_str(&format!("((v_trial{i} - v_d{i}) * global_alpha).abs()"));
        }
    }
    code.push_str(&format!(
        "; if max_dv > {dv_limit:.6} {{ global_alpha *= ({dv_limit:.6} / max_dv).max(0.1); any_limited = true; }} }}\n"
    ));

    // Apply globally damped step
    for i in 0..dim {
        code.push_str(&format!("{indent}i_nl[{i}] -= global_alpha * delta{i};\n"));
    }

    // RELTOL convergence check — break from loop (not return)
    code.push_str(&format!(
        "\n{indent}// Convergence check (SPICE RELTOL=0.001, VNTOL=1e-6)\n"
    ));
    code.push_str(&format!("{indent}if !any_limited {{\n"));
    code.push_str(&format!("{indent}    let mut nr_converged = true;\n"));
    for i in 0..dim {
        code.push_str(&format!(
            "{indent}    {{ let dv = (v_trial{i} - v_d{i}) * global_alpha; let threshold = 1e-3 * v_d{i}.abs().max((v_d{i} + dv).abs()) + 1e-6; if dv.abs() > threshold {{ nr_converged = false; }} }}\n"
        ));
    }
    code.push_str(&format!("{indent}    if nr_converged {{\n"));
    code.push_str(&format!(
        "{indent}        state.last_nr_iterations = iter as u32;\n"
    ));
    code.push_str(&format!("{indent}        converged = true;\n"));
    code.push_str(&format!("{indent}        break;\n"));
    code.push_str(&format!("{indent}    }}\n"));
    code.push_str(&format!("{indent}}}\n"));
}
