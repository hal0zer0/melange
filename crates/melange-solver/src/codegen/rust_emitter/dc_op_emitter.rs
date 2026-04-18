//! Runtime DC operating-point recompute emission (Oomox P6, Phase E).
//!
//! Emits `CircuitState::recompute_dc_op()` into generated plugin code so that
//! plugins with per-instance pot/switch jitter can re-solve the DC bias at
//! construction time, skipping the `WARMUP_SAMPLES_RECOMMENDED` silence loop.
//!
//! ## Phase E scope
//!
//! **MVP — Direct Newton-Raphson only**, starting from `self.v_prev` (which is
//! a physically valid state from the prior solve). No source/Gmin stepping,
//! no basin-trap handling. Circuits that only converge via stepping in the
//! codegen-time solver remain reliant on the warmup loop.
//!
//! Device coverage: diode, BJT (EM + GP), JFET, MOSFET, triode, pentode,
//! linear op-amp, VCA — i.e. whatever the compile-time DC OP handles today,
//! minus multi-equilibrium refinements.
//!
//! ## Layering (implemented across subsequent commits)
//!
//! 1. `emit_recompute_dc_op_body_dk` — top-level no-op stub (E.2).
//! 2. `emit_dc_op_build_g_aug_dk` — G_aug construction from live pot/switch
//!    state (E.3, this file).
//! 3. `emit_dc_op_evaluate_devices_dk` — per-device i_nl + Jacobian (E.4).
//! 4. `emit_dc_op_solve` — NR loop, limiting, dense LU (E.5).
//! 5. Compose + state resets (E.6).
//! 6. Nodal mirror (E.8).
//!
//! The skeleton body below builds `g_aug` from the current pot/switch fields
//! and emits the per-device current + Jacobian evaluator, but does not yet
//! wrap them in an NR loop. Subsequent phases close that loop. See the
//! Phase E handoff memory file for the full definition of done.

use crate::codegen::ir::CircuitIR;
use crate::codegen::CodegenError;

use super::helpers::fmt_f64;
use super::nr_helpers::emit_dk_device_evaluation;

/// Emit the body of `CircuitState::recompute_dc_op()` for a DK-path circuit.
///
/// **Current state (Phase E.4)**: builds a local `g_aug` matrix and a local
/// `v_node` warm-started from `self.v_prev`, extracts `v_nl = N_v · v_node`,
/// and evaluates the per-device current + dense Jacobian. Nothing consumes
/// those values yet — the NR loop (E.5) will iterate `g_aug`, `i_nl`, and
/// `j_dev` to convergence, then write back to `self.dc_operating_point`,
/// `self.v_prev`, and `self.i_nl_prev`. Until E.5 lands, everything is
/// discarded with `let _ = (...);` so the emitted method stays compilable
/// and callable (as a no-op semantically).
pub(super) fn emit_recompute_dc_op_body_dk(ir: &CircuitIR) -> Result<String, CodegenError> {
    let mut body = String::new();
    body.push_str(
        "        // Phase E MVP (Oomox P6) — Direct Newton-Raphson DC OP recompute.\n\
         \x20       //\n\
         \x20       // Phases E.3+E.4 build `g_aug` and evaluate per-device currents +\n\
         \x20       // Jacobians from the live pot/switch state and a warm-started\n\
         \x20       // `v_node`. Phase E.5 wraps them in the NR loop and writes back\n\
         \x20       // to `self.dc_operating_point`, `self.v_prev`, and\n\
         \x20       // `self.i_nl_prev`. Until all phases land, calling this is a\n\
         \x20       // no-op — plugin code that needs jittered-pot convergence must\n\
         \x20       // keep using the `WARMUP_SAMPLES_RECOMMENDED` silence loop.\n\
         \x20       //\n\
         \x20       // Scope notes for the MVP (subject to revision before E.6):\n\
         \x20       //   * Parasitic-BJT internal nodes (RB/RC/RE > 0) are NOT\n\
         \x20       //     expanded on the DK path — see `docs/aidocs/DC_OP.md`.\n\
         \x20       //   * Inductor-as-short augmented rows are not re-stamped;\n\
         \x20       //     `G` already carries the trapezoidal companion shunt\n\
         \x20       //     `g_eq = T/(2L)`, so the fixed point solved here matches\n\
         \x20       //     `process_sample(0.0, ...)` steady state, not the\n\
         \x20       //     inductor-short DC OP that `dc_op.rs` would compute.\n\
         \x20       //     For inductor-free circuits these agree bitwise.\n",
    );
    body.push_str(&emit_dc_op_build_g_aug_dk(ir));

    // E.4: per-device i_nl + dense Jacobian from a warm-started v_node.
    // Wrapped in a scoped block so the transient `v_d{i}` / `i_dev{i}` /
    // `jdev_{r}_{c}` locals shared with `solve_nonlinear` don't leak into
    // the outer function body.
    if ir.topology.m > 0 && !ir.device_slots.is_empty() {
        body.push_str(&emit_dc_op_evaluate_devices_dk(ir, "        ")?);
    }

    // Phase E.5 will iterate the NR loop. Until then discard everything so
    // the unused-variable lint stays quiet. The discard list grows as
    // subsequent phases add more locals (v_node / i_nl / j_dev).
    body.push_str(
        "\n        // Phase E.5 will iterate the NR loop over `g_aug`, `i_nl`,\n\
         \x20       // and `j_dev`. Until then, discard so the unused-variable\n\
         \x20       // lint stays quiet.\n\
         \x20       let _ = g_aug;\n",
    );
    Ok(body)
}

/// Emit Rust that constructs a local `g_aug: [[f64; N]; N]` from:
///   * the const `G` (input conductance, VS/VCVS rows, opamp VCCS, linearized
///     triodes, inductor companion shunts — all already baked at codegen time),
///   * per-pot conductance deltas `1/self.pot_i_resistance - POT_i_G_NOM`,
///   * per-switch resistor deltas `1/R_new - 1/R_nominal` (C/L switch
///     components do not affect `g_aug`; capacitors are open at DC and
///     inductors are handled by the const-`G` companion shunt — see module
///     docstring for the MVP simplification).
///
/// Mirrors the pot / switch-R stamps in `emit_switch_methods`'
/// `rebuild_matrices` so the two paths stay consistent. Kept inline (no
/// helper emission) to avoid coupling to the `stamp_conductance` fn which is
/// only emitted when the circuit has inductors or switches — otherwise a
/// pots-only circuit would fail to compile.
fn emit_dc_op_build_g_aug_dk(ir: &CircuitIR) -> String {
    let mut body = String::new();

    body.push_str(
        "\n        // Start from the const G matrix (input conductance, opamp VCCS,\n\
         \x20       // linearized-triode stamps, and inductor companion shunts are\n\
         \x20       // already included).\n\
         \x20       let mut g_aug: [[f64; N]; N] = G;\n",
    );

    // --- Switch resistor deltas --------------------------------------------
    //
    // At DC only R-type switch components affect `g_aug`. Capacitors are open
    // (no G contribution) and inductors are shorts but the companion shunt is
    // already in G (see module-level note).
    let has_r_switch = ir
        .switches
        .iter()
        .any(|sw| sw.components.iter().any(|c| c.component_type == 'R'));
    if has_r_switch {
        body.push_str(
            "\n        // Apply switch R-component deltas at the current position.\n",
        );
        for sw in &ir.switches {
            for (ci, comp) in sw.components.iter().enumerate() {
                if comp.component_type != 'R' {
                    continue;
                }
                let nominal = super::helpers::fmt_f64(comp.nominal_value);
                body.push_str(&format!(
                    "        {{\n\
                     \x20           let new_val = SWITCH_{sw_idx}_VALUES[self.switch_{sw_idx}_position][{ci}];\n\
                     \x20           let delta_g = 1.0 / new_val - 1.0 / {nominal};\n\
                     \x20           stamp_conductance(&mut g_aug, SWITCH_{sw_idx}_COMP_{ci}_NODE_P, SWITCH_{sw_idx}_COMP_{ci}_NODE_Q, delta_g);\n\
                     \x20       }}\n",
                    sw_idx = sw.index,
                ));
            }
        }
    }

    // --- Pot conductance deltas --------------------------------------------
    //
    // Inlined (not `stamp_conductance`) so pots-only circuits without
    // switches or inductors still compile — the helper is only emitted when
    // one of those exists.
    if !ir.pots.is_empty() {
        body.push_str(
            "\n        // Apply pot conductance deltas (current resistance vs nominal).\n",
        );
        for (idx, pot) in ir.pots.iter().enumerate() {
            let np = pot.node_p;
            let nq = pot.node_q;
            body.push_str(&format!(
                "        {{\n\
                 \x20           let delta_g = 1.0 / self.pot_{idx}_resistance - POT_{idx}_G_NOM;\n",
            ));
            if np > 0 {
                body.push_str(&format!(
                    "            g_aug[{i}][{i}] += delta_g;\n",
                    i = np - 1
                ));
            }
            if nq > 0 {
                body.push_str(&format!(
                    "            g_aug[{j}][{j}] += delta_g;\n",
                    j = nq - 1
                ));
            }
            if np > 0 && nq > 0 {
                body.push_str(&format!(
                    "            g_aug[{i}][{j}] -= delta_g;\n\
                     \x20           g_aug[{j}][{i}] -= delta_g;\n",
                    i = np - 1,
                    j = nq - 1
                ));
            }
            body.push_str("        }\n");
        }
    }

    // --- Op-amp AOL cap (deliberately skipped in MVP) ---------------------
    //
    // `dc_op.rs::build_dc_system` caps each op-amp's effective AOL at
    // `AOL_DC_MAX = 1000` to keep NR stable through precision-rectifier
    // multi-equilibria. The transient NR paths do NOT apply this cap, so
    // matching their behavior here keeps `recompute_dc_op` converging to the
    // same equilibrium that `process_sample(0.0, ...)` would reach. Circuits
    // that need the cap (basin-trap-prone precision rectifiers) are already
    // scoped out of the MVP — plugins on those topologies keep using the
    // `WARMUP_SAMPLES_RECOMMENDED` silence loop.

    body
}

/// Emit `let v_d{i} = <N_v row i> · v_node;` bindings — one per nonlinear
/// dimension.
///
/// Structurally identical to `dk_emitter::emit_extract_voltages` except
/// * source is `v_node` (not `v_pred` — the DC-OP NR iterates over node
///   voltages, not the Schur-decomposed predictor),
/// * output is individual `let`-bindings rather than an array initializer
///   (so the device-eval helper can consume them directly).
///
/// Sparsity pattern comes from `ir.sparsity.n_v.nz_by_row`, coefficient
/// values from `ir.n_v(i, j)`. Rows with no non-zero columns emit `0.0` —
/// which can happen in pathological device layouts and stays valid Rust
/// even though it short-circuits the device evaluator (the paired
/// `i_dev{i}` will evaluate to whatever the device helpers return at zero
/// controlling voltage, which may or may not be physically meaningful).
fn emit_dc_op_extract_v_nl_dk(ir: &CircuitIR, indent: &str) -> String {
    let m = ir.topology.m;
    let mut out = String::new();
    for i in 0..m {
        let nz_cols = &ir.sparsity.n_v.nz_by_row[i];
        let mut expr = String::new();
        if nz_cols.is_empty() {
            expr.push_str("0.0");
        } else {
            let mut first = true;
            for &j in nz_cols {
                let coeff = ir.n_v(i, j);
                let abs_val = coeff.abs();
                let is_negative = coeff < 0.0;
                if first {
                    if is_negative {
                        expr.push('-');
                    }
                } else if is_negative {
                    expr.push_str(" - ");
                } else {
                    expr.push_str(" + ");
                }
                if (abs_val - 1.0).abs() < 1e-15 {
                    expr.push_str(&format!("v_node[{j}]"));
                } else {
                    expr.push_str(&format!("{} * v_node[{j}]", fmt_f64(abs_val)));
                }
                first = false;
            }
        }
        out.push_str(&format!("{indent}let v_d{i} = {expr};\n"));
    }
    out
}

/// Emit the Phase E.4 per-device evaluator block inside `recompute_dc_op`.
///
/// Layout of the emitted block:
/// ```text
/// {
///     let v_node: [f64; N] = self.v_prev;   // warm-start seed
///     let v_d0 = …; let v_d1 = …; …          // v_nl = N_v · v_node
///     let i_dev0 = …; let jdev_0_0 = …; …    // shared device-eval helper
///     let mut i_nl = [0.0; M];
///     let mut j_dev = [[0.0; M]; M];
///     i_nl[0] = i_dev0; …                    // pack flat locals → dense arrays
///     j_dev[0][0] = jdev_0_0; …
///     let _ = (v_node, i_nl, j_dev);          // E.5 replaces this discard
/// }
/// ```
///
/// The inner locals deliberately reuse the `v_d{i}` / `i_dev{i}` /
/// `jdev_{r}_{c}` naming from `solve_nonlinear` so the shared
/// [`emit_dk_device_evaluation`] helper drops in without renaming.
///
/// Caller guarantees `ir.topology.m > 0` and non-empty `device_slots` — the
/// block would compile when those are empty but packing a `[f64; 0]` is
/// dead code, so the outer dispatcher skips the call entirely in that case.
fn emit_dc_op_evaluate_devices_dk(
    ir: &CircuitIR,
    indent: &str,
) -> Result<String, CodegenError> {
    let m = ir.topology.m;
    let inner = format!("{indent}    ");
    let mut out = String::new();

    out.push_str(&format!(
        "\n{indent}// --- Phase E.4: evaluate device currents + dense Jacobian ----\n\
         {indent}//\n\
         {indent}// Warm-starts from `self.v_prev`, extracts `v_nl = N_v·v_node`,\n\
         {indent}// and reuses the transient `solve_nonlinear` device dispatch via\n\
         {indent}// `nr_helpers::emit_dk_device_evaluation`. E.5 wraps this block\n\
         {indent}// in the NR iteration; until then the outputs are discarded.\n\
         {indent}//\n\
         {indent}// The shared device evaluator emits `state.device_N_*` accessors\n\
         {indent}// (it's the same block `solve_nonlinear(&mut CircuitState)` uses).\n\
         {indent}// `recompute_dc_op(&mut self)` has no `state` binding, so we\n\
         {indent}// introduce one here — immutable borrow is enough because device\n\
         {indent}// evaluation only reads per-device parameters.\n\
         {indent}{{\n\
         {inner}let state: &CircuitState = &*self;\n\
         {inner}let v_node: [f64; N] = state.v_prev;\n\n"
    ));

    // v_nl extraction (from v_node, not v_pred).
    out.push_str(&emit_dc_op_extract_v_nl_dk(ir, &inner));
    out.push('\n');

    // Per-device current + Jacobian emission (shared with solve_nonlinear).
    emit_dk_device_evaluation(&mut out, ir, &inner)?;

    // Pack flat `i_dev{i}` / `jdev_{r}_{c}` locals into dense arrays the
    // NR loop (E.5) will consume.
    out.push_str(&format!(
        "{inner}// Pack into dense i_nl / j_dev for E.5's NR iteration.\n\
         {inner}let mut i_nl: [f64; M] = [0.0; M];\n"
    ));
    for i in 0..m {
        out.push_str(&format!("{inner}i_nl[{i}] = i_dev{i};\n"));
    }
    out.push_str(&format!(
        "{inner}let mut j_dev: [[f64; M]; M] = [[0.0; M]; M];\n"
    ));
    for (dev_num, slot) in ir.device_slots.iter().enumerate() {
        let _ = dev_num;
        let blk_start = slot.start_idx;
        let blk_dim = slot.dimension;
        for r in blk_start..blk_start + blk_dim {
            for c in blk_start..blk_start + blk_dim {
                out.push_str(&format!(
                    "{inner}j_dev[{r}][{c}] = jdev_{r}_{c};\n"
                ));
            }
        }
    }

    // E.5 replaces this with the NR loop.
    out.push_str(&format!(
        "\n{inner}// Discard pending E.5 — keeps the unused-variable lint quiet.\n\
         {inner}let _ = (v_node, i_nl, j_dev);\n\
         {indent}}}\n"
    ));

    Ok(out)
}
