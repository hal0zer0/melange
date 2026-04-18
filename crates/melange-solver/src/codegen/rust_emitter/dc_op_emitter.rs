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
//! 3. `emit_dc_op_evaluate_devices` — per-device i_nl + Jacobian (E.4).
//! 4. `emit_dc_op_solve` — NR loop, limiting, dense LU (E.5).
//! 5. Compose + state resets (E.6).
//! 6. Nodal mirror (E.8).
//!
//! The skeleton body below builds `g_aug` from the current pot/switch fields
//! but does not yet consume it. Subsequent phases iterate the NR loop over
//! this matrix. See the Phase E handoff memory file for the full definition
//! of done.

use crate::codegen::ir::CircuitIR;

/// Emit the body of `CircuitState::recompute_dc_op()` for a DK-path circuit.
///
/// **Current state (Phase E.3)**: builds a local `g_aug` matrix from the
/// const `G`, the live `self.pot_*_resistance` values, and the live
/// `self.switch_*_position` values. The NR loop that consumes `g_aug` lands
/// in subsequent phases; until then the matrix is discarded with a
/// `let _ = g_aug;` so the emitted method stays compilable and callable.
pub(super) fn emit_recompute_dc_op_body_dk(ir: &CircuitIR) -> String {
    let mut body = String::new();
    body.push_str(
        "        // Phase E MVP (Oomox P6) — Direct Newton-Raphson DC OP recompute.\n\
         \x20       //\n\
         \x20       // Phases E.3+ build `g_aug` below from the live pot/switch state,\n\
         \x20       // run NR over it, and write back to `self.dc_operating_point`,\n\
         \x20       // `self.v_prev`, and `self.i_nl_prev`. Until all phases land the\n\
         \x20       // matrix is built but unused — calling this is still a no-op,\n\
         \x20       // so plugin code that needs jittered-pot convergence must keep\n\
         \x20       // using the `WARMUP_SAMPLES_RECOMMENDED` silence loop.\n\
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
    body.push_str(
        "\n        // Phase E.4+ will iterate `g_aug` through the NR loop. Until\n\
         \x20       // then, discard it so the unused-variable lint stays quiet.\n\
         \x20       let _ = g_aug;\n",
    );
    body
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
