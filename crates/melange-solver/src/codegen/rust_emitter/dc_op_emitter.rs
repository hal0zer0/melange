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
//! ## Layering (shipped across E.1 → E.6)
//!
//! 1. `emit_recompute_dc_op_body_dk` — top-level body assembler (DK path).
//! 2. `emit_dc_op_build_g_aug_dk` — G_aug base construction from live
//!    pot/switch state (E.3).
//! 3. `emit_dc_op_build_b_dc_dk` — DC RHS from `RHS_CONST` (halved on
//!    node rows in trapezoidal mode; verbatim under BE-primary) +
//!    `.runtime` voltage source fields (E.5).
//! 4. `emit_dc_op_extract_v_nl_dk` + `emit_dk_device_evaluation` —
//!    per-device i_nl + Jacobian evaluator shared with transient NR (E.4).
//! 5. `emit_dc_op_nr_loop_dk` — Direct NR loop with LU solve, flat
//!    damping, convergence check (E.5).
//! 6. `emit_dc_op_writeback_dk` — write converged `v_node` + `i_nl` back
//!    to state, seed DC blocker at new DC output, sync pot `_prev`,
//!    zero oversampler/inductor/transformer history (E.5 + E.6).
//! 7. `emit_recompute_dc_op_body_nodal` — permanent stub emitter for the
//!    nodal full-LU path. Emits a method that bumps
//!    `diag_nr_max_iter_count` and returns, so plugins can install the
//!    Phase E surface uniformly while nodal-routed circuits continue
//!    using the `WARMUP_SAMPLES_RECOMMENDED` silence loop (the
//!    documented path for nodal circuits). The full nodal NR body is
//!    deferred indefinitely — see the function's doc comment for the
//!    rationale and the reviver's plan.
//!
//! ## DC fixed-point algebra (E.5 derivation)
//!
//! The trapezoidal per-sample equation is
//!
//! ```text
//!     A · v_{n+1} = RHS_CONST + A_neg · v_n + N_i · i_nl_prev + input
//! ```
//!
//! and the NR loop in `process_sample` adds `N_i · i_nl(v_{n+1})` to both
//! sides. Substituting steady state `v_{n+1} = v_n = v_dc`, `input = 0`,
//! `i_nl_prev = i_nl_dc`, and using `A - A_neg = 2·G` on node rows (`A_neg`
//! is zeroed on VS/VCVS algebraic rows by `get_a_neg_matrix`, so the row-
//! wise identity `A - A_neg = G` holds there and `N_i`'s VS-row entries are
//! structurally zero anyway) gives the DC fixed point
//!
//! ```text
//!     2·G · v_dc = RHS_CONST + 2·N_i · i_nl_dc            (node rows)
//!        G · v_dc = RHS_CONST                               (VS/VCVS rows)
//! ```
//!
//! Halving the node-row equation and folding both into a single Newton step
//! yields the compile-time `dc_op.rs` form
//!
//! ```text
//!     G_aug_nr = g_aug − N_i · J_dev · N_v
//!     rhs_nr   = b_dc  + N_i · (i_nl − J_dev · v_nl)
//!     v_new    = G_aug_nr⁻¹ · rhs_nr
//! ```
//!
//! with `b_dc` built from `RHS_CONST` by halving rows `[0..n_nodes)` — **in
//! trapezoidal mode only**. Under BE-primary integration
//! (`solver_config.backward_euler`) the baked `RHS_CONST` is already the ×1
//! BE build and the row-wise identity is `A − A_neg = G` on ALL rows (BE
//! also skips the `N_i·i_nl_prev` history stamp), so `b_dc = RHS_CONST`
//! verbatim — halving there would converge to a fixed point with HALF the
//! DC current-source injection.
//!
//! This converges to the exact compile-time DC OP for inductor-free
//! circuits. Inductor-bearing circuits: the `g_eq = T/(2L)` companion shunt
//! already baked into `G` differs from the inductor-short augmented row the
//! compile-time solver uses, so the solved fixed point is only a true
//! `process_sample(0.0)` equilibrium when `V_L ≈ 0` across every companion
//! winding (i.e. no DC current through the winding). The writeback guards
//! this honestly: if any winding sees `|V_L|` above a small tolerance, the
//! recompute reports failure (bumps `diag_nr_max_iter_count`) instead of
//! writing back a false equilibrium, and `settle_dc_op` routes to the
//! `WARMUP_SAMPLES_RECOMMENDED` loop which does reach the true OP.

use crate::codegen::ir::CircuitIR;
use crate::codegen::CodegenError;

use super::helpers::{fmt_f64, oversampling_info};
use super::nr_helpers::emit_dk_device_evaluation;

/// Emit the body of `CircuitState::recompute_dc_op()` for a DK-path circuit.
///
/// Assembles the full Direct-NR runtime DC OP solve:
///   1. Build the resistive `g_aug` base matrix from `G` + live pot/switch
///      state (E.3).
///   2. Build the DC RHS `b_dc` from `RHS_CONST` (halving node rows) and
///      any `.runtime` voltage source fields (E.5).
///   3. Direct NR loop: warm-start from `self.v_prev`, extract `v_nl`,
///      evaluate per-device currents + Jacobian, LU-solve for the Newton
///      step, apply flat voltage damping + clamp, check convergence (E.5).
///   4. On convergence, write the solution back to `self.dc_operating_point`,
///      `self.v_prev`, `self.i_nl_prev` / `i_nl_prev_prev`, and clear the
///      DC blocker's sample history so it doesn't need seconds to re-settle
///      from a stale offset (E.5).
///
/// Linear-only circuits (`M == 0` or no device slots) short-circuit the NR
/// loop: a single `invert_n_equilibrated(g_aug)` solve gives the resistive fixed point.
pub(super) fn emit_recompute_dc_op_body_dk(ir: &CircuitIR) -> Result<String, CodegenError> {
    let mut body = String::new();
    body.push_str(
        "        // Phase E MVP (Oomox P6) — Direct Newton-Raphson DC OP recompute.\n\
         \x20       //\n\
         \x20       // Warm-starts from `self.v_prev` (a physically valid prior\n\
         \x20       // equilibrium), rebuilds `g_aug` from live pot/switch state,\n\
         \x20       // and iterates classic companion-linearized NR until the node-\n\
         \x20       // voltage step falls below the convergence tolerance. On\n\
         \x20       // convergence the result is written back; on failure, state is\n\
         \x20       // left untouched and `diag_nr_max_iter_count` is bumped so the\n\
         \x20       // caller can fall back to the `WARMUP_SAMPLES_RECOMMENDED`\n\
         \x20       // silence loop.\n\
         \x20       //\n\
         \x20       // Scope notes (documented in the method's public docstring):\n\
         \x20       //   * Parasitic-BJT internal nodes (RB/RC/RE > 0) are NOT\n\
         \x20       //     expanded on the DK path — see `docs/aidocs/DC_OP.md`.\n\
         \x20       //   * Inductor-as-short augmented rows are not re-stamped;\n\
         \x20       //     `G` carries the companion shunt `g_eq = T/(2L)` instead,\n\
         \x20       //     so the solved fixed point is only a true equilibrium\n\
         \x20       //     when V_L ≈ 0 across every companion winding. Windings\n\
         \x20       //     carrying DC current are detected after convergence and\n\
         \x20       //     reported as failure (see writeback guard) so callers\n\
         \x20       //     fall back to the warmup loop rather than accepting a\n\
         \x20       //     false equilibrium. Inductor-free circuits agree bitwise\n\
         \x20       //     with `dc_op.rs`.\n\
         \x20       //   * No source / Gmin stepping, no basin-trap handling.\n",
    );
    body.push_str(&emit_dc_op_build_g_aug_dk(ir));
    body.push_str(&emit_dc_op_build_b_dc_dk(ir));

    if ir.topology.m == 0 || ir.device_slots.is_empty() {
        // Linear circuit: single LU solve. No NR loop needed.
        body.push_str(&emit_dc_op_linear_solve_dk(ir));
    } else {
        body.push_str(&emit_dc_op_nr_loop_dk(ir)?);
    }

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
        body.push_str("\n        // Apply switch R-component deltas at the current position.\n");
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

/// Emit the DC RHS vector `b_dc: [f64; N]` for `recompute_dc_op`.
///
/// Maps `RHS_CONST` (the per-sample Norton current vector) to the DC
/// steady-state RHS:
///   * Node rows `[0..n_nodes)` are halved in trapezoidal mode (`RHS_CONST`
///     doubles current-source injections for trapezoidal averaging — at DC
///     that averaging collapses back to the single DC source value). Under
///     BE-primary integration (`solver_config.backward_euler`) the baked
///     `RHS_CONST` is already the ×1 BE build (`A − A_neg = G` row-wise), so
///     node rows are preserved verbatim — halving would solve a fixed point
///     with half the DC current-source injection.
///   * VS / VCVS / ideal-transformer aug rows `[n_nodes..n_aug)` are preserved
///     (their per-sample value is already the algebraic RHS — `V_dc` for VS,
///     0 for VCVS / ideal-xfmr KVL — with no trapezoidal scaling).
///   * Inductor branch rows `[n_aug..N)` are preserved (zero, since inductor
///     short-circuit constraints contribute 0 to `RHS_CONST` by construction
///     in `dk::build_rhs_const`).
///   * `.runtime` voltage sources add `self.<field>` to their target VS row
///     so `recompute_dc_op` converges to the bias point the host will drive
///     on the first real sample.
fn emit_dc_op_build_b_dc_dk(ir: &CircuitIR) -> String {
    let mut body = String::new();
    let n_nodes = ir.topology.n_nodes;

    let halve_node_rows = !ir.solver_config.backward_euler;
    if halve_node_rows {
        body.push_str(
            "\n        // Build b_dc: DC steady-state RHS from the per-sample RHS_CONST.\n\
             \x20       // Node rows are halved (trapezoidal averaging collapses at DC);\n\
             \x20       // VS/VCVS/ideal-xfmr algebraic rows and inductor branch rows are\n\
             \x20       // preserved verbatim. See module-level DC fixed-point derivation.\n",
        );
    } else {
        body.push_str(
            "\n        // Build b_dc: DC steady-state RHS from the per-sample RHS_CONST.\n\
             \x20       // BE-primary: RHS_CONST is the ×1 backward-Euler build and\n\
             \x20       // A − A_neg = G holds row-wise, so b_dc = RHS_CONST verbatim\n\
             \x20       // (no node-row halving — that is trapezoidal-only algebra).\n\
             \x20       // See module-level DC fixed-point derivation.\n",
        );
    }
    if ir.has_dc_sources {
        body.push_str("        let mut b_dc: [f64; N] = RHS_CONST;\n");
        if n_nodes > 0 && halve_node_rows {
            body.push_str(&format!(
                "        for i in 0..{n_nodes} {{ b_dc[i] *= 0.5; }}\n",
            ));
        }
    } else {
        body.push_str("        let mut b_dc: [f64; N] = [0.0; N];\n");
    }

    if !ir.runtime_sources.is_empty() {
        body.push_str(
            "\n        // `.runtime` voltage sources: include their current value so\n\
             \x20       // recompute converges to the equilibrium the host will drive.\n",
        );
        for rt in &ir.runtime_sources {
            body.push_str(&format!(
                "        b_dc[{row}] += self.{field};\n",
                row = rt.vs_row,
                field = rt.field_name,
            ));
        }
    }

    body
}

/// Emit the linear-only DC OP solve (`M == 0` or no device slots).
///
/// Skips the NR loop entirely: the system `g_aug · v = b_dc` is already
/// linear, so a single `invert_n_equilibrated` + matrix-vector multiply gives the fixed
/// point. On singular `g_aug`, state is left unchanged and both
/// `diag_singular_matrix_count` and `diag_nr_max_iter_count` are bumped —
/// the latter is the signal `settle_dc_op` watches, so the warmup fallback
/// engages instead of silently proceeding with stale state.
fn emit_dc_op_linear_solve_dk(_ir: &CircuitIR) -> String {
    let mut body = String::new();
    body.push_str(
        "\n        // Linear circuit (M == 0): solve g_aug · v = b_dc once.\n\
         \x20       // Use the equilibrated inverse — g_aug can be badly scaled\n\
         \x20       // (mixed conductance/constraint rows), and the runtime pot/\n\
         \x20       // switch values it was rebuilt from can push cond(g_aug) far\n\
         \x20       // beyond the codegen-time matrix.\n\
         \x20       let (g_inv, singular) = invert_n_equilibrated(g_aug);\n\
         \x20       if singular {\n\
         \x20           self.diag_singular_matrix_count += 1;\n\
         \x20           // settle_dc_op watches diag_nr_max_iter_count exclusively as\n\
         \x20           // its \"state not updated\" signal — bump it too so the\n\
         \x20           // WARMUP_SAMPLES_RECOMMENDED fallback engages.\n\
         \x20           self.diag_nr_max_iter_count += 1;\n\
         \x20           return;\n\
         \x20       }\n\
         \x20       let mut v_node = [0.0_f64; N];\n\
         \x20       for i in 0..N {\n\
         \x20           let mut sum = 0.0;\n\
         \x20           for j in 0..N {\n\
         \x20               sum += g_inv[i][j] * b_dc[j];\n\
         \x20           }\n\
         \x20           v_node[i] = sum;\n\
         \x20       }\n",
    );
    body.push_str(&emit_dc_op_writeback_dk(_ir, /*nonlinear=*/ false));
    body
}

/// Emit the Direct-Newton-Raphson loop for the nonlinear DC OP solve.
///
/// Wraps the shared `emit_dk_device_evaluation` helper inside a fixed-
/// iteration loop that:
///   1. Extracts `v_nl = N_v · v_node` from the current iterate.
///   2. Evaluates per-device currents + dense Jacobian.
///   3. Builds `G_aug_nr = g_aug − N_i · J_dev · N_v`.
///   4. Builds `rhs_nr = b_dc + N_i · (i_nl − J_dev · v_nl)`.
///   5. LU-solves `G_aug_nr · v_new = rhs_nr` via the emitted `invert_n_equilibrated`
///      helper (returns `(inv, singular)`).
///   6. Applies flat global voltage damping when any node-voltage element
///      of the step exceeds the damping threshold, plus a per-element
///      clamp, and advances `v_node`. Damping/clamp sweep VOLTAGE rows
///      only (`[0..n_nodes)`); VS/VCVS branch-current rows (amperes) are
///      applied undamped, mirroring `dc_op.rs`'s `is_voltage_row`
///      classification.
///   7. Declares convergence when every applied step element satisfies the
///      SPICE-style per-variable check `|Δ| < RELTOL·|v| + TOL`.
///
/// On convergence `v_node` and the final `i_nl` are written back via
/// `emit_dc_op_writeback_dk`; on max-iter exhaustion `diag_nr_max_iter_count`
/// is bumped and state is left untouched so the caller can fall back to the
/// warmup loop.
fn emit_dc_op_nr_loop_dk(ir: &CircuitIR) -> Result<String, CodegenError> {
    const MAX_ITER: usize = 200;
    const TOL: f64 = 1e-9;
    /// Relative convergence term, mirroring `dc_op.rs::DcOpConfig::reltol`
    /// (SPICE-style `|Δ| < reltol·|v| + abstol`). Keeps 300 V circuits from
    /// false-failing at the LU round-off floor, where an absolute-only 1e-9
    /// check can never be satisfied.
    const RELTOL: f64 = 1e-6;
    const DAMP_THRESHOLD: f64 = 10.0;
    const CLAMP: f64 = 50.0;

    let m = ir.topology.m;
    let mut out = String::new();

    out.push_str(&format!(
        "\n        // --- Direct Newton-Raphson DC OP loop -----------------------\n\
         \x20       let mut v_node: [f64; N] = self.v_prev;\n\
         \x20       let mut i_nl_final: [f64; M] = [0.0; M];\n\
         \x20       let mut nr_converged = false;\n\
         \x20       const MAX_ITER: usize = {MAX_ITER};\n\
         \x20       const TOL: f64 = {TOL:e};\n\
         \x20       const RELTOL: f64 = {RELTOL:e};\n\
         \x20       for _iter in 0..MAX_ITER {{\n"
    ));

    let inner = "            ";

    // v_nl extraction — locals `v_d{i}` live inside the loop body.
    out.push_str(&format!(
        "{inner}// Extract controlling voltages v_nl = N_v · v_node.\n"
    ));
    out.push_str(&emit_dc_op_extract_v_nl_dk(ir, inner));
    out.push('\n');

    // MOSFET body effect: refresh vt_eff from the current iterate BEFORE
    // taking the immutable `state` alias below. The shared evaluator reads
    // `state.device_{d}_vt`; freezing it at its pre-recompute value would
    // converge a GAMMA-carrying MOSFET with Vsb ≠ 0 to a no-body-effect OP
    // (both the offline `dc_op.rs` solver and the transient NR update
    // vt_eff per iteration / per sample). Formula replicates the transient
    // emission in dk_emitter's `body_effect_update` with `v_node` standing
    // in for `v_pred`.
    {
        use crate::codegen::ir::DeviceParams;
        let mut body_effect = String::new();
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            if let DeviceParams::Mosfet(mp) = &slot.params {
                if mp.has_body_effect() {
                    let vs_expr = if mp.source_node > 0 {
                        format!("v_node[{}]", mp.source_node - 1)
                    } else {
                        "0.0".to_string()
                    };
                    let vb_expr = if mp.bulk_node > 0 {
                        format!("v_node[{}]", mp.bulk_node - 1)
                    } else {
                        "0.0".to_string()
                    };
                    let sign = if mp.is_p_channel { -1.0 } else { 1.0 };
                    body_effect.push_str(&format!(
                        "{inner}{{ // MOSFET {dev_num} body effect (per-iteration vt_eff)\n\
                         {inner}    let vsb = ({sign:.1}) * ({vs_expr} - {vb_expr});\n\
                         {inner}    self.device_{dev_num}_vt = DEVICE_{dev_num}_VT + ({sign:.1}) * DEVICE_{dev_num}_GAMMA * ((DEVICE_{dev_num}_PHI + vsb.max(0.0)).sqrt() - DEVICE_{dev_num}_PHI.sqrt());\n\
                         {inner}}}\n"
                    ));
                }
            }
        }
        if !body_effect.is_empty() {
            out.push_str(&format!(
                "{inner}// Refresh body-effect VT from the current iterate (mirrors the\n\
                 {inner}// transient per-sample update; offline dc_op.rs does the same\n\
                 {inner}// per NR iteration).\n"
            ));
            out.push_str(&body_effect);
            out.push('\n');
        }
    }

    // Device evaluation — shared helper emits `state.device_*` reads, so we
    // need a `state: &CircuitState` local inside the loop body. Immutable
    // borrow is enough because device evaluation never mutates self.
    out.push_str(&format!(
        "{inner}// Reuse the transient per-device evaluator (shared helper).\n\
         {inner}let state: &CircuitState = &*self;\n"
    ));
    // use_k_eff=false: DC-OP recompute uses node-voltage NR with G_aug, not
    // state.k. v_d holds external terminal voltages, so parasitic BJTs need
    // the inner-NR path (bjt_with_parasitics) when has_internal_mna_nodes=false.
    emit_dk_device_evaluation(&mut out, ir, inner, false)?;

    // Pack flat per-device locals into dense arrays.
    out.push_str(&format!(
        "{inner}// Pack flat device locals into dense i_nl / j_dev arrays.\n\
         {inner}let mut i_nl: [f64; M] = [0.0; M];\n"
    ));
    for i in 0..m {
        out.push_str(&format!("{inner}i_nl[{i}] = i_dev{i};\n"));
    }
    out.push_str(&format!(
        "{inner}let mut j_dev: [[f64; M]; M] = [[0.0; M]; M];\n"
    ));
    for slot in &ir.device_slots {
        let s = slot.start_idx;
        let d = slot.dimension;
        for r in s..s + d {
            for c in s..s + d {
                out.push_str(&format!("{inner}j_dev[{r}][{c}] = jdev_{r}_{c};\n"));
            }
        }
    }

    // v_nl as a dense array for the J·v_nl product below.
    out.push_str(&format!("{inner}let v_nl: [f64; M] = ["));
    for i in 0..m {
        if i > 0 {
            out.push_str(", ");
        }
        out.push_str(&format!("v_d{i}"));
    }
    out.push_str("];\n\n");

    // Build G_aug_nr = g_aug − N_i · J_dev · N_v.
    // We use the N_I / N_V consts directly (dense scan). At typical M ≤ 6
    // the inner loop is trivially small; optimizer unrolls it. `N_I` is
    // stored transposed as [M][N], so `N_i[a][i] = N_I[i][a]`.
    out.push_str(&format!(
        "{inner}// Apply companion linearization: g_aug_nr = g_aug − N_i · J · N_v.\n\
         {inner}let mut g_aug_nr = g_aug;\n\
         {inner}for a in 0..N {{\n\
         {inner}    for b in 0..N {{\n\
         {inner}        let mut corr = 0.0;\n\
         {inner}        for i in 0..M {{\n\
         {inner}            let ni_ai = N_I[i][a];\n\
         {inner}            if ni_ai == 0.0 {{ continue; }}\n\
         {inner}            for j in 0..M {{\n\
         {inner}                corr += ni_ai * j_dev[i][j] * N_V[j][b];\n\
         {inner}            }}\n\
         {inner}        }}\n\
         {inner}        g_aug_nr[a][b] -= corr;\n\
         {inner}    }}\n\
         {inner}}}\n\n"
    ));

    // Build rhs_nr = b_dc + N_i · (i_nl − J_dev · v_nl).
    out.push_str(&format!(
        "{inner}// Build rhs_nr = b_dc + N_i · (i_nl − J · v_nl).\n\
         {inner}let mut rhs_nr = b_dc;\n\
         {inner}for i in 0..M {{\n\
         {inner}    let mut jv = 0.0;\n\
         {inner}    for j in 0..M {{\n\
         {inner}        jv += j_dev[i][j] * v_nl[j];\n\
         {inner}    }}\n\
         {inner}    let i_comp = i_nl[i] - jv;\n\
         {inner}    if i_comp == 0.0 {{ continue; }}\n\
         {inner}    for a in 0..N {{\n\
         {inner}        rhs_nr[a] += N_I[i][a] * i_comp;\n\
         {inner}    }}\n\
         {inner}}}\n\n"
    ));

    // LU-solve.
    out.push_str(&format!(
        "{inner}// Solve G_aug_nr · v_new = rhs_nr via the shared equilibrated inverse\n\
         {inner}// (same rationale as the linear solve: G_aug_nr mixes conductance,\n\
         {inner}// constraint, and device-Jacobian scales).\n\
         {inner}let (g_inv, singular) = invert_n_equilibrated(g_aug_nr);\n\
         {inner}if singular {{\n\
         {inner}    self.diag_singular_matrix_count += 1;\n\
         {inner}    break;\n\
         {inner}}}\n\
         {inner}let mut v_new = [0.0_f64; N];\n\
         {inner}for a in 0..N {{\n\
         {inner}    let mut sum = 0.0;\n\
         {inner}    for b in 0..N {{\n\
         {inner}        sum += g_inv[a][b] * rhs_nr[b];\n\
         {inner}    }}\n\
         {inner}    v_new[a] = sum;\n\
         {inner}}}\n\n"
    ));

    // Guard against NaN/Inf — if any entry is non-finite, bail.
    out.push_str(&format!(
        "{inner}if !v_new.iter().all(|x| x.is_finite()) {{\n\
         {inner}    self.diag_nan_reset_count += 1;\n\
         {inner}    break;\n\
         {inner}}}\n\n"
    ));

    // Global flat damping + per-element clamp. Matches the simple-form
    // damping used in `dc_op.rs::nr_dc_solve` (scale all deltas by
    // `DAMP_THRESHOLD / max_delta` when max exceeds threshold), including
    // its voltage-row classification: only rows [0..n_nodes) are node
    // voltages on the DK path (internal parasitic nodes are not expanded
    // here); rows [n_nodes..N) are VS/VCVS branch currents in AMPERES —
    // sweeping them into the voltage damping scan or the ±50 "V" clamp
    // would stall convergence whenever a branch current legitimately
    // exceeds the thresholds. Convergence uses the SPICE-style
    // per-variable check `|Δ| < RELTOL·|v| + TOL` (mirrors dc_op.rs
    // reltol) so high-voltage circuits (300 V rails) don't false-fail at
    // the LU round-off floor.
    let n_v_rows = ir.topology.n_nodes;
    out.push_str(&format!(
        "{inner}// Flat damping + per-element clamp on the Newton step.\n\
         {inner}// Voltage rows only ([0..{n_v_rows}) = circuit nodes); VS/VCVS\n\
         {inner}// branch-current rows are in amperes and are applied undamped\n\
         {inner}// (mirrors dc_op.rs is_voltage_row classification).\n\
         {inner}let mut max_delta = 0.0_f64;\n\
         {inner}for a in 0..{n_v_rows} {{\n\
         {inner}    let d = (v_new[a] - v_node[a]).abs();\n\
         {inner}    if d > max_delta {{ max_delta = d; }}\n\
         {inner}}}\n\
         {inner}let damping = if max_delta > {DAMP_THRESHOLD:.1}_f64 {{\n\
         {inner}    ({DAMP_THRESHOLD:.1}_f64 / max_delta).max(0.1)\n\
         {inner}}} else {{ 1.0 }};\n\
         {inner}if damping < 1.0 {{ self.diag_voltage_damp_count += 1; }}\n\
         {inner}// Per-variable SPICE-style convergence on the APPLIED step:\n\
         {inner}// |delta| < RELTOL·|v| + TOL (abstol). See dc_op.rs.\n\
         {inner}let mut all_within_tol = true;\n\
         {inner}for a in 0..N {{\n\
         {inner}    let delta = v_new[a] - v_node[a];\n\
         {inner}    let limited = if a < {n_v_rows} {{\n\
         {inner}        (delta * damping).clamp(-{CLAMP:.1}_f64, {CLAMP:.1}_f64)\n\
         {inner}    }} else {{\n\
         {inner}        delta\n\
         {inner}    }};\n\
         {inner}    v_node[a] += limited;\n\
         {inner}    if limited.abs() >= RELTOL * v_node[a].abs() + TOL {{\n\
         {inner}        all_within_tol = false;\n\
         {inner}    }}\n\
         {inner}}}\n\
         {inner}i_nl_final = i_nl;\n\
         {inner}if all_within_tol {{\n\
         {inner}    nr_converged = true;\n\
         {inner}    break;\n\
         {inner}}}\n\
         \x20       }}\n\n"
    ));

    // Max-iter fallback path.
    out.push_str(
        "        if !nr_converged {\n\
         \x20           self.diag_nr_max_iter_count += 1;\n\
         \x20           return;\n\
         \x20       }\n",
    );

    out.push_str(&emit_dc_op_writeback_dk(ir, /*nonlinear=*/ true));
    Ok(out)
}

/// Emit state writeback for a converged DC OP solve.
///
/// Writes the converged node voltages to `self.dc_operating_point`, seeds
/// `self.v_prev`, and (when the circuit has nonlinear devices) both
/// `self.i_nl_prev` and `self.i_nl_prev_prev` with the converged `i_nl`.
///
/// Also touches the following mirror-of-`reset()` state so the first
/// `process_sample` call after `recompute_dc_op` starts from a consistent
/// equilibrium instead of the stale pre-recompute trajectory:
///
///   * `dc_block_x_prev[k] = v_node[OUTPUT_NODES[k]]`, `dc_block_y_prev[k] = 0`
///     — the DC-blocker IIR's fixed point is `y = 0, x = V_dc`. Setting
///     `x_prev` to the raw DC output (not 0) suppresses the step transient
///     the blocker would otherwise generate on the first sample.
///   * `pot_N_resistance_prev = pot_N_resistance` for every pot — the
///     per-sample A_neg correction uses `_prev` to undo last sample's
///     conductance contribution. Without this sync the first sample after
///     recompute would see a phantom pot jump if the user had called
///     `set_pot_N` earlier (which leaves `_prev` pointing at the pre-jitter
///     value).
///   * Oversampler filter taps zeroed (`os_up_state`, `os_dn_state`,
///     and the `_outer` pair for 4× OS). Mid-stream the filter state
///     reflects the old DC level and would ring down as the output steps to
///     the new one; zeroing matches `reset()`.
///   * Linear-companion-model history for inductors / coupled inductors /
///     transformer groups (`ind_{i,v}_prev`, `ind_i_hist`, etc.) zeroed —
///     valid ONLY when `V_L ≈ 0` across every companion winding at the
///     converged point (zero history is the `I_L = V_L = 0` equilibrium).
///     A DC-carrying winding (e.g. a plate choke) has `V_L ≠ 0` at the
///     companion-shunt fixed point, so writing back zeroed history there
///     would be a false equilibrium that slews audibly as `process_sample`
///     rebuilds the true history. The writeback therefore guards: if any
///     winding's `|V_L|` exceeds 1 mV, it bumps `diag_nr_max_iter_count`
///     (the `settle_dc_op` failure signal) and returns with state
///     untouched, routing callers to the `WARMUP_SAMPLES_RECOMMENDED`
///     fallback which does reach the true DC OP.
///
/// Deliberately preserved: `noise_rng_state` (footgun — resetting would make
/// plugins produce identical noise sequences after every parameter change),
/// pot resistances and switch positions (they're the INPUT to this solve),
/// runtime voltage sources, device runtime params, BJT junction temperature,
/// diagnostic counters (caller may be watching them), and `last_nr_iterations`
/// (the NR loop itself isn't plumbed through this counter in the MVP).
fn emit_dc_op_writeback_dk(ir: &CircuitIR, nonlinear: bool) -> String {
    let mut body = String::new();

    // Inductor equilibrium guard (honest-failure principle): the zeroed
    // winding history written back below is only a true equilibrium when
    // V_L ≈ 0 across every companion winding. Check the converged node
    // voltages first; on violation, report failure via the settle_dc_op
    // signal counter and leave state untouched.
    let has_windings = !ir.inductors.is_empty()
        || !ir.coupled_inductors.is_empty()
        || !ir.transformer_groups.is_empty();
    if has_windings {
        let node_expr = |node: usize| -> String {
            if node > 0 {
                format!("v_node[{}]", node - 1)
            } else {
                "0.0".to_string()
            }
        };
        body.push_str(
            "\n        // --- Inductor equilibrium guard ---------------------------\n\
             \x20       // The zeroed winding history below is only an equilibrium when\n\
             \x20       // V_L = 0. A DC-carrying winding (plate choke, DC-biased xfmr\n\
             \x20       // primary) sees V_L = I·2L/T at the companion fixed point —\n\
             \x20       // writing back a false \"settled\" state would slew audibly as\n\
             \x20       // process_sample rebuilds the true history. Refuse honestly:\n\
             \x20       // bump the settle_dc_op failure signal so callers run the\n\
             \x20       // WARMUP_SAMPLES_RECOMMENDED loop (which reaches the true OP).\n\
             \x20       let mut max_winding_v = 0.0_f64;\n",
        );
        for ind in &ir.inductors {
            body.push_str(&format!(
                "        {{ let vl = ({} - {}).abs(); if vl > max_winding_v {{ max_winding_v = vl; }} }}\n",
                node_expr(ind.node_i),
                node_expr(ind.node_j),
            ));
        }
        for ci in &ir.coupled_inductors {
            body.push_str(&format!(
                "        {{ let vl = ({} - {}).abs(); if vl > max_winding_v {{ max_winding_v = vl; }} }}\n",
                node_expr(ci.l1_node_i),
                node_expr(ci.l1_node_j),
            ));
            body.push_str(&format!(
                "        {{ let vl = ({} - {}).abs(); if vl > max_winding_v {{ max_winding_v = vl; }} }}\n",
                node_expr(ci.l2_node_i),
                node_expr(ci.l2_node_j),
            ));
        }
        for g in &ir.transformer_groups {
            for w in 0..g.num_windings {
                body.push_str(&format!(
                    "        {{ let vl = ({} - {}).abs(); if vl > max_winding_v {{ max_winding_v = vl; }} }}\n",
                    node_expr(g.winding_node_i[w]),
                    node_expr(g.winding_node_j[w]),
                ));
            }
        }
        body.push_str(
            "        if max_winding_v > 1e-3 {\n\
             \x20           self.diag_nr_max_iter_count += 1;\n\
             \x20           return;\n\
             \x20       }\n",
        );
    }

    body.push_str(
        "\n        // --- Converged: write back to state -----------------------\n\
         \x20       self.dc_operating_point = v_node;\n\
         \x20       self.v_prev = v_node;\n\
         \x20       self.input_prev = 0.0;\n",
    );
    if nonlinear && ir.topology.m > 0 {
        body.push_str(
            "        self.i_nl_prev = i_nl_final;\n\
             \x20       self.i_nl_prev_prev = i_nl_final;\n",
        );
    }

    // DC blocker: seed x_prev with the converged DC output (not 0.0) so the
    // next sample's IIR evaluation `raw_out - x_prev + r*y_prev` gives 0,
    // i.e. the filter starts already at steady state. y_prev stays 0 because
    // the fixed point of `y = x - x + r*y` is `y*(1-r) = 0 ⇒ y = 0`.
    if ir.dc_block && !ir.solver_config.output_nodes.is_empty() {
        body.push_str(
            "        // Seed DC blocker at its steady state (y=0, x=V_dc_out) so it\n\
             \x20       // doesn't generate a step transient on the first sample.\n",
        );
        for (k, &node) in ir.solver_config.output_nodes.iter().enumerate() {
            body.push_str(&format!(
                "        self.dc_block_x_prev[{k}] = v_node[{node}];\n",
            ));
        }
        body.push_str("        self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n");
    }

    // Pot prev sync — the per-sample A_neg correction uses the previous
    // timestep's conductance. After recompute we've jumped to a new
    // equilibrium at the current pot value, so `_prev` must track it too.
    if !ir.pots.is_empty() {
        body.push_str(
            "        // Sync pot_N_resistance_prev so the first sample's A_neg\n\
             \x20       // correction doesn't fire on a phantom conductance delta.\n",
        );
        for idx in 0..ir.pots.len() {
            body.push_str(&format!(
                "        self.pot_{idx}_resistance_prev = self.pot_{idx}_resistance;\n",
            ));
        }
    }

    // Oversampler filter state — mid-stream these taps carry the old DC
    // trajectory. Zero them so the new DC doesn't appear as a step that the
    // half-band IIR has to ring down. Sizes are compile-time derived from
    // the oversampling factor (matching `state.rs.tera`'s literal sizing).
    if ir.solver_config.oversampling_factor > 1 {
        let os = oversampling_info(ir.solver_config.oversampling_factor);
        body.push_str(&format!(
            "        // Reset oversampler half-band filter state (stale DC trajectory).\n\
             \x20       self.os_up_state = [0.0; {size}];\n\
             \x20       self.os_dn_state = [[0.0; {size}]; NUM_OUTPUTS];\n",
            size = os.state_size,
        ));
        if ir.solver_config.oversampling_factor == 4 {
            body.push_str(&format!(
                "        self.os_up_state_outer = [0.0; {size}];\n\
                 \x20       self.os_dn_state_outer = [[0.0; {size}]; NUM_OUTPUTS];\n",
                size = os.state_size_outer,
            ));
        }
    }

    // Linear-companion-model history: inductors, coupled inductors, and
    // transformer winding currents. The DK MVP's DC equilibrium is the
    // `process_sample(0.0)` fixed point where the companion shunt keeps
    // `V_L = 0` and therefore `I_L = 0` — which is exactly the zeroed state.
    if !ir.inductors.is_empty() {
        let n = ir.inductors.len();
        body.push_str(&format!(
            "        self.ind_i_prev = [0.0; {n}];\n\
             \x20       self.ind_v_prev = [0.0; {n}];\n\
             \x20       self.ind_i_hist = [0.0; {n}];\n",
        ));
    }
    if !ir.coupled_inductors.is_empty() {
        let n = ir.coupled_inductors.len();
        body.push_str(&format!(
            "        self.ci_i1_prev = [0.0; {n}];\n\
             \x20       self.ci_i2_prev = [0.0; {n}];\n\
             \x20       self.ci_v1_prev = [0.0; {n}];\n\
             \x20       self.ci_v2_prev = [0.0; {n}];\n\
             \x20       self.ci_i1_hist = [0.0; {n}];\n\
             \x20       self.ci_i2_hist = [0.0; {n}];\n",
        ));
    }
    for (idx, g) in ir.transformer_groups.iter().enumerate() {
        let nw = g.num_windings;
        body.push_str(&format!(
            "        self.xfmr_{idx}_i_prev = [0.0; {nw}];\n\
             \x20       self.xfmr_{idx}_v_prev = [0.0; {nw}];\n\
             \x20       self.xfmr_{idx}_i_hist = [0.0; {nw}];\n",
        ));
    }

    body
}

/// Emit the body of `CircuitState::recompute_dc_op()` for a nodal full-LU
/// circuit.
///
/// The nodal solver operates on the augmented MNA directly (N = n_aug rather
/// than the DK path's N = n_nodes + augmented_rows), and its NR device
/// evaluator uses `emit_nodal_device_evaluation_body/_final` — a different
/// stamping scheme than `emit_dk_device_evaluation`. A correct runtime
/// recompute would therefore need a nodal-specific parallel of
/// `emit_dc_op_build_g_aug_dk` / `_build_b_dc_dk` / `_nr_loop_dk`.
///
/// That nodal body is **deferred indefinitely**. Shipping nodal-routed
/// plugins (pultec, 4kbuscomp, VCR Audio ALC, wurli power amp) continue
/// using the `WARMUP_SAMPLES_RECOMMENDED` silence loop, which is the
/// documented path for nodal circuits — not a temporary workaround. The
/// warmup loop runs the full per-sample NR and is guaranteed to converge
/// to the physically correct DC OP.
///
/// This stub keeps the `recompute_dc_op` method surface uniform across DK
/// and nodal circuits (so plugin host code doesn't need a solver-path
/// branch). It bumps the NR max-iter diagnostic counter and returns
/// without touching `v_prev` or `dc_operating_point`, matching the DK
/// path's NR-failure branch — callers' fallback logic handles both the
/// DK-convergence-failure and the nodal-stub-no-op cases uniformly.
///
/// See the `phase_e_handoff_runtime_dc_op` memory + `docs/aidocs/DC_OP.md`
/// "Runtime DC OP recompute" for the reviver's technical plan if
/// priorities ever flip.
pub(super) fn emit_recompute_dc_op_body_nodal(_ir: &CircuitIR) -> Result<String, CodegenError> {
    Ok(
        "        // Nodal full-LU path: runtime DC OP recompute is deferred\n\
        \x20       // indefinitely (warmup loop is the documented path for nodal\n\
        \x20       // circuits). Bump the NR max-iter diag counter so callers'\n\
        \x20       // standard fallback pattern (`if counter ticked, run warmup`)\n\
        \x20       // routes through the `WARMUP_SAMPLES_RECOMMENDED` silence loop.\n\
        \x20       //\n\
        \x20       // See `docs/aidocs/DC_OP.md` \"Runtime DC OP recompute\" for\n\
        \x20       // the rationale and the reviver's plan if priorities change.\n\
        \x20       self.diag_nr_max_iter_count += 1;\n"
            .to_string(),
    )
}

/// Emit the body of `CircuitState::settle_dc_op()` — the
/// recompute-then-fallback-on-failure wrapper both DK and nodal paths
/// share.
///
/// Pattern:
///
/// 1. Snapshot `diag_nr_max_iter_count`.
/// 2. Call `recompute_dc_op()`.
/// 3. If the counter advanced (DK NR divergence / singular matrix / NaN
///    reset / max-iter exhaustion, OR nodal permanent stub), fall back to
///    `WARMUP_SAMPLES_RECOMMENDED` iterations of `process_sample(0.0, self)`.
///
/// Why check only `diag_nr_max_iter_count`: damping and substep counters
/// can fire during successful DK NR convergence. Singular-matrix and
/// NaN-reset paths break out of the NR loop without setting
/// `nr_converged`, which routes the return through the max-iter-exhaustion
/// branch that increments this specific counter. The nodal stub also
/// bumps exactly this counter. So this one field is the unambiguous
/// "did recompute_dc_op fail to update state?" signal.
///
/// The body is identical for DK and nodal paths — the path-specific
/// behavior lives entirely in `recompute_dc_op` itself.
///
/// Emitted as a `pub fn settle_dc_op(&mut self)` on `CircuitState`, gated
/// behind the same `emit_dc_op_recompute` flag as `recompute_dc_op`.
pub(super) fn emit_settle_dc_op_body() -> String {
    "        // Settle to the DC operating point at the current pot / switch /\n\
     \x20       // device values. Tries the fast runtime NR first; on failure\n\
     \x20       // (DK-path NR divergence or nodal-path permanent stub) runs the\n\
     \x20       // `WARMUP_SAMPLES_RECOMMENDED` silence loop as fallback.\n\
     \x20       //\n\
     \x20       // The counter-check uses `diag_nr_max_iter_count` exclusively —\n\
     \x20       // damping and substep counters can fire during successful\n\
     \x20       // convergence, so they're not failure signals. The DK recompute\n\
     \x20       // increments this counter on every \"state not updated\" outcome\n\
     \x20       // (NR iter exhaustion, singular LU on either the linear or NR\n\
     \x20       // path, NaN resets, DC-biased winding rejection), and the\n\
     \x20       // nodal stub increments it unconditionally — so this single\n\
     \x20       // field cleanly distinguishes \"ready\" from \"fall back\".\n\
     \x20       let before = self.diag_nr_max_iter_count;\n\
     \x20       self.recompute_dc_op();\n\
     \x20       if self.diag_nr_max_iter_count > before {\n\
     \x20           for _ in 0..WARMUP_SAMPLES_RECOMMENDED {\n\
     \x20               let _ = process_sample(0.0, self);\n\
     \x20           }\n\
     \x20       }\n"
        .to_string()
}
