//! Runtime DC operating-point recompute emission.
//!
//! Emits `CircuitState::recompute_dc_op()` into generated plugin code so that
//! plugins with per-instance pot/switch jitter can re-solve the DC bias at
//! construction time, skipping the `WARMUP_SAMPLES_RECOMMENDED` silence loop.
//!
//! ## Scope
//!
//! **Direct Newton-Raphson only**, starting from `self.v_prev` (which is
//! a physically valid state from the prior solve). No source/Gmin stepping,
//! no basin-trap handling. Circuits that only converge via stepping in the
//! codegen-time solver remain reliant on the warmup loop.
//!
//! Device coverage: diode, BJT (EM + GP), JFET, MOSFET, triode, pentode,
//! linear op-amp, VCA — i.e. whatever the compile-time DC OP handles today,
//! minus multi-equilibrium refinements.
//!
//! ## Layering
//!
//! 1. `emit_recompute_dc_op_body_dk` — top-level body assembler (DK path).
//! 2. `emit_dc_op_build_g_aug_dk` — G_aug base construction from live
//!    pot/switch state.
//! 3. `emit_dc_op_build_b_dc_dk` — DC RHS = `RHS_CONST` +
//!    `.runtime` voltage source fields.
//! 4. `emit_dc_op_extract_v_nl_dk` + `emit_dk_device_evaluation` —
//!    per-device i_nl + Jacobian evaluator shared with transient NR.
//! 5. `emit_dc_op_nr_loop_dk` — Direct NR loop with LU solve, flat
//!    damping, and the two-part certification: the step test on the
//!    undamped Newton correction AND the per-row KCL residual gate at the
//!    point written back (both at `dc_op.rs` tolerances).
//! 6. `emit_dc_op_writeback_dk` — write converged `v_node` + `i_nl` back
//!    to state (the live point and its own currents, `dc_op_i_nl`, as a
//!    pair), zero `q_dot`, seed the DC blocker at the new DC output,
//!    sync pot `_prev`, zero oversampler history.
//! 7. `emit_dc_op_state_methods_dk` — the always-emitted `dc_kcl_residual`,
//!    `dc_op_writeback` and full `set_dc_operating_point` methods.
//!
//! The nodal route's body (`recompute_dc_op` on the augmented MNA with the
//! transient's own device evaluator, limiter and live matrices) and its
//! state methods live in `nodal_emitter/dc_op.rs`; `emit_settle_dc_op_body`
//! below is shared by both routes.
//!
//! ## DC fixed-point algebra
//!
//! The per-sample equation (charge form, both integrators) is
//!
//! ```text
//!     A · v_{n+1} = RHS_CONST + A_neg · v_n + q_dot_n + input(n+1)
//! ```
//!
//! and the NR loop in `process_sample` adds `N_i · i_nl(v_{n+1})` to both
//! sides. At steady state `v_{n+1} = v_n = v_dc`, `input = 0`, and the
//! charge derivative `q_dot` is zero. With `A − A_neg = G` on every row
//! (`A_neg = alpha·C`), that gives the DC fixed point
//!
//! ```text
//!     G · v_dc = RHS_CONST + N_i · i_nl_dc
//! ```
//!
//! which one Newton step at a time is the compile-time `dc_op.rs` form
//!
//! ```text
//!     G_aug_nr = g_aug − N_i · J_dev · N_v
//!     rhs_nr   = b_dc  + N_i · (i_nl − J_dev · v_nl)
//!     v_new    = G_aug_nr⁻¹ · rhs_nr
//! ```
//!
//! with `b_dc = RHS_CONST`.
//!
//! Inductors, coupled inductors and transformer windings are augmented
//! branch rows whose `L` sits in `C`. In `G` each such row is the
//! short-circuit constraint `v_a − v_b = 0`, the same DC short the
//! compile-time solver stamps (`dc_op.rs`), so the fixed point above holds
//! with inductors as well.

use crate::codegen::ir::CircuitIR;
use crate::codegen::CodegenError;

use super::helpers::{carries_q_dot, fmt_f64, oversampling_info};
use super::nr_helpers::emit_dk_device_evaluation;

/// Emit the body of `CircuitState::recompute_dc_op()` for a DK-path circuit.
///
/// Assembles the full Direct-NR runtime DC OP solve:
///   1. Build the resistive `g_aug` base matrix from `G` + live pot/switch
///      state.
///   2. Build the DC RHS `b_dc` from `RHS_CONST` (verbatim) and any
///      `.runtime` voltage source fields.
///   3. Direct NR loop: warm-start from `self.v_prev`, extract `v_nl`,
///      evaluate per-device currents + Jacobian, LU-solve for the Newton
///      step, apply flat voltage damping + clamp, check convergence.
///   4. On convergence, write the solution back to `self.dc_operating_point`,
///      `self.v_prev`, `self.i_nl_prev` / `i_nl_prev_prev`, and seed the
///      DC blocker at the new DC output so it doesn't need seconds to
///      re-settle from a stale offset.
///
/// Linear-only circuits (`M == 0` or no device slots) short-circuit the NR
/// loop: a single `invert_n_equilibrated(g_aug)` solve gives the resistive fixed point.
pub(super) fn emit_recompute_dc_op_body_dk(ir: &CircuitIR) -> Result<String, CodegenError> {
    let mut body = String::new();
    body.push_str(
        "        // Runtime DC operating-point recompute: direct Newton-Raphson.\n\
         \x20       //\n\
         \x20       // Warm-starts from `self.v_prev` (a physically valid prior\n\
         \x20       // equilibrium), rebuilds `g_aug` from live pot/switch state,\n\
         \x20       // and iterates classic companion-linearized NR until an iterate\n\
         \x20       // passes both the undamped-correction step test and the KCL\n\
         \x20       // residual gate (the compile-time solver's tolerances). On\n\
         \x20       // convergence the result is written back; on failure, state is\n\
         \x20       // left untouched and `diag_nr_max_iter_count` is bumped so the\n\
         \x20       // caller can fall back to the `WARMUP_SAMPLES_RECOMMENDED`\n\
         \x20       // silence loop.\n\
         \x20       //\n\
         \x20       // Scope notes (documented in the method's public docstring):\n\
         \x20       //   * Parasitic-BJT internal nodes (RB/RC/RE > 0) are NOT\n\
         \x20       //     expanded on the DK path — see `docs/aidocs/DC_OP.md`.\n\
         \x20       //   * Inductors are augmented branch rows: in `G` each is the\n\
         \x20       //     DC short `v_a − v_b = 0` (its `L` sits in `C`).\n\
         \x20       //   * No source / Gmin stepping, no basin-trap handling.\n",
    );
    body.push_str(&emit_dc_op_build_g_aug_dk(ir));
    body.push_str(&emit_dc_op_build_b_dc_dk(ir));
    let (vt_snapshot, vt_restore) = emit_body_effect_vt_snapshot_dk(ir);
    body.push_str(&vt_snapshot);

    let linear = ir.topology.m == 0 || ir.device_slots.is_empty();
    let pinned: Vec<&crate::codegen::ir::OpampIR> = ir
        .opamps
        .iter()
        .filter(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
        .collect();
    let hard = ir.solver_config.opamp_rail_mode == crate::codegen::OpampRailMode::Hard;
    if !hard || pinned.is_empty() {
        if linear {
            // Linear circuit: single LU solve. No NR loop needed.
            body.push_str(&emit_dc_op_linear_solve_dk(ir, "self.v_prev"));
        } else {
            body.push_str(&emit_dc_op_nr_loop_dk(ir, "self.v_prev", &vt_restore)?);
        }
        body.push_str(&emit_dc_op_writeback_dk(ir, !linear));
        return Ok(body);
    }

    // Hard rail mode: a railed op-amp output sits at its zero-load limit at
    // the terminal, as the transient clamps it and as the compile-time DC OP
    // (`dc_op::pin_railed_opamps`, `DcRail::Terminal`) places it. An active
    // set: pin every output whose linear model passes its limit
    // (`AOL*(v+ - v-)`), re-solve with those rows fixed, repeat until the set
    // is unchanged. A set that does not settle is a failed recompute
    // (`diag_nr_max_iter_count`), so `settle_dc_op` falls back to warmup.
    const MAX_RAIL_ROUNDS: usize = 8;
    let k = pinned.len();
    // Newton warm-starts each round from the last; a linear solve needs none.
    let (seed_decl, seed_advance) = if linear {
        ("", "")
    } else {
        (
            "        let mut v_seed: [f64; N] = self.v_prev;\n",
            "        v_seed = v_node;\n",
        )
    };
    body.push_str(&format!(
        "\n        // --- Railed op-amp outputs: terminal pin (hard rail mode) ---\n\
         \x20       let g_aug_base = g_aug;\n\
         \x20       let b_dc_base = b_dc;\n\
         \x20       let mut rail_pins: [Option<f64>; {k}] = [None; {k}];\n\
         {seed_decl}\
         \x20       for _rail_round in 0..={MAX_RAIL_ROUNDS} {{\n\
         \x20       let mut g_aug = g_aug_base;\n\
         \x20       let mut b_dc = b_dc_base;\n"
    ));
    for (idx, oa) in pinned.iter().enumerate() {
        let o = oa.n_out_idx;
        body.push_str(&format!(
            "        if let Some(c) = rail_pins[{idx}] {{\n\
             \x20           for j in 0..N {{ g_aug[{o}][j] = 0.0; }}\n\
             \x20           g_aug[{o}][{o}] = 1.0;\n\
             \x20           b_dc[{o}] = c;\n\
             \x20       }}\n"
        ));
    }
    if linear {
        body.push_str(&emit_dc_op_linear_solve_dk(ir, "v_seed"));
    } else {
        body.push_str(&emit_dc_op_nr_loop_dk(ir, "v_seed", &vt_restore)?);
    }
    // A held pin stays only while its own test still passes and otherwise
    // releases; it never moves to the other rail in one round (pinned, the
    // loop is open and `v+ - v-` points at the opposite rail). The same rule
    // as the compile-time DC OP (`dc_op::hold_or_release`).
    body.push_str(&format!("        let next_pins: [Option<f64>; {k}] = [\n"));
    for (idx, oa) in pinned.iter().enumerate() {
        let aol = oa.gm / oa.g_out;
        let vp = oa
            .n_plus_idx
            .map_or("0.0".to_string(), |i| format!("v_node[{i}]"));
        let vm = oa
            .n_minus_idx
            .map_or("0.0".to_string(), |i| format!("v_node[{i}]"));
        let mut expr = format!("{{ let u = {aol:.17e} * ({vp} - {vm}); let fresh = ");
        if oa.vclamp_hi.is_finite() {
            expr.push_str(&format!(
                "if u >= {hi:.17e} {{ Some({hi:.17e}) }} else ",
                hi = oa.vclamp_hi
            ));
        }
        if oa.vclamp_lo.is_finite() {
            expr.push_str(&format!(
                "if u <= {lo:.17e} {{ Some({lo:.17e}) }} else ",
                lo = oa.vclamp_lo
            ));
        }
        expr.push_str(&format!(
            "{{ None }}; match rail_pins[{idx}] {{ Some(h) => (fresh == Some(h)).then_some(h), None => fresh }} }}"
        ));
        body.push_str(&format!("            {expr},\n"));
    }
    body.push_str(
        "        ];\n\
         \x20       if next_pins == rail_pins {\n",
    );
    body.push_str(&emit_dc_op_writeback_dk(ir, !linear));
    body.push_str(&format!(
        "        return;\n\
         \x20       }}\n\
         \x20       rail_pins = next_pins;\n\
         {seed_advance}\
         \x20       }}\n\
         \x20       // The railed op-amp pin set did not settle: a failed recompute.\n\
         {vt_restore}\
         \x20       self.diag_nr_max_iter_count += 1;\n",
    ));
    Ok(body)
}

/// The body-effect MOSFET threshold fields the Newton loop writes, and how to
/// put them back.
///
/// The shared device evaluator reads a MOSFET's threshold from
/// `state.device_{d}_vt`, so the loop refreshes that field at every iterate
/// (as the transient does per sample). The field is the only channel into the
/// evaluator, so it cannot be iterated on a local; instead its value before
/// the solve is kept here and written back on every failure return, which
/// keeps the method's contract that a failed recompute leaves state untouched.
/// On success the field holds the threshold at the converged point, which is
/// the writeback. Returns `(snapshot, restore)`: the `let` bindings to emit
/// before the solve and the assignments to emit before each failure return;
/// both empty without body-effect MOSFETs.
fn emit_body_effect_vt_snapshot_dk(ir: &CircuitIR) -> (String, String) {
    let mut snapshot = String::new();
    let mut restore = String::new();
    for (_k, dev_num, _slot, _mp) in super::helpers::body_effect_mosfets(ir) {
        if snapshot.is_empty() {
            snapshot.push_str(
                "\n        // Body-effect thresholds before the solve: the device evaluator\n\
                 \x20       // reads `device_{d}_vt`, so the Newton loop writes it at every\n\
                 \x20       // iterate; a failed recompute puts these back.\n",
            );
        }
        snapshot.push_str(&format!(
            "        let vt_saved_{dev_num} = self.device_{dev_num}_vt;\n"
        ));
        restore.push_str(&format!(
            "        self.device_{dev_num}_vt = vt_saved_{dev_num};\n"
        ));
    }
    (snapshot, restore)
}

/// Emit Rust that constructs a local `g_aug: [[f64; N]; N]` from:
///   * the const `G` (input conductance, VS/VCVS rows, opamp VCCS, linearized
///     triodes, inductor branch rows — all already baked at codegen time),
///   * per-pot conductance deltas `1/self.pot_i_resistance - POT_i_G_NOM`,
///   * per-switch resistor deltas `1/R_new - 1/R_nominal` (C/L switch
///     components do not affect `g_aug`: capacitors are open at DC, and an
///     inductor's value sits in `C` while its `G` branch row is the DC short
///     `v_a − v_b = 0` — see the module docstring).
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
         \x20       // linearized-triode stamps, and inductor branch rows (DC shorts)\n\
         \x20       // are already included).\n\
         \x20       let mut g_aug: [[f64; N]; N] = G;\n",
    );

    // --- Switch resistor deltas --------------------------------------------
    //
    // At DC only R-type switch components affect `g_aug`. Capacitors are open
    // (no G contribution) and inductors are shorts, already in G as their
    // augmented branch rows (see module-level note).
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
/// `b_dc = RHS_CONST`, verbatim, under both integrators: the charge form
/// stamps each DC source once, at n+1, so the per-sample constant already
/// is the DC right-hand side (`A − A_neg = G` on every row). Inductor
/// branch rows are zero in `RHS_CONST` (short-circuit constraints), VS rows
/// carry `V_dc`. `.runtime` voltage sources add `self.<field>` to their VS
/// row so `recompute_dc_op` converges to the bias point the host will drive
/// on the first real sample.
fn emit_dc_op_build_b_dc_dk(ir: &CircuitIR) -> String {
    let mut body = String::new();
    body.push_str(
        "\n        // Build b_dc: the DC steady-state RHS is the per-sample RHS_CONST\n\
         \x20       // (charge form: every source enters once). See the module-level\n\
         \x20       // DC fixed-point derivation.\n",
    );
    if ir.has_dc_sources {
        body.push_str("        let mut b_dc: [f64; N] = RHS_CONST;\n");
    } else {
        body.push_str("        let mut b_dc: [f64; N] = [0.0; N];\n");
    }

    if !ir.runtime_sources.is_empty() {
        body.push_str(
            "\n        // `.runtime` voltage sources: include their current value so\n\
             \x20       // recompute converges to the equilibrium the host will drive. A\n\
             \x20       // non-finite value is counted and reads as 0, as in `process_sample`.\n",
        );
        for rt in &ir.runtime_sources {
            body.push_str(&format!(
                "        b_dc[{row}] += if self.{field}.is_finite() {{ self.{field} }} else {{ self.diag_runtime_nan_count += 1; 0.0 }};\n",
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
fn emit_dc_op_linear_solve_dk(_ir: &CircuitIR, _seed: &str) -> String {
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
         \x20       }\n\
         \x20       // No device currents, and no Newton iteration to count.\n\
         \x20       let i_nl_final: [f64; M] = [0.0; M];\n\
         \x20       let nr_iters: u32 = 0;\n",
    );
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
///   6. Tests the UNDAMPED Newton correction `v_new − v_node` per variable,
///      `|Δ| < RELTOL·max(|v_node|, |v_new|) + TOL`: Newton's own estimate of
///      the distance to the root, taken before any limiter touches the step
///      (the rule the nodal transient certifies with, at `dc_op.rs`
///      tolerances). The applied step is what moves the iterate; its size is
///      not evidence.
///   7. Applies flat global voltage damping when any node-voltage element
///      of the step exceeds the damping threshold, plus a per-element
///      clamp, and advances `v_node`. Damping/clamp sweep VOLTAGE rows
///      only (`[0..n_nodes)`); VS/VCVS branch-current rows (amperes) are
///      applied undamped, mirroring `dc_op.rs`'s `is_voltage_row`
///      classification.
///   8. At the top of the next iteration, with the devices freshly evaluated
///      at the point the sub-threshold correction reached, gates acceptance
///      on the per-row KCL residual `|F_i| <= RELTOL·scale_i + KCL_ABSTOL`
///      over the voltage rows, `F = g_aug·v − b_dc − N_i·i_nl(N_v·v)` and
///      `scale_i = max(|b_i|, |(N_i·i_nl)_i|, max_j |g_aug_ij·v_j|)`, as
///      `dc_op.rs` computes it. The step test alone is satisfied at any fixed
///      point of the limit/damp map, root or not; the gate alone is vacuous
///      on a weak mode (at 1e-9 A a node tied to the circuit by nanosiemens
///      passes anywhere within a fraction of a volt). Convergence needs both,
///      and both are read at the point that is written back.
///
/// On convergence `v_node` and the device currents evaluated AT it are
/// written back via `emit_dc_op_writeback_dk`; on iteration exhaustion (or a
/// singular / non-finite solve) `diag_nr_max_iter_count` is bumped, the
/// body-effect thresholds are put back (`vt_restore`) and the method returns
/// with every other field untouched so the caller can fall back to the warmup
/// loop.
fn emit_dc_op_nr_loop_dk(
    ir: &CircuitIR,
    seed: &str,
    vt_restore: &str,
) -> Result<String, CodegenError> {
    const MAX_ITER: usize = 200;
    const TOL: f64 = 1e-9;
    /// Relative convergence term, mirroring `dc_op.rs::DcOpConfig::reltol`
    /// (SPICE-style `|Δ| < reltol·|v| + abstol`). Keeps 300 V circuits from
    /// false-failing at the LU round-off floor, where an absolute-only 1e-9
    /// check can never be satisfied.
    const RELTOL: f64 = 1e-6;
    /// Absolute floor of the KCL residual gate, in amperes: the compile-time
    /// solver's `DC_OP_KCL_ABSTOL_AMPS` (`dc_op.rs`). A separate constant from
    /// the volts step floor `TOL`; the two happen to share a value.
    const KCL_ABSTOL: f64 = 1e-9;

    let m = ir.topology.m;
    let mut out = String::new();

    out.push_str(&format!(
        "\n        // --- Direct Newton-Raphson DC OP loop -----------------------\n\
         \x20       let mut v_node: [f64; N] = {seed};\n\
         \x20       let mut i_nl_final: [f64; M] = [0.0; M];\n\
         \x20       let mut nr_converged = false;\n\
         \x20       // The 0-indexed iteration that converged (`last_nr_iterations`).\n\
         \x20       let mut nr_iters: u32 = 0;\n\
         \x20       // True when the previous iteration's UNDAMPED Newton correction\n\
         \x20       // passed the step test, so the point this iteration evaluates\n\
         \x20       // the devices at is a candidate for the KCL gate.\n\
         \x20       let mut step_ok = false;\n\
         \x20       const MAX_ITER: usize = {MAX_ITER};\n\
         \x20       const TOL: f64 = {TOL:e};\n\
         \x20       const RELTOL: f64 = {RELTOL:e};\n\
         \x20       const KCL_ABSTOL: f64 = {KCL_ABSTOL:e};\n\
         \x20       for iter in 0..MAX_ITER {{\n"
    ));

    let inner = "            ";
    out.push_str(&emit_dc_op_device_eval_dk(ir, inner)?);

    // v_nl as a dense array for the J·v_nl product below.
    out.push_str(&format!("{inner}let v_nl: [f64; M] = ["));
    for i in 0..m {
        if i > 0 {
            out.push_str(", ");
        }
        out.push_str(&format!("v_d{i}"));
    }
    out.push_str("];\n\n");

    emit_dc_op_nr_loop_dk_tail(ir, &mut out, inner, vt_restore);
    Ok(out)
}

/// Emit, at `indent`, the device evaluation of the DK runtime DC solve at the
/// node vector `v_node` (a local in scope): the controlling voltages `v_d{i}`,
/// the body-effect threshold refresh, the shared transient device evaluator
/// behind a `state: &CircuitState` alias, and the dense `i_nl: [f64; M]` /
/// `j_dev: [[f64; M]; M]` packs of its flat locals. Shared by the Newton loop
/// (every iterate) and `set_dc_operating_point` (once, at the given vector).
fn emit_dc_op_device_eval_dk(ir: &CircuitIR, inner: &str) -> Result<String, CodegenError> {
    let m = ir.topology.m;
    let mut out = String::new();

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
                 {inner}// per NR iteration). The field is the evaluator's only input\n\
                 {inner}// for it; a failed recompute restores the pre-solve value.\n"
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
    Ok(out)
}

/// The rest of the DK Newton iteration after the device evaluation and the
/// dense `v_nl`: the KCL gate, the Newton system and solve, the guards, the
/// step test, the damped step, and the failure return after the loop.
fn emit_dc_op_nr_loop_dk_tail(ir: &CircuitIR, out: &mut String, inner: &str, vt_restore: &str) {
    const DAMP_THRESHOLD: f64 = 10.0;
    const CLAMP: f64 = 50.0;
    let n_v_rows = ir.topology.n_nodes;

    // KCL residual gate at the current iterate, reached by a sub-threshold
    // Newton correction. Devices were just evaluated at this exact v_node, so
    // the residual costs one matrix-vector product and no extra evaluation.
    // Voltage rows only ([0..n_nodes); DK expands no internal nodes): the
    // VS/VCVS branch rows are KVL rows the LU solve satisfies exactly.
    out.push_str(&format!(
        "{inner}// KCL residual gate at this iterate (devices evaluated here), when\n\
         {inner}// the previous iteration's undamped correction passed the step test.\n\
         {inner}// F_i = (g_aug·v)_i − b_dc_i − (N_i·i_nl)_i over the voltage rows,\n\
         {inner}// accepted when |F_i| <= RELTOL·scale_i + KCL_ABSTOL with scale_i =\n\
         {inner}// max(|b_dc_i|, |(N_i·i_nl)_i|, max_j |g_aug_ij·v_j|) (dc_op.rs's gate).\n\
         {inner}// Written `!(x <= tol)` so a NaN residual fails.\n\
         {inner}if step_ok {{\n\
         {inner}    let mut kcl_ok = true;\n\
         {inner}    for a in 0..{n_v_rows} {{\n\
         {inner}        let mut q = 0.0_f64;\n\
         {inner}        for i in 0..M {{ q += N_I[a][i] * i_nl[i]; }}\n\
         {inner}        let mut acc = -b_dc[a] - q;\n\
         {inner}        let mut scale = b_dc[a].abs().max(q.abs());\n\
         {inner}        for b in 0..N {{\n\
         {inner}            let t = g_aug[a][b] * v_node[b];\n\
         {inner}            acc += t;\n\
         {inner}            let ta = t.abs(); if ta > scale {{ scale = ta; }}\n\
         {inner}        }}\n\
         {inner}        if !(acc.abs() <= RELTOL * scale + KCL_ABSTOL) {{ kcl_ok = false; }}\n\
         {inner}    }}\n\
         {inner}    if kcl_ok {{\n\
         {inner}        // Converged: this point passed both tests, and i_nl is its own.\n\
         {inner}        i_nl_final = i_nl;\n\
         {inner}        nr_converged = true;\n\
         {inner}        nr_iters = iter as u32;\n\
         {inner}        break;\n\
         {inner}    }}\n\
         {inner}}}\n\n"
    ));

    // Build G_aug_nr = g_aug − N_i · J_dev · N_v.
    // We use the N_I / N_V consts directly (dense scan). At typical M ≤ 6
    // the inner loop is trivially small; optimizer unrolls it. `N_I` is
    // stored in operator shape [N][M], so `N_i[a][i] = N_I[a][i]`.
    out.push_str(&format!(
        "{inner}// Apply companion linearization: g_aug_nr = g_aug − N_i · J · N_v.\n\
         {inner}let mut g_aug_nr = g_aug;\n\
         {inner}for a in 0..N {{\n\
         {inner}    for b in 0..N {{\n\
         {inner}        let mut corr = 0.0;\n\
         {inner}        for i in 0..M {{\n\
         {inner}            let ni_ai = N_I[a][i];\n\
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
         {inner}        rhs_nr[a] += N_I[a][i] * i_comp;\n\
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

    // Guard against NaN/Inf and finite-but-implausible magnitude — a
    // saturated solve can produce a huge-but-finite value that bypasses
    // is_finite() (see STATE_MAX_PLAUSIBLE_MAGNITUDE doc in
    // constants.rs.tera). Bail either way; `nr_converged` stays false so
    // the max-iter fallback below returns WITHOUT writing state back.
    out.push_str(&format!(
        "{inner}let v_new_finite = v_new.iter().all(|x| x.is_finite());\n\
         {inner}if !v_new_finite || v_new.iter().any(|x| x.abs() > STATE_MAX_PLAUSIBLE_MAGNITUDE) {{\n\
         {inner}    if v_new_finite {{ self.diag_magnitude_reset_count += 1; }} else {{ self.diag_nan_reset_count += 1; }}\n\
         {inner}    break;\n\
         {inner}}}\n\n"
    ));

    // Step test on the UNDAMPED correction, then the damped, clamped step.
    //
    // The test reads `v_new − v_node`, Newton's own estimate of the distance
    // to the root, per variable against `RELTOL·max(|v_node|, |v_new|) +
    // TOL` (the threshold at the correction's target, the nodal transient's
    // rule; `dc_op.rs` tolerances). The test used to read the applied step
    // after damping and clamping: a limiter's small step is not evidence of
    // convergence, and the transient measured points certified that way
    // 1.7 to 810 thresholds from Newton's target. A sub-threshold correction
    // is also below every limiter's threshold, so the point it reaches is
    // `v_new` itself; the next iteration evaluates the devices there and
    // gates it on the KCL residual (above). Convergence needs both tests.
    //
    // Global flat damping + per-element clamp. Matches the simple-form
    // damping used in `dc_op.rs::nr_dc_solve` (scale all deltas by
    // `DAMP_THRESHOLD / max_delta` when max exceeds threshold), including
    // its voltage-row classification: only rows [0..n_nodes) are node
    // voltages on the DK path (internal parasitic nodes are not expanded
    // here); rows [n_nodes..N) are VS/VCVS branch currents in AMPERES —
    // sweeping them into the voltage damping scan or the ±50 "V" clamp
    // would stall convergence whenever a branch current legitimately
    // exceeds the thresholds.
    out.push_str(&format!(
        "{inner}// Step test on the UNDAMPED Newton correction, per variable:\n\
         {inner}// |v_new − v| < RELTOL·max(|v|, |v_new|) + TOL. Read before any\n\
         {inner}// limiter: the applied step's size is not evidence (see dc_op.rs\n\
         {inner}// tolerances; the nodal transient's rule). NaN-honest (`!(a < b)`).\n\
         {inner}step_ok = true;\n\
         {inner}for a in 0..N {{\n\
         {inner}    let delta = v_new[a] - v_node[a];\n\
         {inner}    let threshold = RELTOL * v_node[a].abs().max(v_new[a].abs()) + TOL;\n\
         {inner}    if !(delta.abs() < threshold) {{ step_ok = false; }}\n\
         {inner}}}\n\
         {inner}// Flat damping + per-element clamp on the Newton step.\n\
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
         {inner}for a in 0..N {{\n\
         {inner}    let delta = v_new[a] - v_node[a];\n\
         {inner}    let limited = if a < {n_v_rows} {{\n\
         {inner}        (delta * damping).clamp(-{CLAMP:.1}_f64, {CLAMP:.1}_f64)\n\
         {inner}    }} else {{\n\
         {inner}        delta\n\
         {inner}    }};\n\
         {inner}    v_node[a] += limited;\n\
         {inner}}}\n\
         \x20       }}\n\n"
    ));

    // Failure path: iteration exhaustion, a singular solve or a non-finite
    // iterate all arrive here with `nr_converged` false. State is untouched
    // but for the body-effect thresholds, which are put back.
    let vt_restore_nested = vt_restore.replace("        self.", "            self.");
    out.push_str(&format!(
        "        if !nr_converged {{\n\
         {vt_restore_nested}\
         \x20           self.diag_nr_max_iter_count += 1;\n\
         \x20           return;\n\
         \x20       }}\n"
    ));
}

/// The recompute's writeback: one call of the always-emitted
/// `dc_op_writeback` method (see [`emit_dc_op_state_methods_dk`]) with the
/// converged `v_node`, the device currents evaluated at it and the
/// converging iteration.
fn emit_dc_op_writeback_dk(_ir: &CircuitIR, _nonlinear: bool) -> String {
    "\n        // --- Converged: write back to state -----------------------\n\
     \x20       self.dc_op_writeback(v_node, i_nl_final, nr_iters);\n"
        .to_string()
}

/// The DC RHS the residual reads, without the NaN counting of
/// [`emit_dc_op_build_b_dc_dk`] (a `&self` method cannot count): a non-finite
/// `.runtime` field reads as 0, as it does in `process_sample`.
fn emit_dc_op_build_b_dc_dk_readonly(ir: &CircuitIR) -> String {
    let mut body = String::new();
    if ir.has_dc_sources {
        body.push_str("        let mut b_dc: [f64; N] = RHS_CONST;\n");
    } else {
        body.push_str("        let mut b_dc: [f64; N] = [0.0; N];\n");
    }
    for rt in &ir.runtime_sources {
        body.push_str(&format!(
            "        b_dc[{row}] += if self.{field}.is_finite() {{ self.{field} }} else {{ 0.0 }};\n",
            row = rt.vs_row,
            field = rt.field_name,
        ));
    }
    if ir.runtime_sources.is_empty() {
        // Nothing stamps into it: keep the binding immutable.
        body = body.replace("let mut b_dc", "let b_dc");
    }
    body
}

/// Emit the three `CircuitState` methods every DK build carries, whatever
/// the recompute flag: `dc_kcl_residual`, `dc_op_writeback` and the full
/// `set_dc_operating_point`. Rendered into `state.rs.tera` as
/// `dc_op_state_methods` (4-space method indent).
///
/// `set_dc_operating_point(v_dc)` is the writeback `recompute_dc_op` does,
/// with the device currents from one evaluation at `v_dc`, and it returns
/// the KCL residual of `v_dc` against the live circuit: the largest
/// `|F_i|` over the circuit-node rows in amperes and that row's index. It
/// reports a non-equilibrium vector; it never refuses one.
///
/// `dc_op_writeback(v_node, i_nl_final, nr_iters)` writes the live operating
/// point and its own currents as a pair (`dc_operating_point`, `dc_op_i_nl`),
/// seeds `v_prev` / `i_nl_prev` / `i_nl_prev_prev` from them, puts the charge
/// derivative `q_dot` at rest, zeroes the input history, seeds the DC blocker
/// at its fixed point (`x_prev = V_dc_out`, `y_prev = 0`, so the first sample
/// sees no step), syncs `pot_N_resistance_prev`, zeroes the oversampler taps
/// (mid-stream they carry the old DC level and would ring down as the output
/// steps to the new one) and records `last_nr_iterations`. Deliberately
/// preserved: the noise RNG (resetting it would repeat the same noise after
/// every parameter change), pots and switches (the solve's inputs), runtime
/// sources, device runtime parameters and junction temperatures, and the
/// diagnostic counters (a caller may be watching them).
pub(super) fn emit_dc_op_state_methods_dk(ir: &CircuitIR) -> Result<String, CodegenError> {
    let m = ir.topology.m;
    let n_v_rows = ir.topology.n_nodes;
    let multi_input = ir.solver_config.num_inputs() > 1;
    let linear = m == 0 || ir.device_slots.is_empty();
    let mut code = String::new();

    // --- dc_kcl_residual ---------------------------------------------------
    code.push_str(
        "    /// KCL residual of the node vector `v` with the device currents `i_nl`\n\
         \x20   /// against the live circuit (current pots and switches): the largest\n\
         \x20   /// `|F_i|`, `F = G·v − b_dc − N_i·i_nl`, over the circuit-node rows in\n\
         \x20   /// amperes, and that row's index. The VS/VCVS branch rows are KVL rows\n\
         \x20   /// in volts and are not part of it. `set_dc_operating_point` returns\n\
         \x20   /// it; the runtime recompute gates on the same per-row residual.\n\
         \x20   fn dc_kcl_residual(&self, v: &[f64; N], i_nl: &[f64; M]) -> (f64, usize) {\n",
    );
    code.push_str(&emit_dc_op_build_g_aug_dk(ir));
    code.push_str(&emit_dc_op_build_b_dc_dk_readonly(ir));
    code.push_str(&format!(
        "        let mut worst = 0.0_f64;\n\
         \x20       let mut worst_row = 0usize;\n\
         \x20       for a in 0..{n_v_rows} {{\n\
         \x20           let mut q = 0.0_f64;\n\
         \x20           for i in 0..M {{ q += N_I[a][i] * i_nl[i]; }}\n\
         \x20           let mut acc = -b_dc[a] - q;\n\
         \x20           for b in 0..N {{ acc += g_aug[a][b] * v[b]; }}\n\
         \x20           // NaN-honest: a non-finite residual wins the worst slot.\n\
         \x20           if !(acc.abs() <= worst) {{ worst = acc.abs(); worst_row = a; }}\n\
         \x20       }}\n\
         \x20       (worst, worst_row)\n\
         \x20   }}\n\n"
    ));

    // --- dc_op_writeback ---------------------------------------------------
    code.push_str(
        "    /// Install `v_node` as the live DC operating point with its own device\n\
         \x20   /// currents `i_nl_final` (the pair the NaN/magnitude recovery restarts\n\
         \x20   /// from), and seed every history the first sample reads: `v_prev`,\n\
         \x20   /// `i_nl_prev` / `i_nl_prev_prev`, the charge derivative at rest, zero\n\
         \x20   /// input history, the DC blocker at its fixed point, pot `_prev` fields\n\
         \x20   /// in step, oversampler taps zeroed, `last_nr_iterations`. Preserved:\n\
         \x20   /// the noise RNG, pots and switches, runtime sources, device runtime\n\
         \x20   /// parameters and junction temperatures, the diagnostic counters.\n\
         \x20   fn dc_op_writeback(&mut self, v_node: [f64; N], i_nl_final: [f64; M], nr_iters: u32) {\n\
         \x20       self.dc_operating_point = v_node;\n\
         \x20       // The live point's own currents, kept beside it for the NaN/\n\
         \x20       // magnitude recovery (which restarts from the pair).\n\
         \x20       self.dc_op_i_nl = i_nl_final;\n\
         \x20       self.v_prev = v_node;\n\
         \x20       self.i_nl_prev = i_nl_final;\n\
         \x20       self.i_nl_prev_prev = i_nl_final;\n",
    );
    if carries_q_dot(ir) {
        code.push_str("        self.q_dot = [0.0; N];\n");
    }
    if multi_input {
        code.push_str("        self.inputs_prev = [0.0; NUM_INPUTS];\n");
    } else {
        code.push_str("        self.input_prev = 0.0;\n");
    }
    if ir.solver_config.has_inject_or_tap() {
        code.push_str("        self.injections_prev = [0.0; NUM_INJECT];\n");
    }
    code.push_str("        self.last_nr_iterations = nr_iters;\n");

    // DC blocker: seed x_prev with the DC output (not 0.0) so the next
    // sample's IIR evaluation `raw_out - x_prev + r*y_prev` gives 0, i.e. the
    // filter starts already at steady state. y_prev stays 0 because the fixed
    // point of `y = x - x + r*y` is `y*(1-r) = 0 ⇒ y = 0`.
    if ir.dc_block && !ir.solver_config.output_nodes.is_empty() {
        code.push_str(
            "        // Seed DC blocker at its steady state (y=0, x=V_dc_out) so it\n\
             \x20       // doesn't generate a step transient on the first sample.\n",
        );
        for (k, &node) in ir.solver_config.output_nodes.iter().enumerate() {
            code.push_str(&format!(
                "        self.dc_block_x_prev[{k}] = v_node[{node}];\n",
            ));
        }
        code.push_str("        self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n");
    }

    // Pot prev sync — every committed sample leaves `_prev` equal to the
    // current resistance; the writeback does the same.
    if !ir.pots.is_empty() {
        code.push_str(
            "        // Sync pot_N_resistance_prev with the current resistance, as\n\
             \x20       // every committed sample does.\n",
        );
        for idx in 0..ir.pots.len() {
            code.push_str(&format!(
                "        self.pot_{idx}_resistance_prev = self.pot_{idx}_resistance;\n",
            ));
        }
    }

    // Oversampler filter state — mid-stream these taps carry the old DC
    // trajectory. Zero them so the new DC doesn't appear as a step that the
    // half-band IIR has to ring down. Sizes are compile-time derived from
    // the oversampling factor (matching `state.rs.tera`'s literal sizing).
    if super::runtime_os::runtime(ir).is_some() {
        code.push_str(
            "        // Reset oversampler half-band filter state (stale DC trajectory).\n\
             \x20       self.reset_oversampler();\n",
        );
    } else if ir.solver_config.oversampling_factor > 1 {
        let os = oversampling_info(ir.solver_config.oversampling_factor);
        code.push_str(&format!(
            "        // Reset oversampler half-band filter state (stale DC trajectory).\n\
             \x20       self.os_up_state = [0.0; {size}];\n\
             \x20       self.os_dn_state = [[0.0; {size}]; NUM_OUTPUTS];\n",
            size = os.state_size,
        ));
        if ir.solver_config.oversampling_factor == 4 {
            code.push_str(&format!(
                "        self.os_up_state_outer = [0.0; {size}];\n\
                 \x20       self.os_dn_state_outer = [[0.0; {size}]; NUM_OUTPUTS];\n",
                size = os.state_size_outer,
            ));
        }
        code.push_str(&super::inject_tap::emit_inject_os_state_reset(
            ir, "self", "        ",
        ));
    }
    // The Step-6c containment threshold follows the live point.
    code.push_str("        self.refresh_damp_thresh();\n    }\n\n");

    // --- set_dc_operating_point --------------------------------------------
    code.push_str(
        "    /// Install `v_dc` as the DC operating point: the full writeback\n\
         \x20   /// `recompute_dc_op` does (`dc_op_writeback`), with the device currents\n\
         \x20   /// from one evaluation at `v_dc`.\n\
         \x20   ///\n\
         \x20   /// Returns the KCL residual of `v_dc` against the live circuit: the\n\
         \x20   /// largest `|F_i|` over the circuit-node rows, in amperes, and that\n\
         \x20   /// row. A vector that is an equilibrium at the current controls reads\n\
         \x20   /// ~1e-9 A or less; a node-voltages-only vector of a deck with a\n\
         \x20   /// DC-carrying winding (its branch-current rows left at zero) shows the\n\
         \x20   /// winding's current as the residual of the node it feeds. The vector\n\
         \x20   /// is installed either way; the residual is the caller's to read.\n\
         \x20   ///\n\
         \x20   /// For instantiation, `reset()`, preset recall and per-instance jitter,\n\
         \x20   /// where no audio is running; see `recompute_dc_op` for the usage note.\n\
         \x20   pub fn set_dc_operating_point(&mut self, v_dc: [f64; N]) -> (f64, usize) {\n\
         \x20       let v_node = v_dc;\n",
    );
    if linear {
        code.push_str("        let i_nl_final: [f64; M] = [0.0; M];\n");
    } else {
        code.push_str("        let i_nl_final: [f64; M] = {\n");
        code.push_str(&emit_dc_op_device_eval_dk(ir, "            ")?);
        // The Jacobian is evaluated with the currents and unused here.
        code.push_str("            let _ = j_dev;\n            i_nl\n        };\n");
    }
    code.push_str(
        "        let (residual, worst_row) = self.dc_kcl_residual(&v_node, &i_nl_final);\n\
         \x20       self.dc_op_writeback(v_node, i_nl_final, 0);\n\
         \x20       (residual, worst_row)\n\
         \x20   }\n",
    );
    Ok(code)
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
pub(super) fn emit_settle_dc_op_body(ir: &crate::codegen::ir::CircuitIR) -> String {
    let mut s = "        // Settle to the DC operating point at the current pot / switch /\n\
     \x20       // device values. Tries the fast runtime NR first; on failure\n\
     \x20       // (DK-path NR divergence or nodal-path permanent stub) runs the\n\
     \x20       // `WARMUP_SAMPLES_RECOMMENDED` silence loop as fallback.\n\
     \x20       //\n\
     \x20       // The counter-check uses `diag_nr_max_iter_count` exclusively —\n\
     \x20       // damping and substep counters can fire during successful\n\
     \x20       // convergence, so they're not failure signals. The DK recompute\n\
     \x20       // increments this counter on every \"state not updated\" outcome\n\
     \x20       // (NR iter exhaustion, singular LU on either the linear or NR\n\
     \x20       // path, NaN resets, an unsettled railed op-amp pin set), and the\n\
     \x20       // nodal stub increments it unconditionally — so this single\n\
     \x20       // field cleanly distinguishes \"ready\" from \"fall back\".\n\
     \x20       let before = self.diag_nr_max_iter_count;\n\
     \x20       self.recompute_dc_op();\n\
     \x20       if self.diag_nr_max_iter_count > before {\n\
     \x20           for _ in 0..WARMUP_SAMPLES_RECOMMENDED {\n"
        .to_string();
    // Warmup call matches the public signature (adds the all-zero injections
    // array + tuple return for `.inject`/`.tap` decks).
    s.push_str(&super::dk_emitter::emit_warmup_call(
        ir,
        "                ",
        true,
    ));
    s.push_str("            }\n        }\n");
    s
}
