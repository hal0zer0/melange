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
//! ## Layering (shipped across E.1 → E.5)
//!
//! 1. `emit_recompute_dc_op_body_dk` — top-level body assembler.
//! 2. `emit_dc_op_build_g_aug_dk` — G_aug base construction from live
//!    pot/switch state (E.3).
//! 3. `emit_dc_op_build_b_dc_dk` — DC RHS from `RHS_CONST` (halved on
//!    node rows) + `.runtime` voltage source fields (E.5).
//! 4. `emit_dc_op_extract_v_nl_dk` + `emit_dk_device_evaluation` —
//!    per-device i_nl + Jacobian evaluator shared with transient NR (E.4).
//! 5. `emit_dc_op_nr_loop_dk` — Direct NR loop with LU solve, flat
//!    damping, convergence check (E.5).
//! 6. `emit_dc_op_writeback_dk` — write converged `v_node` + `i_nl` back
//!    to state, clear DC-blocker history (E.5).
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
//! with `b_dc` built from `RHS_CONST` by halving rows `[0..n_nodes)`. This
//! converges to the exact compile-time DC OP for inductor-free circuits —
//! inductor-bearing circuits converge to the `process_sample(0.0)` steady
//! state instead (the `g_eq = T/(2L)` companion shunt already baked into
//! `G` differs from the inductor-short augmented row the compile-time
//! solver uses). That distinction is documented in the method docstring.

use crate::codegen::ir::CircuitIR;
use crate::codegen::CodegenError;

use super::helpers::fmt_f64;
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
/// loop: a single `invert_n(g_aug)` solve gives the resistive fixed point.
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
         \x20       //     `G` already carries the trapezoidal companion shunt\n\
         \x20       //     `g_eq = T/(2L)`, so the fixed point solved here matches\n\
         \x20       //     `process_sample(0.0, ...)` steady state, not the\n\
         \x20       //     inductor-short DC OP that `dc_op.rs` would compute.\n\
         \x20       //     For inductor-free circuits these agree bitwise.\n\
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

/// Emit the DC RHS vector `b_dc: [f64; N]` for `recompute_dc_op`.
///
/// Maps `RHS_CONST` (the per-sample trapezoidal Norton current vector) to the
/// DC steady-state RHS:
///   * Node rows `[0..n_nodes)` are halved (`RHS_CONST` doubles current-source
///     injections for trapezoidal averaging — at DC that averaging collapses
///     back to the single DC source value).
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

    body.push_str(
        "\n        // Build b_dc: DC steady-state RHS from the per-sample RHS_CONST.\n\
         \x20       // Node rows are halved (trapezoidal averaging collapses at DC);\n\
         \x20       // VS/VCVS/ideal-xfmr algebraic rows and inductor branch rows are\n\
         \x20       // preserved verbatim. See module-level DC fixed-point derivation.\n",
    );
    if ir.has_dc_sources {
        body.push_str("        let mut b_dc: [f64; N] = RHS_CONST;\n");
        if n_nodes > 0 {
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
/// linear, so a single `invert_n` + matrix-vector multiply gives the fixed
/// point. On singular `g_aug`, state is left unchanged and the diag counter
/// is bumped.
fn emit_dc_op_linear_solve_dk(_ir: &CircuitIR) -> String {
    let mut body = String::new();
    body.push_str(
        "\n        // Linear circuit (M == 0): solve g_aug · v = b_dc once.\n\
         \x20       let (g_inv, singular) = invert_n(g_aug);\n\
         \x20       if singular {\n\
         \x20           self.diag_singular_matrix_count += 1;\n\
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
///   5. LU-solves `G_aug_nr · v_new = rhs_nr` via the emitted `invert_n`
///      helper (returns `(inv, singular)`).
///   6. Applies flat global voltage damping when any element of the step
///      exceeds the damping threshold, plus a per-element clamp, and
///      advances `v_node`.
///   7. Declares convergence when the damped step falls below `TOL`.
///
/// On convergence `v_node` and the final `i_nl` are written back via
/// `emit_dc_op_writeback_dk`; on max-iter exhaustion `diag_nr_max_iter_count`
/// is bumped and state is left untouched so the caller can fall back to the
/// warmup loop.
fn emit_dc_op_nr_loop_dk(ir: &CircuitIR) -> Result<String, CodegenError> {
    const MAX_ITER: usize = 200;
    const TOL: f64 = 1e-9;
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
         \x20       for _iter in 0..MAX_ITER {{\n"
    ));

    let inner = "            ";

    // v_nl extraction — locals `v_d{i}` live inside the loop body.
    out.push_str(&format!(
        "{inner}// Extract controlling voltages v_nl = N_v · v_node.\n"
    ));
    out.push_str(&emit_dc_op_extract_v_nl_dk(ir, inner));
    out.push('\n');

    // Device evaluation — shared helper emits `state.device_*` reads, so we
    // need a `state: &CircuitState` local inside the loop body. Immutable
    // borrow is enough because device evaluation never mutates self.
    out.push_str(&format!(
        "{inner}// Reuse the transient per-device evaluator (shared helper).\n\
         {inner}let state: &CircuitState = &*self;\n"
    ));
    emit_dk_device_evaluation(&mut out, ir, inner)?;

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
                out.push_str(&format!(
                    "{inner}j_dev[{r}][{c}] = jdev_{r}_{c};\n"
                ));
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
        "{inner}// Solve G_aug_nr · v_new = rhs_nr via the shared `invert_n`.\n\
         {inner}let (g_inv, singular) = invert_n(g_aug_nr);\n\
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
    // `DAMP_THRESHOLD / max_delta` when max exceeds threshold).
    out.push_str(&format!(
        "{inner}// Flat damping + per-element clamp on the Newton step.\n\
         {inner}let mut max_delta = 0.0_f64;\n\
         {inner}for a in 0..N {{\n\
         {inner}    let d = (v_new[a] - v_node[a]).abs();\n\
         {inner}    if d > max_delta {{ max_delta = d; }}\n\
         {inner}}}\n\
         {inner}let damping = if max_delta > {DAMP_THRESHOLD:.1}_f64 {{\n\
         {inner}    ({DAMP_THRESHOLD:.1}_f64 / max_delta).max(0.1)\n\
         {inner}}} else {{ 1.0 }};\n\
         {inner}if damping < 1.0 {{ self.diag_voltage_damp_count += 1; }}\n\
         {inner}let mut step_max = 0.0_f64;\n\
         {inner}for a in 0..N {{\n\
         {inner}    let delta = (v_new[a] - v_node[a]) * damping;\n\
         {inner}    let limited = delta.clamp(-{CLAMP:.1}_f64, {CLAMP:.1}_f64);\n\
         {inner}    v_node[a] += limited;\n\
         {inner}    let la = limited.abs();\n\
         {inner}    if la > step_max {{ step_max = la; }}\n\
         {inner}}}\n\
         {inner}i_nl_final = i_nl;\n\
         {inner}if step_max < TOL {{\n\
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
/// Also clears the DC-blocking filter's per-output sample history so it
/// doesn't take seconds to re-settle from a stale output offset. Does NOT
/// touch input history, RNG state, oversampler taps, or inductor history —
/// those either remain physically correct or aren't in scope for the MVP.
fn emit_dc_op_writeback_dk(ir: &CircuitIR, nonlinear: bool) -> String {
    let mut body = String::new();
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
    if ir.dc_block {
        body.push_str(
            "        // Clear DC blocker history — the new DC offset replaces the old\n\
             \x20       // one, and the IIR high-pass would otherwise take ~5/fc seconds\n\
             \x20       // to re-converge.\n\
             \x20       self.dc_block_x_prev = [0.0; NUM_OUTPUTS];\n\
             \x20       self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n",
        );
    }
    body
}
