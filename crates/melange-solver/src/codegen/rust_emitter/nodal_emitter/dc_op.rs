//! The nodal route's DC operating-point methods: the runtime solve
//! (`recompute_dc_op`, behind `--emit-dc-op-recompute`), the always-emitted
//! `set_dc_operating_point` with its KCL residual, and the writeback both
//! share.
//!
//! The solve is the compile-time solver's equation on the live circuit,
//! `G·v = b_dc + N_i·i_nl(N_v·v)`, with `G` the matrix the transient runs
//! (the working copy the pot/switch setters stamp, carrying the baked node
//! Gmin) and `b_dc = RHS_CONST` plus the `.runtime` voltage sources (the charge
//! form stamps every DC source once, so the per-sample constant is the DC
//! right-hand side on every row; see `dc_op_emitter.rs`). Every row of the
//! augmented system is an unknown: node voltages, source and constraint rows,
//! inductor and transformer winding currents (their rows are the DC short
//! `v_a − v_b = 0`, their currents entries of the solution).
//!
//! Newton on it uses the transient's own ingredients: the device evaluator
//! and chord stamp of the per-sample loop, the dense equilibrated `lu_solve`
//! (this is not the hot path, so no sparse schedule), the transient's limiter
//! (pnjlim/fetlim shrinking one ray `alpha` on the proposed device voltages,
//! then the node-step damping), and the transient's certification rule: the
//! step test on the UNDAMPED Newton correction and the per-row KCL residual
//! gate, both at the compile-time solver's tolerances, both at the point
//! written back.

use super::residual::emit_sparse_nv_matvec;
use super::sites::{has_be_instance, live_g_c};
use super::stamps::{
    emit_body_gmb_companion, emit_body_gmb_stamp, emit_nodal_companion_rhs,
    emit_nodal_jacobian_stamp,
};
use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::helpers::{
    body_effect_mosfets, carries_q_dot, has_latched_device, kcl_rows, oversampling_info,
};
use crate::codegen::rust_emitter::inject_tap::emit_inject_os_state_reset;
use crate::codegen::rust_emitter::RustEmitter;
use crate::codegen::OpampRailMode;

/// Newton budget of the runtime DC solve: the compile-time solver's
/// `DcOpConfig::max_iterations`.
const DC_OP_MAX_ITER: usize = 200;
/// Absolute floor of the step test, in volts: `DcOpConfig::tolerance`.
const DC_OP_TOL: f64 = 1e-9;
/// Relative term of the step test and of the KCL gate: `DcOpConfig::reltol`.
const DC_OP_RELTOL: f64 = 1e-6;
/// Absolute floor of the KCL residual gate, in amperes: the compile-time
/// solver's `DC_OP_KCL_ABSTOL_AMPS`. A separate constant from the volts
/// floor; the two happen to share a value.
const DC_OP_KCL_ABSTOL: f64 = 1e-9;

/// Why a build's `recompute_dc_op` refuses (bumps `diag_nr_max_iter_count` and
/// returns, so `settle_dc_op` falls back to the warmup loop), in the order the
/// method documents them. Empty when the build solves.
pub(super) fn recompute_refusals(ir: &CircuitIR) -> Vec<&'static str> {
    let mut why = Vec::new();
    // Classed on `idt()` / `time`: an integrator whose integrand is nonzero at
    // rest advances every sample, so such a deck has no DC operating point and
    // no warmup settles it either. Static and ddt-only expressions have a
    // fixed point (ddt is exactly 0 there), but no deck needs that branch
    // yet, and building it means seeding `bsrc_x_prev` at the converged
    // inner values and landing the same inclusion in the compile-time solver
    // so the two points agree. Until then every behavioral deck refuses; a
    // later inclusion widens this to the idt/time class, it does not reverse it.
    if !ir.behavioral_sources.is_empty() {
        why.push("behavioral B-source");
    }
    // A railed op-amp is never solved through its linear model (a comparator
    // railed at rest lands at AOL·vd, thousands of volts). The transient's
    // active set is not carried by this first cut, so a deck whose rail mode
    // clamps a finite rail refuses. BoyleDiodes rails are catch diodes, i.e.
    // devices the solve sees; `none` applies no rail, so the linear model is
    // the transient's own fixed point.
    let clamps = matches!(
        ir.solver_config.opamp_rail_mode,
        OpampRailMode::Hard | OpampRailMode::ActiveSet | OpampRailMode::ActiveSetBe
    );
    if clamps
        && ir
            .opamps
            .iter()
            .any(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
    {
        why.push("clampable op-amp");
    }
    // A latched device's lit/unlit state is part of the equation set: a glow
    // deck can have two DC roots or none (a relaxation divider).
    if has_latched_device(ir) {
        why.push("latched (glow) device");
    }
    why
}

/// The Rust iterator over `rows` for emitted loops: `0..K` when the rows are
/// exactly the first `K` indices, the explicit list otherwise.
fn rows_iter(rows: &[usize]) -> String {
    if rows.iter().enumerate().all(|(k, &r)| k == r) {
        format!("0..{}", rows.len())
    } else {
        format!(
            "[{}]",
            rows.iter()
                .map(|r| r.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        )
    }
}

/// The body-effect threshold fields the solve writes, as `(snapshot, restore)`
/// text at `indent` with `recv` as the receiver: the transient's device
/// evaluator refreshes `device_{d}_vt` at every iterate through the field, so
/// a failed recompute puts the pre-solve values back (the method's contract
/// is that a failure leaves state untouched). Both empty without
/// body-effect MOSFETs.
fn body_effect_vt_snapshot(ir: &CircuitIR, recv: &str, indent: &str) -> (String, String) {
    let mut snapshot = String::new();
    let mut restore = String::new();
    for (_k, dev_num, _slot, _mp) in body_effect_mosfets(ir) {
        if snapshot.is_empty() {
            snapshot.push_str(&format!(
                "{indent}// Body-effect thresholds before the solve: the device evaluator\n\
                 {indent}// refreshes `device_{{d}}_vt` at every iterate through the field;\n\
                 {indent}// a failed recompute puts these back.\n"
            ));
        }
        snapshot.push_str(&format!(
            "{indent}let vt_saved_{dev_num} = {recv}.device_{dev_num}_vt;\n"
        ));
        restore.push_str(&format!(
            "{indent}{recv}.device_{dev_num}_vt = vt_saved_{dev_num};\n"
        ));
    }
    (snapshot, restore)
}

/// `b_dc` at `indent`: `RHS_CONST` (or zeros) plus the `.runtime` voltage
/// sources read through `recv`. `count_nan` emits the `process_sample` rule
/// for a non-finite field (counted in `diag_runtime_nan_count`, read as 0);
/// a `&self` method cannot count and reads it as 0 silently.
fn emit_b_dc(ir: &CircuitIR, recv: &str, indent: &str, count_nan: bool) -> String {
    let mut code = String::new();
    let mutable = if ir.runtime_sources.is_empty() {
        ""
    } else {
        "mut "
    };
    if ir.has_dc_sources {
        code.push_str(&format!(
            "{indent}let {mutable}b_dc: [f64; N] = RHS_CONST;\n"
        ));
    } else {
        code.push_str(&format!(
            "{indent}let {mutable}b_dc: [f64; N] = [0.0; N];\n"
        ));
    }
    for rt in &ir.runtime_sources {
        let field = &rt.field_name;
        let row = rt.vs_row;
        if count_nan {
            code.push_str(&format!(
                "{indent}b_dc[{row}] += if {recv}.{field}.is_finite() {{ {recv}.{field} }} else {{ {recv}.diag_runtime_nan_count += 1; 0.0 }};\n"
            ));
        } else {
            code.push_str(&format!(
                "{indent}b_dc[{row}] += if {recv}.{field}.is_finite() {{ {recv}.{field} }} else {{ 0.0 }};\n"
            ));
        }
    }
    code
}

impl RustEmitter {
    /// Emit the `CircuitState` methods every nodal build carries, whatever the
    /// recompute flag: `dc_kcl_residual`, `dc_op_writeback` and the full
    /// `set_dc_operating_point` (4-space method indent).
    pub(super) fn emit_nodal_dc_op_state_methods(ir: &CircuitIR, use_full_nodal: bool) -> String {
        let m = ir.topology.m;
        let multi_input = ir.solver_config.num_inputs() > 1;
        let (g_src, _c_src) = live_g_c(ir, use_full_nodal, "self");
        let rows = kcl_rows(ir);
        let mut code = String::new();

        // --- dc_kcl_residual -----------------------------------------------
        code.push_str(
            "    /// KCL residual of the node vector `v` with the device currents `i_nl`\n\
             \x20   /// against the live circuit (current pots and switches): the largest\n\
             \x20   /// `|F_i|`, `F = G·v − b_dc − N_i·i_nl`, over the circuit-node rows\n\
             \x20   /// (and a parasitic BJT's internal nodes) in amperes, and that row's\n\
             \x20   /// index. Source, constraint and winding rows are KVL rows in volts\n\
             \x20   /// and are not part of it. `set_dc_operating_point` returns it; the\n\
             \x20   /// runtime recompute gates on the same per-row residual.\n\
             \x20   fn dc_kcl_residual(&self, v: &[f64; N], i_nl: &[f64; M]) -> (f64, usize) {\n",
        );
        code.push_str(&emit_b_dc(ir, "self", "        ", false));
        code.push_str(&format!(
            "        let mut worst = 0.0_f64;\n\
             \x20       let mut worst_row = 0usize;\n\
             \x20       for a in {rows_it} {{\n\
             \x20           let mut q = 0.0_f64;\n\
             \x20           for i in 0..M {{ q += N_I[a][i] * i_nl[i]; }}\n\
             \x20           let mut acc = -b_dc[a] - q;\n\
             \x20           for b in 0..N {{ acc += {g_src}[a][b] * v[b]; }}\n\
             \x20           // NaN-honest: a non-finite residual wins the worst slot.\n\
             \x20           if !(acc.abs() <= worst) {{ worst = acc.abs(); worst_row = a; }}\n\
             \x20       }}\n\
             \x20       (worst, worst_row)\n\
             \x20   }}\n\n",
            rows_it = rows_iter(&rows),
        ));

        // --- dc_op_writeback -----------------------------------------------
        code.push_str(
            "    /// Install `v` as the live DC operating point with its own device\n\
             \x20   /// currents `i_nl` (the pair the NaN/magnitude recovery restarts from),\n\
             \x20   /// and seed every history the first sample reads: `v_prev`, `i_nl_prev`\n\
             \x20   /// / `i_nl_prev_prev`, the charge derivative at rest, zero input and\n\
             \x20   /// injection history, the DC blocker at its fixed point, the chord\n\
             \x20   /// factorisations invalid, the BE latch and breakpoint cleared as\n\
             \x20   /// `reset()` clears them, pot `_prev` fields in step, oversampler taps\n\
             \x20   /// zeroed, `last_nr_iterations`. An inductor's or winding's current is\n\
             \x20   /// a row of `v` and its flux history `q_dot` is at rest with it, so\n\
             \x20   /// magnetics carry no separate state. Preserved: the noise RNG, pots\n\
             \x20   /// and switches, runtime sources, device runtime parameters and\n\
             \x20   /// junction temperatures, the diagnostic counters, the factory point\n\
             \x20   /// `reset()` returns to.\n\
             \x20   fn dc_op_writeback(&mut self, v: [f64; N], i_nl: [f64; M], nr_iters: u32) {\n\
             \x20       self.dc_operating_point = v;\n\
             \x20       self.dc_op_i_nl = i_nl;\n\
             \x20       self.v_prev = v;\n\
             \x20       self.i_nl_prev = i_nl;\n\
             \x20       self.i_nl_prev_prev = i_nl;\n",
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
        if ir.dc_block {
            code.push_str(
                "        // DC blocker at its fixed point (x_prev = V_dc_out, y_prev = 0): the\n\
                 \x20       // first sample sees no step through the differentiator.\n",
            );
            for (k, &node) in ir.solver_config.output_nodes.iter().enumerate() {
                code.push_str(&format!("        self.dc_block_x_prev[{k}] = v[{node}];\n"));
            }
            code.push_str("        self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n");
        }
        // The chord factorisations (declared with any Newton loop) were taken
        // at the old point.
        if m > 0 || !ir.saturating_inductors.is_empty() || !ir.behavioral_sources.is_empty() {
            code.push_str(
                "        // The chord factorisations were taken at the old point.\n\
                 \x20       self.chord_valid = false;\n",
            );
            if use_full_nodal && has_be_instance(ir) {
                code.push_str("        self.chord_be_valid = false;\n");
            }
        }
        if ir.solver_config.runtime_be_latch {
            code.push_str("        // BE-latch detector state, as reset() clears it.\n");
            for f in [
                "be_x_mean",
                "be_x_prev",
                "be_r1_num",
                "be_pow",
                "be_in_x_mean",
                "be_in_x_prev",
                "be_in_r1_num",
                "be_in_pow",
                "be_ref_in",
                "be_env",
                "be_ref",
            ] {
                code.push_str(&format!("        self.{f} = 0.0;\n"));
            }
            code.push_str("        self.be_latched = false;\n");
        }
        if ir.solver_config.breakpoint_be {
            code.push_str("        self.breakpoint_be = 0;\n");
        }
        if !ir.pots.is_empty() {
            code.push_str(
                "        // pot_N_resistance_prev in step with the current resistance, as\n\
                 \x20       // every committed sample leaves it.\n",
            );
            for idx in 0..ir.pots.len() {
                code.push_str(&format!(
                    "        self.pot_{idx}_resistance_prev = self.pot_{idx}_resistance;\n"
                ));
            }
        }
        // Oversampler taps: mid-stream they carry the old DC level and would
        // ring down as the output steps to the new one.
        if crate::codegen::rust_emitter::runtime_os::runtime(ir).is_some() {
            code.push_str("        self.reset_oversampler();\n");
        } else if ir.solver_config.oversampling_factor > 1 {
            let os = oversampling_info(ir.solver_config.oversampling_factor);
            code.push_str(&format!(
                "        self.os_up_state = [0.0; {size}];\n\
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
            code.push_str(&emit_inject_os_state_reset(ir, "self", "        "));
        }
        code.push_str("    }\n\n");

        // --- set_dc_operating_point ----------------------------------------
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
             \x20   pub fn set_dc_operating_point(&mut self, v_dc: [f64; N]) -> (f64, usize) {\n",
        );
        if m > 0 {
            code.push_str(
                "        let i_nl: [f64; M] = {\n\
                 \x20           let state: &CircuitState = &*self;\n\
                 \x20           let mut v_nl_final = [0.0f64; M];\n",
            );
            code.push_str(&emit_sparse_nv_matvec(
                ir,
                "v_nl_final",
                "v_dc",
                "            ",
            ));
            code.push_str("            let mut i_nl = [0.0f64; M];\n");
            Self::emit_nodal_device_evaluation_final(&mut code, ir, "            ", "v_dc");
            code.push_str("            i_nl\n        };\n");
        } else {
            code.push_str("        let i_nl: [f64; M] = [0.0; M];\n");
        }
        code.push_str(
            "        let (residual, worst_row) = self.dc_kcl_residual(&v_dc, &i_nl);\n\
             \x20       self.dc_op_writeback(v_dc, i_nl, 0);\n\
             \x20       (residual, worst_row)\n\
             \x20   }\n\n",
        );
        code
    }

    /// Emit the body of `CircuitState::recompute_dc_op()` for a nodal build
    /// (Schur and full-LU alike), 8-space indent. See the module doc for the
    /// equation and the ingredients; [`recompute_refusals`] for the builds
    /// that refuse instead.
    pub(super) fn emit_recompute_dc_op_body_nodal(ir: &CircuitIR, use_full_nodal: bool) -> String {
        let m = ir.topology.m;
        let n = ir.topology.n;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let rows = kcl_rows(ir);
        let mut code = String::new();

        let refusals = recompute_refusals(ir);
        if !refusals.is_empty() {
            code.push_str(&format!(
                "        // REFUSED on this build ({}): no runtime DC solve is emitted for\n\
                 \x20       // it (see the method doc for each class). The counter advances so\n\
                 \x20       // `settle_dc_op` falls back to the warmup loop; state is untouched.\n\
                 \x20       self.diag_nr_max_iter_count += 1;\n",
                refusals.join(", ")
            ));
            return code;
        }

        let (g_src, _c_src) = live_g_c(ir, use_full_nodal, "state");
        let needs_rebuild_state =
            !ir.pots.is_empty() || !ir.switches.is_empty() || !ir.saturating_inductors.is_empty();
        let (vt_snapshot, vt_restore) = body_effect_vt_snapshot(ir, "state", "        ");

        code.push_str(
            "        // Runtime DC operating-point solve on the augmented MNA: Newton on\n\
             \x20       // G·v = b_dc + N_i·i_nl(N_v·v) with the live G, from v_prev, with\n\
             \x20       // the transient's device evaluator and limiter. Certified by the\n\
             \x20       // undamped-correction step test AND the per-row KCL gate at the\n\
             \x20       // point written back (compile-time solver tolerances). On failure\n\
             \x20       // state is untouched and diag_nr_max_iter_count advances.\n\
             \x20       let state: &mut CircuitState = self;\n",
        );
        if needs_rebuild_state {
            code.push_str(
                "        // The setters stamp the working G at once and defer the rate-\n\
                 \x20       // dependent matrices; rebuild them now so the state this solve\n\
                 \x20       // leaves is current for its first sample.\n\
                 \x20       if state.matrices_dirty {\n\
                 \x20           state.rebuild_matrices(state.current_sample_rate * OVERSAMPLING_FACTOR as f64);\n\
                 \x20           state.matrices_dirty = false;\n\
                 \x20       }\n",
            );
        }
        code.push_str(&format!(
            "        // The live conductance matrix (the transient's own, baked node Gmin\n\
             \x20       // included) and the DC right-hand side.\n\
             \x20       let g_aug: [[f64; N]; N] = {g_src};\n"
        ));
        code.push_str(&emit_b_dc(ir, "state", "        ", true));
        code.push_str(&vt_snapshot);
        code.push_str(&format!(
            "        let mut v: [f64; N] = state.v_prev;\n\
             \x20       let {i_nl_mut}i_nl = [0.0f64; M];\n\
             \x20       let mut converged = false;\n\
             \x20       // The 0-indexed iteration that converged (`last_nr_iterations`).\n\
             \x20       let mut nr_iters: u32 = 0;\n\
             \x20       // True when the previous iteration's UNDAMPED Newton correction passed\n\
             \x20       // the step test, so the point the devices are evaluated at this\n\
             \x20       // iteration is a candidate for the KCL gate.\n\
             \x20       let mut step_ok = false;\n\
             \x20       const DC_OP_MAX_ITER: usize = {DC_OP_MAX_ITER};\n\
             \x20       const DC_OP_TOL: f64 = {DC_OP_TOL:e};\n\
             \x20       const DC_OP_RELTOL: f64 = {DC_OP_RELTOL:e};\n\
             \x20       const DC_OP_KCL_ABSTOL: f64 = {DC_OP_KCL_ABSTOL:e};\n\
             \x20       for iter in 0..DC_OP_MAX_ITER {{\n",
            i_nl_mut = if m > 0 { "mut " } else { "" },
        ));
        let it = "            ";
        if m > 0 {
            code.push_str(&format!(
                "{it}// Device currents and Jacobian at this iterate (the transient's evaluator).\n\
                 {it}let mut v_nl = [0.0f64; M];\n"
            ));
            code.push_str(&emit_sparse_nv_matvec(ir, "v_nl", "v", it));
            code.push_str(&format!("{it}let mut j_dev = [0.0f64; M * M];\n"));
            Self::emit_nodal_device_evaluation_body(&mut code, ir, it, "v");
            code.push('\n');
        }

        // KCL residual gate at this iterate, which a sub-threshold Newton
        // correction reached; the currents were just evaluated here.
        code.push_str(&format!(
            "{it}// KCL residual gate at this iterate (reached by a sub-threshold Newton\n\
             {it}// correction; the devices were just evaluated here): over the circuit-\n\
             {it}// node rows, |F_i| <= DC_OP_RELTOL·scale_i + DC_OP_KCL_ABSTOL with\n\
             {it}// F = g_aug·v − b_dc − N_i·i_nl and scale_i = max(|b_dc_i|, |(N_i·i_nl)_i|,\n\
             {it}// max_j |g_aug_ij·v_j|), the compile-time solver's gate. Written\n\
             {it}// `!(x <= tol)` so a NaN residual fails. The step test alone is met at\n\
             {it}// any fixed point of the limiter, root or not; the gate alone is vacuous\n\
             {it}// on a weak mode (a node tied to the circuit by nanosiemens passes at\n\
             {it}// 1e-9 A anywhere within a fraction of a volt). Both are required.\n\
             {it}if step_ok {{\n\
             {it}    let mut kcl_ok = true;\n\
             {it}    for a in {rows_it} {{\n\
             {it}        let mut q = 0.0_f64;\n\
             {it}        for i in 0..M {{ q += N_I[a][i] * i_nl[i]; }}\n\
             {it}        let mut acc = -b_dc[a] - q;\n\
             {it}        let mut scale = b_dc[a].abs().max(q.abs());\n\
             {it}        for b in 0..N {{\n\
             {it}            let t = g_aug[a][b] * v[b];\n\
             {it}            acc += t;\n\
             {it}            let ta = t.abs(); if ta > scale {{ scale = ta; }}\n\
             {it}        }}\n\
             {it}        if !(acc.abs() <= DC_OP_RELTOL * scale + DC_OP_KCL_ABSTOL) {{ kcl_ok = false; }}\n\
             {it}    }}\n\
             {it}    if kcl_ok {{\n\
             {it}        converged = true;\n\
             {it}        nr_iters = iter as u32;\n\
             {it}        break;\n\
             {it}    }}\n\
             {it}}}\n\n",
            rows_it = rows_iter(&rows),
        ));

        // The Newton system: the chord stamp into a dense copy of the live G,
        // the companion current into the DC right-hand side.
        code.push_str(&format!(
            "{it}// Newton system: G_nr = g_aug − N_i·J·N_v, rhs = b_dc + N_i·(i_nl − J·v_nl)\n\
             {it}// (the per-sample loop's stamps, on the DC matrices).\n\
             {it}let mut g_nr = g_aug;\n\
             {it}let mut rhs = b_dc;\n"
        ));
        if m > 0 {
            emit_nodal_jacobian_stamp(&mut code, ir, m, "g_nr", it);
            emit_nodal_companion_rhs(&mut code, ir, m, "rhs", "j_dev", it);
            emit_body_gmb_stamp(&mut code, ir, "g_nr", "body_gmb", it);
            emit_body_gmb_companion(&mut code, ir, "rhs", "body_gmb", "v", it);
        }
        // Solve. Dense, equilibrated, with one refinement step: the full-LU
        // route always emits `lu_solve`; the Schur route emits it for M > 0
        // and otherwise has `invert_n`.
        if use_full_nodal || m > 0 {
            code.push_str(&format!(
                "{it}let mut v_new = rhs;\n\
                 {it}// A singular Newton matrix ends the solve unconverged.\n\
                 {it}if !lu_solve(&mut g_nr, &mut v_new) {{ break; }}\n"
            ));
        } else {
            code.push_str(&format!(
                "{it}let mut v_new = [0.0f64; N];\n\
                 {it}// A singular matrix ends the solve unconverged.\n\
                 {it}let Some(g_inv) = invert_n(&g_nr) else {{ break; }};\n\
                 {it}for i in 0..N {{ let mut s = 0.0; for j in 0..N {{ s += g_inv[i][j] * rhs[j]; }} v_new[i] = s; }}\n"
            ));
        }
        // Guards: a non-finite or implausible iterate ends the solve, counted
        // as the per-sample recovery counts it.
        code.push_str(&format!(
            "{it}let v_new_finite = v_new.iter().all(|x| x.is_finite());\n\
             {it}if !v_new_finite || v_new.iter().any(|x| x.abs() > STATE_MAX_PLAUSIBLE_MAGNITUDE) {{\n\
             {it}    if v_new_finite {{ state.diag_magnitude_reset_count += 1; }} else {{ state.diag_nan_reset_count += 1; }}\n\
             {it}    break;\n\
             {it}}}\n\n"
        ));
        // Step test on the undamped correction, every row.
        code.push_str(&format!(
            "{it}// Step test on the UNDAMPED Newton correction, per variable over every\n\
             {it}// row: |v_new − v| < DC_OP_RELTOL·max(|v|, |v_new|) + DC_OP_TOL, read\n\
             {it}// before any limiter touches the step (Newton's own estimate of the\n\
             {it}// distance to the root; the applied step's size is not evidence). A\n\
             {it}// sub-threshold correction is under every limiter's threshold, so the\n\
             {it}// point it reaches is v_new itself, gated on the next iteration.\n\
             {it}step_ok = true;\n\
             {it}for a in 0..N {{\n\
             {it}    let delta = v_new[a] - v[a];\n\
             {it}    let threshold = DC_OP_RELTOL * v[a].abs().max(v_new[a].abs()) + DC_OP_TOL;\n\
             {it}    if !(delta.abs() < threshold) {{ step_ok = false; }}\n\
             {it}}}\n\n\
             {it}// The transient's limiter: pnjlim/fetlim on each proposed device\n\
             {it}// voltage shrink one ray `alpha` (Newton's direction is kept), then\n\
             {it}// the node-step damping bounds the largest node move at\n\
             {it}// max(10 V, 5 % of the largest node voltage).\n\
             {it}let mut alpha = 1.0_f64;\n"
        ));
        if m > 0 {
            Self::emit_nodal_voltage_limiting_indented(&mut code, ir, it);
        }
        code.push_str(&format!(
            "{it}{{\n\
             {it}    let mut max_node_dv = 0.0_f64;\n\
             {it}    for i in 0..{n_nodes} {{ let dv = alpha * (v_new[i] - v[i]); max_node_dv = max_node_dv.max(dv.abs()); }}\n\
             {it}    let mut max_v = 0.0_f64;\n\
             {it}    for i in 0..{n_nodes} {{ max_v = max_v.max(v[i].abs()); }}\n\
             {it}    let damp_thresh = 10.0_f64.max(max_v * 0.05);\n\
             {it}    if max_node_dv > damp_thresh {{ alpha *= damp_thresh / max_node_dv; }}\n\
             {it}}}\n\
             {it}for i in 0..N {{ v[i] += alpha * (v_new[i] - v[i]); }}\n\
             \x20       }}\n\n\
             \x20       if !converged {{\n\
             \x20           // Iteration exhaustion, a singular solve or a non-finite iterate:\n\
             \x20           // state untouched but for the thresholds put back here.\n\
             {vt_restore_nested}\
             \x20           state.diag_nr_max_iter_count += 1;\n\
             \x20           return;\n\
             \x20       }}\n\
             \x20       state.dc_op_writeback(v, i_nl, nr_iters);\n",
            vt_restore_nested = vt_restore.replace("        state.", "            state."),
        ));
        code
    }
}
