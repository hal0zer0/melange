//! Sub-sample fire: variable-dt breakpoint re-solve at glow strikes
//! (nodal-Schur route, `--subsample-fire`). Stage A prototype — correctness
//! first, NOT real-time optimised (two O(N^3) inversions per breakpoint).
//!
//! ## Why
//! A latched stateful device (glow discharge) flips its latch AFTER the
//! sample's solve, so conduction begins one full inner sample late and the
//! firing instant is quantised to the sample grid. The device already reports
//! the linear crossing fraction `alpha = (VO - v_start)/(v_end - v_start)` of
//! the strike threshold inside the step it was evaluated over; this module
//! consumes it.
//!
//! ## Both latch flips are breakpoints
//! Resolving the strike alone made lock WORSE (measured: the 5-stage divider
//! needed two more octaves of inner rate than the whole-sample latch). With a
//! continuous strike but a grid-evaluated extinction, the lit duration — and so
//! the reset depth of the slower lower-stage lamps (tau = RS*C around dt) —
//! jitters cycle to cycle instead of being consistently quantised. So the glow
//! hook reports EITHER flip (strike `fired`, extinction `extinguished`) with
//! its linear crossing fraction, and both are breakpoints.
//!
//! ## Multi-breakpoint, re-detected per segment
//! A divider cascade fires SEVERAL lamps inside one inner sample: the upstream
//! lamp's flyback (tau = RS*C, far below dt) couples into the next cathode and
//! strikes it within microseconds. Those cascade strikes are invisible to the
//! whole-sample solve (the upstream latch is frozen dark for it), so a schedule
//! taken from the whole-sample alphas would miss them. Instead the emitted
//! block is an EVENT LOOP over the sample, in fraction `t` of `dt`:
//!
//! 1. The remaining segment `[t0, 1]` has been solved (iteration 0 reuses the
//!    whole-sample solve). Run every stateful `update()` hook over it and take
//!    the EARLIEST interior flip (smallest alpha).
//! 2. No interior flip (or the segment guards say "grid point"): the hooks'
//!    latch flips stand, the segment end is the sample result. Done.
//! 3. Interior flip at `tc = t0 + alpha*(1-t0)`: rewind the latches to the
//!    segment start, solve the PRE-FLIP part `[t0, tc]` (skipped when its
//!    length is below the guard — a coincident/leading-edge flip), run the
//!    hooks over it, force the flipping lamp's latch to its post-flip value,
//!    then solve the REST `[tc, 1]` on the L-stable backward-Euler companion
//!    and go to 1 with `t0 = tc`. The next flip (a cascade strike, or this
//!    lamp's own extinction) is found by the hooks on that rest segment and
//!    gets its own breakpoint.
//!
//! Each segment is solved on a Schur triple rebuilt at rate `1/(len*dt)`
//! (`subsample_schur_build`), from the previous segment's converged `v`/`i_nl`
//! (zero-order NR warm start), with input and `.inject` values linearly
//! interpolated to the segment ends. The first pre-flip segment uses the
//! integrator the whole-sample solve used (trap, or BE via fallback /
//! breakpoint-BE); every later segment is BE. The trap RHS carries the
//! segment-start `N_I*i_nl` midpoint stamp; BE does not (matches the glow BE
//! fallback).
//!
//! History correctness: melange's "no history vector" companion form derives
//! the trap RHS `(2C/T - G) v_n + u_n + u_{n+1} + N_I(i_n + i_{n+1})` from KCL
//! at the step start, which holds at the end of ANY implicit step regardless
//! of its length or scheme, so chaining segments of different length and the
//! next full-dt step is self-consistent as long as each step uses matrices for
//! its own dt.
//!
//! Failure (singular A, NR budget exhausted, non-finite) ABANDONS the sample's
//! split: the whole-sample solution and its post-hook latches are restored and
//! `diag_subsample_fire_abandon_count` increments. A per-sample breakpoint
//! ceiling (`SUBSAMPLE_FIRE_MAX_BREAKPOINTS` = two per latched device) bounds
//! the loop; flips beyond it stay grid-quantised (counted).
//!
//! Every emitter here returns `""` unless `ir.solver_config.subsample_fire`,
//! so decks without the feature are byte-identical to the pre-feature emitter.

use super::dk_emitter::{emit_noise_replay_body, NoiseEmission};
use super::helpers::{emit_stateful_update_at, stateful_device_data, StatefulDeviceData};
use super::nodal_emitter::emit_sparse_ni_matvec_add;
use super::nr_helpers::{emit_nr_singular_fallback, emit_schur_nr_limit_and_converge};
use super::RustEmitter;
use crate::codegen::ir::CircuitIR;
use crate::codegen::CodegenError;

/// Segment-length guard, in fractions of the inner sample. A dark segment
/// shorter than this is skipped (the strike is taken at the segment start —
/// the continuous limit of the split, also the coincident-strike case); a rest
/// segment shorter than this is not split (the strike is at the grid point,
/// where the whole-sample latch flip is already exact). Bounds the sub-step
/// rate at `1e3 * rate`.
const MIN_SEGMENT: f64 = 1e-3;

/// Module-level consts (spliced into the nodal constants section).
pub(super) fn emit_subsample_fire_constants(ir: &CircuitIR) -> String {
    if !ir.solver_config.subsample_fire {
        return String::new();
    }
    // Two flips (strike + extinction) per latched device per sample.
    let max_breaks = 2 * stateful_device_data(ir)
        .iter()
        .filter(|d| d.is_latched)
        .count();
    let lit_tau = glow_lit_tau_min(ir);
    // Diagnostic lit sub-step multiplier; 0/unset → the 1.0 (tau_min) shipping
    // default (arbiter t303 — the last tested-safe point).
    let lit_factor = {
        let f = ir.solver_config.subsample_lit_factor;
        if f > 0.0 {
            f
        } else {
            1.0
        }
    };
    format!(
        "/// Sub-sample fire: minimum sub-step length as a fraction of the inner sample.\n\
         /// A dark segment shorter than this is skipped (strike taken at the segment\n\
         /// start); a rest segment shorter than this is not split (strike at the grid\n\
         /// point, already exact). Bounds the sub-step rate at 1e3x the inner rate.\n\
         pub const SUBSAMPLE_FIRE_MIN_SEGMENT: f64 = {MIN_SEGMENT:e};\n\
         /// Sub-sample fire: per-sample breakpoint ceiling (a strike and an extinction\n\
         /// per latched device). Flips beyond it stay grid-quantised and are counted in\n\
         /// `diag_subsample_fire_detected` and `diag_subsample_fire_unresolved_ceiling`.\n\
         pub const SUBSAMPLE_FIRE_MAX_BREAKPOINTS: u32 = {max_breaks};\n\
         /// Sub-sample fire: fastest lit discharge time constant among the glow\n\
         /// devices, tau = RS * C(terminal node) [s]. While any lamp is lit, segments\n\
         /// are capped at tau/2 so the L-stable BE companion integrates the stiff\n\
         /// discharge (a single BE step over dt >> tau decays algebraically, not\n\
         /// exponentially, holding the lamp lit far too long). 0.0 = not derivable,\n\
         /// lit sub-stepping off.\n\
         pub const SUBSAMPLE_FIRE_LIT_TAU_S: f64 = {lit_tau:e};\n\
         /// Sub-sample fire: diagnostic multiplier on the lit sub-step target\n\
         /// length (`FACTOR * LIT_TAU_S`); default 0.5. Smaller = finer lit\n\
         /// integration (fleet-arbiter thread 303 demand-3 sweep / lock-margin gate).\n\
         pub const SUBSAMPLE_FIRE_LIT_FACTOR: f64 = {lit_factor:e};\n\
         /// Sub-sample fire: ceiling on lit sub-steps per inner sample.\n\
         pub const SUBSAMPLE_FIRE_LIT_SUBSTEPS_MAX: u32 = 32;\n\n"
    )
}

/// Fastest lit discharge time constant among the glow devices: RS times the
/// larger terminal-node self-capacitance (`C[n][n]` sums every capacitor on the
/// node — the reservoir/timing cap dominates on the anode of a relaxation
/// stage). 0.0 when no glow device has a capacitive terminal.
fn glow_lit_tau_min(ir: &CircuitIR) -> f64 {
    use crate::codegen::ir::DeviceParams;
    let n = ir.topology.n;
    let c = &ir.matrices.c_matrix;
    let mut tau_min = f64::INFINITY;
    for slot in &ir.device_slots {
        let (DeviceParams::Glow(gp), Some(spec)) = (&slot.params, slot.stateful.as_ref()) else {
            continue;
        };
        let mut c_max = 0.0f64;
        for &node in &spec.terminal_nodes {
            if node > 0 && node - 1 < n && c.len() >= n * n {
                c_max = c_max.max(c[(node - 1) * n + (node - 1)].abs());
            }
        }
        if c_max > 0.0 && gp.rs > 0.0 {
            tau_min = tau_min.min(gp.rs * c_max);
        }
    }
    if tau_min.is_finite() {
        tau_min
    } else {
        0.0
    }
}

/// `CircuitState` diagnostic fields (struct body, 4-space indent).
pub(super) fn emit_subsample_fire_state_fields(ir: &CircuitIR) -> String {
    if !ir.solver_config.subsample_fire {
        return String::new();
    }
    "    /// Diagnostic: inner samples re-solved with at least one sub-sample\n\
     \x20   /// breakpoint (sub-sample fire). Nonzero is the feature working.\n\
     \x20   pub diag_subsample_fire_count: u64,\n\
     \x20   /// Diagnostic: sub-sample fire samples ABANDONED (singular sub-step\n\
     \x20   /// matrix, sub-step NR non-convergence, or non-finite result); the\n\
     \x20   /// whole-sample solution was kept for that sample.\n\
     \x20   pub diag_subsample_fire_abandon_count: u64,\n\
     \x20   /// Diagnostic: glow latch flips (strikes AND extinctions) that took effect,\n\
     \x20   /// however they were timed. Exact split: `detected = resolved +\n\
     \x20   /// unresolved_ceiling + unresolved_gridpoint + unresolved_coincident`\n\
     \x20   /// (an abandoned sample contributes to none of them).\n\
     \x20   pub diag_subsample_fire_detected: u64,\n\
     \x20   /// Diagnostic: latch flips resolved to a sub-sample breakpoint (committed).\n\
     \x20   pub diag_subsample_fire_resolved: u64,\n\
     \x20   /// Diagnostic: flips left grid-quantised because the per-sample breakpoint\n\
     \x20   /// budget (`SUBSAMPLE_FIRE_MAX_BREAKPOINTS`) was already spent. The crossing\n\
     \x20   /// was interior to its segment (a real timing miss, up to the segment length).\n\
     \x20   pub diag_subsample_fire_unresolved_ceiling: u64,\n\
     \x20   /// Diagnostic: flips left on the segment end because the crossing lay within\n\
     \x20   /// `SUBSAMPLE_FIRE_MIN_SEGMENT` of it (timing error below the guard; benign).\n\
     \x20   /// Counted before the ceiling: a grid-point flip is never a ceiling miss.\n\
     \x20   pub diag_subsample_fire_unresolved_gridpoint: u64,\n\
     \x20   /// Diagnostic: a SECOND device flipping inside the re-solved pre-flip segment\n\
     \x20   /// of a breakpoint; latched at that breakpoint `tc` rather than at its own\n\
     \x20   /// crossing (timing error up to the pre-flip segment length).\n\
     \x20   pub diag_subsample_fire_unresolved_coincident: u64,\n\
     \x20   /// Diagnostic: extra sub-step segments solved (pre-flip + rest + lit\n\
     \x20   /// sub-steps), i.e. the cost the feature added on top of one solve/sample.\n\
     \x20   pub diag_subsample_fire_segments: u64,\n\
     \x20   /// Diagnostic: Schur triples actually rebuilt (`subsample_schur_build`\n\
     \x20   /// calls that missed the same-(rate,be) memo). O(N^3) each.\n\
     \x20   pub diag_subsample_fire_schur_builds: u64,\n\
     \x20   /// Diagnostic: Schur-triple rebuilds AVOIDED by the same-(rate,be) memo\n\
     \x20   /// (bit-identical reuse of the cached triple).\n\
     \x20   pub diag_subsample_fire_schur_reuses: u64,\n"
        .to_string()
}

/// `Default` struct-literal entries (12-space indent).
pub(super) fn emit_subsample_fire_default_fields(ir: &CircuitIR) -> String {
    if !ir.solver_config.subsample_fire {
        return String::new();
    }
    "            diag_subsample_fire_count: 0,\n\
     \x20           diag_subsample_fire_abandon_count: 0,\n\
     \x20           diag_subsample_fire_detected: 0,\n\
     \x20           diag_subsample_fire_resolved: 0,\n\
     \x20           diag_subsample_fire_unresolved_ceiling: 0,\n\
     \x20           diag_subsample_fire_unresolved_gridpoint: 0,\n\
     \x20           diag_subsample_fire_unresolved_coincident: 0,\n\
     \x20           diag_subsample_fire_segments: 0,\n\
     \x20           diag_subsample_fire_schur_builds: 0,\n\
     \x20           diag_subsample_fire_schur_reuses: 0,\n"
        .to_string()
}

/// `reset()` body lines (8-space indent).
pub(super) fn emit_subsample_fire_reset(ir: &CircuitIR) -> String {
    if !ir.solver_config.subsample_fire {
        return String::new();
    }
    "        self.diag_subsample_fire_count = 0;\n\
     \x20       self.diag_subsample_fire_abandon_count = 0;\n\
     \x20       self.diag_subsample_fire_detected = 0;\n\
     \x20       self.diag_subsample_fire_resolved = 0;\n\
     \x20       self.diag_subsample_fire_unresolved_ceiling = 0;\n\
     \x20       self.diag_subsample_fire_unresolved_gridpoint = 0;\n\
     \x20       self.diag_subsample_fire_unresolved_coincident = 0;\n\
     \x20       self.diag_subsample_fire_segments = 0;\n\
     \x20       self.diag_subsample_fire_schur_builds = 0;\n\
     \x20       self.diag_subsample_fire_schur_reuses = 0;\n"
        .to_string()
}

/// The scratch Schur triple type and its builder (module level, after
/// `invert_n`). `g_src`/`c_src` are the same G/C sources `rebuild_matrices`
/// reads (`G`/`C` consts, or the working copies when pots/switches exist),
/// spelled against a `state: &CircuitState` receiver.
pub(super) fn emit_subsample_schur_builder(ir: &CircuitIR) -> String {
    if !ir.solver_config.subsample_fire {
        return String::new();
    }
    let has_pots = !ir.pots.is_empty();
    let has_switches = !ir.switches.is_empty();
    let (g, c) = if has_pots || has_switches {
        ("state.g_work", "state.c_work")
    } else {
        ("G", "C")
    };
    let n = ir.topology.n;
    let n_nodes = if ir.topology.n_nodes > 0 {
        ir.topology.n_nodes
    } else {
        n
    };
    let n_aug = ir.topology.n_aug;
    let mut s = String::new();
    s.push_str(
        "/// Scratch Schur triple for one variable-dt sub-step (sub-sample fire).\n\
         /// Stack-allocated per firing sample in Stage A (not RT-optimised).\n\
         struct SubsampleSchur {\n\
         \x20   a_neg: [[f64; N]; N],\n\
         \x20   s: [[f64; N]; N],\n\
         \x20   s_ni: [[f64; M]; N],\n\
         \x20   k: [[f64; M]; M],\n\
         }\n\n\
         impl SubsampleSchur {\n\
         \x20   #[inline(never)]\n\
         \x20   fn zeroed() -> Self {\n\
         \x20       SubsampleSchur {\n\
         \x20           a_neg: [[0.0; N]; N],\n\
         \x20           s: [[0.0; N]; N],\n\
         \x20           s_ni: [[0.0; M]; N],\n\
         \x20           k: [[0.0; M]; M],\n\
         \x20       }\n\
         \x20   }\n\
         }\n\n",
    );
    s.push_str(&format!(
        "/// Build A = G + alpha*C at the sub-step rate, invert it, and form the Schur\n\
         /// products S_NI = S*N_i, K = N_v*S_NI. `be` selects the backward-Euler\n\
         /// companion (alpha = rate, A_neg = alpha*C) over trapezoidal (alpha = 2*rate,\n\
         /// A_neg = alpha*C - G). Same construction as `rebuild_matrices`, including\n\
         /// the zeroed voltage-source algebraic history rows. False if A is singular.\n\
         #[inline(never)]\n\
         fn subsample_schur_build(state: &CircuitState, rate: f64, be: bool, out: &mut SubsampleSchur) -> bool {{\n\
         \x20   let _ = state;\n\
         \x20   let alpha = if be {{ rate }} else {{ 2.0 * rate }};\n\
         \x20   let mut a = [[0.0f64; N]; N];\n\
         \x20   for i in 0..N {{\n\
         \x20       for j in 0..N {{\n\
         \x20           a[i][j] = {g}[i][j] + alpha * {c}[i][j];\n\
         \x20           out.a_neg[i][j] = if be {{ alpha * {c}[i][j] }} else {{ alpha * {c}[i][j] - {g}[i][j] }};\n\
         \x20       }}\n\
         \x20   }}\n"
    ));
    if n_nodes < n_aug {
        s.push_str(&format!(
            "    for i in {n_nodes}..{n_aug} {{\n\
             \x20       for j in 0..N {{ out.a_neg[i][j] = 0.0; }}\n\
             \x20   }}\n"
        ));
    }
    s.push_str(
        "    let inv = match invert_n(&a) {\n\
         \x20       Some(inv) => inv,\n\
         \x20       None => return false,\n\
         \x20   };\n\
         \x20   out.s = inv;\n\
         \x20   for i in 0..N {\n\
         \x20       for j in 0..M {\n\
         \x20           let mut sum = 0.0;\n\
         \x20           for a in 0..N { sum += out.s[i][a] * N_I[a][j]; }\n\
         \x20           out.s_ni[i][j] = sum;\n\
         \x20       }\n\
         \x20   }\n\
         \x20   for i in 0..M {\n\
         \x20       for j in 0..M {\n\
         \x20           let mut sum = 0.0;\n\
         \x20           for a in 0..N { sum += N_V[i][a] * out.s_ni[a][j]; }\n\
         \x20           out.k[i][j] = sum;\n\
         \x20       }\n\
         \x20   }\n\
         \x20   true\n\
         }\n\n",
    );
    s
}

/// One Schur NR solve on the scratch triple `ssf_sub`, from the RHS in
/// `rhs_var`, warm-started from `i_from`. On success writes `out_v`/`out_i`;
/// on failure (budget exhausted or non-finite) clears `ok_var` and counts an
/// NR max-iter event. Emits its own block scope so the NR locals (`p`, `i_nl`,
/// `v_d*`, `i_dev*`, `jdev_*`) shadow nothing outside it.
fn emit_substep_solve(
    code: &mut String,
    ir: &CircuitIR,
    indent: &str,
    rhs_var: &str,
    i_from: &str,
    out_v: &str,
    out_i: &str,
    ok_var: &str,
) -> Result<(), CodegenError> {
    let m = ir.topology.m;
    let i1 = format!("{indent}    ");
    let i2 = format!("{indent}        ");
    let budget = if ir.solver_config.breakpoint_be {
        "BREAKPOINT_BE_MAX_ITER"
    } else {
        "MAX_ITER"
    };
    code.push_str(&format!("{indent}{{\n"));
    code.push_str(&format!(
        "{i1}let mut v_pred = [0.0f64; N];\n\
         {i1}for i in 0..N {{ let mut sum = 0.0; for j in 0..N {{ sum += ssf_sub.s[i][j] * {rhs_var}[j]; }} v_pred[i] = sum; }}\n\
         {i1}let mut p = [0.0f64; M];\n\
         {i1}for i in 0..M {{ let mut sum = 0.0; for j in 0..N {{ sum += N_V[i][j] * v_pred[j]; }} p[i] = sum; }}\n\
         {i1}// Zero-order warm start from the segment's own history (glow deck).\n\
         {i1}let mut i_nl = {i_from};\n\
         {i1}state.last_nr_iterations = {budget} as u32;\n\
         {i1}for iter in 0..{budget} {{\n"
    ));
    // v_d = p + K * i_nl (dense: the scratch K has no sparsity record)
    for i in 0..m {
        code.push_str(&format!("{i2}let v_d{i} = p[{i}]"));
        for j in 0..m {
            code.push_str(&format!(" + ssf_sub.k[{i}][{j}] * i_nl[{j}]"));
        }
        code.push_str(";\n");
    }
    // Device currents + Jacobians (latch state as set by the caller).
    for (dev_num, slot) in ir.device_slots.iter().enumerate() {
        RustEmitter::emit_dk_device_eval_for_nodal_schur_indented(code, dev_num, slot, &i2)?;
    }
    for i in 0..m {
        code.push_str(&format!("{i2}let f{i} = i_nl[{i}] - i_dev{i};\n"));
    }
    for i in 0..m {
        let slot = ir
            .device_slots
            .iter()
            .find(|s| i >= s.start_idx && i < s.start_idx + s.dimension)
            .ok_or_else(|| {
                CodegenError::InvalidConfig(format!(
                    "no device slot found for M-dimension index {i} (sub-sample fire)"
                ))
            })?;
        let mut terms = String::new();
        for j in 0..m {
            let diag = if i == j { "1.0" } else { "0.0" };
            terms.clear();
            for k in slot.start_idx..slot.start_idx + slot.dimension {
                terms.push_str(&format!(" - jdev_{i}_{k} * ssf_sub.k[{k}][{j}]"));
            }
            code.push_str(&format!("{i2}let j{i}_{j} = {diag}{terms};\n"));
        }
    }
    match m {
        1 => {
            code.push_str(&format!(
                "{i2}let det = j0_0;\n\
                 {i2}if det.abs() < 1e-15 {{\n"
            ));
            emit_nr_singular_fallback(code, 1, &format!("{i2}    "));
            code.push_str(&format!(
                "{i2}    continue;\n{i2}}}\n{i2}let delta0 = f0 / det;\n"
            ));
            emit_schur_nr_limit_and_converge(code, ir, 1, &i2, "ssf_sub.k");
        }
        2 => {
            code.push_str(&format!(
                "{i2}let det = j0_0 * j1_1 - j0_1 * j1_0;\n\
                 {i2}if det.abs() < 1e-15 {{\n"
            ));
            emit_nr_singular_fallback(code, 2, &format!("{i2}    "));
            code.push_str(&format!(
                "{i2}    continue;\n{i2}}}\n\
                 {i2}let inv_det = 1.0 / det;\n\
                 {i2}let delta0 = inv_det * (j1_1 * f0 - j0_1 * f1);\n\
                 {i2}let delta1 = inv_det * (-j1_0 * f0 + j0_0 * f1);\n"
            ));
            emit_schur_nr_limit_and_converge(code, ir, 2, &i2, "ssf_sub.k");
        }
        3..=24 => {
            // Fixed 8-space indent inside the helper (cosmetic only).
            RustEmitter::generate_schur_gauss_elim_k(code, ir, m, "ssf_sub.k");
        }
        _ => {
            return Err(CodegenError::UnsupportedTopology(format!(
                "M={} not supported (max {})",
                m,
                crate::dk::MAX_M
            )));
        }
    }
    code.push_str(&format!("{i1}}}\n"));
    code.push_str(&format!(
        "{i1}if state.last_nr_iterations >= {budget} as u32 || !i_nl.iter().all(|x| x.is_finite()) {{\n\
         {i1}    state.diag_nr_max_iter_count += 1;\n\
         {i1}    {ok_var} = false;\n\
         {i1}}} else {{\n\
         {i1}    let mut v_s = v_pred;\n\
         {i1}    for i in 0..N {{ for j in 0..M {{ v_s[i] += ssf_sub.s_ni[i][j] * i_nl[j]; }} }}\n\
         {i1}    if v_s.iter().all(|x| x.is_finite()) {{\n\
         {i1}        {out_v} = v_s;\n\
         {i1}        {out_i} = i_nl;\n\
         {i1}    }} else {{\n\
         {i1}        {ok_var} = false;\n\
         {i1}    }}\n\
         {i1}}}\n"
    ));
    code.push_str(&format!("{indent}}}\n"));
    Ok(())
}

/// Full RHS for a segment `[ta, tb]` of the inner sample: DC constants,
/// `A_neg * v_start`, trap-midpoint `N_I * i_start` (trap only), the input and
/// `.inject` sources interpolated to the segment ends (trap: start+end, BE:
/// end only), runtime sources, noise replay. `ta_var`/`tb_var` are the segment
/// end fractions in scope.
fn emit_segment_rhs(
    code: &mut String,
    ir: &CircuitIR,
    noise: &NoiseEmission,
    indent: &str,
    rhs_var: &str,
    v_start: &str,
    i_start: &str,
    ta_var: &str,
    tb_var: &str,
    be_var: &str,
) {
    let has_rhs_be = ir.has_dc_sources && !ir.matrices.rhs_const_be.is_empty();
    let trap_const = if ir.has_dc_sources {
        "RHS_CONST"
    } else {
        "[0.0f64; N]"
    };
    let be_const = if has_rhs_be {
        "RHS_CONST_BE"
    } else {
        "[0.0f64; N]"
    };
    code.push_str(&format!(
        "{indent}let mut {rhs_var}: [f64; N] = if {be_var} {{ {be_const} }} else {{ {trap_const} }};\n\
         {indent}for i in 0..N {{ for j in 0..N {{ {rhs_var}[i] += ssf_sub.a_neg[i][j] * {v_start}[j]; }} }}\n"
    ));
    if !ir.solver_config.backward_euler {
        // Trap-midpoint stamp of the segment-start nonlinear current. Omitted
        // on BE segments (the glow BE fallback omits it too: BE is N_I*i(n+1)).
        code.push_str(&format!("{indent}if !{be_var} {{\n"));
        code.push_str(&emit_sparse_ni_matvec_add(
            ir,
            rhs_var,
            i_start,
            &format!("{indent}    "),
        ));
        code.push_str(&format!("{indent}}}\n"));
    }
    // Input: linear ramp across the sample, evaluated at the segment ends.
    code.push_str(&format!(
        "{indent}{{\n\
         {indent}    let ua = state.input_prev + {ta_var} * (input - state.input_prev);\n\
         {indent}    let ub = state.input_prev + {tb_var} * (input - state.input_prev);\n\
         {indent}    {rhs_var}[INPUT_NODE] += (if {be_var} {{ ub }} else {{ ua + ub }}) * input_conductance;\n\
         {indent}}}\n"
    ));
    if !ir.solver_config.injections.is_empty() {
        code.push_str(&format!(
            "{indent}for k in 0..NUM_INJECT {{\n\
             {indent}    let ia = state.injections_prev[k] + {ta_var} * (injections[k] - state.injections_prev[k]);\n\
             {indent}    let ib = state.injections_prev[k] + {tb_var} * (injections[k] - state.injections_prev[k]);\n\
             {indent}    let inj = if {be_var} {{ ib }} else {{ ia + ib }};\n\
             {indent}    if INJECT_IS_NORTON[k] {{\n\
             {indent}        {rhs_var}[INJECT_NODES[k]] += inj;\n\
             {indent}    }} else {{\n\
             {indent}        {rhs_var}[INJECT_NODES[k]] += inj / INJECT_RESISTANCES[k];\n\
             {indent}    }}\n\
             {indent}}}\n"
        ));
    }
    for rt in &ir.runtime_sources {
        code.push_str(&format!(
            "{indent}{rhs_var}[{}] += state.{};\n",
            rt.vs_row, rt.field_name
        ));
    }
    if noise.enabled {
        code.push_str(&format!(
            "{indent}// Noise replay (cached i_n; consumes no RNG draws).\n"
        ));
        code.push_str(&emit_noise_replay_body(
            noise.replay_counts,
            rhs_var,
            indent,
        ));
    }
}

/// Emit a memoized `subsample_schur_build` call. Within a sample G/C are fixed,
/// so the Schur triple is a pure function of `(rate, be)`; a segment whose exact
/// rate bit-pattern and `be` flag match the currently-cached triple reuses it
/// verbatim (bit-identical to a rebuild), otherwise it rebuilds and updates the
/// cache. Expects `ssf_sub`, `ssf_sub_key`, `ssf_sub_key_be`, `ssf_sub_valid`,
/// `ssf_builds`, `ssf_reuses`, `state` in scope. `fail_stmt` runs on a singular
/// rebuild (e.g. `ssf_ok = false;` or `ssf_ok = false; break;`); a cache hit
/// cannot fail (the identical build already succeeded this sample).
fn emit_cached_build(
    code: &mut String,
    indent: &str,
    rate_expr: &str,
    be_expr: &str,
    fail_stmt: &str,
) {
    let i1 = format!("{indent}    ");
    code.push_str(&format!(
        "{indent}{{\n\
         {i1}let ssf_r = {rate_expr};\n\
         {i1}let ssf_be = {be_expr};\n\
         {i1}let ssf_kbits = ssf_r.to_bits();\n\
         {i1}if ssf_sub_valid && ssf_sub_key == ssf_kbits && ssf_sub_key_be == ssf_be {{\n\
         {i1}    ssf_reuses += 1;\n\
         {i1}}} else if subsample_schur_build(state, ssf_r, ssf_be, &mut ssf_sub) {{\n\
         {i1}    ssf_sub_key = ssf_kbits;\n\
         {i1}    ssf_sub_key_be = ssf_be;\n\
         {i1}    ssf_sub_valid = true;\n\
         {i1}    ssf_builds += 1;\n\
         {i1}}} else {{\n\
         {i1}    {fail_stmt}\n\
         {i1}}}\n\
         {indent}}}\n"
    ));
}

/// The per-sample block. Replaces the plain `emit_stateful_update` splice on
/// the nodal-Schur path when the feature is active. Expects in scope: `input`,
/// `input_conductance`, `injections` (if `.inject`), `converged`, `v` (mut),
/// `i_nl` (mut), `state`.
pub(super) fn emit_subsample_fire_block(
    ir: &CircuitIR,
    noise: &NoiseEmission,
) -> Result<String, CodegenError> {
    let devs: Vec<StatefulDeviceData> = stateful_device_data(ir);
    let latched: Vec<&StatefulDeviceData> = devs.iter().filter(|d| d.is_latched).collect();
    if latched.is_empty() {
        return Err(CodegenError::InvalidConfig(
            "sub-sample fire requested but the circuit has no latched stateful device".to_string(),
        ));
    }
    if ir.solver_config.num_inputs() > 1 {
        return Err(CodegenError::InvalidConfig(
            "sub-sample fire does not support multi-input ports (M>0 never does)".to_string(),
        ));
    }
    let be_primary = ir.solver_config.backward_euler;
    let mut code = String::new();
    let i0 = "    ";
    let i1 = "        ";
    let i2 = "            ";
    let i3 = "                ";

    let snapshot = |code: &mut String, indent: &str, prefix: &str| {
        for d in &devs {
            let n = d.dev_num;
            code.push_str(&format!("{indent}{prefix}{n} = state.device_{n}_state;\n"));
        }
    };
    let restore = |code: &mut String, indent: &str, prefix: &str| {
        for d in &devs {
            let n = d.dev_num;
            code.push_str(&format!("{indent}state.device_{n}_state = {prefix}{n};\n"));
        }
    };
    let any_lit: String = latched
        .iter()
        .map(|d| format!("state.device_{}_state[0] >= 0.5", d.dev_num))
        .collect::<Vec<_>>()
        .join(" || ");

    code.push_str(&format!(
        "{i0}// Sub-sample fire (Stage A, multi-breakpoint): event loop over the inner\n\
         {i0}// sample in fraction t. A segment [t0, t1] is solved (iteration 0 reuses\n\
         {i0}// the whole-sample solve unless a lamp is lit at the sample start), the\n\
         {i0}// stateful hooks run over it, and the EARLIEST interior latch flip splits\n\
         {i0}// it: rewind latches, solve the pre-flip part [t0, tc], force the flip,\n\
         {i0}// continue from tc. While any lamp is lit the segments are capped at\n\
         {i0}// ~tau/2 of the fastest lit discharge so BE integrates the stiff lit phase\n\
         {i0}// instead of one algebraic step. Cascade strikes and extinctions are\n\
         {i0}// re-detected on every segment.\n\
         {i0}{{\n\
         {i1}let ssf_rate = state.current_sample_rate * OVERSAMPLING_FACTOR as f64;\n\
         {i1}let ssf_dt = 1.0 / ssf_rate;\n\
         {i1}let ssf_saved_iters = state.last_nr_iterations;\n\
         {i1}let ssf_lit_seg = if SUBSAMPLE_FIRE_LIT_TAU_S > 0.0 {{\n\
         {i2}let k = (ssf_dt / (SUBSAMPLE_FIRE_LIT_FACTOR * SUBSAMPLE_FIRE_LIT_TAU_S)).ceil().clamp(1.0, SUBSAMPLE_FIRE_LIT_SUBSTEPS_MAX as f64);\n\
         {i2}1.0 / k\n\
         {i1}}} else {{\n\
         {i2}1.0\n\
         {i1}}};\n\
         {i1}let mut ssf_t0 = 0.0f64;\n\
         {i1}let mut ssf_t1 = 1.0f64;\n\
         {i1}let mut ssf_v0 = state.v_prev;\n\
         {i1}let mut ssf_i0 = state.i_nl_prev;\n\
         {i1}let mut ssf_v_end = v;\n\
         {i1}let mut ssf_i_end = i_nl;\n\
         {i1}let mut ssf_seg_be = {seg_be};\n\
         {i1}let mut ssf_breaks = 0u32;\n\
         {i1}let mut ssf_segs = 0u32;\n\
         {i1}let mut ssf_ok = true;\n\
         {i1}let mut ssf_detected = 0u64;\n\
         {i1}let mut ssf_unres_ceiling = 0u64;\n\
         {i1}let mut ssf_unres_gridpoint = 0u64;\n\
         {i1}let mut ssf_unres_coincident = 0u64;\n\
         {i1}let mut ssf_sub = SubsampleSchur::zeroed();\n\
         {i1}// Byte-neutral Schur-triple memo: within a sample G/C are fixed, so the\n\
         {i1}// triple depends only on (rate, be). Same-alpha segments (the repeated\n\
         {i1}// lit sub-steps) reuse the last build verbatim; a distinct (rate, be)\n\
         {i1}// rebuilds. Keyed on the exact rate bit-pattern -> the reused triple is\n\
         {i1}// bit-identical to a rebuild, so the emitted audio is unchanged.\n\
         {i1}let mut ssf_sub_key: u64 = 0u64;\n\
         {i1}let mut ssf_sub_key_be: bool = false;\n\
         {i1}let mut ssf_sub_valid: bool = false;\n\
         {i1}let mut ssf_builds: u32 = 0;\n\
         {i1}let mut ssf_reuses: u32 = 0;\n",
        seg_be = if be_primary { "true" } else { "!converged" }
    ));
    // Pre-sample latch snapshot: the abandon path replays the whole-sample hooks from it.
    snapshot(&mut code, i1, "let ssf_start_");
    code.push_str(&format!(
        "{i1}if ({any_lit}) && ssf_lit_seg < 1.0 {{\n\
         {i2}// A lamp is lit at the sample start: integrate the stiff lit phase in\n\
         {i2}// sub-steps instead of taking the whole-sample BE solve.\n\
         {i2}ssf_t1 = ssf_lit_seg;\n\
         {i2}ssf_seg_be = true;\n\
         {i2}ssf_segs = 1;\n"
    ));
    emit_cached_build(
        &mut code,
        i2,
        "ssf_rate / ssf_t1",
        "true",
        "ssf_ok = false;",
    );
    code.push_str(&format!("{i2}if ssf_ok {{\n"));
    emit_segment_rhs(
        &mut code,
        ir,
        noise,
        i3,
        "ssf_rhs",
        "ssf_v0",
        "ssf_i0",
        "ssf_t0",
        "ssf_t1",
        "ssf_seg_be",
    );
    emit_substep_solve(
        &mut code,
        ir,
        i3,
        "ssf_rhs",
        "ssf_i0",
        "ssf_v_end",
        "ssf_i_end",
        "ssf_ok",
    )?;
    code.push_str(&format!(
        "{i2}}}\n\
         {i1}}}\n\
         {i1}while ssf_ok {{\n\
         {i2}// Hooks over the segment [t0, t1]; earliest interior latch flip.\n"
    ));
    snapshot(&mut code, i2, "let ssf_pre_");
    code.push_str(&format!(
        "{i2}let mut ssf_alpha = f64::INFINITY;\n\
         {i2}let mut ssf_dev = usize::MAX;\n\
         {i2}let mut ssf_dir = 0.0f64; // latch value after the flip (1.0 strike, 0.0 extinction)\n\
         {i2}let mut ssf_fired = 0u64;\n\
         {i2}let ssf_seg_dt = (ssf_t1 - ssf_t0) * ssf_dt;\n"
    ));
    code.push_str(&emit_stateful_update_at(
        &devs,
        i2,
        "ssf_v0",
        "ssf_v_end",
        "ssf_seg_dt",
        "if upd.fired || upd.extinguished { ssf_fired += 1; if upd.alpha < ssf_alpha { ssf_alpha = upd.alpha; ssf_dev = {n}; ssf_dir = if upd.fired { 1.0 } else { 0.0 }; } }",
    ));
    code.push_str(&format!(
        "{i2}let ssf_tc = ssf_t0 + ssf_alpha * (ssf_t1 - ssf_t0);\n\
         {i2}let ssf_split = ssf_dev != usize::MAX\n\
         {i3}&& ssf_breaks < SUBSAMPLE_FIRE_MAX_BREAKPOINTS\n\
         {i3}&& (ssf_t1 - ssf_tc) > SUBSAMPLE_FIRE_MIN_SEGMENT;\n\
         {i2}if !ssf_split {{\n\
         {i3}// Commit the segment: the hooks' flips stand, left on the segment end.\n\
         {i3}// Reason split (grid point first: a crossing within the guard of the end\n\
         {i3}// is exact by construction and never a budget miss). `ssf_fired > 0`\n\
         {i3}// implies `ssf_dev != usize::MAX` (every reported alpha is finite).\n\
         {i3}ssf_detected += ssf_fired;\n\
         {i3}if ssf_fired > 0 {{\n\
         {i3}    if (ssf_t1 - ssf_tc) <= SUBSAMPLE_FIRE_MIN_SEGMENT {{\n\
         {i3}        ssf_unres_gridpoint += ssf_fired;\n\
         {i3}    }} else {{\n\
         {i3}        ssf_unres_ceiling += ssf_fired;\n\
         {i3}    }}\n\
         {i3}}}\n\
         {i3}ssf_t0 = ssf_t1;\n\
         {i3}ssf_v0 = ssf_v_end;\n\
         {i3}ssf_i0 = ssf_i_end;\n\
         {i3}if ssf_t0 >= 1.0 {{ break; }}\n\
         {i2}}} else {{\n\
         {i3}// Interior flip at tc: rewind the latches to the segment start.\n"
    ));
    restore(&mut code, i3, "ssf_pre_");
    code.push_str(&format!(
        "{i3}ssf_breaks += 1;\n\
         {i3}let mut ssf_vc = ssf_v0;\n\
         {i3}let mut ssf_ic = ssf_i0;\n\
         {i3}if ssf_tc - ssf_t0 > SUBSAMPLE_FIRE_MIN_SEGMENT {{\n"
    ));
    let i4 = "                    ";
    code.push_str(&format!(
        "{i4}// PRE-FLIP segment [t0, tc] on the segment's integrator.\n"
    ));
    emit_cached_build(
        &mut code,
        i4,
        "ssf_rate / (ssf_tc - ssf_t0)",
        "ssf_seg_be",
        "ssf_ok = false; break;",
    );
    emit_segment_rhs(
        &mut code,
        ir,
        noise,
        i4,
        "ssf_rhs",
        "ssf_v0",
        "ssf_i0",
        "ssf_t0",
        "ssf_tc",
        "ssf_seg_be",
    );
    emit_substep_solve(
        &mut code, ir, i4, "ssf_rhs", "ssf_i0", "ssf_vc", "ssf_ic", "ssf_ok",
    )?;
    code.push_str(&format!(
        "{i4}if !ssf_ok {{ break; }}\n\
         {i4}// Hooks over the pre-flip segment (another lamp flipping before tc lands on\n\
         {i4}// tc). The flipping lamp itself is expected to read as flipped here — its\n\
         {i4}// crossing IS tc — and is counted once, at the forced flip below.\n\
         {i4}let ssf_pre_dt = (ssf_tc - ssf_t0) * ssf_dt;\n"
    ));
    code.push_str(&emit_stateful_update_at(
        &devs,
        i4,
        "ssf_v0",
        "ssf_vc",
        "ssf_pre_dt",
        "if (upd.fired || upd.extinguished) && {n} != ssf_dev { ssf_detected += 1; ssf_unres_coincident += 1; }",
    ));
    code.push_str(&format!(
        "{i3}}}\n\
         {i3}// The flip: force the earliest lamp's latch to its post-flip value at tc\n\
         {i3}// (its own hook may read the interpolated crossing voltage a hair short).\n\
         {i3}match ssf_dev {{\n"
    ));
    for d in &latched {
        let n = d.dev_num;
        code.push_str(&format!(
            "{i4}{n} => state.device_{n}_state[0] = ssf_dir,\n"
        ));
    }
    code.push_str(&format!(
        "{i4}_ => {{}}\n\
         {i3}}}\n\
         {i3}ssf_detected += 1;\n\
         {i3}ssf_t0 = ssf_tc;\n\
         {i3}ssf_v0 = ssf_vc;\n\
         {i3}ssf_i0 = ssf_ic;\n\
         {i2}}}\n\
         {i2}// NEXT segment [t0, t1] on the L-stable BE companion (a breakpoint sample is\n\
         {i2}// a BE sample, as with breakpoint-BE / the glow lit-hold). While a lamp is\n\
         {i2}// lit the segment is capped at the lit sub-step so BE resolves the discharge.\n\
         {i2}ssf_seg_be = true;\n\
         {i2}ssf_t1 = if {any_lit} {{ (ssf_t0 + ssf_lit_seg).min(1.0) }} else {{ 1.0 }};\n\
         {i2}if 1.0 - ssf_t1 < SUBSAMPLE_FIRE_MIN_SEGMENT {{ ssf_t1 = 1.0; }}\n\
         {i2}ssf_segs += 1;\n\
         {i2}if ssf_segs > SUBSAMPLE_FIRE_MAX_BREAKPOINTS + SUBSAMPLE_FIRE_LIT_SUBSTEPS_MAX + 2 {{ ssf_ok = false; break; }}\n"
    ));
    emit_cached_build(
        &mut code,
        i2,
        "ssf_rate / (ssf_t1 - ssf_t0)",
        "true",
        "ssf_ok = false; break;",
    );
    emit_segment_rhs(
        &mut code,
        ir,
        noise,
        i2,
        "ssf_rhs",
        "ssf_v0",
        "ssf_i0",
        "ssf_t0",
        "ssf_t1",
        "ssf_seg_be",
    );
    emit_substep_solve(
        &mut code,
        ir,
        i2,
        "ssf_rhs",
        "ssf_i0",
        "ssf_v_end",
        "ssf_i_end",
        "ssf_ok",
    )?;
    code.push_str(&format!(
        "{i1}}}\n\
         {i1}if ssf_ok {{\n\
         {i2}v = ssf_v_end;\n\
         {i2}i_nl = ssf_i_end;\n\
         {i2}state.diag_subsample_fire_detected += ssf_detected;\n\
         {i2}state.diag_subsample_fire_resolved += ssf_breaks as u64;\n\
         {i2}state.diag_subsample_fire_unresolved_ceiling += ssf_unres_ceiling;\n\
         {i2}state.diag_subsample_fire_unresolved_gridpoint += ssf_unres_gridpoint;\n\
         {i2}state.diag_subsample_fire_unresolved_coincident += ssf_unres_coincident;\n\
         {i2}state.diag_subsample_fire_segments += ssf_segs as u64;\n\
         {i2}if ssf_breaks > 0 {{ state.diag_subsample_fire_count += 1; }}\n\
         {i1}}} else {{\n\
         {i2}// Abandon: keep the whole-sample solution and replay the whole-sample\n\
         {i2}// hooks from the pre-sample latches (the pre-feature outcome).\n"
    ));
    restore(&mut code, i2, "ssf_start_");
    code.push_str(&emit_stateful_update_at(
        &devs,
        i2,
        "state.v_prev",
        "v",
        "ssf_dt",
        "let _ = upd;",
    ));
    code.push_str(&format!(
        "{i2}state.diag_subsample_fire_abandon_count += 1;\n\
         {i1}}}\n\
         {i1}state.diag_subsample_fire_schur_builds += ssf_builds as u64;\n\
         {i1}state.diag_subsample_fire_schur_reuses += ssf_reuses as u64;\n\
         {i1}state.last_nr_iterations = ssf_saved_iters;\n\
         {i0}}}\n"
    ));
    Ok(code)
}
