//! Regression tests for the 2026-07-18 nodal-emitter NR control-flow fixes.
//!
//! Covers (numbering matches the fix campaign):
//!  1. `input_prev` committed at end-of-sample (not before the sub-step
//!     machinery reads it) — plus a DK-vs-nodal step-response equivalence
//!     check that would catch a missing/double commit (the classic "~3%
//!     error from 2·V·G instead of (V+V_prev)·G" signature).
//!  2. Full-LU sub-step `alpha_sub` tracks the RUNTIME sample rate instead
//!     of a literal baked at codegen time.
//!  3. Full-LU sub-step Gmin is 1e-12 (was 1e-6 — 6 orders stronger than
//!     every other Gmin stamp in the emitter).
//!  4. BE-primary builds must not re-add the trap-midpoint `N_I·i_nl_prev`
//!     stamp in the BE-fallback RHS (trap-primary keeps it per the
//!     2026-05-28 restoration).
//!  5. `.runtime V` sources are stamped in ALL from-scratch RHS rebuilds
//!     (trap, sub-step, BE fallback) — they are integration-scheme
//!     independent algebraic constraints.
//!  6. Schur ActiveSetBe sub-step honesty: a diverged Picard fixed point
//!     must leave `converged = false` (BE fallback runs) instead of being
//!     committed unconditionally.
//!  7. Schur BE-fallback voltage limiting uses a single scalar alpha
//!     (min across ALL dims) — per-dimension alpha breaks the coupled
//!     Newton direction (VOLTAGE_LIMITING.md).
//!  8. Diag counter contract: `last_nr_iterations` pessimistically
//!     initialized on the full-LU path; `diag_be_fallback_count` counts
//!     fallback ENTRY (not success); `diag_nr_max_iter_count` counts only
//!     genuine trap max-iter.
//!  9. `set_sample_rate` same-rate early return: guarded by an all-default
//!     pot/switch check, and `current_sample_rate` is recorded BEFORE the
//!     early return (e2e: moving a pot then calling set_sample_rate at the
//!     codegen rate must NOT silently snap matrices back to nominal).
//! 10. `current_sample_rate` stores the HOST rate (DK parity); consumers
//!     multiply by `OVERSAMPLING_FACTOR`.
//! 11. Rate-change DC-blocker history reseeds from the DC operating point
//!     (not zeros).
//! 12. NaN reset: DC-blocker reseed from DC OP, no pot snap (covered in
//!     `nodal_codegen_tests::test_nodal_nan_reset_includes_extended_state`),
//!     and the DK denormal bias `dc_blocked + 1e-20` on the blocker
//!     feedback.
//! 13. Non-finite NR iterates read as NOT converged (negated predicates).

mod support;

use melange_solver::codegen::CodegenConfig;

const SR: f64 = 44100.0;

/// Diode clipper with caps: M=2, nonlinear, well-conditioned on both the DK
/// path and the nodal path.
const CLIPPER: &str = "\
Diode Clipper
R1 in mid 4.7k
D1 mid 0 DCLIP
D2 0 mid DCLIP
C1 mid 0 47n
Cout mid out 1u
Rload out 0 100k
.model DCLIP D(IS=1e-14 N=1.9)
";

/// Same clipper forced onto the full-LU route via a cap-only node (two
/// series 1 pF caps with no resistive path make `max|S| > 1e6`, tripping the
/// `s_ill_conditioned` full-LU routing guard — same trick as
/// `opamp_rail_mode_regression_tests.rs`). NOTE: the behavioral B-source
/// forcing branch used elsewhere is unsuitable here — behavioral sources
/// force BE-primary on the nodal path (ddt backward-difference contract),
/// which removes the trap machinery these tests inspect.
const CLIPPER_FULL_LU: &str = "\
Diode Clipper (forced full-LU via cap-only node)
R1 in mid 4.7k
D1 mid 0 DCLIP
D2 0 mid DCLIP
C1 mid 0 47n
Cout mid out 1u
Rload out 0 100k
Cxa in x1 1p
Cxb x1 0 1p
.model DCLIP D(IS=1e-14 N=1.9)
";

/// Clipper with a pot (for the set_sample_rate / rebuild-rate tests).
const CLIPPER_POT: &str = "\
Diode Clipper With Pot
R1 in mid 4.7k
Rpot mid 0 10k
D1 mid 0 DCLIP
D2 0 mid DCLIP
C1 mid 0 47n
Cout mid out 1u
Rload out 0 100k
.pot Rpot 1k 100k
.model DCLIP D(IS=1e-14 N=1.9)
";

/// Clipper with a `.runtime V` control source.
const CLIPPER_RUNTIME_V: &str = "\
Diode Clipper With Runtime Bias
R1 in mid 4.7k
D1 mid 0 DCLIP
D2 0 mid DCLIP
C1 mid 0 47n
Vctrl bias 0 DC 0
Rb bias mid 100k
Cout mid out 1u
Rload out 0 100k
.runtime Vctrl as ctrl_v
.model DCLIP D(IS=1e-14 N=1.9)
";

fn nodal_code(spice: &str) -> String {
    // The inert full-LU forcing branch pushes spectral_radius(S*A_neg) to
    // ~1.0, which auto-promotes the build to backward Euler and removes the
    // trap-path machinery these tests inspect. Pin trap-primary; the
    // BE-primary variants use `nodal_code_with(|c| c.backward_euler = true)`.
    nodal_code_with(spice, |c| c.force_trap = true)
}

fn nodal_code_with(spice: &str, f: impl FnOnce(&mut CodegenConfig)) -> String {
    let mut config = support::config_for_spice(spice, SR);
    f(&mut config);
    support::generate_circuit_code_nodal(spice, &config).0
}

/// Assert `needle` occurs in `hay` and return the byte offset of the first
/// occurrence.
fn find(hay: &str, needle: &str, why: &str) -> usize {
    hay.find(needle)
        .unwrap_or_else(|| panic!("expected generated code to contain {needle:?} — {why}"))
}

// ═══════════════════════════════════════════════════════════════════════
// Fix 1 — input_prev commit placement
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn full_lu_input_prev_committed_after_substep_block() {
    let code = nodal_code(CLIPPER_FULL_LU);
    assert!(
        code.contains("// Cross-timestep chord"),
        "circuit must route full-LU for this test (forcing branch broken?)"
    );

    let commit = find(
        &code,
        "state.input_prev = input;",
        "input_prev must still be committed once per sample",
    );
    assert_eq!(
        code.matches("state.input_prev = input;").count(),
        1,
        "input_prev must be committed exactly once (double commit would zero \
         the sub-step ramp again)"
    );
    let substep = find(
        &code,
        "Adaptive sub-stepping",
        "full-LU path must emit the sub-step retry block",
    );
    let interp = find(
        &code,
        "(input - state.input_prev) / subdiv as f64",
        "sub-step input interpolation must read input_prev",
    );
    assert!(
        commit > substep && commit > interp,
        "input_prev must be committed AFTER the sub-step machinery reads it \
         (commit at byte {commit}, substep block at {substep}, interpolation \
         at {interp}) — committing before makes the sub-step input ramp \
         identically zero on exactly the hard-transient samples that trigger \
         sub-stepping"
    );
    // The trapezoidal input stamp must still read the previous value.
    find(
        &code,
        "(input + state.input_prev) * input_conductance",
        "trap input stamp must remain (V + V_prev) * G_in",
    );
}

#[test]
fn schur_input_prev_committed_in_state_update() {
    let code = nodal_code(CLIPPER);
    assert!(
        !code.contains("// Cross-timestep chord"),
        "clipper without forcing branch should route Schur; if routing \
         changed, update this test's circuit"
    );
    let commit = find(&code, "state.input_prev = input;", "commit must exist");
    assert_eq!(code.matches("state.input_prev = input;").count(), 1);
    let state_update = find(&code, "// State update", "state update block");
    assert!(
        commit > state_update,
        "Schur input_prev commit must live in the end-of-sample state-update \
         block (commit at {commit}, state update at {state_update})"
    );
}

/// End-to-end: DK and nodal builds of the same clipper must agree on a step
/// response. A missing (or doubled) `input_prev` commit shows up here as a
/// persistent trap-rule input error (the classic "~3% error from 2·V·G"
/// signature).
#[test]
fn step_response_dk_vs_nodal_matches() {
    let config = support::config_for_spice(CLIPPER, SR);
    let dk = support::build_circuit(CLIPPER, &config, "ctlflow_step_dk");
    let nodal = support::build_circuit_nodal(CLIPPER, &config, "ctlflow_step_nodal");

    let n = 512;
    let dk_out = support::run_step(&dk, 0.5, n, SR);
    let nodal_out = support::run_step(&nodal, 0.5, n, SR);
    support::assert_finite(&dk_out);
    support::assert_finite(&nodal_out);
    support::assert_samples_match(&dk_out, &nodal_out, 1e-6, "clipper step dk-vs-nodal");
}

// ═══════════════════════════════════════════════════════════════════════
// Fix 2 + 3 — full-LU sub-step alpha_sub / Gmin
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn full_lu_alpha_sub_tracks_runtime_rate() {
    let code = nodal_code(CLIPPER_FULL_LU);
    find(
        &code,
        "let alpha_sub = 2.0 * state.current_sample_rate * OVERSAMPLING_FACTOR as f64 * subdiv as f64;",
        "sub-step alpha must be computed from the runtime host rate",
    );
    // The old form baked 2·fs·os as a 17-digit literal.
    let baked = format!("{:.17e} * subdiv as f64", 2.0 * SR);
    assert!(
        !code.contains(&baked),
        "sub-step alpha must not bake the codegen-time rate ({baked})"
    );
    // The runtime-rate expression requires the state field even without pots.
    find(
        &code,
        "pub current_sample_rate: f64,",
        "full-LU circuits must carry current_sample_rate for the sub-step alpha",
    );
}

#[test]
fn full_lu_substep_gmin_matches_other_stamps() {
    let code = nodal_code(CLIPPER_FULL_LU);
    find(
        &code,
        "a_sub[i][i] += 1e-12;",
        "sub-step Gmin must be 1e-12 like every other Gmin stamp",
    );
    assert!(
        !code.contains("a_sub[i][i] += 1e-6;"),
        "sub-step Gmin of 1e-6 skews high-impedance nodes on sub-stepped samples"
    );
}

// ═══════════════════════════════════════════════════════════════════════
// Fix 4 — BE-primary fallback must not re-add the trap-midpoint stamp
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn be_primary_fallback_skips_trap_midpoint_stamp() {
    // The runtime-loop form of the stamp only appears in the BE-fallback RHS
    // builds (the primary trap RHS uses unrolled sparse stamps).
    let midpoint_stamp = "sum += N_I[i][j] * state.i_nl_prev[j];";

    for (tag, spice) in [("schur", CLIPPER), ("full_lu", CLIPPER_FULL_LU)] {
        let trap_code = nodal_code(spice);
        assert!(
            trap_code.contains(midpoint_stamp),
            "{tag}: trap-primary build keeps the fallback midpoint stamp \
             (2026-05-28 restoration)"
        );
        let be_code = nodal_code_with(spice, |c| c.backward_euler = true);
        assert!(
            !be_code.contains(midpoint_stamp),
            "{tag}: BE-primary build must not re-add N_I*i_nl_prev in the \
             BE fallback RHS — the primary RHS already skips it under BE and \
             the fallback is the same discretization"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Fix 5 — .runtime V stamped in every from-scratch RHS rebuild
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn runtime_v_source_stamped_in_all_rhs_rebuilds() {
    // Schur route: trap RHS + BE fallback.
    let schur = nodal_code(CLIPPER_RUNTIME_V);
    assert!(
        schur.contains("state.ctrl_v"),
        "runtime source field must be consumed"
    );
    let re_rhs = regex_lite_count(&schur, "rhs[", "state.ctrl_v;");
    let re_be = regex_lite_count(&schur, "rhs_be[", "state.ctrl_v;");
    assert!(
        re_rhs >= 1,
        "Schur trap RHS must stamp the runtime V source"
    );
    assert!(
        re_be >= 1,
        "Schur BE-fallback RHS must stamp the runtime V source (it vanished \
         to 0 V on fallback samples before this fix)"
    );

    // Full-LU route: trap RHS + sub-step RHS + BE fallback.
    let full = {
        let spice = format!("{}B_frc frc 0 V={{0}}\nR_frc frc 0 1k\n", CLIPPER_RUNTIME_V);
        nodal_code(&spice)
    };
    assert!(
        full.contains("// Cross-timestep chord"),
        "forcing branch must route full-LU"
    );
    assert!(
        regex_lite_count(&full, "rhs[", "state.ctrl_v;") >= 1,
        "full-LU trap RHS must stamp the runtime V source"
    );
    assert!(
        regex_lite_count(&full, "rhs_s[", "state.ctrl_v;") >= 1,
        "full-LU sub-step RHS must stamp the runtime V source"
    );
    assert!(
        regex_lite_count(&full, "rhs_be[", "state.ctrl_v;") >= 1,
        "full-LU BE-fallback RHS must stamp the runtime V source"
    );
}

/// Count lines that contain both fragments (poor man's regex).
fn regex_lite_count(code: &str, frag_a: &str, frag_b: &str) -> usize {
    code.lines()
        .filter(|l| l.contains(frag_a) && l.contains(frag_b))
        .count()
}

// ═══════════════════════════════════════════════════════════════════════
// Fix 7 — Schur BE fallback scalar alpha
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn schur_be_fallback_uses_scalar_alpha() {
    let code = nodal_code(CLIPPER);
    find(
        &code,
        "let mut alpha_scalar = alpha[0].min(alpha[1]);",
        "BE fallback must min-reduce alpha across ALL dims (M=2 clipper)",
    );
    find(
        &code,
        "i_nl[0] -= alpha_scalar * delta0;",
        "BE fallback step must apply the scalar alpha",
    );
    assert!(
        !code.contains("i_nl[0] -= alpha[0] * delta0;"),
        "per-dimension alpha application breaks the coupled Newton direction \
         (VOLTAGE_LIMITING.md / nr_helpers.rs scalar-alpha rationale)"
    );
    assert!(
        !code.contains("let dev_alpha = alpha["),
        "the per-device grouping is subsumed by the global scalar min"
    );
}

// ═══════════════════════════════════════════════════════════════════════
// Fix 8 — diag counter contract
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn full_lu_diag_counter_contract() {
    let code = nodal_code(CLIPPER_FULL_LU);

    // (a) pessimistic init before the trap loop — the field must not carry a
    // stale value from the previous sample into the failure branch.
    let init = find(
        &code,
        "state.last_nr_iterations = MAX_ITER as u32;",
        "full-LU must pessimistically initialize last_nr_iterations",
    );
    let trap_loop = find(&code, "for iter in 0..MAX_ITER {", "trap NR loop");
    assert!(
        init < trap_loop,
        "pessimistic init must precede the trap loop (init at {init}, loop at {trap_loop})"
    );

    // (b) exactly one genuine-max-iter increment site in process_sample,
    // gated on last_nr_iterations (not on fallback entry). reset() zeroes
    // the counter; the ActiveSet-resolve helper and dc-op stub have their
    // own increments, so scope the count to process_sample.
    let ps_start = find(&code, "pub fn process_sample", "process_sample");
    let ps_code = &code[ps_start..];
    let bumps = ps_code
        .matches("state.diag_nr_max_iter_count += 1;")
        .count();
    assert_eq!(
        bumps, 1,
        "process_sample must have exactly ONE diag_nr_max_iter_count \
         increment (the post-loop genuine-max-iter site); found {bumps}"
    );
    let gate = find(
        ps_code,
        "if state.last_nr_iterations >= MAX_ITER as u32 {",
        "the increment must be gated on genuine trap max-iter",
    );
    let bump = find(ps_code, "state.diag_nr_max_iter_count += 1;", "increment");
    assert!(
        bump > gate,
        "increment must sit inside the last_nr_iterations gate"
    );

    // (c) be_fallback counts ENTRY, not success: the single increment must
    // come before the BE NR loop, not inside `if be_converged`.
    let be_bumps = ps_code
        .matches("state.diag_be_fallback_count += 1;")
        .count();
    assert_eq!(
        be_bumps, 1,
        "process_sample must count BE fallback exactly once, at entry"
    );
    let be_bump = find(
        ps_code,
        "state.diag_be_fallback_count += 1;",
        "BE entry count",
    );
    let be_loop = find(
        ps_code,
        "// Rebuild RHS with backward Euler matrices",
        "BE fallback RHS rebuild",
    );
    assert!(
        be_bump < be_loop,
        "diag_be_fallback_count must increment at fallback ENTRY (before the \
         BE RHS rebuild), not on success only"
    );
}

#[test]
fn schur_diag_max_iter_gated_on_genuine_max_iter() {
    let code = nodal_code(CLIPPER);
    let ps_start = find(&code, "pub fn process_sample", "process_sample");
    let ps_code = &code[ps_start..];
    let gate = find(
        ps_code,
        "if state.last_nr_iterations >= MAX_ITER as u32 {",
        "Schur BE-fallback entry must gate the max-iter count",
    );
    let bump = find(ps_code, "state.diag_nr_max_iter_count += 1;", "increment");
    assert!(
        bump > gate,
        "Schur diag_nr_max_iter_count must only count genuine trap max-iter \
         (ActiveSetBe rail-engagement entries are not NR failures)"
    );
}

// ═══════════════════════════════════════════════════════════════════════
// Fix 9 + 10 + 11 — set_sample_rate semantics
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn set_sample_rate_same_rate_guarded_and_host_rate_recorded_first() {
    let code = nodal_code(CLIPPER_POT);

    // 9b/10: host rate recorded before the same-rate early return.
    let record = find(
        &code,
        "self.current_sample_rate = sample_rate;",
        "set_sample_rate must record the HOST rate",
    );
    let same_rate = find(
        &code,
        "if (sample_rate - SAMPLE_RATE).abs() < 0.5 {",
        "same-rate early return",
    );
    assert!(
        record < same_rate,
        "current_sample_rate must be assigned BEFORE the early return \
         (assign at {record}, early return at {same_rate}) — otherwise every \
         later pot rebuild runs at a stale rate after returning to the \
         codegen rate"
    );
    assert!(
        !code.contains("self.current_sample_rate = internal_rate;"),
        "the field stores the HOST rate (DK parity); internal-rate stores \
         double-scale every consumer that multiplies by OVERSAMPLING_FACTOR"
    );

    // 9a: the defaults fast path is guarded by an all-default check.
    let guard = find(
        &code,
        "let all_default = true",
        "same-rate defaults load must be guarded by pot/switch defaults",
    );
    assert!(
        guard > same_rate,
        "all_default guard belongs inside the same-rate branch"
    );
    find(
        &code,
        "&& (self.pot_0_resistance - ",
        "guard must compare the pot field against its nominal resistance",
    );

    // 10: consumers multiply the host rate up to the internal rate.
    find(
        &code,
        "state.rebuild_matrices(state.current_sample_rate * OVERSAMPLING_FACTOR as f64);",
        "lazy rebuild must convert host rate to internal rate",
    );

    // 11: rate-change path reseeds the DC blocker per-output via the shared
    // reseed helper (DC-OP value when the circuit has one, per-index 0.0
    // otherwise), never the blanket zero-array of the old emission.
    find(
        &code,
        "self.dc_block_x_prev[0] = ",
        "set_sample_rate must reseed dc_block_x_prev via the shared helper",
    );
    assert!(
        !code.contains("self.dc_block_x_prev = [0.0; NUM_OUTPUTS];"),
        "zeroing dc_block_x_prev turns the output DC bias into a full-scale \
         step through the blocker on the first post-rate-change sample"
    );
}

#[test]
fn current_sample_rate_initialized_to_host_rate() {
    // 2× oversampling: before the fix the field was initialized to
    // fs × OVERSAMPLING_FACTOR, silently double-scaling consumers.
    let code = nodal_code_with(CLIPPER_POT, |c| c.oversampling_factor = 2);
    let host = format!("current_sample_rate: {:.17e},", SR);
    let internal = format!("current_sample_rate: {:.17e},", SR * 2.0);
    find(
        &code,
        &host,
        "current_sample_rate must initialize to the HOST rate",
    );
    assert!(
        !code.contains(&internal),
        "current_sample_rate must not initialize to the internal (oversampled) rate"
    );
}

/// End-to-end for fix 9a: move a pot, then call `set_sample_rate` at the
/// codegen rate. The output afterwards must match a state that had the pot
/// moved and never touched `set_sample_rate` — before the fix the early
/// return loaded the nominal-pot `*_DEFAULT` matrices while the pot field
/// still claimed the moved value (matrices/fields desync until the next
/// knob move).
#[test]
fn set_sample_rate_same_rate_preserves_moved_pot() {
    let config = support::config_for_spice(CLIPPER_POT, SR);
    let (code, _n, _m) = support::generate_circuit_code_nodal(CLIPPER_POT, &config);

    let main_code = r#"
fn main() {
    let steps = 512usize;
    let amp = 0.5f64;

    // Reference: pot moved, no set_sample_rate call.
    let mut ref_state = CircuitState::default();
    ref_state.set_pot_0(2_000.0);
    let mut ref_out = Vec::with_capacity(steps);
    for _ in 0..steps { ref_out.push(process_sample(amp, &mut ref_state)[0]); }

    // Candidate: pot moved, then set_sample_rate at the codegen rate.
    let mut cand_state = CircuitState::default();
    cand_state.set_pot_0(2_000.0);
    cand_state.set_sample_rate(SAMPLE_RATE);
    let mut cand_out = Vec::with_capacity(steps);
    for _ in 0..steps { cand_out.push(process_sample(amp, &mut cand_state)[0]); }

    let mut max_diff = 0.0f64;
    for i in 0..steps {
        let d = (ref_out[i] - cand_out[i]).abs();
        if d > max_diff { max_diff = d; }
    }
    // Also sanity-check the pot actually matters at this drive level:
    // a nominal-pot state must NOT match the moved-pot reference.
    let mut nom_state = CircuitState::default();
    let mut nom_diff = 0.0f64;
    for i in 0..steps {
        let d = (ref_out[i] - process_sample(amp, &mut nom_state)[0]).abs();
        if d > nom_diff { nom_diff = d; }
    }
    println!("max_diff={max_diff:.3e}");
    println!("nom_diff={nom_diff:.3e}");
}
"#;

    let out = support::compile_and_run(&code, main_code, "ssr_pot_guard");
    let max_diff = out.parse_kv("max_diff").expect("main must print max_diff");
    let nom_diff = out.parse_kv("nom_diff").expect("main must print nom_diff");
    assert!(
        nom_diff > 1e-6,
        "test premise broken: pot position doesn't affect the output \
         (nom_diff={nom_diff:.3e}) — pick a stronger pot"
    );
    assert!(
        max_diff < 1e-12,
        "set_sample_rate at the codegen rate with a moved pot must not snap \
         matrices back to nominal (max_diff={max_diff:.3e}, pot sensitivity \
         nom_diff={nom_diff:.3e})"
    );
}

// ═══════════════════════════════════════════════════════════════════════
// Fix 6 — ActiveSetBe sub-step honesty (emission shape)
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn schur_active_set_be_substep_honest_convergence() {
    use melange_solver::codegen::OpampRailMode;
    // Op-amp with feedback clipping diodes: M=2, ActiveSetBe, Schur route.
    let spice = "\
ActiveSetBe Feedback Clipper
Rin in inv 10k
U1 0 inv out OA_TEST
Rfb out inv 100k
D1 inv out DCLIP
D2 out inv DCLIP
Rload out 0 10k
Cout out out_ac 1u
Rld2 out_ac 0 100k
.model OA_TEST OA(AOL=100000 ROUT=100 VCC=12 VEE=-12)
.model DCLIP D(IS=1e-14 N=1.9)
";
    let mut config = support::config_for_spice(spice, SR);
    config.opamp_rail_mode = OpampRailMode::ActiveSetBe;
    let (code, _n, _m) = support::generate_circuit_code_nodal(spice, &config);

    if !code.contains("Sub-step at 2× rate") {
        panic!(
            "test premise broken: ActiveSetBe circuit no longer emits the \
             Schur sub-step block — update the circuit or the marker"
        );
    }
    find(
        &code,
        "let mut sub_ok = true;",
        "sub-step recovery must track per-substep convergence",
    );
    let commit_gate = find(
        &code,
        "if sub_ok {",
        "sub-step result must only be committed when every fixed point settled",
    );
    let substep_count = find(
        &code,
        "state.diag_substep_count += 1;",
        "substep diag count",
    );
    assert!(
        substep_count > commit_gate,
        "diag_substep_count must only count successful sub-step recoveries"
    );
    assert!(
        !code.contains("let mut nr_ok = true;\n                for i in 0..M"),
        "the fixed-point convergence flag must carry the FINAL iteration's \
         status out of the loop (declared before the loop, not per-iteration)"
    );
}

// ═══════════════════════════════════════════════════════════════════════
// Fix 12 + 13 — denormal bias, NaN-honest predicates
// ═══════════════════════════════════════════════════════════════════════

#[test]
fn dc_blocker_feedback_carries_denormal_bias() {
    for (tag, spice) in [("schur", CLIPPER), ("full_lu", CLIPPER_FULL_LU)] {
        let code = nodal_code(spice);
        assert!(
            code.contains("state.dc_block_y_prev[out_idx] = dc_blocked + 1e-20;"),
            "{tag}: DC blocker feedback must carry the DK denormal bias"
        );
        assert!(
            !code.contains("state.dc_block_y_prev[out_idx] = dc_blocked;"),
            "{tag}: unbiased blocker feedback denormalizes during long silences"
        );
    }
}

#[test]
fn full_lu_convergence_predicates_treat_nan_as_not_converged() {
    let code = nodal_code(CLIPPER_FULL_LU);
    assert!(
        code.contains("if !(step.abs() < threshold)"),
        "voltage-step convergence predicate must be NaN-honest (negated form)"
    );
    assert!(
        !code.contains("if step.abs() >= threshold"),
        "positive-comparison form reads a NaN step as converged"
    );
    assert!(
        code.contains("if !(r <= tol)"),
        "residual predicate must be NaN-honest (negated form)"
    );
}
