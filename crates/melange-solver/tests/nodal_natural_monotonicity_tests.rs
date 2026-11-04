//! The full-LU Newton sites' line search accepts a trial by Deuflhard's
//! natural monotonicity test — `‖G_aug⁻¹·F(trial)‖_w <= (1 − NATURAL_TEST_SLOPE·λ)·‖d‖_w`
//! in the step's own factorization, weights frozen per iteration from the
//! node-step test's tolerance — not by the residual-norm Armijo condition,
//! which the sub-step ladder keeps (it solves in place and has no
//! factorization to back-solve with).
//!
//! Witness (2026-11-04, a FET limiter's output Darlington with its C-E
//! breakdown `D(BV=60)` driven by a 31.6 V square, forced full-LU): the
//! Armijo test on `‖F‖` in raw amps backtracked the base-emitter junction
//! limiter's 2.5·vt step to 1/4..1/64 on every iteration, because on the
//! collector node two ~7 mA exponentials (the collector junction and the
//! breakdown branch) cancel to a 0.3 mA residual and either one's
//! second-order term outweighs the residual's linear decrease; 48 / 83 / 31
//! samples per 0.4 s hit the ceiling at 1x / 2x / 4x where the Schur
//! sub-path, which has no search, converged in 16 iterations. Under the
//! natural test: 0 / 0 / 0, the sample converges in 17. The deck is a
//! product deck, so this file pins the emitted code; the corpus counters
//! and the witness numbers are in CODEGEN.md.

mod support;

use melange_solver::codegen::NodalSubPathOverride;

/// Choke-loaded MOSFET stage: routes nodal full-LU on its own (the ISAT
/// inductor); a trapezoidal build, so it carries the trapezoidal Newton
/// site, its backward-Euler instance and the sub-step ladder.
const MOSFET_CHOKE_LOAD: &str = "MOSFET Choke-Loaded Gain Stage
.model 2N7000 NMOS(KP=0.1 VTO=2.1 LAMBDA=0.01 GAMMA=0.5 PHI=0.6)
VCC vcc 0 DC 24
Cin  in     gate_i 1u
Rb1  vcc    gate_i 1MEG
Rb2  gate_i 0      170k
Rg   gate_i gate   1k
Lp   vcc    drain  5 ISAT=20m
Rdcr drain  drain_d 120
Cw   vcc    drain  220p
Rw   vcc    drain  470k
M1   drain_d gate  src  0  2N7000
Rs   src    0      220
Cs   src    0      100u
Cout drain  out    100n
Rl   out    0      100k
.END
";

/// A diode clipper with a bias source: M = 1, nodal on request, Schur-able.
const DIODE_CLIPPER: &str = "Diode clipper behind a source
.model DS D(IS=1e-12 N=1.2)
Rin in a 1k
D1 a out DS
Rload out 0 1k
Cout out 0 100n
VB bias 0 DC 1
Rb bias a 100k
.end
";

fn build(spice: &str, output: &str, solver: &str, sub: NodalSubPathOverride) -> String {
    let mut config = support::config_in_out_or_node1(spice, 48000.0);
    let netlist = support::parse_expanded(spice);
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).expect("MNA");
    let names = mna.node_names_in_index_order();
    let idx = names
        .iter()
        .position(|n| *n == output)
        .unwrap_or_else(|| panic!("no node {output}"))
        - 1;
    config.output_nodes = vec![idx];
    support::try_build_shipped_with(spice, &config, solver, |o| {
        o.nodal_sub_path_override = sub;
    })
    .unwrap_or_else(|e| panic!("build refused: {e}"))
    .generated
    .code
}

/// The text of one emitted function, from its signature to the first line
/// that is a bare `}`.
fn body<'a>(code: &'a str, sig: &str) -> &'a str {
    let start = code.find(sig).unwrap_or_else(|| panic!("no `{sig}`"));
    let rest = &code[start..];
    let end = rest.find("\n}\n").expect("function end");
    &rest[..end]
}

#[test]
fn the_newton_sites_test_natural_monotonicity_in_their_own_factorization() {
    let code = build(MOSFET_CHOKE_LOAD, "out", "auto", NodalSubPathOverride::Auto);
    assert!(
        code.contains("full-lu"),
        "the witness deck routes nodal full-LU"
    );
    // The constants are emitted by name, with the ruled values.
    assert!(code.contains("pub const NATURAL_TEST_SLOPE: f64 = 0.25;"));
    assert!(code.contains("pub const NATURAL_TEST_CURRENT_FLOOR: f64 = 1e-9;"));
    // The residual VECTOR helper, same rows as the scalar helpers.
    let vec_body = body(&code, "fn kcl_residual_inl_vec(");
    assert!(vec_body.contains("// KCL rows (A)"));
    assert!(
        vec_body.contains("fres[0] = acc;"),
        "every row writes its residual"
    );
    for scalar in ["fn kcl_residual(", "fn kcl_residual_inl("] {
        assert!(
            !body(&code, scalar).contains("fres[0] = acc;"),
            "{scalar} stays scalar"
        );
    }
    // Both Newton sites (trapezoidal and its backward-Euler instance) test a
    // trial by the back-solved residual in their own chord factorization.
    let accept = "if nb.sqrt() <= (1.0 - nm_slope * (alpha * s)) * nd {";
    assert_eq!(
        code.matches(accept).count(),
        2,
        "one natural test per Newton site"
    );
    // The slope is the ruled constant on a fresh factorization and 0
    // (non-expansion) on a chord iteration.
    assert_eq!(
        code.matches("let nm_slope = if need_refactor { NATURAL_TEST_SLOPE } else { 0.0 };")
            .count(),
        2
    );
    // The step test runs before the search and gates it: no search on an
    // iteration whose correction is inside the certification tolerance.
    assert_eq!(
        code.matches("        if max_step_exceeded {\n        // Natural monotonicity test")
            .count(),
        2
    );
    assert!(!code.contains("nd_inf"));
    assert!(code.contains("lu_back_solve(&state.chord_lu, &state.chord_dr, &state.chord_dc, &state.chord_perm, &mut fv);"));
    assert!(code.contains("lu_back_solve(&state.chord_be_lu, &state.chord_be_dr, &state.chord_be_dc, &state.chord_be_perm, &mut fv);"));
    // Weights: node rows with VNTOL, branch rows with the current floor,
    // computed BEFORE the backtracking loop and never inside it.
    for site in code.split(accept).take(2) {
        let weights = site
            .rfind("let mut w = [0.0f64; N];")
            .expect("weights declared");
        let search = site
            .rfind("while s >= (1.0 / 1024.0) {")
            .expect("search loop");
        assert!(weights < search, "weights are frozen before the trials");
        let inside = &site[search..];
        assert!(
            !inside.contains("w[i] = "),
            "no weight is recomputed at a trial"
        );
        let before = &site[weights..search];
        assert!(before.contains("+ VNTOL); let d ="));
        assert!(before.contains("+ NATURAL_TEST_CURRENT_FLOOR); let d ="));
    }
    // The Newton sites no longer run the residual-norm Armijo test; each
    // sub-step ladder (one per Newton site) still does.
    let ladders = code
        .matches("// Adaptive sub-stepping: local refinement")
        .count();
    assert!(ladders >= 1);
    assert_eq!(
        code.matches("if rc <= (1.0 - 1e-4 * s) * r0 { accepted = true; break; }")
            .count(),
        ladders,
        "the ladders keep the Armijo condition"
    );
    assert!(!code.contains("if rc.0 <= (1.0 - 1e-4 * s) * r0"));
    // This deck's saturating inductor keeps the device re-evaluation after
    // the step, so its trials read `kcl_residual_vec`; the canary counts on
    // acceptance only.
    assert_eq!(code.matches("kcl_residual_vec(&vc,").count(), 2);
    assert!(code.contains("pub diag_ls_merit_up_count: u64,"));
    assert_eq!(
        code.matches("if rc > r0 { state.diag_ls_merit_up_count += 1; }")
            .count(),
        2
    );
    assert!(
        !code.contains("ls_fv"),
        "no correction reuse without Jacobian reuse"
    );
}

#[test]
fn a_reuse_site_keeps_the_accepted_trial_and_its_residual() {
    // A junction-only deck forced to full-LU: the accepted trial's device
    // currents, Jacobian and residual tuple are kept for the step it commits
    // (`armijo_trial_reuse`), with the residual vector beside them.
    let code = build(DIODE_CLIPPER, "out", "nodal", NodalSubPathOverride::FullLu);
    assert!(code.contains("full-lu"));
    assert_eq!(
        code.matches(
            "kcl_residual_eval_jac(&vc, &rhs, &state.a, state, &mut ls_i_nl, &mut j_dev, &mut fv)"
        )
        .count(),
        1
    );
    assert_eq!(code.matches("ls_res = rc; ls_fv = fv; break;").count(), 2);
    assert_eq!(
        code.matches("if rc.0 > r0 { state.diag_ls_merit_up_count += 1; }")
            .count(),
        2
    );
    assert!(!code.contains("fn kcl_residual_vec("));
    // The iteration after an accepted trial, factorization kept, takes the
    // trial's back-solved residual as its correction; a debug build checks
    // it against the companion solve.
    assert_eq!(
        code.matches("if ls_prev_hit && !need_refactor {").count(),
        2
    );
    assert_eq!(
        code.matches("for i in 0..N { v_new[i] = v[i] - ls_fv[i]; }")
            .count(),
        2
    );
    assert_eq!(
        code.matches("if cfg!(debug_assertions) && ls_prev_hit && !need_refactor {")
            .count(),
        2
    );
    assert!(code.contains("pub const NATURAL_REUSE_DEBUG_RELTOL: f64 = 1e-8;"));
}

/// A clipping common-emitter stage behind a 12 V supply: a junction-only
/// full-LU build whose reused corrections (the iteration after an accepted
/// trial) swing by volts at the clip edges, so the debug invariant below is
/// exercised far above its bound.
const CLIPPING_CE_STAGE: &str = "Clipping common-emitter stage
.model QN NPN(IS=1e-14 BF=200)
VCC vcc 0 DC 12
Rin in b 2.2k
Rb vcc b 220k
Q1 c b e QN
Rc vcc c 4.7k
Re e 0 100
Ce e 0 47u
Cout c out 1u
Rl out 0 47k
.end
";

/// A debug build runs the reused correction beside the companion solve and
/// asserts they agree to `NATURAL_REUSE_DEBUG_RELTOL`: compiled with debug
/// assertions on (`support` compiles with `-O`) and rendered 0.5 s at 3 V
/// into the clip. The render also records the largest reused correction, so
/// the test knows the invariant was exercised on corrections of volts, not
/// on the near-root rounding noise the bound is blind to.
#[test]
fn a_debug_build_holds_the_reused_correction_invariant() {
    let code = build(
        CLIPPING_CE_STAGE,
        "out",
        "nodal",
        NodalSubPathOverride::FullLu,
    );
    assert!(code.contains("if ls_prev_hit && !need_refactor {"));
    // Record the largest reused correction (test-only instrumentation).
    let code = code.replace(
        "for i in 0..N { v_new[i] = v[i] - ls_fv[i]; }",
        "for i in 0..N { v_new[i] = v[i] - ls_fv[i]; let a = ls_fv[i].abs(); if a > unsafe { MAX_REUSED } { unsafe { MAX_REUSED = a; } } }",
    );
    let main = "static mut MAX_REUSED: f64 = 0.0;
fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let mut iters = 0u64;
    for i in 0..24000usize {
        let x = 3.0 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin();
        let _ = process_sample(x, &mut s);
        iters += s.last_nr_iterations as u64;
    }
    println!(\"{} {:e}\", iters, unsafe { MAX_REUSED });
}";
    let dir = support::scratch_dir();
    let src = dir.join("natural_reuse_debug.rs");
    let bin = dir.join("natural_reuse_debug");
    std::fs::write(&src, format!("{code}\n{main}\n")).unwrap();
    let out = std::process::Command::new("rustc")
        .arg(&src)
        .arg("-o")
        .arg(&bin)
        .arg("--edition=2024")
        .arg("-C")
        .arg("debug-assertions=on")
        .arg("-A")
        .arg("warnings")
        .output()
        .expect("rustc");
    assert!(
        out.status.success(),
        "debug compile failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let run = std::process::Command::new(&bin).output().expect("run");
    assert!(
        run.status.success(),
        "the debug invariant fired:\n{}",
        String::from_utf8_lossy(&run.stderr)
    );
    let stdout = String::from_utf8_lossy(&run.stdout);
    let mut it = stdout.split_whitespace();
    let iters: u64 = it.next().unwrap().parse().unwrap();
    let max_reused: f64 = it.next().unwrap().parse().unwrap();
    assert!(iters > 24000, "the witness must iterate: {iters}");
    // Volts of reused correction: the invariant was tested where a wrong
    // vector would show, not only at the root.
    assert!(
        max_reused > 1e-2,
        "largest reused correction {max_reused:e} V is not above the bound's reach"
    );
}

#[test]
fn a_schur_build_carries_neither_the_test_nor_its_constants() {
    let code = build(DIODE_CLIPPER, "out", "nodal", NodalSubPathOverride::Schur);
    assert!(code.contains("schur"), "routes Schur");
    assert!(!code.contains("NATURAL_TEST"));
    assert!(!code.contains("fn kcl_residual_inl_vec("));
    assert!(!code.contains("diag_ls_merit_up_count"));
    // Its ladder keeps the Armijo condition.
    assert!(code.contains("if rc <= (1.0 - 1e-4 * s) * r0 { accepted = true; break; }"));
}
