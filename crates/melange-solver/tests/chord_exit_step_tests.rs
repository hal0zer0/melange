//! A full-LU Newton solve accepts only an exact-Jacobian step.
//!
//! The full-LU loop reuses a factored Jacobian (the chord) across iterations
//! and samples. A chord step leaves a KCL residual `(J - J_chord)·Δ`, first
//! order in the step. The node-step test cannot see it: on a stiff junction
//! row the node tolerance is tens of µA of current. So a chord-accepted
//! iterate takes one more Newton step, refactored at the accepted point,
//! when its node residual is above the row test's absolute floor (1e-9 A).
//! Below that floor the step would change nothing, so on quiet signal the
//! chord keeps its reuse across samples.
//!
//! Witness: the diode node of a single-supply overdrive, 384 kHz, 0.1 V drive.
//! The op-amp swings rail to rail between plateaus, and the full-LU loop
//! accepted chord steps through the swing. Measured under the charge form:
//! 1.09 µA residual on the accepted samples with chord acceptance, 0.390 µA
//! with the exit step, the same as the Schur sub-path, whose M-dimensional
//! Newton refactors every iteration.

mod support;

use melange_solver::codegen::{NodalSubPathOverride, OpampRailMode};

const OVERDRIVE: &str = "single-supply op-amp overdrive into a diode clipper\n\
Vcc vcc 0 DC 9\nR_b1 vcc vbias 100k\nR_b2 vbias 0 100k\nC_b vbias 0 10u\n\
C_in in np 100n\nR_in np vbias 1Meg\nU1 np nm oa OPA\nR_f oa nm 500k\n\
R_g nm ng 4.7k\nC_g ng 0 10u\nC_c oa n1 1u\nR_1 n1 n2 1k\nD_1 n2 0 DSIG\n\
D_2 0 n2 DSIG\nR_t n2 n3 10k\nC_t n3 0 22n\nC_o n3 out 1u\nR_v out 0 100k\n\
.model OPA OA(AOL=200000 VCC=9 VEE=0)\n.model DSIG D(IS=2.52n N=1.752)\n";

const FS: f64 = 384000.0;

const EXIT: &str = "if converged_check && !need_refactor && !exit_step";
const GATE: &str = "kcl_inf > 1e-9 &&";

fn code_for(sub_path: NodalSubPathOverride) -> String {
    let mut config = support::config_for_spice(OVERDRIVE, FS);
    config.nodal_sub_path_override = sub_path;
    config.opamp_rail_mode = OpampRailMode::ActiveSet;
    support::generate_circuit_code_nodal(OVERDRIVE, &config).0
}

/// max |KCL residual at n2| over the last 0.1 s of a 0.5 s, 1 kHz, 0.1 V render.
fn residual(code: &str, tag: &str) -> f64 {
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    let n = {n}usize;
    let tail = n - {w}usize;
    let (is, nvt) = (s.device_0_is, s.device_0_n_vt);
    let mut worst = 0.0f64;
    for i in 0..n {{
        let _ = process_sample(0.1 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / {FS:?}).sin(), &mut s);
        if i >= tail {{
            let v = s.v_prev;
            let x = v[NODE_N2];
            let id = is * ((x / nvt).exp() - 1.0) - is * ((-x / nvt).exp() - 1.0);
            let r = (v[NODE_N1] - x) / 1e3 - (x - v[NODE_N3]) / 1e4 - id;
            worst = worst.max(r.abs());
        }}
    }}
    assert_eq!({unsolved}, 0, \"unsolved samples\");
    println!(\"residual={{:e}}\", worst);
}}",
        n = (0.5 * FS) as usize,
        w = (0.1 * FS) as usize,
        unsolved = support::unsolved_expr(code, "s"),
    );
    support::compile_and_run(code, &main, tag)
        .parse_kv("residual")
        .unwrap()
}

#[test]
fn full_lu_exits_on_an_exact_jacobian_step() {
    let full_lu = code_for(NodalSubPathOverride::FullLu);
    assert!(full_lu.contains("state.chord_lu"), "not a full-LU build");
    // The trapezoidal loop and the backward-Euler instance both carry it.
    assert_eq!(full_lu.matches(EXIT).count(), 2);
    assert_eq!(full_lu.matches(GATE).count(), 2);
    let schur = code_for(NodalSubPathOverride::Schur);
    assert!(
        !schur.contains("exit_step"),
        "the Schur Newton refactors every iteration"
    );

    let r_full_lu = residual(&full_lu, "exit_full_lu");
    let r_schur = residual(&schur, "exit_schur");
    let r_mutant = residual(
        &full_lu.replace(EXIT, "if false && converged_check"),
        "exit_mutant",
    );
    eprintln!(
        "n2 residual: full-LU {r_full_lu:.3e} A, Schur {r_schur:.3e} A, chord-accepting mutant {r_mutant:.3e} A"
    );
    assert!(
        r_mutant > 2.0 * r_schur,
        "the witness no longer sees chord acceptance ({r_mutant:e} A against Schur {r_schur:e} A)"
    );
    assert!(
        r_full_lu < 1.2 * r_schur,
        "full-LU {r_full_lu:e} A against Schur {r_schur:e} A"
    );
}

/// Refactors per sample over 0.25 s of silence at 48 kHz.
fn silent_refactors(code: &str, tag: &str) -> f64 {
    let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for _ in 0..12000usize { let _ = process_sample(0.0, &mut s); }
    println!(\"refactors={}\", s.diag_refactor_count as f64 / 12000.0);
}";
    support::compile_and_run(code, main, tag)
        .parse_kv("refactors")
        .unwrap()
}

/// At silence the accepted point carries no residual, so the gate keeps the
/// chord's reuse across samples; forcing the gate open refactors every sample.
#[test]
fn the_exit_gate_keeps_the_chord_at_silence() {
    let mut config = support::config_for_spice(OVERDRIVE, 48000.0);
    config.nodal_sub_path_override = NodalSubPathOverride::FullLu;
    config.opamp_rail_mode = OpampRailMode::ActiveSet;
    let code = support::generate_circuit_code_nodal(OVERDRIVE, &config).0;
    let gated = silent_refactors(&code, "gate_silence");
    let open = silent_refactors(&code.replace(GATE, "true &&"), "gate_open_silence");
    eprintln!("refactors per sample at silence: gated {gated:.4}, gate forced open {open:.4}");
    assert!(
        gated < 0.01,
        "gated build refactors {gated} per sample at silence"
    );
    assert!(
        open > 0.9,
        "the mutant no longer shows the gate ({open} per sample)"
    );
}
