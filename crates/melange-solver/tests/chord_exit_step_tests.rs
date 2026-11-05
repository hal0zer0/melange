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
//! A chord iteration that fails the device current-change check has shown
//! its chord is stale where the solve now is (after the sample's first
//! step, which moves the currents with the input whatever the chord), so
//! the next iteration is exact too and counts as the exit step.
//!
//! Witness: the diode node of a single-supply overdrive, 384 kHz, 0.1 V drive.
//! The op-amp swings rail to rail between plateaus, and the full-LU loop
//! accepted chord steps through the swing. Measured under the charge form:
//! 1.00 µA residual on the accepted samples with chord acceptance (both
//! exact-step paths removed), 0.240 µA with them, the same as the Schur
//! sub-path, whose M-dimensional Newton refactors every iteration (with its
//! current-residual check; 0.390 µA before Schur had one).

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
// A failed current-change check on a chord iteration after the first makes
// the next iteration exact (and the exit step).
const STALE: &str = "if !need_refactor && iter > 0 { exit_step = true; }";
// The KCL infinity norm against the row test's own floor, by name
// (`thresholds::KCL_ABSTOL`, 1e-9 A).
const GATE: &str = "kcl_inf > KCL_ABSTOL &&";

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
    assert_eq!(full_lu.matches(STALE).count(), 2);
    let schur = code_for(NodalSubPathOverride::Schur);
    assert!(
        !schur.contains("exit_step"),
        "the Schur Newton refactors every iteration"
    );

    let r_full_lu = residual(&full_lu, "exit_full_lu");
    let r_schur = residual(&schur, "exit_schur");
    let r_mutant = residual(
        &full_lu
            .replace(EXIT, "if false && converged_check")
            .replace(STALE, "if false { exit_step = true; }"),
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

/// An emitter follower at a 10 V bias whose emitter a large capacitor holds:
/// over a sample the base-emitter voltage follows the input, so the sample's
/// first step moves the collector current past the current-change check's
/// RELTOL while every node steps inside its own RELTOL·|v|, and the chord
/// stays good at this drive (measured, 10 mV at 220 Hz: the rule applied
/// from the first iteration refactors 0.94 times per sample, from the
/// second 0.044).
const FOLLOWER: &str = "capacitor-held emitter follower at 10 V\n\
Vcc vcc 0 DC 20\nRb1 vcc b 1k\nRb2 b 0 1k\nCin in b 100u\nQ1 vcc b e QN\n\
Re e 0 10k\nCe e 0 100u\nRo e out 1k\nRl out 0 100k\n\
.model QN NPN(IS=1e-14 BF=200)\n";

/// Refactors per sample over 0.25 s of a quiet 220 Hz sine at 48 kHz.
fn quiet_sine_refactors(code: &str, tag: &str) -> f64 {
    let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for i in 0..12000usize {
        let x = 0.01 * (2.0 * std::f64::consts::PI * 220.0 * i as f64 / 48000.0).sin();
        let _ = process_sample(x, &mut s);
    }
    println!(\"refactors={}\", s.diag_refactor_count as f64 / 12000.0);
}";
    support::compile_and_run(code, main, tag)
        .parse_kv("refactors")
        .unwrap()
}

/// The first step of a sample moves the device currents with the input
/// whatever the chord, so a failed current-change check THERE is no evidence
/// of a stale chord: the stale-chord rule starts at the second iteration. On
/// a quietly driven deck the sample's first check fails on most samples and
/// the chord's second iteration certifies; applying the rule from the first
/// iteration refactored every sample for nothing (a bus compressor +51 % per
/// sample at 0.3 V, the same iterations).
#[test]
fn the_stale_chord_rule_skips_the_samples_first_step() {
    let mut config = support::config_for_spice(FOLLOWER, 48000.0);
    config.nodal_sub_path_override = NodalSubPathOverride::FullLu;
    let code = support::generate_circuit_code_nodal(FOLLOWER, &config).0;
    assert_eq!(code.matches(STALE).count(), 2);
    let gated = quiet_sine_refactors(&code, "stale_quiet");
    let first = quiet_sine_refactors(
        &code.replace(STALE, "if !need_refactor { exit_step = true; }"),
        "stale_quiet_first_step",
    );
    eprintln!("refactors per sample, quiet sine: rule from the second iteration {gated:.4}, from the first {first:.4}");
    assert!(
        gated < 0.05,
        "the rule refactors {gated} per sample on a quiet sine"
    );
    assert!(
        first > 0.5,
        "the witness no longer fails the first check on quiet signal ({first} per sample)"
    );
}
