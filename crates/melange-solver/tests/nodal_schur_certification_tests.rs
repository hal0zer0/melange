//! The nodal Schur Newton certifies an iterate on the device-voltage step
//! AND the dense DK Newton's current-residual check.
//!
//! The Schur sub-path solves the M-dimensional device system
//! `f(i) = i_nl − i_dev(p + K·i_nl) = 0`; its linear rows are exact by
//! construction (`v = v_pred + S_NI·i_nl`), so `f` is the whole equation
//! residual. It used to certify on the undamped device-voltage step alone,
//! a window of `RELTOL·|v_d|` that is wide where a junction law is steep at
//! a large voltage, and it could certify a sample on the warm start's first
//! step with `f` far over tolerance. The dense DK Newton has always also
//! required `|f_i| <= RELTOL·max(|i_nl|, |i_dev|, ABSTOL_SCALE_FLOOR) +
//! ABSTOL`; the Schur Newton now applies the same check, verbatim, from the
//! residual its iteration already formed (no extra device evaluation).
//!
//! Witnesses: a FET limiter's driven line amp forced to Schur sat −66 dB
//! from a converged solve at +24 dB drive with flyback edges landing volts
//! off (with the check −153 dB), and the corpus's Schur decks moved toward
//! their converged references or stayed. Here: a zener clamp driven past its
//! knee, 1.5 mV from a converged render on the step test alone, 1.3e-8 V
//! with the check.

mod support;

use melange_solver::codegen::{NodalSubPathOverride, OpampRailMode};

const OVERDRIVE: &str = "single-supply op-amp overdrive into a diode clipper\n\
Vcc vcc 0 DC 9\nR_b1 vcc vbias 100k\nR_b2 vbias 0 100k\nC_b vbias 0 10u\n\
C_in in np 100n\nR_in np vbias 1Meg\nU1 np nm oa OPA\nR_f oa nm 500k\n\
R_g nm ng 4.7k\nC_g ng 0 10u\nC_c oa n1 1u\nR_1 n1 n2 1k\nD_1 n2 0 DSIG\n\
D_2 0 n2 DSIG\nR_t n2 n3 10k\nC_t n3 0 22n\nC_o n3 out 1u\nR_v out 0 100k\n\
.model OPA OA(AOL=200000 VCC=9 VEE=0)\n.model DSIG D(IS=2.52n N=1.752)\n";

const FS: f64 = 48000.0;

/// The certification line the check adds to the Schur Newton.
const GATED: &str = "if !any_limited && residual_ok {";

fn code_for(sub_path: NodalSubPathOverride) -> String {
    let mut config = support::config_for_spice(OVERDRIVE, FS);
    config.nodal_sub_path_override = sub_path;
    config.opamp_rail_mode = OpampRailMode::ActiveSet;
    support::generate_circuit_code_nodal(OVERDRIVE, &config).0
}

/// The check is emitted on every Schur Newton site, reads the residual the
/// iteration formed and adds no device evaluation.
#[test]
fn schur_newton_carries_the_dk_current_residual_check() {
    let schur = code_for(NodalSubPathOverride::Schur);
    assert!(!schur.contains("state.chord_lu"), "not a Schur build");
    let sites = schur.matches(GATED).count();
    assert!(
        sites >= 2,
        "trapezoidal and backward-Euler Newton: {sites} sites"
    );
    // M = 2 (two diodes): two residual lines per site, the DK formula verbatim.
    let line = "{ let i_thr = RELTOL * i_nl[0].abs().max(i_dev0.abs()).max(ABSTOL_SCALE_FLOOR) + ABSTOL; if !(f0.abs() <= i_thr) { residual_ok = false; } }";
    assert_eq!(schur.matches(line).count(), sites);
    // No extra device evaluation: the mutant without the check has the same
    // number of diode-current evaluations.
    let mutant = schur.replace(GATED, "if !any_limited {");
    assert_eq!(
        schur.matches("diode_current(").count(),
        mutant.matches("diode_current(").count()
    );
}

/// A zener clamp driven 2x past its knee: the clamped diode sits at
/// `v_d ≈ −BV`, where the device-voltage step test admits `RELTOL·BV` = 10 mV
/// (0.4 Vt of an `N = 1` breakdown branch, ~40 % of its current).
const ZENER: &str = "zener clamp\nR1 in z 1k\nD1 0 z DZ\nC1 z 0 10n\nR2 z out 1k\nRl out 0 100k\n\
.model DZ D(IS=1e-14 N=1 BV=10 IBV=1e-3)\n";

/// The same build with the certification constants 1e4 times tighter: a
/// converged reference for the shipped one.
fn tightened(code: &str) -> String {
    let mut out = String::new();
    for line in code.lines() {
        let l = if line.starts_with("pub const RELTOL: f64 =") {
            "pub const RELTOL: f64 = 1e-7;".to_string()
        } else if line.starts_with("pub const VNTOL: f64 =") {
            "pub const VNTOL: f64 = 1e-10;".to_string()
        } else if line.starts_with("pub const ABSTOL: f64 =") {
            "pub const ABSTOL: f64 = 1e-16;".to_string()
        } else {
            line.to_string()
        };
        out.push_str(&l);
        out.push('\n');
    }
    out
}

/// The output over 20 ms of a 20 V, 1 kHz sine at 48 kHz, one value per line.
fn render(code: &str, tag: &str) -> Vec<f64> {
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for i in 0..960usize {{
        let y = process_sample(20.0 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin(), &mut s)[0];
        println!(\"{{:e}}\", y);
    }}
    assert_eq!({unsolved}, 0, \"unsolved samples\");
}}",
        unsolved = support::unsolved_expr(code, "s"),
    );
    support::compile_and_run(code, &main, tag).parse_samples()
}

fn max_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// Behavioural pin: against a converged reference, the Schur render with the
/// check is far closer than the render certified on the voltage step alone.
#[test]
fn schur_certification_converges_the_zener_clamp() {
    let mut config = support::config_for_spice(ZENER, 48000.0);
    config.nodal_sub_path_override = NodalSubPathOverride::Schur;
    let code = support::generate_circuit_code_nodal(ZENER, &config).0;
    assert!(code.contains(GATED), "not a Schur build with the check");
    let reference = render(&tightened(&code), "zener_tight");
    let shipped = render(&code, "zener_shipped");
    let mutant = render(&code.replace(GATED, "if !any_limited {"), "zener_step_only");
    let (d_shipped, d_mutant) = (
        max_diff(&shipped, &reference),
        max_diff(&mutant, &reference),
    );
    eprintln!(
        "max |out - converged|: with the check {d_shipped:.3e} V, step test only {d_mutant:.3e} V"
    );
    assert!(
        d_mutant > 1e-4,
        "the witness no longer shows the step-only certification ({d_mutant:e} V)"
    );
    assert!(
        d_shipped < 1e-6,
        "{d_shipped:e} V from the converged render (step test only: {d_mutant:e} V)"
    );
}
