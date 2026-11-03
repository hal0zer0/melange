//! The full-LU Newton loop's work-saving transforms change no output bit.
//!
//! - The chord refactor builds G_aug only at the positions the sparse factor
//!   and back-solve touch (`sparse_lu_touch_pattern`), and the dense fallback
//!   rebuilds the full matrix instead of restoring a saved copy. Witness:
//!   filling the whole chord storage with NaN before every such build changes
//!   nothing, with a dense fallback forced on every third refactor so the
//!   sparse path also runs on storage a dense factorization left behind.
//! - The accepted Armijo trial's device currents, Jacobian and residual stand
//!   in for the device-chord check's, the KCL gate's and the next iteration's
//!   (`armijo_trial_reuse` and the two predicates after it in `residual.rs`).
//!   Witness: a build whose `ls_hit` is never set (every one of those
//!   evaluations runs again) renders the same bits.
//! - Which builds take the reuse, and how much of it.
//! - With the structural pattern the sparse factor works on packed storage
//!   (`LU_SLOTS` slots, the touch pattern) with the static row swaps folded
//!   into its indices; the N x N array is left to the dense fallback.

mod support;

use melange_solver::codegen::{NodalSubPathOverride, OpampRailMode};

/// Three CE stages behind a 15-section RC ladder, into an antiparallel diode
/// clipper, with three feedback resistors: N = 29 with 8 device dimensions,
/// sparse enough (and N >= 20) for the structural equilibration pattern, so
/// the sparse schedule runs on the touched-position build. The feedback paths
/// make the elimination fill positions outside the input pattern, which the
/// build must initialise too.
const BJT_LADDER: &str = "three-stage BJT amp behind an RC ladder, into a diode clipper
VCC vcc 0 DC 12
R_a1 in a1 1k
C_a1 a1 0 1n
R_a2 a1 a2 1k
C_a2 a2 0 1n
R_a3 a2 a3 1k
C_a3 a3 0 1n
R_a4 a3 a4 1k
C_a4 a4 0 1n
R_a5 a4 a5 1k
C_a5 a5 0 1n
R_a6 a5 a6 1k
C_a6 a6 0 1n
R_a7 a6 a7 1k
C_a7 a7 0 1n
R_a8 a7 a8 1k
C_a8 a8 0 1n
R_a9 a8 a9 1k
C_a9 a9 0 1n
R_a10 a9 a10 1k
C_a10 a10 0 1n
R_a11 a10 a11 1k
C_a11 a11 0 1n
R_a12 a11 a12 1k
C_a12 a12 0 1n
R_a13 a12 a13 1k
C_a13 a13 0 1n
R_a14 a13 a14 1k
C_a14 a14 0 1n
R_a15 a14 a15 1k
C_a15 a15 0 1n
C_in a15 b1 1u
R11 vcc b1 100k
R12 b1 0 22k
RC1 vcc c1 4.7k
RE1 e1 0 1k
CE1 e1 0 47u
Q1 c1 b1 e1 QN
C12 c1 b2 1u
R21 vcc b2 100k
R22 b2 0 22k
RC2 vcc c2 4.7k
RE2 e2 0 1k
CE2 e2 0 47u
Q2 c2 b2 e2 QN
C23 c2 b3 1u
R31 vcc b3 100k
R32 b3 0 22k
RC3 vcc c3 4.7k
RE3 e3 0 1k
CE3 e3 0 47u
Q3 c3 b3 e3 QN
C3o c3 d1 1u
R_d d1 out 1k
D1 out 0 DS
D2 0 out DS
C_out out 0 10n
R_l out 0 100k
R_fb c3 b1 1Meg
R_fb2 out e2 220k
R_fb3 c2 a8 470k
.model QN NPN(IS=1e-14 BF=200 BR=1 VAF=100)
.model DS D(IS=2.52n N=1.752)
.END";

/// Two triode stages: the tube's body and `_final` helpers differ at the
/// plate-current cutoff, so only the trial-for-check reuse applies.
const TWO_TUBE_STAGE: &str = "\
Two tube preamp
R1 in grid1 100k
Rg1 grid1 0 1Meg
Ra1 vcc plate1 100k
Rk1 cathode1 0 1.5k
Ck1 cathode1 0 25u
C1 plate1 grid2 100n
Rg2 grid2 0 470k
Ra2 vcc plate2 100k
Rk2 cathode2 0 1.5k
Ck2 cathode2 0 25u
Cout plate2 out 100n
Rout out 0 100k
T1 grid1 plate1 cathode1 12AX7
T2 grid2 plate2 cathode2 12AX7
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300 CCG=1.6p CGP=1.7p CCP=0.46p)
VCC vcc 0 DC 250
Cin in 0 100p
.END";

/// An op-amp gain stage into a diode clipper (for the `Hard` rail mode gate).
const OPAMP_CLIPPER: &str = "op-amp gain stage into a diode clipper
Vcc vcc 0 DC 9
R_b1 vcc vbias 100k
R_b2 vbias 0 100k
C_b vbias 0 10u
C_in in np 100n
R_in np vbias 1Meg
U1 np nm oa OPA
R_f oa nm 500k
R_g nm ng 4.7k
C_g ng 0 10u
C_c oa n1 1u
R_1 n1 n2 1k
D_1 n2 0 DSIG
D_2 0 n2 DSIG
C_o n2 out 1u
R_v out 0 100k
.model OPA OA(AOL=200000 VCC=9 VEE=0)
.model DSIG D(IS=2.52n N=1.752)
.END";

const FS: f64 = 48000.0;

fn full_lu_code(spice: &str, rail: Option<OpampRailMode>) -> String {
    let mut config = support::config_for_spice(spice, FS);
    config.nodal_sub_path_override = NodalSubPathOverride::FullLu;
    if let Some(r) = rail {
        config.opamp_rail_mode = r;
    }
    let code = support::generate_circuit_code_nodal(spice, &config).0;
    assert!(code.contains("state.chord_lu"), "not a full-LU build");
    code
}

/// Every output sample's bits, the refactor count and the summed Newton
/// iteration count of a 1 kHz render at `amp` volts.
fn render_bits(code: &str, amp: f64, samples: usize, tag: &str) -> Vec<u64> {
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    let mut iters = 0u64;
    for i in 0..{samples}usize {{
        let x = {amp:?} * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / {FS:?}).sin();
        let y = process_sample(x, &mut s);
        iters += s.last_nr_iterations as u64;
        println!(\"{{}}\", y[0].to_bits());
    }}
    println!(\"{{}}\", s.diag_refactor_count);
    println!(\"{{}}\", iters);
}}"
    );
    support::compile_and_run(code, &main, tag)
        .stdout
        .lines()
        .filter_map(|l| l.trim().parse::<u64>().ok())
        .collect()
}

/// `code` with the whole chord storage set to NaN before every touched-
/// position G_aug build (trapezoidal and backward-Euler sites).
fn poison_touched_builds(code: &str) -> String {
    let marker = "positions the sparse factor and back-solve touch";
    let lines: Vec<&str> = code.lines().collect();
    let mut out = String::with_capacity(code.len() + 4096);
    let mut hits = 0;
    for (k, line) in lines.iter().enumerate() {
        if line.contains(marker) {
            let next = lines[k + 1].trim_start();
            let lu = &next[..next.find('[').expect("indexed store after the marker")];
            let indent = &line[..line.len() - line.trim_start().len()];
            // The packed sparse storage (`*_lu_pk`) is poisoned whole, and so is
            // the N x N array the dense fallback shares with it.
            if let Some(full) = lu.strip_suffix("_pk") {
                out.push_str(&format!("{indent}{lu} = [f64::NAN; LU_SLOTS];\n"));
                out.push_str(&format!("{indent}{full} = [[f64::NAN; N]; N];\n"));
            } else {
                out.push_str(&format!("{indent}{lu} = [[f64::NAN; N]; N];\n"));
            }
            hits += 1;
        }
        out.push_str(line);
        out.push('\n');
    }
    assert!(hits >= 1, "no touched-position build found");
    out
}

#[test]
fn touched_position_build_never_reads_outside_its_positions() {
    let code = full_lu_code(BJT_LADDER, None);
    assert!(
        code.contains("const EQUIL_PAT"),
        "witness must take the structural pattern"
    );
    assert!(
        code.contains("fn sparse_lu_factor"),
        "witness must take the sparse schedule"
    );
    assert!(
        !code.contains("g_aug_stamped"),
        "the saved-copy fallback is gone"
    );
    // The touched set must reach past the input pattern (fill positions).
    let count_after = |key: &str| -> usize {
        let at = code.find(key).expect(key) + key.len();
        code[at..]
            .chars()
            .take_while(|c| c.is_ascii_digit())
            .collect::<String>()
            .parse()
            .unwrap()
    };
    let equil = count_after("EQUIL_PAT: [(u16, u16); ");
    let touched = count_after("// Only the ");
    assert!(
        touched > equil,
        "witness has no fill outside the input pattern: {touched} vs {equil}"
    );
    // A dense fallback on every third refactor, in both builds, so the
    // sparse path also starts from storage a dense factorization wrote.
    let alternate = |c: &str| {
        let out = c.replace(
            "if sparse_lu_factor(",
            "if state.diag_refactor_count % 3 != 1 && sparse_lu_factor(",
        );
        assert_ne!(out, c, "fallback forcing did not apply");
        out
    };
    let plain = alternate(&code);
    let poisoned = alternate(&poison_touched_builds(&code));
    let n = 6000;
    let a = render_bits(&plain, 2.0, n, "touch_plain");
    let b = render_bits(&poisoned, 2.0, n, "touch_poisoned");
    assert_eq!(a.len(), n + 2);
    let refactors = a[n];
    assert!(refactors > 100, "witness must refactor often: {refactors}");
    assert!(
        a[..n].iter().any(|&bits| f64::from_bits(bits).abs() > 1e-3),
        "witness output is trivial"
    );
    assert_eq!(a, b, "a position outside the touched set was read");
}

/// `ls_hit` never set: every evaluation the reuse skips runs again.
fn without_reuse(code: &str) -> String {
    let out = code.replace("ls_hit = true;", "ls_hit = false;");
    assert_ne!(out, code, "reuse disabling did not apply");
    out
}

#[test]
fn accepted_trial_reuse_is_bit_identical() {
    for (spice, amp, tag) in [(BJT_LADDER, 2.0, "bjt"), (TWO_TUBE_STAGE, 3.0, "tube")] {
        let code = full_lu_code(spice, None);
        let n = 6000;
        let a = render_bits(&code, amp, n, &format!("reuse_{tag}"));
        let b = render_bits(&without_reuse(&code), amp, n, &format!("reuse_off_{tag}"));
        assert_eq!(a.len(), n + 2);
        assert!(a[n + 1] > n as u64 / 2, "{tag}: witness must iterate");
        assert_eq!(a, b, "{tag}: reuse changed the render");
    }
}

#[test]
fn accepted_trial_reuse_gating() {
    // Junction devices: the trial carries the Jacobian, the next iteration
    // starts from it, and r0 is the trial's residual.
    let bjt = full_lu_code(BJT_LADDER, None);
    assert!(bjt.contains("kcl_residual_eval_jac(&vc"));
    assert!(bjt.contains("if ls_prev_hit {"));
    assert!(!bjt.contains("fn kcl_residual_eval("));
    // Triodes: only the device-chord check and the gate take the trial.
    let tube = full_lu_code(TWO_TUBE_STAGE, None);
    assert!(tube.contains("kcl_residual_eval(&vc"));
    assert!(tube.contains("i_nl_resid = ls_i_nl;"));
    assert!(!tube.contains("ls_prev_hit"));
    // Hard op-amp rails clamp `v` after the step: no reuse at all.
    let hard = full_lu_code(OPAMP_CLIPPER, Some(OpampRailMode::Hard));
    assert!(!hard.contains("ls_hit"));
    let active = full_lu_code(OPAMP_CLIPPER, Some(OpampRailMode::ActiveSet));
    assert!(active.contains("ls_hit"));
}

/// The sparse factor and back-solve address only the packed slots, never the
/// N x N array; the static row swaps are folded into the slot indices; the
/// Newton loop builds, factors and back-solves the packed storage at both
/// sites and leaves the N x N array to the dense fallback.
#[test]
fn sparse_factor_uses_packed_storage() {
    let code = full_lu_code(BJT_LADDER, None);
    let body = |name: &str| -> &str {
        let at = code.find(&format!("fn {name}(")).expect(name);
        let end = code[at..].find("\n}\n").expect("function end") + at;
        &code[at..end]
    };
    let slots: usize = {
        let key = "const LU_SLOTS: usize = ";
        let at = code.find(key).expect("LU_SLOTS") + key.len();
        code[at..]
            .chars()
            .take_while(|c| c.is_ascii_digit())
            .collect::<String>()
            .parse()
            .unwrap()
    };
    let touched = {
        let key = "// Only the ";
        let at = code.find(key).expect(key) + key.len();
        code[at..]
            .chars()
            .take_while(|c| c.is_ascii_digit())
            .collect::<String>()
            .parse::<usize>()
            .unwrap()
    };
    assert_eq!(slots, touched, "packed slots must be the touch pattern");
    for name in ["sparse_lu_factor", "sparse_lu_back_solve"] {
        let b = body(name);
        assert!(b.contains("[f64; LU_SLOTS]"), "{name} is not packed");
        // `a[<digits>][` / `a_lu[<digits>][`: a 2-D literal index into the matrix.
        let two_d = ["a[", "a_lu["].iter().any(|key| {
            b.match_indices(key).any(|(at, _)| {
                let tail = &b[at + key.len()..];
                let digits = tail.chars().take_while(|c| c.is_ascii_digit()).count();
                digits > 0 && tail[digits..].starts_with("][")
            })
        });
        assert!(!two_d, "{name} still indexes a 2-D array");
    }
    assert!(
        !body("sparse_lu_factor").contains("a.swap("),
        "row swaps not folded"
    );
    assert!(code.contains("sparse_lu_factor(&mut state.chord_lu_pk,"));
    assert!(code.contains("sparse_lu_back_solve(&state.chord_lu_pk,"));
    // Between the packed build and the sparse factor call, nothing writes the
    // N x N array: every stamp went to the packed slots.
    let start = code
        .find("positions the sparse factor and back-solve touch, packed")
        .unwrap();
    let call = code[start..].find("if sparse_lu_factor(").unwrap() + start;
    assert!(
        !code[start..call].contains("state.chord_lu["),
        "a stamp reached the N x N array instead of the packed factor"
    );
}
