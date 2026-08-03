//! Canonical conformance for the diode reverse-breakdown (Zener) model.
//!
//! Breakdown is codegen-only (`templates/rust/device_diode.rs.tera`,
//! `diode_breakdown_current`) and — per the correctness audit — had *zero*
//! tests of any kind. This checks the emitted function directly against the
//! canonical Zener breakdown law, independent of ngspice:
//!
//!   I_bv(v_d) = −IBV · exp(−(v_d + BV) / (N·VT))
//!
//! The defining property is I_bv(−BV) = −IBV (the current is IBV *at* the
//! breakdown voltage). We generate a BV diode circuit so the template emits
//! the breakdown/forward/`fast_exp` helpers, then a custom main calls them
//! across a reverse-bias sweep. The shipping `fast_exp` is a 5th-order minimax
//! polynomial (<0.0004% max relative error over [−40, 40]; exact `f64::exp`
//! only under `--cfg melange_precise_exp`), so agreement with the canonical
//! exponential is to that ~4e-6 accuracy — this test confirms the *shipping*
//! diode math follows the canonical Zener law, not an idealized build. The
//! defining anchor I(−BV) = −IBV is nonetheless exact, since fast_exp(0) = 1
//! exactly, and the ±40 clamp is a documented, testable boundary.

mod support;

use melange_solver::codegen::CodegenConfig;

/// Tolerance for the canonical exponential *form*: the shipping fast_exp is a
/// polynomial approximation with <0.0004% (4e-6) documented max error.
const TOL_FASTEXP: f64 = 5e-6;

// Parameters passed to the emitted functions (the functions take them as
// arguments, so these need not be physical — they exercise the arithmetic).
// n_vt = 0.05 keeps the breakdown sweep inside the ±40 exp clamp for
// v_d ∈ [−7.6, −3.6] (x = −(v_d+BV)/n_vt ∈ [−40, 40]).
const N_VT: f64 = 0.05;
const BV: f64 = 5.6;
const IBV: f64 = 5e-3;
const IS: f64 = 1e-14;

// A diode with a BV model so codegen emits the breakdown helpers.
const ZENER: &str = "\
Zener
R1 in out 1k
D1 out 0 ZMOD
.model ZMOD D(IS=1e-14 N=1 BV=5.6 IBV=5e-3)
";

/// Canonical breakdown current, replicating the documented ±40 exp clamp.
fn breakdown_ref(v_d: f64) -> f64 {
    let x = (-(v_d + BV) / N_VT).clamp(-40.0, 40.0);
    -IBV * x.exp()
}

/// Canonical Shockley forward current (legacy path, x within clamp).
fn shockley_ref(v_d: f64) -> f64 {
    IS * ((v_d / N_VT).clamp(-40.0, 40.0).exp() - 1.0)
}

fn assert_rel(got: f64, want: f64, tol: f64, ctx: &str) {
    let denom = want.abs().max(1e-30);
    let rel = (got - want).abs() / denom;
    assert!(
        rel < tol,
        "{ctx}: got {got:.15e}, canonical {want:.15e}, rel {rel:.3e}"
    );
}

/// Generate the BV-diode circuit, append a main that calls the emitted
/// `diode_breakdown_current` / `diode_current` at the sweep points, and return
/// the parsed (label, v_d, emitted_value) rows.
fn emitted_values() -> Vec<(String, f64, f64)> {
    let config = CodegenConfig {
        sample_rate: 48_000.0,
        ..support::config_for_spice(ZENER, 48_000.0)
    };
    let (code, _n, _m) = support::generate_circuit_code(ZENER, &config);

    let bd_pts = "[-5.6_f64, -5.65, -5.7, -6.0, -7.0, -10.0]";
    let fd_pts = "[-0.1_f64, 0.0, 0.3, 0.5, 0.6]";
    let main_code = format!(
        r#"
fn main() {{
    let n_vt = {N_VT:?}; let bv = {BV:?}; let ibv = {IBV:?}; let is = {IS:?};
    for v_d in {bd_pts} {{
        println!("BD {{:.17e}} {{:.17e}}", v_d, diode_breakdown_current(v_d, n_vt, bv, ibv));
    }}
    for v_d in {fd_pts} {{
        println!("FD {{:.17e}} {{:.17e}}", v_d, diode_current(v_d, is, n_vt));
    }}
}}
"#
    );

    let out = support::compile_and_run(&code, &main_code, "zener_breakdown");
    out.stdout
        .lines()
        .filter_map(|l| {
            let mut it = l.split_whitespace();
            let label = it.next()?;
            if label != "BD" && label != "FD" {
                return None;
            }
            let v_d = it.next()?.parse().ok()?;
            let val = it.next()?.parse().ok()?;
            Some((label.to_string(), v_d, val))
        })
        .collect()
}

#[test]
fn breakdown_matches_canonical_zener_law() {
    let rows = emitted_values();
    let bd: Vec<_> = rows.iter().filter(|(l, ..)| l == "BD").collect();
    let fd: Vec<_> = rows.iter().filter(|(l, ..)| l == "FD").collect();
    assert_eq!(bd.len(), 6, "expected 6 breakdown points");
    assert_eq!(fd.len(), 5, "expected 5 forward points");

    // Emitted breakdown current matches the canonical Zener law everywhere,
    // to the accuracy of the polynomial fast_exp.
    for (_, v_d, got) in &bd {
        assert_rel(
            *got,
            breakdown_ref(*v_d),
            TOL_FASTEXP,
            &format!("breakdown Id({v_d})"),
        );
    }
    // Emitted forward current matches Shockley.
    for (_, v_d, got) in &fd {
        assert_rel(
            *got,
            shockley_ref(*v_d),
            TOL_FASTEXP,
            &format!("forward Id({v_d})"),
        );
    }
}

#[test]
fn breakdown_defining_properties() {
    let rows = emitted_values();
    let bd = |v: f64| {
        rows.iter()
            .find(|(l, vd, _)| l == "BD" && (*vd - v).abs() < 1e-9)
            .map(|(.., val)| *val)
            .unwrap_or_else(|| panic!("no breakdown point at v_d={v}"))
    };

    // Defining property: at v_d = −BV the breakdown current is exactly −IBV
    // (exact, not approximate — fast_exp(0) = 1 to the bit).
    assert_rel(bd(-5.6), -IBV, 1e-12, "I_bv(−BV) = −IBV");

    // Exponential steepness: making v_d more negative by n_vt multiplies the
    // magnitude by e (the −BV−n_vt point is v_d = −5.65), to fast_exp accuracy.
    let ratio = bd(-5.65) / bd(-5.6);
    assert_rel(ratio, std::f64::consts::E, TOL_FASTEXP, "ΔV=−n_vt ⇒ ×e");

    // Clamp boundary: at v_d = −10, x = −(v_d+BV)/n_vt = 88 clamps to 40, so the
    // emitted value saturates near −IBV·e^40 rather than the (huge) unclamped e^88.
    assert_rel(
        bd(-10.0),
        -IBV * 40.0_f64.exp(),
        TOL_FASTEXP,
        "±40 exp clamp",
    );
}
