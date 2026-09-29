//! The nodal-Schur Newton starts where the full-LU Newton does.
//!
//! Near a regenerative fold the implicit step has more than one root, and the
//! root Newton reaches depends on where it starts. Full-LU starts at
//! `v = v_prev`. Schur now starts at the device currents whose controlling
//! voltages `p + K·i_nl` equal `N_v·v_prev`, so its first iterate is full-LU's.
//! The first-order current predictor it used before reached a genuine root on
//! the switched branch early: every sample KCL-valid, no counter moving, and
//! the wrong period. The end-to-end witness (an IC-seeded astable against its
//! ngspice period) is `cli_integration::test_ic_seeded_astable_schur_period_matches_spice`.

mod support;

use melange_solver::codegen::NodalSubPathOverride;

const CLIPPER: &str = "diode clipper\nR1 in mid 1k\nC1 mid 0 10n\nD1 mid out DX\nD2 out mid DX\n\
                       R2 out 0 100k\nC2 out 0 1n\nRb mid 0 1Meg\n\
                       .model DX D(IS=2.52n N=1.752 RS=0.568 CJO=4p TT=20n)\n";

fn schur_code(deck: &str, fs: f64) -> String {
    let mut c = support::config_for_spice(deck, fs);
    c.nodal_sub_path_override = NodalSubPathOverride::Schur;
    let code = support::generate_circuit_code_nodal(deck, &c).0;
    assert!(
        code.contains("Warm start at full-LU's starting point"),
        "not a Schur build with the exact warm start"
    );
    code
}

/// `K` is singular whenever two devices share a controlling voltage: this
/// antiparallel diode pair gives `K = [[−a, a], [a, −a]]`. That system is
/// consistent and solved exactly, so a normal build never falls back. A start
/// the device currents cannot reach (here `K` zeroed after construction,
/// test-only) falls back to the first-order predictor on every Newton solve,
/// and every fallback is counted.
#[test]
fn only_an_unreachable_start_falls_back_and_it_is_counted() {
    let code = schur_code(CLIPPER, 48000.0);
    let run = |zero: bool, tag: &str| -> (u64, u64) {
        let setup = if zero {
            "s.k = [[0.0; M]; M]; s.k_be = [[0.0; M]; M];"
        } else {
            ""
        };
        let main = format!(
            "fn main() {{
    let mut s = CircuitState::default();
    let f0 = s.diag_warm_start_fallback_count;
    {setup}
    for i in 1..=100usize {{
        let _ = process_sample(0.5 * (i as f64 * 0.05).sin(), &mut s);
    }}
    println!(\"fallback={{}}\", s.diag_warm_start_fallback_count - f0);
    println!(\"unsolved={{}}\", s.diag_unsolved_sample_count);
}}"
        );
        let out = support::compile_and_run(&code, &main, tag);
        (
            out.parse_kv("fallback").unwrap() as u64,
            out.parse_kv("unsolved").unwrap() as u64,
        )
    };
    assert_eq!(
        run(false, "ws_normal"),
        (0, 0),
        "a normal build never falls back"
    );
    let (fallback, _) = run(true, "ws_singular");
    assert!(
        fallback >= 100,
        "every solve from an unreachable start must be counted ({fallback})"
    );
}
