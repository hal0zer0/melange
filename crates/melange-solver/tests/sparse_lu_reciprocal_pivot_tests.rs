//! The sparse factor multiplies by reciprocal pivots: each reciprocal is
//! formed once, after its column's tiny-pivot test; the growth sweep reads the
//! true pivots and the diagonal takes the reciprocals only after it passes;
//! the back-solve multiplies by them; the dense fallback still divides.

mod support;

use melange_solver::codegen::NodalSubPathOverride;

/// A diode clipper behind a 15-section RC ladder, forced onto the full-LU route so the
/// sparse schedule runs.
const LADDER_CLIPPER: &str = "RC ladder into a diode clipper
R1 in a1 1k
C1 a1 0 10n
R2 a1 a2 1k
C2 a2 0 10n
R3 a2 a3 1k
C3 a3 0 10n
R4 a3 a4 1k
C4 a4 0 10n
R5 a4 a5 1k
C5 a5 0 10n
R6 a5 a6 1k
C6 a6 0 10n
R7 a6 a7 1k
C7 a7 0 10n
R8 a7 a8 1k
C8 a8 0 10n
R9 a8 a9 1k
C9 a9 0 10n
R10 a9 a10 1k
C10 a10 0 10n
R11 a10 a11 1k
C11 a11 0 10n
R12 a11 a12 1k
C12 a12 0 10n
R13 a12 a13 1k
C13 a13 0 10n
R14 a13 a14 1k
C14 a14 0 10n
R15 a14 a15 1k
C15 a15 0 10n
R16 a15 out 1k
D1 out 0 DSIG
D2 0 out DSIG
C16 out 0 1n
R17 out 0 100k
.model DSIG D(IS=2.52n N=1.752)
.END";

fn body<'a>(code: &'a str, name: &str) -> &'a str {
    let at = code.find(&format!("fn {name}(")).expect(name);
    let end = code[at..].find("\n}\n").expect("function end") + at;
    &code[at..end]
}

#[test]
fn sparse_factor_multiplies_by_reciprocal_pivots() {
    let mut config = support::config_for_spice(LADDER_CLIPPER, 48000.0);
    config.nodal_sub_path_override = NodalSubPathOverride::FullLu;
    let code = support::generate_circuit_code_nodal(LADDER_CLIPPER, &config).0;
    assert!(
        code.contains("fn sparse_lu_factor("),
        "witness must take the sparse schedule"
    );
    let n: usize = {
        let key = "pub const N: usize = ";
        let at = code.find(key).expect("N") + key.len();
        code[at..]
            .chars()
            .take_while(|c| c.is_ascii_digit())
            .collect::<String>()
            .parse()
            .unwrap()
    };

    let f = body(&code, "sparse_lu_factor");
    assert!(!f.contains("/="), "the sparse factor still divides");
    // One reciprocal per pivot, each formed once.
    let lets: Vec<&str> = f
        .lines()
        .filter(|l| l.trim_start().starts_with("let inv_"))
        .collect();
    assert_eq!(lets.len(), n, "one reciprocal per pivot");
    // A reciprocal formed during the elimination follows its column's
    // tiny-pivot test on the line before.
    let lines: Vec<&str> = f.lines().collect();
    for (k, l) in lines.iter().enumerate() {
        if l.trim_start().starts_with("let inv_") && lines[k - 1].contains("< 1e-30") {
            let pivot = l
                .split('=')
                .nth(1)
                .unwrap()
                .trim()
                .trim_start_matches("1.0 / ")
                .trim_end_matches(';');
            assert!(
                lines[k - 1].contains(&format!("if {pivot}.abs() < 1e-30")),
                "{l}"
            );
        }
    }
    // The diagonal takes the reciprocals only after the growth sweep passes.
    let gate = f
        .find("if !growth.is_finite() || growth > 1e8 { return false; }")
        .expect("growth gate");
    let diag = f.find("Diagonal := 1/pivot").expect("diagonal write");
    assert!(diag > gate, "reciprocals written before the growth sweep");
    assert_eq!(
        f[diag..].matches("= inv_").count(),
        n,
        "every pivot's reciprocal reaches the diagonal"
    );

    let b = body(&code, "sparse_lu_back_solve");
    assert!(!b.contains("/="), "the back-solve still divides");
    assert_eq!(
        b.matches("*= a_lu[").count(),
        n,
        "one reciprocal multiply per pivot"
    );

    // The dense fallback is untouched: it still divides by its pivots.
    assert!(
        body(&code, "lu_factor").contains("let factor = a[row][col] / pivot;"),
        "dense fallback changed"
    );
}
