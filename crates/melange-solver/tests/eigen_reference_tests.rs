//! The eigen solver and the ring predicate against LAPACK.
//!
//! References come from numpy/scipy `eig` (LAPACK `dgeev`, left and right
//! vectors), written by `tests/data/gen_ring_reference.py`:
//! - `eigen_reference.json`: synthetic matrices — dense random up to 120,
//!   clusters of three poles near −1 (spacing down to 1e-7), near-defective
//!   pairs near −1, complex pairs near −1, similarities scaled over 1e12,
//!   and a propagator-like spectrum with repeated zeros.
//! - `ring_reference.json`: the linearised charge propagator of the in-repo
//!   golden decks (`tools/golden-harness/decks`), with the predicate's ring
//!   modes.
//!
//! Gates:
//! - every eigenvalue within `1e-10·max(|λ|, 1e-2·ρ) + n·κ·ε·‖A‖_F` of its
//!   LAPACK counterpart. `κ = 1/|lᴴ·r|` is the eigenvalue's condition number
//!   (from the reference vectors). The first term is the relative accuracy
//!   asked of a well-conditioned eigenvalue, floored so that eigenvalues at
//!   or near 0 are not asked for a relative accuracy no algorithm has. The
//!   second is the standard first-order bound for a backward-stable method:
//!   a near-defective pair (condition ~1e6) is not computable to 1e-10
//!   relative by LAPACK either, and the two differ by about that bound.
//! - every Nyquist-side residue within 0.1 dB, where it is above the
//!   floating-point floor (1e-9 of the passband for the predicate, 1e-9 of
//!   the largest residue for the synthetic cases). Below that floor both
//!   sides are rounding noise, 120 dB under the −60 dB decision, and only
//!   have to stay there.

use melange_solver::codegen::ring::{self, RingSystem};
use melange_solver::eigen::{self, Complex};

fn load(name: &str) -> serde_json::Value {
    let path = format!("{}/tests/data/{name}", env!("CARGO_MANIFEST_DIR"));
    serde_json::from_str(&std::fs::read_to_string(&path).expect(&path)).unwrap()
}

/// Numbers, or exact round-trip strings (parsed by `str::parse`, which
/// rounds correctly; serde_json's number parser can be off by an ulp, and a
/// regression matrix can depend on its last bit).
fn f64s(v: &serde_json::Value) -> Vec<f64> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|x| match x.as_str() {
            Some(s) => s.parse().unwrap(),
            None => x.as_f64().unwrap(),
        })
        .collect()
}

fn cplxs(v: &serde_json::Value) -> Vec<Complex> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|p| Complex::new(p[0].as_f64().unwrap(), p[1].as_f64().unwrap()))
        .collect()
}

/// Match each computed eigenvalue to a distinct reference one (greedy,
/// nearest first). Returns the worst error as a fraction of its tolerance,
/// the worst plain relative error `|Δλ| / max(|λ|, 1e-2·ρ)`, and the worst
/// relative error among the eigenvalues the ring predicate reads
/// (`|λ| ≥ 0.9`: growth and the lasting Nyquist-side poles).
fn match_spectra(got: &[Complex], want: &[Complex], cond: &[f64], norm: f64) -> (f64, f64, f64) {
    assert_eq!(got.len(), want.len(), "eigenvalue count");
    let n = want.len() as f64;
    let rho = want.iter().fold(0.0_f64, |m, z| m.max(z.abs()));
    let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
    for (i, g) in got.iter().enumerate() {
        for (j, w) in want.iter().enumerate() {
            pairs.push(((*g - *w).abs(), i, j));
        }
    }
    pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut used_g = vec![false; got.len()];
    let mut used_w = vec![false; want.len()];
    let (mut worst_frac, mut worst_rel, mut worst_read) = (0.0_f64, 0.0_f64, 0.0_f64);
    for (d, i, j) in pairs {
        if used_g[i] || used_w[j] {
            continue;
        }
        used_g[i] = true;
        used_w[j] = true;
        let scale = want[j].abs().max(1e-2 * rho).max(f64::MIN_POSITIVE);
        let tol = 1e-10 * scale + n * cond[j] * f64::EPSILON * norm;
        if std::env::var("MELANGE_EIG_DEBUG").is_ok() && d / scale > 1e-10 {
            eprintln!(
                "    got {:+.12e}{:+.3e}i want {:+.12e}{:+.3e}i  |d| {:.2e} rel {:.2e} cond {:.2e} frac {:.3}",
                got[i].re, got[i].im, want[j].re, want[j].im, d, d / scale, cond[j], d / tol
            );
        }
        worst_frac = worst_frac.max(d / tol);
        worst_rel = worst_rel.max(d / scale);
        if want[j].abs() >= 0.9 {
            worst_read = worst_read.max(d / want[j].abs());
        }
    }
    (worst_frac, worst_rel, worst_read)
}

fn db(ratio: f64) -> f64 {
    20.0 * ratio.log10()
}

#[test]
fn eigenvalues_and_residues_match_lapack() {
    check_eigen_cases("eigen_reference.json");
}

/// Real propagators on which an earlier QR stalled. Each carries 30 columns
/// that are exactly zero (the algebraic directions), so the zero eigenvalue
/// comes in long defective chains; the iteration cycled there with 49 and 54
/// eigenvalues unresolved, at any sweep budget. Isolating the eigenvalues the
/// sparsity pattern exposes takes them out before QR runs.
#[test]
fn stalled_propagators_converge_and_match_lapack() {
    check_eigen_cases("eigen_regression.json");
}

fn check_eigen_cases(file: &str) {
    let data = load(file);
    for case in data["cases"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let n = case["n"].as_u64().unwrap() as usize;
        let a = f64s(&case["a"]);
        let b = f64s(&case["b"]);
        let c = f64s(&case["c"]);
        let want = cplxs(&case["eigenvalues"]);
        let want_res = cplxs(&case["residues"]);

        let cond = f64s(&case["cond"]);
        let norm = case["norm"].as_f64().unwrap();
        let got = eigen::eigenvalues(&a, n).expect(name);
        let (frac, rel, read) = match_spectra(&got, &want, &cond, norm);
        eprintln!(
            "{name}: worst eigenvalue error {rel:.2e} relative ({read:.2e} where |z| >= 0.9), {frac:.3} of tolerance (max condition {:.1e})",
            cond.iter().fold(0.0_f64, |m, &c| m.max(c))
        );
        assert!(frac <= 1.0, "{name}: eigenvalue error {frac} x tolerance");

        // Residues of the Nyquist-side poles (the ones the predicate reads).
        let max_res = want_res.iter().fold(0.0_f64, |m, r| m.max(r.abs()));
        let mut worst_db = 0.0_f64;
        for (w, wr) in want.iter().zip(&want_res) {
            if w.re >= 0.0 || w.abs() < 0.9 || wr.abs() < 1e-9 * max_res {
                continue;
            }
            let r = eigen::modal_residue(&a, n, *w, &b, &c);
            let d = db(r.abs() / wr.abs()).abs();
            worst_db = worst_db.max(d);
            assert!(
                d <= 0.1,
                "{name}: residue at {w:?}: {:e} vs {:e} ({d:.3} dB)",
                r.abs(),
                wr.abs()
            );
        }
        eprintln!("{name}: worst Nyquist-side residue error {worst_db:.2e} dB");
    }
}

fn check_ring_case(sys: &RingSystem, reference: &serde_json::Value, label: &str) {
    let want = cplxs(&reference["eigenvalues"]);
    let cond = f64s(&reference["cond"]);
    let norm = reference["norm"].as_f64().unwrap();
    let got = ring::propagator_eigenvalues(sys).expect(label);
    assert_eq!(
        got.len(),
        want.len(),
        "{label}: propagator dimension (rank(H) reference {})",
        reference["rank_h"]
    );
    let (frac, worst, read) = match_spectra(&got, &want, &cond, norm);
    assert!(
        frac <= 1.0,
        "{label}: eigenvalue error {frac} x tolerance ({worst:e} relative)"
    );

    let v = ring::analyze(sys).expect(label);
    let want_modes = reference["ring_modes"].as_array().unwrap();
    assert_eq!(
        v.ring_modes.len(),
        want_modes.len(),
        "{label}: ring-mode count"
    );
    let mut worst_db = 0.0_f64;
    for wm in want_modes {
        let z = Complex::new(wm["z"][0].as_f64().unwrap(), wm["z"][1].as_f64().unwrap());
        let rel = wm["residue_rel"].as_f64().unwrap();
        let got = v
            .ring_modes
            .iter()
            .min_by(|a, b| (a.z - z).abs().total_cmp(&(b.z - z).abs()))
            .unwrap();
        if rel < 1e-9 {
            assert!(
                got.residue_rel < 1e-8,
                "{label}: residue at {z:?}: {:e} vs {rel:e}",
                got.residue_rel
            );
            continue;
        }
        let d = db(got.residue_rel / rel).abs();
        worst_db = worst_db.max(d);
        assert!(
            d <= 0.1,
            "{label}: residue at {z:?}: {:e} vs {rel:e}",
            got.residue_rel
        );
    }
    eprintln!(
        "{label}: dim {}, worst eigenvalue error {:.2e} relative ({read:.2e} where |z| >= 0.9; {frac:.3} of tolerance, max condition {:.1e}), worst residue error {worst_db:.2e} dB, verdict {}",
        got.len(),
        worst,
        cond.iter().fold(0.0_f64, |m, &c| m.max(c)),
        if v.promote { "BE" } else { "trap" }
    );
}

#[test]
fn ring_predicate_matches_lapack_on_the_in_repo_decks() {
    let data = load("ring_reference.json");
    for case in data["cases"].as_array().unwrap() {
        let sys: RingSystem = serde_json::from_value(case["system"].clone()).unwrap();
        let label = case["name"].as_str().unwrap_or("?").to_string();
        check_ring_case(&sys, &case["reference"], &label);
    }
}

/// The same gate over a local directory of `*.ring.json` files (system +
/// reference, as `gen_ring_reference.py ring` writes them one per file).
/// Not every corpus deck is in this repository, so this runs on demand:
/// `MELANGE_RING_DIR=<dir> cargo test --test eigen_reference_tests -- --ignored`.
#[test]
#[ignore]
fn ring_predicate_matches_lapack_on_a_local_corpus() {
    let dir = std::env::var("MELANGE_RING_DIR").expect("MELANGE_RING_DIR");
    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.to_string_lossy().ends_with(".ring.json"))
        .collect();
    files.sort();
    assert!(!files.is_empty(), "no *.ring.json in {dir}");
    for f in files {
        let case: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&f).unwrap()).unwrap();
        let sys: RingSystem = serde_json::from_value(case["system"].clone()).unwrap();
        check_ring_case(
            &sys,
            &case["reference"],
            &f.file_name().unwrap().to_string_lossy(),
        );
    }
}
