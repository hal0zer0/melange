//! Datasheet saturation ratings (ISAT_DROP=, ISAT_BASIS=, L_AT_IDC=) convert
//! to the model's tanh scale current against the winding's terminal law.

use melange_solver::mna::{isat_from_datasheet, MnaSystem};
use melange_solver::parser::{IsatBasis, IsatSpec, Netlist};

/// Terminal inductance / L0 at x = i/ISAT, for coupling k and floor F.
fn terminal(x: f64, k: f64, f: f64, basis: IsatBasis) -> f64 {
    let m = k - f;
    match basis {
        IsatBasis::Incremental => (1.0 - k) + f + m / x.cosh().powi(2),
        IsatBasis::Apparent => (1.0 - k) + f + m * x.tanh() / x,
    }
}

/// Round trip: the converted ISAT puts the terminal inductance exactly
/// (1 - d)·L0 at the rated current, for single inductors and shared cores,
/// both bases.
#[test]
fn converted_isat_reproduces_the_rated_drop() {
    for (k, f) in [(1.0, 3e-4), (1.0, 1e-3), (0.9999, 3e-4), (0.99, 3e-4)] {
        for d in [0.05, 0.1, 0.2, 0.3, 0.5] {
            for basis in [IsatBasis::Incremental, IsatBasis::Apparent] {
                let spec = IsatSpec::Drop { drop: d, basis };
                let isat = isat_from_datasheet("L", 0.01, spec, 1.0, k, f).unwrap();
                let got = terminal(0.01 / isat, k, f, basis);
                assert!(
                    (got - (1.0 - d)).abs() < 1e-12,
                    "k {k} F {f} d {d} {basis:?}: L/L0 {got} at the rated current"
                );
            }
        }
    }
}

/// The analog-EE review's table (single inductor, LAIR 3e-4): ISAT / i_ds.
#[test]
fn conversion_matches_the_published_factors() {
    for (d, incr, app) in [(0.1, 3.05, 1.71), (0.2, 2.08, 1.13), (0.3, 1.63, 0.84)] {
        let f = |basis| {
            isat_from_datasheet("L", 1.0, IsatSpec::Drop { drop: d, basis }, 1.0, 1.0, 3e-4)
                .unwrap()
        };
        assert!((f(IsatBasis::Incremental) - incr).abs() < 5e-3, "d {d}");
        assert!((f(IsatBasis::Apparent) - app).abs() < 5e-3, "d {d}");
    }
    // L at rated DC = an incremental drop of 1 - L/L0.
    let l = isat_from_datasheet("L", 0.05, IsatSpec::LAtIdc { l: 0.8 }, 1.0, 1.0, 3e-4).unwrap();
    let d = isat_from_datasheet(
        "L",
        0.05,
        IsatSpec::Drop {
            drop: 0.2,
            basis: IsatBasis::Incremental,
        },
        1.0,
        1.0,
        3e-4,
    )
    .unwrap();
    assert!((l - d).abs() < 1e-15 * d);
}

fn mna(spice: &str) -> Result<MnaSystem, String> {
    let n = Netlist::parse(spice).map_err(|e| e.to_string())?;
    MnaSystem::from_netlist(&n).map_err(|e| e.to_string())
}

#[test]
fn datasheet_forms_reach_the_model_and_refuse_what_cannot_be() {
    let isat = |spice: &str| {
        mna(spice)
            .unwrap()
            .inductors
            .iter()
            .find_map(|i| i.isat)
            .unwrap()
    };
    let plain = isat("t\nR1 in a 99\nL1 a 0 1 ISAT=10m LAIR=3e-4\n");
    assert_eq!(plain, 10e-3);
    let rated = isat("t\nR1 in a 99\nL1 a 0 1 ISAT=10m ISAT_DROP=0.2 LAIR=3e-4\n");
    assert!((rated / 10e-3 - 2.0777).abs() < 1e-3, "{rated}");
    let at_dc = isat("t\nR1 in a 99\nL1 a 0 1 L_AT_IDC=0.8,10m LAIR=3e-4\n");
    assert!((at_dc - rated).abs() < 1e-15);
    // A drop beyond the saturable part is refused, with the reason.
    let e = mna("t\nR1 in a 99\nL1 a 0 1 ISAT=10m ISAT_DROP=0.9999 LAIR=3e-4\n").unwrap_err();
    assert!(e.contains("cannot be reached"), "{e}");
    // Shared core: converted against the core, one rated winding only.
    let core = isat(
        "x\nR1 in p 99\nL_pri p 0 1 ISAT=10m ISAT_DROP=0.2 CORE=steel\nL_sec s 0 1\n\
         K1 L_pri L_sec 0.9999\nR2 s 0 1k\n",
    );
    let expect = isat_from_datasheet(
        "L",
        10e-3,
        IsatSpec::Drop {
            drop: 0.2,
            basis: IsatBasis::Incremental,
        },
        1.0,
        0.9999,
        3e-4,
    )
    .unwrap();
    assert!((core - expect).abs() < 1e-15, "{core} vs {expect}");
    let e = mna(
        "x\nR1 in p 99\nL_pri p 0 1 ISAT=10m ISAT_DROP=0.2\nL_sec s 0 1 ISAT=10m\n\
         K1 L_pri L_sec 0.9999\nR2 s 0 1k\n",
    )
    .unwrap_err();
    assert!(e.contains("one winding only"), "{e}");
}

/// A converted or referred ISAT that is not a finite positive current is
/// refused. A drop of 1e-17 used to convert through acosh(1) = 0 to
/// `SAT_IND_0_ISAT = inf`, compile exited 0 and the emitted Rust did not
/// build; a referral that underflowed to 0 gave NaN every sample.
#[test]
fn unusable_saturation_currents_are_refused() {
    for spice in [
        "t\nR1 in a 99\nL1 a 0 1 ISAT=10m ISAT_DROP=1e-17\n",
        "t\nR1 in a 99\nL1 a 0 1 ISAT=10m ISAT_DROP=1e-17 ISAT_BASIS=apparent\n",
        "t\nR1 in a 99\nL1 a 0 1 ISAT=1e308 ISAT_DROP=0.2\n",
    ] {
        let e = mna(spice).unwrap_err();
        assert!(e.contains("L1"), "{spice}: {e}");
    }
    let e = mna("x\nR1 in p 99\nL1 p 0 1\nL2 s 0 1e-9 ISAT=5e-324\nK1 L1 L2 0.9999\nR2 s 0 1k\n")
        .unwrap_err();
    assert!(e.contains("referred"), "{e}");
    // The conversion near its resolvable limit is still exact (the atanh form).
    let at = |d: f64| {
        isat_from_datasheet(
            "L",
            1.0,
            IsatSpec::Drop {
                drop: d,
                basis: IsatBasis::Incremental,
            },
            1.0,
            1.0,
            3e-4,
        )
        .unwrap()
    };
    let m = 1.0 - 3e-4;
    let x = 1.0 / at(1e-6);
    let got = (1.0 - m) + m / x.cosh().powi(2);
    assert!(((1.0 - got) - 1e-6).abs() < 1e-15, "{got}");
}

#[test]
fn datasheet_keywords_are_parsed_strictly() {
    for bad in [
        "L1 a 0 1 ISAT_DROP=0.2",
        "L1 a 0 1 ISAT=10m ISAT_DROP=0",
        "L1 a 0 1 ISAT=10m ISAT_DROP=1",
        "L1 a 0 1 ISAT=10m ISAT_BASIS=apparent",
        "L1 a 0 1 ISAT=10m ISAT_DROP=0.2 ISAT_BASIS=secant",
        "L1 a 0 1 L_AT_IDC=0.8",
        "L1 a 0 1 L_AT_IDC=1.2,10m",
        "L1 a 0 1 L_AT_IDC=0.8,10m ISAT=10m",
        "L1 a 0 1 L_AT_IDC=0.8,10m ISAT_BASIS=apparent",
    ] {
        assert!(
            Netlist::parse(&format!("t\n{bad}\nR1 a 0 1k\n")).is_err(),
            "must be refused: {bad}"
        );
    }
    assert!(
        Netlist::parse("t\nL1 a 0 1 isat=10m isat_drop=0.2 isat_basis=Apparent\nR1 a 0 1k\n")
            .is_ok()
    );
}
