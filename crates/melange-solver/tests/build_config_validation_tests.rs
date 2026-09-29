//! The shipped build validates its codegen config on every route.
//!
//! The DK route generates through `generate_with_dc_op` (it hands over the DC
//! OP preflight), which used to skip every check `generate` makes: an
//! oversampling factor outside {1, 2, 4}, a non-positive tolerance, a zero
//! Newton budget, out-of-range ports. A library caller of `build` got code, not
//! a refusal; factor 3 emitted the 2x wrapper around matrices built at 3x, so
//! each output sample advanced 2/3 of a sample period. The CLI verbs, the
//! parser and melange-validate each refuse such a factor before building, so
//! no command reached it.

mod support;

const CLIPPER: &str = "\
Diode clipper
R1 in out 1k
D1 out 0 D1N4148
D2 0 out D1N4148
C1 out 0 10n
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

fn refusal(
    solver: &str,
    tweak: impl FnOnce(&mut melange_solver::codegen::CodegenConfig),
) -> String {
    let mut config = support::config_for_spice(CLIPPER, 48000.0);
    tweak(&mut config);
    match support::try_build_shipped(CLIPPER, &config, solver) {
        Ok(_) => panic!("--solver {solver}: the build must refuse this config"),
        Err(e) => e,
    }
}

#[test]
fn oversampling_factor_three_is_refused_on_both_routes() {
    for solver in ["dk", "nodal"] {
        let err = refusal(solver, |c| c.oversampling_factor = 3);
        assert!(
            err.contains("oversampling_factor must be 1, 2, or 4, got 3"),
            "--solver {solver}: {err}"
        );
    }
}

#[test]
fn non_positive_tolerance_is_refused_on_both_routes() {
    for solver in ["dk", "nodal"] {
        let err = refusal(solver, |c| c.tolerance = 0.0);
        assert!(
            err.contains("tolerance must be positive and finite"),
            "--solver {solver}: {err}"
        );
    }
}

#[test]
fn zero_newton_budget_is_refused_on_both_routes() {
    for solver in ["dk", "nodal"] {
        let err = refusal(solver, |c| c.max_iterations = 0);
        assert!(
            err.contains("max_iterations must be > 0"),
            "--solver {solver}: {err}"
        );
    }
}

#[test]
fn the_valid_factors_still_build_on_both_routes() {
    for solver in ["dk", "nodal"] {
        for os in [1, 2, 4] {
            let mut config = support::config_for_spice(CLIPPER, 48000.0);
            config.oversampling_factor = os;
            let built = support::build_shipped(CLIPPER, &config, solver);
            assert_eq!(built.oversampling, os, "--solver {solver}");
        }
    }
}

/// A diode biased from a 9 V supply: its linear start puts the junction at
/// about 4.5 V, so its operating point takes Newton more than one iteration.
const BIASED_DIODE: &str = "\
Biased diode
Vcc vcc 0 DC 9
Rb vcc out 10k
D1 out 0 DX
Rin in out 10k
C1 out 0 10n
.model DX D(IS=1e-14)
";

/// A one-iteration DC-OP Newton budget (the test-only knob): the biased
/// diode's operating point cannot converge in it, so the build is refused
/// without depending on any open convergence bug.
fn one_dc_op_iteration(o: &mut melange_solver::build::BuildOptions) {
    o.dc_op_max_iterations = Some(1);
}

#[test]
fn an_unconverged_dc_operating_point_is_refused() {
    let config = support::config_for_spice(BIASED_DIODE, 48000.0);
    for solver in ["auto", "nodal"] {
        let err = match support::try_build_shipped_with(
            BIASED_DIODE,
            &config,
            solver,
            one_dc_op_iteration,
        ) {
            Ok(_) => panic!("--solver {solver}: an unconverged DC OP must be refused"),
            Err(e) => e,
        };
        assert!(
            err.contains("the DC operating point did not converge")
                && err.contains("--allow-unconverged-dc-op"),
            "--solver {solver}: {err}"
        );
    }
}

#[test]
fn allow_unconverged_dc_op_builds_it_and_says_so_in_the_code() {
    let config = support::config_for_spice(BIASED_DIODE, 48000.0);
    let built = support::try_build_shipped_with(BIASED_DIODE, &config, "auto", |o| {
        one_dc_op_iteration(o);
        o.allow_unconverged_dc_op = true;
    })
    .expect("--allow-unconverged-dc-op builds it");
    assert!(!built.dc_op.converged);
    assert!(
        built
            .generated
            .code
            .contains("pub const DC_OP_CONVERGED: bool = false;"),
        "the generated code records the unconverged start"
    );
}
