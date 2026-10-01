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

// ── The nodal Newton budget floor ───────────────────────────────────────

/// `MAX_ITER` as the generated code declares it.
fn emitted_max_iter(code: &str) -> usize {
    code.lines()
        .find_map(|l| {
            l.trim()
                .strip_prefix("pub const MAX_ITER: usize = ")
                .and_then(|r| r.strip_suffix(';'))
        })
        .unwrap_or_else(|| panic!("no `pub const MAX_ITER` in the generated code"))
        .parse()
        .unwrap()
}

/// A nodal build never ships fewer than `NODAL_MAX_ITER_FLOOR` Newton
/// iterations, so a `--max-iter` pin below it is refused rather than silently
/// raised (the pin would not be the budget that ships).
#[test]
fn max_iter_pin_below_the_nodal_floor_is_refused() {
    let floor = melange_solver::codegen::policy::NODAL_MAX_ITER_FLOOR;
    let err = refusal("nodal", |c| c.max_iterations = 50);
    assert!(
        err.contains("--max-iter 50")
            && err.contains(&format!("floor of {floor}"))
            && err.contains("Armijo line search")
            && err.contains(&format!("--max-iter {floor} or more")),
        "the refusal must name the pin, the floor and why: {err}"
    );
    let err = refusal("nodal", |c| c.max_iterations = floor - 1);
    assert!(err.contains(&format!("floor of {floor}")), "{err}");
}

/// A pin at or above the floor ships as pinned, and the build reports the
/// budget the code carries.
#[test]
fn max_iter_pin_at_or_above_the_nodal_floor_ships_as_pinned() {
    let floor = melange_solver::codegen::policy::NODAL_MAX_ITER_FLOOR;
    for pin in [floor, 150] {
        // Pinned through `BuildOptions` directly: the support mapping reads a
        // config budget equal to the default (100) as "not pinned".
        let config = support::config_for_spice(CLIPPER, 48000.0);
        let built =
            support::try_build_shipped_with(CLIPPER, &config, "nodal", |o| o.max_iter = Some(pin))
                .unwrap_or_else(|e| panic!("--max-iter {pin}: {e}"));
        assert_eq!(built.solver_label, "nodal");
        assert_eq!(built.max_iter, pin);
        assert_eq!(emitted_max_iter(&built.generated.code), pin);
    }
}

/// DK has no floor: a low pin ships as pinned.
#[test]
fn max_iter_pin_below_the_nodal_floor_ships_on_dk() {
    let mut config = support::config_for_spice(CLIPPER, 48000.0);
    config.max_iterations = 50;
    let built = support::build_shipped(CLIPPER, &config, "dk");
    assert_eq!(built.solver_label, "DK");
    assert_eq!(built.max_iter, 50);
    assert_eq!(emitted_max_iter(&built.generated.code), 50);
}

/// Unpinned, the auto-tuned budget is raised to the floor silently, as it
/// always was, and `Built::max_iter` is what the code carries — the value the
/// console prints and the provenance `Build:` line records.
#[test]
fn auto_tuned_budget_reports_the_value_that_ships() {
    let floor = melange_solver::codegen::policy::NODAL_MAX_ITER_FLOOR;
    for solver in ["dk", "nodal"] {
        let config = support::config_for_spice(CLIPPER, 48000.0);
        let built = support::build_shipped(CLIPPER, &config, solver);
        let shipped = emitted_max_iter(&built.generated.code);
        assert_eq!(built.max_iter, shipped, "--solver {solver}");
        assert!(
            built
                .generated
                .code
                .contains(&format!("max_iter={shipped},")),
            "--solver {solver}: the provenance Build: line must carry {shipped}"
        );
        if solver == "nodal" {
            assert!(shipped >= floor, "nodal auto budget {shipped} < {floor}");
        }
    }
}

/// The transistor-organ master oscillator: at 48 kHz the router picks DK, the
/// DK build refuses it (its DC operating point has a growing pole) and it is
/// built on nodal. The pin is checked on that route too.
const SELF_STARTING_OSCILLATOR: &str = "\
Transistor-organ master oscillator (LC tank, regenerative feedback) + squarer
Vrail rail 0 DC 8
Vvib vterm 0 DC 8
.runtime Vvib as v_osc_vterm
C_kick in b1 1n
R_e18 rail node_a 1.8k
C_e25 rail node_a 25u
L_fb node_b node_a 8.35m
R_180 node_b e1 180
Q_TN1G c1 b1 e1 SFT307
L_tank c1 tanklo 1.36
K1 L_tank L_fb 0.3
C_tank c1 tanklo 13.5n
R_47k tanklo b1 47k
R_27k2 tanklo 0 2.7k
R_10k rail b1 10k
R_27k vterm b1 27k
C_sq c1 sq_n 10n
R_sqb sq_n b2 47k
R_470k b2 0 470k
Q_TN2G c2 b2 rail SFT307
R_c2 c2 0 10k
C_fout c2 term_f 1u
R_fbleed term_f 0 100k
.model SFT307 PNP(IS=2e-7 BF=110 VAF=60 RB=50 RC=5 RE=1 CJE=60p CJC=25p TF=1n)
";

#[test]
fn max_iter_pin_below_the_floor_is_refused_when_dk_falls_back_to_nodal() {
    let mut config = support::config_for_spice(SELF_STARTING_OSCILLATOR, 48000.0);
    config.output_nodes = vec![support::node_index(SELF_STARTING_OSCILLATOR, "term_f")];
    let auto = support::build_shipped(SELF_STARTING_OSCILLATOR, &config, "auto");
    assert_eq!(
        auto.routing.route,
        melange_solver::codegen::routing::SolverRoute::DkSchur
    );
    assert_eq!(
        auto.solver_label, "nodal",
        "the DK build must fall back to nodal"
    );

    config.max_iterations = 50;
    let err = match support::try_build_shipped(SELF_STARTING_OSCILLATOR, &config, "auto") {
        Ok(_) => panic!("a sub-floor pin must be refused on the fallback route"),
        Err(e) => e,
    };
    assert!(
        err.contains("--max-iter 50") && err.contains("floor of"),
        "{err}"
    );
}
