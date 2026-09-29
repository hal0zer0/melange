//! `diag_unsolved_sample_count` is present on every generated build and counts
//! every unsolved sample, whatever the mechanism.
//!
//! A plugin test asserts this one field is zero; it must not need to know
//! which route its circuit compiles to or how that route fails (the death-spiral
//! hold on nodal builds, a committed unconverged iterate on DK and at a failed
//! op-amp pin). Those mechanism counters stay as detail.

mod support;

use melange_solver::codegen::{CodegenConfig, NodalSubPathOverride};

const LINEAR: &str = "rc\nR1 in out 1k\nC1 out 0 100n\n";
const DIODE: &str =
    "diode stage\nR1 in out 1k\nC1 out 0 100n\nD1 out 0 DX\n.model DX D(IS=1e-14)\n";

#[derive(Clone, Copy, Debug)]
enum Route {
    Dk,
    Schur,
    FullLu,
}

fn code(deck: &str, route: Route, be: bool) -> String {
    let mut c: CodegenConfig = support::config_for_spice(deck, 48000.0);
    c.backward_euler = be;
    match route {
        Route::Dk => {
            c.force_trap = !be;
            support::generate_circuit_code(deck, &c).0
        }
        Route::Schur | Route::FullLu => {
            c.nodal_sub_path_override = if matches!(route, Route::Schur) {
                NodalSubPathOverride::Schur
            } else {
                NodalSubPathOverride::FullLu
            };
            support::generate_circuit_code_nodal(deck, &c).0
        }
    }
}

/// Render 200 samples; print the unified count and the mechanism counters the
/// build declares.
fn counts(code: &str, tag: &str) -> (u64, u64) {
    let detail = ["diag_nr_hold_count", "diag_nr_unconverged_commit_count"]
        .into_iter()
        .filter(|f| code.contains(&format!("pub {f}: ")))
        .map(|f| format!("s.{f}"))
        .collect::<Vec<_>>();
    let detail = if detail.is_empty() {
        "0u64".to_string()
    } else {
        detail.join(" + ")
    };
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    for i in 0..200usize {{
        let _ = process_sample(0.5 * (i as f64 * 0.05).sin(), &mut s);
    }}
    println!(\"unified={{}}\", s.diag_unsolved_sample_count);
    println!(\"detail={{}}\", {detail});
}}"
    );
    let out = support::compile_and_run(code, &main, tag);
    (
        out.parse_kv("unified").unwrap() as u64,
        out.parse_kv("detail").unwrap() as u64,
    )
}

#[test]
fn every_build_declares_the_unified_count() {
    for (deck, m) in [(LINEAR, 0), (DIODE, 1)] {
        for route in [Route::Dk, Route::Schur, Route::FullLu] {
            for be in [false, true] {
                let tag = format!("unsolved_m{m}_{route:?}_be{be}");
                let code = code(deck, route, be);
                assert!(
                    code.contains("pub diag_unsolved_sample_count: u64"),
                    "{tag}: the unified count is not declared"
                );
                assert_eq!(
                    counts(&code, &tag),
                    (0, 0),
                    "{tag}: a clean render counts nothing"
                );
            }
        }
    }
}

/// With every Newton loop cut to zero iterations (a test-only patch of the
/// generated code), every sample is unsolved, warm-up included: DK commits it,
/// nodal holds it.
/// The unified count must see each one, and equal the mechanism counters.
#[test]
fn the_unified_count_sees_every_mechanism() {
    for route in [Route::Dk, Route::Schur, Route::FullLu] {
        let tag = format!("unsolved_forced_{route:?}");
        let code = code(DIODE, route, false);
        let loops = code.matches("in 0..MAX_ITER {").count();
        assert!(loops >= 1, "{tag}: Newton loop not found");
        let code = code.replace("in 0..MAX_ITER {", "in 0..0usize {");
        let (unified, detail) = counts(&code, &tag);
        // Every rendered sample, plus any warm-up samples `default()` runs
        // through the same solve (nodal builds: 50).
        assert!(
            unified >= 200,
            "{tag}: every sample is unsolved ({unified})"
        );
        assert_eq!(
            unified, detail,
            "{tag}: unified count vs the mechanism counters"
        );
    }
}

/// The third mechanism: a failed op-amp pin whose iterate is committed (M = 0,
/// both nodal routes). A comparator that rails on every half cycle, with the
/// pinned Newton cut to zero iterations.
#[test]
fn the_unified_count_sees_a_committed_pin() {
    use melange_solver::codegen::OpampRailMode;
    let deck = "comparator\nRleak in 0 1Meg\nU1 in 0 out OX\nRl out 0 10k\n\
                .model OX OA(AOL=100000 VCC=15 VEE=-15)\n";
    for (sub_path, sp) in [
        (NodalSubPathOverride::Schur, "schur"),
        (NodalSubPathOverride::FullLu, "full_lu"),
    ] {
        let tag = format!("unsolved_pin_{sp}");
        let mut c = support::config_for_spice(deck, 48000.0);
        c.opamp_rail_mode = OpampRailMode::ActiveSet;
        c.nodal_sub_path_override = sub_path;
        let code = support::generate_circuit_code_nodal(deck, &c).0;
        assert!(
            code.contains("pub diag_nr_unconverged_commit_count: "),
            "{tag}: a committed pin is counted"
        );
        let code = code.replace("in 0..MAX_ITER {", "in 0..0usize {");
        let (unified, detail) = counts(&code, &tag);
        assert!(
            unified > 100,
            "{tag}: the pin must engage and fail ({unified})"
        );
        assert_eq!(
            unified, detail,
            "{tag}: unified count vs the mechanism counters"
        );
    }
}
