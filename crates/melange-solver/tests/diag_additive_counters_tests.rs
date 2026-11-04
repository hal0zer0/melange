//! Two additive diagnostic counters on the generated `CircuitState`:
//! `diag_nr_iter_total` (every build: one count per iteration of every Newton
//! loop on the sample path) and `diag_sparse_reject_count` (full-LU builds with
//! a sparse LU schedule: the sparse factor refused its static pivots and the
//! dense factor ran). Each is declared, initialised and zeroed by `reset()`,
//! and increments where it should: the iteration total at the top of every
//! Newton loop, so a run of N samples on a nonlinear deck counts at least N;
//! the sparse rejection on the dense-fallback branch of every
//! `sparse_lu_factor` call. A deck whose sparse factor is refused at runtime
//! is not constructible on purpose, so that branch is checked in the source.

mod support;

use melange_solver::codegen::{NodalSubPathOverride, NoiseMode};

/// A diode-loaded RC ladder: enough nodes (N >= 8, tridiagonal) for the
/// nodal build to plan a sparse LU schedule, and a Newton loop on every route.
const LADDER: &str = "Diode ladder for the additive counters
R1 in n1 1k
C1 n1 0 10n
R2 n1 n2 1k
C2 n2 0 10n
R3 n2 n3 1k
C3 n3 0 10n
R4 n3 n4 1k
C4 n4 0 10n
D2 n4 0 DX
R5 n4 n5 1k
C5 n5 0 10n
R6 n5 n6 1k
C6 n6 0 10n
R7 n6 n7 1k
C7 n7 0 10n
R8 n7 out 1k
C8 out 0 10n
D1 out 0 DX
.model DX D(IS=2.52n N=1.752)
.end
";

const SAMPLES: usize = 2000;

fn build(route: (&str, NodalSubPathOverride)) -> String {
    let config = melange_solver::codegen::CodegenConfig {
        nodal_sub_path_override: route.1,
        ..support::config_in_out_or_node1(LADDER, 48000.0)
    };
    support::try_build_shipped_with(LADDER, &config, route.0, |o| {
        o.oversampling = Some(1);
        o.noise_mode = NoiseMode::Off;
    })
    .unwrap_or_else(|e| panic!("build refused: {e}"))
    .generated
    .code
}

fn main_for(code: &str) -> String {
    let rejects = if code.contains("pub diag_sparse_reject_count: u64") {
        "s.diag_sparse_reject_count"
    } else {
        "0u64"
    };
    format!(
        "
fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let warm = s.diag_nr_iter_total;
    for k in 0..{SAMPLES} {{
        let x = 0.5 * (2.0 * std::f64::consts::PI * 997.0 * k as f64 / 48000.0).sin();
        let _ = process_sample(x, &mut s);
    }}
    let total = s.diag_nr_iter_total - warm;
    let rejects = {rejects};
    s.reset();
    let after_reset = s.diag_nr_iter_total;
    println!(\"warm={{warm}} total={{total}} rejects={{rejects}} after_reset={{after_reset}}\");
}}
"
    )
}

fn kv(line: &str, key: &str) -> u64 {
    line.split_whitespace()
        .find_map(|t| t.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("no `{key}` in `{line}`"))
        .parse()
        .unwrap()
}

/// The previous non-blank line of `code` before the line holding byte
/// offset `at`.
fn line_before(code: &str, at: usize) -> &str {
    let line_start = code[..at].rfind('\n').map(|i| i + 1).unwrap_or(0);
    code[..line_start]
        .lines()
        .rev()
        .find(|l| !l.trim().is_empty())
        .unwrap_or("")
}

/// The line that follows the line containing byte offset `at`.
fn line_after(code: &str, at: usize) -> &str {
    let rest = &code[at..];
    let nl = rest.find('\n').map(|i| i + 1).unwrap_or(rest.len());
    rest[nl..].lines().next().unwrap_or("")
}

/// The iteration total is declared, initialised, zeroed by `reset()`, and
/// every increment sits at the top of a Newton loop (the line after a
/// `for … in 0..` header), so each counts exactly one iteration.
fn assert_iter_total_wiring(code: &str, tag: &str) {
    assert!(
        code.contains("pub diag_nr_iter_total: u64,"),
        "{tag}: field not declared"
    );
    assert!(
        code.contains("diag_nr_iter_total: 0,"),
        "{tag}: field not initialised"
    );
    assert!(
        code.contains("self.diag_nr_iter_total = 0;"),
        "{tag}: reset() does not zero the field"
    );
    let needle = "diag_nr_iter_total += 1;";
    let sites: Vec<usize> = code.match_indices(needle).map(|(i, _)| i).collect();
    assert!(!sites.is_empty(), "{tag}: no increment emitted");
    for at in &sites {
        let header = line_before(code, *at).trim();
        assert!(
            header.starts_with("for ") && header.contains(" in 0..") && header.ends_with('{'),
            "{tag}: an increment is not the first statement of a Newton loop; preceded by `{header}`"
        );
    }
}

fn run(code: &str, tag: &str) -> (u64, u64, u64, u64) {
    let out = support::compile_and_run(code, &main_for(code), tag);
    let line = out.stdout.trim();
    assert!(!line.is_empty(), "{tag}: no output:\n{}", out.stderr);
    let warm = kv(line, "warm");
    let total = kv(line, "total");
    let rejects = kv(line, "rejects");
    let after_reset = kv(line, "after_reset");
    // Every sample runs at least one Newton iteration on a nonlinear deck.
    assert!(
        total >= SAMPLES as u64,
        "{tag}: {SAMPLES} samples counted only {total} Newton iterations: {line}"
    );
    // reset() zeroes the count; what remains is at most its own warm-up,
    // which is the same warm-up `default()` ran.
    assert!(
        after_reset <= warm,
        "{tag}: reset() left {after_reset} iterations (default's warm-up counted {warm}): {line}"
    );
    (warm, total, rejects, after_reset)
}

#[test]
fn nr_iter_total_counts_every_newton_iteration_on_nodal_full_lu() {
    let code = build(("nodal", NodalSubPathOverride::FullLu));
    assert_iter_total_wiring(&code, "full_lu");
    let (_, total, rejects, _) = run(&code, "iter_total_full_lu");
    assert!(rejects <= total, "rejections are a subset of iterations");
}

#[test]
fn nr_iter_total_counts_every_newton_iteration_on_nodal_schur() {
    let code = build(("nodal", NodalSubPathOverride::Schur));
    assert_iter_total_wiring(&code, "schur");
    run(&code, "iter_total_schur");
}

#[test]
fn nr_iter_total_counts_every_newton_iteration_on_dk() {
    let code = build(("dk", NodalSubPathOverride::Auto));
    assert_iter_total_wiring(&code, "dk");
    // DK's loop is `solve_nonlinear`, shared by the trapezoidal solve and the
    // backward-Euler fallback: one loop, one increment.
    assert_eq!(
        code.matches("diag_nr_iter_total += 1;").count(),
        1,
        "dk: one Newton loop"
    );
    run(&code, "iter_total_dk");
}

/// The sparse rejection counter exists only where the branch does: a full-LU
/// build with a sparse schedule. There it is declared, initialised, zeroed,
/// and increments first thing on the dense-fallback `else` of EVERY
/// `sparse_lu_factor` call (the primary solve and the backward-Euler
/// instance), nowhere else.
#[test]
fn sparse_reject_count_increments_on_the_dense_fallback_branch() {
    let code = build(("nodal", NodalSubPathOverride::FullLu));
    assert!(
        code.contains("fn sparse_lu_factor("),
        "the ladder must plan a sparse LU schedule for this test to mean anything"
    );
    assert!(code.contains("pub diag_sparse_reject_count: u64,"));
    assert!(code.contains("diag_sparse_reject_count: 0,"));
    assert!(code.contains("self.diag_sparse_reject_count = 0;"));

    let calls: Vec<usize> = code
        .match_indices("if sparse_lu_factor(")
        .map(|(i, _)| i)
        .collect();
    assert!(!calls.is_empty(), "no runtime sparse factor call");
    for at in &calls {
        let rest = &code[*at..];
        let else_at = rest
            .find("} else {")
            .expect("the sparse factor call has a dense-fallback branch");
        let first = line_after(rest, else_at).trim();
        assert_eq!(
            first, "state.diag_sparse_reject_count += 1;",
            "the dense-fallback branch's first statement counts the rejection"
        );
    }
    assert_eq!(
        code.matches("diag_sparse_reject_count += 1;").count(),
        calls.len(),
        "one increment per sparse factor call, nowhere else"
    );
}

#[test]
fn sparse_reject_count_is_absent_where_no_sparse_factor_runs() {
    for (tag, route) in [
        ("schur", ("nodal", NodalSubPathOverride::Schur)),
        ("dk", ("dk", NodalSubPathOverride::Auto)),
    ] {
        let code = build(route);
        assert!(
            !code.contains("diag_sparse_reject_count"),
            "{tag}: declares a counter for a branch it does not emit"
        );
    }
}
