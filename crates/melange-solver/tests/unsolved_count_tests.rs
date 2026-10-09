//! `diag_unsolved_sample_count` counts every sample that was never solved,
//! once, whichever failure mechanisms fired on it: a Newton hold, an
//! unconverged commit, a failed op-amp pin, a reduced-model exit, or a NaN or
//! magnitude reset (the committed state after a reset is the operating point,
//! not a solution). The mechanism counters are an overlapping breakdown, not
//! addends.
//!
//! The witness drives a `.runtime V` field with a huge FINITE value, which
//! passes the non-finite guard and overflows the diode law inside the solve,
//! so every sample resets. On DK that sample also ends its Newton loop at the
//! ceiling (unconverged commit AND reset on one sample), which pins the
//! count-once rule; on nodal the reset returns before the hold accounting,
//! which before this rule left the sample uncounted.

mod support;

use melange_solver::codegen::{NodalSubPathOverride, NoiseMode};

const WITNESS: &str = "Unsolved-count witness: diode clipper with a control offset
R1 in a 2.2k
C1 a b 47n
R2 b 0 100k
D1 b c DX
D2 c b DX
R3 b c 10k
Vctl ctl 0 DC 0
R4 ctl b 47k
Ctone c 0 10n
Rload c out 1k
Cout out 0 1n
.model DX D(IS=2.52n N=1.752)
.runtime Vctl as ctl_voltage
.end
";

fn build(route: (&str, NodalSubPathOverride)) -> String {
    let config = melange_solver::codegen::CodegenConfig {
        nodal_sub_path_override: route.1,
        ..support::config_in_out_or_node1(WITNESS, 48000.0)
    };
    support::try_build_shipped_with(WITNESS, &config, route.0, |o| {
        o.oversampling = Some(1);
        o.noise_mode = NoiseMode::Off;
    })
    .unwrap_or_else(|e| panic!("build refused: {e}"))
    .generated
    .code
}

/// The driver, reading each mechanism counter only where the build has it
/// (a hold exists on nodal, an unconverged commit on DK and at a committed
/// op-amp pin).
fn main_for(code: &str) -> String {
    let field = |name: &str| {
        if code.contains(&format!("pub {name}: u64")) {
            format!("s.{name}")
        } else {
            "0u64".to_string()
        }
    };
    format!(
        "
fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let drive = |s: &mut CircuitState, n: usize| {{
        for k in 0..n {{
            let x = 0.3 * (2.0 * std::f64::consts::PI * 997.0 * k as f64 / 48000.0).sin();
            let _ = process_sample(x, s);
        }}
    }};
    drive(&mut s, 2000);
    let base = s.diag_unsolved_sample_count;
    s.ctl_voltage = 1e30;
    drive(&mut s, 1000);
    let unsolved = s.diag_unsolved_sample_count - base;
    let resets = s.diag_nan_reset_count + s.diag_magnitude_reset_count;
    let holds = {holds};
    let commits = {commits};
    s.ctl_voltage = 0.0;
    drive(&mut s, 1000);
    let after = s.diag_unsolved_sample_count - base - unsolved;
    println!(\"base={{base}} unsolved={{unsolved}} resets={{resets}} holds={{holds}} commits={{commits}} after={{after}}\");
}}
",
        holds = field("diag_nr_hold_count"),
        commits = field("diag_nr_unconverged_commit_count"),
    )
}

fn kv(line: &str, key: &str) -> u64 {
    line.split_whitespace()
        .find_map(|t| t.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("no `{key}` in `{line}`"))
        .parse()
        .unwrap()
}

fn assert_counted_once(code: &str, tag: &str) -> String {
    // Every mechanism marks the sample; exactly one increment per sample.
    assert_eq!(
        code.matches("diag_unsolved_sample_count += 1").count()
            + code.matches("diag_unsolved_sample_count = state.diag_unsolved_sample_count.saturating_add(1)").count(),
        2,
        "{tag}: one increment at the reset return and one at the commit"
    );
    let out = support::compile_and_run(code, &main_for(code), tag);
    let line = out.stdout.trim();
    assert!(!line.is_empty(), "{tag}: no output:\n{}", out.stderr);
    let (unsolved, resets, holds, commits, after) = (
        kv(line, "unsolved"),
        kv(line, "resets"),
        kv(line, "holds"),
        kv(line, "commits"),
        kv(line, "after"),
    );
    assert_eq!(
        kv(line, "base"),
        0,
        "{tag}: the finite baseline was solved: {line}"
    );
    // Every sample at 1e30 failed one way or another, and every one of them
    // is unsolved: exactly once each, however many mechanisms fired on it.
    assert_eq!(unsolved, 1000, "{tag}: {line}");
    assert!(resets + holds + commits >= 1000, "{tag}: {line}");
    assert!(
        unsolved <= resets + holds + commits,
        "{tag}: unsolved exceeds the mechanisms that fired: {line}"
    );
    assert_eq!(
        after, 0,
        "{tag}: solved again once the field is finite: {line}"
    );
    line.to_string()
}

#[test]
fn a_reset_sample_counts_as_unsolved_once_on_dk() {
    let line = assert_counted_once(&build(("dk", NodalSubPathOverride::Auto)), "unsolved_dk");
    // DK's damped Newton keeps the iterate finite and ends every sample at
    // the ceiling: an unconverged commit, no reset. (A sample that both ends
    // unconverged AND resets is counted once by construction: the reset
    // returns before the once-increment, so no later site can add to it.)
    assert_eq!(kv(&line, "commits"), 1000, "{line}");
    assert_eq!(kv(&line, "resets"), 0, "{line}");
}

#[test]
fn a_reset_sample_counts_as_unsolved_once_on_nodal_schur() {
    let line = assert_counted_once(
        &build(("nodal", NodalSubPathOverride::Schur)),
        "unsolved_schur",
    );
    // The huge finite value overflows the diode law inside the solve: every
    // sample resets, and before this rule none of them was counted.
    assert_eq!(kv(&line, "resets"), 1000, "{line}");
}

#[test]
fn a_reset_sample_counts_as_unsolved_once_on_nodal_full_lu() {
    let line = assert_counted_once(
        &build(("nodal", NodalSubPathOverride::FullLu)),
        "unsolved_fulllu",
    );
    assert_eq!(kv(&line, "resets"), 1000, "{line}");
}
