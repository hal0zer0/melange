//! Rank-1 (Sherman–Morrison) pot updates on linear DK builds.
//!
//! A DK build with pots and no nonlinear devices consumes the dirty flag with
//! `apply_matrix_changes`: pot-only changes update `s` by one exact rank-1
//! step per changed pot; structural changes (switch, sample rate, reset, the
//! first call) and the two guards (conductance-ratio trigger, near-zero
//! denominator) take the full rebuild. Nonlinear builds keep the full rebuild.
//! The relative setter deadband applies to every pot setter on both routes.

mod support;

/// Linear DK witness: a pot between two live nodes, a `.runtime R` to ground,
/// a switch for the structural path.
const LINEAR: &str = "Rank-1 pot witness
R1 in a 1k
C1 a b 100n
Rpot b c 10k
R2 c 0 10k
C2 c 0 10n
Rg c 0 100k
R3 b out 2.2k
C3 out 0 4.7n
.pot Rpot 1k 1Meg
.runtime Rg 1k 10Meg as r_cell
.switch R2 10k 22k \"Load\"
";

/// Same network with a diode across the output: M = 1, so the build keeps
/// the full rebuild (K and S·N_i change with the pot; their rank-1 update is a
/// later landing).
const NONLINEAR: &str = "Rank-1 pot witness, nonlinear
R1 in a 1k
C1 a b 100n
Rpot b c 10k
R2 c 0 10k
C2 c 0 10n
Rg c 0 100k
R3 b out 2.2k
C3 out 0 4.7n
D1 out 0 DX
.model DX D(IS=2.52n N=1.752)
.pot Rpot 1k 1Meg
.runtime Rg 1k 10Meg as r_cell
";

fn build(spice: &str, route: &str, name: &str) -> String {
    let mut config = support::config_in_out_or_node1(spice, 44100.0);
    config.circuit_name = name.to_string();
    support::build_as_shipped(spice, &config, route).0
}

fn kv(line: &str, key: &str) -> f64 {
    line.split_whitespace()
        .find_map(|t| t.strip_prefix(&format!("{key}=")))
        .unwrap_or_else(|| panic!("no `{key}` in `{line}`"))
        .parse()
        .unwrap()
}

/// Entrywise relative error of `s` against `r` over entries above 1e-12·max|r|
/// (max-abs normalisation would hide the small entries).
const ENTRYWISE_FN: &str = "
fn entrywise_rel(s: &[[f64; N]; N], r: &[[f64; N]; N]) -> f64 {
    let floor = r.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs())) * 1e-12;
    let mut worst = 0.0f64;
    for i in 0..N { for j in 0..N {
        if r[i][j].abs() > floor { worst = worst.max(((s[i][j] - r[i][j]) / r[i][j]).abs()); }
    } }
    worst
}
";

#[test]
fn linear_dk_pot_build_emits_the_rank1_path() {
    let code = build(LINEAR, "dk", "rank1_linear");
    for needle in [
        "fn apply_matrix_changes(&mut self)",
        "fn rank1_update_s(&mut self, p: usize, q: Option<usize>, dg: f64) -> bool",
        "const RANK1_REBUILD_RATIO: f64 = 10.0;",
        "const RANK1_DENOM_MIN: f64 = 1e-6;",
        "pub structural_dirty: bool,",
        "pub diag_rank1_update_count: u64,",
        "pub diag_rank1_trigger_rebuild_count: u64,",
        "pub diag_rank1_fallback_count: u64,",
        "state.apply_matrix_changes();",
    ] {
        assert!(
            code.contains(needle),
            "linear DK pot build must emit `{needle}`"
        );
    }
    // The dirty block is the only consumer, and it takes the rank-1 path.
    assert_eq!(code.matches("if state.matrices_dirty").count(), 1);
    assert!(
        !code.contains("        state.rebuild_matrices();\n        state.matrices_dirty = false;")
    );
    // Switch setters flag a structural change; reset does too.
    let sw = code.find("pub fn set_switch_0(").expect("switch setter");
    let sw_body = &code[sw..sw + code[sw..].find("\n    }\n").unwrap()];
    assert!(
        sw_body.contains("self.structural_dirty = true;"),
        "switch setter must flag structural:\n{sw_body}"
    );
    let rs = code.find("pub fn reset(&mut self)").expect("reset");
    let rs_body = &code[rs..rs + code[rs..].find("\n    }\n").unwrap()];
    assert!(
        rs_body.contains("self.structural_dirty = true;"),
        "reset must flag structural:\n{rs_body}"
    );
    assert!(rs_body.contains("self.diag_rank1_update_count = 0;"));
    // The full rebuild re-syncs the bookkeeping for every pot.
    assert!(code.contains("self.pot_0_g_stamped = 1.0 / self.pot_0_resistance;"));
    assert!(code.contains("self.pot_1_g_stamped = 1.0 / self.pot_1_resistance;"));
}

#[test]
fn nonlinear_dk_pot_build_keeps_the_full_rebuild() {
    let code = build(NONLINEAR, "dk", "rank1_nonlinear");
    for needle in [
        "apply_matrix_changes",
        "rank1_update_s",
        "RANK1_REBUILD_RATIO",
        "structural_dirty",
        "diag_rank1_update_count",
    ] {
        assert!(
            !code.contains(needle),
            "nonlinear DK build must not emit `{needle}`"
        );
    }
    assert!(
        code.contains("        state.rebuild_matrices();\n        state.matrices_dirty = false;")
    );
}

#[test]
fn rank1_path_matches_the_full_rebuild_over_decade_sweeps() {
    // Two states, same input and the same pot trajectory: `a` is forced onto
    // the full rebuild every sample (structural_dirty), `b` takes the rank-1
    // path. Phase 1 sweeps the pot up two decades (conductance falling: the
    // ratio trigger fires once, at the first decade) while the cell sweeps
    // down; phase 2 reverses both (the cell's fall fires the trigger once).
    let code = build(LINEAR, "dk", "rank1_sweep");
    let main = format!(
        "{ENTRYWISE_FN}
fn main() {{
    let fs = 44100.0;
    let mut a = CircuitState::default(); a.set_sample_rate(fs);
    let mut b = CircuitState::default(); b.set_sample_rate(fs);
    let n = 3000usize;
    let (mut dy, mut srel, mut peak) = (0.0f64, 0.0f64, 0.0f64);
    for k in 0..n {{
        let t = k as f64 / 1500.0;
        let (rp, rc) = if k < 1500 {{ (1e4 * 100f64.powf(t), 1e5 * 0.01f64.powf(t)) }}
                       else {{ (1e6 * 0.01f64.powf(t - 1.0), 1e3 * 100f64.powf(t - 1.0)) }};
        a.set_pot_0(rp); a.set_runtime_R_r_cell(rc); a.structural_dirty = true;
        b.set_pot_0(rp); b.set_runtime_R_r_cell(rc);
        let x = 0.5 * (2.0 * std::f64::consts::PI * 440.0 * k as f64 / fs).sin();
        let ya = process_sample(x, &mut a)[0];
        let yb = process_sample(x, &mut b)[0];
        peak = peak.max(ya.abs()); dy = dy.max((ya - yb).abs());
        if k % 100 == 99 {{ srel = srel.max(entrywise_rel(&b.s, &a.s)); }}
    }}
    println!(\"peak={{peak:e}} dy={{dy:e}} srel={{srel:e}} updates={{}} triggers={{}} fallbacks={{}} ref_updates={{}}\",
        b.diag_rank1_update_count, b.diag_rank1_trigger_rebuild_count, b.diag_rank1_fallback_count, a.diag_rank1_update_count);
}}
"
    );
    let out = support::compile_and_run(&code, &main, "rank1_sweep");
    let line = out
        .stdout
        .lines()
        .find(|l| l.starts_with("peak="))
        .expect("result line");
    let (peak, dy, srel) = (kv(line, "peak"), kv(line, "dy"), kv(line, "srel"));
    let (updates, triggers, fallbacks, ref_updates) = (
        kv(line, "updates"),
        kv(line, "triggers"),
        kv(line, "fallbacks"),
        kv(line, "ref_updates"),
    );
    assert!(peak > 1e-3, "witness must produce signal: {line}");
    assert!(
        dy <= 1e-10,
        "rank-1 output must match the full rebuild: {line}"
    );
    assert!(
        srel <= 1e-8,
        "rank-1 S must match the full rebuild entrywise: {line}"
    );
    assert_eq!(
        fallbacks, 0.0,
        "no denominator fallback on a well-conditioned deck: {line}"
    );
    assert_eq!(
        triggers, 2.0,
        "one ratio-trigger rebuild per falling decade sweep: {line}"
    );
    assert_eq!(
        ref_updates, 0.0,
        "the forced-structural state never takes the rank-1 path: {line}"
    );
    // Sample 0 sets both pots to their nominal values (no change, nothing
    // consumed); every later sample changes both: two updates each, except the
    // trigger samples, which rebuild in full instead.
    assert_eq!(
        updates,
        2.0 * 2999.0 - 2.0 * triggers,
        "one update per changed pot per sample: {line}"
    );
}

#[test]
fn structural_changes_and_resync_take_the_full_rebuild() {
    let code = build(LINEAR, "dk", "rank1_structural");
    let main = format!(
        "{ENTRYWISE_FN}
fn main() {{
    let fs = 44100.0;
    let x = 0.1;
    // Fresh state, no rate call: the first consumption is structural (the
    // bookkeeping starts dirty against the baked matrices).
    let mut c = CircuitState::default();
    c.set_pot_0(30e3); process_sample(x, &mut c);
    let c0 = c.diag_rank1_update_count;
    // A same-rate set_sample_rate with nominal pots loads the baked inverse
    // and re-syncs, so the first pot change after it is already rank-1.
    let mut b = CircuitState::default(); b.set_sample_rate(fs);
    b.set_pot_0(30e3); process_sample(x, &mut b);
    let u0 = b.diag_rank1_update_count;
    // Pot-only change: rank-1.
    b.set_pot_0(50e3); process_sample(x, &mut b);
    let u1 = b.diag_rank1_update_count;
    // Switch flip: structural, and the result equals a fresh full rebuild.
    b.set_switch_0(1); process_sample(x, &mut b);
    let u2 = b.diag_rank1_update_count;
    let mut r = CircuitState::default(); r.set_sample_rate(fs);
    r.set_pot_0(50e3); r.set_switch_0(1); process_sample(x, &mut r);
    let e_switch = entrywise_rel(&b.s, &r.s);
    // Same-rate set_sample_rate with pots off default rebuilds in full and
    // re-syncs; the next pot change is rank-1 again and still exact.
    b.set_sample_rate(fs);
    let sd_after_rate = b.structural_dirty;
    b.set_pot_0(60e3); process_sample(x, &mut b);
    let u3 = b.diag_rank1_update_count;
    r.set_pot_0(60e3); r.structural_dirty = true; process_sample(x, &mut r);
    let e_rate = entrywise_rel(&b.s, &r.s);
    // reset(): structural, counters cleared, baked matrices + nominal pots.
    b.reset();
    let sd_reset = b.structural_dirty;
    let u_reset = b.diag_rank1_update_count;
    process_sample(x, &mut b);
    let u4 = b.diag_rank1_update_count;
    b.set_pot_0(12e3); process_sample(x, &mut b);
    let u5 = b.diag_rank1_update_count;
    println!(\"c0={{c0}} u0={{u0}} u1={{u1}} u2={{u2}} u3={{u3}} u4={{u4}} u5={{u5}} e_switch={{e_switch:e}} e_rate={{e_rate:e}} sd_after_rate={{}} sd_reset={{}} u_reset={{u_reset}} triggers={{}} fallbacks={{}}\",
        sd_after_rate as u8, sd_reset as u8, b.diag_rank1_trigger_rebuild_count, b.diag_rank1_fallback_count);
}}
"
    );
    let out = support::compile_and_run(&code, &main, "rank1_structural");
    let line = out
        .stdout
        .lines()
        .find(|l| l.starts_with("c0="))
        .expect("result line");
    assert_eq!(
        kv(line, "c0"),
        0.0,
        "fresh state: first consumption is a full rebuild: {line}"
    );
    assert_eq!(kv(line, "u0"), 1.0, "after a same-rate rate call the baked inverse is re-synced; first change is rank-1: {line}");
    assert_eq!(
        kv(line, "u1"),
        2.0,
        "pot-only change is one rank-1 update: {line}"
    );
    assert_eq!(
        kv(line, "u2"),
        2.0,
        "switch flip takes the full rebuild, no update: {line}"
    );
    assert_eq!(
        kv(line, "e_switch"),
        0.0,
        "after a structural rebuild S is the full rebuild's S: {line}"
    );
    assert_eq!(
        kv(line, "sd_after_rate"),
        0.0,
        "set_sample_rate rebuilt in full and re-synced: {line}"
    );
    assert_eq!(
        kv(line, "u3"),
        3.0,
        "pot change after the rate call is rank-1 again: {line}"
    );
    assert!(kv(line, "e_rate") <= 1e-9, "and still exact: {line}");
    assert_eq!(kv(line, "sd_reset"), 1.0, "reset flags structural: {line}");
    assert_eq!(
        kv(line, "u_reset"),
        0.0,
        "reset clears the counters: {line}"
    );
    assert_eq!(
        kv(line, "u4"),
        0.0,
        "the consumption after reset is a full rebuild: {line}"
    );
    assert_eq!(kv(line, "u5"), 1.0, "then rank-1 resumes: {line}");
    assert_eq!(kv(line, "fallbacks"), 0.0, "{line}");
}

#[test]
fn pot_setter_deadband_is_relative_on_both_routes() {
    let dk = build(LINEAR, "dk", "rank1_deadband_dk");
    assert!(
        dk.contains(
            "if (r - self.pot_0_resistance).abs() <= 1e-12 * self.pot_0_resistance { return; }"
        ),
        "DK pot setter deadband must be relative to the stored value"
    );
    assert!(
        !dk.contains(".abs() < 1e-12 { return; }"),
        "no absolute 1e-12 deadband left on DK"
    );
    let nodal = build(NONLINEAR, "nodal", "rank1_deadband_nodal");
    assert!(
        nodal.contains(
            "if (r - self.pot_0_resistance).abs() <= 1e-12 * self.pot_0_resistance { return; }"
        ),
        "nodal pot setter deadband must be relative to the stored value"
    );
    // Behaviour: one part in 10^13 is ignored, one part in 10^11 is not — at
    // 20 kΩ the old absolute 1e-12 Ω threshold would have accepted both.
    let main = "
fn main() {
    let mut b = CircuitState::default(); b.set_sample_rate(44100.0);
    b.set_pot_0(20e3); process_sample(0.0, &mut b);
    let r = b.pot_0_resistance;
    b.set_pot_0(r * (1.0 + 1e-13)); let d1 = b.matrices_dirty;
    b.set_pot_0(r * (1.0 + 1e-11)); let d2 = b.matrices_dirty;
    println!(\"d1={} d2={}\", d1 as u8, d2 as u8);
}
";
    let out = support::compile_and_run(&dk, main, "rank1_deadband_run");
    let line = out
        .stdout
        .lines()
        .find(|l| l.starts_with("d1="))
        .expect("result line");
    assert_eq!(
        kv(line, "d1"),
        0.0,
        "1e-13 relative change is inside the deadband: {line}"
    );
    assert_eq!(
        kv(line, "d2"),
        1.0,
        "1e-11 relative change marks dirty: {line}"
    );
}
