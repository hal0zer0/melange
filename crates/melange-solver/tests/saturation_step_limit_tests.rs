//! Newton step limit on the saturating-inductor branch-current row.
//!
//! Nothing limited the flux row: node damping covers node rows, pnjlim covers
//! devices. From deep saturation (`L_diff ~ L_air`) a Newton step that crosses
//! the knee is amps long and lands deep on the far side, where the slope is
//! flat again, and Newton 2-cycles. On a saturating RL (30 Ω + 1 Ω source,
//! 1 H, Isat 10 mA) under a ±20 V 100 Hz square, trapezoidal Newton exhausted
//! MAX_ITER on 398 samples per second; the sub-step recovered each one, but on
//! a finer timestep, so the committed current was 3.3 % off the circuit's own
//! 1× trapezoidal solution. The limit scales the step fraction (as pnjlim
//! does) so a step crossing the knee lands 2·Isat past it.
//!
//! Every site that can commit a sample carries the limit. A trapezoidal build
//! runs one solve routine twice (trapezoidal, then backward Euler), each with a
//! main loop, a sub-step and, with a railing op-amp, a pinned solve. The
//! trapezoidal main loop, its sub-step and the BE main loop each have a witness
//! that fails without their own limit. The BE sub-step is the same emitted code
//! as the trapezoidal one (its limit shows only as cost there: 148 -> 212
//! ns/sample on the latched witness), and the pinned solve starts from the
//! previous sample's current, so its steps do not cross the knee on any deck
//! tried; those two are checked structurally.

mod support;

use melange_solver::codegen::OpampRailMode;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const FS: f64 = 48000.0;

/// Saturating RL: 30 Ω + the 1 Ω source, 1 H, Isat 10 mA, steel floor.
const RL: &str = "limit\nR_1 in out 30\nL_1 out 0 1 ISAT=10m LAIR=3e-4\n";

/// The limiter line the emitter writes, one per Newton site.
const LIMIT_MARK: &str = "let i1 = ";

fn nodal_code(spice: &str, mode: Option<OpampRailMode>) -> String {
    let mna = MnaSystem::from_netlist(&Netlist::parse(spice).unwrap()).unwrap();
    let mut config = support::config_for_spice(spice, FS);
    config.output_nodes = vec![mna.node_map["out"] - 1];
    config.dc_block = false;
    if let Some(m) = mode {
        config.opamp_rail_mode = m;
    }
    support::generate_circuit_code_nodal(spice, &config).0
}

/// Drop the limiter lines numbered in `sites` (1-based, emission order).
fn without_limit(code: &str, sites: &[usize]) -> String {
    let mut n = 0;
    let out: Vec<&str> = code
        .lines()
        .filter(|l| {
            if l.contains(LIMIT_MARK) && l.contains("SAT_IND_0_AUG_ROW") {
                n += 1;
                return !sites.contains(&n);
            }
            true
        })
        .collect();
    out.join("\n")
}

struct Run {
    i_l: Vec<f64>,
    max_iter: u64,
    substep: u64,
    hold: u64,
}

/// 1 s of a ±`amp` square at `f`; `latched` forces the backward-Euler latch
/// before any sample, so every sample runs the BE fallback.
fn run(code: &str, amp: f64, f: f64, latched: bool, tag: &str) -> Run {
    let latch = if latched { "s.be_latched = true;" } else { "" };
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    {latch}
    let n = {FS:?} as usize;
    for k in 0..n {{
        let w = (2.0 * std::f64::consts::PI * {f:?} * k as f64 / {FS:?}).sin();
        let _ = process_sample(if w >= 0.0 {{ {amp:?} }} else {{ -{amp:?} }}, &mut s);
        println!(\"{{:e}}\", s.v_prev[SAT_IND_0_AUG_ROW]);
    }}
    println!(\"{{}} {{}} {{}}\", s.diag_nr_max_iter_count, s.diag_substep_count, s.diag_nr_hold_count);
}}"
    );
    let out = support::compile_and_run(code, &main, tag).stdout;
    let mut lines: Vec<&str> = out.lines().collect();
    let c: Vec<u64> = lines
        .pop()
        .unwrap()
        .split_whitespace()
        .map(|t| t.parse().unwrap())
        .collect();
    Run {
        i_l: lines.iter().map(|l| l.parse().unwrap()).collect(),
        max_iter: c[0],
        substep: c[1],
        hold: c[2],
    }
}

/// The circuit's own 1× trapezoidal solution, from a scalar recurrence of
/// V − R·i = dΦ/dt solved by bracketed Newton to 1e-16 (independent of the
/// generated code). Same drive indexing as `run`.
fn recurrence(r_total: f64, amp: f64, f: f64) -> Vec<f64> {
    let (l0, lair, isat) = (1.0f64, 3e-4f64, 10e-3f64);
    let (l_air, l_mag) = (lair * l0, (1.0 - lair) * l0);
    let phi = |i: f64| l_mag * isat * (i / isat).tanh() + l_air * i;
    let ld = |i: f64| {
        let c = (i / isat).clamp(-40.0, 40.0).cosh();
        l_mag / (c * c) + l_air
    };
    let h = 1.0 / FS;
    let (mut i, mut vprev) = (0.0f64, 0.0f64);
    let mut out = Vec::new();
    for k in 0..FS as usize {
        let w = (2.0 * std::f64::consts::PI * f * k as f64 / FS).sin();
        let v = if w >= 0.0 { amp } else { -amp };
        let rhs = phi(i) + 0.5 * h * (v + vprev) - 0.5 * h * r_total * i;
        let g = |x: f64| phi(x) + 0.5 * h * r_total * x - rhs;
        let (mut lo, mut hi) = (-2.0 * amp / r_total - 1.0, 2.0 * amp / r_total + 1.0);
        let mut y = i;
        for _ in 0..400 {
            let gy = g(y);
            if gy > 0.0 {
                hi = y;
            } else {
                lo = y;
            }
            let mut yn = y - gy / (ld(y) + 0.5 * h * r_total);
            if !(yn > lo && yn < hi) {
                yn = 0.5 * (lo + hi);
            }
            if (yn - y).abs() < 1e-16 {
                y = yn;
                break;
            }
            y = yn;
        }
        i = y;
        vprev = v;
        out.push(i);
    }
    out
}

fn worst_rel(a: &[f64], b: &[f64]) -> f64 {
    let pk = b.iter().fold(0.0f64, |m, &v| m.max(v.abs()));
    a.iter()
        .zip(b)
        .fold(0.0f64, |m, (x, y)| m.max((x - y).abs()))
        / pk
}

/// Main-loop site. With the limit the square solves on every sample at the
/// main loop and matches the 1× recurrence (Newton's stop is 1e-5 of the
/// per-sample increment). Without the main-loop limit, knee edges exhaust
/// MAX_ITER again.
#[test]
fn knee_crossings_solve_in_the_main_loop() {
    let code = nodal_code(RL, None);
    let r = run(&code, 20.0, 100.0, false, "satlim_main");
    assert_eq!(
        (r.max_iter, r.substep, r.hold),
        (0, 0, 0),
        "limited build: max_iter/substep/hold"
    );
    let e = worst_rel(&r.i_l, &recurrence(31.0, 20.0, 100.0));
    assert!(e < 1e-5, "i_L off the 1x recurrence by {e:e} of peak");

    let m = run(
        &without_limit(&code, &[1]),
        20.0,
        100.0,
        false,
        "satlim_main_mut",
    );
    assert!(
        m.max_iter > 100 && m.substep == m.max_iter && m.hold == 0,
        "without the main-loop limit: max_iter {} substep {} hold {}",
        m.max_iter,
        m.substep,
        m.hold
    );
}

/// Sub-step site. With the main-loop limit removed the sub-step takes the knee
/// edges; its own limit makes those sub-stepped samples land close enough that
/// fewer later edges fail. Removing it as well restores the unlimited build.
#[test]
fn substep_site_carries_the_limit() {
    let code = nodal_code(RL, None);
    let sub_limited = run(
        &without_limit(&code, &[1]),
        20.0,
        100.0,
        false,
        "satlim_sub",
    );
    let unlimited = run(
        &without_limit(&code, &[1, 2]),
        20.0,
        100.0,
        false,
        "satlim_sub_mut",
    );
    assert!(
        sub_limited.hold == 0 && unlimited.hold == 0,
        "holds: {} / {}",
        sub_limited.hold,
        unlimited.hold
    );
    assert!(
        unlimited.substep > sub_limited.substep,
        "sub-step limit made no difference: {} vs {} sub-steps",
        sub_limited.substep,
        unlimited.substep
    );
}

/// Backward-Euler main loop, reached on every sample by forcing the latch.
/// With its limit every knee edge solves there; without it Newton 2-cycles at
/// the edges and the BE sub-step has to take them.
#[test]
fn knee_crossings_solve_in_the_backward_euler_solve() {
    let code = nodal_code(RL, None);
    let r = run(&code, 20.0, 100.0, true, "satlim_be");
    assert_eq!(
        (r.max_iter, r.substep, r.hold),
        (0, 0, 0),
        "latched build: max_iter/substep/hold"
    );
    let m = run(
        &without_limit(&code, &[3]),
        20.0,
        100.0,
        true,
        "satlim_be_mut",
    );
    assert!(
        m.max_iter > 100 && m.substep == m.max_iter && m.hold == 0,
        "without the BE main-loop limit: max_iter {} substep {} hold {}",
        m.max_iter,
        m.substep,
        m.hold
    );
}

/// Parity: every Newton site that can commit a sample carries the limit. A
/// passive deck has 4 (trapezoidal and BE: main loop and sub-step each); a
/// railing op-amp driving the core adds a pinned solve to each instance (6).
#[test]
fn every_newton_site_carries_the_limit() {
    let count = |code: &str| {
        code.lines()
            .filter(|l| l.contains(LIMIT_MARK) && l.contains("SAT_IND_0_AUG_ROW"))
            .count()
    };
    assert_eq!(
        count(&nodal_code(RL, None)),
        4,
        "trap main, trap sub-step, BE main, BE sub-step"
    );
    let oa = "choke\nVcc vcc 0 DC 9\nR_b1 vcc vbias 100k\nR_b2 vbias 0 100k\nC_b vbias 0 10u\n\
              C_in in np 100n\nR_in np vbias 1Meg\nU1 np nm oa TL072\nR_f oa nm 500k\n\
              R_g nm ng 4.7k\nC_g ng 0 10u\nC_c oa n1 1u\nR_1 n1 n2 1k\n\
              L_sat n2 0 100m ISAT=2m CORE=gapped\nR_t n2 out 10k\nR_v out 0 100k\n\
              .model TL072 OA(AOL=200000 VCC=9 VEE=0)\n";
    let code = nodal_code(oa, Some(OpampRailMode::ActiveSet));
    let pinned = code
        .lines()
        .filter(|l| l.contains(LIMIT_MARK) && l.contains("v_pin[SAT_IND_0_AUG_ROW]"))
        .count();
    assert_eq!(
        (count(&code), pinned),
        (6, 2),
        "main, sub-step and pinned, in each of the trapezoidal and BE solves"
    );
}
