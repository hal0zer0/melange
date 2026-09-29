//! The full-LU adaptive sub-step solves the circuit the knobs describe.
//!
//! It used to build its matrices from the compile-time `G`/`C`, while
//! `.pot`/`.switch` setters write the working copies `rebuild_matrices` reads.
//! With a knob off its default, a sub-stepped sample solved the NOMINAL
//! circuit, and passed its own convergence checks (they were built from the
//! same wrong matrix), so it was committed as converged. On a saturating RL
//! with its series pot at 50 Ω (nominal 99 Ω), ±5 V 30 Hz square: one
//! sub-step, then 47200 of 96000 samples held with the inductor current
//! frozen, where a native 50 Ω deck sub-steps 119 times and holds none.
//!
//! Each witness below runs a knob build (knob moved off default before the
//! drive) against a native build of the same circuit with that value
//! written in. The drive must reach the sub-step, or the witness says
//! nothing; then both builds take the same path and give the same output.
//!
//! The route into the sub-step is the saturating core's knee edges with the
//! main loop's flux-row step limit removed (from both builds alike): without
//! it trapezoidal Newton 2-cycles at those edges and hands them to the
//! sub-step, which is the path under test. The circuit itself is unchanged.

mod support;

use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const FS: f64 = 48000.0;

fn node(spice: &str, name: &str) -> usize {
    let mna = MnaSystem::from_netlist(&Netlist::parse(spice).unwrap()).unwrap();
    mna.node_map[name] - 1
}

/// Render 1 s of a square wave (after 10 silent samples, during which
/// `setup` has run) and print every output sample plus the counters.
fn run(spice: &str, setup: &str, amp: f64, f: f64, tag: &str) -> (Vec<f64>, u64, u64) {
    let mut config = support::config_for_spice(spice, FS);
    config.output_nodes = vec![node(spice, "out")];
    config.dc_block = false;
    let full = support::generate_circuit_code_nodal(spice, &config).0;
    assert!(
        full.contains("local refinement of the failing sub-step"),
        "{tag}: build has no adaptive sub-step"
    );
    // Drop the first (main-loop) flux-row step limit; see the module doc.
    let mut dropped = 0;
    let code: String = full
        .lines()
        .filter(|l| {
            let limit = l.contains("let i1 = ") && l.contains("SAT_IND_0_AUG_ROW");
            if limit && dropped == 0 {
                dropped += 1;
                return false;
            }
            true
        })
        .collect::<Vec<_>>()
        .join("\n");
    assert_eq!(dropped, 1, "{tag}: main-loop flux limit not found");
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate({FS:?});
    {setup}
    for _ in 0..10 {{ let _ = process_sample(0.0, &mut s); }}
    let n = {FS:?} as usize;
    for k in 0..n {{
        let w = (2.0 * std::f64::consts::PI * {f:?} * k as f64 / {FS:?}).sin();
        let x = if w >= 0.0 {{ {amp:?} }} else {{ -{amp:?} }};
        let y = process_sample(x, &mut s);
        println!(\"{{:e}}\", y[0]);
    }}
    println!(\"substep {{}} hold {{}}\", s.diag_substep_count, s.diag_nr_hold_count);
}}"
    );
    let out = support::compile_and_run(&code, &main, tag).stdout;
    let mut lines: Vec<&str> = out.lines().collect();
    let last: Vec<&str> = lines.pop().unwrap().split_whitespace().collect();
    let y = lines.iter().map(|l| l.parse().unwrap()).collect();
    (y, last[1].parse().unwrap(), last[3].parse().unwrap())
}

fn assert_twins(knob: (Vec<f64>, u64, u64), native: (Vec<f64>, u64, u64), what: &str) {
    let (yk, sk, hk) = knob;
    let (yn, sn, hn) = native;
    assert!(
        sn > 0,
        "{what}: the native build never sub-steps; the witness is void"
    );
    assert_eq!(
        (hk, hn),
        (0, 0),
        "{what}: held samples (knob {hk}, native {hn})"
    );
    assert_eq!(
        sk, sn,
        "{what}: sub-step counts differ (knob {sk}, native {sn})"
    );
    let peak = yn.iter().fold(0.0f64, |a, &b| a.max(b.abs()));
    let worst = yk
        .iter()
        .zip(&yn)
        .fold(0.0f64, |a, (p, q)| a.max((p - q).abs()));
    assert!(
        worst <= 1e-9 * peak,
        "{what}: knob build differs from native by {worst:e} on a {peak:e} output"
    );
}

#[test]
fn substep_solves_the_pot_setting_not_the_nominal_circuit() {
    let knob = run(
        "pot\nR_1 in out 99\nL_1 out 0 1 ISAT=10m LAIR=3e-4\n.pot R_1 10 1k 99 \"Drive\"\n",
        "s.set_pot_0(50.0);",
        5.0,
        30.0,
        "substep_pot_knob",
    );
    let native = run(
        "native\nR_1 in out 50\nL_1 out 0 1 ISAT=10m LAIR=3e-4\n",
        "",
        5.0,
        30.0,
        "substep_pot_native",
    );
    assert_twins(knob, native, "pot at 50 ohm");
}

#[test]
fn substep_solves_the_switch_position_not_the_nominal_circuit() {
    let knob = run(
        "sw\nR_1 in out 30\nL_1 out 0 1 ISAT=10m LAIR=3e-4\nC_1 out 0 1u\n\
         .switch C_1 1u 100n \"Cap\"\n",
        "s.set_switch_0(1);",
        20.0,
        100.0,
        "substep_switch_knob",
    );
    let native = run(
        "native\nR_1 in out 30\nL_1 out 0 1 ISAT=10m LAIR=3e-4\nC_1 out 0 100n\n",
        "",
        20.0,
        100.0,
        "substep_switch_native",
    );
    assert_twins(knob, native, "switch at 100 nF");
}
