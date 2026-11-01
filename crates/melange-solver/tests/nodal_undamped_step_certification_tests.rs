//! The nodal Newton's step test reads the UNDAMPED correction every iteration.
//!
//! Certification of a nodal full-LU iterate is three tests whatever the
//! damping: the node-step test on `v_new − v` (Newton's own estimate of the
//! distance to the root), the device-chord consistency check and the
//! row-residual gate. The step test used to read the damped step
//! `alpha·(v_new − v)` and to be skipped on any iteration the line search or
//! the node damping had shrunk `alpha` below 1; the gate bounds row residuals,
//! not distance, so a point inside it could be certified volts from the root
//! (a choke-loaded MOSFET stage at 3 V drive: drain 16 V off, `alpha = 0.31`;
//! a transformer-coupled triode at 10 V: 12.5 V off on 22 samples with
//! `alpha = 0.99`). This file pins the emitted test and keeps the MOSFET deck
//! solving without a held sample at the drive that used to false-certify.

mod support;

/// A choke-loaded N-channel MOSFET stage (the melange-circuits example
/// `mosfet-choke-load`, prose stripped): saturating choke load, so it routes
/// nodal full-LU; at 3 V peak drive the code before this test certified
/// sample 653 with the drain node 16.4 V from Newton's target.
const MOSFET_CHOKE_LOAD: &str = "MOSFET Choke-Loaded Gain Stage
.model 2N7000 NMOS(KP=0.1 VTO=2.1 LAMBDA=0.01 GAMMA=0.5 PHI=0.6)
VCC vcc 0 DC 24
Cin  in     gate_i 1u
Rb1  vcc    gate_i 1MEG
Rb2  gate_i 0      170k
Rg   gate_i gate   1k
Lp   vcc    drain  5 ISAT=20m
Rdcr drain  drain_d 120
Cw   vcc    drain  220p
Rw   vcc    drain  470k
M1   drain_d gate  src  0  2N7000
Rs   src    0      220
Cs   src    0      100u
Cout drain  out    100n
Rl   out    0      100k
.END
";

fn build(spice: &str, output: &str) -> melange_solver::build::Built {
    let mut config = support::config_in_out_or_node1(spice, 48000.0);
    let netlist = support::parse_expanded(spice);
    let mna = melange_solver::mna::MnaSystem::from_netlist(&netlist).expect("MNA");
    let names = mna.node_names_in_index_order();
    let idx = names
        .iter()
        .position(|n| *n == output)
        .unwrap_or_else(|| panic!("no node {output}"))
        - 1;
    config.output_nodes = vec![idx];
    support::try_build_shipped_with(spice, &config, "auto", |_| {})
        .unwrap_or_else(|e| panic!("build refused: {e}"))
}

#[test]
fn the_step_test_reads_the_undamped_correction_and_is_never_skipped() {
    let code = build(MOSFET_CHOKE_LOAD, "out").generated.code;
    assert!(
        code.contains("full-lu") || code.contains("full_lu"),
        "the witness deck routes nodal full-LU"
    );
    // The damped-step exemption is gone: no `limited` flag, no skip.
    assert!(!code.contains("let limited = alpha < 1.0;"), "the `limited` flag is gone");
    assert!(!code.contains("if !limited {"), "the step test is never skipped");
    // Every step-test line reads `v_new − v` with the threshold at `v_new`,
    // and none of them scales the correction by `alpha`.
    let step_lines: Vec<&str> = code
        .lines()
        .filter(|l| l.contains("let threshold = 1e-3 *") && l.contains("max_step_exceeded = true"))
        .collect();
    assert!(!step_lines.is_empty(), "no step-test lines emitted");
    for l in &step_lines {
        assert!(
            l.contains("let step = v_new[") && l.contains(".max(v_new["),
            "step test on the undamped correction with the threshold at v_new: {l}"
        );
        assert!(!l.contains("alpha"), "damped step in a step test: {l}");
    }
}

#[test]
fn the_choke_loaded_mosfet_stage_solves_every_sample_at_the_drive_that_used_to_false_certify() {
    let code = build(MOSFET_CHOKE_LOAD, "out").generated.code;
    // 1 kHz at 3 V peak, 0.5 s: the drive at which the code before this test
    // certified a drain voltage 16 V from Newton's target on one sample.
    let main = "
fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for i in 0..24000usize {
        let x = 3.0 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin();
        let _ = process_sample(x, &mut s);
    }
    println!(\"{} {} {}\", s.diag_nr_hold_count, s.diag_unsolved_sample_count, s.diag_nr_max_iter_count);
}
";
    let out = support::compile_and_run(&code, main, "undamped_step_mosfet_choke");
    let mut it = out.stdout.split_whitespace().map(|t| t.parse::<u64>().unwrap());
    let (held, unsolved, _ceiling) = (it.next().unwrap(), it.next().unwrap(), it.next().unwrap());
    assert_eq!(held, 0, "held samples:\n{}", out.stderr);
    assert_eq!(unsolved, 0, "unsolved samples:\n{}", out.stderr);
}
