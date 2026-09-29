//! A railed op-amp output sags under load: it sits on the load line
//! `limit − R_SAG·I_load`.
//!
//! `R_SAG` is the saturated sag, fitted from datasheet V_OM-versus-load lines;
//! it is not the open-loop `ROUT`. The active-set modes pin a railed output on
//! that line, continuously at engagement and release, and the DC operating
//! point does the same, so a railed-at-rest output starts on its line. Hard
//! clips at the zero-load limit, without sag.
//!
//! Every deck here: VCC = ±15 V, default drop 1.0 V (zero-load limit 14 V),
//! R_SAG 200 Ω, ROUT 75 Ω.

mod support;

use melange_solver::codegen::OpampRailMode;

fn comparator(load: &str) -> String {
    format!(
        "comparator\nRleak in 0 1Meg\nU1 in 0 out OX\nRl out 0 {load}\n\
         .model OX OA(AOL=100000 VCC=15 VEE=-15)\n"
    )
}

fn code(deck: &str, mode: OpampRailMode) -> String {
    let mut config = support::config_for_spice(deck, 48000.0);
    config.dc_block = false;
    config.opamp_rail_mode = mode;
    support::generate_circuit_code_nodal(deck, &config).0
}

/// Peak output under a sine of `amp` volts at `freq`, plus every sample.
fn render(code: &str, amp: f64, freq: f64, n: usize, tag: &str) -> Vec<f64> {
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for i in 0..{n}usize {{
        let u = {amp:?} * (2.0 * std::f64::consts::PI * {freq:?} * i as f64 / 48000.0).sin();
        let y = process_sample(u, &mut s)[0];
        println!(\"{{:.17e}}\", y);
    }}
    assert_eq!(s.diag_nr_unconverged_commit_count, 0);
}}"
    );
    support::compile_and_run(code, &main, tag).parse_samples()
}

fn peak(y: &[f64]) -> f64 {
    y.iter().copied().fold(f64::MIN, f64::max)
}

#[test]
fn a_railed_output_sits_on_its_load_line() {
    for (load, r_l) in [("10k", 10e3), ("1k", 1e3)] {
        let deck = comparator(load);
        // 100x the input that reaches the limit.
        let amp = 100.0 * 14.0 / 1e5;
        let active = peak(&render(
            &code(&deck, OpampRailMode::ActiveSet),
            amp,
            1000.0,
            480,
            &format!("sag_active_{load}"),
        ));
        let hand = 14.0 / (1.0 + 200.0 / r_l);
        assert!(
            (active - hand).abs() < 1e-6,
            "{load}: active-set saturates at {active:.7} V, the load line gives {hand:.7} V"
        );
        let hard = peak(&render(
            &code(&deck, OpampRailMode::Hard),
            amp,
            1000.0,
            480,
            &format!("sag_hard_{load}"),
        ));
        assert!(
            (hard - 14.0).abs() < 1e-9,
            "{load}: hard clips at the zero-load limit, got {hard:.7} V"
        );
    }
}

/// On a slow sine clipped by an inverting ×20 stage, the output step at every
/// pin edge is no larger than the steps on the unclipped side next to it.
/// The mutant detects the rail on the terminal voltage (the old rule), so the
/// output engages at the limit and drops onto the load line in one sample.
#[test]
fn pin_engagement_and_release_are_continuous() {
    for (load, r_l) in [("10k", 10e3), ("1k", 1e3)] {
        let deck = format!(
            "inverting x20\nR1 in inv 10k\nRf inv out 200k\nU1 0 inv out OX\nRl out 0 {load}\n\
             .model OX OA(AOL=100000 VCC=15 VEE=-15)\n"
        );
        let shipped = code(&deck, OpampRailMode::ActiveSet);
        let worst = |code: &str, tag: &str| -> f64 {
            let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    for i in 0..2400usize {
        let u = (2.0 * std::f64::consts::PI * 100.0 * i as f64 / 48000.0).sin();
        let y = process_sample(u, &mut s)[0];
        println!(\"{:.17e} {:.17e}\", y, s.v_prev[NODE_INV]);
    }
    assert_eq!(s.diag_nr_unconverged_commit_count, 0);
}";
            let out = support::compile_and_run(code, main, tag);
            let rows: Vec<(f64, f64)> = out
                .stdout
                .lines()
                .filter_map(|l| {
                    let mut it = l.split_whitespace().map(|t| t.parse::<f64>().ok());
                    Some((it.next()??, it.next()??))
                })
                .collect();
            let y: Vec<f64> = rows.iter().map(|r| r.0).collect();
            // The load current: into R_l and back through Rf to the inverting input.
            let on_line = |&(v, inv): &(f64, f64)| {
                let i = v / r_l + (v - inv) / 200e3;
                ((v - (14.0 - 200.0 * i)).abs() < 1e-6) || ((v - (-14.0 - 200.0 * i)).abs() < 1e-6)
            };
            let pinned: Vec<bool> = rows.iter().map(on_line).collect();
            let dv: Vec<f64> = y.windows(2).map(|w| (w[1] - w[0]).abs()).collect();
            let mut worst = 0.0_f64;
            for e in 0..pinned.len() - 1 {
                if pinned[e] == pinned[e + 1] || e < 4 || e + 4 >= dv.len() {
                    continue;
                }
                let side: Vec<usize> = if pinned[e] { (e + 1..e + 4).collect() } else { (e - 3..e).collect() };
                let neighbour = side.iter().map(|&j| dv[j]).fold(0.0, f64::max);
                worst = worst.max(dv[e] / neighbour);
            }
            worst
        };
        let r = worst(&shipped, &format!("sag_edge_{load}"));
        eprintln!("{load}: worst pin-edge step / unclipped-neighbour step = {r:.3}");
        assert!(r < 1.0, "{load}: a pin edge steps {r:.2}x its neighbours");

        let w_line = shipped
            .lines()
            .find(|l| l.trim_start().starts_with("let w_0 = "))
            .expect("load-line detection emitted")
            .to_string();
        let terminal = format!(
            "{}let w_0 = v[NODE_OUT];",
            &w_line[..w_line.len() - w_line.trim_start().len()]
        );
        let mutant = shipped.replace(&w_line, &terminal);
        assert_ne!(mutant, shipped);
        let rm = worst(&mutant, &format!("sag_edge_mutant_{load}"));
        eprintln!("{load}: terminal-detection mutant {rm:.3}");
        assert!(rm > 1.1, "{load}: the mutant's jump ({rm:.2}x) is not caught");
    }
}

/// A comparator railed at rest: the DC operating point is where the rail
/// mode saturates it, and the first transient samples do not move from it.
#[test]
fn a_railed_at_rest_output_starts_on_its_load_line() {
    let deck = "comparator railed at rest\nVref ref 0 DC 0.1\nRin in 0 10k\nU1 ref in out OX\n\
                Rl out 0 1k\n.model OX OA(AOL=100000 VCC=15 VEE=-15)\n";
    // Each rail mode's own saturation: the load line for the active-set
    // modes, the zero-load limit at the terminal for hard.
    for (mode, line) in [
        (OpampRailMode::ActiveSet, 14.0 / 1.2),
        (OpampRailMode::ActiveSetBe, 14.0 / 1.2),
        (OpampRailMode::Hard, 14.0),
    ] {
        let y = render(&code(deck, mode), 0.0, 1000.0, 48, &format!("sag_rest_{mode:?}"));
        for (n, v) in y.iter().enumerate() {
            assert!(
                (v - line).abs() < 1e-6,
                "{mode:?}: sample {n} at {v:.7} V, the load line is {line:.7} V"
            );
        }
    }
}

const RAILED_AT_REST: &str = "comparator railed at rest\nVref ref 0 DC 0.1\nRin in 0 10k\n\
                              U1 ref in out OX\nRl out 0 1k\n\
                              .model OX OA(AOL=100000 VCC=15 VEE=-15)\n";

/// The DC operating point's pin outcome is part of the build's record: a
/// pinned output is named in the provenance, and a pin that falls back to the
/// terminal clamp is reported as a fallback, never silently.
#[test]
fn the_rail_pin_outcome_is_surfaced_and_its_fallback_is_counted() {
    use melange_solver::codegen::ir::CircuitIR;
    use melange_solver::dc_op::{self, DcOpConfig, RailPin};
    use melange_solver::mna::MnaSystem;
    use melange_solver::parser::Netlist;

    let code = code(RAILED_AT_REST, OpampRailMode::ActiveSet);
    assert!(
        code.contains("\"dc_op_rail_pin\":\"pinned 1\""),
        "the provenance names the pinned output"
    );

    let netlist = Netlist::parse(RAILED_AT_REST).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
    let pinned = dc_op::solve_dc_operating_point(&mna, &slots, &DcOpConfig::default());
    assert_eq!(pinned.rail_pin, RailPin::Pinned(1));
    let out = pinned.v_node[mna.node_map["out"] - 1];
    assert!((out - 14.0 / 1.2).abs() < 1e-6, "pinned DC OP {out} V");

    // No active-set rounds allowed: the fallback path keeps the operating
    // point without the pin (on this linear deck, the unclamped linear
    // model's), and must say so.
    let config = DcOpConfig {
        max_rail_pin_rounds: 0,
        ..DcOpConfig::default()
    };
    let fell_back = dc_op::solve_dc_operating_point(&mna, &slots, &config);
    assert!(
        matches!(&fell_back.rail_pin, RailPin::FellBack(why) if why.contains("did not settle")),
        "{:?}",
        fell_back.rail_pin
    );
    assert!(fell_back.rail_pin.label().starts_with("FELL BACK"));
    let unpinned = fell_back.v_node[mna.node_map["out"] - 1];
    assert!((unpinned - 14.0 / 1.2).abs() > 1.0, "fallback DC OP {unpinned} V");
}

/// The runtime DC OP recompute (DK route, hard rail mode here) places a
/// railed output where the transient keeps it — the zero-load limit at the
/// terminal — and the first sample after a pot move does not kick. Without
/// the pin the recompute solved the linear model, an unclamped `AOL*vd`
/// operating point (here 8989 V on a 15 V supply).
#[test]
fn the_runtime_recompute_keeps_a_railed_output_on_its_rail() {
    let deck = "comparator railed at rest, pot load\nVref ref 0 DC 0.1\nRin in 0 10k\n\
                U1 ref in out OX\nRl out 0 1k\nRpot out 0 10k\n.pot Rpot 1k 100k \"Load\"\n\
                .model OX OA(AOL=100000 VCC=15 VEE=-15)\n";
    let mut config = support::config_for_spice(deck, 48000.0);
    config.dc_block = false;
    config.emit_dc_op_recompute = true;
    let code = support::generate_circuit_code(deck, &config).0;
    assert!(code.contains("pub const OPAMP_RAIL_MODE: &str = \"hard\""));
    assert!(code.contains("Railed op-amp outputs: terminal pin"));
    let main = "fn main() {
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let before = s.diag_nr_max_iter_count;
    s.set_pot_0(2000.0);
    s.recompute_dc_op();
    assert_eq!(s.diag_nr_max_iter_count, before, \"the recompute failed\");
    println!(\"dc={:.17e}\", s.dc_operating_point[NODE_OUT]);
    for _ in 0..8 {
        let y = process_sample(0.0, &mut s)[0];
        println!(\"{:.17e}\", y);
    }
}";
    let out = support::compile_and_run(&code, main, "sag_runtime_recompute");
    let dc = out.parse_kv("dc").unwrap();
    assert!((dc - 14.0).abs() < 1e-9, "recomputed DC OP {dc} V, the hard limit is 14 V");
    for (n, y) in out.parse_samples().iter().enumerate() {
        assert!((y - 14.0).abs() < 1e-9, "sample {n} at {y} V after the recompute");
    }
    // Mutant: write back the first, unpinned solve (the old recompute).
    let mutant = code.replace("        if next_pins == rail_pins {", "        if true {");
    assert_ne!(mutant, code);
    let m = support::compile_and_run(&mutant, main, "sag_runtime_recompute_mutant");
    let dc_m = m.parse_kv("dc").unwrap();
    eprintln!("unpinned recompute: DC OP {dc_m} V");
    assert!(dc_m > 20.0, "the mutant's unclamped DC OP ({dc_m} V) is not caught");
}
