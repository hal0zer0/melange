//! A JFET's gate is a pn junction to the channel, source and drain side.
//!
//! melange had neither junction: the gate drew no current at any bias, so a
//! gate driven into forward bias was not clamped (driven to +2 V through
//! 100 kΩ it sat at 2.000 V where ngspice clamps it at 0.546 V). The
//! junctions are SPICE's level-1 JFET gate diodes, `IS·(exp(V/(N·Vt)) − 1)`
//! at Vgs and Vgd, with the ngspice default IS = 1e-14 and N = 1 (ngspice's
//! level-1 JFET has no N; its junction is fixed at 1).
//!
//! References: ngspice .op / .tran on the same decks with GMIN off
//! (`gmin=1e-25`, tight tolerances). ngspice adds 1e-12 S across each gate
//! junction by default; melange's junctions carry no conditioning term, so
//! the fixed point is the device's.

mod support;

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::dc_op::{solve_dc_operating_point, DcOpConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const CARD: &str = ".model JX NJF(VTO=-2 BETA=1e-3)\n";

/// (node voltages by name, JFET currents [drain, gate]) at the DC OP.
fn dc(spice: &str) -> (std::collections::HashMap<String, f64>, [f64; 2]) {
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
    let dc = solve_dc_operating_point(&mna, &slots, &DcOpConfig::default());
    assert!(dc.converged, "{:?}", dc.method);
    let v = mna
        .node_map
        .iter()
        .filter(|(_, &i)| i > 0)
        .map(|(n, &i)| (n.clone(), dc.v_node[i - 1]))
        .collect();
    (v, [dc.i_nl[0], dc.i_nl[1]])
}

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol,
        "{what}: melange {got:.12e}, ngspice {want:.12e}"
    );
}

const CLAMP: &str = "gate driven to +2 V through 100k\nVCC vcc 0 DC 12\nVG gd 0 DC 2\n\
                     RG gd g 100k\nRD vcc d 2.2k\nJ1 d g 0 JX\n";

#[test]
fn a_forward_driven_gate_clamps() {
    let (v, _) = dc(&format!("{CLAMP}{CARD}"));
    close("v(g)", v["g"], 5.456927302190e-01, 1e-6);
    close("v(d)", v["d"], 1.278043394671e+00, 1e-6);
}

/// The junctions are what clamps: with them disabled (IS = 0, the model
/// before them) the gate sits at the drive.
#[test]
fn without_the_junctions_the_gate_does_not_clamp() {
    let card = CARD.replace("BETA=1e-3", "BETA=1e-3 IS=0");
    let (v, _) = dc(&format!("{CLAMP}{card}"));
    assert!((v["g"] - 2.0).abs() < 1e-6, "v(g) {}", v["g"]);
}

#[test]
fn forward_gate_current_near_0_6_v() {
    let deck = format!(
        "gate near +0.6 V\nVCC vcc 0 DC 12\nVG gd 0 DC 0.6\nRG gd g 1k\nRD vcc d 2.2k\n\
         J1 d g 0 JX\n{CARD}"
    );
    let (v, _) = dc(&deck);
    close("v(g)", v["g"], 5.669362077403e-01, 1e-6);
    close("v(d)", v["d"], 1.260240839526e+00, 1e-6);
}

/// Reverse biased, the gate draws each junction's saturation current:
/// ngspice's i(VG) = 2e-14 A, read here as the device's gate current (a
/// node voltage cannot resolve 1e-14 A against the DC solve's node Gmin).
#[test]
fn reverse_gate_leakage() {
    let deck =
        format!("gate at -5 V\nVCC vcc 0 DC 12\nVG g 0 DC -5\nRD vcc d 2.2k\nJ1 d g 0 JX\n{CARD}");
    let (_, [_, i_gate]) = dc(&deck);
    let i_vg = 1.999999809404e-14;
    assert!(
        (-i_gate - i_vg).abs() <= 1e-6 * i_vg,
        "gate current {i_gate:e}, ngspice i(VG) {i_vg:e}"
    );
}

/// Drain pulled below the gate: the gate-drain junction conducts.
#[test]
fn a_forward_gate_drain_junction_conducts() {
    let deck = format!(
        "drain below gate\nVDD dd 0 DC -3\nRD dd d 1k\nVG gd 0 DC 0\nRG gd g 10k\n\
         J1 d g 0 JX\n{CARD}"
    );
    let (v, _) = dc(&deck);
    close("v(g)", v["g"], -3.70896401147e-02, 1e-6);
    close("v(d)", v["d"], -5.47441525993e-01, 1e-6);
}

/// A JFET variable resistor at Vgs ~ 0: signal through 1k into the drain,
/// source grounded, gate held at 0 V through 1M. At 5 V the drain swings to
/// -1 V, the gate-drain junction conducts (Vgd peaks at 0.46 V where it would
/// reach 1 V without it) and pulls the gate down to -0.125 V on average;
/// at 1 V it barely does. Per drive: H1 [V], H2/H1, H3/H1 of v(d) and the
/// mean of v(g) over 0.1-0.2 s, from ngspice .tran (0.2 us step, reltol 1e-7).
/// The 1 nF drain cap is in both decks: without a capacitor melange
/// auto-inserts 10 pF parasitics across the junctions (a pole near 16 kHz on
/// the 1M gate), which the ngspice twin lacks; with it the two agree to
/// 2e-6 on H1 at 48 kHz and at 384 kHz. Gates as C1's.
#[test]
fn a_jfet_resistor_near_vgs_0_follows_ngspice() {
    const FS: f64 = 48000.0;
    let spice = format!("jfet vvr\nR1 in d 1k\nJ1 d g 0 JX\nRG g 0 1Meg\nCd d 0 1n\n{CARD}");
    let reference: [(f64, f64, f64, f64, f64); 2] = [
        (
            1.0,
            2.003239892e-01,
            2.009974e-02,
            8.094049e-04,
            -2.563006087e-06,
        ),
        (
            5.0,
            1.140549138e+00,
            7.655062e-02,
            4.866959e-02,
            -1.250272224e-01,
        ),
    ];
    let mut config = support::config_for_spice(&spice, FS);
    config.output_nodes = vec![support::node_index(&spice, "d")];
    config.dc_block = false;
    let code = support::generate_circuit_code(&spice, &config).0;
    let g = support::node_index(&spice, "g");
    let bad = support::unsolved_expr(&code, "s");
    let main = format!(
        "fn main() {{
    for amp in [1.0f64, 5.0] {{
        let mut s = CircuitState::default();
        s.set_sample_rate({FS:?});
        let n = (0.2 * {FS:?}) as usize;
        let (mut ss, mut gg) = (Vec::new(), 0.0f64);
        for k in 1..=n {{
            let x = amp * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / {FS:?}).sin();
            let _ = process_sample(x, &mut s);
            if k > n / 2 {{ ss.push(s.v_prev[OUTPUT_NODES[0]]); gg += s.v_prev[{g}]; }}
        }}
        let mut h = [0.0f64; 4];
        for hh in 1..4 {{
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (j, &v) in ss.iter().enumerate() {{
                let w = 2.0 * std::f64::consts::PI * hh as f64 * 1000.0 * j as f64 / {FS:?};
                re += v * w.cos();
                im += v * w.sin();
            }}
            h[hh] = 2.0 * (re * re + im * im).sqrt() / ss.len() as f64;
        }}
        println!(\"{{}} {{}} {{}} {{}} {{}}\", h[1], h[2] / h[1], h[3] / h[1], gg / ss.len() as f64, {bad});
    }}
}}"
    );
    let out = support::compile_and_run(&code, &main, "jfet_vvr").stdout;
    for (line, &(amp, h1, r2, r3, vg)) in out.lines().zip(&reference) {
        let v: Vec<f64> = line
            .split_whitespace()
            .map(|t| t.parse().unwrap())
            .collect();
        eprintln!(
            "{amp} V: H1 {:.6e} ({:+.2e} rel), H2/H1 {:.4e} vs {r2:.4e}, H3/H1 {:.4e} vs {r3:.4e}, \
             mean v(g) {:.6e} vs {vg:.6e}",
            v[0],
            (v[0] - h1) / h1,
            v[1],
            v[2],
            v[3]
        );
        assert_eq!(v[4], 0.0, "{amp} V: unsolved samples");
        assert!(
            ((v[0] - h1) / h1).abs() <= 1e-5,
            "{amp} V: H1 {} vs {h1}",
            v[0]
        );
        assert!((v[1] - r2).abs() <= 1e-4, "{amp} V: H2/H1 {} vs {r2}", v[1]);
        assert!((v[2] - r3).abs() <= 1e-4, "{amp} V: H3/H1 {} vs {r3}", v[2]);
        assert!(
            (v[3] - vg).abs() <= 1e-5,
            "{amp} V: mean v(g) {} vs {vg}",
            v[3]
        );
    }
}
