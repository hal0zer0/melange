//! MOSFET body effect (GAMMA/PHI): the threshold moves with Vsb.
//!
//! The DC operating point and every transient path (DK, nodal Schur, nodal
//! full-LU) evaluate the threshold at the live Newton iterate and carry
//! gmb = gm·dVT/dVsb in the Jacobian. Two past defects are pinned here:
//! - MOSFET source/bulk nodes were resolved only after the nodal DC operating
//!   point was solved, so it left body effect out (V(src) 1.101 V instead of
//!   ngspice's 0.903 V on the choke-loaded stage below);
//! - DK and nodal Schur took Vsb from the linear prediction, which leaves out
//!   the device's own current. In the follower below that current IS what
//!   sets the source, so the stage settled 1 V high with H1 8 % hot and about
//!   a ninth of the second harmonic.
//!
//! References: ngspice Level-1 with the same law (square law, CLM
//! `1 + LAMBDA·|Vds|`, `VT = VTO + GAMMA·(√(PHI + Vsb) − √PHI)`).

mod support;

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::NodalSubPathOverride;
use melange_solver::dc_op::{self, DcOpConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

/// Common-source stage with a choke load; bulk on ground, source biased up.
const CHOKE_STAGE: &str = "\
choke-loaded common-source stage
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
";

/// Source follower; bulk on ground, so Vsb is the output bias.
const FOLLOWER: &str = "\
mosfet source follower
.model BS170 NMOS(KP=0.05 VTO=2.1 LAMBDA=0.02 GAMMA=0.5 PHI=0.6)
VCC vcc 0 DC 24
Cin  in    gate_i 1u
Rg   gate_i gate  1k
Rb1  vcc   gate_i 1MEG
Rb2  gate_i 0     1MEG
M1   vcc   gate   src  0  BS170
Rs   src   0      4.7k
Cout src   out    10u
Rl   out   0      100k
";

fn dc_op(spice: &str) -> (dc_op::DcOpResult, std::collections::HashMap<String, usize>) {
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
    let config = DcOpConfig {
        input_node: mna.node_map["in"] - 1,
        input_resistance: 1.0,
        ..DcOpConfig::default()
    };
    (
        dc_op::solve_dc_operating_point(&mna, &slots, &config),
        mna.node_map.clone(),
    )
}

/// The operating point includes body effect and is solved by Newton, not by
/// a fixed-point iteration on the threshold: without gmb in the DC Jacobian
/// the KCL residual stalled at 4.2e-9 A (choke) and 3.9e-10 A (follower).
///
/// The follower's source is 1.1e-5 V below ngspice's 8.54005 V: the DC solve's
/// node Gmin (1e-12 S) pulls its 1 MΩ gate divider 1.2e-5 V low, and the
/// follower passes that through. That leak is not body effect; the 2e-5 V
/// tolerance covers it and nothing else.
#[test]
fn dc_operating_point_includes_body_effect_at_newton_precision() {
    for (spice, ngspice_src, tol) in [
        (CHOKE_STAGE, 0.902_803_2, 2e-6),
        (FOLLOWER, 8.540_054, 2e-5),
    ] {
        let (r, nodes) = dc_op(spice);
        assert!(r.converged, "DC OP did not converge");
        assert!(
            r.kcl_residual_max <= 1e-12,
            "KCL residual {:.3e} A: Newton is not converging quadratically on the threshold",
            r.kcl_residual_max
        );
        let src = r.v_node[nodes["src"] - 1];
        assert!(
            (src - ngspice_src).abs() <= tol,
            "V(src) {src:.7} V vs ngspice {ngspice_src} V"
        );
    }
}

/// ngspice twin of FOLLOWER at 1 V, 1 kHz through the 1 Ω source, 1 µs max
/// step, reltol 1e-6: H1 and H2/H1 of v(out) over 2-3 s.
const FOLLOWER_H1: f64 = 0.909_896_067;
const FOLLOWER_H2_OVER_H1: f64 = 1.068_575e-3;

/// H1 and H2/H1 of v(out) over 2-3 s at 48 kHz, 1 V at 1 kHz.
fn follower_harmonics(code: &str, tag: &str) -> (f64, f64) {
    let netlist = Netlist::parse(FOLLOWER).unwrap();
    let out = MnaSystem::from_netlist(&netlist).unwrap().node_map["out"] - 1;
    let main = format!(
        "fn main() {{
    let fs = 48000.0f64;
    let mut s = CircuitState::default();
    s.set_sample_rate(fs);
    let n = (3.0 * fs) as usize;
    let (mut r1, mut i1, mut r2, mut i2) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for k in 0..n {{
        let w = 2.0 * std::f64::consts::PI * 1000.0 * k as f64 / fs;
        let _ = process_sample(w.sin(), &mut s);
        if k >= n - fs as usize {{
            let y = s.v_prev[{out}];
            r1 += y * w.cos(); i1 += y * w.sin();
            r2 += y * (2.0 * w).cos(); i2 += y * (2.0 * w).sin();
        }}
    }}
    let h1 = 2.0 * r1.hypot(i1) / fs;
    let h2 = 2.0 * r2.hypot(i2) / fs;
    println!(\"{{h1}} {{}}\", h2 / h1);
}}"
    );
    let out = support::compile_and_run(code, &main, tag).stdout;
    let v: Vec<f64> = out.split_whitespace().map(|t| t.parse().unwrap()).collect();
    (v[0], v[1])
}

/// Every transient path against the twin at the knee tests' gates: H1 within
/// 1e-5 relative, H2/H1 within 1e-4 absolute. Before the fix DK was 8.2 % high
/// on H1 with H2/H1 = 1.2e-4.
#[test]
fn follower_matches_ngspice_on_every_path() {
    let config = support::config_for_spice(FOLLOWER, 48000.0);
    let mut schur = config.clone();
    schur.nodal_sub_path_override = NodalSubPathOverride::Schur;
    let mut full_lu = config.clone();
    full_lu.nodal_sub_path_override = NodalSubPathOverride::FullLu;
    let builds = [
        ("dk", support::generate_circuit_code(FOLLOWER, &config).0),
        (
            "schur",
            support::generate_circuit_code_nodal(FOLLOWER, &schur).0,
        ),
        (
            "full_lu",
            support::generate_circuit_code_nodal(FOLLOWER, &full_lu).0,
        ),
    ];
    for (name, code) in builds {
        let (h1, h2r) = follower_harmonics(&code, &format!("follower_{name}"));
        assert!(
            ((h1 - FOLLOWER_H1) / FOLLOWER_H1).abs() <= 1e-5,
            "{name}: H1 {h1:.9} vs ngspice {FOLLOWER_H1}"
        );
        assert!(
            (h2r - FOLLOWER_H2_OVER_H1).abs() <= 1e-4,
            "{name}: H2/H1 {h2r:.6e} vs ngspice {FOLLOWER_H2_OVER_H1:.6e}"
        );
    }
}
