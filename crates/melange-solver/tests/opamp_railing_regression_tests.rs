//! A railing op-amp with a nonlinear device after its output coupling cap.
//!
//! The regression corpus had no deck whose op-amp actually reaches its rails,
//! so the active-set rail handling was never exercised with M > 0. This is the
//! textbook single-supply overdrive: non-inverting gain ~107, output cap into a
//! 1N914 pair to ground, a tone RC and a volume load. The op-amp rails from
//! 0.05 V drive up.
//!
//! The pinned resolve used to be ONE linear solve with the device currents of
//! the unpinned solve frozen. The pin moves the op-amp output by volts, the
//! output cap passes that step straight to the diodes, and the frozen currents
//! are then wrong: the clipper node was driven to -2 V, a reverse diode was
//! re-evaluated at 3.6e9 A, and the next sample diverged (1000 unsolved
//! samples per second, peaks of 21 kV). It is now Newton on the pinned system.
//!
//! Reference: ngspice, with the op-amp modelled as gm = 0.2 S into
//! 1 MΩ ∥ 10.61 nF (AOL 2e5, 15 Hz pole, GBW 3 MHz), an anti-windup wall at
//! 0 / 9 V and an ideal output buffer. Steady-state output peak (0.5-1.0 s):
//! 0.466 / 0.480 / 0.487 / 0.490 / 0.491 V at 0.05 / 0.1 / 0.2 / 0.5 / 1.0 V;
//! |v(n1)| 4.77 / 5.14 / 5.29 / 5.38 / 5.41 V. The two op-amp models differ
//! only in details worth a few percent on a diode-clipped square wave (ROUT,
//! the exact wall), so the gate is ±5 %. Measured: within 2.4 % at 1×, 1.0 %
//! at 4×.

mod support;

use melange_solver::codegen::routing::{self, SolverRoute};
use melange_solver::codegen::OpampRailMode;
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const RAILING_OVERDRIVE: &str = "\
single-supply op-amp overdrive, diode clipper after the output cap
Vcc vcc 0 DC 9
R_b1 vcc vbias 100k
R_b2 vbias 0 100k
C_b vbias 0 10u
C_in in np 100n
R_in np vbias 1Meg
U1 np nm oa TL072
R_f oa nm 500k
R_g nm ng 4.7k
C_g ng 0 10u
C_c oa n1 1u
R_1 n1 n2 1k
D_1 n2 0 D1N914
D_2 0 n2 D1N914
R_t n2 n3 10k
C_t n3 0 22n
C_o n3 out 1u
R_v out 0 100k
.model TL072 OA(AOL=200000 GBW=3e6 VCC=9 VEE=0)
.model D1N914 D(IS=2.52n N=1.752)
";

/// (drive V, ngspice output peak V, ngspice |n1| max V)
const REFERENCE: [(f64, f64, f64); 5] = [
    (0.05, 0.4660, 4.773),
    (0.1, 0.4803, 5.141),
    (0.2, 0.4865, 5.294),
    (0.5, 0.4899, 5.383),
    (1.0, 0.4910, 5.410),
];

fn node(name: &str) -> usize {
    let netlist = Netlist::parse(RAILING_OVERDRIVE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.node_map[name] - 1
}

/// Render 1 s of a 1 kHz sine at each drive and report the steady-state
/// (0.5-1.0 s) output peak, |n1| max, op-amp output range and the unsolved /
/// reset counters, one line per drive.
fn render(oversampling: usize) -> Vec<(f64, f64, f64, f64, f64, u64)> {
    let mut config = support::config_for_spice(RAILING_OVERDRIVE, 48000.0);
    config.oversampling_factor = oversampling;
    config.opamp_rail_mode = OpampRailMode::Auto;
    let (code, _n, _m) = support::generate_circuit_code_nodal(RAILING_OVERDRIVE, &config);
    assert!(
        code.contains("pub const OPAMP_RAIL_MODE: &str = \"active-set"),
        "test premise: auto must pick an active-set mode"
    );
    let (out, n1, oa) = (node("out"), node("n1"), node("oa"));
    let drives: Vec<String> = REFERENCE.iter().map(|r| format!("{:?}", r.0)).collect();
    let main = format!(
        "fn main() {{
    for amp in [{drives}] {{
        let mut s = CircuitState::default();
        s.set_sample_rate(48000.0);
        let (mut pk, mut n1, mut oa_lo, mut oa_hi) = (0.0f64, 0.0f64, f64::MAX, f64::MIN);
        for i in 0..48000 {{
            let x = amp * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin();
            let _ = process_sample(x, &mut s);
            if i >= 24000 {{
                pk = pk.max(s.v_prev[{out}].abs());
                n1 = n1.max(s.v_prev[{n1}].abs());
                oa_lo = oa_lo.min(s.v_prev[{oa}]);
                oa_hi = oa_hi.max(s.v_prev[{oa}]);
            }}
        }}
        let bad = s.diag_nr_unconverged_commit_count + s.diag_magnitude_reset_count + s.diag_nan_reset_count;
        println!(\"{{amp}} {{pk}} {{n1}} {{oa_lo}} {{oa_hi}} {{bad}}\");
    }}
}}",
        drives = drives.join(", ")
    );
    let run = support::compile_and_run(&code, &main, &format!("railing_{oversampling}x"));
    run.stdout
        .lines()
        .map(|l| {
            let f: Vec<f64> = l.split_whitespace().map(|t| t.parse().unwrap()).collect();
            (f[0], f[1], f[2], f[3], f[4], f[5] as u64)
        })
        .collect()
}

fn check_against_reference(oversampling: usize) {
    let rows = render(oversampling);
    assert_eq!(rows.len(), REFERENCE.len());
    for (row, &(amp, ref_pk, _ref_n1)) in rows.iter().zip(REFERENCE.iter()) {
        let (_, pk, n1, oa_lo, oa_hi, bad) = *row;
        let err = (pk - ref_pk) / ref_pk;
        assert_eq!(
            bad, 0,
            "{oversampling}x {amp} V: {bad} unsolved/reset samples"
        );
        assert!(
            err.abs() <= 0.05,
            "{oversampling}x {amp} V: out peak {pk:.4} V vs ngspice {ref_pk:.4} V ({:+.1} %)",
            100.0 * err
        );
        assert!(
            n1 <= 5.6,
            "{oversampling}x {amp} V: |n1| {n1:.2} V (ngspice ≤ 5.41 V)"
        );
        assert!(
            oa_lo >= -1e-6 && oa_hi <= 9.0 + 1e-6,
            "{oversampling}x {amp} V: op-amp output [{oa_lo}, {oa_hi}] outside its 0..9 V rails"
        );
    }
}

#[test]
fn railing_overdrive_routes_nodal() {
    let netlist = Netlist::parse(RAILING_OVERDRIVE).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let input = mna.node_map["in"] - 1;
    mna.g[input][input] += 1.0;
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();
    let d = routing::auto_route(&kernel, &mna, false, OpampRailMode::Auto);
    assert!(
        d.opamp_active_set && d.route == SolverRoute::Nodal,
        "{}",
        d.reason
    );
}

#[test]
fn railing_overdrive_matches_ngspice_at_1x() {
    check_against_reference(1);
}

#[test]
fn railing_overdrive_matches_ngspice_at_4x() {
    check_against_reference(4);
}
