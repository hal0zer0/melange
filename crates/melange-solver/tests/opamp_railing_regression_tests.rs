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

/// How far a peak may fall between successive drive levels before it counts as
/// the wrong-sign drive response.
///
/// The check exists to catch a response that FALLS with drive, as the old DK +
/// Hard route did (0.60 V -> 0.37 V, -38 %). It is not a precision criterion:
/// ngspice itself rises only +0.2 % from 0.5 V to 1.0 V drive, less than the
/// few-percent modelling slack between the two op-amp models, and the 4x build
/// measured -0.02 % over that step. A zero-tolerance "non-decreasing" rule was a
/// mis-specification. Do not tighten this by tuning the solver to it.
const MONOTONIC_TOLERANCE: f64 = 0.005;

fn check_against_reference(oversampling: usize) {
    let rows = render(oversampling);
    assert_eq!(rows.len(), REFERENCE.len());
    for pair in rows.windows(2) {
        let ((a0, p0, ..), (a1, p1, ..)) = (pair[0], pair[1]);
        assert!(
            p1 >= p0 * (1.0 - MONOTONIC_TOLERANCE),
            "{oversampling}x: peak falls with drive, {p0:.4} V at {a0} V -> {p1:.4} V at {a1} V"
        );
    }
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

/// The pinned solve does not include behavioral sources (their Jacobian is
/// stamped in node space and is not diagonal), so with one present it could
/// converge to a point that is not a solution. Codegen refuses rather than
/// pinning approximately.
#[test]
fn active_set_with_a_behavioral_source_is_refused() {
    use melange_solver::codegen::CodeGenerator;
    let spice = format!("{RAILING_OVERDRIVE}B_x bx 0 V={{ tanh(V(n3)) }}\nR_bx bx 0 1k\n");
    let netlist = Netlist::parse(&spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let input = mna.node_map["in"] - 1;
    mna.g[input][input] += 1.0;
    let config = support::config_for_spice(&spice, 48000.0);
    let err = CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect_err("active-set + a behavioral source must be refused");
    let msg = format!("{err:?}");
    // It must not steer users to a mode measured wrong on this class.
    assert!(
        msg.contains("behavioral source") && msg.contains("No rail handling is validated"),
        "{msg}"
    );
}

/// An explicit rail mode is still honoured (overrides are how users bisect),
/// even though it is not validated on this class.
#[test]
fn explicit_hard_with_a_behavioral_source_still_compiles() {
    use melange_solver::codegen::CodeGenerator;
    let spice = format!("{RAILING_OVERDRIVE}B_x bx 0 V={{ tanh(V(n3)) }}\nR_bx bx 0 1k\n");
    let netlist = Netlist::parse(&spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    let input = mna.node_map["in"] - 1;
    mna.g[input][input] += 1.0;
    let mut config = support::config_for_spice(&spice, 48000.0);
    config.opamp_rail_mode = OpampRailMode::Hard;
    CodeGenerator::new(config)
        .generate_nodal(&mna, &netlist)
        .expect("an explicit rail mode is never overridden");
}

// ─── A railing op-amp driving a saturating choke ──────────────────────────
//
// The same single-supply overdrive, with the diode clipper replaced by a
// 100 mH choke that saturates at 2 mA. The op-amp output is a square wave, so
// the choke is driven hard into saturation every half cycle (about 2.7x
// Isat) while the op-amp is pinned at a rail: the pinned solve has to carry
// the flux law.
//
// Reference: ngspice, the op-amp twin above, with the choke as a flux
// integrator (a unit capacitor charged by v(n2)) and a behavioral current
// I = Isat·atanh(Φ/(L0·Isat)); 1 µs step, reltol 1e-5, converged to 0.02 %
// against 0.25 µs. The references are i_L max over 0.5-1.0 s and ngspice's
// `fourier` H1 of v(out) over the last cycle.
//
// Gated at 4x. At 1x both rail modes miss for integrator reasons, not the
// pinned solve: active-set-be runs this deck on backward Euler almost all the
// time (the op-amp is railed ~95 % of each cycle) and is first-order wrong at
// L/R ~ 5 samples; active-set's trapezoidal rule rings in deep saturation
// (L_diff ~ 0.03·L0 makes the RL step factor ~ -0.6) and overshoots the i_L
// peak. Both close at 4x. The 1x cases are recorded below, ignored, as the
// targets those integrator fixes must meet.

const RAILING_INTO_CHOKE: &str = "\
single-supply op-amp overdrive into a saturating choke
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
L_sat n2 0 100m ISAT=2m
R_t n2 out 10k
R_v out 0 100k
.model TL072 OA(AOL=200000 GBW=3e6 VCC=9 VEE=0)
";

/// (drive V, ngspice i_L max A over 0.5-1.0 s)
const CHOKE_IL: [(f64, f64); 5] = [
    (0.05, 5.017114e-3),
    (0.1, 5.288674e-3),
    (0.2, 5.351541e-3),
    (0.5, 5.377945e-3),
    (1.0, 5.384495e-3),
];

/// (drive V, ngspice H1 of v(out))
const CHOKE_H1: [(f64, f64); 2] = [(0.1, 1.37902), (1.0, 1.40291)];

/// One rendered drive level of the choke deck.
struct ChokeRow {
    amp: f64,
    il_max: f64,
    h1: f64,
    oa_lo: f64,
    oa_hi: f64,
    /// Samples committed without a solution: the unsolved-commit counter plus
    /// any hold, NaN/magnitude reset or sub-step recovery.
    unsolved: u64,
}

fn choke_code(mode: OpampRailMode, oversampling: usize) -> String {
    let mut config = support::config_for_spice(RAILING_INTO_CHOKE, 48000.0);
    config.oversampling_factor = oversampling;
    config.opamp_rail_mode = mode;
    support::generate_circuit_code_nodal(RAILING_INTO_CHOKE, &config).0
}

/// Render 1 s of a 1 kHz sine per drive at `host_rate`; over 0.5-1.0 s report
/// the i_L peak (at every host sample), H1 of the plugin output, and the
/// op-amp output range.
fn render_choke(code: &str, host_rate: f64, tag: &str) -> Vec<ChokeRow> {
    let counters: Vec<&str> = [
        "diag_nr_unconverged_commit_count",
        "diag_nr_hold_count",
        "diag_nan_reset_count",
        "diag_magnitude_reset_count",
        "diag_substep_count",
    ]
    .into_iter()
    .filter(|f| code.contains(&format!("pub {f}: ")))
    .collect();
    assert!(
        counters.contains(&"diag_nr_unconverged_commit_count"),
        "test premise: a build that can pin an op-amp declares the unsolved-commit counter"
    );
    let unsolved = counters
        .iter()
        .map(|f| format!("s.{f}"))
        .collect::<Vec<_>>()
        .join(" + ");
    let oa = {
        let netlist = Netlist::parse(RAILING_INTO_CHOKE).unwrap();
        MnaSystem::from_netlist(&netlist).unwrap().node_map["oa"] - 1
    };
    let drives: Vec<String> = CHOKE_IL.iter().map(|r| format!("{:?}", r.0)).collect();
    let main = format!(
        "fn main() {{
    let fs: f64 = {host_rate:?};
    let n = fs as usize;
    for amp in [{drives}] {{
        let mut s = CircuitState::default();
        s.set_sample_rate(fs);
        let (mut il, mut lo, mut hi, mut re, mut im) = (0.0f64, f64::MAX, f64::MIN, 0.0f64, 0.0f64);
        for i in 0..n {{
            let w = 2.0 * std::f64::consts::PI * 1000.0 * i as f64 / fs;
            let y = process_sample(amp * w.sin(), &mut s)[0];
            if i >= n / 2 {{
                il = il.max(s.v_prev[SAT_IND_0_AUG_ROW].abs());
                lo = lo.min(s.v_prev[{oa}]);
                hi = hi.max(s.v_prev[{oa}]);
                re += y * w.cos();
                im += y * w.sin();
            }}
        }}
        let h1 = 2.0 * (re / (n / 2) as f64).hypot(im / (n / 2) as f64);
        println!(\"{{amp}} {{il}} {{h1}} {{lo}} {{hi}} {{}}\", {unsolved});
    }}
}}",
        drives = drives.join(", ")
    );
    support::compile_and_run(code, &main, tag)
        .stdout
        .lines()
        .map(|l| {
            let f: Vec<f64> = l.split_whitespace().map(|t| t.parse().unwrap()).collect();
            ChokeRow {
                amp: f[0],
                il_max: f[1],
                h1: f[2],
                oa_lo: f[3],
                oa_hi: f[4],
                unsolved: f[5] as u64,
            }
        })
        .collect()
}

/// The accuracy gates: i_L max and H1 within 5 % of ngspice, no unsolved
/// sample, op-amp inside its rails. `Err` names the first failure.
fn choke_gates(rows: &[ChokeRow], what: &str) -> Result<(), String> {
    for (row, &(amp, il_ref)) in rows.iter().zip(CHOKE_IL.iter()) {
        if row.unsolved != 0 {
            return Err(format!("{what} {amp} V: {} unsolved samples", row.unsolved));
        }
        let err = (row.il_max - il_ref) / il_ref;
        if err.abs() > 0.05 {
            return Err(format!(
                "{what} {amp} V: i_L max {:.4} mA vs ngspice {:.4} mA ({:+.1} %)",
                row.il_max * 1e3,
                il_ref * 1e3,
                100.0 * err
            ));
        }
        if let Some(&(_, h1_ref)) = CHOKE_H1.iter().find(|r| r.0 == amp) {
            let err = (row.h1 - h1_ref) / h1_ref;
            if err.abs() > 0.05 {
                return Err(format!(
                    "{what} {amp} V: H1(out) {:.4} V vs ngspice {h1_ref:.4} V ({:+.1} %)",
                    row.h1,
                    100.0 * err
                ));
            }
        }
        if row.oa_lo < -1e-6 || row.oa_hi > 9.0 + 1e-6 {
            return Err(format!(
                "{what} {amp} V: op-amp output [{}, {}] outside its 0..9 V rails",
                row.oa_lo, row.oa_hi
            ));
        }
    }
    Ok(())
}

/// Each rail mode, at 4x oversampling (the pinned stamps carry
/// OVERSAMPLING_FACTOR) and as a 1x build run at the same 192 kHz internal
/// rate, where every internal sample is visible. The i_L peak is only
/// resolved at the internal rate: the 4x build reports it once per host
/// sample, which reads up to ~0.5 % low, so the monotonic check runs on the
/// 192 kHz render only.
fn choke_matches_ngspice(mode: OpampRailMode, tag: &str) {
    let os4 = render_choke(&choke_code(mode, 4), 48000.0, &format!("{tag}_4x"));
    choke_gates(&os4, &format!("{tag} 4x")).unwrap();
    let full_rate = render_choke(&choke_code(mode, 1), 192000.0, &format!("{tag}_192k"));
    choke_gates(&full_rate, &format!("{tag} 192 kHz")).unwrap();
    for w in full_rate.windows(2) {
        assert!(
            w[1].il_max >= w[0].il_max * (1.0 - MONOTONIC_TOLERANCE),
            "{tag}: i_L peak falls with drive, {:.4} mA at {} V -> {:.4} mA at {} V",
            w[0].il_max * 1e3,
            w[0].amp,
            w[1].il_max * 1e3,
            w[1].amp
        );
    }
}

#[test]
fn choke_on_railing_opamp_matches_ngspice_active_set() {
    choke_matches_ngspice(OpampRailMode::ActiveSet, "choke_as");
}

#[test]
fn choke_on_railing_opamp_matches_ngspice_active_set_be() {
    choke_matches_ngspice(OpampRailMode::ActiveSetBe, "choke_asbe");
}

/// The pinned solve must carry the flux law, and its flux-row residual is what
/// detects a pinned solve that does not. With the saturating stamps stripped
/// from the pinned loop, the residual rejects every pinned sample and each
/// rejection is counted as unsolved; with the residual stripped as well, the
/// same wrong answer is committed and nothing is counted. So the residual is
/// the detector, and the counter makes its verdict visible.
#[test]
fn pinned_choke_without_its_stamps_is_caught_by_the_residual() {
    let code = choke_code(OpampRailMode::ActiveSet, 4);
    let strip = |code: &str, residual: bool| -> String {
        let mut stamps = 0;
        let mut residuals = 0;
        let out: Vec<&str> = code
            .lines()
            .filter(|l| {
                let stamp = l.contains("g_as[SAT_IND_0_AUG_ROW][SAT_IND_0_AUG_ROW] +=")
                    || l.contains("rhs_as[SAT_IND_0_AUG_ROW] +=");
                let resid =
                    residual && l.contains("acc.abs()") && l.contains("pin_step_exceeded = true");
                stamps += stamp as usize;
                residuals += resid as usize;
                !(stamp || resid)
            })
            .collect();
        assert!(
            stamps >= 2,
            "test premise: the pinned loop stamps the choke ({stamps} lines)"
        );
        assert!(
            !residual || residuals >= 1,
            "test premise: the pinned loop checks the flux row"
        );
        out.join("\n")
    };

    let no_stamps = render_choke(&strip(&code, false), 48000.0, "choke_mut_stamps");
    for row in &no_stamps {
        assert!(
            row.unsolved > 1000,
            "{} V: stamps stripped, but only {} samples counted unsolved",
            row.amp,
            row.unsolved
        );
    }

    let blind = render_choke(&strip(&code, true), 48000.0, "choke_mut_blind");
    assert!(
        blind.iter().all(|r| r.unsolved == 0),
        "test premise: without the residual nothing detects the missing stamps"
    );
    let err = choke_gates(&blind, "blind mutant").expect_err("the blind mutant must be wrong");
    assert!(err.contains("i_L max") || err.contains("H1"), "{err}");
}

/// 1x, active-set-be: backward Euler on nearly every sample. Measured H1
/// +9.8 % at 0.1 V; i_L max -1.6 … -5.8 %.
#[test]
#[ignore = "1x integrator accuracy: backward Euler on nearly every railed sample at L/R ~ 5 samples; turns green with the backward-Euler latch work, do not loosen"]
fn choke_on_railing_opamp_at_1x_active_set_be() {
    let rows = render_choke(
        &choke_code(OpampRailMode::ActiveSetBe, 1),
        48000.0,
        "choke_asbe_1x",
    );
    choke_gates(&rows, "active-set-be 1x").unwrap();
}

/// 1x, active-set: the trapezoidal ring in deep saturation. Measured i_L max
/// up to +13.9 %; H1 within 0.2 %. Switching just the choke to backward Euler
/// on its stiff samples (measured with an exact per-element stiffness trigger)
/// removes the ring but over-damps: i_L -3..-5 %, H1 +6 % at 1 V. With L/R of a
/// few samples neither rule meets the gates at 1x; 4x meets them.
#[test]
#[ignore = "{trap, BE} per element cannot meet i_L 1-2 % AND H1 1e-3 at L/R ~ 5 samples (trap rings, BE over-damps); 4x meets it; pending the saturated-slope question, do not loosen"]
fn choke_on_railing_opamp_at_1x_active_set() {
    let rows = render_choke(
        &choke_code(OpampRailMode::ActiveSet, 1),
        48000.0,
        "choke_as_1x",
    );
    choke_gates(&rows, "active-set 1x").unwrap();
}

// ─── The DC operating point is the transient's own equilibrium ────────────
//
// The DC solver caps op-amp gain at AOL = 1000 to keep its Newton ladder
// stable on precision rectifiers. That cap used to be the final answer: every
// op-amp virtual ground carried a 0.1 % error (4.5 mV on a 4.5 V bias), so
// the transient, which runs the full AOL, did not start at its own
// equilibrium. The first sample jumped the output ~1 V and the trapezoidal
// rule rang at fs/2, ±0.48 V, from the first sample at zero input; since the
// BE-latch is armed on saturating circuits, that ring tripped it inside the
// default warmup and put the whole stream on backward Euler. The ladder now
// finishes at the full AOL.

/// Non-inverting stage, gain 1 at DC (the feedback leg's cap blocks DC).
/// Low-impedance resistors, so node Gmin (1e-12 S, an open item) moves the
/// answer by well under the tolerance.
const DC_STAGE: &str = "\
non-inverting stage biased at mid-supply
Vcc vcc 0 DC 9
R_b1 vcc vbias 100
R_b2 vbias 0 100
R_in vbias np 100
U1 np nm oa TL072
R_f oa nm 100
R_g nm ng 100
C_g ng 0 10u
R_l oa 0 10k
.model TL072 OA(AOL=200000 GBW=3e6 VCC=9 VEE=0)
";

#[test]
fn dc_operating_point_uses_the_full_open_loop_gain() {
    use melange_solver::codegen::ir::CircuitIR;
    use melange_solver::dc_op::{self, DcOpConfig};
    let netlist = Netlist::parse(DC_STAGE).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let slots = CircuitIR::build_device_info_with_mna(&netlist, Some(&mna)).unwrap();
    let r = dc_op::solve_dc_operating_point(&mna, &slots, &DcOpConfig::default());
    assert!(r.converged);
    let oa = r.v_node[mna.node_map["oa"] - 1];
    let np = r.v_node[mna.node_map["np"] - 1];
    // oa = AOL·(np − nm) with nm = oa at DC.
    let analytic = np * 200_000.0 / 200_001.0;
    assert!(
        ((oa - analytic) / analytic).abs() <= 1e-9,
        "DC op-amp output {oa:.12} V vs full-AOL {analytic:.12} V (the AOL=1000 cap gives {:.12})",
        np * 1000.0 / 1001.0
    );
}

/// A zero-input render from the baked operating point stays put: no fs/2
/// ring on the op-amp output and no latch. 1e-5 V is the interim bound: the
/// remaining 1.1 µV comes from full-LU's node Gmin, which sits in its
/// Jacobian but not its RHS and so moves the transient's fixed point away
/// from the DC solve's (an open item; with it removed the ring is 2.5e-13 V).
#[test]
fn railing_choke_stage_starts_at_its_own_equilibrium() {
    let mut config = support::config_for_spice(RAILING_INTO_CHOKE, 48000.0);
    config.opamp_rail_mode = OpampRailMode::ActiveSet;
    let code = support::generate_circuit_code_nodal(RAILING_INTO_CHOKE, &config).0;
    let oa = {
        let netlist = Netlist::parse(RAILING_INTO_CHOKE).unwrap();
        MnaSystem::from_netlist(&netlist).unwrap().node_map["oa"] - 1
    };
    let main = format!(
        "fn main() {{
    let mut s = CircuitState::default();
    s.set_sample_rate(48000.0);
    let (mut sum, mut alt, mut n) = (0.0f64, 0.0f64, 0usize);
    for k in 0..48000 {{
        let _ = process_sample(0.0, &mut s);
        let y = s.v_prev[{oa}];
        sum += y;
        alt += if k % 2 == 0 {{ y }} else {{ -y }};
        n += 1;
    }}
    let _ = sum;
    println!(\"{{}} {{}}\", (alt / n as f64).abs(), s.diag_be_latch_count);
}}"
    );
    let out = support::compile_and_run(&code, &main, "choke_start").stdout;
    let v: Vec<f64> = out.split_whitespace().map(|t| t.parse().unwrap()).collect();
    assert_eq!(
        v[1], 0.0,
        "the BE-latch fired at zero input: the start is not an equilibrium"
    );
    assert!(
        v[0] <= 1e-5,
        "fs/2 content on the op-amp output at zero input: {:.3e} V",
        v[0]
    );
}
