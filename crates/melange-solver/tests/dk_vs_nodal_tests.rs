//! DK vs Nodal cross-validation tests.
//!
//! Verifies that the DK Schur codegen path and the Nodal full-LU codegen
//! path produce identical output for the same circuit and input. This proves
//! that both solver implementations are correct (or at least consistent).

mod support;

use melange_solver::codegen::CodegenConfig;

const SR: f64 = 48000.0;

fn config(spice: &str) -> CodegenConfig {
    support::config_for_spice(spice, SR)
}

fn assert_dk_nodal_match(spice: &str, tag: &str, freq: f64, amp: f64, tol: f64) {
    let c = config(spice);
    let dk_circuit = support::build_circuit(spice, &c, &format!("{tag}_dk"));
    let nodal_circuit = support::build_circuit_nodal(spice, &c, &format!("{tag}_nodal"));

    let n = 960; // 20ms
    let dk_out = support::run_sine(&dk_circuit, freq, amp, n, SR);
    let nodal_out = support::run_sine(&nodal_circuit, freq, amp, n, SR);

    support::assert_finite(&dk_out);
    support::assert_finite(&nodal_out);

    // Both should produce non-trivial output
    let dk_peak: f64 = dk_out.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    let nodal_peak: f64 = nodal_out.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    assert!(dk_peak > 1e-6, "{tag} DK: peak too low ({dk_peak:.6e})");
    assert!(nodal_peak > 1e-6, "{tag} Nodal: peak too low ({nodal_peak:.6e})");

    support::assert_samples_match(&dk_out, &nodal_out, tol, tag);
}

// ── Linear circuits ──────────────────────────────────────────────────

#[test]
fn test_dk_vs_nodal_rc_lowpass() {
    let spice = "\
RC Lowpass
R1 in out 1k
C1 out 0 1u
";
    assert_dk_nodal_match(spice, "rc_lowpass", 1000.0, 0.5, 1e-10);
}

// ── Nonlinear circuits ───────────────────────────────────────────────

#[test]
fn test_dk_vs_nodal_diode_clipper() {
    let spice = "\
Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
";
    assert_dk_nodal_match(spice, "diode_clip", 500.0, 0.5, 1e-4);
}

#[test]
fn test_dk_vs_nodal_jfet_cs() {
    let spice = "\
JFET Common Source
Cin in gate 10u
Rg gate 0 1Meg
J1 drain gate source J2N5457
Rd vdd drain 2.2k
Rs source 0 1k
Cs source 0 100u
Cout drain out 10u
Rload out 0 100k
Vdd vdd 0 DC 12
.model J2N5457 NJ(VTO=-2.0 IDSS=5e-3 LAMBDA=0.001)
";
    assert_dk_nodal_match(spice, "jfet_cs", 500.0, 0.1, 5e-3);
}

#[test]
fn test_dk_vs_nodal_mosfet_cs() {
    let spice = "\
MOSFET Common Source
Cin in gate 10u
R1 vdd gate 47k
R2 gate 0 100k
M1 drain gate source 0 NMOD
Rd vdd drain 1k
Rs source 0 100
Cs source 0 100u
Cout drain out 10u
Rload out 0 100k
Vdd vdd 0 DC 5
.model NMOD NM(VTO=2.0 KP=0.1 LAMBDA=0.01)
";
    assert_dk_nodal_match(spice, "mosfet_cs", 500.0, 0.1, 5e-3);
}

#[test]
fn test_dk_vs_nodal_triode_cc() {
    let spice = "\
12AX7 Common Cathode
Rin in 0 1Meg
Cin in grid 100n
Rg grid 0 1Meg
T1 grid plate cathode 12AX7
Rk cathode 0 1.5k
Ck cathode 0 25u
Rp vcc plate 100k
Cout plate out 100n
Rout out 0 1Meg
Vcc vcc 0 DC 250
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";
    // Triode has slightly different DC OP paths, allow wider tolerance
    assert_dk_nodal_match(spice, "triode_cc", 500.0, 0.5, 5e-2);
}
