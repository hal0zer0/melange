//! Golden reference tests: codegen output vs recorded runtime solver output.
//!
//! These tests verify that the codegen pipeline produces output matching
//! the runtime solver's recorded output within tolerance. After the runtime
//! solver is deleted, these become the primary correctness tests.
//!
//! Golden files were recorded by `record_golden_refs.rs` and live in
//! `tests/golden/*.json`.

mod support;

use melange_solver::codegen::CodegenConfig;

const SR: f64 = 48000.0;

// ── Circuit definitions (same as record_golden_refs.rs) ────────────────

const RC_LOWPASS: &str = "\
RC Lowpass
R1 in out 1k
C1 out 0 1u
";

const RC_HIGHPASS: &str = "\
RC Highpass
C1 in out 1u
R1 out 0 1k
";

const DIODE_CLIPPER: &str = "\
Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
";

const BJT_CE: &str = "\
BJT Common Emitter
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYBJTCE
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYBJTCE NPN(IS=1e-14 BF=200 BR=3)
";

const JFET_CS: &str = "\
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

const MOSFET_CS: &str = "\
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

const TRIODE_CC: &str = "\
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

const OPAMP_INVERTING: &str = "\
Inverting Amplifier
R1 in vminus 1k
R2 vminus out 10k
U1 0 vminus out OPA
.model OPA OA(AOL=200000 ROUT=1)
";

const OPAMP_NONINVERTING: &str = "\
Non-Inverting Amplifier
R1 0 vminus 1k
R2 vminus out 10k
U1 in vminus out OPA
.model OPA OA(AOL=200000 ROUT=1)
";

// ── Helpers ────────────────────────────────────────────────────────────

fn config(spice: &str) -> CodegenConfig {
    support::config_for_spice(spice, SR)
}

// ── Linear circuit golden ref tests ────────────────────────────────────

#[test]
fn golden_rc_lowpass_step() {
    let c = config(RC_LOWPASS);
    let circuit = support::build_circuit(RC_LOWPASS, &c, "g_rc_step");
    let output = support::run_step(&circuit, 1.0, 300, SR);
    support::assert_matches_golden(&output, "rc_lowpass_step.json", 1e-10);
}

#[test]
fn golden_rc_lowpass_sine_1k() {
    let c = config(RC_LOWPASS);
    let circuit = support::build_circuit(RC_LOWPASS, &c, "g_rc_sine");
    let output = support::run_sine(&circuit, 1000.0, 0.5, 480, SR);
    support::assert_matches_golden(&output, "rc_lowpass_sine_1k.json", 1e-10);
}

#[test]
fn golden_rc_highpass_sine_1k() {
    let c = config(RC_HIGHPASS);
    let circuit = support::build_circuit(RC_HIGHPASS, &c, "g_rc_hp");
    let output = support::run_sine(&circuit, 1000.0, 0.5, 480, SR);
    support::assert_matches_golden(&output, "rc_highpass_sine_1k.json", 1e-10);
}

#[test]
fn golden_opamp_inverting_sine_1k() {
    let c = config(OPAMP_INVERTING);
    let circuit = support::build_circuit(OPAMP_INVERTING, &c, "g_opamp_inv");
    let output = support::run_sine(&circuit, 1000.0, 0.1, 480, SR);
    support::assert_matches_golden(&output, "opamp_inverting_sine_1k.json", 1e-10);
}

#[test]
fn golden_opamp_noninverting_sine_1k() {
    let c = config(OPAMP_NONINVERTING);
    let circuit = support::build_circuit(OPAMP_NONINVERTING, &c, "g_opamp_ni");
    let output = support::run_sine(&circuit, 1000.0, 0.1, 480, SR);
    support::assert_matches_golden(&output, "opamp_noninverting_sine_1k.json", 1e-10);
}

// ── Nonlinear circuit golden ref tests ─────────────────────────────────

#[test]
fn golden_diode_clipper_sine_500hz() {
    let c = config(DIODE_CLIPPER);
    let circuit = support::build_circuit(DIODE_CLIPPER, &c, "g_diode");
    let output = support::run_sine(&circuit, 500.0, 1.0, 960, SR);
    support::assert_matches_golden(&output, "diode_clipper_sine_500hz.json", 1e-6);
}

#[test]
fn golden_bjt_ce_sine_500hz() {
    let c = config(BJT_CE);
    let circuit = support::build_circuit(BJT_CE, &c, "g_bjt");
    let output = support::run_sine(&circuit, 500.0, 0.05, 960, SR);
    support::assert_matches_golden(&output, "bjt_ce_sine_500hz.json", 1e-4);
}

#[test]
fn golden_jfet_cs_sine_500hz() {
    let c = config(JFET_CS);
    let circuit = support::build_circuit(JFET_CS, &c, "g_jfet");
    let output = support::run_sine(&circuit, 500.0, 0.1, 960, SR);
    support::assert_matches_golden(&output, "jfet_cs_sine_500hz.json", 1e-4);
}

#[test]
fn golden_mosfet_cs_sine_500hz() {
    let c = config(MOSFET_CS);
    let circuit = support::build_circuit(MOSFET_CS, &c, "g_mosfet");
    let output = support::run_sine(&circuit, 500.0, 0.1, 960, SR);
    support::assert_matches_golden(&output, "mosfet_cs_sine_500hz.json", 1e-4);
}

#[test]
fn golden_triode_cc_sine_500hz() {
    let c = config(TRIODE_CC);
    let circuit = support::build_circuit(TRIODE_CC, &c, "g_triode");
    let output = support::run_sine(&circuit, 500.0, 0.5, 960, SR);
    // Triode has slightly different DC OP between runtime (Ebers-Moll) and codegen (GP).
    // 2e-4 tolerance is well within inaudible range.
    support::assert_matches_golden(&output, "triode_cc_sine_500hz.json", 2e-4);
}
