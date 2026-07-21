//! Pinned-gain tripwire suite (review-round-2 H4, proposal 2).
//!
//! One representative small-signal stage per gain-bearing device family,
//! each asserting its measured stage gain (settled-window RMS out/in) in
//! dB against a pinned literal ±0.2 dB. Purpose: convert a silent
//! calibration/model change (the July 2026 pattern — triode ×2, flicker
//! ×2400 — where nothing asserted an absolute level) into a loud failure
//! that forces the OOMOX_CONTRACT.md §7 process step.
//!
//! RE-PIN PROTOCOL (applies to every literal in this file): before
//! changing a pin, (1) produce the §7 downstream impact list — which
//! shipped plugins contain the device family, per-stage dB delta ×
//! cascade count, which §2.1 calibration rows go stale — and (2)
//! re-derive the independent reference cited next to the pin; never
//! re-pin from melange output alone.
//!
//! Reference provenance (all measured 2026-07-21, ngspice-42,
//! `.OPTIONS INTERP reltol=1e-4`, Thevenin PWL 1 Ω drive, settled-RMS
//! gain over the identical window):
//! - triode family: pinned in `golden_ref_tests.rs`
//!   (`golden_triode_cc_small_sine_500hz`, ngspice B-source Koren) and
//!   `cascade_gain_gate_tests.rs` (per-stage + ×8 cascade).
//! - BJT CE (golden circuit @ 5 mV): ngspice 47.3565 dB, melange
//!   47.3629 dB (Δ 0.0064 dB at this drive; the large-signal GP gain
//!   ratio 1.024 documented in spice_validation.rs does not apply at
//!   small signal).
//! - JFET CS (golden circuit @ 20 mV, ngspice NJF BETA=IDSS/VTO²=1.25e-3):
//!   ngspice 13.9859 dB, melange 13.9859 dB (Δ < 0.0001 dB).
//! - op-amp NI/INV (golden circuits @ 0.1 V): exact linear-algebra
//!   solution of the VCCS network including the 1 Ω input Thevenin:
//!   NI +10.999395 → 20.8274 dB, INV −9.989461 → 19.9908 dB; measured
//!   melange 20.8299 / 19.9934 dB (Δ ≈ 0.0026 dB = 1 kHz vs DC analytic
//!   + 5 Hz blocker residue). The same circuits' full waveforms are
//!   ngspice-pinned to 1.3e-8 V in the goldens.
//! - pentode (EL84 bench @ 20 mV): NO independent reference exists —
//!   ngspice has no pentode Koren twin and building a B-source
//!   screen-current model was out of scope. Pin is melange-at-HEAD,
//!   explicitly a DRIFT TRIPWIRE ONLY, labeled as such below.
//!
//! Failure-mode verification (2026-07-21): removing the Koren ×2 in a
//! scratch copy shifts the triode/cascade pins by 4–30 dB (see
//! cascade_gain_gate_tests.rs); scaling any single family's device
//! current by ×2 moves its pin here by ≈ 6 dB — 30× the gate.

mod support;

use melange_solver::codegen::CodegenConfig;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const SR: f64 = 48_000.0;

/// Golden BJT CE stage (same netlist as golden_ref_tests.rs).
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

/// Golden JFET CS stage (same netlist as golden_ref_tests.rs).
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

/// Golden op-amp circuits (same netlists as golden_ref_tests.rs).
const OPAMP_NONINVERTING: &str = "\
Non-Inverting Amplifier
R1 0 vminus 1k
R2 vminus out 10k
U1 in vminus out OPA
.model OPA OA(AOL=200000 ROUT=1)
";

const OPAMP_INVERTING: &str = "\
Inverting Amplifier
R1 in vminus 1k
R2 vminus out 10k
U1 0 vminus out OPA
.model OPA OA(AOL=200000 ROUT=1)
";

/// EL84 pentode bench (same netlist as the noise-suite partition bench).
const PENTODE_EL84: &str = "\
EL84 pentode single stage
Rin in 0 1Meg
Cin in grid 100n
Rg grid 0 1Meg
P1 plate grid cathode screen EL84
Rk cathode 0 130
Ck cathode 0 100u
Rscreen vcc screen 1k
Cscreen screen 0 47u
Rp vcc plate 4.7k
Cout plate out 1u
Rout out 0 100k
V1 vcc 0 DC 300
.model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275.0 KP=152.4 KVB=4015.8 ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148)
";

fn config_for(spice: &str, name: &str) -> CodegenConfig {
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    CodegenConfig {
        circuit_name: name.to_string(),
        sample_rate: SR,
        input_node: mna.node_map["in"] - 1,
        output_nodes: vec![mna.node_map["out"] - 1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

/// Settled-window RMS gain in dB. `settle` must leave an integer number
/// of cycles so the RMS is unbiased.
fn measure_gain_db(
    spice: &str,
    tag: &str,
    freq: f64,
    amp: f64,
    n_samples: usize,
    settle: usize,
) -> f64 {
    let c = config_for(spice, tag);
    let circuit = support::build_circuit(spice, &c, tag);
    let out = support::run_sine(&circuit, freq, amp, n_samples, SR);
    support::assert_finite(&out);
    let settled = &out[settle..];
    let out_rms = (settled.iter().map(|v| v * v).sum::<f64>() / settled.len() as f64).sqrt();
    20.0 * (out_rms / (amp / std::f64::consts::SQRT_2)).log10()
}

fn assert_pinned(gain_db: f64, pin: f64, family: &str, reference: &str) {
    assert!(
        (gain_db - pin).abs() < 0.2,
        "{family} stage gain drifted: measured {gain_db:+.4} dB vs pinned {pin:+.4} dB \
         (reference: {reference}). Produce the OOMOX_CONTRACT §7 downstream impact \
         list and re-derive the independent reference before re-pinning."
    );
}

/// BJT family. 5 mV drive keeps the CE stage in its small-signal region
/// (~1.2 V p-p out) where GP parity with ngspice is 0.0064 dB.
#[test]
fn pinned_gain_bjt_ce() {
    let g = measure_gain_db(BJT_CE, "tripwire_bjt", 500.0, 0.005, 4800, 2400);
    assert_pinned(
        g,
        47.3629,
        "BJT CE",
        "ngspice-42 same deck, 47.3565 dB (Δ 0.0064 dB), 2026-07-21",
    );
}

/// JFET family. Shichman-Hodges vs ngspice NJF (BETA=IDSS/VTO²) agree to
/// <0.0001 dB at this drive.
#[test]
fn pinned_gain_jfet_cs() {
    let g = measure_gain_db(JFET_CS, "tripwire_jfet", 500.0, 0.02, 4800, 2400);
    assert_pinned(
        g,
        13.9859,
        "JFET CS",
        "ngspice-42 NJF twin, 13.9859 dB (Δ <0.0001 dB), 2026-07-21",
    );
}

/// Op-amp family, noninverting (closed-loop magnitude; polarity is gated
/// open-loop in opamp_polarity_gate_tests.rs).
#[test]
fn pinned_gain_opamp_noninverting() {
    let g = measure_gain_db(OPAMP_NONINVERTING, "tripwire_oani", 1000.0, 0.1, 480, 240);
    assert_pinned(
        g,
        20.8299,
        "op-amp noninverting",
        "exact VCCS linear solve +10.999395 = 20.8274 dB; ngspice waveform \
         pinned to 1.3e-8 V in the golden",
    );
}

/// Op-amp family, inverting.
#[test]
fn pinned_gain_opamp_inverting() {
    let g = measure_gain_db(OPAMP_INVERTING, "tripwire_oainv", 1000.0, 0.1, 480, 240);
    assert_pinned(
        g,
        19.9934,
        "op-amp inverting",
        "exact VCCS linear solve −9.989461 = 19.9908 dB; ngspice waveform \
         pinned to 1.1e-8 V in the golden",
    );
}

/// Pentode family — DRIFT TRIPWIRE ONLY. The pin is melange-at-HEAD
/// (2026-07-21); no independent reference exists (no ngspice pentode
/// Koren twin; a B-source screen-current encoding of the
/// Rational/variable-mu screen form was judged out of scope in
/// review-round-2). This gate cannot adjudicate correctness — it exists
/// so a pentode calibration change is LOUD instead of silent. If it
/// fires on an intended model fix, that fix must ship with the §7 impact
/// list (el84/ac15/tweed/kt88-pp/plexi plugins) and, ideally, the
/// still-missing independent pentode reference.
#[test]
fn pinned_gain_pentode_el84_head_tripwire() {
    let g = measure_gain_db(PENTODE_EL84, "tripwire_pent", 500.0, 0.02, 4800, 2400);
    assert_pinned(
        g,
        29.9108,
        "EL84 pentode",
        "melange-at-HEAD 2026-07-21 — drift tripwire only, no independent reference",
    );
}
