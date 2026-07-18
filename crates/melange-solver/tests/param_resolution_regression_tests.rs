//! Regression tests for codegen parameter-resolution fixes (2026-07-18):
//!
//! 1. P-JFET VTO sign normalization: vendor PJF cards use the SPICE
//!    convention VTO < 0; melange's device convention stores VP positive
//!    for P-channel (the device model flips internally). Copying VTO
//!    verbatim double-flipped the pinch-off and left the device dead.
//! 2. NMOS VTO sign is PRESERVED (VTO < 0 = depletion-mode device).
//! 3. Unknown diode model falls back to the SPICE default diode
//!    (IS=1e-14, N=1.0), not the old 1N4148-IS/N=1.0 chimera.
//! 4. IBV defaults to SPICE3f5's 1e-3 (was 1e-10, which pushed the
//!    breakdown knee ~0.4·N V past BV).
//! 5. Self-heating CTH=0 is accepted (quasi-static junction temperature);
//!    CTH < 0 is rejected.

use melange_solver::codegen::ir::{CircuitIR, DeviceParams};
use melange_solver::parser::Netlist;

fn resolve_slots(spice: &str) -> Vec<melange_solver::codegen::ir::DeviceSlot> {
    let netlist = Netlist::parse(spice).expect("parse failed");
    CircuitIR::build_device_info(&netlist).expect("build_device_info failed")
}

// ============================================================================
// P-JFET VTO sign normalization
// ============================================================================

/// SPICE-convention PJF card (VTO=-1.8) must normalize to melange
/// convention vp=+1.8 and produce a device that conducts at Vgs=0.
#[test]
fn pjf_spice_convention_vto_normalized_and_conducts_at_vgs0() {
    let spice = "\
PJF SPICE convention
.model PJ1 PJF(VTO=-1.8 IDSS=3e-3 LAMBDA=0.002)
V1 vdd 0 DC -9
R1 vdd drain 4.7k
J1 drain 0 0 PJ1
";
    let slots = resolve_slots(spice);
    let jp = slots
        .iter()
        .find_map(|s| match &s.params {
            DeviceParams::Jfet(jp) => Some(jp),
            _ => None,
        })
        .expect("expected a JFET slot");

    assert!(jp.is_p_channel, "PJF must resolve as P-channel");
    assert!(
        (jp.vp - 1.8).abs() < 1e-12,
        "SPICE-convention VTO=-1.8 must normalize to melange vp=+1.8, got {}",
        jp.vp
    );

    // Evaluate through the devices crate: a healthy P-JFET conducts near
    // IDSS at Vgs=0. Under the old double-flip bug (vp stored as -1.8),
    // vgst = 0 - 1.8 < 0 → cutoff → electrically dead device.
    use melange_devices::jfet::{Jfet, JfetChannel};
    let dev = Jfet::new(JfetChannel::P, jp.vp, jp.idss);
    let id = dev.drain_current(0.0, -5.0);
    assert!(
        id.abs() > 0.5 * jp.idss,
        "P-JFET must conduct at Vgs=0 (|Id| near IDSS), got Id={id} vs IDSS={}",
        jp.idss
    );
}

/// A PJF card already in melange convention (VTO > 0) is accepted verbatim.
#[test]
fn pjf_positive_vto_accepted_as_melange_convention() {
    let spice = "\
PJF melange convention
.model PJ1 PJF(VTO=2.5 IDSS=5e-3)
J1 d 0 0 PJ1
R1 d 0 10k
";
    let slots = resolve_slots(spice);
    let jp = slots
        .iter()
        .find_map(|s| match &s.params {
            DeviceParams::Jfet(jp) => Some(jp),
            _ => None,
        })
        .expect("expected a JFET slot");
    assert!(
        (jp.vp - 2.5).abs() < 1e-12,
        "melange-convention VTO=2.5 must pass through unchanged, got {}",
        jp.vp
    );
}

/// N-channel VTO < 0 is the standard SPICE convention and already matches
/// melange's — it must be untouched by the PJF normalization.
#[test]
fn njf_negative_vto_unchanged() {
    let spice = "\
NJF standard
.model NJ1 NJF(VTO=-2.0 IDSS=2e-3)
J1 d 0 0 NJ1
R1 d 0 10k
";
    let slots = resolve_slots(spice);
    let jp = slots
        .iter()
        .find_map(|s| match &s.params {
            DeviceParams::Jfet(jp) => Some(jp),
            _ => None,
        })
        .expect("expected a JFET slot");
    assert!(!jp.is_p_channel);
    assert!(
        (jp.vp - (-2.0)).abs() < 1e-12,
        "N-channel VTO=-2.0 must pass through unchanged, got {}",
        jp.vp
    );
}

/// IDSS derived from BETA must use the NORMALIZED vp (sign-safe since vp is
/// squared, but the magnitude must come from the normalized binding).
#[test]
fn pjf_idss_from_beta_uses_normalized_vp() {
    let spice = "\
PJF beta card
.model PJ1 PJF(VTO=-1.8 BETA=1e-3)
J1 d 0 0 PJ1
R1 d 0 10k
";
    let slots = resolve_slots(spice);
    let jp = slots
        .iter()
        .find_map(|s| match &s.params {
            DeviceParams::Jfet(jp) => Some(jp),
            _ => None,
        })
        .expect("expected a JFET slot");
    let expected = 1e-3 * 1.8 * 1.8;
    assert!(
        (jp.idss - expected).abs() < 1e-12,
        "IDSS = BETA * vp^2 with normalized vp: expected {expected}, got {}",
        jp.idss
    );
}

// ============================================================================
// NMOS depletion-mode VTO sign preservation
// ============================================================================

/// NMOS with VTO < 0 is a depletion-mode device — the sign must be
/// PRESERVED (no PJF-style normalization for MOSFETs).
#[test]
fn nmos_negative_vto_preserved_as_depletion_mode() {
    let spice = "\
NMOS depletion
.model NDEP NMOS(KP=1e-3 VTO=-1.0)
M1 d g 0 0 NDEP
R1 d 0 10k
R2 g 0 1Meg
";
    let slots = resolve_slots(spice);
    let mp = slots
        .iter()
        .find_map(|s| match &s.params {
            DeviceParams::Mosfet(mp) => Some(mp),
            _ => None,
        })
        .expect("expected a MOSFET slot");
    assert!(!mp.is_p_channel);
    assert!(
        (mp.vt - (-1.0)).abs() < 1e-12,
        "NMOS VTO=-1.0 (depletion) must resolve with negative vt, got {}",
        mp.vt
    );
}

// ============================================================================
// Diode defaults: SPICE default fallback + IBV
// ============================================================================

/// Unknown diode model (no catalog hit, no IS on the card) must fall back
/// to the SPICE default diode IS=1e-14, N=1.0 — not the old chimera
/// (1N4148's IS=2.52e-9 paired with N=1.0).
#[test]
fn unknown_diode_model_falls_back_to_spice_default() {
    let spice = "\
Unknown diode card
.model MYSTERYDIODE D(RS=1)
D1 a 0 MYSTERYDIODE
R1 a 0 10k
";
    let slots = resolve_slots(spice);
    let dp = slots
        .iter()
        .find_map(|s| match &s.params {
            DeviceParams::Diode(dp) => Some(dp),
            _ => None,
        })
        .expect("expected a diode slot");
    assert!(
        (dp.is - 1e-14).abs() < 1e-26,
        "unknown diode model must default to SPICE IS=1e-14, got {}",
        dp.is
    );
    // N defaults to 1.0 → n_vt == VT_ROOM
    assert!(
        (dp.n_vt - melange_primitives::VT_ROOM).abs() < 1e-12,
        "unknown diode model must default to N=1.0 (n_vt=VT_ROOM), got n_vt={}",
        dp.n_vt
    );
}

/// IBV must default to SPICE3f5's 1e-3 so the breakdown knee lands at BV.
#[test]
fn diode_ibv_defaults_to_spice3f5_1e_minus_3() {
    let spice = "\
Zener default IBV
.model ZX D(IS=1e-14 N=1.0 BV=5.1)
D1 a 0 ZX
R1 a 0 10k
";
    let slots = resolve_slots(spice);
    let dp = slots
        .iter()
        .find_map(|s| match &s.params {
            DeviceParams::Diode(dp) => Some(dp),
            _ => None,
        })
        .expect("expected a diode slot");
    assert!(
        (dp.ibv - 1e-3).abs() < 1e-15,
        "IBV must default to 1e-3 (SPICE3f5), got {}",
        dp.ibv
    );
}

// ============================================================================
// Self-heating parameter validation (RTH/CTH)
// ============================================================================

/// CTH=0 with finite RTH is allowed (quasi-static junction temperature).
#[test]
fn diode_cth_zero_allowed_quasi_static() {
    let spice = "\
Quasi-static thermal diode
.model DTH D(IS=1e-14 N=1.0 RTH=500 CTH=0)
D1 a 0 DTH
R1 a 0 10k
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let res = CircuitIR::build_device_info(&netlist);
    assert!(
        res.is_ok(),
        "diode CTH=0 (quasi-static) must be accepted, got {:?}",
        res.err()
    );
}

/// Negative CTH is rejected at codegen time.
#[test]
fn diode_negative_cth_rejected() {
    let spice = "\
Bad thermal diode
.model DTH D(IS=1e-14 N=1.0 RTH=500 CTH=-1e-3)
D1 a 0 DTH
R1 a 0 10k
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let res = CircuitIR::build_device_info(&netlist);
    assert!(res.is_err(), "diode CTH<0 must be rejected");
}

/// RTH<=0 (finite) is rejected at codegen time.
#[test]
fn diode_nonpositive_rth_rejected() {
    let spice = "\
Bad thermal diode rth
.model DTH D(IS=1e-14 N=1.0 RTH=-100 CTH=1e-3)
D1 a 0 DTH
R1 a 0 10k
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let res = CircuitIR::build_device_info(&netlist);
    assert!(res.is_err(), "diode RTH<=0 must be rejected");
}

/// BJT CTH=0 allowed (quasi-static); CTH<0 rejected.
#[test]
fn bjt_cth_zero_allowed_negative_rejected() {
    let ok = "\
Quasi-static thermal BJT
.model QTH NPN(IS=1e-14 BF=200 BR=3 RTH=500 CTH=0)
Q1 c b 0 QTH
R1 c 0 10k
R2 b 0 100k
";
    let netlist = Netlist::parse(ok).expect("parse failed");
    assert!(
        CircuitIR::build_device_info(&netlist).is_ok(),
        "BJT CTH=0 (quasi-static) must be accepted"
    );

    let bad = "\
Bad thermal BJT
.model QTH NPN(IS=1e-14 BF=200 BR=3 RTH=500 CTH=-1)
Q1 c b 0 QTH
R1 c 0 10k
R2 b 0 100k
";
    let netlist = Netlist::parse(bad).expect("parse failed");
    assert!(
        CircuitIR::build_device_info(&netlist).is_err(),
        "BJT CTH<0 must be rejected"
    );
}

/// Tube CTH=0 with finite RTH is now allowed (quasi-static envelope
/// temperature) instead of erroring.
#[test]
fn tube_cth_zero_with_finite_rth_allowed() {
    let spice = "\
Quasi-static thermal triode
.model 12AX7T TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300 RTH=500 CTH=0 VBIAS_ALPHA=3e-4)
T1 g p k 12AX7T
R1 p 0 100k
R2 k 0 1.5k
R3 g 0 1Meg
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let res = CircuitIR::build_device_info(&netlist);
    assert!(
        res.is_ok(),
        "tube CTH=0 with finite RTH (quasi-static) must be accepted, got {:?}",
        res.err()
    );
}
