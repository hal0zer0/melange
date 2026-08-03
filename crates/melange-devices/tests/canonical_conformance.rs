//! Canonical device-model conformance — verification against the *published*
//! model equations, computed independently in-test, NOT against ngspice.
//!
//! The correctness audit found that essentially all device correctness in
//! melange grounds out in ngspice waveform correlation: the device docs cite
//! ngspice C-source line numbers, and no test checks a device against its
//! originating publication. That is circular — a shared model error would pass.
//! These tests break the circularity for the closed-form Level-1 devices by
//! encoding the canonical equations directly (with the arithmetic of each
//! anchor point shown so a reader can verify it by hand) and by asserting
//! model-defining physics invariants that hold regardless of parameterization
//! convention (square-law scaling, region continuity, Id=IDSS at Vgs=0, the
//! Shockley exponential ratio).
//!
//! References:
//!   Shichman & Hodges, "Modeling and simulation of insulated-gate field-effect
//!     transistor switching circuits," IEEE JSSC SC-3(3), 1968 — the SPICE
//!     Level-1 MOSFET square law.  Same square-law family underlies the SPICE
//!     JFET model (IDSS/Vp parameterization).
//!   Shockley, "The theory of p-n junctions in semiconductors," BSTJ 28, 1949.

use melange_devices::{
    DiodeShockley, Jfet, JfetChannel, Mosfet, MosfetChannelType as ChannelType, VT_ROOM,
};

const REL: f64 = 1e-12; // machine-precision agreement with the closed form

fn assert_rel(got: f64, want: f64, ctx: &str) {
    let denom = want.abs().max(1e-30);
    let rel = (got - want).abs() / denom;
    assert!(
        rel < REL,
        "{ctx}: got {got:.15e}, canonical {want:.15e}, rel {rel:.3e}"
    );
}

// ── MOSFET — Shichman-Hodges Level 1 ────────────────────────────────────
//
// N-channel, forward region (Vds ≥ 0). Overdrive Vov = Vgs − VT.
//   triode      (Vds < Vov):  Id = kp·(Vov·Vds − ½Vds²)·(1 + λVds)
//   saturation  (Vds ≥ Vov):  Id = ½·kp·Vov²·(1 + λVds)
// (melange folds SPICE's KP·W/L into a single `kp`; this is the canonical
// β/2 square law with β = kp.)

fn mos_triode_ref(kp: f64, vov: f64, lambda: f64, vds: f64) -> f64 {
    kp * (vov * vds - 0.5 * vds * vds) * (1.0 + lambda * vds)
}
fn mos_sat_ref(kp: f64, vov: f64, lambda: f64, vds: f64) -> f64 {
    0.5 * kp * vov * vov * (1.0 + lambda * vds)
}

#[test]
fn mosfet_matches_shichman_hodges_operating_points() {
    let vt = 2.0;
    let kp = 0.1;
    let m = Mosfet::new(ChannelType::N, vt, kp, 0.0);

    // Vgs=4 ⇒ Vov=2; Vds=5 ≥ Vov ⇒ saturation.  Id = ½·0.1·2² = 0.2 A.
    assert_rel(m.drain_current(4.0, 5.0), 0.2, "MOS sat Id(4,5)");
    assert_rel(
        m.drain_current(4.0, 5.0),
        mos_sat_ref(kp, 2.0, 0.0, 5.0),
        "MOS sat vs ref",
    );

    // Vds=1 < Vov=2 ⇒ triode.  Id = 0.1·(2·1 − ½·1²) = 0.1·1.5 = 0.15 A.
    assert_rel(m.drain_current(4.0, 1.0), 0.15, "MOS triode Id(4,1)");
    assert_rel(
        m.drain_current(4.0, 1.0),
        mos_triode_ref(kp, 2.0, 0.0, 1.0),
        "MOS triode vs ref",
    );

    // Vgs=1 ⇒ Vov=−1 < 0 ⇒ cutoff (melange keeps a ~1e-12 subthreshold leak).
    assert!(
        m.drain_current(1.0, 5.0).abs() < 1e-10,
        "MOS cutoff should be ~0"
    );
}

#[test]
fn mosfet_triode_saturation_continuity() {
    // At Vds = Vov the two branches must agree (no kink). Vgs=4 ⇒ Vov=2.
    let m = Mosfet::new(ChannelType::N, 2.0, 0.1, 0.0);
    let boundary = m.drain_current(4.0, 2.0);
    assert_rel(
        mos_triode_ref(0.1, 2.0, 0.0, 2.0),
        mos_sat_ref(0.1, 2.0, 0.0, 2.0),
        "ref continuity",
    );
    assert_rel(boundary, 0.2, "MOS boundary value");
}

#[test]
fn mosfet_square_law_and_lambda_are_canonical() {
    // Square law: in saturation Id ∝ Vov², independent of kp/VT convention.
    // Vgs=4⇒Vov=2, Vgs=3⇒Vov=1 ⇒ ratio must be exactly 4.
    let m = Mosfet::new(ChannelType::N, 2.0, 0.1, 0.0);
    let ratio = m.drain_current(4.0, 5.0) / m.drain_current(3.0, 5.0);
    assert_rel(ratio, 4.0, "MOS square-law Vov² scaling");

    // Channel-length modulation is the exact linear factor (1 + λ·Vds).
    let ml = Mosfet::new(ChannelType::N, 2.0, 0.1, 0.01);
    // Vov=2, Vds=5 ⇒ Id = 0.2·(1 + 0.01·5) = 0.21 A.
    assert_rel(ml.drain_current(4.0, 5.0), 0.21, "MOS λ modulation");
}

// ── JFET — Shichman-Hodges (IDSS/Vp) ────────────────────────────────────
//
// N-channel: Vp < 0, overdrive vgst = Vgs − Vp = Vgs + |Vp|.
//   triode     (Vds < vgst):  Id = (IDSS/Vp²)·(2·vgst·Vds − Vds²)·(1 + λVds)
//   saturation (Vds ≥ vgst):  Id = IDSS·(vgst/|Vp|)²·(1 + λVds)

fn jfet_sat_ref(idss: f64, vp_abs: f64, vgst: f64, lambda: f64, vds: f64) -> f64 {
    idss * (vgst / vp_abs).powi(2) * (1.0 + lambda * vds)
}
fn jfet_triode_ref(idss: f64, vp_abs: f64, vgst: f64, lambda: f64, vds: f64) -> f64 {
    idss / (vp_abs * vp_abs) * (2.0 * vgst * vds - vds * vds) * (1.0 + lambda * vds)
}

#[test]
fn jfet_matches_shichman_hodges_operating_points() {
    let idss = 2e-3;
    let vp_abs = 2.0; // N-channel Vp = −2
    let mut j = Jfet::new(JfetChannel::N, -2.0, idss);
    j.lambda = 0.0; // isolate the square law (new() defaults λ=0.001)

    // Vgs=0 ⇒ vgst=2=|Vp|; Vds=5 ≥ vgst ⇒ saturation.
    // Id = IDSS·(2/2)² = IDSS = 2e-3 A — the defining property Id=IDSS at Vgs=0.
    assert_rel(j.drain_current(0.0, 5.0), idss, "JFET Id=IDSS at Vgs=0");

    // Vgs=−1 ⇒ vgst=1; saturation.  Id = 2e-3·(1/2)² = 5e-4 A.
    assert_rel(j.drain_current(-1.0, 5.0), 5e-4, "JFET sat Id(−1,5)");
    assert_rel(
        j.drain_current(-1.0, 5.0),
        jfet_sat_ref(idss, vp_abs, 1.0, 0.0, 5.0),
        "JFET sat vs ref",
    );

    // Vgs=0, Vds=1 < vgst=2 ⇒ triode.
    // Id = (2e-3/4)·(2·2·1 − 1²) = 5e-4·3 = 1.5e-3 A.
    assert_rel(j.drain_current(0.0, 1.0), 1.5e-3, "JFET triode Id(0,1)");
    assert_rel(
        j.drain_current(0.0, 1.0),
        jfet_triode_ref(idss, vp_abs, 2.0, 0.0, 1.0),
        "JFET triode vs ref",
    );

    // Vgs=−2 ⇒ vgst=0 ⇒ pinch-off/cutoff.
    assert!(
        j.drain_current(-2.0, 5.0).abs() < 1e-10,
        "JFET cutoff at pinch-off"
    );
}

#[test]
fn jfet_triode_saturation_continuity() {
    // At Vds = vgst the branches must agree. Vgs=0 ⇒ vgst=2.
    let mut j = Jfet::new(JfetChannel::N, -2.0, 2e-3);
    j.lambda = 0.0;
    let boundary = j.drain_current(0.0, 2.0);
    assert_rel(
        jfet_triode_ref(2e-3, 2.0, 2.0, 0.0, 2.0),
        jfet_sat_ref(2e-3, 2.0, 2.0, 0.0, 2.0),
        "JFET ref continuity",
    );
    assert_rel(boundary, 2e-3, "JFET boundary = IDSS");
}

#[test]
fn jfet_square_law_and_lambda_are_canonical() {
    // Saturation square law: Id ∝ vgst². Vgs=0⇒vgst=2, Vgs=−1⇒vgst=1 ⇒ ratio 4.
    let mut j = Jfet::new(JfetChannel::N, -2.0, 2e-3);
    j.lambda = 0.0;
    let ratio = j.drain_current(0.0, 5.0) / j.drain_current(-1.0, 5.0);
    assert_rel(ratio, 4.0, "JFET square-law vgst² scaling");

    // Channel-length modulation is the exact (1 + λ·Vds) factor.
    let mut jl = Jfet::new(JfetChannel::N, -2.0, 2e-3);
    jl.lambda = 0.01;
    // vgst=2=|Vp|, Vds=5 ⇒ Id = IDSS·1·(1 + 0.01·5) = 2e-3·1.05 = 2.1e-3.
    assert_rel(jl.drain_current(0.0, 5.0), 2.1e-3, "JFET λ modulation");
}

// ── Diode — Shockley ────────────────────────────────────────────────────
//   Id = IS·(exp(Vd/(N·VT)) − 1)

fn shockley_ref(is: f64, n: f64, v: f64) -> f64 {
    is * ((v / (n * VT_ROOM)).exp() - 1.0)
}

#[test]
fn diode_matches_shockley_equation() {
    let is = 1e-14;
    let n = 1.0;
    let d = DiodeShockley::new_room_temp(is, n);
    // Stay below the IS-aware clamp (x = V/(N·VT) < 40 ⇒ V < ~1.03 V).
    for &v in &[-0.1, 0.0, 0.2, 0.4, 0.6, 0.7] {
        assert_rel(
            d.current_at(v),
            shockley_ref(is, n, v),
            &format!("Shockley Id({v})"),
        );
    }
    // At Vd=0 the current is exactly zero.
    assert_rel(d.current_at(0.0), 0.0, "Shockley Id(0)=0");
}

#[test]
fn diode_exponential_ratio_is_canonical() {
    // Model-defining invariant, independent of IS: raising Vd by exactly N·VT
    // multiplies the forward current (Id+IS) by e. Emission coefficient N=1.752.
    let is = 2.52e-9;
    let n = 1.752;
    let d = DiodeShockley::new_room_temp(is, n);
    let v0 = 0.4;
    let v1 = v0 + n * VT_ROOM;
    let ratio = (d.current_at(v1) + is) / (d.current_at(v0) + is);
    assert_rel(ratio, std::f64::consts::E, "Shockley ΔV=N·VT ⇒ ×e");
}
