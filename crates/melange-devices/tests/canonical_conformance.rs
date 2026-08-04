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

use melange_devices::tube::KorenPentode;
use melange_devices::{
    BjtEbersMoll, BjtGummelPoon, BjtPolarity, DiodeShockley, Jfet, JfetChannel, Mosfet,
    MosfetChannelType as ChannelType, Vca, VT_ROOM,
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

// ── BJT — SPICE Gummel-Poon ─────────────────────────────────────────────
//
// Canonical SGP (Gummel & Poon 1970; SPICE Gummel-Poon as documented in
// Antognetti & Massobrio, "Semiconductor Device Modeling with SPICE"):
//   q1 = 1/(1 − Vbe/VAR − Vbc/VAF)
//   cbe = IS·(exp(Vbe/NF·Vt) − 1),  cbc = IS·(exp(Vbc/NR·Vt) − 1)
//   q2 = cbe/IKF + cbc/IKR
//   qb = q1·(1 + √(1+4·q2))/2
//   Ic = IS·(exp(Vbe/NF·Vt) − exp(Vbc/NR·Vt))/qb − IS/BR·(exp(Vbc/NR·Vt) − 1)
//   Ib = IS/BF·(exp(Vbe/NF·Vt) − 1) + IS/BR·(exp(Vbc/NR·Vt) − 1)   [ISE=ISC=0]
// The base-charge qb modulates the transport (collector) current only — the
// base current is pure Ebers-Moll. All tests below use NPN, ISE=ISC=0.

const IS: f64 = 1e-14;
const BF: f64 = 200.0;
const BR: f64 = 5.0;

/// Clamped exp mirroring the device crate's safe_exp (exact within ±40).
fn ce(x: f64) -> f64 {
    x.clamp(-40.0, 40.0).exp()
}

fn qb_ref(vaf: f64, var: f64, ikf: f64, ikr: f64, vbe: f64, vbc: f64) -> f64 {
    let vt = VT_ROOM;
    let q1_denom = 1.0 - vbe / var - vbc / vaf;
    let q1 = if q1_denom <= 0.0 || q1_denom.abs() < 1e-30 {
        1.0
    } else {
        1.0 / q1_denom
    };
    let cbe = IS * (ce(vbe / vt) - 1.0);
    let cbc = IS * (ce(vbc / vt) - 1.0);
    let q2 = cbe / ikf + cbc / ikr;
    q1 * (1.0 + (1.0 + 4.0 * q2).max(0.0).sqrt()) / 2.0
}

fn ic_ref(vaf: f64, var: f64, ikf: f64, ikr: f64, vbe: f64, vbc: f64) -> f64 {
    let vt = VT_ROOM;
    let icc = IS * (ce(vbe / vt) - ce(vbc / vt));
    icc / qb_ref(vaf, var, ikf, ikr, vbe, vbc) - IS / BR * (ce(vbc / vt) - 1.0)
}

fn ib_ref(vbe: f64, vbc: f64) -> f64 {
    let vt = VT_ROOM;
    IS / BF * (ce(vbe / vt) - 1.0) + IS / BR * (ce(vbc / vt) - 1.0)
}

fn npn(vaf: f64, var: f64, ikf: f64, ikr: f64) -> BjtGummelPoon {
    let em = BjtEbersMoll::new_room_temp(IS, BF, BR, BjtPolarity::Npn);
    BjtGummelPoon::new(em, vaf, var, ikf, ikr)
}

#[test]
fn bjt_gp_reduces_to_ebers_moll() {
    // All GP params infinite ⇒ qb ≡ 1 ⇒ pure Ebers-Moll transport model.
    let inf = f64::INFINITY;
    let gp = npn(inf, inf, inf, inf);
    // Forward active, Vbc=0: Ic = IS·(exp(Vbe/Vt) − 1), Ib = Ic/BF ⇒ β = BF.
    let (vbe, vbc) = (0.65, 0.0);
    let ic = gp.collector_current(vbe, vbc);
    let ib = gp.base_current(vbe, vbc);
    assert_rel(ic, IS * (ce(vbe / VT_ROOM) - 1.0), "GP→EM Ic");
    assert_rel(ib, ic / BF, "GP→EM Ib = Ic/BF");
    assert_rel(ic / ib, BF, "GP→EM β = BF");
}

#[test]
fn bjt_gp_matches_canonical_at_operating_points() {
    // 2N2222A-like GP params; verify Ic and Ib against the SGP equations across
    // forward-active and into high injection.
    let (vaf, var, ikf, ikr) = (100.0, 10.0, 0.3, 6e-3);
    let gp = npn(vaf, var, ikf, ikr);
    for &(vbe, vbc) in &[
        (0.55, 0.0),
        (0.6, -2.0),
        (0.65, -5.0),
        (0.7, 0.0),
        (0.72, -3.0),
    ] {
        assert_rel(
            gp.collector_current(vbe, vbc),
            ic_ref(vaf, var, ikf, ikr, vbe, vbc),
            &format!("GP Ic({vbe},{vbc})"),
        );
        assert_rel(
            gp.base_current(vbe, vbc),
            ib_ref(vbe, vbc),
            &format!("GP Ib({vbe},{vbc})"),
        );
    }
}

#[test]
fn bjt_gp_early_and_high_injection_are_canonical() {
    // Early effect: at fixed Vbe, more reverse Vbc (higher Vce) raises Ic.
    let gp = npn(100.0, 10.0, f64::INFINITY, f64::INFINITY);
    assert!(
        gp.collector_current(0.6, -5.0) > gp.collector_current(0.6, 0.0),
        "Early effect: Ic must rise with Vce"
    );

    // High injection: with only IKF finite (no Early), β = Ic/Ib = BF/qb exactly
    // — qb modulates Ic but not Ib. Verify β·qb = BF to machine precision, and
    // that β rolls off (qb > 1) as Ic approaches the IKF knee.
    let inf = f64::INFINITY;
    let ikf = 0.3;
    let gp = npn(inf, inf, ikf, inf);
    for &vbe in &[0.5, 0.65, 0.75] {
        let beta = gp.collector_current(vbe, 0.0) / gp.base_current(vbe, 0.0);
        let qb = qb_ref(inf, inf, ikf, inf, vbe, 0.0);
        assert_rel(beta * qb, BF, &format!("GP β·qb = BF at Vbe={vbe}"));
    }
    let beta_lo = gp.collector_current(0.5, 0.0) / gp.base_current(0.5, 0.0);
    let beta_hi = gp.collector_current(0.75, 0.0) / gp.base_current(0.75, 0.0);
    assert!(
        beta_hi < 0.9 * beta_lo,
        "high-injection β rolloff: β(0.75V)={beta_hi:.1} should be well below β(0.5V)={beta_lo:.1}"
    );
}

// ── VCA — Blackmer current-mode exponential (THAT 2180 / DBX 2150) ───────
//   gain(Vc) = G0 · exp(−Vc / VSCALE),   I = gain · V_sig
// The exponential control law is the defining property of a Blackmer VCA: the
// control port is dB-linear. THAT 2180A spec is 6.1 mV/dB, encoded as
// VSCALE = 0.05298 V.

#[test]
fn vca_gain_law_is_canonical_blackmer() {
    let vscale = 0.05298;
    let g0 = 1.0;
    let vca = Vca::new(vscale, g0);
    assert_rel(vca.gain(0.0), g0, "VCA gain(0) = G0");
    // VSCALE is the e-folding control voltage.
    assert_rel(
        vca.gain(vscale),
        g0 / std::f64::consts::E,
        "VCA gain(VSCALE) = G0/e",
    );
    assert_rel(
        vca.gain(-vscale),
        g0 * std::f64::consts::E,
        "VCA gain(−VSCALE) = G0·e",
    );
    // Exact exponential form.
    for &vc in &[-0.2, -0.1, 0.0, 0.05, 0.1, 0.2] {
        assert_rel(
            vca.gain(vc),
            g0 * (-vc / vscale).exp(),
            &format!("VCA gain law @ {vc}"),
        );
    }
}

#[test]
fn vca_control_is_db_linear() {
    // THAT 2180A: a 6.1 mV control step changes gain by exactly −1.0 dB.
    let vca = Vca::new(0.05298, 1.0);
    let db = |v: f64| 20.0 * (vca.gain(v) / vca.gain(0.0)).log10();
    assert!(
        (db(6.1e-3) - (-1.0)).abs() < 1e-3,
        "6.1 mV ⇒ −1 dB, got {:.5} dB",
        db(6.1e-3)
    );
    // dB-linearity: equal control-voltage steps ⇒ equal dB steps (the log-antilog
    // law makes gain multiplicative in equal Vc increments).
    let step = 0.02;
    assert_rel(
        db(2.0 * step) - db(step),
        db(step) - db(0.0),
        "VCA dB-linear steps",
    );
}

#[test]
fn vca_current_is_gain_times_signal() {
    let vca = Vca::new(0.05298, 1.0); // new() ⇒ thd = 0
    for &(vs, vc) in &[(0.1, 0.0), (0.5, 0.1), (-0.3, -0.05)] {
        assert_rel(
            vca.current(vs, vc),
            vca.gain(vc) * vs,
            &format!("VCA I = gain·Vsig @ ({vs},{vc})"),
        );
    }
}

// ── Pentode — Reefman "Derk" structural invariants ──────────────────────
//
// The Derk plate and screen currents share one Koren current Ip0(Vgk,Vg2k):
//   Ip  = Ip0 · F(Vpk),   Ig2 = Ip0 · H(Vpk)
// So Ip/Ig2 = F(Vpk)/H(Vpk) depends ONLY on Vpk — independent of Vgk. That
// exact cancellation is a strong structural check of the shared-Ip0 model.
// Plus the defining pentode physics: plate-current flatness above the knee,
// monotonic grid control, and grid cutoff.

#[test]
fn pentode_screen_plate_ratio_is_vgk_independent() {
    // Ip/Ig2 = F(Vpk)/H(Vpk): the shared Ip0(Vgk,Vg2k) cancels exactly, so the
    // ratio must be identical across Vgk at fixed (Vpk, Vg2k) — even into deep
    // cutoff, since the cancellation is algebraic, not magnitude-dependent.
    let p = KorenPentode::el84();
    let ratio = |vgk: f64| p.plate_current(vgk, 250.0, 250.0) / p.screen_current(vgk, 250.0, 250.0);
    let r0 = ratio(-5.0);
    for &vgk in &[-8.0, -12.0, -40.0] {
        assert_rel(
            ratio(vgk),
            r0,
            &format!("Ip/Ig2 Vgk-independent @ Vgk={vgk}"),
        );
    }
    // …but it DOES vary with Vpk (F/H are functions of Vpk) — the model is not
    // trivially constant.
    let r_150 = p.plate_current(-7.0, 150.0, 300.0) / p.screen_current(-7.0, 150.0, 300.0);
    let r_300 = p.plate_current(-7.0, 300.0, 300.0) / p.screen_current(-7.0, 300.0, 300.0);
    assert!((r_150 - r_300).abs() > 0.1, "Ip/Ig2 must depend on Vpk");
}

#[test]
fn pentode_defining_physics() {
    let p = KorenPentode::el84();
    let ip = |vgk: f64, vpk: f64| p.plate_current(vgk, vpk, 300.0);

    // Plate-current flatness: doubling Vpk (150→300 V) changes Ip by <15% — the
    // defining pentode trait (high plate resistance), unlike a triode.
    let flat = ip(-7.0, 300.0) / ip(-7.0, 150.0);
    assert!(
        (1.0..1.15).contains(&flat),
        "pentode plate flatness: Ip(300)/Ip(150) = {flat:.3}"
    );

    // Monotonic grid control: less-negative Vgk ⇒ more plate current.
    assert!(
        ip(-5.0, 250.0) > ip(-8.0, 250.0),
        "grid control monotonic (−5 > −8)"
    );
    assert!(
        ip(-8.0, 250.0) > ip(-12.0, 250.0),
        "grid control monotonic (−8 > −12)"
    );

    // Grid cutoff: deep negative Vgk drives Ip to a negligible fraction.
    assert!(
        ip(-40.0, 250.0) < 1e-6 * ip(-5.0, 250.0),
        "grid cutoff at Vgk = −40 V"
    );
}
