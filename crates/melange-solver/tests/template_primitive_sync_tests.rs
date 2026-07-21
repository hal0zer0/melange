//! Template <-> primitives/devices synchronization tests.
//!
//! The self-contained Rust templates (`spice_limiting.rs.tera`,
//! `device_diode.rs.tera`, `device_bjt.rs.tera`, `device_jfet.rs.tera`,
//! `device_mosfet.rs.tera`, `device_vca.rs.tera`, `device_tube.rs.tera`) are
//! duplicates of canonical code in `melange-primitives` (pnjlim/fetlim) and
//! `melange-devices` (DiodeShockley/DiodeWithRs, BjtEbersMoll/BjtGummelPoon,
//! Jfet, Mosfet, Vca, KorenTriode, KorenPentode). The DC operating point is
//! computed with the devices-crate models while the generated transient
//! runtime uses the template copies — any drift between the two produces the
//! "DC OP vs runtime bias divergence" failure class (the July 2026 triode ×2
//! regression lived in exactly this gap for months). Every duplicated device
//! family is pinned here over dense grids so drift fails loudly.
//!
//! Mechanisms:
//! - Templates with no tera tags (`spice_limiting`, `device_diode`,
//!   `device_bjt`, `device_jfet`, `device_mosfet`, `device_vca`) are pure
//!   Rust and are `include!`d directly into the `tpl` module.
//! - `device_tube.rs.tera` contains whole-line `{% if ... %}` section gates,
//!   so it cannot be `include!`d. Instead the tube tests strip the tag lines
//!   (enabling every section), append a probe `main()`, compile the result
//!   with `rustc` at test time (same pattern as `cross_validation_tests.rs`),
//!   and compare the printed values against the devices crate. Probe inputs
//!   are printed by the harness in round-trippable `{:.17e}` form and parsed
//!   back, so the two sides always evaluate identical f64 inputs.
//!
//! `fast_exp` / `fast_ln` are provided as the precise clamped forms (the
//! generated code's `--cfg melange_precise_exp` path), so template-vs-devices
//! comparisons are exact up to f64 rounding / operation-order ULPs.
//!
//! Honest-test notes — duplicated-math surfaces NOT pinned here, and why:
//! - `bjt_with_parasitics` (template) and `dc_op.rs::bjt_with_parasitics_dc`
//!   have no devices-crate canonical twin: both are inner-NR wrappers whose
//!   fixed point is defined by the (pinned) intrinsic `bjt_evaluate`. They
//!   use different damping/iteration schedules by design. Pinned only via
//!   the RB=RC=RE=0 exact-reduction test below.
//! - Triode RGI helpers (`tube_solve_vgk_int` / `*_with_rgi`): RGI exists
//!   only in the template/emitters — the devices crate KorenTriode has no
//!   grid-stopper solve. Pinned only at RGI=0 (exact reduction to the base
//!   triode path, which IS pinned).
//! - `jfet_jacobian_with_rd_rs` / `mosfet_jacobian_with_rd_rs`: the ohmic
//!   closed-form correction has no devices-crate twin; pinned only at
//!   RD=RS=0 (exact reduction) plus the pinned underlying Jacobian.
//! - `fast_exp` / `fast_ln` polynomial approximations themselves (the
//!   non-precise cfg) are an accuracy question, not a twin-sync question,
//!   and are not compared against libm here.
//! - JFET gate current: the template's `jfet_ig` is identically 0 and
//!   `dc_op.rs:280` deliberately matches (Ig ≡ 0). The devices crate's
//!   `Jfet::gate_current` diode-leakage helper is NOT consumed by the solver
//!   and is intentionally not the twin.

use std::process::Command;
use std::sync::OnceLock;

use melange_devices::bjt::{BjtEbersMoll, BjtGummelPoon, BjtPolarity};
use melange_devices::diode::{DiodeShockley, DiodeWithRs};
use melange_devices::jfet::{Jfet, JfetChannel};
use melange_devices::tube::{KorenPentode, KorenTriode, ScreenForm};
use melange_devices::{Mosfet, MosfetChannelType};
use melange_devices::{NonlinearDevice, Vca};
use melange_primitives::VT_ROOM;

/// Template code compiled as-is. `fast_exp`/`fast_ln` are provided as the
/// precise clamped forms (the generated code's `--cfg melange_precise_exp`
/// path), so template-vs-devices comparisons are exact up to f64 rounding.
#[allow(dead_code, clippy::too_many_arguments)]
mod tpl {
    #[inline(always)]
    fn fast_exp(x: f64) -> f64 {
        x.clamp(-40.0, 40.0).exp()
    }
    #[inline(always)]
    fn fast_ln(x: f64) -> f64 {
        x.ln()
    }

    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/templates/rust/spice_limiting.rs.tera"
    ));
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/templates/rust/device_diode.rs.tera"
    ));
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/templates/rust/device_bjt.rs.tera"
    ));
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/templates/rust/device_jfet.rs.tera"
    ));
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/templates/rust/device_mosfet.rs.tera"
    ));
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/templates/rust/device_vca.rs.tera"
    ));

    // Public shims (template items are private to this module).
    pub fn fetlim_tpl(vnew: f64, vold: f64, vto: f64) -> f64 {
        fetlim(vnew, vold, vto)
    }
    pub fn pnjlim_tpl(vnew: f64, vold: f64, vt: f64, vcrit: f64) -> f64 {
        pnjlim(vnew, vold, vt, vcrit)
    }
    pub fn diode_current_tpl(v_d: f64, is: f64, n_vt: f64) -> f64 {
        diode_current(v_d, is, n_vt)
    }
    pub fn diode_conductance_tpl(v_d: f64, is: f64, n_vt: f64) -> f64 {
        diode_conductance(v_d, is, n_vt)
    }
    pub fn diode_current_with_rs_tpl(v_d: f64, is: f64, n_vt: f64, rs: f64) -> f64 {
        diode_current_with_rs(v_d, is, n_vt, rs)
    }
    pub fn diode_conductance_with_rs_tpl(v_d: f64, is: f64, n_vt: f64, rs: f64) -> f64 {
        diode_conductance_with_rs(v_d, is, n_vt, rs)
    }
    #[allow(clippy::too_many_arguments)]
    pub fn bjt_evaluate_tpl(
        vbe: f64,
        vbc: f64,
        is: f64,
        vt: f64,
        nf: f64,
        nr: f64,
        beta_f: f64,
        beta_r: f64,
        sign: f64,
        use_gp: bool,
        vaf: f64,
        var: f64,
        ikf: f64,
        ikr: f64,
        ise: f64,
        ne: f64,
        isc: f64,
        nc: f64,
    ) -> (f64, f64, [f64; 4]) {
        bjt_evaluate(
            vbe, vbc, is, vt, nf, nr, beta_f, beta_r, sign, use_gp, vaf, var, ikf, ikr, ise, ne,
            isc, nc,
        )
    }
    #[allow(clippy::too_many_arguments)]
    pub fn bjt_with_parasitics_tpl(
        vbe_ext: f64,
        vbc_ext: f64,
        is: f64,
        vt: f64,
        nf: f64,
        nr: f64,
        beta_f: f64,
        beta_r: f64,
        sign: f64,
        use_gp: bool,
        vaf: f64,
        var: f64,
        ikf: f64,
        ikr: f64,
        ise: f64,
        ne: f64,
        isc: f64,
        nc: f64,
        rb: f64,
        rc: f64,
        re: f64,
    ) -> (f64, f64, [f64; 4]) {
        bjt_with_parasitics(
            vbe_ext, vbc_ext, is, vt, nf, nr, beta_f, beta_r, sign, use_gp, vaf, var, ikf, ikr,
            ise, ne, isc, nc, rb, rc, re,
        )
    }
    pub fn jfet_id_tpl(vgs: f64, vds: f64, idss: f64, vp: f64, lambda: f64, sign: f64) -> f64 {
        jfet_id(vgs, vds, idss, vp, lambda, sign)
    }
    pub fn jfet_ig_tpl(vgs: f64, sign: f64) -> f64 {
        jfet_ig(vgs, sign)
    }
    pub fn jfet_jacobian_tpl(
        vgs: f64,
        vds: f64,
        idss: f64,
        vp: f64,
        lambda: f64,
        sign: f64,
    ) -> [f64; 4] {
        jfet_jacobian(vgs, vds, idss, vp, lambda, sign)
    }
    #[allow(clippy::too_many_arguments)]
    pub fn jfet_jacobian_with_rd_rs_tpl(
        vgs: f64,
        vds: f64,
        idss: f64,
        vp: f64,
        lambda: f64,
        sign: f64,
        rd: f64,
        rs: f64,
    ) -> [f64; 4] {
        jfet_jacobian_with_rd_rs(vgs, vds, idss, vp, lambda, sign, rd, rs)
    }
    pub fn mosfet_id_tpl(vgs: f64, vds: f64, kp: f64, vt: f64, lambda: f64, sign: f64) -> f64 {
        mosfet_id(vgs, vds, kp, vt, lambda, sign)
    }
    pub fn mosfet_jacobian_tpl(
        vgs: f64,
        vds: f64,
        kp: f64,
        vt: f64,
        lambda: f64,
        sign: f64,
    ) -> [f64; 4] {
        mosfet_jacobian(vgs, vds, kp, vt, lambda, sign)
    }
    #[allow(clippy::too_many_arguments)]
    pub fn mosfet_jacobian_with_rd_rs_tpl(
        vgs: f64,
        vds: f64,
        kp: f64,
        vt: f64,
        lambda: f64,
        sign: f64,
        rd: f64,
        rs: f64,
    ) -> [f64; 4] {
        mosfet_jacobian_with_rd_rs(vgs, vds, kp, vt, lambda, sign, rd, rs)
    }
    pub fn vca_current_tpl(v_sig: f64, v_ctrl: f64, g0: f64, vscale: f64, thd: f64) -> f64 {
        vca_current(v_sig, v_ctrl, g0, vscale, thd)
    }
    pub fn vca_jacobian_tpl(v_sig: f64, v_ctrl: f64, g0: f64, vscale: f64, thd: f64) -> [f64; 4] {
        vca_jacobian(v_sig, v_ctrl, g0, vscale, thd)
    }
}

/// Relative + absolute-floor closeness check.
///
/// For the `include!`d families the op sequences are identical up to
/// association order (e.g. devices' precomputed `inv_beta_f` vs the
/// template's `is / beta_f`), so `rel = 1e-9` is generous while still
/// catching any real drift (a dropped factor, swapped column, or changed
/// clamp moves values by many orders of magnitude more than 1e-9).
/// `abs_floor` absorbs sub-numerical guard-region differences (documented at
/// each call site) that sit far below any solver tolerance.
#[track_caller]
fn assert_close(label: &str, a: f64, b: f64, rel: f64, abs_floor: f64) {
    let tol = rel * a.abs().max(b.abs()) + abs_floor;
    assert!(
        (a - b).abs() <= tol,
        "{label}: template={a:.17e} canonical={b:.17e} |diff|={:.3e} tol={:.3e}",
        (a - b).abs(),
        tol
    );
}

// ============================================================================
// pnjlim / fetlim (pre-existing pins)
// ============================================================================

/// Template fetlim must be bit-identical to melange_primitives::fetlim over a
/// dense grid covering all branches (on/off x increasing/decreasing, middle
/// region, both signs of vto).
#[test]
fn template_fetlim_bit_identical_to_primitives() {
    let mut checked = 0u32;
    for &vto in &[-4.0, -2.0, 0.0, 0.7, 2.0] {
        let mut vold = -25.0;
        while vold <= 25.0 {
            let mut vnew = -25.0;
            while vnew <= 25.0 {
                let a = tpl::fetlim_tpl(vnew, vold, vto);
                let b = melange_primitives::fetlim(vnew, vold, vto);
                assert!(
                    a.to_bits() == b.to_bits(),
                    "fetlim drift at (vnew={}, vold={}, vto={}): template={}, primitives={}",
                    vnew,
                    vold,
                    vto,
                    a,
                    b
                );
                checked += 1;
                vnew += 0.73;
            }
            vold += 0.91;
        }
    }
    assert!(checked > 10_000, "grid too sparse: {}", checked);
}

/// Template pnjlim must be bit-identical to melange_primitives::pnjlim.
#[test]
fn template_pnjlim_bit_identical_to_primitives() {
    let vt = 0.025851991;
    let vcrit = melange_primitives::pn_vcrit(vt, 1e-14);
    let mut v = -5.0;
    while v <= 5.0 {
        let mut vold = -5.0;
        while vold <= 5.0 {
            let a = tpl::pnjlim_tpl(v, vold, vt, vcrit);
            let b = melange_primitives::pnjlim(v, vold, vt, vcrit);
            assert!(
                a.to_bits() == b.to_bits(),
                "pnjlim drift at (vnew={}, vold={}): template={}, primitives={}",
                v,
                vold,
                a,
                b
            );
            vold += 0.137;
        }
        v += 0.119;
    }
}

/// Pin template fetlim against hand-evaluated SPICE3f5 DEVfetlim values at
/// points covering all four quadrants (mirrors the pin test in
/// melange-primitives/src/nr.rs — see devsup.c lines ~93-150).
#[test]
fn template_fetlim_matches_spice3f5_devfetlim() {
    let cases: [(f64, f64, f64, f64); 8] = [
        // on/far-on/increasing, clamped: vtsthi = 10 -> 4 + 10 = 14
        (20.0, 4.0, 0.0, 14.0),
        // on/far-on/decreasing, vnew < vtox -> max(vnew, vto+2) = 2
        (1.0, 10.0, 0.0, 2.0),
        // middle, decreasing -> max(vnew, vto-0.5)
        (-3.0, 1.0, 0.0, -0.5),
        // middle, increasing -> min(vnew, vto+4)
        (8.0, 1.0, 0.0, 4.0),
        // off/decreasing, clamped with vtsthi = 6 -> -2 - 6 = -8
        // (deviant pre-fix code used vtstlo and returned -7)
        (-20.0, -2.0, 0.0, -8.0),
        // off/decreasing, -delv = 5.5 <= vtsthi = 6 -> unclamped
        (-7.5, -2.0, 0.0, -7.5),
        // off/increasing, vnew > vto + 0.5 -> vto + 0.5
        (3.0, -2.0, 0.0, 0.5),
        // nonzero vto, off/decreasing: vtsthi = 8 -> -5 - 8 = -13
        (-30.0, -5.0, -2.0, -13.0),
    ];
    for &(vnew, vold, vto, expected) in &cases {
        let got = tpl::fetlim_tpl(vnew, vold, vto);
        assert!(
            (got - expected).abs() < 1e-12,
            "template fetlim({}, {}, {}) = {}, DEVfetlim gives {}",
            vnew,
            vold,
            vto,
            got,
            expected
        );
    }
}

// ============================================================================
// Diode (pre-existing pins + RS solve)
// ============================================================================

/// Template diode deep-reverse region must linearly extend with a slope equal
/// to the reported conductance (value/derivative consistency), matching the
/// canonical DiodeShockley. The pre-fix template flat-clamped the current
/// below -40*n_vt while returning nonzero conductance.
///
/// Numerical note: the extension slope is (is/n_vt)*e^-40, which for physical
/// is values sits below the ULP of the ~(-is) reverse current, so a local
/// finite-difference check cancels to exactly 0.0 and proves nothing. We use
/// is = 1.0 (pure function test — magnitudes then resolve in f64) and a wide
/// secant baseline so the linear term is thousands of ULPs.
#[test]
fn template_diode_deep_reverse_value_derivative_consistent() {
    let is = 1.0;
    let n_vt = 0.025851991;
    let v_clamp = -40.0 * n_vt; // ~ -1.034 V

    // Slope in the deep-reverse region must equal the reported conductance.
    // g_clamp = (is/n_vt)*e^-40 ~ 1.64e-16; over a 1000 V baseline the linear
    // term is ~1.64e-13, ~750 ULPs of the ~1.0-magnitude currents.
    let (v1, v2) = (-2.0, -1002.0);
    let g = tpl::diode_conductance_tpl(v1, is, n_vt);
    let g2 = tpl::diode_conductance_tpl(v2, is, n_vt);
    assert!(
        (g - g2).abs() <= g.abs() * 1e-12,
        "deep-reverse conductance must be constant: g({})={:.6e} g({})={:.6e}",
        v1,
        g,
        v2,
        g2
    );
    let i1 = tpl::diode_current_tpl(v1, is, n_vt);
    let i2 = tpl::diode_current_tpl(v2, is, n_vt);
    let secant = (i1 - i2) / (v1 - v2);
    assert!(
        (secant - g).abs() <= g.abs() * 0.05,
        "deep-reverse dI/dV must match conductance: secant={:.6e} g={:.6e} \
         (pre-fix flat clamp gives secant=0 with nonzero g)",
        secant,
        g
    );

    // Strictly decreasing, not flat.
    assert!(
        i2 < i1,
        "deep-reverse current must follow the linear extension: I({})={:.17e} I({})={:.17e}",
        v1,
        i1,
        v2,
        i2
    );

    // Continuity at the clamp boundary (linear extension anchored at -40*n_vt).
    let eps = 1e-9;
    let below = tpl::diode_current_tpl(v_clamp - eps, is, n_vt);
    let above = tpl::diode_current_tpl(v_clamp + eps, is, n_vt);
    assert!(
        (above - below).abs() < 1e-12,
        "current must be continuous at the clamp boundary: below={:.17e} above={:.17e}",
        below,
        above
    );
}

/// Template diode current/conductance must match the canonical
/// melange_devices::DiodeShockley across forward, reverse, and both extension
/// regions (template compiled with precise exp, so agreement is tight).
#[test]
fn template_diode_matches_devices_crate() {
    for &(is, n) in &[(1e-12_f64, 1.5_f64), (2.68e-14, 1.07), (1e-30, 2.0)] {
        let d = DiodeShockley::new_room_temp(is, n);
        let n_vt = n * 0.025851991;
        let mut v = -20.0;
        while v <= 5.0 {
            let i_t = tpl::diode_current_tpl(v, is, n_vt);
            let i_d = d.current_at(v);
            let g_t = tpl::diode_conductance_tpl(v, is, n_vt);
            let g_d = d.conductance_at(v);
            assert!(
                (i_t - i_d).abs() <= i_d.abs() * 1e-9 + 1e-24,
                "diode current drift at v={} (is={:.0e}, n={}): tpl={:.9e} dev={:.9e}",
                v,
                is,
                n,
                i_t,
                i_d
            );
            assert!(
                (g_t - g_d).abs() <= g_d.abs() * 1e-9 + 1e-24,
                "diode conductance drift at v={} (is={:.0e}, n={}): tpl={:.9e} dev={:.9e}",
                v,
                is,
                n,
                g_t,
                g_d
            );
            v += 0.037;
        }
    }
}

/// Template diode-with-RS solve (`diode_solve_vj` NR: vcrit seed + pnjlim
/// compressed steps, 32 iters) must match the canonical
/// melange_devices::DiodeWithRs across sub-knee, heavy-forward-drive, and
/// wide-bandgap (IS=1e-30, Vf ≈ 3 V) regimes. This is the RS rewrite that
/// chunk-5 M3 flagged as touching 8 shipped circuits — the template and the
/// DC-OP path (which uses DiodeWithRs) must agree or the baked bias diverges
/// from the runtime clipping curve.
#[test]
fn template_diode_rs_solve_matches_devices_crate() {
    let cases: &[(f64, f64, f64)] = &[
        // (is, n, rs)
        (2.52e-9, 1.752, 7.0), // 1N4148-class with typical RS
        (1e-12, 1.5, 100.0),   // large RS: heavy g_d*RS product
        (2.68e-14, 1.07, 0.5), // small RS
        (1e-30, 2.0, 10.0),    // wide-bandgap: knee above the ±40 clamp
    ];
    let mut conducting = 0u32;
    for &(is, n, rs) in cases {
        let d = DiodeWithRs::new(DiodeShockley::new_room_temp(is, n), rs);
        let n_vt = n * VT_ROOM;
        let mut v = -10.0;
        while v <= 8.0 {
            let i_t = tpl::diode_current_with_rs_tpl(v, is, n_vt, rs);
            let i_d = d.current_at(v);
            let g_t = tpl::diode_conductance_with_rs_tpl(v, is, n_vt, rs);
            let g_d = d.conductance_at(v);
            assert_close(
                &format!("diode_rs current at v={v} (is={is:.0e}, n={n}, rs={rs})"),
                i_t,
                i_d,
                1e-9,
                1e-24,
            );
            assert_close(
                &format!("diode_rs conductance at v={v} (is={is:.0e}, n={n}, rs={rs})"),
                g_t,
                g_d,
                1e-9,
                1e-24,
            );
            if i_d > 1e-6 {
                conducting += 1;
            }
            v += 0.093;
        }
    }
    // Grid honesty: the forward-conduction region (where the RS solve
    // actually iterates) must be well represented.
    assert!(
        conducting > 100,
        "RS-diode grid barely conducts ({} points) — solve loop untested",
        conducting
    );
}

// ============================================================================
// BJT (Ebers-Moll + Gummel-Poon incl. q2/qb)
// ============================================================================

/// Template `bjt_evaluate` in Ebers-Moll mode must match the canonical
/// melange_devices::BjtEbersMoll (currents + full 2x2 Jacobian), including
/// ISE/NE + ISC/NC leakage terms and PNP polarity.
///
/// abs_floor 1e-21 absorbs association-order ULPs (devices precomputes
/// 1/beta; the template divides) on femtoamp-scale terms.
#[test]
fn template_bjt_ebers_moll_matches_devices_crate() {
    // (is, beta_f, beta_r, nf, nr, ise, ne, isc, nc, polarity)
    #[allow(clippy::type_complexity)]
    let variants: &[(f64, f64, f64, f64, f64, f64, f64, f64, f64, BjtPolarity)] = &[
        (
            1.26e-14,
            200.0,
            3.0,
            1.0,
            1.0,
            0.0,
            1.5,
            0.0,
            1.0,
            BjtPolarity::Npn,
        ),
        (
            1e-14,
            150.0,
            5.0,
            1.1,
            1.05,
            1e-12,
            2.0,
            1e-13,
            1.5,
            BjtPolarity::Npn,
        ),
        (
            2.0e-14,
            180.0,
            4.0,
            1.0,
            1.0,
            5e-13,
            1.8,
            0.0,
            1.0,
            BjtPolarity::Pnp,
        ),
    ];
    for &(is, bf, br, nf, nr, ise, ne, isc, nc, pol) in variants {
        let dev = BjtEbersMoll::new(is, VT_ROOM, bf, br, pol)
            .with_nf(nf)
            .with_nr(nr)
            .with_leakage(ise, ne, isc, nc);
        let sign = dev.sign();
        let mut vbe = -3.0;
        while vbe <= 2.0 {
            let mut vbc = -3.0;
            while vbc <= 2.0 {
                let (ic_t, ib_t, jac_t) = tpl::bjt_evaluate_tpl(
                    vbe, vbc, is, VT_ROOM, nf, nr, bf, br, sign, false, 100.0, 10.0, 0.3, 0.006,
                    ise, ne, isc, nc,
                );
                let ctx = format!("EM (pol={pol:?}, nf={nf}) at vbe={vbe:.3} vbc={vbc:.3}");
                assert_close(
                    &format!("{ctx} Ic"),
                    ic_t,
                    dev.collector_current(vbe, vbc),
                    1e-9,
                    1e-21,
                );
                assert_close(
                    &format!("{ctx} Ib"),
                    ib_t,
                    dev.base_current(vbe, vbc),
                    1e-9,
                    1e-21,
                );
                let (dic_dvbe, dic_dvbc) = dev.collector_jacobian(vbe, vbc);
                assert_close(&format!("{ctx} dIc/dVbe"), jac_t[0], dic_dvbe, 1e-9, 1e-21);
                assert_close(&format!("{ctx} dIc/dVbc"), jac_t[1], dic_dvbc, 1e-9, 1e-21);
                assert_close(
                    &format!("{ctx} dIb/dVbe"),
                    jac_t[2],
                    dev.base_current_jacobian_dvbe(vbe, vbc),
                    1e-9,
                    1e-21,
                );
                assert_close(
                    &format!("{ctx} dIb/dVbc"),
                    jac_t[3],
                    dev.base_current_jacobian_dvbc(vbe, vbc),
                    1e-9,
                    1e-21,
                );
                vbc += 0.155;
            }
            vbe += 0.155;
        }
    }
}

/// Template `bjt_evaluate` in Gummel-Poon mode must match the canonical
/// melange_devices::BjtGummelPoon — the q1 Early clamp, q2 = cbe/IKF +
/// cbc/IKR (NF·VT / NR·VT exponentials, ngspice bjtload.c:571), the
/// qb = q1·(1+D)/2 discriminant chain, the quotient-rule Ic Jacobian, and
/// Ib NOT divided by qb. This is the exact math [bjt_gp_ngspice_parity]
/// aligned to ngspice — a drifted twin here re-splits DC OP from runtime
/// for every GP BJT with NF≠1 or a finite IKF.
#[test]
fn template_bjt_gummel_poon_matches_devices_crate() {
    // (is, bf, br, nf, nr, vaf, var, ikf, ikr, ise, ne, isc, nc, polarity)
    #[allow(clippy::type_complexity)]
    let variants: &[(
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        f64,
        BjtPolarity,
    )] = &[
        // 2N2222A-class, NF≠1 to keep the NF·VT-vs-VT distinction live,
        // small IKF so the high-injection knee is inside the grid.
        (
            1.26e-14,
            200.0,
            3.0,
            1.1,
            1.05,
            100.0,
            10.0,
            0.014,
            0.006,
            0.0,
            1.5,
            0.0,
            1.0,
            BjtPolarity::Npn,
        ),
        // PNP with leakage diodes active.
        (
            2.0e-14,
            180.0,
            4.0,
            1.0,
            1.0,
            80.0,
            15.0,
            0.1,
            0.02,
            1e-12,
            2.0,
            1e-13,
            1.5,
            BjtPolarity::Pnp,
        ),
    ];
    for &(is, bf, br, nf, nr, vaf, var, ikf, ikr, ise, ne, isc, nc, pol) in variants {
        let em = BjtEbersMoll::new(is, VT_ROOM, bf, br, pol)
            .with_nf(nf)
            .with_nr(nr)
            .with_leakage(ise, ne, isc, nc);
        let sign = em.sign();
        let gp = BjtGummelPoon::new(em, vaf, var, ikf, ikr);
        let mut vbe = -3.0;
        while vbe <= 2.0 {
            let mut vbc = -3.0;
            while vbc <= 2.0 {
                let (ic_t, ib_t, jac_t) = tpl::bjt_evaluate_tpl(
                    vbe, vbc, is, VT_ROOM, nf, nr, bf, br, sign, true, vaf, var, ikf, ikr, ise, ne,
                    isc, nc,
                );
                let ctx = format!("GP (pol={pol:?}, ikf={ikf}) at vbe={vbe:.3} vbc={vbc:.3}");
                assert_close(
                    &format!("{ctx} Ic"),
                    ic_t,
                    gp.collector_current(vbe, vbc),
                    1e-9,
                    1e-21,
                );
                assert_close(
                    &format!("{ctx} Ib"),
                    ib_t,
                    gp.base_current(vbe, vbc),
                    1e-9,
                    1e-21,
                );
                let jac_ic = NonlinearDevice::<2>::jacobian(&gp, &[vbe, vbc]);
                assert_close(&format!("{ctx} dIc/dVbe"), jac_t[0], jac_ic[0], 1e-9, 1e-21);
                assert_close(&format!("{ctx} dIc/dVbc"), jac_t[1], jac_ic[1], 1e-9, 1e-21);
                // GP Ib Jacobian is pure Ebers-Moll (gpi/gmu not qb-modulated).
                assert_close(
                    &format!("{ctx} dIb/dVbe"),
                    jac_t[2],
                    gp.base.base_current_jacobian_dvbe(vbe, vbc),
                    1e-9,
                    1e-21,
                );
                assert_close(
                    &format!("{ctx} dIb/dVbc"),
                    jac_t[3],
                    gp.base.base_current_jacobian_dvbc(vbe, vbc),
                    1e-9,
                    1e-21,
                );
                vbc += 0.155;
            }
            vbe += 0.155;
        }
    }

    // Grid honesty: the high-injection (q2/qb) mechanism must actually be
    // exercised — GP and EM collector currents must differ visibly somewhere
    // on the grid, or this test pins nothing GP-specific.
    let (is, bf, br, nf, nr, vaf, var, ikf, ikr) =
        (1.26e-14, 200.0, 3.0, 1.1, 1.05, 100.0, 10.0, 0.014, 0.006);
    let (ic_gp, _, _) = tpl::bjt_evaluate_tpl(
        0.8, -1.0, is, VT_ROOM, nf, nr, bf, br, 1.0, true, vaf, var, ikf, ikr, 0.0, 1.5, 0.0, 1.0,
    );
    let (ic_em, _, _) = tpl::bjt_evaluate_tpl(
        0.8, -1.0, is, VT_ROOM, nf, nr, bf, br, 1.0, false, vaf, var, ikf, ikr, 0.0, 1.5, 0.0, 1.0,
    );
    assert!(
        (ic_gp - ic_em).abs() > 0.05 * ic_em.abs(),
        "GP high-injection/Early modulation not exercised: ic_gp={ic_gp:.6e} ic_em={ic_em:.6e}"
    );
}

/// `bjt_with_parasitics` has no devices-crate canonical twin (dc_op.rs uses
/// its own damped-Newton `bjt_with_parasitics_dc`; both wrap the pinned
/// `bjt_evaluate` fixed point). The exact reduction RB=RC=RE=0 must be
/// bit-identical to the intrinsic evaluation — the inner residual is zero at
/// the seed, so the loop must exit without perturbing anything.
#[test]
fn template_bjt_with_parasitics_zero_r_reduces_to_intrinsic() {
    for &(vbe, vbc) in &[
        (-1.0, -5.0),
        (0.65, -3.0),
        (0.7, 0.6),
        (0.0, 0.0),
        (0.8, -0.2),
    ] {
        let a = tpl::bjt_with_parasitics_tpl(
            vbe, vbc, 1.26e-14, VT_ROOM, 1.0, 1.0, 200.0, 3.0, 1.0, true, 100.0, 10.0, 0.3, 0.006,
            0.0, 1.5, 0.0, 1.0, 0.0, 0.0, 0.0,
        );
        let b = tpl::bjt_evaluate_tpl(
            vbe, vbc, 1.26e-14, VT_ROOM, 1.0, 1.0, 200.0, 3.0, 1.0, true, 100.0, 10.0, 0.3, 0.006,
            0.0, 1.5, 0.0, 1.0,
        );
        assert!(
            a.0.to_bits() == b.0.to_bits()
                && a.1.to_bits() == b.1.to_bits()
                && a.2
                    .iter()
                    .zip(b.2.iter())
                    .all(|(x, y)| x.to_bits() == y.to_bits()),
            "bjt_with_parasitics(0,0,0) != bjt_evaluate at (vbe={vbe}, vbc={vbc}): {a:?} vs {b:?}"
        );
    }
}

// ============================================================================
// JFET
// ============================================================================

/// Template JFET (Shichman-Hodges with reverse mode + tanh-gated
/// subthreshold) must match the canonical melange_devices::Jfet for both
/// channels across all quadrants: triode, saturation, cutoff/subthreshold,
/// reverse mode (Vds < 0 for N), and the Vds=0 mode boundary.
///
/// abs_floor 1e-24 absorbs the one known sub-numerical divergence: in deep
/// subthreshold (vgst/(2VT) < -40) the template's fast_exp clamps at e^-40
/// while the devices crate evaluates the raw exp — both sides are ≤ 4e-30 A
/// there, below any solver tolerance.
#[test]
fn template_jfet_matches_devices_crate() {
    // (channel, vp, idss, lambda) — vp follows the shared convention:
    // negative as stored for N, positive as stored for P.
    let variants: &[(JfetChannel, f64, f64, f64)] = &[
        (JfetChannel::N, -2.0, 5e-3, 0.01),
        (JfetChannel::N, -0.8, 6e-4, 0.002), // J201-class
        (JfetChannel::P, 2.5, 5e-3, 0.002),
    ];
    for &(channel, vp, idss, lambda) in variants {
        let mut dev = Jfet::new(channel, vp, idss);
        dev.lambda = lambda;
        let sign = match channel {
            JfetChannel::N => 1.0,
            JfetChannel::P => -1.0,
        };
        let (mut fwd, mut rev, mut sub) = (0u32, 0u32, 0u32);
        let mut vgs = -4.0;
        while vgs <= 4.0 {
            let mut vds = -4.0;
            while vds <= 4.0 {
                let id_t = tpl::jfet_id_tpl(vgs, vds, idss, vp, lambda, sign);
                let id_d = dev.drain_current(vgs, vds);
                let ctx = format!("JFET ({channel:?}) at vgs={vgs:.3} vds={vds:.3}");
                assert_close(&format!("{ctx} Id"), id_t, id_d, 1e-9, 1e-24);
                let jac_t = tpl::jfet_jacobian_tpl(vgs, vds, idss, vp, lambda, sign);
                let (gm_d, gds_d) = dev.jacobian_partial(vgs, vds);
                assert_close(&format!("{ctx} dId/dVgs"), jac_t[0], gm_d, 1e-9, 1e-24);
                assert_close(&format!("{ctx} dId/dVds"), jac_t[1], gds_d, 1e-9, 1e-24);
                assert_eq!(jac_t[2], 0.0, "{ctx} dIg/dVgs must be 0");
                assert_eq!(jac_t[3], 0.0, "{ctx} dIg/dVds must be 0");
                assert_eq!(
                    tpl::jfet_ig_tpl(vgs, sign),
                    0.0,
                    "{ctx} template Ig must be identically 0 (matches dc_op.rs Ig ≡ 0)"
                );
                // RD=RS=0 exact reduction of the ohmic-corrected Jacobian.
                let jac_rr =
                    tpl::jfet_jacobian_with_rd_rs_tpl(vgs, vds, idss, vp, lambda, sign, 0.0, 0.0);
                assert_eq!(jac_rr[0].to_bits(), jac_t[0].to_bits(), "{ctx} rd/rs=0 gm");
                assert_eq!(jac_rr[1].to_bits(), jac_t[1].to_bits(), "{ctx} rd/rs=0 gds");
                // Region bookkeeping for grid honesty.
                if id_d.abs() > 1e-6 {
                    if sign * vds >= 0.0 {
                        fwd += 1;
                    } else {
                        rev += 1;
                    }
                } else {
                    sub += 1;
                }
                vds += 0.19;
            }
            vgs += 0.19;
        }
        assert!(
            fwd > 100 && rev > 20 && sub > 100,
            "JFET ({channel:?}) grid missed a region: fwd={fwd} rev={rev} sub={sub}"
        );
    }
}

// ============================================================================
// MOSFET
// ============================================================================

/// Template MOSFET (Level 1 with signed VTO, reverse mode, tanh-gated
/// subthreshold) must match the canonical melange_devices::Mosfet across
/// enhancement AND depletion devices of both polarities and all quadrants.
/// Signed-VTO semantics (depletion NMOS conducts at Vgs=0; positive-VTO PMOS
/// means depletion) are exactly the campaign semantics-tightening from
/// chunk1 L1 — a twin drift here would silently re-split them.
#[test]
fn template_mosfet_matches_devices_crate() {
    // (channel, vt, kp, lambda)
    let variants: &[(MosfetChannelType, f64, f64, f64)] = &[
        (MosfetChannelType::N, 2.0, 0.1, 0.01),    // N enhancement
        (MosfetChannelType::N, -2.0, 0.05, 0.005), // N depletion
        (MosfetChannelType::P, -2.0, 0.1, 0.01),   // P enhancement
        (MosfetChannelType::P, 2.0, 0.02, 0.0),    // P depletion
    ];
    for &(channel, vt, kp, lambda) in variants {
        let dev = Mosfet::new(channel, vt, kp, lambda);
        let sign = match channel {
            MosfetChannelType::N => 1.0,
            MosfetChannelType::P => -1.0,
        };
        let (mut on, mut off) = (0u32, 0u32);
        let mut vgs = -6.0;
        while vgs <= 6.0 {
            let mut vds = -6.0;
            while vds <= 6.0 {
                let id_t = tpl::mosfet_id_tpl(vgs, vds, kp, vt, lambda, sign);
                let id_d = dev.drain_current(vgs, vds);
                let ctx = format!("MOSFET ({channel:?}, vt={vt}) at vgs={vgs:.3} vds={vds:.3}");
                assert_close(&format!("{ctx} Id"), id_t, id_d, 1e-9, 1e-24);
                let jac_t = tpl::mosfet_jacobian_tpl(vgs, vds, kp, vt, lambda, sign);
                let (gm_d, gds_d) = dev.jacobian_partial(vgs, vds);
                assert_close(&format!("{ctx} dId/dVgs"), jac_t[0], gm_d, 1e-9, 1e-24);
                assert_close(&format!("{ctx} dId/dVds"), jac_t[1], gds_d, 1e-9, 1e-24);
                assert_eq!(jac_t[2], 0.0, "{ctx} dIg/dVgs must be 0");
                assert_eq!(jac_t[3], 0.0, "{ctx} dIg/dVds must be 0");
                // RD=RS=0 exact reduction of the ohmic-corrected Jacobian.
                let jac_rr =
                    tpl::mosfet_jacobian_with_rd_rs_tpl(vgs, vds, kp, vt, lambda, sign, 0.0, 0.0);
                assert_eq!(jac_rr[0].to_bits(), jac_t[0].to_bits(), "{ctx} rd/rs=0 gm");
                assert_eq!(jac_rr[1].to_bits(), jac_t[1].to_bits(), "{ctx} rd/rs=0 gds");
                if id_d.abs() > 1e-6 {
                    on += 1;
                } else {
                    off += 1;
                }
                vds += 0.31;
            }
            vgs += 0.31;
        }
        assert!(
            on > 100 && off > 100,
            "MOSFET ({channel:?}, vt={vt}) grid missed a region: on={on} off={off}"
        );
    }
    // Signed-VTO teeth: a depletion NMOS must conduct at Vgs = 0.
    let depl = Mosfet::new(MosfetChannelType::N, -2.0, 0.05, 0.005);
    let id_t = tpl::mosfet_id_tpl(0.0, 5.0, 0.05, -2.0, 0.005, 1.0);
    assert!(
        id_t > 1e-3 && depl.drain_current(0.0, 5.0) > 1e-3,
        "depletion NMOS must conduct at Vgs=0 in BOTH copies (tpl={id_t:.3e})"
    );
}

// ============================================================================
// VCA (Blackmer)
// ============================================================================

/// Template VCA (THAT 2180 Blackmer gain law + gain-dependent cubic THD,
/// including the thd_chain Jacobian term the campaign fixed) must match the
/// canonical melange_devices::Vca, with and without THD, into both exp-clamp
/// regions. sus-bus's sidechain CV law rides directly on this math.
#[test]
fn template_vca_matches_devices_crate() {
    // (vscale, g0, thd)
    let variants: &[(f64, f64, f64)] = &[
        (0.05298, 1.0, 0.0),
        (0.05298, 1.0, 0.002),
        (0.00528, 2.0, 0.001),
    ];
    for &(vscale, g0, thd) in variants {
        let dev = Vca::new_with_thd(vscale, g0, thd);
        let mut v_ctrl = -0.5;
        while v_ctrl <= 0.5 {
            let mut v_sig = -2.0;
            while v_sig <= 2.0 {
                let ctx = format!(
                    "VCA (vscale={vscale}, thd={thd}) at vsig={v_sig:.3} vctrl={v_ctrl:.4}"
                );
                let i_t = tpl::vca_current_tpl(v_sig, v_ctrl, g0, vscale, thd);
                assert_close(
                    &format!("{ctx} I"),
                    i_t,
                    dev.current(v_sig, v_ctrl),
                    1e-9,
                    1e-24,
                );
                let jac_t = tpl::vca_jacobian_tpl(v_sig, v_ctrl, g0, vscale, thd);
                let jac_d = dev.jacobian(v_sig, v_ctrl);
                for k in 0..4 {
                    assert_close(&format!("{ctx} jac[{k}]"), jac_t[k], jac_d[k], 1e-9, 1e-24);
                }
                v_sig += 0.13;
            }
            v_ctrl += 0.037;
        }
        // exp-clamp extremes (both copies clamp the exponent at ±40).
        for &v_ctrl in &[-5.0, 5.0] {
            let i_t = tpl::vca_current_tpl(1.0, v_ctrl, g0, vscale, thd);
            assert_close(
                &format!("VCA clamp region vctrl={v_ctrl}"),
                i_t,
                dev.current(1.0, v_ctrl),
                1e-9,
                1e-24,
            );
        }
    }
}

// ============================================================================
// Tube (triode + pentode) — rustc harness over the tera-gated template
// ============================================================================

/// Pentode parameter card used by both the harness source and the
/// devices-crate comparison.
#[derive(Clone, Copy)]
struct PentP {
    mu: f64,
    ex: f64,
    kg1: f64,
    kg2: f64,
    kp: f64,
    kvb: f64,
    alpha_s: f64,
    a_factor: f64,
    beta_factor: f64,
    ig_max: f64,
    vgk_onset: f64,
    mu_b: f64,
    svar: f64,
    ex_b: f64,
}

impl PentP {
    fn to_device(self, form: ScreenForm) -> KorenPentode {
        KorenPentode {
            mu: self.mu,
            ex: self.ex,
            kg1: self.kg1,
            kg2: self.kg2,
            kp: self.kp,
            kvb: self.kvb,
            alpha_s: self.alpha_s,
            a_factor: self.a_factor,
            beta_factor: self.beta_factor,
            ig_max: self.ig_max,
            vgk_onset: self.vgk_onset,
            screen_form: form,
            mu_b: self.mu_b,
            svar: self.svar,
            ex_b: self.ex_b,
        }
    }
}

// Reefman TubeLib.inc / Cohen-Hélie fits (same cards as the devices-crate
// constructors; varimu cards from the catalog).
const P_EL84: PentP = PentP {
    mu: 23.36,
    ex: 1.138,
    kg1: 117.4,
    kg2: 1275.0,
    kp: 152.4,
    kvb: 4015.8,
    alpha_s: 7.66,
    a_factor: 4.344e-4,
    beta_factor: 0.148,
    ig_max: 2e-3,
    vgk_onset: 0.5,
    mu_b: 0.0,
    svar: 0.0,
    ex_b: 0.0,
};
const P_6L6GC: PentP = PentP {
    mu: 9.41,
    ex: 1.306,
    kg1: 446.6,
    kg2: 6672.5,
    kp: 45.2,
    kvb: 3205.1,
    alpha_s: 8.10,
    a_factor: 4.91e-4,
    beta_factor: 0.069,
    ig_max: 10e-3,
    vgk_onset: 0.7,
    mu_b: 0.0,
    svar: 0.0,
    ex_b: 0.0,
};
const P_KT88: PentP = PentP {
    mu: 8.8,
    ex: 1.35,
    kg1: 730.0,
    kg2: 4200.0,
    kp: 32.0,
    kvb: 16.0,
    alpha_s: 0.0,
    a_factor: 0.0,
    beta_factor: 0.0,
    ig_max: 10e-3,
    vgk_onset: 0.7,
    mu_b: 0.0,
    svar: 0.0,
    ex_b: 0.0,
};
const P_6K7: PentP = PentP {
    mu: 15.5,
    ex: 1.573,
    kg1: 1407.7,
    kg2: 8335.8,
    kp: 36.0,
    kvb: 1309.0,
    alpha_s: 4.07,
    a_factor: 1.55e-9,
    beta_factor: 0.15,
    ig_max: 2e-3,
    vgk_onset: 0.5,
    mu_b: 3.4,
    svar: 0.083,
    ex_b: 1.223,
};
const P_EF89: PentP = PentP {
    mu: 25.0,
    ex: 1.418,
    kg1: 328.3,
    kg2: 1199.3,
    kp: 58.8,
    kvb: 1.0,
    alpha_s: 2.07,
    a_factor: 0.0,
    beta_factor: 0.122,
    ig_max: 2e-3,
    vgk_onset: 0.5,
    mu_b: 7.8,
    svar: 0.068,
    ex_b: 0.978,
};

/// Grid loops shared (textually) by every pentode-family harness block.
/// vgk covers deep cutoff through positive grid; vpk covers the negative-probe
/// guard, the knee, and high plate voltage; vg2k covers the ≤0 guard plus two
/// operating screen voltages.
const PENT_LOOP_HEAD: &str = "\
    let mut vgk = -30.0f64;\n\
    while vgk <= 2.0 {\n\
        for &vpk in &[-5.0f64, 0.0, 3.0, 25.0, 120.0, 300.0] {\n\
            for &vg2k in &[-1.0f64, 50.0, 250.0] {\n";
const PENT_LOOP_TAIL: &str = "\
            }\n\
        }\n\
        vgk += 2.3;\n\
    }\n";

/// Emit one Derk/DerkE-signature pentode family block (sharp forms).
fn pent_block_sharp(label: &str, fn_suffix: &str, p: &PentP) -> String {
    let PentP {
        mu,
        ex,
        kg1,
        kg2,
        kp,
        kvb,
        alpha_s,
        a_factor,
        beta_factor,
        ig_max,
        vgk_onset,
        ..
    } = *p;
    format!(
        "{{\n{PENT_LOOP_HEAD}\
        let ip = tube_ip_{fn_suffix}(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {a_factor:?}, {beta_factor:?});\n\
        let ig2 = tube_is_{fn_suffix}(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {beta_factor:?});\n\
        let j = tube_jacobian_{fn_suffix}(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {a_factor:?}, {beta_factor:?}, {ig_max:?}, {vgk_onset:?});\n\
        let (eip, eig2, eig1, ej) = tube_evaluate_{fn_suffix}(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {a_factor:?}, {beta_factor:?}, {ig_max:?}, {vgk_onset:?});\n\
        print!(\"{label} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}}\", vgk, vpk, vg2k, ip, ig2);\n\
        for x in j {{ print!(\" {{:.17e}}\", x); }}\n\
        print!(\" {{:.17e}} {{:.17e}} {{:.17e}}\", eip, eig2, eig1);\n\
        for x in ej {{ print!(\" {{:.17e}}\", x); }}\n\
        println!();\n\
{PENT_LOOP_TAIL}}}\n"
    )
}

/// Emit one variable-mu pentode family block (extra mu_b/svar/ex_b args).
fn pent_block_varimu(label: &str, fn_suffix: &str, p: &PentP) -> String {
    let PentP {
        mu,
        ex,
        kg1,
        kg2,
        kp,
        kvb,
        alpha_s,
        a_factor,
        beta_factor,
        ig_max,
        vgk_onset,
        mu_b,
        svar,
        ex_b,
    } = *p;
    format!(
        "{{\n{PENT_LOOP_HEAD}\
        let ip = tube_ip_{fn_suffix}(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {a_factor:?}, {beta_factor:?}, {mu_b:?}, {svar:?}, {ex_b:?});\n\
        let ig2 = tube_is_{fn_suffix}(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {beta_factor:?}, {mu_b:?}, {svar:?}, {ex_b:?});\n\
        let j = tube_jacobian_{fn_suffix}(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {a_factor:?}, {beta_factor:?}, {ig_max:?}, {vgk_onset:?}, {mu_b:?}, {svar:?}, {ex_b:?});\n\
        let (eip, eig2, eig1, ej) = tube_evaluate_{fn_suffix}(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {a_factor:?}, {beta_factor:?}, {ig_max:?}, {vgk_onset:?}, {mu_b:?}, {svar:?}, {ex_b:?});\n\
        print!(\"{label} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}}\", vgk, vpk, vg2k, ip, ig2);\n\
        for x in j {{ print!(\" {{:.17e}}\", x); }}\n\
        print!(\" {{:.17e}} {{:.17e}} {{:.17e}}\", eip, eig2, eig1);\n\
        for x in ej {{ print!(\" {{:.17e}}\", x); }}\n\
        println!();\n\
{PENT_LOOP_TAIL}}}\n"
    )
}

/// Emit the Classical block (6-parameter signature).
fn pent_block_classical(label: &str, p: &PentP) -> String {
    let PentP {
        mu,
        ex,
        kg1,
        kg2,
        kp,
        kvb,
        ig_max,
        vgk_onset,
        ..
    } = *p;
    format!(
        "{{\n{PENT_LOOP_HEAD}\
        let ip = tube_ip_pentode_classical(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?});\n\
        let ig2 = tube_is_pentode_classical(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?});\n\
        let j = tube_jacobian_pentode_classical(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {ig_max:?}, {vgk_onset:?});\n\
        let (eip, eig2, eig1, ej) = tube_evaluate_pentode_classical(vgk, vpk, vg2k, {mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {ig_max:?}, {vgk_onset:?});\n\
        print!(\"{label} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}}\", vgk, vpk, vg2k, ip, ig2);\n\
        for x in j {{ print!(\" {{:.17e}}\", x); }}\n\
        print!(\" {{:.17e}} {{:.17e}} {{:.17e}}\", eip, eig2, eig1);\n\
        for x in ej {{ print!(\" {{:.17e}}\", x); }}\n\
        println!();\n\
{PENT_LOOP_TAIL}}}\n"
    )
}

/// Emit one grid-off wrapper block (2x2 Jacobian, frozen Vg2k).
fn pent_block_grid_off(label: &str, fn_suffix: &str, p: &PentP, classical: bool) -> String {
    let PentP {
        mu,
        ex,
        kg1,
        kg2,
        kp,
        kvb,
        alpha_s,
        a_factor,
        beta_factor,
        ig_max,
        vgk_onset,
        ..
    } = *p;
    let (ip_args, is_args, j_args) = if classical {
        (
            format!("{mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}"),
            format!("{mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}"),
            format!("{mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {ig_max:?}, {vgk_onset:?}"),
        )
    } else {
        (
            format!("{mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {a_factor:?}, {beta_factor:?}"),
            format!("{mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {beta_factor:?}"),
            format!("{mu:?}, {ex:?}, {kg1:?}, {kg2:?}, {kp:?}, {kvb:?}, {alpha_s:?}, {a_factor:?}, {beta_factor:?}, {ig_max:?}, {vgk_onset:?}"),
        )
    };
    format!(
        "{{\n\
    let vg2k_frozen = 250.0f64;\n\
    let mut vgk = -30.0f64;\n\
    while vgk <= 2.0 {{\n\
        for &vpk in &[-5.0f64, 0.0, 3.0, 25.0, 120.0, 300.0] {{\n\
            let ip = tube_ip_{fn_suffix}(vgk, vpk, vg2k_frozen, {ip_args});\n\
            let ig2 = tube_is_{fn_suffix}(vgk, vpk, vg2k_frozen, {is_args});\n\
            let j = tube_jacobian_{fn_suffix}(vgk, vpk, vg2k_frozen, {j_args});\n\
            print!(\"{label} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}}\", vgk, vpk, vg2k_frozen, ip, ig2);\n\
            for x in j {{ print!(\" {{:.17e}}\", x); }}\n\
            println!();\n\
        }}\n\
        vgk += 2.3;\n\
    }}\n\
}}\n"
    )
}

/// Build, compile (rustc), and run the tube-template harness once per test
/// process; return the parsed (label, values) rows.
fn tube_harness_rows() -> &'static Vec<(String, Vec<f64>)> {
    static ROWS: OnceLock<Vec<(String, Vec<f64>)>> = OnceLock::new();
    ROWS.get_or_init(|| {
        // Strip the whole-line `{% ... %}` tera section gates, enabling every
        // section (pentode, beam tetrode, variable-mu, classical, grid-off).
        // The tube template contains no inline `{{ ... }}` substitutions.
        let raw = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/templates/rust/device_tube.rs.tera"
        ));
        assert!(
            !raw.contains("{{"),
            "device_tube.rs.tera now contains inline tera substitutions — the \
             line-strip harness in this test can no longer compile it verbatim; \
             extend the harness before shipping the template change"
        );
        let stripped: String = raw
            .lines()
            .filter(|l| {
                let t = l.trim();
                !(t.starts_with("{%") && t.ends_with("%}"))
            })
            .collect::<Vec<_>>()
            .join("\n");

        let mut main_body = String::new();

        // Triode blocks: 12AX7 card, lambda = 0 and lambda > 0 (Early effect),
        // plus the RGI=0 exact-reduction columns.
        for (set, lambda) in [(0u32, 0.0f64), (1u32, 2e-4f64)] {
            let (mu, ex, kg1, kp, kvb, ig_max, vgk_onset) =
                (100.0f64, 1.4f64, 1060.0f64, 600.0f64, 300.0f64, 2e-3f64, 0.5f64);
            main_body.push_str(&format!(
                "{{\n\
    let mut vgk = -6.0f64;\n\
    while vgk <= 2.0 {{\n\
        let mut vpk = -5.0f64;\n\
        while vpk <= 400.0 {{\n\
            let ip = tube_ip(vgk, vpk, {mu:?}, {ex:?}, {kg1:?}, {kp:?}, {kvb:?}, {lambda:?});\n\
            let ig = tube_ig(vgk, {ig_max:?}, {vgk_onset:?});\n\
            let dig = tube_ig_deriv(vgk, {ig_max:?}, {vgk_onset:?});\n\
            let j = tube_jacobian(vgk, vpk, {mu:?}, {ex:?}, {kg1:?}, {kp:?}, {kvb:?}, {ig_max:?}, {vgk_onset:?}, {lambda:?});\n\
            let (eip, eig, ej) = tube_evaluate(vgk, vpk, {mu:?}, {ex:?}, {kg1:?}, {kp:?}, {kvb:?}, {ig_max:?}, {vgk_onset:?}, {lambda:?});\n\
            let (rip, rig, rj) = tube_evaluate_with_rgi(vgk, vpk, {mu:?}, {ex:?}, {kg1:?}, {kp:?}, {kvb:?}, {ig_max:?}, {vgk_onset:?}, {lambda:?}, 0.0);\n\
            print!(\"triode{set} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}} {{:.17e}}\", vgk, vpk, ip, ig, dig);\n\
            for x in j {{ print!(\" {{:.17e}}\", x); }}\n\
            print!(\" {{:.17e}} {{:.17e}}\", eip, eig);\n\
            for x in ej {{ print!(\" {{:.17e}}\", x); }}\n\
            print!(\" {{:.17e}} {{:.17e}}\", rip, rig);\n\
            for x in rj {{ print!(\" {{:.17e}}\", x); }}\n\
            println!();\n\
            vpk += 21.7;\n\
        }}\n\
        vgk += 0.37;\n\
    }}\n\
}}\n"
            ));
        }

        // Pentode families (one block per equation family / screen form).
        main_body.push_str(&pent_block_sharp("pent_rat", "pentode", &P_EL84));
        main_body.push_str(&pent_block_sharp("pent_exp", "beam_tetrode", &P_6L6GC));
        main_body.push_str(&pent_block_classical("pent_cls", &P_KT88));
        main_body.push_str(&pent_block_varimu("pent_vrat", "pentode_v", &P_6K7));
        main_body.push_str(&pent_block_varimu("pent_vexp", "beam_tetrode_v", &P_EF89));

        // Grid-off wrappers (Phase 1b 2D reduction, all three sharp forms).
        main_body.push_str(&pent_block_grid_off("goff_rat", "pentode_grid_off", &P_EL84, false));
        main_body.push_str(&pent_block_grid_off("goff_exp", "beam_tetrode_grid_off", &P_6L6GC, false));
        main_body.push_str(&pent_block_grid_off("goff_cls", "pentode_classical_grid_off", &P_KT88, true));

        let source = format!(
            "#![allow(dead_code, unused_variables, clippy::all)]\n\
             // Precise fast_exp/fast_ln (the generated code's melange_precise_exp path).\n\
             #[inline(always)]\n\
             fn fast_exp(x: f64) -> f64 {{ x.clamp(-40.0, 40.0).exp() }}\n\
             #[inline(always)]\n\
             fn fast_ln(x: f64) -> f64 {{ x.ln() }}\n\
             {stripped}\n\
             fn main() {{\n{main_body}}}\n"
        );

        let tmp_dir = std::env::temp_dir();
        let id = std::process::id();
        let src_path = tmp_dir.join(format!("melange_tube_sync_{id}.rs"));
        let bin_path = tmp_dir.join(format!("melange_tube_sync_{id}"));
        std::fs::write(&src_path, &source).expect("write harness source");

        let compile = Command::new("rustc")
            .arg(&src_path)
            .arg("-o")
            .arg(&bin_path)
            .arg("--edition=2024")
            .arg("-O")
            .output()
            .expect("rustc");
        let _ = std::fs::remove_file(&src_path);
        assert!(
            compile.status.success(),
            "tube template harness failed to compile (template drift or strip \
             failure):\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );

        let run = Command::new(&bin_path).output().expect("run harness");
        let _ = std::fs::remove_file(&bin_path);
        assert!(run.status.success(), "tube harness crashed");

        let rows: Vec<(String, Vec<f64>)> = String::from_utf8_lossy(&run.stdout)
            .lines()
            .filter(|l| !l.trim().is_empty())
            .map(|l| {
                let mut it = l.split_whitespace();
                let label = it.next().expect("row label").to_string();
                let vals: Vec<f64> = it.map(|t| t.parse::<f64>().expect("row value")).collect();
                (label, vals)
            })
            .collect();
        assert!(rows.len() > 1500, "harness produced too few rows: {}", rows.len());
        rows
    })
}

/// Tolerances for harness comparisons: rel 1e-9 for the near-identical op
/// sequences; abs floor 1e-14 absorbs the one known guard-window difference
/// (`tube_evaluate` uses the Jacobian's E1 ≤ 1e-10 deep-cutoff guard for its
/// current output while `plate_current` guards at E1 ≤ 0 — the disagreement
/// window tops out around 4e-17 A for a 12AX7).
const TUBE_REL: f64 = 1e-9;
const TUBE_ABS: f64 = 1e-14;

/// Template triode (Koren, WITH the ×2 PWR+PWRS factor) must match the
/// canonical melange_devices::KorenTriode: plate current, grid current +
/// derivative, full 2x2 Jacobian, the fused `tube_evaluate`, and the RGI=0
/// exact reduction of `tube_evaluate_with_rgi`. This is the precise drift
/// class behind the July 2026 +39 dB series-of-tubes regression (×2 present
/// in the devices crate, absent from the template for months).
///
/// Not pinned here (no canonical twin): RGI > 0 — the grid-stopper inner NR
/// exists only in the template/emitters.
#[test]
fn template_triode_matches_devices_crate() {
    for (set, lambda) in [(0u32, 0.0f64), (1u32, 2e-4f64)] {
        let dev = KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, lambda);
        let label = format!("triode{set}");
        let mut n = 0u32;
        let mut conducting = 0u32;
        for (l, v) in tube_harness_rows().iter().filter(|(l, _)| *l == label) {
            assert_eq!(v.len(), 21, "bad {l} row width");
            let (vgk, vpk) = (v[0], v[1]);
            let ctx = format!("{l} at vgk={vgk:.3} vpk={vpk:.1}");
            let ip_d = dev.plate_current(vgk, vpk);
            let ig_d = dev.grid_current(vgk);
            let dig_d = dev.grid_current_jacobian(vgk);
            let jac_d = NonlinearDevice::<2>::jacobian(&dev, &[vgk, vpk]);
            assert_close(&format!("{ctx} Ip"), v[2], ip_d, TUBE_REL, TUBE_ABS);
            assert_close(&format!("{ctx} Ig"), v[3], ig_d, TUBE_REL, TUBE_ABS);
            assert_close(&format!("{ctx} dIg/dVgk"), v[4], dig_d, TUBE_REL, TUBE_ABS);
            assert_close(
                &format!("{ctx} dIp/dVgk"),
                v[5],
                jac_d[0],
                TUBE_REL,
                TUBE_ABS,
            );
            assert_close(
                &format!("{ctx} dIp/dVpk"),
                v[6],
                jac_d[1],
                TUBE_REL,
                TUBE_ABS,
            );
            assert_close(
                &format!("{ctx} jac dIg/dVgk"),
                v[7],
                dig_d,
                TUBE_REL,
                TUBE_ABS,
            );
            assert_eq!(v[8], 0.0, "{ctx} dIg/dVpk must be 0");
            // Fused evaluate.
            assert_close(&format!("{ctx} eval Ip"), v[9], ip_d, TUBE_REL, TUBE_ABS);
            assert_close(&format!("{ctx} eval Ig"), v[10], ig_d, TUBE_REL, TUBE_ABS);
            assert_close(
                &format!("{ctx} eval dIp/dVgk"),
                v[11],
                jac_d[0],
                TUBE_REL,
                TUBE_ABS,
            );
            assert_close(
                &format!("{ctx} eval dIp/dVpk"),
                v[12],
                jac_d[1],
                TUBE_REL,
                TUBE_ABS,
            );
            assert_close(
                &format!("{ctx} eval dIg/dVgk"),
                v[13],
                dig_d,
                TUBE_REL,
                TUBE_ABS,
            );
            assert_eq!(v[14], 0.0, "{ctx} eval dIg/dVpk must be 0");
            // RGI = 0 exact reduction (inner solve converges at the seed).
            assert_close(&format!("{ctx} rgi0 Ip"), v[15], v[9], 0.0, 0.0);
            assert_close(&format!("{ctx} rgi0 Ig"), v[16], v[10], 0.0, 0.0);
            for k in 0..4 {
                assert_close(
                    &format!("{ctx} rgi0 jac[{k}]"),
                    v[17 + k],
                    v[11 + k],
                    0.0,
                    0.0,
                );
            }
            n += 1;
            if ip_d > 1e-4 {
                conducting += 1;
            }
        }
        assert!(n > 300, "{label}: too few harness rows ({n})");
        assert!(
            conducting > 50,
            "{label}: grid barely conducts ({conducting})"
        );
    }
}

/// Absolute anchor for the Koren ×2: at the RCA typical-operation point
/// (Vgk=−2 V, Vpk=250 V) the 12AX7 card MUST give ~0.95 mA. A missing ×2
/// gives 0.476 mA — this catches the failure mode where BOTH twins drop the
/// factor together (which the relative comparison above cannot see).
#[test]
fn template_triode_koren_x2_absolute_anchor() {
    // Devices-crate side (independent of the harness).
    let dev = KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, 0.0);
    let ip_dev = dev.plate_current(-2.0, 250.0);
    assert!(
        (0.90e-3..1.00e-3).contains(&ip_dev),
        "devices-crate 12AX7 Ip(-2V, 250V) = {:.4} mA — expected ~0.95 mA \
         (0.476 mA means the Koren ×2 was dropped)",
        ip_dev * 1e3
    );
    // Template side: nearest harness grid point to the anchor (vgk=-2.04,
    // vpk=255.6 lands within the loop; use the closest row and a physical
    // window instead of grid-point luck).
    let mut best: Option<(f64, f64, f64)> = None;
    for (_, v) in tube_harness_rows().iter().filter(|(l, _)| l == "triode0") {
        let d = (v[0] + 2.0).abs() + (v[1] - 250.0).abs() / 100.0;
        if best.is_none_or(|(bd, _, _)| d < bd) {
            best = Some((d, v[2], dev.plate_current(v[0], v[1])));
        }
    }
    let (_, ip_tpl, ip_ref) = best.expect("no triode rows");
    assert!(
        (ip_tpl / ip_ref - 1.0).abs() < 1e-9,
        "template 12AX7 anchor-point Ip {ip_tpl:.6e} != devices {ip_ref:.6e}"
    );
    assert!(
        ip_tpl > 0.5e-3,
        "template 12AX7 Ip near the anchor is {:.4} mA — halved-gm signature",
        ip_tpl * 1e3
    );
}

/// Shared checker for the 3D pentode families (26-column rows).
fn check_pentode_family(label: &str, dev: &KorenPentode) {
    let mut n = 0u32;
    let mut conducting = 0u32;
    for (l, v) in tube_harness_rows().iter().filter(|(l, _)| l == label) {
        assert_eq!(v.len(), 26, "bad {l} row width");
        let (vgk, vpk, vg2k) = (v[0], v[1], v[2]);
        let ctx = format!("{l} at vgk={vgk:.3} vpk={vpk:.1} vg2k={vg2k:.1}");
        let ip_d = dev.plate_current(vgk, vpk, vg2k);
        let ig2_d = dev.screen_current(vgk, vpk, vg2k);
        let ig1_d = dev.grid_current(vgk);
        let jac_d = dev.jacobian_3x3(vgk, vpk, vg2k);
        assert_close(&format!("{ctx} Ip"), v[3], ip_d, TUBE_REL, TUBE_ABS);
        assert_close(&format!("{ctx} Ig2"), v[4], ig2_d, TUBE_REL, TUBE_ABS);
        for r in 0..3 {
            for c in 0..3 {
                assert_close(
                    &format!("{ctx} jac[{r}][{c}]"),
                    v[5 + 3 * r + c],
                    jac_d[r][c],
                    TUBE_REL,
                    TUBE_ABS,
                );
            }
        }
        assert_close(&format!("{ctx} eval Ip"), v[14], ip_d, TUBE_REL, TUBE_ABS);
        assert_close(&format!("{ctx} eval Ig2"), v[15], ig2_d, TUBE_REL, TUBE_ABS);
        assert_close(&format!("{ctx} eval Ig1"), v[16], ig1_d, TUBE_REL, TUBE_ABS);
        for r in 0..3 {
            for c in 0..3 {
                assert_close(
                    &format!("{ctx} eval jac[{r}][{c}]"),
                    v[17 + 3 * r + c],
                    jac_d[r][c],
                    TUBE_REL,
                    TUBE_ABS,
                );
            }
        }
        n += 1;
        if ip_d > 1e-4 {
            conducting += 1;
        }
    }
    assert!(n > 200, "{label}: too few harness rows ({n})");
    assert!(
        conducting > 30,
        "{label}: grid barely conducts ({conducting})"
    );
}

/// Sharp pentode, Reefman Derk §4.4 Rational screen form (EL84 card):
/// template `tube_*_pentode` vs canonical KorenPentode. Note the Derk Ip0
/// correctly has NO ×2 (Reefman's own `(1+sgn(E1))/2` form; Kg1 fits carry
/// it) — this pin also protects against someone "fixing" it to match the
/// triode.
#[test]
fn template_pentode_rational_matches_devices_crate() {
    check_pentode_family("pent_rat", &P_EL84.to_device(ScreenForm::Rational));
}

/// Beam tetrode, Reefman DerkE §4.5 Exponential screen form (6L6GC card):
/// template `tube_*_beam_tetrode` vs canonical KorenPentode.
#[test]
fn template_pentode_exponential_matches_devices_crate() {
    check_pentode_family("pent_exp", &P_6L6GC.to_device(ScreenForm::Exponential));
}

/// Classical Norman Koren pentode (KT88 card): template
/// `tube_*_pentode_classical` vs canonical KorenPentode. The Classical plate
/// current DOES carry the ×2 (`2·E1^Ex/Kg1·arctan(Vpk/Kvb)`) — same factor
/// class as the triode, pinned here in its pentode incarnation.
#[test]
fn template_pentode_classical_matches_devices_crate() {
    check_pentode_family("pent_cls", &P_KT88.to_device(ScreenForm::Classical));
}

/// Variable-mu (Reefman §5 two-section blend), both screen forms:
/// 6K7 (PenthodeVD, Rational) and EF89 (PenthodeVDE, Exponential) —
/// template `tube_*_pentode_v` / `tube_*_beam_tetrode_v` vs canonical
/// KorenPentode with svar > 0.
#[test]
fn template_pentode_variable_mu_matches_devices_crate() {
    check_pentode_family("pent_vrat", &P_6K7.to_device(ScreenForm::Rational));
    check_pentode_family("pent_vexp", &P_EF89.to_device(ScreenForm::Exponential));
}

/// Grid-off (Phase 1b 2D reduction) wrappers for all three sharp forms:
/// template `tube_*_grid_off` vs canonical
/// `plate_current_grid_off` / `screen_current_grid_off` /
/// `jacobian_2x2_grid_off`. Primarily pins the 3x3→2x2 index mapping
/// (jac9[0,1,3,4] = upper-left block) on both sides.
#[test]
fn template_pentode_grid_off_matches_devices_crate() {
    let families: &[(&str, KorenPentode)] = &[
        ("goff_rat", P_EL84.to_device(ScreenForm::Rational)),
        ("goff_exp", P_6L6GC.to_device(ScreenForm::Exponential)),
        ("goff_cls", P_KT88.to_device(ScreenForm::Classical)),
    ];
    for (label, dev) in families {
        let mut n = 0u32;
        for (l, v) in tube_harness_rows().iter().filter(|(l, _)| l == label) {
            assert_eq!(v.len(), 9, "bad {l} row width");
            let (vgk, vpk, vg2k_frozen) = (v[0], v[1], v[2]);
            let ctx = format!("{l} at vgk={vgk:.3} vpk={vpk:.1}");
            let ip_d = dev.plate_current_grid_off(vgk, vpk, vg2k_frozen);
            let ig2_d = dev.screen_current_grid_off(vgk, vpk, vg2k_frozen);
            let jac_d = dev.jacobian_2x2_grid_off(vgk, vpk, vg2k_frozen);
            assert_close(&format!("{ctx} Ip"), v[3], ip_d, TUBE_REL, TUBE_ABS);
            assert_close(&format!("{ctx} Ig2"), v[4], ig2_d, TUBE_REL, TUBE_ABS);
            assert_close(
                &format!("{ctx} jac[0][0]"),
                v[5],
                jac_d[0][0],
                TUBE_REL,
                TUBE_ABS,
            );
            assert_close(
                &format!("{ctx} jac[0][1]"),
                v[6],
                jac_d[0][1],
                TUBE_REL,
                TUBE_ABS,
            );
            assert_close(
                &format!("{ctx} jac[1][0]"),
                v[7],
                jac_d[1][0],
                TUBE_REL,
                TUBE_ABS,
            );
            assert_close(
                &format!("{ctx} jac[1][1]"),
                v[8],
                jac_d[1][1],
                TUBE_REL,
                TUBE_ABS,
            );
            n += 1;
        }
        assert!(n > 50, "{label}: too few harness rows ({n})");
    }
}
