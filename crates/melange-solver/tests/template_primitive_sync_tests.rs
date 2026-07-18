//! Template <-> primitives synchronization tests.
//!
//! The self-contained Rust templates (`spice_limiting.rs.tera`,
//! `device_diode.rs.tera`) are duplicates of canonical code in
//! `melange-primitives` (pnjlim/fetlim) and `melange-devices`
//! (DiodeShockley). They are pure Rust (no tera tags), so we `include!` them
//! directly and pin their behavior against the canonical implementations.
//! Any drift between the copies fails loudly here.

/// Template code compiled as-is. `fast_exp` is provided as the precise
/// clamped exp (the generated code's `--cfg melange_precise_exp` path), so
/// template-vs-devices comparisons are exact up to f64 rounding.
#[allow(dead_code)]
mod tpl {
    #[inline(always)]
    fn fast_exp(x: f64) -> f64 {
        x.clamp(-40.0, 40.0).exp()
    }

    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/templates/rust/spice_limiting.rs.tera"
    ));
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/templates/rust/device_diode.rs.tera"
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
}

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
    use melange_devices::DiodeShockley;

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
