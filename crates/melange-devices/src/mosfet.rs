//! MOSFET transistor models.
//!
//! Simple square-law and level-1 SPICE models.

use crate::{NonlinearDevice, VT_ROOM};

/// MOSFET channel type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelType {
    N,
    P,
}

/// Simple MOSFET model (square law, saturation only).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mosfet {
    /// Channel type (N or P)
    pub channel: ChannelType,
    /// Threshold voltage [V]
    pub vt: f64,
    /// Transconductance parameter [A/V²]
    pub kp: f64,
    /// Channel length modulation [1/V]
    pub lambda: f64,
}

impl Mosfet {
    /// Create a new MOSFET.
    ///
    /// # Panics
    /// Panics if `kp` is not positive.
    pub fn new(channel: ChannelType, vt: f64, kp: f64, lambda: f64) -> Self {
        assert!(kp > 0.0, "MOSFET Kp must be positive, got {}", kp);
        Self {
            channel,
            vt,
            kp,
            lambda,
        }
    }

    /// 2N7000 N-channel MOSFET (common small-signal).
    pub fn n_2n7000() -> Self {
        let c = crate::catalog::mosfets::lookup("2N7000").expect("2N7000 catalog entry");
        Self::new(ChannelType::N, c.vt, c.kp, c.lambda)
    }

    /// BS170 N-channel MOSFET.
    pub fn n_bs170() -> Self {
        let c = crate::catalog::mosfets::lookup("BS170").expect("BS170 catalog entry");
        Self::new(ChannelType::N, c.vt, c.kp, c.lambda)
    }

    /// Get sign multiplier based on channel type.
    fn sign(&self) -> f64 {
        match self.channel {
            ChannelType::N => 1.0,
            ChannelType::P => -1.0,
        }
    }

    /// Drain current given Vgs and Vds.
    ///
    /// VTO is kept signed through the effective-space math:
    /// `vov = vgs_eff - s*vt` handles enhancement AND depletion devices for
    /// both polarities (depletion NMOS has VTO < 0 and conducts at Vgs = 0;
    /// standard PMOS has VTO < 0 with s = -1).
    ///
    /// SPICE-style mode handling: for `vds_eff >= 0` (normal mode) the
    /// source-referenced overdrive governs. For `vds_eff < 0` (reverse mode)
    /// the drain acts as the source, so the drain-referenced overdrive
    /// `vgd_eff - vt_eff` governs: the normal-mode equations are evaluated at
    /// `(vgd, -vds)` and the result is negated. The triode expression is
    /// exactly symmetric under this swap, so value and all Jacobian entries
    /// are continuous across Vds = 0.
    pub fn drain_current(&self, vgs: f64, vds: f64) -> f64 {
        let s = self.sign();
        let vgs_eff = s * vgs;
        let vds_eff = s * vds;

        // Signed threshold in effective (N-channel) space.
        let vt_eff = s * self.vt;

        // Mode selection: vc = controlling gate reference, vd = |vds_eff|,
        // m = polarity of the drain current.
        let (vc, vd, m) = if vds_eff >= 0.0 {
            (vgs_eff, vds_eff, 1.0)
        } else {
            // Reverse mode: drain acts as source. vgd_eff = vgs_eff - vds_eff.
            (vgs_eff - vds_eff, -vds_eff, -1.0)
        };
        let vov = vc - vt_eff; // Overdrive voltage

        if vov <= 0.0 {
            // Subthreshold: weak exponential for smooth NR convergence.
            // Gated by tanh(vd/2VT) so the leakage is continuous (and zero)
            // at Vds = 0 where the mode polarity m flips sign.
            let sub = 1e-12 * (vov / (2.0 * VT_ROOM)).exp().min(1.0);
            return s * m * sub * (vd / (2.0 * VT_ROOM)).tanh();
        }

        // Channel-length modulation uses the mode-consistent |Vds| (SPICE
        // behavior — see DEVICE_MODELS.md, `1 + LAMBDA*|Vds|`).
        let clm = 1.0 + self.lambda * vd;

        if vd < vov {
            // Linear (triode) region
            let id = self.kp * (vov * vd - 0.5 * vd * vd);
            s * m * id * clm
        } else {
            // Saturation region
            let id = 0.5 * self.kp * vov * vov;
            s * m * id * clm
        }
    }

    /// Partial derivatives for Jacobian.
    ///
    /// Returns (∂Id/∂Vgs, ∂Id/∂Vds)
    pub fn jacobian_partial(&self, vgs: f64, vds: f64) -> (f64, f64) {
        let s = self.sign();
        let vgs_eff = s * vgs;
        let vds_eff = s * vds;

        let vt_eff = s * self.vt;

        // Mode selection — must mirror drain_current exactly.
        // Id = s * m * f(vc, vd), with:
        //   normal  (m=+1): vc = vgs_eff,           vd = vds_eff
        //   reverse (m=-1): vc = vgs_eff - vds_eff, vd = -vds_eff
        // Chain rule to external (vgs, vds), with s² = 1:
        //   normal:  dId/dVgs = f_c            dId/dVds = f_d
        //   reverse: dId/dVgs = -f_c           dId/dVds = f_c + f_d
        let (vc, vd, m) = if vds_eff >= 0.0 {
            (vgs_eff, vds_eff, 1.0)
        } else {
            (vgs_eff - vds_eff, -vds_eff, -1.0)
        };
        let vov = vc - vt_eff;

        // (f_c, f_d) = (∂f/∂vc, ∂f/∂vd)
        let (f_c, f_d) = if vov <= 0.0 {
            // Subthreshold: derivative of tanh-gated weak exponential.
            // Matches device_mosfet.rs.tera — unconditional, no dead branch.
            let sub = 1e-12 * (vov / (2.0 * VT_ROOM)).exp().min(1.0);
            let t = (vd / (2.0 * VT_ROOM)).tanh();
            (
                sub * t / (2.0 * VT_ROOM),
                sub * (1.0 - t * t) / (2.0 * VT_ROOM),
            )
        } else {
            let clm = 1.0 + self.lambda * vd;

            if vd < vov {
                // Linear (triode) region
                // f_c = ∂/∂vc [kp*((vc-Vt)*vd - 0.5*vd^2)*(1+lambda*vd)] = kp*vd*clm
                let f_c = self.kp * vd * clm;
                // f_d = kp*(vov - vd)*clm + kp*(vov*vd - 0.5*vd^2)*lambda
                let id_base = self.kp * (vov - vd);
                let id_full = self.kp * (vov * vd - 0.5 * vd * vd);
                (f_c, id_base * clm + id_full * self.lambda)
            } else {
                // Saturation region
                let f_c = self.kp * vov * clm;
                let f_d = 0.5 * self.kp * vov * vov * self.lambda;
                (f_c, f_d)
            }
        };

        if m >= 0.0 {
            (f_c, f_d)
        } else {
            (-f_c, f_c + f_d)
        }
    }
}

impl NonlinearDevice<2> for Mosfet {
    /// Input: [Vgs, Vds]
    fn current(&self, v: &[f64; 2]) -> f64 {
        self.drain_current(v[0], v[1])
    }

    fn jacobian(&self, v: &[f64; 2]) -> [f64; 2] {
        let (d_id_d_vgs, d_id_d_vds) = self.jacobian_partial(v[0], v[1]);
        [d_id_d_vgs, d_id_d_vds]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mosfet_cutoff() {
        let mos = Mosfet::n_2n7000();

        // Vgs < Vt: cutoff
        let id = mos.drain_current(0.0, 5.0);
        assert!(id.abs() < 1e-10);
    }

    #[test]
    fn test_mosfet_saturation() {
        let mos = Mosfet::n_2n7000();

        // Vgs > Vt, Vds > Vgs - Vt: saturation
        let id = mos.drain_current(5.0, 10.0);
        assert!(id > 0.0);
    }

    #[test]
    fn test_mosfet_linear() {
        let mos = Mosfet::n_2n7000();

        // Vgs > Vt, Vds < Vgs - Vt: linear region
        let id_lin = mos.drain_current(5.0, 1.0);
        let id_sat = mos.drain_current(5.0, 10.0);

        // Linear current should be less than saturation current
        assert!(id_lin < id_sat);
    }

    #[test]
    fn test_mosfet_jacobian() {
        let mos = Mosfet::n_2n7000();

        let vgs = 5.0;
        let vds = 5.0;

        let (d_id_d_vgs, d_id_d_vds) = mos.jacobian_partial(vgs, vds);

        // Transconductance should be positive
        assert!(d_id_d_vgs > 0.0);

        // Output conductance should be positive
        assert!(d_id_d_vds >= 0.0);
    }

    #[test]
    fn test_mosfet_jacobian_numerical() {
        let mos = Mosfet::n_2n7000();

        let vgs = 5.0;
        let vds = 5.0;

        // Analytical Jacobian
        let (d_id_d_vgs, d_id_d_vds) = mos.jacobian_partial(vgs, vds);

        // Numerical verification
        let dv = 1e-6;
        let id = mos.drain_current(vgs, vds);
        let id_vgs = mos.drain_current(vgs + dv, vds);
        let id_vds = mos.drain_current(vgs, vds + dv);

        let num_d_id_d_vgs = (id_vgs - id) / dv;
        let num_d_id_d_vds = (id_vds - id) / dv;

        // Should match within 1%
        assert!((d_id_d_vgs - num_d_id_d_vgs).abs() / d_id_d_vgs.abs() < 0.01);
        assert!((d_id_d_vds - num_d_id_d_vds).abs() / d_id_d_vds.abs() < 0.01);
    }

    #[test]
    fn test_mosfet_p_channel_jacobian_numerical() {
        // P-channel MOSFET: Vt=-2.0, same Kp
        let mos = Mosfet::new(ChannelType::P, -2.0, 0.1, 0.01);

        // P-channel: negative Vgs (more negative than Vt to conduct), negative Vds
        let vgs = -5.0;
        let vds = -5.0;

        // Analytical Jacobian
        let (d_id_d_vgs, d_id_d_vds) = mos.jacobian_partial(vgs, vds);

        // Numerical verification (central difference)
        let eps = 1e-6;
        let num_d_id_d_vgs =
            (mos.drain_current(vgs + eps, vds) - mos.drain_current(vgs - eps, vds)) / (2.0 * eps);
        let num_d_id_d_vds =
            (mos.drain_current(vgs, vds + eps) - mos.drain_current(vgs, vds - eps)) / (2.0 * eps);

        // Should match within 1%
        assert!(
            (d_id_d_vgs - num_d_id_d_vgs).abs() / num_d_id_d_vgs.abs().max(1e-15) < 0.01,
            "P-channel gm: analytical={:.6e}, numerical={:.6e}",
            d_id_d_vgs,
            num_d_id_d_vgs
        );
        assert!(
            (d_id_d_vds - num_d_id_d_vds).abs() / num_d_id_d_vds.abs().max(1e-15) < 0.01,
            "P-channel gds: analytical={:.6e}, numerical={:.6e}",
            d_id_d_vds,
            num_d_id_d_vds
        );
    }

    /// Comprehensive multi-region FD Jacobian test covering cutoff boundary,
    /// linear/triode, saturation, and P-channel operation.
    #[test]
    fn test_mosfet_multi_region_fd_jacobian() {
        let eps = 1e-7;

        // N-channel: Vt=2.0, Kp=0.1, lambda=0.01
        let n_mos = Mosfet::n_2n7000();

        // Operating points covering all regions:
        // (Vgs, Vds, description)
        let n_points: &[(f64, f64, &str)] = &[
            // Near cutoff boundary: Vgs above Vt with enough margin for FD
            (2.5, 5.0, "N near cutoff boundary"),
            // Saturation: Vds > Vgs - Vt
            (5.0, 10.0, "N saturation Vgs=5 Vds=10"),
            (3.0, 5.0, "N saturation Vgs=3 Vds=5"),
            (4.0, 8.0, "N saturation Vgs=4 Vds=8"),
            // Linear/triode: Vds < Vgs - Vt
            (5.0, 1.0, "N linear Vgs=5 Vds=1"),
            (5.0, 0.5, "N linear Vgs=5 Vds=0.5"),
            (10.0, 2.0, "N linear Vgs=10 Vds=2"),
        ];

        for &(vgs, vds, desc) in n_points {
            let (d_id_d_vgs, d_id_d_vds) = n_mos.jacobian_partial(vgs, vds);

            let fd_vgs = (n_mos.drain_current(vgs + eps, vds)
                - n_mos.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (n_mos.drain_current(vgs, vds + eps)
                - n_mos.drain_current(vgs, vds - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dId/dVgs", d_id_d_vgs, fd_vgs),
                ("dId/dVds", d_id_d_vds, fd_vds),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "N-MOSFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgs,
                    vds,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }

        // P-channel: Vt=-2.0, Kp=0.1, lambda=0.01
        let p_mos = Mosfet::new(ChannelType::P, -2.0, 0.1, 0.01);

        let p_points: &[(f64, f64, &str)] = &[
            // P-channel near cutoff boundary: Vgs below -Vt with enough margin for FD
            (-2.5, -5.0, "P near cutoff boundary"),
            // P-channel saturation: |Vds| > |Vgs - Vt|
            (-5.0, -10.0, "P saturation Vgs=-5 Vds=-10"),
            (-3.0, -5.0, "P saturation Vgs=-3 Vds=-5"),
            // P-channel linear: |Vds| < |Vgs - Vt|
            (-5.0, -1.0, "P linear Vgs=-5 Vds=-1"),
            (-5.0, -0.5, "P linear Vgs=-5 Vds=-0.5"),
            (-10.0, -2.0, "P linear Vgs=-10 Vds=-2"),
        ];

        for &(vgs, vds, desc) in p_points {
            let (d_id_d_vgs, d_id_d_vds) = p_mos.jacobian_partial(vgs, vds);

            let fd_vgs = (p_mos.drain_current(vgs + eps, vds)
                - p_mos.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (p_mos.drain_current(vgs, vds + eps)
                - p_mos.drain_current(vgs, vds - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dId/dVgs", d_id_d_vgs, fd_vgs),
                ("dId/dVds", d_id_d_vds, fd_vds),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "P-MOSFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgs,
                    vds,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }
    }

    #[test]
    #[should_panic(expected = "Kp must be positive")]
    fn test_mosfet_zero_kp_rejected() {
        let _ = Mosfet::new(ChannelType::N, 2.0, 0.0, 0.01);
    }

    #[test]
    fn test_mosfet_subthreshold_gm_boundary() {
        // Regression: prior code had `if sub < 1e-12 { ... } else { 0.0 }`
        // which incorrectly returned gm=0 at the exact vov=0 boundary
        // where sub = 1e-12 exactly. Codegen template has the unconditional
        // form; runtime must match.
        let mos = Mosfet::new(ChannelType::N, 2.0, 0.1, 0.01);
        let (gm, gds) = mos.jacobian_partial(2.0, 1.0); // vgs == Vt -> vov = 0
                                                        // tanh(vd/2VT) gate at vd=1.0 is 1.0 to machine precision.
        let expected_gm = 1e-12 / (2.0 * crate::VT_ROOM);
        assert!(
            (gm - expected_gm).abs() / expected_gm < 1e-9,
            "subthreshold gm at vov=0: got {}, expected {}",
            gm,
            expected_gm
        );
        // gds carries the tanh-gate derivative sub*(1-t^2)/(2VT); at vd=1.0
        // sech^2 has fully decayed, so it is zero to well below any solver
        // tolerance (< 1e-24 A/V).
        assert!(
            gds.abs() < 1e-24,
            "subthreshold gds at vov=0, vds=1: got {}",
            gds
        );
    }

    /// VTO must stay signed: a depletion-mode NMOS (VTO < 0, e.g. LND150 /
    /// DN2540 CCS parts) conducts at Vgs = 0. The old `vt.abs()` silently
    /// converted these into enhancement devices (off at Vgs = 0).
    #[test]
    fn test_mosfet_depletion_nmos_conducts_at_vgs_zero() {
        // Depletion NMOS: VTO = -2.0 V
        let mos = Mosfet::new(ChannelType::N, -2.0, 1e-3, 0.01);

        // Vgs = 0: vov = 0 - (-2) = 2 > 0 -> conducts (saturation at Vds=10)
        let id = mos.drain_current(0.0, 10.0);
        let expected = 0.5 * 1e-3 * 2.0 * 2.0 * (1.0 + 0.01 * 10.0);
        assert!(
            (id - expected).abs() / expected < 1e-12,
            "depletion NMOS at Vgs=0: got {:.6e}, expected {:.6e}",
            id,
            expected
        );
        assert!(id > 1e-3, "depletion NMOS should conduct mA at Vgs=0");

        // Pinches off below VTO: Vgs = -3 < -2 -> subthreshold
        assert!(
            mos.drain_current(-3.0, 10.0) < 1e-9,
            "depletion NMOS should be off at Vgs=-3"
        );

        // FD Jacobian check at the Vgs=0 operating point
        let eps = 1e-7;
        let (gm, gds) = mos.jacobian_partial(0.0, 10.0);
        let fd_gm = (mos.drain_current(eps, 10.0) - mos.drain_current(-eps, 10.0)) / (2.0 * eps);
        let fd_gds =
            (mos.drain_current(0.0, 10.0 + eps) - mos.drain_current(0.0, 10.0 - eps)) / (2.0 * eps);
        assert!((gm - fd_gm).abs() / fd_gm.abs() < 0.01);
        assert!((gds - fd_gds).abs() / fd_gds.abs() < 0.01);
    }

    /// Enhancement NMOS sanity after the signed-VTO change (unchanged
    /// behavior): off at Vgs=0, on above VTO.
    #[test]
    fn test_mosfet_enhancement_nmos_signed_vto() {
        let mos = Mosfet::new(ChannelType::N, 2.0, 0.1, 0.01);
        assert!(mos.drain_current(0.0, 10.0).abs() < 1e-12);
        let id = mos.drain_current(4.0, 10.0);
        let expected = 0.5 * 0.1 * 2.0 * 2.0 * (1.0 + 0.01 * 10.0);
        assert!((id - expected).abs() / expected < 1e-12);
    }

    /// Standard PMOS (VTO < 0) must still work with signed VTO (previously
    /// rescued by the abs()): off at Vgs=0, on when Vgs < VTO.
    #[test]
    fn test_mosfet_pmos_negative_vto_signed() {
        let mos = Mosfet::new(ChannelType::P, -2.0, 0.1, 0.01);
        // Off at Vgs = 0
        assert!(mos.drain_current(0.0, -10.0).abs() < 1e-12);
        // On at Vgs = -4: vov = -(-4) - (-1)*(-2) = 4 - 2 = 2
        let id = mos.drain_current(-4.0, -10.0);
        let expected = -0.5 * 0.1 * 2.0 * 2.0 * (1.0 + 0.01 * 10.0);
        assert!(
            (id - expected).abs() / expected.abs() < 1e-12,
            "PMOS at Vgs=-4: got {:.6e}, expected {:.6e}",
            id,
            expected
        );
    }

    /// Reverse-mode conduction: when Vgs is below threshold but the
    /// drain-referenced overdrive Vgd - VTO > 0 (Vds negative enough), the
    /// device conducts reverse saturation. The old source-referenced-only
    /// cutoff returned picoamps with dId/dVds = 0.
    #[test]
    fn test_mosfet_reverse_saturation_conducts() {
        let mos = Mosfet::new(ChannelType::N, 2.0, 0.1, 0.01);

        // Vgs = 1 (below VTO=2), Vds = -2: Vgd = 3 -> reverse overdrive 1 V.
        // vd = 2 >= vov_r = 1 -> reverse saturation:
        // Id = -0.5*kp*1^2*(1+lambda*2)
        let id = mos.drain_current(1.0, -2.0);
        let expected = -0.5 * 0.1 * 1.0 * (1.0 + 0.01 * 2.0);
        assert!(
            (id - expected).abs() / expected.abs() < 1e-12,
            "reverse saturation: got {:.6e}, expected {:.6e}",
            id,
            expected
        );

        let (gm, gds) = mos.jacobian_partial(1.0, -2.0);
        assert!(gds > 1e-3, "reverse-mode gds should be substantial");
        assert!(gm < 0.0, "reverse-mode gm should be negative");
    }

    /// FD Jacobian checks in the reverse quadrant (Vds < 0 for N-channel),
    /// covering reverse triode, reverse saturation, points straddling the
    /// Vds=0 mode boundary, and points straddling the reverse cutoff
    /// boundary. Matches the multi-region FD test style.
    #[test]
    fn test_mosfet_reverse_mode_fd_jacobian() {
        let eps = 1e-7;

        // N-channel: Vt=2.0, Kp=0.1, lambda=0.01
        let n_mos = Mosfet::new(ChannelType::N, 2.0, 0.1, 0.01);
        let n_points: &[(f64, f64, &str)] = &[
            // Reverse saturation (Vgs below Vt, Vgd above Vt)
            (1.0, -2.0, "N reverse saturation"),
            (0.0, -4.0, "N reverse saturation deep"),
            // Reverse triode (both overdrives positive)
            (5.0, -1.0, "N reverse triode"),
            (4.0, -0.5, "N reverse triode shallow"),
            // Straddling the Vds=0 mode boundary while conducting
            (5.0, 0.02, "N mode boundary + side"),
            (5.0, -0.02, "N mode boundary - side"),
            // Straddling the reverse cutoff boundary (Vgd crosses Vt):
            // Vgd = Vgs - Vds = 2.2 / 1.8 around Vt = 2.0
            (1.0, -1.2, "N reverse just conducting"),
            (1.0, -0.8, "N reverse just cutoff"),
        ];

        for &(vgs, vds, desc) in n_points {
            let (d_id_d_vgs, d_id_d_vds) = n_mos.jacobian_partial(vgs, vds);

            let fd_vgs = (n_mos.drain_current(vgs + eps, vds)
                - n_mos.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (n_mos.drain_current(vgs, vds + eps)
                - n_mos.drain_current(vgs, vds - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dId/dVgs", d_id_d_vgs, fd_vgs),
                ("dId/dVds", d_id_d_vds, fd_vds),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "N-MOSFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgs,
                    vds,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }

        // P-channel: Vt=-2.0; reverse quadrant is Vds > 0.
        let p_mos = Mosfet::new(ChannelType::P, -2.0, 0.1, 0.01);
        let p_points: &[(f64, f64, &str)] = &[
            (-1.0, 2.0, "P reverse saturation"),
            (-5.0, 1.0, "P reverse triode"),
            (-5.0, 0.02, "P mode boundary + side"),
            (-5.0, -0.02, "P mode boundary - side"),
        ];

        for &(vgs, vds, desc) in p_points {
            let (d_id_d_vgs, d_id_d_vds) = p_mos.jacobian_partial(vgs, vds);

            let fd_vgs = (p_mos.drain_current(vgs + eps, vds)
                - p_mos.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (p_mos.drain_current(vgs, vds + eps)
                - p_mos.drain_current(vgs, vds - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dId/dVgs", d_id_d_vgs, fd_vgs),
                ("dId/dVds", d_id_d_vds, fd_vds),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "P-MOSFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    desc,
                    vgs,
                    vds,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }
    }

    /// Value and Jacobian continuity across the Vds=0 mode boundary, in
    /// conduction, subthreshold, and at the threshold boundary itself.
    #[test]
    fn test_mosfet_continuity_across_vds_zero() {
        let mos = Mosfet::new(ChannelType::N, 2.0, 0.1, 0.01);
        let eps = 1e-9;

        for &vgs in &[5.0, 3.0, 2.0, 1.0, 0.0] {
            let id_pos = mos.drain_current(vgs, eps);
            let id_neg = mos.drain_current(vgs, -eps);
            // Continuous function: |Id(+eps) - Id(-eps)| is O(2*eps*gds),
            // not zero. gds here is at most ~0.3 S -> bound 1e-8 A.
            assert!(
                (id_pos - id_neg).abs() < 1e-8,
                "Id discontinuity at Vds=0, Vgs={}: {:.3e} vs {:.3e}",
                vgs,
                id_pos,
                id_neg
            );

            let (gm_pos, gds_pos) = mos.jacobian_partial(vgs, eps);
            let (gm_neg, gds_neg) = mos.jacobian_partial(vgs, -eps);
            assert!(
                (gm_pos - gm_neg).abs() < 1e-6,
                "gm discontinuity at Vds=0, Vgs={}: {:.6e} vs {:.6e}",
                vgs,
                gm_pos,
                gm_neg
            );
            assert!(
                (gds_pos - gds_neg).abs() < 1e-6,
                "gds discontinuity at Vds=0, Vgs={}: {:.6e} vs {:.6e}",
                vgs,
                gds_pos,
                gds_neg
            );
        }
    }
}
