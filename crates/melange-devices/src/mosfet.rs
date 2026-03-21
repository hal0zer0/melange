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
        let c = crate::catalog::mosfets::lookup("2N7000").unwrap();
        Self::new(ChannelType::N, c.vt, c.kp, c.lambda)
    }

    /// BS170 N-channel MOSFET.
    pub fn n_bs170() -> Self {
        let c = crate::catalog::mosfets::lookup("BS170").unwrap();
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
    pub fn drain_current(&self, vgs: f64, vds: f64) -> f64 {
        let s = self.sign();
        let vgs_eff = s * vgs;
        let vds_eff = s * vds;

        let vt_abs = self.vt.abs();
        let vov = vgs_eff - vt_abs; // Overdrive voltage

        if vov <= 0.0 {
            // Subthreshold: weak exponential for smooth NR convergence
            return s * 1e-12 * (vov / (2.0 * VT_ROOM)).exp().min(1.0);
        }

        if vds_eff < vov {
            // Linear (triode) region
            let id = self.kp * (vov * vds_eff - 0.5 * vds_eff * vds_eff);
            s * id * (1.0 + self.lambda * vds_eff)
        } else {
            // Saturation region
            let id = 0.5 * self.kp * vov * vov;
            s * id * (1.0 + self.lambda * vds_eff)
        }
    }

    /// Partial derivatives for Jacobian.
    ///
    /// Returns (∂Id/∂Vgs, ∂Id/∂Vds)
    pub fn jacobian_partial(&self, vgs: f64, vds: f64) -> (f64, f64) {
        let s = self.sign();
        let vgs_eff = s * vgs;
        let vds_eff = s * vds;

        let vt_abs = self.vt.abs();
        let vov = vgs_eff - vt_abs;

        if vov <= 0.0 {
            // Subthreshold: derivative of weak exponential
            let sub = 1e-12 * (vov / (2.0 * VT_ROOM)).exp().min(1.0);
            let gm = if sub < 1e-12 { sub / (2.0 * VT_ROOM) } else { 0.0 };
            return (gm, 0.0);
        }

        // ∂Id/∂Vgs
        let d_id_d_vgs = if vds_eff < vov {
            // Linear: ∂/∂Vgs [kp*((Vgs-Vt)*Vds - 0.5*Vds^2)] = kp * Vds
            self.kp * vds_eff * (1.0 + self.lambda * vds_eff)
        } else {
            // Saturation: ∂/∂Vgs [0.5*kp*(Vgs-Vt)^2] = kp * (Vgs-Vt)
            self.kp * vov * (1.0 + self.lambda * vds_eff)
        };

        // ∂Id/∂Vds
        let d_id_d_vds = if vds_eff < vov {
            // Linear: ∂/∂Vds [kp*((Vgs-Vt)*Vds - 0.5*Vds^2)*(1+lambda*Vds)]
            // = kp*(Vgs-Vt - Vds)*(1+lambda*Vds) + kp*((Vgs-Vt)*Vds - 0.5*Vds^2)*lambda
            let id_base = self.kp * (vov - vds_eff);
            let id_full = self.kp * (vov * vds_eff - 0.5 * vds_eff * vds_eff);
            id_base * (1.0 + self.lambda * vds_eff) + id_full * self.lambda
        } else {
            // Saturation: 0.5*kp*Vov^2 * lambda
            0.5 * self.kp * vov * vov * self.lambda
        };

        // Apply chain rule for polarity: ∂Id/∂vgs = s * ∂(s*f)/∂(s*vgs) = s² * ∂f/∂vgs_eff
        // For N-channel (s=1): s²=1, no change. For P-channel (s=-1): s²=1, sign preserved.
        (s * s * d_id_d_vgs, s * s * d_id_d_vds)
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
}
