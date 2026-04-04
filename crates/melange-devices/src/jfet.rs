//! JFET (Junction Field-Effect Transistor) models.

use crate::{safeguards, NonlinearDevice, VT_ROOM};

/// JFET channel type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JfetChannel {
    N,
    P,
}

/// JFET model (Shichman-Hodges/SPICE Level 1).
///
/// For N-channel: negative Vgs to control, positive Vds
/// For P-channel: positive Vgs to control, negative Vds
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Jfet {
    /// Channel type
    pub channel: JfetChannel,
    /// Pinch-off voltage [V] (always negative for N-channel)
    pub vp: f64,
    /// Saturation current [A] (IDSS)
    pub idss: f64,
    /// Channel length modulation [1/V]
    pub lambda: f64,
    /// Gate saturation current [A]
    pub is: f64,
}

impl Jfet {
    /// Create a new JFET.
    ///
    /// # Panics
    /// Panics if `vp` is zero (would cause division by zero) or `idss` is not positive.
    pub fn new(channel: JfetChannel, vp: f64, idss: f64) -> Self {
        assert!(vp.abs() > 1e-15, "JFET Vp must be non-zero, got {}", vp);
        assert!(idss > 0.0, "JFET IDSS must be positive, got {}", idss);
        Self {
            channel,
            vp,
            idss,
            lambda: 0.001,
            is: 1e-14,
        }
    }

    /// 2N5457 N-channel JFET.
    pub fn n_2n5457() -> Self {
        let c = crate::catalog::jfets::lookup("2N5457").expect("2N5457 catalog entry");
        let mut j = Self::new(JfetChannel::N, c.vp, c.idss);
        j.lambda = c.lambda;
        j
    }

    /// J201 N-channel JFET (common in audio).
    pub fn n_j201() -> Self {
        let c = crate::catalog::jfets::lookup("J201").expect("J201 catalog entry");
        let mut j = Self::new(JfetChannel::N, c.vp, c.idss);
        j.lambda = c.lambda;
        j
    }

    /// 2N3819 N-channel JFET.
    pub fn n_2n3819() -> Self {
        let c = crate::catalog::jfets::lookup("2N3819").expect("2N3819 catalog entry");
        let mut j = Self::new(JfetChannel::N, c.vp, c.idss);
        j.lambda = c.lambda;
        j
    }

    /// 2N5460 P-channel JFET.
    pub fn p_2n5460() -> Self {
        let c = crate::catalog::jfets::lookup("2N5460").expect("2N5460 catalog entry");
        let mut j = Self::new(JfetChannel::P, c.vp, c.idss);
        j.lambda = c.lambda;
        j
    }

    fn sign(&self) -> f64 {
        match self.channel {
            JfetChannel::N => 1.0,
            JfetChannel::P => -1.0,
        }
    }

    /// Drain current given Vgs and Vds.
    pub fn drain_current(&self, vgs: f64, vds: f64) -> f64 {
        let s = self.sign();

        // Convert to N-channel equivalent voltages
        // After sign flip, both N and P channel use N-channel equations
        let vgs_eff = s * vgs;
        let vds_eff = s * vds;

        // For P-channel JFET:
        // - Constructor stores vp as positive (e.g., 2.5V)
        // - To use N-channel equations, we need vp_eff to be negative (-2.5V)
        // - vgst = vgs_eff - vp_eff = -Vgs - (-Vp) = Vp - Vgs
        // - For conduction: vgst > 0 means Vp > Vgs (correct for P-channel!)
        let vp_eff = match self.channel {
            JfetChannel::N => self.vp,  // Already negative
            JfetChannel::P => -self.vp, // Flip to negative
        };
        let vgst = vgs_eff - vp_eff;

        if vgst <= 0.0 {
            // Subthreshold: weak exponential for smooth NR convergence
            return s * 1e-12 * (vgst / (2.0 * VT_ROOM)).exp().min(1.0);
        }

        // Saturation voltage (Vds at which device enters saturation)
        let vds_sat = vgst;

        let vp_abs = self.vp.abs();

        if vds_eff < vds_sat {
            // Linear (triode) region
            // Id = (2*IDSS/Vp^2) * ((Vgs-Vp)*Vds - Vds^2/2)
            let id = self.idss / (vp_abs * vp_abs) * (2.0 * vgst * vds_eff - vds_eff * vds_eff);
            s * id * (1.0 + self.lambda * vds_eff)
        } else {
            // Saturation region
            // Id = IDSS * (1 - (Vgs-Vp)/Vp)^2 = IDSS * (Vgst/Vp)^2
            let id = self.idss * (vgst / vp_abs).powi(2);
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

        let vp_eff = match self.channel {
            JfetChannel::N => self.vp,
            JfetChannel::P => -self.vp,
        };
        let vgst = vgs_eff - vp_eff;
        if vgst <= 0.0 {
            // Subthreshold: derivative of weak exponential
            let sub = 1e-12 * (vgst / (2.0 * VT_ROOM)).exp().min(1.0);
            let gm = if sub < 1e-12 {
                sub / (2.0 * VT_ROOM)
            } else {
                0.0
            };
            return (gm, 0.0);
        }

        let vds_sat = vgst;
        let vp_abs = self.vp.abs();

        // ∂Id/∂Vgs
        let d_id_d_vgs = if vds_eff < vds_sat {
            // Linear: (2*IDSS/Vp^2) * Vds
            self.idss * 2.0 * vds_eff / (vp_abs * vp_abs) * (1.0 + self.lambda * vds_eff)
        } else {
            // Saturation: (2*IDSS/Vp^2) * Vgst
            self.idss * 2.0 * vgst / (vp_abs * vp_abs) * (1.0 + self.lambda * vds_eff)
        };

        // ∂Id/∂Vds
        let d_id_d_vds = if vds_eff < vds_sat {
            // Linear: derivative of (2*Vgst*Vds - Vds^2)
            let id_base = self.idss / (vp_abs * vp_abs) * (2.0 * vgst - 2.0 * vds_eff);
            let id_lambda = self.idss / (vp_abs * vp_abs)
                * (2.0 * vgst * vds_eff - vds_eff * vds_eff)
                * self.lambda;
            id_base * (1.0 + self.lambda * vds_eff) + id_lambda
        } else {
            // Saturation: IDSS * (Vgst/Vp)^2 * lambda
            self.idss * (vgst / vp_abs).powi(2) * self.lambda
        };

        // Apply chain rule for polarity: ∂Id/∂vgs = s * ∂(s*f)/∂(s*vgs) = s² * ∂f/∂vgs_eff
        // For N-channel (s=1): s²=1, no change. For P-channel (s=-1): s²=1, sign preserved.
        (s * s * d_id_d_vgs, s * s * d_id_d_vds)
    }

    /// Gate current (very small, only conducts when forward biased).
    pub fn gate_current(&self, vgs: f64) -> f64 {
        // Gate-source diode
        let vgs_eff = match self.channel {
            JfetChannel::N => vgs,  // Forward bias when Vgs > 0
            JfetChannel::P => -vgs, // Forward bias when Vgs < 0
        };

        if vgs_eff > 0.0 {
            // Forward biased
            self.is * (safeguards::safe_exp(vgs_eff / VT_ROOM) - 1.0)
        } else {
            // Reverse biased
            -self.is
        }
    }
}

impl NonlinearDevice<2> for Jfet {
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
    fn test_jfet_cutoff() {
        let jfet = Jfet::n_j201();

        // Vgs more negative than Vp: cutoff
        // Vp = -0.8, so Vgs = -2.0 < -0.8 should cutoff
        let id = jfet.drain_current(-2.0, 5.0);
        assert!(id.abs() < 1e-10);
    }

    #[test]
    fn test_jfet_saturation() {
        let jfet = Jfet::n_j201();

        // Vgs = 0, Vds > Vgs - Vp: saturation at IDSS
        // Vgst = 0 - (-0.8) = 0.8
        let id = jfet.drain_current(0.0, 5.0);

        // Should be close to IDSS (with channel length modulation)
        assert!(id > 0.0);
        assert!(id > jfet.idss * 0.5); // At least half of IDSS
    }

    #[test]
    fn test_jfet_idss() {
        let jfet = Jfet::n_j201();

        // At Vgs = 0, Id should be approximately IDSS
        let id = jfet.drain_current(0.0, 9.0);

        // Allow for channel length modulation
        assert!(id > jfet.idss); // lambda > 0 increases current at high Vds
    }

    #[test]
    fn test_jfet_polarity() {
        let n_jfet = Jfet::n_j201();
        let p_jfet = Jfet::p_2n5460();

        // N-channel: positive Vds, small negative Vgs (above Vp to conduct)
        // Vp = -0.8, Vgs = -0.3 > -0.8, so conducts
        let id_n = n_jfet.drain_current(-0.3, 5.0);

        // P-channel: negative Vds, small positive Vgs (below Vp to conduct)
        // Vp = 2.5, Vgs = 2.0 < 2.5, so conducts
        let id_p = p_jfet.drain_current(2.0, -5.0);

        assert!(
            id_n > 0.0,
            "N-channel current should be positive, got {}",
            id_n
        );
        assert!(
            id_p < 0.0,
            "P-channel current should be negative, got {}",
            id_p
        );
    }

    #[test]
    fn test_jfet_jacobian() {
        let jfet = Jfet::n_j201();

        let vgs = -0.3;
        let vds = 5.0;

        let (d_id_d_vgs, d_id_d_vds) = jfet.jacobian_partial(vgs, vds);

        // Transconductance should be positive
        assert!(
            d_id_d_vgs > 0.0,
            "gm should be positive, got {}",
            d_id_d_vgs
        );

        // Output conductance should be positive
        assert!(
            d_id_d_vds > 0.0,
            "gds should be positive, got {}",
            d_id_d_vds
        );
    }

    #[test]
    fn test_jfet_jacobian_numerical() {
        let jfet = Jfet::n_j201();

        let vgs = -0.3;
        let vds = 5.0;

        // Analytical Jacobian
        let (d_id_d_vgs, d_id_d_vds) = jfet.jacobian_partial(vgs, vds);

        // Numerical verification
        let dv = 1e-6;
        let id = jfet.drain_current(vgs, vds);
        let id_vgs = jfet.drain_current(vgs + dv, vds);
        let id_vds = jfet.drain_current(vgs, vds + dv);

        let num_d_id_d_vgs = (id_vgs - id) / dv;
        let num_d_id_d_vds = (id_vds - id) / dv;

        // Should match within 1%
        assert!((d_id_d_vgs - num_d_id_d_vgs).abs() / d_id_d_vgs.abs() < 0.01);
        assert!((d_id_d_vds - num_d_id_d_vds).abs() / d_id_d_vds.abs() < 0.01);
    }

    #[test]
    fn test_jfet_p_channel_jacobian_numerical() {
        let jfet = Jfet::p_2n5460();

        // P-channel: positive Vgs (below Vp to conduct), negative Vds
        let vgs = 2.0;
        let vds = -5.0;

        // Analytical Jacobian
        let (d_id_d_vgs, d_id_d_vds) = jfet.jacobian_partial(vgs, vds);

        // Numerical verification (central difference)
        let eps = 1e-6;
        let num_d_id_d_vgs =
            (jfet.drain_current(vgs + eps, vds) - jfet.drain_current(vgs - eps, vds)) / (2.0 * eps);
        let num_d_id_d_vds =
            (jfet.drain_current(vgs, vds + eps) - jfet.drain_current(vgs, vds - eps)) / (2.0 * eps);

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
    fn test_jfet_multi_region_fd_jacobian() {
        let eps = 1e-7;

        // N-channel: Vp = -0.8, IDSS = 0.3e-3
        let n_jfet = Jfet::n_j201();

        // Operating points covering all regions:
        // (Vgs, Vds, description)
        let n_points: &[(f64, f64, &str)] = &[
            // Cutoff boundary: Vgs just above Vp
            (-0.75, 5.0, "N cutoff boundary"),
            // Saturation: Vgs=0, Vds >> Vgs-Vp
            (0.0, 5.0, "N saturation Vgs=0"),
            (-0.3, 5.0, "N saturation Vgs=-0.3"),
            (-0.5, 3.0, "N saturation Vgs=-0.5"),
            // Linear/triode: Vds < Vgs - Vp
            (0.0, 0.2, "N linear Vds=0.2"),
            (0.0, 0.5, "N linear Vds=0.5"),
            (-0.3, 0.1, "N linear Vgs=-0.3 Vds=0.1"),
        ];

        for &(vgs, vds, desc) in n_points {
            let (d_id_d_vgs, d_id_d_vds) = n_jfet.jacobian_partial(vgs, vds);

            let fd_vgs = (n_jfet.drain_current(vgs + eps, vds)
                - n_jfet.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (n_jfet.drain_current(vgs, vds + eps)
                - n_jfet.drain_current(vgs, vds - eps))
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
                    "N-JFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
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

        // P-channel: Vp = 2.5, IDSS = 5e-3
        let p_jfet = Jfet::p_2n5460();

        let p_points: &[(f64, f64, &str)] = &[
            // P-channel cutoff boundary: Vgs just below Vp
            (2.4, -5.0, "P cutoff boundary"),
            // P-channel saturation: Vgs=0, Vds negative
            (0.0, -5.0, "P saturation Vgs=0"),
            (1.0, -5.0, "P saturation Vgs=1.0"),
            // P-channel linear: |Vds| < |Vgs - Vp|
            (0.0, -0.5, "P linear Vds=-0.5"),
            (1.0, -0.2, "P linear Vgs=1 Vds=-0.2"),
        ];

        for &(vgs, vds, desc) in p_points {
            let (d_id_d_vgs, d_id_d_vds) = p_jfet.jacobian_partial(vgs, vds);

            let fd_vgs = (p_jfet.drain_current(vgs + eps, vds)
                - p_jfet.drain_current(vgs - eps, vds))
                / (2.0 * eps);
            let fd_vds = (p_jfet.drain_current(vgs, vds + eps)
                - p_jfet.drain_current(vgs, vds - eps))
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
                    "P-JFET {} at {} (Vgs={}, Vds={}): analytic={:.6e} fd={:.6e} err={:.2e}",
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
    #[should_panic(expected = "Vp must be non-zero")]
    fn test_jfet_vp_zero_rejected() {
        let _ = Jfet::new(JfetChannel::N, 0.0, 1e-3);
    }

    #[test]
    #[should_panic(expected = "IDSS must be positive")]
    fn test_jfet_negative_idss_rejected() {
        let _ = Jfet::new(JfetChannel::N, -2.0, -1e-3);
    }
}
