//! Vacuum tube (valve) models.
//!
//! Koren's models for triodes and pentodes.

use crate::NonlinearDevice;

/// Koren triode model with improved grid current.
///
/// Based on Norman Koren's improved vacuum tube models.
/// Parameters from SPICE model cards.
///
/// Grid current uses a smooth power-law model (Leach-style):
///   Ig = ig_max * max(0, Vgk/vgk_onset)^1.5
/// which is physically motivated and has a well-defined analytical Jacobian.
///
/// Optional extension:
/// - **Channel-length modulation (`lambda`)**: `Ip_final = Ip_koren * (1 + lambda * Vpk)`.
///   Gives plate resistance rp = 1/(lambda * Ip) at the operating point.
///   Default `0.0` (no correction, backward compatible).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KorenTriode {
    /// Amplification factor (mu)
    pub mu: f64,
    /// Exponent for Koren's equation
    pub ex: f64,
    /// Kg1 coefficient
    pub kg1: f64,
    /// Kp coefficient
    pub kp: f64,
    /// Kvb coefficient (for knee shaping)
    pub kvb: f64,
    /// Maximum grid current [A] at Vgk = vgk_onset
    pub ig_max: f64,
    /// Grid current onset voltage [V]
    pub vgk_onset: f64,
    /// Channel-length modulation coefficient [1/V]. 0.0 = disabled (default).
    /// Ip_final = Ip_koren * (1 + lambda * Vpk).
    /// Gives plate resistance rp approximately 1/(lambda * Ip) at the operating point.
    pub lambda: f64,
}

/// Default grid current maximum [A].
const DEFAULT_IG_MAX: f64 = 2e-3;
/// Default grid current onset voltage [V].
const DEFAULT_VGK_ONSET: f64 = 0.5;

impl KorenTriode {
    /// Create a new triode model with default grid current parameters.
    pub fn new(mu: f64, ex: f64, kg1: f64, kp: f64, kvb: f64) -> Self {
        Self {
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max: DEFAULT_IG_MAX,
            vgk_onset: DEFAULT_VGK_ONSET,
            lambda: 0.0,
        }
    }

    /// Create a new triode model with custom grid current parameters.
    pub fn with_grid_params(
        mu: f64,
        ex: f64,
        kg1: f64,
        kp: f64,
        kvb: f64,
        ig_max: f64,
        vgk_onset: f64,
    ) -> Self {
        Self {
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max,
            vgk_onset,
            lambda: 0.0,
        }
    }

    /// Create a new triode model with all parameters including lambda (Early effect).
    #[allow(clippy::too_many_arguments)]
    pub fn with_all_params(
        mu: f64,
        ex: f64,
        kg1: f64,
        kp: f64,
        kvb: f64,
        ig_max: f64,
        vgk_onset: f64,
        lambda: f64,
    ) -> Self {
        Self {
            mu,
            ex,
            kg1,
            kp,
            kvb,
            ig_max,
            vgk_onset,
            lambda,
        }
    }

    /// 12AX7 (ECC83) - high-mu twin triode, common in guitar amps.
    ///
    /// Uses original Koren 1996 parameters (Kg1=1060). Overestimates plate current
    /// vs datasheet (~3.4mA vs ~1.2mA at Vgk=0, Vpk=250V). For datasheet-accurate
    /// plate current, use [`ecc83_fitted`](Self::ecc83_fitted).
    pub fn ecc83() -> Self {
        let c = crate::catalog::tubes::lookup("12AX7").unwrap();
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// 12AX7 (ECC83) fitted to RCA datasheet - high-mu twin triode.
    ///
    /// Kg1 re-fit from 1060 to 3000 to match RCA 12AX7 datasheet:
    /// ~1.2mA at Vgk=0, Vpk=250V (vs ~3.4mA with original Koren params).
    /// All other parameters identical to [`ecc83`](Self::ecc83).
    pub fn ecc83_fitted() -> Self {
        let c = crate::catalog::tubes::lookup("12AX7F").unwrap();
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// 12AU7 (ECC82) - medium-mu twin triode.
    pub fn ecc82() -> Self {
        let c = crate::catalog::tubes::lookup("12AU7").unwrap();
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// 12AT7 (ECC81) - medium-mu triode.
    pub fn ecc81() -> Self {
        let c = crate::catalog::tubes::lookup("12AT7").unwrap();
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// 6SL7 - high-mu octal triode.
    pub fn _6sl7() -> Self {
        let c = crate::catalog::tubes::lookup("6SL7").unwrap();
        Self::with_all_params(
            c.mu,
            c.ex,
            c.kg1,
            c.kp,
            c.kvb,
            c.ig_max,
            c.vgk_onset,
            c.lambda,
        )
    }

    /// Calculate plate current given Vgk and Vpk.
    ///
    /// Vgk = grid-cathode voltage
    /// Vpk = plate-cathode voltage
    ///
    /// Uses Koren 1996 equation with optional Early-effect multiplier:
    ///   inner = Kp * (1/mu + Vgk / sqrt(Kvb + Vpk^2))
    ///   E1 = (Vpk / Kp) * ln(1 + exp(inner))
    ///   Ip_koren = E1^ex / Kg1   (if E1 > 0)
    ///   Ip = Ip_koren * (1 + lambda * Vpk)
    pub fn plate_current(&self, vgk: f64, vpk: f64) -> f64 {
        let vpk = vpk.max(1e-3); // Soft floor: prevents hard zero discontinuity

        let s = (self.kvb + vpk * vpk).sqrt();
        let inner = self.kp * (1.0 / self.mu + vgk / s);

        // Numerically stable softplus: ln(1 + exp(x))
        let softplus = if inner > 20.0 {
            inner // ln(1+exp(x)) approx x for large x
        } else if inner < -20.0 {
            0.0 // ln(1+exp(x)) approx 0 for very negative x
        } else {
            (1.0 + inner.exp()).ln()
        };

        let e1 = (vpk / self.kp) * softplus;

        let ip_koren = if e1 <= 0.0 {
            0.0
        } else {
            e1.powf(self.ex) / self.kg1
        };

        // Early-effect multiplier (channel-length modulation)
        ip_koren * (1.0 + self.lambda * vpk)
    }

    /// Grid current using smooth power-law model (Leach-style).
    ///
    /// Ig = ig_max * max(0, Vgk / vgk_onset)^1.5
    ///
    /// Physically motivated: grid acts as a diode when positive, with
    /// a smooth onset and well-defined Jacobian. Returns 0 for Vgk <= 0.
    pub fn grid_current(&self, vgk: f64) -> f64 {
        if vgk <= 0.0 {
            return 0.0;
        }
        let x = vgk / self.vgk_onset;
        self.ig_max * x * x.sqrt() // x^1.5 = x * sqrt(x)
    }

    /// Grid current Jacobian: dIg/dVgk.
    ///
    /// dIg/dVgk = ig_max * 1.5 * (Vgk/vgk_onset)^0.5 / vgk_onset
    pub fn grid_current_jacobian(&self, vgk: f64) -> f64 {
        if vgk <= 0.0 {
            return 0.0;
        }
        let x = vgk / self.vgk_onset;
        self.ig_max * 1.5 * x.sqrt() / self.vgk_onset
    }
}

impl NonlinearDevice<2> for KorenTriode {
    /// Input: [Vgk, Vpk]
    /// Output: Plate current (Ip)
    fn current(&self, v: &[f64; 2]) -> f64 {
        self.plate_current(v[0], v[1])
    }

    /// Jacobian: [dIp/dVgk, dIp/dVpk]
    ///
    /// Computed via chain rule through the Koren E1 equation with Early-effect:
    ///   Ip_koren = E1^ex / Kg1
    ///   Ip = Ip_koren * (1 + lambda * Vpk)
    ///   dIp/dVgk = dIp_koren/dVgk * (1 + lambda * Vpk)
    ///   dIp/dVpk = dIp_koren/dVpk * (1 + lambda * Vpk) + Ip_koren * lambda
    fn jacobian(&self, v: &[f64; 2]) -> [f64; 2] {
        let vgk = v[0];
        let vpk = v[1].max(1e-3); // Soft floor: matches plate_current

        let s = (self.kvb + vpk * vpk).sqrt();
        let inner = self.kp * (1.0 / self.mu + vgk / s);

        // Numerically stable sigmoid and softplus
        let (sigmoid, softplus) = if inner > 20.0 {
            (1.0, inner)
        } else if inner < -20.0 {
            (0.0, 0.0)
        } else {
            let exp_inner = inner.exp();
            (exp_inner / (1.0 + exp_inner), (1.0 + exp_inner).ln())
        };

        let e1 = (vpk / self.kp) * softplus;

        if e1 <= 1e-10 {
            return [0.0, 0.0];
        }

        let ip_koren = e1.powf(self.ex) / self.kg1;
        let lambda_factor = 1.0 + self.lambda * vpk;

        // dIp_koren/dE1 = ex * E1^(ex-1) / Kg1
        let dip_de1 = self.ex * e1.powf(self.ex - 1.0) / self.kg1;

        // dE1/dVgk = Vpk * sigmoid(inner) / s
        let de1_dvgk = vpk * sigmoid / s;

        // dE1/dVpk = softplus/Kp - sigmoid * Vgk * Vpk^2 / s^3
        let de1_dvpk = softplus / self.kp - sigmoid * vgk * vpk * vpk / (s * s * s);

        // dIp_koren/dVgk and dIp_koren/dVpk
        let dip_koren_dvgk = dip_de1 * de1_dvgk;
        let dip_koren_dvpk = dip_de1 * de1_dvpk;

        // Chain rule with Early-effect multiplier:
        // dIp/dVgk = dIp_koren/dVgk * (1 + lambda*Vpk)
        // dIp/dVpk = dIp_koren/dVpk * (1 + lambda*Vpk) + Ip_koren * lambda
        [
            dip_koren_dvgk * lambda_factor,
            dip_koren_dvpk * lambda_factor + ip_koren * self.lambda,
        ]
    }
}

/// Koren pentode model.
///
/// Uses the screen grid voltage (Vsg) in the E1 calculation for screen-limited
/// behavior, with a plate voltage correction factor (1 - exp(-Vpk/Kx)).
///
/// **Experimental**: 3D pentode -- not yet supported by solver (max device dim = 2).
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KorenPentode {
    /// Triode part (provides mu, ex, kg1, kp, kvb)
    pub triode: KorenTriode,
    /// Screen grid mu
    pub mu_sg: f64,
    /// Screen grid kg
    pub kg2: f64,
    /// Plate voltage knee parameter (typically 10-30V)
    pub kx: f64,
}

impl KorenPentode {
    /// EL84 (6BQ5) - popular output pentode.
    pub fn el84() -> Self {
        Self {
            triode: KorenTriode::new(20.0, 1.4, 2000.0, 1000.0, 300.0),
            mu_sg: 15.0,
            kg2: 4000.0,
            kx: 20.0,
        }
    }

    /// EL34 (6CA7) - higher power pentode.
    pub fn el34() -> Self {
        Self {
            triode: KorenTriode::new(11.0, 1.4, 1500.0, 800.0, 200.0),
            mu_sg: 8.0,
            kg2: 3000.0,
            kx: 25.0,
        }
    }

    /// Compute E1 using screen grid voltage instead of plate voltage.
    fn e1_from_screen(&self, vgk: f64, vsg: f64) -> f64 {
        if vsg <= 0.0 {
            return 0.0;
        }
        let s = (self.triode.kvb + vsg * vsg).sqrt();
        let inner = self.triode.kp * (1.0 / self.triode.mu + vgk / s);
        let softplus = if inner > 20.0 {
            inner
        } else if inner < -20.0 {
            0.0
        } else {
            (1.0 + inner.exp()).ln()
        };
        (vsg / self.triode.kp) * softplus
    }
}

impl NonlinearDevice<3> for KorenPentode {
    /// Input: [Vgk, Vpk, Vsg] (grid, plate, screen)
    fn current(&self, v: &[f64; 3]) -> f64 {
        let vgk = v[0];
        let vpk = v[1];
        let vsg = v[2];

        let e1 = self.e1_from_screen(vgk, vsg);
        if e1 <= 0.0 {
            return 0.0;
        }

        // Plate current with plate voltage correction
        let ip_screen = e1.powf(self.triode.ex) / self.triode.kg1;

        // Plate voltage correction: approaches 1 for Vpk >> Kx
        let plate_factor = if vpk > 0.0 {
            1.0 - (-vpk / self.kx).exp()
        } else {
            0.0
        };

        ip_screen * plate_factor
    }

    fn jacobian(&self, v: &[f64; 3]) -> [f64; 3] {
        let vgk = v[0];
        let vpk = v[1];
        let vsg = v[2];

        let e1 = self.e1_from_screen(vgk, vsg);
        if e1 <= 1e-30 || vpk <= 0.0 || vsg <= 0.0 {
            return [0.0, 0.0, 0.0];
        }

        let ip_screen = e1.powf(self.triode.ex) / self.triode.kg1;
        let plate_factor = 1.0 - (-vpk / self.kx).exp();
        let dplate_dvpk = (1.0 / self.kx) * (-vpk / self.kx).exp();

        // dIp/dVpk = ip_screen * dplate_factor/dVpk
        let dip_dvpk = ip_screen * dplate_dvpk;

        // dIp/dE1 = ex * E1^(ex-1) / Kg1
        let dip_de1 = self.triode.ex * e1.powf(self.triode.ex - 1.0) / self.triode.kg1;

        // E1 derivatives w.r.t. Vgk and Vsg (computed like triode but with Vsg)
        let s = (self.triode.kvb + vsg * vsg).sqrt();
        let inner = self.triode.kp * (1.0 / self.triode.mu + vgk / s);
        let sigmoid = if inner > 20.0 {
            1.0
        } else if inner < -20.0 {
            0.0
        } else {
            let exp_inner = inner.exp();
            exp_inner / (1.0 + exp_inner)
        };
        let softplus = if inner > 20.0 {
            inner
        } else if inner < -20.0 {
            0.0
        } else {
            (1.0 + inner.exp()).ln()
        };

        let de1_dvgk = vsg * sigmoid / s;
        let de1_dvsg = softplus / self.triode.kp - sigmoid * vgk * vsg * vsg / (s * s * s);

        let dip_dvgk = dip_de1 * de1_dvgk * plate_factor;
        let dip_dvsg = dip_de1 * de1_dvsg * plate_factor;

        [dip_dvgk, dip_dvpk, dip_dvsg]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triode_cutoff() {
        let tube = KorenTriode::ecc83();

        // Negative grid voltage should reduce current
        // The Koren model doesn't have a hard cutoff, just reduced current
        let ip_off = tube.plate_current(-2.0, 250.0);
        let ip_on = tube.plate_current(0.0, 250.0);
        let ip_very_off = tube.plate_current(-5.0, 250.0);

        // More negative grid should give less current
        assert!(ip_off < ip_on, "More negative grid should reduce current");
        assert!(
            ip_very_off < ip_off,
            "Even more negative should reduce further"
        );
    }

    #[test]
    fn test_triode_conduction() {
        let tube = KorenTriode::ecc83();

        // At Vgk=0, Vpk=250V, should have milliamps of plate current
        let ip = tube.plate_current(0.0, 250.0);
        assert!(ip > 0.001, "Ip should be in mA range: {} A", ip);
        assert!(ip < 0.01, "Ip should not be excessive: {} A", ip);
    }

    #[test]
    fn test_triode_mu() {
        let tube = KorenTriode::ecc83();

        // Test that mu amplifies grid voltage effect
        let ip_vgk_0 = tube.plate_current(0.0, 250.0);
        let ip_vgk_minus1 = tube.plate_current(-1.0, 250.0);
        let ip_vgk_minus2 = tube.plate_current(-2.0, 250.0);

        assert!(ip_vgk_minus1 < ip_vgk_0);
        assert!(ip_vgk_minus2 < ip_vgk_minus1);

        let ratio = ip_vgk_0 / ip_vgk_minus2;
        assert!(
            ratio > 2.0,
            "Mu effect should be noticeable, ratio = {}",
            ratio
        );
    }

    #[test]
    fn test_triode_trait() {
        let tube = KorenTriode::ecc83();

        let ip = tube.current(&[0.0, 250.0]);
        let jac = tube.jacobian(&[0.0, 250.0]);

        assert!(ip > 0.0);
        assert!(jac[0] > 0.0); // dIp/dVgk > 0 (transconductance)
        assert!(jac[1] > 0.0); // dIp/dVpk > 0 (output conductance)
    }

    #[test]
    fn test_12ax7_plate_curves() {
        let tube = KorenTriode::ecc83();

        let ip_0 = tube.plate_current(0.0, 250.0);
        assert!(
            ip_0 > 0.5e-3 && ip_0 < 5.0e-3,
            "Ip(Vgk=0, Vpk=250) = {:.3}mA, expected 0.5-5.0mA",
            ip_0 * 1000.0
        );

        let ip_m1 = tube.plate_current(-1.0, 250.0);
        assert!(
            ip_m1 > 0.1e-3 && ip_m1 < 3.0e-3,
            "Ip(Vgk=-1, Vpk=250) = {:.3}mA, expected 0.1-3.0mA",
            ip_m1 * 1000.0
        );

        let ip_m2 = tube.plate_current(-2.0, 250.0);
        assert!(
            ip_m2 > 0.01e-3 && ip_m2 < 1.5e-3,
            "Ip(Vgk=-2, Vpk=250) = {:.3}mA, expected 0.01-1.5mA",
            ip_m2 * 1000.0
        );

        assert!(ip_0 > ip_m1 && ip_m1 > ip_m2);

        let ip_m4 = tube.plate_current(-4.0, 250.0);
        assert!(
            ip_m4 < 0.01e-3,
            "Ip(Vgk=-4, Vpk=250) = {:.4}mA, expected near cutoff",
            ip_m4 * 1000.0
        );
    }

    #[test]
    fn test_12ax7_fitted_plate_curves() {
        let tube = KorenTriode::ecc83_fitted();

        // Fitted to RCA datasheet: Vgk=0, Vpk=250V → ~1.2mA
        let ip_0 = tube.plate_current(0.0, 250.0);
        assert!(
            ip_0 > 1.0e-3 && ip_0 < 1.4e-3,
            "Ip(Vgk=0, Vpk=250) = {:.4}mA, expected ~1.2mA",
            ip_0 * 1000.0
        );

        // Vgk=-1V: fitted → ~0.59mA (datasheet: ~0.5mA)
        let ip_m1 = tube.plate_current(-1.0, 250.0);
        assert!(
            ip_m1 > 0.3e-3 && ip_m1 < 0.8e-3,
            "Ip(Vgk=-1, Vpk=250) = {:.4}mA, expected ~0.5-0.6mA",
            ip_m1 * 1000.0
        );

        // Vgk=-2V: fitted → ~0.17mA (datasheet: ~0.1mA)
        let ip_m2 = tube.plate_current(-2.0, 250.0);
        assert!(
            ip_m2 > 0.05e-3 && ip_m2 < 0.3e-3,
            "Ip(Vgk=-2, Vpk=250) = {:.4}mA, expected ~0.1-0.2mA",
            ip_m2 * 1000.0
        );

        // Monotonicity and cutoff
        assert!(ip_0 > ip_m1 && ip_m1 > ip_m2);
        assert!(
            tube.plate_current(-4.0, 250.0) < 0.01e-3,
            "Should be near cutoff at Vgk=-4"
        );

        // Verify Kg1 is the only difference from default
        let default = KorenTriode::ecc83();
        assert_eq!(tube.mu, default.mu);
        assert_eq!(tube.ex, default.ex);
        assert_eq!(tube.kp, default.kp);
        assert_eq!(tube.kvb, default.kvb);
        assert!(tube.kg1 > default.kg1, "Fitted Kg1 should be larger");
    }

    /// Verify triode Jacobian against finite differences.
    #[test]
    fn test_triode_jacobian_finite_difference() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-6;

        for &(vgk, vpk) in &[(0.0, 250.0), (-1.0, 200.0), (-2.0, 100.0), (-0.5, 300.0)] {
            let jac = tube.jacobian(&[vgk, vpk]);

            let ip_plus = tube.plate_current(vgk + eps, vpk);
            let ip_minus = tube.plate_current(vgk - eps, vpk);
            let fd_dvgk = (ip_plus - ip_minus) / (2.0 * eps);

            let ip_plus = tube.plate_current(vgk, vpk + eps);
            let ip_minus = tube.plate_current(vgk, vpk - eps);
            let fd_dvpk = (ip_plus - ip_minus) / (2.0 * eps);

            let rel_err_gk = if fd_dvgk.abs() > 1e-15 {
                (jac[0] - fd_dvgk).abs() / fd_dvgk.abs()
            } else {
                jac[0].abs()
            };
            let rel_err_pk = if fd_dvpk.abs() > 1e-15 {
                (jac[1] - fd_dvpk).abs() / fd_dvpk.abs()
            } else {
                jac[1].abs()
            };

            assert!(
                rel_err_gk < 1e-4,
                "dIp/dVgk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                vpk,
                jac[0],
                fd_dvgk,
                rel_err_gk
            );
            assert!(
                rel_err_pk < 1e-4,
                "dIp/dVpk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                vpk,
                jac[1],
                fd_dvpk,
                rel_err_pk
            );
        }
    }

    #[test]
    fn test_pentode_plate_current() {
        let pent = KorenPentode::el84();
        let ip = pent.current(&[0.0, 250.0, 250.0]);
        assert!(
            ip > 1e-3,
            "EL84 Ip should be in mA range, got {:.3}mA",
            ip * 1000.0
        );

        let ip_100 = pent.current(&[0.0, 100.0, 250.0]);
        let ip_250 = pent.current(&[0.0, 250.0, 250.0]);
        assert!(ip_250 >= ip_100 * 0.95);

        let ip_neg = pent.current(&[-5.0, 250.0, 250.0]);
        assert!(ip_neg < ip);
    }

    #[test]
    fn test_pentode_jacobian_finite_difference() {
        let pent = KorenPentode::el84();
        let eps = 1e-6;

        let v = [0.0, 250.0, 250.0];
        let jac = pent.jacobian(&v);

        for dim in 0..3 {
            let mut v_plus = v;
            let mut v_minus = v;
            v_plus[dim] += eps;
            v_minus[dim] -= eps;
            let fd = (pent.current(&v_plus) - pent.current(&v_minus)) / (2.0 * eps);
            let rel_err = if fd.abs() > 1e-15 {
                (jac[dim] - fd).abs() / fd.abs()
            } else {
                jac[dim].abs()
            };
            assert!(
                rel_err < 1e-3,
                "Pentode Jacobian[{}] mismatch: analytic={:.6e} fd={:.6e} err={:.2e}",
                dim,
                jac[dim],
                fd,
                rel_err
            );
        }
    }

    #[test]
    fn test_grid_current_model() {
        let tube = KorenTriode::ecc83();
        assert_eq!(tube.grid_current(-1.0), 0.0);
        assert_eq!(tube.grid_current(0.0), 0.0);

        let ig = tube.grid_current(0.5);
        assert!(ig > 0.0);
        assert!((ig - tube.ig_max).abs() < 1e-10);

        let ig2 = tube.grid_current(1.0);
        assert!(ig2 > ig);
    }

    #[test]
    fn test_grid_current_jacobian_finite_difference() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-7;

        for &vgk in &[0.1, 0.3, 0.5, 1.0, 2.0] {
            let jac = tube.grid_current_jacobian(vgk);
            let fd = (tube.grid_current(vgk + eps) - tube.grid_current(vgk - eps)) / (2.0 * eps);
            let rel_err = if fd.abs() > 1e-15 {
                (jac - fd).abs() / fd.abs()
            } else {
                jac.abs()
            };
            assert!(
                rel_err < 1e-4,
                "dIg/dVgk mismatch at Vgk={}: analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                jac,
                fd,
                rel_err
            );
        }
        assert_eq!(tube.grid_current_jacobian(0.0), 0.0);
        assert_eq!(tube.grid_current_jacobian(-1.0), 0.0);
    }

    /// Verify Jacobian is bounded for fractional exponents (ex < 1.0).
    /// Previously e1.powf(ex-1.0) produced huge values near the guard threshold.
    #[test]
    fn test_fractional_exponent_jacobian_bounded() {
        // ex=0.5 means e1^(ex-1) = e1^(-0.5) which diverges for small e1
        let tube = KorenTriode::new(100.0, 0.5, 1060.0, 600.0, 300.0);

        // Test at low Vpk where e1 is very small
        for &vpk in &[0.1, 1.0, 5.0, 10.0] {
            let jac = tube.jacobian(&[-3.0, vpk]);
            assert!(jac[0].is_finite(), "dIp/dVgk must be finite at vpk={}", vpk);
            assert!(jac[1].is_finite(), "dIp/dVpk must be finite at vpk={}", vpk);
            assert!(
                jac[0].abs() < 1e6,
                "dIp/dVgk must be bounded at vpk={}, got {:.2e}",
                vpk,
                jac[0]
            );
            assert!(
                jac[1].abs() < 1e6,
                "dIp/dVpk must be bounded at vpk={}, got {:.2e}",
                vpk,
                jac[1]
            );
        }

        // Normal operating point should still work
        let jac = tube.jacobian(&[0.0, 250.0]);
        assert!(jac[0] > 0.0, "Transconductance should be positive");
        assert!(jac[1] > 0.0, "Output conductance should be positive");
    }

    /// Verify lambda multiplier increases plate current proportional to Vpk.
    #[test]
    fn test_lambda_early_effect() {
        let tube_no_lambda = KorenTriode::ecc83();
        let tube_with_lambda =
            KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, 0.001);

        let vgk = -1.0;
        let vpk = 250.0;
        let ip_without = tube_no_lambda.plate_current(vgk, vpk);
        let ip_with = tube_with_lambda.plate_current(vgk, vpk);

        let expected_ratio = 1.0 + 0.001 * vpk;
        let actual_ratio = ip_with / ip_without;
        assert!(
            (actual_ratio - expected_ratio).abs() < 1e-10,
            "Lambda should multiply by (1+lambda*Vpk): ratio={:.6e}, expected={:.6e}",
            actual_ratio,
            expected_ratio
        );
    }

    /// Verify lambda=0.0 is backward compatible.
    #[test]
    fn test_lambda_zero_backward_compat() {
        let tube_old = KorenTriode::new(100.0, 1.4, 1060.0, 600.0, 300.0);
        let tube_new =
            KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, 0.0);

        for &(vgk, vpk) in &[(0.0, 250.0), (-1.0, 200.0), (-2.0, 100.0)] {
            let ip_old = tube_old.plate_current(vgk, vpk);
            let ip_new = tube_new.plate_current(vgk, vpk);
            assert_eq!(
                ip_old, ip_new,
                "lambda=0.0 should be backward compatible at ({}, {})",
                vgk, vpk
            );
        }
    }

    /// Verify plate current and Jacobian with positive grid voltage (grid current region).
    #[test]
    fn test_triode_grid_current_region_fd_jacobian() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-7;

        // Grid current region: Vgk > 0
        let grid_positive_points: &[(f64, f64, &str)] = &[
            (0.5, 250.0, "Vgk=0.5 moderate grid current"),
            (1.0, 250.0, "Vgk=1.0 strong grid current"),
            (2.0, 200.0, "Vgk=2.0 heavy grid current"),
            (0.1, 300.0, "Vgk=0.1 onset of grid current"),
        ];

        for &(vgk, vpk, desc) in grid_positive_points {
            // Check plate current Jacobian
            let jac = tube.jacobian(&[vgk, vpk]);

            let fd_dvgk = (tube.plate_current(vgk + eps, vpk)
                - tube.plate_current(vgk - eps, vpk))
                / (2.0 * eps);
            let fd_dvpk = (tube.plate_current(vgk, vpk + eps)
                - tube.plate_current(vgk, vpk - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dIp/dVgk", jac[0], fd_dvgk),
                ("dIp/dVpk", jac[1], fd_dvpk),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "Grid current region {} at {} (Vgk={}, Vpk={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name, desc, vgk, vpk, analytic, fd, rel_err
                );
            }

            // Also check grid current Jacobian
            let ig_jac = tube.grid_current_jacobian(vgk);
            let ig_fd = (tube.grid_current(vgk + eps) - tube.grid_current(vgk - eps))
                / (2.0 * eps);
            let ig_err = if ig_fd.abs() > 1e-15 {
                (ig_jac - ig_fd).abs() / ig_fd.abs()
            } else {
                ig_jac.abs()
            };
            assert!(
                ig_err < 0.01,
                "Grid Ig Jacobian at {} (Vgk={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                desc, vgk, ig_jac, ig_fd, ig_err
            );

            // Grid current should be positive for Vgk > 0
            let ig = tube.grid_current(vgk);
            assert!(
                ig > 0.0,
                "Grid current should be positive at Vgk={}: {:.6e}",
                vgk,
                ig
            );
        }
    }

    /// Verify Jacobian at low plate voltage where E1 is small.
    #[test]
    fn test_triode_low_plate_voltage_fd_jacobian() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-7;

        let low_vpk_points: &[(f64, f64, &str)] = &[
            (0.0, 5.0, "Vgk=0 Vpk=5"),
            (0.0, 10.0, "Vgk=0 Vpk=10"),
            (-1.0, 5.0, "Vgk=-1 Vpk=5"),
            (-1.0, 20.0, "Vgk=-1 Vpk=20"),
            (0.0, 1.0, "Vgk=0 Vpk=1"),
        ];

        for &(vgk, vpk, desc) in low_vpk_points {
            let jac = tube.jacobian(&[vgk, vpk]);

            // Both Jacobian entries must be finite
            assert!(
                jac[0].is_finite(),
                "dIp/dVgk must be finite at {} (Vgk={}, Vpk={}): {}",
                desc, vgk, vpk, jac[0]
            );
            assert!(
                jac[1].is_finite(),
                "dIp/dVpk must be finite at {} (Vgk={}, Vpk={}): {}",
                desc, vgk, vpk, jac[1]
            );

            let fd_dvgk = (tube.plate_current(vgk + eps, vpk)
                - tube.plate_current(vgk - eps, vpk))
                / (2.0 * eps);
            let fd_dvpk = (tube.plate_current(vgk, vpk + eps)
                - tube.plate_current(vgk, vpk - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dIp/dVgk", jac[0], fd_dvgk),
                ("dIp/dVpk", jac[1], fd_dvpk),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "Low Vpk {} at {} (Vgk={}, Vpk={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name, desc, vgk, vpk, analytic, fd, rel_err
                );
            }
        }
    }

    /// Verify Jacobian near cutoff where plate current approaches zero.
    #[test]
    fn test_triode_near_cutoff_fd_jacobian() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-7;

        // Near-cutoff: very negative Vgk where Ip is very small but nonzero
        let cutoff_points: &[(f64, f64, &str)] = &[
            (-3.0, 250.0, "Vgk=-3 near cutoff"),
            (-3.5, 250.0, "Vgk=-3.5 deep cutoff"),
            (-2.5, 150.0, "Vgk=-2.5 low Vpk"),
            (-4.0, 300.0, "Vgk=-4 very deep cutoff"),
        ];

        for &(vgk, vpk, desc) in cutoff_points {
            let ip = tube.plate_current(vgk, vpk);
            let jac = tube.jacobian(&[vgk, vpk]);

            // Plate current should be very small but non-negative
            assert!(
                ip >= 0.0,
                "Ip should be non-negative near cutoff at {}: {:.6e}",
                desc,
                ip
            );
            assert!(ip.is_finite(), "Ip must be finite at {}", desc);

            // Jacobian must be finite
            assert!(
                jac[0].is_finite(),
                "dIp/dVgk must be finite at {}: {}",
                desc,
                jac[0]
            );
            assert!(
                jac[1].is_finite(),
                "dIp/dVpk must be finite at {}: {}",
                desc,
                jac[1]
            );

            let fd_dvgk = (tube.plate_current(vgk + eps, vpk)
                - tube.plate_current(vgk - eps, vpk))
                / (2.0 * eps);
            let fd_dvpk = (tube.plate_current(vgk, vpk + eps)
                - tube.plate_current(vgk, vpk - eps))
                / (2.0 * eps);

            for (name, analytic, fd) in [
                ("dIp/dVgk", jac[0], fd_dvgk),
                ("dIp/dVpk", jac[1], fd_dvpk),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 0.01,
                    "Near-cutoff {} at {} (Vgk={}, Vpk={}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name, desc, vgk, vpk, analytic, fd, rel_err
                );
            }
        }
    }

    /// Verify Jacobian with lambda against finite differences.
    #[test]
    fn test_jacobian_with_lambda_finite_difference() {
        let tube = KorenTriode::with_all_params(100.0, 1.4, 1060.0, 600.0, 300.0, 2e-3, 0.5, 0.001);
        let eps = 1e-6;

        for &(vgk, vpk) in &[(0.0, 250.0), (-1.0, 200.0), (-2.0, 100.0), (-0.5, 300.0)] {
            let jac = tube.jacobian(&[vgk, vpk]);

            let ip_plus = tube.plate_current(vgk + eps, vpk);
            let ip_minus = tube.plate_current(vgk - eps, vpk);
            let fd_dvgk = (ip_plus - ip_minus) / (2.0 * eps);

            let ip_plus = tube.plate_current(vgk, vpk + eps);
            let ip_minus = tube.plate_current(vgk, vpk - eps);
            let fd_dvpk = (ip_plus - ip_minus) / (2.0 * eps);

            let rel_err_gk = if fd_dvgk.abs() > 1e-15 {
                (jac[0] - fd_dvgk).abs() / fd_dvgk.abs()
            } else {
                jac[0].abs()
            };
            let rel_err_pk = if fd_dvpk.abs() > 1e-15 {
                (jac[1] - fd_dvpk).abs() / fd_dvpk.abs()
            } else {
                jac[1].abs()
            };

            assert!(
                rel_err_gk < 1e-4,
                "dIp/dVgk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                vpk,
                jac[0],
                fd_dvgk,
                rel_err_gk
            );
            assert!(
                rel_err_pk < 1e-4,
                "dIp/dVpk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk,
                vpk,
                jac[1],
                fd_dvpk,
                rel_err_pk
            );
        }
    }
}
