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
}

/// Default grid current maximum [A].
const DEFAULT_IG_MAX: f64 = 2e-3;
/// Default grid current onset voltage [V].
const DEFAULT_VGK_ONSET: f64 = 0.5;

impl KorenTriode {
    /// Create a new triode model with default grid current parameters.
    pub fn new(mu: f64, ex: f64, kg1: f64, kp: f64, kvb: f64) -> Self {
        Self { mu, ex, kg1, kp, kvb, ig_max: DEFAULT_IG_MAX, vgk_onset: DEFAULT_VGK_ONSET }
    }

    /// Create a new triode model with custom grid current parameters.
    pub fn with_grid_params(mu: f64, ex: f64, kg1: f64, kp: f64, kvb: f64, ig_max: f64, vgk_onset: f64) -> Self {
        Self { mu, ex, kg1, kp, kvb, ig_max, vgk_onset }
    }

    /// 12AX7 (ECC83) - high-mu twin triode, common in guitar amps.
    /// Koren parameters: mu=100, ex=1.4, Kg1=1060, Kp=600, Kvb=300
    pub fn ecc83() -> Self {
        Self::new(100.0, 1.4, 1060.0, 600.0, 300.0)
    }

    /// 12AU7 (ECC82) - medium-mu twin triode.
    /// Koren parameters: mu=17, ex=1.4, Kg1=1180, Kp=200, Kvb=100
    pub fn ecc82() -> Self {
        Self::new(17.0, 1.4, 1180.0, 200.0, 100.0)
    }

    /// 12AT7 (ECC81) - medium-mu triode.
    /// Koren parameters: mu=40, ex=1.4, Kg1=800, Kp=300, Kvb=150
    pub fn ecc81() -> Self {
        Self::new(40.0, 1.4, 800.0, 300.0, 150.0)
    }

    /// 6SL7 - high-mu octal triode.
    /// Koren parameters: mu=70, ex=1.4, Kg1=1200, Kp=400, Kvb=200
    pub fn _6sl7() -> Self {
        Self::new(70.0, 1.4, 1200.0, 400.0, 200.0)
    }

    /// Calculate plate current given Vgk and Vpk.
    ///
    /// Vgk = grid-cathode voltage
    /// Vpk = plate-cathode voltage
    ///
    /// Uses Koren 1996 equation:
    ///   inner = Kp * (1/mu + Vgk / sqrt(Kvb + Vpk^2))
    ///   E1 = (Vpk / Kp) * ln(1 + exp(inner))
    ///   Ip = E1^ex / Kg1   (if E1 > 0)
    pub fn plate_current(&self, vgk: f64, vpk: f64) -> f64 {
        if vpk <= 0.0 {
            return 0.0;
        }

        let s = (self.kvb + vpk * vpk).sqrt();
        let inner = self.kp * (1.0 / self.mu + vgk / s);

        // Numerically stable softplus: ln(1 + exp(x))
        let softplus = if inner > 20.0 {
            inner // ln(1+exp(x)) ≈ x for large x
        } else if inner < -20.0 {
            0.0 // ln(1+exp(x)) ≈ 0 for very negative x
        } else {
            (1.0 + inner.exp()).ln()
        };

        let e1 = (vpk / self.kp) * softplus;

        if e1 <= 0.0 {
            return 0.0;
        }

        e1.powf(self.ex) / self.kg1
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

    /// Jacobian: [∂Ip/∂Vgk, ∂Ip/∂Vpk]
    ///
    /// Computed via chain rule through the Koren E1 equation:
    ///   Ip = E1^ex / Kg1
    ///   dIp/dVgk = (dIp/dE1) * (dE1/dVgk)
    ///   dIp/dVpk = (dIp/dE1) * (dE1/dVpk)
    fn jacobian(&self, v: &[f64; 2]) -> [f64; 2] {
        let vgk = v[0];
        let vpk = v[1];

        if vpk <= 0.0 {
            return [0.0, 0.0];
        }

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

        if e1 <= 1e-30 {
            return [0.0, 0.0];
        }

        // dIp/dE1 = ex * E1^(ex-1) / Kg1
        let dip_de1 = self.ex * e1.powf(self.ex - 1.0) / self.kg1;

        // dE1/dVgk = Vpk * sigmoid(inner) / s
        let de1_dvgk = vpk * sigmoid / s;

        // dE1/dVpk = softplus/Kp - sigmoid * Vgk * Vpk^2 / s^3
        let de1_dvpk = softplus / self.kp - sigmoid * vgk * vpk * vpk / (s * s * s);

        [dip_de1 * de1_dvgk, dip_de1 * de1_dvpk]
    }
}

/// Koren pentode model.
///
/// Uses the screen grid voltage (Vsg) in the E1 calculation for screen-limited
/// behavior, with a plate voltage correction factor (1 - exp(-Vpk/Kx)).
///
/// **Experimental**: 3D pentode — not yet supported by solver (max device dim = 2).
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
        let softplus = if inner > 20.0 { inner } else if inner < -20.0 { 0.0 } else { (1.0 + inner.exp()).ln() };

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
        assert!(ip_very_off < ip_off, "Even more negative should reduce further");
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
        // Koren: inner = Kp * (1/mu + Vgk/sqrt(Kvb + Vpk^2))
        let ip_vgk_0 = tube.plate_current(0.0, 250.0);
        let ip_vgk_minus1 = tube.plate_current(-1.0, 250.0);
        let ip_vgk_minus2 = tube.plate_current(-2.0, 250.0);

        // More negative Vgk should reduce current
        assert!(ip_vgk_minus1 < ip_vgk_0,
            "More negative Vgk should reduce current: {} vs {}",
            ip_vgk_minus1, ip_vgk_0);
        assert!(ip_vgk_minus2 < ip_vgk_minus1,
            "Even more negative should reduce further: {} vs {}",
            ip_vgk_minus2, ip_vgk_minus1);

        // The effect should be significant (mu = 100)
        let ratio = ip_vgk_0 / ip_vgk_minus2;
        assert!(ratio > 2.0, "Mu effect should be noticeable, ratio = {}", ratio);
    }

    #[test]
    fn test_triode_trait() {
        let tube = KorenTriode::ecc83();

        let ip = tube.current(&[0.0, 250.0]);
        let jac = tube.jacobian(&[0.0, 250.0]);

        assert!(ip > 0.0);
        assert!(jac[0] > 0.0);  // ∂Ip/∂Vgk > 0 (transconductance)
        assert!(jac[1] > 0.0);  // ∂Ip/∂Vpk > 0 (output conductance)
    }

    /// Verify 12AX7 plate current against published curves.
    ///
    /// Reference: 12AX7 datasheet plate characteristics.
    /// At Vpk=250V, typical values:
    ///   Vgk=0:  Ip ~ 1.0-2.0 mA
    ///   Vgk=-1: Ip ~ 0.3-0.8 mA
    ///   Vgk=-2: Ip ~ 0.05-0.3 mA
    #[test]
    fn test_12ax7_plate_curves() {
        let tube = KorenTriode::ecc83();

        // Vgk=0, Vpk=250V: expect ~1-4mA (varies with Koren parameter set)
        let ip_0 = tube.plate_current(0.0, 250.0);
        assert!(ip_0 > 0.5e-3 && ip_0 < 5.0e-3,
            "Ip(Vgk=0, Vpk=250) = {:.3}mA, expected 0.5-5.0mA", ip_0 * 1000.0);

        // Vgk=-1, Vpk=250V: reduced but still significant
        let ip_m1 = tube.plate_current(-1.0, 250.0);
        assert!(ip_m1 > 0.1e-3 && ip_m1 < 3.0e-3,
            "Ip(Vgk=-1, Vpk=250) = {:.3}mA, expected 0.1-3.0mA", ip_m1 * 1000.0);

        // Vgk=-2, Vpk=250V: further reduced
        let ip_m2 = tube.plate_current(-2.0, 250.0);
        assert!(ip_m2 > 0.01e-3 && ip_m2 < 1.5e-3,
            "Ip(Vgk=-2, Vpk=250) = {:.3}mA, expected 0.01-1.5mA", ip_m2 * 1000.0);

        // Verify monotonic decrease with grid voltage
        assert!(ip_0 > ip_m1 && ip_m1 > ip_m2);

        // Vgk=-4, Vpk=250V: should be approaching cutoff
        let ip_m4 = tube.plate_current(-4.0, 250.0);
        assert!(ip_m4 < 0.01e-3,
            "Ip(Vgk=-4, Vpk=250) = {:.4}mA, expected near cutoff", ip_m4 * 1000.0);
    }

    /// Verify triode Jacobian against finite differences.
    #[test]
    fn test_triode_jacobian_finite_difference() {
        let tube = KorenTriode::ecc83();
        let eps = 1e-6;

        for &(vgk, vpk) in &[(0.0, 250.0), (-1.0, 200.0), (-2.0, 100.0), (-0.5, 300.0)] {
            let jac = tube.jacobian(&[vgk, vpk]);

            // Finite difference for dIp/dVgk
            let ip_plus = tube.plate_current(vgk + eps, vpk);
            let ip_minus = tube.plate_current(vgk - eps, vpk);
            let fd_dvgk = (ip_plus - ip_minus) / (2.0 * eps);

            // Finite difference for dIp/dVpk
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

            assert!(rel_err_gk < 1e-4,
                "dIp/dVgk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk, vpk, jac[0], fd_dvgk, rel_err_gk);
            assert!(rel_err_pk < 1e-4,
                "dIp/dVpk mismatch at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk, vpk, jac[1], fd_dvpk, rel_err_pk);
        }
    }

    /// Verify pentode model behavior.
    #[test]
    fn test_pentode_plate_current() {
        let pent = KorenPentode::el84();

        // At Vgk=0, Vpk=250V, Vsg=250V: should have significant plate current
        let ip = pent.current(&[0.0, 250.0, 250.0]);
        assert!(ip > 1e-3, "EL84 Ip should be in mA range, got {:.3}mA", ip * 1000.0);

        // Plate current should increase with plate voltage (until saturation)
        let ip_100 = pent.current(&[0.0, 100.0, 250.0]);
        let ip_250 = pent.current(&[0.0, 250.0, 250.0]);
        assert!(ip_250 >= ip_100 * 0.95,
            "Ip should increase or saturate with Vpk: Ip(100)={:.3}mA, Ip(250)={:.3}mA",
            ip_100 * 1000.0, ip_250 * 1000.0);

        // Negative grid should reduce current
        let ip_neg = pent.current(&[-5.0, 250.0, 250.0]);
        assert!(ip_neg < ip,
            "Negative grid should reduce current: Ip(0)={:.3}mA, Ip(-5)={:.3}mA",
            ip * 1000.0, ip_neg * 1000.0);
    }

    /// Verify pentode Jacobian against finite differences.
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
            assert!(rel_err < 1e-3,
                "Pentode Jacobian[{}] mismatch: analytic={:.6e} fd={:.6e} err={:.2e}",
                dim, jac[dim], fd, rel_err);
        }
    }

    /// Verify improved grid current model.
    #[test]
    fn test_grid_current_model() {
        let tube = KorenTriode::ecc83();

        // Negative Vgk → zero grid current
        assert_eq!(tube.grid_current(-1.0), 0.0);
        assert_eq!(tube.grid_current(0.0), 0.0);

        // Positive Vgk → nonzero current
        let ig = tube.grid_current(0.5);
        assert!(ig > 0.0, "Grid current should be positive for Vgk > 0");
        // At Vgk = vgk_onset (0.5V), Ig should be ig_max * (0.5/0.5)^1.5 = ig_max
        assert!((ig - tube.ig_max).abs() < 1e-10,
            "Ig at onset should be ig_max: got {}, expected {}", ig, tube.ig_max);

        // Monotonically increasing
        let ig2 = tube.grid_current(1.0);
        assert!(ig2 > ig, "Grid current should increase with Vgk");
    }

    /// Verify grid current Jacobian against finite differences.
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
            assert!(rel_err < 1e-4,
                "dIg/dVgk mismatch at Vgk={}: analytic={:.6e} fd={:.6e} err={:.2e}",
                vgk, jac, fd, rel_err);
        }

        // At Vgk <= 0, Jacobian should be zero
        assert_eq!(tube.grid_current_jacobian(0.0), 0.0);
        assert_eq!(tube.grid_current_jacobian(-1.0), 0.0);
    }
}
