//! Bipolar Junction Transistor (BJT) models.
//!
//! Includes:
//! - Ebers-Moll model (basic, good for most audio applications)
//! - Gummel-Poon model (extended with Early effect and high-level injection)

use crate::{NonlinearDevice, VT_ROOM, safeguards};

/// BJT polarity (NPN or PNP).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BjtPolarity {
    Npn,
    Pnp,
}

/// Ebers-Moll BJT model (transport version).
///
/// This is the simplified transport version which is numerically
/// better behaved than the injection version.
///
/// Terminal voltages: v[0] = Vbe (base-emitter), v[1] = Vbc (base-collector)
/// Current returned: Collector current (positive into collector for NPN)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BjtEbersMoll {
    /// Saturation current [A]
    pub is: f64,
    /// Thermal voltage [V]
    pub vt: f64,
    /// Forward current gain (beta_f)
    pub beta_f: f64,
    /// Reverse current gain (beta_r)
    pub beta_r: f64,
    /// Polarity (NPN or PNP)
    pub polarity: BjtPolarity,
    /// Precomputed: 1 / beta_f
    inv_beta_f: f64,
    /// Precomputed: 1 / beta_r
    inv_beta_r: f64,
}

impl BjtEbersMoll {
    /// Create a new BJT model.
    pub fn new(is: f64, vt: f64, beta_f: f64, beta_r: f64, polarity: BjtPolarity) -> Self {
        Self {
            is,
            vt,
            beta_f,
            beta_r,
            polarity,
            inv_beta_f: 1.0 / beta_f,
            inv_beta_r: 1.0 / beta_r,
        }
    }

    /// Create at room temperature.
    pub fn new_room_temp(is: f64, beta_f: f64, beta_r: f64, polarity: BjtPolarity) -> Self {
        Self::new(is, VT_ROOM, beta_f, beta_r, polarity)
    }

    /// 2N2222A NPN transistor (common general-purpose).
    pub fn npn_2n2222a() -> Self {
        Self::new_room_temp(1.26e-14, 200.0, 3.0, BjtPolarity::Npn)
    }

    /// 2N3904 NPN transistor.
    pub fn npn_2n3904() -> Self {
        Self::new_room_temp(6.73e-15, 416.0, 0.737, BjtPolarity::Npn)
    }

    /// 2N3906 PNP transistor.
    pub fn pnp_2n3906() -> Self {
        Self::new_room_temp(1.27e-14, 207.0, 1.68, BjtPolarity::Pnp)
    }

    /// AC128 Germanium PNP (vintage fuzz tones).
    pub fn pnp_ac128() -> Self {
        // Germanium has much higher Is
        Self::new_room_temp(1e-6, 70.0, 3.0, BjtPolarity::Pnp)
    }

    /// Get the sign multiplier for polarity.
    pub fn sign(&self) -> f64 {
        match self.polarity {
            BjtPolarity::Npn => 1.0,
            BjtPolarity::Pnp => -1.0,
        }
    }

    /// Calculate collector current given Vbe and Vbc.
    ///
    /// Returns positive current into collector for NPN.
    pub fn collector_current(&self, vbe: f64, vbc: f64) -> f64 {
        let s = self.sign();
        
        // Apply polarity to voltages
        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        // Transport current
        let exp_be = safeguards::safe_exp(vbe_eff / self.vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / self.vt);
        let i_cc = self.is * (exp_be - exp_bc);

        // Collector current: Ic = I_cc - Is/βr * (exp(Vbc/Vt) - 1)
        let ic = i_cc - self.is * self.inv_beta_r * (exp_bc - 1.0);
        
        s * ic  // Apply polarity to current
    }

    /// Calculate base current given Vbe and Vbc.
    pub fn base_current(&self, vbe: f64, vbc: f64) -> f64 {
        let s = self.sign();
        
        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        let exp_be = safeguards::safe_exp(vbe_eff / self.vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / self.vt);

        // Base current
        let ib = self.is * self.inv_beta_f * (exp_be - 1.0)
               + self.is * self.inv_beta_r * (exp_bc - 1.0);
        
        s * ib
    }

    /// Partial derivative of base current with respect to Vbe.
    pub fn base_current_jacobian_dvbe(&self, vbe: f64, _vbc: f64) -> f64 {
        let s = self.sign();
        let vbe_eff = s * vbe;
        let exp_be = safeguards::safe_exp(vbe_eff / self.vt);

        // ∂Ib/∂Vbe = (Is/(βf*Vt)) * exp(Vbe/Vt)
        let dib_dvbe = self.is * self.inv_beta_f / self.vt * exp_be;
        
        // Apply chain rule for polarity (s * s = 1, so no sign change)
        s * s * dib_dvbe
    }

    /// Partial derivative of base current with respect to Vbc.
    pub fn base_current_jacobian_dvbc(&self, _vbe: f64, vbc: f64) -> f64 {
        let s = self.sign();
        let vbc_eff = s * vbc;
        let exp_bc = safeguards::safe_exp(vbc_eff / self.vt);

        // ∂Ib/∂Vbc = (Is/(βr*Vt)) * exp(Vbc/Vt)
        let dib_dvbc = self.is * self.inv_beta_r / self.vt * exp_bc;
        
        // Apply chain rule for polarity
        s * s * dib_dvbc
    }

    /// Calculate emitter current given Vbe and Vbc.
    pub fn emitter_current(&self, vbe: f64, vbc: f64) -> f64 {
        -(self.collector_current(vbe, vbc) + self.base_current(vbe, vbc))
    }

    /// Partial derivatives for Jacobian.
    ///
    /// Returns (∂Ic/∂Vbe, ∂Ic/∂Vbc)
    pub fn collector_jacobian(&self, vbe: f64, vbc: f64) -> (f64, f64) {
        let s = self.sign();
        
        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        let exp_be = safeguards::safe_exp(vbe_eff / self.vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / self.vt);

        // ∂Ic/∂Vbe = (Is/Vt) * exp(Vbe/Vt)
        let d_ic_d_vbe = (self.is / self.vt) * exp_be;

        // ∂Ic/∂Vbc = -(Is/Vt) * exp(Vbc/Vt) - (Is/(βr*Vt)) * exp(Vbc/Vt)
        let d_ic_d_vbc = -(self.is / self.vt) * exp_bc
                        - (self.is * self.inv_beta_r / self.vt) * exp_bc;

        // Apply chain rule for polarity
        (s * s * d_ic_d_vbe, s * s * d_ic_d_vbc)
    }
}

impl NonlinearDevice<2> for BjtEbersMoll {
    /// Current as a function of [Vbe, Vbc].
    ///
    /// Returns collector current (positive into collector for NPN).
    fn current(&self, v: &[f64; 2]) -> f64 {
        self.collector_current(v[0], v[1])
    }

    /// Jacobian: [∂Ic/∂Vbe, ∂Ic/∂Vbc].
    fn jacobian(&self, v: &[f64; 2]) -> [f64; 2] {
        let (d_ic_d_vbe, d_ic_d_vbc) = self.collector_jacobian(v[0], v[1]);
        [d_ic_d_vbe, d_ic_d_vbc]
    }
}

/// Gummel-Poon BJT model (extended).
///
/// Adds:
/// - Early effect (base-width modulation)
/// - High-level injection
/// - More accurate for high currents
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BjtGummelPoon {
    /// Base Ebers-Moll model
    pub base: BjtEbersMoll,
    /// Forward Early voltage [V]
    pub vaf: f64,
    /// Reverse Early voltage [V]
    pub var: f64,
    /// High-current knee current [A]
    pub ikf: f64,
    /// Reverse high-current knee current [A]
    pub ikr: f64,
}

impl BjtGummelPoon {
    /// Create a new Gummel-Poon model.
    pub fn new(base: BjtEbersMoll, vaf: f64, var: f64, ikf: f64, ikr: f64) -> Self {
        Self { base, vaf, var, ikf, ikr }
    }

    /// 2N2222A with typical Gummel-Poon parameters.
    pub fn npn_2n2222a() -> Self {
        Self::new(
            BjtEbersMoll::npn_2n2222a(),
            100.0,   // VAF
            10.0,    // VAR
            0.3,     // IKF
            0.006,   // IKR
        )
    }

    /// Base charge factor qb (accounts for Early effect and high injection).
    fn qb(&self, vbe: f64, vbc: f64) -> f64 {
        let s = self.base.sign();
        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        // Early effect
        let q1_denom = 1.0 - vbe_eff / self.var - vbc_eff / self.vaf;
        if q1_denom.abs() < 1e-30 {
            return 1.0;
        }
        let q1 = 1.0 / q1_denom;

        // High-level injection
        let i_cc = self.base.is * ((vbe_eff / self.base.vt).exp() - (vbc_eff / self.base.vt).exp());
        let q2 = i_cc / self.ikf;

        q1 * (1.0 + (1.0 + 4.0 * q2).sqrt()) / 2.0
    }

    /// Collector current with Early effect and high injection.
    pub fn collector_current(&self, vbe: f64, vbc: f64) -> f64 {
        let s = self.base.sign();
        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        let exp_be = safeguards::safe_exp(vbe_eff / self.base.vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / self.base.vt);

        let i_cc = self.base.is * (exp_be - exp_bc);
        let qb = self.qb(vbe, vbc);

        let ic = i_cc / qb - self.base.is * self.base.inv_beta_r * (exp_bc - 1.0);
        
        s * ic
    }
}

impl NonlinearDevice<2> for BjtGummelPoon {
    fn current(&self, v: &[f64; 2]) -> f64 {
        self.collector_current(v[0], v[1])
    }

    fn jacobian(&self, v: &[f64; 2]) -> [f64; 2] {
        let s = self.base.sign();
        let vbe_eff = s * v[0];
        let vbc_eff = s * v[1];
        let vt = self.base.vt;
        let is = self.base.is;

        let exp_be = safeguards::safe_exp(vbe_eff / vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / vt);

        // Transport current and derivatives (in effective voltage space)
        let icc = is * (exp_be - exp_bc);
        let dicc_dvbe = is / vt * exp_be;
        let dicc_dvbc = -is / vt * exp_bc;

        // Base charge factor q1
        let q1_denom = 1.0 - vbe_eff / self.var - vbc_eff / self.vaf;
        let q1 = if q1_denom.abs() > 1e-15 { 1.0 / q1_denom } else { 1e15_f64.copysign(q1_denom) };
        let dq1_dvbe = q1 * q1 / self.var;
        let dq1_dvbc = q1 * q1 / self.vaf;

        // High injection q2
        let q2 = icc / self.ikf;
        let dq2_dvbe = dicc_dvbe / self.ikf;
        let dq2_dvbc = dicc_dvbc / self.ikf;

        // Discriminant D = sqrt(1 + 4*q2)
        let disc = (1.0 + 4.0 * q2).max(0.0);
        let d = disc.sqrt();
        let dd_dvbe = if d > 1e-15 { 2.0 * dq2_dvbe / d } else { 0.0 };
        let dd_dvbc = if d > 1e-15 { 2.0 * dq2_dvbc / d } else { 0.0 };

        // qb = q1 * (1 + D) / 2
        let qb = q1 * (1.0 + d) / 2.0;
        let dqb_dvbe = dq1_dvbe * (1.0 + d) / 2.0 + q1 * dd_dvbe / 2.0;
        let dqb_dvbc = dq1_dvbc * (1.0 + d) / 2.0 + q1 * dd_dvbc / 2.0;

        // Ic_eff = Icc/qb - Is/βr * (exp(Vbc_eff/Vt) - 1)
        // d(Icc/qb)/dV = (dIcc/dV * qb - Icc * dqb/dV) / qb^2  (quotient rule)
        let qb2 = qb * qb;
        let quotient_dvbe = if qb2 > 1e-30 {
            (dicc_dvbe * qb - icc * dqb_dvbe) / qb2
        } else {
            dicc_dvbe
        };
        let quotient_dvbc = if qb2 > 1e-30 {
            (dicc_dvbc * qb - icc * dqb_dvbc) / qb2
        } else {
            dicc_dvbc
        };

        let d_bc_term_dvbc = is * self.base.inv_beta_r / vt * exp_bc;

        // Polarity: dIc/dVbe = s * dIc_eff/dVbe_eff * s = dIc_eff/dVbe_eff (s² = 1)
        [quotient_dvbe, quotient_dvbc - d_bc_term_dvbc]
    }
}

/// BJT operating region classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BjtRegion {
    /// Cutoff: Vbe < ~0.5V, Vbc < ~0.5V
    Cutoff,
    /// Forward active: Vbe ≈ 0.6-0.7V, Vbc < ~0.5V (normal amplifier operation)
    ForwardActive,
    /// Reverse active: Vbe < ~0.5V, Vbc ≈ 0.6-0.7V
    ReverseActive,
    /// Saturation: Vbe ≈ 0.7V, Vbc ≈ 0.7V (switch fully on)
    Saturation,
}

/// Helper to classify BJT operating region.
pub fn classify_region(vbe: f64, vbc: f64, polarity: BjtPolarity) -> BjtRegion {
    let (vbe_eff, vbc_eff) = match polarity {
        BjtPolarity::Npn => (vbe, vbc),
        BjtPolarity::Pnp => (-vbe, -vbc),
    };

    const VBE_ON: f64 = 0.5;
    const VBC_ON: f64 = 0.5;

    match (vbe_eff > VBE_ON, vbc_eff > VBC_ON) {
        (false, false) => BjtRegion::Cutoff,
        (true, false) => BjtRegion::ForwardActive,
        (false, true) => BjtRegion::ReverseActive,
        (true, true) => BjtRegion::Saturation,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bjt_cutoff() {
        let bjt = BjtEbersMoll::npn_2n2222a();
        
        // In cutoff (Vbe = 0), currents should be near zero
        let ic = bjt.collector_current(0.0, -5.0);
        let ib = bjt.base_current(0.0, -5.0);
        
        assert!(ic.abs() < 1e-12, "Ic in cutoff: {}", ic);
        assert!(ib.abs() < 1e-12, "Ib in cutoff: {}", ib);
    }

    #[test]
    fn test_bjt_forward_active() {
        let bjt = BjtEbersMoll::npn_2n2222a();
        
        // Forward active: Vbe ≈ 0.7V, Vbc negative (Vce > 0)
        let vbe = 0.7;
        let vbc = -5.0;  // Vce = 5.7V
        
        let ic = bjt.collector_current(vbe, vbc);
        let ib = bjt.base_current(vbe, vbc);
        
        // Should have significant current
        assert!(ic > 0.0, "Ic should be positive in forward active");
        assert!(ib > 0.0, "Ib should be positive");
        
        // Current gain
        let beta = ic / ib;
        assert!(beta > 100.0 && beta < 300.0, "Beta = {}", beta);
    }

    #[test]
    fn test_bjt_polarity() {
        let npn = BjtEbersMoll::npn_2n2222a();
        let pnp = BjtEbersMoll::pnp_2n3906();
        
        // Same voltages should give opposite current signs
        let vbe = 0.7;
        let vbc = -5.0;
        
        let ic_npn = npn.collector_current(vbe, vbc);
        let ic_pnp = pnp.collector_current(-vbe, -vbc);  // Inverted for PNP
        
        assert!(ic_npn > 0.0);
        assert!(ic_pnp < 0.0);
        
        // Magnitudes should be similar
        assert!((ic_npn.abs() - ic_pnp.abs()).abs() / ic_npn.abs() < 0.5);
    }

    #[test]
    fn test_bjt_jacobian() {
        let bjt = BjtEbersMoll::npn_2n2222a();
        
        let vbe = 0.7;
        let vbc = -5.0;
        
        let (d_ic_d_vbe, d_ic_d_vbc) = bjt.collector_jacobian(vbe, vbc);
        
        // Transconductance should be positive and significant
        assert!(d_ic_d_vbe > 0.0, "gm should be positive");
        assert!(d_ic_d_vbe > 0.01, "gm should be significant");
        
        // Feedback term should be small (negative)
        assert!(d_ic_d_vbc < 0.0, "dIc/dVbc should be negative");
    }

    #[test]
    fn test_region_classification() {
        assert_eq!(classify_region(0.0, 0.0, BjtPolarity::Npn), BjtRegion::Cutoff);
        assert_eq!(classify_region(0.7, -5.0, BjtPolarity::Npn), BjtRegion::ForwardActive);
        assert_eq!(classify_region(0.7, 0.7, BjtPolarity::Npn), BjtRegion::Saturation);
    }

    #[test]
    fn test_gummel_poon() {
        let bjt = BjtGummelPoon::npn_2n2222a();

        let ic = bjt.collector_current(0.7, -5.0);
        assert!(ic.is_finite());
        assert!(ic > 0.0);
    }

    /// Verify BJT collector current against Ebers-Moll formula.
    ///
    /// For 2N2222A: Is=1.26e-14, beta_f=200, Vt=0.02585V
    /// At Vbe=0.7V, Vbc=-5V (forward active):
    ///   Ic ≈ Is * exp(Vbe/Vt) = 1.26e-14 * exp(27.07) ≈ several mA
    #[test]
    fn test_bjt_collector_current_value() {
        let bjt = BjtEbersMoll::npn_2n2222a();

        let ic = bjt.collector_current(0.7, -5.0);
        let ib = bjt.base_current(0.7, -5.0);

        // Ic should be in the mA range for Vbe=0.7V
        assert!(ic > 1e-4, "Ic = {:.3}mA, expected > 0.1mA", ic * 1000.0);
        assert!(ic < 0.1, "Ic = {:.3}mA, expected < 100mA", ic * 1000.0);

        // Current gain should be approximately beta_f
        let beta = ic / ib;
        assert!((beta - 200.0).abs() / 200.0 < 0.05,
            "Beta = {:.1}, expected ~200", beta);

        // KCL: Ie = -(Ic + Ib)
        let ie = -(ic + ib);
        assert!(ie < 0.0, "Emitter current should be negative (exits emitter for NPN)");
    }

    /// Verify BJT Jacobian against finite differences.
    #[test]
    fn test_bjt_jacobian_finite_difference() {
        let bjt = BjtEbersMoll::npn_2n2222a();
        let eps = 1e-7;

        for &(vbe, vbc) in &[(0.7, -5.0), (0.6, -2.0), (0.65, -10.0)] {
            let (dic_dvbe, dic_dvbc) = bjt.collector_jacobian(vbe, vbc);

            let ic_p = bjt.collector_current(vbe + eps, vbc);
            let ic_m = bjt.collector_current(vbe - eps, vbc);
            let fd_dvbe = (ic_p - ic_m) / (2.0 * eps);

            let ic_p = bjt.collector_current(vbe, vbc + eps);
            let ic_m = bjt.collector_current(vbe, vbc - eps);
            let fd_dvbc = (ic_p - ic_m) / (2.0 * eps);

            let rel_err_be = if fd_dvbe.abs() > 1e-15 {
                (dic_dvbe - fd_dvbe).abs() / fd_dvbe.abs()
            } else {
                dic_dvbe.abs()
            };
            let rel_err_bc = if fd_dvbc.abs() > 1e-15 {
                (dic_dvbc - fd_dvbc).abs() / fd_dvbc.abs()
            } else {
                dic_dvbc.abs()
            };

            assert!(rel_err_be < 1e-4,
                "dIc/dVbe at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vbe, vbc, dic_dvbe, fd_dvbe, rel_err_be);
            assert!(rel_err_bc < 1e-4,
                "dIc/dVbc at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vbe, vbc, dic_dvbc, fd_dvbc, rel_err_bc);
        }
    }

    /// Verify Gummel-Poon collector current values and Early effect.
    #[test]
    fn test_gummel_poon_early_effect() {
        let bjt = BjtGummelPoon::npn_2n2222a();

        // Forward active: Vbe=0.7V at different Vce (different Vbc)
        let ic_vce5 = bjt.collector_current(0.7, 0.7 - 5.0);   // Vce=5V
        let ic_vce10 = bjt.collector_current(0.7, 0.7 - 10.0);  // Vce=10V

        // With Early effect, Ic should increase slightly with Vce
        assert!(ic_vce10 > ic_vce5,
            "Early effect: Ic should increase with Vce. Ic(5V)={:.4}mA, Ic(10V)={:.4}mA",
            ic_vce5 * 1000.0, ic_vce10 * 1000.0);

        // The increase should be modest (not orders of magnitude)
        let ratio = ic_vce10 / ic_vce5;
        assert!(ratio < 1.5, "Early effect ratio should be modest: {:.3}", ratio);
    }

    /// Verify Gummel-Poon Jacobian against finite differences.
    #[test]
    fn test_gummel_poon_jacobian_finite_difference() {
        let bjt = BjtGummelPoon::npn_2n2222a();
        let eps = 1e-7;

        let vbe = 0.7_f64;
        let vbc = -5.0_f64;
        let jac = bjt.jacobian(&[vbe, vbc]);
        let dic_dvbe = jac[0];
        let dic_dvbc = jac[1];

        let ic_p = bjt.collector_current(vbe + eps, vbc);
        let ic_m = bjt.collector_current(vbe - eps, vbc);
        let fd_dvbe: f64 = (ic_p - ic_m) / (2.0 * eps);

        let ic_p = bjt.collector_current(vbe, vbc + eps);
        let ic_m = bjt.collector_current(vbe, vbc - eps);
        let fd_dvbc: f64 = (ic_p - ic_m) / (2.0 * eps);

        let rel_err_be = (dic_dvbe - fd_dvbe).abs() / fd_dvbe.abs();
        let rel_err_bc = if fd_dvbc.abs() > 1e-15 {
            (dic_dvbc - fd_dvbc).abs() / fd_dvbc.abs()
        } else {
            dic_dvbc.abs()
        };

        assert!(rel_err_be < 1e-3,
            "GP dIc/dVbe: analytic={:.6e} fd={:.6e} err={:.2e}",
            dic_dvbe, fd_dvbe, rel_err_be);
        assert!(rel_err_bc < 1e-3,
            "GP dIc/dVbc: analytic={:.6e} fd={:.6e} err={:.2e}",
            dic_dvbc, fd_dvbc, rel_err_bc);
    }
}
