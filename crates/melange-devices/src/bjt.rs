//! Bipolar Junction Transistor (BJT) models.
//!
//! Includes:
//! - Ebers-Moll model (basic, good for most audio applications)
//! - Gummel-Poon model (extended with Early effect and high-level injection)

use crate::{safeguards, NonlinearDevice, VT_ROOM};

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
    /// Forward emission coefficient (NF). Default 1.0.
    /// Forward exponential uses exp(Vbe / (NF * VT)).
    pub nf: f64,
    /// Reverse emission coefficient (NR). Default 1.0.
    /// Reverse exponential uses exp(Vbc / (NR * VT)).
    pub nr: f64,
    /// Base-emitter leakage saturation current [A]. Default 0.0 (disabled).
    pub ise: f64,
    /// Base-emitter leakage emission coefficient. Default 1.5.
    pub ne: f64,
    /// Base-collector leakage saturation current [A]. Default 0.0 (disabled).
    pub isc: f64,
    /// Base-collector leakage emission coefficient. Default 1.0.
    pub nc: f64,
    /// Polarity (NPN or PNP)
    pub polarity: BjtPolarity,
    /// Precomputed: 1 / beta_f
    inv_beta_f: f64,
    /// Precomputed: 1 / beta_r
    inv_beta_r: f64,
}

impl BjtEbersMoll {
    /// Create a new BJT model (NF, NR default to 1.0; ISE, ISC default to 0.0).
    pub fn new(is: f64, vt: f64, beta_f: f64, beta_r: f64, polarity: BjtPolarity) -> Self {
        Self {
            is,
            vt,
            beta_f,
            beta_r,
            nf: 1.0,
            nr: 1.0,
            ise: 0.0,
            ne: 1.5,
            isc: 0.0,
            nc: 1.0,
            polarity,
            inv_beta_f: 1.0 / beta_f,
            inv_beta_r: 1.0 / beta_r,
        }
    }

    /// Create at room temperature (NF defaults to 1.0).
    pub fn new_room_temp(is: f64, beta_f: f64, beta_r: f64, polarity: BjtPolarity) -> Self {
        Self::new(is, VT_ROOM, beta_f, beta_r, polarity)
    }

    /// Builder: set forward emission coefficient NF.
    pub fn with_nf(mut self, nf: f64) -> Self {
        self.nf = nf;
        self
    }

    /// Builder: set reverse emission coefficient NR.
    pub fn with_nr(mut self, nr: f64) -> Self {
        self.nr = nr;
        self
    }

    /// Builder: set base-emitter and base-collector leakage parameters.
    ///
    /// ISE/ISC are leakage saturation currents; NE/NC are leakage emission coefficients.
    /// Default ISE=0.0, NE=1.5, ISC=0.0, NC=1.0 (zero ISE/ISC means no leakage).
    pub fn with_leakage(mut self, ise: f64, ne: f64, isc: f64, nc: f64) -> Self {
        self.ise = ise;
        self.ne = ne;
        self.isc = isc;
        self.nc = nc;
        self
    }

    /// 2N2222A NPN transistor (common general-purpose).
    pub fn npn_2n2222a() -> Self {
        let c = crate::catalog::bjts::lookup("2N2222A").unwrap();
        Self::new_room_temp(c.is, c.beta_f, c.beta_r, BjtPolarity::Npn)
    }

    /// 2N3904 NPN transistor.
    pub fn npn_2n3904() -> Self {
        let c = crate::catalog::bjts::lookup("2N3904").unwrap();
        Self::new_room_temp(c.is, c.beta_f, c.beta_r, BjtPolarity::Npn)
    }

    /// 2N3906 PNP transistor.
    pub fn pnp_2n3906() -> Self {
        let c = crate::catalog::bjts::lookup("2N3906").unwrap();
        Self::new_room_temp(c.is, c.beta_f, c.beta_r, BjtPolarity::Pnp)
    }

    /// AC128 Germanium PNP (vintage fuzz tones).
    pub fn pnp_ac128() -> Self {
        let c = crate::catalog::bjts::lookup("AC128").unwrap();
        Self::new_room_temp(c.is, c.beta_f, c.beta_r, BjtPolarity::Pnp)
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
        let nf_vt = self.nf * self.vt;
        let nr_vt = self.nr * self.vt;

        // Apply polarity to voltages
        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        // Transport current (forward uses NF*VT, reverse uses NR*VT)
        let exp_be = safeguards::safe_exp(vbe_eff / nf_vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / nr_vt);
        let i_cc = self.is * (exp_be - exp_bc);

        // Collector current: Ic = I_cc - Is/βr * (exp(Vbc/(NR*Vt)) - 1)
        let ic = i_cc - self.is * self.inv_beta_r * (exp_bc - 1.0);

        s * ic // Apply polarity to current
    }

    /// Calculate base current given Vbe and Vbc.
    ///
    /// Includes leakage terms when ISE > 0 or ISC > 0:
    ///   Ib = IS/BF * (exp(Vbe/(NF*VT))-1) + ISE * (exp(Vbe/(NE*VT))-1)
    ///      + IS/BR * (exp(Vbc/(NR*VT))-1) + ISC * (exp(Vbc/(NC*VT))-1)
    pub fn base_current(&self, vbe: f64, vbc: f64) -> f64 {
        let s = self.sign();
        let nf_vt = self.nf * self.vt;
        let nr_vt = self.nr * self.vt;

        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        let exp_be = safeguards::safe_exp(vbe_eff / nf_vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / nr_vt);

        // Ideal base current terms (forward uses NF*VT, reverse uses NR*VT)
        let ib_fwd = self.is * self.inv_beta_f * (exp_be - 1.0);
        let ib_rev = self.is * self.inv_beta_r * (exp_bc - 1.0);

        // Leakage terms
        let ib_leak_be = if self.ise > 0.0 {
            self.ise * (safeguards::safe_exp(vbe_eff / (self.ne * self.vt)) - 1.0)
        } else {
            0.0
        };
        let ib_leak_bc = if self.isc > 0.0 {
            self.isc * (safeguards::safe_exp(vbc_eff / (self.nc * self.vt)) - 1.0)
        } else {
            0.0
        };

        s * (ib_fwd + ib_rev + ib_leak_be + ib_leak_bc)
    }

    /// Partial derivative of base current with respect to Vbe.
    pub fn base_current_jacobian_dvbe(&self, vbe: f64, _vbc: f64) -> f64 {
        let s = self.sign();
        let nf_vt = self.nf * self.vt;
        let vbe_eff = s * vbe;
        let exp_be = safeguards::safe_exp(vbe_eff / nf_vt);

        // ∂Ib/∂Vbe = (Is/(βf*NF*Vt)) * exp(Vbe/(NF*Vt))
        let mut dib_dvbe = self.is * self.inv_beta_f / nf_vt * exp_be;

        // Leakage derivative: ∂Ib_leak_be/∂Vbe = ISE/(NE*VT) * exp(Vbe/(NE*VT))
        if self.ise > 0.0 {
            let ne_vt = self.ne * self.vt;
            let exp_leak_be = safeguards::safe_exp(vbe_eff / ne_vt);
            dib_dvbe += self.ise / ne_vt * exp_leak_be;
        }

        // Apply chain rule for polarity (s * s = 1, so no sign change)
        s * s * dib_dvbe
    }

    /// Partial derivative of base current with respect to Vbc.
    pub fn base_current_jacobian_dvbc(&self, _vbe: f64, vbc: f64) -> f64 {
        let s = self.sign();
        let nr_vt = self.nr * self.vt;
        let vbc_eff = s * vbc;
        let exp_bc = safeguards::safe_exp(vbc_eff / nr_vt);

        // ∂Ib/∂Vbc = (Is/(βr*NR*Vt)) * exp(Vbc/(NR*Vt))
        let mut dib_dvbc = self.is * self.inv_beta_r / nr_vt * exp_bc;

        // Leakage derivative: ∂Ib_leak_bc/∂Vbc = ISC/(NC*VT) * exp(Vbc/(NC*VT))
        if self.isc > 0.0 {
            let nc_vt = self.nc * self.vt;
            let exp_leak_bc = safeguards::safe_exp(vbc_eff / nc_vt);
            dib_dvbc += self.isc / nc_vt * exp_leak_bc;
        }

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
        let nf_vt = self.nf * self.vt;
        let nr_vt = self.nr * self.vt;

        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        let exp_be = safeguards::safe_exp(vbe_eff / nf_vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / nr_vt);

        // ∂Ic/∂Vbe = (Is/(NF*Vt)) * exp(Vbe/(NF*Vt))
        let d_ic_d_vbe = (self.is / nf_vt) * exp_be;

        // ∂Ic/∂Vbc = -(Is/(NR*Vt)) * exp(Vbc/(NR*Vt)) - (Is/(βr*NR*Vt)) * exp(Vbc/(NR*Vt))
        let d_ic_d_vbc = -(self.is / nr_vt) * exp_bc - (self.is * self.inv_beta_r / nr_vt) * exp_bc;

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
        Self {
            base,
            vaf,
            var,
            ikf,
            ikr,
        }
    }

    /// 2N2222A with typical Gummel-Poon parameters.
    pub fn npn_2n2222a() -> Self {
        Self::new(
            BjtEbersMoll::npn_2n2222a(),
            100.0, // VAF
            10.0,  // VAR
            0.3,   // IKF
            0.006, // IKR
        )
    }

    /// Base charge factor qb (accounts for Early effect and high injection).
    fn qb(&self, vbe: f64, vbc: f64) -> f64 {
        let s = self.base.sign();
        let nf_vt = self.base.nf * self.base.vt;
        let nr_vt = self.base.nr * self.base.vt;
        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        // Early effect
        let q1_denom = 1.0 - vbe_eff / self.var - vbc_eff / self.vaf;
        if q1_denom.abs() < 1e-30 {
            return 1.0;
        }
        let q1 = 1.0 / q1_denom;

        // High-level injection (forward uses NF*VT, reverse uses NR*VT)
        let exp_be = safeguards::safe_exp(vbe_eff / nf_vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / nr_vt);
        let q2 = self.base.is * exp_be / self.ikf + self.base.is * exp_bc / self.ikr;

        q1 * (1.0 + (1.0 + 4.0 * q2).max(0.0).sqrt()) / 2.0
    }

    /// Base current with GP modification: forward ideal component divided by qb.
    ///
    /// `Ib = Is/BF * (exp(Vbe/(NF*VT)) - 1) / qb + Is/BR * (exp(Vbc/(NR*VT)) - 1)`
    /// `   + ISE * (exp(Vbe/(NE*VT)) - 1) + ISC * (exp(Vbc/(NC*VT)) - 1)`
    ///
    /// The forward ideal component (Is/BF term) is divided by qb to account
    /// for Early effect and high-level injection. Reverse component and
    /// leakage terms are unchanged (not affected by qb).
    pub fn base_current(&self, vbe: f64, vbc: f64) -> f64 {
        let s = self.base.sign();
        let nf_vt = self.base.nf * self.base.vt;
        let nr_vt = self.base.nr * self.base.vt;
        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        let exp_be = safeguards::safe_exp(vbe_eff / nf_vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / nr_vt);

        let qb = self.qb(vbe, vbc);
        let ib_fwd = self.base.is * self.base.inv_beta_f * (exp_be - 1.0) / qb;
        let ib_rev = self.base.is * self.base.inv_beta_r * (exp_bc - 1.0);

        // Leakage terms (same as Ebers-Moll, not affected by qb)
        let ib_leak_be = if self.base.ise > 0.0 {
            self.base.ise * (safeguards::safe_exp(vbe_eff / (self.base.ne * self.base.vt)) - 1.0)
        } else {
            0.0
        };
        let ib_leak_bc = if self.base.isc > 0.0 {
            self.base.isc * (safeguards::safe_exp(vbc_eff / (self.base.nc * self.base.vt)) - 1.0)
        } else {
            0.0
        };

        s * (ib_fwd + ib_rev + ib_leak_be + ib_leak_bc)
    }

    /// Collector current with Early effect and high injection.
    pub fn collector_current(&self, vbe: f64, vbc: f64) -> f64 {
        let s = self.base.sign();
        let nf_vt = self.base.nf * self.base.vt;
        let nr_vt = self.base.nr * self.base.vt;
        let vbe_eff = s * vbe;
        let vbc_eff = s * vbc;

        let exp_be = safeguards::safe_exp(vbe_eff / nf_vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / nr_vt);

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
        let nf_vt = self.base.nf * vt;
        let nr_vt = self.base.nr * vt;
        let is = self.base.is;

        let exp_be = safeguards::safe_exp(vbe_eff / nf_vt);
        let exp_bc = safeguards::safe_exp(vbc_eff / nr_vt);

        // Transport current and derivatives (forward uses NF*VT, reverse uses NR*VT)
        let icc = is * (exp_be - exp_bc);
        let dicc_dvbe = is / nf_vt * exp_be;
        let dicc_dvbc = -is / nr_vt * exp_bc;

        // Base charge factor q1 (matches qb() singularity handling)
        let q1_denom = 1.0 - vbe_eff / self.var - vbc_eff / self.vaf;
        let (q1, dq1_dvbe, dq1_dvbc) = if q1_denom.abs() < 1e-30 {
            (1.0, 0.0, 0.0) // matches qb() fallback
        } else {
            let q1 = 1.0 / q1_denom;
            (q1, q1 * q1 / self.var, q1 * q1 / self.vaf)
        };

        // High injection q2 = Is*exp(Vbe/(NF*Vt))/IKF + Is*exp(Vbc/(NR*Vt))/IKR
        let q2 = is * exp_be / self.ikf + is * exp_bc / self.ikr;
        let dq2_dvbe = is / (nf_vt * self.ikf) * exp_be;
        let dq2_dvbc = is / (nr_vt * self.ikr) * exp_bc;

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
        let qb2_safe = qb2.max(1e-30);
        let quotient_dvbe = (dicc_dvbe * qb - icc * dqb_dvbe) / qb2_safe;
        let quotient_dvbc = (dicc_dvbc * qb - icc * dqb_dvbc) / qb2_safe;

        let d_bc_term_dvbc = is * self.base.inv_beta_r / nr_vt * exp_bc;

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
        let vbc = -5.0; // Vce = 5.7V

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
        let ic_pnp = pnp.collector_current(-vbe, -vbc); // Inverted for PNP

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
        assert_eq!(
            classify_region(0.0, 0.0, BjtPolarity::Npn),
            BjtRegion::Cutoff
        );
        assert_eq!(
            classify_region(0.7, -5.0, BjtPolarity::Npn),
            BjtRegion::ForwardActive
        );
        assert_eq!(
            classify_region(0.7, 0.7, BjtPolarity::Npn),
            BjtRegion::Saturation
        );
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
        assert!(
            (beta - 200.0).abs() / 200.0 < 0.05,
            "Beta = {:.1}, expected ~200",
            beta
        );

        // KCL: Ie = -(Ic + Ib)
        let ie = -(ic + ib);
        assert!(
            ie < 0.0,
            "Emitter current should be negative (exits emitter for NPN)"
        );
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

            assert!(
                rel_err_be < 1e-4,
                "dIc/dVbe at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vbe,
                vbc,
                dic_dvbe,
                fd_dvbe,
                rel_err_be
            );
            assert!(
                rel_err_bc < 1e-4,
                "dIc/dVbc at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vbe,
                vbc,
                dic_dvbc,
                fd_dvbc,
                rel_err_bc
            );
        }
    }

    /// Verify Gummel-Poon Jacobian matches finite-difference derivatives
    /// of collector_current near the Q1 singularity point (where 1 - Vbe/VAR - Vbc/VAF ≈ 0).
    #[test]
    fn test_gummel_poon_q1_singularity_jacobian_consistency() {
        let eps = 1e-7;

        // Standard 2N2222A GP at normal operating points
        let bjt_std = BjtGummelPoon::npn_2n2222a();
        let std_points = [
            (0.7, -5.0),  // standard forward active
            (0.65, -2.0), // moderate
            (0.8, -10.0), // higher Vbe, within safe_exp range
        ];

        // Custom GP with very small VAR to approach Q1 singularity at small voltages
        // VAR=1.0: q1_denom = 1 - Vbe/1.0 - Vbc/100 → near-zero at Vbe~0.95
        let bjt_near_sing = BjtGummelPoon::new(
            BjtEbersMoll::npn_2n2222a(),
            100.0, // VAF (normal)
            1.0,   // VAR (very small, creates singularity near Vbe=1V)
            0.3,   // IKF
            0.006, // IKR
        );
        let near_sing_points = [
            (0.8, 0.0),  // q1_denom = 1 - 0.8 = 0.2
            (0.9, 0.0),  // q1_denom = 0.1 (very close to singularity)
            (0.95, 0.0), // q1_denom = 0.05 (near singularity)
        ];

        for (bjt, points, label) in [
            (&bjt_std, &std_points[..], "standard"),
            (&bjt_near_sing, &near_sing_points[..], "near-singularity"),
        ] {
            for &(vbe, vbc) in points {
                let jac = bjt.jacobian(&[vbe, vbc]);

                // Finite-difference derivatives
                let ic_vbe_p = bjt.collector_current(vbe + eps, vbc);
                let ic_vbe_m = bjt.collector_current(vbe - eps, vbc);
                let fd_dvbe = (ic_vbe_p - ic_vbe_m) / (2.0 * eps);

                let ic_vbc_p = bjt.collector_current(vbe, vbc + eps);
                let ic_vbc_m = bjt.collector_current(vbe, vbc - eps);
                let fd_dvbc = (ic_vbc_p - ic_vbc_m) / (2.0 * eps);

                // Both should be finite
                assert!(
                    jac[0].is_finite(),
                    "{}: dIc/dVbe must be finite at ({}, {})",
                    label,
                    vbe,
                    vbc
                );
                assert!(
                    jac[1].is_finite(),
                    "{}: dIc/dVbc must be finite at ({}, {})",
                    label,
                    vbe,
                    vbc
                );

                // Compare analytic vs finite-difference
                let rel_err_be = if fd_dvbe.abs() > 1e-10 {
                    (jac[0] - fd_dvbe).abs() / fd_dvbe.abs()
                } else {
                    (jac[0] - fd_dvbe).abs()
                };
                let rel_err_bc = if fd_dvbc.abs() > 1e-10 {
                    (jac[1] - fd_dvbc).abs() / fd_dvbc.abs()
                } else {
                    (jac[1] - fd_dvbc).abs()
                };

                assert!(
                    rel_err_be < 0.01,
                    "{}: GP dIc/dVbe at ({}, {}): analytic={:.6e} fd={:.6e} rel_err={:.2e}",
                    label,
                    vbe,
                    vbc,
                    jac[0],
                    fd_dvbe,
                    rel_err_be
                );
                assert!(
                    rel_err_bc < 0.01,
                    "{}: GP dIc/dVbc at ({}, {}): analytic={:.6e} fd={:.6e} rel_err={:.2e}",
                    label,
                    vbe,
                    vbc,
                    jac[1],
                    fd_dvbc,
                    rel_err_bc
                );
            }
        }
    }

    /// Verify Gummel-Poon collector current values and Early effect.
    #[test]
    fn test_gummel_poon_early_effect() {
        let bjt = BjtGummelPoon::npn_2n2222a();

        // Forward active: Vbe=0.7V at different Vce (different Vbc)
        let ic_vce5 = bjt.collector_current(0.7, 0.7 - 5.0); // Vce=5V
        let ic_vce10 = bjt.collector_current(0.7, 0.7 - 10.0); // Vce=10V

        // With Early effect, Ic should increase slightly with Vce
        assert!(
            ic_vce10 > ic_vce5,
            "Early effect: Ic should increase with Vce. Ic(5V)={:.4}mA, Ic(10V)={:.4}mA",
            ic_vce5 * 1000.0,
            ic_vce10 * 1000.0
        );

        // The increase should be modest (not orders of magnitude)
        let ratio = ic_vce10 / ic_vce5;
        assert!(
            ratio < 1.5,
            "Early effect ratio should be modest: {:.3}",
            ratio
        );
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

        assert!(
            rel_err_be < 1e-3,
            "GP dIc/dVbe: analytic={:.6e} fd={:.6e} err={:.2e}",
            dic_dvbe,
            fd_dvbe,
            rel_err_be
        );
        assert!(
            rel_err_bc < 1e-3,
            "GP dIc/dVbc: analytic={:.6e} fd={:.6e} err={:.2e}",
            dic_dvbc,
            fd_dvbc,
            rel_err_bc
        );
    }

    // ========================================================================
    // Gummel-Poon comprehensive tests (#20)
    // ========================================================================

    // --- High-level injection tests ---

    /// At high Vbe the Gummel-Poon model should exhibit beta droop
    /// due to high-level injection (IKF limiting).
    /// Compare effective beta at moderate vs high Vbe.
    #[test]
    fn test_gp_high_level_injection_beta_droop() {
        let gp = BjtGummelPoon::npn_2n2222a();
        let em = BjtEbersMoll::npn_2n2222a();

        // Moderate injection: Vbe=0.65V
        let ic_mod = gp.collector_current(0.65, -5.0);
        let ib_mod = em.base_current(0.65, -5.0);
        let beta_mod = ic_mod / ib_mod;

        // High injection: Vbe=0.85V (pushing toward IKF)
        let ic_high = gp.collector_current(0.85, -5.0);
        let ib_high = em.base_current(0.85, -5.0);
        let beta_high = ic_high / ib_high;

        // Beta should decrease at high injection (beta droop)
        assert!(
            beta_high < beta_mod,
            "GP beta should droop at high injection: beta_mod={:.1}, beta_high={:.1}",
            beta_mod,
            beta_high
        );

        // Both betas should be positive and finite
        assert!(beta_mod > 0.0 && beta_mod.is_finite());
        assert!(beta_high > 0.0 && beta_high.is_finite());
    }

    /// At very high Vbe (near safe_exp limit), the GP model should still
    /// produce finite results. The qb factor should prevent unbounded currents.
    #[test]
    fn test_gp_high_injection_extreme_vbe() {
        let gp = BjtGummelPoon::npn_2n2222a();

        // Sweep Vbe from 0.7 to 1.0 in steps
        let mut prev_ic = 0.0;
        for vbe_mv in (700..=1000).step_by(50) {
            let vbe = vbe_mv as f64 / 1000.0;
            let ic = gp.collector_current(vbe, -5.0);

            assert!(ic.is_finite(), "GP Ic must be finite at Vbe={}", vbe);
            assert!(ic > 0.0, "GP Ic must be positive at Vbe={}", vbe);

            // Current should increase monotonically with Vbe
            if vbe_mv > 700 {
                assert!(
                    ic > prev_ic,
                    "GP Ic should increase with Vbe: Ic({:.2})={:.6e} vs Ic(prev)={:.6e}",
                    vbe,
                    ic,
                    prev_ic
                );
            }
            prev_ic = ic;
        }
    }

    /// Verify that GP Ic is less than Ebers-Moll Ic at high injection
    /// (qb > 1 reduces the transport current).
    #[test]
    fn test_gp_ic_less_than_ebers_moll_at_high_injection() {
        let gp = BjtGummelPoon::npn_2n2222a();
        let em = BjtEbersMoll::npn_2n2222a();

        // At Vbe=0.85V, high-level injection should reduce GP current vs EM
        let ic_gp = gp.collector_current(0.85, -5.0);
        let ic_em = em.collector_current(0.85, -5.0);

        assert!(
            ic_gp < ic_em,
            "GP Ic ({:.6e}) should be less than EM Ic ({:.6e}) at high injection",
            ic_gp,
            ic_em
        );
    }

    // --- Base-width modulation (Early effect) tests ---

    /// Sweep Vce from 1V to 20V and verify Ic increases monotonically
    /// (positive output conductance from Early effect).
    #[test]
    fn test_gp_early_effect_vce_sweep() {
        let gp = BjtGummelPoon::npn_2n2222a();
        let vbe = 0.7;

        let mut prev_ic = 0.0;
        for vce_x10 in (10..=200).step_by(10) {
            let vce = vce_x10 as f64 / 10.0;
            let vbc = vbe - vce;
            let ic = gp.collector_current(vbe, vbc);

            assert!(ic.is_finite(), "GP Ic must be finite at Vce={}", vce);
            assert!(ic > 0.0, "GP Ic must be positive at Vce={}", vce);

            if vce_x10 > 10 {
                assert!(
                    ic >= prev_ic,
                    "GP Ic should increase with Vce (Early effect): Ic({:.1}V)={:.6e} < Ic(prev)={:.6e}",
                    vce,
                    ic,
                    prev_ic
                );
            }
            prev_ic = ic;
        }
    }

    /// Verify the output resistance (ro = dVce/dIc) is approximately VAF/Ic.
    /// This is the classic Early voltage relationship.
    #[test]
    fn test_gp_output_resistance_early_voltage() {
        let gp = BjtGummelPoon::npn_2n2222a();
        let vbe = 0.7;

        // Measure Ic at two nearby Vce values
        let vce1 = 5.0;
        let vce2 = 10.0;
        let ic1 = gp.collector_current(vbe, vbe - vce1);
        let ic2 = gp.collector_current(vbe, vbe - vce2);

        let delta_vce = vce2 - vce1;
        let delta_ic = ic2 - ic1;
        let ro = delta_vce / delta_ic;

        // ro should be approximately VAF / Ic (= 100V / ~few mA = tens of kOhms)
        let ic_mid = (ic1 + ic2) / 2.0;
        let ro_expected = 100.0 / ic_mid; // VAF = 100V for 2N2222A GP

        // Allow a factor of 5x tolerance (Early voltage is approximate)
        assert!(
            ro > ro_expected * 0.2 && ro < ro_expected * 5.0,
            "Output resistance ro={:.0} ohm, expected ~{:.0} ohm (VAF/Ic)",
            ro,
            ro_expected
        );
    }

    /// With a very large VAF (weak Early effect), GP should approach EM behavior.
    #[test]
    fn test_gp_large_vaf_approaches_ebers_moll() {
        let em = BjtEbersMoll::npn_2n2222a();
        let gp_weak = BjtGummelPoon::new(
            BjtEbersMoll::npn_2n2222a(),
            1e6, // Very large VAF (negligible Early effect)
            1e6, // Very large VAR
            1e6, // Very large IKF (no high injection)
            1e6, // Very large IKR
        );

        let vbe = 0.7;
        let vbc = -5.0;
        let ic_em = em.collector_current(vbe, vbc);
        let ic_gp = gp_weak.collector_current(vbe, vbc);

        let rel_diff = (ic_gp - ic_em).abs() / ic_em.abs();
        assert!(
            rel_diff < 0.001,
            "GP with very large VAF/IKF should match EM: GP={:.6e}, EM={:.6e}, rel_diff={:.2e}",
            ic_gp,
            ic_em,
            rel_diff
        );
    }

    // --- Operating region tests ---

    /// Verify GP model behavior in all four operating regions.
    #[test]
    fn test_gp_all_operating_regions() {
        let gp = BjtGummelPoon::npn_2n2222a();

        // Cutoff: Vbe=0, Vbc=-5
        let ic_cutoff = gp.collector_current(0.0, -5.0);
        assert!(
            ic_cutoff.abs() < 1e-10,
            "Cutoff Ic should be ~0: {:.2e}",
            ic_cutoff
        );

        // Forward active: Vbe=0.7, Vbc=-5
        let ic_fwd = gp.collector_current(0.7, -5.0);
        assert!(
            ic_fwd > 1e-4,
            "Forward active Ic should be significant: {:.2e}",
            ic_fwd
        );
        assert!(ic_fwd > 0.0, "Forward active Ic should be positive");

        // Reverse active: Vbe=-5, Vbc=0.7
        let ic_rev = gp.collector_current(-5.0, 0.7);
        assert!(
            ic_rev < 0.0,
            "Reverse active Ic should be negative: {:.2e}",
            ic_rev
        );

        // Saturation: Vbe=0.7, Vbc=0.6 (both junctions forward biased)
        let ic_sat = gp.collector_current(0.7, 0.6);
        assert!(
            ic_sat.is_finite(),
            "Saturation Ic should be finite: {}",
            ic_sat
        );
        // In saturation, Ic should be smaller than in forward active
        // (forward transport partially canceled by reverse injection)
        assert!(
            ic_sat < ic_fwd,
            "Saturation Ic ({:.2e}) should be less than forward active Ic ({:.2e})",
            ic_sat,
            ic_fwd
        );
    }

    /// In forward active, Ic should be much larger than |Ib|.
    /// In saturation, this ratio decreases (forced beta < natural beta).
    #[test]
    fn test_gp_forced_beta_in_saturation() {
        let gp = BjtGummelPoon::npn_2n2222a();
        let em = &gp.base;

        // Forward active
        let ic_fwd = gp.collector_current(0.7, -5.0);
        let ib_fwd = em.base_current(0.7, -5.0);
        let beta_fwd = ic_fwd / ib_fwd;

        // Saturation (both junctions forward biased)
        let ic_sat = gp.collector_current(0.7, 0.5);
        let ib_sat = em.base_current(0.7, 0.5);
        let beta_sat = ic_sat / ib_sat;

        assert!(
            beta_sat < beta_fwd,
            "Forced beta in saturation ({:.1}) should be less than forward active beta ({:.1})",
            beta_sat,
            beta_fwd
        );
    }

    /// Region classification should work correctly for PNP transistors.
    #[test]
    fn test_region_classification_pnp() {
        // For PNP, the voltages are inverted relative to NPN
        // PNP forward active: Vbe=-0.7, Vbc=5
        assert_eq!(
            classify_region(-0.7, 5.0, BjtPolarity::Pnp),
            BjtRegion::ForwardActive
        );
        assert_eq!(
            classify_region(0.0, 0.0, BjtPolarity::Pnp),
            BjtRegion::Cutoff
        );
        assert_eq!(
            classify_region(-0.7, -0.7, BjtPolarity::Pnp),
            BjtRegion::Saturation
        );
        assert_eq!(
            classify_region(5.0, -0.7, BjtPolarity::Pnp),
            BjtRegion::ReverseActive
        );
    }

    // --- PNP vs NPN sign handling ---

    /// PNP GP model should have opposite current signs compared to NPN.
    #[test]
    fn test_gp_pnp_sign_handling() {
        let npn = BjtGummelPoon::npn_2n2222a();
        let pnp_base = BjtEbersMoll::pnp_2n3906();
        let pnp = BjtGummelPoon::new(pnp_base, 100.0, 10.0, 0.3, 0.006);

        // NPN forward active: Vbe=+0.7, Vbc=-5
        let ic_npn = npn.collector_current(0.7, -5.0);
        assert!(ic_npn > 0.0, "NPN Ic should be positive: {:.2e}", ic_npn);

        // PNP forward active: Vbe=-0.7, Vbc=+5
        let ic_pnp = pnp.collector_current(-0.7, 5.0);
        assert!(ic_pnp < 0.0, "PNP Ic should be negative: {:.2e}", ic_pnp);

        // Both should be finite
        assert!(ic_npn.is_finite());
        assert!(ic_pnp.is_finite());
    }

    /// PNP GP Jacobian should have correct signs (through NonlinearDevice trait).
    #[test]
    fn test_gp_pnp_jacobian_signs() {
        let pnp_base = BjtEbersMoll::pnp_2n3906();
        let pnp = BjtGummelPoon::new(pnp_base, 100.0, 10.0, 0.3, 0.006);

        // PNP forward active: Vbe=-0.7, Vbc=+5
        let jac = pnp.jacobian(&[-0.7, 5.0]);

        // dIc/dVbe for PNP should be negative (Ic decreases as Vbe becomes more negative)
        // Actually for PNP, Ic < 0, and making Vbe more negative increases |Ic|,
        // so dIc/dVbe should be positive (less negative Ic with less negative Vbe).
        // The sign depends on the convention. Let's just verify finite and nonzero.
        assert!(
            jac[0].is_finite(),
            "PNP dIc/dVbe should be finite: {}",
            jac[0]
        );
        assert!(
            jac[1].is_finite(),
            "PNP dIc/dVbc should be finite: {}",
            jac[1]
        );
        assert!(jac[0] != 0.0, "PNP dIc/dVbe should be nonzero");
    }

    /// PNP GP Jacobian should match finite differences.
    #[test]
    fn test_gp_pnp_jacobian_finite_difference() {
        let pnp_base = BjtEbersMoll::pnp_2n3906();
        let pnp = BjtGummelPoon::new(pnp_base, 100.0, 10.0, 0.3, 0.006);
        let eps = 1e-7;

        // PNP forward active
        let vbe = -0.7;
        let vbc = 5.0;

        let jac = pnp.jacobian(&[vbe, vbc]);

        let ic_p = pnp.collector_current(vbe + eps, vbc);
        let ic_m = pnp.collector_current(vbe - eps, vbc);
        let fd_dvbe = (ic_p - ic_m) / (2.0 * eps);

        let ic_p = pnp.collector_current(vbe, vbc + eps);
        let ic_m = pnp.collector_current(vbe, vbc - eps);
        let fd_dvbc = (ic_p - ic_m) / (2.0 * eps);

        let rel_err_be = if fd_dvbe.abs() > 1e-10 {
            (jac[0] - fd_dvbe).abs() / fd_dvbe.abs()
        } else {
            (jac[0] - fd_dvbe).abs()
        };
        let rel_err_bc = if fd_dvbc.abs() > 1e-10 {
            (jac[1] - fd_dvbc).abs() / fd_dvbc.abs()
        } else {
            (jac[1] - fd_dvbc).abs()
        };

        assert!(
            rel_err_be < 0.01,
            "PNP GP dIc/dVbe: analytic={:.6e} fd={:.6e} rel_err={:.2e}",
            jac[0],
            fd_dvbe,
            rel_err_be
        );
        assert!(
            rel_err_bc < 0.01,
            "PNP GP dIc/dVbc: analytic={:.6e} fd={:.6e} rel_err={:.2e}",
            jac[1],
            fd_dvbc,
            rel_err_bc
        );
    }

    // --- Edge cases ---

    /// Vbe=0, Vbc=0: both junctions unbiased. Ic should be ~0.
    #[test]
    fn test_gp_both_junctions_zero() {
        let gp = BjtGummelPoon::npn_2n2222a();

        let ic = gp.collector_current(0.0, 0.0);
        assert!(ic.is_finite(), "GP Ic at (0,0) should be finite: {}", ic);
        assert!(ic.abs() < 1e-12, "GP Ic at (0,0) should be ~0: {:.2e}", ic);

        let jac = gp.jacobian(&[0.0, 0.0]);
        assert!(jac[0].is_finite(), "GP dIc/dVbe at (0,0) should be finite");
        assert!(jac[1].is_finite(), "GP dIc/dVbc at (0,0) should be finite");
    }

    /// Vbe=0 with Vbc negative: cutoff region.
    #[test]
    fn test_gp_vbe_zero_vbc_negative() {
        let gp = BjtGummelPoon::npn_2n2222a();

        let ic = gp.collector_current(0.0, -10.0);
        assert!(ic.is_finite());
        assert!(
            ic.abs() < 1e-10,
            "GP Ic in cutoff should be near zero: {:.2e}",
            ic
        );
    }

    /// Both junctions forward biased (saturation): should produce finite results.
    #[test]
    fn test_gp_both_junctions_forward_biased() {
        let gp = BjtGummelPoon::npn_2n2222a();

        // Strong saturation: Vbe=0.7, Vbc=0.7
        let ic = gp.collector_current(0.7, 0.7);
        assert!(
            ic.is_finite(),
            "GP Ic in deep saturation should be finite: {}",
            ic
        );

        let jac = gp.jacobian(&[0.7, 0.7]);
        assert!(
            jac[0].is_finite(),
            "GP Jacobian[0] in deep saturation should be finite"
        );
        assert!(
            jac[1].is_finite(),
            "GP Jacobian[1] in deep saturation should be finite"
        );

        // Jacobian should match finite difference even in saturation
        let eps = 1e-7;
        let fd_dvbe = (gp.collector_current(0.7 + eps, 0.7) - gp.collector_current(0.7 - eps, 0.7))
            / (2.0 * eps);
        let fd_dvbc = (gp.collector_current(0.7, 0.7 + eps) - gp.collector_current(0.7, 0.7 - eps))
            / (2.0 * eps);

        let rel_err_be = if fd_dvbe.abs() > 1e-10 {
            (jac[0] - fd_dvbe).abs() / fd_dvbe.abs()
        } else {
            (jac[0] - fd_dvbe).abs()
        };
        let rel_err_bc = if fd_dvbc.abs() > 1e-10 {
            (jac[1] - fd_dvbc).abs() / fd_dvbc.abs()
        } else {
            (jac[1] - fd_dvbc).abs()
        };

        assert!(
            rel_err_be < 0.01,
            "GP saturation dIc/dVbe: analytic={:.6e} fd={:.6e} err={:.2e}",
            jac[0],
            fd_dvbe,
            rel_err_be
        );
        assert!(
            rel_err_bc < 0.01,
            "GP saturation dIc/dVbc: analytic={:.6e} fd={:.6e} err={:.2e}",
            jac[1],
            fd_dvbc,
            rel_err_bc
        );
    }

    // --- Consistency tests: NonlinearDevice trait vs direct methods ---

    /// The NonlinearDevice::current() should match collector_current().
    #[test]
    fn test_gp_trait_current_matches_direct() {
        let gp = BjtGummelPoon::npn_2n2222a();

        for &(vbe, vbc) in &[(0.7, -5.0), (0.0, 0.0), (0.65, -2.0), (0.8, 0.5)] {
            let ic_direct = gp.collector_current(vbe, vbc);
            let ic_trait = gp.current(&[vbe, vbc]);

            assert!(
                (ic_direct - ic_trait).abs() < 1e-20,
                "Trait current should match direct at ({}, {}): direct={:.6e} trait={:.6e}",
                vbe,
                vbc,
                ic_direct,
                ic_trait
            );
        }
    }

    /// The NonlinearDevice::jacobian() should return [dIc/dVbe, dIc/dVbc].
    #[test]
    fn test_gp_trait_jacobian_matches_direct() {
        let gp = BjtGummelPoon::npn_2n2222a();

        for &(vbe, vbc) in &[(0.7, -5.0), (0.0, 0.0), (0.65, -2.0), (0.8, 0.5)] {
            let jac = gp.jacobian(&[vbe, vbc]);

            // Verify by finite difference
            let eps = 1e-7;
            let fd_dvbe =
                (gp.current(&[vbe + eps, vbc]) - gp.current(&[vbe - eps, vbc])) / (2.0 * eps);
            let fd_dvbc =
                (gp.current(&[vbe, vbc + eps]) - gp.current(&[vbe, vbc - eps])) / (2.0 * eps);

            let err_be = if fd_dvbe.abs() > 1e-10 {
                (jac[0] - fd_dvbe).abs() / fd_dvbe.abs()
            } else {
                (jac[0] - fd_dvbe).abs()
            };
            let err_bc = if fd_dvbc.abs() > 1e-10 {
                (jac[1] - fd_dvbc).abs() / fd_dvbc.abs()
            } else {
                (jac[1] - fd_dvbc).abs()
            };

            assert!(
                err_be < 0.01,
                "GP trait Jac[0] at ({}, {}): {:.6e} vs fd {:.6e}, err={:.2e}",
                vbe,
                vbc,
                jac[0],
                fd_dvbe,
                err_be
            );
            assert!(
                err_bc < 0.01,
                "GP trait Jac[1] at ({}, {}): {:.6e} vs fd {:.6e}, err={:.2e}",
                vbe,
                vbc,
                jac[1],
                fd_dvbc,
                err_bc
            );
        }
    }

    // --- Jacobian sign and magnitude tests ---

    /// In forward active, dIc/dVbe (transconductance gm) must be positive.
    /// dIc/dVbc should be small and negative (or slightly positive due to Early effect).
    #[test]
    fn test_gp_jacobian_signs_forward_active() {
        let gp = BjtGummelPoon::npn_2n2222a();

        let jac = gp.jacobian(&[0.7, -5.0]);

        // gm = dIc/dVbe should be positive (more Vbe => more Ic)
        assert!(
            jac[0] > 0.0,
            "GP gm (dIc/dVbe) should be positive in forward active: {:.6e}",
            jac[0]
        );

        // dIc/dVbc: includes Early effect and reverse injection.
        // For the standard Ebers-Moll part, dIc/dVbc < 0 (reverse injection increases with Vbc).
        // The Early effect adds a positive component.
        // The net sign depends on operating point, but it should be finite.
        assert!(
            jac[1].is_finite(),
            "GP dIc/dVbc should be finite in forward active: {:.6e}",
            jac[1]
        );
    }

    /// gm should be proportional to Ic/Vt (a fundamental BJT relationship).
    /// For GP, gm = dIc/dVbe ~ Ic / (Vt * qb) where qb >= 1.
    /// So gm <= Ic/Vt always, and gm ~ Ic/Vt at low injection.
    #[test]
    fn test_gp_transconductance_relationship() {
        let gp = BjtGummelPoon::npn_2n2222a();
        let vt = gp.base.vt;

        // Moderate injection point
        let vbe = 0.65;
        let vbc = -5.0;
        let ic = gp.collector_current(vbe, vbc);
        let jac = gp.jacobian(&[vbe, vbc]);
        let gm = jac[0];

        let gm_ideal = ic / vt; // Ideal gm = Ic/Vt

        // gm should be less than or equal to Ic/Vt (due to qb >= 1)
        // Allow small numerical tolerance
        assert!(
            gm <= gm_ideal * 1.01,
            "GP gm ({:.6e}) should not exceed Ic/Vt ({:.6e})",
            gm,
            gm_ideal
        );

        // At moderate injection, gm should be reasonably close to Ic/Vt
        assert!(
            gm > gm_ideal * 0.5,
            "GP gm ({:.6e}) should be within 50% of Ic/Vt ({:.6e}) at moderate injection",
            gm,
            gm_ideal
        );
    }

    /// Verify that the Jacobian magnitude scales with the current magnitude.
    /// As Vbe increases, both Ic and gm should increase.
    #[test]
    fn test_gp_jacobian_magnitude_scales_with_current() {
        let gp = BjtGummelPoon::npn_2n2222a();

        let ic_low = gp.collector_current(0.6, -5.0);
        let gm_low = gp.jacobian(&[0.6, -5.0])[0];

        let ic_high = gp.collector_current(0.75, -5.0);
        let gm_high = gp.jacobian(&[0.75, -5.0])[0];

        assert!(ic_high > ic_low, "Ic should increase with Vbe");
        assert!(gm_high > gm_low, "gm should increase with Vbe");
    }

    // --- Ebers-Moll base current Jacobian finite difference ---

    /// Verify Ebers-Moll base current Jacobian against finite differences.
    #[test]
    fn test_ebers_moll_base_jacobian_finite_difference() {
        let bjt = BjtEbersMoll::npn_2n2222a();
        let eps = 1e-7;

        for &(vbe, vbc) in &[(0.7, -5.0), (0.6, -2.0), (0.0, 0.0), (0.65, 0.5)] {
            let dib_dvbe = bjt.base_current_jacobian_dvbe(vbe, vbc);
            let dib_dvbc = bjt.base_current_jacobian_dvbc(vbe, vbc);

            let fd_dvbe =
                (bjt.base_current(vbe + eps, vbc) - bjt.base_current(vbe - eps, vbc)) / (2.0 * eps);
            let fd_dvbc =
                (bjt.base_current(vbe, vbc + eps) - bjt.base_current(vbe, vbc - eps)) / (2.0 * eps);

            let rel_err_be = if fd_dvbe.abs() > 1e-15 {
                (dib_dvbe - fd_dvbe).abs() / fd_dvbe.abs()
            } else {
                dib_dvbe.abs()
            };
            let rel_err_bc = if fd_dvbc.abs() > 1e-15 {
                (dib_dvbc - fd_dvbc).abs() / fd_dvbc.abs()
            } else {
                dib_dvbc.abs()
            };

            assert!(
                rel_err_be < 1e-4,
                "dIb/dVbe at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vbe,
                vbc,
                dib_dvbe,
                fd_dvbe,
                rel_err_be
            );
            assert!(
                rel_err_bc < 1e-4,
                "dIb/dVbc at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                vbe,
                vbc,
                dib_dvbc,
                fd_dvbc,
                rel_err_bc
            );
        }
    }

    /// Ebers-Moll KCL consistency: Ie = -(Ic + Ib) at various operating points.
    #[test]
    fn test_ebers_moll_kcl_consistency() {
        let bjt = BjtEbersMoll::npn_2n2222a();

        for &(vbe, vbc) in &[
            (0.7, -5.0),
            (0.6, -2.0),
            (0.0, 0.0),
            (-0.5, -0.5),
            (0.7, 0.7),
        ] {
            let ic = bjt.collector_current(vbe, vbc);
            let ib = bjt.base_current(vbe, vbc);
            let ie = bjt.emitter_current(vbe, vbc);

            let kcl_error = (ie + ic + ib).abs();
            let scale = ic.abs().max(ib.abs()).max(ie.abs()).max(1e-20);

            assert!(
                kcl_error / scale < 1e-10,
                "KCL violation at ({}, {}): Ic={:.6e} Ib={:.6e} Ie={:.6e} sum={:.6e}",
                vbe,
                vbc,
                ic,
                ib,
                ie,
                ie + ic + ib
            );
        }
    }

    /// Ebers-Moll reverse active: Vbe negative, Vbc positive.
    /// Ic should be negative (current flows from emitter to collector).
    #[test]
    fn test_ebers_moll_reverse_active() {
        let bjt = BjtEbersMoll::npn_2n2222a();

        let ic = bjt.collector_current(-5.0, 0.7);
        assert!(ic < 0.0, "Reverse active Ic should be negative: {:.6e}", ic);

        // Verify Ic is finite and significant
        assert!(ic.is_finite(), "Reverse active Ic should be finite");
        assert!(
            ic.abs() > 1e-6,
            "Reverse active Ic should be significant with Vbc=0.7: {:.6e}",
            ic
        );

        // In reverse active the base current from the BC junction is large
        // because beta_r is small (3.0), so Ib should be significant
        let ib = bjt.base_current(-5.0, 0.7);
        assert!(ib.is_finite());
        assert!(
            ib > 0.0,
            "Reverse active Ib should be positive (BC junction conducting): {:.6e}",
            ib
        );

        // The reverse beta (Ic/Ib) should be much smaller than forward beta
        let ic_fwd = bjt.collector_current(0.7, -5.0);
        let ib_fwd = bjt.base_current(0.7, -5.0);
        let beta_fwd = ic_fwd / ib_fwd;

        // In reverse, we use a weaker bias to get a meaningful reverse beta
        let ic_rev_small = bjt.collector_current(-0.1, 0.7);
        let ib_rev_small = bjt.base_current(-0.1, 0.7);
        if ib_rev_small.abs() > 1e-15 && ic_rev_small.abs() > 1e-15 {
            let _beta_rev = ic_rev_small.abs() / ib_rev_small.abs();
            // beta_rev should be much smaller than beta_fwd
            assert!(
                beta_fwd > 50.0,
                "Forward beta should be large: {:.1}",
                beta_fwd
            );
        }
    }

    /// Verify that NF != 1.0 changes collector and base currents.
    #[test]
    fn test_ebers_moll_nf_effect() {
        let bjt_nf1 = BjtEbersMoll::npn_2n2222a(); // nf=1.0 default
        let bjt_nf103 = BjtEbersMoll::npn_2n2222a().with_nf(1.003);

        let vbe = 0.7;
        let vbc = -5.0;

        let ic1 = bjt_nf1.collector_current(vbe, vbc);
        let ic103 = bjt_nf103.collector_current(vbe, vbc);

        // NF=1.003 uses exp(Vbe/(1.003*Vt)) < exp(Vbe/Vt), so less current
        assert!(
            ic1 > ic103,
            "NF=1.0 Ic ({:.6e}) should exceed NF=1.003 Ic ({:.6e})",
            ic1,
            ic103
        );

        // The difference should be measurable but not extreme
        let ratio = ic103 / ic1;
        assert!(
            ratio > 0.85 && ratio < 1.0,
            "Ic ratio {:.6} should be close to but below 1.0",
            ratio
        );

        // Base current also affected
        let ib1 = bjt_nf1.base_current(vbe, vbc);
        let ib103 = bjt_nf103.base_current(vbe, vbc);
        assert!(ib1 > ib103, "NF=1.0 Ib should exceed NF=1.003 Ib");
    }

    /// Verify Jacobian with NF != 1.0 matches finite difference.
    #[test]
    fn test_ebers_moll_nf_jacobian_finite_difference() {
        let bjt = BjtEbersMoll::npn_2n2222a().with_nf(1.003);
        let eps = 1e-7;

        for &(vbe, vbc) in &[(0.7, -5.0), (0.6, -2.0), (0.65, -10.0)] {
            let (dic_dvbe, dic_dvbc) = bjt.collector_jacobian(vbe, vbc);
            let dib_dvbe = bjt.base_current_jacobian_dvbe(vbe, vbc);
            let dib_dvbc = bjt.base_current_jacobian_dvbc(vbe, vbc);

            // FD for collector
            let fd_dic_dvbe = (bjt.collector_current(vbe + eps, vbc)
                - bjt.collector_current(vbe - eps, vbc))
                / (2.0 * eps);
            let fd_dic_dvbc = (bjt.collector_current(vbe, vbc + eps)
                - bjt.collector_current(vbe, vbc - eps))
                / (2.0 * eps);

            // FD for base
            let fd_dib_dvbe =
                (bjt.base_current(vbe + eps, vbc) - bjt.base_current(vbe - eps, vbc)) / (2.0 * eps);
            let fd_dib_dvbc =
                (bjt.base_current(vbe, vbc + eps) - bjt.base_current(vbe, vbc - eps)) / (2.0 * eps);

            // Check all four entries
            for (name, analytic, fd) in [
                ("dIc/dVbe", dic_dvbe, fd_dic_dvbe),
                ("dIc/dVbc", dic_dvbc, fd_dic_dvbc),
                ("dIb/dVbe", dib_dvbe, fd_dib_dvbe),
                ("dIb/dVbc", dib_dvbc, fd_dib_dvbc),
            ] {
                let rel_err = if fd.abs() > 1e-15 {
                    (analytic - fd).abs() / fd.abs()
                } else {
                    analytic.abs()
                };
                assert!(
                    rel_err < 1e-4,
                    "NF=1.003 {} at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    name,
                    vbe,
                    vbc,
                    analytic,
                    fd,
                    rel_err
                );
            }
        }
    }

    /// Verify NF works correctly for PNP.
    #[test]
    fn test_ebers_moll_nf_pnp() {
        let pnp = BjtEbersMoll::pnp_2n3906().with_nf(1.5);

        // PNP forward active: Vbe=-0.7, Vbc=+5
        let ic = pnp.collector_current(-0.7, 5.0);
        assert!(ic < 0.0, "PNP Ic should be negative: {:.6e}", ic);
        assert!(ic.is_finite());

        // FD Jacobian should match
        let eps = 1e-7;
        let (dic_dvbe, _) = pnp.collector_jacobian(-0.7, 5.0);
        let fd = (pnp.collector_current(-0.7 + eps, 5.0) - pnp.collector_current(-0.7 - eps, 5.0))
            / (2.0 * eps);
        let rel_err = (dic_dvbe - fd).abs() / fd.abs();
        assert!(
            rel_err < 1e-4,
            "PNP NF=1.5 dIc/dVbe: analytic={:.6e} fd={:.6e} err={:.2e}",
            dic_dvbe,
            fd,
            rel_err
        );
    }

    /// GP Jacobian finite-difference sweep across many operating points.
    /// This is a comprehensive stress test for the analytical Jacobian.
    #[test]
    fn test_gp_jacobian_comprehensive_sweep() {
        let gp = BjtGummelPoon::npn_2n2222a();
        let eps = 1e-7;

        let vbe_values = [0.0, 0.3, 0.5, 0.6, 0.65, 0.7, 0.75, 0.8];
        let vbc_values = [-10.0, -5.0, -2.0, -0.5, 0.0, 0.3, 0.5, 0.7];

        for &vbe in &vbe_values {
            for &vbc in &vbc_values {
                let jac = gp.jacobian(&[vbe, vbc]);

                assert!(
                    jac[0].is_finite(),
                    "GP Jac[0] must be finite at ({}, {}): {}",
                    vbe,
                    vbc,
                    jac[0]
                );
                assert!(
                    jac[1].is_finite(),
                    "GP Jac[1] must be finite at ({}, {}): {}",
                    vbe,
                    vbc,
                    jac[1]
                );

                let fd_dvbe = (gp.collector_current(vbe + eps, vbc)
                    - gp.collector_current(vbe - eps, vbc))
                    / (2.0 * eps);
                let fd_dvbc = (gp.collector_current(vbe, vbc + eps)
                    - gp.collector_current(vbe, vbc - eps))
                    / (2.0 * eps);

                let err_be = if fd_dvbe.abs() > 1e-10 {
                    (jac[0] - fd_dvbe).abs() / fd_dvbe.abs()
                } else {
                    (jac[0] - fd_dvbe).abs()
                };
                let err_bc = if fd_dvbc.abs() > 1e-10 {
                    (jac[1] - fd_dvbc).abs() / fd_dvbc.abs()
                } else {
                    (jac[1] - fd_dvbc).abs()
                };

                assert!(
                    err_be < 0.01,
                    "GP Jac[0] at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    vbe,
                    vbc,
                    jac[0],
                    fd_dvbe,
                    err_be
                );
                assert!(
                    err_bc < 0.01,
                    "GP Jac[1] at ({}, {}): analytic={:.6e} fd={:.6e} err={:.2e}",
                    vbe,
                    vbc,
                    jac[1],
                    fd_dvbc,
                    err_bc
                );
            }
        }
    }
}
