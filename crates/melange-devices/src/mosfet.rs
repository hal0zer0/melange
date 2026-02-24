//! MOSFET transistor models.
//!
//! Simple square-law and level-1 SPICE models.

use crate::NonlinearDevice;

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
    pub fn new(channel: ChannelType, vt: f64, kp: f64, lambda: f64) -> Self {
        Self { channel, vt, kp, lambda }
    }

    /// 2N7000 N-channel MOSFET (common small-signal).
    pub fn n_2n7000() -> Self {
        Self::new(ChannelType::N, 2.0, 0.1, 0.01)
    }

    /// BS170 N-channel MOSFET.
    pub fn n_bs170() -> Self {
        Self::new(ChannelType::N, 0.8, 0.05, 0.01)
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

        if vgs_eff <= self.vt {
            return 0.0;  // Cutoff
        }

        let vov = vgs_eff - self.vt;  // Overdrive voltage

        if vds_eff < vov {
            // Linear (triode) region
            let id = self.kp * ((vgs_eff - self.vt) * vds_eff - 0.5 * vds_eff * vds_eff);
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

        if vgs_eff <= self.vt {
            return (0.0, 0.0);
        }

        let vov = vgs_eff - self.vt;

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

        // Apply chain rule
        (s * d_id_d_vgs, s * d_id_d_vds)
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
}
