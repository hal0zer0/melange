//! Voltage-Controlled Amplifier (VCA) model.
//!
//! Models a THAT 2180 / DBX 2150 Blackmer-style current-mode exponential
//! gain element with 4 terminals:
//! - sig+, sig-: signal current path
//! - ctrl+, ctrl-: control voltage (high impedance, draws no current)
//!
//! Gain law: `I_signal = G0 * exp(-V_control / VSCALE) * V_signal`
//! Control current: `I_control = 0` (ideal high-Z control input)

use crate::safeguards;

/// THAT 2180-style Blackmer VCA parameters.
#[derive(Debug, Clone)]
pub struct Vca {
    /// Control voltage scaling (V/neper). THAT 2180A default: 0.05298
    pub vscale: f64,
    /// Unity-gain conductance (S). Default: 1.0
    pub g0: f64,
    /// THD coefficient for gain-dependent cubic distortion.
    /// 0.0 = ideal (default), typical THAT 2180: 0.001
    pub thd: f64,
}

impl Default for Vca {
    fn default() -> Self {
        Self {
            vscale: 0.05298, // THAT 2180A: 6.1 mV/dB → 6.1e-3 / (ln(10)/20) V/neper
            g0: 1.0,
            thd: 0.0,
        }
    }
}

impl Vca {
    pub fn new(vscale: f64, g0: f64) -> Self {
        Self {
            vscale,
            g0,
            thd: 0.0,
        }
    }

    pub fn new_with_thd(vscale: f64, g0: f64, thd: f64) -> Self {
        Self { vscale, g0, thd }
    }

    /// Compute signal current: I = G0 * exp(-Vc / Vscale) * V_sig
    /// With THD: I = gain * (V_sig + thd_factor * V_sig^3)
    /// where thd_factor = thd * (1 - gain.min(1))
    pub fn current(&self, v_sig: f64, v_ctrl: f64) -> f64 {
        let gain = self.gain(v_ctrl);
        if self.thd == 0.0 {
            gain * v_sig
        } else {
            let thd_factor = self.thd * (1.0 - gain.min(1.0));
            gain * (v_sig + thd_factor * v_sig * v_sig * v_sig)
        }
    }

    /// Compute gain: G = G0 * exp(-Vc / Vscale)
    pub fn gain(&self, v_ctrl: f64) -> f64 {
        self.g0 * safeguards::safe_exp(-v_ctrl / self.vscale)
    }

    /// Compute 2x2 Jacobian [dI_sig/dV_sig, dI_sig/dV_ctrl, dI_ctrl/dV_sig, dI_ctrl/dV_ctrl]
    pub fn jacobian(&self, v_sig: f64, v_ctrl: f64) -> [f64; 4] {
        let gain = self.gain(v_ctrl);
        if self.thd == 0.0 {
            [
                gain,                        // dI_sig/dV_sig
                -gain * v_sig / self.vscale, // dI_sig/dV_ctrl
                0.0,                         // dI_ctrl/dV_sig (no control current)
                0.0,                         // dI_ctrl/dV_ctrl
            ]
        } else {
            let thd_factor = self.thd * (1.0 - gain.min(1.0));
            let sig_with_thd = v_sig + thd_factor * v_sig * v_sig * v_sig;
            let di_dvsig = gain * (1.0 + 3.0 * thd_factor * v_sig * v_sig);
            let di_dvctrl = -gain * sig_with_thd / self.vscale;
            [
                di_dvsig,  // dI_sig/dV_sig
                di_dvctrl, // dI_sig/dV_ctrl
                0.0,       // dI_ctrl/dV_sig (no control current)
                0.0,       // dI_ctrl/dV_ctrl
            ]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gain_at_zero_ctrl() {
        let vca = Vca::new(0.00528, 1.0);
        let g = vca.gain(0.0);
        assert!(
            (g - 1.0).abs() < 1e-12,
            "Gain at V_ctrl=0 should be G0=1.0, got {}",
            g
        );
    }

    #[test]
    fn test_gain_at_vscale() {
        let vca = Vca::new(0.00528, 1.0);
        let g = vca.gain(0.00528);
        let expected = 1.0 / std::f64::consts::E;
        assert!(
            (g - expected).abs() < 1e-10,
            "Gain at V_ctrl=VSCALE should be G0/e={}, got {}",
            expected,
            g
        );
    }

    #[test]
    fn test_jacobian_vs_finite_differences() {
        let vca = Vca::new(0.00528, 1.0);
        let v_sig = 0.1;
        let v_ctrl = 0.01;
        let h = 1e-7;

        let jac = vca.jacobian(v_sig, v_ctrl);

        // dI_sig/dV_sig (index 0)
        let di_dvsig =
            (vca.current(v_sig + h, v_ctrl) - vca.current(v_sig - h, v_ctrl)) / (2.0 * h);
        assert!(
            (jac[0] - di_dvsig).abs() < 1e-4,
            "dI_sig/dV_sig: analytical={}, numerical={}",
            jac[0],
            di_dvsig
        );

        // dI_sig/dV_ctrl (index 1)
        let di_dvctrl =
            (vca.current(v_sig, v_ctrl + h) - vca.current(v_sig, v_ctrl - h)) / (2.0 * h);
        assert!(
            (jac[1] - di_dvctrl).abs() / (di_dvctrl.abs() + 1e-15) < 1e-4,
            "dI_sig/dV_ctrl: analytical={}, numerical={}",
            jac[1],
            di_dvctrl
        );

        // dI_ctrl/dV_sig (index 2) — should be 0
        assert_eq!(jac[2], 0.0, "dI_ctrl/dV_sig should be 0");

        // dI_ctrl/dV_ctrl (index 3) — should be 0
        assert_eq!(jac[3], 0.0, "dI_ctrl/dV_ctrl should be 0");
    }

    #[test]
    fn test_clamping_large_ctrl() {
        let vca = Vca::new(0.00528, 1.0);

        // Very large positive V_ctrl → gain should be very small but finite
        let g_large = vca.gain(1000.0);
        assert!(
            g_large.is_finite(),
            "Gain should be finite for large V_ctrl"
        );
        assert!(g_large >= 0.0, "Gain should be non-negative");

        // Very large negative V_ctrl → gain should be large but finite
        let g_neg = vca.gain(-1000.0);
        assert!(
            g_neg.is_finite(),
            "Gain should be finite for large negative V_ctrl"
        );
        assert!(g_neg > 0.0, "Gain should be positive");

        // Current should also be finite
        let i = vca.current(1.0, 1000.0);
        assert!(i.is_finite(), "Current should be finite for large V_ctrl");
        let i_neg = vca.current(1.0, -1000.0);
        assert!(
            i_neg.is_finite(),
            "Current should be finite for large negative V_ctrl"
        );
    }

    #[test]
    fn test_symmetry_negative_ctrl_increases_gain() {
        let vca = Vca::default();
        let g_zero = vca.gain(0.0);
        let g_pos = vca.gain(0.01);
        let g_neg = vca.gain(-0.01);

        assert!(
            g_neg > g_zero,
            "Negative V_ctrl should increase gain: g_neg={} > g_zero={}",
            g_neg,
            g_zero
        );
        assert!(
            g_pos < g_zero,
            "Positive V_ctrl should decrease gain: g_pos={} < g_zero={}",
            g_pos,
            g_zero
        );
    }

    #[test]
    fn test_thd_zero_matches_original() {
        // With thd=0, new_with_thd should behave identically to new()
        let vca_orig = Vca::new(0.00528, 1.0);
        let vca_thd0 = Vca::new_with_thd(0.00528, 1.0, 0.0);
        let v_sig = 0.1;
        let v_ctrl = 0.01;

        assert_eq!(
            vca_orig.current(v_sig, v_ctrl),
            vca_thd0.current(v_sig, v_ctrl)
        );
        assert_eq!(
            vca_orig.jacobian(v_sig, v_ctrl),
            vca_thd0.jacobian(v_sig, v_ctrl)
        );
    }

    #[test]
    fn test_thd_increases_with_gain_reduction() {
        let thd = 0.001; // typical THAT 2180
        let vca = Vca::new_with_thd(0.05298, 1.0, thd);

        // At zero ctrl, gain=1.0, thd_factor=0 → output should be linear
        let v_sig = 0.5;
        let i_unity = vca.current(v_sig, 0.0);
        let gain_unity = vca.gain(0.0);
        let expected_linear = gain_unity * v_sig;
        assert!(
            (i_unity - expected_linear).abs() < 1e-12,
            "At unity gain, THD should be zero: i={}, expected={}",
            i_unity,
            expected_linear
        );

        // At positive ctrl (gain < 1), thd_factor > 0 → cubic distortion present
        let v_ctrl_reduced = 0.1; // gain reduced well below 1
        let gain_reduced = vca.gain(v_ctrl_reduced);
        assert!(gain_reduced < 1.0, "gain should be < 1 for positive v_ctrl");

        let i_with_thd = vca.current(v_sig, v_ctrl_reduced);
        let i_without_thd = gain_reduced * v_sig; // what it would be without THD
        let distortion = (i_with_thd - i_without_thd).abs();

        assert!(
            distortion > 0.0,
            "Should have nonzero distortion when gain < 1 and thd > 0"
        );

        // Distortion should be cubic (proportional to v_sig^3)
        let v_sig2 = 0.25;
        let i_with_thd2 = vca.current(v_sig2, v_ctrl_reduced);
        let i_without_thd2 = gain_reduced * v_sig2;
        let distortion2 = (i_with_thd2 - i_without_thd2).abs();

        // Ratio should be approximately (0.5/0.25)^3 = 8
        let ratio = distortion / distortion2;
        assert!(
            (ratio - 8.0).abs() < 0.1,
            "Distortion should scale as v_sig^3: ratio={}, expected ~8",
            ratio
        );
    }

    #[test]
    fn test_thd_jacobian_vs_finite_differences() {
        let vca = Vca::new_with_thd(0.05298, 1.0, 0.001);
        let v_sig = 0.3;
        let v_ctrl = 0.05;
        let h = 1e-7;

        let jac = vca.jacobian(v_sig, v_ctrl);

        // dI_sig/dV_sig (index 0)
        let di_dvsig =
            (vca.current(v_sig + h, v_ctrl) - vca.current(v_sig - h, v_ctrl)) / (2.0 * h);
        assert!(
            (jac[0] - di_dvsig).abs() < 1e-4,
            "dI_sig/dV_sig with THD: analytical={}, numerical={}",
            jac[0],
            di_dvsig
        );

        // dI_sig/dV_ctrl (index 1)
        let di_dvctrl =
            (vca.current(v_sig, v_ctrl + h) - vca.current(v_sig, v_ctrl - h)) / (2.0 * h);
        assert!(
            (jac[1] - di_dvctrl).abs() / (di_dvctrl.abs() + 1e-15) < 1e-4,
            "dI_sig/dV_ctrl with THD: analytical={}, numerical={}",
            jac[1],
            di_dvctrl
        );

        // Control port still zero
        assert_eq!(jac[2], 0.0);
        assert_eq!(jac[3], 0.0);
    }
}
