//! Operational amplifier models.

use crate::NonlinearDevice;

/// Ideal op-amp model (infinite gain, infinite bandwidth).
///
/// Used for basic circuit analysis. In a real solver,
/// ideal op-amps are handled as constraints in the MNA matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdealOpamp;

/// Boyle op-amp macromodel.
///
/// A simplified macromodel that captures key op-amp behaviors:
/// - Finite gain and bandwidth
/// - Slew rate limiting
/// - Output saturation
/// - Input offset (optional)
///
/// **Placeholder**: Does not implement `NonlinearDevice`. Not used by solver or codegen.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoyleOpamp {
    /// DC open-loop gain
    pub gain: f64,
    /// Dominant pole frequency [Hz]
    pub dominant_pole: f64,
    /// Second pole frequency [Hz] (for stability)
    pub second_pole: f64,
    /// Maximum output voltage [V]
    pub vout_max: f64,
    /// Minimum output voltage [V]
    pub vout_min: f64,
    /// Slew rate [V/µs]
    pub slew_rate: f64,
    /// Input offset voltage [V]
    pub voffset: f64,
}

impl BoyleOpamp {
    /// Create a new Boyle macromodel.
    pub fn new(gain: f64, dominant_pole: f64, vout_max: f64, slew_rate: f64) -> Self {
        Self {
            gain,
            dominant_pole,
            second_pole: dominant_pole * 10.0, // Typical ratio
            vout_max,
            vout_min: -vout_max,
            slew_rate,
            voffset: 0.0,
        }
    }

    /// 741-style op-amp parameters.
    pub fn lm741() -> Self {
        Self::new(200e3, 5.0, 13.0, 0.5)
    }

    /// TL072-style (FET input, higher bandwidth).
    pub fn tl072() -> Self {
        Self::new(200e3, 20.0, 13.0, 13.0)
    }

    /// NE5532 (low noise, higher slew rate).
    pub fn ne5532() -> Self {
        Self::new(100e3, 10.0, 13.0, 9.0)
    }
}

/// Simple op-amp model for transient simulation.
///
/// Single-pole approximation with output limiting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimpleOpamp {
    pub gain: f64,
    pub bandwidth: f64, // Hz
    pub vout_max: f64,
    pub vout_min: f64,
}

impl SimpleOpamp {
    /// Create a simple op-amp model.
    pub fn new(gain: f64, bandwidth: f64, vout_max: f64) -> Self {
        Self {
            gain,
            bandwidth,
            vout_max,
            vout_min: -vout_max,
        }
    }
}

impl NonlinearDevice<1> for SimpleOpamp {
    /// Linear approximation: Iout = Gm * Vin
    fn current(&self, v: &[f64; 1]) -> f64 {
        // Output voltage (limited), returned as current through unity load
        (v[0] * self.gain).clamp(self.vout_min, self.vout_max)
    }

    fn jacobian(&self, v: &[f64; 1]) -> [f64; 1] {
        let vout = v[0] * self.gain;
        if vout >= self.vout_max || vout <= self.vout_min {
            [0.0] // Saturated — output does not change with input
        } else {
            [self.gain]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_opamp_741() {
        let opa = BoyleOpamp::lm741();
        assert!(opa.gain > 100e3);
        assert_eq!(opa.vout_max, 13.0);
    }

    #[test]
    fn test_simple_opamp() {
        let opa = SimpleOpamp::new(100e3, 10e6, 12.0);

        // Small signal: linear region
        let i = opa.current(&[0.1e-3]); // 0.1mV input
        assert!(i > 0.0);

        // Large signal: saturation
        let i_sat = opa.current(&[1.0]);
        assert!(i_sat <= opa.vout_max);
    }

    /// Verify SimpleOpamp linear gain accuracy.
    #[test]
    fn test_simple_opamp_gain() {
        let gain = 200e3;
        let opa = SimpleOpamp::new(gain, 10e6, 12.0);

        // Small signal: output = gain * input
        let vin = 1e-5; // 10µV — well within linear range
        let vout = opa.current(&[vin]);
        let actual_gain = vout / vin;
        assert!(
            (actual_gain - gain).abs() / gain < 1e-6,
            "Gain should be {:.0}, got {:.0}",
            gain,
            actual_gain
        );
    }

    /// Verify positive and negative saturation.
    #[test]
    fn test_simple_opamp_saturation() {
        let opa = SimpleOpamp::new(100e3, 10e6, 12.0);

        // Positive saturation
        let vout_pos = opa.current(&[1.0]); // 1V * 100k = 100kV → clamps to +12
        assert_eq!(
            vout_pos, 12.0,
            "Positive saturation should clamp to vout_max"
        );

        // Negative saturation
        let vout_neg = opa.current(&[-1.0]);
        assert_eq!(
            vout_neg, -12.0,
            "Negative saturation should clamp to vout_min"
        );

        // Just at boundary
        let vin_edge = 12.0 / 100e3; // exactly at saturation
        let vout_edge = opa.current(&[vin_edge]);
        assert!(
            (vout_edge - 12.0).abs() < 1e-6,
            "At saturation boundary: {:.6}",
            vout_edge
        );
    }

    /// Verify Jacobian matches finite difference.
    #[test]
    fn test_simple_opamp_jacobian_fd() {
        let opa = SimpleOpamp::new(100e3, 10e6, 12.0);
        let eps = 1e-9;

        // Linear region
        let vin = 1e-5;
        let jac = opa.jacobian(&[vin]);
        let fd = (opa.current(&[vin + eps]) - opa.current(&[vin - eps])) / (2.0 * eps);
        assert!(
            (jac[0] - fd).abs() / fd.abs() < 1e-4,
            "Linear region: jac={:.6e}, fd={:.6e}",
            jac[0],
            fd
        );

        // Saturated region (Jacobian should be 0)
        let jac_sat = opa.jacobian(&[1.0]);
        assert_eq!(jac_sat[0], 0.0, "Saturated Jacobian should be 0");

        let jac_neg_sat = opa.jacobian(&[-1.0]);
        assert_eq!(
            jac_neg_sat[0], 0.0,
            "Negative saturated Jacobian should be 0"
        );
    }

    /// Verify all preset constructors produce valid models.
    #[test]
    fn test_boyle_presets() {
        for (name, opa) in [
            ("LM741", BoyleOpamp::lm741()),
            ("TL072", BoyleOpamp::tl072()),
            ("NE5532", BoyleOpamp::ne5532()),
        ] {
            assert!(opa.gain > 1e4, "{name}: gain too low: {:.0}", opa.gain);
            assert!(opa.dominant_pole > 0.0, "{name}: invalid pole");
            assert!(
                opa.second_pole > opa.dominant_pole,
                "{name}: second pole should exceed dominant"
            );
            assert!(opa.vout_max > 0.0, "{name}: invalid vout_max");
            assert_eq!(opa.vout_min, -opa.vout_max, "{name}: symmetric rails");
            assert!(opa.slew_rate > 0.0, "{name}: invalid slew rate");
        }
    }

    /// Verify SimpleOpamp zero input gives zero output.
    #[test]
    fn test_simple_opamp_zero() {
        let opa = SimpleOpamp::new(100e3, 10e6, 12.0);
        assert_eq!(opa.current(&[0.0]), 0.0);
        assert_eq!(opa.jacobian(&[0.0])[0], 100e3);
    }
}
