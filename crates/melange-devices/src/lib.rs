// melange-devices: Parameterized nonlinear component models
//
// Layer 2 of the melange stack. Each device implements the NonlinearDevice trait
// providing i(v) and di/dv for use in MNA/DK solvers. Models include:
// - BJT (Ebers-Moll, Gummel-Poon) with SPICE model card import
// - Vacuum tubes (Koren triode/pentode)
// - Diodes (Shockley equation)
// - MOSFETs, JFETs
// - Opamps (Boyle macromodel)
// - CdS LDR photocell (asymmetric attack/release, power-law resistance)

pub mod bjt;
pub mod diode;
pub mod tube;
pub mod ldr;
pub mod opamp;
pub mod mosfet;
pub mod jfet;

pub use bjt::{BjtEbersMoll, BjtGummelPoon, BjtPolarity, BjtRegion, classify_region};
pub use diode::{DiodeShockley, DiodeWithRs, Led};
pub use tube::KorenTriode;
pub use jfet::{Jfet, JfetChannel};
pub use mosfet::{Mosfet, ChannelType as MosfetChannelType};
pub use ldr::CdsLdr;
pub use opamp::{IdealOpamp, SimpleOpamp};

/// A nonlinear circuit element for use in MNA/DK solvers.
///
/// This trait abstracts over all two-terminal nonlinear devices.
/// For devices with more terminals (BJT, tube), see the specific
/// device implementations.
///
/// # Type Parameters
/// The generic `N` parameter represents the number of controlling voltages.
/// For simple two-terminal devices, N=1. For BJTs, N=2 (Vbe, Vbc).
pub trait NonlinearDevice<const N: usize> {
    /// Current as a function of terminal voltages.
    ///
    /// Returns the current into the device's terminal(s).
    fn current(&self, v: &[f64; N]) -> f64;

    /// Partial derivatives of current with respect to each voltage.
    ///
    /// Returns the conductance values (di/dv) for use in the
    /// Newton-Raphson Jacobian.
    fn jacobian(&self, v: &[f64; N]) -> [f64; N];

    /// Combined current and Jacobian for efficiency.
    ///
    /// Default implementation calls current() and jacobian() separately.
    fn eval(&self, v: &[f64; N]) -> (f64, [f64; N]) {
        (self.current(v), self.jacobian(v))
    }
}

/// A two-terminal nonlinear device (simple case).
///
/// This is a convenience trait for devices where N=1.
pub trait TwoTerminalDevice: NonlinearDevice<1> {
    /// Current as function of single voltage.
    fn i(&self, v: f64) -> f64;

    /// Derivative di/dv.
    fn g(&self, v: f64) -> f64;
}

impl<T: NonlinearDevice<1>> TwoTerminalDevice for T {
    fn i(&self, v: f64) -> f64 {
        self.current(&[v])
    }

    fn g(&self, v: f64) -> f64 {
        self.jacobian(&[v])[0]
    }
}

pub use melange_primitives::VT_ROOM;

/// Compute thermal voltage at a given temperature.
pub fn thermal_voltage(temp_c: f64) -> f64 {
    const K_BOLTZMANN: f64 = 1.380649e-23;   // J/K
    const Q_ELECTRON: f64 = 1.602176634e-19; // C
    const T_ABS_ZERO: f64 = 273.15;          // K

    let temp_k = temp_c + T_ABS_ZERO;
    K_BOLTZMANN * temp_k / Q_ELECTRON
}

/// Safeguards for numerical stability in device models.
pub mod safeguards {
    /// Minimum voltage for exponential calculations.
    /// Prevents exp() overflow for large negative voltages.
    pub const MIN_EXP_V: f64 = -40.0;  // exp(-40) ≈ 4e-18

    /// Maximum voltage for exponential calculations.
    /// Prevents exp() overflow for large positive voltages.
    pub const MAX_EXP_V: f64 = 40.0;   // exp(40) ≈ 2e17

    /// Minimum conductance to prevent divide-by-zero.
    pub const MIN_CONDUCTANCE: f64 = 1e-15;

    /// Limit voltage for safe exponential calculation.
    #[inline(always)]
    pub fn limit_exp_v(v: f64, vt: f64) -> f64 {
        let v_over_vt = v / vt;
        if v_over_vt < MIN_EXP_V {
            MIN_EXP_V * vt
        } else if v_over_vt > MAX_EXP_V {
            MAX_EXP_V * vt
        } else {
            v
        }
    }

    /// Safe exp() that limits the argument.
    #[inline(always)]
    pub fn safe_exp(v: f64) -> f64 {
        if v < MIN_EXP_V {
            MIN_EXP_V.exp()
        } else if v > MAX_EXP_V {
            MAX_EXP_V.exp()
        } else {
            v.exp()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_thermal_voltage() {
        let vt = thermal_voltage(27.0);
        // Allow for floating point precision in constants
        assert!((vt - VT_ROOM).abs() < 1e-4, "VT = {}, expected ~{}", vt, VT_ROOM);
    }

    #[test]
    fn test_safeguards() {
        assert!(safeguards::safe_exp(100.0) < f64::INFINITY);
        assert!(safeguards::safe_exp(-100.0) > 0.0);
    }
}
