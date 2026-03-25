//! Parameterized nonlinear device models for analog circuit simulation.
//!
//! Layer 2 of the melange stack. Each device provides `i(v)` and `di/dv` for use in
//! Newton-Raphson solvers. Models are parameterized from SPICE `.model` cards.
//!
//! # Supported Devices
//!
//! | Device | Struct | Dimensions | Model |
//! |--------|--------|-----------|-------|
//! | Diode | [`DiodeShockley`], [`DiodeWithRs`], [`Led`] | 1D | Shockley equation, optional Rs, BV |
//! | BJT | [`BjtEbersMoll`], [`BjtGummelPoon`] | 2D (Vbe, Vbc) | Ebers-Moll / Gummel-Poon |
//! | JFET | [`Jfet`] | 2D (Vgs, Vds) | Shichman-Hodges |
//! | MOSFET | [`Mosfet`] | 2D (Vgs, Vds) | Level 1 SPICE |
//! | Triode | [`KorenTriode`] | 2D (Vgk, Vpk) | Koren + Leach grid current |
//! | VCA | [`Vca`] | 2D (Vsig, Vctrl) | THAT 2180 exponential |
//! | Op-amp | [`SimpleOpamp`], [`IdealOpamp`] | linear | Boyle VCCS macromodel |
//! | LDR | [`CdsLdr`] | — | VTL5C3/4, NSL-32 photocell |
//!
//! # Traits
//!
//! - [`NonlinearDevice<N>`] — core trait: `current(&[f64; N]) -> f64` and `jacobian(&[f64; N]) -> [f64; N]`
//! - [`TwoTerminalDevice`] — convenience trait for N=1 devices (auto-implemented)
//!
//! # Catalog
//!
//! The [`catalog`] module provides lookup tables for common device models (2N2222, 1N4148,
//! 12AX7, etc.) with pre-populated SPICE parameters.

pub mod bjt;
pub mod catalog;
pub mod diode;
pub mod jfet;
pub mod ldr;
pub mod mosfet;
pub mod opamp;
pub mod tube;
pub mod vca;

pub use bjt::{classify_region, BjtEbersMoll, BjtGummelPoon, BjtPolarity, BjtRegion};
pub use diode::{DiodeShockley, DiodeWithRs, Led};
pub use jfet::{Jfet, JfetChannel};
pub use ldr::CdsLdr;
pub use mosfet::{ChannelType as MosfetChannelType, Mosfet};
pub use opamp::{IdealOpamp, SimpleOpamp};
pub use tube::KorenTriode;
pub use vca::Vca;

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

pub use melange_primitives::{thermal_voltage, VT_ROOM};

/// Safeguards for numerical stability in device models.
pub mod safeguards {
    /// Minimum voltage for exponential calculations.
    /// Prevents exp() overflow for large negative voltages.
    pub const MIN_EXP_V: f64 = -40.0; // exp(-40) ≈ 4e-18

    /// Maximum voltage for exponential calculations.
    /// Prevents exp() overflow for large positive voltages.
    pub const MAX_EXP_V: f64 = 40.0; // exp(40) ≈ 2e17

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
    fn test_safeguards() {
        assert!(safeguards::safe_exp(100.0) < f64::INFINITY);
        assert!(safeguards::safe_exp(-100.0) > 0.0);
    }
}
