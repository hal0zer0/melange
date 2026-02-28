//! Diode models for circuit simulation.

use crate::{NonlinearDevice, VT_ROOM, safeguards};

/// Shockley diode equation model.
///
/// The classic exponential diode model:
///   I = Is * (exp(V / (n*Vt)) - 1)
///
/// Parameters:
/// - Is: Saturation current (typically 1e-15 to 1e-12 A)
/// - n: Ideality factor (1.0 for ideal, 1.5-2.0 for real diodes)
/// - Vt: Thermal voltage (~26 mV at room temperature)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiodeShockley {
    /// Saturation current [A]
    pub is: f64,
    /// Ideality factor (emission coefficient)
    pub n: f64,
    /// Thermal voltage [V]
    pub vt: f64,
    /// n * vt (precomputed)
    n_vt: f64,
}

impl DiodeShockley {
    /// Create a new diode with the given parameters.
    pub fn new(is: f64, n: f64, vt: f64) -> Self {
        Self {
            is,
            n,
            vt,
            n_vt: n * vt,
        }
    }

    /// Create a diode at room temperature.
    pub fn new_room_temp(is: f64, n: f64) -> Self {
        Self::new(is, n, VT_ROOM)
    }

    /// Create a generic silicon diode with realistic defaults.
    pub fn silicon() -> Self {
        Self::new_room_temp(1e-12, 1.5)
    }

    /// Create a 1N4148 signal diode (well-characterized).
    pub fn silicon_1n4148() -> Self {
        Self::new_room_temp(2.52e-9, 1.752)
    }

    /// Create an ideal silicon diode (theoretical).
    pub fn silicon_ideal() -> Self {
        Self::new_room_temp(1e-14, 1.0)
    }

    /// Create a germanium diode (1N34A-like).
    pub fn germanium() -> Self {
        Self::new_room_temp(2e-7, 1.05)
    }

    /// Create a Schottky diode (1N5819-like).
    pub fn schottky() -> Self {
        Self::new_room_temp(1e-8, 1.04)
    }

    /// Calculate current for a given voltage.
    pub fn current_at(&self, v: f64) -> f64 {
        let v_limited = safeguards::limit_exp_v(v, self.n_vt);
        self.is * ((v_limited / self.n_vt).exp() - 1.0)
    }

    /// Calculate conductance (di/dv) for a given voltage.
    pub fn conductance_at(&self, v: f64) -> f64 {
        let v_limited = safeguards::limit_exp_v(v, self.n_vt);
        // di/dv = (Is / (n*Vt)) * exp(V / (n*Vt))
        (self.is / self.n_vt) * (v_limited / self.n_vt).exp()
    }
}

impl NonlinearDevice<1> for DiodeShockley {
    fn current(&self, v: &[f64; 1]) -> f64 {
        self.current_at(v[0])
    }

    fn jacobian(&self, v: &[f64; 1]) -> [f64; 1] {
        [self.conductance_at(v[0])]
    }
}

/// Diode with series resistance.
///
/// More accurate model that includes the ohmic resistance of the
/// semiconductor material and contacts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiodeWithRs {
    /// Base diode model
    pub diode: DiodeShockley,
    /// Series resistance [Ohms]
    pub rs: f64,
}

impl DiodeWithRs {
    /// Create a new diode with series resistance.
    pub fn new(diode: DiodeShockley, rs: f64) -> Self {
        Self { diode, rs }
    }

    /// Current-voltage relationship.
    ///
    /// Requires solving: I = Is * (exp((V - I*Rs) / (n*Vt)) - 1)
    /// We use an approximate solution: I ≈ diode_current(V) for small Rs
    /// or iterative solution for accuracy.
    pub fn current_at(&self, v: f64) -> f64 {
        // Simple approximation: just use the diode current
        // For more accuracy, could use NR iteration here
        let i_approx = self.diode.current_at(v);
        
        // Better approximation: account for voltage drop across Rs
        let v_diode = v - i_approx * self.rs;
        self.diode.current_at(v_diode)
    }

    /// Conductance with series resistance.
    pub fn conductance_at(&self, v: f64) -> f64 {
        // Evaluate at the corrected junction voltage (consistent with current_at)
        let i_approx = self.diode.current_at(v);
        let v_diode = v - i_approx * self.rs;
        // g_total = g_diode / (1 + g_diode * Rs)
        let g_d = self.diode.conductance_at(v_diode);
        g_d / (1.0 + g_d * self.rs)
    }
}

impl NonlinearDevice<1> for DiodeWithRs {
    fn current(&self, v: &[f64; 1]) -> f64 {
        self.current_at(v[0])
    }

    fn jacobian(&self, v: &[f64; 1]) -> [f64; 1] {
        [self.conductance_at(v[0])]
    }
}

/// LED model (diode with higher forward voltage).
///
/// LEDs have the same physics as regular diodes but with
/// higher bandgap (and thus higher forward voltage).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Led {
    diode: DiodeShockley,
    /// Forward voltage drop (for rough approximations) [V]
    pub vf: f64,
}

impl Led {
    /// Create an LED with the given color/voltage characteristics.
    pub fn new(is: f64, vf: f64) -> Self {
        Self {
            diode: DiodeShockley::new_room_temp(is, 1.0),
            vf,
        }
    }

    /// Red LED (~1.8V forward voltage).
    pub fn red() -> Self {
        Self::new(1e-20, 1.8)
    }

    /// Green LED (~2.2V forward voltage).
    pub fn green() -> Self {
        Self::new(1e-20, 2.2)
    }

    /// Blue/White LED (~3.3V forward voltage).
    pub fn blue() -> Self {
        Self::new(1e-25, 3.3)
    }
}

impl NonlinearDevice<1> for Led {
    fn current(&self, v: &[f64; 1]) -> f64 {
        self.diode.current(v)
    }

    fn jacobian(&self, v: &[f64; 1]) -> [f64; 1] {
        self.diode.jacobian(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_diode_forward_bias() {
        let diode = DiodeShockley::silicon();
        
        // At 0.7V, should have significant current
        let i_700mv = diode.current_at(0.7);
        assert!(i_700mv > 0.0);
        
        // At 0V, current should be approximately 0
        let i_0v = diode.current_at(0.0);
        assert!(i_0v.abs() < 1e-10);
        
        // Reverse bias: small negative current (approximately -Is)
        let i_rev = diode.current_at(-1.0);
        assert!(i_rev < 0.0);
        assert!(i_rev > -1e-11);  // Should be small (≈ -Is)
    }

    #[test]
    fn test_diode_iv_curve() {
        let diode = DiodeShockley::silicon();
        
        // Current should increase with voltage
        let i1 = diode.current_at(0.6);
        let i2 = diode.current_at(0.7);
        let i3 = diode.current_at(0.8);
        
        assert!(i2 > i1);
        assert!(i3 > i2);
        
        // Should be exponential (roughly)
        let ratio1 = i2 / i1;
        let ratio2 = i3 / i2;
        assert!((ratio1 - ratio2).abs() / ratio1 < 0.5);  // Within 50%
    }

    #[test]
    fn test_diode_conductance() {
        let diode = DiodeShockley::silicon();
        
        // Conductance should be positive and increase with voltage
        let g1 = diode.conductance_at(0.6);
        let g2 = diode.conductance_at(0.7);
        
        assert!(g1 > 0.0);
        assert!(g2 > g1);
    }

    #[test]
    fn test_diode_trait() {
        let diode = DiodeShockley::silicon();
        
        let i = diode.current(&[0.7]);
        let g = diode.jacobian(&[0.7]);
        
        assert!(i > 0.0);
        assert!(g[0] > 0.0);
    }

    #[test]
    fn test_led_forward_voltage() {
        let red = Led::red();
        let blue = Led::blue();

        // Blue LED should require higher voltage for same current
        let i_red = red.current(&[2.0]);
        let i_blue = blue.current(&[2.0]);

        assert!(i_red > i_blue, "Red LED should conduct more at 2V than blue");
    }

    /// Verify diode forward voltage at 1mA against expected values.
    ///
    /// For silicon() (Is=1e-12, n=1.5):
    ///   V = n*Vt * ln(I/Is + 1) = 1.5*0.02585*ln(1e9) ≈ 0.80V
    #[test]
    fn test_diode_forward_voltage_at_1ma() {
        let diode = DiodeShockley::silicon();

        // Find forward voltage at 1mA using bisection
        let target_i = 1e-3;
        let mut v_lo = 0.0;
        let mut v_hi = 2.0;
        for _ in 0..100 {
            let v_mid = (v_lo + v_hi) / 2.0;
            if diode.current_at(v_mid) < target_i {
                v_lo = v_mid;
            } else {
                v_hi = v_mid;
            }
        }
        let vf = (v_lo + v_hi) / 2.0;

        // Silicon() with Is=1e-12, n=1.5 should be ~0.80V at 1mA
        assert!(vf > 0.6 && vf < 1.0,
            "Forward voltage at 1mA = {:.3}V, expected 0.6-1.0V", vf);
    }

    /// Verify 1N4148 forward voltage at typical current.
    #[test]
    fn test_1n4148_forward_voltage() {
        let diode = DiodeShockley::silicon_1n4148();

        // Find Vf at 1mA
        let target_i = 1e-3;
        let mut v_lo = 0.0;
        let mut v_hi = 2.0;
        for _ in 0..100 {
            let v_mid = (v_lo + v_hi) / 2.0;
            if diode.current_at(v_mid) < target_i {
                v_lo = v_mid;
            } else {
                v_hi = v_mid;
            }
        }
        let vf = (v_lo + v_hi) / 2.0;

        // 1N4148 typical Vf at 1mA ≈ 0.55-0.65V
        assert!(vf > 0.45 && vf < 0.75,
            "1N4148 Vf at 1mA = {:.3}V, expected 0.45-0.75V", vf);
    }

    /// Verify diode conductance matches finite difference of current.
    #[test]
    fn test_diode_conductance_finite_difference() {
        let diode = DiodeShockley::silicon();
        let eps = 1e-8;

        for &v in &[0.3, 0.5, 0.6, 0.7] {
            let g = diode.conductance_at(v);
            let fd = (diode.current_at(v + eps) - diode.current_at(v - eps)) / (2.0 * eps);
            let rel_err = (g - fd).abs() / fd.abs();
            assert!(rel_err < 1e-4,
                "Conductance at {:.1}V: analytic={:.6e} fd={:.6e} err={:.2e}",
                v, g, fd, rel_err);
        }
    }

    /// Verify exponential I-V ratio is approximately constant (Shockley equation).
    #[test]
    fn test_diode_exponential_ratio() {
        let diode = DiodeShockley::silicon();

        // For an ideal exponential diode: I(V+dV)/I(V) = exp(dV/(n*Vt))
        let dv = 0.05;
        let expected_ratio = (dv / (diode.n * diode.vt)).exp();

        for &v in &[0.5, 0.6, 0.7] {
            let i1 = diode.current_at(v);
            let i2 = diode.current_at(v + dv);
            let actual_ratio = i2 / i1;
            let rel_err = (actual_ratio - expected_ratio).abs() / expected_ratio;
            assert!(rel_err < 0.02,
                "Ratio at {:.1}V: actual={:.3} expected={:.3} err={:.4}",
                v, actual_ratio, expected_ratio, rel_err);
        }
    }
}
