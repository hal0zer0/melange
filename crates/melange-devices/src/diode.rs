//! Diode models for circuit simulation.

use crate::{safeguards, NonlinearDevice, VT_ROOM};

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
        let c = crate::catalog::diodes::lookup("1N4148").expect("1N4148 catalog entry");
        Self::new_room_temp(c.is, c.n)
    }

    /// Create an ideal silicon diode (theoretical).
    pub fn silicon_ideal() -> Self {
        Self::new_room_temp(1e-14, 1.0)
    }

    /// Create a germanium diode (1N34A-like).
    pub fn germanium() -> Self {
        let c = crate::catalog::diodes::lookup("1N34A").expect("1N34A catalog entry");
        Self::new_room_temp(c.is, c.n)
    }

    /// Create a Schottky diode (1N5819-like).
    pub fn schottky() -> Self {
        let c = crate::catalog::diodes::lookup("1N5819").expect("1N5819 catalog entry");
        Self::new_room_temp(c.is, c.n)
    }

    /// Calculate current for a given voltage.
    ///
    /// Uses linear extension beyond the exponential clamp boundaries so that
    /// the derivative of `current_at` matches `conductance_at` everywhere.
    pub fn current_at(&self, v: f64) -> f64 {
        let v_over_nvt = v / self.n_vt;
        if v_over_nvt > safeguards::MAX_EXP_V {
            // Linear extension: I(v_clamp) + G(v_clamp) * (v - v_clamp)
            let v_clamp = safeguards::MAX_EXP_V * self.n_vt;
            let exp_clamp = safeguards::MAX_EXP_V.exp();
            let i_clamp = self.is * (exp_clamp - 1.0);
            let g_clamp = (self.is / self.n_vt) * exp_clamp;
            i_clamp + g_clamp * (v - v_clamp)
        } else if v_over_nvt < safeguards::MIN_EXP_V {
            // Linear extension for large reverse bias
            let v_clamp = safeguards::MIN_EXP_V * self.n_vt;
            let exp_clamp = safeguards::MIN_EXP_V.exp();
            let i_clamp = self.is * (exp_clamp - 1.0);
            let g_clamp = (self.is / self.n_vt) * exp_clamp;
            i_clamp + g_clamp * (v - v_clamp)
        } else {
            self.is * (v_over_nvt.exp() - 1.0)
        }
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
    /// Solves V_j + Id(V_j)*Rs = V for the junction voltage V_j using
    /// Newton-Raphson with SPICE-style voltage limiting (steps clamped to
    /// 4*n*Vt per iteration). Then I = Id(V_j).
    ///
    /// The previous single fixed-point iteration had O((g_d*Rs)^2) error —
    /// unusable for forward-biased diodes with Rs > 10 ohm.
    pub fn current_at(&self, v: f64) -> f64 {
        if self.rs <= 0.0 {
            return self.diode.current_at(v);
        }
        let v_j = self.solve_junction_voltage(v);
        self.diode.current_at(v_j)
    }

    /// Conductance with series resistance.
    ///
    /// g_total = g_d / (1 + g_d * Rs) where g_d is evaluated at the
    /// NR-solved junction voltage.
    pub fn conductance_at(&self, v: f64) -> f64 {
        if self.rs <= 0.0 {
            return self.diode.conductance_at(v);
        }
        let v_j = self.solve_junction_voltage(v);
        let g_d = self.diode.conductance_at(v_j);
        g_d / (1.0 + g_d * self.rs)
    }

    /// Solve for junction voltage: V_j + Id(V_j)*Rs = V.
    ///
    /// NR with step limiting (4*n*Vt per iteration) for robust convergence
    /// even with stiff exponentials and large Rs.
    fn solve_junction_voltage(&self, v: f64) -> f64 {
        let n_vt = self.diode.n * self.diode.vt;
        let max_step = 4.0 * n_vt;
        // Initial guess: clamp to a reasonable forward voltage
        let mut v_j = if v > 0.0 { v.min(0.7) } else { v };
        for _ in 0..15 {
            let id = self.diode.current_at(v_j);
            let gd = self.diode.conductance_at(v_j);
            // f(V_j) = V_j + Id(V_j)*Rs - V
            let f = v_j + id * self.rs - v;
            let fp = 1.0 + gd * self.rs;
            let mut delta = f / fp;
            // Clamp step to prevent overshoot in stiff exponential region
            delta = delta.clamp(-max_step, max_step);
            v_j -= delta;
            if delta.abs() < 1e-10 {
                break;
            }
        }
        v_j
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
/// LEDs have the same physics as regular diodes but with higher bandgap
/// (and thus higher forward voltage). Preset constructors (`red`, `green`,
/// `blue`) derive `Is` from the datasheet Vf at 1 mA using an LED-typical
/// emission coefficient `n`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Led {
    diode: DiodeShockley,
    /// Forward voltage drop at the design operating current [V]
    pub vf: f64,
}

impl Led {
    /// Create an LED by specifying `Is` directly. Uses a default emission
    /// coefficient `n = 2.0`, typical for red-orange LEDs.
    pub fn new(is: f64, vf: f64) -> Self {
        Self {
            diode: DiodeShockley::new_room_temp(is, 2.0),
            vf,
        }
    }

    /// Construct an LED from a datasheet operating point `(I_typ, Vf)` and
    /// emission coefficient `n`. Derives `Is = I_typ / (exp(Vf/(n·Vt)) − 1)`.
    ///
    /// `n` must be large enough that `Vf/(n·Vt) ≤ 40` (the `safe_exp` clamp),
    /// otherwise the diode's current at Vf falls into the linear-extension
    /// region and the preset is unphysical. For a given target Vf this
    /// translates to `n ≥ Vf / (40·Vt) ≈ Vf / 1.034` at room temperature.
    pub fn with_vf_at(current_amps: f64, vf_v: f64, n: f64) -> Self {
        let denom = (vf_v / (n * VT_ROOM)).exp() - 1.0;
        let is = current_amps / denom;
        Self {
            diode: DiodeShockley::new_room_temp(is, n),
            vf: vf_v,
        }
    }

    /// Red LED (~1.8 V @ 1 mA, n ≈ 2.0).
    pub fn red() -> Self {
        Self::with_vf_at(1.0e-3, 1.8, 2.0)
    }

    /// Green LED (~2.2 V @ 1 mA, n ≈ 2.5).
    pub fn green() -> Self {
        Self::with_vf_at(1.0e-3, 2.2, 2.5)
    }

    /// Blue / white LED (~3.3 V @ 1 mA, n ≈ 3.5).
    ///
    /// Blue LEDs use InGaN and have ideality factors up to 3-5 in practice.
    /// The high `n` here also keeps `Vf/(n·Vt) < 40` under the `safe_exp`
    /// clamp so the Shockley equation evaluates without linear extension.
    pub fn blue() -> Self {
        Self::with_vf_at(1.0e-3, 3.3, 3.5)
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
        assert!(i_rev > -1e-11); // Should be small (≈ -Is)
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
        assert!((ratio1 - ratio2).abs() / ratio1 < 0.5); // Within 50%
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

        assert!(
            i_red > i_blue,
            "Red LED should conduct more at 2V than blue"
        );
    }

    /// Verify LED presets produce ~1 mA at their datasheet Vf. The previous
    /// implementation hardcoded n=1.0 internally and kept `vf` as a cosmetic
    /// field, so `red()` actually conducted ~1 mA near 1.0 V instead of 1.8 V.
    #[test]
    fn test_led_presets_hit_vf_at_1ma() {
        let target = 1.0e-3; // 1 mA design point
        for (name, led, vf) in [
            ("red", Led::red(), 1.8),
            ("green", Led::green(), 2.2),
            ("blue", Led::blue(), 3.3),
        ] {
            let i_at_vf = led.current(&[vf]);
            let rel_err = (i_at_vf - target).abs() / target;
            assert!(
                rel_err < 0.2,
                "LED {} at {} V should produce ~{} A, got {} A (rel_err {:.3})",
                name,
                vf,
                target,
                i_at_vf,
                rel_err
            );
        }
    }

    /// Verify DiodeWithRs conductance is always finite, even at edge voltages
    /// where the denominator (1 + g_d * Rs) could vanish.
    #[test]
    fn test_diode_with_rs_conductance_finite() {
        // Use large Rs and various voltages to stress the denominator guard
        let diode = DiodeShockley::silicon();
        let drs = DiodeWithRs::new(diode, 100.0);

        // Test a wide range of voltages including extreme values
        for &v in &[
            -10.0, -1.0, -0.5, 0.0, 0.3, 0.5, 0.6, 0.7, 0.8, 1.0, 5.0, 10.0,
        ] {
            let g = drs.conductance_at(v);
            assert!(
                g.is_finite(),
                "DiodeWithRs conductance must be finite at v={}, got {}",
                v,
                g
            );
            // Conductance should be non-negative for a diode
            assert!(
                g >= 0.0,
                "DiodeWithRs conductance must be non-negative at v={}, got {}",
                v,
                g
            );
        }

        // Also test with very small Rs
        let drs_small = DiodeWithRs::new(diode, 1e-10);
        for &v in &[0.0, 0.7, 1.0] {
            let g = drs_small.conductance_at(v);
            assert!(
                g.is_finite(),
                "Small Rs conductance must be finite at v={}",
                v
            );
        }
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
        assert!(
            vf > 0.6 && vf < 1.0,
            "Forward voltage at 1mA = {:.3}V, expected 0.6-1.0V",
            vf
        );
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
        assert!(
            vf > 0.45 && vf < 0.75,
            "1N4148 Vf at 1mA = {:.3}V, expected 0.45-0.75V",
            vf
        );
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
            assert!(
                rel_err < 1e-4,
                "Conductance at {:.1}V: analytic={:.6e} fd={:.6e} err={:.2e}",
                v,
                g,
                fd,
                rel_err
            );
        }
    }

    /// Verify current_at derivative matches conductance_at at and beyond clamp boundaries.
    #[test]
    fn test_diode_linearized_continuation() {
        let diode = DiodeShockley::silicon();
        let n_vt = diode.n * diode.vt;
        let eps = 1e-8;

        // Test at clamp boundary and well beyond it
        let v_clamp = safeguards::MAX_EXP_V * n_vt;
        for &v in &[v_clamp - 0.01, v_clamp, v_clamp + 0.1, v_clamp + 1.0] {
            let g = diode.conductance_at(v);
            let fd = (diode.current_at(v + eps) - diode.current_at(v - eps)) / (2.0 * eps);
            let err = (g - fd).abs();
            assert!(
                err < g.abs() * 1e-4 + 1e-10,
                "Derivative mismatch at v={:.4}: conductance={:.6e} fd={:.6e} err={:.2e}",
                v,
                g,
                fd,
                err
            );
        }

        // Negative clamp boundary
        let v_neg = safeguards::MIN_EXP_V * n_vt;
        for &v in &[v_neg + 0.01, v_neg, v_neg - 0.1] {
            let g = diode.conductance_at(v);
            let fd = (diode.current_at(v + eps) - diode.current_at(v - eps)) / (2.0 * eps);
            let err = (g - fd).abs();
            assert!(
                err < g.abs() * 1e-4 + 1e-10,
                "Derivative mismatch at v={:.4}: conductance={:.6e} fd={:.6e} err={:.2e}",
                v,
                g,
                fd,
                err
            );
        }
    }

    /// Verify DiodeWithRs NR solution: I = Is*(exp((V-I*Rs)/(n*Vt))-1).
    /// Check that the solved current satisfies the implicit equation.
    #[test]
    fn test_diode_with_rs_nr_accuracy() {
        let diode = DiodeShockley::silicon();

        // Test with large Rs where single fixed-point fails
        for &rs in &[10.0, 100.0, 1000.0] {
            let drs = DiodeWithRs::new(diode, rs);
            for &v in &[0.7, 1.0, 2.0, 5.0] {
                let i = drs.current_at(v);
                // Verify implicit equation: I should equal Id(V - I*Rs)
                let v_j = v - i * rs;
                let id = diode.current_at(v_j);
                let residual = (i - id).abs();
                assert!(
                    residual < 1e-10,
                    "NR residual at v={}, Rs={}: I={:.6e}, Id(Vj)={:.6e}, |I-Id|={:.2e}",
                    v,
                    rs,
                    i,
                    id,
                    residual
                );
            }
        }
    }

    /// Verify DiodeWithRs conductance matches finite difference of current.
    #[test]
    fn test_diode_with_rs_conductance_fd() {
        let diode = DiodeShockley::silicon();
        let eps = 1e-7;

        for &rs in &[10.0, 100.0] {
            let drs = DiodeWithRs::new(diode, rs);
            for &v in &[0.7, 1.0, 2.0] {
                let g = drs.conductance_at(v);
                let fd = (drs.current_at(v + eps) - drs.current_at(v - eps)) / (2.0 * eps);
                let rel_err = if fd.abs() > 1e-15 {
                    (g - fd).abs() / fd.abs()
                } else {
                    g.abs()
                };
                assert!(
                    rel_err < 1e-3,
                    "DiodeWithRs g mismatch at v={}, Rs={}: g={:.6e} fd={:.6e} err={:.2e}",
                    v,
                    rs,
                    g,
                    fd,
                    rel_err
                );
            }
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
            assert!(
                rel_err < 0.02,
                "Ratio at {:.1}V: actual={:.3} expected={:.3} err={:.4}",
                v,
                actual_ratio,
                expected_ratio,
                rel_err
            );
        }
    }
}
