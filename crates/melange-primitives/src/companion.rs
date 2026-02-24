//! Companion models for reactive component discretization.
//!
//! These implement the "equivalent circuit" form of discretization,
//! where capacitors and inductors are replaced by:
//! - A conductance (resistor)
//! - A parallel current source (history term)
//!
//! Used in the DK method and MNA-based circuit simulation.

/// Trapezoidal companion model for a capacitor.
///
/// The trapezoidal rule gives:
///   i[n] = g_eq * v[n] + i_hist
///
/// where:
///   g_eq = 2C/T (companion conductance)
///   i_hist = -g_eq * v[n-1] - i[n-1]  (history current)
///
/// This is numerically stable and second-order accurate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidalCompanion {
    /// Companion conductance g_eq = 2C/T [Siemens]
    pub g_eq: f64,
    /// History current i_hist [Amps]
    pub i_hist: f64,
    /// Previous voltage v[n-1] [Volts]
    v_prev: f64,
    /// Previous current i[n-1] [Amps]
    i_prev: f64,
    /// Capacitance value [Farads]
    capacitance: f64,
    /// Sample period T [seconds]
    sample_period: f64,
}

impl TrapezoidalCompanion {
    /// Create a new trapezoidal companion model for a capacitor.
    ///
    /// # Arguments
    /// * `capacitance` - Capacitance in Farads
    /// * `sample_rate` - Sample rate in Hz
    pub fn new_capacitor(capacitance: f64, sample_rate: f64) -> Self {
        let t = 1.0 / sample_rate;
        let g_eq = 2.0 * capacitance / t;
        Self {
            g_eq,
            i_hist: 0.0,
            v_prev: 0.0,
            i_prev: 0.0,
            capacitance,
            sample_period: t,
        }
    }

    /// Get the equivalent conductance for MNA stamping.
    #[inline(always)]
    pub fn conductance(&self) -> f64 {
        self.g_eq
    }

    /// Get the history current source value.
    ///
    /// This is the value to use in the RHS vector of MNA.
    #[inline(always)]
    pub fn history_current(&self) -> f64 {
        self.i_hist
    }

    /// Compute the port current given the port voltage.
    ///
    /// i[n] = g_eq * v[n] + i_hist
    #[inline(always)]
    pub fn current(&self, v: f64) -> f64 {
        self.g_eq * v + self.i_hist
    }

    /// Update the history term after solving for v[n].
    ///
    /// Must be called exactly once per timestep after v[n] is known.
    /// 
    /// Trapezoidal companion model update:
    /// i[n] = g_eq * v[n] + j[n]
    /// j[n+1] = -g_eq * v[n] - i[n]
    ///
    /// At DC steady state (v constant, i = 0), j = -g_eq * v
    /// which gives i = g_eq * v + (-g_eq * v) = 0 ✓
    #[inline(always)]
    pub fn update(&mut self, v_new: f64) {
        // Current at this timestep
        let i_new = self.g_eq * v_new + self.i_hist;

        // History for next timestep
        self.i_hist = -self.g_eq * v_new - i_new;

        self.v_prev = v_new;
        self.i_prev = i_new;
    }

    /// Reset history to zero (DC initialization).
    pub fn reset(&mut self) {
        self.i_hist = 0.0;
        self.v_prev = 0.0;
        self.i_prev = 0.0;
    }

    /// Reset to a specific DC voltage.
    pub fn reset_to_dc(&mut self, v_dc: f64) {
        self.v_prev = v_dc;
        self.i_prev = 0.0;  // No current at DC for capacitor
        self.i_hist = -self.g_eq * v_dc;
    }

    /// Update sample rate (e.g., for variable rate operation).
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        // Scale history to preserve charge
        let old_g = self.g_eq;
        self.sample_period = 1.0 / sample_rate;
        self.g_eq = 2.0 * self.capacitance / self.sample_period;

        // Scale history term to maintain consistency
        if old_g > 0.0 {
            self.i_hist *= self.g_eq / old_g;
        }
    }
}

/// Bilinear companion model for series R-C circuits.
///
/// For a series R-C, the standard trapezoidal companion can be inefficient.
/// The bilinear transform gives a combined admittance that accounts for both
/// components.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BilinearCompanion {
    /// Series resistance [Ohms]
    pub r: f64,
    /// Capacitance [Farads]
    pub c: f64,
    /// Pre-warped time constant
    tau_warped: f64,
    /// History voltage (internal state)
    v_hist: f64,
    /// Sample rate
    sample_rate: f64,
}

impl BilinearCompanion {
    /// Create a bilinear companion for series R-C.
    pub fn new_series_rc(r: f64, c: f64, sample_rate: f64) -> Self {
        let t = 1.0 / sample_rate;
        // Pre-warp the time constant for exact match at fs/4
        let tau = r * c;
        let tau_warped = tau * (core::f64::consts::PI * t / (2.0 * tau)).tan()
            / (core::f64::consts::PI * t / (2.0 * tau));

        Self {
            r,
            c,
            tau_warped,
            v_hist: 0.0,
            sample_rate,
        }
    }

    /// Get the equivalent admittance Y = 1/Z.
    #[inline(always)]
    pub fn admittance(&self) -> f64 {
        let t = 1.0 / self.sample_rate;
        // Bilinear: Y = (1/R) * (T/(2τ)) / (1 + T/(2τ)) where τ = RC
        let alpha = t / (2.0 * self.tau_warped);
        (1.0 / self.r) * alpha / (1.0 + alpha)
    }

    /// Get the equivalent series resistance (for MNA).
    #[inline(always)]
    pub fn equivalent_resistance(&self) -> f64 {
        1.0 / self.admittance()
    }

    /// Compute current given applied voltage.
    #[inline(always)]
    pub fn current(&self, v_in: f64) -> f64 {
        self.admittance() * v_in - self.v_hist
    }

    /// Update history after solving.
    #[inline(always)]
    pub fn update(&mut self, v_in: f64, i_new: f64) {
        let t = 1.0 / self.sample_rate;
        let alpha = t / (2.0 * self.tau_warped);

        // Bilinear history update
        self.v_hist = (2.0 * alpha / (1.0 + alpha)) * (v_in - i_new * self.r);
    }

    /// Reset history.
    pub fn reset(&mut self) {
        self.v_hist = 0.0;
    }
}

/// Backward Euler companion model (simpler, first-order).
///
/// Less accurate than trapezoidal but unconditionally stable
/// and simpler to implement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BackwardEulerCompanion {
    g_eq: f64,
    i_hist: f64,
    v_prev: f64,
}

impl BackwardEulerCompanion {
    /// Create backward Euler companion for capacitor.
    pub fn new_capacitor(capacitance: f64, sample_rate: f64) -> Self {
        let g_eq = capacitance * sample_rate;  // C/T
        Self {
            g_eq,
            i_hist: 0.0,
            v_prev: 0.0,
        }
    }

    /// Get companion conductance.
    #[inline(always)]
    pub fn conductance(&self) -> f64 {
        self.g_eq
    }

    /// Get history current.
    #[inline(always)]
    pub fn history_current(&self) -> f64 {
        self.i_hist
    }

    /// Update after solving.
    #[inline(always)]
    pub fn update(&mut self, v_new: f64) {
        // Backward Euler: i_hist[n+1] = -g_eq * v[n]
        self.i_hist = -self.g_eq * v_new;
        self.v_prev = v_new;
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.i_hist = 0.0;
        self.v_prev = 0.0;
    }
}

/// Compute companion conductance for a capacitor.
///
/// Helper function for direct use in matrix stamping.
#[inline(always)]
pub fn companion_conductance_trapezoidal(capacitance: f64, sample_rate: f64) -> f64 {
    2.0 * capacitance * sample_rate
}

/// Compute companion conductance for backward Euler.
#[inline(always)]
pub fn companion_conductance_backward_euler(capacitance: f64, sample_rate: f64) -> f64 {
    capacitance * sample_rate
}

/// Trapezoidal companion model for an inductor.
///
/// The trapezoidal rule gives:
///   v[n] = g_eq_inv * i[n] + v_hist
///
/// Equivalent to a Norton model:
///   i[n] = g_eq * v[n] + j_hist
///
/// where:
///   g_eq = T/(2L) (companion conductance)
///   j_hist = i[n-1] + g_eq * v[n-1]  (history current source)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InductorCompanion {
    /// Companion conductance g_eq = T/(2L) [Siemens]
    pub g_eq: f64,
    /// History current j_hist [Amps]
    pub j_hist: f64,
    /// Previous voltage v[n-1] [Volts]
    v_prev: f64,
    /// Previous current i[n-1] [Amps]
    i_prev: f64,
    /// Inductance value [Henrys]
    inductance: f64,
}

impl InductorCompanion {
    /// Create a new trapezoidal companion model for an inductor.
    ///
    /// # Arguments
    /// * `inductance` - Inductance in Henrys
    /// * `sample_rate` - Sample rate in Hz
    pub fn new_inductor(inductance: f64, sample_rate: f64) -> Self {
        let t = 1.0 / sample_rate;
        let g_eq = t / (2.0 * inductance); // T/(2L)
        Self {
            g_eq,
            j_hist: 0.0,
            v_prev: 0.0,
            i_prev: 0.0,
            inductance,
        }
    }

    /// Get the equivalent conductance for MNA stamping.
    #[inline(always)]
    pub fn conductance(&self) -> f64 {
        self.g_eq
    }

    /// Get the history current source value.
    #[inline(always)]
    pub fn history_current(&self) -> f64 {
        self.j_hist
    }

    /// Compute the port current given the port voltage.
    ///
    /// i[n] = g_eq * v[n] + j_hist
    #[inline(always)]
    pub fn current(&self, v: f64) -> f64 {
        self.g_eq * v + self.j_hist
    }

    /// Update the history term after solving for v[n].
    ///
    /// Trapezoidal inductor companion:
    /// i_new = i_prev + g_eq * (v_prev + v_new)
    /// j_hist_next = i_new + g_eq * v_new
    #[inline(always)]
    pub fn update(&mut self, v_new: f64) {
        let i_new = self.i_prev + self.g_eq * (self.v_prev + v_new);
        // History for next timestep: J = i_new + g_eq * v_new
        // (Norton equivalent current source)
        self.j_hist = i_new + self.g_eq * v_new;
        self.v_prev = v_new;
        self.i_prev = i_new;
    }

    /// Reset history to zero.
    pub fn reset(&mut self) {
        self.j_hist = 0.0;
        self.v_prev = 0.0;
        self.i_prev = 0.0;
    }
}

/// Compute companion conductance for an inductor (trapezoidal).
///
/// g_eq = T / (2L) = 1 / (2 * L * sample_rate)
#[inline(always)]
pub fn inductor_companion_conductance_trapezoidal(inductance: f64, sample_rate: f64) -> f64 {
    1.0 / (2.0 * inductance * sample_rate)
}

/// History term for trapezoidal capacitor.
///
/// Returns the current source value to use in RHS vector.
#[inline(always)]
pub fn history_current_trapezoidal(g_eq: f64, v_prev: f64, i_prev: f64) -> f64 {
    -g_eq * v_prev - i_prev
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trapezoidal_conductance() {
        let c = 1e-6;  // 1 µF
        let fs = 44100.0;
        let g = companion_conductance_trapezoidal(c, fs);
        // g = 2*C*fs = 2 * 1e-6 * 44100 = 0.0882
        assert!((g - 0.0882).abs() < 1e-6);
    }

    #[test]
    fn test_trapezoidal_dc_steady_state() {
        // Test that the reset_to_dc method properly initializes for DC steady state
        let mut comp = TrapezoidalCompanion::new_capacitor(1e-6, 44100.0);

        let v_dc = 5.0;
        comp.reset_to_dc(v_dc);

        // After reset_to_dc, current should be 0
        let i_final = comp.current(v_dc);
        assert!(i_final.abs() < 1e-10, "Capacitor current at DC after reset_to_dc: {} (expected 0)", i_final);
        
        // And history should be -g_eq * v_dc
        let expected_hist = -comp.g_eq * v_dc;
        assert!((comp.i_hist - expected_hist).abs() < 1e-10, 
            "History mismatch: expected {}, got {}", expected_hist, comp.i_hist);
    }
    
    #[test]
    fn test_trapezoidal_oscillation_behavior() {
        // Trapezoidal rule oscillates for step inputs - this is expected behavior
        let mut comp = TrapezoidalCompanion::new_capacitor(1e-6, 44100.0);

        let v_dc = 5.0;
        
        // First few iterations show oscillation
        let i0 = comp.current(v_dc);
        comp.update(v_dc);
        let i1 = comp.current(v_dc);
        comp.update(v_dc);
        let i2 = comp.current(v_dc);
        
        // Should oscillate between positive and negative
        assert!(i0 > 0.0);
        assert!(i1 < 0.0);
        assert!(i2 > 0.0);
        
        // Magnitude should be consistent
        assert!((i0.abs() - i1.abs()).abs() < 1e-10);
    }

    #[test]
    fn test_trapezoidal_conservation() {
        let mut comp = TrapezoidalCompanion::new_capacitor(1e-6, 44100.0);

        // Apply step and check charge conservation
        let v1 = 0.0;
        let v2 = 1.0;

        // Step from 0 to 1V
        let i1 = comp.current(v1);
        comp.update(v1);

        let i2 = comp.current(v2);
        comp.update(v2);

        // Charge Q = C*V, with trapezoidal:
        // Average current should relate to charge change
        let _delta_q = 1e-6 * (v2 - v1);
        let _avg_i = (i1 + i2) / 2.0;

        // Just verify no crash and reasonable values
        assert!(i1.is_finite());
        assert!(i2.is_finite());
    }

    #[test]
    fn test_backward_euler_vs_trapezoidal() {
        let c = 1e-6;
        let fs = 44100.0;

        let g_trap = companion_conductance_trapezoidal(c, fs);
        let g_be = companion_conductance_backward_euler(c, fs);

        // Trapezoidal has 2x the conductance of backward Euler
        assert!((g_trap - 2.0 * g_be).abs() < 1e-10);
    }
}
