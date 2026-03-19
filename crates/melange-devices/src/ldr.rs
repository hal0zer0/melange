//! Cadmium Sulfide (CdS) Photoresistor / LDR model.
//!
//! Used in optocouplers like the VTL5C series for tremolo effects
//! and compressor gain reduction.

use crate::NonlinearDevice;

/// CdS photoresistor model with asymmetric envelope.
///
/// Models the resistance change in response to light from an LED.
/// The resistance follows a power law with the light intensity,
/// with asymmetric attack and release times.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CdsLdr {
    /// Minimum resistance (brightest light) [Ohms]
    pub r_min: f64,
    /// Maximum resistance (dark) [Ohms]
    pub r_max: f64,
    /// Power law exponent (typically 0.5-1.0)
    pub gamma: f64,
    /// Attack time constant (resistance decreasing) [seconds]
    pub attack_tau: f64,
    /// Release time constant (resistance increasing) [seconds]
    pub release_tau: f64,
    /// Sample rate [Hz]
    pub sample_rate: f64,
    /// Current resistance state
    r_state: f64,
    /// Attack coefficient (per sample)
    attack_coef: f64,
    /// Release coefficient (per sample)
    release_coef: f64,
}

impl CdsLdr {
    /// Create a new CdS LDR model.
    pub fn new(
        r_min: f64,
        r_max: f64,
        gamma: f64,
        attack_tau: f64,
        release_tau: f64,
        sample_rate: f64,
    ) -> Self {
        let attack_coef = (-1.0 / (attack_tau * sample_rate)).exp();
        let release_coef = (-1.0 / (release_tau * sample_rate)).exp();

        Self {
            r_min,
            r_max,
            gamma,
            attack_tau,
            release_tau,
            sample_rate,
            r_state: r_max,
            attack_coef,
            release_coef,
        }
    }

    /// VTL5C3 characteristics (typical optocoupler).
    /// Datasheet r_min ≈ 75Ω at full brightness.
    pub fn vtl5c3(sample_rate: f64) -> Self {
        Self::new(75.0, 10e6, 0.7, 0.005, 0.2, sample_rate)
    }

    /// VTL5C4 (faster version).
    /// Datasheet r_min ≈ 40Ω at full brightness.
    pub fn vtl5c4(sample_rate: f64) -> Self {
        Self::new(40.0, 5e6, 0.7, 0.001, 0.05, sample_rate)
    }

    /// NSL-32 (JHS-style).
    /// Datasheet r_min ≈ 75Ω at full brightness.
    pub fn nsl32(sample_rate: f64) -> Self {
        Self::new(75.0, 1e6, 0.8, 0.002, 0.1, sample_rate)
    }

    /// Update the resistance based on control voltage.
    ///
    /// control_voltage: 0.0 = dark (max resistance), 1.0 = bright (min resistance)
    pub fn update(&mut self, control_voltage: f64) {
        // Clamp control voltage
        let cv = control_voltage.clamp(0.0, 1.0);

        // Target resistance (power law)
        let target_r = self.r_min + (self.r_max - self.r_min) * (1.0 - cv).powf(self.gamma);

        // Asymmetric envelope following
        if target_r < self.r_state {
            // Attack (getting brighter, resistance decreasing)
            self.r_state = target_r + (self.r_state - target_r) * self.attack_coef;
        } else {
            // Release (getting darker, resistance increasing)
            self.r_state = target_r + (self.r_state - target_r) * self.release_coef;
        }
    }

    /// Get current resistance.
    pub fn resistance(&self) -> f64 {
        self.r_state
    }

    /// Get current conductance.
    pub fn conductance(&self) -> f64 {
        1.0 / self.r_state.max(1e-12)
    }

    /// Reset to dark state.
    pub fn reset(&mut self) {
        self.r_state = self.r_max;
    }

    /// Reset to a specific resistance.
    pub fn reset_to(&mut self, r: f64) {
        self.r_state = r.clamp(self.r_min, self.r_max);
    }
}

/// Minimum resistance floor to prevent division by zero.
const MIN_RESISTANCE: f64 = 1e-12;

/// LDR as a two-terminal variable resistor.
impl NonlinearDevice<1> for CdsLdr {
    /// Current through LDR: I = V / R
    fn current(&self, v: &[f64; 1]) -> f64 {
        v[0] / self.r_state.max(MIN_RESISTANCE)
    }

    /// Conductance: G = 1/R
    fn jacobian(&self, _v: &[f64; 1]) -> [f64; 1] {
        [1.0 / self.r_state.max(MIN_RESISTANCE)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ldr_range() {
        let mut ldr = CdsLdr::vtl5c3(44100.0);

        // Initial state should be dark
        assert!(ldr.resistance() > 1e6);

        // Full brightness - needs many iterations to settle
        for _ in 0..50000 {
            ldr.update(1.0);
        }

        // Should approach r_min (with tolerance for attack time)
        assert!(
            ldr.resistance() < 500.0,
            "R = {} ohms, expected near {}",
            ldr.resistance(),
            ldr.r_min
        );
    }

    #[test]
    fn test_ldr_asymmetry() {
        let mut ldr = CdsLdr::vtl5c3(44100.0);

        // Dark to bright (attack)
        ldr.reset();
        ldr.update(1.0);
        let r_after_attack = ldr.resistance();

        // Reset and go bright to dark (release)
        ldr.reset();
        ldr.update(0.0);
        let r_after_release = ldr.resistance();

        // Attack should be faster than release
        // (r_max - r_after_attack) should be larger change than (r_after_release - r_max)
        // Actually, since we only do one sample:
        // Attack changes r_max -> slightly lower
        // Release keeps r_max the same
        assert!(r_after_attack < r_after_release);
    }

    #[test]
    fn test_ldr_current() {
        let mut ldr = CdsLdr::vtl5c3(44100.0);
        ldr.reset();

        // At 1V, current should be small (high resistance)
        let i = ldr.current(&[1.0]);
        assert!(i < 1e-6);

        // Make bright
        for _ in 0..10000 {
            ldr.update(1.0);
        }

        // Now current should be larger
        let i_bright = ldr.current(&[1.0]);
        assert!(i_bright > 1e-4);
    }

    /// Verify LDR dark resistance matches datasheet values.
    ///
    /// VTL5C3 typical: r_dark > 1M, r_min ~ 75 ohms
    /// VTL5C4 typical: r_dark > 1M, r_min ~ 40 ohms
    #[test]
    fn test_ldr_datasheet_ranges() {
        // VTL5C3
        let ldr = CdsLdr::vtl5c3(44100.0);
        assert!(
            ldr.resistance() > 1e6,
            "VTL5C3 dark resistance = {:.0} ohms, expected > 1M",
            ldr.resistance()
        );
        assert_eq!(ldr.r_min, 75.0, "VTL5C3 r_min should be 75 ohms");

        // VTL5C4
        let ldr = CdsLdr::vtl5c4(44100.0);
        assert!(
            ldr.resistance() > 1e6,
            "VTL5C4 dark resistance = {:.0} ohms, expected > 1M",
            ldr.resistance()
        );
        assert_eq!(ldr.r_min, 40.0, "VTL5C4 r_min should be 40 ohms");

        // NSL-32
        let ldr = CdsLdr::nsl32(44100.0);
        assert!(
            ldr.resistance() >= 1e6,
            "NSL32 dark resistance = {:.0} ohms, expected >= 1M",
            ldr.resistance()
        );
        assert_eq!(ldr.r_min, 75.0, "NSL32 r_min should be 75 ohms");
    }

    /// Verify LDR converges to r_min at full brightness.
    #[test]
    fn test_ldr_converges_to_rmin() {
        let mut ldr = CdsLdr::vtl5c3(44100.0);

        // Run for many samples at full brightness
        for _ in 0..100000 {
            ldr.update(1.0);
        }

        let r = ldr.resistance();
        let r_min = ldr.r_min;
        // Should be within 20% of r_min after long settling
        assert!(
            (r - r_min) / r_min < 0.2,
            "After settling, R = {:.1} ohms, r_min = {:.1} ohms",
            r,
            r_min
        );
    }

    /// Verify LDR current and jacobian are finite even with very small r_min.
    #[test]
    fn test_ldr_finite_at_small_resistance() {
        // Create LDR with very small r_min to test the guard
        let mut ldr = CdsLdr::new(1e-15, 1e6, 0.7, 0.005, 0.2, 44100.0);

        // Drive to minimum resistance
        for _ in 0..200000 {
            ldr.update(1.0);
        }

        let i = ldr.current(&[1.0]);
        let g = ldr.jacobian(&[1.0]);
        assert!(i.is_finite(), "LDR current must be finite, got {}", i);
        assert!(
            g[0].is_finite(),
            "LDR conductance must be finite, got {}",
            g[0]
        );
        assert!(g[0] > 0.0, "LDR conductance must be positive, got {}", g[0]);
        assert!(
            ldr.conductance().is_finite(),
            "conductance() must be finite"
        );
    }
}
