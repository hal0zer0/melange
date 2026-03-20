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

    /// Verify attack is faster than release by measuring time constants.
    /// After N samples at cv=1.0, attack should reach ~63% of the way to r_min.
    /// After same N samples at cv=0.0, release should reach ~63% of the way to r_max.
    #[test]
    fn test_ldr_attack_release_time_constants() {
        let sr = 44100.0;
        let mut ldr = CdsLdr::vtl5c3(sr);

        // VTL5C3: attack_tau=0.005s, release_tau=0.2s
        // After attack_tau * sr samples ≈ 220 samples, should be ~63% toward r_min
        let attack_samples = (0.005 * sr) as usize;

        ldr.reset(); // start at r_max
        let r_start = ldr.resistance();
        for _ in 0..attack_samples {
            ldr.update(1.0);
        }
        let r_after_attack = ldr.resistance();
        let attack_progress = (r_start - r_after_attack) / (r_start - ldr.r_min);

        // Should be approximately 63% (1 - 1/e) after one time constant
        assert!(
            attack_progress > 0.4 && attack_progress < 0.9,
            "Attack progress after 1 tau: {:.1}% (expected ~63%)",
            attack_progress * 100.0
        );

        // Now test release: start at r_min, go dark
        ldr.reset_to(ldr.r_min);
        let release_samples = (0.2 * sr) as usize;
        let r_start_rel = ldr.resistance();
        for _ in 0..release_samples {
            ldr.update(0.0);
        }
        let r_after_release = ldr.resistance();
        let release_progress = (r_after_release - r_start_rel) / (ldr.r_max - r_start_rel);

        assert!(
            release_progress > 0.4 && release_progress < 0.9,
            "Release progress after 1 tau: {:.1}% (expected ~63%)",
            release_progress * 100.0
        );

        // Attack tau (0.005) << release tau (0.2), so attack should be 40x faster
        // Verify by comparing progress at the SAME number of samples
        let test_samples = 500;
        ldr.reset();
        for _ in 0..test_samples {
            ldr.update(1.0);
        }
        let r_attack_500 = ldr.resistance();

        ldr.reset_to(ldr.r_min);
        for _ in 0..test_samples {
            ldr.update(0.0);
        }
        let r_release_500 = ldr.resistance();

        let attack_frac = (ldr.r_max - r_attack_500) / (ldr.r_max - ldr.r_min);
        let release_frac = (r_release_500 - ldr.r_min) / (ldr.r_max - ldr.r_min);

        assert!(
            attack_frac > release_frac * 5.0,
            "Attack should be much faster: attack={:.1}% vs release={:.1}%",
            attack_frac * 100.0,
            release_frac * 100.0
        );
    }

    /// Verify power law gamma shapes the control curve.
    /// gamma < 1 makes curve concave (fast initial, slow final)
    /// gamma = 1 is linear
    #[test]
    fn test_ldr_gamma_power_law() {
        let sr = 44100.0;

        // gamma=0.5 (concave) vs gamma=1.0 (linear)
        let mut ldr_g05 = CdsLdr::new(100.0, 1e6, 0.5, 0.001, 0.001, sr);
        let mut ldr_g10 = CdsLdr::new(100.0, 1e6, 1.0, 0.001, 0.001, sr);

        // At cv=0.5, gamma=0.5 gives (1-0.5)^0.5 = 0.707, gamma=1.0 gives 0.5
        // So target_r = r_min + (r_max - r_min) * factor
        // gamma=0.5: factor=0.707 → higher resistance (closer to dark)
        // gamma=1.0: factor=0.5 → lower resistance

        // Settle both at cv=0.5
        for _ in 0..100000 {
            ldr_g05.update(0.5);
            ldr_g10.update(0.5);
        }

        // gamma=0.5 should have higher resistance at cv=0.5
        assert!(
            ldr_g05.resistance() > ldr_g10.resistance(),
            "gamma=0.5 R={:.0} should be > gamma=1.0 R={:.0} at cv=0.5",
            ldr_g05.resistance(),
            ldr_g10.resistance()
        );

        // Both should converge to same r_min at cv=1.0
        for _ in 0..100000 {
            ldr_g05.update(1.0);
            ldr_g10.update(1.0);
        }
        let diff = (ldr_g05.resistance() - ldr_g10.resistance()).abs();
        assert!(
            diff < 10.0,
            "Both should converge to r_min: g05={:.1}, g10={:.1}",
            ldr_g05.resistance(),
            ldr_g10.resistance()
        );
    }

    /// Verify control voltage clamping and edge cases.
    #[test]
    fn test_ldr_control_voltage_edges() {
        let mut ldr = CdsLdr::vtl5c3(44100.0);

        // Negative CV should be treated as 0 (dark)
        ldr.reset();
        let r_before = ldr.resistance();
        ldr.update(-1.0);
        assert!(
            (ldr.resistance() - r_before).abs() / r_before < 0.001,
            "Negative CV should clamp to 0 (dark)"
        );

        // CV > 1 should be treated as 1 (full brightness)
        ldr.reset();
        for _ in 0..50000 {
            ldr.update(5.0);
        }
        let r_overcv = ldr.resistance();

        ldr.reset();
        for _ in 0..50000 {
            ldr.update(1.0);
        }
        let r_cv1 = ldr.resistance();

        let diff = (r_overcv - r_cv1).abs();
        assert!(
            diff < 1.0,
            "CV>1 should equal CV=1: r(5.0)={:.1}, r(1.0)={:.1}",
            r_overcv,
            r_cv1
        );
    }

    /// Verify reset_to clamps to valid range.
    #[test]
    fn test_ldr_reset_to_clamping() {
        let mut ldr = CdsLdr::vtl5c3(44100.0);

        ldr.reset_to(0.0); // below r_min
        assert_eq!(ldr.resistance(), ldr.r_min);

        ldr.reset_to(1e12); // above r_max
        assert_eq!(ldr.resistance(), ldr.r_max);

        ldr.reset_to(1000.0); // normal value
        assert_eq!(ldr.resistance(), 1000.0);
    }

    /// Verify sample rate changes time constant behavior correctly.
    #[test]
    fn test_ldr_sample_rate_independence() {
        // Same physical time (0.01s) at different sample rates should give similar result
        let tau = 0.005;

        let mut ldr_441 = CdsLdr::new(100.0, 1e6, 0.7, tau, 0.2, 44100.0);
        let mut ldr_96 = CdsLdr::new(100.0, 1e6, 0.7, tau, 0.2, 96000.0);

        let time = 0.01; // 2 time constants
        let n_441 = (time * 44100.0) as usize;
        let n_96 = (time * 96000.0) as usize;

        for _ in 0..n_441 {
            ldr_441.update(1.0);
        }
        for _ in 0..n_96 {
            ldr_96.update(1.0);
        }

        let rel_diff = (ldr_441.resistance() - ldr_96.resistance()).abs()
            / ldr_441.resistance();
        assert!(
            rel_diff < 0.01,
            "Same time at different rates should match: r@44.1k={:.1}, r@96k={:.1}, diff={:.2}%",
            ldr_441.resistance(),
            ldr_96.resistance(),
            rel_diff * 100.0
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
