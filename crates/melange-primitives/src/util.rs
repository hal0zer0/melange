//! Utility functions for audio DSP.
//!
//! - Per-instance variation (deterministic detuning)
//! - MIDI note conversion
//! - Other helpers

/// Deterministic hash function for per-instance variation.
///
/// Used to create subtle variations between circuit instances (e.g., for
/// polyphonic voices) without requiring random number generation.
///
/// Based on a simple 32-bit LCG-like hash. Not cryptographically secure,
/// but fast and deterministic.
///
/// # Arguments
/// * `seed` - Base seed value (e.g., voice index)
/// * `index` - Secondary index (e.g., component index)
///
/// # Returns
/// A value in the range [0.0, 1.0)
pub fn variation_hash(seed: u32, index: u32) -> f64 {
    // Combine seed and index
    let mut h = seed.wrapping_mul(0x9e3779b9).wrapping_add(index);

    // PCG-like output permutation (simplified)
    h = h.wrapping_mul(747796405);
    h = h ^ (h >> 16);

    // Convert to [0, 1)
    (h as f64) / (u32::MAX as f64 + 1.0)
}

/// Generate a deterministic variation value in a range.
///
/// Returns a value in [center - range, center + range]
pub fn variation_range(seed: u32, index: u32, center: f64, range: f64) -> f64 {
    center + (variation_hash(seed, index) * 2.0 - 1.0) * range
}

/// Convert MIDI note number to frequency (Hz).
///
/// Uses A4 = 440 Hz standard tuning.
///
/// # Arguments
/// * `note` - MIDI note number (69 = A4)
///
/// # Example
/// ```
/// use melange_primitives::midi_to_freq;
/// assert!((midi_to_freq(69) - 440.0).abs() < 0.01);
/// assert!((midi_to_freq(60) - 261.63).abs() < 0.01);  // Middle C
/// ```
pub fn midi_to_freq(note: u8) -> f64 {
    440.0 * 2.0_f64.powf((note as f64 - 69.0) / 12.0)
}

/// Convert frequency (Hz) to MIDI note number.
///
/// # Arguments
/// * `freq` - Frequency in Hz
///
/// # Returns
/// MIDI note number (may be fractional for non-exact pitches)
pub fn freq_to_midi(freq: f64) -> f64 {
    69.0 + 12.0 * (freq / 440.0).log2()
}

/// Map a value from one range to another.
///
/// # Arguments
/// * `x` - Input value
/// * `in_min`, `in_max` - Input range
/// * `out_min`, `out_max` - Output range
///
/// # Example
/// ```
/// use melange_primitives::map_range;
/// assert_eq!(map_range(0.5, 0.0, 1.0, 0.0, 100.0), 50.0);
/// ```
pub fn map_range(x: f64, in_min: f64, in_max: f64, out_min: f64, out_max: f64) -> f64 {
    out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min)
}

/// Convert dB to linear gain.
#[inline(always)]
pub fn db_to_gain(db: f64) -> f64 {
    10.0_f64.powf(db / 20.0)
}

/// Convert linear gain to dB.
#[inline(always)]
pub fn gain_to_db(gain: f64) -> f64 {
    20.0 * gain.log10()
}

/// Linear interpolation between two values.
#[inline(always)]
pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Clamp a value to the range [0.0, 1.0].
#[inline(always)]
pub fn saturate(x: f64) -> f64 {
    x.clamp(0.0, 1.0)
}

/// Soft clipping using tanh (symmetric).
#[inline(always)]
pub fn soft_clip(x: f64) -> f64 {
    x.tanh()
}

/// Soft clipping using tanh with adjustable hardness.
///
/// # Arguments
/// * `x` - Input value
/// * `hardness` - 0.0 = linear, 1.0 = hard clip, intermediate = soft clip
pub fn soft_clip_variable(x: f64, hardness: f64) -> f64 {
    if hardness <= 0.0 {
        x
    } else if hardness >= 1.0 {
        x.clamp(-1.0, 1.0)
    } else {
        let k = hardness / (1.0 - hardness);
        (k * x).tanh() / k.tanh()
    }
}

/// Parabolic approximation of sin for cheap oscillators.
///
/// Valid for x in [-π, π].
#[inline(always)]
pub fn sin_approx(x: f64) -> f64 {
    // Parabolic sine approximation
    // sin(x) ≈ (4/π²) * x * (π - |x|) * sign(x)
    let b = 4.0 / core::f64::consts::PI;
    let c = -4.0 / (core::f64::consts::PI * core::f64::consts::PI);

    let y = b * x + c * x * x.abs();

    // Refinement for better accuracy
    let q = 0.225;
    q * (y * y.abs() - y) + y
}

/// Cheap triangle wave from phase accumulator.
#[inline(always)]
pub fn triangle_from_phase(phase: f64) -> f64 {
    // phase in [0, 1)
    let t = phase * 2.0;
    if t < 1.0 {
        2.0 * t - 1.0
    } else {
        3.0 - 2.0 * t
    }
}

/// Check if a value is in a reasonable range for audio signals.
///
/// Useful for debug assertions.
pub fn is_valid_audio(x: f64) -> bool {
    x.is_finite() && x.abs() < 1e6
}

/// Get the thermal voltage Vt at a given temperature.
///
/// Vt = kT/q ≈ 25.85 mV at 27°C (300K)
///
/// # Arguments
/// * `temp_c` - Temperature in Celsius
pub fn thermal_voltage(temp_c: f64) -> f64 {
    const K_BOLTZMANN: f64 = 1.380649e-23;  // J/K
    const Q_ELECTRON: f64 = 1.602176634e-19; // C
    const T_ABS_ZERO: f64 = 273.15;          // K

    let temp_k = temp_c + T_ABS_ZERO;
    K_BOLTZMANN * temp_k / Q_ELECTRON
}

/// Standard thermal voltage at room temperature (27°C).
pub const VT_ROOM: f64 = 0.02585;  // ≈ 26 mV

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_variation_hash_deterministic() {
        let h1 = variation_hash(123, 0);
        let h2 = variation_hash(123, 0);
        assert_eq!(h1, h2);

        let h3 = variation_hash(123, 1);
        assert_ne!(h1, h3);
    }

    #[test]
    fn test_variation_hash_range() {
        // Check that hash values are in [0, 1)
        for i in 0..100 {
            let h = variation_hash(i, i * 7);
            assert!(h >= 0.0 && h < 1.0, "Hash out of range: {}", h);
        }
    }

    #[test]
    fn test_midi_to_freq() {
        assert!((midi_to_freq(69) - 440.0).abs() < 0.01);
        assert!((midi_to_freq(60) - 261.63).abs() < 0.5);  // Middle C
        assert!((midi_to_freq(57) - 220.0).abs() < 0.01);  // A3
    }

    #[test]
    fn test_freq_to_midi_roundtrip() {
        for note in 20..100 {
            let freq = midi_to_freq(note);
            let back = freq_to_midi(freq);
            assert!((back - note as f64).abs() < 0.01);
        }
    }

    #[test]
    fn test_db_conversion() {
        assert!((db_to_gain(0.0) - 1.0).abs() < 1e-10);
        assert!((db_to_gain(-6.0) - 0.501).abs() < 0.01);
        assert!((db_to_gain(6.0) - 1.995).abs() < 0.01);

        assert!((gain_to_db(1.0) - 0.0).abs() < 1e-10);
        assert!((gain_to_db(2.0) - 6.0206).abs() < 0.01);
    }

    #[test]
    fn test_thermal_voltage() {
        let vt_27 = thermal_voltage(27.0);
        assert!((vt_27 - 0.02585).abs() < 0.001);

        let vt_0 = thermal_voltage(0.0);
        assert!(vt_0 < vt_27);
    }

    #[test]
    fn test_soft_clip() {
        assert!(soft_clip(0.0).abs() < 1e-10);
        assert!((soft_clip(1.0) - 0.7615).abs() < 0.01);
        assert!((soft_clip(10.0) - 1.0).abs() < 0.01);
        assert!(soft_clip(-1.0) < 0.0);  // Symmetric
    }
}
