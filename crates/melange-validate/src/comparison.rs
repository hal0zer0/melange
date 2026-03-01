//! Multi-metric comparison engine for SPICE vs melange validation
//!
//! This module provides comprehensive signal comparison with multiple error metrics
//! including time-domain errors, correlation, and spectral analysis (THD).

use rustfft::{num_complex::Complex, FftPlanner};
use std::f64::consts::PI;

/// A signal with associated sample rate for comparison
#[derive(Debug, Clone)]
pub struct Signal {
    /// Sample values
    pub samples: Vec<f64>,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Signal name/identifier
    pub name: String,
}

impl Signal {
    /// Create a new signal from samples
    pub fn new(samples: Vec<f64>, sample_rate: f64, name: impl Into<String>) -> Self {
        Self {
            samples,
            sample_rate,
            name: name.into(),
        }
    }

    /// Get the duration of the signal in seconds
    pub fn duration(&self) -> f64 {
        self.samples.len() as f64 / self.sample_rate
    }

    /// Get the number of samples
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Check if the signal is empty
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Compute the RMS value of the signal
    pub fn rms(&self) -> f64 {
        if self.samples.is_empty() {
            return 0.0;
        }
        let sum_sq: f64 = self.samples.iter().map(|&s| s * s).sum();
        (sum_sq / self.samples.len() as f64).sqrt()
    }

    /// Find the peak (maximum absolute) value
    pub fn peak(&self) -> f64 {
        self.samples
            .iter()
            .map(|&s| s.abs())
            .fold(0.0, f64::max)
    }

    /// Compute the mean value
    pub fn mean(&self) -> f64 {
        if self.samples.is_empty() {
            return 0.0;
        }
        self.samples.iter().sum::<f64>() / self.samples.len() as f64
    }

    /// Resample signal to a target sample rate using linear interpolation
    pub fn resample(&self, target_rate: f64) -> Signal {
        if (self.sample_rate - target_rate).abs() < f64::EPSILON {
            return self.clone();
        }

        let duration = self.duration();
        let new_len = (duration * target_rate).ceil() as usize;
        let mut new_samples = Vec::with_capacity(new_len);

        let ratio = self.sample_rate / target_rate;

        for i in 0..new_len {
            let src_idx = i as f64 * ratio;
            let idx_low = src_idx.floor() as usize;
            let idx_high = (idx_low + 1).min(self.samples.len() - 1);
            let frac = src_idx - idx_low as f64;

            let val_low = self.samples.get(idx_low).copied().unwrap_or(0.0);
            let val_high = self.samples.get(idx_high).copied().unwrap_or(val_low);

            let interpolated = val_low + frac * (val_high - val_low);
            new_samples.push(interpolated);
        }

        Signal::new(new_samples, target_rate, format!("{}_resampled", self.name))
    }

    /// Remove DC offset
    pub fn remove_dc(&self) -> Signal {
        let mean = self.mean();
        Signal::new(
            self.samples.iter().map(|&s| s - mean).collect(),
            self.sample_rate,
            format!("{}_ac", self.name),
        )
    }

    /// Apply a window function (Hann window) for spectral analysis
    pub fn apply_window(&self) -> Signal {
        let n = self.samples.len();
        if n <= 1 {
            // Single sample or empty: Hann window is trivially 0 at boundaries
            return Signal::new(vec![0.0; n], self.sample_rate, format!("{}_windowed", self.name));
        }
        let windowed: Vec<f64> = self
            .samples
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let hann = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos());
                s * hann
            })
            .collect();

        Signal::new(windowed, self.sample_rate, format!("{}_windowed", self.name))
    }
}

/// Configuration for signal comparison tolerances
#[derive(Debug, Clone, Copy)]
pub struct ComparisonConfig {
    /// RMS error tolerance (e.g., 0.001 = 0.1% of full scale)
    pub rms_error_tolerance: f64,
    /// Peak error tolerance (e.g., 0.01 = 10mV for 1V signal)
    pub peak_error_tolerance: f64,
    /// Maximum relative error tolerance (e.g., 0.01 = 1%)
    pub max_relative_tolerance: f64,
    /// Minimum correlation coefficient (e.g., 0.9999)
    pub correlation_min: f64,
    /// THD error tolerance in dB (e.g., 1.0 dB)
    pub thd_error_tolerance_db: f64,
    /// Full scale reference for relative error calculations
    pub full_scale: f64,
    /// Skip THD computation (faster for large signals)
    pub skip_thd: bool,
}

impl Default for ComparisonConfig {
    fn default() -> Self {
        Self {
            rms_error_tolerance: 0.001,    // 0.1%
            peak_error_tolerance: 0.01,    // 10mV
            max_relative_tolerance: 0.01,  // 1%
            correlation_min: 0.9999,
            thd_error_tolerance_db: 1.0,   // 1 dB
            full_scale: 1.0,               // Assume 1V full scale by default
            skip_thd: false,
        }
    }
}

impl ComparisonConfig {
    /// Create a strict configuration for high-precision validation
    pub fn strict() -> Self {
        Self {
            rms_error_tolerance: 0.0001,   // 0.01%
            peak_error_tolerance: 0.001,   // 1mV
            max_relative_tolerance: 0.001, // 0.1%
            correlation_min: 0.99999,
            thd_error_tolerance_db: 0.1,   // 0.1 dB
            full_scale: 1.0,
            skip_thd: false,
        }
    }

    /// Create a relaxed configuration for approximate validation
    pub fn relaxed() -> Self {
        Self {
            rms_error_tolerance: 0.01,     // 1%
            peak_error_tolerance: 0.1,     // 100mV
            max_relative_tolerance: 0.05,  // 5%
            correlation_min: 0.999,
            thd_error_tolerance_db: 3.0,   // 3 dB
            full_scale: 1.0,
            skip_thd: false,
        }
    }
}

/// Comprehensive comparison report between two signals
#[derive(Debug, Clone)]
pub struct ComparisonReport {
    /// Name of the circuit being validated
    pub circuit_name: String,
    /// Name of the output node being compared
    pub node_name: String,
    /// Number of samples compared
    pub sample_count: usize,
    /// Sample rate of the comparison
    pub sample_rate: f64,

    // Time-domain metrics
    /// RMS error between signals
    pub rms_error: f64,
    /// Peak (maximum) absolute error
    pub peak_error: f64,
    /// Maximum relative error
    pub max_relative_error: f64,
    /// Mean absolute error
    pub mean_absolute_error: f64,
    /// Normalized RMS error (relative to reference RMS)
    pub normalized_rms_error: f64,

    // Correlation
    /// Pearson correlation coefficient
    pub correlation_coefficient: f64,
    /// Signal-to-noise ratio in dB
    pub snr_db: f64,

    // Spectral metrics
    /// THD of the SPICE/reference signal in dB
    pub thd_spice: f64,
    /// THD of the melange signal in dB
    pub thd_melange: f64,
    /// Absolute THD difference in dB
    pub thd_error_db: f64,

    // Pass/fail
    /// Whether all tolerances were met
    pub passed: bool,
    /// List of specific failures
    pub failures: Vec<String>,

    // Raw error signal (for debugging/plotting)
    /// Per-sample absolute errors (only stored if configured)
    pub absolute_errors: Option<Vec<f64>>,
    /// Per-sample relative errors
    pub relative_errors: Option<Vec<f64>>,
}

impl ComparisonReport {
    /// Generate a summary string of the comparison results
    pub fn summary(&self) -> String {
        let mut summary = format!(
            "Validation Report for {} (node: {})\n",
            self.circuit_name, self.node_name
        );
        summary.push_str(&format!("Samples: {} at {:.0} Hz\n", self.sample_count, self.sample_rate));
        summary.push_str(&format!("Status: {}\n\n", if self.passed { "PASSED ✓" } else { "FAILED ✗" }));

        summary.push_str("Time-Domain Metrics:\n");
        summary.push_str(&format!("  RMS Error:        {:.6e} ({:.4}%)\n", 
            self.rms_error, self.normalized_rms_error * 100.0));
        summary.push_str(&format!("  Peak Error:       {:.6e}\n", self.peak_error));
        summary.push_str(&format!("  Mean Abs Error:   {:.6e}\n", self.mean_absolute_error));
        summary.push_str(&format!("  Max Rel Error:    {:.6e} ({:.4}%)\n", 
            self.max_relative_error, self.max_relative_error * 100.0));

        summary.push('\n');
        summary.push_str("Correlation:\n");
        summary.push_str(&format!("  Correlation:      {:.8}\n", self.correlation_coefficient));
        summary.push_str(&format!("  SNR:              {:.2} dB\n", self.snr_db));

        if self.thd_spice.is_finite() && self.thd_melange.is_finite() {
            summary.push('\n');
            summary.push_str("Spectral Metrics:\n");
            summary.push_str(&format!("  THD (SPICE):      {:.2} dB\n", self.thd_spice));
            summary.push_str(&format!("  THD (melange):    {:.2} dB\n", self.thd_melange));
            summary.push_str(&format!("  THD Error:        {:.2} dB\n", self.thd_error_db));
        }

        if !self.failures.is_empty() {
            summary.push('\n');
            summary.push_str("Failures:\n");
            for failure in &self.failures {
                summary.push_str(&format!("  - {}\n", failure));
            }
        }

        summary
    }

    /// Check if the comparison passed all tolerance checks
    pub fn is_passed(&self) -> bool {
        self.passed
    }

    /// Get the worst metric (highest relative violation of tolerance)
    pub fn worst_metric(&self, config: &ComparisonConfig) -> Option<(String, f64)> {
        let mut worst = None;
        let mut worst_ratio = 0.0;

        let metrics = [
            ("RMS Error", self.rms_error / config.rms_error_tolerance),
            ("Peak Error", self.peak_error / config.peak_error_tolerance),
            ("Max Rel Error", self.max_relative_error / config.max_relative_tolerance),
            ("Correlation", (1.0 - self.correlation_coefficient) / (1.0 - config.correlation_min)),
            ("THD Error", self.thd_error_db.abs() / config.thd_error_tolerance_db),
        ];

        for (name, ratio) in metrics {
            if ratio > worst_ratio {
                worst_ratio = ratio;
                worst = Some((name.to_string(), ratio));
            }
        }

        worst
    }
}

/// Compare two signals with configurable tolerances
///
/// # Arguments
///
/// * `reference` - The reference signal (typically from SPICE)
/// * `actual` - The signal to validate (typically from melange)
/// * `config` - Comparison configuration with tolerance settings
///
/// # Returns
///
/// A `ComparisonReport` containing all metrics and pass/fail status
///
/// # Example
///
/// ```rust
/// use melange_validate::comparison::{Signal, ComparisonConfig, compare_signals};
///
/// let reference = Signal::new(vec![0.0, 0.5, 1.0, 0.5, 0.0], 44100.0, "reference");
/// let actual = Signal::new(vec![0.001, 0.501, 1.002, 0.499, 0.001], 44100.0, "actual");
///
/// let config = ComparisonConfig::default();
/// let report = compare_signals(&reference, &actual, &config);
///
/// println!("{}", report.summary());
/// ```
pub fn compare_signals(
    reference: &Signal,
    actual: &Signal,
    config: &ComparisonConfig,
) -> ComparisonReport {
    // Ensure signals have the same sample rate
    let resampled_actual = if (reference.sample_rate - actual.sample_rate).abs() > f64::EPSILON {
        actual.resample(reference.sample_rate)
    } else {
        actual.clone()
    };

    // Use the shorter length
    let len = reference.len().min(resampled_actual.len());
    if len == 0 {
        return ComparisonReport {
            circuit_name: String::new(),
            node_name: actual.name.clone(),
            sample_count: 0,
            sample_rate: reference.sample_rate,
            rms_error: f64::NAN,
            peak_error: f64::NAN,
            max_relative_error: f64::NAN,
            mean_absolute_error: f64::NAN,
            normalized_rms_error: f64::NAN,
            correlation_coefficient: f64::NAN,
            snr_db: f64::NAN,
            thd_spice: f64::NAN,
            thd_melange: f64::NAN,
            thd_error_db: f64::NAN,
            passed: false,
            failures: vec!["Empty signals".to_string()],
            absolute_errors: None,
            relative_errors: None,
        };
    }

    let ref_slice = &reference.samples[..len];
    let act_slice = &resampled_actual.samples[..len];

    // Compute error signal
    let errors: Vec<f64> = ref_slice
        .iter()
        .zip(act_slice.iter())
        .map(|(r, a)| a - r)
        .collect();

    // 1. RMS Error: sqrt(mean(error^2))
    let mean_sq_error: f64 = errors.iter().map(|&e| e * e).sum::<f64>() / len as f64;
    let rms_error = mean_sq_error.sqrt();

    // 2. Peak Error: max(|error|)
    let peak_error = errors.iter().map(|&e| e.abs()).fold(0.0, f64::max);

    // 3. Mean Absolute Error: mean(|error|)
    let mean_absolute_error = errors.iter().map(|&e| e.abs()).sum::<f64>() / len as f64;

    // 4. Max Relative Error: max(|error| / |reference|), handling near-zero values
    let max_relative_error = ref_slice
        .iter()
        .zip(errors.iter())
        .map(|(r, e)| {
            if r.abs() > 1e-12 {
                (e / r).abs()
            } else {
                0.0 // Ignore relative error when reference is near zero
            }
        })
        .fold(0.0, f64::max);

    // 5. Normalized RMS Error (relative to reference RMS)
    let ref_rms = reference.rms();
    let normalized_rms_error = if ref_rms > 1e-12 {
        rms_error / ref_rms
    } else {
        0.0
    };

    // 6. Pearson Correlation Coefficient
    let ref_mean = ref_slice.iter().sum::<f64>() / len as f64;
    let act_mean = act_slice.iter().sum::<f64>() / len as f64;

    let mut num = 0.0;
    let mut ref_denom = 0.0;
    let mut act_denom = 0.0;

    for i in 0..len {
        let ref_diff = ref_slice[i] - ref_mean;
        let act_diff = act_slice[i] - act_mean;
        num += ref_diff * act_diff;
        ref_denom += ref_diff * ref_diff;
        act_denom += act_diff * act_diff;
    }

    let correlation_coefficient = if ref_denom > 0.0 && act_denom > 0.0 {
        num / (ref_denom.sqrt() * act_denom.sqrt())
    } else {
        1.0 // Both signals are constant
    };

    // 7. SNR in dB (remove DC offset before computing power for proper AC SNR)
    let ref_dc = ref_slice.iter().sum::<f64>() / len as f64;
    let signal_power = ref_slice.iter().map(|&r| (r - ref_dc) * (r - ref_dc)).sum::<f64>() / len as f64;
    let noise_power = errors.iter().map(|&e| e * e).sum::<f64>() / len as f64;
    let snr_db = if noise_power > 0.0 && signal_power > 0.0 {
        10.0 * (signal_power / noise_power).log10()
    } else if noise_power == 0.0 {
        f64::INFINITY
    } else {
        0.0 // No signal (pure DC)
    };

    // 8. THD Computation (spectral analysis)
    let (thd_spice, thd_melange, thd_error_db) = if config.skip_thd {
        (f64::NAN, f64::NAN, f64::NAN)
    } else {
        compute_thd_metrics(reference, &resampled_actual, len)
    };

    // Determine pass/fail
    let mut failures = Vec::new();

    if normalized_rms_error > config.rms_error_tolerance {
        failures.push(format!(
            "RMS error {:.6e} exceeds tolerance {:.6e}",
            normalized_rms_error, config.rms_error_tolerance
        ));
    }

    if peak_error > config.peak_error_tolerance {
        failures.push(format!(
            "Peak error {:.6e} exceeds tolerance {:.6e}",
            peak_error, config.peak_error_tolerance
        ));
    }

    if max_relative_error > config.max_relative_tolerance {
        failures.push(format!(
            "Max relative error {:.6e} exceeds tolerance {:.6e}",
            max_relative_error, config.max_relative_tolerance
        ));
    }

    if correlation_coefficient < config.correlation_min {
        failures.push(format!(
            "Correlation {:.8} below minimum {:.8}",
            correlation_coefficient, config.correlation_min
        ));
    }

    if !config.skip_thd && (!thd_error_db.is_finite() || thd_error_db.abs() > config.thd_error_tolerance_db) {
        failures.push(format!(
            "THD error {:.2} dB exceeds tolerance {:.2} dB",
            thd_error_db.abs(),
            config.thd_error_tolerance_db
        ));
    }

    let passed = failures.is_empty();

    ComparisonReport {
        circuit_name: String::new(), // To be filled by caller
        node_name: actual.name.clone(),
        sample_count: len,
        sample_rate: reference.sample_rate,
        rms_error,
        peak_error,
        max_relative_error,
        mean_absolute_error,
        normalized_rms_error,
        correlation_coefficient,
        snr_db,
        thd_spice,
        thd_melange,
        thd_error_db,
        passed,
        failures,
        absolute_errors: Some(errors.iter().map(|&e| e.abs()).collect()),
        relative_errors: Some(
            ref_slice
                .iter()
                .zip(errors.iter())
                .map(|(r, e)| if r.abs() > 1e-12 { (e / r).abs() } else { 0.0 })
                .collect()
        ),
    }
}

/// Compute THD (Total Harmonic Distortion) for both signals
fn compute_thd_metrics(reference: &Signal, actual: &Signal, len: usize) -> (f64, f64, f64) {
    // Use a reasonable FFT size (next power of 2)
    let fft_size = len.next_power_of_two();
    
    // Compute THD for reference
    let thd_ref = compute_thd(&reference.samples[..len], fft_size, reference.sample_rate);
    
    // Compute THD for actual
    let thd_act = compute_thd(&actual.samples[..len], fft_size, actual.sample_rate);
    
    // THD error in dB
    let thd_error = if thd_ref.is_finite() && thd_act.is_finite() {
        (thd_act - thd_ref).abs()
    } else {
        f64::NAN
    };
    
    (thd_ref, thd_act, thd_error)
}

/// Compute THD in dB using FFT analysis
///
/// THD is computed as the ratio of harmonic power to fundamental power
fn compute_thd(samples: &[f64], fft_size: usize, sample_rate: f64) -> f64 {
    if samples.len() < 64 {
        return f64::NAN;
    }

    // Apply Hann window to reduce spectral leakage
    let n = samples.len();
    let windowed: Vec<f64> = samples
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            let hann = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos());
            s * hann
        })
        .collect();

    // Prepare FFT input
    let mut input: Vec<Complex<f64>> = windowed
        .iter()
        .map(|&s| Complex::new(s, 0.0))
        .chain(std::iter::repeat(Complex::new(0.0, 0.0)))
        .take(fft_size)
        .collect();

    // Perform FFT
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(fft_size);
    fft.process(&mut input);

    // Compute magnitude spectrum (first half only, since it's symmetric)
    let half_size = fft_size / 2;
    let magnitudes: Vec<f64> = input[..half_size]
        .iter()
        .map(|c| c.norm())
        .collect();

    // Find fundamental frequency: first significant peak (low→high), not global max.
    // This prevents strong harmonics from being misidentified as the fundamental.
    let search_start = (20.0 * fft_size as f64 / sample_rate) as usize; // Skip below 20 Hz
    let search_end = (20000.0 * fft_size as f64 / sample_rate) as usize; // Limit to 20 kHz

    // First pass: find the global max to establish a threshold
    let global_max = magnitudes[search_start..search_end.min(half_size)]
        .iter()
        .copied()
        .fold(0.0_f64, f64::max);

    // Second pass: find the first peak above 10% of the global max
    let threshold = global_max * 0.1;
    let mut fundamental_idx = 0;
    let mut fundamental_mag = 0.0;

    for i in search_start..search_end.min(half_size) {
        if magnitudes[i] > threshold {
            // Check if this is a local peak (higher than neighbors)
            let prev = if i > 0 { magnitudes[i - 1] } else { 0.0 };
            let next = if i + 1 < half_size { magnitudes[i + 1] } else { 0.0 };
            if magnitudes[i] >= prev && magnitudes[i] >= next {
                fundamental_idx = i;
                fundamental_mag = magnitudes[i];
                break;
            }
        }
    }

    // Fallback to global max if no peak found
    if fundamental_mag < 1e-12 {
        for i in search_start..search_end.min(half_size) {
            if magnitudes[i] > fundamental_mag {
                fundamental_mag = magnitudes[i];
                fundamental_idx = i;
            }
        }
    }

    if fundamental_mag < 1e-12 || fundamental_idx == 0 {
        return f64::NAN;
    }

    // Frequency resolution
    let freq_resolution = sample_rate / fft_size as f64;
    let fundamental_freq = fundamental_idx as f64 * freq_resolution;

    // Sum harmonic powers (2nd through 10th harmonic)
    let mut harmonic_power = 0.0;
    for harmonic in 2..=10 {
        let harmonic_freq = fundamental_freq * harmonic as f64;
        let harmonic_idx = (harmonic_freq / freq_resolution) as usize;
        
        if harmonic_idx < half_size {
            // Average around the bin to capture spread
            let start = harmonic_idx.saturating_sub(1);
            let end = (harmonic_idx + 2).min(half_size);
            let peak_mag: f64 = magnitudes[start..end].iter().copied().fold(0.0, f64::max);
            harmonic_power += peak_mag * peak_mag;
        }
    }

    // Compute THD in dB
    let fundamental_power = fundamental_mag * fundamental_mag;
    if fundamental_power > 0.0 {
        let thd_ratio = harmonic_power / fundamental_power;
        10.0 * thd_ratio.log10()
    } else {
        f64::NAN
    }
}

/// Batch comparison of multiple signals
pub fn batch_compare(
    circuit_name: impl Into<String>,
    reference_signals: &[(&str, Signal)],
    actual_signals: &[(&str, Signal)],
    config: &ComparisonConfig,
) -> Vec<ComparisonReport> {
    let name = circuit_name.into();
    let mut reports = Vec::new();

    for (node_name, ref_sig) in reference_signals {
        if let Some((_, act_sig)) = actual_signals.iter().find(|(n, _)| n == node_name) {
            let mut report = compare_signals(ref_sig, act_sig, config);
            report.circuit_name = name.clone();
            report.node_name = node_name.to_string();
            reports.push(report);
        }
    }

    reports
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signal_rms() {
        // Sine wave with amplitude 1.0 has RMS of 1/sqrt(2)
        let samples: Vec<f64> = (0..1000)
            .map(|i| (2.0 * PI * i as f64 / 1000.0).sin())
            .collect();
        let signal = Signal::new(samples, 44100.0, "test");
        let rms = signal.rms();
        let expected = 1.0 / 2.0f64.sqrt();
        assert!((rms - expected).abs() < 0.01, "RMS {} != {}", rms, expected);
    }

    #[test]
    fn test_compare_identical_signals() {
        let samples: Vec<f64> = (0..1000)
            .map(|i| (2.0 * PI * 1000.0 * i as f64 / 44100.0).sin())
            .collect();
        
        let reference = Signal::new(samples.clone(), 44100.0, "reference");
        let actual = Signal::new(samples, 44100.0, "actual");

        let config = ComparisonConfig::strict();
        let report = compare_signals(&reference, &actual, &config);

        assert!(report.passed, "Identical signals should pass");
        assert!(report.rms_error < 1e-10, "RMS error should be near zero");
        assert!((report.correlation_coefficient - 1.0).abs() < 1e-10, "Correlation should be 1.0");
    }

    #[test]
    fn test_compare_different_signals() {
        let ref_samples: Vec<f64> = (0..1000)
            .map(|i| (2.0 * PI * 1000.0 * i as f64 / 44100.0).sin())
            .collect();
        let act_samples: Vec<f64> = (0..1000)
            .map(|i| 0.5 * (2.0 * PI * 1000.0 * i as f64 / 44100.0).sin()) // Half amplitude
            .collect();
        
        let reference = Signal::new(ref_samples, 44100.0, "reference");
        let actual = Signal::new(act_samples, 44100.0, "actual");

        let config = ComparisonConfig::strict();
        let report = compare_signals(&reference, &actual, &config);

        assert!(!report.passed, "Different signals should fail strict check");
        assert!(report.rms_error > 0.1, "RMS error should be significant");
    }

    #[test]
    fn test_signal_resample() {
        let samples: Vec<f64> = (0..100)
            .map(|i| (2.0 * PI * i as f64 / 100.0).sin())
            .collect();
        let signal = Signal::new(samples, 1000.0, "test");
        
        let resampled = signal.resample(500.0);
        
        // Duration should be preserved
        assert!((resampled.duration() - signal.duration()).abs() < 0.001);
        // Length should be roughly halved
        assert_eq!(resampled.len(), 50);
    }

    #[test]
    fn test_comparison_config_levels() {
        let strict = ComparisonConfig::strict();
        let default = ComparisonConfig::default();
        let relaxed = ComparisonConfig::relaxed();

        assert!(strict.rms_error_tolerance < default.rms_error_tolerance);
        assert!(default.rms_error_tolerance < relaxed.rms_error_tolerance);
        assert!(strict.correlation_min > default.correlation_min);
    }
}
