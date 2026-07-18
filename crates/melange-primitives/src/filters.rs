//! Filter primitives for audio DSP.
//!
//! All filters are designed for real-time use:
//! - No heap allocation
//! - No dynamic dispatch
//! - `Copy` where possible for easy voice duplication

/// One-pole lowpass filter (6 dB/octave).
///
/// Transfer function: H(z) = g / (1 - (1-g)*z^-1)
/// where g = 1 - exp(-2π * fc / fs)
///
/// Suitable for simple smoothing, envelope followers, basic tone control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OnePoleLpf {
    state: f64,
    g: f64,  // feedforward coefficient
    g1: f64, // feedback coefficient (1 - g)
}

impl OnePoleLpf {
    /// Create a new one-pole LPF at the given cutoff frequency.
    ///
    /// # Arguments
    /// * `fc` - Cutoff frequency in Hz
    /// * `fs` - Sample rate in Hz
    pub fn new(fc: f64, fs: f64) -> Self {
        // Clamp cutoff below Nyquist to prevent unstable coefficients
        let fc_safe = fc.min(fs * 0.499);
        let g = 1.0 - (-core::f64::consts::TAU * fc_safe / fs).exp();
        Self {
            state: 0.0,
            g,
            g1: 1.0 - g,
        }
    }

    /// Set the cutoff frequency (smoothly update coefficients).
    pub fn set_cutoff(&mut self, fc: f64, fs: f64) {
        let fc_safe = fc.min(fs * 0.499);
        self.g = 1.0 - (-core::f64::consts::TAU * fc_safe / fs).exp();
        self.g1 = 1.0 - self.g;
    }

    /// Process a single sample.
    #[inline(always)]
    pub fn process(&mut self, input: f64) -> f64 {
        self.state = self.g * input + self.g1 * self.state;
        self.state
    }

    /// Reset state to zero.
    pub fn reset(&mut self) {
        self.state = 0.0;
    }

    /// Reset state to a specific value (useful for DC initialization).
    pub fn reset_to(&mut self, value: f64) {
        self.state = value;
    }
}

impl Default for OnePoleLpf {
    fn default() -> Self {
        Self::new(1000.0, 44100.0)
    }
}

/// One-pole highpass filter (6 dB/octave).
///
/// Complementary to OnePoleLpf: output = input - lpf_output
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OnePoleHpf {
    lpf: OnePoleLpf,
}

impl OnePoleHpf {
    /// Create a new one-pole HPF at the given cutoff frequency.
    pub fn new(fc: f64, fs: f64) -> Self {
        Self {
            lpf: OnePoleLpf::new(fc, fs),
        }
    }

    /// Set the cutoff frequency.
    pub fn set_cutoff(&mut self, fc: f64, fs: f64) {
        self.lpf.set_cutoff(fc, fs);
    }

    /// Process a single sample.
    #[inline(always)]
    pub fn process(&mut self, input: f64) -> f64 {
        let low = self.lpf.process(input);
        input - low
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.lpf.reset();
    }
}

impl Default for OnePoleHpf {
    fn default() -> Self {
        Self::new(20.0, 44100.0)
    }
}

/// Topology-Preserving Transform (TPT) lowpass filter (Zavalishin).
///
/// Uses trapezoidal integration (bilinear transform) with internal saturation
/// for superior behavior at high frequencies and large input signals.
///
/// This is essentially a ZDF (Zero Delay Feedback) integrator wrapped as a one-pole.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TptLpf {
    s: f64, // integrator state
    g: f64, // tan(ω/2) coefficient
}

impl TptLpf {
    /// Create a new TPT LPF at the given cutoff frequency.
    pub fn new(fc: f64, fs: f64) -> Self {
        // Clamp below Nyquist to prevent tan() blowup
        let fc_safe = fc.min(fs * 0.499);
        let g = (core::f64::consts::PI * fc_safe / fs).tan();
        Self { s: 0.0, g }
    }

    /// Set the cutoff frequency.
    pub fn set_cutoff(&mut self, fc: f64, fs: f64) {
        let fc_safe = fc.min(fs * 0.499);
        self.g = (core::f64::consts::PI * fc_safe / fs).tan();
    }

    /// Process a single sample (linear, no saturation).
    #[inline(always)]
    pub fn process(&mut self, input: f64) -> f64 {
        // TPT structure: v = (input - s) * g
        //                y = v + s
        //                s = y + v (state update)
        let v = (input - self.s) * self.g;
        let y = v + self.s;
        self.s = y + v;
        y
    }

    /// Process with tanh saturation (for nonlinear filter behavior).
    #[inline(always)]
    pub fn process_saturated(&mut self, input: f64) -> f64 {
        let v = (input - self.s).tanh() * self.g;
        let y = v + self.s;
        self.s = y + v;
        y
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.s = 0.0;
    }

    /// Get the current state (useful for analysis).
    pub fn state(&self) -> f64 {
        self.s
    }
}

impl Default for TptLpf {
    fn default() -> Self {
        Self::new(1000.0, 44100.0)
    }
}

/// DC blocker (highpass at very low frequency).
///
/// Removes DC offset while preserving audible frequencies.
/// Default cutoff is 20 Hz (configurable).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DcBlocker {
    hpf: OnePoleHpf,
}

impl DcBlocker {
    /// Create a new DC blocker with the given cutoff (default: 20 Hz).
    pub fn new(fc: f64, fs: f64) -> Self {
        Self {
            hpf: OnePoleHpf::new(fc, fs),
        }
    }

    /// Create with default 20 Hz cutoff.
    pub fn new_default(fs: f64) -> Self {
        Self::new(20.0, fs)
    }

    /// Process a single sample.
    #[inline(always)]
    pub fn process(&mut self, input: f64) -> f64 {
        self.hpf.process(input)
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.hpf.reset();
    }
}

impl Default for DcBlocker {
    fn default() -> Self {
        Self::new_default(44100.0)
    }
}

/// Biquad filter (transposed Direct Form II).
///
/// General second-order filter with configurable type:
/// - Lowpass, Highpass, Bandpass
/// - Peaking EQ, Low shelf, High shelf
///
/// Uses transposed DF-II for better numerical properties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Biquad {
    // State variables (unit delays)
    s1: f64,
    s2: f64,
    // Coefficients
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
}

/// Biquad filter types.
///
/// Shelf filters take an RBJ shelf slope parameter `slope` (the cookbook's
/// `S`). `slope = 1.0` is the standard maximally-steep monotonic shelf; use
/// [`BiquadType::low_shelf`] / [`BiquadType::high_shelf`] for that default.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BiquadType {
    Lowpass { fc: f64, q: f64 },
    Highpass { fc: f64, q: f64 },
    Bandpass { fc: f64, q: f64 },
    PeakingEq { fc: f64, q: f64, gain_db: f64 },
    LowShelf { fc: f64, gain_db: f64, slope: f64 },
    HighShelf { fc: f64, gain_db: f64, slope: f64 },
}

impl BiquadType {
    /// Low shelf with the default RBJ slope S = 1.0.
    pub fn low_shelf(fc: f64, gain_db: f64) -> Self {
        BiquadType::LowShelf {
            fc,
            gain_db,
            slope: 1.0,
        }
    }

    /// High shelf with the default RBJ slope S = 1.0.
    pub fn high_shelf(fc: f64, gain_db: f64) -> Self {
        BiquadType::HighShelf {
            fc,
            gain_db,
            slope: 1.0,
        }
    }
}

/// RBJ audio-EQ-cookbook shelf intermediate term `2*sqrt(A)*alpha`.
///
/// From the cookbook (shelving-filter case):
///   alpha = sin(w0)/2 * sqrt( (A + 1/A)*(1/S - 1) + 2 )
/// so:
///   2*sqrt(A)*alpha = sqrt(A) * sin(w0) * sqrt( (A + 1/A)*(1/S - 1) + 2 )
/// At S = 1 this reduces to sqrt(2*A) * sin(w0).
#[inline]
fn rbj_shelf_term(a: f64, sin_w0: f64, slope: f64) -> f64 {
    let s = slope.max(1e-6); // guard against division by zero / negative S
    let alpha = sin_w0 / 2.0 * ((a + 1.0 / a) * (1.0 / s - 1.0) + 2.0).sqrt();
    2.0 * a.sqrt() * alpha
}

impl Biquad {
    /// Create a biquad filter from type specification.
    pub fn new(filter_type: BiquadType, fs: f64) -> Self {
        let mut b = Self {
            s1: 0.0,
            s2: 0.0,
            b0: 0.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
        };
        b.set_type(filter_type, fs);
        b
    }

    /// Recalculate coefficients for a new filter type.
    pub fn set_type(&mut self, filter_type: BiquadType, fs: f64) {
        match filter_type {
            BiquadType::Lowpass { fc, q } => {
                let w0 = core::f64::consts::TAU * fc.min(fs * 0.499) / fs;
                let cos_w0 = w0.cos();
                let sin_w0 = w0.sin();
                let alpha = sin_w0 / (2.0 * q);

                let a0 = 1.0 + alpha;
                self.b0 = ((1.0 - cos_w0) / 2.0) / a0;
                self.b1 = (1.0 - cos_w0) / a0;
                self.b2 = ((1.0 - cos_w0) / 2.0) / a0;
                self.a1 = (-2.0 * cos_w0) / a0;
                self.a2 = (1.0 - alpha) / a0;
            }
            BiquadType::Highpass { fc, q } => {
                let w0 = core::f64::consts::TAU * fc.min(fs * 0.499) / fs;
                let cos_w0 = w0.cos();
                let sin_w0 = w0.sin();
                let alpha = sin_w0 / (2.0 * q);

                let a0 = 1.0 + alpha;
                self.b0 = ((1.0 + cos_w0) / 2.0) / a0;
                self.b1 = -(1.0 + cos_w0) / a0;
                self.b2 = ((1.0 + cos_w0) / 2.0) / a0;
                self.a1 = (-2.0 * cos_w0) / a0;
                self.a2 = (1.0 - alpha) / a0;
            }
            BiquadType::Bandpass { fc, q } => {
                let w0 = core::f64::consts::TAU * fc.min(fs * 0.499) / fs;
                let cos_w0 = w0.cos();
                let sin_w0 = w0.sin();
                let alpha = sin_w0 / (2.0 * q);

                let a0 = 1.0 + alpha;
                self.b0 = alpha / a0;
                self.b1 = 0.0;
                self.b2 = -alpha / a0;
                self.a1 = (-2.0 * cos_w0) / a0;
                self.a2 = (1.0 - alpha) / a0;
            }
            BiquadType::PeakingEq { fc, q, gain_db } => {
                let w0 = core::f64::consts::TAU * fc.min(fs * 0.499) / fs;
                let cos_w0 = w0.cos();
                let sin_w0 = w0.sin();
                let a = 10.0_f64.powf(gain_db / 40.0);
                let alpha = sin_w0 / (2.0 * q);

                let a0 = 1.0 + alpha / a;
                self.b0 = (1.0 + alpha * a) / a0;
                self.b1 = (-2.0 * cos_w0) / a0;
                self.b2 = (1.0 - alpha * a) / a0;
                self.a1 = (-2.0 * cos_w0) / a0;
                self.a2 = (1.0 - alpha / a) / a0;
            }
            BiquadType::LowShelf { fc, gain_db, slope } => {
                let w0 = core::f64::consts::TAU * fc.min(fs * 0.499) / fs;
                let cos_w0 = w0.cos();
                let sin_w0 = w0.sin();
                let a = 10.0_f64.powf(gain_db / 40.0);
                let shelf = rbj_shelf_term(a, sin_w0, slope); // 2*sqrt(A)*alpha

                let a0 = (a + 1.0) + (a - 1.0) * cos_w0 + shelf;
                self.b0 = (a * ((a + 1.0) - (a - 1.0) * cos_w0 + shelf)) / a0;
                self.b1 = (2.0 * a * ((a - 1.0) - (a + 1.0) * cos_w0)) / a0;
                self.b2 = (a * ((a + 1.0) - (a - 1.0) * cos_w0 - shelf)) / a0;
                self.a1 = (-2.0 * ((a - 1.0) + (a + 1.0) * cos_w0)) / a0;
                self.a2 = ((a + 1.0) + (a - 1.0) * cos_w0 - shelf) / a0;
            }
            BiquadType::HighShelf { fc, gain_db, slope } => {
                let w0 = core::f64::consts::TAU * fc.min(fs * 0.499) / fs;
                let cos_w0 = w0.cos();
                let sin_w0 = w0.sin();
                let a = 10.0_f64.powf(gain_db / 40.0);
                let shelf = rbj_shelf_term(a, sin_w0, slope); // 2*sqrt(A)*alpha

                let a0 = (a + 1.0) - (a - 1.0) * cos_w0 + shelf;
                self.b0 = (a * ((a + 1.0) + (a - 1.0) * cos_w0 + shelf)) / a0;
                self.b1 = (-2.0 * a * ((a - 1.0) + (a + 1.0) * cos_w0)) / a0;
                self.b2 = (a * ((a + 1.0) + (a - 1.0) * cos_w0 - shelf)) / a0;
                self.a1 = (2.0 * ((a - 1.0) - (a + 1.0) * cos_w0)) / a0;
                self.a2 = ((a + 1.0) - (a - 1.0) * cos_w0 - shelf) / a0;
            }
        }
    }

    /// Process a single sample (transposed DF-II).
    #[inline(always)]
    pub fn process(&mut self, input: f64) -> f64 {
        // Transposed DF-II: better numerical properties
        let output = self.b0 * input + self.s1;
        self.s1 = self.b1 * input - self.a1 * output + self.s2;
        self.s2 = self.b2 * input - self.a2 * output;
        output
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
    }
}

impl Default for Biquad {
    fn default() -> Self {
        Self::new(
            BiquadType::Lowpass {
                fc: 1000.0,
                q: 0.707,
            },
            44100.0,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_one_pole_lpf_basic() {
        let mut lpf = OnePoleLpf::new(1000.0, 44100.0);
        let output = lpf.process(1.0);
        assert!(output > 0.0 && output < 1.0);
    }

    #[test]
    fn test_dc_blocker_removes_dc() {
        let mut dc = DcBlocker::new_default(44100.0);
        // Process DC input (needs many samples for 20Hz HPF to settle at 44.1kHz)
        for _ in 0..10000 {
            dc.process(1.0);
        }
        // Output should approach 0
        let output = dc.process(1.0);
        assert!(
            output.abs() < 0.05,
            "DC blocker failed to attenuate DC: {}",
            output
        );
    }

    #[test]
    fn test_biquad_lowpass() {
        let mut bq = Biquad::new(
            BiquadType::Lowpass {
                fc: 1000.0,
                q: 0.707,
            },
            44100.0,
        );
        let output = bq.process(1.0);
        assert!(output > 0.0 && output <= 1.0);
    }

    /// Magnitude response of a biquad at frequency f (Hz).
    fn biquad_mag(bq: &Biquad, f: f64, fs: f64) -> f64 {
        let w = core::f64::consts::TAU * f / fs;
        let (c1, s1) = (w.cos(), w.sin());
        let (c2, s2) = ((2.0 * w).cos(), (2.0 * w).sin());
        let num_re = bq.b0 + bq.b1 * c1 + bq.b2 * c2;
        let num_im = -(bq.b1 * s1 + bq.b2 * s2);
        let den_re = 1.0 + bq.a1 * c1 + bq.a2 * c2;
        let den_im = -(bq.a1 * s1 + bq.a2 * s2);
        ((num_re * num_re + num_im * num_im) / (den_re * den_re + den_im * den_im)).sqrt()
    }

    /// Pin shelf coefficients against hand-computed RBJ audio-EQ-cookbook
    /// values at fs=48kHz, fc=1kHz, +6 dB, S=1.
    ///
    /// Reference (cookbook): A = 10^(6/40), w0 = 2*pi*1000/48000,
    /// alpha = sin(w0)/2 * sqrt((A + 1/A)*(1/S - 1) + 2) = sin(w0)/2 * sqrt(2),
    /// shelf term = 2*sqrt(A)*alpha = sqrt(2A)*sin(w0).
    /// The pre-fix code used 2*sqrt(A)*sin(w0) (i.e. alpha = sin(w0), a Q=0.5
    /// resonance-style term), which is NOT any RBJ S=1 shelf.
    #[test]
    fn test_shelf_coefficients_match_rbj_cookbook() {
        let fs = 48000.0;
        let low = Biquad::new(BiquadType::low_shelf(1000.0, 6.0), fs);
        let expected_low = [
            1.0325624832475901,   // b0
            -1.8388568718996405,  // b1
            0.8287476843124698,   // b2
            -1.8444568671609198,  // a1
            0.8557101722987808,   // a2
        ];
        let got_low = [low.b0, low.b1, low.b2, low.a1, low.a2];
        for (g, e) in got_low.iter().zip(expected_low.iter()) {
            assert!((g - e).abs() < 1e-12, "low shelf: got {:?}, expected {:?}", got_low, expected_low);
        }

        let high = Biquad::new(BiquadType::high_shelf(1000.0, 6.0), fs);
        let expected_high = [
            1.9323405094996573,   // b0
            -3.5641187224398734,  // b1
            1.6535234303238655,   // b2
            -1.7808674067995507,  // a1
            0.8026126241831999,   // a2
        ];
        let got_high = [high.b0, high.b1, high.b2, high.a1, high.a2];
        for (g, e) in got_high.iter().zip(expected_high.iter()) {
            assert!((g - e).abs() < 1e-12, "high shelf: got {:?}, expected {:?}", got_high, expected_high);
        }
    }

    /// RBJ shelf response properties: low shelf has full gain at DC and unity
    /// at Nyquist (high shelf mirrored), and exactly half the dB gain at fc
    /// (the cookbook defines fc as the shelf midpoint frequency).
    #[test]
    fn test_shelf_response_properties() {
        let fs = 48000.0;
        let gain_db = 6.0;
        let full = 10.0_f64.powf(gain_db / 20.0);
        let half = 10.0_f64.powf(gain_db / 40.0); // half gain in dB

        let low = Biquad::new(BiquadType::low_shelf(1000.0, gain_db), fs);
        assert!((biquad_mag(&low, 1e-3, fs) - full).abs() < 1e-6);
        assert!((biquad_mag(&low, fs / 2.0, fs) - 1.0).abs() < 1e-6);
        assert!(
            (biquad_mag(&low, 1000.0, fs) - half).abs() < 1e-6,
            "low shelf midpoint gain at fc: got {}, expected {}",
            biquad_mag(&low, 1000.0, fs),
            half
        );

        let high = Biquad::new(BiquadType::high_shelf(1000.0, gain_db), fs);
        assert!((biquad_mag(&high, 1e-3, fs) - 1.0).abs() < 1e-6);
        assert!((biquad_mag(&high, fs / 2.0, fs) - full).abs() < 1e-6);
        assert!(
            (biquad_mag(&high, 1000.0, fs) - half).abs() < 1e-6,
            "high shelf midpoint gain at fc: got {}, expected {}",
            biquad_mag(&high, 1000.0, fs),
            half
        );
    }

    #[test]
    fn test_tpt_lpf() {
        let mut tpt = TptLpf::new(1000.0, 44100.0);
        let output = tpt.process(1.0);
        assert!(output > 0.0 && output <= 1.0);
    }
}
