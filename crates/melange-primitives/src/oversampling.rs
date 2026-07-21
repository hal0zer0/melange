//! Polyphase IIR oversampling.
//!
//! Uses cascaded first-order allpass sections in a two-path polyphase
//! half-band structure (Valenzuela & Constantinides 1983, popularized by
//! Laurent de Soras' hiir library). Each branch of the half-band filter is
//! clocked once per LOW-rate sample:
//!
//! - Interpolator (1 -> 2): `out[2n] = A_even(x[n])`, `out[2n+1] = A_odd(x[n])`.
//! - Decimator   (2 -> 1): `y[n] = (A_even(x[2n+1]) + A_odd(x[2n])) / 2`.
//!
//! The branch allpass cells realize `(c + z^-2) / (1 + c*z^-2)` at the
//! internal (high) rate; clocking each cell once per low-rate sample makes it
//! first-order in the branch's own clock, which is exactly the polyphase
//! realization. Clocking a branch more than once per low-rate sample (the
//! pre-2026-07 melange bug) collapses the cells to first-order in the
//! internal-rate z and destroys the stopband entirely.

use crate::filters::OnePoleLpf;

/// Allpass filter section for polyphase filters.
///
/// First-order allpass in the branch clock: H(z) = (c + z^-1) / (1 + c*z^-1)
/// where c is the allpass coefficient.
#[derive(Debug, Clone, Copy, PartialEq)]
struct AllpassSection {
    c: f64,  // coefficient
    x1: f64, // input delay
    y1: f64, // output delay
}

impl AllpassSection {
    fn new(c: f64) -> Self {
        Self {
            c,
            x1: 0.0,
            y1: 0.0,
        }
    }

    #[inline(always)]
    fn process(&mut self, input: f64) -> f64 {
        // y[n] = c*x[n] + x[n-1] - c*y[n-1]
        let y = self.c * input + self.x1 - self.c * self.y1;
        self.x1 = input;
        self.y1 = y;
        y
    }

    fn reset(&mut self) {
        self.x1 = 0.0;
        self.y1 = 0.0;
    }
}

/// Half-band filter coefficients.
///
/// All sets generated with the published hiir designer
/// `PolyphaseIir2Designer::compute_coefs_spec_order_tbw(n, tbw)`
/// (Laurent de Soras, 2005, WTFPL; <http://ldesoras.free.fr/prod.html#src_hiir>,
/// mirrored at <https://github.com/unevens/hiir>), based on
/// R.A. Valenzuela, A.G. Constantinides, "Digital Signal Processing Schemes
/// for Efficient Interpolation and Decimation", IEE Proceedings, Dec 1983.
///
/// `tbw` is the transition bandwidth normalized to the filter's running
/// (internal) rate: passband edge = (0.5 - tbw)/2, stopband edge =
/// (0.5 + tbw)/2. The "STEEP" sets use tbw = 0.04, i.e. passband to
/// 0.23 * f_internal (20.3 kHz at a 44.1 kHz host rate for a 2x stage).
/// The rejection figures below are worst-case stopband magnitudes verified
/// by direct evaluation of |0.5*(A0(z^2) + z^-1*A1(z^2))| and match the
/// designer's `compute_atten` prediction to 0.01 dB.
pub mod coefficients {
    /// 5-section steep half-band: tbw = 0.04, -62.1 dB stopband.
    /// `compute_coefs_spec_order_tbw(5, 0.04)`.
    pub const HB_STEEP_5SECTION: [f64; 5] = [
        0.09360402485619887,
        0.3153060787098146,
        0.5575274910154708,
        0.7598272203250263,
        0.922691853357221,
    ];

    /// 7-section steep half-band: tbw = 0.04, -86.9 dB stopband.
    /// `compute_coefs_spec_order_tbw(7, 0.04)`.
    /// Default for 2x oversampling and for the base-Nyquist (outer) stage
    /// of the 4x cascade.
    pub const HB_STEEP_7SECTION: [f64; 7] = [
        0.05180201146164933,
        0.1879784418196106,
        0.3650536901969154,
        0.5423273752059077,
        0.6977781305199374,
        0.8282265929955739,
        0.9431266539721422,
    ];

    /// 9-section steep half-band: tbw = 0.04, -111.7 dB stopband.
    /// `compute_coefs_spec_order_tbw(9, 0.04)`.
    pub const HB_STEEP_9SECTION: [f64; 9] = [
        0.03270139024814252,
        0.12292192727433598,
        0.2509245293149022,
        0.39384824646144356,
        0.5334382037734814,
        0.659440556757765,
        0.7691751797540284,
        0.8654980725777349,
        0.9549727484449699,
    ];

    /// 3-section wide-transition half-band for the inner (higher-rate) stage
    /// of the 4x cascade: tbw = 0.27, -95.1 dB stopband over its design band.
    /// `compute_coefs_spec_order_tbw(3, 0.27)`.
    ///
    /// tbw follows the hiir cascade rule `TBW[stage] = (TBW[stage-1]+0.5)/2`
    /// (= (0.04+0.5)/2 = 0.27): the inner stage only needs to protect the
    /// spectrum the outer (steep) stage will keep.
    pub const HB_WIDE_3SECTION: [f64; 3] =
        [0.06687030230470327, 0.2756202830232181, 0.6763597685457587];
}

/// Half-band filter using allpass decomposition.
///
/// Two parallel allpass chains; even-indexed coefficients form the A0 (even)
/// branch, odd-indexed the A1 (odd) branch, per the standard polyphase
/// decomposition (hiir convention).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HalfBandFilter<const N_SECTIONS: usize> {
    /// Allpass sections for even phase
    even: [AllpassSection; N_SECTIONS],
    /// Allpass sections for odd phase
    odd: [AllpassSection; N_SECTIONS],
}

/// Type aliases for common configurations
pub type HalfBand2 = HalfBandFilter<2>;
pub type HalfBand3 = HalfBandFilter<3>;
pub type HalfBand4 = HalfBandFilter<4>;
pub type HalfBand7 = HalfBandFilter<7>;

impl<const N_SECTIONS: usize> HalfBandFilter<N_SECTIONS> {
    /// Create a half-band filter with the given coefficients.
    ///
    /// Coefficients are split between even and odd allpass chains using the
    /// standard polyphase decomposition: even-indexed coefficients go to the
    /// even path, odd-indexed coefficients go to the odd path. This produces
    /// a proper half-band filter via allpass complementary pair.
    pub fn new(coeffs: [f64; N_SECTIONS]) -> Self {
        let even = [AllpassSection::new(0.0); N_SECTIONS];
        let odd = [AllpassSection::new(0.0); N_SECTIONS];
        let mut filter = Self { even, odd };

        let mut even_idx = 0;
        let mut odd_idx = 0;
        for (i, &c) in coeffs.iter().enumerate() {
            if i % 2 == 0 {
                filter.even[even_idx] = AllpassSection::new(c);
                even_idx += 1;
            } else {
                filter.odd[odd_idx] = AllpassSection::new(c);
                odd_idx += 1;
            }
        }

        filter
    }

    /// Number of active sections in the even allpass chain.
    #[inline(always)]
    fn even_len(&self) -> usize {
        N_SECTIONS.div_ceil(2)
    }

    /// Number of active sections in the odd allpass chain.
    #[inline(always)]
    fn odd_len(&self) -> usize {
        N_SECTIONS / 2
    }

    /// Interpolator step: produce the two internal-rate samples for one
    /// low-rate input sample.
    ///
    /// Returns `(even_output, odd_output)` = `(out[2n], out[2n+1])`.
    /// Each branch is clocked exactly once per call.
    #[inline(always)]
    fn process(&mut self, input: f64) -> (f64, f64) {
        let mut even_out = input;
        for i in 0..self.even_len() {
            even_out = self.even[i].process(even_out);
        }

        let mut odd_out = input;
        for i in 0..self.odd_len() {
            odd_out = self.odd[i].process(odd_out);
        }

        (even_out, odd_out)
    }

    /// Decimator step: consume a pair of internal-rate samples
    /// (`x0` earlier, `x1` later), produce one low-rate output sample.
    ///
    /// hiir convention (`Downsampler2x::process_sample`): the even (A0)
    /// branch filters the LATER sample, the odd (A1) branch the EARLIER
    /// sample; the output is their average. Each branch is clocked exactly
    /// once per output sample — this is what makes the branch cells act as
    /// `(c + z^-2)/(1 + c*z^-2)` at the internal rate.
    #[inline(always)]
    fn decimate(&mut self, x0: f64, x1: f64) -> f64 {
        let mut even_out = x1;
        for i in 0..self.even_len() {
            even_out = self.even[i].process(even_out);
        }

        let mut odd_out = x0;
        for i in 0..self.odd_len() {
            odd_out = self.odd[i].process(odd_out);
        }

        (even_out + odd_out) * 0.5
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        for section in &mut self.even {
            section.reset();
        }
        for section in &mut self.odd {
            section.reset();
        }
    }
}

/// 2x oversampler using polyphase IIR half-band filters.
///
/// The oversampling chain: upsample → process at 2x → downsample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Oversampler2x<const N_SECTIONS: usize> {
    /// Anti-imaging filter (upsampling)
    up_filter: HalfBandFilter<N_SECTIONS>,
    /// Anti-aliasing filter (downsampling)
    down_filter: HalfBandFilter<N_SECTIONS>,
    /// Pre-emphasis filter (optional)
    pre_emphasis: OnePoleLpf,
    /// De-emphasis filter
    de_emphasis: OnePoleLpf,
    /// Enable pre/de-emphasis
    use_emphasis: bool,
}

/// ~60 dB-class 2x oversampler (use with `HB_STEEP_5SECTION`).
pub type Oversampler2xFast = Oversampler2x<5>;
/// ~87 dB-class 2x oversampler (use with `HB_STEEP_7SECTION`). Default.
pub type Oversampler2xStandard = Oversampler2x<7>;
/// ~112 dB-class 2x oversampler (use with `HB_STEEP_9SECTION`).
pub type Oversampler2xQuality = Oversampler2x<9>;

impl<const N_SECTIONS: usize> Oversampler2x<N_SECTIONS> {
    /// Create a 2x oversampler with the given half-band quality.
    pub fn new(coeffs: [f64; N_SECTIONS], use_emphasis: bool, fs: f64) -> Self {
        let emphasis_fc = fs * 0.1; // Pre-emphasis at ~10% of Nyquist

        Self {
            up_filter: HalfBandFilter::new(coeffs),
            down_filter: HalfBandFilter::new(coeffs),
            pre_emphasis: OnePoleLpf::new(emphasis_fc, fs),
            de_emphasis: OnePoleLpf::new(emphasis_fc, fs),
            use_emphasis,
        }
    }

    /// Reset all filter states.
    pub fn reset(&mut self) {
        self.up_filter.reset();
        self.down_filter.reset();
        self.pre_emphasis.reset();
        self.de_emphasis.reset();
    }

    /// Process a single sample through the oversampling chain.
    ///
    /// The process_func is called at 2x the base sample rate.
    #[inline(always)]
    pub fn process<F>(&mut self, input: f64, mut process_func: F) -> f64
    where
        F: FnMut(f64) -> f64,
    {
        // Upsample: one input sample -> two internal-rate samples.
        // up_even = out[2n] (earlier), up_odd = out[2n+1] (later).
        let (up_even, up_odd) = self.up_filter.process(input);

        // Apply pre-emphasis if enabled
        let up_even = if self.use_emphasis {
            self.pre_emphasis.process(up_even)
        } else {
            up_even
        };
        let up_odd = if self.use_emphasis {
            self.pre_emphasis.process(up_odd)
        } else {
            up_odd
        };

        // Process both samples through the nonlinear function
        let proc_even = process_func(up_even);
        let proc_odd = process_func(up_odd);

        // Apply de-emphasis if enabled
        let proc_even = if self.use_emphasis {
            self.de_emphasis.process(proc_even)
        } else {
            proc_even
        };
        let proc_odd = if self.use_emphasis {
            self.de_emphasis.process(proc_odd)
        } else {
            proc_odd
        };

        // Downsample: single polyphase decimator step per output sample.
        self.down_filter.decimate(proc_even, proc_odd)
    }
}

/// 4x oversampler (cascades two 2x stages).
///
/// Stage assignment follows the hiir cascade rule: the STEEP filter guards
/// the base-Nyquist boundary (outer stage, 1x<->2x), the cheap wide-band
/// filter runs at the inner 2x<->4x boundary where only a wide transition
/// band is required.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Oversampler4x {
    /// Outer stage (1x <-> 2x): steep, protects the audio band.
    stage_outer: Oversampler2x<7>,
    /// Inner stage (2x <-> 4x): wide transition band, cheap.
    stage_inner: Oversampler2x<3>,
}

impl Oversampler4x {
    /// Create a 4x oversampler.
    pub fn new(fs: f64) -> Self {
        Self {
            stage_outer: Oversampler2x::new(coefficients::HB_STEEP_7SECTION, false, fs),
            stage_inner: Oversampler2x::new(coefficients::HB_WIDE_3SECTION, false, fs * 2.0),
        }
    }

    /// Reset all filters.
    pub fn reset(&mut self) {
        self.stage_outer.reset();
        self.stage_inner.reset();
    }

    /// Process a single sample.
    #[inline(always)]
    pub fn process<F>(&mut self, input: f64, mut process_func: F) -> f64
    where
        F: FnMut(f64) -> f64,
    {
        self.stage_outer
            .process(input, |x| self.stage_inner.process(x, &mut process_func))
    }
}

/// Generic oversampler supporting 1x, 2x, and 4x.
///
/// Note: This enum is intentionally `Copy` and stores filter states inline.
/// The size difference between variants is accepted for performance.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum Oversampler {
    Bypass,
    Double(Oversampler2xStandard),
    Quad(Oversampler4x),
}

impl Oversampler {
    /// Create an oversampler for the given factor.
    ///
    /// # Errors
    /// Returns an error string if the factor is not 1, 2, or 4.
    pub fn new(factor: usize, fs: f64) -> Result<Self, String> {
        match factor {
            1 => Ok(Self::Bypass),
            2 => Ok(Self::Double(Oversampler2x::new(
                coefficients::HB_STEEP_7SECTION,
                false,
                fs,
            ))),
            4 => Ok(Self::Quad(Oversampler4x::new(fs))),
            _ => Err(format!(
                "Unsupported oversampling factor: {} (use 1, 2, or 4)",
                factor
            )),
        }
    }

    /// Reset filter states.
    pub fn reset(&mut self) {
        match self {
            Self::Bypass => {}
            Self::Double(o) => o.reset(),
            Self::Quad(o) => o.reset(),
        }
    }

    /// Process a sample.
    #[inline(always)]
    pub fn process<F>(&mut self, input: f64, mut process_func: F) -> f64
    where
        F: FnMut(f64) -> f64,
    {
        match self {
            Self::Bypass => process_func(input),
            Self::Double(o) => o.process(input, process_func),
            Self::Quad(o) => o.process(input, process_func),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_allpass_section() {
        let mut ap = AllpassSection::new(0.5);
        let output = ap.process(1.0);
        assert!(output.is_finite());
    }

    #[test]
    fn test_half_band() {
        let mut hb = HalfBandFilter::new(coefficients::HB_STEEP_7SECTION);
        let (even, odd) = hb.process(1.0);
        assert!(even.is_finite());
        assert!(odd.is_finite());
        let y = hb.decimate(1.0, 1.0);
        assert!(y.is_finite());
    }

    #[test]
    fn test_oversampler_2x_basic() {
        let mut os = Oversampler2x::new(coefficients::HB_STEEP_7SECTION, false, 44100.0);

        // Process a DC input - identity processing
        let y = os.process(1.0, |x| x);

        // Output should be finite and reasonably close to input after settling
        assert!(y.is_finite());
        assert!(y >= 0.0, "Output should be non-negative for positive input");
    }

    #[test]
    fn test_oversampler_2x_many_samples() {
        let mut os = Oversampler2x::new(coefficients::HB_STEEP_7SECTION, false, 44100.0);

        // Process many DC samples to allow filter to settle
        let mut y = 0.0;
        for _ in 0..200 {
            y = os.process(1.0, |x| x);
        }

        // After settling, output should be close to input
        assert!(
            (y - 1.0).abs() < 0.1,
            "Output {} should be close to 1.0 after settling",
            y
        );
    }

    #[test]
    fn test_oversampler_nonlinear() {
        let mut os = Oversampler2x::new(coefficients::HB_STEEP_7SECTION, false, 44100.0);

        // Process many samples to settle
        let mut y = 0.0;
        for _ in 0..200 {
            y = os.process(2.0, |x| x.clamp(-1.0, 1.0));
        }

        // Should be clamped near 1.0
        assert!(y > 0.8, "Clipped output should be near 1.0, got {}", y);
        assert!(y <= 1.1, "Output should not exceed clamp limit much");
    }

    #[test]
    fn test_oversampler_4x() {
        let mut os = Oversampler4x::new(44100.0);

        let mut y = 0.0;
        for _ in 0..400 {
            y = os.process(0.5, |x| x * x);
        }

        assert!(y.is_finite());
        assert!(y > 0.0);
    }

    // ====================================================================
    // Measurement tests. These would have caught the 2026-07 bug pair:
    // (A) decimator branches clocked twice per output sample (no stopband),
    // (B) invalid coefficient tables (-16/-20 dB where -60/-80 was claimed).
    // ====================================================================

    /// Single-bin DFT amplitude (rectangular window, exact-bin tones).
    fn bin_amplitude(signal: &[f64], k: usize) -> f64 {
        let n = signal.len();
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &s) in signal.iter().enumerate() {
            let w = 2.0 * PI * (k as f64) * (i as f64) / (n as f64);
            re += s * w.cos();
            im -= s * w.sin();
        }
        2.0 * (re * re + im * im).sqrt() / n as f64
    }

    const N_FFT: usize = 4096;
    const SETTLE: usize = 4096;

    /// Drive the DOWNSAMPLING path with an internal-rate tone injected via
    /// the process closure, collect output-rate samples.
    fn run_decimation_tone_2x<const N: usize>(
        coeffs: [f64; N],
        cycles_per_internal_sample: f64,
    ) -> Vec<f64> {
        let mut os = Oversampler2x::new(coeffs, false, 44100.0);
        let mut i_int: u64 = 0;
        let mut out = Vec::with_capacity(SETTLE + N_FFT);
        for _ in 0..(SETTLE + N_FFT) {
            let y = os.process(0.0, |_| {
                let ph = 2.0 * PI * cycles_per_internal_sample * i_int as f64;
                i_int += 1;
                ph.sin()
            });
            out.push(y);
        }
        out.split_off(SETTLE)
    }

    fn run_decimation_tone_4x(cycles_per_internal_sample: f64) -> Vec<f64> {
        let mut os = Oversampler4x::new(44100.0);
        let mut i_int: u64 = 0;
        let mut out = Vec::with_capacity(SETTLE + N_FFT);
        for _ in 0..(SETTLE + N_FFT) {
            let y = os.process(0.0, |_| {
                let ph = 2.0 * PI * cycles_per_internal_sample * i_int as f64;
                i_int += 1;
                ph.sin()
            });
            out.push(y);
        }
        out.split_off(SETTLE)
    }

    /// (a) 2x decimator alias rejection: internal tone at 0.9*pi
    /// (0.45 cycles/internal sample) folds to bin k_a. Design rejection for
    /// HB_STEEP_7SECTION is -86.9 dB; assert < -75 dB (margin for transient
    /// leakage). The pre-fix code measured about -1 dB here.
    #[test]
    fn decimator_2x_alias_rejection() {
        let k_a = 410; // folded alias bin, ~0.1 * fs_out
        let nu_a = k_a as f64 / N_FFT as f64;
        let u = (1.0 - nu_a) / 2.0; // internal-rate tone, ~0.45 = 0.9*pi
        let out = run_decimation_tone_2x(coefficients::HB_STEEP_7SECTION, u);
        let level = bin_amplitude(&out, k_a);
        let db = 20.0 * level.log10();
        eprintln!("measured 2x decimator alias rejection: {:.2} dB", db);
        assert!(
            db < -75.0,
            "2x decimator alias rejection {:.1} dB, expected < -75 dB",
            db
        );
    }

    /// (a') Same measurement for the fast/quality tables so every shipped
    /// set is held to its design spec (-62.1 / -111.7 dB).
    #[test]
    fn decimator_2x_alias_rejection_fast_and_quality() {
        let k_a = 410;
        let nu_a = k_a as f64 / N_FFT as f64;
        let u = (1.0 - nu_a) / 2.0;

        let out = run_decimation_tone_2x(coefficients::HB_STEEP_5SECTION, u);
        let db = 20.0 * bin_amplitude(&out, k_a).log10();
        eprintln!("measured 5-section alias rejection: {:.2} dB", db);
        assert!(db < -55.0, "5-section alias rejection {:.1} dB", db);

        let out = run_decimation_tone_2x(coefficients::HB_STEEP_9SECTION, u);
        let db = 20.0 * bin_amplitude(&out, k_a).log10();
        eprintln!("measured 9-section alias rejection: {:.2} dB", db);
        assert!(db < -95.0, "9-section alias rejection {:.1} dB", db);
    }

    /// (b) 2x decimator passband flatness: internal tone at 0.25*pi
    /// (0.125 cycles/internal sample -> 0.25 * fs_out) must come through
    /// within +/-0.1 dB. The pre-fix code drooped ~0.4-0.5 dB per stage.
    #[test]
    fn decimator_2x_passband_flatness() {
        let k = N_FFT / 4; // 0.25 * fs_out
        let u = 0.125; // cycles per internal sample = 0.25*pi rad
        let out = run_decimation_tone_2x(coefficients::HB_STEEP_7SECTION, u);
        let db = 20.0 * bin_amplitude(&out, k).log10();
        eprintln!("measured 2x decimator passband level: {:.5} dB", db);
        assert!(
            db.abs() < 0.1,
            "2x decimator passband level {:.4} dB, expected within +/-0.1 dB",
            db
        );
    }

    /// (c) 2x round trip up -> identity -> down at 0.45 * base Nyquist
    /// (0.225 cycles/output sample). The up/down pair composes to the pure
    /// allpass A0(z^2)*A1(z^2), so magnitude must be flat within +/-0.2 dB.
    #[test]
    fn round_trip_2x_droop() {
        let k = 922; // ~0.2251 cycles/sample ~ 0.45 * Nyquist
        let nu = k as f64 / N_FFT as f64;
        let mut os = Oversampler2x::new(coefficients::HB_STEEP_7SECTION, false, 44100.0);
        let mut out = Vec::with_capacity(SETTLE + N_FFT);
        for i in 0..(SETTLE + N_FFT) {
            let x = (2.0 * PI * nu * i as f64).sin();
            out.push(os.process(x, |v| v));
        }
        let tail = out.split_off(SETTLE);
        let db = 20.0 * bin_amplitude(&tail, k).log10();
        eprintln!("measured 2x round-trip level: {:.5} dB", db);
        assert!(
            db.abs() < 0.2,
            "2x round-trip level {:.4} dB at 0.45*Nyquist, expected within +/-0.2 dB",
            db
        );
    }

    /// (c') 4x round trip droop at 0.45 * base Nyquist, +/-0.2 dB.
    #[test]
    fn round_trip_4x_droop() {
        let k = 922;
        let nu = k as f64 / N_FFT as f64;
        let mut os = Oversampler4x::new(44100.0);
        let mut out = Vec::with_capacity(SETTLE + N_FFT);
        for i in 0..(SETTLE + N_FFT) {
            let x = (2.0 * PI * nu * i as f64).sin();
            out.push(os.process(x, |v| v));
        }
        let tail = out.split_off(SETTLE);
        let db = 20.0 * bin_amplitude(&tail, k).log10();
        eprintln!("measured 4x round-trip level: {:.5} dB", db);
        assert!(
            db.abs() < 0.2,
            "4x round-trip level {:.4} dB at 0.45*Nyquist, expected within +/-0.2 dB",
            db
        );
    }

    /// (a'') 4x alias rejection, inner-stage stopband: tone at 0.45
    /// cycles/4x-sample folds through both decimators to bin k_a with the
    /// inner stage's full -95 dB design rejection (outer stage passband).
    #[test]
    fn decimator_4x_alias_rejection_inner_band() {
        let k_a = 820; // ~0.2 * fs_out after double fold
        let nu_out = k_a as f64 / N_FFT as f64;
        let nu_2x = (1.0 - nu_out / 2.0) / 1.0; // apparent at 2x before outer? derive below
                                                // Derivation: output bin nu_out came through outer WITHOUT folding
                                                // (nu_out/2 < 0.25 at 2x rate), from inner fold of u:
                                                //   2*u mod 1 = 1 - nu_out/2  =>  u = (1 - nu_out/2)/2
        let _ = nu_2x;
        let u = (1.0 - nu_out / 2.0) / 2.0; // ~0.44995 cycles/4x-sample
        let out = run_decimation_tone_4x(u);
        let db = 20.0 * bin_amplitude(&out, k_a).log10();
        eprintln!("measured 4x inner-band alias rejection: {:.2} dB", db);
        assert!(
            db < -80.0,
            "4x inner-band alias rejection {:.1} dB, expected < -80 dB",
            db
        );
    }

    /// (a''') 4x alias rejection, outer-stage stopband: tone at 0.175
    /// cycles/4x-sample (0.7 * fs_out) passes the inner stage (transition
    /// band) and must be killed by the steep outer stage (-86.9 dB design).
    #[test]
    fn decimator_4x_alias_rejection_outer_band() {
        let k_a = 1229; // ~0.3 * fs_out
        let nu_out = k_a as f64 / N_FFT as f64;
        // Outer folds nu_2x in (0.25, 0.5): nu_out = 1 - 2*nu_2x.
        let nu_2x = (1.0 - nu_out) / 2.0; // ~0.34998 cycles/2x-sample
        let u = nu_2x / 2.0; // inner passes unfolded: ~0.17499 cycles/4x-sample
        let out = run_decimation_tone_4x(u);
        let db = 20.0 * bin_amplitude(&out, k_a).log10();
        eprintln!("measured 4x outer-band alias rejection: {:.2} dB", db);
        assert!(
            db < -70.0,
            "4x outer-band alias rejection {:.1} dB, expected < -70 dB",
            db
        );
    }
}
