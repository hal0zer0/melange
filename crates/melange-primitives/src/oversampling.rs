//! Polyphase IIR oversampling.
//!
//! Uses cascaded allpass filters for a computationally efficient
//! half-band filter structure. Popularized by O. Niemitalo and
//! widely used in audio DSP.

use crate::filters::OnePoleLpf;

/// Allpass filter section for polyphase filters.
///
/// First-order allpass: H(z) = (c + z^-1) / (1 + c*z^-1)
/// where c is the allpass coefficient.
#[derive(Debug, Clone, Copy, PartialEq)]
struct AllpassSection {
    c: f64,      // coefficient
    x1: f64,     // input delay
    y1: f64,     // output delay
}

impl AllpassSection {
    fn new(c: f64) -> Self {
        Self { c, x1: 0.0, y1: 0.0 }
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

/// Half-band filter coefficients for different quality levels.
pub mod coefficients {
    /// 2-section half-band (minimal CPU, ~60dB rejection).
    pub const HB_2SECTION: [f64; 2] = [0.07986642623635751, 0.5453536510716122];

    /// 3-section half-band (balanced, ~80dB rejection).
    pub const HB_3SECTION: [f64; 3] = [0.036681502163648017, 0.2746317593794541, 0.7856959333713522];

    /// 4-section half-band (high quality, ~100dB rejection).
    pub const HB_4SECTION: [f64; 4] = [
        0.019287696917501716,
        0.15053258922549724,
        0.4985894271657593,
        0.8756415228935117,
    ];

    /// 5-section half-band (excellent, ~120dB rejection).
    #[allow(clippy::excessive_precision)]
    pub const HB_5SECTION: [f64; 5] = [
        0.011266153671749254,
        0.08836290435912563,
        0.31190708239012636,
        0.6621633850686569,
        0.9304376538377054,
    ];
}

/// Half-band filter using allpass decomposition.
///
/// The half-band filter is implemented using two parallel allpass chains.
/// This is equivalent to a half-band filter but computationally efficient.
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

impl<const N_SECTIONS: usize> HalfBandFilter<N_SECTIONS> {
    /// Create a half-band filter with the given coefficients.
    pub fn new(coeffs: [f64; N_SECTIONS]) -> Self {
        let even = [AllpassSection::new(0.0); N_SECTIONS];
        let odd = [AllpassSection::new(0.0); N_SECTIONS];
        let mut filter = Self { even, odd };
        
        for (i, &c) in coeffs.iter().enumerate() {
            filter.even[i] = AllpassSection::new(c);
            filter.odd[i] = AllpassSection::new(c);
        }
        
        filter
    }

    /// Process a sample through the half-band filter.
    /// 
    /// Returns (even_output, odd_output) for polyphase processing.
    #[inline(always)]
    fn process(&mut self, input: f64) -> (f64, f64) {
        // Even and odd allpass outputs
        let mut even_out = input;
        let mut odd_out = input;
        
        for i in 0..N_SECTIONS {
            even_out = self.even[i].process(even_out);
            odd_out = self.odd[i].process(odd_out);
        }
        
        (even_out, odd_out)
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
/// The oversampling chain: upsample → filter → process → filter → downsample
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

pub type Oversampler2xFast = Oversampler2x<2>;
pub type Oversampler2xStandard = Oversampler2x<3>;
pub type Oversampler2xQuality = Oversampler2x<4>;

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
        // Upsampling by 2 with zero insertion and filtering
        // Input sample produces two output samples
        
        // First upsampled sample (at time 2n): uses even allpass output
        // Second upsampled sample (at time 2n+1): uses odd allpass output
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
        
        // Downsampling: filter both samples and combine
        // The half-band filter naturally reconstructs the proper output
        let (out_even, _) = self.down_filter.process(proc_even);
        let (_, out_odd) = self.down_filter.process(proc_odd);
        
        // Return the averaged output (compensate for gain)
        (out_even + out_odd) * 0.5
    }
}

/// 4x oversampler (cascades two 2x stages).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Oversampler4x {
    stage1: Oversampler2x<2>,  // 2-section for first stage
    stage2: Oversampler2x<4>,  // 4-section for final stage
}

impl Oversampler4x {
    /// Create a 4x oversampler.
    pub fn new(fs: f64) -> Self {
        Self {
            stage1: Oversampler2x::new(coefficients::HB_2SECTION, false, fs * 2.0),
            stage2: Oversampler2x::new(coefficients::HB_4SECTION, false, fs * 2.0),
        }
    }

    /// Reset all filters.
    pub fn reset(&mut self) {
        self.stage1.reset();
        self.stage2.reset();
    }

    /// Process a single sample.
    #[inline(always)]
    pub fn process<F>(&mut self, input: f64, mut process_func: F) -> f64
    where
        F: FnMut(f64) -> f64,
    {
        self.stage1.process(input, |x| self.stage2.process(x, &mut process_func))
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
            2 => Ok(Self::Double(Oversampler2x::new(coefficients::HB_3SECTION, false, fs))),
            4 => Ok(Self::Quad(Oversampler4x::new(fs))),
            _ => Err(format!("Unsupported oversampling factor: {} (use 1, 2, or 4)", factor)),
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

    #[test]
    fn test_allpass_section() {
        let mut ap = AllpassSection::new(0.5);
        let output = ap.process(1.0);
        assert!(output.is_finite());
    }

    #[test]
    fn test_half_band() {
        let mut hb = HalfBand3::new(coefficients::HB_3SECTION);
        let (even, odd) = hb.process(1.0);
        assert!(even.is_finite());
        assert!(odd.is_finite());
    }

    #[test]
    fn test_oversampler_2x_basic() {
        let mut os = Oversampler2x::new(coefficients::HB_3SECTION, false, 44100.0);
        
        // Process a DC input - identity processing
        let y = os.process(1.0, |x| x);
        
        // Output should be finite and reasonably close to input after settling
        assert!(y.is_finite());
        assert!(y > 0.0, "Output should be positive for positive input");
    }

    #[test]
    fn test_oversampler_2x_many_samples() {
        let mut os = Oversampler2x::new(coefficients::HB_3SECTION, false, 44100.0);
        
        // Process many DC samples to allow filter to settle
        let mut y = 0.0;
        for _ in 0..100 {
            y = os.process(1.0, |x| x);
        }
        
        // After settling, output should be close to input
        assert!((y - 1.0).abs() < 0.1, "Output {} should be close to 1.0 after settling", y);
    }

    #[test]
    fn test_oversampler_nonlinear() {
        let mut os = Oversampler2x::new(coefficients::HB_3SECTION, false, 44100.0);
        
        // Process many samples to settle
        let mut y = 0.0;
        for _ in 0..100 {
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
        for _ in 0..200 {
            y = os.process(0.5, |x| x * x);
        }
        
        assert!(y.is_finite());
        assert!(y > 0.0);
    }
}
