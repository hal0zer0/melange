//! The continuous stimulus a sequence of samples implies, for the reference.
//!
//! melange's input is a sequence of samples. The reference must be driven by
//! the continuous signal those are samples of. For an analytic stimulus that
//! is the analytic source (validate declares it); for anything else it is the
//! band-limited signal through the samples. A PWL at the sample rate is linear
//! interpolation instead: it carries sinc² images around every multiple of
//! the sample rate, which a deck whose gain rises toward fs amplifies and the
//! reference's output sampling folds back onto the passband (a transformer
//! input driven through 1 MOhm read 1 % low that way).
//!
//! `band_limited` evaluates the Kaiser-windowed sinc interpolation of the
//! samples at `factor` points per sample, for a PWL at that rate: its own
//! images then sit around `factor·fs`, far above any deck's passband.

use crate::alignment::{bessel_i0, sinc};

/// Kaiser window shape and half-width (samples) of the interpolation kernel:
/// within 1e-7 of a sine's value between its samples up to 20 kHz at 48 kHz
/// (measured; 32 samples at beta 9 was 1e-5).
const BETA: f64 = 14.0;
const HALF: isize = 128;

/// The samples' band-limited interpolation at `factor` points per sample
/// (length `x.len() * factor`), samples outside `x` taken as zero.
pub(crate) fn band_limited(x: &[f64], factor: usize) -> Vec<f64> {
    let i0_beta = bessel_i0(BETA);
    let h = HALF as f64;
    let n = x.len() as isize;
    let mut out = Vec::with_capacity(x.len() * factor);
    // Taps per phase, shared across the signal.
    let phases: Vec<Vec<f64>> = (0..factor)
        .map(|j| {
            let frac = j as f64 / factor as f64;
            let taps: Vec<f64> = (-HALF + 1..=HALF)
                .map(|k| {
                    let u = frac - k as f64;
                    let r = u / h;
                    let w = if r.abs() >= 1.0 {
                        0.0
                    } else {
                        bessel_i0(BETA * (1.0 - r * r).sqrt()) / i0_beta
                    };
                    sinc(u) * w
                })
                .collect();
            taps
        })
        .collect();
    for i in 0..n {
        for taps in &phases {
            let mut y = 0.0;
            for (t, k) in taps.iter().zip(-HALF + 1..=HALF) {
                let idx = i + k;
                if (0..n).contains(&idx) {
                    y += t * x[idx as usize];
                }
            }
            out.push(y);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A sine well inside the band comes back as itself between the samples.
    #[test]
    fn a_sine_is_reconstructed_between_its_samples() {
        let fs = 48000.0;
        let f = 1000.0;
        let x: Vec<f64> = (0..9600)
            .map(|k| (2.0 * std::f64::consts::PI * f * k as f64 / fs).sin())
            .collect();
        let y = band_limited(&x, 16);
        // Away from the edges (the kernel's half-width).
        for (m, &v) in y.iter().enumerate().take(y.len() - 256 * 16).skip(256 * 16) {
            let t = m as f64 / (16.0 * fs);
            let want = (2.0 * std::f64::consts::PI * f * t).sin();
            assert!((v - want).abs() < 1e-7, "m {m}: {v} vs {want}");
        }
        // Exact at the samples.
        for k in 256..(x.len() - 256) {
            assert!((y[16 * k] - x[k]).abs() < 1e-12);
        }
    }
}
