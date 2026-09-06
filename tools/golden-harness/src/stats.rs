//! Per-render statistics: level metrics + band energies via an internal
//! radix-2 FFT (no external DSP deps; everything f64 and deterministic).

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Sentinel for "no energy at all" (log of zero).
pub const SILENT_DB: f64 = -400.0;

pub fn db(amplitude: f64) -> f64 {
    if amplitude > 0.0 {
        20.0 * amplitude.log10()
    } else {
        SILENT_DB
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Stats {
    pub channels: usize,
    pub frames: usize,
    pub sample_rate: f64,
    /// Peak/RMS/DC over all interleaved samples (all channels).
    pub peak_dbfs: f64,
    pub rms_dbfs: f64,
    pub dc_mean: f64,
    pub nan_count: u64,
    pub inf_count: u64,
    /// Band RMS levels (dBFS), computed on channel 0. Keys are prefixed
    /// for stable ordering: a_sub20 (<20 Hz), b_20_200, c_200_2k,
    /// d_2k_20k, e_20k_nyq (>20 kHz up to fs/2).
    pub bands_dbfs: BTreeMap<String, f64>,
    /// Solver diagnostic counters at end of render (BE fallbacks, NaN resets,
    /// line-search failures, ...). Empty for baselines captured before these
    /// were recorded, and for modules that declare none.
    ///
    /// Recorded because the audio renders cannot see the recovery ladders:
    /// only 7 of 41 corpus circuits enter any ladder, and several never fire on
    /// any deck, so a change to that code is invisible to a render diff.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub diagnostics: BTreeMap<String, f64>,
}

/// In-place iterative radix-2 Cooley-Tukey FFT. `re`/`im` length must be a
/// power of two.
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    assert!(n.is_power_of_two());
    // Bit-reversal permutation.
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * std::f64::consts::PI / len as f64;
        let (wr, wi) = (ang.cos(), ang.sin());
        let mut i = 0;
        while i < n {
            let (mut cur_r, mut cur_i) = (1.0f64, 0.0f64);
            for k in 0..len / 2 {
                let (ur, ui) = (re[i + k], im[i + k]);
                let (vr, vi) = (
                    re[i + k + len / 2] * cur_r - im[i + k + len / 2] * cur_i,
                    re[i + k + len / 2] * cur_i + im[i + k + len / 2] * cur_r,
                );
                re[i + k] = ur + vr;
                im[i + k] = ui + vi;
                re[i + k + len / 2] = ur - vr;
                im[i + k + len / 2] = ui - vi;
                let nr = cur_r * wr - cur_i * wi;
                cur_i = cur_r * wi + cur_i * wr;
                cur_r = nr;
            }
            i += len;
        }
        len <<= 1;
    }
}

const BAND_KEYS: [&str; 5] = ["a_sub20", "b_20_200", "c_200_2k", "d_2k_20k", "e_20k_nyq"];
const BAND_EDGES: [f64; 4] = [20.0, 200.0, 2000.0, 20000.0];

fn band_index(freq: f64) -> usize {
    BAND_EDGES.iter().position(|&e| freq < e).unwrap_or(4)
}

/// Band RMS (as amplitudes, then dB) of channel 0 via zero-padded FFT.
/// Parseval: sum_n x^2 = (1/Nfft) sum_k |X_k|^2, so
/// band_rms = sqrt(band_energy / (Nfft * Nsig)).
fn band_levels(ch0: &[f64], fs: f64) -> BTreeMap<String, f64> {
    let nsig = ch0.len().max(1);
    let nfft = nsig.next_power_of_two();
    let mut re = vec![0.0f64; nfft];
    let mut im = vec![0.0f64; nfft];
    for (i, &x) in ch0.iter().enumerate() {
        re[i] = if x.is_finite() { x } else { 0.0 };
    }
    fft(&mut re, &mut im);
    let mut acc = [0.0f64; 5];
    for k in 0..=nfft / 2 {
        let freq = k as f64 * fs / nfft as f64;
        let mag2 = re[k] * re[k] + im[k] * im[k];
        // One-sided: double all bins except DC and Nyquist.
        let w = if k == 0 || k == nfft / 2 { 1.0 } else { 2.0 };
        acc[band_index(freq)] += w * mag2;
    }
    let mut out = BTreeMap::new();
    for (i, key) in BAND_KEYS.iter().enumerate() {
        let rms = (acc[i] / (nfft as f64 * nsig as f64)).sqrt();
        out.insert(key.to_string(), db(rms));
    }
    out
}

pub fn compute(
    interleaved: &[f64],
    channels: usize,
    frames: usize,
    fs: f64,
    diagnostics: BTreeMap<String, f64>,
) -> Stats {
    let mut peak = 0.0f64;
    let mut sum_sq = 0.0f64;
    let mut sum = 0.0f64;
    let mut nan_count = 0u64;
    let mut inf_count = 0u64;
    let mut finite_n = 0u64;
    for &x in interleaved {
        if x.is_nan() {
            nan_count += 1;
            continue;
        }
        if x.is_infinite() {
            inf_count += 1;
            continue;
        }
        finite_n += 1;
        peak = peak.max(x.abs());
        sum_sq += x * x;
        sum += x;
    }
    let n = finite_n.max(1) as f64;
    let ch0: Vec<f64> = interleaved
        .iter()
        .step_by(channels.max(1))
        .copied()
        .collect();
    Stats {
        channels,
        frames,
        sample_rate: fs,
        peak_dbfs: db(peak),
        rms_dbfs: db((sum_sq / n).sqrt()),
        dc_mean: sum / n,
        nan_count,
        inf_count,
        bands_dbfs: band_levels(&ch0, fs),
        diagnostics,
    }
}
