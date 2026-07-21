//! Deterministic render programs.
//!
//! All programs are pure functions of (sample index, fs, level) — no RNG,
//! no time, no environment. Rendered inputs are formatted `{:.17e}` (exact
//! f64 round-trip) so the byte stream fed to the generated binary is
//! identical on every run.

#[derive(Debug, Clone, Copy)]
pub struct Program {
    pub name: &'static str,
    pub seconds: f64,
    /// This program drives the pot-sweep driver binary instead of the
    /// standard driver.
    pub pot_sweep: bool,
}

pub const SILENCE: Program = Program {
    name: "silence",
    seconds: 2.0,
    pot_sweep: false,
};
pub const SINE1K: Program = Program {
    name: "sine1k",
    seconds: 2.0,
    pot_sweep: false,
};
pub const SWEEP: Program = Program {
    name: "sweep",
    seconds: 4.0,
    pot_sweep: false,
};
pub const STEP: Program = Program {
    name: "step",
    seconds: 1.0,
    pot_sweep: false,
};
pub const POTSWEEP: Program = Program {
    name: "potsweep",
    seconds: 2.0,
    pot_sweep: true,
};

/// Pot positions update every this many samples in the potsweep program
/// (mirrors block-based host automation; also keeps the per-update matrix
/// rebuild cost sane).
pub const POT_UPDATE_INTERVAL: usize = 64;

pub fn programs(has_pots: bool) -> Vec<Program> {
    let mut v = vec![SILENCE, SINE1K, SWEEP, STEP];
    if has_pots {
        v.push(POTSWEEP);
    }
    v
}

pub fn frames(p: &Program, fs: f64) -> usize {
    (p.seconds * fs).round() as usize
}

/// Generate the input signal for a program. Deterministic.
pub fn gen_input(p: &Program, fs: f64, level: f64) -> Vec<f64> {
    let n = frames(p, fs);
    let mut out = Vec::with_capacity(n);
    match p.name {
        "silence" => out.resize(n, 0.0),
        "sine1k" | "potsweep" => {
            for i in 0..n {
                let t = i as f64 / fs;
                out.push(level * (2.0 * std::f64::consts::PI * 1000.0 * t).sin());
            }
        }
        "sweep" => {
            // Log sweep 20 Hz -> 20 kHz over the program duration.
            // phase(t) = 2*pi*f0*(T/ln(r)) * (r^(t/T) - 1), r = f1/f0.
            let f0 = 20.0;
            let f1 = 20000.0;
            let big_t = p.seconds;
            let r = f1 / f0;
            let k = 2.0 * std::f64::consts::PI * f0 * big_t / r.ln();
            for i in 0..n {
                let t = i as f64 / fs;
                let phase = k * (r.powf(t / big_t) - 1.0);
                out.push(level * phase.sin());
            }
        }
        "step" => {
            // 0 V for the first 0.1 s (captures rest state), then a level-V
            // step held to the end. Deliberately an abrupt transient test.
            let edge = (0.1 * fs).round() as usize;
            for i in 0..n {
                out.push(if i < edge { 0.0 } else { level });
            }
        }
        other => panic!("unknown program {other}"),
    }
    out
}
