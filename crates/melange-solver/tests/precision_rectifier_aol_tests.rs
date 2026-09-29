//! A precision rectifier keeps its op-amp's full open-loop gain.
//!
//! melange used to cap AOL at 1000 in the transient solve of any op-amp whose
//! non-inverting input sits on a DC rail with a diode from its output to its
//! inverting input (a precision rectifier or comparator), against LU
//! back-substitution contamination under a post-solve clamp. Under the charge
//! form and active-set pinning the uncapped solve converges on every sample,
//! and the cap moved the answer: on this biased half-wave rectifier it left a
//! 4.5 mV virtual-ground error (11.8 % of the signal at 0.1 V drive). The cap
//! is gone; only an author's `AOL_TRANSIENT_CAP` applies one.
//!
//! Reference: ngspice-42, the same deck with the op-amp as melange stamps it
//! (VCCS gm = AOL/ROUT into ROUT), a 1 Ω Thevenin drive, `reltol=1e-6`,
//! 1 µs steps, sampled at the 48 kHz instants over 0.1–0.2 s (recorded
//! 2026-09-29). melange agrees to 0.6 µV peak.

mod support;

const RECTIFIER: &str = "precision half-wave rectifier biased at 4.5 V
Vref ref 0 DC 4.5
Cin in a 1u
Rin a inv 10k
U1 ref inv oa OX
D1 inv oa D1N914
D2 oa out D1N914
Rf out inv 10k
Rl out ref 100k
.model OX OA(AOL=200000 ROUT=75)
.model D1N914 D(IS=2.52n N=1.752)
";

const SR: f64 = 48000.0;

/// (drive V, ngspice mean / max / min of v(out) over 0.1–0.2 s)
const NGSPICE: [(f64, [f64; 3]); 2] = [
    (0.1, [4.5317608, 4.5999125, 4.4999579]),
    (1.0, [4.8180249, 5.4995868, 4.4999581]),
];

const TOL_V: f64 = 1e-5;

#[test]
fn a_biased_precision_rectifier_matches_ngspice_at_full_aol() {
    let mut config = support::config_for_spice(RECTIFIER, SR);
    config.dc_block = false;
    let circuit = support::build_circuit_nodal(RECTIFIER, &config, "precision_rectifier_aol");
    for (amp, [mean_ref, max_ref, min_ref]) in NGSPICE {
        let out = support::run_sine(&circuit, 1000.0, amp, (0.2 * SR) as usize, SR);
        let tail = &out[(0.1 * SR) as usize..];
        let mean = tail.iter().sum::<f64>() / tail.len() as f64;
        let max = tail.iter().copied().fold(f64::MIN, f64::max);
        let min = tail.iter().copied().fold(f64::MAX, f64::min);
        eprintln!("{amp} V: mean {mean:.7} max {max:.7} min {min:.7}");
        for (what, got, want) in [
            ("mean", mean, mean_ref),
            ("max", max, max_ref),
            ("min", min, min_ref),
        ] {
            assert!(
                (got - want).abs() < TOL_V,
                "{amp} V drive: {what} v(out) {got:.7} V vs ngspice {want:.7} V \
                 (a capped AOL left a ~4.5 mV virtual-ground error here)"
            );
        }
    }
}
