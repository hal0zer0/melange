//! Trap-rule stability analysis on the propagation operator `S · A_neg`.
//!
//! Used by the DK and nodal auto-BE gates to decide whether the trap rule
//! will produce a usable result for a given circuit.
//!
//! Power iteration on `S · A_neg` returns `max|eigenvalue|` (the spectral
//! radius). For trap stability, ρ > 1 → unstable (mode grows). But ρ
//! exactly at 1 also matters: trap has gain magnitude 1 at z = -1
//! (Nyquist), so any eigenvalue near -1 is a marginal mode that the trap
//! rule cannot damp. On high-gain cap-coupled cascades, sub-µV f64
//! round-off in the per-sample matrix-vector multiplications seeds
//! Nyquist content that is amplified by the cascade gain into mV-level
//! limit cycles at the output.
//!
//! We discriminate "ρ near 1, eigenvalue near +1" (slow LF mode, trap
//! handles fine — bilinear preserves these poles exactly) from "ρ near 1,
//! eigenvalue near -1" (Nyquist-marginal trap mode, requires BE).
//!
//! After power iteration converges to the dominant eigenvector x, one
//! more application of S·A_neg gives y ≈ λ·x. The sign of `<x, y>`
//! recovers sign(λ) when λ is real. Complex conjugate pairs near the
//! unit circle are rare in audio circuits and not the common failure
//! mode this gate is designed to catch.

/// Result of trap-rule stability analysis on `S · A_neg`.
#[derive(Debug, Clone, Copy)]
pub struct TrapStability {
    /// Magnitude of the dominant eigenvalue of `S · A_neg`.
    pub rho: f64,
    /// Sign of `<x, (S·A_neg)·x>` at the converged power-iteration
    /// eigenvector. Approximately `sign(λ_dom)` when the dominant
    /// eigenvalue is real:
    /// - `> 0` → eigenvalue near `+1` (slow LF mode, trap is fine)
    /// - `< 0` → eigenvalue near `-1` (Nyquist-marginal trap mode that
    ///   trap cannot damp; auto-promote to BE on high-gain cap-coupled
    ///   cascades to avoid mV-level fs/2 limit cycles at the output)
    /// - `0`  → dominant magnitude is too small to classify (degenerate
    ///   or null case)
    pub dominant_sign: f64,
    /// `max |S[i][j]|` — magnitude of the largest resolvent entry. Used as a
    /// gain proxy for the Nyquist-marginal auto-BE gate: a marginal `z ≈ -1`
    /// mode only sustains an *audible* fs/2 limit cycle on high-gain
    /// topologies (large resolvent → high-impedance cap-coupled cascades like
    /// 3× triode stages, `max|S| ~ 5e5`). Low-gain marginal circuits
    /// (e.g. a 2-stage BJT preamp, `max|S| ~ 3e4`) keep their µV limit cycle
    /// sub-audible under trap, so they must NOT be promoted to BE. See
    /// [`trap_needs_be`].
    pub max_abs_s: f64,
}

/// Power-iteration estimate of the trap propagation operator's stability.
///
/// `s` and `a_neg` are flat row-major `n × n` matrices (`s[i*n + j]`).
/// 20 iterations matches the existing helpers in `routing.rs` and
/// `ir.rs`; convergence on real audio circuits' dominant eigenvalue is
/// usually within 5-10 iterations.
pub fn analyze_trap_stability(s: &[f64], a_neg: &[f64], n: usize) -> TrapStability {
    if n == 0 || s.is_empty() || a_neg.is_empty() {
        return TrapStability {
            rho: 0.0,
            dominant_sign: 0.0,
            max_abs_s: 0.0,
        };
    }

    let max_abs_s = s.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
    let mut x = vec![1.0 / (n as f64).sqrt(); n];
    let mut rho = 0.0_f64;

    for _ in 0..20 {
        let y = apply_s_a_neg(s, a_neg, n, &x);
        let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm < 1e-30 {
            return TrapStability {
                rho,
                dominant_sign: 0.0,
                max_abs_s,
            };
        }
        let x_norm: f64 = x.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-30);
        rho = norm / x_norm;
        for i in 0..n {
            x[i] = y[i] / norm;
        }
    }

    // After power iteration converges, `(S·A_neg)·x` is approximately
    // λ_dom · x. <x, y> ≈ λ_dom · |x|² recovers the sign of λ_dom when
    // it's real.
    let y_post = apply_s_a_neg(s, a_neg, n, &x);
    let dot: f64 = x.iter().zip(y_post.iter()).map(|(a, b)| a * b).sum();
    let dominant_sign = if dot.abs() < 1e-30 { 0.0 } else { dot.signum() };

    TrapStability {
        rho,
        dominant_sign,
        max_abs_s,
    }
}

/// Same as [`analyze_trap_stability`] but with the contribution of a
/// specific input node deflated, so the discriminator does not pick up
/// the spurious "fake" eigenvalue at z ≈ -1 introduced by Thevenin input
/// stamping.
///
/// Input-node Thevenin: stamping `G_in = 1/R_in` into `G[in][in]` and
/// adding the source current to RHS makes the input-row of `S·A_neg`
/// have a near-`-1` diagonal (because `S[in][in] ≈ 1/A[in][in] ≈ 1/G_in`
/// and `A_neg[in][in] ≈ -G_in`). That eigenvalue corresponds to the
/// "input is whatever you give it" virtual mode, not any physical
/// circuit dynamics, so it must not gate auto-BE. Without deflation the
/// discriminator false-fires on every nonlinear circuit with a Thevenin
/// input (including a single-stage CE preamp).
///
/// Deflation strategy: at each power-iteration step, project `x` to be
/// orthogonal to the unit basis vector at the input node. This removes
/// the input-direction component without affecting other modes.
pub fn analyze_trap_stability_deflated(
    s: &[f64],
    a_neg: &[f64],
    n: usize,
    input_node: usize,
) -> TrapStability {
    if n == 0 || s.is_empty() || a_neg.is_empty() || input_node >= n {
        return analyze_trap_stability(s, a_neg, n);
    }

    let max_abs_s = s.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
    let mut x = vec![1.0 / (n as f64).sqrt(); n];
    x[input_node] = 0.0;
    let init_norm: f64 = x.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-30);
    for v in &mut x {
        *v /= init_norm;
    }
    let mut rho = 0.0_f64;

    for _ in 0..20 {
        let mut y = apply_s_a_neg(s, a_neg, n, &x);
        // Deflate: project out the input-direction component.
        y[input_node] = 0.0;
        let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm < 1e-30 {
            return TrapStability {
                rho,
                dominant_sign: 0.0,
                max_abs_s,
            };
        }
        let x_norm: f64 = x.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-30);
        rho = norm / x_norm;
        for i in 0..n {
            x[i] = y[i] / norm;
        }
    }

    let mut y_post = apply_s_a_neg(s, a_neg, n, &x);
    y_post[input_node] = 0.0;
    let dot: f64 = x.iter().zip(y_post.iter()).map(|(a, b)| a * b).sum();
    let dominant_sign = if dot.abs() < 1e-30 { 0.0 } else { dot.signum() };

    TrapStability {
        rho,
        dominant_sign,
        max_abs_s,
    }
}

fn apply_s_a_neg(s: &[f64], a_neg: &[f64], n: usize, x: &[f64]) -> Vec<f64> {
    let mut ax = vec![0.0; n];
    for i in 0..n {
        for j in 0..n {
            ax[i] += a_neg[i * n + j] * x[j];
        }
    }
    let mut y = vec![0.0; n];
    for i in 0..n {
        for j in 0..n {
            y[i] += s[i * n + j] * ax[j];
        }
    }
    y
}

/// Decide whether the trap rule's spectral profile requires backward
/// Euler promotion.
///
/// Two regimes:
/// - `ρ > 1.002`: classic trap instability (mode grows unboundedly).
///   Always promote. Threshold matches the historical value chosen by
///   the 2026-04-22 passive-LC fix: strict `> 1.0` false-fires on
///   trivial passive networks where power iteration converges to ≈ 1.0
///   plus float noise.
/// - `ρ > 0.999` AND `dominant_sign < 0` AND `max|S| > 1e5`:
///   Nyquist-marginal trap mode on a *high-gain* topology. The dominant
///   eigenvalue is real-negative near `-1`; trap has gain magnitude
///   exactly 1 at `z = -1`, so any seed from f64 round-off in the
///   per-sample matrix-vector multiplications persists indefinitely.
///   This is only *audible* when the circuit's gain amplifies that seed
///   to a meaningful output level — catastrophic on high-gain cap-coupled
///   cascades (noyce-cascaded-triodes: 3× 12AX7, ρ = 0.9999,
///   `max|S| ≈ 5e5`, fs/2 limit cycle of 28 mV). The `max|S| > 1e5` gate
///   (resolvent magnitude as a gain proxy) keeps *low-gain* marginal
///   circuits on trap: a 2-stage BJT preamp (wurli-preamp, ρ ≈ 1.0000,
///   `max|S| ≈ 3e4`) has the same z ≈ -1 mode but its limit cycle stays
///   at the µV (sub-audible) level, and promoting it to BE would instead
///   introduce an audible low-frequency pump under per-sample `.runtime R`
///   modulation. The 0.999 threshold excludes positive-eigenvalue modes
///   (slow LF resonance — bilinear preserves those exactly). Genuinely
///   unstable circuits (`ρ > 1.002`) are caught by the first clause
///   regardless of gain.
pub fn trap_needs_be(stability: TrapStability) -> bool {
    stability.rho > 1.002
        || (stability.rho > 0.999
            && stability.dominant_sign < 0.0
            && stability.max_abs_s > 1.0e5)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat(rows: &[&[f64]]) -> (Vec<f64>, usize) {
        let n = rows.len();
        let mut out = vec![0.0; n * n];
        for (i, row) in rows.iter().enumerate() {
            assert_eq!(row.len(), n);
            for (j, &v) in row.iter().enumerate() {
                out[i * n + j] = v;
            }
        }
        (out, n)
    }

    #[test]
    fn analyze_identity_returns_unit_radius_positive() {
        // S·A_neg = I → only eigenvalue is 1 (positive).
        let (m, n) = flat(&[&[1.0, 0.0], &[0.0, 1.0]]);
        let id = m.clone();
        // Treat S = I, A_neg = I → S·A_neg = I.
        let r = analyze_trap_stability(&m, &id, n);
        assert!((r.rho - 1.0).abs() < 1e-6, "rho={}", r.rho);
        assert!(r.dominant_sign > 0.0, "expected positive sign, got {}", r.dominant_sign);
    }

    #[test]
    fn analyze_negative_identity_returns_unit_radius_negative() {
        // S·A_neg = -I → dominant eigenvalue is -1. Power iteration converges,
        // and the post-iteration <x, (S·A_neg)·x> = -|x|² has negative sign.
        let (s, n) = flat(&[&[-1.0, 0.0], &[0.0, -1.0]]);
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let r = analyze_trap_stability(&s, &id, n);
        assert!((r.rho - 1.0).abs() < 1e-6, "rho={}", r.rho);
        assert!(r.dominant_sign < 0.0, "expected negative sign, got {}", r.dominant_sign);
        // max|S| = 1 (low gain) → marginal Nyquist mode is sub-audible → must
        // NOT trip the gain-gated auto-BE clause.
        assert!(
            !trap_needs_be(r),
            "low-gain (max|S|=1) Nyquist mode must NOT trip auto-BE"
        );
    }

    #[test]
    fn nyquist_marginal_high_gain_trips_be() {
        // ρ ≈ 1, dominant eigenvalue near -1, AND a large resolvent entry
        // (max|S| = 3e5 > 1e5). Models noyce-cascaded-triodes: marginal
        // Nyquist mode on a high-gain cascade → audible fs/2 limit cycle →
        // must promote to BE. Upper-triangular so eigenvalues are -1, -0.5.
        let (s, n) = flat(&[&[-1.0, 3.0e5], &[0.0, -0.5]]);
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let r = analyze_trap_stability(&s, &id, n);
        assert!(r.dominant_sign < 0.0, "expected negative sign, got {}", r.dominant_sign);
        assert!(r.max_abs_s > 1.0e5, "max_abs_s={}", r.max_abs_s);
        assert!(
            trap_needs_be(r),
            "high-gain (max|S|=3e5) Nyquist-marginal mode must trip auto-BE"
        );
    }

    #[test]
    fn nyquist_marginal_low_gain_stays_trap() {
        // Same marginal Nyquist mode but low resolvent (max|S| = 1e3 < 1e5).
        // Models wurli-preamp: µV-level limit cycle, sub-audible → keep trap
        // (promoting to BE would add an audible .runtime-R pump).
        let (s, n) = flat(&[&[-1.0, 1.0e3], &[0.0, -0.5]]);
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let r = analyze_trap_stability(&s, &id, n);
        assert!(r.dominant_sign < 0.0, "expected negative sign, got {}", r.dominant_sign);
        assert!(
            !trap_needs_be(r),
            "low-gain (max|S|=1e3) Nyquist-marginal mode must NOT trip auto-BE"
        );
    }

    #[test]
    fn analyze_passive_lc_eigenvalue_near_plus_one_does_not_trip() {
        // S·A_neg ≈ diag(0.9999, 0.5) — slow LF mode at z ≈ +1.
        // Should classify as positive sign and NOT trip auto-BE.
        let (s, n) = flat(&[&[0.9999, 0.0], &[0.0, 0.5]]);
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let r = analyze_trap_stability(&s, &id, n);
        assert!(r.dominant_sign > 0.0, "passive LF mode should classify positive, got {}", r.dominant_sign);
        assert!(!trap_needs_be(r), "ρ near +1 (passive LF) must NOT trip auto-BE");
    }

    #[test]
    fn analyze_unstable_eigenvalue_above_threshold_trips_regardless_of_sign() {
        // ρ = 1.05 > 1.002 → trip regardless of sign.
        let (s, n) = flat(&[&[1.05, 0.0], &[0.0, 0.5]]);
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let r = analyze_trap_stability(&s, &id, n);
        assert!(r.rho > 1.002);
        assert!(trap_needs_be(r));
    }

    #[test]
    fn analyze_handles_empty_matrix() {
        let r = analyze_trap_stability(&[], &[], 0);
        assert_eq!(r.rho, 0.0);
        assert_eq!(r.dominant_sign, 0.0);
        assert!(!trap_needs_be(r));
    }
}
