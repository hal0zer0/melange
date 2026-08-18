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

/// Trap-rule promotes to backward Euler above this spectral radius.
/// Strict `> 1.0` false-fires on trivial passive networks where power
/// iteration converges to ≈ 1.0 plus float noise (2026-04-22 passive-LC fix).
pub const TRAP_BE_PROMOTION_RHO: f64 = 1.002;

/// Lower-bound threshold for the sign-aware Nyquist-marginal gate.
/// Activates when ρ between this and `TRAP_BE_PROMOTION_RHO` AND the
/// dominant eigenvalue is real-negative AND the gain proxy `max|S|`
/// exceeds [`NYQUIST_GATE_MAX_ABS_S`]. Excludes positive-eigenvalue
/// modes (slow LF resonance — bilinear preserves those exactly).
pub const TRAP_BE_SIGN_FLIP_RHO: f64 = 0.999;

/// Gain proxy (resolvent magnitude) above which the Nyquist-marginal gate
/// fires. Keeps low-gain marginal circuits on trap (a 2-stage BJT preamp
/// at `max|S| ~ 3e4` stays at µV limit cycle; a 3× triode cascade at
/// `max|S| ~ 5e5` produces an audible mV limit cycle without BE).
pub const NYQUIST_GATE_MAX_ABS_S: f64 = 1.0e5;

/// Post-promotion sanity check on backward-Euler matrices. BE is L-stable
/// by construction, so `ρ` on the BE matrices must be ≤ 1. A violation
/// above this threshold (1.0 + 1e-6 to absorb numerical noise) signals a
/// matrix-builder bug, not a stability margin.
pub const BE_POST_PROMOTION_LIMIT: f64 = 1.0 + 1e-6;

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

/// Relative-change convergence tolerance for the power iteration's growth
/// (norm-ratio) estimate. Chosen so that with a subdominant-to-dominant
/// ratio of ~0.98 (the worst realistic case: promotion thresholds
/// 0.999/1.002 are only 0.3% apart) the stopping error stays well below
/// 1e-4 — the fixed 20-iteration scheme this replaces had ~0.5-1% bias in
/// that regime, larger than the threshold gap itself.
const POWER_ITER_REL_TOL: f64 = 1e-6;

/// Hard cap on power iterations. Cost is O(cap · n²) at compile time,
/// negligible. Sized so a subdominant/dominant ratio of 0.977 (the
/// λ = {1.003, 0.98} worst case around the 0.999/1.002 thresholds) reaches
/// the relative-change stop (~150 iterations with the stability window)
/// instead of being truncated with ~1e-4 residual bias.
const POWER_ITER_MAX: usize = 500;

/// The relative change must stay below [`POWER_ITER_REL_TOL`] for this many
/// CONSECUTIVE iterations before the loop stops. Guards against spurious
/// stops on transient plateaus of strongly non-normal operators (the
/// noyce-class triode cascade has large off-diagonal S entries whose
/// transient growth phase can hold the estimate briefly steady before the
/// asymptotic regime).
const POWER_ITER_STABLE_ITERS: usize = 5;

/// Shared power-iteration core: iterate `x ← (S·A_neg)·x` (optionally
/// deflating one basis direction) until the growth estimate
/// `ρ_k = |y| / |x|` stabilizes to [`POWER_ITER_REL_TOL`] relative change
/// for [`POWER_ITER_STABLE_ITERS`] consecutive iterations, capped at
/// [`POWER_ITER_MAX`].
///
/// Returns `(rho, dominant_sign)`:
/// - `rho` = converged norm ratio. For a well-separated real dominant
///   eigenvalue this converges to |λ_dom| at the same geometric rate as the
///   eigenvector, and — unlike a fixed iteration count — the stop criterion
///   guarantees the estimate has actually settled (fixed-20 carried
///   ~0.5-1% bias with close subdominant eigenvalues). For an eigenvalue
///   CLUSTER near the unit circle (cap-coupled cascades put one z ≈ -1
///   mode per coupling cap), the iterate settles into the near-invariant
///   cluster subspace: the norm ratio converges to the cluster's growth
///   rate — exactly the quantity the trap-stability gates need — even
///   though no single eigenvector is ever isolated. This is why ρ is NOT
///   estimated from the Rayleigh quotient: on the noyce-cascaded-triodes
///   operator the quotient needs tens of thousands of iterations to cross
///   the cluster (measured: still -0.87 after 2000 iterations while the
///   norm ratio locked at 0.9998 by iteration 100).
/// - `dominant_sign` = sign of `⟨x, (S·A_neg)·x⟩` at the stopped iterate
///   (the Rayleigh quotient's sign). Recovers sign(λ_dom) for a real
///   dominant eigenvalue, and empirically classifies the z ≈ -1 cluster
///   correctly (the quotient is negative throughout the iteration).
///
/// Known limits (documented, accepted): for a **complex conjugate**
/// dominant pair the iterate rotates in the invariant plane; the norm
/// ratio oscillates around |λ| with the rotation and the loop may run to
/// the cap, returning a point on that oscillation, while `dominant_sign`
/// reflects only the real part's orientation. Complex pairs near the unit
/// circle are rare in audio circuits and are not the failure mode the
/// auto-BE gates discriminate; a genuinely unstable complex pair still
/// shows sustained growth that trips the `rho > 1.002` clause.
fn power_iterate_rho_sign(
    s: &[f64],
    a_neg: &[f64],
    n: usize,
    x: &mut [f64],
    deflate: &[usize],
) -> (f64, f64) {
    let mut rho = 0.0_f64;
    let mut rho_prev = f64::NAN;
    let mut last_dot = 0.0_f64;
    let mut stable_iters = 0usize;

    for _ in 0..POWER_ITER_MAX {
        let mut y = apply_s_a_neg(s, a_neg, n, x);
        for &d in deflate {
            y[d] = 0.0;
        }
        let x_norm: f64 = x.iter().map(|v| v * v).sum::<f64>().sqrt();
        if x_norm < 1e-30 {
            break;
        }
        last_dot = x.iter().zip(y.iter()).map(|(a, b)| a * b).sum::<f64>() / (x_norm * x_norm);
        let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm < 1e-30 {
            // Operator annihilates the iterate — degenerate/null direction.
            return (rho, 0.0);
        }
        rho = norm / x_norm;
        for i in 0..n {
            x[i] = y[i] / norm;
        }
        // Converge on the estimate, not on an iteration count.
        if rho_prev.is_finite() && (rho - rho_prev).abs() <= POWER_ITER_REL_TOL * rho.max(1e-30) {
            stable_iters += 1;
            if stable_iters >= POWER_ITER_STABLE_ITERS {
                break;
            }
        } else {
            stable_iters = 0;
        }
        rho_prev = rho;
    }

    let dominant_sign = if last_dot.abs() < 1e-30 {
        0.0
    } else {
        last_dot.signum()
    };
    (rho, dominant_sign)
}

/// Power-iteration estimate of the trap propagation operator's stability.
///
/// `s` and `a_neg` are flat row-major `n × n` matrices (`s[i*n + j]`).
/// Iterates to a relative-change tolerance on the growth (norm-ratio)
/// estimate (see [`power_iterate_rho_sign`]) instead of a fixed iteration
/// count — the old fixed-20 scheme carried ~0.5-1% bias when a subdominant
/// eigenvalue sat close to the dominant one, while the promotion
/// thresholds (0.999 / 1.002) are only 0.3% apart.
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
    let (rho, dominant_sign) = power_iterate_rho_sign(s, a_neg, n, &mut x, &[]);

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
    input_nodes: &[usize],
) -> TrapStability {
    // Keep only in-range input nodes; if none, fall back to the undeflated
    // analyzer (matches the historical single-node `input_node >= n` guard).
    let deflate: Vec<usize> = input_nodes.iter().copied().filter(|&d| d < n).collect();
    if n == 0 || s.is_empty() || a_neg.is_empty() || deflate.is_empty() {
        return analyze_trap_stability(s, a_neg, n);
    }

    let max_abs_s = s.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
    let mut x = vec![1.0 / (n as f64).sqrt(); n];
    for &d in &deflate {
        x[d] = 0.0;
    }
    let init_norm: f64 = x.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-30);
    for v in &mut x {
        *v /= init_norm;
    }
    let (rho, dominant_sign) = power_iterate_rho_sign(s, a_neg, n, &mut x, &deflate);

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
    stability.rho > TRAP_BE_PROMOTION_RHO
        || (stability.rho > TRAP_BE_SIGN_FLIP_RHO
            && stability.dominant_sign < 0.0
            && stability.max_abs_s > NYQUIST_GATE_MAX_ABS_S)
}

/// Post-promotion sanity check on newly-built backward-Euler matrices,
/// shared by the DK and nodal BE builders.
///
/// L-stability guarantees `rho(S_be*A_neg_be) <= 1` ONLY for a circuit whose
/// linearization is itself continuum-stable (every mode has `Re(lambda) <=
/// 0`). A circuit that is genuinely unstable at its DC operating point (a
/// regenerative oscillator sitting on an unstable bias point by design) has
/// a real growing mode that no consistent integrator — BE included — can
/// make appear as `rho <= 1` without falsifying the circuit's own physics.
/// The accurate, converged, input-deflated analyzer's `dominant_sign` lets
/// us tell the two cases apart: `dominant_sign > 0` is the signature of a
/// real growing pole (expected, benign — logged as a warning), while
/// `dominant_sign < 0` has no known physical mechanism at this construction
/// and more likely indicates a genuine matrix-builder defect (logged as an
/// error).
///
/// Verified 2026-08-14 on a Ge regenerative-LC-oscillator repro
/// (`memory/dk_backward_euler_ignored_trap_unstable.md`): two independently
/// coded BE builders (DK's `build_dk_be_matrices_at_rate` and nodal's inline
/// build in `CircuitIR::from_mna`) agree on the deflated rho to 6
/// significant figures (1.347488, dominant_sign +1) on the identical
/// circuit, and the resulting transient (full-LU NR on the nodal path)
/// converges to a bounded, physical oscillation — conclusive evidence the
/// matrices are correct and the growth is real, not a stamping defect.
///
/// `label` identifies the caller in the log line (e.g. `"DK"`, `"Nodal"`).
pub fn log_be_post_promotion_check(
    label: &str,
    s: &[f64],
    a_neg: &[f64],
    n: usize,
    input_nodes: &[usize],
) {
    if n == 0 || s.is_empty() || a_neg.is_empty() {
        return;
    }
    let stability = analyze_trap_stability_deflated(s, a_neg, n, input_nodes);
    if stability.rho <= BE_POST_PROMOTION_LIMIT {
        return;
    }
    if stability.dominant_sign < 0.0 {
        log::error!(
            "{label}: BE matrices have spectral_radius(S_be*A_neg_be) = {:.4} \
             (dominant_sign {:+.0}) after promotion. BE is L-stable for a \
             continuum-stable circuit; a NEGATIVE dominant sign here has no known \
             physical explanation (unlike a real growing pole) and likely indicates \
             a BE matrix-builder defect — investigate.",
            stability.rho,
            stability.dominant_sign
        );
    } else {
        log::warn!(
            "{label}: BE matrices have spectral_radius(S_be*A_neg_be) = {:.4} \
             (dominant_sign {:+.0}) after promotion. This typically means the circuit \
             is genuinely unstable at its DC operating point (e.g. a regenerative \
             oscillator — a positive dominant sign is the expected signature of a real \
             growing pole, not a matrix defect). No integrator can make a real growing \
             pole read spectral_radius <= 1 without falsifying the circuit's physics; \
             backward Euler remains the shipped default here because it does not \
             meaningfully change this circuit's transient behavior relative to \
             trapezoidal (verified on a regenerative-LC-oscillator repro: bounded, \
             physical oscillation under both, peak/frequency within ~1%). If this \
             circuit is INTENDED to free-run as an oscillator/latch and you want the \
             legacy trapezoidal transient instead, use `--force-trap` (CLI) or \
             `.integrator trap` (netlist directive) to opt out of this promotion. If \
             this circuit is expected to be passively stable, investigate the BE \
             matrix builder instead.",
            stability.rho,
            stability.dominant_sign
        );
    }
}

/// Whether an independent, less-precise "trap unstable" finding (e.g. the
/// DK-vs-nodal router's fixed-iteration-count, non-deflected spectral-radius
/// estimate — `routing::RoutingDecision::dk_unstable`) should be allowed to
/// lift the gain-gate exclusion in [`trap_needs_be`]'s Nyquist-marginal
/// clause for the nodal auto-BE promotion.
///
/// The router's own estimator is measurably less accurate than the shared,
/// converged, input-deflated analyzer used here (verified 2026-07-25: on the
/// same DK-kernel matrices, the router's raw fixed-20-iteration number can
/// disagree with the converged+deflated value by several tens of percent —
/// e.g. tungsten-thunder-horse measures 1.1163 raw vs 0.8157
/// converged+deflated, comfortably trap-stable). Trusting the router's flag
/// unconditionally reproduces the wurli-power-amp fix but also force-promotes
/// circuits the accurate local analysis reports as nowhere near marginal,
/// which showed up as a severe (non-benign, near-zero-correlation)
/// golden-audio regression on tungsten-thunder-horse.
///
/// So the router's finding is only trusted as a tie-breaker: it may lift the
/// `max_abs_s` gain-gate requirement, but only when the LOCAL, accurate
/// estimate independently corroborates that this circuit is at least in the
/// marginal Nyquist zone (`rho > TRAP_BE_SIGN_FLIP_RHO` with a negative
/// dominant eigenvalue). A circuit the local analysis reports as comfortably
/// stable is never promoted no matter what the router claims.
pub fn router_corroborates_marginal_instability(
    router_flagged_unstable: bool,
    stability: TrapStability,
) -> bool {
    router_flagged_unstable
        && stability.rho > TRAP_BE_SIGN_FLIP_RHO
        && stability.dominant_sign < 0.0
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
        assert!(
            r.dominant_sign > 0.0,
            "expected positive sign, got {}",
            r.dominant_sign
        );
    }

    #[test]
    fn analyze_negative_identity_returns_unit_radius_negative() {
        // S·A_neg = -I → dominant eigenvalue is -1. Power iteration converges,
        // and the post-iteration <x, (S·A_neg)·x> = -|x|² has negative sign.
        let (s, n) = flat(&[&[-1.0, 0.0], &[0.0, -1.0]]);
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let r = analyze_trap_stability(&s, &id, n);
        assert!((r.rho - 1.0).abs() < 1e-6, "rho={}", r.rho);
        assert!(
            r.dominant_sign < 0.0,
            "expected negative sign, got {}",
            r.dominant_sign
        );
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
        assert!(
            r.dominant_sign < 0.0,
            "expected negative sign, got {}",
            r.dominant_sign
        );
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
        assert!(
            r.dominant_sign < 0.0,
            "expected negative sign, got {}",
            r.dominant_sign
        );
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
        assert!(
            r.dominant_sign > 0.0,
            "passive LF mode should classify positive, got {}",
            r.dominant_sign
        );
        assert!(
            !trap_needs_be(r),
            "ρ near +1 (passive LF) must NOT trip auto-BE"
        );
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
    fn close_subdominant_eigenvalue_converges_within_1e4() {
        // λ = {1.003, 0.98}: subdominant/dominant ratio 0.977. The old
        // fixed-20-iteration norm-ratio estimate lands at ≈ 0.9964 — BELOW
        // the 1.002 promotion threshold even though the true dominant
        // eigenvalue (1.003) is above it, so a genuinely trap-unstable mode
        // escaped auto-BE. The CONVERGED estimate must land within 1e-4 of
        // the true 1.003.
        let (s, n) = flat(&[&[1.003, 0.0], &[0.0, 0.98]]);
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let r = analyze_trap_stability(&s, &id, n);
        assert!(
            (r.rho - 1.003).abs() < 1e-4,
            "converged rho should be within 1e-4 of 1.003, got {}",
            r.rho
        );
        assert!(r.dominant_sign > 0.0, "dominant sign should be positive");
        assert!(
            trap_needs_be(r),
            "rho=1.003 > 1.002 must trip auto-BE (old fixed-20 estimate ~0.9964 missed it)"
        );
    }

    #[test]
    fn close_subdominant_negative_dominant_converges() {
        // Same closeness stress but with a negative dominant eigenvalue:
        // λ = {-1.003, 0.98}: rho must converge to 1.003, sign negative.
        let (s, n) = flat(&[&[-1.003, 0.0], &[0.0, 0.98]]);
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let r = analyze_trap_stability(&s, &id, n);
        assert!(
            (r.rho - 1.003).abs() < 1e-4,
            "converged rho should be within 1e-4 of 1.003, got {}",
            r.rho
        );
        assert!(
            r.dominant_sign < 0.0,
            "dominant sign should be negative, got {}",
            r.dominant_sign
        );
    }

    #[test]
    fn exact_plus_minus_pair_reports_unit_magnitude() {
        // diag(1, -1): the Rayleigh quotient cancels exactly (x·Ax = 0 for
        // the equal-component start vector), so the sign is unclassifiable
        // (0), while the norm ratio still reports the mode magnitude 1.
        let (s, n) = flat(&[&[1.0, 0.0], &[0.0, -1.0]]);
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let r = analyze_trap_stability(&s, &id, n);
        assert!(
            (r.rho - 1.0).abs() < 1e-6,
            "±1 pair should report rho=1 via norm-ratio fallback, got {}",
            r.rho
        );
        assert_eq!(r.dominant_sign, 0.0, "±λ pair sign is unclassifiable");
    }

    #[test]
    fn analyze_handles_empty_matrix() {
        let r = analyze_trap_stability(&[], &[], 0);
        assert_eq!(r.rho, 0.0);
        assert_eq!(r.dominant_sign, 0.0);
        assert!(!trap_needs_be(r));
    }

    // -- router_corroborates_marginal_instability (2026-07-25 fix) --------

    #[test]
    fn router_hint_corroborated_by_marginal_local_rho_trips() {
        // wurli-power-amp's actual measured values: nodal-local (converged,
        // deflated) rho=1.0005, dominant_sign=-1, max_abs_s=9.4e3 (below the
        // 1e5 gain gate, so `trap_needs_be` alone declines). The router
        // independently measured DK-kernel rho=1.0040 (> 1.002) on the
        // un-reduced circuit. The local estimate IS in the marginal window
        // (>0.999, sign<0), so the router's finding should be trusted here.
        let stability = TrapStability {
            rho: 1.0005,
            dominant_sign: -1.0,
            max_abs_s: 9.4e3,
        };
        assert!(
            !trap_needs_be(stability),
            "gain gate should decline promotion on its own for this low max_abs_s"
        );
        assert!(
            router_corroborates_marginal_instability(true, stability),
            "router's trap-unstable finding must be honored when the local estimate \
             independently corroborates a marginal (rho>0.999, sign<0) mode"
        );
    }

    #[test]
    fn router_hint_not_corroborated_by_comfortably_stable_local_rho_does_not_trip() {
        // tungsten-thunder-horse's actual measured values: nodal-local
        // (converged, deflated) rho=0.8157 — comfortably trap-stable,
        // nowhere near the marginal window. The router's raw (fixed-20-
        // iteration, non-deflected) estimate on the same DK-kernel matrices
        // was 1.1163 (> 1.002) — but that number is demonstrably inaccurate
        // (verified 2026-07-25 against the converged+deflated recompute on
        // the identical kernel matrices), so it must NOT force promotion
        // when the local estimate flatly disagrees. Blindly trusting the
        // router's flag here caused a severe, non-benign golden-audio
        // regression (correlation collapsing to ~0.006-0.6 on some test
        // programs) before this corroboration gate was added.
        let stability = TrapStability {
            rho: 0.8157,
            dominant_sign: -1.0,
            max_abs_s: 5.87e5,
        };
        assert!(!trap_needs_be(stability));
        assert!(
            !router_corroborates_marginal_instability(true, stability),
            "router's flag must NOT force promotion when the local estimate is \
             comfortably stable (rho=0.8157, far from the marginal window)"
        );
    }

    #[test]
    fn router_hint_ignored_when_not_flagged() {
        // Even a marginal local rho must not trip the corroboration gate
        // when the router itself never flagged trap-unstable (e.g. routed
        // nodal for an unrelated reason, or dk_unstable's own gates never
        // fired). This mirrors `trap_needs_be`'s existing gain-gate decline.
        let stability = TrapStability {
            rho: 1.0005,
            dominant_sign: -1.0,
            max_abs_s: 9.4e3,
        };
        assert!(!router_corroborates_marginal_instability(false, stability));
    }

    #[test]
    fn router_hint_ignored_on_positive_dominant_sign() {
        // Marginal magnitude but positive dominant eigenvalue (slow LF mode,
        // e.g. a passive-LC tank) must not be promoted even with the router
        // flag set — positive-sign marginal modes are handled exactly by
        // the bilinear transform (2026-04-22 passive-LC precedent).
        let stability = TrapStability {
            rho: 1.0005,
            dominant_sign: 1.0,
            max_abs_s: 9.4e3,
        };
        assert!(!router_corroborates_marginal_instability(true, stability));
    }
}
