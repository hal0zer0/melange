//! The nodal Schur-vs-full-LU COST rule: the last trigger in the sub-path
//! chain, consulted only for a circuit every structural and conditioning
//! guard has left on Schur. It moves a circuit to full-LU when the two
//! implementations of the same model are predicted to differ in per-sample
//! cost by more than [`COST_RATIO_MAX`], and never the other way. Ruled
//! 2026-11-04 (solver-design review of the Schur-vs-full-LU size/cost rule).
//!
//! # Why a cost rule exists
//!
//! On the nodal path both sub-paths solve the same circuit to the same
//! certification. What differs is the per-sample work: the Schur sub-path
//! applies the dense N×N `S = A⁻¹` to the right-hand side every sample and
//! eliminates a dense M×M Jacobian every Newton iteration, while the full-LU
//! sub-path back-solves a packed sparse factor of `G_aug` whose length is the
//! fill of the circuit's own pattern (`crate::lu::LuSparsity::solve_flops`),
//! typically ~1 % of N². Past N ≈ 60 the dense matvec alone costs more than
//! the whole sparse solve, and a large-N circuit with no conditioning guard
//! tripped (a 1 MΩ leak keeps max|S| a hair under the s-ill threshold) ships
//! on the route that is two to three times dearer for nothing.
//!
//! # The model
//!
//! Per INNER sample (one solver step; a host sample at 2× oversampling is two):
//!
//! ```text
//! schur   = SCHUR_NS_PER_N2·N² + NOMINAL_ITERATIONS_SCHUR·(SCHUR_NS_PER_M3·M³ + SCHUR_NS_PER_M2·M² + SCHUR_NS_PER_ITERATION)
//! full_lu = NOMINAL_ITERATIONS_FULL_LU·(FULL_LU_NS_PER_SOLVE_FLOP·solve_flops + FULL_LU_NS_PER_M·M + FULL_LU_NS_PER_ITERATION)
//! route full-LU when schur > COST_RATIO_MAX·full_lu
//! ```
//!
//! Every input is known at compile time: N and M from the topology,
//! `solve_flops` from the packed sparse factor the IR builds before the
//! sub-path is chosen. Where no packed pattern exists (`ir.sparsity.lu` is
//! `None`: dense full-LU would be the alternative, and that is never cheaper
//! than Schur) the rule is inert.
//!
//! # Provenance of the constants
//!
//! Least-squares fit of the two forms above to forced-both-ways timings
//! (`--nodal-subpath schur` and `full-lu` on the same netlist, same flags)
//! measured 2026-11-04 with `tools/perf-harness/bench.sh` (0.3 V 220 Hz
//! sine, 2 M samples × 3 reps, best rep, one core of this development
//! machine, `-C target-cpu=x86-64-v3`) on the nodal corpus decks with M ≥ 10
//! plus one M = 9 boundary deck, at their golden-manifest flags; iteration
//! counts from a 1 s `simulate -v` of the same stimulus. Rows where either
//! route failed to certify (ceilings, holds, unsolved samples) are excluded,
//! as is a deck declared unrepresentative (moonladder: K ill-conditioned at
//! 5e11, the fit's largest miss when included; dropping it moved the Schur
//! coefficients by < 15 % and halved `FULL_LU_NS_PER_SOLVE_FLOP` against a
//! rise in `FULL_LU_NS_PER_M` — the two are collinear on this corpus — with
//! no deck's predicted ratio moving by more than 0.1). The 15 fitted rows,
//! ns per inner sample, Schur then full-LU, measured (iterations per sample):
//!
//! ```text
//! deck                      N   M  solve_flops   Schur ns (it)   full-LU ns (it)   measured S/F
//! gravity (2×)             38   9     290        1162 (2.71)     1243 (3.01)        0.93
//! kt88-pp-stage            23  10     236        1432 (2.39)     2231 (3.45)        0.64
//! split-band-triode-drive  33  10     288        1500 (2.11)     2175 (3.09)        0.69
//! twill-deluxe             23  10     236        1439 (2.38)     2313 (3.28)        0.62
//! 4kbuscomp                76  10     622        2799 (1.00)     1040 (1.98)        2.69
//! farfisa-se15-preamp      48  11     412        1856 (2.16)     2375 (3.15)        0.78
//! split-band-ge-clamp      39  12     336        2393 (2.28)     3324 (3.30)        0.72
//! steve-1073-preamp        72  12     806        3888 (2.63)     3509 (3.89)        1.11
//! seven-triode-cascade     30  14     236        2796 (2.63)     3598 (4.03)        0.78
//! gravity-stereo (2×)      64  15     544        3729 (2.74)     1932 (3.03)        1.93
//! rexi-mockup              28  18     318        3713 (2.77)     4101 (4.20)        0.91
//! fet limiter, 1e12-K form 84  27     972       21579 (6.68)     4001 (2.83)        5.39   (forced Schur on a K-diag-negative deck)
//! same deck (2×)           84  27     972        5987 (1.00)     3528 (2.64)        1.70
//! fet limiter, revised     84  28     972       10289 (2.40)     5259 (3.75)        1.96
//! same deck (2×)           84  28     972        9648 (2.21)     4624 (3.35)        2.09
//! ```
//!
//! The fit predicts the measured per-sample cost within 15 % on 12 of the 15
//! rows at the measured iteration counts; its misses are all in the direction
//! of Schur being dearer than modelled (4kbuscomp 2.05 predicted vs 2.69,
//! gravity-stereo 1.54 vs 1.93) except farfisa-se15-preamp (0.90 vs 0.78).
//! With the nominal iteration pair in place of the measured counts the worst
//! over-prediction of Schur's relative cost on the corpus is about 1.33×,
//! under the 1.5× margin: no corpus deck that measured Schur-faster is moved.
//! Evaluated over every nodal M > 0 deck of the corpus the rule moves exactly
//! one (the revised FET limiter, 2.3), and that one is also the direct
//! measurement above and the adopting repo's drum-loop timings (Schur 1.9–2.8×
//! full-LU across 0 to +24 dB of drive, both rates).
//!
//! **A per-sample performance change to EITHER sub-path invalidates these
//! constants.** The crossover is what the rule places; a sparser Schur
//! matvec, a cheaper packed back-solve, a different Newton loop body all move
//! it. Re-run the forced-both-ways bench on the same deck set and re-fit
//! before shipping such a change, and update this table.

/// Schur sub-path: ns per inner sample per N², the dense `S·rhs` matvec and
/// its N-sized neighbours (one core, 2026-11-04 fit; see module doc).
pub const SCHUR_NS_PER_N2: f64 = 0.424;
/// Schur sub-path: ns per Newton iteration per M³, the dense M×M elimination.
pub const SCHUR_NS_PER_M3: f64 = 0.019;
/// Schur sub-path: ns per Newton iteration per M², the Jacobian assembly
/// `I − J_dev·K` and the M×M products around it.
pub const SCHUR_NS_PER_M2: f64 = 3.2;
/// Schur sub-path: ns per Newton iteration independent of size (device
/// evaluation overhead, loop bookkeeping).
pub const SCHUR_NS_PER_ITERATION: f64 = 105.0;
/// Full-LU sub-path: ns per Newton iteration per packed back-solve flop
/// (`LuSparsity::solve_flops`). Collinear with the M term on the fitted
/// corpus; the pair, not either alone, is the measurement.
pub const FULL_LU_NS_PER_SOLVE_FLOP: f64 = 0.077;
/// Full-LU sub-path: ns per Newton iteration per nonlinear dimension (device
/// evaluation and the Jacobian stamp into `G_aug`).
pub const FULL_LU_NS_PER_M: f64 = 37.0;
/// Full-LU sub-path: ns per Newton iteration independent of size (residual
/// assembly, the step and residual tests, refactor bookkeeping).
pub const FULL_LU_NS_PER_ITERATION: f64 = 293.0;
/// Nominal Newton iterations per inner sample on the Schur sub-path at which
/// the rule is evaluated: the corpus median at 0.3 V (2.1–2.9 measured).
pub const NOMINAL_ITERATIONS_SCHUR: f64 = 2.25;
/// Nominal Newton iterations per inner sample on the full-LU sub-path: the
/// corpus median at 0.3 V (3.0–4.2 measured). The pair's ratio (0.75) is
/// the corpus relation; per-deck ratios ran 0.64–0.93.
pub const NOMINAL_ITERATIONS_FULL_LU: f64 = 3.0;
/// Predicted Schur / full-LU cost ratio above which the rule routes full-LU.
/// 1.5 leaves the corpus's worst over-prediction (about 1.33×) under it and
/// the one deck it moves (2.3) above it by 1.55×; any value in [1.2, 2.2]
/// moves the same one deck.
pub const COST_RATIO_MAX: f64 = 1.5;

/// The two per-inner-sample cost predictions the rule compares, in ns.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CostPrediction {
    /// Predicted Schur sub-path cost at [`NOMINAL_ITERATIONS_SCHUR`].
    pub schur_ns: f64,
    /// Predicted full-LU sub-path cost at [`NOMINAL_ITERATIONS_FULL_LU`].
    pub full_lu_ns: f64,
}

impl CostPrediction {
    /// Schur / full-LU.
    pub fn ratio(&self) -> f64 {
        self.schur_ns / self.full_lu_ns
    }

    /// Whether the rule routes full-LU: predicted Schur cost above
    /// [`COST_RATIO_MAX`] times the predicted full-LU cost.
    pub fn prefers_full_lu(&self) -> bool {
        self.schur_ns > COST_RATIO_MAX * self.full_lu_ns
    }
}

/// Predict both sub-paths' per-inner-sample cost for a nodal circuit with
/// `n` unknowns, `m` nonlinear dimensions and a packed sparse factor of
/// `solve_flops` back-solve flops. `None` when there is no packed factor
/// (`solve_flops` is `None`): the rule is inert there, see the module doc.
pub fn predict(n: usize, m: usize, solve_flops: Option<usize>) -> Option<CostPrediction> {
    let solve_flops = solve_flops? as f64;
    let (n, m) = (n as f64, m as f64);
    let schur_ns = SCHUR_NS_PER_N2 * n * n
        + NOMINAL_ITERATIONS_SCHUR
            * (SCHUR_NS_PER_M3 * m * m * m + SCHUR_NS_PER_M2 * m * m + SCHUR_NS_PER_ITERATION);
    let full_lu_ns = NOMINAL_ITERATIONS_FULL_LU
        * (FULL_LU_NS_PER_SOLVE_FLOP * solve_flops
            + FULL_LU_NS_PER_M * m
            + FULL_LU_NS_PER_ITERATION);
    Some(CostPrediction {
        schur_ns,
        full_lu_ns,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The fitted rows of the module doc, evaluated at the nominal iteration
    /// pair: the one deck the rule moves and the nearest decks it leaves.
    #[test]
    fn corpus_rows_at_the_nominal_pair() {
        // The revised FET limiter: the one corpus deck the rule moves.
        let rev = predict(84, 28, Some(972)).unwrap();
        assert!(rev.prefers_full_lu(), "{rev:?}");
        assert!((2.2..2.5).contains(&rev.ratio()), "{}", rev.ratio());
        // The nearest auto-Schur decks stay: rexi-mockup (measured 0.91) and
        // kt88-pp-stage (measured 0.64).
        let rexi = predict(28, 18, Some(318)).unwrap();
        assert!(!rexi.prefers_full_lu(), "{rexi:?}");
        assert!((0.95..1.2).contains(&rexi.ratio()), "{}", rexi.ratio());
        let kt88 = predict(23, 10, Some(236)).unwrap();
        assert!(!kt88.prefers_full_lu(), "{kt88:?}");
        assert!(kt88.ratio() < 0.7, "{}", kt88.ratio());
        // A deck already on full-LU by conditioning, for scale: the rule
        // agrees (4kbuscomp, measured 2.69).
        let bus = predict(76, 10, Some(622)).unwrap();
        assert!(bus.prefers_full_lu(), "{bus:?}");
    }

    /// The fit reproduces the measured per-sample cost at the MEASURED
    /// iteration counts within 15 % on the rows the module doc says it does.
    #[test]
    fn fit_reproduces_measured_rows() {
        let schur = |n: f64, m: f64, it: f64| {
            SCHUR_NS_PER_N2 * n * n
                + it * (SCHUR_NS_PER_M3 * m * m * m
                    + SCHUR_NS_PER_M2 * m * m
                    + SCHUR_NS_PER_ITERATION)
        };
        let full_lu = |m: f64, sf: f64, it: f64| {
            it * (FULL_LU_NS_PER_SOLVE_FLOP * sf + FULL_LU_NS_PER_M * m + FULL_LU_NS_PER_ITERATION)
        };
        // (N, M, solve_flops, Schur ns, it_S, full-LU ns, it_F)
        let rows: [(f64, f64, f64, f64, f64, f64, f64); 5] = [
            (84.0, 28.0, 972.0, 10289.0, 2.40, 5259.0, 3.75), // revised FET limiter 1×
            (84.0, 28.0, 972.0, 9648.0, 2.21, 4624.0, 3.35),  // same, 2×
            (72.0, 12.0, 806.0, 3888.0, 2.63, 3509.0, 3.89),  // steve-1073-preamp
            (28.0, 18.0, 318.0, 3713.0, 2.77, 4101.0, 4.20),  // rexi-mockup
            (23.0, 10.0, 236.0, 1432.0, 2.39, 2231.0, 3.45),  // kt88-pp-stage
        ];
        for (n, m, sf, ms, its, mf, itf) in rows {
            let ps = schur(n, m, its);
            let pf = full_lu(m, sf, itf);
            assert!(
                (ps / ms - 1.0).abs() < 0.15,
                "Schur N={n} M={m}: {ps} vs {ms}"
            );
            assert!(
                (pf / mf - 1.0).abs() < 0.15,
                "full-LU N={n} M={m}: {pf} vs {mf}"
            );
        }
    }

    #[test]
    fn inert_without_a_packed_factor() {
        assert!(predict(200, 40, None).is_none());
    }
}
