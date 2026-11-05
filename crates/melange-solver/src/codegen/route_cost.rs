//! The nodal Schur-vs-full-LU COST rule: the last trigger in the sub-path
//! chain, consulted only for a circuit every structural and conditioning
//! guard has left on Schur. It moves a circuit to full-LU when the two
//! implementations of the same model are predicted to differ in per-sample
//! cost by more than [`COST_RATIO_MAX`], and never the other way. Ruled
//! 2026-11-04 (solver-design review of the Schur-vs-full-LU size/cost rule).
//!
//! # Why a cost rule exists
//!
//! On the nodal path both sub-paths solve the same circuit, each to its own
//! certification. Full-LU certifies an iterate on the undamped node step, the
//! device current-change check (the device currents after the step against
//! those before it) and the row-residual gate, and takes an exact-Jacobian
//! exit step after a chord-accepted iterate (or after a chord iteration past
//! the sample's first fails the current-change check). Schur certifies on
//! the undamped device-voltage step and the dense DK Newton's current-residual
//! check (`|i_nl − i_dev|`; its linear rows are exact by construction). Until the
//! residual check was added Schur certified on the step alone, and part of
//! its measured speed was that looser test: with it, the mid-size decks of
//! the fit sit at a measured Schur/full-LU ratio of about 1.0 (gravity 1.02,
//! rexi-mockup 1.01, seven-triode-cascade 0.99, against 0.93 / 0.91 / 0.78
//! before). The rule only ever moves a circuit from Schur to full-LU, and
//! [`COST_RATIO_MAX`] stays 1.5. The constants below are fitted to the
//! sub-paths as they certify now; a change to either certification is a
//! per-sample performance change (see the end of this doc).
//!
//! What differs otherwise is the per-sample work: the Schur sub-path
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
//! sine, 600 k samples × 3 reps, best rep, one core of this development
//! machine, `-C target-cpu=x86-64-v3`) on the nodal corpus decks with M ≥ 10
//! plus one M = 9 boundary deck, at their golden-manifest flags; iteration
//! counts from a 1 s `simulate -v` of the same stimulus. Re-fitted the same
//! day after Schur gained the current-residual check, on the same deck set
//! (the first fit's data through the same procedure reproduces the first
//! constants exactly). Rows where either route failed to certify (ceilings,
//! holds, unsolved samples) are excluded, as is a deck declared
//! unrepresentative (moonladder: K ill-conditioned at 5e11, the first fit's
//! largest miss when included). On this corpus `FULL_LU_NS_PER_SOLVE_FLOP`
//! is collinear with `FULL_LU_NS_PER_ITERATION` (the re-fit doubled the first
//! and moved the second by 3 % against the first fit, with the per-iteration
//! total on every row within a few %; the first fit's collinear pair was the
//! flop term with `FULL_LU_NS_PER_M`): the full-LU triple, not any one term,
//! is the measurement. The 15 fitted rows, ns per inner sample, Schur then
//! full-LU, measured (iterations per sample):
//!
//! ```text
//! deck                      N   M  solve_flops   Schur ns (it)   full-LU ns (it)   measured S/F
//! gravity (2×)             38   9     290        1253 (3.09)     1226 (2.77)        1.02
//! kt88-pp-stage            23  10     236        1495 (2.60)     2173 (3.35)        0.69
//! split-band-triode-drive  33  10     288        1702 (2.59)     2044 (2.90)        0.83
//! twill-deluxe             23  10     236        1568 (2.76)     2255 (3.13)        0.70
//! 4kbuscomp                76  10     622        2853 (1.09)     1044 (1.98)        2.73
//! farfisa-se15-preamp      48  11     412        2172 (2.71)     2229 (2.84)        0.97
//! split-band-ge-clamp      39  12     336        2638 (2.69)     3103 (3.08)        0.85
//! steve-1073-preamp        72  12     806        4349 (3.33)     3309 (3.30)        1.31
//! seven-triode-cascade     30  14     236        3199 (3.17)     3241 (3.58)        0.99
//! gravity-stereo (2×)      64  15     544        3995 (3.12)     1958 (2.79)        2.04
//! rexi-mockup              28  18     318        4072 (3.07)     4020 (4.11)        1.01
//! fet limiter, 1e12-K form 84  27     972       23301 (7.48)     3996 (2.83)        5.83   (forced Schur on a K-diag-negative deck)
//! same deck (2×)           84  27     972        8662 (1.99)     3567 (2.64)        2.43
//! fet limiter, revised     84  28     972       11403 (2.76)     5004 (3.38)        2.28
//! same deck (2×)           84  28     972       10519 (2.47)     4471 (3.10)        2.35
//! ```
//!
//! The fit predicts both sub-paths' measured per-sample cost within 15 % on
//! 11 of the 15 rows at the measured iteration counts (misses: gravity, both
//! sub-paths over by 41 % / 50 % with the ratio within 6 %; 4kbuscomp and
//! gravity-stereo full-LU over by 44 % / 31 %; split-band-ge-clamp full-LU
//! under by 23 %). On the predicted Schur/full-LU RATIO, which is what the
//! rule reads, the worst over-prediction of Schur's relative cost at the
//! measured iteration counts is 1.13× (farfisa-se15-preamp 1.10 predicted vs
//! 0.97, steve-1073-preamp 1.48 vs 1.31), and the large misses are all in the
//! direction of Schur being dearer than modelled (4kbuscomp 2.01 vs 2.73,
//! gravity-stereo 1.75 vs 2.04). With the nominal iteration pair in place of
//! the measured counts the worst over-prediction is 1.21× (rexi-mockup 1.22
//! vs 1.01), under the 1.5× margin: no corpus deck that measured
//! Schur-faster is moved (their largest nominal ratio is 1.04). Evaluated
//! over every nodal M > 0 deck of the corpus the rule moves exactly one (the
//! revised FET limiter, 2.54), and that one is also the direct measurement
//! above and the adopting repo's drum-loop timings (Schur 1.9–2.8× full-LU
//! across 0 to +24 dB of drive, both rates).
//!
//! **A per-sample performance change to EITHER sub-path invalidates these
//! constants.** The crossover is what the rule places; a sparser Schur
//! matvec, a cheaper packed back-solve, a different Newton loop body all move
//! it. Re-run the forced-both-ways bench on the same deck set and re-fit
//! before shipping such a change, and update this table.

/// Schur sub-path: ns per inner sample per N², the dense `S·rhs` matvec and
/// its N-sized neighbours (one core, 2026-11-04 fit; see module doc).
pub const SCHUR_NS_PER_N2: f64 = 0.442;
/// Schur sub-path: ns per Newton iteration per M³, the dense M×M elimination.
pub const SCHUR_NS_PER_M3: f64 = 0.018;
/// Schur sub-path: ns per Newton iteration per M², the Jacobian assembly
/// `I − J_dev·K` and the M×M products around it.
pub const SCHUR_NS_PER_M2: f64 = 3.1;
/// Schur sub-path: ns per Newton iteration independent of size (device
/// evaluation overhead, loop bookkeeping).
pub const SCHUR_NS_PER_ITERATION: f64 = 100.0;
/// Full-LU sub-path: ns per Newton iteration per packed back-solve flop
/// (`LuSparsity::solve_flops`). Collinear with the per-iteration constant on
/// the fitted corpus; the full-LU triple, not any one term, is the
/// measurement.
pub const FULL_LU_NS_PER_SOLVE_FLOP: f64 = 0.171;
/// Full-LU sub-path: ns per Newton iteration per nonlinear dimension (device
/// evaluation and the Jacobian stamp into `G_aug`).
pub const FULL_LU_NS_PER_M: f64 = 35.0;
/// Full-LU sub-path: ns per Newton iteration independent of size (residual
/// assembly, the step and residual tests, refactor bookkeeping).
pub const FULL_LU_NS_PER_ITERATION: f64 = 301.0;
/// Nominal Newton iterations per inner sample on the Schur sub-path at which
/// the rule is evaluated: the corpus median at 0.3 V, 2.76 with the
/// current-residual check (2.25 before it; 1.09–3.33 measured, 7.48 on the
/// forced-Schur 1e12-K deck).
pub const NOMINAL_ITERATIONS_SCHUR: f64 = 2.7;
/// Nominal Newton iterations per inner sample on the full-LU sub-path: the
/// corpus median at 0.3 V (3.08; 1.98–4.11 measured). The pair's ratio (0.9)
/// is the corpus relation (median 0.89; 0.55–1.18 per deck apart from the
/// 1e12-K deck's 2.6).
pub const NOMINAL_ITERATIONS_FULL_LU: f64 = 3.0;
/// Predicted Schur / full-LU cost ratio above which the rule routes full-LU.
/// 1.5 leaves the corpus's worst over-prediction (1.21× at the nominal pair)
/// under it and the one deck it moves (2.54) above it by 1.69×; any value in
/// [1.25, 2.5] moves the same one deck (the nearest auto-Schur deck,
/// rexi-mockup, predicts 1.22).
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
        assert!((2.4..2.7).contains(&rev.ratio()), "{}", rev.ratio());
        // The nearest auto-Schur decks stay: rexi-mockup (measured 1.01) and
        // kt88-pp-stage (measured 0.69).
        let rexi = predict(28, 18, Some(318)).unwrap();
        assert!(!rexi.prefers_full_lu(), "{rexi:?}");
        assert!((1.1..1.35).contains(&rexi.ratio()), "{}", rexi.ratio());
        let kt88 = predict(23, 10, Some(236)).unwrap();
        assert!(!kt88.prefers_full_lu(), "{kt88:?}");
        assert!(kt88.ratio() < 0.7, "{}", kt88.ratio());
        // A deck already on full-LU by conditioning, for scale: the rule
        // agrees (4kbuscomp, measured 2.73).
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
            (84.0, 28.0, 972.0, 11403.0, 2.76, 5004.0, 3.38), // revised FET limiter 1×
            (84.0, 28.0, 972.0, 10519.0, 2.47, 4471.0, 3.10), // same, 2×
            (72.0, 12.0, 806.0, 4349.0, 3.33, 3309.0, 3.30),  // steve-1073-preamp
            (28.0, 18.0, 318.0, 4072.0, 3.07, 4020.0, 4.11),  // rexi-mockup
            (23.0, 10.0, 236.0, 1495.0, 2.60, 2173.0, 3.35),  // kt88-pp-stage
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
