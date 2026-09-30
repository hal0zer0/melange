//! Validate across sample rates, to separate discretization from model error.
//!
//! A deck that fails at its rate may be modelled exactly and merely
//! integrated coarsely: its error then falls with the step toward the
//! reference. One whose error stops falling, or rises, converges to something
//! other than the reference: a model or harness mismatch. [`rate_sweep`]
//! renders the deck at `fs`, `2·fs` and `4·fs` (oversampling off, the
//! analytic stimulus sampled at each rate), and at `8·fs` too when three
//! rates leave the convergence ambiguous, and classifies it:
//!
//! - `Pass` at the requested rate;
//! - `Converges`: the error falls toward the reference. The asymptotic
//!   (MODEL) error and the rate needed for a tolerance are reported;
//! - `Plateau`: the error has stopped falling (the finest ratio below
//!   [`PLATEAU_RATIO`]); `Diverges`: it rises;
//! - `Unresolved`: the sweep cannot tell, and says why ([`Unresolved`]).
//!
//! **One reference.** Every render is graded against ONE reference: the
//! finest run's converged ngspice output, taken at the render's rate (an
//! exact subsample: the rates are multiples) and DC-blocked there as the
//! render is. A reference per rate is a different object at each rate, which
//! confounded the per-rate comparison on a near-square deck; one blocker at
//! the finest rate would leave the blocker's own first-order discretization
//! in the errors. Each rate's error is taken over ALL its samples.
//!
//! **The error extrapolated.** The asymptote is extrapolated on the error
//! waveform `e_k = y_k − ref_k` of the three finest renders, at the instants
//! they share (the coarsest one's grid), not on the error metric (RMS errors
//! add in quadrature when uncorrelated, so a metric fit is biased exactly
//! where a floor exists):
//!
//! ```text
//! r   = Σ (e1 − e2)(e2 − e4) / Σ (e2 − e4)²     (least squares), p = log2 r
//! e∞  = e4 + (e4 − e2)/(2^p − 1)
//! model error = ‖e∞‖ / ‖ref‖
//! ```
//!
//! Aitken on the three error figures is the cross-check, and the fallback
//! when the waveform fit has no answer, which the verdict then says. The
//! shared grid samples the finest render at one instant in four: where its
//! error lives in edges a few samples wide, the grid cannot see it, and the
//! fit is refused rather than made on a measure that misses the error
//! ([`EDGE_RATIO`]).
//!
//! **The reference must resolve what it grades.** Every reference has
//! passed its own convergence ladder (`reference`), and the finest one must
//! also be accurate to a third of the smallest error it grades. When it is
//! not, it is refined once to that bound; if it still is not, the
//! convergence is `Unresolved`, not classified.

use std::path::Path;

use crate::{
    validate_circuit_with_options, AnalyticStimulus, ComparisonConfig, ValidationError,
    ValidationOptions, ValidationResult,
};

/// One rate of a sweep.
#[derive(Debug, Clone)]
pub struct RateRow {
    pub sample_rate: f64,
    /// Normalized RMS error of this render against the finest reference at
    /// this rate, over all its samples after the settle window.
    pub error: f64,
    /// Normalized RMS error from validate's own comparison at this rate (its
    /// own reference, aligned): the ordinary validate number.
    pub own_error: f64,
    /// Whether validate's own comparison passed at this rate.
    pub passed: bool,
    /// The integrator the build used here.
    pub integrator: &'static str,
}

/// The finest reference's self-check must be at most this fraction of the
/// smallest error it grades. Stated.
pub const RESOLUTION_FRACTION: f64 = 1.0 / 3.0;

/// The error has stopped falling when the finest ratio `e(2h)/e(h)` is below
/// this. Stated.
pub const PLATEAU_RATIO: f64 = 1.5;

/// The two successive error ratios describe one convergence when they agree
/// to within this fraction. Stated.
pub const RATIO_AGREEMENT: f64 = 0.2;

/// The shared grid sees the finest render's error when its grid error and
/// all-sample error agree to within this factor. Stated.
pub const EDGE_RATIO: f64 = 1.5;

/// Which fit a convergence verdict rests on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fit {
    /// Extrapolated on the waveform.
    Waveform,
    /// The waveform fit had no answer (pre-asymptotic); Aitken on the three
    /// error metrics.
    ErrorMetric,
}

/// Why a sweep cannot classify the convergence.
#[derive(Debug, Clone, PartialEq)]
pub enum Unresolved {
    /// The finest reference's self-check is not below `RESOLUTION_FRACTION`
    /// of the smallest error graded.
    ReferenceTooCoarse { self_check: f64, error: f64 },
    /// The finest render's error on the shared grid and over all its
    /// samples differ by more than `EDGE_RATIO`: the error lives between the
    /// grid's instants.
    EdgeDominated { grid: f64, all: f64 },
    /// The successive error ratios disagree by more than `RATIO_AGREEMENT`
    /// (`ratios` = e(4h)/e(2h), e(2h)/e(h) of the three finest renders).
    PreAsymptotic { ratios: [f64; 2] },
    /// The ratios agree and the error still falls, but the fit puts a floor
    /// at half the finest error or more: the fits contradict each other.
    FloorAmbiguous { model_error: f64, error: f64 },
}

/// How a deck's error behaves with the step.
#[derive(Debug, Clone)]
pub enum SweepVerdict {
    /// Passes at the requested (lowest) rate.
    Pass,
    /// Falls toward the reference.
    Converges {
        /// Order of the fit.
        order: f64,
        /// The asymptotic (model) error.
        model_error: f64,
        fit: Fit,
    },
    /// Stops falling: converges to something other than the reference.
    Plateau { order: f64 },
    /// Rises with the rate.
    Diverges,
    /// The sweep cannot tell.
    Unresolved(Unresolved),
}

/// The rate a tolerance needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RateFor {
    /// This rate.
    At(f64),
    /// No rate: the model error is at or above the tolerance, or the deck
    /// does not converge.
    Unreachable,
    /// The sweep cannot tell: the reference is not accurate enough, or the
    /// convergence is unresolved.
    Unresolved,
}

/// A sweep's rows, fits and verdict.
#[derive(Debug, Clone)]
pub struct RateSweep {
    /// `fs`, `2fs`, `4fs`, and `8fs` when three rates were ambiguous.
    pub rows: Vec<RateRow>,
    pub verdict: SweepVerdict,
    /// The classification of the errors whether or not the requested rate
    /// passed: what the rate for a tolerance is read from.
    pub convergence: SweepVerdict,
    /// Cross-check: the asymptote from the three finest error figures
    /// (Aitken), `None` where they do not fall monotonically.
    pub model_error_metric: Option<f64>,
    /// The finest reference's own self-check (normalized RMS against its
    /// refinements).
    pub reference_self_check: f64,
    /// Whether the finest reference was refined past validate's own bound
    /// to resolve the errors graded.
    pub reference_refined: bool,
    /// The finest render's error on the shared grid (against `rows`' last
    /// all-sample error, for the edge check).
    pub finest_grid_error: f64,
}

impl RateSweep {
    /// The sample rate at which the error falls to `tol`. The requested rate
    /// itself when it already meets `tol`; otherwise extrapolated with the
    /// fit from the finest rate whose error is still above `tol`, and no
    /// higher than the first rate that meets it. `Unresolved` when the
    /// reference's own error is not below `RESOLUTION_FRACTION` of `tol` or
    /// the convergence is unresolved.
    pub fn rate_for(&self, tol: f64) -> RateFor {
        let Some(base) = self.rows.first() else {
            return RateFor::Unreachable;
        };
        if self.reference_self_check > RESOLUTION_FRACTION * tol {
            return RateFor::Unresolved;
        }
        if base.error <= tol {
            return RateFor::At(base.sample_rate);
        }
        match self.convergence {
            SweepVerdict::Converges {
                order, model_error, ..
            } if model_error < tol => {
                let meets = self.rows.iter().position(|r| r.error <= tol);
                let anchor = &self.rows[meets.map_or(self.rows.len() - 1, |i| i - 1)];
                let factor = ((anchor.error - model_error) / (tol - model_error)).powf(1.0 / order);
                let rate = anchor.sample_rate * factor.max(1.0);
                RateFor::At(meets.map_or(rate, |i| rate.min(self.rows[i].sample_rate)))
            }
            SweepVerdict::Unresolved(_) => RateFor::Unresolved,
            _ => RateFor::Unreachable,
        }
    }
}

/// Aitken asymptote of three errors at `h`, `h/2`, `h/4` (the cross-check).
pub fn metric_asymptote(e: [f64; 3]) -> Option<f64> {
    let [e1, e2, e4] = e;
    if !(e1 > e2 && e2 > e4) {
        return None;
    }
    let order = ((e1 - e2) / (e2 - e4)).log2();
    (order.is_finite() && order > 0.0).then(|| (e4 - (e2 - e4) / (2f64.powf(order) - 1.0)).max(0.0))
}

/// The waveform fit: order and asymptotic waveform from three renders at
/// common instants.
pub fn extrapolate(y1: &[f64], y2: &[f64], y4: &[f64]) -> Option<(f64, Vec<f64>)> {
    let (mut num, mut den) = (0.0, 0.0);
    for i in 0..y4.len() {
        let d12 = y1[i] - y2[i];
        let d24 = y2[i] - y4[i];
        num += d12 * d24;
        den += d24 * d24;
    }
    let r = num / den;
    if !(r.is_finite() && r > 1.0) {
        return None;
    }
    let order = r.log2();
    let k = 1.0 / (2f64.powf(order) - 1.0);
    let y_inf = y4.iter().zip(y2).map(|(&a, &b)| a + (a - b) * k).collect();
    Some((order, y_inf))
}

/// RMS of `v`.
fn rms(v: &[f64]) -> f64 {
    (v.iter().map(|x| x * x).sum::<f64>() / v.len().max(1) as f64).sqrt()
}

/// Relative RMS of `a − b` against `b`.
#[cfg(test)]
fn rel_rms(a: &[f64], b: &[f64]) -> f64 {
    let d: Vec<f64> = a.iter().zip(b).map(|(x, y)| x - y).collect();
    rms(&d) / rms(b).max(1e-300)
}

/// Classify three errors at `h`, `h/2`, `h/4` with the waveform fit
/// `(order, model error)`. Without a waveform fit, falls back to the metric
/// fit.
pub fn classify(e: [f64; 3], waveform: Option<(f64, f64)>) -> SweepVerdict {
    let [e1, e2, e4] = e;
    if e4 > e2 * 1.05 || e2 > e1 * 1.05 {
        return SweepVerdict::Diverges;
    }
    let (r1, r2) = (e1 / e2, e2 / e4);
    if r2 < PLATEAU_RATIO {
        return SweepVerdict::Plateau { order: r2.log2() };
    }
    if (r1 / r2 - 1.0).abs() > RATIO_AGREEMENT {
        return SweepVerdict::Unresolved(Unresolved::PreAsymptotic { ratios: [r1, r2] });
    }
    let (order, model_error, fit) = match (waveform, metric_asymptote(e)) {
        (Some((order, model_error)), _) => (order, model_error, Fit::Waveform),
        (None, Some(model_error)) => (
            ((e1 - e2) / (e2 - e4)).log2(),
            model_error,
            Fit::ErrorMetric,
        ),
        (None, None) => {
            return SweepVerdict::Unresolved(Unresolved::PreAsymptotic { ratios: [r1, r2] })
        }
    };
    if model_error >= 0.5 * e4 {
        return SweepVerdict::Unresolved(Unresolved::FloorAmbiguous {
            model_error,
            error: e4,
        });
    }
    SweepVerdict::Converges {
        order,
        model_error,
        fit,
    }
}

/// A render: its factor over the base rate, its rate, its validation.
type Run = (usize, f64, ValidationResult);

/// The renders graded against the finest run's reference.
struct Graded {
    rows: Vec<RateRow>,
    /// The waveform fit on the three finest renders' shared grid.
    fit: Option<(f64, f64)>,
    /// The finest render's error on that grid.
    finest_grid: f64,
}

/// Grade every render against the finest run's reference at its own rate,
/// over all its samples; fit the three finest on their shared grid.
fn grade(runs: &[Run], base_rate: f64, settle_s: f64) -> Result<Graded, ValidationError> {
    let finest = &runs[runs.len() - 1].2;
    // The finest reference, at each render's rate and DC-blocked there, as
    // the render was by its own blocker. One blocker for every rate would
    // leave the blocker's own discretization (first order in the step: its
    // gain at 1 kHz is 3e-4 low at 48 kHz, 8e-5 at 192 kHz) in the errors.
    let references: Vec<Vec<f64>> = runs
        .iter()
        .map(|(_, fs, _)| {
            let step = (finest.reference_rate / fs).round() as usize;
            let mut r: Vec<f64> = finest.reference_raw.iter().step_by(step).copied().collect();
            crate::dc_block_signal(&mut r, *fs);
            r
        })
        .collect();
    let rows = runs
        .iter()
        .zip(&references)
        .map(|((_, fs, r), reference)| {
            let end = r.melange_raw.len().min(reference.len());
            let start = ((settle_s * fs).ceil() as usize).min(end);
            let e: Vec<f64> = r.melange_raw[start..end]
                .iter()
                .zip(&reference[start..end])
                .map(|(y, x)| y - x)
                .collect();
            RateRow {
                sample_rate: *fs,
                error: rms(&e) / rms(&reference[start..end]).max(1e-300),
                own_error: r.report.normalized_rms_error,
                passed: r.report.passed,
                integrator: r.integrator,
            }
        })
        .collect();

    // The three finest renders at the instants they share: the coarsest
    // one's grid.
    let n = runs.len();
    let base = runs[n - 3].0;
    let grid_rate = base_rate * base as f64;
    let start = (settle_s * grid_rate).ceil() as usize;
    let len = (n - 3..n)
        .map(|k| runs[k].2.melange_raw.len().min(references[k].len()) / (runs[k].0 / base))
        .min()
        .unwrap_or(0);
    if len <= start + 16 {
        return Err(ValidationError::InvalidInput(
            "rate sweep: no common window after the settle time".to_string(),
        ));
    }
    let at = |v: &[f64], step: usize| -> Vec<f64> { (start..len).map(|i| v[i * step]).collect() };
    let reference = at(&references[n - 1], runs[n - 1].0 / base);
    let errors: Vec<Vec<f64>> = (n - 3..n)
        .map(|k| {
            let step = runs[k].0 / base;
            at(&runs[k].2.melange_raw, step)
                .iter()
                .zip(at(&references[k], step))
                .map(|(y, x)| y - x)
                .collect()
        })
        .collect();
    let reference_rms = rms(&reference).max(1e-300);
    let norm = |e: &[f64]| rms(e) / reference_rms;
    let fit = extrapolate(&errors[0], &errors[1], &errors[2]).map(|(p, e_inf)| (p, norm(&e_inf)));
    Ok(Graded {
        rows,
        fit,
        finest_grid: norm(&errors[2]),
    })
}

/// The finest run's reference must resolve the errors: refined once when
/// not, then the three finest renders classified.
struct Evaluated {
    graded: Graded,
    convergence: SweepVerdict,
    self_check: f64,
    refined: bool,
}

fn evaluate(
    runs: &mut [Run],
    run: &dyn Fn(usize, Option<f64>) -> Result<Run, ValidationError>,
    base_rate: f64,
    settle_s: f64,
) -> Result<Evaluated, ValidationError> {
    let n = runs.len();
    let self_check = |runs: &[Run]| {
        runs[runs.len() - 1]
            .2
            .report
            .reference_self_check
            .unwrap_or(0.0)
    };
    let mut graded = grade(runs, base_rate, settle_s)?;
    let mut refined = false;
    let e_finest = graded.rows[n - 1].error;
    if self_check(runs) > RESOLUTION_FRACTION * e_finest {
        // Refine the finest reference to the error it must resolve. A ladder
        // that cannot reach it leaves the convergence unresolved.
        match run(runs[n - 1].0, Some(RESOLUTION_FRACTION * e_finest)) {
            Ok(finer) => {
                runs[n - 1] = finer;
                graded = grade(runs, base_rate, settle_s)?;
                refined = true;
            }
            Err(ValidationError::ReferenceNotConverged(_)) => {}
            Err(e) => return Err(e),
        }
    }
    let sc = self_check(runs);
    let e = [
        graded.rows[n - 3].error,
        graded.rows[n - 2].error,
        graded.rows[n - 1].error,
    ];
    let (grid, all) = (graded.finest_grid, e[2]);
    let convergence = if sc > RESOLUTION_FRACTION * e[2] {
        SweepVerdict::Unresolved(Unresolved::ReferenceTooCoarse {
            self_check: sc,
            error: e[2],
        })
    } else if grid.max(all) > EDGE_RATIO * grid.min(all) {
        SweepVerdict::Unresolved(Unresolved::EdgeDominated { grid, all })
    } else {
        classify(e, graded.fit)
    };
    Ok(Evaluated {
        graded,
        convergence,
        self_check: sc,
        refined,
    })
}

/// Validate `netlist_path` at `base_rate`, twice and four times it (and
/// eight times when three rates are ambiguous), with oversampling off, and
/// classify the deck. The comparison window starts after `settle_s`.
#[allow(clippy::too_many_arguments)]
pub fn rate_sweep(
    netlist_path: &Path,
    stimulus: AnalyticStimulus,
    duration: f64,
    settle_s: f64,
    base_rate: f64,
    output_node: &str,
    config: &ComparisonConfig,
    options: &ValidationOptions,
) -> Result<RateSweep, ValidationError> {
    let run = |factor: usize, reference_bound: Option<f64>| -> Result<Run, ValidationError> {
        let fs = base_rate * factor as f64;
        let n = (duration * fs) as usize;
        let input: Vec<f64> = (0..n).map(|i| stimulus.at(i as f64 / fs)).collect();
        let opts = ValidationOptions {
            analytic_stimulus: Some(stimulus),
            oversampling: 1,
            generate_html_on_failure: false,
            generate_csv: false,
            reference_bound: reference_bound.or(options.reference_bound),
            ..options.clone()
        };
        validate_circuit_with_options(netlist_path, &input, fs, output_node, config, &opts)
            .map(|r| (factor, fs, r))
    };
    let mut runs = vec![run(1, None)?, run(2, None)?, run(4, None)?];
    let mut evaluated = evaluate(&mut runs, &run, base_rate, settle_s)?;
    if matches!(
        evaluated.convergence,
        SweepVerdict::Unresolved(
            Unresolved::PreAsymptotic { .. } | Unresolved::FloorAmbiguous { .. }
        )
    ) {
        // Ambiguous at three rates: a fourth, and the three finest decide.
        runs.push(run(8, None)?);
        evaluated = evaluate(&mut runs, &run, base_rate, settle_s)?;
    }
    let Evaluated {
        graded,
        convergence,
        self_check,
        refined,
    } = evaluated;
    let n = graded.rows.len();
    let e = [
        graded.rows[n - 3].error,
        graded.rows[n - 2].error,
        graded.rows[n - 1].error,
    ];
    let verdict = if graded.rows[0].passed {
        SweepVerdict::Pass
    } else {
        convergence.clone()
    };
    Ok(RateSweep {
        rows: graded.rows,
        verdict,
        convergence,
        model_error_metric: metric_asymptote(e),
        reference_self_check: self_check,
        reference_refined: refined,
        finest_grid_error: graded.finest_grid,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// y_k = truth + model + disc·h_k²: the waveform fit recovers order 2
    /// and the model error exactly, however the two errors correlate.
    #[test]
    fn the_waveform_fit_recovers_order_and_model_error() {
        let n = 1000;
        let truth: Vec<f64> = (0..n).map(|i| (i as f64 * 0.01).sin()).collect();
        let model: Vec<f64> = (0..n).map(|i| 0.002 * (i as f64 * 0.013).cos()).collect();
        let disc: Vec<f64> = (0..n).map(|i| 0.03 * (i as f64 * 0.007).sin()).collect();
        let y = |h: f64| -> Vec<f64> {
            (0..n)
                .map(|i| truth[i] + model[i] + disc[i] * h * h)
                .collect()
        };
        let (order, y_inf) = extrapolate(&y(1.0), &y(0.5), &y(0.25)).unwrap();
        assert!((order - 2.0).abs() < 1e-9, "{order}");
        let got = rel_rms(&y_inf, &truth);
        let with_model: Vec<f64> = truth.iter().zip(&model).map(|(t, m)| t + m).collect();
        let want = rel_rms(&with_model, &truth);
        assert!((got - want).abs() < 1e-9, "{got} vs {want}");
    }

    #[test]
    fn a_stopped_error_is_a_plateau_and_a_rise_diverges() {
        assert!(matches!(
            classify([0.02, 0.019, 0.0185], Some((1.0, 0.018))),
            SweepVerdict::Plateau { .. }
        ));
        assert!(matches!(
            classify([0.02, 0.03, 0.01], None),
            SweepVerdict::Diverges
        ));
        assert!(matches!(
            classify([0.0301, 0.0076, 0.001975], Some((2.0, 0.0001))),
            SweepVerdict::Converges { .. }
        ));
    }

    /// An error still falling 2.7x per halving is not a plateau, whatever
    /// floor a three-point fit puts under it.
    #[test]
    fn a_falling_error_with_a_high_fitted_floor_is_unresolved_not_a_plateau() {
        match classify([0.0237, 0.00744, 0.00276], Some((1.47, 0.0014))) {
            SweepVerdict::Unresolved(Unresolved::FloorAmbiguous { .. }) => {}
            v => panic!("{v:?}"),
        }
    }

    #[test]
    fn disagreeing_ratios_are_pre_asymptotic() {
        match classify([0.04, 0.01, 0.005], Some((1.0, 0.0))) {
            SweepVerdict::Unresolved(Unresolved::PreAsymptotic { ratios }) => {
                assert!((ratios[0] - 4.0).abs() < 1e-12 && (ratios[1] - 2.0).abs() < 1e-12)
            }
            v => panic!("{v:?}"),
        }
    }

    #[test]
    fn the_metric_cross_check_is_aitken() {
        let m = metric_asymptote([0.0301, 0.0076, 0.001975]).unwrap();
        assert!((m - 0.0001).abs() < 1e-9, "{m}");
        assert!(metric_asymptote([0.02, 0.03, 0.01]).is_none());
    }

    fn row(fs: f64, e: f64) -> RateRow {
        RateRow {
            sample_rate: fs,
            error: e,
            own_error: e,
            passed: false,
            integrator: "trapezoidal",
        }
    }

    #[test]
    fn the_rate_for_a_tolerance_follows_the_fit() {
        let fit = SweepVerdict::Converges {
            order: 2.0,
            model_error: 0.0001,
            fit: Fit::Waveform,
        };
        let sweep = RateSweep {
            rows: vec![
                row(48000.0, 0.0301),
                row(96000.0, 0.0076),
                row(192000.0, 0.001975),
            ],
            verdict: fit.clone(),
            convergence: fit,
            model_error_metric: Some(0.0001),
            reference_self_check: 0.0,
            reference_refined: false,
            finest_grid_error: 0.001975,
        };
        // 1 %: between 48 and 96 kHz, from the 48 kHz row.
        let RateFor::At(fs) = sweep.rate_for(0.01) else {
            panic!("{:?}", sweep.rate_for(0.01))
        };
        assert!(
            (fs - 48000.0 * (0.03 / 0.0099f64).sqrt()).abs() < 1.0,
            "{fs}"
        );
        // 0.1 %: past the finest row, from it.
        let RateFor::At(fs) = sweep.rate_for(0.001) else {
            panic!("{:?}", sweep.rate_for(0.001))
        };
        assert!(
            (fs - 192000.0 * (0.001875 / 0.0009f64).sqrt()).abs() < 1.0,
            "{fs}"
        );
        assert_eq!(
            sweep.rate_for(0.00005),
            RateFor::Unreachable,
            "below the model error"
        );
        let coarse = RateSweep {
            reference_self_check: 0.0005,
            ..sweep
        };
        assert_eq!(coarse.rate_for(0.001), RateFor::Unresolved);
        assert!(matches!(coarse.rate_for(0.01), RateFor::At(_)));
    }

    /// Pre-asymptotic in the fit's sense only: no waveform fit, but the
    /// ratios agree, so the metric fit carries the verdict.
    #[test]
    fn without_a_waveform_fit_the_metric_fit_decides() {
        match classify([0.009185, 0.000763, 0.00007], None) {
            SweepVerdict::Converges { fit, order, .. } => {
                assert_eq!(fit, Fit::ErrorMetric);
                assert!(order > 3.0, "{order}");
            }
            v => panic!("{v:?}"),
        }
    }
}
