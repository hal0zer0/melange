//! Validate across sample rates, to separate discretization from model error.
//!
//! A deck that fails at its rate may be modelled exactly and merely
//! integrated coarsely: its error then falls with the step toward the
//! reference. One whose error stops falling, or rises, converges to something
//! other than the reference: a model or harness mismatch. [`rate_sweep`]
//! renders the deck at `fs`, `2·fs` and `4·fs` (oversampling off, the
//! analytic stimulus sampled at each rate) and classifies it:
//!
//! - `Pass` at the requested rate;
//! - `Converges`: the error falls toward the reference. The asymptotic
//!   (MODEL) error and the rate needed for a tolerance are reported;
//! - `Plateau` / `Diverges`: the error stops falling or rises.
//!
//! **One reference, the error extrapolated.** Every render is compared, at
//! the instants the three rates share (the `fs` grid: every second and
//! fourth sample of the finer renders, so no interpolation), against ONE
//! reference: ngspice's output at the finest rate, taken at the render's
//! rate and DC-blocked there as the render is. A reference per rate is a
//! different object at each rate (each one resampled to and aligned against
//! its own render), which confounded the per-rate comparison on a
//! near-square deck. The asymptote is extrapolated on the error waveform
//! `e_k = y_k − ref_k`, not on the error metric (RMS errors add in
//! quadrature when uncorrelated, so a metric fit is biased exactly where a
//! floor exists):
//!
//! ```text
//! r   = Σ (e1 − e2)(e2 − e4) / Σ (e2 − e4)²     (least squares), p = log2 r
//! e∞  = e4 + (e4 − e2)/(2^p − 1)
//! model error = ‖e∞‖ / ‖ref‖
//! ```
//!
//! The error-metric fit (Aitken on the three errors) is kept as a
//! cross-check; a material disagreement between the two flags a floor. On a
//! deck still short of its asymptotic range the waveform fit has no answer
//! (its successive differences do not shrink by a common ratio), and the
//! verdict falls back to the metric fit and says so.
//!
//! **The reference must resolve what it grades.** Every reference has
//! passed its own convergence ladder (`reference`), and the finest one must
//! also be accurate to a third of the smallest error it grades (the `4fs`
//! render's). When it is not, it is refined once to that bound; if it still
//! is not, the convergence is `Unresolved`, not classified.

use std::path::Path;

use crate::{
    validate_circuit_with_options, AnalyticStimulus, ComparisonConfig, ValidationError,
    ValidationOptions, ValidationResult,
};

/// One rate of a sweep.
#[derive(Debug, Clone)]
pub struct RateRow {
    pub sample_rate: f64,
    /// Normalized RMS error of this render against the finest reference, at
    /// the common instants.
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

/// Which fit a convergence verdict rests on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fit {
    /// Extrapolated on the waveform.
    Waveform,
    /// The waveform fit had no answer (pre-asymptotic); Aitken on the three
    /// error metrics.
    ErrorMetric,
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
    /// The finest reference's own error is not below
    /// `RESOLUTION_FRACTION` of the smallest error graded, so the errors'
    /// behaviour is not measured.
    Unresolved {
        /// The reference's self-check.
        self_check: f64,
        /// The smallest error graded.
        error: f64,
    },
}

/// The rate a tolerance needs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RateFor {
    /// This rate.
    At(f64),
    /// No rate: the model error is at or above the tolerance, or the deck
    /// does not converge.
    Unreachable,
    /// The reference is not accurate enough to tell.
    Unresolved,
}

/// A sweep's rows, fits and verdict.
#[derive(Debug, Clone)]
pub struct RateSweep {
    pub rows: Vec<RateRow>,
    pub verdict: SweepVerdict,
    /// The classification of the three errors whether or not the requested
    /// rate passed: what the rate for a tolerance is read from.
    pub convergence: SweepVerdict,
    /// Cross-check: the asymptote from the error metrics (Aitken), `None`
    /// where the errors do not fall monotonically.
    pub model_error_metric: Option<f64>,
    /// The finest reference's own self-check (normalized RMS against its
    /// next-coarser rung).
    pub reference_self_check: f64,
    /// Whether the finest reference was refined past validate's own bound
    /// to resolve the errors graded.
    pub reference_refined: bool,
}

impl RateSweep {
    /// The sample rate at which the error falls to `tol`, from the fit. The
    /// requested rate itself when it already meets `tol`. `Unresolved` when
    /// the reference's own error is not below `RESOLUTION_FRACTION` of
    /// `tol`.
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
                let factor = ((base.error - model_error) / (tol - model_error)).powf(1.0 / order);
                RateFor::At(base.sample_rate * factor.max(1.0))
            }
            SweepVerdict::Unresolved { .. } => RateFor::Unresolved,
            _ => RateFor::Unreachable,
        }
    }
}

/// Aitken asymptote of three errors at `fs`, `2fs`, `4fs` (the cross-check).
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

/// Classify from the errors against the finest reference and the waveform
/// fit `(order, model error)`. Without a waveform fit, falls back to the
/// metric fit.
pub fn classify(e: [f64; 3], waveform: Option<(f64, f64)>) -> SweepVerdict {
    let [e1, e2, e4] = e;
    if e4 > e2 * 1.05 || e2 > e1 * 1.05 {
        return SweepVerdict::Diverges;
    }
    let (order, model_error, fit) = match (waveform, metric_asymptote(e)) {
        (Some((order, model_error)), _) => (order, model_error, Fit::Waveform),
        (None, Some(model_error)) => (
            ((e1 - e2) / (e2 - e4)).log2(),
            model_error,
            Fit::ErrorMetric,
        ),
        (None, None) => return SweepVerdict::Plateau { order: 0.0 },
    };
    if !(e1 > e2 && e2 > e4) || model_error >= 0.5 * e4 {
        return SweepVerdict::Plateau { order };
    }
    SweepVerdict::Converges {
        order,
        model_error,
        fit,
    }
}

/// The errors of three renders against one reference, and the waveform fit.
struct Graded {
    rows: Vec<RateRow>,
    fit: Option<(f64, f64)>,
}

/// Validate `netlist_path` at `base_rate`, twice and four times it, with
/// oversampling off, and classify the deck. The comparison window starts
/// after `settle_s`.
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
    let run = |factor: usize, reference_bound: Option<f64>| {
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
    let self_check = |runs: &[(usize, f64, ValidationResult)]| {
        runs[2].2.report.reference_self_check.unwrap_or(0.0)
    };
    let mut graded = grade(&runs, base_rate, settle_s)?;
    let mut reference_refined = false;
    let e4 = graded.rows[2].error;
    if self_check(&runs) > RESOLUTION_FRACTION * e4 {
        // Refine the finest reference to the error it must resolve. A ladder
        // that cannot reach it leaves the convergence unresolved.
        match run(4, Some(RESOLUTION_FRACTION * e4)) {
            Ok(finer) => {
                runs[2] = finer;
                graded = grade(&runs, base_rate, settle_s)?;
                reference_refined = true;
            }
            Err(ValidationError::ReferenceNotConverged(_)) => {}
            Err(e) => return Err(e),
        }
    }
    let reference_self_check = self_check(&runs);
    let e = [
        graded.rows[0].error,
        graded.rows[1].error,
        graded.rows[2].error,
    ];
    let convergence = if reference_self_check > RESOLUTION_FRACTION * e[2] {
        SweepVerdict::Unresolved {
            self_check: reference_self_check,
            error: e[2],
        }
    } else {
        classify(e, graded.fit)
    };
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
        reference_self_check,
        reference_refined,
    })
}

/// Grade the three renders against the finest run's reference at the
/// common instants.
fn grade(
    runs: &[(usize, f64, ValidationResult)],
    base_rate: f64,
    settle_s: f64,
) -> Result<Graded, ValidationError> {
    // The finest reference, at each render's rate and DC-blocked there, as
    // the render was by its own blocker. One blocker for every rate would
    // leave the blocker's own discretization (first order in the step: its
    // gain at 1 kHz is 3e-4 low at 48 kHz, 8e-5 at 192 kHz) in the errors.
    let finest = &runs[2].2;
    let references: Vec<Vec<f64>> = runs
        .iter()
        .map(|(_, fs, _)| {
            let step = (finest.reference_rate / fs).round() as usize;
            let mut r: Vec<f64> = finest.reference_raw.iter().step_by(step).copied().collect();
            crate::dc_block_signal(&mut r, *fs);
            r
        })
        .collect();
    // The common instants: the fs grid, after the settle window, where every
    // render and reference has a sample.
    let start = (settle_s * base_rate).ceil() as usize;
    let len = runs
        .iter()
        .zip(&references)
        .map(|((f, _, r), reference)| r.melange_raw.len().min(reference.len()) / f)
        .min()
        .unwrap_or(0);
    if len <= start + 16 {
        return Err(ValidationError::InvalidInput(
            "rate sweep: no common window after the settle time".to_string(),
        ));
    }
    let at = |v: &[f64], step: usize| -> Vec<f64> { (start..len).map(|i| v[i * step]).collect() };
    let reference = at(&references[2], 4);
    // Each render's error waveform against its own-rate reference.
    let errors: Vec<Vec<f64>> = runs
        .iter()
        .zip(&references)
        .map(|((f, _, r), rf)| {
            at(&r.melange_raw, *f)
                .iter()
                .zip(at(rf, *f))
                .map(|(y, x)| y - x)
                .collect()
        })
        .collect();
    let reference_rms = rms(&reference).max(1e-300);
    let norm = |e: &[f64]| rms(e) / reference_rms;
    let rows: Vec<RateRow> = runs
        .iter()
        .zip(&errors)
        .map(|((_, fs, r), e)| RateRow {
            sample_rate: *fs,
            error: norm(e),
            own_error: r.report.normalized_rms_error,
            passed: r.report.passed,
            integrator: r.integrator,
        })
        .collect();
    let fit = extrapolate(&errors[0], &errors[1], &errors[2]).map(|(p, e_inf)| (p, norm(&e_inf)));
    Ok(Graded { rows, fit })
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
    fn a_floor_is_a_plateau_and_a_rise_diverges() {
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

    #[test]
    fn the_metric_cross_check_is_aitken() {
        let m = metric_asymptote([0.0301, 0.0076, 0.001975]).unwrap();
        assert!((m - 0.0001).abs() < 1e-9, "{m}");
        assert!(metric_asymptote([0.02, 0.03, 0.01]).is_none());
    }

    #[test]
    fn the_rate_for_a_tolerance_follows_the_fit() {
        let row = |fs: f64, e: f64| RateRow {
            sample_rate: fs,
            error: e,
            own_error: e,
            passed: false,
            integrator: "trapezoidal",
        };
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
        };
        let RateFor::At(fs) = sweep.rate_for(0.01) else {
            panic!("{:?}", sweep.rate_for(0.01))
        };
        assert!(
            (fs - 48000.0 * (0.03 / 0.0099f64).sqrt()).abs() < 1.0,
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

    /// Pre-asymptotic: no waveform fit, but the errors fall monotonically
    /// and faster than they will, so the metric fit carries the verdict.
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
