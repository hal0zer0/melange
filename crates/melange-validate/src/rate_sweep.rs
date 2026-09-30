//! Validate across sample rates, to separate discretization from model error.
//!
//! A deck that fails at its rate may be modelled exactly and merely
//! integrated coarsely: its error then falls at the integration scheme's
//! order as the step shrinks, toward the reference. One whose error stops
//! falling, or rises, converges to something other than the reference: a
//! model or harness mismatch. [`rate_sweep`] runs `validate` at `fs`, `2·fs`
//! and `4·fs` (oversampling off, the analytic stimulus sampled at each rate)
//! and classifies the deck:
//!
//! - `Pass` at the requested rate;
//! - `Converges`: the error falls at close to the scheme's order. The
//!   Richardson-extrapolated asymptotic error (the MODEL error) and the rate
//!   needed for a tolerance are reported;
//! - `Plateau` / `Diverges`: the error stops falling or rises.
//!
//! The order estimate is the ratio of successive error differences,
//! `p = log2((e1 − e2)/(e2 − e4))`, and the asymptote
//! `e∞ = e4 − (e2 − e4)/(2^p − 1)` (Aitken on the three errors; clamped at
//! 0). A constant floor cancels in the differences, so the order cannot see
//! one; the asymptote does: a floor at least half the finest error means the
//! error has stopped falling (a plateau). The order is reported against the
//! scheme's (2 trapezoidal, 1 backward Euler); a deck still well below it at
//! these rates is pre-asymptotic, not a plateau.

use std::path::Path;

use crate::{
    validate_circuit_with_options, AnalyticStimulus, ComparisonConfig, ValidationError,
    ValidationOptions,
};

/// One rate of a sweep.
#[derive(Debug, Clone)]
pub struct RateRow {
    pub sample_rate: f64,
    /// Normalized RMS error against the reference.
    pub error: f64,
    /// Whether the comparison's gates passed at this rate.
    pub passed: bool,
    /// The integrator the build used here.
    pub integrator: &'static str,
}

/// How a deck's error behaves with the step.
#[derive(Debug, Clone)]
pub enum SweepVerdict {
    /// Passes at the requested (lowest) rate.
    Pass,
    /// Falls at close to the scheme's order toward the reference.
    Converges {
        /// Measured order.
        order: f64,
        /// The asymptotic (model) error.
        model_error: f64,
    },
    /// Stops falling: converges to something other than the reference.
    Plateau { order: f64 },
    /// Rises with the rate.
    Diverges,
}

/// A sweep's rows and verdict.
#[derive(Debug, Clone)]
pub struct RateSweep {
    pub rows: Vec<RateRow>,
    pub verdict: SweepVerdict,
}

impl RateSweep {
    /// The sample rate at which the error falls to `tol`, extrapolated from
    /// the fitted convergence; `None` if the model error is at or above it
    /// (no rate reaches it) or the deck does not converge. The requested
    /// rate itself when it already passes the tolerance.
    pub fn rate_for(&self, tol: f64) -> Option<f64> {
        let base = self.rows.first()?;
        if base.error <= tol {
            return Some(base.sample_rate);
        }
        match self.verdict {
            SweepVerdict::Converges { order, model_error } if model_error < tol => {
                let factor = ((base.error - model_error) / (tol - model_error)).powf(1.0 / order);
                Some(base.sample_rate * factor.max(1.0))
            }
            _ => None,
        }
    }
}

/// Classify three errors at `fs`, `2fs`, `4fs`.
pub fn classify(e: [f64; 3]) -> SweepVerdict {
    let [e1, e2, e4] = e;
    if e4 > e2 * 1.05 || e2 > e1 * 1.05 {
        return SweepVerdict::Diverges;
    }
    if !(e1 > e2 && e2 > e4) {
        return SweepVerdict::Plateau { order: 0.0 };
    }
    let order = ((e1 - e2) / (e2 - e4)).log2();
    if !(order.is_finite() && order > 0.0) {
        return SweepVerdict::Plateau { order: 0.0 };
    }
    let model_error = (e4 - (e2 - e4) / (2f64.powf(order) - 1.0)).max(0.0);
    if model_error >= 0.5 * e4 {
        return SweepVerdict::Plateau { order };
    }
    SweepVerdict::Converges { order, model_error }
}

/// Validate `netlist_path` at `base_rate`, twice and four times it, with
/// oversampling off, and classify the deck. The first row's `passed` is the
/// ordinary validate verdict at the requested rate.
pub fn rate_sweep(
    netlist_path: &Path,
    stimulus: AnalyticStimulus,
    duration: f64,
    base_rate: f64,
    output_node: &str,
    config: &ComparisonConfig,
    options: &ValidationOptions,
) -> Result<RateSweep, ValidationError> {
    let mut rows = Vec::new();
    for factor in [1.0, 2.0, 4.0] {
        let fs = base_rate * factor;
        let n = (duration * fs) as usize;
        let input: Vec<f64> = (0..n).map(|i| stimulus.at(i as f64 / fs)).collect();
        let opts = ValidationOptions {
            analytic_stimulus: Some(stimulus),
            oversampling: 1,
            generate_html_on_failure: false,
            generate_csv: false,
            ..options.clone()
        };
        let result =
            validate_circuit_with_options(netlist_path, &input, fs, output_node, config, &opts)?;
        rows.push(RateRow {
            sample_rate: fs,
            error: result.report.normalized_rms_error,
            passed: result.report.passed,
            integrator: result.integrator,
        });
    }
    let verdict = if rows[0].passed {
        SweepVerdict::Pass
    } else {
        classify([rows[0].error, rows[1].error, rows[2].error])
    };
    Ok(RateSweep { rows, verdict })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_second_order_fall_converges_to_its_asymptote() {
        // e = 0.01 % + 3 % * (fs0/fs)^2
        let e = [0.0301, 0.0076, 0.001975];
        match classify(e) {
            SweepVerdict::Converges { order, model_error } => {
                assert!((order - 2.0).abs() < 1e-9, "{order}");
                assert!((model_error - 0.0001).abs() < 1e-9, "{model_error}");
            }
            v => panic!("{v:?}"),
        }
    }

    #[test]
    fn a_floor_is_a_plateau_and_a_rise_diverges() {
        assert!(matches!(
            classify([0.02, 0.019, 0.0185]),
            SweepVerdict::Plateau { .. }
        ));
        assert!(matches!(
            classify([0.02, 0.03, 0.01]),
            SweepVerdict::Diverges
        ));
    }

    #[test]
    fn the_rate_for_a_tolerance_follows_the_fit() {
        let sweep = RateSweep {
            rows: vec![
                RateRow {
                    sample_rate: 48000.0,
                    error: 0.0301,
                    passed: false,
                    integrator: "trapezoidal",
                },
                RateRow {
                    sample_rate: 96000.0,
                    error: 0.0076,
                    passed: false,
                    integrator: "trapezoidal",
                },
                RateRow {
                    sample_rate: 192000.0,
                    error: 0.001975,
                    passed: false,
                    integrator: "trapezoidal",
                },
            ],
            verdict: classify([0.0301, 0.0076, 0.001975]),
        };
        // 3 % * (48k/fs)^2 = 0.99 % -> fs = 48k * sqrt(3/0.99)
        let fs = sweep.rate_for(0.01).unwrap();
        assert!(
            (fs - 48000.0 * (0.03 / 0.0099f64).sqrt()).abs() < 1.0,
            "{fs}"
        );
        assert!(sweep.rate_for(0.00005).is_none(), "below the model error");
    }
}
