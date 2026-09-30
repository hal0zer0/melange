//! Whether the ngspice reference is converged.
//!
//! The reference is a numerical solution too: ngspice's trapezoidal rule
//! with its own step control. At its default maximum step (the output step)
//! and a smooth analytic drive, it integrated with the same step as the
//! melange render it judged, and its error was charged to melange: 2.4 %
//! and 11.8 % at 48 kHz on two tube decks whose converged references put
//! melange at 0.216 % and 0.156 %. So no reference is used until it has shown
//! it is converged, in both of the parameters that set its accuracy: the
//! maximum internal step and the relative tolerance ngspice's step control
//! works to. From a starting point, the step is halved and the tolerance
//! tightened tenfold in turn; the reference is accepted when both
//! refinements move it by no more than a bound ([`SETTLED_FRACTION`] of the
//! tolerance graded, in validate). Halving the step alone is not enough:
//! where ngspice's own error control already keeps its steps shorter than
//! the maximum, halving the maximum changes nothing, and two identical runs
//! would "agree" at whatever error the tolerance allows. A reference that
//! runs out of refinements refuses the verdict. The figure it settled at is
//! reported next to melange's error.

use crate::comparison::ComparisonReport;
use crate::spice_runner::{ReferenceStep, SpiceData, SpiceError, REFERENCE_STEP_DIVISOR};
use crate::ValidationError;

/// Two consecutive rungs agree when their difference (normalized RMS, the
/// measure the verdict grades) is at most this fraction of the tolerance.
/// Stated, not derived: a reference 10x more accurate than the gate cannot
/// move a verdict by more than a tenth of it.
pub const SETTLED_FRACTION: f64 = 0.1;

/// The finest maximum step tried: tstep/256.
const FINEST_TMAX_DIVISOR: f64 = 16.0 * REFERENCE_STEP_DIVISOR;

/// The tightest relative tolerance tried.
const FINEST_RELTOL: f64 = 1e-6;

/// The next refinement of `step` in one parameter, if within the limits.
fn halve_tmax(step: ReferenceStep) -> Option<ReferenceStep> {
    (step.tmax_divisor < FINEST_TMAX_DIVISOR).then_some(ReferenceStep {
        tmax_divisor: 2.0 * step.tmax_divisor,
        ..step
    })
}

fn tighten_reltol(step: ReferenceStep) -> Option<ReferenceStep> {
    (step.reltol > FINEST_RELTOL * 1.5).then_some(ReferenceStep {
        reltol: step.reltol / 10.0,
        ..step
    })
}

/// How the reference showed it was converged.
#[derive(Debug, Clone, PartialEq)]
pub struct ReferenceConvergence {
    /// The refinement the reference was taken at.
    pub step: ReferenceStep,
    /// The larger of the two differences (normalized RMS) that accepted it:
    /// halving the step, and tightening the tolerance.
    pub self_check: f64,
    /// The bound that difference met: [`SETTLED_FRACTION`] of the tolerance.
    pub bound: f64,
}

impl ReferenceConvergence {
    /// The self-check, the rung it was taken at, and its bound.
    pub fn note(&self) -> String {
        format!(
            "reference self-check {:.4} % (at {}, against half the step and a tenth of the \
             tolerance; bound {:.4} %)",
            self.self_check * 100.0,
            self.step,
            self.bound * 100.0
        )
    }

    /// Put it on `report`, where the summary prints it next to the error.
    pub fn attach(&self, report: &mut ComparisonReport) {
        report.reference_self_check = Some(self.self_check);
        report.reference_self_check_note = Some(self.note());
    }
}

/// Normalized RMS difference of `a` from `b`, both DC-blocked at `rate` as
/// the graded reference is, after `settle_s`. Zero when `b` is silent: there
/// is then nothing for a step to be wrong about, and the comparison's own
/// silent-reference gate decides the verdict.
pub fn normalized_difference(a: &[f64], b: &[f64], rate: f64, settle_s: f64) -> f64 {
    let n = a.len().min(b.len());
    let mut a = a[..n].to_vec();
    let mut b = b[..n].to_vec();
    crate::dc_block_signal(&mut a, rate);
    crate::dc_block_signal(&mut b, rate);
    let skip = ((settle_s * rate).round() as usize).min(n);
    let (a, b) = (&a[skip..], &b[skip..]);
    let rms = |x: &mut dyn Iterator<Item = f64>| {
        let (s, k) = x.fold((0.0, 0usize), |(s, k), v| (s + v * v, k + 1));
        if k == 0 {
            0.0
        } else {
            (s / k as f64).sqrt()
        }
    };
    let ref_rms = rms(&mut b.iter().copied());
    if ref_rms <= 1e-12 {
        return 0.0;
    }
    rms(&mut a.iter().zip(b).map(|(x, y)| x - y)) / ref_rms
}

/// The reference on `output_node`, refined from `ReferenceStep::default()`
/// until halving its maximum step and tightening its tolerance tenfold each
/// move it by at most `bound` (normalized RMS), with its convergence. `run`
/// simulates the reference at a refinement. Refuses when the limits are
/// reached first. validate's bound is `SETTLED_FRACTION` of its RMS
/// tolerance.
pub fn converged_reference<F>(
    mut run: F,
    output_node: &str,
    settle_s: f64,
    bound: f64,
) -> Result<(SpiceData, ReferenceConvergence), ValidationError>
where
    F: FnMut(ReferenceStep) -> Result<SpiceData, SpiceError>,
{
    let diff = |a: &SpiceData, b: &SpiceData| -> Result<f64, ValidationError> {
        Ok(normalized_difference(
            a.get_node_voltage(output_node)?,
            b.get_node_voltage(output_node)?,
            a.sample_rate,
            settle_s,
        ))
    };
    let mut tried = Vec::new();
    let mut step = ReferenceStep::default();
    let mut current = run(step)?;
    // A refinement ngspice cannot run (it abandons the transient at a tight
    // tolerance) is the end of the refinements, not a failed validation.
    let mut refine = |step: ReferenceStep, tried: &mut Vec<String>| {
        run(step).map_err(|e| {
            tried.push(format!("{step}: ngspice could not run it ({e})"));
        })
    };
    loop {
        // Halve the step; where that settles, tighten the tolerance.
        let (next, d_step) = match halve_tmax(step) {
            Some(finer) => {
                let Ok(data) = refine(finer, &mut tried) else {
                    break;
                };
                let d = diff(&data, &current)?;
                tried.push(format!("{finer}: {:.4} %", d * 100.0));
                (Some((finer, data)), Some(d))
            }
            None => (None, None),
        };
        if d_step.is_some_and(|d| d > bound) {
            let (finer, data) = next.expect("a difference implies a run");
            step = finer;
            current = data;
            continue;
        }
        // The step settled, or can go no finer: the tolerance, from the
        // finest run so far.
        let (base_step, base) = match next {
            Some((finer, data)) => (finer, data),
            None => (step, current),
        };
        let Some(tighter) = tighten_reltol(base_step) else {
            break;
        };
        let Ok(data) = refine(tighter, &mut tried) else {
            break;
        };
        let d_tol = diff(&data, &base)?;
        tried.push(format!("{tighter}: {:.4} %", d_tol * 100.0));
        if d_tol <= bound && d_step.is_some() {
            return Ok((
                data,
                ReferenceConvergence {
                    step: tighter,
                    self_check: d_step.unwrap_or(0.0).max(d_tol),
                    bound,
                },
            ));
        }
        if d_tol <= bound {
            // The step is at its limit and did not settle; a settled
            // tolerance does not make up for it.
            break;
        }
        step = tighter;
        current = data;
    }
    Err(ValidationError::ReferenceNotConverged(format!(
        "the ngspice reference is not converged: refining it still moves it by more than the \
         {:.4} % it must settle to ({}). No verdict is given against a reference whose own error \
         is that large.",
        bound * 100.0,
        tried.join("; ")
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fake reference whose error is `e_step(divisor) + e_tol(reltol)`,
    /// a constant offset on a sine.
    fn fake<'a>(
        e: impl Fn(ReferenceStep) -> f64 + 'a,
        log: &'a std::cell::RefCell<Vec<ReferenceStep>>,
    ) -> impl FnMut(ReferenceStep) -> Result<SpiceData, SpiceError> + 'a {
        move |step| {
            log.borrow_mut().push(step);
            let rate = 48000.0;
            let err = e(step);
            let v: Vec<f64> = (0..4800)
                .map(|k| {
                    (1.0 + err) * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / rate).sin()
                })
                .collect();
            let mut data = SpiceData {
                sample_rate: rate,
                ..SpiceData::default()
            };
            data.voltages.insert("out".to_string(), v);
            Ok(data)
        }
    }

    #[test]
    fn a_step_limited_reference_settles_when_both_refinements_agree() {
        let log = std::cell::RefCell::new(Vec::new());
        // Step error 1e-3 * (16/divisor)^2; tolerance error 1e-7-scale.
        let e = |s: ReferenceStep| 1e-3 * (16.0 / s.tmax_divisor).powi(2) + s.reltol * 1e-3;
        let (_, c) = converged_reference(fake(e, &log), "out", 0.0, 1e-4).unwrap();
        // /16 -> /32 moves 7.5e-4, /32 -> /64 1.9e-4, /64 -> /128 4.7e-5:
        // settled, then the tolerance agrees.
        assert_eq!(c.step.tmax_divisor, 128.0);
        assert!((c.step.reltol - 1e-5).abs() < 1e-12, "{:?}", c.step);
        assert!(c.self_check <= 1e-4);
        assert_eq!(log.borrow().len(), 5);
    }

    /// Where the tolerance is the binding error, halving the step changes
    /// nothing: that agreement alone must not accept the reference.
    #[test]
    fn a_tolerance_limited_reference_is_not_accepted_on_the_step_alone() {
        let log = std::cell::RefCell::new(Vec::new());
        let e = |s: ReferenceStep| s.reltol * 5.0;
        let (_, c) = converged_reference(fake(e, &log), "out", 0.0, 1e-4).unwrap();
        // tstep/32 agrees with tstep/16 exactly, but 1e-5 moves it 4.5e-4;
        // at 1e-5, tstep/64 agrees again and 1e-6 moves it 4.5e-5.
        assert_eq!(c.step.tmax_divisor, 64.0);
        assert!((c.step.reltol - 1e-6).abs() < 1e-15, "{:?}", c.step);
        assert!(c.self_check > 4e-5, "{}", c.self_check);
    }

    #[test]
    fn a_refinement_ngspice_cannot_run_ends_the_refinements() {
        let log = std::cell::RefCell::new(Vec::new());
        let mut inner = fake(|s: ReferenceStep| 1.0 / s.tmax_divisor, &log);
        let run = |s: ReferenceStep| {
            if s.tmax_divisor > 32.0 {
                Err(SpiceError::SimulationFailed("timestep too small".into()))
            } else {
                inner(s)
            }
        };
        match converged_reference(run, "out", 0.0, 1e-4) {
            Err(ValidationError::ReferenceNotConverged(msg)) => {
                assert!(msg.contains("timestep too small"), "{msg}")
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_reference_that_never_settles_is_refused() {
        let log = std::cell::RefCell::new(Vec::new());
        let e = |s: ReferenceStep| 1.0 / s.tmax_divisor;
        let err = converged_reference(fake(e, &log), "out", 0.0, 1e-4).unwrap_err();
        assert!(
            matches!(err, ValidationError::ReferenceNotConverged(_)),
            "{err}"
        );
    }

    #[test]
    fn the_difference_is_normalized_to_the_reference() {
        let rate = 48000.0;
        let b: Vec<f64> = (0..4800)
            .map(|k| (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / rate).sin())
            .collect();
        let a: Vec<f64> = b.iter().map(|v| 1.01 * v).collect();
        let d = normalized_difference(&a, &b, rate, 0.02);
        assert!((d - 0.01).abs() < 1e-4, "{d}");
        assert_eq!(normalized_difference(&a, &vec![0.0; 4800], rate, 0.0), 0.0);
    }
}
