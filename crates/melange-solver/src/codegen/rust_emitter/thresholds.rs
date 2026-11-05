//! The certification thresholds of the generated solvers.
//!
//! Each threshold the generated code accepts a Newton iterate by is ONE
//! constant here, read by the emitter where the test is written into the
//! generated code and by the tests and documentation that quote it, so the
//! emitted code and its documentation cannot drift. None is configurable:
//! the `--tolerance` flag that used to emit a `TOL` constant was removed
//! (ruled 2026-11-03) because no certification test read it (its only reader
//! was the sub-step ladder's acceptance, where a 1 mV absolute term swamped
//! its 1e-9 default), and one number cannot express SPICE's RELTOL/VNTOL/
//! ABSTOL triple.
//!
//! The generated code carries the transient constants under the same names,
//! at module level on both routes (`emit_transient_thresholds`); the DC-solve
//! trio is the compile-time solver's (`crate::dc_op`), emitted function-locally
//! into `recompute_dc_op` as `DC_OP_RELTOL` / `DC_OP_TOL` / `DC_OP_KCL_ABSTOL`.
//! The emitted expressions use the names with the same literal values, so the
//! generated code computes what it computed with the literals.

pub use crate::dc_op::{DC_OP_KCL_ABSTOL_AMPS, DC_OP_RELTOL, DC_OP_STEP_ABSTOL_VOLTS};

/// Relative term of every transient certification test, dimensionless:
/// SPICE's RELTOL. The Newton node-step test, the per-row KCL residual gate
/// and the device-current consistency check all scale their tolerance by it
/// (on purpose: one RELTOL, three absolute floors in each test's unit).
/// Ruled 2026-11-03 to be named; value is SPICE's default, unchanged since
/// the first step test was written.
pub const RELTOL: f64 = 1e-3;

/// Absolute floor, in volts, of the Newton node-step test on the UNDAMPED
/// correction, `|v_new − v| < RELTOL·max(|v|, |v_new|) + VNTOL` per device
/// node (SPICE's VNTOL). Also the floor of the residual helpers' source /
/// constraint / inductor-branch rows, whose unit is volts. Ruled 2026-11-03 to
/// be named; value is SPICE's default, unchanged since the first step test
/// was written. The test's placement on the undamped correction is the
/// certification ruled 2026-11-01.
pub const VNTOL: f64 = 1e-6;

/// Absolute floor, in amps, of the per-row KCL residual gate,
/// `|F_i| <= RELTOL·den_i + KCL_ABSTOL` on every KCL row (`den_i` the row's
/// largest term), and the chord exit step's test on the KCL infinity norm.
/// Ruled 2026-11-03 to be named; value unchanged since the row-residual gate
/// was introduced, and the same floor as the DC solve's
/// [`DC_OP_KCL_ABSTOL_AMPS`].
pub const KCL_ABSTOL: f64 = 1e-9;

/// Absolute floor, in amps, of the device-current checks' tolerance
/// `RELTOL·max(|a|, |b|, ABSTOL_SCALE_FLOOR) + ABSTOL` per device dimension
/// (SPICE's ABSTOL). Two checks share the tolerance but test different
/// quantities: the DK Newton's current-residual test compares `i_nl` with
/// `i_dev(v_d)` (the Newton residual), the nodal full-LU device
/// current-change check compares the currents after a step with those before
/// it (first order in the step, no Jacobian term). Ruled 2026-11-03 to be named; value is SPICE's
/// default, unchanged since the DK current-residual check was written (the
/// full-LU check copied it).
pub const ABSTOL: f64 = 1e-12;

/// Floor, in amps, of the magnitude the device-current consistency check
/// scales RELTOL by, so a device carrying ~0 A does not demand an unreachable
/// tolerance. Ruled 2026-11-03 to be named; value unchanged since the DK
/// current-residual check was written.
pub const ABSTOL_SCALE_FLOOR: f64 = 1e-9;

/// Absolute acceptance of the nodal sub-step ladder, in volts: a sub-step's
/// Newton iterate is accepted when the largest raw (undamped) node correction
/// is under this AND the row residual gate passes. An absolute 1 mV on every
/// node: looser than the per-sample step test (`RELTOL·|v| + VNTOL`) below
/// ~1 V and tighter above it. Unifying the two would be a separate
/// bit-changing change with its own measurement. Ruled 2026-11-03 to be
/// named; until then the test was written `TOL + 1e-3` with the removed
/// `--tolerance` flag's `TOL` (default 1e-9), so it differed from this value
/// only for a largest correction in [1e-3, 1e-3 + 1e-9) V.
pub const SUBSTEP_ACCEPT_VOLTS: f64 = 1e-3;

/// Relative tolerance of a saturating inductor's flux row, dimensionless:
/// the row residual is accepted when `|acc| <= FLUX_ROW_RELTOL·den`, `den` the
/// row's largest term after the stiff pair `alpha·Φ − rhs` is summed (the
/// per-sample increment, the volts across the winding), floored at
/// `FLUX_ROW_ULP_FLOOR·eps` of the stiff pair. Its own constant, not RELTOL:
/// whatever residual the row accepts is integrated into the flux history
/// with one sign under a DC bias, so it accumulates instead of averaging out;
/// at 1e-3 a 5 mA biased core drifted by −1.6e-5 A over 2 s, at 1e-5 by
/// ~1e-10 A for about one more iteration per sample (the measurement in
/// `sat_ind.rs`). Ruled 2026-11-03 to be named; value unchanged since that
/// measurement.
pub const FLUX_ROW_RELTOL: f64 = 1e-5;

/// Floor of the flux-row tolerance, in units of `f64::EPSILON` times the
/// stiff pair's magnitude: what the pairing cannot remove is rounding in
/// `alpha·Φ − rhs[k]`, a few ulp of the operands, not of their difference,
/// and a fixed absolute floor would reopen the drift at small increments.
/// Ruled 2026-11-03 to be named; value unchanged since the flux-row test was
/// written.
pub const FLUX_ROW_ULP_FLOOR: f64 = 64.0;

/// Slope of the full-LU Newton line search's natural monotonicity test: a
/// trial at fraction `λ` of the Newton correction `d` is accepted when the
/// simplified correction at the trial, `d̄ = G_aug⁻¹·F(v + λ·d)` in the same
/// factorization as the step, satisfies `‖d̄‖_w <= (1 − NATURAL_TEST_SLOPE·λ)·‖d‖_w`.
/// A linear problem gives `‖d̄‖ = (1 − λ)‖d‖` exactly, so the slope is the
/// share of that contraction a trial must keep. 1/4 is recalled as the
/// restricted form of Deuflhard's natural monotonicity test (his NLEQ-ERR
/// codes) and 1/2 as the unrestricted test of his Newton Methods for
/// Nonlinear Problems; both from general knowledge, unread here (2026-11-04),
/// so the witness below, not the citation, carries the constant.
/// Measured on the golden corpus: at 1/2 a fresh-factorization step that
/// contracted the Newton distance to 63 % and the residual sixty-fold was
/// refused on a choke-loaded stage far from its root and then taken anyway
/// (796 such refusals on one render); at 1/4 it is accepted. Applied on
/// iterations whose factorization is the iterate's own; a chord iteration
/// requires non-expansion only (slope 0), since its simplified correction
/// contracts by `1 − λ·k` with the chord's `k` unknown. Not a certification
/// threshold: the test steers the path and certification is the separate
/// three-part test. Ruled 2026-11-04.
pub const NATURAL_TEST_SLOPE: f64 = 0.25;

/// Absolute floor, in amps, of the natural monotonicity test's weight on a
/// branch-current unknown (a source, VCVS, transformer, op-amp or inductor
/// row): `w_i = 1 / (RELTOL·max(|x_i|, |x_new_i|) + NATURAL_TEST_CURRENT_FLOOR)`,
/// the current analogue of the node rows' `VNTOL`. The value is
/// [`KCL_ABSTOL`]'s, the floor the residual gate already applies to currents,
/// stated as its own constant so the branch rows do not inherit a volt floor
/// silently (the node-step test applies `VNTOL` to a saturating inductor's
/// branch row, volts as amps; this test does not). Ruled 2026-11-04.
pub const NATURAL_TEST_CURRENT_FLOOR: f64 = 1e-9;

/// Debug-build invariant of the reused Newton correction, relative: on the
/// iteration after an accepted line-search trial whose factorization is kept,
/// the trial's back-solved residual `G_aug⁻¹·F(v)` is the correction the
/// companion solve `G_aug·v_new = rhs_work` would give, `v_new = v − fv`, and
/// a debug build computes both and requires
/// `max_i |v_new_i − v_cmp_i| <= NATURAL_REUSE_DEBUG_RELTOL · max(1, max_i |v_new_i|)`.
/// The two forms round differently: the companion form solves for the
/// 60 V-scale node voltages from a right-hand side of 10 mA-scale currents
/// that cancel to the nanoamp residual, so its error is the conditioning times
/// `u` on the full voltage, while the correction form solves for the small
/// correction from the residual itself. Measured 2026-11-04 over 10 s of the
/// bench stimulus: 2.4e-9 V on a 60 V FET limiter (4e-11 relative), 3.4e-11 V
/// on a power amp, 9.3e-14 V on a compressor's control path (gravity); a
/// bus compressor had no reused iteration at all (its searches are skipped
/// by the step test). The bound is 250× the largest. Relative to the largest
/// node voltage, it catches a wrong vector, not a near-root rounding
/// difference; the test that drives it holds reused corrections well above
/// the bound. Release builds compile it out (`cfg!(debug_assertions)`).
pub const NATURAL_REUSE_DEBUG_RELTOL: f64 = 1e-8;

/// Emit the transient certification thresholds into the generated code as
/// named module-level constants (both routes). `substep` adds the sub-step
/// ladder's acceptance, which only the nodal route has; `natural` adds the
/// full-LU line search's natural monotonicity constants.
pub(super) fn emit_transient_thresholds(
    code: &mut String,
    substep: bool,
    flux: bool,
    natural: bool,
) {
    if flux {
        code.push_str(
            "/// Relative tolerance of a saturating inductor's flux row,\n\
             /// `|acc| <= FLUX_ROW_RELTOL * den`, `den` the row's largest term after the\n\
             /// stiff pair is summed; floored at FLUX_ROW_ULP_FLOOR ulp of that pair.\n\
             /// Its own constant: the accepted residual integrates into the flux.\n",
        );
        code.push_str(&format!(
            "pub const FLUX_ROW_RELTOL: f64 = {FLUX_ROW_RELTOL:e};\n\n\
             /// Floor of the flux-row tolerance, in ulp of the stiff pair.\n\
             pub const FLUX_ROW_ULP_FLOOR: f64 = {FLUX_ROW_ULP_FLOOR:?};\n\n"
        ));
    }
    code.push_str(
        "/// Relative term of every Newton certification test (SPICE RELTOL): the\n\
         /// node-step test, the per-row KCL residual gate and the device-current\n\
         /// consistency check scale their tolerance by it. Not configurable.\n",
    );
    code.push_str(&format!("pub const RELTOL: f64 = {RELTOL:e};\n\n"));
    code.push_str(
        "/// Absolute floor (V) of the node-step test on the UNDAMPED Newton\n\
         /// correction, `|v_new - v| < RELTOL * max(|v|, |v_new|) + VNTOL` (SPICE\n\
         /// VNTOL); also the floor of the residual rows whose unit is volts.\n",
    );
    code.push_str(&format!("pub const VNTOL: f64 = {VNTOL:e};\n\n"));
    code.push_str(
        "/// Absolute floor (A) of the per-row KCL residual gate,\n\
         /// `|F_i| <= RELTOL * den_i + KCL_ABSTOL`, and of the chord exit step's\n\
         /// test on the KCL infinity norm.\n",
    );
    code.push_str(&format!("pub const KCL_ABSTOL: f64 = {KCL_ABSTOL:e};\n\n"));
    code.push_str(
        "/// Absolute floor (A) of the device-current consistency check on the\n\
         /// accepting iteration, `|i_nl - i_dev(v)| <= RELTOL * max(|i_nl|, |i_dev|,\n\
         /// ABSTOL_SCALE_FLOOR) + ABSTOL` per device dimension (SPICE ABSTOL).\n",
    );
    code.push_str(&format!("pub const ABSTOL: f64 = {ABSTOL:e};\n\n"));
    code.push_str(
        "/// Floor (A) of the magnitude that check scales RELTOL by, so a device\n\
         /// carrying ~0 A does not demand an unreachable tolerance.\n",
    );
    code.push_str(&format!(
        "pub const ABSTOL_SCALE_FLOOR: f64 = {ABSTOL_SCALE_FLOOR:e};\n\n"
    ));
    if substep {
        code.push_str(
            "/// Absolute acceptance (V) of the sub-step ladder: a sub-step's iterate is\n\
             /// accepted when its largest raw (undamped) node correction is under this\n\
             /// and the row residual gate passes. 1 mV on every node: looser than the\n\
             /// per-sample step test below ~1 V, tighter above it.\n",
        );
        code.push_str(&format!(
            "pub const SUBSTEP_ACCEPT_VOLTS: f64 = {SUBSTEP_ACCEPT_VOLTS:e};\n\n"
        ));
    }
    if natural {
        code.push_str(
            "/// Slope of the Newton line search's natural monotonicity test: a trial at\n\
             /// fraction lambda of the correction d is accepted when the simplified\n\
             /// correction there, G_aug^-1 F(v + lambda d) in the step's own factorization,\n\
             /// has weighted norm <= (1 - NATURAL_TEST_SLOPE * lambda) * ||d||_w (Deuflhard).\n\
             /// Steers the path only; certification is the separate three-part test.\n",
        );
        code.push_str(&format!(
            "pub const NATURAL_TEST_SLOPE: f64 = {NATURAL_TEST_SLOPE:?};\n\n"
        ));
        code.push_str(
            "/// Floor (A) of that test's weight on a branch-current unknown,\n\
             /// `1 / (RELTOL * max(|x|, |x_new|) + NATURAL_TEST_CURRENT_FLOOR)`; node rows\n\
             /// take VNTOL. The value is KCL_ABSTOL's, stated on its own.\n",
        );
        code.push_str(&format!(
            "pub const NATURAL_TEST_CURRENT_FLOOR: f64 = {NATURAL_TEST_CURRENT_FLOOR:e};\n\n"
        ));
        code.push_str(
            "/// Debug-build invariant of the reused Newton correction: the accepted trial's\n\
             /// back-solved residual must match the companion solve to this relative bound\n\
             /// (the two forms round differently; measured 4e-11, bound 250x that).\n",
        );
        code.push_str(&format!(
            "pub const NATURAL_REUSE_DEBUG_RELTOL: f64 = {NATURAL_REUSE_DEBUG_RELTOL:e};\n\n"
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The emitted constants print as the exact literals the expressions used
    /// before they were named, so the generated code's values are the same.
    #[test]
    fn emitted_literals_are_the_named_values() {
        let mut code = String::new();
        emit_transient_thresholds(&mut code, true, true, true);
        for line in [
            "pub const RELTOL: f64 = 1e-3;",
            "pub const VNTOL: f64 = 1e-6;",
            "pub const KCL_ABSTOL: f64 = 1e-9;",
            "pub const ABSTOL: f64 = 1e-12;",
            "pub const ABSTOL_SCALE_FLOOR: f64 = 1e-9;",
            "pub const SUBSTEP_ACCEPT_VOLTS: f64 = 1e-3;",
            "pub const NATURAL_TEST_SLOPE: f64 = 0.25;",
            "pub const NATURAL_TEST_CURRENT_FLOOR: f64 = 1e-9;",
            "pub const NATURAL_REUSE_DEBUG_RELTOL: f64 = 1e-8;",
        ] {
            assert!(code.contains(line), "missing `{line}` in:\n{code}");
        }
        let mut dk = String::new();
        emit_transient_thresholds(&mut dk, false, false, false);
        assert!(!dk.contains("SUBSTEP_ACCEPT_VOLTS"));
        assert!(!dk.contains("NATURAL_TEST"));
        assert_eq!(format!("{DC_OP_RELTOL:e}"), "1e-6");
        assert_eq!(format!("{DC_OP_STEP_ABSTOL_VOLTS:e}"), "1e-9");
        assert_eq!(format!("{DC_OP_KCL_ABSTOL_AMPS:e}"), "1e-9");
    }
}
