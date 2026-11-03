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

/// Absolute floor, in amps, of the device-current consistency check: on the
/// accepting iteration, `|i_nl − i_dev(v)| <= RELTOL·max(|i_nl|, |i_dev|,
/// ABSTOL_SCALE_FLOOR) + ABSTOL` per device dimension (SPICE's ABSTOL). The
/// DK Newton's current-residual test and the nodal full-LU chord consistency
/// check are the same test. Ruled 2026-11-03 to be named; value is SPICE's
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

/// Emit the transient certification thresholds into the generated code as
/// named module-level constants (both routes). `substep` adds the sub-step
/// ladder's acceptance, which only the nodal route has.
pub(super) fn emit_transient_thresholds(code: &mut String, substep: bool) {
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
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The emitted constants print as the exact literals the expressions used
    /// before they were named, so the generated code's values are the same.
    #[test]
    fn emitted_literals_are_the_named_values() {
        let mut code = String::new();
        emit_transient_thresholds(&mut code, true);
        for line in [
            "pub const RELTOL: f64 = 1e-3;",
            "pub const VNTOL: f64 = 1e-6;",
            "pub const KCL_ABSTOL: f64 = 1e-9;",
            "pub const ABSTOL: f64 = 1e-12;",
            "pub const ABSTOL_SCALE_FLOOR: f64 = 1e-9;",
            "pub const SUBSTEP_ACCEPT_VOLTS: f64 = 1e-3;",
        ] {
            assert!(code.contains(line), "missing `{line}` in:\n{code}");
        }
        let mut dk = String::new();
        emit_transient_thresholds(&mut dk, false);
        assert!(!dk.contains("SUBSTEP_ACCEPT_VOLTS"));
        assert_eq!(format!("{DC_OP_RELTOL:e}"), "1e-6");
        assert_eq!(format!("{DC_OP_STEP_ABSTOL_VOLTS:e}"), "1e-9");
        assert_eq!(format!("{DC_OP_KCL_ABSTOL_AMPS:e}"), "1e-9");
    }
}
