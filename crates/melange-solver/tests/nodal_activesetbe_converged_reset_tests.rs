//! Regression test for the nodal full-LU ActiveSetBe BE-fallback
//! `converged`-not-reset defect (2026-08-17, dr-debuggenshmirtz / L6 arbiter).
//!
//! ## The bug
//!
//! `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs`, the
//! full-LU nodal `process_sample` (`emit_nodal_process_sample`), emits a
//! Backward-Euler fallback that is entered either when the primary trap/BE NR
//! failed (`!converged`) OR, in `ActiveSetBe` mode, when an op-amp output
//! engaged its rail on an otherwise-converged primary solve
//! (`active_set_engaged`):
//!
//! ```text
//! if !converged || active_set_engaged {
//!     state.diag_be_fallback_count += 1;
//!     chord_valid = false;
//!     // ... BE NR loop; sets `converged = true` only on BE convergence ...
//! }
//! // ...
//! if !converged { v = state.v_prev; i_nl = state.i_nl_prev; chord_valid = false; }
//! ```
//!
//! When the fallback was entered via `active_set_engaged`, `converged` is still
//! `true` from the primary solve. If the BE NR loop then FAILS to converge, it
//! never clears the flag, so:
//!   - the death-spiral guard `if !converged { v = state.v_prev; ... }` is
//!     skipped, committing the diverged BE iterate over the valid primary root;
//!   - `diag_nr_max_iter_count` stays 0 (the trap loop did converge), so the
//!     NR-starvation warning cannot see the failure.
//!
//! Silent wrong output with a clean diagnostic. The Schur path does not share
//! this defect: it derives `converged = last_nr_iterations < MAX_ITER &&
//! !active_set_engaged`, so `converged` is already false on the
//! rail-engagement entry.
//!
//! ## The fix
//!
//! Reset `converged = false` at fallback ENTRY (right after the diag/chord
//! bookkeeping, before the BE loop). The BE loop sets it back to `true` on
//! success, so a fallback whose BE solve converges is byte-identical; only a
//! genuine BE failure now correctly reverts to `state.v_prev` via the existing
//! death-spiral guard.
//!
//! ## Test
//!
//! Code-string pin on the full-LU ActiveSetBe path: the reset must appear
//! contiguously after the fallback bookkeeping and before the BE loop, and the
//! death-spiral guard it re-enables must be present.

mod support;

use melange_solver::codegen::OpampRailMode;

const SR: f64 = 48000.0;

/// Op-amp gain stage with finite (±9 V) rails, cap-coupled ("audio-path")
/// output, and a diode to ground so the nonlinear NR dimension is non-empty
/// (M > 0). The trailing behavioral B-source across its own grounded resistor
/// forces the full-LU nodal path (`emit_nodal` routes `use_full_nodal = true`
/// whenever `ir.behavioral_sources` is non-empty) — mirrors the FULL_LU_FORCE
/// trick in `nodal_emitter_regression_tests.rs`.
const ACTIVE_SET_BE_SPICE: &str = "\
ActiveSetBe full-LU fixture
Rin in ninv 10k
Rf ninv oap 100k
U1 0 ninv oap OA9
Cout oap out 1u
Rload out 0 100k
D1 out 0 D1N4148
Vee vee 0 DC -9
Vcc vcc 0 DC 9
.model OA9 OA(AOL=200000 ROUT=50 GBW=3MEG VCC=9 VEE=-9)
.model D1N4148 D(IS=2.52e-9 N=1.752)
";

fn active_set_be_code() -> (String, usize, usize) {
    let base = support::config_for_spice(ACTIVE_SET_BE_SPICE, SR);
    let config = melange_solver::codegen::CodegenConfig {
        circuit_name: "activesetbe_converged_reset_test".to_string(),
        sample_rate: SR,
        // Force full-LU explicitly instead of the old behavioral-dummy routing
        // lever (`B_frc frc 0 V={0}`), which now carries fallback-gating semantics.
        force_full_lu: true,
        // Force the mode so the test does not depend on the auto-detector's
        // audio-path heuristic; ActiveSetBe is what a cap-coupled op-amp
        // output resolves to in production anyway.
        opamp_rail_mode: OpampRailMode::ActiveSetBe,
        ..base
    };
    support::generate_circuit_code_nodal(ACTIVE_SET_BE_SPICE, &config)
}

#[test]
fn test_activesetbe_full_lu_be_fallback_resets_converged_at_entry() {
    let (code, n, m) = active_set_be_code();
    assert!(
        n > 0 && m > 0,
        "expected a nontrivial nodal circuit (n={n}, m={m})"
    );

    // Confirm this is the full-LU ActiveSetBe path (the entry condition that
    // can bring in a converged primary via rail engagement).
    assert!(
        code.contains("if !converged || active_set_engaged"),
        "fixture did not route to the full-LU ActiveSetBe BE fallback"
    );

    // The reset must be emitted contiguously after the fallback bookkeeping,
    // before the BE NR loop runs. Without it, a failed BE solve entered via
    // `active_set_engaged` commits a diverged iterate over the valid primary
    // root (converged stays true) and reports clean (nr_max_iter_count == 0).
    assert!(
        code.contains(
            "state.diag_be_fallback_count += 1;\n        chord_valid = false;\n        converged = false;\n"
        ),
        "ActiveSetBe BE fallback must reset `converged = false` at entry so a \
         failed BE solve falls through the death-spiral guard instead of \
         committing the diverged iterate over the converged primary root"
    );
}

#[test]
fn test_activesetbe_full_lu_death_spiral_guard_present() {
    let (code, _n, _m) = active_set_be_code();

    // The reset is only meaningful if the death-spiral guard it re-enables is
    // present: on a genuine BE failure `converged` is now false, and this guard
    // reverts to the last known-good state instead of committing garbage.
    assert!(
        code.contains("if !converged {")
            && code.contains("v = state.v_prev;")
            && code.contains("i_nl = state.i_nl_prev;"),
        "full-LU path must keep the `if !converged {{ v = state.v_prev; \
         i_nl = state.i_nl_prev; }}` death-spiral guard that the entry reset \
         re-enables for failed BE fallbacks"
    );
}
