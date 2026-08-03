//! Regression test for the nodal full-LU BE-fallback alpha-floor bug
//! (2026-08-03, dr-debuggenshmirtz investigation).
//!
//! ## The bug
//!
//! `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs` emits a
//! "global node voltage damping" layer (both in the primary trap/BE-primary
//! NR loop and in the Backward Euler fallback loop) that caps the worst-case
//! per-iteration node voltage step at a threshold (`damp_thresh`, or a fixed
//! 10.0 V in the BE fallback):
//!
//! ```text
//! if max_node_dv > damp_thresh {
//!     alpha *= (damp_thresh / max_node_dv).max(0.01);   // BUGGY
//! }
//! ```
//!
//! The `.max(0.01)` floor bounds how much the damping ratio can shrink BY,
//! not what the resulting damped step actually IS. When a single NR
//! iteration's companion-model LU solve produces a raw voltage delta many
//! orders of magnitude beyond `damp_thresh` (observed: 3.8e7 V on
//! wurli-power-amp at a class-AB crossover device-state transition), the 1%
//! floor still lets a multiple of `damp_thresh` through (0.01 * 3.8e7 =
//! 380,000 V — nowhere near the intended <=10 V cap). The resulting
//! ~3.8 kV single-iteration jump launches the trajectory into a deeply
//! nonphysical operating point that the remaining NR iterations, still
//! locally damped, cannot recover from within the iteration budget.
//!
//! The BE-fallback path's voltage-step convergence criterion
//! (`be_step_exceeded`) then declares false convergence: its relative
//! tolerance (`1e-3 * v[node].abs()`) scales with the ALREADY-DIVERGED node
//! voltage, so once a node sits at ~-16,000 V, an oscillating ~10-160 V
//! per-iteration step trivially satisfies the tolerance. The wildly
//! nonphysical state gets committed to `state.v_prev`/`state.i_nl_prev`,
//! corrupting every subsequent sample.
//!
//! ## Real-circuit confirmation (CLI, not this test)
//!
//! Reproduced and fixed on `melange-circuits/unstable/amp/wurli-power-amp.cir`
//! (N=20, M=14 after `.linearize Q9`, auto-routed to nodal full-LU NR with
//! Backward-Euler-primary integration) via `melange compile` /
//! `melange simulate`. A 1 kHz sine at 88.2 kHz drove an internal node
//! (`emit_pair`, the differential pair's shared emitter) as high as
//! -16,079 V to -27,977 V depending on drive amplitude — thousands of volts
//! outside any physically sane range for a +-22.5 V-rail amplifier. After
//! removing the `.max(0.01)` floor, the same sweep (amplitudes 0.05-2.00 V)
//! stayed within 20-32 V (matching the +-22.5 V rails) at every amplitude,
//! and `diag_nr_max_iter_count`/`diag_be_fallback_count` both dropped by
//! 10-70x (bad state no longer cascades into subsequent samples' NR).
//!
//! This exact scenario isn't reproduced as an automated test here because
//! `.linearize` reduction requires the CLI's `apply_linearize_reductions`
//! DC-OP preflight (`tools/melange-cli/src/main.rs`), which isn't exposed
//! to library-level tests; the un-linearized M=16 variant of this circuit
//! doesn't converge at all in melange (a separate, known limitation — Q9's
//! full-nonlinear Vbe-multiplier topology is why it was linearized in the
//! first place) and so can't stand in for the M=14 circuit's dynamics.
//!
//! ## The fix
//!
//! Remove the `.max(0.01)` floor — `alpha *= damp_thresh / max_node_dv`
//! (uncapped division). This keeps the worst-case per-iteration node step
//! at exactly `damp_thresh` regardless of how large the raw delta is,
//! matching the layer's documented intent ("Global node voltage damping").
//! Applied to both the primary-loop damping and the BE-fallback damping in
//! `emit_nodal` (`nodal_emitter.rs`).
//!
//! ## This test
//!
//! Code-string pin on the full-LU nodal path (forced via the same inert
//! behavioral-B-source trick `nodal_emitter_regression_tests.rs` uses):
//! asserts the emitted damping lines divide uncapped by `damp_thresh`/`10.0`
//! and that the `.max(0.01)` floor is gone from both the primary loop and
//! the Backward Euler fallback loop. Fails before the fix (old text
//! present, new text absent) and passes after.

mod support;

const SR: f64 = 48000.0;

/// Small BJT common-emitter stage. Only needs to exercise the nodal
/// full-LU path's device-evaluation + damping code — the specific circuit
/// doesn't matter for a code-string pin, unlike the real wurli-power-amp
/// blowup (which needs the exact M=14 linearized topology, see module docs).
const BJT_CE_SPICE: &str = "\
BJT Common Emitter
Cin in base 1u
R1 vcc base 47k
R2 base 0 10k
Q1 coll base emit Q2N3904
Rc vcc coll 2.2k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 1u
Rload out 0 100k
Vcc vcc 0 DC 12
.model Q2N3904 NPN(IS=6.734e-15 BF=416.4 VAF=74.03 NF=1)

* Inert full-LU forcing branch (mirrors nodal_emitter_regression_tests.rs
* FULL_LU_FORCE): behavioral B-sources are only representable on the nodal
* full-LU path (`emit_nodal` routes `use_full_nodal = true` whenever
* `ir.behavioral_sources` is non-empty), so a 0 V B-source across its own
* grounded resistor deterministically forces full LU without coupling to
* the rest of the circuit.
B_frc frc 0 V={0}
R_frc frc 0 1k
";

/// Code-string pin: the `.max(0.01)` ratio floor must not reappear in
/// either the primary-loop or the BE-fallback global node damping.
#[test]
fn test_nodal_full_lu_node_damping_has_no_ratio_floor() {
    let config = melange_solver::codegen::CodegenConfig {
        circuit_name: "nodal_full_lu_damping_test".to_string(),
        sample_rate: SR,
        backward_euler: true,
        ..support::config_for_spice(BJT_CE_SPICE, SR)
    };
    let (code, n, m) = support::generate_circuit_code_nodal(BJT_CE_SPICE, &config);
    assert!(
        n > 0 && m > 0,
        "expected a nontrivial nodal circuit (n={n}, m={m})"
    );

    assert!(
        code.contains("alpha *= damp_thresh / max_node_dv;"),
        "primary-loop node damping should divide uncapped by damp_thresh (no ratio floor)"
    );
    assert!(
        code.contains("alpha *= 10.0 / max_node_dv;"),
        "BE-fallback node damping should divide uncapped by 10.0 (no ratio floor)"
    );
    assert!(
        !code.contains("(damp_thresh / max_node_dv).max(0.01)"),
        "primary-loop node damping must not reintroduce the 1% ratio floor \
         (lets a multiple of damp_thresh through when the raw NR step is huge)"
    );
    assert!(
        !code.contains("(10.0 / max_node_dv).max(0.01)"),
        "BE-fallback node damping must not reintroduce the 1% ratio floor \
         (lets a multiple of 10.0 V through when the raw NR step is huge)"
    );
}
