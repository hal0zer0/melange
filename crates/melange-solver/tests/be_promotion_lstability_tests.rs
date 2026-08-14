//! Regression tests for the BE post-promotion diagnostic revision
//! (2026-08-14, "Bug 2" of `memory/dk_backward_euler_ignored_trap_unstable.md`).
//!
//! Before this fix, both the nodal and (newly added) DK post-BE-promotion
//! checks asserted "BE is L-stable by construction; rho > 1 means the
//! matrix builder has a stamping bug" unconditionally. That premise is only
//! true for a circuit whose *linearization* is itself continuum-stable
//! (every mode has `Re(lambda) <= 0`). A circuit that is genuinely unstable
//! at its DC operating point — a regenerative oscillator sitting on an
//! unstable bias point by design — has a real growing mode that NO
//! consistent integrator, backward Euler included, can force to `rho <= 1`
//! without falsifying the circuit's own physics.
//!
//! Two circuits pin the two sides of this:
//!
//! 1. `dissipative_circuit_achieves_rho_below_one_under_be` — a diode-RC
//!    circuit (no feedback, no gain, obviously dissipative) sampled at an
//!    absurd rate (5 MHz) where trap's discretization shows a spurious
//!    "trap unstable" reading purely from round-off (rho = 1.0084,
//!    dominant_sign -1 — the round-off/Nyquist artifact signature, NOT a
//!    real growing pole). This is exactly the case the post-promotion check
//!    exists to validate: BE genuinely achieves rho < 1 here, proving the
//!    BE matrix-building math (alpha = 1/T, A_neg_be = alpha*C, no -G term)
//!    is correct.
//!
//! 2. `regenerative_oscillator_be_rho_exceeds_one_is_not_a_bug` (in
//!    `routing_oversampling_rate_tests.rs`'s sibling coverage — see the g10
//!    oscillator handling in `codegen::ir::mod` tests) is NOT re-asserted
//!    here as "rho < 1" because doing so would be asserting something
//!    false about real circuit physics. Instead
//!    `crates/melange-solver/tests/routing_oversampling_rate_tests.rs`
//!    pins that such a circuit correctly routes to nodal (which has a
//!    full-LU NR fallback) instead of demanding an impossible rho.

use melange_solver::codegen::{ir::CircuitIR, CodegenConfig};
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const DIODE_RC: &str = "\
Diode RC (dissipative, no feedback)
Rin in n1 1k
D1 n1 0 DMOD
C1 n1 0 10n
Rout n1 out 1k
Rload out 0 100k
.model DMOD D(IS=1e-14 N=1.0)
";

/// At 5 MHz this obviously-dissipative circuit (a diode clamp feeding an RC
/// lowpass — no gain, no feedback, unconditionally stable for any real
/// component values) shows a trap-instability reading purely from f64
/// round-off in the `(2/T)*C` conditioning at an absurd sample rate. It is
/// the "false positive, not a real growing pole" counterpart to the g10
/// regenerative oscillator (which IS a real growing pole and correctly
/// keeps rho > 1 even under BE).
#[test]
fn dissipative_circuit_achieves_rho_below_one_under_be() {
    let netlist = Netlist::parse(DIODE_RC).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    mna.g[input_node][input_node] += 1.0;

    let cfg = CodegenConfig {
        circuit_name: "diode_rc_hirate".to_string(),
        sample_rate: 5.0e6,
        input_node,
        output_nodes: vec![output_node],
        output_scales: vec![1.0],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };

    let ir = CircuitIR::from_mna(&mna, &netlist, &cfg).expect("nodal IR build");

    assert!(
        ir.solver_config.backward_euler,
        "fixture regression: this circuit must auto-promote to BE at 5 MHz \
         (trap propagation operator must read unstable here) for the test to \
         exercise the post-promotion path at all"
    );

    let stability = melange_solver::codegen::stability::analyze_trap_stability_deflated(
        &ir.matrices.s,
        &ir.matrices.a_neg,
        ir.topology.n,
        input_node,
    );
    assert!(
        stability.rho < 1.0,
        "BE matrices must genuinely achieve rho < 1 for a dissipative, feedback-free \
         circuit misdetected as trap-unstable due to sample-rate round-off — got \
         rho={:.6}, dominant_sign={:+.0}. A violation here (unlike a genuinely \
         unstable circuit's linearization) WOULD indicate a real BE matrix-builder \
         defect.",
        stability.rho,
        stability.dominant_sign
    );
    // Also pins the field the nodal emitter's Schur-vs-full-LU gate reads.
    assert!(
        ir.matrices.spectral_radius_s_aneg < 1.0 + 1e-6,
        "spectral_radius_s_aneg (emitter-contract value) must also read <= 1 after \
         promotion for this dissipative circuit, got {}",
        ir.matrices.spectral_radius_s_aneg
    );
}
