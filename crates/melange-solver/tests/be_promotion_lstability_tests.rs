//! Backward Euler after a promotion: the spectral radius it achieves.
//!
//! "BE is L-stable, so `rho > 1` after promotion means the matrix builder has
//! a stamping bug" holds only for a circuit whose *linearization* is itself
//! continuum-stable (every mode has `Re(lambda) <= 0`). A circuit that is
//! genuinely unstable at its DC operating point — a regenerative oscillator
//! sitting on an unstable bias point by design — has a real growing mode
//! that NO consistent integrator, backward Euler included, can force to
//! `rho <= 1` without falsifying the circuit's own physics.
//!
//! `dissipative_circuit_achieves_rho_below_one_under_be` pins the
//! dissipative side: a stiff diode-clamped node (1 kOhm into 10 pF) that the
//! ring predicate promotes to backward Euler (its trapezoidal Nyquist ring
//! starts at -54 dB of the passband from the input). BE must genuinely
//! achieve rho < 1 there, which checks the BE matrix-building math
//! (alpha = 1/T, A_neg_be = alpha*C, no -G term).
//!
//! The regenerative side (a real growing pole keeps rho > 1 under BE) is not
//! re-asserted here as "rho < 1", because that would assert something false
//! about the circuit's physics; `routing_oversampling_rate_tests.rs` covers
//! it.

use melange_solver::codegen::ir::{CircuitIR, IntegratorSelection};
use melange_solver::codegen::CodegenConfig;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const STIFF_CLAMP: &str = "\
Stiff diode-clamped node (dissipative, no feedback)
R_s in out 1k
C_p out 0 10p
D_1 out 0 DX
D_2 0 out DX
R_l out 0 100k
.model DX D(IS=2.52n N=1.752)
";

const DIODE_RC: &str = "\
Diode RC (dissipative, no feedback)
Rin in n1 1k
D1 n1 0 DMOD
C1 n1 0 10n
Rout n1 out 1k
Rload out 0 100k
.model DMOD D(IS=1e-14 N=1.0)
";

fn build(deck: &str, sample_rate: f64) -> (CircuitIR, usize) {
    let netlist = Netlist::parse(deck).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let input_node = mna.node_map["in"] - 1;
    let output_node = mna.node_map["out"] - 1;
    mna.g[input_node][input_node] += 1.0;
    let cfg = CodegenConfig {
        circuit_name: "lstability".to_string(),
        sample_rate,
        input_node,
        output_nodes: vec![output_node],
        output_scales: vec![1.0],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    (
        CircuitIR::from_mna(&mna, &netlist, &cfg).expect("nodal IR build"),
        input_node,
    )
}

#[test]
fn dissipative_circuit_achieves_rho_below_one_under_be() {
    let (ir, input_node) = build(STIFF_CLAMP, 48000.0);
    assert_eq!(
        ir.integrator_selection,
        IntegratorSelection::BeAuto,
        "fixture regression: the ring predicate must promote this stiff node ({})",
        ir.integration_reason
    );
    let stability = melange_solver::codegen::stability::analyze_trap_stability_deflated(
        &ir.matrices.s,
        &ir.matrices.a_neg,
        ir.topology.n,
        &[input_node],
    );
    assert!(
        stability.rho < 1.0,
        "BE matrices must genuinely achieve rho < 1 for a dissipative, feedback-free \
         circuit — got rho={:.6}, dominant_sign={:+.0}. A violation here (unlike a \
         genuinely unstable circuit's linearization) WOULD indicate a real BE \
         matrix-builder defect.",
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

/// At 5 MHz the whole-system power iteration read this diode-RC as
/// trap-unstable (rho = 1.0084, a round-off artifact of the `(2/T)*C`
/// conditioning) and promoted it. The ring predicate works on the exact
/// eigenvalues of the charge propagator and finds nothing to promote.
#[test]
fn a_round_off_reading_at_5_mhz_does_not_promote() {
    let (ir, _) = build(DIODE_RC, 5.0e6);
    assert_eq!(
        ir.integrator_selection,
        IntegratorSelection::TrapDefault,
        "{}",
        ir.integration_reason
    );
    assert!(!ir.integration_reason.is_empty());
}
