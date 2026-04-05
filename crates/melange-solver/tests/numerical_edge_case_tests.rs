//! Numerical edge case tests (#26).
//!
//! Tests for numerical stability under extreme component values,
//! subnormal inputs, near-singular matrices, and device model edge cases.
//!
//! Tests that only exercise device models or kernel validation are unchanged.
//! Tests that previously used runtime solvers now use codegen compile-and-run.

mod support;

use melange_devices::{BjtEbersMoll, DiodeShockley, NonlinearDevice};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

const SAMPLE_RATE: f64 = 44100.0;

// ============================================================================
// 1. Very small capacitors (1pF) -- alpha*C should not overflow
// ============================================================================

/// A 1pF capacitor with 1k resistor at 44.1kHz.
#[test]
fn test_very_small_capacitor_1pf() {
    let spice = "Small Cap\nR1 in out 1k\nC1 out 0 1p\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // Capacitor value should be stamped correctly
    let out_idx_raw = *mna.node_map.get("out").unwrap();
    assert!(mna.c[out_idx_raw - 1][out_idx_raw - 1] > 0.0);

    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();
    for &val in &kernel.s {
        assert!(val.is_finite(), "S not finite with 1pF cap: {}", val);
    }
    for &val in &kernel.a_neg {
        assert!(val.is_finite(), "A_neg not finite with 1pF cap: {}", val);
    }

    // Run codegen and verify finite output
    let config = support::config_for_spice(spice, SAMPLE_RATE);
    let circuit = support::build_circuit(spice, &config, "small_1pf");
    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);
}

/// 100fF capacitor — extreme case.
#[test]
fn test_very_small_capacitor_100ff() {
    let spice = "Tiny Cap\nR1 in out 1k\nC1 out 0 100f\n";
    let config = support::config_for_spice(spice, SAMPLE_RATE);
    let circuit = support::build_circuit(spice, &config, "small_100ff");
    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);
}

/// Small cap with a nonlinear device.
#[test]
fn test_very_small_capacitor_with_diode() {
    let spice = "Small Cap Diode\nRin in out 1k\nD1 out 0 D1N4148\nC1 out 0 1p\n.model D1N4148 D(IS=1e-15)\n";
    let config = support::config_for_spice(spice, SAMPLE_RATE);
    let circuit = support::build_circuit(spice, &config, "small_cap_diode");
    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);
}

// ============================================================================
// 2. Very large resistors
// ============================================================================

/// 10 MΩ resistor — tests solver precision with tiny conductances.
#[test]
fn test_very_large_resistor_10m() {
    let spice = "Large R\nR1 in out 10meg\nC1 out 0 1u\n";
    let config = support::config_for_spice(spice, SAMPLE_RATE);
    let circuit = support::build_circuit(spice, &config, "large_r");
    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);

    // With 10MΩ and G_in=1.0, the RC time constant is huge — output should
    // be small (slow charging). The key test is just that it's finite.
}

/// Impedance mismatch: 10M and 1Ω in the same circuit.
#[test]
fn test_impedance_mismatch_10m_and_1ohm() {
    let spice = "Mismatch\nR1 in mid 10meg\nR2 mid out 1\nC1 out 0 1u\n";
    let config = support::config_for_spice(spice, SAMPLE_RATE);
    let circuit = support::build_circuit(spice, &config, "mismatch");
    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);
}

// ============================================================================
// 3. Extreme device model parameters
// ============================================================================

/// Diode with very small IS=1e-18 (steep exponential).
#[test]
fn test_diode_extreme_small_is() {
    let diode = DiodeShockley::new_room_temp(1e-18, 1.0);

    let i_fwd = diode.current_at(0.7);
    assert!(i_fwd.is_finite() && i_fwd > 0.0);

    let g_fwd = diode.conductance_at(0.7);
    assert!(g_fwd.is_finite() && g_fwd > 0.0);

    let i_rev = diode.current_at(-1.0);
    assert!(i_rev.is_finite() && i_rev < 0.0 && i_rev.abs() < 1e-14);

    let i_zero = diode.current_at(0.0);
    assert!(i_zero.abs() < 1e-15);
}

/// Diode with IS=1e-18 in a circuit solver.
#[test]
fn test_diode_extreme_small_is_in_circuit() {
    let spice = "Stiff Diode\nRin in out 1k\nD1 out 0 D1N4148\nC1 out 0 1u\n.model D1N4148 D(IS=1e-18)\n";
    let config = support::config_for_spice(spice, SAMPLE_RATE);
    let circuit = support::build_circuit(spice, &config, "stiff_diode");
    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);
}

/// BJT with IS=1e-18 (very small saturation current).
#[test]
fn test_bjt_extreme_small_is() {
    let bjt = BjtEbersMoll::new_room_temp(1e-18, 200.0, 3.0, melange_devices::BjtPolarity::Npn);

    let ic = bjt.collector_current(0.7, -5.0);
    assert!(ic.is_finite() && ic > 0.0);

    let ib = bjt.base_current(0.7, -5.0);
    assert!(ib.is_finite() && ib > 0.0);

    let (dic_dvbe, dic_dvbc) = bjt.collector_jacobian(0.7, -5.0);
    assert!(dic_dvbe.is_finite());
    assert!(dic_dvbc.is_finite());
}

/// Diode with very large IS=1e-6 (germanium-like, very leaky).
#[test]
fn test_diode_large_is() {
    let diode = DiodeShockley::new_room_temp(1e-6, 1.5);

    let i_fwd = diode.current_at(0.3);
    assert!(i_fwd.is_finite() && i_fwd > 0.0);

    let i_rev = diode.current_at(-1.0);
    assert!(i_rev.is_finite() && i_rev < 0.0 && i_rev.abs() > 1e-8);
}

// ============================================================================
// 4. Subnormal float inputs to device functions
// ============================================================================

#[test]
fn test_diode_subnormal_voltage() {
    let diode = DiodeShockley::silicon();
    let subnormal: f64 = f64::MIN_POSITIVE / 2.0;
    assert!(subnormal > 0.0 && subnormal < f64::MIN_POSITIVE);

    assert!(diode.current_at(subnormal).is_finite());
    assert!(diode.conductance_at(subnormal).is_finite());
    assert!(diode.current_at(-subnormal).is_finite());
}

#[test]
fn test_bjt_subnormal_voltages() {
    let bjt = BjtEbersMoll::npn_2n2222a();
    let subnormal: f64 = f64::MIN_POSITIVE / 2.0;

    assert!(bjt.collector_current(subnormal, -subnormal).is_finite());
    assert!(bjt.base_current(subnormal, -subnormal).is_finite());
    let (d1, d2) = bjt.collector_jacobian(subnormal, -subnormal);
    assert!(d1.is_finite());
    assert!(d2.is_finite());
}

#[test]
fn test_gummel_poon_subnormal_voltages() {
    use melange_devices::BjtGummelPoon;
    let base = BjtEbersMoll::new_room_temp(1e-14, 200.0, 3.0, melange_devices::BjtPolarity::Npn);
    let gp = BjtGummelPoon::new(base, f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let subnormal: f64 = f64::MIN_POSITIVE / 2.0;
    let jac = gp.jacobian(&[subnormal, -subnormal]);
    for &val in &jac {
        assert!(val.is_finite(), "GP Jacobian with subnormal should be finite: {}", val);
    }
}

#[test]
fn test_device_functions_at_exact_zero() {
    let diode = DiodeShockley::silicon();
    assert!(diode.current_at(0.0).is_finite());
    assert!(diode.conductance_at(0.0).is_finite());

    let bjt = BjtEbersMoll::npn_2n2222a();
    assert!(bjt.collector_current(0.0, 0.0).is_finite());
    assert!(bjt.base_current(0.0, 0.0).is_finite());
    let (d1, d2) = bjt.collector_jacobian(0.0, 0.0);
    assert!(d1.is_finite());
    assert!(d2.is_finite());
}

// ============================================================================
// 5. Near-singular matrix conditioning
// ============================================================================

/// Extremely mismatched impedances: 1 ohm and 10M ohm.
#[test]
fn test_near_singular_extreme_mismatch() {
    let spice = "Mismatch\nR1 in mid 1\nR2 mid out 10meg\nC1 out 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let result = DkKernel::from_mna(&mna, SAMPLE_RATE);
    assert!(result.is_ok(), "Kernel should handle 1:10M mismatch: {:?}", result.err());
    let kernel = result.unwrap();
    for &val in &kernel.s {
        assert!(val.is_finite(), "S not finite in mismatched circuit: {}", val);
    }
}

/// One node has very low impedance to ground (near short).
#[test]
fn test_near_singular_low_impedance_node() {
    let spice = "Low Z\nR1 in out 1k\nR2 out 0 0.01\nC1 out 0 1u\n";
    let config = support::config_for_spice(spice, SAMPLE_RATE);
    let circuit = support::build_circuit(spice, &config, "low_z");
    let output = support::run_step(&circuit, 1.0, 100, SAMPLE_RATE);
    support::assert_finite(&output);
    // Output should be very small since the node is nearly shorted
    for (i, &v) in output.iter().enumerate() {
        assert!(v.abs() < 0.1, "Output[{}] should be small with near-short: {:.6}", i, v);
    }
}

/// Multiple voltage sources create near-singular G matrix blocks.
#[test]
fn test_multiple_voltage_sources_conditioning() {
    let spice = "Multi VS\nV1 vcc 0 DC 12\nV2 vee 0 DC -12\nR1 vcc out 1k\nR2 out vee 1k\nC1 out 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();
    for &val in &kernel.s {
        assert!(val.is_finite(), "S not finite with multiple VS: {}", val);
    }
    for &val in &kernel.a_neg {
        assert!(val.is_finite(), "A_neg not finite with multiple VS: {}", val);
    }
}

// ============================================================================
// 6. High sample rate edge case
// ============================================================================

#[test]
fn test_high_sample_rate_small_cap() {
    let spice = "High SR\nR1 in out 1k\nC1 out 0 1p\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 192000.0).unwrap();
    for &val in &kernel.s {
        assert!(val.is_finite(), "S not finite at 192kHz with 1pF: {}", val);
    }

    let mut config = support::config_for_spice(spice, 192000.0);
    config.sample_rate = 192000.0;
    let circuit = support::build_circuit(spice, &config, "high_sr");
    let output = support::run_step(&circuit, 1.0, 100, 192000.0);
    support::assert_finite(&output);
}

// ============================================================================
// 7. safe_exp edge cases
// ============================================================================

#[test]
fn test_safe_exp_boundary_values() {
    use melange_devices::safeguards;

    let exp_40 = safeguards::safe_exp(40.0);
    assert!(exp_40.is_finite());

    let exp_100 = safeguards::safe_exp(100.0);
    assert!(exp_100.is_finite());
    assert_eq!(exp_100, exp_40, "safe_exp should clamp at 40");

    let exp_neg40 = safeguards::safe_exp(-40.0);
    assert!(exp_neg40.is_finite() && exp_neg40 > 0.0);

    let exp_neg100 = safeguards::safe_exp(-100.0);
    assert!(exp_neg100.is_finite() && exp_neg100 > 0.0);
    assert_eq!(exp_neg100, exp_neg40);

    let _ = safeguards::safe_exp(f64::NAN); // should not panic

    let exp_inf = safeguards::safe_exp(f64::INFINITY);
    assert!(exp_inf.is_finite());

    let exp_neg_inf = safeguards::safe_exp(f64::NEG_INFINITY);
    assert!(exp_neg_inf.is_finite() && exp_neg_inf > 0.0);
}

/// Diode at voltages that would normally cause exp() overflow.
#[test]
fn test_diode_extreme_forward_bias() {
    let diode = DiodeShockley::silicon();
    assert!(diode.current_at(10.0).is_finite());
    assert!(diode.conductance_at(10.0).is_finite());

    let i_neg10v = diode.current_at(-10.0);
    assert!(i_neg10v.is_finite() && i_neg10v < 0.0);
}

/// BJT at voltages that stress safe_exp limits.
#[test]
fn test_bjt_extreme_voltages() {
    let bjt = BjtEbersMoll::npn_2n2222a();

    assert!(bjt.collector_current(2.0, -5.0).is_finite());
    assert!(bjt.collector_current(0.7, -50.0).is_finite());
    assert!(bjt.collector_current(2.0, 2.0).is_finite());

    let (d1, d2) = bjt.collector_jacobian(2.0, -50.0);
    assert!(d1.is_finite());
    assert!(d2.is_finite());
}

// ============================================================================
// 8. Very large circuit with many nodes
// ============================================================================

#[test]
fn test_large_rc_chain() {
    let spice = "RC Chain\nR1 in n1 1k\nC1 n1 0 100n\nR2 n1 n2 1k\nC2 n2 0 100n\nR3 n2 n3 1k\nC3 n3 0 100n\nR4 n3 n4 1k\nC4 n4 0 100n\nR5 n4 out 1k\nC5 out 0 100n\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    assert!(mna.n >= 5, "RC chain should have at least 5 nodes, got {}", mna.n);

    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();
    for &val in &kernel.s {
        assert!(val.is_finite(), "S not finite in RC chain: {}", val);
    }

    let config = support::config_for_spice(spice, SAMPLE_RATE);
    let circuit = support::build_circuit(spice, &config, "rc_chain");
    let output = support::run_step(&circuit, 1.0, 500, SAMPLE_RATE);
    support::assert_finite(&output);

    // After 500 samples, the output should have started charging
    let final_out = *output.last().unwrap();
    assert!(final_out > 0.0, "RC chain final output should be positive: {:.6}", final_out);
}

// ============================================================================
// 9. Diode ideality factor edge cases (device model only — no solver)
// ============================================================================

#[test]
fn test_diode_ideality_factor_1() {
    let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
    assert!(diode.current_at(0.7).is_finite() && diode.current_at(0.7) > 0.0);
    assert!(diode.conductance_at(0.7).is_finite() && diode.conductance_at(0.7) > 0.0);
}

#[test]
fn test_diode_ideality_factor_3() {
    let diode = DiodeShockley::new_room_temp(1e-12, 3.0);
    let i_fwd = diode.current_at(0.7);
    assert!(i_fwd.is_finite() && i_fwd > 0.0);

    let diode_n1 = DiodeShockley::new_room_temp(1e-12, 1.0);
    let i_n1 = diode_n1.current_at(0.7);
    assert!(i_fwd < i_n1, "n=3 should have less current than n=1: n3={:.2e}, n1={:.2e}", i_fwd, i_n1);
}

// ============================================================================
// 10. Inductor with very small value
// ============================================================================

#[test]
fn test_very_small_inductor() {
    let spice = "Small L\nR1 in out 1k\nL1 out 0 1u\nC1 out 0 100p\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    assert_eq!(kernel.inductors.len(), 1);
    let g_eq = kernel.inductors[0].g_eq;
    assert!(g_eq.is_finite() && g_eq > 1.0, "1uH inductor g_eq should be large: {}", g_eq);

    let config = support::config_for_spice(spice, SAMPLE_RATE);
    let circuit = support::build_circuit(spice, &config, "small_l");
    let output = support::run_step(&circuit, 1.0, 200, SAMPLE_RATE);
    support::assert_finite(&output);
}
