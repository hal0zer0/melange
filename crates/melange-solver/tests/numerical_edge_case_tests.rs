//! Numerical edge case tests (#26).
//!
//! Tests for numerical stability under extreme component values,
//! subnormal inputs, near-singular matrices, and device model edge cases.

use melange_devices::{BjtEbersMoll, DiodeShockley, NonlinearDevice};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::solver::{CircuitSolver, DeviceEntry, LinearSolver};

const SAMPLE_RATE: f64 = 44100.0;

// ============================================================================
// 1. Very small capacitors (1pF) -- alpha*C should not overflow
// ============================================================================

/// A 1pF capacitor with 1k resistor at 44.1kHz.
/// alpha = 2/T = 2*44100 = 88200. alpha*C = 88200 * 1e-12 = 8.82e-8.
/// This is tiny but should not cause overflow or loss of significance.
#[test]
fn test_very_small_capacitor_1pf() {
    let spice = "Small Cap\nR1 in out 1k\nC1 out 0 1p\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // Capacitor value should be stamped correctly
    let out_idx_raw = *mna.node_map.get("out").unwrap();
    assert!(
        mna.c[out_idx_raw - 1][out_idx_raw - 1] > 0.0,
        "1pF capacitor should produce positive C stamp"
    );

    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    // S matrix should be finite and well-formed
    for i in 0..kernel.n {
        for j in 0..kernel.n {
            let s_ij = kernel.s[i * kernel.n + j];
            assert!(
                s_ij.is_finite(),
                "S[{},{}] = {} is not finite with 1pF cap",
                i,
                j,
                s_ij
            );
        }
    }

    // A_neg matrix should also be finite
    for i in 0..kernel.n {
        for j in 0..kernel.n {
            let a_neg_ij = kernel.a_neg[i * kernel.n + j];
            assert!(
                a_neg_ij.is_finite(),
                "A_neg[{},{}] = {} is not finite with 1pF cap",
                i,
                j,
                a_neg_ij
            );
        }
    }

    // Run the solver and verify finite output
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = out_idx_raw - 1;
    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.input_conductance = 0.001; // 1/1k

    for i in 0..200 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite with 1pF cap, got {}",
            i,
            output
        );
    }
}

/// Even smaller: 0.1pF (100fF). This is at the edge of practical values.
#[test]
fn test_very_small_capacitor_100ff() {
    let spice = "Tiny Cap\nR1 in out 10k\nC1 out 0 0.1p\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;
    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.input_conductance = 1.0 / 10000.0;

    for i in 0..100 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite with 100fF cap, got {}",
            i,
            output
        );
    }
}

/// 1pF cap with a nonlinear device (diode). The small capacitor
/// should provide enough damping to avoid oscillation.
#[test]
fn test_very_small_capacitor_with_diode() {
    let spice =
        "Diode + 1pF\nRin in out 1k\nD1 out 0 D1N4148\nC1 out 0 1p\n.model D1N4148 D(IS=1e-15)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let diode = DiodeShockley::new_room_temp(1e-15, 1.0);
    let devices = vec![DeviceEntry::new_diode(diode, 0)];
    let mut solver = CircuitSolver::new(kernel, devices, in_idx, out_idx).unwrap();
    solver.input_conductance = 0.001;

    for i in 0..200 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite with 1pF + diode, got {}",
            i,
            output
        );
    }
}

// ============================================================================
// 2. Very large resistors (10M ohm) -- precision loss tests
// ============================================================================

/// 10 Megohm resistor has conductance 1e-7 S. This is very small compared
/// to the 2C/T term from even moderate capacitors, but should not cause
/// precision loss in the A matrix or its inverse.
#[test]
fn test_very_large_resistor_10m() {
    let spice = "Large R\nR1 in out 10meg\nC1 out 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // Conductance stamp should be very small but nonzero
    let in_idx_raw = *mna.node_map.get("in").unwrap();
    let g_stamp = mna.g[in_idx_raw - 1][in_idx_raw - 1];
    assert!(
        g_stamp > 0.0 && g_stamp < 1e-5,
        "10M resistor should stamp very small conductance: {}",
        g_stamp
    );

    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    // Matrices should be finite
    for val in &kernel.s {
        assert!(val.is_finite(), "S element is not finite: {}", val);
    }

    let in_idx = in_idx_raw - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;
    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.input_conductance = 1e-7; // 1/10M

    // With RC = 10M * 1u = 10s = 441,000 samples, the time constant is very long.
    // After 100 samples, output should be very small (barely charged).
    for i in 0..100 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite with 10M resistor, got {}",
            i,
            output
        );
    }
}

/// Impedance mismatch: 10M ohm in parallel with 1 ohm.
/// The G matrix will have entries spanning 7 orders of magnitude.
#[test]
fn test_impedance_mismatch_10m_and_1ohm() {
    let spice = "Mismatch\nR1 in mid 10meg\nR2 mid out 1\nC1 out 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    // S matrix should still be finite despite large condition number
    for val in &kernel.s {
        assert!(
            val.is_finite(),
            "S element not finite in mismatched circuit: {}",
            val
        );
    }

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;
    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.input_conductance = 1e-7;

    for i in 0..100 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite with impedance mismatch, got {}",
            i,
            output
        );
    }
}

// ============================================================================
// 3. Extreme device parameters (very small IS)
// ============================================================================

/// Diode with IS=1e-18 (extremely small saturation current).
/// This makes the exponential extremely steep near the forward voltage.
#[test]
fn test_diode_extreme_small_is() {
    let diode = DiodeShockley::new_room_temp(1e-18, 1.0);

    // Forward bias: should produce finite current
    let i_fwd = diode.current_at(0.7);
    assert!(
        i_fwd.is_finite(),
        "IS=1e-18 diode forward current should be finite: {}",
        i_fwd
    );
    assert!(
        i_fwd > 0.0,
        "IS=1e-18 diode forward current should be positive: {}",
        i_fwd
    );

    // Conductance should be finite
    let g_fwd = diode.conductance_at(0.7);
    assert!(
        g_fwd.is_finite(),
        "IS=1e-18 diode conductance should be finite: {}",
        g_fwd
    );
    assert!(
        g_fwd > 0.0,
        "IS=1e-18 diode conductance should be positive: {}",
        g_fwd
    );

    // Reverse bias: current should be very small (close to -IS)
    let i_rev = diode.current_at(-1.0);
    assert!(i_rev.is_finite());
    assert!(i_rev < 0.0);
    assert!(
        i_rev.abs() < 1e-14,
        "IS=1e-18 reverse current should be tiny: {:.2e}",
        i_rev
    );

    // At V=0, current should be ~0
    let i_zero = diode.current_at(0.0);
    assert!(
        i_zero.abs() < 1e-15,
        "IS=1e-18 at V=0 should be ~0: {:.2e}",
        i_zero
    );
}

/// Diode with IS=1e-18 in a circuit solver.
#[test]
fn test_diode_extreme_small_is_in_circuit() {
    let spice =
        "Stiff Diode\nRin in out 1k\nD1 out 0 D1N4148\nC1 out 0 1u\n.model D1N4148 D(IS=1e-18)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let diode = DiodeShockley::new_room_temp(1e-18, 1.0);
    let devices = vec![DeviceEntry::new_diode(diode, 0)];

    let mut solver = CircuitSolver::new(kernel, devices, in_idx, out_idx).unwrap();
    solver.input_conductance = 0.001;

    for i in 0..200 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite with IS=1e-18 diode, got {}",
            i,
            output
        );
    }
}

/// BJT with IS=1e-18 (very small saturation current).
#[test]
fn test_bjt_extreme_small_is() {
    let bjt = BjtEbersMoll::new_room_temp(1e-18, 200.0, 3.0, melange_devices::BjtPolarity::Npn);

    // Forward active
    let ic = bjt.collector_current(0.7, -5.0);
    assert!(ic.is_finite(), "IS=1e-18 BJT Ic should be finite: {}", ic);
    assert!(
        ic > 0.0,
        "IS=1e-18 BJT Ic should be positive in forward active"
    );

    let ib = bjt.base_current(0.7, -5.0);
    assert!(ib.is_finite(), "IS=1e-18 BJT Ib should be finite: {}", ib);
    assert!(
        ib > 0.0,
        "IS=1e-18 BJT Ib should be positive in forward active"
    );

    // Jacobian should be finite
    let (dic_dvbe, dic_dvbc) = bjt.collector_jacobian(0.7, -5.0);
    assert!(
        dic_dvbe.is_finite(),
        "IS=1e-18 BJT dIc/dVbe should be finite: {}",
        dic_dvbe
    );
    assert!(
        dic_dvbc.is_finite(),
        "IS=1e-18 BJT dIc/dVbc should be finite: {}",
        dic_dvbc
    );
}

/// Diode with very large IS=1e-6 (germanium-like, very leaky).
#[test]
fn test_diode_large_is() {
    let diode = DiodeShockley::new_room_temp(1e-6, 1.5);

    let i_fwd = diode.current_at(0.3);
    assert!(
        i_fwd.is_finite(),
        "Large IS diode forward current should be finite: {}",
        i_fwd
    );
    assert!(i_fwd > 0.0);

    // Reverse current should be significant
    let i_rev = diode.current_at(-1.0);
    assert!(i_rev.is_finite());
    assert!(i_rev < 0.0);
    assert!(
        i_rev.abs() > 1e-8,
        "Large IS diode should have measurable reverse current: {:.2e}",
        i_rev
    );
}

// ============================================================================
// 4. Subnormal float inputs to device functions
// ============================================================================

/// Subnormal floats (very small denormalized numbers) should not cause
/// NaN or Inf in device calculations.
#[test]
fn test_diode_subnormal_voltage() {
    let diode = DiodeShockley::silicon();

    // Subnormal float: smallest positive denormalized f64
    let subnormal: f64 = f64::MIN_POSITIVE / 2.0; // This is subnormal
    assert!(subnormal > 0.0 && subnormal < f64::MIN_POSITIVE);

    let i = diode.current_at(subnormal);
    assert!(
        i.is_finite(),
        "Diode current with subnormal voltage should be finite: {}",
        i
    );

    let g = diode.conductance_at(subnormal);
    assert!(
        g.is_finite(),
        "Diode conductance with subnormal voltage should be finite: {}",
        g
    );

    // Negative subnormal
    let i_neg = diode.current_at(-subnormal);
    assert!(
        i_neg.is_finite(),
        "Diode current with negative subnormal should be finite: {}",
        i_neg
    );
}

/// Subnormal Vbe/Vbc to BJT model.
#[test]
fn test_bjt_subnormal_voltages() {
    let bjt = BjtEbersMoll::npn_2n2222a();
    let subnormal: f64 = f64::MIN_POSITIVE / 2.0;

    let ic = bjt.collector_current(subnormal, -subnormal);
    assert!(
        ic.is_finite(),
        "BJT Ic with subnormal voltages should be finite: {}",
        ic
    );

    let ib = bjt.base_current(subnormal, -subnormal);
    assert!(
        ib.is_finite(),
        "BJT Ib with subnormal voltages should be finite: {}",
        ib
    );

    let (dic_dvbe, dic_dvbc) = bjt.collector_jacobian(subnormal, -subnormal);
    assert!(
        dic_dvbe.is_finite(),
        "BJT dIc/dVbe with subnormal should be finite: {}",
        dic_dvbe
    );
    assert!(
        dic_dvbc.is_finite(),
        "BJT dIc/dVbc with subnormal should be finite: {}",
        dic_dvbc
    );
}

/// Gummel-Poon model with subnormal voltages.
#[test]
fn test_gummel_poon_subnormal_voltages() {
    let gp = melange_devices::BjtGummelPoon::npn_2n2222a();
    let subnormal: f64 = f64::MIN_POSITIVE / 2.0;

    let ic = gp.collector_current(subnormal, -subnormal);
    assert!(
        ic.is_finite(),
        "GP Ic with subnormal voltages should be finite: {}",
        ic
    );

    let jac = gp.jacobian(&[subnormal, -subnormal]);
    assert!(
        jac[0].is_finite(),
        "GP Jac[0] with subnormal should be finite: {}",
        jac[0]
    );
    assert!(
        jac[1].is_finite(),
        "GP Jac[1] with subnormal should be finite: {}",
        jac[1]
    );
}

/// Device functions with exact zero voltage (distinct from subnormal).
#[test]
fn test_device_functions_at_exact_zero() {
    let diode = DiodeShockley::silicon();
    let bjt = BjtEbersMoll::npn_2n2222a();

    // Diode at V=0: I=Is*(exp(0)-1) = 0
    let i_diode = diode.current_at(0.0);
    assert_eq!(i_diode, 0.0, "Diode current at V=0 should be exactly 0");

    let g_diode = diode.conductance_at(0.0);
    assert!(
        g_diode.is_finite() && g_diode > 0.0,
        "Diode conductance at V=0 should be finite and positive: {}",
        g_diode
    );

    // BJT at Vbe=0, Vbc=0
    let ic = bjt.collector_current(0.0, 0.0);
    assert!(ic.abs() < 1e-20, "BJT Ic at (0,0) should be ~0: {:.2e}", ic);

    let ib = bjt.base_current(0.0, 0.0);
    assert!(ib.abs() < 1e-20, "BJT Ib at (0,0) should be ~0: {:.2e}", ib);
}

// ============================================================================
// 5. Near-singular matrix handling (mismatched impedances)
// ============================================================================

/// Circuit with extremely mismatched impedances: 1 ohm and 10M ohm.
/// The A matrix will have a very large condition number.
/// The DK kernel should still produce finite results.
#[test]
fn test_near_singular_extreme_mismatch() {
    // 1 ohm + 10M ohm + 1uF: ratio is 1e7
    let spice = "Mismatch\nR1 in mid 1\nR2 mid out 10meg\nC1 out 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    // The kernel should build successfully despite poor conditioning
    let result = DkKernel::from_mna(&mna, SAMPLE_RATE);
    assert!(
        result.is_ok(),
        "DK kernel should handle 1:10M impedance mismatch: {:?}",
        result.err()
    );

    let kernel = result.unwrap();

    // S matrix elements should all be finite
    for (idx, &val) in kernel.s.iter().enumerate() {
        assert!(
            val.is_finite(),
            "S[{}] = {} is not finite in mismatched circuit",
            idx,
            val
        );
    }
}

/// Circuit where one node has very low impedance to ground (near short).
/// This creates a near-singular submatrix.
#[test]
fn test_near_singular_low_impedance_node() {
    // R1=0.01 ohm (almost a short to ground)
    let spice = "Low Z\nR1 in out 1k\nR2 out 0 0.01\nC1 out 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;
    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.input_conductance = 0.001;

    // The output node is almost shorted to ground, so output should be ~0
    for i in 0..100 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite with 0.01 ohm to ground, got {}",
            i,
            output
        );
        // Output should be very small since the node is nearly shorted
        assert!(
            output.abs() < 0.1,
            "Output[{}] should be small with near-short to ground: {:.6}",
            i,
            output
        );
    }
}

/// Multiple voltage sources create near-singular G matrix blocks.
/// The Norton equivalent stamps G=1e6 S per source.
#[test]
fn test_multiple_voltage_sources_conditioning() {
    let spice =
        "Multi VS\nV1 vcc 0 DC 12\nV2 vee 0 DC -12\nR1 vcc out 1k\nR2 out vee 1k\nC1 out 0 1u\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    // All matrices should be finite
    for &val in &kernel.s {
        assert!(
            val.is_finite(),
            "S element not finite with multiple VS: {}",
            val
        );
    }
    for &val in &kernel.a_neg {
        assert!(
            val.is_finite(),
            "A_neg element not finite with multiple VS: {}",
            val
        );
    }
}

// ============================================================================
// 6. High sample rate edge case
// ============================================================================

/// At 192kHz sample rate, alpha = 2*192000 = 384000.
/// With a 1pF cap, alpha*C = 3.84e-7 which is very small.
/// The A matrix should still be invertible.
#[test]
fn test_high_sample_rate_small_cap() {
    let spice = "High SR\nR1 in out 1k\nC1 out 0 1p\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    let kernel = DkKernel::from_mna(&mna, 192000.0).unwrap();

    for &val in &kernel.s {
        assert!(val.is_finite(), "S not finite at 192kHz with 1pF: {}", val);
    }

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;
    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.input_conductance = 0.001;

    for i in 0..100 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite at 192kHz/1pF, got {}",
            i,
            output
        );
    }
}

// ============================================================================
// 7. safe_exp edge cases (large arguments)
// ============================================================================

/// Verify that safe_exp prevents overflow for very large arguments.
#[test]
fn test_safe_exp_boundary_values() {
    use melange_devices::safeguards;

    // At the boundary
    let exp_40 = safeguards::safe_exp(40.0);
    assert!(
        exp_40.is_finite(),
        "safe_exp(40) should be finite: {}",
        exp_40
    );

    // Beyond the boundary
    let exp_100 = safeguards::safe_exp(100.0);
    assert!(
        exp_100.is_finite(),
        "safe_exp(100) should be finite: {}",
        exp_100
    );
    assert_eq!(exp_100, exp_40, "safe_exp should clamp at 40");

    // Negative boundary
    let exp_neg40 = safeguards::safe_exp(-40.0);
    assert!(exp_neg40.is_finite() && exp_neg40 > 0.0);

    let exp_neg100 = safeguards::safe_exp(-100.0);
    assert!(exp_neg100.is_finite() && exp_neg100 > 0.0);
    assert_eq!(exp_neg100, exp_neg40, "safe_exp should clamp at -40");

    // NaN and Inf
    let exp_nan = safeguards::safe_exp(f64::NAN);
    // NaN < -40 is false, NaN > 40 is false, so safe_exp returns NaN.exp() = NaN
    // This is acceptable -- NaN input is sanitized at higher levels.
    let _ = exp_nan; // Just ensure no panic

    let exp_inf = safeguards::safe_exp(f64::INFINITY);
    assert!(
        exp_inf.is_finite(),
        "safe_exp(inf) should be clamped to finite: {}",
        exp_inf
    );

    let exp_neg_inf = safeguards::safe_exp(f64::NEG_INFINITY);
    assert!(
        exp_neg_inf.is_finite() && exp_neg_inf > 0.0,
        "safe_exp(-inf) should be clamped to small positive: {}",
        exp_neg_inf
    );
}

/// Diode at voltages that would normally cause exp() overflow.
#[test]
fn test_diode_extreme_forward_bias() {
    let diode = DiodeShockley::silicon();

    // Very large forward voltage: without safe_exp this would overflow
    let i_10v = diode.current_at(10.0);
    assert!(
        i_10v.is_finite(),
        "Diode current at 10V should be clamped/finite: {}",
        i_10v
    );

    let g_10v = diode.conductance_at(10.0);
    assert!(
        g_10v.is_finite(),
        "Diode conductance at 10V should be finite: {}",
        g_10v
    );

    // Very large reverse voltage
    let i_neg10v = diode.current_at(-10.0);
    assert!(
        i_neg10v.is_finite(),
        "Diode current at -10V should be finite: {}",
        i_neg10v
    );
    assert!(i_neg10v < 0.0);
}

/// BJT at voltages that stress safe_exp limits.
#[test]
fn test_bjt_extreme_voltages() {
    let bjt = BjtEbersMoll::npn_2n2222a();

    // Large forward Vbe
    let ic_high = bjt.collector_current(2.0, -5.0);
    assert!(
        ic_high.is_finite(),
        "BJT Ic at Vbe=2V should be finite: {}",
        ic_high
    );

    // Large reverse Vbc
    let ic_rev = bjt.collector_current(0.7, -50.0);
    assert!(
        ic_rev.is_finite(),
        "BJT Ic at Vbc=-50V should be finite: {}",
        ic_rev
    );

    // Both extreme
    let ic_both = bjt.collector_current(2.0, 2.0);
    assert!(
        ic_both.is_finite(),
        "BJT Ic at Vbe=Vbc=2V should be finite: {}",
        ic_both
    );

    // Jacobian at extreme voltages
    let (d1, d2) = bjt.collector_jacobian(2.0, -50.0);
    assert!(
        d1.is_finite(),
        "BJT dIc/dVbe at extreme should be finite: {}",
        d1
    );
    assert!(
        d2.is_finite(),
        "BJT dIc/dVbc at extreme should be finite: {}",
        d2
    );
}

// ============================================================================
// 8. Very large circuit with many nodes
// ============================================================================

/// A chain of RC sections (6 nodes) to test that the matrix inversion
/// handles larger systems without precision loss.
#[test]
fn test_large_rc_chain() {
    let spice = "RC Chain\nR1 in n1 1k\nC1 n1 0 100n\nR2 n1 n2 1k\nC2 n2 0 100n\nR3 n2 n3 1k\nC3 n3 0 100n\nR4 n3 n4 1k\nC4 n4 0 100n\nR5 n4 out 1k\nC5 out 0 100n\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();

    assert!(
        mna.n >= 5,
        "RC chain should have at least 5 non-ground nodes, got {}",
        mna.n
    );

    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    // All S matrix elements should be finite
    for (idx, &val) in kernel.s.iter().enumerate() {
        assert!(
            val.is_finite(),
            "S[{}] not finite in RC chain: {}",
            idx,
            val
        );
    }

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;
    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.input_conductance = 0.001;

    for i in 0..500 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite in RC chain, got {}",
            i,
            output
        );
    }

    // After 500 samples, the output should have started charging toward 1V
    let final_output = solver.process_sample(1.0);
    assert!(
        final_output > 0.0,
        "RC chain final output should be positive: {:.6}",
        final_output
    );
}

// ============================================================================
// 9. Diode ideality factor edge cases
// ============================================================================

/// Diode with n=1.0 (ideal) is the stiffest case.
/// Verify it doesn't cause issues.
#[test]
fn test_diode_ideality_factor_1() {
    let diode = DiodeShockley::new_room_temp(1e-15, 1.0);

    // n*Vt = 0.02585V. exp(0.7/0.02585) = exp(27.07) ~ 5.6e11
    let i_fwd = diode.current_at(0.7);
    assert!(i_fwd.is_finite());
    assert!(i_fwd > 0.0);

    let g_fwd = diode.conductance_at(0.7);
    assert!(g_fwd.is_finite());
    assert!(g_fwd > 0.0);
}

/// Diode with n=3.0 (very non-ideal, like an LED or organic diode).
#[test]
fn test_diode_ideality_factor_3() {
    let diode = DiodeShockley::new_room_temp(1e-12, 3.0);

    // n*Vt = 3*0.02585 = 0.0776V. Much softer exponential.
    let i_fwd = diode.current_at(0.7);
    assert!(i_fwd.is_finite());
    assert!(i_fwd > 0.0);

    // At same voltage, n=3 should have less current than n=1 (with same IS)
    let diode_n1 = DiodeShockley::new_room_temp(1e-12, 1.0);
    let i_n1 = diode_n1.current_at(0.7);
    assert!(
        i_fwd < i_n1,
        "n=3 diode should have less current than n=1 at same V: n3={:.2e}, n1={:.2e}",
        i_fwd,
        i_n1
    );
}

// ============================================================================
// 10. Inductor with very small value
// ============================================================================

/// Very small inductor (1uH). g_eq = T/(2L) = (1/44100)/(2*1e-6) = 11.34 S.
/// This is a very high conductance, meaning the inductor acts like a short
/// at audio frequencies.
#[test]
fn test_very_small_inductor() {
    let spice = "Small L\nR1 in out 1k\nL1 out 0 1u\nC1 out 0 100p\n";
    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, SAMPLE_RATE).unwrap();

    assert_eq!(kernel.inductors.len(), 1);
    let g_eq = kernel.inductors[0].g_eq;
    assert!(g_eq.is_finite(), "Inductor g_eq should be finite: {}", g_eq);
    assert!(
        g_eq > 1.0,
        "1uH inductor at 44.1kHz should have large g_eq: {}",
        g_eq
    );

    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;
    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.input_conductance = 0.001;

    for i in 0..200 {
        let output = solver.process_sample(1.0);
        assert!(
            output.is_finite(),
            "Output[{}] must be finite with 1uH inductor, got {}",
            i,
            output
        );
    }
}
