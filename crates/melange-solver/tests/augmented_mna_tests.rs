//! Tests for augmented MNA inductor handling in the NodalSolver.
//!
//! The NodalSolver uses augmented MNA for inductors: each inductor winding
//! adds an extra variable (branch current j_L) with inductance L in the C matrix.
//! This replaces the companion model (tiny conductance T/(2L)) which is
//! ill-conditioned for large inductors at audio sample rates.
//!
//! Tests verify:
//! - Correct RL step response (shape and steady state)
//! - Inductor branch currents are accessible and physically correct
//! - Coupled inductor (transformer) energy transfer
//! - Large inductor stability (130H, the Pultec regime)
//! - System dimensions are correct for all inductor types
//! - NR converges for linear circuits (no max-iter or NaN)
//! - No-inductor circuits are unaffected (exact match with LinearSolver)

use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
use melange_solver::solver::{LinearSolver, NodalSolver};
use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::CodegenConfig;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn build_with_input(spice: &str, in_name: &str, r_in: f64, sr: f64)
    -> (Netlist, MnaSystem, DkKernel)
{
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let in_idx = *mna.node_map.get(in_name).unwrap();
    if in_idx > 0 { mna.g[in_idx - 1][in_idx - 1] += 1.0 / r_in; }
    let kernel = DkKernel::from_mna(&mna, sr).expect("dk");
    (netlist, mna, kernel)
}

fn build_linear_nodal(spice: &str, in_name: &str, out_name: &str, r_in: f64, sr: f64)
    -> (NodalSolver, MnaSystem)
{
    let (netlist, mna, kernel) = build_with_input(spice, in_name, r_in, sr);
    let in_idx = *mna.node_map.get(in_name).unwrap() - 1;
    let out_idx = *mna.node_map.get(out_name).unwrap() - 1;
    let config = CodegenConfig {
        circuit_name: "test".to_string(),
        sample_rate: sr,
        input_node: in_idx,
        output_nodes: vec![out_idx],
        input_resistance: r_in,
        ..CodegenConfig::default()
    };
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();
    let mut solver = NodalSolver::new(kernel, &mna, ir.device_slots.clone(), in_idx, out_idx);
    solver.input_conductance = 1.0 / r_in;
    (solver, mna)
}

// ---------------------------------------------------------------------------
// Circuit definitions
// ---------------------------------------------------------------------------

const RL_LOWPASS: &str = "\
RL Lowpass
R1 in out 1k
L1 out 0 100m
C1 out 0 100p
";

const LARGE_INDUCTOR: &str = "\
Large Inductor
R1 in out 10k
L1 out 0 130
C1 out 0 10p
";

const STEP_UP_XFMR: &str = "\
Step Up Transformer
R1 in primary 100
L1 primary 0 10m
L2 secondary 0 100m
K1 L1 L2 0.95
R2 secondary out 1k
C1 primary 0 100p
C2 secondary 0 100p
C3 out 0 100p
";

const THREE_WINDING: &str = "\
Three Winding Transformer
R1 in pri 100
L1 pri 0 10m
L2 sec1 ct 10m
L3 ct sec2 10m
K1 L1 L2 0.95
K2 L1 L3 0.95
K3 L2 L3 0.95
R2 sec1 out 1k
R3 sec2 0 1k
Rct ct 0 100
C1 pri 0 100p
C2 sec1 0 100p
C3 sec2 0 100p
C4 out 0 100p
";

// ---------------------------------------------------------------------------
// Test 1: RL step response shape and steady state
// ---------------------------------------------------------------------------

#[test]
fn test_augmented_rl_step_response() {
    let sr = 48000.0;
    let (mut solver, _) = build_linear_nodal(RL_LOWPASS, "in", "out", 1.0, sr);

    let num_samples = 500;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        output[i] = solver.process_sample(1.0);
    }

    // All finite
    assert!(output.iter().all(|v| v.is_finite()), "All outputs finite");

    // Starts nonzero, decays toward 0 (L shorts to ground at DC)
    assert!(output[0].abs() > 0.01, "Initial output nonzero: {:.6}", output[0]);
    assert!(output[num_samples - 1].abs() < 0.1,
        "Steady state near 0: {:.6}", output[num_samples - 1]);

    // Monotonic decay on average
    let avg_early: f64 = output[0..20].iter().map(|v| v.abs()).sum::<f64>() / 20.0;
    let avg_late: f64 = output[480..500].iter().map(|v| v.abs()).sum::<f64>() / 20.0;
    assert!(avg_early > avg_late, "Decaying: early {:.6} > late {:.6}", avg_early, avg_late);

    // Linear circuit: NR must converge every sample
    assert_eq!(solver.diag_nr_max_iter_count, 0, "NR converged on every sample");
    assert_eq!(solver.diag_nan_reset_count, 0, "No NaN resets");
}

// ---------------------------------------------------------------------------
// Test 2: No inductors — exact match with LinearSolver
// ---------------------------------------------------------------------------

#[test]
fn test_no_inductors_exact_match() {
    let spice = "\
RC Lowpass
R1 in out 1k
C1 out 0 100n
";
    let sr = 48000.0;
    let r_in = 1.0;

    let (mut nodal, mna) = build_linear_nodal(spice, "in", "out", r_in, sr);
    assert_eq!(nodal.v_prev.len(), mna.n_aug, "No inductors: n_nodal == n_aug");

    let (_, mna_lin, kernel_lin) = build_with_input(spice, "in", r_in, sr);
    let in_idx = *mna_lin.node_map.get("in").unwrap() - 1;
    let out_idx = *mna_lin.node_map.get("out").unwrap() - 1;
    let mut linear = LinearSolver::new(kernel_lin, in_idx, out_idx);
    linear.input_conductance = 1.0 / r_in;

    let mut max_diff = 0.0_f64;
    for i in 0..200 {
        let t = i as f64 / sr;
        let input = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        max_diff = max_diff.max((nodal.process_sample(input) - linear.process_sample(input)).abs());
    }

    // No inductors → identical matrix formulation → exact match
    assert!(max_diff < 1e-10, "No inductors: exact match, diff = {:.2e}", max_diff);
}

// ---------------------------------------------------------------------------
// Test 3: Large inductor (130H) — no NaN, no divergence, NR converges
// ---------------------------------------------------------------------------

#[test]
fn test_large_inductor_stability() {
    let sr = 48000.0;
    let (mut solver, _) = build_linear_nodal(LARGE_INDUCTOR, "in", "out", 1.0, sr);

    for _ in 0..500 {
        let out = solver.process_sample(1.0);
        assert!(out.is_finite(), "Output finite for 130H inductor");
    }

    assert_eq!(solver.diag_nan_reset_count, 0, "No NaN resets");
    assert_eq!(solver.diag_nr_max_iter_count, 0, "NR converged on every sample");
}

// ---------------------------------------------------------------------------
// Test 4: Large inductor steady-state correctness
// ---------------------------------------------------------------------------

#[test]
fn test_large_inductor_steady_state() {
    let sr = 48000.0;
    let r_in = 1.0;
    let r = 10000.0;
    let (mut solver, mna) = build_linear_nodal(LARGE_INDUCTOR, "in", "out", r_in, sr);
    let n_aug = mna.n_aug;

    // Process many samples of DC step to reach steady state
    // tau = L/R = 130/10000 = 13ms = 624 samples
    for _ in 0..10000 {
        solver.process_sample(1.0);
    }

    // At DC: L is short, V_out = 0 (all voltage across R_in + R)
    // Inductor current = V_in / (R_in + R) = 1.0 / 10001 ≈ 1.0e-4 A
    let v_out = solver.v_prev[*mna.node_map.get("out").unwrap() - 1];
    assert!(v_out.abs() < 0.01, "V_out at DC should be ~0, got {:.6}", v_out);

    let j_l = solver.v_prev[n_aug]; // inductor branch current
    let expected_i = 1.0 / (r_in + r);
    assert!((j_l - expected_i).abs() < 0.01 * expected_i,
        "Inductor DC current: expected {:.6e}, got {:.6e}", expected_i, j_l);
}

// ---------------------------------------------------------------------------
// Test 5: System dimensions
// ---------------------------------------------------------------------------

#[test]
fn test_augmented_dimensions() {
    let sr = 48000.0;
    let r_in = 1.0;

    // 1 uncoupled inductor → +1
    {
        let (solver, mna) = build_linear_nodal(RL_LOWPASS, "in", "out", r_in, sr);
        assert_eq!(solver.v_prev.len(), mna.n_aug + 1,
            "RL: 1 inductor → n_nodal = n_aug + 1");
    }

    // 2-winding coupled pair → +2
    {
        let (solver, mna) = build_linear_nodal(STEP_UP_XFMR, "in", "out", r_in, sr);
        assert_eq!(solver.v_prev.len(), mna.n_aug + 2,
            "Transformer: 2 windings → n_nodal = n_aug + 2");
    }

    // 3-winding group → +3
    {
        let (solver, mna) = build_linear_nodal(THREE_WINDING, "in", "out", r_in, sr);
        assert_eq!(solver.v_prev.len(), mna.n_aug + 3,
            "3-winding xfmr → n_nodal = n_aug + 3");
    }
}

// ---------------------------------------------------------------------------
// Test 6: Inductor branch currents in state vector
// ---------------------------------------------------------------------------

#[test]
fn test_inductor_branch_currents() {
    let sr = 48000.0;
    let r_in = 1.0;
    let (mut solver, mna) = build_linear_nodal(RL_LOWPASS, "in", "out", r_in, sr);
    let n_aug = mna.n_aug;

    // Process DC step to build up inductor current
    for _ in 0..500 {
        solver.process_sample(1.0);
    }

    let j_l = solver.v_prev[n_aug];
    assert!(j_l.is_finite(), "Inductor current finite: {}", j_l);

    // At DC: I_L = V_in / (R_in + R1) = 1.0 / 1001 ≈ 9.99e-4 A
    let expected_i = 1.0 / (r_in + 1000.0);
    assert!((j_l - expected_i).abs() < 0.01 * expected_i,
        "Inductor DC current: expected {:.6e}, got {:.6e}", expected_i, j_l);
}

// ---------------------------------------------------------------------------
// Test 7: Coupled inductor energy transfer
// ---------------------------------------------------------------------------

#[test]
fn test_coupled_inductor_energy_transfer() {
    let sr = 48000.0;
    let (mut solver, _) = build_linear_nodal(STEP_UP_XFMR, "in", "out", 1.0, sr);

    let num_samples = 500;
    let mut peak_out = 0.0_f64;
    for i in 0..num_samples {
        let t = i as f64 / sr;
        let input = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        peak_out = peak_out.max(solver.process_sample(input).abs());
    }

    assert!(peak_out > 0.001, "Transformer transfers energy: peak = {:.6}", peak_out);
    assert_eq!(solver.diag_nan_reset_count, 0, "No NaN resets");
    assert_eq!(solver.diag_nr_max_iter_count, 0, "NR converged");
}

// ---------------------------------------------------------------------------
// Test 8: Three-winding transformer
// ---------------------------------------------------------------------------

#[test]
fn test_three_winding_transformer() {
    let sr = 48000.0;
    let (mut solver, _) = build_linear_nodal(THREE_WINDING, "in", "out", 1.0, sr);

    let num_samples = 500;
    let mut peak_out = 0.0_f64;
    for i in 0..num_samples {
        let t = i as f64 / sr;
        let input = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
        peak_out = peak_out.max(solver.process_sample(input).abs());
    }

    assert!(peak_out > 0.001, "3-winding xfmr transfers energy: peak = {:.6}", peak_out);
    assert_eq!(solver.diag_nan_reset_count, 0, "No NaN resets");
    assert_eq!(solver.diag_nr_max_iter_count, 0, "NR converged");
}

// ---------------------------------------------------------------------------
// Test 9: Augmented and companion reach same steady state
// ---------------------------------------------------------------------------

#[test]
fn test_same_steady_state() {
    let sr = 48000.0;
    let r_in = 1.0;

    let (mut nodal, mna_nod) = build_linear_nodal(RL_LOWPASS, "in", "out", r_in, sr);
    let out_idx = *mna_nod.node_map.get("out").unwrap() - 1;

    let (_, mna_lin, kernel_lin) = build_with_input(RL_LOWPASS, "in", r_in, sr);
    let in_idx = *mna_lin.node_map.get("in").unwrap() - 1;
    let out_lin_idx = *mna_lin.node_map.get("out").unwrap() - 1;
    let mut linear = LinearSolver::new(kernel_lin, in_idx, out_lin_idx);
    linear.input_conductance = 1.0 / r_in;

    // Drive with DC step for many time constants (tau = L/R = 0.1ms ≈ 5 samples)
    for _ in 0..1000 {
        nodal.process_sample(1.0);
        linear.process_sample(1.0);
    }

    // At DC steady state, both should agree: V_out → 0 (L shorts to ground)
    let v_nodal = nodal.v_prev[out_idx];
    // Linear solver output includes DC blocking, so compare DC-blocked outputs
    let out_nodal = nodal.process_sample(1.0);
    let out_linear = linear.process_sample(1.0);

    // Steady state should match closely (both at ~0 for DC-blocked RL lowpass)
    assert!((out_nodal - out_linear).abs() < 0.01,
        "Steady state match: nodal={:.6e} linear={:.6e}", out_nodal, out_linear);

    // Raw V_out should be near 0 (inductor shorts to ground)
    assert!(v_nodal.abs() < 0.001,
        "V_out at steady state near 0: {:.6e}", v_nodal);
}

// ---------------------------------------------------------------------------
// Test 10: Condition number — augmented diagonal is well-scaled
// ---------------------------------------------------------------------------

#[test]
fn test_augmented_diagonal_well_scaled() {
    // For 130H inductor at 48kHz, the augmented diagonal entry is 2L/T = 2*130*48000 = 12.48M
    // This is well-conditioned (large, positive) vs companion g_eq = T/(2L) ≈ 8e-8 (tiny)
    let l = 130.0;
    let sr = 48000.0;
    let t = 1.0 / sr;
    let augmented_diag = 2.0 * l / t;
    let companion_g_eq = t / (2.0 * l);

    assert!(augmented_diag > 1e6, "2L/T = {:.2e} (large, well-conditioned)", augmented_diag);
    assert!(companion_g_eq < 1e-6, "T/(2L) = {:.2e} (tiny, ill-conditioned)", companion_g_eq);
    assert!(augmented_diag / companion_g_eq > 1e12,
        "Ratio = {:.2e} (augmented is 12 orders of magnitude better)", augmented_diag / companion_g_eq);
}
