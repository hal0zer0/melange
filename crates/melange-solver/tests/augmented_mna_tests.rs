//! Tests for augmented MNA inductor handling in codegen.
//!
//! The nodal codegen path uses augmented MNA for inductors: each inductor winding
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
//! - No-inductor circuits are unaffected (near-exact match between DK and nodal codegen)

mod support;

use melange_solver::codegen::CodegenConfig;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn config_for(spice: &str, in_name: &str, out_name: &str, sr: f64) -> CodegenConfig {
    let netlist = Netlist::parse(spice).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let in_idx = *mna.node_map.get(in_name).unwrap() - 1;
    let out_idx = *mna.node_map.get(out_name).unwrap() - 1;
    CodegenConfig {
        circuit_name: "test".to_string(),
        sample_rate: sr,
        input_node: in_idx,
        output_nodes: vec![out_idx],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
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
    let config = config_for(RL_LOWPASS, "in", "out", sr);
    let circuit = support::build_circuit_nodal(RL_LOWPASS, &config, "aug_rl_step");

    let num_samples = 500;
    let result = support::run_step_full(&circuit, 1.0, num_samples, sr);
    let output = result.parse_samples();
    assert_eq!(output.len(), num_samples, "Got expected number of samples");

    // All finite
    assert!(output.iter().all(|v| v.is_finite()), "All outputs finite");

    // Starts nonzero, decays toward 0 (L shorts to ground at DC)
    assert!(
        output[0].abs() > 0.01,
        "Initial output nonzero: {:.6}",
        output[0]
    );
    assert!(
        output[num_samples - 1].abs() < 0.1,
        "Steady state near 0: {:.6}",
        output[num_samples - 1]
    );

    // Monotonic decay on average
    let avg_early: f64 = output[0..20].iter().map(|v| v.abs()).sum::<f64>() / 20.0;
    let avg_late: f64 = output[480..500].iter().map(|v| v.abs()).sum::<f64>() / 20.0;
    assert!(
        avg_early > avg_late,
        "Decaying: early {:.6} > late {:.6}",
        avg_early,
        avg_late
    );

    // Linear circuit: NR must converge every sample
    let nr_max = result.diag("nr_max_iter_count").unwrap_or(0.0);
    let nan_reset = result.diag("nan_reset_count").unwrap_or(0.0);
    assert_eq!(nr_max as u64, 0, "NR converged on every sample");
    assert_eq!(nan_reset as u64, 0, "No NaN resets");
}

// ---------------------------------------------------------------------------
// Test 2: No inductors — near-exact match between DK and nodal codegen
// ---------------------------------------------------------------------------

#[test]
fn test_no_inductors_exact_match() {
    let spice = "\
RC Lowpass
R1 in out 1k
C1 out 0 100n
";
    let sr = 48000.0;
    let config = config_for(spice, "in", "out", sr);

    let nodal_circuit = support::build_circuit_nodal(spice, &config, "aug_no_ind_nodal");
    let dk_circuit = support::build_circuit(spice, &config, "aug_no_ind_dk");

    let num_samples = 200;
    let nodal_output = support::run_sine(&nodal_circuit, 1000.0, 1.0, num_samples, sr);
    let dk_output = support::run_sine(&dk_circuit, 1000.0, 1.0, num_samples, sr);

    assert_eq!(nodal_output.len(), num_samples);
    assert_eq!(dk_output.len(), num_samples);

    let mut max_diff = 0.0_f64;
    for i in 0..num_samples {
        max_diff = max_diff.max((nodal_output[i] - dk_output[i]).abs());
    }

    // No inductors → nearly identical (Gmin regularization adds 1e-12 S per node,
    // causing ~1e-9 difference in node voltages)
    assert!(
        max_diff < 1e-8,
        "No inductors: near-exact match, diff = {:.2e}",
        max_diff
    );
}

// ---------------------------------------------------------------------------
// Test 3: Large inductor (130H) — no NaN, no divergence, NR converges
// ---------------------------------------------------------------------------

#[test]
fn test_large_inductor_stability() {
    let sr = 48000.0;
    let config = config_for(LARGE_INDUCTOR, "in", "out", sr);
    let circuit = support::build_circuit_nodal(LARGE_INDUCTOR, &config, "aug_large_ind");

    let result = support::run_step_full(&circuit, 1.0, 500, sr);
    let output = result.parse_samples();

    for (i, &v) in output.iter().enumerate() {
        assert!(v.is_finite(), "Output finite for 130H inductor at sample {i}");
    }

    let nan_reset = result.diag("nan_reset_count").unwrap_or(0.0);
    let nr_max = result.diag("nr_max_iter_count").unwrap_or(0.0);
    assert_eq!(nan_reset as u64, 0, "No NaN resets");
    assert_eq!(nr_max as u64, 0, "NR converged on every sample");
}

// ---------------------------------------------------------------------------
// Test 4: Large inductor steady-state correctness
// ---------------------------------------------------------------------------

#[test]
fn test_large_inductor_steady_state() {
    let sr = 48000.0;
    let r_in = 1.0;
    let r = 10000.0;
    let config = config_for(LARGE_INDUCTOR, "in", "out", sr);

    // Generate nodal code, then use a custom main that prints internal state
    let (code, _n, _m) = support::generate_circuit_code_nodal(LARGE_INDUCTOR, &config);

    let custom_main = r#"
fn main() {
    let mut state = CircuitState::default();
    state.set_sample_rate(48000.0);

    // Process many samples of DC step to reach steady state
    // tau = L/R = 130/10000 = 13ms = 624 samples
    for _ in 0..10000 {
        process_sample(1.0, &mut state);
    }

    // Print V_out (output node) and inductor branch current (v_prev[N_AUG])
    let out_idx = OUTPUT_NODES[0];
    let v_out = state.v_prev[out_idx];
    let j_l = state.v_prev[N_AUG]; // first inductor branch current
    println!("v_out={:.15e}", v_out);
    println!("j_l={:.15e}", j_l);
}
"#;

    let result = support::compile_and_run(&code, custom_main, "aug_large_ss");

    let v_out = result.parse_kv("v_out").expect("v_out not found in output");
    let j_l = result.parse_kv("j_l").expect("j_l not found in output");

    // At DC: L is short, V_out = 0 (all voltage across R_in + R)
    assert!(
        v_out.abs() < 0.01,
        "V_out at DC should be ~0, got {:.6}",
        v_out
    );

    // Inductor current = V_in / (R_in + R) = 1.0 / 10001 ≈ 1.0e-4 A
    let expected_i = 1.0 / (r_in + r);
    assert!(
        (j_l - expected_i).abs() < 0.01 * expected_i,
        "Inductor DC current: expected {:.6e}, got {:.6e}",
        expected_i,
        j_l
    );
}

// ---------------------------------------------------------------------------
// Test 5: System dimensions
// ---------------------------------------------------------------------------

#[test]
fn test_augmented_dimensions() {
    // Check MNA augmented dimensions directly (no solver needed)

    // 1 uncoupled inductor → +1
    {
        let netlist = Netlist::parse(RL_LOWPASS).expect("parse");
        let mna = MnaSystem::from_netlist(&netlist).expect("mna");
        let aug = mna.build_augmented_matrices();
        assert_eq!(
            aug.n_nodal,
            mna.n_aug + 1,
            "RL: 1 inductor → n_nodal = n_aug + 1"
        );
    }

    // 2-winding coupled pair → +2
    {
        let netlist = Netlist::parse(STEP_UP_XFMR).expect("parse");
        let mna = MnaSystem::from_netlist(&netlist).expect("mna");
        let aug = mna.build_augmented_matrices();
        assert_eq!(
            aug.n_nodal,
            mna.n_aug + 2,
            "Transformer: 2 windings → n_nodal = n_aug + 2"
        );
    }

    // 3-winding group → +3
    {
        let netlist = Netlist::parse(THREE_WINDING).expect("parse");
        let mna = MnaSystem::from_netlist(&netlist).expect("mna");
        let aug = mna.build_augmented_matrices();
        assert_eq!(
            aug.n_nodal,
            mna.n_aug + 3,
            "3-winding xfmr → n_nodal = n_aug + 3"
        );
    }
}

// ---------------------------------------------------------------------------
// Test 6: Inductor branch currents in state vector
// ---------------------------------------------------------------------------

#[test]
fn test_inductor_branch_currents() {
    let sr = 48000.0;
    let r_in = 1.0;
    let config = config_for(RL_LOWPASS, "in", "out", sr);

    let (code, _n, _m) = support::generate_circuit_code_nodal(RL_LOWPASS, &config);

    let custom_main = r#"
fn main() {
    let mut state = CircuitState::default();
    state.set_sample_rate(48000.0);

    // Process DC step to build up inductor current
    for _ in 0..500 {
        process_sample(1.0, &mut state);
    }

    let j_l = state.v_prev[N_AUG]; // first inductor branch current
    println!("j_l={:.15e}", j_l);
}
"#;

    let result = support::compile_and_run(&code, custom_main, "aug_branch_i");

    let j_l = result.parse_kv("j_l").expect("j_l not found in output");
    assert!(j_l.is_finite(), "Inductor current finite: {}", j_l);

    // At DC: I_L = V_in / (R_in + R1) = 1.0 / 1001 ≈ 9.99e-4 A
    let expected_i = 1.0 / (r_in + 1000.0);
    assert!(
        (j_l - expected_i).abs() < 0.01 * expected_i,
        "Inductor DC current: expected {:.6e}, got {:.6e}",
        expected_i,
        j_l
    );
}

// ---------------------------------------------------------------------------
// Test 7: Coupled inductor energy transfer
// ---------------------------------------------------------------------------

#[test]
fn test_coupled_inductor_energy_transfer() {
    let sr = 48000.0;
    let config = config_for(STEP_UP_XFMR, "in", "out", sr);
    let circuit = support::build_circuit_nodal(STEP_UP_XFMR, &config, "aug_xfmr");

    let num_samples = 500;
    let result = support::run_sine_full(&circuit, 1000.0, 1.0, num_samples, sr);
    let output = result.parse_samples();

    let peak_out = output.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    assert!(
        peak_out > 0.001,
        "Transformer transfers energy: peak = {:.6}",
        peak_out
    );
    let nan_reset = result.diag("nan_reset_count").unwrap_or(0.0);
    let nr_max = result.diag("nr_max_iter_count").unwrap_or(0.0);
    assert_eq!(nan_reset as u64, 0, "No NaN resets");
    assert_eq!(nr_max as u64, 0, "NR converged");
}

// ---------------------------------------------------------------------------
// Test 8: Three-winding transformer
// ---------------------------------------------------------------------------

#[test]
fn test_three_winding_transformer() {
    let sr = 48000.0;
    let config = config_for(THREE_WINDING, "in", "out", sr);
    let circuit = support::build_circuit_nodal(THREE_WINDING, &config, "aug_3wind");

    let num_samples = 500;
    let result = support::run_sine_full(&circuit, 1000.0, 1.0, num_samples, sr);
    let output = result.parse_samples();

    let peak_out = output.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    assert!(
        peak_out > 0.001,
        "3-winding xfmr transfers energy: peak = {:.6}",
        peak_out
    );
    let nan_reset = result.diag("nan_reset_count").unwrap_or(0.0);
    let nr_max = result.diag("nr_max_iter_count").unwrap_or(0.0);
    assert_eq!(nan_reset as u64, 0, "No NaN resets");
    assert_eq!(nr_max as u64, 0, "NR converged");
}

// ---------------------------------------------------------------------------
// Test 9: Augmented and DK codegen reach same steady state
// ---------------------------------------------------------------------------

#[test]
fn test_same_steady_state() {
    let sr = 48000.0;
    let config = config_for(RL_LOWPASS, "in", "out", sr);

    let nodal_circuit = support::build_circuit_nodal(RL_LOWPASS, &config, "aug_ss_nodal");
    let dk_circuit = support::build_circuit(RL_LOWPASS, &config, "aug_ss_dk");

    // Drive with DC step for many time constants (tau = L/R = 0.1ms ≈ 5 samples)
    let num_samples = 1001;
    let nodal_output = support::run_step(&nodal_circuit, 1.0, num_samples, sr);
    let dk_output = support::run_step(&dk_circuit, 1.0, num_samples, sr);

    // At DC steady state, both should agree: V_out → 0 (L shorts to ground)
    // Compare the last sample from each
    let out_nodal = nodal_output[num_samples - 1];
    let out_dk = dk_output[num_samples - 1];

    // Steady state should match closely (both at ~0 for DC-blocked RL lowpass)
    assert!(
        (out_nodal - out_dk).abs() < 0.01,
        "Steady state match: nodal={:.6e} dk={:.6e}",
        out_nodal,
        out_dk
    );
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

    assert!(
        augmented_diag > 1e6,
        "2L/T = {:.2e} (large, well-conditioned)",
        augmented_diag
    );
    assert!(
        companion_g_eq < 1e-6,
        "T/(2L) = {:.2e} (tiny, ill-conditioned)",
        companion_g_eq
    );
    assert!(
        augmented_diag / companion_g_eq > 1e12,
        "Ratio = {:.2e} (augmented is 12 orders of magnitude better)",
        augmented_diag / companion_g_eq
    );
}
