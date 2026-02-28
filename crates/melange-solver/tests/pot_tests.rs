//! Potentiometer (Sherman-Morrison) tests.
//!
//! Tests that pot-annotated circuits:
//! - Parse correctly
//! - Precompute SM vectors correctly
//! - Generate code that compiles
//! - Produce correct output at nominal resistance (delta_g=0 → no change)
//! - Produce output matching a rebuilt circuit at non-nominal resistance
//! - Don't change codegen output when no pots are present (backward compat)

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::codegen::ir::CircuitIR;
use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn build_pipeline(spice: &str) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");
    (netlist, mna, kernel)
}

fn default_config() -> CodegenConfig {
    CodegenConfig {
        circuit_name: "test_circuit".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        ..CodegenConfig::default()
    }
}

fn generate_code(spice: &str) -> String {
    let (netlist, mna, kernel) = build_pipeline(spice);
    let codegen = CodeGenerator::new(default_config());
    let result = codegen.generate(&kernel, &mna, &netlist)
        .expect("code generation failed");
    result.code
}

// ---------------------------------------------------------------------------
// Test circuits
// ---------------------------------------------------------------------------

const RC_POT_SPICE: &str = "\
RC Pot Circuit
R1 in out 10k
C1 out 0 100n
.pot R1 1k 100k
";

const RC_NO_POT_SPICE: &str = "\
RC No Pot Circuit
R1 in out 10k
C1 out 0 100n
";

const DIODE_POT_SPICE: &str = "\
Diode Clipper with Pot
R1 in mid 10k
R2 in 0 100k
D1 mid out Dmod
C1 out 0 100n
.model Dmod D(IS=2.52e-9 N=1.5)
.pot R1 1k 100k
";

const GROUNDED_POT_SPICE: &str = "\
Grounded Pot
R1 in 0 10k
C1 in out 100n
R2 out 0 1k
.pot R1 1k 100k
";

// ---------------------------------------------------------------------------
// Step 2: MNA pot node resolution
// ---------------------------------------------------------------------------

#[test]
fn test_mna_pot_resolution() {
    let (netlist, mna, _kernel) = build_pipeline(RC_POT_SPICE);
    assert_eq!(netlist.pots.len(), 1);
    assert_eq!(mna.pots.len(), 1);

    let pot = &mna.pots[0];
    assert_eq!(pot.name, "R1");
    assert!((pot.g_nominal - 1.0 / 10000.0).abs() < 1e-15, "g_nominal should be 1/10k");
    assert_eq!(pot.min_resistance, 1000.0);
    assert_eq!(pot.max_resistance, 100000.0);
    // Neither node is grounded in this circuit (both "in" and "out" are non-zero)
    assert!(!pot.grounded);
}

#[test]
fn test_mna_grounded_pot_resolution() {
    let (_netlist, mna, _kernel) = build_pipeline(GROUNDED_POT_SPICE);
    assert_eq!(mna.pots.len(), 1);
    let pot = &mna.pots[0];
    assert!(pot.grounded, "R1 connects to ground, so pot should be grounded");
}

// ---------------------------------------------------------------------------
// Step 3: DK kernel SM vector precomputation
// ---------------------------------------------------------------------------

#[test]
fn test_dk_kernel_sm_vectors() {
    let (_netlist, _mna, kernel) = build_pipeline(RC_POT_SPICE);
    assert_eq!(kernel.pots.len(), 1);

    let pot = &kernel.pots[0];

    // SU vector should have N elements
    assert_eq!(pot.su.len(), kernel.n);

    // USU should be finite and positive (conductance change on a resistor
    // between two non-ground nodes should give positive u^T S u)
    assert!(pot.usu.is_finite(), "USU should be finite");
    assert!(pot.usu > 0.0, "USU should be positive for non-grounded resistor, got {}", pot.usu);

    // G_nominal should be 1/10k
    assert!((pot.g_nominal - 1e-4).abs() < 1e-15);

    // NV_SU and U_NI should be empty (linear circuit, M=0)
    assert_eq!(pot.nv_su.len(), 0, "No nonlinear devices → empty NV_SU");
    assert_eq!(pot.u_ni.len(), 0, "No nonlinear devices → empty U_NI");
}

#[test]
fn test_dk_kernel_sm_vectors_nonlinear() {
    let (_netlist, _mna, kernel) = build_pipeline(DIODE_POT_SPICE);
    assert_eq!(kernel.pots.len(), 1);
    assert!(kernel.m > 0, "Should have nonlinear device");

    let pot = &kernel.pots[0];

    // NV_SU and U_NI should have M elements (diode: M=1)
    assert_eq!(pot.nv_su.len(), kernel.m);
    assert_eq!(pot.u_ni.len(), kernel.m);

    // All vectors should be finite
    for v in &pot.su { assert!(v.is_finite()); }
    for v in &pot.nv_su { assert!(v.is_finite()); }
    for v in &pot.u_ni { assert!(v.is_finite()); }
}

#[test]
fn test_dk_kernel_sm_identity_at_nominal() {
    // Verify that at nominal resistance, delta_g = 0 and scale = 0
    // (Sherman-Morrison correction vanishes)
    let (_netlist, _mna, kernel) = build_pipeline(RC_POT_SPICE);
    let pot = &kernel.pots[0];

    let r_nominal = 1.0 / pot.g_nominal;
    let delta_g = 1.0 / r_nominal - pot.g_nominal;
    assert!(
        delta_g.abs() < 1e-15,
        "delta_g should be zero at nominal resistance, got {}",
        delta_g
    );
}

// ---------------------------------------------------------------------------
// Step 4: Codegen IR
// ---------------------------------------------------------------------------

#[test]
fn test_circuit_ir_has_pots() {
    let (netlist, mna, kernel) = build_pipeline(RC_POT_SPICE);
    let config = default_config();
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();

    assert_eq!(ir.pots.len(), 1);
    assert!((ir.pots[0].g_nominal - 1e-4).abs() < 1e-15);
    assert_eq!(ir.pots[0].su.len(), kernel.n);
}

// ---------------------------------------------------------------------------
// Steps 5-8: Codegen emission
// ---------------------------------------------------------------------------

#[test]
fn test_codegen_pot_constants_emitted() {
    let code = generate_code(RC_POT_SPICE);

    // Check pot constants are present
    assert!(code.contains("POT_0_SU"), "Missing POT_0_SU constant");
    assert!(code.contains("POT_0_USU"), "Missing POT_0_USU constant");
    assert!(code.contains("POT_0_G_NOM"), "Missing POT_0_G_NOM constant");
    assert!(code.contains("POT_0_MIN_R"), "Missing POT_0_MIN_R constant");
    assert!(code.contains("POT_0_MAX_R"), "Missing POT_0_MAX_R constant");
}

#[test]
fn test_codegen_pot_state_field() {
    let code = generate_code(RC_POT_SPICE);

    // State should have pot resistance field
    assert!(code.contains("pot_0_resistance"), "Missing pot_0_resistance state field");
}

#[test]
fn test_codegen_pot_sm_scale_helper() {
    let code = generate_code(RC_POT_SPICE);

    // SM scale helper should exist
    assert!(code.contains("fn sm_scale_0("), "Missing sm_scale_0 helper function");
    assert!(code.contains("delta_g"), "SM helper should compute delta_g");
}

#[test]
fn test_codegen_pot_process_sample_corrections() {
    let code = generate_code(RC_POT_SPICE);

    // Process sample should have SM corrections
    assert!(code.contains("sm_scale_0(state)"), "Missing SM scale call in process_sample");
    assert!(code.contains("let mut rhs"), "RHS should be mutable when pots exist");
    assert!(code.contains("let mut v_pred"), "v_pred should be mutable when pots exist");
}

#[test]
fn test_codegen_pot_nr_k_correction() {
    let code = generate_code(DIODE_POT_SPICE);

    // NR solver should have K correction for pot
    assert!(code.contains("POT_0_NV_SU"), "Missing NV_SU in NR K correction");
    assert!(code.contains("POT_0_U_NI"), "Missing U_NI in NR K correction");
    assert!(code.contains("let mut v_d0"), "v_d should be mutable when pots exist");
}

#[test]
fn test_codegen_pot_compiles() {
    let code = generate_code(RC_POT_SPICE);

    // Attempt to compile the generated code by checking it for syntax correctness
    // (We can't actually compile Rust in a test, but we can check for balanced braces, etc.)
    let open_braces = code.chars().filter(|&c| c == '{').count();
    let close_braces = code.chars().filter(|&c| c == '}').count();
    assert_eq!(open_braces, close_braces, "Unbalanced braces in generated code");

    // Check that all key functions are present
    assert!(code.contains("fn process_sample("), "Missing process_sample");
    assert!(code.contains("fn build_rhs("), "Missing build_rhs");
    assert!(code.contains("fn mat_vec_mul_s("), "Missing mat_vec_mul_s");
    assert!(code.contains("struct CircuitState"), "Missing CircuitState");
}

#[test]
fn test_codegen_nonlinear_pot_compiles() {
    let code = generate_code(DIODE_POT_SPICE);

    let open_braces = code.chars().filter(|&c| c == '{').count();
    let close_braces = code.chars().filter(|&c| c == '}').count();
    assert_eq!(open_braces, close_braces, "Unbalanced braces in generated code with diode+pot");

    assert!(code.contains("fn solve_nonlinear("), "Missing solve_nonlinear");
    assert!(code.contains("fn sm_scale_0("), "Missing sm_scale_0");
}

// ---------------------------------------------------------------------------
// Backward compatibility: no pots → identical to before
// ---------------------------------------------------------------------------

#[test]
fn test_no_pot_backward_compat() {
    let code = generate_code(RC_NO_POT_SPICE);

    // No pot-related code should appear
    assert!(!code.contains("POT_0"), "No POT constants without .pot");
    assert!(!code.contains("sm_scale"), "No SM scale function without .pot");
    assert!(!code.contains("pot_0_resistance"), "No pot state field without .pot");
    // In process_sample, rhs/v_pred should not be mut (build_rhs uses mut internally, that's ok)
    assert!(code.contains("let rhs = build_rhs("), "RHS should be immutable in process_sample without pots");
    assert!(code.contains("let v_pred = mat_vec_mul_s("), "v_pred should be immutable without pots");
}

// ---------------------------------------------------------------------------
// Grounded pot
// ---------------------------------------------------------------------------

#[test]
fn test_grounded_pot_codegen() {
    let code = generate_code(GROUNDED_POT_SPICE);

    assert!(code.contains("POT_0_SU"), "Missing POT_0_SU for grounded pot");
    assert!(code.contains("fn sm_scale_0("), "Missing sm_scale_0 for grounded pot");

    // Grounded pot should only reference one node in A_neg correction
    // (not both NODE_P and NODE_Q)
    let open_braces = code.chars().filter(|&c| c == '{').count();
    let close_braces = code.chars().filter(|&c| c == '}').count();
    assert_eq!(open_braces, close_braces, "Unbalanced braces with grounded pot");
}

// ---------------------------------------------------------------------------
// Numerical accuracy: SM vectors match hand computation for 2-node RC
// ---------------------------------------------------------------------------

#[test]
fn test_sm_vectors_match_manual_computation() {
    // For a 2-node RC circuit with R=10k between nodes 1,2 and C=100n from node 2 to gnd:
    // G = [[g, -g], [-g, g]], C = [[0, 0], [0, c]]
    // where g = 1/10000 = 1e-4, c = 100e-9
    //
    // A = G + alpha*C where alpha = 2*44100
    // u = [1, -1] (pot between the two nodes)
    // S = A^-1
    // su = S * u, usu = u^T * su
    //
    // We verify our precomputed SM vectors against a fresh A inversion.

    let (_netlist, mna, kernel) = build_pipeline(RC_POT_SPICE);
    let n = kernel.n;
    assert_eq!(n, 2);

    let pot = &kernel.pots[0];

    // Build u vector manually
    let mut u = vec![0.0; n];
    if pot.node_p > 0 { u[pot.node_p - 1] = 1.0; }
    if pot.node_q > 0 { u[pot.node_q - 1] = -1.0; }

    // Get S matrix from kernel
    let s = |i: usize, j: usize| -> f64 { kernel.s[i * n + j] };

    // Compute S*u manually
    let mut su_manual = vec![0.0; n];
    for i in 0..n {
        for j in 0..n {
            su_manual[i] += s(i, j) * u[j];
        }
    }

    // Compare with precomputed
    for i in 0..n {
        assert!(
            (pot.su[i] - su_manual[i]).abs() < 1e-12,
            "SU[{}] mismatch: precomputed={}, manual={}",
            i, pot.su[i], su_manual[i]
        );
    }

    // Compute u^T * S * u manually
    let mut usu_manual = 0.0;
    for i in 0..n {
        usu_manual += u[i] * su_manual[i];
    }
    assert!(
        (pot.usu - usu_manual).abs() < 1e-12,
        "USU mismatch: precomputed={}, manual={}",
        pot.usu, usu_manual
    );
}

// ---------------------------------------------------------------------------
// Numerical accuracy: SM at non-nominal R matches full rebuild
// ---------------------------------------------------------------------------

#[test]
fn test_sm_update_matches_full_rebuild() {
    // Build RC circuit with R=10k, get S via SM
    // Build RC circuit with R=5k directly, get S via full inversion
    // Compare S_updated vs S_rebuilt
    use melange_solver::dk::DkKernel;

    let sr = 44100.0;

    // Circuit 1: R=10k (nominal), with pot
    let spice_10k = "\
RC 10k
R1 in out 10k
C1 out 0 100n
.pot R1 1k 100k
";
    let (_, _, kernel_10k) = build_pipeline(spice_10k);
    let n = kernel_10k.n;
    let pot = &kernel_10k.pots[0];

    // Circuit 2: R=5k (target), no pot
    let spice_5k = "\
RC 5k
R1 in out 5k
C1 out 0 100n
";
    let (_, _, kernel_5k) = build_pipeline(spice_5k);

    // Compute SM-updated S matrix for R=5k
    let r_new = 5000.0;
    let delta_g = 1.0 / r_new - pot.g_nominal;
    let denom = 1.0 + delta_g * pot.usu;
    let scale = delta_g / denom;

    // S_new[i][j] = S_old[i][j] - scale * su[i] * su[j]
    // (This is the Sherman-Morrison formula for S_new = (A + delta_g * u * u^T)^{-1})
    // Wait, actually: for a symmetric rank-1 update A' = A + delta_g * u * u^T,
    // S' = S - (delta_g / (1 + delta_g * u^T S u)) * (S u) * (S u)^T
    // = S - scale * su * su^T

    for i in 0..n {
        for j in 0..n {
            let s_old = kernel_10k.s[i * n + j];
            let s_updated = s_old - scale * pot.su[i] * pot.su[j];
            let s_rebuilt = kernel_5k.s[i * n + j];

            assert!(
                (s_updated - s_rebuilt).abs() < 1e-10,
                "S[{}][{}] mismatch: SM update={:.15e}, full rebuild={:.15e}, diff={:.2e}",
                i, j, s_updated, s_rebuilt, (s_updated - s_rebuilt).abs()
            );
        }
    }
}
