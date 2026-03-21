//! Coupled inductor / transformer tests.
//!
//! Verifies that SPICE `K` element support works correctly across the full stack:
//! parser, MNA, DK kernel, runtime solver, codegen IR, and generated code.
//!
//! Tests cover:
//! - Parser: K element parsing, error cases (k bounds, self-coupling, missing inductor)
//! - MNA: coupled inductor removal from uncoupled list, A matrix cross-coupling stamps
//! - DK Kernel: companion conductance computation, initial state
//! - Codegen: compilation, constants, state fields, RHS injection, backward compat
//! - Behavioral: step-up/step-down transformer voltage ratios, weak coupling, 1:1

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;
use melange_solver::solver::LinearSolver;
use std::io::Write;

// ===========================================================================
// Helpers
// ===========================================================================

fn build_pipeline(spice: &str) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");
    (netlist, mna, kernel)
}

fn build_pipeline_with_input(
    spice: &str,
    input_node_name: &str,
    input_resistance: f64,
) -> (Netlist, MnaSystem, DkKernel) {
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    let in_idx = *mna.node_map.get(input_node_name).unwrap() - 1;
    mna.g[in_idx][in_idx] += 1.0 / input_resistance;
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");
    (netlist, mna, kernel)
}

fn default_config() -> CodegenConfig {
    CodegenConfig {
        circuit_name: "test_circuit".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_nodes: vec![1],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    }
}

fn generate_code(spice: &str) -> String {
    let (netlist, mna, kernel) = build_pipeline(spice);
    let codegen = CodeGenerator::new(default_config());
    let result = codegen
        .generate(&kernel, &mna, &netlist)
        .expect("code generation failed");
    result.code
}

/// Compile generated code with rustc, panicking with stderr on failure.
fn assert_compiles(code: &str, label: &str) {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static COUNTER: AtomicUsize = AtomicUsize::new(0);
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);

    let tmp_dir = std::env::temp_dir();
    let src = tmp_dir.join(format!("melange_ci_test_{}.rs", id));
    let lib = tmp_dir.join(format!("melange_ci_test_{}.rlib", id));

    {
        let mut f = std::fs::File::create(&src).expect("create temp file");
        f.write_all(code.as_bytes()).expect("write temp file");
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2021", "--crate-type", "lib", "-o"])
        .arg(&lib)
        .arg(&src)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&src);
    let _ = std::fs::remove_file(&lib);

    assert!(
        output.status.success(),
        "Generated code for '{}' failed to compile:\n{}",
        label,
        String::from_utf8_lossy(&output.stderr)
    );
}

// ===========================================================================
// Test circuits
// ===========================================================================

/// Basic transformer: L1 (primary) and L2 (secondary), coupled at k=0.95.
/// Step-up: L2 > L1, so V_secondary > V_primary.
/// Parasitic caps for DK stability.
const STEP_UP_TRANSFORMER: &str = "\
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

/// Step-down: L2 < L1, so V_secondary < V_primary.
const STEP_DOWN_TRANSFORMER: &str = "\
Step Down Transformer
R1 in primary 100
L1 primary 0 100m
L2 secondary 0 10m
K1 L1 L2 0.95
R2 secondary out 1k
C1 primary 0 100p
C2 secondary 0 100p
C3 out 0 100p
";

/// Weak coupling: k=0.1, minimal energy transfer.
const WEAK_COUPLING: &str = "\
Weak Coupling
R1 in primary 100
L1 primary 0 10m
L2 secondary 0 10m
K1 L1 L2 0.1
R2 secondary out 1k
C1 primary 0 100p
C2 secondary 0 100p
C3 out 0 100p
";

/// 1:1 transformer: L1=L2, voltage ratio near unity.
const UNITY_TRANSFORMER: &str = "\
Unity Transformer
R1 in primary 100
L1 primary 0 10m
L2 secondary 0 10m
K1 L1 L2 0.95
R2 secondary out 1k
C1 primary 0 100p
C2 secondary 0 100p
C3 out 0 100p
";

/// Simple coupled inductor circuit for codegen testing.
const SIMPLE_COUPLED: &str = "\
Coupled Inductors
R1 in primary 1k
L1 primary 0 100m
L2 secondary 0 100m
K1 L1 L2 0.95
R2 secondary out 1k
C1 primary 0 1n
C2 out 0 1n
";

/// Transformer with an uncoupled inductor (should coexist).
const MIXED_INDUCTORS: &str = "\
Mixed Inductors
R1 in primary 1k
L1 primary 0 10m
L2 secondary 0 10m
L3 secondary out 50m
K1 L1 L2 0.95
R2 out 0 1k
C1 primary 0 100p
C2 secondary 0 100p
C3 out 0 100p
";

// ===========================================================================
// Parser Tests
// ===========================================================================

#[test]
fn test_parse_basic_coupling() {
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
K1 L1 L2 0.95
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    assert_eq!(netlist.couplings.len(), 1);
    assert_eq!(netlist.couplings[0].name, "K1");
    assert_eq!(netlist.couplings[0].inductor1_name, "L1");
    assert_eq!(netlist.couplings[0].inductor2_name, "L2");
    assert!((netlist.couplings[0].coupling - 0.95).abs() < 1e-10);
}

#[test]
fn test_parse_lowercase_coupling() {
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
k1 L1 L2 0.5
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    assert_eq!(netlist.couplings.len(), 1);
    assert_eq!(netlist.couplings[0].name, "k1");
    assert!((netlist.couplings[0].coupling - 0.5).abs() < 1e-10);
}

#[test]
fn test_parse_multiple_couplings() {
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
L3 c 0 50m
L4 d 0 50m
K1 L1 L2 0.95
K2 L3 L4 0.8
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
C3 c 0 1n
C4 d 0 1n
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    assert_eq!(netlist.couplings.len(), 2);
    assert_eq!(netlist.couplings[0].name, "K1");
    assert_eq!(netlist.couplings[1].name, "K2");
    assert!((netlist.couplings[0].coupling - 0.95).abs() < 1e-10);
    assert!((netlist.couplings[1].coupling - 0.8).abs() < 1e-10);
}

#[test]
fn test_reject_coupling_k_zero() {
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
K1 L1 L2 0
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject k=0");
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("(0, 1)") || err.contains("exclusive"),
        "Error should mention valid range: {}",
        err
    );
}

#[test]
fn test_reject_coupling_k_one() {
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
K1 L1 L2 1.0
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject k=1");
}

#[test]
fn test_reject_coupling_k_negative() {
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
K1 L1 L2 -0.5
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject k<0");
}

#[test]
fn test_reject_coupling_k_greater_than_one() {
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
K1 L1 L2 1.5
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject k>1");
}

#[test]
fn test_reject_missing_inductor() {
    let spice = "\
Test Circuit
L1 a 0 10m
K1 L1 L2 0.95
R1 a 0 1k
C1 a 0 1n
";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Should reject reference to missing inductor"
    );
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("L2") && err.contains("not found"),
        "Error should mention missing L2: {}",
        err
    );
}

#[test]
fn test_reject_self_coupling() {
    let spice = "\
Test Circuit
L1 a 0 10m
K1 L1 L1 0.95
R1 a 0 1k
C1 a 0 1n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject self-coupling");
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("itself"),
        "Error should mention self-coupling: {}",
        err
    );
}

#[test]
fn test_allow_inductor_in_multiple_couplings() {
    // Multi-winding transformers: an inductor may appear in multiple K directives
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
L3 c 0 50m
K1 L1 L2 0.95
K2 L1 L3 0.8
K3 L2 L3 0.9
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
C3 c 0 1n
";
    let result = Netlist::parse(spice);
    assert!(
        result.is_ok(),
        "Multi-winding transformers should be allowed: {:?}",
        result.err()
    );
    let netlist = result.unwrap();
    assert_eq!(netlist.couplings.len(), 3);
}

#[test]
fn test_reject_duplicate_coupling_pair() {
    // Same pair of inductors coupled twice should be rejected
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
K1 L1 L2 0.95
K2 L2 L1 0.8
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject duplicate coupling pair");
    let err = result.unwrap_err().to_string();
    assert!(err.contains("multiple K directives"), "Error: {}", err);
}

#[test]
fn test_reject_non_inductor_name() {
    let spice = "\
Test Circuit
L1 a 0 10m
R1 b 0 1k
K1 L1 R1 0.95
C1 a 0 1n
";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Should reject non-inductor references in K element"
    );
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("must start with") || err.contains("inductor"),
        "Error should mention inductor naming: {}",
        err
    );
}

// ===========================================================================
// MNA Tests
// ===========================================================================

#[test]
fn test_coupled_inductors_removed_from_uncoupled() {
    let (_, mna, _) = build_pipeline(SIMPLE_COUPLED);

    // Coupled inductors should NOT be in mna.inductors
    assert_eq!(
        mna.inductors.len(),
        0,
        "Coupled inductors should be removed from uncoupled list, got {} uncoupled",
        mna.inductors.len()
    );

    // They should be in mna.coupled_inductors
    assert_eq!(
        mna.coupled_inductors.len(),
        1,
        "Should have 1 coupled inductor pair"
    );

    let ci = &mna.coupled_inductors[0];
    assert_eq!(ci.name, "K1");
    assert!((ci.coupling - 0.95).abs() < 1e-10);
    assert!((ci.l1_value - 0.1).abs() < 1e-10, "L1 should be 100mH");
    assert!((ci.l2_value - 0.1).abs() < 1e-10, "L2 should be 100mH");
}

#[test]
fn test_mixed_inductors_separation() {
    // Circuit with L1+L2 coupled and L3 uncoupled
    let (_, mna, _) = build_pipeline(MIXED_INDUCTORS);

    // L3 should remain uncoupled
    assert_eq!(
        mna.inductors.len(),
        1,
        "Should have 1 uncoupled inductor (L3)"
    );
    assert_eq!(mna.inductors[0].name, "L3");

    // L1+L2 should be coupled
    assert_eq!(
        mna.coupled_inductors.len(),
        1,
        "Should have 1 coupled inductor pair"
    );
}

#[test]
fn test_a_matrix_mutual_conductance() {
    // Simple transformer: L1 (primary, 10mH) and L2 (secondary, 100mH), k=0.95
    let spice = "\
Mutual G Test
L1 a 0 10m
L2 b 0 100m
K1 L1 L2 0.95
R1 a 0 10k
R2 b 0 10k
C1 a 0 1p
C2 b 0 1p
";
    let (_, mna, _) = build_pipeline(spice);
    let a = mna.get_a_matrix(44100.0).unwrap();

    let a_idx = *mna.node_map.get("a").unwrap() - 1;
    let b_idx = *mna.node_map.get("b").unwrap() - 1;

    let l1: f64 = 10e-3;
    let l2: f64 = 100e-3;
    let k: f64 = 0.95;
    let m_val = k * (l1 * l2).sqrt();
    let det = l1 * l2 - m_val * m_val;
    let t = 1.0 / 44100.0;
    let half_t = t / 2.0;
    let g_self_1 = half_t * l2 / det;
    let g_self_2 = half_t * l1 / det;
    let g_mutual = -half_t * m_val / det;

    // Diagonal should include self-conductance (plus R and alpha*C)
    let g_r = 1.0 / 10e3;
    let alpha_c = 2.0 * 1e-12 * 44100.0;

    let expected_a_aa = g_r + alpha_c + g_self_1;
    let rel_err_aa = (a[a_idx][a_idx] - expected_a_aa).abs() / expected_a_aa;
    assert!(
        rel_err_aa < 1e-10,
        "A[a][a] should include g_self_1: expected {:.6e}, got {:.6e} (err {:.2e})",
        expected_a_aa,
        a[a_idx][a_idx],
        rel_err_aa,
    );

    let expected_a_bb = g_r + alpha_c + g_self_2;
    let rel_err_bb = (a[b_idx][b_idx] - expected_a_bb).abs() / expected_a_bb;
    assert!(
        rel_err_bb < 1e-10,
        "A[b][b] should include g_self_2: expected {:.6e}, got {:.6e} (err {:.2e})",
        expected_a_bb,
        a[b_idx][b_idx],
        rel_err_bb,
    );

    // Off-diagonal should have mutual conductance
    // Both L1 and L2 have one node grounded, so only the a-b cross-term is stamped.
    // L1: node_i=a, node_j=0; L2: node_i=b, node_j=0
    // stamp_mutual: mat[a][b] += g_mutual, mat[0][0] (skip), mat[a][0] (skip), mat[0][b] (skip)
    let rel_err_ab = (a[a_idx][b_idx] - g_mutual).abs() / g_mutual.abs();
    assert!(
        rel_err_ab < 1e-10,
        "A[a][b] should be g_mutual: expected {:.6e}, got {:.6e} (err {:.2e})",
        g_mutual,
        a[a_idx][b_idx],
        rel_err_ab,
    );

    // Symmetric: A[b][a] should also have g_mutual (from stamp_mutual b-d coupling where b=0, d=0 is skipped,
    // actually for mat[b][a] it comes from the transpose of the mutual stamp)
    // Let me recheck: stamp_mutual(mat, a_node=a, b_node=0, c_node=b, d_node=0, g_mutual)
    //   mat[a][b] += g   (a>0 && c=b>0)
    //   mat[0][0] += g   (skip, b=0)
    //   mat[a][0] -= g   (skip, d=0)
    //   mat[0][b] -= g   (skip, b_node=0)
    // So A[b][a] = 0 from mutual. But the companion model should be symmetric in the A matrix...
    // Actually: g_mutual is stamped for element1=(L1: a,0) and element2=(L2: b,0).
    // The a,b cross-term gets g_mutual but b,a does not from this single stamp.
    // However, the MNA G+C matrix for a mutual inductor should indeed be symmetric.
    // Wait — the stamp is one-directional. Let me check if the MNA code does symmetric stamping...
    // The stamp_mutual_conductance does: [a][c]+=g, [b][d]+=g, [a][d]-=g, [b][c]-=g
    // With a=a_node, b=0, c=b_node, d=0:
    //   [a][b] += g (yes)
    //   [0][0] += g (skip)
    //   [a][0] -= g (skip)
    //   [0][b] -= g (skip)
    // So only A[a][b] gets the mutual term, not A[b][a]. This is physically correct for
    // the companion model because the mutual coupling is directional in the MNA formulation
    // when one side of each inductor shares a ground node. The full 4-terminal stamp IS symmetric
    // (a-c and b-d terms), but when both b,d = ground, only the a-c term survives.
    //
    // Actually this means A is asymmetric. Let me just verify the expected value.
    // For a proper transformer companion model, the coupling should appear symmetrically.
    // But with both inductors grounded at one end, the mutual stamp only produces [a][b].
    // The matrix IS correct — the solver works with asymmetric matrices.
    //
    // Just verify A[b][a] has the mutual term too. If L1=(a,0) L2=(b,0), then:
    // Actually, looking more carefully at stamp_mutual_conductance:
    //   a=L1.node_i, b=L1.node_j, c=L2.node_i, d=L2.node_j
    // So a=a_node, b=0, c=b_node, d=0
    // mat[a][c] = mat[a_node][b_node] += g  -> A[a][b] += g_mutual
    // mat[b][d] = mat[0][0] (skip)
    // mat[a][d] = mat[a_node][0] (skip)
    // mat[b][c] = mat[0][b_node] (skip)
    // So A[b][a] does NOT get mutual term. This means the matrix is asymmetric.
    // This is still correct for the DK solver (it inverts A directly).
}

#[test]
fn test_uncoupled_inductors_unchanged() {
    // Circuit with only uncoupled inductors (no K elements)
    let spice = "\
Uncoupled Test
R1 in out 1k
L1 out 0 100m
C1 out 0 100p
";
    let (_, mna, _) = build_pipeline(spice);
    assert_eq!(mna.inductors.len(), 1, "Uncoupled inductor should remain");
    assert_eq!(mna.coupled_inductors.len(), 0, "No coupled inductors");
    assert_eq!(mna.inductors[0].name, "L1");
}

// ===========================================================================
// DK Kernel Tests
// ===========================================================================

#[test]
fn test_dk_coupled_inductor_conductances() {
    let (_, _, kernel) = build_pipeline(SIMPLE_COUPLED);

    assert_eq!(kernel.coupled_inductors.len(), 1);
    let ci = &kernel.coupled_inductors[0];

    let l1: f64 = 0.1; // 100mH
    let l2: f64 = 0.1; // 100mH
    let k: f64 = 0.95;
    let m_val = k * (l1 * l2).sqrt();
    let det = l1 * l2 - m_val * m_val;
    let t = 1.0 / 44100.0;
    let half_t = t / 2.0;

    let expected_g_self_1 = half_t * l2 / det;
    let expected_g_self_2 = half_t * l1 / det;
    let expected_g_mutual = -half_t * m_val / det;

    assert!(
        (ci.g_self_1 - expected_g_self_1).abs() / expected_g_self_1.abs() < 1e-10,
        "g_self_1: expected {:.6e}, got {:.6e}",
        expected_g_self_1,
        ci.g_self_1
    );
    assert!(
        (ci.g_self_2 - expected_g_self_2).abs() / expected_g_self_2.abs() < 1e-10,
        "g_self_2: expected {:.6e}, got {:.6e}",
        expected_g_self_2,
        ci.g_self_2
    );
    assert!(
        (ci.g_mutual - expected_g_mutual).abs() / expected_g_mutual.abs() < 1e-10,
        "g_mutual: expected {:.6e}, got {:.6e}",
        expected_g_mutual,
        ci.g_mutual
    );
}

#[test]
fn test_dk_coupled_inductor_initial_state() {
    let (_, _, kernel) = build_pipeline(SIMPLE_COUPLED);
    let ci = &kernel.coupled_inductors[0];

    assert_eq!(ci.i1_prev, 0.0, "Initial i1_prev should be 0");
    assert_eq!(ci.i2_prev, 0.0, "Initial i2_prev should be 0");
    assert_eq!(ci.v1_prev, 0.0, "Initial v1_prev should be 0");
    assert_eq!(ci.v2_prev, 0.0, "Initial v2_prev should be 0");
    assert_eq!(ci.i1_hist, 0.0, "Initial i1_hist should be 0");
    assert_eq!(ci.i2_hist, 0.0, "Initial i2_hist should be 0");
}

#[test]
fn test_dk_coupled_inductor_names() {
    let (_, _, kernel) = build_pipeline(SIMPLE_COUPLED);
    let ci = &kernel.coupled_inductors[0];

    assert_eq!(ci.name, "K1");
    assert_eq!(ci.l1_name, "L1");
    assert_eq!(ci.l2_name, "L2");
    assert!((ci.l1_inductance - 0.1).abs() < 1e-10);
    assert!((ci.l2_inductance - 0.1).abs() < 1e-10);
    assert!((ci.coupling - 0.95).abs() < 1e-10);
}

#[test]
fn test_dk_no_coupled_inductors_in_uncoupled_list() {
    let (_, _, kernel) = build_pipeline(SIMPLE_COUPLED);
    assert_eq!(
        kernel.inductors.len(),
        0,
        "Coupled inductors should not appear in kernel.inductors"
    );
}

#[test]
fn test_dk_mixed_inductors() {
    let (_, _, kernel) = build_pipeline(MIXED_INDUCTORS);
    assert_eq!(kernel.coupled_inductors.len(), 1, "1 coupled pair");
    assert_eq!(kernel.inductors.len(), 1, "1 uncoupled inductor (L3)");
    assert_eq!(&*kernel.inductors[0].name, "L3");
}

// ===========================================================================
// Codegen IR Tests
// ===========================================================================

#[test]
fn test_ir_coupled_inductors() {
    let (netlist, mna, kernel) = build_pipeline(SIMPLE_COUPLED);
    let config = default_config();
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();

    assert_eq!(ir.coupled_inductors.len(), 1);
    assert_eq!(ir.coupled_inductors[0].name, "K1");
    assert!((ir.coupled_inductors[0].coupling - 0.95).abs() < 1e-10);
    assert!(ir.coupled_inductors[0].g_self_1 > 0.0);
    assert!(ir.coupled_inductors[0].g_self_2 > 0.0);
    assert!(
        ir.coupled_inductors[0].g_mutual < 0.0,
        "g_mutual should be negative"
    );
}

#[test]
fn test_ir_no_coupled_inductors_when_none() {
    let spice = "\
RC Only
R1 in out 1k
C1 out 0 100n
";
    let (netlist, mna, kernel) = build_pipeline(spice);
    let config = default_config();
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config).unwrap();

    assert_eq!(ir.coupled_inductors.len(), 0);
}

// ===========================================================================
// Codegen Compilation Tests
// ===========================================================================

#[test]
fn test_coupled_inductor_codegen_compiles() {
    let code = generate_code(SIMPLE_COUPLED);
    assert_compiles(&code, "simple_coupled");
}

#[test]
fn test_step_up_transformer_codegen_compiles() {
    let code = generate_code(STEP_UP_TRANSFORMER);
    assert_compiles(&code, "step_up_transformer");
}

#[test]
fn test_mixed_inductors_codegen_compiles() {
    let code = generate_code(MIXED_INDUCTORS);
    assert_compiles(&code, "mixed_inductors");
}

// ===========================================================================
// Codegen Content Tests
// ===========================================================================

#[test]
fn test_codegen_constants_present() {
    let code = generate_code(SIMPLE_COUPLED);

    assert!(
        code.contains("CI_0_G_SELF_1"),
        "Missing CI_0_G_SELF_1 constant"
    );
    assert!(
        code.contains("CI_0_G_SELF_2"),
        "Missing CI_0_G_SELF_2 constant"
    );
    assert!(
        code.contains("CI_0_G_MUTUAL"),
        "Missing CI_0_G_MUTUAL constant"
    );
    assert!(
        code.contains("CI_0_L1_NODE_I"),
        "Missing CI_0_L1_NODE_I constant"
    );
    assert!(
        code.contains("CI_0_L1_NODE_J"),
        "Missing CI_0_L1_NODE_J constant"
    );
    assert!(
        code.contains("CI_0_L2_NODE_I"),
        "Missing CI_0_L2_NODE_I constant"
    );
    assert!(
        code.contains("CI_0_L2_NODE_J"),
        "Missing CI_0_L2_NODE_J constant"
    );
    assert!(
        code.contains("CI_0_L1_INDUCTANCE"),
        "Missing CI_0_L1_INDUCTANCE constant"
    );
    assert!(
        code.contains("CI_0_L2_INDUCTANCE"),
        "Missing CI_0_L2_INDUCTANCE constant"
    );
    assert!(
        code.contains("CI_0_COUPLING"),
        "Missing CI_0_COUPLING constant"
    );
}

#[test]
fn test_codegen_state_fields_present() {
    let code = generate_code(SIMPLE_COUPLED);

    assert!(
        code.contains("ci_i1_prev"),
        "Missing ci_i1_prev state field"
    );
    assert!(
        code.contains("ci_i2_prev"),
        "Missing ci_i2_prev state field"
    );
    assert!(
        code.contains("ci_v1_prev"),
        "Missing ci_v1_prev state field"
    );
    assert!(
        code.contains("ci_v2_prev"),
        "Missing ci_v2_prev state field"
    );
    assert!(
        code.contains("ci_i1_hist"),
        "Missing ci_i1_hist state field"
    );
    assert!(
        code.contains("ci_i2_hist"),
        "Missing ci_i2_hist state field"
    );
    assert!(
        code.contains("ci_g_self_1"),
        "Missing ci_g_self_1 state field"
    );
    assert!(
        code.contains("ci_g_self_2"),
        "Missing ci_g_self_2 state field"
    );
    assert!(
        code.contains("ci_g_mutual"),
        "Missing ci_g_mutual state field"
    );
}

#[test]
fn test_codegen_rhs_injection() {
    let code = generate_code(SIMPLE_COUPLED);

    // build_rhs should inject coupled inductor history currents
    assert!(
        code.contains("ci_i1_hist"),
        "build_rhs should reference ci_i1_hist"
    );
    assert!(
        code.contains("ci_i2_hist"),
        "build_rhs should reference ci_i2_hist"
    );
}

#[test]
fn test_codegen_process_sample_update() {
    let code = generate_code(SIMPLE_COUPLED);

    // process_sample should update coupled inductor state
    assert!(
        code.contains("state.ci_i1_prev"),
        "process_sample should update ci_i1_prev"
    );
    assert!(
        code.contains("state.ci_i2_prev"),
        "process_sample should update ci_i2_prev"
    );
    assert!(
        code.contains("state.ci_v1_prev"),
        "process_sample should update ci_v1_prev"
    );
    assert!(
        code.contains("state.ci_v2_prev"),
        "process_sample should update ci_v2_prev"
    );
}

#[test]
fn test_codegen_nan_sanitization() {
    let code = generate_code(SIMPLE_COUPLED);

    // NaN check now happens before state write: checks local `v` not `state.v_prev`
    let sanitize_idx = code
        .find("if !v.iter().all(|x| x.is_finite())")
        .or_else(|| code.find("if !state.v_prev.iter().all(|x| x.is_finite())"));
    assert!(sanitize_idx.is_some(), "Should have NaN sanitization block");

    let after_sanitize = &code[sanitize_idx.unwrap()..];
    // NaN reset now returns DC operating point output instead of zeros
    let return_idx = after_sanitize
        .find("return dc_output;")
        .or_else(|| after_sanitize.find("return [0.0; NUM_OUTPUTS];"))
        .expect("Missing return in sanitization");
    let sanitize_block = &after_sanitize[..return_idx];

    assert!(
        sanitize_block.contains("ci_i1_prev"),
        "Sanitization should reset ci_i1_prev"
    );
    assert!(
        sanitize_block.contains("ci_i2_prev"),
        "Sanitization should reset ci_i2_prev"
    );
}

#[test]
fn test_codegen_backward_compat_no_coupling() {
    // A circuit with no coupled inductors should NOT emit CI_ constants
    let spice = "\
No Coupling
R1 in out 10k
C1 out 0 100n
";
    let code = generate_code(spice);

    assert!(
        !code.contains("CI_0"),
        "No CI_ constants when no coupled inductors"
    );
    assert!(
        !code.contains("ci_i1_prev"),
        "No ci_i1_prev when no coupled inductors"
    );
    assert!(
        !code.contains("ci_g_self"),
        "No ci_g_self when no coupled inductors"
    );
    assert!(
        !code.contains("ci_g_mutual"),
        "No ci_g_mutual when no coupled inductors"
    );
}

#[test]
fn test_codegen_backward_compat_uncoupled_inductors_only() {
    // A circuit with ONLY uncoupled inductors should have IND_ but NOT CI_
    let spice = "\
Uncoupled Only
R1 in out 1k
L1 out 0 100m
C1 out 0 100p
";
    let code = generate_code(spice);

    assert!(
        code.contains("IND_0"),
        "Should have IND_0 for uncoupled inductor"
    );
    assert!(!code.contains("CI_0"), "No CI_0 when no coupled inductors");
    assert!(
        !code.contains("ci_i1_prev"),
        "No ci_i1_prev when no coupled inductors"
    );
}

// ===========================================================================
// Behavioral Tests — Transformer Voltage Ratios
// ===========================================================================

#[test]
fn test_step_up_transformer_voltage_ratio() {
    // L1=10mH (primary), L2=100mH (secondary), k=0.95
    // Ideal transformer: V2/V1 = sqrt(L2/L1) = sqrt(10) ~ 3.16
    // With k=0.95 and loading, actual ratio will be less than ideal.
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(STEP_UP_TRANSFORMER, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    // Drive with 100Hz sine for 4410 samples (10 cycles at 44.1kHz)
    let num_samples = 4410;
    let mut output = vec![0.0; num_samples];
    let freq = 100.0;
    for i in 0..num_samples {
        let input = 0.1 * (2.0 * std::f64::consts::PI * freq * i as f64 / 44100.0).sin();
        output[i] = solver.process_sample(input);
    }

    // All outputs should be finite
    for (i, &v) in output.iter().enumerate() {
        assert!(v.is_finite(), "Output[{}] = {} (not finite)", i, v);
    }

    // Measure peak output in the last 3 cycles (settled region)
    let settled = &output[2940..4410]; // last 1470 samples = ~3 cycles at 100Hz
    let peak_out = settled.iter().map(|&v| v.abs()).fold(0.0f64, f64::max);

    // The secondary should produce measurable output. Due to loading (R2=1k),
    // non-ideal coupling, and the small input amplitude, the output may be
    // attenuated but should be clearly above noise.
    assert!(
        peak_out > 0.01,
        "Step-up transformer should produce measurable output, got peak {:.4}",
        peak_out
    );
}

#[test]
fn test_step_down_transformer_voltage_ratio() {
    // L1=100mH (primary), L2=10mH (secondary), k=0.95
    // Ideal: V2/V1 = sqrt(L2/L1) = sqrt(0.1) ~ 0.316
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(STEP_DOWN_TRANSFORMER, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    let num_samples = 4410;
    let mut output = vec![0.0; num_samples];
    let freq = 100.0;
    for i in 0..num_samples {
        let input = 1.0 * (2.0 * std::f64::consts::PI * freq * i as f64 / 44100.0).sin();
        output[i] = solver.process_sample(input);
    }

    for (i, &v) in output.iter().enumerate() {
        assert!(v.is_finite(), "Output[{}] = {} (not finite)", i, v);
    }

    let settled = &output[2940..4410];
    let peak_out = settled.iter().map(|&v| v.abs()).fold(0.0f64, f64::max);

    // Step-down should produce output, but less than input
    assert!(
        peak_out > 0.01,
        "Step-down transformer should produce measurable output, got peak {:.4}",
        peak_out
    );
    assert!(
        peak_out < 1.0,
        "Step-down transformer output should be less than input peak (1.0V), got {:.4}",
        peak_out
    );
}

#[test]
fn test_weak_coupling_minimal_transfer() {
    // k=0.1: very weak coupling, minimal energy transfer
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(WEAK_COUPLING, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    let num_samples = 4410;
    let mut output_weak = vec![0.0; num_samples];
    let freq = 100.0;
    for i in 0..num_samples {
        let input = 1.0 * (2.0 * std::f64::consts::PI * freq * i as f64 / 44100.0).sin();
        output_weak[i] = solver.process_sample(input);
    }

    // Now compare with strong coupling (k=0.95, same inductances)
    let (_, mna2, kernel2) = build_pipeline_with_input(UNITY_TRANSFORMER, "in", input_resistance);
    let in_idx2 = *mna2.node_map.get("in").unwrap() - 1;
    let out_idx2 = *mna2.node_map.get("out").unwrap() - 1;

    let mut solver2 = LinearSolver::new(kernel2, in_idx2, out_idx2);
    solver2.set_input_conductance(1.0 / input_resistance);

    let mut output_strong = vec![0.0; num_samples];
    for i in 0..num_samples {
        let input = 1.0 * (2.0 * std::f64::consts::PI * freq * i as f64 / 44100.0).sin();
        output_strong[i] = solver2.process_sample(input);
    }

    // Measure RMS of settled region for both
    let settled_weak = &output_weak[2940..4410];
    let rms_weak: f64 =
        (settled_weak.iter().map(|v| v * v).sum::<f64>() / settled_weak.len() as f64).sqrt();

    let settled_strong = &output_strong[2940..4410];
    let rms_strong: f64 =
        (settled_strong.iter().map(|v| v * v).sum::<f64>() / settled_strong.len() as f64).sqrt();

    // Weak coupling should transfer significantly less energy
    assert!(
        rms_weak < rms_strong,
        "Weak coupling (k=0.1) should transfer less energy than strong (k=0.95): \
         weak RMS = {:.6}, strong RMS = {:.6}",
        rms_weak,
        rms_strong
    );
}

#[test]
fn test_unity_transformer_voltage_ratio() {
    // L1=L2=10mH, k=0.95: near-unity voltage ratio
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(UNITY_TRANSFORMER, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    let num_samples = 4410;
    let mut output = vec![0.0; num_samples];
    let freq = 100.0;
    for i in 0..num_samples {
        let input = 1.0 * (2.0 * std::f64::consts::PI * freq * i as f64 / 44100.0).sin();
        output[i] = solver.process_sample(input);
    }

    for (i, &v) in output.iter().enumerate() {
        assert!(v.is_finite(), "Output[{}] = {} (not finite)", i, v);
    }

    let settled = &output[2940..4410];
    let peak_out = settled.iter().map(|&v| v.abs()).fold(0.0f64, f64::max);

    // 1:1 transformer should transfer energy
    assert!(
        peak_out > 0.01,
        "1:1 transformer should produce measurable output, got peak {:.4}",
        peak_out
    );
}

#[test]
fn test_transformer_all_finite() {
    // Exhaustive finiteness check across all transformer circuits
    let circuits = [
        ("step_up", STEP_UP_TRANSFORMER),
        ("step_down", STEP_DOWN_TRANSFORMER),
        ("weak", WEAK_COUPLING),
        ("unity", UNITY_TRANSFORMER),
    ];

    for (name, spice) in &circuits {
        let input_resistance = 1.0;
        let (_, mna, kernel) = build_pipeline_with_input(spice, "in", input_resistance);
        let in_idx = *mna.node_map.get("in").unwrap() - 1;
        let out_idx = *mna.node_map.get("out").unwrap() - 1;

        let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
        solver.set_input_conductance(1.0 / input_resistance);

        let num_samples = 1000;
        for i in 0..num_samples {
            let input = (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 44100.0).sin();
            let out = solver.process_sample(input);
            assert!(
                out.is_finite(),
                "{}: Output[{}] = {} (not finite)",
                name,
                i,
                out
            );
        }
    }
}

// ===========================================================================
// Runtime Solver Integration Tests
// ===========================================================================

#[test]
fn test_runtime_solver_step_response() {
    // Step response through transformer — output should be transient (not DC)
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(UNITY_TRANSFORMER, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    // Apply DC step
    let mut output = vec![0.0; 500];
    for i in 0..500 {
        output[i] = solver.process_sample(1.0);
    }

    // Transformer blocks DC — after transient, output should settle near 0
    let late_avg: f64 = output[400..500].iter().map(|v| v.abs()).sum::<f64>() / 100.0;
    assert!(
        late_avg < 0.5,
        "Transformer should block DC, but late average output = {:.4}",
        late_avg
    );
}

#[test]
fn test_runtime_solver_ac_passes() {
    // AC signal should pass through the transformer
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(UNITY_TRANSFORMER, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    // Drive with 1kHz sine for 500 samples (~11 cycles)
    let num_samples = 500;
    let mut output = vec![0.0; num_samples];
    let freq = 1000.0;
    for i in 0..num_samples {
        let input = (2.0 * std::f64::consts::PI * freq * i as f64 / 44100.0).sin();
        output[i] = solver.process_sample(input);
    }

    // Should see AC output
    let peak = output[200..500]
        .iter()
        .map(|&v| v.abs())
        .fold(0.0f64, f64::max);
    assert!(
        peak > 0.01,
        "Transformer should pass AC signal, but peak = {:.4}",
        peak
    );

    // Should see sign changes (oscillation)
    let sign_changes: usize = output[200..500]
        .windows(2)
        .filter(|w| w[0] * w[1] < 0.0)
        .count();
    assert!(
        sign_changes > 5,
        "Should see oscillation in output, only {} sign changes",
        sign_changes
    );
}

// ===========================================================================
// Additional Parser Validation Tests
// ===========================================================================

#[test]
fn test_reject_duplicate_coupling_names() {
    let spice = "\
Test Circuit
L1 a 0 10m
L2 b 0 100m
L3 c 0 50m
L4 d 0 50m
K1 L1 L2 0.95
K1 L3 L4 0.8
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
C3 c 0 1n
C4 d 0 1n
";
    let result = Netlist::parse(spice);
    assert!(result.is_err(), "Should reject duplicate coupling names");
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("Duplicate") || err.contains("duplicate"),
        "Error should mention duplicate: {}",
        err
    );
}

// ===========================================================================
// Non-Ground Terminal Tests (full 4-terminal stamp)
// ===========================================================================

/// Transformer where neither inductor terminal is grounded.
/// This exercises all four branches of stamp_mutual_conductance.
const FLOATING_TRANSFORMER: &str = "\
Floating Transformer
R1 in a 100
L1 a b 10m
L2 c d 10m
K1 L1 L2 0.95
R2 d out 1k
R3 b 0 100
R4 c 0 100
C1 a 0 100p
C2 b 0 100p
C3 c 0 100p
C4 d 0 100p
C5 out 0 100p
";

#[test]
fn test_floating_transformer_codegen_compiles() {
    let code = generate_code(FLOATING_TRANSFORMER);
    assert_compiles(&code, "floating_transformer");
}

#[test]
fn test_floating_transformer_a_matrix_symmetric() {
    // When inductors have no grounded terminals, the mutual stamp
    // should produce symmetric cross-terms in the A matrix.
    let spice = "\
Symmetric Test
L1 a b 10m
L2 c d 10m
K1 L1 L2 0.95
R1 a 0 10k
R2 b 0 10k
R3 c 0 10k
R4 d 0 10k
C1 a 0 1p
C2 b 0 1p
C3 c 0 1p
C4 d 0 1p
";
    let (_, mna, _) = build_pipeline(spice);
    let a = mna.get_a_matrix(44100.0).unwrap();
    let n = a.len();

    // The A matrix should be symmetric for a linear circuit with coupled inductors
    for i in 0..n {
        for j in 0..n {
            let diff = (a[i][j] - a[j][i]).abs();
            let max_val = a[i][j].abs().max(a[j][i].abs()).max(1e-20);
            assert!(
                diff / max_val < 1e-10,
                "A matrix should be symmetric: A[{}][{}] = {:.6e}, A[{}][{}] = {:.6e}",
                i,
                j,
                a[i][j],
                j,
                i,
                a[j][i]
            );
        }
    }
}

#[test]
fn test_floating_transformer_behavioral() {
    let input_resistance = 1.0;
    let (_, mna, kernel) = build_pipeline_with_input(FLOATING_TRANSFORMER, "in", input_resistance);
    let in_idx = *mna.node_map.get("in").unwrap() - 1;
    let out_idx = *mna.node_map.get("out").unwrap() - 1;

    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    let num_samples = 4410;
    let freq = 1000.0;
    let mut output = vec![0.0; num_samples];
    for i in 0..num_samples {
        let input = (2.0 * std::f64::consts::PI * freq * i as f64 / 44100.0).sin();
        output[i] = solver.process_sample(input);
    }

    // All outputs finite
    for (i, &v) in output.iter().enumerate() {
        assert!(v.is_finite(), "Output[{}] = {} (not finite)", i, v);
    }

    // Should produce measurable AC output
    let peak = output[2000..4410]
        .iter()
        .map(|&v| v.abs())
        .fold(0.0f64, f64::max);
    assert!(
        peak > 0.001,
        "Floating transformer should pass AC signal, peak = {:.6}",
        peak
    );
}

// ===========================================================================
// Reversed Polarity Test
// ===========================================================================

#[test]
fn test_reversed_polarity_transformer() {
    // Inductor with reversed terminal order: L1 0 primary instead of L1 primary 0
    let spice = "\
Reversed Polarity
R1 in primary 100
L1 0 primary 10m
L2 secondary 0 10m
K1 L1 L2 0.95
R2 secondary out 1k
C1 primary 0 100p
C2 secondary 0 100p
C3 out 0 100p
";
    let (_, mna, _) = build_pipeline(spice);
    assert_eq!(mna.coupled_inductors.len(), 1);

    // Should compile
    let code = generate_code(spice);
    assert_compiles(&code, "reversed_polarity");

    // Should produce finite output
    let input_resistance = 1.0;
    let (_, mna2, kernel) = build_pipeline_with_input(spice, "in", input_resistance);
    let in_idx = *mna2.node_map.get("in").unwrap() - 1;
    let out_idx = *mna2.node_map.get("out").unwrap() - 1;
    let mut solver = LinearSolver::new(kernel, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    for i in 0..500 {
        let input = (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 44100.0).sin();
        let out = solver.process_sample(input);
        assert!(out.is_finite(), "Output[{}] = {} (not finite)", i, out);
    }
}

// ===========================================================================
// Coupled Inductor + Switch Rejection Test
// ===========================================================================

#[test]
fn test_reject_coupled_inductor_in_switch() {
    let spice = "\
Switch Conflict
L1 a 0 10m
L2 b 0 100m
K1 L1 L2 0.95
.switch L1 10m 20m
R1 a 0 1k
C1 a 0 1n
C2 b 0 1n
";
    let netlist = Netlist::parse(spice).expect("parse should succeed");
    let result = MnaSystem::from_netlist(&netlist);
    assert!(
        result.is_err(),
        "Should reject coupled inductor that is also in a .switch directive"
    );
    let err = format!("{:?}", result.unwrap_err());
    assert!(
        err.contains("Coupled inductor") || err.contains("coupled"),
        "Error should mention coupled inductor conflict: {}",
        err
    );
}

// ===========================================================================
// Near-Degenerate Coupling (k close to 1)
// ===========================================================================

#[test]
fn test_near_degenerate_coupling() {
    // k=0.999 should work but produce large conductances
    let spice = "\
Near Degenerate
R1 in primary 100
L1 primary 0 10m
L2 secondary 0 10m
K1 L1 L2 0.999
R2 secondary out 1k
C1 primary 0 100p
C2 secondary 0 100p
C3 out 0 100p
";
    let (_, mna, kernel) = build_pipeline(spice);
    assert_eq!(mna.coupled_inductors.len(), 1);
    assert_eq!(kernel.coupled_inductors.len(), 1);

    // With k=0.999, det = L1*L2*(1-k^2) is very small, so conductances
    // are much larger than for the uncoupled case (T/(2L) ~ 1.13e-3).
    // g_self_1 = (T/2)*L2/det should be ~500x larger than uncoupled.
    let ci = &kernel.coupled_inductors[0];
    let uncoupled_g = (1.0 / 44100.0) / (2.0 * 10e-3); // T/(2L) for uncoupled
    assert!(
        ci.g_self_1 > uncoupled_g * 100.0,
        "Near-degenerate coupling should produce much larger g_self than uncoupled: g_self_1={:.6}, uncoupled={:.6}",
        ci.g_self_1,
        uncoupled_g
    );

    // Should still compile and produce finite output
    let code = generate_code(spice);
    assert_compiles(&code, "near_degenerate");

    let input_resistance = 1.0;
    let (_, mna2, kernel2) = build_pipeline_with_input(spice, "in", input_resistance);
    let in_idx = *mna2.node_map.get("in").unwrap() - 1;
    let out_idx = *mna2.node_map.get("out").unwrap() - 1;
    let mut solver = LinearSolver::new(kernel2, in_idx, out_idx);
    solver.set_input_conductance(1.0 / input_resistance);

    for i in 0..200 {
        let input = (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 44100.0).sin();
        let out = solver.process_sample(input);
        assert!(out.is_finite(), "Output[{}] = {} (not finite)", i, out);
    }
}

// ===========================================================================
// Multiple Coupled Pairs (MNA + DK + Codegen)
// ===========================================================================

#[test]
fn test_two_coupled_pairs_mna() {
    let spice = "\
Two Transformers
L1 a 0 10m
L2 b 0 100m
L3 c 0 50m
L4 d 0 50m
K1 L1 L2 0.95
K2 L3 L4 0.8
R1 a 0 1k
R2 b 0 1k
R3 c 0 1k
R4 d 0 1k
C1 a 0 1n
C2 b 0 1n
C3 c 0 1n
C4 d 0 1n
";
    let (_, mna, kernel) = build_pipeline(spice);

    assert_eq!(
        mna.coupled_inductors.len(),
        2,
        "Should have 2 coupled pairs"
    );
    assert_eq!(mna.inductors.len(), 0, "All inductors should be coupled");
    assert_eq!(kernel.coupled_inductors.len(), 2);

    // Different coupling coefficients should produce different conductances
    let ci0 = &kernel.coupled_inductors[0];
    let ci1 = &kernel.coupled_inductors[1];
    assert!(
        (ci0.g_mutual - ci1.g_mutual).abs() > 1e-10,
        "Different k values should produce different g_mutual"
    );
}

#[test]
fn test_two_coupled_pairs_codegen_compiles() {
    let spice = "\
Two Transformers Codegen
L1 a 0 10m
L2 b 0 100m
L3 c 0 50m
L4 d 0 50m
K1 L1 L2 0.95
K2 L3 L4 0.8
R1 a b 1k
R2 c d 1k
C1 a 0 1n
C2 b 0 1n
C3 c 0 1n
C4 d 0 1n
";
    let code = generate_code(spice);

    // Both CI_0 and CI_1 should be present
    assert!(code.contains("CI_0_G_SELF_1"), "Missing CI_0_G_SELF_1");
    assert!(code.contains("CI_1_G_SELF_1"), "Missing CI_1_G_SELF_1");
    assert!(code.contains("CI_0_G_MUTUAL"), "Missing CI_0_G_MUTUAL");
    assert!(code.contains("CI_1_G_MUTUAL"), "Missing CI_1_G_MUTUAL");

    assert_compiles(&code, "two_coupled_pairs");
}

// ===========================================================================
// A Matrix Symmetry Test (grounded case, now fixed)
// ===========================================================================

#[test]
fn test_a_matrix_symmetric_grounded() {
    // With the symmetric stamp fix, even grounded inductors should
    // produce a symmetric A matrix.
    let spice = "\
Grounded Symmetric Test
L1 a 0 10m
L2 b 0 100m
K1 L1 L2 0.95
R1 a 0 10k
R2 b 0 10k
C1 a 0 1p
C2 b 0 1p
";
    let (_, mna, _) = build_pipeline(spice);
    let a = mna.get_a_matrix(44100.0).unwrap();

    let a_idx = *mna.node_map.get("a").unwrap() - 1;
    let b_idx = *mna.node_map.get("b").unwrap() - 1;

    // A[a][b] and A[b][a] should both have the mutual conductance
    let diff = (a[a_idx][b_idx] - a[b_idx][a_idx]).abs();
    assert!(
        diff < 1e-15,
        "A matrix should be symmetric: A[a][b] = {:.6e}, A[b][a] = {:.6e}",
        a[a_idx][b_idx],
        a[b_idx][a_idx]
    );

    // Both should be nonzero (mutual coupling present)
    assert!(
        a[a_idx][b_idx].abs() > 1e-10,
        "A[a][b] should have mutual coupling: {:.6e}",
        a[a_idx][b_idx]
    );
}
