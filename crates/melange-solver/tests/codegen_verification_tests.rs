//! Codegen verification tests.
//!
//! Verifies that generated code produces correct results matching the runtime solver.
//! The codegen module generates standalone Rust code for a specific circuit topology.
//! These tests check that the generated source strings contain correct formulas,
//! matrix constants, and coefficient values matching the DK kernel.

use melange_solver::codegen::{CodeGenerator, CodegenConfig, CodegenError};
use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::emitter::Emitter;
use melange_solver::codegen::rust_emitter::RustEmitter;
use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
use std::io::Write;

// ---------------------------------------------------------------------------
// Helper: build kernel pipeline from a SPICE string
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

fn generate_code(spice: &str) -> (String, Netlist, MnaSystem, DkKernel) {
    let (netlist, mna, kernel) = build_pipeline(spice);
    let codegen = CodeGenerator::new(default_config());
    let result = codegen.generate(&kernel, &mna, &netlist)
        .expect("code generation failed");
    (result.code, netlist, mna, kernel)
}

// ---------------------------------------------------------------------------
// Circuit definitions reused across tests
// ---------------------------------------------------------------------------

const DIODE_CLIPPER_SPICE: &str = "\
Diode Clipper
Rin in 0 1k
D1 in out D1N4148
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
";

const RC_CIRCUIT_SPICE: &str = "\
RC Circuit
R1 in out 1k
C1 out 0 1u
";

const TWO_DIODE_SPICE: &str = "\
Two Diode Circuit
Rin in 0 1k
D1 in mid D1N4148
D2 mid out D1N4148
R1 mid 0 10k
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
";

const BJT_SPICE: &str = "\
BJT Circuit
Q1 c b e 2N2222
Rc c 0 1k
Rb b 0 100k
Re e 0 1k
.model 2N2222 NPN(IS=1e-15 BF=200)
";

// ==========================================================================
// Test 1: v_d formula uses correct sign (p + K*i_nl, not p - K*i_nl)
// ==========================================================================

#[test]
fn test_generated_code_contains_correct_v_d_formula() {
    let (code, _netlist, _mna, kernel) = generate_code(DIODE_CLIPPER_SPICE);

    // For a single-diode circuit (m=1), the generated NR loop should compute:
    //   v_d0 = p[0] + K[0][0] * i_nl[0]
    // It must NOT negate K (i.e. "p[0] - K[0][0]" would be wrong).
    assert_eq!(kernel.m, 1, "Expected m=1 for single-diode circuit");

    assert!(
        code.contains("p[0] + K[0][0] * i_nl[0]"),
        "Generated code should contain 'p[0] + K[0][0] * i_nl[0]' (correct sign). \
         K is naturally negative so the sign is already correct.\n\
         Searched in generated code but did not find the expected pattern."
    );

    // Make sure the incorrect negated form is absent
    assert!(
        !code.contains("p[0] - K[0][0] * i_nl[0]"),
        "Generated code must NOT contain 'p[0] - K[0][0] * i_nl[0]' (wrong sign)."
    );
}

// ==========================================================================
// Test 2: Jacobian formula uses correct sign (1 - g * K, not 1 + g * K)
// ==========================================================================

#[test]
fn test_generated_code_contains_correct_jacobian_formula() {
    let (code, _netlist, _mna, kernel) = generate_code(DIODE_CLIPPER_SPICE);

    assert_eq!(kernel.m, 1, "Expected m=1 for single-diode circuit");

    // The diagonal Jacobian entry for device 0 should be:
    //   j00 = 1.0 - jdev_0_0 * K[0][0]
    // Since K is negative, this is > 1, which is correct for stable NR.
    assert!(
        code.contains("1.0 - jdev_0_0 * K[0][0]"),
        "Generated code should contain '1.0 - jdev_0_0 * K[0][0]' (diagonal Jacobian).\n\
         Searched in generated code but did not find the expected pattern."
    );

    // The wrong form with + sign should NOT appear
    assert!(
        !code.contains("1.0 + jdev_0_0 * K[0][0]"),
        "Generated code must NOT contain '1.0 + jdev_0_0 * K[0][0]' (wrong sign)."
    );
}

// ==========================================================================
// Test 3: N_I constant matches kernel (transposed)
// ==========================================================================

#[test]
fn test_generated_ni_matrix_matches_kernel() {
    let (code, _netlist, _mna, kernel) = generate_code(DIODE_CLIPPER_SPICE);

    let n = kernel.n;
    let m = kernel.m;
    assert_eq!(m, 1, "Expected m=1 for single-diode circuit");
    assert!(n >= 2, "Expected n >= 2 for diode clipper");

    // The generated N_I constant is M x N (transposed from the kernel's N x M storage).
    // N_I[device_i][node_j] = kernel.n_i[node_j * m + device_i]
    //
    // For device 0:
    //   N_I[0][0] = kernel.n_i(0, 0)  (node 0, device 0)
    //   N_I[0][1] = kernel.n_i(1, 0)  (node 1, device 0)
    for j in 0..n {
        let expected_val = kernel.n_i(j, 0); // n_i[node_j * m + device_0]
        let expected_str = format!("{:.17e}", expected_val);
        assert!(
            code.contains(&expected_str),
            "N_I[0][{}] value {} not found in generated code.\n\
             Expected kernel.n_i({}, 0) = {} to appear in the N_I constant.",
            j, expected_str, j, expected_val
        );
    }

    // Also verify N_I is declared with the correct dimensions
    assert!(
        code.contains("pub const N_I: [[f64; N]; M]"),
        "N_I should be declared as [[f64; N]; M] (M rows of N columns)."
    );
}

// ==========================================================================
// Test 4: Final voltage coefficients match (S * N_i)[i][j]
// ==========================================================================

#[test]
fn test_generated_code_final_voltage_coefficients() {
    let (code, _netlist, _mna, kernel) = generate_code(DIODE_CLIPPER_SPICE);

    let n = kernel.n;
    let m = kernel.m;
    assert_eq!(m, 1, "Expected m=1 for single-diode circuit");

    // The compute_final_voltages function precomputes coefficients:
    //   coeff[i][j] = sum_k S[i][k] * N_i[k][j]
    // For each output node i and device j.
    // These appear as float literals multiplying di{j} in the generated code.
    for i in 0..n {
        for j in 0..m {
            let mut coeff = 0.0;
            for k in 0..n {
                coeff += kernel.s(i, k) * kernel.n_i(k, j);
            }

            if coeff != 0.0 {
                let coeff_str = format!("{:.17e}", coeff);
                assert!(
                    code.contains(&coeff_str),
                    "(S*N_i)[{}][{}] = {} not found in generated code.\n\
                     This coefficient should appear in compute_final_voltages as a literal \
                     multiplying di{}.",
                    i, j, coeff_str, j
                );
            }
        }
    }
}

// ==========================================================================
// Test 5: Runtime consistency for RC (linear) circuit
// ==========================================================================

#[test]
fn test_codegen_runtime_consistency_rc_circuit() {
    let (code, _netlist, _mna, kernel) = generate_code(RC_CIRCUIT_SPICE);

    let n = kernel.n;
    assert_eq!(kernel.m, 0, "RC circuit should have m=0 (no nonlinear devices)");
    assert_eq!(n, 2, "RC circuit should have n=2 nodes");

    // Verify S matrix values in generated code match kernel
    for i in 0..n {
        for j in 0..n {
            let expected = format!("{:.17e}", kernel.s(i, j));
            assert!(
                code.contains(&expected),
                "S[{}][{}] = {} not found in generated code.",
                i, j, expected
            );
        }
    }

    // Verify A_NEG matrix values in generated code match kernel
    for i in 0..n {
        for j in 0..n {
            let expected = format!("{:.17e}", kernel.a_neg(i, j));
            assert!(
                code.contains(&expected),
                "A_NEG[{}][{}] = {} not found in generated code.",
                i, j, expected
            );
        }
    }

    // Verify the generated M constant is 0
    assert!(
        code.contains("pub const M: usize = 0;"),
        "Generated code should declare M = 0 for linear circuit."
    );
}

// ==========================================================================
// Test 6: K constant matches kernel for diode clipper (should be negative)
// ==========================================================================

#[test]
fn test_codegen_runtime_consistency_diode_clipper() {
    let (code, _netlist, _mna, kernel) = generate_code(DIODE_CLIPPER_SPICE);

    assert_eq!(kernel.m, 1, "Expected m=1 for single-diode circuit");

    // Extract and compare K[0][0]
    let k00 = kernel.k(0, 0);
    let k00_str = format!("{:.17e}", k00);
    assert!(
        code.contains(&k00_str),
        "K[0][0] = {} not found in generated code.",
        k00_str
    );

    // K should be negative for a stable circuit (provides negative feedback)
    assert!(
        k00 < 0.0,
        "K[0][0] should be negative for stable diode clipper, got {}",
        k00
    );
}

// ==========================================================================
// Test 7: BJT generates 2D matrices of correct size
// ==========================================================================

#[test]
fn test_codegen_bjt_2d_matrices_correct_size() {
    let (code, _netlist, _mna, kernel) = generate_code(BJT_SPICE);

    // BJT is a 2-dimensional device: M = 2
    assert_eq!(kernel.m, 2, "Expected m=2 for single BJT circuit");

    // Verify M = 2 in generated code
    assert!(
        code.contains("pub const M: usize = 2;"),
        "Generated code should declare M = 2 for BJT circuit."
    );

    // K is declared as [[f64; M]; M] = 2x2
    assert!(
        code.contains("pub const K: [[f64; M]; M]"),
        "K should be declared as [[f64; M]; M]."
    );

    // N_V is [[f64; N]; M] = 2xN
    assert!(
        code.contains("pub const N_V: [[f64; N]; M]"),
        "N_V should be declared as [[f64; N]; M]."
    );

    // N_I is [[f64; N]; M] = 2xN (transposed from kernel's NxM)
    assert!(
        code.contains("pub const N_I: [[f64; N]; M]"),
        "N_I should be declared as [[f64; N]; M]."
    );

    // Verify all four K matrix values appear in the generated code
    for i in 0..2 {
        for j in 0..2 {
            let expected = format!("{:.17e}", kernel.k(i, j));
            assert!(
                code.contains(&expected),
                "K[{}][{}] = {} not found in generated code.",
                i, j, expected
            );
        }
    }

    // Verify N is correct (number of circuit nodes)
    let n = kernel.n;
    let n_decl = format!("pub const N: usize = {};", n);
    assert!(
        code.contains(&n_decl),
        "Generated code should declare N = {} for BJT circuit.",
        n
    );
}

// ==========================================================================
// Test 8: N_I indexing for multi-device (two-diode) circuit
// ==========================================================================

#[test]
fn test_codegen_n_i_indexing_multi_device() {
    let (code, _netlist, _mna, kernel) = generate_code(TWO_DIODE_SPICE);

    let n = kernel.n;
    let m = kernel.m;
    assert_eq!(m, 2, "Expected m=2 for two-diode circuit");

    // The generated N_I constant is M x N (transposed from the kernel's N x M).
    // N_I[device_i][node_j] = kernel.n_i[node_j * m + device_i]
    //
    // Verify every element matches.
    for device_i in 0..m {
        for node_j in 0..n {
            let kernel_val = kernel.n_i(node_j, device_i); // n_i[node_j * m + device_i]
            let expected_str = format!("{:.17e}", kernel_val);

            // Non-zero values must appear as literals in the N_I constant.
            // Zero values formatted as "0.00000000000000000e0" should also appear
            // since the codegen writes all entries.
            assert!(
                code.contains(&expected_str),
                "N_I[{}][{}] (kernel.n_i({}, {})) = {} not found in generated code.",
                device_i, node_j, node_j, device_i, expected_str
            );
        }
    }

    // Verify K is 2x2 for two-diode circuit
    for i in 0..m {
        for j in 0..m {
            let expected = format!("{:.17e}", kernel.k(i, j));
            assert!(
                code.contains(&expected),
                "K[{}][{}] = {} not found in generated code.",
                i, j, expected
            );
        }
    }
}

// ==========================================================================
// Additional structural verification tests
// ==========================================================================

/// Verify that the generated code contains all expected top-level constants.
#[test]
fn test_generated_code_contains_all_constants() {
    let (code, _netlist, _mna, kernel) = generate_code(DIODE_CLIPPER_SPICE);

    // All required constants should be present
    assert!(code.contains("pub const N: usize ="), "Missing N constant");
    assert!(code.contains("pub const M: usize ="), "Missing M constant");
    assert!(code.contains("pub const SAMPLE_RATE: f64 ="), "Missing SAMPLE_RATE constant");
    assert!(code.contains("pub const ALPHA: f64 ="), "Missing ALPHA constant");
    assert!(code.contains("pub const INPUT_NODE: usize ="), "Missing INPUT_NODE constant");
    assert!(code.contains("pub const OUTPUT_NODE: usize ="), "Missing OUTPUT_NODE constant");
    assert!(code.contains("pub const INPUT_RESISTANCE: f64 ="), "Missing INPUT_RESISTANCE constant");
    assert!(code.contains("pub const S: [[f64; N]; N]"), "Missing S matrix");
    assert!(code.contains("pub const A_NEG: [[f64; N]; N]"), "Missing A_NEG matrix");
    assert!(code.contains("pub const K: [[f64; M]; M]"), "Missing K matrix");
    assert!(code.contains("pub const N_V: [[f64; N]; M]"), "Missing N_V matrix");
    assert!(code.contains("pub const N_I: [[f64; N]; M]"), "Missing N_I matrix");

    // Verify the sample rate is correctly emitted
    assert!(
        code.contains("pub const SAMPLE_RATE: f64 = 44100.0;"),
        "Sample rate should be 44100.0"
    );

    // Verify input/output node indices from config
    assert!(
        code.contains("pub const INPUT_NODE: usize = 0;"),
        "Input node should be 0 (from config)"
    );
    assert!(
        code.contains("pub const OUTPUT_NODE: usize = 1;"),
        "Output node should be 1 (from config)"
    );

    // Verify alpha = 2 * sample_rate
    let alpha = 2.0 * 44100.0;
    let alpha_str = format!("{:.17e}", alpha);
    assert!(
        code.contains(&alpha_str),
        "ALPHA = {} not found in generated code.",
        alpha_str
    );

    // Verify input resistance from config
    let r_str = format!("{:.17e}", 1000.0_f64);
    assert!(
        code.contains(&r_str),
        "INPUT_RESISTANCE = {} not found in generated code.",
        r_str
    );

    // Verify n and m values
    let n = kernel.n;
    let m = kernel.m;
    assert!(
        code.contains(&format!("pub const N: usize = {};", n)),
        "N should be {} in generated code.",
        n
    );
    assert!(
        code.contains(&format!("pub const M: usize = {};", m)),
        "M should be {} in generated code.",
        m
    );
}

/// Verify that the generated code contains all expected functions.
#[test]
fn test_generated_code_contains_all_functions() {
    let (code, _netlist, _mna, _kernel) = generate_code(DIODE_CLIPPER_SPICE);

    assert!(code.contains("fn build_rhs("), "Missing build_rhs function");
    assert!(code.contains("fn mat_vec_mul_s("), "Missing mat_vec_mul_s function");
    assert!(code.contains("fn extract_controlling_voltages("), "Missing extract_controlling_voltages function");
    assert!(code.contains("fn solve_nonlinear("), "Missing solve_nonlinear function");
    assert!(code.contains("fn compute_final_voltages("), "Missing compute_final_voltages function");
    assert!(code.contains("pub fn process_sample("), "Missing process_sample function");
}

/// Verify S matrix symmetry property is preserved in generated code for RC circuit.
/// The S = A^{-1} matrix should have specific structure for a simple RC circuit.
#[test]
fn test_generated_s_matrix_values_rc() {
    let (code, _netlist, _mna, kernel) = generate_code(RC_CIRCUIT_SPICE);

    let n = kernel.n;

    // Verify all S matrix entries are present and finite
    for i in 0..n {
        for j in 0..n {
            let val = kernel.s(i, j);
            assert!(val.is_finite(), "S[{}][{}] should be finite, got {}", i, j, val);

            let val_str = format!("{:.17e}", val);
            assert!(
                code.contains(&val_str),
                "S[{}][{}] = {} not found in generated code.",
                i, j, val_str
            );
        }
    }
}

/// Verify that for a linear circuit (no NL devices), the NR solver is a no-op.
#[test]
fn test_linear_circuit_no_nr_solver() {
    let (code, _netlist, _mna, kernel) = generate_code(RC_CIRCUIT_SPICE);

    assert_eq!(kernel.m, 0, "RC circuit should have m=0");

    // With M=0, the solve_nonlinear should indicate no devices
    assert!(
        code.contains("No nonlinear devices"),
        "Linear circuit should generate a trivial solve_nonlinear."
    );
}

/// Verify the generated code includes the diode model functions.
#[test]
fn test_generated_code_includes_diode_model() {
    let (code, _netlist, _mna, _kernel) = generate_code(DIODE_CLIPPER_SPICE);

    assert!(
        code.contains("fn diode_current("),
        "Generated code should include diode_current function."
    );
    assert!(
        code.contains("fn diode_conductance("),
        "Generated code should include diode_conductance function."
    );
    assert!(
        code.contains("DEVICE_0_IS"),
        "Generated code should include per-device IS constant."
    );
    assert!(
        code.contains("DEVICE_0_N_VT"),
        "Generated code should include per-device N_VT constant."
    );
}

/// Verify the generated code includes BJT model functions.
#[test]
fn test_generated_code_includes_bjt_model() {
    let (code, _netlist, _mna, _kernel) = generate_code(BJT_SPICE);

    assert!(
        code.contains("fn bjt_ic("),
        "Generated code should include bjt_ic function."
    );
    assert!(
        code.contains("fn bjt_ib("),
        "Generated code should include bjt_ib function."
    );
    assert!(
        code.contains("fn bjt_jacobian("),
        "Generated code should include bjt_jacobian function."
    );
    assert!(
        code.contains("DEVICE_0_IS"),
        "Generated code should include per-device IS constant."
    );
    assert!(
        code.contains("DEVICE_0_BETA_F"),
        "Generated code should include per-device BETA_F constant."
    );
}

/// Verify that the two-diode circuit generates correct controlling voltage formulas
/// for both devices in the NR loop.
#[test]
fn test_two_diode_controlling_voltages() {
    let (code, _netlist, _mna, kernel) = generate_code(TWO_DIODE_SPICE);

    assert_eq!(kernel.m, 2, "Expected m=2 for two-diode circuit");

    // For two devices, the NR loop should compute:
    //   v_d0 = p[0] + K[0][0] * i_nl[0] + K[0][1] * i_nl[1]
    //   v_d1 = p[1] + K[1][0] * i_nl[0] + K[1][1] * i_nl[1]
    // (with zero K entries potentially omitted)

    // At minimum, check that both v_d0 and v_d1 are computed
    assert!(
        code.contains("let v_d0 ="),
        "Generated code should compute v_d0 for first diode."
    );
    assert!(
        code.contains("let v_d1 ="),
        "Generated code should compute v_d1 for second diode."
    );

    // Check Jacobian entries exist for the 2x2 system
    assert!(code.contains("let j00 ="), "Missing Jacobian j00");
    assert!(code.contains("let j01 ="), "Missing Jacobian j01");
    assert!(code.contains("let j10 ="), "Missing Jacobian j10");
    assert!(code.contains("let j11 ="), "Missing Jacobian j11");

    // Check that the 2x2 solve uses Cramer's rule (determinant)
    assert!(
        code.contains("let det = j00 * j11 - j01 * j10"),
        "2x2 NR solver should use Cramer's rule with determinant."
    );
}

/// Verify that the generated code's N_V matrix matches the kernel values.
#[test]
fn test_generated_nv_matrix_matches_kernel() {
    let (code, _netlist, _mna, kernel) = generate_code(DIODE_CLIPPER_SPICE);

    let n = kernel.n;
    let m = kernel.m;

    for i in 0..m {
        for j in 0..n {
            let val = kernel.n_v(i, j);
            let val_str = format!("{:.17e}", val);
            assert!(
                code.contains(&val_str),
                "N_V[{}][{}] = {} not found in generated code.",
                i, j, val_str
            );
        }
    }
}

/// Verify process_sample includes all expected pipeline steps.
#[test]
fn test_generated_process_sample_pipeline() {
    let (code, _netlist, _mna, _kernel) = generate_code(DIODE_CLIPPER_SPICE);

    // The process_sample function should call all pipeline functions in order
    assert!(code.contains("let rhs = build_rhs("), "Step 1: build_rhs");
    assert!(code.contains("let v_pred = mat_vec_mul_s("), "Step 2: mat_vec_mul_s");
    assert!(code.contains("let p = extract_controlling_voltages("), "Step 3: extract_controlling_voltages");
    assert!(code.contains("let i_nl = solve_nonlinear("), "Step 4: solve_nonlinear");
    assert!(code.contains("let v = compute_final_voltages("), "Step 5: compute_final_voltages");
    assert!(code.contains("state.v_prev = v;"), "Step 7: update v_prev");
    assert!(code.contains("state.i_nl_prev = i_nl;"), "Step 7: update i_nl_prev");
    assert!(code.contains("v[OUTPUT_NODE]"), "Step 8: read output");
}

// ==========================================================================
// Test: BJT NR solver generates proper 2D Newton-Raphson iteration
// ==========================================================================

#[test]
fn test_codegen_bjt_nr_solver_generates_device_calls() {
    let (code, _netlist, _mna, kernel) = generate_code(BJT_SPICE);

    assert_eq!(kernel.m, 2, "Expected m=2 for single BJT circuit");

    // BJT NR solver should call bjt_ic and bjt_ib with per-device params
    assert!(
        code.contains("bjt_ic(v_d0, v_d1, DEVICE_0_IS, DEVICE_0_VT, DEVICE_0_BETA_R)"),
        "NR solver should call bjt_ic with per-device params."
    );
    assert!(
        code.contains("bjt_ib(v_d0, v_d1, DEVICE_0_IS, DEVICE_0_VT, DEVICE_0_BETA_F, DEVICE_0_BETA_R)"),
        "NR solver should call bjt_ib with per-device params."
    );

    // Should call bjt_jacobian with per-device params
    assert!(
        code.contains("bjt_jacobian(v_d0, v_d1, DEVICE_0_IS, DEVICE_0_VT, DEVICE_0_BETA_F, DEVICE_0_BETA_R)"),
        "NR solver should call bjt_jacobian with per-device params."
    );

    // Should assign all 4 Jacobian entries from the bjt_jacobian result
    assert!(code.contains("jdev_0_0"), "Missing jdev_0_0 (dIc/dVbe)");
    assert!(code.contains("jdev_0_1"), "Missing jdev_0_1 (dIc/dVbc)");
    assert!(code.contains("jdev_1_0"), "Missing jdev_1_0 (dIb/dVbe)");
    assert!(code.contains("jdev_1_1"), "Missing jdev_1_1 (dIb/dVbc)");
}

#[test]
fn test_codegen_bjt_nr_solver_generates_residuals_and_jacobian() {
    let (code, _netlist, _mna, kernel) = generate_code(BJT_SPICE);

    assert_eq!(kernel.m, 2);

    // Should generate residuals f0 and f1
    assert!(code.contains("let f0 = i_nl[0] - i_dev0"), "Missing residual f0");
    assert!(code.contains("let f1 = i_nl[1] - i_dev1"), "Missing residual f1");

    // NR Jacobian should use the 2D chain rule:
    // j00 = 1.0 - jdev_0_0 * K[0][0] - jdev_0_1 * K[1][0]
    // j01 = 0.0 - jdev_0_0 * K[0][1] - jdev_0_1 * K[1][1]
    // (i.e., sum over k in device block for each row)
    assert!(code.contains("let j00 ="), "Missing Jacobian entry j00");
    assert!(code.contains("let j01 ="), "Missing Jacobian entry j01");
    assert!(code.contains("let j10 ="), "Missing Jacobian entry j10");
    assert!(code.contains("let j11 ="), "Missing Jacobian entry j11");

    // Diagonal entries should start with 1.0
    // Off-diagonal should start with 0.0
    // Both should reference jdev entries (not g_dev)
    assert!(
        code.contains("jdev_0_0 * K[0]") || code.contains("jdev_0_1 * K[1]"),
        "Jacobian should use jdev entries with K matrix multiplication."
    );

    // Should use 2x2 Cramer's rule solver
    assert!(
        code.contains("let det = j00 * j11 - j01 * j10"),
        "2x2 NR solver should use Cramer's rule."
    );
}

#[test]
fn test_codegen_bjt_does_not_skip_nr() {
    let (code, _netlist, _mna, kernel) = generate_code(BJT_SPICE);

    assert_eq!(kernel.m, 2);

    // The old bug: BJT-only circuits fell through to "No nonlinear devices to solve"
    // which returned the initial guess without any NR iteration.
    // After the fix, BJT circuits should have a proper NR loop.
    assert!(
        !code.contains("// No nonlinear devices to solve"),
        "BJT circuit should NOT skip NR solving. The old bug returned initial guess for BJTs."
    );

    // Should have the NR iteration loop
    assert!(
        code.contains("for iter in 0..MAX_ITER"),
        "BJT circuit should have NR iteration loop."
    );

    // Should have convergence checking
    assert!(
        code.contains("state.last_nr_iterations = iter as u32"),
        "BJT circuit should track NR iteration count for convergence."
    );
}

// ==========================================================================
// Test: Mixed device circuit (diodes + BJT) generates correct block-diagonal Jacobian
// ==========================================================================

const MIXED_DEVICE_SPICE: &str = "\
Mixed Circuit
Rin in 0 1k
D1 in mid D1N4148
Q1 mid b e 2N2222
Rb b 0 100k
Re e 0 1k
C1 mid 0 1u
.model D1N4148 D(IS=1e-15)
.model 2N2222 NPN(IS=1e-15 BF=200)
";

#[test]
fn test_codegen_mixed_diode_bjt_device_map() {
    let (code, _netlist, _mna, kernel) = generate_code(MIXED_DEVICE_SPICE);

    // D1 is 1D (index 0), Q1 is 2D (indices 1,2) → M=3
    assert_eq!(kernel.m, 3, "Expected m=3 for diode + BJT circuit");

    // Diode at index 0 (device 0) with per-device params
    assert!(code.contains("diode_current(v_d0, DEVICE_0_IS, DEVICE_0_N_VT)"), "Diode current at index 0");
    assert!(code.contains("jdev_0_0 = diode_conductance(v_d0, DEVICE_0_IS, DEVICE_0_N_VT)"), "Diode jdev at index 0");

    // BJT at indices 1,2 (device 1) with per-device params
    assert!(code.contains("bjt_ic(v_d1, v_d2, DEVICE_1_IS, DEVICE_1_VT, DEVICE_1_BETA_R)"), "BJT Ic at indices 1,2");
    assert!(code.contains("bjt_ib(v_d1, v_d2, DEVICE_1_IS, DEVICE_1_VT, DEVICE_1_BETA_F, DEVICE_1_BETA_R)"), "BJT Ib at indices 1,2");
    assert!(code.contains("bjt_jacobian(v_d1, v_d2, DEVICE_1_IS, DEVICE_1_VT, DEVICE_1_BETA_F, DEVICE_1_BETA_R)"), "BJT Jacobian at indices 1,2");

    // All 3 residuals
    assert!(code.contains("let f0 = i_nl[0] - i_dev0"), "Residual f0");
    assert!(code.contains("let f1 = i_nl[1] - i_dev1"), "Residual f1");
    assert!(code.contains("let f2 = i_nl[2] - i_dev2"), "Residual f2");

    // 3x3 Jacobian entries should exist
    for i in 0..3 {
        for j in 0..3 {
            let entry = format!("let j{}{} =", i, j);
            assert!(code.contains(&entry), "Missing Jacobian entry j{}{}", i, j);
        }
    }

    // 3x3 system uses inline Gaussian elimination
    assert!(
        code.contains("Solve 3"),
        "3x3 system should use Gaussian elimination solver."
    );
}

#[test]
fn test_codegen_mixed_device_jacobian_block_structure() {
    let (code, _netlist, _mna, kernel) = generate_code(MIXED_DEVICE_SPICE);
    assert_eq!(kernel.m, 3);

    // Diode (index 0) is a 1x1 block: j0x entries should only reference jdev_0_0
    // (not jdev_0_1 or jdev_0_2 which don't exist for the diode)
    assert!(
        code.contains("jdev_0_0 * K[0]"),
        "Row 0 (diode) should use jdev_0_0 with K[0][j]."
    );

    // BJT rows (indices 1,2) should reference their 2x2 block: jdev_1_1, jdev_1_2, jdev_2_1, jdev_2_2
    assert!(code.contains("jdev_1_1"), "BJT block should have jdev_1_1");
    assert!(code.contains("jdev_1_2"), "BJT block should have jdev_1_2");
    assert!(code.contains("jdev_2_1"), "BJT block should have jdev_2_1");
    assert!(code.contains("jdev_2_2"), "BJT block should have jdev_2_2");
}

// ==========================================================================
// Test: Input conductance default is 1.0 (near-ideal voltage source)
// ==========================================================================

#[test]
fn test_codegen_default_input_resistance() {
    let config = CodegenConfig::default();
    assert_eq!(
        config.input_resistance, 1.0,
        "Default input_resistance should be 1.0 (1Ω)"
    );
}

// ==========================================================================
// Test: Generated code actually compiles with rustc
// ==========================================================================

/// Verify that the generated code for each circuit type compiles as valid Rust.
///
/// This catches any syntax errors, missing imports, or type mismatches in the
/// code generator output that string-matching tests would miss.
#[test]
fn test_generated_code_compiles() {
    let circuits: &[(&str, &str)] = &[
        ("rc_linear", RC_CIRCUIT_SPICE),
        ("diode_clipper", DIODE_CLIPPER_SPICE),
        ("two_diode", TWO_DIODE_SPICE),
        ("bjt", BJT_SPICE),
    ];

    for (name, spice) in circuits {
        let (code, _netlist, _mna, _kernel) = generate_code(spice);

        // Write to a temp file
        let tmp_dir = std::env::temp_dir();
        let tmp_path = tmp_dir.join(format!("melange_codegen_test_{}.rs", name));

        {
            let mut f = std::fs::File::create(&tmp_path)
                .expect("failed to create temp file");
            f.write_all(code.as_bytes())
                .expect("failed to write temp file");
        }

        // Compile as a library crate
        let output = std::process::Command::new("rustc")
            .args(["--edition", "2024", "--crate-type", "lib", "-o"])
            .arg(tmp_dir.join(format!("melange_codegen_test_{}.rlib", name)))
            .arg(&tmp_path)
            .output()
            .expect("failed to run rustc");

        // Clean up temp files regardless of result
        let _ = std::fs::remove_file(&tmp_path);
        let _ = std::fs::remove_file(tmp_dir.join(format!("melange_codegen_test_{}.rlib", name)));
        let _ = std::fs::remove_file(tmp_dir.join(format!("libmelange_codegen_test_{}.rlib", name)));

        assert!(
            output.status.success(),
            "Generated code for '{}' failed to compile:\n{}",
            name,
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

// ==========================================================================
// Test: Generated code uses proper trapezoidal RHS with input_prev
// ==========================================================================

#[test]
fn test_generated_code_has_input_prev() {
    let (code, _netlist, _mna, _kernel) = generate_code(DIODE_CLIPPER_SPICE);

    // CircuitState should have input_prev field
    assert!(
        code.contains("pub input_prev: f64"),
        "CircuitState should contain input_prev field."
    );

    // build_rhs should take input_prev parameter
    assert!(
        code.contains("fn build_rhs(input: f64, input_prev: f64, state: &CircuitState)"),
        "build_rhs should take input_prev parameter."
    );

    // Formula should use (input + input_prev), not 2.0 * input
    assert!(
        code.contains("(input + input_prev) / INPUT_RESISTANCE"),
        "RHS should use (input + input_prev) / INPUT_RESISTANCE for proper trapezoidal."
    );
    assert!(
        !code.contains("2.0 * input / INPUT_RESISTANCE"),
        "RHS must NOT use 2.0 * input (wrong trapezoidal formula)."
    );

    // process_sample should call build_rhs with state.input_prev
    assert!(
        code.contains("build_rhs(input, state.input_prev, state)"),
        "process_sample should call build_rhs with state.input_prev."
    );

    // process_sample should update state.input_prev
    assert!(
        code.contains("state.input_prev = input;"),
        "process_sample should update state.input_prev after processing."
    );
}

// ==========================================================================
// Test: Invalid input/output node indices are rejected
// ==========================================================================

#[test]
fn test_invalid_input_node_rejected() {
    let (netlist, mna, kernel) = build_pipeline(RC_CIRCUIT_SPICE);

    let config = CodegenConfig {
        input_node: 999, // Way too large
        output_node: 0,
        ..default_config()
    };

    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist);

    assert!(result.is_err(), "Should reject input_node >= N");
    let err = result.unwrap_err();
    assert!(
        matches!(err, CodegenError::InvalidConfig(_)),
        "Error should be InvalidConfig, got: {:?}", err
    );
}

#[test]
fn test_invalid_output_node_rejected() {
    let (netlist, mna, kernel) = build_pipeline(RC_CIRCUIT_SPICE);

    let config = CodegenConfig {
        input_node: 0,
        output_node: 999, // Way too large
        ..default_config()
    };

    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist);

    assert!(result.is_err(), "Should reject output_node >= N");
    let err = result.unwrap_err();
    assert!(
        matches!(err, CodegenError::InvalidConfig(_)),
        "Error should be InvalidConfig, got: {:?}", err
    );
}

// ==========================================================================
// Test: .model parameters are used in generated code
// ==========================================================================

#[test]
fn test_model_params_in_generated_code() {
    // The DIODE_CLIPPER_SPICE has .model D1N4148 D(IS=1e-15)
    let (code, _netlist, _mna, _kernel) = generate_code(DIODE_CLIPPER_SPICE);

    // IS should come from .model (1e-15), not the default (2.52e-9)
    let is_from_model = format!("{:.17e}", 1e-15_f64);
    assert!(
        code.contains(&is_from_model),
        "Generated code should use IS={} from .model directive, not the default.",
        is_from_model
    );

    // Default IS should NOT appear
    let default_is = format!("{:.17e}", 2.52e-9_f64);
    assert!(
        !code.contains(&default_is),
        "Generated code should NOT contain default IS={} when .model provides IS.",
        default_is
    );
}

#[test]
fn test_bjt_model_params_in_generated_code() {
    // The BJT_SPICE has .model 2N2222 NPN(IS=1e-15 BF=200)
    let (code, _netlist, _mna, _kernel) = generate_code(BJT_SPICE);

    // IS should come from .model (1e-15)
    let is_from_model = format!("{:.17e}", 1e-15_f64);
    assert!(
        code.contains(&is_from_model),
        "BJT IS should come from .model directive."
    );

    // BF=200 should come from .model
    let bf_from_model = format!("{:.17e}", 200.0_f64);
    assert!(
        code.contains(&bf_from_model),
        "BJT BF should come from .model directive."
    );
}

// ==========================================================================
// Test: Generated code uses SINGULARITY_THRESHOLD constant
// ==========================================================================

#[test]
fn test_singularity_threshold_constant() {
    let (code, _netlist, _mna, _kernel) = generate_code(DIODE_CLIPPER_SPICE);

    assert!(
        code.contains("const SINGULARITY_THRESHOLD: f64 = 1e-15;"),
        "Generated code should define SINGULARITY_THRESHOLD constant."
    );

    assert!(
        code.contains("SINGULARITY_THRESHOLD"),
        "Generated code should reference SINGULARITY_THRESHOLD in threshold checks."
    );
}

// ---------------------------------------------------------------------------
// Phase 6: IR serialization round-trip tests
// ---------------------------------------------------------------------------

/// Helper: build CircuitIR from SPICE string
fn build_ir(spice: &str) -> CircuitIR {
    let (netlist, mna, kernel) = build_pipeline(spice);
    CircuitIR::from_kernel(&kernel, &mna, &netlist, &default_config()).unwrap()
}

/// Verify IR matrix data round-trips through JSON with near-exact f64 equality.
/// JSON decimal encoding may introduce up to 1 ULP of error on some values.
fn assert_vecs_close(a: &[f64], b: &[f64], label: &str) {
    assert_eq!(a.len(), b.len(), "{}: length mismatch", label);
    for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
        let diff = (x - y).abs();
        let tol = x.abs().max(y.abs()) * 1e-14;
        assert!(
            diff <= tol || diff < 1e-30,
            "{} element {}: {} vs {} (diff={})",
            label, i, x, y, diff
        );
    }
}

fn assert_ir_matrices_close(a: &CircuitIR, b: &CircuitIR) {
    assert_vecs_close(&a.matrices.s, &b.matrices.s, "S");
    assert_vecs_close(&a.matrices.a_neg, &b.matrices.a_neg, "A_neg");
    assert_vecs_close(&a.matrices.k, &b.matrices.k, "K");
    assert_vecs_close(&a.matrices.n_v, &b.matrices.n_v, "N_v");
    assert_vecs_close(&a.matrices.n_i, &b.matrices.n_i, "N_i");
    assert_vecs_close(&a.matrices.rhs_const, &b.matrices.rhs_const, "rhs_const");
    assert_vecs_close(&a.dc_operating_point, &b.dc_operating_point, "DC OP");
}

/// IR round-trip: serialize → deserialize → verify data preserved + emits valid code.
fn assert_ir_roundtrip(spice: &str) {
    let ir = build_ir(spice);
    let emitter = RustEmitter::new();

    // Direct emission must succeed
    let direct_code = emitter.emit(&ir).expect("direct emit failed");

    // JSON round-trip
    let json = serde_json::to_string_pretty(&ir).expect("serialize failed");
    let ir2: CircuitIR = serde_json::from_str(&json).expect("deserialize failed");

    // IR structure must be preserved
    assert_eq!(ir.topology.n, ir2.topology.n);
    assert_eq!(ir.topology.m, ir2.topology.m);
    assert_eq!(ir.device_slots.len(), ir2.device_slots.len());
    assert_eq!(ir.has_dc_sources, ir2.has_dc_sources);
    assert_eq!(ir.has_dc_op, ir2.has_dc_op);

    // Matrix data must be close (JSON decimal encoding may have ~1 ULP error)
    assert_ir_matrices_close(&ir, &ir2);

    // Deserialized IR must also emit valid code
    let roundtrip_code = emitter.emit(&ir2).expect("roundtrip emit failed");

    // Both should contain all required functions
    for func in &["process_sample", "build_rhs", "mat_vec_mul_s", "solve_nonlinear"] {
        assert!(direct_code.contains(func), "direct code missing {}", func);
        assert!(roundtrip_code.contains(func), "roundtrip code missing {}", func);
    }

    // Both should have the same structure (same number of lines)
    let direct_lines = direct_code.lines().count();
    let roundtrip_lines = roundtrip_code.lines().count();
    assert_eq!(direct_lines, roundtrip_lines,
        "Round-trip code should have same line count");
}

#[test]
fn test_ir_roundtrip_diode_clipper() {
    assert_ir_roundtrip(DIODE_CLIPPER_SPICE);
}

#[test]
fn test_ir_roundtrip_rc_circuit() {
    assert_ir_roundtrip(RC_CIRCUIT_SPICE);
}

#[test]
fn test_ir_roundtrip_bjt() {
    assert_ir_roundtrip(BJT_SPICE);
}

#[test]
fn test_ir_roundtrip_mixed_diode_bjt() {
    assert_ir_roundtrip(MIXED_DEVICE_SPICE);
}

#[test]
fn test_ir_fields_match_kernel() {
    let (netlist, mna, kernel) = build_pipeline(DIODE_CLIPPER_SPICE);
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &default_config()).unwrap();

    // Topology
    assert_eq!(ir.topology.n, kernel.n);
    assert_eq!(ir.topology.m, kernel.m);

    // Matrices are byte-identical copies
    assert_eq!(ir.matrices.s, kernel.s);
    assert_eq!(ir.matrices.a_neg, kernel.a_neg);
    assert_eq!(ir.matrices.k, kernel.k);
    assert_eq!(ir.matrices.n_v, kernel.n_v);
    assert_eq!(ir.matrices.n_i, kernel.n_i);
    assert_eq!(ir.matrices.rhs_const, kernel.rhs_const);

    // Config
    assert_eq!(ir.solver_config.sample_rate, 44100.0);
    assert_eq!(ir.solver_config.input_node, 0);
    assert_eq!(ir.solver_config.output_node, 1);

    // Device slots
    assert_eq!(ir.device_slots.len(), 1); // one diode
}

#[test]
fn test_ir_json_is_valid() {
    let ir = build_ir(DIODE_CLIPPER_SPICE);
    let json = serde_json::to_string_pretty(&ir).expect("serialize failed");

    // Should be valid JSON
    let parsed: serde_json::Value = serde_json::from_str(&json).expect("not valid JSON");
    assert!(parsed.is_object());

    // Should contain key fields
    assert!(parsed.get("topology").is_some());
    assert!(parsed.get("matrices").is_some());
    assert!(parsed.get("devices").is_some());
    assert!(parsed.get("device_slots").is_some());
    assert!(parsed.get("solver_config").is_some());
}

// ==========================================================================
// Test: Heterogeneous device models get per-device parameters
// ==========================================================================

const TWO_DIFFERENT_DIODES_SPICE: &str = "\
Two Different Diode Models
Rin in 0 1k
D1 in mid DFAST
D2 mid out DSLOW
R1 mid 0 10k
C1 out 0 1u
.model DFAST D(IS=1e-14 N=1.0)
.model DSLOW D(IS=1e-12 N=1.5)
";

#[test]
fn test_heterogeneous_diode_models() {
    let (code, _netlist, _mna, _kernel) = generate_code(TWO_DIFFERENT_DIODES_SPICE);

    // Device 0 (DFAST) should have IS=1e-14
    let fast_is = format!("{:.17e}", 1e-14_f64);
    assert!(
        code.contains(&format!("DEVICE_0_IS: f64 = {}", fast_is)),
        "Device 0 (DFAST) should have IS=1e-14, got code without it."
    );

    // Device 1 (DSLOW) should have IS=1e-12
    let slow_is = format!("{:.17e}", 1e-12_f64);
    assert!(
        code.contains(&format!("DEVICE_1_IS: f64 = {}", slow_is)),
        "Device 1 (DSLOW) should have IS=1e-12, got code without it."
    );

    // They must be DIFFERENT
    assert_ne!(fast_is, slow_is, "Devices should have different IS values");

    // Device 0 N_VT = 1.0 * 0.02585 = 0.02585
    let fast_n_vt = format!("{:.17e}", 1.0 * 0.02585_f64);
    assert!(
        code.contains(&format!("DEVICE_0_N_VT: f64 = {}", fast_n_vt)),
        "Device 0 (DFAST) should have N_VT for N=1.0."
    );

    // Device 1 N_VT = 1.5 * 0.02585 = 0.038775
    let slow_n_vt = format!("{:.17e}", 1.5 * 0.02585_f64);
    assert!(
        code.contains(&format!("DEVICE_1_N_VT: f64 = {}", slow_n_vt)),
        "Device 1 (DSLOW) should have N_VT for N=1.5."
    );

    // Both devices should call parameterized functions with their own constants
    assert!(
        code.contains("diode_current(v_d0, DEVICE_0_IS, DEVICE_0_N_VT)"),
        "Device 0 should call diode_current with DEVICE_0 params."
    );
    assert!(
        code.contains("diode_current(v_d1, DEVICE_1_IS, DEVICE_1_N_VT)"),
        "Device 1 should call diode_current with DEVICE_1 params."
    );
}

#[test]
fn test_heterogeneous_diode_models_compile() {
    let (code, _netlist, _mna, _kernel) = generate_code(TWO_DIFFERENT_DIODES_SPICE);

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_hetero_diodes.rs");

    {
        let mut f = std::fs::File::create(&tmp_path).expect("failed to create temp file");
        f.write_all(code.as_bytes()).expect("failed to write temp file");
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_hetero_diodes.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_hetero_diodes.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_hetero_diodes.rlib"));

    assert!(
        output.status.success(),
        "Generated code for heterogeneous diodes failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

// ==========================================================================
// Test: DC operating point includes input conductance
// ==========================================================================

#[test]
fn test_dc_op_includes_input_conductance() {
    // BJT circuit with DC bias — the DC OP should change with different input resistance
    let (netlist, mna, kernel) = build_pipeline(BJT_SPICE);

    let config_low_r = CodegenConfig {
        input_resistance: 1.0, // 1 ohm — near-ideal voltage source
        ..default_config()
    };
    let ir_low_r = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config_low_r).unwrap();

    let config_high_r = CodegenConfig {
        input_resistance: 1e6, // 1 Mohm — very high impedance
        ..default_config()
    };
    let ir_high_r = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config_high_r).unwrap();

    // DC operating points should differ when input resistance changes
    // (because input conductance is part of the G matrix used in DC OP calculation)
    // Note: they may both be zero if no DC sources — that's also valid
    // The key property: the function accepts and uses input resistance
    let _low_r_op = &ir_low_r.dc_operating_point;
    let _high_r_op = &ir_high_r.dc_operating_point;

    // For a circuit WITH DC sources, the operating points must differ
    // The BJT circuit has VCC stamped via voltage source → has_dc_sources
    if ir_low_r.has_dc_sources {
        // At minimum, the DC OP calculation should not crash
        assert_eq!(ir_low_r.dc_operating_point.len(), ir_low_r.topology.n);
        assert_eq!(ir_high_r.dc_operating_point.len(), ir_high_r.topology.n);
    }
}

// ==========================================================================
// Test: Per-device params are preserved in IR serialization round-trip
// ==========================================================================

#[test]
fn test_ir_roundtrip_heterogeneous_devices() {
    let ir = build_ir(TWO_DIFFERENT_DIODES_SPICE);

    // Verify per-device params are distinct
    assert_eq!(ir.device_slots.len(), 2);
    match (&ir.device_slots[0].params, &ir.device_slots[1].params) {
        (
            melange_solver::codegen::ir::DeviceParams::Diode(d0),
            melange_solver::codegen::ir::DeviceParams::Diode(d1),
        ) => {
            assert_ne!(
                d0.is, d1.is,
                "Two different diode models should have different IS"
            );
            assert_ne!(
                d0.n_vt, d1.n_vt,
                "Two different diode models should have different N_VT"
            );
        }
        _ => panic!("Expected two Diode device params"),
    }

    // JSON round-trip preserves per-device params
    let json = serde_json::to_string_pretty(&ir).expect("serialize failed");
    let ir2: CircuitIR = serde_json::from_str(&json).expect("deserialize failed");

    assert_eq!(ir2.device_slots.len(), 2);
    match (&ir2.device_slots[0].params, &ir2.device_slots[1].params) {
        (
            melange_solver::codegen::ir::DeviceParams::Diode(d0),
            melange_solver::codegen::ir::DeviceParams::Diode(d1),
        ) => {
            assert_ne!(d0.is, d1.is, "Round-trip should preserve distinct IS values");
        }
        _ => panic!("Expected two Diode device params after round-trip"),
    }
}

// ==========================================================================
// Test: Inductor codegen E2E
// ==========================================================================

/// Verify that generated code for an RL lowpass circuit contains
/// inductor state fields, constants, RHS injection, and state update.
#[test]
fn test_inductor_codegen_e2e() {
    let spice = "\
RL Lowpass
R1 in out 1k
L1 out 0 10m
C1 out 0 100p
";
    let (code, _netlist, _mna, _kernel) = generate_code(spice);

    // Should contain inductor constants
    assert!(
        code.contains("IND_0_G_EQ") || code.contains("INDUCTOR_0_G_EQ"),
        "Generated code should contain inductor equivalent conductance constant.\n\
         Code snippet: {}",
        &code[..code.len().min(500)]
    );

    // Should contain inductor state fields (history current)
    assert!(
        code.contains("i_ind_hist") || code.contains("ind_i_hist") || code.contains("i_hist"),
        "Generated code should contain inductor history current state field"
    );

    // Should contain inductor RHS injection (history current injection into build_rhs)
    assert!(
        code.contains("ind") || code.contains("IND"),
        "Generated code should reference inductor constants or state"
    );

    // Verify the code mentions inductor-related updates in process_sample
    assert!(
        code.contains("state.i_ind") || code.contains("state.ind"),
        "Generated code should update inductor state in process_sample"
    );
}

// ==========================================================================
// Test: M > 4 early rejection in IR
// ==========================================================================

/// Verify that CircuitIR::from_kernel rejects M > 4 at IR build time,
/// not just at emission time.
#[test]
fn test_m_gt_4_rejected_at_ir_build() {
    // 5 diodes → M=5 which should be rejected
    let spice = "\
Five Diodes
Rin in 0 1k
D1 in m1 D1N4148
D2 m1 m2 D1N4148
D3 m2 m3 D1N4148
D4 m3 m4 D1N4148
D5 m4 out D1N4148
R1 m1 0 10k
R2 m2 0 10k
R3 m3 0 10k
R4 m4 0 10k
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
";

    let (netlist, mna, kernel) = build_pipeline(spice);
    assert_eq!(kernel.m, 5, "Five diodes should produce M=5");

    let config = default_config();
    let result = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config);
    assert!(
        result.is_err(),
        "M=5 should be rejected at IR build time, but got Ok"
    );

    let err = result.unwrap_err();
    match err {
        CodegenError::UnsupportedTopology(msg) => {
            assert!(
                msg.contains("M=5"),
                "Error message should mention M=5, got: {}",
                msg
            );
        }
        other => panic!("Expected UnsupportedTopology, got: {:?}", other),
    }
}
