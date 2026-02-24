//! Codegen verification tests.
//!
//! Verifies that generated code produces correct results matching the runtime solver.
//! The codegen module generates standalone Rust code for a specific circuit topology.
//! These tests check that the generated source strings contain correct formulas,
//! matrix constants, and coefficient values matching the DK kernel.

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;

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
        input_conductance: 0.001,
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
        code.contains("DIODE_IS"),
        "Generated code should include DIODE_IS constant."
    );
    assert!(
        code.contains("DIODE_N_VT"),
        "Generated code should include DIODE_N_VT constant."
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
        code.contains("BJT_IS"),
        "Generated code should include BJT_IS constant."
    );
    assert!(
        code.contains("BJT_BETA_F"),
        "Generated code should include BJT_BETA_F constant."
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

    // BJT NR solver should call bjt_ic and bjt_ib for device currents
    assert!(
        code.contains("bjt_ic(v_d0, v_d1)"),
        "NR solver should call bjt_ic(v_d0, v_d1) for collector current."
    );
    assert!(
        code.contains("bjt_ib(v_d0, v_d1)"),
        "NR solver should call bjt_ib(v_d0, v_d1) for base current."
    );

    // Should call bjt_jacobian for the 2x2 device Jacobian
    assert!(
        code.contains("bjt_jacobian(v_d0, v_d1)"),
        "NR solver should call bjt_jacobian(v_d0, v_d1) for device Jacobian."
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

    // Diode at index 0
    assert!(code.contains("diode_current(v_d0)"), "Diode current at index 0");
    assert!(code.contains("jdev_0_0 = diode_conductance(v_d0)"), "Diode jdev at index 0");

    // BJT at indices 1,2
    assert!(code.contains("bjt_ic(v_d1, v_d2)"), "BJT Ic at indices 1,2");
    assert!(code.contains("bjt_ib(v_d1, v_d2)"), "BJT Ib at indices 1,2");
    assert!(code.contains("bjt_jacobian(v_d1, v_d2)"), "BJT Jacobian at indices 1,2");

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
fn test_codegen_default_input_conductance() {
    let config = CodegenConfig::default();
    assert_eq!(
        config.input_resistance, 1.0,
        "Default input_resistance should be 1.0 (1Ω)"
    );
    assert_eq!(
        config.input_conductance, 1.0,
        "Default input_conductance should be 1.0 (1/1Ω)"
    );
}
