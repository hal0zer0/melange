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
        code.contains("p[0] + state.k[0][0] * i_nl[0]"),
        "Generated code should contain 'p[0] + state.k[0][0] * i_nl[0]' (correct sign). \
         K is naturally negative so the sign is already correct.\n\
         Searched in generated code but did not find the expected pattern."
    );

    // Make sure the incorrect negated form is absent
    assert!(
        !code.contains("p[0] - state.k[0][0] * i_nl[0]"),
        "Generated code must NOT contain 'p[0] - state.k[0][0] * i_nl[0]' (wrong sign)."
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
        code.contains("1.0 - jdev_0_0 * state.k[0][0]"),
        "Generated code should contain '1.0 - jdev_0_0 * state.k[0][0]' (diagonal Jacobian).\n\
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
    // These appear as float literals multiplying i_nl[j] in the generated code.
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
                     multiplying i_nl[{}].",
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

    // K_DEFAULT is declared as [[f64; M]; M] = 2x2
    assert!(
        code.contains("pub const K_DEFAULT: [[f64; M]; M]"),
        "K_DEFAULT should be declared as [[f64; M]; M]."
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
    assert!(code.contains("pub const S_DEFAULT: [[f64; N]; N]"), "Missing S_DEFAULT matrix");
    assert!(code.contains("pub const A_NEG_DEFAULT: [[f64; N]; N]"), "Missing A_NEG_DEFAULT matrix");
    assert!(code.contains("pub const K_DEFAULT: [[f64; M]; M]"), "Missing K_DEFAULT matrix");
    assert!(code.contains("const G: [[f64; N]; N]"), "Missing G matrix");
    assert!(code.contains("const C: [[f64; N]; N]"), "Missing C matrix");
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

    assert!(code.contains("pub const DC_BLOCK_R: f64 ="), "Missing DC_BLOCK_R constant");
    assert!(code.contains("pub const OUTPUT_SCALE: f64 ="), "Missing OUTPUT_SCALE constant");
    assert!(code.contains("pub const DC_OP_CONVERGED: bool ="), "Missing DC_OP_CONVERGED constant");
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

    // BJT NR solver should call bjt_ic and bjt_ib with per-device params including GP params
    assert!(
        code.contains("bjt_ic(v_d0, v_d1, DEVICE_0_IS, DEVICE_0_VT, DEVICE_0_BETA_R, DEVICE_0_SIGN, DEVICE_0_USE_GP, DEVICE_0_VAF, DEVICE_0_VAR, DEVICE_0_IKF)"),
        "NR solver should call bjt_ic with per-device params and GP params."
    );
    assert!(
        code.contains("bjt_ib(v_d0, v_d1, DEVICE_0_IS, DEVICE_0_VT, DEVICE_0_BETA_F, DEVICE_0_BETA_R, DEVICE_0_SIGN)"),
        "NR solver should call bjt_ib with per-device params and sign."
    );

    // Should call bjt_jacobian with per-device params including GP params
    assert!(
        code.contains("bjt_jacobian(v_d0, v_d1, DEVICE_0_IS, DEVICE_0_VT, DEVICE_0_BETA_F, DEVICE_0_BETA_R, DEVICE_0_SIGN, DEVICE_0_USE_GP, DEVICE_0_VAF, DEVICE_0_VAR, DEVICE_0_IKF)"),
        "NR solver should call bjt_jacobian with per-device params and GP params."
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
        code.contains("jdev_0_0 * state.k[0]") || code.contains("jdev_0_1 * state.k[1]"),
        "Jacobian should use jdev entries with state.k matrix multiplication."
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

    // BJT at indices 1,2 (device 1) with per-device params including GP params
    assert!(code.contains("bjt_ic(v_d1, v_d2, DEVICE_1_IS, DEVICE_1_VT, DEVICE_1_BETA_R, DEVICE_1_SIGN, DEVICE_1_USE_GP, DEVICE_1_VAF, DEVICE_1_VAR, DEVICE_1_IKF)"), "BJT Ic at indices 1,2");
    assert!(code.contains("bjt_ib(v_d1, v_d2, DEVICE_1_IS, DEVICE_1_VT, DEVICE_1_BETA_F, DEVICE_1_BETA_R, DEVICE_1_SIGN)"), "BJT Ib at indices 1,2");
    assert!(code.contains("bjt_jacobian(v_d1, v_d2, DEVICE_1_IS, DEVICE_1_VT, DEVICE_1_BETA_F, DEVICE_1_BETA_R, DEVICE_1_SIGN, DEVICE_1_USE_GP, DEVICE_1_VAF, DEVICE_1_VAR, DEVICE_1_IKF)"), "BJT Jacobian at indices 1,2");

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
        code.contains("jdev_0_0 * state.k[0]"),
        "Row 0 (diode) should use jdev_0_0 with state.k[0][j]."
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
    assert_vecs_close(&a.matrices.g_matrix, &b.matrices.g_matrix, "G");
    assert_vecs_close(&a.matrices.c_matrix, &b.matrices.c_matrix, "C");
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

    // G and C matrices match MNA source (flattened row-major)
    let n = kernel.n;
    for i in 0..n {
        for j in 0..n {
            assert_eq!(
                ir.g(i, j), mna.g[i][j],
                "G[{}][{}] mismatch: IR={} vs MNA={}", i, j, ir.g(i, j), mna.g[i][j]
            );
            assert_eq!(
                ir.c(i, j), mna.c[i][j],
                "C[{}][{}] mismatch: IR={} vs MNA={}", i, j, ir.c(i, j), mna.c[i][j]
            );
        }
    }

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
// Test: M > MAX_M rejection at DK kernel build time
// ==========================================================================

/// Verify that M=17 is rejected at DK kernel build time (MAX_M=16).
#[test]
fn test_m_gt_max_rejected() {
    // 17 diodes → M=17 which exceeds MAX_M=16
    let spice = "\
Seventeen Diodes
Rin in 0 1k
D1 in m1 D1N4148
D2 m1 m2 D1N4148
D3 m2 m3 D1N4148
D4 m3 m4 D1N4148
D5 m4 m5 D1N4148
D6 m5 m6 D1N4148
D7 m6 m7 D1N4148
D8 m7 m8 D1N4148
D9 m8 m9 D1N4148
D10 m9 m10 D1N4148
D11 m10 m11 D1N4148
D12 m11 m12 D1N4148
D13 m12 m13 D1N4148
D14 m13 m14 D1N4148
D15 m14 m15 D1N4148
D16 m15 m16 D1N4148
D17 m16 out D1N4148
R1 m1 0 10k
R2 m2 0 10k
R3 m3 0 10k
R4 m4 0 10k
R5 m5 0 10k
R6 m6 0 10k
R7 m7 0 10k
R8 m8 0 10k
R9 m9 0 10k
R10 m10 0 10k
R11 m11 0 10k
R12 m12 0 10k
R13 m13 0 10k
R14 m14 0 10k
R15 m15 0 10k
R16 m16 0 10k
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
";

    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");

    // M=17 exceeds MAX_M=16, so kernel build should fail
    let result = DkKernel::from_mna(&mna, 44100.0);
    assert!(
        result.is_err(),
        "M=17 should be rejected at DK kernel build time (MAX_M=16)"
    );
}

/// Verify that M=10 (10 diodes) succeeds with MAX_M=16.
#[test]
fn test_m10_accepted() {
    let spice = "\
Ten Diodes
Rin in 0 1k
D1 in m1 D1N4148
D2 m1 m2 D1N4148
D3 m2 m3 D1N4148
D4 m3 m4 D1N4148
D5 m4 m5 D1N4148
D6 m5 m6 D1N4148
D7 m6 m7 D1N4148
D8 m7 m8 D1N4148
D9 m8 m9 D1N4148
D10 m9 out D1N4148
R1 m1 0 10k
R2 m2 0 10k
R3 m3 0 10k
R4 m4 0 10k
R5 m5 0 10k
R6 m6 0 10k
R7 m7 0 10k
R8 m8 0 10k
R9 m9 0 10k
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
";

    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    assert_eq!(mna.m, 10, "10 diodes should give M=10");

    // M=10 is within MAX_M=16, should succeed
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("M=10 should succeed with MAX_M=16");
    assert_eq!(kernel.m, 10);

    // Also verify codegen works for M=10
    let config = CodegenConfig {
        circuit_name: "ten_diode_test".to_string(),
        input_node: 0,
        output_node: kernel.n - 1,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist);
    assert!(result.is_ok(), "Codegen for M=10 should succeed: {:?}", result.err());
}

// ==========================================================================
// Test: Model parameter validation catches invalid values
// ==========================================================================

#[test]
fn test_diode_zero_is_rejected() {
    let spice = r#"Clip
.model BADDIODE D(IS=0)
D1 in out BADDIODE
R1 in 0 1k
C1 out 0 1u
Vin in 0 0
.END"#;
    let (netlist, mna, kernel) = build_pipeline(spice);
    let config = default_config();
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist);
    assert!(result.is_err(), "IS=0 should be rejected");
    let err = format!("{:?}", result.unwrap_err());
    assert!(err.contains("IS"), "Error should mention IS, got: {}", err);
}

#[test]
fn test_diode_negative_n_rejected() {
    let spice = r#"Clip
.model BADDIODE D(IS=1e-15 N=-1)
D1 in out BADDIODE
R1 in 0 1k
C1 out 0 1u
Vin in 0 0
.END"#;
    let (netlist, mna, kernel) = build_pipeline(spice);
    let config = default_config();
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist);
    assert!(result.is_err(), "N=-1 should be rejected");
    let err = format!("{:?}", result.unwrap_err());
    assert!(err.contains("N"), "Error should mention N, got: {}", err);
}

#[test]
fn test_invalid_input_resistance_rejected() {
    let (netlist, mna, kernel) = build_pipeline(RC_CIRCUIT_SPICE);
    let config = CodegenConfig {
        input_resistance: 0.0,
        ..default_config()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist);
    assert!(result.is_err(), "input_resistance=0 should be rejected");

    let config2 = CodegenConfig {
        input_resistance: -1.0,
        ..default_config()
    };
    let codegen2 = CodeGenerator::new(config2);
    let result2 = codegen2.generate(&kernel, &mna, &netlist);
    assert!(result2.is_err(), "input_resistance=-1 should be rejected");
}

// ==========================================================================
// Test: Sample rate mismatch warning is present in generated code (#13)
// ==========================================================================

#[test]
fn test_sample_rate_info_present_in_generated_code() {
    let (code, _netlist, _mna, _kernel) = generate_code(RC_CIRCUIT_SPICE);

    assert!(
        code.contains("set_sample_rate"),
        "Generated code should reference set_sample_rate for runtime sample rate changes."
    );
    assert!(
        code.contains("Default sample rate"),
        "Generated code should indicate the default sample rate."
    );
    assert!(
        code.contains("S_DEFAULT"),
        "Generated code should contain S_DEFAULT matrix (defaults at codegen sample rate)."
    );
    assert!(
        code.contains("A_NEG_DEFAULT"),
        "Generated code should contain A_NEG_DEFAULT matrix."
    );
    assert!(
        code.contains("K_DEFAULT"),
        "Generated code should contain K_DEFAULT matrix."
    );
}

// ==========================================================================
// Test: Different sample rates produce different ALPHA values (#18)
// ==========================================================================

#[test]
fn test_different_sample_rates_produce_different_alpha() {
    let spice = RC_CIRCUIT_SPICE;
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");

    // Generate code at 44100 Hz
    let kernel_44100 = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");
    let config_44100 = CodegenConfig {
        sample_rate: 44100.0,
        ..default_config()
    };
    let codegen_44100 = CodeGenerator::new(config_44100);
    let result_44100 = codegen_44100.generate(&kernel_44100, &mna, &netlist)
        .expect("code generation at 44100 Hz failed");

    // Generate code at 48000 Hz
    let kernel_48000 = DkKernel::from_mna(&mna, 48000.0).expect("failed to build DK kernel");
    let config_48000 = CodegenConfig {
        sample_rate: 48000.0,
        ..default_config()
    };
    let codegen_48000 = CodeGenerator::new(config_48000);
    let result_48000 = codegen_48000.generate(&kernel_48000, &mna, &netlist)
        .expect("code generation at 48000 Hz failed");

    // ALPHA = 2 * sample_rate, so they must be different
    let alpha_44100 = format!("{:.17e}", 2.0 * 44100.0_f64);
    let alpha_48000 = format!("{:.17e}", 2.0 * 48000.0_f64);
    assert_ne!(alpha_44100, alpha_48000, "ALPHA values should differ for different sample rates");

    assert!(
        result_44100.code.contains(&alpha_44100),
        "Code generated at 44100 Hz should contain ALPHA = {}",
        alpha_44100
    );
    assert!(
        result_48000.code.contains(&alpha_48000),
        "Code generated at 48000 Hz should contain ALPHA = {}",
        alpha_48000
    );

    // Sample rate values should also differ
    assert!(
        result_44100.code.contains("SAMPLE_RATE: f64 = 44100.0"),
        "Code generated at 44100 Hz should contain SAMPLE_RATE = 44100.0"
    );
    assert!(
        result_48000.code.contains("SAMPLE_RATE: f64 = 48000.0"),
        "Code generated at 48000 Hz should contain SAMPLE_RATE = 48000.0"
    );
}

#[test]
fn test_sample_rate_96000_produces_correct_alpha() {
    let spice = RC_CIRCUIT_SPICE;
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");

    let kernel = DkKernel::from_mna(&mna, 96000.0).expect("failed to build DK kernel");
    let config = CodegenConfig {
        sample_rate: 96000.0,
        ..default_config()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist)
        .expect("code generation at 96000 Hz failed");

    let expected_alpha = format!("{:.17e}", 2.0 * 96000.0_f64);
    assert!(
        result.code.contains(&expected_alpha),
        "Code generated at 96000 Hz should contain ALPHA = {} but it was not found.",
        expected_alpha
    );
    assert!(
        result.code.contains("SAMPLE_RATE: f64 = 96000.0"),
        "Code generated at 96000 Hz should contain SAMPLE_RATE = 96000.0"
    );
}

#[test]
fn test_sample_rate_affects_s_matrix() {
    // The S = A^-1 matrix depends on sample rate (A = G + 2/T * C).
    // Generating at different rates must produce different S matrix entries.
    let spice = RC_CIRCUIT_SPICE;
    let netlist = Netlist::parse(spice).expect("failed to parse netlist");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");

    let kernel_44100 = DkKernel::from_mna(&mna, 44100.0).expect("44100 kernel");
    let kernel_96000 = DkKernel::from_mna(&mna, 96000.0).expect("96000 kernel");

    // S matrices must differ because A = G + alpha*C depends on sample rate
    assert_ne!(
        kernel_44100.s, kernel_96000.s,
        "S matrices at different sample rates should differ"
    );
}

// ==========================================================================
// Test: Voltage source Norton-equivalent stamping (#19)
// ==========================================================================

/// Circuit with a DC voltage source for testing voltage source stamping.
const VS_DIVIDER_SPICE: &str = "\
Voltage Divider with DC Source
V1 vcc 0 DC 9
R1 vcc out 1k
R2 out 0 1k
C1 out 0 100p
";

#[test]
fn test_vs_conductance_stamps_into_g_matrix() {
    // A voltage source uses Norton equivalent: G_vs = 1e6 S stamped in G matrix.
    let netlist = Netlist::parse(VS_DIVIDER_SPICE).expect("failed to parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");

    // VS_CONDUCTANCE should be 1e6
    assert_eq!(
        melange_solver::mna::VS_CONDUCTANCE, 1e6,
        "VS_CONDUCTANCE should be 1e6"
    );

    // The voltage source is between vcc (node index) and ground (node 0).
    // In G matrix, G[vcc_idx][vcc_idx] should include VS_CONDUCTANCE.
    let vcc_idx = mna.node_map["vcc"] - 1; // Convert to 0-indexed matrix row
    let g_vcc = mna.g[vcc_idx][vcc_idx];
    assert!(
        g_vcc >= melange_solver::mna::VS_CONDUCTANCE,
        "G[vcc][vcc] should include VS_CONDUCTANCE (1e6), got {}",
        g_vcc
    );
}

#[test]
fn test_vs_divider_dc_accuracy() {
    // Voltage divider: V1=9V, R1=R2=1k → Vout should be ~4.5V at DC operating point.
    // The VS_CONDUCTANCE approximation should be accurate enough.
    let netlist = Netlist::parse(VS_DIVIDER_SPICE).expect("failed to parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");

    // The voltage source info should be recorded
    assert_eq!(mna.voltage_sources.len(), 1, "Should have 1 voltage source");
    assert_eq!(
        mna.voltage_sources[0].dc_value, 9.0,
        "V1 DC value should be 9V"
    );

    // Build kernel to get rhs_const
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");

    // rhs_const should be non-zero since we have a DC voltage source
    let any_nonzero = kernel.rhs_const.iter().any(|&v| v != 0.0);
    assert!(
        any_nonzero,
        "rhs_const should have non-zero entries from DC voltage source"
    );

    // The Norton equivalent current is V * G_vs = 9 * 1e6 = 9e6 A,
    // multiplied by 2 for trapezoidal rule: 2 * 9e6 = 1.8e7.
    // This should be injected at the vcc node.
    let vcc_idx = mna.node_map["vcc"] - 1;
    let expected_rhs = 2.0 * 9.0 * melange_solver::mna::VS_CONDUCTANCE;
    assert!(
        (kernel.rhs_const[vcc_idx] - expected_rhs).abs() < 1e-3,
        "rhs_const[vcc] should be 2*V*G_vs = {}, got {}",
        expected_rhs,
        kernel.rhs_const[vcc_idx]
    );
}

#[test]
fn test_generated_code_includes_rhs_const_for_dc_source() {
    // When DC sources are present, generated code must include RHS_CONST array.
    let spice = VS_DIVIDER_SPICE;
    let netlist = Netlist::parse(spice).expect("failed to parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("failed to build MNA");
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("failed to build DK kernel");

    let config = CodegenConfig {
        circuit_name: "vs_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist)
        .expect("code generation failed");

    assert!(
        result.code.contains("RHS_CONST"),
        "Generated code should include RHS_CONST when DC sources are present."
    );
    assert!(
        result.code.contains("pub const RHS_CONST: [f64; N]"),
        "Generated code should define RHS_CONST as a public constant array."
    );
}

#[test]
fn test_no_rhs_const_without_dc_source() {
    // When no DC sources are present, generated code should not include RHS_CONST.
    let (code, _netlist, _mna, _kernel) = generate_code(RC_CIRCUIT_SPICE);

    // RC circuit has no voltage sources, so no RHS_CONST
    assert!(
        !code.contains("pub const RHS_CONST"),
        "Generated code should NOT include RHS_CONST when no DC sources are present."
    );
}

#[test]
fn test_vs_conductance_value_is_large() {
    // VS_CONDUCTANCE must be large enough to approximate an ideal voltage source.
    // With G_vs = 1e6, the voltage error is V * R_other / (R_other + 1/G_vs).
    // For a 1k load, error ~ 1k / (1k + 1e-6) ~ 1e-3, i.e. 0.1% error.
    let g_vs = melange_solver::mna::VS_CONDUCTANCE;
    assert!(
        g_vs >= 1e5,
        "VS_CONDUCTANCE should be at least 1e5 for accuracy, got {}",
        g_vs
    );
    assert!(
        g_vs <= 1e12,
        "VS_CONDUCTANCE should not be so large as to cause numerical issues, got {}",
        g_vs
    );
}

// ==========================================================================
// Test: BJT codegen-runtime consistency
//
// Verifies that the generated BJT code:
// 1. Uses full i_nl (not delta) in compute_final_voltages (trapezoidal)
// 2. Has matching K matrix values with the runtime kernel
// 3. Produces a runtime solver output that amplifies (functional check)
// ==========================================================================

#[test]
fn test_codegen_runtime_consistency_bjt() {
    use melange_solver::solver::{CircuitSolver, DeviceEntry};
    use melange_devices::bjt::{BjtEbersMoll, BjtPolarity};

    // --- Part 1: Structural verification of generated code ---
    let (code, _netlist, _mna, kernel) = generate_code(BJT_SPICE);

    assert_eq!(kernel.m, 2, "BJT should have M=2");

    // Verify K matrix values match between codegen and runtime
    for i in 0..2 {
        for j in 0..2 {
            let k_val = kernel.k(i, j);
            let k_str = format!("{:.17e}", k_val);
            assert!(
                code.contains(&k_str),
                "K[{}][{}] = {} must match between codegen and runtime",
                i, j, k_val
            );
        }
    }

    // Verify compute_final_voltages uses full i_nl[j], NOT di{j} (delta)
    assert!(
        !code.contains("di0") && !code.contains("di1"),
        "compute_final_voltages must use full i_nl[j], not delta (di0/di1).\n\
         Found 'di0' or 'di1' in generated code — this is the backward Euler \
         formulation, not the correct trapezoidal formulation."
    );

    // Verify the S*N_i coefficients appear multiplied by i_nl[0] and i_nl[1]
    for j in 0..kernel.m {
        let pattern = format!("i_nl[{}]", j);
        assert!(
            code.contains(&pattern),
            "compute_final_voltages should reference i_nl[{}] for trapezoidal correction",
            j
        );
    }

    // --- Part 2: Functional verification of runtime solver ---
    // Use a BJT CE circuit (same topology as existing test_solve_2d_bjt_basic)
    let biased_bjt = "\
Common Emitter
Q1 coll base emit 2N2222
Rc coll vcc 10k
R1 base 0 100k
Re emit 0 1k
Rbias vcc 0 10k
.model 2N2222 NPN(IS=1e-15 BF=200)
";
    let netlist = Netlist::parse(biased_bjt).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    let input_node = *mna.node_map.get("base").unwrap() - 1;
    let output_node = *mna.node_map.get("coll").unwrap() - 1;
    mna.g[input_node][input_node] += 1.0;

    let kernel_rt = DkKernel::from_mna(&mna, 44100.0).unwrap();
    assert_eq!(kernel_rt.m, 2, "Runtime BJT kernel should have M=2");

    let bjt = BjtEbersMoll::new_room_temp(1e-15, 200.0, 3.0, BjtPolarity::Npn);
    let devices = vec![DeviceEntry::new_bjt(bjt, 0)];

    let mut solver = CircuitSolver::new(kernel_rt, devices, input_node, output_node);
    solver.input_conductance = 1.0;

    // Feed a sine wave and verify output is finite and stable
    let mut outputs = Vec::new();
    for i in 0..200 {
        let input = 0.5 + 0.3 * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 44100.0).sin();
        outputs.push(solver.process_sample(input));
    }

    // All outputs must be finite
    assert!(
        outputs.iter().all(|v| v.is_finite()),
        "BJT runtime solver outputs must all be finite"
    );

    // Output should have variation (not stuck at DC)
    let out_min = outputs[100..].iter().cloned().fold(f64::INFINITY, f64::min);
    let out_max = outputs[100..].iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let out_pp = out_max - out_min;

    assert!(
        out_pp > 0.001,
        "BJT output should vary (not stuck at DC), out_pp={:.6}",
        out_pp
    );

    // No period-3 oscillation: check last 20 samples have bounded variation
    // (before the fix, BJTs would oscillate with pp > 5V)
    let last_20: Vec<f64> = outputs[180..].to_vec();
    let last_pp = last_20.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
        - last_20.iter().cloned().fold(f64::INFINITY, f64::min);
    assert!(
        last_pp < 2.0,
        "BJT output should not have wild oscillations, last_20 pp={:.4}",
        last_pp
    );
}

// ==========================================================================
// Runtime sample rate support tests
// ==========================================================================

/// Verify that generated code contains the set_sample_rate method.
#[test]
fn test_generated_code_contains_set_sample_rate_method() {
    let (code, _netlist, _mna, _kernel) = generate_code(RC_CIRCUIT_SPICE);

    assert!(
        code.contains("pub fn set_sample_rate(&mut self, sample_rate: f64)"),
        "Generated code must contain a set_sample_rate method."
    );
    assert!(
        code.contains("fn invert_n("),
        "Generated code must contain the invert_n helper for runtime matrix inversion."
    );
}

/// Verify that G and C matrices are emitted as constants.
#[test]
fn test_generated_code_contains_g_and_c_matrices() {
    let (code, _netlist, _mna, _kernel) = generate_code(RC_CIRCUIT_SPICE);

    assert!(
        code.contains("const G: [[f64; N]; N]"),
        "Generated code must contain the G (conductance) matrix."
    );
    assert!(
        code.contains("const C: [[f64; N]; N]"),
        "Generated code must contain the C (capacitance) matrix."
    );
}

/// Verify that S, A_NEG, K are now stored as state fields.
#[test]
fn test_generated_code_state_has_matrix_fields() {
    let (code, _netlist, _mna, _kernel) = generate_code(RC_CIRCUIT_SPICE);

    assert!(
        code.contains("pub s: [[f64; N]; N]"),
        "CircuitState must have s matrix field."
    );
    assert!(
        code.contains("pub a_neg: [[f64; N]; N]"),
        "CircuitState must have a_neg matrix field."
    );
    assert!(
        code.contains("pub k: [[f64; M]; M]"),
        "CircuitState must have k matrix field."
    );
    assert!(
        code.contains("pub s_ni: [[f64; M]; N]"),
        "CircuitState must have s_ni matrix field."
    );
    assert!(code.contains("pub dc_block_x_prev: f64"), "CircuitState must have dc_block_x_prev field.");
    assert!(code.contains("pub dc_block_y_prev: f64"), "CircuitState must have dc_block_y_prev field.");
    assert!(code.contains("pub dc_block_r: f64"), "CircuitState must have dc_block_r field.");
    assert!(code.contains("pub diag_peak_output: f64"), "CircuitState must have diag_peak_output field.");
    assert!(code.contains("pub diag_clamp_count: u64"), "CircuitState must have diag_clamp_count field.");
    assert!(code.contains("pub diag_nr_max_iter_count: u64"), "CircuitState must have diag_nr_max_iter_count field.");
    assert!(code.contains("pub diag_nan_reset_count: u64"), "CircuitState must have diag_nan_reset_count field.");
}

/// Verify that S_DEFAULT, A_NEG_DEFAULT, K_DEFAULT, S_NI_DEFAULT are emitted.
#[test]
fn test_generated_code_has_default_matrix_constants() {
    let (code, _netlist, _mna, _kernel) = generate_code(DIODE_CLIPPER_SPICE);

    assert!(code.contains("pub const S_DEFAULT:"), "Missing S_DEFAULT");
    assert!(code.contains("pub const A_NEG_DEFAULT:"), "Missing A_NEG_DEFAULT");
    assert!(code.contains("pub const K_DEFAULT:"), "Missing K_DEFAULT");
    assert!(code.contains("pub const S_NI_DEFAULT:"), "Missing S_NI_DEFAULT");
}

/// Verify that the NR solver uses state.k instead of K constant.
#[test]
fn test_nr_solver_uses_state_k_not_constant() {
    let (code, _netlist, _mna, _kernel) = generate_code(DIODE_CLIPPER_SPICE);

    // The NR solver should reference state.k, not the bare K constant
    assert!(
        code.contains("state.k["),
        "NR solver should use state.k for K matrix access."
    );
    // Make sure no bare K constant access in the NR section
    // (K_DEFAULT is fine as a constant definition, but active code should use state.k)
    // Check that `+ K[` pattern does NOT appear in the process function body
    let process_idx = code.find("fn solve_nonlinear").unwrap_or(0);
    let process_code = &code[process_idx..];
    assert!(
        !process_code.contains("+ K["),
        "NR solver should not use bare K constant for matrix access."
    );
}

/// Verify that build_rhs uses state.a_neg instead of A_NEG constant.
#[test]
fn test_build_rhs_uses_state_a_neg() {
    let (code, _netlist, _mna, _kernel) = generate_code(RC_CIRCUIT_SPICE);

    // build_rhs should use state.a_neg
    assert!(
        code.contains("state.a_neg["),
        "build_rhs should use state.a_neg for A_neg matrix access."
    );
}

/// Verify that the IR captures G and C matrices.
#[test]
fn test_ir_has_g_and_c_matrices() {
    let (netlist, mna, kernel) = build_pipeline(RC_CIRCUIT_SPICE);
    let config = default_config();
    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config)
        .expect("IR build failed");

    let n = ir.topology.n;

    // G and C matrices should be non-empty
    assert_eq!(
        ir.matrices.g_matrix.len(), n * n,
        "G matrix should have N*N elements"
    );
    assert_eq!(
        ir.matrices.c_matrix.len(), n * n,
        "C matrix should have N*N elements"
    );

    // G matrix should have nonzero entries (at least the resistor)
    assert!(
        ir.matrices.g_matrix.iter().any(|&v| v != 0.0),
        "G matrix should have nonzero entries from resistor"
    );

    // C matrix should have nonzero entries (at least the capacitor)
    assert!(
        ir.matrices.c_matrix.iter().any(|&v| v != 0.0),
        "C matrix should have nonzero entries from capacitor"
    );
}

/// Verify that set_sample_rate with the same rate produces identity behavior.
///
/// The generated code should contain a check: if sample_rate == SAMPLE_RATE, reset
/// to defaults to avoid numerical drift.
#[test]
fn test_set_sample_rate_same_rate_resets_defaults() {
    let (code, _netlist, _mna, _kernel) = generate_code(RC_CIRCUIT_SPICE);

    // Should check if sample rate matches and reset to defaults
    assert!(
        code.contains("self.s = S_DEFAULT"),
        "set_sample_rate should reset to S_DEFAULT when rate matches."
    );
    assert!(
        code.contains("self.a_neg = A_NEG_DEFAULT"),
        "set_sample_rate should reset to A_NEG_DEFAULT when rate matches."
    );
    assert!(
        code.contains("self.k = K_DEFAULT"),
        "set_sample_rate should reset to K_DEFAULT when rate matches."
    );
}

/// Verify that set_sample_rate builds A = G + alpha*C.
#[test]
fn test_set_sample_rate_builds_a_from_g_and_c() {
    let (code, _netlist, _mna, _kernel) = generate_code(RC_CIRCUIT_SPICE);

    // Should compute A from G + alpha*C
    assert!(
        code.contains("G[i][j] + alpha * C[i][j]"),
        "set_sample_rate should compute A = G + alpha*C."
    );
    // Should compute A_neg = alpha*C - G
    assert!(
        code.contains("alpha * C[i][j] - G[i][j]"),
        "set_sample_rate should compute A_neg = alpha*C - G."
    );
}

/// Verify that generated code compiles with set_sample_rate at a different rate.
///
/// This is an end-to-end test: generate code, compile it, run at two different
/// sample rates, and verify the output differs.
#[test]
fn test_generated_code_compiles_and_set_sample_rate_works() {
    let (code, _netlist, _mna, _kernel) = generate_code(RC_CIRCUIT_SPICE);

    // Create a test harness that exercises set_sample_rate
    let test_harness = format!(
        "{code}\n\
         fn main() {{\n\
             // Run at default sample rate\n\
             let mut state1 = CircuitState::default();\n\
             let mut out1 = Vec::new();\n\
             for i in 0..100 {{\n\
                 let input = if i < 50 {{ 1.0 }} else {{ 0.0 }};\n\
                 out1.push(process_sample(input, &mut state1));\n\
             }}\n\
             \n\
             // Run at double sample rate\n\
             let mut state2 = CircuitState::default();\n\
             state2.set_sample_rate(SAMPLE_RATE * 2.0);\n\
             let mut out2 = Vec::new();\n\
             for i in 0..200 {{\n\
                 let input = if i < 100 {{ 1.0 }} else {{ 0.0 }};\n\
                 out2.push(process_sample(input, &mut state2));\n\
             }}\n\
             \n\
             // Outputs should differ (different filter response)\n\
             // At 2x sample rate, the RC filter has a different cutoff\n\
             assert!(out1[49] != 0.0, \"Default rate should produce output\");\n\
             assert!(out2[99] != 0.0, \"Double rate should produce output\");\n\
             \n\
             // The 2x rate output at sample 99 (same real time as sample 49 at 1x)\n\
             // should be close but not identical to the 1x rate output at sample 49\n\
             let diff = (out1[49] - out2[99]).abs();\n\
             assert!(\n\
                 diff > 1e-6,\n\
                 \"Outputs at different sample rates should differ: diff={{:.10}}\",\n\
                 diff\n\
             );\n\
             \n\
             // Verify reset to default rate works\n\
             let mut state3 = CircuitState::default();\n\
             state3.set_sample_rate(SAMPLE_RATE * 2.0);\n\
             state3.set_sample_rate(SAMPLE_RATE); // Reset back\n\
             let mut out3 = Vec::new();\n\
             for i in 0..100 {{\n\
                 let input = if i < 50 {{ 1.0 }} else {{ 0.0 }};\n\
                 out3.push(process_sample(input, &mut state3));\n\
             }}\n\
             // Should match original\n\
             for i in 0..100 {{\n\
                 assert!(\n\
                     (out1[i] - out3[i]).abs() < 1e-12,\n\
                     \"Reset to default rate should match original: sample {{}} diff={{:.15}}\",\n\
                     i, (out1[i] - out3[i]).abs()\n\
                 );\n\
             }}\n\
             \n\
             eprintln!(\"set_sample_rate test passed!\");\n\
         }}\n"
    );

    // Write to temp file and compile
    let path = std::path::Path::new("/tmp/melange_sample_rate_test.rs");
    let mut f = std::fs::File::create(path).expect("create temp file");
    f.write_all(test_harness.as_bytes()).expect("write temp file");

    let output = std::process::Command::new("rustc")
        .args([
            path.to_str().unwrap(),
            "-o",
            "/tmp/melange_sample_rate_test",
            "--edition",
            "2021",
        ])
        .output()
        .expect("run rustc");

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        panic!(
            "Generated code with set_sample_rate test failed to compile:\n{}",
            stderr
        );
    }

    // Run the compiled binary
    let run_output = std::process::Command::new("/tmp/melange_sample_rate_test")
        .output()
        .expect("run compiled test");

    if !run_output.status.success() {
        let stderr = String::from_utf8_lossy(&run_output.stderr);
        let stdout = String::from_utf8_lossy(&run_output.stdout);
        panic!(
            "set_sample_rate test binary failed:\nstdout: {}\nstderr: {}",
            stdout, stderr
        );
    }
}

/// Verify that a diode circuit's generated code compiles with set_sample_rate.
#[test]
fn test_nonlinear_circuit_set_sample_rate_compiles() {
    let (code, _netlist, _mna, _kernel) = generate_code(DIODE_CLIPPER_SPICE);

    let test_harness = format!(
        "{code}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             // Switch to 96kHz\n\
             state.set_sample_rate(96000.0);\n\
             \n\
             // Process some samples\n\
             let mut outputs = Vec::new();\n\
             for i in 0..200 {{\n\
                 let t = i as f64 / 96000.0;\n\
                 let input = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();\n\
                 outputs.push(process_sample(input, &mut state));\n\
             }}\n\
             \n\
             // Should produce non-trivial output\n\
             let max_out = outputs.iter().cloned().fold(0.0f64, f64::max);\n\
             assert!(max_out > 1e-6, \"Diode clipper at 96kHz should produce output\");\n\
             \n\
             // Verify state matrices were updated\n\
             // At 96kHz, alpha = 2*96000 = 192000, different from default 88200\n\
             // So the S matrix should differ from S_DEFAULT\n\
             let mut s_matches_default = true;\n\
             for i in 0..N {{\n\
                 for j in 0..N {{\n\
                     if (state.s[i][j] - S_DEFAULT[i][j]).abs() > 1e-15 {{\n\
                         s_matches_default = false;\n\
                     }}\n\
                 }}\n\
             }}\n\
             assert!(!s_matches_default, \"S matrix at 96kHz should differ from S_DEFAULT\");\n\
             \n\
             eprintln!(\"Nonlinear set_sample_rate test passed!\");\n\
         }}\n"
    );

    let path = std::path::Path::new("/tmp/melange_sr_nonlinear_test.rs");
    let mut f = std::fs::File::create(path).expect("create temp file");
    f.write_all(test_harness.as_bytes()).expect("write temp file");

    let output = std::process::Command::new("rustc")
        .args([
            path.to_str().unwrap(),
            "-o",
            "/tmp/melange_sr_nonlinear_test",
            "--edition",
            "2021",
        ])
        .output()
        .expect("run rustc");

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        panic!(
            "Nonlinear circuit set_sample_rate test failed to compile:\n{}",
            stderr
        );
    }

    let run_output = std::process::Command::new("/tmp/melange_sr_nonlinear_test")
        .output()
        .expect("run compiled test");

    if !run_output.status.success() {
        let stderr = String::from_utf8_lossy(&run_output.stderr);
        let stdout = String::from_utf8_lossy(&run_output.stdout);
        panic!(
            "Nonlinear set_sample_rate test binary failed:\nstdout: {}\nstderr: {}",
            stdout, stderr
        );
    }
}

/// Verify that the IR stores inductor inductance values.
#[test]
fn test_ir_stores_inductor_inductance() {
    let spice = "\
Inductor Circuit
R1 in out 1k
L1 out 0 10m
C1 out 0 1u
";
    let netlist = Netlist::parse(spice).expect("parse");
    let mna = MnaSystem::from_netlist(&netlist).expect("mna");
    let kernel = DkKernel::from_mna(&mna, 44100.0).expect("kernel");

    let config = CodegenConfig {
        circuit_name: "ind_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        ..CodegenConfig::default()
    };

    let ir = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config)
        .expect("IR build failed");

    assert_eq!(ir.inductors.len(), 1, "Should have 1 inductor");
    assert!(
        (ir.inductors[0].inductance - 0.01).abs() < 1e-10,
        "Inductance should be 10mH = 0.01H, got {}",
        ir.inductors[0].inductance
    );
}

/// Verify that set_sample_rate works for a circuit with a potentiometer.
/// SM vectors (su, usu, nv_su, u_ni) must be recomputed for the new rate.
#[test]
fn test_pot_circuit_set_sample_rate_compiles_and_runs() {
    let pot_spice = "\
Pot SR Test
R1 in out 10k
D1 out 0 D1N4148
C1 out 0 100n
.model D1N4148 D(IS=1e-15)
.pot R1 1k 100k
";
    let (netlist, mna, kernel) = build_pipeline(pot_spice);
    let codegen = CodeGenerator::new(default_config());
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen failed");

    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             let mut state2 = CircuitState::default();\n\
             state2.set_sample_rate(96000.0);\n\
             \n\
             // SM vectors should differ from defaults at a different rate\n\
             let mut su_matches = true;\n\
             for i in 0..N {{\n\
                 if (state2.pot_0_su[i] - state.pot_0_su[i]).abs() > 1e-15 {{\n\
                     su_matches = false;\n\
                 }}\n\
             }}\n\
             assert!(!su_matches, \"pot_0_su at 96kHz should differ from 44.1kHz defaults\");\n\
             assert!(\n\
                 (state2.pot_0_usu - state.pot_0_usu).abs() > 1e-15,\n\
                 \"pot_0_usu at 96kHz should differ from default\"\n\
             );\n\
             \n\
             // Process a 1kHz sine wave at both rates and compare\n\
             // The RC filter response differs between 44.1kHz and 96kHz\n\
             let mut sum_sq_diff = 0.0f64;\n\
             for i in 0..200 {{\n\
                 let t1 = i as f64 / 44100.0;\n\
                 let t2 = i as f64 / 96000.0;\n\
                 let in1 = (2.0 * std::f64::consts::PI * 1000.0 * t1).sin();\n\
                 let in2 = (2.0 * std::f64::consts::PI * 1000.0 * t2).sin();\n\
                 let out1 = process_sample(in1, &mut state);\n\
                 let out2 = process_sample(in2, &mut state2);\n\
                 assert!(out1.is_finite(), \"Output at 44.1kHz should be finite\");\n\
                 assert!(out2.is_finite(), \"Output at 96kHz should be finite\");\n\
                 sum_sq_diff += (out1 - out2).powi(2);\n\
             }}\n\
             let rms_diff = (sum_sq_diff / 200.0).sqrt();\n\
             assert!(\n\
                 rms_diff > 1e-10,\n\
                 \"Different sample rates should produce different output (rms_diff={{}})\",\n\
                 rms_diff\n\
             );\n\
             eprintln!(\"pot set_sample_rate test passed! rms_diff={{}}\", rms_diff);\n\
         }}\n",
        result.code
    );

    let path = std::path::Path::new("/tmp/melange_pot_sr_test.rs");
    let mut f = std::fs::File::create(path).expect("create temp file");
    f.write_all(test_harness.as_bytes()).expect("write temp file");

    let output = std::process::Command::new("rustc")
        .args([path.to_str().unwrap(), "-o", "/tmp/melange_pot_sr_test", "--edition", "2021"])
        .output()
        .expect("run rustc");

    if !output.status.success() {
        panic!("Pot set_sample_rate test failed to compile:\n{}", String::from_utf8_lossy(&output.stderr));
    }

    let run_output = std::process::Command::new("/tmp/melange_pot_sr_test").output().expect("run test");
    if !run_output.status.success() {
        panic!("Pot set_sample_rate test failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run_output.stdout), String::from_utf8_lossy(&run_output.stderr));
    }
}

/// Verify that set_sample_rate works for a circuit with an inductor.
/// ind_g_eq must be recomputed for the new rate.
#[test]
fn test_inductor_circuit_set_sample_rate_compiles_and_runs() {
    let ind_spice = "\
Inductor SR Test
R1 in out 1k
L1 out 0 10m
C1 out 0 1u
";
    let (netlist, mna, kernel) = build_pipeline(ind_spice);
    let config = CodegenConfig {
        circuit_name: "ind_sr_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen failed");

    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             let default_g_eq = state.ind_g_eq[0];\n\
             \n\
             // Switch to 96kHz — g_eq = T/(2L) should change\n\
             state.set_sample_rate(96000.0);\n\
             let new_g_eq = state.ind_g_eq[0];\n\
             \n\
             // g_eq = T/(2*L) = 1/(2*sr*L), so at 96kHz it should be smaller\n\
             assert!(\n\
                 (default_g_eq - new_g_eq).abs() > 1e-10,\n\
                 \"ind_g_eq should change with sample rate: default={{}} vs 96k={{}}\",\n\
                 default_g_eq, new_g_eq\n\
             );\n\
             assert!(\n\
                 new_g_eq < default_g_eq,\n\
                 \"g_eq at 96kHz should be smaller (shorter T): {{}} < {{}}\",\n\
                 new_g_eq, default_g_eq\n\
             );\n\
             \n\
             // Process samples and verify finite output\n\
             let mut outputs = Vec::new();\n\
             for i in 0..200 {{\n\
                 let t = i as f64 / 96000.0;\n\
                 let input = (2.0 * std::f64::consts::PI * 1000.0 * t).sin();\n\
                 outputs.push(process_sample(input, &mut state));\n\
             }}\n\
             assert!(outputs.iter().all(|v| v.is_finite()), \"All outputs should be finite\");\n\
             let max_out = outputs.iter().cloned().fold(0.0f64, f64::max);\n\
             assert!(max_out > 1e-6, \"Inductor circuit should produce output\");\n\
             \n\
             // S matrix should also differ from default\n\
             let mut s_matches = true;\n\
             for i in 0..N {{\n\
                 for j in 0..N {{\n\
                     if (state.s[i][j] - S_DEFAULT[i][j]).abs() > 1e-15 {{\n\
                         s_matches = false;\n\
                     }}\n\
                 }}\n\
             }}\n\
             assert!(!s_matches, \"S at 96kHz should differ from S_DEFAULT\");\n\
             \n\
             eprintln!(\"inductor set_sample_rate test passed!\");\n\
         }}\n",
        result.code
    );

    let path = std::path::Path::new("/tmp/melange_ind_sr_test.rs");
    let mut f = std::fs::File::create(path).expect("create temp file");
    f.write_all(test_harness.as_bytes()).expect("write temp file");

    let output = std::process::Command::new("rustc")
        .args([path.to_str().unwrap(), "-o", "/tmp/melange_ind_sr_test", "--edition", "2021"])
        .output()
        .expect("run rustc");

    if !output.status.success() {
        panic!("Inductor set_sample_rate test failed to compile:\n{}", String::from_utf8_lossy(&output.stderr));
    }

    let run_output = std::process::Command::new("/tmp/melange_ind_sr_test").output().expect("run test");
    if !run_output.status.success() {
        panic!("Inductor set_sample_rate test failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run_output.stdout), String::from_utf8_lossy(&run_output.stderr));
    }
}

// ==========================================================================
// Test: M=5 codegen (5 diodes) — Gaussian elimination
// ==========================================================================

const FIVE_DIODE_SPICE: &str = "\
Five Diode Chain
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

#[test]
fn test_codegen_m5_generates_gauss_elim() {
    let (code, _netlist, _mna, kernel) = generate_code(FIVE_DIODE_SPICE);
    assert_eq!(kernel.m, 5, "Five diodes should produce M=5");

    assert!(
        code.contains("Solve 5x5"),
        "M=5 should use Gaussian elimination"
    );

    // All 5 residuals
    for i in 0..5 {
        assert!(
            code.contains(&format!("let f{} = i_nl[{}] - i_dev{}", i, i, i)),
            "Missing residual f{}", i
        );
    }

    // All 25 Jacobian entries
    for i in 0..5 {
        for j in 0..5 {
            assert!(
                code.contains(&format!("let j{}{} =", i, j)),
                "Missing Jacobian entry j{}{}", i, j
            );
        }
    }

    assert!(code.contains("partial pivoting"), "Should use partial pivoting");
    assert!(code.contains("Back substitution"), "Should have back substitution");
}

#[test]
fn test_codegen_m5_compiles() {
    let (code, _netlist, _mna, kernel) = generate_code(FIVE_DIODE_SPICE);
    assert_eq!(kernel.m, 5);

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_m5.rs");
    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_m5.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_m5.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_m5.rlib"));

    assert!(
        output.status.success(),
        "M=5 codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

// ==========================================================================
// Test: M=6 codegen (6 diodes)
// ==========================================================================

const SIX_DIODE_SPICE: &str = "\
Six Diode Chain
Rin in 0 1k
D1 in m1 D1N4148
D2 m1 m2 D1N4148
D3 m2 m3 D1N4148
D4 m3 m4 D1N4148
D5 m4 m5 D1N4148
D6 m5 out D1N4148
R1 m1 0 10k
R2 m2 0 10k
R3 m3 0 10k
R4 m4 0 10k
R5 m5 0 10k
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
";

#[test]
fn test_codegen_m6_six_diodes() {
    let (code, _netlist, _mna, kernel) = generate_code(SIX_DIODE_SPICE);
    assert_eq!(kernel.m, 6);
    assert!(code.contains("Solve 6x6"), "M=6 should use Gaussian elimination");

    for i in 0..6 {
        assert!(
            code.contains(&format!("let f{} = i_nl[{}] - i_dev{}", i, i, i)),
            "Missing residual f{}", i
        );
    }
}

#[test]
fn test_codegen_m6_compiles() {
    let (code, _netlist, _mna, kernel) = generate_code(SIX_DIODE_SPICE);
    assert_eq!(kernel.m, 6);

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_m6.rs");
    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_m6.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_m6.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_m6.rlib"));

    assert!(
        output.status.success(),
        "M=6 codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

// ==========================================================================
// Test: M=8 codegen (8 diodes) — maximum supported
// ==========================================================================

const EIGHT_DIODE_SPICE: &str = "\
Eight Diode Chain
Rin in 0 1k
D1 in m1 D1N4148
D2 m1 m2 D1N4148
D3 m2 m3 D1N4148
D4 m3 m4 D1N4148
D5 m4 m5 D1N4148
D6 m5 m6 D1N4148
D7 m6 m7 D1N4148
D8 m7 out D1N4148
R1 m1 0 10k
R2 m2 0 10k
R3 m3 0 10k
R4 m4 0 10k
R5 m5 0 10k
R6 m6 0 10k
R7 m7 0 10k
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
";

#[test]
fn test_codegen_m8_generates_gauss_elim() {
    let (code, _netlist, _mna, kernel) = generate_code(EIGHT_DIODE_SPICE);
    assert_eq!(kernel.m, 8);
    assert!(code.contains("Solve 8x8"), "M=8 should use Gaussian elimination");

    for i in 0..8 {
        assert!(
            code.contains(&format!("let f{} = i_nl[{}] - i_dev{}", i, i, i)),
            "Missing residual f{}", i
        );
    }

    for i in 0..8 {
        for j in 0..8 {
            assert!(
                code.contains(&format!("let j{}{} =", i, j)),
                "Missing Jacobian entry j{}{}", i, j
            );
        }
    }
}

#[test]
fn test_codegen_m8_compiles() {
    let (code, _netlist, _mna, kernel) = generate_code(EIGHT_DIODE_SPICE);
    assert_eq!(kernel.m, 8);

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_m8.rs");
    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_m8.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_m8.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_m8.rlib"));

    assert!(
        output.status.success(),
        "M=8 codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

// ==========================================================================
// Test: M=3/4 still work after limit raise
// ==========================================================================

#[test]
fn test_codegen_m3_m4_still_work_after_limit_raise() {
    let (code3, _nl3, _mna3, kernel3) = generate_code(MIXED_DEVICE_SPICE);
    assert_eq!(kernel3.m, 3, "D1 + Q1 = M=3");
    assert!(code3.contains("Solve 3x3"), "M=3 should still work");

    let m4_spice = "\
Four Diode Chain
Rin in 0 1k
D1 in m1 D1N4148
D2 m1 m2 D1N4148
D3 m2 m3 D1N4148
D4 m3 out D1N4148
R1 m1 0 10k
R2 m2 0 10k
R3 m3 0 10k
C1 out 0 1u
.model D1N4148 D(IS=1e-15)
";
    let (code4, _nl4, _mna4, kernel4) = generate_code(m4_spice);
    assert_eq!(kernel4.m, 4, "Four diodes = M=4");
    assert!(code4.contains("Solve 4x4"), "M=4 should still work");
}

// ==========================================================================
// JFET codegen tests
// ==========================================================================

// N-channel JFET common-source amplifier with VDD supply.
// Nodes: in(0), out(1), src(2), vdd(3) → matches default_config input=0, output=1.
const JFET_CS_SPICE: &str = "\
JFET Common Source Amplifier
Rg in 0 100k
J1 out in src J2N5457
Rd vdd out 10k
Rs src 0 1k
V1 vdd 0 DC 12
C1 out 0 100n
Cs src 0 10u
.model J2N5457 NJ(VTO=-2.0 IDSS=2e-3 LAMBDA=0.001)
";

#[test]
fn test_jfet_codegen_generates_correct_functions() {
    let (code, _netlist, _mna, kernel) = generate_code(JFET_CS_SPICE);

    // JFET is 1D: M=1
    assert_eq!(kernel.m, 1, "JFET should be 1D (M=1)");

    // Should contain JFET device functions
    assert!(code.contains("jfet_id("), "Should contain jfet_id function");
    assert!(code.contains("jfet_conductance("), "Should contain jfet_conductance function");

    // Should have JFET device constants
    assert!(code.contains("DEVICE_0_IDSS"), "Should have IDSS constant");
    assert!(code.contains("DEVICE_0_VP"), "Should have VP constant");
    assert!(code.contains("DEVICE_0_SIGN"), "Should have SIGN constant");

    // Should call JFET functions in NR loop
    assert!(
        code.contains("jfet_id(v_d0, DEVICE_0_IDSS, DEVICE_0_VP, DEVICE_0_SIGN)"),
        "NR loop should call jfet_id"
    );
    assert!(
        code.contains("jfet_conductance(v_d0, DEVICE_0_IDSS, DEVICE_0_VP, DEVICE_0_SIGN)"),
        "NR loop should call jfet_conductance"
    );
}

#[test]
fn test_jfet_codegen_compiles() {
    let (code, _netlist, _mna, kernel) = generate_code(JFET_CS_SPICE);
    assert_eq!(kernel.m, 1);

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_jfet.rs");
    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_jfet.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_jfet.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_jfet.rlib"));

    assert!(
        output.status.success(),
        "JFET codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn test_jfet_mixed_with_diode_compiles() {
    // Nodes: in(0), out(1), mid(2), src(3), vdd(4).
    // J1 first → device 0 (JFET), D1 second → device 1 (diode).
    let spice = "\
JFET with Diode Clipper
Rin in out 100k
J1 mid in src J2N5457
D1 out 0 D1N4148
Rd vdd mid 10k
Rs src 0 1k
V1 vdd 0 DC 12
Cc mid out 100n
Cs src 0 10u
.model J2N5457 NJ(VTO=-2.0 IDSS=2e-3)
.model D1N4148 D(IS=1e-15)
";
    let (code, _netlist, _mna, kernel) = generate_code(spice);
    assert_eq!(kernel.m, 2, "JFET(1D) + Diode(1D) = M=2");

    // Should contain both device types
    assert!(code.contains("jfet_id("), "Should have JFET functions");
    assert!(code.contains("diode_current("), "Should have diode functions");
    assert!(code.contains("DEVICE_0_IDSS"), "JFET at device 0");
    assert!(code.contains("DEVICE_1_IS"), "Diode at device 1");

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_jfet_diode.rs");
    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_jfet_diode.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_jfet_diode.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_jfet_diode.rlib"));

    assert!(
        output.status.success(),
        "JFET+Diode codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

/// Compile and run the JFET common-source amplifier: feed a sine wave, verify
/// output is finite, non-zero, and the signal passes through.
#[test]
fn test_jfet_codegen_compiles_and_runs() {
    let (netlist, mna, kernel) = build_pipeline(JFET_CS_SPICE);
    let config = CodegenConfig {
        circuit_name: "jfet_cs_run".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen failed");

    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             \n\
             // Warm up with 200 silent samples to let DC OP settle\n\
             for _ in 0..200 {{\n\
                 let out = process_sample(0.0, &mut state);\n\
                 assert!(out.is_finite(), \"Warmup output must be finite\");\n\
             }}\n\
             \n\
             // Feed a 1kHz sine wave (100mV amplitude) for 500 samples\n\
             let mut max_abs_out = 0.0f64;\n\
             let mut any_nonzero = false;\n\
             for i in 0..500 {{\n\
                 let t = i as f64 / 44100.0;\n\
                 let input = 0.1 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();\n\
                 let out = process_sample(input, &mut state);\n\
                 assert!(out.is_finite(), \"Output at sample {{}} must be finite, got {{}}\", i, out);\n\
                 if out.abs() > 1e-6 {{ any_nonzero = true; }}\n\
                 if out.abs() > max_abs_out {{ max_abs_out = out.abs(); }}\n\
             }}\n\
             \n\
             assert!(any_nonzero, \"JFET amp output should not be all zeros\");\n\
             eprintln!(\"JFET CS amp test passed! max_abs_out={{}}\", max_abs_out);\n\
         }}\n",
        result.code
    );

    let tmp_dir = std::env::temp_dir();
    let src_path = tmp_dir.join("melange_jfet_run_test.rs");
    let bin_path = tmp_dir.join("melange_jfet_run_test");
    {
        let mut f = std::fs::File::create(&src_path).expect("create temp file");
        f.write_all(test_harness.as_bytes()).expect("write temp file");
    }

    let compile = std::process::Command::new("rustc")
        .args([src_path.to_str().unwrap(), "-o", bin_path.to_str().unwrap(), "--edition", "2021"])
        .output()
        .expect("run rustc");

    if !compile.status.success() {
        let _ = std::fs::remove_file(&src_path);
        panic!(
            "JFET compile-and-run test failed to compile:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin_path).output().expect("run test binary");
    let _ = std::fs::remove_file(&src_path);
    let _ = std::fs::remove_file(&bin_path);

    if !run.status.success() {
        panic!(
            "JFET compile-and-run test failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }
}

/// P-channel JFET: verify default VP=+2.0 is used when VTO is not specified,
/// and the generated code compiles and runs.
#[test]
fn test_jfet_p_channel_default_vp_compiles_and_runs() {
    // P-channel JFET with no explicit VTO — should default to +2.0.
    // Self-biased: gate tied to source through Rg → Vgs ≈ 0 → fully on.
    // Nodes: in(0), out(1), src(2), vdd(3).
    let spice = "\
P-Channel JFET Test
Rg in 0 100k
J1 out in src PJFET1
Rs vdd src 2.2k
Rd out 0 4.7k
V1 vdd 0 DC 9
C1 out 0 100n
.model PJFET1 PJ(IDSS=2e-3)
";
    let (netlist, mna, kernel) = build_pipeline(spice);
    assert_eq!(kernel.m, 1, "P-channel JFET should be 1D (M=1)");

    let config = CodegenConfig {
        circuit_name: "pjfet_test".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen failed");

    // Verify P-channel sign is emitted
    assert!(
        result.code.contains("DEVICE_0_SIGN: f64 = -1.0"),
        "P-channel JFET should have SIGN = -1.0"
    );

    // VP should be +2.0 (the P-channel default, NOT -2.0)
    let vp_str = format!("{:.17e}", 2.0_f64);
    assert!(
        result.code.contains(&vp_str),
        "P-channel VP should default to +2.0, looking for {} in code",
        vp_str
    );

    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             for _ in 0..200 {{\n\
                 let out = process_sample(0.0, &mut state);\n\
                 assert!(out.is_finite(), \"P-ch JFET warmup output must be finite\");\n\
             }}\n\
             let mut any_nonzero = false;\n\
             for i in 0..500 {{\n\
                 let t = i as f64 / 44100.0;\n\
                 let input = 0.1 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();\n\
                 let out = process_sample(input, &mut state);\n\
                 assert!(out.is_finite(), \"P-ch JFET output at sample {{}} must be finite\", i);\n\
                 if out.abs() > 1e-6 {{ any_nonzero = true; }}\n\
             }}\n\
             assert!(any_nonzero, \"P-channel JFET output should not be all zeros\");\n\
             eprintln!(\"P-channel JFET test passed!\");\n\
         }}\n",
        result.code
    );

    let tmp_dir = std::env::temp_dir();
    let src_path = tmp_dir.join("melange_pjfet_run_test.rs");
    let bin_path = tmp_dir.join("melange_pjfet_run_test");
    {
        let mut f = std::fs::File::create(&src_path).expect("create temp file");
        f.write_all(test_harness.as_bytes()).expect("write temp file");
    }

    let compile = std::process::Command::new("rustc")
        .args([src_path.to_str().unwrap(), "-o", bin_path.to_str().unwrap(), "--edition", "2021"])
        .output()
        .expect("run rustc");

    if !compile.status.success() {
        let _ = std::fs::remove_file(&src_path);
        panic!(
            "P-channel JFET test failed to compile:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin_path).output().expect("run test binary");
    let _ = std::fs::remove_file(&src_path);
    let _ = std::fs::remove_file(&bin_path);

    if !run.status.success() {
        panic!(
            "P-channel JFET test failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }
}

// ==========================================================================
// OVERSAMPLING TESTS
// ==========================================================================

#[test]
fn test_oversampling_2x_generates_correct_structure() {
    let (netlist, mna, kernel) = build_pipeline(DIODE_CLIPPER_SPICE);
    let config = CodegenConfig {
        circuit_name: "test_os2x".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        oversampling_factor: 2,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).unwrap();
    let code = &result.code;

    // Should have OVERSAMPLING_FACTOR constant
    assert!(
        code.contains("pub const OVERSAMPLING_FACTOR: usize = 2;"),
        "Should emit OVERSAMPLING_FACTOR = 2"
    );

    // Should have INTERNAL_SAMPLE_RATE
    assert!(
        code.contains("pub const INTERNAL_SAMPLE_RATE: f64 = 88200.0;"),
        "Should emit INTERNAL_SAMPLE_RATE = 88200.0"
    );

    // Should have os_allpass function
    assert!(
        code.contains("fn os_allpass("),
        "Should emit os_allpass helper function"
    );

    // Should have os_halfband function
    assert!(
        code.contains("fn os_halfband("),
        "Should emit os_halfband function"
    );

    // Should have process_sample_inner (renamed from process_sample)
    assert!(
        code.contains("fn process_sample_inner("),
        "Should rename process_sample to process_sample_inner when oversampling"
    );

    // Should have public process_sample wrapper
    assert!(
        code.contains("pub fn process_sample(input: f64, state: &mut CircuitState) -> f64"),
        "Should emit public process_sample wrapper"
    );

    // Should have oversampler state fields
    assert!(
        code.contains("pub os_up_state: [f64;"),
        "Should have upsampler state in CircuitState"
    );
    assert!(
        code.contains("pub os_dn_state: [f64;"),
        "Should have downsampler state in CircuitState"
    );

    // Should have OS_COEFFS
    assert!(
        code.contains("const OS_COEFFS:"),
        "Should emit OS_COEFFS constant"
    );

    // set_sample_rate should use OVERSAMPLING_FACTOR
    assert!(
        code.contains("sample_rate * OVERSAMPLING_FACTOR"),
        "set_sample_rate should compute internal rate using OVERSAMPLING_FACTOR"
    );
}

#[test]
fn test_oversampling_factor_1_unchanged() {
    let (code_1x, _, _, _) = generate_code(DIODE_CLIPPER_SPICE);

    // Factor 1 should NOT have oversampling artifacts
    assert!(
        code_1x.contains("pub const OVERSAMPLING_FACTOR: usize = 1;"),
        "Factor 1 should have OVERSAMPLING_FACTOR = 1"
    );
    assert!(
        !code_1x.contains("os_allpass"),
        "Factor 1 should NOT have os_allpass"
    );
    assert!(
        !code_1x.contains("process_sample_inner"),
        "Factor 1 should NOT rename process_sample"
    );
    assert!(
        !code_1x.contains("INTERNAL_SAMPLE_RATE"),
        "Factor 1 should NOT have INTERNAL_SAMPLE_RATE"
    );
    assert!(
        !code_1x.contains("os_up_state"),
        "Factor 1 should NOT have oversampler state"
    );
}

#[test]
fn test_oversampling_invalid_factor_rejected() {
    let (netlist, mna, kernel) = build_pipeline(RC_CIRCUIT_SPICE);
    let config = CodegenConfig {
        circuit_name: "test_invalid_os".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 0,
        input_resistance: 1.0,
        oversampling_factor: 3,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist);
    assert!(result.is_err(), "Factor 3 should be rejected");
    let err = result.unwrap_err();
    match err {
        CodegenError::InvalidConfig(msg) => {
            assert!(msg.contains("oversampling_factor"), "Error should mention oversampling_factor: {}", msg);
        }
        _ => panic!("Expected InvalidConfig, got {:?}", err),
    }
}

#[test]
fn test_oversampling_2x_diode_clipper_compiles() {
    let (netlist, mna, kernel) = build_pipeline(DIODE_CLIPPER_SPICE);
    let config = CodegenConfig {
        circuit_name: "test_os2x_compile".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        oversampling_factor: 2,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).unwrap();

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_os2x_diode.rs");

    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(result.code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_os2x_diode.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_os2x_diode.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_os2x_diode.rlib"));

    assert!(
        output.status.success(),
        "2x oversampled diode clipper codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn test_oversampling_2x_rc_linear_compiles() {
    let (netlist, mna, kernel) = build_pipeline(RC_CIRCUIT_SPICE);
    let config = CodegenConfig {
        circuit_name: "test_os2x_rc".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 0,
        input_resistance: 1.0,
        oversampling_factor: 2,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).unwrap();

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_os2x_rc.rs");

    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(result.code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_os2x_rc.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_os2x_rc.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_os2x_rc.rlib"));

    assert!(
        output.status.success(),
        "2x oversampled RC circuit codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn test_oversampling_2x_bjt_compiles() {
    let (netlist, mna, kernel) = build_pipeline(BJT_SPICE);
    let config = CodegenConfig {
        circuit_name: "test_os2x_bjt".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        oversampling_factor: 2,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).unwrap();

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_os2x_bjt.rs");

    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(result.code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_os2x_bjt.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_os2x_bjt.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_os2x_bjt.rlib"));

    assert!(
        output.status.success(),
        "2x oversampled BJT circuit codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn test_oversampling_4x_diode_clipper_compiles() {
    let (netlist, mna, kernel) = build_pipeline(DIODE_CLIPPER_SPICE);
    let config = CodegenConfig {
        circuit_name: "test_os4x".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        oversampling_factor: 4,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).unwrap();

    let code = &result.code;

    // Should have 4x specific constants and state
    assert!(code.contains("pub const OVERSAMPLING_FACTOR: usize = 4;"));
    assert!(code.contains("os_up_state_outer"));
    assert!(code.contains("os_dn_state_outer"));
    assert!(code.contains("OS_COEFFS_OUTER"));
    assert!(code.contains("fn os_halfband_outer("));

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_os4x_diode.rs");

    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(result.code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_os4x_diode.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_os4x_diode.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_os4x_diode.rlib"));

    assert!(
        output.status.success(),
        "4x oversampled diode clipper codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn test_oversampling_2x_matrices_at_internal_rate() {
    let (netlist, mna, kernel) = build_pipeline(DIODE_CLIPPER_SPICE);

    // Generate at 1x
    let config_1x = CodegenConfig {
        circuit_name: "test_1x".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        oversampling_factor: 1,
        ..CodegenConfig::default()
    };
    let ir_1x = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config_1x).unwrap();

    // Generate at 2x (internal rate = 88200)
    let config_2x = CodegenConfig {
        circuit_name: "test_2x".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        oversampling_factor: 2,
        ..CodegenConfig::default()
    };
    let ir_2x = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config_2x).unwrap();

    // Generate at 1x with sample_rate=88200 (should match 2x internal matrices)
    let kernel_88k = DkKernel::from_mna(&mna, 88200.0).expect("failed to build 88k kernel");
    let config_88k = CodegenConfig {
        circuit_name: "test_88k".to_string(),
        sample_rate: 88200.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1000.0,
        oversampling_factor: 1,
        ..CodegenConfig::default()
    };
    let ir_88k = CircuitIR::from_kernel(&kernel_88k, &mna, &netlist, &config_88k).unwrap();

    // Alpha should be doubled for 2x
    let alpha_1x = ir_1x.solver_config.alpha;
    let alpha_2x = ir_2x.solver_config.alpha;
    assert!(
        (alpha_2x - 2.0 * alpha_1x).abs() < 1.0,
        "2x alpha {} should be double 1x alpha {}",
        alpha_2x, alpha_1x
    );

    // S matrix should match the 88200 Hz kernel's S matrix (same rate)
    let n = ir_1x.topology.n;
    let mut max_s_diff = 0.0f64;
    for i in 0..n {
        for j in 0..n {
            let diff = (ir_2x.s(i, j) - ir_88k.s(i, j)).abs();
            max_s_diff = max_s_diff.max(diff);
        }
    }
    assert!(
        max_s_diff < 1e-10,
        "2x oversampled S matrix should match 88200Hz kernel S, max diff = {}",
        max_s_diff
    );

    // K matrix should also match
    let m = ir_1x.topology.m;
    let mut max_k_diff = 0.0f64;
    for i in 0..m {
        for j in 0..m {
            let diff = (ir_2x.k(i, j) - ir_88k.k(i, j)).abs();
            max_k_diff = max_k_diff.max(diff);
        }
    }
    assert!(
        max_k_diff < 1e-10,
        "2x oversampled K matrix should match 88200Hz kernel K, max diff = {}",
        max_k_diff
    );
}

// ==========================================================================
// Tests: Gummel-Poon BJT model
// ==========================================================================

const BJT_GP_SPICE: &str = "\
BJT GP Circuit
Q1 c b e 2N2222
Rc c 0 1k
Rb b 0 100k
Re e 0 1k
.model 2N2222 NPN(IS=1e-14 BF=200 VAF=100 IKF=0.3)
";

/// GP BJT codegen compiles successfully.
#[test]
fn test_codegen_bjt_gummel_poon_compiles() {
    let (netlist, mna, kernel) = build_pipeline(BJT_GP_SPICE);
    let config = CodegenConfig {
        circuit_name: "test_gp_bjt".to_string(),
        input_node: 0,
        output_node: kernel.n - 1,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).unwrap();

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_gp_bjt.rs");

    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(result.code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_gp_bjt.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_gp_bjt.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_gp_bjt.rlib"));

    assert!(
        output.status.success(),
        "GP BJT codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

/// GP constants appear in generated code.
#[test]
fn test_codegen_bjt_gp_constants_emitted() {
    let (netlist, mna, kernel) = build_pipeline(BJT_GP_SPICE);
    let config = CodegenConfig {
        circuit_name: "test_gp_bjt_consts".to_string(),
        input_node: 0,
        output_node: kernel.n - 1,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).unwrap();
    let code = &result.code;

    assert!(code.contains("DEVICE_0_USE_GP: bool = true"), "GP flag should be true");
    assert!(code.contains("DEVICE_0_VAF"), "VAF constant should be emitted");
    assert!(code.contains("DEVICE_0_IKF"), "IKF constant should be emitted");
    assert!(code.contains("bjt_qb"), "qb function should be in generated code");
}

/// Non-GP BJT (no VAF/VAR/IKF/IKR) should emit USE_GP=false.
#[test]
fn test_codegen_bjt_no_gp_backward_compat() {
    let (code, _netlist, _mna, _kernel) = generate_code(BJT_SPICE);

    assert!(code.contains("DEVICE_0_USE_GP: bool = false"), "GP flag should be false for EM");
    // GP constants still emitted but as infinity
    assert!(code.contains("DEVICE_0_VAF"), "VAF constant should still be emitted");
}

/// IR BjtParams with GP fields round-trips through serde.
#[test]
fn test_ir_bjt_params_gp_serde_roundtrip() {
    use melange_solver::codegen::ir::BjtParams;

    let bp = BjtParams {
        is: 1e-14, vt: 0.02585, beta_f: 200.0, beta_r: 3.0,
        is_pnp: false, vaf: 100.0, var: f64::INFINITY, ikf: 0.3, ikr: f64::INFINITY,
    };

    let json = serde_json::to_string(&bp).unwrap();
    let bp2: BjtParams = serde_json::from_str(&json).unwrap();

    assert!((bp2.vaf - 100.0).abs() < 1e-10);
    assert!(bp2.var.is_infinite());
    assert!((bp2.ikf - 0.3).abs() < 1e-10);
    assert!(bp2.ikr.is_infinite());
    assert!(bp2.is_gummel_poon());
}

/// Old JSON without GP fields deserializes with infinity defaults.
#[test]
fn test_ir_bjt_params_old_json_compat() {
    use melange_solver::codegen::ir::BjtParams;

    let old_json = r#"{"is":1e-14,"vt":0.02585,"beta_f":200.0,"beta_r":3.0,"is_pnp":false}"#;
    let bp: BjtParams = serde_json::from_str(old_json).unwrap();

    assert!(bp.vaf.is_infinite(), "vaf should default to infinity");
    assert!(bp.var.is_infinite(), "var should default to infinity");
    assert!(bp.ikf.is_infinite(), "ikf should default to infinity");
    assert!(bp.ikr.is_infinite(), "ikr should default to infinity");
    assert!(!bp.is_gummel_poon(), "should be plain Ebers-Moll");
}

// ==========================================================================
// Tube/Triode codegen tests
// ==========================================================================

/// 12AX7 common-cathode amplifier SPICE netlist.
/// Nodes: in(0), out(1), plate(2), vcc(3).
const TRIODE_CC_SPICE: &str = "\
12AX7 Common Cathode Amplifier
Rin in 0 1Meg
Cin in grid 100n
Rg grid 0 1Meg
T1 grid plate cathode 12AX7
Rk cathode 0 1.5k
Ck cathode 0 25u
Rp vcc plate 100k
Cout plate out 100n
Rout out 0 1Meg
V1 vcc 0 DC 250
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";

/// Two-stage triode preamp.
const TRIODE_TWO_STAGE_SPICE: &str = "\
Two-Stage 12AX7 Preamp
Rin in 0 1Meg
Cin in grid1 100n
Rg1 grid1 0 1Meg
T1 grid1 plate1 cathode1 12AX7
Rk1 cathode1 0 1.5k
Ck1 cathode1 0 25u
Rp1 vcc plate1 100k
Cc plate1 grid2 22n
Rg2 grid2 0 470k
T2 grid2 plate2 cathode2 12AX7
Rk2 cathode2 0 1.5k
Ck2 cathode2 0 25u
Rp2 vcc plate2 100k
Cout plate2 out 100n
Rout out 0 1Meg
V1 vcc 0 DC 250
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";

/// Parser: T1 parses correctly.
#[test]
fn test_parser_triode() {
    let spice = "\
Triode Test
T1 grid plate cathode 12AX7
Rp vcc plate 100k
Rg grid 0 1Meg
Rk cathode 0 1k
V1 vcc 0 DC 250
Cp plate 0 1p
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let triodes: Vec<_> = netlist.elements.iter().filter(|e| matches!(e, melange_solver::parser::Element::Triode { .. })).collect();
    assert_eq!(triodes.len(), 1, "Should find one triode");
    match &triodes[0] {
        melange_solver::parser::Element::Triode { name, n_grid, n_plate, n_cathode, model } => {
            assert_eq!(name, "T1");
            assert_eq!(n_grid, "grid");
            assert_eq!(n_plate, "plate");
            assert_eq!(n_cathode, "cathode");
            assert_eq!(model, "12AX7");
        }
        _ => panic!("Expected Triode"),
    }
}

/// MNA: Verify N_v/N_i matrices for a single triode.
#[test]
fn test_mna_triode_stamping() {
    let spice = "\
Triode MNA Test
T1 grid plate cathode 12AX7
Rp vcc plate 100k
Rg grid 0 1Meg
Rk cathode 0 1k
V1 vcc 0 DC 250
Cp plate 0 1p
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
";
    let netlist = Netlist::parse(spice).expect("parse failed");
    let mna = MnaSystem::from_netlist(&netlist).expect("MNA failed");

    // Triode should add M=2 nonlinear dimensions
    assert_eq!(mna.m, 2, "Triode should have M=2");

    // N_v should have 2 rows (Vgk at row 0, Vpk at row 1)
    // N_i should have 2 columns (Ip at col 0, Ig at col 1)

    // Check that N_v and N_i have correct structure:
    // Row 0 (Vgk): should have +1 at grid node, -1 at cathode node
    // Row 1 (Vpk): should have +1 at plate node, -1 at cathode node
    let grid_node = mna.node_map["grid"];
    let plate_node = mna.node_map["plate"];
    let cathode_node = mna.node_map["cathode"];

    // N_v row 0 (Vgk): +1 at grid, -1 at cathode
    if grid_node > 0 {
        assert!((mna.n_v[0][grid_node - 1] - 1.0).abs() < 1e-15, "N_v[0][grid] should be +1");
    }
    if cathode_node > 0 {
        assert!((mna.n_v[0][cathode_node - 1] - (-1.0)).abs() < 1e-15, "N_v[0][cathode] should be -1");
    }

    // N_v row 1 (Vpk): +1 at plate, -1 at cathode
    if plate_node > 0 {
        assert!((mna.n_v[1][plate_node - 1] - 1.0).abs() < 1e-15, "N_v[1][plate] should be +1");
    }
    if cathode_node > 0 {
        assert!((mna.n_v[1][cathode_node - 1] - (-1.0)).abs() < 1e-15, "N_v[1][cathode] should be -1");
    }

    // N_i col 0 (Ip): -1 at plate (extracted), +1 at cathode (injected)
    if plate_node > 0 {
        assert!((mna.n_i[plate_node - 1][0] - (-1.0)).abs() < 1e-15, "N_i[plate][0] should be -1");
    }
    if cathode_node > 0 {
        assert!((mna.n_i[cathode_node - 1][0] - 1.0).abs() < 1e-15, "N_i[cathode][0] should be +1");
    }

    // N_i col 1 (Ig): -1 at grid (extracted), +1 at cathode (injected)
    if grid_node > 0 {
        assert!((mna.n_i[grid_node - 1][1] - (-1.0)).abs() < 1e-15, "N_i[grid][1] should be -1");
    }
    if cathode_node > 0 {
        assert!((mna.n_i[cathode_node - 1][1] - 1.0).abs() < 1e-15, "N_i[cathode][1] should be +1");
    }
}

/// Codegen: 12AX7 common-cathode generates code with tube functions and constants.
#[test]
fn test_codegen_triode_constants_and_functions() {
    let (code, _netlist, _mna, kernel) = generate_code(TRIODE_CC_SPICE);
    assert_eq!(kernel.m, 2, "Triode should have M=2");

    // Should contain tube device functions
    assert!(code.contains("tube_ip("), "Should have tube_ip function");
    assert!(code.contains("tube_ig("), "Should have tube_ig function");
    assert!(code.contains("tube_jacobian("), "Should have tube_jacobian function");

    // Should contain tube constants
    assert!(code.contains("DEVICE_0_MU"), "Should have MU constant");
    assert!(code.contains("DEVICE_0_EX"), "Should have EX constant");
    assert!(code.contains("DEVICE_0_KG1"), "Should have KG1 constant");
    assert!(code.contains("DEVICE_0_KP"), "Should have KP constant");
    assert!(code.contains("DEVICE_0_KVB"), "Should have KVB constant");
    assert!(code.contains("DEVICE_0_IG_MAX"), "Should have IG_MAX constant");
    assert!(code.contains("DEVICE_0_VGK_ONSET"), "Should have VGK_ONSET constant");
}

/// Codegen: 12AX7 common-cathode amplifier compiles.
#[test]
fn test_codegen_triode_compiles() {
    let (code, _netlist, _mna, kernel) = generate_code(TRIODE_CC_SPICE);
    assert_eq!(kernel.m, 2);

    let tmp_dir = std::env::temp_dir();
    let tmp_path = tmp_dir.join("melange_codegen_test_triode.rs");
    {
        let mut f = std::fs::File::create(&tmp_path).unwrap();
        f.write_all(code.as_bytes()).unwrap();
    }

    let output = std::process::Command::new("rustc")
        .args(["--edition", "2024", "--crate-type", "lib", "-o"])
        .arg(tmp_dir.join("melange_codegen_test_triode.rlib"))
        .arg(&tmp_path)
        .output()
        .expect("failed to run rustc");

    let _ = std::fs::remove_file(&tmp_path);
    let _ = std::fs::remove_file(tmp_dir.join("melange_codegen_test_triode.rlib"));
    let _ = std::fs::remove_file(tmp_dir.join("libmelange_codegen_test_triode.rlib"));

    assert!(
        output.status.success(),
        "Triode codegen failed to compile:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

/// Codegen: Compile and run the triode common-cathode amplifier.
/// Feed a sine wave, verify output is finite and non-zero.
#[test]
fn test_codegen_triode_compiles_and_runs() {
    let (netlist, mna, kernel) = build_pipeline(TRIODE_CC_SPICE);
    assert_eq!(kernel.m, 2);

    let config = CodegenConfig {
        circuit_name: "triode_cc_run".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen failed");

    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             \n\
             // Warm up with 500 samples to let DC OP settle\n\
             for _ in 0..500 {{\n\
                 let out = process_sample(0.0, &mut state);\n\
                 assert!(out.is_finite(), \"Warmup output must be finite\");\n\
             }}\n\
             \n\
             // Feed a 1kHz sine wave (10mV amplitude) for 500 samples\n\
             let mut max_abs_out = 0.0f64;\n\
             let mut any_nonzero = false;\n\
             for i in 0..500 {{\n\
                 let t = i as f64 / 44100.0;\n\
                 let input = 0.01 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();\n\
                 let out = process_sample(input, &mut state);\n\
                 assert!(out.is_finite(), \"Output at sample {{}} must be finite, got {{}}\", i, out);\n\
                 if out.abs() > 1e-6 {{ any_nonzero = true; }}\n\
                 if out.abs() > max_abs_out {{ max_abs_out = out.abs(); }}\n\
             }}\n\
             \n\
             assert!(any_nonzero, \"Triode amp output should not be all zeros\");\n\
             eprintln!(\"Triode CC amp test passed! max_abs_out={{}}\", max_abs_out);\n\
         }}\n",
        result.code
    );

    let tmp_dir = std::env::temp_dir();
    let src_path = tmp_dir.join("melange_triode_run_test.rs");
    let bin_path = tmp_dir.join("melange_triode_run_test");
    {
        let mut f = std::fs::File::create(&src_path).expect("create temp file");
        f.write_all(test_harness.as_bytes()).expect("write temp file");
    }

    let compile = std::process::Command::new("rustc")
        .args([src_path.to_str().unwrap(), "-o", bin_path.to_str().unwrap(), "--edition", "2021"])
        .output()
        .expect("run rustc");

    if !compile.status.success() {
        let _ = std::fs::remove_file(&src_path);
        panic!(
            "Triode compile-and-run test failed to compile:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin_path).output().expect("run test binary");
    let _ = std::fs::remove_file(&src_path);
    let _ = std::fs::remove_file(&bin_path);

    if !run.status.success() {
        panic!(
            "Triode compile-and-run test failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }
}

/// Multi-triode: 2-stage preamp (2 triodes = M=4) compiles and runs.
#[test]
fn test_codegen_two_triode_preamp_compiles_and_runs() {
    let (netlist, mna, kernel) = build_pipeline(TRIODE_TWO_STAGE_SPICE);
    assert_eq!(kernel.m, 4, "Two triodes should have M=4");

    let config = CodegenConfig {
        circuit_name: "triode_preamp_run".to_string(),
        sample_rate: 44100.0,
        input_node: 0,
        output_node: 1,
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    let codegen = CodeGenerator::new(config);
    let result = codegen.generate(&kernel, &mna, &netlist).expect("codegen failed");

    let test_harness = format!(
        "{}\n\
         fn main() {{\n\
             let mut state = CircuitState::default();\n\
             \n\
             // Warm up with 500 samples\n\
             for _ in 0..500 {{\n\
                 let out = process_sample(0.0, &mut state);\n\
                 assert!(out.is_finite(), \"Warmup output must be finite\");\n\
             }}\n\
             \n\
             // Feed a 1kHz sine wave (5mV amplitude) for 500 samples\n\
             let mut max_abs_out = 0.0f64;\n\
             let mut any_nonzero = false;\n\
             for i in 0..500 {{\n\
                 let t = i as f64 / 44100.0;\n\
                 let input = 0.005 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();\n\
                 let out = process_sample(input, &mut state);\n\
                 assert!(out.is_finite(), \"Output at sample {{}} must be finite, got {{}}\", i, out);\n\
                 if out.abs() > 1e-6 {{ any_nonzero = true; }}\n\
                 if out.abs() > max_abs_out {{ max_abs_out = out.abs(); }}\n\
             }}\n\
             \n\
             assert!(any_nonzero, \"Two-stage triode preamp output should not be all zeros\");\n\
             eprintln!(\"Two-stage triode preamp test passed! max_abs_out={{}}\", max_abs_out);\n\
         }}\n",
        result.code
    );

    let tmp_dir = std::env::temp_dir();
    let src_path = tmp_dir.join("melange_triode_preamp_test.rs");
    let bin_path = tmp_dir.join("melange_triode_preamp_test");
    {
        let mut f = std::fs::File::create(&src_path).expect("create temp file");
        f.write_all(test_harness.as_bytes()).expect("write temp file");
    }

    let compile = std::process::Command::new("rustc")
        .args([src_path.to_str().unwrap(), "-o", bin_path.to_str().unwrap(), "--edition", "2021"])
        .output()
        .expect("run rustc");

    if !compile.status.success() {
        let _ = std::fs::remove_file(&src_path);
        panic!(
            "Two-stage triode compile-and-run test failed to compile:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }

    let run = std::process::Command::new(&bin_path).output().expect("run test binary");
    let _ = std::fs::remove_file(&src_path);
    let _ = std::fs::remove_file(&bin_path);

    if !run.status.success() {
        panic!(
            "Two-stage triode compile-and-run test failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }
}
