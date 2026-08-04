//! Mathematical verification tests for DK method implementation.
//!
//! These tests verify the formulas against David Yeh's 2009 thesis:
//! "Digital Implementation of Musical Distortion Circuits by Analysis and Simulation"
//!
//! Key formulas to verify:
//! - A = 2C/T + G (trapezoidal discretization forward matrix)
//! - A_neg = 2C/T - G (history matrix)
//! - S = A^{-1} (inverse)
//! - K = N_v * S * N_i (nonlinear kernel)
//! - v_pred = S * rhs (linear prediction)

#[cfg(test)]
mod tests {
    use crate::dk::{invert_matrix, mat_mul, mat_vec_mul, DkKernel};
    use crate::mna::MnaSystem;
    use crate::parser::Netlist;

    /// Helper: Check if two matrices are approximately equal
    fn matrices_approx_eq(a: &[Vec<f64>], b: &[Vec<f64>], eps: f64) -> bool {
        if a.len() != b.len() {
            return false;
        }
        for i in 0..a.len() {
            if a[i].len() != b[i].len() {
                return false;
            }
            for j in 0..a[i].len() {
                if (a[i][j] - b[i][j]).abs() > eps {
                    println!("Mismatch at [{},{}]: {} vs {}", i, j, a[i][j], b[i][j]);
                    return false;
                }
            }
        }
        true
    }

    /// Helper: Check if two vectors are approximately equal
    fn vectors_approx_eq(a: &[f64], b: &[f64], eps: f64) -> bool {
        if a.len() != b.len() {
            return false;
        }
        for i in 0..a.len() {
            if (a[i] - b[i]).abs() > eps {
                println!("Mismatch at [{}]: {} vs {}", i, a[i], b[i]);
                return false;
            }
        }
        true
    }

    /// Helper: Create identity matrix
    fn identity_matrix(n: usize) -> Vec<Vec<f64>> {
        let mut i = vec![vec![0.0; n]; n];
        for j in 0..n {
            i[j][j] = 1.0;
        }
        i
    }

    /// Test 1: Verify A matrix construction
    /// A = 2C/T + G (trapezoidal rule)
    #[test]
    fn test_a_matrix_construction() {
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sample_rate = 44100.0;
        let t = 1.0 / sample_rate;
        let alpha = 2.0 / t; // 2/T for trapezoidal

        // Get A from MNA system
        let a = mna.get_a_matrix(sample_rate).unwrap();

        // Manually compute A = G + (2/T)*C
        let n = mna.n;
        let mut a_expected = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in 0..n {
                a_expected[i][j] = mna.g[i][j] + alpha * mna.c[i][j];
            }
        }

        assert!(
            matrices_approx_eq(&a, &a_expected, 1e-10),
            "A matrix should equal G + (2/T)*C"
        );

        // Verify specific values for RC circuit
        // Node 0 (in): G[0,0] = 1/R = 0.001, C[0,0] = 0
        // Node 1 (out): G[1,1] = 1/R = 0.001, C[1,1] = 1uF
        // G[0,1] = G[1,0] = -1/R = -0.001
        let r = 1000.0;
        let c = 1e-6;
        let g = 1.0 / r;

        // A[0,0] = G[0,0] + (2/T)*C[0,0] = 0.001 + 0 = 0.001
        assert!((a[0][0] - g).abs() < 1e-10, "A[0,0] should be 1/R = {}", g);

        // A[1,1] = G[1,1] + (2/T)*C[1,1] = 0.001 + (2/T)*1e-6
        let expected_a11 = g + alpha * c;
        assert!(
            (a[1][1] - expected_a11).abs() < 1e-6,
            "A[1,1] should be 1/R + (2/T)*C = {}, got {}",
            expected_a11,
            a[1][1]
        );

        // A[0,1] = A[1,0] = -1/R = -0.001
        assert!((a[0][1] + g).abs() < 1e-10, "A[0,1] should be -1/R");
        assert!((a[1][0] + g).abs() < 1e-10, "A[1,0] should be -1/R");

        println!("A matrix construction test PASSED");
        println!("  A[0,0] = {}, expected = {}", a[0][0], g);
        println!("  A[1,1] = {}, expected = {}", a[1][1], expected_a11);
        println!("  2C/T contribution at [1,1] = {}", alpha * c);
    }

    /// Test 2: Verify A_neg matrix construction
    /// A_neg = 2C/T - G (history matrix for trapezoidal rule)
    #[test]
    fn test_a_neg_matrix_construction() {
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sample_rate = 44100.0;
        let t = 1.0 / sample_rate;
        let alpha = 2.0 / t;

        // Get A_neg from MNA system
        let a_neg = mna.get_a_neg_matrix(sample_rate).unwrap();

        // Manually compute A_neg = (2/T)*C - G
        let n = mna.n;
        let mut a_neg_expected = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in 0..n {
                a_neg_expected[i][j] = alpha * mna.c[i][j] - mna.g[i][j];
            }
        }

        assert!(
            matrices_approx_eq(&a_neg, &a_neg_expected, 1e-10),
            "A_neg matrix should equal (2/T)*C - G"
        );

        // Verify specific values
        let r = 1000.0;
        let c = 1e-6;
        let g = 1.0 / r;

        // A_neg[0,0] = (2/T)*C[0,0] - G[0,0] = 0 - 0.001 = -0.001
        assert!((a_neg[0][0] + g).abs() < 1e-10, "A_neg[0,0] should be -1/R");

        // A_neg[1,1] = (2/T)*C[1,1] - G[1,1] = (2/T)*1e-6 - 0.001
        let expected_a_neg_11 = alpha * c - g;
        assert!(
            (a_neg[1][1] - expected_a_neg_11).abs() < 1e-6,
            "A_neg[1,1] should be (2/T)*C - 1/R"
        );

        // A_neg[0,1] = A_neg[1,0] = -(-1/R) = +0.001 (sign flip from G off-diagonal)
        assert!((a_neg[0][1] - g).abs() < 1e-10, "A_neg[0,1] should be +1/R");
        assert!((a_neg[1][0] - g).abs() < 1e-10, "A_neg[1,0] should be +1/R");

        println!("A_neg matrix construction test PASSED");
        println!("  A_neg[0,0] = {}, expected = {}", a_neg[0][0], -g);
        println!(
            "  A_neg[1,1] = {}, expected = {}",
            a_neg[1][1], expected_a_neg_11
        );
    }

    /// Test 3: Verify matrix inversion S = A^{-1}
    /// S * A should equal identity matrix
    #[test]
    fn test_matrix_inversion_satisfies_s_a_equals_i() {
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sample_rate = 44100.0;
        let a = mna.get_a_matrix(sample_rate).unwrap();

        // Compute S = A^{-1}
        let s = invert_matrix(&a).unwrap();

        // Verify S * A = I
        let product = mat_mul(&s, &a);
        let identity = identity_matrix(mna.n);

        assert!(
            matrices_approx_eq(&product, &identity, 1e-10),
            "S * A should equal identity matrix"
        );

        // Also verify A * S = I
        let product2 = mat_mul(&a, &s);
        assert!(
            matrices_approx_eq(&product2, &identity, 1e-10),
            "A * S should also equal identity matrix"
        );

        println!("Matrix inversion test PASSED");
        println!("  S * A = I verified");
        println!("  A * S = I verified");
    }

    /// Test 4: Verify kernel computation K = N_v * S * N_i
    /// For circuits with nonlinear devices
    #[test]
    fn test_kernel_computation_k_equals_nv_s_ni() {
        let spice = r#"Diode Clipper with RC
R1 in out 1k
R2 out 0 10k
D1 out 0 1N4148
.model 1N4148 D(IS=2.52e-9 N=1.752)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        assert_eq!(mna.n, 2, "Should have 2 nodes");
        assert_eq!(mna.m, 1, "Should have 1 nonlinear device");

        let sample_rate = 44100.0;
        let dk = DkKernel::from_mna(&mna, sample_rate).unwrap();

        // Verify K dimensions (flattened: M * M elements)
        assert_eq!(dk.k.len(), mna.m * mna.m, "K should have M*M elements");

        // Manually compute K = N_v * S * N_i using 2D helper
        // No negation: N_i's injection convention makes K naturally negative
        // S is N x N, N_i is N x M, N_v is M x N
        // Convert S back to 2D for matrix multiplication helper
        let s_2d: Vec<Vec<f64>> = (0..dk.n)
            .map(|i| (0..dk.n).map(|j| dk.s(i, j)).collect())
            .collect();
        // First: temp = S * N_i (N x M)
        let temp = mat_mul(&s_2d, &mna.n_i);
        // Then: k_expected = N_v * temp (M x M)
        let k_expected: Vec<Vec<f64>> = mat_mul(&mna.n_v, &temp);

        // Convert K to 2D for comparison
        let k_2d: Vec<Vec<f64>> = (0..dk.m)
            .map(|i| (0..dk.m).map(|j| dk.k(i, j)).collect())
            .collect();

        assert!(
            matrices_approx_eq(&k_2d, &k_expected, 1e-10),
            "K should equal N_v * S * N_i"
        );

        println!("Kernel computation test PASSED");
        println!("  K = N_v * S * N_i verified");
        println!("  K[0][0] = {}", dk.k(0, 0));
    }

    /// Test 5: Verify prediction step v_pred = S * rhs
    #[test]
    fn test_prediction_step_v_pred_equals_s_rhs() {
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sample_rate = 44100.0;
        let dk = DkKernel::from_mna(&mna, sample_rate).unwrap();

        // Create a test RHS vector
        // For an RC circuit with input, rhs would include input contributions
        let rhs = vec![1.0, 0.0]; // Unit input at node 0

        // Compute prediction using DK kernel
        let mut v_pred = vec![0.0; dk.n];
        dk.predict_into(&rhs, &mut v_pred);

        // Manually compute v_pred = S * rhs
        // Convert S to 2D for mat_vec_mul helper
        let s_2d: Vec<Vec<f64>> = (0..dk.n)
            .map(|i| (0..dk.n).map(|j| dk.s(i, j)).collect())
            .collect();
        let v_pred_expected = mat_vec_mul(&s_2d, &rhs);

        assert!(
            vectors_approx_eq(&v_pred, &v_pred_expected, 1e-10),
            "v_pred should equal S * rhs"
        );

        // For an RC circuit with 1V at input node:
        // Node voltages should follow voltage divider behavior
        // v_out = S[1,0] * 1.0 (since rhs[0] = 1)
        println!("Prediction step test PASSED");
        println!(
            "  v_pred[0] = {} (should be close to 1 for input node)",
            v_pred[0]
        );
        println!("  v_pred[1] = {} (output node voltage)", v_pred[1]);
    }

    /// Test 6: Verify prediction produces finite, stable results
    /// For a valid circuit, the prediction step should give finite node voltages
    #[test]
    fn test_prediction_stability() {
        // Simple RC circuit - should give stable, finite results
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sample_rate = 44100.0;
        let dk = DkKernel::from_mna(&mna, sample_rate).unwrap();

        // Create a test RHS vector with unit input
        let rhs = vec![1.0, 0.0]; // Unit input at node 0
        let mut v_pred = vec![0.0; dk.n];
        dk.predict_into(&rhs, &mut v_pred);

        // Results should be finite (not NaN or Infinity)
        assert!(v_pred[0].is_finite(), "v_pred[0] should be finite");
        assert!(v_pred[1].is_finite(), "v_pred[1] should be finite");

        // For this circuit, the output voltage should be less than input
        // due to the RC filtering behavior
        println!("Prediction stability test PASSED");
        println!("  v_pred[0] = {} (input node)", v_pred[0]);
        println!("  v_pred[1] = {} (output node)", v_pred[1]);
        println!("  Both values are finite and numerically stable");
    }

    /// Test 7: Verify full DK reduction for a simple circuit
    /// This tests the complete pipeline
    #[test]
    fn test_full_dk_reduction_pipeline() {
        let spice = r#"Simple RC
R1 in out 10k
C1 out 0 10n
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sample_rate = 48000.0;
        let dk = DkKernel::from_mna(&mna, sample_rate).unwrap();

        // Verify all matrices have correct dimensions (flattened)
        assert_eq!(dk.s.len(), mna.n * mna.n, "S should have N*N elements");
        assert_eq!(
            dk.a_neg.len(),
            mna.n * mna.n,
            "A_neg should have N*N elements"
        );

        // For a linear circuit with no nonlinear devices, K should be empty
        assert!(dk.k.is_empty(), "K should be empty for linear circuit");
        assert_eq!(dk.m, 0, "M should be 0 for linear circuit");

        println!("Full DK reduction pipeline test PASSED");
        println!("  N = {} nodes", dk.n);
        println!("  M = {} nonlinear devices", dk.m);
        println!("  Sample rate = {} Hz", dk.sample_rate);
    }

    /// Test 8: Numerical stability test with matrix inversion
    /// Test with poorly conditioned matrix
    #[test]
    fn test_matrix_inversion_numerical_stability() {
        // A nearly singular matrix (very different resistor values)
        // This tests the Gaussian elimination with partial pivoting
        let a = vec![vec![1.0 + 1e6, -1e6], vec![-1e6, 1.0 + 1e6]];

        let s = invert_matrix(&a).unwrap();
        let product = mat_mul(&s, &a);
        let identity = identity_matrix(2);

        // Use larger epsilon for numerical stability check
        assert!(
            matrices_approx_eq(&product, &identity, 1e-6),
            "Matrix inversion should handle poorly conditioned matrices"
        );

        println!("Numerical stability test PASSED");
    }

    /// Test 9: Verify A + A_neg = 4C/T (relationship for trapezoidal)
    /// A + A_neg = (2C/T + G) + (2C/T - G) = 4C/T
    #[test]
    fn test_a_plus_a_neg_equals_4c_over_t() {
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sample_rate = 44100.0;
        let t = 1.0 / sample_rate;

        let a = mna.get_a_matrix(sample_rate).unwrap();
        let a_neg = mna.get_a_neg_matrix(sample_rate).unwrap();

        // Compute A + A_neg
        let n = mna.n;
        let mut sum = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in 0..n {
                sum[i][j] = a[i][j] + a_neg[i][j];
            }
        }

        // Expected: 4C/T
        let expected_factor = 4.0 / t;
        let mut expected = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in 0..n {
                expected[i][j] = expected_factor * mna.c[i][j];
            }
        }

        assert!(
            matrices_approx_eq(&sum, &expected, 1e-10),
            "A + A_neg should equal 4C/T"
        );

        println!("A + A_neg = 4C/T relationship test PASSED");
        println!("  4/T = {}", expected_factor);
    }

    /// Test 10: Verify A - A_neg = 2G (relationship for trapezoidal)
    /// A - A_neg = (2C/T + G) - (2C/T - G) = 2G
    #[test]
    fn test_a_minus_a_neg_equals_2g() {
        let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();

        let sample_rate = 44100.0;

        let a = mna.get_a_matrix(sample_rate).unwrap();
        let a_neg = mna.get_a_neg_matrix(sample_rate).unwrap();

        // Compute A - A_neg
        let n = mna.n;
        let mut diff = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in 0..n {
                diff[i][j] = a[i][j] - a_neg[i][j];
            }
        }

        // Expected: 2G
        let mut expected = vec![vec![0.0; n]; n];
        for i in 0..n {
            for j in 0..n {
                expected[i][j] = 2.0 * mna.g[i][j];
            }
        }

        assert!(
            matrices_approx_eq(&diff, &expected, 1e-10),
            "A - A_neg should equal 2G"
        );

        println!("A - A_neg = 2G relationship test PASSED");
    }

    // ── Multi-circuit identity tests ─────────────────────────────────
    // Extend RC-only tests to diode clipper, BJT CE, and inductor circuits

    #[test]
    fn test_a_plus_a_neg_equals_4c_over_t_diode_clipper() {
        let spice = r#"Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let sample_rate = 48000.0;
        let t = 1.0 / sample_rate;

        let a = mna.get_a_matrix(sample_rate).unwrap();
        let a_neg = mna.get_a_neg_matrix(sample_rate).unwrap();

        let n = mna.n;
        for i in 0..n {
            for j in 0..n {
                let sum = a[i][j] + a_neg[i][j];
                let expected = (4.0 / t) * mna.c[i][j];
                assert!(
                    (sum - expected).abs() < 1e-6,
                    "diode clipper: A+A_neg [{i}][{j}] = {sum}, expected 4C/T = {expected}"
                );
            }
        }
    }

    #[test]
    fn test_a_minus_a_neg_equals_2g_diode_clipper() {
        let spice = r#"Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let sample_rate = 48000.0;

        let a = mna.get_a_matrix(sample_rate).unwrap();
        let a_neg = mna.get_a_neg_matrix(sample_rate).unwrap();

        let n = mna.n;
        for i in 0..n {
            for j in 0..n {
                let diff = a[i][j] - a_neg[i][j];
                let expected = 2.0 * mna.g[i][j];
                assert!(
                    (diff - expected).abs() < 1e-10,
                    "diode clipper: A-A_neg [{i}][{j}] = {diff}, expected 2G = {expected}"
                );
            }
        }
    }

    #[test]
    fn test_a_plus_a_neg_equals_4c_over_t_bjt_ce() {
        let spice = r#"BJT Common Emitter
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYBJTCE
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYBJTCE NPN(IS=1e-14 BF=200 BR=3)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let sample_rate = 44100.0;
        let t = 1.0 / sample_rate;

        let a = mna.get_a_matrix(sample_rate).unwrap();
        let a_neg = mna.get_a_neg_matrix(sample_rate).unwrap();

        // For augmented rows (voltage source), C should be zero
        let n = a.len();
        for i in 0..n {
            for j in 0..n {
                let sum = a[i][j] + a_neg[i][j];
                // For non-augmented rows: A+A_neg = 4C/T
                // For augmented rows (VS): both G and C contributions are zero (algebraic)
                if i < mna.n && j < mna.n {
                    let expected = (4.0 / t) * mna.c[i][j];
                    assert!(
                        (sum - expected).abs() < 1e-4,
                        "BJT CE: A+A_neg [{i}][{j}] = {sum}, expected 4C/T = {expected}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_s_inverse_identity_bjt() {
        let spice = r#"BJT CE
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYBJTCE
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYBJTCE NPN(IS=1e-14 BF=200 BR=3)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let sample_rate = 44100.0;

        let a = mna.get_a_matrix(sample_rate).unwrap();
        let s = invert_matrix(&a).unwrap();

        // S * A should equal identity
        let product = mat_mul(&s, &a);
        let n = a.len();
        for i in 0..n {
            for j in 0..n {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (product[i][j] - expected).abs() < 1e-6,
                    "BJT CE: (S*A)[{i}][{j}] = {}, expected {expected}",
                    product[i][j]
                );
            }
        }
    }

    #[test]
    fn test_k_equals_nv_s_ni_bjt() {
        let spice = r#"BJT CE
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYBJTCE
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYBJTCE NPN(IS=1e-14 BF=200 BR=3)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        assert!(mna.m >= 2, "BJT should have at least M=2");

        let sample_rate = 44100.0;
        let dk = DkKernel::from_mna(&mna, sample_rate).unwrap();

        // Manually compute K = N_v * S * N_i
        let s_2d: Vec<Vec<f64>> = (0..dk.n)
            .map(|i| (0..dk.n).map(|j| dk.s(i, j)).collect())
            .collect();
        let temp = mat_mul(&s_2d, &mna.n_i);
        let k_expected = mat_mul(&mna.n_v, &temp);

        for i in 0..dk.m {
            for j in 0..dk.m {
                assert!(
                    (dk.k(i, j) - k_expected[i][j]).abs() < 1e-10,
                    "BJT CE: K[{i}][{j}] = {}, expected {}",
                    dk.k(i, j),
                    k_expected[i][j]
                );
            }
        }
    }

    #[test]
    fn test_k_diagonal_negative_jfet() {
        let spice = r#"JFET CS
Rg in 0 1Meg
J1 drain in source J2N5457
Rd vdd drain 2.2k
Rs source 0 1k
Cs source 0 100u
Vdd vdd 0 DC 12
.model J2N5457 NJ(VTO=-2.0 IDSS=5e-3 LAMBDA=0.001)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let dk = DkKernel::from_mna(&mna, 44100.0).unwrap();

        for i in 0..dk.m {
            assert!(
                dk.k(i, i) <= 0.0,
                "JFET: K[{i}][{i}] = {} should be <= 0 (negative feedback)",
                dk.k(i, i)
            );
        }
    }

    // ── Companion-path inductor time-march vs exact trapezoidal reference ──
    //
    // These tests march the DK kernel's companion-model inductor path
    // (LinearSolver-equivalent loop: rhs = rhs_const + A_neg*v_prev +
    // i_hist injections + trapezoidal input, v = S*rhs, update state)
    // against an independent per-step solve of the exact trapezoidal
    // discretization of the circuit ODEs. Any error in the stored history
    // current (F1: i_hist must be 2*i[n-1], not the single-step Norton
    // i[n] - g_eq*v[n]) shows up as a large per-sample deviation.

    /// March the kernel's companion path for `steps` samples of `input`,
    /// with G_in = 1.0 stamped at `input_idx` (0-based). Returns node
    /// voltage history (steps × n_nodes).
    fn march_companion_kernel(
        kernel: &mut DkKernel,
        input: &[f64],
        input_idx: usize,
    ) -> Vec<Vec<f64>> {
        let n = kernel.n;
        let mut v_prev = vec![0.0; n];
        let mut v = vec![0.0; n];
        let mut rhs = vec![0.0; n];
        let mut input_prev = 0.0;
        let mut out = Vec::with_capacity(input.len());
        for &u in input {
            for i in 0..n {
                let mut sum = kernel.rhs_const[i];
                for j in 0..n {
                    sum += kernel.a_neg(i, j) * v_prev[j];
                }
                rhs[i] = sum;
            }
            for ind in &kernel.inductors {
                if ind.node_i > 0 {
                    rhs[ind.node_i - 1] -= ind.i_hist;
                }
                if ind.node_j > 0 {
                    rhs[ind.node_j - 1] += ind.i_hist;
                }
            }
            for ci in &kernel.coupled_inductors {
                if ci.l1_node_i > 0 {
                    rhs[ci.l1_node_i - 1] -= ci.i1_hist;
                }
                if ci.l1_node_j > 0 {
                    rhs[ci.l1_node_j - 1] += ci.i1_hist;
                }
                if ci.l2_node_i > 0 {
                    rhs[ci.l2_node_i - 1] -= ci.i2_hist;
                }
                if ci.l2_node_j > 0 {
                    rhs[ci.l2_node_j - 1] += ci.i2_hist;
                }
            }
            for group in &kernel.transformer_groups {
                for k in 0..group.num_windings {
                    if group.winding_node_i[k] > 0 {
                        rhs[group.winding_node_i[k] - 1] -= group.i_hist[k];
                    }
                    if group.winding_node_j[k] > 0 {
                        rhs[group.winding_node_j[k] - 1] += group.i_hist[k];
                    }
                }
            }
            rhs[input_idx] += (u + input_prev) * 1.0;
            input_prev = u;
            for i in 0..n {
                let mut sum = 0.0;
                for j in 0..n {
                    sum += kernel.s(i, j) * rhs[j];
                }
                v[i] = sum;
            }
            v_prev.copy_from_slice(&v);
            kernel.update_inductors(&v);
            kernel.update_coupled_inductors(&v);
            kernel.update_transformer_groups(&v);
            out.push(v.clone());
        }
        out
    }

    fn sine_input(steps: usize, fs: f64, freq: f64, amp: f64) -> Vec<f64> {
        (0..steps)
            .map(|k| amp * (2.0 * std::f64::consts::PI * freq * k as f64 / fs).sin())
            .collect()
    }

    /// Uncoupled inductor: series R (Thevenin 1Ω + 100Ω) into L to ground.
    /// Exact trapezoidal reference:
    ///   i[n](1 + c) = i[n-1](1 - c) + (T/2L)(u[n] + u[n-1]),  c = T*R_tot/(2L)
    ///   v_out[n] = u[n] - R_tot*i[n]
    #[test]
    fn test_companion_inductor_rl_time_march_matches_exact_trapezoidal() {
        let spice = "RL Highpass\nR1 in out 100\nL1 out 0 10m\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        let in_idx = mna.node_map["in"] - 1;
        let out_idx = mna.node_map["out"] - 1;
        mna.g[in_idx][in_idx] += 1.0;

        let fs = 48000.0;
        let t = 1.0 / fs;
        let mut kernel = DkKernel::from_mna(&mna, fs).unwrap();
        let input = sine_input(512, fs, 1000.0, 1.0);
        let v_hist = march_companion_kernel(&mut kernel, &input, in_idx);

        // Exact trapezoidal reference
        let r_tot = 101.0;
        let l = 10e-3;
        let c = t * r_tot / (2.0 * l);
        let mut i = 0.0;
        let mut u_prev = 0.0;
        let mut max_err = 0.0_f64;
        for (k, &u) in input.iter().enumerate() {
            i = (i * (1.0 - c) + (t / (2.0 * l)) * (u + u_prev)) / (1.0 + c);
            u_prev = u;
            let v_out_ref = u - r_tot * i;
            let v_in_ref = u - 1.0 * i;
            max_err = max_err
                .max((v_hist[k][out_idx] - v_out_ref).abs())
                .max((v_hist[k][in_idx] - v_in_ref).abs());
        }
        assert!(
            max_err < 1e-12,
            "RL companion march deviates from exact trapezoidal reference: \
             max_err = {max_err:.3e} (i_hist formula wrong? F1: must inject -2*i[n-1])"
        );
    }

    /// Series RLC: exercises the interaction of companion-inductor history
    /// with the A_neg capacitor history.
    #[test]
    fn test_companion_inductor_rlc_time_march_matches_exact_trapezoidal() {
        let spice = "RLC Series\nR1 in a 100\nL1 a b 10m\nC1 b 0 100n\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        let in_idx = mna.node_map["in"] - 1;
        let a_idx = mna.node_map["a"] - 1;
        let b_idx = mna.node_map["b"] - 1;
        mna.g[in_idx][in_idx] += 1.0;

        let fs = 48000.0;
        let t = 1.0 / fs;
        let mut kernel = DkKernel::from_mna(&mna, fs).unwrap();
        let input = sine_input(512, fs, 2000.0, 1.0);
        let v_hist = march_companion_kernel(&mut kernel, &input, in_idx);

        // Exact trapezoidal reference: states i (series current), v_c.
        //   v_L = u - R_tot*i - v_c
        //   i[n] = i[n-1] + (T/2L)(v_L[n] + v_L[n-1])
        //   v_c[n] = v_c[n-1] + (T/2C)(i[n] + i[n-1])
        let r_tot = 101.0;
        let l = 10e-3;
        let cap = 100e-9;
        let al = t / (2.0 * l);
        let bc = t / (2.0 * cap);
        let mut i = 0.0;
        let mut v_c = 0.0;
        let mut v_l = 0.0;
        let mut max_err = 0.0_f64;
        for (k, &u) in input.iter().enumerate() {
            // i[n]*(1 + al*R_tot + al*bc) = i[n-1]*(1 - al*bc) + al*(u - v_c[n-1] + v_L[n-1])
            let i_new = (i * (1.0 - al * bc) + al * (u - v_c + v_l)) / (1.0 + al * r_tot + al * bc);
            let v_c_new = v_c + bc * (i_new + i);
            let v_l_new = u - r_tot * i_new - v_c_new;
            i = i_new;
            v_c = v_c_new;
            v_l = v_l_new;
            let v_in_ref = u - 1.0 * i;
            let v_a_ref = u - r_tot * i;
            let v_b_ref = v_c;
            max_err = max_err
                .max((v_hist[k][in_idx] - v_in_ref).abs())
                .max((v_hist[k][a_idx] - v_a_ref).abs())
                .max((v_hist[k][b_idx] - v_b_ref).abs());
        }
        assert!(
            max_err < 1e-12,
            "RLC companion march deviates from exact trapezoidal reference: max_err = {max_err:.3e}"
        );
    }

    /// Coupled 2-winding pair (K statement): vector companion history.
    /// Reference: i[n] = i[n-1] + (T/2)*inv(L)*(v[n] + v[n-1]) with
    /// v = b - D*i (b = [u, 0], D = diag(R_tot, R_load)).
    #[test]
    fn test_companion_coupled_pair_time_march_matches_exact_trapezoidal() {
        let spice =
            "Coupled Pair\nR1 in p1 100\nL1 p1 0 10m\nL2 s1 0 10m\nK1 L1 L2 0.9\nRload s1 0 1k\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        assert_eq!(mna.coupled_inductors.len(), 1);
        let in_idx = mna.node_map["in"] - 1;
        let p1_idx = mna.node_map["p1"] - 1;
        let s1_idx = mna.node_map["s1"] - 1;
        mna.g[in_idx][in_idx] += 1.0;

        let fs = 48000.0;
        let t = 1.0 / fs;
        let mut kernel = DkKernel::from_mna(&mna, fs).unwrap();
        let input = sine_input(512, fs, 1000.0, 1.0);
        let v_hist = march_companion_kernel(&mut kernel, &input, in_idx);

        // Reference
        let l1 = 10e-3_f64;
        let l2 = 10e-3_f64;
        let m_val = 0.9 * (l1 * l2).sqrt();
        let l_mat = vec![vec![l1, m_val], vec![m_val, l2]];
        let y_l = invert_matrix(&l_mat).unwrap();
        let d = [101.0, 1000.0];
        let ht = t / 2.0;
        // Solve (I + ht*Y*D) i[n] = i[n-1] + ht*Y*(b[n] + v[n-1]) each step
        let mut sys = vec![vec![0.0; 2]; 2];
        for r in 0..2 {
            for c in 0..2 {
                sys[r][c] = (r == c) as u32 as f64 + ht * y_l[r][c] * d[c];
            }
        }
        let sys_inv = invert_matrix(&sys).unwrap();
        let mut i = [0.0_f64; 2];
        let mut v = [0.0_f64; 2];
        let mut max_err = 0.0_f64;
        for (k, &u) in input.iter().enumerate() {
            let b = [u, 0.0];
            let mut rhs = [0.0_f64; 2];
            for r in 0..2 {
                rhs[r] = i[r];
                for c in 0..2 {
                    rhs[r] += ht * y_l[r][c] * (b[c] + v[c]);
                }
            }
            let mut i_new = [0.0_f64; 2];
            for r in 0..2 {
                for c in 0..2 {
                    i_new[r] += sys_inv[r][c] * rhs[c];
                }
            }
            for r in 0..2 {
                v[r] = b[r] - d[r] * i_new[r];
            }
            i = i_new;
            max_err = max_err
                .max((v_hist[k][p1_idx] - v[0]).abs())
                .max((v_hist[k][s1_idx] - v[1]).abs());
        }
        assert!(
            max_err < 1e-12,
            "Coupled-pair companion march deviates from exact trapezoidal reference: \
             max_err = {max_err:.3e}"
        );
    }

    /// 3-winding transformer group (2-winding K pairs route to
    /// coupled_inductors; the NxN group path needs >= 3 windings).
    #[test]
    fn test_companion_transformer_group_time_march_matches_exact_trapezoidal() {
        let spice = "Three Winding\nR1 in p1 100\nL1 p1 0 10m\nL2 s1 0 10m\nL3 s2 0 10m\n\
K1 L1 L2 0.9\nK2 L1 L3 0.9\nK3 L2 L3 0.9\nR2 s1 0 1k\nR3 s2 0 2.2k\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        assert_eq!(
            mna.transformer_groups.len(),
            1,
            "3 mutually coupled windings should form one transformer group"
        );
        assert_eq!(mna.transformer_groups[0].num_windings, 3);
        let in_idx = mna.node_map["in"] - 1;
        let p1_idx = mna.node_map["p1"] - 1;
        let s1_idx = mna.node_map["s1"] - 1;
        let s2_idx = mna.node_map["s2"] - 1;
        mna.g[in_idx][in_idx] += 1.0;

        let fs = 48000.0;
        let t = 1.0 / fs;
        let mut kernel = DkKernel::from_mna(&mna, fs).unwrap();
        let input = sine_input(512, fs, 1000.0, 1.0);
        let v_hist = march_companion_kernel(&mut kernel, &input, in_idx);

        // Reference with 3x3 L matrix (all couplings 0.9)
        let lv = 10e-3;
        let mv = 0.9 * lv;
        let l_mat = vec![vec![lv, mv, mv], vec![mv, lv, mv], vec![mv, mv, lv]];
        let y_l = invert_matrix(&l_mat).unwrap();
        let d = [101.0, 1000.0, 2200.0];
        let ht = t / 2.0;
        let mut sys = vec![vec![0.0; 3]; 3];
        for r in 0..3 {
            for c in 0..3 {
                sys[r][c] = (r == c) as u32 as f64 + ht * y_l[r][c] * d[c];
            }
        }
        let sys_inv = invert_matrix(&sys).unwrap();
        let mut i = [0.0_f64; 3];
        let mut v = [0.0_f64; 3];
        let mut max_err = 0.0_f64;
        for (k, &u) in input.iter().enumerate() {
            let b = [u, 0.0, 0.0];
            let mut rhs = [0.0_f64; 3];
            for r in 0..3 {
                rhs[r] = i[r];
                for c in 0..3 {
                    rhs[r] += ht * y_l[r][c] * (b[c] + v[c]);
                }
            }
            let mut i_new = [0.0_f64; 3];
            for r in 0..3 {
                for c in 0..3 {
                    i_new[r] += sys_inv[r][c] * rhs[c];
                }
            }
            for r in 0..3 {
                v[r] = b[r] - d[r] * i_new[r];
            }
            i = i_new;
            max_err = max_err
                .max((v_hist[k][p1_idx] - v[0]).abs())
                .max((v_hist[k][s1_idx] - v[1]).abs())
                .max((v_hist[k][s2_idx] - v[2]).abs());
        }
        assert!(
            max_err < 1e-12,
            "Transformer-group companion march deviates from exact trapezoidal reference: \
             max_err = {max_err:.3e}"
        );
    }

    // ── Convergence-order verification ──────────────────────────────────
    //
    // The companion closed-form tests above compare the kernel against the
    // exact *discrete* trapezoidal recurrence, so they pass to 1e-12 no
    // matter what order the integrator actually is — they cannot see the
    // integration order. These tests instead compare against the
    // *continuous-time* analytical solution and check how the error scales
    // as the timestep is halved. Trapezoidal integration is 2nd-order, so
    // halving dt must quarter the global error (ratio → 4). A ratio near 2
    // would betray a 1st-order (backward-Euler-like) integrator or a
    // half-sample time-alignment error; a ratio near 1 a broken integrator.

    /// Continuous-time response of the RC lowpass `R1 in out 1k / C1 out 0 1u`
    /// with a Thevenin source (G_in = 1 ⇒ R_src = 1Ω) driven by A·sin(ωt),
    /// with initial condition V_out(0) = 0.
    ///
    /// The single storage node obeys  V' = -(1/τ)(V - A·sin(ωt))  with
    /// τ = R_tot·C, R_tot = R_src + R1. Forced first-order response:
    ///   V(t) = A·H·sin(ωt + φ) - A·H·sin(φ)·e^{-t/τ}
    /// with H = 1/√(1+(ωτ)²), φ = -atan(ωτ).
    fn rc_lowpass_continuous(t: f64, amp: f64, omega: f64, tau: f64) -> f64 {
        let h = 1.0 / (1.0 + (omega * tau).powi(2)).sqrt();
        let phi = -(omega * tau).atan();
        amp * h * (omega * t + phi).sin() - amp * h * phi.sin() * (-t / tau).exp()
    }

    #[test]
    fn test_trapezoidal_integrator_is_second_order_rc() {
        let spice = "RC Lowpass Order\nR1 in out 1k\nC1 out 0 1u\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        let in_idx = mna.node_map["in"] - 1;
        let out_idx = mna.node_map["out"] - 1;
        mna.g[in_idx][in_idx] += 1.0;

        // τ = R_tot·C, R_tot = R_src(1Ω) + R1(1kΩ) = 1001Ω.
        let r_tot = 1001.0;
        let cap = 1e-6;
        let tau = r_tot * cap;
        let amp = 1.0;
        let freq = 1000.0;
        let omega = 2.0 * std::f64::consts::PI * freq;

        // Fixed physical window (~4 time constants: transient + steady state).
        let duration = 4e-3;

        // Refine the timestep by 2× each time; measure max error vs the
        // continuous solution over the whole window.
        let sample_rates: [f64; 3] = [48_000.0, 96_000.0, 192_000.0];
        let mut errors = Vec::new();
        for &fs in &sample_rates {
            let dt = 1.0 / fs;
            let steps = (duration * fs).round() as usize;
            let input: Vec<f64> = (0..steps)
                .map(|k| amp * (omega * k as f64 * dt).sin())
                .collect();
            let mut kernel = DkKernel::from_mna(&mna, fs).unwrap();
            let v_hist = march_companion_kernel(&mut kernel, &input, in_idx);

            let mut max_err = 0.0_f64;
            // Skip k=0 (trivially the IC); compare v_hist[k] to V(k·dt).
            for k in 1..steps {
                let t = k as f64 * dt;
                let v_ref = rc_lowpass_continuous(t, amp, omega, tau);
                max_err = max_err.max((v_hist[k][out_idx] - v_ref).abs());
            }
            errors.push(max_err);
        }

        let ratio_lo = errors[0] / errors[1];
        let ratio_hi = errors[1] / errors[2];
        // 2nd order ⇒ each halving of dt quarters the error (ratio → 4).
        // Generous window absorbs the finite-dt approach to the asymptote.
        assert!(
            (3.3..=4.7).contains(&ratio_lo) && (3.3..=4.7).contains(&ratio_hi),
            "trapezoidal integrator is not 2nd-order: errors = {errors:?}, \
             ratios = {ratio_lo:.3}, {ratio_hi:.3} (expected ~4.0 each). \
             A ratio ~2 indicates 1st-order or a half-sample time misalignment."
        );
        // Sanity: the errors must actually be small and strictly shrinking.
        assert!(
            errors[0] < 1e-3 && errors[1] < errors[0] && errors[2] < errors[1],
            "RC convergence errors not shrinking as expected: {errors:?}"
        );
    }

    /// Continuous-time capacitor voltage of the series RLC
    /// `R1 in a 100 / L1 a b 10m / C1 b 0 100n` with a Thevenin source
    /// (G_in = 1 ⇒ R_src = 1Ω), driven by A·sin(ωt), ICs v_c(0)=0, i(0)=0.
    ///
    /// Governing ODE (series loop, output = v_c across C):
    ///   v_c'' + (R_tot/L)·v_c' + (1/LC)·v_c = u/(LC),  R_tot = R_src + R1.
    /// Underdamped forced response = steady state + decaying homogeneous:
    ///   v_c(t) = M·sin(ωt+ψ) + e^{-σt}(C1·cos ωd t + C2·sin ωd t)
    /// with ω0=1/√(LC), σ=R_tot/2L, ζ=σ/ω0, ωd=ω0√(1-ζ²),
    ///   M = A·ω0²/√((ω0²-ω²)² + (2σω)²),  ψ = -atan2(2σω, ω0²-ω²),
    /// and C1, C2 fixed by the zero initial conditions.
    fn rlc_series_vc_continuous(t: f64, amp: f64, omega: f64, r_tot: f64, l: f64, cap: f64) -> f64 {
        let w0 = 1.0 / (l * cap).sqrt();
        let sigma = r_tot / (2.0 * l);
        let zeta = sigma / w0;
        let wd = w0 * (1.0 - zeta * zeta).sqrt();
        let mag = amp * w0 * w0
            / ((w0 * w0 - omega * omega).powi(2) + (2.0 * sigma * omega).powi(2)).sqrt();
        let psi = -(2.0 * sigma * omega).atan2(w0 * w0 - omega * omega);
        // Steady-state value and derivative at t=0, from v_ss = M sin(ωt+ψ).
        let vss0 = mag * psi.sin();
        let vss_dot0 = mag * omega * psi.cos();
        // ICs v_c(0)=0, v_c'(0)=0 ⇒ solve for homogeneous constants.
        let c1 = -vss0;
        let c2 = (sigma * c1 - vss_dot0) / wd;
        let vss = mag * (omega * t + psi).sin();
        vss + (-sigma * t).exp() * (c1 * (wd * t).cos() + c2 * (wd * t).sin())
    }

    #[test]
    fn test_trapezoidal_integrator_is_second_order_rlc() {
        let spice = "RLC Order\nR1 in a 100\nL1 a b 10m\nC1 b 0 100n\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        let in_idx = mna.node_map["in"] - 1;
        let b_idx = mna.node_map["b"] - 1;
        mna.g[in_idx][in_idx] += 1.0;

        let r_tot = 101.0; // R_src(1Ω) + R1(100Ω)
        let l = 10e-3;
        let cap = 100e-9;
        let amp = 1.0;
        let freq = 2000.0; // below the ~5.03 kHz resonance
        let omega = 2.0 * std::f64::consts::PI * freq;
        let duration = 2e-3;

        let sample_rates: [f64; 3] = [96_000.0, 192_000.0, 384_000.0];
        let mut errors = Vec::new();
        for &fs in &sample_rates {
            let dt = 1.0 / fs;
            let steps = (duration * fs).round() as usize;
            let input: Vec<f64> = (0..steps)
                .map(|k| amp * (omega * k as f64 * dt).sin())
                .collect();
            let mut kernel = DkKernel::from_mna(&mna, fs).unwrap();
            let v_hist = march_companion_kernel(&mut kernel, &input, in_idx);

            let mut max_err = 0.0_f64;
            for k in 1..steps {
                let t = k as f64 * dt;
                let v_ref = rlc_series_vc_continuous(t, amp, omega, r_tot, l, cap);
                max_err = max_err.max((v_hist[k][b_idx] - v_ref).abs());
            }
            errors.push(max_err);
        }

        let ratio_lo = errors[0] / errors[1];
        let ratio_hi = errors[1] / errors[2];
        assert!(
            (3.3..=4.7).contains(&ratio_lo) && (3.3..=4.7).contains(&ratio_hi),
            "trapezoidal integrator is not 2nd-order on RLC: errors = {errors:?}, \
             ratios = {ratio_lo:.3}, {ratio_hi:.3} (expected ~4.0 each)"
        );
        assert!(
            errors[0] < 2e-2 && errors[1] < errors[0] && errors[2] < errors[1],
            "RLC convergence errors not shrinking as expected: {errors:?}"
        );
    }

    /// Ideal resistive divider: the settled DC output must equal the exact
    /// series-divider ratio to machine precision. `R1 in mid 1k / R2 mid 0
    /// 1k`, output at `mid`, with the Thevenin source (G_in=1 ⇒ R_src=1Ω)
    /// in series. A tiny cap at `mid` gives a settling path; at DC the cap
    /// is open, so V_mid → V_src·R2/(R_src+R1+R2).
    #[test]
    fn test_ideal_voltage_divider_dc_ratio() {
        let spice = "Divider\nR1 in mid 1k\nR2 mid 0 1k\nC1 mid 0 100n\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        let in_idx = mna.node_map["in"] - 1;
        let mid_idx = mna.node_map["mid"] - 1;
        mna.g[in_idx][in_idx] += 1.0;

        let fs = 48_000.0;
        let mut kernel = DkKernel::from_mna(&mna, fs).unwrap();
        // Hold a 1V DC input long enough to settle far past machine epsilon
        // (τ ≈ (R_src+R1)‖R2 · C ≈ 50µs; 0.2s ≈ 4000τ).
        let v_src = 1.0;
        let steps = (0.2 * fs) as usize;
        let input = vec![v_src; steps];
        let v_hist = march_companion_kernel(&mut kernel, &input, in_idx);

        let expected = v_src * 1000.0 / (1.0 + 1000.0 + 1000.0); // R2/(R_src+R1+R2)
        let settled = v_hist.last().unwrap()[mid_idx];
        assert!(
            (settled - expected).abs() < 1e-12,
            "divider settled to {settled:.15} but exact ratio is {expected:.15}"
        );
    }

    // ── from_mna_augmented identity tests (F4b) ─────────────────────────

    /// Augmented-MNA inductor branch rows must carry exact trapezoidal
    /// structure: A[k][k] = 2L/T on the branch diagonal, A_neg branch row
    /// mirroring it (same 2L/T, opposite-signed node couplings — so
    /// A_neg*x_prev preserves the v_L_prev + (2L/T)*j_prev history), and
    /// VS algebraic rows zeroed in A_neg.
    #[test]
    fn test_from_mna_augmented_branch_row_identities() {
        let spice = "Aug RL\nVcc vcc 0 DC 9\nR1 vcc a 100\nL1 a 0 10m\nC1 a 0 100n\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        let in_idx = mna.node_map["vcc"] - 1;
        mna.g[in_idx][in_idx] += 1.0;

        let fs = 48000.0;
        let t = 1.0 / fs;
        let kernel = DkKernel::from_mna_augmented(&mna, fs).unwrap();

        let n_aug = mna.n_aug;
        let n = kernel.n;
        assert_eq!(n, n_aug + 1, "one uncoupled inductor adds one branch var");
        let branch = n_aug; // first (only) inductor branch row
        let l = 10e-3;
        let expected_diag = 2.0 * l / t;

        // A is not stored on the kernel, but A = S^{-1}; check the identity
        // through A*S = I is already covered elsewhere. Instead verify the
        // stored A_neg directly and rebuild A from the augmented matrices.
        let aug = mna.build_augmented_matrices();
        let alpha = 2.0 / t;
        let a_branch_diag = aug.g[branch][branch] + alpha * aug.c[branch][branch];
        assert!(
            (a_branch_diag.abs() - expected_diag).abs() < 1e-9,
            "branch row diagonal |A[k][k]| = {a_branch_diag} should be 2L/T = {expected_diag}"
        );

        // A_neg branch row: diagonal keeps +2L/T (from alpha*C), node
        // couplings flip sign vs A (from -G) — preserving trapezoidal
        // history v_L_prev + (2L/T)*j_prev.
        let a_neg_branch_diag = kernel.a_neg(branch, branch);
        assert!(
            (a_neg_branch_diag.abs() - expected_diag).abs() < 1e-9,
            "A_neg branch diagonal |{a_neg_branch_diag}| should be 2L/T = {expected_diag}"
        );
        let a_node = mna.node_map["a"] - 1;
        let g_branch_node = aug.g[branch][a_node];
        assert!(
            g_branch_node.abs() > 0.5,
            "branch row must couple to the inductor node with a ±1 entry"
        );
        assert!(
            (kernel.a_neg(branch, a_node) + g_branch_node).abs() < 1e-12,
            "A_neg branch/node coupling must be -G coupling (sign flip vs A)"
        );

        // VS algebraic row (row n_nodes..n_aug) must be zeroed in A_neg
        for row in mna.n..n_aug {
            for j in 0..n {
                assert_eq!(
                    kernel.a_neg(row, j),
                    0.0,
                    "VS algebraic A_neg row {row} must be zeroed (col {j})"
                );
            }
        }
    }

    // ── build_rhs_const scaling (F4c) ───────────────────────────────────

    /// DC current-source node rows are doubled (trapezoidal average of a
    /// constant); VS augmented rows are NOT doubled (algebraic constraint).
    #[test]
    fn test_build_rhs_const_scaling() {
        let spice =
            "RHS Const\nVcc vcc 0 DC 9\nI1 0 a DC 2m\nR1 vcc a 10k\nR2 a 0 10k\nC1 a 0 1u\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

        // I1 0 a: node a is n_minus → injection is -I_dc, doubled by the
        // trapezoidal source average to -2*I_dc.
        let a_node = mna.node_map["a"] - 1;
        assert!(
            (kernel.rhs_const[a_node] - (-2.0 * 2e-3)).abs() < 1e-15,
            "current-source node row must be ±2*I_dc (trapezoidal doubling), got {}",
            kernel.rhs_const[a_node]
        );

        let vs = &mna.voltage_sources[0];
        let vs_row = mna.n + vs.ext_idx;
        assert!(
            (kernel.rhs_const[vs_row] - 9.0).abs() < 1e-15,
            "VS augmented row must be V_dc un-doubled, got {}",
            kernel.rhs_const[vs_row]
        );
    }

    // ── K verified against an independent solve (F4d) ───────────────────

    /// K = N_v * S * N_i is only meaningful if S is actually A^{-1}.
    /// Recomputing N_v*S*N_i from the kernel's own S is circular; instead
    /// assert A * (S*N_i) = N_i column-wise, which validates the solve
    /// underlying K without requiring a second matrix inverter.
    #[test]
    fn test_k_via_a_times_s_ni_equals_n_i() {
        let spice = r#"BJT CE for K check
Cin in base 10u
R1 vcc base 100k
R2 base 0 22k
Q1 coll base emit MYBJTCE
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Cout coll out 10u
Rload out 0 100k
Vcc vcc 0 DC 12
.model MYBJTCE NPN(IS=1e-14 BF=200 BR=3)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();
        let a = mna.get_a_matrix(44100.0).unwrap();
        let n = kernel.n;
        let m = kernel.m;
        assert!(m >= 2);

        // s_ni[i][j] = sum_k S[i][k] * N_i[k][j]
        let mut s_ni = vec![vec![0.0; m]; n];
        for i in 0..n {
            for j in 0..m {
                for k in 0..n {
                    s_ni[i][j] += kernel.s(i, k) * kernel.n_i(k, j);
                }
            }
        }
        // Check A * (S*N_i) = N_i column-wise
        for col in 0..m {
            for row in 0..n {
                let mut sum = 0.0;
                for k in 0..n {
                    sum += a[row][k] * s_ni[k][col];
                }
                assert!(
                    (sum - kernel.n_i(row, col)).abs() < 1e-9,
                    "A*(S*N_i) != N_i at [{row}][{col}]: {sum} vs {}",
                    kernel.n_i(row, col)
                );
            }
        }
    }

    #[test]
    fn test_k_diagonal_negative_triode() {
        let spice = r#"Triode CC
Cin in grid 100n
Rg grid 0 1Meg
T1 grid plate cathode 12AX7
Rk cathode 0 1.5k
Ck cathode 0 25u
Rp vcc plate 100k
Vcc vcc 0 DC 250
.model 12AX7 TUBE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
"#;
        let netlist = Netlist::parse(spice).unwrap();
        let mna = MnaSystem::from_netlist(&netlist).unwrap();
        let dk = DkKernel::from_mna(&mna, 44100.0).unwrap();

        for i in 0..dk.m {
            assert!(
                dk.k(i, i) <= 0.0,
                "Triode: K[{i}][{i}] = {} should be <= 0 (negative feedback)",
                dk.k(i, i)
            );
        }
    }
}
