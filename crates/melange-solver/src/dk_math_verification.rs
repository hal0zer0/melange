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
    use crate::dk::{DkKernel, invert_matrix, mat_mul, mat_vec_mul};
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
        let a = mna.get_a_matrix(sample_rate);

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
        let a_neg = mna.get_a_neg_matrix(sample_rate);

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
        let a = mna.get_a_matrix(sample_rate);

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

        let a = mna.get_a_matrix(sample_rate);
        let a_neg = mna.get_a_neg_matrix(sample_rate);

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

        let a = mna.get_a_matrix(sample_rate);
        let a_neg = mna.get_a_neg_matrix(sample_rate);

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
}
