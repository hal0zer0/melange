//! Newton-Raphson solvers for nonlinear equations.
//!
//! These are designed for real-time audio circuit simulation:
//! - Fixed maximum iterations (bounded runtime)
//! - Warm start from previous solution
//! - Step clamping for convergence
//! - NaN/inf detection and recovery
//!
//! All solvers use stack allocation only - no heap allocation.

/// Result of a Newton-Raphson solve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NrResult {
    /// Converged to solution within tolerance.
    Converged { iterations: u32 },
    /// Did not converge within max iterations (may still be usable).
    MaxIterations { final_error: f64 },
    /// Divergence detected (NaN or inf).
    Divergence,
}

impl NrResult {
    /// Returns true if the solve converged.
    pub fn converged(&self) -> bool {
        matches!(self, NrResult::Converged { .. })
    }

    /// Returns the number of iterations (if converged).
    pub fn iterations(&self) -> Option<u32> {
        match self {
            NrResult::Converged { iterations } => Some(*iterations),
            _ => None,
        }
    }
}

/// Solve a 1D nonlinear equation using Newton-Raphson.
///
/// Finds x such that f(x) = 0, given f and its derivative df/dx.
///
/// # Arguments
/// * `f` - Function to find root of
/// * `df` - Derivative of f
/// * `x0` - Initial guess
/// * `max_iter` - Maximum iterations (typically 20-50)
/// * `tol` - Tolerance for convergence (typically 1e-7)
/// * `clamp` - Maximum step size per iteration (e.g., 2*Vt = 0.05 for diodes)
///
/// # Returns
/// (solution_x, result)
pub fn nr_solve_1d<F, DF>(
    f: F,
    df: DF,
    x0: f64,
    max_iter: u32,
    tol: f64,
    clamp: f64,
) -> (f64, NrResult)
where
    F: Fn(f64) -> f64,
    DF: Fn(f64) -> f64,
{
    let mut x = x0;

    for iter in 1..=max_iter {
        let fx = f(x);

        // Check residual convergence
        if fx.abs() < tol {
            return (x, NrResult::Converged { iterations: iter });
        }

        let dfx = df(x);

        // Check for zero derivative
        if dfx.abs() < 1e-15 {
            return (x, NrResult::Divergence);
        }

        // Newton step with clamping
        let dx = fx / dfx;
        let dx_clamped = dx.clamp(-clamp, clamp);
        
        // Check step-size convergence
        if dx_clamped.abs() < tol {
            return (x, NrResult::Converged { iterations: iter });
        }
        
        x -= dx_clamped;

        // Check for NaN/inf
        if !x.is_finite() {
            return (x0, NrResult::Divergence);
        }
    }

    // Max iterations reached
    let final_error = f(x).abs();
    (x, NrResult::MaxIterations { final_error })
}

/// Solve a 2D nonlinear system using Newton-Raphson with Cramer's rule.
///
/// Finds (x, y) such that:
///   f1(x, y) = 0
///   f2(x, y) = 0
///
/// Uses Cramer's rule for the 2x2 linear solve (faster than LU decomposition).
///
/// # Arguments
/// * `f1`, `f2` - Functions to find roots of
/// * `df1_dx`, `df1_dy` - Partial derivatives of f1
/// * `df2_dx`, `df2_dy` - Partial derivatives of f2
/// * `(x0, y0)` - Initial guess
/// * `max_iter` - Maximum iterations
/// * `tol` - Tolerance for convergence
/// * `clamp` - Maximum step size per iteration (per dimension)
/// 
/// Note: This function has many parameters because it takes 6 callback functions
/// plus solver configuration. This is intentional for flexibility.
#[allow(clippy::too_many_arguments)]
pub fn nr_solve_2d<F1, F2, DF1X, DF1Y, DF2X, DF2Y>(
    f1: F1,
    f2: F2,
    df1_dx: DF1X,
    df1_dy: DF1Y,
    df2_dx: DF2X,
    df2_dy: DF2Y,
    x0: f64,
    y0: f64,
    max_iter: u32,
    tol: f64,
    clamp: f64,
) -> (f64, f64, NrResult)
where
    F1: Fn(f64, f64) -> f64,
    F2: Fn(f64, f64) -> f64,
    DF1X: Fn(f64, f64) -> f64,
    DF1Y: Fn(f64, f64) -> f64,
    DF2X: Fn(f64, f64) -> f64,
    DF2Y: Fn(f64, f64) -> f64,
{
    let mut x = x0;
    let mut y = y0;

    for iter in 1..=max_iter {
        let f1_val = f1(x, y);
        let f2_val = f2(x, y);

        // Check residual convergence
        let error_sq = f1_val * f1_val + f2_val * f2_val;
        if error_sq < tol * tol {
            return (x, y, NrResult::Converged { iterations: iter });
        }

        // Build Jacobian
        let j11 = df1_dx(x, y);
        let j12 = df1_dy(x, y);
        let j21 = df2_dx(x, y);
        let j22 = df2_dy(x, y);

        // Determinant of Jacobian
        let det = j11 * j22 - j12 * j21;

        if det.abs() < 1e-15 {
            return (x, y, NrResult::Divergence);
        }

        // Cramer's rule for J * [dx; dy] = [f1; f2]
        let dx = (f1_val * j22 - j12 * f2_val) / det;
        let dy = (j11 * f2_val - f1_val * j21) / det;

        // Check step-size convergence
        let step_sq = dx * dx + dy * dy;
        if step_sq < tol * tol {
            return (x, y, NrResult::Converged { iterations: iter });
        }

        // Clamped update
        x -= dx.clamp(-clamp, clamp);
        y -= dy.clamp(-clamp, clamp);

        // Check for NaN/inf
        if !x.is_finite() || !y.is_finite() {
            return (x0, y0, NrResult::Divergence);
        }
    }

    // Max iterations
    let final_error = (f1(x, y).powi(2) + f2(x, y).powi(2)).sqrt();
    (x, y, NrResult::MaxIterations { final_error })
}

/// Newton-Raphson solver for the DK-method reduced system.
///
/// Solves: v_nl = p + K * i_nl(v_nl)
///
/// Or equivalently: f(v) = v - p - K*i(v) = 0
///
/// This is the specialized solver used in melange-solver's code generation.
///
/// # Type Parameters
/// * `M` - Number of nonlinear devices (dimension of the kernel)
/// * `I` - Current function: i_nl(v) -> [f64; M]
/// * `G` - Gradient function: di_nl/dv(v) -> [[f64; M]; M] (diagonal matrix)
///
/// # Arguments
/// * `p` - Predicted voltages from linear solve
/// * `k` - Kernel matrix K (M×M)
/// * `i_nl` - Nonlinear current function
/// * `g_nl` - Nonlinear gradient function (returns diagonal conductances)
/// * `v0` - Initial guess
/// * `max_iter` - Maximum iterations
/// * `tol` - Tolerance
/// * `clamp` - Maximum step size per iteration (per dimension)
///
/// # Returns
/// (solution_v_nl, iterations, converged)
/// Solve M-dimensional nonlinear system for DK method.
#[allow(clippy::too_many_arguments)]
pub fn nr_solve_dk<const M: usize, I, G>(
    p: &[f64; M],
    k: &[[f64; M]; M],
    i_nl: I,
    g_nl: G,
    v0: &[f64; M],
    max_iter: u32,
    tol: f64,
    clamp: f64,
) -> ([f64; M], u32, bool)
where
    I: Fn(&[f64; M]) -> [f64; M],
    G: Fn(&[f64; M]) -> [f64; M],  // Returns diagonal of Jacobian (conductances)
{
    let mut v = *v0;

    for iter in 1..=max_iter {
        let i = i_nl(&v);
        let g = g_nl(&v);  // Conductances (diagonal of ∂i/∂v)

        // Residual: f(v) = v - p - K*i(v)
        let mut residual = [0.0; M];
        for m in 0..M {
            residual[m] = v[m] - p[m];
            for n in 0..M {
                residual[m] -= k[m][n] * i[n];
            }
        }

        // Check residual convergence
        let error_sq: f64 = residual.iter().map(|r| r * r).sum();
        if error_sq < tol * tol {
            return (v, iter, true);
        }

        // Jacobian: J = I - K*G where G = diag(g)
        // For M ≤ 4, we can do explicit inverse or use Gaussian elimination
        // Here we use Gaussian elimination with partial pivoting for generality

        // Build J and -residual (for solving J*dv = -residual)
        let mut j = [[0.0; M]; M];
        let mut rhs = [0.0; M];
        for m in 0..M {
            rhs[m] = -residual[m];
            for n in 0..M {
                j[m][n] = if m == n { 1.0 } else { 0.0 };
                j[m][n] -= k[m][n] * g[n];
            }
        }

        // Solve J * dv = rhs using Gaussian elimination
        let (dv, singular) = solve_linear_m::<M>(&j, &rhs);

        // Singular Jacobian — fall back to damped step
        if singular {
            for m in 0..M {
                v[m] -= rhs[m] * 0.5_f64.min(clamp);
            }
            continue;
        }

        // Check step-size convergence
        let step_sq: f64 = dv.iter().map(|x| x * x).sum();
        if step_sq < tol * tol {
            return (v, iter, true);
        }

        // Update with clamping
        for m in 0..M {
            v[m] += dv[m].clamp(-clamp, clamp);
        }

        // Check for NaN
        if !v.iter().all(|x| x.is_finite()) {
            return (*v0, 0, false);
        }
    }

    (v, max_iter, false)
}

/// Solve M×M linear system using Gaussian elimination with partial pivoting.
///
/// For small M (≤4), this is faster than general LU decomposition.
/// Returns `(solution, singular)` where `singular` is true if the matrix was (near-)singular.
///
/// Uses stack allocation only - no heap allocation.
#[allow(clippy::needless_range_loop)]
fn solve_linear_m<const M: usize>(a: &[[f64; M]; M], b: &[f64; M]) -> ([f64; M], bool) {
    // Maximum supported dimension is 8 (more than enough for audio circuits)
    const MAX_M: usize = 8;
    assert!(M <= MAX_M, "solve_linear_m only supports M up to {}", MAX_M);
    
    // Create augmented matrix [A | b] on stack with fixed max size
    let mut aug = [[0.0; MAX_M + 1]; MAX_M];
    for i in 0..M {
        for j in 0..M {
            aug[i][j] = a[i][j];
        }
        aug[i][M] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..M {
        // Find pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..M {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for j in col..=M {
                let tmp = aug[col][j];
                aug[col][j] = aug[max_row][j];
                aug[max_row][j] = tmp;
            }
        }

        // Check for singular matrix
        if aug[col][col].abs() < 1e-15 {
            return ([0.0; M], true);
        }

        // Eliminate column
        for row in (col + 1)..M {
            let factor = aug[row][col] / aug[col][col];
            for j in col..=M {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = [0.0; M];
    for i in (0..M).rev() {
        let mut sum = aug[i][M];
        for j in (i + 1)..M {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() < 1e-15 {
            return ([0.0; M], true);
        }
        x[i] = sum / aug[i][i];
    }

    (x, false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nr_1d_sqrt() {
        // Solve x^2 - 2 = 0
        let (x, result) = nr_solve_1d(
            |x| x * x - 2.0,
            |x| 2.0 * x,
            1.0,
            50,
            1e-10,
            10.0,
        );
        assert!(result.converged());
        assert!((x - 1.4142135623730951).abs() < 1e-8);
    }

    #[test]
    fn test_nr_1d_diode() {
        // Diode equation: I = Is * (exp(V/Vt) - 1)
        // Solve for V given I = 1mA, Is = 1e-15, Vt = 0.02585
        let is = 1e-15;
        let vt = 0.02585;
        let target_i = 0.001;

        let (v, result) = nr_solve_1d(
            |v| is * ((v / vt).exp() - 1.0) - target_i,
            |v| is * (v / vt).exp() / vt,
            0.7,  // Typical diode drop
            50,
            1e-12,
            0.1,  // Clamp to 0.1V (≈ 4*Vt)
        );
        assert!(result.converged());
        // V = Vt * ln(I/Is + 1) ≈ 0.02585 * ln(0.001/1e-15) ≈ 0.73V
        assert!(v > 0.6 && v < 0.8);
    }

    #[test]
    fn test_nr_2d_linear() {
        // Solve: 2x + y = 5
        //        x - y = 1
        // Solution: x = 2, y = 1
        let (x, y, result) = nr_solve_2d(
            |x, y| 2.0 * x + y - 5.0,
            |x, y| x - y - 1.0,
            |_x, _y| 2.0,
            |_x, _y| 1.0,
            |_x, _y| 1.0,
            |_x, _y| -1.0,
            0.0, 0.0,
            10,
            1e-10,
            10.0,
        );
        assert!(result.converged());
        assert!((x - 2.0).abs() < 1e-8);
        assert!((y - 1.0).abs() < 1e-8);
    }

    #[test]
    fn test_nr_dk_1d() {
        // Simple 1D DK system: v = p + k * i(v)
        // where i(v) = v^2 (nonlinear "current")
        // Solve: v = 0.5 + 0.5 * v^2
        // => v^2 - 2v + 1 = 0
        // => (v-1)^2 = 0
        // => v = 1 (exact solution)

        let p = [0.5];
        let k = [[0.5]];

        let (v, iters, converged) = nr_solve_dk::<1, _, _>(
            &p,
            &k,
            |v| [v[0] * v[0]],  // i(v) = v^2
            |v| [2.0 * v[0]],  // di/dv = 2v
            &[0.5],  // Initial guess
            50,
            1e-10,
            0.1,
        );

        assert!(converged, "Failed to converge after {} iterations", iters);
        // Verify: v should be 1.0 (within reasonable tolerance)
        assert!((v[0] - 1.0).abs() < 1e-4, "Expected v≈1.0, got {} (error: {})", v[0], (v[0] - 1.0).abs());
    }

    #[test]
    fn test_linear_solve_2x2() {
        let a = [[4.0, 3.0], [6.0, 3.0]];
        let b = [10.0, 12.0];
        let (x, singular) = solve_linear_m::<2>(&a, &b);
        assert!(!singular, "Should not be singular");
        // 4x + 3y = 10
        // 6x + 3y = 12
        // => 2x = 2 => x = 1, y = 2
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_solve_3x3() {
        let a = [[3.0, 2.0, -1.0], [2.0, -2.0, 4.0], [-1.0, 0.5, -1.0]];
        let b = [1.0, -2.0, 0.0];
        let (x, singular) = solve_linear_m::<3>(&a, &b);
        assert!(!singular, "Should not be singular");

        // Verify solution
        for i in 0..3 {
            let mut sum = 0.0;
            for j in 0..3 {
                sum += a[i][j] * x[j];
            }
            assert!((sum - b[i]).abs() < 1e-9, "Row {}: {} != {}", i, sum, b[i]);
        }
    }
    
    #[test]
    fn test_nr_step_size_convergence() {
        // Test that solver converges on step size, not just residual
        // Use a function that converges reliably: f(x) = x^2 - 1
        
        let (x, result) = nr_solve_1d(
            |x| x * x - 1.0,
            |x| 2.0 * x,
            2.0,  // Start at x=2
            100,
            1e-10,
            1.0,
        );
        
        // Should converge to x = 1.0
        assert!(result.converged());
        assert!((x - 1.0).abs() < 1e-8);
    }
}
