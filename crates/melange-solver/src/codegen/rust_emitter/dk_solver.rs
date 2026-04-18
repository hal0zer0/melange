//! DK Newton-Raphson solver generation.
//!
//! Contains `generate_solve_nonlinear`, `generate_gauss_elim`, and
//! `generate_schur_gauss_elim` — the procedural NR solver code that is
//! too deeply conditional for Tera templates.

use crate::codegen::ir::CircuitIR;
use crate::codegen::CodegenError;
use super::RustEmitter;
use super::nr_helpers::{
    emit_dk_device_evaluation, emit_nr_limit_and_converge, emit_nr_singular_fallback,
    emit_schur_nr_limit_and_converge,
};

impl RustEmitter {
    /// Generate solve_nonlinear function using Newton-Raphson.
    ///
    /// This method stays procedural due to its deeply conditional
    /// structure: M=1/2/3/4 branching, device-type dispatch, block-diagonal
    /// Jacobian assembly.
    pub(super) fn generate_solve_nonlinear(
        &self,
        code: &mut String,
        ir: &CircuitIR,
    ) -> Result<(), CodegenError> {
        let m = ir.topology.m;

        code.push_str("/// Solve M×M nonlinear system via Newton-Raphson\n");
        code.push_str("/// \n");
        code.push_str("/// Solves: i_nl - i_d(p + K*i_nl) = 0\n");
        code.push_str("/// where p = N_v * v_pred is the linear prediction\n");
        code.push_str("#[inline(always)]\n");
        code.push_str(
            "fn solve_nonlinear(p: &[f64; M], state: &mut CircuitState) -> [f64; M] {\n",
        );
        code.push_str(&format!(
            "    const MAX_ITER: usize = {};\n",
            ir.solver_config.max_iterations
        ));
        code.push_str(&format!(
            "    const TOL: f64 = {:.17e};\n",
            ir.solver_config.tolerance
        ));
        code.push_str("    const SINGULARITY_THRESHOLD: f64 = 1e-15;\n\n");

        code.push_str(
            "    // First-order predictor for NR warm start: i_guess = 2*i_nl[n-1] - i_nl[n-2]\n",
        );
        code.push_str(
            "    // Extrapolates the trend from the last two samples. On smooth signals this\n",
        );
        code.push_str(
            "    // puts the initial guess much closer to the solution, reducing NR iterations.\n",
        );
        code.push_str("    let mut i_nl = [0.0; M];\n");
        code.push_str("    for i in 0..M {\n");
        code.push_str("        i_nl[i] = 2.0 * state.i_nl_prev[i] - state.i_nl_prev_prev[i];\n");
        code.push_str("    }\n\n");

        if m == 0 {
            code.push_str("    // No nonlinear devices\n");
            code.push_str("    state.last_nr_iterations = 0;\n");
            code.push_str("    i_nl\n");
            code.push_str("}\n\n");
            return Ok(());
        }

        let has_nonlinear = !ir.device_slots.is_empty();

        code.push_str("    // Newton-Raphson iteration\n");
        code.push_str("    for iter in 0..MAX_ITER {\n\n");

        // Compute v_d = p + K * i_nl (K is exact from rebuild_matrices)
        {
            code.push_str("        // Compute controlling voltages: v_d = p + K * i_nl\n");
            for i in 0..m {
                code.push_str(&format!("        let v_d{} = p[{}]", i, i));
                for &j in &ir.sparsity.k.nz_by_row[i] {
                    code.push_str(&format!(" + state.k[{}][{}] * i_nl[{}]", i, j, j));
                }
                code.push_str(";\n");
            }
        }
        code.push('\n');

        if has_nonlinear {
            // Device currents and Jacobian entries (shared with Phase E
            // runtime DC-OP recompute — see `nr_helpers::emit_dk_device_evaluation`).
            emit_dk_device_evaluation(code, ir, "        ")?;

            // Residuals
            code.push_str("        // Residuals: f(i) = i - i_dev(v(i)) = 0\n");
            for i in 0..m {
                code.push_str(&format!("        let f{} = i_nl[{}] - i_dev{};\n", i, i, i));
            }
            code.push('\n');

            // NR Jacobian: J[i][j] = delta_ij - sum_k(jdev_ik * K'[k][j])
            // When pots are present, K' = K - scale * nv_su * su_ni^T
            if !ir.pots.is_empty() {
                code.push_str(
                    "        // Jacobian: J[i][j] = delta_ij - sum_k(jdev_ik * K_eff[k][j])\n",
                );
            } else {
                code.push_str(
                    "        // Jacobian: J[i][j] = delta_ij - sum_k(jdev_ik * K[k][j])\n",
                );
            }
            for i in 0..m {
                let slot = ir
                    .device_slots
                    .iter()
                    .find(|s| i >= s.start_idx && i < s.start_idx + s.dimension)
                    .ok_or_else(|| {
                        CodegenError::InvalidConfig(format!(
                            "no device slot found for M-dimension index {}",
                            i
                        ))
                    })?;
                let blk_start = slot.start_idx;
                let blk_dim = slot.dimension;
                for j in 0..m {
                    let diag = if i == j { "1.0" } else { "0.0" };
                    let mut terms = String::new();
                    for k in blk_start..blk_start + blk_dim {
                        terms.push_str(&format!(" - jdev_{}_{} * state.k[{}][{}]", i, k, k, j));
                    }
                    // Separator is load-bearing: `j{i}{j}` without it collides
                    // at M≥12 (e.g. j110 could be i=1,j=10 or i=11,j=0).
                    code.push_str(&format!("        let j{}_{} = {}{};\n", i, j, diag, terms));
                }
            }
            code.push('\n');

            // Solve the linear system based on matrix size
            match m {
                1 => {
                    code.push_str("        // Solve 1x1 system: J * delta = f\n");
                    code.push_str("        let det = j0_0;\n");
                    code.push_str("        if det.abs() < SINGULARITY_THRESHOLD {\n");
                    emit_nr_singular_fallback(code, 1, "            ");
                    code.push_str("            continue;\n");
                    code.push_str("        }\n");
                    code.push_str("        let delta0 = f0 / det;\n\n");
                    emit_nr_limit_and_converge(code, ir, 1, "        ");
                }
                2 => {
                    code.push_str("        // Solve 2x2 system: J * delta = f (Cramer's rule)\n");
                    code.push_str("        let det = j0_0 * j1_1 - j0_1 * j1_0;\n");
                    code.push_str("        if det.abs() < SINGULARITY_THRESHOLD {\n");
                    emit_nr_singular_fallback(code, 2, "            ");
                    code.push_str("            continue;\n");
                    code.push_str("        }\n");
                    code.push_str("        let inv_det = 1.0 / det;\n");
                    code.push_str("        let delta0 = inv_det * (j1_1 * f0 - j0_1 * f1);\n");
                    code.push_str("        let delta1 = inv_det * (-j1_0 * f0 + j0_0 * f1);\n\n");
                    emit_nr_limit_and_converge(code, ir, 2, "        ");
                }
                3..=24 => {
                    Self::generate_gauss_elim(code, ir, m);
                }
                _ => {
                    return Err(CodegenError::UnsupportedTopology(format!(
                        "M={} nonlinear dimensions not supported (max {})",
                        m,
                        crate::dk::MAX_M
                    )));
                }
            }
        } else {
            code.push_str("        // No nonlinear devices to solve\n");
            code.push_str("        state.last_nr_iterations = 0;\n");
            code.push_str("        return i_nl;\n");
        }

        code.push_str("    }\n\n");

        // Max iterations reached
        code.push_str("    // Max iterations reached - return best guess\n");
        code.push_str("    state.last_nr_iterations = MAX_ITER as u32;\n");

        // NaN/Inf check — fall back to previous good values (not zero, which causes glitches)
        code.push_str("    // Safety: check for NaN/Inf and fall back to previous values\n");
        for i in 0..m {
            code.push_str(&format!(
                "    if !i_nl[{}].is_finite() {{ i_nl[{}] = state.i_nl_prev[{}]; }}\n",
                i, i, i
            ));
        }
        code.push('\n');
        code.push_str("    i_nl\n");
        code.push_str("}\n\n");

        Ok(())
    }

    /// Generate inline Gaussian elimination for M=3..=16.
    pub(super) fn generate_gauss_elim(code: &mut String, ir: &CircuitIR, dim: usize) {
        code.push_str(&format!(
            "        // Solve {dim}x{dim} system via inline Gaussian elimination\n"
        ));

        // Build augmented matrix [a | b] from Jacobian entries and residuals
        code.push_str("        let mut a = [\n");
        for i in 0..dim {
            let row = (0..dim)
                .map(|j| format!("j{i}_{j}"))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!("            [{row}],\n"));
        }
        code.push_str("        ];\n");

        let b_init = (0..dim)
            .map(|i| format!("f{i}"))
            .collect::<Vec<_>>()
            .join(", ");
        code.push_str(&format!("        let mut b = [{b_init}];\n"));

        // Forward elimination with partial pivoting
        code.push_str(&format!(
            "        let mut singular = false;\n\
             \x20       // Forward elimination with partial pivoting\n\
             \x20       for col in 0..{dim} {{\n\
             \x20           let mut max_row = col;\n\
             \x20           let mut max_val = a[col][col].abs();\n\
             \x20           for row in (col+1)..{dim} {{\n\
             \x20               if a[row][col].abs() > max_val {{\n\
             \x20                   max_val = a[row][col].abs();\n\
             \x20                   max_row = row;\n\
             \x20               }}\n\
             \x20           }}\n\
             \x20           if max_val < SINGULARITY_THRESHOLD {{ singular = true; break; }}\n\
             \x20           if max_row != col {{ a.swap(col, max_row); b.swap(col, max_row); }}\n\
             \x20           let pivot = a[col][col];\n\
             \x20           for row in (col+1)..{dim} {{\n\
             \x20               let factor = a[row][col] / pivot;\n\
             \x20               for j in (col+1)..{dim} {{ a[row][j] -= factor * a[col][j]; }}\n\
             \x20               b[row] -= factor * b[col];\n\
             \x20           }}\n\
             \x20       }}\n"
        ));

        // Back substitution
        code.push_str(&format!(
            "        if !singular {{\n\
             \x20           // Back substitution\n\
             \x20           for i in (0..{dim}).rev() {{\n\
             \x20               let mut sum = b[i];\n\
             \x20               for j in (i+1)..{dim} {{ sum -= a[i][j] * b[j]; }}\n\
             \x20               if a[i][i].abs() < SINGULARITY_THRESHOLD {{ singular = true; break; }}\n\
             \x20               b[i] = sum / a[i][i];\n\
             \x20           }}\n\
             \x20       }}\n"
        ));

        // Clamp and converge (alias b[i] as delta{i} for the shared helper)
        code.push_str("        if !singular {\n");
        for i in 0..dim {
            code.push_str(&format!("            let delta{i} = b[{i}];\n"));
        }
        emit_nr_limit_and_converge(code, ir, dim, "            ");
        code.push_str("        } else {\n");
        emit_nr_singular_fallback(code, dim, "            ");
        code.push_str("        }\n");
    }

    /// Generate Gauss elimination for Schur NR (uses `break` not `return`).
    pub(super) fn generate_schur_gauss_elim(code: &mut String, ir: &CircuitIR, dim: usize) {
        code.push_str(&format!(
            "        // Solve {dim}x{dim} system via inline Gaussian elimination\n"
        ));

        code.push_str("        let mut a = [\n");
        for i in 0..dim {
            let row = (0..dim)
                .map(|j| format!("j{i}_{j}"))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!("            [{row}],\n"));
        }
        code.push_str("        ];\n");

        let b_init = (0..dim)
            .map(|i| format!("f{i}"))
            .collect::<Vec<_>>()
            .join(", ");
        code.push_str(&format!("        let mut b = [{b_init}];\n"));

        code.push_str(&format!(
            "        let mut singular = false;\n\
             \x20       for col in 0..{dim} {{\n\
             \x20           let mut max_row = col;\n\
             \x20           let mut max_val = a[col][col].abs();\n\
             \x20           for row in (col+1)..{dim} {{\n\
             \x20               if a[row][col].abs() > max_val {{\n\
             \x20                   max_val = a[row][col].abs();\n\
             \x20                   max_row = row;\n\
             \x20               }}\n\
             \x20           }}\n\
             \x20           if max_val < 1e-15 {{ singular = true; break; }}\n\
             \x20           if max_row != col {{ a.swap(col, max_row); b.swap(col, max_row); }}\n\
             \x20           let pivot = a[col][col];\n\
             \x20           for row in (col+1)..{dim} {{\n\
             \x20               let factor = a[row][col] / pivot;\n\
             \x20               for j in (col+1)..{dim} {{ a[row][j] -= factor * a[col][j]; }}\n\
             \x20               b[row] -= factor * b[col];\n\
             \x20           }}\n\
             \x20       }}\n"
        ));

        code.push_str(&format!(
            "        if !singular {{\n\
             \x20           for i in (0..{dim}).rev() {{\n\
             \x20               let mut sum = b[i];\n\
             \x20               for j in (i+1)..{dim} {{ sum -= a[i][j] * b[j]; }}\n\
             \x20               if a[i][i].abs() < 1e-15 {{ singular = true; break; }}\n\
             \x20               b[i] = sum / a[i][i];\n\
             \x20           }}\n\
             \x20       }}\n"
        ));

        code.push_str("        if !singular {\n");
        for i in 0..dim {
            code.push_str(&format!("            let delta{i} = b[{i}];\n"));
        }
        emit_schur_nr_limit_and_converge(code, ir, dim, "            ", "state.k");
        code.push_str("        } else {\n");
        emit_nr_singular_fallback(code, dim, "            ");
        code.push_str("        }\n");
    }
}
