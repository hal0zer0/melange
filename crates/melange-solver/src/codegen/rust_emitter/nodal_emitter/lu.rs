//! Dense and sparse LU solve / factor / back-solve emitters and `invert_n`.

use super::equil::{build_growth_pattern, emit_equilibration, EquilPattern};
use crate::codegen::ir::{CircuitIR, LuOp};
use crate::codegen::rust_emitter::helpers::section_banner;
use crate::codegen::rust_emitter::RustEmitter;

impl RustEmitter {
    /// Emit `invert_n` function: runtime N×N matrix inversion via Gauss-Jordan.
    ///
    /// Used by `rebuild_matrices()` to recompute S = A^{-1} when sample rate or
    /// pot/switch values change.
    pub(super) fn emit_nodal_invert_n(_ir: &CircuitIR) -> String {
        let mut code = section_banner("MATRIX INVERSION (for rebuild_matrices)");

        code.push_str("/// Invert an N×N matrix via LU factorization with partial pivoting.\n");
        code.push_str("/// Returns None if the matrix is singular (pivot < 1e-30).\n");
        code.push_str("///\n");
        code.push_str("/// Called three times per `rebuild_matrices` on the nodal path\n");
        code.push_str("/// (S = A^-1, S_be = A_be^-1, S_sub = A_sub^-1), so factor-then-\n");
        code.push_str("/// solve uses ~half the work of the prior Gauss-Jordan on an\n");
        code.push_str("/// augmented [A | I] matrix and halves the stack footprint.\n");
        code.push_str("#[inline(never)]\n");
        code.push_str("fn invert_n(a: &[[f64; N]; N]) -> Option<[[f64; N]; N]> {\n");

        // LU factorization in place with partial pivoting.
        code.push_str("    let mut lu = *a;\n");
        code.push_str("    let mut perm = [0usize; N];\n");
        code.push_str("    for i in 0..N { perm[i] = i; }\n\n");

        code.push_str("    for k in 0..N {\n");
        code.push_str("        let mut max_row = k;\n");
        code.push_str("        let mut max_val = lu[k][k].abs();\n");
        code.push_str("        for i in (k + 1)..N {\n");
        code.push_str("            let v = lu[i][k].abs();\n");
        code.push_str("            if v > max_val { max_val = v; max_row = i; }\n");
        code.push_str("        }\n");
        code.push_str("        if max_val < 1e-30 { return None; }\n");
        code.push_str("        if max_row != k { lu.swap(k, max_row); perm.swap(k, max_row); }\n");
        code.push_str("        let pivot = lu[k][k];\n");
        code.push_str("        for i in (k + 1)..N {\n");
        code.push_str("            let m = lu[i][k] / pivot;\n");
        code.push_str("            lu[i][k] = m;\n");
        code.push_str("            for j in (k + 1)..N {\n");
        code.push_str("                lu[i][j] -= m * lu[k][j];\n");
        code.push_str("            }\n");
        code.push_str("        }\n");
        code.push_str("    }\n\n");

        // Identity-column back-solves.
        code.push_str("    let mut result = [[0.0f64; N]; N];\n");
        code.push_str("    for col in 0..N {\n");
        code.push_str("        let mut b = [0.0f64; N];\n");
        code.push_str("        let mut start = N;\n");
        code.push_str("        for i in 0..N {\n");
        code.push_str("            if perm[i] == col { b[i] = 1.0; start = i; break; }\n");
        code.push_str("        }\n");

        // Forward substitution (L unit lower triangular, skip leading zeros).
        code.push_str("        for i in (start + 1)..N {\n");
        code.push_str("            let mut sum = b[i];\n");
        code.push_str("            for j in start..i { sum -= lu[i][j] * b[j]; }\n");
        code.push_str("            b[i] = sum;\n");
        code.push_str("        }\n");

        // Backward substitution (U on and above diagonal).
        code.push_str("        for i in (0..N).rev() {\n");
        code.push_str("            let mut sum = b[i];\n");
        code.push_str("            for j in (i + 1)..N { sum -= lu[i][j] * b[j]; }\n");
        code.push_str("            let pivot = lu[i][i];\n");
        code.push_str("            if pivot.abs() < 1e-30 { return None; }\n");
        code.push_str("            b[i] = sum / pivot;\n");
        code.push_str("        }\n");

        code.push_str("        for i in 0..N { result[i][col] = b[i]; }\n");
        code.push_str("    }\n\n");

        code.push_str("    Some(result)\n");
        code.push_str("}\n\n");

        code
    }

    /// Emit LU solve function for the nodal solver (N x N with partial pivoting).
    /// Used by the full-LU codegen path (not Schur).
    pub(super) fn emit_nodal_lu_solve(_ir: &CircuitIR, pat: Option<&EquilPattern>) -> String {
        let mut code = section_banner(
            "LU SOLVE (Equilibrated Gaussian elimination with iterative refinement)",
        );

        code.push_str("/// Solve A*x = b using equilibrated LU with partial pivoting + iterative refinement.\n");
        code.push_str("///\n");
        code.push_str("/// Asymmetric row/column max-norm equilibration: rows are scaled by\n");
        code.push_str(
            "/// 1/max_j(|A[i][j]|), then columns by 1/max_i(|A[i][j]|), to reduce the\n",
        );
        code.push_str(
            "/// condition number. One round of iterative refinement corrects residual error.\n",
        );
        code.push_str(
            "/// Modifies `a` in place (LU factors). On success, `b` contains the solution.\n",
        );
        code.push_str("#[inline(always)]\n");
        code.push_str("fn lu_solve(a: &mut [[f64; N]; N], b: &mut [f64; N]) -> bool {\n");
        code.push_str("    // Save original A and b for iterative refinement\n");
        code.push_str("    let a_orig = *a;\n");
        code.push_str("    let b_orig = *b;\n\n");

        // Step 1: Asymmetric row/column max-norm equilibration
        code.push_str("    // Step 1: Asymmetric row/column equilibration\n");
        code.push_str("    let mut dr = [1.0f64; N];\n");
        code.push_str("    let mut dc = [1.0f64; N];\n");
        emit_equilibration(&mut code, pat, true);

        // Step 2: LU factorize with partial pivoting (stores L below diagonal, U on/above)
        code.push_str("    // Step 2: LU factorize with partial pivoting\n");
        code.push_str("    let mut perm = [0usize; N];\n");
        code.push_str("    for i in 0..N { perm[i] = i; }\n\n");

        code.push_str("    for col in 0..N {\n");
        code.push_str("        let mut max_row = col;\n");
        code.push_str("        let mut max_val = a[col][col].abs();\n");
        code.push_str("        for row in (col + 1)..N {\n");
        code.push_str("            if a[row][col].abs() > max_val {\n");
        code.push_str("                max_val = a[row][col].abs();\n");
        code.push_str("                max_row = row;\n");
        code.push_str("            }\n");
        code.push_str("        }\n");
        code.push_str("        if max_val < 1e-30 { return false; }\n");
        code.push_str("        if max_row != col {\n");
        code.push_str("            a.swap(col, max_row);\n");
        code.push_str("            perm.swap(col, max_row);\n");
        code.push_str("        }\n");
        code.push_str("        let pivot = a[col][col];\n");
        code.push_str("        for row in (col + 1)..N {\n");
        code.push_str("            let factor = a[row][col] / pivot;\n");
        code.push_str("            a[row][col] = factor; // Store L factor\n");
        code.push_str("            for j in (col + 1)..N {\n");
        code.push_str("                a[row][j] -= factor * a[col][j];\n");
        code.push_str("            }\n");
        code.push_str("        }\n");
        code.push_str("    }\n\n");

        // Step 3: Forward/backward substitution — solve LU * x_eq = Dr * P * b
        code.push_str("    // Step 3: Solve LU * x_eq = Dr * P * b\n");
        code.push_str("    let mut x = [0.0f64; N];\n");
        code.push_str("    for i in 0..N { x[i] = dr[perm[i]] * b_orig[perm[i]]; }\n\n");

        code.push_str("    // Forward substitution (L)\n");
        code.push_str("    for i in 1..N {\n");
        code.push_str("        let mut sum = 0.0;\n");
        code.push_str("        for j in 0..i { sum += a[i][j] * x[j]; }\n");
        code.push_str("        x[i] -= sum;\n");
        code.push_str("    }\n");
        code.push_str("    // Backward substitution (U)\n");
        code.push_str("    for i in (0..N).rev() {\n");
        code.push_str("        let mut sum = 0.0;\n");
        code.push_str("        for j in (i + 1)..N { sum += a[i][j] * x[j]; }\n");
        code.push_str("        if a[i][i].abs() < 1e-30 { return false; }\n");
        code.push_str("        x[i] = (x[i] - sum) / a[i][i];\n");
        code.push_str("    }\n\n");

        // Step 4: Iterative refinement — compute residual in equilibrated space, correct
        code.push_str("    // Step 4: Iterative refinement\n");
        code.push_str("    let mut r = [0.0f64; N];\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        let pi = perm[i];\n");
        code.push_str("        let mut ax_i = 0.0;\n");
        code.push_str("        for j in 0..N {\n");
        code.push_str("            ax_i += dr[pi] * a_orig[pi][j] * dc[j] * x[j];\n");
        code.push_str("        }\n");
        code.push_str("        r[i] = dr[pi] * b_orig[pi] - ax_i;\n");
        code.push_str("    }\n");
        code.push_str("    // Solve LU * dx = r\n");
        code.push_str("    for i in 1..N {\n");
        code.push_str("        let mut sum = 0.0;\n");
        code.push_str("        for j in 0..i { sum += a[i][j] * r[j]; }\n");
        code.push_str("        r[i] -= sum;\n");
        code.push_str("    }\n");
        code.push_str("    for i in (0..N).rev() {\n");
        code.push_str("        let mut sum = 0.0;\n");
        code.push_str("        for j in (i + 1)..N { sum += a[i][j] * r[j]; }\n");
        code.push_str("        r[i] = (r[i] - sum) / a[i][i];\n");
        code.push_str("    }\n\n");

        // Step 5: Apply correction and undo equilibration (column scaling)
        code.push_str("    // Step 5: Apply correction and undo equilibration (column scaling)\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        b[i] = dc[i] * (x[i] + r[i]);\n");
        code.push_str("    }\n\n");

        code.push_str("    true\n");
        code.push_str("}\n\n");

        code
    }

    /// Emit LU factorization function (chord method: factor once, solve many times).
    ///
    /// Equilibrates, then LU-factorizes with partial pivoting. The factored matrix,
    /// scaling vector `d`, and permutation `perm` are stored for repeated back-solves.
    pub(super) fn emit_nodal_lu_factor(_ir: &CircuitIR, pat: Option<&EquilPattern>) -> String {
        let mut code = String::new();

        code.push_str(
            "/// LU factorization with asymmetric row/column equilibration and partial pivoting.\n",
        );
        code.push_str("///\n");
        code.push_str(
            "/// After this call, `a` contains the LU factors, `dr`/`dc` the row/column\n",
        );
        code.push_str(
            "/// equilibration, and `perm` the row permutation. Use `lu_back_solve` to solve.\n",
        );
        code.push_str("#[inline(always)]\n");
        code.push_str("fn lu_factor(a: &mut [[f64; N]; N], dr: &mut [f64; N], dc: &mut [f64; N], perm: &mut [usize; N]) -> bool {\n");

        // Step 1: Asymmetric row/column equilibration
        emit_equilibration(&mut code, pat, false);

        // Step 2: LU factorize with partial pivoting
        code.push_str("    // LU factorize with partial pivoting\n");
        code.push_str("    for i in 0..N { perm[i] = i; }\n");
        code.push_str("    for col in 0..N {\n");
        code.push_str("        let mut max_row = col;\n");
        code.push_str("        let mut max_val = a[col][col].abs();\n");
        code.push_str("        for row in (col + 1)..N {\n");
        code.push_str("            if a[row][col].abs() > max_val {\n");
        code.push_str("                max_val = a[row][col].abs();\n");
        code.push_str("                max_row = row;\n");
        code.push_str("            }\n");
        code.push_str("        }\n");
        code.push_str("        if max_val < 1e-30 { return false; }\n");
        code.push_str("        if max_row != col {\n");
        code.push_str("            a.swap(col, max_row);\n");
        code.push_str("            perm.swap(col, max_row);\n");
        code.push_str("        }\n");
        code.push_str("        let pivot = a[col][col];\n");
        code.push_str("        for row in (col + 1)..N {\n");
        code.push_str("            let factor = a[row][col] / pivot;\n");
        code.push_str("            a[row][col] = factor;\n");
        code.push_str("            for j in (col + 1)..N {\n");
        code.push_str("                a[row][j] -= factor * a[col][j];\n");
        code.push_str("            }\n");
        code.push_str("        }\n");
        code.push_str("    }\n");
        code.push_str("    true\n");
        code.push_str("}\n\n");

        code
    }

    /// Emit forward/backward substitution using pre-factored LU (chord method).
    ///
    /// O(N²) per call — no factorization. Used on NR iterations 1+ where the
    /// Jacobian is reused from iteration 0 (chord / modified Newton-Raphson).
    pub(super) fn emit_nodal_lu_back_solve(_ir: &CircuitIR) -> String {
        let mut code = String::new();

        code.push_str(
            "/// Solve using pre-factored LU: forward/backward substitution + de-equilibrate.\n",
        );
        code.push_str("///\n");
        code.push_str(
            "/// `a_lu` contains LU factors from `lu_factor`. `dr`/`dc` and `perm` are the\n",
        );
        code.push_str("/// equilibration and permutation from the same call. On return, `b`\n");
        code.push_str("/// contains the solution. O(N²) — no iterative refinement.\n");
        code.push_str("#[inline(always)]\n");
        code.push_str("fn lu_back_solve(a_lu: &[[f64; N]; N], dr: &[f64; N], dc: &[f64; N], perm: &[usize; N], b: &mut [f64; N]) {\n");

        // Apply permutation + row equilibration scaling to RHS
        code.push_str("    let mut x = [0.0f64; N];\n");
        code.push_str("    for i in 0..N { x[i] = dr[perm[i]] * b[perm[i]]; }\n\n");

        // Forward substitution (L)
        code.push_str("    // Forward substitution (L)\n");
        code.push_str("    for i in 1..N {\n");
        code.push_str("        let mut sum = 0.0;\n");
        code.push_str("        for j in 0..i { sum += a_lu[i][j] * x[j]; }\n");
        code.push_str("        x[i] -= sum;\n");
        code.push_str("    }\n");

        // Backward substitution (U)
        code.push_str("    // Backward substitution (U)\n");
        code.push_str("    for i in (0..N).rev() {\n");
        code.push_str("        let mut sum = 0.0;\n");
        code.push_str("        for j in (i + 1)..N { sum += a_lu[i][j] * x[j]; }\n");
        code.push_str("        x[i] = (x[i] - sum) / a_lu[i][i];\n");
        code.push_str("    }\n\n");

        // De-equilibrate (column scaling)
        code.push_str("    // De-equilibrate (column scaling)\n");
        code.push_str("    for i in 0..N { b[i] = dc[i] * x[i]; }\n");
        code.push_str("}\n\n");

        code
    }

    /// Emit compile-time sparse LU factorization (straight-line code, no loops).
    ///
    /// Uses the pre-computed AMD ordering and symbolic elimination schedule from
    /// `LuSparsity`. Each operation is one line of generated code, in-place on
    /// original indices — AMD ordering determines the ORDER of elimination, not
    /// the physical layout, and there are no permutation arrays.
    ///
    /// Pivoting is STATIC (fixed at codegen time), so the emitted function
    /// guards itself at runtime: it returns `false` on a numerically-tiny pivot
    /// or when the post-factorization growth check trips (max |entry| of the
    /// factored matrix > 1e8; the input is equilibrated to unit max-norm, so
    /// that ratio IS the element growth factor). The caller then re-factors the
    /// same stamped G_aug densely with partial pivoting (see the `chord_dense`
    /// fallback in `emit_nodal_process_sample`).
    pub(super) fn emit_sparse_lu_factor(ir: &CircuitIR, pat: Option<&EquilPattern>) -> String {
        let lu = match &ir.sparsity.lu {
            Some(lu) => lu,
            None => return String::new(),
        };
        let n = lu.n;

        let mut code = String::new();
        code.push_str("/// Sparse LU factorization (compile-time unrolled, original indices).\n");
        code.push_str("///\n");
        code.push_str(&format!(
            "/// {} FLOPs (vs ~{} dense). AMD fill-reducing ordering.\n",
            lu.factor_flops,
            n * n * n / 3
        ));
        code.push_str(
            "/// Static (symbolic) pivoting. Returns false when a pivot is numerically\n",
        );
        code.push_str(
            "/// too small OR the growth-factor check trips (max |factored entry| > 1e8\n",
        );
        code.push_str("/// against the equilibrated unit max-norm input); the caller re-factors\n");
        code.push_str("/// the same stamped matrix with dense partial-pivoting lu_factor.\n");
        code.push_str("#[inline(always)]\n");
        code.push_str("fn sparse_lu_factor(a: &mut [[f64; N]; N], dr: &mut [f64; N], dc: &mut [f64; N]) -> bool {\n");

        // Step 1: Asymmetric row/column equilibration
        emit_equilibration(&mut code, pat, false);

        // Static row swaps (for zero diagonals, determined at codegen time)
        if !lu.row_swaps.is_empty() {
            code.push_str("    // Static row swaps for zero diagonals\n");
            for &(r1, r2) in &lu.row_swaps {
                code.push_str(&format!("    a.swap({r1}, {r2});\n"));
            }
            code.push('\n');
        }

        // Sparse elimination — straight-line code in AMD order, original indices
        code.push_str(&format!(
            "    // Sparse Gaussian elimination: {} ops in AMD order (unrolled)\n",
            lu.ops.len()
        ));
        for op in &lu.ops {
            match op {
                LuOp::DivPivot { row, col } => {
                    code.push_str(&format!(
                        "    if a[{col}][{col}].abs() < 1e-30 {{ return false; }}\n"
                    ));
                    code.push_str(&format!("    a[{row}][{col}] /= a[{col}][{col}];\n"));
                }
                LuOp::SubMul { row, col, j } => {
                    code.push_str(&format!(
                        "    a[{row}][{j}] -= a[{row}][{col}] * a[{col}][{j}];\n"
                    ));
                }
            }
        }

        // Growth-factor check. Equilibration bounded the pre-factor matrix to
        // unit max-norm, so max |entry| of the factored matrix directly measures
        // element growth under the static pivot order. Dense partial pivoting
        // bounds growth; static pivoting does not — a knee-crossing device
        // Jacobian can make a symbolically-fine pivot numerically tiny and blow
        // the factors up. Reject so the caller re-factors densely.
        code.push_str("\n    // Growth-factor check (pre-factor matrix has unit max-norm)\n");
        match pat {
            // Sparse sweep over the factored possibly-nonzero set. Byte-identical
            // to the dense N×N sweep: excluded positions are structural zeros.
            // Gated on the same `EquilPattern` that guards the sparse
            // equilibration, so the two stay consistent (and small/dense circuits
            // that keep the dense equilibration keep the dense growth sweep, which
            // vectorizes and wins there).
            Some(p) => {
                let growth_pat = build_growth_pattern(p, &lu.row_swaps, &lu.ops);
                let entries = growth_pat
                    .iter()
                    .map(|(i, j)| format!("({i},{j})"))
                    .collect::<Vec<_>>()
                    .join(", ");
                code.push_str(&format!(
                    "    const GROWTH_PAT: [(u16, u16); {}] = [{}];\n",
                    growth_pat.len(),
                    entries
                ));
                code.push_str("    let mut growth = 0.0f64;\n");
                code.push_str("    for &(i, j) in GROWTH_PAT.iter() {\n");
                code.push_str("        let v = a[i as usize][j as usize].abs();\n");
                code.push_str("        if v > growth { growth = v; }\n");
                code.push_str("    }\n");
            }
            None => {
                code.push_str("    let mut growth = 0.0f64;\n");
                code.push_str("    for i in 0..N {\n");
                code.push_str(
                    "        for j in 0..N { let v = a[i][j].abs(); if v > growth { growth = v; } }\n",
                );
                code.push_str("    }\n");
            }
        }
        code.push_str("    if !growth.is_finite() || growth > 1e8 { return false; }\n");

        code.push_str("\n    true\n");
        code.push_str("}\n\n");

        code
    }

    /// Emit compile-time sparse forward/backward substitution (original indices).
    ///
    /// Forward sub processes L entries in AMD elimination order.
    /// Backward sub processes U entries in reverse AMD order.
    /// No permutation arrays — order is baked into the emitted code.
    pub(super) fn emit_sparse_lu_back_solve(ir: &CircuitIR) -> String {
        let lu = match &ir.sparsity.lu {
            Some(lu) => lu,
            None => return String::new(),
        };
        let n = lu.n;

        let mut code = String::new();
        code.push_str(
            "/// Sparse forward/backward substitution (compile-time unrolled, original indices).\n",
        );
        code.push_str("///\n");
        code.push_str(&format!(
            "/// {} FLOPs (vs ~{} dense).\n",
            lu.solve_flops,
            n * n * 2
        ));
        code.push_str("#[inline(always)]\n");
        code.push_str(
            "fn sparse_lu_back_solve(a_lu: &[[f64; N]; N], dr: &[f64; N], dc: &[f64; N], b: &mut [f64; N]) {\n",
        );

        // Equilibrate RHS (row scaling) + apply static row swaps
        code.push_str("    let mut x = [0.0f64; N];\n");
        code.push_str("    for i in 0..N { x[i] = dr[i] * b[i]; }\n");
        if !lu.row_swaps.is_empty() {
            code.push_str("    // Static row swaps (matching factorization)\n");
            for &(r1, r2) in &lu.row_swaps {
                code.push_str(&format!("    x.swap({r1}, {r2});\n"));
            }
        }
        code.push('\n');

        // Sparse forward substitution (L entries in AMD order)
        // L entries are already ordered by elimination step in l_nnz
        code.push_str("    // Sparse forward substitution (L, AMD order)\n");
        for &(row, col) in &lu.l_nnz {
            code.push_str(&format!("    x[{row}] -= a_lu[{row}][{col}] * x[{col}];\n"));
        }

        // Sparse backward substitution (U entries in reverse AMD order)
        code.push_str("\n    // Sparse backward substitution (U, reverse AMD order)\n");
        // Group U entries by pivot (row) and process in reverse elimination order
        {
            let mut u_by_pivot: std::collections::HashMap<usize, Vec<usize>> =
                std::collections::HashMap::new();
            for &(row, col) in &lu.u_nnz {
                if col != row {
                    u_by_pivot.entry(row).or_default().push(col);
                }
            }
            // Process pivots in reverse AMD order
            for &pivot in lu.elim_order.iter().rev() {
                if let Some(cols) = u_by_pivot.get(&pivot) {
                    for &c in cols {
                        code.push_str(&format!("    x[{pivot}] -= a_lu[{pivot}][{c}] * x[{c}];\n"));
                    }
                }
                code.push_str(&format!("    x[{pivot}] /= a_lu[{pivot}][{pivot}];\n"));
            }
        }

        // De-equilibrate (column scaling — no column permutation to undo, we're in original space)
        code.push_str("\n    // De-equilibrate (column scaling)\n");
        code.push_str("    for i in 0..N { b[i] = dc[i] * x[i]; }\n");
        code.push_str("}\n\n");

        code
    }
}
