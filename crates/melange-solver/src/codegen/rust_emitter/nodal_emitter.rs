//! Nodal solver code emission methods.
//!
//! Contains the nodal entry point (`emit_nodal`) and all nodal-specific
//! methods: constants, state, LU solve/factor/back-solve, sparse LU,
//! Schur and full-LU process_sample, active-set resolve, device evaluation,
//! and voltage limiting.

use crate::codegen::ir::{CircuitIR, DeviceParams, DeviceType, LuOp};
use crate::codegen::CodegenError;
use super::RustEmitter;
use super::helpers::{
    fmt_f64, format_matrix_rows, section_banner, oversampling_info,
    pentode_dispatch, emit_pentode_nr_dk_stamp,
    self_heating_device_data, device_param_template_data,
};
use super::nr_helpers::{
    emit_nr_singular_fallback, emit_schur_nr_limit_and_converge,
};

// ============================================================================
// Sparse N_V / N_I helpers
// ============================================================================

/// Emit a sparse `result = N_V[row] * vec` product.
///
/// Instead of `for j in 0..N { result += N_V[row][j] * vec[j]; }`,
/// emits only the nonzero terms: `N_V[row][c1] * vec[c1] + N_V[row][c2] * vec[c2]`.
/// N_V typically has 2 nonzeros per row (±1 at device nodes), so this is ~28x faster at N=57.
fn emit_sparse_nv_dot(ir: &CircuitIR, row: usize, result_var: &str, vec_var: &str, indent: &str) -> String {
    let nz = &ir.sparsity.n_v.nz_by_row;
    if row < nz.len() && !nz[row].is_empty() {
        let terms: Vec<String> = nz[row]
            .iter()
            .map(|&col| format!("N_V[{}][{}] * {}[{}]", row, col, vec_var, col))
            .collect();
        format!("{indent}let {result_var} = {};\n", terms.join(" + "))
    } else {
        format!("{indent}let {result_var} = 0.0;\n")
    }
}

/// Emit sparse `v_nl[i] = sum_j N_V[i][j] * vec[j]` for all M rows.
fn emit_sparse_nv_matvec(ir: &CircuitIR, result_arr: &str, vec_var: &str, indent: &str) -> String {
    let m = ir.topology.m;
    let mut code = String::new();
    for i in 0..m {
        let nz = &ir.sparsity.n_v.nz_by_row;
        if i < nz.len() && !nz[i].is_empty() {
            let terms: Vec<String> = nz[i]
                .iter()
                .map(|&col| format!("N_V[{}][{}] * {}[{}]", i, col, vec_var, col))
                .collect();
            code.push_str(&format!("{indent}{result_arr}[{i}] = {};\n", terms.join(" + ")));
        }
    }
    code
}

/// Emit sparse `rhs[i] += sum_j N_I[i][j] * vec[j]` for all N rows.
fn emit_sparse_ni_matvec_add(ir: &CircuitIR, result_arr: &str, vec_var: &str, indent: &str) -> String {
    let n = ir.topology.n;
    let mut code = String::new();
    let nz = &ir.sparsity.n_i.nz_by_row;
    for i in 0..n {
        if i < nz.len() && !nz[i].is_empty() {
            let terms: Vec<String> = nz[i]
                .iter()
                .map(|&col| format!("N_I[{}][{}] * {}[{}]", i, col, vec_var, col))
                .collect();
            code.push_str(&format!("{indent}{result_arr}[{i}] += {};\n", terms.join(" + ")));
        }
    }
    code
}

// ============================================================================
// Nodal solver emission (full N×N NR per sample, LU solve per iteration)
// ============================================================================

impl RustEmitter {
    /// Emit complete generated code for the nodal solver path.
    ///
    /// The nodal path differs from DK in that it does full N-dimensional
    /// Newton-Raphson with LU factorization per iteration, instead of
    /// precomputing S=A^{-1} and doing M-dimensional NR.
    ///
    /// Shared with DK: header, device models, SPICE limiting, safe_exp.
    /// Nodal-specific: constants, state, process_sample, LU solve, set_sample_rate.
    pub(super) fn emit_nodal(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = String::new();

        // Compute use_full_nodal flag FIRST — needed by emit_nodal_state for hot/cold split.
        let m = ir.topology.m;
        let has_positive_k_with_current = if m > 0 {
            (0..m).any(|i| {
                let k_ii = ir.matrices.k[i * m + i];
                if k_ii <= 0.0 {
                    return false;
                }
                // Only flag positive K for dimensions with actual current injection
                // (VCA control ports have zero N_i column — positive K there is harmless)
                ir.sparsity.n_i.nz_by_row.iter().any(|row| row.contains(&i))
            })
        } else {
            false
        };
        let k_diag_min = if m > 0 {
            (0..m)
                .map(|i| ir.matrices.k[i * m + i])
                .fold(0.0_f64, f64::min)
        } else {
            0.0
        };
        // Check for degenerate K (all near-zero). When K ≈ 0, the Schur NR
        // has J = I (identity) — no device feedback. The device Jacobian that
        // provides essential damping for high-gain active elements (Boyle op-amps)
        // is invisible to the Schur decomposition. The full N×N LU NR must be
        // used instead, as it builds G_aug = A - N_I*J_dev*N_V each iteration.
        let k_max_abs = if m > 0 {
            (0..m * m)
                .map(|i| ir.matrices.k[i].abs())
                .fold(0.0_f64, f64::max)
        } else {
            0.0
        };
        let k_degenerate = m > 0 && k_max_abs < 1e-6;
        // The spectral radius of S*A_neg measures LINEAR prediction stability.
        // When rho < 1.0, the Schur NR's nonlinear correction (S*N_i*i_nl)
        // damps the marginal mode — the DK path uses identical matrices and
        // works fine. When rho > 1.0, the linear prediction genuinely amplifies
        // errors. With well-conditioned K (negative diagonal, not degenerate),
        // the Schur NR still handles rho slightly above 1.0 (up to ~1.002,
        // matching the DK auto-BE threshold). Only route to full-LU when K
        // itself is pathological OR spectral radius indicates true instability
        // that even the Schur NR can't damp.
        //
        // K magnitude check: circuits where nonlinear devices are the sole
        // current path between nodes (no parallel resistors) produce K entries
        // spanning many orders of magnitude (e.g. 5×10^11 for a transistor
        // ladder filter). The Schur NR forms J = I - J_dev*K; when |K| is
        // extreme, J_dev*K >> I and the 16×16 Gauss elimination is hopelessly
        // ill-conditioned. The full LU NR avoids K entirely by stamping device
        // Jacobians into G_aug directly.
        let k_ill_conditioned = m > 0 && k_max_abs > 1e8;
        // S matrix check: nodes connected only through caps and device junctions
        // (no resistive path) produce extreme S = A^{-1} entries. This is
        // invariant to FA reduction — FA changes N_V/N_I/K but not A/S. When
        // S has entries > 1e6, the Schur prediction v = S*rhs amplifies roundoff
        // into the nonlinear solver, producing garbage regardless of K magnitude.
        let n = ir.topology.n;
        let s_max_abs = if n > 0 && !ir.matrices.s.is_empty() {
            ir.matrices.s.iter().map(|v| v.abs()).fold(0.0_f64, f64::max)
        } else {
            0.0
        };
        let s_ill_conditioned = s_max_abs > 1e6;

        // Linearized device bypass: when triodes/BJTs are linearized at DC OP,
        // their small-signal conductances (gm, 1/rp) stamp into G, creating
        // high-gain coupling chains. These inflate S = A^{-1}, K = N_V*S*N_I,
        // and K diagonals (both negative and positive) — but the values
        // represent correct high-gain transfer functions, not numerical
        // instability. The Schur NR iterates in M-space using K directly and
        // converges in ~4 iterations regardless of chain length. The full-LU
        // NR redundantly re-solves the constant linear partition every
        // iteration, degrading to ~39 iterations for long chains.
        //
        // When linearized devices are present, suppress ALL magnitude-based
        // guards (positive K, extreme K diagonal, K ill-conditioned, S
        // ill-conditioned) and rely solely on:
        //   1. k_degenerate (K ≈ 0 makes Schur trivial: J = I, no damping)
        //   2. spectral radius (rho > 1.0 means linear prediction unstable)
        let has_linearized = ir.topology.num_linearized_devices > 0;
        let linearized_bypass = has_linearized && !k_degenerate;
        if linearized_bypass {
            log::warn!(
                "Nodal: {} linearized devices — suppressing magnitude guards \
                 (max|K|={:.2e}, max|S|={:.2e}, K_diag_min={:.2e}, \
                 pos_K_with_I={}). Spectral radius = {:.4}.",
                ir.topology.num_linearized_devices, k_max_abs, s_max_abs,
                k_diag_min, has_positive_k_with_current,
                ir.matrices.spectral_radius_s_aneg
            );
        }

        let k_well_conditioned = m > 0
            && !k_degenerate
            && !has_positive_k_with_current
            && !k_ill_conditioned
            && !s_ill_conditioned
            && k_diag_min > -1e12;
        // Spectral radius thresholds:
        // - k_well_conditioned: 1.002 (DK auto-BE threshold, tight)
        // - linearized_bypass: 1.05 (relaxed — the Schur NR corrects the
        //   linear prediction every sample via M-dim NR, and the runtime
        //   BE fallback catches any sample where trapezoidal diverges.
        //   At rho=1.01, the prediction error is ~1%/sample — trivially
        //   corrected by the first NR iteration)
        // - pathological K (no bypass): 1.0 (strict — unknown K structure)
        let schur_unstable = if k_well_conditioned {
            ir.matrices.spectral_radius_s_aneg > 1.002
        } else if linearized_bypass {
            ir.matrices.spectral_radius_s_aneg > 1.05
        } else {
            ir.matrices.spectral_radius_s_aneg > 1.0
        };
        let use_full_nodal = if linearized_bypass {
            // Only k_degenerate and spectral radius can block Schur
            k_degenerate || schur_unstable
        } else {
            has_positive_k_with_current
                || k_diag_min < -1e12
                || k_degenerate
                || k_ill_conditioned
                || s_ill_conditioned
                || schur_unstable
        };
        // The dense `lu_solve` helper is emitted whenever any generated code
        // path needs it. The full-LU nodal path always needs it. The Schur
        // path also needs it when op-amp rail handling is in `ActiveSet` mode,
        // because the post-NR constrained resolve does one dense LU solve per
        // clamp-active sample. Without this gate the Schur path would
        // reference an undefined function and fail to compile.
        use crate::codegen::OpampRailMode;
        let needs_lu_solve = use_full_nodal
            || (matches!(
                    ir.solver_config.opamp_rail_mode,
                    OpampRailMode::ActiveSet | OpampRailMode::ActiveSetBe
                )
                && !ir.opamps.is_empty());

        // Now emit header, constants, device models, state (needs use_full_nodal)
        code.push_str(&self.emit_header(ir)?);
        code.push_str(&self.emit_nodal_constants(ir));
        code.push_str(&self.emit_device_models(ir)?);
        code.push_str(&self.emit_nodal_state(ir, use_full_nodal));
        code.push_str(&Self::emit_nodal_invert_n(ir));

        if use_full_nodal {
            if has_positive_k_with_current {
                log::warn!(
                    "Nodal: using full N×N LU NR (positive K diagonal with current injection)"
                );
            } else if k_degenerate {
                log::warn!(
                    "Nodal: using full N×N LU NR (K degenerate, max|K|={:.2e} — device Jacobian provides essential damping)",
                    k_max_abs
                );
            } else if k_ill_conditioned {
                log::warn!(
                    "Nodal: using full N×N LU NR (max|K|={:.2e}, extreme K magnitude — device nodes lack resistive paths)",
                    k_max_abs
                );
            } else if s_ill_conditioned {
                log::warn!(
                    "Nodal: using full N×N LU NR (max|S|={:.2e}, cap-only nodes lack resistive paths — Schur prediction unreliable)",
                    s_max_abs
                );
            } else if schur_unstable {
                log::warn!(
                    "Nodal: using full N×N LU NR (spectral_radius(S*A_neg) = {:.4}, Schur feedback unstable)",
                    ir.matrices.spectral_radius_s_aneg
                );
            } else {
                log::warn!(
                    "Nodal: using full N×N LU NR (K_diag_min={:.1}, ill-conditioned)",
                    k_diag_min
                );
            }
            code.push_str(&Self::emit_nodal_lu_solve(ir));
            code.push_str(&Self::emit_nodal_lu_factor(ir));
            code.push_str(&Self::emit_nodal_lu_back_solve(ir));
            // Sparse LU when available (dense kept as fallback)
            if ir.sparsity.lu.is_some() {
                code.push_str(&Self::emit_sparse_lu_factor(ir));
                code.push_str(&Self::emit_sparse_lu_back_solve(ir));
            }
            code.push_str(&Self::emit_nodal_process_sample(ir));
        } else {
            if linearized_bypass {
                log::warn!(
                    "Nodal: using Schur NR (M={}, {} linearized devices, rho={:.4})",
                    m, ir.topology.num_linearized_devices,
                    ir.matrices.spectral_radius_s_aneg
                );
            }
            // Schur path doesn't emit lu_solve by default, but active-set
            // resolve needs it. Emit it on demand.
            if needs_lu_solve {
                code.push_str(&Self::emit_nodal_lu_solve(ir));
            }
            code.push_str(&Self::emit_nodal_schur_process_sample(ir)?);
        }

        if ir.solver_config.oversampling_factor > 1 {
            code.push_str(&Self::emit_oversampler(ir));
        }

        Ok(code)
    }

    /// Emit constants section for nodal solver.
    ///
    /// Includes A, A_neg, A_be, A_neg_be, N_v, N_i, G, C, RHS_CONST, RHS_CONST_BE,
    /// DC_OP, DC_NL_I, and topology dimensions. No S, K, or S_NI.
    pub(super) fn emit_nodal_constants(&self, ir: &CircuitIR) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let n_aug = ir.topology.n_aug;
        let num_outputs = ir.solver_config.output_nodes.len();

        let mut code = section_banner("CONSTANTS: Compile-time circuit topology (Nodal solver)");

        // Dimension constants
        code.push_str(
            "/// Number of augmented system nodes (including VS/VCVS/inductor branch variables)\n",
        );
        code.push_str(&format!("pub const N: usize = {};\n\n", n));
        code.push_str("/// Number of original circuit nodes (excluding ground)\n");
        code.push_str(&format!("pub const N_NODES: usize = {};\n\n", n_nodes));
        code.push_str("/// Boundary between VS/VCVS rows and inductor branch variables\n");
        code.push_str(&format!("pub const N_AUG: usize = {};\n\n", n_aug));
        code.push_str("/// Total nonlinear dimension (sum of device dimensions)\n");
        code.push_str(&format!("pub const M: usize = {};\n\n", m));
        code.push_str("/// Maximum NR iterations per sample\n");
        code.push_str(&format!(
            "pub const MAX_ITER: usize = {};\n\n",
            ir.solver_config.max_iterations
        ));
        code.push_str(
            "/// Chord method: re-factor Jacobian every N iterations (full LU path only).\n",
        );
        code.push_str(
            "/// Iter 0 always factors. Between refactors, O(N²) back-solve reuses stored LU.\n",
        );
        code.push_str(
            "/// Must be odd — even values can cause refactoring/convergence resonance.\n",
        );
        code.push_str("pub const CHORD_REFACTOR: usize = 5;\n\n");

        code.push_str("/// NR convergence tolerance (VNTOL)\n");
        code.push_str(&format!(
            "pub const TOL: f64 = {};\n\n",
            fmt_f64(ir.solver_config.tolerance)
        ));

        // Sample rate
        code.push_str(&format!(
            "/// Default sample rate (Hz) used at code generation time.\npub const SAMPLE_RATE: f64 = {:.1};\n\n",
            ir.solver_config.sample_rate
        ));
        code.push_str(&format!(
            "/// Oversampling factor (1 = none, 2 = 2x, 4 = 4x)\npub const OVERSAMPLING_FACTOR: usize = {};\n",
            ir.solver_config.oversampling_factor
        ));
        if ir.solver_config.oversampling_factor > 1 {
            let internal_rate =
                ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
            code.push_str(&format!(
                "/// Internal sample rate = SAMPLE_RATE * OVERSAMPLING_FACTOR\npub const INTERNAL_SAMPLE_RATE: f64 = {:.1};\n",
                internal_rate
            ));
        }
        code.push('\n');

        // I/O configuration
        code.push_str(&format!(
            "/// Input node index\npub const INPUT_NODE: usize = {};\n\n",
            ir.solver_config.input_node
        ));
        code.push_str(&format!(
            "/// Number of output channels\npub const NUM_OUTPUTS: usize = {};\n\n",
            num_outputs
        ));
        let output_nodes_values = ir
            .solver_config
            .output_nodes
            .iter()
            .map(|n| n.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        code.push_str(&format!(
            "/// Output node indices (one per output channel)\npub const OUTPUT_NODES: [usize; NUM_OUTPUTS] = [{}];\n\n",
            output_nodes_values
        ));
        let output_scales_values = ir
            .solver_config
            .output_scales
            .iter()
            .map(|s| fmt_f64(*s))
            .collect::<Vec<_>>()
            .join(", ");
        code.push_str(&format!(
            "/// Output scale factors (applied after DC blocking)\npub const OUTPUT_SCALES: [f64; NUM_OUTPUTS] = [{}];\n\n",
            output_scales_values
        ));
        code.push_str(&format!(
            "/// Input resistance (Thevenin equivalent)\npub const INPUT_RESISTANCE: f64 = {};\n\n",
            fmt_f64(ir.solver_config.input_resistance)
        ));

        // IIR op-amp constants (per-op-amp with GBW dominant pole)
        //
        // Bilinear-transformed dominant pole (PURE EXPLICIT formulation):
        //   y[n] = a1*y[n-1] + b0*(x[n-1] + x[n-1]) = a1*y[n-1] + 2*b0*x[n-1]
        //   a1 = (alpha*C_dom - Go) / (alpha*C_dom + Go)
        //   b0 = Gm / (alpha*C_dom + Go)
        //
        // Gm is NOT in G — it has been stripped in ir.rs. The entire VCCS current
        // is injected in RHS using only previous-sample state (y_prev, x_prev):
        //   y_new = a1*y_prev + 2*b0*x_prev          (computed BEFORE solve)
        //   rhs[o] += Go*(y_new + y_prev)            (trapezoidal VCCS injection)
        //
        // DC gain check: y_ss*(1-a1) = 2*b0*x_ss  ⇒  y_ss = (Gm/Go)*x_ss = AOL*x_ss ✓
        //
        // After solve, save x_new = v[np] - v[nm] from the converged solution and
        // advance y_prev = y_new (pre-computed value, before it was consumed by RHS).
        let internal_rate = ir.solver_config.sample_rate
            * ir.solver_config.oversampling_factor as f64;
        for (idx, oa_iir) in ir.opamp_iir.iter().enumerate() {
            let alpha_oa = 2.0 * internal_rate;
            let denom = alpha_oa * oa_iir.c_dom + oa_iir.go;
            let a1 = (alpha_oa * oa_iir.c_dom - oa_iir.go) / denom;
            let b0 = oa_iir.gm / denom;
            code.push_str(&format!(
                "/// IIR op-amp {idx}: Gm={:.4}, Go={:.4}, C_dom={:.4e} (pure explicit)\n",
                oa_iir.gm, oa_iir.go, oa_iir.c_dom
            ));
            code.push_str(&format!(
                "const OA{idx}_GM: f64 = {:.17e};\n",
                oa_iir.gm
            ));
            code.push_str(&format!(
                "const OA{idx}_GO: f64 = {:.17e};\n",
                oa_iir.go
            ));
            code.push_str(&format!(
                "const OA{idx}_C_DOM: f64 = {:.17e};\n",
                oa_iir.c_dom
            ));
            code.push_str(&format!(
                "const OA{idx}_A1_DEFAULT: f64 = {:.17e};\n",
                a1
            ));
            code.push_str(&format!(
                "const OA{idx}_B0_DEFAULT: f64 = {:.17e};\n",
                b0
            ));
            code.push_str(&format!(
                "const OA{idx}_OUT: usize = {};\n",
                oa_iir.out_idx
            ));
            if let Some(vhi) = Some(oa_iir.vclamp_hi).filter(|v| v.is_finite()) {
                code.push_str(&format!(
                    "const OA{idx}_VCC: f64 = {:.17e};\n", vhi
                ));
            }
            if let Some(vlo) = Some(oa_iir.vclamp_lo).filter(|v| v.is_finite()) {
                code.push_str(&format!(
                    "const OA{idx}_VEE: f64 = {:.17e};\n", vlo
                ));
            }
            code.push('\n');
        }

        // Op-amp slew-rate constants (V/s). Emitted for every op-amp in
        // `ir.opamps` whose `.model OA(SR=…)` was finite. The SR constant
        // is consumed by `emit_opamp_slew_limit` (called from the main
        // process_sample path right before state.v_prev = v).
        // Index matches the enumerate index over `ir.opamps` — NOT the
        // filtered slice — so the `OA{idx}_SR` name aligns with the
        // `emit_opamp_slew_limit` helper's indexing.
        for (idx, oa) in ir.opamps.iter().enumerate() {
            if oa.sr.is_finite() {
                code.push_str(&format!(
                    "/// Op-amp {idx} slew rate (V/s). Parsed from .model OA(SR=…) in V/μs.\n\
                     const OA{idx}_SR: f64 = {:.17e};\n\n",
                    oa.sr
                ));
            }
        }

        // G and C matrices (sample-rate independent, Gm-stripped for IIR op-amps)
        code.push_str("/// G matrix: conductance matrix (sample-rate independent, Gm stripped for IIR op-amps)\nconst G: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.g(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        code.push_str("/// C matrix: capacitance matrix (sample-rate independent)\nconst C: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.c(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // A = G + alpha*C (trapezoidal forward matrix)
        code.push_str("/// Default A matrix: A = G + (2/T)*C (trapezoidal, at SAMPLE_RATE)\nconst A_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.a_matrix(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // A_neg = alpha*C - G (trapezoidal history matrix)
        code.push_str("/// Default A_neg matrix: A_neg = (2/T)*C - G (trapezoidal history, at SAMPLE_RATE)\nconst A_NEG_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.a_neg(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // A_be = G + (1/T)*C (backward Euler forward matrix)
        code.push_str("/// Default A_be matrix: A_be = G + (1/T)*C (backward Euler, at SAMPLE_RATE)\nconst A_BE_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.a_matrix_be(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // A_neg_be = (1/T)*C (backward Euler history matrix)
        code.push_str("/// Default A_neg_be matrix: (1/T)*C (backward Euler history, at SAMPLE_RATE)\nconst A_NEG_BE_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.a_neg_be(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // N_v: voltage extraction matrix (M × N)
        code.push_str("/// N_v matrix: extracts controlling voltages from node voltages (M x N)\npub const N_V: [[f64; N]; M] = [\n");
        for row in format_matrix_rows(m, n, |i, j| ir.n_v(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // N_i: current injection matrix (N × M), matching runtime layout
        code.push_str("/// N_i matrix: maps nonlinear currents to node injections (N x M)\npub const N_I: [[f64; M]; N] = [\n");
        for row in format_matrix_rows(n, m, |i, j| ir.n_i(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // Schur complement matrices: S = A^{-1}, K = N_v * S * N_i, S_NI = S * N_i
        code.push_str("/// S matrix: A^{-1} (precomputed inverse, trapezoidal, at SAMPLE_RATE)\nconst S_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.s(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        if m > 0 {
            code.push_str("/// K matrix: N_v * S * N_i (nonlinear kernel, trapezoidal, at SAMPLE_RATE)\nconst K_DEFAULT: [[f64; M]; M] = [\n");
            for row in format_matrix_rows(m, m, |i, j| ir.k(i, j)) {
                code.push_str(&format!("    [{}],\n", row));
            }
            code.push_str("];\n\n");

            // S_NI = S * N_i (N × M)
            code.push_str("/// S_NI matrix: S * N_i (precomputed for final voltage recovery, N x M)\nconst S_NI_DEFAULT: [[f64; M]; N] = [\n");
            for i in 0..n {
                let row: Vec<String> = (0..m)
                    .map(|j| {
                        let mut val = 0.0;
                        for k in 0..n {
                            val += ir.s(i, k) * ir.n_i(k, j);
                        }
                        fmt_f64(val)
                    })
                    .collect();
                code.push_str(&format!("    [{}],\n", row.join(", ")));
            }
            code.push_str("];\n\n");
        }

        // Backward Euler Schur complement matrices
        if !ir.matrices.s_be.is_empty() {
            code.push_str("/// S_be matrix: A_be^{-1} (backward Euler, at SAMPLE_RATE)\nconst S_BE_DEFAULT: [[f64; N]; N] = [\n");
            for row in format_matrix_rows(n, n, |i, j| ir.s_be(i, j)) {
                code.push_str(&format!("    [{}],\n", row));
            }
            code.push_str("];\n\n");

            if m > 0 && !ir.matrices.k_be.is_empty() {
                code.push_str("/// K_be matrix: N_v * S_be * N_i (backward Euler kernel)\nconst K_BE_DEFAULT: [[f64; M]; M] = [\n");
                for row in format_matrix_rows(m, m, |i, j| ir.k_be(i, j)) {
                    code.push_str(&format!("    [{}],\n", row));
                }
                code.push_str("];\n\n");

                // S_NI_be = S_be * N_i (N × M)
                code.push_str("/// S_NI_be matrix: S_be * N_i (backward Euler, for final voltage recovery)\nconst S_NI_BE_DEFAULT: [[f64; M]; N] = [\n");
                for i in 0..n {
                    let row: Vec<String> = (0..m)
                        .map(|j| {
                            let mut val = 0.0;
                            for k in 0..n {
                                val += ir.s_be(i, k) * ir.n_i(k, j);
                            }
                            fmt_f64(val)
                        })
                        .collect();
                    code.push_str(&format!("    [{}],\n", row.join(", ")));
                }
                code.push_str("];\n\n");
            }
        }

        // Sub-step matrices (trap at 2× internal rate for ActiveSetBe sub-stepping)
        if !ir.matrices.s_sub.is_empty() {
            code.push_str("/// S_sub matrix: (G + 4C/T)^{-1} (trap at 2× rate, for sub-stepping)\nconst S_SUB_DEFAULT: [[f64; N]; N] = [\n");
            for row in format_matrix_rows(n, n, |i, j| ir.s_sub(i, j)) {
                code.push_str(&format!("    [{}],\n", row));
            }
            code.push_str("];\n\n");

            code.push_str("/// A_neg_sub matrix: 4C/T - G (trap at 2× rate history)\nconst A_NEG_SUB_DEFAULT: [[f64; N]; N] = [\n");
            for row in format_matrix_rows(n, n, |i, j| ir.a_neg_sub(i, j)) {
                code.push_str(&format!("    [{}],\n", row));
            }
            code.push_str("];\n\n");

            if m > 0 && !ir.matrices.k_sub.is_empty() {
                code.push_str("/// K_sub matrix: N_v * S_sub * N_i (sub-step kernel)\nconst K_SUB_DEFAULT: [[f64; M]; M] = [\n");
                for row in format_matrix_rows(m, m, |i, j| ir.k_sub(i, j)) {
                    code.push_str(&format!("    [{}],\n", row));
                }
                code.push_str("];\n\n");

                // S_NI_sub = S_sub * N_i (N × M)
                code.push_str("/// S_NI_sub matrix: S_sub * N_i (sub-step voltage recovery)\nconst S_NI_SUB_DEFAULT: [[f64; M]; N] = [\n");
                for i in 0..n {
                    let row: Vec<String> = (0..m)
                        .map(|j| {
                            let mut val = 0.0;
                            for k in 0..n {
                                val += ir.s_sub(i, k) * ir.n_i(k, j);
                            }
                            fmt_f64(val)
                        })
                        .collect();
                    code.push_str(&format!("    [{}],\n", row.join(", ")));
                }
                code.push_str("];\n\n");
            }
        }

        // RHS_CONST (trapezoidal)
        if ir.has_dc_sources {
            let rhs_const_values = (0..n)
                .map(|i| fmt_f64(ir.matrices.rhs_const[i]))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// RHS constant contribution from DC sources (trapezoidal: node rows x2, VS rows x1)\npub const RHS_CONST: [f64; N] = [{}];\n\n",
                rhs_const_values
            ));
        }

        // RHS_CONST_BE (backward Euler)
        if ir.has_dc_sources && !ir.matrices.rhs_const_be.is_empty() {
            let rhs_const_be_values = (0..n)
                .map(|i| {
                    if i < ir.matrices.rhs_const_be.len() {
                        fmt_f64(ir.matrices.rhs_const_be[i])
                    } else {
                        fmt_f64(0.0)
                    }
                })
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// RHS constant contribution from DC sources (backward Euler: all rows x1)\npub const RHS_CONST_BE: [f64; N] = [{}];\n\n",
                rhs_const_be_values
            ));
        }

        // DC blocking coefficient
        if ir.dc_block {
            let internal_rate =
                ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
            let dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / internal_rate;
            code.push_str(&format!(
                "/// DC blocking filter coefficient: R = 1 - 2*pi*fc/sr (5Hz cutoff at internal rate)\npub const DC_BLOCK_R: f64 = {:.17e};\n\n",
                dc_block_r
            ));
        }

        // DC OP convergence flag
        code.push_str(&format!(
            "/// Whether the nonlinear DC OP solver converged at codegen time\npub const DC_OP_CONVERGED: bool = {};\n\n",
            ir.dc_op_converged
        ));

        // Potentiometer constants
        for (idx, pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "pub const POT_{}_NODE_P: usize = {};\n\
                 pub const POT_{}_NODE_Q: usize = {};\n\
                 pub const POT_{}_G_NOM: f64 = {:.17e};\n\
                 pub const POT_{}_MIN_R: f64 = {:.17e};\n\
                 pub const POT_{}_MAX_R: f64 = {:.17e};\n\n",
                idx,
                pot.node_p,
                idx,
                pot.node_q,
                idx,
                pot.g_nominal,
                idx,
                pot.min_resistance,
                idx,
                pot.max_resistance
            ));
        }

        // Saturating inductor constants
        for (idx, si) in ir.saturating_inductors.iter().enumerate() {
            code.push_str(&format!(
                "/// Saturating inductor {idx}: {} (L0={:.4e} H, Isat={:.4e} A)\n\
                 pub const SAT_IND_{idx}_L0: f64 = {l0:.17e};\n\
                 pub const SAT_IND_{idx}_ISAT: f64 = {isat:.17e};\n\
                 pub const SAT_IND_{idx}_AUG_ROW: usize = {row};\n\n",
                si.name,
                si.l0,
                si.isat,
                l0 = si.l0,
                isat = si.isat,
                row = si.aug_row,
            ));
        }
        let num_sat_ind = ir.saturating_inductors.len();
        let has_any_saturation = !ir.saturating_inductors.is_empty()
            || !ir.saturating_coupled.is_empty()
            || !ir.saturating_xfmr_groups.is_empty();
        if num_sat_ind > 0 {
            code.push_str(&format!(
                "pub const NUM_SAT_IND: usize = {};\n\n",
                num_sat_ind
            ));
        }
        if has_any_saturation {
            code.push_str(
                "/// Saturation update interval (samples). L_eff recomputed every N samples.\n\
                 /// Saturation follows the signal envelope (~ms timescale), not individual samples.\n\
                 pub const SAT_UPDATE_INTERVAL: u32 = 32;\n\
                 /// Full matrix rebuild every N SM updates to eliminate accumulated drift.\n\
                 /// O(N²) SM between resyncs, O(N³) rebuild at resync. 16 keeps drift negligible.\n\
                 pub const SAT_RESYNC_INTERVAL: u32 = 16;\n\n",
            );
        }

        // Saturating coupled inductor constants
        for (idx, sc) in ir.saturating_coupled.iter().enumerate() {
            code.push_str(&format!(
                "/// Saturating coupled pair {idx}: {name} ({l1}+{l2}, κ={k:.4})\n\
                 pub const SAT_CI_{idx}_L1_L0: f64 = {l1_l0:.17e};\n\
                 pub const SAT_CI_{idx}_L2_L0: f64 = {l2_l0:.17e};\n\
                 pub const SAT_CI_{idx}_L1_ISAT: f64 = {l1_isat:.17e};\n\
                 pub const SAT_CI_{idx}_L2_ISAT: f64 = {l2_isat:.17e};\n\
                 pub const SAT_CI_{idx}_COUPLING: f64 = {coupling:.17e};\n\
                 pub const SAT_CI_{idx}_K1: usize = {k1};\n\
                 pub const SAT_CI_{idx}_K2: usize = {k2};\n\n",
                name = sc.name,
                l1 = sc.l1_name,
                l2 = sc.l2_name,
                k = sc.coupling,
                l1_l0 = sc.l1_l0,
                l2_l0 = sc.l2_l0,
                l1_isat = sc.l1_isat,
                l2_isat = sc.l2_isat,
                coupling = sc.coupling,
                k1 = sc.k1,
                k2 = sc.k2,
            ));
        }

        // Saturating transformer group constants
        for (idx, sg) in ir.saturating_xfmr_groups.iter().enumerate() {
            let w = sg.num_windings;
            let l0_str: Vec<String> = sg.l0s.iter().map(|v| format!("{:.17e}", v)).collect();
            let isat_str: Vec<String> = sg.isats.iter().map(|v| format!("{:.17e}", v)).collect();
            let row_str: Vec<String> = sg.aug_rows.iter().map(|v| format!("{}", v)).collect();
            let kappa_str: Vec<String> = sg.coupling_flat.iter().map(|v| format!("{:.17e}", v)).collect();
            code.push_str(&format!(
                "/// Saturating transformer group {idx}: {} ({w} windings)\n\
                 pub const SAT_XG_{idx}_W: usize = {w};\n\
                 pub const SAT_XG_{idx}_L0: [f64; {w}] = [{}];\n\
                 pub const SAT_XG_{idx}_ISAT: [f64; {w}] = [{}];\n\
                 pub const SAT_XG_{idx}_ROWS: [usize; {w}] = [{}];\n\
                 pub const SAT_XG_{idx}_KAPPA: [f64; {}] = [{}];\n\n",
                sg.name,
                l0_str.join(", "),
                isat_str.join(", "),
                row_str.join(", "),
                w * w,
                kappa_str.join(", "),
            ));
        }

        // Switch constants (position values)
        for (idx, sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!(
                "pub const SWITCH_{}_NUM_POSITIONS: usize = {};\n",
                idx, sw.num_positions
            ));
            for (ci, comp) in sw.components.iter().enumerate() {
                let values: Vec<String> = sw.positions.iter().map(|pos| fmt_f64(pos[ci])).collect();
                code.push_str(&format!(
                    "pub const SWITCH_{}_COMP_{}_VALUES: [f64; {}] = [{}];\n\
                     pub const SWITCH_{}_COMP_{}_TYPE: char = '{}';\n\
                     pub const SWITCH_{}_COMP_{}_NODE_P: usize = {};\n\
                     pub const SWITCH_{}_COMP_{}_NODE_Q: usize = {};\n\
                     pub const SWITCH_{}_COMP_{}_NOM: f64 = {:.17e};\n",
                    idx,
                    ci,
                    sw.num_positions,
                    values.join(", "),
                    idx,
                    ci,
                    comp.component_type,
                    idx,
                    ci,
                    comp.node_p,
                    idx,
                    ci,
                    comp.node_q,
                    idx,
                    ci,
                    comp.nominal_value,
                ));
            }
            code.push('\n');
        }

        code
    }

    /// Emit state struct, Default impl, set_sample_rate, and reset for nodal solver.
    pub(super) fn emit_nodal_state(&self, ir: &CircuitIR, use_full_nodal: bool) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let n_aug = ir.topology.n_aug;
        let num_outputs = ir.solver_config.output_nodes.len();
        let has_pots = !ir.pots.is_empty();
        let has_switches = !ir.switches.is_empty();
        let has_sat_ind = !ir.saturating_inductors.is_empty();
        let has_sat_coupled = !ir.saturating_coupled.is_empty() || !ir.saturating_xfmr_groups.is_empty();
        let has_any_saturation = has_sat_ind || has_sat_coupled;

        let mut code = section_banner("STATE STRUCTURE (Nodal solver)");

        // DC OP constant
        let has_dc_op = ir.has_dc_op;
        if has_dc_op {
            let dc_op_values = ir
                .dc_operating_point
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// DC operating point: steady-state node voltages\npub const DC_OP: [f64; N] = [{}];\n\n",
                dc_op_values
            ));
        }

        // DC NL currents
        let has_dc_nl = m > 0
            && !ir.dc_nl_currents.is_empty()
            && ir.dc_nl_currents.iter().any(|&v| v.abs() > 1e-30);
        if has_dc_nl {
            let dc_nl_i_values = ir
                .dc_nl_currents
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// DC operating point: nonlinear device currents at bias point\npub const DC_NL_I: [f64; M] = [{}];\n\n",
                dc_nl_i_values
            ));
        }

        // State struct
        code.push_str("/// Circuit state for one processing channel (nodal solver).\n");
        code.push_str("///\n");
        code.push_str("/// Contains per-sample state and sample-rate-dependent matrices.\n");
        code.push_str(
            "/// Call [`set_sample_rate`](CircuitState::set_sample_rate) before processing\n",
        );
        code.push_str("/// if the host sample rate differs from [`SAMPLE_RATE`].\n");
        // When using full-LU nodal path, emit a separate cold struct for
        // Schur complement + work matrices. These are only accessed on
        // pot/switch/sample-rate changes, not per sample. Keeping them
        // heap-allocated via Box prevents them from polluting L2 cache.
        if use_full_nodal {
            code.push_str("/// Cold state: matrices only accessed on pot/switch/sample-rate changes.\n");
            code.push_str("/// Heap-allocated to keep the per-sample hot path in L2 cache.\n");
            code.push_str("#[derive(Clone, Debug)]\n");
            code.push_str("pub struct CircuitStateCold {\n");
            code.push_str("    pub s: [[f64; N]; N],\n");
            if m > 0 {
                code.push_str("    pub k: [[f64; M]; M],\n");
                code.push_str("    pub s_ni: [[f64; M]; N],\n");
            }
            code.push_str("    pub s_be: [[f64; N]; N],\n");
            if m > 0 {
                code.push_str("    pub k_be: [[f64; M]; M],\n");
                code.push_str("    pub s_ni_be: [[f64; M]; N],\n");
            }
            if !ir.matrices.s_sub.is_empty() {
                code.push_str("    pub s_sub: [[f64; N]; N],\n");
                code.push_str("    pub a_neg_sub: [[f64; N]; N],\n");
                if m > 0 {
                    code.push_str("    pub k_sub: [[f64; M]; M],\n");
                    code.push_str("    pub s_ni_sub: [[f64; M]; N],\n");
                }
            }
            if has_pots || has_switches || has_sat_ind || has_sat_coupled {
                code.push_str("    pub g_work: [[f64; N]; N],\n");
                code.push_str("    pub c_work: [[f64; N]; N],\n");
            }
            code.push_str("}\n\n");
        }

        code.push_str("#[derive(Clone, Debug)]\n");
        code.push_str("pub struct CircuitState {\n");
        code.push_str("    /// Previous node voltages v[n-1]\n");
        code.push_str("    pub v_prev: [f64; N],\n\n");
        code.push_str("    /// Previous nonlinear currents i_nl[n-1]\n");
        code.push_str("    pub i_nl_prev: [f64; M],\n\n");
        code.push_str(
            "    /// Nonlinear currents from two samples ago i_nl[n-2] (for NR predictor)\n",
        );
        code.push_str("    pub i_nl_prev_prev: [f64; M],\n\n");
        code.push_str("    /// DC operating point (for reset/sleep/wake)\n");
        code.push_str("    pub dc_operating_point: [f64; N],\n\n");
        code.push_str("    /// Previous input sample for trapezoidal integration\n");
        code.push_str("    pub input_prev: f64,\n\n");
        code.push_str("    /// NR convergence diagnostic from the last sample's solve.\n");
        code.push_str("    ///\n");
        code.push_str("    /// Semantics:\n");
        code.push_str("    /// - 0..MAX_ITER-1: 0-indexed iteration at which convergence was detected\n");
        code.push_str("    ///   (i.e. `last_nr_iterations + 1` iterations actually ran)\n");
        code.push_str("    /// - MAX_ITER: loop exhausted without convergence → NR failed\n");
        code.push_str("    ///\n");
        code.push_str("    /// The check `last_nr_iterations >= MAX_ITER` is the BE-fallback trigger;\n");
        code.push_str("    /// storing `iter` (not `iter + 1`) is intentional so that convergence at\n");
        code.push_str("    /// the final permitted iteration (iter == MAX_ITER - 1) does not false-trigger.\n");
        code.push_str("    pub last_nr_iterations: u32,\n\n");
        if ir.dc_block {
            code.push_str("    /// DC blocking filter: previous input samples (one per output)\n");
            code.push_str("    pub dc_block_x_prev: [f64; NUM_OUTPUTS],\n");
            code.push_str("    /// DC blocking filter: previous output samples (one per output)\n");
            code.push_str("    pub dc_block_y_prev: [f64; NUM_OUTPUTS],\n");
            code.push_str(
                "    /// DC blocking filter coefficient (recomputed on sample rate change)\n",
            );
            code.push_str("    pub dc_block_r: f64,\n\n");
        }
        code.push_str("    /// Diagnostic: peak absolute output (pre-clamp)\n");
        code.push_str("    pub diag_peak_output: f64,\n");
        code.push_str("    /// Diagnostic: number of times output exceeded +/-10V\n");
        code.push_str("    pub diag_clamp_count: u64,\n");
        code.push_str("    /// Diagnostic: number of times NR hit max iterations\n");
        code.push_str("    pub diag_nr_max_iter_count: u64,\n");
        code.push_str("    /// Diagnostic: number of backward Euler fallback activations\n");
        code.push_str("    pub diag_be_fallback_count: u64,\n");
        code.push_str("    /// Diagnostic: number of times NaN triggered state reset\n");
        code.push_str("    pub diag_nan_reset_count: u64,\n");
        code.push_str("    /// Diagnostic: number of samples that needed adaptive sub-stepping\n");
        code.push_str("    pub diag_substep_count: u64,\n");
        code.push_str("    /// Diagnostic: number of LU refactorizations performed\n");
        code.push_str("    pub diag_refactor_count: u64,\n");
        code.push_str("    /// Diagnostic: number of samples hit by the global voltage-damping\n");
        code.push_str("    /// safety net. This is a legacy safeguard that scales v_new toward\n");
        code.push_str("    /// v_prev when any node moves more than ~2V (or 5% of max DC OP) in\n");
        code.push_str("    /// one sample. Per CLAUDE.md, output limiting must never mask solver\n");
        code.push_str("    /// bugs — treat a nonzero count here as a signal that the solver needs\n");
        code.push_str("    /// investigation, not as an acceptable steady-state.\n");
        code.push_str("    pub diag_voltage_damp_count: u64,\n\n");

        // Cross-timestep chord state (persisted LU for full LU path)
        if m > 0 {
            code.push_str(
                "    // --- Cross-timestep chord method state (persisted LU factors) ---\n",
            );
            code.push_str(
                "    /// LU-factored Jacobian from previous convergence (chord method)\n",
            );
            code.push_str("    pub chord_lu: [[f64; N]; N],\n");
            code.push_str("    /// Row equilibration scaling from LU factorization\n");
            code.push_str("    pub chord_dr: [f64; N],\n");
            code.push_str("    /// Column equilibration scaling from LU factorization\n");
            code.push_str("    pub chord_dc: [f64; N],\n");
            code.push_str("    /// Row permutation from LU factorization\n");
            code.push_str("    pub chord_perm: [usize; N],\n");
            code.push_str("    /// Device Jacobian consistent with chord_lu (for companion RHS)\n");
            code.push_str("    pub chord_j_dev: [f64; M * M],\n");
            code.push_str(
                "    /// Whether chord LU factors are valid (false until first convergence)\n",
            );
            code.push_str("    pub chord_valid: bool,\n\n");
        }

        code.push_str(
            "    /// A matrix: G + alpha*C (trapezoidal), recomputed by set_sample_rate\n",
        );
        code.push_str("    pub a: [[f64; N]; N],\n");
        code.push_str("    /// A_neg matrix: alpha*C - G (trapezoidal history), recomputed by set_sample_rate\n");
        code.push_str("    pub a_neg: [[f64; N]; N],\n");
        code.push_str(
            "    /// A_be matrix: G + (1/T)*C (backward Euler), recomputed by set_sample_rate\n",
        );
        code.push_str("    pub a_be: [[f64; N]; N],\n");
        code.push_str("    /// A_neg_be matrix: (1/T)*C (backward Euler history), recomputed by set_sample_rate\n");
        code.push_str("    pub a_neg_be: [[f64; N]; N],\n");

        // Schur complement matrices + work matrices.
        // When use_full_nodal, these are in a heap-allocated CircuitStateCold
        // to keep the hot per-sample working set in L2 cache.
        if use_full_nodal {
            code.push_str("    /// Cold state: Schur complement + work matrices (heap-allocated,\n");
            code.push_str("    /// only accessed on pot/switch changes and sample rate changes).\n");
            code.push_str("    pub cold: Box<CircuitStateCold>,\n");
        } else {
            // Schur path: all matrices inline (small N, no cache pressure)
            code.push_str("    /// S matrix: A^{-1} (trapezoidal), recomputed by set_sample_rate\n");
            code.push_str("    pub s: [[f64; N]; N],\n");
            if m > 0 {
                code.push_str("    /// K matrix: N_v * S * N_i (nonlinear kernel), recomputed by set_sample_rate\n");
                code.push_str("    pub k: [[f64; M]; M],\n");
                code.push_str(
                    "    /// S_NI matrix: S * N_i (voltage recovery), recomputed by set_sample_rate\n",
                );
                code.push_str("    pub s_ni: [[f64; M]; N],\n");
            }
            code.push_str(
                "    /// S_be matrix: A_be^{-1} (backward Euler), recomputed by set_sample_rate\n",
            );
            code.push_str("    pub s_be: [[f64; N]; N],\n");
            if m > 0 {
                code.push_str("    /// K_be matrix: N_v * S_be * N_i (BE kernel), recomputed by set_sample_rate\n");
                code.push_str("    pub k_be: [[f64; M]; M],\n");
                code.push_str("    /// S_NI_be matrix: S_be * N_i (BE voltage recovery), recomputed by set_sample_rate\n");
                code.push_str("    pub s_ni_be: [[f64; M]; N],\n");
            }
            if !ir.matrices.s_sub.is_empty() {
                code.push_str("    /// S_sub matrix: (G+4C/T)^{-1} (trap at 2× rate), recomputed by set_sample_rate\n");
                code.push_str("    pub s_sub: [[f64; N]; N],\n");
                code.push_str("    /// A_neg_sub matrix: 4C/T-G (trap at 2× rate history), recomputed by set_sample_rate\n");
                code.push_str("    pub a_neg_sub: [[f64; N]; N],\n");
                if m > 0 {
                    code.push_str("    /// K_sub matrix: N_v*S_sub*N_i (sub-step kernel)\n");
                    code.push_str("    pub k_sub: [[f64; M]; M],\n");
                    code.push_str("    /// S_NI_sub matrix: S_sub*N_i (sub-step voltage recovery)\n");
                    code.push_str("    pub s_ni_sub: [[f64; M]; N],\n");
                }
            }
        }
        code.push('\n');

        // Mutable G and C for pot/switch/saturating-inductor re-stamping
        if has_pots || has_switches || has_sat_ind || has_sat_coupled {
            if use_full_nodal {
                // g_work/c_work are cold (only used in rebuild_matrices, not per-sample).
                // current_sample_rate and matrices_dirty are tiny scalars, stay hot.
                code.push_str("    /// Current sample rate (for rebuild_matrices)\n");
                code.push_str("    pub current_sample_rate: f64,\n");
                code.push_str("    /// Lazy rebuild flag: set by set_pot/set_switch, cleared by process_sample\n");
                code.push_str("    pub matrices_dirty: bool,\n\n");
            } else {
                code.push_str("    /// Working G matrix (modified by pots/switches)\n");
                code.push_str("    pub g_work: [[f64; N]; N],\n");
                code.push_str("    /// Working C matrix (modified by switches/saturating inductors)\n");
                code.push_str("    pub c_work: [[f64; N]; N],\n");
                code.push_str("    /// Current sample rate (for rebuild_matrices)\n");
                code.push_str("    pub current_sample_rate: f64,\n");
                code.push_str("    /// Lazy rebuild flag: set by set_pot/set_switch, cleared by process_sample\n");
                code.push_str("    pub matrices_dirty: bool,\n\n");
            }
        }

        // Pot state fields
        for (idx, _pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "    /// Potentiometer {}: current resistance (ohms)\n\
                 \x20   pub pot_{}_resistance: f64,\n\
                 \x20   /// Potentiometer {}: previous timestep resistance (trapezoidal A_neg)\n\
                 \x20   pub pot_{}_resistance_prev: f64,\n",
                idx, idx, idx, idx
            ));
        }
        if has_pots {
            code.push('\n');
        }

        // Switch state fields
        for (idx, _sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!(
                "    /// Switch {}: current position (0-indexed)\n\
                 \x20   pub switch_{}_position: usize,\n",
                idx, idx
            ));
        }
        if has_switches {
            code.push('\n');
        }

        // Saturation decimation counter
        if has_any_saturation {
            code.push_str("    /// Decimation counter for saturation updates (counts down to 0)\n");
            code.push_str("    pub sat_update_counter: u32,\n");
            code.push_str("    /// SM update counter; triggers full rebuild at SAT_RESYNC_INTERVAL\n");
            code.push_str("    pub sat_resync_counter: u32,\n\n");
        }

        // Saturating inductor state fields
        for (idx, si) in ir.saturating_inductors.iter().enumerate() {
            code.push_str(&format!(
                "    /// Saturating inductor {idx} ({}): current effective inductance\n\
                 \x20   pub sat_ind_{idx}_l_eff: f64,\n",
                si.name,
            ));
        }
        if !ir.saturating_inductors.is_empty() {
            code.push('\n');
        }

        // Saturating coupled inductor state fields
        for (idx, sc) in ir.saturating_coupled.iter().enumerate() {
            code.push_str(&format!(
                "    /// Saturating coupled pair {idx} ({name}): effective inductances and mutual\n\
                 \x20   pub sat_ci_{idx}_l1_eff: f64,\n\
                 \x20   pub sat_ci_{idx}_l2_eff: f64,\n\
                 \x20   pub sat_ci_{idx}_m_eff: f64,\n",
                name = sc.name,
            ));
        }
        if !ir.saturating_coupled.is_empty() {
            code.push('\n');
        }

        // Saturating transformer group state fields
        for (idx, sg) in ir.saturating_xfmr_groups.iter().enumerate() {
            let w = sg.num_windings;
            code.push_str(&format!(
                "    /// Saturating transformer group {idx} ({}): effective L per winding\n\
                 \x20   pub sat_xg_{idx}_l_eff: [f64; {w}],\n\
                 \x20   /// Effective mutual inductance matrix (flat W×W)\n\
                 \x20   pub sat_xg_{idx}_m_eff: [f64; {}],\n",
                sg.name,
                w * w,
            ));
        }
        if !ir.saturating_xfmr_groups.is_empty() {
            code.push('\n');
        }

        // IIR op-amp state fields
        if !ir.opamp_iir.is_empty() {
            code.push_str("    // --- IIR op-amp dominant pole filter state ---\n");
            for (idx, _oa_iir) in ir.opamp_iir.iter().enumerate() {
                code.push_str(&format!(
                    "    pub oa{idx}_y_prev: f64,\n\
                     \x20   pub oa{idx}_x_prev: f64,\n\
                     \x20   pub oa{idx}_a1: f64,\n\
                     \x20   pub oa{idx}_b0: f64,\n"
                ));
            }
            code.push('\n');
        }

        // Device parameter state fields (runtime-adjustable)
        let device_params = device_param_template_data(ir);
        if !device_params.is_empty() {
            code.push_str("\n    // --- Runtime-adjustable device parameters ---\n");
            for dev in &device_params {
                for p in &dev.params {
                    code.push_str(&format!(
                        "    /// Device {} {} ({}) — runtime adjustable\n",
                        dev.dev_num, p.const_suffix, dev.device_type
                    ));
                    code.push_str(&format!(
                        "    pub device_{}_{}: f64,\n",
                        dev.dev_num, p.field_suffix
                    ));
                }
            }
        }

        // BJT self-heating thermal state
        let thermal_devices = self_heating_device_data(ir);
        if !thermal_devices.is_empty() {
            code.push_str("\n    // --- BJT self-heating thermal state ---\n");
            for td in &thermal_devices {
                code.push_str(&format!(
                    "    /// BJT {} junction temperature [K]\n\
                     \x20   pub device_{}_tj: f64,\n",
                    td.dev_num, td.dev_num
                ));
            }
        }

        // Oversampling state
        let os_factor = ir.solver_config.oversampling_factor;
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "\n    /// Oversampler upsampling half-band filter state (single input)\n\
                     \x20   pub os_up_state: [f64; {}],\n\
                     \x20   /// Oversampler downsampling half-band filter state (per output)\n\
                     \x20   pub os_dn_state: [[f64; {}]; NUM_OUTPUTS],\n",
                os_info.state_size, os_info.state_size
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "    /// 4x oversampler outer upsampling filter state\n\
                         \x20   pub os_up_state_outer: [f64; {}],\n\
                         \x20   /// 4x oversampler outer downsampling filter state\n\
                         \x20   pub os_dn_state_outer: [[f64; {}]; NUM_OUTPUTS],\n",
                    os_info.state_size_outer, os_info.state_size_outer
                ));
            }
        }

        code.push_str("}\n\n");

        // Default impl
        code.push_str("impl Default for CircuitState {\n");
        code.push_str("    fn default() -> Self {\n");
        code.push_str("        let mut state = Self {\n");
        if has_dc_op {
            code.push_str("            v_prev: DC_OP,\n");
        } else {
            code.push_str("            v_prev: [0.0; N],\n");
        }
        if has_dc_nl {
            code.push_str("            i_nl_prev: DC_NL_I,\n");
            code.push_str("            i_nl_prev_prev: DC_NL_I,\n");
        } else {
            code.push_str("            i_nl_prev: [0.0; M],\n");
            code.push_str("            i_nl_prev_prev: [0.0; M],\n");
        }
        if has_dc_op {
            code.push_str("            dc_operating_point: DC_OP,\n");
        } else {
            code.push_str("            dc_operating_point: [0.0; N],\n");
        }
        code.push_str("            input_prev: 0.0,\n");
        code.push_str("            last_nr_iterations: 0,\n");
        // Initialize DC blocking filter from DC OP so first sample sees zero delta
        if ir.dc_block {
            let output_nodes = &ir.solver_config.output_nodes;
            let dc_x: Vec<String> = output_nodes
                .iter()
                .map(|&node| {
                    if has_dc_op && node < ir.dc_operating_point.len() {
                        fmt_f64(ir.dc_operating_point[node])
                    } else {
                        "0.0".to_string()
                    }
                })
                .collect();
            code.push_str(&format!(
                "            dc_block_x_prev: [{}],\n",
                dc_x.join(", ")
            ));
            code.push_str(&format!(
                "            dc_block_y_prev: [{}],\n",
                vec!["0.0"; output_nodes.len()].join(", ")
            ));
            code.push_str("            dc_block_r: DC_BLOCK_R,\n");
        }
        code.push_str("            diag_peak_output: 0.0,\n");
        code.push_str("            diag_clamp_count: 0,\n");
        code.push_str("            diag_nr_max_iter_count: 0,\n");
        code.push_str("            diag_be_fallback_count: 0,\n");
        code.push_str("            diag_nan_reset_count: 0,\n");
        code.push_str("            diag_substep_count: 0,\n");
        code.push_str("            diag_refactor_count: 0,\n");
        code.push_str("            diag_voltage_damp_count: 0,\n");
        if m > 0 {
            code.push_str("            chord_lu: [[0.0; N]; N],\n");
            code.push_str("            chord_dr: [1.0; N],\n");
            code.push_str("            chord_dc: [1.0; N],\n");
            code.push_str("            chord_perm: {{ let mut p = [0usize; N]; let mut i = 0; while i < N { p[i] = i; i += 1; } p }},\n");
            code.push_str("            chord_j_dev: [0.0; M * M],\n");
            code.push_str("            chord_valid: false,\n");
        }
        code.push_str("            a: A_DEFAULT,\n");
        code.push_str("            a_neg: A_NEG_DEFAULT,\n");
        code.push_str("            a_be: A_BE_DEFAULT,\n");
        code.push_str("            a_neg_be: A_NEG_BE_DEFAULT,\n");
        if use_full_nodal {
            // Cold fields go into Box<CircuitStateCold>
            code.push_str("            cold: Box::new(CircuitStateCold {\n");
            code.push_str("                s: S_DEFAULT,\n");
            if m > 0 {
                code.push_str("                k: K_DEFAULT,\n");
                code.push_str("                s_ni: S_NI_DEFAULT,\n");
            }
            code.push_str("                s_be: S_BE_DEFAULT,\n");
            if m > 0 {
                code.push_str("                k_be: K_BE_DEFAULT,\n");
                code.push_str("                s_ni_be: S_NI_BE_DEFAULT,\n");
            }
            if !ir.matrices.s_sub.is_empty() {
                code.push_str("                s_sub: S_SUB_DEFAULT,\n");
                code.push_str("                a_neg_sub: A_NEG_SUB_DEFAULT,\n");
                if m > 0 {
                    code.push_str("                k_sub: K_SUB_DEFAULT,\n");
                    code.push_str("                s_ni_sub: S_NI_SUB_DEFAULT,\n");
                }
            }
            if has_pots || has_switches || has_sat_ind || has_sat_coupled {
                code.push_str("                g_work: G,\n");
                code.push_str("                c_work: C,\n");
            }
            code.push_str("            }),\n");
        } else {
            code.push_str("            s: S_DEFAULT,\n");
            if m > 0 {
                code.push_str("            k: K_DEFAULT,\n");
                code.push_str("            s_ni: S_NI_DEFAULT,\n");
            }
            code.push_str("            s_be: S_BE_DEFAULT,\n");
            if !ir.matrices.s_sub.is_empty() {
                code.push_str("            s_sub: S_SUB_DEFAULT,\n");
                code.push_str("            a_neg_sub: A_NEG_SUB_DEFAULT,\n");
                if m > 0 {
                    code.push_str("            k_sub: K_SUB_DEFAULT,\n");
                    code.push_str("            s_ni_sub: S_NI_SUB_DEFAULT,\n");
                }
            }
            if m > 0 {
                code.push_str("            k_be: K_BE_DEFAULT,\n");
                code.push_str("            s_ni_be: S_NI_BE_DEFAULT,\n");
            }
        }

        if has_pots || has_switches || has_sat_ind || has_sat_coupled {
            if use_full_nodal {
                // g_work/c_work are in cold; only emit the scalars here
                code.push_str(&format!(
                    "            current_sample_rate: {:.17e},\n",
                    ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64
                ));
                code.push_str("            matrices_dirty: false,\n");
            } else {
                code.push_str("            g_work: G,\n");
                code.push_str("            c_work: C,\n");
                code.push_str(&format!(
                    "            current_sample_rate: {:.17e},\n",
                    ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64
                ));
                code.push_str("            matrices_dirty: false,\n");
            }
        }
        // Saturation decimation counter: 0 triggers update on first sample
        if has_any_saturation {
            code.push_str("            sat_update_counter: 0,\n");
            code.push_str("            sat_resync_counter: 0,\n");
        }
        // Saturating inductor state: start at nominal L
        for (idx, _) in ir.saturating_inductors.iter().enumerate() {
            code.push_str(&format!(
                "            sat_ind_{}_l_eff: SAT_IND_{}_L0,\n",
                idx, idx
            ));
        }
        // Saturating coupled inductor state: start at nominal
        for (idx, sc) in ir.saturating_coupled.iter().enumerate() {
            let m0 = sc.coupling * (sc.l1_l0 * sc.l2_l0).sqrt();
            code.push_str(&format!(
                "            sat_ci_{idx}_l1_eff: SAT_CI_{idx}_L1_L0,\n\
                 \x20           sat_ci_{idx}_l2_eff: SAT_CI_{idx}_L2_L0,\n\
                 \x20           sat_ci_{idx}_m_eff: {m0:.17e},\n",
            ));
        }
        // Saturating transformer group state: start at nominal
        for (idx, sg) in ir.saturating_xfmr_groups.iter().enumerate() {
            let w = sg.num_windings;
            code.push_str(&format!(
                "            sat_xg_{idx}_l_eff: SAT_XG_{idx}_L0,\n",
            ));
            // Build nominal M matrix: M[i][j] = kappa[i][j] * sqrt(L0[i] * L0[j])
            let mut m_vals = Vec::with_capacity(w * w);
            for i in 0..w {
                for j in 0..w {
                    let m = sg.coupling_flat[i * w + j] * (sg.l0s[i] * sg.l0s[j]).sqrt();
                    m_vals.push(format!("{:.17e}", m));
                }
            }
            code.push_str(&format!(
                "            sat_xg_{idx}_m_eff: [{}],\n",
                m_vals.join(", "),
            ));
        }

        // IIR op-amp state initialization: initialize to DC steady state.
        // At DC, v+ - v- = Go/Gm * v_out ≈ 22µV (for AOL=200K), and the Boyle
        // internal node y_ss = AOL*(v+-v-) = v_out level. This is physically correct
        // — NOT a rounding error. Initializing to zero makes the op-amps think they
        // have to drive a huge imbalance on startup and causes oscillation.
        for (idx, oa_iir) in ir.opamp_iir.iter().enumerate() {
            let np_val = match oa_iir.np_idx {
                Some(i) if i < ir.dc_operating_point.len() => ir.dc_operating_point[i],
                _ => 0.0,
            };
            let nm_val = match oa_iir.nm_idx {
                Some(i) if i < ir.dc_operating_point.len() => ir.dc_operating_point[i],
                _ => 0.0,
            };
            let x0 = np_val - nm_val;
            let y0 = if oa_iir.go > 0.0 {
                (oa_iir.gm / oa_iir.go) * x0
            } else {
                0.0
            };
            code.push_str(&format!(
                "            oa{idx}_y_prev: {:.17e},\n\
                 \x20           oa{idx}_x_prev: {:.17e},\n\
                 \x20           oa{idx}_a1: OA{idx}_A1_DEFAULT,\n\
                 \x20           oa{idx}_b0: OA{idx}_B0_DEFAULT,\n",
                y0, x0
            ));
        }

        for (idx, pot) in ir.pots.iter().enumerate() {
            let r_nom = 1.0 / pot.g_nominal;
            code.push_str(&format!(
                "            pot_{}_resistance: {:.17e},\n\
                 \x20           pot_{}_resistance_prev: {:.17e},\n",
                idx, r_nom, idx, r_nom
            ));
        }

        for (idx, _sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!("            switch_{}_position: 0,\n", idx));
        }

        for dev in &device_params {
            for p in &dev.params {
                code.push_str(&format!(
                    "            device_{}_{}: DEVICE_{}_{},\n",
                    dev.dev_num, p.field_suffix, dev.dev_num, p.const_suffix
                ));
            }
        }

        for td in &thermal_devices {
            code.push_str(&format!(
                "            device_{}_tj: DEVICE_{}_TAMB,\n",
                td.dev_num, td.dev_num
            ));
        }

        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "            os_up_state: [0.0; {}],\n\
                 \x20           os_dn_state: [[0.0; {}]; NUM_OUTPUTS],\n",
                os_info.state_size, os_info.state_size
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "            os_up_state_outer: [0.0; {}],\n\
                     \x20           os_dn_state_outer: [[0.0; {}]; NUM_OUTPUTS],\n",
                    os_info.state_size_outer, os_info.state_size_outer
                ));
            }
        }

        code.push_str("        };\n");
        code.push_str("        state.warmup();\n");
        code.push_str("        state\n");
        code.push_str("    }\n");
        code.push_str("}\n\n");

        // impl CircuitState
        code.push_str("impl CircuitState {\n");

        // reset()
        code.push_str("    /// Reset to DC operating point\n");
        code.push_str("    pub fn reset(&mut self) {\n");
        code.push_str("        self.v_prev = self.dc_operating_point;\n");
        if has_dc_nl {
            code.push_str("        self.i_nl_prev = DC_NL_I;\n");
            code.push_str("        self.i_nl_prev_prev = DC_NL_I;\n");
        } else {
            code.push_str("        self.i_nl_prev = [0.0; M];\n");
            code.push_str("        self.i_nl_prev_prev = [0.0; M];\n");
        }
        code.push_str("        self.input_prev = 0.0;\n");
        code.push_str("        self.last_nr_iterations = 0;\n");
        if m > 0 {
            code.push_str("        self.chord_valid = false;\n");
        }
        // Re-init DC blocker from DC OP (prevents transient on reset)
        if ir.dc_block {
            let output_nodes = &ir.solver_config.output_nodes;
            for (oi, &node) in output_nodes.iter().enumerate() {
                if has_dc_op && node < ir.dc_operating_point.len() {
                    code.push_str(&format!(
                        "        self.dc_block_x_prev[{}] = self.dc_operating_point[{}];\n",
                        oi, node
                    ));
                } else {
                    code.push_str(&format!("        self.dc_block_x_prev[{}] = 0.0;\n", oi));
                }
            }
            code.push_str("        self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n");
        }
        code.push_str("        self.diag_peak_output = 0.0;\n");
        code.push_str("        self.diag_clamp_count = 0;\n");
        code.push_str("        self.diag_nr_max_iter_count = 0;\n");
        code.push_str("        self.diag_be_fallback_count = 0;\n");
        code.push_str("        self.diag_nan_reset_count = 0;\n");
        code.push_str("        self.diag_voltage_damp_count = 0;\n");
        code.push_str("        self.diag_substep_count = 0;\n");
        code.push_str("        self.diag_refactor_count = 0;\n");
        // Reset Schur complement matrices to defaults
        let cp = if use_full_nodal { "self.cold." } else { "self." };
        code.push_str(&format!("        {}s = S_DEFAULT;\n", cp));
        if m > 0 {
            code.push_str(&format!("        {}k = K_DEFAULT;\n", cp));
            code.push_str(&format!("        {}s_ni = S_NI_DEFAULT;\n", cp));
        }
        code.push_str(&format!("        {}s_be = S_BE_DEFAULT;\n", cp));
        if m > 0 {
            code.push_str(&format!("        {}k_be = K_BE_DEFAULT;\n", cp));
            code.push_str(&format!("        {}s_ni_be = S_NI_BE_DEFAULT;\n", cp));
        }
        if !ir.matrices.s_sub.is_empty() {
            code.push_str(&format!("        {}s_sub = S_SUB_DEFAULT;\n", cp));
            code.push_str(&format!("        {}a_neg_sub = A_NEG_SUB_DEFAULT;\n", cp));
            if m > 0 {
                code.push_str(&format!("        {}k_sub = K_SUB_DEFAULT;\n", cp));
                code.push_str(&format!("        {}s_ni_sub = S_NI_SUB_DEFAULT;\n", cp));
            }
        }
        if has_pots || has_switches || has_sat_ind || has_sat_coupled {
            code.push_str(&format!("        {}g_work = G;\n", cp));
            code.push_str(&format!("        {}c_work = C;\n", cp));
        }
        // Reset saturation counter
        if has_any_saturation {
            code.push_str("        self.sat_update_counter = 0;\n");
            code.push_str("        self.sat_resync_counter = 0;\n");
        }
        // Reset saturating inductor L_eff to nominal
        for (idx, _) in ir.saturating_inductors.iter().enumerate() {
            code.push_str(&format!(
                "        self.sat_ind_{}_l_eff = SAT_IND_{}_L0;\n",
                idx, idx
            ));
        }
        // Reset saturating coupled inductor state to nominal
        for (idx, sc) in ir.saturating_coupled.iter().enumerate() {
            let m0 = sc.coupling * (sc.l1_l0 * sc.l2_l0).sqrt();
            code.push_str(&format!(
                "        self.sat_ci_{idx}_l1_eff = SAT_CI_{idx}_L1_L0;\n\
                 \x20       self.sat_ci_{idx}_l2_eff = SAT_CI_{idx}_L2_L0;\n\
                 \x20       self.sat_ci_{idx}_m_eff = {m0:.17e};\n",
            ));
        }
        // Reset saturating transformer group state to nominal
        for (idx, sg) in ir.saturating_xfmr_groups.iter().enumerate() {
            let w = sg.num_windings;
            code.push_str(&format!(
                "        self.sat_xg_{idx}_l_eff = SAT_XG_{idx}_L0;\n",
            ));
            let mut m_vals = Vec::with_capacity(w * w);
            for i in 0..w {
                for j in 0..w {
                    let m = sg.coupling_flat[i * w + j] * (sg.l0s[i] * sg.l0s[j]).sqrt();
                    m_vals.push(format!("{:.17e}", m));
                }
            }
            code.push_str(&format!(
                "        self.sat_xg_{idx}_m_eff = [{}];\n",
                m_vals.join(", "),
            ));
        }
        // Reset IIR op-amp state — to DC steady state (see default() comment).
        for (idx, oa_iir) in ir.opamp_iir.iter().enumerate() {
            let np_val = match oa_iir.np_idx {
                Some(i) if i < ir.dc_operating_point.len() => ir.dc_operating_point[i],
                _ => 0.0,
            };
            let nm_val = match oa_iir.nm_idx {
                Some(i) if i < ir.dc_operating_point.len() => ir.dc_operating_point[i],
                _ => 0.0,
            };
            let x0 = np_val - nm_val;
            let y0 = if oa_iir.go > 0.0 {
                (oa_iir.gm / oa_iir.go) * x0
            } else {
                0.0
            };
            code.push_str(&format!(
                "        self.oa{idx}_y_prev = {:.17e};\n\
                 \x20       self.oa{idx}_x_prev = {:.17e};\n\
                 \x20       self.oa{idx}_a1 = OA{idx}_A1_DEFAULT;\n\
                 \x20       self.oa{idx}_b0 = OA{idx}_B0_DEFAULT;\n",
                y0, x0
            ));
        }
        for (idx, pot) in ir.pots.iter().enumerate() {
            let r_nom = 1.0 / pot.g_nominal;
            code.push_str(&format!(
                "        self.pot_{}_resistance = {:.17e};\n\
                 \x20       self.pot_{}_resistance_prev = {:.17e};\n",
                idx, r_nom, idx, r_nom
            ));
        }
        for (idx, _sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!("        self.switch_{}_position = 0;\n", idx));
        }
        for dev in &device_params {
            for p in &dev.params {
                code.push_str(&format!(
                    "        self.device_{}_{} = DEVICE_{}_{};\n",
                    dev.dev_num, p.field_suffix, dev.dev_num, p.const_suffix
                ));
            }
        }
        for td in &thermal_devices {
            code.push_str(&format!(
                "        self.device_{}_tj = DEVICE_{}_TAMB;\n",
                td.dev_num, td.dev_num
            ));
        }
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "        self.os_up_state = [0.0; {}];\n\
                 \x20       self.os_dn_state = [[0.0; {}]; NUM_OUTPUTS];\n",
                os_info.state_size, os_info.state_size
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "        self.os_up_state_outer = [0.0; {}];\n\
                     \x20       self.os_dn_state_outer = [[0.0; {}]; NUM_OUTPUTS];\n",
                    os_info.state_size_outer, os_info.state_size_outer
                ));
            }
        }
        code.push_str("        self.warmup();\n");
        code.push_str("    }\n\n");

        // warmup()
        code.push_str(
            "    /// Run silent warmup samples to settle into the correct operating point.\n",
        );
        code.push_str("    ///\n");
        code.push_str(
            "    /// Call after construction or reset() to ensure the circuit settles into\n",
        );
        code.push_str(
            "    /// the physically correct basin of attraction. Without warmup, circuits\n",
        );
        code.push_str("    /// with high-gain op-amps may lock into a parasitic equilibrium.\n");
        code.push_str("    ///\n");
        code.push_str("    /// The default `CircuitState::default()` calls this automatically.\n");
        code.push_str("    pub fn warmup(&mut self) {\n");
        code.push_str("        for _ in 0..50 {\n");
        code.push_str("            process_sample(0.0, self);\n");
        code.push_str("        }\n");
        code.push_str("    }\n\n");

        // set_dc_operating_point()
        code.push_str("    /// Set DC operating point (call after DC analysis)\n");
        code.push_str("    pub fn set_dc_operating_point(&mut self, v_dc: [f64; N]) {\n");
        code.push_str("        self.dc_operating_point = v_dc;\n");
        code.push_str("        self.v_prev = v_dc;\n");
        code.push_str("    }\n\n");

        // set_sample_rate()
        code.push_str(
            "    /// Recompute all sample-rate-dependent matrices for a new sample rate.\n",
        );
        code.push_str("    ///\n");
        code.push_str(
            "    /// Call this once during plugin initialization (NOT on the audio thread).\n",
        );
        code.push_str("    /// Rebuilds A, A_neg, A_be, A_neg_be from stored G and C matrices.\n");
        code.push_str("    pub fn set_sample_rate(&mut self, sample_rate: f64) {\n");
        code.push_str("        if !(sample_rate > 0.0 && sample_rate.is_finite()) {\n");
        code.push_str("            return;\n");
        code.push_str("        }\n\n");
        code.push_str("        // If same as codegen sample rate, reset to defaults\n");
        code.push_str("        if (sample_rate - SAMPLE_RATE).abs() < 0.5 {\n");
        code.push_str("            self.a = A_DEFAULT;\n");
        code.push_str("            self.a_neg = A_NEG_DEFAULT;\n");
        code.push_str("            self.a_be = A_BE_DEFAULT;\n");
        code.push_str("            self.a_neg_be = A_NEG_BE_DEFAULT;\n");
        if matches!(
            ir.solver_config.opamp_rail_mode,
            crate::codegen::OpampRailMode::BoyleDiodes
        ) {
            code.push_str("            // BoyleDiodes-only: invalidate the cross-timestep chord LU.\n");
            code.push_str("            // The chord_lu / chord_j_dev cache holds a *factored*\n");
            code.push_str("            // matrix and is paired with a specific (v_prev, j_dev)\n");
            code.push_str("            // pair from the most recent refactor. After the 50-sample\n");
            code.push_str("            // default warmup, chord_j_dev still reflects the deeply-\n");
            code.push_str("            // reverse-biased catch diodes (j_dev ≈ 1e-31), and the\n");
            code.push_str("            // adaptive trigger doesn't fire on the first signal sample\n");
            code.push_str("            // because both `j_dev` and `chord_j_dev` are still ≈ 1e-31\n");
            code.push_str("            // — but v_prev has drifted enough during warmup that the\n");
            code.push_str("            // back-solve against the stale factor produces a wrong\n");
            code.push_str("            // linear prediction. Forcing a refactor here is harmless\n");
            code.push_str("            // for non-BoyleDiodes modes (no Schottky-class circuit\n");
            code.push_str("            // exhibits the same staleness pattern), but it shifts the\n");
            code.push_str("            // first-iteration NR state for VCR ALC and other\n");
            code.push_str("            // attack-timing-sensitive control circuits, so we gate\n");
            code.push_str("            // the reset on BoyleDiodes mode only.\n");
            code.push_str("            self.chord_valid = false;\n");
        }
        code.push_str(&format!("            {}s = S_DEFAULT;\n", cp));
        code.push_str(&format!("            {}s_be = S_BE_DEFAULT;\n", cp));
        if m > 0 {
            code.push_str(&format!("            {}k = K_DEFAULT;\n", cp));
            code.push_str(&format!("            {}s_ni = S_NI_DEFAULT;\n", cp));
            code.push_str(&format!("            {}k_be = K_BE_DEFAULT;\n", cp));
            code.push_str(&format!("            {}s_ni_be = S_NI_BE_DEFAULT;\n", cp));
        }
        if ir.dc_block {
            code.push_str("            self.dc_block_r = DC_BLOCK_R;\n");
            code.push_str("            self.dc_block_x_prev = [0.0; NUM_OUTPUTS];\n");
            code.push_str("            self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n");
        }
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "            self.os_up_state = [0.0; {}];\n\
                 \x20           self.os_dn_state = [[0.0; {}]; NUM_OUTPUTS];\n",
                os_info.state_size, os_info.state_size
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "            self.os_up_state_outer = [0.0; {}];\n\
                     \x20           self.os_dn_state_outer = [[0.0; {}]; NUM_OUTPUTS];\n",
                    os_info.state_size_outer, os_info.state_size_outer
                ));
            }
        }
        code.push_str("            return;\n");
        code.push_str("        }\n\n");

        code.push_str(&format!(
            "        let internal_rate = sample_rate * {}.0;\n",
            ir.solver_config.oversampling_factor
        ));
        if has_pots || has_switches {
            code.push_str("        self.current_sample_rate = internal_rate;\n");
        }
        code.push_str("        self.rebuild_matrices(internal_rate);\n\n");

        // DC block recomputation
        if ir.dc_block {
            code.push_str("        // Recompute DC blocking coefficient\n\
                 \x20       self.dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / internal_rate;\n\
                 \x20       self.dc_block_x_prev = [0.0; NUM_OUTPUTS];\n\
                 \x20       self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n");
        }

        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "        self.os_up_state = [0.0; {}];\n\
                 \x20       self.os_dn_state = [[0.0; {}]; NUM_OUTPUTS];\n",
                os_info.state_size, os_info.state_size
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "        self.os_up_state_outer = [0.0; {}];\n\
                     \x20       self.os_dn_state_outer = [[0.0; {}]; NUM_OUTPUTS];\n",
                    os_info.state_size_outer, os_info.state_size_outer
                ));
            }
        }

        code.push_str("    }\n\n");

        // rebuild_matrices: recompute A/A_neg/A_be/A_neg_be from G+C
        let g_src = if has_pots || has_switches {
            if use_full_nodal { "self.cold.g_work" } else { "self.g_work" }
        } else {
            "G"
        };
        let c_src = if has_pots || has_switches {
            if use_full_nodal { "self.cold.c_work" } else { "self.c_work" }
        } else {
            "C"
        };

        code.push_str("    /// Recompute A, A_neg, A_be, A_neg_be, S, K, S_NI (and BE variants) from G and C.\n");
        code.push_str("    ///\n");
        code.push_str("    /// Called by set_sample_rate, set_pot, and set_switch.\n");
        code.push_str("    /// Includes O(N^3) matrix inversion for Schur complement NR.\n");
        code.push_str("    pub fn rebuild_matrices(&mut self, internal_rate: f64) {\n");
        if ir.solver_config.backward_euler {
            code.push_str("        let alpha = internal_rate; // backward Euler: alpha = 1/T\n");
        } else {
            code.push_str("        let alpha = 2.0 * internal_rate; // trapezoidal: alpha = 2/T\n");
        }
        code.push_str("        let alpha_be = internal_rate;\n");
        if !ir.matrices.s_sub.is_empty() {
            code.push_str("        let alpha_sub = 4.0 * internal_rate; // trap at 2× rate\n");
        }
        code.push('\n');

        // Build A = G + alpha*C and A_neg:
        //   trapezoidal: A_neg = alpha*C - G
        //   backward Euler: A_neg = alpha*C  (no G term — history has only capacitor)
        // No Boyle elimination — IIR op-amp model keeps Gm out of the MNA matrix.
        let a_neg_formula = if ir.solver_config.backward_euler {
            format!("alpha * {}[i][j]", c_src)
        } else {
            format!("alpha * {}[i][j] - {}[i][j]", c_src, g_src)
        };
        code.push_str(&format!(
            "        for i in 0..N {{\n\
             \x20           for j in 0..N {{\n\
             \x20               self.a[i][j] = {}[i][j] + alpha * {}[i][j];\n\
             \x20               self.a_neg[i][j] = {};\n\
             \x20               self.a_be[i][j] = {}[i][j] + alpha_be * {}[i][j];\n\
             \x20               self.a_neg_be[i][j] = alpha_be * {}[i][j];\n\
             \x20           }}\n\
             \x20       }}\n",
            g_src, c_src, a_neg_formula, g_src, c_src, c_src
        ));

        if !ir.matrices.s_sub.is_empty() {
            code.push_str(&format!(
                "        for i in 0..N {{\n\
                 \x20           for j in 0..N {{\n\
                 \x20               {cp}a_neg_sub[i][j] = alpha_sub * {c}[i][j] - {g}[i][j];\n\
                 \x20           }}\n\
                 \x20       }}\n",
                cp = cp, c = c_src, g = g_src
            ));
        }

        // Zero VS/VCVS algebraic rows in A_neg, A_neg_be, A_neg_sub (NOT inductor rows)
        if n_nodes < n_aug {
            code.push_str(&format!(
                "        for i in {}..{} {{\n\
                 \x20           for j in 0..N {{\n\
                 \x20               self.a_neg[i][j] = 0.0;\n\
                 \x20               self.a_neg_be[i][j] = 0.0;\n\
                 \x20           }}\n\
                 \x20       }}\n",
                n_nodes, n_aug
            ));
            if !ir.matrices.s_sub.is_empty() {
                code.push_str(&format!(
                    "        for i in {}..{} {{ for j in 0..N {{ {}a_neg_sub[i][j] = 0.0; }} }}\n",
                    n_nodes, n_aug, cp
                ));
            }
        }

        // Recompute IIR op-amp filter coefficients for the new sample rate
        for (idx, _oa_iir) in ir.opamp_iir.iter().enumerate() {
            code.push_str(&format!(
                "        {{\n\
                 \x20           let alpha_oa = 2.0 * internal_rate;\n\
                 \x20           let denom = alpha_oa * OA{idx}_C_DOM + OA{idx}_GO;\n\
                 \x20           self.oa{idx}_a1 = (alpha_oa * OA{idx}_C_DOM - OA{idx}_GO) / denom;\n\
                 \x20           self.oa{idx}_b0 = OA{idx}_GM / denom;\n\
                 \x20       }}\n"
            ));
        }

        // Recompute Schur complement matrices: S = A^{-1}, K = N_v*S*N_i, S_NI = S*N_i
        code.push_str("\n        // Recompute S = A^{-1} (trapezoidal)\n");
        code.push_str("        if let Some(inv) = invert_n(&self.a) {\n");
        code.push_str(&format!("            {}s = inv;\n", cp));
        if m > 0 {
            code.push_str("            // K = N_v * S * N_i\n");
            code.push_str("            for i in 0..M {\n");
            code.push_str("                for j in 0..M {\n");
            code.push_str("                    let mut sum = 0.0;\n");
            code.push_str("                    for a in 0..N {\n");
            code.push_str("                        let mut s_ni_aj = 0.0;\n");
            code.push_str(&format!(
                "                        for b in 0..N {{ s_ni_aj += {}s[a][b] * N_I[b][j]; }}\n", cp
            ));
            code.push_str("                        sum += N_V[i][a] * s_ni_aj;\n");
            code.push_str("                    }\n");
            code.push_str(&format!("                    {}k[i][j] = sum;\n", cp));
            code.push_str("                }\n");
            code.push_str("            }\n");
            code.push_str("            // S_NI = S * N_i\n");
            code.push_str("            for i in 0..N {\n");
            code.push_str("                for j in 0..M {\n");
            code.push_str("                    let mut sum = 0.0;\n");
            code.push_str(&format!(
                "                    for a in 0..N {{ sum += {}s[i][a] * N_I[a][j]; }}\n", cp
            ));
            code.push_str(&format!("                    {}s_ni[i][j] = sum;\n", cp));
            code.push_str("                }\n");
            code.push_str("            }\n");
        }
        code.push_str("        }\n");

        // Recompute S_be = A_be^{-1}
        code.push_str("        // Recompute S_be = A_be^{-1} (backward Euler)\n");
        code.push_str("        if let Some(inv) = invert_n(&self.a_be) {\n");
        code.push_str(&format!("            {}s_be = inv;\n", cp));
        if m > 0 {
            code.push_str("            for i in 0..M {\n");
            code.push_str("                for j in 0..M {\n");
            code.push_str("                    let mut sum = 0.0;\n");
            code.push_str("                    for a in 0..N {\n");
            code.push_str("                        let mut s_ni_aj = 0.0;\n");
            code.push_str(&format!(
                "                        for b in 0..N {{ s_ni_aj += {}s_be[a][b] * N_I[b][j]; }}\n", cp
            ));
            code.push_str("                        sum += N_V[i][a] * s_ni_aj;\n");
            code.push_str("                    }\n");
            code.push_str(&format!("                    {}k_be[i][j] = sum;\n", cp));
            code.push_str("                }\n");
            code.push_str("            }\n");
            code.push_str("            for i in 0..N {\n");
            code.push_str("                for j in 0..M {\n");
            code.push_str("                    let mut sum = 0.0;\n");
            code.push_str(&format!(
                "                    for a in 0..N {{ sum += {}s_be[i][a] * N_I[a][j]; }}\n", cp
            ));
            code.push_str(&format!("                    {}s_ni_be[i][j] = sum;\n", cp));
            code.push_str("                }\n");
            code.push_str("            }\n");
        }
        code.push_str("        }\n");

        // Recompute S_sub = (G + alpha_sub*C)^{-1} (trap at 2× rate)
        if !ir.matrices.s_sub.is_empty() {
            code.push_str("        // Recompute S_sub = (G + alpha_sub*C)^{-1} (trap at 2× rate)\n");
            code.push_str(&format!(
                "        let mut a_sub = [[0.0f64; N]; N];\n\
                 \x20       for i in 0..N {{ for j in 0..N {{ a_sub[i][j] = {g}[i][j] + alpha_sub * {c}[i][j]; }} }}\n",
                g = g_src, c = c_src
            ));
            code.push_str("        if let Some(inv) = invert_n(&a_sub) {\n");
            code.push_str(&format!("            {}s_sub = inv;\n", cp));
            if m > 0 {
                code.push_str("            for i in 0..M {\n");
                code.push_str("                for j in 0..M {\n");
                code.push_str("                    let mut sum = 0.0;\n");
                code.push_str("                    for a in 0..N {\n");
                code.push_str("                        let mut s_ni_aj = 0.0;\n");
                code.push_str(&format!(
                    "                        for b in 0..N {{ s_ni_aj += {}s_sub[a][b] * N_I[b][j]; }}\n", cp
                ));
                code.push_str("                        sum += N_V[i][a] * s_ni_aj;\n");
                code.push_str("                    }\n");
                code.push_str(&format!("                    {}k_sub[i][j] = sum;\n", cp));
                code.push_str("                }\n");
                code.push_str("            }\n");
                code.push_str("            for i in 0..N {\n");
                code.push_str("                for j in 0..M {\n");
                code.push_str("                    let mut sum = 0.0;\n");
                code.push_str(&format!(
                    "                    for a in 0..N {{ sum += {}s_sub[i][a] * N_I[a][j]; }}\n", cp
                ));
                code.push_str(&format!("                    {}s_ni_sub[i][j] = sum;\n", cp));
                code.push_str("                }\n");
                code.push_str("            }\n");
            }
            code.push_str("        }\n");
        }

        // Invalidate cross-timestep chord LU — the A matrix changed
        if m > 0 {
            code.push_str("\n        // Invalidate chord LU cache (matrices changed)\n");
            code.push_str("        self.chord_valid = false;\n");
        }

        code.push_str("    }\n\n");

        // set_pot_N() methods — O(1) delta stamping into A/A_neg/A_be matrices
        // Since A = G + alpha*C, changing G by delta_g means A changes by delta_g at the same entries.
        // A_neg = alpha*C - G, so A_neg changes by -delta_g. A_neg_be has no G term (unchanged).
        for (idx, pot) in ir.pots.iter().enumerate() {
            let np = pot.node_p;
            let nq = pot.node_q;
            code.push_str(&format!(
                "    /// Set potentiometer {} resistance (clamped to [{:.1}..{:.1}] ohms).\n\
                 \x20   ///\n\
                 \x20   /// Updates g_work and rebuilds all matrices (O(N^3)).\n\
                 \x20   /// Call per-block, not per-sample.\n",
                idx, pot.min_resistance, pot.max_resistance
            ));
            code.push_str(&format!(
                "    pub fn set_pot_{}(&mut self, resistance: f64) {{\n",
                idx
            ));
            code.push_str(&format!(
                "        if !resistance.is_finite() {{ return; }}\n\
                 \x20       let r = resistance.clamp(POT_{}_MIN_R, POT_{}_MAX_R);\n\
                 \x20       if (r - self.pot_{}_resistance).abs() < 1e-12 {{ return; }}\n\n\
                 \x20       // Delta conductance: stamp into A, A_neg, A_be (NOT A_neg_be: no G term)\n\
                 \x20       let delta_g = 1.0 / r - 1.0 / self.pot_{}_resistance;\n",
                idx, idx, idx, idx
            ));

            // Emit conductance stamp into g_work (full dimension if Boyle, reduced otherwise)
            // Pot node indices (np, nq) are 1-indexed MNA nodes (< n_nodes), same in both systems.
            let gw = if use_full_nodal { "self.cold.g_work" } else { "self.g_work" };
            let emit_g_work_stamp = |code: &mut String| {
                if np > 0 {
                    code.push_str(&format!(
                        "        {}[{}][{}] += delta_g;\n",
                        gw, np - 1, np - 1
                    ));
                }
                if nq > 0 {
                    code.push_str(&format!(
                        "        {}[{}][{}] += delta_g;\n",
                        gw, nq - 1, nq - 1
                    ));
                }
                if np > 0 && nq > 0 {
                    code.push_str(&format!(
                        "        {}[{}][{}] -= delta_g;\n\
                         \x20       {}[{}][{}] -= delta_g;\n",
                        gw, np - 1, nq - 1,
                        gw, nq - 1, np - 1
                    ));
                }
            };

            {
                // Delta stamp A/A_neg/A_be directly (fast path)
                let emit_delta_stamp = |code: &mut String, matrix: &str, sign: &str| {
                    if np > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] {sign}= delta_g;\n",
                            np - 1, np - 1
                        ));
                    }
                    if nq > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] {sign}= delta_g;\n",
                            nq - 1, nq - 1
                        ));
                    }
                    if np > 0 && nq > 0 {
                        let neg_sign = if sign == "+" { "-" } else { "+" };
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] {neg_sign}= delta_g;\n\
                             \x20       self.{matrix}[{}][{}] {neg_sign}= delta_g;\n",
                            np - 1, nq - 1,
                            nq - 1, np - 1
                        ));
                    }
                };

                emit_delta_stamp(&mut code, "a", "+");
                emit_delta_stamp(&mut code, "a_neg", "-");
                emit_delta_stamp(&mut code, "a_be", "+");
                // A_neg_be = alpha_be * C — no G term, unchanged by pot

                // Also update g_work for consistency
                if has_pots || has_switches {
                    code.push_str(
                        "\n        // Update working G for sample rate rebuild consistency\n",
                    );
                    emit_g_work_stamp(&mut code);
                }
            }

            // Capture old resistance for warm DC-OP re-init gate before updating
            code.push_str(&format!("\n        let r_prev_{} = self.pot_{}_resistance;\n", idx, idx));
            code.push_str(&format!("        self.pot_{}_resistance = r;\n", idx));
            code.push_str("        self.matrices_dirty = true;\n");

            // Warm DC-OP re-init: on large pot jumps, reset NR seed state to the baked DC
            // operating point. This prevents NR max-iter-cap hits when v_prev is stale from
            // the old operating point. Gated at 20% relative delta so per-sample smoothed
            // sweeps do not snap v_prev mid-signal and cause clicks. See Batch D Phase 2.
            let has_dc_op_nodal = ir.has_dc_op;
            let has_dc_nl_nodal = ir.topology.m > 0
                && !ir.dc_nl_currents.is_empty()
                && ir.dc_nl_currents.iter().any(|&v| v.abs() > 1e-30);
            code.push_str(&format!(
                "        let rel_delta_{idx} = (r - r_prev_{idx}).abs() / r_prev_{idx}.max(1e-12);\n\
                 \x20       if rel_delta_{idx} > 0.20 {{\n",
            ));
            if has_dc_op_nodal {
                code.push_str("            self.v_prev = DC_OP;\n");
            } else {
                code.push_str("            self.v_prev = [0.0; N];\n");
            }
            if has_dc_nl_nodal {
                code.push_str("            self.i_nl_prev = DC_NL_I;\n");
            }
            code.push_str("        }\n");

            code.push_str("    }\n\n");
        }

        // set_switch_N() methods
        for (idx, sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!(
                "    /// Set switch {} position (0-indexed, {} positions).\n",
                idx, sw.num_positions
            ));
            code.push_str(&format!(
                "    pub fn set_switch_{}(&mut self, position: usize) {{\n\
                 \x20       if position >= SWITCH_{}_NUM_POSITIONS {{ return; }}\n\
                 \x20       if position == self.switch_{}_position {{ return; }}\n\n",
                idx, idx, idx
            ));

            for (ci, comp) in sw.components.iter().enumerate() {
                let np = comp.node_p;
                let nq = comp.node_q;
                let matrix = if comp.component_type == 'R' {
                    if use_full_nodal { "cold.g_work" } else { "g_work" }
                } else {
                    if use_full_nodal { "cold.c_work" } else { "c_work" }
                };

                code.push_str(&format!(
                    "        // Switch {} component {} ({}, type {})\n",
                    idx, ci, comp.name, comp.component_type
                ));
                code.push_str(&format!(
                    "        let old_val_{} = SWITCH_{}_COMP_{}_VALUES[self.switch_{}_position];\n\
                     \x20       let new_val_{} = SWITCH_{}_COMP_{}_VALUES[position];\n",
                    ci, idx, ci, idx, ci, idx, ci
                ));

                if comp.component_type == 'R' {
                    // Resistor: unstamp old conductance, stamp new
                    code.push_str(&format!(
                        "        let g_old_{ci} = 1.0 / old_val_{ci};\n\
                         \x20       let g_new_{ci} = 1.0 / new_val_{ci};\n\
                         \x20       let delta_{ci} = g_new_{ci} - g_old_{ci};\n"
                    ));
                } else {
                    // Capacitor or Inductor in C matrix: delta is new - old directly
                    code.push_str(&format!(
                        "        let delta_{ci} = new_val_{ci} - old_val_{ci};\n"
                    ));
                }

                // Stamp delta into g_work or c_work
                if let Some(aug_row) = comp.augmented_row.filter(|_| comp.component_type == 'L') {
                    // Augmented MNA: L value on diagonal of branch variable row
                    let cw = if use_full_nodal { "self.cold.c_work" } else { "self.c_work" };
                    code.push_str(&format!(
                        "        {cw}[{aug_row}][{aug_row}] += delta_{ci};\n"
                    ));
                } else {
                    // R or C: conductance stamp at circuit nodes
                    if np > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] += delta_{ci};\n",
                            np - 1,
                            np - 1
                        ));
                    }
                    if nq > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] += delta_{ci};\n",
                            nq - 1,
                            nq - 1
                        ));
                    }
                    if np > 0 && nq > 0 {
                        code.push_str(&format!(
                            "        self.{matrix}[{}][{}] -= delta_{ci};\n\
                             \x20       self.{matrix}[{}][{}] -= delta_{ci};\n",
                            np - 1,
                            nq - 1,
                            nq - 1,
                            np - 1
                        ));
                    }
                }
                code.push('\n');
            }

            // Recompute off-diagonal mutual inductance entries
            if !sw.mutual_entries.is_empty() {
                code.push_str("        // Update mutual inductance off-diagonal entries\n");
                let cw_m = if use_full_nodal { "self.cold.c_work" } else { "self.c_work" };
                for me in &sw.mutual_entries {
                    code.push_str(&format!(
                        "        {{\n\
                         \x20           let m = {:.17e}_f64 * ({cw_m}[{}][{}] * {cw_m}[{}][{}]).sqrt();\n\
                         \x20           {cw_m}[{}][{}] = m;\n\
                         \x20           {cw_m}[{}][{}] = m;\n\
                         \x20       }}\n",
                        me.coupling,
                        me.row_a, me.row_a,
                        me.row_b, me.row_b,
                        me.row_a, me.row_b,
                        me.row_b, me.row_a,
                    ));
                }
                code.push('\n');
            }

            code.push_str(&format!(
                "        self.switch_{}_position = position;\n",
                idx
            ));
            code.push_str("        self.matrices_dirty = true;\n");

            // Warm DC-OP re-init on switch change: reset NR seed to DC bias.
            // Switches have no smoother so any change is a step; always re-init.
            // See Batch D Phase 2.
            let has_dc_op_sw = ir.has_dc_op;
            let has_dc_nl_sw = ir.topology.m > 0
                && !ir.dc_nl_currents.is_empty()
                && ir.dc_nl_currents.iter().any(|&v| v.abs() > 1e-30);
            if has_dc_op_sw {
                code.push_str("        self.v_prev = DC_OP;\n");
            } else {
                code.push_str("        self.v_prev = [0.0; N];\n");
            }
            if has_dc_nl_sw {
                code.push_str("        self.i_nl_prev = DC_NL_I;\n");
            }

            code.push_str("    }\n\n");
        }

        code.push_str("}\n\n");

        let _ = (n, m, n_nodes, n_aug, num_outputs);

        code
    }

    /// Emit `invert_n` function: runtime N×N matrix inversion via Gauss-Jordan.
    ///
    /// Used by `rebuild_matrices()` to recompute S = A^{-1} when sample rate or
    /// pot/switch values change.
    pub(super) fn emit_nodal_invert_n(_ir: &CircuitIR) -> String {
        let mut code = section_banner("MATRIX INVERSION (for rebuild_matrices)");

        code.push_str(
            "/// Invert an N×N matrix using Gauss-Jordan elimination with partial pivoting.\n",
        );
        code.push_str("/// Returns None if the matrix is singular.\n");
        code.push_str("#[inline(never)]\n");
        code.push_str("fn invert_n(a: &[[f64; N]; N]) -> Option<[[f64; N]; N]> {\n");
        code.push_str("    let mut aug = [[0.0f64; 2 * N]; N];\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N {\n");
        code.push_str("            aug[i][j] = a[i][j];\n");
        code.push_str("        }\n");
        code.push_str("        aug[i][N + i] = 1.0;\n");
        code.push_str("    }\n\n");

        // Forward elimination with partial pivoting
        code.push_str("    for col in 0..N {\n");
        code.push_str("        let mut max_row = col;\n");
        code.push_str("        let mut max_val = aug[col][col].abs();\n");
        code.push_str("        for row in (col + 1)..N {\n");
        code.push_str("            let v = aug[row][col].abs();\n");
        code.push_str("            if v > max_val { max_val = v; max_row = row; }\n");
        code.push_str("        }\n");
        code.push_str("        if max_val < 1e-30 { return None; }\n");
        code.push_str("        if max_row != col { aug.swap(col, max_row); }\n");
        code.push_str("        let pivot = aug[col][col];\n");
        code.push_str("        for row in (col + 1)..N {\n");
        code.push_str("            let factor = aug[row][col] / pivot;\n");
        code.push_str(
            "            for j in col..(2 * N) { aug[row][j] -= factor * aug[col][j]; }\n",
        );
        code.push_str("        }\n");
        code.push_str("    }\n\n");

        // Back substitution
        code.push_str("    for col in (0..N).rev() {\n");
        code.push_str("        let pivot = aug[col][col];\n");
        code.push_str("        if pivot.abs() < 1e-30 { return None; }\n");
        code.push_str("        for j in 0..(2 * N) { aug[col][j] /= pivot; }\n");
        code.push_str("        for row in 0..col {\n");
        code.push_str("            let factor = aug[row][col];\n");
        code.push_str("            for j in 0..(2 * N) { aug[row][j] -= factor * aug[col][j]; }\n");
        code.push_str("        }\n");
        code.push_str("    }\n\n");

        // Extract result
        code.push_str("    let mut result = [[0.0f64; N]; N];\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N { result[i][j] = aug[i][N + j]; }\n");
        code.push_str("    }\n");
        code.push_str("    Some(result)\n");
        code.push_str("}\n\n");

        code
    }

    /// Emit process_sample using Schur complement: precomputed S = A^{-1}, M-dim NR.
    ///
    /// This replaces the old O(N^3)-per-iteration LU solve with:
    /// 1. Build RHS (same as before)
    /// 2. Linear prediction: v_pred = S * rhs (O(N^2))
    /// 3. Extract device voltages: p = N_v * v_pred (O(M*N))
    /// 4. M-dim NR (same as DK: O(M^3) per iteration)
    /// 5. Recover full v = v_pred + S_NI * i_nl (O(M*N))
    pub(super) fn emit_nodal_schur_process_sample(ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let num_outputs = ir.solver_config.output_nodes.len();
        let os_factor = ir.solver_config.oversampling_factor;
        let has_pots = !ir.pots.is_empty();

        let mut code = section_banner(
            "PROCESS SAMPLE (Schur complement: M-dim NR via precomputed S = A^{-1})",
        );

        // Function signature
        if os_factor > 1 {
            code.push_str("/// Process a single sample at the internal (oversampled) rate.\n");
            code.push_str("///\n");
            code.push_str("/// Called by `process_sample()` through the oversampling chain.\n");
            code.push_str("#[inline(always)]\n");
            code.push_str("fn process_sample_inner(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n");
        } else {
            code.push_str("/// Process a single audio sample through the circuit.\n");
            code.push_str("///\n");
            code.push_str(
                "/// Uses Schur complement NR: precomputes S = A^{-1}, iterates in M-space.\n",
            );
            code.push_str("/// Cost: O(N^2) linear prediction + O(M^3) per NR iteration.\n");
            code.push_str("#[inline]\n");
            code.push_str("pub fn process_sample(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n");
        }

        // Input sanitization
        code.push_str(
            "    let input = if input.is_finite() { input.clamp(-100.0, 100.0) } else { 0.0 };\n\n",
        );

        // Lazy rebuild: process all pot/switch changes in one batch
        let has_any_saturation = !ir.saturating_inductors.is_empty()
            || !ir.saturating_coupled.is_empty()
            || !ir.saturating_xfmr_groups.is_empty();
        let has_rebuild = has_pots
            || !ir.switches.is_empty()
            || has_any_saturation;
        if has_rebuild {
            code.push_str(
                "    // Lazy rebuild: batch all pot/switch changes into one matrix rebuild\n\
                 \x20   if state.matrices_dirty {\n\
                 \x20       state.rebuild_matrices(state.current_sample_rate);\n\
                 \x20       state.matrices_dirty = false;\n\
                 \x20   }\n\n",
            );
        }

        // Flush denormals in state vectors (prevents 50-100x CPU penalty during silence)
        code.push_str("    for v in state.v_prev.iter_mut() { *v = *v + 1e-25 - 1e-25; }\n");
        if m > 0 {
            code.push_str("    for v in state.i_nl_prev.iter_mut() { *v = *v + 1e-25 - 1e-25; }\n");
        }
        code.push('\n');

        // Decimated saturation update: check every SAT_UPDATE_INTERVAL samples
        if has_any_saturation {
            code.push_str("    // Decimated saturation update (every SAT_UPDATE_INTERVAL samples)\n");
            code.push_str("    if state.sat_update_counter == 0 {\n");
            code.push_str("        state.sat_update_counter = SAT_UPDATE_INTERVAL;\n");
        }

        // Saturating inductor L(I) update: compute L_eff, patch c_work, rebuild
        if !ir.saturating_inductors.is_empty() {
            code.push_str("        // Saturating inductors: L_eff = L0 / cosh^2(I / Isat)\n");
            code.push_str("        let mut sat_changed = false;\n");
            for (idx, _si) in ir.saturating_inductors.iter().enumerate() {
                code.push_str(&format!(
                    "        {{ // Saturating inductor {idx}\n\
                     \x20           let i_branch = state.v_prev[SAT_IND_{idx}_AUG_ROW];\n\
                     \x20           let x = i_branch / SAT_IND_{idx}_ISAT;\n\
                     \x20           let cosh_x = x.cosh();\n\
                     \x20           let l_eff = (SAT_IND_{idx}_L0 / (cosh_x * cosh_x)).max(SAT_IND_{idx}_L0 * 0.01);\n\
                     \x20           if (l_eff - state.sat_ind_{idx}_l_eff).abs() > SAT_IND_{idx}_L0 * 1e-4 {{\n\
                     \x20               state.c_work[SAT_IND_{idx}_AUG_ROW][SAT_IND_{idx}_AUG_ROW] = l_eff;\n\
                     \x20               state.sat_ind_{idx}_l_eff = l_eff;\n\
                     \x20               sat_changed = true;\n\
                     \x20           }}\n\
                     \x20       }}\n",
                ));
            }
        }

        // Saturating coupled inductors: compute L_eff per winding, patch c_work
        if !ir.saturating_coupled.is_empty() {
            if ir.saturating_inductors.is_empty() {
                code.push_str("        let mut sat_changed = false;\n");
            }
            for (idx, _sc) in ir.saturating_coupled.iter().enumerate() {
                code.push_str(&format!(
                    "        {{ // Coupled pair {idx}\n\
                     \x20           let i1 = state.v_prev[SAT_CI_{idx}_K1];\n\
                     \x20           let i2 = state.v_prev[SAT_CI_{idx}_K2];\n\
                     \x20           let c1 = (i1 / SAT_CI_{idx}_L1_ISAT).cosh();\n\
                     \x20           let l1_eff = (SAT_CI_{idx}_L1_L0 / (c1 * c1)).max(SAT_CI_{idx}_L1_L0 * 0.01);\n\
                     \x20           let c2 = (i2 / SAT_CI_{idx}_L2_ISAT).cosh();\n\
                     \x20           let l2_eff = (SAT_CI_{idx}_L2_L0 / (c2 * c2)).max(SAT_CI_{idx}_L2_L0 * 0.01);\n\
                     \x20           let m_eff = SAT_CI_{idx}_COUPLING * (l1_eff * l2_eff).sqrt();\n\
                     \x20           if (l1_eff - state.sat_ci_{idx}_l1_eff).abs() > SAT_CI_{idx}_L1_L0 * 1e-4 || (l2_eff - state.sat_ci_{idx}_l2_eff).abs() > SAT_CI_{idx}_L2_L0 * 1e-4 || (m_eff - state.sat_ci_{idx}_m_eff).abs() > 1e-6 {{\n\
                     \x20               state.c_work[SAT_CI_{idx}_K1][SAT_CI_{idx}_K1] = l1_eff;\n\
                     \x20               state.c_work[SAT_CI_{idx}_K2][SAT_CI_{idx}_K2] = l2_eff;\n\
                     \x20               state.c_work[SAT_CI_{idx}_K1][SAT_CI_{idx}_K2] = m_eff;\n\
                     \x20               state.c_work[SAT_CI_{idx}_K2][SAT_CI_{idx}_K1] = m_eff;\n\
                     \x20               state.sat_ci_{idx}_l1_eff = l1_eff;\n\
                     \x20               state.sat_ci_{idx}_l2_eff = l2_eff;\n\
                     \x20               state.sat_ci_{idx}_m_eff = m_eff;\n\
                     \x20               sat_changed = true;\n\
                     \x20           }}\n\
                     \x20       }}\n",
                ));
            }
        }

        // Saturating transformer groups: compute L_eff per winding, patch c_work
        for (idx, sg) in ir.saturating_xfmr_groups.iter().enumerate() {
            let w = sg.num_windings;
            if ir.saturating_inductors.is_empty() && ir.saturating_coupled.is_empty() && idx == 0 {
                code.push_str("        let mut sat_changed = false;\n");
            }
            code.push_str(&format!(
                "        {{ // Transformer group {idx} ({name}, {w} windings)\n\
                 \x20           let w = SAT_XG_{idx}_W;\n\
                 \x20           let mut l_eff = [0.0f64; SAT_XG_{idx}_W];\n\
                 \x20           let mut any_changed = false;\n\
                 \x20           for wi in 0..w {{\n\
                 \x20               let i_branch = state.v_prev[SAT_XG_{idx}_ROWS[wi]];\n\
                 \x20               let x = i_branch / SAT_XG_{idx}_ISAT[wi];\n\
                 \x20               let c = x.cosh();\n\
                 \x20               l_eff[wi] = (SAT_XG_{idx}_L0[wi] / (c * c)).max(SAT_XG_{idx}_L0[wi] * 0.01);\n\
                 \x20               if (l_eff[wi] - state.sat_xg_{idx}_l_eff[wi]).abs() > SAT_XG_{idx}_L0[wi] * 1e-4 {{\n\
                 \x20                   any_changed = true;\n\
                 \x20               }}\n\
                 \x20           }}\n\
                 \x20           if any_changed {{\n\
                 \x20               let mut m_new = [0.0f64; SAT_XG_{idx}_W * SAT_XG_{idx}_W];\n\
                 \x20               for i in 0..w {{\n\
                 \x20                   for j in 0..w {{\n\
                 \x20                       m_new[i * w + j] = SAT_XG_{idx}_KAPPA[i * w + j] * (l_eff[i] * l_eff[j]).sqrt();\n\
                 \x20                   }}\n\
                 \x20               }}\n\
                 \x20               for i in 0..w {{\n\
                 \x20                   for j in 0..w {{\n\
                 \x20                       state.c_work[SAT_XG_{idx}_ROWS[i]][SAT_XG_{idx}_ROWS[j]] = m_new[i * w + j];\n\
                 \x20                   }}\n\
                 \x20               }}\n\
                 \x20               state.sat_xg_{idx}_l_eff = l_eff;\n\
                 \x20               state.sat_xg_{idx}_m_eff = m_new;\n\
                 \x20               sat_changed = true;\n\
                 \x20           }}\n\
                 \x20       }}\n",
                name = sg.name,
            ));
        }

        // SM rank-1 update for each changed uncoupled inductor (O(N²) per inductor)
        // c_work already patched above; SM updates S, K, S_NI, A_neg in-place.
        // Every SAT_RESYNC_INTERVAL SM updates, do a full O(N³) rebuild from c_work.
        if has_any_saturation {
            code.push_str("        if sat_changed {\n");
            code.push_str("            state.sat_resync_counter += 1;\n");
            code.push_str("            if state.sat_resync_counter >= SAT_RESYNC_INTERVAL {\n");
            code.push_str("                state.sat_resync_counter = 0;\n");
            code.push_str("                state.rebuild_matrices(state.current_sample_rate);\n");
            code.push_str("            } else {\n");
            code.push_str("                // SM rank-1 update: O(N²) per changed diagonal entry\n");
            if ir.solver_config.backward_euler {
                code.push_str("                let alpha = state.current_sample_rate; // backward Euler\n");
            } else {
                code.push_str("                let alpha = 2.0 * state.current_sample_rate; // trapezoidal\n");
            }
        }

        // Emit SM for each uncoupled saturating inductor
        for (idx, _si) in ir.saturating_inductors.iter().enumerate() {
            code.push_str(&format!(
                "                {{ // SM for inductor {idx}\n\
                 \x20                   let k = SAT_IND_{idx}_AUG_ROW;\n\
                 \x20                   let old_l = state.sat_ind_{idx}_l_eff; // already updated above\n\
                 \x20                   // delta_a is how much A[k][k] changed\n\
                 \x20                   // We need the delta from what the matrices currently think to what l_eff is now.\n\
                 \x20                   // Since c_work was just patched, but matrices haven't been rebuilt,\n\
                 \x20                   // the delta is alpha * (new_l - old_l_that_matrices_know).\n\
                 \x20                   // The matrices' last known l is c_work BEFORE we patched it,\n\
                 \x20                   // but we already patched c_work. Use the tracking: old value is\n\
                 \x20                   // what sat_ind_N_l_eff WAS before the update, but we already updated it.\n\
                 \x20                   // We need to track the pre-update value. Recompute delta from A_neg.\n\
                 \x20                   // A_neg[k][k] = alpha * C[k][k] - G[k][k], so current matrix has\n\
                 \x20                   // A[k][k] = G[k][k] + alpha * old_C, new A[k][k] = G[k][k] + alpha * new_C\n\
                 \x20                   // delta_a = alpha * (new_C - old_C) = alpha * (l_eff - l_eff_before_patch)\n\
                 \x20                   // But we already updated sat_ind_N_l_eff. We know c_work was patched.\n\
                 \x20                   // Simple: reconstruct delta from a_neg.\n\
                 \x20                   // Actually: the simplest approach is to compute delta_a from\n\
                 \x20                   // the difference between new A[k][k] and current A[k][k]:\n\
                 \x20                   let new_a_kk = state.g_work[k][k] + alpha * state.c_work[k][k];\n\
                 \x20                   let delta_a = new_a_kk - state.a[k][k];\n\
                 \x20                   if delta_a.abs() > 1e-15 {{\n\
                 \x20                       let scale = delta_a / (1.0 + delta_a * state.s[k][k]);\n\
                 \x20                       let mut s_col = [0.0f64; N];\n\
                 \x20                       let mut s_row = [0.0f64; N];\n\
                 \x20                       for i in 0..N {{ s_col[i] = state.s[i][k]; s_row[i] = state.s[k][i]; }}\n\
                 \x20                       for i in 0..N {{\n\
                 \x20                           let sc = scale * s_col[i];\n\
                 \x20                           for j in 0..N {{ state.s[i][j] -= sc * s_row[j]; }}\n\
                 \x20                       }}\n\
                 \x20                       state.a[k][k] = new_a_kk;\n\
                 \x20                       state.a_neg[k][k] = alpha * state.c_work[k][k] - state.g_work[k][k];\n",
            ));
            if m > 0 {
                code.push_str(
                    "                       let mut nv_su = [0.0f64; M];\n\
                     \x20                       let mut u_ni = [0.0f64; M];\n\
                     \x20                       for i in 0..M {\n\
                     \x20                           for p in 0..N { nv_su[i] += N_V[i][p] * s_col[p]; }\n\
                     \x20                           for p in 0..N { u_ni[i] += s_row[p] * N_I[p][i]; }\n\
                     \x20                       }\n\
                     \x20                       for i in 0..M {\n\
                     \x20                           let sc = scale * nv_su[i];\n\
                     \x20                           for j in 0..M { state.k[i][j] -= sc * u_ni[j]; }\n\
                     \x20                       }\n\
                     \x20                       for i in 0..N {\n\
                     \x20                           let sc = scale * s_col[i];\n\
                     \x20                           for j in 0..M { state.s_ni[i][j] -= sc * u_ni[j]; }\n\
                     \x20                       }\n",
                );
            }
            code.push_str("                   }\n                }\n");
        }

        // TODO: SM for coupled inductors and transformer groups would go here.
        // For now, those types trigger a full rebuild (they're rarer and more complex).
        if !ir.saturating_coupled.is_empty() || !ir.saturating_xfmr_groups.is_empty() {
            code.push_str("                // Coupled/transformer: full rebuild (complex multi-entry update)\n");
            code.push_str("                state.rebuild_matrices(state.current_sample_rate);\n");
        }

        if has_any_saturation {
            code.push_str("            }\n"); // close else branch
            code.push_str("        }\n"); // close if sat_changed
        }

        // Close decimation block
        if has_any_saturation {
            code.push_str("    }\n");
            code.push_str("    state.sat_update_counter = state.sat_update_counter.saturating_sub(1);\n\n");
        }
        // Step 1: Build RHS = rhs_const + A_neg * v_prev + N_i * i_nl_prev + input (sparse)
        code.push_str(
            "    // Step 1: Build RHS (sparse A_neg * v_prev + sparse N_i * i_nl_prev)\n",
        );
        if ir.has_dc_sources {
            code.push_str("    let mut rhs = RHS_CONST;\n");
        } else {
            code.push_str("    let mut rhs = [0.0f64; N];\n");
        }
        // Sparse A_neg * v_prev
        for i in 0..n {
            let nz_cols = &ir.sparsity.a_neg.nz_by_row[i];
            if nz_cols.is_empty() {
                continue;
            }
            for &j in nz_cols {
                code.push_str(&format!(
                    "    rhs[{}] += state.a_neg[{}][{}] * state.v_prev[{}];\n",
                    i, i, j, j
                ));
            }
        }
        // Sparse N_i * i_nl_prev
        if m > 0 {
            for i in 0..n {
                for &j in &ir.sparsity.n_i.nz_by_row[i] {
                    code.push_str(&format!(
                        "    rhs[{}] += N_I[{}][{}] * state.i_nl_prev[{}];\n",
                        i, i, j, j
                    ));
                }
            }
        }
        // IIR op-amp RHS injection (PURE EXPLICIT — Gm is NOT in G):
        //   y_new = a1*y_prev + 2*b0*x_prev    (1-sample delay)
        //   rhs[o] += Go*(y_new + y_prev)      (full VCCS current, no matrix coupling)
        // At DC: y_ss = (2*b0/(1-a1))*x_ss = (Gm/Go)*x_ss = AOL*x_ss ✓
        for (idx, oa_iir) in ir.opamp_iir.iter().enumerate() {
            code.push_str(&format!(
                "    let oa{idx}_y_new = state.oa{idx}_a1 * state.oa{idx}_y_prev + 2.0 * state.oa{idx}_b0 * state.oa{idx}_x_prev;\n"
            ));
            code.push_str(&format!(
                "    rhs[{out}] += OA{idx}_GO * (oa{idx}_y_new + state.oa{idx}_y_prev);\n",
                out = oa_iir.out_idx
            ));
        }
        code.push('\n');

        // Input source (Thevenin, trapezoidal)
        code.push_str("    // Input source (Thevenin, trapezoidal)\n");
        code.push_str("    let input_conductance = 1.0 / INPUT_RESISTANCE;\n");
        code.push_str("    rhs[INPUT_NODE] += (input + state.input_prev) * input_conductance;\n");
        code.push_str("    state.input_prev = input;\n\n");

        // Step 2: Linear prediction v_pred = S * rhs (O(N^2))
        code.push_str("    // Step 2: Linear prediction v_pred = S * rhs (O(N^2))\n");
        code.push_str("    let mut v_pred = [0.0f64; N];\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        let mut sum = 0.0;\n");
        code.push_str("        for j in 0..N { sum += state.s[i][j] * rhs[j]; }\n");
        code.push_str("        v_pred[i] = sum;\n");
        code.push_str("    }\n\n");

        // Handle linear circuits (M=0): v_pred is the final answer
        if m == 0 {
            code.push_str("    // Linear circuit: v = v_pred (no NR needed)\n");
            code.push_str("    let v = v_pred;\n\n");
        } else {
            // Step 3: Extract device voltages p = N_v * v_pred (O(M*N))
            code.push_str("    // Step 3: Extract device voltages p = N_v * v_pred (sparse)\n");
            code.push_str("    let mut p = [0.0f64; M];\n");
            for i in 0..m {
                let nz_cols = &ir.sparsity.n_v.nz_by_row[i];
                if nz_cols.is_empty() {
                    continue;
                }
                let terms: Vec<String> = nz_cols
                    .iter()
                    .map(|&j| format!("N_V[{}][{}] * v_pred[{}]", i, j, j))
                    .collect();
                code.push_str(&format!("    p[{}] = {};\n", i, terms.join(" + ")));
            }
            code.push('\n');

            // Step 3c: MOSFET body effect update (from v_pred, before NR)
            for (dev_num, slot) in ir.device_slots.iter().enumerate() {
                if let DeviceParams::Mosfet(mp) = &slot.params {
                    if mp.has_body_effect() {
                        let vs_expr = if mp.source_node > 0 {
                            format!("v_pred[{}]", mp.source_node - 1)
                        } else {
                            "0.0".to_string()
                        };
                        let vb_expr = if mp.bulk_node > 0 {
                            format!("v_pred[{}]", mp.bulk_node - 1)
                        } else {
                            "0.0".to_string()
                        };
                        let sign = if mp.is_p_channel { -1.0 } else { 1.0 };
                        code.push_str(&format!(
                            "    {{ // MOSFET {dev_num} body effect\n\
                             \x20       let vsb = ({sign:.1}) * ({vs_expr} - {vb_expr});\n\
                             \x20       state.device_{dev_num}_vt = DEVICE_{dev_num}_VT + DEVICE_{dev_num}_GAMMA * ((DEVICE_{dev_num}_PHI + vsb.max(0.0)).sqrt() - DEVICE_{dev_num}_PHI.sqrt());\n\
                             \x20   }}\n"
                        ));
                    }
                }
            }

            // Step 4: M-dim NR (same structure as DK solve_nonlinear)
            code.push_str("    // Step 4: M-dim Newton-Raphson (Schur complement)\n");
            code.push_str("    // First-order predictor warm start\n");
            code.push_str("    let mut i_nl = [0.0f64; M];\n");
            code.push_str("    for i in 0..M {\n");
            code.push_str(
                "        i_nl[i] = 2.0 * state.i_nl_prev[i] - state.i_nl_prev_prev[i];\n",
            );
            code.push_str("    }\n");
            code.push_str("    let mut converged = false;\n");
            code.push_str("    state.last_nr_iterations = MAX_ITER as u32;\n\n");

            // Trapezoidal NR loop
            code.push_str("    for iter in 0..MAX_ITER {\n");

            // 4a. Compute v_d = p + K * i_nl
            code.push_str("        // 4a. Compute controlling voltages: v_d = p + K * i_nl\n");
            for i in 0..m {
                code.push_str(&format!("        let v_d{} = p[{}]", i, i));
                for &j in &ir.sparsity.k.nz_by_row[i] {
                    code.push_str(&format!(" + state.k[{}][{}] * i_nl[{}]", i, j, j));
                }
                code.push_str(";\n");
            }
            code.push('\n');

            // 4b. Evaluate device currents and Jacobian (reuse DK style)
            code.push_str("        // 4b. Evaluate device currents and Jacobians\n");
            for (dev_num, slot) in ir.device_slots.iter().enumerate() {
                Self::emit_dk_device_eval_for_nodal_schur(&mut code, dev_num, slot)?;
            }
            code.push('\n');

            // 4c. Residuals
            code.push_str("        // 4c. Residuals: f(i) = i_nl - i_dev = 0\n");
            for i in 0..m {
                code.push_str(&format!("        let f{} = i_nl[{}] - i_dev{};\n", i, i, i));
            }
            code.push('\n');

            // 4d. NR Jacobian: J[i][j] = delta_ij - sum_k(jdev_ik * K[k][j])
            code.push_str("        // 4d. Jacobian: J[i][j] = delta_ij - jdev * K\n");
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
                    code.push_str(&format!("        let j{}{} = {}{};\n", i, j, diag, terms));
                }
            }
            code.push('\n');

            // 4e. Solve the M×M linear system (1×1, 2×2 Cramer, 3..16 Gauss)
            // Uses `break` on convergence (not `return i_nl` like DK's solve_nonlinear)
            match m {
                1 => {
                    code.push_str("        // Solve 1x1: delta = f / J\n");
                    code.push_str("        let det = j00;\n");
                    code.push_str("        if det.abs() < 1e-15 {\n");
                    emit_nr_singular_fallback(&mut code, 1, "            ");
                    code.push_str("            continue;\n");
                    code.push_str("        }\n");
                    code.push_str("        let delta0 = f0 / det;\n\n");
                    emit_schur_nr_limit_and_converge(&mut code, ir, 1, "        ", "state.k");
                }
                2 => {
                    code.push_str("        // Solve 2x2 (Cramer's rule)\n");
                    code.push_str("        let det = j00 * j11 - j01 * j10;\n");
                    code.push_str("        if det.abs() < 1e-15 {\n");
                    emit_nr_singular_fallback(&mut code, 2, "            ");
                    code.push_str("            continue;\n");
                    code.push_str("        }\n");
                    code.push_str("        let inv_det = 1.0 / det;\n");
                    code.push_str("        let delta0 = inv_det * (j11 * f0 - j01 * f1);\n");
                    code.push_str("        let delta1 = inv_det * (-j10 * f0 + j00 * f1);\n\n");
                    emit_schur_nr_limit_and_converge(&mut code, ir, 2, "        ", "state.k");
                }
                3..=16 => {
                    Self::generate_schur_gauss_elim(&mut code, ir, m);
                }
                _ => {
                    return Err(CodegenError::UnsupportedTopology(format!(
                        "M={} not supported (max {})",
                        m,
                        crate::dk::MAX_M
                    )));
                }
            }

            code.push_str("    }\n\n"); // end trapezoidal NR loop

            // Step 5: Recover full v = v_pred + S_NI * i_nl
            code.push_str("    // Step 5: Recover full node voltages: v = v_pred + S_NI * i_nl\n");
            code.push_str("    let mut v = v_pred;\n");
            code.push_str("    for i in 0..N {\n");
            code.push_str("        for j in 0..M { v[i] += state.s_ni[i][j] * i_nl[j]; }\n");
            code.push_str("    }\n");

            // Op-amp supply rail handling.
            //
            // * `Hard`  — apply the post-NR `v[out].clamp(VEE, VCC)` mutation
            //             (matches pre-2026-04 behavior). This corrupts cap
            //             history for AC-coupled downstream stages; only use
            //             on circuits with DC-coupled downstream.
            // * `ActiveSet` — call `emit_nodal_active_set_resolve` against
            //             `state.a` (trapezoidal). Detects rail violations
            //             and pins them via row/column elimination, then
            //             re-solves the whole network so KCL is satisfied at
            //             every node with the clamped outputs. Preserves the
            //             steady DC rail value the op-amp converged to —
            //             required for control-path op-amps where the rail
            //             value drives a nonlinear device's operating point
            //             (VCR ALC sidechain → VCA control). May develop a
            //             Nyquist-rate limit cycle on audio-path op-amps
            //             whose output is cap-coupled to a downstream stage
            //             that integrates the rail behavior — for those use
            //             ActiveSetBe.
            // * `ActiveSetBe` — detect rail violations here without mutating;
            //             if any are detected, fall through to the BE fallback
            //             below (which re-runs NR with backward-Euler matrices
            //             and then applies the active-set row/col elimination
            //             using `state.a_be`). Trapezoidal + pin develops a
            //             Nyquist-rate limit cycle when the clamp is engaged
            //             across multiple samples on audio-path op-amps
            //             (cap-history term `(2/T)·C·v_prev` alternates sign
            //             every sample); BE damps this. The auto-detector
            //             picks ActiveSetBe over ActiveSet for audio-path
            //             topologies (no R-only path from op-amp output to a
            //             nonlinear device terminal).
            // * `BoyleDiodes` — physical catch diodes are already in the
            //             MNA via `augment_netlist_with_boyle_diodes`. NR
            //             handles saturation naturally through the diode
            //             exponential, producing a soft knee. Emit nothing
            //             here.
            // * `None`  — no clamping; caller accepts unbounded output.
            use crate::codegen::OpampRailMode;
            let active_set_be_mode = matches!(
                ir.solver_config.opamp_rail_mode,
                OpampRailMode::ActiveSetBe
            );
            if active_set_be_mode {
                code.push_str("    let mut active_set_engaged = false;\n");
            }
            match ir.solver_config.opamp_rail_mode {
                OpampRailMode::Hard => {
                    for oa in &ir.opamps {
                        // Skip op-amps that only appear in OpampIR for
                        // slew-rate limiting (VCC/VEE both infinite).
                        if !oa.vclamp_lo.is_finite() && !oa.vclamp_hi.is_finite() {
                            continue;
                        }
                        code.push_str(&format!(
                            "    v[{idx}] = v[{idx}].clamp({lo:.17e}, {hi:.17e});\n",
                            idx = oa.n_out_idx,
                            lo = oa.vclamp_lo,
                            hi = oa.vclamp_hi,
                        ));
                    }
                }
                OpampRailMode::ActiveSet => {
                    // Original trap+pin behavior — preserves steady DC rail
                    // for control-path topologies (e.g. VCR ALC sidechain).
                    Self::emit_nodal_active_set_resolve(
                        &mut code,
                        ir,
                        "    ",
                        "state.a",
                        "rhs",
                    );
                }
                OpampRailMode::ActiveSetBe => {
                    // Detect-only here; resolve happens in BE fallback below.
                    Self::emit_nodal_active_set_check(
                        &mut code,
                        ir,
                        "    ",
                        "active_set_engaged",
                    );
                }
                OpampRailMode::BoyleDiodes => {
                    // Catch diodes are physically in the circuit — no extra
                    // post-NR mutation needed.
                }
                OpampRailMode::None => {
                    // No clamping — caller accepts unbounded op-amp output.
                }
                OpampRailMode::Auto => {
                    // resolve_opamp_rail_mode() is responsible for converting
                    // Auto to a concrete mode before reaching the emitter.
                    unreachable!(
                        "OpampRailMode::Auto should have been resolved in ir::from_mna; \
                         emitter should only see concrete modes"
                    );
                }
            }

            // Convergence check: NR must have converged AND, in ActiveSetBe
            // mode, no op-amp output may have engaged its rail. The latter
            // triggers the BE fallback so the cap history stays consistent.
            if active_set_be_mode {
                code.push_str(
                    "    let mut converged = state.last_nr_iterations < MAX_ITER as u32 \
                     && !active_set_engaged;\n\n",
                );
            } else {
                code.push_str(
                    "    let converged = state.last_nr_iterations < MAX_ITER as u32;\n\n",
                );
            }

            // ActiveSetBe sub-stepping: when trap NR converged but the op-amp
            // output is railed, sub-step at 2x the sample rate using the trap
            // rule with ActiveSet pin at each sub-step. At the finer timestep
            // the discrete-time LC resonator (C15 + R network in the Klon)
            // has its Nyquist at 2× the original, so the Δv pulse from the
            // pin excites a mode that decays instead of sustaining.
            //
            // This matches the mechanism behind ngspice's adaptive timestep:
            // smaller dt at clipping transitions naturally damps the trap-rule
            // Nyquist artifact. The sub-stepping fires only on rail-engaged
            // samples, so linear-regime performance is unaffected.
            //
            // The sub-steps use the LINEAR prediction with frozen i_nl (the
            // NR already converged; the nonlinear devices barely change across
            // half-steps). Cost: 2 × O(N²) matvec per rail-engaged sample.
            if active_set_be_mode && !ir.matrices.s_sub.is_empty() {
                code.push_str("    if !converged && active_set_engaged && state.last_nr_iterations < MAX_ITER as u32 {\n");
                code.push_str("        // Sub-step at 2× rate using precomputed Schur matrices.\n");
                code.push_str("        // At the finer timestep the discrete-time LC resonator from\n");
                code.push_str("        // the ActiveSet pin has its Nyquist at 2× the audio Nyquist,\n");
                code.push_str("        // so the artifact decays instead of sustaining. Cost: 2 × O(N²)\n");
                code.push_str("        // matvec + O(M³) NR per sub-step — same as the normal Schur path.\n");
                code.push_str("        const N_SUB: usize = 2;\n");
                code.push_str("        let mut v_sub = state.v_prev;\n");
                code.push_str("        let mut i_nl_sub = state.i_nl_prev;\n");
                code.push_str("        let input_step = (input - state.input_prev) / N_SUB as f64;\n");
                code.push_str("        for step in 0..N_SUB {\n");
                code.push_str("            let inp_s = state.input_prev + input_step * (step + 1) as f64;\n");
                code.push_str("            let inp_prev_s = state.input_prev + input_step * step as f64;\n");
                // Build RHS: A_neg_sub * v_sub + N_i * i_nl_sub + input + rhs_const
                code.push_str("            let mut rhs_s = [0.0f64; N];\n");
                if ir.has_dc_sources {
                    code.push_str("            for i in 0..N { rhs_s[i] = RHS_CONST[i]; }\n");
                }
                code.push_str("            for i in 0..N { for j in 0..N { rhs_s[i] += state.a_neg_sub[i][j] * v_sub[j]; } }\n");
                code.push_str(&emit_sparse_ni_matvec_add(ir, "rhs_s", "i_nl_sub", "            "));
                code.push_str("            rhs_s[INPUT_NODE] += (inp_s + inp_prev_s) * input_conductance;\n");
                // Linear prediction: v_pred_s = S_sub * rhs_s (O(N²))
                code.push_str("            let mut v_pred_s = [0.0f64; N];\n");
                code.push_str("            for i in 0..N { for j in 0..N { v_pred_s[i] += state.s_sub[i][j] * rhs_s[j]; } }\n");
                // Extract device voltages: p_s = N_v * v_pred_s
                code.push_str("            let mut p_s = [0.0f64; M];\n");
                code.push_str("            for i in 0..M { for j in 0..N { p_s[i] += N_V[i][j] * v_pred_s[j]; } }\n");
                // Schur NR with K_sub (same iteration as normal path but with sub-step kernel)
                code.push_str("            i_nl_sub = state.i_nl_prev;\n");
                code.push_str("            for _iter in 0..MAX_ITER {\n");
                code.push_str("                let mut v_nl = [0.0f64; M];\n");
                code.push_str("                for i in 0..M { v_nl[i] = p_s[i]; for j in 0..M { v_nl[i] += state.k_sub[i][j] * i_nl_sub[j]; } }\n");
                code.push_str("                let mut i_nl = [0.0f64; M];\n");
                code.push_str("                let mut j_dev = [0.0f64; M * M];\n");
                Self::emit_nodal_device_evaluation_body(&mut code, ir, "                ");
                code.push_str("                let mut nr_ok = true;\n");
                code.push_str("                for i in 0..M { let d = i_nl[i] - i_nl_sub[i]; if d.abs() > TOL + 1e-3 * i_nl[i].abs() { nr_ok = false; } }\n");
                code.push_str("                i_nl_sub = i_nl;\n");
                code.push_str("                if nr_ok { break; }\n");
                code.push_str("            }\n");
                // Recover full v: v_sub = v_pred_s + S_NI_sub * i_nl_sub
                code.push_str("            v_sub = v_pred_s;\n");
                code.push_str("            for i in 0..N { for j in 0..M { v_sub[i] += state.s_ni_sub[i][j] * i_nl_sub[j]; } }\n");
                // Hard clamp op-amp outputs at each sub-step. At 2× rate the
                // clamp artifact is above audio Nyquist and decays naturally.
                // Downstream nodes are NOT re-solved (to avoid Nyquist
                // reintroduction from S_sub column propagation); the elevated
                // downstream voltages are handled by output scaling.
                for oa in &ir.opamps {
                    if !oa.vclamp_lo.is_finite() && !oa.vclamp_hi.is_finite() {
                        continue;
                    }
                    code.push_str(&format!(
                        "            v_sub[{idx}] = v_sub[{idx}].clamp({lo:.17e}, {hi:.17e});\n",
                        idx = oa.n_out_idx, lo = oa.vclamp_lo, hi = oa.vclamp_hi,
                    ));
                }
                code.push_str("        }\n");
                code.push_str("        v = v_sub;\n");
                code.push_str("        i_nl = i_nl_sub;\n");
                code.push_str("        converged = true;\n");
                code.push_str("        state.diag_substep_count += 1;\n");
                code.push_str("    }\n\n");
            }

            // Backward Euler fallback (also fires when active-set engaged in
            // ActiveSetBe mode — see comment above).
            code.push_str("    // Backward Euler fallback\n");
            code.push_str("    if !converged {\n");
            code.push_str("        state.diag_nr_max_iter_count += 1;\n");
            code.push_str("        state.diag_be_fallback_count += 1;\n\n");

            // Rebuild RHS with BE matrices
            code.push_str("        // Rebuild RHS with backward Euler matrices\n");
            code.push_str("        let mut rhs_be = [0.0f64; N];\n");
            code.push_str("        for i in 0..N {\n");
            if ir.has_dc_sources && !ir.matrices.rhs_const_be.is_empty() {
                code.push_str("            let mut sum = RHS_CONST_BE[i];\n");
            } else {
                code.push_str("            let mut sum = 0.0;\n");
            }
            code.push_str(
                "            for j in 0..N { sum += state.a_neg_be[i][j] * state.v_prev[j]; }\n",
            );
            code.push_str("            for j in 0..M { sum += N_I[i][j] * state.i_nl_prev[j]; }\n");
            code.push_str("            rhs_be[i] = sum;\n");
            code.push_str("        }\n");
            code.push_str("        rhs_be[INPUT_NODE] += input * input_conductance;\n\n");

            // BE linear prediction
            code.push_str("        // BE linear prediction: v_pred_be = S_be * rhs_be\n");
            code.push_str("        let mut v_pred_be = [0.0f64; N];\n");
            code.push_str("        for i in 0..N {\n");
            code.push_str("            let mut sum = 0.0;\n");
            code.push_str("            for j in 0..N { sum += state.s_be[i][j] * rhs_be[j]; }\n");
            code.push_str("            v_pred_be[i] = sum;\n");
            code.push_str("        }\n\n");

            // BE device voltages
            code.push_str("        let mut p_be = [0.0f64; M];\n");
            code.push_str("        for i in 0..M {\n");
            code.push_str("            let mut sum = 0.0;\n");
            code.push_str("            for j in 0..N { sum += N_V[i][j] * v_pred_be[j]; }\n");
            code.push_str("            p_be[i] = sum;\n");
            code.push_str("        }\n\n");

            // BE NR loop (use k_be, s_ni_be)
            code.push_str("        // Reset i_nl to predictor for BE attempt\n");
            code.push_str("        for i in 0..M {\n");
            code.push_str(
                "            i_nl[i] = 2.0 * state.i_nl_prev[i] - state.i_nl_prev_prev[i];\n",
            );
            code.push_str("        }\n\n");

            code.push_str("        for _iter in 0..MAX_ITER {\n");

            // v_d = p_be + K_be * i_nl
            for i in 0..m {
                code.push_str(&format!("            let v_d{} = p_be[{}]", i, i));
                for j in 0..m {
                    code.push_str(&format!(" + state.k_be[{}][{}] * i_nl[{}]", i, j, j));
                }
                code.push_str(";\n");
            }
            code.push('\n');

            // Evaluate devices (reuse same functions)
            for (dev_num, slot) in ir.device_slots.iter().enumerate() {
                Self::emit_dk_device_eval_for_nodal_schur_indented(
                    &mut code,
                    dev_num,
                    slot,
                    "            ",
                )?;
            }
            code.push('\n');

            // Residuals
            for i in 0..m {
                code.push_str(&format!(
                    "            let f{} = i_nl[{}] - i_dev{};\n",
                    i, i, i
                ));
            }
            code.push('\n');

            // Jacobian (using k_be)
            for i in 0..m {
                let slot = ir
                    .device_slots
                    .iter()
                    .find(|s| i >= s.start_idx && i < s.start_idx + s.dimension)
                    .ok_or_else(|| {
                        CodegenError::InvalidConfig(format!(
                            "No device slot found for NR dimension index {}",
                            i
                        ))
                    })?;
                let blk_start = slot.start_idx;
                let blk_dim = slot.dimension;
                for j in 0..m {
                    let diag = if i == j { "1.0" } else { "0.0" };
                    let mut terms = String::new();
                    for k in blk_start..blk_start + blk_dim {
                        terms.push_str(&format!(" - jdev_{}_{} * state.k_be[{}][{}]", i, k, k, j));
                    }
                    code.push_str(&format!(
                        "            let j{}{} = {}{};\n",
                        i, j, diag, terms
                    ));
                }
            }
            code.push('\n');

            // Solve (same structure but at 12-space indent)
            match m {
                1 => {
                    code.push_str("            let det = j00;\n");
                    code.push_str("            if det.abs() < 1e-15 { i_nl[0] -= (f0 * 0.5).clamp(-0.01, 0.01); continue; }\n");
                    code.push_str("            let delta0 = f0 / det;\n");
                    // Simple inline limit+converge for BE
                    Self::emit_be_nr_limit_and_converge(&mut code, ir, m, "            ");
                }
                2 => {
                    code.push_str("            let det = j00 * j11 - j01 * j10;\n");
                    code.push_str("            if det.abs() < 1e-15 { i_nl[0] -= (f0 * 0.5).clamp(-0.01, 0.01); i_nl[1] -= (f1 * 0.5).clamp(-0.01, 0.01); continue; }\n");
                    code.push_str("            let inv_det = 1.0 / det;\n");
                    code.push_str("            let delta0 = inv_det * (j11 * f0 - j01 * f1);\n");
                    code.push_str("            let delta1 = inv_det * (-j10 * f0 + j00 * f1);\n");
                    Self::emit_be_nr_limit_and_converge(&mut code, ir, m, "            ");
                }
                3..=16 => {
                    // Inline Gaussian elimination for BE
                    code.push_str("            let mut a = [\n");
                    for i in 0..m {
                        let row = (0..m)
                            .map(|j| format!("j{i}{j}"))
                            .collect::<Vec<_>>()
                            .join(", ");
                        code.push_str(&format!("                [{row}],\n"));
                    }
                    code.push_str("            ];\n");
                    let b_init = (0..m)
                        .map(|i| format!("f{i}"))
                        .collect::<Vec<_>>()
                        .join(", ");
                    code.push_str(&format!("            let mut b = [{b_init}];\n"));
                    code.push_str(&format!(
                        "            let mut singular = false;\n\
                         \x20           for col in 0..{m} {{\n\
                         \x20               let mut max_row = col;\n\
                         \x20               let mut max_val = a[col][col].abs();\n\
                         \x20               for row in (col+1)..{m} {{\n\
                         \x20                   if a[row][col].abs() > max_val {{ max_val = a[row][col].abs(); max_row = row; }}\n\
                         \x20               }}\n\
                         \x20               if max_val < 1e-15 {{ singular = true; break; }}\n\
                         \x20               if max_row != col {{ a.swap(col, max_row); b.swap(col, max_row); }}\n\
                         \x20               let pivot = a[col][col];\n\
                         \x20               for row in (col+1)..{m} {{\n\
                         \x20                   let factor = a[row][col] / pivot;\n\
                         \x20                   for j in (col+1)..{m} {{ a[row][j] -= factor * a[col][j]; }}\n\
                         \x20                   b[row] -= factor * b[col];\n\
                         \x20               }}\n\
                         \x20           }}\n"
                    ));
                    code.push_str(&format!(
                        "            if !singular {{\n\
                         \x20               for i in (0..{m}).rev() {{\n\
                         \x20                   let mut sum = b[i];\n\
                         \x20                   for j in (i+1)..{m} {{ sum -= a[i][j] * b[j]; }}\n\
                         \x20                   if a[i][i].abs() < 1e-15 {{ singular = true; break; }}\n\
                         \x20                   b[i] = sum / a[i][i];\n\
                         \x20               }}\n\
                         \x20           }}\n"
                    ));
                    code.push_str("            if !singular {\n");
                    for i in 0..m {
                        code.push_str(&format!("                let delta{i} = b[{i}];\n"));
                    }
                    Self::emit_be_nr_limit_and_converge(&mut code, ir, m, "                ");
                    code.push_str("            } else {\n");
                    for i in 0..m {
                        code.push_str(&format!(
                            "                i_nl[{i}] -= (f{i} * 0.5).clamp(-0.01, 0.01);\n"
                        ));
                    }
                    code.push_str("            }\n");
                }
                _ => {}
            }

            code.push_str("        }\n\n"); // end BE NR loop

            // Recover full v from BE
            code.push_str("        // Recover full v from BE solve\n");
            code.push_str("        v = v_pred_be;\n");
            code.push_str("        for i in 0..N {\n");
            code.push_str("            for j in 0..M { v[i] += state.s_ni_be[i][j] * i_nl[j]; }\n");
            code.push_str("        }\n");

            // Op-amp supply rail handling on the BE result. ActiveSetBe mode
            // runs the constrained re-solve against `state.a_be` so the cap
            // history (built from BE matrices) stays KCL-consistent. ActiveSet
            // mode is not normally reachable here (its trap path doesn't trip
            // BE on engagement), but if NR genuinely failed and substep didn't
            // recover, fall back to ActiveSet on BE matrices too. Hard mode
            // falls back to a plain post-recovery clamp (only safe for
            // DC-coupled downstream stages — see opamp_rail_clamp_bug.md).
            match ir.solver_config.opamp_rail_mode {
                OpampRailMode::ActiveSetBe | OpampRailMode::ActiveSet => {
                    Self::emit_nodal_active_set_resolve(
                        &mut code,
                        ir,
                        "        ",
                        "state.a_be",
                        "rhs_be",
                    );
                }
                OpampRailMode::Hard | OpampRailMode::None => {
                    for oa in &ir.opamps {
                        if !oa.vclamp_lo.is_finite() && !oa.vclamp_hi.is_finite() {
                            continue;
                        }
                        code.push_str(&format!(
                            "        v[{idx}] = v[{idx}].clamp({lo:.17e}, {hi:.17e});\n",
                            idx = oa.n_out_idx, lo = oa.vclamp_lo, hi = oa.vclamp_hi,
                        ));
                    }
                }
                OpampRailMode::BoyleDiodes => {
                    // Catch diodes are physically in the circuit; no post-NR
                    // mutation. (BE NR converged with the diodes active, so v
                    // is already saturation-bounded.)
                }
                OpampRailMode::Auto => unreachable!(
                    "OpampRailMode::Auto should have been resolved in ir::from_mna"
                ),
            }

            code.push_str("    }\n\n"); // end BE fallback block
        }

        // NaN check BEFORE state update (prevents corruption of v_prev/i_nl_prev)
        code.push_str("    if !v.iter().all(|x| x.is_finite()) {\n");
        code.push_str("        state.v_prev = state.dc_operating_point;\n");
        if ir.dc_nl_currents.iter().any(|&v| v != 0.0) {
            code.push_str("        state.i_nl_prev = DC_NL_I;\n");
            code.push_str("        state.i_nl_prev_prev = DC_NL_I;\n");
        } else {
            code.push_str("        state.i_nl_prev = [0.0; M];\n");
            code.push_str("        state.i_nl_prev_prev = [0.0; M];\n");
        }
        code.push_str("        state.input_prev = 0.0;\n");
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            if let crate::codegen::ir::DeviceParams::Bjt(bp) = &slot.params {
                if bp.has_self_heating() {
                    code.push_str(&format!(
                        "        state.device_{dev_num}_tj = DEVICE_{dev_num}_TAMB;\n\
                         \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS;\n\
                         \x20       state.device_{dev_num}_vt = DEVICE_{dev_num}_VT;\n"
                    ));
                }
            }
        }
        // Reset IIR op-amp state on NaN
        for (idx, _oa_iir) in ir.opamp_iir.iter().enumerate() {
            code.push_str(&format!(
                "        state.oa{idx}_x_prev = 0.0;\n\
                 \x20       state.oa{idx}_y_prev = 0.0;\n"
            ));
        }
        code.push_str("        state.diag_nan_reset_count += 1;\n");
        code.push_str("        return [0.0; NUM_OUTPUTS];\n");
        code.push_str("    }\n\n");

        // Op-amp slew-rate limiting (nodal Schur path). Clamp the per-sample
        // voltage delta at each op-amp output node to ±SR*dt. This is
        // mathematically equivalent to clamping the Boyle dominant-pole
        // integrator input current to ±I_slew = ±SR*C_dom: the per-sample
        // voltage step of an integrator with current `i_in` through cap
        // `C_dom` is `Δv = (i_in*dt)/C_dom`, so capping `|Δv| ≤ SR*dt`
        // caps `|i_in| ≤ SR*C_dom`. Emitted only for op-amps with finite
        // SR; circuits without `SR=` in the .model generate identical code.
        Self::emit_opamp_slew_limit(&mut code, ir, "    ", "v");

        // State update
        code.push_str("    // State update\n");
        code.push_str("    state.v_prev = v;\n");
        if m > 0 {
            code.push_str("    state.i_nl_prev_prev = state.i_nl_prev;\n");
            code.push_str("    state.i_nl_prev = i_nl;\n");
        }
        // IIR op-amp state update (PURE EXPLICIT):
        //   x_prev ← x_new (from converged v, for next sample's explicit injection)
        //   y_prev ← y_new (pre-computed before solve)
        for (idx, oa_iir) in ir.opamp_iir.iter().enumerate() {
            let x_expr = match (oa_iir.np_idx, oa_iir.nm_idx) {
                (Some(np), Some(nm)) => format!("v[{np}] - v[{nm}]"),
                (Some(np), None) => format!("v[{np}]"),
                (None, Some(nm)) => format!("-v[{nm}]"),
                (None, None) => "0.0".to_string(),
            };
            code.push_str(&format!(
                "    {{\n\
                 \x20       let x_new = {x_expr};\n\
                 \x20       state.oa{idx}_x_prev = x_new;\n\
                 \x20       state.oa{idx}_y_prev = oa{idx}_y_new;\n\
                 \x20   }}\n"
            ));
        }
        for (idx, _pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "    state.pot_{}_resistance_prev = state.pot_{}_resistance;\n",
                idx, idx
            ));
        }
        code.push('\n');

        // BJT self-heating thermal update
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            if let crate::codegen::ir::DeviceParams::Bjt(bp) = &slot.params {
                if bp.has_self_heating() {
                    let s = slot.start_idx;
                    let s1 = s + 1;
                    code.push_str(&format!(
                        "    {{ // BJT {dev_num} self-heating thermal update\n\
                         \x20       let ic = i_nl[{s}];\n\
                         \x20       let ib = i_nl[{s1}];\n\
                         \x20       let mut vbe_sum = 0.0f64;\n\
                         \x20       let mut vbc_sum = 0.0f64;\n\
                         \x20       for j in 0..N {{ vbe_sum += N_V[{s}][j] * v[j]; }}\n\
                         \x20       for j in 0..N {{ vbc_sum += N_V[{s1}][j] * v[j]; }}\n\
                         \x20       let vce = vbe_sum - vbc_sum;\n\
                         \x20       let p = vce * ic + vbe_sum * ib;\n\
                         \x20       let dt = 1.0 / SAMPLE_RATE;\n\
                         \x20       let d_tj = (p - (state.device_{dev_num}_tj - DEVICE_{dev_num}_TAMB) / DEVICE_{dev_num}_RTH) / DEVICE_{dev_num}_CTH * dt;\n\
                         \x20       state.device_{dev_num}_tj += d_tj;\n\
                         \x20       state.device_{dev_num}_tj = state.device_{dev_num}_tj.clamp(200.0, 500.0);\n\
                         \x20       state.device_{dev_num}_vt = BOLTZMANN_Q * state.device_{dev_num}_tj;\n\
                         \x20       let t_ratio = state.device_{dev_num}_tj / DEVICE_{dev_num}_TAMB;\n\
                         \x20       let vt_nom = BOLTZMANN_Q * DEVICE_{dev_num}_TAMB;\n\
                         \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS_NOM\n\
                         \x20           * t_ratio.powf(DEVICE_{dev_num}_XTI)\n\
                         \x20           * fast_exp((DEVICE_{dev_num}_EG / vt_nom) * (1.0 - DEVICE_{dev_num}_TAMB / state.device_{dev_num}_tj));\n\
                         \x20   }}\n"
                    ));
                }
            }
        }

        // (NaN check already done before state update)

        // Output extraction
        code.push_str("    // Extract outputs, DC blocking, and scaling\n");
        code.push_str("    let mut output = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let raw_out = v[OUTPUT_NODES[out_idx]];\n");
        code.push_str("        let raw_out = if raw_out.is_finite() { raw_out } else { 0.0 };\n");
        if ir.dc_block {
            code.push_str("        let dc_blocked = raw_out - state.dc_block_x_prev[out_idx]\n");
            code.push_str("            + state.dc_block_r * state.dc_block_y_prev[out_idx];\n");
            code.push_str("        state.dc_block_x_prev[out_idx] = raw_out;\n");
            code.push_str("        state.dc_block_y_prev[out_idx] = dc_blocked;\n");
            code.push_str("        let scaled = dc_blocked * OUTPUT_SCALES[out_idx];\n");
        } else {
            code.push_str("        let scaled = raw_out * OUTPUT_SCALES[out_idx];\n");
        }
        code.push_str("        let abs_out = scaled.abs();\n");
        code.push_str(
            "        if abs_out > state.diag_peak_output { state.diag_peak_output = abs_out; }\n",
        );
        if ir.dc_block {
            code.push_str("        if abs_out > 10.0 { state.diag_clamp_count += 1; }\n");
            code.push_str("        output[out_idx] = scaled.clamp(-10.0, 10.0);\n");
        } else {
            code.push_str(
                "        output[out_idx] = if scaled.is_finite() { scaled } else { 0.0 };\n",
            );
        }
        code.push_str("    }\n");
        code.push_str("    output\n");
        code.push_str("}\n\n");

        let _ = (num_outputs, n_nodes, has_pots);

        Ok(code)
    }

    /// Emit DK-style device evaluation for a single device in Schur NR (default indent "        ").
    fn emit_dk_device_eval_for_nodal_schur(
        code: &mut String,
        dev_num: usize,
        slot: &crate::codegen::ir::DeviceSlot,
    ) -> Result<(), CodegenError> {
        Self::emit_dk_device_eval_for_nodal_schur_indented(code, dev_num, slot, "        ")
    }

    /// Emit DK-style device evaluation for a single device at given indent.
    ///
    /// Declares `i_dev{s}` and `jdev_{i}_{j}` local variables matching the
    /// DK `solve_nonlinear` naming convention. Uses `v_d{s}` from the caller.
    fn emit_dk_device_eval_for_nodal_schur_indented(
        code: &mut String,
        dev_num: usize,
        slot: &crate::codegen::ir::DeviceSlot,
        indent: &str,
    ) -> Result<(), CodegenError> {
        use crate::codegen::ir::{DeviceParams, DeviceType};
        let s = slot.start_idx;
        let d = dev_num;

        match (&slot.device_type, &slot.params) {
            (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                if dp.has_rs() && dp.has_bv() {
                    code.push_str(&format!(
                        "{indent}let i_dev{s} = diode_current_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS) + diode_breakdown_current(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n\
                         {indent}let jdev_{s}_{s} = diode_conductance_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS) + diode_breakdown_conductance(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n"
                    ));
                } else if dp.has_rs() {
                    code.push_str(&format!(
                        "{indent}let i_dev{s} = diode_current_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS);\n\
                         {indent}let jdev_{s}_{s} = diode_conductance_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS);\n"
                    ));
                } else if dp.has_bv() {
                    code.push_str(&format!(
                        "{indent}let i_dev{s} = diode_current(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt) + diode_breakdown_current(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n\
                         {indent}let jdev_{s}_{s} = diode_conductance(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt) + diode_breakdown_conductance(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n"
                    ));
                } else {
                    code.push_str(&format!(
                        "{indent}let i_dev{s} = diode_current(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt);\n\
                         {indent}let jdev_{s}_{s} = diode_conductance(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt);\n"
                    ));
                }
            }
            (DeviceType::Bjt, DeviceParams::Bjt(bp)) => {
                let s1 = s + 1;
                if bp.has_parasitics() && !slot.has_internal_mna_nodes {
                    code.push_str(&format!(
                        "{indent}let (i_dev{s}, i_dev{s1}, bjt{d}_jac) = bjt_with_parasitics(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, DEVICE_{d}_NR, state.device_{d}_bf, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF, DEVICE_{d}_IKR, DEVICE_{d}_ISE, DEVICE_{d}_NE, DEVICE_{d}_ISC, DEVICE_{d}_NC, DEVICE_{d}_RB, DEVICE_{d}_RC, DEVICE_{d}_RE);\n"
                    ));
                } else {
                    // Combined evaluation: shared exp() across ic, ib, jacobian
                    // (parasitics handled by MNA internal nodes when has_internal_mna_nodes)
                    code.push_str(&format!(
                        "{indent}let (i_dev{s}, i_dev{s1}, bjt{d}_jac) = bjt_evaluate(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, DEVICE_{d}_NR, state.device_{d}_bf, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF, DEVICE_{d}_IKR, DEVICE_{d}_ISE, DEVICE_{d}_NE, DEVICE_{d}_ISC, DEVICE_{d}_NC);\n"
                    ));
                }
                code.push_str(&format!(
                    "{indent}let jdev_{s}_{s} = bjt{d}_jac[0];\n\
                     {indent}let jdev_{s}_{s1} = bjt{d}_jac[1];\n\
                     {indent}let jdev_{s1}_{s} = bjt{d}_jac[2];\n\
                     {indent}let jdev_{s1}_{s1} = bjt{d}_jac[3];\n"
                ));
            }
            (DeviceType::BjtForwardActive, DeviceParams::Bjt(_bp)) => {
                code.push_str(&format!(
                    "{indent}let vbe_{d} = v_d{s} * DEVICE_{d}_SIGN;\n\
                     {indent}let exp_be_{d} = fast_exp(vbe_{d} / (DEVICE_{d}_NF * state.device_{d}_vt));\n\
                     {indent}let i_dev{s} = state.device_{d}_is * (exp_be_{d} - 1.0) * DEVICE_{d}_SIGN;\n\
                     {indent}let jdev_{s}_{s} = state.device_{d}_is / (DEVICE_{d}_NF * state.device_{d}_vt) * exp_be_{d};\n"
                ));
            }
            (DeviceType::Jfet, DeviceParams::Jfet(jp)) => {
                let s1 = s + 1;
                code.push_str(&format!(
                    "{indent}let i_dev{s} = jfet_id(v_d{s1}, v_d{s}, state.device_{d}_idss, state.device_{d}_vp, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n\
                     {indent}let i_dev{s1} = jfet_ig(v_d{s1}, DEVICE_{d}_SIGN);\n"
                ));
                if jp.has_rd_rs() {
                    code.push_str(&format!(
                        "{indent}let jfet{d}_jac = jfet_jacobian_with_rd_rs(v_d{s1}, v_d{s}, state.device_{d}_idss, state.device_{d}_vp, state.device_{d}_lambda, DEVICE_{d}_SIGN, DEVICE_{d}_RD, DEVICE_{d}_RS);\n"
                    ));
                } else {
                    code.push_str(&format!(
                        "{indent}let jfet{d}_jac = jfet_jacobian(v_d{s1}, v_d{s}, state.device_{d}_idss, state.device_{d}_vp, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n"
                    ));
                }
                code.push_str(&format!(
                    "{indent}let jdev_{s}_{s} = jfet{d}_jac[1];\n\
                     {indent}let jdev_{s}_{s1} = jfet{d}_jac[0];\n\
                     {indent}let jdev_{s1}_{s} = jfet{d}_jac[3];\n\
                     {indent}let jdev_{s1}_{s1} = jfet{d}_jac[2];\n"
                ));
            }
            (DeviceType::Mosfet, DeviceParams::Mosfet(mp)) => {
                let s1 = s + 1;
                code.push_str(&format!(
                    "{indent}let i_dev{s} = mosfet_id(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n\
                     {indent}let i_dev{s1} = mosfet_ig(v_d{s1}, DEVICE_{d}_SIGN);\n"
                ));
                if mp.has_rd_rs() {
                    code.push_str(&format!(
                        "{indent}let mos{d}_jac = mosfet_jacobian_with_rd_rs(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN, DEVICE_{d}_RD, DEVICE_{d}_RS);\n"
                    ));
                } else {
                    code.push_str(&format!(
                        "{indent}let mos{d}_jac = mosfet_jacobian(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n"
                    ));
                }
                code.push_str(&format!(
                    "{indent}let jdev_{s}_{s} = mos{d}_jac[1];\n\
                     {indent}let jdev_{s}_{s1} = mos{d}_jac[0];\n\
                     {indent}let jdev_{s1}_{s} = mos{d}_jac[3];\n\
                     {indent}let jdev_{s1}_{s1} = mos{d}_jac[2];\n"
                ));
            }
            (DeviceType::Tube, DeviceParams::Tube(tp)) => {
                let s1 = s + 1;
                if tp.is_pentode() {
                    // Pentode / beam tetrode NR block. See
                    // [`pentode_dispatch`] for the 8-way helper family
                    // selection and [`emit_pentode_nr_dk_stamp`] for the
                    // shared (primary + BE fallback) DK Schur emitter.
                    emit_pentode_nr_dk_stamp(code, tp, d, s, indent);
                } else {
                    if tp.has_rgi() {
                        code.push_str(&format!(
                            "{indent}let (i_dev{s}, i_dev{s1}, tube{d}_jac) = tube_evaluate_with_rgi(v_d{s}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_ig_max, state.device_{d}_vgk_onset, state.device_{d}_lambda, DEVICE_{d}_RGI);\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}let (i_dev{s}, i_dev{s1}, tube{d}_jac) = tube_evaluate(v_d{s}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_ig_max, state.device_{d}_vgk_onset, state.device_{d}_lambda);\n"
                        ));
                    }
                    code.push_str(&format!(
                        "{indent}let jdev_{s}_{s} = tube{d}_jac[0];\n\
                         {indent}let jdev_{s}_{s1} = tube{d}_jac[1];\n\
                         {indent}let jdev_{s1}_{s} = tube{d}_jac[2];\n\
                         {indent}let jdev_{s1}_{s1} = tube{d}_jac[3];\n"
                    ));
                }
            }
            (DeviceType::Vca, DeviceParams::Vca(_vp)) => {
                let s1 = s + 1;
                code.push_str(&format!(
                    "{indent}let i_dev{s} = vca_current(v_d{s}, v_d{s1}, state.device_{d}_g0, state.device_{d}_vscale, DEVICE_{d}_THD);\n\
                     {indent}let i_dev{s1} = 0.0;\n\
                     {indent}let vca{d}_jac = vca_jacobian(v_d{s}, v_d{s1}, state.device_{d}_g0, state.device_{d}_vscale, DEVICE_{d}_THD);\n\
                     {indent}let jdev_{s}_{s} = vca{d}_jac[0];\n\
                     {indent}let jdev_{s}_{s1} = vca{d}_jac[1];\n\
                     {indent}let jdev_{s1}_{s} = vca{d}_jac[2];\n\
                     {indent}let jdev_{s1}_{s1} = vca{d}_jac[3];\n"
                ));
            }
            _ => {}
        }
        Ok(())
    }

    /// Emit voltage limiting + convergence + step for BE fallback NR in Schur mode.
    fn emit_be_nr_limit_and_converge(code: &mut String, ir: &CircuitIR, dim: usize, indent: &str) {
        // Compute voltage-space changes for limiting
        for i in 0..dim {
            code.push_str(&format!("{indent}let dv{i} = -("));
            let mut first = true;
            for j in 0..dim {
                if !first {
                    code.push_str(" + ");
                }
                code.push_str(&format!("state.k_be[{i}][{j}] * delta{j}"));
                first = false;
            }
            code.push_str(");\n");
        }

        // Compute per-dimension damping factor from per-device limiting
        code.push_str(&format!("{indent}let mut alpha = [1.0_f64; {dim}];\n"));
        code.push_str(&format!("{indent}let mut any_limited = false;\n"));
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            for d in 0..slot.dimension {
                let i = slot.start_idx + d;
                code.push_str(&format!("{indent}if dv{i}.abs() > 1e-15 {{\n"));
                match (&slot.device_type, d) {
                    (crate::codegen::ir::DeviceType::Diode, _) => {
                        code.push_str(&format!(
                            "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (crate::codegen::ir::DeviceType::Bjt, _)
                    | (crate::codegen::ir::DeviceType::BjtForwardActive, _) => {
                        code.push_str(&format!(
                            "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vt, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (crate::codegen::ir::DeviceType::Jfet, 0) | (crate::codegen::ir::DeviceType::Mosfet, 0) => {
                        code.push_str(&format!(
                            "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, 0.0);\n"
                        ));
                    }
                    (crate::codegen::ir::DeviceType::Jfet, _) => {
                        code.push_str(&format!(
                            "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vp);\n"
                        ));
                    }
                    (crate::codegen::ir::DeviceType::Mosfet, _) => {
                        code.push_str(&format!(
                            "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vt);\n"
                        ));
                    }
                    (crate::codegen::ir::DeviceType::Tube, 0) => {
                        code.push_str(&format!(
                            "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vgk_onset / 3.0, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (crate::codegen::ir::DeviceType::Tube, 2) => {
                        // Pentode dim 2 = Vg2k — log-junction limiting (see DK NR limiter).
                        code.push_str(&format!(
                            "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vgk_onset / 3.0, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (crate::codegen::ir::DeviceType::Tube, _) => {
                        code.push_str(&format!(
                            "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, 0.0);\n"
                        ));
                    }
                    (crate::codegen::ir::DeviceType::Vca, _) => {
                        // VCA: no junction limiting needed — fast_exp already clamps
                        code.push_str(&format!("{indent}    let v_lim = v_d{i} + dv{i};\n"));
                    }
                }
                code.push_str(&format!(
                    "{indent}    let ratio = ((v_lim - v_d{i}) / dv{i}).max(0.01);\n\
                     {indent}    if ratio < alpha[{i}] {{ alpha[{i}] = ratio; if ratio < 1.0 {{ any_limited = true; }} }}\n\
                     {indent}}}\n"
                ));
            }
        }

        // Enforce per-device grouping: both dims of a 2D device share the minimum alpha
        for slot in &ir.device_slots {
            if slot.dimension > 1 {
                let s = slot.start_idx;
                let s1 = s + 1;
                code.push_str(&format!(
                    "{indent}{{ let dev_alpha = alpha[{s}].min(alpha[{s1}]); alpha[{s}] = dev_alpha; alpha[{s1}] = dev_alpha; }}\n"
                ));
            }
        }

        // Global voltage backstop: adaptive limit based on DC operating point voltages
        let max_dc_v = ir
            .dc_operating_point
            .iter()
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max);
        let dv_limit = if max_dc_v > 20.0 {
            (max_dc_v * 0.15).max(3.5)
        } else {
            3.5
        };
        code.push_str(&format!(
            "{indent}// Global voltage backstop: limit max voltage change to {dv_limit:.1}V\n"
        ));
        code.push_str(&format!("{indent}let max_dv = "));
        for i in 0..dim {
            if i > 0 {
                code.push_str(&format!(".max((dv{i} * alpha[{i}]).abs())"));
            } else {
                code.push_str(&format!("(dv{i} * alpha[{i}]).abs()"));
            }
        }
        code.push_str(";\n");
        code.push_str(&format!(
            "{indent}if max_dv > {dv_limit:.6} {{ let factor = ({dv_limit:.6} / max_dv).max(0.1); for a in alpha.iter_mut() {{ *a *= factor; }} }}\n"
        ));

        // Apply damped step
        for i in 0..dim {
            code.push_str(&format!("{indent}i_nl[{i}] -= alpha[{i}] * delta{i};\n"));
        }

        // RELTOL convergence check
        code.push_str(&format!(
            "{indent}// Convergence check (SPICE RELTOL=0.001, VNTOL=1e-6)\n"
        ));
        code.push_str(&format!("{indent}if !any_limited {{\n"));
        code.push_str(&format!("{indent}    let mut nr_converged = true;\n"));
        for i in 0..dim {
            code.push_str(&format!(
                "{indent}    {{ let step = dv{i} * alpha[{i}]; let v_new = v_d{i} + step; let threshold = 1e-3 * v_d{i}.abs().max(v_new.abs()) + 1e-6; if step.abs() > threshold {{ nr_converged = false; }} }}\n"
            ));
        }
        code.push_str(&format!(
            "{indent}    if nr_converged {{ state.last_nr_iterations = _iter as u32; break; }}\n"
        ));
        code.push_str(&format!("{indent}}}\n"));
    }

    /// Emit LU solve function for the nodal solver (N x N with partial pivoting).
    /// Used by the full-LU codegen path (not Schur).
    pub(super) fn emit_nodal_lu_solve(_ir: &CircuitIR) -> String {
        let mut code = section_banner(
            "LU SOLVE (Equilibrated Gaussian elimination with iterative refinement)",
        );

        code.push_str("/// Solve A*x = b using equilibrated LU with partial pivoting + iterative refinement.\n");
        code.push_str("///\n");
        code.push_str(
            "/// Diagonal equilibration scales rows/cols by 1/sqrt(|A[i][i]|) to reduce\n",
        );
        code.push_str(
            "/// condition number. One round of iterative refinement corrects residual error.\n",
        );
        code.push_str(
            "/// Matches the runtime solver's `solve_equilibrated()` for numerical parity.\n",
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
        code.push_str("    // Row scaling: dr[i] = 1/max_j(|A[i][j]|)\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        let mut row_max = 0.0f64;\n");
        code.push_str("        for j in 0..N { let v = a[i][j].abs(); if v > row_max { row_max = v; } }\n");
        code.push_str("        if row_max > 1e-30 { dr[i] = 1.0 / row_max; }\n");
        code.push_str("    }\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N { a[i][j] *= dr[i]; }\n");
        code.push_str("    }\n");
        code.push_str("    // Column scaling: dc[j] = 1/max_i(|A[i][j]|) (after row scaling)\n");
        code.push_str("    for j in 0..N {\n");
        code.push_str("        let mut col_max = 0.0f64;\n");
        code.push_str("        for i in 0..N { let v = a[i][j].abs(); if v > col_max { col_max = v; } }\n");
        code.push_str("        if col_max > 1e-30 { dc[j] = 1.0 / col_max; }\n");
        code.push_str("    }\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N { a[i][j] *= dc[j]; }\n");
        code.push_str("    }\n\n");

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
    pub(super) fn emit_nodal_lu_factor(_ir: &CircuitIR) -> String {
        let mut code = String::new();

        code.push_str("/// LU factorization with asymmetric row/column equilibration and partial pivoting.\n");
        code.push_str("///\n");
        code.push_str("/// After this call, `a` contains the LU factors, `dr`/`dc` the row/column\n");
        code.push_str(
            "/// equilibration, and `perm` the row permutation. Use `lu_back_solve` to solve.\n",
        );
        code.push_str("#[inline(always)]\n");
        code.push_str("fn lu_factor(a: &mut [[f64; N]; N], dr: &mut [f64; N], dc: &mut [f64; N], perm: &mut [usize; N]) -> bool {\n");

        // Step 1: Asymmetric row/column equilibration
        code.push_str("    // Row scaling: dr[i] = 1/max_j(|A[i][j]|)\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        let mut row_max = 0.0f64;\n");
        code.push_str("        for j in 0..N { let v = a[i][j].abs(); if v > row_max { row_max = v; } }\n");
        code.push_str("        dr[i] = if row_max > 1e-30 { 1.0 / row_max } else { 1.0 };\n");
        code.push_str("    }\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N { a[i][j] *= dr[i]; }\n");
        code.push_str("    }\n");
        code.push_str("    // Column scaling: dc[j] = 1/max_i(|A[i][j]|) (after row scaling)\n");
        code.push_str("    for j in 0..N {\n");
        code.push_str("        let mut col_max = 0.0f64;\n");
        code.push_str("        for i in 0..N { let v = a[i][j].abs(); if v > col_max { col_max = v; } }\n");
        code.push_str("        dc[j] = if col_max > 1e-30 { 1.0 / col_max } else { 1.0 };\n");
        code.push_str("    }\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N { a[i][j] *= dc[j]; }\n");
        code.push_str("    }\n\n");

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
        code.push_str("/// `a_lu` contains LU factors from `lu_factor`. `dr`/`dc` and `perm` are the\n");
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
    /// `LuSparsity`. Each operation is one line of generated code. Static pivoting
    /// with a runtime growth-factor check and dense fallback.
    /// Emit compile-time sparse LU factorization (in-place on original indices).
    ///
    /// All operations reference original node indices. AMD ordering determines
    /// the ORDER of elimination, not the physical layout. No permutation arrays.
    fn emit_sparse_lu_factor(ir: &CircuitIR) -> String {
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
            "/// Returns false if a pivot is too small (fall back to dense lu_factor).\n",
        );
        code.push_str("#[inline(always)]\n");
        code.push_str("fn sparse_lu_factor(a: &mut [[f64; N]; N], dr: &mut [f64; N], dc: &mut [f64; N]) -> bool {\n");

        // Asymmetric row/column equilibration (dense — O(N²), cheap compared to factorization)
        code.push_str("    // Row scaling: dr[i] = 1/max_j(|A[i][j]|)\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        let mut row_max = 0.0f64;\n");
        code.push_str("        for j in 0..N { let v = a[i][j].abs(); if v > row_max { row_max = v; } }\n");
        code.push_str("        dr[i] = if row_max > 1e-30 { 1.0 / row_max } else { 1.0 };\n");
        code.push_str("    }\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N { a[i][j] *= dr[i]; }\n");
        code.push_str("    }\n");
        code.push_str("    // Column scaling: dc[j] = 1/max_i(|A[i][j]|) (after row scaling)\n");
        code.push_str("    for j in 0..N {\n");
        code.push_str("        let mut col_max = 0.0f64;\n");
        code.push_str("        for i in 0..N { let v = a[i][j].abs(); if v > col_max { col_max = v; } }\n");
        code.push_str("        dc[j] = if col_max > 1e-30 { 1.0 / col_max } else { 1.0 };\n");
        code.push_str("    }\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N { a[i][j] *= dc[j]; }\n");
        code.push_str("    }\n\n");

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

        code.push_str("\n    true\n");
        code.push_str("}\n\n");

        code
    }

    /// Emit compile-time sparse forward/backward substitution (original indices).
    ///
    /// Forward sub processes L entries in AMD elimination order.
    /// Backward sub processes U entries in reverse AMD order.
    /// No permutation arrays — order is baked into the emitted code.
    fn emit_sparse_lu_back_solve(ir: &CircuitIR) -> String {
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
                code.push_str(&format!(
                    "    {{ let tmp = x[{r1}]; x[{r1}] = x[{r2}]; x[{r2}] = tmp; }}\n"
                ));
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

    /// Emit the complete process_sample function for the nodal solver (O(N^3) LU path).
    ///
    /// Used when the Schur path is unstable: K ≈ 0 (device Jacobian provides
    /// essential damping not captured by Schur), positive K diagonal, or
    /// ill-conditioned K. Matches the runtime NodalSolver's solve_equilibrated().
    pub(super) fn emit_nodal_process_sample(ir: &CircuitIR) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let num_outputs = ir.solver_config.output_nodes.len();
        let os_factor = ir.solver_config.oversampling_factor;

        let mut code = section_banner("PROCESS SAMPLE (Full-nodal NR with LU solve)");

        // Function signature
        if os_factor > 1 {
            code.push_str("/// Process a single sample at the internal (oversampled) rate.\n");
            code.push_str("///\n");
            code.push_str("/// Called by `process_sample()` through the oversampling chain.\n");
            code.push_str("#[inline(always)]\n");
            code.push_str("fn process_sample_inner(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n");
        } else {
            code.push_str("/// Process a single audio sample through the circuit.\n");
            code.push_str("///\n");
            code.push_str(
                "/// Uses full-nodal Newton-Raphson with LU factorization per iteration.\n",
            );
            code.push_str("/// Includes backward Euler fallback for unconditional stability.\n");
            code.push_str("#[inline]\n");
            code.push_str("pub fn process_sample(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n");
        }

        // Input sanitization
        code.push_str(
            "    let input = if input.is_finite() { input.clamp(-100.0, 100.0) } else { 0.0 };\n\n",
        );

        // Lazy rebuild: batch all pot/switch changes into one matrix rebuild
        let has_pots = !ir.pots.is_empty();
        let has_switches = !ir.switches.is_empty();
        if has_pots || has_switches {
            code.push_str(
                "    // Lazy rebuild: batch all pot/switch changes into one matrix rebuild\n\
                 \x20   if state.matrices_dirty {\n\
                 \x20       state.rebuild_matrices(state.current_sample_rate);\n\
                 \x20       state.matrices_dirty = false;\n\
                 \x20   }\n\n",
            );
        }

        // Flush denormals in state vectors (prevents 50-100x CPU penalty during silence)
        code.push_str("    for v in state.v_prev.iter_mut() { *v = *v + 1e-25 - 1e-25; }\n");
        if m > 0 {
            code.push_str("    for v in state.i_nl_prev.iter_mut() { *v = *v + 1e-25 - 1e-25; }\n");
        }
        code.push('\n');

        // Step 1: Build RHS = rhs_const + A_neg * v_prev + N_i * i_nl_prev + input (sparse)
        code.push_str(
            "    // Step 1: Build RHS (sparse A_neg * v_prev + sparse N_i * i_nl_prev)\n",
        );
        if ir.has_dc_sources {
            code.push_str("    let mut rhs = RHS_CONST;\n");
        } else {
            code.push_str("    let mut rhs = [0.0f64; N];\n");
        }
        // Sparse A_neg * v_prev
        for i in 0..n {
            let nz_cols = &ir.sparsity.a_neg.nz_by_row[i];
            if nz_cols.is_empty() {
                continue;
            }
            for &j in nz_cols {
                code.push_str(&format!(
                    "    rhs[{}] += state.a_neg[{}][{}] * state.v_prev[{}];\n",
                    i, i, j, j
                ));
            }
        }
        // Sparse N_i * i_nl_prev (N_I is N×M, direct row access)
        if m > 0 {
            for i in 0..n {
                for &j in &ir.sparsity.n_i.nz_by_row[i] {
                    code.push_str(&format!(
                        "    rhs[{}] += N_I[{}][{}] * state.i_nl_prev[{}];\n",
                        i, i, j, j
                    ));
                }
            }
        }
        // IIR op-amp RHS injection (PURE EXPLICIT — Gm is NOT in G):
        //   y_new = a1*y_prev + 2*b0*x_prev    (1-sample delay)
        //   rhs[o] += Go*(y_new + y_prev)      (full VCCS current, no matrix coupling)
        // At DC: y_ss = (2*b0/(1-a1))*x_ss = (Gm/Go)*x_ss = AOL*x_ss ✓
        for (idx, oa_iir) in ir.opamp_iir.iter().enumerate() {
            code.push_str(&format!(
                "    let oa{idx}_y_new = state.oa{idx}_a1 * state.oa{idx}_y_prev + 2.0 * state.oa{idx}_b0 * state.oa{idx}_x_prev;\n"
            ));
            code.push_str(&format!(
                "    rhs[{out}] += OA{idx}_GO * (oa{idx}_y_new + state.oa{idx}_y_prev);\n",
                out = oa_iir.out_idx
            ));
        }
        code.push('\n');

        // Input source (Thevenin)
        code.push_str("    let input_conductance = 1.0 / INPUT_RESISTANCE;\n");
        if ir.solver_config.backward_euler {
            code.push_str("    // Input source (backward Euler: V_in * G_in)\n");
            code.push_str("    rhs[INPUT_NODE] += input * input_conductance;\n");
        } else {
            code.push_str("    // Input source (trapezoidal: (V_in + V_in_prev) * G_in)\n");
            code.push_str(
                "    rhs[INPUT_NODE] += (input + state.input_prev) * input_conductance;\n",
            );
        }
        code.push_str("    state.input_prev = input;\n\n");

        // Handle linear circuits (M=0): direct LU solve, no NR iteration
        if m == 0 {
            code.push_str("    // Linear circuit: direct LU solve (no NR needed)\n");
            code.push_str("    let mut g_aug = state.a;\n");
            code.push_str(
                "    // Gmin regularization: improves conditioning for high-gain VCCS (op-amps)\n",
            );
            code.push_str("    for i in 0..N_NODES { g_aug[i][i] += 1e-12; }\n");
            code.push_str("    let mut v = rhs;\n");
            code.push_str("    if !lu_solve(&mut g_aug, &mut v) {\n");
            code.push_str("        v = state.v_prev;\n");
            code.push_str("    }\n\n");

            // No VSAT clamping — matches runtime NodalSolver. Clamping any node
            // creates inconsistency with unclamped neighbors (e.g., 100Ω apart but
            // 227V difference), corrupting the trapezoidal history feedback.
        } else {
            // Step 2: Newton-Raphson in full augmented voltage space
            code.push_str("    // Step 2: Newton-Raphson in full augmented voltage space\n");
            code.push_str("    let mut v = state.v_prev;\n");
            code.push_str("    let mut converged = false;\n");
            code.push_str("    let mut i_nl = [0.0f64; M];\n");
            // ActiveSetBe rail engagement flag — set after substep, checked
            // before BE fallback. Only used in ActiveSetBe mode (NOT plain
            // ActiveSet, which keeps its trap+pin behavior). See
            // OpampRailMode::ActiveSetBe handling below.
            let active_set_be_mode_full_lu = matches!(
                ir.solver_config.opamp_rail_mode,
                crate::codegen::OpampRailMode::ActiveSetBe
            );
            if active_set_be_mode_full_lu {
                code.push_str("    let mut active_set_engaged = false;\n");
            }
            code.push('\n');

            // Cross-timestep chord: reuse LU factors from previous sample when valid.
            // Local aliases avoid repeated state.chord_* indirection in the hot loop.
            // After convergence, chord_lu/d/perm/j_dev are persisted back to state.
            code.push_str(
                "    // Cross-timestep chord: start from previous sample's converged LU\n",
            );
            code.push_str("    let mut chord_lu = state.chord_lu;\n");
            code.push_str("    let mut chord_dr = state.chord_dr;\n");
            code.push_str("    let mut chord_dc = state.chord_dc;\n");
            code.push_str("    let mut chord_perm = state.chord_perm;\n");
            code.push_str("    let mut chord_j_dev = state.chord_j_dev;\n");
            code.push_str("    let mut chord_valid = state.chord_valid;\n\n");

            // Trapezoidal NR loop
            code.push_str("    for iter in 0..MAX_ITER {\n");

            // 2a. Extract nonlinear voltages: v_nl = N_v * v (sparse)
            code.push_str("        // 2a. Extract nonlinear voltages: v_nl = N_v * v (sparse)\n");
            code.push_str("        let mut v_nl = [0.0f64; M];\n");
            for i in 0..m {
                let nz_cols = &ir.sparsity.n_v.nz_by_row[i];
                if nz_cols.is_empty() {
                    continue;
                }
                let terms: Vec<String> = nz_cols
                    .iter()
                    .map(|&j| format!("N_V[{}][{}] * v[{}]", i, j, j))
                    .collect();
                code.push_str(&format!("        v_nl[{}] = {};\n", i, terms.join(" + ")));
            }
            code.push('\n');

            // 2b. Evaluate device currents and Jacobian
            code.push_str(
                "        // 2b. Evaluate device currents and Jacobian (block-diagonal)\n",
            );
            // i_nl is declared in outer scope (line above NR loop), j_dev is per-iteration
            code.push_str("        let mut j_dev = [0.0f64; M * M];\n");
            Self::emit_nodal_device_evaluation_body(&mut code, ir, "        ");
            code.push('\n');

            // 2c. Build and factor Jacobian: G_aug = A - N_i * J_dev * N_v (chord method)
            // Factor LU periodically: iter 0, then every CHORD_REFACTOR iterations.
            // Between refactors, reuse stored LU for O(N²) back-solve instead of O(N³) factor.
            // J_dev is block-diagonal; N_i and N_v have ~2 nonzero entries per device dim.
            // Compile-time unrolled: ~64 stamps for 4 tubes vs 107K dense iterations.
            code.push_str(
                "        // 2c. Build and factor Jacobian (adaptive chord: reuse across timesteps)\n",
            );
            // Refactor when: (a) no valid LU, (b) within-sample periodic
            // refresh, OR (BoyleDiodes only) (c) any device's diagonal
            // Jacobian has diverged from `chord_j_dev` by >50 % relative.
            //
            // The cross-timestep chord persistence holds `chord_j_dev`
            // frozen across many samples. That's fine for smoothly-
            // conducting devices (Schottky, BJT, JFET, MOSFET, tube), but
            // it breaks for the Boyle catch-diode model: at the DC
            // operating point a catch diode is deeply reverse-biased and
            // `j_dev ≈ 1e-31`, but the moment an op-amp output reaches
            // its rail the same diode wants `j_dev ≈ 1e1` — 32 orders of
            // magnitude away. Even with the residual check (added
            // separately below), NR can't find the true convergence
            // within MAX_ITER=50 iterations because every refactor only
            // happens at iter % 5 == 0, by which point pnjlim has
            // damped the steps so far that the chord can't catch up.
            //
            // Gating the adaptive trigger on BoyleDiodes mode keeps
            // VCR ALC's compressor attack timing tests byte-for-byte
            // unchanged: the rectifier diodes in the sidechain don't
            // exhibit the deep-reverse → deep-forward transition that
            // catch diodes do, so the trigger never fires for them
            // anyway when BoyleDiodes is off (and we don't even emit
            // the check for non-BoyleDiodes circuits).
            //
            // The 50 % threshold is empirical: lower (e.g. 20 %) causes
            // spurious refactors that trap pnjlim's step damping in a
            // different slow-oscillation regime; higher (e.g. 80 %)
            // misses the diode-knee transition.
            let emit_adaptive_refactor = matches!(
                ir.solver_config.opamp_rail_mode,
                crate::codegen::OpampRailMode::BoyleDiodes
            );
            if emit_adaptive_refactor {
                code.push_str("        let mut need_refactor = !chord_valid || (iter > 0 && iter % CHORD_REFACTOR == 0) || iter >= 10;\n");
                code.push_str("        if !need_refactor {\n");
                code.push_str("            for k in 0..M {\n");
                code.push_str("                let jk = j_dev[k * M + k];\n");
                code.push_str("                let ck = chord_j_dev[k * M + k];\n");
                code.push_str("                let mx = jk.abs().max(ck.abs());\n");
                code.push_str("                if mx > 1e-20 && (jk - ck).abs() / mx > 0.5 {\n");
                code.push_str("                    need_refactor = true;\n");
                code.push_str("                    break;\n");
                code.push_str("                }\n");
                code.push_str("            }\n");
                code.push_str("        }\n");
            } else {
                code.push_str("        let need_refactor = !chord_valid || (iter > 0 && iter % CHORD_REFACTOR == 0) || iter >= 10;\n");
            }
            code.push_str("        if need_refactor {\n");
            code.push_str("            chord_j_dev = j_dev;\n");
            code.push_str("            chord_lu = state.a;\n");
            code.push_str("            // Gmin regularization\n");
            code.push_str("            for i in 0..N_NODES { chord_lu[i][i] += 1e-12; }\n");
            {
                // Build transpose of N_i sparsity: for each device dim i, which nodes a are nonzero
                let mut ni_nz_by_dev = vec![Vec::new(); m];
                for (a, cols) in ir.sparsity.n_i.nz_by_row.iter().enumerate() {
                    for &i in cols {
                        ni_nz_by_dev[i].push(a);
                    }
                }

                // For each device, stamp N_i[:,i] * j_dev[i,j] * N_v[j,:] for all nonzero entries
                for slot in &ir.device_slots {
                    let s = slot.start_idx;
                    let dim = slot.dimension;
                    for di in 0..dim {
                        let i = s + di;
                        let ni_nodes = &ni_nz_by_dev[i];
                        for dj in 0..dim {
                            let j = s + dj;
                            let nv_nodes = &ir.sparsity.n_v.nz_by_row[j];
                            if ni_nodes.is_empty() || nv_nodes.is_empty() {
                                continue;
                            }
                            let jd_ij = i * m + j;
                            for &a in ni_nodes {
                                for &b in nv_nodes {
                                    code.push_str(&format!(
                                        "            chord_lu[{}][{}] -= N_I[{}][{}] * j_dev[{}] * N_V[{}][{}];\n",
                                        a, b, a, i, jd_ij, j, b
                                    ));
                                }
                            }
                        }
                    }
                }
            }
            // Factor: try sparse LU (if available), fall back to dense
            if ir.sparsity.lu.is_some() {
                code.push_str("            if !sparse_lu_factor(&mut chord_lu, &mut chord_dr, &mut chord_dc) {\n");
                code.push_str("                // Sparse pivot too small — fall back to dense LU with partial pivoting\n");
                code.push_str("                state.diag_nr_max_iter_count += 1;\n");
                code.push_str("                break;\n");
                code.push_str("            }\n");
            } else {
                code.push_str(
                    "            if !lu_factor(&mut chord_lu, &mut chord_dr, &mut chord_dc, &mut chord_perm) {\n",
                );
                code.push_str("                state.diag_nr_max_iter_count += 1;\n");
                code.push_str("                break;\n");
                code.push_str("            }\n");
            }
            code.push_str("            chord_valid = true;\n");
            code.push_str("            state.diag_refactor_count += 1;\n");
            code.push_str("        }\n\n");

            // 2d. Build companion RHS: rhs_base + N_i * (i_nl - J_dev_0 * v_nl) (sparse)
            // Uses chord_j_dev (from iter 0) to match the LU factorization.
            // Using current j_dev here would create an inconsistent fixed point.
            code.push_str(
                "        // 2d. Build companion RHS: rhs + N_i * (i_nl - chord_J_dev * v_nl) (sparse)\n",
            );
            code.push_str("        let mut rhs_work = rhs;\n");
            {
                // Build transpose of N_i sparsity
                let mut ni_nz_by_dev = vec![Vec::new(); m];
                for (a, cols) in ir.sparsity.n_i.nz_by_row.iter().enumerate() {
                    for &i in cols {
                        ni_nz_by_dev[i].push(a);
                    }
                }

                for slot in &ir.device_slots {
                    let s = slot.start_idx;
                    let dim = slot.dimension;
                    for di in 0..dim {
                        let i = s + di;
                        // Compute chord_jdev_vnl = sum_j chord_j_dev[i*M+j] * v_nl[j] (only within block)
                        let jdv_terms: Vec<String> = (0..dim)
                            .map(|dj| {
                                let j = s + dj;
                                let jd_ij = i * m + j;
                                format!("chord_j_dev[{}] * v_nl[{}]", jd_ij, j)
                            })
                            .collect();
                        code.push_str(&format!(
                            "        {{ let i_comp = i_nl[{}] - ({});\n",
                            i,
                            jdv_terms.join(" + ")
                        ));
                        // Stamp N_i * i_comp at nonzero nodes (N_I is N×M)
                        for &a in &ni_nz_by_dev[i] {
                            code.push_str(&format!(
                                "          rhs_work[{}] += N_I[{}][{}] * i_comp;\n",
                                a, a, i
                            ));
                        }
                        code.push_str("        }\n");
                    }
                }
            }
            code.push('\n');

            // 2e. Solve with stored LU factors (O(N²) back-solve)
            if ir.sparsity.lu.is_some() {
                code.push_str("        // 2e. Sparse back-solve with stored LU factors\n");
                code.push_str("        let mut v_new = rhs_work;\n");
                code.push_str("        sparse_lu_back_solve(&chord_lu, &chord_dr, &chord_dc, &mut v_new);\n\n");
            } else {
                code.push_str(
                    "        // 2e. Solve with stored LU factors (chord: O(N²) back-solve)\n",
                );
                code.push_str("        let mut v_new = rhs_work;\n");
                code.push_str(
                    "        lu_back_solve(&chord_lu, &chord_dr, &chord_dc, &chord_perm, &mut v_new);\n\n",
                );
            }

            // Op-amp output clamping inside NR loop — prevents physically impossible
            // voltages that destabilize downstream device evaluation. Applied after the
            // back-solve, before device voltage limiting.
            {
                let clampable: Vec<&crate::codegen::ir::OpampIR> = ir
                    .opamps
                    .iter()
                    .filter(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
                    .collect();
                if !clampable.is_empty() {
                    code.push_str(
                        "        // Per-iteration op-amp output rail clamp\n",
                    );
                    for oa in &clampable {
                        let o = oa.n_out_idx;
                        if oa.vclamp_hi.is_finite() {
                            code.push_str(&format!(
                                "        if v_new[{o}] > {hi:.17e} {{ v_new[{o}] = {hi:.17e}; }}\n",
                                o = o,
                                hi = oa.vclamp_hi,
                            ));
                        }
                        if oa.vclamp_lo.is_finite() {
                            code.push_str(&format!(
                                "        if v_new[{o}] < {lo:.17e} {{ v_new[{o}] = {lo:.17e}; }}\n",
                                o = o,
                                lo = oa.vclamp_lo,
                            ));
                        }
                    }
                    code.push('\n');
                }
            }

            // 2f. SPICE-style voltage limiting + global node damping
            code.push_str("        // 2f. SPICE voltage limiting + node damping\n");
            code.push_str("        let mut alpha = 1.0_f64;\n\n");

            // Layer 1: Device voltage limiting
            code.push_str("        // Layer 1: SPICE device voltage limiting\n");
            Self::emit_nodal_voltage_limiting(&mut code, ir);
            code.push('\n');

            // Layer 2: Global node voltage damping (adaptive threshold)
            code.push_str("        // Layer 2: Global node voltage damping\n");
            code.push_str("        {\n");
            code.push_str("            let mut max_node_dv = 0.0_f64;\n");
            code.push_str(&format!(
                "            for i in 0..{} {{\n\
                 \x20               let dv = alpha * (v_new[i] - v[i]);\n\
                 \x20               max_node_dv = max_node_dv.max(dv.abs());\n\
                 \x20           }}\n",
                n_nodes
            ));
            // Adaptive threshold: max(10V, 5% of max node voltage)
            code.push_str("            let mut max_v = 0.0_f64;\n");
            code.push_str(&format!(
                "            for i in 0..{} {{ max_v = max_v.max(v[i].abs()); }}\n",
                n_nodes
            ));
            code.push_str("            let damp_thresh = 10.0_f64.max(max_v * 0.05);\n");
            code.push_str("            if max_node_dv > damp_thresh {\n");
            code.push_str("                alpha *= (damp_thresh / max_node_dv).max(0.01);\n");
            code.push_str("            }\n");
            code.push_str("        }\n\n");

            // Apply damped Newton step and check convergence
            // Compute step BEFORE updating v, so convergence check sees the actual delta
            // Skip convergence check on iter 0 — need at least one full NR update
            // Convergence check on nonlinear device nodes only (N_V nonzero columns).
            // Linearized stages respond linearly and converge passively — checking
            // all N nodes causes spurious chord refactors when linear coupling
            // ripple from nonlinear stages hasn't settled to sub-µV precision.
            code.push_str("        // Compute damped step, check convergence, then apply\n");
            code.push_str("        let mut max_step_exceeded = false;\n");
            {
                let mut device_nodes: Vec<usize> = ir.sparsity.n_v.nz_by_row
                    .iter()
                    .flat_map(|row| row.iter().copied())
                    .collect();
                device_nodes.sort();
                device_nodes.dedup();
                for &node in &device_nodes {
                    code.push_str(&format!(
                        "        {{ let step = alpha * (v_new[{node}] - v[{node}]); let threshold = 1e-3 * v[{node}].abs().max((v[{node}] + step).abs()) + 1e-6; if step.abs() >= threshold {{ max_step_exceeded = true; }} }}\n"
                    ));
                }
            }
            code.push_str("        for i in 0..N { v[i] += alpha * (v_new[i] - v[i]); }\n");

            // Mid-NR op-amp output clamping (VCC/VEE).
            //
            // In `Hard` and `None` modes we either apply the clamp every
            // iteration (Hard) or do nothing (None). In `ActiveSet` mode the
            // in-NR clamp is SKIPPED — NR converges to whatever the linear
            // system dictates, then a post-convergence constrained resolve
            // pins the rail-violating nodes and re-solves the network to a
            // KCL-consistent state (see emit_nodal_active_set_resolve).
            //
            // The reason we skip the mid-NR clamp for ActiveSet: the hard
            // clamp writes `v_clamped` back into `v` but the rest of the
            // solve is consistent with `v_unclamped`. That inconsistency
            // corrupts the trapezoidal cap history on the next sample. The
            // active-set resolve rebuilds the full v so KCL is satisfied at
            // every node given the clamped value — no corruption.
            //
            // BoyleDiodes mode will eventually make this block unreachable
            // (rail saturation will be modeled by nonlinear catch diodes in
            // J_dev), but until that lands we still fall through to Hard
            // semantics if requested.
            //
            // Step 4 fix: when we do apply the clamp, re-check convergence
            // against the POST-clamp v so a silent mutation of v[out] can't
            // be reported as "converged". The previous code computed the
            // step magnitude against the unclamped v[out] + step — if
            // clamping then shifted v[out] by a significant amount, NR would
            // still report converged because `max_step_exceeded` was computed
            // on the pre-clamp step. This check catches that case.
            use crate::codegen::OpampRailMode;
            // BoyleDiodes mode has already inserted physical catch diodes into
            // the MNA; the NR solve naturally drives v[out] toward the rails
            // via those diodes. Emitting a post-NR hard clamp on top would
            // double-limit and re-introduce the KCL corruption we're trying
            // to avoid. So BoyleDiodes skips the in-NR clamp entirely, same
            // as ActiveSet.
            let emit_in_nr_clamp = matches!(
                ir.solver_config.opamp_rail_mode,
                OpampRailMode::Hard
            );
            if emit_in_nr_clamp && !ir.opamps.is_empty() {
                for oa in &ir.opamps {
                    if !oa.vclamp_lo.is_finite() && !oa.vclamp_hi.is_finite() {
                        continue;
                    }
                    code.push_str(&format!(
                        "        {{\n\
                         \x20           let v_pre_clamp = v[{idx}];\n\
                         \x20           v[{idx}] = v[{idx}].clamp({lo:.17e}, {hi:.17e});\n\
                         \x20           // Post-clamp convergence re-check: if the clamp mutated\n\
                         \x20           // v[{idx}] meaningfully, the iteration hasn't really\n\
                         \x20           // converged — the rest of the network is still consistent\n\
                         \x20           // with the pre-clamp value.\n\
                         \x20           let clamp_delta = (v[{idx}] - v_pre_clamp).abs();\n\
                         \x20           let clamp_thresh = 1e-3 * v[{idx}].abs().max(v_pre_clamp.abs()) + 1e-6;\n\
                         \x20           if clamp_delta >= clamp_thresh {{ max_step_exceeded = true; }}\n\
                         \x20       }}\n",
                        idx = oa.n_out_idx,
                        lo = oa.vclamp_lo,
                        hi = oa.vclamp_hi,
                    ));
                }
            }

            // Residual-based convergence safety net (BoyleDiodes only).
            //
            // The voltage-step check above (`max_step_exceeded`) declares
            // convergence whenever the damped Newton step is small. That
            // criterion is necessary but NOT sufficient when the chord
            // persistence holds a stale Jacobian: the LU back-solve uses
            // `chord_j_dev` while the actual device current at the new
            // operating point is `i_dev(v_nl_new)`. If `chord_j_dev` is
            // grossly out of date — which happens specifically for Boyle
            // catch diodes whose `j_dev` spans 32 OOM between deeply-
            // reverse-biased (≈1e-31 S at v_nl=-3 V) and forward-biased
            // (≈1e+1 S at v_nl=+0.8 V) — the LU's "fixed point" is a
            // non-physical state where KCL is satisfied for the LINEARISED
            // network but not for the actual nonlinear devices. NR happily
            // reports converged on a wildly wrong v.
            //
            // The DK Schur NR loop has had a residual check since
            // melange-solver inception (see `emit_nr_limit_and_converge`,
            // ~line 2920). This block ports the same idea to the full-LU
            // path: after the step is applied, re-evaluate `i_nl_fresh`
            // from the device equations at the updated v and require
            // it to match the `i_nl` that the LU was solved against.
            // Mismatch ⇒ NR keeps iterating, eventually triggering the
            // periodic chord refactor (iter % 5 == 0 or iter ≥ 10), the
            // sub-step retry, or the BE fallback — any of which has a
            // chance to break out of the stale-chord trap.
            //
            // Gated to BoyleDiodes because (a) no other validated circuit
            // exhibits the false-convergence pattern, and (b) the
            // re-evaluation costs M device-equation calls per NR
            // iteration. Schottky/BJT/JFET/MOSFET/tube circuits all have
            // their `j_dev` evolve smoothly enough that the chord stays
            // close enough to satisfy the voltage-step criterion as a
            // sufficient convergence check on its own.
            //
            // Implementation note: the device-evaluation helper writes
            // into local `v_nl`, `i_nl`, `j_dev` arrays of fixed names.
            // We use a nested `{ }` block so Rust shadows those bindings
            // — the throwaway `j_dev` inside the block doesn't disturb
            // the chord-stamped `j_dev` in the outer scope.
            // (`OpampRailMode` is already imported above.)
            if matches!(
                ir.solver_config.opamp_rail_mode,
                OpampRailMode::BoyleDiodes
            ) {
                code.push_str("        // BoyleDiodes residual check: re-evaluate i_nl at the\n");
                code.push_str("        // post-step v and force NR to keep iterating if the\n");
                code.push_str("        // chord linearisation produced an inconsistent fixed point.\n");
                code.push_str("        if !max_step_exceeded {\n");
                code.push_str("            let i_nl_chord = i_nl;\n");
                code.push_str("            let mut v_nl = [0.0f64; M];\n");
                code.push_str(&emit_sparse_nv_matvec(ir, "v_nl", "v", "            "));
                code.push_str("            let mut i_nl = [0.0f64; M];\n");
                code.push_str("            let mut j_dev = [0.0f64; M * M];\n");
                Self::emit_nodal_device_evaluation_body(&mut code, ir, "            ");
                code.push_str("            // Tolerance matches DK Schur path: ABSTOL=1e-12, RELTOL=1e-3,\n");
                code.push_str("            // with a 1e-9 floor on the magnitude denominator so devices\n");
                code.push_str("            // with i_nl ≈ 0 don't have an unreachably tight tolerance.\n");
                code.push_str("            for i in 0..M {\n");
                code.push_str("                let r = (i_nl[i] - i_nl_chord[i]).abs();\n");
                code.push_str("                let tol = 1e-3 * i_nl[i].abs().max(i_nl_chord[i].abs()).max(1e-9) + 1e-12;\n");
                code.push_str("                if r > tol {\n");
                code.push_str("                    max_step_exceeded = true;\n");
                code.push_str("                    break;\n");
                code.push_str("                }\n");
                code.push_str("            }\n");
                code.push_str("        }\n\n");
            }

            code.push_str("        let converged_check = !max_step_exceeded;\n\n");

            code.push_str("        if converged_check {\n");
            code.push_str("            converged = true;\n");
            code.push_str("            state.last_nr_iterations = iter as u32;\n");

            // Final device evaluation at converged point
            code.push_str("            // Final device evaluation at converged point\n");
            code.push_str("            let mut v_nl_final = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(ir, "v_nl_final", "v", "            "));
            Self::emit_nodal_device_evaluation_final(&mut code, ir, "            ");

            // ActiveSet (plain) — pin and re-solve in trap matrices, preserving
            // the steady DC rail value the op-amp converged to. Used by
            // control-path topologies (VCR ALC sidechain etc.) where the
            // rail-clamped value drives a nonlinear device's operating point.
            // ActiveSetBe takes a different path: detect-only here, BE
            // fallback for the actual resolve.
            if matches!(
                ir.solver_config.opamp_rail_mode,
                crate::codegen::OpampRailMode::ActiveSet
            ) {
                Self::emit_nodal_active_set_resolve(
                    &mut code,
                    ir,
                    "            ",
                    "state.a",
                    "rhs",
                );
            }

            code.push_str("            break;\n");
            code.push_str("        }\n");
            code.push_str("    }\n\n"); // end trapezoidal NR loop

            // Adaptive sub-stepping: when trapezoidal NR fails, subdivide the timestep
            // and retry with tighter capacitor conductances. This is how ngspice handles
            // positive-feedback circuits (compressor sidechains, oscillators, etc.).
            code.push_str("    // Adaptive sub-stepping: retry with subdivided timestep\n");
            code.push_str("    if !converged {\n");
            code.push_str("        'substep: for subdiv_power in 1..=3u32 {\n");
            code.push_str("            let subdiv = 1u32 << subdiv_power; // 2, 4, 8\n");
            code.push_str(&format!(
                "            let alpha_sub = {:.17e} * subdiv as f64;\n",
                2.0 * ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64
            ));
            code.push_str("            // Rebuild A and A_neg at finer timestep\n");
            code.push_str("            let mut a_sub = [[0.0f64; N]; N];\n");
            code.push_str("            let mut a_neg_sub = [[0.0f64; N]; N];\n");
            code.push_str("            for i in 0..N {\n");
            code.push_str("                for j in 0..N {\n");
            code.push_str("                    a_sub[i][j] = G[i][j] + alpha_sub * C[i][j];\n");
            code.push_str("                    a_neg_sub[i][j] = alpha_sub * C[i][j] - G[i][j];\n");
            code.push_str("                }\n");
            code.push_str("            }\n");
            // Zero VS/VCVS algebraic rows
            let n_aug = ir.topology.n_aug;
            if n_nodes < n_aug {
                code.push_str(&format!(
                    "            for i in {}..{} {{ for j in 0..N {{ a_neg_sub[i][j] = 0.0; }} }}\n",
                    n_nodes, n_aug
                ));
            }
            // Gmin on A_sub
            code.push_str("            for i in 0..N_NODES { a_sub[i][i] += 1e-6; }\n");
            code.push_str("            // Run subdivided sub-steps\n");
            code.push_str("            let mut v_sub = state.v_prev;\n");
            code.push_str("            let mut i_nl_sub = state.i_nl_prev;\n");
            code.push_str(
                "            let input_step = (input - state.input_prev) / subdiv as f64;\n",
            );
            code.push_str("            let mut all_sub_converged = true;\n");
            code.push_str("            for step in 0..subdiv {\n");
            code.push_str(
                "                let inp_s = state.input_prev + input_step * (step + 1) as f64;\n",
            );
            code.push_str(
                "                let inp_prev_s = state.input_prev + input_step * step as f64;\n",
            );
            // Build sub-step RHS
            code.push_str("                // Sub-step RHS\n");
            if ir.has_dc_sources {
                code.push_str("                let mut rhs_s = RHS_CONST;\n");
            } else {
                code.push_str("                let mut rhs_s = [0.0f64; N];\n");
            }
            code.push_str("                for i in 0..N { for j in 0..N { rhs_s[i] += a_neg_sub[i][j] * v_sub[j]; } }\n");
            if m > 0 {
                code.push_str(&emit_sparse_ni_matvec_add(ir, "rhs_s", "i_nl_sub", "                "));
            }
            code.push_str("                rhs_s[INPUT_NODE] += (inp_s + inp_prev_s) * (1.0 / INPUT_RESISTANCE);\n");
            // Sub-step NR loop
            code.push_str("                let mut sub_converged = false;\n");
            code.push_str("                for _iter in 0..MAX_ITER {\n");
            code.push_str("                    let mut v_nl = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(ir, "v_nl", "v_sub", "                    "));
            code.push_str("                    let mut i_nl = [0.0f64; M];\n");
            code.push_str("                    let mut j_dev = [0.0f64; M * M];\n");
            // Device evaluation
            Self::emit_nodal_device_evaluation_body(&mut code, ir, "                    ");
            code.push('\n');
            // Build G_aug from a_sub
            code.push_str("                    let mut g_s = a_sub;\n");
            {
                let mut ni_nz_by_dev = vec![Vec::new(); m];
                for (a, cols) in ir.sparsity.n_i.nz_by_row.iter().enumerate() {
                    for &i in cols {
                        ni_nz_by_dev[i].push(a);
                    }
                }
                for slot in &ir.device_slots {
                    let s = slot.start_idx;
                    let dim = slot.dimension;
                    for di in 0..dim {
                        let i = s + di;
                        let ni_nodes = &ni_nz_by_dev[i];
                        for dj in 0..dim {
                            let j = s + dj;
                            let jd_ij = i * m + j;
                            let nv_nodes = &ir.sparsity.n_v.nz_by_row[j];
                            if ni_nodes.is_empty() || nv_nodes.is_empty() {
                                continue;
                            }
                            for &a in ni_nodes {
                                for &b in nv_nodes {
                                    code.push_str(&format!(
                                        "                    g_s[{}][{}] -= N_I[{}][{}] * j_dev[{}] * N_V[{}][{}];\n",
                                        a, b, a, i, jd_ij, j, b
                                    ));
                                }
                            }
                        }
                    }
                }
            }
            // Build companion RHS
            code.push_str("                    let mut rhs_w = rhs_s;\n");
            {
                let mut ni_nz_by_dev = vec![Vec::new(); m];
                for (a, cols) in ir.sparsity.n_i.nz_by_row.iter().enumerate() {
                    for &i in cols {
                        ni_nz_by_dev[i].push(a);
                    }
                }
                for slot in &ir.device_slots {
                    let s = slot.start_idx;
                    let dim = slot.dimension;
                    for di in 0..dim {
                        let i = s + di;
                        let jdv_terms: Vec<String> = (0..dim)
                            .map(|dj| {
                                let j = s + dj;
                                let jd_ij = i * m + j;
                                format!("j_dev[{}] * v_nl[{}]", jd_ij, j)
                            })
                            .collect();
                        code.push_str(&format!(
                            "                    {{ let i_comp = i_nl[{}] - ({});\n",
                            i,
                            jdv_terms.join(" + ")
                        ));
                        for &a in &ni_nz_by_dev[i] {
                            code.push_str(&format!(
                                "                      rhs_w[{}] += N_I[{}][{}] * i_comp;\n",
                                a, a, i
                            ));
                        }
                        code.push_str("                    }\n");
                    }
                }
            }
            // LU solve
            code.push_str("                    let mut v_new_s = rhs_w;\n");
            code.push_str("                    if !lu_solve(&mut g_s, &mut v_new_s) { break; }\n");
            // Op-amp supply rail clamping (VCC/VEE) in sub-step. Applied for all
            // modes that have clampable op-amps — prevents physically impossible
            // voltages that destabilize downstream device evaluation.
            {
                let clampable: Vec<&crate::codegen::ir::OpampIR> = ir
                    .opamps
                    .iter()
                    .filter(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
                    .collect();
                if !clampable.is_empty() {
                    for oa in &clampable {
                        let o = oa.n_out_idx;
                        if oa.vclamp_hi.is_finite() {
                            code.push_str(&format!(
                                "                    if v_new_s[{o}] > {hi:.17e} {{ v_new_s[{o}] = {hi:.17e}; }}\n",
                                o = o,
                                hi = oa.vclamp_hi,
                            ));
                        }
                        if oa.vclamp_lo.is_finite() {
                            code.push_str(&format!(
                                "                    if v_new_s[{o}] < {lo:.17e} {{ v_new_s[{o}] = {lo:.17e}; }}\n",
                                o = o,
                                lo = oa.vclamp_lo,
                            ));
                        }
                    }
                }
            }
            // Convergence check + update
            code.push_str("                    let mut max_step = 0.0f64;\n");
            code.push_str("                    for i in 0..N_NODES {\n");
            code.push_str("                        let step = v_new_s[i] - v_sub[i];\n");
            code.push_str(
                "                        if step.abs() > max_step { max_step = step.abs(); }\n",
            );
            code.push_str("                    }\n");
            code.push_str("                    v_sub = v_new_s;\n");
            code.push_str("                    // Re-extract i_nl at converged v\n");
            code.push_str("                    let mut v_nl_f = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(ir, "v_nl_f", "v_sub", "                    "));
            code.push_str("                    i_nl_sub = [0.0f64; M];\n");
            Self::emit_nodal_device_evaluation_body(&mut code, ir, "                    ");
            code.push_str("                    if max_step < TOL + 1e-3 {\n");
            code.push_str("                        sub_converged = true;\n");
            code.push_str("                        break;\n");
            code.push_str("                    }\n");
            code.push_str("                }\n"); // end sub-step NR loop
            code.push_str(
                "                if !sub_converged { all_sub_converged = false; break; }\n",
            );
            code.push_str("            }\n"); // end sub-step loop
            code.push_str("            if all_sub_converged {\n");
            code.push_str("                v = v_sub;\n");
            code.push_str("                i_nl = i_nl_sub;\n");
            code.push_str("                converged = true;\n");
            code.push_str("                state.diag_substep_count += 1;\n");
            code.push_str("                break 'substep;\n");
            code.push_str("            }\n");
            code.push_str("        }\n"); // end subdiv_power loop
            code.push_str("    }\n\n"); // end if !converged

            // ActiveSetBe rail engagement check (post-trap, post-substep).
            // Runs on the final v from either the regular NR loop or the
            // substep recovery, whichever produced the converged result. If
            // any op-amp output is railed, set the flag so the BE fallback
            // fires — trap+pin develops a Nyquist-rate limit cycle on
            // audio-path topologies; BE+pin doesn't.
            //
            // Plain ActiveSet doesn't take this path — its trap+pin happens
            // inside the NR break block above.
            if active_set_be_mode_full_lu {
                code.push_str("    if converged {\n");
                Self::emit_nodal_active_set_check(
                    &mut code,
                    ir,
                    "        ",
                    "active_set_engaged",
                );
                code.push_str("    }\n\n");
            }

            // Backward Euler fallback. Triggered when trapezoidal NR (and
            // sub-stepping) failed, OR when ActiveSetBe detected a rail
            // engagement (BE doesn't ring under the row/col pin where
            // trapezoidal does).
            code.push_str(
                "    // Backward Euler fallback: if trapezoidal NR and sub-stepping both failed,\n",
            );
            code.push_str(
                "    // or if ActiveSetBe detected a rail engagement on the trap result.\n",
            );
            if active_set_be_mode_full_lu {
                code.push_str("    if !converged || active_set_engaged {\n");
            } else {
                code.push_str("    if !converged {\n");
            }
            code.push_str("        state.diag_nr_max_iter_count += 1;\n");
            code.push_str("        chord_valid = false;\n\n");

            // Rebuild RHS with BE matrices
            code.push_str("        // Rebuild RHS with backward Euler matrices\n");
            code.push_str("        v = state.v_prev;\n");
            code.push_str("        let mut rhs_be = [0.0f64; N];\n");
            code.push_str("        for i in 0..N {\n");
            if ir.has_dc_sources && !ir.matrices.rhs_const_be.is_empty() {
                code.push_str("            let mut sum = RHS_CONST_BE[i];\n");
            } else {
                code.push_str("            let mut sum = 0.0;\n");
            }
            code.push_str("            for j in 0..N {\n");
            code.push_str("                sum += state.a_neg_be[i][j] * state.v_prev[j];\n");
            code.push_str("            }\n");
            if m > 0 {
                code.push_str("            for j in 0..M {\n");
                code.push_str("                sum += N_I[i][j] * state.i_nl_prev[j];\n");
                code.push_str("            }\n");
            }
            code.push_str("            rhs_be[i] = sum;\n");
            code.push_str("        }\n");
            code.push_str("        // BE input: just input[n+1] * G_in (no trapezoidal average)\n");
            code.push_str("        rhs_be[INPUT_NODE] += input * input_conductance;\n\n");

            // BE NR loop
            code.push_str("        for _iter in 0..MAX_ITER {\n");

            // Extract v_nl (sparse N_V)
            code.push_str("            let mut v_nl = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(ir, "v_nl", "v", "            "));
            code.push_str("\n");

            // Evaluate devices (write to outer i_nl, declare local j_dev)
            code.push_str("            // Evaluate devices\n");
            code.push_str("            let mut j_dev = [0.0f64; M * M];\n");
            Self::emit_nodal_device_evaluation_body(&mut code, ir, "            ");
            code.push('\n');

            // Build Jacobian for BE (sparse, same structure as trapezoidal)
            code.push_str("            let mut g_aug = state.a_be;\n");
            code.push_str("            // Gmin regularization\n");
            code.push_str("            for i in 0..N_NODES { g_aug[i][i] += 1e-12; }\n");
            {
                let mut ni_nz_by_dev = vec![Vec::new(); m];
                for (a, cols) in ir.sparsity.n_i.nz_by_row.iter().enumerate() {
                    for &i in cols {
                        ni_nz_by_dev[i].push(a);
                    }
                }
                for slot in &ir.device_slots {
                    let s = slot.start_idx;
                    let dim = slot.dimension;
                    for di in 0..dim {
                        let i = s + di;
                        for dj in 0..dim {
                            let j = s + dj;
                            let jd_ij = i * m + j;
                            let nv_nodes = &ir.sparsity.n_v.nz_by_row[j];
                            for &a in &ni_nz_by_dev[i] {
                                for &b in nv_nodes {
                                    code.push_str(&format!(
                                        "            g_aug[{}][{}] -= N_I[{}][{}] * j_dev[{}] * N_V[{}][{}];\n",
                                        a, b, a, i, jd_ij, j, b
                                    ));
                                }
                            }
                        }
                    }
                }
            }
            code.push('\n');

            // Companion RHS for BE (sparse)
            code.push_str("            let mut rhs_work = rhs_be;\n");
            {
                let mut ni_nz_by_dev = vec![Vec::new(); m];
                for (a, cols) in ir.sparsity.n_i.nz_by_row.iter().enumerate() {
                    for &i in cols {
                        ni_nz_by_dev[i].push(a);
                    }
                }
                for slot in &ir.device_slots {
                    let s = slot.start_idx;
                    let dim = slot.dimension;
                    for di in 0..dim {
                        let i = s + di;
                        let jdv_terms: Vec<String> = (0..dim)
                            .map(|dj| {
                                let j = s + dj;
                                let jd_ij = i * m + j;
                                format!("j_dev[{}] * v_nl[{}]", jd_ij, j)
                            })
                            .collect();
                        code.push_str(&format!(
                            "            {{ let i_comp = i_nl[{}] - ({});\n",
                            i,
                            jdv_terms.join(" + ")
                        ));
                        for &a in &ni_nz_by_dev[i] {
                            code.push_str(&format!(
                                "              rhs_work[{}] += N_I[{}][{}] * i_comp;\n",
                                a, a, i
                            ));
                        }
                        code.push_str("            }\n");
                    }
                }
            }
            code.push('\n');

            // LU solve for BE
            code.push_str("            let mut v_new = rhs_work;\n");
            code.push_str("            if !lu_solve(&mut g_aug, &mut v_new) { break; }\n\n");

            // Per-iteration op-amp output rail clamp for BE path
            {
                let clampable: Vec<&crate::codegen::ir::OpampIR> = ir
                    .opamps
                    .iter()
                    .filter(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
                    .collect();
                if !clampable.is_empty() {
                    code.push_str(
                        "            // Per-iteration op-amp output rail clamp (BE)\n",
                    );
                    for oa in &clampable {
                        let o = oa.n_out_idx;
                        if oa.vclamp_hi.is_finite() {
                            code.push_str(&format!(
                                "            if v_new[{o}] > {hi:.17e} {{ v_new[{o}] = {hi:.17e}; }}\n",
                                o = o, hi = oa.vclamp_hi,
                            ));
                        }
                        if oa.vclamp_lo.is_finite() {
                            code.push_str(&format!(
                                "            if v_new[{o}] < {lo:.17e} {{ v_new[{o}] = {lo:.17e}; }}\n",
                                o = o, lo = oa.vclamp_lo,
                            ));
                        }
                    }
                    code.push('\n');
                }
            }

            // Limiting and damping for BE (same structure)
            code.push_str("            let mut alpha = 1.0_f64;\n");
            Self::emit_nodal_voltage_limiting_indented(&mut code, ir, "            ");
            code.push_str("            {\n");
            code.push_str("                let mut max_node_dv = 0.0_f64;\n");
            code.push_str(&format!(
                "                for i in 0..{} {{\n\
                 \x20                   let dv = alpha * (v_new[i] - v[i]);\n\
                 \x20                   max_node_dv = max_node_dv.max(dv.abs());\n\
                 \x20               }}\n",
                n_nodes
            ));
            code.push_str("                if max_node_dv > 10.0 { alpha *= (10.0 / max_node_dv).max(0.01); }\n");
            code.push_str("            }\n\n");

            // Apply damped step and check convergence (compute delta before updating).
            // Convergence check on nonlinear device nodes only (N_V nonzero columns),
            // matching the trapezoidal NR path. Checking all N nodes includes VCCS rows
            // for op-amps which may never satisfy the step criterion when op-amp outputs
            // are railed — causing BE to loop to MAX_ITER perpetually.
            code.push_str("            let mut be_step_exceeded = false;\n");
            {
                let mut device_nodes: Vec<usize> = ir.sparsity.n_v.nz_by_row
                    .iter()
                    .flat_map(|row| row.iter().copied())
                    .collect();
                device_nodes.sort();
                device_nodes.dedup();
                for &node in &device_nodes {
                    code.push_str(&format!(
                        "            {{ let step = alpha * (v_new[{node}] - v[{node}]); let threshold = 1e-3 * v[{node}].abs().max((v[{node}] + step).abs()) + 1e-6; if step.abs() >= threshold {{ be_step_exceeded = true; }} }}\n"
                    ));
                }
            }
            code.push_str("            for i in 0..N { v[i] += alpha * (v_new[i] - v[i]); }\n");

            // Mid-NR op-amp clamping for BE path. Only emit for Hard mode;
            // ActiveSet and BoyleDiodes handle rails via their respective
            // mechanisms (active-set resolve / physical catch diodes), and a
            // hard clamp here would conflict with them.
            if matches!(ir.solver_config.opamp_rail_mode, crate::codegen::OpampRailMode::Hard)
                && !ir.opamps.is_empty()
            {
                for oa in &ir.opamps {
                    if !oa.vclamp_lo.is_finite() && !oa.vclamp_hi.is_finite() {
                        continue;
                    }
                    code.push_str(&format!(
                        "            v[{idx}] = v[{idx}].clamp({lo:.17e}, {hi:.17e});\n",
                        idx = oa.n_out_idx, lo = oa.vclamp_lo, hi = oa.vclamp_hi,
                    ));
                }
            }

            // BoyleDiodes residual check (BE fallback path).
            //
            // BE rebuilds `j_dev` fresh each iteration (no chord
            // persistence), so the staleness mode that causes the
            // trapezoidal NR loop to false-converge doesn't apply here.
            // But the BE convergence criterion is still voltage-step
            // only (`be_step_exceeded`), which can declare a small
            // damped step "converged" while the actual i_nl mismatch
            // (between the i_nl that fed the LU and i_nl re-evaluated
            // at the post-step v) is large — exactly the same false
            // convergence on a non-physical state. We mirror the
            // residual check from the main NR loop here, gated on
            // BoyleDiodes for the same reason: every other validated
            // mode passes the voltage-step check as a sufficient
            // criterion.
            // BE does true Newton (fresh LU each iteration), so the residual
            // check is less critical here than in the chord-based trap path.
            // Only emit for BoyleDiodes where catch diodes have extreme knees.
            if matches!(
                ir.solver_config.opamp_rail_mode,
                crate::codegen::OpampRailMode::BoyleDiodes
            ) {
                code.push_str("            // BoyleDiodes residual check (BE path)\n");
                code.push_str("            if !be_step_exceeded {\n");
                code.push_str("                let i_nl_be_chord = i_nl;\n");
                code.push_str("                let mut v_nl = [0.0f64; M];\n");
                code.push_str(&emit_sparse_nv_matvec(ir, "v_nl", "v", "                "));
                code.push_str("                let mut i_nl = [0.0f64; M];\n");
                code.push_str("                let mut j_dev = [0.0f64; M * M];\n");
                Self::emit_nodal_device_evaluation_body(&mut code, ir, "                ");
                code.push_str("                for i in 0..M {\n");
                code.push_str("                    let r = (i_nl[i] - i_nl_be_chord[i]).abs();\n");
                code.push_str("                    let tol = 1e-3 * i_nl[i].abs().max(i_nl_be_chord[i].abs()).max(1e-9) + 1e-12;\n");
                code.push_str("                    if r > tol {\n");
                code.push_str("                        be_step_exceeded = true;\n");
                code.push_str("                        break;\n");
                code.push_str("                    }\n");
                code.push_str("                }\n");
                code.push_str("            }\n\n");
            }

            code.push_str("            let be_converged = !be_step_exceeded;\n\n");

            code.push_str("            if be_converged {\n");
            code.push_str("                converged = true;\n");
            code.push_str("                state.diag_be_fallback_count += 1;\n");
            code.push_str("                let mut v_nl_final = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(ir, "v_nl_final", "v", "                "));
            Self::emit_nodal_device_evaluation_final(&mut code, ir, "                ");
            code.push_str("                break;\n");
            code.push_str("            }\n");
            code.push_str("        }\n\n"); // end BE NR loop

            // If still not converged, ensure i_nl is consistent
            code.push_str("        // If still not converged, ensure i_nl is consistent with v\n");
            code.push_str("        if !converged {\n");
            code.push_str("            let mut v_nl_final = [0.0f64; M];\n");
            code.push_str(&emit_sparse_nv_matvec(ir, "v_nl_final", "v", "            "));
            Self::emit_nodal_device_evaluation_final(&mut code, ir, "            ");
            code.push_str("        }\n");

            // ActiveSetBe post-BE resolve: if any op-amp output is railed in
            // the BE result, pin and re-solve against `state.a_be`. BE+pin
            // doesn't develop the trap+pin Nyquist limit cycle, so the cap
            // history stays consistent across the next sample.
            if active_set_be_mode_full_lu {
                Self::emit_nodal_active_set_resolve(
                    &mut code,
                    ir,
                    "        ",
                    "state.a_be",
                    "rhs_be",
                );
            }

            code.push_str("    }\n\n"); // end BE fallback block
        }

        // NaN check BEFORE state update (prevents corruption of v_prev/i_nl_prev)
        code.push_str("    if !v.iter().all(|x| x.is_finite()) {\n");
        code.push_str("        state.v_prev = state.dc_operating_point;\n");
        if ir.dc_nl_currents.iter().any(|&v| v != 0.0) {
            code.push_str("        state.i_nl_prev = DC_NL_I;\n");
            code.push_str("        state.i_nl_prev_prev = DC_NL_I;\n");
        } else {
            code.push_str("        state.i_nl_prev = [0.0; M];\n");
            code.push_str("        state.i_nl_prev_prev = [0.0; M];\n");
        }
        code.push_str("        state.input_prev = 0.0;\n");
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.has_self_heating() {
                    code.push_str(&format!(
                        "        state.device_{dev_num}_tj = DEVICE_{dev_num}_TAMB;\n\
                         \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS;\n\
                         \x20       state.device_{dev_num}_vt = DEVICE_{dev_num}_VT;\n"
                    ));
                }
            }
        }
        // Reset IIR op-amp state on NaN
        for (idx, _oa_iir) in ir.opamp_iir.iter().enumerate() {
            code.push_str(&format!(
                "        state.oa{idx}_x_prev = 0.0;\n\
                 \x20       state.oa{idx}_y_prev = 0.0;\n"
            ));
        }
        code.push_str("        state.diag_nan_reset_count += 1;\n");
        if m > 0 {
            code.push_str("        state.chord_valid = false;\n");
        }
        // Return DC OP output (not zero) to minimize discontinuity on NaN recovery
        code.push_str("        let mut nan_out = [0.0f64; NUM_OUTPUTS];\n");
        for (oi, &node) in ir.solver_config.output_nodes.iter().enumerate() {
            if node < ir.dc_operating_point.len() {
                let dc_val = ir.dc_operating_point[node];
                let scale = ir.solver_config.output_scales.get(oi).copied().unwrap_or(1.0);
                let out_val = dc_val * scale;
                code.push_str(&format!(
                    "        nan_out[{oi}] = {:.17e};\n",
                    out_val
                ));
            }
        }
        code.push_str("        return nan_out;\n");
        code.push_str("    }\n\n");

        // No VSAT clamping on v — clamping any subset of nodes creates physical
        // inconsistency with unclamped neighbors (e.g., 100Ω resistor between
        // clamped node at 13V and unclamped node at 240V), corrupting the
        // trapezoidal history (a_neg * v_prev). Matches runtime NodalSolver.
        // Output is clamped downstream by DC block (±10V) or ear protection.

        // Op-amp slew-rate limiting (nodal full-LU path). Clamp
        // `|v[out] - v_prev[out]|` to `SR*dt` for each op-amp with finite
        // SR. Equivalent to clamping the Boyle dominant-pole integrator
        // input current at ±SR*C_dom. Zero code is emitted when all
        // op-amps have infinite SR.
        Self::emit_opamp_slew_limit(&mut code, ir, "    ", "v");

        // Death spiral protection: when ALL NR paths fail (trap + substep + BE),
        // do NOT store the bad partial iterate into v_prev/i_nl_prev. Each failed
        // sample would hand a progressively worse initial condition to the next,
        // creating a cascade. Instead, keep the previous (presumably converged)
        // state so the next sample starts from a reasonable point. The chord LU
        // is invalidated to force a fresh factorization.
        if m > 0 {
            code.push_str("    if !converged {\n");
            code.push_str("        // NR failed on all paths — keep previous state, invalidate chord\n");
            code.push_str("        v = state.v_prev;\n");
            code.push_str("        i_nl = state.i_nl_prev;\n");
            code.push_str("        chord_valid = false;\n");
            code.push_str("    }\n\n");
        }

        // Step 3: Update state
        code.push_str("    // Step 3: Update state\n");
        code.push_str("    state.v_prev = v;\n");
        if m > 0 {
            code.push_str("    state.i_nl_prev_prev = state.i_nl_prev;\n");
            code.push_str("    state.i_nl_prev = i_nl;\n");
            // Persist chord LU for cross-timestep reuse
            code.push_str("    state.chord_lu = chord_lu;\n");
            code.push_str("    state.chord_dr = chord_dr;\n");
            code.push_str("    state.chord_dc = chord_dc;\n");
            code.push_str("    state.chord_perm = chord_perm;\n");
            code.push_str("    state.chord_j_dev = chord_j_dev;\n");
            code.push_str("    state.chord_valid = chord_valid;\n");
        }
        // IIR op-amp state update (PURE EXPLICIT):
        //   x_prev ← x_new (from converged v, for next sample's explicit injection)
        //   y_prev ← y_new (pre-computed before solve)
        for (idx, oa_iir) in ir.opamp_iir.iter().enumerate() {
            let x_expr = match (oa_iir.np_idx, oa_iir.nm_idx) {
                (Some(np), Some(nm)) => format!("v[{np}] - v[{nm}]"),
                (Some(np), None) => format!("v[{np}]"),
                (None, Some(nm)) => format!("-v[{nm}]"),
                (None, None) => "0.0".to_string(),
            };
            code.push_str(&format!(
                "    {{\n\
                 \x20       let x_new = {x_expr};\n\
                 \x20       state.oa{idx}_x_prev = x_new;\n\
                 \x20       state.oa{idx}_y_prev = oa{idx}_y_new;\n\
                 \x20   }}\n"
            ));
        }
        for (idx, _pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "    state.pot_{}_resistance_prev = state.pot_{}_resistance;\n",
                idx, idx
            ));
        }
        code.push('\n');

        // Step 3b: BJT self-heating thermal update (quasi-static, outside NR)
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.has_self_heating() {
                    let s = slot.start_idx;
                    let s1 = s + 1;
                    code.push_str(&format!(
                            "    {{ // BJT {dev_num} self-heating thermal update\n\
                             \x20       let ic = i_nl[{s}];\n\
                             \x20       let ib = i_nl[{s1}];\n\
                             \x20       // Extract controlling voltages from final node voltages\n\
                             \x20       let mut vbe_sum = 0.0f64;\n\
                             \x20       let mut vbc_sum = 0.0f64;\n\
                             \x20       for j in 0..N {{ vbe_sum += N_V[{s}][j] * v[j]; }}\n\
                             \x20       for j in 0..N {{ vbc_sum += N_V[{s1}][j] * v[j]; }}\n\
                             \x20       let vce = vbe_sum - vbc_sum;\n\
                             \x20       let p = vce * ic + vbe_sum * ib;\n\
                             \x20       let dt = 1.0 / SAMPLE_RATE;\n\
                             \x20       let d_tj = (p - (state.device_{dev_num}_tj - DEVICE_{dev_num}_TAMB) / DEVICE_{dev_num}_RTH) / DEVICE_{dev_num}_CTH * dt;\n\
                             \x20       state.device_{dev_num}_tj += d_tj;\n\
                             \x20       state.device_{dev_num}_tj = state.device_{dev_num}_tj.clamp(200.0, 500.0);\n\
                             \x20       state.device_{dev_num}_vt = BOLTZMANN_Q * state.device_{dev_num}_tj;\n\
                             \x20       let t_ratio = state.device_{dev_num}_tj / DEVICE_{dev_num}_TAMB;\n\
                             \x20       let vt_nom = BOLTZMANN_Q * DEVICE_{dev_num}_TAMB;\n\
                             \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS_NOM\n\
                             \x20           * t_ratio.powf(DEVICE_{dev_num}_XTI)\n\
                             \x20           * fast_exp((DEVICE_{dev_num}_EG / vt_nom) * (1.0 - DEVICE_{dev_num}_TAMB / state.device_{dev_num}_tj));\n\
                             \x20   }}\n"
                        ));
                }
            }
        }

        // (NaN check already done before state update)

        if m > 0 {
            code.push_str("    if state.last_nr_iterations >= MAX_ITER as u32 {\n");
            code.push_str("        state.diag_nr_max_iter_count += 1;\n");
            code.push_str("    }\n\n");
        }

        // Step 4: Extract outputs, apply DC blocking and scaling
        code.push_str("    // Step 4: Extract outputs, DC blocking, and scaling\n");
        code.push_str("    let mut output = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let raw_out = v[OUTPUT_NODES[out_idx]];\n");
        code.push_str("        let raw_out = if raw_out.is_finite() { raw_out } else { 0.0 };\n");
        if ir.dc_block {
            code.push_str("        let dc_blocked = raw_out - state.dc_block_x_prev[out_idx]\n");
            code.push_str("            + state.dc_block_r * state.dc_block_y_prev[out_idx];\n");
            code.push_str("        state.dc_block_x_prev[out_idx] = raw_out;\n");
            code.push_str("        state.dc_block_y_prev[out_idx] = dc_blocked;\n");
            code.push_str("        let scaled = dc_blocked * OUTPUT_SCALES[out_idx];\n");
        } else {
            code.push_str("        let scaled = raw_out * OUTPUT_SCALES[out_idx];\n");
        }
        code.push_str("        let abs_out = scaled.abs();\n");
        code.push_str(
            "        if abs_out > state.diag_peak_output { state.diag_peak_output = abs_out; }\n",
        );
        if ir.dc_block {
            code.push_str("        if abs_out > 10.0 { state.diag_clamp_count += 1; }\n");
            code.push_str("        output[out_idx] = scaled.clamp(-10.0, 10.0);\n");
        } else {
            code.push_str(
                "        output[out_idx] = if scaled.is_finite() { scaled } else { 0.0 };\n",
            );
        }
        code.push_str("    }\n");
        code.push_str("    output\n");
        code.push_str("}\n\n");

        let _ = (num_outputs, n_nodes);

        code
    }

    /// Emit the active-set constrained-resolve block for a nodal NR path.
    ///
    /// Called after NR has converged in the Schur or full-LU nodal paths when
    /// `OpampRailMode::ActiveSet` is selected. At entry:
    ///   - `v` holds the *unclamped* converged solution (possibly with op-amp
    ///     outputs outside their supply rails).
    ///   - `i_nl` holds the final device currents at the converged v.
    ///
    /// **Key observation**: at NR convergence, the nonlinear companion equation
    /// collapses to the linear relation `A * v = rhs + N_i * i_nl`. The device
    /// Jacobian `J_dev` contributions that appear inside the NR iteration
    /// cancel identically at the fixed point — you can verify this by writing
    /// out `(A − N_i·J_dev·N_v) v = rhs + N_i·(i_nl − J_dev·v_nl)` with
    /// `v_nl = N_v·v` and expanding. That means the active-set resolve only
    /// needs `A` (from `state.a` or `state.a_be`) and the converged `i_nl`, not
    /// the per-NR Jacobian, so it's portable between the Schur, full-LU, and
    /// BE-fallback code paths.
    ///
    /// The resolve does:
    ///   1. Detect which op-amp outputs exceed their VCC/VEE rails.
    ///   2. If none do, nothing to do — leave `v` alone.
    ///   3. Otherwise build the linear system `A * v = rhs + N_i * i_nl` using
    ///      `matrix_name` (with a small Gmin on node diagonals for conditioning).
    ///   4. For each rail-violating node `k` with rail value `c_k`:
    ///        - subtract `A[:,k] * c_k` from `rhs` at every row except `k`
    ///        - zero column `k` in all non-`k` rows
    ///        - zero row `k` and set `A[k][k] = 1, rhs[k] = c_k`
    ///
    ///      This pins `v[k] = c_k` by row/column elimination.
    ///   5. Solve the constrained system with the generated dense `lu_solve`.
    ///      The result is a `v'` where every node satisfies KCL given the
    ///      clamped outputs, so the next sample's `A_neg * v_prev` sees a
    ///      KCL-consistent state and the cap-history corruption bug goes away.
    ///   6. Re-evaluate `i_nl` at the new `v` so the next sample's trapezoidal
    ///      history term (`N_i * i_nl_prev`) is also consistent with the
    ///      constrained voltages.
    ///
    /// If the dense LU solve fails (shouldn't happen for well-posed circuits),
    /// leave `v` at the unclamped solution; the downstream output clamp
    /// (`output[idx].clamp(-10, 10)`) still prevents catastrophic output, and
    /// the diagnostic counter `diag_nr_max_iter_count` records the failure.
    ///
    /// # Caller contract
    ///
    /// The caller must have `v` (mutable), `i_nl`, and a variable named
    /// `rhs_name` in scope. The `rhs` variable means the linear RHS *before*
    /// any nonlinear companion contribution — i.e.,
    /// `A_neg·v_prev + rhs_const + (input+input_prev)*G_in` (trapezoidal) or
    /// `A_neg_be·v_prev + rhs_const_be + input*G_in` (backward Euler).
    ///
    /// `matrix_name` is the matrix expression to use for the constrained
    /// resolve — `"state.a"` for trapezoidal, `"state.a_be"` for BE. Must be
    /// consistent with the integrator that produced `rhs`.
    pub(super) fn emit_nodal_active_set_resolve(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
        matrix_name: &str,
        rhs_name: &str,
    ) {
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            ir.topology.n
        };

        // Only clampable op-amps participate — ones with at least one finite rail.
        let clampable: Vec<&crate::codegen::ir::OpampIR> = ir
            .opamps
            .iter()
            .filter(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
            .collect();
        if clampable.is_empty() {
            return;
        }

        code.push_str(&format!(
            "{indent}// --- Active-set op-amp rail resolve ---\n"
        ));
        code.push_str(&format!(
            "{indent}// Pin any rail-violating op-amp outputs and re-solve for\n"
        ));
        code.push_str(&format!(
            "{indent}// a KCL-consistent v (see emit_nodal_active_set_resolve docs).\n"
        ));
        code.push_str(&format!("{indent}{{\n"));

        // Step 1: detect violations. `pinned_N` carries either `Some(rail)` or
        // `None` based on whether v[out] exceeds its range. `any_pinned` short-
        // circuits the whole resolve when nothing needs clamping (the common case).
        code.push_str(&format!("{indent}    let mut any_pinned = false;\n"));
        for (idx, oa) in clampable.iter().enumerate() {
            code.push_str(&format!(
                "{indent}    let pinned_{idx}: Option<f64> = if v[{node}] > {hi:.17e} {{\n\
                 {indent}        any_pinned = true;\n\
                 {indent}        Some({hi:.17e})\n\
                 {indent}    }} else if v[{node}] < {lo:.17e} {{\n\
                 {indent}        any_pinned = true;\n\
                 {indent}        Some({lo:.17e})\n\
                 {indent}    }} else {{\n\
                 {indent}        None\n\
                 {indent}    }};\n",
                idx = idx,
                node = oa.n_out_idx,
                hi = oa.vclamp_hi,
                lo = oa.vclamp_lo,
            ));
        }

        code.push_str(&format!("{indent}    if any_pinned {{\n"));

        // Step 2: build the linear system `A * v = rhs + N_i * i_nl` at the
        // converged operating point. Copy the matrix as scratch, add Gmin on
        // node diagonals for LU conditioning, copy `rhs` and add the nonlinear
        // current injection via the sparse N_i pattern.
        code.push_str(&format!(
            "{indent}        let mut g_as = {matrix_name};\n\
             {indent}        for i in 0..{n_nodes} {{ g_as[i][i] += 1e-12; }}\n",
            n_nodes = n_nodes,
        ));
        code.push_str(&format!("{indent}        let mut rhs_as = {rhs_name};\n"));
        if m > 0 {
            let mut ni_nz_by_dev = vec![Vec::new(); m];
            for (a, cols) in ir.sparsity.n_i.nz_by_row.iter().enumerate() {
                for &i in cols {
                    ni_nz_by_dev[i].push(a);
                }
            }
            for i in 0..m {
                for &a in &ni_nz_by_dev[i] {
                    code.push_str(&format!(
                        "{indent}        rhs_as[{a}] += N_I[{a}][{i}] * i_nl[{i}];\n",
                        a = a,
                        i = i,
                    ));
                }
            }
        }

        // Step 3: apply active-set constraints via row/column elimination. For
        // each pinned op-amp output k with clamp value c_k, move its
        // contribution out of the other rows and pin the row to an identity
        // equation `v[k] = c_k`.
        for (idx, oa) in clampable.iter().enumerate() {
            let node = oa.n_out_idx;
            code.push_str(&format!(
                "{indent}        if let Some(c_k) = pinned_{idx} {{\n\
                 {indent}            for i in 0..N {{\n\
                 {indent}                if i != {node} {{ rhs_as[i] -= g_as[i][{node}] * c_k; }}\n\
                 {indent}                g_as[i][{node}] = 0.0;\n\
                 {indent}            }}\n\
                 {indent}            for j in 0..N {{ g_as[{node}][j] = 0.0; }}\n\
                 {indent}            g_as[{node}][{node}] = 1.0;\n\
                 {indent}            rhs_as[{node}] = c_k;\n\
                 {indent}        }}\n",
                idx = idx, node = node,
            ));
        }

        // Step 4: dense LU solve of the constrained system.
        code.push_str(&format!(
            "{indent}        let mut v_as = rhs_as;\n\
             {indent}        if lu_solve(&mut g_as, &mut v_as) {{\n\
             {indent}            v = v_as;\n",
        ));

        // Step 5: re-evaluate i_nl at the new v so next sample's trapezoidal
        // history is consistent. Reuses the existing
        // `emit_nodal_device_evaluation_final` helper in a nested scope to
        // re-bind `v_nl_final` without colliding with any outer binding.
        if m > 0 {
            code.push_str(&format!("{indent}            {{\n"));
            code.push_str(&format!(
                "{indent}                let mut v_nl_final = [0.0f64; M];\n\
                 {indent}                for i in 0..M {{\n\
                 {indent}                    let mut sum = 0.0;\n\
                 {indent}                    for j in 0..N {{ sum += N_V[i][j] * v[j]; }}\n\
                 {indent}                    v_nl_final[i] = sum;\n\
                 {indent}                }}\n",
            ));
            Self::emit_nodal_device_evaluation_final(
                code,
                ir,
                &format!("{indent}                "),
            );
            code.push_str(&format!("{indent}            }}\n"));
        }

        code.push_str(&format!(
            "{indent}        }} else {{\n\
             {indent}            // LU failed — keep unclamped v. The output-stage\n\
             {indent}            // clamp and diag counters still catch pathological cases.\n\
             {indent}            state.diag_nr_max_iter_count += 1;\n\
             {indent}        }}\n\
             {indent}    }}\n\
             {indent}}}\n",
        ));
    }

    /// Emit per-op-amp slew-rate limiting on the converged node voltages.
    ///
    /// For each op-amp whose `.model OA(SR=…)` sets a finite slew rate, this
    /// emits a per-sample voltage-delta clamp of the form
    ///
    /// ```ignore
    /// {
    ///     let prev = state.v_prev[OUT];
    ///     let max_dv = OA{idx}_SR * (1.0 / state.current_sample_rate);
    ///     let delta  = v_name[OUT] - prev;
    ///     v_name[OUT] = prev + delta.clamp(-max_dv, max_dv);
    /// }
    /// ```
    ///
    /// where `OA{idx}_SR` is a per-device constant in V/s and
    /// `state.current_sample_rate` is the internal (post-oversampling)
    /// sample rate.
    ///
    /// ## Physical justification
    ///
    /// In the Boyle macromodel the dominant pole is a cap `C_dom` integrating
    /// the `Gm*(v+ - v-)` current at an internal gain node. Real op-amps
    /// slew-limit because their input stage can't source more than `I_tail`
    /// into `C_dom`, capping `|dV/dt| = I_tail / C_dom ≡ SR`. The equivalent
    /// integrator current limit is `I_slew = SR * C_dom`. melange currently
    /// stamps the op-amp as an ideal VCCS (no explicit integrator), so the
    /// same limit is applied directly in voltage space as
    /// `|Δv_out| ≤ SR*dt`. This is numerically identical to
    /// `|i_in_integrator| ≤ I_slew`, because
    /// `Δv_integrator = (i_in*dt)/C_dom`.
    ///
    /// ## Rail-mode interaction
    ///
    /// The slew limit is applied AFTER the rail clamp so slew-limited
    /// transients can't overshoot the rails, and BEFORE `state.v_prev = v`
    /// so the cap history `(2/T)·C·v_prev` term on the next sample reflects
    /// the slew-limited voltage (preserving KCL). The limit is compatible
    /// with `None`, `Hard`, `ActiveSet`, and `ActiveSetBe` rail modes.
    /// `BoyleDiodes` mode is supported but note that the existing
    /// BoyleDiodes heavy-clip convergence issues (see
    /// `docs/aidocs/OPAMP_RAIL_MODES.md`) are independent of slew limiting.
    ///
    /// No code is emitted when all op-amps have `sr = INFINITY`, so
    /// circuits without `SR=` in their .model card produce byte-identical
    /// generated code to the pre-slew-rate behaviour.
    fn emit_opamp_slew_limit(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
        v_name: &str,
    ) {
        let slew_opamps: Vec<(usize, &crate::codegen::ir::OpampIR)> = ir
            .opamps
            .iter()
            .enumerate()
            .filter(|(_, oa)| oa.sr.is_finite())
            .collect();
        if slew_opamps.is_empty() {
            return;
        }
        code.push_str(&format!(
            "{indent}// Op-amp slew-rate limiting: clamp |Δv_out| ≤ SR*dt\n"
        ));
        code.push_str(&format!(
            "{indent}// (equivalent to clamping Boyle C_dom integrator input to ±SR*C_dom)\n"
        ));
        code.push_str(&format!(
            "{indent}let _oa_slew_dt = 1.0 / state.current_sample_rate;\n"
        ));
        for (idx, oa) in &slew_opamps {
            code.push_str(&format!(
                "{indent}{{\n\
                 {indent}    let prev = state.v_prev[{node}];\n\
                 {indent}    let max_dv = OA{idx}_SR * _oa_slew_dt;\n\
                 {indent}    let delta = {v_name}[{node}] - prev;\n\
                 {indent}    {v_name}[{node}] = prev + delta.clamp(-max_dv, max_dv);\n\
                 {indent}}}\n",
                node = oa.n_out_idx,
            ));
        }
    }

    /// Emit a cheap rail-violation check that sets `<flag_name> = true` if any
    /// clampable op-amp output is outside its VCC/VEE range. Used in the
    /// trapezoidal NR path to decide whether to fall through to the BE
    /// fallback (where the active-set resolve runs against BE matrices, which
    /// don't develop the Nyquist limit cycle that trap+pin does).
    ///
    /// The caller must have `v` (immutable read access) and the boolean flag
    /// in scope. This emits no resolve, no LU solve, no state mutation — only
    /// the check.
    pub(super) fn emit_nodal_active_set_check(
        code: &mut String,
        ir: &CircuitIR,
        indent: &str,
        flag_name: &str,
    ) {
        let clampable: Vec<&crate::codegen::ir::OpampIR> = ir
            .opamps
            .iter()
            .filter(|oa| oa.vclamp_hi.is_finite() || oa.vclamp_lo.is_finite())
            .collect();
        if clampable.is_empty() {
            return;
        }

        for oa in &clampable {
            code.push_str(&format!(
                "{indent}if v[{node}] > {hi:.17e} || v[{node}] < {lo:.17e} {{ {flag_name} = true; }}\n",
                node = oa.n_out_idx,
                hi = oa.vclamp_hi,
                lo = oa.vclamp_lo,
            ));
        }
    }

    /// Emit device evaluation code WITHOUT declarations (writes to existing i_nl, j_dev).
    pub(super) fn emit_nodal_device_evaluation_body(code: &mut String, ir: &CircuitIR, indent: &str) {
        let m = ir.topology.m;

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            let s = slot.start_idx;
            // Pre-compute flat j_dev index for diagonal (avoids `0 * M + 0` identity_op in generated code)
            let jd_ss = s * m + s;
            match (&slot.device_type, &slot.params) {
                (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                    if dp.has_rs() {
                        // Series resistance: use helper functions
                        let bv_i = if dp.has_bv() {
                            format!(
                                " + diode_breakdown_current(v_nl[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV)"
                            )
                        } else {
                            String::new()
                        };
                        let bv_g = if dp.has_bv() {
                            format!(
                                " + diode_breakdown_conductance(v_nl[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV)"
                            )
                        } else {
                            String::new()
                        };
                        code.push_str(&format!(
                            "{indent}{{ // Diode {dev_num} (RS={has_rs}, BV={has_bv})\n\
                             {indent}    i_nl[{s}] = diode_current_with_rs(v_nl[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_RS){bv_i};\n\
                             {indent}    j_dev[{jd_ss}] = diode_conductance_with_rs(v_nl[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_RS){bv_g};\n\
                             {indent}}}\n",
                            has_rs = dp.has_rs(), has_bv = dp.has_bv(),
                        ));
                    } else if dp.has_bv() {
                        // Breakdown only (no RS): inline standard + breakdown
                        code.push_str(&format!(
                            "{indent}{{ // Diode {dev_num} (BV)\n\
                             {indent}    let v = v_nl[{s}];\n\
                             {indent}    let v_clamped = v.clamp(-40.0 * state.device_{dev_num}_n_vt, 40.0 * state.device_{dev_num}_n_vt);\n\
                             {indent}    let e = fast_exp(v_clamped / state.device_{dev_num}_n_vt);\n\
                             {indent}    i_nl[{s}] = state.device_{dev_num}_is * (e - 1.0) + diode_breakdown_current(v, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV);\n\
                             {indent}    let g = state.device_{dev_num}_is * e / state.device_{dev_num}_n_vt;\n\
                             {indent}    let g_base = if v > 40.0 * state.device_{dev_num}_n_vt {{ g + state.device_{dev_num}_is / state.device_{dev_num}_n_vt }} else {{ g }};\n\
                             {indent}    j_dev[{jd_ss}] = g_base + diode_breakdown_conductance(v, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV);\n\
                             {indent}}}\n"
                        ));
                    } else {
                        // Standard diode (no RS, no BV)
                        code.push_str(&format!(
                            "{indent}{{ // Diode {dev_num}\n\
                             {indent}    let v = v_nl[{s}];\n\
                             {indent}    let v_clamped = v.clamp(-40.0 * state.device_{dev_num}_n_vt, 40.0 * state.device_{dev_num}_n_vt);\n\
                             {indent}    let e = fast_exp(v_clamped / state.device_{dev_num}_n_vt);\n\
                             {indent}    i_nl[{s}] = state.device_{dev_num}_is * (e - 1.0);\n\
                             {indent}    let g = state.device_{dev_num}_is * e / state.device_{dev_num}_n_vt;\n\
                             {indent}    j_dev[{jd_ss}] = if v > 40.0 * state.device_{dev_num}_n_vt {{ g + state.device_{dev_num}_is / state.device_{dev_num}_n_vt }} else {{ g }};\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::Bjt, DeviceParams::Bjt(bp)) => {
                    let s1 = s + 1;
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    if bp.has_parasitics() && !slot.has_internal_mna_nodes {
                        code.push_str(&format!(
                            "{indent}{{ // BJT {dev_num} (RB/RC/RE inner NR)\n\
                             {indent}    let vbe = v_nl[{s}];\n\
                             {indent}    let vbc = v_nl[{s1}];\n\
                             {indent}    let (ic, ib, jac) = bjt_with_parasitics(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, DEVICE_{dev_num}_NR, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_ISC, DEVICE_{dev_num}_NC, DEVICE_{dev_num}_RB, DEVICE_{dev_num}_RC, DEVICE_{dev_num}_RE);\n\
                             {indent}    i_nl[{s}] = ic;\n\
                             {indent}    i_nl[{s1}] = ib;\n\
                             {indent}    j_dev[{jd_ss}] = jac[0];\n\
                             {indent}    j_dev[{jd_01}] = jac[1];\n\
                             {indent}    j_dev[{jd_10}] = jac[2];\n\
                             {indent}    j_dev[{jd_11}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    } else {
                        let mna_note = if slot.has_internal_mna_nodes {
                            " (MNA internal nodes)"
                        } else {
                            ""
                        };
                        code.push_str(&format!(
                            "{indent}{{ // BJT {dev_num}{mna_note}\n\
                             {indent}    let vbe = v_nl[{s}];\n\
                             {indent}    let vbc = v_nl[{s1}];\n\
                             {indent}    let (ic, ib, jac) = bjt_evaluate(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, DEVICE_{dev_num}_NR, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_ISC, DEVICE_{dev_num}_NC);\n\
                             {indent}    i_nl[{s}] = ic;\n\
                             {indent}    i_nl[{s1}] = ib;\n\
                             {indent}    j_dev[{jd_ss}] = jac[0];\n\
                             {indent}    j_dev[{jd_01}] = jac[1];\n\
                             {indent}    j_dev[{jd_10}] = jac[2];\n\
                             {indent}    j_dev[{jd_11}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::BjtForwardActive, DeviceParams::Bjt(_bp)) => {
                    // 1D forward-active BJT: only Vbe→Ic, single jdev entry
                    code.push_str(&format!(
                        "{indent}{{ // BJT {dev_num} forward-active (1D)\n\
                         {indent}    let vbe = v_nl[{s}] * DEVICE_{dev_num}_SIGN;\n\
                         {indent}    let exp_be = fast_exp(vbe / (DEVICE_{dev_num}_NF * state.device_{dev_num}_vt));\n\
                         {indent}    i_nl[{s}] = state.device_{dev_num}_is * (exp_be - 1.0) * DEVICE_{dev_num}_SIGN;\n\
                         {indent}    j_dev[{jd_ss}] = state.device_{dev_num}_is / (DEVICE_{dev_num}_NF * state.device_{dev_num}_vt) * exp_be;\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Jfet, DeviceParams::Jfet(jp)) => {
                    let s1 = s + 1;
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    let jac_fn = if jp.has_rd_rs() {
                        format!(
                            "jfet_jacobian_with_rd_rs(vgs, vds, state.device_{dev_num}_idss, state.device_{dev_num}_vp, state.device_{dev_num}_lambda, sign, DEVICE_{dev_num}_RD, DEVICE_{dev_num}_RS)"
                        )
                    } else {
                        format!(
                            "jfet_jacobian(vgs, vds, state.device_{dev_num}_idss, state.device_{dev_num}_vp, state.device_{dev_num}_lambda, sign)"
                        )
                    };
                    code.push_str(&format!(
                        "{indent}{{ // JFET {dev_num}\n\
                         {indent}    let vds = v_nl[{s}];\n\
                         {indent}    let vgs = v_nl[{s1}];\n\
                         {indent}    let sign = DEVICE_{dev_num}_SIGN;\n\
                         {indent}    i_nl[{s}] = jfet_id(vgs, vds, state.device_{dev_num}_idss, state.device_{dev_num}_vp, state.device_{dev_num}_lambda, sign);\n\
                         {indent}    i_nl[{s1}] = jfet_ig(vgs, sign);\n\
                         {indent}    let jac = {jac_fn};\n\
                         {indent}    j_dev[{jd_ss}] = jac[0];\n\
                         {indent}    j_dev[{jd_01}] = jac[1];\n\
                         {indent}    j_dev[{jd_10}] = jac[2];\n\
                         {indent}    j_dev[{jd_11}] = jac[3];\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Mosfet, DeviceParams::Mosfet(mp)) => {
                    let s1 = s + 1;
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    // For body effect, compute VT_eff from node voltages at each NR iteration
                    let vt_expr = if mp.has_body_effect() {
                        let vs_expr = if mp.source_node > 0 {
                            format!("v[{}]", mp.source_node - 1)
                        } else {
                            "0.0".to_string()
                        };
                        let vb_expr = if mp.bulk_node > 0 {
                            format!("v[{}]", mp.bulk_node - 1)
                        } else {
                            "0.0".to_string()
                        };
                        let sign_val = if mp.is_p_channel { -1.0 } else { 1.0 };
                        code.push_str(&format!(
                            "{indent}{{ // MOSFET {dev_num} body effect\n\
                             {indent}    let vsb = {sign_val:.1} * ({vs_expr} - {vb_expr});\n\
                             {indent}    let vt_eff = DEVICE_{dev_num}_VT + DEVICE_{dev_num}_GAMMA * ((DEVICE_{dev_num}_PHI + vsb.max(0.0)).sqrt() - DEVICE_{dev_num}_PHI.sqrt());\n\
                             {indent}    state.device_{dev_num}_vt = vt_eff;\n"
                        ));
                        "vt_eff".to_string()
                    } else {
                        code.push_str(&format!("{indent}{{ // MOSFET {dev_num}\n"));
                        format!("state.device_{dev_num}_vt")
                    };
                    let jac_fn = if mp.has_rd_rs() {
                        format!(
                            "mosfet_jacobian_with_rd_rs(vgs, vds, state.device_{dev_num}_kp, {vt_expr}, state.device_{dev_num}_lambda, sign, DEVICE_{dev_num}_RD, DEVICE_{dev_num}_RS)"
                        )
                    } else {
                        format!(
                            "mosfet_jacobian(vgs, vds, state.device_{dev_num}_kp, {vt_expr}, state.device_{dev_num}_lambda, sign)"
                        )
                    };
                    code.push_str(&format!(
                        "{indent}    let vds = v_nl[{s}];\n\
                         {indent}    let vgs = v_nl[{s1}];\n\
                         {indent}    let sign = DEVICE_{dev_num}_SIGN;\n\
                         {indent}    i_nl[{s}] = mosfet_id(vgs, vds, state.device_{dev_num}_kp, {vt_expr}, state.device_{dev_num}_lambda, sign);\n\
                         {indent}    i_nl[{s1}] = 0.0; // Insulated gate\n\
                         {indent}    let jac = {jac_fn};\n\
                         {indent}    j_dev[{jd_ss}] = jac[0];\n\
                         {indent}    j_dev[{jd_01}] = jac[1];\n\
                         {indent}    j_dev[{jd_10}] = jac[2];\n\
                         {indent}    j_dev[{jd_11}] = jac[3];\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Tube, DeviceParams::Tube(tp)) => {
                    let s1 = s + 1;
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    if tp.is_pentode() {
                        // Pentode / beam tetrode NR block. See
                        // [`pentode_dispatch`] for the 8-way helper family
                        // selection (5 sharp/var-mu × 3 grid-off wrappers).
                        let dispatch = pentode_dispatch(tp, dev_num);
                        let helper_suffix = dispatch.suffix;
                        let eval_args = &dispatch.eval_args;
                        if dispatch.is_grid_off {
                            // Grid-off 2D reduction: Ig1 dropped, Vg2k frozen.
                            // Wrapper returns (ip, ig2, [f64;4]) — only 2×2
                            // stamps go into j_dev.
                            code.push_str(&format!(
                                "{indent}{{ // Pentode {dev_num} (grid-off)\n\
                                 {indent}    let vgk = v_nl[{s}];\n\
                                 {indent}    let vpk = v_nl[{s1}];\n\
                                 {indent}    let (ip_t, ig2_t, jac) = tube_evaluate_{helper_suffix}(vgk, vpk, DEVICE_{dev_num}_VG2K_FROZEN, {eval_args});\n\
                                 {indent}    i_nl[{s}] = ip_t; i_nl[{s1}] = ig2_t;\n\
                                 {indent}    j_dev[{jd_ss}] = jac[0];\n\
                                 {indent}    j_dev[{jd_01}] = jac[1];\n\
                                 {indent}    j_dev[{jd_10}] = jac[2];\n\
                                 {indent}    j_dev[{jd_11}] = jac[3];\n\
                                 {indent}}}\n"
                            ));
                        } else {
                            let s2 = s + 2;
                            let jd_02 = s * m + s2;
                            let jd_12 = s1 * m + s2;
                            let jd_20 = s2 * m + s;
                            let jd_21 = s2 * m + s1;
                            let jd_22 = s2 * m + s2;
                            code.push_str(&format!(
                                "{indent}{{ // Pentode {dev_num}\n\
                                 {indent}    let vgk = v_nl[{s}];\n\
                                 {indent}    let vpk = v_nl[{s1}];\n\
                                 {indent}    let vg2k = v_nl[{s2}];\n\
                                 {indent}    let (ip_t, ig2_t, ig1_t, jac) = tube_evaluate_{helper_suffix}(vgk, vpk, vg2k, {eval_args});\n\
                                 {indent}    i_nl[{s}] = ip_t; i_nl[{s1}] = ig2_t; i_nl[{s2}] = ig1_t;\n\
                                 {indent}    j_dev[{jd_ss}] = jac[0];\n\
                                 {indent}    j_dev[{jd_01}] = jac[1];\n\
                                 {indent}    j_dev[{jd_02}] = jac[2];\n\
                                 {indent}    j_dev[{jd_10}] = jac[3];\n\
                                 {indent}    j_dev[{jd_11}] = jac[4];\n\
                                 {indent}    j_dev[{jd_12}] = jac[5];\n\
                                 {indent}    j_dev[{jd_20}] = jac[6];\n\
                                 {indent}    j_dev[{jd_21}] = jac[7];\n\
                                 {indent}    j_dev[{jd_22}] = jac[8];\n\
                                 {indent}}}\n"
                            ));
                        }
                    } else if tp.has_rgi() {
                        code.push_str(&format!(
                            "{indent}{{ // Tube {dev_num} (RGI)\n\
                             {indent}    let vgk = v_nl[{s}];\n\
                             {indent}    let vpk = v_nl[{s1}];\n\
                             {indent}    let (ip_t, ig_t, jac) = tube_evaluate_with_rgi(vgk, vpk, state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset, state.device_{dev_num}_lambda, DEVICE_{dev_num}_RGI);\n\
                             {indent}    i_nl[{s}] = ip_t; i_nl[{s1}] = ig_t;\n\
                             {indent}    j_dev[{jd_ss}] = jac[0];\n\
                             {indent}    j_dev[{jd_01}] = jac[1];\n\
                             {indent}    j_dev[{jd_10}] = jac[2];\n\
                             {indent}    j_dev[{jd_11}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}{{ // Tube {dev_num}\n\
                             {indent}    let vgk = v_nl[{s}];\n\
                             {indent}    let vpk = v_nl[{s1}];\n\
                             {indent}    let (ip_t, ig_t, jac) = tube_evaluate(vgk, vpk, state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset, state.device_{dev_num}_lambda);\n\
                             {indent}    i_nl[{s}] = ip_t; i_nl[{s1}] = ig_t;\n\
                             {indent}    j_dev[{jd_ss}] = jac[0];\n\
                             {indent}    j_dev[{jd_01}] = jac[1];\n\
                             {indent}    j_dev[{jd_10}] = jac[2];\n\
                             {indent}    j_dev[{jd_11}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::Vca, DeviceParams::Vca(_vp)) => {
                    let s1 = s + 1;
                    let jd_01 = s * m + s1;
                    let jd_10 = s1 * m + s;
                    let jd_11 = s1 * m + s1;
                    code.push_str(&format!(
                        "{indent}{{ // VCA {dev_num}\n\
                         {indent}    let v_sig = v_nl[{s}];\n\
                         {indent}    let v_ctrl = v_nl[{s1}];\n\
                         {indent}    i_nl[{s}] = vca_current(v_sig, v_ctrl, state.device_{dev_num}_g0, state.device_{dev_num}_vscale, DEVICE_{dev_num}_THD);\n\
                         {indent}    i_nl[{s1}] = 0.0;\n\
                         {indent}    let jac = vca_jacobian(v_sig, v_ctrl, state.device_{dev_num}_g0, state.device_{dev_num}_vscale, DEVICE_{dev_num}_THD);\n\
                         {indent}    j_dev[{jd_ss}] = jac[0];\n\
                         {indent}    j_dev[{jd_01}] = jac[1];\n\
                         {indent}    j_dev[{jd_10}] = jac[2];\n\
                         {indent}    j_dev[{jd_11}] = jac[3];\n\
                         {indent}}}\n"
                    ));
                }
                _ => {} // Mismatched type/params — skip
            }
        }
    }

    /// Emit final device evaluation at converged point (writes into existing `i_nl`).
    pub(super) fn emit_nodal_device_evaluation_final(code: &mut String, ir: &CircuitIR, indent: &str) {
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            let s = slot.start_idx;
            match (&slot.device_type, &slot.params) {
                (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                    if dp.has_rs() {
                        let bv_i = if dp.has_bv() {
                            format!(
                                " + diode_breakdown_current(v_nl_final[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV)"
                            )
                        } else {
                            String::new()
                        };
                        code.push_str(&format!(
                            "{indent}i_nl[{s}] = diode_current_with_rs(v_nl_final[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_RS){bv_i};\n"
                        ));
                    } else if dp.has_bv() {
                        code.push_str(&format!(
                            "{indent}{{ let v = v_nl_final[{s}];\n\
                             {indent}  let v_clamped = v.clamp(-40.0 * state.device_{dev_num}_n_vt, 40.0 * state.device_{dev_num}_n_vt);\n\
                             {indent}  i_nl[{s}] = state.device_{dev_num}_is * (fast_exp(v_clamped / state.device_{dev_num}_n_vt) - 1.0) + diode_breakdown_current(v, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV);\n\
                             {indent}}}\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}{{ let v = v_nl_final[{s}];\n\
                             {indent}  let v_clamped = v.clamp(-40.0 * state.device_{dev_num}_n_vt, 40.0 * state.device_{dev_num}_n_vt);\n\
                             {indent}  i_nl[{s}] = state.device_{dev_num}_is * (fast_exp(v_clamped / state.device_{dev_num}_n_vt) - 1.0);\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::Bjt, DeviceParams::Bjt(bp)) => {
                    let s1 = s + 1;
                    if bp.has_parasitics() && !slot.has_internal_mna_nodes {
                        code.push_str(&format!(
                            "{indent}{{ let vbe = v_nl_final[{s}];\n\
                             {indent}  let vbc = v_nl_final[{s1}];\n\
                             {indent}  let (ic, ib, _jac) = bjt_with_parasitics(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, DEVICE_{dev_num}_NR, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_ISC, DEVICE_{dev_num}_NC, DEVICE_{dev_num}_RB, DEVICE_{dev_num}_RC, DEVICE_{dev_num}_RE);\n\
                             {indent}  i_nl[{s}] = ic;\n\
                             {indent}  i_nl[{s1}] = ib;\n\
                             {indent}}}\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}{{ let vbe = v_nl_final[{s}];\n\
                             {indent}  let vbc = v_nl_final[{s1}];\n\
                             {indent}  let (ic, ib, _) = bjt_evaluate(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, DEVICE_{dev_num}_NR, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_ISC, DEVICE_{dev_num}_NC);\n\
                             {indent}  i_nl[{s}] = ic;\n\
                             {indent}  i_nl[{s1}] = ib;\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::BjtForwardActive, DeviceParams::Bjt(_bp)) => {
                    // 1D forward-active BJT: only Vbe→Ic
                    code.push_str(&format!(
                        "{indent}{{ let vbe = v_nl_final[{s}] * DEVICE_{dev_num}_SIGN;\n\
                         {indent}  let exp_be = fast_exp(vbe / (DEVICE_{dev_num}_NF * state.device_{dev_num}_vt));\n\
                         {indent}  i_nl[{s}] = state.device_{dev_num}_is * (exp_be - 1.0) * DEVICE_{dev_num}_SIGN;\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Jfet, DeviceParams::Jfet(_jp)) => {
                    let s1 = s + 1;
                    code.push_str(&format!(
                        "{indent}i_nl[{s}] = jfet_id(v_nl_final[{s1}], v_nl_final[{s}], state.device_{dev_num}_idss, state.device_{dev_num}_vp, state.device_{dev_num}_lambda, DEVICE_{dev_num}_SIGN);\n\
                         {indent}i_nl[{s1}] = jfet_ig(v_nl_final[{s1}], DEVICE_{dev_num}_SIGN);\n"
                    ));
                }
                (DeviceType::Mosfet, DeviceParams::Mosfet(_mp)) => {
                    let s1 = s + 1;
                    code.push_str(&format!(
                        "{indent}i_nl[{s}] = mosfet_id(v_nl_final[{s1}], v_nl_final[{s}], state.device_{dev_num}_kp, state.device_{dev_num}_vt, state.device_{dev_num}_lambda, DEVICE_{dev_num}_SIGN);\n\
                         {indent}i_nl[{s1}] = 0.0;\n"
                    ));
                }
                (DeviceType::Tube, DeviceParams::Tube(tp)) => {
                    let s1 = s + 1;
                    if tp.is_pentode() {
                        let dispatch = pentode_dispatch(tp, dev_num);
                        let helper_suffix = dispatch.suffix;
                        let ip_args = &dispatch.ip_args;
                        let is_args = &dispatch.is_args;
                        if dispatch.is_grid_off {
                            // Grid-off 2D: read Vg2k from the frozen constant,
                            // stamp only Ip and Ig2. Ig1 is identically zero
                            // and the slot contributes no s+2 dimension.
                            code.push_str(&format!(
                                "{indent}i_nl[{s}] = tube_ip_{helper_suffix}(v_nl_final[{s}], v_nl_final[{s1}], DEVICE_{dev_num}_VG2K_FROZEN, {ip_args});\n\
                                 {indent}i_nl[{s1}] = tube_is_{helper_suffix}(v_nl_final[{s}], v_nl_final[{s1}], DEVICE_{dev_num}_VG2K_FROZEN, {is_args});\n"
                            ));
                        } else {
                            let s2 = s + 2;
                            code.push_str(&format!(
                                "{indent}i_nl[{s}] = tube_ip_{helper_suffix}(v_nl_final[{s}], v_nl_final[{s1}], v_nl_final[{s2}], {ip_args});\n\
                                 {indent}i_nl[{s1}] = tube_is_{helper_suffix}(v_nl_final[{s}], v_nl_final[{s1}], v_nl_final[{s2}], {is_args});\n\
                                 {indent}i_nl[{s2}] = tube_ig(v_nl_final[{s}], state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset);\n"
                            ));
                        }
                    } else if tp.has_rgi() {
                        code.push_str(&format!(
                            "{indent}i_nl[{s}] = tube_ip_with_rgi(v_nl_final[{s}], v_nl_final[{s1}], state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_lambda, state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset, DEVICE_{dev_num}_RGI);\n\
                             {indent}i_nl[{s1}] = tube_ig_with_rgi(v_nl_final[{s}], state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset, DEVICE_{dev_num}_RGI);\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}i_nl[{s}] = tube_ip(v_nl_final[{s}], v_nl_final[{s1}], state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_lambda);\n\
                             {indent}i_nl[{s1}] = tube_ig(v_nl_final[{s}], state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset);\n"
                        ));
                    }
                }
                (DeviceType::Vca, DeviceParams::Vca(_vp)) => {
                    let s1 = s + 1;
                    code.push_str(&format!(
                        "{indent}i_nl[{s}] = vca_current(v_nl_final[{s}], v_nl_final[{s1}], state.device_{dev_num}_g0, state.device_{dev_num}_vscale, DEVICE_{dev_num}_THD);\n\
                         {indent}i_nl[{s1}] = 0.0;\n"
                    ));
                }
                _ => {}
            }
        }
    }

    /// Emit SPICE voltage limiting for nodal solver (trapezoidal NR, at default indent).
    pub(super) fn emit_nodal_voltage_limiting(code: &mut String, ir: &CircuitIR) {
        Self::emit_nodal_voltage_limiting_indented(code, ir, "        ");
    }

    /// Emit SPICE voltage limiting for nodal solver at a given indent level.
    ///
    /// For each nonlinear device dimension, computes the proposed device voltage
    /// from v_new via N_v, applies pnjlim/fetlim, and reduces alpha if needed.
    pub(super) fn emit_nodal_voltage_limiting_indented(code: &mut String, ir: &CircuitIR, indent: &str) {
        let n = ir.topology.n;

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            for d in 0..slot.dimension {
                let i = slot.start_idx + d;

                // Compute proposed device voltage from v_new via N_v
                code.push_str(&format!("{indent}{{ // Device {dev_num} dim {d}\n"));
                code.push_str(&emit_sparse_nv_dot(ir, i, "v_nl_proposed", "v_new", &format!("{indent}    ")));
                code.push_str(&format!("{indent}    let v_nl_current = v_nl[{i}];\n"));
                code.push_str(&format!(
                    "{indent}    let dv = v_nl_proposed - v_nl_current;\n"
                ));
                code.push_str(&format!("{indent}    if dv.abs() > 1e-15 {{\n"));

                // Apply per-device limiter
                match (&slot.device_type, d) {
                    (DeviceType::Diode, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = pnjlim(v_nl_proposed, v_nl_current, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (DeviceType::Bjt, _) | (DeviceType::BjtForwardActive, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = pnjlim(v_nl_proposed, v_nl_current, state.device_{dev_num}_vt, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (DeviceType::Jfet, 0) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = fetlim(v_nl_proposed, v_nl_current, 0.0);\n"
                        ));
                    }
                    (DeviceType::Jfet, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = fetlim(v_nl_proposed, v_nl_current, state.device_{dev_num}_vp);\n"
                        ));
                    }
                    (DeviceType::Mosfet, 0) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = fetlim(v_nl_proposed, v_nl_current, 0.0);\n"
                        ));
                    }
                    (DeviceType::Mosfet, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = fetlim(v_nl_proposed, v_nl_current, state.device_{dev_num}_vt);\n"
                        ));
                    }
                    (DeviceType::Tube, 0) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = pnjlim(v_nl_proposed, v_nl_current, state.device_{dev_num}_vgk_onset / 3.0, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (DeviceType::Tube, 2) => {
                        // Pentode dim 2 = Vg2k — log-junction limiting (see DK NR limiter).
                        code.push_str(&format!(
                            "{indent}        let v_lim = pnjlim(v_nl_proposed, v_nl_current, state.device_{dev_num}_vgk_onset / 3.0, DEVICE_{dev_num}_VCRIT);\n"
                        ));
                    }
                    (DeviceType::Tube, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = fetlim(v_nl_proposed, v_nl_current, 0.0);\n"
                        ));
                    }
                    (DeviceType::Vca, _) => {
                        // VCA: no junction limiting needed — fast_exp already clamps
                        code.push_str(&format!("{indent}        let v_lim = v_nl_proposed;\n"));
                    }
                }

                code.push_str(&format!(
                    "{indent}        let ratio = ((v_lim - v_nl_current) / dv).clamp(0.01, 1.0);\n\
                     {indent}        if ratio < alpha {{ alpha = ratio; }}\n\
                     {indent}    }}\n\
                     {indent}}}\n"
                ));
            }
        }

        let _ = n;
    }
}
