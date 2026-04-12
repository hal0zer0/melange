//! DK Newton-Raphson solver generation.
//!
//! Contains `generate_solve_nonlinear`, `generate_gauss_elim`, and
//! `generate_schur_gauss_elim` — the procedural NR solver code that is
//! too deeply conditional for Tera templates.

use crate::codegen::ir::{CircuitIR, DeviceParams, DeviceType};
use crate::codegen::CodegenError;
use super::RustEmitter;
use super::helpers::emit_pentode_nr_dk_stamp;
use super::nr_helpers::{
    emit_nr_singular_fallback, emit_nr_limit_and_converge, emit_schur_nr_limit_and_converge,
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
            // Device currents and Jacobian entries
            code.push_str("        // Evaluate device currents and Jacobians\n");
            for (dev_num, slot) in ir.device_slots.iter().enumerate() {
                match slot.device_type {
                    DeviceType::Diode => {
                        let s = slot.start_idx;
                        let d = dev_num;
                        let dp = match &slot.params {
                            DeviceParams::Diode(dp) => dp,
                            other => return Err(CodegenError::InvalidDevice(format!("device_type=Diode but params={:?}", other))),
                        };
                        if dp.has_rs() && dp.has_bv() {
                            // RS + BV: solve inner NR for junction voltage, then add breakdown
                            code.push_str(&format!(
                                "        let i_dev{s} = diode_current_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS) + diode_breakdown_current(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n"
                            ));
                            code.push_str(&format!(
                                "        let jdev_{s}_{s} = diode_conductance_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS) + diode_breakdown_conductance(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n"
                            ));
                        } else if dp.has_rs() {
                            // RS only: solve inner NR for junction voltage
                            code.push_str(&format!(
                                "        let i_dev{s} = diode_current_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS);\n"
                            ));
                            code.push_str(&format!(
                                "        let jdev_{s}_{s} = diode_conductance_with_rs(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt, DEVICE_{d}_RS);\n"
                            ));
                        } else if dp.has_bv() {
                            // BV only: add breakdown to standard diode
                            code.push_str(&format!(
                                "        let i_dev{s} = diode_current(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt) + diode_breakdown_current(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n"
                            ));
                            code.push_str(&format!(
                                "        let jdev_{s}_{s} = diode_conductance(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt) + diode_breakdown_conductance(v_d{s}, state.device_{d}_n_vt, DEVICE_{d}_BV, DEVICE_{d}_IBV);\n"
                            ));
                        } else {
                            // Standard diode (no RS, no BV)
                            code.push_str(&format!(
                                "        let i_dev{s} = diode_current(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt);\n"
                            ));
                            code.push_str(&format!(
                                "        let jdev_{s}_{s} = diode_conductance(v_d{s}, state.device_{d}_is, state.device_{d}_n_vt);\n"
                            ));
                        }
                    }
                    DeviceType::Bjt => {
                        let s = slot.start_idx;
                        let s1 = s + 1;
                        let d = dev_num;
                        let bp = match &slot.params {
                            DeviceParams::Bjt(bp) => bp,
                            other => return Err(CodegenError::InvalidDevice(format!("device_type=Bjt but params={:?}", other))),
                        };
                        if bp.has_parasitics() && !slot.has_internal_mna_nodes {
                            // Use inner 2D NR for parasitic resistances
                            code.push_str(&format!(
                                "        let (i_dev{s}, i_dev{s1}, bjt{d}_jac) = bjt_with_parasitics(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, DEVICE_{d}_NR, state.device_{d}_bf, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF, DEVICE_{d}_IKR, DEVICE_{d}_ISE, DEVICE_{d}_NE, DEVICE_{d}_ISC, DEVICE_{d}_NC, DEVICE_{d}_RB, DEVICE_{d}_RC, DEVICE_{d}_RE);\n"
                            ));
                        } else {
                            // Combined evaluation: shared exp() across ic, ib, jacobian
                            // (parasitics handled by MNA internal nodes when has_internal_mna_nodes)
                            code.push_str(&format!(
                                "        let (i_dev{s}, i_dev{s1}, bjt{d}_jac) = bjt_evaluate(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, DEVICE_{d}_NR, state.device_{d}_bf, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF, DEVICE_{d}_IKR, DEVICE_{d}_ISE, DEVICE_{d}_NE, DEVICE_{d}_ISC, DEVICE_{d}_NC);\n"
                            ));
                        }
                        code.push_str(&format!(
                            "        let jdev_{}_{} = bjt{}_jac[0];\n",
                            s, s, dev_num
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = bjt{}_jac[1];\n",
                            s, s1, dev_num
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = bjt{}_jac[2];\n",
                            s1, s, dev_num
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = bjt{}_jac[3];\n",
                            s1, s1, dev_num
                        ));
                    }
                    DeviceType::BjtForwardActive => {
                        // 1D forward-active BJT: only Vbe→Ic, Ib=Ic/BF folded into N_i
                        let s = slot.start_idx;
                        let d = dev_num;
                        code.push_str(&format!(
                            "        // BJT {d} forward-active (1D: Vbe→Ic only)\n"
                        ));
                        code.push_str(&format!(
                            "        let vbe_{d} = v_d{s} * DEVICE_{d}_SIGN;\n"
                        ));
                        code.push_str(&format!(
                            "        let exp_be_{d} = fast_exp(vbe_{d} / (DEVICE_{d}_NF * state.device_{d}_vt));\n"
                        ));
                        code.push_str(&format!(
                            "        let i_dev{s} = state.device_{d}_is * (exp_be_{d} - 1.0) * DEVICE_{d}_SIGN;\n"
                        ));
                        code.push_str(&format!(
                            "        let jdev_{s}_{s} = state.device_{d}_is / (DEVICE_{d}_NF * state.device_{d}_vt) * exp_be_{d};\n"
                        ));
                    }
                    DeviceType::Jfet => {
                        let s = slot.start_idx;
                        let s1 = s + 1;
                        let d = dev_num;
                        let jp = match &slot.params {
                            DeviceParams::Jfet(jp) => jp,
                            other => return Err(CodegenError::InvalidDevice(format!("device_type=Jfet but params={:?}", other))),
                        };
                        // IDSS, VP, LAMBDA from state; SIGN stays as const.
                        // N_v ordering: dim s = Vds, dim s+1 = Vgs.
                        // Functions expect (vgs, vds), so pass (v_d{s1}, v_d{s}).
                        code.push_str(&format!(
                            "        let i_dev{s} = jfet_id(v_d{s1}, v_d{s}, state.device_{d}_idss, state.device_{d}_vp, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n"
                        ));
                        code.push_str(&format!(
                            "        let i_dev{s1} = jfet_ig(v_d{s1}, DEVICE_{d}_SIGN);\n"
                        ));
                        if jp.has_rd_rs() {
                            code.push_str(&format!(
                                "        let jfet{d}_jac = jfet_jacobian_with_rd_rs(v_d{s1}, v_d{s}, state.device_{d}_idss, state.device_{d}_vp, state.device_{d}_lambda, DEVICE_{d}_SIGN, DEVICE_{d}_RD, DEVICE_{d}_RS);\n"
                            ));
                        } else {
                            code.push_str(&format!(
                                "        let jfet{d}_jac = jfet_jacobian(v_d{s1}, v_d{s}, state.device_{d}_idss, state.device_{d}_vp, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n"
                            ));
                        }
                        // In dim-space (dim0=Vds, dim1=Vgs):
                        //   jdev_s_s   = dId/dVds = jac[1]
                        //   jdev_s_s1  = dId/dVgs = jac[0]
                        //   jdev_s1_s  = dIg/dVds = jac[3]
                        //   jdev_s1_s1 = dIg/dVgs = jac[2]
                        code.push_str(&format!(
                            "        let jdev_{}_{} = jfet{}_jac[1];\n",
                            s,
                            s,
                            d // dId/dVds
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = jfet{}_jac[0];\n",
                            s,
                            s1,
                            d // dId/dVgs
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = jfet{}_jac[3];\n",
                            s1,
                            s,
                            d // dIg/dVds
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = jfet{}_jac[2];\n",
                            s1,
                            s1,
                            d // dIg/dVgs
                        ));
                    }
                    DeviceType::Mosfet => {
                        let s = slot.start_idx;
                        let s1 = s + 1;
                        let d = dev_num;
                        let mp = match &slot.params {
                            DeviceParams::Mosfet(mp) => mp,
                            other => return Err(CodegenError::InvalidDevice(format!("device_type=Mosfet but params={:?}", other))),
                        };
                        // KP, VT, LAMBDA from state; SIGN stays as const.
                        // N_v ordering: dim s = Vds, dim s+1 = Vgs.
                        // Functions expect (vgs, vds), so pass (v_d{s1}, v_d{s}).
                        code.push_str(&format!(
                            "        let i_dev{s} = mosfet_id(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n"
                        ));
                        code.push_str(&format!(
                            "        let i_dev{s1} = mosfet_ig(v_d{s1}, DEVICE_{d}_SIGN);\n"
                        ));
                        if mp.has_rd_rs() {
                            code.push_str(&format!(
                                "        let mos{d}_jac = mosfet_jacobian_with_rd_rs(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN, DEVICE_{d}_RD, DEVICE_{d}_RS);\n"
                            ));
                        } else {
                            code.push_str(&format!(
                                "        let mos{d}_jac = mosfet_jacobian(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n"
                            ));
                        }
                        code.push_str(&format!(
                            "        let jdev_{}_{} = mos{}_jac[1];\n",
                            s,
                            s,
                            d // dId/dVds
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = mos{}_jac[0];\n",
                            s,
                            s1,
                            d // dId/dVgs
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = mos{}_jac[3];\n",
                            s1,
                            s,
                            d // dIg/dVds = 0
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = mos{}_jac[2];\n",
                            s1,
                            s1,
                            d // dIg/dVgs = 0
                        ));
                    }
                    DeviceType::Tube => {
                        let s = slot.start_idx;
                        let s1 = s + 1;
                        let d = dev_num;
                        let tp = match &slot.params {
                            DeviceParams::Tube(tp) => tp,
                            other => return Err(CodegenError::InvalidDevice(format!("device_type=Tube but params={:?}", other))),
                        };
                        if tp.is_pentode() {
                            // Pentode / beam tetrode NR block. 3D (Vgk→Ip,
                            // Vpk→Ig2, Vg2k→Ig1) for sharp / variable-mu /
                            // Classical; 2D for grid-off (Vg2k frozen, Ig1
                            // dropped). See [`pentode_dispatch`] for the
                            // 8-way helper family selection.
                            emit_pentode_nr_dk_stamp(code, tp, d, s, "        ");
                        } else {
                            if tp.has_rgi() {
                                // RGI: solve for internal Vgk, evaluate at internal voltage
                                code.push_str(&format!(
                                    "        let (i_dev{s}, i_dev{s1}, tube{d}_jac) = tube_evaluate_with_rgi(v_d{s}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_ig_max, state.device_{d}_vgk_onset, state.device_{d}_lambda, DEVICE_{d}_RGI);\n"
                                ));
                            } else {
                                // Standard tube (no RGI)
                                code.push_str(&format!(
                                    "        let (i_dev{s}, i_dev{s1}, tube{d}_jac) = tube_evaluate(v_d{s}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_ig_max, state.device_{d}_vgk_onset, state.device_{d}_lambda);\n"
                                ));
                            }
                            code.push_str(&format!(
                                "        let jdev_{}_{} = tube{}_jac[0];\n",
                                s, s, d
                            ));
                            code.push_str(&format!(
                                "        let jdev_{}_{} = tube{}_jac[1];\n",
                                s, s1, d
                            ));
                            code.push_str(&format!(
                                "        let jdev_{}_{} = tube{}_jac[2];\n",
                                s1, s, d
                            ));
                            code.push_str(&format!(
                                "        let jdev_{}_{} = tube{}_jac[3];\n",
                                s1, s1, d
                            ));
                        }
                    }
                    DeviceType::Vca => {
                        let s = slot.start_idx;
                        let s1 = s + 1;
                        let d = dev_num;
                        // VCA dim 0 = V_sig, dim 1 = V_ctrl (direct mapping, no swap)
                        code.push_str(&format!(
                            "        let i_dev{s} = vca_current(v_d{s}, v_d{s1}, state.device_{d}_g0, state.device_{d}_vscale, DEVICE_{d}_THD);\n"
                        ));
                        code.push_str(&format!("        let i_dev{s1} = 0.0;\n"));
                        code.push_str(&format!(
                            "        let vca{d}_jac = vca_jacobian(v_d{s}, v_d{s1}, state.device_{d}_g0, state.device_{d}_vscale, DEVICE_{d}_THD);\n"
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = vca{}_jac[0];\n",
                            s, s, d
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = vca{}_jac[1];\n",
                            s, s1, d
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = vca{}_jac[2];\n",
                            s1, s, d
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = vca{}_jac[3];\n",
                            s1, s1, d
                        ));
                    }
                }
            }
            code.push('\n');

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
                    code.push_str(&format!("        let j{}{} = {}{};\n", i, j, diag, terms));
                }
            }
            code.push('\n');

            // Solve the linear system based on matrix size
            match m {
                1 => {
                    code.push_str("        // Solve 1x1 system: J * delta = f\n");
                    code.push_str("        let det = j00;\n");
                    code.push_str("        if det.abs() < SINGULARITY_THRESHOLD {\n");
                    emit_nr_singular_fallback(code, 1, "            ");
                    code.push_str("            continue;\n");
                    code.push_str("        }\n");
                    code.push_str("        let delta0 = f0 / det;\n\n");
                    emit_nr_limit_and_converge(code, ir, 1, "        ");
                }
                2 => {
                    code.push_str("        // Solve 2x2 system: J * delta = f (Cramer's rule)\n");
                    code.push_str("        let det = j00 * j11 - j01 * j10;\n");
                    code.push_str("        if det.abs() < SINGULARITY_THRESHOLD {\n");
                    emit_nr_singular_fallback(code, 2, "            ");
                    code.push_str("            continue;\n");
                    code.push_str("        }\n");
                    code.push_str("        let inv_det = 1.0 / det;\n");
                    code.push_str("        let delta0 = inv_det * (j11 * f0 - j01 * f1);\n");
                    code.push_str("        let delta1 = inv_det * (-j10 * f0 + j00 * f1);\n\n");
                    emit_nr_limit_and_converge(code, ir, 2, "        ");
                }
                3..=16 => {
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
                .map(|j| format!("j{i}{j}"))
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
                .map(|j| format!("j{i}{j}"))
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
