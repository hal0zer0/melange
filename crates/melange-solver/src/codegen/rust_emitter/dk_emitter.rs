//! DK-path code emission methods.
//!
//! Contains the DK entry point (`emit_dk`) and all template-based emission
//! methods for constants, state, device models, switches, pots, RHS, and
//! process_sample. Also includes the shared methods used by the nodal path
//! (emit_header, emit_device_models, emit_oversampler).

use tera::Context;

use crate::codegen::ir::{CircuitIR, DeviceParams, PotentiometerIR};
use crate::codegen::{CodegenError, NoiseMode};
use super::RustEmitter;
use super::helpers::{
    self_heating_device_data, device_param_template_data, fmt_f64, format_matrix_rows,
    inductor_template_data, coupled_inductor_template_data, transformer_group_template_data,
    named_const_entries, recommended_warmup_samples, section_banner, emit_device_const,
    oversampling_info, SwitchTemplateData, SwitchCompTemplateData,
};

impl RustEmitter {
    /// Emit DK-method generated code (original path).
    pub(super) fn emit_dk(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = String::new();
        let noise = self.build_noise_emission(ir);

        code.push_str(&self.emit_header(ir)?);
        code.push_str(&self.emit_constants(ir)?);
        code.push_str(&self.emit_pot_constants(ir));
        if noise.enabled {
            code.push_str(&noise.top_level);
        }
        code.push_str(&self.emit_state(ir, &noise)?);
        code.push_str(&Self::emit_transformer_group_helpers(ir));
        code.push_str(&self.emit_device_models(ir)?);
        // SM pot helpers (sm_scale_N) removed — per-block rebuild replaces SM
        code.push_str(&self.emit_build_rhs(ir, &noise)?);
        code.push_str(&self.emit_mat_vec_mul_s(ir)?);
        code.push_str(&self.emit_extract_voltages(ir)?);
        self.generate_solve_nonlinear(&mut code, ir)?;
        code.push_str(&self.emit_final_voltages(ir)?);
        code.push_str(&self.emit_update_history()?);
        code.push_str(&self.emit_process_sample(ir)?);

        if ir.solver_config.oversampling_factor > 1 {
            code.push_str(&Self::emit_oversampler(ir));
        }

        Ok(code)
    }
}

// ============================================================================
// Template-based emission methods
// ============================================================================

impl RustEmitter {
    pub(super) fn emit_header(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        // Sanitize title: replace newlines and control characters with spaces
        // to prevent template injection through a crafted SPICE netlist title line.
        let sanitized_title: String = ir
            .metadata
            .title
            .chars()
            .map(|c| if c.is_control() { ' ' } else { c })
            .collect();
        ctx.insert("title", &sanitized_title);
        self.render("header", &ctx)
    }

    fn emit_constants(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();

        ctx.insert("n", &n);
        ctx.insert("m", &m);
        // n_nodes: original circuit node count (before augmented VS/VCVS variables).
        // Used to zero out augmented rows in A_neg during rebuild_matrices.
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        ctx.insert("n_nodes", &n_nodes);
        let has_augmented = n_nodes < n;
        ctx.insert("has_augmented", &has_augmented);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("n_aug", &ir.topology.n_aug);

        // When augmented_inductors is true, companion model constants (IND_*_G_EQ,
        // CI_*_G_SELF/MUTUAL, XFMR_*_Y) are not needed. The G/C matrices already
        // contain inductor stamps and A_neg handles history.
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, true));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            ctx.insert("transformer_groups", &transformer_group_template_data(ir));
        }
        ctx.insert(
            "sample_rate",
            &format!("{:.1}", ir.solver_config.sample_rate),
        );
        ctx.insert("oversampling_factor", &ir.solver_config.oversampling_factor);
        if ir.solver_config.oversampling_factor > 1 {
            let internal_rate =
                ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
            ctx.insert("internal_sample_rate", &format!("{:.1}", internal_rate));
        }
        ctx.insert("alpha", &fmt_f64(ir.solver_config.alpha));
        ctx.insert("input_node", &ir.solver_config.input_node);
        let num_outputs = ir.solver_config.output_nodes.len();
        ctx.insert("num_outputs", &num_outputs);
        let output_nodes_values = ir
            .solver_config
            .output_nodes
            .iter()
            .map(|n| n.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        ctx.insert("output_nodes_values", &output_nodes_values);
        let output_scales_values = ir
            .solver_config
            .output_scales
            .iter()
            .map(|s| fmt_f64(*s))
            .collect::<Vec<_>>()
            .join(", ");
        ctx.insert("output_scales_values", &output_scales_values);
        ctx.insert(
            "input_resistance",
            &fmt_f64(ir.solver_config.input_resistance),
        );
        ctx.insert("has_dc_sources", &ir.has_dc_sources);

        // Named topology constants (Oomox P2 + P3). Always inserted so the
        // template can unconditionally reference `named_nodes`, `named_vsources`,
        // `named_pots` — empty lists produce no emission.
        ctx.insert(
            "named_nodes",
            &named_const_entries(&ir.named_constants.nodes),
        );
        ctx.insert(
            "named_vsources",
            &named_const_entries(&ir.named_constants.vsources),
        );
        ctx.insert(
            "named_pots",
            &named_const_entries(&ir.named_constants.pots),
        );

        // Runtime voltage sources (.runtime directive). Always insert the list
        // (possibly empty) so state.rs.tera and build_rhs.rs.tera can use
        // `runtime_sources | length > 0` guards unconditionally.
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // WARMUP_SAMPLES_RECOMMENDED (Oomox P5): 5τ_max at the internal sample
        // rate, rounded up, minimum 1. Plugins driving per-instance parameter
        // jitter (e.g. SeriesOfTubes) use this to size the silent warmup loop.
        ctx.insert("warmup_samples_recommended", &recommended_warmup_samples(ir));

        // G and C matrices (sample-rate independent)
        ctx.insert("g_rows", &format_matrix_rows(n, n, |i, j| ir.g(i, j)));
        ctx.insert("c_rows", &format_matrix_rows(n, n, |i, j| ir.c(i, j)));

        ctx.insert("s_rows", &format_matrix_rows(n, n, |i, j| ir.s(i, j)));
        ctx.insert(
            "a_neg_rows",
            &format_matrix_rows(n, n, |i, j| ir.a_neg(i, j)),
        );

        if ir.has_dc_sources {
            let rhs_const_values = (0..n)
                .map(|i| fmt_f64(ir.matrices.rhs_const[i]))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("rhs_const_values", &rhs_const_values);
        }

        ctx.insert("k_rows", &format_matrix_rows(m, m, |i, j| ir.k(i, j)));
        ctx.insert("n_v_rows", &format_matrix_rows(m, n, |i, j| ir.n_v(i, j)));
        // N_i transposed: N_I[device][node] = n_i[node][device]
        ctx.insert("n_i_rows", &format_matrix_rows(m, n, |i, j| ir.n_i(j, i)));

        // S*N_i product: precomputed for final voltage correction
        // S_NI[node][device] = sum_k S[node][k] * N_i[k][device]
        let s_ni_rows: Vec<String> = (0..n)
            .map(|i| {
                (0..m)
                    .map(|j| {
                        let mut val = 0.0;
                        for k in 0..n {
                            val += ir.s(i, k) * ir.n_i(k, j);
                        }
                        fmt_f64(val)
                    })
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .collect();
        ctx.insert("s_ni_rows", &s_ni_rows);

        // Backward Euler fallback constants (for BE fallback in DK NR solver)
        let has_be_fallback = !ir.matrices.s_be.is_empty() && m > 0;
        ctx.insert("has_be_fallback", &has_be_fallback);
        if has_be_fallback {
            ctx.insert("s_be_rows", &format_matrix_rows(n, n, |i, j| ir.s_be(i, j)));
            ctx.insert("k_be_rows", &format_matrix_rows(m, m, |i, j| ir.k_be(i, j)));
            ctx.insert(
                "a_neg_be_rows",
                &format_matrix_rows(n, n, |i, j| ir.a_neg_be(i, j)),
            );

            // S_NI_be = S_be * N_i (N x M)
            let s_ni_be_rows: Vec<String> = (0..n)
                .map(|i| {
                    (0..m)
                        .map(|j| {
                            let mut val = 0.0;
                            for k in 0..n {
                                val += ir.s_be(i, k) * ir.n_i(k, j);
                            }
                            fmt_f64(val)
                        })
                        .collect::<Vec<_>>()
                        .join(", ")
                })
                .collect();
            ctx.insert("s_ni_be_rows", &s_ni_be_rows);

            // RHS_CONST_BE (backward Euler: DC sources x1)
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
                ctx.insert("rhs_const_be_values", &rhs_const_be_values);
            }
        }

        // Switch constants
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);
        if num_switches > 0 {
            let switch_data: Vec<SwitchTemplateData> = ir
                .switches
                .iter()
                .map(|sw| {
                    let components: Vec<SwitchCompTemplateData> = sw
                        .components
                        .iter()
                        .map(|comp| SwitchCompTemplateData {
                            node_p: comp.node_p,
                            node_q: comp.node_q,
                            nominal: fmt_f64(comp.nominal_value),
                            comp_type: comp.component_type,
                            inductor_index: comp.inductor_index.map(|i| i as i64).unwrap_or(-1),
                        })
                        .collect();
                    let position_rows: Vec<String> = sw
                        .positions
                        .iter()
                        .map(|pos| {
                            pos.iter()
                                .map(|v| fmt_f64(*v))
                                .collect::<Vec<_>>()
                                .join(", ")
                        })
                        .collect();
                    SwitchTemplateData {
                        index: sw.index,
                        num_positions: sw.num_positions,
                        num_components: sw.components.len(),
                        components,
                        position_rows,
                    }
                })
                .collect();
            ctx.insert("switches", &switch_data);
        }

        // DC block coefficient: R = 1 - 2*pi*5/sr
        let internal_rate =
            ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
        let dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / internal_rate;
        ctx.insert("dc_block_r", &format!("{:.17e}", dc_block_r));
        ctx.insert("dc_block", &ir.dc_block);
        ctx.insert("dc_op_converged", &ir.dc_op_converged);

        // Op-amp slew-rate constants. One entry per op-amp whose .model
        // card set a finite SR (V/μs, converted to V/s in MNA). Emitted by
        // constants.rs.tera as `const OA{idx}_SR: f64 = …;`. Consumed by
        // process_sample.rs.tera in the slew-limit block.
        let opamp_slew: Vec<std::collections::HashMap<&str, String>> = ir
            .opamps
            .iter()
            .enumerate()
            .filter(|(_, oa)| oa.sr.is_finite())
            .map(|(idx, oa)| {
                let mut m = std::collections::HashMap::new();
                m.insert("idx", idx.to_string());
                m.insert("out_idx", oa.n_out_idx.to_string());
                m.insert("sr", format!("{:.17e}", oa.sr));
                m
            })
            .collect();
        if !opamp_slew.is_empty() {
            ctx.insert("opamp_slew", &opamp_slew);
        }

        self.render("constants", &ctx)
    }

    fn emit_state(&self, ir: &CircuitIR, noise: &NoiseEmission) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        // Noise fragments (empty strings when noise is off → template blocks become no-ops)
        ctx.insert("noise_enabled_emit", &noise.enabled);
        ctx.insert("noise_state_fields", &noise.state_fields);
        ctx.insert("noise_default_stmts", &noise.default_stmts);
        ctx.insert("noise_default_fields", &noise.default_fields);
        ctx.insert("noise_reset_body", &noise.reset_body);
        ctx.insert("noise_set_sample_rate_body", &noise.set_sample_rate_body);
        ctx.insert("noise_methods", &noise.methods);
        ctx.insert("has_dc_op", &ir.has_dc_op);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("n_aug", &ir.topology.n_aug);
        ctx.insert("n_nodes", &ir.topology.n_nodes);
        // When augmented_inductors is true, companion model state (ind_i_prev, ci_i_hist,
        // xfmr_y, etc.) is not needed. A_neg handles history through augmented G/C.
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        ctx.insert("num_inductors", &num_inductors);
        let num_pots = ir.pots.len();
        ctx.insert("num_pots", &num_pots);
        let num_outputs = ir.solver_config.output_nodes.len();
        ctx.insert("num_outputs", &num_outputs);

        let os_factor = ir.solver_config.oversampling_factor;
        ctx.insert("oversampling_factor", &os_factor);
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            ctx.insert("os_state_size", &os_info.state_size);
            ctx.insert("oversampling_4x", &(os_factor == 4));
            if os_factor == 4 {
                ctx.insert("os_state_size_outer", &os_info.state_size_outer);
            }
        } else {
            ctx.insert("oversampling_4x", &false);
        }

        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, true));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            ctx.insert("transformer_groups", &transformer_group_template_data(ir));

            // Generate set_sample_rate recomputation lines procedurally
            let mut xfmr_ssr_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                xfmr_ssr_lines.push_str(
                    "        {\n\
                     \x20           let half_t = t / 2.0;\n",
                );
                // Build L matrix
                for i in 0..w {
                    for j in 0..w {
                        xfmr_ssr_lines.push_str(&format!(
                            "            let l_{i}_{j} = XFMR_{gi}_COUPLING[{}] * (XFMR_{gi}_INDUCTANCES[{i}] * XFMR_{gi}_INDUCTANCES[{j}]).sqrt();\n",
                            i * w + j,
                        ));
                    }
                }
                // Call inversion helper
                xfmr_ssr_lines.push_str(&format!("            let y = invert_xfmr_{gi}(["));
                for i in 0..w {
                    if i > 0 {
                        xfmr_ssr_lines.push_str(", ");
                    }
                    xfmr_ssr_lines.push('[');
                    for j in 0..w {
                        if j > 0 {
                            xfmr_ssr_lines.push_str(", ");
                        }
                        xfmr_ssr_lines.push_str(&format!("l_{i}_{j}"));
                    }
                    xfmr_ssr_lines.push(']');
                }
                xfmr_ssr_lines.push_str("]);\n");
                // Store Y and stamp
                for i in 0..w {
                    for j in 0..w {
                        xfmr_ssr_lines.push_str(&format!(
                            "            self.xfmr_{gi}_y[{}] = half_t * y[{i}][{j}];\n",
                            i * w + j,
                        ));
                    }
                }
                // Stamp self-conductances
                for i in 0..w {
                    let flat = i * w + i;
                    xfmr_ssr_lines.push_str(&format!(
                        "            stamp_conductance(&mut a, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], self.xfmr_{gi}_y[{flat}]);\n\
                         \x20           stamp_conductance(&mut a_neg, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], -self.xfmr_{gi}_y[{flat}]);\n"
                    ));
                }
                // Stamp mutual conductances
                for i in 0..w {
                    for j in 0..w {
                        if i == j {
                            continue;
                        }
                        let flat = i * w + j;
                        xfmr_ssr_lines.push_str(&format!(
                            "            stamp_mutual(&mut a, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], XFMR_{gi}_NODE_I[{j}], XFMR_{gi}_NODE_J[{j}], self.xfmr_{gi}_y[{flat}]);\n\
                             \x20           stamp_mutual(&mut a_neg, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], XFMR_{gi}_NODE_I[{j}], XFMR_{gi}_NODE_J[{j}], -self.xfmr_{gi}_y[{flat}]);\n"
                        ));
                    }
                }
                xfmr_ssr_lines.push_str("        }\n");
            }
            // Reset transformer group transient state
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                xfmr_ssr_lines.push_str(&format!(
                    "        self.xfmr_{gi}_i_prev = [0.0; {}];\n\
                     \x20       self.xfmr_{gi}_v_prev = [0.0; {}];\n\
                     \x20       self.xfmr_{gi}_i_hist = [0.0; {}];\n",
                    g.num_windings, g.num_windings, g.num_windings,
                ));
            }
            ctx.insert("xfmr_set_sample_rate_lines", &xfmr_ssr_lines);
        }

        let pot_defaults: Vec<String> =
            ir.pots.iter().map(|p| fmt_f64(1.0 / p.g_nominal)).collect();
        ctx.insert("pot_defaults", &pot_defaults);

        // Pot indices for template iteration
        let pot_indices: Vec<usize> = (0..num_pots).collect();
        ctx.insert("pot_indices", &pot_indices);

        if ir.has_dc_op {
            let dc_op_values = ir
                .dc_operating_point
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("dc_op_values", &dc_op_values);
        }

        // DC nonlinear currents: emit DC_NL_I constant if M > 0 and any i_nl is nonzero
        let has_dc_nl = ir.topology.m > 0
            && !ir.dc_nl_currents.is_empty()
            && ir.dc_nl_currents.iter().any(|&v| v.abs() > 1e-30);
        ctx.insert("has_dc_nl", &has_dc_nl);
        if has_dc_nl {
            let dc_nl_i_values = ir
                .dc_nl_currents
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("dc_nl_i_values", &dc_nl_i_values);
        }

        // Switch data
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);
        // Always provide switch_indices (empty when no switches) so template can iterate safely
        let switch_indices: Vec<usize> = (0..num_switches).collect();
        ctx.insert("switch_indices", &switch_indices);
        // Generate pot/switch methods procedurally (rebuild_matrices, set_pot_N, set_switch_N)
        if num_switches > 0 || num_pots > 0 {
            let switch_methods = self.emit_switch_methods(ir, noise);
            ctx.insert("switch_methods", &switch_methods);
        }

        // Device parameter state fields (runtime-adjustable)
        let device_params = device_param_template_data(ir);
        let num_device_params = device_params.len();
        ctx.insert("num_device_params", &num_device_params);
        if num_device_params > 0 {
            ctx.insert("device_params", &device_params);
        }

        // BJT self-heating thermal state
        let thermal_devices = self_heating_device_data(ir);
        let num_thermal_devices = thermal_devices.len();
        ctx.insert("num_thermal_devices", &num_thermal_devices);
        if num_thermal_devices > 0 {
            ctx.insert("thermal_devices", &thermal_devices);
        }

        ctx.insert("dc_block", &ir.dc_block);

        // Backward Euler fallback state fields (for BE fallback in DK NR solver)
        let has_be_fallback = !ir.matrices.s_be.is_empty() && ir.topology.m > 0;
        ctx.insert("has_be_fallback", &has_be_fallback);
        ctx.insert("backward_euler", &ir.solver_config.backward_euler);

        // Runtime voltage sources (.runtime directive). Same list is used by
        // state.rs.tera for field + default + reset emission; build_rhs.rs.tera
        // emits the per-sample `rhs[row] += state.<field>` stamps.
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // Named nodes for the dc_op_dump() pretty printer (Oomox P4).
        ctx.insert(
            "named_nodes",
            &named_const_entries(&ir.named_constants.nodes),
        );

        // Runtime DC operating point recompute (Oomox P6 / Phase E). The
        // template renders a stub body when this flag is on; full device
        // eval + NR loop emission is layered on in subsequent commits.
        let emit_dc_op_recompute = ir.solver_config.emit_dc_op_recompute;
        ctx.insert("emit_dc_op_recompute", &emit_dc_op_recompute);
        if emit_dc_op_recompute {
            let body = super::dc_op_emitter::emit_recompute_dc_op_body_dk(ir)?;
            ctx.insert("recompute_dc_op_body", &body);
            ctx.insert(
                "settle_dc_op_body",
                &super::dc_op_emitter::emit_settle_dc_op_body(),
            );
        }

        self.render("state", &ctx)
    }

    pub(super) fn emit_device_models(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = section_banner("DEVICE MODELS");

        let mut has_diode = false;
        let mut has_bjt = false;
        let mut has_jfet = false;
        let mut has_mosfet = false;
        let mut has_tube = false;
        let mut has_vca = false;
        let mut has_bjt_self_heating = false;

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            match &slot.params {
                DeviceParams::Diode(dp) => {
                    has_diode = true;
                    emit_device_const(&mut code, dev_num, "IS", dp.is);
                    emit_device_const(&mut code, dev_num, "N_VT", dp.n_vt);
                    // Precomputed critical voltage for SPICE pnjlim
                    let vcrit = dp.n_vt * (dp.n_vt / (std::f64::consts::SQRT_2 * dp.is)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    if dp.has_rs() {
                        emit_device_const(&mut code, dev_num, "RS", dp.rs);
                    }
                    if dp.has_bv() {
                        emit_device_const(&mut code, dev_num, "BV", dp.bv);
                        emit_device_const(&mut code, dev_num, "IBV", dp.ibv);
                    }
                    code.push('\n');
                }
                DeviceParams::Bjt(bp) => {
                    has_bjt = true;
                    if bp.has_self_heating() {
                        has_bjt_self_heating = true;
                    }
                    emit_device_const(&mut code, dev_num, "IS", bp.is);
                    emit_device_const(&mut code, dev_num, "VT", bp.vt);
                    emit_device_const(&mut code, dev_num, "BETA_F", bp.beta_f);
                    emit_device_const(&mut code, dev_num, "BETA_R", bp.beta_r);
                    emit_device_const(&mut code, dev_num, "NF", bp.nf);
                    emit_device_const(&mut code, dev_num, "NR", bp.nr);
                    emit_device_const(&mut code, dev_num, "ISE", bp.ise);
                    emit_device_const(&mut code, dev_num, "NE", bp.ne);
                    emit_device_const(&mut code, dev_num, "ISC", bp.isc);
                    emit_device_const(&mut code, dev_num, "NC", bp.nc);
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n",
                        dev_num, sign
                    ));
                    code.push_str(&format!(
                        "const DEVICE_{}_USE_GP: bool = {};\n",
                        dev_num,
                        bp.is_gummel_poon()
                    ));
                    emit_device_const(&mut code, dev_num, "VAF", bp.vaf);
                    emit_device_const(&mut code, dev_num, "VAR", bp.var);
                    emit_device_const(&mut code, dev_num, "IKF", bp.ikf);
                    emit_device_const(&mut code, dev_num, "IKR", bp.ikr);
                    // Precomputed critical voltage for SPICE pnjlim (both Vbe and Vbc junctions)
                    let vcrit = bp.vt * (bp.vt / (std::f64::consts::SQRT_2 * bp.is)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    if bp.has_parasitics() {
                        emit_device_const(&mut code, dev_num, "RB", bp.rb);
                        emit_device_const(&mut code, dev_num, "RC", bp.rc);
                        emit_device_const(&mut code, dev_num, "RE", bp.re);
                    }
                    if bp.has_self_heating() {
                        emit_device_const(&mut code, dev_num, "RTH", bp.rth);
                        emit_device_const(&mut code, dev_num, "CTH", bp.cth);
                        emit_device_const(&mut code, dev_num, "XTI", bp.xti);
                        emit_device_const(&mut code, dev_num, "EG", bp.eg);
                        emit_device_const(&mut code, dev_num, "TAMB", bp.tamb);
                        emit_device_const(&mut code, dev_num, "IS_NOM", bp.is);
                    }
                    code.push('\n');
                }
                DeviceParams::Jfet(jp) => {
                    has_jfet = true;
                    emit_device_const(&mut code, dev_num, "IDSS", jp.idss);
                    emit_device_const(&mut code, dev_num, "VP", jp.vp);
                    emit_device_const(&mut code, dev_num, "LAMBDA", jp.lambda);
                    if jp.has_rd_rs() {
                        emit_device_const(&mut code, dev_num, "RD", jp.rd);
                        emit_device_const(&mut code, dev_num, "RS", jp.rs);
                    }
                    let sign = if jp.is_p_channel { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n\n",
                        dev_num, sign
                    ));
                }
                DeviceParams::Mosfet(mp) => {
                    has_mosfet = true;
                    emit_device_const(&mut code, dev_num, "KP", mp.kp);
                    emit_device_const(&mut code, dev_num, "VT", mp.vt);
                    emit_device_const(&mut code, dev_num, "LAMBDA", mp.lambda);
                    if mp.has_rd_rs() {
                        emit_device_const(&mut code, dev_num, "RD", mp.rd);
                        emit_device_const(&mut code, dev_num, "RS", mp.rs);
                    }
                    if mp.has_body_effect() {
                        emit_device_const(&mut code, dev_num, "GAMMA", mp.gamma);
                        emit_device_const(&mut code, dev_num, "PHI", mp.phi);
                        code.push_str(&format!(
                            "const DEVICE_{}_SOURCE_NODE: usize = {};\n",
                            dev_num, mp.source_node
                        ));
                        code.push_str(&format!(
                            "const DEVICE_{}_BULK_NODE: usize = {};\n",
                            dev_num, mp.bulk_node
                        ));
                    }
                    let sign = if mp.is_p_channel { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n\n",
                        dev_num, sign
                    ));
                }
                DeviceParams::Tube(tp) => {
                    has_tube = true;
                    emit_device_const(&mut code, dev_num, "MU", tp.mu);
                    emit_device_const(&mut code, dev_num, "EX", tp.ex);
                    emit_device_const(&mut code, dev_num, "KG1", tp.kg1);
                    emit_device_const(&mut code, dev_num, "KP", tp.kp);
                    emit_device_const(&mut code, dev_num, "KVB", tp.kvb);
                    emit_device_const(&mut code, dev_num, "IG_MAX", tp.ig_max);
                    emit_device_const(&mut code, dev_num, "VGK_ONSET", tp.vgk_onset);
                    emit_device_const(&mut code, dev_num, "LAMBDA", tp.lambda);
                    if tp.has_rgi() {
                        emit_device_const(&mut code, dev_num, "RGI", tp.rgi);
                    }
                    // Pentode-only constants (Reefman Derk §4.4). Triodes leave
                    // these unset so the emitted output is byte-identical for
                    // pure-triode circuits.
                    if tp.is_pentode() {
                        emit_device_const(&mut code, dev_num, "KG2", tp.kg2);
                        emit_device_const(&mut code, dev_num, "ALPHA_S", tp.alpha_s);
                        emit_device_const(&mut code, dev_num, "A_FACTOR", tp.a_factor);
                        emit_device_const(&mut code, dev_num, "BETA_FACTOR", tp.beta_factor);
                    }
                    // Grid-off reduced pentode constant: the DC-OP-converged
                    // screen voltage Vg2k that the 2D reduced NR block uses in
                    // place of the dropped NR dimension. Read from the
                    // `DeviceSlot`, not `TubeParams` — it's runtime state
                    // captured by the DC-OP grid-off detection pass.
                    if tp.is_grid_off_pentode() {
                        emit_device_const(&mut code, dev_num, "VG2K_FROZEN", slot.vg2k_frozen);
                    }
                    // Variable-mu §5 constants. Emitted only when `svar > 0`
                    // to preserve byte-identity for sharp (phase 1a/1a.1)
                    // circuits. Applies to BOTH triodes and pentodes.
                    if tp.is_variable_mu() {
                        emit_device_const(&mut code, dev_num, "MU_B", tp.mu_b);
                        emit_device_const(&mut code, dev_num, "SVAR", tp.svar);
                        emit_device_const(&mut code, dev_num, "EX_B", tp.ex_b);
                    }
                    // Precomputed critical voltage for SPICE pnjlim (grid current onset)
                    let vt_tube = tp.vgk_onset / 3.0;
                    let vcrit = vt_tube * (vt_tube / (std::f64::consts::SQRT_2 * 1e-10)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    code.push('\n');
                }
                DeviceParams::Vca(vp) => {
                    has_vca = true;
                    emit_device_const(&mut code, dev_num, "VSCALE", vp.vscale);
                    emit_device_const(&mut code, dev_num, "G0", vp.g0);
                    emit_device_const(&mut code, dev_num, "THD", vp.thd);
                    code.push('\n');
                }
            }
        }

        // Boltzmann constant / elementary charge (k/q in eV/K)
        if has_bjt_self_heating {
            code.push_str("/// Boltzmann constant / elementary charge [eV/K]\n");
            code.push_str("const BOLTZMANN_Q: f64 = 8.617333262e-5;\n\n");
        }

        // Fast exp() approximation (needed by all nonlinear device models)
        if has_diode || has_bjt || has_jfet || has_mosfet || has_tube || has_vca {
            code.push_str(&Self::emit_fast_exp());
        }

        // SPICE voltage limiting functions (needed by all nonlinear devices)
        if has_diode || has_bjt || has_jfet || has_mosfet || has_tube || has_vca {
            code.push_str(&self.render("spice_limiting", &Context::new())?);
        }

        if has_diode {
            code.push_str(&self.render("device_diode", &Context::new())?);
        }
        if has_bjt {
            code.push_str(&self.render("device_bjt", &Context::new())?);
        }
        if has_jfet {
            code.push_str(&self.render("device_jfet", &Context::new())?);
        }
        if has_mosfet {
            code.push_str(&self.render("device_mosfet", &Context::new())?);
        }
        if has_tube {
            // Five Tera guards for pentode-family tubes:
            //   any_pentode            — sharp Rational (Derk §4.4): EL84/EL34/EF86
            //   any_beam_tetrode       — sharp Exponential (DerkE §4.5): 6L6GC/6V6GT
            //   any_variable_mu_pentode       — §5 two-section Rational: 6K7
            //   any_variable_mu_beam_tetrode  — §5 two-section Exponential: EF89
            //   any_classical_pentode  — Classical Norman Koren (Cohen-Hélie §2): KT88/6550
            //
            // Byte-identity guarantee: a circuit containing only non-pentode tubes
            // (pure-triode) has all five false and emits the phase-1a triode block
            // unchanged. A circuit containing only sharp pentodes (svar=0) has
            // `any_variable_mu_*` and `any_classical_pentode` false and emits
            // phase-1a.1 output byte-identical. A pure-Derk (Rational/Exponential)
            // circuit has `any_classical_pentode` false and emits phase-1c output
            // byte-identical. Variable-mu Classical is rejected by
            // `TubeParams::validate()`, so no `any_variable_mu_classical_pentode`
            // flag exists.
            // Grid-off pentode guard: excluded from the sharp flags below so
            // the existing sharp / variable-mu / Classical helper families
            // stay byte-identical for circuits with zero grid-off slots.
            // `any_grid_off_pentode` gates a fresh block of thin 2D wrapper
            // helpers that delegate back to the sharp 3D helpers with
            // `vg2k_frozen` substituted for the live Vg2k dimension.
            let any_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Rational)
                        && !tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_beam_tetrode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Exponential)
                        && !tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_variable_mu_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Rational)
                        && tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_variable_mu_beam_tetrode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Exponential)
                        && tp.is_variable_mu()
                        && !tp.is_grid_off_pentode())
            });
            let any_classical_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Classical)
                        && !tp.is_grid_off_pentode())
            });
            let any_grid_off_pentode = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp) if tp.is_grid_off_pentode())
            });
            // Grid-off wrapper helpers delegate to the sharp 3D helpers of
            // the matching screen form, so whichever sharp family a grid-off
            // slot uses MUST have its own helper emitted too. Force the
            // matching sharp flag on whenever a grid-off slot exists of that
            // screen form. This keeps the template simple (a single
            // `{% if any_grid_off_pentode %}` block can reference both the
            // wrapper and the 3D helper it delegates to) without leaking
            // sharp helpers into circuits that have neither.
            let any_grid_off_rational = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_grid_off_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Rational))
            });
            let any_grid_off_exponential = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_grid_off_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Exponential))
            });
            let any_grid_off_classical = ir.device_slots.iter().any(|slot| {
                matches!(&slot.params, DeviceParams::Tube(tp)
                    if tp.is_grid_off_pentode()
                        && matches!(tp.screen_form, crate::device_types::ScreenForm::Classical))
            });
            let any_pentode = any_pentode || any_grid_off_rational;
            let any_beam_tetrode = any_beam_tetrode || any_grid_off_exponential;
            let any_classical_pentode = any_classical_pentode || any_grid_off_classical;
            let mut tube_ctx = Context::new();
            tube_ctx.insert("any_pentode", &any_pentode);
            tube_ctx.insert("any_beam_tetrode", &any_beam_tetrode);
            tube_ctx.insert("any_variable_mu_pentode", &any_variable_mu_pentode);
            tube_ctx.insert(
                "any_variable_mu_beam_tetrode",
                &any_variable_mu_beam_tetrode,
            );
            tube_ctx.insert("any_classical_pentode", &any_classical_pentode);
            tube_ctx.insert("any_grid_off_pentode", &any_grid_off_pentode);
            tube_ctx.insert("any_grid_off_rational", &any_grid_off_rational);
            tube_ctx.insert("any_grid_off_exponential", &any_grid_off_exponential);
            tube_ctx.insert("any_grid_off_classical", &any_grid_off_classical);
            code.push_str(&self.render("device_tube", &tube_ctx)?);
        }
        if has_vca {
            code.push_str(&self.render("device_vca", &Context::new())?);
        }

        Ok(code)
    }

    /// Emit fast exp() approximation function.
    ///
    /// Default: polynomial range reduction + 5th-order minimax (~6 cycles, <0.0004% error).
    /// Opt-in: `--cfg melange_precise_exp` uses hardware/libm exp (~38 cycles).
    ///
    /// Accuracy of polynomial path: <0.0004% max relative error over [-40, 40].
    pub(super) fn emit_fast_exp() -> String {
        let mut code = String::new();
        code.push_str(
            "/// Fast exp() for audio circuit simulation.\n\
             /// Input clamped to [-40, 40] (matches melange safe_exp convention).\n\
             ///\n\
             /// Default: polynomial approximation (<0.0004% error, ~6x faster than libm).\n\
             /// To use hardware/libm exp, compile with: `--cfg melange_precise_exp`\n\
             #[inline(always)]\n\
             fn fast_exp(x: f64) -> f64 {\n\
             \x20   #[cfg(melange_precise_exp)]\n\
             \x20   { x.clamp(-40.0, 40.0).exp() }\n\
             \x20   #[cfg(not(melange_precise_exp))]\n\
             \x20   {\n\
             \x20       // Range reduction + 5th-order minimax polynomial. <0.0004% max relative error.\n\
             \x20       // No lookup tables, no libm dependency, branchless hot path.\n\
             \x20       let x = x.clamp(-40.0, 40.0);\n\
             \x20       const LN2_INV: f64 = std::f64::consts::LOG2_E;\n\
             \x20       const LN2_HI: f64 = 0.6931471803691238;\n\
             \x20       const LN2_LO: f64 = 1.9082149292705877e-10;\n\
             \x20       const SHIFT: f64 = 6755399441055744.0; // 2^52 + 2^51\n\
             \x20       let z = x * LN2_INV + SHIFT;\n\
             \x20       let n_i64 = z.to_bits() as i64 - SHIFT.to_bits() as i64;\n\
             \x20       let n = n_i64 as f64;\n\
             \x20       let f = (x - n * LN2_HI) - n * LN2_LO;\n\
             \x20       let p = 1.0 + f * (1.0 + f * (0.5 + f * (0.16666666666666607\n\
             \x20           + f * (0.04166666666665876 + f * 0.008333333333492337))));\n\
             \x20       let pow2n = f64::from_bits(((1023 + n_i64) as u64) << 52);\n\
             \x20       p * pow2n\n\
             \x20   }\n\
             }\n\n",
        );
        // fast_ln: used in tube softplus ln(1+exp(x))
        code.push_str(
            "/// Fast ln() for audio circuit simulation.\n\
             /// Symmetric log series (~0.005% max relative error, ~3x faster than libm).\n\
             /// Only valid for positive inputs.\n\
             #[inline(always)]\n\
             fn fast_ln(x: f64) -> f64 {\n\
             \x20   #[cfg(melange_precise_exp)]\n\
             \x20   { x.ln() }\n\
             \x20   #[cfg(not(melange_precise_exp))]\n\
             \x20   {\n\
             \x20       // Extract exponent and mantissa from IEEE 754 double\n\
             \x20       let bits = x.to_bits();\n\
             \x20       let e = ((bits >> 52) & 0x7FF) as i64 - 1023;\n\
             \x20       // Normalize mantissa to [1, 2)\n\
             \x20       let m_bits = (bits & 0x000F_FFFF_FFFF_FFFF) | 0x3FF0_0000_0000_0000;\n\
             \x20       let m = f64::from_bits(m_bits);\n\
             \x20       // Symmetric log series: u = (m-1)/(m+1), ln(m) = 2u(1 + u²/3 + u⁴/5 + u⁶/7)\n\
             \x20       // For m in [1,2), u in [0,1/3): converges much faster than Taylor.\n\
             \x20       let u = (m - 1.0) / (m + 1.0);\n\
             \x20       let u2 = u * u;\n\
             \x20       let ln_m = 2.0 * u * (1.0 + u2 * (0.3333333333333333 + u2 * (0.2 + u2 * 0.14285714285714285)));\n\
             \x20       ln_m + (e as f64) * std::f64::consts::LN_2\n\
             \x20   }\n\
             }\n\n",
        );
        code
    }

    /// Generate switch setter methods and rebuild_matrices() procedurally.
    fn emit_switch_methods(&self, ir: &CircuitIR, noise: &NoiseEmission) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let num_pots = ir.pots.len();
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        let mut code = String::new();

        // Emit set_switch_N() for each switch (DK path)
        for sw in &ir.switches {
            // Authentic-noise coefficient refresh for any R-type components
            // in this switch that back a thermal noise source. Emitted only
            // when noise is compiled in AND this switch has at least one
            // noise-tracked R. DK uses the 2D const array
            // `SWITCH_<N>_VALUES[position][comp_idx]`, distinct from
            // the nodal path's per-component arrays.
            let noise_update: String = if noise.enabled {
                let idx = sw.index;
                noise
                    .switch_comp_to_noise_slot
                    .get(idx)
                    .map(|slots| {
                        let mut out = String::new();
                        for (ci, maybe_slot) in slots.iter().enumerate() {
                            if let Some(slot) = maybe_slot {
                                out.push_str(&format!(
                                    "        self.noise_thermal_sqrt_inv_r[{slot}] = (1.0 / SWITCH_{idx}_VALUES[position][{ci}]).sqrt();\n",
                                ));
                            }
                        }
                        out
                    })
                    .unwrap_or_default()
            } else {
                String::new()
            };
            code.push_str(&format!(
                "    /// Set switch {} position (0..{}).\n\
                 \x20   ///\n\
                 \x20   /// Marks matrices dirty. Rebuild deferred to next `process_sample()`.\n\
                 \x20   /// A switch flip is a topology step — follow with `recompute_dc_op()`\n\
                 \x20   /// to refresh the NR seed and avoid audible glitches on the next sample.\n\
                 \x20   pub fn set_switch_{}(&mut self, position: usize) {{\n\
                 \x20       if position >= SWITCH_{}_NUM_POSITIONS {{ return; }}\n\
                 \x20       if self.switch_{}_position == position {{ return; }}\n\
                 \x20       self.switch_{}_position = position;\n\
                 \x20       self.matrices_dirty = true;\n\
{noise_update}\
                 \x20   }}\n\n",
                sw.index,
                sw.num_positions - 1,
                sw.index, sw.index, sw.index, sw.index,
            ));
        }

        // Emit set_pot_N() / set_runtime_R_<field>() methods for DK path.
        //
        // Both setters have identical bodies: clamp → skip-if-unchanged →
        // stamp the dirty flag → refresh noise coefficient. No NR-state
        // reseed — callers that need one (preset recall, raw unsmoothed
        // jumps) should follow the setter with `recompute_dc_op()`.
        // Plugin-template callers feed `.smoothed.next()` values; nih-plug
        // smoothing keeps per-sample deltas tiny, so NR stays within basin
        // on the previous sample's seed.
        for (idx, pot) in ir.pots.iter().enumerate() {
            // Authentic-noise coefficient refresh — emitted only when
            // noise is enabled at codegen AND this pot backs a thermal
            // source. Keeps `state.noise_thermal_sqrt_inv_r[k]` in sync
            // with the live R so Johnson-Nyquist variance tracks the knob.
            let noise_update: String = if noise.enabled {
                match noise.pot_to_noise_slot.get(idx).copied().flatten() {
                    Some(slot) => format!(
                        "        self.noise_thermal_sqrt_inv_r[{slot}] = (1.0 / r).sqrt();\n",
                    ),
                    None => String::new(),
                }
            } else {
                String::new()
            };

            match &pot.runtime_field {
                None => {
                    code.push_str(&format!(
                        "    /// Set potentiometer {idx} resistance (clamped to [{:.1}..{:.1}] ohms).\n\
                         \x20   ///\n\
                         \x20   /// Marks matrices dirty. Rebuild deferred to next `process_sample()`.\n\
                         \x20   /// For preset recall / unsmoothed jumps, follow with `recompute_dc_op()`.\n\
                         \x20   pub fn set_pot_{idx}(&mut self, resistance: f64) {{\n\
                         \x20       if !resistance.is_finite() {{ return; }}\n\
                         \x20       let r = resistance.clamp(POT_{idx}_MIN_R, POT_{idx}_MAX_R);\n\
                         \x20       if (r - self.pot_{idx}_resistance).abs() < 1e-12 {{ return; }}\n\
                         \x20       self.pot_{idx}_resistance = r;\n\
                         \x20       self.matrices_dirty = true;\n\
{noise_update}\
                         \x20   }}\n\n",
                        pot.min_resistance, pot.max_resistance,
                    ));
                }
                Some(field) => {
                    let field_upper = field.to_ascii_uppercase();
                    code.push_str(&format!(
                        "    /// Current resistance of runtime resistor `{field}` (ohms).\n\
                         \x20   ///\n\
                         \x20   /// Read-only accessor; use `set_runtime_R_{field}` to update.\n\
                         \x20   #[inline]\n\
                         \x20   pub fn {field}(&self) -> f64 {{ self.pot_{idx}_resistance }}\n\n\
                         \x20   /// Set runtime resistor `{field}` (clamped to [{:.1}..{:.1}] ohms).\n\
                         \x20   ///\n\
                         \x20   /// Audio-rate safe: no internal smoothing; caller (plugin-side\n\
                         \x20   /// envelope follower) is the smoother.\n\
                         \x20   /// Marks matrices dirty. Rebuild deferred to next `process_sample()`.\n\
                         \x20   pub fn set_runtime_R_{field}(&mut self, resistance: f64) {{\n\
                         \x20       if !resistance.is_finite() {{ return; }}\n\
                         \x20       let r = resistance.clamp(RUNTIME_R_{field_upper}_MIN, RUNTIME_R_{field_upper}_MAX);\n\
                         \x20       if (r - self.pot_{idx}_resistance).abs() < 1e-12 {{ return; }}\n\
                         \x20       self.pot_{idx}_resistance = r;\n\
                         \x20       self.matrices_dirty = true;\n\
{noise_update}\
                         \x20   }}\n\n",
                        pot.min_resistance, pot.max_resistance,
                    ));
                }
            }
        }

        // Emit rebuild_matrices()
        code.push_str(
            "    /// Rebuild all sample-rate-dependent matrices from G/C constants.\n\
             \x20   ///\n\
             \x20   /// Applies switch/pot deltas to G/C, then rebuilds A, S, K, S*N_i.\n\
             \x20   /// Called by `set_switch_N()`, `set_pot_N()`, and `set_sample_rate()`.\n\
             \x20   fn rebuild_matrices(&mut self) {\n\
             \x20       let internal_rate = self.current_sample_rate * OVERSAMPLING_FACTOR as f64;\n",
        );
        if ir.solver_config.backward_euler {
            code.push_str(
                "        let alpha = internal_rate; // backward Euler: alpha = 1/T\n",
            );
        } else {
            code.push_str(
                "        let alpha = 2.0 * internal_rate; // trapezoidal: alpha = 2/T\n",
            );
        }
        if num_inductors > 0 {
            code.push_str("        let t = 1.0 / internal_rate;\n");
        }

        // Start from constant G, C
        let has_r_switch = ir
            .switches
            .iter()
            .any(|sw| sw.components.iter().any(|c| c.component_type == 'R'));
        let has_c_switch = ir
            .switches
            .iter()
            .any(|sw| sw.components.iter().any(|c| c.component_type == 'C'));
        let g_mut = if has_r_switch || num_pots > 0 { "mut " } else { "" };
        let c_mut = if has_c_switch { "mut " } else { "" };
        code.push_str(&format!(
            "\n\
             \x20       // Start from constant G and C matrices\n\
             \x20       let {}g_eff = G;\n\
             \x20       let {}c_eff = C;\n",
            g_mut, c_mut,
        ));

        // Apply switch deltas
        code.push_str(
            "\n\
             \x20       // Apply switch position deltas\n",
        );
        for sw in &ir.switches {
            for (ci, comp) in sw.components.iter().enumerate() {
                let nominal = fmt_f64(comp.nominal_value);
                code.push_str(&format!(
                    "        {{\n\
                     \x20           let new_val = SWITCH_{}_VALUES[self.switch_{}_position][{}];\n",
                    sw.index, sw.index, ci,
                ));
                match comp.component_type {
                    'R' => {
                        code.push_str(&format!(
                            "            let delta_g = 1.0 / new_val - 1.0 / {};\n\
                             \x20           stamp_conductance(&mut g_eff, SWITCH_{}_COMP_{}_NODE_P, SWITCH_{}_COMP_{}_NODE_Q, delta_g);\n",
                            nominal, sw.index, ci, sw.index, ci,
                        ));
                    }
                    'C' => {
                        code.push_str(&format!(
                            "            let delta_c = new_val - {};\n\
                             \x20           stamp_conductance(&mut c_eff, SWITCH_{}_COMP_{}_NODE_P, SWITCH_{}_COMP_{}_NODE_Q, delta_c);\n",
                            nominal, sw.index, ci, sw.index, ci,
                        ));
                    }
                    'L' => {
                        if let Some(aug_row) = comp.augmented_row {
                            // Augmented MNA: L value lives on diagonal of branch variable row
                            code.push_str(&format!(
                                "            let delta_l = new_val - {};\n\
                                 \x20           c_eff[{}][{}] += delta_l;\n",
                                nominal, aug_row, aug_row,
                            ));
                        } else {
                            // DK companion model: handled in inductor companion stamp below
                            code.push_str(
                                "            // Inductor: handled in companion model stamp below\n",
                            );
                            code.push_str("            let _ = new_val;\n");
                        }
                    }
                    _ => {}
                }
                code.push_str("        }\n");
            }
        }

        // Apply pot conductance deltas (relative to nominal)
        if num_pots > 0 {
            code.push_str(
                "\n\
                 \x20       // Apply pot conductance deltas (current resistance vs nominal)\n",
            );
            for (idx, pot) in ir.pots.iter().enumerate() {
                let np = pot.node_p;
                let nq = pot.node_q;
                code.push_str(&format!(
                    "        {{\n\
                     \x20           let delta_g = 1.0 / self.pot_{idx}_resistance - POT_{idx}_G_NOM;\n",
                ));
                if np > 0 {
                    code.push_str(&format!(
                        "            g_eff[{}][{}] += delta_g;\n", np - 1, np - 1
                    ));
                }
                if nq > 0 {
                    code.push_str(&format!(
                        "            g_eff[{}][{}] += delta_g;\n", nq - 1, nq - 1
                    ));
                }
                if np > 0 && nq > 0 {
                    code.push_str(&format!(
                        "            g_eff[{}][{}] -= delta_g;\n\
                         \x20           g_eff[{}][{}] -= delta_g;\n",
                        np - 1, nq - 1, nq - 1, np - 1
                    ));
                }
                code.push_str("        }\n");
            }
        }

        // Build A = g_eff + alpha * c_eff
        code.push_str(
            "\n\
             \x20       // Build A = G_eff + alpha * C_eff\n\
             \x20       let mut a = [[0.0f64; N]; N];\n\
             \x20       for i in 0..N {\n\
             \x20           for j in 0..N {\n\
             \x20               a[i][j] = g_eff[i][j] + alpha * c_eff[i][j];\n\
             \x20           }\n\
             \x20       }\n\n",
        );
        // A_neg formula: trapezoidal = alpha*C - G; backward Euler = alpha*C (no G term)
        let a_neg_formula = if ir.solver_config.backward_euler {
            "alpha * c_eff[i][j]"
        } else {
            "alpha * c_eff[i][j] - g_eff[i][j]"
        };
        code.push_str(&format!(
            "        // Build A_neg: {}\n\
             \x20       let mut a_neg = [[0.0f64; N]; N];\n\
             \x20       for i in 0..N {{\n\
             \x20           for j in 0..N {{\n\
             \x20               a_neg[i][j] = {};\n\
             \x20           }}\n\
             \x20       }}\n",
            if ir.solver_config.backward_euler { "alpha*C (backward Euler, no G)" } else { "alpha*C - G (trapezoidal)" },
            a_neg_formula
        ));
        // Zero augmented rows in A_neg (algebraic constraints for VS/VCVS)
        // When augmented_inductors, only zero n_nodes..n_aug (not inductor rows)
        let n_aug = ir.topology.n_aug;
        let a_neg_zero_end = if ir.topology.augmented_inductors {
            n_aug
        } else {
            n
        };
        if n_nodes < a_neg_zero_end {
            code.push_str(&format!(
                "        // Zero VS/VCVS algebraic rows in A_neg (NOT inductor rows)\n\
                 \x20       for i in {n_nodes}..{a_neg_zero_end} {{\n\
                 \x20           for j in 0..N {{\n\
                 \x20               a_neg[i][j] = 0.0;\n\
                 \x20           }}\n\
                 \x20       }}\n"
            ));
        }

        // Inductor companion stamps (with switch-aware inductance)
        if num_inductors > 0 {
            code.push_str("\n        // Add inductor companion model conductances\n");
            for (li, ind) in ir.inductors.iter().enumerate() {
                // Check if any switch controls this inductor
                let mut switched = false;
                for sw in &ir.switches {
                    for (ci, comp) in sw.components.iter().enumerate() {
                        if comp.component_type == 'L' && comp.inductor_index == Some(li) {
                            code.push_str(&format!(
                                "        {{\n\
                                 \x20           let inductance = SWITCH_{}_VALUES[self.switch_{}_position][{}];\n\
                                 \x20           let g_eq = t / (2.0 * inductance);\n\
                                 \x20           self.ind_g_eq[{}] = g_eq;\n\
                                 \x20           stamp_conductance(&mut a, {}, {}, g_eq);\n\
                                 \x20           stamp_conductance(&mut a_neg, {}, {}, -g_eq);\n\
                                 \x20       }}\n",
                                sw.index, sw.index, ci,
                                li,
                                ind.node_i, ind.node_j,
                                ind.node_i, ind.node_j,
                            ));
                            switched = true;
                            break;
                        }
                    }
                    if switched {
                        break;
                    }
                }
                if !switched {
                    // Non-switched inductor: use constant
                    code.push_str(&format!(
                        "        {{\n\
                         \x20           let g_eq = t / (2.0 * IND_{}_INDUCTANCE);\n\
                         \x20           self.ind_g_eq[{}] = g_eq;\n\
                         \x20           stamp_conductance(&mut a, IND_{}_NODE_I, IND_{}_NODE_J, g_eq);\n\
                         \x20           stamp_conductance(&mut a_neg, IND_{}_NODE_I, IND_{}_NODE_J, -g_eq);\n\
                         \x20       }}\n",
                        li, li, li, li, li, li,
                    ));
                }
            }
            code.push_str(&format!(
                "        self.ind_i_prev = [0.0; {}];\n\
                 \x20       self.ind_v_prev = [0.0; {}];\n\
                 \x20       self.ind_i_hist = [0.0; {}];\n",
                num_inductors, num_inductors, num_inductors,
            ));
        }

        // Coupled inductor companion stamps
        let num_coupled = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        if num_coupled > 0 {
            if num_inductors == 0 {
                code.push_str("        let t = 1.0 / internal_rate;\n");
            }
            code.push_str("\n        // Add coupled inductor companion model conductances\n");
            for (ci_idx, ci) in ir.coupled_inductors.iter().enumerate() {
                // Check if either winding is switch-controlled
                let mut l1_switch: Option<(usize, usize)> = None; // (sw_index, comp_index)
                let mut l2_switch: Option<(usize, usize)> = None;
                for sw in &ir.switches {
                    for (comp_i, comp) in sw.components.iter().enumerate() {
                        if comp.coupled_inductor_index == Some(ci_idx) {
                            match comp.coupled_winding {
                                Some(1) => l1_switch = Some((sw.index, comp_i)),
                                Some(2) => l2_switch = Some((sw.index, comp_i)),
                                _ => {}
                            }
                        }
                    }
                }

                let l1_expr = if let Some((sw_idx, comp_idx)) = l1_switch {
                    format!("SWITCH_{}_COMP_{}_VALUES[self.switch_{}_position]", sw_idx, comp_idx, sw_idx)
                } else {
                    format!("CI_{}_L1_INDUCTANCE", ci_idx)
                };
                let l2_expr = if let Some((sw_idx, comp_idx)) = l2_switch {
                    format!("SWITCH_{}_COMP_{}_VALUES[self.switch_{}_position]", sw_idx, comp_idx, sw_idx)
                } else {
                    format!("CI_{}_L2_INDUCTANCE", ci_idx)
                };

                code.push_str(&format!(
                    "        {{\n\
                     \x20           let l1_val = {l1};\n\
                     \x20           let l2_val = {l2};\n\
                     \x20           let m_val = CI_{ci}_COUPLING * (l1_val * l2_val).sqrt();\n\
                     \x20           let det = l1_val * l2_val - m_val * m_val;\n\
                     \x20           let half_t = t / 2.0;\n\
                     \x20           let gs1 = half_t * l2_val / det;\n\
                     \x20           let gs2 = half_t * l1_val / det;\n\
                     \x20           let gm = -half_t * m_val / det;\n\
                     \x20           self.ci_g_self_1[{ci}] = gs1;\n\
                     \x20           self.ci_g_self_2[{ci}] = gs2;\n\
                     \x20           self.ci_g_mutual[{ci}] = gm;\n\
                     \x20           stamp_conductance(&mut a, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, gs1);\n\
                     \x20           stamp_conductance(&mut a_neg, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, -gs1);\n\
                     \x20           stamp_conductance(&mut a, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, gs2);\n\
                     \x20           stamp_conductance(&mut a_neg, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, -gs2);\n\
                     \x20           stamp_mutual(&mut a, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, gm);\n\
                     \x20           stamp_mutual(&mut a, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, gm);\n\
                     \x20           stamp_mutual(&mut a_neg, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, -gm);\n\
                     \x20           stamp_mutual(&mut a_neg, CI_{ci}_L2_NODE_I, CI_{ci}_L2_NODE_J, CI_{ci}_L1_NODE_I, CI_{ci}_L1_NODE_J, -gm);\n\
                     \x20       }}\n",
                    l1 = l1_expr,
                    l2 = l2_expr,
                    ci = ci_idx,
                ));
                let _ = ci; // suppress unused warning
            }
            code.push_str(&format!(
                "        self.ci_i1_prev = [0.0; {n}];\n\
                 \x20       self.ci_i2_prev = [0.0; {n}];\n\
                 \x20       self.ci_v1_prev = [0.0; {n}];\n\
                 \x20       self.ci_v2_prev = [0.0; {n}];\n\
                 \x20       self.ci_i1_hist = [0.0; {n}];\n\
                 \x20       self.ci_i2_hist = [0.0; {n}];\n",
                n = num_coupled,
            ));
        }

        // Transformer group companion stamps
        let num_xfmr_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        if num_xfmr_groups > 0 {
            if num_inductors == 0 && num_coupled == 0 {
                code.push_str("        let t = 1.0 / internal_rate;\n");
            }
            code.push_str("\n        // Add transformer group companion model conductances\n");
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                // Build L matrix from inductances and couplings, invert, multiply by T/2
                code.push_str(
                    "        {\n\
                     \x20           let half_t = t / 2.0;\n\
                     \x20           // Build inductance matrix L[i][j] = k[i][j] * sqrt(Li*Lj)\n",
                );
                // Emit L matrix construction
                for i in 0..w {
                    for j in 0..w {
                        code.push_str(&format!(
                            "            let l_{i}_{j} = XFMR_{gi}_COUPLING[{flat}] * (XFMR_{gi}_INDUCTANCES[{i}] * XFMR_{gi}_INDUCTANCES[{j}]).sqrt();\n",
                            flat = i * w + j,
                        ));
                    }
                }
                // Inline Gauss elimination to invert W x W matrix
                code.push_str(&format!("            let y = invert_xfmr_{gi}(["));
                for i in 0..w {
                    if i > 0 {
                        code.push_str(", ");
                    }
                    code.push('[');
                    for j in 0..w {
                        if j > 0 {
                            code.push_str(", ");
                        }
                        code.push_str(&format!("l_{i}_{j}"));
                    }
                    code.push(']');
                }
                code.push_str("]);\n");
                // Store Y = half_t * inv(L) and stamp
                for i in 0..w {
                    for j in 0..w {
                        code.push_str(&format!(
                            "            self.xfmr_{gi}_y[{flat}] = half_t * y[{i}][{j}];\n",
                            flat = i * w + j,
                        ));
                    }
                }
                // Stamp self-conductances (diagonal)
                for i in 0..w {
                    let flat = i * w + i;
                    code.push_str(&format!(
                        "            stamp_conductance(&mut a, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], self.xfmr_{gi}_y[{flat}]);\n\
                         \x20           stamp_conductance(&mut a_neg, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], -self.xfmr_{gi}_y[{flat}]);\n",
                    ));
                }
                // Stamp mutual conductances (off-diagonal)
                for i in 0..w {
                    for j in 0..w {
                        if i == j {
                            continue;
                        }
                        let flat = i * w + j;
                        code.push_str(&format!(
                            "            stamp_mutual(&mut a, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], XFMR_{gi}_NODE_I[{j}], XFMR_{gi}_NODE_J[{j}], self.xfmr_{gi}_y[{flat}]);\n",
                        ));
                        code.push_str(&format!(
                            "            stamp_mutual(&mut a_neg, XFMR_{gi}_NODE_I[{i}], XFMR_{gi}_NODE_J[{i}], XFMR_{gi}_NODE_I[{j}], XFMR_{gi}_NODE_J[{j}], -self.xfmr_{gi}_y[{flat}]);\n",
                        ));
                    }
                }
                code.push_str("        }\n");
            }
            // Reset transformer group transient state
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                code.push_str(&format!(
                    "        self.xfmr_{gi}_i_prev = [0.0; {w}];\n\
                     \x20       self.xfmr_{gi}_v_prev = [0.0; {w}];\n\
                     \x20       self.xfmr_{gi}_i_hist = [0.0; {w}];\n",
                ));
            }
        }

        // Invert A → S, compute S_NI, K
        code.push_str(&format!(
            "\n\
             \x20       // Invert A to get S\n\
             \x20       let (s, singular) = invert_n(a);\n\
             \x20       if singular {{ self.diag_singular_matrix_count += 1; }}\n\n\
             \x20       // Compute S * N_i product (N x M)\n\
             \x20       let mut s_ni = [[0.0f64; M]; N];\n\
             \x20       for i in 0..N {{\n\
             \x20           for j in 0..M {{\n\
             \x20               let mut sum = 0.0;\n\
             \x20               for kk in 0..N {{\n\
             \x20                   sum += s[i][kk] * N_I[j][kk];\n\
             \x20               }}\n\
             \x20               s_ni[i][j] = sum;\n\
             \x20           }}\n\
             \x20       }}\n\n\
             \x20       // Compute K = N_v * S_NI (M x M)\n\
             \x20       let mut k = [[0.0f64; {m}]; {m}];\n\
             \x20       for i in 0..M {{\n\
             \x20           for j in 0..M {{\n\
             \x20               let mut sum = 0.0;\n\
             \x20               for n_idx in 0..N {{\n\
             \x20                   sum += N_V[i][n_idx] * s_ni[n_idx][j];\n\
             \x20               }}\n\
             \x20               k[i][j] = sum;\n\
             \x20           }}\n\
             \x20       }}\n\n\
             \x20       self.s = s;\n\
             \x20       self.a_neg = a_neg;\n\
             \x20       self.k = k;\n\
             \x20       self.s_ni = s_ni;\n",
            m = m,
        ));

        // SM pot recomputation removed — per-block rebuild handles pots exactly

        // Intentionally do NOT reset oversampler state or DC blocker state here.
        // rebuild_matrices() is called on every pot/switch change, but pot/switch
        // changes do not invalidate filter history. Zeroing os_up_state/os_dn_state
        // on every knob move causes an audible click from the half-band filter
        // ringing through; zeroing dc_block_x_prev/y_prev produces an instant DC
        // step and multi-second HPF re-settle. The `dc_block_r` coefficient depends
        // only on the sample rate (handled by set_sample_rate), not on pots/switches,
        // so it does not need recomputing here either. (Legitimate filter resets still
        // happen in reset() and set_sample_rate().)

        code.push_str("    }\n");
        code
    }

    fn emit_pot_constants(&self, ir: &CircuitIR) -> String {
        if ir.pots.is_empty() {
            return String::new();
        }
        let mut code = section_banner("POTENTIOMETER CONSTANTS");

        for (idx, pot) in ir.pots.iter().enumerate() {
            // G_NOM: nominal conductance for rebuild_matrices delta computation
            code.push_str(&format!(
                "const POT_{}_G_NOM: f64 = {};\n",
                idx,
                fmt_f64(pot.g_nominal)
            ));
            code.push_str(&format!(
                "const POT_{}_MIN_R: f64 = {};\n",
                idx,
                fmt_f64(pot.min_resistance)
            ));
            code.push_str(&format!(
                "const POT_{}_MAX_R: f64 = {};\n",
                idx,
                fmt_f64(pot.max_resistance)
            ));
            // For .runtime R entries, also emit discoverable public aliases
            // keyed on the runtime field name so plugin code can read the
            // clamp range without knowing the pot index.
            if let Some(field) = &pot.runtime_field {
                let u = field.to_ascii_uppercase();
                code.push_str(&format!(
                    "pub const RUNTIME_R_{u}_MIN: f64 = POT_{}_MIN_R;\n",
                    idx
                ));
                code.push_str(&format!(
                    "pub const RUNTIME_R_{u}_MAX: f64 = POT_{}_MAX_R;\n",
                    idx
                ));
                code.push_str(&format!(
                    "pub const RUNTIME_R_{u}_NOMINAL: f64 = 1.0 / POT_{}_G_NOM;\n",
                    idx
                ));
            }
            code.push('\n');
        }
        code
    }

    /// Emit inline Gaussian elimination inversion functions for transformer groups.
    ///
    /// Each transformer group of size W gets its own `invert_xfmr_{n}` function
    /// that inverts a W x W matrix. The size is known at codegen time so the
    /// function is fully unrolled.
    fn emit_transformer_group_helpers(ir: &CircuitIR) -> String {
        if ir.transformer_groups.is_empty() {
            return String::new();
        }
        let mut code = section_banner("TRANSFORMER GROUP INVERSION HELPERS");

        for (gi, g) in ir.transformer_groups.iter().enumerate() {
            let w = g.num_windings;
            code.push_str(&format!(
                "/// Invert a {w}x{w} matrix for transformer group {gi} using Gaussian elimination.\n\
                 #[inline(always)]\n\
                 fn invert_xfmr_{gi}(a: [[f64; {w}]; {w}]) -> [[f64; {w}]; {w}] {{\n\
                 \x20   let mut aug = [[0.0f64; {w2}]; {w}];\n\
                 \x20   for i in 0..{w} {{\n\
                 \x20       for j in 0..{w} {{\n\
                 \x20           aug[i][j] = a[i][j];\n\
                 \x20       }}\n\
                 \x20       aug[i][{w} + i] = 1.0;\n\
                 \x20   }}\n\n",
                w2 = w * 2,
            ));
            // Forward elimination with partial pivoting
            code.push_str(&format!(
                "    for col in 0..{w} {{\n\
                 \x20       let mut max_row = col;\n\
                 \x20       let mut max_val = aug[col][col].abs();\n\
                 \x20       for row in (col + 1)..{w} {{\n\
                 \x20           if aug[row][col].abs() > max_val {{\n\
                 \x20               max_val = aug[row][col].abs();\n\
                 \x20               max_row = row;\n\
                 \x20           }}\n\
                 \x20       }}\n\
                 \x20       if max_val < 1e-30 {{\n\
                 \x20           let mut result = [[0.0f64; {w}]; {w}];\n\
                 \x20           for i in 0..{w} {{ result[i][i] = 1.0; }}\n\
                 \x20           return result;\n\
                 \x20       }}\n\
                 \x20       if max_row != col {{ aug.swap(col, max_row); }}\n\
                 \x20       let pivot = aug[col][col];\n\
                 \x20       for row in (col + 1)..{w} {{\n\
                 \x20           let factor = aug[row][col] / pivot;\n\
                 \x20           for j in col..{w2} {{\n\
                 \x20               aug[row][j] -= factor * aug[col][j];\n\
                 \x20           }}\n\
                 \x20       }}\n\
                 \x20   }}\n\n",
                w2 = w * 2,
            ));
            // Back-substitution
            code.push_str(&format!(
                "    for col in (0..{w}).rev() {{\n\
                 \x20       let pivot = aug[col][col];\n\
                 \x20       if pivot.abs() < 1e-30 {{\n\
                 \x20           let mut result = [[0.0f64; {w}]; {w}];\n\
                 \x20           for i in 0..{w} {{ result[i][i] = 1.0; }}\n\
                 \x20           return result;\n\
                 \x20       }}\n\
                 \x20       for j in 0..{w2} {{ aug[col][j] /= pivot; }}\n\
                 \x20       for row in 0..col {{\n\
                 \x20           let factor = aug[row][col];\n\
                 \x20           for j in 0..{w2} {{ aug[row][j] -= factor * aug[col][j]; }}\n\
                 \x20       }}\n\
                 \x20   }}\n\n",
                w2 = w * 2,
            ));
            // Extract result
            code.push_str(&format!(
                "    let mut result = [[0.0f64; {w}]; {w}];\n\
                 \x20   for i in 0..{w} {{\n\
                 \x20       for j in 0..{w} {{\n\
                 \x20           result[i][j] = aug[i][{w} + j];\n\
                 \x20       }}\n\
                 \x20   }}\n\
                 \x20   result\n\
                 }}\n\n",
            ));
        }
        code
    }

    fn emit_build_rhs(&self, ir: &CircuitIR, _noise: &NoiseEmission) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();

        ctx.insert("has_dc_sources", &ir.has_dc_sources);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("backward_euler", &ir.solver_config.backward_euler);
        // Runtime voltage sources: emit `rhs[row] += state.<field>` per entry
        // after the input stamp. Safe on both trapezoidal and BE paths — the
        // field is the raw per-sample value, not rate-dependent.
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // When augmented_inductors is true, companion model history is handled by A_neg,
        // so num_inductors/num_coupled_inductors/num_transformer_groups should be 0
        // for the build_rhs template (no history current injection).
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, false));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            let mut xfmr_rhs_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                for k in 0..g.num_windings {
                    if g.winding_node_i[k] > 0 {
                        xfmr_rhs_lines.push_str(&format!(
                            "    rhs[{}] -= state.xfmr_{}_i_hist[{}];\n",
                            g.winding_node_i[k] - 1,
                            gi,
                            k
                        ));
                    }
                    if g.winding_node_j[k] > 0 {
                        xfmr_rhs_lines.push_str(&format!(
                            "    rhs[{}] += state.xfmr_{}_i_hist[{}];\n",
                            g.winding_node_j[k] - 1,
                            gi,
                            k
                        ));
                    }
                }
            }
            ctx.insert("xfmr_rhs_lines", &xfmr_rhs_lines);
        }

        // A_neg * v_prev lines (using pre-analyzed sparsity)
        let assign_op = if ir.has_dc_sources { "+=" } else { "=" };
        let mut a_neg_lines = String::new();
        for i in 0..n {
            let nz_cols = &ir.sparsity.a_neg.nz_by_row[i];
            if nz_cols.is_empty() {
                if !ir.has_dc_sources {
                    a_neg_lines.push_str(&format!("    rhs[{}] = 0.0;\n", i));
                }
            } else {
                let terms: Vec<String> = nz_cols
                    .iter()
                    .map(|&j| format!("state.a_neg[{}][{}] * state.v_prev[{}]", i, j, j))
                    .collect();
                a_neg_lines.push_str(&format!(
                    "    rhs[{}] {} {};\n",
                    i,
                    assign_op,
                    terms.join(" + ")
                ));
            }
        }
        ctx.insert("a_neg_lines", &a_neg_lines);

        // N_i * i_nl_prev lines (using pre-analyzed sparsity)
        let has_nl_prev = m > 0;
        ctx.insert("has_nl_prev", &has_nl_prev);
        if has_nl_prev {
            let mut nl_prev_lines = String::new();
            for i in 0..n {
                for &j in &ir.sparsity.n_i.nz_by_row[i] {
                    nl_prev_lines.push_str(&format!(
                        "    rhs[{}] += N_I[{}][{}] * state.i_nl_prev[{}];\n",
                        i, j, i, j
                    ));
                }
            }
            ctx.insert("nl_prev_lines", &nl_prev_lines);
        }

        self.render("build_rhs", &ctx)
    }

    fn emit_mat_vec_mul_s(&self, _ir: &CircuitIR) -> Result<String, CodegenError> {
        // The template now uses a runtime loop over state.s, no context needed
        self.render("mat_vec_mul_s", &Context::new())
    }

    fn emit_extract_voltages(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let m = ir.topology.m;
        let mut ctx = Context::new();

        // N_v extraction lines (using pre-analyzed sparsity)
        let mut extract_lines = String::new();
        for i in 0..m {
            extract_lines.push_str("        ");
            let nz_cols = &ir.sparsity.n_v.nz_by_row[i];
            if nz_cols.is_empty() {
                extract_lines.push_str("0.0");
            } else {
                let mut first = true;
                for &j in nz_cols {
                    let coeff = ir.n_v(i, j);
                    let abs_val = coeff.abs();
                    let is_negative = coeff < 0.0;

                    if first {
                        if is_negative {
                            extract_lines.push('-');
                        }
                    } else if is_negative {
                        extract_lines.push_str(" - ");
                    } else {
                        extract_lines.push_str(" + ");
                    }

                    if (abs_val - 1.0).abs() < 1e-15 {
                        extract_lines.push_str(&format!("v_pred[{}]", j));
                    } else {
                        extract_lines.push_str(&format!("{} * v_pred[{}]", fmt_f64(abs_val), j));
                    }
                    first = false;
                }
            }
            extract_lines.push_str(",\n");
        }
        ctx.insert("extract_lines", &extract_lines);

        self.render("extract_voltages", &ctx)
    }

    fn emit_final_voltages(&self, _ir: &CircuitIR) -> Result<String, CodegenError> {
        // The template now uses a runtime loop over state.s_ni, no context needed
        self.render("final_voltages", &Context::new())
    }

    fn emit_update_history(&self) -> Result<String, CodegenError> {
        self.render("update_history", &Context::new())
    }

    fn emit_process_sample(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        // Rebuild noise fragment — caller (emit_dk) already built one, but
        // re-deriving is free and keeps this method's signature stable.
        let noise = self.build_noise_emission(ir);
        let mut ctx = Context::new();
        ctx.insert("noise_enabled_emit", &noise.enabled);
        ctx.insert("noise_rhs_stamp", &noise.rhs_stamp);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            ir.topology.n
        };
        ctx.insert("n_nodes", &n_nodes);
        // BE fallback in process_sample: only for circuits NOT using auto-BE (which
        // already uses BE for ALL samples). Avoids changing output of well-conditioned
        // circuits where the fallback would trigger on legitimate transient overshoots.
        let has_be_fallback =
            !ir.matrices.s_be.is_empty() && ir.topology.m > 0 && !ir.solver_config.backward_euler;
        ctx.insert("has_be_fallback", &has_be_fallback);
        ctx.insert("has_dc_sources", &ir.has_dc_sources);
        ctx.insert("max_iter", &ir.solver_config.max_iterations);
        // V_MAX_DC: maximum physically reasonable node voltage (supply rails + margin).
        // Used by BE fallback to detect trapezoidal ringing artifacts.
        let v_max_dc = ir
            .dc_operating_point
            .iter()
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max)
            .max(1.0)
            * 3.0
            + 10.0; // 3× margin + 10V headroom (generous for transients)
        ctx.insert("v_max_dc", &format!("{:.17e}", v_max_dc));
        // When augmented_inductors is true, companion model state update is not needed —
        // A_neg handles all inductor history through the augmented G/C matrices.
        let num_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.inductors.len()
        };
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, false));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors {
            0
        } else {
            ir.coupled_inductors.len()
        };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors {
            0
        } else {
            ir.transformer_groups.len()
        };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            // Generate transformer group state update code procedurally
            let mut xfmr_update_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                xfmr_update_lines.push_str("    {\n");
                // Extract winding voltages
                for k in 0..w {
                    let v_i = if g.winding_node_i[k] > 0 {
                        format!("v[{}]", g.winding_node_i[k] - 1)
                    } else {
                        "0.0".to_string()
                    };
                    let v_j = if g.winding_node_j[k] > 0 {
                        format!("v[{}]", g.winding_node_j[k] - 1)
                    } else {
                        "0.0".to_string()
                    };
                    xfmr_update_lines
                        .push_str(&format!("        let v_new_{k} = {v_i} - {v_j};\n"));
                }
                // Compute new currents: i_new[k] = i_prev[k] + sum_j Y[k][j] * (v_prev[j] + v_new[j])
                for k in 0..w {
                    xfmr_update_lines.push_str(&format!(
                        "        let i_new_{k} = state.xfmr_{gi}_i_prev[{k}]"
                    ));
                    for j in 0..w {
                        xfmr_update_lines.push_str(&format!(
                            " + state.xfmr_{gi}_y[{}] * (state.xfmr_{gi}_v_prev[{j}] + v_new_{j})",
                            k * w + j,
                        ));
                    }
                    xfmr_update_lines.push_str(";\n");
                }
                // Compute history currents: i_hist[k] = i_new[k] - sum_j Y[k][j] * v_new[j]
                for k in 0..w {
                    xfmr_update_lines
                        .push_str(&format!("        state.xfmr_{gi}_i_hist[{k}] = i_new_{k}"));
                    for j in 0..w {
                        xfmr_update_lines
                            .push_str(&format!(" - state.xfmr_{gi}_y[{}] * v_new_{j}", k * w + j,));
                    }
                    xfmr_update_lines.push_str(";\n");
                }
                // Update i_prev and v_prev
                for k in 0..w {
                    xfmr_update_lines.push_str(&format!(
                        "        state.xfmr_{gi}_i_prev[{k}] = i_new_{k};\n\
                         \x20       state.xfmr_{gi}_v_prev[{k}] = v_new_{k};\n"
                    ));
                }
                xfmr_update_lines.push_str("    }\n");
            }
            ctx.insert("xfmr_update_lines", &xfmr_update_lines);

            // Generate NaN reset lines
            let mut xfmr_nan_reset_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                xfmr_nan_reset_lines.push_str(&format!(
                    "        state.xfmr_{gi}_i_prev = [0.0; {}];\n\
                     \x20       state.xfmr_{gi}_v_prev = [0.0; {}];\n\
                     \x20       state.xfmr_{gi}_i_hist = [0.0; {}];\n",
                    g.num_windings, g.num_windings, g.num_windings,
                ));
            }
            ctx.insert("xfmr_nan_reset_lines", &xfmr_nan_reset_lines);
        }

        let os_factor = ir.solver_config.oversampling_factor;
        ctx.insert("oversampling_factor", &os_factor);
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            ctx.insert("os_state_size", &os_info.state_size);
            ctx.insert("oversampling_4x", &(os_factor == 4));
            if os_factor == 4 {
                ctx.insert("os_state_size_outer", &os_info.state_size_outer);
            }
        } else {
            ctx.insert("oversampling_4x", &false);
        }

        // DC nonlinear currents availability (for NaN reset path)
        let has_dc_nl = ir.topology.m > 0
            && !ir.dc_nl_currents.is_empty()
            && ir.dc_nl_currents.iter().any(|&v| v.abs() > 1e-30);
        ctx.insert("has_dc_nl", &has_dc_nl);

        let num_outputs = ir.solver_config.output_nodes.len();
        ctx.insert("num_outputs", &num_outputs);

        ctx.insert("max_iter", &ir.solver_config.max_iterations);
        ctx.insert("m", &ir.topology.m);

        let num_pots = ir.pots.len();
        ctx.insert("num_pots", &num_pots);
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);
        let pot_defaults: Vec<String> =
            ir.pots.iter().map(|p| fmt_f64(1.0 / p.g_nominal)).collect();
        ctx.insert("pot_defaults", &pot_defaults);

        // Generate pot correction code blocks procedurally
        if num_pots > 0 {
            let n = ir.topology.n;
            let m = ir.topology.m;

            // Sequential SM setup with cross-corrections for multi-pot stability
            let sm_scale_lines = Self::emit_sequential_sm_setup(&ir.pots, n, m);
            ctx.insert("sm_scale_lines", &sm_scale_lines);

            // A_neg correction: modify rhs for pot conductance change on v_prev
            let mut a_neg_correction = String::new();
            for (idx, pot) in ir.pots.iter().enumerate() {
                Self::emit_a_neg_correction(&mut a_neg_correction, idx, pot);
            }
            ctx.insert("a_neg_correction", &a_neg_correction);

            // S correction: apply SM to v_pred after mat_vec_mul_s
            let mut s_correction = String::new();
            for (idx, pot) in ir.pots.iter().enumerate() {
                Self::emit_s_correction(&mut s_correction, idx, pot, n);
            }
            ctx.insert("s_correction", &s_correction);

            // S*N_i correction: after compute_final_voltages (only if M > 0)
            let mut sni_correction = String::new();
            if m > 0 {
                for (idx, pot) in ir.pots.iter().enumerate() {
                    Self::emit_sni_correction(&mut sni_correction, idx, pot, n, m);
                }
            }
            ctx.insert("sni_correction", &sni_correction);

            // SM k_eff removed — per-block rebuild keeps state.k exact
            ctx.insert("use_k_eff", &false);
        }

        // MOSFET body effect: compute VT_eff from v_pred before NR (DK path only)
        let mut body_effect_update = String::new();
        if ir.solver_mode == crate::codegen::ir::SolverMode::Dk {
            for (dev_num, slot) in ir.device_slots.iter().enumerate() {
                if let DeviceParams::Mosfet(mp) = &slot.params {
                    if mp.has_body_effect() {
                        // Extract Vsb from v_pred (node indices are 1-based; 0 = ground)
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
                        body_effect_update.push_str(&format!(
                                "    {{ // MOSFET {dev_num} body effect\n\
                                 \x20       let vsb = ({sign:.1}) * ({vs_expr} - {vb_expr});\n\
                                 \x20       state.device_{dev_num}_vt = DEVICE_{dev_num}_VT + DEVICE_{dev_num}_GAMMA * ((DEVICE_{dev_num}_PHI + vsb.max(0.0)).sqrt() - DEVICE_{dev_num}_PHI.sqrt());\n\
                                 \x20   }}\n"
                            ));
                    }
                }
            }
        }
        if !body_effect_update.is_empty() {
            ctx.insert("body_effect_update", &body_effect_update);
        }

        // BJT self-heating thermal update (after NR, before state save)
        let mut thermal_update = String::new();
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.has_self_heating() {
                    let s = slot.start_idx;
                    let s1 = s + 1;
                    // Extract Ic, Ib from converged i_nl; compute Vbe, Vbc from final v
                    thermal_update.push_str(&format!(
                            "    {{ // BJT {dev_num} self-heating thermal update\n\
                             \x20       let ic = i_nl[{s}];\n\
                             \x20       let ib = i_nl[{s1}];\n\
                             \x20       let v_nl_th = extract_controlling_voltages(&v);\n\
                             \x20       let vbe = v_nl_th[{s}];\n\
                             \x20       let vbc = v_nl_th[{s1}];\n\
                             \x20       let vce = vbe - vbc;\n\
                             \x20       let p = vce * ic + vbe * ib;\n\
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
        if !thermal_update.is_empty() {
            ctx.insert("thermal_update", &thermal_update);
            let thermal_devices = self_heating_device_data(ir);
            ctx.insert("num_thermal_devices", &thermal_devices.len());
            ctx.insert("thermal_devices", &thermal_devices);
        } else {
            ctx.insert("num_thermal_devices", &0usize);
        }

        ctx.insert("dc_block", &ir.dc_block);

        // Op-amp supply rail clamping data for DK template. Skip op-amps
        // whose VCC/VEE are both infinite — those entries exist in
        // `ir.opamps` only because `sr` is finite (slew limiting is handled
        // via a separate `opamp_slew` context block below).
        if !ir.opamps.is_empty() {
            let opamp_clamps: Vec<std::collections::HashMap<&str, String>> = ir
                .opamps
                .iter()
                .filter(|oa| oa.vclamp_lo.is_finite() || oa.vclamp_hi.is_finite())
                .map(|oa| {
                    let mut m = std::collections::HashMap::new();
                    m.insert("out_idx", oa.n_out_idx.to_string());
                    m.insert("lo", format!("{:.17e}", oa.vclamp_lo));
                    m.insert("hi", format!("{:.17e}", oa.vclamp_hi));
                    m.insert("has_internal", "".to_string());
                    m.insert("int_idx", "0".to_string());
                    m
                })
                .collect();
            if !opamp_clamps.is_empty() {
                ctx.insert("opamp_clamps", &opamp_clamps);
            }
        }

        // Op-amp slew-rate limiting data for DK template. Entries are
        // emitted as a per-sample voltage-delta clamp on the output node,
        // applied before the state update. Only op-amps with finite `sr`
        // contribute; when all op-amps have infinite SR the Tera template
        // emits no slew code at all, keeping generated output byte-
        // identical to the pre-slew behaviour.
        if !ir.opamps.is_empty() {
            let opamp_slew: Vec<std::collections::HashMap<&str, String>> = ir
                .opamps
                .iter()
                .enumerate()
                .filter(|(_, oa)| oa.sr.is_finite())
                .map(|(idx, oa)| {
                    let mut m = std::collections::HashMap::new();
                    m.insert("idx", idx.to_string());
                    m.insert("out_idx", oa.n_out_idx.to_string());
                    m.insert("sr", format!("{:.17e}", oa.sr));
                    m
                })
                .collect();
            if !opamp_slew.is_empty() {
                ctx.insert("opamp_slew", &opamp_slew);
            }
        }

        self.render("process_sample", &ctx)
    }

    /// Emit oversampling wrapper: constants, allpass helper, halfband, and process_sample.
    pub(super) fn emit_oversampler(ir: &CircuitIR) -> String {
        let factor = ir.solver_config.oversampling_factor;
        let info = oversampling_info(factor);
        let mut code = section_banner("OVERSAMPLING");

        // Emit coefficients as constants
        code.push_str("/// Half-band filter coefficients for allpass polyphase oversampler.\n");
        let coeffs_str = info
            .coeffs
            .iter()
            .map(|c| fmt_f64(*c))
            .collect::<Vec<_>>()
            .join(", ");
        code.push_str(&format!(
            "const OS_COEFFS: [f64; {}] = [{}];\n\n",
            info.num_sections, coeffs_str
        ));

        if factor == 4 {
            let coeffs_outer_str = info
                .coeffs_outer
                .iter()
                .map(|c| fmt_f64(*c))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "const OS_COEFFS_OUTER: [f64; {}] = [{}];\n\n",
                info.num_sections_outer, coeffs_outer_str
            ));
        }

        // Emit allpass inline function (takes slice + offset to avoid double &mut borrow)
        code.push_str(
            "/// First-order allpass section: y = c*(x - y1) + x1\n\
             /// State layout: state[base] = x1, state[base+1] = y1\n\
             #[inline(always)]\n\
             fn os_allpass(x: f64, c: f64, state: &mut [f64], base: usize) -> f64 {\n\
             \x20   let y = c * x + state[base] - c * state[base + 1];\n\
             \x20   state[base] = x;\n\
             \x20   state[base + 1] = y;\n\
             \x20   y\n\
             }\n\n",
        );

        // Emit halfband_process inline function
        Self::emit_halfband_fn(&mut code, "os_halfband", &info.coeffs, info.state_size);
        if factor == 4 {
            Self::emit_halfband_fn(
                &mut code,
                "os_halfband_outer",
                &info.coeffs_outer,
                info.state_size_outer,
            );
        }

        let num_outputs = ir.solver_config.output_nodes.len();

        // Emit the public process_sample wrapper
        code.push_str("/// Process a single audio sample through the circuit with oversampling.\n");
        code.push_str("///\n");
        code.push_str(&format!(
            "/// Runs the circuit at {}x the host sample rate to reduce aliasing.\n",
            factor
        ));
        code.push_str("#[inline]\n");
        code.push_str(
            "pub fn process_sample(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n",
        );
        code.push_str(
            "    let input = if input.is_finite() { input.clamp(-100.0, 100.0) } else { 0.0 };\n\n",
        );

        if factor == 2 {
            Self::emit_2x_wrapper(&mut code, num_outputs, ir.dc_block);
        } else if factor == 4 {
            Self::emit_4x_wrapper(&mut code, num_outputs, ir.dc_block);
        }

        code.push_str("}\n\n");
        code
    }

    /// Emit a halfband filter function that processes input through even/odd allpass chains.
    pub(super) fn emit_halfband_fn(code: &mut String, name: &str, coeffs: &[f64], state_size: usize) {
        let num_sections = coeffs.len();
        let even_count = num_sections.div_ceil(2);
        let odd_count = num_sections / 2;

        code.push_str(&format!(
            "/// Half-band filter: processes input through even/odd allpass chains.\n\
             #[inline(always)]\n\
             fn {name}(input: f64, coeffs: &[f64], state: &mut [f64; {state_size}]) -> (f64, f64) {{\n"
        ));

        // Even chain: coefficients at indices 0, 2, 4, ...
        // State layout: even sections first, then odd sections
        code.push_str("    let mut even = input;\n");
        for i in 0..even_count {
            let coeff_idx = i * 2; // even-indexed coefficients
            let state_base = i * 2; // sequential state storage for even chain
            code.push_str(&format!(
                "    even = os_allpass(even, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        // Odd chain: coefficients at indices 1, 3, 5, ...
        code.push_str("    let mut odd = input;\n");
        let odd_state_offset = even_count * 2;
        for i in 0..odd_count {
            let coeff_idx = i * 2 + 1; // odd-indexed coefficients
            let state_base = odd_state_offset + i * 2;
            code.push_str(&format!(
                "    odd = os_allpass(odd, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        code.push_str("    (even, odd)\n");
        code.push_str("}\n\n");
    }

    /// Emit the 2x oversampling wrapper body.
    pub(super) fn emit_2x_wrapper(code: &mut String, num_outputs: usize, dc_block: bool) {
        // Upsample: halfband → 2 samples (single input)
        code.push_str(
            "    // Upsample: half-band filter produces 2 samples at internal rate\n\
             \x20   let (up_even, up_odd) = os_halfband(input, &OS_COEFFS, &mut state.os_up_state);\n\n",
        );

        // Process both at 2x rate — returns [f64; NUM_OUTPUTS]
        code.push_str(
            "    // Process both samples at 2x rate\n\
             \x20   let out_even = process_sample_inner(up_even, state);\n\
             \x20   let out_odd = process_sample_inner(up_odd, state);\n\n",
        );

        // Downsample per-output
        code.push_str("    // Downsample: per-output half-band filter combines 2 samples into 1\n");
        code.push_str("    let mut result = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let (dn1_even, _) = os_halfband(out_even[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("        let (_, dn2_odd) = os_halfband(out_odd[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("        let v = (dn1_even + dn2_odd) * 0.5;\n");
        if dc_block {
            code.push_str(
                "        result[out_idx] = if v.is_finite() { v.clamp(-10.0, 10.0) } else { 0.0 };\n",
            );
        } else {
            code.push_str("        result[out_idx] = if v.is_finite() { v } else { 0.0 };\n");
        }
        code.push_str("    }\n");
        code.push_str("    result\n");
        let _ = num_outputs; // used for signature type
    }

    /// Emit the 4x oversampling wrapper body (cascaded 2x stages).
    pub(super) fn emit_4x_wrapper(code: &mut String, num_outputs: usize, dc_block: bool) {
        // Outer upsample: 1 → 2 at 2x rate (single input)
        code.push_str(
            "    // Outer upsample: 1 → 2 samples at 2x rate\n\
             \x20   let (outer_even, outer_odd) = os_halfband_outer(\n\
             \x20       input, &OS_COEFFS_OUTER, &mut state.os_up_state_outer,\n\
             \x20   );\n\n",
        );

        // Inner upsample + process for each outer sample — returns [f64; NUM_OUTPUTS]
        code.push_str(
            "    // Inner upsample + process: each 2x sample → 2 samples at 4x rate\n\
             \x20   let (inner_e0, inner_o0) = os_halfband(outer_even, &OS_COEFFS, &mut state.os_up_state);\n\
             \x20   let proc_e0 = process_sample_inner(inner_e0, state);\n\
             \x20   let proc_o0 = process_sample_inner(inner_o0, state);\n\n",
        );

        // Inner downsample per-output for first 2x pair
        code.push_str("    let mut inner_out0 = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let (dn_e0, _) = os_halfband(proc_e0[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("        let (_, dn_o0) = os_halfband(proc_o0[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("        inner_out0[out_idx] = (dn_e0 + dn_o0) * 0.5;\n");
        code.push_str("    }\n\n");

        // Second inner upsample + process pair
        code.push_str(
            "    let (inner_e1, inner_o1) = os_halfband(outer_odd, &OS_COEFFS, &mut state.os_up_state);\n\
             \x20   let proc_e1 = process_sample_inner(inner_e1, state);\n\
             \x20   let proc_o1 = process_sample_inner(inner_o1, state);\n\n",
        );

        // Inner downsample per-output for second 2x pair
        code.push_str("    let mut inner_out1 = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let (dn_e1, _) = os_halfband(proc_e1[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("        let (_, dn_o1) = os_halfband(proc_o1[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("        inner_out1[out_idx] = (dn_e1 + dn_o1) * 0.5;\n");
        code.push_str("    }\n\n");

        // Outer downsample per-output
        code.push_str("    // Outer downsample: per-output 2 → 1 sample at host rate\n");
        code.push_str("    let mut result = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let (dn_outer_e, _) = os_halfband_outer(\n");
        code.push_str("            inner_out0[out_idx], &OS_COEFFS_OUTER, &mut state.os_dn_state_outer[out_idx],\n");
        code.push_str("        );\n");
        code.push_str("        let (_, dn_outer_o) = os_halfband_outer(\n");
        code.push_str("            inner_out1[out_idx], &OS_COEFFS_OUTER, &mut state.os_dn_state_outer[out_idx],\n");
        code.push_str("        );\n");
        code.push_str("        let v = (dn_outer_e + dn_outer_o) * 0.5;\n");
        if dc_block {
            code.push_str(
                "        result[out_idx] = if v.is_finite() { v.clamp(-10.0, 10.0) } else { 0.0 };\n",
            );
        } else {
            code.push_str("        result[out_idx] = if v.is_finite() { v } else { 0.0 };\n");
        }
        code.push_str("    }\n");
        code.push_str("    result\n");
        let _ = num_outputs; // used for signature type
    }

    fn emit_a_neg_correction(code: &mut String, idx: usize, pot: &PotentiometerIR) {
        // A_neg correction: the backward term uses the PREVIOUS timestep's conductance.
        // Trapezoidal discretization of time-varying G(t):
        //   Forward: A[n] = G[n] + (2/T)*C → uses current delta_g (in SM correction)
        //   Backward: A_neg[n-1] = (2/T)*C - G[n-1] → uses previous delta_g
        // Using current delta_g here causes artifacts when pot changes between samples
        // (e.g., tremolo LDR modulation at 5.63 Hz).
        code.push_str(&format!(
            "    let delta_g_{idx}_prev = {{\n\
             \x20       let r = state.pot_{idx}_resistance_prev.clamp(POT_{idx}_MIN_R, POT_{idx}_MAX_R);\n\
             \x20       1.0 / r - POT_{idx}_G_NOM\n\
             \x20   }};\n"
        ));
        if pot.node_p > 0 && pot.node_q > 0 {
            let p = pot.node_p - 1;
            let q = pot.node_q - 1;
            code.push_str(&format!(
                "    let v_diff_{} = state.v_prev[{}] - state.v_prev[{}];\n",
                idx, p, q
            ));
            code.push_str(&format!(
                "    rhs[{}] -= delta_g_{}_prev * v_diff_{};\n",
                p, idx, idx
            ));
            code.push_str(&format!(
                "    rhs[{}] += delta_g_{}_prev * v_diff_{};\n",
                q, idx, idx
            ));
        } else if pot.node_p > 0 {
            let p = pot.node_p - 1;
            code.push_str(&format!(
                "    rhs[{}] -= delta_g_{}_prev * state.v_prev[{}];\n",
                p, idx, p
            ));
        } else if pot.node_q > 0 {
            let q = pot.node_q - 1;
            code.push_str(&format!(
                "    rhs[{}] -= delta_g_{}_prev * state.v_prev[{}];\n",
                q, idx, q
            ));
        }
    }

    fn emit_s_correction(code: &mut String, idx: usize, _pot: &PotentiometerIR, n: usize) {
        // S correction using sequentially corrected local su_c{idx} and scale_c{idx}.
        code.push_str(&format!("    let mut su_dot_rhs_{} = 0.0;\n", idx));
        code.push_str(&format!(
            "    for _k in 0..N {{ su_dot_rhs_{idx} += su_c{idx}[_k] * rhs[_k]; }}\n"
        ));

        code.push_str(&format!(
            "    let factor_{} = scale_c{} * su_dot_rhs_{};\n",
            idx, idx, idx
        ));
        for k in 0..n {
            code.push_str(&format!(
                "    v_pred[{}] -= factor_{} * su_c{}[{}];\n",
                k, idx, idx, k
            ));
        }
    }

    fn emit_sni_correction(
        code: &mut String,
        idx: usize,
        _pot: &PotentiometerIR,
        n: usize,
        _m: usize,
    ) {
        // S*N_i correction using sequentially corrected local vectors.
        code.push_str(&format!("    let mut u_ni_dot_inl_{} = 0.0;\n", idx));
        code.push_str(&format!(
            "    for _j in 0..M {{ u_ni_dot_inl_{idx} += u_ni_c{idx}[_j] * i_nl[_j]; }}\n"
        ));
        code.push_str(&format!(
            "    let sni_factor_{} = scale_c{} * u_ni_dot_inl_{};\n",
            idx, idx, idx
        ));
        for k in 0..n {
            code.push_str(&format!(
                "    v[{}] -= sni_factor_{} * su_c{}[{}];\n",
                k, idx, idx, k
            ));
        }
    }

    /// Generate sequential SM setup code with cross-corrections between pots.
    ///
    /// For multiple pots, independent SM rank-1 updates are incorrect because
    /// each update changes S, which affects subsequent updates. This method
    /// generates code that applies corrections sequentially: pot k's SU vector
    /// is corrected for the cumulative effect of pots 0..k-1.
    ///
    /// Math: su_ck = S_corrected * u_k = su_k - Σ_{j<k} scale_cj * su_cj * (su_cj^T * u_k)
    fn emit_sequential_sm_setup(pots: &[PotentiometerIR], _n: usize, m: usize) -> String {
        let mut code = String::new();
        code.push_str("    // Sequential Sherman-Morrison setup with cross-corrections\n");

        for k in 0..pots.len() {
            let pot_k = &pots[k];

            // Delta G computation (independent per pot)
            code.push_str(&format!(
                "    let delta_g_{k} = {{\n\
                 \x20       let r = state.pot_{k}_resistance.clamp(POT_{k}_MIN_R, POT_{k}_MAX_R);\n\
                 \x20       1.0 / r - POT_{k}_G_NOM\n\
                 \x20   }};\n"
            ));

            if k == 0 {
                // First pot: no cross-corrections needed
                code.push_str(&format!("    let su_c{k} = state.pot_{k}_su;\n"));
            } else {
                // Compute cross-correction dot products: su_cj^T * u_k
                for j in 0..k {
                    let dot_expr = Self::emit_dot_su_u(j, pot_k);
                    code.push_str(&format!("    let dot_{j}_{k} = {dot_expr};\n"));
                }

                // Corrected SU vector: su_ck = su_k - Σ_{j<k} scale_cj * su_cj * dot_j_k
                code.push_str(&format!("    let mut su_c{k} = state.pot_{k}_su;\n"));
                code.push_str("    for _n in 0..N {\n");
                for j in 0..k {
                    code.push_str(&format!(
                        "        su_c{k}[_n] -= scale_c{j} * su_c{j}[_n] * dot_{j}_{k};\n"
                    ));
                }
                code.push_str("    }\n");
            }

            // Corrected USU: u_k^T * su_ck (extract from corrected su_ck at pot's node indices)
            let usu_expr = Self::emit_usu_from_su(k, pot_k);
            code.push_str(&format!("    let usu_c{k} = {usu_expr};\n"));

            // Scale factor
            code.push_str(&format!(
                "    let scale_c{k} = if (1.0 + delta_g_{k} * usu_c{k}).abs() > 1e-15 {{\n\
                 \x20       delta_g_{k} / (1.0 + delta_g_{k} * usu_c{k})\n\
                 \x20   }} else {{ 0.0 }};\n"
            ));

            // If M > 0, compute corrected nv_su and u_ni for K_eff
            if m > 0 {
                if k == 0 {
                    code.push_str(&format!("    let nv_su_c{k} = state.pot_{k}_nv_su;\n"));
                    code.push_str(&format!("    let u_ni_c{k} = state.pot_{k}_u_ni;\n"));
                } else {
                    // Corrected NV_SU: nv_su_ck = nv_su_k - Σ_{j<k} scale_cj * nv_su_cj * dot_j_k
                    code.push_str(&format!("    let mut nv_su_c{k} = state.pot_{k}_nv_su;\n"));
                    for j in 0..k {
                        code.push_str(&format!(
                            "    for _i in 0..M {{ nv_su_c{k}[_i] -= scale_c{j} * nv_su_c{j}[_i] * dot_{j}_{k}; }}\n"
                        ));
                    }
                    // Corrected U_NI: u_ni_ck = u_ni_k - Σ_{j<k} scale_cj * u_ni_cj * dot_j_k
                    code.push_str(&format!("    let mut u_ni_c{k} = state.pot_{k}_u_ni;\n"));
                    for j in 0..k {
                        code.push_str(&format!(
                            "    for _i in 0..M {{ u_ni_c{k}[_i] -= scale_c{j} * u_ni_c{j}[_i] * dot_{j}_{k}; }}\n"
                        ));
                    }
                }
            }

            code.push('\n');
        }

        code
    }

    /// Emit dot product expression: su_cj^T * u_k
    /// u_k has entries +1 at node_p-1 and -1 at node_q-1 (grounded nodes omitted)
    fn emit_dot_su_u(j: usize, pot_k: &PotentiometerIR) -> String {
        if pot_k.node_p > 0 && pot_k.node_q > 0 {
            format!(
                "su_c{}[{}] - su_c{}[{}]",
                j,
                pot_k.node_p - 1,
                j,
                pot_k.node_q - 1
            )
        } else if pot_k.node_p > 0 {
            format!("su_c{}[{}]", j, pot_k.node_p - 1)
        } else if pot_k.node_q > 0 {
            format!("-su_c{}[{}]", j, pot_k.node_q - 1)
        } else {
            "0.0".to_string()
        }
    }

    /// Emit USU expression from corrected SU: u_k^T * su_ck
    fn emit_usu_from_su(k: usize, pot_k: &PotentiometerIR) -> String {
        if pot_k.node_p > 0 && pot_k.node_q > 0 {
            format!(
                "su_c{}[{}] - su_c{}[{}]",
                k,
                pot_k.node_p - 1,
                k,
                pot_k.node_q - 1
            )
        } else if pot_k.node_p > 0 {
            format!("su_c{}[{}]", k, pot_k.node_p - 1)
        } else if pot_k.node_q > 0 {
            format!("-su_c{}[{}]", k, pot_k.node_q - 1)
        } else {
            "0.0".to_string()
        }
    }

}

// ============================================================================
// Noise emission (Phase 1: Johnson-Nyquist thermal)
// ============================================================================

/// All code fragments produced for authentic circuit noise.
///
/// When the IR's noise mode is `Off` or no eligible sources are present,
/// every field is the empty string and no emitted Rust token differs from a
/// noiseless build. Fragments are injected at specific points in the
/// emitted file by the DK and nodal paths.
#[derive(Debug, Default)]
pub(super) struct NoiseEmission {
    /// Self-contained block: constants (K_B, T_ROOM_K, NOISE_THERMAL_N, …),
    /// RNG struct, SplitMix64, Marsaglia polar Gaussian helper. Emitted once
    /// between `emit_constants` and `emit_state`.
    pub top_level: String,
    /// Struct-field declarations for `CircuitState` (inside the struct body).
    pub state_fields: String,
    /// Initialiser expressions for `Default::default()` (the statements come
    /// first, the `self` fields come as trailing `field: value,` assignments).
    pub default_stmts: String,
    pub default_fields: String,
    /// Per-sample stamp into `build_rhs` (after existing RHS construction,
    /// before return). Empty when no sources.
    pub rhs_stamp: String,
    /// Body of `reset()` (re-seed RNG, clear gaussian cache).
    pub reset_body: String,
    /// Body to append inside `set_sample_rate` — recomputes `thermal_scale`.
    pub set_sample_rate_body: String,
    /// `impl CircuitState` methods: set_noise_enabled, set_noise_gain, …
    pub methods: String,
    /// `true` when any code is emitted (for template `{% if noise_enabled %}`).
    pub enabled: bool,
    /// Count of thermal sources. Kept for debug logging.
    #[allow(dead_code)]
    pub thermal_n: usize,
    /// Reverse lookup: `pot_index → noise source index` for dynamic
    /// sources (`.pot` / `.wiper` / `.runtime R` members). Empty vec of
    /// length `mna.pots.len()` when noise is off. A `Some(k)` entry means
    /// the pot setter for that index should update
    /// `state.noise_thermal_sqrt_inv_r[k]` after writing the new resistance.
    pub pot_to_noise_slot: Vec<Option<usize>>,
    /// Reverse lookup: `[switch_idx][comp_idx] → noise source index` for
    /// R-type switch components. The outer Vec is indexed by the switch
    /// number (matching `ir.switches`), the inner by the component index
    /// within that switch. A `Some(k)` entry means `set_switch_N(position)`
    /// should update `state.noise_thermal_sqrt_inv_r[k]` from the
    /// position-indexed R value. C/L components always map to `None`.
    pub switch_comp_to_noise_slot: Vec<Vec<Option<usize>>>,
}

impl RustEmitter {
    /// Produce all code fragments for Phase 1 thermal-noise emission.
    ///
    /// Returns all-empty `NoiseEmission` (enabled=false) when
    /// `ir.noise.mode == NoiseMode::Off` or `ir.noise.thermal_sources` is
    /// empty. All other phases (shot/flicker/op-amp) are deferred — their
    /// source lists are not populated by the IR builder yet.
    pub(super) fn build_noise_emission(&self, ir: &CircuitIR) -> NoiseEmission {
        let thermal_n = ir.noise.thermal_sources.len();
        let shot_n = ir.noise.shot_sources.len();
        let flicker_n = ir.noise.flicker_sources.len();
        if ir.noise.mode == NoiseMode::Off || (thermal_n == 0 && shot_n == 0 && flicker_n == 0) {
            return NoiseEmission::default();
        }

        let mut top = String::new();
        top.push_str("// ----------------------------------------------------------------------\n");
        top.push_str("// Authentic circuit noise — Phases 1 (thermal) + 2 (shot) + 3 (flicker)\n");
        top.push_str("// Generated when --noise {thermal|shot|full}. See docs/aidocs/NOISE.md\n");
        top.push_str("// ----------------------------------------------------------------------\n\n");

        top.push_str("/// Boltzmann constant [J/K] (exact SI 2019).\n");
        top.push_str("pub const K_B: f64 = 1.380649e-23;\n");
        top.push_str("/// Standard lab noise temperature [K] (16.85 °C, the \"kT\" reference).\n");
        top.push_str("pub const T_ROOM_K: f64 = 290.0;\n");
        if shot_n > 0 {
            top.push_str("/// Elementary charge [C] (exact SI 2019). Used for shot-noise PSD.\n");
            top.push_str("pub const Q_E: f64 = 1.602176634e-19;\n");
        }
        top.push_str(&format!("/// Default master seed baked in by codegen. `0` → entropy-seeded at Default.\n"));
        top.push_str(&format!(
            "pub const NOISE_MASTER_SEED_DEFAULT: u64 = {};\n\n",
            ir.noise.master_seed
        ));

        top.push_str(&format!("pub const NOISE_THERMAL_N: usize = {};\n", thermal_n));
        // Emit 1-indexed node arrays; 0 = ground (matches MNA convention)
        let fmt_usize_arr = |items: &[usize]| -> String {
            items
                .iter()
                .map(|v| v.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        };
        let node_i: Vec<usize> = ir.noise.thermal_sources.iter().map(|s| s.node_i).collect();
        let node_j: Vec<usize> = ir.noise.thermal_sources.iter().map(|s| s.node_j).collect();
        top.push_str(&format!(
            "pub(crate) const NOISE_THERMAL_NODE_I: [usize; NOISE_THERMAL_N] = [{}];\n",
            fmt_usize_arr(&node_i)
        ));
        top.push_str(&format!(
            "pub(crate) const NOISE_THERMAL_NODE_J: [usize; NOISE_THERMAL_N] = [{}];\n",
            fmt_usize_arr(&node_j)
        ));
        // Precomputed sqrt(1/R) default values — one per source. Static
        // entries (fixed resistors) are baked once and never change;
        // dynamic entries (`.pot` / `.wiper` / `.runtime R`) start from the
        // pot's nominal R and get refreshed in `set_pot_N` /
        // `set_runtime_R_<field>` so the per-sample coefficient tracks the
        // live resistance. The runtime mirror lives in
        // `state.noise_thermal_sqrt_inv_r[k]` — both are read from the same
        // index in the RHS stamp.
        let sqrt_inv_r: Vec<String> = ir
            .noise
            .thermal_sources
            .iter()
            .map(|s| fmt_f64((1.0 / s.resistance).sqrt()))
            .collect();
        top.push_str(&format!(
            "pub(crate) const NOISE_THERMAL_SQRT_INV_R_DEFAULT: [f64; NOISE_THERMAL_N] = [{}];\n\n",
            sqrt_inv_r.join(", ")
        ));

        // Shot-noise source table: one entry per forward-biased junction.
        // `SLOT_IDX` indexes `state.i_nl_prev`; the per-sample coefficient
        // is `sqrt(4·q·|I_prev|·fs)` with the same 2× trap-MNA calibration
        // as thermal (see `docs/aidocs/NOISE.md` "Constant derivation").
        // Gated on `shot_n > 0` so thermal-only builds stay byte-identical
        // to pre-Step-4 codegen — no dead `NOISE_SHOT_N = 0` constants leak.
        if shot_n > 0 {
            top.push_str(&format!("pub const NOISE_SHOT_N: usize = {};\n", shot_n));
            let shot_slot: Vec<usize> = ir.noise.shot_sources.iter().map(|s| s.slot_idx).collect();
            let shot_ni: Vec<usize> = ir.noise.shot_sources.iter().map(|s| s.node_i).collect();
            let shot_nj: Vec<usize> = ir.noise.shot_sources.iter().map(|s| s.node_j).collect();
            top.push_str(&format!(
                "pub(crate) const NOISE_SHOT_SLOT_IDX: [usize; NOISE_SHOT_N] = [{}];\n",
                fmt_usize_arr(&shot_slot)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_SHOT_NODE_I: [usize; NOISE_SHOT_N] = [{}];\n",
                fmt_usize_arr(&shot_ni)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_SHOT_NODE_J: [usize; NOISE_SHOT_N] = [{}];\n\n",
                fmt_usize_arr(&shot_nj)
            ));
        }

        // Flicker (1/f) noise source table. Per-sample amplitude is
        //   sqrt(4·fs) · sqrt(KF) · |I_prev|^(AF/2) · N(0,1)
        // fed into a Paul Kellett 7-pole pink filter. The `4·…·fs` matches
        // the thermal/shot trap-MNA compensation (see NOISE.md "Constant
        // derivation"). Source collection only adds devices whose `.model`
        // supplies `KF > 0`, so zero-KF builds leak no flicker constants.
        if flicker_n > 0 {
            top.push_str(&format!(
                "pub const NOISE_FLICKER_N: usize = {};\n",
                flicker_n
            ));
            let fl_slot: Vec<usize> = ir
                .noise
                .flicker_sources
                .iter()
                .map(|s| s.slot_idx)
                .collect();
            let fl_ni: Vec<usize> =
                ir.noise.flicker_sources.iter().map(|s| s.node_i).collect();
            let fl_nj: Vec<usize> =
                ir.noise.flicker_sources.iter().map(|s| s.node_j).collect();
            let fl_sqrt_kf: Vec<String> = ir
                .noise
                .flicker_sources
                .iter()
                .map(|s| fmt_f64(s.kf.sqrt()))
                .collect();
            let fl_half_af: Vec<String> = ir
                .noise
                .flicker_sources
                .iter()
                .map(|s| fmt_f64(0.5 * s.af))
                .collect();
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_SLOT_IDX: [usize; NOISE_FLICKER_N] = [{}];\n",
                fmt_usize_arr(&fl_slot)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_NODE_I: [usize; NOISE_FLICKER_N] = [{}];\n",
                fmt_usize_arr(&fl_ni)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_NODE_J: [usize; NOISE_FLICKER_N] = [{}];\n",
                fmt_usize_arr(&fl_nj)
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_SQRT_KF: [f64; NOISE_FLICKER_N] = [{}];\n",
                fl_sqrt_kf.join(", ")
            ));
            top.push_str(&format!(
                "pub(crate) const NOISE_FLICKER_HALF_AF: [f64; NOISE_FLICKER_N] = [{}];\n\n",
                fl_half_af.join(", ")
            ));
        }

        // xoshiro256++ RNG: fast, high-quality, 256-bit state per stream.
        top.push_str("#[derive(Clone, Copy, Debug)]\n");
        top.push_str("pub struct Xoshiro256pp { pub s: [u64; 4] }\n\n");
        top.push_str("impl Xoshiro256pp {\n");
        top.push_str("    #[inline(always)]\n");
        top.push_str("    pub fn next_u64(&mut self) -> u64 {\n");
        top.push_str("        let result = self.s[0].wrapping_add(self.s[3]).rotate_left(23).wrapping_add(self.s[0]);\n");
        top.push_str("        let t = self.s[1] << 17;\n");
        top.push_str("        self.s[2] ^= self.s[0];\n");
        top.push_str("        self.s[3] ^= self.s[1];\n");
        top.push_str("        self.s[1] ^= self.s[2];\n");
        top.push_str("        self.s[0] ^= self.s[3];\n");
        top.push_str("        self.s[2] ^= t;\n");
        top.push_str("        self.s[3] = self.s[3].rotate_left(45);\n");
        top.push_str("        result\n");
        top.push_str("    }\n");
        top.push_str("    /// Uniform f64 in [0, 1). Upper 53 bits of the u64.\n");
        top.push_str("    #[inline(always)]\n");
        top.push_str("    pub fn next_f64(&mut self) -> f64 {\n");
        top.push_str("        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)\n");
        top.push_str("    }\n");
        top.push_str("}\n\n");

        // SplitMix64: derives per-stream seeds from one master seed.
        top.push_str("#[inline(always)]\n");
        top.push_str("fn splitmix64(state: &mut u64) -> u64 {\n");
        top.push_str("    *state = state.wrapping_add(0x9E3779B97F4A7C15);\n");
        top.push_str("    let mut z = *state;\n");
        top.push_str("    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);\n");
        top.push_str("    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);\n");
        top.push_str("    z ^ (z >> 31)\n");
        top.push_str("}\n\n");

        top.push_str("/// Seed an array of Xoshiro256pp streams from one master seed via SplitMix64.\n");
        top.push_str("/// Every stream gets statistically independent state — no cross-source correlation.\n");
        top.push_str("fn seed_noise_rngs<const N: usize>(master: u64) -> [Xoshiro256pp; N] {\n");
        top.push_str("    seed_noise_rngs_salted::<N>(master, 0)\n");
        top.push_str("}\n\n");
        top.push_str("/// Seed an array of Xoshiro256pp streams with an additional 64-bit salt.\n");
        top.push_str("/// Used to derive per-phase streams (thermal vs shot) from one master\n");
        top.push_str("/// seed without any cross-phase prefix overlap. Salt is XORed into the\n");
        top.push_str("/// SplitMix64 state after the standard entropy resolution.\n");
        top.push_str("fn seed_noise_rngs_salted<const N: usize>(master: u64, salt: u64) -> [Xoshiro256pp; N] {\n");
        top.push_str("    let mut sm = if master == 0 {\n");
        top.push_str("        // master=0 → entropy from system clock. Plugin hosts wanting\n");
        top.push_str("        // determinism should call set_seed(nonzero) before processing.\n");
        top.push_str("        std::time::SystemTime::now()\n");
        top.push_str("            .duration_since(std::time::UNIX_EPOCH)\n");
        top.push_str("            .map(|d| d.as_nanos() as u64)\n");
        top.push_str("            .unwrap_or(0x0123456789ABCDEF)\n");
        top.push_str("    } else { master };\n");
        top.push_str("    sm ^= salt;\n");
        top.push_str("    // First mix to avoid weak seeds\n");
        top.push_str("    let _ = splitmix64(&mut sm);\n");
        top.push_str("    let mut out = [Xoshiro256pp { s: [0; 4] }; N];\n");
        top.push_str("    for k in 0..N {\n");
        top.push_str("        out[k].s[0] = splitmix64(&mut sm);\n");
        top.push_str("        out[k].s[1] = splitmix64(&mut sm);\n");
        top.push_str("        out[k].s[2] = splitmix64(&mut sm);\n");
        top.push_str("        out[k].s[3] = splitmix64(&mut sm);\n");
        top.push_str("        // xoshiro requires at least one nonzero state word.\n");
        top.push_str("        if out[k].s == [0; 4] { out[k].s[0] = 1; }\n");
        top.push_str("    }\n");
        top.push_str("    out\n");
        top.push_str("}\n\n");
        top.push_str("/// Shot-noise salt. Distinct 64-bit constant applied to the master\n");
        top.push_str("/// seed so shot streams do not share any prefix with thermal streams\n");
        top.push_str("/// under a deterministic seed. Chosen to be a high-entropy value.\n");
        top.push_str("pub const NOISE_SHOT_SALT: u64 = 0xA5A5_DEAD_BEEF_CAFE;\n\n");

        if flicker_n > 0 {
            top.push_str("/// Flicker-noise salt. Distinct from thermal and shot salts so\n");
            top.push_str("/// every pink-filter input stream is independent under a\n");
            top.push_str("/// deterministic master seed.\n");
            top.push_str(
                "pub const NOISE_FLICKER_SALT: u64 = 0xC0DE_BABE_DEAD_BEEF;\n\n",
            );
            // Paul Kellett 7-pole pink filter (musicdsp.org pk3 variant,
            // ±0.05 dB over 9.2 octaves). White in, pink out with a ~1/f
            // PSD shape. The `* 0.11` tail normalizes the cascade to ~unit
            // RMS gain on unit-variance white — without it, Kellett's raw
            // output is ~3× the input RMS. Calibration is empirical; the
            // shipped PSD test asserts slope, not absolute level.
            top.push_str("#[inline(always)]\n");
            top.push_str("fn kellett_pink(white: f64, state: &mut [f64; 7]) -> f64 {\n");
            top.push_str("    state[0] = 0.99886 * state[0] + white * 0.0555179;\n");
            top.push_str("    state[1] = 0.99332 * state[1] + white * 0.0750759;\n");
            top.push_str("    state[2] = 0.96900 * state[2] + white * 0.1538520;\n");
            top.push_str("    state[3] = 0.86650 * state[3] + white * 0.3104856;\n");
            top.push_str("    state[4] = 0.55000 * state[4] + white * 0.5329522;\n");
            top.push_str("    state[5] = -0.7616 * state[5] - white * 0.0168980;\n");
            top.push_str("    let pink = state[0] + state[1] + state[2] + state[3]\n");
            top.push_str("        + state[4] + state[5] + state[6] + white * 0.5362;\n");
            top.push_str("    state[6] = white * 0.115926;\n");
            top.push_str("    pink * 0.11\n");
            top.push_str("}\n\n");
        }

        // Gaussian via Marsaglia polar method.
        top.push_str("/// Standard-normal (µ=0, σ=1) sample via Marsaglia polar method.\n");
        top.push_str("/// One RNG pair yields two Gaussians; the second is cached for the next call.\n");
        top.push_str("#[inline(always)]\n");
        top.push_str("fn gaussian(rng: &mut Xoshiro256pp, cache: &mut Option<f64>) -> f64 {\n");
        top.push_str("    if let Some(z) = cache.take() { return z; }\n");
        top.push_str("    loop {\n");
        top.push_str("        let u = 2.0 * rng.next_f64() - 1.0;\n");
        top.push_str("        let v = 2.0 * rng.next_f64() - 1.0;\n");
        top.push_str("        let s = u * u + v * v;\n");
        top.push_str("        if s > 0.0 && s < 1.0 {\n");
        top.push_str("            let factor = (-2.0 * s.ln() / s).sqrt();\n");
        top.push_str("            *cache = Some(v * factor);\n");
        top.push_str("            return u * factor;\n");
        top.push_str("        }\n");
        top.push_str("    }\n");
        top.push_str("}\n\n");

        // State fields — injected inside CircuitState struct body
        let mut state_fields = String::new();
        state_fields.push_str(&format!(
            "    /// Per-source xoshiro256++ state — independent streams, no cross-correlation.\n"
        ));
        state_fields.push_str(&format!(
            "    pub noise_rng: [Xoshiro256pp; NOISE_THERMAL_N],\n"
        ));
        state_fields.push_str(&format!(
            "    /// Cached second Gaussian from Marsaglia polar pair.\n"
        ));
        state_fields.push_str(&format!(
            "    pub noise_gaussian_cache: [Option<f64>; NOISE_THERMAL_N],\n"
        ));
        state_fields.push_str("    /// Master noise switch — runtime. Default false (opt-in).\n");
        state_fields.push_str("    pub noise_enabled: bool,\n");
        state_fields.push_str("    /// Master scalar applied to every noise source.\n");
        state_fields.push_str("    pub noise_gain: f64,\n");
        state_fields.push_str("    /// Scalar applied only to Johnson-Nyquist thermal sources.\n");
        state_fields.push_str("    pub thermal_gain: f64,\n");
        state_fields.push_str("    /// Circuit temperature [K]. Runtime-settable.\n");
        state_fields.push_str("    pub temperature_k: f64,\n");
        state_fields.push_str("    /// Master seed recorded for reset() re-derivation.\n");
        state_fields.push_str("    pub noise_master_seed: u64,\n");
        state_fields.push_str("    /// Precomputed sqrt(8·K_B·T·fs_internal). Per-source coefficient is this × sqrt(1/R). See kTC-theorem note at codegen site.\n");
        state_fields.push_str("    pub noise_thermal_scale: f64,\n");
        state_fields.push_str("    /// Effective internal sample rate (host_rate × OVERSAMPLING_FACTOR).\n");
        state_fields.push_str("    /// Tracked so `set_temperature_k` can recompute `noise_thermal_scale`.\n");
        state_fields.push_str("    pub noise_fs: f64,\n");
        state_fields.push_str("    /// Per-source `sqrt(1/R)` — live mirror of `NOISE_THERMAL_SQRT_INV_R_DEFAULT`.\n");
        state_fields.push_str("    /// Static entries stay at their baked value. Dynamic entries (`.pot` /\n");
        state_fields.push_str("    /// `.wiper` / `.runtime R` members) are refreshed inside the matching\n");
        state_fields.push_str("    /// `set_pot_N` / `set_runtime_R_<field>` setter.\n");
        state_fields.push_str("    pub noise_thermal_sqrt_inv_r: [f64; NOISE_THERMAL_N],\n");
        if shot_n > 0 {
            state_fields.push_str("    /// Per-source xoshiro256++ state for shot noise — salted so streams\n");
            state_fields.push_str("    /// cannot share a prefix with thermal streams under any master seed.\n");
            state_fields.push_str("    pub noise_shot_rng: [Xoshiro256pp; NOISE_SHOT_N],\n");
            state_fields.push_str("    /// Cached second Gaussian from Marsaglia polar pair (shot stream).\n");
            state_fields.push_str("    pub noise_shot_gaussian_cache: [Option<f64>; NOISE_SHOT_N],\n");
            state_fields.push_str("    /// Scalar applied only to shot-noise sources (Phase 2). Runtime.\n");
            state_fields.push_str("    pub shot_gain: f64,\n");
            state_fields.push_str("    /// Precomputed `sqrt(4·Q_E·fs_internal)`. The per-sample shot\n");
            state_fields.push_str("    /// amplitude is `noise_shot_scale · sqrt(|I_prev|) · N(0,1)`.\n");
            state_fields.push_str("    /// Updated by `set_sample_rate`.\n");
            state_fields.push_str("    pub noise_shot_scale: f64,\n");
        }
        if flicker_n > 0 {
            state_fields.push_str("    /// Per-source xoshiro256++ state for the Kellett pink-filter\n");
            state_fields.push_str("    /// input — salted distinct from thermal and shot streams.\n");
            state_fields.push_str("    pub noise_flicker_rng: [Xoshiro256pp; NOISE_FLICKER_N],\n");
            state_fields.push_str("    /// Cached second Gaussian from Marsaglia polar pair (flicker stream).\n");
            state_fields.push_str("    pub noise_flicker_gaussian_cache: [Option<f64>; NOISE_FLICKER_N],\n");
            state_fields.push_str("    /// Per-source 7-pole Kellett filter state. Zeroed at `default()`\n");
            state_fields.push_str("    /// and at `reset()`; settles in a handful of samples once audio\n");
            state_fields.push_str("    /// processing begins.\n");
            state_fields.push_str("    pub noise_flicker_state: [[f64; 7]; NOISE_FLICKER_N],\n");
            state_fields.push_str("    /// Scalar applied only to flicker sources (Phase 3). Runtime.\n");
            state_fields.push_str("    pub flicker_gain: f64,\n");
            state_fields.push_str("    /// Precomputed `sqrt(4·fs_internal)`. Per-sample amplitude is\n");
            state_fields.push_str("    /// `noise_flicker_scale · NOISE_FLICKER_SQRT_KF[k] · |I_prev|^(AF/2)`\n");
            state_fields.push_str("    /// before being shaped by the Kellett pink filter.\n");
            state_fields.push_str("    pub noise_flicker_scale: f64,\n");
        }

        // Default impl: compute thermal_scale and seed RNGs
        let mut default_stmts = String::new();
        default_stmts.push_str("        // Noise state (thermal only in Phase 1)\n");
        default_stmts.push_str("        let fs_internal = SAMPLE_RATE * OVERSAMPLING_FACTOR as f64;\n");
        // Per-sample Norton-current variance:  σ² = 8·k_B·T·fs / R
        // (The physically correct one-sided PSD  S_i = 4·k_B·T/R  over [0, fs/2]
        //  would naively give σ² = 2·k_B·T·fs/R, but melange's DK-trap
        //  formulation satisfies  (A - A_neg)·v_ss = stamp,  so a steady
        //  current source gets half the continuous-time DC gain. The input
        //  stamp compensates via (V_new + V_prev)·G_in; here we compensate
        //  by doubling the per-sample variance instead of caching a second
        //  RNG draw. Net: output V²_rms = kT/C across every RC lowpass,
        //  matching the Nyquist kTC equilibrium. Validated by the kTC
        //  theorem test in tests/noise_psd_validation.rs.)
        default_stmts.push_str("        let noise_thermal_scale = (8.0 * K_B * T_ROOM_K * fs_internal).sqrt();\n");
        default_stmts.push_str("        let noise_rng = seed_noise_rngs::<NOISE_THERMAL_N>(NOISE_MASTER_SEED_DEFAULT);\n");
        if shot_n > 0 {
            default_stmts.push_str("        // Shot-noise streams: salted distinct from thermal streams.\n");
            default_stmts.push_str("        // Per-sample variance: σ² = 4·Q_E·|I|·fs. Same 2× trap-MNA\n");
            default_stmts.push_str("        // compensation as thermal (PSD 2·q·|I| on [0, fs/2] gives\n");
            default_stmts.push_str("        // naive σ² = q·|I|·fs; doubling to preserve DK-trap DC gain\n");
            default_stmts.push_str("        // yields 2·q·|I|·fs; amplitude expressed as sqrt(X·Q_E·|I|·fs)\n");
            default_stmts.push_str("        // has X=4).\n");
            default_stmts.push_str("        let noise_shot_scale = (4.0 * Q_E * fs_internal).sqrt();\n");
            default_stmts.push_str("        let noise_shot_rng = seed_noise_rngs_salted::<NOISE_SHOT_N>(NOISE_MASTER_SEED_DEFAULT, NOISE_SHOT_SALT);\n");
        }
        if flicker_n > 0 {
            default_stmts.push_str("        // Flicker streams: salted distinct from thermal/shot streams.\n");
            default_stmts.push_str("        // Per-source white-input variance fed into the Kellett filter:\n");
            default_stmts.push_str("        //   σ_w² = 4·KF·|I|^AF·fs\n");
            default_stmts.push_str("        // (same 2× trap-MNA compensation as thermal/shot). The `sqrt(4·fs)`\n");
            default_stmts.push_str("        // factor is shared across sources; sqrt(KF) and |I|^(AF/2) live\n");
            default_stmts.push_str("        // in per-source constants and runtime state.\n");
            default_stmts.push_str("        let noise_flicker_scale = (4.0 * fs_internal).sqrt();\n");
            default_stmts.push_str("        let noise_flicker_rng = seed_noise_rngs_salted::<NOISE_FLICKER_N>(NOISE_MASTER_SEED_DEFAULT, NOISE_FLICKER_SALT);\n");
        }

        let mut default_fields = String::new();
        default_fields.push_str("            noise_rng,\n");
        default_fields.push_str("            noise_gaussian_cache: [None; NOISE_THERMAL_N],\n");
        default_fields.push_str("            noise_enabled: false,\n");
        default_fields.push_str("            noise_gain: 1.0,\n");
        default_fields.push_str("            thermal_gain: 1.0,\n");
        default_fields.push_str("            temperature_k: T_ROOM_K,\n");
        default_fields.push_str("            noise_master_seed: NOISE_MASTER_SEED_DEFAULT,\n");
        default_fields.push_str("            noise_thermal_scale,\n");
        default_fields.push_str("            noise_fs: fs_internal,\n");
        default_fields.push_str("            noise_thermal_sqrt_inv_r: NOISE_THERMAL_SQRT_INV_R_DEFAULT,\n");
        if shot_n > 0 {
            default_fields.push_str("            noise_shot_rng,\n");
            default_fields.push_str("            noise_shot_gaussian_cache: [None; NOISE_SHOT_N],\n");
            default_fields.push_str("            shot_gain: 1.0,\n");
            default_fields.push_str("            noise_shot_scale,\n");
        }
        if flicker_n > 0 {
            default_fields.push_str("            noise_flicker_rng,\n");
            default_fields.push_str(
                "            noise_flicker_gaussian_cache: [None; NOISE_FLICKER_N],\n",
            );
            default_fields
                .push_str("            noise_flicker_state: [[0.0; 7]; NOISE_FLICKER_N],\n");
            default_fields.push_str("            flicker_gain: 1.0,\n");
            default_fields.push_str("            noise_flicker_scale,\n");
        }

        // reset() — reseed RNG and clear gaussian cache; keep user settings.
        let mut reset_body = String::new();
        reset_body.push_str("        // Re-seed noise RNGs (keeps noise_enabled, gains, temperature untouched).\n");
        reset_body.push_str("        self.noise_rng = seed_noise_rngs::<NOISE_THERMAL_N>(self.noise_master_seed);\n");
        reset_body.push_str("        self.noise_gaussian_cache = [None; NOISE_THERMAL_N];\n");
        reset_body.push_str("        // Restore per-source sqrt(1/R) to defaults. Dynamic sources will\n");
        reset_body.push_str("        // be re-updated by any subsequent set_pot_N / set_runtime_R call;\n");
        reset_body.push_str("        // this mirrors how reset() restores pot_<i>_resistance to nominal.\n");
        reset_body.push_str("        self.noise_thermal_sqrt_inv_r = NOISE_THERMAL_SQRT_INV_R_DEFAULT;\n");
        if shot_n > 0 {
            reset_body.push_str("        // Re-seed shot RNGs (same master, distinct salt from thermal).\n");
            reset_body.push_str("        self.noise_shot_rng = seed_noise_rngs_salted::<NOISE_SHOT_N>(self.noise_master_seed, NOISE_SHOT_SALT);\n");
            reset_body.push_str("        self.noise_shot_gaussian_cache = [None; NOISE_SHOT_N];\n");
        }
        if flicker_n > 0 {
            reset_body.push_str(
                "        // Re-seed flicker RNGs + zero Kellett pink-filter state.\n",
            );
            reset_body.push_str("        self.noise_flicker_rng = seed_noise_rngs_salted::<NOISE_FLICKER_N>(self.noise_master_seed, NOISE_FLICKER_SALT);\n");
            reset_body
                .push_str("        self.noise_flicker_gaussian_cache = [None; NOISE_FLICKER_N];\n");
            reset_body
                .push_str("        self.noise_flicker_state = [[0.0; 7]; NOISE_FLICKER_N];\n");
        }

        // set_sample_rate tail: recompute thermal_scale at the new rate
        let mut ssr_body = String::new();
        ssr_body.push_str("        // Noise: recompute sqrt(4·K_B·T·fs_internal) for the new rate.\n");
        ssr_body.push_str("        self.noise_fs = sample_rate * OVERSAMPLING_FACTOR as f64;\n");
        ssr_body.push_str("        self.noise_thermal_scale = (8.0 * K_B * self.temperature_k * self.noise_fs).sqrt();\n");
        if shot_n > 0 {
            ssr_body.push_str("        self.noise_shot_scale = (4.0 * Q_E * self.noise_fs).sqrt();\n");
        }
        if flicker_n > 0 {
            ssr_body.push_str(
                "        self.noise_flicker_scale = (4.0 * self.noise_fs).sqrt();\n",
            );
        }

        // Public API
        let mut methods = String::new();
        methods.push_str("\n    // --- Noise controls (Phase 1) ---\n\n");
        methods.push_str("    /// Turn circuit noise on or off. When off, all per-sample RNG calls are skipped.\n");
        methods.push_str("    pub fn set_noise_enabled(&mut self, on: bool) { self.noise_enabled = on; }\n\n");
        methods.push_str("    /// Master scalar applied to every noise source (post-per-category).\n");
        methods.push_str("    pub fn set_noise_gain(&mut self, gain: f64) { self.noise_gain = gain; }\n\n");
        methods.push_str("    /// Scalar applied only to Johnson-Nyquist thermal sources.\n");
        methods.push_str("    pub fn set_thermal_gain(&mut self, gain: f64) { self.thermal_gain = gain; }\n\n");
        methods.push_str("    /// Circuit temperature in Kelvin. 290 K is standard (~16.85 °C).\n");
        methods.push_str("    /// Cold gear is quieter: 77 K (liquid N2) ≈ −5.76 dB, 3 K ≈ −19.9 dB.\n");
        methods.push_str("    pub fn set_temperature_k(&mut self, kelvin: f64) {\n");
        methods.push_str("        if !(kelvin.is_finite() && kelvin > 0.0) { return; }\n");
        methods.push_str("        self.temperature_k = kelvin;\n");
        methods.push_str("        // Recompute thermal_scale at the currently-set sample rate.\n");
        methods.push_str("        self.noise_thermal_scale = (8.0 * K_B * kelvin * self.noise_fs).sqrt();\n");
        methods.push_str("    }\n\n");
        if shot_n > 0 {
            methods.push_str("    /// Scalar applied only to shot (junction) noise sources.\n");
            methods.push_str("    /// Runtime-settable. Set to `0.0` to mute shot content without\n");
            methods.push_str("    /// also muting thermal.\n");
            methods.push_str("    pub fn set_shot_gain(&mut self, gain: f64) { self.shot_gain = gain; }\n\n");
        }
        if flicker_n > 0 {
            methods.push_str("    /// Scalar applied only to flicker (1/f) noise sources.\n");
            methods.push_str("    /// Runtime-settable. Set to `0.0` to mute flicker without\n");
            methods.push_str("    /// also muting thermal or shot.\n");
            methods.push_str(
                "    pub fn set_flicker_gain(&mut self, gain: f64) { self.flicker_gain = gain; }\n\n",
            );
        }
        methods.push_str("    /// Set the master seed. `0` → entropy-seeded from system clock.\n");
        methods.push_str("    /// Any nonzero value → deterministic (same seed → bit-identical noise).\n");
        methods.push_str("    pub fn set_seed(&mut self, master: u64) {\n");
        methods.push_str("        self.noise_master_seed = master;\n");
        methods.push_str("        self.noise_rng = seed_noise_rngs::<NOISE_THERMAL_N>(master);\n");
        methods.push_str("        self.noise_gaussian_cache = [None; NOISE_THERMAL_N];\n");
        if shot_n > 0 {
            methods.push_str("        self.noise_shot_rng = seed_noise_rngs_salted::<NOISE_SHOT_N>(master, NOISE_SHOT_SALT);\n");
            methods.push_str("        self.noise_shot_gaussian_cache = [None; NOISE_SHOT_N];\n");
        }
        if flicker_n > 0 {
            methods.push_str("        self.noise_flicker_rng = seed_noise_rngs_salted::<NOISE_FLICKER_N>(master, NOISE_FLICKER_SALT);\n");
            methods
                .push_str("        self.noise_flicker_gaussian_cache = [None; NOISE_FLICKER_N];\n");
            methods.push_str("        self.noise_flicker_state = [[0.0; 7]; NOISE_FLICKER_N];\n");
        }
        methods.push_str("    }\n");

        // build_rhs stamp
        let mut rhs_stamp = String::new();
        rhs_stamp.push_str("\n    // Authentic circuit noise — Phases 1 (thermal) + 2 (shot).\n");
        rhs_stamp.push_str("    // Skipped entirely (zero RNG calls) when noise_enabled is false.\n");
        rhs_stamp.push_str("    if state.noise_enabled {\n");
        rhs_stamp.push_str("        let scale = state.noise_thermal_scale * state.noise_gain * state.thermal_gain;\n");
        rhs_stamp.push_str("        if scale != 0.0 {\n");
        rhs_stamp.push_str("            for k in 0..NOISE_THERMAL_N {\n");
        rhs_stamp.push_str("                let g = gaussian(&mut state.noise_rng[k], &mut state.noise_gaussian_cache[k]);\n");
        rhs_stamp.push_str("                let i_n = scale * state.noise_thermal_sqrt_inv_r[k] * g;\n");
        rhs_stamp.push_str("                let ni = NOISE_THERMAL_NODE_I[k];\n");
        rhs_stamp.push_str("                let nj = NOISE_THERMAL_NODE_J[k];\n");
        rhs_stamp.push_str("                if ni > 0 { rhs[ni - 1] += i_n; }\n");
        rhs_stamp.push_str("                if nj > 0 { rhs[nj - 1] -= i_n; }\n");
        rhs_stamp.push_str("            }\n");
        rhs_stamp.push_str("        }\n");
        if shot_n > 0 {
            rhs_stamp.push_str("        // Shot: sqrt(4·q·|I_prev|·fs) per source. `|I_prev|` is the\n");
            rhs_stamp.push_str("        // one-sample-lagged junction current from `state.i_nl_prev`;\n");
            rhs_stamp.push_str("        // at audio rates the lag is inaudible and lets the stamp run\n");
            rhs_stamp.push_str("        // before NR. The gate includes `shot_gain` so hosts can mute\n");
            rhs_stamp.push_str("        // shot alone without touching thermal.\n");
            rhs_stamp.push_str("        let shot_scale = state.noise_shot_scale * state.noise_gain * state.shot_gain;\n");
            rhs_stamp.push_str("        if shot_scale != 0.0 {\n");
            rhs_stamp.push_str("            for k in 0..NOISE_SHOT_N {\n");
            rhs_stamp.push_str("                let i_abs = state.i_nl_prev[NOISE_SHOT_SLOT_IDX[k]].abs();\n");
            rhs_stamp.push_str("                if i_abs <= 0.0 { continue; }\n");
            rhs_stamp.push_str("                let g = gaussian(&mut state.noise_shot_rng[k], &mut state.noise_shot_gaussian_cache[k]);\n");
            rhs_stamp.push_str("                let i_n = shot_scale * i_abs.sqrt() * g;\n");
            rhs_stamp.push_str("                let ni = NOISE_SHOT_NODE_I[k];\n");
            rhs_stamp.push_str("                let nj = NOISE_SHOT_NODE_J[k];\n");
            rhs_stamp.push_str("                if ni > 0 { rhs[ni - 1] += i_n; }\n");
            rhs_stamp.push_str("                if nj > 0 { rhs[nj - 1] -= i_n; }\n");
            rhs_stamp.push_str("            }\n");
            rhs_stamp.push_str("        }\n");
        }
        if flicker_n > 0 {
            rhs_stamp.push_str("        // Flicker (1/f): white draw → sqrt(4·KF·|I|^AF·fs) scale →\n");
            rhs_stamp.push_str("        // Kellett 7-pole pink filter → RHS. `|I_prev|` comes from\n");
            rhs_stamp.push_str("        // `state.i_nl_prev` (one-sample lag, same as shot). The per-\n");
            rhs_stamp.push_str("        // source sqrt(KF) and AF/2 are compile-time baked so the hot\n");
            rhs_stamp.push_str("        // loop is branch-free.\n");
            rhs_stamp.push_str("        let flicker_scale = state.noise_flicker_scale * state.noise_gain * state.flicker_gain;\n");
            rhs_stamp.push_str("        if flicker_scale != 0.0 {\n");
            rhs_stamp.push_str("            for k in 0..NOISE_FLICKER_N {\n");
            rhs_stamp.push_str("                let i_abs = state.i_nl_prev[NOISE_FLICKER_SLOT_IDX[k]].abs();\n");
            rhs_stamp.push_str("                if i_abs <= 0.0 { continue; }\n");
            rhs_stamp.push_str("                let white = gaussian(&mut state.noise_flicker_rng[k], &mut state.noise_flicker_gaussian_cache[k]);\n");
            rhs_stamp.push_str("                let pink = kellett_pink(white, &mut state.noise_flicker_state[k]);\n");
            rhs_stamp.push_str("                let amp = flicker_scale * NOISE_FLICKER_SQRT_KF[k] * i_abs.powf(NOISE_FLICKER_HALF_AF[k]);\n");
            rhs_stamp.push_str("                let i_n = amp * pink;\n");
            rhs_stamp.push_str("                let ni = NOISE_FLICKER_NODE_I[k];\n");
            rhs_stamp.push_str("                let nj = NOISE_FLICKER_NODE_J[k];\n");
            rhs_stamp.push_str("                if ni > 0 { rhs[ni - 1] += i_n; }\n");
            rhs_stamp.push_str("                if nj > 0 { rhs[nj - 1] -= i_n; }\n");
            rhs_stamp.push_str("            }\n");
            rhs_stamp.push_str("        }\n");
        }
        rhs_stamp.push_str("    }\n");

        // Reverse lookup populated from each source's `pot_slot`. Pots with
        // no noise source (there aren't any in the current pipeline, but
        // future FA reductions / skip lists may produce them) stay `None`.
        let mut pot_to_noise_slot = vec![None; ir.pots.len()];
        for (k, src) in ir.noise.thermal_sources.iter().enumerate() {
            if let Some(p) = src.pot_slot {
                if p < pot_to_noise_slot.len() {
                    pot_to_noise_slot[p] = Some(k);
                }
            }
        }

        // Same for switches: a per-(switch, component) reverse lookup so
        // the emitted `set_switch_N(position)` can spot-update every
        // R-backed noise slot. C/L components map to `None` by default.
        let mut switch_comp_to_noise_slot: Vec<Vec<Option<usize>>> = ir
            .switches
            .iter()
            .map(|sw| vec![None; sw.components.len()])
            .collect();
        for (k, src) in ir.noise.thermal_sources.iter().enumerate() {
            if let Some((sw, comp)) = src.switch_slot {
                if sw < switch_comp_to_noise_slot.len()
                    && comp < switch_comp_to_noise_slot[sw].len()
                {
                    switch_comp_to_noise_slot[sw][comp] = Some(k);
                }
            }
        }

        NoiseEmission {
            top_level: top,
            state_fields,
            default_stmts,
            default_fields,
            rhs_stamp,
            reset_body,
            set_sample_rate_body: ssr_body,
            methods,
            enabled: true,
            thermal_n,
            pot_to_noise_slot,
            switch_comp_to_noise_slot,
        }
    }
}
