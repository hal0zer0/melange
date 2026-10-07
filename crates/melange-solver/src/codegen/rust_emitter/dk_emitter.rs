//! DK-path code emission methods.
//!
//! Contains the DK entry point (`emit_dk`) and the template-based emission
//! methods for constants, state, switches, pots, RHS, and process_sample. The
//! pieces shared with the nodal path live in `header`, `device_models`,
//! `oversampler`, `inject_tap` and `noise_emitter`.

use tera::Context;

pub(super) use super::header::GlowProvenance;
use super::inject_tap::insert_inject_ctx;
pub(super) use super::inject_tap::{
    emit_inject_rhs_stamp, emit_inject_substep_stamp, emit_inject_tap_constants, emit_warmup_call,
};
pub(super) use super::noise_emitter::{emit_noise_replay_body, NoiseEmission};

use super::helpers::{
    carries_q_dot, device_param_template_data, emit_stateful_default_fields,
    emit_stateful_set_sample_rate_body, emit_stateful_state_fields, emit_stateful_state_restore,
    emit_stateful_update, emit_thermal_tj_advance, fmt_f64, format_matrix_rows,
    history_zero_row_ranges, named_const_entries, oversampling_info, q_dot_start,
    recommended_warmup_samples, section_banner, self_heating_device_data, stateful_device_data,
    warmup_estimate_capped, SwitchCompTemplateData, SwitchTemplateData,
};
use super::RustEmitter;
use crate::codegen::ir::{CircuitIR, DeviceParams, DeviceType};
use crate::codegen::CodegenError;

/// Insert the multi-input-port Tera variables (`multi_input`, `num_inputs`,
/// `input_nodes_values`, `input_resistances_values`) into a template context.
///
/// `multi_input` gates every input-related emission divergence; when it is
/// false (the single-input case) the templates emit the historical single-input
/// code byte-for-byte. Called for every local context whose template references
/// these variables (constants, state, build_rhs, process_sample).
fn insert_multi_input_ctx(ctx: &mut Context, ir: &CircuitIR) {
    let num_inputs = ir.solver_config.num_inputs();
    ctx.insert("multi_input", &(num_inputs > 1));
    ctx.insert("num_inputs", &num_inputs);
    let input_nodes_values = ir
        .solver_config
        .input_node_indices()
        .iter()
        .map(|n| n.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    ctx.insert("input_nodes_values", &input_nodes_values);
    let input_resistances_values = ir
        .solver_config
        .input_resistance_values()
        .iter()
        .map(|r| fmt_f64(*r))
        .collect::<Vec<_>>()
        .join(", ");
    ctx.insert("input_resistances_values", &input_resistances_values);
}

/// Charge form (DK): `q_dot` for the committed `v`, before `v_prev` moves:
/// trapezoidal `alpha*C*(v - v_prev) - q_dot`, or on a BE-fallback sample
/// `(1/T)*C*(v - v_prev)` (which re-seeds the trapezoidal history).
fn dk_q_dot_commit(ir: &CircuitIR, has_be_fallback: bool) -> String {
    let n = ir.topology.n;
    let lines = |mat: &str, sp: &crate::codegen::ir::MatrixSparsity, trap: bool, ind: &str| {
        let mut out = String::new();
        for i in 0..n {
            let terms: Vec<String> = sp.nz_by_row[i]
                .iter()
                .map(|&j| format!("state.{mat}[{i}][{j}] * (v[{j}] - state.v_prev[{j}])"))
                .collect();
            if terms.is_empty() {
                continue;
            }
            let tail = if trap {
                format!(" - state.q_dot[{i}]")
            } else {
                String::new()
            };
            out.push_str(&format!("{ind}q[{i}] = {}{tail};\n", terms.join(" + ")));
        }
        out
    };
    let mut code = String::from(
        "    // Charge form: q_dot = C*dx/dt at the committed sample.\n\
         \x20   let mut q = [0.0f64; N];\n",
    );
    if has_be_fallback {
        code.push_str("    if q_be {\n");
        code.push_str(&lines("a_neg_be", &ir.sparsity.a_neg_be, false, "        "));
        code.push_str("    } else {\n");
        code.push_str(&lines("a_neg", &ir.sparsity.a_neg, true, "        "));
        code.push_str("    }\n");
    } else {
        code.push_str(&lines("a_neg", &ir.sparsity.a_neg, true, "    "));
    }
    code.push_str("    state.q_dot = q;\n");
    code
}

impl RustEmitter {
    /// Emit DK-method generated code (original path).
    pub(super) fn emit_dk(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        // `state.k` holds K − R_p on each parasitic-absorbed BJT's 2×2 block,
        // and the NR loop reads K only at its pattern's positions: a block
        // position outside the pattern would drop the parasitic drop.
        let m = ir.topology.m;
        for i in 0..m {
            for j in 0..m {
                if parasitic_r_p_dk(ir, i, j) != 0.0
                    && !ir
                        .sparsity
                        .k
                        .nz_by_row
                        .get(i)
                        .is_some_and(|r| r.contains(&j))
                {
                    return Err(CodegenError::InvalidConfig(format!(
                        "structural sparsity: K[{i}][{j}] carries a parasitic BJT resistance \
                         but is outside K's pattern. This is a melange bug; please report it \
                         with the deck."
                    )));
                }
            }
        }
        let mut code = String::new();
        let noise = self.build_noise_emission(ir);

        let glow_prov = GlowProvenance::for_dk(ir);
        // DK route has no nodal Schur/full-LU sub-path.
        code.push_str(&self.emit_header(ir, &glow_prov, None)?);
        code.push_str(&self.emit_constants(ir)?);
        code.push_str(&self.emit_pot_constants(ir));
        if noise.enabled {
            code.push_str(&noise.top_level);
        }
        code.push_str(&self.emit_state(ir, &noise)?);
        code.push_str(&self.emit_device_models(ir)?);
        // (Stateful-device update() hooks are emitted inside emit_device_models,
        //  which both the DK and nodal generate paths call — single source.)
        // SM pot helpers (sm_scale_N) removed — per-block rebuild replaces SM
        code.push_str(&self.emit_build_rhs(ir, &noise)?);
        code.push_str(&self.emit_mat_vec_mul_s(ir)?);
        code.push_str(&self.emit_extract_voltages(ir)?);
        self.generate_solve_nonlinear(&mut code, ir)?;
        code.push_str(&self.emit_final_voltages(ir)?);
        code.push_str(&self.emit_update_history()?);
        code.push_str(&self.emit_process_sample(ir, &noise)?);

        if super::runtime_os::runtime(ir).is_some() {
            code.push_str(&Self::emit_runtime_oversampler(ir));
        } else if ir.solver_config.oversampling_factor > 1 {
            code.push_str(&Self::emit_oversampler(ir));
        } else if ir.solver_config.has_inject_or_tap() {
            // No oversampling, but `.inject`/`.tap` still emit a private
            // process_sample_inner; wrap it in the array-API public entry.
            code.push_str(&Self::emit_inject_wrapper_1x(ir));
        }

        Ok(code)
    }
}

// ============================================================================
// Template-based emission methods
// ============================================================================

impl RustEmitter {
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

        ctx.insert(
            "sample_rate",
            &format!("{:.1}", ir.solver_config.sample_rate),
        );
        ctx.insert("oversampling_factor", &ir.solver_config.oversampling_factor);
        ctx.insert("opamp_rail_consts", &super::helpers::opamp_rail_consts(ir));
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
        // Multi-input ports (M=0 only): see `insert_multi_input_ctx`.
        insert_multi_input_ctx(&mut ctx, ir);
        insert_inject_ctx(&mut ctx, ir);
        ctx.insert("has_dc_sources", &ir.has_dc_sources);

        // Named topology constants. Always inserted so the
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
        ctx.insert("named_pots", &named_const_entries(&ir.named_constants.pots));

        // NODE_NAMES parallel array + dc_op_by_name lookup.
        // `node_names_values` is the `[&str; N]` body; `has_dc_op` gates the
        // lookup fn (needs the DC_OP const, emitted in state.rs.tera).
        ctx.insert(
            "node_names_values",
            &super::helpers::node_names_array_body(ir),
        );
        ctx.insert("dc_op_by_name_fn", super::helpers::DC_OP_BY_NAME_FN);
        ctx.insert("has_dc_op", &ir.has_dc_op);

        // Runtime voltage sources (.runtime directive). Always insert the list
        // (possibly empty) so state.rs.tera and build_rhs.rs.tera can use
        // `runtime_sources | length > 0` guards unconditionally.
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // WARMUP_SAMPLES_RECOMMENDED: 5τ_max in host-rate samples, rounded
        // up, minimum 1. Plugins driving per-instance parameter
        // jitter (e.g. SeriesOfTubes) use this to size the silent warmup loop.
        ctx.insert(
            "warmup_samples_recommended",
            &recommended_warmup_samples(ir),
        );
        // Companion flag: true when the value above hit the sanity cap (upper
        // bound, not a measured settle). Never launder a capped estimate into a
        // plausible number silently (oomox 2026-08-15).
        ctx.insert("warmup_estimate_capped", &warmup_estimate_capped(ir));

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

        // K_eff: pre-subtract parasitic-BJT R_p so the controlling-voltage map
        // v_d = p + state.k * i feeds bjt_evaluate the *internal* junction
        // voltage. Replaces the inner-NR cost of bjt_with_parasitics. R_p is
        // exact (linear in i), so K_eff has the same fixed point.
        ctx.insert(
            "k_rows",
            &format_matrix_rows(m, m, |i, j| ir.k(i, j) - parasitic_r_p_dk(ir, i, j)),
        );
        ctx.insert("n_v_rows", &format_matrix_rows(m, n, |i, j| ir.n_v(i, j)));
        // N_i in operator shape: N_I[node][device] = n_i[node][device].
        // Normalized to N x M to match the nodal emitter — one layout for one
        // public symbol (see the `N_I` doc comment in constants.rs.tera).
        ctx.insert("n_i_rows", &format_matrix_rows(n, m, |i, j| ir.n_i(i, j)));

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
            // BE fallback K also gets the K_eff treatment — same parasitic
            // BJT absorption applies because the BE NR uses the same K-mediated
            // controlling-voltage map.
            ctx.insert(
                "k_be_rows",
                &format_matrix_rows(m, m, |i, j| ir.k_be(i, j) - parasitic_r_p_dk(ir, i, j)),
            );
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

            // RHS_CONST_BE (backward Euler: DC sources x1).
            //
            // Guard on `has_dc_sources` ONLY — must stay symmetric with the
            // template pair: constants.rs.tera emits the constant under
            // `{% if has_dc_sources %}` (inside `has_be_fallback`) and
            // process_sample.rs.tera references it under the same condition.
            // A short/empty `rhs_const_be` vec is padded with zeros below so
            // the constant is always well-formed when the guard fires.
            if ir.has_dc_sources {
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
                        label: sw.label.clone(),
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
        let dc_block_r = 1.0
            - 2.0 * std::f64::consts::PI * crate::codegen::policy::DC_BLOCK_CUTOFF_HZ
                / internal_rate;
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

        ctx.insert("runtime_os_consts", &super::runtime_os::emit_consts(ir));
        self.render("constants", &ctx)
    }

    fn emit_state(&self, ir: &CircuitIR, noise: &NoiseEmission) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        // Gates the unsolved-sample counter (no Newton solve at M = 0).
        ctx.insert("m", &ir.topology.m);
        // A reduced device's region exit is an unsolved sample.
        ctx.insert(
            "has_reduced_device",
            &super::helpers::has_reduced_device(ir),
        );
        insert_multi_input_ctx(&mut ctx, ir);
        insert_inject_ctx(&mut ctx, ir);
        // Noise fragments (empty strings when noise is off → template blocks become no-ops)
        ctx.insert("noise_enabled_emit", &noise.enabled);
        ctx.insert("noise_state_fields", &noise.state_fields);
        ctx.insert("noise_default_stmts", &noise.default_stmts);
        ctx.insert("noise_default_fields", &noise.default_fields);
        ctx.insert("noise_reset_body", &noise.reset_body);
        ctx.insert("noise_set_sample_rate_body", &noise.set_sample_rate_body);
        ctx.insert("noise_methods", &noise.methods);
        // Stateful-device opaque state block (Phase 0c). Empty strings when no
        // device is stateful → template blocks are no-ops and the deck is
        // byte-identical. NaN-recovery + after-solve update live in
        // process_sample.rs.tera (emit_process_sample), not here.
        let stateful_devs = stateful_device_data(ir);
        let has_stateful = !stateful_devs.is_empty();
        ctx.insert("has_stateful", &has_stateful);
        ctx.insert(
            "stateful_state_fields",
            &emit_stateful_state_fields(&stateful_devs),
        );
        ctx.insert(
            "stateful_default_fields",
            &emit_stateful_default_fields(&stateful_devs),
        );
        ctx.insert(
            "stateful_reset_body",
            &emit_stateful_state_restore(&stateful_devs, "self."),
        );
        ctx.insert(
            "stateful_set_sample_rate_body",
            &emit_stateful_set_sample_rate_body(ir, &stateful_devs),
        );
        ctx.insert("has_dc_op", &ir.has_dc_op);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("n_aug", &ir.topology.n_aug);
        ctx.insert("n_nodes", &ir.topology.n_nodes);
        let num_pots = ir.pots.len();
        ctx.insert("num_pots", &num_pots);
        let num_outputs = ir.solver_config.output_nodes.len();
        ctx.insert("num_outputs", &num_outputs);

        let os_factor = ir.solver_config.oversampling_factor;
        ctx.insert("oversampling_factor", &os_factor);
        // A runtime-oversampling build keeps its filter states per factor
        // (`runtime_os`), not in the fixed build's `os_*` fields.
        let runtime_os = super::runtime_os::runtime(ir).is_some();
        ctx.insert("runtime_os", &runtime_os);
        ctx.insert("os_fixed_states", &(os_factor > 1 && !runtime_os));
        ctx.insert(
            "os_factor_f64",
            &super::runtime_os::factor_f64_literal(ir, "self"),
        );
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            ctx.insert("os_state_size", &os_info.state_size);
            ctx.insert("oversampling_4x", &(os_factor == 4 && !runtime_os));
            if os_factor == 4 {
                ctx.insert("os_state_size_outer", &os_info.state_size_outer);
            }
        } else {
            ctx.insert("oversampling_4x", &false);
        }

        let pot_defaults: Vec<String> =
            ir.pots.iter().map(|p| fmt_f64(1.0 / p.g_nominal)).collect();
        ctx.insert("pot_defaults", &pot_defaults);

        if ir.has_dc_op {
            let dc_op_values = ir
                .dc_operating_point
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("dc_op_values", &dc_op_values);
        }

        // IC=-bearing capacitors: initial-state seed for `v_prev` (see
        // `docs/aidocs/DC_OP.md` "IC= initial condition"). Independent of
        // `has_dc_op` — a circuit with no DC sources but an IC= cap still
        // needs this constant.
        let has_cap_ic = ir.v_prev_ic_seed.is_some();
        ctx.insert("has_cap_ic", &has_cap_ic);
        if let Some(v_prev_ic) = &ir.v_prev_ic_seed {
            let v_prev_ic_values = v_prev_ic
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("v_prev_ic_values", &v_prev_ic_values);
        }
        ctx.insert("carries_q_dot", &carries_q_dot(ir));
        ctx.insert("q_dot_start", q_dot_start(ir));
        ctx.insert("has_q_dot_ic_seed", &ir.q_dot_ic_seed.is_some());
        if let Some(q) = &ir.q_dot_ic_seed {
            let values = q.iter().map(|v| fmt_f64(*v)).collect::<Vec<_>>().join(", ");
            ctx.insert("q_dot_ic_seed_values", &values);
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

        // DC nonlinear currents at the IC-seeded operating point — paired
        // ONLY with V_PREV_IC_SEED (never with the plain DC_OP/DC_NL_I pair
        // used by the reset fallback). See the pairing comment on
        // `dc_nl_currents_ic_seed` in `codegen/ir/mod.rs`.
        let has_dc_nl_ic_seed = ir.topology.m > 0 && ir.dc_nl_currents_ic_seed.is_some();
        ctx.insert("has_dc_nl_ic_seed", &has_dc_nl_ic_seed);
        if has_dc_nl_ic_seed {
            let dc_nl_i_ic_seed_values = ir
                .dc_nl_currents_ic_seed
                .as_ref()
                .unwrap()
                .iter()
                .map(|v| fmt_f64(*v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("dc_nl_i_ic_seed_values", &dc_nl_i_ic_seed_values);
        }

        // Switch data
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);
        // Always provide switch_indices (empty when no switches) so template can iterate safely
        let switch_indices: Vec<usize> = (0..num_switches).collect();
        ctx.insert("switch_indices", &switch_indices);
        // Generate pot/switch methods procedurally (rebuild_matrices, set_pot_N, set_switch_N)
        if num_switches > 0 || num_pots > 0 {
            let switch_methods = self.emit_switch_methods(ir, noise)?;
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

        // The DC-block cutoff as an emitted literal, so state.rs.tera spells the
        // formula from the same source of truth the codegen-time coefficient uses.
        ctx.insert(
            "dc_block_cutoff_hz",
            &crate::codegen::policy::dc_block_cutoff_hz_literal(),
        );
        ctx.insert("dc_block", &ir.dc_block);

        // `current_sample_rate` is read by rebuild_matrices (pots/switches),
        // by the op-amp slew-rate limiter's per-sample dt, AND by the device
        // self-heating thermal update's dt (all in process_sample.rs.tera /
        // emit_thermal_tj_advance). Emit the field whenever any consumer
        // exists — mirrors the nodal-emitter fix (7e32bf8): a DK circuit
        // with a finite-SR op-amp (or a thermal device) and no pots would
        // otherwise fail rustc.
        let needs_current_sr = num_pots > 0
            || num_switches > 0
            || ir.opamps.iter().any(|oa| oa.sr.is_finite())
            || num_thermal_devices > 0
            || has_stateful;
        // `set_oversampling` rebuilds at the current host rate.
        let needs_current_sr = needs_current_sr || super::runtime_os::runtime(ir).is_some();
        ctx.insert("needs_current_sr", &needs_current_sr);
        ctx.insert(
            "runtime_os_fields",
            &format!(
                "{}{}",
                super::runtime_os::state_field_decl(ir),
                super::runtime_os::os_state_fields(ir)
            ),
        );
        ctx.insert(
            "runtime_os_inits",
            &format!(
                "{}{}",
                super::runtime_os::state_field_init(ir),
                super::runtime_os::os_state_inits(ir)
            ),
        );
        ctx.insert("runtime_os_methods", &super::runtime_os::emit_methods(ir));

        // Backward Euler fallback state fields (for BE fallback in DK NR solver)
        let has_be_fallback = !ir.matrices.s_be.is_empty() && ir.topology.m > 0;
        ctx.insert("has_be_fallback", &has_be_fallback);
        ctx.insert("backward_euler", &ir.solver_config.backward_euler);

        // Runtime voltage sources (.runtime directive). Same list is used by
        // state.rs.tera for field + default + reset emission; build_rhs.rs.tera
        // emits the per-sample `rhs[row] += state.<field>` stamps.
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // Named nodes for the dc_op_dump() pretty printer.
        ctx.insert(
            "named_nodes",
            &named_const_entries(&ir.named_constants.nodes),
        );

        // K_eff twin for the no-pot set_sample_rate rebuild (template body).
        // The baked K_DEFAULT/K_BE_DEFAULT and the pot-path rebuild_matrices
        // both absorb parasitic-BJT RB/RC/RE into K; a genuine rate change
        // through the template body must apply the same absorption or the
        // parasitics are electrically deleted at any non-codegen host rate.
        // The fragments render byte-neutral (empty string, no extra lines)
        // when no slot qualifies — which is every shipped circuit today.
        let k_eff_stmts = k_eff_adjust_stmts(ir, "k", "        ");
        let k_eff_fragment = if k_eff_stmts.is_empty() {
            String::new()
        } else {
            format!(
                "\n\n        // K_eff: absorb parasitic-BJT R drops into K so v_d = p + state.k * i\n\
                 \x20       // gives the internal junction voltage directly. Lets bjt_evaluate\n\
                 \x20       // (intrinsic) replace the inner-NR cost of bjt_with_parasitics.\n{}",
                k_eff_stmts.trim_end_matches('\n')
            )
        };
        ctx.insert("k_eff_adjust_lines", &k_eff_fragment);
        let k_be_eff_stmts = k_eff_adjust_stmts(ir, "k_be", "        ");
        let k_be_eff_fragment = if k_be_eff_stmts.is_empty() {
            String::new()
        } else {
            format!(
                "\n        // K_eff for the BE kernel (same parasitic-BJT absorption as trap K)\n{}",
                k_be_eff_stmts.trim_end_matches('\n')
            )
        };
        ctx.insert("k_be_eff_adjust_lines", &k_be_eff_fragment);

        // Runtime DC operating point recompute. When this flag is on the
        // template renders `recompute_dc_op` with the body built by
        // `dc_op_emitter::emit_recompute_dc_op_body_dk`.
        let emit_dc_op_recompute = ir.solver_config.emit_dc_op_recompute;
        ctx.insert("emit_dc_op_recompute", &emit_dc_op_recompute);
        if emit_dc_op_recompute {
            let body = super::dc_op_emitter::emit_recompute_dc_op_body_dk(ir)?;
            ctx.insert("recompute_dc_op_body", &body);
            ctx.insert(
                "settle_dc_op_body",
                &super::dc_op_emitter::emit_settle_dc_op_body(ir),
            );
        }

        // #1 Step-6c damping: cache the damping threshold in state instead of
        // recomputing max|dc_operating_point| every sample. `damp_thresh_init`
        // is the has_dc_op-branch value (dc_operating_point == DC_OP at Default/
        // reset); the has_dc_op-false branch is a literal 2.0 (max of zeros).
        // Computed with the exact same fold + mul_add(0.05, 2.0) as the old
        // per-sample loop so the cached value is byte-identical. n_nodes is also
        // needed by the refresh_damp_thresh() helper's loop bound.
        let n_nodes = ir.topology.n_nodes;
        ctx.insert("n_nodes", &n_nodes);
        let damp_thresh_init = ir
            .dc_operating_point
            .iter()
            .take(n_nodes)
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max)
            .mul_add(0.05, 2.0);
        ctx.insert("damp_thresh_init", &format!("{:.17e}", damp_thresh_init));

        let mut state = self.render("state", &ctx)?;
        // Restores (`X = NAME;`) load the running factor's baked values; the
        // constructor's `NAME,` initializers keep the default factor's.
        super::runtime_os::switch_restores(ir, &mut state, 0);
        Ok(state)
    }

    /// Generate switch setter methods and rebuild_matrices() procedurally.
    fn emit_switch_methods(
        &self,
        ir: &CircuitIR,
        noise: &NoiseEmission,
    ) -> Result<String, CodegenError> {
        let m = ir.topology.m;
        let num_pots = ir.pots.len();
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
                let mut out = String::new();
                if let Some(slots) = noise.switch_comp_to_noise_slot.get(idx) {
                    for (ci, maybe_slot) in slots.iter().enumerate() {
                        if let Some(slot) = maybe_slot {
                            out.push_str(&format!(
                                "        self.noise_thermal_sqrt_inv_r[{slot}] = (1.0 / SWITCH_{idx}_VALUES[position][{ci}]).sqrt();\n",
                            ));
                        }
                    }
                }
                if let Some(slots) = noise.switch_comp_to_r_flicker_slot.get(idx) {
                    for (ci, maybe_slot) in slots.iter().enumerate() {
                        if let Some(slot) = maybe_slot {
                            out.push_str(&format!(
                                "        self.noise_r_flicker_inv_r[{slot}] = 1.0 / SWITCH_{idx}_VALUES[position][{ci}];\n",
                            ));
                        }
                    }
                }
                // Phase 4: switch R at an op-amp in+ shifts the en Norton
                // conversion factor — absolute recompute (previously the
                // switch hook was missing entirely; only pots refreshed).
                // Runs after `self.switch_N_position = position`.
                if noise
                    .switch_to_opamp_en_refresh
                    .get(idx)
                    .copied()
                    .unwrap_or(false)
                {
                    out.push_str("        self.refresh_opamp_en_g_diag();\n");
                }
                out
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
                sw.index,
                sw.index,
                sw.index,
                sw.index,
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
            // Phase 4: op-amp `en_g_diag` refresh follows the same hook
            // for pots that touch an op-amp's non-inverting input.
            let opamp_refresh_targets: &[usize] = noise
                .pot_to_opamp_en_refresh
                .get(idx)
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            let noise_update: String = if noise.enabled {
                let mut out = String::new();
                if let Some(slot) = noise.pot_to_noise_slot.get(idx).copied().flatten() {
                    out.push_str(&format!(
                        "        self.noise_thermal_sqrt_inv_r[{slot}] = (1.0 / r).sqrt();\n",
                    ));
                }
                if let Some(slot) = noise.pot_to_r_flicker_slot.get(idx).copied().flatten() {
                    out.push_str(&format!(
                        "        self.noise_r_flicker_inv_r[{slot}] = 1.0 / r;\n",
                    ));
                }
                // Phase 4: op-amp en_g_diag refresh for pots touching an
                // op-amp in+. Absolute recompute (BASE + Σ live dynamic G) —
                // see refresh_opamp_en_g_diag; runs after the new resistance
                // is stored so it reads the fresh value.
                if !opamp_refresh_targets.is_empty() {
                    out.push_str("        self.refresh_opamp_en_g_diag();\n");
                }
                out
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
            code.push_str("        let alpha = internal_rate; // backward Euler: alpha = 1/T\n");
        } else {
            code.push_str("        let alpha = 2.0 * internal_rate; // trapezoidal: alpha = 2/T\n");
        }

        // Start from constant G, C
        let has_r_switch = ir
            .switches
            .iter()
            .any(|sw| sw.components.iter().any(|c| c.component_type == 'R'));
        // A C switch stamps `c_eff`; so does an L switch on the augmented
        // kernel, whose inductance sits on its branch row's diagonal of C.
        let has_c_switch = ir.switches.iter().any(|sw| {
            sw.components.iter().any(|c| {
                c.component_type == 'C' || (c.component_type == 'L' && c.augmented_row.is_some())
            })
        });
        let g_mut = if has_r_switch || num_pots > 0 {
            "mut "
        } else {
            ""
        };
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
                        // Augmented MNA: L value lives on diagonal of branch variable row.
                        // Every generated inductor is a branch row (`build_dk` refuses a
                        // companion-model kernel); an L component without one has nothing
                        // the switch could change, so refuse rather than emit a switch
                        // that silently does nothing.
                        let Some(aug_row) = comp.augmented_row else {
                            return Err(CodegenError::InvalidConfig(format!(
                                ".switch '{}': inductor {} has no branch row in this build, so \
                                 the switch cannot change it.",
                                sw.label, comp.name
                            )));
                        };
                        code.push_str(&format!(
                            "            let delta_l = new_val - {};\n\
                             \x20           c_eff[{}][{}] += delta_l;\n",
                            nominal, aug_row, aug_row,
                        ));
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
                        "            g_eff[{}][{}] += delta_g;\n",
                        np - 1,
                        np - 1
                    ));
                }
                if nq > 0 {
                    code.push_str(&format!(
                        "            g_eff[{}][{}] += delta_g;\n",
                        nq - 1,
                        nq - 1
                    ));
                }
                if np > 0 && nq > 0 {
                    code.push_str(&format!(
                        "            g_eff[{}][{}] -= delta_g;\n\
                         \x20           g_eff[{}][{}] -= delta_g;\n",
                        np - 1,
                        nq - 1,
                        nq - 1,
                        np - 1
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
        // The history matrix is alpha*C under both integrators (charge form).
        code.push_str(
            "        // Build A_neg = alpha*C (charge-form history, no G term)\n\
             \x20       let mut a_neg = [[0.0f64; N]; N];\n\
             \x20       for i in 0..N {\n\
             \x20           for j in 0..N {\n\
             \x20               a_neg[i][j] = alpha * c_eff[i][j];\n\
             \x20           }\n\
             \x20       }\n",
        );
        // Zero augmented rows in A_neg (algebraic constraints for VS/VCVS),
        // not inductor rows.
        for (lo, hi) in history_zero_row_ranges(ir) {
            code.push_str(&format!(
                "        // Zero VS/VCVS algebraic rows in A_neg (NOT inductor rows)\n\
                 \x20       for i in {lo}..{hi} {{\n\
                 \x20           for j in 0..N {{\n\
                 \x20               a_neg[i][j] = 0.0;\n\
                 \x20           }}\n\
                 \x20       }}\n"
            ));
        }

        // Invert A → S, compute S_NI, K. Use the equilibrated invert so
        // partial-pivoting LU keeps cond(A) inside f64 precision even when
        // G_in (≈1 S) dominates internal conductances (1e-4 to 1e-6 S) by
        // 4-6 decades.
        code.push_str(&format!(
            "\n\
             \x20       // Invert A to get S (asymmetric row/column equilibration)\n\
             \x20       let (s, singular) = invert_n_equilibrated(a);\n\
             \x20       if singular {{ self.diag_singular_matrix_count += 1; }}\n\n\
             \x20       // Compute S * N_i product (N x M)\n\
             \x20       let mut s_ni = [[0.0f64; M]; N];\n\
             \x20       for i in 0..N {{\n\
             \x20           for j in 0..M {{\n\
             \x20               let mut sum = 0.0;\n\
             \x20               for kk in 0..N {{\n\
             \x20                   sum += s[i][kk] * N_I[kk][j];\n\
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
             \x20       }}\n",
            m = m,
        ));

        // K_eff: subtract parasitic-BJT R_p from each affected 2x2 block.
        // Mirrors the codegen-time K_DEFAULT adjustment, so state.k holds
        // K_eff after both default-init and any pot/switch rebuild.
        // Gated on slot.dimension == 2: FA-reduced (1D) BJTs ignore parasitics
        // by design (see ir.rs::detect_forward_active_bjts comment), and their
        // start_idx+1 lands in the next device's slot, not the same BJT's Vbc row.
        let k_eff_stmts = k_eff_adjust_stmts(ir, "k", "        ");
        if !k_eff_stmts.is_empty() {
            code.push_str(
                "\n        // K_eff: absorb parasitic-BJT R drops into K so v_d = p + state.k * i\n\
                 \x20       // gives the internal junction voltage directly. Lets bjt_evaluate\n\
                 \x20       // (intrinsic) replace the inner-NR cost of bjt_with_parasitics.\n",
            );
            code.push_str(&k_eff_stmts);
        }

        code.push_str(
            "\n        self.s = s;\n\
             \x20       self.a_neg = a_neg;\n\
             \x20       self.k = k;\n\
             \x20       self.s_ni = s_ni;\n",
        );

        // Rebuild the backward-Euler fallback matrix set from the same
        // g_eff/c_eff. Without this, s_be/k_be/a_neg_be/s_ni_be go stale on
        // every pot/switch move — and permanently wrong after
        // set_sample_rate (which delegates here on pot/switch circuits, so
        // the no-pot template's BE rebuild never runs). The BE fallback
        // fires on exactly the stressed samples, so a stale BE set corrupts
        // the samples that most need it. Mirrors the no-pot
        // set_sample_rate body in state.rs.tera.
        let has_be_fallback = !ir.matrices.s_be.is_empty() && ir.topology.m > 0;
        if has_be_fallback {
            code.push_str(
                "\n        // Rebuild backward-Euler fallback matrices (stale BE set\n\
                 \x20       // would corrupt exactly the stressed samples that trigger\n\
                 \x20       // the fallback)\n\
                 \x20       let alpha_be = internal_rate; // 1/T for backward Euler\n\
                 \x20       let mut a_be = [[0.0f64; N]; N];\n\
                 \x20       for i in 0..N {\n\
                 \x20           for j in 0..N {\n\
                 \x20               a_be[i][j] = g_eff[i][j] + alpha_be * c_eff[i][j];\n\
                 \x20           }\n\
                 \x20       }\n\
                 \x20       let mut a_neg_be = [[0.0f64; N]; N];\n\
                 \x20       for i in 0..N {\n\
                 \x20           for j in 0..N {\n\
                 \x20               a_neg_be[i][j] = alpha_be * c_eff[i][j];\n\
                 \x20           }\n\
                 \x20       }\n",
            );
            for (lo, hi) in history_zero_row_ranges(ir) {
                code.push_str(&format!(
                    "        // Zero VS/VCVS algebraic rows in A_neg_be (NOT inductor rows)\n\
                     \x20       for i in {lo}..{hi} {{\n\
                     \x20           for j in 0..N {{\n\
                     \x20               a_neg_be[i][j] = 0.0;\n\
                     \x20           }}\n\
                     \x20       }}\n"
                ));
            }
            code.push_str(
                "        let (s_be, singular_be) = invert_n_equilibrated(a_be);\n\
                 \x20       if singular_be { self.diag_singular_matrix_count += 1; }\n\
                 \x20       let mut s_ni_be = [[0.0f64; M]; N];\n\
                 \x20       for i in 0..N {\n\
                 \x20           for j in 0..M {\n\
                 \x20               let mut sum = 0.0;\n\
                 \x20               for kk in 0..N {\n\
                 \x20                   sum += s_be[i][kk] * N_I[kk][j];\n\
                 \x20               }\n\
                 \x20               s_ni_be[i][j] = sum;\n\
                 \x20           }\n\
                 \x20       }\n\
                 \x20       let mut k_be = [[0.0f64; M]; M];\n\
                 \x20       for i in 0..M {\n\
                 \x20           for j in 0..M {\n\
                 \x20               let mut sum = 0.0;\n\
                 \x20               for n_idx in 0..N {\n\
                 \x20                   sum += N_V[i][n_idx] * s_ni_be[n_idx][j];\n\
                 \x20               }\n\
                 \x20               k_be[i][j] = sum;\n\
                 \x20           }\n\
                 \x20       }\n",
            );
            // K_eff: same parasitic-BJT absorption as the trap K above —
            // K_BE_DEFAULT is emitted with this adjustment, so the rebuild
            // must apply it too.
            let k_be_eff_stmts = k_eff_adjust_stmts(ir, "k_be", "        ");
            if !k_be_eff_stmts.is_empty() {
                code.push_str(
                    "        // K_eff for the BE kernel (same parasitic-BJT absorption as trap K)\n",
                );
                code.push_str(&k_be_eff_stmts);
            }
            code.push_str(
                "        self.s_be = s_be;\n\
                 \x20       self.k_be = k_be;\n\
                 \x20       self.s_ni_be = s_ni_be;\n\
                 \x20       self.a_neg_be = a_neg_be;\n",
            );
        }

        // SM pot recomputation removed — per-block rebuild handles pots exactly

        // Intentionally do NOT reset oversampler state or DC blocker state here.
        // rebuild_matrices() is called on every pot/switch change, but pot/switch
        // changes do not invalidate filter history. Zeroing os_up_state/os_dn_state
        // on every knob move causes an audible click from the half-band filter
        // ringing through; zeroing dc_block_x_prev/y_prev produces an instant DC
        // step and multi-second HPF re-settle. The `dc_block_r` coefficient depends
        // only on the sample rate, not on pots/switches, so it is not recomputed
        // here — set_sample_rate recomputes it on a genuine rate change (in BOTH
        // the pot/switch and no-pot template variants) and restores DC_BLOCK_R on
        // same-rate calls. (Legitimate filter resets happen only on a genuine
        // rate change in set_sample_rate(), plus reset()'s blocker reseed.)

        code.push_str("    }\n");
        Ok(code)
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

    fn emit_build_rhs(
        &self,
        ir: &CircuitIR,
        _noise: &NoiseEmission,
    ) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let mut ctx = Context::new();

        ctx.insert("has_dc_sources", &ir.has_dc_sources);
        insert_multi_input_ctx(&mut ctx, ir);
        insert_inject_ctx(&mut ctx, ir);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("backward_euler", &ir.solver_config.backward_euler);
        // Runtime voltage sources: emit `rhs[row] += state.<field>` per entry
        // after the input stamp. Safe on both trapezoidal and BE paths — the
        // field is the raw per-sample value, not rate-dependent.
        ctx.insert("runtime_sources", &ir.runtime_sources);

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

        // Charge form: the carried q_dot on every row that carries history.
        ctx.insert("carries_q_dot", &carries_q_dot(ir));
        if carries_q_dot(ir) {
            let mut q_dot_lines = String::new();
            for i in 0..n {
                if !ir.sparsity.a_neg.nz_by_row[i].is_empty() {
                    q_dot_lines.push_str(&format!("    rhs[{i}] += state.q_dot[{i}];\n"));
                }
            }
            ctx.insert("q_dot_lines", &q_dot_lines);
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

    fn emit_process_sample(
        &self,
        ir: &CircuitIR,
        noise: &NoiseEmission,
    ) -> Result<String, CodegenError> {
        // Noise fragment is built once by the caller (emit_dk) and threaded in
        // — build_noise_emission resolves every noise source and assembles ~500
        // lines of strings, so re-deriving it here doubled that codegen work.
        let mut ctx = Context::new();
        insert_multi_input_ctx(&mut ctx, ir);
        insert_inject_ctx(&mut ctx, ir);
        ctx.insert("noise_enabled_emit", &noise.enabled);
        ctx.insert("noise_rhs_stamp", &noise.rhs_stamp);
        ctx.insert("noise_rhs_stamp_be", &noise.rhs_stamp_be);
        ctx.insert("noise_nan_recovery", &noise.nan_recovery_body);
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
        ctx.insert("carries_q_dot", &carries_q_dot(ir));
        if carries_q_dot(ir) {
            ctx.insert("q_dot_commit", &dk_q_dot_commit(ir, has_be_fallback));
        }
        // #P1: sparse-prune the BE-fallback matvecs. rhs_be = A_neg_be·v_prev
        // (+ RHS_CONST_BE) and p_be = N_V·v_pred_be are all structurally
        // sparse; S_be / S_ni_be are dense inverses and stay looped in the
        // template. Skipped entries are exactly zero (A_neg_be uses its OWN
        // pattern, not a_neg's, to avoid the αC−G near-cancellation trap).
        //
        // No N_I·i_nl_prev term: a BE step stamps only the nonlinear current at
        // the NEW sample (S_ni_be·i_nl in the template). Adding i_nl_prev too
        // double-counts every device's bias current, so the fallback sample is
        // not a fixed point of the DC operating point and the excursion lands
        // in null(C) — trap's exact z=-1 eigenspace — where trap never damps
        // it (philicorda-voicing-coupled, 2026-09-14; same defect as the nodal
        // emitter's fallback, see its comment). The trap-primary build_rhs
        // above keeps its N_I·i_nl_prev half — that is the trap average split
        // across build_rhs and compute_final_voltages, a different scheme.
        let (be_rhs_lines, be_p_lines) = if has_be_fallback {
            let mut rhs = String::new();
            for i in 0..ir.topology.n {
                let mut terms: Vec<String> = Vec::new();
                if ir.has_dc_sources {
                    terms.push(format!("RHS_CONST_BE[{i}]"));
                }
                for &j in &ir.sparsity.a_neg_be.nz_by_row[i] {
                    terms.push(format!("state.a_neg_be[{i}][{j}] * state.v_prev[{j}]"));
                }
                if terms.is_empty() {
                    rhs.push_str(&format!("        rhs_be[{i}] = 0.0;\n"));
                } else {
                    rhs.push_str(&format!("        rhs_be[{i}] = {};\n", terms.join(" + ")));
                }
            }
            let mut p = String::new();
            for i in 0..ir.topology.m {
                let terms: Vec<String> = ir.sparsity.n_v.nz_by_row[i]
                    .iter()
                    .map(|&j| format!("N_V[{i}][{j}] * v_pred_be[{j}]"))
                    .collect();
                if terms.is_empty() {
                    p.push_str(&format!("        p_be[{i}] = 0.0;\n"));
                } else {
                    p.push_str(&format!("        p_be[{i}] = {};\n", terms.join(" + ")));
                }
            }
            (rhs, p)
        } else {
            (String::new(), String::new())
        };
        ctx.insert("be_rhs_lines", &be_rhs_lines);
        ctx.insert("be_p_lines", &be_p_lines);
        ctx.insert("has_dc_sources", &ir.has_dc_sources);
        ctx.insert("max_iter", &ir.solver_config.max_iterations);
        let os_factor = ir.solver_config.oversampling_factor;
        ctx.insert("oversampling_factor", &os_factor);
        // A runtime-oversampling build keeps its filter states per factor
        // (`runtime_os`), not in the fixed build's `os_*` fields.
        let runtime_os = super::runtime_os::runtime(ir).is_some();
        ctx.insert("runtime_os", &runtime_os);
        ctx.insert("os_fixed_states", &(os_factor > 1 && !runtime_os));
        ctx.insert(
            "os_factor_f64",
            &super::runtime_os::factor_f64_literal(ir, "self"),
        );
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            ctx.insert("os_state_size", &os_info.state_size);
            ctx.insert("oversampling_4x", &(os_factor == 4 && !runtime_os));
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

        // Output clamp bound (CodegenConfig::output_clamp_v, default ±10 V).
        // Used for the final output clamp, the diag_clamp_count threshold,
        // and the NaN-recovery return path. High-voltage circuits (e.g. the
        // Wurlitzer power amp at ±30 V) set this above the default; the
        // nodal emitter already honors it (`nodal_emitter/`: full_lu.rs,
        // schur.rs, reset.rs, state.rs) — this threads
        // it into the DK template as well.
        ctx.insert(
            "output_clamp_v",
            &format!("{:e}", ir.solver_config.output_clamp_v),
        );

        ctx.insert("max_iter", &ir.solver_config.max_iterations);
        ctx.insert("m", &ir.topology.m);
        ctx.insert(
            "region_exit_lines",
            &super::helpers::emit_region_exit_lines(ir, "    "),
        );

        let num_pots = ir.pots.len();
        ctx.insert("num_pots", &num_pots);
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);

        // Runtime voltage sources (.runtime directive): the BE-fallback RHS
        // in process_sample.rs.tera stamps the same rows/values as build_rhs
        // (VS algebraic rows are integration-scheme-independent).
        ctx.insert("runtime_sources", &ir.runtime_sources);

        // Per-sample SM pot corrections removed — pot changes are handled by
        // per-block rebuild_matrices (Batch D). No sm_scale_lines /
        // a_neg_correction / s_correction / sni_correction context vars are
        // emitted; the templates never referenced them.

        // MOSFET body effect: VT is re-evaluated inside solve_nonlinear at every
        // Newton iterate (V(source), V(bulk) = v_pred + S_NI·i_nl), with gmb in
        // the Jacobian; solve_nonlinear then needs v_pred.
        let dk_body_effect = ir.solver_mode == crate::codegen::ir::SolverMode::Dk
            && !super::helpers::body_effect_mosfets(ir).is_empty();
        ctx.insert("dk_body_effect", &dk_body_effect);

        // Device self-heating thermal update (after NR, before state save).
        // Runs for each BJT/diode whose `.model` sets a finite RTH. Uses the
        // converged `i_nl` and node voltages from this sample to compute P
        // and advances Tj by one exact-exponential step of the RC thermal
        // ODE (see `emit_thermal_tj_advance`).
        let mut thermal_update = String::new();
        // #P4: extract_controlling_voltages(&v) (= N_v·v) is identical for every
        // self-heating device — a function of the converged v only. Emit it once
        // here rather than re-extracting inside each device's thermal block.
        let has_self_heating = ir.device_slots.iter().any(|slot| match &slot.params {
            DeviceParams::Bjt(bp) => bp.has_self_heating() && slot.device_type == DeviceType::Bjt,
            DeviceParams::Diode(dp) => dp.has_self_heating(),
            DeviceParams::Tube(tp) => tp.has_self_heating(),
            _ => false,
        });
        if has_self_heating {
            thermal_update.push_str("    let v_nl_th = extract_controlling_voltages(&v);\n");
        }
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            match &slot.params {
                // The `device_type == Bjt` guard (not BjtForwardActive) is
                // belt-and-braces against slot aliasing: this arm reads the
                // 2D slot pair (i_nl[s], i_nl[s+1]); on a 1D FA-reduced slot
                // s+1 would be the NEXT device's slot (or out of bounds).
                // detect_forward_active_bjts excludes self-heating BJTs from
                // FA reduction, so this guard should never fire — but if that
                // gating ever regresses, aliasing stays structurally
                // impossible here.
                DeviceParams::Bjt(bp)
                    if bp.has_self_heating() && slot.device_type == DeviceType::Bjt =>
                {
                    let s = slot.start_idx;
                    let s1 = s + 1;
                    let tj_advance = emit_thermal_tj_advance(dev_num, bp.cth);
                    // Extract Ic, Ib from converged i_nl; compute Vbe, Vbc from final v
                    thermal_update.push_str(&format!(
                            "    {{ // BJT {dev_num} self-heating thermal update\n\
                             \x20       let ic = i_nl[{s}];\n\
                             \x20       let ib = i_nl[{s1}];\n\
                             \x20       let vbe = v_nl_th[{s}];\n\
                             \x20       let vbc = v_nl_th[{s1}];\n\
                             \x20       let vce = vbe - vbc;\n\
                             \x20       let p = vce * ic + vbe * ib;\n\
                             {tj_advance}\
                             \x20       state.device_{dev_num}_vt = BOLTZMANN_Q * state.device_{dev_num}_tj;\n\
                             \x20       let t_ratio = state.device_{dev_num}_tj / DEVICE_{dev_num}_TAMB;\n\
                             \x20       let vt_nom = BOLTZMANN_Q * DEVICE_{dev_num}_TAMB;\n\
                             \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS_NOM\n\
                             \x20           * t_ratio.powf(DEVICE_{dev_num}_XTI)\n\
                             \x20           * fast_exp((DEVICE_{dev_num}_EG / vt_nom) * (1.0 - DEVICE_{dev_num}_TAMB / state.device_{dev_num}_tj));\n\
                             \x20       // Beta temperature dependence (SPICE XTB). XTB defaults to 0.0,\n\
                             \x20       // so `powf` returns exactly 1.0 and this is inert on cards that\n\
                             \x20       // omit it — but a self-heating device whose beta never moved was\n\
                             \x20       // physically wrong, not merely incomplete.\n\
                             \x20       state.device_{dev_num}_bf = DEVICE_{dev_num}_BETA_F * t_ratio.powf(DEVICE_{dev_num}_XTB);\n\
                             \x20       state.device_{dev_num}_br = DEVICE_{dev_num}_BETA_R * t_ratio.powf(DEVICE_{dev_num}_XTB);\n\
                             \x20   }}\n"
                        ));
                }
                DeviceParams::Diode(dp) if dp.has_self_heating() => {
                    let s = slot.start_idx;
                    let tj_advance = emit_thermal_tj_advance(dev_num, dp.cth);
                    // SPICE3f5 diode temperature scaling includes the emission
                    // coefficient N:
                    //   IS(T) = IS_NOM · (Tj/TAMB)^(XTI/N)
                    //                  · exp((EG/(N·vt_nom)) · (1 − TAMB/Tj))
                    // N is not stored separately in DiodeParams — recover it
                    // from n_vt = N · Vt(TAMB) (ir/mod.rs resolve_diode_params)
                    // and bake it into the emitted expression at codegen time.
                    let n_emission = dp.n_vt
                        / (melange_primitives::VT_ROOM * (dp.tamb / melange_primitives::T_NOM));
                    thermal_update.push_str(&format!(
                            "    {{ // Diode {dev_num} self-heating thermal update\n\
                             \x20       let id = i_nl[{s}];\n\
                             \x20       let vd = v_nl_th[{s}];\n\
                             \x20       let p = vd * id;\n\
                             {tj_advance}\
                             \x20       let t_ratio = state.device_{dev_num}_tj / DEVICE_{dev_num}_TAMB;\n\
                             \x20       state.device_{dev_num}_n_vt = DEVICE_{dev_num}_N_VT_NOM * t_ratio;\n\
                             \x20       let vt_nom = BOLTZMANN_Q * DEVICE_{dev_num}_TAMB;\n\
                             \x20       state.device_{dev_num}_is = DEVICE_{dev_num}_IS_NOM\n\
                             \x20           * t_ratio.powf(DEVICE_{dev_num}_XTI / {n_emission:.17e})\n\
                             \x20           * fast_exp((DEVICE_{dev_num}_EG / ({n_emission:.17e} * vt_nom)) * (1.0 - DEVICE_{dev_num}_TAMB / state.device_{dev_num}_tj));\n\
                             \x20   }}\n"
                        ));
                }
                DeviceParams::Tube(tp) if tp.has_self_heating() => {
                    // Sharp-cutoff triode only in phase 1 (gated by
                    // TubeParams::has_self_heating). Pdiss ≈ Ip·Vpk + Ig·Vgk;
                    // the Ig·Vgk term is tiny in normal Class-A operation
                    // (nA to µA Ig, mV Vgk) but carried through for
                    // correctness under grid-current clip. No IS(T) /
                    // VT(T) drift — the Vgk bias shift lives at the NR call
                    // site in `nr_helpers.rs`, not here.
                    let s = slot.start_idx;
                    let s1 = s + 1;
                    let tj_advance = emit_thermal_tj_advance(dev_num, tp.cth);
                    thermal_update.push_str(&format!(
                        "    {{ // Triode {dev_num} self-heating thermal update\n\
                             \x20       let ip = i_nl[{s}];\n\
                             \x20       let ig = i_nl[{s1}];\n\
                             \x20       let vgk = v_nl_th[{s}];\n\
                             \x20       let vpk = v_nl_th[{s1}];\n\
                             \x20       let p = ip * vpk + ig * vgk;\n\
                             {tj_advance}\
                             \x20   }}\n"
                    ));
                }
                _ => {}
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

        // Stateful-device (Phase 0c) after-solve update + NaN-recovery restore.
        // Both come from the shared helpers so DK and nodal cannot drift. Only
        // inserted when non-empty; the template guards on `is defined and != ""`
        // so a non-stateful deck is byte-identical.
        let stateful_devs = stateful_device_data(ir);
        let stateful_update = emit_stateful_update(&stateful_devs);
        if !stateful_update.is_empty() {
            ctx.insert("stateful_update", &stateful_update);
        }
        let stateful_nan_recovery = emit_stateful_state_restore(&stateful_devs, "state.");
        if !stateful_nan_recovery.is_empty() {
            ctx.insert("stateful_nan_recovery", &stateful_nan_recovery);
        }

        ctx.insert("dc_block", &ir.dc_block);

        // Op-amp supply rail clamping data for DK template. Skip op-amps
        // whose VCC/VEE are both infinite — those entries exist in
        // `ir.opamps` only because `sr` is finite (slew limiting is handled
        // via a separate `opamp_slew` context block below).
        //
        // Rail-mode consumption (DK path): `--opamp-rail-mode none` suppresses
        // the clamp vars entirely; every other mode emits the Hard post-NR
        // clamp (ActiveSet/ActiveSetBe degrade to Hard+BE-fallback here — a
        // one-shot runtime warning is emitted from the generated Default,
        // see emit_state).
        if !ir.opamps.is_empty()
            && ir.solver_config.opamp_rail_mode != crate::codegen::OpampRailMode::None
        {
            let opamp_clamps: Vec<std::collections::HashMap<&str, String>> = ir
                .opamps
                .iter()
                .filter(|oa| oa.vclamp_lo.is_finite() || oa.vclamp_hi.is_finite())
                .map(|oa| {
                    let mut m = std::collections::HashMap::new();
                    // Single-sided supplies (only one of VCC/VEE given) leave
                    // the other bound infinite. `{:.17e}` renders f64 infinity
                    // as `inf` — not a valid Rust token — so emit the
                    // `f64::INFINITY` const path instead. `x.clamp(f64::NEG_INFINITY, hi)`
                    // behaves exactly as the one-sided `.min(hi)` (and
                    // symmetrically for a missing hi bound).
                    let fmt_bound = |v: f64| {
                        if v.is_finite() {
                            format!("{v:.17e}")
                        } else if v > 0.0 {
                            "f64::INFINITY".to_string()
                        } else {
                            "f64::NEG_INFINITY".to_string()
                        }
                    };
                    m.insert("out_idx", oa.n_out_idx.to_string());
                    m.insert("lo", fmt_bound(oa.vclamp_lo));
                    m.insert("hi", fmt_bound(oa.vclamp_hi));
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
}

/// Block-diagonal parasitic-R coupling matrix R_p_full[i][j] for parasitic
/// BJTs in the DK path. Returns 0 for entries outside any parasitic-BJT block,
/// and `[RE, RB+RE, -RC, RB]` for the 2×2 block at each affected slot.
///
/// K_eff = K - R_p_full lets `bjt_evaluate` (intrinsic) replace `bjt_with_parasitics`
/// (inner 2D NR) on the DK path, because v_d = p + K_eff * i then equals the
/// internal junction voltage exactly. Only applied when the slot lacks
/// internal MNA nodes (DK transient path; nodal path expands internal nodes
/// instead, and DC-OP recompute uses node-voltage NR which doesn't touch K).
///
/// Gated on slot.dimension == 2: FA-reduced (1D) BJTs ignore parasitics
/// by design (see ir.rs::detect_forward_active_bjts comment) and their
/// start_idx+1 lands in the next device's slot, not the same BJT's Vbc row.
pub(super) fn parasitic_r_p_dk(ir: &CircuitIR, i: usize, j: usize) -> f64 {
    for slot in &ir.device_slots {
        if let DeviceParams::Bjt(bp) = &slot.params {
            if bp.has_parasitics() && !slot.has_internal_mna_nodes && slot.dimension == 2 {
                let s = slot.start_idx;
                if i == s && j == s {
                    return bp.re;
                }
                if i == s && j == s + 1 {
                    return bp.rb + bp.re;
                }
                if i == s + 1 && j == s {
                    return -bp.rc;
                }
                if i == s + 1 && j == s + 1 {
                    return bp.rb;
                }
            }
        }
    }
    0.0
}

/// Emit the per-slot K_eff adjustment statements (`{var}[s][s] -= RE;` …) for
/// every parasitic-absorbed BJT. Gating is identical to [`parasitic_r_p_dk`]:
/// `has_parasitics() && !has_internal_mna_nodes && dimension == 2`.
///
/// Shared by the three K-rebuild sites that must agree byte-for-byte on the
/// absorption: the baked `K_DEFAULT`/`K_BE_DEFAULT` (via `parasitic_r_p_dk`),
/// the pot/switch `rebuild_matrices` body, and the no-pot `set_sample_rate`
/// template body (passed in as `k_eff_adjust_lines`/`k_be_eff_adjust_lines`
/// context). Returns an empty string when no slot qualifies.
fn k_eff_adjust_stmts(ir: &CircuitIR, var: &str, indent: &str) -> String {
    let mut out = String::new();
    for (d, slot) in ir.device_slots.iter().enumerate() {
        if let DeviceParams::Bjt(bp) = &slot.params {
            if bp.has_parasitics() && !slot.has_internal_mna_nodes && slot.dimension == 2 {
                let s = slot.start_idx;
                let s1 = s + 1;
                out.push_str(&format!(
                    "{indent}{var}[{s}][{s}] -= DEVICE_{d}_RE;\n\
                     {indent}{var}[{s}][{s1}] -= DEVICE_{d}_RB + DEVICE_{d}_RE;\n\
                     {indent}{var}[{s1}][{s}] -= -DEVICE_{d}_RC;\n\
                     {indent}{var}[{s1}][{s1}] -= DEVICE_{d}_RB;\n",
                ));
            }
        }
    }
    out
}

// The junction-temperature advance for a self-heating device now lives in
// `super::helpers::emit_thermal_tj_advance` — a single source of truth shared
// verbatim by the DK and nodal emitters (imported above). See that helper and
// the `thermal_tj_advance_dk_nodal_string_identity` codegen test.
