//! Rust code emitter implementing the Emitter trait.
//!
//! Generates a standalone Rust module containing a specialized circuit solver.
//! All matrices are emitted as compile-time constants and loops are unrolled.
//!
//! Uses Tera templates for declarative sections and procedural code for the
//! complex NR solver (deeply conditional M=1/2/3/4 branching).

use serde::Serialize;
use tera::{Context, Tera};

use super::emitter::Emitter;
use super::ir::{CircuitIR, DeviceParams, DeviceType, PotentiometerIR};
use super::CodegenError;

/// Inductor data passed to Tera templates.
#[derive(Serialize)]
struct InductorTemplateData {
    name: String,
    node_i: usize,
    node_j: usize,
    /// Formatted g_eq string for constants template (empty when not needed)
    g_eq: String,
}

// ============================================================================
// Formatting helpers — reduce repetition in string-building code
// ============================================================================

/// Format a float with full precision for codegen constants.
fn fmt_f64(v: f64) -> String {
    format!("{:.17e}", v)
}

/// Format a matrix as rows of comma-separated full-precision floats.
///
/// `rows` x `cols` elements are read from `get(i, j)`.
fn format_matrix_rows(
    rows: usize,
    cols: usize,
    get: impl Fn(usize, usize) -> f64,
) -> Vec<String> {
    (0..rows)
        .map(|i| {
            (0..cols)
                .map(|j| fmt_f64(get(i, j)))
                .collect::<Vec<_>>()
                .join(", ")
        })
        .collect()
}

/// Build `InductorTemplateData` from IR inductors.
///
/// When `with_g_eq` is true, each entry includes the formatted g_eq value
/// (needed by the constants template). Otherwise g_eq is left empty.
fn inductor_template_data(ir: &CircuitIR, with_g_eq: bool) -> Vec<InductorTemplateData> {
    ir.inductors
        .iter()
        .map(|ind| InductorTemplateData {
            name: ind.name.clone(),
            node_i: ind.node_i,
            node_j: ind.node_j,
            g_eq: if with_g_eq {
                fmt_f64(ind.g_eq)
            } else {
                String::new()
            },
        })
        .collect()
}

/// Emit a section banner comment.
fn section_banner(title: &str) -> String {
    format!(
        "// =============================================================================\n\
         // {}\n\
         // =============================================================================\n\n",
        title
    )
}

/// Format a slice of f64 as comma-separated full-precision values.
fn format_f64_slice(values: &[f64]) -> String {
    values
        .iter()
        .map(|v| fmt_f64(*v))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Emit a single `const DEVICE_{n}_{suffix}: f64 = ...;` line.
fn emit_device_const(code: &mut String, dev_num: usize, suffix: &str, value: f64) {
    code.push_str(&format!(
        "const DEVICE_{}_{}: f64 = {};\n",
        dev_num, suffix, fmt_f64(value)
    ));
}

// Embed templates at compile time — no runtime file loading.
const TMPL_HEADER: &str = include_str!("../../templates/rust/header.rs.tera");
const TMPL_CONSTANTS: &str = include_str!("../../templates/rust/constants.rs.tera");
const TMPL_STATE: &str = include_str!("../../templates/rust/state.rs.tera");
const TMPL_DEVICE_DIODE: &str = include_str!("../../templates/rust/device_diode.rs.tera");
const TMPL_DEVICE_BJT: &str = include_str!("../../templates/rust/device_bjt.rs.tera");
const TMPL_BUILD_RHS: &str = include_str!("../../templates/rust/build_rhs.rs.tera");
const TMPL_MAT_VEC_MUL_S: &str = include_str!("../../templates/rust/mat_vec_mul_s.rs.tera");
const TMPL_EXTRACT_VOLTAGES: &str = include_str!("../../templates/rust/extract_voltages.rs.tera");
const TMPL_FINAL_VOLTAGES: &str = include_str!("../../templates/rust/final_voltages.rs.tera");
const TMPL_UPDATE_HISTORY: &str = include_str!("../../templates/rust/update_history.rs.tera");
const TMPL_PROCESS_SAMPLE: &str = include_str!("../../templates/rust/process_sample.rs.tera");

/// Rust language emitter.
pub struct RustEmitter {
    tera: Tera,
}

impl RustEmitter {
    pub fn new() -> Self {
        let mut tera = Tera::default();
        tera.add_raw_templates(vec![
            ("header", TMPL_HEADER),
            ("constants", TMPL_CONSTANTS),
            ("state", TMPL_STATE),
            ("device_diode", TMPL_DEVICE_DIODE),
            ("device_bjt", TMPL_DEVICE_BJT),
            ("build_rhs", TMPL_BUILD_RHS),
            ("mat_vec_mul_s", TMPL_MAT_VEC_MUL_S),
            ("extract_voltages", TMPL_EXTRACT_VOLTAGES),
            ("final_voltages", TMPL_FINAL_VOLTAGES),
            ("update_history", TMPL_UPDATE_HISTORY),
            ("process_sample", TMPL_PROCESS_SAMPLE),
        ])
        .expect("embedded templates should parse");
        Self { tera }
    }

    fn render(&self, name: &str, ctx: &Context) -> Result<String, CodegenError> {
        self.tera
            .render(name, ctx)
            .map_err(|e| CodegenError::TemplateError(format!("{}: {}", name, e)))
    }
}

impl Emitter for RustEmitter {
    fn language(&self) -> &str {
        "rust"
    }

    fn emit(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;

        // Validate node indices
        if ir.solver_config.input_node >= n {
            return Err(CodegenError::InvalidConfig(format!(
                "input_node {} >= N={}",
                ir.solver_config.input_node, n
            )));
        }
        if ir.solver_config.output_node >= n {
            return Err(CodegenError::InvalidConfig(format!(
                "output_node {} >= N={}",
                ir.solver_config.output_node, n
            )));
        }

        let mut code = String::new();

        code.push_str(&self.emit_header(ir)?);
        code.push_str(&self.emit_constants(ir)?);
        code.push_str(&self.emit_pot_constants(ir));
        code.push_str(&self.emit_state(ir)?);
        code.push_str(&self.emit_device_models(ir)?);
        code.push_str(&self.emit_pot_helpers(ir));
        code.push_str(&self.emit_build_rhs(ir)?);
        code.push_str(&self.emit_mat_vec_mul_s(ir)?);
        code.push_str(&self.emit_extract_voltages(ir)?);
        self.generate_solve_nonlinear(&mut code, ir)?;
        code.push_str(&self.emit_final_voltages(ir)?);
        code.push_str(&self.emit_update_history()?);
        code.push_str(&self.emit_process_sample(ir)?);

        Ok(code)
    }
}

// ============================================================================
// Template-based emission methods
// ============================================================================

impl RustEmitter {
    fn emit_header(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        ctx.insert("title", &ir.metadata.title);
        self.render("header", &ctx)
    }

    fn emit_constants(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();

        ctx.insert("n", &n);
        ctx.insert("m", &m);

        let num_inductors = ir.inductors.len();
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, true));
        }
        ctx.insert(
            "sample_rate",
            &format!("{:.1}", ir.solver_config.sample_rate),
        );
        ctx.insert("alpha", &fmt_f64(ir.solver_config.alpha));
        ctx.insert("input_node", &ir.solver_config.input_node);
        ctx.insert("output_node", &ir.solver_config.output_node);
        ctx.insert("input_resistance", &fmt_f64(ir.solver_config.input_resistance));
        ctx.insert("has_dc_sources", &ir.has_dc_sources);

        ctx.insert("s_rows", &format_matrix_rows(n, n, |i, j| ir.s(i, j)));
        ctx.insert("a_neg_rows", &format_matrix_rows(n, n, |i, j| ir.a_neg(i, j)));

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

        self.render("constants", &ctx)
    }

    fn emit_state(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        ctx.insert("has_dc_op", &ir.has_dc_op);
        ctx.insert("num_inductors", &ir.inductors.len());
        ctx.insert("num_pots", &ir.pots.len());

        let pot_defaults: Vec<String> = ir.pots.iter().map(|p| fmt_f64(1.0 / p.g_nominal)).collect();
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

        self.render("state", &ctx)
    }

    fn emit_device_models(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = section_banner("DEVICE MODELS");

        let mut has_diode = false;
        let mut has_bjt = false;

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            match &slot.params {
                DeviceParams::Diode(dp) => {
                    has_diode = true;
                    emit_device_const(&mut code, dev_num, "IS", dp.is);
                    emit_device_const(&mut code, dev_num, "N_VT", dp.n_vt);
                    code.push('\n');
                }
                DeviceParams::Bjt(bp) => {
                    has_bjt = true;
                    emit_device_const(&mut code, dev_num, "IS", bp.is);
                    emit_device_const(&mut code, dev_num, "VT", bp.vt);
                    emit_device_const(&mut code, dev_num, "BETA_F", bp.beta_f);
                    emit_device_const(&mut code, dev_num, "BETA_R", bp.beta_r);
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n\n",
                        dev_num, sign
                    ));
                }
            }
        }

        if has_diode {
            code.push_str(&self.render("device_diode", &Context::new())?);
        }
        if has_bjt {
            code.push_str(&self.render("device_bjt", &Context::new())?);
        }

        Ok(code)
    }

    fn emit_pot_constants(&self, ir: &CircuitIR) -> String {
        if ir.pots.is_empty() {
            return String::new();
        }
        let m = ir.topology.m;
        let mut code = section_banner("POTENTIOMETER CONSTANTS (Sherman-Morrison precomputed vectors)");

        for (idx, pot) in ir.pots.iter().enumerate() {
            let su_values = format_f64_slice(&pot.su);
            code.push_str(&format!(
                "const POT_{}_SU: [f64; N] = [{}];\n", idx, su_values
            ));
            code.push_str(&format!(
                "const POT_{}_USU: f64 = {};\n", idx, fmt_f64(pot.usu)
            ));
            code.push_str(&format!(
                "const POT_{}_G_NOM: f64 = {};\n", idx, fmt_f64(pot.g_nominal)
            ));

            if m > 0 {
                code.push_str(&format!(
                    "const POT_{}_NV_SU: [f64; M] = [{}];\n", idx, format_f64_slice(&pot.nv_su)
                ));
                code.push_str(&format!(
                    "const POT_{}_U_NI: [f64; M] = [{}];\n", idx, format_f64_slice(&pot.u_ni)
                ));
            }

            if pot.node_p > 0 {
                code.push_str(&format!(
                    "const POT_{}_NODE_P: usize = {};\n", idx, pot.node_p - 1
                ));
            }
            if pot.node_q > 0 {
                code.push_str(&format!(
                    "const POT_{}_NODE_Q: usize = {};\n", idx, pot.node_q - 1
                ));
            }

            code.push_str(&format!(
                "const POT_{}_MIN_R: f64 = {};\n", idx, fmt_f64(pot.min_resistance)
            ));
            code.push_str(&format!(
                "const POT_{}_MAX_R: f64 = {};\n", idx, fmt_f64(pot.max_resistance)
            ));
            code.push('\n');
        }
        code
    }

    fn emit_pot_helpers(&self, ir: &CircuitIR) -> String {
        if ir.pots.is_empty() {
            return String::new();
        }
        let mut code = section_banner("POTENTIOMETER SM SCALE HELPERS");

        for (idx, _pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "/// Sherman-Morrison scale factor for pot {idx}\n\
                 #[inline(always)]\n\
                 fn sm_scale_{idx}(state: &CircuitState) -> (f64, f64) {{\n\
                 \x20   let r = state.pot_{idx}_resistance;\n\
                 \x20   if !r.is_finite() {{ return (0.0, 0.0); }}\n\
                 \x20   let r = r.clamp(POT_{idx}_MIN_R, POT_{idx}_MAX_R);\n\
                 \x20   let delta_g = 1.0 / r - POT_{idx}_G_NOM;\n\
                 \x20   let denom = 1.0 + delta_g * POT_{idx}_USU;\n\
                 \x20   let scale = if denom.abs() > 1e-15 {{ delta_g / denom }} else {{ 0.0 }};\n\
                 \x20   (delta_g, scale)\n\
                 }}\n\n",
            ));
        }
        code
    }

    fn emit_build_rhs(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();

        ctx.insert("has_dc_sources", &ir.has_dc_sources);

        let num_inductors = ir.inductors.len();
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, false));
        }

        // A_neg * v_prev lines
        let assign_op = if ir.has_dc_sources { "+=" } else { "=" };
        let mut a_neg_lines = String::new();
        for i in 0..n {
            let mut terms = Vec::new();
            for j in 0..n {
                if ir.a_neg(i, j) != 0.0 {
                    terms.push(format!("A_NEG[{}][{}] * state.v_prev[{}]", i, j, j));
                }
            }
            if terms.is_empty() {
                if !ir.has_dc_sources {
                    a_neg_lines.push_str(&format!("    rhs[{}] = 0.0;\n", i));
                }
            } else {
                a_neg_lines.push_str(&format!(
                    "    rhs[{}] {} {};\n",
                    i,
                    assign_op,
                    terms.join(" + ")
                ));
            }
        }
        ctx.insert("a_neg_lines", &a_neg_lines);

        // N_i * i_nl_prev lines
        let has_nl_prev = m > 0;
        ctx.insert("has_nl_prev", &has_nl_prev);
        if has_nl_prev {
            let mut nl_prev_lines = String::new();
            for i in 0..n {
                for j in 0..m {
                    if ir.n_i(i, j) != 0.0 {
                        nl_prev_lines.push_str(&format!(
                            "    rhs[{}] += N_I[{}][{}] * state.i_nl_prev[{}];\n",
                            i, j, i, j
                        ));
                    }
                }
            }
            ctx.insert("nl_prev_lines", &nl_prev_lines);
        }

        self.render("build_rhs", &ctx)
    }

    fn emit_mat_vec_mul_s(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let mut ctx = Context::new();

        let mut mul_lines = String::new();
        for i in 0..n {
            mul_lines.push_str("        ");
            let mut first = true;
            for j in 0..n {
                if ir.s(i, j) != 0.0 || (!first && j == n - 1) {
                    if !first {
                        mul_lines.push_str(" + ");
                    }
                    mul_lines.push_str(&format!("S[{}][{}] * rhs[{}]", i, j, j));
                    first = false;
                }
            }
            if first {
                mul_lines.push_str("0.0");
            }
            mul_lines.push_str(",\n");
        }
        ctx.insert("mul_lines", &mul_lines);

        self.render("mat_vec_mul_s", &ctx)
    }

    fn emit_extract_voltages(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();

        let mut extract_lines = String::new();
        for i in 0..m {
            extract_lines.push_str("        ");
            let mut first = true;
            for j in 0..n {
                let coeff = ir.n_v(i, j);
                if coeff != 0.0 {
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
            if first {
                extract_lines.push_str("0.0");
            }
            extract_lines.push_str(",\n");
        }
        ctx.insert("extract_lines", &extract_lines);

        self.render("extract_voltages", &ctx)
    }

    fn emit_final_voltages(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();
        ctx.insert("m", &m);

        // No delta_i lines needed — trapezoidal uses full i_nl (not delta)
        let di_lines = String::new();
        ctx.insert("di_lines", &di_lines);

        // voltage computation lines: v = v_pred + S * N_i * i_nl
        // Using full i_nl implements trapezoidal nonlinear integration:
        // combined with N_i * i_nl_prev in the RHS, gives S * N_i * (i_nl + i_nl_prev)
        let mut voltage_lines = String::new();
        for i in 0..n {
            voltage_lines.push_str(&format!("        v_pred[{}]", i));
            for j in 0..m {
                let mut s_ni_ij = 0.0;
                for k in 0..n {
                    s_ni_ij += ir.s(i, k) * ir.n_i(k, j);
                }
                if s_ni_ij != 0.0 {
                    voltage_lines.push_str(&format!(" + {} * i_nl[{}]", fmt_f64(s_ni_ij), j));
                }
            }
            voltage_lines.push_str(",\n");
        }
        ctx.insert("voltage_lines", &voltage_lines);

        self.render("final_voltages", &ctx)
    }

    fn emit_update_history(&self) -> Result<String, CodegenError> {
        self.render("update_history", &Context::new())
    }

    fn emit_process_sample(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        let num_inductors = ir.inductors.len();
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, false));
        }

        let num_pots = ir.pots.len();
        ctx.insert("num_pots", &num_pots);
        let pot_defaults: Vec<String> = ir.pots.iter().map(|p| fmt_f64(1.0 / p.g_nominal)).collect();
        ctx.insert("pot_defaults", &pot_defaults);

        // Generate pot correction code blocks procedurally
        if num_pots > 0 {
            let n = ir.topology.n;
            let m = ir.topology.m;

            // SM scale computation (top of function)
            let mut sm_scale_lines = String::new();
            for idx in 0..num_pots {
                sm_scale_lines.push_str(&format!(
                    "    let (delta_g_{}, scale_{}) = sm_scale_{}(state);\n",
                    idx, idx, idx
                ));
            }
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
        }

        self.render("process_sample", &ctx)
    }

    fn emit_a_neg_correction(code: &mut String, idx: usize, pot: &PotentiometerIR) {
        // A_neg correction: delta_g changes the effective A_neg matrix
        // The pot conductance change affects the RHS through v_prev:
        // rhs[p] -= delta_g * (v_prev[p] - v_prev[q])
        // rhs[q] += delta_g * (v_prev[p] - v_prev[q])  (if not grounded)
        if pot.node_p > 0 && pot.node_q > 0 {
            let p = pot.node_p - 1;
            let q = pot.node_q - 1;
            code.push_str(&format!(
                "    let v_diff_{} = state.v_prev[{}] - state.v_prev[{}];\n",
                idx, p, q
            ));
            code.push_str(&format!(
                "    rhs[{}] -= delta_g_{} * v_diff_{};\n", p, idx, idx
            ));
            code.push_str(&format!(
                "    rhs[{}] += delta_g_{} * v_diff_{};\n", q, idx, idx
            ));
        } else if pot.node_p > 0 {
            let p = pot.node_p - 1;
            code.push_str(&format!(
                "    rhs[{}] -= delta_g_{} * state.v_prev[{}];\n", p, idx, p
            ));
        } else if pot.node_q > 0 {
            let q = pot.node_q - 1;
            code.push_str(&format!(
                "    rhs[{}] -= delta_g_{} * state.v_prev[{}];\n", q, idx, q
            ));
        }
    }

    fn emit_s_correction(code: &mut String, idx: usize, pot: &PotentiometerIR, n: usize) {
        // S correction: v_pred -= scale * (su^T . rhs) * SU
        // Sherman-Morrison: S' * rhs = S*rhs - scale * su * (su^T * rhs)
        // where su = S*u, so the inner product must use su (POT_SU), not u.
        code.push_str(&format!("    let su_dot_rhs_{} = ", idx));
        let mut first = true;
        for k in 0..n {
            if pot.su[k] != 0.0 {
                if !first { code.push_str(" + "); }
                code.push_str(&format!("POT_{}_SU[{}] * rhs[{}]", idx, k, k));
                first = false;
            }
        }
        if first {
            // All su entries are zero (degenerate pot)
            code.push_str("0.0");
        }
        code.push_str(";\n");

        code.push_str(&format!(
            "    let factor_{} = scale_{} * su_dot_rhs_{};\n", idx, idx, idx
        ));
        for k in 0..n {
            code.push_str(&format!(
                "    v_pred[{}] -= factor_{} * POT_{}_SU[{}];\n", k, idx, idx, k
            ));
        }
    }

    fn emit_sni_correction(code: &mut String, idx: usize, _pot: &PotentiometerIR, n: usize, m: usize) {
        // S*N_i correction: v -= scale * (u^T . N_i . i_nl) * SU
        // where u^T . N_i is the precomputed POT_idx_U_NI vector
        // Uses full i_nl (not delta) for trapezoidal nonlinear integration
        code.push_str(&format!("    let u_ni_dot_inl_{} = ", idx));
        let mut first = true;
        for j in 0..m {
            if !first { code.push_str(" + "); }
            code.push_str(&format!(
                "POT_{}_U_NI[{}] * i_nl[{}]", idx, j, j
            ));
            first = false;
        }
        code.push_str(";\n");
        code.push_str(&format!(
            "    let sni_factor_{} = scale_{} * u_ni_dot_inl_{};\n", idx, idx, idx
        ));
        for k in 0..n {
            code.push_str(&format!(
                "    v[{}] -= sni_factor_{} * POT_{}_SU[{}];\n", k, idx, idx, k
            ));
        }
    }
}

// ============================================================================
// Procedural NR solver generation (too complex for templates)
// ============================================================================

/// Emit damped fallback lines for a singular Jacobian: `i_nl[i] -= (fi * 0.5).clamp(...)`.
fn emit_nr_singular_fallback(code: &mut String, dim: usize, indent: &str) {
    code.push_str(&format!("{indent}// Singular Jacobian — damped fallback (0.5 * residual)\n"));
    for i in 0..dim {
        code.push_str(&format!(
            "{indent}i_nl[{i}] -= (f{i} * 0.5).clamp(-STEP_CLAMP, STEP_CLAMP);\n"
        ));
    }
}

/// Emit clamp, update, and convergence check for NR delta values.
///
/// Assumes `delta0..delta{dim-1}` are already defined. Emits clamping,
/// `i_nl` update, and early return on convergence.
fn emit_nr_clamp_and_converge(code: &mut String, dim: usize, indent: &str) {
    code.push_str(&format!("{indent}// Clamp steps to prevent overshoot\n"));
    for i in 0..dim {
        code.push_str(&format!(
            "{indent}let clamped_delta{i} = delta{i}.clamp(-STEP_CLAMP, STEP_CLAMP);\n"
        ));
    }
    for i in 0..dim {
        code.push_str(&format!("{indent}i_nl[{i}] -= clamped_delta{i};\n"));
    }
    code.push_str(&format!("\n{indent}// Convergence check\n"));
    code.push_str(&format!("{indent}if "));
    for i in 0..dim {
        if i > 0 {
            code.push_str(" + ");
        }
        code.push_str(&format!("clamped_delta{i}.abs()"));
    }
    code.push_str(" < TOL {\n");
    code.push_str(&format!("{indent}    state.last_nr_iterations = iter as u32;\n"));
    code.push_str(&format!("{indent}    return i_nl;\n"));
    code.push_str(&format!("{indent}}}\n"));
}

impl RustEmitter {
    /// Generate solve_nonlinear function using Newton-Raphson.
    ///
    /// This method stays procedural due to its deeply conditional
    /// structure: M=1/2/3/4 branching, device-type dispatch, block-diagonal
    /// Jacobian assembly.
    fn generate_solve_nonlinear(
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
            "fn solve_nonlinear(p: &[f64; M], state: &mut CircuitState) -> [f64; M] {\n"
        );
        code.push_str(&format!(
            "    const MAX_ITER: usize = {};\n",
            ir.solver_config.max_iterations
        ));
        code.push_str(&format!(
            "    const TOL: f64 = {:.17e};\n",
            ir.solver_config.tolerance
        ));
        // STEP_CLAMP=0.01 matches the runtime solver's clamp value.
        code.push_str("    const STEP_CLAMP: f64 = 0.01;  // Matches runtime solver clamp\n");
        code.push_str("    const SINGULARITY_THRESHOLD: f64 = 1e-15;\n\n");

        code.push_str("    // Initial guess from previous sample (warm start)\n");
        code.push_str("    let mut i_nl = state.i_nl_prev;\n\n");

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

        // Compute v_d = p + K * i_nl
        code.push_str("        // Compute controlling voltages: v_d = p + K * i_nl\n");
        let has_pots = !ir.pots.is_empty();
        let vd_let = if has_pots { "let mut" } else { "let" };
        for i in 0..m {
            code.push_str(&format!("        {} v_d{} = ", vd_let, i));
            code.push_str(&format!("p[{}]", i));
            for j in 0..m {
                if ir.k(i, j) != 0.0 {
                    code.push_str(&format!(" + K[{}][{}] * i_nl[{}]", i, j, j));
                }
            }
            code.push_str(";\n");
        }

        // K correction for pots: v_d[i] -= scale * NV_SU[i] * (u^T * N_i * i_nl)
        if !ir.pots.is_empty() {
            code.push_str("\n        // Sherman-Morrison K correction for potentiometers\n");
            for (idx, pot) in ir.pots.iter().enumerate() {
                if pot.u_ni.is_empty() {
                    continue;
                }
                code.push_str(&format!(
                    "        let (_, k_scale_{}) = sm_scale_{}(state);\n", idx, idx
                ));
                // Compute u^T * N_i * i_nl
                code.push_str(&format!("        let u_ni_dot_inl_{} = ", idx));
                let mut first = true;
                for j in 0..m {
                    if !first { code.push_str(" + "); }
                    code.push_str(&format!("POT_{}_U_NI[{}] * i_nl[{}]", idx, j, j));
                    first = false;
                }
                code.push_str(";\n");
                // Apply correction to each v_d
                for i in 0..m {
                    code.push_str(&format!(
                        "        v_d{} -= k_scale_{} * POT_{}_NV_SU[{}] * u_ni_dot_inl_{};\n",
                        i, idx, idx, i, idx
                    ));
                }
            }
        }
        code.push_str("\n");

        if has_nonlinear {
            // Device currents and Jacobian entries
            code.push_str("        // Evaluate device currents and Jacobians\n");
            for (dev_num, slot) in ir.device_slots.iter().enumerate() {
                match slot.device_type {
                    DeviceType::Diode => {
                        let s = slot.start_idx;
                        code.push_str(&format!(
                            "        let i_dev{} = diode_current(v_d{}, DEVICE_{}_IS, DEVICE_{}_N_VT);\n",
                            s, s, dev_num, dev_num
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = diode_conductance(v_d{}, DEVICE_{}_IS, DEVICE_{}_N_VT);\n",
                            s, s, s, dev_num, dev_num
                        ));
                    }
                    DeviceType::Bjt => {
                        let s = slot.start_idx;
                        let s1 = s + 1;
                        code.push_str(&format!(
                            "        let i_dev{} = bjt_ic(v_d{}, v_d{}, DEVICE_{}_IS, DEVICE_{}_VT, DEVICE_{}_BETA_R, DEVICE_{}_SIGN);\n",
                            s, s, s1, dev_num, dev_num, dev_num, dev_num
                        ));
                        code.push_str(&format!(
                            "        let i_dev{} = bjt_ib(v_d{}, v_d{}, DEVICE_{}_IS, DEVICE_{}_VT, DEVICE_{}_BETA_F, DEVICE_{}_BETA_R, DEVICE_{}_SIGN);\n",
                            s1, s, s1, dev_num, dev_num, dev_num, dev_num, dev_num
                        ));
                        code.push_str(&format!(
                            "        let bjt{}_jac = bjt_jacobian(v_d{}, v_d{}, DEVICE_{}_IS, DEVICE_{}_VT, DEVICE_{}_BETA_F, DEVICE_{}_BETA_R, DEVICE_{}_SIGN);\n",
                            dev_num, s, s1, dev_num, dev_num, dev_num, dev_num, dev_num
                        ));
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
                }
            }
            code.push_str("\n");

            // Residuals
            code.push_str("        // Residuals: f(i) = i - i_dev(v(i)) = 0\n");
            for i in 0..m {
                code.push_str(&format!(
                    "        let f{} = i_nl[{}] - i_dev{};\n",
                    i, i, i
                ));
            }
            code.push_str("\n");

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
                    .ok_or_else(|| CodegenError::InvalidConfig(
                        format!("no device slot found for M-dimension index {}", i)
                    ))?;
                let blk_start = slot.start_idx;
                let blk_dim = slot.dimension;
                for j in 0..m {
                    let diag = if i == j { "1.0" } else { "0.0" };
                    let mut terms = String::new();
                    for k in blk_start..blk_start + blk_dim {
                        if ir.pots.is_empty() {
                            terms.push_str(&format!(
                                " - jdev_{}_{} * K[{}][{}]",
                                i, k, k, j
                            ));
                        } else {
                            // Use corrected K': K[k][j] - sum_pots(scale_p * nv_su_p[k] * u_ni_p[j])
                            let mut k_correction = String::new();
                            for (idx, _pot) in ir.pots.iter().enumerate() {
                                k_correction.push_str(&format!(
                                    " - k_scale_{} * POT_{}_NV_SU[{}] * POT_{}_U_NI[{}]",
                                    idx, idx, k, idx, j
                                ));
                            }
                            terms.push_str(&format!(
                                " - jdev_{}_{} * (K[{}][{}]{})",
                                i, k, k, j, k_correction
                            ));
                        }
                    }
                    code.push_str(&format!(
                        "        let j{}{} = {}{};\n",
                        i, j, diag, terms
                    ));
                }
            }
            code.push_str("\n");

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
                    emit_nr_clamp_and_converge(code, 1, "        ");
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
                    emit_nr_clamp_and_converge(code, 2, "        ");
                }
                3 | 4 => {
                    Self::generate_gauss_elim(code, m);
                }
                _ => {
                    return Err(CodegenError::UnsupportedTopology(format!(
                        "M={} nonlinear devices not supported (max 4)",
                        m
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

        // NaN/Inf check
        code.push_str("    // Safety: check for NaN/Inf and clamp\n");
        for i in 0..m {
            code.push_str(&format!(
                "    if !i_nl[{}].is_finite() {{ i_nl[{}] = 0.0; }}\n",
                i, i
            ));
        }
        code.push_str("\n");
        code.push_str("    i_nl\n");
        code.push_str("}\n\n");

        Ok(())
    }

    /// Generate inline Gaussian elimination for M=3 or M=4.
    fn generate_gauss_elim(code: &mut String, dim: usize) {
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
        emit_nr_clamp_and_converge(code, dim, "            ");
        code.push_str("        } else {\n");
        emit_nr_singular_fallback(code, dim, "            ");
        code.push_str("        }\n");
    }
}
