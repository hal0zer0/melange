//! Rust code emitter implementing the Emitter trait.
//!
//! Generates a standalone Rust module containing a specialized circuit solver.
//! All matrices are emitted as compile-time constants and loops are unrolled.
//!
//! Uses Tera templates for declarative sections and procedural code for the
//! complex NR solver (deeply conditional M=1/2/3..8 branching).

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
    /// Formatted inductance string for constants template (empty when not needed)
    inductance: String,
}

// ============================================================================
// Formatting helpers — reduce repetition in string-building code
// ============================================================================

/// Format a float with full precision for codegen constants.
fn fmt_f64(v: f64) -> String {
    if v.is_infinite() {
        if v > 0.0 { "f64::INFINITY".to_string() }
        else { "f64::NEG_INFINITY".to_string() }
    } else if v.is_nan() {
        "f64::NAN".to_string()
    } else {
        format!("{:.17e}", v)
    }
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
            inductance: if with_g_eq {
                fmt_f64(ind.inductance)
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

// ============================================================================
// Oversampling configuration
// ============================================================================

/// Half-band filter coefficients from melange-primitives/src/oversampling.rs.
/// 3-section (~80dB rejection, balanced quality/cost).
const HB_3SECTION: [f64; 3] = [
    0.036681502163648017,
    0.2746317593794541,
    0.7856959333713522,
];

/// 2-section half-band (minimal CPU, ~60dB rejection) for 4x outer stage.
const HB_2SECTION: [f64; 2] = [0.07986642623635751, 0.5453536510716122];

/// Oversampling stage configuration.
struct OversamplingInfo {
    /// Number of allpass sections per filter (for inner 2x stage)
    num_sections: usize,
    /// Coefficients for inner 2x stage
    coeffs: Vec<f64>,
    /// State size per filter = num_sections * 2 (x1, y1 per section)
    state_size: usize,
    /// For 4x: state size of the outer 2x stage
    state_size_outer: usize,
    /// For 4x: coefficients for outer 2x stage
    coeffs_outer: Vec<f64>,
    /// For 4x: number of sections in outer stage
    num_sections_outer: usize,
}

/// Get oversampling configuration for a given factor.
fn oversampling_info(factor: usize) -> OversamplingInfo {
    match factor {
        2 => {
            let num_sections = HB_3SECTION.len();
            OversamplingInfo {
                num_sections,
                coeffs: HB_3SECTION.to_vec(),
                state_size: num_sections * 2,
                state_size_outer: 0,
                coeffs_outer: Vec::new(),
                num_sections_outer: 0,
            }
        }
        4 => {
            // Inner 2x stage uses 3-section, outer uses 2-section
            let inner = HB_3SECTION.len();
            let outer = HB_2SECTION.len();
            OversamplingInfo {
                num_sections: inner,
                coeffs: HB_3SECTION.to_vec(),
                state_size: inner * 2,
                state_size_outer: outer * 2,
                coeffs_outer: HB_2SECTION.to_vec(),
                num_sections_outer: outer,
            }
        }
        _ => OversamplingInfo {
            num_sections: 0,
            coeffs: Vec::new(),
            state_size: 0,
            state_size_outer: 0,
            coeffs_outer: Vec::new(),
            num_sections_outer: 0,
        },
    }
}

// Embed templates at compile time — no runtime file loading.
const TMPL_HEADER: &str = include_str!("../../templates/rust/header.rs.tera");
const TMPL_CONSTANTS: &str = include_str!("../../templates/rust/constants.rs.tera");
const TMPL_STATE: &str = include_str!("../../templates/rust/state.rs.tera");
const TMPL_DEVICE_DIODE: &str = include_str!("../../templates/rust/device_diode.rs.tera");
const TMPL_DEVICE_BJT: &str = include_str!("../../templates/rust/device_bjt.rs.tera");
const TMPL_DEVICE_JFET: &str = include_str!("../../templates/rust/device_jfet.rs.tera");
const TMPL_DEVICE_TUBE: &str = include_str!("../../templates/rust/device_tube.rs.tera");
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

impl Default for RustEmitter {
    fn default() -> Self {
        Self::new()
    }
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
            ("device_jfet", TMPL_DEVICE_JFET),
            ("device_tube", TMPL_DEVICE_TUBE),
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
        ctx.insert("oversampling_factor", &ir.solver_config.oversampling_factor);
        if ir.solver_config.oversampling_factor > 1 {
            let internal_rate = ir.solver_config.sample_rate
                * ir.solver_config.oversampling_factor as f64;
            ctx.insert("internal_sample_rate", &format!("{:.1}", internal_rate));
        }
        ctx.insert("alpha", &fmt_f64(ir.solver_config.alpha));
        ctx.insert("input_node", &ir.solver_config.input_node);
        ctx.insert("output_node", &ir.solver_config.output_node);
        ctx.insert("input_resistance", &fmt_f64(ir.solver_config.input_resistance));
        ctx.insert("has_dc_sources", &ir.has_dc_sources);

        // G and C matrices (sample-rate independent)
        ctx.insert("g_rows", &format_matrix_rows(n, n, |i, j| ir.g(i, j)));
        ctx.insert("c_rows", &format_matrix_rows(n, n, |i, j| ir.c(i, j)));

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

        // S*N_i product: precomputed for final voltage correction
        // S_NI[node][device] = sum_k S[node][k] * N_i[k][device]
        let s_ni_rows: Vec<String> = (0..n).map(|i| {
            (0..m).map(|j| {
                let mut val = 0.0;
                for k in 0..n {
                    val += ir.s(i, k) * ir.n_i(k, j);
                }
                fmt_f64(val)
            }).collect::<Vec<_>>().join(", ")
        }).collect();
        ctx.insert("s_ni_rows", &s_ni_rows);

        self.render("constants", &ctx)
    }

    fn emit_state(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        ctx.insert("has_dc_op", &ir.has_dc_op);
        let num_inductors = ir.inductors.len();
        ctx.insert("num_inductors", &num_inductors);
        let num_pots = ir.pots.len();
        ctx.insert("num_pots", &num_pots);

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

        let pot_defaults: Vec<String> = ir.pots.iter().map(|p| fmt_f64(1.0 / p.g_nominal)).collect();
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

        self.render("state", &ctx)
    }

    fn emit_device_models(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = section_banner("DEVICE MODELS");

        let mut has_diode = false;
        let mut has_bjt = false;
        let mut has_jfet = false;
        let mut has_tube = false;

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
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n",
                        dev_num, sign
                    ));
                    code.push_str(&format!(
                        "const DEVICE_{}_USE_GP: bool = {};\n",
                        dev_num, bp.is_gummel_poon()
                    ));
                    emit_device_const(&mut code, dev_num, "VAF", bp.vaf);
                    emit_device_const(&mut code, dev_num, "VAR", bp.var);
                    emit_device_const(&mut code, dev_num, "IKF", bp.ikf);
                    emit_device_const(&mut code, dev_num, "IKR", bp.ikr);
                    code.push('\n');
                }
                DeviceParams::Jfet(jp) => {
                    has_jfet = true;
                    emit_device_const(&mut code, dev_num, "IDSS", jp.idss);
                    emit_device_const(&mut code, dev_num, "VP", jp.vp);
                    let sign = if jp.is_p_channel { -1.0 } else { 1.0 };
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
                    code.push('\n');
                }
            }
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
        if has_tube {
            code.push_str(&self.render("device_tube", &Context::new())?);
        }

        Ok(code)
    }

    fn emit_pot_constants(&self, ir: &CircuitIR) -> String {
        if ir.pots.is_empty() {
            return String::new();
        }
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

            code.push_str(&format!(
                "const POT_{}_NV_SU: [f64; M] = [{}];\n", idx, format_f64_slice(&pot.nv_su)
            ));
            code.push_str(&format!(
                "const POT_{}_U_NI: [f64; M] = [{}];\n", idx, format_f64_slice(&pot.u_ni)
            ));

            // Node indices (1-indexed, 0 = ground) for set_sample_rate recomputation
            code.push_str(&format!(
                "const POT_{}_NODE_P: usize = {};\n", idx, pot.node_p
            ));
            code.push_str(&format!(
                "const POT_{}_NODE_Q: usize = {};\n", idx, pot.node_q
            ));

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
                 \x20   let denom = 1.0 + delta_g * state.pot_{idx}_usu;\n\
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
        let mut ctx = Context::new();
        let num_inductors = ir.inductors.len();
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, false));
        }

        let os_factor = ir.solver_config.oversampling_factor;
        ctx.insert("oversampling_factor", &os_factor);

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

    /// Emit oversampling wrapper: constants, allpass helper, halfband, and process_sample.
    fn emit_oversampler(ir: &CircuitIR) -> String {
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

        // Emit the public process_sample wrapper
        code.push_str("/// Process a single audio sample through the circuit with oversampling.\n");
        code.push_str("///\n");
        code.push_str(&format!(
            "/// Runs the circuit at {}x the host sample rate to reduce aliasing.\n",
            factor
        ));
        code.push_str("#[inline]\n");
        code.push_str(
            "pub fn process_sample(input: f64, state: &mut CircuitState) -> f64 {\n",
        );
        code.push_str(
            "    let input = if input.is_finite() { input.clamp(-100.0, 100.0) } else { 0.0 };\n\n",
        );

        if factor == 2 {
            Self::emit_2x_wrapper(&mut code);
        } else if factor == 4 {
            Self::emit_4x_wrapper(&mut code);
        }

        code.push_str("}\n\n");
        code
    }

    /// Emit a halfband filter function that processes input through even/odd allpass chains.
    fn emit_halfband_fn(code: &mut String, name: &str, coeffs: &[f64], state_size: usize) {
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
    fn emit_2x_wrapper(code: &mut String) {
        // Upsample: halfband → 2 samples
        code.push_str(
            "    // Upsample: half-band filter produces 2 samples at internal rate\n\
             \x20   let (up_even, up_odd) = os_halfband(input, &OS_COEFFS, &mut state.os_up_state);\n\n",
        );

        // Process both at 2x rate
        code.push_str(
            "    // Process both samples at 2x rate\n\
             \x20   let out_even = process_sample_inner(up_even, state);\n\
             \x20   let out_odd = process_sample_inner(up_odd, state);\n\n",
        );

        // Downsample: filter both and combine
        code.push_str(
            "    // Downsample: half-band filter combines 2 samples into 1\n\
             \x20   let (dn1_even, _) = os_halfband(out_even, &OS_COEFFS, &mut state.os_dn_state);\n\
             \x20   let (_, dn2_odd) = os_halfband(out_odd, &OS_COEFFS, &mut state.os_dn_state);\n\
             \x20   let out = (dn1_even + dn2_odd) * 0.5;\n\
             \x20   if out.is_finite() { out.clamp(-10.0, 10.0) } else { 0.0 }\n",
        );
    }

    /// Emit the 4x oversampling wrapper body (cascaded 2x stages).
    fn emit_4x_wrapper(code: &mut String) {
        // Outer upsample: 1 → 2 at 2x rate
        code.push_str(
            "    // Outer upsample: 1 → 2 samples at 2x rate\n\
             \x20   let (outer_even, outer_odd) = os_halfband_outer(\n\
             \x20       input, &OS_COEFFS_OUTER, &mut state.os_up_state_outer,\n\
             \x20   );\n\n",
        );

        // Inner upsample + process for each outer sample
        code.push_str(
            "    // Inner upsample + process: each 2x sample → 2 samples at 4x rate\n\
             \x20   let (inner_e0, inner_o0) = os_halfband(outer_even, &OS_COEFFS, &mut state.os_up_state);\n\
             \x20   let proc_e0 = process_sample_inner(inner_e0, state);\n\
             \x20   let proc_o0 = process_sample_inner(inner_o0, state);\n\
             \x20   let (dn_e0, _) = os_halfband(proc_e0, &OS_COEFFS, &mut state.os_dn_state);\n\
             \x20   let (_, dn_o0) = os_halfband(proc_o0, &OS_COEFFS, &mut state.os_dn_state);\n\
             \x20   let inner_out0 = (dn_e0 + dn_o0) * 0.5;\n\n",
        );

        code.push_str(
            "    let (inner_e1, inner_o1) = os_halfband(outer_odd, &OS_COEFFS, &mut state.os_up_state);\n\
             \x20   let proc_e1 = process_sample_inner(inner_e1, state);\n\
             \x20   let proc_o1 = process_sample_inner(inner_o1, state);\n\
             \x20   let (dn_e1, _) = os_halfband(proc_e1, &OS_COEFFS, &mut state.os_dn_state);\n\
             \x20   let (_, dn_o1) = os_halfband(proc_o1, &OS_COEFFS, &mut state.os_dn_state);\n\
             \x20   let inner_out1 = (dn_e1 + dn_o1) * 0.5;\n\n",
        );

        // Outer downsample
        code.push_str(
            "    // Outer downsample: 2 → 1 sample at host rate\n\
             \x20   let (dn_outer_e, _) = os_halfband_outer(\n\
             \x20       inner_out0, &OS_COEFFS_OUTER, &mut state.os_dn_state_outer,\n\
             \x20   );\n\
             \x20   let (_, dn_outer_o) = os_halfband_outer(\n\
             \x20       inner_out1, &OS_COEFFS_OUTER, &mut state.os_dn_state_outer,\n\
             \x20   );\n\
             \x20   let out = (dn_outer_e + dn_outer_o) * 0.5;\n\
             \x20   if out.is_finite() { out.clamp(-10.0, 10.0) } else { 0.0 }\n",
        );
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

    fn emit_s_correction(code: &mut String, idx: usize, _pot: &PotentiometerIR, n: usize) {
        // S correction: v_pred -= scale * (su^T . rhs) * SU
        // Sherman-Morrison: S' * rhs = S*rhs - scale * su * (su^T * rhs)
        // where su = S*u, so the inner product must use su (state.pot_su), not u.
        // Uses runtime state fields (not constants) for sample-rate independence.
        code.push_str(&format!("    let mut su_dot_rhs_{} = 0.0;\n", idx));
        code.push_str(&format!(
            "    for _k in 0..N {{ su_dot_rhs_{idx} += state.pot_{idx}_su[_k] * rhs[_k]; }}\n"
        ));

        code.push_str(&format!(
            "    let factor_{} = scale_{} * su_dot_rhs_{};\n", idx, idx, idx
        ));
        for k in 0..n {
            code.push_str(&format!(
                "    v_pred[{}] -= factor_{} * state.pot_{}_su[{}];\n", k, idx, idx, k
            ));
        }
    }

    fn emit_sni_correction(code: &mut String, idx: usize, _pot: &PotentiometerIR, n: usize, _m: usize) {
        // S*N_i correction: v -= scale * (u^T . N_i . i_nl) * SU
        // where u^T . N_i is the state.pot_{idx}_u_ni vector
        // Uses full i_nl (not delta) for trapezoidal nonlinear integration
        // Uses runtime state fields (not constants) for sample-rate independence.
        code.push_str(&format!("    let mut u_ni_dot_inl_{} = 0.0;\n", idx));
        code.push_str(&format!(
            "    for _j in 0..M {{ u_ni_dot_inl_{idx} += state.pot_{idx}_u_ni[_j] * i_nl[_j]; }}\n"
        ));
        code.push_str(&format!(
            "    let sni_factor_{} = scale_{} * u_ni_dot_inl_{};\n", idx, idx, idx
        ));
        for k in 0..n {
            code.push_str(&format!(
                "    v[{}] -= sni_factor_{} * state.pot_{}_su[{}];\n", k, idx, idx, k
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

        // Compute v_d = p + K * i_nl (using pre-analyzed sparsity)
        code.push_str("        // Compute controlling voltages: v_d = p + K * i_nl\n");
        let has_pots = !ir.pots.is_empty();
        let vd_let = if has_pots { "let mut" } else { "let" };
        for i in 0..m {
            code.push_str(&format!("        {} v_d{} = ", vd_let, i));
            code.push_str(&format!("p[{}]", i));
            for &j in &ir.sparsity.k.nz_by_row[i] {
                code.push_str(&format!(" + state.k[{}][{}] * i_nl[{}]", i, j, j));
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
                    code.push_str(&format!("state.pot_{}_u_ni[{}] * i_nl[{}]", idx, j, j));
                    first = false;
                }
                code.push_str(";\n");
                // Apply correction to each v_d
                for i in 0..m {
                    code.push_str(&format!(
                        "        v_d{} -= k_scale_{} * state.pot_{}_nv_su[{}] * u_ni_dot_inl_{};\n",
                        i, idx, idx, i, idx
                    ));
                }
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
                        let d = dev_num;
                        code.push_str(&format!(
                            "        let i_dev{s} = bjt_ic(v_d{s}, v_d{s1}, DEVICE_{d}_IS, DEVICE_{d}_VT, DEVICE_{d}_BETA_R, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF);\n"
                        ));
                        code.push_str(&format!(
                            "        let i_dev{s1} = bjt_ib(v_d{s}, v_d{s1}, DEVICE_{d}_IS, DEVICE_{d}_VT, DEVICE_{d}_BETA_F, DEVICE_{d}_BETA_R, DEVICE_{d}_SIGN);\n"
                        ));
                        code.push_str(&format!(
                            "        let bjt{d}_jac = bjt_jacobian(v_d{s}, v_d{s1}, DEVICE_{d}_IS, DEVICE_{d}_VT, DEVICE_{d}_BETA_F, DEVICE_{d}_BETA_R, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF);\n"
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
                    DeviceType::Jfet => {
                        let s = slot.start_idx;
                        code.push_str(&format!(
                            "        let i_dev{} = jfet_id(v_d{}, DEVICE_{}_IDSS, DEVICE_{}_VP, DEVICE_{}_SIGN);\n",
                            s, s, dev_num, dev_num, dev_num
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = jfet_conductance(v_d{}, DEVICE_{}_IDSS, DEVICE_{}_VP, DEVICE_{}_SIGN);\n",
                            s, s, s, dev_num, dev_num, dev_num
                        ));
                    }
                    DeviceType::Tube => {
                        let s = slot.start_idx;
                        let s1 = s + 1;
                        let d = dev_num;
                        code.push_str(&format!(
                            "        let i_dev{s} = tube_ip(v_d{s}, v_d{s1}, DEVICE_{d}_MU, DEVICE_{d}_EX, DEVICE_{d}_KG1, DEVICE_{d}_KP, DEVICE_{d}_KVB);\n"
                        ));
                        code.push_str(&format!(
                            "        let i_dev{s1} = tube_ig(v_d{s}, DEVICE_{d}_IG_MAX, DEVICE_{d}_VGK_ONSET);\n"
                        ));
                        code.push_str(&format!(
                            "        let tube{d}_jac = tube_jacobian(v_d{s}, v_d{s1}, DEVICE_{d}_MU, DEVICE_{d}_EX, DEVICE_{d}_KG1, DEVICE_{d}_KP, DEVICE_{d}_KVB, DEVICE_{d}_IG_MAX, DEVICE_{d}_VGK_ONSET);\n"
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = tube{}_jac[0];\n", s, s, d
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = tube{}_jac[1];\n", s, s1, d
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = tube{}_jac[2];\n", s1, s, d
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = tube{}_jac[3];\n", s1, s1, d
                        ));
                    }
                }
            }
            code.push('\n');

            // Residuals
            code.push_str("        // Residuals: f(i) = i - i_dev(v(i)) = 0\n");
            for i in 0..m {
                code.push_str(&format!(
                    "        let f{} = i_nl[{}] - i_dev{};\n",
                    i, i, i
                ));
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
                                " - jdev_{}_{} * state.k[{}][{}]",
                                i, k, k, j
                            ));
                        } else {
                            // Use corrected K': state.k[k][j] - sum_pots(scale_p * nv_su_p[k] * u_ni_p[j])
                            let mut k_correction = String::new();
                            for (idx, _pot) in ir.pots.iter().enumerate() {
                                k_correction.push_str(&format!(
                                    " - k_scale_{} * state.pot_{}_nv_su[{}] * state.pot_{}_u_ni[{}]",
                                    idx, idx, k, idx, j
                                ));
                            }
                            terms.push_str(&format!(
                                " - jdev_{}_{} * (state.k[{}][{}]{})",
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
                3..=16 => {
                    Self::generate_gauss_elim(code, m);
                }
                _ => {
                    return Err(CodegenError::UnsupportedTopology(format!(
                        "M={} nonlinear dimensions not supported (max {})",
                        m, crate::dk::MAX_M
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
        code.push('\n');
        code.push_str("    i_nl\n");
        code.push_str("}\n\n");

        Ok(())
    }

    /// Generate inline Gaussian elimination for M=3..=8.
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
