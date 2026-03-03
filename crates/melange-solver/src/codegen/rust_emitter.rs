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

/// Coupled inductor data passed to Tera templates.
#[derive(Serialize)]
struct CoupledInductorTemplateData {
    name: String,
    l1_node_i: usize,
    l1_node_j: usize,
    l2_node_i: usize,
    l2_node_j: usize,
    l1_inductance: String,
    l2_inductance: String,
    coupling: String,
    g_self_1: String,
    g_self_2: String,
    g_mutual: String,
}

/// Switch component data passed to Tera templates.
#[derive(Serialize)]
struct SwitchCompTemplateData {
    node_p: usize,
    node_q: usize,
    nominal: String,
    comp_type: char,
    inductor_index: i64, // -1 if not an inductor
}

/// Switch data passed to Tera templates.
#[derive(Serialize)]
struct SwitchTemplateData {
    index: usize,
    num_positions: usize,
    num_components: usize,
    components: Vec<SwitchCompTemplateData>,
    position_rows: Vec<String>,
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

/// Build `CoupledInductorTemplateData` from IR coupled inductors.
fn coupled_inductor_template_data(ir: &CircuitIR) -> Vec<CoupledInductorTemplateData> {
    ir.coupled_inductors
        .iter()
        .map(|ci| CoupledInductorTemplateData {
            name: ci.name.clone(),
            l1_node_i: ci.l1_node_i,
            l1_node_j: ci.l1_node_j,
            l2_node_i: ci.l2_node_i,
            l2_node_j: ci.l2_node_j,
            l1_inductance: fmt_f64(ci.l1_inductance),
            l2_inductance: fmt_f64(ci.l2_inductance),
            coupling: fmt_f64(ci.coupling),
            g_self_1: fmt_f64(ci.g_self_1),
            g_self_2: fmt_f64(ci.g_self_2),
            g_mutual: fmt_f64(ci.g_mutual),
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
        for (i, &node) in ir.solver_config.output_nodes.iter().enumerate() {
            if node >= n {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= N={}",
                    i, node, n
                )));
            }
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

        let num_inductors = ir.inductors.len();
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, true));
        }
        let num_coupled_inductors = ir.coupled_inductors.len();
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
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
        let num_outputs = ir.solver_config.output_nodes.len();
        ctx.insert("num_outputs", &num_outputs);
        let output_nodes_values = ir.solver_config.output_nodes.iter()
            .map(|n| n.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        ctx.insert("output_nodes_values", &output_nodes_values);
        let output_scales_values = ir.solver_config.output_scales.iter()
            .map(|s| fmt_f64(*s))
            .collect::<Vec<_>>()
            .join(", ");
        ctx.insert("output_scales_values", &output_scales_values);
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

        // Switch constants
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);
        if num_switches > 0 {
            let switch_data: Vec<SwitchTemplateData> = ir.switches.iter().map(|sw| {
                let components: Vec<SwitchCompTemplateData> = sw.components.iter().map(|comp| {
                    SwitchCompTemplateData {
                        node_p: comp.node_p,
                        node_q: comp.node_q,
                        nominal: fmt_f64(comp.nominal_value),
                        comp_type: comp.component_type,
                        inductor_index: comp.inductor_index.map(|i| i as i64).unwrap_or(-1),
                    }
                }).collect();
                let position_rows: Vec<String> = sw.positions.iter().map(|pos| {
                    pos.iter().map(|v| fmt_f64(*v)).collect::<Vec<_>>().join(", ")
                }).collect();
                SwitchTemplateData {
                    index: sw.index,
                    num_positions: sw.num_positions,
                    num_components: sw.components.len(),
                    components,
                    position_rows,
                }
            }).collect();
            ctx.insert("switches", &switch_data);
        }

        // DC block coefficient: R = 1 - 2*pi*5/sr
        let internal_rate = ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
        let dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / internal_rate;
        ctx.insert("dc_block_r", &format!("{:.17e}", dc_block_r));
        ctx.insert("dc_op_converged", &ir.dc_op_converged);

        self.render("constants", &ctx)
    }

    fn emit_state(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        ctx.insert("has_dc_op", &ir.has_dc_op);
        let num_inductors = ir.inductors.len();
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
        let num_coupled_inductors = ir.coupled_inductors.len();
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
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

        // Switch data
        let num_switches = ir.switches.len();
        ctx.insert("num_switches", &num_switches);
        if num_switches > 0 {
            let switch_indices: Vec<usize> = (0..num_switches).collect();
            ctx.insert("switch_indices", &switch_indices);
            // Generate switch methods procedurally (too complex for Tera conditionals)
            let switch_methods = self.emit_switch_methods(ir);
            ctx.insert("switch_methods", &switch_methods);
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

    /// Generate switch setter methods and rebuild_matrices() procedurally.
    fn emit_switch_methods(&self, ir: &CircuitIR) -> String {
        let m = ir.topology.m;
        let num_pots = ir.pots.len();
        let num_inductors = ir.inductors.len();
        let os_factor = ir.solver_config.oversampling_factor;
        let mut code = String::new();

        // Emit set_switch_N() for each switch
        for sw in &ir.switches {
            code.push_str(&format!(
                "    /// Set switch {} position (0..{}).\n\
                 \x20   ///\n\
                 \x20   /// Triggers a full matrix rebuild. Call from UI thread, NOT audio thread.\n\
                 \x20   pub fn set_switch_{}(&mut self, position: usize) {{\n\
                 \x20       if position >= SWITCH_{}_NUM_POSITIONS {{ return; }}\n\
                 \x20       if self.switch_{}_position == position {{ return; }}\n\
                 \x20       self.switch_{}_position = position;\n\
                 \x20       self.rebuild_matrices();\n\
                 \x20   }}\n\n",
                sw.index,
                sw.num_positions - 1,
                sw.index, sw.index, sw.index, sw.index,
            ));
        }

        // Emit rebuild_matrices()
        code.push_str(
            "    /// Rebuild all sample-rate-dependent matrices from G/C constants.\n\
             \x20   ///\n\
             \x20   /// Applies switch position deltas to G/C, then rebuilds A, S, K, S*N_i.\n\
             \x20   /// Called by `set_switch_N()` and `set_sample_rate()`.\n\
             \x20   fn rebuild_matrices(&mut self) {\n\
             \x20       let internal_rate = self.current_sample_rate * OVERSAMPLING_FACTOR as f64;\n\
             \x20       let alpha = 2.0 * internal_rate;\n",
        );
        if num_inductors > 0 {
            code.push_str("        let t = 1.0 / internal_rate;\n");
        }

        // Start from constant G, C
        let has_r_switch = ir.switches.iter().any(|sw| sw.components.iter().any(|c| c.component_type == 'R'));
        let has_c_switch = ir.switches.iter().any(|sw| sw.components.iter().any(|c| c.component_type == 'C'));
        let g_mut = if has_r_switch { "mut " } else { "" };
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
                        // Inductors: don't modify G/C here; handled in inductor companion stamp below
                        code.push_str("            // Inductor: handled in companion model stamp below\n");
                        code.push_str("            let _ = new_val;\n");
                    }
                    _ => {}
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
             \x20       }\n\n\
             \x20       // Build A_neg = alpha * C_eff - G_eff\n\
             \x20       let mut a_neg = [[0.0f64; N]; N];\n\
             \x20       for i in 0..N {\n\
             \x20           for j in 0..N {\n\
             \x20               a_neg[i][j] = alpha * c_eff[i][j] - g_eff[i][j];\n\
             \x20           }\n\
             \x20       }\n",
        );

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
                    if switched { break; }
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
        let num_coupled = ir.coupled_inductors.len();
        if num_coupled > 0 {
            if num_inductors == 0 {
                code.push_str("        let t = 1.0 / internal_rate;\n");
            }
            code.push_str("\n        // Add coupled inductor companion model conductances\n");
            for (ci_idx, ci) in ir.coupled_inductors.iter().enumerate() {
                code.push_str(&format!(
                    "        {{\n\
                     \x20           let m_val = CI_{ci}_COUPLING * (CI_{ci}_L1_INDUCTANCE * CI_{ci}_L2_INDUCTANCE).sqrt();\n\
                     \x20           let det = CI_{ci}_L1_INDUCTANCE * CI_{ci}_L2_INDUCTANCE - m_val * m_val;\n\
                     \x20           let half_t = t / 2.0;\n\
                     \x20           let gs1 = half_t * CI_{ci}_L2_INDUCTANCE / det;\n\
                     \x20           let gs2 = half_t * CI_{ci}_L1_INDUCTANCE / det;\n\
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

        // Pot SM recomputation
        if num_pots > 0 {
            code.push_str("\n        // Recompute Sherman-Morrison vectors for pots\n");
            for idx in 0..num_pots {
                let pot = &ir.pots[idx];
                code.push_str(&format!(
                    "        {{\n\
                     \x20           let mut u = [0.0f64; N];\n\
                     \x20           if POT_{idx}_NODE_P > 0 {{ u[POT_{idx}_NODE_P - 1] = 1.0; }}\n\
                     \x20           if POT_{idx}_NODE_Q > 0 {{ u[POT_{idx}_NODE_Q - 1] = -1.0; }}\n\
                     \x20           let mut su = [0.0f64; N];\n\
                     \x20           for i in 0..N {{ let mut sum = 0.0; for j in 0..N {{ sum += self.s[i][j] * u[j]; }} su[i] = sum; }}\n\
                     \x20           let mut usu = 0.0f64;\n\
                     \x20           for i in 0..N {{ usu += u[i] * su[i]; }}\n",
                    idx = idx,
                ));
                if m > 0 {
                    code.push_str(&format!(
                        "            let mut nv_su = [0.0f64; M];\n\
                         \x20           for i in 0..M {{ let mut sum = 0.0; for j in 0..N {{ sum += N_V[i][j] * su[j]; }} nv_su[i] = sum; }}\n\
                         \x20           let mut u_ni = [0.0f64; M];\n\
                         \x20           for j in 0..M {{ let mut sum = 0.0; for i in 0..N {{ sum += su[i] * N_I[j][i]; }} u_ni[j] = sum; }}\n\
                         \x20           self.pot_{idx}_nv_su = nv_su;\n\
                         \x20           self.pot_{idx}_u_ni = u_ni;\n",
                        idx = idx,
                    ));
                } else {
                    // Still need to declare nv_su/u_ni even if M=0
                    let _ = pot;
                }
                code.push_str(&format!(
                    "            self.pot_{idx}_su = su;\n\
                     \x20           self.pot_{idx}_usu = usu;\n\
                     \x20       }}\n",
                    idx = idx,
                ));
            }
        }

        // Reset oversampler state
        if os_factor > 1 {
            let os_info = oversampling_info(os_factor);
            code.push_str(&format!(
                "\n        self.os_up_state = [0.0; {}];\n\
                 \x20       self.os_dn_state = [[0.0; {}]; NUM_OUTPUTS];\n",
                os_info.state_size, os_info.state_size,
            ));
            if os_factor == 4 {
                code.push_str(&format!(
                    "        self.os_up_state_outer = [0.0; {}];\n\
                     \x20       self.os_dn_state_outer = [[0.0; {}]; NUM_OUTPUTS];\n",
                    os_info.state_size_outer, os_info.state_size_outer,
                ));
            }
        }

        // DC block recomputation
        code.push_str(&format!(
            "\n        // Recompute DC blocking coefficient\n\
             \x20       let internal_rate = self.current_sample_rate * {}.0;\n\
             \x20       self.dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / internal_rate;\n\
             \x20       self.dc_block_x_prev = [0.0; NUM_OUTPUTS];\n\
             \x20       self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n",
            os_factor,
        ));

        code.push_str("    }\n");
        code
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
                 #[allow(dead_code)]\n\
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
        let num_coupled_inductors = ir.coupled_inductors.len();
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
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
        let num_coupled_inductors = ir.coupled_inductors.len();
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }

        let os_factor = ir.solver_config.oversampling_factor;
        ctx.insert("oversampling_factor", &os_factor);

        let num_outputs = ir.solver_config.output_nodes.len();
        ctx.insert("num_outputs", &num_outputs);

        ctx.insert("max_iter", &ir.solver_config.max_iterations);
        ctx.insert("m", &ir.topology.m);

        let num_pots = ir.pots.len();
        ctx.insert("num_pots", &num_pots);
        let pot_defaults: Vec<String> = ir.pots.iter().map(|p| fmt_f64(1.0 / p.g_nominal)).collect();
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

            // K_eff: precomputed corrected K matrix for NR solver (M > 0 with pots)
            let use_k_eff = m > 0;
            ctx.insert("use_k_eff", &use_k_eff);
            if use_k_eff {
                let k_eff_setup = Self::emit_k_eff_setup(&ir.pots, m);
                ctx.insert("k_eff_setup", &k_eff_setup);
            }
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
            Self::emit_2x_wrapper(&mut code, num_outputs);
        } else if factor == 4 {
            Self::emit_4x_wrapper(&mut code, num_outputs);
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
    fn emit_2x_wrapper(code: &mut String, num_outputs: usize) {
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
        code.push_str("        result[out_idx] = if v.is_finite() { v.clamp(-10.0, 10.0) } else { 0.0 };\n");
        code.push_str("    }\n");
        code.push_str("    result\n");
        let _ = num_outputs; // used for signature type
    }

    /// Emit the 4x oversampling wrapper body (cascaded 2x stages).
    fn emit_4x_wrapper(code: &mut String, num_outputs: usize) {
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
        code.push_str("        result[out_idx] = if v.is_finite() { v.clamp(-10.0, 10.0) } else { 0.0 };\n");
        code.push_str("    }\n");
        code.push_str("    result\n");
        let _ = num_outputs; // used for signature type
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
        // S correction using sequentially corrected local su_c{idx} and scale_c{idx}.
        code.push_str(&format!("    let mut su_dot_rhs_{} = 0.0;\n", idx));
        code.push_str(&format!(
            "    for _k in 0..N {{ su_dot_rhs_{idx} += su_c{idx}[_k] * rhs[_k]; }}\n"
        ));

        code.push_str(&format!(
            "    let factor_{} = scale_c{} * su_dot_rhs_{};\n", idx, idx, idx
        ));
        for k in 0..n {
            code.push_str(&format!(
                "    v_pred[{}] -= factor_{} * su_c{}[{}];\n", k, idx, idx, k
            ));
        }
    }

    fn emit_sni_correction(code: &mut String, idx: usize, _pot: &PotentiometerIR, n: usize, _m: usize) {
        // S*N_i correction using sequentially corrected local vectors.
        code.push_str(&format!("    let mut u_ni_dot_inl_{} = 0.0;\n", idx));
        code.push_str(&format!(
            "    for _j in 0..M {{ u_ni_dot_inl_{idx} += u_ni_c{idx}[_j] * i_nl[_j]; }}\n"
        ));
        code.push_str(&format!(
            "    let sni_factor_{} = scale_c{} * u_ni_dot_inl_{};\n", idx, idx, idx
        ));
        for k in 0..n {
            code.push_str(&format!(
                "    v[{}] -= sni_factor_{} * su_c{}[{}];\n", k, idx, idx, k
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
            format!("su_c{}[{}] - su_c{}[{}]", j, pot_k.node_p - 1, j, pot_k.node_q - 1)
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
            format!("su_c{}[{}] - su_c{}[{}]", k, pot_k.node_p - 1, k, pot_k.node_q - 1)
        } else if pot_k.node_p > 0 {
            format!("su_c{}[{}]", k, pot_k.node_p - 1)
        } else if pot_k.node_q > 0 {
            format!("-su_c{}[{}]", k, pot_k.node_q - 1)
        } else {
            "0.0".to_string()
        }
    }

    /// Emit K_eff setup: precomputed corrected K matrix incorporating all pot SM corrections.
    /// K_eff = K - Σ_k scale_ck * nv_su_ck ⊗ u_ni_ck
    fn emit_k_eff_setup(pots: &[PotentiometerIR], m: usize) -> String {
        let mut code = String::new();
        code.push_str("    // Precompute corrected K matrix: K_eff = K - Σ_k scale_ck * nv_su_ck ⊗ u_ni_ck\n");
        code.push_str("    let mut k_eff = state.k;\n");
        for k in 0..pots.len() {
            for i in 0..m {
                for j in 0..m {
                    code.push_str(&format!(
                        "    k_eff[{i}][{j}] -= scale_c{k} * nv_su_c{k}[{i}] * u_ni_c{k}[{j}];\n"
                    ));
                }
            }
        }
        code
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
        let has_pots = !ir.pots.is_empty();
        code.push_str("#[inline(always)]\n");
        if has_pots && m > 0 {
            code.push_str(
                "fn solve_nonlinear(p: &[f64; M], k_eff: &[[f64; M]; M], state: &mut CircuitState) -> [f64; M] {\n"
            );
        } else {
            code.push_str(
                "fn solve_nonlinear(p: &[f64; M], state: &mut CircuitState) -> [f64; M] {\n"
            );
        }
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
        if has_pots && m > 0 {
            // Use precomputed K_eff (corrected for all pots via sequential SM)
            code.push_str("        // Compute controlling voltages: v_d = p + K_eff * i_nl\n");
            for i in 0..m {
                code.push_str(&format!("        let v_d{} = p[{}]", i, i));
                for j in 0..m {
                    code.push_str(&format!(" + k_eff[{}][{}] * i_nl[{}]", i, j, j));
                }
                code.push_str(";\n");
            }
        } else {
            // Use base K matrix with sparsity optimization
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
                        if has_pots {
                            // Use precomputed k_eff (corrected for all pots)
                            terms.push_str(&format!(
                                " - jdev_{}_{} * k_eff[{}][{}]",
                                i, k, k, j
                            ));
                        } else {
                            terms.push_str(&format!(
                                " - jdev_{}_{} * state.k[{}][{}]",
                                i, k, k, j
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

    /// Generate inline Gaussian elimination for M=3..=16.
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
