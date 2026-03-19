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

/// Transformer group data passed to Tera templates.
#[derive(Serialize)]
struct TransformerGroupTemplateData {
    index: usize,
    name: String,
    num_windings: usize,
    winding_node_i: Vec<usize>,
    winding_node_j: Vec<usize>,
    inductances: Vec<String>,
    coupling_flat: Vec<String>,
    y_matrix: Vec<String>,
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

/// Device parameter data passed to Tera templates for runtime-adjustable state fields.
///
/// Each device slot produces one entry. The `params` vec contains (field_suffix, const_name)
/// pairs for parameters that should become CircuitState fields.
/// SIGN, USE_GP, and Gummel-Poon params (VAF/VAR/IKF/IKR) remain as const-only.
#[derive(Serialize)]
struct DeviceParamTemplateData {
    /// Device index (0-based)
    dev_num: usize,
    /// Device type tag for conditional template logic
    device_type: String,
    /// Runtime-adjustable parameter entries: (lowercase_field_suffix, CONST_SUFFIX)
    params: Vec<DeviceParamEntry>,
}

/// A single runtime-adjustable device parameter.
#[derive(Serialize)]
struct DeviceParamEntry {
    /// Lowercase field name suffix, e.g. "is", "n_vt", "bf"
    field_suffix: String,
    /// Uppercase const name suffix, e.g. "IS", "N_VT", "BETA_F"
    const_suffix: String,
}

/// Data for a BJT with self-heating enabled (passed to Tera templates).
#[derive(Serialize)]
struct SelfHeatingDeviceData {
    /// Device index (0-based)
    dev_num: usize,
    /// Start index in M-dimensional NR space (Vbe/Ic slot)
    start_idx: usize,
}

/// Collect BJT devices that have self-heating enabled.
fn self_heating_device_data(ir: &CircuitIR) -> Vec<SelfHeatingDeviceData> {
    ir.device_slots
        .iter()
        .enumerate()
        .filter_map(|(dev_num, slot)| {
            if let DeviceParams::Bjt(bp) = &slot.params {
                if bp.has_self_heating() {
                    return Some(SelfHeatingDeviceData {
                        dev_num,
                        start_idx: slot.start_idx,
                    });
                }
            }
            None
        })
        .collect()
}

/// Build `DeviceParamTemplateData` for each device slot in the IR.
///
/// Only parameters that should be runtime-adjustable are included.
/// SIGN constants and Gummel-Poon parameters (VAF, VAR, IKF, IKR, USE_GP)
/// are intentionally excluded — they remain as compile-time constants.
fn device_param_template_data(ir: &CircuitIR) -> Vec<DeviceParamTemplateData> {
    ir.device_slots
        .iter()
        .enumerate()
        .map(|(dev_num, slot)| {
            let (device_type, params) = match &slot.params {
                DeviceParams::Diode(_) => (
                    "Diode".to_string(),
                    vec![
                        DeviceParamEntry { field_suffix: "is".into(), const_suffix: "IS".into() },
                        DeviceParamEntry { field_suffix: "n_vt".into(), const_suffix: "N_VT".into() },
                    ],
                ),
                DeviceParams::Bjt(_) => (
                    "Bjt".to_string(),
                    vec![
                        DeviceParamEntry { field_suffix: "is".into(), const_suffix: "IS".into() },
                        DeviceParamEntry { field_suffix: "vt".into(), const_suffix: "VT".into() },
                        DeviceParamEntry { field_suffix: "bf".into(), const_suffix: "BETA_F".into() },
                        DeviceParamEntry { field_suffix: "br".into(), const_suffix: "BETA_R".into() },
                    ],
                ),
                DeviceParams::Jfet(_) => (
                    "Jfet".to_string(),
                    vec![
                        DeviceParamEntry { field_suffix: "idss".into(), const_suffix: "IDSS".into() },
                        DeviceParamEntry { field_suffix: "vp".into(), const_suffix: "VP".into() },
                        DeviceParamEntry { field_suffix: "lambda".into(), const_suffix: "LAMBDA".into() },
                    ],
                ),
                DeviceParams::Mosfet(_) => (
                    "Mosfet".to_string(),
                    vec![
                        DeviceParamEntry { field_suffix: "kp".into(), const_suffix: "KP".into() },
                        DeviceParamEntry { field_suffix: "vt".into(), const_suffix: "VT".into() },
                        DeviceParamEntry { field_suffix: "lambda".into(), const_suffix: "LAMBDA".into() },
                    ],
                ),
                DeviceParams::Tube(_) => (
                    "Tube".to_string(),
                    vec![
                        DeviceParamEntry { field_suffix: "mu".into(), const_suffix: "MU".into() },
                        DeviceParamEntry { field_suffix: "ex".into(), const_suffix: "EX".into() },
                        DeviceParamEntry { field_suffix: "kg1".into(), const_suffix: "KG1".into() },
                        DeviceParamEntry { field_suffix: "kp".into(), const_suffix: "KP".into() },
                        DeviceParamEntry { field_suffix: "kvb".into(), const_suffix: "KVB".into() },
                        DeviceParamEntry { field_suffix: "ig_max".into(), const_suffix: "IG_MAX".into() },
                        DeviceParamEntry { field_suffix: "vgk_onset".into(), const_suffix: "VGK_ONSET".into() },
                        DeviceParamEntry { field_suffix: "lambda".into(), const_suffix: "LAMBDA".into() },
                    ],
                ),
            };
            DeviceParamTemplateData { dev_num, device_type, params }
        })
        .collect()
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

/// Build `TransformerGroupTemplateData` from IR transformer groups.
fn transformer_group_template_data(ir: &CircuitIR) -> Vec<TransformerGroupTemplateData> {
    ir.transformer_groups
        .iter()
        .enumerate()
        .map(|(idx, g)| TransformerGroupTemplateData {
            index: idx,
            name: g.name.clone(),
            num_windings: g.num_windings,
            winding_node_i: g.winding_node_i.clone(),
            winding_node_j: g.winding_node_j.clone(),
            inductances: g.inductances.iter().map(|v| fmt_f64(*v)).collect(),
            coupling_flat: g.coupling_flat.iter().map(|v| fmt_f64(*v)).collect(),
            y_matrix: g.y_matrix.iter().map(|v| fmt_f64(*v)).collect(),
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
const TMPL_DEVICE_MOSFET: &str = include_str!("../../templates/rust/device_mosfet.rs.tera");
const TMPL_DEVICE_TUBE: &str = include_str!("../../templates/rust/device_tube.rs.tera");
const TMPL_BUILD_RHS: &str = include_str!("../../templates/rust/build_rhs.rs.tera");
const TMPL_MAT_VEC_MUL_S: &str = include_str!("../../templates/rust/mat_vec_mul_s.rs.tera");
const TMPL_EXTRACT_VOLTAGES: &str = include_str!("../../templates/rust/extract_voltages.rs.tera");
const TMPL_FINAL_VOLTAGES: &str = include_str!("../../templates/rust/final_voltages.rs.tera");
const TMPL_UPDATE_HISTORY: &str = include_str!("../../templates/rust/update_history.rs.tera");
const TMPL_PROCESS_SAMPLE: &str = include_str!("../../templates/rust/process_sample.rs.tera");
const TMPL_SPICE_LIMITING: &str = include_str!("../../templates/rust/spice_limiting.rs.tera");

/// Rust language emitter.
pub struct RustEmitter {
    tera: Tera,
}

impl RustEmitter {
    pub fn new() -> Result<Self, CodegenError> {
        let mut tera = Tera::default();
        tera.add_raw_templates(vec![
            ("header", TMPL_HEADER),
            ("constants", TMPL_CONSTANTS),
            ("state", TMPL_STATE),
            ("device_diode", TMPL_DEVICE_DIODE),
            ("device_bjt", TMPL_DEVICE_BJT),
            ("device_jfet", TMPL_DEVICE_JFET),
            ("device_mosfet", TMPL_DEVICE_MOSFET),
            ("device_tube", TMPL_DEVICE_TUBE),
            ("build_rhs", TMPL_BUILD_RHS),
            ("mat_vec_mul_s", TMPL_MAT_VEC_MUL_S),
            ("extract_voltages", TMPL_EXTRACT_VOLTAGES),
            ("final_voltages", TMPL_FINAL_VOLTAGES),
            ("update_history", TMPL_UPDATE_HISTORY),
            ("process_sample", TMPL_PROCESS_SAMPLE),
            ("spice_limiting", TMPL_SPICE_LIMITING),
        ])
        .map_err(|e| CodegenError::TemplateError(format!("template init: {}", e)))?;
        Ok(Self { tera })
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
        use super::ir::SolverMode;

        let n = ir.topology.n;
        // n_nodes: original circuit node count. Fallback to n for backward compat (n_nodes=0 in old data).
        let n_nodes = if ir.topology.n_nodes > 0 { ir.topology.n_nodes } else { n };

        // Validate node indices against original circuit nodes (not augmented dimension)
        if ir.solver_config.input_node >= n_nodes {
            return Err(CodegenError::InvalidConfig(format!(
                "input_node {} >= n_nodes={}",
                ir.solver_config.input_node, n_nodes
            )));
        }
        for (i, &node) in ir.solver_config.output_nodes.iter().enumerate() {
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={}",
                    i, node, n_nodes
                )));
            }
        }

        match ir.solver_mode {
            SolverMode::Dk => self.emit_dk(ir),
            SolverMode::Nodal => self.emit_nodal(ir),
        }
    }
}

impl RustEmitter {
    /// Emit DK-method generated code (original path).
    fn emit_dk(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = String::new();

        code.push_str(&self.emit_header(ir)?);
        code.push_str(&self.emit_constants(ir)?);
        code.push_str(&self.emit_pot_constants(ir));
        code.push_str(&self.emit_state(ir)?);
        code.push_str(&Self::emit_transformer_group_helpers(ir));
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
        // n_nodes: original circuit node count (before augmented VS/VCVS variables).
        // Used to zero out augmented rows in A_neg during rebuild_matrices.
        let n_nodes = if ir.topology.n_nodes > 0 { ir.topology.n_nodes } else { n };
        ctx.insert("n_nodes", &n_nodes);
        let has_augmented = n_nodes < n;
        ctx.insert("has_augmented", &has_augmented);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("n_aug", &ir.topology.n_aug);

        // When augmented_inductors is true, companion model constants (IND_*_G_EQ,
        // CI_*_G_SELF/MUTUAL, XFMR_*_Y) are not needed. The G/C matrices already
        // contain inductor stamps and A_neg handles history.
        let num_inductors = if ir.topology.augmented_inductors { 0 } else { ir.inductors.len() };
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, true));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors { 0 } else { ir.coupled_inductors.len() };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors { 0 } else { ir.transformer_groups.len() };
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
        ctx.insert("dc_block", &ir.dc_block);
        ctx.insert("dc_op_converged", &ir.dc_op_converged);

        self.render("constants", &ctx)
    }

    fn emit_state(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        ctx.insert("has_dc_op", &ir.has_dc_op);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        ctx.insert("n_aug", &ir.topology.n_aug);
        ctx.insert("n_nodes", &ir.topology.n_nodes);
        // When augmented_inductors is true, companion model state (ind_i_prev, ci_i_hist,
        // xfmr_y, etc.) is not needed. A_neg handles history through augmented G/C.
        let num_inductors = if ir.topology.augmented_inductors { 0 } else { ir.inductors.len() };
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
        let num_coupled_inductors = if ir.topology.augmented_inductors { 0 } else { ir.coupled_inductors.len() };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors { 0 } else { ir.transformer_groups.len() };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            ctx.insert("transformer_groups", &transformer_group_template_data(ir));

            // Generate set_sample_rate recomputation lines procedurally
            let mut xfmr_ssr_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                xfmr_ssr_lines.push_str(&format!("        {{\n\
                     \x20           let half_t = t / 2.0;\n"));
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
                    if i > 0 { xfmr_ssr_lines.push_str(", "); }
                    xfmr_ssr_lines.push('[');
                    for j in 0..w {
                        if j > 0 { xfmr_ssr_lines.push_str(", "); }
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
                        if i == j { continue; }
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

        self.render("state", &ctx)
    }

    fn emit_device_models(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = section_banner("DEVICE MODELS");

        let mut has_diode = false;
        let mut has_bjt = false;
        let mut has_jfet = false;
        let mut has_mosfet = false;
        let mut has_tube = false;
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
                    if bp.has_self_heating() { has_bjt_self_heating = true; }
                    emit_device_const(&mut code, dev_num, "IS", bp.is);
                    emit_device_const(&mut code, dev_num, "VT", bp.vt);
                    emit_device_const(&mut code, dev_num, "BETA_F", bp.beta_f);
                    emit_device_const(&mut code, dev_num, "BETA_R", bp.beta_r);
                    emit_device_const(&mut code, dev_num, "NF", bp.nf);
                    emit_device_const(&mut code, dev_num, "ISE", bp.ise);
                    emit_device_const(&mut code, dev_num, "NE", bp.ne);
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
                        emit_device_const(&mut code, dev_num, "RS_PARAM", jp.rs_param);
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
                        emit_device_const(&mut code, dev_num, "RS_PARAM", mp.rs_param);
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
                    // Precomputed critical voltage for SPICE pnjlim (grid current onset)
                    let vt_tube = tp.vgk_onset / 3.0;
                    let vcrit = vt_tube * (vt_tube / (std::f64::consts::SQRT_2 * 1e-10)).ln();
                    emit_device_const(&mut code, dev_num, "VCRIT", vcrit);
                    code.push('\n');
                }
            }
        }

        // Boltzmann constant / elementary charge (k/q in eV/K)
        if has_bjt_self_heating {
            code.push_str("/// Boltzmann constant / elementary charge [eV/K]\n");
            code.push_str("const BOLTZMANN_Q: f64 = 8.617333262e-5;\n\n");
        }

        // Fast exp() approximation (needed by diode, BJT, tube device models)
        if has_diode || has_bjt || has_tube {
            code.push_str(&Self::emit_fast_exp());
        }

        // SPICE voltage limiting functions (needed by all nonlinear devices)
        if has_diode || has_bjt || has_jfet || has_mosfet || has_tube {
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
            code.push_str(&self.render("device_tube", &Context::new())?);
        }

        Ok(code)
    }

    /// Emit fast exp() approximation function.
    ///
    /// Provides two implementations selectable at compile time via `MELANGE_FAST_EXP`:
    /// 1. Default: `x.clamp().exp()` -- uses hardware/libm exp, optimal on x86-64 with glibc
    /// 2. Polynomial: range reduction + 5th-order minimax -- no libm dependency, optimal on
    ///    ARM, WASM, or platforms without fast hardware exp
    ///
    /// Accuracy of polynomial path: <0.0004% max relative error over [-40, 40].
    /// Both paths produce identical results to within 0.0004%.
    fn emit_fast_exp() -> String {
        let mut code = String::new();
        code.push_str(
            "/// Fast exp() for audio circuit simulation.\n\
             /// Input clamped to [-40, 40] (matches melange safe_exp convention).\n\
             ///\n\
             /// To use the polynomial approximation (faster on ARM/WASM, no libm dependency),\n\
             /// compile with: `--cfg melange_fast_exp`\n\
             #[inline(always)]\n\
             fn fast_exp(x: f64) -> f64 {\n\
             \x20   #[cfg(not(melange_fast_exp))]\n\
             \x20   { x.clamp(-40.0, 40.0).exp() }\n\
             \x20   #[cfg(melange_fast_exp)]\n\
             \x20   {\n\
             \x20       // Range reduction + 5th-order minimax polynomial. <0.0004% max relative error.\n\
             \x20       // No lookup tables, no libm dependency, branchless hot path.\n\
             \x20       let x = x.clamp(-40.0, 40.0);\n\
             \x20       const LN2_INV: f64 = 1.4426950408889634;\n\
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
        code
    }

    /// Generate switch setter methods and rebuild_matrices() procedurally.
    fn emit_switch_methods(&self, ir: &CircuitIR) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 { ir.topology.n_nodes } else { n };
        let num_pots = ir.pots.len();
        let num_inductors = if ir.topology.augmented_inductors { 0 } else { ir.inductors.len() };
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

        // Zero augmented rows in A_neg (algebraic constraints for VS/VCVS)
        // When augmented_inductors, only zero n_nodes..n_aug (not inductor rows)
        let n_aug = ir.topology.n_aug;
        let a_neg_zero_end = if ir.topology.augmented_inductors { n_aug } else { n };
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
        let num_coupled = if ir.topology.augmented_inductors { 0 } else { ir.coupled_inductors.len() };
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

        // Transformer group companion stamps
        let num_xfmr_groups = if ir.topology.augmented_inductors { 0 } else { ir.transformer_groups.len() };
        if num_xfmr_groups > 0 {
            if num_inductors == 0 && num_coupled == 0 {
                code.push_str("        let t = 1.0 / internal_rate;\n");
            }
            code.push_str("\n        // Add transformer group companion model conductances\n");
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                let w = g.num_windings;
                // Build L matrix from inductances and couplings, invert, multiply by T/2
                code.push_str(&format!(
                    "        {{\n\
                     \x20           let half_t = t / 2.0;\n\
                     \x20           // Build inductance matrix L[i][j] = k[i][j] * sqrt(Li*Lj)\n"
                ));
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
                code.push_str(&format!(
                    "            let y = invert_xfmr_{gi}(["));
                for i in 0..w {
                    if i > 0 { code.push_str(", "); }
                    code.push('[');
                    for j in 0..w {
                        if j > 0 { code.push_str(", "); }
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
                        if i == j { continue; }
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
        if ir.dc_block {
            code.push_str(&format!(
                "\n        // Recompute DC blocking coefficient\n\
                 \x20       let internal_rate = self.current_sample_rate * {}.0;\n\
                 \x20       self.dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / internal_rate;\n\
                 \x20       self.dc_block_x_prev = [0.0; NUM_OUTPUTS];\n\
                 \x20       self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n",
                os_factor,
            ));
        }

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

    fn emit_build_rhs(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();

        ctx.insert("has_dc_sources", &ir.has_dc_sources);
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);

        // When augmented_inductors is true, companion model history is handled by A_neg,
        // so num_inductors/num_coupled_inductors/num_transformer_groups should be 0
        // for the build_rhs template (no history current injection).
        let num_inductors = if ir.topology.augmented_inductors { 0 } else { ir.inductors.len() };
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, false));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors { 0 } else { ir.coupled_inductors.len() };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors { 0 } else { ir.transformer_groups.len() };
        ctx.insert("num_transformer_groups", &num_transformer_groups);
        if num_transformer_groups > 0 {
            let mut xfmr_rhs_lines = String::new();
            for (gi, g) in ir.transformer_groups.iter().enumerate() {
                for k in 0..g.num_windings {
                    if g.winding_node_i[k] > 0 {
                        xfmr_rhs_lines.push_str(&format!(
                            "    rhs[{}] -= state.xfmr_{}_i_hist[{}];\n",
                            g.winding_node_i[k] - 1, gi, k
                        ));
                    }
                    if g.winding_node_j[k] > 0 {
                        xfmr_rhs_lines.push_str(&format!(
                            "    rhs[{}] += state.xfmr_{}_i_hist[{}];\n",
                            g.winding_node_j[k] - 1, gi, k
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
        let mut ctx = Context::new();
        ctx.insert("augmented_inductors", &ir.topology.augmented_inductors);
        // When augmented_inductors is true, companion model state update is not needed —
        // A_neg handles all inductor history through the augmented G/C matrices.
        let num_inductors = if ir.topology.augmented_inductors { 0 } else { ir.inductors.len() };
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            ctx.insert("inductors", &inductor_template_data(ir, false));
        }
        let num_coupled_inductors = if ir.topology.augmented_inductors { 0 } else { ir.coupled_inductors.len() };
        ctx.insert("num_coupled_inductors", &num_coupled_inductors);
        if num_coupled_inductors > 0 {
            ctx.insert("coupled_inductors", &coupled_inductor_template_data(ir));
        }
        let num_transformer_groups = if ir.topology.augmented_inductors { 0 } else { ir.transformer_groups.len() };
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
                    xfmr_update_lines.push_str(&format!(
                        "        let v_new_{k} = {v_i} - {v_j};\n"
                    ));
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
                    xfmr_update_lines.push_str(&format!(
                        "        state.xfmr_{gi}_i_hist[{k}] = i_new_{k}"
                    ));
                    for j in 0..w {
                        xfmr_update_lines.push_str(&format!(
                            " - state.xfmr_{gi}_y[{}] * v_new_{j}",
                            k * w + j,
                        ));
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

        // MOSFET body effect: compute VT_eff from v_pred before NR (DK path only)
        let mut body_effect_update = String::new();
        if ir.solver_mode == super::ir::SolverMode::Dk {
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

/// Emit damped fallback for singular Jacobian: half-step on residual, clamped.
fn emit_nr_singular_fallback(code: &mut String, dim: usize, indent: &str) {
    code.push_str(&format!("{indent}// Singular Jacobian — damped fallback (0.5 * residual)\n"));
    for i in 0..dim {
        code.push_str(&format!(
            "{indent}i_nl[{i}] -= (f{i} * 0.5).clamp(-0.01, 0.01);\n"
        ));
    }
}

/// Emit SPICE-style voltage-space limiting and convergence check for NR.
///
/// After computing Newton deltas (delta0..delta{dim-1}), converts to voltage
/// space via K matrix, applies per-device pnjlim/fetlim, and uses scalar
/// damping to maintain current-space NR consistency.
///
/// Assumes `delta0..delta{dim-1}` and `v_d0..v_d{dim-1}` are in scope.
fn emit_nr_limit_and_converge(
    code: &mut String,
    ir: &CircuitIR,
    dim: usize,
    indent: &str,
    has_pots: bool,
) {
    // Compute implied voltage changes: dv[i] = -sum_j K[i][j] * delta[j]
    code.push_str(&format!("{indent}// Voltage-space limiting (SPICE pnjlim/fetlim through K matrix)\n"));
    for i in 0..dim {
        code.push_str(&format!("{indent}let dv{i} = -("));
        let mut first = true;
        if has_pots {
            for j in 0..dim {
                if !first { code.push_str(" + "); }
                code.push_str(&format!("k_eff[{i}][{j}] * delta{j}"));
                first = false;
            }
        } else {
            // Use sparsity info when available
            for j in 0..dim {
                if !first { code.push_str(" + "); }
                code.push_str(&format!("state.k[{i}][{j}] * delta{j}"));
                first = false;
            }
        }
        code.push_str(");\n");
    }

    // Compute scalar damping factor from per-device limiting
    code.push_str(&format!("{indent}let mut alpha = 1.0_f64;\n"));
    for (dev_num, slot) in ir.device_slots.iter().enumerate() {
        for d in 0..slot.dimension {
            let i = slot.start_idx + d;
            code.push_str(&format!("{indent}if dv{i}.abs() > 1e-15 {{\n"));
            // Emit per-device limiter call based on device type and dimension
            match (&slot.device_type, d) {
                (DeviceType::Diode, _) => {
                    code.push_str(&format!(
                        "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_VCRIT);\n"
                    ));
                }
                (DeviceType::Bjt, _) | (DeviceType::BjtForwardActive, _) => {
                    // PN junction limiting (Vbe for forward-active, both Vbe/Vbc for full)
                    code.push_str(&format!(
                        "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vt, DEVICE_{dev_num}_VCRIT);\n"
                    ));
                }
                (DeviceType::Jfet, 0) => {
                    // dim 0 = Vds — generous limiting
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, 0.0);\n"
                    ));
                }
                (DeviceType::Jfet, _) => {
                    // dim 1 = Vgs — limit around pinch-off voltage
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vp);\n"
                    ));
                }
                (DeviceType::Mosfet, 0) => {
                    // dim 0 = Vds — generous limiting
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, 0.0);\n"
                    ));
                }
                (DeviceType::Mosfet, _) => {
                    // dim 1 = Vgs — limit around threshold voltage
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vt);\n"
                    ));
                }
                (DeviceType::Tube, 0) => {
                    code.push_str(&format!(
                        "{indent}    let v_lim = pnjlim(v_d{i} + dv{i}, v_d{i}, state.device_{dev_num}_vgk_onset / 3.0, DEVICE_{dev_num}_VCRIT);\n"
                    ));
                }
                (DeviceType::Tube, _) => {
                    code.push_str(&format!(
                        "{indent}    let v_lim = fetlim(v_d{i} + dv{i}, v_d{i}, 0.0);\n"
                    ));
                }
            }
            code.push_str(&format!(
                "{indent}    let ratio = ((v_lim - v_d{i}) / dv{i}).max(0.01);\n"
            ));
            code.push_str(&format!("{indent}    if ratio < alpha {{ alpha = ratio; }}\n"));
            code.push_str(&format!("{indent}}}\n"));
        }
    }

    // Apply damped step
    for i in 0..dim {
        code.push_str(&format!("{indent}i_nl[{i}] -= alpha * delta{i};\n"));
    }

    // Convergence check on actual step taken
    code.push_str(&format!("\n{indent}// Convergence check (max-norm on actual step)\n"));
    code.push_str(&format!("{indent}if "));
    for i in 0..dim {
        if i > 0 {
            code.push_str(".max(");
        }
        code.push_str(&format!("(alpha * delta{i}).abs()"));
        if i > 0 {
            code.push(')');
        }
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
        code.push_str("    const SINGULARITY_THRESHOLD: f64 = 1e-15;\n\n");

        code.push_str("    // First-order predictor for NR warm start: i_guess = 2*i_nl[n-1] - i_nl[n-2]\n");
        code.push_str("    // Extrapolates the trend from the last two samples. On smooth signals this\n");
        code.push_str("    // puts the initial guess much closer to the solution, reducing NR iterations.\n");
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
                        let d = dev_num;
                        let dp = match &slot.params { DeviceParams::Diode(dp) => dp, _ => unreachable!() };
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
                        let bp = match &slot.params { DeviceParams::Bjt(bp) => bp, _ => unreachable!() };
                        if bp.has_parasitics() {
                            // Use inner 2D NR for parasitic resistances
                            code.push_str(&format!(
                                "        let (i_dev{s}, i_dev{s1}, bjt{d}_jac) = bjt_with_parasitics(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, state.device_{d}_bf, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF, DEVICE_{d}_IKR, DEVICE_{d}_ISE, DEVICE_{d}_NE, DEVICE_{d}_RB, DEVICE_{d}_RC, DEVICE_{d}_RE);\n"
                            ));
                        } else {
                            // IS, VT, BETA_R, NF from state; SIGN, USE_GP, VAF, VAR, IKF, ISE, NE stay as const
                            code.push_str(&format!(
                                "        let i_dev{s} = bjt_ic(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF, DEVICE_{d}_IKR);\n"
                            ));
                            code.push_str(&format!(
                                "        let i_dev{s1} = bjt_ib(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, state.device_{d}_bf, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_ISE, DEVICE_{d}_NE);\n"
                            ));
                            code.push_str(&format!(
                                "        let bjt{d}_jac = bjt_jacobian(v_d{s}, v_d{s1}, state.device_{d}_is, state.device_{d}_vt, DEVICE_{d}_NF, state.device_{d}_bf, state.device_{d}_br, DEVICE_{d}_SIGN, DEVICE_{d}_USE_GP, DEVICE_{d}_VAF, DEVICE_{d}_VAR, DEVICE_{d}_IKF, DEVICE_{d}_IKR, DEVICE_{d}_ISE, DEVICE_{d}_NE);\n"
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
                        let jp = match &slot.params { DeviceParams::Jfet(jp) => jp, _ => unreachable!() };
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
                                "        let jfet{d}_jac = jfet_jacobian_with_rd_rs(v_d{s1}, v_d{s}, state.device_{d}_idss, state.device_{d}_vp, state.device_{d}_lambda, DEVICE_{d}_SIGN, DEVICE_{d}_RD, DEVICE_{d}_RS_PARAM);\n"
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
                            "        let jdev_{}_{} = jfet{}_jac[1];\n", s, s, d   // dId/dVds
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = jfet{}_jac[0];\n", s, s1, d  // dId/dVgs
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = jfet{}_jac[3];\n", s1, s, d  // dIg/dVds
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = jfet{}_jac[2];\n", s1, s1, d // dIg/dVgs
                        ));
                    }
                    DeviceType::Mosfet => {
                        let s = slot.start_idx;
                        let s1 = s + 1;
                        let d = dev_num;
                        let mp = match &slot.params { DeviceParams::Mosfet(mp) => mp, _ => unreachable!() };
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
                                "        let mos{d}_jac = mosfet_jacobian_with_rd_rs(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN, DEVICE_{d}_RD, DEVICE_{d}_RS_PARAM);\n"
                            ));
                        } else {
                            code.push_str(&format!(
                                "        let mos{d}_jac = mosfet_jacobian(v_d{s1}, v_d{s}, state.device_{d}_kp, state.device_{d}_vt, state.device_{d}_lambda, DEVICE_{d}_SIGN);\n"
                            ));
                        }
                        code.push_str(&format!(
                            "        let jdev_{}_{} = mos{}_jac[1];\n", s, s, d    // dId/dVds
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = mos{}_jac[0];\n", s, s1, d   // dId/dVgs
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = mos{}_jac[3];\n", s1, s, d   // dIg/dVds = 0
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = mos{}_jac[2];\n", s1, s1, d  // dIg/dVgs = 0
                        ));
                    }
                    DeviceType::Tube => {
                        let s = slot.start_idx;
                        let s1 = s + 1;
                        let d = dev_num;
                        let tp = match &slot.params { DeviceParams::Tube(tp) => tp, _ => unreachable!() };
                        if tp.has_rgi() {
                            // RGI: solve for internal Vgk, evaluate at internal voltage
                            code.push_str(&format!(
                                "        let i_dev{s} = tube_ip_with_rgi(v_d{s}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_lambda, state.device_{d}_ig_max, state.device_{d}_vgk_onset, DEVICE_{d}_RGI);\n"
                            ));
                            code.push_str(&format!(
                                "        let i_dev{s1} = tube_ig_with_rgi(v_d{s}, state.device_{d}_ig_max, state.device_{d}_vgk_onset, DEVICE_{d}_RGI);\n"
                            ));
                            code.push_str(&format!(
                                "        let tube{d}_jac = tube_jacobian_with_rgi(v_d{s}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_ig_max, state.device_{d}_vgk_onset, state.device_{d}_lambda, DEVICE_{d}_RGI);\n"
                            ));
                        } else {
                            // Standard tube (no RGI)
                            code.push_str(&format!(
                                "        let i_dev{s} = tube_ip(v_d{s}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_lambda);\n"
                            ));
                            code.push_str(&format!(
                                "        let i_dev{s1} = tube_ig(v_d{s}, state.device_{d}_ig_max, state.device_{d}_vgk_onset);\n"
                            ));
                            code.push_str(&format!(
                                "        let tube{d}_jac = tube_jacobian(v_d{s}, v_d{s1}, state.device_{d}_mu, state.device_{d}_ex, state.device_{d}_kg1, state.device_{d}_kp, state.device_{d}_kvb, state.device_{d}_ig_max, state.device_{d}_vgk_onset, state.device_{d}_lambda);\n"
                            ));
                        }
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
                    emit_nr_limit_and_converge(code, ir, 1, "        ", has_pots);
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
                    emit_nr_limit_and_converge(code, ir, 2, "        ", has_pots);
                }
                3..=16 => {
                    Self::generate_gauss_elim(code, ir, m, has_pots);
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
    fn generate_gauss_elim(code: &mut String, ir: &CircuitIR, dim: usize, has_pots: bool) {
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
        emit_nr_limit_and_converge(code, ir, dim, "            ", has_pots);
        code.push_str("        } else {\n");
        emit_nr_singular_fallback(code, dim, "            ");
        code.push_str("        }\n");
    }
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
    fn emit_nodal(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = String::new();

        // Shared: header
        code.push_str(&self.emit_header(ir)?);

        // Nodal-specific: constants
        code.push_str(&self.emit_nodal_constants(ir));

        // Shared: device model constants + functions
        code.push_str(&self.emit_device_models(ir)?);

        // Nodal-specific: state struct, Default, set_sample_rate, reset
        code.push_str(&self.emit_nodal_state(ir));

        // Nodal-specific: LU solve function
        code.push_str(&Self::emit_nodal_lu_solve(ir));

        // Nodal-specific: process_sample with full NR loop
        code.push_str(&Self::emit_nodal_process_sample(ir));

        if ir.solver_config.oversampling_factor > 1 {
            code.push_str(&Self::emit_oversampler(ir));
        }

        Ok(code)
    }

    /// Emit constants section for nodal solver.
    ///
    /// Includes A, A_neg, A_be, A_neg_be, N_v, N_i, G, C, RHS_CONST, RHS_CONST_BE,
    /// DC_OP, DC_NL_I, and topology dimensions. No S, K, or S_NI.
    fn emit_nodal_constants(&self, ir: &CircuitIR) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 { ir.topology.n_nodes } else { n };
        let n_aug = ir.topology.n_aug;
        let num_outputs = ir.solver_config.output_nodes.len();

        let mut code = section_banner("CONSTANTS: Compile-time circuit topology (Nodal solver)");

        // Dimension constants
        code.push_str(&format!("/// Number of augmented system nodes (including VS/VCVS/inductor branch variables)\n"));
        code.push_str(&format!("pub const N: usize = {};\n\n", n));
        code.push_str(&format!("/// Number of original circuit nodes (excluding ground)\n"));
        code.push_str(&format!("pub const N_NODES: usize = {};\n\n", n_nodes));
        code.push_str(&format!("/// Boundary between VS/VCVS rows and inductor branch variables\n"));
        code.push_str(&format!("pub const N_AUG: usize = {};\n\n", n_aug));
        code.push_str(&format!("/// Total nonlinear dimension (sum of device dimensions)\n"));
        code.push_str(&format!("pub const M: usize = {};\n\n", m));
        code.push_str(&format!("/// Maximum NR iterations per sample\n"));
        code.push_str(&format!("pub const MAX_ITER: usize = {};\n\n", ir.solver_config.max_iterations));
        code.push_str(&format!("/// NR convergence tolerance (VNTOL)\n"));
        code.push_str(&format!("pub const TOL: f64 = {};\n\n", fmt_f64(ir.solver_config.tolerance)));

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
            let internal_rate = ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
            code.push_str(&format!(
                "/// Internal sample rate = SAMPLE_RATE * OVERSAMPLING_FACTOR\npub const INTERNAL_SAMPLE_RATE: f64 = {:.1};\n",
                internal_rate
            ));
        }
        code.push('\n');

        // I/O configuration
        code.push_str(&format!("/// Input node index\npub const INPUT_NODE: usize = {};\n\n", ir.solver_config.input_node));
        code.push_str(&format!("/// Number of output channels\npub const NUM_OUTPUTS: usize = {};\n\n", num_outputs));
        let output_nodes_values = ir.solver_config.output_nodes.iter()
            .map(|n| n.to_string()).collect::<Vec<_>>().join(", ");
        code.push_str(&format!(
            "/// Output node indices (one per output channel)\npub const OUTPUT_NODES: [usize; NUM_OUTPUTS] = [{}];\n\n",
            output_nodes_values
        ));
        let output_scales_values = ir.solver_config.output_scales.iter()
            .map(|s| fmt_f64(*s)).collect::<Vec<_>>().join(", ");
        code.push_str(&format!(
            "/// Output scale factors (applied after DC blocking)\npub const OUTPUT_SCALES: [f64; NUM_OUTPUTS] = [{}];\n\n",
            output_scales_values
        ));
        code.push_str(&format!(
            "/// Input resistance (Thevenin equivalent)\npub const INPUT_RESISTANCE: f64 = {};\n\n",
            fmt_f64(ir.solver_config.input_resistance)
        ));

        // G and C matrices (sample-rate independent)
        code.push_str("/// G matrix: conductance matrix (sample-rate independent)\nconst G: [[f64; N]; N] = [\n");
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

        // RHS_CONST (trapezoidal)
        if ir.has_dc_sources {
            let rhs_const_values = (0..n).map(|i| fmt_f64(ir.matrices.rhs_const[i])).collect::<Vec<_>>().join(", ");
            code.push_str(&format!(
                "/// RHS constant contribution from DC sources (trapezoidal: node rows x2, VS rows x1)\npub const RHS_CONST: [f64; N] = [{}];\n\n",
                rhs_const_values
            ));
        }

        // RHS_CONST_BE (backward Euler)
        if ir.has_dc_sources && !ir.matrices.rhs_const_be.is_empty() {
            let rhs_const_be_values = (0..n).map(|i| {
                if i < ir.matrices.rhs_const_be.len() { fmt_f64(ir.matrices.rhs_const_be[i]) } else { fmt_f64(0.0) }
            }).collect::<Vec<_>>().join(", ");
            code.push_str(&format!(
                "/// RHS constant contribution from DC sources (backward Euler: all rows x1)\npub const RHS_CONST_BE: [f64; N] = [{}];\n\n",
                rhs_const_be_values
            ));
        }

        // DC blocking coefficient
        if ir.dc_block {
            let internal_rate = ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
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
                idx, pot.node_p, idx, pot.node_q,
                idx, pot.g_nominal,
                idx, pot.min_resistance, idx, pot.max_resistance
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
                    idx, ci, sw.num_positions, values.join(", "),
                    idx, ci, comp.component_type,
                    idx, ci, comp.node_p,
                    idx, ci, comp.node_q,
                    idx, ci, comp.nominal_value,
                ));
            }
            code.push('\n');
        }

        code
    }

    /// Emit state struct, Default impl, set_sample_rate, and reset for nodal solver.
    fn emit_nodal_state(&self, ir: &CircuitIR) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 { ir.topology.n_nodes } else { n };
        let n_aug = ir.topology.n_aug;
        let num_outputs = ir.solver_config.output_nodes.len();
        let has_pots = !ir.pots.is_empty();
        let has_switches = !ir.switches.is_empty();

        let mut code = section_banner("STATE STRUCTURE (Nodal solver)");

        // DC OP constant
        let has_dc_op = ir.has_dc_op;
        if has_dc_op {
            let dc_op_values = ir.dc_operating_point.iter().map(|v| fmt_f64(*v)).collect::<Vec<_>>().join(", ");
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
            let dc_nl_i_values = ir.dc_nl_currents.iter().map(|v| fmt_f64(*v)).collect::<Vec<_>>().join(", ");
            code.push_str(&format!(
                "/// DC operating point: nonlinear device currents at bias point\npub const DC_NL_I: [f64; M] = [{}];\n\n",
                dc_nl_i_values
            ));
        }

        // State struct
        code.push_str("/// Circuit state for one processing channel (nodal solver).\n");
        code.push_str("///\n");
        code.push_str("/// Contains per-sample state and sample-rate-dependent matrices.\n");
        code.push_str("/// Call [`set_sample_rate`](CircuitState::set_sample_rate) before processing\n");
        code.push_str("/// if the host sample rate differs from [`SAMPLE_RATE`].\n");
        code.push_str("#[derive(Clone, Debug)]\n");
        code.push_str("pub struct CircuitState {\n");
        code.push_str("    /// Previous node voltages v[n-1]\n");
        code.push_str("    pub v_prev: [f64; N],\n\n");
        code.push_str("    /// Previous nonlinear currents i_nl[n-1]\n");
        code.push_str("    pub i_nl_prev: [f64; M],\n\n");
        code.push_str("    /// Nonlinear currents from two samples ago i_nl[n-2] (for NR predictor)\n");
        code.push_str("    pub i_nl_prev_prev: [f64; M],\n\n");
        code.push_str("    /// DC operating point (for reset/sleep/wake)\n");
        code.push_str("    pub dc_operating_point: [f64; N],\n\n");
        code.push_str("    /// Previous input sample for trapezoidal integration\n");
        code.push_str("    pub input_prev: f64,\n\n");
        code.push_str("    /// Iteration count from last solve (for diagnostics)\n");
        code.push_str("    pub last_nr_iterations: u32,\n\n");
        if ir.dc_block {
            code.push_str("    /// DC blocking filter: previous input samples (one per output)\n");
            code.push_str("    pub dc_block_x_prev: [f64; NUM_OUTPUTS],\n");
            code.push_str("    /// DC blocking filter: previous output samples (one per output)\n");
            code.push_str("    pub dc_block_y_prev: [f64; NUM_OUTPUTS],\n");
            code.push_str("    /// DC blocking filter coefficient (recomputed on sample rate change)\n");
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
        code.push_str("    pub diag_nan_reset_count: u64,\n\n");
        code.push_str("    /// A matrix: G + alpha*C (trapezoidal), recomputed by set_sample_rate\n");
        code.push_str("    pub a: [[f64; N]; N],\n");
        code.push_str("    /// A_neg matrix: alpha*C - G (trapezoidal history), recomputed by set_sample_rate\n");
        code.push_str("    pub a_neg: [[f64; N]; N],\n");
        code.push_str("    /// A_be matrix: G + (1/T)*C (backward Euler), recomputed by set_sample_rate\n");
        code.push_str("    pub a_be: [[f64; N]; N],\n");
        code.push_str("    /// A_neg_be matrix: (1/T)*C (backward Euler history), recomputed by set_sample_rate\n");
        code.push_str("    pub a_neg_be: [[f64; N]; N],\n");

        // Mutable G and C for pot/switch re-stamping
        if has_pots || has_switches {
            code.push_str("    /// Working G matrix (modified by pots/switches)\n");
            code.push_str("    pub g_work: [[f64; N]; N],\n");
            code.push_str("    /// Working C matrix (modified by switches)\n");
            code.push_str("    pub c_work: [[f64; N]; N],\n");
            code.push_str("    /// Current sample rate (for rebuild_matrices)\n");
            code.push_str("    pub current_sample_rate: f64,\n\n");
        }

        // Pot state fields
        for (idx, _pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "    /// Potentiometer {}: current resistance (ohms)\n\
                 \x20   pub pot_{}_resistance: f64,\n",
                idx, idx
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
        code.push_str("        Self {\n");
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
            let dc_x: Vec<String> = output_nodes.iter().map(|&node| {
                if has_dc_op && node < ir.dc_operating_point.len() {
                    fmt_f64(ir.dc_operating_point[node])
                } else {
                    "0.0".to_string()
                }
            }).collect();
            code.push_str(&format!("            dc_block_x_prev: [{}],\n", dc_x.join(", ")));
            code.push_str(&format!("            dc_block_y_prev: [{}],\n", vec!["0.0"; output_nodes.len()].join(", ")));
            code.push_str("            dc_block_r: DC_BLOCK_R,\n");
        }
        code.push_str("            diag_peak_output: 0.0,\n");
        code.push_str("            diag_clamp_count: 0,\n");
        code.push_str("            diag_nr_max_iter_count: 0,\n");
        code.push_str("            diag_be_fallback_count: 0,\n");
        code.push_str("            diag_nan_reset_count: 0,\n");
        code.push_str("            a: A_DEFAULT,\n");
        code.push_str("            a_neg: A_NEG_DEFAULT,\n");
        code.push_str("            a_be: A_BE_DEFAULT,\n");
        code.push_str("            a_neg_be: A_NEG_BE_DEFAULT,\n");

        if has_pots || has_switches {
            code.push_str("            g_work: G,\n");
            code.push_str("            c_work: C,\n");
            code.push_str(&format!("            current_sample_rate: {:.17e},\n",
                ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64));
        }

        for (idx, pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "            pot_{}_resistance: {:.17e},\n",
                idx, 1.0 / pot.g_nominal
            ));
        }

        for (idx, _sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!(
                "            switch_{}_position: 0,\n",
                idx
            ));
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

        code.push_str("        }\n");
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
        // Re-init DC blocker from DC OP (prevents transient on reset)
        if ir.dc_block {
            let output_nodes = &ir.solver_config.output_nodes;
            for (oi, &node) in output_nodes.iter().enumerate() {
                if has_dc_op && node < ir.dc_operating_point.len() {
                    code.push_str(&format!(
                        "        self.dc_block_x_prev[{}] = self.dc_operating_point[{}];\n", oi, node
                    ));
                } else {
                    code.push_str(&format!(
                        "        self.dc_block_x_prev[{}] = 0.0;\n", oi
                    ));
                }
            }
            code.push_str("        self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n");
        }
        code.push_str("        self.diag_peak_output = 0.0;\n");
        code.push_str("        self.diag_clamp_count = 0;\n");
        code.push_str("        self.diag_nr_max_iter_count = 0;\n");
        code.push_str("        self.diag_be_fallback_count = 0;\n");
        code.push_str("        self.diag_nan_reset_count = 0;\n");
        if has_pots || has_switches {
            code.push_str("        self.g_work = G;\n");
            code.push_str("        self.c_work = C;\n");
        }
        for (idx, pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "        self.pot_{}_resistance = {:.17e};\n",
                idx, 1.0 / pot.g_nominal
            ));
        }
        for (idx, _sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!(
                "        self.switch_{}_position = 0;\n", idx
            ));
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
        code.push_str("    }\n\n");

        // set_dc_operating_point()
        code.push_str("    /// Set DC operating point (call after DC analysis)\n");
        code.push_str("    pub fn set_dc_operating_point(&mut self, v_dc: [f64; N]) {\n");
        code.push_str("        self.dc_operating_point = v_dc;\n");
        code.push_str("        self.v_prev = v_dc;\n");
        code.push_str("    }\n\n");

        // set_sample_rate()
        code.push_str("    /// Recompute all sample-rate-dependent matrices for a new sample rate.\n");
        code.push_str("    ///\n");
        code.push_str("    /// Call this once during plugin initialization (NOT on the audio thread).\n");
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
        code.push_str("        let alpha = 2.0 * internal_rate;\n");
        code.push_str("        let alpha_be = internal_rate;\n\n");

        if has_pots || has_switches {
            code.push_str("        self.current_sample_rate = internal_rate;\n");
        }
        code.push_str("        self.rebuild_matrices(internal_rate);\n\n");

        // DC block recomputation
        if ir.dc_block {
            code.push_str(&format!(
                "        // Recompute DC blocking coefficient\n\
                 \x20       self.dc_block_r = 1.0 - 2.0 * std::f64::consts::PI * 5.0 / internal_rate;\n\
                 \x20       self.dc_block_x_prev = [0.0; NUM_OUTPUTS];\n\
                 \x20       self.dc_block_y_prev = [0.0; NUM_OUTPUTS];\n"
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

        code.push_str("    }\n\n");

        // rebuild_matrices: recompute A/A_neg/A_be/A_neg_be from G+C
        let g_src = if has_pots || has_switches { "self.g_work" } else { "G" };
        let c_src = if has_pots || has_switches { "self.c_work" } else { "C" };

        code.push_str("    /// Recompute A, A_neg, A_be, A_neg_be from G and C matrices.\n");
        code.push_str("    ///\n");
        code.push_str("    /// Called by set_sample_rate, set_pot, and set_switch.\n");
        code.push_str("    pub fn rebuild_matrices(&mut self, internal_rate: f64) {\n");
        code.push_str("        let alpha = 2.0 * internal_rate;\n");
        code.push_str("        let alpha_be = internal_rate;\n\n");

        code.push_str(&format!(
            "        for i in 0..N {{\n\
             \x20           for j in 0..N {{\n\
             \x20               self.a[i][j] = {}[i][j] + alpha * {}[i][j];\n\
             \x20               self.a_neg[i][j] = alpha * {}[i][j] - {}[i][j];\n\
             \x20               self.a_be[i][j] = {}[i][j] + alpha_be * {}[i][j];\n\
             \x20               self.a_neg_be[i][j] = alpha_be * {}[i][j];\n\
             \x20           }}\n\
             \x20       }}\n",
            g_src, c_src, c_src, g_src, g_src, c_src, c_src
        ));

        // Zero VS/VCVS algebraic rows in A_neg and A_neg_be (NOT inductor rows)
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
                 \x20   /// O(1) cost: stamps delta conductance directly into A matrices.\n\
                 \x20   /// Safe to call per-sample.\n",
                idx, pot.min_resistance, pot.max_resistance
            ));
            code.push_str(&format!(
                "    pub fn set_pot_{}(&mut self, resistance: f64) {{\n", idx
            ));
            code.push_str(&format!(
                "        let r = resistance.clamp(POT_{}_MIN_R, POT_{}_MAX_R);\n\
                 \x20       if (r - self.pot_{}_resistance).abs() < 1e-12 {{ return; }}\n\n\
                 \x20       // Delta conductance: stamp into A, A_neg, A_be (NOT A_neg_be: no G term)\n\
                 \x20       let delta_g = 1.0 / r - 1.0 / self.pot_{}_resistance;\n",
                idx, idx, idx, idx
            ));

            // Helper: emit delta stamp for one matrix with sign multiplier
            // A and A_be get +delta_g, A_neg gets -delta_g
            let emit_delta_stamp = |code: &mut String, matrix: &str, sign: &str| {
                if np > 0 {
                    code.push_str(&format!(
                        "        self.{matrix}[{}][{}] {sign}= delta_g;\n", np - 1, np - 1
                    ));
                }
                if nq > 0 {
                    code.push_str(&format!(
                        "        self.{matrix}[{}][{}] {sign}= delta_g;\n", nq - 1, nq - 1
                    ));
                }
                if np > 0 && nq > 0 {
                    let neg_sign = if sign == "+" { "-" } else { "+" };
                    code.push_str(&format!(
                        "        self.{matrix}[{}][{}] {neg_sign}= delta_g;\n\
                         \x20       self.{matrix}[{}][{}] {neg_sign}= delta_g;\n",
                        np - 1, nq - 1, nq - 1, np - 1
                    ));
                }
            };

            emit_delta_stamp(&mut code, "a", "+");
            emit_delta_stamp(&mut code, "a_neg", "-");
            emit_delta_stamp(&mut code, "a_be", "+");
            // A_neg_be = alpha_be * C — no G term, unchanged by pot

            // Also update g_work for consistency (needed by rebuild_matrices on sample rate change)
            if has_pots || has_switches {
                code.push_str("\n        // Update working G for sample rate rebuild consistency\n");
                if np > 0 {
                    code.push_str(&format!(
                        "        self.g_work[{}][{}] += delta_g;\n", np - 1, np - 1
                    ));
                }
                if nq > 0 {
                    code.push_str(&format!(
                        "        self.g_work[{}][{}] += delta_g;\n", nq - 1, nq - 1
                    ));
                }
                if np > 0 && nq > 0 {
                    code.push_str(&format!(
                        "        self.g_work[{}][{}] -= delta_g;\n\
                         \x20       self.g_work[{}][{}] -= delta_g;\n",
                        np - 1, nq - 1, nq - 1, np - 1
                    ));
                }
            }

            code.push_str(&format!(
                "\n        self.pot_{}_resistance = r;\n", idx
            ));
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
                let matrix = if comp.component_type == 'R' { "g_work" } else { "c_work" };

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

                // Stamp delta
                if np > 0 {
                    code.push_str(&format!(
                        "        self.{matrix}[{}][{}] += delta_{ci};\n", np - 1, np - 1
                    ));
                }
                if nq > 0 {
                    code.push_str(&format!(
                        "        self.{matrix}[{}][{}] += delta_{ci};\n", nq - 1, nq - 1
                    ));
                }
                if np > 0 && nq > 0 {
                    code.push_str(&format!(
                        "        self.{matrix}[{}][{}] -= delta_{ci};\n\
                         \x20       self.{matrix}[{}][{}] -= delta_{ci};\n",
                        np - 1, nq - 1, nq - 1, np - 1
                    ));
                }
                code.push('\n');
            }

            code.push_str(&format!(
                "        self.switch_{}_position = position;\n", idx
            ));
            code.push_str("        self.rebuild_matrices(self.current_sample_rate);\n");
            code.push_str("    }\n\n");
        }

        code.push_str("}\n\n");

        let _ = (n, m, n_nodes, n_aug, num_outputs);

        code
    }

    /// Emit LU solve function for the nodal solver (N x N with partial pivoting).
    fn emit_nodal_lu_solve(_ir: &CircuitIR) -> String {
        let mut code = section_banner("LU SOLVE (Equilibrated Gaussian elimination with iterative refinement)");

        code.push_str("/// Solve A*x = b using equilibrated LU with partial pivoting + iterative refinement.\n");
        code.push_str("///\n");
        code.push_str("/// Diagonal equilibration scales rows/cols by 1/sqrt(|A[i][i]|) to reduce\n");
        code.push_str("/// condition number. One round of iterative refinement corrects residual error.\n");
        code.push_str("/// Matches the runtime solver's `solve_equilibrated()` for numerical parity.\n");
        code.push_str("/// Modifies `a` in place (LU factors). On success, `b` contains the solution.\n");
        code.push_str("#[inline(always)]\n");
        code.push_str("fn lu_solve(a: &mut [[f64; N]; N], b: &mut [f64; N]) -> bool {\n");
        code.push_str("    // Save original A and b for iterative refinement\n");
        code.push_str("    let a_orig = *a;\n");
        code.push_str("    let b_orig = *b;\n\n");

        // Step 1: Equilibrate
        code.push_str("    // Step 1: Diagonal equilibration — d[i] = 1/sqrt(|A[i][i]|)\n");
        code.push_str("    let mut d = [1.0f64; N];\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        let diag = a[i][i].abs();\n");
        code.push_str("        if diag > 1e-30 { d[i] = 1.0 / diag.sqrt(); }\n");
        code.push_str("    }\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        for j in 0..N {\n");
        code.push_str("            a[i][j] *= d[i] * d[j];\n");
        code.push_str("        }\n");
        code.push_str("        b[i] *= d[i];\n");
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

        // Step 3: Forward/backward substitution — solve LU * x_eq = D * P * b
        code.push_str("    // Step 3: Solve LU * x_eq = D * P * b\n");
        code.push_str("    let mut x = [0.0f64; N];\n");
        code.push_str("    for i in 0..N { x[i] = d[perm[i]] * b_orig[perm[i]]; }\n\n");

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
        code.push_str("            ax_i += d[pi] * a_orig[pi][j] * d[j] * x[j];\n");
        code.push_str("        }\n");
        code.push_str("        r[i] = d[pi] * b_orig[pi] - ax_i;\n");
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

        // Step 5: Apply correction and undo equilibration
        code.push_str("    // Step 5: Apply correction and undo equilibration\n");
        code.push_str("    for i in 0..N {\n");
        code.push_str("        b[i] = d[i] * (x[i] + r[i]);\n");
        code.push_str("    }\n\n");

        code.push_str("    true\n");
        code.push_str("}\n\n");

        code
    }

    /// Emit the complete process_sample function for the nodal solver.
    ///
    /// This is the core NR loop: build RHS, iterate (Jacobian stamp, LU solve,
    /// SPICE limiting, damping, convergence), BE fallback, state update.
    fn emit_nodal_process_sample(ir: &CircuitIR) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 { ir.topology.n_nodes } else { n };
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
            code.push_str("/// Uses full-nodal Newton-Raphson with LU factorization per iteration.\n");
            code.push_str("/// Includes backward Euler fallback for unconditional stability.\n");
            code.push_str("#[inline]\n");
            code.push_str("pub fn process_sample(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n");
        }

        // Input sanitization
        code.push_str("    let input = if input.is_finite() { input.clamp(-100.0, 100.0) } else { 0.0 };\n\n");

        // Step 1: Build RHS = rhs_const + A_neg * v_prev + N_i * i_nl_prev + input (sparse)
        code.push_str("    // Step 1: Build RHS (sparse A_neg * v_prev + sparse N_i * i_nl_prev)\n");
        if ir.has_dc_sources {
            code.push_str("    let mut rhs = RHS_CONST;\n");
        } else {
            code.push_str("    let mut rhs = [0.0f64; N];\n");
        }
        // Sparse A_neg * v_prev
        for i in 0..n {
            let nz_cols = &ir.sparsity.a_neg.nz_by_row[i];
            if nz_cols.is_empty() { continue; }
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
        code.push('\n');

        // Input source (Thevenin, trapezoidal)
        code.push_str("    // Input source (Thevenin, trapezoidal)\n");
        code.push_str("    let input_conductance = 1.0 / INPUT_RESISTANCE;\n");
        code.push_str("    rhs[INPUT_NODE] += (input + state.input_prev) * input_conductance;\n");
        code.push_str("    state.input_prev = input;\n\n");

        // Handle linear circuits (M=0): direct LU solve, no NR iteration
        if m == 0 {
            code.push_str("    // Linear circuit: direct LU solve (no NR needed)\n");
            code.push_str("    let mut g_aug = state.a;\n");
            code.push_str("    let mut v = rhs;\n");
            code.push_str("    if !lu_solve(&mut g_aug, &mut v) {\n");
            code.push_str("        v = state.v_prev;\n");
            code.push_str("    }\n\n");
        } else {
            // Step 2: Newton-Raphson in full augmented voltage space
            code.push_str("    // Step 2: Newton-Raphson in full augmented voltage space\n");
            code.push_str("    let mut v = state.v_prev;\n");
            code.push_str("    let mut converged = false;\n");
            code.push_str("    let mut i_nl = [0.0f64; M];\n\n");

            // Trapezoidal NR loop
            code.push_str("    for iter in 0..MAX_ITER {\n");

            // 2a. Extract nonlinear voltages: v_nl = N_v * v (sparse)
            code.push_str("        // 2a. Extract nonlinear voltages: v_nl = N_v * v (sparse)\n");
            code.push_str("        let mut v_nl = [0.0f64; M];\n");
            for i in 0..m {
                let nz_cols = &ir.sparsity.n_v.nz_by_row[i];
                if nz_cols.is_empty() { continue; }
                let terms: Vec<String> = nz_cols.iter().map(|&j| {
                    format!("N_V[{}][{}] * v[{}]", i, j, j)
                }).collect();
                code.push_str(&format!("        v_nl[{}] = {};\n", i, terms.join(" + ")));
            }
            code.push('\n');

            // 2b. Evaluate device currents and Jacobian
            code.push_str("        // 2b. Evaluate device currents and Jacobian (block-diagonal)\n");
            // i_nl is declared in outer scope (line above NR loop), j_dev is per-iteration
            code.push_str("        let mut j_dev = [0.0f64; M * M];\n");
            Self::emit_nodal_device_evaluation_body(&mut code, ir, "        ");
            code.push('\n');

            // 2c. Build Jacobian: G_aug = A - N_i * J_dev * N_v (sparse)
            // J_dev is block-diagonal; N_i and N_v have ~2 nonzero entries per device dim.
            // Compile-time unrolled: ~64 stamps for 4 tubes vs 107K dense iterations.
            code.push_str("        // 2c. Build Jacobian: G_aug = A - N_i * J_dev * N_v (sparse)\n");
            code.push_str("        let mut g_aug = state.a;\n");
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
                            if ni_nodes.is_empty() || nv_nodes.is_empty() { continue; }
                            for &a in ni_nodes {
                                for &b in nv_nodes {
                                    code.push_str(&format!(
                                        "        g_aug[{}][{}] -= N_I[{}][{}] * j_dev[{} * M + {}] * N_V[{}][{}];\n",
                                        a, b, a, i, i, j, j, b
                                    ));
                                }
                            }
                        }
                    }
                }
            }
            code.push('\n');

            // 2d. Build companion RHS: rhs_base + N_i * (i_nl - J_dev * v_nl) (sparse)
            code.push_str("        // 2d. Build companion RHS: rhs + N_i * (i_nl - J_dev * v_nl) (sparse)\n");
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
                        // Compute jdev_vnl = sum_j j_dev[i*M+j] * v_nl[j] (only within block)
                        let jdv_terms: Vec<String> = (0..dim).map(|dj| {
                            let j = s + dj;
                            format!("j_dev[{} * M + {}] * v_nl[{}]", i, j, j)
                        }).collect();
                        code.push_str(&format!(
                            "        {{ let i_comp = i_nl[{}] - ({});\n",
                            i, jdv_terms.join(" + ")
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

            // 2e. LU solve: v_new = G_aug^{-1} * rhs_work
            code.push_str("        // 2e. LU solve: v_new = G_aug^{-1} * rhs_work\n");
            code.push_str("        let mut v_new = rhs_work;\n");
            code.push_str("        if !lu_solve(&mut g_aug, &mut v_new) {\n");
            code.push_str("            state.diag_nr_max_iter_count += 1;\n");
            code.push_str("            break;\n");
            code.push_str("        }\n\n");

            // 2f. SPICE-style voltage limiting + global node damping
            code.push_str("        // 2f. SPICE voltage limiting + node damping\n");
            code.push_str("        let mut alpha = 1.0_f64;\n\n");

            // Layer 1: Device voltage limiting
            code.push_str("        // Layer 1: SPICE device voltage limiting\n");
            Self::emit_nodal_voltage_limiting(&mut code, ir);
            code.push('\n');

            // Layer 2: Global node voltage damping (10V threshold)
            code.push_str("        // Layer 2: Global node voltage damping (10V threshold)\n");
            code.push_str("        {\n");
            code.push_str("            let mut max_node_dv = 0.0_f64;\n");
            code.push_str(&format!(
                "            for i in 0..{} {{\n\
                 \x20               let dv = alpha * (v_new[i] - v[i]);\n\
                 \x20               max_node_dv = max_node_dv.max(dv.abs());\n\
                 \x20           }}\n",
                n_nodes
            ));
            code.push_str("            if max_node_dv > 10.0 {\n");
            code.push_str("                alpha *= (10.0 / max_node_dv).max(0.01);\n");
            code.push_str("            }\n");
            code.push_str("        }\n\n");

            // Apply damped Newton step and check convergence
            // Compute step BEFORE updating v, so convergence check sees the actual delta
            // Skip convergence check on iter 0 — need at least one full NR update
            code.push_str("        // Compute damped step, check convergence, then apply\n");
            code.push_str("        let mut max_step_exceeded = false;\n");
            code.push_str("        for i in 0..N {\n");
            code.push_str("            let step = alpha * (v_new[i] - v[i]);\n");
            code.push_str("            let threshold = 1e-6 * v[i].abs().max((v[i] + step).abs()) + TOL;\n");
            code.push_str("            if step.abs() >= threshold { max_step_exceeded = true; }\n");
            code.push_str("            v[i] += step;\n");
            code.push_str("        }\n");
            code.push_str("        let converged_check = !max_step_exceeded;\n\n");

            code.push_str("        if converged_check {\n");
            code.push_str("            converged = true;\n");
            code.push_str("            state.last_nr_iterations = iter as u32;\n");

            // Final device evaluation at converged point
            code.push_str("            // Final device evaluation at converged point\n");
            code.push_str("            let mut v_nl_final = [0.0f64; M];\n");
            code.push_str("            for i in 0..M {\n");
            code.push_str("                let mut sum = 0.0;\n");
            code.push_str("                for j in 0..N { sum += N_V[i][j] * v[j]; }\n");
            code.push_str("                v_nl_final[i] = sum;\n");
            code.push_str("            }\n");
            Self::emit_nodal_device_evaluation_final(&mut code, ir, "            ");
            code.push_str("            break;\n");
            code.push_str("        }\n");
            code.push_str("    }\n\n"); // end trapezoidal NR loop

            // Backward Euler fallback
            code.push_str("    // Backward Euler fallback: if trapezoidal NR didn't converge, retry with BE\n");
            code.push_str("    if !converged {\n");
            code.push_str("        state.diag_nr_max_iter_count += 1;\n\n");

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

            // Extract v_nl
            code.push_str("            let mut v_nl = [0.0f64; M];\n");
            code.push_str("            for i in 0..M {\n");
            code.push_str("                let mut sum = 0.0;\n");
            code.push_str("                for j in 0..N { sum += N_V[i][j] * v[j]; }\n");
            code.push_str("                v_nl[i] = sum;\n");
            code.push_str("            }\n\n");

            // Evaluate devices (write to outer i_nl, declare local j_dev)
            code.push_str("            // Evaluate devices\n");
            code.push_str("            let mut j_dev = [0.0f64; M * M];\n");
            Self::emit_nodal_device_evaluation_body(&mut code, ir, "            ");
            code.push('\n');

            // Build Jacobian for BE (sparse, same structure as trapezoidal)
            code.push_str("            let mut g_aug = state.a_be;\n");
            {
                let mut ni_nz_by_dev = vec![Vec::new(); m];
                for (a, cols) in ir.sparsity.n_i.nz_by_row.iter().enumerate() {
                    for &i in cols { ni_nz_by_dev[i].push(a); }
                }
                for slot in &ir.device_slots {
                    let s = slot.start_idx;
                    let dim = slot.dimension;
                    for di in 0..dim {
                        let i = s + di;
                        for dj in 0..dim {
                            let j = s + dj;
                            let nv_nodes = &ir.sparsity.n_v.nz_by_row[j];
                            for &a in &ni_nz_by_dev[i] {
                                for &b in nv_nodes {
                                    code.push_str(&format!(
                                        "            g_aug[{}][{}] -= N_I[{}][{}] * j_dev[{} * M + {}] * N_V[{}][{}];\n",
                                        a, b, a, i, i, j, j, b
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
                    for &i in cols { ni_nz_by_dev[i].push(a); }
                }
                for slot in &ir.device_slots {
                    let s = slot.start_idx;
                    let dim = slot.dimension;
                    for di in 0..dim {
                        let i = s + di;
                        let jdv_terms: Vec<String> = (0..dim).map(|dj| {
                            let j = s + dj;
                            format!("j_dev[{} * M + {}] * v_nl[{}]", i, j, j)
                        }).collect();
                        code.push_str(&format!(
                            "            {{ let i_comp = i_nl[{}] - ({});\n", i, jdv_terms.join(" + ")
                        ));
                        for &a in &ni_nz_by_dev[i] {
                            code.push_str(&format!(
                                "              rhs_work[{}] += N_I[{}][{}] * i_comp;\n", a, a, i
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

            // Apply damped step and check convergence (compute delta before updating)
            code.push_str("            let mut be_step_exceeded = false;\n");
            code.push_str("            for i in 0..N {\n");
            code.push_str("                let step = alpha * (v_new[i] - v[i]);\n");
            code.push_str("                let threshold = 1e-6 * v[i].abs().max((v[i] + step).abs()) + TOL;\n");
            code.push_str("                if step.abs() >= threshold { be_step_exceeded = true; }\n");
            code.push_str("                v[i] += step;\n");
            code.push_str("            }\n");
            code.push_str("            let be_converged = !be_step_exceeded;\n\n");

            code.push_str("            if be_converged {\n");
            code.push_str("                converged = true;\n");
            code.push_str("                state.diag_be_fallback_count += 1;\n");
            code.push_str("                let mut v_nl_final = [0.0f64; M];\n");
            code.push_str("                for i in 0..M {\n");
            code.push_str("                    let mut sum = 0.0;\n");
            code.push_str("                    for j in 0..N { sum += N_V[i][j] * v[j]; }\n");
            code.push_str("                    v_nl_final[i] = sum;\n");
            code.push_str("                }\n");
            Self::emit_nodal_device_evaluation_final(&mut code, ir, "                ");
            code.push_str("                break;\n");
            code.push_str("            }\n");
            code.push_str("        }\n\n"); // end BE NR loop

            // If still not converged, ensure i_nl is consistent
            code.push_str("        // If still not converged, ensure i_nl is consistent with v\n");
            code.push_str("        if !converged {\n");
            code.push_str("            let mut v_nl_final = [0.0f64; M];\n");
            code.push_str("            for i in 0..M {\n");
            code.push_str("                let mut sum = 0.0;\n");
            code.push_str("                for j in 0..N { sum += N_V[i][j] * v[j]; }\n");
            code.push_str("                v_nl_final[i] = sum;\n");
            code.push_str("            }\n");
            Self::emit_nodal_device_evaluation_final(&mut code, ir, "            ");
            code.push_str("        }\n");
            code.push_str("    }\n\n"); // end BE fallback block
        }

        // Step 3: Update state
        code.push_str("    // Step 3: Update state\n");
        code.push_str("    state.v_prev = v;\n");
        if m > 0 {
            code.push_str("    state.i_nl_prev_prev = state.i_nl_prev;\n");
            code.push_str("    state.i_nl_prev = i_nl;\n");
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

        // NaN check
        code.push_str("    // Sanitize state: if NaN/inf entered, reset to DC operating point\n");
        code.push_str("    if !state.v_prev.iter().all(|x| x.is_finite()) {\n");
        code.push_str("        state.v_prev = state.dc_operating_point;\n");
        code.push_str("        state.i_nl_prev = [0.0; M];\n");
        code.push_str("        state.i_nl_prev_prev = [0.0; M];\n");
        code.push_str("        state.input_prev = 0.0;\n");
        // Reset thermal state on NaN
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
        code.push_str("        state.diag_nan_reset_count += 1;\n");
        code.push_str("        return [0.0; NUM_OUTPUTS];\n");
        code.push_str("    }\n\n");

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
        code.push_str("        if abs_out > state.diag_peak_output { state.diag_peak_output = abs_out; }\n");
        if ir.dc_block {
            code.push_str("        if abs_out > 10.0 { state.diag_clamp_count += 1; }\n");
            code.push_str("        output[out_idx] = scaled.clamp(-10.0, 10.0);\n");
        } else {
            code.push_str("        output[out_idx] = if scaled.is_finite() { scaled } else { 0.0 };\n");
        }
        code.push_str("    }\n");
        code.push_str("    output\n");
        code.push_str("}\n\n");

        let _ = (num_outputs, n_nodes);

        code
    }

    /// Emit device evaluation code for the NR loop body (trapezoidal, no indent prefix).
    fn emit_nodal_device_evaluation(code: &mut String, ir: &CircuitIR) {
        Self::emit_nodal_device_evaluation_indented(code, ir, "        ");
    }

    /// Emit device evaluation code with declarations (i_nl + j_dev).
    fn emit_nodal_device_evaluation_indented(code: &mut String, ir: &CircuitIR, indent: &str) {
        code.push_str(&format!("{indent}let mut i_nl = [0.0f64; M];\n"));
        code.push_str(&format!("{indent}let mut j_dev = [0.0f64; M * M];\n"));
        Self::emit_nodal_device_evaluation_body(code, ir, indent);
    }

    /// Emit device evaluation code WITHOUT declarations (writes to existing i_nl, j_dev).
    fn emit_nodal_device_evaluation_body(code: &mut String, ir: &CircuitIR, indent: &str) {
        let _m = ir.topology.m;

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            let s = slot.start_idx;
            match (&slot.device_type, &slot.params) {
                (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                    if dp.has_rs() {
                        // Series resistance: use helper functions
                        let bv_i = if dp.has_bv() { format!(" + diode_breakdown_current(v_nl[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV)") } else { String::new() };
                        let bv_g = if dp.has_bv() { format!(" + diode_breakdown_conductance(v_nl[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV)") } else { String::new() };
                        code.push_str(&format!(
                            "{indent}{{ // Diode {dev_num} (RS={has_rs}, BV={has_bv})\n\
                             {indent}    i_nl[{s}] = diode_current_with_rs(v_nl[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_RS){bv_i};\n\
                             {indent}    j_dev[{s} * M + {s}] = diode_conductance_with_rs(v_nl[{s}], state.device_{dev_num}_is, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_RS){bv_g};\n\
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
                             {indent}    j_dev[{s} * M + {s}] = g_base + diode_breakdown_conductance(v, state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV);\n\
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
                             {indent}    j_dev[{s} * M + {s}] = if v > 40.0 * state.device_{dev_num}_n_vt {{ g + state.device_{dev_num}_is / state.device_{dev_num}_n_vt }} else {{ g }};\n\
                             {indent}}}\n"
                        ));
                    }
                }
                (DeviceType::Bjt, DeviceParams::Bjt(bp)) => {
                    let s1 = s + 1;
                    if bp.has_parasitics() {
                        code.push_str(&format!(
                            "{indent}{{ // BJT {dev_num} (RB/RC/RE)\n\
                             {indent}    let vbe = v_nl[{s}] * DEVICE_{dev_num}_SIGN;\n\
                             {indent}    let vbc = v_nl[{s1}] * DEVICE_{dev_num}_SIGN;\n\
                             {indent}    let (ic, ib, jac) = bjt_with_parasitics(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_RB, DEVICE_{dev_num}_RC, DEVICE_{dev_num}_RE);\n\
                             {indent}    i_nl[{s}] = ic * DEVICE_{dev_num}_SIGN;\n\
                             {indent}    i_nl[{s1}] = ib * DEVICE_{dev_num}_SIGN;\n\
                             {indent}    j_dev[{s} * M + {s}] = jac[0];\n\
                             {indent}    j_dev[{s} * M + {s1}] = jac[1];\n\
                             {indent}    j_dev[{s1} * M + {s}] = jac[2];\n\
                             {indent}    j_dev[{s1} * M + {s1}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}{{ // BJT {dev_num}\n\
                             {indent}    let vbe = v_nl[{s}] * DEVICE_{dev_num}_SIGN;\n\
                             {indent}    let vbc = v_nl[{s1}] * DEVICE_{dev_num}_SIGN;\n\
                             {indent}    let ic = bjt_ic(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR);\n\
                             {indent}    let ib = bjt_ib(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE);\n\
                             {indent}    let jac = bjt_jacobian(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE);\n\
                             {indent}    i_nl[{s}] = ic * DEVICE_{dev_num}_SIGN;\n\
                             {indent}    i_nl[{s1}] = ib * DEVICE_{dev_num}_SIGN;\n\
                             {indent}    j_dev[{s} * M + {s}] = jac[0];\n\
                             {indent}    j_dev[{s} * M + {s1}] = jac[1];\n\
                             {indent}    j_dev[{s1} * M + {s}] = jac[2];\n\
                             {indent}    j_dev[{s1} * M + {s1}] = jac[3];\n\
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
                         {indent}    j_dev[{s} * M + {s}] = state.device_{dev_num}_is / (DEVICE_{dev_num}_NF * state.device_{dev_num}_vt) * exp_be;\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Jfet, DeviceParams::Jfet(jp)) => {
                    let s1 = s + 1;
                    let jac_fn = if jp.has_rd_rs() {
                        format!("jfet_jacobian_with_rd_rs(vgs, vds, state.device_{dev_num}_idss, state.device_{dev_num}_vp, state.device_{dev_num}_lambda, sign, DEVICE_{dev_num}_RD, DEVICE_{dev_num}_RS_PARAM)")
                    } else {
                        format!("jfet_jacobian(vgs, vds, state.device_{dev_num}_idss, state.device_{dev_num}_vp, state.device_{dev_num}_lambda, sign)")
                    };
                    code.push_str(&format!(
                        "{indent}{{ // JFET {dev_num}\n\
                         {indent}    let vds = v_nl[{s}];\n\
                         {indent}    let vgs = v_nl[{s1}];\n\
                         {indent}    let sign = DEVICE_{dev_num}_SIGN;\n\
                         {indent}    i_nl[{s}] = jfet_id(vgs, vds, state.device_{dev_num}_idss, state.device_{dev_num}_vp, state.device_{dev_num}_lambda, sign);\n\
                         {indent}    i_nl[{s1}] = jfet_ig(vgs, sign);\n\
                         {indent}    let jac = {jac_fn};\n\
                         {indent}    j_dev[{s} * M + {s}] = jac[0];\n\
                         {indent}    j_dev[{s} * M + {s1}] = jac[1];\n\
                         {indent}    j_dev[{s1} * M + {s}] = jac[2];\n\
                         {indent}    j_dev[{s1} * M + {s1}] = jac[3];\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Mosfet, DeviceParams::Mosfet(mp)) => {
                    let s1 = s + 1;
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
                        format!("vt_eff")
                    } else {
                        code.push_str(&format!(
                            "{indent}{{ // MOSFET {dev_num}\n"
                        ));
                        format!("state.device_{dev_num}_vt")
                    };
                    let jac_fn = if mp.has_rd_rs() {
                        format!("mosfet_jacobian_with_rd_rs(vgs, vds, state.device_{dev_num}_kp, {vt_expr}, state.device_{dev_num}_lambda, sign, DEVICE_{dev_num}_RD, DEVICE_{dev_num}_RS_PARAM)")
                    } else {
                        format!("mosfet_jacobian(vgs, vds, state.device_{dev_num}_kp, {vt_expr}, state.device_{dev_num}_lambda, sign)")
                    };
                    code.push_str(&format!(
                        "{indent}    let vds = v_nl[{s}];\n\
                         {indent}    let vgs = v_nl[{s1}];\n\
                         {indent}    let sign = DEVICE_{dev_num}_SIGN;\n\
                         {indent}    i_nl[{s}] = mosfet_id(vgs, vds, state.device_{dev_num}_kp, {vt_expr}, state.device_{dev_num}_lambda, sign);\n\
                         {indent}    i_nl[{s1}] = 0.0; // Insulated gate\n\
                         {indent}    let jac = {jac_fn};\n\
                         {indent}    j_dev[{s} * M + {s}] = jac[0];\n\
                         {indent}    j_dev[{s} * M + {s1}] = jac[1];\n\
                         {indent}    j_dev[{s1} * M + {s}] = jac[2];\n\
                         {indent}    j_dev[{s1} * M + {s1}] = jac[3];\n\
                         {indent}}}\n"
                    ));
                }
                (DeviceType::Tube, DeviceParams::Tube(tp)) => {
                    let s1 = s + 1;
                    if tp.has_rgi() {
                        code.push_str(&format!(
                            "{indent}{{ // Tube {dev_num} (RGI)\n\
                             {indent}    let vgk = v_nl[{s}];\n\
                             {indent}    let vpk = v_nl[{s1}];\n\
                             {indent}    i_nl[{s}] = tube_ip_with_rgi(vgk, vpk, state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_lambda, state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset, DEVICE_{dev_num}_RGI);\n\
                             {indent}    i_nl[{s1}] = tube_ig_with_rgi(vgk, state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset, DEVICE_{dev_num}_RGI);\n\
                             {indent}    let jac = tube_jacobian_with_rgi(vgk, vpk, state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset, state.device_{dev_num}_lambda, DEVICE_{dev_num}_RGI);\n\
                             {indent}    j_dev[{s} * M + {s}] = jac[0];\n\
                             {indent}    j_dev[{s} * M + {s1}] = jac[1];\n\
                             {indent}    j_dev[{s1} * M + {s}] = jac[2];\n\
                             {indent}    j_dev[{s1} * M + {s1}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}{{ // Tube {dev_num}\n\
                             {indent}    let vgk = v_nl[{s}];\n\
                             {indent}    let vpk = v_nl[{s1}];\n\
                             {indent}    i_nl[{s}] = tube_ip(vgk, vpk, state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_lambda);\n\
                             {indent}    i_nl[{s1}] = tube_ig(vgk, state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset);\n\
                             {indent}    let jac = tube_jacobian(vgk, vpk, state.device_{dev_num}_mu, state.device_{dev_num}_ex, state.device_{dev_num}_kg1, state.device_{dev_num}_kp, state.device_{dev_num}_kvb, state.device_{dev_num}_ig_max, state.device_{dev_num}_vgk_onset, state.device_{dev_num}_lambda);\n\
                             {indent}    j_dev[{s} * M + {s}] = jac[0];\n\
                             {indent}    j_dev[{s} * M + {s1}] = jac[1];\n\
                             {indent}    j_dev[{s1} * M + {s}] = jac[2];\n\
                             {indent}    j_dev[{s1} * M + {s1}] = jac[3];\n\
                             {indent}}}\n"
                        ));
                    }
                }
                _ => {} // Mismatched type/params — skip
            }
        }

        let _ = _m;
    }

    /// Emit final device evaluation at converged point (writes into existing `i_nl`).
    fn emit_nodal_device_evaluation_final(code: &mut String, ir: &CircuitIR, indent: &str) {
        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            let s = slot.start_idx;
            match (&slot.device_type, &slot.params) {
                (DeviceType::Diode, DeviceParams::Diode(dp)) => {
                    if dp.has_rs() {
                        let bv_i = if dp.has_bv() { format!(" + diode_breakdown_current(v_nl_final[{s}], state.device_{dev_num}_n_vt, DEVICE_{dev_num}_BV, DEVICE_{dev_num}_IBV)") } else { String::new() };
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
                    if bp.has_parasitics() {
                        code.push_str(&format!(
                            "{indent}{{ let vbe = v_nl_final[{s}] * DEVICE_{dev_num}_SIGN;\n\
                             {indent}  let vbc = v_nl_final[{s1}] * DEVICE_{dev_num}_SIGN;\n\
                             {indent}  let (ic, ib, _jac) = bjt_with_parasitics(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE, DEVICE_{dev_num}_RB, DEVICE_{dev_num}_RC, DEVICE_{dev_num}_RE);\n\
                             {indent}  i_nl[{s}] = ic;\n\
                             {indent}  i_nl[{s1}] = ib;\n\
                             {indent}}}\n"
                        ));
                    } else {
                        code.push_str(&format!(
                            "{indent}{{ let vbe = v_nl_final[{s}] * DEVICE_{dev_num}_SIGN;\n\
                             {indent}  let vbc = v_nl_final[{s1}] * DEVICE_{dev_num}_SIGN;\n\
                             {indent}  i_nl[{s}] = bjt_ic(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_USE_GP, DEVICE_{dev_num}_VAF, DEVICE_{dev_num}_VAR, DEVICE_{dev_num}_IKF, DEVICE_{dev_num}_IKR);\n\
                             {indent}  i_nl[{s1}] = bjt_ib(vbe, vbc, state.device_{dev_num}_is, state.device_{dev_num}_vt, DEVICE_{dev_num}_NF, state.device_{dev_num}_bf, state.device_{dev_num}_br, DEVICE_{dev_num}_SIGN, DEVICE_{dev_num}_ISE, DEVICE_{dev_num}_NE);\n\
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
                    if tp.has_rgi() {
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
                _ => {}
            }
        }
    }

    /// Emit SPICE voltage limiting for nodal solver (trapezoidal NR, at default indent).
    fn emit_nodal_voltage_limiting(code: &mut String, ir: &CircuitIR) {
        Self::emit_nodal_voltage_limiting_indented(code, ir, "        ");
    }

    /// Emit SPICE voltage limiting for nodal solver at a given indent level.
    ///
    /// For each nonlinear device dimension, computes the proposed device voltage
    /// from v_new via N_v, applies pnjlim/fetlim, and reduces alpha if needed.
    fn emit_nodal_voltage_limiting_indented(code: &mut String, ir: &CircuitIR, indent: &str) {
        let n = ir.topology.n;

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            for d in 0..slot.dimension {
                let i = slot.start_idx + d;

                // Compute proposed device voltage from v_new via N_v
                code.push_str(&format!("{indent}{{ // Device {dev_num} dim {d}\n"));
                code.push_str(&format!("{indent}    let mut v_nl_proposed = 0.0;\n"));
                code.push_str(&format!("{indent}    for j in 0..N {{ v_nl_proposed += N_V[{i}][j] * v_new[j]; }}\n"));
                code.push_str(&format!("{indent}    let v_nl_current = v_nl[{i}];\n"));
                code.push_str(&format!("{indent}    let dv = v_nl_proposed - v_nl_current;\n"));
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
                    (DeviceType::Tube, _) => {
                        code.push_str(&format!(
                            "{indent}        let v_lim = fetlim(v_nl_proposed, v_nl_current, 0.0);\n"
                        ));
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
