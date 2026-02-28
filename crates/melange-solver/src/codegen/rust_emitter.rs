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
use super::ir::{CircuitIR, DeviceParams, DeviceType};
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
        code.push_str(&self.emit_state(ir)?);
        code.push_str(&self.emit_device_models(ir)?);
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

        // Inductor data
        let num_inductors = ir.inductors.len();
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            // Build inductor data with g_eq formatted to full precision
            let inductors_data: Vec<InductorTemplateData> = ir.inductors.iter().map(|ind| {
                InductorTemplateData {
                    name: ind.name.clone(),
                    node_i: ind.node_i,
                    node_j: ind.node_j,
                    g_eq: format!("{:.17e}", ind.g_eq),
                }
            }).collect();
            ctx.insert("inductors", &inductors_data);
        }
        ctx.insert(
            "sample_rate",
            &format!("{:.1}", ir.solver_config.sample_rate),
        );
        ctx.insert("alpha", &format!("{:.17e}", ir.solver_config.alpha));
        ctx.insert("input_node", &ir.solver_config.input_node);
        ctx.insert("output_node", &ir.solver_config.output_node);
        ctx.insert(
            "input_resistance",
            &format!("{:.17e}", ir.solver_config.input_resistance),
        );
        ctx.insert("has_dc_sources", &ir.has_dc_sources);

        // S matrix rows
        let s_rows: Vec<String> = (0..n)
            .map(|i| {
                (0..n)
                    .map(|j| format!("{:.17e}", ir.s(i, j)))
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .collect();
        ctx.insert("s_rows", &s_rows);

        // A_neg matrix rows
        let a_neg_rows: Vec<String> = (0..n)
            .map(|i| {
                (0..n)
                    .map(|j| format!("{:.17e}", ir.a_neg(i, j)))
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .collect();
        ctx.insert("a_neg_rows", &a_neg_rows);

        // RHS_CONST
        if ir.has_dc_sources {
            let rhs_const_values = (0..n)
                .map(|i| format!("{:.17e}", ir.matrices.rhs_const[i]))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("rhs_const_values", &rhs_const_values);
        }

        // K matrix rows
        let k_rows: Vec<String> = (0..m)
            .map(|i| {
                (0..m)
                    .map(|j| format!("{:.17e}", ir.k(i, j)))
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .collect();
        ctx.insert("k_rows", &k_rows);

        // N_v matrix rows
        let n_v_rows: Vec<String> = (0..m)
            .map(|i| {
                (0..n)
                    .map(|j| format!("{:.17e}", ir.n_v(i, j)))
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .collect();
        ctx.insert("n_v_rows", &n_v_rows);

        // N_i matrix rows (transposed: N_I[device][node] = n_i[node][device])
        let n_i_rows: Vec<String> = (0..m)
            .map(|i| {
                (0..n)
                    .map(|j| format!("{:.17e}", ir.n_i(j, i)))
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .collect();
        ctx.insert("n_i_rows", &n_i_rows);

        self.render("constants", &ctx)
    }

    fn emit_state(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut ctx = Context::new();
        ctx.insert("has_dc_op", &ir.has_dc_op);
        ctx.insert("num_inductors", &ir.inductors.len());

        if ir.has_dc_op {
            let dc_op_values = ir
                .dc_operating_point
                .iter()
                .map(|v| format!("{:.17e}", v))
                .collect::<Vec<_>>()
                .join(", ");
            ctx.insert("dc_op_values", &dc_op_values);
        }

        self.render("state", &ctx)
    }

    fn emit_device_models(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let mut code = String::new();
        code.push_str("// =============================================================================\n");
        code.push_str("// DEVICE MODELS\n");
        code.push_str("// =============================================================================\n\n");

        // Emit per-device constants
        let mut has_diode = false;
        let mut has_bjt = false;

        for (dev_num, slot) in ir.device_slots.iter().enumerate() {
            match &slot.params {
                DeviceParams::Diode(dp) => {
                    has_diode = true;
                    code.push_str(&format!(
                        "const DEVICE_{}_IS: f64 = {:.17e};\n",
                        dev_num, dp.is
                    ));
                    code.push_str(&format!(
                        "const DEVICE_{}_N_VT: f64 = {:.17e};\n\n",
                        dev_num, dp.n_vt
                    ));
                }
                DeviceParams::Bjt(bp) => {
                    has_bjt = true;
                    code.push_str(&format!(
                        "const DEVICE_{}_IS: f64 = {:.17e};\n",
                        dev_num, bp.is
                    ));
                    code.push_str(&format!(
                        "const DEVICE_{}_VT: f64 = {:.17e};\n",
                        dev_num, bp.vt
                    ));
                    code.push_str(&format!(
                        "const DEVICE_{}_BETA_F: f64 = {:.17e};\n",
                        dev_num, bp.beta_f
                    ));
                    code.push_str(&format!(
                        "const DEVICE_{}_BETA_R: f64 = {:.17e};\n",
                        dev_num, bp.beta_r
                    ));
                    let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                    code.push_str(&format!(
                        "const DEVICE_{}_SIGN: f64 = {:.1};\n\n",
                        dev_num, sign
                    ));
                }
            }
        }

        // Emit parameterized device functions (once per type used)
        if has_diode {
            code.push_str(&self.render("device_diode", &Context::new())?);
        }
        if has_bjt {
            code.push_str(&self.render("device_bjt", &Context::new())?);
        }

        Ok(code)
    }

    fn emit_build_rhs(&self, ir: &CircuitIR) -> Result<String, CodegenError> {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let mut ctx = Context::new();

        ctx.insert("has_dc_sources", &ir.has_dc_sources);

        // Inductor data for history current injection
        let num_inductors = ir.inductors.len();
        ctx.insert("num_inductors", &num_inductors);
        if num_inductors > 0 {
            let inductors_data: Vec<InductorTemplateData> = ir.inductors.iter().map(|ind| {
                InductorTemplateData {
                    name: ind.name.clone(),
                    node_i: ind.node_i,
                    node_j: ind.node_j,
                    g_eq: String::new(), // not needed for build_rhs
                }
            }).collect();
            ctx.insert("inductors", &inductors_data);
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
                        extract_lines.push_str(&format!("{:.17e} * v_pred[{}]", abs_val, j));
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

        // delta_i lines
        let mut di_lines = String::new();
        if m > 0 {
            for j in 0..m {
                di_lines.push_str(&format!(
                    "    let di{} = i_nl[{}] - state.i_nl_prev[{}];\n",
                    j, j, j
                ));
            }
        }
        ctx.insert("di_lines", &di_lines);

        // voltage computation lines
        let mut voltage_lines = String::new();
        for i in 0..n {
            voltage_lines.push_str(&format!("        v_pred[{}]", i));
            for j in 0..m {
                let mut s_ni_ij = 0.0;
                for k in 0..n {
                    s_ni_ij += ir.s(i, k) * ir.n_i(k, j);
                }
                if s_ni_ij != 0.0 {
                    voltage_lines.push_str(&format!(" + {:.17e} * di{}", s_ni_ij, j));
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
            let inductors_data: Vec<InductorTemplateData> = ir.inductors.iter().map(|ind| {
                InductorTemplateData {
                    name: ind.name.clone(),
                    node_i: ind.node_i,
                    node_j: ind.node_j,
                    g_eq: String::new(), // not needed for process_sample (uses IND_N_G_EQ constants)
                }
            }).collect();
            ctx.insert("inductors", &inductors_data);
        }
        self.render("process_sample", &ctx)
    }
}

// ============================================================================
// Procedural NR solver generation (too complex for templates)
// ============================================================================

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
        code.push_str(&format!(
            "fn solve_nonlinear(p: &[f64; M], state: &mut CircuitState) -> [f64; M] {{\n"
        ));
        code.push_str(&format!(
            "    const MAX_ITER: usize = {};\n",
            ir.solver_config.max_iterations
        ));
        code.push_str(&format!(
            "    const TOL: f64 = {:.17e};\n",
            ir.solver_config.tolerance
        ));
        // NOTE: STEP_CLAMP=0.01 is tighter than the runtime solver's 0.1.
        // Generated code runs unsupervised, so tighter clamping ensures stability
        // across all circuits without manual tuning.
        code.push_str("    const STEP_CLAMP: f64 = 0.01;  // Tighter than runtime (0.1) for unsupervised stability\n");
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
        for i in 0..m {
            code.push_str(&format!("        let v_d{} = ", i));
            code.push_str(&format!("p[{}]", i));
            for j in 0..m {
                if ir.k(i, j) != 0.0 {
                    code.push_str(&format!(" + K[{}][{}] * i_nl[{}]", i, j, j));
                }
            }
            code.push_str(";\n");
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

            // NR Jacobian
            code.push_str(
                "        // Jacobian: J[i][j] = delta_ij - sum_k(jdev_ik * K[k][j])\n",
            );
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
                        terms.push_str(&format!(
                            " - jdev_{}_{} * K[{}][{}]",
                            i, k, k, j
                        ));
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
                    code.push_str("        // Solve 1×1 system: J * delta = f\n");
                    code.push_str("        let det = j00;\n");
                    code.push_str("        if det.abs() < SINGULARITY_THRESHOLD {\n");
                    code.push_str(
                        "            state.last_nr_iterations = MAX_ITER as u32;\n",
                    );
                    code.push_str("            return i_nl;  // Singular Jacobian, return best guess\n");
                    code.push_str("        }\n");
                    code.push_str("        let delta0 = f0 / det;\n");
                    code.push_str("\n");
                    code.push_str("        // Clamp step to prevent overshoot\n");
                    code.push_str(
                        "        let clamped_delta0 = delta0.clamp(-STEP_CLAMP, STEP_CLAMP);\n",
                    );
                    code.push_str("        i_nl[0] -= clamped_delta0;\n\n");
                    code.push_str("        // Convergence check\n");
                    code.push_str("        if clamped_delta0.abs() < TOL {\n");
                    code.push_str(
                        "            state.last_nr_iterations = iter as u32;\n",
                    );
                    code.push_str("            return i_nl;\n");
                    code.push_str("        }\n");
                }
                2 => {
                    code.push_str("        // Solve 2×2 system: J * delta = f\n");
                    code.push_str(
                        "        let det = j00 * j11 - j01 * j10;\n",
                    );
                    code.push_str("        if det.abs() < SINGULARITY_THRESHOLD {\n");
                    code.push_str(
                        "            state.last_nr_iterations = MAX_ITER as u32;\n",
                    );
                    code.push_str("            return i_nl;  // Singular Jacobian, return best guess\n");
                    code.push_str("        }\n");
                    code.push_str("        let inv_det = 1.0 / det;\n");
                    code.push_str(
                        "        let delta0 = inv_det * (j11 * f0 - j01 * f1);\n",
                    );
                    code.push_str(
                        "        let delta1 = inv_det * (-j10 * f0 + j00 * f1);\n",
                    );
                    code.push_str("\n");
                    code.push_str("        // Clamp steps to prevent overshoot\n");
                    code.push_str(
                        "        let clamped_delta0 = delta0.clamp(-STEP_CLAMP, STEP_CLAMP);\n",
                    );
                    code.push_str(
                        "        let clamped_delta1 = delta1.clamp(-STEP_CLAMP, STEP_CLAMP);\n",
                    );
                    code.push_str("        i_nl[0] -= clamped_delta0;\n");
                    code.push_str("        i_nl[1] -= clamped_delta1;\n\n");
                    code.push_str("        // Convergence check\n");
                    code.push_str("        if clamped_delta0.abs() + clamped_delta1.abs() < TOL {\n");
                    code.push_str(
                        "            state.last_nr_iterations = iter as u32;\n",
                    );
                    code.push_str("            return i_nl;\n");
                    code.push_str("        }\n");
                }
                3 => {
                    Self::generate_gauss_elim(code, 3);
                }
                4 => {
                    Self::generate_gauss_elim(code, 4);
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
            "        // Solve {}×{} system via inline Gaussian elimination\n",
            dim, dim
        ));
        code.push_str("        let mut a = [\n");
        for i in 0..dim {
            code.push_str("            [");
            for j in 0..dim {
                if j > 0 {
                    code.push_str(", ");
                }
                code.push_str(&format!("j{}{}", i, j));
            }
            code.push_str("],\n");
        }
        code.push_str("        ];\n");

        code.push_str("        let mut b = [");
        for i in 0..dim {
            if i > 0 {
                code.push_str(", ");
            }
            code.push_str(&format!("f{}", i));
        }
        code.push_str("];\n");

        code.push_str("        let mut singular = false;\n");
        code.push_str("        // Forward elimination with partial pivoting\n");
        code.push_str(&format!(
            "        for col in 0..{} {{\n",
            dim
        ));
        code.push_str("            let mut max_row = col;\n");
        code.push_str("            let mut max_val = a[col][col].abs();\n");
        code.push_str(&format!(
            "            for row in (col+1)..{} {{\n",
            dim
        ));
        code.push_str("                if a[row][col].abs() > max_val {\n");
        code.push_str("                    max_val = a[row][col].abs();\n");
        code.push_str("                    max_row = row;\n");
        code.push_str("                }\n");
        code.push_str("            }\n");
        code.push_str(
            "            if max_val < SINGULARITY_THRESHOLD { singular = true; break; }\n",
        );
        code.push_str(
            "            if max_row != col { a.swap(col, max_row); b.swap(col, max_row); }\n",
        );
        code.push_str("            let pivot = a[col][col];\n");
        code.push_str(&format!(
            "            for row in (col+1)..{} {{\n",
            dim
        ));
        code.push_str("                let factor = a[row][col] / pivot;\n");
        code.push_str(&format!(
            "                for j in (col+1)..{} {{ a[row][j] -= factor * a[col][j]; }}\n",
            dim
        ));
        code.push_str("                b[row] -= factor * b[col];\n");
        code.push_str("            }\n");
        code.push_str("        }\n");

        code.push_str("        if !singular {\n");
        code.push_str("            // Back substitution\n");
        code.push_str(&format!(
            "            for i in (0..{}).rev() {{\n",
            dim
        ));
        code.push_str("                let mut sum = b[i];\n");
        code.push_str(&format!(
            "                for j in (i+1)..{} {{ sum -= a[i][j] * b[j]; }}\n",
            dim
        ));
        code.push_str("                if a[i][i].abs() < SINGULARITY_THRESHOLD { singular = true; break; }\n");
        code.push_str("                b[i] = sum / a[i][i];\n");
        code.push_str("            }\n");
        code.push_str("        }\n");
        code.push_str("        if !singular {\n");

        for i in 0..dim {
            code.push_str(&format!(
                "            let clamped_delta{} = b[{}].clamp(-STEP_CLAMP, STEP_CLAMP);\n",
                i, i
            ));
            code.push_str(&format!(
                "            i_nl[{}] -= clamped_delta{};\n",
                i, i
            ));
        }

        code.push_str("            // Convergence check\n");
        code.push_str("            if ");
        for i in 0..dim {
            if i > 0 {
                code.push_str(" + ");
            }
            code.push_str(&format!("clamped_delta{}.abs()", i));
        }
        code.push_str(" < TOL {\n");
        code.push_str("                state.last_nr_iterations = iter as u32;\n");
        code.push_str("                return i_nl;\n");
        code.push_str("            }\n");
        code.push_str("        } else {\n");
        code.push_str("            // Singular Jacobian — return best guess\n");
        code.push_str("            state.last_nr_iterations = MAX_ITER as u32;\n");
        code.push_str("            return i_nl;\n");
        code.push_str("        }\n");
    }
}
