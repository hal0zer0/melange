//! Rust code emitter implementing the Emitter trait.
//!
//! Generates a standalone Rust module containing a specialized circuit solver.
//! All matrices are emitted as compile-time constants and loops are unrolled.
//!
//! Uses Tera templates for declarative sections and procedural code for the
//! complex NR solver (deeply conditional M=1/2/3..8 branching).
//!
//! ## Module structure
//!
//! - `helpers` — template data structs, formatting utils, pentode/oversampling helpers
//! - `dk_emitter` — DK-path emit methods (constants, state, process_sample, etc.)
//! - `dk_solver` — DK NR solver generation (solve_nonlinear, Gauss elimination)
//! - `nr_helpers` — shared NR helper functions (voltage limiting, convergence)
//! - `nodal_emitter` — nodal-path emit methods (Schur + full-LU)

mod helpers;
mod dk_emitter;
mod dk_solver;
mod nr_helpers;
mod nodal_emitter;

use tera::{Context, Tera};

use super::emitter::Emitter;
use super::ir::CircuitIR;
use super::CodegenError;

use helpers::collapse_blank_lines;

// Embed templates at compile time — no runtime file loading.
const TMPL_HEADER: &str = include_str!("../../../templates/rust/header.rs.tera");
const TMPL_CONSTANTS: &str = include_str!("../../../templates/rust/constants.rs.tera");
const TMPL_STATE: &str = include_str!("../../../templates/rust/state.rs.tera");
const TMPL_DEVICE_DIODE: &str = include_str!("../../../templates/rust/device_diode.rs.tera");
const TMPL_DEVICE_BJT: &str = include_str!("../../../templates/rust/device_bjt.rs.tera");
const TMPL_DEVICE_JFET: &str = include_str!("../../../templates/rust/device_jfet.rs.tera");
const TMPL_DEVICE_MOSFET: &str = include_str!("../../../templates/rust/device_mosfet.rs.tera");
const TMPL_DEVICE_TUBE: &str = include_str!("../../../templates/rust/device_tube.rs.tera");
const TMPL_DEVICE_VCA: &str = include_str!("../../../templates/rust/device_vca.rs.tera");
const TMPL_BUILD_RHS: &str = include_str!("../../../templates/rust/build_rhs.rs.tera");
const TMPL_MAT_VEC_MUL_S: &str = include_str!("../../../templates/rust/mat_vec_mul_s.rs.tera");
const TMPL_EXTRACT_VOLTAGES: &str = include_str!("../../../templates/rust/extract_voltages.rs.tera");
const TMPL_FINAL_VOLTAGES: &str = include_str!("../../../templates/rust/final_voltages.rs.tera");
const TMPL_UPDATE_HISTORY: &str = include_str!("../../../templates/rust/update_history.rs.tera");
const TMPL_PROCESS_SAMPLE: &str = include_str!("../../../templates/rust/process_sample.rs.tera");
const TMPL_SPICE_LIMITING: &str = include_str!("../../../templates/rust/spice_limiting.rs.tera");

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
            ("device_vca", TMPL_DEVICE_VCA),
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

    pub(super) fn render(&self, name: &str, ctx: &Context) -> Result<String, CodegenError> {
        self.tera
            .render(name, ctx)
            .map_err(|e| {
                use std::error::Error;
                let mut msg = format!("{}: {}", name, e);
                let mut source = e.source();
                while let Some(s) = source {
                    msg.push_str(&format!("\n  caused by: {}", s));
                    source = s.source();
                }
                CodegenError::TemplateError(msg)
            })
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
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };

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

        let code = match ir.solver_mode {
            SolverMode::Dk => self.emit_dk(ir)?,
            SolverMode::Nodal => self.emit_nodal(ir)?,
        };

        Ok(collapse_blank_lines(&code))
    }
}
