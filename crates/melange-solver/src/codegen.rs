//! Code generation for specialized circuit solvers.
//!
//! This module generates zero-overhead Rust code from a compiled DkKernel.
//! The generated code is specialized for a specific circuit topology with
//! compile-time constant matrices and unrolled loops.

use crate::dk::DkKernel;
use crate::mna::MnaSystem;
use crate::parser::{Element, Netlist};

/// Configuration for code generation
#[derive(Debug, Clone)]
pub struct CodegenConfig {
    /// Circuit name for generated code
    pub circuit_name: String,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Maximum iterations for Newton-Raphson
    pub max_iterations: usize,
    /// Alias for max_iterations (compatibility)
    pub max_iter: usize,
    /// Convergence tolerance
    pub tolerance: f64,
    /// Alias for tolerance (compatibility)
    pub tol: f64,
    /// Input resistance (Thevenin equivalent)
    pub input_resistance: f64,
    /// Input node index
    pub input_node: usize,
    /// Output node index
    pub output_node: usize,
    /// Include DC operating point in generated code
    pub include_dc_op: bool,
    /// Input conductance (for compatibility)
    pub input_conductance: f64,
}

impl Default for CodegenConfig {
    fn default() -> Self {
        Self {
            circuit_name: "unnamed_circuit".to_string(),
            sample_rate: 44100.0,
            max_iterations: 20,
            max_iter: 20,
            tolerance: 1e-9,
            tol: 1e-9,
            input_resistance: 1.0, // 1Ω default (near-ideal voltage source)
            input_node: 0,
            output_node: 0,
            include_dc_op: true,
            input_conductance: 1.0, // 1/1Ω
        }
    }
}

/// Error type for code generation failures
#[derive(Debug, Clone)]
pub enum CodegenError {
    /// Invalid kernel configuration
    InvalidKernel(String),
    /// Unsupported circuit topology
    UnsupportedTopology(String),
    /// Invalid device model
    InvalidDevice(String),
}

impl std::fmt::Display for CodegenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CodegenError::InvalidKernel(s) => write!(f, "Invalid kernel: {}", s),
            CodegenError::UnsupportedTopology(s) => write!(f, "Unsupported topology: {}", s),
            CodegenError::InvalidDevice(s) => write!(f, "Invalid device: {}", s),
        }
    }
}

impl std::error::Error for CodegenError {}

/// Generated circuit solver code
#[derive(Debug, Clone)]
pub struct GeneratedCode {
    /// The generated Rust source code
    pub code: String,
    /// Circuit metadata
    pub n: usize,
    pub m: usize,
}

/// Code generator for circuit solvers
pub struct CodeGenerator {
    config: CodegenConfig,
}

impl CodeGenerator {
    /// Create a new code generator with the given configuration
    pub fn new(config: CodegenConfig) -> Self {
        Self { config }
    }

    /// Generate a complete circuit solver module
    ///
    /// # Arguments
    /// * `kernel` - The compiled DK kernel
    /// * `netlist` - Original netlist for extracting component values
    /// * `mna` - MNA system for node mapping
    ///
    /// # Returns
    /// Generated Rust source code as a string
    /// Generate a complete circuit solver module.
    /// 
    /// Alias for `generate_circuit_solver` with the argument order expected by tests.
    pub fn generate(
        &self,
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
    ) -> Result<GeneratedCode, CodegenError> {
        let code = self.generate_circuit_solver(kernel, netlist, mna)?;
        Ok(GeneratedCode {
            code,
            n: kernel.n,
            m: kernel.m,
        })
    }

    /// Generate a complete circuit solver module
    ///
    /// # Arguments
    /// * `kernel` - The compiled DK kernel
    /// * `netlist` - Original netlist for extracting component values
    /// * `mna` - MNA system for node mapping
    ///
    /// # Returns
    /// Generated Rust source code as a string
    pub fn generate_circuit_solver(
        &self,
        kernel: &DkKernel,
        netlist: &Netlist,
        mna: &MnaSystem,
    ) -> Result<String, CodegenError> {
        let mut code = String::new();

        // File header
        self.generate_header(&mut code, netlist);

        // Constants (N, M, SAMPLE_RATE, matrices)
        self.generate_constants(&mut code, kernel);

        // State structure
        self.generate_struct_and_constants(&mut code, kernel, netlist, mna)?;

        // Device models
        self.generate_device_models(&mut code, netlist)?;

        // Helper functions
        self.generate_build_rhs(&mut code, kernel, netlist, mna);
        self.generate_mat_vec_mul_s(&mut code, kernel);
        self.generate_extract_controlling_voltages(&mut code, kernel);
        self.generate_solve_nonlinear(&mut code, kernel, netlist)?;
        self.generate_compute_final_voltages(&mut code, kernel);
        self.generate_update_history(&mut code, kernel, netlist, mna);

        // Main process function
        self.generate_process_sample(&mut code, kernel);

        Ok(code)
    }

    /// Generate file header with metadata
    fn generate_header(&self, code: &mut String, netlist: &Netlist) {
        code.push_str("// ============================================================================\n");
        code.push_str("// Generated by melange-solver\n");
        code.push_str(&format!("// Circuit: \"{}\"\n", &netlist.title));
        code.push_str("// ============================================================================\n\n");
        code.push_str("#![allow(clippy::excessive_precision)]\n");
        code.push_str("#![allow(dead_code)]\n\n");
    }

    /// Generate compile-time constants
    fn generate_constants(&self, code: &mut String, kernel: &DkKernel) {
        let n = kernel.n;
        let m = kernel.m;

        code.push_str("// =============================================================================\n");
        code.push_str("// CONSTANTS: Compile-time circuit topology\n");
        code.push_str("// =============================================================================\n\n");

        code.push_str(&format!("/// Number of circuit nodes (including ground reference)\n"));
        code.push_str(&format!("pub const N: usize = {};\n\n", n));

        code.push_str(&format!("/// Number of nonlinear devices\n"));
        code.push_str(&format!("pub const M: usize = {};\n\n", m));

        code.push_str(&format!("/// Sample rate (Hz)\n"));
        code.push_str(&format!("pub const SAMPLE_RATE: f64 = {:.1};\n\n", self.config.sample_rate));

        code.push_str(&format!("/// Alpha = 2/T for trapezoidal integration\n"));
        let alpha = 2.0 * self.config.sample_rate;
        code.push_str(&format!("pub const ALPHA: f64 = {:.17e};\n\n", alpha));

        code.push_str(&format!("/// Input node index\n"));
        code.push_str(&format!("pub const INPUT_NODE: usize = {};\n\n", self.config.input_node));

        code.push_str(&format!("/// Output node index\n"));
        code.push_str(&format!("pub const OUTPUT_NODE: usize = {};\n\n", self.config.output_node));

        code.push_str(&format!("/// Input resistance (Thevenin equivalent)\n"));
        code.push_str(&format!("pub const INPUT_RESISTANCE: f64 = {:.17e};\n\n", self.config.input_resistance));

        // S matrix
        code.push_str("/// S matrix: S = A⁻¹ where A = G + alpha*C\n");
        code.push_str(&format!("pub const S: [[f64; N]; N] = [\n"));
        for i in 0..n {
            code.push_str("    [");
            for j in 0..n {
                if j > 0 {
                    code.push_str(", ");
                }
                code.push_str(&format!("{:.17e}", kernel.s[i * n + j]));
            }
            code.push_str("],\n");
        }
        code.push_str("];\n\n");

        // A_neg matrix (for RHS computation)
        code.push_str("/// A_neg matrix: A_neg = alpha*C - G\n");
        code.push_str("/// Used for RHS computation: A_neg * v_prev gives history contribution\n");
        code.push_str(&format!("pub const A_NEG: [[f64; N]; N] = [\n"));
        for i in 0..n {
            code.push_str("    [");
            for j in 0..n {
                if j > 0 {
                    code.push_str(", ");
                }
                code.push_str(&format!("{:.17e}", kernel.a_neg[i * n + j]));
            }
            code.push_str("],\n");
        }
        code.push_str("];\n\n");

        // K matrix
        code.push_str("/// K matrix: Nonlinear kernel K = N_v * S * N_i^T\n");
        code.push_str(&format!("pub const K: [[f64; M]; M] = [\n"));
        for i in 0..m {
            code.push_str("    [");
            for j in 0..m {
                if j > 0 {
                    code.push_str(", ");
                }
                code.push_str(&format!("{:.17e}", kernel.k[i * m + j]));
            }
            code.push_str("],\n");
        }
        code.push_str("];\n\n");

        // N_v matrix
        code.push_str("/// N_v matrix: Extracts controlling voltages from node voltages\n");
        code.push_str(&format!("pub const N_V: [[f64; N]; M] = [\n"));
        for i in 0..m {
            code.push_str("    [");
            for j in 0..n {
                if j > 0 {
                    code.push_str(", ");
                }
                code.push_str(&format!("{:.17e}", kernel.n_v[i * n + j]));
            }
            code.push_str("],\n");
        }
        code.push_str("];\n\n");

        // N_i matrix (transposed from kernel's N×M storage to M×N for codegen)
        // kernel.n_i is stored as (N × M): n_i[node * m + device]
        // We generate N_I[device][node] = kernel.n_i[node * m + device]
        code.push_str("/// N_i matrix (transposed): Maps nonlinear currents to node injections\n");
        code.push_str(&format!("pub const N_I: [[f64; N]; M] = [\n"));
        for i in 0..m {
            code.push_str("    [");
            for j in 0..n {
                if j > 0 {
                    code.push_str(", ");
                }
                // N_I[device_i][node_j] = kernel.n_i[node_j * m + device_i]
                code.push_str(&format!("{:.17e}", kernel.n_i[j * m + i]));
            }
            code.push_str("],\n");
        }
        code.push_str("];\n\n");
    }

    /// Generate state structure and related constants
    fn generate_struct_and_constants(
        &self,
        code: &mut String,
        kernel: &DkKernel,
        _netlist: &Netlist,
        _mna: &MnaSystem,
    ) -> Result<(), CodegenError> {
        let _n = kernel.n;
        let _m = kernel.m;

        code.push_str("// =============================================================================\n");
        code.push_str("// STATE STRUCTURE\n");
        code.push_str("// =============================================================================\n\n");

        code.push_str("/// Circuit state for one processing channel\n");
        code.push_str("#[derive(Clone, Debug)]\n");
        code.push_str("pub struct CircuitState {\n");
        code.push_str("    /// Previous node voltages v[n-1]\n");
        code.push_str(&format!("    pub v_prev: [f64; N],\n\n"));

        code.push_str("    /// Previous nonlinear currents i_nl[n-1]\n");
        code.push_str(&format!("    pub i_nl_prev: [f64; M],\n\n"));

        code.push_str("    /// DC operating point (for reset/sleep/wake)\n");
        code.push_str(&format!("    pub dc_operating_point: [f64; N],\n\n"));

        code.push_str("    /// Iteration count from last solve (for diagnostics)\n");
        code.push_str("    pub last_nr_iterations: u32,\n");
        code.push_str("}\n\n");

        code.push_str("impl Default for CircuitState {\n");
        code.push_str("    fn default() -> Self {\n");
        code.push_str("        Self {\n");
        code.push_str(&format!("            v_prev: [0.0; N],\n"));
        code.push_str(&format!("            i_nl_prev: [0.0; M],\n"));
        code.push_str(&format!("            dc_operating_point: [0.0; N],\n"));
        code.push_str("            last_nr_iterations: 0,\n");
        code.push_str("        }\n");
        code.push_str("    }\n");
        code.push_str("}\n\n");

        code.push_str("impl CircuitState {\n");
        code.push_str("    /// Reset to DC operating point\n");
        code.push_str("    pub fn reset(&mut self) {\n");
        code.push_str("        self.v_prev = self.dc_operating_point;\n");
        code.push_str(&format!("        self.i_nl_prev = [0.0; M];\n"));
        code.push_str("        self.last_nr_iterations = 0;\n");
        code.push_str("    }\n\n");

        code.push_str("    /// Set DC operating point (call after DC analysis)\n");
        code.push_str("    pub fn set_dc_operating_point(&mut self, v_dc: [f64; N]) {\n");
        code.push_str("        self.dc_operating_point = v_dc;\n");
        code.push_str("        self.v_prev = v_dc;\n");
        code.push_str("    }\n");
        code.push_str("}\n\n");

        Ok(())
    }

    /// Generate device model functions
    fn generate_device_models(&self, code: &mut String, netlist: &Netlist) -> Result<(), CodegenError> {
        code.push_str("// =============================================================================\n");
        code.push_str("// DEVICE MODELS\n");
        code.push_str("// =============================================================================\n\n");

        // Check device types
        let has_diodes = netlist.elements.iter().any(|e| matches!(e, Element::Diode { .. }));
        let has_bjts = netlist.elements.iter().any(|e| matches!(e, Element::Bjt { .. }));

        if has_diodes {
            let vt = 0.02585; // Thermal voltage at room temperature
            let is = 2.52e-9; // Default IS
            let n = 1.0;      // Default ideality factor
            let n_vt = n * vt;

            code.push_str("/// Diode saturation current\n");
            code.push_str(&format!("const DIODE_IS: f64 = {:.17e};\n\n", is));

            code.push_str("/// Diode ideality factor * thermal voltage\n");
            code.push_str(&format!("const DIODE_N_VT: f64 = {:.17e};\n\n", n_vt));

            code.push_str("/// Diode current: i_d = IS * (exp(v_d / (N*VT)) - 1)\n");
            code.push_str("#[inline(always)]\n");
            code.push_str("fn diode_current(v_d: f64) -> f64 {\n");
            code.push_str("    let v_clamped = v_d.clamp(-100.0 * DIODE_N_VT, 100.0 * DIODE_N_VT);\n");
            code.push_str("    DIODE_IS * ((v_clamped / DIODE_N_VT).exp() - 1.0)\n");
            code.push_str("}\n\n");

            code.push_str("/// Diode conductance: g_d = di/dv = (IS / (N*VT)) * exp(v_d / (N*VT))\n");
            code.push_str("#[inline(always)]\n");
            code.push_str("fn diode_conductance(v_d: f64) -> f64 {\n");
            code.push_str("    let v_clamped = v_d.clamp(-100.0 * DIODE_N_VT, 100.0 * DIODE_N_VT);\n");
            code.push_str("    (DIODE_IS / DIODE_N_VT) * (v_clamped / DIODE_N_VT).exp()\n");
            code.push_str("}\n\n");
        }

        if has_bjts {
            let vt = 0.02585;
            let is = 1.26e-14; // Default IS for 2N2222A
            let beta_f = 200.0;
            let beta_r = 3.0;

            code.push_str(&format!("const BJT_IS: f64 = {:.17e};\n", is));
            code.push_str(&format!("const BJT_VT: f64 = {:.17e};\n", vt));
            code.push_str(&format!("const BJT_BETA_F: f64 = {:.17e};\n", beta_f));
            code.push_str(&format!("const BJT_BETA_R: f64 = {:.17e};\n\n", beta_r));

            code.push_str("/// Safe exponential (clamp to prevent overflow)\n");
            code.push_str("#[inline(always)]\n");
            code.push_str("fn safe_exp(x: f64) -> f64 { x.clamp(-500.0, 500.0).exp() }\n\n");

            code.push_str("/// BJT Ebers-Moll collector current Ic(Vbe, Vbc)\n");
            code.push_str("#[inline(always)]\n");
            code.push_str("fn bjt_ic(vbe: f64, vbc: f64) -> f64 {\n");
            code.push_str("    let exp_be = safe_exp(vbe / BJT_VT);\n");
            code.push_str("    let exp_bc = safe_exp(vbc / BJT_VT);\n");
            code.push_str("    BJT_IS * (exp_be - exp_bc) - BJT_IS / BJT_BETA_R * (exp_bc - 1.0)\n");
            code.push_str("}\n\n");

            code.push_str("/// BJT Ebers-Moll base current Ib(Vbe, Vbc)\n");
            code.push_str("#[inline(always)]\n");
            code.push_str("fn bjt_ib(vbe: f64, vbc: f64) -> f64 {\n");
            code.push_str("    let exp_be = safe_exp(vbe / BJT_VT);\n");
            code.push_str("    let exp_bc = safe_exp(vbc / BJT_VT);\n");
            code.push_str("    BJT_IS / BJT_BETA_F * (exp_be - 1.0) + BJT_IS / BJT_BETA_R * (exp_bc - 1.0)\n");
            code.push_str("}\n\n");

            code.push_str("/// BJT Jacobian: [dIc/dVbe, dIc/dVbc, dIb/dVbe, dIb/dVbc]\n");
            code.push_str("#[inline(always)]\n");
            code.push_str("fn bjt_jacobian(vbe: f64, vbc: f64) -> [f64; 4] {\n");
            code.push_str("    let exp_be = safe_exp(vbe / BJT_VT);\n");
            code.push_str("    let exp_bc = safe_exp(vbc / BJT_VT);\n");
            code.push_str("    let dic_dvbe = BJT_IS / BJT_VT * exp_be;\n");
            code.push_str("    let dic_dvbc = -(BJT_IS / BJT_VT) * exp_bc - (BJT_IS / (BJT_BETA_R * BJT_VT)) * exp_bc;\n");
            code.push_str("    let dib_dvbe = (BJT_IS / (BJT_BETA_F * BJT_VT)) * exp_be;\n");
            code.push_str("    let dib_dvbc = (BJT_IS / (BJT_BETA_R * BJT_VT)) * exp_bc;\n");
            code.push_str("    [dic_dvbe, dic_dvbc, dib_dvbe, dib_dvbc]\n");
            code.push_str("}\n\n");
        }

        Ok(())
    }

    /// Generate build_rhs function
    fn generate_build_rhs(&self, code: &mut String, kernel: &DkKernel, _netlist: &Netlist, _mna: &MnaSystem) {
        let n = kernel.n;
        let m = kernel.m;

        code.push_str("/// Build RHS vector: rhs = A_neg * v_prev + N_i^T * i_nl_prev + input\n");
        code.push_str("/// \n");
        code.push_str("/// The A_neg * v_prev term captures capacitor history contribution\n");
        code.push_str("/// via alpha*C*v_prev (implicit in the matrix formulation).\n");
        code.push_str("#[inline(always)]\n");
        code.push_str(&format!("fn build_rhs(input: f64, state: &CircuitState) -> [f64; N] {{\n"));
        code.push_str("    let mut rhs = [0.0; N];\n\n");

        // A_neg * v_prev contribution (includes capacitor history)
        code.push_str("    // A_neg * v_prev contribution (includes alpha*C*v_prev capacitor history)\n");
        for i in 0..n {
            code.push_str(&format!("    rhs[{}] = ", i));
            let mut first = true;
            for j in 0..n {
                if kernel.a_neg[i * n + j] != 0.0 {
                    if !first {
                        code.push_str(" + ");
                    }
                    code.push_str(&format!("A_NEG[{}][{}] * state.v_prev[{}]", i, j, j));
                    first = false;
                }
            }
            if first {
                code.push_str("0.0");
            }
            code.push_str(";\n");
        }
        code.push_str("\n");

        // N_i * i_nl_prev contribution
        // N_I (generated constant) is M×N = transpose of kernel N_i (N×M)
        // rhs[i] += sum_j N_I[j][i] * i_nl_prev[j] = sum_j N_i[i][j] * i_nl_prev[j]
        if m > 0 {
            code.push_str("    // Add N_i * i_nl_prev contribution\n");
            for i in 0..n {
                for j in 0..m {
                    if kernel.n_i[i * m + j] != 0.0 {
                        code.push_str(&format!(
                            "    rhs[{}] += N_I[{}][{}] * state.i_nl_prev[{}];\n",
                            i, j, i, j
                        ));
                    }
                }
            }
            code.push_str("\n");
        }

        // Input contribution — factor of 2 from trapezoidal discretization
        code.push_str("    // Add input contribution (Thevenin: 2 * V_in * G_in)\n");
        code.push_str("    rhs[INPUT_NODE] += 2.0 * input / INPUT_RESISTANCE;\n\n");

        code.push_str("    rhs\n");
        code.push_str("}\n\n");
    }

    /// Generate matrix-vector multiplication with S
    fn generate_mat_vec_mul_s(&self, code: &mut String, kernel: &DkKernel) {
        let n = kernel.n;

        code.push_str("/// Multiply vector by S matrix: result = S * rhs\n");
        code.push_str("#[inline(always)]\n");
        code.push_str(&format!("fn mat_vec_mul_s(rhs: &[f64; N]) -> [f64; N] {{\n"));
        code.push_str("    [\n");

        for i in 0..n {
            code.push_str("        ");
            let mut first = true;
            for j in 0..n {
                if kernel.s[i * n + j] != 0.0 || (!first && j == n - 1) {
                    if !first {
                        code.push_str(" + ");
                    }
                    code.push_str(&format!("S[{}][{}] * rhs[{}]", i, j, j));
                    first = false;
                }
            }
            if first {
                code.push_str("0.0");
            }
            code.push_str(",\n");
        }

        code.push_str("    ]\n");
        code.push_str("}\n\n");
    }

    /// Generate extract_controlling_voltages function
    fn generate_extract_controlling_voltages(&self, code: &mut String, kernel: &DkKernel) {
        let n = kernel.n;
        let m = kernel.m;

        code.push_str("/// Extract controlling voltages: p = N_v * v_pred\n");
        code.push_str("#[inline(always)]\n");
        code.push_str(&format!("fn extract_controlling_voltages(v_pred: &[f64; N]) -> [f64; M] {{\n"));
        code.push_str("    [\n");

        for i in 0..m {
            code.push_str("        ");
            let mut first = true;
            for j in 0..n {
                if kernel.n_v[i * n + j] != 0.0 {
                    if !first {
                        if kernel.n_v[i * n + j] > 0.0 {
                            code.push_str(" + ");
                        } else {
                            code.push_str(" - ");
                        }
                    }
                    let abs_val = kernel.n_v[i * n + j].abs();
                    if (abs_val - 1.0).abs() < 1e-15 {
                        code.push_str(&format!("v_pred[{}]", j));
                    } else {
                        code.push_str(&format!("{:.17e} * v_pred[{}]", abs_val, j));
                    }
                    first = false;
                }
            }
            if first {
                code.push_str("0.0");
            }
            code.push_str(",\n");
        }

        code.push_str("    ]\n");
        code.push_str("}\n\n");
    }


    /// Generate solve_nonlinear function using Newton-Raphson
    fn generate_solve_nonlinear(
        &self,
        code: &mut String,
        kernel: &DkKernel,
        netlist: &Netlist,
    ) -> Result<(), CodegenError> {
        let m = kernel.m;

        // Build device map from netlist (mirrors MNA builder device ordering)
        // Each entry: (device_type, start_idx_in_M, dimension)
        let mut device_slots: Vec<(&str, usize, usize)> = Vec::new();
        let mut dim_offset = 0;
        for elem in &netlist.elements {
            match elem {
                Element::Diode { .. } => {
                    device_slots.push(("diode", dim_offset, 1));
                    dim_offset += 1;
                }
                Element::Bjt { .. } => {
                    device_slots.push(("bjt", dim_offset, 2));
                    dim_offset += 2;
                }
                Element::Jfet { .. } => {
                    device_slots.push(("jfet", dim_offset, 1));
                    dim_offset += 1;
                }
                Element::Mosfet { .. } => {
                    device_slots.push(("mosfet", dim_offset, 1));
                    dim_offset += 1;
                }
                _ => {}
            }
        }
        let has_nonlinear = dim_offset > 0;

        code.push_str("/// Solve M×M nonlinear system via Newton-Raphson\n");
        code.push_str("/// \n");
        code.push_str("/// Solves: i_nl - i_d(p + K*i_nl) = 0\n");
        code.push_str("/// where p = N_v * v_pred is the linear prediction\n");
        code.push_str("#[inline(always)]\n");
        code.push_str(&format!("fn solve_nonlinear(p: &[f64; M], state: &mut CircuitState) -> [f64; M] {{\n"));
        code.push_str(&format!("    const MAX_ITER: usize = {};\n", self.config.max_iterations));
        code.push_str(&format!("    const TOL: f64 = {:.17e};\n", self.config.tolerance));
        code.push_str("    const STEP_CLAMP: f64 = 0.01;  // Prevent overshoot\n\n");

        // Initial guess from previous sample (warm start)
        code.push_str("    // Initial guess from previous sample (warm start)\n");
        code.push_str("    let mut i_nl = state.i_nl_prev;\n\n");

        if m == 0 {
            code.push_str("    // No nonlinear devices\n");
            code.push_str("    state.last_nr_iterations = 0;\n");
            code.push_str("    i_nl\n");
            code.push_str("}\n\n");
            return Ok(());
        }

        // Generate controlling voltages computation
        code.push_str("    // Newton-Raphson iteration\n");
        code.push_str("    for iter in 0..MAX_ITER {\n\n");

        // Compute v_d = p + K * i_nl
        code.push_str("        // Compute controlling voltages: v_d = p + K * i_nl\n");
        for i in 0..m {
            code.push_str(&format!("        let v_d{} = ", i));
            // p contribution
            code.push_str(&format!("p[{}]", i));
            // +K * i_nl contribution
            for j in 0..m {
                if kernel.k[i * m + j] != 0.0 {
                    code.push_str(&format!(" + K[{}][{}] * i_nl[{}]", i, j, j));
                }
            }
            code.push_str(";\n");
        }
        code.push_str("\n");

        if has_nonlinear {
            // Generate device currents and Jacobian entries
            // Each device contributes its currents and partial derivatives
            code.push_str("        // Evaluate device currents and Jacobians\n");
            for (dev_num, &(dev_type, start, _dim)) in device_slots.iter().enumerate() {
                match dev_type {
                    "diode" => {
                        code.push_str(&format!(
                            "        let i_dev{} = diode_current(v_d{});\n", start, start
                        ));
                        code.push_str(&format!(
                            "        let jdev_{}_{} = diode_conductance(v_d{});\n", start, start, start
                        ));
                    }
                    "bjt" => {
                        let s1 = start + 1;
                        code.push_str(&format!(
                            "        let i_dev{} = bjt_ic(v_d{}, v_d{});\n", start, start, s1
                        ));
                        code.push_str(&format!(
                            "        let i_dev{} = bjt_ib(v_d{}, v_d{});\n", s1, start, s1
                        ));
                        code.push_str(&format!(
                            "        let bjt{}_jac = bjt_jacobian(v_d{}, v_d{});\n", dev_num, start, s1
                        ));
                        code.push_str(&format!("        let jdev_{}_{} = bjt{}_jac[0];\n", start, start, dev_num));
                        code.push_str(&format!("        let jdev_{}_{} = bjt{}_jac[1];\n", start, s1, dev_num));
                        code.push_str(&format!("        let jdev_{}_{} = bjt{}_jac[2];\n", s1, start, dev_num));
                        code.push_str(&format!("        let jdev_{}_{} = bjt{}_jac[3];\n", s1, s1, dev_num));
                    }
                    unsupported => {
                        return Err(CodegenError::UnsupportedTopology(format!(
                            "Device type '{}' not yet supported in NR codegen", unsupported
                        )));
                    }
                }
            }
            code.push_str("\n");

            // Generate residuals
            code.push_str("        // Residuals: f(i) = i - i_dev(v(i)) = 0\n");
            for i in 0..m {
                code.push_str(&format!("        let f{} = i_nl[{}] - i_dev{};\n", i, i, i));
            }
            code.push_str("\n");

            // Generate NR Jacobian: J[i][j] = δ_ij - Σ_k jdev_{ik} * K[k][j]
            // For each row i, sum over k in the same device block as i.
            // For diodes (1D): single term. For BJTs (2D): two terms per entry.
            code.push_str("        // Jacobian: J[i][j] = delta_ij - sum_k(jdev_ik * K[k][j])\n");
            for i in 0..m {
                let &(_, blk_start, blk_dim) = device_slots.iter()
                    .find(|&&(_, s, d)| i >= s && i < s + d)
                    .expect("every M index should belong to a device");
                for j in 0..m {
                    let diag = if i == j { "1.0" } else { "0.0" };
                    let mut terms = String::new();
                    for k in blk_start..blk_start + blk_dim {
                        terms.push_str(&format!(" - jdev_{}_{} * K[{}][{}]", i, k, k, j));
                    }
                    code.push_str(&format!("        let j{}{} = {}{};\n", i, j, diag, terms));
                }
            }
            code.push_str("\n");

            // Solve the linear system based on matrix size
            match m {
                1 => {
                    code.push_str("        // Solve 1×1 system: J * delta = f\n");
                    code.push_str("        let det = j00;\n");
                    code.push_str("        if det.abs() < 1e-15 {\n");
                    code.push_str("            state.last_nr_iterations = MAX_ITER as u32;\n");
                    code.push_str("            return i_nl;  // Singular Jacobian, return best guess\n");
                    code.push_str("        }\n");
                    code.push_str("        let delta0 = f0 / det;\n");
                    code.push_str("\n");
                    code.push_str("        // Clamp step to prevent overshoot\n");
                    code.push_str("        let clamped_delta0 = delta0.clamp(-STEP_CLAMP, STEP_CLAMP);\n");
                    code.push_str("        i_nl[0] -= clamped_delta0;\n\n");
                    code.push_str("        // Convergence check\n");
                    code.push_str("        if clamped_delta0.abs() < TOL {\n");
                    code.push_str("            state.last_nr_iterations = iter as u32;\n");
                    code.push_str("            return i_nl;\n");
                    code.push_str("        }\n");
                }
                2 => {
                    code.push_str("        // Solve 2×2 system: J * delta = f\n");
                    code.push_str("        let det = j00 * j11 - j01 * j10;\n");
                    code.push_str("        if det.abs() < 1e-15 {\n");
                    code.push_str("            state.last_nr_iterations = MAX_ITER as u32;\n");
                    code.push_str("            return i_nl;  // Singular Jacobian, return best guess\n");
                    code.push_str("        }\n");
                    code.push_str("        let inv_det = 1.0 / det;\n");
                    code.push_str("        let delta0 = inv_det * (j11 * f0 - j01 * f1);\n");
                    code.push_str("        let delta1 = inv_det * (-j10 * f0 + j00 * f1);\n");
                    code.push_str("\n");
                    code.push_str("        // Clamp steps to prevent overshoot\n");
                    code.push_str("        let clamped_delta0 = delta0.clamp(-STEP_CLAMP, STEP_CLAMP);\n");
                    code.push_str("        let clamped_delta1 = delta1.clamp(-STEP_CLAMP, STEP_CLAMP);\n");
                    code.push_str("        i_nl[0] -= clamped_delta0;\n");
                    code.push_str("        i_nl[1] -= clamped_delta1;\n\n");
                    code.push_str("        // Convergence check\n");
                    code.push_str("        if clamped_delta0.abs() + clamped_delta1.abs() < TOL {\n");
                    code.push_str("            state.last_nr_iterations = iter as u32;\n");
                    code.push_str("            return i_nl;\n");
                    code.push_str("        }\n");
                }
                3 => {
                    code.push_str("        // Solve 3×3 system via inline Gaussian elimination\n");
                    code.push_str("        let mut a = [\n");
                    code.push_str("            [j00, j01, j02],\n");
                    code.push_str("            [j10, j11, j12],\n");
                    code.push_str("            [j20, j21, j22],\n");
                    code.push_str("        ];\n");
                    code.push_str("        let mut b = [f0, f1, f2];\n");
                    code.push_str("        let mut singular = false;\n");
                    code.push_str("        // Forward elimination with partial pivoting\n");
                    code.push_str("        for col in 0..3 {\n");
                    code.push_str("            let mut max_row = col;\n");
                    code.push_str("            let mut max_val = a[col][col].abs();\n");
                    code.push_str("            for row in (col+1)..3 {\n");
                    code.push_str("                if a[row][col].abs() > max_val {\n");
                    code.push_str("                    max_val = a[row][col].abs();\n");
                    code.push_str("                    max_row = row;\n");
                    code.push_str("                }\n");
                    code.push_str("            }\n");
                    code.push_str("            if max_val < 1e-15 { singular = true; break; }\n");
                    code.push_str("            if max_row != col { a.swap(col, max_row); b.swap(col, max_row); }\n");
                    code.push_str("            let pivot = a[col][col];\n");
                    code.push_str("            for row in (col+1)..3 {\n");
                    code.push_str("                let factor = a[row][col] / pivot;\n");
                    code.push_str("                for j in (col+1)..3 { a[row][j] -= factor * a[col][j]; }\n");
                    code.push_str("                b[row] -= factor * b[col];\n");
                    code.push_str("            }\n");
                    code.push_str("        }\n");
                    code.push_str("        if !singular {\n");
                    code.push_str("            // Back substitution\n");
                    code.push_str("            for i in (0..3).rev() {\n");
                    code.push_str("                let mut sum = b[i];\n");
                    code.push_str("                for j in (i+1)..3 { sum -= a[i][j] * b[j]; }\n");
                    code.push_str("                b[i] = sum / a[i][i];\n");
                    code.push_str("            }\n");
                    for i in 0..3 {
                        code.push_str(&format!(
                            "            let clamped_delta{} = b[{}].clamp(-STEP_CLAMP, STEP_CLAMP);\n",
                            i, i
                        ));
                        code.push_str(&format!("            i_nl[{}] -= clamped_delta{};\n", i, i));
                    }
                    code.push_str("            // Convergence check\n");
                    code.push_str("            if clamped_delta0.abs() + clamped_delta1.abs() + clamped_delta2.abs() < TOL {\n");
                    code.push_str("                state.last_nr_iterations = iter as u32;\n");
                    code.push_str("                return i_nl;\n");
                    code.push_str("            }\n");
                    code.push_str("        }\n");
                }
                4 => {
                    code.push_str("        // Solve 4×4 system via inline Gaussian elimination\n");
                    code.push_str("        let mut a = [\n");
                    code.push_str("            [j00, j01, j02, j03],\n");
                    code.push_str("            [j10, j11, j12, j13],\n");
                    code.push_str("            [j20, j21, j22, j23],\n");
                    code.push_str("            [j30, j31, j32, j33],\n");
                    code.push_str("        ];\n");
                    code.push_str("        let mut b = [f0, f1, f2, f3];\n");
                    code.push_str("        let mut singular = false;\n");
                    code.push_str("        // Forward elimination with partial pivoting\n");
                    code.push_str("        for col in 0..4 {\n");
                    code.push_str("            let mut max_row = col;\n");
                    code.push_str("            let mut max_val = a[col][col].abs();\n");
                    code.push_str("            for row in (col+1)..4 {\n");
                    code.push_str("                if a[row][col].abs() > max_val {\n");
                    code.push_str("                    max_val = a[row][col].abs();\n");
                    code.push_str("                    max_row = row;\n");
                    code.push_str("                }\n");
                    code.push_str("            }\n");
                    code.push_str("            if max_val < 1e-15 { singular = true; break; }\n");
                    code.push_str("            if max_row != col { a.swap(col, max_row); b.swap(col, max_row); }\n");
                    code.push_str("            let pivot = a[col][col];\n");
                    code.push_str("            for row in (col+1)..4 {\n");
                    code.push_str("                let factor = a[row][col] / pivot;\n");
                    code.push_str("                for j in (col+1)..4 { a[row][j] -= factor * a[col][j]; }\n");
                    code.push_str("                b[row] -= factor * b[col];\n");
                    code.push_str("            }\n");
                    code.push_str("        }\n");
                    code.push_str("        if !singular {\n");
                    code.push_str("            // Back substitution\n");
                    code.push_str("            for i in (0..4).rev() {\n");
                    code.push_str("                let mut sum = b[i];\n");
                    code.push_str("                for j in (i+1)..4 { sum -= a[i][j] * b[j]; }\n");
                    code.push_str("                b[i] = sum / a[i][i];\n");
                    code.push_str("            }\n");
                    for i in 0..4 {
                        code.push_str(&format!(
                            "            let clamped_delta{} = b[{}].clamp(-STEP_CLAMP, STEP_CLAMP);\n",
                            i, i
                        ));
                        code.push_str(&format!("            i_nl[{}] -= clamped_delta{};\n", i, i));
                    }
                    code.push_str("            // Convergence check\n");
                    code.push_str("            if clamped_delta0.abs() + clamped_delta1.abs() + clamped_delta2.abs() + clamped_delta3.abs() < TOL {\n");
                    code.push_str("                state.last_nr_iterations = iter as u32;\n");
                    code.push_str("                return i_nl;\n");
                    code.push_str("            }\n");
                    code.push_str("        }\n");
                }
                _ => {
                    return Err(CodegenError::UnsupportedTopology(format!(
                        "M={} nonlinear devices not supported (max 4)",
                        m
                    )));
                }
            }
        } else {
            // No nonlinear devices - return initial guess
            code.push_str("        // No nonlinear devices to solve\n");
            code.push_str("        state.last_nr_iterations = 0;\n");
            code.push_str("        return i_nl;\n");
        }

        code.push_str("    }\n\n");

        // Max iterations reached
        code.push_str("    // Max iterations reached - return best guess\n");
        code.push_str("    state.last_nr_iterations = MAX_ITER as u32;\n");
        
        // NaN/Inf check on output
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

    /// Generate compute_final_voltages function
    fn generate_compute_final_voltages(&self, code: &mut String, kernel: &DkKernel) {
        let n = kernel.n;
        let m = kernel.m;

        code.push_str("/// Compute final node voltages: v = v_pred + S * N_i^T * (i_nl - i_nl_prev)\n");
        code.push_str("#[inline(always)]\n");
        code.push_str(&format!("fn compute_final_voltages(\n"));
        code.push_str("    v_pred: &[f64; N],\n");
        code.push_str("    i_nl: &[f64; M],\n");
        code.push_str("    state: &CircuitState,\n");
        code.push_str(&format!(") -> [f64; N] {{\n"
        ));

        // Compute delta_i = i_nl - i_nl_prev for each nonlinear device
        if m > 0 {
            code.push_str("    // Compute current differences\n");
            for j in 0..m {
                code.push_str(&format!(
                    "    let di{} = i_nl[{}] - state.i_nl_prev[{}];\n",
                    j, j, j
                ));
            }
            code.push_str("\n");
        }

        // Compute v = v_pred + correction
        code.push_str("    // Compute final voltages with correction term\n");
        code.push_str("    [\n");
        for i in 0..n {
            code.push_str(&format!("        v_pred[{}]", i));
            
            // Add correction: sum over j of (S * N_i)[i][j] * di[j]
            // (S * N_i)[i][j] = sum_k S[i][k] * N_i[k][j]
            for j in 0..m {
                let mut s_ni_ij = 0.0;
                for k in 0..n {
                    s_ni_ij += kernel.s[i * n + k] * kernel.n_i[k * m + j];
                }
                if s_ni_ij != 0.0 {
                    code.push_str(&format!(" + {:.17e} * di{}", s_ni_ij, j));
                }
            }
            code.push_str(",\n");
        }
        code.push_str("    ]\n");
        code.push_str("}\n\n");
    }

    /// Generate update_history function.
    /// 
    /// NOTE: For the DK method with A_neg formulation, explicit capacitor history
    /// tracking is NOT needed. The A_neg * v_prev term in build_rhs already
    /// includes alpha*C*v_prev which captures the capacitor history contribution.
    fn generate_update_history(&self, code: &mut String, _kernel: &DkKernel, _netlist: &Netlist, _mna: &MnaSystem) {
        code.push_str("/// Update capacitor history for companion model.\n");
        code.push_str("/// \n");
        code.push_str("/// NOTE: In the DK method, capacitor history is implicitly handled by\n");
        code.push_str("/// the A_neg * v_prev term in build_rhs. This function is a no-op.\n");
        code.push_str("#[inline(always)]\n");
        code.push_str("fn update_history(_v: &[f64; N], _state: &mut CircuitState) {\n");
        code.push_str("    // Capacitor history is handled implicitly by A_neg * v_prev\n");
        code.push_str("}\n\n");
    }

    /// Generate process_sample function
    fn generate_process_sample(&self, code: &mut String, _kernel: &DkKernel) {
        code.push_str("// =============================================================================\n");
        code.push_str("// MAIN PROCESS FUNCTION\n");
        code.push_str("// =============================================================================\n\n");

        code.push_str("/// Process a single audio sample through the circuit\n");
        code.push_str("/// \n");
        code.push_str("/// # Arguments\n");
        code.push_str("/// * `input` - Input voltage at INPUT_NODE\n");
        code.push_str("/// * `state` - Mutable circuit state (preserved across calls)\n");
        code.push_str("/// \n");
        code.push_str("/// # Returns\n");
        code.push_str("/// Output voltage at OUTPUT_NODE\n");
        code.push_str("#[inline]\n");
        code.push_str("pub fn process_sample(input: f64, state: &mut CircuitState) -> f64 {\n");
        code.push_str("    // Validate input — handle NaN, inf, and extreme values\n");
        code.push_str("    let input = if input.is_finite() { input.clamp(-100.0, 100.0) } else { 0.0 };\n\n");
        code.push_str("    // Step 1: Build RHS vector from history and input\n");
        code.push_str("    let rhs = build_rhs(input, state);\n\n");
        
        code.push_str("    // Step 2: Linear prediction: v_pred = S * rhs\n");
        code.push_str("    let v_pred = mat_vec_mul_s(&rhs);\n\n");
        
        code.push_str("    // Step 3: Extract controlling voltages for nonlinear devices\n");
        code.push_str("    let p = extract_controlling_voltages(&v_pred);\n\n");
        
        code.push_str("    // Step 4: Solve M×M nonlinear system via Newton-Raphson\n");
        code.push_str("    let i_nl = solve_nonlinear(&p, state);\n\n");
        
        code.push_str("    // Step 5: Compute final node voltages\n");
        code.push_str("    let v = compute_final_voltages(&v_pred, &i_nl, state);\n\n");
        
        code.push_str("    // Step 6: Update history (no-op, handled by A_neg formulation)\n");
        code.push_str("    update_history(&v, state);\n\n");
        
        code.push_str("    // Step 7: Update state for next iteration\n");
        code.push_str("    state.v_prev = v;\n");
        code.push_str("    state.i_nl_prev = i_nl;\n\n");

        code.push_str("    // Step 8: Sanitize state — if NaN/inf entered, reset to prevent poisoning\n");
        code.push_str("    if !state.v_prev.iter().all(|x| x.is_finite()) {\n");
        code.push_str("        state.v_prev = [0.0; N];\n");
        code.push_str("        state.i_nl_prev = [0.0; M];\n");
        code.push_str("        return 0.0;\n");
        code.push_str("    }\n\n");

        code.push_str("    // Return output voltage, clamped to safe audio range\n");
        code.push_str("    let out = v[OUTPUT_NODE];\n");
        code.push_str("    if out.is_finite() { out.clamp(-10.0, 10.0) } else { 0.0 }\n");
        code.push_str("}\n");
    }

    /// Extract capacitors from netlist
    #[allow(dead_code)]
    fn extract_capacitors(&self, netlist: &Netlist) -> Vec<(String, String, f64)> {
        use crate::parser::Element;
        netlist.elements.iter().filter_map(|e| {
            if let Element::Capacitor { n_plus, n_minus, value, .. } = e {
                Some((n_plus.clone(), n_minus.clone(), *value))
            } else {
                None
            }
        }).collect()
    }

    /// Convert node name to index (0-based, ground = None)
    #[allow(dead_code)]
    fn node_name_to_index(&self, name: &str, mna: &MnaSystem) -> Option<usize> {
        if name == "0" || name.to_lowercase() == "ground" {
            return None;
        }
        // Find index in node_map (1-based in MNA, convert to 0-based)
        mna.node_map.get(name).copied().map(|n| n.saturating_sub(1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_codegen_config_default() {
        let config = CodegenConfig::default();
        assert_eq!(config.sample_rate, 44100.0);
        assert_eq!(config.max_iterations, 20);
        assert!(config.tolerance > 0.0);
    }
}
