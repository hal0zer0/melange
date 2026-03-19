//! Code generation for specialized circuit solvers.
//!
//! This module generates zero-overhead Rust code from a compiled DkKernel.
//! The generated code is specialized for a specific circuit topology with
//! compile-time constant matrices and unrolled loops.
//!
//! ## Architecture
//!
//! ```text
//! Netlist → MNA → DkKernel → CircuitIR → Emitter → Source Code
//! ```
//!
//! - [`ir::CircuitIR`] — serializable, language-agnostic intermediate representation
//! - [`emitter::Emitter`] — trait that language backends implement
//! - [`rust_emitter::RustEmitter`] — Rust language backend

pub mod emitter;
pub mod ir;
pub mod rust_emitter;

use crate::dk::DkKernel;
use crate::mna::MnaSystem;
use crate::parser::Netlist;

use emitter::Emitter;
use ir::CircuitIR;
use rust_emitter::RustEmitter;

/// Configuration for code generation
#[derive(Debug, Clone)]
pub struct CodegenConfig {
    /// Circuit name for generated code
    pub circuit_name: String,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Maximum iterations for Newton-Raphson
    pub max_iterations: usize,
    /// Convergence tolerance
    pub tolerance: f64,
    /// Input resistance (Thevenin equivalent)
    pub input_resistance: f64,
    /// Input node index
    pub input_node: usize,
    /// Output node indices (one per output channel)
    pub output_nodes: Vec<usize>,
    /// Oversampling factor (1, 2, or 4). Default 1 (no oversampling).
    /// Factor > 1 reduces aliasing from nonlinearities.
    pub oversampling_factor: usize,
    /// Output scale factors applied after DC blocking (one per output, default [1.0])
    pub output_scales: Vec<f64>,
    /// Include DC operating point in generated code
    pub include_dc_op: bool,
    /// Maximum NR iterations for DC operating point solver
    pub dc_op_max_iterations: usize,
    /// Convergence tolerance for DC operating point solver
    pub dc_op_tolerance: f64,
    /// Include DC blocking filter on outputs (default true).
    /// Set to false for circuits with output coupling caps or when the downstream
    /// pipeline handles DC offset. Removes the 5Hz HPF and its settling time.
    pub dc_block: bool,
    /// Number of silent samples to process after pot-triggered matrix rebuild.
    /// Settles the NR to the new nonlinear DC operating point. Default 64.
    /// Set to 0 for zero-latency pot changes (may glitch on large pot swings).
    pub pot_settle_samples: usize,
}

impl Default for CodegenConfig {
    fn default() -> Self {
        Self {
            circuit_name: "unnamed_circuit".to_string(),
            sample_rate: 44100.0,
            max_iterations: 100,
            tolerance: 1e-9,
            input_resistance: 1.0, // 1Ω default (near-ideal voltage source)
            input_node: 0,
            output_nodes: vec![0],
            oversampling_factor: 1,
            output_scales: vec![1.0],
            include_dc_op: true,
            dc_op_max_iterations: 200,
            dc_op_tolerance: 1e-9,
            dc_block: true,
            pot_settle_samples: 64,
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
    /// Invalid configuration parameter
    InvalidConfig(String),
    /// Template rendering error
    TemplateError(String),
}

impl std::fmt::Display for CodegenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (label, msg) = match self {
            CodegenError::InvalidKernel(s) => ("Invalid kernel", s.as_str()),
            CodegenError::UnsupportedTopology(s) => ("Unsupported topology", s.as_str()),
            CodegenError::InvalidDevice(s) => ("Invalid device", s.as_str()),
            CodegenError::InvalidConfig(s) => ("Invalid config", s.as_str()),
            CodegenError::TemplateError(s) => ("Template error", s.as_str()),
        };
        write!(f, "{label}: {msg}")
    }
}

impl std::error::Error for CodegenError {}

/// Generated circuit solver code.
#[derive(Debug, Clone)]
pub struct GeneratedCode {
    /// The generated Rust source code
    pub code: String,
    /// Number of circuit nodes (excluding ground)
    pub n: usize,
    /// Total nonlinear dimension (sum of device dimensions)
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

    /// Generate a complete circuit solver module.
    ///
    /// # Arguments
    /// * `kernel` - The compiled DK kernel
    /// * `mna` - MNA system for node mapping
    /// * `netlist` - Original netlist for extracting component values
    ///
    /// # Returns
    /// Generated Rust source code as a string
    pub fn generate(
        &self,
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
    ) -> Result<GeneratedCode, CodegenError> {
        // Validate config
        match self.config.oversampling_factor {
            1 | 2 | 4 => {}
            f => {
                return Err(CodegenError::InvalidConfig(format!(
                    "oversampling_factor must be 1, 2, or 4, got {f}"
                )));
            }
        }
        if self.config.input_resistance <= 0.0 || !self.config.input_resistance.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "input_resistance must be positive finite, got {}",
                self.config.input_resistance
            )));
        }
        if self.config.input_node >= kernel.n {
            return Err(CodegenError::InvalidConfig(format!(
                "input_node {} >= N={}",
                self.config.input_node, kernel.n
            )));
        }
        if self.config.output_nodes.is_empty() {
            return Err(CodegenError::InvalidConfig(
                "output_nodes must not be empty".to_string(),
            ));
        }
        for (i, &node) in self.config.output_nodes.iter().enumerate() {
            // Validate against n_nodes (original circuit nodes), not n_aug
            let n_nodes = kernel.n_nodes;
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (original circuit node count)",
                    i, node, n_nodes
                )));
            }
        }
        if self.config.output_scales.len() != self.config.output_nodes.len() {
            return Err(CodegenError::InvalidConfig(format!(
                "output_scales length ({}) must match output_nodes length ({})",
                self.config.output_scales.len(),
                self.config.output_nodes.len()
            )));
        }

        let ir = CircuitIR::from_kernel(kernel, mna, netlist, &self.config)?;
        let code = RustEmitter::new()?.emit(&ir)?;

        Ok(GeneratedCode {
            code,
            n: ir.topology.n,
            m: ir.topology.m,
        })
    }

    /// Generate code using the NodalSolver path (full N×N NR per sample).
    ///
    /// This bypasses the DkKernel entirely — no S=A⁻¹ precomputation, no K matrix.
    /// The generated code does LU factorization per NR iteration, which handles
    /// any circuit topology including transformer-coupled NFB with large inductors.
    ///
    /// # Errors
    /// Returns `CodegenError` if input/output node validation fails or if code
    /// generation encounters an error.
    pub fn generate_nodal(
        &self,
        mna: &MnaSystem,
        netlist: &Netlist,
    ) -> Result<GeneratedCode, CodegenError> {
        // Validate config (same checks as generate, but against MNA dimensions)
        match self.config.oversampling_factor {
            1 | 2 | 4 => {}
            f => {
                return Err(CodegenError::InvalidConfig(format!(
                    "oversampling_factor must be 1, 2, or 4, got {f}"
                )));
            }
        }
        if self.config.input_resistance <= 0.0 || !self.config.input_resistance.is_finite() {
            return Err(CodegenError::InvalidConfig(format!(
                "input_resistance must be positive finite, got {}",
                self.config.input_resistance
            )));
        }
        if self.config.input_node >= mna.n {
            return Err(CodegenError::InvalidConfig(format!(
                "input_node {} >= n_nodes={}",
                self.config.input_node, mna.n
            )));
        }
        if self.config.output_nodes.is_empty() {
            return Err(CodegenError::InvalidConfig(
                "output_nodes must not be empty".to_string(),
            ));
        }
        for (i, &node) in self.config.output_nodes.iter().enumerate() {
            if node >= mna.n {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={}",
                    i, node, mna.n
                )));
            }
        }
        if self.config.output_scales.len() != self.config.output_nodes.len() {
            return Err(CodegenError::InvalidConfig(format!(
                "output_scales length ({}) must match output_nodes length ({})",
                self.config.output_scales.len(),
                self.config.output_nodes.len()
            )));
        }

        let ir = CircuitIR::from_mna(mna, netlist, &self.config)?;
        let code = RustEmitter::new()?.emit(&ir)?;

        Ok(GeneratedCode {
            code,
            n: ir.topology.n,
            m: ir.topology.m,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_codegen_config_default() {
        let config = CodegenConfig::default();
        assert_eq!(config.sample_rate, 44100.0);
        assert_eq!(config.max_iterations, 100);
        assert!(config.tolerance > 0.0);
    }
}
