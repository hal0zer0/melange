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

pub mod ir;
pub mod emitter;
pub mod rust_emitter;

use crate::dk::DkKernel;
use crate::mna::MnaSystem;
use crate::parser::Netlist;

use ir::CircuitIR;
use emitter::Emitter;
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
    /// Output node index
    pub output_node: usize,
    /// Include DC operating point in generated code
    pub include_dc_op: bool,
}

impl Default for CodegenConfig {
    fn default() -> Self {
        Self {
            circuit_name: "unnamed_circuit".to_string(),
            sample_rate: 44100.0,
            max_iterations: 20,
            tolerance: 1e-9,
            input_resistance: 1.0, // 1Ω default (near-ideal voltage source)
            input_node: 0,
            output_node: 0,
            include_dc_op: true,
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
        match self {
            CodegenError::InvalidKernel(s) => write!(f, "Invalid kernel: {}", s),
            CodegenError::UnsupportedTopology(s) => write!(f, "Unsupported topology: {}", s),
            CodegenError::InvalidDevice(s) => write!(f, "Invalid device: {}", s),
            CodegenError::InvalidConfig(s) => write!(f, "Invalid config: {}", s),
            CodegenError::TemplateError(s) => write!(f, "Template error: {}", s),
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
        // Step 1: Build language-agnostic IR
        let ir = CircuitIR::from_kernel(kernel, mna, netlist, &self.config)?;

        // Step 2: Emit Rust code from IR
        let emitter = RustEmitter::new();
        let code = emitter.emit(&ir)?;

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
        assert_eq!(config.max_iterations, 20);
        assert!(config.tolerance > 0.0);
    }
}
