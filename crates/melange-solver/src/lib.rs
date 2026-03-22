// Matrix math code uses index loops for clarity; iterator patterns obscure the math.
#![allow(clippy::needless_range_loop)]
// String building for code generation uses format! and to_string extensively.
#![allow(clippy::format_in_format_args)]
#![allow(clippy::unnecessary_to_owned)]
// Circuit solver functions have many parameters by nature.
#![allow(clippy::too_many_arguments)]
// Manual slice copying is clearer for small fixed-size operations.
#![allow(clippy::manual_memcpy)]

//! MNA/DK-method circuit solver engine for real-time audio.
//!
//! Layer 3 of the melange stack. This crate implements:
//! - SPICE netlist parser (subset sufficient for audio circuits)
//! - Modified Nodal Analysis (MNA) matrix assembly
//! - Companion models for reactive components (trapezoidal rule)
//! - DK kernel extraction (reduce N-node system to M-nonlinearity kernel)
//! - Newton-Raphson solver for nonlinear systems
//!
//! # Example Usage
//!
//! ```
//! use melange_solver::parser::Netlist;
//! use melange_solver::mna::MnaSystem;
//! use melange_solver::dk::DkKernel;
//!
//! // Parse a SPICE netlist
//! let spice = r#"RC Circuit
//! R1 in out 1k
//! C1 out 0 1u
//! "#;
//! let netlist = Netlist::parse(spice).unwrap();
//!
//! // Build MNA system
//! let mna = MnaSystem::from_netlist(&netlist).unwrap();
//!
//! // Create DK kernel at sample rate
//! let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();
//! ```
//!
//! # Real-Time Safety
//!
//! The [`CircuitSolver`](solver::CircuitSolver) is designed for real-time audio:
//! - All buffers pre-allocated at construction
//! - No heap allocation in `process_sample()`
//! - Zero `unsafe` code
//!
//! # Limitations
//!
//! See [`docs/limitations.md`](../../docs/limitations.md) for known limitations.

pub mod codegen;
pub mod dc_op;
pub mod device_types;
pub mod dk;
pub mod mna;
pub mod parser;
pub mod solver;

#[cfg(test)]
mod dk_math_verification;

// Explicit re-exports of commonly used types (avoid glob pollution).
#[cfg(feature = "codegen")]
pub use codegen::emitter::Emitter;
#[cfg(feature = "codegen")]
pub use codegen::ir::CircuitIR;
pub use codegen::CodegenError;
#[cfg(feature = "codegen")]
pub use codegen::{CodeGenerator, CodegenConfig, GeneratedCode};
pub use device_types::{
    BjtParams, DeviceParams, DeviceSlot, DeviceType, DiodeParams, JfetParams, MosfetParams,
    TubeParams, VcaParams,
};
pub use dk::{DkError, DkKernel};
pub use mna::{MnaError, MnaSystem};
pub use parser::{Netlist, ParseError};
pub use solver::{CircuitSolver, LinearSolver, NodalSolver, SolverError};
