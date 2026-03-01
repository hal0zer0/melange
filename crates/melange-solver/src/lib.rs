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

pub mod parser;
pub mod mna;
pub mod dk;
pub mod solver;
pub mod codegen;
pub mod dc_op;

#[cfg(test)]
mod dk_math_verification;

pub use parser::*;
pub use mna::*;
pub use dk::*;
pub use solver::*;
pub use codegen::*;
