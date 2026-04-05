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
//! Layer 3 of the melange stack — the core engine. This crate implements the full pipeline
//! from SPICE netlist to real-time DSP code.
//!
//! # Pipeline
//!
//! ```text
//! SPICE Netlist → Parser → MNA System → DK Kernel → CircuitIR → Emitter → Source Code
//! ```
//!
//! # Key Types
//!
//! - [`Netlist`](parser::Netlist) — parsed SPICE netlist (elements, models, directives)
//! - [`MnaSystem`](mna::MnaSystem) — Modified Nodal Analysis matrices (G, C, N_v, N_i)
//! - [`DkKernel`](dk::DkKernel) — reduced M×M kernel for fast real-time solving
//! - [`LinearSolver`](linear_solver::LinearSolver) — runtime linear solver (M=0 only)
//! - [`CircuitIR`] — language-agnostic intermediate representation for code generation
//! - [`Emitter`] — trait for generating code in different languages (Rust implemented)
//!
//! # Example: Parse and Solve
//!
//! ```
//! use melange_solver::parser::Netlist;
//! use melange_solver::mna::MnaSystem;
//! use melange_solver::dk::DkKernel;
//!
//! let spice = r#"RC Circuit
//! R1 in out 1k
//! C1 out 0 1u
//! "#;
//! let netlist = Netlist::parse(spice).unwrap();
//! let mna = MnaSystem::from_netlist(&netlist).unwrap();
//! let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();
//! ```
//!
//! # Error Handling
//!
//! All public functions return `Result<T, E>` with specific error types:
//! - [`ParseError`] — netlist syntax errors
//! - [`MnaError`] — invalid circuit topology (floating nodes, singular matrix)
//! - [`DkError`] — kernel build failure (M > MAX_M, singular matrix)
//! - [`SolverError`] — solver construction issues
//! - [`CodegenError`] — code generation configuration errors (requires `codegen` feature)
//!
//! All error types are `#[non_exhaustive]` — new variants may be added in minor versions.
//!
//! # Features
//!
//! - `codegen` (default) — enables code generation pipeline ([`CircuitIR`], [`CodeGenerator`],
//!   [`Emitter`]). Adds `tera` template engine dependency. Disable for runtime-only usage.

pub mod codegen;
pub mod dc_op;
pub mod device_types;
pub mod dk;
pub mod linear_solver;
pub mod lu;
pub mod mna;
pub mod parser;

// Keep the old solver module for backward compatibility during transition.
// It re-exports LinearSolver and SolverError from linear_solver.
pub mod solver;

#[cfg(test)]
mod dk_math_verification;

// Explicit re-exports of commonly used types (avoid glob pollution).
#[cfg(feature = "codegen")]
pub use codegen::emitter::Emitter;
#[cfg(feature = "codegen")]
pub use codegen::ir::CircuitIR;
#[cfg(feature = "codegen")]
pub use codegen::CodegenError;
#[cfg(feature = "codegen")]
pub use codegen::{CodeGenerator, CodegenConfig, GeneratedCode};
pub use device_types::{
    BjtParams, DeviceParams, DeviceSlot, DeviceType, DiodeParams, JfetParams, MosfetParams,
    TubeParams, VcaParams,
};
pub use dk::{DkError, DkKernel, MAX_M};
pub use linear_solver::{LinearSolver, SolverError};
pub use mna::{MnaError, MnaSystem};
pub use parser::{Netlist, ParseError};
