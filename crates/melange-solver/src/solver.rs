//! Backward-compatibility re-exports from the old solver module.
//!
//! The runtime solvers (CircuitSolver, NodalSolver, DeviceEntry) have been removed.
//! All circuit processing now goes through the codegen pipeline.
//!
//! LinearSolver and SolverError are re-exported from `linear_solver`.

pub use crate::linear_solver::{LinearSolver, SolverError};
