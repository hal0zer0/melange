//! Backward-compatibility re-exports from the old solver module.
//!
//! The runtime solvers (CircuitSolver, NodalSolver, DeviceEntry) have been removed.
//! All circuit processing now goes through the codegen pipeline.
//!
//! LinearSolver and SolverError are re-exported from `linear_solver`.
//!
//! **Deprecated (0.1.14):** this module and both re-exported types are
//! removed in the next release.
#![allow(deprecated)]

pub use crate::linear_solver::{LinearSolver, SolverError};
