//! Emitter trait for language-specific code generation backends.

use super::ir::CircuitIR;
use super::CodegenError;

/// A language backend that converts a [`CircuitIR`] into source code.
pub trait Emitter {
    /// Language name (e.g. `"rust"`, `"cpp"`).
    fn language(&self) -> &str;

    /// Emit source code from the IR.
    fn emit(&self, ir: &CircuitIR) -> Result<String, CodegenError>;
}
