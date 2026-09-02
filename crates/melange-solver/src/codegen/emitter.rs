//! Emitter trait for language-specific code generation backends.

use super::ir::CircuitIR;
use super::CodegenError;
use super::NodalSubPath;

/// One emitted source file.
///
/// `name` is a bare filename, not a path — the caller decides where files land.
/// A single-file backend emits one of these; C++ will emit a header plus an
/// implementation unit.
#[derive(Debug, Clone)]
pub struct EmitFile {
    pub name: String,
    pub contents: String,
}

/// What a backend produces: the source files, plus what the emitter decided
/// while producing them.
///
/// The metadata travels with the output rather than being recomputed by the
/// pipeline. Recomputing would create a second source of truth that could
/// silently diverge from what the emitter actually did — the pipeline would be
/// reporting what it believes the emitter should have chosen, which is not the
/// same claim.
#[derive(Debug, Clone)]
pub struct EmitOutput {
    /// Emitted files. Never empty; `files[0]` is the primary translation unit.
    pub files: Vec<EmitFile>,
    /// Nodal sub-path taken, when the circuit routed nodal. `None` on DK.
    pub nodal_sub_path: Option<NodalSubPath>,
}

impl EmitOutput {
    /// Single-file convenience constructor.
    pub fn single(name: impl Into<String>, contents: String) -> Self {
        Self {
            files: vec![EmitFile {
                name: name.into(),
                contents,
            }],
            nodal_sub_path: None,
        }
    }

    pub fn with_nodal_sub_path(mut self, sp: Option<NodalSubPath>) -> Self {
        self.nodal_sub_path = sp;
        self
    }

    /// Contents of the primary translation unit.
    ///
    /// Panics only if a backend returned no files, which is a backend bug — the
    /// type's invariant is that `files` is non-empty.
    pub fn primary(&self) -> &str {
        &self
            .files
            .first()
            .expect("emitter returned no files (backend bug)")
            .contents
    }
}

/// A language backend that converts a [`CircuitIR`] into source code.
pub trait Emitter {
    /// Language name (e.g. `"rust"`, `"cpp"`).
    fn language(&self) -> &str;

    /// Emit source from the IR.
    fn emit(&self, ir: &CircuitIR) -> Result<EmitOutput, CodegenError>;
}
