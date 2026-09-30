//! Build script: stamp `MELANGE_GIT_COMMIT` = `<short commit>[-dirty]`, read
//! as `build_identity::GIT_COMMIT` by the generated `circuit.rs` provenance
//! header and by `melange --version` (one definition, `build_git.rs`).
//!
//! It re-runs when anything that goes into the binaries changes (every linked
//! crate's source and manifest, the templates, the lock file) or HEAD moves,
//! so the `-dirty` marker is current whichever crate was edited. Degrades
//! gracefully: without git, or in a packaged crate with no `.git`, the variable
//! is left unset and `GIT_COMMIT` reads "unknown".

use std::path::Path;

#[path = "build_git.rs"]
mod build_git;

fn main() {
    let workspace = Path::new("../..");
    if !workspace.join(".git").exists() {
        println!("cargo:rerun-if-changed=build.rs");
        return;
    }
    for path in build_git::WATCHED {
        println!("cargo:rerun-if-changed=../../{path}");
    }
    // On a branch, committing updates .git/logs/HEAD, not .git/HEAD; a
    // detached/tag checkout updates .git/HEAD. Watch both.
    println!("cargo:rerun-if-changed=../../.git/HEAD");
    println!("cargo:rerun-if-changed=../../.git/logs/HEAD");
    if let Some(id) = build_git::identity(workspace) {
        println!("cargo:rustc-env=MELANGE_GIT_COMMIT={id}");
    }
}
