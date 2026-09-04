//! Build script: bake the crate version + short git commit into
//! `MELANGE_VERSION` so `melange --version` prints e.g. `0.1.5 (cd133c6)`
//! instead of a bare `0.1.5`. Three different builds all reporting the same
//! bare version string (a released tag, an unreleased main, a local fix)
//! caused a real cross-repo misdiagnosis; the commit disambiguates them.
//!
//! Mirrors `crates/melange-solver/build.rs` (which stamps the generated
//! `circuit.rs` provenance header). Degrades gracefully: if `git` is
//! unavailable or this is a packaged crate with no `.git`, the commit falls
//! back to `unknown` and the version string is still emitted.

use std::path::Path;
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    // The workspace `.git` lives two levels up from this crate root
    // (tools/melange-cli), same as crates/melange-solver.
    let git_dir = Path::new("../../.git");
    if git_dir.exists() {
        // On a branch, committing updates .git/logs/HEAD, not .git/HEAD; a
        // detached/tag checkout updates .git/HEAD. Watch both so the stamp
        // stays current for local branch builds and CI tag builds alike.
        println!("cargo:rerun-if-changed=../../.git/HEAD");
        println!("cargo:rerun-if-changed=../../.git/logs/HEAD");
    }

    let version = std::env::var("CARGO_PKG_VERSION").unwrap_or_default();
    let commit = short_commit().unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env=MELANGE_VERSION={version} ({commit})");
}

/// `git rev-parse --short HEAD`, or `None` if git or the repo is unavailable.
fn short_commit() -> Option<String> {
    let out = Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let commit = String::from_utf8(out.stdout).ok()?.trim().to_string();
    if commit.is_empty() {
        None
    } else {
        Some(commit)
    }
}
