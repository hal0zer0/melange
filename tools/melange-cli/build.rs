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
    // Re-run when this crate's own sources change so the `-dirty` marker below
    // reflects local edits to it. This is best-effort: the marker cannot see
    // edits in OTHER crates that link in (the build script does not re-run for
    // them), which is exactly why the EXACT build identity is the runtime exe
    // hash (`melange_solver::build_identity`), and the version+commit(+dirty)
    // are only a labelled pointer to where to look.
    println!("cargo:rerun-if-changed=src");

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
    let (commit, dirty) = if git_dir.exists() {
        (
            short_commit().unwrap_or_else(|| "unknown".to_string()),
            dirty_marker(),
        )
    } else {
        ("unknown".to_string(), "")
    };
    println!("cargo:rustc-env=MELANGE_VERSION={version} ({commit}{dirty})");
}

/// `"-dirty"` when the working tree has uncommitted TRACKED changes vs HEAD,
/// else `""`. Uses `git diff --quiet HEAD` (exit 1 == differences), which
/// ignores untracked files so a stray scratch file does not read as dirty.
/// Best-effort by nature — the exact identity is the runtime exe hash. Any git
/// error degrades to `""`.
fn dirty_marker() -> &'static str {
    match Command::new("git")
        .args(["diff", "--quiet", "HEAD"])
        .status()
    {
        Ok(s) if s.success() => "",              // clean: no tracked diff vs HEAD
        Ok(s) if s.code() == Some(1) => "-dirty", // tracked changes present
        _ => "",                                  // git error / no HEAD → graceful
    }
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
