//! Build script: capture the short git commit at melange build time so the
//! generated `circuit.rs` provenance header can record which melange produced
//! it (`// melange: <version> (<commit>)`).
//!
//! Local builds between tags are normal, so the header carries both the crate
//! version (`CARGO_PKG_VERSION`, available without this script) and the commit
//! captured here. Degrades gracefully: if `git` is unavailable, or this is a
//! packaged crate with no `.git`, `MELANGE_GIT_COMMIT` is simply left unset and
//! the emitter falls back to `(unknown)` via `option_env!`.

use std::path::Path;
use std::process::Command;

fn main() {
    // Never force a spurious rebuild: only this script itself always re-runs.
    println!("cargo:rerun-if-changed=build.rs");

    // The workspace `.git` lives two levels up from this crate root.
    let git_dir = Path::new("../../.git");
    if git_dir.exists() {
        // Re-capture the commit when HEAD moves or the tree is re-checked-out.
        // (Guarded on `.git` existing so a packaged crate — where these paths
        // are absent — does not get treated as perpetually dirty by cargo.)
        println!("cargo:rerun-if-changed=../../.git/HEAD");
    }

    if let Some(commit) = short_commit() {
        println!("cargo:rustc-env=MELANGE_GIT_COMMIT={commit}");
    }
    // No `else`: leaving the var unset is the documented graceful-degradation
    // path (`option_env!("MELANGE_GIT_COMMIT")` → None → "unknown").
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
