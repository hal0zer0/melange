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
    // Re-run when this crate's own sources change so the `-dirty` marker reflects
    // local edits to it. Best-effort (cannot see edits in other crates that link
    // in); the EXACT build identity is the runtime exe hash (`build_identity`).
    println!("cargo:rerun-if-changed=src");

    // The workspace `.git` lives two levels up from this crate root.
    let git_dir = Path::new("../../.git");
    if git_dir.exists() {
        // Re-capture the commit when HEAD moves. On a branch, committing updates
        // the reflog (`.git/logs/HEAD`), NOT `.git/HEAD` (which just holds
        // `ref: refs/heads/<branch>`); a detached/tag checkout updates
        // `.git/HEAD` directly. Watch both so the stamp stays current for local
        // branch builds AND CI tag builds. (Guarded on `.git` existing so a
        // packaged crate — where these paths are absent — does not rebuild
        // spuriously.)
        println!("cargo:rerun-if-changed=../../.git/HEAD");
        println!("cargo:rerun-if-changed=../../.git/logs/HEAD");
    }

    if let Some(commit) = short_commit() {
        // Append a best-effort `-dirty` marker (tracked changes vs HEAD) so the
        // generated header's commit is not silently that of a clean tree when it
        // was not. The exe hash in the provenance JSON is the exact identity.
        let dirty = if git_dir.exists() { dirty_marker() } else { "" };
        println!("cargo:rustc-env=MELANGE_GIT_COMMIT={commit}{dirty}");
    }
    // No `else`: leaving the var unset is the documented graceful-degradation
    // path (`option_env!("MELANGE_GIT_COMMIT")` → None → "unknown").
}

/// `"-dirty"` when the working tree has uncommitted TRACKED changes vs HEAD,
/// else `""`. Uses `git diff --quiet HEAD` (exit 1 == differences), ignoring
/// untracked files. Best-effort — the exact identity is the runtime exe hash.
fn dirty_marker() -> &'static str {
    match Command::new("git")
        .args(["diff", "--quiet", "HEAD"])
        .status()
    {
        Ok(s) if s.success() => "",
        Ok(s) if s.code() == Some(1) => "-dirty",
        _ => "",
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
