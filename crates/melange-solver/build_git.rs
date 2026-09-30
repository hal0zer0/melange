// Git identity for melange's build stamp: `<short commit>[-dirty]`.
//
// One definition, used by `build.rs` (which stamps `MELANGE_GIT_COMMIT`, read
// as `build_identity::GIT_COMMIT` by the generated-code header and by
// `melange --version`) and by `tests/build_git_tests.rs`. The label is a
// best-effort pointer; the exact identity is the runtime exe hash
// (`build_identity`).

use std::path::Path;
use std::process::Command;

/// Workspace paths whose edits change what the melange binaries contain: the
/// source and manifest of every crate `melange-cli` links, the codegen
/// templates, and the lock file. The build script re-runs when any of them
/// changes, and "dirty" means a tracked change vs HEAD in one of them.
pub const WATCHED: &[&str] = &[
    "Cargo.toml",
    "Cargo.lock",
    "crates/melange-primitives/Cargo.toml",
    "crates/melange-primitives/src",
    "crates/melange-devices/Cargo.toml",
    "crates/melange-devices/src",
    "crates/melange-solver/Cargo.toml",
    "crates/melange-solver/build.rs",
    "crates/melange-solver/build_git.rs",
    "crates/melange-solver/src",
    "crates/melange-solver/templates",
    "crates/melange-validate/Cargo.toml",
    "crates/melange-validate/src",
    "tools/melange-cli/Cargo.toml",
    "tools/melange-cli/src",
];

/// `<short commit>`, with `-dirty` when a tracked file under [`WATCHED`]
/// differs from HEAD (untracked files are ignored). `None` without git or a
/// commit.
pub fn identity(workspace: &Path) -> Option<String> {
    let out = Command::new("git")
        .current_dir(workspace)
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()?;
    let commit = String::from_utf8(out.stdout).ok()?.trim().to_string();
    if !out.status.success() || commit.is_empty() {
        return None;
    }
    let dirty = Command::new("git")
        .current_dir(workspace)
        .args(["diff", "--quiet", "HEAD", "--"])
        .args(WATCHED)
        .status()
        .map(|s| s.code() == Some(1))
        .unwrap_or(false);
    Some(if dirty {
        format!("{commit}-dirty")
    } else {
        commit
    })
}
