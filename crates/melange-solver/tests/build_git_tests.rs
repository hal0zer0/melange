//! The build stamp's `-dirty` marker, from the one definition the generated
//! header and `melange --version` share (`build_git.rs`).
//!
//! `melange --version` once read "a6c624a" on a binary built from a6c624a plus
//! uncommitted solver edits, while the generated header said "a6c624a-dirty":
//! the CLI had its own build script, which only re-ran on edits to the CLI.
//! Now there is one stamp, and the build re-runs on any source that goes into
//! the binaries.

#[path = "../build_git.rs"]
mod build_git;

use std::path::Path;
use std::process::Command;

fn git(dir: &Path, args: &[&str]) {
    let ok = Command::new("git")
        .current_dir(dir)
        .args([
            "-c",
            "user.name=test",
            "-c",
            "user.email=test@example.invalid",
        ])
        .args(args)
        .status()
        .expect("git")
        .success();
    assert!(ok, "git {args:?}");
}

/// A throwaway repository: a tracked file under a watched path, and one
/// outside every watched path.
#[test]
fn a_tracked_edit_to_a_watched_source_reads_dirty() {
    let dir = std::env::temp_dir().join(format!("melange_build_git_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let src = dir.join("crates/melange-devices/src");
    std::fs::create_dir_all(&src).unwrap();
    std::fs::create_dir_all(dir.join("docs")).unwrap();
    std::fs::write(src.join("lib.rs"), "// v1\n").unwrap();
    std::fs::write(dir.join("docs/notes.md"), "v1\n").unwrap();
    git(&dir, &["init", "-q"]);
    git(&dir, &["add", "."]);
    git(&dir, &["commit", "-q", "-m", "init"]);

    let clean = build_git::identity(&dir).expect("a commit");
    assert!(!clean.ends_with("-dirty") && !clean.is_empty(), "{clean}");

    // A source edit in another crate than the one being built.
    std::fs::write(src.join("lib.rs"), "// v2\n").unwrap();
    assert_eq!(build_git::identity(&dir).unwrap(), format!("{clean}-dirty"));
    git(&dir, &["checkout", "-q", "--", "."]);
    assert_eq!(build_git::identity(&dir).unwrap(), clean);

    // Untracked files and edits outside the binaries' sources do not count.
    std::fs::write(src.join("scratch.rs"), "// untracked\n").unwrap();
    std::fs::write(dir.join("docs/notes.md"), "v2\n").unwrap();
    assert_eq!(build_git::identity(&dir).unwrap(), clean);

    let _ = std::fs::remove_dir_all(&dir);
}

/// The `melange-*` dependencies listed under `[dependencies]` in a manifest.
fn melange_deps(manifest: &str) -> Vec<String> {
    manifest
        .lines()
        .skip_while(|l| l.trim() != "[dependencies]")
        .skip(1)
        .take_while(|l| !l.trim_start().starts_with('['))
        .filter_map(|l| l.split('=').next().map(str::trim))
        .filter(|name| name.starts_with("melange-"))
        .map(str::to_string)
        .collect()
}

/// Every melange crate the CLI links, directly or through another melange
/// crate, is watched (source and manifest), so an edit to any of them
/// re-runs the stamp.
#[test]
fn every_crate_the_cli_links_is_watched() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest = std::fs::read_to_string(root.join("tools/melange-cli/Cargo.toml")).unwrap();
    let mut deps: Vec<String> = Vec::new();
    let mut queue = melange_deps(&manifest);
    while let Some(dep) = queue.pop() {
        if deps.contains(&dep) {
            continue;
        }
        let m = std::fs::read_to_string(root.join(format!("crates/{dep}/Cargo.toml")))
            .unwrap_or_else(|e| panic!("crates/{dep}/Cargo.toml: {e}"));
        queue.extend(melange_deps(&m));
        deps.push(dep);
    }
    // solver, validate, devices, primitives: the CLI links all four.
    assert!(deps.len() >= 4, "{deps:?}");
    for dep in deps {
        for part in ["src", "Cargo.toml"] {
            let path = format!("crates/{dep}/{part}");
            assert!(root.join(&path).exists(), "{path}");
            assert!(
                build_git::WATCHED.contains(&path.as_str()),
                "{path} is not watched"
            );
        }
    }
    for path in [
        "tools/melange-cli/src",
        "crates/melange-solver/templates",
        "Cargo.lock",
    ] {
        assert!(build_git::WATCHED.contains(&path), "{path} is not watched");
    }
}
