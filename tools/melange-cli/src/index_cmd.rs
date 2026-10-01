//! `melange index` — generate or check a repository's `circuits-index.json`.
//!
//! Format spec: `docs/CIRCUIT_INDEX.md`. This is one conforming generator;
//! melange-circuits' `tools/circuits_index.py` is another, and the two must
//! produce byte-identical output for the same tree so either one's `--check`
//! accepts the other's file.

use anyhow::{Context, Result};
use serde::Serialize;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

pub const INDEX_FILENAME: &str = "circuits-index.json";

/// Serialised with fields in declaration order: `schema`, then `circuits`.
#[derive(Serialize)]
struct Index {
    schema: u32,
    /// BTreeMap so names come out sorted, which is half of the byte contract.
    circuits: BTreeMap<String, Entry>,
}

#[derive(Serialize, Debug)]
struct Entry {
    path: String,
}

/// Every `.cir` under `root`, keyed by basename without the extension.
///
/// Hidden directories are skipped: `.git` is the obvious one, but so is any
/// editor or tooling directory a repository happens to carry.
fn scan(root: &Path) -> Result<BTreeMap<String, Entry>> {
    let mut out: BTreeMap<String, Entry> = BTreeMap::new();
    let mut seen: BTreeMap<String, PathBuf> = BTreeMap::new();
    let mut stack = vec![root.to_path_buf()];

    while let Some(dir) = stack.pop() {
        let entries = std::fs::read_dir(&dir)
            .with_context(|| format!("Failed to read directory: {}", dir.display()))?;
        for e in entries.flatten() {
            let p = e.path();
            let name = match p.file_name().and_then(|s| s.to_str()) {
                Some(n) => n,
                None => continue,
            };
            if name.starts_with('.') {
                continue;
            }
            if p.is_dir() {
                stack.push(p);
            } else if p.extension().and_then(|s| s.to_str()) == Some("cir") {
                let stem = p
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or_default()
                    .to_string();
                let rel = p
                    .strip_prefix(root)
                    .unwrap_or(&p)
                    .to_string_lossy()
                    .replace('\\', "/");
                // A collision makes the whole index ambiguous, and picking a
                // winner would resolve the same name to different circuits on
                // different machines depending on directory order. Refuse.
                if let Some(first) = seen.get(&stem) {
                    anyhow::bail!(
                        "Duplicate circuit name '{stem}':\n  {}\n  {}\n\n\
                         Names must be unique across the repository — they are what \
                         `source:name` resolves. Rename one.",
                        first.display(),
                        p.display()
                    );
                }
                seen.insert(stem.clone(), p.clone());
                out.insert(stem, Entry { path: rel });
            }
        }
    }
    Ok(out)
}

/// Exactly the bytes the spec calls for: two-space indent, trailing newline.
fn render(circuits: BTreeMap<String, Entry>) -> Result<String> {
    let idx = Index {
        schema: 1,
        circuits,
    };
    let mut s = serde_json::to_string_pretty(&idx).context("Failed to serialise index")?;
    s.push('\n');
    Ok(s)
}

/// `melange index [DIR] [--check]`.
///
/// Without `--check`, writes the index. With it, writes nothing and fails if
/// the file is missing or does not match the tree — the CI form. A silently
/// stale index is worse than none, because consumers trust it.
pub fn run(dir: &Path, check: bool) -> Result<()> {
    if !dir.is_dir() {
        anyhow::bail!("Not a directory: {}", dir.display());
    }
    let circuits = scan(dir)?;
    let n = circuits.len();
    let rendered = render(circuits)?;
    let out = dir.join(INDEX_FILENAME);

    if check {
        let existing = std::fs::read_to_string(&out).with_context(|| {
            format!(
                "{} is missing. Run `melange index {}` and commit it.",
                out.display(),
                dir.display()
            )
        })?;
        // Compare MEANING, not bytes. The spec says readers must ignore keys
        // they do not recognise, so a repository is free to enrich its entries
        // — melange-circuits publishes `tier` and `category`. A byte compare
        // would hold that against them and go permanently red, which would
        // make the extensibility rule a lie. Checked against their live index:
        // same 43 names and paths, extras they emit and melange does not.
        let have: serde_json::Value = serde_json::from_str(&existing)
            .with_context(|| format!("{} is not valid JSON", out.display()))?;
        let want: serde_json::Value = serde_json::from_str(&rendered)?;
        let paths = |v: &serde_json::Value| -> BTreeMap<String, String> {
            v.get("circuits")
                .and_then(|c| c.as_object())
                .map(|m| {
                    m.iter()
                        .filter_map(|(k, e)| {
                            Some((k.clone(), e.get("path")?.as_str()?.to_string()))
                        })
                        .collect()
                })
                .unwrap_or_default()
        };
        let (a, b) = (paths(&have), paths(&want));
        if have.get("schema") != want.get("schema") || a != b {
            let missing: Vec<&String> = b.keys().filter(|k| !a.contains_key(*k)).collect();
            let extra: Vec<&String> = a.keys().filter(|k| !b.contains_key(*k)).collect();
            let moved: Vec<String> = b
                .iter()
                .filter(|(k, v)| a.get(*k).is_some_and(|o| o != *v))
                .map(|(k, v)| format!("{k}: {} -> {v}", a[k]))
                .collect();
            let mut why = Vec::new();
            if !missing.is_empty() {
                why.push(format!("not in the index: {missing:?}"));
            }
            if !extra.is_empty() {
                why.push(format!("in the index but not on disk: {extra:?}"));
            }
            if !moved.is_empty() {
                why.push(format!("moved: {}", moved.join(", ")));
            }
            anyhow::bail!(
                "{} does not match the tree ({n} circuit(s) found).\n  {}\n\
                 Run `melange index {}` and commit the result.",
                out.display(),
                why.join("\n  "),
                dir.display()
            );
        }
        println!("{}: up to date ({n} circuits)", out.display());
        return Ok(());
    }

    std::fs::write(&out, &rendered)
        .with_context(|| format!("Failed to write {}", out.display()))?;
    println!("{}: wrote {n} circuits", out.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree(files: &[&str]) -> tempfile::TempDir {
        let d = tempfile::tempdir().unwrap();
        for f in files {
            let p = d.path().join(f);
            std::fs::create_dir_all(p.parent().unwrap()).unwrap();
            std::fs::write(&p, "* test\n").unwrap();
        }
        d
    }

    #[test]
    fn flat_and_nested_both_index() {
        let d = tree(&["rc.cir", "fuzz/fuzz-pedal.cir"]);
        let s = render(scan(d.path()).unwrap()).unwrap();
        assert!(
            s.contains("\"rc\": {\n      \"path\": \"rc.cir\"\n    }"),
            "{s}"
        );
        assert!(s.contains("\"path\": \"fuzz/fuzz-pedal.cir\""), "{s}");
    }

    /// The byte contract: sorted names, 2-space indent, trailing newline,
    /// `schema` before `circuits`.
    #[test]
    fn byte_format_is_the_documented_one() {
        let d = tree(&["b.cir", "a.cir"]);
        let s = render(scan(d.path()).unwrap()).unwrap();
        assert_eq!(
            s,
            "{\n  \"schema\": 1,\n  \"circuits\": {\n    \"a\": {\n      \"path\": \"a.cir\"\n    },\n    \"b\": {\n      \"path\": \"b.cir\"\n    }\n  }\n}\n"
        );
    }

    #[test]
    fn hidden_directories_are_skipped() {
        let d = tree(&["ok.cir", ".git/objects/sneaky.cir"]);
        let idx = scan(d.path()).unwrap();
        assert!(idx.contains_key("ok"));
        assert!(!idx.contains_key("sneaky"));
    }

    /// Ambiguity must fail, not pick a winner: directory order differs between
    /// machines, so a "winner" resolves the same name to different circuits.
    #[test]
    fn duplicate_names_refuse() {
        let d = tree(&["a/dup.cir", "b/dup.cir"]);
        let e = scan(d.path()).unwrap_err().to_string();
        assert!(e.contains("Duplicate circuit name 'dup'"), "{e}");
    }

    /// An index carrying extras melange does not emit (melange-circuits
    /// publishes `tier`/`category`) must still pass --check. Byte-comparing
    /// would contradict the spec's own "ignore unknown keys" rule.
    #[test]
    fn check_accepts_an_enriched_index() {
        let d = tree(&["testing/filters/passive-eq1a.cir"]);
        std::fs::write(
            d.path().join(INDEX_FILENAME),
            "{\n  \"schema\": 1,\n  \"circuits\": {\n    \"passive-eq1a\": {\n      \"path\": \"testing/filters/passive-eq1a.cir\",\n      \"tier\": \"testing\",\n      \"category\": \"filters\"\n    }\n  }\n}\n",
        )
        .unwrap();
        run(d.path(), true).expect("enriched index must pass --check");
    }

    #[test]
    fn check_detects_staleness() {
        let d = tree(&["a.cir"]);
        run(d.path(), false).unwrap();
        run(d.path(), true).unwrap();
        std::fs::write(d.path().join("b.cir"), "* test\n").unwrap();
        let e = run(d.path(), true).unwrap_err().to_string();
        assert!(e.contains("does not match the tree"), "{e}");
    }
}
