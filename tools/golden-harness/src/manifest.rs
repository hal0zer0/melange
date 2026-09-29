//! Lenient manifest loading.
//!
//! The real manifest is produced by another agent; field names may vary
//! slightly, and entries may carry extra fields. We accept:
//! - a top-level JSON array, or an object with a `circuits` / `entries` key
//! - name under `plugin` | `name` | `circuit` | `id`
//! - netlist path under `cir` | `cir_path` | `netlist` | `path`
//! - compile command under `compile_cmd` | `compile` | `cmd`
//! - `has_pots` | `pots`, `has_noise` | `noise` (booleans)
//! - `input_level` | `level` (volts, default 0.1)
//! - `expected_output_clamp`: `{ "reason": "...", "programs": ["sweep", ...] }`,
//!   declaring that the generated output clamp engages on purpose (`programs`
//!   absent = every program). The reason is required. Without it, a render
//!   whose output clamp engages fails capture.
//!
//! Unknown fields are ignored.

use std::path::Path;

#[derive(Debug, Clone)]
pub struct Entry {
    pub plugin: String,
    pub cir: String,
    pub compile_cmd: Option<String>,
    pub has_pots: bool,
    pub has_noise: bool,
    pub input_level: f64,
    /// Declared output-clamp engagement: `(reason, programs)`; an empty
    /// program list means every program.
    pub expected_output_clamp: Option<(String, Vec<String>)>,
}

impl Entry {
    /// The declared reason when `program`'s output clamp is expected to engage.
    pub fn expected_clamp_reason(&self, program: &str) -> Option<&str> {
        let (reason, programs) = self.expected_output_clamp.as_ref()?;
        (programs.is_empty() || programs.iter().any(|p| p == program)).then_some(reason.as_str())
    }
}

fn get_str(v: &serde_json::Value, keys: &[&str]) -> Option<String> {
    keys.iter()
        .find_map(|k| v.get(k).and_then(|x| x.as_str()).map(|s| s.to_string()))
}

fn get_bool(v: &serde_json::Value, keys: &[&str], default: bool) -> bool {
    keys.iter()
        .find_map(|k| v.get(k).and_then(|x| x.as_bool()))
        .unwrap_or(default)
}

fn get_f64(v: &serde_json::Value, keys: &[&str], default: f64) -> f64 {
    keys.iter()
        .find_map(|k| v.get(k).and_then(|x| x.as_f64()))
        .unwrap_or(default)
}

fn expand_tilde(p: &str) -> String {
    if let Some(rest) = p.strip_prefix("~/") {
        if let Ok(home) = std::env::var("HOME") {
            return format!("{home}/{rest}");
        }
    }
    p.to_string()
}

pub fn load(path: &Path) -> Result<Vec<Entry>, String> {
    let txt = std::fs::read_to_string(path)
        .map_err(|e| format!("read manifest {}: {e}", path.display()))?;
    let v: serde_json::Value =
        serde_json::from_str(&txt).map_err(|e| format!("parse manifest JSON: {e}"))?;
    let arr = if let Some(a) = v.as_array() {
        a.clone()
    } else if let Some(a) = v
        .get("circuits")
        .or_else(|| v.get("entries"))
        .and_then(|x| x.as_array())
    {
        a.clone()
    } else {
        return Err("manifest must be a JSON array or an object with a `circuits` array".into());
    };

    let mut out = Vec::new();
    for (i, item) in arr.iter().enumerate() {
        let plugin = get_str(item, &["plugin", "name", "circuit", "id"])
            .ok_or_else(|| format!("manifest entry {i}: no plugin/name field"))?;
        let cir = get_str(item, &["cir", "cir_path", "netlist", "path"])
            .ok_or_else(|| format!("manifest entry {i} ({plugin}): no cir/netlist path field"))?;
        let expected_output_clamp = match item.get("expected_output_clamp") {
            None => None,
            Some(c) => {
                let reason = c
                    .get("reason")
                    .and_then(|r| r.as_str())
                    .map(str::trim)
                    .filter(|r| !r.is_empty())
                    .ok_or_else(|| {
                        format!(
                            "manifest entry {i} ({plugin}): expected_output_clamp needs a \
                             non-empty \"reason\""
                        )
                    })?
                    .to_string();
                let programs = c
                    .get("programs")
                    .and_then(|p| p.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|x| x.as_str().map(str::to_string))
                            .collect()
                    })
                    .unwrap_or_default();
                Some((reason, programs))
            }
        };
        out.push(Entry {
            plugin,
            cir: expand_tilde(&cir),
            compile_cmd: get_str(item, &["compile_cmd", "compile", "cmd"]),
            has_pots: get_bool(item, &["has_pots", "pots"], false),
            has_noise: get_bool(item, &["has_noise", "noise"], false),
            input_level: get_f64(item, &["input_level", "level"], 0.1),
            expected_output_clamp,
        });
    }
    if out.is_empty() {
        return Err("manifest contains no circuits".into());
    }
    Ok(out)
}

#[cfg(test)]
mod expected_output_clamp_tests {
    use super::*;

    fn load_str(json: &str) -> Result<Vec<Entry>, String> {
        let dir = std::env::temp_dir().join(format!("gh-manifest-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join(format!("m{}.json", json.len()));
        std::fs::write(&p, json).unwrap();
        load(&p)
    }

    #[test]
    fn a_declared_clamp_names_its_programs_and_needs_a_reason() {
        let e = load_str(
            r#"[{"plugin":"a","cir":"a.cir","expected_output_clamp":{"reason":"rail test","programs":["sweep"]}},
                {"plugin":"b","cir":"b.cir","expected_output_clamp":{"reason":"always"}},
                {"plugin":"c","cir":"c.cir"}]"#,
        )
        .unwrap();
        assert_eq!(e[0].expected_clamp_reason("sweep"), Some("rail test"));
        assert_eq!(e[0].expected_clamp_reason("sine1k"), None);
        assert_eq!(e[1].expected_clamp_reason("step"), Some("always"));
        assert_eq!(e[2].expected_clamp_reason("sweep"), None);
        let err =
            load_str(r#"[{"plugin":"d","cir":"d.cir","expected_output_clamp":{"reason":"  "}}]"#)
                .unwrap_err();
        assert!(err.contains("reason"), "{err}");
    }
}
