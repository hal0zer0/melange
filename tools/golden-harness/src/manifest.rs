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
        out.push(Entry {
            plugin,
            cir: expand_tilde(&cir),
            compile_cmd: get_str(item, &["compile_cmd", "compile", "cmd"]),
            has_pots: get_bool(item, &["has_pots", "pots"], false),
            has_noise: get_bool(item, &["has_noise", "noise"], false),
            input_level: get_f64(item, &["input_level", "level"], 0.1),
        });
    }
    if out.is_empty() {
        return Err("manifest contains no circuits".into());
    }
    Ok(out)
}
