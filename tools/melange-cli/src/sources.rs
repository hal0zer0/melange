//! Friendly sources configuration for melange-cli
//!
//! Manages external circuit repositories that can be referenced
//! using the friendly `source:circuit` syntax.

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// A source's published `circuits-index.json`. Schema 1: only `path` is
/// required per entry; unknown keys (`tier`, `category`, future additions) are
/// ignored by design so the format can grow without breaking older clients.
#[derive(Debug, Deserialize)]
pub struct CircuitIndex {
    #[allow(dead_code)]
    pub schema: u32,
    pub circuits: HashMap<String, CircuitIndexEntry>,
}

#[derive(Debug, Deserialize)]
pub struct CircuitIndexEntry {
    /// Repo-relative path to the `.cir`, relative to the index file.
    pub path: String,
}

/// Names within edit distance 2, closest first, at most three. Cheap enough
/// for an index of this size and it turns a dead end into a next step.
fn near_matches<'a>(want: &str, have: impl Iterator<Item = &'a String>) -> Vec<&'a str> {
    let mut scored: Vec<(usize, &str)> = have
        .filter_map(|h| {
            let d = edit_distance(want, h);
            (d <= 2).then_some((d, h.as_str()))
        })
        .collect();
    scored.sort_by_key(|(d, n)| (*d, *n));
    scored.into_iter().take(3).map(|(_, n)| n).collect()
}

fn edit_distance(a: &str, b: &str) -> usize {
    let (a, b): (Vec<char>, Vec<char>) = (a.chars().collect(), b.chars().collect());
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut cur = vec![0usize; b.len() + 1];
    for i in 1..=a.len() {
        cur[0] = i;
        for j in 1..=b.len() {
            let cost = usize::from(a[i - 1] != b[j - 1]);
            cur[j] = (prev[j] + 1).min(cur[j - 1] + 1).min(prev[j - 1] + cost);
        }
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[b.len()]
}

/// Configuration for a single external circuit source
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SourceConfig {
    /// Base URL for fetching circuits
    pub url: String,
    /// License identifier (SPDX format preferred)
    pub license: Option<String>,
    /// Attribution string for generated code
    pub attribution: Option<String>,
    /// Default subdirectory path (if any)
    pub subdirectory: Option<String>,
}

/// Complete sources configuration
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct SourcesConfig {
    /// Map of source names to their configurations
    pub sources: HashMap<String, SourceConfig>,
    /// Default source for unqualified circuit names
    pub default_source: Option<String>,
}

impl SourcesConfig {
    /// Load configuration from the config directory
    ///
    /// If no config exists, creates default configuration and saves it.
    pub fn load() -> Result<Self> {
        let config_path = Self::config_path()?;

        if config_path.exists() {
            let content = std::fs::read_to_string(&config_path).with_context(|| {
                format!("Failed to read config file: {}", config_path.display())
            })?;
            let config: SourcesConfig = toml::from_str(&content)
                .with_context(|| "Failed to parse config file (invalid TOML)")?;
            Ok(config)
        } else {
            // Create default config
            let config = Self::default_config();
            config.save()?;
            Ok(config)
        }
    }

    /// Save configuration to the config directory
    pub fn save(&self) -> Result<()> {
        let config_path = Self::config_path()?;

        // Ensure parent directory exists
        if let Some(parent) = config_path.parent() {
            std::fs::create_dir_all(parent).with_context(|| {
                format!("Failed to create config directory: {}", parent.display())
            })?;
        }

        let content =
            toml::to_string_pretty(self).with_context(|| "Failed to serialize config to TOML")?;

        std::fs::write(&config_path, content)
            .with_context(|| format!("Failed to write config file: {}", config_path.display()))?;

        Ok(())
    }

    /// Get the path to the config file
    pub fn config_path() -> Result<std::path::PathBuf> {
        dirs::config_dir()
            .map(|p| p.join("melange").join("sources.toml"))
            .ok_or_else(|| anyhow::anyhow!("Cannot find config directory"))
    }

    /// Create the default configuration.
    ///
    /// No external circuit sources are pre-seeded. The repos this used to point
    /// at (`melange-audio/circuits`, `tonestack/tonestack`) do not exist — they
    /// 404'd — and shipping a dead link is worse than shipping none. The
    /// passive-eq demo is available as a builtin (`melange compile
    /// passive-eq1a`); add your own circuit repo with
    /// `melange sources add <name> <url>`. A vetted external example may be
    /// seeded here once one is published.
    fn default_config() -> Self {
        Self {
            sources: HashMap::new(),
            default_source: None,
        }
    }

    /// Add a new source
    pub fn add_source(
        &mut self,
        name: &str,
        url: &str,
        license: Option<&str>,
        attribution: Option<&str>,
    ) {
        self.sources.insert(
            name.to_string(),
            SourceConfig {
                url: url.to_string(),
                license: license.map(|s| s.to_string()),
                attribution: attribution.map(|s| s.to_string()),
                subdirectory: None,
            },
        );
    }

    /// Remove a source
    pub fn remove_source(&mut self, name: &str) -> bool {
        self.sources.remove(name).is_some()
    }

    /// Resolve a circuit from a source to a full URL
    ///
    /// Automatically appends `.cir` extension if not present.
    /// The source's base as a local directory, if that is what it is.
    ///
    /// A source is a *place circuits live*; nothing about that requires HTTP.
    /// `melange index` writes an index into a local directory, so refusing to
    /// read one back would mean the generator's own output is unusable until
    /// it is published somewhere.
    pub fn local_dir(&self, source: &str) -> Option<std::path::PathBuf> {
        let c = self.sources.get(source)?;
        let p = std::path::Path::new(c.url.trim_end_matches('/'));
        p.is_dir().then(|| p.to_path_buf())
    }

    /// Resolve a name inside a local source directory.
    ///
    /// Deliberately the same protocol as the remote path — index first, flat
    /// fallback, and a present-but-missing name is an error with near-matches,
    /// never a silent fall-through. Local and remote sources behaving
    /// differently would make the documented protocol a half-truth.
    pub fn resolve_local(dir: &std::path::Path, circuit: &str) -> Result<std::path::PathBuf> {
        let name = circuit.strip_suffix(".cir").unwrap_or(circuit);
        let index_path = dir.join("circuits-index.json");

        if let Ok(raw) = std::fs::read_to_string(&index_path) {
            let index: CircuitIndex = serde_json::from_str(&raw).with_context(|| {
                format!(
                    "{}: not a valid circuits-index.json (see docs/CIRCUIT_INDEX.md)",
                    index_path.display()
                )
            })?;
            return match index.circuits.get(name) {
                Some(e) => Ok(dir.join(e.path.trim_start_matches('/'))),
                None => {
                    let near = near_matches(name, index.circuits.keys());
                    let hint = if near.is_empty() {
                        format!("{} circuits are indexed there.", index.circuits.len())
                    } else {
                        format!("Did you mean: {}?", near.join(", "))
                    };
                    anyhow::bail!(
                        "'{name}' is not in the index at {}. {hint}",
                        index_path.display()
                    )
                }
            };
        }

        let flat = dir.join(format!("{name}.cir"));
        if flat.is_file() {
            return Ok(flat);
        }
        anyhow::bail!(
            "'{name}' not found in {}. No circuits-index.json there, and no {name}.cir.\n\
             Run `melange index {}` to index that directory.",
            dir.display(),
            dir.display()
        )
    }

    /// Resolve a circuit name against a source's published index, falling back
    /// to a flat layout when the source publishes none.
    ///
    /// The protocol (agreed with melange-circuits, thread 587; spec in
    /// `docs/CIRCUIT_INDEX.md`) is deliberately generic — any repository can
    /// serve one, and melange special-cases nobody:
    ///
    /// 1. GET `<base>/circuits-index.json`.
    /// 2. Present  -> resolve `name` through it to a repo-relative path.
    /// 3. Absent (404) -> flat `<base>/<name>.cir`, which is what melange did
    ///    before indexes existed, so unindexed sources keep working.
    ///
    /// **Present-but-missing is an ERROR, not a fallback.** An indexed source
    /// has declared its contents; guessing at `<base>/<typo>.cir` past that
    /// declaration costs a request and then reports the wrong problem — "not
    /// found at .../passiveq1a.cir" instead of "that name is not in this
    /// source; did you mean passive-eq1a?".
    ///
    /// `force_index_refresh` re-fetches the index past the cache. The caller
    /// uses it after a deck 404 on an indexed source, which is how a promotion
    /// (a deck moving tier) self-heals instead of needing a manual cache clear.
    pub fn resolve_circuit_indexed(
        &self,
        source: &str,
        circuit: &str,
        cache: &crate::cache::Cache,
        force_index_refresh: bool,
    ) -> Result<String> {
        let base_url = self.source_base(source)?;
        let index_url = format!("{}/circuits-index.json", base_url);

        let raw = match cache.get_sync(&index_url, force_index_refresh) {
            Ok(raw) => raw,
            Err(e) if e.downcast_ref::<crate::cache::NotFound>().is_some() => {
                // No index published: flat layout, as before.
                return self.resolve_circuit(source, circuit);
            }
            Err(e) => return Err(e),
        };

        let index: CircuitIndex = serde_json::from_str(&raw).with_context(|| {
            format!("{index_url}: not a valid circuits-index.json (see docs/CIRCUIT_INDEX.md)")
        })?;
        let name = circuit.strip_suffix(".cir").unwrap_or(circuit);

        match index.circuits.get(name) {
            Some(entry) => Ok(format!(
                "{}/{}",
                base_url,
                entry.path.trim_start_matches('/')
            )),
            None => {
                let near = near_matches(name, index.circuits.keys());
                let hint = if near.is_empty() {
                    format!(
                        "Run `melange sources show {source}` for the source, or browse its \
                         circuits-index.json ({} circuits).",
                        index.circuits.len()
                    )
                } else {
                    format!("Did you mean: {}?", near.join(", "))
                };
                anyhow::bail!("'{name}' is not in the '{source}' circuit index. {hint}")
            }
        }
    }

    fn source_base(&self, source: &str) -> Result<String> {
        let c = self.sources.get(source).ok_or_else(|| {
            anyhow::anyhow!(
                "Unknown source: '{}'\nUse 'melange sources list' to see available sources.",
                source
            )
        })?;
        Ok(c.url.trim_end_matches('/').to_string())
    }

    pub fn resolve_circuit(&self, source: &str, circuit: &str) -> Result<String> {
        let source_config = self.sources.get(source).ok_or_else(|| {
            anyhow::anyhow!(
                "Unknown source: '{}'\n\
                 Use 'melange sources list' to see available sources.",
                source
            )
        })?;

        // Construct circuit filename
        let circuit_name = if circuit.ends_with(".cir") {
            circuit.to_string()
        } else {
            format!("{}.cir", circuit)
        };

        // Build URL
        let base_url = source_config.url.trim_end_matches('/');
        let url = if let Some(subdir) = &source_config.subdirectory {
            format!("{}/{}/{}", base_url, subdir.trim_matches('/'), circuit_name)
        } else {
            format!("{}/{}", base_url, circuit_name)
        };

        Ok(url)
    }

    /// List all configured sources
    pub fn list_sources(&self) -> Vec<(&String, &SourceConfig)> {
        self.sources.iter().collect()
    }

    /// Get a specific source configuration
    pub fn get_source(&self, name: &str) -> Option<&SourceConfig> {
        self.sources.get(name)
    }

    /// Check if a source exists
    pub fn has_source(&self, name: &str) -> bool {
        self.sources.contains_key(name)
    }
}

/// Display sources in a formatted table
pub fn format_sources_list(config: &SourcesConfig) -> String {
    let mut output = String::new();

    output.push_str("Configured circuit sources:\n");
    output.push('\n');

    if config.sources.is_empty() {
        output.push_str("  (no sources configured)\n");
        return output;
    }

    // Find column widths
    let name_width = config
        .sources
        .keys()
        .map(|k| k.len())
        .max()
        .unwrap_or(10)
        .max(10);

    // Header
    output.push_str(&format!(
        "  {:<width$}  {:<40}  {:<15}\n",
        "NAME",
        "URL",
        "LICENSE",
        width = name_width
    ));
    output.push_str(&format!(
        "  {:-<width$}  {:-<40}  {:-<15}\n",
        "",
        "",
        "",
        width = name_width
    ));

    // Rows
    let mut sources: Vec<_> = config.sources.iter().collect();
    sources.sort_by(|a, b| a.0.cmp(b.0));

    for (name, source) in sources {
        let license = source.license.as_deref().unwrap_or("unknown");
        output.push_str(&format!(
            "  {:<width$}  {:<40}  {:<15}\n",
            name,
            truncate(&source.url, 38),
            license,
            width = name_width
        ));
    }

    if let Some(default) = &config.default_source {
        output.push_str(&format!("\n* Default source: {}\n", default));
    }

    output
}

fn truncate(s: &str, max_len: usize) -> String {
    if s.len() <= max_len {
        s.to_string()
    } else {
        // Find a char boundary at or before the desired cut point
        let end = max_len.saturating_sub(3);
        let boundary = s
            .char_indices()
            .take_while(|&(i, _)| i <= end)
            .last()
            .map(|(i, _)| i)
            .unwrap_or(0);
        format!("{}...", &s[..boundary])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        // No external sources are pre-seeded (they 404'd); a fresh install
        // relies on the builtin demo + user-added sources.
        let config = SourcesConfig::default_config();
        assert!(config.sources.is_empty());
        assert!(config.default_source.is_none());
    }

    #[test]
    /// A local source resolves through an index exactly as a remote one does.
    /// `melange index` writes these; if they were only usable after publishing
    /// to HTTP, the generator's own output would be dead on arrival.
    #[test]
    fn local_source_resolves_through_its_index() {
        let d = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(d.path().join("fuzz")).unwrap();
        std::fs::write(d.path().join("fuzz/big-muff.cir"), "* x\n").unwrap();
        std::fs::write(
            d.path().join("circuits-index.json"),
            r#"{"schema":1,"circuits":{"big-muff":{"path":"fuzz/big-muff.cir"}}}"#,
        )
        .unwrap();
        let got = SourcesConfig::resolve_local(d.path(), "big-muff").unwrap();
        assert_eq!(got, d.path().join("fuzz/big-muff.cir"));
    }

    /// No index: flat layout, same fallback the remote path uses.
    #[test]
    fn local_source_falls_back_to_flat() {
        let d = tempfile::tempdir().unwrap();
        std::fs::write(d.path().join("rc.cir"), "* x\n").unwrap();
        assert_eq!(
            SourcesConfig::resolve_local(d.path(), "rc").unwrap(),
            d.path().join("rc.cir")
        );
    }

    /// Indexed but absent is an error with a suggestion, never a silent
    /// fall-through to a flat guess — the rule the remote path follows.
    #[test]
    fn local_missing_name_suggests_instead_of_guessing() {
        let d = tempfile::tempdir().unwrap();
        std::fs::write(d.path().join("big-muff.cir"), "* x\n").unwrap();
        std::fs::write(
            d.path().join("circuits-index.json"),
            r#"{"schema":1,"circuits":{"big-muff":{"path":"big-muff.cir"}}}"#,
        )
        .unwrap();
        let e = SourcesConfig::resolve_local(d.path(), "bigmuff")
            .unwrap_err()
            .to_string();
        assert!(e.contains("Did you mean: big-muff?"), "{e}");
    }

    /// Unindexed and absent names the command that fixes it.
    #[test]
    fn local_unindexed_miss_names_the_fix() {
        let d = tempfile::tempdir().unwrap();
        let e = SourcesConfig::resolve_local(d.path(), "nope")
            .unwrap_err()
            .to_string();
        assert!(e.contains("melange index"), "{e}");
    }

    fn test_resolve_circuit() {
        // Resolution mechanics, against a user-added source (nothing pre-seeded).
        let mut config = SourcesConfig::default_config();
        config.add_source(
            "tonestack",
            "https://raw.githubusercontent.com/tonestack/tonestack/main",
            Some("MIT"),
            None,
        );

        // Without extension
        let url = config
            .resolve_circuit("tonestack", "fender-bassman")
            .unwrap();
        assert!(url.ends_with("fender-bassman.cir"));

        // With extension
        let url = config.resolve_circuit("tonestack", "test.cir").unwrap();
        assert!(url.ends_with("test.cir"));
        assert!(!url.contains("test.cir.cir"));
    }

    #[test]
    fn test_resolve_unknown_source() {
        let config = SourcesConfig::default_config();
        let result = config.resolve_circuit("unknown", "circuit");
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("Unknown source"));
    }

    #[test]
    fn test_add_remove_source() {
        let mut config = SourcesConfig::default();

        config.add_source("test", "https://example.com", Some("MIT"), None);
        assert!(config.has_source("test"));

        let removed = config.remove_source("test");
        assert!(removed);
        assert!(!config.has_source("test"));

        let removed = config.remove_source("nonexistent");
        assert!(!removed);
    }
}
