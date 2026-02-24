//! Friendly sources configuration for melange-cli
//!
//! Manages external circuit repositories that can be referenced
//! using the friendly `source:circuit` syntax.

use std::collections::HashMap;
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

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
            let content = std::fs::read_to_string(&config_path)
                .with_context(|| format!("Failed to read config file: {}", config_path.display()))?;
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
            std::fs::create_dir_all(parent)
                .with_context(|| format!("Failed to create config directory: {}", parent.display()))?;
        }
        
        let content = toml::to_string_pretty(self)
            .with_context(|| "Failed to serialize config to TOML")?;
        
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

    /// Create default configuration with known sources
    fn default_config() -> Self {
        let mut sources = HashMap::new();

        // Guitarix - popular guitar effects collection
        sources.insert("guitarix".to_string(), SourceConfig {
            url: "https://raw.githubusercontent.com/brummer10/guitarix/master".to_string(),
            license: Some("GPL-3.0".to_string()),
            attribution: Some("Guitarix Project (brummer10)".to_string()),
            subdirectory: Some("trunk/tools/ampsim".to_string()),
        });

        // Tonestack - tone stack circuits
        sources.insert("tonestack".to_string(), SourceConfig {
            url: "https://raw.githubusercontent.com/tonestack/tonestack/main".to_string(),
            license: Some("MIT".to_string()),
            attribution: Some("Tonestack Project".to_string()),
            subdirectory: None,
        });

        // Melange community circuits
        sources.insert("melange".to_string(), SourceConfig {
            url: "https://raw.githubusercontent.com/melange-audio/circuits/main".to_string(),
            license: Some("MIT OR Apache-2.0".to_string()),
            attribution: Some("Melange Community".to_string()),
            subdirectory: None,
        });

        Self { 
            sources,
            default_source: Some("melange".to_string()),
        }
    }

    /// Add a new source
    pub fn add_source(&mut self, name: &str, url: &str, license: Option<&str>, attribution: Option<&str>) {
        self.sources.insert(name.to_string(), SourceConfig {
            url: url.to_string(),
            license: license.map(|s| s.to_string()),
            attribution: attribution.map(|s| s.to_string()),
            subdirectory: None,
        });
    }

    /// Remove a source
    pub fn remove_source(&mut self, name: &str) -> bool {
        self.sources.remove(name).is_some()
    }

    /// Resolve a circuit from a source to a full URL
    ///
    /// Automatically appends `.cir` extension if not present.
    pub fn resolve_circuit(&self, source: &str, circuit: &str) -> Result<String> {
        let source_config = self.sources.get(source)
            .ok_or_else(|| anyhow::anyhow!(
                "Unknown source: '{}'\n\
                 Use 'melange sources list' to see available sources.",
                source
            ))?;

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
    output.push_str("\n");

    if config.sources.is_empty() {
        output.push_str("  (no sources configured)\n");
        return output;
    }

    // Find column widths
    let name_width = config.sources.keys().map(|k| k.len()).max().unwrap_or(10).max(10);
    
    // Header
    output.push_str(&format!("  {:<width$}  {:<40}  {:<15}\n", 
        "NAME", "URL", "LICENSE", width = name_width));
    output.push_str(&format!("  {:-<width$}  {:-<40}  {:-<15}\n", 
        "", "", "", width = name_width));

    // Rows
    let mut sources: Vec<_> = config.sources.iter().collect();
    sources.sort_by(|a, b| a.0.cmp(b.0));

    for (name, source) in sources {
        let license = source.license.as_deref().unwrap_or("unknown");
        output.push_str(&format!("  {:<width$}  {:<40}  {:<15}\n", 
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
        format!("{}...", &s[..max_len.saturating_sub(3)])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = SourcesConfig::default_config();
        assert!(!config.sources.is_empty());
        assert!(config.sources.contains_key("guitarix"));
        assert!(config.sources.contains_key("tonestack"));
    }

    #[test]
    fn test_resolve_circuit() {
        let config = SourcesConfig::default_config();
        
        // With subdirectory
        let url = config.resolve_circuit("guitarix", "bigmuff").unwrap();
        assert!(url.contains("githubusercontent.com"));
        assert!(url.ends_with("bigmuff.cir"));
        
        // Without extension
        let url = config.resolve_circuit("tonestack", "fender-bassman").unwrap();
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
