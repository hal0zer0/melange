//! Circuit resolution system for melange-cli
//!
//! Supports multiple circuit sources:
//! - Builtins: Embedded circuits shipped with melange
//! - Friendly sources: `source:circuit` pattern (e.g., guitarix:bigmuff)
//! - Direct URLs: Full HTTP(S) URLs
//! - Local files: Path resolution

use anyhow::{Context, Result};
use std::path::PathBuf;

/// Resolved circuit source information
#[derive(Debug, Clone)]
pub enum CircuitSource {
    /// Built-in circuit embedded at compile time
    Builtin { name: String, content: String },
    /// Friendly source reference resolved to URL
    Friendly {
        source: String,
        circuit: String,
        url: String,
    },
    /// Direct HTTP(S) URL
    Url { url: String },
    /// Local file path
    Local { path: PathBuf },
}

impl CircuitSource {
    /// Get a display name for this circuit source
    pub fn name(&self) -> String {
        match self {
            CircuitSource::Builtin { name, .. } => format!("builtin:{}", name),
            CircuitSource::Friendly {
                source, circuit, ..
            } => format!("{}:{}", source, circuit),
            CircuitSource::Url { url } => url.clone(),
            CircuitSource::Local { path } => path.display().to_string(),
        }
    }

    /// Get the circuit content as a string
    ///
    /// For remote sources (Url, Friendly), this requires fetching from network.
    pub fn content_sync(&self) -> Result<String> {
        match self {
            CircuitSource::Builtin { content, .. } => Ok(content.clone()),
            CircuitSource::Local { path } => std::fs::read_to_string(path)
                .with_context(|| format!("Failed to read local circuit file: {}", path.display())),
            CircuitSource::Url { .. } | CircuitSource::Friendly { .. } => {
                anyhow::bail!(
                    "Remote circuit sources require async content fetching. Use content_async() or the cache module."
                )
            }
        }
    }

    /// Get the circuit content (async version for remote sources)
    #[cfg(feature = "async")]
    pub async fn content_async(&self, cache: &crate::cache::Cache) -> Result<String> {
        match self {
            CircuitSource::Builtin { content, .. } => Ok(content.clone()),
            CircuitSource::Local { path } => std::fs::read_to_string(path)
                .with_context(|| format!("Failed to read local circuit file: {}", path.display())),
            CircuitSource::Url { url } => cache.get(url, false).await,
            CircuitSource::Friendly { url, .. } => cache.get(url, false).await,
        }
    }
}

/// Resolve a circuit reference to its source
///
/// Resolution order:
/// 1. Local file (if exists on disk)
/// 2. Builtin (embedded)
/// 3. Friendly source (source:circuit pattern)
/// 4. Direct URL (http:// or https://)
///
/// # Examples
///
/// ```
/// use melange_cli::circuits::resolve;
///
/// // Builtin circuit
/// let source = resolve("tube-screamer").unwrap();
///
/// // Friendly source
/// let source = resolve("guitarix:bigmuff").unwrap();
///
/// // Direct URL
/// let source = resolve("https://example.com/circuit.cir").unwrap();
///
/// // Local file
/// let source = resolve("./my-circuit.cir").unwrap();
/// ```
pub fn resolve(circuit_ref: &str) -> Result<CircuitSource> {
    // Try local file first (if it exists and has an extension or path separator)
    let path = PathBuf::from(circuit_ref);
    if path.exists() && (path.is_file() || circuit_ref.contains('/') || circuit_ref.contains('\\'))
    {
        return Ok(CircuitSource::Local { path });
    }

    // Try builtin
    if let Some(content) = get_builtin(circuit_ref) {
        return Ok(CircuitSource::Builtin {
            name: circuit_ref.to_string(),
            content,
        });
    }

    // Try friendly source (source:circuit pattern)
    if let Some((source, circuit)) = parse_friendly_ref(circuit_ref) {
        let config = crate::sources::SourcesConfig::load()?;
        let url = config.resolve_circuit(&source, &circuit)?;
        return Ok(CircuitSource::Friendly {
            source,
            circuit,
            url,
        });
    }

    // Try direct URL
    if circuit_ref.starts_with("http://") || circuit_ref.starts_with("https://") {
        return Ok(CircuitSource::Url {
            url: circuit_ref.to_string(),
        });
    }

    anyhow::bail!(
        "Cannot resolve circuit reference: '{}'\n\n\
         Tried:\n\
         - Local file (not found)\n\
         - Builtin circuit (not found)\n\
         - Friendly source (invalid format)\n\
         - URL (must start with http:// or https://)\n\n\
         Use 'melange builtins' to list available builtin circuits.",
        circuit_ref
    )
}

/// Get builtin circuit content
fn get_builtin(name: &str) -> Option<String> {
    match name {
        "tube-screamer" => Some(include_str!("builtins/tube-screamer.cir").to_string()),
        "fuzz-face" => Some(include_str!("builtins/fuzz-face.cir").to_string()),
        "big-muff" => Some(include_str!("builtins/big-muff.cir").to_string()),
        "rc-lowpass" => Some(include_str!("builtins/rc-lowpass.cir").to_string()),
        "mordor-screamer" => Some(include_str!("builtins/mordor-screamer.cir").to_string()),
        "tube-preamp" => Some(include_str!("builtins/tube-preamp.cir").to_string()),
        _ => None,
    }
}

/// Parse friendly source reference (source:circuit or circuit@source)
///
/// Supports two formats:
/// - `source:circuit` (colon separator, preferred)
/// - `circuit@source` (at separator, alternative)
fn parse_friendly_ref(circuit_ref: &str) -> Option<(String, String)> {
    // Handle source:circuit format (preferred)
    if let Some((source, circuit)) = circuit_ref.split_once(':') {
        // Make sure it's not a Windows path like C:\file.txt
        if source.len() > 1 || !source.chars().next().unwrap().is_ascii_alphabetic() {
            return Some((source.to_string(), circuit.to_string()));
        }
    }
    // Handle circuit@source format (alternative)
    if let Some((circuit, source)) = circuit_ref.rsplit_once('@') {
        return Some((source.to_string(), circuit.to_string()));
    }
    None
}

/// List all available builtin circuits
pub fn list_builtins() -> Vec<(&'static str, &'static str)> {
    vec![
        (
            "tube-screamer",
            "Op-amp overdrive with diode feedback clipping (TS808 style)",
        ),
        ("fuzz-face", "2-transistor fuzz (Fuzz Face style)"),
        (
            "big-muff",
            "4-transistor fuzz with dual clipping (Big Muff style)",
        ),
        (
            "tube-preamp",
            "Common-cathode 12AX7 triode gain stage with tone control",
        ),
        ("rc-lowpass", "Simple RC lowpass filter for testing"),
        (
            "mordor-screamer",
            "High-gain distortion forged in Mount Doom",
        ),
    ]
}

/// Fetch circuit content synchronously using blocking HTTP client
///
/// This is a convenience function for simple use cases.
/// For production use with caching, use the cache module.
pub fn fetch_url_sync(url: &str) -> Result<String> {
    use std::io::Read;

    let response = ureq::get(url)
        .call()
        .with_context(|| format!("Failed to fetch URL: {}", url))?;

    let mut content = String::new();
    response
        .into_reader()
        .read_to_string(&mut content)
        .with_context(|| "Failed to read response body")?;

    Ok(content)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_friendly_ref_colon() {
        let result = parse_friendly_ref("guitarix:bigmuff");
        assert_eq!(
            result,
            Some(("guitarix".to_string(), "bigmuff".to_string()))
        );
    }

    #[test]
    fn test_parse_friendly_ref_at() {
        let result = parse_friendly_ref("bigmuff@guitarix");
        assert_eq!(
            result,
            Some(("guitarix".to_string(), "bigmuff".to_string()))
        );
    }

    #[test]
    fn test_parse_friendly_ref_no_match() {
        let result = parse_friendly_ref("just-a-name");
        assert_eq!(result, None);
    }

    #[test]
    fn test_parse_friendly_ref_windows_path() {
        // Windows paths like C:\file.txt should not be parsed as friendly refs
        let result = parse_friendly_ref("C:\\file.txt");
        assert_eq!(result, None);
    }

    #[test]
    fn test_get_builtin() {
        assert!(get_builtin("tube-screamer").is_some());
        assert!(get_builtin("rc-lowpass").is_some());
        assert!(get_builtin("nonexistent").is_none());
    }

    #[test]
    fn test_list_builtins() {
        let builtins = list_builtins();
        assert!(!builtins.is_empty());
        assert!(builtins.iter().any(|(name, _)| *name == "tube-screamer"));
    }
}
