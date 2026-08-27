//! Circuit resolution system for melange-cli
//!
//! Supports multiple circuit sources:
//! - Builtins: Embedded circuits shipped with melange
//! - Friendly sources: `source:circuit` pattern (e.g., melange:tube-screamer)
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
/// let source = resolve("melange:tube-screamer").unwrap();
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

    // Try direct URL. This must run BEFORE the friendly-ref branch: a URL
    // contains ':' and would otherwise be mis-parsed as source "https" /
    // circuit "//example.com/…", making direct URL refs unreachable.
    if circuit_ref.starts_with("http://") || circuit_ref.starts_with("https://") {
        return Ok(CircuitSource::Url {
            url: circuit_ref.to_string(),
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

    // Bare circuit name → fall back to the configured default source. This is
    // the documented intent of `melange builtins`' usage examples
    // ("melange compile tube-screamer …" with no source prefix).
    if is_bare_name(circuit_ref) {
        let config = crate::sources::SourcesConfig::load()?;
        if let Some(default) = config.default_source.clone() {
            if config.has_source(&default) {
                let url = config.resolve_circuit(&default, circuit_ref)?;
                return Ok(CircuitSource::Friendly {
                    source: default,
                    circuit: circuit_ref.to_string(),
                    url,
                });
            }
        }
    }

    anyhow::bail!(
        "Cannot resolve circuit reference: '{}'\n\n\
         Tried:\n\
         - Local file (not found)\n\
         - Builtin circuit (not found)\n\
         - URL (must start with http:// or https://)\n\
         - Friendly source (invalid format)\n\
         - Default source (bare name, but no default source is configured)\n\n\
         Use 'melange sources list' to see configured sources.",
        circuit_ref
    )
}

/// The one builtin demo circuit: a Pultec-style passive tube EQ. Embedded at
/// compile time (`include_str!` from `examples/`, so the builtin and the
/// bundled example are the same bytes — one source of truth). This gives
/// melange a way to compile/simulate/demo itself with zero network and no
/// external circuit repo. The full circuit library lives in a separate repo
/// (being published); once it is reachable, add it with `melange sources add`
/// and browse with `melange sources list`.
const BUILTIN_PASSIVE_EQ: &str = include_str!("../../../examples/passive-eq1a.cir");

/// Get builtin circuit content.
///
/// Only the passive-eq demo is embedded (see [`BUILTIN_PASSIVE_EQ`]); every
/// other circuit resolves through configured sources. `passive-eq` is accepted
/// as a friendly alias for `passive-eq1a`.
fn get_builtin(name: &str) -> Option<String> {
    match name {
        "passive-eq1a" | "passive-eq" => Some(BUILTIN_PASSIVE_EQ.to_string()),
        _ => None,
    }
}

/// A bare circuit name: no source prefix, no path separators, non-empty.
/// Eligible for the default-source fallback.
fn is_bare_name(circuit_ref: &str) -> bool {
    !circuit_ref.is_empty()
        && !circuit_ref.contains(':')
        && !circuit_ref.contains('@')
        && !circuit_ref.contains('/')
        && !circuit_ref.contains('\\')
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

/// List all available builtin circuits.
///
/// Just the one demo circuit for now — the full catalog lives in the separate
/// circuit-library repo (browse with `melange sources list` once it is added).
pub fn list_builtins() -> Vec<(&'static str, &'static str)> {
    vec![(
        "passive-eq1a",
        "Pultec-style passive tube EQ — 4 tubes, 3 transformers, global NFB (demo)",
    )]
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
        let result = parse_friendly_ref("melange:tube-screamer");
        assert_eq!(
            result,
            Some(("melange".to_string(), "tube-screamer".to_string()))
        );
    }

    #[test]
    fn test_parse_friendly_ref_at() {
        let result = parse_friendly_ref("tube-screamer@melange");
        assert_eq!(
            result,
            Some(("melange".to_string(), "tube-screamer".to_string()))
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
    fn test_resolve_direct_url() {
        // Regression: URLs used to be swallowed by the friendly-ref branch
        // (split at ':' → source "https"), so direct URL refs always failed
        // with "Unknown source: 'https'".
        let source = resolve("https://example.com/circuits/fuzz.cir").unwrap();
        match source {
            CircuitSource::Url { url } => {
                assert_eq!(url, "https://example.com/circuits/fuzz.cir");
            }
            other => panic!("expected Url variant, got {other:?}"),
        }
        let source = resolve("http://example.com/a.cir").unwrap();
        assert!(matches!(source, CircuitSource::Url { .. }));
    }

    #[test]
    fn test_is_bare_name() {
        assert!(is_bare_name("tube-screamer"));
        assert!(is_bare_name("rc_lowpass"));
        assert!(!is_bare_name(""));
        assert!(!is_bare_name("melange:tube-screamer"));
        assert!(!is_bare_name("tube-screamer@melange"));
        assert!(!is_bare_name("./local/file.cir"));
        assert!(!is_bare_name("C:\\file.txt"));
        assert!(!is_bare_name("https://example.com/a.cir"));
    }

    #[test]
    fn test_get_builtin() {
        // The passive-eq demo is the one embedded builtin (+ its alias);
        // everything else resolves through configured sources.
        assert!(get_builtin("passive-eq1a").is_some());
        assert!(get_builtin("passive-eq").is_some());
        assert!(get_builtin("tube-screamer").is_none());
        assert!(get_builtin("nonexistent").is_none());
    }

    #[test]
    fn test_builtin_content_is_the_passive_eq_deck() {
        // Sanity: the embedded content is actually the passive-eq netlist,
        // and resolve() picks it up as a Builtin for a bare name.
        let content = get_builtin("passive-eq1a").unwrap();
        assert!(content.contains("Passive-EQ1A"));
        match resolve("passive-eq1a").unwrap() {
            CircuitSource::Builtin { name, .. } => assert_eq!(name, "passive-eq1a"),
            other => panic!("expected Builtin, got {other:?}"),
        }
    }

    #[test]
    fn test_list_builtins() {
        let builtins = list_builtins();
        assert_eq!(builtins.len(), 1);
        assert_eq!(builtins[0].0, "passive-eq1a");
    }
}
