//! Simple HTTP caching for remote circuit files
//!
//! Caches downloaded circuits in the user's cache directory
//! to avoid repeated network requests.

use std::path::PathBuf;
use anyhow::{Context, Result};

/// Circuit file cache manager
pub struct Cache {
    cache_dir: PathBuf,
}

impl Cache {
    /// Create a new cache instance
    ///
    /// The cache directory is typically:
    /// - Linux: ~/.cache/melange/circuits/
    /// - macOS: ~/Library/Caches/melange/circuits/
    /// - Windows: %LOCALAPPDATA%\melange\cache\circuits\
    pub fn new() -> Result<Self> {
        let cache_dir = dirs::cache_dir()
            .map(|p| p.join("melange").join("circuits"))
            .ok_or_else(|| anyhow::anyhow!("Cannot find cache directory"))?;

        std::fs::create_dir_all(&cache_dir)
            .with_context(|| format!("Failed to create cache directory: {}", cache_dir.display()))?;

        Ok(Self { cache_dir })
    }

    /// Get the cache directory path
    pub fn cache_dir(&self) -> &PathBuf {
        &self.cache_dir
    }

    /// Get content from cache or fetch from URL if missing/stale
    ///
    /// # Arguments
    /// * `url` - The URL to fetch
    /// * `force_refresh` - If true, always fetch from network
    ///
    /// # Caching behavior
    /// - Cache entries are valid for 24 hours by default
    /// - Stale entries are automatically refreshed
    pub fn get_sync(&self, url: &str, force_refresh: bool) -> Result<String> {
        let cache_path = self.url_to_cache_path(url);

        // Check if we can use cached version
        if !force_refresh
            && cache_path.exists()
            && let Ok(metadata) = std::fs::metadata(&cache_path)
            && let Ok(modified) = metadata.modified()
            && let Ok(age) = std::time::SystemTime::now().duration_since(modified)
            && age < std::time::Duration::from_secs(24 * 3600)
        {
            return std::fs::read_to_string(&cache_path)
                .with_context(|| format!("Failed to read cached file: {}", cache_path.display()));
        }

        // Fetch from URL using blocking client
        let content = fetch_url_sync(url)?;

        // Save to cache (best effort - don't fail if cache write fails)
        if let Some(parent) = cache_path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let _ = std::fs::write(&cache_path, &content);

        Ok(content)
    }

    /// Clear all cached files
    pub fn clear(&self) -> Result<()> {
        if self.cache_dir.exists() {
            std::fs::remove_dir_all(&self.cache_dir)
                .with_context(|| format!("Failed to clear cache directory: {}", self.cache_dir.display()))?;
        }
        std::fs::create_dir_all(&self.cache_dir)
            .with_context(|| format!("Failed to recreate cache directory: {}", self.cache_dir.display()))?;
        Ok(())
    }

    /// Clear cache for a specific URL
    pub fn clear_url(&self, url: &str) -> Result<()> {
        let cache_path = self.url_to_cache_path(url);
        if cache_path.exists() {
            std::fs::remove_file(&cache_path)
                .with_context(|| format!("Failed to remove cache file: {}", cache_path.display()))?;
        }
        Ok(())
    }

    /// Get cache statistics (recursively traverses subdirectories)
    pub fn stats(&self) -> CacheStats {
        let mut total_files = 0;
        let mut total_bytes = 0;

        fn visit_dir(dir: &std::path::Path, files: &mut usize, bytes: &mut u64) {
            if let Ok(entries) = std::fs::read_dir(dir) {
                for entry in entries.flatten() {
                    if let Ok(metadata) = entry.metadata() {
                        if metadata.is_file() {
                            *files += 1;
                            *bytes += metadata.len();
                        } else if metadata.is_dir() {
                            visit_dir(&entry.path(), files, bytes);
                        }
                    }
                }
            }
        }

        visit_dir(&self.cache_dir, &mut total_files, &mut total_bytes);

        CacheStats { total_files, total_bytes }
    }

    /// Convert URL to cache file path
    fn url_to_cache_path(&self, url: &str) -> PathBuf {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let mut hasher = DefaultHasher::new();
        url.hash(&mut hasher);
        let hash = hasher.finish();
        
        // Create a directory structure to avoid too many files in one directory
        let dir1 = format!("{:02x}", (hash >> 56) as u8);
        let dir2 = format!("{:02x}", ((hash >> 48) & 0xFF) as u8);
        let filename = format!("{:016x}.cir", hash);

        self.cache_dir.join(dir1).join(dir2).join(filename)
    }
}

/// Cache statistics
#[derive(Debug, Clone, Copy)]
pub struct CacheStats {
    pub total_files: usize,
    pub total_bytes: u64,
}

impl CacheStats {
    /// Format total bytes as human-readable string
    pub fn formatted_size(&self) -> String {
        const UNITS: &[&str] = &["B", "KB", "MB", "GB"];
        let mut size = self.total_bytes as f64;
        let mut unit_idx = 0;
        
        while size >= 1024.0 && unit_idx < UNITS.len() - 1 {
            size /= 1024.0;
            unit_idx += 1;
        }
        
        format!("{:.1} {}", size, UNITS[unit_idx])
    }
}

/// Fetch content from URL using blocking HTTP client
fn fetch_url_sync(url: &str) -> Result<String> {
    use std::io::Read;
    
    let response = ureq::get(url)
        .call()
        .with_context(|| format!("Failed to fetch URL: {}", url))?;
    
    let mut content = String::new();
    response.into_reader()
        .read_to_string(&mut content)
        .with_context(|| "Failed to read response body")?;
    
    Ok(content)
}

/// Format cache contents for display
pub fn format_cache_list(cache: &Cache) -> String {
    let mut output = String::new();
    let stats = cache.stats();
    
    output.push_str("Cached circuit files:\n");
    output.push_str(&format!("Location: {}\n", cache.cache_dir().display()));
    output.push_str(&format!("Total: {} files ({})", stats.total_files, stats.formatted_size()));
    
    if stats.total_files == 0 {
        output.push_str("\n\n  (cache is empty)\n");
        return output;
    }

    output.push_str("\n\n");

    // Recursively collect cached files
    fn collect_files(dir: &std::path::Path, base: &std::path::Path, out: &mut Vec<(String, u64)>) {
        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.flatten() {
                if let Ok(metadata) = entry.metadata() {
                    if metadata.is_file() {
                        let rel = entry.path().strip_prefix(base)
                            .map(|p| p.to_string_lossy().to_string())
                            .unwrap_or_else(|_| entry.file_name().to_string_lossy().to_string());
                        out.push((rel, metadata.len()));
                    } else if metadata.is_dir() {
                        collect_files(&entry.path(), base, out);
                    }
                }
            }
        }
    }

    let mut files = Vec::new();
    collect_files(cache.cache_dir(), cache.cache_dir(), &mut files);
    files.sort_by(|a, b| a.0.cmp(&b.0));

    for (name, size) in files.iter().take(20) {
        output.push_str(&format!("  {:<30} {:>8} B\n", name, size));
    }

    if files.len() > 20 {
        output.push_str(&format!("\n  ... and {} more files\n", files.len() - 20));
    }

    output
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_cache_stats() {
        let stats = CacheStats { total_files: 5, total_bytes: 1536 };
        assert_eq!(stats.formatted_size(), "1.5 KB");
        
        let stats = CacheStats { total_files: 0, total_bytes: 0 };
        assert_eq!(stats.formatted_size(), "0.0 B");
        
        let stats = CacheStats { total_files: 1, total_bytes: 1_500_000 };
        assert!(stats.formatted_size().contains("MB"));
    }

    #[test]
    fn test_url_to_cache_path() {
        let cache = Cache::new().unwrap();
        let path1 = cache.url_to_cache_path("https://example.com/circuit.cir");
        let path2 = cache.url_to_cache_path("https://example.com/circuit.cir");
        let path3 = cache.url_to_cache_path("https://other.com/circuit.cir");
        
        // Same URL should produce same path
        assert_eq!(path1, path2);
        // Different URLs should produce different paths
        assert_ne!(path1, path3);
        
        // Path should be under cache directory
        assert!(path1.starts_with(&cache.cache_dir));
    }
}
