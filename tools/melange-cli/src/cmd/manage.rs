use crate::cli::{CacheAction, SourceAction};
use crate::{circuits, codegen_runner};
use anyhow::{Context, Result};

pub(crate) fn handle_sources(action: SourceAction) -> Result<()> {
    use crate::sources::{format_sources_list, SourcesConfig};

    match action {
        SourceAction::List => {
            let config = SourcesConfig::load()?;
            println!("{}", format_sources_list(&config));
            Ok(())
        }
        SourceAction::Add {
            name,
            url,
            license,
            attribution,
        } => {
            let mut config = SourcesConfig::load()?;

            // Refuse here rather than store something that only fails later.
            // A rejected source used to be accepted, listed as healthy by
            // `sources list`, and then die at first use with a url-crate
            // internal message telling the user their absolute path was a
            // "relative URL without a base".
            let looks_remote = url.starts_with("http://") || url.starts_with("https://");
            if !looks_remote && !std::path::Path::new(url.trim_end_matches('/')).is_dir() {
                let hint = if url.starts_with("file://") {
                    "For a local folder give the plain path, not a file:// URL."
                } else if std::path::Path::new(&url).exists() {
                    "That path exists but is not a directory. A source is the \
                     FOLDER circuits live in, not one .cir file — to compile a \
                     single file just pass it directly."
                } else {
                    "A source is either an http(s) base URL or a local directory \
                     that exists."
                };
                anyhow::bail!("Cannot use '{url}' as a source. {hint}");
            }

            // Store a local directory absolute: a relative path only resolves
            // from the directory it was added in.
            let url = if looks_remote {
                url
            } else {
                std::fs::canonicalize(url.trim_end_matches('/'))
                    .with_context(|| format!("Cannot resolve '{url}'"))?
                    .to_string_lossy()
                    .into_owned()
            };

            if config.has_source(&name) {
                println!("Warning: Source '{}' already exists. Overwriting.", name);
            }

            config.add_source(&name, &url, license.as_deref(), attribution.as_deref());
            config.save()?;

            println!("Added source '{}': {}", name, url);
            if let Some(lic) = license {
                println!("  License: {}", lic);
            }
            if let Some(attr) = attribution {
                println!("  Attribution: {}", attr);
            }

            Ok(())
        }
        SourceAction::Remove { name } => {
            let mut config = SourcesConfig::load()?;

            if config.remove_source(&name) {
                config.save()?;
                println!("Removed source '{}'", name);
            } else {
                anyhow::bail!("Source '{}' not found", name);
            }

            Ok(())
        }
        SourceAction::Show { name } => {
            let config = SourcesConfig::load()?;

            if let Some(source) = config.get_source(&name) {
                println!("Source: {}", name);
                println!("  URL: {}", source.url);
                if let Some(lic) = &source.license {
                    println!("  License: {}", lic);
                }
                if let Some(attr) = &source.attribution {
                    println!("  Attribution: {}", attr);
                }
                if let Some(subdir) = &source.subdirectory {
                    println!("  Subdirectory: {}", subdir);
                }
                let cache = crate::cache::Cache::new()?;
                match config.list_circuits(&name, &cache)? {
                    Some(circuits) => {
                        println!();
                        println!("  {} circuits:", circuits.len());
                        let width = circuits.iter().map(|(n, _)| n.len()).max().unwrap_or(0);
                        for (circuit, entry) in &circuits {
                            let meta: Vec<&str> =
                                [entry.category.as_deref(), entry.tier.as_deref()]
                                    .into_iter()
                                    .flatten()
                                    .collect();
                            println!("    {circuit:<width$}  {}", meta.join(", "));
                        }
                        if let Some((first, _)) = circuits.first() {
                            println!();
                            println!("  Use one as `{name}:<circuit>`, e.g. `melange nodes {name}:{first}`.");
                        }
                    }
                    None => {
                        println!();
                        println!(
                            "  No circuits-index.json published, so its circuits cannot be \
                             listed; `{name}:<file-name>` still resolves <base>/<file-name>.cir."
                        );
                    }
                }
            } else {
                anyhow::bail!("Source '{}' not found", name);
            }

            Ok(())
        }
    }
}

pub(crate) fn list_builtins() -> Result<()> {
    println!("Available builtin circuits:");
    println!();

    let builtins = circuits::list_builtins();

    for (name, description) in builtins {
        println!("  {:<15} - {}", name, description);
    }

    println!();
    println!("Usage examples:");
    println!("  melange compile passive-eq1a --format plugin -o passive-eq");
    println!("  melange simulate passive-eq1a --amplitude 0.1 -o drive.wav");
    println!("  melange nodes passive-eq1a");
    println!();
    println!("The full circuit library lives in a separate repo. Add it, list it, use it:");
    println!(
        "  melange sources add melange-circuits \
         https://gitlab.com/oomox-group/melange-circuits/-/raw/main"
    );
    println!("  melange sources show melange-circuits");
    println!("  melange nodes melange-circuits:<circuit>");

    Ok(())
}

pub(crate) fn handle_cache(action: CacheAction) -> Result<()> {
    use crate::cache::{format_cache_list, Cache};
    use crate::codegen_runner::BinaryCache;

    match action {
        CacheAction::List => {
            let cache = Cache::new()?;
            println!("{}", format_cache_list(&cache));
            let bin_cache = BinaryCache::new()?;
            let bin_stats = bin_cache.stats();
            println!();
            println!("Compiled binaries:");
            println!(
                "  {} files ({})",
                bin_stats.total_files,
                bin_stats.formatted_size()
            );
            Ok(())
        }
        CacheAction::Clear { binaries } => {
            if !binaries {
                let cache = Cache::new()?;
                cache.clear()?;
            }
            let bin_cache = BinaryCache::new()?;
            bin_cache.clear()?;
            if binaries {
                println!("Compiled binaries cleared (downloaded circuits kept).");
            } else {
                println!("Cache cleared (circuits + compiled binaries).");
            }
            Ok(())
        }
        CacheAction::Stats => {
            let cache = Cache::new()?;
            let stats = cache.stats();
            println!("Circuit cache:");
            println!("  Location: {}", cache.cache_dir().display());
            println!("  Files: {}", stats.total_files);
            println!("  Size: {}", stats.formatted_size());
            let bin_cache = BinaryCache::new()?;
            let bin_stats = bin_cache.stats();
            println!();
            println!(
                "Binary cache (compiled simulate/analyze runs; `melange cache clear --binaries` \
                 empties it, `melange cache clear` empties both):"
            );
            println!("  Location: {}", bin_cache.cache_dir().display());
            println!("  Files: {}", bin_stats.total_files);
            println!("  Size: {}", bin_stats.formatted_size());
            println!(
                "  Limit: {} (oldest-used binaries are removed past it; {} sets it in MiB, 0 = \
                 no limit)",
                codegen_runner::format_cap(codegen_runner::binary_cache_cap()),
                codegen_runner::BINARY_CACHE_MAX_ENV
            );
            Ok(())
        }
    }
}
