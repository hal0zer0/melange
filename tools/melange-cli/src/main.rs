//! melange-cli - Command line tool for circuit modeling
//!
//! Usage:
//!   melange compile input.cir --output circuit.rs
//!   melange validate input.cir --reference reference.raw
//!   melange simulate input.cir --input input.wav --output output.wav
//!   melange sources list
//!   melange builtins

mod builtins {
    // This module exists to include builtin circuit files
    // The actual content is embedded using include_str! in circuits.rs
}

pub mod cache;
pub mod circuits;
pub mod plugin_template;
pub mod sources;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "melange")]
#[command(about = "Circuit modeling toolkit - from SPICE to real-time DSP")]
#[command(version)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Compile a SPICE netlist to optimized Rust code
    Compile {
        /// Input SPICE netlist file or circuit reference
        /// (builtin:circuit, source:circuit, URL, or local path)
        input: String,

        /// Output Rust file or directory
        #[arg(short, long)]
        output: PathBuf,

        /// Sample rate in Hz
        #[arg(short, long, default_value = "48000")]
        sample_rate: f64,

        /// Input node name
        #[arg(short, long, default_value = "in")]
        input_node: String,

        /// Output node name
        #[arg(short = 'n', long, default_value = "out")]
        output_node: String,

        /// Maximum NR iterations
        #[arg(long, default_value = "20")]
        max_iter: usize,

        /// Convergence tolerance
        #[arg(long, default_value = "1e-9")]
        tolerance: f64,

        /// Output format
        #[arg(short = 'f', long, value_enum, default_value = "code")]
        format: OutputFormat,
    },

    /// Validate circuit against SPICE reference
    Validate {
        /// Input SPICE netlist file or circuit reference
        input: String,

        /// Reference SPICE output file
        #[arg(short, long)]
        reference: PathBuf,

        /// Sample rate in Hz
        #[arg(short, long, default_value = "48000")]
        sample_rate: f64,
    },

    /// Simulate circuit with input signal
    Simulate {
        /// Input SPICE netlist file or circuit reference
        input: String,

        /// Input audio file (WAV)
        #[arg(short, long)]
        input_audio: Option<PathBuf>,

        /// Output audio file (WAV)
        #[arg(short, long)]
        output: PathBuf,

        /// Sample rate in Hz
        #[arg(short, long, default_value = "48000")]
        sample_rate: f64,

        /// Duration in seconds
        #[arg(short, long, default_value = "1.0")]
        duration: f64,
    },

    /// List available nodes in a netlist
    Nodes {
        /// Input SPICE netlist file or circuit reference
        input: String,
    },

    /// Manage circuit sources (friendly source:circuit references)
    Sources {
        #[command(subcommand)]
        action: SourceAction,
    },

    /// List available builtin circuits
    Builtins,

    /// Manage the circuit cache
    Cache {
        #[command(subcommand)]
        action: CacheAction,
    },
}

#[derive(Subcommand)]
enum SourceAction {
    /// List configured sources
    List,

    /// Add a new source
    Add {
        /// Source name
        name: String,
        /// Base URL for the source
        url: String,
        /// License identifier (optional)
        #[arg(short, long)]
        license: Option<String>,
        /// Attribution string (optional)
        #[arg(short, long)]
        attribution: Option<String>,
    },

    /// Remove a source
    Remove {
        /// Source name to remove
        name: String,
    },

    /// Show details for a source
    Show {
        /// Source name
        name: String,
    },
}

#[derive(Subcommand)]
enum CacheAction {
    /// Show cache contents
    List,

    /// Clear all cached files
    Clear,

    /// Show cache statistics
    Stats,
}

#[derive(ValueEnum, Clone, Debug)]
enum OutputFormat {
    /// Generate only the circuit code (default)
    Code,
    /// Generate a complete plugin project
    Plugin,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Compile {
            input,
            output,
            sample_rate,
            input_node,
            output_node,
            max_iter,
            tolerance,
            format,
        } => {
            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            compile_circuit_source(
                &circuit_source,
                &output,
                sample_rate,
                &input_node,
                &output_node,
                max_iter,
                tolerance,
                format,
            )
        }
        Commands::Validate {
            input,
            reference,
            sample_rate,
        } => {
            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            validate_circuit_source(&circuit_source, &reference, sample_rate)
        }
        Commands::Simulate {
            input,
            input_audio,
            output,
            sample_rate,
            duration,
        } => {
            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            simulate_circuit_source(&circuit_source, input_audio.as_deref(), &output, sample_rate, duration)
        }
        Commands::Nodes { input } => {
            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            list_nodes_source(&circuit_source)
        }
        Commands::Sources { action } => handle_sources(action),
        Commands::Builtins => list_builtins(),
        Commands::Cache { action } => handle_cache(action),
    }
}

fn compile_circuit_source(
    circuit_source: &circuits::CircuitSource,
    output: &PathBuf,
    sample_rate: f64,
    input_node: &str,
    output_node: &str,
    max_iter: usize,
    tolerance: f64,
    format: OutputFormat,
) -> Result<()> {
    use melange_solver::{
        codegen::{CodegenConfig, CodeGenerator},
        dk::DkKernel,
        mna::MnaSystem,
        parser::Netlist,
    };

    println!("melange compile");
    println!("  Source: {}", circuit_source.name());
    println!("  Output: {}", output.display());
    println!("  Sample rate: {} Hz", sample_rate);
    println!();

    // Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, name } => {
            println!("  Using builtin circuit: {}", name);
            content.clone()
        }
        circuits::CircuitSource::Local { path } => {
            std::fs::read_to_string(path)
                .with_context(|| format!("Failed to read local file: {}", path.display()))?
        }
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            println!("  Fetching from URL: {}", url);
            let cache = cache::Cache::new()?;
            cache.get_sync(url, false)?
        }
    };

    // Step 1: Parse netlist
    println!("Step 1: Parsing SPICE netlist...");
    let netlist = Netlist::parse(&netlist_str)
        .with_context(|| "Failed to parse SPICE netlist")?;

    println!("  ✓ Parsed {} elements", netlist.elements.len());

    // Step 2: Build MNA system
    println!("Step 2: Building MNA system...");
    let mut mna = MnaSystem::from_netlist(&netlist)
        .with_context(|| "Failed to build MNA system")?;

    println!("  ✓ {} nodes, {} nonlinear devices", mna.n, mna.nonlinear_devices.len());

    // Get input node index and add input conductance to G matrix
    // This models the source impedance of the input voltage source
    let input_node_idx = mna
        .node_map
        .get(input_node)
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);
    // Use 1Ω input impedance so the input signal appears directly at the input node.
    // In a DAW plugin context, the input represents the actual voltage, not a source
    // with series impedance. A low R_in ensures V_node ≈ V_input.
    let input_resistance = 1.0;  // 1Ω (near-ideal voltage source)
    let input_conductance = 1.0 / input_resistance;
    if input_node_idx < mna.n {
        mna.g[input_node_idx][input_node_idx] += input_conductance;
    }

    // Step 3: Create DK kernel
    println!("Step 3: Creating DK kernel...");
    let kernel = DkKernel::from_mna(&mna, sample_rate)
        .with_context(|| "Failed to create DK kernel")?;

    println!("  ✓ Matrix dimensions: N={}, M={}", kernel.n, kernel.m);

    // Step 4: Generate code
    println!("Step 4: Generating Rust code...");

    let circuit_name = output
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("circuit")
        .to_string();

    let input_node_idx = mna
        .node_map
        .get(input_node)
        .copied()
        .unwrap_or(1)
        .saturating_sub(1);

    let output_node_idx = mna
        .node_map
        .get(output_node)
        .copied()
        .unwrap_or(2)
        .saturating_sub(1);

    let config = CodegenConfig {
        circuit_name,
        input_node: input_node_idx,
        output_node: output_node_idx,
        sample_rate,
        max_iter,
        max_iterations: max_iter,
        tol: tolerance,
        tolerance,
        include_dc_op: true,
        input_conductance,  // Matches the G matrix stamping above
        input_resistance,   // 10kΩ - proper voltage divider with circuit input
    };

    let generator = CodeGenerator::new(config);
    let generated = generator
        .generate(&kernel, &mna, &netlist)
        .with_context(|| "Code generation failed")?;

    println!("  ✓ Generated {} lines of Rust code", generated.code.lines().count());

    // Step 5: Write output
    println!("Step 5: Writing output...");

    match format {
        OutputFormat::Code => {
            // Write single file (existing behavior)
            std::fs::write(output, &generated.code)
                .with_context(|| format!("Failed to write output file: {}", output.display()))?;

            println!("  ✓ Done!");
            println!();
            println!("Generated code written to: {}", output.display());
            println!("This code can be used with:");
            println!("  - melange-plugin for VST/AU/CLAP plugins");
            println!("  - Standalone integration in your own projects");
            println!();
            println!("To generate a complete plugin project instead, use:");
            println!("  melange compile {} --output {} --format plugin", input_node, output.display());
        }
        OutputFormat::Plugin => {
            // Generate complete plugin project
            let project_dir = if output.extension().is_some() {
                output.with_extension("")
            } else {
                output.to_path_buf()
            };

            // Get a sanitized circuit name
            let circuit_name = project_dir
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("circuit")
                .to_string();

            plugin_template::generate_plugin_project(&project_dir, &generated.code, &circuit_name)?;

            println!("  ✓ Done!");
            println!();
            println!("Generated plugin project at: {}", project_dir.display());
            println!();
            println!("To build and run:");
            println!("  cd {}", project_dir.file_name().unwrap().to_str().unwrap());
            println!("  cargo run --release --bin {}-standalone", circuit_name);
            println!();
            println!("To build plugin formats (CLAP, VST3):");
            println!("  cargo xtask bundle {} --release", circuit_name);
            println!();
            println!("See README.md in the project directory for more information.");
        }
    }

    Ok(())
}

fn validate_circuit_source(
    _circuit_source: &circuits::CircuitSource,
    _reference: &PathBuf,
    _sample_rate: f64,
) -> Result<()> {
    println!("Validation not yet implemented.");
    println!("Use 'melange compile' to generate code, then compare against SPICE manually.");
    Ok(())
}

fn simulate_circuit_source(
    _circuit_source: &circuits::CircuitSource,
    _input_audio: Option<&std::path::Path>,
    _output: &PathBuf,
    _sample_rate: f64,
    _duration: f64,
) -> Result<()> {
    println!("Simulation not yet implemented.");
    println!("Use 'melange compile' to generate code, then process audio in your application.");
    Ok(())
}

fn list_nodes_source(circuit_source: &circuits::CircuitSource) -> Result<()> {
    use melange_solver::{mna::MnaSystem, parser::Netlist};

    println!("melange nodes");
    println!("  Source: {}", circuit_source.name());
    println!();

    // Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, .. } => content.clone(),
        circuits::CircuitSource::Local { path } => {
            std::fs::read_to_string(path)
                .with_context(|| format!("Failed to read local file: {}", path.display()))?
        }
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            let cache = cache::Cache::new()?;
            cache.get_sync(url, false)?
        }
    };

    let netlist = Netlist::parse(&netlist_str)
        .with_context(|| "Failed to parse SPICE netlist")?;

    let mna = MnaSystem::from_netlist(&netlist)
        .with_context(|| "Failed to build MNA system")?;

    println!("Nodes in circuit:");
    println!("  (0) GND - Ground reference");

    let mut nodes: Vec<_> = mna.node_map.iter().collect();
    nodes.sort_by(|a, b| a.1.cmp(b.1));

    for (name, &idx) in nodes {
        if name != "0" {
            println!("  ({}) {}", idx, name);
        }
    }

    if !mna.nonlinear_devices.is_empty() {
        println!();
        println!("Nonlinear devices:");
        for dev in &mna.nonlinear_devices {
            println!(
                "  {}: {:?} (dimension: {})",
                dev.name, dev.device_type, dev.dimension
            );
        }
    }

    Ok(())
}

fn handle_sources(action: SourceAction) -> Result<()> {
    use crate::sources::{SourcesConfig, format_sources_list};

    match action {
        SourceAction::List => {
            let config = SourcesConfig::load()?;
            println!("{}", format_sources_list(&config));
            Ok(())
        }
        SourceAction::Add { name, url, license, attribution } => {
            let mut config = SourcesConfig::load()?;
            
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
            } else {
                anyhow::bail!("Source '{}' not found", name);
            }
            
            Ok(())
        }
    }
}

fn list_builtins() -> Result<()> {
    println!("Available builtin circuits:");
    println!();

    let builtins = circuits::list_builtins();
    
    for (name, description) in builtins {
        println!("  {:<15} - {}", name, description);
    }

    println!();
    println!("Usage examples:");
    println!("  melange compile tube-screamer --output ts9.rs");
    println!("  melange compile rc-lowpass --output filter.rs");
    println!("  melange nodes big-muff");

    Ok(())
}

fn handle_cache(action: CacheAction) -> Result<()> {
    use crate::cache::{Cache, format_cache_list};

    match action {
        CacheAction::List => {
            let cache = Cache::new()?;
            println!("{}", format_cache_list(&cache));
            Ok(())
        }
        CacheAction::Clear => {
            let cache = Cache::new()?;
            cache.clear()?;
            println!("Cache cleared.");
            Ok(())
        }
        CacheAction::Stats => {
            let cache = Cache::new()?;
            let stats = cache.stats();
            println!("Cache statistics:");
            println!("  Location: {}", cache.cache_dir().display());
            println!("  Files: {}", stats.total_files);
            println!("  Size: {}", stats.formatted_size());
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_parse() {
        // Test that CLI parsing works
        let cli = Cli::parse_from(["melange", "builtins"]);
        match cli.command {
            Commands::Builtins => {},
            _ => panic!("Expected Builtins command"),
        }
    }
}
