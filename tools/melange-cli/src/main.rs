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

        /// Output node name(s), comma-separated for multi-output (e.g., "out_l,out_r")
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

        /// Output scale factor (default 1.0, use 0.1 to map ±10V to ±1.0 audio)
        #[arg(long, default_value = "1.0")]
        output_scale: f64,

        /// Add Input Level and Output Level parameters to the plugin (default: true)
        #[arg(long, default_value = "true")]
        with_level_params: bool,

        /// Generate plugin without Input/Output Level parameters
        #[arg(long)]
        no_level_params: bool,

        /// Override input resistance (ohms). Default: 1Ω, or from .input_impedance directive.
        #[arg(long)]
        input_resistance: Option<f64>,
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

        /// Input audio file (WAV). If omitted, generates a 1kHz sine wave.
        #[arg(short = 'a', long)]
        input_audio: Option<PathBuf>,

        /// Output audio file (WAV)
        #[arg(short, long)]
        output: PathBuf,

        /// Sample rate in Hz (used when no input audio provided)
        #[arg(short, long, default_value = "48000")]
        sample_rate: f64,

        /// Input node name
        #[arg(short = 'I', long, default_value = "in")]
        input_node: String,

        /// Output node name
        #[arg(short = 'n', long, default_value = "out")]
        output_node: String,

        /// Duration in seconds (used when no input audio provided)
        #[arg(short, long, default_value = "1.0")]
        duration: f64,

        /// Input signal amplitude (0.0 to 1.0)
        #[arg(long, default_value = "0.5")]
        amplitude: f64,

        /// Override input resistance (ohms). Default: 1Ω, or from .input_impedance directive.
        #[arg(long)]
        input_resistance: Option<f64>,
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
            output_scale,
            format,
            with_level_params,
            no_level_params,
            input_resistance: input_resistance_flag,
        } => {
            // Validate numeric CLI parameters
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!("Sample rate must be positive and finite, got {}", sample_rate);
            }
            if tolerance <= 0.0 || !tolerance.is_finite() {
                anyhow::bail!("Tolerance must be positive and finite, got {}", tolerance);
            }
            if max_iter == 0 {
                anyhow::bail!("max-iter must be at least 1, got 0");
            }

            // Level params are on by default; --no-level-params disables them
            let level_params = with_level_params && !no_level_params;

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
                output_scale,
                format,
                level_params,
                input_resistance_flag,
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
            input_node,
            output_node,
            duration,
            amplitude,
            input_resistance: input_resistance_flag,
        } => {
            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            simulate_circuit_source(
                &circuit_source,
                &SimulateOptions {
                    input_audio: input_audio.as_deref(),
                    output: &output,
                    sample_rate,
                    input_node: &input_node,
                    output_node: &output_node,
                    duration,
                    amplitude,
                    input_resistance_flag,
                },
            )
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

#[allow(clippy::too_many_arguments)]
fn compile_circuit_source(
    circuit_source: &circuits::CircuitSource,
    output: &PathBuf,
    sample_rate: f64,
    input_node: &str,
    output_node: &str,
    max_iter: usize,
    tolerance: f64,
    output_scale: f64,
    format: OutputFormat,
    with_level_params: bool,
    input_resistance_flag: Option<f64>,
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
    let mut netlist = Netlist::parse(&netlist_str)
        .with_context(|| "Failed to parse SPICE netlist")?;

    // Expand subcircuit instances (X elements) before MNA
    if !netlist.subcircuits.is_empty() {
        let num_subcircuits = netlist.subcircuits.len();
        netlist.expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
        println!("  ✓ Expanded {} subcircuit definition(s)", num_subcircuits);
    }

    println!("  ✓ Parsed {} elements", netlist.elements.len());

    // Step 2: Build MNA system
    println!("Step 2: Building MNA system...");
    let mut mna = MnaSystem::from_netlist(&netlist)
        .with_context(|| "Failed to build MNA system")?;

    println!("  ✓ {} nodes, {} nonlinear devices", mna.n, mna.nonlinear_devices.len());

    // Get input node index and add input conductance to G matrix
    // This models the source impedance of the input voltage source
    let input_node_raw = mna
        .node_map
        .get(input_node)
        .copied()
        .ok_or_else(|| anyhow::anyhow!(
            "Input node '{}' not found in circuit. Available: {:?}",
            input_node, mna.node_map.keys().collect::<Vec<_>>()
        ))?;
    if input_node_raw == 0 {
        anyhow::bail!("Input node cannot be ground (0). Please specify a non-ground node.");
    }
    let input_node_idx = input_node_raw - 1;
    // Resolve input resistance: CLI flag > .input_impedance directive > default 1Ω
    let (input_resistance, ir_source) = if let Some(r) = input_resistance_flag {
        (r, "from --input-resistance flag")
    } else if let Some(r) = netlist.input_impedance {
        (r, "from .input_impedance directive")
    } else {
        (1.0, "default")
    };
    println!("  Input resistance: {} ohm ({})", input_resistance, ir_source);
    let input_conductance = 1.0 / input_resistance;
    if input_node_idx < mna.n {
        mna.g[input_node_idx][input_node_idx] += input_conductance;
    }

    // Warn if passive EQ topology detected with low source impedance
    if input_resistance < 10.0 && !mna.pots.is_empty() {
        // Check if any pot is connected to the input node
        let mut connected_pots = Vec::new();
        for pot in &mna.pots {
            // Direct connection: pot node matches input node (both 1-indexed)
            let direct = pot.node_p == input_node_raw || pot.node_q == input_node_raw;
            // One-hop: connected through another component (check G matrix)
            let pot_p_0 = if pot.node_p > 0 { pot.node_p - 1 } else { usize::MAX };
            let pot_q_0 = if pot.node_q > 0 { pot.node_q - 1 } else { usize::MAX };
            let one_hop = (pot_p_0 < mna.n && mna.g[input_node_idx][pot_p_0] != 0.0)
                || (pot_q_0 < mna.n && mna.g[input_node_idx][pot_q_0] != 0.0);
            if direct || one_hop {
                connected_pots.push(pot.name.clone());
            }
        }
        if !connected_pots.is_empty() {
            eprintln!();
            eprintln!("  WARNING: Passive EQ topology detected with {:.0}Ω source impedance.", input_resistance);
            eprintln!("  Pots connected to input: {}", connected_pots.join(", "));
            eprintln!("  A low source impedance overwhelms passive EQ networks, making pots inert.");
            eprintln!("  Consider adding to your netlist:  .input_impedance 600");
            eprintln!("  Or use the CLI flag:              --input-resistance 600");
            eprintln!();
        }
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

    // Parse comma-separated output nodes
    let output_node_names: Vec<&str> = output_node.split(',').map(|s| s.trim()).collect();
    let mut output_node_indices = Vec::new();
    for name in &output_node_names {
        let raw = mna
            .node_map
            .get(*name)
            .copied()
            .ok_or_else(|| anyhow::anyhow!(
                "Output node '{}' not found in circuit. Available: {:?}",
                name, mna.node_map.keys().collect::<Vec<_>>()
            ))?;
        if raw == 0 {
            anyhow::bail!("Output node '{}' cannot be ground (0). Please specify a non-ground node.", name);
        }
        output_node_indices.push(raw - 1);
    }

    // Broadcast single output_scale to all outputs
    let output_scales = vec![output_scale; output_node_indices.len()];

    let config = CodegenConfig {
        circuit_name,
        input_node: input_node_idx,
        output_nodes: output_node_indices.clone(),
        sample_rate,
        max_iterations: max_iter,
        tolerance,
        output_scales,
        include_dc_op: true,
        input_resistance,
        ..CodegenConfig::default()
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
            println!("  melange compile {} --output {} --format plugin", circuit_source.name(), output.display());
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

            // Build pot parameter info from kernel
            let pot_params: Vec<plugin_template::PotParamInfo> = kernel.pots.iter().enumerate().map(|(idx, p)| {
                plugin_template::PotParamInfo {
                    index: idx,
                    name: netlist.pots.get(idx).map(|d| {
                        d.label.clone().unwrap_or_else(|| d.resistor_name.clone())
                    }).unwrap_or_else(|| format!("Pot {}", idx)),
                    min_resistance: p.min_resistance,
                    max_resistance: p.max_resistance,
                    default_resistance: 1.0 / p.g_nominal,
                }
            }).collect();

            // Build switch parameter info from netlist
            let switch_params: Vec<plugin_template::SwitchParamInfo> = netlist.switches.iter().enumerate().map(|(idx, sw)| {
                plugin_template::SwitchParamInfo {
                    index: idx,
                    name: sw.label.clone().unwrap_or_else(|| format!("Switch {} ({})", idx, sw.component_names.join("+"))),
                    num_positions: sw.positions.len(),
                }
            }).collect();

            plugin_template::generate_plugin_project(&project_dir, &generated.code, &circuit_name, with_level_params, &pot_params, &switch_params, output_node_indices.len())?;

            println!("  ✓ Done!");
            println!();
            println!("Generated plugin project at: {}", project_dir.display());
            println!();
            println!("To build the plugin (CLAP + VST3):");
            println!("  cd {}", project_dir.file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("<project-dir>"));
            println!("  cargo build --release");
            println!();
            println!("The compiled plugin will be at:");
            println!("  target/release/lib{}.so", circuit_name.replace("-", "_"));
            println!();
            println!("To install the CLAP plugin:");
            println!("  mkdir -p ~/.clap");
            println!("  cp target/release/lib{}.so ~/.clap/{}.clap", 
                     circuit_name.replace("-", "_"), circuit_name);
            println!();
            println!("The VST3 plugin is included in the same binary.");
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

struct SimulateOptions<'a> {
    input_audio: Option<&'a std::path::Path>,
    output: &'a PathBuf,
    sample_rate: f64,
    input_node: &'a str,
    output_node: &'a str,
    duration: f64,
    amplitude: f64,
    input_resistance_flag: Option<f64>,
}

fn simulate_circuit_source(
    circuit_source: &circuits::CircuitSource,
    opts: &SimulateOptions,
) -> Result<()> {
    use melange_solver::{
        dk::DkKernel,
        mna::MnaSystem,
        parser::Netlist,
        solver::{CircuitSolver, DeviceEntry},
    };
    use melange_devices::bjt::{BjtEbersMoll, BjtPolarity};
    use melange_devices::diode::DiodeShockley;

    println!("melange simulate");
    println!("  Source: {}", circuit_source.name());
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
    let mut netlist = Netlist::parse(&netlist_str)
        .with_context(|| "Failed to parse SPICE netlist")?;

    // Expand subcircuit instances (X elements) before MNA
    if !netlist.subcircuits.is_empty() {
        let num_subcircuits = netlist.subcircuits.len();
        netlist.expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
        println!("  Expanded {} subcircuit definition(s)", num_subcircuits);
    }
    println!("  Parsed {} elements", netlist.elements.len());

    // Step 2: Build MNA
    println!("Step 2: Building MNA system...");
    let mut mna = MnaSystem::from_netlist(&netlist)
        .with_context(|| "Failed to build MNA system")?;

    let input_node_raw = mna.node_map.get(opts.input_node).copied()
        .ok_or_else(|| anyhow::anyhow!(
            "Input node '{}' not found. Available: {:?}",
            opts.input_node, mna.node_map.keys().collect::<Vec<_>>()
        ))?;
    if input_node_raw == 0 {
        anyhow::bail!("Input node cannot be ground (0)");
    }
    let input_node_idx = input_node_raw - 1;

    let output_node_raw = mna.node_map.get(opts.output_node).copied()
        .ok_or_else(|| anyhow::anyhow!(
            "Output node '{}' not found. Available: {:?}",
            opts.output_node, mna.node_map.keys().collect::<Vec<_>>()
        ))?;
    if output_node_raw == 0 {
        anyhow::bail!("Output node cannot be ground (0)");
    }
    let output_node_idx = output_node_raw - 1;

    // Resolve input resistance: CLI flag > .input_impedance directive > default 1Ω
    let (input_resistance, ir_source) = if let Some(r) = opts.input_resistance_flag {
        (r, "from --input-resistance flag")
    } else if let Some(r) = netlist.input_impedance {
        (r, "from .input_impedance directive")
    } else {
        (1.0, "default")
    };
    println!("  Input resistance: {} ohm ({})", input_resistance, ir_source);
    let input_conductance = 1.0 / input_resistance;
    if input_node_idx < mna.n {
        mna.g[input_node_idx][input_node_idx] += input_conductance;
    }

    println!("  {} nodes, {} nonlinear devices", mna.n, mna.nonlinear_devices.len());

    // Step 3: Read input audio or generate test signal
    let (samples, actual_sample_rate) = if let Some(audio_path) = opts.input_audio {
        println!("Step 3: Reading input audio: {}", audio_path.display());
        let reader = hound::WavReader::open(audio_path)
            .with_context(|| format!("Failed to open WAV file: {}", audio_path.display()))?;
        let spec = reader.spec();
        let sr = spec.sample_rate as f64;
        let input_samples: Vec<f64> = match spec.sample_format {
            hound::SampleFormat::Int => {
                let max_val = (1i64 << (spec.bits_per_sample - 1)) as f64;
                reader.into_samples::<i32>()
                    .map(|s| s.unwrap() as f64 / max_val)
                    .collect()
            }
            hound::SampleFormat::Float => {
                reader.into_samples::<f32>()
                    .map(|s| s.unwrap() as f64)
                    .collect()
            }
        };
        println!("  {} samples at {} Hz ({:.2}s)",
            input_samples.len(), sr, input_samples.len() as f64 / sr);
        (input_samples, sr)
    } else {
        println!("Step 3: Generating 1kHz sine wave ({:.1}s at {} Hz)...", opts.duration, opts.sample_rate);
        let num_samples = (opts.duration * opts.sample_rate) as usize;
        let samples: Vec<f64> = (0..num_samples)
            .map(|i| opts.amplitude * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / opts.sample_rate).sin())
            .collect();
        (samples, opts.sample_rate)
    };

    // Step 4: Build DK kernel
    println!("Step 4: Building DK kernel at {} Hz...", actual_sample_rate);
    let kernel = DkKernel::from_mna(&mna, actual_sample_rate)
        .with_context(|| "Failed to create DK kernel")?;
    println!("  N={}, M={}", kernel.n, kernel.m);

    // Step 5: Build solver
    println!("Step 5: Creating solver...");

    // Helper: find model param
    let find_model_param = |model_name: &str, param: &str| -> Option<f64> {
        netlist.models.iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| m.params.iter()
                .find(|(k, _)| k.eq_ignore_ascii_case(param))
                .map(|(_, v)| *v))
    };

    // Build device entries from MNA nonlinear device list
    let mut devices = Vec::new();
    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                // Find model name for this diode
                let model_name = netlist.elements.iter().find_map(|e| {
                    if let melange_solver::parser::Element::Diode { name, model, .. } = e {
                        if name.eq_ignore_ascii_case(&dev_info.name) { Some(model.as_str()) } else { None }
                    } else { None }
                }).unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-15);
                let n = find_model_param(model_name, "N").unwrap_or(1.0);
                let diode = DiodeShockley::new_room_temp(is, n);
                devices.push(DeviceEntry::new_diode(diode, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                let model_name = netlist.elements.iter().find_map(|e| {
                    if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                        if name.eq_ignore_ascii_case(&dev_info.name) { Some(model.as_str()) } else { None }
                    } else { None }
                }).unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-14);
                let bf = find_model_param(model_name, "BF").unwrap_or(200.0);
                let br = find_model_param(model_name, "BR").unwrap_or(3.0);
                // Check for PNP
                let is_pnp = netlist.models.iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                let polarity = if is_pnp { BjtPolarity::Pnp } else { BjtPolarity::Npn };
                let bjt = BjtEbersMoll::new(is, 0.02585, bf, br, polarity);
                devices.push(DeviceEntry::new_bjt(bjt, dev_info.start_idx));
            }
            _ => {
                println!("  Warning: unsupported device type for '{}', skipping", dev_info.name);
            }
        }
    }

    let has_nonlinear = !devices.is_empty();

    if kernel.m == 0 {
        // Linear circuit: use LinearSolver
        let mut solver = melange_solver::solver::LinearSolver::new(
            kernel, input_node_idx, output_node_idx,
        );
        solver.input_conductance = input_conductance;

        println!("  Linear solver ready");
        println!();

        // Process
        println!("Step 6: Processing {} samples...", samples.len());
        let mut output_samples = Vec::with_capacity(samples.len());
        for &s in &samples {
            output_samples.push(solver.process_sample(s));
        }

        eprintln!("  Peak output: {:.2}V", solver.diag_peak_output);
        if solver.diag_clamp_count > 0 {
            eprintln!("  WARNING: Output exceeded ±10V {} times -- try reducing --amplitude or adding gain reduction to the circuit", solver.diag_clamp_count);
        }

        write_wav(opts.output, actual_sample_rate, &output_samples)?;
    } else {
        // Nonlinear circuit: use CircuitSolver
        let mut solver = CircuitSolver::new(
            kernel, devices, input_node_idx, output_node_idx,
        );
        solver.input_conductance = input_conductance;

        // Initialize DC operating point for nonlinear circuits
        if has_nonlinear {
            println!("  Initializing DC operating point...");
            // Build device slots for DC OP (same as codegen IR uses)
            let device_slots = build_device_slots(&netlist, &mna);
            let converged = solver.initialize_dc_op(&mna, &device_slots);
            if !converged {
                eprintln!("  Warning: DC operating point did not fully converge");
            }
            println!("  DC OP initialized");
        }

        println!("  Nonlinear solver ready (M={})", mna.nonlinear_devices.iter().map(|d| d.dimension).sum::<usize>());
        println!();

        // Process
        println!("Step 6: Processing {} samples...", samples.len());
        let mut output_samples = Vec::with_capacity(samples.len());
        for &s in &samples {
            output_samples.push(solver.process_sample(s));
        }

        eprintln!("  Peak output: {:.2}V", solver.diag_peak_output);
        if solver.diag_clamp_count > 0 {
            eprintln!("  WARNING: Output exceeded ±10V {} times -- try reducing --amplitude or adding gain reduction to the circuit", solver.diag_clamp_count);
        }
        if solver.diag_nr_max_iter_count > 0 {
            eprintln!("  NR max iterations: {} times", solver.diag_nr_max_iter_count);
        }
        if solver.diag_nan_reset_count > 0 {
            eprintln!("  NaN resets: {}", solver.diag_nan_reset_count);
        }

        write_wav(opts.output, actual_sample_rate, &output_samples)?;
    }

    Ok(())
}

fn build_device_slots(
    netlist: &melange_solver::parser::Netlist,
    mna: &melange_solver::mna::MnaSystem,
) -> Vec<melange_solver::codegen::ir::DeviceSlot> {
    use melange_solver::codegen::ir::{DeviceSlot, DeviceType, DeviceParams, DiodeParams, BjtParams};

    let find_param = |model_name: &str, param: &str| -> Option<f64> {
        netlist.models.iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| m.params.iter()
                .find(|(k, _)| k.eq_ignore_ascii_case(param))
                .map(|(_, v)| *v))
    };

    let mut slots = Vec::new();
    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                let model_name = netlist.elements.iter().find_map(|e| {
                    if let melange_solver::parser::Element::Diode { name, model, .. } = e {
                        if name.eq_ignore_ascii_case(&dev_info.name) { Some(model.clone()) } else { None }
                    } else { None }
                }).unwrap_or_default();
                let is = find_param(&model_name, "IS").unwrap_or(1e-15);
                let n = find_param(&model_name, "N").unwrap_or(1.0);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Diode,
                    start_idx: dev_info.start_idx,
                    dimension: 1,
                    params: DeviceParams::Diode(DiodeParams { is, n_vt: n * 0.02585 }),
                });
            }
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                let model_name = netlist.elements.iter().find_map(|e| {
                    if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                        if name.eq_ignore_ascii_case(&dev_info.name) { Some(model.clone()) } else { None }
                    } else { None }
                }).unwrap_or_default();
                let is = find_param(&model_name, "IS").unwrap_or(1e-14);
                let bf = find_param(&model_name, "BF").unwrap_or(200.0);
                let br = find_param(&model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist.models.iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Bjt,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Bjt(BjtParams {
                        is,
                        vt: 0.02585,
                        beta_f: bf,
                        beta_r: br,
                        is_pnp,
                        vaf: f64::INFINITY,
                        var: f64::INFINITY,
                        ikf: f64::INFINITY,
                        ikr: f64::INFINITY,
                    }),
                });
            }
            _ => {}
        }
    }
    slots
}

fn write_wav(output: &PathBuf, sample_rate: f64, samples: &[f64]) -> Result<()> {
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: sample_rate as u32,
        bits_per_sample: 32,
        sample_format: hound::SampleFormat::Float,
    };
    let mut writer = hound::WavWriter::create(output, spec)
        .with_context(|| format!("Failed to create WAV file: {}", output.display()))?;

    let mut peak = 0.0f64;
    for &s in samples {
        peak = peak.max(s.abs());
        writer.write_sample(s as f32)
            .with_context(|| "Failed to write WAV sample")?;
    }
    writer.finalize()
        .with_context(|| "Failed to finalize WAV file")?;

    println!();
    println!("Output written to: {}", output.display());
    println!("  Samples: {}", samples.len());
    println!("  Duration: {:.2}s", samples.len() as f64 / sample_rate);
    println!("  Peak level: {:.4} ({:.1} dB)",
        peak, if peak > 0.0 { 20.0 * peak.log10() } else { f64::NEG_INFINITY });

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

    let mut netlist = Netlist::parse(&netlist_str)
        .with_context(|| "Failed to parse SPICE netlist")?;

    // Expand subcircuit instances (X elements) before MNA
    if !netlist.subcircuits.is_empty() {
        netlist.expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
    }

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
