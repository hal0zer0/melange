//! melange-cli - Command line tool for circuit modeling
//!
//! Usage:
//!   melange compile input.cir --output circuit.rs
//!   melange validate input.cir --output-node out
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
        #[arg(long, default_value = "50")]
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

        /// Disable DC blocking filter on outputs. Use for circuits with output coupling
        /// caps or when downstream handles DC offset. Removes the 5Hz HPF settling time.
        #[arg(long)]
        no_dc_block: bool,

        /// Override input resistance (ohms). Default: 1Ω, or from .input_impedance directive.
        #[arg(long)]
        input_resistance: Option<f64>,

        /// Oversampling factor (1=none, 2=2x, 4=4x). Higher reduces aliasing and improves NR stability.
        #[arg(long, default_value = "1")]
        oversampling: usize,

        /// Solver type: auto (default), dk, nodal.
        /// Auto selects DK for most circuits, nodal for multi-transformer.
        /// Use nodal for large M circuits where DK NR doesn't converge.
        #[arg(long, default_value = "auto")]
        solver: String,

        /// Use backward Euler integration instead of trapezoidal.
        /// Unconditionally stable — fixes divergence in high-gain feedback amplifiers.
        /// Trades second-order accuracy for first-order (slight HF rolloff).
        #[arg(long)]
        backward_euler: bool,

        /// Plugin display name (defaults to capitalized circuit filename)
        #[arg(long)]
        name: Option<String>,

        /// Generate mono (1-channel) plugin instead of stereo
        #[arg(long)]
        mono: bool,

        /// Add wet/dry mix parameter to generated plugin
        #[arg(long)]
        wet_dry_mix: bool,

        /// Disable ear-protection output limiter. The limiter is a transparent soft
        /// clipper that engages near 0 dBFS to protect speakers and hearing.
        /// On by default — use this flag only for measurement or testing.
        #[arg(long)]
        no_ear_protection: bool,
    },

    /// Validate circuit against ngspice reference simulation
    ///
    /// Runs the melange solver and ngspice on the same circuit with a test input
    /// signal, then compares the outputs. Requires ngspice to be installed.
    Validate {
        /// Input SPICE netlist file or circuit reference
        /// (builtin:circuit, source:circuit, URL, or local path)
        input: String,

        /// Output node name to compare
        #[arg(short = 'n', long, default_value = "out")]
        output_node: String,

        /// Sample rate in Hz
        #[arg(short, long, default_value = "48000")]
        sample_rate: f64,

        /// Test signal duration in seconds
        #[arg(long, default_value = "1.0")]
        duration: f64,

        /// Test signal amplitude (volts)
        #[arg(long, default_value = "0.1")]
        amplitude: f64,

        /// Input node name (for informational display)
        #[arg(short = 'I', long, default_value = "in")]
        input_node: String,

        /// Write comparison data to CSV file
        #[arg(long)]
        csv: Option<PathBuf>,

        /// Use relaxed tolerances (1% RMS, 0.999 correlation)
        #[arg(long)]
        relaxed: bool,
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

        /// Solver type: auto (default), dk (DK method), nodal (full-nodal NR).
        /// Auto selects nodal for nonlinear circuits with inductors, dk otherwise.
        #[arg(long, default_value = "auto")]
        solver: String,
    },

    /// Analyze circuit frequency response
    Analyze {
        /// Input SPICE netlist file or circuit reference
        input: String,

        /// Input node name
        #[arg(short = 'I', long, default_value = "in")]
        input_node: String,

        /// Output node name
        #[arg(short = 'n', long, default_value = "out")]
        output_node: String,

        /// Start frequency in Hz
        #[arg(long, default_value = "20.0")]
        start_freq: f64,

        /// End frequency in Hz
        #[arg(long, default_value = "20000.0")]
        end_freq: f64,

        /// Frequency points per decade
        #[arg(long, default_value = "10")]
        points_per_decade: usize,

        /// Input amplitude in volts
        #[arg(long, default_value = "0.1")]
        amplitude: f64,

        /// Sample rate in Hz
        #[arg(short, long, default_value = "96000")]
        sample_rate: f64,

        /// Override input resistance (ohms)
        #[arg(long)]
        input_resistance: Option<f64>,

        /// Write CSV output to file instead of stdout
        #[arg(short, long)]
        output: Option<PathBuf>,

        /// Set pot value: "Label=value" or "Rname=value" (e.g. "LF Boost=10k")
        #[arg(long = "pot", value_name = "NAME=VALUE")]
        pot_overrides: Vec<String>,

        /// Set switch position: "Label=pos" or index=pos (0-indexed, e.g. "LF Freq=3")
        #[arg(long = "switch", value_name = "NAME=POS")]
        switch_overrides: Vec<String>,
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
            no_dc_block,
            input_resistance: input_resistance_flag,
            oversampling,
            solver,
            backward_euler,
            name,
            mono,
            wet_dry_mix,
            no_ear_protection,
        } => {
            // Validate numeric CLI parameters
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!(
                    "Sample rate must be positive and finite, got {}",
                    sample_rate
                );
            }
            if tolerance <= 0.0 || !tolerance.is_finite() {
                anyhow::bail!("Tolerance must be positive and finite, got {}", tolerance);
            }
            if max_iter == 0 {
                anyhow::bail!("max-iter must be at least 1, got 0");
            }
            if oversampling != 1 && oversampling != 2 && oversampling != 4 {
                anyhow::bail!("oversampling must be 1, 2, or 4, got {}", oversampling);
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
                oversampling,
                no_dc_block,
                &solver,
                backward_euler,
                name.as_deref(),
                mono,
                wet_dry_mix,
                !no_ear_protection,
            )
        }
        Commands::Validate {
            input,
            output_node,
            sample_rate,
            duration,
            amplitude,
            input_node,
            csv,
            relaxed,
        } => {
            // Validate numeric CLI parameters
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!("sample-rate must be positive and finite");
            }
            if duration <= 0.0 || !duration.is_finite() {
                anyhow::bail!("duration must be positive and finite");
            }
            if amplitude <= 0.0 || !amplitude.is_finite() {
                anyhow::bail!("amplitude must be positive and finite");
            }

            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            validate_circuit_source(
                &circuit_source,
                &output_node,
                sample_rate,
                duration,
                amplitude,
                &input_node,
                csv.as_ref(),
                relaxed,
            )
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
            solver,
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
                    solver: &solver,
                },
            )
        }
        Commands::Analyze {
            input,
            input_node,
            output_node,
            start_freq,
            end_freq,
            points_per_decade,
            amplitude,
            sample_rate,
            input_resistance,
            output,
            pot_overrides,
            switch_overrides,
        } => {
            // Validate numeric CLI parameters
            if start_freq <= 0.0 || !start_freq.is_finite() {
                anyhow::bail!("start-freq must be positive and finite");
            }
            if end_freq <= start_freq || !end_freq.is_finite() {
                anyhow::bail!("end-freq must be greater than start-freq and finite");
            }
            if amplitude <= 0.0 || !amplitude.is_finite() {
                anyhow::bail!("amplitude must be positive and finite");
            }
            if sample_rate <= 0.0 || !sample_rate.is_finite() {
                anyhow::bail!("sample-rate must be positive and finite");
            }
            if points_per_decade == 0 {
                anyhow::bail!("points-per-decade must be at least 1");
            }

            let circuit_source = circuits::resolve(&input)?;
            analyze_freq_response(
                &circuit_source,
                &input_node,
                &output_node,
                start_freq,
                end_freq,
                points_per_decade,
                amplitude,
                sample_rate,
                input_resistance,
                output.as_ref(),
                &pot_overrides,
                &switch_overrides,
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
    oversampling: usize,
    no_dc_block: bool,
    solver_override: &str,
    backward_euler: bool,
    plugin_name: Option<&str>,
    mono: bool,
    wet_dry_mix: bool,
    ear_protection: bool,
) -> Result<()> {
    use melange_solver::{
        codegen::{CodeGenerator, CodegenConfig},
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
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read local file: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            println!("  Fetching from URL: {}", url);
            let cache = cache::Cache::new()?;
            cache.get_sync(url, false)?
        }
    };

    // Step 1: Parse netlist
    println!("Step 1: Parsing SPICE netlist...");
    let mut netlist =
        Netlist::parse(&netlist_str).with_context(|| "Failed to parse SPICE netlist")?;

    // Expand subcircuit instances (X elements) before MNA
    if !netlist.subcircuits.is_empty() {
        let num_subcircuits = netlist.subcircuits.len();
        netlist
            .expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
        println!("  ✓ Expanded {} subcircuit definition(s)", num_subcircuits);
    }

    println!("  ✓ Parsed {} elements", netlist.elements.len());

    // Step 2: Build MNA system
    println!("Step 2: Building MNA system...");
    let mut mna =
        MnaSystem::from_netlist(&netlist).with_context(|| "Failed to build MNA system")?;

    println!(
        "  ✓ {} nodes, {} nonlinear devices",
        mna.n,
        mna.nonlinear_devices.len()
    );

    // Get input node index and add input conductance to G matrix
    // This models the source impedance of the input voltage source
    let input_node_raw = mna.node_map.get(input_node).copied().ok_or_else(|| {
        anyhow::anyhow!(
            "Input node '{}' not found in circuit. Available: {:?}",
            input_node,
            mna.node_map.keys().collect::<Vec<_>>()
        )
    })?;
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
    println!(
        "  Input resistance: {} ohm ({})",
        input_resistance, ir_source
    );
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
            let pot_p_0 = if pot.node_p > 0 {
                pot.node_p - 1
            } else {
                usize::MAX
            };
            let pot_q_0 = if pot.node_q > 0 {
                pot.node_q - 1
            } else {
                usize::MAX
            };
            let one_hop = (pot_p_0 < mna.n && mna.g[input_node_idx][pot_p_0] != 0.0)
                || (pot_q_0 < mna.n && mna.g[input_node_idx][pot_q_0] != 0.0);
            if direct || one_hop {
                connected_pots.push(pot.name.clone());
            }
        }
        if !connected_pots.is_empty() {
            eprintln!();
            eprintln!(
                "  WARNING: Passive EQ topology detected with {:.0}Ω source impedance.",
                input_resistance
            );
            eprintln!("  Pots connected to input: {}", connected_pots.join(", "));
            eprintln!(
                "  A low source impedance overwhelms passive EQ networks, making pots inert."
            );
            eprintln!("  Consider adding to your netlist:  .input_impedance 600");
            eprintln!("  Or use the CLI flag:              --input-resistance 600");
            eprintln!();
        }
    }

    // Stamp junction capacitances BEFORE FA detection (caps affect DC OP).
    // Internal node expansion happens AFTER FA detection to avoid disrupting it.
    {
        let device_slots =
            melange_solver::codegen::ir::CircuitIR::build_device_info(&netlist).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    // Detect forward-active BJTs (runs DC OP with 2D model, checks Vbc < -1V)
    let fa_config = melange_solver::codegen::CodegenConfig {
        input_node: input_node_idx,
        input_resistance,
        ..melange_solver::codegen::CodegenConfig::default()
    };
    // Skip FA detection for nodal solver (it handles all M natively via N×N NR).
    // FA only benefits the DK solver where M×M NR cost dominates.
    // Skip FA detection for nodal solver — the nodal-Schur path handles all M natively
    // and the FA+nodal DC OP interaction has indexing issues that need separate fixing.
    let forward_active = if solver_override == "nodal" {
        std::collections::HashSet::new()
    } else {
        melange_solver::codegen::ir::CircuitIR::detect_forward_active_bjts(
            &mna, &netlist, &fa_config,
        )
    };
    if !forward_active.is_empty() {
        println!(
            "  Forward-active BJTs: {:?} (M reduces by {})",
            forward_active,
            forward_active.len()
        );
        // Rebuild MNA with reduced dimensions
        mna = MnaSystem::from_netlist_forward_active(&netlist, &forward_active)
            .with_context(|| "Failed to rebuild MNA for forward-active BJTs")?;
        // Re-stamp input conductance
        if input_node_idx < mna.n {
            mna.g[input_node_idx][input_node_idx] += input_conductance;
        }
        // Re-stamp junction capacitances on rebuilt MNA
        {
            let device_slots = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
                &netlist,
                Some(&mna),
            )
            .unwrap_or_default();
            if !device_slots.is_empty() {
                mna.stamp_device_junction_caps(&device_slots);
            }
        }
    }

    // Linearize BJTs specified by .linearize directives
    let linearized: std::collections::HashSet<String> =
        netlist.linearize_devices.iter().cloned().collect();
    if !linearized.is_empty() {
        // Compute DC OP on current MNA to get g-parameters for linearized BJTs
        let device_slots = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
            &netlist,
            Some(&mna),
        )
        .unwrap_or_default();
        let dc_op_config = melange_solver::dc_op::DcOpConfig {
            input_node: input_node_idx,
            input_resistance,
            ..melange_solver::dc_op::DcOpConfig::default()
        };
        let dc_result =
            melange_solver::dc_op::solve_dc_operating_point(&mna, &device_slots, &dc_op_config);

        // Extract g-parameters from DC OP Jacobian for each linearized BJT
        let mut lin_infos = Vec::new();
        for slot in &device_slots {
            if let melange_solver::codegen::ir::DeviceParams::Bjt(bp) = &slot.params {
                // Find the device name from MNA nonlinear_devices
                let dev = mna
                    .nonlinear_devices
                    .iter()
                    .find(|d| d.start_idx == slot.start_idx);
                if let Some(dev) = dev {
                    if linearized.contains(&dev.name.to_ascii_uppercase()) {
                        let s = slot.start_idx;
                        // Extract v_nl at DC OP
                        let vbe = dc_result.v_nl.get(s).copied().unwrap_or(0.0);
                        let vbc = dc_result.v_nl.get(s + 1).copied().unwrap_or(0.0);
                        let ic = dc_result.i_nl.get(s).copied().unwrap_or(0.0);
                        let ib = dc_result.i_nl.get(s + 1).copied().unwrap_or(0.0);

                        // Compute Jacobian at DC OP
                        let sign = if bp.is_pnp { -1.0 } else { 1.0 };
                        let vbe_eff = sign * vbe;
                        let vbc_eff = sign * vbc;
                        let nf_vt = bp.nf * bp.vt;
                        let exp_be = (vbe_eff / nf_vt).clamp(-40.0, 40.0).exp();
                        let exp_bc = (vbc_eff / bp.vt).clamp(-40.0, 40.0).exp();

                        // dIc/dVbe (transconductance)
                        let gm = bp.is / nf_vt * exp_be;
                        // dIc/dVbc
                        let gmu =
                            (bp.is / bp.vt * exp_bc + bp.is / (bp.beta_r * bp.vt) * exp_bc).abs();
                        // dIb/dVbe
                        let gpi = bp.is / (bp.beta_f * nf_vt) * exp_be;
                        // dIb/dVbc
                        let go = bp.is / (bp.beta_r * bp.vt) * exp_bc;

                        let (nc, nb, ne) = (
                            dev.node_indices[0],
                            dev.node_indices[1],
                            dev.node_indices[2],
                        );
                        println!(
                            "  Linearized {}: gm={:.4e} gpi={:.4e} gmu={:.4e} Ic_dc={:.4e} Ib_dc={:.4e}",
                            dev.name, gm, gpi, gmu, ic, ib
                        );
                        lin_infos.push(melange_solver::mna::LinearizedBjtInfo {
                            name: dev.name.clone(),
                            nc,
                            nb,
                            ne,
                            gm,
                            gpi,
                            gmu,
                            go,
                            ic_dc: ic,
                            ib_dc: ib,
                        });
                    }
                }
            }
        }

        // Rebuild MNA with forward-active + linearized
        mna = MnaSystem::from_netlist_with_linearized(&netlist, &forward_active, &linearized)
            .with_context(|| "Failed to rebuild MNA with linearized BJTs")?;
        if input_node_idx < mna.n {
            mna.g[input_node_idx][input_node_idx] += input_conductance;
        }
        // Re-stamp junction caps
        {
            let ds = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
                &netlist,
                Some(&mna),
            )
            .unwrap_or_default();
            if !ds.is_empty() {
                mna.stamp_device_junction_caps(&ds);
            }
        }
        // Stamp linearized g-parameters into G
        mna.linearized_bjts = lin_infos;
        mna.stamp_linearized_bjts();
        println!(
            "  Linearized {} BJTs (M reduced by {})",
            linearized.len(),
            linearized.len() * 2
        );
    }

    // NOTE: Internal node expansion for parasitic BJTs is deferred until after
    // solver routing. The DK path handles parasitics via bjt_with_parasitics() inner NR.
    // Only the nodal path benefits from MNA-level internal nodes (eliminates inner NR).

    // Step 3: Create DK kernel
    // Use augmented MNA for inductor circuits (well-conditioned for large L)
    let has_inductors_compile = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();
    println!("Step 3: Creating DK kernel...");
    let kernel_result = if has_inductors_compile {
        println!("  Using augmented MNA for inductors");
        DkKernel::from_mna_augmented(&mna, sample_rate)
    } else {
        DkKernel::from_mna(&mna, sample_rate)
    };
    // If DK kernel fails (e.g., positive feedback / oscillator circuit),
    // auto-fall back to nodal solver which has no K diagonal constraint.
    let (kernel, dk_failed) = match kernel_result {
        Ok(k) => {
            println!("  ✓ Matrix dimensions: N={}, M={}", k.n, k.m);
            (k, false)
        }
        Err(ref e) => {
            if solver_override == "dk" {
                // User explicitly requested DK — propagate the error
                kernel_result.with_context(|| "Failed to create DK kernel")?;
                unreachable!()
            } else {
                println!("  DK kernel failed: {}", e);
                println!("  Auto-selecting nodal solver (handles positive feedback / oscillators)");
                // Build a dummy kernel for dimension info — nodal path doesn't use it
                let m = mna.m;
                let n = mna.n_aug;
                let dummy = DkKernel {
                    n,
                    m,
                    n_nodes: mna.n,
                    num_devices: mna.num_devices,
                    sample_rate,
                    s: vec![0.0; n * n],
                    a_neg: vec![0.0; n * n],
                    k: vec![0.0; m * m],
                    n_v: vec![0.0; m * n],
                    n_i: vec![0.0; n * m],
                    rhs_const: vec![0.0; n],
                    inductors: vec![],
                    coupled_inductors: vec![],
                    transformer_groups: vec![],
                    pots: vec![],
                };
                (dummy, true)
            }
        }
    };

    // Step 4: Generate code
    println!("Step 4: Generating Rust code...");

    let circuit_name = output
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("circuit")
        .to_string();
    let circuit_name: String = circuit_name
        .to_lowercase()
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    let circuit_name = if circuit_name.starts_with(|c: char| c.is_ascii_digit()) {
        format!("circuit_{circuit_name}")
    } else {
        circuit_name
    };

    // Parse comma-separated output nodes
    let output_node_names: Vec<&str> = output_node.split(',').map(|s| s.trim()).collect();
    let mut output_node_indices = Vec::new();
    for name in &output_node_names {
        let raw = mna.node_map.get(*name).copied().ok_or_else(|| {
            anyhow::anyhow!(
                "Output node '{}' not found in circuit. Available: {:?}",
                name,
                mna.node_map.keys().collect::<Vec<_>>()
            )
        })?;
        if raw == 0 {
            anyhow::bail!(
                "Output node '{}' cannot be ground (0). Please specify a non-ground node.",
                name
            );
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
        oversampling_factor: oversampling,
        dc_block: !no_dc_block,
        backward_euler,
        ..CodegenConfig::default()
    };

    let generator = CodeGenerator::new(config);
    // DK codegen handles standard and augmented (single-transformer) circuits.
    // For circuits with multiple transformer groups (e.g., Pultec with HS-29 + S-217-D),
    // the DK K matrix can have stability issues from inter-transformer coupling.
    // Fall back to nodal codegen for those circuits.
    let n_xfmr_groups = mna.transformer_groups.len()
        + if !mna.coupled_inductors.is_empty() {
            1
        } else {
            0
        };
    // Check trapezoidal stability: spectral radius of S*A_neg > 1.001 means
    // the trapezoidal method will ring. Route to nodal (not DK+BE) since nodal
    // handles stiff circuits correctly via voltage-space NR.
    let dk_unstable = if !dk_failed && kernel.m > 0 {
        let n_k = kernel.n;
        let mut x = vec![1.0 / (n_k as f64).sqrt(); n_k];
        let mut y = vec![0.0; n_k];
        let mut rho = 0.0;
        for _ in 0..20 {
            let mut ax = vec![0.0; n_k];
            for (i, ax_i) in ax.iter_mut().enumerate() {
                for (j, x_j) in x.iter().enumerate() {
                    *ax_i += kernel.a_neg[i * n_k + j] * x_j;
                }
            }
            for (i, y_i) in y.iter_mut().enumerate() {
                *y_i = 0.0;
                for (j, ax_j) in ax.iter().enumerate() {
                    *y_i += kernel.s[i * n_k + j] * ax_j;
                }
            }
            let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
            if norm < 1e-30 {
                break;
            }
            rho = norm / x.iter().map(|v| v * v).sum::<f64>().sqrt();
            for i in 0..n_k {
                x[i] = y[i] / norm;
            }
        }
        if rho > 1.002 {
            println!(
                "  Trapezoidal unstable (spectral radius {:.4}), routing to nodal",
                rho
            );
            true
        } else {
            false
        }
    } else {
        false
    };

    let use_nodal_codegen = match solver_override {
        "nodal" => true,
        "dk" => false,
        _ => {
            // Auto-select nodal when:
            // 1. DK kernel failed (positive feedback / oscillator circuits)
            // 2. DK trapezoidal unstable (spectral radius > 1.001, stiff circuits)
            // 3. Multi-transformer circuits (DK K matrix unstable for inter-transformer coupling)
            // 4. Large M (≥10): M×M Gauss elimination is expensive and NR can diverge.
            let multi_xfmr = has_inductors_compile && n_xfmr_groups > 1;
            let large_m = kernel.m >= 10;
            dk_failed || dk_unstable || multi_xfmr || large_m
        }
    };

    // When auto-routing to nodal (not user-requested), undo FA reduction.
    // FA assumes permanently forward-active BJTs, but circuits that need nodal
    // (oscillators, high-gain feedback) have BJTs that swing between regions.
    if use_nodal_codegen && solver_override != "nodal" && !forward_active.is_empty() {
        println!("  Undoing FA reduction for nodal path (BJTs may leave forward-active during transient)");
        mna = MnaSystem::from_netlist(&netlist)
            .with_context(|| "Failed to rebuild MNA without FA")?;
        if input_node_idx < mna.n {
            mna.g[input_node_idx][input_node_idx] += input_conductance;
        }
        let device_slots =
            melange_solver::codegen::ir::CircuitIR::build_device_info(&netlist).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    let generated = if use_nodal_codegen {
        println!("  Using nodal solver codegen");
        // Expand MNA with internal nodes for parasitic BJTs.
        // Skip expansion if K will be ill-conditioned (K_diag < -100), which triggers
        // the full N×N LU path instead of Schur. The LU path handles parasitics via
        // bjt_with_parasitics() — internal nodes would increase N without benefit.
        let k_diag_min = if kernel.m > 0 {
            (0..kernel.m)
                .map(|i| kernel.k[i * kernel.m + i])
                .fold(0.0_f64, f64::min)
        } else {
            0.0
        };
        let skip_expansion = k_diag_min < -100.0;
        if !skip_expansion {
            let device_slots = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
                &netlist,
                Some(&mna),
            )
            .unwrap_or_default();
            if !device_slots.is_empty() {
                mna.expand_bjt_internal_nodes(&device_slots);
            }
        } else {
            println!("  Skipping internal node expansion (K ill-conditioned, using full LU)");
        }
        generator
            .generate_nodal(&mna, &netlist)
            .with_context(|| "Nodal code generation failed")?
    } else {
        // DK path: do NOT expand internal nodes. The DK kernel is ill-conditioned
        // with high-conductance parasitic nodes. Instead, bjt_with_parasitics()
        // inner NR handles parasitics in the generated code.
        if has_inductors_compile {
            println!("  Using DK codegen with augmented MNA for inductors");
        }
        generator
            .generate(&kernel, &mna, &netlist)
            .with_context(|| "Code generation failed")?
    };

    println!(
        "  ✓ Generated {} lines of Rust code",
        generated.code.lines().count()
    );

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
            println!(
                "  melange compile {} --output {} --format plugin",
                circuit_source.name(),
                output.display()
            );
        }
        OutputFormat::Plugin => {
            // Generate complete plugin project
            let project_dir = if output.extension().is_some() {
                output.with_extension("")
            } else {
                output.to_path_buf()
            };

            // Get a sanitized circuit name
            let circuit_name: String = project_dir
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("circuit")
                .to_lowercase()
                .chars()
                .map(|c| {
                    if c.is_ascii_alphanumeric() || c == '_' {
                        c
                    } else {
                        '_'
                    }
                })
                .collect();
            let circuit_name = if circuit_name.starts_with(|c: char| c.is_ascii_digit()) {
                format!("circuit_{circuit_name}")
            } else {
                circuit_name
            };

            // Build pot/switch parameter info from MNA data (works for both DK and nodal paths).
            let pot_params: Vec<plugin_template::PotParamInfo> = mna
                .pots
                .iter()
                .enumerate()
                .map(|(idx, p)| plugin_template::PotParamInfo {
                    index: idx,
                    name: netlist
                        .pots
                        .get(idx)
                        .map(|d| d.label.clone().unwrap_or_else(|| d.resistor_name.clone()))
                        .unwrap_or_else(|| format!("Pot {}", idx)),
                    min_resistance: p.min_resistance,
                    max_resistance: p.max_resistance,
                    default_resistance: netlist
                        .pots
                        .get(idx)
                        .and_then(|d| d.default_value)
                        .unwrap_or(1.0 / p.g_nominal),
                })
                .collect();
            let switch_params: Vec<plugin_template::SwitchParamInfo> = netlist
                .switches
                .iter()
                .enumerate()
                .map(|(idx, sw)| plugin_template::SwitchParamInfo {
                    index: idx,
                    name: sw.label.clone().unwrap_or_else(|| {
                        format!("Switch {} ({})", idx, sw.component_names.join("+"))
                    }),
                    num_positions: sw.positions.len(),
                })
                .collect();

            let plugin_options = plugin_template::PluginOptions {
                plugin_name,
                mono,
                wet_dry_mix,
                ear_protection,
            };
            plugin_template::generate_plugin_project_with_oversampling(
                &project_dir,
                &generated.code,
                &circuit_name,
                with_level_params,
                &pot_params,
                &switch_params,
                output_node_indices.len(),
                has_inductors_compile,
                oversampling,
                &plugin_options,
            )?;

            println!("  ✓ Done!");
            println!();
            println!("Generated plugin project at: {}", project_dir.display());
            println!();
            println!("To build the plugin (CLAP + VST3):");
            println!(
                "  cd {}",
                project_dir
                    .file_name()
                    .and_then(|n| n.to_str())
                    .unwrap_or("<project-dir>")
            );
            println!("  cargo nih-plug bundle {} --release", circuit_name);
            println!();
            println!("The compiled plugin (CLAP + VST3) will be in:");
            println!("  target/bundled/");
        }
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn validate_circuit_source(
    circuit_source: &circuits::CircuitSource,
    output_node: &str,
    sample_rate: f64,
    duration: f64,
    amplitude: f64,
    input_node: &str,
    csv_output: Option<&PathBuf>,
    relaxed: bool,
) -> Result<()> {
    use melange_validate::{
        comparison::ComparisonConfig, spice_runner::is_ngspice_available,
        validate_circuit_with_options, ValidationOptions,
    };

    println!("melange validate");
    println!("  Source: {}", circuit_source.name());
    println!("  Output node: {}", output_node);
    println!("  Input node: {}", input_node);
    println!("  Sample rate: {} Hz", sample_rate);
    println!("  Duration: {}s", duration);
    println!("  Amplitude: {}V", amplitude);
    println!(
        "  Tolerances: {}",
        if relaxed { "relaxed" } else { "strict" }
    );
    println!();

    // Step 1: Check ngspice availability
    println!("Step 1: Checking ngspice...");
    if !is_ngspice_available() {
        anyhow::bail!(
            "ngspice is not installed or not found in PATH.\n\
             Install it with: sudo apt install ngspice (Debian/Ubuntu)\n\
             or: brew install ngspice (macOS)"
        );
    }
    println!("  ngspice found");

    // Step 2: Get circuit netlist as a file path
    // validate_circuit needs a file path. For local files, use directly.
    // For builtins/URLs, write to a temp file.
    println!("Step 2: Loading circuit...");
    let (netlist_path, _temp_file) = match circuit_source {
        circuits::CircuitSource::Local { path } => {
            // Verify the file exists
            if !path.exists() {
                anyhow::bail!("Circuit file not found: {}", path.display());
            }
            (path.clone(), None)
        }
        circuits::CircuitSource::Builtin { content, name } => {
            println!("  Using builtin circuit: {}", name);
            let temp_dir = std::env::temp_dir();
            let temp_path = temp_dir.join(format!("melange_validate_{}.cir", std::process::id()));
            std::fs::write(&temp_path, content).with_context(|| {
                format!("Failed to write temp netlist to {}", temp_path.display())
            })?;
            (temp_path.clone(), Some(temp_path))
        }
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            println!("  Fetching from URL: {}", url);
            let cache = cache::Cache::new()?;
            let content = cache.get_sync(url, false)?;
            let temp_dir = std::env::temp_dir();
            let temp_path = temp_dir.join(format!("melange_validate_{}.cir", std::process::id()));
            std::fs::write(&temp_path, &content).with_context(|| {
                format!("Failed to write temp netlist to {}", temp_path.display())
            })?;
            (temp_path.clone(), Some(temp_path))
        }
    };

    // Step 3: Generate test input signal (1kHz sine)
    println!(
        "Step 3: Generating test signal ({:.1}s, {:.3}V amplitude, 1kHz sine)...",
        duration, amplitude
    );
    let num_samples = (duration * sample_rate) as usize;
    let input_signal: Vec<f64> = (0..num_samples)
        .map(|i| amplitude * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / sample_rate).sin())
        .collect();
    println!("  {} samples", input_signal.len());

    // Step 4: Configure comparison
    let config = if relaxed {
        ComparisonConfig::relaxed()
    } else {
        ComparisonConfig::strict()
    };

    let options = ValidationOptions {
        generate_html_on_failure: false,
        generate_html_on_success: false,
        generate_csv: csv_output.is_some(),
        output_dir: csv_output.and_then(|p| p.parent().map(|d| d.to_path_buf())),
        circuit_name: Some(circuit_source.name()),
        input_node: input_node.to_string(),
        ..Default::default()
    };

    // Step 5: Run validation
    println!("Step 4: Running validation (ngspice + melange solver)...");
    let result = validate_circuit_with_options(
        &netlist_path,
        &input_signal,
        sample_rate,
        output_node,
        &config,
        &options,
    );

    // Clean up temp file if we created one
    if let Some(ref temp) = _temp_file {
        let _ = std::fs::remove_file(temp);
    }

    let result = result.with_context(|| "Validation failed")?;

    // Step 6: Print report
    println!();
    println!("{}", result.report.summary());

    // Step 7: Write CSV if requested
    if let Some(csv_path) = csv_output {
        // The validate library may have already written CSV if output_dir matched,
        // but if the user specified a specific path, write it explicitly
        if result.csv_path.as_ref() != Some(&csv_path.to_path_buf()) {
            // We need to reconstruct signals from the report info to write CSV.
            // Re-run would be expensive, so only rely on the library's CSV if it wrote one.
            if let Some(lib_csv) = &result.csv_path {
                // Copy the library-generated CSV to the user-specified path
                std::fs::copy(lib_csv, csv_path)
                    .with_context(|| format!("Failed to copy CSV to {}", csv_path.display()))?;
                println!("CSV written to: {}", csv_path.display());
            }
        } else if result.csv_path.is_some() {
            println!("CSV written to: {}", csv_path.display());
        }
    }

    // Exit with error if validation failed
    if result.report.passed {
        println!("Validation PASSED");
        Ok(())
    } else {
        anyhow::bail!(
            "Validation FAILED: {} tolerance check(s) exceeded.\n\
             Use --relaxed for less strict tolerances, or investigate the differences.",
            result.report.failures.len()
        );
    }
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
    solver: &'a str,
}

fn simulate_circuit_source(
    circuit_source: &circuits::CircuitSource,
    opts: &SimulateOptions,
) -> Result<()> {
    use melange_devices::bjt::{BjtEbersMoll, BjtPolarity};
    use melange_devices::diode::DiodeShockley;
    use melange_devices::jfet::{Jfet, JfetChannel};
    use melange_devices::mosfet::{ChannelType, Mosfet};
    use melange_devices::tube::KorenTriode;
    use melange_solver::{
        dk::DkKernel,
        mna::MnaSystem,
        parser::Netlist,
        solver::{CircuitSolver, DeviceEntry, NodalSolver},
    };

    println!("melange simulate");
    println!("  Source: {}", circuit_source.name());
    println!();

    // Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, name } => {
            println!("  Using builtin circuit: {}", name);
            content.clone()
        }
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read local file: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            println!("  Fetching from URL: {}", url);
            let cache = cache::Cache::new()?;
            cache.get_sync(url, false)?
        }
    };

    // Step 1: Parse netlist
    println!("Step 1: Parsing SPICE netlist...");
    let mut netlist =
        Netlist::parse(&netlist_str).with_context(|| "Failed to parse SPICE netlist")?;

    // Expand subcircuit instances (X elements) before MNA
    if !netlist.subcircuits.is_empty() {
        let num_subcircuits = netlist.subcircuits.len();
        netlist
            .expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
        println!("  Expanded {} subcircuit definition(s)", num_subcircuits);
    }
    println!("  Parsed {} elements", netlist.elements.len());

    // Step 2: Build MNA
    println!("Step 2: Building MNA system...");
    let mut mna =
        MnaSystem::from_netlist(&netlist).with_context(|| "Failed to build MNA system")?;

    let input_node_raw = mna.node_map.get(opts.input_node).copied().ok_or_else(|| {
        anyhow::anyhow!(
            "Input node '{}' not found. Available: {:?}",
            opts.input_node,
            mna.node_map.keys().collect::<Vec<_>>()
        )
    })?;
    if input_node_raw == 0 {
        anyhow::bail!("Input node cannot be ground (0)");
    }
    let input_node_idx = input_node_raw - 1;

    let output_node_raw = mna.node_map.get(opts.output_node).copied().ok_or_else(|| {
        anyhow::anyhow!(
            "Output node '{}' not found. Available: {:?}",
            opts.output_node,
            mna.node_map.keys().collect::<Vec<_>>()
        )
    })?;
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
    println!(
        "  Input resistance: {} ohm ({})",
        input_resistance, ir_source
    );
    let input_conductance = 1.0 / input_resistance;
    if input_node_idx < mna.n {
        mna.g[input_node_idx][input_node_idx] += input_conductance;
    }

    println!(
        "  {} nodes, {} nonlinear devices",
        mna.n,
        mna.nonlinear_devices.len()
    );

    // Expand MNA with internal nodes for parasitic BJTs, then stamp junction caps.
    // Must happen BEFORE kernel build so parasitic R and caps are in A = G + 2C/T.
    {
        let device_slots =
            melange_solver::codegen::ir::CircuitIR::build_device_info(&netlist).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.expand_bjt_internal_nodes(&device_slots);
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

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
                reader
                    .into_samples::<i32>()
                    .map(|s| s.unwrap() as f64 / max_val)
                    .collect()
            }
            hound::SampleFormat::Float => reader
                .into_samples::<f32>()
                .map(|s| s.unwrap() as f64)
                .collect(),
        };
        println!(
            "  {} samples at {} Hz ({:.2}s)",
            input_samples.len(),
            sr,
            input_samples.len() as f64 / sr
        );
        (input_samples, sr)
    } else {
        println!(
            "Step 3: Generating 1kHz sine wave ({:.1}s at {} Hz)...",
            opts.duration, opts.sample_rate
        );
        let num_samples = (opts.duration * opts.sample_rate) as usize;
        let samples: Vec<f64> = (0..num_samples)
            .map(|i| {
                opts.amplitude
                    * (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / opts.sample_rate).sin()
            })
            .collect();
        (samples, opts.sample_rate)
    };

    // Step 4: Build DK kernel
    let has_inductors = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();

    println!("Step 4: Building DK kernel at {} Hz...", actual_sample_rate);
    let kernel = if has_inductors && opts.solver != "dk" {
        // Use augmented MNA for inductors (well-conditioned for large L)
        println!("  Using augmented MNA for {} inductor variables", {
            mna.inductors.len()
                + mna.coupled_inductors.len() * 2
                + mna
                    .transformer_groups
                    .iter()
                    .map(|g| g.num_windings)
                    .sum::<usize>()
        });
        DkKernel::from_mna_augmented(&mna, actual_sample_rate)
            .with_context(|| "Failed to create augmented DK kernel")?
    } else {
        DkKernel::from_mna(&mna, actual_sample_rate)
            .with_context(|| "Failed to create DK kernel")?
    };
    println!("  N={}, M={}", kernel.n, kernel.m);

    // Step 5: Build solver
    println!("Step 5: Creating solver...");

    // Helper: find model param
    let find_model_param = |model_name: &str, param: &str| -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param))
                    .map(|(_, v)| *v)
            })
    };

    // Build device entries from MNA nonlinear device list
    let mut devices = Vec::new();
    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                // Find model name for this diode
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Diode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-15);
                let n = find_model_param(model_name, "N").unwrap_or(1.0);
                let diode = DiodeShockley::new_room_temp(is, n);
                devices.push(DeviceEntry::new_diode(diode, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-14);
                let bf = find_model_param(model_name, "BF").unwrap_or(200.0);
                let br = find_model_param(model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                let polarity = if is_pnp {
                    BjtPolarity::Pnp
                } else {
                    BjtPolarity::Npn
                };
                let nf = find_model_param(model_name, "NF").unwrap_or(1.0);
                let bjt = BjtEbersMoll::new(is, melange_primitives::VT_ROOM, bf, br, polarity)
                    .with_nf(nf);
                devices.push(DeviceEntry::new_bjt(bjt, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Jfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Jfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
                    .unwrap_or(false);
                let channel = if is_p_channel {
                    JfetChannel::P
                } else {
                    JfetChannel::N
                };
                let default_vp = if is_p_channel { 2.0 } else { -2.0 };
                let vp = find_model_param(model_name, "VTO").unwrap_or(default_vp);
                // ngspice BETA = IDSS / VP^2, so IDSS = BETA * VP^2
                let idss = if let Some(beta) = find_model_param(model_name, "BETA") {
                    beta * vp * vp
                } else {
                    find_model_param(model_name, "IDSS").unwrap_or(2e-3)
                };
                let mut jfet = Jfet::new(channel, vp, idss);
                jfet.lambda = find_model_param(model_name, "LAMBDA").unwrap_or(0.001);
                devices.push(DeviceEntry::new_jfet(jfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Mosfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Mosfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PM"))
                    .unwrap_or(false);
                let channel = if is_p_channel {
                    ChannelType::P
                } else {
                    ChannelType::N
                };
                let default_vt = if is_p_channel { -2.0 } else { 2.0 };
                let vt = find_model_param(model_name, "VTO").unwrap_or(default_vt);
                let kp = find_model_param(model_name, "KP").unwrap_or(0.1);
                let lambda = find_model_param(model_name, "LAMBDA").unwrap_or(0.01);
                let mosfet = Mosfet::new(channel, vt, kp, lambda);
                devices.push(DeviceEntry::new_mosfet(mosfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Tube => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Triode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let mu = find_model_param(model_name, "MU").unwrap_or(100.0);
                let ex = find_model_param(model_name, "EX").unwrap_or(1.4);
                let kg1 = find_model_param(model_name, "KG1").unwrap_or(1060.0);
                let kp = find_model_param(model_name, "KP").unwrap_or(600.0);
                let kvb = find_model_param(model_name, "KVB").unwrap_or(300.0);
                let ig_max = find_model_param(model_name, "IG_MAX").unwrap_or(2e-3);
                let vgk_onset = find_model_param(model_name, "VGK_ONSET").unwrap_or(0.5);
                let lambda = find_model_param(model_name, "LAMBDA").unwrap_or(0.0);
                let tube =
                    KorenTriode::with_all_params(mu, ex, kg1, kp, kvb, ig_max, vgk_onset, lambda);
                devices.push(DeviceEntry::new_tube(tube, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::BjtForwardActive => {
                // BjtForwardActive uses the same runtime 2D model as Bjt
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-14);
                let bf = find_model_param(model_name, "BF").unwrap_or(200.0);
                let br = find_model_param(model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                let polarity = if is_pnp {
                    BjtPolarity::Pnp
                } else {
                    BjtPolarity::Npn
                };
                let nf = find_model_param(model_name, "NF").unwrap_or(1.0);
                let bjt = BjtEbersMoll::new(is, melange_primitives::VT_ROOM, bf, br, polarity)
                    .with_nf(nf);
                devices.push(DeviceEntry::new_bjt(bjt, dev_info.start_idx));
            }
        }
    }

    let has_nonlinear = !devices.is_empty();

    // Determine solver type: auto picks nodal for nonlinear circuits with inductors
    let use_nodal = match opts.solver {
        "nodal" => true,
        "dk" => false,
        _ => has_nonlinear && has_inductors, // "auto"
    };

    if kernel.m == 0 && !use_nodal {
        // Linear circuit: use LinearSolver
        let mut solver =
            melange_solver::solver::LinearSolver::new(kernel, input_node_idx, output_node_idx);
        solver.set_input_conductance(input_conductance);

        println!("  Linear solver ready");
        println!();

        // Process
        println!("Step 6: Processing {} samples...", samples.len());
        let mut output_samples = Vec::with_capacity(samples.len());
        for &s in &samples {
            output_samples.push(solver.process_sample(s));
        }

        eprintln!("  Peak output: {:.2}V", solver.diag_peak_output());
        if solver.diag_clamp_count() > 0 {
            eprintln!(
                "  WARNING: Output exceeded ±10V {} times -- try reducing --amplitude or adding gain reduction to the circuit",
                solver.diag_clamp_count()
            );
        }

        write_wav(opts.output, actual_sample_rate, &output_samples)?;
    } else if use_nodal {
        // Full-nodal NR solver (augmented MNA for inductors)
        let device_slots = build_device_slots(&netlist, &mna);
        let mut solver = NodalSolver::new(
            kernel,
            &mna,
            &netlist,
            device_slots.clone(),
            input_node_idx,
            output_node_idx,
        )
        .with_context(|| "Failed to create nodal solver")?;
        solver.set_input_conductance(input_conductance);

        if has_nonlinear {
            println!("  Initializing DC operating point (nodal)...");
            solver.initialize_dc_op(&mna, &device_slots);
            println!("  DC OP initialized");
        }

        let n_inductor_vars = mna.inductors.len()
            + mna.coupled_inductors.len() * 2
            + mna
                .transformer_groups
                .iter()
                .map(|g| g.num_windings)
                .sum::<usize>();
        println!(
            "  Nodal solver ready (M={}, {} inductor variables, n_nodal={})",
            mna.nonlinear_devices
                .iter()
                .map(|d| d.dimension)
                .sum::<usize>(),
            n_inductor_vars,
            solver.v_prev().len()
        );
        println!();

        // Process
        println!("Step 6: Processing {} samples...", samples.len());
        let mut output_samples = Vec::with_capacity(samples.len());
        for &s in &samples {
            output_samples.push(solver.process_sample(s));
        }

        eprintln!("  Peak output: {:.2}V", solver.diag_peak_output());
        if solver.diag_clamp_count() > 0 {
            eprintln!(
                "  WARNING: Output exceeded ±10V {} times -- try reducing --amplitude or adding gain reduction to the circuit",
                solver.diag_clamp_count()
            );
        }
        if solver.diag_nr_max_iter_count() > 0 {
            eprintln!(
                "  NR max iterations: {} times",
                solver.diag_nr_max_iter_count()
            );
        }
        if solver.diag_be_fallback_count() > 0 {
            eprintln!("  BE fallback: {} samples", solver.diag_be_fallback_count());
        }
        if solver.diag_nan_reset_count() > 0 {
            eprintln!("  NaN resets: {}", solver.diag_nan_reset_count());
        }

        write_wav(opts.output, actual_sample_rate, &output_samples)?;
    } else {
        // Nonlinear circuit: use CircuitSolver (DK method)
        let mut solver = CircuitSolver::new(kernel, devices, input_node_idx, output_node_idx)
            .with_context(|| "Failed to create circuit solver")?;
        solver.set_input_conductance(input_conductance);

        // Apply K_eff parasitic R corrections for BJTs
        {
            let device_slots = build_device_slots(&netlist, &mna);
            let mut k_eff_corrections = Vec::new();
            for slot in &device_slots {
                if let melange_solver::codegen::ir::DeviceParams::Bjt(bp) = &slot.params {
                    if bp.has_parasitics()
                        && slot.device_type == melange_solver::codegen::ir::DeviceType::Bjt
                    {
                        k_eff_corrections.push((slot.start_idx, bp.rb, bp.rc, bp.re));
                    }
                }
            }
            if !k_eff_corrections.is_empty() {
                solver.apply_k_eff_corrections(&k_eff_corrections);
            }
        }

        // Initialize DC operating point for nonlinear circuits
        if has_nonlinear {
            println!("  Initializing DC operating point...");
            let device_slots = build_device_slots(&netlist, &mna);
            let converged = solver.initialize_dc_op(&mna, &device_slots);
            if !converged {
                eprintln!("  Warning: DC operating point did not fully converge");
            }
            println!("  DC OP initialized");
        }

        println!(
            "  Nonlinear solver ready (M={})",
            mna.nonlinear_devices
                .iter()
                .map(|d| d.dimension)
                .sum::<usize>()
        );
        println!();

        // Process
        println!("Step 6: Processing {} samples...", samples.len());
        let mut output_samples = Vec::with_capacity(samples.len());
        for &s in &samples {
            output_samples.push(solver.process_sample(s));
        }

        eprintln!("  Peak output: {:.2}V", solver.diag_peak_output());
        if solver.diag_clamp_count() > 0 {
            eprintln!(
                "  WARNING: Output exceeded ±10V {} times -- try reducing --amplitude or adding gain reduction to the circuit",
                solver.diag_clamp_count()
            );
        }
        if solver.diag_nr_max_iter_count() > 0 {
            eprintln!(
                "  NR max iterations: {} times",
                solver.diag_nr_max_iter_count()
            );
        }
        if solver.diag_nan_reset_count() > 0 {
            eprintln!("  NaN resets: {}", solver.diag_nan_reset_count());
        }

        write_wav(opts.output, actual_sample_rate, &output_samples)?;
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn analyze_freq_response(
    circuit_source: &circuits::CircuitSource,
    input_node_name: &str,
    output_node_name: &str,
    start_freq: f64,
    end_freq: f64,
    points_per_decade: usize,
    amplitude: f64,
    sample_rate: f64,
    input_resistance_flag: Option<f64>,
    output_file: Option<&PathBuf>,
    pot_overrides: &[String],
    switch_overrides: &[String],
) -> Result<()> {
    use melange_solver::{
        dk::DkKernel,
        mna::MnaSystem,
        parser::Netlist,
        solver::{CircuitSolver, LinearSolver},
    };

    eprintln!("melange analyze (frequency response)");

    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, name } => {
            eprintln!("  Using builtin circuit: {}", name);
            content.clone()
        }
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            let cache = cache::Cache::new()?;
            cache.get_sync(url, false)?
        }
    };

    let mut netlist =
        Netlist::parse(&netlist_str).with_context(|| "Failed to parse SPICE netlist")?;
    if !netlist.subcircuits.is_empty() {
        netlist
            .expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
    }

    // Apply pot overrides: modify element values in netlist before building MNA.
    // Also apply .pot defaults for any pot NOT overridden (so analyze uses "flat" defaults).
    {
        let mut overridden_resistors: std::collections::HashSet<String> =
            std::collections::HashSet::new();

        // First, apply explicit --pot overrides
        for spec in pot_overrides {
            let (name, val_str) = spec.split_once('=').ok_or_else(|| {
                anyhow::anyhow!("Invalid --pot format '{}', expected NAME=VALUE", spec)
            })?;
            let value = melange_solver::parser::parse_value(val_str).map_err(|_| {
                anyhow::anyhow!("Invalid resistance value '{}' in --pot {}", val_str, spec)
            })?;
            if value <= 0.0 || !value.is_finite() {
                anyhow::bail!("Pot value must be positive and finite: {}", spec);
            }

            // Match by pot label or resistor name
            let resistor_name = netlist
                .pots
                .iter()
                .find(|p| {
                    p.label
                        .as_deref()
                        .map(|l| l.eq_ignore_ascii_case(name))
                        .unwrap_or(false)
                        || p.resistor_name.eq_ignore_ascii_case(name)
                })
                .map(|p| p.resistor_name.clone())
                .ok_or_else(|| {
                    let available: Vec<String> = netlist
                        .pots
                        .iter()
                        .map(|p| {
                            format!(
                                "{} ({})",
                                p.resistor_name,
                                p.label.as_deref().unwrap_or("no label")
                            )
                        })
                        .collect();
                    anyhow::anyhow!(
                        "Pot '{}' not found. Available: {}",
                        name,
                        available.join(", ")
                    )
                })?;

            // Update element value
            let found = netlist.elements.iter_mut().any(|e| {
                if let melange_solver::parser::Element::Resistor {
                    name: n, value: v, ..
                } = e
                {
                    if n.eq_ignore_ascii_case(&resistor_name) {
                        *v = value;
                        return true;
                    }
                }
                false
            });
            if !found {
                anyhow::bail!(
                    "Resistor '{}' referenced by pot not found in netlist",
                    resistor_name
                );
            }
            overridden_resistors.insert(resistor_name.to_ascii_uppercase());
            eprintln!("  Pot override: {} = {:.1}", resistor_name, value);
        }

        // Apply .pot defaults for pots not explicitly overridden
        for pot in &netlist.pots {
            if overridden_resistors.contains(&pot.resistor_name.to_ascii_uppercase()) {
                continue;
            }
            if let Some(default) = pot.default_value {
                for e in netlist.elements.iter_mut() {
                    if let melange_solver::parser::Element::Resistor {
                        name: n, value: v, ..
                    } = e
                    {
                        if n.eq_ignore_ascii_case(&pot.resistor_name) {
                            *v = default;
                            break;
                        }
                    }
                }
            }
        }
    }

    // Apply switch overrides: modify element values in netlist before building MNA.
    // Also apply switch position 0 as default for unoverridden switches.
    {
        let mut overridden_switches: std::collections::HashSet<usize> =
            std::collections::HashSet::new();

        for spec in switch_overrides {
            let (name, pos_str) = spec.split_once('=').ok_or_else(|| {
                anyhow::anyhow!("Invalid --switch format '{}', expected NAME=POS", spec)
            })?;
            let position: usize = pos_str.parse().map_err(|_| {
                anyhow::anyhow!("Invalid switch position '{}' in --switch {}", pos_str, spec)
            })?;

            // Match by label or index
            let switch_idx = if let Ok(idx) = name.parse::<usize>() {
                if idx >= netlist.switches.len() {
                    anyhow::bail!(
                        "Switch index {} out of range (0..{})",
                        idx,
                        netlist.switches.len()
                    );
                }
                idx
            } else {
                netlist
                    .switches
                    .iter()
                    .position(|s| {
                        s.label
                            .as_deref()
                            .map(|l| l.eq_ignore_ascii_case(name))
                            .unwrap_or(false)
                    })
                    .ok_or_else(|| {
                        let available: Vec<String> = netlist
                            .switches
                            .iter()
                            .enumerate()
                            .map(|(i, s)| {
                                format!("{}: {}", i, s.label.as_deref().unwrap_or("no label"))
                            })
                            .collect();
                        anyhow::anyhow!(
                            "Switch '{}' not found. Available: {}",
                            name,
                            available.join(", ")
                        )
                    })?
            };

            let sw = &netlist.switches[switch_idx];
            if position >= sw.positions.len() {
                anyhow::bail!(
                    "Switch '{}' position {} out of range (0..{})",
                    name,
                    position,
                    sw.positions.len()
                );
            }

            apply_switch_position(&mut netlist.elements, sw, position);
            overridden_switches.insert(switch_idx);
            eprintln!(
                "  Switch override: {} = position {}",
                sw.label.as_deref().unwrap_or(name),
                position
            );
        }

        // Default: switches stay at position 0 (netlist nominal values are position 0)
        // No action needed — netlist already has position 0 values.
    }

    let mut mna =
        MnaSystem::from_netlist(&netlist).with_context(|| "Failed to build MNA system")?;

    let input_node_raw = mna.node_map.get(input_node_name).copied().ok_or_else(|| {
        anyhow::anyhow!(
            "Input node '{}' not found. Available: {:?}",
            input_node_name,
            mna.node_map.keys().collect::<Vec<_>>()
        )
    })?;
    if input_node_raw == 0 {
        anyhow::bail!("Input node cannot be ground (0)");
    }
    let input_node_idx = input_node_raw - 1;

    let output_node_raw = mna.node_map.get(output_node_name).copied().ok_or_else(|| {
        anyhow::anyhow!(
            "Output node '{}' not found. Available: {:?}",
            output_node_name,
            mna.node_map.keys().collect::<Vec<_>>()
        )
    })?;
    if output_node_raw == 0 {
        anyhow::bail!("Output node cannot be ground (0)");
    }
    let output_node_idx = output_node_raw - 1;

    let (input_resistance, ir_source) = if let Some(r) = input_resistance_flag {
        (r, "flag")
    } else if let Some(r) = netlist.input_impedance {
        (r, "directive")
    } else {
        (1.0, "default")
    };
    let input_conductance = 1.0 / input_resistance;
    if input_node_idx < mna.n {
        mna.g[input_node_idx][input_node_idx] += input_conductance;
    }
    eprintln!(
        "  N={}, M={}, R_in={} ({})",
        mna.n,
        mna.nonlinear_devices
            .iter()
            .map(|d| d.dimension)
            .sum::<usize>(),
        input_resistance,
        ir_source
    );

    // Generate log-spaced frequencies
    let freqs = generate_log_frequencies(start_freq, end_freq, points_per_decade);
    eprintln!(
        "  Sweeping {} frequencies from {:.0} Hz to {:.0} Hz",
        freqs.len(),
        start_freq,
        end_freq
    );

    let nyquist = sample_rate / 2.0;
    if end_freq > nyquist {
        eprintln!(
            "  Warning: end frequency {:.0} Hz exceeds Nyquist ({:.0} Hz), results above Nyquist will alias",
            end_freq, nyquist
        );
    }

    // Expand MNA with internal nodes for parasitic BJTs, then stamp junction caps.
    {
        let device_slots =
            melange_solver::codegen::ir::CircuitIR::build_device_info(&netlist).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.expand_bjt_internal_nodes(&device_slots);
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    // Detect inductors — auto-select NodalSolver for nonlinear+inductor circuits
    let has_inductors = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();
    let has_nonlinear = !mna.nonlinear_devices.is_empty();
    let use_nodal = has_nonlinear && has_inductors;

    // Build kernel (uses augmented MNA for inductor circuits)
    let kernel = if use_nodal {
        DkKernel::from_mna_augmented(&mna, sample_rate)
            .with_context(|| "Failed to create augmented DK kernel")?
    } else {
        DkKernel::from_mna(&mna, sample_rate).with_context(|| "Failed to create DK kernel")?
    };

    if use_nodal {
        eprintln!("  Using NodalSolver (inductors + nonlinear devices)");
    }

    // Build device slots for DC OP initialization
    let device_slots = build_device_slots(&netlist, &mna);

    // Create base solver (compute DC OP once, clone for each frequency)
    enum BaseSolver {
        Dk(melange_solver::solver::CircuitSolver),
        Nodal(melange_solver::solver::NodalSolver),
    }

    let base_solver: Option<BaseSolver> = if use_nodal {
        let mut solver = melange_solver::solver::NodalSolver::new(
            kernel.clone(),
            &mna,
            &netlist,
            device_slots.clone(),
            input_node_idx,
            output_node_idx,
        )
        .with_context(|| "Failed to create nodal solver")?;
        solver.set_input_conductance(input_conductance);
        solver.initialize_dc_op(&mna, &device_slots);
        Some(BaseSolver::Nodal(solver))
    } else if has_nonlinear {
        let devices = build_device_entries(&netlist, &mna);
        let mut solver =
            CircuitSolver::new(kernel.clone(), devices, input_node_idx, output_node_idx)
                .with_context(|| "Failed to create circuit solver")?;
        solver.set_input_conductance(input_conductance);
        // Apply K_eff parasitic R corrections for BJTs
        {
            let mut k_corrections = Vec::new();
            for slot in &device_slots {
                if let melange_solver::codegen::ir::DeviceParams::Bjt(bp) = &slot.params {
                    if bp.has_parasitics()
                        && slot.device_type == melange_solver::codegen::ir::DeviceType::Bjt
                    {
                        k_corrections.push((slot.start_idx, bp.rb, bp.rc, bp.re));
                    }
                }
            }
            if !k_corrections.is_empty() {
                solver.apply_k_eff_corrections(&k_corrections);
            }
        }
        solver.initialize_dc_op(&mna, &device_slots);
        Some(BaseSolver::Dk(solver))
    } else {
        None
    };

    // Measure at each frequency
    let mut results: Vec<(f64, f64, f64)> = Vec::with_capacity(freqs.len());

    for &freq in &freqs {
        let (gain_db, phase_deg) = match &base_solver {
            None => {
                let mut solver = LinearSolver::new(kernel.clone(), input_node_idx, output_node_idx);
                solver.set_input_conductance(input_conductance);
                measure_at_frequency(
                    &mut SolverWrapper::Linear(&mut solver),
                    freq,
                    amplitude,
                    sample_rate,
                )
            }
            Some(BaseSolver::Dk(base)) => {
                let mut solver = base.clone();
                measure_at_frequency(
                    &mut SolverWrapper::Nonlinear(&mut solver),
                    freq,
                    amplitude,
                    sample_rate,
                )
            }
            Some(BaseSolver::Nodal(base)) => {
                let mut solver = base.clone();
                measure_at_frequency(
                    &mut SolverWrapper::Nodal(&mut solver),
                    freq,
                    amplitude,
                    sample_rate,
                )
            }
        };
        results.push((freq, gain_db, phase_deg));
        eprintln!("  {:.1} Hz: {:.2} dB, {:.1}°", freq, gain_db, phase_deg);
    }

    // Format CSV output
    let csv = format_freq_response_csv(&results);
    if let Some(path) = output_file {
        std::fs::write(path, &csv)
            .with_context(|| format!("Failed to write output: {}", path.display()))?;
        eprintln!(
            "  Wrote {} frequency points to {}",
            results.len(),
            path.display()
        );
    } else {
        print!("{}", csv);
    }

    Ok(())
}

/// Apply a switch position by updating element values in the netlist.
fn apply_switch_position(
    elements: &mut [melange_solver::parser::Element],
    switch: &melange_solver::parser::SwitchDirective,
    position: usize,
) {
    for (comp_idx, comp_name) in switch.component_names.iter().enumerate() {
        let value = switch.positions[position][comp_idx];
        for e in elements.iter_mut() {
            let (name, val) = match e {
                melange_solver::parser::Element::Resistor { name, value, .. } => {
                    (name as &str, value)
                }
                melange_solver::parser::Element::Capacitor { name, value, .. } => {
                    (name as &str, value)
                }
                melange_solver::parser::Element::Inductor { name, value, .. } => {
                    (name as &str, value)
                }
                _ => continue,
            };
            if name.eq_ignore_ascii_case(comp_name) {
                *val = value;
                break;
            }
        }
    }
}

fn generate_log_frequencies(start: f64, end: f64, points_per_decade: usize) -> Vec<f64> {
    let log_start = start.log10();
    let log_end = end.log10();
    let decades = log_end - log_start;
    let total_points = (decades * points_per_decade as f64).ceil() as usize + 1;
    (0..total_points)
        .map(|i| {
            let log_f = log_start + i as f64 * decades / (total_points - 1).max(1) as f64;
            10.0_f64.powf(log_f)
        })
        .collect()
}

/// Wrapper to abstract over linear/nonlinear solvers for freq measurement.
enum SolverWrapper<'a> {
    Linear(&'a mut melange_solver::solver::LinearSolver),
    Nonlinear(&'a mut melange_solver::solver::CircuitSolver),
    Nodal(&'a mut melange_solver::solver::NodalSolver),
}

impl SolverWrapper<'_> {
    fn process_sample(&mut self, input: f64) -> f64 {
        match self {
            SolverWrapper::Linear(s) => s.process_sample(input),
            SolverWrapper::Nonlinear(s) => s.process_sample(input),
            SolverWrapper::Nodal(s) => s.process_sample(input),
        }
    }
}

/// Measure gain (dB) and phase (degrees) at a single frequency using single-bin DFT.
///
/// Uses a minimum settle time to ensure circuits with large inductors (transformers)
/// reach steady state before measurement. The settle time must exceed ~5× the largest
/// L/R time constant in the circuit.
fn measure_at_frequency(
    solver: &mut SolverWrapper,
    freq: f64,
    amplitude: f64,
    sample_rate: f64,
) -> (f64, f64) {
    use std::f64::consts::PI;
    // Minimum 5 seconds settle + at least 10 cycles, whichever is more samples.
    // 5s covers 5τ for L/R up to ~1 second (e.g. 130H / 100Ω = 1.3s → 5τ = 6.5s, close enough).
    let min_settle_seconds = 5.0;
    let min_settle_from_time = (min_settle_seconds * sample_rate) as usize;
    let period_samples = (sample_rate / freq).round() as usize;
    let min_settle_from_cycles = 10 * period_samples;
    let settle_samples = min_settle_from_time.max(min_settle_from_cycles);
    let measure_cycles = 5;
    let measure_samples = (measure_cycles as f64 * sample_rate / freq).round() as usize;

    // Settle phase
    for i in 0..settle_samples {
        let t = i as f64 / sample_rate;
        let input = amplitude * (2.0 * PI * freq * t).sin();
        solver.process_sample(input);
    }

    // Measure phase using single-bin DFT (correlate with sin/cos at test frequency)
    let mut sin_acc = 0.0;
    let mut cos_acc = 0.0;
    let mut in_sin_acc = 0.0;
    let mut in_cos_acc = 0.0;

    for i in 0..measure_samples {
        let t = (settle_samples + i) as f64 / sample_rate;
        let phase = 2.0 * PI * freq * t;
        let input = amplitude * phase.sin();
        let output = solver.process_sample(input);

        let sin_ref = phase.sin();
        let cos_ref = phase.cos();
        sin_acc += output * sin_ref;
        cos_acc += output * cos_ref;
        in_sin_acc += input * sin_ref;
        in_cos_acc += input * cos_ref;
    }

    // Output amplitude and phase from DFT coefficients
    let out_amp = 2.0 * (sin_acc * sin_acc + cos_acc * cos_acc).sqrt() / measure_samples as f64;
    let in_amp =
        2.0 * (in_sin_acc * in_sin_acc + in_cos_acc * in_cos_acc).sqrt() / measure_samples as f64;

    let gain_db = if in_amp > 1e-30 && out_amp > 1e-30 {
        20.0 * (out_amp / in_amp).log10()
    } else {
        -120.0
    };

    let out_phase = cos_acc.atan2(sin_acc);
    let in_phase = in_cos_acc.atan2(in_sin_acc);
    let phase_diff = (out_phase - in_phase).to_degrees();
    // Normalize to [-180, 180]
    let phase_deg = if phase_diff > 180.0 {
        phase_diff - 360.0
    } else if phase_diff < -180.0 {
        phase_diff + 360.0
    } else {
        phase_diff
    };

    (gain_db, phase_deg)
}

fn format_freq_response_csv(results: &[(f64, f64, f64)]) -> String {
    let mut csv = String::from("frequency_hz,gain_db,phase_deg\n");
    for &(freq, gain, phase) in results {
        csv.push_str(&format!("{:.2},{:.4},{:.2}\n", freq, gain, phase));
    }
    csv
}

/// Build DeviceEntry list for CircuitSolver from netlist + MNA.
fn build_device_entries(
    netlist: &melange_solver::parser::Netlist,
    mna: &melange_solver::mna::MnaSystem,
) -> Vec<melange_solver::solver::DeviceEntry> {
    use melange_devices::bjt::{BjtEbersMoll, BjtPolarity};
    use melange_devices::diode::DiodeShockley;
    use melange_devices::jfet::{Jfet, JfetChannel};
    use melange_devices::mosfet::{ChannelType, Mosfet};
    use melange_devices::tube::KorenTriode;
    use melange_solver::solver::DeviceEntry;

    let find_model_param = |model_name: &str, param: &str| -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param))
                    .map(|(_, v)| *v)
            })
    };

    let mut devices = Vec::new();
    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Diode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-15);
                let n = find_model_param(model_name, "N").unwrap_or(1.0);
                devices.push(DeviceEntry::new_diode(
                    DiodeShockley::new_room_temp(is, n),
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-14);
                let bf = find_model_param(model_name, "BF").unwrap_or(200.0);
                let br = find_model_param(model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                let polarity = if is_pnp {
                    BjtPolarity::Pnp
                } else {
                    BjtPolarity::Npn
                };
                let nf = find_model_param(model_name, "NF").unwrap_or(1.0);
                devices.push(DeviceEntry::new_bjt(
                    BjtEbersMoll::new(is, melange_primitives::VT_ROOM, bf, br, polarity)
                        .with_nf(nf),
                    dev_info.start_idx,
                ));
            }
            melange_solver::mna::NonlinearDeviceType::Jfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Jfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
                    .unwrap_or(false);
                let channel = if is_p_channel {
                    JfetChannel::P
                } else {
                    JfetChannel::N
                };
                let default_vp = if is_p_channel { 2.0 } else { -2.0 };
                let vp = find_model_param(model_name, "VTO").unwrap_or(default_vp);
                let idss = if let Some(beta) = find_model_param(model_name, "BETA") {
                    beta * vp * vp
                } else {
                    find_model_param(model_name, "IDSS").unwrap_or(2e-3)
                };
                let mut jfet = Jfet::new(channel, vp, idss);
                jfet.lambda = find_model_param(model_name, "LAMBDA").unwrap_or(0.001);
                devices.push(DeviceEntry::new_jfet(jfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Mosfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Mosfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PM"))
                    .unwrap_or(false);
                let channel = if is_p_channel {
                    ChannelType::P
                } else {
                    ChannelType::N
                };
                let default_vt = if is_p_channel { -2.0 } else { 2.0 };
                let vt = find_model_param(model_name, "VTO").unwrap_or(default_vt);
                let kp = find_model_param(model_name, "KP").unwrap_or(0.1);
                let lambda = find_model_param(model_name, "LAMBDA").unwrap_or(0.01);
                let mosfet = Mosfet::new(channel, vt, kp, lambda);
                devices.push(DeviceEntry::new_mosfet(mosfet, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::Tube => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Triode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let mu = find_model_param(model_name, "MU").unwrap_or(100.0);
                let ex = find_model_param(model_name, "EX").unwrap_or(1.4);
                let kg1 = find_model_param(model_name, "KG1").unwrap_or(1060.0);
                let kp = find_model_param(model_name, "KP").unwrap_or(600.0);
                let kvb = find_model_param(model_name, "KVB").unwrap_or(300.0);
                let ig_max = find_model_param(model_name, "IG_MAX").unwrap_or(2e-3);
                let vgk_onset = find_model_param(model_name, "VGK_ONSET").unwrap_or(0.5);
                let lambda = find_model_param(model_name, "LAMBDA").unwrap_or(0.0);
                let tube =
                    KorenTriode::with_all_params(mu, ex, kg1, kp, kvb, ig_max, vgk_onset, lambda);
                devices.push(DeviceEntry::new_tube(tube, dev_info.start_idx));
            }
            melange_solver::mna::NonlinearDeviceType::BjtForwardActive => {
                // BjtForwardActive uses the same runtime 2D model as Bjt
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.as_str())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or("");
                let is = find_model_param(model_name, "IS").unwrap_or(1e-14);
                let bf = find_model_param(model_name, "BF").unwrap_or(200.0);
                let br = find_model_param(model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                let polarity = if is_pnp {
                    BjtPolarity::Pnp
                } else {
                    BjtPolarity::Npn
                };
                let nf = find_model_param(model_name, "NF").unwrap_or(1.0);
                devices.push(DeviceEntry::new_bjt(
                    BjtEbersMoll::new(is, melange_primitives::VT_ROOM, bf, br, polarity)
                        .with_nf(nf),
                    dev_info.start_idx,
                ));
            }
        }
    }
    devices
}

fn build_device_slots(
    netlist: &melange_solver::parser::Netlist,
    mna: &melange_solver::mna::MnaSystem,
) -> Vec<melange_solver::codegen::ir::DeviceSlot> {
    use melange_solver::codegen::ir::{
        BjtParams, DeviceParams, DeviceSlot, DeviceType, DiodeParams, JfetParams, MosfetParams,
        TubeParams,
    };

    let find_param = |model_name: &str, param: &str| -> Option<f64> {
        netlist
            .models
            .iter()
            .find(|m| m.name.eq_ignore_ascii_case(model_name))
            .and_then(|m| {
                m.params
                    .iter()
                    .find(|(k, _)| k.eq_ignore_ascii_case(param))
                    .map(|(_, v)| *v)
            })
    };

    let mut slots = Vec::new();
    for dev_info in &mna.nonlinear_devices {
        match dev_info.device_type {
            melange_solver::mna::NonlinearDeviceType::Diode => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Diode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is = find_param(&model_name, "IS").unwrap_or(1e-15);
                let n = find_param(&model_name, "N").unwrap_or(1.0);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Diode,
                    start_idx: dev_info.start_idx,
                    dimension: 1,
                    params: DeviceParams::Diode(DiodeParams {
                        is,
                        n_vt: n * melange_primitives::VT_ROOM,
                        cjo: find_param(&model_name, "CJO").unwrap_or(0.0),
                        rs: find_param(&model_name, "RS").unwrap_or(0.0),
                        bv: find_param(&model_name, "BV").unwrap_or(f64::INFINITY),
                        ibv: find_param(&model_name, "IBV").unwrap_or(1e-10),
                    }),
                    has_internal_mna_nodes: false,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Bjt => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is = find_param(&model_name, "IS").unwrap_or(1e-14);
                let bf = find_param(&model_name, "BF").unwrap_or(200.0);
                let br = find_param(&model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Bjt,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Bjt(BjtParams {
                        is,
                        vt: melange_primitives::VT_ROOM,
                        beta_f: bf,
                        beta_r: br,
                        is_pnp,
                        vaf: f64::INFINITY,
                        var: f64::INFINITY,
                        ikf: f64::INFINITY,
                        ikr: f64::INFINITY,
                        cje: find_param(&model_name, "CJE").unwrap_or(0.0),
                        cjc: find_param(&model_name, "CJC").unwrap_or(0.0),
                        nf: find_param(&model_name, "NF").unwrap_or(1.0),
                        nr: find_param(&model_name, "NR").unwrap_or(1.0),
                        ise: find_param(&model_name, "ISE").unwrap_or(0.0),
                        ne: find_param(&model_name, "NE").unwrap_or(1.5),
                        isc: find_param(&model_name, "ISC").unwrap_or(0.0),
                        nc: find_param(&model_name, "NC").unwrap_or(2.0),
                        rb: find_param(&model_name, "RB").unwrap_or(0.0),
                        rc: find_param(&model_name, "RC").unwrap_or(0.0),
                        re: find_param(&model_name, "RE").unwrap_or(0.0),
                        rth: f64::INFINITY,
                        cth: 1e-3,
                        xti: 3.0,
                        eg: 1.11,
                        tamb: 300.15,
                    }),
                    has_internal_mna_nodes: false,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Jfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Jfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PJ"))
                    .unwrap_or(false);
                let default_vp = if is_p_channel { 2.0 } else { -2.0 };
                let vp = find_param(&model_name, "VTO").unwrap_or(default_vp);
                let idss = if let Some(beta) = find_param(&model_name, "BETA") {
                    beta * vp * vp
                } else {
                    find_param(&model_name, "IDSS").unwrap_or(2e-3)
                };
                let lambda = find_param(&model_name, "LAMBDA").unwrap_or(0.001);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Jfet,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Jfet(JfetParams {
                        idss,
                        vp,
                        lambda,
                        is_p_channel,
                        cgs: find_param(&model_name, "CGS").unwrap_or(0.0),
                        cgd: find_param(&model_name, "CGD").unwrap_or(0.0),
                        rd: find_param(&model_name, "RD").unwrap_or(0.0),
                        rs_param: find_param(&model_name, "RS").unwrap_or(0.0),
                    }),
                    has_internal_mna_nodes: false,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Mosfet => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Mosfet { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is_p_channel = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.to_uppercase().starts_with("PM"))
                    .unwrap_or(false);
                let default_vt = if is_p_channel { -2.0 } else { 2.0 };
                let vt = find_param(&model_name, "VTO").unwrap_or(default_vt);
                let kp = find_param(&model_name, "KP").unwrap_or(0.1);
                let lambda = find_param(&model_name, "LAMBDA").unwrap_or(0.01);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Mosfet,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Mosfet(MosfetParams {
                        kp,
                        vt,
                        lambda,
                        is_p_channel,
                        cgs: find_param(&model_name, "CGS").unwrap_or(0.0),
                        cgd: find_param(&model_name, "CGD").unwrap_or(0.0),
                        rd: find_param(&model_name, "RD").unwrap_or(0.0),
                        rs_param: find_param(&model_name, "RS").unwrap_or(0.0),
                        gamma: find_param(&model_name, "GAMMA").unwrap_or(0.0),
                        phi: find_param(&model_name, "PHI").unwrap_or(0.6),
                        source_node: 0,
                        bulk_node: 0,
                    }),
                    has_internal_mna_nodes: false,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Tube => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Triode { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let mu = find_param(&model_name, "MU").unwrap_or(100.0);
                let ex = find_param(&model_name, "EX").unwrap_or(1.4);
                let kg1 = find_param(&model_name, "KG1").unwrap_or(1060.0);
                let kp = find_param(&model_name, "KP").unwrap_or(600.0);
                let kvb = find_param(&model_name, "KVB").unwrap_or(300.0);
                let ig_max = find_param(&model_name, "IG_MAX").unwrap_or(2e-3);
                let vgk_onset = find_param(&model_name, "VGK_ONSET").unwrap_or(0.5);
                let lambda = find_param(&model_name, "LAMBDA").unwrap_or(0.0);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Tube,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Tube(TubeParams {
                        mu,
                        ex,
                        kg1,
                        kp,
                        kvb,
                        ig_max,
                        vgk_onset,
                        lambda,
                        ccg: find_param(&model_name, "CCG").unwrap_or(0.0),
                        cgp: find_param(&model_name, "CGP").unwrap_or(0.0),
                        ccp: find_param(&model_name, "CCP").unwrap_or(0.0),
                        rgi: find_param(&model_name, "RGI").unwrap_or(0.0),
                    }),
                    has_internal_mna_nodes: false,
                });
            }
            melange_solver::mna::NonlinearDeviceType::BjtForwardActive => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Bjt { name, model, .. } = e {
                            if name.eq_ignore_ascii_case(&dev_info.name) {
                                Some(model.clone())
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
                    .unwrap_or_default();
                let is = find_param(&model_name, "IS").unwrap_or(1e-14);
                let bf = find_param(&model_name, "BF").unwrap_or(200.0);
                let br = find_param(&model_name, "BR").unwrap_or(3.0);
                let is_pnp = netlist
                    .models
                    .iter()
                    .find(|m| m.name.eq_ignore_ascii_case(&model_name))
                    .map(|m| m.model_type.eq_ignore_ascii_case("PNP"))
                    .unwrap_or(false);
                slots.push(DeviceSlot {
                    device_type: DeviceType::BjtForwardActive,
                    start_idx: dev_info.start_idx,
                    dimension: 1,
                    params: DeviceParams::Bjt(BjtParams {
                        is,
                        vt: melange_primitives::VT_ROOM,
                        beta_f: bf,
                        beta_r: br,
                        is_pnp,
                        vaf: f64::INFINITY,
                        var: f64::INFINITY,
                        ikf: f64::INFINITY,
                        ikr: f64::INFINITY,
                        cje: find_param(&model_name, "CJE").unwrap_or(0.0),
                        cjc: find_param(&model_name, "CJC").unwrap_or(0.0),
                        nf: find_param(&model_name, "NF").unwrap_or(1.0),
                        nr: find_param(&model_name, "NR").unwrap_or(1.0),
                        ise: find_param(&model_name, "ISE").unwrap_or(0.0),
                        ne: find_param(&model_name, "NE").unwrap_or(1.5),
                        isc: find_param(&model_name, "ISC").unwrap_or(0.0),
                        nc: find_param(&model_name, "NC").unwrap_or(2.0),
                        rb: find_param(&model_name, "RB").unwrap_or(0.0),
                        rc: find_param(&model_name, "RC").unwrap_or(0.0),
                        re: find_param(&model_name, "RE").unwrap_or(0.0),
                        rth: f64::INFINITY,
                        cth: 1e-3,
                        xti: 3.0,
                        eg: 1.11,
                        tamb: 300.15,
                    }),
                    has_internal_mna_nodes: false,
                });
            }
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
        writer
            .write_sample(s as f32)
            .with_context(|| "Failed to write WAV sample")?;
    }
    writer
        .finalize()
        .with_context(|| "Failed to finalize WAV file")?;

    println!();
    println!("Output written to: {}", output.display());
    println!("  Samples: {}", samples.len());
    println!("  Duration: {:.2}s", samples.len() as f64 / sample_rate);
    println!(
        "  Peak level: {:.4} ({:.1} dB)",
        peak,
        if peak > 0.0 {
            20.0 * peak.log10()
        } else {
            f64::NEG_INFINITY
        }
    );

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
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read local file: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            let cache = cache::Cache::new()?;
            cache.get_sync(url, false)?
        }
    };

    let mut netlist =
        Netlist::parse(&netlist_str).with_context(|| "Failed to parse SPICE netlist")?;

    // Expand subcircuit instances (X elements) before MNA
    if !netlist.subcircuits.is_empty() {
        netlist
            .expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
    }

    let mna = MnaSystem::from_netlist(&netlist).with_context(|| "Failed to build MNA system")?;

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
    use crate::cache::{format_cache_list, Cache};

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
            Commands::Builtins => {}
            _ => panic!("Expected Builtins command"),
        }
    }
}
