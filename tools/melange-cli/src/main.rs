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
pub mod codegen_runner;
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

        /// Pentode grid-off dimension reduction mode.
        ///
        /// When a pentode's grid is biased well below cutoff at DC-OP, the
        /// Ig1 NR dimension can be dropped and Vg2k frozen, reducing M by 1
        /// per grid-off tube. This enables DK Schur for circuits that would
        /// otherwise exceed the M=16 cap (e.g. 4×EL34 Plexi: M=18 → M=14).
        ///
        /// Valid values:{n}{n}
        /// * auto — inspect DC-OP bias and reduce where Vgk < cutoff (default){n}{n}
        /// * on — force grid-off on every non-variable-mu pentode regardless of
        ///   bias. For testing / debugging only.{n}{n}
        /// * off — never reduce; all pentodes keep their full 3D NR block.
        ///   Use for regression parity with pre-1b codegen.
        #[arg(long, default_value = "auto")]
        tube_grid_fa: String,

        /// Op-amp supply rail saturation strategy.
        ///
        /// Controls how the generated solver models an op-amp's output hitting
        /// its supply rails. Different circuits need different trade-offs
        /// between numerical correctness, harmonic accuracy, and runtime cost.
        ///
        /// Valid values:{n}{n}
        /// * auto — inspect the circuit and pick the cheapest correct mode.
        ///   Logged at compile time so you can see what was chosen. Override
        ///   when bisecting issues.{n}{n}
        /// * none — no clamping at all (op-amp output is unbounded). Use only
        ///   for verified-linear circuits.{n}{n}
        /// * hard — post-NR v[out].clamp(VEE, VCC). Cheapest, matches
        ///   pre-2026-04 behavior. Breaks KCL for AC-coupled downstream caps
        ///   (see Klon investigation).{n}{n}
        /// * active-set — post-NR constrained re-solve. KCL-consistent hard
        ///   clip. Fixes Klon-class cap-history corruption. Still produces
        ///   square-wave harmonics.{n}{n}
        /// * boyle-diodes — auto-inserted physical catch diodes anchored to
        ///   rail-offset voltage sources. Matches commercial SPICE Boyle
        ///   macromodels. Produces soft exponential knee — best for
        ///   distortion pedals.
        #[arg(long, value_name = "MODE", default_value = "auto")]
        opamp_rail_mode: String,

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

        /// Plugin vendor name shown to the DAW (e.g., "Acme Audio").
        /// Defaults to "Melange" when omitted. Applies to `--format plugin`.
        #[arg(long, value_name = "STR")]
        vendor: Option<String>,

        /// Plugin vendor homepage URL (must start with http:// or https://).
        /// Defaults to "https://github.com/melange". Applies to `--format plugin`.
        #[arg(long, value_name = "URL")]
        vendor_url: Option<String>,

        /// Plugin vendor contact email (e.g., "support@acme.example").
        /// Defaults to "dev@melange.audio". Applies to `--format plugin`.
        #[arg(long, value_name = "ADDR")]
        email: Option<String>,

        /// Override the VST3 class ID with exactly 16 printable ASCII bytes.
        /// Without this flag the ID is derived from the circuit filename, so
        /// renaming the .cir file breaks DAW sessions. Pin an explicit value
        /// for stable releases. Applies to `--format plugin`.
        #[arg(long, value_name = "16-CHAR")]
        vst3_id: Option<String>,

        /// Override the CLAP plugin ID (reverse-DNS style, e.g.,
        /// "com.acme.wurli"). Defaults to "com.melange.<circuit>" when omitted.
        /// Applies to `--format plugin`.
        #[arg(long, value_name = "STR")]
        clap_id: Option<String>,
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

        /// Op-amp rail saturation mode: auto, none, hard, active-set, active-set-be, boyle-diodes.
        /// Default 'auto' inspects the topology and picks the cheapest correct mode.
        #[arg(long, default_value = "auto")]
        opamp_rail_mode: String,
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

    /// Import a KiCad netlist to Melange .cir format
    ///
    /// Supports KiCad XML intermediate netlist (full fidelity, preserves Melange.*
    /// custom fields), KiCad SPICE netlist (best-effort, standard components only),
    /// and .kicad_sch schematics (requires kicad-cli). Format is auto-detected.
    Import {
        /// Input file (KiCad XML .xml, SPICE .cir/.spice, or .kicad_sch schematic)
        input: PathBuf,

        /// Output Melange .cir file
        #[arg(short, long)]
        output: PathBuf,

        /// Input format override (auto-detected by default)
        #[arg(long, value_enum, default_value = "auto")]
        format: ImportFormat,

        /// Input is a .kicad_sch schematic file (shells out to kicad-cli)
        #[arg(long)]
        from_schematic: bool,
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

#[derive(ValueEnum, Clone, Debug, PartialEq)]
enum OutputFormat {
    /// Generate only the circuit code (default)
    Code,
    /// Generate a complete plugin project
    Plugin,
}

#[derive(ValueEnum, Clone, Debug)]
enum ImportFormat {
    /// Auto-detect from file content
    Auto,
    /// KiCad XML intermediate netlist (full fidelity)
    Xml,
    /// KiCad SPICE netlist (best-effort)
    Spice,
}

mod kicad_import;

fn main() -> Result<()> {
    // Initialize logger so log::info!/warn! from melange-solver are visible.
    // Default: only warnings. RUST_LOG=info or RUST_LOG=melange_solver=debug for more.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .format_target(false)
        .init();

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
            tube_grid_fa,
            opamp_rail_mode,
            name,
            mono,
            wet_dry_mix,
            no_ear_protection,
            vendor,
            vendor_url,
            email,
            vst3_id,
            clap_id,
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

            // Parse op-amp rail mode. Unknown values are user errors, not silent fallbacks.
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Unknown --opamp-rail-mode '{}'. Valid values: auto, none, hard, active-set, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;

            // Level params are on by default; --no-level-params disables them
            let level_params = with_level_params && !no_level_params;

            // Validate shipability flags (plugin branding).
            if let Some(url) = vendor_url.as_deref() {
                // Simple scheme check — we don't pull in a URL-parsing crate for this.
                if !(url.starts_with("http://") || url.starts_with("https://")) {
                    anyhow::bail!(
                        "--vendor-url '{}' must start with http:// or https://",
                        url
                    );
                }
            }
            if let Some(id) = vst3_id.as_deref() {
                plugin_template::validate_vst3_id(id)?;
            }
            if let Some(id) = clap_id.as_deref() {
                if id.is_empty() {
                    anyhow::bail!("--clap-id cannot be empty");
                }
            }

            let circuit_source = circuits::resolve(&input)?;
            println!("Resolved circuit: {}", circuit_source.name());
            // Validate tube-grid-fa mode.
            if !matches!(tube_grid_fa.as_str(), "auto" | "on" | "off") {
                anyhow::bail!(
                    "Unknown --tube-grid-fa '{}'. Valid values: auto, on, off",
                    tube_grid_fa
                );
            }

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
                &tube_grid_fa,
                rail_mode,
                name.as_deref(),
                mono,
                wet_dry_mix,
                !no_ear_protection,
                vendor.as_deref(),
                vendor_url.as_deref(),
                email.as_deref(),
                vst3_id.as_deref(),
                clap_id.as_deref(),
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
            opamp_rail_mode,
        } => {
            let rail_mode = melange_solver::codegen::OpampRailMode::parse(&opamp_rail_mode)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Invalid --opamp-rail-mode '{}'. Valid: auto, none, hard, \
                         active-set, active-set-be, boyle-diodes",
                        opamp_rail_mode
                    )
                })?;
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
                    opamp_rail_mode: rail_mode,
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
        Commands::Import {
            input,
            output,
            format,
            from_schematic,
        } => kicad_import::import_kicad(&input, &output, &format, from_schematic),
    }
}

/// Check if a capacitor is directly connected to the given output node.
/// Returns true if the circuit already has an output coupling cap, meaning
/// melange's built-in DC blocker is redundant.
fn has_output_coupling_cap(
    netlist: &melange_solver::parser::Netlist,
    output_node_name: &str,
) -> bool {
    use melange_solver::parser::Element;
    netlist.elements.iter().any(|elem| {
        matches!(elem, Element::Capacitor { n_plus, n_minus, .. }
            if n_plus == output_node_name || n_minus == output_node_name)
    })
}

/// Count nonlinear devices by type. Used to suggest oversampling.
fn count_nonlinear_devices(netlist: &melange_solver::parser::Netlist) -> (usize, usize, usize) {
    use melange_solver::parser::Element;
    let mut diodes = 0usize;
    let mut opamps = 0usize;
    let mut tubes = 0usize;
    for elem in &netlist.elements {
        match elem {
            Element::Diode { .. } => diodes += 1,
            Element::Opamp { .. } => opamps += 1,
            Element::Triode { .. } | Element::Pentode { .. } => tubes += 1,
            _ => {}
        }
    }
    (diodes, opamps, tubes)
}

#[allow(clippy::too_many_arguments)]
/// Suggest similar node names when a lookup fails.
/// Returns names that share a common prefix or contain the query as a substring.
fn suggest_node_names<'a>(query: &str, available: impl Iterator<Item = &'a String>) -> Vec<String> {
    let q = query.to_ascii_lowercase();
    let mut suggestions: Vec<(usize, String)> = Vec::new();
    for name in available {
        let n = name.to_ascii_lowercase();
        // Exact case-insensitive match
        if n == q {
            suggestions.push((0, name.clone()));
        }
        // One is a prefix of the other
        else if n.starts_with(&q) || q.starts_with(&n) {
            suggestions.push((1, name.clone()));
        }
        // Substring match
        else if n.contains(&q) || q.contains(&n) {
            suggestions.push((2, name.clone()));
        }
        // Common audio I/O aliases
        else {
            let input_aliases = ["in", "input", "vin", "audio_in", "sig_in", "mid_in"];
            let output_aliases = ["out", "output", "vout", "audio_out", "sig_out"];
            let q_is_input = input_aliases.contains(&q.as_str());
            let n_is_input = input_aliases.contains(&n.as_str());
            let q_is_output = output_aliases.contains(&q.as_str());
            let n_is_output = output_aliases.contains(&n.as_str());
            if (q_is_input && n_is_input) || (q_is_output && n_is_output) {
                suggestions.push((1, name.clone()));
            }
        }
    }
    suggestions.sort_by_key(|(score, _)| *score);
    suggestions.into_iter().map(|(_, name)| name).collect()
}

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
    tube_grid_fa: &str,
    opamp_rail_mode: melange_solver::codegen::OpampRailMode,
    plugin_name: Option<&str>,
    mono: bool,
    wet_dry_mix: bool,
    ear_protection: bool,
    vendor: Option<&str>,
    vendor_url: Option<&str>,
    email: Option<&str>,
    vst3_id_override: Option<&str>,
    clap_id_override: Option<&str>,
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
        let suggestions = suggest_node_names(input_node, mna.node_map.keys());
        let hint = if suggestions.is_empty() {
            format!("Available: {:?}", mna.node_map.keys().collect::<Vec<_>>())
        } else {
            format!("Did you mean: {}?", suggestions.join(", "))
        };
        anyhow::anyhow!(
            "Input node '{}' not found in circuit. {}", input_node, hint
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
    if !(input_resistance > 0.0 && input_resistance.is_finite()) {
        anyhow::bail!(
            "input resistance must be positive and finite, got {}",
            input_resistance
        );
    }
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

    // Phase 1b: detect grid-off pentodes (Vgk < -(vgk_onset + 0.5) →
    // pentode drops to 2D NR block with Vg2k frozen). Rebuilds MNA with
    // the reduced dimension. Only runs on DK solver — nodal doesn't
    // benefit from M-reduction at the solver level.
    // `--tube-grid-fa off` skips entirely; `on` forces all pentodes.
    let grid_off_pentodes = if tube_grid_fa == "off" || solver_override == "nodal" {
        std::collections::HashMap::new()
    } else {
        let force_all = tube_grid_fa == "on";
        melange_solver::codegen::ir::CircuitIR::detect_grid_off_pentodes(
            &mna, &netlist, &fa_config, force_all,
        )
    };
    if !grid_off_pentodes.is_empty() {
        // Sorted pretty-print: names + frozen Vg2k per slot. Useful when
        // diagnosing plexi-class circuits where the screen bias tells you
        // which tube dropped into cutoff and by how much.
        let mut pretty: Vec<(String, f64)> = grid_off_pentodes
            .iter()
            .map(|(k, v)| (k.clone(), *v))
            .collect();
        pretty.sort_by(|a, b| a.0.cmp(&b.0));
        let pretty_str: String = pretty
            .iter()
            .map(|(n, v)| format!("{n}(Vg2k={v:.1}V)"))
            .collect::<Vec<_>>()
            .join(", ");
        println!(
            "  Grid-off pentodes: [{}] (M reduces by {})",
            pretty_str,
            grid_off_pentodes.len()
        );
        // Rebuild MNA with reduced-dimension pentodes
        mna = MnaSystem::from_netlist_with_grid_off(&netlist, &grid_off_pentodes)
            .with_context(|| "Failed to rebuild MNA for grid-off pentodes")?;
        // Re-stamp input conductance (primary always grounded, but the rebuild
        // zeroes it out so we re-apply like the FA path does)
        if input_node_idx < mna.n {
            mna.g[input_node_idx][input_node_idx] += input_conductance;
        }
        // Re-stamp junction capacitances
        {
            let device_slots =
                melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
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
                    wiper_groups: vec![],
                    gang_groups: vec![],
                };
                (dummy, true)
            }
        }
    };

    // Step 4: Generate code
    println!("Step 4: Generating Rust code...");

    // Resolve the op-amp rail mode so we can print the auto-decision before
    // the emitter runs its own logs. This is a side-effect-free inspection —
    // the actual mode baked into the IR happens inside codegen (which does
    // the same resolution deterministically).
    {
        use melange_solver::codegen::ir::{
            refine_active_set_for_audio_path, resolve_opamp_rail_mode,
        };
        let resolved = refine_active_set_for_audio_path(
            resolve_opamp_rail_mode(&mna, opamp_rail_mode),
            &mna,
            &netlist,
        );
        println!(
            "  Op-amp rail mode: {} ({})",
            resolved.mode,
            resolved.reason.as_str()
        );
    }

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
            let suggestions = suggest_node_names(name, mna.node_map.keys());
            let hint = if suggestions.is_empty() {
                format!("Available: {:?}", mna.node_map.keys().collect::<Vec<_>>())
            } else {
                format!("Did you mean: {}?", suggestions.join(", "))
            };
            anyhow::anyhow!(
                "Output node '{}' not found in circuit. {}", name, hint
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

    // Auto-mono: single output node with single-channel circuit → default to mono.
    // Stereo duplicates the same mono circuit per channel, which is correct but
    // doubles CPU for no benefit unless the user has a stereo reason (e.g. wet/dry).
    let mono = if !mono && output_node_indices.len() == 1 && format == OutputFormat::Plugin {
        println!("  Auto-selecting mono (single output node). Use two output nodes for stereo.");
        true
    } else {
        mono
    };

    // DC-block auto-detection: if the circuit already has a coupling cap on the
    // output node, the built-in 5 Hz DC blocker is redundant (double-filtering
    // and adds 200ms settle time). Suggest --no-dc-block.
    let dc_block_auto_skip = !no_dc_block
        && output_node_names.iter().all(|name| has_output_coupling_cap(&netlist, name));
    if dc_block_auto_skip {
        println!(
            "  Output coupling cap detected on \"{}\". Consider --no-dc-block to avoid double filtering.",
            output_node_names.join(", ")
        );
    }

    // Oversampling suggestion: hard clippers (diodes) benefit most from oversampling.
    // Tubes and op-amps produce softer harmonics and alias less at 1×.
    if oversampling == 1 {
        let (diodes, _opamps, _tubes) = count_nonlinear_devices(&netlist);
        if diodes >= 4 {
            println!("  Hint: {} diodes detected. Consider --oversampling 4 to reduce aliasing.", diodes);
        } else if diodes >= 2 {
            println!("  Hint: {} diodes detected. Consider --oversampling 2 to reduce aliasing.", diodes);
        }
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
        opamp_rail_mode,
        ..CodegenConfig::default()
    };

    let generator = CodeGenerator::new(config);
    // Route solver: use library auto_route() for consistent logic, or honor --solver override.
    let routing = melange_solver::codegen::routing::auto_route(&kernel, &mna, dk_failed);
    let use_nodal_codegen = match solver_override {
        "nodal" => true,
        "dk" => false,
        _ => routing.route == melange_solver::codegen::routing::SolverRoute::Nodal,
    };
    let solver_label = if use_nodal_codegen { "nodal" } else { "DK" };
    let solver_reason = if solver_override == "nodal" || solver_override == "dk" {
        format!("--solver {} (user override)", solver_override)
    } else {
        routing.reason.clone()
    };

    // FA reduction is kept for nodal path when the DC OP confirms deeply
    // reverse-biased B-C junctions (Vbc < -0.5V). The FA detection threshold
    // is conservative enough that BJTs passing it have adequate margin for
    // audio-level transients. Previously this blanket-undid all FA for nodal,
    // but that forces M=16 for ladder filters where all BJTs are clearly FA.

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

    let line_count = generated.code.lines().count();
    println!("  ✓ Generated {} lines of Rust code", line_count);

    // Compilation summary: report all auto-detected decisions in one place.
    println!();
    println!("  Summary:");
    println!("    Circuit: {} nodes, {} nonlinear dimensions", generated.n, generated.m);
    println!("    Solver: {} ({})", solver_label, solver_reason);
    if routing.spectral_radius > 0.0 {
        println!("    Spectral radius: {:.4}", routing.spectral_radius);
    }
    println!("    Integration: {}", if backward_euler { "Backward Euler" } else { "Trapezoidal" });
    println!("    Oversampling: {}×", oversampling);
    println!("    Input: node \"{}\", resistance {}Ω ({})", input_node, input_resistance, ir_source);
    println!(
        "    Output: node \"{}\", scale {}",
        output_node_names.join(", "),
        output_scale
    );
    println!("    DC block: {}", if !no_dc_block { "enabled (5 Hz HPF)" } else { "disabled" });
    // DC operating point
    if generated.m > 0 {
        let meta = &generated.meta;
        if meta.dc_op_converged {
            println!("    DC operating point: converged ({}, {} iterations)", meta.dc_op_method, meta.dc_op_iterations);
        } else {
            println!("    DC operating point: *** DID NOT CONVERGE *** ({}, {} iterations)", meta.dc_op_method, meta.dc_op_iterations);
        }
        if meta.backward_euler_auto {
            println!("    Integration: Backward Euler (auto-selected, spectral radius > 1.002)");
        }
        if meta.parasitic_caps_inserted {
            println!("    Parasitic caps: auto-inserted (no capacitors in circuit)");
        }
    }
    if !forward_active.is_empty() {
        println!("    Forward-active reduction: {} BJTs linearized to 1D", forward_active.len());
    }
    if !grid_off_pentodes.is_empty() {
        println!("    Grid-off reduction: {} pentodes reduced to 2D", grid_off_pentodes.len());
    }
    if !linearized.is_empty() {
        println!("    Linearized BJTs: {} (fully linear, M reduced by {})", linearized.len(), linearized.len() * 2);
    }
    println!("    Generated: {} lines of Rust", line_count);
    println!();

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

            // Build wiper params from MNA wiper groups
            let wiper_params: Vec<plugin_template::WiperParamInfo> = mna
                .wiper_groups
                .iter()
                .enumerate()
                .map(|(idx, wg)| plugin_template::WiperParamInfo {
                    wiper_index: idx,
                    cw_pot_index: wg.cw_pot_index,
                    ccw_pot_index: wg.ccw_pot_index,
                    total_resistance: wg.total_resistance,
                    default_position: wg.default_position,
                    name: wg.label
                        .clone()
                        .unwrap_or_else(|| format!("Wiper {}", idx)),
                })
                .collect();

            // Build gang params from MNA gang groups
            let gang_params: Vec<plugin_template::GangParamInfo> = mna
                .gang_groups
                .iter()
                .enumerate()
                .map(|(idx, gg)| plugin_template::GangParamInfo {
                    index: idx,
                    label: gg.label.clone(),
                    default_position: gg.default_position,
                    pot_members: gg
                        .pot_members
                        .iter()
                        .map(|&(pot_idx, inverted)| {
                            (pot_idx, mna.pots[pot_idx].min_resistance, mna.pots[pot_idx].max_resistance, inverted)
                        })
                        .collect(),
                    wiper_members: gg
                        .wiper_members
                        .iter()
                        .map(|&(wg_idx, inverted)| {
                            let wg = &mna.wiper_groups[wg_idx];
                            (wg.cw_pot_index, wg.ccw_pot_index, wg.total_resistance, inverted)
                        })
                        .collect(),
                })
                .collect();

            // Collect pot indices claimed by gangs
            let gang_claimed_pots: std::collections::HashSet<usize> = gang_params
                .iter()
                .flat_map(|g| g.pot_members.iter().map(|&(idx, _, _, _)| idx))
                .collect();
            let gang_claimed_wipers: std::collections::HashSet<usize> = gang_params
                .iter()
                .flat_map(|g| {
                    g.wiper_members
                        .iter()
                        .flat_map(|&(cw, ccw, _, _)| [cw, ccw])
                })
                .collect();

            // Filter out wiper-claimed and gang-claimed pots from individual pot params
            let wiper_claimed: std::collections::HashSet<usize> = wiper_params
                .iter()
                .flat_map(|w| [w.cw_pot_index, w.ccw_pot_index])
                .collect();
            let pot_params: Vec<_> = pot_params
                .into_iter()
                .filter(|p| {
                    !wiper_claimed.contains(&p.index) && !gang_claimed_pots.contains(&p.index)
                })
                .collect();

            // Filter out gang-claimed wipers from individual wiper params
            let wiper_params: Vec<_> = wiper_params
                .into_iter()
                .filter(|w| {
                    !gang_claimed_wipers.contains(&w.cw_pot_index)
                })
                .collect();

            let plugin_options = plugin_template::PluginOptions {
                plugin_name,
                mono,
                wet_dry_mix,
                ear_protection,
                vendor,
                url: vendor_url,
                email,
                vst3_id: vst3_id_override,
                clap_id: clap_id_override,
            };
            plugin_template::generate_plugin_project_with_oversampling(
                &project_dir,
                &generated.code,
                &circuit_name,
                with_level_params,
                &pot_params,
                &wiper_params,
                &gang_params,
                &switch_params,
                output_node_indices.len(),
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
            println!("  bash build.sh");
            println!();
            println!("One-time setup (clone nih-plug if you haven't):");
            println!("  git clone https://github.com/robbert-vdh/nih-plug.git ~/src/nih-plug");
            println!();
            println!("The compiled plugin (CLAP + VST3) will be in:");
            println!("  target/bundled/");
            println!();
            println!("See the generated README.md for full details.");
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
    // For builtins/URLs, write to a secure temp file (random name, auto-cleanup on drop).
    // Uses tempfile::NamedTempFile to avoid TOCTOU/symlink clobber attacks from
    // predictable PID-based paths on shared hosts.
    println!("Step 2: Loading circuit...");
    use std::io::Write as _;
    let (netlist_path, _temp_file): (std::path::PathBuf, Option<tempfile::NamedTempFile>) =
        match circuit_source {
            circuits::CircuitSource::Local { path } => {
                // Verify the file exists
                if !path.exists() {
                    anyhow::bail!("Circuit file not found: {}", path.display());
                }
                (path.clone(), None)
            }
            circuits::CircuitSource::Builtin { content, name } => {
                println!("  Using builtin circuit: {}", name);
                let mut tmp = tempfile::Builder::new()
                    .prefix("melange_validate_")
                    .suffix(".cir")
                    .tempfile()
                    .context("Failed to create temp netlist file")?;
                tmp.write_all(content.as_bytes())
                    .context("Failed to write temp netlist")?;
                tmp.flush().context("Failed to flush temp netlist")?;
                let path = tmp.path().to_path_buf();
                (path, Some(tmp))
            }
            circuits::CircuitSource::Url { url }
            | circuits::CircuitSource::Friendly { url, .. } => {
                println!("  Fetching from URL: {}", url);
                let cache = cache::Cache::new()?;
                let content = cache.get_sync(url, false)?;
                let mut tmp = tempfile::Builder::new()
                    .prefix("melange_validate_")
                    .suffix(".cir")
                    .tempfile()
                    .context("Failed to create temp netlist file")?;
                tmp.write_all(content.as_bytes())
                    .context("Failed to write temp netlist")?;
                tmp.flush().context("Failed to flush temp netlist")?;
                let path = tmp.path().to_path_buf();
                (path, Some(tmp))
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

    // _temp_file drops here, auto-cleaning the NamedTempFile on function exit.
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
    opamp_rail_mode: melange_solver::codegen::OpampRailMode,
}

fn simulate_circuit_source(
    circuit_source: &circuits::CircuitSource,
    opts: &SimulateOptions,
) -> Result<()> {
    use melange_solver::{
        codegen::{CodeGenerator, CodegenConfig, routing},
        dk::DkKernel,
        mna::MnaSystem,
        parser::Netlist,
    };

    println!("melange simulate");
    println!("  Source: {}", circuit_source.name());
    println!();

    // Step 1: Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, name } => {
            println!("  Using builtin circuit: {}", name);
            content.clone()
        }
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            println!("  Fetching: {}", url);
            cache::Cache::new()?.get_sync(url, false)?
        }
    };

    // Step 2: Parse netlist
    println!("Step 1: Parsing SPICE netlist...");
    let mut netlist = Netlist::parse(&netlist_str)
        .with_context(|| "Failed to parse SPICE netlist")?;
    if !netlist.subcircuits.is_empty() {
        netlist.expand_subcircuits()
            .with_context(|| "Failed to expand subcircuits")?;
    }
    println!("  {} elements", netlist.elements.len());

    // Step 3: Build MNA
    println!("Step 2: Building MNA system...");
    let mut mna = MnaSystem::from_netlist(&netlist)
        .with_context(|| "Failed to build MNA system")?;

    let input_node_raw = mna.node_map.get(opts.input_node).copied().ok_or_else(|| {
        anyhow::anyhow!("Input node '{}' not found. Available: {:?}",
            opts.input_node, mna.node_map.keys().collect::<Vec<_>>())
    })?;
    if input_node_raw == 0 { anyhow::bail!("Input node cannot be ground"); }
    let input_node_idx = input_node_raw - 1;

    let output_node_raw = mna.node_map.get(opts.output_node).copied().ok_or_else(|| {
        anyhow::anyhow!("Output node '{}' not found. Available: {:?}",
            opts.output_node, mna.node_map.keys().collect::<Vec<_>>())
    })?;
    if output_node_raw == 0 { anyhow::bail!("Output node cannot be ground"); }
    let output_node_idx = output_node_raw - 1;

    // Input resistance
    let (input_resistance, ir_source) = if let Some(r) = opts.input_resistance_flag {
        (r, "from --input-resistance flag")
    } else if let Some(r) = netlist.input_impedance {
        (r, "from .input_impedance directive")
    } else {
        (1.0, "default")
    };
    if !(input_resistance > 0.0 && input_resistance.is_finite()) {
        anyhow::bail!("input resistance must be positive and finite, got {}", input_resistance);
    }
    println!("  Input resistance: {} ohm ({})", input_resistance, ir_source);
    let input_conductance = 1.0 / input_resistance;
    if input_node_idx < mna.n {
        mna.g[input_node_idx][input_node_idx] += input_conductance;
    }

    println!("  {} nodes, {} nonlinear devices", mna.n, mna.nonlinear_devices.len());

    // Stamp junction caps
    {
        let device_slots = melange_solver::codegen::ir::CircuitIR::build_device_info(&netlist)
            .unwrap_or_default();
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    // Detect forward-active BJTs (runs DC OP, checks Vbc < -0.5V)
    let config_for_fa = CodegenConfig {
        circuit_name: "fa_detect".to_string(),
        sample_rate: opts.sample_rate,
        input_resistance,
        input_node: input_node_idx,
        output_nodes: vec![output_node_idx],
        ..CodegenConfig::default()
    };
    let forward_active = if opts.solver == "nodal" {
        std::collections::HashSet::new()
    } else {
        melange_solver::codegen::ir::CircuitIR::detect_forward_active_bjts(
            &mna, &netlist, &config_for_fa,
        )
    };
    if !forward_active.is_empty() {
        println!(
            "  Forward-active BJTs: {:?} (M reduces by {})",
            forward_active,
            forward_active.len()
        );
        mna = MnaSystem::from_netlist_forward_active(&netlist, &forward_active)
            .with_context(|| "Failed to rebuild MNA for forward-active BJTs")?;
        if input_node_idx < mna.n {
            mna.g[input_node_idx][input_node_idx] += input_conductance;
        }
        let device_slots =
            melange_solver::codegen::ir::CircuitIR::build_device_info(&netlist).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    // Step 4: Build DK kernel and route
    let has_inductors = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();

    println!("Step 3: Building DK kernel...");
    let mut dk_failed = false;
    let kernel = if has_inductors && opts.solver != "dk" {
        match DkKernel::from_mna_augmented(&mna, opts.sample_rate) {
            Ok(k) => k,
            Err(e) => {
                if opts.solver == "dk" { anyhow::bail!("DK kernel failed: {e}"); }
                println!("  Augmented DK kernel failed: {e}, auto-selecting nodal");
                dk_failed = true;
                let m = mna.m;
                let n = mna.n_aug;
                DkKernel {
                    n, m, n_nodes: mna.n, num_devices: mna.num_devices,
                    sample_rate: opts.sample_rate,
                    s: vec![0.0; n * n], a_neg: vec![0.0; n * n],
                    k: vec![0.0; m * m], n_v: vec![0.0; m * n],
                    n_i: vec![0.0; n * m], rhs_const: vec![0.0; n],
                    inductors: vec![], coupled_inductors: vec![],
                    transformer_groups: vec![], pots: vec![],
                    wiper_groups: vec![], gang_groups: vec![],
                }
            }
        }
    } else {
        match DkKernel::from_mna(&mna, opts.sample_rate) {
            Ok(k) => k,
            Err(e) => {
                if opts.solver == "dk" { anyhow::bail!("DK kernel failed: {e}"); }
                println!("  DK kernel failed: {e}, auto-selecting nodal");
                dk_failed = true;
                // Try augmented; if that also fails, build dummy kernel
                match DkKernel::from_mna_augmented(&mna, opts.sample_rate) {
                    Ok(k) => k,
                    Err(_) => {
                        let m = mna.m;
                        let n = mna.n_aug;
                        DkKernel {
                            n, m, n_nodes: mna.n, num_devices: mna.num_devices,
                            sample_rate: opts.sample_rate,
                            s: vec![0.0; n * n], a_neg: vec![0.0; n * n],
                            k: vec![0.0; m * m], n_v: vec![0.0; m * n],
                            n_i: vec![0.0; n * m], rhs_const: vec![0.0; n],
                            inductors: vec![], coupled_inductors: vec![],
                            transformer_groups: vec![], pots: vec![],
                            wiper_groups: vec![], gang_groups: vec![],
                        }
                    }
                }
            }
        }
    };
    println!("  N={}, M={}", kernel.n, kernel.m);

    // Route: DK or nodal
    let decision = routing::auto_route(&kernel, &mna, dk_failed);
    let use_nodal = match opts.solver {
        "nodal" => true,
        "dk" => false,
        _ => decision.route == routing::SolverRoute::Nodal,
    };
    println!("  Solver: {} ({})", if use_nodal { "nodal" } else { "DK" }, decision.reason);

    // When auto-routing to nodal, expand BJT internal nodes
    if use_nodal && opts.solver != "nodal" {
        let device_slots = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
            &netlist, Some(&mna),
        ).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.expand_bjt_internal_nodes(&device_slots);
        }
    }

    // Step 5: Generate circuit code
    println!("Step 4: Generating code...");
    let config = CodegenConfig {
        circuit_name: "simulate".to_string(),
        sample_rate: opts.sample_rate,
        max_iterations: 100,
        tolerance: 1e-9,
        input_resistance,
        input_node: input_node_idx,
        output_nodes: vec![output_node_idx],
        oversampling_factor: 1,
        output_scales: vec![1.0],
        include_dc_op: true,
        dc_op_max_iterations: 200,
        dc_op_tolerance: 1e-9,
        dc_block: false, // preserve DC for accurate WAV output
        pot_settle_samples: 64,
        backward_euler: false,
        disable_be_fallback: false,
        opamp_rail_mode: opts.opamp_rail_mode,
    };
    let generator = CodeGenerator::new(config);
    let generated = if use_nodal {
        generator.generate_nodal(&mna, &netlist)
            .with_context(|| "Nodal codegen failed")?
    } else {
        generator.generate(&kernel, &mna, &netlist)
            .with_context(|| "DK codegen failed")?
    };
    println!("  {} lines of code", generated.code.lines().count());

    // Step 6: Append simulate main, compile, run
    println!("Step 5: Compiling and running...");
    let simulate_main = codegen_runner::generate_simulate_main(
        opts.sample_rate,
        &[], // pot overrides (future)
        &[], // switch overrides (future)
        if opts.input_audio.is_none() { Some(opts.amplitude) } else { None },
        1000.0, // test tone freq
        opts.duration,
    );
    let full_source = format!("{}\n{}", generated.code, simulate_main);

    let binary_cache = codegen_runner::BinaryCache::new()
        .with_context(|| "Failed to create binary cache")?;
    let compiled = binary_cache.compile(&full_source, "simulate")
        .with_context(|| "Compilation failed")?;
    if compiled.cached {
        println!("  Using cached binary");
    } else {
        println!("  Compiled successfully");
    }

    // Run the binary
    let mut cmd = std::process::Command::new(&compiled.path);
    if let Some(audio_path) = opts.input_audio {
        cmd.arg(audio_path.to_str().unwrap_or("input.wav"));
    } else {
        cmd.arg("--tone");
    }
    cmd.arg(opts.output.to_str().unwrap_or("output.wav"));

    let result = cmd.output()
        .with_context(|| "Failed to run compiled binary")?;

    if !result.status.success() {
        let stderr = String::from_utf8_lossy(&result.stderr);
        anyhow::bail!("Simulate binary failed:\n{}", stderr);
    }

    // Parse diagnostics from stderr
    let stderr = String::from_utf8_lossy(&result.stderr);
    for line in stderr.lines() {
        if let Some(diag) = line.strip_prefix("DIAG:") {
            let parts: Vec<&str> = diag.splitn(2, '=').collect();
            if parts.len() == 2 {
                println!("  {}: {}", parts[0], parts[1]);
            }
        }
    }

    println!();
    println!("Output written to: {}", opts.output.display());
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
        codegen::{CodeGenerator, CodegenConfig, routing},
        dk::DkKernel,
        mna::MnaSystem,
        parser::Netlist,
    };

    eprintln!("melange analyze (frequency response)");

    // Get circuit content
    let netlist_str = match circuit_source {
        circuits::CircuitSource::Builtin { content, name } => {
            eprintln!("  Using builtin circuit: {}", name);
            content.clone()
        }
        circuits::CircuitSource::Local { path } => std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read: {}", path.display()))?,
        circuits::CircuitSource::Url { url } | circuits::CircuitSource::Friendly { url, .. } => {
            cache::Cache::new()?.get_sync(url, false)?
        }
    };

    let mut netlist = Netlist::parse(&netlist_str)
        .with_context(|| "Failed to parse SPICE netlist")?;
    if !netlist.subcircuits.is_empty() {
        netlist.expand_subcircuits()
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
            // Also update PotDirective.default_value so MNA pot_default_overrides
            // doesn't override the element value we just set.
            for p in netlist.pots.iter_mut() {
                if p.resistor_name.eq_ignore_ascii_case(&resistor_name) {
                    p.default_value = Some(value);
                    break;
                }
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

    // Build MNA
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

    // Input resistance
    let (input_resistance, ir_source) = if let Some(r) = input_resistance_flag {
        (r, "from --input-resistance")
    } else if let Some(r) = netlist.input_impedance {
        (r, "from .input_impedance directive")
    } else {
        (1.0, "default")
    };
    if !(input_resistance > 0.0 && input_resistance.is_finite()) {
        anyhow::bail!(
            "input resistance must be positive and finite, got {}",
            input_resistance
        );
    }
    eprintln!("  Input resistance: {} ohm ({})", input_resistance, ir_source);
    let input_conductance = 1.0 / input_resistance;
    if input_node_idx < mna.n {
        mna.g[input_node_idx][input_node_idx] += input_conductance;
    }

    // Stamp junction caps
    {
        let device_slots =
            melange_solver::codegen::ir::CircuitIR::build_device_info(&netlist).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    // Detect forward-active BJTs
    let config_for_fa = CodegenConfig {
        circuit_name: "fa_detect".to_string(),
        sample_rate,
        input_resistance,
        input_node: input_node_idx,
        output_nodes: vec![output_node_idx],
        ..CodegenConfig::default()
    };
    let forward_active =
        melange_solver::codegen::ir::CircuitIR::detect_forward_active_bjts(
            &mna, &netlist, &config_for_fa,
        );
    if !forward_active.is_empty() {
        eprintln!(
            "  Forward-active BJTs: {:?} (M reduces by {})",
            forward_active,
            forward_active.len()
        );
        mna = MnaSystem::from_netlist_forward_active(&netlist, &forward_active)
            .with_context(|| "Failed to rebuild MNA for forward-active BJTs")?;
        if input_node_idx < mna.n {
            mna.g[input_node_idx][input_node_idx] += input_conductance;
        }
        let device_slots =
            melange_solver::codegen::ir::CircuitIR::build_device_info(&netlist).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.stamp_device_junction_caps(&device_slots);
        }
    }

    // Build DK kernel and route
    let has_inductors = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();

    let mut dk_failed = false;
    let kernel = if has_inductors {
        match DkKernel::from_mna_augmented(&mna, sample_rate) {
            Ok(k) => k,
            Err(e) => {
                eprintln!("  Augmented DK kernel failed: {e}, auto-selecting nodal");
                dk_failed = true;
                let m = mna.m;
                let n = mna.n_aug;
                DkKernel {
                    n, m, n_nodes: mna.n, num_devices: mna.num_devices,
                    sample_rate,
                    s: vec![0.0; n * n], a_neg: vec![0.0; n * n],
                    k: vec![0.0; m * m], n_v: vec![0.0; m * n],
                    n_i: vec![0.0; n * m], rhs_const: vec![0.0; n],
                    inductors: vec![], coupled_inductors: vec![],
                    transformer_groups: vec![], pots: vec![],
                    wiper_groups: vec![], gang_groups: vec![],
                }
            }
        }
    } else {
        match DkKernel::from_mna(&mna, sample_rate) {
            Ok(k) => k,
            Err(e) => {
                eprintln!("  DK kernel failed: {e}, using nodal");
                dk_failed = true;
                match DkKernel::from_mna_augmented(&mna, sample_rate) {
                    Ok(k) => k,
                    Err(_) => {
                        let m = mna.m;
                        let n = mna.n_aug;
                        DkKernel {
                            n, m, n_nodes: mna.n, num_devices: mna.num_devices,
                            sample_rate,
                            s: vec![0.0; n * n], a_neg: vec![0.0; n * n],
                            k: vec![0.0; m * m], n_v: vec![0.0; m * n],
                            n_i: vec![0.0; n * m], rhs_const: vec![0.0; n],
                            inductors: vec![], coupled_inductors: vec![],
                            transformer_groups: vec![], pots: vec![],
                            wiper_groups: vec![], gang_groups: vec![],
                        }
                    }
                }
            }
        }
    };

    let decision = routing::auto_route(&kernel, &mna, dk_failed);
    let use_nodal = decision.route == routing::SolverRoute::Nodal;
    eprintln!("  N={}, M={}, solver: {}", kernel.n, kernel.m, decision.reason);

    if use_nodal {
        let device_slots = melange_solver::codegen::ir::CircuitIR::build_device_info_with_mna(
            &netlist, Some(&mna),
        ).unwrap_or_default();
        if !device_slots.is_empty() {
            mna.expand_bjt_internal_nodes(&device_slots);
        }
    }

    // Generate circuit code
    let config = CodegenConfig {
        circuit_name: "analyze".to_string(),
        sample_rate,
        max_iterations: 100,
        tolerance: 1e-9,
        input_resistance,
        input_node: input_node_idx,
        output_nodes: vec![output_node_idx],
        oversampling_factor: 1,
        output_scales: vec![1.0],
        include_dc_op: true,
        dc_op_max_iterations: 200,
        dc_op_tolerance: 1e-9,
        dc_block: false,
        pot_settle_samples: 64,
        backward_euler: false,
        disable_be_fallback: false,
        opamp_rail_mode: melange_solver::codegen::OpampRailMode::Auto,
    };
    let generator = CodeGenerator::new(config);
    let generated = if use_nodal {
        generator.generate_nodal(&mna, &netlist)
            .with_context(|| "Nodal codegen failed")?
    } else {
        generator.generate(&kernel, &mna, &netlist)
            .with_context(|| "DK codegen failed")?
    };

    // Generate frequency list
    let frequencies = generate_log_frequencies(start_freq, end_freq, points_per_decade);
    eprintln!("  {} frequency points from {:.0} Hz to {:.0} Hz", frequencies.len(), start_freq, end_freq);

    // Determine settle time
    let settle_secs = if has_inductors { 5.0 } else { 0.5 };

    // Append analyze main, compile, run
    let analyze_main = codegen_runner::generate_analyze_main(
        &frequencies,
        amplitude,
        sample_rate,
        settle_secs,
        &[], // pot calls (already baked into netlist)
        &[], // switch calls (already baked into netlist)
    );
    let full_source = format!("{}\n{}", generated.code, analyze_main);

    let binary_cache = codegen_runner::BinaryCache::new()
        .with_context(|| "Failed to create binary cache")?;
    let compiled = binary_cache.compile(&full_source, "analyze")
        .with_context(|| "Compilation failed")?;
    if compiled.cached {
        eprintln!("  Using cached binary");
    }

    let result = std::process::Command::new(&compiled.path)
        .output()
        .with_context(|| "Failed to run analyze binary")?;

    if !result.status.success() {
        let stderr = String::from_utf8_lossy(&result.stderr);
        anyhow::bail!("Analyze binary failed:\n{}", stderr);
    }

    let stdout = String::from_utf8_lossy(&result.stdout);

    // Print per-frequency diagnostics from stderr
    let stderr = String::from_utf8_lossy(&result.stderr);
    for line in stderr.lines() {
        eprintln!("{}", line);
    }

    // Output CSV
    if let Some(out_path) = output_file {
        std::fs::write(out_path, stdout.as_bytes())
            .with_context(|| format!("Failed to write: {}", out_path.display()))?;
        eprintln!("  Results written to: {}", out_path.display());
    } else {
        print!("{}", stdout);
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

// build_device_entries and build_device_slots removed — runtime solvers replaced by codegen.
// write_wav removed — WAV I/O is now handled by compiled circuit binaries.
#[cfg(any())]
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
            melange_solver::mna::NonlinearDeviceType::Vca => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Vca { name, model, .. } = e {
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
                let vscale = find_model_param(model_name, "VSCALE").unwrap_or(0.05298);
                let g0 = find_model_param(model_name, "G0").unwrap_or(1.0);
                let vca = melange_devices::Vca::new(vscale, g0);
                devices.push(DeviceEntry::new_vca(vca, dev_info.start_idx));
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

#[cfg(any())]
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
            vg2k_frozen: 0.0,
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
            vg2k_frozen: 0.0,
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
                        rs: find_param(&model_name, "RS").unwrap_or(0.0),
                    }),
                    has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
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
                        rs: find_param(&model_name, "RS").unwrap_or(0.0),
                        gamma: find_param(&model_name, "GAMMA").unwrap_or(0.0),
                        phi: find_param(&model_name, "PHI").unwrap_or(0.6),
                        source_node: 0,
                        bulk_node: 0,
                    }),
                    has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
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
                        kind: melange_solver::device_types::TubeKind::SharpTriode,
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
                        kg2: 0.0,
                        alpha_s: 0.0,
                        a_factor: 0.0,
                        beta_factor: 0.0,
                        screen_form: melange_solver::device_types::ScreenForm::Rational,
                        mu_b: 0.0,
                        svar: 0.0,
                        ex_b: 0.0,
                    }),
                    has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
                });
            }
            melange_solver::mna::NonlinearDeviceType::Vca => {
                let model_name = netlist
                    .elements
                    .iter()
                    .find_map(|e| {
                        if let melange_solver::parser::Element::Vca { name, model, .. } = e {
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
                let vscale = find_param(&model_name, "VSCALE").unwrap_or(0.05298);
                let g0 = find_param(&model_name, "G0").unwrap_or(1.0);
                let thd = find_param(&model_name, "THD").unwrap_or(0.0);
                slots.push(DeviceSlot {
                    device_type: DeviceType::Vca,
                    start_idx: dev_info.start_idx,
                    dimension: 2,
                    params: DeviceParams::Vca(melange_solver::VcaParams {
                        vscale,
                        g0,
                        thd,
                    }),
                    has_internal_mna_nodes: false,
            vg2k_frozen: 0.0,
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
            vg2k_frozen: 0.0,
                });
            }
        }
    }
    slots
}

#[cfg(any())]
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
    use crate::codegen_runner::BinaryCache;

    match action {
        CacheAction::List => {
            let cache = Cache::new()?;
            println!("{}", format_cache_list(&cache));
            let bin_cache = BinaryCache::new()?;
            let bin_stats = bin_cache.stats();
            println!();
            println!("Compiled binaries:");
            println!("  {} files ({})", bin_stats.total_files, bin_stats.formatted_size());
            Ok(())
        }
        CacheAction::Clear => {
            let cache = Cache::new()?;
            cache.clear()?;
            let bin_cache = BinaryCache::new()?;
            bin_cache.clear()?;
            println!("Cache cleared (circuits + compiled binaries).");
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
            println!("Binary cache:");
            println!("  Files: {}", bin_stats.total_files);
            println!("  Size: {}", bin_stats.formatted_size());
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
