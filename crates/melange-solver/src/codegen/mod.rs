//! Code generation for specialized circuit solvers.
//!
//! This module generates zero-overhead Rust code from a compiled DkKernel.
//! The generated code is specialized for a specific circuit topology with
//! compile-time constant matrices and unrolled loops.
//!
//! ## Architecture
//!
//! ```text
//! Netlist → MNA → DkKernel → CircuitIR → Emitter → Source Code
//! ```
//!
//! - [`ir::CircuitIR`] — serializable, language-agnostic intermediate representation
//! - [`emitter::Emitter`] — trait that language backends implement
//! - [`rust_emitter::RustEmitter`] — Rust language backend

#[cfg(feature = "codegen")]
pub mod emitter;
#[cfg(feature = "codegen")]
pub mod ir;
#[cfg(feature = "codegen")]
pub mod routing;
#[cfg(feature = "codegen")]
pub mod rust_emitter;

#[cfg(feature = "codegen")]
use crate::dk::DkKernel;
#[cfg(feature = "codegen")]
use crate::mna::MnaSystem;
#[cfg(feature = "codegen")]
use crate::parser::Netlist;

#[cfg(feature = "codegen")]
use emitter::Emitter;
#[cfg(feature = "codegen")]
use ir::CircuitIR;
#[cfg(feature = "codegen")]
use rust_emitter::RustEmitter;

/// Strategy for modeling op-amp output saturation at the supply rails.
///
/// Real op-amps can't drive their output past their supply rails; in melange,
/// the linear VCCS model (`Gm = AOL/ROUT`) has unbounded output and requires
/// an explicit rail-saturation mechanism. Different circuits need different
/// mechanisms because they have different trade-offs between numerical fidelity,
/// harmonic fidelity, and runtime cost.
///
/// # Auto-selection (default)
///
/// When set to [`Auto`](OpampRailMode::Auto), codegen inspects the circuit and
/// picks the cheapest correct mode. The decision is logged at compile time so
/// users can see what was chosen and why. Override with one of the explicit
/// variants when bisecting issues or measuring.
///
/// # Variant semantics
///
/// - [`None`](OpampRailMode::None): no clamping at all. Op-amp output is
///   unbounded; circuit must never drive the op-amp into saturation. Suitable
///   only for verified-linear circuits (clean mixers, buffers, flat EQs).
///
/// - [`Hard`](OpampRailMode::Hard): post-NR `v[out].clamp(VEE, VCC)`. Cheapest,
///   matches pre-2026-04 behavior. **Breaks KCL** for any cap connecting the
///   clamped node to another node — downstream AC-coupled integrators will
///   drift to physically impossible values. Only safe when every op-amp
///   output is DC-coupled to its downstream load (no series coupling cap).
///
/// - [`ActiveSet`](OpampRailMode::ActiveSet): post-NR constrained re-solve.
///   After NR converges, any clamped node is pinned via row replacement and
///   the rest of the network is re-solved to match. KCL-consistent. Fixes
///   the Klon-class cap-history corruption. Cost: one extra LU back-solve
///   (O(N²)) on samples where clamping is active. Still produces hard-clip
///   harmonics — fine for utility clamping, not ideal for distortion pedals.
///
/// - [`BoyleDiodes`](OpampRailMode::BoyleDiodes): auto-inserted catch diodes
///   per op-amp, anchored to rail-offset voltage sources at `VCC − VOH_DROP`
///   and `VEE + VOL_DROP`. Matches the Boyle macromodel used by every
///   commercial SPICE and produces the soft exponential knee characteristic
///   of real op-amp output stages. Most accurate for distortion circuits
///   (Klon, Tube Screamer, etc.). Cost: +2 N and +2 M per op-amp, plus the
///   synthesized voltage sources' augmented rows.
#[cfg(feature = "codegen")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum OpampRailMode {
    /// Auto-select based on circuit topology. See type-level docs.
    Auto,
    /// No clamping — op-amp output is unbounded (only safe for linear circuits).
    None,
    /// Post-NR hard clamp (pre-2026-04 behavior). Breaks KCL for AC-coupled downstream.
    Hard,
    /// Post-NR constrained re-solve on trapezoidal `state.a`. KCL-consistent hard
    /// clip with square-wave harmonics. Preserves the steady DC rail value the
    /// op-amp converged to — required for circuits where the rail-clipped value
    /// IS the signal (e.g. control-path op-amps driving a VCA gain port).
    ActiveSet,
    /// On rail engagement, fall through to the BE NR fallback and run the
    /// constrained re-solve against `state.a_be` (backward Euler). Trapezoidal
    /// + post-NR pin develops a Nyquist-rate limit cycle when the clamp is
    /// engaged across multiple samples (the cap-history term `(2/T)·C·v_prev`
    /// alternates sign every sample); BE damps this. Required for audio-path
    /// op-amps whose output is cap-coupled to a downstream stage that
    /// integrates the op-amp's transient behavior — e.g. Klon Centaur's
    /// tone-out → C15 → output. Cost: BE NR runs every sample where the rail
    /// is engaged (~2x NR work for those samples). Compared to ActiveSet,
    /// produces cleaner clipped output but slightly different envelope
    /// dynamics (BE damps cap-coupled feedback more aggressively).
    ActiveSetBe,
    /// Auto-inserted Boyle catch diodes. Soft exponential knee, correct physics.
    BoyleDiodes,
}

#[cfg(feature = "codegen")]
impl OpampRailMode {
    /// Parse a mode name (case-insensitive) from a CLI flag or config string.
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "auto" => Some(Self::Auto),
            "none" | "off" => Some(Self::None),
            "hard" | "clamp" => Some(Self::Hard),
            "active-set" | "active_set" | "activeset" => Some(Self::ActiveSet),
            "active-set-be" | "active_set_be" | "activesetbe" | "be-on-clamp" => {
                Some(Self::ActiveSetBe)
            }
            "boyle-diodes" | "boyle_diodes" | "boylediodes" | "boyle" | "diodes" => {
                Some(Self::BoyleDiodes)
            }
            _ => None,
        }
    }

    /// Human-readable name for logging.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::None => "none",
            Self::Hard => "hard",
            Self::ActiveSet => "active-set",
            Self::ActiveSetBe => "active-set-be",
            Self::BoyleDiodes => "boyle-diodes",
        }
    }
}

#[cfg(feature = "codegen")]
impl std::fmt::Display for OpampRailMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Configuration for code generation
#[cfg(feature = "codegen")]
#[derive(Debug, Clone)]
pub struct CodegenConfig {
    /// Circuit name for generated code
    pub circuit_name: String,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Maximum iterations for Newton-Raphson
    pub max_iterations: usize,
    /// Convergence tolerance
    pub tolerance: f64,
    /// Input resistance (Thevenin equivalent)
    pub input_resistance: f64,
    /// Input node index
    pub input_node: usize,
    /// Output node indices (one per output channel)
    pub output_nodes: Vec<usize>,
    /// Oversampling factor (1, 2, or 4). Default 1 (no oversampling).
    /// Factor > 1 reduces aliasing from nonlinearities.
    pub oversampling_factor: usize,
    /// Output scale factors applied after DC blocking (one per output, default [1.0])
    pub output_scales: Vec<f64>,
    /// Include DC operating point in generated code
    pub include_dc_op: bool,
    /// Maximum NR iterations for DC operating point solver
    pub dc_op_max_iterations: usize,
    /// Convergence tolerance for DC operating point solver
    pub dc_op_tolerance: f64,
    /// Include DC blocking filter on outputs (default true).
    /// Set to false for circuits with output coupling caps or when the downstream
    /// pipeline handles DC offset. Removes the 5Hz HPF and its settling time.
    pub dc_block: bool,
    /// Number of silent samples to process after pot-triggered matrix rebuild.
    /// Settles the NR to the new nonlinear DC operating point. Default 64.
    /// Set to 0 for zero-latency pot changes (may glitch on large pot swings).
    pub pot_settle_samples: usize,
    /// Use backward Euler integration instead of trapezoidal.
    /// Unconditionally stable (L-stable) — fixes divergence in high-gain feedback
    /// amplifiers where trapezoidal's imaginary-axis preservation causes oscillation.
    /// Trades second-order accuracy for first-order, giving slight HF rolloff.
    pub backward_euler: bool,
    /// Disable adaptive backward Euler fallback for the DK codegen path.
    /// When false (default), the generated code includes pre-computed BE matrices
    /// and can fall back to BE for individual samples where trapezoidal NR diverges.
    /// Set to true to save memory (~1.4KB for N=8, M=4) and compile time when
    /// the circuit is known to be well-conditioned.
    pub disable_be_fallback: bool,
    /// Strategy for op-amp supply rail saturation. Default [`OpampRailMode::Auto`],
    /// which inspects the circuit topology and picks the cheapest correct mode.
    /// See [`OpampRailMode`] for the full menu and trade-offs.
    pub opamp_rail_mode: OpampRailMode,
}

#[cfg(feature = "codegen")]
impl CodegenConfig {
    /// Validate configuration parameters.
    pub fn validate(&self) -> Result<(), CodegenError> {
        if !(self.sample_rate > 0.0 && self.sample_rate.is_finite()) {
            return Err(CodegenError::InvalidConfig(format!(
                "sample_rate must be positive and finite, got {}",
                self.sample_rate
            )));
        }
        if !(self.tolerance > 0.0 && self.tolerance.is_finite()) {
            return Err(CodegenError::InvalidConfig(format!(
                "tolerance must be positive and finite, got {}",
                self.tolerance
            )));
        }
        if self.max_iterations == 0 {
            return Err(CodegenError::InvalidConfig(
                "max_iterations must be > 0".to_string(),
            ));
        }
        if !(self.input_resistance > 0.0 && self.input_resistance.is_finite()) {
            return Err(CodegenError::InvalidConfig(format!(
                "input_resistance must be positive and finite, got {}",
                self.input_resistance
            )));
        }
        for (i, &scale) in self.output_scales.iter().enumerate() {
            if !scale.is_finite() {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_scales[{}] must be finite, got {}",
                    i, scale
                )));
            }
        }
        if !self.output_scales.is_empty()
            && !self.output_nodes.is_empty()
            && self.output_scales.len() != self.output_nodes.len()
        {
            return Err(CodegenError::InvalidConfig(format!(
                "output_scales length ({}) must match output_nodes length ({})",
                self.output_scales.len(),
                self.output_nodes.len()
            )));
        }
        Ok(())
    }
}

#[cfg(feature = "codegen")]
impl Default for CodegenConfig {
    fn default() -> Self {
        Self {
            circuit_name: "unnamed_circuit".to_string(),
            sample_rate: 44100.0,
            max_iterations: 100,
            tolerance: 1e-9,
            input_resistance: 1.0, // 1Ω default (near-ideal voltage source)
            input_node: 0,
            output_nodes: vec![0],
            oversampling_factor: 1,
            output_scales: vec![1.0],
            include_dc_op: true,
            dc_op_max_iterations: 200,
            dc_op_tolerance: 1e-9,
            dc_block: true,
            pot_settle_samples: 64,
            backward_euler: false,
            disable_be_fallback: false,
            opamp_rail_mode: OpampRailMode::Auto,
        }
    }
}

/// Error type for code generation failures
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum CodegenError {
    /// Invalid kernel configuration
    InvalidKernel(String),
    /// Unsupported circuit topology
    UnsupportedTopology(String),
    /// Invalid device model
    InvalidDevice(String),
    /// Invalid configuration parameter
    InvalidConfig(String),
    /// Template rendering error
    TemplateError(String),
    /// An upstream DK error
    Dk(crate::dk::DkError),
    /// An upstream MNA error
    Mna(crate::mna::MnaError),
}

impl std::fmt::Display for CodegenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (label, msg) = match self {
            CodegenError::InvalidKernel(s) => ("Invalid kernel", s.as_str()),
            CodegenError::UnsupportedTopology(s) => ("Unsupported topology", s.as_str()),
            CodegenError::InvalidDevice(s) => ("Invalid device", s.as_str()),
            CodegenError::InvalidConfig(s) => ("Invalid config", s.as_str()),
            CodegenError::TemplateError(s) => ("Template error", s.as_str()),
            CodegenError::Dk(e) => return write!(f, "Codegen error: {}", e),
            CodegenError::Mna(e) => return write!(f, "Codegen error: {}", e),
        };
        write!(f, "{label}: {msg}")
    }
}

impl std::error::Error for CodegenError {}

impl From<crate::dk::DkError> for CodegenError {
    fn from(e: crate::dk::DkError) -> Self {
        CodegenError::Dk(e)
    }
}

impl From<crate::mna::MnaError> for CodegenError {
    fn from(e: crate::mna::MnaError) -> Self {
        CodegenError::Mna(e)
    }
}

/// Generated circuit solver code.
#[cfg(feature = "codegen")]
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct GeneratedCode {
    /// The generated Rust source code
    pub code: String,
    /// Number of circuit nodes (excluding ground)
    pub n: usize,
    /// Total nonlinear dimension (sum of device dimensions)
    pub m: usize,
}

/// Code generator for circuit solvers
#[cfg(feature = "codegen")]
pub struct CodeGenerator {
    config: CodegenConfig,
}

#[cfg(feature = "codegen")]
impl CodeGenerator {
    /// Create a new code generator with the given configuration
    pub fn new(config: CodegenConfig) -> Self {
        Self { config }
    }

    /// Generate a complete circuit solver module.
    ///
    /// # Arguments
    /// * `kernel` - The compiled DK kernel
    /// * `mna` - MNA system for node mapping
    /// * `netlist` - Original netlist for extracting component values
    ///
    /// # Returns
    /// Generated Rust source code as a string
    pub fn generate(
        &self,
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
    ) -> Result<GeneratedCode, CodegenError> {
        // Validate config
        self.config.validate()?;
        match self.config.oversampling_factor {
            1 | 2 | 4 => {}
            f => {
                return Err(CodegenError::InvalidConfig(format!(
                    "oversampling_factor must be 1, 2, or 4, got {f}"
                )));
            }
        }
        if self.config.input_node >= kernel.n {
            return Err(CodegenError::InvalidConfig(format!(
                "input_node {} >= N={}",
                self.config.input_node, kernel.n
            )));
        }
        if self.config.output_nodes.is_empty() {
            return Err(CodegenError::InvalidConfig(
                "output_nodes must not be empty".to_string(),
            ));
        }
        for (i, &node) in self.config.output_nodes.iter().enumerate() {
            // Validate against n_nodes (original circuit nodes), not n_aug
            let n_nodes = kernel.n_nodes;
            if node >= n_nodes {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={} (original circuit node count)",
                    i, node, n_nodes
                )));
            }
        }
        if self.config.output_scales.len() != self.config.output_nodes.len() {
            return Err(CodegenError::InvalidConfig(format!(
                "output_scales length ({}) must match output_nodes length ({})",
                self.config.output_scales.len(),
                self.config.output_nodes.len()
            )));
        }

        self.generate_with_dc_op(kernel, mna, netlist, None)
    }

    /// Generate Rust solver code with a pre-computed DC operating point.
    ///
    /// When `dc_op` is provided, it is used instead of running the internal DC OP solver.
    /// This is useful when the MNA has been expanded (e.g., with internal nodes for
    /// parasitic BJTs) after the DC OP was computed on the original system.
    pub fn generate_with_dc_op(
        &self,
        kernel: &DkKernel,
        mna: &MnaSystem,
        netlist: &Netlist,
        dc_op: Option<crate::dc_op::DcOpResult>,
    ) -> Result<GeneratedCode, CodegenError> {
        // BoyleDiodes mode auto-inserts catch diodes into the netlist, which
        // grows the MNA/kernel dimensions. Rebuilding the DkKernel from the
        // augmented netlist would also change the DK vs nodal routing decision
        // (new N, new spectral radius, etc.), so for now BoyleDiodes is only
        // supported on the nodal path. Explicit request on the DK path is a
        // user error with a clear remedy.
        let resolved = ir::refine_active_set_for_audio_path(
            ir::resolve_opamp_rail_mode(mna, self.config.opamp_rail_mode),
            mna,
            netlist,
        );
        if resolved.mode == OpampRailMode::BoyleDiodes {
            return Err(CodegenError::UnsupportedTopology(format!(
                "OpampRailMode::BoyleDiodes is not yet supported on the DK codegen \
                 path. Either rerun with `--solver nodal` or use a different rail \
                 mode (`--opamp-rail-mode {{hard|active-set|active-set-be}}`). The \
                 auto-detector never picks BoyleDiodes today, so this error only \
                 triggers on an explicit user override."
            )));
        }

        // Auto-insert parasitic caps if C matrix is all zeros and circuit has
        // nonlinear devices. The IR stores G/C for runtime sample rate recomputation,
        // so these must include the parasitic caps (matching the kernel, which also
        // auto-inserts them in from_mna/from_mna_augmented).
        let patched_mna;
        let mna = if mna.m > 0 && !mna.c.iter().any(|row| row.iter().any(|&v| v != 0.0)) {
            log::info!(
                "Codegen: C matrix is all zeros with M={} nonlinear devices; \
                 auto-inserting parasitic caps for IR",
                mna.m
            );
            patched_mna = {
                let mut m = mna.clone();
                m.add_parasitic_caps();
                m
            };
            &patched_mna
        } else {
            mna
        };

        let ir = CircuitIR::from_kernel_with_dc_op(kernel, mna, netlist, &self.config, dc_op)?;
        let code = RustEmitter::new()?.emit(&ir)?;

        Ok(GeneratedCode {
            code,
            n: ir.topology.n,
            m: ir.topology.m,
        })
    }

    /// Generate code using the NodalSolver path (full N×N NR per sample).
    ///
    /// This bypasses the DkKernel entirely — no S=A⁻¹ precomputation, no K matrix.
    /// The generated code does LU factorization per NR iteration, which handles
    /// any circuit topology including transformer-coupled NFB with large inductors.
    ///
    /// # Errors
    /// Returns `CodegenError` if input/output node validation fails or if code
    /// generation encounters an error.
    pub fn generate_nodal(
        &self,
        mna: &MnaSystem,
        netlist: &Netlist,
    ) -> Result<GeneratedCode, CodegenError> {
        // Validate config (same checks as generate, but against MNA dimensions)
        self.config.validate()?;
        match self.config.oversampling_factor {
            1 | 2 | 4 => {}
            f => {
                return Err(CodegenError::InvalidConfig(format!(
                    "oversampling_factor must be 1, 2, or 4, got {f}"
                )));
            }
        }
        if self.config.input_node >= mna.n {
            return Err(CodegenError::InvalidConfig(format!(
                "input_node {} >= n_nodes={}",
                self.config.input_node, mna.n
            )));
        }
        if self.config.output_nodes.is_empty() {
            return Err(CodegenError::InvalidConfig(
                "output_nodes must not be empty".to_string(),
            ));
        }
        for (i, &node) in self.config.output_nodes.iter().enumerate() {
            if node >= mna.n {
                return Err(CodegenError::InvalidConfig(format!(
                    "output_nodes[{}] = {} >= n_nodes={}",
                    i, node, mna.n
                )));
            }
        }
        if self.config.output_scales.len() != self.config.output_nodes.len() {
            return Err(CodegenError::InvalidConfig(format!(
                "output_scales length ({}) must match output_nodes length ({})",
                self.config.output_scales.len(),
                self.config.output_nodes.len()
            )));
        }

        // BoyleDiodes mode: partial implementation, NOT yet wired into the
        // codegen pipeline.
        //
        // What's in place (this commit):
        //   * `augment_netlist_with_boyle_diodes` (in `codegen/ir.rs`)
        //     synthesizes the internal gain node, output-buffer VCVS,
        //     rail-reference VS, and catch diodes.
        //   * `mna::MnaSystem::from_netlist` op-amp stamping auto-detects
        //     `_oa_int_{name}` in `node_map` and switches that op-amp's
        //     transconductance stamp from the output node to the internal
        //     node, with `Gm = AOL/R_BOYLE_INT_LOAD` and
        //     `Go = 1/R_BOYLE_INT_LOAD`.
        //   * `OpampInfo::n_int_idx` records the internal node index per
        //     op-amp.
        //   * `mna::R_BOYLE_INT_LOAD` constant (= 1 MΩ) sets the
        //     high-impedance load.
        //   * `augment_netlist_with_boyle_diodes_on_klon_produces_valid_mna`
        //     unit test pins the augmented MNA contract.
        //
        // Verified:
        //   * DC OP on the augmented Klon matches the un-augmented Klon to
        //     11 decimal places (the buffer + internal node give the same
        //     bias values everywhere).
        //   * Manual dense LU on the augmented Klon's `A = G + 2C/T` linear
        //     system produces the correct first-sample transient response
        //     for a 1 mV 1 kHz input — `out` ramps smoothly to ~30 µV over
        //     the first 5 samples, matching the closed-loop gain through
        //     the gain stage.
        //   * Standalone augmented inverting amp test (`/tmp/opamp_sign_test`)
        //     gives `V_out = -10 V` for `V_in = 1 V` (gain -10) at both DC
        //     and across the first 5 transient samples, including via the
        //     internal node + buffer.
        //
        // What's broken:
        //   The codegen Newton-Raphson loop (full N×N LU path, since the
        //   catch diodes' positive K diagonal forces it) produces wildly
        //   wrong v[buf_in] = 135 V on the first sample with non-zero input
        //   for Klon, even though the input is only 0.13 mV and the matrix
        //   the codegen sees (`state.a`) matches the standalone matrix
        //   element-by-element. Switching the codegen from sparse_lu_factor
        //   to dense lu_factor produces the *same* wrong answer, so it's
        //   not a sparse-LU AMD-ordering issue. The discrepancy must be in
        //   how the chord_lu / companion-rhs interact with the augmented
        //   system's sparsity, but I haven't isolated it yet.
        //
        // Next steps (for a fresh-context follow-up):
        //   1. Compare full v_prev arrays (including all augmented branch
        //      currents at indices 44..N) between standalone and codegen at
        //      DC OP. If they match, the discrepancy is in the rhs build.
        //   2. Print every entry of state.a vs the standalone `a` matrix
        //      and find the first index where they differ.
        //   3. If state.a matches and rhs matches, the LU itself is wrong
        //      — likely a Gmin/static-row-swap interaction with the new
        //      sparsity pattern.
        //
        // Until that's resolved, BoyleDiodes returns a clear error pointing
        // users at the working `active-set-be` mode.
        let resolved = ir::refine_active_set_for_audio_path(
            ir::resolve_opamp_rail_mode(mna, self.config.opamp_rail_mode),
            mna,
            netlist,
        );
        if resolved.mode == OpampRailMode::BoyleDiodes {
            return Err(CodegenError::UnsupportedTopology(
                "OpampRailMode::BoyleDiodes is partially implemented but the \
                 codegen NR loop does not yet converge for circuits with the \
                 augmented internal gain node. The MNA pipeline (augment \
                 helper, op-amp internal-node stamping, OpampInfo.n_int_idx, \
                 unit tests) is in place and verified correct — DC OP and \
                 manual linear LU give expected values — but the generated \
                 process_sample's NR iteration produces wildly wrong values \
                 on the first non-zero input sample. Use \
                 `--opamp-rail-mode active-set-be` (the current best \
                 alternative for distortion-pedal topologies) until the \
                 codegen NR convergence issue is resolved. See \
                 `crates/melange-solver/src/codegen/mod.rs` for the \
                 detailed status note."
                    .to_string(),
            ));
        }

        // Auto-insert parasitic caps if C matrix is all zeros and circuit has
        // nonlinear devices. Without capacitors, A = G and the trapezoidal
        // integrator degenerates (no energy storage → no dynamics).
        let patched_mna;
        let mna = if mna.m > 0 && !mna.c.iter().any(|row| row.iter().any(|&v| v != 0.0)) {
            log::info!(
                "Codegen nodal: C matrix is all zeros with M={} nonlinear devices; \
                 auto-inserting parasitic caps",
                mna.m
            );
            patched_mna = {
                let mut m = mna.clone();
                m.add_parasitic_caps();
                m
            };
            &patched_mna
        } else {
            mna
        };

        let ir = CircuitIR::from_mna(mna, netlist, &self.config)?;
        let code = RustEmitter::new()?.emit(&ir)?;

        Ok(GeneratedCode {
            code,
            n: ir.topology.n,
            m: ir.topology.m,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_codegen_config_default() {
        let config = CodegenConfig::default();
        assert_eq!(config.sample_rate, 44100.0);
        assert_eq!(config.max_iterations, 100);
        assert!(config.tolerance > 0.0);
        // Default op-amp rail handling is auto-selection.
        assert_eq!(config.opamp_rail_mode, OpampRailMode::Auto);
    }

    #[test]
    fn test_opamp_rail_mode_parse_canonical() {
        assert_eq!(OpampRailMode::parse("auto"), Some(OpampRailMode::Auto));
        assert_eq!(OpampRailMode::parse("none"), Some(OpampRailMode::None));
        assert_eq!(OpampRailMode::parse("hard"), Some(OpampRailMode::Hard));
        assert_eq!(
            OpampRailMode::parse("active-set"),
            Some(OpampRailMode::ActiveSet)
        );
        assert_eq!(
            OpampRailMode::parse("boyle-diodes"),
            Some(OpampRailMode::BoyleDiodes)
        );
    }

    #[test]
    fn test_opamp_rail_mode_parse_aliases_and_case() {
        // Case-insensitive.
        assert_eq!(OpampRailMode::parse("AUTO"), Some(OpampRailMode::Auto));
        assert_eq!(OpampRailMode::parse("Hard"), Some(OpampRailMode::Hard));
        // Underscore variants.
        assert_eq!(
            OpampRailMode::parse("active_set"),
            Some(OpampRailMode::ActiveSet)
        );
        assert_eq!(
            OpampRailMode::parse("boyle_diodes"),
            Some(OpampRailMode::BoyleDiodes)
        );
        // Common short aliases.
        assert_eq!(OpampRailMode::parse("off"), Some(OpampRailMode::None));
        assert_eq!(OpampRailMode::parse("clamp"), Some(OpampRailMode::Hard));
        assert_eq!(OpampRailMode::parse("boyle"), Some(OpampRailMode::BoyleDiodes));
        assert_eq!(OpampRailMode::parse("diodes"), Some(OpampRailMode::BoyleDiodes));
    }

    #[test]
    fn test_opamp_rail_mode_parse_rejects_unknown() {
        assert_eq!(OpampRailMode::parse(""), None);
        assert_eq!(OpampRailMode::parse("soft"), None);
        assert_eq!(OpampRailMode::parse("tanh"), None);
        assert_eq!(OpampRailMode::parse("bogus"), None);
    }

    #[test]
    fn test_opamp_rail_mode_display_round_trips() {
        // Display format must parse back to the same variant so CLI logging stays stable.
        for mode in [
            OpampRailMode::Auto,
            OpampRailMode::None,
            OpampRailMode::Hard,
            OpampRailMode::ActiveSet,
            OpampRailMode::BoyleDiodes,
        ] {
            let text = mode.to_string();
            assert_eq!(
                OpampRailMode::parse(&text),
                Some(mode),
                "Display -> parse round-trip failed for {:?} (rendered as {:?})",
                mode,
                text
            );
        }
    }
}
