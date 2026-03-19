# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-03-19

### Added
- **Codegen**: Rust code generation from SPICE netlists (`melange compile`)
  - DK method with Sherman-Morrison dynamic potentiometers
  - Nodal solver fallback for multi-transformer circuits
  - Oversampling (2x/4x polyphase half-band IIR)
  - Sparsity-aware emission for zero-skipping in matrix operations
  - Per-device `.model` params with heterogeneous model support
- **Device models**: Junction capacitances (CJE/CJC, CGS/CGD, CJO, CCG/CGP/CCP),
  parasitic resistances (RS, RB/RC/RE, RD/RS, RGI), diode breakdown (BV/IBV),
  MOSFET body effect (GAMMA/PHI), BJT NF/ISE/NE emission params
- **Forward-active BJT optimization**: Auto-detect BJTs with Vbc < -1V at DC OP,
  model as 1D (Vbe→Ic only), reducing M by 1 per BJT. 46% speedup on wurli-preamp.
- **NR predictor warm start**: First-order extrapolation reduces NR iterations by 39%
- **Centralized fast_exp**: All exp() calls through inline function, optional polynomial path
- **Augmented MNA for inductors**: Branch current variables replace companion model,
  well-conditioned for large inductances. DK codegen for single-transformer circuits.
- **Subcircuit expansion** (`.subckt` / `X` elements)
- **AC frequency response** (`melange analyze` with `--pot`/`--switch` flags)
- **WAV simulation** (`melange simulate` with `--input` audio or `--amplitude` sine)
- **Plugin generation** (`melange compile --format plugin` for CLAP/VST3 via nih-plug)
- **Dynamic potentiometers**: `.pot` directive with Sherman-Morrison rank-1 updates
- **Multi-position switches**: `.switch` directive with matrix rebuild
- **Trapezoidal pot tracking**: Backward A_neg term uses previous resistance (SPICE-validated)
- **Optional DC blocking**: `--no-dc-block` flag for circuits with output coupling caps
- **Output clamp control**: Removed ±10V clamp when DC block is off
- **SPICE validation**: 12 tests against ngspice (all pass), including wurli-preamp (corr 0.999997)
- **GitHub Actions CI**: test, clippy, format, build, SPICE validation
- **Built-in circuits**: rc-lowpass, tube-screamer, big-muff, fuzz-face, tube-preamp, mordor-screamer
- **Reference circuits**: pultec-eq (4 tubes, 2 transformers), wurli-preamp, tweed-preamp

### Fixed
- Parser: Handle comma-separated SPICE `.model` parameters
- Pultec: True push-pull topology (HS-29 pin 8 → grid2, not cathode)
- Pultec: 20pF plate cap (not 20nF — 1000x error from µF convention)
- Pultec: Ra1/Ra2 from node_g (not v250)
- Pultec: S-217-D 220H primary inductance, 71-turn tertiary (0.447H)
- DC OP: Junction-aware initial guess for direct-coupled BJT stages

### Security
- Zero unsafe code across entire codebase
- Input validation for NaN, infinite, and out-of-bounds values
- Safe exp() clamping to [-40, 40] prevents overflow

## [0.1.0] - 2026-02-23

### Added
- Initial release with core functionality
- Workspace with 6 crates: primitives, devices, solver, validate, plugin, cli
- SPICE netlist parser, MNA assembler, DK kernel extraction
- Device models: Diode, BJT, JFET, MOSFET, Tube, Op-amp, LDR
- Filter primitives, oversampling, Newton-Raphson solvers
- Comprehensive test suite

[Unreleased]: https://github.com/melange-audio/melange/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/melange-audio/melange/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/melange-audio/melange/releases/tag/v0.1.0
