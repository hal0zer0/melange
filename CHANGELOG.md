# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Initial workspace structure with 5-layer architecture
- SPICE netlist parser supporting R, C, L, V, I, D, Q, J, M, X elements
- MNA (Modified Nodal Analysis) matrix assembler
- DK (Discrete K)-method kernel extraction
- Circuit solver with real-time safe processing
- Device models: Diode (Shockley, with Rs, LED), BJT (Ebers-Moll, Gummel-Poon)
- Device models: JFET, MOSFET (square law), Vacuum tube (Koren triode/pentode)
- Device models: CdS LDR with asymmetric attack/release, Op-amp (Boyle, Simple)
- Filter primitives: One-pole LPF/HPF, TPT LPF, Biquad, DC blocker
- Oversampling: Polyphase IIR 2x/4x with configurable quality
- Newton-Raphson solvers: 1D, 2D, and M-dimensional with warm start
- Companion models: Trapezoidal, Bilinear, Backward Euler for reactive components
- Comprehensive test suite: 91 tests passing
- Documentation: Architecture, DK method, MNA assembly, device models

### Changed
- N/A (initial release)

### Deprecated
- N/A (initial release)

### Removed
- N/A (initial release)

### Fixed
- Parser: Fixed femto vs Farad ambiguity in value parsing
- Parser: Fixed model parameter parentheses handling
- Solver: Fixed DK kernel sign convention
- Solver: Fixed BJT emitter current sign in N_i stamping
- Solver: Fixed inductor companion model history formula
- Solver: Fixed inductor history formula (i_hist = i_new - g_eq*v_new)

### Security
- Zero unsafe code verified across entire codebase
- Input validation added for NaN, infinite, and out-of-bounds values
- Division by zero protection for sample_rate and component values
- Index bounds checking with assertions moved to construction time

## [0.1.0] - 2026-02-23

### Added
- Initial release with core functionality
- Workspace with 6 crates: primitives, devices, solver, validate, plugin, cli

[Unreleased]: https://github.com/melange-audio/melange/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/melange-audio/melange/releases/tag/v0.1.0
