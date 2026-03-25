# Melange

*The spice must flow.*

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-2021%20edition-orange.svg)](https://www.rust-lang.org)

An open-source Rust toolkit for translating analog circuit schematics into real-time audio DSP code. Melange bridges the gap between SPICE simulation and production audio plugins — automating the pipeline that every circuit modeler currently does by hand.

> **IMPORTANT: Always use caution when listening to Melange output.** Generated plugins include a default-on ear-protection soft limiter that prevents output from exceeding 0 dBFS, but circuits can still produce unexpected gain levels or unstable behavior. Always start with your monitor level at zero and increase gradually. The limiter can be toggled off at runtime or disabled entirely with `--no-ear-protection`.

## The Problem

There is no open-source tool that takes a circuit netlist and produces optimized real-time DSP code. Every audio developer who wants to model a real circuit (guitar amp, pedal, preamp, compressor) currently:

1. Reads a schematic by hand
2. Derives MNA matrices by hand
3. Implements a solver by hand
4. Validates against SPICE by hand
5. Optimizes for real-time by hand

**Melange automates steps 2-5.**

## One Command, Done

**NOTE** This project is in early alpha, and the current development focus has been on supporting the hardware needed for the OpenWurli project. Additional circuit tests are in the works. 

### SPICE netlist to audio plugin

```bash
melange compile fuzz-face --format plugin -o my-fuzz
cd my-fuzz && cargo build --release
# You now have a VST3/CLAP plugin.
```

That's it. One command turns a circuit into a buildable nih-plug project with all the DSP, parameters, and real-time-safe code generated for you.

### Simulate a circuit to WAV

```bash
# Push a guitar recording through a Tube Screamer
melange simulate tube-screamer --input-audio guitar.wav -o output.wav

# Quick test tone through a Big Muff
melange simulate big-muff --amplitude 0.1 -o fuzz.wav
```

No code, no compilation — just circuit + audio in, audio out.

### Fetch circuits from anywhere

```bash
# Built-in circuits — just use the name
melange compile tube-screamer --format plugin -o ts9

# Local SPICE netlist
melange compile my-circuit.cir --format plugin -o my-plugin

# Add a git repo as a circuit source
melange sources add pedalboards https://github.com/someone/spice-pedals
melange compile pedalboards:rat-distortion --format plugin -o rat
```

Sources are cached locally. Share circuit libraries via git, pull them by name.

### Inspect any circuit

```bash
melange nodes big-muff
# Nodes in circuit:
#   (0) GND - Ground reference
#   (1) in
#   (2) base1
#   ...
# Nonlinear devices:
#   Q1: Bjt (dimension: 2)
#   Q2: Bjt (dimension: 2)
#   D1: Diode (dimension: 1)
#   D2: Diode (dimension: 1)
```

### Cross-compile for macOS from Linux

```bash
cargo zigbuild --release --target universal2-apple-darwin
rcodesign sign target/universal2-apple-darwin/release/libmy_plugin.dylib
# Universal Mac binary, ad-hoc signed. Ship it.
```

## Documentation

- **[Getting Started](docs/GETTING_STARTED.md)** — from zero to working plugin in 5 minutes
- **[Netlist Writing Guide](docs/NETLIST_GUIDE.md)** — how to write SPICE netlists from schematics
- **[Plugin Development Guide](docs/PLUGIN_GUIDE.md)** — customizing generated plugins
- **[SPICE Grammar Reference](docs/spice-grammar.md)** — complete syntax for all elements and directives

## Built-in Circuits

Melange ships with classic circuits ready to compile:

| Circuit | Description |
|---------|-------------|
| `tube-screamer` | Op-amp overdrive with diode feedback clipping (TS808 style) |
| `fuzz-face` | 2-transistor fuzz (Fuzz Face style) |
| `big-muff` | 4-transistor fuzz with dual clipping (Big Muff style) |
| `tube-preamp` | Common-cathode 12AX7 triode gain stage with tone control |
| `rc-lowpass` | Simple RC lowpass filter for testing |
| `mordor-screamer` | High-gain distortion forged in Mount Doom |
| `ssl-bus-compressor` | SSL 4000E bus compressor (VCA + sidechain) |

Plus example circuits in `circuits/`:

| Circuit | Description |
|---------|-------------|
| `wurli-preamp.cir` | Wurlitzer 200A preamp (2-stage BJT, LDR tremolo pot) |
| `tweed-preamp.cir` | Fender-style 2-stage 12AX7 guitar amp (volume pot, bright switch) |
| `pultec-eq.cir` | Pultec EQP-1A passive EQ (4 tubes, 2 transformers, 7 pots, 3 switches) |

## What Gets Generated

The generated code is completely standalone — zero runtime dependencies on melange. It includes:

- Pre-inverted DK matrices with sparsity-aware emission (only non-zero entries)
- Newton-Raphson solver (M=1 direct, M=2 Cramer's, M=3-16 Gaussian elimination)
- DC operating point initialization for correct bias
- Sherman-Morrison rank-1 updates for dynamic potentiometers (O(N^2) not O(N^3))
- Matrix rebuild for runtime sample rate changes
- DC blocking filter (5 Hz HPF)
- Input/Output Level parameters (both default to 0 dB)
- Optional 2x/4x oversampling (self-contained polyphase half-band IIR)
- All buffers pre-allocated — zero heap allocation in the audio path

## Customizing Generated Plugins

Generated plugins use a two-file architecture:

- **`src/circuit.rs`** — Generated DSP code (matrices, NR solver, state). Don't edit by hand.
- **`src/lib.rs`** — Plugin wrapper (params, GUI, presets). Customize freely.

To iterate on component values without losing your `lib.rs` customizations, regenerate just the circuit:

```bash
melange compile my-circuit.cir --format code -o my-plugin/src/circuit.rs
```

### Parameter Labels

Add human-readable labels to `.pot` and `.switch` directives in your netlist:

```spice
.pot R_vol 500 500k "Volume"
.switch C_bright 1p 120p 470p "Bright"
```

Without labels, params use the component name (e.g. "R_vol", "Switch 0 (C_bright)").

### What's Safe to Customize in lib.rs

- Parameter names, ranges, units, and smoothing
- Plugin metadata (name, vendor, description)
- GUI (nih-plug supports [VIZIA](https://github.com/vizia/vizia) and [iced](https://github.com/iced-rs/iced))
- Pre/post processing (wet/dry mix, oversampling control)
- Presets and state persistence

## Features

- **SPICE Netlist Parser**: Industry-standard SPICE netlists with `.model` and `.subckt` support
- **MNA/DK Solver**: Modified Nodal Analysis with Discrete K-method (Yeh 2009)
- **Device Models**: Diode, BJT (Ebers-Moll/Gummel-Poon), JFET, MOSFET, vacuum tube (Koren), op-amp, CdS LDR
- **Dynamic Controls**: `.pot` (potentiometers) and `.switch` (ganged component switching) directives
- **Real-Time Safe**: Zero heap allocation in audio callback, no `unsafe` code, f64 precision
- **Code Generation**: Optimized Rust with compile-time constants, sparse matrices, unrolled loops
- **Plugin Generation**: One-step CLAP/VST3 plugin projects via nih-plug
- **SPICE Validation**: Automated comparison against ngspice reference simulations
- **Cross-Compilation**: Build macOS plugins from Linux via cargo-zigbuild

## Architecture

```
Layer 5: melange-plugin     — nih-plug integration (voice mgmt, oversampling, params)
Layer 4: melange-validate   — SPICE-to-Rust validation pipeline
Layer 3: melange-solver     — MNA/DK-method engine + code generation (THE CORE)
Layer 2: melange-devices    — Component models (BJT, tube, diode, LDR, opamp)
Layer 1: melange-primitives — DSP building blocks (filters, oversampling, NR helpers)
         melange-cli        — CLI tool
```

Each crate is useful independently. The pipeline:

```
SPICE Netlist → Parser → MNA System → DK Kernel → CircuitIR → Rust Emitter → Source Code
```

## Solver Routing

Melange auto-selects the best solver backend for each circuit:

| Solver | When Selected | Performance | Pot Updates |
|--------|--------------|-------------|-------------|
| **DK** | M < 10, ≤ 1 transformer group | 100-600x realtime | Per-sample (Sherman-Morrison O(N^2)) |
| **Nodal Schur** | M ≥ 10 or 2+ transformer groups | 10-50x realtime | Per-block (matrix rebuild) |
| **Nodal Full LU** | K matrix ill-conditioned, VCA circuits, or complex feedback | 5-15x realtime | Per-block (matrix rebuild) |

Example circuits (48kHz, release build):

| Circuit | N | M | Solver | Performance |
|---------|---|---|--------|-------------|
| RC lowpass | 2 | 0 | DK | >1000x |
| Fuzz Face | 5 | 4 | DK | ~200x |
| Tweed preamp (2x 12AX7) | 13 | 4 | DK | ~150x |
| Wurli power amp (8 BJTs) | 20 | 16 | DK | ~20x |
| Pultec EQP-1A (4 tubes, 2 transformers) | 41 | 8 | Nodal Full LU | ~11x |

Override with `--solver dk` or `--solver nodal` if auto-selection doesn't suit your needs.

## Supported Components

| Component | Runtime | Codegen | Notes |
|-----------|---------|---------|-------|
| Resistor/Capacitor/Inductor | yes | yes | Trapezoidal companion models |
| Diode | yes | yes | Shockley equation, series resistance, LED |
| BJT | yes | yes | Ebers-Moll, Gummel-Poon, junction capacitances, parasitic R |
| JFET | yes | yes | Shichman-Hodges 2D (triode + saturation) |
| MOSFET | yes | yes | Level 1 SPICE 2D (triode + saturation) |
| Vacuum Tube | yes | yes | Koren triode + Leach grid current + lambda |
| Op-Amp | yes | yes | Boyle VCCS macromodel (AOL, ROUT, optional GBW/VSAT) |
| VCA | yes | yes | THAT 2180 exponential gain (current-mode, THD) |
| CdS LDR | yes | — | VTL5C3/4, NSL-32 with asymmetric envelope |
| Voltage Source | yes | yes | DC (Norton equivalent) |
| Potentiometer | — | yes | Sherman-Morrison rank-1 updates (codegen only) |
| Switch | — | yes | Ganged R/C/L component switching |

## Quick Start (Library)

```rust
use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;

// Parse a SPICE netlist
let spice = r#"RC Circuit
R1 in out 1k
C1 out 0 1u
"#;
let netlist = Netlist::parse(spice)?;

// Build MNA system
let mna = MnaSystem::from_netlist(&netlist)?;

// Create DK kernel at sample rate
let kernel = DkKernel::from_mna(&mna, 44100.0)?;
```

## CLI Reference

```
melange compile <circuit> -o <output>     Compile circuit to Rust code or plugin project
melange simulate <circuit> -o <output>    Simulate circuit, write WAV output
melange analyze <circuit>                 Analyze circuit frequency response
melange validate <circuit>                 Compare against ngspice reference
melange nodes <circuit>                   List nodes and devices in a circuit
melange builtins                          List built-in circuits
melange sources add <name> <url>          Add a git repo as a circuit source
melange sources list                      List configured sources
melange sources show <name>               Show details of a circuit source
melange sources remove <name>             Remove a circuit source
melange cache list|clear|stats            Manage cached circuits
```

Key `compile` options:
```
--format plugin          Generate a complete nih-plug project (default: code)
-s, --sample-rate <Hz>   Target sample rate (default: 48000)
--solver <auto|dk|nodal> Solver backend (default: auto)
--oversampling <1|2|4>   Oversampling factor (default: 1)
--output-scale <f64>     Scale factor for circuit output (default: 1.0)
--no-level-params        Omit Input/Output Level parameters from plugin
--no-dc-block            Disable DC blocking filter
--mono                   Generate mono plugin (default: stereo)
-I, --input-node <name>  Input node name (default: "in")
-n, --output-node <name> Output node name (default: "out")
```

Key `analyze` options:
```
--start-freq <Hz>        Start frequency (default: 20)
--end-freq <Hz>          End frequency (default: 20000)
--points-per-decade <N>  Frequency resolution (default: 10)
--amplitude <V>          Input amplitude in volts (default: 0.1)
-I, --input-node <name>  Input node name (default: "in")
-n, --output-node <name> Output node name (default: "out")
-o, --output <file>      Write CSV to file instead of stdout
```

Circuits can be referenced as:
- **Built-in name**: `tube-screamer`, `fuzz-face`, `big-muff`, ...
- **Local file**: `path/to/circuit.cir`
- **Source reference**: `sourcename:circuitname`

## Requirements

- Rust 1.85+ (2021 edition)
- No external dependencies for core library or generated code
- Optional: ngspice for SPICE validation
- Optional: zig + cargo-zigbuild for macOS cross-compilation

## Building

```bash
# Install the melange CLI
cargo install --path tools/melange-cli

# Now use it directly
melange builtins
melange compile tube-screamer --format plugin -o my-plugin
```

Or build and run from the repo without installing:

```bash
cargo build --workspace       # Build everything
cargo test --workspace        # Run all tests (~990 tests)
cargo run -p melange-cli -- compile tube-screamer --format plugin -o my-plugin
```

## Origin

Melange was extracted from the [OpenWurli](https://github.com/openwurli/openwurli) project, a Wurlitzer 200A virtual instrument. The DK solver, device models, and validation pipeline were generalized into this standalone toolkit.

## Roadmap

Melange currently generates self-contained Rust code. Multi-language codegen is coming soon — the internal intermediate representation (`CircuitIR`) and `Emitter` trait are already language-agnostic by design. C++ will be the first additional target, with other languages to follow based on community interest. The goal is to let you compile a SPICE netlist directly to native code in whatever language your plugin framework or audio engine uses.

## Resource Limits

| Limit | Value | What Happens |
|-------|-------|-------------|
| Nonlinear dimensions (M) | 16 max on DK path | Auto-routes to nodal solver |
| Circuit nodes (N) | 256 max | Rejected with error |
| Elements after expansion | 10,000 max | Rejected with error |
| Subcircuit nesting | 8 levels max | Rejected with error |
| Pots per circuit | 32 max | Rejected with error |
| Switches per circuit | 16 max | Rejected with error |

M counts nonlinear *dimensions*, not devices: each diode adds 1, each BJT/JFET/MOSFET/tube/VCA adds 2.

## Known Limitations

- **Fixed temperature (27°C)**: All device models simulate at 300.15K. No temperature coefficients.
- **Triode tubes only**: Pentode tubes (EL84, 6L6, EL34) are not yet supported. 12AX7, 12AU7, and other preamp triodes work fully.
- **Ideal transformers**: Coupled inductors assume constant coupling coefficient with no core saturation or hysteresis.
- **Linear op-amps**: VCCS macromodel with optional GBW pole and VSAT, but no slew-rate limiting.
- **No noise simulation**: Shot noise, thermal noise, and 1/f noise are not modeled.

## Audio Safety Features

Three features protect your ears and speakers (all default-on, all optional):

- **Ear protection** — soft limiter that engages near 0 dBFS. Toggle at runtime or disable with `--no-ear-protection`.
- **DC blocking** — 5 Hz high-pass filter removes DC offset from circuit output. Settles in ~200ms. Disable with `--no-dc-block` if the circuit has its own output coupling capacitor.
- **Oversampling** (opt-in) — 2x or 4x polyphase half-band IIR reduces aliasing from nonlinear devices. Enable with `--oversampling 2` or `--oversampling 4`. CPU cost scales linearly.

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Plugin produces silence | Wrong input/output node names | Check `melange nodes circuit.cir`; use `--input-node` / `--output-node` |
| Very quiet output | Circuit output is millivolts, not volts | Increase `--output-scale` or Input Level parameter |
| NaN or oscillation | DC operating point failed to converge | Check biasing; try `--backward-euler` |
| "M exceeds MAX_M" error | Too many nonlinear dimensions for DK solver | Use `--solver nodal` |
| Clicks on parameter changes | Pot smoothing too short | Increase smoother duration in `lib.rs` |
| ~3% error vs ngspice | Using wrong RHS formulation | Ensure trapezoidal (default); check `.OPTIONS INTERP` in SPICE netlist |
| Output all zeros | VIN source in no-vin netlist | Check that the circuit file doesn't include VIN (melange provides input via conductance) |

See [docs/PLUGIN_GUIDE.md](docs/PLUGIN_GUIDE.md) for plugin-specific troubleshooting.

## License

This project is licensed under the [GNU General Public License v3.0 or later](LICENSE) (GPL-3.0-or-later).

Melange includes code derived from the [OpenWurli](https://github.com/openwurli/openwurli) project, which is licensed under GPL-3.0.

> **Melange needs additional maintainers!** The current maintainer is a SWE but does not have the necessary electrical engineering or Rust expertise to maintain a project of this complexity long-term. If you've got the skills to help, please get in touch.

## Acknowledgments

Melange builds on decades of research in circuit simulation and audio DSP:

**Core methods:**
- David Yeh — Discrete K-method for audio circuit simulation (PhD thesis, Stanford, 2009)
- Larry Nagel — SPICE (UC Berkeley); SPICE3f5 source for pnjlim/fetlim voltage limiting
- Ho, Ruehli, Brennan — Modified Nodal Analysis (IEEE Trans. CAS, 1975)
- Sherman & Morrison — rank-1 matrix update formula (used for dynamic potentiometers)

**Device models:**
- Hermann Gummel & H.C. Poon — BJT model with Early effect and high-injection (Bell Labs, 1970)
- Jiri Shichman & David Hodges — JFET model (IEEE JSSC, 1968)
- Norman Koren — triode vacuum tube model (1996)
- Marshall Leach — grid current model for vacuum tubes
- William Shockley — semiconductor diode equation

**DSP & filters:**
- Olli Niemitalo — polyphase allpass half-band IIR filters for oversampling
- Vadim Zavalishin — topology-preserving transform (TPT) filters

**References:**
- Pillage, Rohrer, Visweswariah — *Electronic Circuit and System Simulation Methods* (McGraw-Hill, 1995)
- [Hack Audio circuit modeling tutorial](https://hackaudio.com/tutorial-courses/audio-circuit-modeling-tutorial/)
- [TU Delft analog electronics webbook](https://analog-electronics.ewi.tudelft.nl/webbook/SED/)
