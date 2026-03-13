# Melange

*The spice must flow.*

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-2024%20edition-orange.svg)](https://www.rust-lang.org)

An open-source Rust toolkit for translating analog circuit schematics into real-time audio DSP code. Melange bridges the gap between SPICE simulation and production audio plugins — automating the pipeline that every circuit modeler currently does by hand.

## The Problem

There is no open-source tool that takes a circuit netlist and produces optimized real-time DSP code. Every audio developer who wants to model a real circuit (guitar amp, pedal, preamp, compressor) currently:

1. Reads a schematic by hand
2. Derives MNA matrices by hand
3. Implements a solver by hand
4. Validates against SPICE by hand
5. Optimizes for real-time by hand

**Melange automates steps 2-5.**

## One Command, Done

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

## Built-in Circuits

Melange ships with classic circuits ready to compile:

| Circuit | Description |
|---------|-------------|
| `tube-screamer` | Classic op-amp clipper (Tube Screamer style) |
| `fuzz-face` | 2-transistor fuzz (Fuzz Face style) |
| `big-muff` | 4-transistor fuzz (Big Muff style) |
| `rc-lowpass` | Simple RC lowpass filter for testing |
| `mordor-screamer` | High-gain distortion forged in Mount Doom |

Plus real-world validated circuits in `circuits/`:

| Circuit | Description |
|---------|-------------|
| `pultec-eq.cir` | Pultec EQP-1A passive tube EQ (3 switches, 5 pots, 12AX7) |
| `wurli-preamp.cir` | Wurlitzer 200A preamp (2-stage BJT, LDR tremolo pot) |
| `tweed-preamp.cir` | Fender-style 2-stage 12AX7 guitar amp (volume pot, bright switch) |

## What Gets Generated

The generated code is completely standalone — zero runtime dependencies on melange. It includes:

- Pre-inverted DK matrices with sparsity-aware emission (only non-zero entries)
- Newton-Raphson solver (M=1 direct, M=2 Cramer's, M=3-16 Gaussian elimination)
- DC operating point initialization for correct bias
- Sherman-Morrison rank-1 updates for dynamic potentiometers (O(N^2) not O(N^3))
- Matrix rebuild for runtime sample rate changes
- DC blocking filter (5 Hz HPF)
- Input/Output Level parameters (input defaults to -12 dB for safe levels)
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
- **7 Device Models**: Diode, BJT (Ebers-Moll/Gummel-Poon), JFET, MOSFET, vacuum tube (Koren), op-amp
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

## Supported Components

| Component | Runtime | Codegen | Notes |
|-----------|---------|---------|-------|
| Resistor/Capacitor/Inductor | yes | yes | Trapezoidal companion models |
| Diode | yes | yes | Shockley equation, series resistance, LED |
| BJT | yes | yes | Ebers-Moll, Gummel-Poon (VAF, VAR, IKF, IKR) |
| JFET | — | yes | Shichman-Hodges 2D (triode + saturation) |
| MOSFET | — | yes | Level 1 SPICE 2D (triode + saturation) |
| Vacuum Tube | — | yes | Koren triode + Leach grid current |
| Op-Amp | yes | yes | VCCS macromodel (linear) |
| CdS LDR | yes | — | VTL5C3/4, NSL-32 with asymmetric envelope |
| Voltage Source | yes | yes | DC (Norton equivalent) |
| Potentiometer | — | yes | Sherman-Morrison rank-1 updates |
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
melange validate <circuit> -r <ref>       Compare against SPICE reference
melange nodes <circuit>                   List nodes and devices in a circuit
melange builtins                          List built-in circuits
melange sources add <name> <url>          Add a git repo as a circuit source
melange sources list                      List configured sources
melange cache list|clear|stats            Manage cached circuits
```

Key `compile` options:
```
--format plugin          Generate a complete nih-plug project (default: code)
--no-level-params        Omit Input/Output Level parameters from plugin
--output-scale <f64>     Scale factor for circuit output (default: 1.0)
-s, --sample-rate <Hz>   Target sample rate (default: 48000)
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

- Rust 1.85+ (2024 edition)
- No external dependencies for core library or generated code
- Optional: ngspice for SPICE validation
- Optional: zig + cargo-zigbuild for macOS cross-compilation

## Building

```bash
cargo build --workspace       # Build everything
cargo test --workspace        # Run all tests (~730 tests)
cargo run -p melange-cli      # Run the CLI
```

## Origin

Melange was extracted from the [OpenWurli](https://github.com/openwurli/openwurli) project, a Wurlitzer 200A virtual instrument. The DK solver, device models, and validation pipeline were generalized into this standalone toolkit.

## License

This project is licensed under the [GNU General Public License v3.0 or later](LICENSE) (GPL-3.0-or-later).

Melange includes code derived from the [OpenWurli](https://github.com/openwurli/openwurli) project, which is licensed under GPL-3.0.

## Acknowledgments

- David Yeh's 2009 thesis on the Discrete K-method
- SPICE — the original circuit simulator by Larry Nagel
- The audio DSP community
