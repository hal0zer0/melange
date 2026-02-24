# Melange

*The spice must flow.*

[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE-MIT)
[![Rust](https://img.shields.io/badge/rust-2024%20edition-orange.svg)](https://www.rust-lang.org)

An open-source Rust toolkit for translating analog circuit schematics into real-time audio DSP code. Melange bridges the gap between SPICE simulation and production audio plugins — automating the pipeline that every circuit modeler currently does by hand.

## The Problem

There is no open-source tool that takes a circuit netlist and produces optimized real-time DSP code. The closest thing (Cytomic CCS) is proprietary and closed. Every audio developer who wants to model a real circuit (guitar amp, pedal, preamp, compressor) currently:

1. Reads a schematic by hand
2. Derives MNA matrices by hand
3. Implements a solver by hand
4. Validates against SPICE by hand
5. Optimizes for real-time by hand

**Melange automates steps 2-5.**

## Features

- **SPICE Netlist Parser**: Parse industry-standard SPICE netlists for audio circuits
- **MNA/DK Solver**: Modified Nodal Analysis with Discrete K-method reduction
- **Device Models**: Parameterized nonlinear models for BJT, tube, diode, MOSFET, JFET, LDR, op-amp
- **Real-Time Safe**: Zero heap allocation in the audio thread, no `unsafe` code
- **Code Generation**: Generate optimized Rust code from circuit descriptions
- **Validation Pipeline**: Automated comparison against ngspice reference simulations

## Quick Start

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

## Architecture

```
Layer 5: melange-plugin     — nih-plug integration (voice mgmt, oversampling, params)
Layer 4: melange-validate   — SPICE-to-Rust validation pipeline
Layer 3: melange-solver     — MNA/DK-method engine + code generation (THE CORE)
Layer 2: melange-devices    — Component models (BJT, tube, diode, LDR, opamp)
Layer 1: melange-primitives — DSP building blocks (filters, oversampling, NR helpers)
         melange-cli        — CLI tool for circuit compilation and benchmarking
```

Each crate is useful independently. You can use `melange-primitives` without any other crate.

## Supported Components

| Component | Status | Models |
|-----------|--------|--------|
| Resistor/Capacitor/Inductor | ✅ Complete | Linear elements with companion models |
| Diode | ✅ Complete | Shockley equation, series resistance, LED |
| BJT | ✅ Complete | Ebers-Moll, Gummel-Poon |
| JFET/MOSFET | ✅ Complete | Shichman-Hodges (Level 1) |
| Vacuum Tube | ✅ Complete | Koren triode/pentode |
| Op-Amp | ⚠️ Partial | Boyle macromodel defined, integration pending |
| CdS LDR | ✅ Complete | VTL5C3/4, NSL-32 with asymmetric envelope |

## Requirements

- Rust 1.85+ (2024 edition)
- No external dependencies for core library
- Optional: ngspice for validation

## Building

```bash
# Build everything
cargo build --workspace

# Run all tests
cargo test --workspace

# Run the CLI
cargo run -p melange-cli
```

## Safety & Performance

- **Zero `unsafe` code** in the entire codebase
- **Real-time safe**: No allocation in audio callback
- **Const generics**: Circuit topology at compile time
- **f64 everywhere**: Double precision for numerical accuracy

## Documentation

- [Architecture Overview](docs/architecture.md)
- [DK Method](docs/dk-method.md)
- [MNA Assembly](docs/mna-assembly.md)
- [Device Models](docs/device-models.md)
- [Known Limitations](docs/limitations.md)

## Testing

91 tests across all layers:

```
melange-primitives: 29 tests (filters, oversampling, NR, companion models)
melange-devices:    33 tests (BJT, diode, tube, JFET, MOSFET, LDR, opamp)
melange-solver:     25 tests (parser, MNA, DK kernel, solver)
doc-tests:           3 tests
```

## Origin

Melange was extracted from the [OpenWurli](https://github.com/openwurli/openwurli) project, a Wurlitzer 200A virtual instrument. The generic components (DK solver, device models, validation pipeline) are being generalized into this standalone toolkit.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contributing

See [AGENTS.md](AGENTS.md) for development guidelines and coding standards.

## Acknowledgments

- David Yeh's 2009 thesis on the Discrete K-method
- SPICE - The original circuit simulator by Larry Nagel
- The audio DSP community
