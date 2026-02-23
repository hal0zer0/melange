# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Melange** is an open-source Rust toolkit for translating analog circuit schematics into real-time audio DSP code. It bridges the gap between SPICE simulation and production audio plugins — the pipeline that every circuit modeler currently does by hand.

The name comes from Dune's melange spice (which is rust-colored, naturally). *The spice must flow.*

### The Problem

There is no open-source tool, in any language, that takes a circuit netlist and produces optimized real-time DSP code. The closest thing (Cytomic CCS) is proprietary and closed. Every audio developer who wants to model a real circuit (guitar amp, pedal, preamp, compressor) currently:
1. Reads a schematic by hand
2. Derives MNA matrices by hand
3. Implements a solver by hand
4. Validates against SPICE by hand
5. Optimizes for real-time by hand

Melange automates steps 2-5.

### The Stack

```
Layer 5: melange-plugin     — nih-plug integration (voice mgmt, oversampling, params)
Layer 4: melange-validate   — SPICE-to-Rust validation pipeline
Layer 3: melange-solver     — MNA/DK-method engine + code generation (THE CORE)
Layer 2: melange-devices    — Component models (BJT, tube, diode, LDR, opamp)
Layer 1: melange-primitives — DSP building blocks (filters, oversampling, NR helpers)
         melange-cli        — CLI tool for circuit compilation and benchmarking
```

### Origin

Melange was extracted from the [OpenWurli](https://github.com/FIXME/openwurli) project, a Wurlitzer 200A virtual instrument that required building all of this infrastructure from scratch. The generic components (DK solver, device models, validation pipeline) are being generalized into this standalone toolkit so that other circuit modeling projects don't have to re-solve the same hard problems.

## Development Philosophy

1. **No approximations without justification** — Every simplification must be documented with its error bound. "Close enough" is not a design principle.
2. **SPICE is ground truth** — Every model must validate against ngspice simulation before it ships.
3. **Real-time or nothing** — Every algorithm must run at audio rate (< 1ms latency at 44.1kHz). No heap allocation in the audio thread. No dynamic dispatch in hot loops.
4. **Const-generic performance** — Generic over circuit topology at compile time, not runtime. A circuit compiled through melange should be as fast as hand-written code.
5. **Layered independence** — Each crate is useful on its own. You can use melange-devices without melange-solver. You can use melange-primitives without anything else.

## Toolchain

| Tool | Purpose |
|------|---------|
| Rust / Cargo | Everything (Rust 2024 edition) |
| ngspice | Reference simulation for validation |
| Python | Analysis tooling only (optional) |

## Build Commands

```bash
# Build everything
cargo build --workspace

# Run all tests
cargo test --workspace

# Run the CLI
cargo run -p melange-cli
```

## Project Structure

```
Cargo.toml                          # Workspace root
crates/
  melange-primitives/               # Layer 1: DSP building blocks
    src/
      lib.rs
  melange-devices/                  # Layer 2: Component models
    src/
      lib.rs
  melange-solver/                   # Layer 3: MNA/DK engine + codegen
    src/
      lib.rs
  melange-validate/                 # Layer 4: SPICE validation pipeline
    src/
      lib.rs
  melange-plugin/                   # Layer 5: nih-plug integration
    src/
      lib.rs
tools/
  melange-cli/                      # CLI tool
    src/
      main.rs
docs/                               # Architecture, derivations, research
```

## Rules

- **Real-time safety is non-negotiable.** No allocation, no locks, no syscalls in any code path that runs per-sample. `no_std` where possible.
- **Every device model needs a SPICE reference.** If you can't validate it against ngspice, it doesn't ship.
- **Const generics over dynamic dispatch.** Matrix sizes, node counts, and nonlinearity counts should be compile-time constants. Use `[f64; N]` not `Vec<f64>` in the solver hot path.
- **Test at every layer.** Unit tests for primitives, comparison tests against SPICE for devices, integration tests for full circuit solves.
- **Document the math.** Every solver algorithm needs a companion doc in `docs/` showing the derivation. Someone should be able to read the doc and understand why the code looks the way it does.

## Key Technical Concepts

### MNA (Modified Nodal Analysis)
The standard method for assembling circuit equations. Each component "stamps" its contribution into a conductance matrix G and capacitance matrix C. The system equation is: `C * dv/dt + G * v = sources`.

### DK Method (Discrete K-Method)
David Yeh's method (Stanford, 2009) for handling nonlinear components. The N-node linear system is solved analytically, reducing the problem to an M-dimensional nonlinear kernel (where M = number of nonlinear devices). This M is typically 1-4 even for complex circuits, making Newton-Raphson iteration cheap.

### Companion Models
Reactive components (capacitors, inductors) are discretized into equivalent resistor + current source pairs using trapezoidal or bilinear transforms. This converts the ODE into an algebraic system solvable at each time step.

### Sherman-Morrison
For circuits with one time-varying element (e.g., an LDR, a potentiometer), the full matrix inverse doesn't need recomputation. A rank-1 correction updates the solution at ~30 FLOP cost per sample.

## Relationship to OpenWurli

OpenWurli is melange's first "customer." As melange matures, OpenWurli's hand-rolled DK solver, device models, and validation tools will be replaced by melange crate dependencies. This provides a real-world integration test for every melange release.

Components being extracted from OpenWurli:
- `filters.rs` → `melange-primitives` (nearly verbatim)
- `oversampler.rs` → `melange-primitives` (add configurable ratio)
- `bjt_stage.rs` → `melange-devices` (parameterize, remove Wurlitzer constants)
- `tremolo.rs` (CdS LDR model) → `melange-devices` (parameterize)
- `power_amp.rs` → `melange-devices` (parameterize)
- `pickup.rs` → `melange-devices` (parameterize RC constants)
- `dk_preamp.rs` → `melange-solver` (generalize topology, hardest extraction)
- `preamp-bench` measurement routines → `melange-validate`
- `spice/` validation methodology → `melange-validate`
