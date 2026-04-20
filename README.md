# Melange

*The SPICE must flow.*

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-2021%20edition-orange.svg)](https://www.rust-lang.org)

Melange is a circuit compiler. Give it a SPICE netlist — or a KiCad schematic — and it emits optimized, real-time-safe DSP code. It handles the math so you can focus on your product.

Melange is **not** a plugin framework. It doesn't generate GUIs, manage presets, or decide what your plugin looks like. It produces a DSP engine with exposed parameters. You bring everything else — that's the point.

> **Status: early alpha.** Several classic circuits are validated and working (Pultec EQP-1A, Wurlitzer 200A, tweed preamp, Vox AC15, Tube Screamer). Others are under active development. See the [circuits table](#circuits) for per-circuit status.

> **Ear safety.** Generated plugins include a default-on soft limiter. Start with your monitor level at zero and increase gradually. Disable with `--no-ear-protection` for measurement.

## The Problem

There is no open-source tool that takes a circuit schematic and produces optimized real-time DSP code. Every audio developer who wants to model a real circuit currently derives MNA matrices by hand, implements a solver by hand, validates against SPICE by hand, and optimizes for real-time by hand.

**Melange automates all of that.**

## Three Ways In

### 1. From a KiCad schematic

Draw your circuit in KiCad, export, compile. Melange ships a [KiCad symbol library and netlist exporter](kicad/README.md) for triodes, pentodes, op-amps, VCAs, pots, wipers, and audio I/O markers.

```
┌──────────┐     Export      ┌──────────┐    melange     ┌──────────┐
│  KiCad   │ ──────────────► │  .cir    │ ─────────────► │  Plugin  │
│ Schematic│   (one click)   │ netlist  │    compile     │ Project  │
└──────────┘                 └──────────┘                └──────────┘
```

```bash
# Export from KiCad (or use File → Export Netlist → Melange in the GUI)
kicad-cli sch export python-bom -o circuit.xml my-circuit.kicad_sch
melange import circuit.xml -o circuit.cir

# Compile to plugin
melange compile circuit.cir --format plugin -o my-plugin
cd my-plugin && cargo build --release
# → VST3/CLAP plugin ready to load in your DAW
```

Standard parts (R, C, L, D, BJT, JFET, MOSFET) use KiCad's built-in `Simulation_SPICE` symbols. Melange-specific parts (triodes, pentodes, VCAs, pots, wipers, I/O markers) use the included `melange.kicad_sym` library. See the [KiCad integration guide](kicad/README.md) for setup.

### 2. From a SPICE netlist

Write a netlist by hand or grab one from a circuit repository:

```bash
melange compile my-circuit.cir --format plugin -o my-plugin
cd my-plugin && cargo build --release
```

### 3. From a circuit library

Circuits live in separate repos. The official library is [melange-audio/circuits](https://github.com/melange-audio/circuits):

```bash
melange compile melange:stable/filters/passive-eq1a --format plugin -o my-eq
cd my-eq && cargo build --release
```

You can add any git repo as a named circuit source:

```bash
melange sources add pedalboards https://github.com/someone/spice-pedals
melange compile pedalboards:rat-distortion --format plugin -o rat
```

## Simulate Without Compiling

Don't want a plugin yet? Push audio through a circuit directly:

```bash
# Process a WAV file through the circuit
melange simulate my-circuit.cir --input-audio guitar.wav -o output.wav

# Quick test tone
melange simulate my-circuit.cir --amplitude 0.1 -o drive.wav

# Frequency response sweep
melange analyze my-circuit.cir --pot "Drive=100k" --switch "Mode=2"
```

## What You Get

The generated plugin project has two files:

| File | You edit it? | Contains |
|------|-------------|----------|
| `src/circuit.rs` | **No** — regenerate it | All DSP: matrices, NR solver, device equations, DC operating point, sample-rate recomputation |
| `src/lib.rs` | **Yes** — it's yours | Plugin wrapper: parameters, GUI, presets, pre/post processing |

To iterate on component values without losing your `lib.rs` work:

```bash
melange compile my-circuit.cir --format code -o my-plugin/src/circuit.rs
```

The generated DSP code is completely standalone — zero runtime dependencies on melange. It includes pre-inverted matrices with sparsity-aware emission, Newton-Raphson iteration, DC bias initialization, per-sample pot smoothing, optional oversampling, DC blocking, and ear protection. All buffers are pre-allocated. Zero heap allocation in the audio path. No `unsafe`.

### What's yours to customize

- GUI (nih-plug supports [VIZIA](https://github.com/vizia/vizia) and [iced](https://github.com/iced-rs/iced))
- Parameter names, ranges, units, smoothing curves
- Plugin metadata (name, vendor, VST3/CLAP IDs)
- Presets and state persistence
- Pre/post processing (wet/dry mix, oversampling control, sidechain)

See the [Plugin Development Guide](docs/PLUGIN_GUIDE.md) for details.

## Supported Devices

| Component | Model | Notes |
|-----------|-------|-------|
| R / C / L | Trapezoidal companion; inductors via augmented MNA | Coupled inductors and transformers supported |
| Diode | Shockley + series resistance + Zener breakdown | LED variant included |
| BJT | Ebers-Moll or Gummel-Poon | Junction caps, parasitic R, Early effect, high-injection knee. Matches ngspice `bjtload.c` line-for-line |
| JFET | Shichman-Hodges (triode + saturation) | N-channel and P-channel |
| MOSFET | SPICE Level 1 | Body effect, channel-length modulation |
| Vacuum Triode | Norman Koren + Leach grid current | 12AX7, 12AU7, 12AT7, 6SN7, 6SL7, and more |
| Vacuum Pentode | 5 equation families, 29 models | EL84, EL34, EF86, 6L6, 6V6, KT88, 6550, and 22 more. Auto grid-off optimization for cutoff |
| Op-Amp | Boyle VCCS macromodel | GBW pole, slew-rate limiting, asymmetric VCC/VEE rails, 4 clamping strategies |
| VCA | THAT 2180 exponential | Current-mode with gain-dependent THD |
| CdS LDR | VTL5C3/4, NSL-32 | Asymmetric attack/release envelope |
| Potentiometer | `.pot` / `.wiper` / `.gang` directives | Per-sample smoothing; `recompute_dc_op()` available for preset-recall NR-seed refresh |
| Switch | `.switch` directive | Ganged R/C/L component switching |

All device parameters use standard SPICE `.model` syntax. Temperature is fixed at 27°C; no noise simulation.

## Circuits

Circuit netlists live in the [melange-audio/circuits](https://github.com/melange-audio/circuits) repo, organized into `stable/`, `testing/`, and `unstable/` tiers. Promotion requires a DAW listening test, not just SPICE correlation numbers.

Melange auto-selects the optimal solver for each circuit (DK, Nodal Schur, or Nodal Full LU). Override with `--solver dk` or `--solver nodal` if needed.

Here's a sample of what melange handles:

| Circuit | What it is | Devices | Performance |
|---------|-----------|---------|-------------|
| Passive tube EQ (Pultec-class) | 7 pots, 3 switches, global NFB | 4 tubes, 2 transformers | ~11× RT |
| Wurlitzer 200A preamp | LDR tremolo, 2-stage BJT | 2 BJTs + 1 diode | ~500× RT |
| Tweed preamp (Fender-class) | 2× 12AX7 with volume pot + bright switch | 2 triodes | ~400× RT |
| Full power amps (AC15/Tweed Deluxe/Plexi-class) | Push-pull pentode output stages + OT | 3-7 tubes | varies |
| Pedal circuits (overdrive, fuzz, distortion) | Op-amp and transistor-based clipping | 1-4 NL devices | ~400× RT |
| Bus compressor (SSL-class) | VCA + op-amp sidechain | 4 op-amps + 1 VCA | ~42× RT |

See the [circuits repo](https://github.com/melange-audio/circuits) for the full catalog with per-circuit status.

## Spotlight: Passive Tube EQ

The hardest circuit melange runs end-to-end, and the clearest proof it handles real vintage gear:

- **4 vacuum tubes** (2× 12AX7, 2× 12AU7), **2 transformers** (input step-up, output with tertiary feedback winding)
- **21 dB of global negative feedback** via differential cathode injection
- **41 circuit nodes, 8 nonlinear dimensions** — every component from the original schematic
- **All 7 EQ curves verified**, including the simultaneous boost + cut trick
- **Zero NR failures** at 1V input, ~11× realtime

```bash
melange compile melange:stable/filters/passive-eq1a --format plugin -o my-eq
melange analyze melange:stable/filters/passive-eq1a --pot "LF Boost=10" --switch "LF Freq=1"
```

## Impossible Circuits

A germanium transistor is just a BJT with different `.model` parameters -- higher leakage current, lower forward voltage (~0.2V vs 0.6V), lower gain. Melange doesn't care. Change `IS`, `BF`, and `NF` in your netlist and you have a germanium device. No special codepath, no approximation.

This matters because NOS germanium transistors are a dwindling supply. Nobody manufactures them anymore. The stockpiles from the 1960s are finite, and matched pairs command collector prices. Building a single Fuzz Face or Tone Bender with hand-selected germanium transistors is hard enough. Building a 16-stage germanium cascade -- which melange runs at 3.4x realtime -- is not practically possible in hardware. You would need 16+ matched devices from a pool that no longer exists, and germanium's temperature drift would compound across stages into thermal instability that no amount of bias trimming can tame.

In simulation, none of that applies. Every device holds its parameters permanently. You can sweep `IS` from silicon to germanium to see exactly where the character changes, lock in the values you like, and ship a plugin built on transistors that left the factory sixty years ago. The same principle extends to every device melange supports: vintage tube types with known-good plate curves, op-amps at specific supply rails, transformers with exact winding ratios. If you can write the `.model` line, you can build the circuit.

## SPICE Validation

Every stable circuit is validated sample-by-sample against ngspice. This isn't "sounds close" — it's correlation coefficients and RMS error against a reference SPICE simulator used in IC design.

| Circuit | Correlation | RMS Error |
|---------|------------|-----------|
| RC lowpass (linear reference) | 0.99999999 (8 nines) | 0.03% |
| Wurlitzer 200A preamp | 0.99999998 (6 nines) | 0.014% |
| BJT common-emitter | 0.9997 (4 nines) | 5.6% |

Run your own validation (requires `ngspice` on PATH):

```bash
melange validate my-circuit.cir --output-node out
```

## CLI Reference

```
melange compile <circuit> -o <dir>        Compile to Rust code or plugin project
melange simulate <circuit> -o <file.wav>  Process audio through a circuit
melange analyze <circuit>                 Frequency response sweep
melange validate <circuit>                Compare against ngspice
melange nodes <circuit>                   List nodes and devices
melange import <file.xml> -o <file.cir>   Import KiCad XML to Melange format
melange builtins                          List embedded circuits (deprecated — use sources)
melange sources add|list|remove           Manage circuit source repos
```

Every subcommand has `--help`. Key flags:

| Flag | On | Does |
|------|----|------|
| `--format plugin` | compile | Emit a full nih-plug project (default: raw code) |
| `--solver auto\|dk\|nodal` | compile/simulate | Override solver selection |
| `--oversampling 2\|4` | compile | 2× or 4× polyphase half-band IIR antialiasing |
| `--backward-euler` | compile | L-stable integration for high-gain feedback circuits |
| `--pot "Name=Value"` | analyze | Set pot value for frequency sweep |
| `--switch "Name=Pos"` | analyze | Set switch position for frequency sweep |
| `--input-audio file.wav` | simulate | Use a WAV file instead of a test tone |
| `--no-ear-protection` | compile | Disable soft limiter (measurement only) |

Full flag documentation: [`compile --help`](docs/PLUGIN_GUIDE.md), [`simulate --help`](docs/GETTING_STARTED.md), [`analyze --help`](docs/GETTING_STARTED.md).

## Architecture

```
SPICE Netlist ─┐
               ├──► Parser ──► MNA System ──► DK Kernel ──► CircuitIR ──► Emitter ──► Code
KiCad XML ─────┘
```

The `CircuitIR` intermediate representation is language-agnostic and serializable. The current emitter targets Rust; C++ and FAUST are planned. The `Emitter` trait is public — write your own backend if you need a different target.

```
crates/
  melange-primitives/   DSP building blocks (filters, oversampling, NR helpers)
  melange-devices/      Component models (diode, BJT, JFET, MOSFET, tube, opamp, VCA, LDR)
  melange-solver/       MNA/DK engine + codegen — the core
  melange-validate/     SPICE validation pipeline (ngspice comparison)
  melange-plugin/       nih-plug integration

tools/
  melange-cli/          Command-line front-end
```

For the math internals (MNA stamping, DK method, Newton-Raphson, Gummel-Poon BJT, tube models, convergence strategies, codegen templates), see [`docs/aidocs/INDEX.md`](docs/aidocs/INDEX.md).

## Cross-Compilation

Build macOS plugins from Linux:

```bash
cargo zigbuild --release --target universal2-apple-darwin
rcodesign sign target/universal2-apple-darwin/release/libmy_plugin.dylib
```

Requires zig 0.13+ and cargo-zigbuild. See [docs](docs/PLUGIN_GUIDE.md) for details.

## Requirements

- Rust 1.85+ (2021 edition)
- No external dependencies for core library or generated code
- Optional: ngspice for SPICE validation
- Optional: zig + cargo-zigbuild for macOS cross-compilation
- Optional: KiCad 8+ for schematic-based workflow

```bash
# Install the CLI
cargo install --path tools/melange-cli

# Or build from repo
cargo build --workspace
cargo test --workspace        # ~1100 tests
```

## Known Limitations

- Fixed temperature (27°C) — no temperature coefficients
- Transformers assume constant coupling (no core saturation or hysteresis)
- No noise simulation (shot, thermal, 1/f)
- Tube models: no space-charge or transit-time effects
- Op-amps: Boyle macromodel (adequate for audio, not a full transistor-level sim)
- See [`docs/aidocs/STATUS.md`](docs/aidocs/STATUS.md) for the complete list

## Origin

Melange was extracted from the [OpenWurli](https://github.com/openwurli/openwurli) project, a Wurlitzer 200A virtual instrument.

## License

[GNU General Public License v3.0 or later](LICENSE) (GPL-3.0-or-later).

**Generated code is also GPL-licensed.** Code produced by `melange compile` is derived from GPL-licensed templates and device models, and is subject to the same license. If you distribute plugins built from melange-generated code, you must comply with the GPL (including providing source code to recipients).

> **Melange needs additional maintainers.** The current maintainer lacks deep EE and Rust expertise for a project of this complexity. If you can help, please get in touch.

## Acknowledgments

### A huge debt to SPICE and ngspice

Melange is, in a very real sense, SPICE with a different front end and a different back end. Almost every piece of numerical behavior that makes the generated code produce the right answer traces back to 50+ years of work on SPICE and, more recently, to the open-source ngspice project.

The device model equations are SPICE's. The convergence machinery (pnjlim, fetlim, source-stepping, Gmin-stepping) is SPICE's. The MNA stamping rules are SPICE's. The parameter names in `.model` lines (`IS`, `BF`, `VAF`, `IKF`, `LAMBDA`, `KP`, `VTO`, `CJE`, ...) are SPICE parameter names. Validated circuits pass a correlation + RMS error test against ngspice. When melange and ngspice disagree, the bug is in melange until proven otherwise.

If SPICE is "the physics of analog circuit simulation," then melange is "that physics, compiled ahead-of-time into a branchless inner loop you can run on the audio thread."

**Please donate to or contribute to [ngspice](https://ngspice.sourceforge.io/) if you find melange useful.** Their work is the foundation we build on.

### Core methods

- David Yeh — Discrete K-method for audio circuit simulation (PhD thesis, Stanford, 2009)
- Larry Nagel, Tom Quarles, and the ngspice maintainers — SPICE, SPICE3f5, and [ngspice](https://ngspice.sourceforge.io/)
- Ho, Ruehli, Brennan — Modified Nodal Analysis (IEEE Trans. CAS, 1975)

### Device models

- Hermann Gummel & H.C. Poon — BJT model (Bell Labs, 1970)
- Jiri Shichman & David Hodges — JFET model (IEEE JSSC, 1968)
- Norman Koren — vacuum tube models (1996)
- Derk Reefman — pentode/beam tetrode models and variable-mu blend (uTracer, TubeLib.inc 2016)
- Marshall Leach — tube grid current model
- William Shockley — semiconductor diode equation

### DSP

- Olli Niemitalo — polyphase allpass half-band IIR for oversampling
- Vadim Zavalishin — topology-preserving transform filters

### References

- Pillage, Rohrer, Visweswariah — *Electronic Circuit and System Simulation Methods* (McGraw-Hill, 1995)
- [Hack Audio circuit modeling tutorial](https://hackaudio.com/tutorial-courses/audio-circuit-modeling-tutorial/)
