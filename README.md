# Melange

*The SPICE must flow.*

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-2021%20edition-orange.svg)](https://www.rust-lang.org)

An open-source Rust toolkit that compiles SPICE netlists into optimized real-time audio DSP code — the step that every circuit modeler currently does by hand. Point it at a netlist and it emits a standalone Rust solver or a ready-to-build CLAP/VST3 plugin.

> **Project status: early alpha.** Current development focus has been on supporting the hardware needed for the [OpenWurli](https://github.com/openwurli/openwurli) project, plus a handful of validated vintage studio circuits (Pultec EQP-1A, tweed preamp, Wurlitzer 200A preamp). Most of the built-in circuits are still under investigation — see the [Built-in Circuits](#built-in-circuits) table for per-circuit status. Additional circuit tests are in the works.

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

### SPICE netlist to audio plugin

```bash
melange compile tube-screamer --format plugin -o my-ts
cd my-ts && cargo build --release
# You now have a VST3/CLAP plugin.
```

That's it. One command turns a circuit into a buildable nih-plug project with all the DSP, parameters, and real-time-safe code generated for you.

### Simulate a circuit to WAV

```bash
# Push a guitar recording through a Tube Screamer
melange simulate tube-screamer --input-audio guitar.wav -o output.wav

# Quick test tone through the same circuit
melange simulate tube-screamer --amplitude 0.1 -o drive.wav
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
melange nodes tube-screamer
# Nodes in circuit:
#   (0) GND - Ground reference
#   (1) in
#   (2) v_plus
#   (3) v_minus
#   ...
# Nonlinear devices:
#   U1: OpAmp (dimension: 0, linear VCCS)
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
- **[Math & Internals (`docs/aidocs/`)](docs/aidocs/INDEX.md)** — MNA, DK-method, Newton-Raphson, Gummel-Poon, Sherman-Morrison, oversampling, codegen, and validation reference docs. Dense and equation-heavy — start here if you want to understand or modify the solver core.

## Features

- **SPICE Netlist Parser**: Industry-standard SPICE netlists with `.model` and `.subckt` support
- **MNA/DK Solver**: Modified Nodal Analysis with Discrete K-method (Yeh 2009)
- **Device Models**: Diode, BJT (Ebers-Moll/Gummel-Poon), JFET, MOSFET, vacuum tube (Koren), op-amp, VCA, CdS LDR
- **Dynamic Controls**: `.pot` (linear potentiometer), `.wiper` (tapered wiper across two resistors), `.switch` (ganged R/C/L switching), and `.gang` (link multiple pots/wipers to a single UI knob) directives
- **Real-Time Safe**: Zero heap allocation in audio callback, no `unsafe` code, f64 precision
- **Code Generation**: Optimized Rust with compile-time constants, sparse matrices, unrolled loops
- **Plugin Generation**: One-step CLAP/VST3 plugin projects via nih-plug
- **SPICE Validation**: Automated comparison against ngspice reference simulations
- **Cross-Compilation**: Build macOS plugins from Linux via cargo-zigbuild

## Supported Components

All nonlinear devices are handled in the **codegen pipeline** — `melange compile` emits
straight-line Rust code with the device equations baked in. Purely linear circuits (M=0)
can also run via an in-process `LinearSolver`, but most users will only touch the codegen path.

| Component | Model |
|-----------|-------|
| Resistor / Capacitor / Inductor | Trapezoidal companion models; inductors via augmented MNA |
| Diode | Shockley + series resistance + Zener breakdown (BV/IBV); LED variant |
| BJT | Ebers-Moll or Gummel-Poon (VAF/VAR/IKF/IKR); CJE/CJC; parasitic RB/RC/RE |
| JFET | Shichman-Hodges 2D (triode + saturation + λ); CGS/CGD; parasitic RD/RS |
| MOSFET | Level 1 SPICE 2D; body effect (GAMMA/PHI); CGS/CGD |
| Vacuum Tube | Koren triode + Leach grid current + λ; CCG/CGP/CCP; parasitic RGI |
| Op-Amp | Boyle VCCS macromodel (AOL, ROUT, GBW pole, VCC/VEE rail clamping) |
| VCA | THAT 2180 exponential gain (current-mode, THD) |
| CdS LDR | VTL5C3/4, NSL-32 with asymmetric envelope (used as a dynamic resistor) |
| Voltage Source | DC (Norton equivalent) |
| Potentiometer | `.pot` + `.wiper` directives; Sherman-Morrison rank-1 updates |
| Switch | `.switch` directive; ganged R/C/L component switching |

Pentodes, temperature coefficients, slew-rate limiting, and noise models are not yet implemented.
See [Known Limitations](#known-limitations) and `docs/aidocs/STATUS.md` for the full list.

## Validated Circuit Spotlight: Pultec EQP-1A

The Pultec EQP-1A passive EQ is the hardest circuit melange currently simulates
end-to-end, and the clearest evidence it handles real-world vintage gear:

- **4 vacuum tubes** (2× 12AX7, 2× 12AU7 Koren models)
- **2 transformers** (HS-29 input, S-217-D output with tertiary feedback winding), verified against Sowter DWG E-72,658-2
- **21 dB of global negative feedback** via differential cathode injection through the S-217-D tertiary
- **7 pots + 3 switches** (LF Boost / LF Atten / HF Boost / HF Cut / Bandwidth, LF Freq / HF Freq / HF Cut Freq)
- **N=41 nodes, M=8 nonlinear dimensions**, routed to the nodal full-LU path
- **Gain: +1.8 dB at 1 kHz** (near unity, spec is 0 dB), flat ±1 dB from 20 Hz to 15 kHz
- **All 7 EQ curves verified** against the schematic, including the famous Pultec Trick
- **Runs ~11× realtime** with chord method + cross-timestep Jacobian persistence + compile-time sparse LU
- **Zero NR failures** at 1 V input

Every feature in this README that isn't a smoke test was built to make this circuit work.
If you want to see what the numerical stack looks like in one circuit, start here:

```bash
melange compile circuits/stable/pultec-eq.cir --format plugin -o pultec-plugin
melange analyze circuits/stable/pultec-eq.cir --pot "LF Boost=10" --switch "LF Freq=1"
```

## Architecture

```
crates/
  Layer 5: melange-plugin     — nih-plug integration (voice mgmt, oversampling, params)
  Layer 4: melange-validate   — SPICE-to-Rust validation pipeline
  Layer 3: melange-solver     — MNA/DK-method engine + code generation (THE CORE)
  Layer 2: melange-devices    — Component models (BJT, tube, diode, LDR, opamp)
  Layer 1: melange-primitives — DSP building blocks (filters, oversampling, NR helpers)

tools/
  melange-cli                 — command-line front-end (consumes Layers 3 + 4)
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

Example circuits from `circuits/stable/` (48kHz, release build):

| Circuit | N | M | Solver | Performance |
|---------|---|---|--------|-------------|
| RC lowpass | 2 | 0 | Linear | trivial (linear reference) |
| Wurli preamp (2× 2N5089 + diode) | 11 | 3–5 | DK | fast (DK range: 100–600× realtime) |
| Tweed preamp (2× 12AX7) | 13 | 4 | DK | fast (DK range: 100–600× realtime) |
| Pultec EQP-1A (4 tubes, 2 transformers) | 41 | 8 | Nodal full LU | ~11× (chord + cross-timestep + sparse LU) |

Override with `--solver dk` or `--solver nodal` if auto-selection doesn't suit your needs.
See `docs/aidocs/STATUS.md` for the full validated-circuits table, including circuits
in `testing/` and `unstable/` that are not yet user-verified.

## Built-in Circuits

Melange embeds a handful of classic circuits you can compile by name (`melange compile <name>`).
Most are under active development — only `rc-lowpass` has passed full user sign-off.
Promotion to `stable/` requires a DAW listening test, not just SPICE correlation.

| Circuit | Status | Description |
|---------|--------|-------------|
| `rc-lowpass` | stable | Simple RC lowpass filter (linear reference / smoke test) |
| `tube-screamer` | testing | Op-amp overdrive with diode feedback clipping (TS808 style) |
| `tube-preamp` | testing | Common-cathode 12AX7 triode gain stage with tone control |
| `mordor-screamer` | testing | High-gain distortion forged in Mount Doom |
| `fuzz-face` | unstable | 2-transistor fuzz (Fuzz Face style) — under investigation |
| `big-muff` | unstable | 4-transistor fuzz with dual clipping — under investigation |
| `ssl-bus-compressor` | unstable | SSL 4000E bus compressor — audio path works, full plugin not stable |

The repository's `circuits/stable/` directory also contains user-verified circuits that
aren't embedded as built-ins. Clone the repo and compile by path:

| Circuit | Description |
|---------|-------------|
| `circuits/stable/wurli-preamp.cir` | Wurlitzer 200A preamp (2 BJTs + diode, LDR tremolo) |
| `circuits/stable/tweed-preamp.cir` | Fender-style 2× 12AX7 guitar preamp (volume pot, bright switch) |
| `circuits/stable/pultec-eq.cir` | **Pultec EQP-1A** — 4 tubes, 2 transformers, 7 pots, 3 switches, 21 dB global NFB (see [spotlight](#validated-circuit-spotlight-pultec-eqp-1a) above) |

See `docs/aidocs/STATUS.md` for the full promotion criteria and the list of circuits currently in `testing/` and `unstable/`.

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
melange compile <circuit> -o <output>      Compile circuit to Rust code or plugin project
melange simulate <circuit> -o <output>     Simulate circuit, write WAV output
melange analyze <circuit>                  Analyze circuit frequency response
melange validate <circuit>                 Compare against ngspice reference
melange nodes <circuit>                    List nodes and devices in a circuit
melange import <file.xml> -o <file.cir>    Import a KiCad XML or SPICE netlist to Melange format
melange builtins                           List built-in circuits
melange sources add <name> <url>           Add a git repo as a circuit source
melange sources list|show|remove           Manage configured sources
melange cache list|clear|stats             Manage cached circuits
```

Every subcommand supports `--help` for the full flag list. Highlights below.

### `compile` — SPICE netlist to Rust code or nih-plug plugin

```
-f, --format <code|plugin>      Output format (default: code)
    --name <str>                Plugin display name (defaults to capitalized circuit filename)
    --mono                      Generate mono plugin (default: stereo)
    --wet-dry-mix               Add a wet/dry mix parameter to the generated plugin
-s, --sample-rate <Hz>          Target sample rate (default: 48000)
-I, --input-node <name>         Input node name (default: "in")
-n, --output-node <name>        Output node name(s), comma-separated for multi-output
    --input-resistance <Ω>      Override Thevenin source impedance (default: 1Ω or .input_impedance directive)
    --solver <auto|dk|nodal>    Solver backend (default: auto)
    --oversampling <1|2|4>      Oversampling factor (default: 1)
    --backward-euler            Backward Euler integration instead of trapezoidal (fixes high-gain feedback divergence)
    --opamp-rail-mode <MODE>    Op-amp rail saturation strategy: auto | none | hard | active-set | boyle-diodes (default: auto)
    --max-iter <N>              Maximum NR iterations (default: 50)
    --tolerance <f64>           NR convergence tolerance (default: 1e-9)
    --output-scale <f64>        Circuit-volts → DAW-units mapping (default: 1.0). Not a bug-fix knob.
    --no-level-params           Omit Input/Output Level parameters from plugin
    --no-dc-block               Disable the 5 Hz DC blocking filter
    --no-ear-protection         Disable the default soft limiter (measurement / testing only)
```

### `simulate` — run a circuit against a WAV file or test tone

```
-a, --input-audio <file>        Input WAV file (omit to generate a test tone)
-o, --output <file>             Output WAV file
-s, --sample-rate <Hz>          Sample rate (used when no input audio) (default: 48000)
-d, --duration <s>              Duration in seconds for generated test tone (default: 1.0)
    --amplitude <0..1>          Test-tone amplitude (default: 0.5)
-I, --input-node <name>         Input node name (default: "in")
-n, --output-node <name>        Output node name (default: "out")
    --input-resistance <Ω>      Override Thevenin source impedance
    --solver <auto|dk|nodal>    Solver backend (default: auto)
    --opamp-rail-mode <MODE>    auto | none | hard | active-set | active-set-be | boyle-diodes
```

### `analyze` — frequency-response sweep

```
    --start-freq <Hz>           Start frequency (default: 20)
    --end-freq <Hz>             End frequency (default: 20000)
    --points-per-decade <N>     Frequency resolution (default: 10)
    --amplitude <V>             Input amplitude (default: 0.1)
-s, --sample-rate <Hz>          Internal sample rate (default: 96000)
-I, --input-node <name>         Input node name (default: "in")
-n, --output-node <name>        Output node name (default: "out")
    --input-resistance <Ω>      Override Thevenin source impedance
    --pot <NAME=VALUE>          Set pot value (e.g. `--pot "LF Boost=10k"`). Repeatable.
    --switch <NAME=POS>         Set switch position (e.g. `--switch "LF Freq=3"`). Repeatable.
-o, --output <file>             Write CSV to file (default: stdout)
```

`--pot` / `--switch` are the whole point of running `analyze` on circuits like the Pultec —
they let you sweep EQ curves by setting each control to a specific value per run.

### `validate` — compare melange output against ngspice

```
-n, --output-node <name>        Output node name (default: "out")
-s, --sample-rate <Hz>          Sample rate (default: 48000)
    --duration <s>              Test signal duration (default: 1.0)
    --amplitude <V>             Test signal amplitude (default: 0.1)
    --csv <file>                Write per-sample comparison to CSV
    --relaxed                   Use relaxed tolerances (1% RMS, 0.999 correlation)
```

Requires `ngspice` on `PATH`.

Circuits can be referenced as:
- **Built-in name**: `tube-screamer`, `fuzz-face`, `big-muff`, ...
- **Local file**: `path/to/circuit.cir`
- **Source reference**: `sourcename:circuitname`

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

Add human-readable labels to `.pot`, `.wiper`, and `.switch` directives in your netlist:

```spice
.pot R_vol 500 500k "Volume"
.wiper R_cw R_ccw 100k 0.5 "Fuzz"
.switch C_bright 1p 120p 470p "Bright"
.gang "Gain" R_gain_a R_gain_b    # link two pots to one knob
```

Without labels, params fall back to the component name (e.g. `R_vol`, `Switch 0 (C_bright)`).

### Op-Amp Supply Rails

Op-amps in melange use the Boyle VCCS macromodel. Specify supply rails on the `.model`
line — symmetric rails via `VSAT`, or asymmetric via explicit `VCC` / `VEE`:

```spice
U1 vin- vin+ vout OA_TL072
.model OA_TL072 OA(AOL=200000 ROUT=75 GBW=3Meg VSAT=13)    ; ±13 V
.model OA_SINGLE OA(AOL=100000 ROUT=75 GBW=1Meg VCC=9 VEE=0)   ; 0..9 V single-supply
.model OA_CHARGE OA(AOL=200000 ROUT=75 GBW=3Meg VCC=18 VEE=-9) ; asymmetric charge pump
```

When the op-amp output approaches the rails, `--opamp-rail-mode` controls the clamping strategy:

| Mode | Behavior | Use when |
|------|----------|----------|
| `auto` (default) | Inspect the circuit and pick the cheapest correct mode; logged at compile time | Always start here |
| `none` | No clamping (unbounded output) | Verified-linear circuits only |
| `hard` | Post-NR `v.clamp(VEE, VCC)` — cheap, breaks KCL for AC-coupled downstream caps | Legacy / debugging |
| `active-set` | Post-NR constrained re-solve (KCL-consistent hard clip) | Klon-class circuits with cap coupling |
| `boyle-diodes` | Auto-inserted physical catch diodes to rail-offset sources (soft exponential knee) | Distortion pedals where the clip shape matters |

### What's Safe to Customize in lib.rs

- Parameter names, ranges, units, and smoothing
- Plugin metadata (name, vendor, description)
- GUI (nih-plug supports [VIZIA](https://github.com/vizia/vizia) and [iced](https://github.com/iced-rs/iced))
- Pre/post processing (wet/dry mix, oversampling control)
- Presets and state persistence

## Known Limitations

- **Fixed temperature (27°C)**: All device models simulate at 300.15K. No temperature coefficients.
- **Triode tubes only**: Pentode tubes (EL84, 6L6, EL34) are not yet supported. 12AX7, 12AU7, and other preamp triodes work fully.
- **Ideal transformers**: Coupled inductors assume constant coupling coefficient with no core saturation or hysteresis.
- **Op-amps**: Boyle VCCS macromodel with optional GBW dominant pole, symmetric (`VSAT`) or asymmetric (`VCC`/`VEE`) supply rails, and five selectable rail-clamping strategies (`--opamp-rail-mode`). No slew-rate limiting.
- **No noise simulation**: Shot noise, thermal noise, and 1/f noise are not modeled.

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

## Audio Safety Features

Three features protect your ears and speakers (all default-on, all optional):

- **Ear protection** — soft limiter that engages near 0 dBFS. Toggle at runtime or disable with `--no-ear-protection`.
- **DC blocking** — 5 Hz high-pass filter removes DC offset from circuit output. Settles in ~200ms. Disable with `--no-dc-block` if the circuit has its own output coupling capacitor.
- **Oversampling** (opt-in) — 2x or 4x polyphase half-band IIR reduces aliasing from nonlinear devices. Enable with `--oversampling 2` or `--oversampling 4`. CPU cost scales linearly.

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Plugin produces silence | Wrong input/output node names | Check `melange nodes circuit.cir`; use `--input-node` / `--output-node` |
| NaN, oscillation, or blown-up output | DC operating point failed to converge, or feedback loop is running away | Check biasing and supply voltages; try `--backward-euler` for high-gain feedback circuits; for op-amp circuits try `--opamp-rail-mode boyle-diodes` |
| Output is in millivolts when you expected volts | Real circuit output level is low (e.g. a guitar-level stage into another stage) | Use `--output-scale` only as a **unit mapping** to match the downstream stage's expected range. Do **not** use it to cover up a solver bug — if the circuit should output volts and it doesn't, that's a modeling/solver issue, not a gain issue. File a report. |
| "M exceeds MAX_M" error | Too many nonlinear dimensions for DK solver | Use `--solver nodal` |
| Clicks on parameter changes | Pot smoothing too short | Increase smoother duration in `lib.rs` |
| ~3% error vs ngspice | Using wrong RHS formulation | Ensure trapezoidal (default); check `.OPTIONS INTERP` in SPICE netlist |
| Output all zeros | VIN source in no-vin netlist | Check that the circuit file doesn't include VIN (melange provides input via conductance) |

See [docs/PLUGIN_GUIDE.md](docs/PLUGIN_GUIDE.md) for plugin-specific troubleshooting.

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
cargo test --workspace        # Run all tests (~1100 tests)
cargo run -p melange-cli -- compile tube-screamer --format plugin -o my-plugin
```

## Roadmap

Melange currently generates self-contained Rust code. Multi-language codegen is coming soon — the internal intermediate representation (`CircuitIR`) and `Emitter` trait are already language-agnostic by design. C++ will be the first additional target, with other languages to follow based on community interest. The goal is to let you compile a SPICE netlist directly to native code in whatever language your plugin framework or audio engine uses.

## Origin

Melange was extracted from the [OpenWurli](https://github.com/openwurli/openwurli) project, a Wurlitzer 200A virtual instrument. The DK solver, device models, and validation pipeline were generalized into this standalone toolkit.

## License

This project is licensed under the [GNU General Public License v3.0 or later](LICENSE) (GPL-3.0-or-later).

**Generated code is also GPL-licensed.** Code produced by `melange compile` is derived from
GPL-licensed templates and device models within this project, and is therefore subject to the
same GPL-3.0-or-later license. If you distribute plugins or binaries built from Melange-generated
code, you must comply with the GPL (including providing source code to recipients).

Melange includes code derived from the [OpenWurli](https://github.com/openwurli/openwurli) project, which is licensed under GPL-3.0.

> **Melange needs additional maintainers!** The current maintainer is a SWE but does not have the necessary electrical engineering or Rust expertise to maintain a project of this complexity long-term. If you've got the skills to help, please get in touch.

## Acknowledgments

### A huge debt to SPICE and ngspice

Melange is, in a very real sense, SPICE with a different front end and a different back end. Almost every piece of numerical behavior that makes the generated code produce the right answer traces back to 50+ years of work on SPICE and, more recently, to the open-source ngspice project.

- **The device model equations are SPICE's.** The Gummel-Poon BJT (`qb`, `q1`, `q2`, high-injection knee), the Shichman-Hodges JFET, the SPICE Level 1 MOSFET, the Shockley diode with series resistance and reverse breakdown, the junction-capacitance depletion formulas (`CJ0 / (1 - Vj/VJ)^MJ`), the Boyle op-amp macromodel — these are all re-implementations of the math written down decades ago in the SPICE2/SPICE3 literature and source code. The parameter names you use in `.model` lines (`IS`, `BF`, `VAF`, `IKF`, `NF`, `LAMBDA`, `KP`, `VTO`, `CGS`, `CJE`, …) are SPICE parameter names because there is no better convention and no reason to invent one.
- **The convergence machinery is SPICE's.** `pnjlim` voltage limiting for pn junctions, `fetlim` for FETs, the source-stepping and Gmin-stepping fallback chain for DC operating point, the RELTOL/VNTOL/ABSTOL tolerance philosophy, the singular-matrix and off-diagonal-pivot checks in the LU factorization — all of this is taken from SPICE3f5 and ngspice's `bjtload.c` / `dioload.c` / `cktop.c`. Where we were unsure what the right thing was, we read ngspice's source and followed it line by line.
- **ngspice is the reference oracle.** Every circuit that ships in `circuits/stable/` passes a correlation + RMS error test against an ngspice run of the same netlist. The `melange-validate` crate shells out to `ngspice` as a subprocess, parses its `.OPTIONS INTERP` output, and compares sample-for-sample. When melange and ngspice disagree, the bug is in melange until proven otherwise. We cannot overstate how much confidence it gives to have a mature, open-source SPICE engine on the other side of every regression test.
- **The MNA stamping rules are SPICE's.** How conductance stamps contribute to G, how companion models for caps/inductors contribute to G and rhs, how VCCS / VCVS / op-amps stamp into the augmented matrix, the sign conventions for current flow — all of this follows the SPICE2 / Nagel (1975) formulation.

If SPICE is "the physics of analog circuit simulation," then melange is "that physics, compiled ahead-of-time into a branchless inner loop you can run on the audio thread." We are standing on Larry Nagel, Tom Quarles, Hermann Gummel, Norman Koren, and the dozens of ngspice maintainers whose names appear in `ngspice-42/src/spicelib/devices/*/load.c`.

**Please donate to or contribute to [ngspice](https://ngspice.sourceforge.io/) if you find melange useful.** Their work is the foundation we build on.

### Core methods

- David Yeh — Discrete K-method for audio circuit simulation (PhD thesis, Stanford, 2009)
- Larry Nagel, Tom Quarles, and the ngspice maintainers — SPICE (UC Berkeley), SPICE3f5, and [ngspice](https://ngspice.sourceforge.io/); source of every device model, convergence heuristic, and validation reference in this project. See note above.
- Ho, Ruehli, Brennan — Modified Nodal Analysis (IEEE Trans. CAS, 1975)
- Sherman & Morrison — rank-1 matrix update formula (used for dynamic potentiometers)

### Device models

- Hermann Gummel & H.C. Poon — BJT model with Early effect and high-injection (Bell Labs, 1970)
- Jiri Shichman & David Hodges — JFET model (IEEE JSSC, 1968)
- Norman Koren — triode vacuum tube model (1996)
- Marshall Leach — grid current model for vacuum tubes
- William Shockley — semiconductor diode equation

### DSP & filters

- Olli Niemitalo — polyphase allpass half-band IIR filters for oversampling
- Vadim Zavalishin — topology-preserving transform (TPT) filters

### References

- Pillage, Rohrer, Visweswariah — *Electronic Circuit and System Simulation Methods* (McGraw-Hill, 1995)
- [Hack Audio circuit modeling tutorial](https://hackaudio.com/tutorial-courses/audio-circuit-modeling-tutorial/)
- [TU Delft analog electronics webbook](https://analog-electronics.ewi.tudelft.nl/webbook/SED/)
