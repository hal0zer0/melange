# Getting Started with Melange

Melange compiles SPICE circuit netlists into real-time audio DSP code. This guide takes you from zero to a working audio plugin.

## Prerequisites

- **Rust 1.85+** — [rustup.rs](https://rustup.rs)
- **melange CLI** — install from the repo:
  ```bash
  cargo install --path tools/melange-cli
  ```

Optional:
- **ngspice** — for validation against SPICE reference (`apt install ngspice` / `brew install ngspice`)
- **zig + cargo-zigbuild** — for macOS cross-compilation from Linux

## Quick Start: Circuit to Plugin

Compile a circuit from the [melange-audio/circuits](https://github.com/melange-audio/circuits) library, or use a local `.cir` file:

```bash
melange compile melange:unstable/pedals/peaches --format plugin -o my-fuzz
# or from a local file:
melange compile my-circuit.cir --format plugin -o my-fuzz
```

This generates a complete nih-plug project in `my-fuzz/` with:
- `src/circuit.rs` — generated DSP code (do not edit)
- `src/lib.rs` — plugin wrapper (safe to customize)
- `Cargo.toml` — ready to build
- `README.md` — build instructions

Build the plugin:

```bash
cd my-fuzz
cargo nih-plug-xtask bundle my_fuzz --release
```

The compiled CLAP and VST3 plugins appear in `target/bundled/`.

**Testing:** Always start with your monitor volume at zero and increase gradually — circuit simulations can produce unexpected levels.

## Quick Start: Your Own Circuit

Write a SPICE netlist. Here's a simple diode clipper:

```spice
* Diode Clipper
Rin in mid 4.7k
D1 mid 0 1N4148
D2 0 mid 1N4148
Rload mid out 1k
Cout out 0 100n

.model 1N4148 D(IS=2.52e-9 N=1.752)
.end
```

Save it as `clipper.cir`, then:

```bash
# Inspect the circuit
melange nodes clipper.cir

# Quick audio test (no compilation needed)
melange simulate clipper.cir --amplitude 0.1 -o test.wav

# Compile to a plugin
melange compile clipper.cir --format plugin -o my-clipper
cd my-clipper
cargo nih-plug-xtask bundle my_clipper --release
```

## Adding Controls

Mark resistors as pots and capacitors as switches in your netlist:

```spice
R_vol mid out 50k
.pot R_vol 1k 100k "Volume"

C_bright 1 0 120p
.switch C_bright 1p 120p 470p "Bright"
```

Each `.pot` becomes a knob and each `.switch` becomes a selector in the generated plugin. See [spice-grammar.md](spice-grammar.md#5-melange-extensions) for full syntax.

## Common CLI Commands

| Command | What it does |
|---------|-------------|
| `melange sources list` | List configured circuit sources |
| `melange nodes circuit.cir` | Show nodes and devices |
| `melange compile circuit.cir -f plugin -o dir` | Generate plugin project |
| `melange compile circuit.cir -f code -o file.rs` | Generate standalone Rust code |
| `melange simulate circuit.cir --amplitude 0.1 -o out.wav` | Process test tone |
| `melange simulate circuit.cir --input-audio audio.wav -o out.wav` | Process audio file |
| `melange analyze circuit.cir` | AC frequency response |
| `melange validate circuit.cir` | Compare against ngspice |

## Compile Options

Key flags for `melange compile`:

| Flag | Default | Description |
|------|---------|-------------|
| `--format code\|plugin` | `code` | Output format |
| `--sample-rate` | 48000 | Design sample rate (Hz) |
| `--oversampling 1\|2\|4` | 1 | Anti-aliasing oversampling factor |
| `--input-node` | `in` | Input node name in netlist |
| `--output-node` | `out` | Output node name(s), comma-separated for stereo |
| `--solver auto\|dk\|nodal` | `auto` | Solver selection (auto picks the best) |
| `--no-dc-block` | off | Disable 5Hz DC blocking filter |
| `--no-level-params` | off | Omit Input/Output Level knobs |
| `--no-ear-protection` | off | Disable output soft limiter |
| `--wet-dry-mix` | off | Add wet/dry mix parameter |
| `--mono` | off | Generate mono instead of stereo |
| `--backward-euler` | off | Use backward Euler (unconditionally stable) |
| `--tube-grid-fa auto\|on\|off` | `auto` | Pentode grid-off dimension reduction |
| `--opamp-rail-mode` | `auto` | Op-amp rail saturation strategy |
| `--vendor` | `"Melange"` | Plugin vendor name (plugin format only) |
| `--vst3-id` | derived | Stable VST3 class ID (16 ASCII chars) |
| `--clap-id` | derived | CLAP plugin ID (reverse-DNS) |

## Customizing the Generated Plugin

After `melange compile --format plugin`:

**Safe to edit** (`src/lib.rs`):
- Plugin name, vendor, URL, category
- Parameter names, ranges, and labels
- Smoothing durations
- Add custom parameters (wet/dry, presets, GUI)

**Do not edit** (`src/circuit.rs`):
- All generated DSP code, matrices, and constants
- To update, re-run `melange compile circuit.cir --format code -o src/circuit.rs`

## Netlist Syntax

Melange uses a dialect of SPICE — standard SPICE syntax for components and models, plus audio-specific extensions (`.pot`, `.switch`, `.input_impedance`). See [spice-grammar.md](spice-grammar.md) for the full reference.

Supported devices: resistors, capacitors, inductors (including saturating with `ISAT=`), voltage/current sources, diodes (including Zener with BV/IBV), BJTs (Ebers-Moll and Gummel-Poon), JFETs (N/P channel), MOSFETs (Level 1, N/P with body effect), triode tubes (Koren model), pentode/beam tetrode tubes (5 equation families, 29 catalog models), op-amps (Boyle macromodel with GBW, rail clamping, slew rate), VCAs (THAT 2180-style), coupled inductors/transformers, VCVS, VCCS, subcircuits.

## Troubleshooting

| Problem | Likely cause | Fix |
|---------|-------------|-----|
| Plugin produces silence | Wrong input/output node names | Check with `melange nodes`; use `--input-node` / `--output-node` |
| Output is very quiet | Input level too low | Increase Input Level param in the plugin UI |
| NaN / oscillation | DC operating point failed | Check biasing network; try `--backward-euler` |
| "M exceeds MAX_M" error | Too many nonlinear devices for DK | Use `--solver nodal` |
| Compilation slow | Large circuit with nodal solver | Expected for N>30 circuits; runtime is still fast |

## Next Steps

- [Plugin Development Guide](PLUGIN_GUIDE.md) — detailed guide to customizing generated plugins
- [Netlist Writing Guide](NETLIST_GUIDE.md) — how to write netlists from schematics
- [SPICE Grammar Reference](spice-grammar.md) — complete syntax reference
- [Architecture Overview](architecture.md) — how melange works internally
