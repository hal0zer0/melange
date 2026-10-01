# Getting Started with Melange

Melange compiles SPICE circuit netlists into real-time audio DSP code. This guide takes you from zero to a working audio plugin.

## Prerequisites

- **Rust 1.85+** — [rustup.rs](https://rustup.rs). Needed at run time, not only
  to install: `simulate`, `analyze` and `validate` compile the generated code
  to a native binary with `rustc` (see [The compiled-binary
  cache](#the-compiled-binary-cache)).
- **melange CLI** — install it from a checkout of this repository (the
  `--path` below is relative, so you must run it from the repo root):
  ```bash
  git clone https://github.com/hal0zer0/melange.git   # skip if you already have it
  cd melange
  cargo install --path tools/melange-cli
  ```
  This puts `melange` on your `PATH` (in `~/.cargo/bin`). Verify with
  `melange --version`. The rest of this guide can then run from any directory.

  To try it without installing, `cargo build --release -p melange-cli` and run
  `./target/release/melange` instead — the README's "Start here" does that. The
  two are the same binary.

Optional:
- **ngspice** — for validation against SPICE reference (`apt install ngspice` / `brew install ngspice`)
- **zig + cargo-zigbuild** — for macOS cross-compilation from Linux

## Quick Start: Circuit to Plugin

Compile the built-in demo circuit (ships with melange, no downloads), or use a local `.cir` file. **Generate the project *outside* the melange repo** — pick any directory that is not inside a Cargo workspace (the DAW bundler walks up to the outermost `Cargo.toml`, so a project nested in the melange checkout cannot be bundled):

```bash
mkdir -p ~/melange-plugins && cd ~/melange-plugins   # any dir outside the melange repo
melange compile passive-eq1a --format plugin -o my-eq
# or from a local file:
melange compile my-circuit.cir --format plugin -o my-fuzz
```

This generates a complete nih-plug project in `my-eq/` with:
- `src/circuit.rs` — generated DSP code (do not edit)
- `src/lib.rs` — plugin wrapper (safe to customize)
- `Cargo.toml` — ready to build
- `xtask/` — the nih-plug bundler bin (do not edit)
- `build.sh` / `README.md` — build instructions

Build the plugin:

```bash
cd my-eq
bash build.sh            # bundles CLAP + VST3 (no separate nih-plug clone needed)
```

The compiled CLAP and VST3 plugins appear in `target/bundled/`.

> If you generated the project inside the melange repo, `melange` prints a
> warning and the bundle step will fail — move the project out (`mv my-eq ~/`)
> and re-run `bash build.sh`. The raw `cargo build --release` library works
> either way.

**Testing:** Always start with your monitor volume at zero and increase gradually — circuit simulations can produce unexpected levels.

## Quick Start: A Circuit From the Library

The circuit library is a separate repository; you do not clone it. Add it once
as a named *source*, list what it has, and compile by `source:name`:

```bash
melange sources add melange-circuits https://gitlab.com/oomox-group/melange-circuits/-/raw/main
melange sources show melange-circuits            # lists its circuits, by category
melange nodes melange-circuits:pipe-shouter      # inspect one
melange compile melange-circuits:pipe-shouter --format plugin -o shouter
cd shouter && bash build.sh
```

The listing also shows each circuit's tier — `stable`, `testing` or `unstable`
(see the README's [Circuits](../README.md#circuits) section for what that means).

A folder of your own works the same way. Index it once and add it by path:

```bash
melange index ~/circuits                 # writes ~/circuits/circuits-index.json
melange sources add mine ~/circuits
melange compile mine:big-muff --format plugin -o muff
```

Without an index a source still resolves `source:name` to `<base>/<name>.cir`;
the index is what lets `sources show` list it and lets circuits move between
folders without breaking the name. Format:
[CIRCUIT_INDEX.md](CIRCUIT_INDEX.md).

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

# Quick audio test: a 1 kHz tone, 1 s long (no plugin build needed)
melange simulate clipper.cir --amplitude 3 -o test.wav

# Compile to a plugin
melange compile clipper.cir --format plugin -o my-clipper
cd my-clipper
bash build.sh   # bundles CLAP+VST3; see generated README.md for nih-plug setup
```

The drive level decides whether you hear a clipper at all. The diodes sit at
`mid`, behind a divider, so at low levels they barely conduct and the circuit
is just a 280 Hz low-pass (`Rin` + `Rload` into `Cout`). Measured at 1 kHz
and 48 kHz: THD is 0.0001 % at 0.1 V, 0.13 % at 1 V, 4.1 % at 2 V and 8 % at
3 V, where H3 is at −22 dBc. To see it yourself:

```bash
melange analyze clipper.cir --harmonics 5 --amplitude 3 --freq 1000
```

`--freq` measures one frequency; without it `analyze` sweeps `--start-freq` to
`--end-freq` (default 20 Hz to 20 kHz) at `--points-per-decade` (default 10).
It runs at 48 kHz unless `-s` says otherwise, the same default as `compile`
and `simulate`, so a bare `analyze` measures the build `compile` ships. The
CSV goes to stdout (or `-o FILE`); the summary above it goes to stderr.

`analyze` characterises the circuit's response to its own stimulus; for
instrument measurements (aliasing, loudness, IMD, decay, noise floor) use a
bench instrument.

`analyze` measures each frequency point in the steady state at that point's
own drive. It drives the circuit at the point's frequency and amplitude for at
least `--preroll-secs` (default 0.25 s), then measures again after each further
stretch of that length until two successive measurements agree within 0.1 %
(the fundamental's gain and phase, and the harmonics), for at most
`--preroll-max-secs` (default 2 s) at drive. A point that has not settled by
then is reported with a warning naming it, and the closing `Steady state:`
line says whether every point settled within the cap; `--preroll-max-secs 0`
takes one measurement with no check, and `--noise` turns the check off because
noisy windows never agree. Before the first point the circuit also runs at zero
drive (0.5 s, or 5 s when it has inductors) to move off the embedded DC
operating point. With `--harmonics N`, `thd_pct` sums H2..HN below 20 kHz (and
below Nyquist), so it is `nan` for a point at or above 10 kHz; the `hN_dbc`
columns are reported up to Nyquist, and a value below −200 dBc prints as
`-inf` (at that depth it is floating-point residue, not circuit content: the
even harmonics of this symmetric clipper read `-inf`). The last column,
`nyquist_dbc`, is the content at exactly half the sample rate: it detects a
numerical limit cycle and is not an aliasing measure (aliases fold to other
frequencies). A healthy circuit reads very low or `-inf` there.

A point whose render contains samples the solver did not solve (held, committed
unconverged, or solved on a `.linearize`d model outside its region) is refused,
naming the point and the counter, as `simulate` refuses such a render;
`--allow-nr-hold` reports it anyway.

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
| `melange sources add <name> <url-or-dir>` | Add a circuit source |
| `melange sources list` | List configured circuit sources |
| `melange sources show <name>` | List the circuits a source publishes |
| `melange index <dir>` | Write a `circuits-index.json` for a folder of circuits |
| `melange nodes circuit.cir` | Show nodes, nonlinear devices, op-amps and controls (pots, switches, a `.wiper` as one knob) |
| `melange compile circuit.cir -f plugin -o dir` | Generate plugin project |
| `melange compile circuit.cir -f code -o file.rs` | Generate standalone Rust code |
| `melange simulate circuit.cir --amplitude 0.1 -o out.wav` | Process test tone |
| `melange simulate circuit.cir --input-audio audio.wav -o out.wav` | Process audio file |
| `melange analyze circuit.cir` | Frequency response (a sine per frequency through the compiled circuit) |
| `melange validate circuit.cir` | Compare against ngspice |

By default the commands are brief: `compile`, `simulate` and `analyze` print
one line naming the solver route and integrator they chose and what they
wrote; `simulate` adds the output peak in volts and dBFS (the WAV's full scale
is 1 V); a solver counter is printed only when it is worth a warning. For the
clipper above, `simulate` prints:

```
melange simulate
  Source: clipper.cir

  Solver: nodal (schur sub-path), chosen automatically; trapezoidal integration. (-v for why)
  Solver diagnostics (-v lists every counter):
    -> nothing to flag: no iteration-ceiling hits, no resets.
  Output peak: 0.5159 V (-5.7 dBFS; the WAV's full scale is 1 V)

Output written to: test.wav
```

Add `-v` (`--verbose`, before or after the subcommand) for everything else:
build steps, matrix sizes, why that route and integrator, the Newton
iteration budget, op-amp rail-mode detail and every solver counter. Warnings
and refusals print either way.

`simulate --input-audio` builds the circuit at the WAV's own sample rate, since
the solver route and integrator are chosen per rate; an explicit
`--sample-rate` that differs from the file's rate is refused. Without
`--input-audio` it renders a 1 kHz test tone at `--sample-rate` (default
48000). It reads 16- and 24-bit PCM and 32-bit float WAVs, plain or
WAVE_FORMAT_EXTENSIBLE, and uses the first channel of a multichannel file.

### The compiled-binary cache

`simulate` and `analyze` do not interpret the circuit. They generate its Rust
code, compile it with `rustc -O` into a native binary, and run that. The binary
is kept, keyed by a hash of the generated source, so an identical re-run skips
the compile. Any change to the circuit or to an option that ends up in the
generated program (`--amplitude`, `--pot`, `--sample-rate`, the sweep range)
makes a new one. A small circuit's binary is about 4.5 MB. The cache is capped
at 2 GiB by default: after each new binary, the least recently used ones are
removed until it fits (a cache hit counts as a use). Set
`MELANGE_BINARY_CACHE_MAX_MB` to change the cap, in MiB; `0` means no limit.

It lives in the platform cache directory: `~/.cache/melange/binaries` on Linux
(`$XDG_CACHE_HOME/melange/binaries` if that is set), `~/Library/Caches/melange/binaries`
on macOS. `melange cache stats` prints the exact location, file count, size
and cap.

```bash
melange cache stats             # location, file count and size of both caches, and the cap
melange cache clear --binaries  # deletes only the compiled binaries
melange cache clear             # deletes the compiled binaries AND downloaded circuit files
```

Plain `cache clear` also empties the circuit cache, the copies of circuits
fetched from remote sources; they are downloaded again on next use. Deleting the
`binaries` directory by hand is equally safe. (`validate` compiles to a
temporary file and removes it; it does not use this cache.)

## Compile Options

Key flags for `melange compile`:

| Flag | Default | Description |
|------|---------|-------------|
| `--format code\|plugin` | `code` | Output format. `code` emits the bare `circuit.rs` — its API is in [CODE_API.md](CODE_API.md) |
| `--sample-rate` | 48000 | Design sample rate (Hz) |
| `--oversampling 1\|2\|4` | 1 | Anti-aliasing oversampling factor |
| `--input-node` | `in` | Input node name in netlist |
| `--output-node`, `-n` | `out` | Output node name(s), comma-separated. `--format plugin` takes one (mono plugin) or two (stereo, one node per channel) and refuses more; `--format code` takes any number |
| `--stereo` | off | Plugin format, one output node: a 2-in/2-out plugin running one copy of the circuit per channel (about twice the CPU; identical component values, every control moves both). Refused with `--format code`, `--mono`, or two output nodes. See [PLUGIN_GUIDE.md](PLUGIN_GUIDE.md) |
| `--solver auto\|dk\|nodal` | `auto` | Solver selection (auto picks the best) |
| `--no-dc-block` | off | Disable 5Hz DC blocking filter |
| `--no-level-params` | off | Omit Input/Output Level knobs (same as `--with-level-params=false`) |
| `--no-ear-protection` | off | Disable output soft limiter |
| `--wet-dry-mix` | off | Add wet/dry mix parameter |
| `--mono` | off | Changes nothing today: one output node builds a 1-in/1-out plugin unless `--stereo` is given, two build a 2-in/2-out plugin, and `--mono` with more than one output node (either format) or with `--stereo` is refused |
| `--noise off\|thermal\|shot\|full` | `off` | Compile in the circuit's own noise; the plugin gets a "Circuit Noise" on/off parameter (on by default). See [NOISE_GUIDE.md](NOISE_GUIDE.md) |
| `--cpu-baseline x86-64-v3\|x86-64-v2\|x86-64` | `x86-64-v3` | x86_64 instruction set for the plugin. v3 is fastest but crashes on pre-2013 CPUs; `x86-64` runs everywhere (plugin format only) |
| `--backward-euler` | off | Use backward Euler (unconditionally stable) |
| `--max-iter` | auto | The most solver iterations allowed per sample. Leave it unset: melange picks the budget per circuit. A value pins it, but a nodal-routed build refuses anything below 100, because its solver shortens its steps through a device's sharp knee (a diode turning on, a transistor saturating) and needs that many to get through it within one sample. DK has no floor. `-v` and the generated file's `Build:` header line show the budget the code runs |
| `--tube-grid-fa auto\|on\|off` | `auto` | Pentode grid-off dimension reduction: opt-in (`on`, warned, not accuracy-neutral); `auto` keeps the full 3D model |
| `--opamp-rail-mode` | `auto` | Op-amp rail saturation strategy |
| `--vendor` | `"Melange"` | Plugin vendor name (plugin format only) |
| `--vendor-url` / `--email` | melange repo / empty | Publisher URL and support contact the DAW shows |
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

Supported devices: resistors, capacitors, inductors (including saturating with `ISAT=`), voltage/current sources, diodes (including Zener with BV/IBV), BJTs (Ebers-Moll and Gummel-Poon), JFETs (N/P channel), MOSFETs (Level 1, N/P with body effect), triode tubes (Koren model), pentode/beam tetrode tubes (5 equation families, 29 catalog models), op-amps (Boyle VCCS macromodel with rail clamping and slew rate; no bandwidth pole), VCAs (THAT 2180-style), coupled inductors/transformers, VCVS, VCCS, subcircuits.

## Troubleshooting

| Problem | Likely cause | Fix |
|---------|-------------|-----|
| Plugin produces silence | Wrong input/output node names | Check with `melange nodes`; use `--input-node` / `--output-node` |
| Output is very quiet | Input level too low | Increase Input Level param in the plugin UI |
| NaN / oscillation | DC operating point failed | Check biasing network; try `--backward-euler` |
| "exceeds MAX_M" error | More than 32 nonlinear dimensions: a melange limit on generated code size, and every solver route refuses | No solver flag avoids it; see "Nonlinear System Size" in `docs/limitations.md` |
| Compilation slow | Large circuit with nodal solver | Expected for N>30 circuits; runtime is still fast |
| `Input node 'in' not found` on an oscillator | The circuit has no audio input, but every build drives one | Add a dummy `in` node and run with `--amplitude 0`; see "Circuits with no audio input" in [NETLIST_GUIDE.md](NETLIST_GUIDE.md) |

## Next Steps

- [Plugin Development Guide](PLUGIN_GUIDE.md) — detailed guide to customizing generated plugins
- [Netlist Writing Guide](NETLIST_GUIDE.md) — how to write netlists from schematics
- [SPICE Grammar Reference](spice-grammar.md) — complete syntax reference
- [Architecture Overview](architecture.md) — how melange works internally
