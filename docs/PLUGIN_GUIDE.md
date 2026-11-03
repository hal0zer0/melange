# Plugin Development Guide

This guide explains how melange-generated plugin projects work and how to customize them.

## Generated Project Structure

When you run `melange compile circuit.cir --format plugin -o my-plugin`, you get:

```
my-plugin/
  Cargo.toml        # Rust package with nih-plug dependency
  build.sh          # Bundles a DAW-loadable CLAP + VST3 into target/bundled/
  src/
    circuit.rs      # Generated DSP code — DO NOT EDIT
    lib.rs          # Plugin wrapper — SAFE TO EDIT
  xtask/            # nih-plug's bundler, used by build.sh (no separate checkout needed)
  .cargo/
    config.toml     # CPU baseline for the build (x86-64-v3 by default; see README.md)
  README.md         # Build instructions
  .gitignore
```

### src/circuit.rs (Generated, Read-Only)

Contains everything the circuit needs to run in real-time:

- **Pre-computed matrices** — `A`, `A_NEG`, `K`, `S_NI`, etc. (the discretized circuit)
- **Device constants** — `DEVICE_0_IS`, `DEVICE_1_BF`, etc. (per-device model parameters)
- **DC operating point** — `DC_NL_I` (bias currents for nonlinear device initialization)
- **CircuitState** — all runtime state (node voltages, previous samples, pot/switch state)
- **process_sample()** — the main DSP function, called once per sample
- **set_sample_rate()** — recomputes matrices for a new sample rate
- **Pot/switch methods** — `set_pot_0()`, `set_switch_0()`, etc. (user knobs)
- **Runtime setters** — `set_runtime_R_<field>()` for `.runtime R` resistors (audio-rate, reseed-free) and `pub <field>: f64` for `.runtime V` voltage sources

If you change the netlist and want to update the DSP without losing your `lib.rs` edits:

```bash
melange compile updated-circuit.cir --format code -o src/circuit.rs
```

`--format code` is also how you use melange without a plugin project at all: the
emitted file is self-contained and dependency-free. Its caller-facing API is
documented in **[Using the Generated DSP Directly](CODE_API.md)** — construction,
`process_sample`'s signature (it is a free function), the constants, and the
output-level contract.

### src/lib.rs (Customizable Plugin Wrapper)

Contains the nih-plug boilerplate that wraps `circuit.rs`:

- **CircuitParams** — all plugin parameters (levels, pots, switches, mix)
- **CircuitPlugin** — the plugin struct (state, sample rate)
- **Plugin trait impl** — initialize, reset, process, audio layout
- **nih_export_clap! / nih_export_vst3!** — plugin format exports

The template emits no editor: a host shows the plugin in its generic parameter
view. A GUI (nih-plug supports VIZIA and iced) is yours to add in `lib.rs`.

## Parameter Types

The generated plugin can have up to six kinds of parameters:

### Level Parameters (default: on)

```rust
input_level: FloatParam   // -24 to +24 dB, default 0 dB
output_level: FloatParam  // -24 to +24 dB, default 0 dB
```

Applied as linear gain before/after `process_sample()`. Smoothed over 50ms to avoid clicks. Disable with `--no-level-params` (or `--with-level-params=false`) if your circuit handles its own gain staging.

### Pot Parameters (from `.pot` directives)

```rust
pot_drive: FloatParam   // 0..1 (shown as 0-100%), mapped onto min..max ohms
pot_tone: FloatParam    // ...  (field named from the .pot label)
```

Each `.pot` directive in the netlist becomes a `FloatParam` with:
- Range: normalized 0..1, displayed as 0–100 %. `process()` maps it
  **linearly** onto the pot's resistance, `min + t * (max - min)` from
  `.pot Rname min max`.
- Direction: 0 % is the min resistance, 100 % the max. If more resistance
  means *less* of the effect in your circuit (a gain-setting resistor, for
  instance), the knob reads backwards. Flip it in `process()` with
  `let t = 1.0 - t;` — the generated code marks the spot.
- Taper: linear, deliberately. The knob maps straight onto the circuit-true
  resistance, which is melange's job; the feel of the knob is the plugin's.
  A linear knob is not an audio taper, so most Volume/Tone pots (log taper
  in hardware) do little over the first part of their travel. The generated
  `lib.rs` marks the one line where `t` is reshaped before it is scaled,
  e.g. `let t = t * t;` for a rough audio taper — change it there and leave
  the circuit alone.
- Default: the explicit `.pot` default if given, otherwise the resistor's
  netlist value.
- Smoothing: 10ms linear ramp

`.wiper` pots are the same shape: a 0..1 wiper position, shown as a percentage.

**Update timing:**
- The parameter smoothers advance every sample (a 50 ms ramp is 50 ms).
- The smoothed values reach the circuit every 32 samples
  (`CONTROL_APPLY_INTERVAL` in `lib.rs`, 0.67 ms at 48 kHz). A changed value
  rebuilds a nodal build's matrices (O(N^3)) and drops its chord factor, and
  on a linear DK build is one rank-1 update; applying every sample while a
  smoother runs cost more than the solve on a large nodal deck.
- Switches are applied once per block.

**Preset recall / large unsmoothed jumps.** The `set_pot_N` setter is
reseed-free — it only updates conductance and flags a rebuild; it does
not touch NR state. For the common case (smoothed knob sweeps) this is
exactly what you want: NR picks up from the previous sample's `v_prev`
with no mid-signal snap. For the rare case of a *large unsmoothed* jump
(preset recall, MIDI program change, automation step that bypasses the
smoother), follow with `recompute_dc_op()`:

```rust
state.set_pot_0(r_new);
state.recompute_dc_op();  // refresh NR seed to the new operating point
```

`recompute_dc_op()` (and its wrapper `settle_dc_op()`) exist only if the
circuit was compiled with `--emit-dc-op-recompute`; without the flag the
generated `CircuitState` has neither, and the generated `lib.rs` never calls
them. You add the call yourself.

On DK-path circuits this re-solves the operating point from the new pot
value and writes `v_prev`/`i_nl_prev` with the converged equilibrium (and
puts the trapezoidal history `q_dot` at rest). On
nodal-path circuits `recompute_dc_op()` does not re-solve; it only bumps
`diag_nr_max_iter_count`, and NR catches up on its own over roughly
`WARMUP_SAMPLES_RECOMMENDED` samples after the jump. `settle_dc_op()` runs
that warmup as a silence loop for you. Neither method is audio-thread safe.

### Switch Parameters (from `.switch` directives)

```rust
switch_0: IntParam   // range: 0..num_positions-1
switch_1: IntParam   // ...
```

Each `.switch` directive becomes an `IntParam`. Position 0 = first value set, position 1 = second, etc. Switches always update per-block (matrix rebuild on position change). A switch flip is a topology step — call `recompute_dc_op()` (needs `--emit-dc-op-recompute`) afterward on DK circuits to refresh the NR seed (nodal falls back to NR catch-up over `WARMUP_SAMPLES_RECOMMENDED` samples).

### Wet/Dry Mix (opt-in)

```rust
mix: FloatParam   // 0.0 (dry) to 1.0 (wet), default 1.0 (fully wet)
```

Enable with `--wet-dry-mix`. Blends the processed signal with the dry input. Smoothed over 10ms.

### Ear Protection (default: on)

```rust
ear_protection: BoolParam   // default: true (on)
```

A transparent soft limiter that engages near 0 dBFS. Protects speakers and hearing during development. Users can toggle it off in the plugin UI for measurement. Disable generation entirely with `--no-ear-protection`.

### Circuit Noise (only with `--noise`)

```rust
circuit_noise: BoolParam   // "Circuit Noise", id "circuit_noise", default: true (on)
```

Present only when the circuit was compiled with `--noise thermal|shot|full`,
which gives the generated `CircuitState` its runtime noise API; a plugin
compiled without `--noise` has no such parameter and is unchanged. The
switch calls `set_noise_enabled` on every circuit instance, in every channel
layout, at `initialize()`, at `reset()` and at the top of each `process()`
block, so it is block-accurate, not sample-accurate. `set_noise_enabled` only
stores a flag, so toggling is real-time safe. See [Circuit Noise](#circuit-noise) below for the
rest of the noise API.

## Audio Processing Flow

The generated `process()` method follows this flow:

```
Per block:
  Apply the Circuit Noise switch                 (if compiled with --noise)
  Read switch positions → rebuild matrices if changed
  Read pot values → rebuild matrices if changed (O(N^3))

Per sample:
  Read smoothed input_level → compute input_gain
  Read smoothed output_level → compute output_gain
  Read smoothed pot values (interpolated between rebuilds)

  input = first input channel                    (each channel's own with --stereo)
  input *= input_gain                            (if level params)
  dry = input                                    (if wet/dry mix)
  outputs = process_sample(input, state)         (circuit DSP, one call per instance)
  For each output channel (one per output node, or one per instance):
    output *= output_gain                        (if level params)
    output = mix * output + (1-mix) * dry        (if wet/dry mix)
    output = ear_protection_limit(output)        (if ear protection on)
    output = clamp to ±1.0 or NaN → 0.0         (safety)
```

**Channel layout** follows the output nodes:

- **One output node** (the usual case): a mono plugin, 1 input and 1 output,
  with one `CircuitState`. `melange compile` picks this on its own and prints
  `Auto-selecting mono (single output node). For stereo, pass two output nodes
  or --stereo.`
- **One output node with `--stereo`**: a stereo plugin, 2 inputs and 2
  outputs, running two copies of the circuit, one `CircuitState` per channel.
  Each channel's input drives its own copy, so this costs about twice the CPU
  of the mono plugin. It is one circuit duplicated, not two units: both copies
  have the same component values, `.tolerance` and `.mismatch` draws included,
  and every knob and switch moves both. Their circuit noise (with `--noise`) is
  independent, as two physical copies' would be: the left channel uses the
  `--noise-seed` (so under a fixed seed it matches the mono plugin), the right
  channel a seed derived from it. With seed 0 the clock is read at every
  initialize and reset, and each channel's seed is derived from that read, so
  the channels get different seeds.
- **Two output nodes** (`--output-node a,b`): a stereo plugin, 2 inputs and
  2 outputs, still one shared `CircuitState`. The circuit is driven from the
  left input; the right input is ignored. Node `a` goes to the left output and
  node `b` to the right.
- **Three or more output nodes**: refused before anything is written, since a
  plugin has no channel for the nodes past the second. Pick two with `-n a,b`,
  or use `--format code`, whose `process_sample` returns every output.

`--stereo` is refused with `--format code` (generated code has no channels:
create one `CircuitState` per channel yourself, giving each its own nonzero
`set_seed` if the circuit has noise), with `--mono`, and with two or more
output nodes, which already make a stereo plugin. Each refusal names the
alternative.

`--mono` changes nothing today: a one-output circuit is already mono unless
`--stereo` is given, and with more than one output node `--mono` is refused
rather than drop a node.

## Customizing Parameters

### Change a Parameter's Range or Name

Find the parameter in `CircuitParams::default()`:

```rust
pot_tone: FloatParam::new(
    "Tone",              // display name — change this
    0.1934,              // default position, 0..1
    FloatRange::Linear {
        min: 0.0,
        max: 1.0,
    },
)
.with_smoother(SmoothingStyle::Linear(10.0))  // smoothing time (ms)
.with_unit("%")                               // display unit
```

The resistance range itself lives in `process()`, where the 0..1 value is
scaled onto the `.pot` min..max — change the circuit's range in the netlist,
not here.

### Add a Custom Parameter

1. Add a field to `CircuitParams`:
   ```rust
   #[id = "my_param"]
   pub my_param: FloatParam,
   ```

2. Initialize in `Default`:
   ```rust
   my_param: FloatParam::new(
       "My Param",
       0.5,
       FloatRange::Linear { min: 0.0, max: 1.0 },
   )
   .with_smoother(SmoothingStyle::Linear(50.0)),
   ```

3. Use in the process loop:
   ```rust
   let my_val = self.params.my_param.smoothed.next();
   ```

### Change Plugin Metadata

Near the bottom of `lib.rs`, find the `Plugin` trait impl:

```rust
const NAME: &'static str = "My Plugin Name";
const VENDOR: &'static str = "Your Name";
const URL: &'static str = "https://yoursite.com";
```

And the CLAP/VST3 IDs (these should be unique per plugin):

```rust
const CLAP_ID: &'static str = "com.melange.my-circuit";
```

## Sample Rate Changes

When the DAW changes sample rate, the plugin calls `state.set_sample_rate(new_rate)`
from the generated `initialize()` method, off the audio thread.

`set_sample_rate()` rebuilds every rate-dependent matrix from the stored G and C
at the new rate and re-seeds the filter state that was built at the old one
(oversampler, DC blocker); pot and switch values are kept. The plugin then runs
correctly at the new host rate. Three things it cannot change, because they are
decided at compile time against the compile rate and are not re-evaluated: the
solver route (both routing stages, and sub-sample-fire activation), the
integrator verdict (the ring predicate judges trapezoidal against backward Euler
at the compiled rate; a host rate above it makes stiff rings decay faster, so
the verdict is conservative there, and below it slightly optimistic), and the
oversampling factor (unless the build declares a runtime set, `.oversampling N
allow=...`, which switches between factors it built). A 48 kHz build running at
96 kHz is correct, with the integrator chosen for 48 kHz. Compile per host rate
for the optimal build.

## Oversampling

When compiled with `--oversampling 2` or `--oversampling 4`:

- All circuit matrices are computed at the internal rate (sample_rate * factor)
- A polyphase half-band IIR filter handles up/downsampling
- Filter is self-contained in the generated code (no runtime dependencies)
- 4x uses cascaded 2x stages (~60dB + ~80dB stopband rejection)

Oversampling helps with:
- Reducing aliasing from nonlinear devices (diodes, tubes, transistors)
- Improving NR convergence stability (smaller timesteps)

Cost: CPU scales roughly linearly with the oversampling factor, and the
half-band filters add latency, measured as the extra delay over a 1× build of
the same circuit, in host samples:

| Factor | Added latency | At 48 kHz |
|---|---|---|
| 2× | 2.65 samples | 55 µs |
| 4× | 3.47 samples | 72 µs |

The generated plugin reports this to the host for delay compensation, rounded
to whole samples: 3 at both 2× and 4×. With `--wet-dry-mix` the dry path is
delayed by the same 3 samples. [OVERSAMPLING.md](OVERSAMPLING.md) has the
measurement and the phase cost that comes with it.

**A runtime set becomes a parameter.** When the circuit was built with a
runtime oversampling set (`--oversampling-set 1,2,4`, or the deck's
`.oversampling N allow=...`), the plugin gets an **Oversampling** parameter
over those factors, defaulting to the build's factor. A change does not run on
the audio thread: `set_oversampling` constructs a fresh state and allocates,
so the plugin asks the host's background thread for states at the new factor
(`OsTask::Build`), the first block that finds them ready swaps them in with a
non-blocking try-lock, re-reports the latency of the new factor to the host,
and hands the old states back to the background thread to drop
(`OsTask::Drop`). The fresh states start at their construction controls and
pick up the current switch and pot values in that same block; `initialize()`
builds straight at the parameter's factor. Output is silent for nothing: the
old states keep running until the swap.

## Circuit Noise

If the circuit was compiled with `--noise {thermal|shot|full}`, the generated
plugin gets the **Circuit Noise** on/off parameter described
[above](#circuit-noise-only-with---noise), on by default, and `CircuitState`
gains a runtime noise API. These methods exist **only** in noise-enabled builds
(and each is emitted only when its mechanism is present in the circuit):

```rust
state.set_noise_enabled(true);    // master on/off
state.set_noise_gain(1.0);        // master scalar over all mechanisms
state.set_thermal_gain(1.0);      // resistor hiss alone
state.set_shot_gain(1.0);         // junction shot (also gates pentode partition)
state.set_flicker_gain(1.0);      // all 1/f (junction + resistor together)
state.set_opamp_input_gain(1.0);  // op-amp en/in (present only with EN/IN models)
state.set_temperature_k(290.0);   // thermal temperature, Kelvin (default 290)
state.set_seed(42);               // deterministic re-seed
```

Wire a "noise amount" knob to `set_noise_gain`, a Temperature knob to
`set_temperature_k`, per-mechanism trims to the `set_*_gain` setters, and a
reproducibility control to `set_seed`. Treat `set_seed` and `set_temperature_k`
as parameter-change callbacks (they recompute per-source coefficients); the
gain scalars and `set_noise_enabled` are per-block-safe.

Melange noise is physically scaled, so bare sources are genuinely quiet
(a Zener junction is sub-microvolt at its terminal). Land the floor at your
target level with a post-`process_sample` output trim in `lib.rs` — that is
voltage→dBFS mapping, the plugin's job, not a fix for the netlist. See the
[Circuit Noise Guide](NOISE_GUIDE.md) for the full API, level calibration, and
per-device coverage.

## NaN Recovery

The generated code includes NaN guards. If `process_sample()` produces NaN or infinity:

1. The output sample is replaced with 0.0 (silence)
2. On the next sample, the circuit state is reset to the DC operating point
3. A diagnostic counter (`diag_nan_reset_count`) is incremented

This prevents a single bad sample from permanently corrupting the plugin state. If you see frequent NaN resets, the circuit likely has a biasing or convergence issue.

## What the plugin does when a sample does not solve

A generated plugin does not crash and does not pass NaN to the host. The
wrapper's output guard replaces a non-finite output sample with 0.0, and the
generated code resets its state to the DC operating point on the next sample
and counts it in `diag_nan_reset_count` (above).

A sample that no Newton path solves is a different case. `process_sample`
always returns a number: on such a sample the generated code commits the
previous state (or, on a DK build, the unconverged iterate), which is bounded
and smooth, so the output looks and sounds healthy while it is not a solution
of the circuit. The CLI refuses a `simulate`, `validate` or `analyze` result
that contains one; a plugin has no such gate. The one counter that sees it is
`diag_unsolved_sample_count`, present on every generated build, cleared by
`reset()`, free to read on the audio thread. The bare template does not read
it — a product wrapper should: poll it once per block and surface whether it
is zero, which is the whole check. [Using the Generated DSP
Directly](CODE_API.md#did-the-solver-actually-solve-it) explains the counter
and its per-mechanism breakdown.

A plugin that takes the host down with "Illegal instruction" on another
machine is not a solver failure; it is the CPU baseline, covered under
[CPU baseline](#cpu-baseline--read-this-before-you-distribute) below.

## Diagnostics

`CircuitState` includes diagnostic fields you can read for debugging:

| Field | Meaning |
|-------|---------|
| `diag_peak_output` | Highest absolute output voltage seen |
| `diag_clamp_count` | Times output exceeded the clamp range |
| `diag_nr_max_iter_count` | Times Newton-Raphson hit max iterations |
| `diag_nan_reset_count` | Times state was reset due to NaN/inf |
| `diag_unsolved_sample_count` | Samples no Newton path solved (see above); zero is the only reassuring value |
| `last_nr_iterations` | Iteration count of the most recent NR solve |

These are informational and don't affect audio output.

## Building and Testing

### Build the plugin

```bash
cd my-plugin
bash build.sh   # bundles CLAP+VST3 via the in-workspace xtask bin
```

`build.sh` runs `cargo xtask bundle <name> --release`. The project ships its
own `xtask/` member crate (a thin bin wrapping `nih_plug_xtask`), which pulls
nih-plug in as a git dependency — **no separate nih-plug checkout is needed**.
Bundle from OUTSIDE any enclosing Cargo workspace: `nih_plug_xtask` finds the
workspace root by walking to the outermost `Cargo.toml`, so a project generated
inside another repo must be moved out before it can be bundled (`cargo build
--release` for the raw library works in place).

Output: `target/bundled/my_plugin.clap` and `target/bundled/my_plugin.vst3`

The project's `.cargo/config.toml` carries sections for the x86_64 Linux,
macOS and Windows (`-msvc` and `-gnu`) targets. The maintainer builds and
tests on Linux, and builds macOS bundles from Linux as described below;
Windows builds are not tested by the maintainer.

### Plugin file locations by platform

| Platform | CLAP | VST3 |
|----------|------|------|
| macOS | `~/Library/Audio/Plug-Ins/CLAP/` | `~/Library/Audio/Plug-Ins/VST3/` |
| Linux | `~/.clap/` | `~/.vst3/` |
| Windows | `C:\Program Files\Common Files\CLAP\` | `C:\Program Files\Common Files\VST3\` |

### CPU baseline — read this before you distribute

Choose it with `--cpu-baseline` on `melange compile --format plugin`:

| `--cpu-baseline` | Needs | Runs on |
|---|---|---|
| `x86-64-v3` (default) | AVX2 + FMA + BMI | Intel Haswell (2013) / AMD Excavator (2015) and newer |
| `x86-64-v2` | SSE4.2 + POPCNT | Intel Nehalem (2008) / AMD Bulldozer (2011) and newer |
| `x86-64` | SSE2 | every x86_64 CPU |

By default the generated project ships a `.cargo/config.toml` that raises the
x86-64 instruction-set baseline to **`x86-64-v3`** for every x86_64 target it
knows about:

```toml
[target.x86_64-unknown-linux-gnu]
rustflags = ["-C", "target-cpu=x86-64-v3"]
```

…with identical sections for `x86_64-apple-darwin`, `x86_64-pc-windows-msvc`,
and `x86_64-pc-windows-gnu`. `aarch64` gets no flag: ARMv8-A already mandates
NEON and fused multiply-add, so there is no uplift to unlock and naming a
specific CPU would only narrow the set of machines the bundle runs on.

**What that means for a plugin you hand to someone else.** A bundle built at
this baseline will fail with `SIGILL` / "Illegal instruction" on any x86_64 CPU
older than Intel Haswell (2013) or AMD Excavator (2015). It does not degrade and
it does not warn — it takes the host DAW down with it, on their machine, in a
way that looks like your plugin is broken. Perfectly fine for a plugin you build
for your own rig; a decision you should make deliberately for one you ship.

To build for older machines, generate the project into a new directory with
`--cpu-baseline x86-64` (or `x86-64-v2`; `compile` will not overwrite an
existing project), or delete the `[target.…]` section matching the target you
are building. The generated DSP is **bit-for-bit identical** either
way — Rust never contracts `a*b + c` into an FMA on its own, so the flag changes
instruction selection, not results. What you give up is throughput: the uplift
is documented at roughly 5–13% on matvec-heavy circuits in
`tools/melange-cli/src/plugin_template.rs`.

One further trap, which the emitted file calls out itself: a `RUSTFLAGS`
environment variable **replaces** these `rustflags` rather than appending to
them (documented Cargo behaviour). If you need extra flags, add them to the
relevant `[target.…]` section rather than exporting `RUSTFLAGS`, or you will
silently drop the baseline and not find out until you benchmark.

### Cross-compile for macOS from Linux

```bash
cargo zigbuild --release --target universal2-apple-darwin
rcodesign sign target/universal2-apple-darwin/release/libmy_plugin.dylib
```

Requires zig 0.13, cargo-zigbuild, macOS SDK 13.3, and rcodesign.

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| No output / silence | Wrong input/output node names | Check `melange nodes circuit.cir` |
| Very quiet output | Level params at default (0 dB) but circuit expects hot input | Increase Input Level in the plugin UI |
| Clicks on parameter changes | Smoothing too short | Increase `.with_smoother(SmoothingStyle::Linear(50.0))` |
| Clicks on preset/DAW load | NR seed stale after large unsmoothed jump | Compile with `--emit-dc-op-recompute`, then call `state.recompute_dc_op()` right after `set_pot_*` / `set_switch_*` (DK path). On nodal circuits the runtime stub just bumps a diagnostic counter — let NR catch up over ~`WARMUP_SAMPLES_RECOMMENDED` samples |
| High CPU usage | Nodal solver with many pots | Expected — pots trigger O(N^3) rebuild. Reduce pot count or use DK-compatible circuit |
| NaN / noise burst | DC operating point wrong | Try `--backward-euler`; check circuit biasing |
| Ear protection clipping clean signal | Output exceeds 0 dBFS | Reduce Output Level, or disable ear protection for measurement |
