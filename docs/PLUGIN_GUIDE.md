# Plugin Development Guide

This guide explains how melange-generated plugin projects work and how to customize them.

## Generated Project Structure

When you run `melange compile circuit.cir --format plugin -o my-plugin`, you get:

```
my-plugin/
  Cargo.toml        # Rust package with nih-plug dependency
  src/
    circuit.rs      # Generated DSP code — DO NOT EDIT
    lib.rs          # Plugin wrapper — SAFE TO EDIT
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
- **Pot/switch methods** — `set_pot_0()`, `set_switch_0()`, etc.

If you change the netlist and want to update the DSP without losing your `lib.rs` edits:

```bash
melange compile updated-circuit.cir --format code -o src/circuit.rs
```

### src/lib.rs (Customizable Plugin Wrapper)

Contains the nih-plug boilerplate that wraps `circuit.rs`:

- **CircuitParams** — all plugin parameters (levels, pots, switches, mix)
- **CircuitPlugin** — the plugin struct (state, sample rate)
- **Plugin trait impl** — initialize, reset, process, audio layout
- **nih_export_clap! / nih_export_vst3!** — plugin format exports

## Parameter Types

The generated plugin can have up to five kinds of parameters:

### Level Parameters (default: on)

```rust
input_level: FloatParam   // -24 to +24 dB, default 0 dB
output_level: FloatParam  // -24 to +24 dB, default 0 dB
```

Applied as linear gain before/after `process_sample()`. Smoothed over 50ms to avoid clicks. Disable with `--no-level-params` if your circuit handles its own gain staging.

### Pot Parameters (from `.pot` directives)

```rust
pot_0: FloatParam   // range: min..max ohms, default: nominal
pot_1: FloatParam   // ...
```

Each `.pot` directive in the netlist becomes a `FloatParam` with:
- Range: min to max resistance (from `.pot Rname min max`)
- Default: netlist nominal (or explicit default if specified)
- Unit: Ohms
- Smoothing: 10ms linear ramp

**Update timing:**
- **All solvers**: per-block (matrix rebuild O(N^3) on value change)
- Per-sample smoothing via `.smoothed.next()` interpolates between rebuilds

### Switch Parameters (from `.switch` directives)

```rust
switch_0: IntParam   // range: 0..num_positions-1
switch_1: IntParam   // ...
```

Each `.switch` directive becomes an `IntParam`. Position 0 = first value set, position 1 = second, etc. Switches always update per-block (matrix rebuild on position change).

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

## Audio Processing Flow

The generated `process()` method follows this flow:

```
Per block:
  Read switch positions → rebuild matrices if changed
  Read pot values → rebuild matrices if changed (O(N^3))

Per sample:
  Read smoothed input_level → compute input_gain
  Read smoothed output_level → compute output_gain
  Read smoothed pot values (interpolated between rebuilds)

  For each channel (stereo: L and R independently):
    input *= input_gain                          (if level params)
    dry = input                                  (if wet/dry mix)
    output = process_sample(input, state)        (circuit DSP)
    output *= output_gain                        (if level params)
    output = mix * output + (1-mix) * dry        (if wet/dry mix)
    output = ear_protection_limit(output)        (if ear protection on)
    output = clamp to ±1.0 or NaN → 0.0         (safety)
```

**Stereo handling:** Each channel gets its own `CircuitState` — left and right are processed independently with no cross-talk. Multi-output circuits (e.g., stereo output nodes) use a single shared state.

## Customizing Parameters

### Change a Parameter's Range or Name

Find the parameter in `CircuitParams::default()`:

```rust
pot_0: FloatParam::new(
    "R1 (Tone)",         // display name — change this
    10000.0,             // default value
    FloatRange::Linear {
        min: 500.0,      // change range here
        max: 50000.0,
    },
)
.with_smoother(SmoothingStyle::Linear(10.0))  // smoothing time (ms)
.with_unit(" Ohm"),                            // display unit
```

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

When the DAW changes sample rate, the plugin calls `state.set_sample_rate(new_rate)`:

- All matrices are recomputed from G and C at the new rate
- Transient state is cleared (brief settling period ~200ms from DC block)
- Pot/switch state is preserved
- Takes ~1ms (matrix math, not allocation)

This happens automatically in the generated `initialize()` method.

## Oversampling

When compiled with `--oversampling 2` or `--oversampling 4`:

- All circuit matrices are computed at the internal rate (sample_rate * factor)
- A polyphase half-band IIR filter handles up/downsampling
- Filter is self-contained in the generated code (no runtime dependencies)
- 4x uses cascaded 2x stages (~60dB + ~80dB stopband rejection)

Oversampling helps with:
- Reducing aliasing from nonlinear devices (diodes, tubes, transistors)
- Improving NR convergence stability (smaller timesteps)

Cost: CPU scales roughly linearly with the oversampling factor.

## NaN Recovery

The generated code includes NaN guards. If `process_sample()` produces NaN or infinity:

1. The output sample is replaced with 0.0 (silence)
2. On the next sample, the circuit state is reset to the DC operating point
3. A diagnostic counter (`diag_nan_reset_count`) is incremented

This prevents a single bad sample from permanently corrupting the plugin state. If you see frequent NaN resets, the circuit likely has a biasing or convergence issue.

## Diagnostics

`CircuitState` includes diagnostic fields you can read for debugging:

| Field | Meaning |
|-------|---------|
| `diag_peak_output` | Highest absolute output voltage seen |
| `diag_clamp_count` | Times output exceeded the clamp range |
| `diag_nr_max_iter_count` | Times Newton-Raphson hit max iterations |
| `diag_nan_reset_count` | Times state was reset due to NaN/inf |
| `last_nr_iterations` | Iteration count of the most recent NR solve |

These are informational and don't affect audio output.

## Building and Testing

### Build the plugin

```bash
cd my-plugin
cargo nih-plug-xtask bundle my_plugin --release
```

Output: `target/bundled/my_plugin.clap` and `target/bundled/my_plugin.vst3`

### Plugin file locations by platform

| Platform | CLAP | VST3 |
|----------|------|------|
| macOS | `~/Library/Audio/Plug-Ins/CLAP/` | `~/Library/Audio/Plug-Ins/VST3/` |
| Linux | `~/.clap/` | `~/.vst3/` |
| Windows | `C:\Program Files\Common Files\CLAP\` | `C:\Program Files\Common Files\VST3\` |

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
| Clicks on preset/DAW load | State not reset | Check that `reset()` calls `state.reset()` |
| High CPU usage | Nodal solver with many pots | Expected — pots trigger O(N^3) rebuild. Reduce pot count or use DK-compatible circuit |
| NaN / noise burst | DC operating point wrong | Try `--backward-euler`; check circuit biasing |
| Ear protection clipping clean signal | Output exceeds 0 dBFS | Reduce Output Level, or disable ear protection for measurement |
