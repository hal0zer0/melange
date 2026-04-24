# Signal Level Contract

## Overview

Melange uses a consistent signal level convention throughout the pipeline.

## Voltage Domain

All internal processing operates in **volts**:

| Stage | Input | Output | Range |
|-------|-------|--------|-------|
| `process_sample(v)` | Volts | Volts (DC-blocked) | ±`output_clamp_v` V clamped (default ±10 V) |
| WAV file (f32) | 1.0 = 1V | 1.0 = 1V | ±1.0 typical |
| Plugin audio buffer | ±1.0 float | ±1.0 float | DAW standard |

## DC Blocking

Generated code applies a **5Hz 1-pole high-pass filter** after the output node
extraction (disable with `--no-dc-block`):

```
y[n] = x[n] - x[n-1] + R * y[n-1]
R = 1 - 2*pi*5/sr
```

This removes DC bias voltages from tube, BJT, and op-amp circuits that would otherwise pass through as a constant offset (silence in audio, or speaker-damaging DC).

The DC block is transparent for audio-frequency signals (>20Hz) and settles within ~200ms.

## Output Scaling

After DC blocking, the output is multiplied by `OUTPUT_SCALE` (default 1.0) and clamped to
`±output_clamp_v` (default ±10 V):

```
output = (dc_blocked * OUTPUT_SCALE).clamp(-OUTPUT_CLAMP_V, OUTPUT_CLAMP_V)
```

`--output-scale` exists as a CLI flag for **volts → DAW-unit mapping only**
(e.g. `--output-scale 0.1` maps a ±10V circuit output into the ±1.0 DAW
convention). It is **not** a knob for fixing simulation errors. Per `CLAUDE.md`:
*"Never use `--output-scale` or output attenuation as a remedy for simulation
errors — it's a voltage→DAW-unit mapping, not a bug fix."* If a circuit produces
a wrong amplitude, the bug is in the solver/device-models, not in the output
scale.

`--output-clamp <V>` raises (or lowers) the post-DC-block ceiling when a circuit's
rails exceed ±10 V. The flag is ignored when DC blocking is disabled
(`--no-dc-block`) since the clamp is only emitted on the DC-blocked path. Setting
it below the natural rail voltage hard-clips the output — use only to match a real
limiter, never to work around simulation errors. Typical values:

| Circuit class | `--output-clamp` |
|---------------|------------------|
| Line-level op-amp, tube preamp, distortion pedal | `10` (default) |
| Class AB push-pull power amp at ±22 V rails | `30` |
| High-voltage guitar amp at ±300 V rails | leave default and use `--output-scale 0.033` to map to ±10 V |

## Plugin Level Controls

Generated plugins include Input Level and Output Level parameters by default
(disable with `--no-level-params`):

| Parameter | Default | Range | Purpose |
|-----------|---------|-------|---------|
| Input Level | **0 dB** | ±24 dB | DAW-to-circuit gain trim |
| Output Level | **0 dB** | ±24 dB | Circuit-to-DAW gain trim |

Both defaults are unity, matching the philosophy that the user's netlist defines
the correct circuit levels. Adjust away from 0 dB only if your DAW signal level
doesn't match the circuit's intended input range. Source: `LEVEL_PARAM_DEFAULTS`
in `tools/melange-cli/src/plugin_template.rs`.

The plugin clamps final output to ±1.0:

```
audio_out = (process_sample(input * input_gain) * output_gain).clamp(-1.0, 1.0)
```

## Diagnostics

Generated code tracks diagnostic counters in `CircuitState`:

| Field | Description |
|-------|-------------|
| `diag_peak_output` | Peak absolute output (pre-clamp) |
| `diag_clamp_count` | Times output exceeded ±`output_clamp_v` |
| `diag_nr_max_iter_count` | Times Newton-Raphson hit max iterations |
| `diag_nan_reset_count` | Times NaN triggered state reset |

The CLI prints these after `melange simulate` with warnings for clipping or
convergence problems.

## SPICE Validation

When comparing melange output against ngspice, the same DC blocking filter is applied to the ngspice reference signal before comparison, since ngspice does not have built-in DC blocking.
