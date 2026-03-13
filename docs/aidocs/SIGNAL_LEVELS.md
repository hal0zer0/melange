# Signal Level Contract

## Overview

Melange uses a consistent signal level convention throughout the pipeline.

## Voltage Domain

All internal processing operates in **volts**:

| Stage | Input | Output | Range |
|-------|-------|--------|-------|
| `process_sample(v)` | Volts | Volts (DC-blocked) | ±10V clamped |
| WAV file (f32) | 1.0 = 1V | 1.0 = 1V | ±1.0 typical |
| Plugin audio buffer | ±1.0 float | ±1.0 float | DAW standard |

## DC Blocking

Every solver (`CircuitSolver`, `LinearSolver`, and generated code) applies a **5Hz 1-pole high-pass filter** after the output node extraction:

```
y[n] = x[n] - x[n-1] + R * y[n-1]
R = 1 - 2*pi*5/sr
```

This removes DC bias voltages from tube, BJT, and op-amp circuits that would otherwise pass through as a constant offset (silence in audio, or speaker-damaging DC).

The DC block is transparent for audio-frequency signals (>20Hz) and settles within ~200ms.

## Output Scaling

After DC blocking, the output is multiplied by `OUTPUT_SCALE` (default 1.0) and clamped to ±10V:

```
output = (dc_blocked * OUTPUT_SCALE).clamp(-10.0, 10.0)
```

Use `--output-scale` in the CLI to adjust for circuits with unusual voltage swing.

## Plugin Level Controls

Generated plugins always include Input Level and Output Level parameters:

| Parameter | Default | Range | Purpose |
|-----------|---------|-------|---------|
| Input Level | **-12 dB** | -36 to +12 dB | Attenuates DAW signal to match circuit's expected input level |
| Output Level | 0 dB | -60 to +12 dB | Final output gain control |

The -12 dB input default maps ±1.0 DAW signal to ±250mV, matching typical guitar pickup levels (50-200mV). This prevents dangerously loud output from amplifying circuits on first load. For line-level circuits (EQs, filters), turn input up toward 0 dB.

To generate a plugin without level controls, pass `--no-level-params`.

The plugin clamps final output to ±1.0:

```
audio_out = (process_sample(input * input_gain) * output_gain).clamp(-1.0, 1.0)
```

## Diagnostics

Both runtime solvers and generated code track:

| Field | Description |
|-------|-------------|
| `diag_peak_output` | Peak absolute output (pre-clamp) |
| `diag_clamp_count` | Times output exceeded ±10V |
| `diag_nr_max_iter_count` | Times Newton-Raphson hit max iterations |
| `diag_nan_reset_count` | Times NaN triggered state reset |

The CLI prints these after simulation with warnings for clipping.

## SPICE Validation

When comparing melange output against ngspice, the same DC blocking filter is applied to the ngspice reference signal before comparison, since ngspice does not have built-in DC blocking.
