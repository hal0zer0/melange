# Oversampling (Polyphase Half-Band IIR)

## Purpose

Reduce aliasing from nonlinear waveshaping by processing at 2x or 4x the nominal
sample rate. Uses self-contained polyphase allpass half-band filters with zero
external dependencies, suitable for generated real-time code.

## Source Files

| Component | File |
|-----------|------|
| Core filter library (`HalfBandFilter`, `AllpassSection`, `HB_*SECTION` coefficients) | `crates/melange-primitives/src/oversampling.rs` |
| Codegen emission of allpass functions and process_sample wrapper | `crates/melange-solver/src/codegen/rust_emitter/dk_emitter.rs` |
| Generated state fields (`os_up_state`, `os_dn_state`, etc.) | `crates/melange-solver/templates/rust/state.rs.tera` |
| Generated `process_sample` wrapper | `crates/melange-solver/templates/rust/process_sample.rs.tera` |
| `OVERSAMPLING_FACTOR` and `INTERNAL_SAMPLE_RATE` constants | `crates/melange-solver/templates/rust/constants.rs.tera` |

## Configuration

```rust
CodegenConfig { oversampling_factor: 1 | 2 | 4, .. }
```

- Factor 1: No oversampling (default). `process_sample` is public directly.
- Factor 2: 2x with 3-section filter (~80dB rejection).
- Factor 4: Cascaded 2x stages. Outer=2-section (~60dB), inner=3-section (~80dB).

## Filter Architecture

### First-Order Allpass Section

Transfer function:
```
H(z) = (c + z^{-1}) / (1 + c * z^{-1})
```

Difference equation (2 state variables: x1, y1):
```
y[n] = c * x[n] + x[n-1] - c * y[n-1]
x1 = x[n]
y1 = y[n]
```

### Polyphase Half-Band Decomposition

The half-band filter splits into two parallel allpass chains:
- **Even chain**: coefficients at indices 0, 2, 4, ...
- **Odd chain**: coefficients at indices 1, 3, 5, ...

Each chain is a cascade of first-order allpass sections. The half-band
filter output is the average of the two chains:
```
H_halfband(z) = (A_even(z^2) + z^{-1} * A_odd(z^2)) / 2
```

This achieves a sharp transition band at half the Nyquist frequency
with minimal computational cost (only allpass sections, no multiplies
for tap weights).

## Coefficients

```
HB_2SECTION = [0.07986642623635751, 0.5453536510716122]
  Even chain: [0.0799]
  Odd chain:  [0.5454]
  Rejection: ~60dB
  State: 4 floats

HB_3SECTION = [0.036681502163648017, 0.2746317593794541, 0.7856959333713522]
  Even chain: [0.0367, 0.7857]
  Odd chain:  [0.2746]
  Rejection: ~80dB
  State: 6 floats

HB_4SECTION = [0.019287696917501716, 0.15053258922549724, 0.4985894271657593, 0.8756415228935117]
  Rejection: ~100dB, State: 8 floats

HB_5SECTION = [0.011266153671749254, 0.08836290435912563, 0.31190708239012636, 0.6621633850686569, 0.9304376538377054]
  Rejection: ~120dB, State: 10 floats
```

## 2x Processing Flow

```
process_sample(input, state) -> output:
  // Upsample: 1 input -> 2 samples at 2x rate
  (up_even, up_odd) = halfband_up(input, &state.os_up_state)

  // Process both samples through circuit at internal rate
  out_even = process_sample_inner(up_even, state)
  out_odd  = process_sample_inner(up_odd, state)

  // Downsample: 2 outputs -> 1 at original rate (per output channel)
  for each output:
    (dn_even, _) = halfband_dn(out_even, &state.os_dn_state)
    (_, dn_odd)  = halfband_dn(out_odd, &state.os_dn_state)
    result = (dn_even + dn_odd) * 0.5
```

## 4x Processing Flow (Cascaded 2x)

```
process_sample(input, state) -> output:
  // Outer upsample: 1 -> 2 at 2x (HB_2SECTION)
  (outer_even, outer_odd) = halfband_outer(input, &state.os_up_state_outer)

  // Inner upsample + process: each 2x sample -> 2 at 4x (HB_3SECTION)
  (inner_e0, inner_e1) = halfband(outer_even, &state.os_up_state)
  out_e0 = process_sample_inner(inner_e0, state)
  out_e1 = process_sample_inner(inner_e1, state)

  (inner_o0, inner_o1) = halfband(outer_odd, &state.os_up_state)
  out_o0 = process_sample_inner(inner_o0, state)
  out_o1 = process_sample_inner(inner_o1, state)

  // Inner downsample: 4 -> 2 per output
  // Outer downsample: 2 -> 1 per output
  result = (dn_even + dn_odd) * 0.5
```

## State Fields (CircuitState)

```rust
// 2x or 4x inner stage:
os_up_state: [f64; STATE_SIZE],                       // Single input chain
os_dn_state: [[f64; STATE_SIZE]; NUM_OUTPUTS],        // Per-output chains

// 4x only (outer stage):
os_up_state_outer: [f64; STATE_SIZE_OUTER],
os_dn_state_outer: [[f64; STATE_SIZE_OUTER]; NUM_OUTPUTS],
```

State sizes: `2 * num_sections` per filter instance.

## Generated Constants

```rust
const OVERSAMPLING_FACTOR: usize = 2;    // or 4
const INTERNAL_SAMPLE_RATE: f64 = 96000.0;  // sample_rate * factor
```

## Sample Rate Interaction

`set_sample_rate(sr)` computes `internal_rate = sr * OVERSAMPLING_FACTOR` and
recomputes all DK matrices (S, A_neg, K, S_NI) at the internal rate. Filter
state arrays are zeroed. DC block coefficient also uses internal rate.

## Code Generation

The codegen emits:
1. `os_allpass()` — inline first-order allpass section
2. `os_halfband()` — polyphase half-band filter (inner stage)
3. `os_halfband_outer()` — outer stage (4x only)
4. `process_sample()` — public wrapper with upsample/downsample
5. `process_sample_inner()` — private, actual circuit processing

All filter coefficients are compile-time constants. No runtime allocation.
The generated code is fully self-contained (no dependency on melange-primitives).
