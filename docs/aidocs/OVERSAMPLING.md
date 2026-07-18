# Oversampling (Polyphase Half-Band IIR)

## Purpose

Reduce aliasing from nonlinear waveshaping by processing at 2x or 4x the nominal
sample rate. Uses self-contained polyphase allpass half-band filters with zero
external dependencies, suitable for generated real-time code.

> **History (2026-07):** the original implementation had two independent bugs —
> the decimator clocked both allpass chains twice per output sample (collapsing
> the branch cells to first-order in the internal-rate z: measured worst-case
> stopband −1.0 dB, i.e. NO alias rejection, plus ~0.4–0.5 dB passband droop
> per stage), and the coefficient tables were invalid designs (−16/−20 dB
> worst-case even under correct topology, despite "60/80 dB" doc claims). The
> 4x cascade also had its stage strengths backwards (cheap filter guarding base
> Nyquist). All three fixed together; every shipped 2x/4x plugin changes HF
> response as a result (flat passband, real rejection).

## Source Files

| Component | File |
|-----------|------|
| Core filter library (`HalfBandFilter`, `AllpassSection`, `coefficients::HB_*`) | `crates/melange-primitives/src/oversampling.rs` |
| Codegen coefficient tables + `OversamplingInfo` (state sizes) | `crates/melange-solver/src/codegen/rust_emitter/helpers.rs` |
| Codegen emission of allpass/halfband functions and process_sample wrapper | `crates/melange-solver/src/codegen/rust_emitter/dk_emitter.rs` (shared by the nodal emitter) |
| Generated state fields (`os_up_state`, `os_dn_state`, etc.) | `crates/melange-solver/templates/rust/state.rs.tera` |
| Generated `process_sample` wrapper | `crates/melange-solver/templates/rust/process_sample.rs.tera` |
| `OVERSAMPLING_FACTOR` and `INTERNAL_SAMPLE_RATE` constants | `crates/melange-solver/templates/rust/constants.rs.tera` |

**Twin-drift hazard:** the coefficient tables and the up/down topology exist in
BOTH melange-primitives and the codegen emitter. They must stay semantically
identical. Measurement tests pin both sides (see Testing below).

## Configuration

```rust
CodegenConfig { oversampling_factor: 1 | 2 | 4, .. }
```

- Factor 1: No oversampling (default). `process_sample` is public directly.
- Factor 2: 2x with the steep 7-section filter (−86.9 dB stopband).
- Factor 4: Cascaded 2x stages. **Outer (base-Nyquist boundary) = steep
  7-section (−86.9 dB); inner (4x-rate) = wide 3-section (−95.1 dB over its
  design band).** The steep filter must sit at the outer stage — it is the one
  protecting the audio band.

## Filter Architecture

### Polyphase Half-Band Decomposition

The half-band filter at the internal rate is

```
H(z) = (A0(z^2) + z^-1 * A1(z^2)) / 2
```

where A0 is the cascade of the even-indexed coefficients' allpass cells and A1
the odd-indexed ones (hiir convention). Each branch cell realizes
`(c + z^-2) / (1 + c*z^-2)` at the internal rate; in the polyphase realization
each cell is implemented as a first-order allpass **in the branch's own clock**:

```
y[n] = c*x[n] + x[n-1] - c*y[n-1]      (2 state floats per cell: x1, y1)
```

**Each branch is clocked exactly once per LOW-rate sample.** Clocking a branch
twice per output sample (the pre-2026-07 decimator) turns the cells into
first-order allpasses in the internal-rate z and destroys the stopband.

### Interpolator (1 → 2, per low-rate input sample)

```
(out[2n], out[2n+1]) = (A_even(x[n]), A_odd(x[n]))
```

Both branches consume the same input sample, once each. Passband gain is 1
(the implicit 2x gain of zero-stuffing is absorbed by using the branches
directly, no averaging).

### Decimator (2 → 1, per low-rate output sample)

```
y[n] = (A_even(x[2n+1]) + A_odd(x[2n])) / 2
```

The even branch consumes the LATER internal sample of the pair, the odd branch
the EARLIER one (hiir `Downsampler2x::process_sample` convention; equivalent to
the textbook even/odd-phase split shifted by one internal sample). One call per
output sample.

A 2x round trip (up → identity → down) composes to the pure allpass
`A0(z^2)*A1(z^2)` — magnitude-flat by construction.

## Coefficients

All sets are generated with the published hiir designer
`PolyphaseIir2Designer::compute_coefs_spec_order_tbw(n, tbw)` (Laurent de
Soras, 2005, WTFPL; <http://ldesoras.free.fr/prod.html#src_hiir>, mirrored at
<https://github.com/unevens/hiir>), based on Valenzuela & Constantinides,
"Digital Signal Processing Schemes for Efficient Interpolation and Decimation",
IEE Proceedings, Dec 1983. `tbw` is the transition bandwidth normalized to the
filter's running rate: passband edge = (0.5−tbw)/2, stopband edge =
(0.5+tbw)/2 of that rate.

Rejection numbers below are worst-case stopband magnitudes verified two ways:
analytically (direct evaluation of |H|) and by tone-through-simulation in the
measurement tests. Both match the designer's `compute_atten` prediction.

| Set | n | tbw | Worst stopband | Passband (2x stage @44.1k host) | State floats |
|-----|---|-----|----------------|--------------------------------|--------------|
| `HB_STEEP_5SECTION` | 5 | 0.04 | −62.1 dB | 0–20.3 kHz | 10 |
| `HB_STEEP_7SECTION` | 7 | 0.04 | **−86.9 dB** | 0–20.3 kHz | 14 |
| `HB_STEEP_9SECTION` | 9 | 0.04 | −111.7 dB | 0–20.3 kHz | 18 |
| `HB_WIDE_3SECTION` | 3 | 0.27 | −95.1 dB (design band) | inner 4x stage only | 6 |

- Codegen ships `HB_STEEP_7SECTION` (2x default + 4x outer) and
  `HB_WIDE_3SECTION` (4x inner). The 5/9-section sets are primitives-only
  quality tiers (`Oversampler2xFast` / `Oversampler2xQuality`).
- `HB_WIDE_3SECTION`'s tbw follows the hiir cascade rule
  `TBW[stage] = (TBW[stage-1] + 0.5) / 2` = (0.04+0.5)/2 = 0.27: the inner
  stage only needs to protect the spectrum the steep outer stage keeps.
  Its wide-band figure near the 4x fold is what matters; it is NOT a
  base-Nyquist filter and must never be placed at the outer stage.
- Passband droop of all sets at the passband edge is < 0.001 dB (elliptic
  allpass-sum designs are equiripple-flat in the passband).

## 2x Processing Flow (generated code)

```
process_sample(input, state) -> [f64; NUM_OUTPUTS]:
  // Upsample: one interpolator step -> (out[2n], out[2n+1])
  (up_even, up_odd) = os_halfband(input, OS_COEFFS, &mut state.os_up_state)

  // Process both samples through circuit at internal rate (up_even first)
  out_even = process_sample_inner(up_even, state)
  out_odd  = process_sample_inner(up_odd, state)

  // Downsample: ONE decimator step per output channel
  for each output:
    result = os_halfband_down(out_even, out_odd, OS_COEFFS, &mut state.os_dn_state[out_idx])
```

## 4x Processing Flow (Cascaded 2x)

```
process_sample(input, state) -> [f64; NUM_OUTPUTS]:
  // Outer upsample (STEEP 7-section): 1 -> 2 at 2x rate
  (outer_even, outer_odd) = os_halfband_outer(input, OS_COEFFS_OUTER, os_up_state_outer)

  // Inner upsample (WIDE 3-section) + process: each 2x sample -> 2 at 4x
  (e0, o0) = os_halfband(outer_even, OS_COEFFS, os_up_state)
  proc_e0 = process_sample_inner(e0);  proc_o0 = process_sample_inner(o0)
  inner_out0 = os_halfband_down(proc_e0, proc_o0, OS_COEFFS, os_dn_state)   // per output

  (e1, o1) = os_halfband(outer_odd, OS_COEFFS, os_up_state)
  proc_e1 = process_sample_inner(e1);  proc_o1 = process_sample_inner(o1)
  inner_out1 = os_halfband_down(proc_e1, proc_o1, OS_COEFFS, os_dn_state)   // per output

  // Outer decimator (STEEP): 2 samples at 2x rate -> 1 at host rate
  result = os_halfband_down_outer(inner_out0, inner_out1, OS_COEFFS_OUTER, os_dn_state_outer)
```

## State Fields (CircuitState)

```rust
// 2x: STATE_SIZE = 14 (steep 7-section)
// 4x inner stage: STATE_SIZE = 6 (wide 3-section)
os_up_state: [f64; STATE_SIZE],                       // Single input chain
os_dn_state: [[f64; STATE_SIZE]; NUM_OUTPUTS],        // Per-output chains

// 4x only (outer stage, steep 7-section): STATE_SIZE_OUTER = 14
os_up_state_outer: [f64; STATE_SIZE_OUTER],
os_dn_state_outer: [[f64; STATE_SIZE_OUTER]; NUM_OUTPUTS],
```

State sizes: `2 * num_sections` per filter instance, plumbed from
`OversamplingInfo { state_size, state_size_outer }` in `helpers.rs` into
`state.rs.tera` / `process_sample.rs.tera` (`os_state_size`,
`os_state_size_outer` template vars). State layout per filter: even-chain
cells first (2 floats each), then odd-chain cells. Upsampler and downsampler
have SEPARATE state arrays.

## Generated Constants

```rust
const OVERSAMPLING_FACTOR: usize = 2;    // or 4
const INTERNAL_SAMPLE_RATE: f64 = 88200.0;  // sample_rate * factor
```

## Sample Rate Interaction

`set_sample_rate(sr)` computes `internal_rate = sr * OVERSAMPLING_FACTOR` and
recomputes all DK matrices (S, A_neg, K, S_NI) at the internal rate. Filter
state arrays are zeroed. DC block coefficient also uses internal rate.

The coefficient tables are rate-independent (half-band designs are normalized
to the running rate); at higher host rates the passband edge scales up with
the host rate (e.g. 0.23 × 96 kHz = 22.1 kHz audio passband at 48 kHz host 2x).

## Code Generation

The codegen emits:
1. `os_allpass()` — inline first-order allpass cell (slice + base offset)
2. `os_halfband()` — polyphase interpolator step (inner/2x stage)
3. `os_halfband_down()` — polyphase decimator step (inner/2x stage)
4. `os_halfband_outer()` / `os_halfband_down_outer()` — outer stage (4x only)
5. `process_sample()` — public wrapper with upsample/downsample
6. `process_sample_inner()` — private, actual circuit processing

All filter coefficients are compile-time constants (`{:.17e}`). No runtime
allocation. The generated code is fully self-contained (no dependency on
melange-primitives). `emit_oversampler` lives in `dk_emitter.rs` and is reused
by the nodal emitter — one fix covers both solver paths.

## Testing

Measurement tests (added 2026-07; these would have caught both historical
bugs — the old suite only tested DC settling):

- `melange-primitives` `oversampling::tests`:
  - `decimator_2x_alias_rejection` (+ fast/quality variants): internal tone at
    0.9π through the decimation path; folded alias must be < −75 dB
    (measured −88.7 dB for the 7-section set).
  - `decimator_2x_passband_flatness`: 0.25π internal tone within ±0.1 dB
    (measured 0.00000 dB).
  - `round_trip_2x_droop` / `round_trip_4x_droop`: up → identity → down at
    0.45 × base Nyquist within ±0.2 dB (measured 0.00000 dB).
  - `decimator_4x_alias_rejection_inner_band` / `_outer_band`: tones in each
    cascade stage's stopband (measured −113.3 / −87.0 dB).
- `melange-solver` `codegen_verification_tests`:
  - `test_oversampling_polyphase_structure_and_stage_assignment`: emitted
    structure + 4x stage-strength ordering.
  - `test_oversampling_2x_emitted_filters_measured`: compiles and RUNS the
    generated 2x code; measures the emitted decimator (alias −88.7 dB,
    passband 0.0 dB) and interpolator (image −88.7 dB) — pinning emitted
    code to the primitives' numbers.
