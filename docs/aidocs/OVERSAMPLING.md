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

## Validating an oversampled build

`--oversampling` is compile-time codegen, so a 2x build is different DSP from a
1x build of the same deck. `melange validate` therefore takes the flag too:

```
melange validate circuit.cir --oversampling {1|2|4}      # default 1
```

Without it there is no way to validate what ships — you validate the 1x code and
ship the 2x code. Plumbing: `ValidationOptions.oversampling` →
`run_melange_solver_from_str` → `CodegenConfig.oversampling_factor`, with the DK
kernel, the routing decision and the forward-active / grid-off gates all built at
`sample_rate * factor`, exactly as `compile` and `simulate` do it. `validate`
does NOT read a deck's `.oversampling` recommendation: it reports the build it
was asked to measure.

### What happens to the reference

ngspice is untouched — it has its own timestep and knows nothing about melange's
internal rate — and it is **not filtered**. The comparison is against the
circuit: an **unfiltered** reference, aligned to the melange output by ONE
best-fit constant delay (`crates/melange-validate/src/alignment.rs`). The same
alignment runs in **every mode, 1x included**, so the 1x/2x/4x rows stay
commensurable; at 1x it lands within a few thousandths of a sample of zero.

The estimator is standardised, and each constraint is load-bearing:

- **Least-squares fractional delay** over the graded window — the delay that
  minimises the residual sum of squares, evaluated with the same band-limited
  interpolator used to apply it. That is the definition of "error modulo a
  constant delay"; any other estimator leaves delay error in the residual and
  bills it as shape.
- A band-limited cross-correlation peak is the **same** estimator when done
  fractionally. Parabolic interpolation of the integer-lag peak is **biased**
  and is not used: the integer scan only picks which cycle, and a continuous
  minimisation refines inside it.
- **Delay only, never gain.** Correlation is gain-blind and a gain-error gate is
  the thing that catches scale; a fit that also scaled would absorb the very
  quantity that gate measures. Pinned by
  `fit_is_delay_only_and_leaves_gain_error_in_the_residual`.
- **Seeded and bounded.** On a periodic tone the residual is nearly periodic in
  the delay with period `1/f0` (48 samples at 1 kHz / 48 kHz) and window-edge
  effects decide between neighbouring cycles. The search is seeded at the
  ANALYTIC round-trip delay from the filter design
  (`oversampling_round_trip_group_delay_samples`: 2.6502 host samples at 1 kHz
  for 2x, 3.4682 for 4x; 0 at 1x) and bounded to ±half a stimulus period. A
  blind wide scan really does land a cycle away — measured: 50.66 samples
  instead of 2.66, exactly 48 out.
- **Reported, not implied.** An `Aligned:` line gives the fitted delay next to
  the analytic one, so a fit far from analytic is visible as the finding it is,
  and says so loudly if the fit ends on its search bound.

The interpolator is a 96-tap Kaiser (β = 9) windowed sinc, unity DC gain, exactly
a delta at integer delay. Its own worst-case (half-sample) error against the
analytic answer is 7.9e-7 at 100 Hz, 7.1e-9 at 1 kHz, 1.7e-6 at 15 kHz on a
unit tone — three orders below the residuals being reported, and pinned by
`interpolator_error_floor_is_far_below_the_measured_residual`.

**No tolerance changes with the flag.** The half-bands' frequency-dependent
phase is not compensated away; it stays in the number, because it ships.

#### The compensation that was retired (2026-09-23), and why

The first version of `--oversampling` (commit `d270a79`, same day) instead
pushed the ngspice reference through the same half-band round trip with the
circuit replaced by an identity, so both sides carried the same filters. The
flag, the magnitude-flat property and the twin-drift guard were all sound. The
comparison method was not.

The validate stimulus is a single 1 kHz sine. On a single tone an allpass is a
pure time shift, and a time-invariant circuit maps a delayed input to an
identically delayed output. So:

| | harmonic `k` carries |
|---|---|
| shipped output | `tau_up(f0) + tau_down(k*f0)` — harmonics are generated AFTER the up leg, so they never pass through it |
| round-tripped reference | `tau_up(k*f0) + tau_down(k*f0)` — ngspice's harmonics already exist, then go through BOTH legs |

The down leg cancels exactly; the up leg is billed at harmonic frequencies it
never saw. What that comparison measured was `tau_up(f0) - tau_up(k*f0)`. The
numbers were real; the attribution to the up leg was an artifact of the
compensation. **Do not repeat it.** `apply_oversampling_round_trip` still exists,
but only as the twin-drift guard's subject and the source of the analytic seed.

### What an oversampled build costs, measured

`tube_screamer_u`, 48 kHz, 0.3 V, 1 kHz tone, re-baselined 2026-09-23 against an
unfiltered delay-aligned reference. The 20 ms window is transient-dominated
(`C_out` 0.1 µF into 1 MΩ gives τ = 0.1 s; the DC blocker's τ is 32 ms), so the
500 ms rows are the ones to read:

| | 20 ms 1−ρ | 20 ms nRMS | 500 ms 1−ρ | 500 ms nRMS | fitted delay (sp) | analytic (sp) |
|---|---|---|---|---|---|---|
| 1x | 2.006e-5 | 0.6348 % | 1.00e-6 | 0.1423 % | 0.0019 / 0.0042 | 0 |
| 2x | 2.817e-5 | 0.7524 % | 5.64e-6 | 0.3361 % | 2.6585 / 2.6606 | 2.6502 |
| 4x | 1.765e-5 | 0.5971 % | 6.25e-6 | 0.3549 % | 3.4763 / 3.4782 | 3.4682 |

An oversampled build loses correlation — **5.6× at 2x and 6.3× at 4x in 1−ρ over
500 ms** — and the loss is in the shipped plugin, not in the harness. The fitted
delays land within 0.01 samples of the analytic round trip in every oversampled
mode, which is the alignment reporting exactly what the filter design predicts.

Movement from the retired method is small (2x 500 ms: 6.43e-6 → 5.64e-6; 4x:
7.17e-6 → 6.25e-6; 1x: 1.16e-6 → 1.00e-6), because the two methods happen to be
comparable in SIZE. They are not comparable in MEANING: the old numbers were
against a filtered reference and attributed to the wrong leg.

Harmonic *magnitudes* still move the other way — the oversampled builds track the
reference better, which is the finer internal timestep doing its job — while
correlation is dominated by the phase term.

### Which leg the phase comes from — measured, not reasoned

`cargo run -p melange-validate --release --example os_leg_attribution` swaps one
leg at a time for a **linear-phase FIR half-band** (Kaiser-windowed sinc, 129
taps, constant 32-host-sample group delay), so the only thing a swap removes is
that leg's *dispersion*. Its shipped/shipped variant reproduces the generated
`process_sample` bit for bit (asserted) before any swap is measured.

`tube_screamer_u`, 48 kHz, 0.3 V, 1 kHz, 2x, 500 ms, delay-aligned, 256-sample
window skip:

| build | nRMS | 1−ρ |
|---|---|---|
| 1x (floor) | 0.0599 % | 1.658e-7 |
| 2x shipped up + shipped down | 0.3077 % | 4.724e-6 |
| 2x **ideal** up + shipped down | 0.3058 % | 4.663e-6 |
| 2x shipped up + **ideal** down | 0.0712 % | 2.439e-7 |
| 2x ideal up + ideal down | 0.0629 % | 1.847e-7 |

Of the 2x excess over the 1x floor, swapping the **decimator** removes **98.3 %**
and swapping the **interpolator** removes **1.3 %**. On this stimulus the phase
residual is the DOWN leg's. That is the opposite of what the retired
compensation implied, and it is the direct consequence of the same fact that
broke it: the harmonics are generated after the up leg, so on a single tone only
the down leg ever filters them.

The per-harmonic diagnostic (fundamental-phase aligned — the wrong frame for a
gate, since it zeroes the fundamental by construction, but the right one for
reading dispersion) shows the signature cleanly. Odd-harmonic phase error of
melange against ngspice, 500 ms, integer-period window:

| harmonic | 1x | 2x shipped |
|---|---|---|
| 3rd (3 kHz) | 0.055° | −0.194° |
| 5th (5 kHz) | 0.113° | −0.954° |
| 7th (7 kHz) | 0.223° | −2.691° |
| 9th (9 kHz) | 0.421° | −5.937° |

Phase error grows with harmonic order while magnitude error does not — dispersion,
not amplitude. (Even harmonics are 1e-7-order here and their phases are noise.)

**The up leg is not inert — it is invisible to a single tone.** Test B drives two
tones and swaps only the up leg (melange against melange, no reference engine
involved):

| tone pair | change in IMD products from swapping the up leg |
|---|---|
| 1 kHz + 1.1 kHz | ≤ 0.024 dB on every product — nothing |
| 19 kHz + 20 kHz | 0.31–1.09 dB; the `f2−f1` difference tone at 1 kHz moves 0.70 dB |

At the top of the passband, where the allpass chain's phase varies fastest, the
up leg genuinely reshapes the waveform that reaches the clipper and the
intermodulation changes. A single tone cannot show this, which is why a
single-tone residual must not be attributed to it. 19 kHz + 20 kHz is the
conventional aliasing pair and the informative one here; 1 kHz + 1.1 kHz is the
musically relevant control, and it shows the up leg doing nothing measurable
where a Tube Screamer is actually played.

Also unremoved: residual imaging/aliasing, and the solver's own change of answer
from running at a finer timestep, which is a genuine improvement rather than an
artifact.

### Twin-drift guard

The primitives' round trip and the shipped build's emitted tables are twins. `crates/melange-validate/tests/oversampling_reference.rs`
pins them together: emitted `OS_COEFFS` / `OS_COEFFS_OUTER` must equal the
primitives' tables bit for bit, and the primitives' round trip must match the
GENERATED, COMPILED oversampled code to < 1e-12 per sample on a pure-gain
circuit (2x and 4x). The guard outlived the compensation it was written for: the
round trip is now the source of the ANALYTIC delay the alignment is seeded at,
and this document quotes its properties as established fact. A silent drift
between the two copies would make both wrong.
