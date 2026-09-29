# Using the generated DSP directly (`--format code`)

`--format code` is the **default** output of `melange compile`. It emits one
self-contained Rust file — no crate, no wrapper, no `Cargo.toml`:

```bash
melange compile my-circuit.cir -o src/circuit.rs                 # --format code is the default
melange compile passive-eq1a --format code -o src/circuit.rs     # explicit, built-in demo circuit
```

That file has **no dependencies**. Drop it into any crate as a module — the
consumer crate used to verify this page has an empty `[dependencies]` and a
`Cargo.lock` containing exactly one package (itself). Nothing links back to
melange at runtime; once the file is emitted you can delete the compiler.

If you want the whole nih-plug project instead (plugin wrapper, parameters,
build script), that is `--format plugin`, documented in the
[Plugin Development Guide](PLUGIN_GUIDE.md). This page is only about the
generated file itself, which is the same file in both cases.

## Ten lines that run

```rust
// src/main.rs, next to the generated src/circuit.rs
mod circuit;

fn main() {
    let mut state = circuit::CircuitState::default(); // there is no ::new()
    state.set_sample_rate(48_000.0);                  // once, off the audio thread
    state.set_pot_0(5_000.0);                         // knobs: per block, not per sample
                                                      // (pot setters exist only if the
                                                      // netlist declares `.pot`s)

    let mut peak = 0.0f64;
    for i in 0..48_000 {
        let x = 0.1 * (i as f64 * 1_000.0 * std::f64::consts::TAU / 48_000.0).sin();
        // FREE FUNCTION, not a method. Returns one VOLTAGE per output node.
        let y = circuit::process_sample(x, &mut state)[0];
        peak = peak.max(y.abs());
    }
    println!("peak {peak:.4} V out of {} output(s)", circuit::NUM_OUTPUTS);
}
```

```toml
# Cargo.toml — this is the whole thing
[package]
name = "standalone"
version = "0.1.0"
edition = "2021"

[dependencies]
```

The two shapes that surprise people: `CircuitState` is constructed through
`Default` (an associated `new()` does not exist), and `process_sample` is a free
function that takes the state by `&mut`, not a method on it.

## The API

Emitted for every circuit:

| Item | Signature | Notes |
|------|-----------|-------|
| `CircuitState::default()` | `fn() -> CircuitState` | The constructor. Seeds the baked DC operating point. |
| `process_sample` | `fn(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS]` | Free function. One call = one host sample (it runs `OVERSAMPLING_FACTOR` internal steps itself). Allocation-free and lock-free. |
| `CircuitState::set_sample_rate` | `fn(&mut self, sample_rate: f64)` | Rebuilds every rate-dependent matrix. Call off the audio thread; non-finite or non-positive rates are ignored. |
| `CircuitState::reset` | `fn(&mut self)` | Back to the DC operating point, history cleared. |
| `CircuitState::dc_op` | `fn(&self) -> &[f64; N]` | The DC operating point baked at codegen time. |

Constants a caller actually needs:

| Constant | Type | Meaning |
|----------|------|---------|
| `SAMPLE_RATE` | `f64` | Rate the matrices were generated at. Any other rate needs `set_sample_rate`. |
| `NUM_OUTPUTS` | `usize` | Length of the array `process_sample` returns. |
| `OUTPUT_NODES` / `OUTPUT_SCALES` | `[usize; NUM_OUTPUTS]` / `[f64; NUM_OUTPUTS]` | Which node each output reads, and the scale already applied to it. |
| `INPUT_NODE` / `INPUT_RESISTANCE` | `usize` / `f64` | Where the input is injected, and its Thevenin source resistance in ohms. |
| `OVERSAMPLING_FACTOR` | `usize` | Internal steps per `process_sample` call. |
| `WARMUP_SAMPLES_RECOMMENDED` | `usize` | Silent samples to push through before real audio (see below). |
| `N`, `M` | `usize` | System size and nonlinear dimension — diagnostics, not something you drive. |
| `DC_OP_CONVERGED` | `bool` | False means the codegen-time DC solve did not converge; warm up longer and treat the bias point with suspicion. |

Emitted only when the netlist asks for them:

- `set_pot_<i>(&mut self, resistance_ohms: f64)` — one per `.pot`, indexed in
  netlist declaration order, clamped to the declared range. A `.wiper` emits
  **two** pots (the two halves of the track), so a single physical knob is two
  setters whose values you keep summing to the track total.
- `set_switch_<i>(&mut self, position: usize)` — one per `.switch`, 0-indexed
  positions, out-of-range values ignored. `SWITCH_LABELS: [&str; _]` carries the
  names from the netlist. There is no equivalent `POT_LABELS`: run
  `melange nodes <circuit>`, which lists the pots with their names and ranges in
  the same order as the setter indices.
- `set_runtime_R_<field>` / `set_runtime_<field>` — `.runtime` controls.
- `warmup(&mut self)`, `dc_op_by_name(name: &str) -> Option<f64>`,
  `recompute_dc_op(&mut self)` — present on some routes, absent on others.
  Check the file before calling.

`CircuitState`'s fields are `pub`. Two of them are the integrator's history:

- `v_prev: [f64; N]` — the last committed node voltages (and augmented
  branch values). Safe to read per sample.
- `q_dot: [f64; N]` — present on trapezoidal builds only: the matching
  charge derivative `C·dv/dt`, i.e. the capacitor currents at `v_prev`
  (`dΦ/dt` on inductor branch rows). Committed together with `v_prev`.

Writing them is not a control path. The trapezoidal history is
`alpha·C·v_prev + q_dot` (the charge form, see
[COMPANION_MODELS.md](aidocs/COMPANION_MODELS.md)), so writing `v_prev` alone
barely moves the next sample on capacitor-free and small-capacitance nodes and
leaves the two inconsistent. If you must set state by hand, set `q_dot`
consistently with it (from KCL), or use `reset()` or
`set_dc_operating_point(v)` — the latter puts `q_dot` at rest, which is right
when `v` is an equilibrium.

Pot and switch setters are cheap themselves but mark the matrices dirty; the
rebuild (O(N³)) happens inside the next `process_sample`. Drive them per block,
never per sample.

`process_sample` has two variants, both visible at the top of the generated
file — grep it rather than assuming:

- decks compiled with several comma-separated input nodes (`-i in_l,in_r`,
  linear `M = 0` circuits only) take `inputs: [f64; NUM_INPUTS]` instead of a
  single `f64`, alongside `NUM_INPUTS` / `INPUT_NODES` / `INPUT_RESISTANCES`;
- `.inject` / `.tap` decks take an extra injection argument and return a tuple.

## Did the solver actually solve it?

`process_sample` always returns a number. It does not always return a
*solution*. When every Newton path fails on a sample, the generated code still
has to emit something, and what it emits is bounded and smooth — so peak, RMS,
clipping indicators and the waveform on your scope all look entirely healthy.
**No level-based check can find this.** One counter can:
`diag_unsolved_sample_count`. It is present on every generated build, whatever
the solver route (always 0 where no Newton solve exists), and counts every
sample that was never solved. Assert it is zero; that is the whole check. Read
it after a render, or poll it per block.

Two mechanism counters carry the detail. Each exists only where its mechanism
does, and a build can have both; the unified count is their sum:

| Field | Present on | What a nonzero value means |
|-------|-----------|----------------------------|
| `diag_nr_hold_count` | nodal builds with a Newton solve (any nonlinear device, behavioral source or saturating inductor), both sub-paths | Every path failed (the solve, the sub-step ladder and, on a trapezoidal build, backward Euler) and the PREVIOUS sample's state was committed as this sample's output. Under a constant input this is a fixed point: the next sample re-poses the identical problem and fails identically, so the circuit can stay frozen until the input changes. |
| `diag_nr_unconverged_commit_count` | DK builds with devices; nodal full-LU builds with an active-set op-amp pin; nodal Schur builds with an active-set pin and no devices | The final solve (a DK Newton solve, or an op-amp's pinned solve) ended unconverged and that iterate was committed. The state still moves, so the solver can recover on its own — but those samples were never solved. |

A mechanism field is absent where its mechanism cannot occur, so a counter that
could never move does not read as reassurance. Code that must work on any
build reads the unified count instead.

All three are `u64`, cleared by `reset()`, and free to read on the audio
thread. They count every sample `process_sample` ran, including the silent
warm-up samples `CircuitState::default()` and `reset()` run.

Treat nonzero as "this render is not trustworthy", not as "quality degraded".
It is not a rounding error: measured on one deck, 43199 held samples out of
48000 produced output 22 dB adrift from the converged answer while reporting a
perfectly respectable −0.50 dBFS peak. A smaller count is not proportionally
safer — 21 held samples in 96000 still means 21 samples of fiction.

Nodal-Schur builds with devices also carry `diag_warm_start_fallback_count`:
Newton solves that could not start from the previous sample's voltages (the
point that keeps a switching circuit on its branch) and used an extrapolated
start instead. It is not an unsolved sample; a nonzero value is worth a look.

Two more counters say whether the circuit was driven with the input you
passed. They exist on every route:

| Field | What a nonzero value means |
|-------|----------------------------|
| `diag_input_clamp_count` | Input samples beyond `INPUT_LIMIT_V` (±100 V) were clamped to it. |
| `diag_input_nan_count` | NaN/Inf input (or `.inject`) samples were replaced by 0 V. |

Both are `u64` and cleared by `reset()`. The clamp keeps garbage host input
from reaching the solver; the counter is how you know it happened.

If you are surfacing one number to a user, surface whether it is zero.

The related counters (`diag_nr_max_iter_count`, `diag_be_fallback_count`,
`diag_substep_count`) are NOT the same claim. A sample that hit the iteration
ceiling and was then rescued by a sub-step or the backward-Euler fallback is a
converged solution reached by another consistent scheme. Those counters
describe how hard the solve was; the two above describe whether it happened.

## Levels, and the thing that will blow your monitors

`process_sample` returns **volts at the output node**, not a normalized ±1
sample. A tube plate circuit legitimately returns tens of volts. What the
generated file does to that value: DC block (5 Hz, unless `--no-dc-block`),
multiply by `OUTPUT_SCALES`, then hard-clamp at ±10 V by default
(`--output-clamp`).

The soft ear-protection limiter advertised in the README lives in the **plugin
wrapper** (`--format plugin`'s `lib.rs`), *not* in `circuit.rs`. On the
`--format code` path you get the hard clamp and nothing else — and with
`--no-dc-block` you do not even get that, only a NaN guard. Put your own gain
staging between this and a speaker.

Melange's answer to "it is too loud" is never `--output-scale`: the number it
returns is the voltage the circuit produces, and if that voltage is wrong, the
bug is in melange. Scale it in *your* code, at the boundary where volts become
DAW samples.

## Startup

`CircuitState::default()` starts from the DC operating point that was solved at
codegen time. Circuits with slow bias networks still need to settle, and
`WARMUP_SAMPLES_RECOMMENDED` is the estimate (`5·τ_max`, in host-rate samples;
it is an upper bound, and `WARMUP_ESTIMATE_CAPPED` tells you when even that is a
sanity cap rather than a measurement):

```rust
for _ in 0..circuit::WARMUP_SAMPLES_RECOMMENDED {
    circuit::process_sample(0.0, &mut state);
}
```

Do this after construction, after `set_sample_rate`, and after any preset-recall
jump of the pots — off the audio thread.

## Real-time rules

- `process_sample` — audio thread. No allocation, no locks, no syscalls.
- `set_pot_*` / `set_switch_*` — block rate. Cheap call, expensive next sample.
- `CircuitState::default()`, `set_sample_rate`, warmup loops — initialization
  only. `set_sample_rate` cannot change the solver route or oversampling, which
  are compile-time structural: to serve a different host rate optimally,
  compile for it.
- `CircuitState` is `Send` but not shared: one instance per voice or channel.

## License

The generated file incorporates GPL-licensed melange source (device equations,
solver, integration templates) and is therefore GPL-3.0-or-later itself. See
[Generated Code License](GENERATED_CODE_LICENSE.md) before shipping it.
