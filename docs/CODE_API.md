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

Pot and switch setters are cheap themselves but mark the matrices dirty; the
rebuild (O(N³)) happens inside the next `process_sample`. Drive them per block,
never per sample.

`process_sample` has two variants, both visible at the top of the generated
file — grep it rather than assuming:

- decks compiled with several comma-separated input nodes (`-i in_l,in_r`,
  linear `M = 0` circuits only) take `inputs: [f64; NUM_INPUTS]` instead of a
  single `f64`, alongside `NUM_INPUTS` / `INPUT_NODES` / `INPUT_RESISTANCES`;
- `.inject` / `.tap` decks take an extra injection argument and return a tuple.

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
