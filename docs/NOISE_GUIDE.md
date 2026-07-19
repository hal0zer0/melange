# Circuit Noise Guide

Melange can emit the *authentic noise floor* of a circuit — Johnson-Nyquist
thermal hiss, junction shot noise, 1/f flicker, pentode partition noise, and
op-amp voltage/current noise — as real time-domain currents solved alongside
the signal. This guide is for netlist authors and plugin developers who want to
turn that on, wire it to knobs, and understand why the numbers look the way
they do.

It is off by default. A build compiled with `--noise off` (the default) is
byte-for-byte identical to a build with no noise support at all.

For the full physics, calibration derivations, and per-phase implementation
detail, see [`docs/aidocs/NOISE.md`](aidocs/NOISE.md). This guide is the
practical layer on top of it.

## Why this is not filtered hiss

Almost every "analog noise" plugin on the market does one of two things: play
back a recorded hiss sample, or sum post-processed white/pink noise into the
output. Both are *decoupled* from the signal path. The noise does not see the
circuit's transfer function, and loud passages do not modulate it.

Melange injects noise as **Norton current sources into the nonlinear MNA
right-hand side**, before each Newton-Raphson solve. That has three
consequences that fall out of the physics for free:

- **The noise is shaped by the circuit.** It passes through every downstream
  stage's transfer function automatically — no separate filter to design. A
  resistor's thermal current at the input of a tube stage comes out the
  plate shaped by that stage's gain and its output impedance.
- **Shot and flicker track the operating point.** Their amplitude scales with
  the instantaneous device current `|I(t)|`. A tube idling cold is quieter
  than the same tube swinging hard, because it *is* carrying less current.
- **Coupled stages correlate correctly.** Noise sources cross-couple through
  the same solver that couples the signal — one linear-algebra problem per
  sample, not a bank of independent generators.

This is why the consumer plugin Noyce can offer a real Temperature knob and a
"Listener mode" where the noise floor breathes with the input: those are
consequences of solving the physics, not features bolted on afterward.

## Turning it on

Two flags, accepted on `melange compile`, `melange simulate`, and
`melange analyze`:

```
--noise {off|thermal|shot|full}     default: off
--noise-seed <u64>                  default: 0  (0 = entropy, nonzero = deterministic)
```

For a plugin, `--noise` on `compile` is the one that matters. `simulate --noise`
is handy for auditioning the floor to a WAV before you build.

| Mode | Mechanisms emitted |
|------|--------------------|
| `off` | None. No code, no state, no CPU. Byte-identical to a noiseless build. |
| `thermal` | Johnson-Nyquist thermal on every resistor (Phase 1). |
| `shot` | Thermal + junction shot on every semiconductor/tube junction (Phase 1 + 2). |
| `full` | Thermal + shot + 1/f flicker + op-amp en/in + pentode partition (Phases 1–5). |

Each mechanism past thermal also gates on the circuit actually containing the
element it needs. `--noise full` on a passive RC network emits only thermal,
because there are no junctions, op-amps, or pentodes to be noisy. Flicker and
op-amp/pentode noise additionally require per-device opt-in (next section), so
`--noise full` on a circuit whose `.model` cards carry no noise parameters emits
only thermal and shot.

Example — audition a diode clipper's noise floor to a WAV:

```bash
melange simulate clipper.cir --noise shot --amplitude 0.1 -o floor.wav
```

Compile it into a plugin with a fixed, reproducible floor:

```bash
melange compile clipper.cir --noise full --noise-seed 12345 --format plugin -o my-clipper
```

## What each device contributes

Thermal noise needs no opt-in — every resistor is a source the moment you pass
`--noise thermal` or higher. Shot noise needs no opt-in either — it appears on
every junction under `--noise shot`/`full`. Flicker, op-amp, and pentode
partition noise are opt-in per device via `.model` parameters.

| Device | Thermal | Shot | 1/f flicker | Partition | Notes |
|--------|---------|------|-------------|-----------|-------|
| Resistor (fixed and `.pot`/`.wiper`/`.switch`/`.runtime R`) | yes, auto | — | opt-in per element (`KF`) | — | Hooge bias-squared; unbiased R emits only thermal |
| Diode | — | yes | opt-in (`KF`) | — | series `RS` is not yet a thermal source |
| BJT | parasitic `RB`/`RC`/`RE`, **nodal path only** | `Ic` + `Ib` (1 source when forward-active-reduced, plus a base-shot term) | opt-in (`KF`, both junctions) | — | rbb′ thermal is **skipped on the DK path** — see below |
| JFET / MOSFET | — | drain-source | opt-in (`KF`) | — | MOS gate shot ≈ 0; JFET gate leakage nonzero |
| Triode | — | plate, space-charge smoothed | opt-in (`KF`) | — | `SHOT_GAMMA2=` overrides the smoothing factor |
| Pentode | — | (replaced by partition) | opt-in (`KF`) | yes | `PARTITION_F=` scales the partition variance |
| Op-amp | — | — | — | — | input-referred `EN`/`IN` only (Boyle macromodel) |
| VCA | — | — | — | — | **no noise emitted** |

Defaults: junction `KF=0` (no flicker) with `AF=1.0`; resistor `KF=0` with
`AF=2.0`; pentode `PARTITION_F=1.0`; triode `SHOT_GAMMA2` auto-computed from the
operating-point transconductance; op-amp `EN=IN=0`. A device left at its
defaults contributes no opt-in source and adds zero generated code.

### The DK-vs-nodal thermal caveat

A BJT's base spreading resistance `rbb′` is the classically dominant transistor
voltage-noise term. Melange collects thermal noise for the parasitic `RB`/`RC`/
`RE` on the model card **only on the nodal codegen path**, where those
resistors occupy real internal nodes it can inject a Norton current across.

On the **DK path** (K_eff absorption, no internal nodes) there is no node pair
to stamp across, so the source is honestly skipped and codegen logs a
`log::warn!` naming the device. If you have a multi-BJT circuit where rbb′ hiss
matters (Neve/Wurlitzer-class output stages when they route DK), compile it with
`--solver nodal` to include that contribution. See
[`docs/aidocs/NOISE.md`](aidocs/NOISE.md) "BJT parasitic-R thermal noise".

## Opting in to flicker, op-amp, and pentode noise

These mechanisms are material/part properties, so they live on the device — on
the `.model` card for junctions and tubes, and per-element for resistors.

### Junction flicker (diodes, BJTs, JFETs, MOSFETs, triodes)

Add ngspice's `KF` and `AF` to the device's `.model` card. `KF=0` (the default)
means no flicker source is generated at all:

```spice
.model GE1N34A D(IS=1e-7 N=1.3 KF=1e-12 AF=1.0)       ; germanium diode, loud 1/f
.model 2N3904 NPN(IS=1e-14 BF=300 KF=1e-15 AF=1.0)    ; silicon BJT, quiet 1/f
.model 12AX7 TUBE(... KF=1e-13 AF=1.0)                 ; DHT/triode flicker
```

`AF` defaults to `1.0` for junctions (the ngspice convention). Rough magnitudes:
germanium BJTs and directly-heated triodes sit around `KF=1e-13`…`1e-11`
(corner 10–100 kHz); silicon BJTs `1e-16`…`1e-14` (corner 1–10 kHz); JFETs are
low-noise at `1e-18`…`1e-16`. These are empirical knobs — pick a value that
hits the character you want, then sweep it for part variants.

### Resistor flicker (Hooge bias-squared)

Resistor 1/f is a per-part property, so it is specified per element, not on a
shared model card. Append `KF` (and optionally `AF`) to the resistor line:

```spice
R1 vcc plate 100k KF=1e-10 AF=2.0    ; carbon-comp plate load, audible 1/f
R2 in mid 22k                        ; metal-film: no KF, thermal only
R3 mid 0 47k KF=1e-13                ; AF defaults to 2.0 when omitted
```

The physics is **Hooge bias-squared**: the flicker current scales with the DC
current through the resistor (`AF=2.0` default), so a resistor carrying no bias
current emits only its thermal floor. This is why in real gear the resistors
that audibly hiss are the biased ones — cathode resistors, plate loads, bias
networks — and why the floor "breathes" with the signal in a driven stage. A
coupling resistor with zero DC drop stays at the thermal floor.

`KF` and `AF` are case-insensitive and order-independent; the parser rejects
`KF < 0` and `AF ≤ 0`.

### Op-amp input-referred noise

Add datasheet `EN` (voltage-noise density, V/√Hz) and `IN` (current-noise
density, A/√Hz) to the op-amp `.model` card:

```spice
.model NE5534 OA(AOL=1e5 ROUT=75 EN=3.5n IN=0.4p)
.model TL072  OA(AOL=2e5 ROUT=50 EN=18n IN=0.01p)
```

Melange stamps three Norton streams (one voltage-noise equivalent at the
non-inverting input, one current-noise stream at each input) so the op-amp's
signature is applied to the real source impedance the circuit presents.

> **Limitation — 1/f corner not yet wired.** `EN_FC` and `IN_FC` parse and are
> stored, but v1 op-amp noise is **white-band only** — the 1/f blend is not yet
> implemented. Setting them does nothing today; do not rely on an op-amp 1/f
> corner. See "Limitations" below.

### Pentode partition

Partition noise (the statistical split of cathode current between plate and
screen) replaces the bare plate-shot stamp on pentodes automatically under
`--noise full`. The only knob is a process-variation scalar on the tube model:

```spice
.model EF86 TUBE(... PARTITION_F=0.6)    ; selected low-noise batch
.model EL84 TUBE(... PARTITION_F=1.0)    ; textbook (default)
```

`PARTITION_F` defaults to `1.0`. The dominant control over a pentode's noise
floor is the bias network in the netlist, which sets the screen/plate current
ratio. Triodes keep full plate shot; only 3-terminal pentode models get
partition. `SHOT_GAMMA2=` (triode) similarly overrides the space-charge
smoothing factor — `1.0` restores bare full-shot.

## The generated runtime API

When you compile with `--noise ≠ off`, `CircuitState` gains a set of methods.
**These exist only in noise-enabled builds** — call them from `#[cfg]`-gated or
feature-gated code, or simply always compile with noise on. The core setters
(`set_noise_enabled`, `set_noise_gain`, `set_thermal_gain`, `set_shot_gain`,
`set_temperature_k`, `set_seed`) are present in any noise build; the
mechanism-specific ones appear only when that mechanism is in the circuit —
`set_flicker_gain` only if some device carries `KF`, `set_opamp_input_gain` only
if an op-amp carries `EN`/`IN`.

```rust
state.set_noise_enabled(true);        // master on/off — branches out of all RNG when false
state.set_noise_gain(1.0);            // master scalar over every mechanism
state.set_thermal_gain(1.0);          // resistor hiss alone
state.set_shot_gain(1.0);             // junction shot (also gates pentode partition)
state.set_flicker_gain(1.0);          // all 1/f: junction and resistor flicker together
state.set_opamp_input_gain(1.0);      // op-amp en/in (present only with EN/IN models)
state.set_temperature_k(290.0);       // thermal temperature, Kelvin (default 290)
state.set_seed(42);                   // re-derive all RNG streams from a master seed
```

None of these are audio-thread-real-time-hostile in the sense of allocating,
but treat `set_seed` and `set_temperature_k` as parameter-change callbacks
(they touch per-source coefficients), and `set_noise_enabled`/`*_gain` as
per-block-safe scalars.

Typical plugin wiring:

- A **"Noise" amount** knob → `set_noise_gain` (or `set_noise_enabled(false)`
  at zero to skip the RNG entirely).
- A **Temperature** knob → `set_temperature_k`. This is real physics: 77 K
  drops thermal ~5.8 dB, 3 K drops it ~20 dB, hotter climbs it. Only thermal
  scales with temperature — shot, flicker, and partition are temperature-
  independent (a model's `BV`/`IS` tempco aside).
- Per-mechanism **trim** knobs → `set_thermal_gain` / `set_shot_gain` /
  `set_flicker_gain`. Noyce's "Age" knob is exactly `set_flicker_gain`.
- A **Seed** control with a re-roll button → `set_seed`.

`reset()` re-derives the RNG streams from the stored master seed (so
determinism survives a reset) but deliberately leaves `noise_enabled`, the gain
scalars, and `temperature_k` alone — those are user preferences, not transient
state.

## Levels and calibration: why bare sources are quiet

Melange noise is *physically scaled*. It is not normalized to a convenient
listening level — it is the actual current the physics produces. That means a
bare source is genuinely quiet, and that is correct, not a bug.

A reverse-breakdown Zener at ~1 mA has a dynamic resistance of roughly 26 Ω at
the knee (`n·Vt/IBV`). Its shot current `sqrt(2·q·I_z)` developed across that
26 Ω comes out on the order of **~100 nV at the junction terminal** — sub-
microvolt. A 10 kΩ resistor at 290 K produces about 12.7 nV/√Hz, roughly 1.8 µV
integrated over a 20 kHz band. These are the real numbers; analog noise floors
live in the microvolt-and-below range.

A plugin closes that gap in its own wrapper, not in the netlist. Noyce lands
each source in a calibrated dBFS band with a per-source `OUTPUT_TRIM` / gain
stage after `process_sample`. That is the honest place to do it: the netlist
emits true physics, and the plugin maps volts to dBFS — exactly the CLAUDE.md
boundary that says `--output-scale` and gain staging are voltage→DAW mapping,
never a fix for a simulation.

If you enable noise and hear nothing, the source is probably working correctly
and just needs downstream gain. Verify with a scope/meter on the raw output
before assuming it is broken.

## Determinism and reproducibility

`--noise-seed 0` (the default) seeds the RNG from the system clock at plugin
init — every render is different, which is what you want for a shipping noise
plugin. Any **nonzero** seed makes the output bit-identical across runs at the
same sample rate: same seed in, same samples out.

Each noise source gets its **own independent RNG stream** (xoshiro256++, derived
from the master seed via SplitMix64, with per-phase salts so thermal, shot, and
flicker streams never share a prefix). That independence is physically correct
— thermal noise in one resistor is uncorrelated with thermal noise in another —
and it means stereo decorrelation is as simple as running two states with two
different seeds.

`set_seed(master)` at runtime re-derives every stream, so a plugin can offer a
reproducible "Seed" control and a re-roll button that just increments it.

## ngspice-parity caveats

Two noise-related netlist features break correlation with ngspice, because
ngspice has no equivalent:

- **Resistor `KF`/`AF`** — ngspice's standard resistor model has no noise-index
  parameters. A resistor line with `KF` compiles fine in melange but is
  meaningless to ngspice.
- **`.mismatch` / `.tolerance` / `.seed`** unit-variation directives — these
  jitter per-device parameters and are melange-only.

If a circuit needs to pass `melange validate`, strip these before running the
comparison. Junction `KF`/`AF` on `.model` cards *are* standard ngspice
parameters and do not break parity, though ngspice's `.NOISE` analysis is a
small-signal frequency-domain linearization — it will never match melange's
time-domain stochastic injection sample-for-sample, only in integrated PSD.

## Limitations (honest scope)

These are real gaps, not soft-pedaled:

- **Op-amp `EN_FC`/`IN_FC` are unwired.** v1 op-amp noise is white-band only.
  The parameters parse but the 1/f corner blend is not implemented. An op-amp's
  low-frequency flicker rise is not modeled today.
- **DK-path BJT `rbb′` thermal noise is skipped** (logged `warn!`). Route the
  circuit `--solver nodal` to include base/collector/emitter parasitic-R
  thermal noise. See the DK-vs-nodal caveat above.
- **Diode series `RS` and tube grid resistance `RGI` are not thermal-noise
  sources.** Same class of gap as the DK rbb′ case, smaller magnitude.
- **VCAs emit no noise** of any kind.
- **Tube microphonics (Phase 6) are not implemented** — mechanical/vibration
  coupling into cathode-plate spacing is research-tier, not shipping.
- **Shot and flicker use a one-sample-lagged operating point.** Amplitude is
  computed from the previous sample's device current. At 44.1–192 kHz this lag
  is well below the audible modulation threshold.

## Worked example: a Zener reverse-junction noise source

This is the classic "synth noise oscillator" done with real physics: a single
reverse-biased Zener at breakdown, whose junction shot current *is* the signal.
It exercises thermal and shot with no gain stage, which makes it a clean
illustration of the compile shape and the level calibration.

> This netlist is illustrative — a teaching example drawn from the Noyce
> palette (oomox `unstable/gimmicks/noyce-zener-junction.cir`), not a promoted
> melange stable circuit. It compiles cleanly today; treat the component values
> as a starting point, not a validated reference.

```spice
* Zener reverse-junction noise source (illustrative)
Vcc vcc 0 DC 9
R_bias vcc n_cath 3.9k
D1 n_zener n_cath DZ5V1
R_iso in n_zener 1meg
Cout n_zener out 1u
Rload out 0 100k
.model DZ5V1 D(IS=1e-14 N=1.5 BV=5.1 IBV=1m)
.end
```

The 9 V rail through the 3.9 kΩ `R_bias` pushes ~1 mA of reverse current through
the 5.1 V Zener `D1` (cathode toward the rail, anode at the audio-coupling node
`n_zener`). `R_iso` gives node `n_zener` a DC path to ground and doubles as a
high-Z input port; `Cout` blocks DC to `out`, where `Rload` sets the output
reference.

Compile it and inspect the noise shape:

```bash
melange compile zener.cir --noise full --format code -o zener.rs
```

The generated code locks the expected source counts:

- `NOISE_THERMAL_N = 3` — `R_bias`, `R_iso`, `Rload`.
- `NOISE_SHOT_N = 1` — `D1`, injected between its anode and cathode nodes.
- No flicker (the model carries no `KF`), no op-amp, no partition.

The shot current at the junction is the audible mechanism, and it is wideband:
because the breakdown junction is low-impedance (Rz ≈ 26 Ω) and the coupling
network has no audio-band RC corner, the spectrum is flat to Nyquist — audibly
different from Clean-RC thermal (which rolls off at the cap corner). At the bare
terminal it is sub-microvolt, per the calibration section; the plugin lifts it
to a usable level with `OUTPUT_TRIM`.

### A subtlety this source flushed out

Stiff reverse-breakdown junctions revealed a numerical issue in the shot path
(fixed 2026-07-19). Under trapezoidal integration, a resistor-only node carries
a `z = −1` pole at Nyquist. A Zener at breakdown has ~26 Ω dynamic resistance,
so its auto-inserted 10 pF parasitic cap sits at ~600 MHz — four decades above
fs/2 — leaving the junction node effectively resistor-only at Nyquist. The
original single-draw shot injection excited that pole into an fs/2 limit cycle,
which the breakdown exponential then rectified into the audio band (seed-
dependent level, tens of dB hot). The fix stamps shot as a two-draw
anti-aliased pair `i_n = w[n] + w[n−1]`, zeroing the Nyquist bin while leaving
the audio band unchanged — the same scheme thermal noise already used. You do
not need to do anything to get the fix; it is automatic. It is documented here
because it is a good illustration of *why* melange injects noise into the solver
RHS rather than filtering it afterward: the noise inherits both the circuit's
correct behavior and its numerical subtleties, and both have to be handled
honestly. Full detail in [`docs/aidocs/NOISE.md`](aidocs/NOISE.md) under
"Nyquist anti-aliasing".

## Where to go deeper

- [`docs/aidocs/NOISE.md`](aidocs/NOISE.md) — the authoritative reference:
  every formula, the trap-MNA calibration derivations, the Kellett pink filter,
  per-phase validation, and the failure-mode table.
- [Netlist Writing Guide](NETLIST_GUIDE.md) — the `.model` and per-element
  syntax the noise opt-in parameters attach to.
- [Plugin Development Guide](PLUGIN_GUIDE.md) — where the runtime API and
  `OUTPUT_TRIM`-style level mapping live in a generated plugin.
- [`docs/aidocs/STATUS.md`](aidocs/STATUS.md) — the feature inventory, including
  the current per-phase noise status.
