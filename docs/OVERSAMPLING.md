# Oversampling

Melange defaults to **1×**. This page is what you need to decide whether that
is right for your circuit, and what you are trading if you change it.

The short version: oversampling buys alias rejection and costs CPU, latency and
phase accuracy. It is not a quality dial that only goes up.

## Why a nonlinear circuit needs it

A linear circuit only ever outputs frequencies you put in. A nonlinear one —
any diode, tube, transistor, or op-amp driven near its rails — generates new
harmonics. Feed it 5 kHz and it makes 10, 15, 20, 25 kHz and beyond.

At 48 kHz, anything above 24 kHz cannot be represented. It does not vanish; it
**folds back** into the audible band, landing at frequencies unrelated to
anything you played. Harmonic distortion is consonant with the note. Aliasing is
not, which is why it reads as grit or harshness rather than warmth.

Oversampling runs the solver at 2× or 4× the host rate, so those harmonics have
somewhere to go, then filters them off before decimating back down.

## How much aliasing you actually have

Do not guess, and do not take a number from this page. `melange analyze
--harmonics N` reports a **`nyquist_dbc`** column — folded-back energy relative
to the fundamental, so less negative is worse.

```bash
melange analyze mycircuit.cir --harmonics 5 --amplitude 1.0 --oversampling 1
melange analyze mycircuit.cir --harmonics 5 --amplitude 1.0 --oversampling 4
```

Compare the worst `nyquist_dbc` across each sweep.

A two-diode clipper (`R1 in out 4k7`, `D1 out 0 DX`, `D2 0 out DX`), swept
20 Hz–20 kHz:

| Input drive | 1× | 4× | Difference |
|---|---|---|---|
| 0.3 V | −41.4 dBc | −48.0 dBc | 6.6 dB |
| 1.0 V | −41.5 dBc | −46.3 dBc | 4.8 dB |
| 3.0 V | **−27.8 dBc** | −34.5 dBc | 6.7 dB |

**Drive dominates the factor.** Going 1 V → 3 V costs about 14 dB of alias
rejection; 4× gives back about 5. A circuit that measures clean at a polite
level can alias badly when someone turns it up, and the steady-state frequency
response will not show it. **Measure at the drive your users will reach.**

**And the benefit is circuit-specific.** On an op-amp overdrive stage the same
1× → 4× comparison measured roughly 19 dB rather than 5. Two nonlinear circuits
are not interchangeable here; one measurement of yours beats any table of
someone else's.

## What it costs: CPU

Roughly linear in the factor — 4× does about four times the solver work per
output sample. Divide the throughput figures in the README by the factor for a
first estimate, then measure.

## What it costs: latency

The half-band filters are not free in time. Measured as the extra delay an
oversampled build has over a 1× build of the same circuit, in **host** samples:

| Factor | Added latency | At 48 kHz |
|---|---|---|
| 2× | 2.65 samples | 55 µs |
| 4× | 3.47 samples | 72 µs |

Fitted from the excess phase over a 1× build of the same circuit; fits at
several frequencies agree to a few thousandths of a sample, which is what tells
you it is a delay and not a frequency-dependent effect.

Tens of microseconds is irrelevant for most uses. It matters if you are
phase-matching against a dry path, splitting into bands, or building something
latency-critical — and it matters when you read a phase plot, below.

### It shows up in `analyze` as phase, and that surprises people

A constant delay is a phase slope. The same RC low-pass at 2 kHz, 48 kHz host
rate:

| Factor | Reported phase |
|---|---|
| 1× | −51.7° |
| 2× | −91.4° |
| 4× | −103.7° |

The circuit did not change — its gain is identical in all three. The extra 40°
and 52° are the filter delay expressed as phase, exactly as a plot of a delayed
signal should look.

⚠️ `analyze` defaults to a **96 kHz** sample rate, so a phase figure read from
it is not the one your 48 kHz plugin produces. Pass `--sample-rate` to match
what you ship.

So when comparing phase across oversampling factors, subtract the delay above,
or compare gain only. `analyze` reports what the built plugin actually does,
delay included, rather than a phase response with the latency quietly removed.

## What it costs: phase accuracy — the part that surprises people

**An oversampled build agrees with a reference engine *less* well than a 1×
build does, not more.** Measured against ngspice on an overdrive deck, 500 ms,
delay-aligned:

| | 500 ms 1−ρ | 500 ms nRMS |
|---|---|---|
| 1× | 1.00e-6 | 0.142 % |
| 2× | 5.64e-6 | 0.336 % |
| 4× | 6.25e-6 | 0.355 % |

That is 5.6× worse correlation at 2×, 6.3× at 4×, and it is in the shipped
plugin rather than in the measurement rig.

The cause is **phase dispersion in the half-band IIR filters**, not an error in
the solve. The two effects pull in opposite directions:

- harmonic **magnitudes** track the reference *better* when oversampled — the
  finer internal timestep genuinely improving the answer;
- harmonic **phases** drift, and correlation is dominated by the phase term.

Phase error grows with harmonic order, which is the signature of dispersion
rather than of an amplitude error:

| Harmonic | 1× | 2× |
|---|---|---|
| 3rd (3 kHz) | 0.055° | −0.194° |
| 5th (5 kHz) | 0.113° | −0.954° |
| 7th (7 kHz) | 0.223° | −2.691° |
| 9th (9 kHz) | 0.421° | −5.937° |

Attribution, measured by swapping one filter leg at a time for a linear-phase
FIR: on a single tone, **98.3 % of the excess is the decimator** and 1.3 % is
the interpolator. The harmonics are created *after* the up leg, so on a single
tone only the down leg ever filters them.

The interpolator is not inert, it is invisible to a single tone. Driven with two
tones at 19 kHz + 20 kHz, swapping only the up leg moves intermodulation
products by 0.31–1.09 dB. At 1 kHz + 1.1 kHz — where a pedal is actually played
— it moves nothing measurable (≤ 0.024 dB).

These are **engine-versus-engine** figures: how closely melange tracks ngspice,
not how closely either tracks hardware.

## So when should you oversample?

There is no universal answer, which is why melange does not pick for you. The
questions that decide it:

- **Is the circuit nonlinear at the drive you ship?** If not, 1×. Oversampling a
  linear circuit buys nothing and costs everything above.
- **What does `nyquist_dbc` say at realistic drive?** That is the only number
  that reflects your circuit.
- **Is anything phase-critical downstream?** Parallel dry paths, mid/side work,
  and multi-band splits care about the dispersion above; a standalone distortion
  generally does not.
- **2× or 4×?** The clipper table shows most of the benefit arriving by 2×.
  Measure before paying for 4×.

## Setting it

Per compile:

```bash
melange compile mycircuit.cir --format plugin --oversampling 4 -o mydist
```

Or as a recommendation in the netlist, which `compile` and `simulate` honour and
a CLI flag overrides:

```
.oversampling 4
```

⚠️ **The factor is compile-time structural.** `set_sample_rate()` cannot change
it, and neither can a host — solver routing and sub-sample-fire activation are
decided at codegen against the internal rate. A plugin that must run at several
host rates needs compiling per rate.

⚠️ `melange validate` does **not** read `.oversampling` from the deck. Pass
`--oversampling` explicitly or you will validate a different build from the one
you ship.

## Implementation

Polyphase half-band IIR (allpass), after Olli Niemitalo; 4× is a cascade of two
2× stages. Coefficients, the polyphase decomposition, the generated processing
flow and the filter's own test corpus are documented for maintainers in
[`aidocs/OVERSAMPLING.md`](aidocs/OVERSAMPLING.md).
