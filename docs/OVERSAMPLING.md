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

Do not guess, and do not take a number from this page. melange has no aliasing
meter of its own yet: render a test tone at 1× and at the factor you are
considering, and compare the two in a spectrum analyser.

```bash
melange simulate mycircuit.cir --input-audio tone.wav -o os1.wav
melange simulate mycircuit.cir --input-audio tone.wav -o os2.wav --oversampling 2
melange simulate mycircuit.cir --input-audio tone.wav -o os4.wav --oversampling 4
```

`tone.wav` is a single sine at the plugin's sample rate (`simulate` builds the
circuit at the WAV's own rate, so the file's rate is the rate you measure), at
a pitch that does not divide that rate. A musical pitch such as 4186 Hz works; 1 kHz or 4 kHz at
48 kHz does not, because every alias of such a tone lands exactly on one of its
own harmonics and cannot be told apart. The file's level is the drive in volts
(full scale = 1 V; a 32-bit float WAV can carry more). In the spectrum, the
harmonics are the multiples of the tone and should not change with the factor.
Everything else is aliasing, and it falls as the factor goes up.

`analyze`'s `nyquist_dbc` column is a different measurement: the output's
component at exactly half the sample rate, which is how a numerical limit cycle
shows up. It does not see aliases, which fold to `fs − k·f`, wherever that lands.

A two-diode clipper (`R1 in out 4k7`, `D1 out 0 DX`, `D2 0 out DX`,
`.model DX D(IS=2.52e-9 N=1.752)`), 4186 Hz at 48 kHz, total level of the
inharmonic products relative to the fundamental (measured 2026-09-30; "below
−92" means under the analysis window's own floor):

| Input drive | 1× | 2× | 4× |
|---|---|---|---|
| 0.3 V | −73.0 dBc | below −92 | below −92 |
| 1.0 V | −38.6 dBc | −62.2 dBc | below −92 |
| 3.0 V | **−19.3 dBc** | −44.7 dBc | −61.7 dBc |

THD was the same at every factor (0.78 %, 19.1 %, 27.2 %): oversampling changes
what folds, not the distortion itself.

**Drive sets how much there is to fold.** At 1×, 0.3 V → 3 V raises the aliases
by about 54 dB. A circuit that measures clean at a polite level can alias badly
when someone turns it up, and the steady-state frequency response will not show
it. **Measure at the drive your users will reach, with a tone near the top of
the range they will play**: the higher the tone, the sooner its harmonics fold.

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

A generated plugin (`--format plugin`) reports this to the host for delay
compensation, rounded to whole samples: 3 at both 2× and 4×. With
`--wet-dry-mix` the dry path is delayed by the same 3 samples.

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

`analyze` defaults to 48 kHz, the same as `compile`. If you ship at another
rate, pass `--sample-rate` to match it: a phase figure read at one rate is not
the one a plugin running at another produces.

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
- **How loud are the aliases at realistic drive?** Render and compare as above;
  that is the only number that reflects your circuit.
- **Is anything phase-critical downstream?** Parallel dry paths, mid/side work,
  and multi-band splits care about the dispersion above; a standalone distortion
  generally does not.
- **2× or 4×?** On the clipper above, 2× took 24–25 dB off the aliases at 1 V
  and 3 V; 4× took another 17 dB at 3 V and more than 30 dB at 1 V. Measure
  your circuit before paying for 4×.

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

To let the plugin choose the factor at runtime (a CPU/quality switch), declare
the set the code may switch between; the first number is the default:

```
.oversampling 2 allow=1,2,4
```

The generated code then has `state.set_oversampling(f)` (off the audio thread;
re-apply controls and re-warm after it, as after construction). At every factor
it computes exactly what a build fixed at that factor computes. melange builds
each factor and refuses the set, saying what differs, when the factors would
not be the same solver: on some circuits the integrator or solver route chosen
at one rate is not the one chosen at another. (Which matrix terms the code
carries is decided by the circuit's structure, the same at every rate, so a
term that is merely small at one rate does not split a set.) Build those once
per factor.

⚠️ **Without `allow=`, the factor is compile-time structural.** `set_sample_rate()`
cannot change it, and neither can a host — solver routing and sub-sample-fire
activation are decided at codegen against the internal rate. A plugin that must
run at several host rates needs compiling per rate.

⚠️ `melange validate` does **not** read `.oversampling` from the deck. Pass
`--oversampling` explicitly or you will validate a different build from the one
you ship.

## Implementation

Polyphase half-band IIR (allpass), after Olli Niemitalo; 4× is a cascade of two
2× stages. Coefficients, the polyphase decomposition, the generated processing
flow and the filter's own test corpus are documented for maintainers in
[`aidocs/OVERSAMPLING.md`](aidocs/OVERSAMPLING.md).
