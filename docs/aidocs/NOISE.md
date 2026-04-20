# Authentic Circuit Noise

> **Status** (2026-04-20)
> - **Phase 1 shipped** (`25b61ba`). Johnson-Nyquist thermal noise on
>   fixed resistors via MNA-RHS Norton-current stamping, DK codegen path.
>   Runtime `set_noise_enabled(bool)` + gain/temperature/seed controls.
>   Default `--noise off` → byte-identical to pre-feature codegen.
> - **Phase 1.5 Step 1 shipped** (`2b09cc3`). Nodal codegen path — `build_noise_emission` wired into `emit_nodal` end-to-end; kTC theorem
>   holds on the nodal path with the same calibration constant.
> - **Phase 1.5 Step 2 shipped** (`1fdcac2`). Dynamic resistors —
>   `.pot` / `.wiper` / `.runtime R` members are now thermal noise
>   sources; the per-sample `sqrt(1/R)` coefficient lives in
>   `state.noise_thermal_sqrt_inv_r[k]` and is refreshed inside each
>   emitted `set_pot_N` / `set_runtime_R_<field>` setter. `.switch`
>   R-components still deferred (need position→R table).
> - **Phase 1.5 Step 3 closed** (investigation only, 2026-04-20). The
>   originally-noted "persistent Nyquist-rate component after
>   `set_noise_enabled(false)`" does not reproduce on the reference RC
>   lowpass + 5 Hz DC-blocker repro — measured residual is ~200 nV,
>   non-Nyquist, and matches the DC-blocker HPF settling time constant.
>   See "Known Phase 1 observations" below for the measurement.
> - **Phase 2 (shot noise) shipped** 2026-04-20. Per-junction shot
>   sources — Diode 1 (anode/cathode), BJT 2 (Ic at C/E, Ib at B/E) or
>   1 when forward-active reduced, JFET/MOSFET 1 (drain/source), Tube 1
>   (plate/cathode). VCA/op-amp skipped. Per-sample amplitude
>   `sqrt(4·q·|I_prev|·fs)` using `state.i_nl_prev[slot]` one-sample
>   lagged — same 2× trap-MNA calibration as thermal. Runtime
>   `set_shot_gain(f64)`; salted RNG streams (`NOISE_SHOT_SALT`) so
>   thermal and shot never share a prefix. Available via `--noise shot`
>   or `--noise full`.
> - **Constants resolved**: thermal `sqrt(8·k_B·T·fs/R)`, shot
>   `sqrt(4·q·|I|·fs)`. Both carry the same 2× trap-MNA compensation;
>   see "Constant derivation".
> - **Tested**: 10 emission-assertion tests in
>   `crates/melange-solver/tests/codegen_verification_tests.rs` + 4
>   end-to-end PSD tests in `tests/noise_psd_validation.rs` (DK kTC,
>   nodal kTC, dynamic-pot first-sample-divergence, shot-noise
>   audible-wiring check).
> - **Phases 3–5 deferred**: 1/f flicker, op-amp en/in, pentode
>   partition. The `NoiseIR` + `build_noise_emission` scaffold accepts
>   them without architectural churn — add per-phase `Vec<*NoiseSource>`
>   fields + a stamp fragment.

## Why this is different

SPICE `.NOISE` is a **small-signal frequency-domain linearization**. It reports
equivalent input-referred noise at the operating point — it does not inject
time-domain stochastic currents into the transient solve. Commercial analog
modelers ship either:

1. Static IR convolution of a recorded hiss sample, or
2. Post-process filtered white/pink noise summed into the output.

Both are *decoupled* from the signal path. Loud passages do not modulate noise
through nonlinearities. Output-stage thermal noise does not see the full
feedback transfer function. A tube idling cold does not sound different from a
tube swinging hard.

Melange has a **nonlinear time-domain MNA solver with full operating-point
tracking**. Noise currents injected as Norton sources into the RHS are:

- Shaped by the Jacobian's transfer function automatically — no separate filter design
- Modulated by the nonlinear operating point (shot noise scales with `|I(t)|`)
- Correctly correlated across coupled stages through S and K
- Integrated by trapezoidal rule at the solver's internal rate (oversampling → more physical bandwidth captured pre-decimation)

This is physically what happens in the real circuit. Nobody else does it this
way because nobody else has the nonlinear MNA to inject into.

## Architecture

### Injection point

Noise is a stochastic contribution to the KCL RHS. Given source `k` between
nodes `i, j` with sampled current `i_n[t]`:

```
rhs[i-1] += i_n[t]      (current injected at i)
rhs[j-1] -= i_n[t]      (extracted at j)
```

Stamped in `build_rhs` **after** `A_neg * v_prev + N_i * i_nl_prev + input`
and **before** the NR solve. The Jacobian is unchanged — noise is purely a
RHS term. Same code path on DK, nodal-Schur, nodal-full-LU.

Shot and flicker amplitudes use **previous-sample operating-point currents**
(read from `state.i_nl_prev` + device parasitic currents derived from
`v_prev`). This is a one-sample lag, which is physically indistinguishable at
audio rates.

### RNG

Per-stream **xoshiro256++** (256-bit state, passes BigCrush, ~5 ns/call). One
independent stream per noise source so thermal noise in R1 is uncorrelated
with thermal noise in R47 (physically correct — independent thermal
reservoirs).

Seeds derived from one user-facing `master_seed: u64` via **SplitMix64**:

```rust
let mut sm = SplitMix64::new(master_seed);
for stream in 0..NOISE_N {
    let s0 = sm.next(); let s1 = sm.next(); let s2 = sm.next(); let s3 = sm.next();
    rng[stream] = Xoshiro256pp { s: [s0, s1, s2, s3] };
}
```

Master seed 0 at `CircuitState::default()` → seeded from system entropy
(`getrandom`); any nonzero value → deterministic. `set_seed(master)` re-derives
all streams. Reproducible renders available; nobody pays for them unless they
opt in.

Gaussian transform: **Marsaglia polar method** (~1 log + 1 sqrt, no trig,
60% acceptance, caches one sample via `has_next_gaussian` flag). No Ziggurat
tables (worse cache behavior for inner-loop use).

### State layout

```rust
pub struct CircuitState {
    // ... existing fields ...

    // Noise state (present only when NOISE_MODE != Off)
    pub noise_rng: [Xoshiro256pp; NOISE_N],   // one stream per source
    pub noise_gaussian_cache: [Option<f64>; NOISE_N],
    pub noise_enabled: bool,      // default false
    pub noise_gain: f64,          // master scalar, default 1.0
    pub thermal_gain: f64,        // thermal-only scalar, default 1.0
    pub shot_gain: f64,           // phase 2+
    pub flicker_gain: f64,        // phase 3+
    pub temperature_k: f64,       // default 290.0 K (16.85 °C lab temp)
    pub noise_master_seed: u64,   // for re-seeding after set_seed()
}
```

Memory: 32 bytes per stream + 9 bytes of cache-flag per stream + ~40 bytes of
scalars. ~1-2 KB per `CircuitState` for a Pultec-class circuit (~100 sources
with all phases enabled). Trivial.

### Runtime controls

All emitted only when `NOISE_MODE != Off` (zero cost when disabled at
compile time):

```rust
impl CircuitState {
    pub fn set_noise_enabled(&mut self, on: bool);     // branch out of all RNG calls
    pub fn set_noise_gain(&mut self, gain: f64);       // master scalar
    pub fn set_thermal_gain(&mut self, gain: f64);     // "resistor hiss" alone
    pub fn set_shot_gain(&mut self, gain: f64);        // "junction crackle"
    pub fn set_flicker_gain(&mut self, gain: f64);     // "1/f warmth"
    pub fn set_temperature_k(&mut self, kelvin: f64);  // cold gear is quieter
    pub fn set_seed(&mut self, master: u64);           // reproducibility
}
```

`reset()` restores RNG state from master seed (keeps determinism) but leaves
`noise_enabled` / `*_gain` / `temperature_k` alone (they are user preferences,
not transient state).

### Zero-cost when off

Three levels of disable:

1. **Compile-time** (`--noise off`, default): no NOISE_N const, no RNG, no
   state fields, no `build_rhs` contribution. Byte-identical to prior codegen.
2. **Runtime compiled-in-but-off** (`set_noise_enabled(false)`): single
   branch at start of noise stanza in `build_rhs` skips all RNG calls.
   State still exists (~1-2 KB) but no CPU cost per sample.
3. **Per-category gain 0.0**: thermal/shot/flicker gain zero skips that
   category's inner loop.

The branch predictor handles #2 perfectly. Measured cost of
`noise_enabled=false` at compile: within measurement noise of noiseless build.

## Physical Formulas

All formulas cite the frequency-domain PSD (`V²/Hz` or `A²/Hz`). To convert
to a per-sample amplitude at sample rate `fs`, multiply by `fs` (one-sided
noise PSD integrated over Nyquist bandwidth) and take the square root.

### Johnson-Nyquist (Thermal)

Every resistor in thermal equilibrium at temperature T dissipates noise power:

```
S_v(f) = 4·k_B·T·R       [V²/Hz]        (Thevenin: voltage noise in series with R)
S_i(f) = 4·k_B·T/R       [A²/Hz]        (Norton: current noise in parallel with R)
```

where `k_B = 1.380649e-23 J/K` (exact SI).

**Per-sample Norton current at `fs`:**

```
i_n_rms = sqrt(8·k_B·T·fs / R)
i_n[t]  = i_n_rms · N(0,1)          // Gaussian with unit variance
```

The `8` (not the textbook `4`) is the melange-specific calibration —
see the "Constant derivation" section below. Validated by the
kTC-theorem test in `tests/noise_psd_validation.rs`.

**Temperature scaling**: `T` is runtime-settable. 290 K is the standard lab
reference (the "kT" in `kTB`). 77 K (liquid nitrogen) reduces noise by ~5.8 dB;
3 K (cryo DACs, academic curiosity) reduces by ~20 dB. This is physically
real and unique to us.

**Validation**: DC-bias R1 into an output, zero input, integrate PSD over band.
Must match `4·k_B·T·R_eff` where `R_eff` is the Thevenin equivalent seen at
the probe point, within 10% (limited by FFT window and sample count).

### Shot (Junction) Noise — Phase 2

Every PN junction with current `I(t)` flowing through it generates:

```
S_i(f) = 2·q·|I(t)|      [A²/Hz]        (current source across junction)
```

where `q = 1.602176634e-19 C` (exact SI).

**Signal-dependent** — this is the killer: loud passages produce more shot
noise during the loud parts, through the correct transfer function. Nobody
else does this.

**Implementation**:
- Diode: inject across (anode, cathode), `I = i_nl_prev[k]`
- BJT: inject at (collector, emitter) for `Ic`, at (base, emitter) for `Ib`,
  using block-diagonal M indices
- JFET/MOSFET: inject at (drain, source) for `Id`; gate shot ≈ 0 for MOS (good
  approximation), nonzero for JFET reverse-biased gate leakage
- Tube: inject at (plate, cathode) for `Ip`, at (grid, cathode) for `Ig`

Sign of `|I|`: always take magnitude. Shot noise doesn't care about direction;
it's the granularity of charge-carrier flow. Use `i_nl_prev` from the
previous sample (one-sample lag; at 48-192 kHz this is below the audible
modulation threshold).

### Flicker (1/f) Noise — Phase 3

ngspice's `KF`, `AF` model parameters. PSD shape:

```
S_id(f) = KF · I_d^AF / f           [A²/Hz] for MOSFET
S_ib(f) = KF · I_b^AF / f           [A²/Hz] for BJT
```

**Implementation**: Paul Kellett pink filter (cascade of 1-pole IIRs, ~3 dB/oct,
accurate to within 0.5 dB over 10 Hz–20 kHz). Per-device state: 7 filter-state
floats. Pink output is scaled by `sqrt(KF · |I|^AF)` using current operating
point.

Some tubes (especially DHT) have audible 1/f. Default `KF=0`, opt-in via
`.model` params. Per-device override supported.

### Op-Amp Input-Referred — Phase 4

Boyle macromodel extension: add two independent noise generators at each input
pin.

```
S_en(f) = EN² + EN_1F² · f_c/f       [V²/Hz]  at non-inverting input
S_in(f) = IN² + IN_1F² · f_c/f       [A²/Hz]  at each input
```

Data-sheet values per op-amp family:
- TL072: `EN = 18 nV/√Hz`, `IN = 0.01 pA/√Hz`
- NE5534: `EN = 3.5 nV/√Hz`, `IN = 0.4 pA/√Hz`
- OP07: `EN = 10 nV/√Hz`, `IN = 0.13 pA/√Hz`

Added as `.model OA(EN=… IN=… EN_FC=… IN_FC=…)` parameters. Defaults zero.

### Pentode Partition — Phase 5

In a pentode, the total plate+screen current is set by grid voltage, but the
*partition* between plate and screen is statistical. This adds a noise term
even when the total current is steady:

```
S_ip(f) = 2·q·I_p · (I_g2 / (I_p + I_g2))       [A²/Hz]
```

Specific to 3-dim pentode codegen path (`SharpPentode` / `SharpPentodeGridOff`
unchanged — grid-off mode skips partition by construction).

### Tube Microphonics — Phase 6 (Research)

Mechanical coupling from cabinet/speaker vibration into cathode-plate spacing.
Adds signal-dependent noise because the driving signal causes the vibration.
Requires a mechanical coupling model (envelope follower + stage-dependent EQ +
possibly an external audio-reference input for the speaker's output). Likely
belongs in the device model with a new `.microphonics` directive, but may end
up as a post-process in oomox depending on what we learn.

## Sample-rate handling

All formulas use `fs_effective = sample_rate * OVERSAMPLING_FACTOR` — the
solver's internal rate. When oversampling is active, we generate noise at the
internal rate and the half-band downsampler filters it along with the signal.
This **increases physical bandwidth captured pre-decimation** (more of the
white-noise tail is represented correctly), which is a free authenticity bonus
for `--oversampling 4`.

`set_sample_rate(sr)` must recompute all per-source `i_n_rms` coefficients.
Done in the existing `set_sample_rate` path where matrices are rebuilt.

## Codegen Surface

### CLI

```
--noise {off|thermal|shot|full}        default off
--noise-seed <u64>                     default 0 → entropy
```

- `off`: no code emitted, zero cost, byte-identical to noiseless build
- `thermal`: Phase 1 only (Johnson-Nyquist on every R)
- `shot`: Phase 1 + Phase 2 (+ junction shot)
- `full`: Phases 1–5 (thermal + shot + 1/f + op-amp en/in + partition)

Phases 2–5 gate on themselves *and* the required circuit element being
present, so `--noise full` on a passive RC circuit emits only thermal.

### Generated code

```rust
const NOISE_MODE: u8 = 1;                 // 0=off, 1=thermal, ...
const NOISE_N_THERMAL: usize = 47;        // number of thermal sources

// Per-source constants
const NOISE_THERMAL_NODE_I: [usize; NOISE_N_THERMAL] = [...];
const NOISE_THERMAL_NODE_J: [usize; NOISE_N_THERMAL] = [...];
const NOISE_THERMAL_RESISTANCE: [f64; NOISE_N_THERMAL] = [...];

// Xoshiro256++ + SplitMix64 + Gaussian (single copy per generated file)
struct Xoshiro256pp { ... }
fn splitmix64(x: u64) -> (u64, u64);
fn gaussian(rng: &mut Xoshiro256pp, cache: &mut Option<f64>) -> f64;

// In build_rhs:
if state.noise_enabled {
    let t_k = state.temperature_k;
    let scale = state.noise_gain * state.thermal_gain;
    for k in 0..NOISE_N_THERMAL {
        let r = NOISE_THERMAL_RESISTANCE[k];
        let coeff = (8.0 * K_B * t_k * FS_INTERNAL / r).sqrt();
        let i_n = coeff * scale * gaussian(&mut state.noise_rng[k],
                                           &mut state.noise_gaussian_cache[k]);
        let ni = NOISE_THERMAL_NODE_I[k];
        let nj = NOISE_THERMAL_NODE_J[k];
        if ni > 0 { rhs[ni - 1] += i_n; }
        if nj > 0 { rhs[nj - 1] -= i_n; }
    }
}
```

With `--noise off`, **none** of this is emitted. The `state` field itself
is behind the same cfg.

### Resistor selection (Phase 1)

- **All fixed resistors** from `netlist.elements` (Element::Resistor)
- **Skip** resistors marked by `.pot` / `.wiper` / `.switch` — their G stamp is
  runtime-mutable, and the per-sample `sqrt(8·k_B·T·fs/R)` would need to be
  recomputed on pot change. Deferred to a Phase 1.5 that recomputes the
  per-source coefficient in `set_pot_N`. Mentioned in a code comment.
- **Include** auto-inserted parasitic caps: N/A — caps are noiseless (no
  dissipation)
- **Include** internal-node parasitics (`RB`, `RC`, `RE`, `RS`, `RGI`): they
  appear as regular R elements after MNA expansion — yes, they count. The
  per-device parasitic Rs are part of the resistor list.

## Validation

### Phase 1: thermal PSD (shipped)

Validation lives in `crates/melange-solver/tests/noise_psd_validation.rs`.
Rather than an FFT bin-by-bin comparison (which is sensitive to window
choice and fs/2 tail truncation), the test uses the **kTC theorem**: for
any passive RC lowpass with thermal noise on `R`, the integrated output
variance at the cap is exactly `k_B·T/C`, **independent of `R`**. This
is a single-number physical invariant that falls out of

```
V²_rms = ∫₀^∞ 4·k_B·T·R / (1 + (2π·f·R·C)²)  df  =  k_B·T/C
```

The test runs four scenarios in one compiled binary:
- `T=290 K, gain=1, seed=42` — variance must match `k_B·T/C` within ±15 %
- `T=77 K, gain=1, seed=42` — variance ratio must be `77/290` within ±15 %
- `T=290 K, gain=0.5, seed=42` — variance ratio must be `0.25` within ±1 %
  (same seed → bit-identical RNG stream → gain scaling is exact)
- `T=290 K, gain=1, seed=42` (repeat) — bit-identical variance

N=2^17 samples at 96 kHz with `dc_block: false` to preserve the full
low-frequency tail.

### Phases 2+

- Shot noise scales linearly with `|I|` (bias test at multiple DC levels)
- Shot noise modulates with signal (FFT at signal frequency + sidebands)
- 1/f slope is -3 dB/oct within ±0.5 dB over 3 decades
- Op-amp noise matches data-sheet EN/IN with input grounded

## Failure modes and sanity

| Symptom | Likely cause | Check |
|---|---|---|
| Output all zero with `set_noise_enabled(true)` | Master seed re-derives identical streams each `reset()` | `noise_rng[*]` should differ across streams |
| PSD is 2× or 0.5× expected | Emitter constant drift from the calibrated `8·k_B·T·fs/R` per-sample variance | Formula is melange-calibrated: `sqrt(8·k_B·T·fs/R)` — see "Constant derivation" below |
| Low-freq roll-off missing | Noise added after `compute_final_voltages` instead of in RHS | Noise must be in `rhs` so NR shapes it |
| Thermal correlated across R's | Shared RNG stream | Each source gets its own xoshiro instance |
| DC offset from noise | Gaussian mean != 0 (bad polar impl) | Test `mean(gaussian())` over 10^6 samples ≈ 0 |
| NR divergence with noise on | RHS too large at device junctions | Normally impossible; thermal is sub-mV per sample at room temp |
| Deterministic seed produces different audio across runs | OS-dependent ordering in resistor enumeration | Order by netlist position, not HashMap iteration |

## Constants

```rust
pub const K_B: f64 = 1.380649e-23;   // Boltzmann constant [J/K], exact SI 2019
pub const Q_E: f64 = 1.602176634e-19; // Elementary charge [C], exact SI 2019
pub const T_ROOM: f64 = 290.0;        // Standard lab noise temperature [K]
```

These go in `crates/melange-solver/src/constants.rs` (new file if none exists,
else add to wherever `VT_THERMAL` etc. live).

## Known Phase 1 observations

### Trapezoidal-rule + DC-blocker post-noise decay

*Status (2026-04-20, after Phase 1.5 Step 3 investigation):* the
originally-documented "persistent 0.1–2 mV Nyquist-rate component" does
**not** reproduce on the reference RC lowpass + 5 Hz DC blocker. Measured
on DK (and bit-identically on nodal), the tail after `set_noise_enabled(false)`
is:

- **No Nyquist content** — sign-flip fraction ≈ 0 across the decay.
- **Peak ≈ 200 nV**, not 0.1–2 mV.
- **Smooth exponential decay** with time constant ~32 ms — exactly the
  `1/(2π·5 Hz)` HPF relaxation of the 5 Hz DC blocker.

The orthogonal control (compile with `--noise off`, inject the same
white-noise energy as external input, stop input, sample tail) produces
the same decay shape — confirming the residual is the DC-blocker's
expected transient response to any broadband-input cessation, not a
noise-specific bug. The plugin-side DAC reconstruction filter handles it
identically to any other zero-input transient.

What may have been fixed between the Phase 1 note and this recheck:
either `232ec5f` (stiff-circuit nodal auto-promotion to backward Euler on
`spectral_radius(S*A_neg) > 1.002`, though the RC lowpass is not stiff
enough to trigger it) or the original observation conflated DC-blocker
HPF settling with a Nyquist limit cycle. No solver fix is needed.

If a specific circuit ever does show genuine Nyquist-rate residual (flip
fraction approaching 1.0), the mitigation ladder is unchanged — check
stiffness first (`--force-trap` off, i.e. auto-BE on), then consider a
1-pole lowpass on the per-sample Gaussian at `fs/4` before scaling.

### Constant derivation — why `8`, not `4` or `2`

Physical one-sided PSD of a resistor's Norton current noise is
`S_i(f) = 4·k_B·T/R  [A²/Hz]` over `[0, fs/2]`, so a naïvely-calibrated
discrete-time white sequence would use per-sample variance
`σ² = S_i · fs / 2 = 2·k_B·T·fs/R` — the textbook "2kT" form.

But melange stamps into the DK-trap MNA equation
`A · v_new = A_neg · v_prev + sources`, where `A = G + (2/T)·C` and
`A_neg = (2/T)·C - G`. At steady state `(A - A_neg) = 2G`, so a *constant*
current-source stamp of `I` yields `2G · v_ss = I`, i.e., half the
continuous-time DC gain. The input stamp compensates by double-stamping
`(V_new + V_prev)·G_in` (so a DC step gets `2V·G_in` and the expected
continuous response). The noise stamp, however, uses a single per-sample
Gaussian draw and therefore sees the factor-of-½ gain directly.

Two equivalent fixes:

1. Stamp `i_n[n+1] + i_n_prev` (two independent draws per sample — variance
   of the sum is `2σ²`, matching the input-stamp symmetry).
2. Stamp a single draw with variance `2σ²` — equivalent statistics, one
   fewer RNG call. **Chosen for Phase 1.**

Under fix (2), per-sample variance becomes `σ²' = 2 · 2·k_B·T·fs/R =
4·k_B·T·fs/R`… which, re-expressed via the `sqrt(X·k_B·T·fs/R)` template
used throughout the emitter, is `X = 8`. Hence `8.0`.

Validation: `tests/noise_psd_validation.rs` asserts the output variance
of a 10 kΩ / 100 nF RC lowpass with zero input matches
`k_B·T/C ≈ 4.00e-14 V²` to within ±15 % — the R-independent Nyquist kTC
equilibrium that falls out of this formula if and only if the per-sample
variance is correct.

## Why Phase 1 alone still beats the field

Even before shot / 1/f / en-in ship, melange thermal noise:

1. Sees the full transfer function of every downstream stage automatically
2. Interacts with nonlinearities (a hot tube stage amplifies its own cathode-R
   hiss through the correct bias-shifted gain curve)
3. Temperature-scales correctly (cold gear is quieter — nobody else models this)
4. Has per-resistor independence (no shared LFO or correlated hiss across
   channels)
5. Is zero-cost at compile time when off — users who don't want it pay
   literally nothing

Phase 1 is already the best-in-class analog-noise baseline. Phases 2–5 pull
further ahead.

## Next phase starting points (for a fresh agent)

Pick one. Each is independently shippable.

### 1. Nodal codegen path (tube-amp circuits)

**Why**: Pultec, Plexi, 4kbuscomp, Neve 1073, etc. all route to the nodal
codegen path (`emit_nodal` in `rust_emitter/nodal_emitter.rs`). Without this
hook-up, `--noise thermal` on those circuits is a silent no-op.

**Where**: `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs`.
Two process_sample paths (Schur and full-LU); both build RHS inline rather
than calling a separate `build_rhs` function. Inject the noise stamp right
after RHS construction, before the first NR evaluation — one sample per
audio-sample, NOT per NR iteration.

**Re-use**: `build_noise_emission(ir)` already produces
`noise.rhs_stamp` — a self-contained code fragment that mutates `rhs` and
`state.noise_rng`. The same string drops into nodal paths unchanged if you
emit the RNG helpers + state fields once (via the existing
`noise.top_level` + the state-template wiring), then inject
`noise.rhs_stamp` inline in each nodal process_sample variant. Expect
30-60 LOC of template/emitter plumbing, not new math.

**Test**: add a Pultec `--noise thermal` test that asserts NOISE_THERMAL_N
matches resistor count, same as the DK test.

### 2. Dynamic-resistor noise (`.pot` / `.switch`)

**Why**: Phase 1 skips `.pot`-marked resistors because their R is runtime-
variable and the coefficient `sqrt(1/R)` baked at codegen time is stale
after a pot change. Users with potted circuits (tube screamer, Pultec)
currently get no noise contribution from their pots.

**Where**: `collect_thermal_noise_sources` in `codegen/ir.rs`; the emitted
`set_pot_N` / `set_switch_N` methods in `dk_emitter.rs`.

**Recipe**:
1. Don't skip pot/switch resistors in the collector — add them with their
   default R, plus a flag `is_dynamic: bool`.
2. For each dynamic source, emit a per-source `noise_thermal_sqrt_inv_r[k]`
   state field (instead of const array entry).
3. In `set_pot_N(&mut self, r: f64)`, after any existing matrix rebuild,
   recompute `self.noise_thermal_sqrt_inv_r[k] = (1.0/r).sqrt()` for the
   affected source.
4. Add a test that verifies `set_pot_0` changes the thermal-noise
   coefficient.

### 3. Trap-rule + DC-blocker Nyquist sustain

**Why**: Noted under "Known Phase 1 observations" — a persistent Nyquist-
rate component of order 0.1-2 mV lingers for ~30 ms after `set_noise_enabled(false)`. Not noise-specific but amplified by it.

**Where**: First investigate — may not be noise's fault. Repro with a
signal-path broadband transient on the same circuit. If it reproduces
without noise, the fix lives in the DC blocker / trap-rule path, not the
noise stamp.

**Possible fix (noise-specific)**: 1-pole lowpass on the Gaussian output
before multiplying by the noise coefficient. Targets fs/4, trades very-HF
fidelity (which is physically questionable at audio rates anyway) for
clean time-domain decay.

### 4. Phase 2 (shot noise)

**Why**: Shot noise is the killer differentiator — current-dependent noise
amplitude means loud passages produce more noise, modulated through the
correct Jacobian-shaped transfer function.

**Where**: Extend `NoiseIR` with `shot_sources: Vec<ShotNoiseSource>`.
Populate by scanning `ir.device_slots` for diode/BJT/JFET/MOSFET/tube
device slots. Each device contributes one or two shot sources
(see "Shot (Junction) Noise — Phase 2" section above).

**Per-sample amplitude**: `sqrt(2·q·|I_prev|·fs)` where `I_prev` comes
from `state.i_nl_prev[slot_idx]`. Inject at the device's Norton-equivalent
nodes (anode/cathode, collector/emitter, etc.).

**Gotcha**: use `|I|` (magnitude) — shot noise doesn't care about current
direction.

## Gotchas recorded from Phase 1 (do not re-hit)

- Noise stamp **can't go in `build_rhs`** (takes `&CircuitState`). Goes in
  `process_sample` where `&mut state` is available.
- `set_temperature_k` **must read `state.noise_fs`**, not compile-time
  `SAMPLE_RATE`. First version used `SAMPLE_RATE` and produced wrong RMS
  at non-default sample rates.
- `Xoshiro256pp.s == [0;0;0;0]` is forbidden — `seed_noise_rngs` clamps
  `s[0] = 1` if SplitMix64 ever emits all-zeros.
- `reset()` re-seeds RNG from `self.noise_master_seed` but does NOT reset
  `noise_enabled` / `*_gain` / `temperature_k` — those are user
  preferences, not transient state.
- Noise ships on **DK path only in Phase 1**. Circuits that route to
  nodal codegen (tube amps with transformers, Pultec, etc.) get no noise
  until Phase 1.5 Step 1.
