# Authentic Circuit Noise

> **Status addendum (2026-07-18) — noise-physics correction wave.**
> Six verified physics bugs fixed in one pass (all validated in
> `tests/noise_psd_validation.rs`; kTC anchors untouched and green):
> 1. **Junction flicker recalibrated** — was ~3 decades hot and
>    fs/OS-dependent (white-input σ² ∝ fs through a fixed digital pink
>    filter ⇒ output PSD ∝ fs; plus the "0.11 tail normalizes to ~unit
>    RMS gain" premise was false — measured white power gain ≈ 0.113).
>    New fs/OS-invariant calibration lands the output at the ngspice
>    semantics `S_i(f) = KF·I^AF/f` (one-sided). See "Flicker calibration
>    (2026-07-18)". At 96 kHz this drops junction flicker by ~30.6 dB
>    (÷1160 in power); the level is now *physical*, not vibes.
> 2. **Resistor flicker** — same fs bug, plus it was missing the trap
>    stamp compensation the other phases carry (the Phase 3.5 claim that
>    it "is not stamped through the trap companion path" was FALSE — it
>    is a Norton RHS stamp identical to junction flicker's). Now shares
>    the exact same calibration; `KF` remains the empirical Hooge knob
>    but is now literally `S_i·f/I^AF`, stable across fs.
> 3. **Flicker Nyquist anti-alias pair** (both flicker phases, trap
>    builds): `i_n = 0.5·amp·(pink[n]+pink[n-1])`. The Kellett cascade
>    leaves only ~−14 dB at fs/2; on resistive junction nodes the trap
>    z=−1 pole amplified that tail by ~+60 dB and the device
>    nonlinearity rectified/intermodulated it into the audio band
>    (measured ×2400 PSD inflation at 1 kHz on a biased-diode bench —
>    the deferred "lower-priority candidate" from 2026-04-24 surfaced).
>    Pair-sum zeroes fs/2, leaves the audio band unchanged
>    (cos²(πf/fs) ≥ −0.05 dB below 5 kHz).
> 4. **BJT parasitic RB/RC/RE thermal noise now actually collected**
>    (nodal internal-node path) — the old "Resistor selection" claim
>    that parasitics "appear as regular R elements after MNA expansion"
>    was false; they never exist as `Element::Resistor`. rbb′ is the
>    classically dominant BJT voltage-noise term. DK path (K_eff
>    absorption) is skipped honestly with a `log::warn!` — see "BJT
>    parasitic-R thermal noise".
> 5. **Triode plate shot space-charge smoothing** (van der Ziel /
>    Thompson-North-Harris `R_eq ≈ 2.5/gm`): `Γ² = 10·k·T₀·gm/(2·q·I_p)`
>    at the DC OP replaces bare full shot. A 12AX7 CE stage measures
>    Γ² ≈ 0.234 (−6.3 dB plate hiss; e_n ≈ 4.7 nV/√Hz, R_eq ≈ 1.4 kΩ —
>    textbook). `.model TUBE(SHOT_GAMMA2=…)` overrides; `1.0` restores
>    the legacy full-shot emission byte-identically.
> 6. **FA-reduced BJT base shot restored** — the 1D reduction folds
>    `Ib = Ic/BF` into the *deterministic* stamping but never carried the
>    base junction's independent shot statistics. New source at
>    (base, emitter), Γ² = 1/BF.
> Also: op-amp en G-diag refresh is now an **absolute recompute**
> (`refresh_opamp_en_g_diag()`, immune to FP drift) and `.switch` R at
> in+ finally hooks it (previously only pots did).

> **Status** (2026-04-20)
> - **Phase 1 shipped** (`25b61ba`). Johnson-Nyquist thermal noise on
>   fixed resistors via MNA-RHS Norton-current stamping, DK codegen path.
>   Runtime `set_noise_enabled(bool)` + gain/temperature/seed controls.
>   Default `--noise off` → byte-identical to pre-feature codegen.
> - **Phase 1.5 Step 1 shipped** (`2b09cc3`). Nodal codegen path — `build_noise_emission` wired into `emit_nodal` end-to-end; kTC theorem
>   holds on the nodal path with the same calibration constant.
> - **Phase 1.5 Step 2 shipped** (`1fdcac2` + `a282217`). Dynamic
>   resistors — `.pot` / `.wiper` / `.runtime R` / `.switch` R members
>   are now thermal noise sources; the per-sample `sqrt(1/R)` coefficient
>   lives in `state.noise_thermal_sqrt_inv_r[k]` and is refreshed inside
>   each emitted `set_pot_N` / `set_runtime_R_<field>` / `set_switch_N`
>   setter.
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
>   lagged — same 2× trap-MNA calibration as thermal. **Two-draw Nyquist
>   anti-alias added 2026-07-19** (`i_n = w[n]+w[n-1]`, per-draw
>   `sqrt(4·q·|I|·fs)·0.5`, `noise_shot_w_prev` state) after the Noyce
>   Zener source revealed that single-draw shot rings the trap z=−1 pole
>   on stiff reverse-breakdown junctions; BE-primary stays single-draw.
>   See "Nyquist anti-aliasing" shot bullet. Runtime `set_shot_gain(f64)`;
>   salted RNG streams (`NOISE_SHOT_SALT`) so thermal and shot never share
>   a prefix. Available via `--noise shot` or `--noise full`.
> - **Phase 3 (1/f flicker) shipped** 2026-04-20. Per-junction flicker
>   sources using Paul Kellett's 7-pole pink filter — same per-device
>   port layout as shot (Diode 1, BJT 2 / forward-active 1, JFET/MOSFET
>   1, Tube 1). Opt-in via `.model NAME TYPE(KF=… AF=…)`; devices with
>   `KF=0` (the default) contribute no flicker source, so zero-KF builds
>   emit byte-identical code to pre-Phase-3. Per-sample amplitude
>   ~~`kellett(sqrt(4·KF·|I_prev|^AF·fs) · N(0,1))`~~ → recalibrated
>   2026-07-18 to the fs-invariant
>   `kellett(sqrt(2·KF/K_pink) · |I_prev|^(AF/2) · N(0,1))` with a
>   Nyquist pair-sum on the pink output (see status addendum). Runtime
>   `set_flicker_gain(f64)`; salted stream `NOISE_FLICKER_SALT`.
>   Available via `--noise full`.
> - **Phase 3.5 (resistor 1/f flicker) shipped** 2026-05-14. Per-resistor
>   flicker via standard Hooge bias-squared form: per-sample amplitude
>   ~~`kellett(sqrt(KF·fs) · |I_R|^(AF/2) · N(0,1))`~~ → recalibrated
>   2026-07-18 to the same fs-invariant
>   `kellett(sqrt(2·KF/K_pink) · |I_R|^(AF/2) · N(0,1))` + Nyquist pair
>   as junction flicker (see status addendum) where
>   `I_R = (V_+ − V_−)/R` is read live from `state.v_prev`. Default
>   `AF = 2.0` (Hooge's exponent for resistors; junctions kept their
>   `AF = 1.0` default). **Bias-squared, not bias-independent** — a
>   resistor with no current carries only thermal, matching every
>   published noise-index measurement. Per-element opt-in:
>   `R1 a b 10k KF=1e-10 AF=2.0`; resistors without `KF` produce no
>   flicker source and emit byte-identical code to pre-Phase-3.5 builds
>   under `--noise full`. Same Kellett filter / runtime gain
>   (`set_flicker_gain` shared with junction flicker) / BE-fallback
>   replay / dynamic-R refresh (`.pot` / `.wiper` / `.runtime R` /
>   `.switch`) as the junction path. Salted stream
>   `NOISE_R_FLICKER_SALT = 0xCA12_B0CC_F11C_E12E`. Available via
>   `--noise full`.
> - **Constants resolved**: thermal `sqrt(8·k_B·T·fs/R)`, shot
>   `sqrt(Γ²)·sqrt(4·q·|I|·fs)` (Γ² = 1 for plain junctions), flicker
>   white input `sqrt(2·KF/K_pink)·|I|^(AF/2)` pre-Kellett cascade
>   (fs-INVARIANT — superseded the original `sqrt(4·KF·|I|^AF·fs)` on
>   2026-07-18; see the status addendum and "Flicker calibration").
>   Thermal/shot carry the 2× trap-MNA amplitude compensation in the
>   `8`/`4`; flicker carries it in the `2` (vs the physical `0.5`).
> - **Tested**: emission-assertion tests in
>   `crates/melange-solver/tests/codegen_verification_tests.rs` + 37
>   tests in `tests/noise_psd_validation.rs` (DK kTC, nodal kTC,
>   dynamic-pot/switch first-sample-divergence, shot-noise
>   audible-wiring check, Kellett filter slope, flicker wiring, and the
>   2026-07-18 additions: flicker ABSOLUTE level + fs/OS invariance,
>   flicker-vs-shot corner, rbb′ vs explicit-resistor equivalence,
>   triode shot smoothing + SHOT_GAMMA2 override, FA base-shot
>   presence, `.switch`-at-in+ en refresh, K_pink stability).
> - **Phase 5 (pentode partition) shipped** 2026-05-17. Per-pentode
>   plate-current noise via Schottky 1918 partition statistics —
>   `S_i(plate) = 2·q · I_p · I_s / (I_p + I_s)` **replaces** the Phase 2
>   bare plate-shot stamp for pentodes (the shot collector filters
>   plate ports for 4/5-node Tube devices; triodes keep full shot). Per-
>   sample amplitude `sqrt(4·q · I_p·I_s/(I_p+I_s) · fs) · PARTITION_F`
>   with the same 2× trap-MNA compensation and two-draw Nyquist anti-alias
>   as thermal/shot. `.model TUBE(PARTITION_F=…)` is a process-variation
>   knob (default 1.0 textbook; ~0.6 for selected low-noise EF86 batches);
>   the dominant control over pentode noise floor is the bias network in
>   the `.cir` (sets `I_s/I_p`). Reuses `noise_shot_scale`, `shot_gain`,
>   and `set_shot_gain` (partition is shot at the screen-divert barrier;
>   one mute call silences both). Salted stream `NOISE_PARTITION_SALT =
>   0x9E27_0DE9_A271_710E`. BE-fallback replay, NaN recovery, and `reset()`
>   /`set_seed()` handle partition state the same as shot. Available via
>   `--noise full`. Zero codegen / zero state for triode-only or
>   passive-only circuits — byte-identical to pre-Phase-5 builds.
> - **Phase 4 (op-amp en/in) shipped** 2026-05-17 (v1 white-only).
>   Per-op-amp three Norton streams via `.model OA(EN=… IN=…)`:
>   - **en** at `n_plus_idx`, amp `EN · noise_opamp_en_g_diag[k] · sqrt(2·fs)`
>     (Norton equivalent of voltage-source-in-series-with-input — no netlist
>     resistor inserted; uses the existing G-matrix diagonal at in+).
>   - **in+** at `n_plus_idx`, amp `IN · sqrt(2·fs)`.
>   - **in-** at `n_minus_idx`, amp `IN · sqrt(2·fs)`.
>   All three use two-draw Nyquist anti-alias and the trap-MNA 4× PSD
>   compensation folded into `sqrt(2·fs)`. New runtime knob
>   `set_opamp_input_gain` — signal-independent, distinct from `shot_gain`
>   per the Noyce response-letter UX. Salts: `NOISE_OPAMP_EN_SALT =
>   0x0FA3_94E5_E700_5A17`, `NOISE_OPAMP_IN_SALT = 0x0FA3_94E5_1700_5A17`
>   (2N stream array — `2k` for in+, `2k+1` for in-). `noise_opamp_en_g_diag[k]`
>   is initialized to the static `G[in+, in+]` and refreshed by
>   `refresh_opamp_en_g_diag()` — since 2026-07-18 an ABSOLUTE recompute
>   (`NOISE_OPAMP_EN_G_BASE[k]` + Σ live dynamic conductances at in+),
>   called from every `set_pot_N` / `set_runtime_R_<field>` **and**
>   `set_switch_N` whose element touches in+. (Replaced the incremental
>   `+= 1/r − 1/r_old` accumulation, which drifted in FP over unbounded
>   knob rides; the `.switch` hook was previously missing entirely.)
>   `EN_FC`/`IN_FC` parameters are parsed and stored on `OpampInfo` /
>   `OpampNoiseSource` but **not yet wired** in v1 — v1 is white-only;
>   Kellett-pink 1/f blend is reserved for follow-up. Zero codegen / zero
>   state for op-amps without EN/IN — byte-identical to pre-Phase-4 builds
>   under `--noise full`. Available via `--noise full`. Runtime smoke test:
>   `tests/noise_psd_validation.rs::opamp_noise_emission_compiles_and_runs`.
>
> - **All five noise phases shipped**. NoiseIR now carries six per-phase
>   source vectors (thermal, shot, junction-flicker, resistor-flicker,
>   pentode-partition, op-amp-en-in); `build_noise_emission` is the
>   single source of truth for both DK and nodal codegen paths.

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

Master seed 0 at `CircuitState::default()` → seeded from the system clock
(`std::time::SystemTime::now()` nanoseconds since epoch — checked against
the generated code 2026-07-18; the original design note said `getrandom`,
but the emitted code never used it: no extra dependency in generated
crates, and clock entropy is entirely adequate for hiss). Any nonzero
value → deterministic. `set_seed(master)` re-derives all streams.
Reproducible renders available; nobody pays for them unless they opt in.

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
  using block-diagonal M indices. **Assumption documented 2026-07-18**: the
  `Ic` and `Ib` shot sources are treated as statistically INDEPENDENT
  streams. Physically both currents originate from the same emitter
  injection, so their shot fluctuations are partially correlated
  (`Ic = Ie − Ib`); the standard SPICE noise model makes the same
  independence approximation (separate `2qIc` and `2qIb` generators), and
  at BF ≫ 1 the correlation term is O(1/BF) — below measurement floor
  for audio circuits. Do not "fix" this without a citation.
- BJT forward-active (1D): TWO sources — `Ic` at (C, E), full shot, plus a
  base-shot source at (B, E) with `Γ² = 1/BF` reading the same Ic slot
  (`S_i = 2·q·Ic/BF = 2·q·Ib`). Added 2026-07-18: the FA reduction folds
  `Ib = Ic/BF` into the *deterministic* N_i stamping, which never carried
  the base junction's independent shot statistics — the old comment
  claiming it did was false, and FA circuits silently lost base shot.
- JFET/MOSFET: inject at (drain, source) for `Id`; gate shot ≈ 0 for MOS (good
  approximation), nonzero for JFET reverse-biased gate leakage
- Tube triode: inject at (plate, cathode) for `Ip` — **space-charge
  smoothed** since 2026-07-18, see below. (Grid current shot, when a grid
  port ever ships, stays full — emission/ion current is not
  space-charge-limited.)
- Tube pentode: plate handled by Phase 5 partition, not bare shot.

Sign of `|I|`: always take magnitude. Shot noise doesn't care about direction;
it's the granularity of charge-carrier flow. Use `i_nl_prev` from the
previous sample (one-sample lag; at 48-192 kHz this is below the audible
modulation threshold).

#### Triode plate shot — space-charge smoothing (2026-07-18)

A space-charge-limited vacuum diode/triode does NOT show full Schottky
shot: the potential minimum in front of the cathode regulates emission
statistics. Bare `2·q·I_p` overstates a small-signal triode's plate hiss
by ~4–8 dB. Melange uses the standard audio-engineering formulation —
the **equivalent noise resistance** `R_eq ≈ 2.5/gm` referenced to
`T₀ = 290 K` (B.J. Thompson, D.O. North, W.A. Harris, "Fluctuations in
Space-Charge-Limited Currents", RCA Review, Jan 1940 (triode result
R_eq ≈ 2.5/gm); A. van der Ziel, *Noise*, Prentice-Hall 1954, §14):

```
e_n² = 4·k·T₀·R_eq          [V²/Hz]  input-referred
S_i  = e_n²·gm² = 10·k·T₀·gm [A²/Hz]  at the plate
Γ²   = S_i / (2·q·I_p) = 10·k·T₀·gm / (2·q·I_p)   (clamped to (0, 1])
```

The emitter (`resolve_shot_gamma2`, dk_emitter.rs) evaluates `gm` at the
DC operating point (central difference of the Koren plate current at the
OP Vgk/Vpk from `ir.dc_operating_point` / `ir.dc_nl_currents`) and bakes
`sqrt(Γ²)` into `NOISE_SHOT_GAMMA_AMP`. The stamp still tracks the live
`|i_nl_prev|`, so signal-dependent shot modulation survives; only the Γ²
*ratio* is frozen at the OP (its drift over the swing is second-order).
Degenerate OPs (cutoff, gm ≤ 0, missing DC data) fall back to Γ² = 1.

Measured on the 12AX7 CE reference stage: `Γ² = 0.234` (−6.3 dB),
`R_eq = 1.40 kΩ`, `e_n = 4.7 nV/√Hz` — textbook 12AX7 values. Validated
by `noise_psd_validation.rs::triode_plate_shot_space_charge_smoothing`
(independent test-side OP solve agrees with the emitted Γ² to 5 decimal
places; runtime variance ratio matches Γ² exactly under a shared seed).

**Override**: `.model TUBE(SHOT_GAMMA2=…)` sets Γ² explicitly (process
knob / A-B tool). `SHOT_GAMMA2=1.0` restores the pre-2026-07-18 bare
full-shot emission **byte-identically** (no gamma const emitted).
Equivalent-Γ² note for the van der Ziel form
`S_i = 4·k·T_c·Γ_vdZ²·gm, T_c ≈ 0.6·T_cath`: at oxide-cathode
temperatures both formulations land within ~2 dB; melange ships the
R_eq form because it is the one audio literature quotes R_eq values for.

### Flicker (1/f) Noise — Phase 3 (shipped 2026-04-20)

ngspice's `KF`, `AF` model parameters. PSD shape:

```
S_id(f) = KF · I_d^AF / f           [A²/Hz] for MOSFET
S_ib(f) = KF · I_b^AF / f           [A²/Hz] for BJT
```

**Filter**: Paul Kellett 7-pole pink filter (musicdsp "pk3" variant,
±0.05 dB over 9.2 octaves at 96 kHz). Per-source state: 7 filter floats,
zeroed at `default()` and `reset()`. **Corrected 2026-07-18**: the
`* 0.11` output tail does NOT normalize the cascade to unit gain — the
tailed cascade's white power gain is ≈ 0.113 (RMS gain ≈ 0.336). The
absolute level is calibrated through the analytic `K_pink` constant
instead (next paragraph); do not retune 0.11 in isolation.

**Per-sample injection** (recalibrated 2026-07-18; see "Flicker
calibration" below for the derivation): for flicker source `k` at sample
`n`, trapezoidal build,
```
white_k   = N(0, 1)                                     // xoshiro256++ + Marsaglia polar
amp_k     = sqrt(2·KF_k / K_pink) · |i_nl_prev[slot_k]|^(AF_k/2)   // fs-INVARIANT
w_k[n]    = 0.5 · amp_k · kellett_pink(white_k, state_k)
i_flicker = w_k[n] + w_k[n-1]                           // Nyquist anti-alias pair
```
`K_pink = kellett_pink_normalized_gain()` ≈ 6.0e-3 (analytic, computed at
codegen time in `ir/noise.rs`). The `2` (vs the physical `0.5`) carries
the trap-MNA ×2-amplitude compensation; BE-primary builds use
`sqrt(0.5·KF/K_pink)` single-draw (no pair — BE damps z = −1 itself).
The pair-sum multiplies the output PSD by `cos²(πf/fs)`: unity in the
audio band (−0.02 dB at 1 kHz / 48 kHz), zero at fs/2 — required because
the Kellett cascade leaves only ~−14 dB at Nyquist, which the trap z=−1
pole on resistive junction nodes otherwise amplifies by tens of dB and
the device nonlinearity folds into the audio band (measured ×2400 PSD
inflation on a biased-diode bench before the pair landed).
Per-device constants `NOISE_FLICKER_SQRT_KF[k]` and
`NOISE_FLICKER_HALF_AF[k]` are baked at codegen to keep the hot loop
branch-free (`i_abs.powf(half_af)` is one `powf` per source per sample).

**Flicker calibration (2026-07-18)** — why the white input variance must
be fs-independent:
```
white input :  S_white(two-sided) = σ² / fs                       [A²/Hz]
Kellett gain:  |H(ν)|² ≈ K_pink / ν   for ν = f/fs (pink band)
pink output :  S_out(f) = (σ²/fs) · K_pink·fs/f = σ² · K_pink / f  (fs cancels)
choose      :  σ² = KF·I^AF / (2·K_pink)
            ⇒  S_out = KF·I^AF/(2f) two-sided = KF·I^AF/f one-sided  ✓ ngspice
```
then apply the same trap/BE stamp compensation as every other phase
(trap ×4 variance ⇒ σ² = 2·KF·I^AF/K_pink; BE ×1). The superseded
`σ² = 4·KF·I^AF·fs` rule is the correct construction for WHITE phases
(whose PSD is σ²/fs) but wrong through a fixed digital pink filter — it
made the output PSD proportional to fs (×2 per fs octave, OS-dependent)
and, combined with the wrong unit-gain assumption about the 0.11 tail,
about `2·K_pink·fs` ≈ **×1160 (+30.6 dB) hot at 96 kHz**. Sanity anchor:
for silicon KF = 1e-15, AF = 1, the 1/f-vs-shot corner is
`f_c = KF·I^(AF−1)/(2q) = KF/(2q) ≈ 3.1 kHz` — single-digit kHz at
mA-class currents, as it should be. Validated by
`junction_flicker_absolute_level_and_fs_os_invariance` (absolute ±3 dB
at 48 k / 96 k / 48 k+2×OS, pairwise ±1 dB) and
`flicker_corner_vs_shot_matches_kf_over_2q`.
Band note: `K_pink` is averaged over ν ∈ [1e-3, 1e-1] (ripple < 0.1 dB);
below ν ≈ 2e-4 the Kellett cascade sags 1–2 dB vs ideal 1/f, so at 4× OS
the sub-40 Hz flicker tail is slightly under-modeled. Audio band is
unaffected.

**Per-device layout** (mirrors shot):
- Diode: 1 source at (anode, cathode), slot = start_idx.
- BJT (2D): 2 sources — Ic at (C, E), slot=start_idx; Ib at (B, E),
  slot=start_idx+1.
- BJT forward-active (1D): 1 source at (C, E).
- JFET / MOSFET: 1 source at (drain, source), slot=start_idx.
- Tube: 1 source at (plate, cathode), slot=start_idx.
- VCA: skipped.

**Opt-in**: `KF` defaults to `0` in the parser. A device with `KF=0` is
**not** added to `NoiseIR.flicker_sources`, so no per-source constants,
no state fields, no rhs-stamp loop iterations, no cost. A circuit where
no `.model` supplies `KF` produces byte-identical code to a pre-Phase-3
build under `--noise full`. `AF` defaults to `1.0` (ngspice convention).

**Runtime**: `set_flicker_gain(f64)` mutes/attenuates flicker without
touching thermal or shot. Salted stream `NOISE_FLICKER_SALT =
0xC0DE_BABE_DEAD_BEEF` guarantees no prefix overlap with
`NOISE_SHOT_SALT` or the unsalted thermal streams under a deterministic
master seed. `reset()` zeros the Kellett filter state (settles in a few
samples; no audible reset thunk).

**Device-class notes**: germanium BJTs and DHTs have audible 1/f —
typical `KF=1e-13` to `1e-11`, corner 10–100 kHz. Silicon BJTs: `KF ≈
1e-16` to `1e-14`, corner 1–10 kHz. JFETs are low-noise: `KF ≈ 1e-18` to
`1e-16`.

### Resistor Flicker (Hooge bias-squared) — Phase 3.5 (shipped 2026-05-14)

Hooge's empirical law for resistor 1/f noise:

```
S_V(f) = α_H · V_DC² / (N · f)       [V²/Hz]
S_I(f) = α_H · I_DC² / (N · f)       [A²/Hz]   ⇒   AF = 2 in ngspice notation
```

Strongest in granular composites (carbon-comp, ~α_H ≈ 1e-3); weaker in
metal-film; weakest in wire-wound. **Bias-squared**: a resistor with no
current emits zero excess 1/f. Below ~10 mV DC drop the excess sits below
the thermal floor and cannot be measured — this is the standard
noise-index methodology constraint.

**Why this matters audibly.** In real audio gear (vintage tube preamps,
guitar drives, RIAA stages), the resistors that *audibly* hiss are the
ones carrying bias current — cathode resistors, plate loads, drive-stage
input-bias networks. Coupling-network resistors with literal zero bias
do not measurably hiss extra over metal-film equivalents. The Hooge form
captures this exactly: loud passages drive more current through the bias
network → louder 1/f, quiet passages let the resistors return to thermal
floor. This is the "compresses and breathes" character of vintage gear.

**Per-element syntax** (Phase 3.5 chose per-element over `.model R(…)`
because Hooge constants are per-resistor material properties, not shared
classes):

```spice
R1 a b 10k KF=1e-10 AF=2.0    ; carbon-comp on a tube cathode network
R2 a b 22k                    ; metal-film, KF default 0 (no flicker)
R3 a b 47k KF=1e-13           ; AF defaults to 2.0 when omitted
```

`KF` and `AF` are case-insensitive, order-independent. `KF = 0` and
`KF` unset are equivalent — both produce no flicker source. AF without
KF is stripped at parse time. The parser rejects `KF < 0` and `AF ≤ 0`.

**Per-sample injection** for resistor `k` between nodes `(i, j)` with
resistance `R_k` and parameters `KF_k`, `AF_k`:

```
i_R_prev[k] = (v_prev[i_k] − v_prev[j_k]) / R_k    // live current, one-sample lag
white_k     = N(0, 1)                              // xoshiro256++ + Marsaglia polar
amp_k       = sqrt(2·KF_k / K_pink) · |i_R_prev[k]|^(AF_k / 2)   // fs-INVARIANT
w_k[n]      = 0.5 · amp_k · kellett_pink(white_k, state_k)
i_flicker   = w_k[n] + w_k[n-1]                    // Nyquist anti-alias pair
```
(2026-07-18 recalibration — identical construction to junction flicker,
including the trap ×2-amplitude compensation and the Nyquist pair;
BE-primary uses `sqrt(0.5·KF/K_pink)` single-draw. Output PSD lands at
`S_i(f) = KF·I_R^AF / f` one-sided, fs/OS-invariant.)

Per-source baked constants `NOISE_R_FLICKER_SQRT_KF[k] = sqrt(KF_k)` and
`NOISE_R_FLICKER_HALF_AF[k] = AF_k / 2`. The scale constant lives in the
state field `noise_r_flicker_sqrt_fs` — **legacy name**: it held
`sqrt(fs)` before 2026-07-18 and now holds the fs-independent
`sqrt(2/K_pink)`; nothing recomputes it in `set_sample_rate` anymore.
The live `1/R` lives in `noise_r_flicker_inv_r[k]` (refreshed by
`set_pot_N` / `set_runtime_R_<field>` / `set_switch_N` for dynamic R).

**Retraction (2026-07-18)** — the original Phase 3.5 claim that resistor
flicker "is not stamped through the trap-MNA `(A − A_neg) = 2G`
companion path" was **false**. The r-flicker stamp is a Norton RHS
current identical in kind to junction flicker's; it sees exactly the
same halved trap LF gain and therefore needs exactly the same
compensation. The missing factor made r-flicker 4× (−6 dB) low RELATIVE
to junction flicker at any given fs (on top of both phases' shared fs
bug). Both classes are now mutually consistent under one calibration.
`KF` on resistors remains an empirical Hooge-style knob (absolute
recalibration of shipped `.cir` values was not performed), but its
meaning is now stable: `KF = S_i·f / I_R^AF`, independent of fs.

**No temperature coupling.** Hooge bias-driven 1/f is T-independent;
`set_temperature_k` adjusts only thermal, never r-flicker. Validated by
`tests/noise_psd_validation.rs::r_flicker_is_temperature_independent`.

**Zero-current guard.** Each per-source loop iterates with
`if i_abs < 1e-15 { continue; }` — same convention as junction flicker.
Unbiased resistors skip both the RNG advance and the Kellett tick, so
they emit literally zero 1/f and burn no CPU. Validated by
`tests/noise_psd_validation.rs::r_flicker_unbiased_resistor_emits_no_excess_one_over_f`.

**Dynamic R support.** `.pot` / `.wiper` / `.runtime R` / `.switch` R
members with `KF` set automatically refresh `noise_r_flicker_inv_r[k]`
inside the corresponding setter (parallel to the existing thermal
refresh). The flicker amplitude tracks the live resistance every block.

**Salted stream**: `NOISE_R_FLICKER_SALT = 0xCA12_B0CC_F11C_E12E`,
distinct from junction flicker (`0xC0DE_BABE_DEAD_BEEF`), shot
(`0xA5A5_DEAD_BEEF_CAFE`), and thermal (unsalted). Resistor 1/f streams
share no prefix with any other phase under a deterministic master seed.

**Runtime knob shared with junction flicker.** `set_flicker_gain(f64)`
mutes/attenuates *both* Phase 3 and Phase 3.5 — one knob silences all
1/f character, which is the natural musical control. Hosts that need
finer separation can disable specific resistors at the netlist level.

**ngspice non-parity warning.** ngspice does not support resistor noise
parameters in the standard model card. Setting `KF` on a resistor in
melange generates valid melange code but breaks ngspice parity — strip
KF/AF before SPICE-validating circuits that need ngspice correlation.
Per-element jitter from `.tolerance R=…` follows the same per-element
pattern, so this is consistent with how passive variation already
breaks parity.

**Per-device layout**: 1 source per opted-in resistor at `(node_i,
node_j)`. Pots / wipers / switches that are also opted-in are still
exactly 1 source each — they're a single resistor whose value happens
to be runtime-mutable.

**KF magnitudes are empirical.** The Hooge `α_H/N` for carbon-comp is
~1e-23 dimensionless, but melange's `KF` parameterization absorbs the
`N` factor and the trap-rule scaling. Real-world calibration: pick `KF`
to hit a target dB-above-thermal at a representative DC drop, then
sweep up/down for material variants. Don't expect `KF = 1e-10` to
correspond to any specific Hooge α_H value across all R values — it's
an empirical knob that produces audible 1/f at the right scale when the
bias is real.

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
- **BJT parasitic RB/RC/RE**: collected since 2026-07-18 on the nodal
  internal-node path — see "BJT parasitic-R thermal noise" below. (The
  original claim here that parasitics "appear as regular R elements after
  MNA expansion" was false: they live on the `.model` card and are stamped
  as raw conductances / absorbed into K_eff, never as `Element::Resistor`,
  so the element walk silently missed them for two months.) `RS` (diode)
  and `RGI` (tube) are still not thermal-noise sources — same class of
  gap, smaller magnitude; extend the same pattern when they matter.

### BJT parasitic-R thermal noise (2026-07-18)

`rbb′` is the classically dominant BJT voltage-noise term
(`e_n² = 4·k·T·rb` in series with the base — it is why low-noise BJTs
advertise low base spreading resistance). `collect_thermal_noise_sources`
now emits one thermal source per parasitic R with `RB`/`RC`/`RE > 0` on
the model card:

- **Nodal path** (internal-node expansion active): the source spans the
  real (external, internal) node pair — exactly where the physical
  resistor sits. No approximation. Validated by
  `noise_psd_validation.rs::bjt_parasitic_rb_thermal_matches_explicit_base_resistor_nodal`:
  a CE stage with `.model … RB=1000` matches the same stage built with an
  explicit external 1 kΩ base resistor to 0.1 % in output noise variance
  (and exceeds the RB=0 control by ~69 % in that bias network).
- **DK path** (K_eff absorption, no internal nodes): there is no node
  pair to inject across with the existing Norton machinery, so the
  source is **skipped** and codegen logs
  `log::warn!("noise: BJT <name> parasitic RB/RC/RE thermal noise skipped …")`.
  This is an honest under-modeling of rbb′ hiss on DK-routed multi-BJT
  circuits (wurli/Neve class when they route DK). A faithful DK-side
  equivalent (base-side voltage noise → current injection across
  (base, emitter) scaled by the small-signal loop admittance at the OP)
  needs the loop Jacobian at codegen time; do it properly or not at all —
  do not fake a magnitude. Routing the circuit nodal includes the noise.

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

### Constant derivation — why `8` and the two-draw anti-alias scheme

**Step 1 — physical PSD.** A resistor's Norton current noise has one-sided
PSD `S_i(f) = 4·k_B·T/R [A²/Hz]` over `[0, fs/2]`.

**Step 2 — trap-MNA factor-of-½.** Melange stamps into the DK-trap
equation `A · v_new = A_neg · v_prev + sources`, where `A = G + (2/T)·C`
and `A_neg = (2/T)·C - G`. At steady state `(A - A_neg) = 2G`, so a
*constant* current-source stamp `I` yields `2G · v_ss = I` — half the
continuous-time DC gain. The audio-input stamp compensates by
double-stamping `(V_new + V_prev)·G_in`. The noise stamp compensates by
doubling the injected PSD: target injected PSD = `8·k_B·T/R [A²/Hz]`
across `[0, fs/2]`, so the trap-MNA halving lands at the physical
`4·k_B·T/R` at the output.

**Step 3 — discrete white calibration.** A per-sample i.i.d. sequence with
variance `σ²` has two-sided PSD `σ²/fs` flat across `[-fs/2, fs/2]`.
Matching `8·k_B·T/R` gives `σ² = 8·k_B·T·fs/R`, hence
`σ = sqrt(8·k_B·T·fs/R) = noise_thermal_scale · sqrt(1/R)`. This is the
single-draw scheme used until 2026-04-24. **The 8 (not 4 or 2) is the
trap-MNA compensation, baked once into `noise_thermal_scale`.**

**Step 4 — the Nyquist problem.** The single-draw PSD is white including
the Nyquist bin. On any MNA node without a shunt cap,
`A_neg[i][i] = -G[i][i]` gives a `z = -1` pole (eigenvalue at `fs/2`)
that accumulates injected Nyquist energy indefinitely. See the
"Nyquist anti-aliasing" section below for symptoms and reproducer.

**Step 5 — two-draw fix (shipped 2026-04-24).** Each sample stamps
`i_n[n] = w[n] + w[n-1]` with `w[n] = (scale/2)·sqrt(1/R)·g[n]`,
`g ~ N(0,1)`. Per-draw variance is

```
σ²_w = (scale/2)² · (1/R) = (8·k_B·T·fs/R) / 4 = 2·k_B·T·fs/R
```

The two-draw sum has PSD (two-sided, ω in radians/sample)

```
S_i(ω) = (σ²_w/fs) · |1 + e^{-jω}|² = (σ²_w/fs) · 4·cos²(ω/2)
```

Substituting `ω = 2πf/fs` gives `4cos²(πf/fs)`.

- **At DC (ω=0)**: `S_i(0) = 4·σ²_w/fs = 8·k_B·T/R` — matches the
  single-draw injected PSD exactly. After the trap-MNA half-gain, output
  PSD lands at the physical `4·k_B·T/R`. **kTC invariant preserved.**
- **At Nyquist (ω=π)**: `S_i(π) = 0`. **No Nyquist energy injected.**
- **At audio (ω small)**: `cos²(πf/fs) ≈ 1` (e.g. 0.99 at 10 kHz / 96 kHz),
  so audio-band PSD is essentially identical to the single-draw scheme.

`noise_thermal_scale = sqrt(8·k_B·T·fs)` is unchanged; the emitter hot
loop computes `scale_half = scale * 0.5` once. New state field
`noise_thermal_w_prev: [f64; NOISE_THERMAL_N]` holds `w[n-1]` per source;
zeroed at `default()`, `reset()`, and `set_seed()` (the latter so the
determinism contract — same seed → bit-identical noise from sample 0 —
holds across re-seeds).

Validation: `tests/noise_psd_validation.rs` asserts:
- `thermal_noise_matches_ktc_theorem` / `_nodal`: output variance of a
  10 kΩ / 100 nF RC matches `k_B·T/C ≈ 4.00e-14 V²` within ±15% on both
  DK and nodal paths.
- `thermal_noise_no_nyquist_artifact_on_resistor_only_output_node`: lag-1
  autocorrelation > -0.5 AND RMS < 100 µV on a diode + series-R circuit
  (would be ~1 mV RMS with lag-1 ≈ -1 without the fix).

### Nyquist anti-aliasing in thermal noise injection (2026-04-24)

**Problem**: Trapezoidal integration creates a Nyquist pole at every MNA
node that lacks a shunt capacitor. Specifically, for a purely resistive
node `i`, `A_neg[i][i] = -G[i][i]`, giving eigenvalue `z = -1` at
Nyquist. A single-sample white Gaussian injection at such a node excites
this pole; the energy accumulates in a stationary `(+1, -1, +1, ...)` mode.

Measured on `wurli-preamp.cir` (two-stage NPN CE amp with series output
resistor R9=6.8k): before the fix, R9's thermal noise gave 950 µV RMS
(lag-1 autocorr = -0.9999) vs the physical expectation of ~1 µV. Total
output noise 1.3 mV vs ngspice ~8 µV. The variance also scaled roughly
linearly with `fs` instead of being fs-independent (kTC invariant).

**Root cause**: The physical Norton PSD `4·k_B·T/R` is white — it contains
Nyquist energy. Injecting white Gaussian draws injects Nyquist energy.
Nodes without C have no mechanism to absorb it; it accumulates indefinitely.

**Fix**: stamp `i_n[n] = w[n] + w[n-1]` with
`w[n] = (scale/2)·sqrt(1/R)·gaussian()`. The sum's PSD is
`∝ |1 + e^{-jωT}|² = 4·cos²(ωT/2)` — exactly zero at Nyquist, ≈ flat at
audio. The `scale/2` keeps the at-DC injected PSD at `8·k_B·T/R`,
matching the single-draw scheme so kTC is preserved. Full algebra in
"Constant derivation" Step 5 above.

**Scope**: shipped on **both DK and nodal codegen paths**.
`build_noise_emission()` (`dk_emitter.rs:2725`) is the single source of
truth for the rhs_stamp; called by `emit_dk` (`dk_emitter.rs:24`) and
`emit_nodal` (`nodal_emitter.rs:422`). Both paths emit the same two-draw
fragment.

**BE-fallback noise replay (2026-04-24)**: The trapezoidal `rhs_stamp`
caches each per-source `i_n` into `state.noise_*_last_i_n[k]`. When the
trap NR fails / rings and the BE fallback rebuilds `rhs_be` from scratch,
the emitted `rhs_stamp_be` re-stamps the same cached currents into
`rhs_be` (gated on `state.noise_enabled`). This avoids audible noise
dropouts during BE cooldown windows (typically 64 samples = 1.3 ms at
48 kHz). Trap-MNA 2× compensation is left in — BE samples are ~+3 dB
hot vs strict physics (BE has no factor-of-½ on a constant stamp), but
this is bounded, rare, and well below the dominating signal that
triggered BE in the first place. The cache is also cleared in the NaN
recovery block, in `set_seed()`, and in `reset()` — same reasoning, no
stale-state replay.

**Phases 2/3 (shot, flicker)**:
- Shot: **fixed 2026-07-19** — it DID surface, exactly as predicted. Shot
  is injected at device junction terminals (anode/cathode,
  collector/emitter, etc.). The 10 pF parasitic caps are auto-inserted
  ONLY when the circuit's C matrix is entirely empty
  (`mna.rs::add_parasitic_caps`, gated at `dk.rs:265`), so any circuit
  with user-defined caps does not get them — AND even when they are
  inserted, a stiff junction defeats them: on a reverse-breakdown Zener
  the dynamic resistance is ~26 Ω (`n_vt/IBV`), so the 10 pF Cak pole sits
  at ~600 MHz, four decades above fs/2, leaving the junction node
  effectively resistor-only at Nyquist. The Noyce Zener source (reported
  by the oomox agent) showed the full signature: single-draw shot excited
  the z=−1 pole into an fs/2 limit cycle (lag-1 autocorr = −1.000) that the
  breakdown exponential rectified into the audio band — seed-dependent σ
  (13–17 dB spread), ~46 dB hot, non-Gaussian (crest ~5 dB). Now the trap
  shot stamp uses the same two-draw pair `i_n = w[n] + w[n−1]`, each draw
  at `sqrt(4·q·|I|·fs)·0.5`; LF PSD unchanged, fs/2 zeroed. New state
  `noise_shot_w_prev`, cleared at `default()` / `reset()` / `set_seed()` /
  NaN recovery; **not** emitted on BE-primary builds (BE damps z=−1;
  single-draw there). A zero-current guard sets `w_new = 0` when
  `|I| < 1e-15` so the lagged half flushes over one sample rather than
  freezing a stale draw. Guard: `noise_psd_validation.rs::
  shot_noise_no_nyquist_artifact_on_stiff_breakdown_junction` (6-seed
  Zener bench; lag-1 > −0.5 per seed is the window-independent primary
  assert). This was pre-existing since Phase 2 shot (2026-04-20), not a
  regression — the shot path simply never carried the two-draw scheme the
  thermal path got on 2026-04-24.
- Flicker: **fixed 2026-07-18** — it DID surface, exactly as predicted,
  the moment an absolute-level test biased a diode with a 10 pF-only
  junction node: the Kellett cascade's ~−14 dB fs/2 tail rang the z=−1
  pole (|H_trap(fs/2)| = T/(4C) ≈ 520 kΩ vs ~350 Ω in-band for the
  bench circuit, +63 dB) and the exponential diode rectified the
  near-Nyquist swing into `i_nl_prev`, inflating the measured 1 kHz PSD
  ×2400 (≈×14 through amplitude modulation, the rest through
  intermodulation). Both flicker phases now stamp the pair-sum
  `i_n = 0.5·amp·(pink[n] + pink[n−1])` on trap builds — LF calibration
  unchanged (cos²(πf/fs)), fs/2 zeroed. New state:
  `noise_flicker_w_prev` / `noise_r_flicker_w_prev`, cleared at
  `default()` / `reset()` / `set_seed()` / NaN recovery, not emitted on
  BE-primary builds (BE damps z=−1; single-draw there).

**Gotcha for future agents**: the kTC theorem test (`thermal_noise_matches_ktc_theorem`)
passes even without the fix because the RC test circuit has a cap at the
output node. The new `thermal_noise_no_nyquist_artifact_on_resistor_only_output_node`
test is the load-bearing guard for this class of bug.

### Backward-Euler-primary stamps (2026-07-18)

All scale constants above are calibrated for the **trapezoidal** kernel,
whose LF stamp-to-voltage gain is `(A − A_neg)⁻¹ = (2G)⁻¹` — half the
physical DC gain. The trap stamps compensate in one of two ways:

- **two-draw phases** (thermal, shot, junction flicker, resistor flicker,
  partition, op-amp en/in): the pair sum `i_n = w[n] + w[n-1]` doubles the
  LF amplitude (×2 at `z = 1`), exactly cancelling the halved kernel gain —
  mirroring the trapezoidal input stamp `(V_new + V_prev)·G_in` — and
  zeroes the fs/2 injection. The amplitude constant still carries its
  explicit ×2 (variance ×4, e.g. the `4` in `sqrt(4·q·I·fs)`); the pair's
  half-scale (`·0.5` per draw) plus the ×2-at-DC of the sum nets to the
  same LF PSD as a single draw at full scale. (Shot joined this group
  2026-07-19; before that it was single-draw and rang the z=−1 pole on
  stiff junction nodes — see the shot bullet under "Nyquist anti-aliasing".)
- **BE-primary builds**: every phase reverts to a **single draw** (no
  pair). BE damps the `z = -1` pole itself, so the `cos²(ωT/2)` envelope of
  a pair would only add a spurious −3 dB@fs/4 droop; the amplitude constant
  drops its trap ×2 compensation (e.g. shot uses `sqrt(q·I·fs)`).

A **BE-primary** build (`--backward-euler`, or auto-BE promotion on
either codegen path) has `A − A_neg = G` — **full** LF gain, 2× trap in
amplitude. Running the trap-calibrated stamps through it lands **+6 dB
hot at LF** (verified numerically against the kTC anchor). It also keeps
the two-draw `cos²(ωT/2)` envelope, which is a *spurious* −3 dB@fs/4
droop under BE — BE is L-stable and damps `z = −1` by itself; no
anti-alias pair is needed.

**Rule** (exact mirror of the DC-source branch, `rhs_const` ×1 under BE
vs ×2 under trap): *every BE-primary stamp must be half the trap stamp's
LF amplitude, delivered as a single draw*. Emitted by
`build_noise_emission` when `ir.solver_config.backward_euler` is set:

| Phase | Trap emission | BE-primary emission |
|-------|---------------|---------------------|
| thermal | two-draw, per-draw `sqrt(8kT·fs)/2 · sqrt(1/R)` | single-draw `sqrt(2·kT·fs) · sqrt(1/R)` |
| shot | two-draw, per-draw `sqrt(Γ²)·sqrt(4·q·fs)/2 · sqrt(I)` | single-draw `sqrt(Γ²)·sqrt(q·fs) · sqrt(I)` |
| junction flicker | pink pair-sum, per-draw `0.5·sqrt(2·KF/K_pink) · I^(AF/2)` | single-draw `sqrt(0.5·KF/K_pink) · I^(AF/2)` → pink |
| resistor flicker | pink pair-sum, per-draw `0.5·sqrt(2·KF/K_pink) · I^(AF/2)` | single-draw `sqrt(0.5·KF/K_pink) · I^(AF/2)` → pink |
| partition | two-draw, per-draw `sqrt(4·q·fs)/2 · …` | single-draw `sqrt(q·fs) · …` |
| op-amp en/in | two-draw, per-draw `0.5 · sqrt(2·fs) · …` | single-draw `sqrt(0.5·fs) · …` |

(Flicker rows rewritten 2026-07-18 — the original `sqrt(4·fs)` /
`sqrt(fs)` entries carried the fs-dependence bug; note flicker's BE:trap
LF ratio is the same amplitude ÷2 as every other phase, delivered via
the fs-invariant constants.)

Note the BE constants for the two-draw phases equal the **trap per-draw
amplitude** (dropping the pair halves the LF amplitude by itself); the
single-draw phases halve the amplitude in the constant (variance ÷4).
A naive "un-double the variance" (e.g. `sqrt(4kT·fs)` single-draw at
full amplitude for thermal) is **+3 dB hot** — the LF gain doubling is
an *amplitude* factor, so the correction is amplitude ÷2 = variance ÷4
relative to the trap stamp's LF-equivalent, not variance ÷2.

`set_sample_rate` and `set_temperature_k` recompute the same BE
constants. The BE-*fallback* replay (previous section) is unrelated: it
applies to trap-primary builds only and intentionally keeps the trap
calibration for its rare 64-sample windows.

**Validation**:
`noise_psd_validation.rs::thermal_noise_be_primary_matches_trap_anchor`
compiles the same RC+diode circuit twice (trap and `backward_euler:
true`) and asserts the output noise variance (LF-dominated, fc ≈ 159 Hz)
matches the validated trap anchor and the analytic kT/C.

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
