# OOMOX_CONTRACT.md — What Oomox Consumes From Generated Circuit Code

**MANDATORY reading before any melange change that touches device models, noise
generation, output levels, setter semantics, codegen structure, or lifecycle
functions.** Surveyed 2026-07-21 against oomox HEAD (`c5badb5`) after the
2026-07-18 accuracy campaign broke shipped plugins without anyone knowing the
downstream coupling existed.

## How to use this when reviewing melange

Melange's contract with oomox is much bigger than the function signatures. It has
four layers, and a change can be "correct" at one layer while silently breaking
the next:

1. **API/ABI layer** — names, signatures, constants the wrapper code compiles
   against. Breakage = oomox stops building. Loud, cheap to catch.
2. **Semantic layer** — call-order rules, threading, units, repeated-call
   behavior the wrappers assume. Breakage = clicks, stale matrices, wrong bias.
3. **Calibration layer** — ~60 hand-pinned constants (OUTPUT_TRIM, VoltageRef
   volts, drive levels, thermal gains) tuned against the *current numeric
   output* of the generated code. Any change to gain, noise magnitude,
   frequency response, or DC level invalidates them **silently** — the build
   passes, the plugin ships wrong. This is where the July 2026 damage happened.
4. **Test layer** — oomox test suites assert absolute dB levels, variance
   bands, seed spreads, autocorrelation, THD. A melange change that moves any
   measured quantity trips them (or worse, doesn't, because the bound was
   loose).

When reviewing a melange fix, the question is never only "is the new math more
accurate?" It is also: **which rows of the tables below move, by how much, and
who re-pins them?** An accuracy fix that changes output level is still correct —
but it must ship with a downstream impact list (which plugins, which constants,
which tests), per the post-campaign protocol. See §7 checklist.

Sibling artifacts: `local-docs/golden-baseline-manifest.{json,md}` (per-circuit
regen recipe + drift status) and `tools/golden-harness/` (deterministic
render/compare) — use them to *measure* the movement this document tells you to
look for.

---

## 0. Inventory — who is downstream

Oomox has **36 plugin directories**; **21 crates + 1 orphan consume melange
codegen** via **42 generated files**. `plugin-circuit-matrix.csv` in oomox
local-docs is the (slightly stale) index; the authoritative `.cir` → `.rs`
mapping is `~/dev/oomox/local-docs/noise-regen-spec.md` plus the golden-baseline
manifest.

Generated files (all paths relative to `~/dev/oomox`):

| Plugin | Generated file(s) | Notes |
|---|---|---|
| basic-bitch | `plugins/basic-bitch/src/circuit.rs` | canonical drive-pedal scaffold |
| five-watt-freddie | `plugins/five-watt-freddie/src/circuit.rs` | canonical amp scaffold |
| funkyinduct | `plugins/funkyinduct/src/circuit.rs` | 16-band passive LC EQ |
| gold-press | `src/{cab,cartridge,mastering,overdrive,riaa}.rs` | **5 circuits chained**; riaa/overdrive OS=4 |
| moonladder | `plugins/moonladder/src/circuit.rs` | |
| noyce | `src/sources/{amp_at_idle,boiler_room,carbon_comp_bank,clean_rc,ef86,germanium_cluster,jrc4558,smps_ripple,tape_head,transformer_triode,triode_12ax7,zener_junction}/circuit.rs` | **12 noise sources**, all `--noise full`, zero-input |
| periodic-pedal | `plugins/periodic-pedal/src/circuit.rs` | switches |
| pipe-shouter | `plugins/pipe-shouter/src/circuit.rs` | `.mismatch`/`.tolerance` seed 42 baked |
| pretty-baby | `plugins/pretty-baby/src/circuit.rs` | |
| qapla-1a | `plugins/qapla-1a/src/circuit.rs` | first switch plugin |
| sad-bastard | `plugins/sad-bastard/src/circuit.rs` | seed 42 baked; runtime-R envelope mod |
| series-of-tubes | `src/circuit.rs` (stage) + `src/warmth.rs` | **one stage cell instantiated 64×** (`NUM_STAGES=64`; banner text says 50) |
| subspace | `src/circuits/radio_am.rs` + `radio_fm.rs` | radio_fm **held back** at old topology |
| sus-bus | `plugins/sus-bus/src/circuit.rs` | VCA CV from sidechain |
| the-kicker | `plugins/the-kicker/src/circuit.rs` | internally-driven synth voice |
| tungsten-glow | `plugins/tungsten-glow/src/circuit.rs` | reads cathode nodes for UI |
| tungsten-thunder-horse | `src/circuit.rs` + `src/{cascade,edge}.rs` | cascade/edge used **only in `tests/perf.rs`**, not the shipped path — still gate the build |
| uniquorn | `src/circuit.rs` + `src/power.rs` | dual circuit, chained |
| vcr-audio | `plugins/vcr-audio/src/circuit.rs` | **orphan**: no Cargo.toml/wrapper, still regenerated |
| velvet-elvis | `plugins/velvet-elvis/src/circuit.rs` | |
| vurli | `plugins/vurli/src/comp/circuit.rs` | 6SL7 leveler stage; OpenWurli engine feeds it |
| warpony | `plugins/warpony/src/circuit.rs` | |

Pure-DSP (out of contract): genniertor, greenroom, hailing-frequencies,
hushpuppy, hyper-moosoleum, inertial-dampener, lobes, moosoleum,
mutilated-lips, poof-b-gone, qrtu, sensor-array, test, wonderwall.

**Even "dead" generated modules break the workspace build** (tth cascade/edge,
vcr-audio orphan): a codegen change that emits non-compiling Rust for *any*
topology in this inventory fails `cargo build --release` for the whole pack.

---

## 1. API surface consumed

Every generated file exports the same core shape (verified uniform across all
42 files). The wrapper code compiles directly against these names — renaming,
re-signaturing, or changing emission conditions is an API break.

### 1.1 Core (used by every circuit plugin)

| Symbol | Signature / value | Consumers & assumed semantics |
|---|---|---|
| `CircuitState` | struct, constructed via `CircuitState::default()` | All. Boxed (`Box<CircuitState>`) in most wrappers to keep nih-plug structs small. Default state = baked DC OP at nominal pots, **codegen-time sample rate (usually 48 kHz)** — wrappers must call `set_sample_rate` after construction (normative in `docs/best-patterns.md`). |
| `process_sample` | `pub fn process_sample(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS]` | All. **Free function, not a method.** Audio thread, per sample. Input in circuit volts; output in circuit volts, DC-blocked (5 Hz) and scaled by `OUTPUT_SCALES`. Uniform signature in all 42 files — any variation is a break. |
| `reset()` | method | All. Audio-thread-safe "factory state": restores baked `DC_OP`, `DC_NL_I`, nominal pot/switch values *together* (coherence), zeroes input_prev/history. **Intentionally discards a runtime-recomputed OP** — wrappers that recall presets re-apply pots then `recompute_dc_op()`/warmup after reset. Noyce calls `reset()` then `set_noise_enabled(true)` (reset does not preserve the noise toggle path re-arm). |
| `set_sample_rate(f64)` | method | All. Called in nih-plug `initialize()` (and per best-patterns in `reset()` flow), NOT audio thread. Semantics wrappers rely on: (a) invalid/non-finite rate = silent no-op; (b) **same-rate call is a click-free no-op reconfiguration** — restores baked matrices only if ALL pots/switches at default, else `rebuild_matrices()` at live values, transient state preserved; (c) genuine rate change rebuilds matrices, recomputes noise scales, **re-seeds DC blocker from `dc_operating_point[OUTPUT_NODES[k]]` and resets oversampler history**. |
| `warmup()` | method, 18+ files | Fixed **50-sample** zero-input loop. Some wrappers/tests call it; most implement their own longer warmup (see §3.1). Not the same thing as `WARMUP_SAMPLES_RECOMMENDED`. |
| `recompute_dc_op()` | method | **series-of-tubes only** on the runtime path (`lib.rs:201` per jittered stage at init, `:336` warmth; `tests/idle_floor.rs`). NOT audio-thread (blocking, µs–ms). Semantics SoT init *depends on*: warm-starts from `v_prev`; direct NR only; **on failure leaves state untouched and bumps `diag_nr_max_iter_count`**; nodal-path circuits: permanent stub that always "fails". SoT replaced its multi-second per-stage warmup with one exact NR solve per stage (2026-06-03) — **if a device-math change makes stage-cell NR fail at ±5% jittered pots, SoT init silently loses its settle step**. |
| `dc_op()` / `set_dc_operating_point([f64; N])` / `settle_dc_op()` / `dc_op_dump()` | methods | Emitted everywhere; **no wrapper calls them on the runtime path today** (matches inside plugin dirs are the generated files themselves). Still part of the documented surface (melange-usage.md, best-practice fallbacks) — removing them breaks the documented contract, not current builds. |
| `rebuild_matrices()` | method | Not called directly by wrapper runtime code (appears only in in-file diagnostics/tests, e.g. uniquorn's perf harness). The load-bearing path is implicit: pot/switch setters mark `matrices_dirty`, rebuild deferred into the next `process_sample` on the audio thread. |

### 1.2 Pots, switches, runtime-R

| Symbol | Consumers & assumed semantics |
|---|---|
| `set_pot_N(resistance: f64)` | 18+ plugins, N up to 31 (funkyinduct). **Units: ohms**, post-taper (oomox owns taper curves via `oomox_core::pots`). Semantics relied on: clamps to `[POT_N_MIN_R, POT_N_MAX_R]`; non-finite = no-op; `<1e-12` delta = no-op early-return; marks dirty, rebuild deferred to next `process_sample`; **reseed-free** (no NR state reset — melange 2026-04-20 setter contract; oomox smooths params and calls per-block). Doc-comment contract: "for preset recall / unsmoothed jumps, follow with `recompute_dc_op()`". Wrappers quantize resistance to 3 sig figs before calling to avoid spurious O(N³) rebuilds from smoother tails (best-patterns §"Quantize"). **Pot index assignment order (netlist order) is load-bearing**: wrappers call `set_pot_3` etc. by bare number; subspace pins indices with compile-time asserts (`lib.rs:60` `radio_fm::POT_R_RECV_INDEX == FM_POT_RECEPTION_INDEX`). Reordering pot indices in codegen silently reroutes knobs everywhere else. |
| `set_switch_N(position: usize)` | basic-bitch, gold-press, periodic-pedal, qapla-1a, sad-bastard, tth. Out-of-range = no-op; same-position = no-op; swaps component values and rebuilds. periodic-pedal masks the transition with a 128-sample mini-warmup (`SWITCH_WARMUP_SAMPLES`). |
| `set_runtime_R_<field>(ohms)` | basic-bitch, sad-bastard, series-of-tubes, subspace, gold-press, tth, uniquorn. Audio-rate-capable envelope modulation (`.runtime R` directive): **no warm re-init, envelope-safe** — but the generated doc says "Call per-block, not per-sample" for the O(N³) rebuild variants. `RUNTIME_R_*_MIN/MAX` consts are pinned by `const _: () = assert!(...)` in wrappers (best-patterns §TonePad) — changing a runtime-R range breaks compile-time asserts (good) or knob travel (bad, if no assert). |
| `set_runtime_<name>` (non-R runtime params) | subspace calls `set_runtime_strength` / `set_runtime_f_offset` at block rate with change-guards (`lib.rs:457-460,546-547`) — behavioral `.runtime` directives emit setters named after the field, not only `set_runtime_R_*`. Directive-derived identifier stability is contract. |
| Runtime **voltage** fields (`.runtime V`) | noyce smps_ripple writes `state.smps_v = <sawtooth volts>` per sample (audio thread) then calls `process_sample(0.0, ...)`. Field name derives from the `.cir` directive — renaming breaks noyce. |

### 1.3 Noise API (emitted only under `--noise ...`)

Consumers: all noyce sources (`--noise full`), plus the `noise = []` feature in
~20 plugin Cargo.tomls (`--noise shot`/`thermal`). **Emission is conditional on
the CLI flag with no warning when absent** — regenerating a noise circuit
without the flag produces a compiling, running, *silent* plugin
(`docs/melange-usage.md` documents this footgun; check for
`pub fn set_noise_enabled` after regen).

| Symbol | Consumers & assumed semantics |
|---|---|
| `set_noise_enabled(bool)` | ~24 plugins. Off = zero per-sample RNG cost. Default off; oomox opt-in policy. |
| `set_noise_gain(f64)` | ~21 plugins. Master linear scalar over all noise categories. Noyce pins 1.0 (physical); pedals expose a user knob (tth default 0.5, smoothed 150 ms). |
| `set_thermal_gain` / `set_shot_gain` / `set_flicker_gain` | Deep four + noyce validation `boot()` helpers (which pin all present gains to 1.0 — **the tests assert melange's physical calibration directly**). tungsten-glow sets `thermal_gain = 1000.0`; vurli 300.0 — tuned against current melange noise magnitudes. `set_shot_gain` covers shot AND pentode partition. |
| `set_temperature_k(kelvin)` | Deep four + noyce (user-facing temperature control, block rate, audio thread). Relied-on physics: thermal ∝ √T (290 K reference; 3 K ≈ −19.9 dB), shot sub-linear. Noyce tests assert the scaling curve. |
| `set_seed(u64)` | noyce, subspace, deep four. **0 = entropy-seeded; nonzero = deterministic, bit-identical across runs.** Noyce derives per-source seeds and asserts: same seed → bitwise identical output; different seeds → decorrelated (`|corr| < 0.1` between instances). Determinism of the generated RNG chain (Xoshiro256pp + salts) is contract. |
| `set_opamp_input_gain` | noyce jrc4558 tests (en/in op-amp noise, Phase 4). |

### 1.4 Constants read by wrappers/tests

`N`, `M` (regen anchors), `SAMPLE_RATE`, `OVERSAMPLING_FACTOR`, `INPUT_NODE`,
`NODE_*` (tungsten-glow UI reads cathode node voltages from `v_prev` via
`NODE_S1_K/S2_K/S3_K`), `OUTPUT_NODES`, `NUM_OUTPUTS`, `OUTPUT_SCALES`,
`INPUT_RESISTANCE`, `WARMUP_SAMPLES_RECOMMENDED` (defined everywhere, e.g.
240000 in vcr-audio — **but no wrapper uses it; every plugin hardcodes its own
warmup**, see §3.2), `DC_OP`, `DC_NL_I`, `DC_OP_CONVERGED`, `DC_BLOCK_R`,
`POT_*_INDEX` (subspace static asserts), `POT_N_MIN_R/MAX_R`,
`RUNTIME_R_*_MIN/MAX` (static asserts), `NOISE_THERMAL_N/SHOT_N/FLICKER_N`
(noyce docs/tests), `NOISE_MASTER_SEED_DEFAULT`, `T_ROOM_K`.

### 1.5 Public state fields poked directly

The generated struct is all-`pub`, and wrappers use that (all verified
file:line, oomox-relative):

- **`ctrl_voltage`** — written directly, per sample, on the audio thread, with
  **no setter**: sus-bus `src/lib.rs:457,462` (`circuit_l.ctrl_voltage = cv`
  from the sidechain detector; `0.0` = unity VCA when bypassed);
  series-of-tubes `lib.rs:197,626,757`. This is the VCA control-port contract:
  the field's name, units (volts, negative = gain reduction,
  `G = G0·exp(−Vc/VSCALE)`), and per-sample writability are all load-bearing.
- **`v_prev[node]`** — read per sample: tungsten-glow `lib.rs:538-539`
  (cathode nodes via `CATHODE_NODES`/`NODE_S*_K`, de-meaned AC power → drives
  filament `set_pot` at block rate); vurli `comp/mod.rs:267-273` (same
  sensing convention). Node index stability across regens is contract.
- **`pot_N_resistance`** — read as ramp-start/default values:
  five-watt-freddie `lib.rs:219`, basic-bitch `lib.rs:338-341`, sad-bastard
  `lib.rs:419-422`, gold-press `lib.rs:718,720`, uniquorn `lib.rs:478-488`.
  **`pot_N_resistance_prev`** — written: series-of-tubes `lib.rs:193-195`
  (sync change-detector after jitter). **`switch_N_position`** — read:
  qapla-1a `lib.rs:428-438` (change detection).
- **`smps_v`** (`.runtime V` field) — noyce smps_ripple writes per-sample.
- **`last_nr_iterations`** — velvet-elvis `lib.rs:784,814` (runtime health),
  uniquorn perf harness, several perf/diagnostic tests.
- **`diag_peak_output`, `diag_clamp_count`, `diag_nr_max_iter_count`,
  `diag_nan_reset_count`, `diag_singular_matrix_count`,
  `diag_be_fallback_count`, `diag_voltage_damp_count`** — asserted `== 0` (or
  bounded) in plugin tests; zeroed and summed by perf harnesses (uniquorn
  `lib.rs:962-981`); `settle_dc_op`'s fallback detection reads
  `diag_nr_max_iter_count` exclusively. Renaming or repurposing a diag counter
  is an API + test break.
- **Diag-counter semantics recount (changed 2026-07 campaign, oomox relay
  pending)**: on the nodal paths, (a) `diag_be_fallback_count` now counts
  every BE-fallback **entry**, not only successes — previously failed BE
  attempts were invisible on the full-LU path; (b) `diag_nr_max_iter_count`
  now counts only genuine trap max-iter / factorization failures — it
  previously also counted ActiveSetBe rail-engagement entries whose trap
  solve had fully converged, and sparse-pivot rejections; (c)
  `last_nr_iterations` is pessimistically initialized — it reads `MAX_ITER`
  after any trap-failed sample instead of the stale count from the last
  converged sample; (d) new counter `diag_active_set_pin_count`. The new
  semantics are uniform with the DK template contract. No oomox hard assert
  gates the two recounted counters today (only `diag_nan_reset_count == 0`
  style gates), but pre-campaign print-only baselines (§4.2) are **not
  comparable** across the campaign boundary: be_fallback counts can jump on
  ActiveSetBe circuits (sus-bus), nr_max_iter counts can drop, and
  velvet-elvis's `last_nr_iterations` runtime-health read changes meaning on
  failed samples.
- `i_nl_prev` / `input_prev` / `dc_block_*` are **not** written by wrappers
  (solver-owned); they appear in wrapper comments (NR-predictor rationale)
  only.

**Struct-field renames, field removals, or making fields private are downstream
breaks even though no melange-side test notices.**

---

## 2. Calibration constants layered on top

These are the silent-invalidation layer: every value below was measured against
the current generated code's output level / noise magnitude / response and
hand-pinned. A melange device-math change moves the measured quantity; nothing
recompiles, nothing fails locally; the shipped plugin is simply wrong until
someone re-pins.

Two idioms (both break identically, the second is harder to grep):
1. scalar `OUTPUT_TRIM` applied after `VoltageRef::to_daw()`;
2. calibration baked into the `VoltageRef { input_v, output_v }` itself
   (tungsten-thunder-horse, warpony, moonladder).

### 2.1 Per-plugin table (hand-written wrapper code only)

| Plugin | Constant | Value | Location (oomox) | Pinned against |
|---|---|---|---|---|
| basic-bitch | `VOLTAGE` | `VoltageRef::GUITAR` | `src/lib.rs:93` | input drive domain |
| | `OUTPUT_TRIM` | `0.0330` | `src/lib.rs:105` | defaults level, re-pin via `tests::gain_staging_at_defaults` |
| five-watt-freddie | `OUTPUT_TRIM` | `0.003397` | `src/lib.rs:83` | Champ output level vs jlbrock.wav |
| gold-press | `OUTPUT_TRIM` | `1.0` | `src/lib.rs:169` | pinned 2026-05-11 via `tests/live_gain_staging.rs` (presets peak −10.6/−8.4/−6.9 dBFS) |
| | click/pop/scratch/hiss volts | `0.015 / 0.045 / 0.018 / 0.004 / 0.012` | `click.rs:58`, `needle_drop.rs:31,34`, `surface_noise.rs:21,38` | **injection amplitudes in circuit-input volts** — scale with cartridge/RIAA chain gain |
| | tonearm `FORCING_RMS_AT_MAX_MOTOR_VOLTS` | `0.005` | `tonearm.rs:50` | resonator level post-RIAA |
| | crosstalk `LF/HF_BLEED_GAIN` | `0.0447 / 0.1778` | `crosstalk.rs:17,19` | voltage-domain bleed |
| moonladder | `VOLTAGE` | `EURORACK` (±5 V) | `src/lib.rs:24` | synth levels; no trim |
| noyce (12 sources) | `OUTPUT_TRIM` per source | clean_rc `1.5e4`; triode_12ax7 `30`; ef86 `50`; zener `1.0e5`; **germanium `1800`** (20→1300→1800 post-campaign); **carbon_comp `4000`** (0.1→200→4000); jrc4558 `1500`; transformer_triode `100`; tape_head `100`; amp_at_idle `1.0` (×`GUITAR.to_daw()`, post-cab); smps_ripple `0.10`; **boiler_room `50_000`** (20000→50000 post-campaign) | `src/sources/*/mod.rs` | **melange noise-magnitude calibration directly**; target peak ∈ [−25,−10], RMS ∈ [−40,−22] dBFS @290 K |
| | `NOISE_GAIN` | `1.0` all sources | same | physical honesty pin |
| | `SMPS_AMPLITUDE_V` | `5.0` (35 kHz saw into `smps_v`) | `smps_ripple/mod.rs` | ripple drive volts |
| periodic-pedal | `VOLTAGE`; `OUTPUT_TRIM` | `LINE`; `0.923` | `src/lib.rs:63,70` | calibrated 2026-06-09, −9 dBFS midpoint |
| pipe-shouter | `OUTPUT_TRIM`; `MID_BOOST_GAIN_DB` | `0.147167` (=−16.64 dB); `4.0` @700 Hz | `src/lib.rs:45,103` | TS808 level + plugin-side mid restore |
| pretty-baby | `OUTPUT_TRIM` | `0.7079` (−3 dB) | `src/lib.rs:34` | gain-staging standard |
| qapla-1a | `VOLTAGE`; `OUTPUT_TRIM` | `LINE`; `1.0` | `src/lib.rs:33,40` | unity vs current EQ makeup stage |
| sad-bastard | `OUTPUT_TRIM`; `TONE_GAIN_DELIVERED_DB` | `0.172`; `12.0` | `src/lib.rs:76,156` | re-pinned when BP node ran +5 dB vs circuit V2.2 |
| series-of-tubes | `INTRINSIC_MAKEUP_DB` | `7.0` | `src/lib.rs:95` | **compensates per-stage attenuation 0.988^N over the cascade** — direct function of stage-cell gain; the ×2 gm change destroyed it (§6) |
| | `VARIATION_PCT` | `0.05` | `src/lib.rs:84` | drives ±5% `.pot` clamp ranges (pots abused for tube mismatch) |
| subspace | `AM/FM_INPUT_SCALE`; `AM/FM_TRIM`; `PILOT_AMPLITUDE_V` | `0.5/0.1`; `0.06/0.10`; `0.0011` | `src/lib.rs:82-105` | calibrated 2026-05-20 via `tests/calibration.rs`; pilot lands −56 dBFS |
| sus-bus | `VSCALE`; `THRESHOLD_KNOB_TO_DBFS`; `PASSBAND_GAIN_COMP` | `0.05298` V/Np; `−14`; `1.82` | `src/sidechain.rs:9,31`, `src/lib.rs:28` | THAT2180 CV law **must match the .model VSCALE**; passband makeup vs current filter response |
| the-kicker | `SINE_DRIVE_VOLTS` | `0.5` | `src/lib.rs:140` | internal drive sized to chain gain ~30× → few V peak leaning on Q_drive |
| tungsten-glow | `OUTPUT_TRIM` (inverting); `TG_THERMAL_GAIN`; `CATHODE_DC_BIAS`; heat makeup | `1.0` (applied negated); `1000.0`; `0.621374`; `10^(−att/20)` | `src/lib.rs:46,66,76,127` | current polarity, noise magnitude, bias point, drive-vs-loudness curve |
| tungsten-thunder-horse | `VoltageRef{input_v:0.5, output_v:30.0}`; noise default | baked; `0.5` | `src/lib.rs:61-63,209` | output_v=30 ⇒ ~7.5 V peak = −12 dBFS |
| uniquorn | `OUTPUT_TRIM` | `0.846` | `src/lib.rs:50` | cascade+power level |
| velvet-elvis | `AUTO_GAIN_MAX_DB`/`AUTO_GAIN_PEAK_CEIL` | `24.0` / `−1.0` dBFS | `src/lib.rs:61,68` | leveler range vs circuit gain |
| vurli | `THERMAL_GAIN`; `COLD_STAGE_GAIN_DB`; `TUBE_DRIVE` | `300.0`; `10.3`; `0.305` | `src/comp/mod.rs:75,96,114` | 6SL7 stage gain at engine levels; TUBE_DRIVE re-cal 2026-04-26 (0.5 baked +4.3 dB error) |
| warpony | `VoltageRef{input_v:0.2, output_v:75.0}` | baked | `src/lib.rs:37-39` | 8-stage cascade: 38 V peak ≈ −6 dBFS |
| funkyinduct, vcr-audio | *(none)* | — | — | funkyinduct raw-domain; vcr-audio has no wrapper |

Normative target for all pedals: **−12..−6 dBFS peak at default knobs with
jlbrock.wav** (`docs/gain-staging-standard.md`). Every OUTPUT_TRIM above encodes
"current circuit gain → this window".

### 2.2 What kinds of melange change invalidate this table

- Device transfer-function changes (gm, saturation curves, junction laws) →
  every OUTPUT_TRIM / VoltageRef.output_v / makeup constant.
- Noise magnitude recalibration (thermal/shot/flicker scale, per-category) →
  all noyce OUTPUT_TRIMs, tungsten-glow/vurli thermal gains, noise defaults.
- DC operating point shifts → CATHODE_DC_BIAS, DC-blocker seeding assumptions,
  injected-signal headroom (gold-press volts, the-kicker drive).
- Frequency-response changes (parasitic caps, oversampling filters, decimators)
  → pipe-shouter MID_BOOST, sus-bus PASSBAND_GAIN_COMP, gold-press crosstalk,
  subspace pilot level.
- VCA/opamp model changes → sus-bus VSCALE law and threshold calibration.

---

## 3. Behavioral assumptions

### 3.1 Canonical lifecycle (normative in `docs/best-patterns.md`)

```
initialize():   circuit = Box<CircuitState::default()>
                circuit.set_sample_rate(sr)           // NOT optional — default state is at codegen fs
                apply ALL pots/switches at current param values   // BEFORE warmup
                warmup (one of three strategies below)
reset():        cheap in-place circuit.reset() OR full re-init; re-warm iff sample rate changed
process():      per block: smoothed params → quantize R to 3 sig figs → change-detect → set_pot_N(r)
                per sample: out = process_sample(in_volts, &mut circuit)
editor thread:  NEVER touches CircuitState — only Arc'd atomics/ring buffers
```

Three warmup strategies in the fleet (which one a plugin uses drives its whole
lifecycle):

- **(A) Exact NR solve** — series-of-tubes only: `default()` →
  `set_sample_rate` → jittered `set_pot_0/1/2` → `recompute_dc_op()` per stage
  (`lib.rs:176-201`); warmth likewise (`:329-336`). No sample loop at all. The
  warmed 64-stage Vec is cached in a process-global
  `RwLock<HashMap<(sr,n), Arc<Vec<CircuitState>>>>` (`:132`) and cloned across
  resets/instances — **generated CircuitState must stay `Clone`-safe and
  deterministic for this cache to be sound**.
- **(B) Silent pot-ramp warmup** — gold-press, uniquorn, basic-bitch,
  five-watt-freddie, sad-bastard, tth: `full_warmup()` (uniquorn `:591`) does
  `default()` → `set_sample_rate` → ramp pots from codegen defaults (read out
  of `pot_N_resistance`) to targets in **30 log-space steps**, each quantized
  + `set_pot`'d + followed by ~15 ms of zero-input `process_sample`; then an
  **NR predictor prime**: 30 ms of 140 Hz / 50 µV sine with 5 ms fade so
  `v_prev`/`i_nl_prev` land mid-manifold (avoids first-block chop). Warmup on
  SR change is deferred to `process()` via `warmup_needed: AtomicBool` (block
  is silenced while it runs).
- **(C) Fixed-duration `warmup_circuit()`** — qapla-1a, velvet-elvis,
  tungsten-glow, moonladder, vurli: pots/switches set first, then 0.5 s
  (`WARMUP_SECONDS`, "5 time-constants for ~100 ms coupling caps") of
  zero-input samples via `oomox_core::pedal::warmup_circuit`. qapla-1a warms
  in `reset()` only (nih-plug calls reset after initialize and would discard
  warm state). sus-bus/subspace/the-kicker use no or minimal (2048-sample)
  warmup.

Generated `warmup()` (50 samples) and `WARMUP_SAMPLES_RECOMMENDED` are both
**bypassed** by these oomox conventions — a melange change to settle-time
behavior changes whether 0.5 s / 30-step ramps are still enough, and nothing
downstream measures that automatically.

### 3.2 Input/output domain

- Per-sample chain everywhere: `DAW f32 → f64 → (drive gain) →
  VoltageRef::to_volts → process_sample → [0] → VoltageRef::to_daw →
  ×OUTPUT_TRIM → clamp/brickwall → f32`. `VoltageRef` consts
  (`oomox-core/src/voltage.rs`): `LINE` 1.0 V, `GUITAR` 0.2 V, `EURORACK`
  5.0 V, plus custom `{input_v, output_v}` pairs (tth `0.5/30`, warpony
  `0.2/75`). series-of-tubes and sus-bus pass raw ±1.0 floats as volts
  (implicit LINE). vurli **negates** the output (single triode stage is
  phase-inverting) — output polarity of generated code is contract.
- Wrappers assume output is already DC-blocked (5 Hz single-pole inside
  generated code, seeded from the DC OP) — plugin-side DC blockers were
  removed as double-filtering (debugging-history). Changing `DC_BLOCK_R`
  policy or seeding changes LF response and onset thumps downstream.
- Output clamp: generated ±10 V clamp is melange's safety; plugins add final
  `.clamp(-4.0, 4.0)` or `brickwall_limit` on the f32.
- Multi-circuit chaining passes **volts directly** between `process_sample`
  calls: series-of-tubes `warmth → 64× stage` (makeup applied only at the
  very end, `lib.rs:446-461`); uniquorn `cascade → power` with **no
  inter-stage scalar** (`lib.rs:816-817`, warmup runs the chained pair
  together); gold-press `overdrive → mastering → cartridge → riaa → cab` with
  only per-channel `l_skew`/`r_skew` on the RIAA input (`:929-930`). A gain
  change in one melange device therefore moves the *operating region* of
  every downstream nonlinear stage, not just final level — this is the §6.1
  amplification mechanism.

### 3.3 Pots, smoothing, presets

- Oomox owns taper/curve shaping (`oomox_core::pots`, `pedal.rs`
  `knob_to_resistance` exp taper, `wiper_split`) and parameter smoothing;
  melange sees only final resistances in ohms. Wrappers cache `prev_*` values
  (NaN-invalidated) and only call `set_pot_N`/`set_switch_N` on change, with
  `quantize_resistance` (3 sig figs) first. **Exception: moonladder calls
  `set_pot_0/1` every sample with raw resistance and relies solely on the
  generated no-op guard** (`lib.rs:259-264`) — the `<1e-12` early-return is
  load-bearing for its CPU budget.
- Wrapper comments still encode the legacy contract that a `set_pot` jump
  >~20% of compiled value fires a DC-OP reset (`v_prev = DC_OP`) → click, and
  that any change costs an O(N³) rebuild. The entire ramp/quantize/
  change-detect machinery exists because of it. Melange has since removed the
  warm-snap, but **re-introducing any jump-triggered reseed in the setters
  re-collides with this machinery**, and plugin FloatParam defaults are kept
  aligned with codegen nominal pot values (implicit .cir ↔ melange ↔ oomox
  invariant).
- Switches are unsmoothed IntParams committed on change; `set_switch`
  DC-reseeds internally, so gold-press masks the click with a ~5 ms cosine
  output fade (`lib.rs:934-947`) and periodic-pedal runs a 128-sample
  mini-warmup. The click-on-switch behavior is compensated *downstream* —
  making switches quieter or louder changes those masks' correctness. Written
  canon in gold-press (`lib.rs:1141-1142`): **"never save/restore `v_prev`
  around `set_switch_0`"** — hard-won from the periodic-pedal state-corruption
  bug (§4.5).
- Special pot drivers: vurli calls `set_pot_0` **per sample** from a thermal
  RMS ODE (`comp/mod.rs:264-265` — filament resistance IS the compression
  mechanism); tungsten-glow drives its filament pots at block rate from
  `v_prev` cathode sensing (`lib.rs:463-484`). Per-sample `set_pot` cost and
  the no-op guard are contract for these two.
- **Preset recall**: presets are plain param sets; recall flows through the
  normal smoothed ramp path (150 ms smoothers, staying under the legacy 20%
  threshold). No plugin calls `recompute_dc_op` on preset recall today;
  `full_warmup()` snaps smoothers (`smoothed.reset(value)`) so a preset
  loaded before first process doesn't ramp default→preset.

### 3.4 Sample-rate change and bypass

- SR change: strategy-B plugins detect `warmed_at_sr != sr` in `reset()` and
  defer `full_warmup()` into the next `process()` (silenced block); C-strategy
  plugins re-run `warmup_circuit` inline; sus-bus/subspace/moonladder/
  the-kicker use in-place `.reset()` which deliberately preserves the SR
  matrices set at initialize (sus-bus `lib.rs:380` comment warns `default()`
  would silently rebuild at 48 kHz). gold-press and uniquorn re-report
  `set_latency_samples` after re-warm.
- Bypass: circuit is never torn down. Either an atomic active-flag skip, or
  soft bypass *through* the circuit: sus-bus sets `ctrl_voltage = 0.0` for
  unity VCA gain (`lib.rs:452-455`); velvet-elvis crossfades. So "circuit at
  idle" behavior (noise floor, DC) is audible in bypassed-but-soft paths.
  Smoothers keep advancing during bypass (uniquorn `lib.rs:750` — skipping
  `next_step` desyncs sample-time).

### 3.5 Noise & edge behavior assumptions

- Noise defaults OFF everywhere (feature-gated opt-in); enabling is per-block.
  Noyce runs sources at `noise_gain = 1.0` and shapes level ONLY via
  OUTPUT_TRIM — **noyce ships melange's physical calibration raw**.
- Deterministic seeding for reproducible unit character: SoT per-stage jitter
  (ChaCha8, seed `0xB0BA1E50B0`, oomox-side) + melange `set_seed` determinism
  (subspace, noyce) — same session must sound identical run-to-run.
- Silence floor when disabled ≈ structural residue (`cond(G)·3e-9 V`);
  noyce asserts source-specific floors (§4.1).
- NaN in the solve → generated state reset + `diag_nan_reset_count` bump;
  tests assert it stays 0.
- the-kicker drives the circuit **internally** (0.5 V sine burst; no DAW
  input); DC drift and retrigger stability asserted.

---

## 4. Tests oomox runs against generated code

### 4.1 Noyce validation suites — the tightest downstream detectors

Noyce asserts melange's **physical noise calibration directly** (all category
gains pinned 1.0). Per-source `plugins/noyce/tests/*_validation.rs`, common
shape (silence input, 290 K, 96 kHz, ~2 s):

- **Raw variance bands** (V², gain 1.0): clean_rc = kTC ratio `0.60–1.15`;
  triode_12ax7 `[1e-8,1e-4]`; ef86 `[1e-9,1e-4]`; zener `[1e-15,1e-10]`;
  germanium `[1e-11,1e-7]` (re-tightened post-campaign for ×2400-quieter
  flicker); carbon_comp `[1e-13,1e-5]`; jrc4558 `[1e-13,1e-7]`;
  transformer/tape `[1e-10,1e-5]`; smps drive-off `[1e-2,1]`; boiler `>1e-16`;
  amp `>1e-7`.
- **Raw peak ceilings** (clamp guards): e.g. zener `<1e-3`, jrc4558 `<0.01`,
  carbon `<3.0`.
- **Temperature scaling**: clean_rc linear ±1 dB across 3 K→1500 K; shot/mixed
  sources ordered `v_hot > v_ref > v_cold` with documented ratios (zener cold
  0.58×, hot 2.79×).
- **Sample-rate sweep** 44.1→192 k: variance monotone increasing; spread `<30×`
  (shot) / `<10×` (1/f).
- **Seed behavior**: same seed → bit-identical; different seeds diverge;
  two instances `|corr| < 0.1` (`tests/integration.rs`).
- **Silence floors** (`noise_enabled=false`): clean_rc `<1e-15`; zener `<1e-9`
  (~2e-10 structural residue ≈ −194 dBFS — matches melange
  `silence_test_residue_floor` model); jrc4558 `<1e-10`.
- **dBFS calibration**: post-TRIM peak ∈ [−25,−10], RMS ∈ [−40,−22].
- **`noise_stationarity.rs`** — the Zener-regression sentinel, mirrors
  melange's bench guard: seeds {1, 7, 0xdeadbeef, 0x12345678deadbeef}, asserts
  **lag-1 autocorr > −0.5** (fs/2 limit cycle = −1.0) and **σ seed-spread
  < 3 dB** (post-fix reality: 0.4 dB). References melange `a472807`.
- **`audible_band_probe.rs`** — `#[ignore]` diagnostic: per-band (<20 / 20–200 /
  200–2k / 2k–20k Hz) energy split; exists because a 1/f source can pass a
  broadband-RMS gate while its audible band is starved by sub-20 Hz content.
- **`smps_ripple_validation.rs`** `switching_whine_dominates`: drive-on/off
  variance ratio `>100×` (tripwire that the `.runtime V` field propagates).
- **`amp_at_idle_validation.rs`** `cab_shapes_spectrum`: mid/HF ratio ∈
  [10, 40) dB post-cab.

Any melange change to noise scale factors, RNG seeding/salting, draw scheme
(single vs two-draw), flicker calibration, or the trapezoidal noise-injection
path lands in these suites first.

### 4.2 Two test archetypes

(a) **Hard regression guards** — numeric asserts that fail CI; (b) **print-only
diagnostics** — dump levels and diag counters without asserting (will NOT catch
a regression): `uniquorn/tests/{diagnostic,matrix_health,signal_behavior,long_run,cold_start_diag}.rs`,
`tungsten-thunder-horse/tests/{diagnostic,perf}.rs`, `warpony/tests/{diagnostic,bass_analysis,stability,perf}.rs`,
`sad-bastard/tests/{silence_probe,perf}.rs`, `series-of-tubes/tests/idle_floor.rs`,
`pretty-baby/tests/signal_diag.rs`, all `perf*.rs`. These print
`last_nr_iterations`, `diag_nr_max_iter_count`, `diag_nan_reset_count`,
`diag_be_fallback_count`, `diag_voltage_damp_count`, `diag_substep_count`,
`diag_refactor_count` — so diag-counter renames break them at compile time even
though they gate nothing.

### 4.3 Hard-assert highlights per plugin (non-noyce)

| Plugin | File | Key quantitative gates a melange change can trip |
|---|---|---|
| basic-bitch | `tests/character.rs` | silence peak `<0.03`; extreme-settings peak `<=4.0` (±4 clamp); `diag_nan_reset_count == 0`; THD(max drive) > THD(min)+1%; volume authority `<-15 dB` / `<-20 dB` at max drive; tone tilt LF `<-6`/HF `>+6 dB`; **compile-time pins `DRV_STAGE_* == circuit::POT_0/1/2_MIN/MAX_R`**; `low_drive_hot_input_no_brickwall_trip` (envelope headroom) |
| basic-bitch, five-watt-freddie, pipe-shouter | `tests/diagnostic.rs` | **seam check**: plugin-domain vs raw-circuit gain residual `<1 dB` after subtracting `OUTPUT_TRIM` — the routing test deciding "bug is plugin-side vs melange-side" (best-patterns §Seam) |
| five-watt-freddie | `tests/character.rs` | silence `<0.03`; brickwall `<=1.0`; THD ordering; cab 12 kHz > 10 dB down vs 1 kHz; **96 vs 48 kHz RMS Δ `<4 dB`**; noise-dormant floor `<-54 dBFS`; pot-const pins |
| funkyinduct | `tests/eq_character.rs` | silence `<0.01`; all-pots-cranked peak `<10`; 7 band-shape asserts; `perf_diagnostic.rs` (ignored): CPU `<50%`, `diag_nan_reset_count == 0` |
| periodic-pedal | `tests/character.rs`, `automation.rs` | **generated R_fb endpoints pinned: Drive=0 → 1000 Ω, Drive=1 → 470 kΩ**; monotonic; default near-silence `<0.05`; ±4 clamp; every crystal ≥6 dB swing; gain-pumping range `<12 dB`; switch-recovery drift `<3 dB` |
| pipe-shouter | `tests/character.rs` | silence `<0.03`; ±4 clamp; seam residuals `<1 dB` in small-signal, freq×drive, and clipping regimes |
| qapla-1a | `tests/{saturation,freq_response,eq_character}.rs` | THD monotone with drive; flat ±6 dB 40 Hz–16 kHz; band-boost deltas; tube THD ∈ (0.01%, 20%); peak `<10`/`<20`; DC offset `<0.05` |
| sad-bastard | `tests/character.rs` | silence `<0.03`; ±4 clamp; **pot pins vs `POT_6/7_MIN/MAX_R`**; envelope adds no floor `<0.03` |
| series-of-tubes | `src/lib.rs` tests module | 3 circuit-character tests **`#[ignore]` KNOWN BROKEN** (see §6.1): `full_chain_is_stable`, `harmonic_response_to_cv`, `cascade_frequency_response_sweep` |
| subspace | `tests/{radio_character,emergent_noise,noise_floor,calibration,character}.rs` | AM/FM SNR full `>18–20 dB`, weak `<8 dB`; FM noise floor present `>-75 dBFS` **and span `<30 dB` across reception** (fixed-antenna-floor regression guard); pilot ∈ [−75,−40] dBFS; Volume=0 → peak `== 0.0` exactly |
| sus-bus | `tests/ssl_character.rs` | GR monotone by ratio (10:1 > 4:1 > 2:1) — depends on VCA CV law |
| the-kicker | `tests/character.rs` | tonal purity `>0.70`; velocity high `>1.5×` low; retrigger peak `<20`; DC-drift tail mean `<0.5`; tuning error bounds |
| tungsten-glow | `tests/{character,signal_path}.rs` | no self-oscillation `<0.001`; gain range `>15 dB` monotone; thermal-compression ordering asserts; `calibration.rs` both `#[ignore]` (jlbrock, `SHIPPED_GAIN=1000`) |
| tungsten-thunder-horse | `tests/character.rs` | THD `>0.1%` at both 48 k and 96 k (only hard asserts) |
| uniquorn | `tests/character.rs` | steady-state near-silence `<0.05`; peak `<3.9`; **drive→stage-R map pinned (100 kΩ/100 Ω progressive bypass)** |
| velvet-elvis | `tests/{tube_character,staging_safety,compressor_behavior}.rs` | THD non-decreasing with drive, full-drive `>0.1%`; leveler steers to −18 dBFS ±3.5; response ±6 dB 80 Hz–10.5 kHz; **Shanna-ceiling hot-vocal gate** (peak `>-0.5 dBFS`, crest `>18 dB` at shipped defaults); DC offset `<0.05`; max-settings peak `<10` |
| vurli | `tests/peak_diagnostic.rs` | HEAT=0 transparency: leveler/engine peak ratio `<1.12` (caught the +4.3 dB TUBE_DRIVE makeup bug); engine peak `<1.05` |
| warpony | `tests/character.rs` | drive THD span `>1.5×`; peak `<500 V` (no blowup); silence DC `<0.2 V`; gain spread `<8 dB` |
| gold-press | `tests/live_gain_staging.rs` | **all cabinet presets peak ∈ [−12, −6] dBFS on jlbrock.wav at defaults** — direct implementation of the gain-staging standard; prints an OUTPUT_TRIM re-pin suggestion when out of spec |
| moonladder | `tests/filter_character.rs` | **all 15 tests `#[ignore]`** ("CircuitState overflows 2 MB test-thread stack in debug") — zero live coverage |

Generated-constant pins that fail on a bad regen: pot min/max asserts
(basic-bitch, five-watt-freddie, sad-bastard), periodic-pedal R_fb endpoints,
subspace pot-index asserts. **Not referenced by any test:**
`WARMUP_SAMPLES_RECOMMENDED`, `OUTPUT_SCALES`, `DC_OP`/`DC_NL_I`, bare `N`/`M`.

### 4.4 Normative docs pinning circuit-output expectations

- **`docs/gain-staging-standard.md`** — reference input jlbrock.wav (clean peak
  −3.7 dBFS). Category peak targets at defaults: clean-boost −6 (−8…−4), medium
  OD −9 (−11…−7), heavy −12 (−14…−10); hard ceiling −3 dBFS, floor −14 dBFS.
  Drive is not volume: full drive sweep must change level `< ~3 dB`. Noise
  generators exempt (peak −25…−10, RMS −40…−22). **Dual-Source Calibration
  Gate**: no level default pinned until rendered through BOTH a quiet source
  AND a hot ~22 dB-crest real source ("Shanna ceiling") — synthetic sines
  under-measure real-speech HF peaks by 20–26 dB (burned Gold Press ~15× hot
  and Velvet Elvis +4.9 dBFS with a green 49-test suite).
- **`docs/plugin-qa-standards.md`** — DC offset after warmup `<0.001`; no
  NaN/Inf/subnormals; SR independence 44.1–192 k with character within 3 dB;
  buffer-size independence to −120 dB; pots set before warmup; 0.5 s default
  warmup.
- **`docs/quality-scorecard-methodology.md`** — harmonic-structure thresholds
  measured on real audio (H2 leads H3 by 6–10 dB; rolloff >3 dB/harmonic past
  H5; IMD >20 dB down; crest retention >60%; clipping asymmetry 1–3 dB; noise
  floor <−80 dBFS A-weighted). Any device-model change to harmonic balance
  lands here (warpony/uniquorn scorecards).

### 4.5 Debugging-history lessons at the melange boundary

From `local-docs/debugging-history.md`, the entries whose root cause was in
generated code or the boundary (each is a regression class to not reintroduce):

- Switch position-0 used static netlist value instead of correct G-matrix value
  (periodic-pedal state corruption); never preserve `v_prev` across switch
  changes.
- Unscaled element output +17 dBFS when no OUTPUT_TRIM existed; ±10 V clamp
  means +34 dBFS worst case at the DAW.
- `set_pot_N` deltas >~20% fire a warm DC-OP re-init (`v_prev = DC_OP`)
  mid-stream → clicks; plugin FloatParam defaults MUST match codegen nominal
  pot values or the reset fires every block (TTH v1, warpony, uniquorn
  "loose-wire" sputter). Interpolate pot changes; never jump >~15%/call.
- Melange already emits a 5 Hz DC-block HPF — plugin-side DC blocking doubles
  it (TTH v1, warpony).
- `VoltageRef.output_v` must be measured via `melange simulate` at defaults,
  not guessed (warpony +14 dBFS over full scale).
- Plugin pot constants exceeding generated `POT_N_MIN/MAX_R` = silently dead
  knob travel (uniquorn v3); after regen, cross-check the generated clamps.
- `set_runtime_R_*` is per-block, not per-sample (4500× slowdown when misused);
  `set_pot_N` per-sample is fatal at M≳6.
- Spectral-radius near 1.0 in compile output predicts trapezoidal
  self-oscillation; `--backward-euler`/grid stoppers (tungsten-glow, warpony
  Nyquist artifacts — since automated melange-side).

---

## 5. Regen protocol

### 5.1 The mechanism

```bash
melange compile ../melange-circuits/<tier>/<cat>/<name>.cir \
    --format code -o plugins/<plugin>/src/circuit.rs [--noise shot|thermal|full] [flags]
```

(oomox `CLAUDE.md` "Build & Test"; per-plugin recipes historically scattered in
`docs/melange-usage.md`, lib.rs header comments, and Cargo.toml `noise` feature
comments.) **The canonical consolidated registry is now melange
`local-docs/golden-baseline-manifest.{json,md}`** (generated 2026-07-21,
verified against melange `8945b67`): per generated file it records the exact
netlist path, compile command, byte-diff status vs a fresh regen
(EXACT / DRIFT-EXPLAINED by post-regen commits `146d51b`·`a472807`·`49ecaa4` /
UNRESOLVED), solver, N/M, and pot/switch/runtime-R/noise counts. Verified
defaults across the fleet: fs 48000, input node `in`, output node `out`,
`--solver auto`, `--noise-seed 0`, no `--output-scale`, no manual BE flags.
**Only three flags ever vary per circuit: `--oversampling`, `--noise <mode>`,
`--emit-dc-op-recompute`.** `MAX_ITER` differences come from the CLI
auto-tuner, not `--max-iter`. No hand-edited generated files exist.

Key regen rules oomox operates by:

- **`circuit.rs` is never hand-edited** ("DO NOT EDIT — regenerate from .cir").
  All local behavior differences live in the `.cir` or the wrapper.
- **Noise flags are per-plugin and mandatory to preserve**: noyce = `--noise
  full`; most pedals = `--noise shot` or `--noise thermal` (recorded in
  Cargo.toml comments). Omitting the flag yields a *silently silent* plugin.
- **Seeded variation is baked**: pipe-shouter and sad-bastard compiled with
  `.mismatch`/`.tolerance` seed 42 — a regen with a different seed changes the
  shipped unit's component values (still "correct", but sound-different).
- **Multi-circuit plugins regen all files together** (series-of-tubes ×2,
  tth ×3, uniquorn ×2, gold-press ×5, subspace ×2, noyce ×12) — mixed-vintage
  files within one plugin are an unsupported state.
- Canonical `.cir` paths can diverge from docs: sus-bus builds from
  `testing/dynamics/4kbuscomp-audiopath.cir`, not the unstable 122-component
  variant (noise-regen-spec.md).

### 5.2 "Anchor diff" verification

From regen commit `ab21b67` (the campaign regen of 40 circuits): after
regenerating, diff old vs new generated file and verify the **anchors are
preserved**: `N`, `M`, `OVERSAMPLING_FACTOR`, `INPUT_NODE`, `OUTPUT_NODES`,
presence/absence of noise functions (the manifest's verification extends this
to `OUTPUT_SCALES`, `INPUT_RESISTANCE`, `SAMPLE_RATE`, and the **full setter
list**). A coefficient-only regen changes only device/matrix numeric literals.
If an anchor moves, the regen is **not drop-in** — the wrapper needs rewiring
and the file is held back instead.

### 5.3 Special-handling inventory (as of 2026-07-21)

- **subspace `radio_fm.rs` — HELD BACK** at the old topology: the reworked
  behavioral-FM `.cir` changed N 16→23; plugin not wired for it. The checked-in
  file does not correspond to melange-circuits HEAD.
- **noyce `transformer_triode/circuit.rs` — regen DEFERRED** (a8c47f3): coupled-
  inductor codegen was non-deterministic (melange HashMap ordering). Melange
  fixed determinism in `49ecaa4`; manifest status is DRIFT (hdr+shot+rows) —
  i.e. reproducible at HEAD modulo known commits, canonical regen still owed
  (qapla-1a shares the rows-drift, it also has coupled inductors).
- **vcr-audio** — orphan generated file, no crate; refreshed on campaign regens
  anyway.
- **tth `cascade.rs`/`edge.rs`** — compiled but never imported; still gate the
  build; do NOT regen the split variants when targeting the monolithic circuit.
- **uniquorn-16stage.cir** — benchmarking artifact, no oomox target.
- **series-of-tubes** — `.pot` ranges are ±5% jitter clamps, not user knobs
  (per-unit tube variation); pot semantics changes hit it unusually.
- Post-regen gate used by the campaign: `cargo build --release` across the
  workspace; **DAW/listening verification is explicitly separate and per-plugin**
  ("NOT DAW-verified" in ab21b67) — which is exactly the crack §6's damage
  slipped through.

---

## 6. Case study — July 2026 accuracy-campaign damage

The campaign (melange `3e246cb..8fb9741`, regen `ab21b67` 2026-07-18) was
*correct* melange work: triode ×2 gm (Koren factor), op-amp rail-clip
differential polarity, Gold Press OS=4 polyphase decimator, corrected noise
floors. Every incident below is the calibration/test layer absorbing a correct
upstream change — none was rolled back.

### 6.1 series-of-tubes: +39 dB hot (commit `5c0c741`)

Triode ×2 gm, compounded over the stage cascade (banner says 50 stages;
`NUM_STAGES = 64` at `lib.rs:79` — makeup math is pinned for both: N=50→5 dB,
N=64→7 dB), left output ~+39 dB hot (finite, stable) and moved the THD
operating point (unity THD ~22%; compression now *reduces* THD; 1 kHz H1
reference ~5.6 vs expected 0.01–0.3). Plugin marked KNOWN BROKEN in the
`lib.rs:3-9` banner; 3 circuit-character tests `#[ignore]`d
(`full_chain_is_stable`, `harmonic_response_to_cv`,
`cascade_frequency_response_sweep`) pending rehoming to melange-circuits;
do-not-ship until re-listened + re-trimmed. **Lesson: cascade plugins multiply
per-cell error by stage count — a fraction-of-a-dB per-stage gain change is
tens of dB at N=50–64. `INTRINSIC_MAKEUP_DB` (0.988^N compensation) is a
direct function of the stage cell's linear gain.**

### 6.2 noyce germanium: ×2400 quieter (commits `a8c47f3`, `960ee70`)

Flicker recalibration to physical ngspice level dropped the Germanium source
~×2400. The plugin's favorite-source went "super quiet" (raw −53 dBFS).
OUTPUT_TRIM re-pinned 20→1300 (interim) →1800 (final: peak ≈ −14 / RMS ≈ −27
dBFS; 2300 auditioned as too loud). boiler_room re-pinned 20000→50000.
Germanium raw-variance test bounds rewritten for the new physics. The
audible-band probe was created because broadband-RMS targets can be satisfied
by inaudible sub-20 Hz energy. **Lesson: noise magnitudes are a shipped-sound
API. "More physically correct level" still requires a coordinated downstream
re-trim and re-audition, and the trim history (20→1300→1800) is the cost.**

### 6.3 noyce zener: +46 dB seed-dependent (specs `noyce-zener-noise-regression{,-response}.md`)

Post-regen investigation found Zener RMS varying 13–17 dB with seed, ~46 dB
above physical, lag-1 autocorr −1.000. Root cause was **melange-side and
pre-existing** (not a campaign regression; byte-identical at `dab5653`):
single-draw shot noise exciting the z=−1 trapezoidal pole on a stiff
resistor-only breakdown node, rectified into audio by the diode exponential.
Oomox's filing pinned the contract line: *"no OUTPUT_TRIM can fix a
seed-dependent absolute level; we will not band-aid a broken circuit
plugin-side"* — with quantitative acceptance criteria (seed spread <0.5 dB,
crest ≈13–14 dB @96 k, level within ~3 dB of 200 nV, off-floor ~2e-10 V).
Melange fixed it (`a472807`, two-draw shot); noyce regen'd 11 sources and added
the `noise_stationarity.rs` sentinel. **Lesson: oomox distinguishes
"recalibrate downstream" from "fix upstream" exactly along the melange
CLAUDE.md accuracy rule, and campaign-scale regens are when latent solver bugs
surface — expect filings.**

### 6.4 July DC-bias-shift class (declared retroactively, 2026-07-21 round-2 audit)

Two more campaign fix families — both mathematically **verified correct** in
the round-2 audit — moved the baked `DC_OP` / `DC_NL_I` constants fleet-wide,
and were not on the declared 4-item change list:

1. **`.linearize` Norton companion constants** (`mna.rs`
   `stamp_linearized_bjts` / `stamp_linearized_triodes`): DC injections
   changed from raw `±I0` to the proper Norton constant `I_lin(v0) − I0` per
   terminal; the old form left the `g·v0` term uncancelled (multi-mA KCL
   error → volts of bias shift at 12AX7-class gm). Affects every shipped
   `.linearize` netlist: basic-bitch, pretty-baby, sad-bastard, uniquorn (×2),
   tungsten-thunder-horse (+cascade), warpony, wurli-power-amp.
2. **DC-OP device-law parity** (`dc_op.rs`): the DC solver now evaluates the
   same laws as the transient runtime — BJT ISE/NE + ISC/NC leakage, the
   RS-diode junction solve (`DiodeWithRs`), damped 2D Newton for parasitic-R
   BJTs (the old undamped 3-iteration fixed point oscillated whenever
   `gm·(RE + RB/β) ≳ 1`), MOSFET body effect, VCA THD. Audit-reproduced
   magnitudes vs ngspice: a 2 V collector-node shift on an ISE'd 2N3904 bias
   network; 0.39 V on an RS-diode fixture.

**Exhibit**: the warpony regen diff shows emitter nodes moving
7.358 → 7.833 V and plate nodes ~1.4 V — that is the size of "coefficient
drift" this class ships as. Per §2.2 / §7 Q4, DC-OP motion silently
invalidates DC-blocker seeding assumptions, `CATHODE_DC_BIAS`-style reads,
gold-press transient-injection headroom (click/pop volts ride the bias
point), and warmup-adequacy assumptions; cascades (warpony ×8) multiply the
operating-region shift per stage. Only basic-bitch/fwf/pipe-shouter have
seam tests tight enough to catch gross level fallout; pretty-baby,
sad-bastard, uniquorn, and warpony have weaker gates. **Lesson: "the DC
operating point moved" is a first-class declared-change category, same tier
as gain and noise-magnitude changes — regens that ship it as unlabeled
coefficient drift leave every bias-pinned wrapper constant silently stale.**

### 6.5 Standing aftermath

- The KNOWN BROKEN series-of-tubes state persists until a melange-blessed
  recalibration story exists for the ×2 gm world.
- Re-pinned noyce trims/tests now encode **post-campaign** melange behavior:
  reverting or re-touching triode gm, flicker calibration, or shot draw scheme
  re-breaks them in the opposite direction.
- This document + golden baselines exist so the *next* campaign ships with a
  downstream impact list instead of a surprise.

---

## 7. Change-risk checklist for melange fix agents

Before merging a melange change, answer every applicable question. "I don't
know" means go measure (golden harness) or go read the oomox file cited above.

**Device models / solver math (gain- or bias-moving):**
1. Does any circuit in §0's inventory contain this device? (All tube plugins ×
   triode/pentode; sus-bus × VCA+opamp; pedals × diode/BJT/JFET; etc.)
2. By how many dB does steady-state output move at default pots for each
   affected circuit? Multiply per-stage deltas by cascade counts
   (series-of-tubes ×64, warpony ×8, uniquorn ×16).
3. Which §2.1 rows (OUTPUT_TRIM, VoltageRef.output_v, makeup constants) go
   stale, and is the oomox side notified with numbers?
4. Does the DC operating point move? (Invalidates cached warmup snapshots,
   `CATHODE_DC_BIAS`, DC-blocker seeding, injected-transient headroom in
   gold-press.)
5. Do THD/knee characteristics move where a plugin's *sidechain* depends on
   them (sus-bus VSCALE law, velvet-elvis leveler range)?

**Noise:**
6. Does per-category magnitude, spectral shape, or fs-scaling change? Which
   noyce variance bands / dBFS windows / temperature-scaling assertions move?
7. Is seed determinism preserved (same seed → bit-identical; salts unchanged;
   instance decorrelation)?
8. Does the draw scheme change (single/two-draw, BE vs trap)? Check
   `noise_stationarity.rs` invariants: lag-1 autocorr > −0.5, seed spread <3 dB.
9. Are the noise setters still emitted under the same `--noise` flags with the
   same names? (Silent-omission footgun.)

**Setters / dynamic params:**
10. Are `set_pot_N` index assignments still netlist-ordered and stable across
    regen? (Subspace asserts; everyone else silently trusts.)
11. Do setter semantics still hold: ohms in, clamp to range, no-op guards,
    dirty-flag deferred rebuild, reseed-free, audio-thread-safe rebuild in
    `process_sample`?
12. Do `.runtime R/V` field names, `RUNTIME_R_*_MIN/MAX`, and per-block vs
    per-sample cost expectations survive?

**Lifecycle:**
13. Does `reset()` still restore baked-coherent factory state? Does
    `set_sample_rate` keep the same-rate no-click and rate-change reseed
    contracts? Does `recompute_dc_op` still fail-soft (state untouched +
    `diag_nr_max_iter_count`)?
14. Is `process_sample`'s free-function signature and `[f64; NUM_OUTPUTS]`
    return preserved in every topology (including OS=4, multi-output,
    behavioral, coupled-L)?

**Codegen structure:**
15. Do all 42 inventory files still compile? (Includes orphan vcr-audio,
    unused tth cascade/edge, held-back subspace radio_fm at its old topology.)
16. Are struct fields wrappers poke (§1.5) still present, public, same-typed?
17. Is codegen byte-deterministic (no HashMap-ordered emission)? Regen diffs
    must be reviewable as coefficient-only.
18. Do the anchors (N, M, OVERSAMPLING_FACTOR, INPUT_NODE, OUTPUT_NODES,
    noise-fn presence) survive for every shipped circuit, or is the change
    explicitly a topology upgrade requiring per-plugin rewiring?

**Process:**
19. Have you produced a downstream impact list (plugins → constants → tests)
    *before* the fix lands, per the post-campaign review protocol?
20. Sound-changing fixes: flagged for Josh's audition before commit; regens are
    "NOT DAW-verified" by default and must say so.
