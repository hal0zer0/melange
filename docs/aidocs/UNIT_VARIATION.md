# Unit Variation: `.mismatch`, `.tolerance`, `.seed`

Three netlist directives that bake per-unit randomness into a generated
plugin at codegen time. Intended to simulate the analog character that
comes from real-world component and device-parameter spread — the thing
that makes "unit A" of a pedal sound subtly different from "unit B" and
keeps distortion from sounding like a mathematically perfect limit
cycle.

Opt-in and deterministic. Zero runtime cost when absent. `melange validate`
turns all three OFF on melange's side and says so on its result line, so a
deck that carries them is still validatable — see
[Validation Implications](#validation-implications).

## Source Files

| Directive    | Parser                                                                             | Apply site                                                      |
|--------------|------------------------------------------------------------------------------------|-----------------------------------------------------------------|
| `.seed`      | `parser.rs::parse_directive`                                                       | Stored on `Netlist::seed`                                       |
| `.mismatch`  | `parser.rs::parse_mismatch_directive` → `MismatchSpec`                             | `codegen/ir.rs::build_device_info_with_mna` via `apply_mismatch` |
| `.tolerance` | `parser.rs::parse_tolerance_directive`                                             | `parser.rs::Netlist::apply_passive_tolerance` at end of parse    |

Shared RNG: `parser.rs::deterministic_draw(seed, class_tag, name)` →
FNV-64 → SplitMix64 finalizer → uniform `[-1, 1]`.

## `.seed` — RNG Master Seed

### Syntax
```spice
.seed 42
```

A `u64` that seeds the deterministic RNG used by both `.mismatch` and
`.tolerance`. When absent, defaults to `0`. Same seed + same netlist →
byte-identical generated code on every run.

## `.mismatch` — Device Parameter Jitter

### Syntax
```spice
.mismatch D IS=0.05 N=0.02           ; every diode: IS ±5%, N·VT ±2%
.mismatch Q IS=0.02 BF=0.05 BR=0.05  ; every BJT: IS ±2%, BF/BR ±5%
.mismatch T MU=0.03 KG1=0.05 KP=0.02 ; every tube: MU ±3%, KG1 ±5%, KP ±2%
.mismatch J IDSS=0.05 VP=0.03        ; every JFET: IDSS ±5%, VP ±3%
.mismatch M KP=0.05 VT=0.03          ; every MOSFET: KP ±5%, VT ±3%
```

Device classes: `D` (diode), `Q` (BJT), `T` (triode + pentode/beam-tetrode),
`J` (JFET), `M` (MOSFET) — all wired to the IR. The class letter matches the
`.mismatch` directive, not the element prefix; `T` covers every tube (both the
`Triode` and `Pentode`/beam-tetrode elements, which share `TubeParams`).

**Why tube mismatch matters:** a balanced push-pull tube stage with
identical-model halves cancels even harmonics *exactly* — the source of the
"~60× too clean, odd-dominant" character measured on `passive-eq1a`. Jittering
the two halves' Koren parameters per device breaks that cancellation, so H2
survives and rises ∝ V with level (the physically-correct even-harmonic
signature of real, imperfectly-matched push-pull gear). Applies to `analyze`
as well as `compile` (both go through the same IR build path).

Tolerances are dimensionless fractions in `[0, 1)`. The jitter is
`nominal · (1 + tol · u)` with `u ∈ [-1, 1]` drawn from
`deterministic_draw(seed, class_tag, device_name)`.

### Semantics

Each device of the named class gets its resolved model parameters
jittered *per device*, not per `.model` card. Two diodes pointing at the
same `.model D1N4148 D(IS=2.52e-9)` end up with distinct constants
`DEVICE_0_IS` and `DEVICE_1_IS` in the generated code. That's the whole
point — antiparallel clippers, push-pull output pairs, and "matched"
transistor pairs are never exactly matched, and the audible asymmetry
is what gives real gear its character.

Wired parameters:

| Class | Parameter | Field in `DeviceParams`  |
|-------|-----------|--------------------------|
| `D`   | `IS`      | `DiodeParams.is`         |
| `D`   | `N`       | `DiodeParams.n_vt`       |
| `D`   | `RS`      | `DiodeParams.rs` (skipped when 0) |
| `Q`   | `IS`      | `BjtParams.is`           |
| `Q`   | `BF`      | `BjtParams.beta_f`       |
| `Q`   | `BR`      | `BjtParams.beta_r`       |
| `T`   | `MU`      | `TubeParams.mu`          |
| `T`   | `EX`      | `TubeParams.ex`          |
| `T`   | `KG1`     | `TubeParams.kg1`         |
| `T`   | `KP`      | `TubeParams.kp`          |
| `T`   | `KVB`     | `TubeParams.kvb`         |
| `T`   | `KG2`     | `TubeParams.kg2` (pentode; skipped when 0) |
| `J`   | `IDSS`    | `JfetParams.idss`        |
| `J`   | `VP`      | `JfetParams.vp`          |
| `J`   | `LAMBDA`  | `JfetParams.lambda`      |
| `M`   | `KP`      | `MosfetParams.kp`        |
| `M`   | `VT`      | `MosfetParams.vt`        |
| `M`   | `LAMBDA`  | `MosfetParams.lambda`    |

The wired set is the core transfer parameters per device — the ones that
drive audible unit-to-unit character. Parasitic fields (junction caps, ohmic
`RD`/`RS` on FETs) are intentionally not jittered; add them here if a target
needs it. Tube jitter is shared by the `Triode` and `Pentode` arms via
`apply_tube_mismatch`; `KG2` is a no-op on triodes (guarded on `kg2 > 0`).

Multiple `.mismatch` directives for the same class are merged; the last
tolerance wins per-param. Unknown params on a supported class are
silently ignored (pass through as pure nominal).

## `.tolerance` — Passive R/C/L Value Jitter

### Syntax
```spice
.tolerance R=0.01                    ; 1% fixed resistors
.tolerance R=0.01 C=0.02 L=0.005     ; one directive, multiple classes
```

Classes: `R`, `C`, `L`. Tolerances are dimensionless fractions in
`[0, 1)`. Applied once at the end of `Netlist::parse()` so MNA and
everything downstream sees already-jittered values — no code in
`mna.rs`, `dk.rs`, or `codegen/` needs to change.

### Skip Set

Components under **external control** are exempt and keep their nominal
values:

- Any resistor named by `.pot` (wiper halves, after `expand_wipers`,
  appear as `.pot` entries and are skipped).
- Any resistor named by `.runtime R`.
- Any R / C / L named by `.switch`.

Jittering those would break the UI-slider-to-resistance mapping the user
explicitly defined.

### RNG Streams

The `.mismatch` and `.tolerance` directives share `deterministic_draw`
but pass different `class_tag`s (`"D"` / `"Q"` for mismatch,
`"R"` / `"C"` / `"L"` for tolerance). Together with the null-byte
separator in `deterministic_draw`, this guarantees no aliasing between
streams — e.g. a resistor named `R1` and a diode's `IS` at the same
seed produce independent draws.

## Combined Example

```spice
.seed 42
.mismatch D IS=0.05 N=0.02
.mismatch Q IS=0.02 BF=0.05
.tolerance R=0.01 C=0.02

Rin in out 1k                 ; jittered to ~994.4 Ω
D1 out 0 D1N4148              ; IS jittered per-device
D2 0 out D1N4148              ; IS jittered, independently of D1
...
```

With the values above (seed 42): D1 lands at `IS = 2.489e-9` (-1.23%),
D2 at `IS = 2.572e-9` (+2.08%). Rin drops to 994.38 Ω (-0.56%). Run the
same netlist again and get the same numbers — change `.seed 42` to
`.seed 99` and get a completely different unit.

## Choosing Magnitudes: Modern vs. Vintage Parts

(Authoring guidance, voltron analog-EE review 2026-09-13.)

The example magnitudes throughout this doc — `.tolerance R=0.01 C=0.02`,
tube `MU ±3%`, BJT `BF ±5%` — describe **modern** precision parts. Period
and vintage components are far wider, and modelling a vintage circuit with
modern tolerances understates its unit-to-unit spread badly:

| Part | Modern (examples above) | Vintage / period spread |
|------|-------------------------|-------------------------|
| Carbon-composition R | 1% | ±5 / 10 / 20% (marked band) |
| Electrolytic C | 2% | ±20% or worse |
| Film / mica C | 2% | ±5–10% |
| Tubes (per parameter) | ~3% | ±5–20% common |
| Small-signal BJT β | ~5% | wide datasheet window — e.g. 2N3904 `BF` spec is 100–300 |

So a `.mismatch T MU=0.15` or a `.tolerance R=0.10` is a *more faithful*
model of a 1960s circuit than the modern example values, and matched-pair
devices (push-pull tubes, antiparallel clippers) should be jittered *at
least* this wide unless the hardware was hand-selected.

### Two limits `.tolerance`/`.mismatch` cannot capture

1. **Ageing is directional and correlated, not uniform jitter.** The draw
   here is a symmetric uniform `u ∈ [-1, 1]` (`deterministic_draw`), which
   models *manufacturing spread* — a fresh unit off the line. Ageing is
   different: it moves parts in a *consistent direction* and often in a
   *correlated* way across a batch. Electrolytics lose capacitance and gain
   ESR as they dry; humid carbon-composition resistors drift *upward* in
   value. A symmetric `[-1, 1]` draw cannot represent that — it would need a
   separate *drift* term (a directional bias, not a re-roll). Do not reach
   for a wider `.tolerance` to fake ageing; you would get a random unit, not
   an old one.

2. **Some ageing is not a value change at all — it is a topology or model
   change**, and `.tolerance` (which only scales existing R/C/L values)
   cannot express it:
   - A **leaky coupling cap** is a new **DC path** — a high-value resistor
     appearing in parallel with the cap — that shifts the *next* stage's
     bias point. That is a new element, not a jittered value.
   - **ESR** on an aged electrolytic is a series resistance the ideal cap
     model does not have.
   - The **voltage coefficient** of a carbon-composition resistor (value
     shifting with the voltage across it) is a nonlinear model, not a
     tolerance.

   These need topology edits or richer device models, not `.tolerance`.

### Scale intuition

A ±20% spread on both R and C moves a first-order corner frequency
`f = 1/(2πRC)` by roughly **−31% to +56%** (`1/(1.2·1.2)` to `1/(0.8·0.8)`).
Useful when judging whether a chosen tolerance is audible: a filter or
tone-stack corner will wander by tens of percent, a bias divider far less.

## Interaction with Self-Heating

`.mismatch` and `.tolerance` are **static** (baked once at codegen). The
BJT/diode self-heating model is **dynamic** (Tj drifts sample-by-sample
from real dissipation). They stack naturally: each self-heating device
starts from its jittered nominal `IS` / `N·VT`, then the thermal loop
scales from there as `IS(T) = IS_nom_jittered · (Tj/Tamb)^XTI · ...`.

## Validation Implications

ngspice doesn't understand these directives and sees the netlist's nominal
values. **`melange validate` therefore disables unit variation on melange's
side too, automatically** — it compares nominal against nominal and names the
disabled directives on the PASSED/FAILED line:

```
Validation PASSED (nominal values: .mismatch T disabled for this comparison; seed 4142 not exercised)
```

A deck carrying `.mismatch` / `.tolerance` needs no edit and no variant `.cir`
to be validatable. `compile`, `simulate` and `analyze` are unaffected and
jitter exactly as documented above — only `validate` moves.

**Mechanism.** One switch, two apply sites, because the two directives land on
opposite sides of the pipeline:

| Site | What the switch does |
|------|----------------------|
| `parser.rs::Netlist::apply_passive_tolerance` | early-returns, so `.tolerance` never scales an R/C/L |
| `codegen/ir/mod.rs::CircuitIR::mismatch_tol_for` | returns `0.0`, which makes `apply_mismatch` a bit-identical pass-through |

Both read `Netlist::unit_variation_disabled`, set by
`Netlist::parse_with_options(deck, ParseOptions { disable_unit_variation: true })`.
The flag rides on the *netlist* — the object that carries the directives —
rather than on `CodegenConfig`, so the two sites cannot desynchronize. The
directives stay recorded either way, because the result line has to be able to
name what was turned off. `melange-validate` sets it in
`run_melange_solver_from_str`, unconditionally; the ngspice-side helpers
(`tube_translate`, `pentode_translate`, `substitute_dynamic_element_defaults`,
`warn_floating_cap_only_islands`) keep the plain `Netlist::parse` and so keep
seeing the deck exactly as written.

**What this does and does not measure.** validate measures the solver against
a reference engine *at the same component values*. Jitter changes values, not
the solver. Whether the *draw itself* is right is a unit-test question ngspice
cannot answer, and it is answered by:

- `melange-solver/src/parser.rs::tests::tolerance_draw_matches_nominal_times_one_plus_tol_u`
- `melange-solver/src/parser.rs::tests::deterministic_draw_matches_independent_reimplementation`
- `melange-solver/tests/codegen_verification_tests.rs::mismatch_draw_matches_nominal_times_one_plus_tol_u`

Each asserts `applied = nominal · (1 + tol · u)` against a `u` derived from an
independent reimplementation of the FNV-64 → SplitMix64 chain, not read back
out of melange.

Substituting the *jittered* values into the ngspice deck was considered and
rejected (arbiter, 2026-09-22): a correlation metric is dominated by the
fundamental and cannot grade an error in a −40 dB H2 residual, so it would add
a regime the score cannot see. The push-pull H2 that `.mismatch T` exists to
create is exactly such a residual — on `passive-eq1a` the nominal comparison
reports THD (SPICE) −113.9 dB against THD (melange) −153.7 dB and still passes,
because the THD gate is (correctly) exempt when melange is the cleaner of two
noise floors.

The regression guard for "absent = byte-identical" is:

- `crates/melange-solver/tests/codegen_verification_tests.rs::test_no_mismatch_is_byte_identical`
- `crates/melange-solver/src/parser.rs::tests::test_tolerance_absent_is_no_op`
- `crates/melange-solver/src/parser.rs::tests::test_mismatch_absent_is_no_op`

If any of those start failing, the no-op path has drifted.

## Design Rationale

- **Why bake at codegen, not jitter at runtime?** A plugin's
  "personality" shouldn't re-roll on every instantiation. Baking means
  a preset always sounds the same, which is what users expect.
- **Why device parameter mismatch at all?** Matched-pair asymmetry is
  a far bigger audible driver than passive tolerance alone. A pedal
  with 0% passive tolerance but 2% mismatched clipping diodes sounds
  more "analog" than one with 5% passive tolerance and bit-matched
  diodes.
- **Why FNV → SplitMix64 instead of a crate RNG?** Zero deps, stable
  output across Rust versions (no reliance on stdlib hasher internals),
  and enough output quality for a perturbation at the `±1e-2` scale.
