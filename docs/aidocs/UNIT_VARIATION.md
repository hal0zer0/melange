# Unit Variation: `.mismatch`, `.tolerance`, `.seed`

Three netlist directives that bake per-unit randomness into a generated
plugin at codegen time. Intended to simulate the analog character that
comes from real-world component and device-parameter spread — the thing
that makes "unit A" of a pedal sound subtly different from "unit B" and
keeps distortion from sounding like a mathematically perfect limit
cycle.

Opt-in and deterministic. Zero runtime cost when absent. The SPICE
validation suite is unaffected because it doesn't use these directives
(and by design *shouldn't* — ngspice has no concept of per-build
randomness).

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
```

Device classes: `D` (diode), `Q` (BJT). `J`/`M`/`T` are accepted by the
parser but not yet wired to the IR — those devices are pass-through.

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
| `D`   | `RS`      | `DiodeParams.rs`         |
| `Q`   | `IS`      | `BjtParams.is`           |
| `Q`   | `BF`      | `BjtParams.beta_f`       |
| `Q`   | `BR`      | `BjtParams.beta_r`       |

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

## Interaction with Self-Heating

`.mismatch` and `.tolerance` are **static** (baked once at codegen). The
BJT/diode self-heating model is **dynamic** (Tj drifts sample-by-sample
from real dissipation). They stack naturally: each self-heating device
starts from its jittered nominal `IS` / `N·VT`, then the thermal loop
scales from there as `IS(T) = IS_nom_jittered · (Tj/Tamb)^XTI · ...`.

## Validation Implications

The `.mismatch` and `.tolerance` directives intentionally break
bit-for-bit ngspice parity — ngspice doesn't understand them and will
see the netlist's nominal values. A circuit that needs SPICE correlation
tests should either omit these directives entirely or use a variant
`.cir` without them for the validation pipeline.

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
