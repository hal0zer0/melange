# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
It is `0.x` software: below 1.0.0 there is no stability guarantee — the solver,
codegen output, CLI flags, and netlist semantics may all change.

## [Unreleased]

## [0.1.6] - 2026-09-04

A correctness-and-diagnostics patch. The headline is an accuracy fix: the
default pentode grid-off reduction is retired because it was not
accuracy-neutral. Two further fixes stop `melange validate` and `melange
analyze` from measuring a circuit the compiler does not ship, `simulate` learns
to drive `.inject` sources, and `melange --version` now carries the build
commit.

**Generated DSP output is not byte-identical to 0.1.5, but the audible change is
confined to one class of circuit — a beam-tetrode/pentode stage that the default
previously grid-off-reduced.** What moves, and what does not:

- **Generated source changes for every circuit**, because every emitter now
  carries a new `diag_region_exit_count` diagnostic counter (see Added). This is
  a source-level diff on all decks; it does not by itself change rendered audio.
- **Rendered audio changes only on pentode/beam-tetrode decks that the old
  default reduced** (grid-off 3D→2D under `--tube-grid-fa auto`). Triode decks
  are byte-identical; every non-pentode circuit is byte-identical. On the
  golden corpus, only the reducing pentode decks move — toward ngspice (e.g.
  `noyce-ef86` peak 0.673→0.659, removing a +2.18% small-signal gain error).
- **Blast radius on shipped product is zero.** The `melange validate` false-pass
  this release fixes existed only on unreleased `main` (it needed the shared
  front end added the same cycle); no tagged release ever carried it. OpenWurli,
  the only shipped product, is a pure-BJT signal path with no pentode and no
  coupled inductor, so none of its generated audio changes. Any downstream that
  pins generated code for a **pentode** circuit should regenerate deliberately
  and re-audition; DK-routed and triode-only circuits regenerate byte-identical
  audio (their source still changes for the new counter).

MSRV is unchanged (1.85). No dependency changed in this release.

**Known limitations carried forward (unfixed, disclosed):**

- **DK routing is wrong on coupled-inductor circuits** — on a deck with a
  coupled-inductor/transformer output stage that routes to the DK solver,
  `--solver dk` produces a non-physical mean plate voltage *above* B+ (measured
  on `twill-deluxe`: DK plate mean 323 V vs a 320 V rail, versus nodal 312.5 V).
  It is latent: no released deck reaches it, and the pentode change below removes
  the one path that used to trigger it (grid-off reduction pulling `M` under the
  nodal threshold and flipping the router to DK). The DK defect itself is
  deferred to a separate ticket. The population is "coupled inductors on a DK
  route", not pentode- or product-bounded; OpenWurli has no coupled inductors.
- **BJT forward-active auto-reduction is unchanged**, pending the counter data
  the new `diag_region_exit_count` is there to collect. The shipped product
  reduces and has no measured failure; the default decision is deferred, not
  made.
- **An exact (accuracy-safe) grid-off reduction that recovers the lost
  performance is deferred.** The current fix keeps the full 3D model by default
  rather than reducing.

### Added

- **`melange simulate --inject FIELD=SPEC`** drives `.inject` runtime sources.
  `simulate` previously built its IR with an empty injection set, silently
  dropping every `.inject` source — so a circuit whose signal path is an
  internal circuit-volts injection (an oscillator/sync injection, a bias
  modulation) could not be driven at all. Values are **circuit volts**, stamped
  at the `.inject` node through its declared physical impedance; the flag is
  repeatable. `SPEC` is `sine:<freq_hz>:<amp_volts>` or `dc:<volts>` (WAV
  deferred). An unnamed field warns and injects 0; an unknown field errors with
  the valid field list. The `NUM_INJECT == 0` path is byte-identical to before.
  `analyze` still drops `.inject` — a flagged follow-up, not bundled here.
- **`melange --version` now includes the build commit** — e.g. `0.1.6
  (71709c7)` — so a released tag, an unreleased `main`, and a local build are
  distinguishable. Three builds all printing a bare `0.1.5` previously caused a
  cross-repo misdiagnosis (an unreleased-`main` regression read as a released
  build). Degrades to `(unknown)` for a packaged crate with no `.git`.
- **`diag_region_exit_count` diagnostic counter** in every generated circuit —
  counts, on the full (unreduced) model, samples where a pentode's grid conducts
  (`Vgk > 0`) or a BJT saturates (`Vbc` forward). It is the instrumentation that
  produces attribution data under the new full-3D default and for the still-auto
  BJT forward-active reduction. Diagnostic only; does not affect audio output.
- **`melange validate` accepts `--bjt-fa {auto|off|force}` and `--tube-grid-fa
  {auto|on|off}`** — the existing mechanism flags, plumbed through so the
  validator can exercise the same reduction the shipped build uses. No new flag
  name was invented for two mechanisms that already have one.

### Changed

- **Pentode grid-off reduction no longer runs by default; `--tube-grid-fa auto`
  now keeps the full 3D model (`auto` == `off`).** The reduction froze `Vg2k =
  V[screen] − V[cathode]` at its DC value, but `Vg2k` is cathode-referenced: an
  unbypassed cathode resistor or screen-stop makes it move with signal, and
  freezing it discards the local negative feedback through `dIp/dVg2k` — a
  small-signal gain error present every sample (measured vs ngspice: EF86 +2.2%,
  EL84 +3.0%, EL84 with a 1 kΩ screen-stop +12.3%). It also dropped grid current
  `Ig1`, so a stage driven into grid conduction silently ran a model with no
  grid current. Neither loss is boundable from a quiescent bias, so there is no
  sound automatic selection. `on` remains available as a **warned, explicit
  opt-in** (exact only for a fully-bypassed screen). A route-parity guard skips
  the `on` reduction when the unreduced circuit routes nodal, so a reduction can
  never lower `M` far enough to flip the router nodal→DK. Validated full-3D
  against ngspice: `twill-deluxe` 0.063%, `el84-single-stage` 0.233%,
  `noyce-6bq5` 0.060%, `noyce-ef86` 0.060%.

### Fixed

- **`melange validate` was verifying a model the compiler does not ship.** After
  the 0.1.5 front-end unification, forward-active and grid-off reduction stayed
  private to `melange-cli`, so `validate` (and the `spice_validation.rs` CI
  harness, which built its own MNA and ran none of the shared steps) validated a
  full-2D/full-3D system for circuits the shipped build reduces — a false pass.
  `should_skip_fa_for_nodal_reroute`, `apply_forward_active_reduction`,
  `apply_grid_off_reduction` and the grid-off log now live in
  `melange_solver::pipeline`; all five consumers — compile, simulate, analyze,
  `melange validate`, and the SPICE test harness — route through it. This bug
  existed only on unreleased `main`; no tagged release carried it.
- **`melange analyze` expanded parasitic-BJT internal nodes unconditionally**,
  while compile, simulate and validate skip that expansion when `K` is
  ill-conditioned (`k_diag_min < −100`, the full-`N` LU path handling parasitics
  directly). So `analyze` reported the frequency response of a *different*
  circuit than compile ships for every ill-conditioned-`K` deck (measured on the
  OpenWurli power stage: compile skips the expansion, analyze did not). All four
  consumers now call the shared conditioning gate; none hand-rolls the threshold.
  compile and simulate output is unchanged (golden `--strict`: 168 identical, 0
  changed).

## [0.1.5] - 2026-09-03

A verification-and-correctness release. Two silent-wrong-output bugs in the
nodal emitter are fixed, `melange validate` stops verifying a circuit the
compiler never builds, and `.model` cards that name a parameter melange does
not know are now rejected instead of quietly ignored.

It also resolves a version-reporting gap: `main` was advanced past the `v0.1.4`
tag without a version bump, so a build from `main` between those commits
reported `melange 0.1.4` while not being the tagged 0.1.4. Anything built from
`main` since 2026-08-30 should be rebuilt from this tag.

**Generated DSP output is not byte-identical to 0.1.4**, but the audible change
is confined to one class of circuit. What moves, and what does not:

- **Generated source changes for essentially every circuit.** Nodal circuits get
  a rewritten `reset()` body (F9); DK circuits get the new `N_I` layout. Both
  are emitted together with the code that reads them, so a regenerated file is
  self-consistent.
- **Rendered audio is unchanged except on noise-enabled circuits that hit one of
  three RHS rebuild paths** (F10). Measured on the 43-circuit / 196-render
  golden corpus in place when the fixes landed: 194 renders identical, 1
  negligible, 1 changed. The one changed render is a noise-enabled tube circuit
  whose broadband level moves +0.001 dB with every octave band inside 0.002 dB;
  its per-sample waveform differs because noise that was previously dropped on
  sub-step samples is now stamped.
- **F9 cannot change `melange simulate` or `melange analyze` output.** Neither
  command calls the generated `reset()`. F9 changes what a *generated plugin*
  does after its host resets it — transport stop, `initialize()` — which is
  where the bug lived.
- **Who should regenerate.** Any generated plugin, for F9: a host reset after a
  pot move previously left the solver running the moved value while every getter
  reported nominal. For OpenWurli specifically: `gen_preamp.rs` is DK-routed and
  its rendered output is unchanged (its source changes for the `N_I` layout);
  `gen_power_amp.rs` and `gen_tremolo.rs` are nodal and pick up the `reset()`
  fix.

MSRV is unchanged (1.85). No dependency changed in this release.

### Added

- **`--nodal-subpath {auto|schur|full-lu}`** on `compile` and `analyze` — pins
  the nodal solver's sub-path instead of letting the router choose. A
  diagnostic control for isolating Schur-versus-full-LU behaviour on a circuit;
  `auto` is the default and the previous behaviour.
- **SPICE `XTB` is honored** on self-heating BJTs — forward and reverse beta now
  carry their temperature dependence instead of being held at the nominal-
  temperature value. Affects only `.model` cards that supply `XTB` on a device
  with `RTH`/`CTH`.
- **`melange validate` can translate pentode (`P`) elements to ngspice**, in all
  three screen-current forms. Ten decks in the circuit library previously had no
  reference oracle at all.
- **Golden harness verification depth.** Renders are captured at f64 rather than
  f32; `compare --strict` adds a refactor gate that passes only on bit-identical
  renders *and* identical generated source; generated `circuit.rs` is diffed;
  solver diagnostic counters (`diag_*`) are recorded and gated; and a capture
  now detects when the `melange` binary used does not match the checked-out
  source.

### Changed

- **Unknown `.model` parameters are now a hard error.** A `.model` key is sorted
  into three tiers: *honored* (silent), *recognized but unimplemented* (warns,
  naming what the omission costs), and *unknown for this device type* (hard
  error, listing the accepted keys plus an alias hint — `VP=` on a JFET card now
  points at `VTO` and warns that the sign convention differs). Previously all
  three warned and continued, so a typo'd key could still produce a plausible
  result for the wrong reason. Blast radius was measured, not assumed: of the 85
  netlists in the circuit library, zero now fail to compile. `TR` and `XCJC`
  stay in the middle tier — they are real SPICE keys that arrive on authentic
  vendor model cards, and erroring on them would mean refusing genuine SPICE
  decks over a gap of melange's own. The change immediately found a real defect:
  `SHOT_GAMMA2` is read by the noise layer but was missing from the tube
  resolver's honored list, so melange had been emitting a false "ignored"
  warning for a parameter it actually uses.
- **`N_I` is emitted in one layout, `[[f64; M]; N]`, on every solver path.** It
  was stored transposed depending on the solver — `[[f64; N]; M]` on DK,
  `[[f64; M]; N]` on both nodal paths — under a single public symbol, while
  `N_V` was uniform. On a circuit where `N == M` that is wrong with no crash and
  no shape error. Generated code and its uses move together, so regenerating is
  sufficient; any code that reads `N_I` from *outside* a generated file must
  swap its index order.

### Fixed

- **Nodal `reset()` left the working matrices stale and rate-blind.** `reset()`
  restored `g_work`/`c_work` and every pot and switch field to nominal but never
  restored `a`/`a_neg`/`a_be`/`a_neg_be`, and never set `matrices_dirty`. After
  `set_pot(x)` → `process_sample()` → `reset()`, the working A matrices still
  carried `x` with no rebuild scheduled, so every getter reported nominal while
  the solver ran the moved value; it healed only on a later
  `set_pot_*`/`set_switch_*`/`set_sample_rate`. Separately, the `*_DEFAULT`
  constants bake the codegen sample rate, and `reset()` reloaded them regardless
  of the live rate, so the two halves of the restore disagreed whenever the host
  ran at another rate. The matrix restore now happens at the end of `reset()`
  and dispatches on the live rate exactly as `set_sample_rate()` does. The DK
  path always set `matrices_dirty` here; only the nodal emitter did not.
- **Noise was silently dropped on three from-scratch RHS rebuilds.** A sample's
  noise draws are consumed once when the primary RHS is built and cached so
  later rebuilds within the same sample can re-stamp them without touching the
  RNG. Three rebuild paths did not re-stamp: Schur breakpoint-BE with `M == 0`,
  the Schur ActiveSetBe sub-step, and the full-LU adaptive sub-step. On any
  sample routed through one of those the noise vanished — while its draws had
  already been consumed, so the RNG stream stayed aligned and the loss was
  invisible to a determinism check.
- **Pentode variable-mu parameters were parsed, validated, and then discarded.**
  `MU_B`, `SVAR` and `EX_B` were read off the `.model` card and checked, and the
  resolver then hardcoded all three to zero, so a variable-mu pentode passed
  validation and silently compiled as a sharp-cutoff device. Byte-identical for
  every sharp pentode (`SVAR = 0` is the unchanged path); changes emitted DSP
  only for `SVAR > 0` decks.
- **`melange validate` was verifying a different circuit than `compile` ships.**
  Four consumers — compile, simulate, analyze, validate — had each grown a copy
  of the front-end pipeline and drifted three ways, the consequential one being
  that validate skipped `.linearize` reductions. On the shipped Wurlitzer power
  amp, validate built an N=44, M=16 system where compile builds N=20, M=14, took
  a different solver sub-path, and reported 1319% RMS error against ngspice —
  which read as a catastrophic solver defect and was a harness artifact. All
  four now share one front end (`melange_solver::pipeline`). The same deck now
  validates at 0.2461% RMS error, correlation 0.99999964, SNR +52.18 dB.
- **A `.model` card that resolved to no catalogue part and supplied no
  device-defining parameter silently fell back to the built-in default device.**
  A typo'd tube name compiled as a 12AX7 with no diagnostic. The
  BJT/JFET/MOSFET/triode/pentode/LDR resolvers now warn on exactly that case.
  Log-only; DSP is byte-identical.
- **The KiCad reference netlist declared `MIT OR Apache-2.0`.** Every crate is
  GPL-3.0-or-later via the workspace and the project is GPL throughout; that
  line was never intended. Comment-only, DSP byte-identical.

### Documentation

- **FAUST is retracted as a planned codegen backend.** It was published as
  planned in four places and does not work: FAUST's generated code is
  deliberately not Turing-complete — each sample costs a fixed number of
  operations — so a Newton-Raphson solve with a data-dependent iteration count
  cannot be expressed, and GRAME's own FAQ names the diode-model Newton
  approximation as the blocking case. Only circuits emitting no NR loop at all
  would be expressible: 6 of 41 in the golden corpus. All four sites now say
  explored-and-impractical.

## [0.1.4] - 2026-08-30

A correctness-sweep release: solver-accuracy and codegen-honesty fixes, no new
features. The headline is a globalized nodal Newton-Raphson (residual-gated
convergence + Armijo line search) that fixes a real MIC-drive limit cycle, plus
a batch of narrower correctness fixes across MNA stamping, DC-OP retention,
routing guards, and the parser. The validate harness also got its grading
tightened.

**Generated DSP output is not byte-identical to 0.1.3.** The change that moves
the compiled audio path is the **nodal NR globalization** (see Changed): it
alters the generated code for circuits routed to the **full-nodal solver with
`M > 0`** (the nodal full-LU path). Concretely, across melange's own golden
corpus every generated-output change this cycle is attributable to that one
change — there are zero unattributable diffs — and the corpus audio impact is
tiny and benign (only two decks move audibly at all: a sub-mV pot-position tail
on `uniquorn`, and an idle shot-noise reshuffle on `tungsten-thunder-horse`;
both re-converge to the pre-sweep result, neither is a regression).

**Who needs to regenerate.** Anything routed to the **DK method is unaffected**
and regenerates byte-identically. Anything routed to the **nodal solver with
`M > 0`** changes (more correctly). For the one shipped downstream, OpenWurli:
`gen_preamp.rs` is **DK-routed and unaffected**; `gen_power_amp.rs` (nodal,
N=20 M=14) and `gen_tremolo.rs` (nodal, N=7 M=4) **will change if regenerated
against 0.1.4** — the change adds the residual gate + line search and is
more-correct, but any downstream that pins generated code should regenerate
deliberately and re-audition. MSRV is unchanged (1.85).

### Changed

- **Nodal Newton-Raphson is now globalized: residual-gated convergence with an
  Armijo line search.** The nodal full-LU path previously accepted a step on a
  bare step-size check, which allowed a limit cycle on hard-driving inputs (the
  motivating case: a MIC-drive stage that never settled). Convergence is now
  gated on the actual residual, and each step is backtracked with an Armijo line
  search; on line-search failure the solver **falls through** to the
  un-line-searched limited step (the residual gate still decides convergence)
  rather than bailing to a removed fallback. This changes generated output for
  nodal `M > 0` circuits and is the reason this release is not byte-identical to
  0.1.3. Device evaluations inside the residual gate are reused (no extra
  per-iteration cost from the gate itself); measured throughput of the bundled
  passive-EQ demo (nodal N=52 M=8) is ~25x realtime at 48 kHz on a Ryzen 9
  7950X, unchanged from 0.1.3 within rounding.
- **`melange validate` grading hardened.** The comparison harness now fails on a
  reference/actual **length mismatch** (previously silently truncated), rejects a
  **silent reference** (a near-zero reference can no longer manufacture a passing
  correlation), and closes a **one-sided-constant correlation** hole where a flat
  actual signal could correlate spuriously. A dead `full_scale` config field was
  removed. Validation-only — no effect on generated DSP.

### Fixed

- **Independent current-source sign corrected to match SPICE/ngspice.** MNA
  stamping of independent current sources used the wrong sign; it now matches the
  SPICE/ngspice convention. Changes output only for circuits that use an
  independent current source (none in the OpenWurli signal path or the golden
  corpus at the captured settings).
- **Zero-delay feedback resolved in `TptLpf` (ZDF one-pole).** The topology-
  preserving one-pole had an unresolved zero-delay feedback path; it is now
  solved directly. Affects circuits whose generated code uses this primitive
  (e.g. oversampling filter paths).
- **DC operating point retains converged candidates instead of discarding them
  on a leaky gate.** A too-eager gate could throw away a genuinely-converged
  low-bias operating point; converged candidates are now retained, improving DC
  bias correctness on marginal circuits.
- **Coupled-inductor determinant division is guarded, and 0-ohm switch/pot
  overrides are handled.** A coupled-inductor block with a near-singular
  determinant, and a switch/pot override that drives a resistor to 0 ohm, no
  longer produce a divide/degenerate result.
- **Routing guards, so a mis-routed compile errors instead of emitting wrong
  code:** DK backward-Euler is rejected when the circuit has companion-modeled
  inductors; a forced `--solver dk` on a structurally-nodal circuit is rejected
  with a clear error; source-dropping fallbacks no longer run on behavioral
  circuits.
- **Parser rejects a `.runtime R` and a `.switch` claiming the same resistor**
  (previously accepted, with undefined precedence).
- **CLI escapes `--vendor` / `--vendor-url` / `--email` / `--clap-id`** when
  interpolating them into generated code, so a value containing quotes or
  backslashes can no longer break (or inject into) the generated project.
- **`validate` no longer mistakes the SPICE title line for a VIN source** when
  stripping the input for the ngspice twin.

### Documentation

- Corrected several aidoc/code mismatches found during the sweep, and grounded
  the DC-OP Gmin-gate rationale in ngspice behavior rather than a single fixture.
  Doc-only.

## [0.1.3] - 2026-08-26

The "melange demos itself" release: a self-contained built-in demo circuit,
per-device mismatch reaching tubes/FETs, and a pass of onboarding fixes from
cold "follow the README" runs.

**Generated DSP output is not byte-identical to 0.1.2.** Two deliberate changes
move the compiled audio path: the bundled passive-EQ example now *colors* (see
Changed), and `.mismatch` on tubes/JFETs/MOSFETs now reaches the IR (see Added).
Both are intentional, more-correct changes, not regressions. `.mismatch` remains
**byte-identical when the directive is absent**, so any circuit that does not use
tubes, JFET/MOSFET mismatch, or the bundled passive-EQ is unaffected and does not
need to regenerate. MSRV is unchanged (1.85).

### Added

- **`.mismatch` now reaches tubes, JFETs, and MOSFETs.** Per-device `.model`
  parameter jitter (`.mismatch T|J|M P=tol …`) was parser-accepted but a no-op in
  the IR for these classes (only diodes `D` and BJTs `Q` were wired). It now
  applies to vacuum triodes/pentodes (Koren `MU`/`EX`/`KG1`/`KP`/`KVB`, plus
  pentode `KG2` when present), JFETs (`IDSS`/`VP`/`LAMBDA`), and MOSFETs
  (`KP`/`VT`/`LAMBDA`). This closes the 0.1.0 note that "`.mismatch` on `J`/`M`/`T`
  parses but is not yet wired into the IR." Byte-identical when the directive is
  absent (tolerance 0 returns the nominal parameter); `analyze` applies it via the
  same IR path. The motivating case: a balanced push-pull tube stage with
  identical model halves cancels even harmonics exactly, so per-device tube
  mismatch is the physically-honest path to the H2 real imperfectly-matched gear
  produces.
- **Built-in demo circuit — `melange {compile,simulate,analyze} passive-eq1a`**
  runs with **no external circuit source**. The passive-EQ netlist is embedded
  (`include_str!` from [`examples/passive-eq1a.cir`](examples/passive-eq1a.cir),
  so the builtin and the bundled example are the same bytes) and resolves ahead of
  any configured source, so a bare name never 404s. `melange builtins` lists it.
  melange can now compile, simulate, analyze, and demo itself offline with zero
  external dependencies.
- **`melange nodes` now lists Controls.** Pots, wipers, switches, and gangs are
  shown with their labels, component names, ranges, and defaults — previously the
  only way to learn a valid `--pot`/`--switch` name was to guess wrong and read
  the error.

### Changed

- **The bundled passive-EQ example now colors.** Re-synced to the canonical
  melange-circuits deck, it carries sourced push-pull tube mismatch (`.seed` +
  `.mismatch T` from published tube acceptance limits), producing H2-dominant,
  level-progressive distortion like the real unit rather than the previous
  idealized linear-iron behavior. **This changes the example's generated DSP
  output** (it EQ'd like the original before; it now colors like it too).
- **`melange validate` default gate retuned to audio-grade.** The default was
  `strict()` (0.01% RMS) — tighter than every per-circuit CI tolerance, so it
  reported FAILED on genuinely-good complex circuits (e.g. the passive-EQ at
  0.23% RMS with 7-nines correlation). The default is now audio-grade (0.5% RMS /
  3% max-rel / 0.9999 correlation anchor / 1.5 dB THD). `strict()`/`relaxed()` are
  unchanged and the CI validation tests pin their own tolerances, so they are
  unaffected.
- **Condition-number warning calmed.** It fired at κ≈1e12 and read as "results may
  be inaccurate," spamming near-unity-coupling transformer circuits whose iron
  legitimately sits ~7e12. Reworded to a calm explanatory note and raised to 1e13;
  genuinely extreme conditioning still warns. Diagnostic only — no solver output
  change.

### Fixed

- **Generated plugin projects now declare their own `[workspace]`.** Generating a
  plugin *inside* the melange repo — the README's own instruction — previously
  errored ("believes it's in a workspace when it's not"). The generated
  `Cargo.toml` now carries an empty `[workspace]` so it builds standalone.
- **Generated README link 404 fixed** (`github.com/melange` →
  `github.com/hal0zer0/melange`).
- **README quick-start accuracy.** `cargo build --release` yields a raw library,
  not a DAW bundle (bundling is a separate xtask step); the quick-start and
  Spotlight now use the built-in demo instead of an unreachable `melange:` source;
  the analyze example uses `--pot "LF Boost=10k"` (the prior `=10` read as 10 Ω —
  pot at minimum, boost off — so the headline example looked flat); "no external
  dependencies" is clarified (true for the standalone generated `circuit.rs`, but
  the nih-plug plugin project pulls `nih_plug` as a git dep and needs network on
  first build); the post-compile hint leads with `cargo build --release`; control
  names are per-circuit and point to `melange nodes <circuit>`.
- **SIGPIPE no longer panics.** `melange <cmd> | head` panicked (exit 101, "Broken
  pipe") on large output; a std-only panic hook now swallows the broken-pipe
  stdout/stderr panic and exits 0. Other panics fall through. No new dependency.

### Removed

- **Pre-seeded dead circuit sources.** The hardcoded default source config pointed
  at `melange-audio/circuits` and `tonestack/tonestack` — both 404 (neither repo
  exists), so a fresh install shipped a dead default source and advertised links
  that don't resolve. A fresh install now ships **no** external sources (the
  passive-eq1a builtin still lets melange demo itself); add your own with
  `melange sources add <name> <url>`.

## [0.1.2] - 2026-08-26 — Sumac

### Added

- **`integration_source` in the generated `// provenance:` JSON** — `"explicit"`
  (`.integrator be` / `--backward-euler`), `"auto-promoted"` (the trap-stability
  discriminator), `"behavioral"` (behavioral-`B`-source forced), or `"trap"`. The
  human `Build:` line already carried this distinction; the JSON now does too, so
  a consumer can assert *why* the integration scheme is in effect without
  string-matching the label.

### Changed

- **Circuits are referred to by function/topology, not brand** across the
  examples and docs (e.g. "passive tube EQ", not "Pultec EQP-1A"); brand names
  appear only as style references ("Pultec-style").
- **Bundled passive-EQ example reframed to its `testing/`-tier status.** The
  canonical circuit was promoted `unstable/` → `testing/` in the circuits
  library; docs now compile it by the `testing/filters/passive-eq1a` path and
  describe it honestly — measured-verified against the factory curve charts,
  **never auditioned**, with **idealized (linear-iron) distortion** (it EQs like
  the original, it does not yet color like it).

### Fixed

- **Generated-code provenance commit no longer goes stale on local branch
  builds** — `build.rs` now watches `.git/logs/HEAD` (updated on every commit),
  not only `.git/HEAD` (unchanged on a branch), so `// melange: <ver> (<commit>)`
  tracks the actual HEAD.

## [0.1.1] - 2026-08-26 — Mace

A hardening and provenance patch. No change to generated DSP: the compiled
audio path is byte-identical to 0.1.0 (verified at the source level — every
codegen change is an added comment header or a never-called `pub` item).
Downstream consumers (OpenWurli/oomox) do **not** need to regenerate. MSRV is
unchanged (1.85).

### Added

- **Self-describing generated code.** Emitted circuits now carry a provenance
  header — a `// melange: <version> (<commit>)` line, an extended `Build:` line
  covering the fully *resolved* DSP-affecting flags (integration, `dc-block`,
  `noise`, `opamp-rail`, `bjt-fa`, oversampling, `max_iter`), and a
  machine-readable `// provenance: {…}` JSON line a consumer can assert against
  at compile time. This makes a silent DSP-contract difference (e.g. `--bjt-fa
  force` vs `auto`, or an unexpected output DC-block) visible in the artifact
  itself.
- **Node-name → DC-OP map in generated code.** A `pub const NODE_NAMES: [&str; N]`
  array (parallel to `DC_OP`, unnamed augmented rows are `""`) plus a
  `dc_op_by_name(name) -> Option<f64>` lookup, emitted by **both** the DK and
  nodal paths. Reading one node's baked operating point is now a lookup instead
  of recompiling the netlist per node.
- **Bundled example.** The Pultec-style passive tube EQ ships in-tree at
  [`examples/passive-eq1a.cir`](examples/passive-eq1a.cir) (a byte-identical
  mirror of the canonical melange-circuits netlist) so a real circuit can be
  compiled without first wiring up a circuit source.

### Fixed

- **SPICE validation now compares the same circuit on both sides.** The ngspice
  reference deck previously kept each `.pot` / `.switch` element at its netlist
  *nominal* value while melange used the element's compiled *default*; when the
  two differed, `melange validate` was correlating two different circuits. The
  reference deck now substitutes each element's melange default (`.gang` is
  intentionally excluded — it is a UI grouping, not baked state). Example:
  passive-eq1a correlation rose from 0.808 to 0.99999728.

### Security

- **`quick-xml` 0.37 → 0.41.0**, clearing the two waived KiCad-import advisories
  RUSTSEC-2026-0194 (quadratic parse on duplicate attributes) and
  RUSTSEC-2026-0195 (unbounded namespace allocation). `cargo audit` is clean.
  MSRV 1.85 is held (0.42 was excluded — its MSRV 1.86 exceeds ours; 0.41.0's is
  1.79). The reader-API migration also fixed a latent entity-drop bug in
  `melange import` (0.41 emits `Event::GeneralRef` separately), covered by two
  new regression tests. Closes the 0.1.1 follow-up tracked in the 0.1.0
  `Security` note.

## [0.1.0] - 2026-08-25 — Saffron

First tagged release. Melange compiles SPICE netlists to standalone, real-time-safe
Rust DSP code (raw code or a nih-plug plugin project). This entry consolidates the
project's development to date into the feature set as it stands at the tag.

**Validation status is deliberately narrow.** The solver numerics and device models
are verified oracle-free to machine precision (Tellegen power-balance on the compiled
binary, trapezoidal integration order by Richardson extrapolation, hand-computed device
anchors). Generated code is compared sample-by-sample against ngspice as a corroborating
peer check. Only **one** circuit — the Wurlitzer 200A preamp — has been checked against
measured real hardware. Everything else is unproven against hardware. See
[README](README.md) "Correctness & Validation" and `docs/aidocs/STATUS.md`.

### Added

- **Compilation pipeline**: SPICE netlist → parser → MNA assembly → DK kernel →
  language-agnostic `CircuitIR` → Rust emitter. Generated DSP is completely
  standalone with zero runtime dependency on melange. Subcircuit expansion
  (`.subckt` / `X` elements, up to 8 levels of nesting).
- **Three auto-selected solver routes**: DK Schur (M<10, ≤1 transformer,
  well-conditioned K), Nodal Schur (medium complexity), and Nodal full LU (K≈0 as in
  VCA circuits, positive-K diagonal, or ill-conditioned K/S). Newton-Raphson at
  dimensions M=1 (direct), M=2 (Cramer's), and M=3..16 (Gaussian elimination with
  partial pivoting); `MAX_M=24`. Full-LU path stacks the chord method, cross-timestep
  Jacobian persistence, and compile-time sparse LU (AMD ordering, symbolic
  factorization). See `docs/aidocs/DK_METHOD.md`, `NR_SOLVER.md`, `LINEAR_ALGEBRA.md`.
- **Trapezoidal companion models** for capacitors and inductors; inductors and
  transformers via augmented MNA (branch-current unknowns), well-conditioned for large
  inductances. Coupled inductors and multi-winding transformers supported. Optional
  backward-Euler integration (`--backward-euler`) for unconditionally stable solving of
  high-gain feedback circuits, plus a nodal auto-BE promoter (with `--force-trap`
  escape hatch) and the `.integrator {trap|be}` netlist directive.
- **DC operating point solver**: LU with partial pivoting, logarithmic junction-aware
  voltage limiting (pnjlim/fetlim style), source stepping and Gmin stepping fallbacks,
  ngspice-style internal nodes for parasitic BJTs, and a low-rate warmup for circuits
  whose direct DC-OP does not converge. See `docs/aidocs/DC_OP.md`.
- **Device models** (standard SPICE `.model` parameter names throughout):
  - **Diode** — Shockley + series resistance `RS` + junction capacitance `CJO` +
    `BV`/`IBV` Zener breakdown.
  - **BJT** — Gummel-Poon (`VAF`/`VAR`/`IKF`/`IKR`, `CJE`/`CJC`, `NF`/`ISE`/`NE`)
    following ngspice `bjtload.c`, with Ebers-Moll fallback and `RB`/`RC`/`RE`
    parasitic resistances. See `docs/aidocs/GUMMEL_POON.md`.
  - **JFET / MOSFET** — 2D Shichman-Hodges / SPICE Level 1, with `CGS`/`CGD` junction
    caps, `RD`/`RS` parasitics, and MOSFET body effect (`GAMMA`/`PHI`).
  - **Vacuum triode** — Norman Koren plate model + Marshall Leach grid current,
    `CCG`/`CGP`/`CCP` junction caps, `RGI` grid-stop.
  - **Vacuum pentode / beam tetrode** — screen-current equation families (Reefman
    Rational §4.4, Reefman Exponential §4.5, Classical Koren, plus a variable-mu
    blend), placed with the `P` element and `VP` model token. Grid-off dimension
    reduction (3D→2D) under `--tube-grid-fa {auto|on|off}`.
  - **Op-amp** — Boyle VCCS macromodel with `GBW` dominant-pole node, `VSAT` output
    clamping, asymmetric `VCC`/`VEE` rails, optional `SR=` slew-rate limiting (V/µs),
    and selectable rail-saturation strategy via `--opamp-rail-mode
    {auto|none|hard|active-set|active-set-be|boyle-diodes}`.
  - **VCA** — THAT 2180 / DBX 2150 Blackmer current-mode exponential gain with
    gain-dependent THD, placed with the `Y` element.
  - **CdS LDR (opto)** — VTL5C-class photocell with attack/release dynamics on the
    stateful-device codegen path, placed with the `O` element. No ngspice twin (SPICE
    has no LDR model).

  See `docs/aidocs/DEVICE_MODELS.md`.
- **Dynamic parameters**: `.pot` / `.wiper` / `.switch` (up to 16) / `.gang` /
  `.runtime` directives. Pot and switch changes trigger a per-block matrix rebuild;
  per-sample smoothing is available for knob moves. `--format plugin` maps these to
  nih-plug parameters. `recompute_dc_op()` (opt-in) re-seeds the NR state for
  preset recall. See `docs/aidocs/DYNAMIC_PARAMS.md`.
- **Behavioral B-sources** (nodal path): arbitrary-expression `V={expr}` / `I={expr}`
  sources over node voltages, `time`, `ddt`, `idt`, and `.param`/`.runtime`
  parameters. Branch-current references and the DK path are not yet supported.
- **Oversampling** 2×/4×: self-contained polyphase half-band IIR anti-aliasing with no
  runtime dependencies (`--oversampling {1|2|4}`). See `docs/aidocs/OVERSAMPLING.md`.
- **Authentic circuit noise** (opt-in, `--noise {off|thermal|shot|full}` +
  `--noise-seed <u64>`; off by default and byte-identical to a noiseless build when
  off). Time-domain stochastic currents injected as Norton sources into the nonlinear
  MNA RHS, so noise is shaped by the circuit and modulated by the operating point.
  Sources: Johnson-Nyquist thermal (fixed and dynamic R), junction shot
  (diode/BJT/FET/triode, triode plate space-charge-smoothed), 1/f flicker on junctions
  (`.model … KF=/AF=`) and resistors (per-element Hooge), pentode partition, and op-amp
  en/in (`.model OA(EN=/IN=)`, white-band v1). Per-stream xoshiro256++ RNG. Runtime
  controls: `set_noise_enabled`, `set_noise_gain`, `set_thermal_gain`/`set_shot_gain`/
  `set_flicker_gain`, `set_temperature_k` (default 290 K), `set_seed`. See
  `docs/aidocs/NOISE.md` and `docs/NOISE_GUIDE.md`.
- **Device self-heating**: quasi-static electrothermal RC model (`RTH`/`CTH`/`XTI`/
  `EG`/`TAMB`) for diodes, BJTs, and triodes. Disabled by default (`RTH=∞` → dead
  code); analytic-validated only (SPICE3f5 silently drops `RTH`, so there is no ngspice
  parity).
- **Unit-variation directives** (`.mismatch` / `.tolerance` / `.seed`): per-device
  `.model` parameter jitter (`.mismatch D|Q P=tol …`) and per-passive value jitter
  (`.tolerance R=/C=/L=`), baked deterministically at codegen time (FNV → SplitMix64).
  Opt-in; byte-identical output when absent; breaks ngspice parity by design. `.mismatch`
  on `J`/`M`/`T` parses but is not yet wired into the IR. See `docs/aidocs/UNIT_VARIATION.md`.
- **Forward-active BJT reduction** via `--bjt-fa {auto|off|force}`. Under `auto`
  (default) only pure Ebers-Moll BJTs biased forward-active (Vbc < −0.5 V) are reduced
  to a 1D NR slot, where the reduction is exact; Gummel-Poon / ISE / self-heating /
  parasitic BJTs stay full-2D. `force` also reduces those (each with a per-device
  accuracy warning — it drops the qb base-charge term and is not accuracy-safe under
  signal); `off` keeps every BJT full-2D. `auto` is byte-identical to prior codegen.
- **Plugin generation** (`--format plugin`): a full nih-plug project targeting CLAP and
  VST3, split into a regenerable `src/circuit.rs` (all DSP) and a user-owned
  `src/lib.rs` (parameters, GUI, presets). Ships a default-on ear-protection soft
  limiter (`--no-ear-protection` to omit), optional Input/Output Level parameters
  (`--no-level-params`), and metadata flags (`--vendor`, `--vendor-url`, `--email`,
  `--vst3-id`, `--clap-id`). All audio-path buffers are pre-allocated — zero heap
  allocation in the callback.
- **CLI**: `compile`, `simulate` (WAV / test-tone through a circuit), `analyze`
  (frequency response with `--pot`/`--switch` overrides), `validate` (ngspice
  comparison), `dc-op`, `nodes`, `import` (KiCad netlist → `.cir`), `sources` (register
  external circuit repositories), `builtins` (deprecated — use `sources`), and `cache`.
  Circuits are referenced as a builtin, a registered `source:circuit`, a URL, or a
  local path.
- **Cross-compilation**: generated plugins build for macOS from Linux via zig 0.13+ /
  cargo-zigbuild (`--target universal2-apple-darwin`) with rcodesign ad-hoc signing.
  The `melange-cli` binary itself does not cross-compile.
- **KiCad integration**: a symbol library (`melange.kicad_sym`) and netlist import path
  covering triodes, pentodes, op-amps, VCAs, pots, wipers, and audio I/O markers.
- **Validation harness**: SPICE validation infrastructure comparing generated code
  against ngspice, plus parser hardening (input-size caps, non-ASCII normalization) and
  a cargo-fuzz target over the parser → MNA → DkKernel → CircuitIR path.
- **Minimum Supported Rust Version 1.85** (2021 edition), declared via `rust-version`
  and enforced in CI.

### Security

- **No `unsafe` code** in the melange library, CLI, or generated solver/DSP code. The
  sole exception: generated *plugin* projects emit one `unsafe` block
  (`std::arch::x86_64::_mm_setcsr`) to enable the CPU's FTZ/DAZ denormal-flush mode for
  real-time performance — the only `unsafe` in any melange output. See
  [SECURITY.md](SECURITY.md).
- **Input validation** — the parser rejects negative, zero, NaN, and infinite component
  values and self-connected components. `safe_exp` clamps arguments to [−40, 40] to
  prevent overflow in device equations.
- **Resource limits** — `MAX_M=24` (NR dimension), `MAX_N=256` nodes,
  `MAX_ELEMENTS=10,000` after expansion, 8-level subcircuit nesting.
- **Bounded iteration** — Newton-Raphson capped at `max_iter` (default 50); DC
  operating point has finite source-stepping and Gmin-stepping fallbacks.
- **Real-time safety** — generated audio callbacks perform no heap allocation, locking,
  or syscalls; all buffers are pre-allocated at construction.
- **Known advisories (waived for 0.1.0).** The KiCad import path (`melange import`, via
  `quick-xml` 0.37) carries two upstream denial-of-service advisories —
  RUSTSEC-2026-0194 (quadratic parse on duplicate attributes) and RUSTSEC-2026-0195
  (unbounded namespace allocation). Reachable only by importing a maliciously-crafted
  KiCad file; no effect on netlist compilation, generated code, or shipped plugins. The
  fix (`quick-xml >= 0.41`) is tracked for 0.1.1.

[Unreleased]: https://github.com/hal0zer0/melange/compare/v0.1.6...HEAD
[0.1.6]: https://github.com/hal0zer0/melange/compare/v0.1.5...v0.1.6
[0.1.5]: https://github.com/hal0zer0/melange/compare/v0.1.4...v0.1.5
[0.1.4]: https://github.com/hal0zer0/melange/compare/v0.1.3...v0.1.4
[0.1.3]: https://github.com/hal0zer0/melange/compare/v0.1.2...v0.1.3
[0.1.2]: https://github.com/hal0zer0/melange/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/hal0zer0/melange/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/hal0zer0/melange/releases/tag/v0.1.0
