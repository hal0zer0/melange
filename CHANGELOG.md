# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
It is `0.x` software: below 1.0.0 there is no stability guarantee — the solver,
codegen output, CLI flags, and netlist semantics may all change.

## [Unreleased]

### Added

- **`melange validate --oversampling {1|2|4}` — validate the code that actually
  ships.** `--oversampling` is a compile-time codegen option: a build at 2x
  upsamples, runs the solver at the internal rate, and decimates through
  polyphase half-band IIR allpass chains. `validate` had no such flag, so there
  was no way to check what you ship — you validated the 1x code and shipped the
  2x code. The flag plumbs through `ValidationOptions.oversampling` into
  `CodegenConfig.oversampling_factor`, with the DK kernel, the routing decision
  and the forward-active / grid-off gates built at `sample_rate * factor`,
  exactly as `compile` and `simulate` do it. Default 1 — every existing run is
  byte-for-byte unchanged (re-measured: `tube_screamer_u` at 48/96/192 kHz
  reproduces the recorded rate sweep to the printed digit).
  - **The filters' response is included in the comparison, not tolerated.**
    ngspice is untouched — it has its own timestep and knows nothing about
    melange's internal rate. Instead the REFERENCE is passed through the same
    half-band round trip the shipped build applies. With the circuit replaced by
    an identity that round trip composes to a cascade of first-order allpasses
    at the HOST rate: magnitude-flat (measured < 0.01 dB, 100 Hz – 18 kHz, 2x
    and 4x), all response in the phase. **No tolerance, preset or pass/fail rule
    moved.** The term it removes is group delay — 2.65 host samples at 1 kHz for
    2x — which uncompensated would cost ~1.5e-2 of correlation against the
    harness's 1 kHz tone, about a thousand times the entire 48 kHz solver
    residual. An uncompensated oversampled run measures the delay and nothing
    else.
  - **What it does not remove stays in the number.** The round trip commutes
    with the circuit only when the circuit is linear. For a nonlinear one the
    interpolator's phase dispersion survives: every harmonic the nonlinearity
    generates inherits the fundamental's time shift, while the compensated
    reference carries each harmonic's own. Measured on `tube_screamer_u`
    (48 kHz, 0.3 V, steady state), melange-vs-reference phase error at the 7th
    harmonic is 0.079° at 1x and 2.53° at 2x, matching the up-filter's measured
    phase-delay dispersion. Harmonic magnitudes move the other way — 2x tracks
    the reference better (7th: −0.030 dB vs −0.135 dB at 1x) — which is the
    finer internal timestep. Both are in the shipped plugin, so both stay.
  - Reported, so it cannot be misread: an oversampled run prints a `Build:` line
    naming the factor, the internal rate and the compensating filter's group
    delay at 1 kHz, above the metrics.
  - `validate` does NOT read a deck's `.oversampling` recommendation, unlike
    `compile`/`simulate`/`analyze`: it reports the build it was asked to
    measure.
  - Twin-drift guard (`crates/melange-validate/tests/oversampling_reference.rs`):
    the emitted `OS_COEFFS`/`OS_COEFFS_OUTER` must equal the `melange-primitives`
    tables bit for bit, and the compensation must reproduce the GENERATED,
    COMPILED oversampled code to < 1e-12 per sample on a pure-gain circuit at
    both 2x and 4x. Compensating with the wrong filter would make every
    oversampled number silently wrong, which is the failure mode this project
    treats as a showstopper.

- **`melange validate` works on op-amp circuits again — the whole class.**
  ngspice has no `U` element and no `OA` model type, so every deck containing an
  op-amp was refused outright. That took the one command that answers "did I
  write this netlist right?" away from the most common hobbyist circuit
  class — a pedal with an op-amp gain stage — and made the README's own
  "Overdrive pedal | op-amp gain + diode clipper" row un-validatable. It was a
  **missing translator**, not a real limitation: melange's op-amp *is* a linear
  Boyle-style VCCS macromodel, so
  `melange-validate/src/opamp_translate.rs` now emits exactly the stamps
  `mna.rs` makes — `GOA_<n> out 0 <in-> <in+> {AOL/ROUT}` (control pair swapped,
  because ngspice's `G` draws current out of `n+` where melange injects it into
  the output node), `ROA_<n> out 0 {ROUT}`, plus `RIN` shunts and `IB` sources
  when the `.model` sets them, each with melange's own grounded-pin guards.
  `GBW` is deliberately not translated: melange computes `iir_c_dom` from it but
  no codegen path consumes it, so its only live effect is defaulting the rails.
  - Native-`U` decks validate **exactly where the hand-expanded ones do**:
    `opamp_inverting_u` returns RMS 1.919045e-7 / peak 4.706210e-7 / correlation
    1.00000000 — digit-for-digit the shipped `opamp_inverting` numbers. The
    overdrive pedal (`tube_screamer_u`: `U1` + `.model OA(AOL=200k ROUT=75)` +
    antiparallel 1N4148 clipping) validates at correlation 0.99999031, RMS
    0.4419%, THD error 0.04 dB, against 0.99999031 / 0.4420% for the
    hand-expanded deck.
  - **The scope limit is enforced, not documented.** The twin is linear: it has
    no VCC/VEE/VSAT rail clamp and no `SR` slew clamp, and melange applies both
    after each NR solve. So each clamped op-amp's output node is added to the
    reference capture, and the run is **refused** if the reference shows the
    clamp would have engaged — naming the op-amp, the rail, the sample time, the
    observed voltage and three ways to proceed. Watching the reference is
    sufficient because the two engines follow one trajectory up to melange's
    first clamp. Measured on the shipped `opamp_railed` deck (gain-of-11 stage on
    ±4.5 V, 0.5 V in): without the guard `validate` would have printed
    **correlation 0.996288** for a melange output clipped at 4.54 V against an
    unclamped 5.53 V reference — comfortably past the 0.99 gate, and a different
    circuit. Rails that are merely *declared* still validate (`opamp_two_stage_rails`,
    three stages on ±4.5 V: correlation 1.00000000).
  - An explicit `.model OA(AOL_TRANSIENT_CAP=…)` is refused before ngspice runs:
    it gives melange's transient G matrix a different Gm than its own DC stamp,
    and one `G` card cannot be both.
  - Op-amps come **off** `deck_guard`'s ngspice-unsupported list. LDR (`O`),
    VCA (`Y`) and glow (`N`) stay on it — each carries device state no ngspice
    primitive reproduces.

- **Netlist topology checks: a mistyped node name is now refused, not
  simulated.** A cold first-user test typo'd `C3 n3 n4 220n` into
  `C3 n33 n4 220n`, which invents a node and floats the tone stage; melange
  printed "Compiled successfully", clean health counters, exit 0, and a 192 KB
  WAV of digital silence. One shared pass
  (`melange_solver::topology`) now runs before anything is built:
  - A node that appears on exactly **one** element terminal in the whole deck is
    **refused** when that element is two-terminal (R, C, L, D, V, I, B, N),
    because such an element carries no current in any circuit. The message names
    the node, the element, the source line and the nearest existing node name by
    edit distance (`n33` → "Did you mean 'n3'?"). A dangling terminal on a
    multi-terminal part (an unused rheostat lug) warns instead.
  - A **cap-only DC island** — a node whose every path out is open at DC — warns,
    with the hedge to confirm it is an intended coupling-cap island.
  - Refusals apply on every build path: `compile` in every output format,
    `simulate`, `analyze` and `validate`. `nodes` and `dc-op` report the findings
    and build anyway: they take no `--output-node`, so they cannot tell an
    orphaned node from an output port, and `nodes` is the command you reach for
    to find the typo.
  - A declared input or output port counts as a connection, and `.tap` /
    `.inject` / a node sensed in a behavioral `B` expression do too.
  - **`.port <node> ...` declares the board's pins**, so a multi-output board is
    not read as a deck full of typos. A filter board with ten numbered pins is
    compiled one output at a time, and every pin that build does not read is a
    node named exactly once; which pin a build reads is a property of the
    *invocation* (`-i`/`-n`), while which pins *exist* is a property of the
    *circuit*, and only the second one belongs in the deck. The directive is
    **direction-neutral** — an undriven INPUT pin needs the same cover as an
    output tap — and repeatable. A declared pin counts as one connection for the
    dangling check **and as nothing else**: it changes no generated code (a
    deck's emitted source is byte-identical with and without its `.port` lines,
    which is what distinguishes it from a `.tap`), it is not a DC path (an
    undriven pin behind a coupling cap is still a floating island and still
    warns), and it selects nothing (`-i`/`-n` stay free to name any node). A
    `.port` naming a node the deck does not have is refused with the same
    nearest-name suggestion, so the declaration cannot become the new place for
    a typo to hide. There is no grandfather clause: a deck with no declaration
    is refused for its dangling nodes exactly as before. The dangling refusal
    now names the directive, so the fix is discoverable at the moment it is
    needed.
  - `melange dc-op` takes its port knowledge from a `.port` declaration when the
    deck has one, instead of guessing from a node literally named `in` — so on
    an annotated deck it refuses the defects every other verb refuses, and keeps
    the old guess (and its warn-only behaviour) on a deck with no declaration.
    `melange nodes` is now report-only by construction (`topology_report`, not
    the gate) rather than by what it happens to know: it is the command you
    reach for to FIND a defect, so no finding at any severity may stop it.

- **`simulate` says so when it renders digital silence.** The peak was already
  computed and printed; it is now reacted to. Warns when the output peak is
  finite and below 1e-6 V (−120 dBFS) — a threshold rather than `== 0.0`,
  because a floated stage can settle on a denormal. NaN/Inf are deliberately not
  silence: a diverged solve is a different failure with its own counters. A
  zero-amplitude drive is not reported. The message uses `max_abs_v_prev` to
  separate "nothing in the circuit moved" from "the circuit is live but the
  output tap is not connected to it".

- **`melange validate` refuses decks the two engines would read differently.**
  ngspice takes a value's mantissa, applies a scale letter only if one
  immediately follows, and silently discards the rest of the token — so it reads
  `4k7` as 4000 against melange's 4700, and `2M2` as 0.002 against 2.2e6. A
  correlation between those two circuits is meaningless, and before this an RC
  lowpass using `4k7` reported `Correlation: 0.99924891 … FAILED` and invited the
  user to go investigate the solver. The guard computes both engines' actual
  readings and flags only a genuine disagreement in value position, so the
  corpus's `2N3904`, `1N4148` and `6K7` device names do not trip it (verified
  across 437 decks). The same check catches a trailing `f`, which melange reads
  as the Farad unit and ngspice as femto.

- **`melange validate` refuses devices ngspice cannot simulate**, instead of
  writing them into the reference deck and relaying ngspice's complaint — which
  pointed at the user's own correct `.model` card. Covers op-amps (`U`/`OA`),
  LDRs (`O`/`LDR`), VCAs (`Y`/`VCA`) and glow tubes (`N`/`NEON`), and explains
  how to hand-expand an op-amp as the VCCS macromodel the shipped validation
  decks already use. Triodes and pentodes are **not** listed: they translate to
  Koren B-source subcircuits and do validate.

- `simulate` gained `--pot` (previously `analyze`-only — for a distortion pedal
  the Drive pot *is* the circuit), `--pcm16` for tools that cannot read float32
  WAVs, and `dc-op` no longer requires a node named `in` to compute a bias point.

- Generated plugin projects now document the `x86-64-v3` CPU baseline (a SIGILL
  on pre-2013 hardware, not a graceful degradation) and where to change `NAME`,
  `VENDOR`, `CLAP_ID` and `VST3_CLASS_ID` before release — two melange plugins
  sharing a class id collide in a DAW. Pot struct fields are named from their
  labels (`pot_tone`, not `pot_0`) while `#[id = "pot_N"]` is unchanged, being
  the persisted automation identity.

- **The `--format code` API is documented** ([`docs/CODE_API.md`](docs/CODE_API.md)).
  `--format code` is the DEFAULT output of `melange compile`, and until now
  `grep 'CircuitState::' docs/*.md README.md` returned nothing: a cold first user
  got through it only by opening his own generated `circuit.rs` and grepping
  `pub fn`, after `CircuitState::new()` failed to exist. The page states the
  shapes that actually surprise people — `CircuitState::default()` is the
  constructor, and `process_sample(input: f64, state: &mut CircuitState) ->
  [f64; NUM_OUTPUTS]` is a FREE FUNCTION, not a method — plus `set_sample_rate`,
  `reset`, the `set_pot_<i>` / `set_switch_<i>` setters (netlist declaration
  order, the order `melange nodes` prints), the constants a caller needs, the
  two `process_sample` signature variants (multi-input, `.inject`/`.tap`), the
  warmup loop, and the real-time rules. Every signature on the page was verified
  by compiling it: the page's own ten-line example builds and runs against a
  generated nodal `passive-eq1a` file (its output tracks `melange simulate` to
  0.03%), and the same calls build against a DK-routed diode-clipper file. The
  README's "zero runtime dependency" claim was checked the same way and is now
  stated with its evidence (empty `[dependencies]`, one package in `Cargo.lock`).
  Linked from the README reading list, the `--format code` paragraph and the CLI
  section, from `PLUGIN_GUIDE.md` and `GETTING_STARTED.md`, and from `compile`'s
  own success output, which now prints the three-line usage sketch where the
  reader actually is.
  - Corrects a README overclaim found while verifying: the soft ear-protection
    limiter is emitted into the plugin wrapper `lib.rs`, NOT into `circuit.rs`.
    On the `--format code` path the output carries a hard +/-10 V clamp and
    nothing else (and with `--no-dc-block`, not even that) — the page says so,
    because the difference is a speaker.

### Changed

- **An exempted THD check now says so instead of printing a bare 40 dB
  "error" under a green PASSED.** `melange validate examples/passive-eq1a.cir`
  printed `THD (SPICE): -113.88 dB / THD (melange): -153.73 dB / THD Error:
  39.85 dB` directly beneath `Status: PASSED`, which reads as either a bug in
  melange or a bug in the report. It is neither: `compare_signals` exempts the
  THD check when melange is at least as clean as the reference AND the reference
  itself sits below the -60 dB no-meaningful-distortion floor, because that
  delta is the gap between two numeric noise floors (ngspice's INTERP floor is
  circuit-dependent; melange's generated code is frequently 40+ dB cleaner).
  The line now reads `39.85 dB (not graded)` followed by an `info (normal):`
  block naming both floors and stating that melange ADDING distortion the
  reference lacks is still graded and still fails. The HTML report's THD row
  shows `not graded` for the same reason, instead of a red mark contradicting
  the run's own verdict. Presentation only: no tolerance, no exemption rule and
  no pass/fail path changed, the number is never suppressed, and a graded THD
  line is byte-for-byte what it was.

- **The floating-island scan in `melange validate` was wrong in both
  directions, and is now a consumer of the shared pass.** It unioned every
  terminal of every non-capacitor element and had no port stamps, so (a) the
  node behind an input coupling cap read as an island — a false positive on
  essentially every guitar pedal (39 decks in the corpus) — and (b) terminals
  that do not conduct at DC held islands together, hiding real defects: an
  op-amp input (`RIN` defaults to `+inf`), a MOSFET gate, a JFET gate and a tube
  grid are all open at DC. A cap-coupled non-inverting op-amp input whose bias
  resistor was typo'd away read as "connected to the output" and passed clean.
  Both are corrections, not tuning. Across the 418-deck circuits corpus the
  island count fell 50 → 11, and two previously silent cases surfaced
  (`basic-bitch`, `sad-bastard`: a resistive band-sum feeding a cathode-follower
  grid with no grid-leak resistor).

Generated DSP is unaffected: the pass reads the netlist and never modifies it.
`examples/passive-eq1a.cir` emits byte-identical code (bar the two provenance
lines) and a byte-identical `simulate` WAV, and 10 corpus decks spot-checked
across both solver routes are byte-identical.

- **`melange validate` compares at nominal values.** `.tolerance` jitters passive
  values in the parser and `.mismatch` jitters device params in codegen, both on
  melange's side only, while the ngspice reference deck kept the values as
  written. Validate was correlating a jittered circuit against a nominal one and
  reporting the number as authoritative — on `examples/passive-eq1a.cir`, 16
  emitted device constants apart, with `MU` off by 7.6%. This was already the
  documented contract; only the automation was missing. Unit variation is now
  disabled on the melange side for the duration of a validate run, and the result
  line says so: `PASSED ✓ (nominal values: .mismatch T disabled for this
  comparison; seed 4142 not exercised)`. Decks with no jitter directives print
  exactly as before. `compile`/`simulate`/`analyze` are unchanged — jitter still
  applies there.

- **Normal-path routing output no longer reads as a fault.** `N=52, M=8, solver:
  multi-transformer circuit (3 groups, DK K matrix unstable)` on the flagship
  example had a first-time user asking whether he had broken the demo. Routing is
  now prefixed `info (normal):` and says why the DK route was not the fit, with
  the maintainer detail kept verbatim. `Skipping BJT internal-node expansion` was
  gated on the K diagonal alone and printed on decks with no BJT at all,
  including that same four-tube example.

- **The three different `N` now say what they count.** The same circuit reported
  41 nodes (`nodes`), `N=40` (`dc-op`) and `N=52` (`analyze`/`compile`) with no
  explanation; `compile`'s summary also contradicted its own output three lines
  above. They are respectively ground plus circuit nodes, circuit nodes excluding
  ground, and those plus one constraint row per voltage source and inductor
  winding.

- `--pot` refuses values outside the pot's declared range instead of accepting
  them silently. `analyze` would previously characterise `LF Boost=1e9` — 100000×
  over the declared `100..10000` — a knob position the generated plugin can never
  reach. Switch positions were already range-checked.

- Unknown `.model` parameters are now checked from one table rather than nine
  inline lists, unreferenced `.model` cards are checked at all, `melange nodes`
  reports unknown keys (it stops before codegen, where the hard error lives), and
  the VCA arm no longer reported the honored `THD` as unrecognized. The tables are
  bound to the resolvers by mutation-tested drift guards.

### Fixed

- **A circuit imported from KiCad had no ground.** `melange import` on the
  bundled `kicad/examples/rc-lowpass` produced `C1 net__c1_pad1 net___pwr01_gnd`
  — the ground symbol became an ordinary floating node, so the MNA system was
  solving against a reference melange picked for itself, a DC operating point
  would report convergence on an ungrounded circuit, and ngspice would reject the
  deck outright. The root cause was in the example, not in KiCad: its embedded
  `power:GND` definition was missing the `(power)` token, so KiCad never made it
  a global net and fell back to the auto-generated name `Net-(#PWR01-GND)`. A
  power symbol drawn from KiCad's stock library emits a net literally named
  `GND`. The schematic is fixed and `rc-lowpass.xml` re-exported from Eeschema
  10.0.6; the example now imports to `C1 net__c1_pad1 0 0.1u` and analyzes to the
  159 Hz corner its reference deck documents. Guarded by
  `shipped_kicad_example_imports_to_the_shipped_reference_circuit`, which imports
  the committed XML (no KiCad needed) and checks it is the same circuit as
  `rc-lowpass-reference.cir` — parts, values, topology and ground on node `0`.
- **A dual supply imported from KiCad had its rails shorted together.**
  `sanitize_node` rewrote every character SPICE cannot carry to `_` and then
  prefixed a leading digit with `n`, so `+15V` and `-15V` both became the single
  node `n15v`. Nothing in the output said so. A leading sign is now carried
  across as the SPICE spelling (`p15v` / `n15v`); interior hyphens are untouched,
  so KiCad's own auto-names (`Net-(C1-Pad1)`) import unchanged. Any residual
  collision — two distinct KiCad nets folding onto one melange node — is now a
  **refusal** naming both nets, because a silent rewire of the circuit is not
  something to warn about and proceed.
- **KiCad's ground spellings map to node `0`.** `GND` (what `power:GND` emits),
  `/GND`, `0` and `ground` all become the reference node. `AGND`, `DGND`,
  `GNDREF` and friends deliberately do **not** — a schematic that draws both
  `GND` and `AGND` has drawn two nets on purpose, and folding them would rewire
  it. Non-ground power symbols (`VCC`, `+15V`) import as ordinary named nodes:
  they are nets the circuit still has to drive, so dropping them or tying them to
  `0` would be wrong. A schematic with no ground symbol still gets no ground —
  melange does not invent one, and the existing no-ground diagnostic is the right
  answer there.
- **A power symbol that KiCad did not treat as one is now named.** `melange
  import` warns when it sees a `Net-(#REF-PIN)` net, explaining that the symbol's
  library definition lacks `(power)` and so created no global net — the exact
  failure melange's own example shipped with, previously visible only as a
  mystery node name.
- **`melange import` no longer claims its own KiCad files are broken.** The
  too-old-`kicad-cli` error still said `melange.kicad_sym` and
  `kicad/examples/rc-lowpass` do not load in KiCad 10.0.6. They were repaired in
  the same batch and both load; the message now says so.
- **`kicad/README.md` states what the KiCad path has and has not been run
  through.** It now leads with the scope: one real end-to-end run, covering stock
  `Simulation_SPICE` R/C parts, wire labels and a `power:GND` symbol. Every
  melange-specific symbol (triode, pentode, op-amp, VCA, pot, wiper, VDC, I/O
  markers) and every `Melange.*` field is untested in a real schematic and said
  to be. It also documents the ground/rail mapping in full, notes that
  `Failed to load schematic file` is KiCad's message for *any* load failure and
  so does not identify whose fault it is, and points at the committed example XML
  as something to run immediately without KiCad.

- **`validate`'s ngspice output parser mis-read tables wider than three
  columns.** ngspice prints at most three data columns per table and emits the
  rest as further column *blocks*, each repeating the whole row range. The
  parser treated every header as more of one table, so `time` came back
  *blocks* × the true length while each voltage stayed correct — making
  `SpiceData::sample_count()` report 3x on a wide capture. Blocks are now
  aligned by row, and a header repeated with the *same* columns is still read as
  ngspice paginating one block. Reachable before only via
  `--additional-nodes`; the op-amp rail probes make wide captures ordinary.

- **Parse errors reported "line 0".** Two defects: 52 error sites never carried a
  line, and the counter counted processed lines, so continuation (`+`) joining
  reported every later error N−1 lines early. Whole-file conditions that have no
  single line now print `Parse error:` rather than claiming line 0.

- **Value errors explain rather than restate.** `1R5`/`10R` remain rejected, and
  the message now says why: ngspice reads `1R5` as 1 Ω, so honouring the BS 1852
  ohms marker would make melange and the ngspice run behind `melange validate`
  simulate different circuits. A test measures every form named in the message
  against the parser so the two cannot drift apart.

- A netlist referencing no ground node warns instead of silently converging
  against a reference melange picked for itself.

- `melange import` printed `kicad-cli: 7.0.11` and then failed with a bare
  "Failed to load schematic file" — so a user concludes their *schematic* is
  broken and redraws it. It now gates on the version it already detected and
  explains that the `.kicad_sch` format melange targets needs KiCad 8+.

- The post-compile hint echoed a `--format code` file path as the plugin project
  *directory*; `--opamp-rail-mode` help and error text omitted `active-set-be`,
  which the parser has always accepted; node lists in error output were
  hash-ordered and differed between runs; and `DIAG:peak` was printed with `{:.6}`,
  so the CLI could not see an output peak below ~5e-7 V.

- **Docs.** The README never linked `NETLIST_GUIDE.md` (zero occurrences) and
  linked `GETTING_STARTED.md` once, behind link text reading "`simulate --help`";
  told you to build debug (1.3 GB vs 493 MB) with no mention of `--release`; and
  buried the hard KiCad 8+ requirement 300 lines down under "Optional:". The
  op-amp parameter table listed 4 of 17 accepted parameters, omitting the
  `VCC`/`VEE` the front page advertises, and did not say that a default op-amp
  cannot clip. Both guides labelled the infix value notation "BS-1852" while
  implementing only the SI-prefix half of it, without saying the `R` marker is
  rejected. `docs/limitations.md` was audited against the source: ~12 stale or
  wrong claims corrected, an unattributed benchmark figure removed, and the open
  conductance-swap transient documented for the first time. The README also
  claimed core saturation via `ISAT=` on transformer windings, which the aidoc
  calls physically wrong, unvalidated and unused.


## [0.1.8] - 2026-09-14

A correctness-and-performance patch. It closes a class of silently-wrong DC
operating points, removes a spurious-noise codegen bug on parasitic-base BJT
circuits, refines when the solver auto-promotes to backward Euler, and adds a
glow-only compile cache. No new shipped feature, no breaking change.

**Generated DSP audio is byte-identical to 0.1.7 for every circuit in the golden
corpus** (168/168 rendered programs across the corpus verified identical). The
DC-op gate is corpus-neutral (0 of 42 golden node vectors change); the
parasitic-BJT and backward-Euler fixes are keyed on conditions no golden deck
reaches; and the glow cache is gated behind `--subsample-fire` (non-glow and
non-ssf codegen is unchanged). MSRV is unchanged (1.85) and no dependency
changed.

### Fixed

- **DC operating points that satisfy the Newton stopping test but violate KCL
  are now rejected.** With two or more parallel junctions biased in the same
  direction, the DC-op solver could report "converged" at a point whose node
  currents do not balance, emitting a silently-wrong bias (and therefore
  silently-wrong audio). A KCL-residual acceptance gate now rejects such points,
  and a joint minimum-norm limiter back-projection corrects the false
  convergence. Corpus-neutral: 0 of 42 golden node vectors flip.
- **Spurious 63× noise on parasitic-base (parasitic-Rb) BJT circuits removed.**
  The trapezoidal history matrix (`A_neg`) was being re-zeroed across a
  parasitic BJT's internal nodes, injecting non-physical noise. Those internal
  nodes are now excluded from the history zeroing. Separately, the
  backward-Euler-latch detector is now mean-removed so it works correctly on
  DC-biased outputs instead of only zero-mean ones.

### Changed

- **`+1` automatic backward-Euler promotion is now gated on whether backward
  Euler actually stabilizes the mode (`rho_be`).** On physical growing poles —
  master oscillators and relaxation oscillators — where backward Euler only
  over-damps rather than stabilizes, the solver now keeps trapezoidal
  integration. This also corrects the misleading warning text emitted when
  backward Euler is forced onto an oscillator.

### Performance

- **Cross-sample LRU cache for the nodal-ssf Schur triple** — six-stage glow
  divider decks (e.g. the Philicorda note boards) reuse the Schur factorization
  across host samples instead of rebuilding it, measured at ~2.49× on a
  six-stage deck. Bit-identical output; gated behind `--subsample-fire`, so
  non-glow and non-ssf circuits emit byte-identical code.

### Security

- **Bumped `rustls` 0.23.43 → 0.23.45** to clear RUSTSEC-2026-0285 (TLS 1.3
  handshake messages incorrectly accepted across encryption-level boundaries,
  medium). `rustls` is a transitive dependency of `ureq`, reachable only from
  the CLI's remote source-fetch path; it is not in the solver, codegen, or any
  generated plugin. Lockfile-only change; MSRV unchanged.

### Docs

- **Analog-EE corrections to the internal device reference docs** (H2 distortion
  is not produced by symmetric hysteresis, output-clamp levels, ageing
  magnitudes, KF/noise-index), from a domain review. Documentation only.

## [0.1.7] - 2026-09-10

A device-and-diagnostics patch. It adds an experimental neon-lamp (glow-discharge)
device and the numerical machinery to run glow relaxation-oscillator/divider
circuits stably, plus build-identity provenance, a deck-declared oversampling
directive, and validate diagnostics. The glow device is functional but
experimental (pre-1.0).

**Generated DSP audio is byte-identical to 0.1.6 for every existing circuit.** The
only change to already-shipping circuits is two clippy-allow lines in the module
header (behavior-neutral) plus inert glow state fields; on the golden corpus all
168 rendered programs across 38 circuits are identical. MSRV is unchanged (1.85)
and no dependency changed. The glow strike/extinction numerics are gated on the
presence of a latched glow device, so no non-glow circuit's audio moves.

### Added

- **Glow-discharge / neon-lamp device** (`.model NEON(VO VM IK RS IHOLD ROFF)`) —
  a latched relaxation device for neon relaxation oscillators and frequency
  dividers. Experimental. The lit branch is the maintaining line `i = (v − V0)/RS`
  with a derived intercept `V0 = VM − RS·IK`.
- **`melange compile --subsample-fire {auto|on|off}`** — variable-dt breakpoint
  re-solve that resolves each glow strike/extinction at its true sub-sample
  crossing instead of at a grid point, curing rate-quantized divider
  injection-lock (a divider that dropped pitch classes at the base grid). `auto`
  enables it for a latched glow device on the nodal-Schur route; the DK and nodal
  full-LU routes are inert and record why in the provenance manifest. Byte-neutral
  for non-glow and for `off`.
- **`melange compile --subsample-lit-factor <x>`** — diagnostic bisection knob for
  the lit sub-step size (not a per-deck tuning control).
- **`.oversampling N` netlist directive** (N ∈ {1,2,4}) — a deck can declare its
  recommended oversampling minimum. CLI `--oversampling` still wins when set
  (even lower, with a warning); honored on compile/simulate/analyze; `validate`
  ignores it.
- **Build identity in `melange --version` and generated provenance** — an
  FNV-1a-64 hash of the running executable (`exe fnv1a64:<hash>`, JSON key
  `exe_fnv1a64`), plus `"solver":"dk"|"nodal"` in the provenance JSON, so two
  builds at the same commit (or a dirty tree) are distinguishable.
- **`melange validate --backward-euler` / `--force-trap`** — integrator
  diagnostics mirroring `compile`; diagnostic only, never the gate.
- **`melange validate --bjt-fa` / `--tube-grid-fa`** — the existing mechanism
  flags plumbed through the validator.
- **Compile-time washout diagnostic for voltage-mode VCAs** driven through a high
  series resistance (`R_drive·G0 ≥ 10`), where the control voltage silently washes
  out; warns to use current-drive mode. Warning only; byte-neutral.

### Changed

- **Generated code emits `#![allow(unused_mut)]` and `#![allow(clippy::manual_memcpy)]`
  in the module header** so a downstream `clippy -D warnings` build stays clean
  across regeneration. This is a source-level diff on every generated file;
  rendered audio is unchanged.
- **`melange validate` derives its ngspice directive-strip set from the parser**
  (`MELANGE_ONLY_DIRECTIVES`) instead of a hand-maintained list, with a two-way
  drift-guard test.

### Fixed

- **Nodal-Schur Newton divergence on multi-stage glow-divider chains** — the
  first-order NR warm-start predictor extrapolated the stiff lit-discharge current
  into diode breakdown and the voltage-step convergence test accepted it, letting
  the chain run away (~1e6 V) while a global magnitude-reset cadence disguised it
  as a collapsed divider. A zero-order warm start for latched-device circuits fixes
  it. Compile-time gated on latched-device presence — non-glow circuits are
  byte-identical.
- **Glow nodal divergence at plugin sample rates** — trapezoidal ringing on the
  ~1e5 glow conductance step rang the stiff mode over the breakdown threshold at
  larger timesteps. Lit-gated Backward Euler (L-stable) plus omitting a
  double-counted trap-midpoint stamp in the BE fallback cure it. Glow-gated;
  non-glow byte-identical.
- **`melange validate` failed on decks carrying `.integrator` or `.delay_feedback`**
  (an ngspice "unimplemented dot command") because those directives were missing
  from the hand-maintained strip list; the parser-derived set fixes it. `.inject`
  is translated to a resistor for ngspice rather than stripped.

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

[Unreleased]: https://github.com/hal0zer0/melange/compare/v0.1.8...HEAD
[0.1.8]: https://github.com/hal0zer0/melange/compare/v0.1.7...v0.1.8
[0.1.7]: https://github.com/hal0zer0/melange/compare/v0.1.6...v0.1.7
[0.1.6]: https://github.com/hal0zer0/melange/compare/v0.1.5...v0.1.6
[0.1.5]: https://github.com/hal0zer0/melange/compare/v0.1.4...v0.1.5
[0.1.4]: https://github.com/hal0zer0/melange/compare/v0.1.3...v0.1.4
[0.1.3]: https://github.com/hal0zer0/melange/compare/v0.1.2...v0.1.3
[0.1.2]: https://github.com/hal0zer0/melange/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/hal0zer0/melange/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/hal0zer0/melange/releases/tag/v0.1.0
