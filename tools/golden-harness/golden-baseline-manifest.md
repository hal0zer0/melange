# Golden Baseline Manifest — melange-generated circuits shipped in oomox

Generated 2026-07-21. Refreshed 2026-08-30 against melange **e53573c** (correctness-sweep HEAD, melange 0.1.3). Superseded interim baseline **83bf48d** (carried the tungsten-thunder-horse line-search regression, fixed by e53573c). Prior baseline: **8945b67**.

**Re-baselined 2026-09-04 against melange `cd133c6`** (pentode grid-off reduction defaults to full 3D). `golden-baselines/cd133c6` supersedes `91aaa48`. Movement vs 91aaa48: **noyce-ef86** audio changed — its frozen-`Vg2k` reduction is now off by default → full 3D, removing the +2.18% small-signal gain error (peak 0.673→0.659, toward ngspice); **five-watt-freddie** (`champ-5f1`) and **noyce-amp-at-idle** are INPUT-CHANGED (committed netlist drift since 91aaa48, not the compiler); `diag_region_exit_count` was added to all generated code, so generated source differs on all 38 circuits and solver diagnostics on 21 renders (e.g. steve-1073-preamp renders byte-identical but now reports region-exits). Circuits repo now clean at `5270e67` (branch `publish-prep`) — all shipped netlists pinnable to a commit. **Downstream:** three *corpus* oomox plugins (five-watt-freddie, noyce-amp-at-idle, noyce-ef86) render from decks that were reducing; their checked-in `circuit.rs` is now stale vs the full-3D default and should be regenerated (corpus, not openwurli/shipped). The per-circuit DRIFT-status table below predates this re-baseline and its own 2026-08-30 refresh; it warrants a separate re-audit and is not reconciled here.

**Re-baselined 2026-09-28 against melange `576a492`** (op-amp rail handling). `golden-baselines/576a492` is the reference. Against v0.1.11: 155 identical, 12 negligible, **1 changed — `vurli/silence`**, −3.1 dB on a 3 µV-peak render (max Δ 1.1 µV). Attribution: **DK→nodal for active-set rail handling** — vurli-leveler's op-amp resolves to active-set, which only nodal implements, so it no longer runs DK's post-solve clamp. The 12 negligible renders are the three in-manifest decks that changed route (vurli-leveler, gold-press-mastering, noyce-4558); the golden programs never rail their op-amps, which is where the routes differ. The removed ActiveSetBe 2× sub-step is audio-neutral on its own (168/168 identical). Generated source differs on all 38 (two new rail-mode constants; unused sub-step matrices removed from nodal state). No render held, none not-converged.

**Added 2026-09-28: three saturation coverage decks** (`sat-knee-rl`, `sat-core-loaded`, `sat-core-open`), in-repo under `tools/golden-harness/decks/`, at a 5 V manifest level. No golden program had taken any inductor past i/Isat = 0.37, so the saturation knee was never exercised; at 5 V the 20–60 Hz part of the sweep reaches it. They are change detectors only; correctness is gated in `crates/melange-solver/tests/saturation_knee_regression_tests.rs` against independent references. Their first capture is a new baseline, not a restored one.

**Added 2026-09-28: two op-amp rail-pin coverage decks** (`opamp-pin-audio`, `opamp-pin-control`), in-repo under `tools/golden-harness/decks/`, at a 0.5 V manifest level. No golden program had pinned an op-amp under any active-set mode, so a change to rail handling was attributed to nothing. Both are a single-supply overdrive (gain ~107, rails 0/9 V) into a diode clipper behind the output cap, and both pin every half cycle. `opamp-pin-audio` resolves to `active-set-be`; `opamp-pin-control` adds an envelope-detector sidechain off the op-amp output, resolves to `active-set` on trapezoidal, and runs a transition-BE sample at each pin and release. They are change detectors only; correctness is gated in `transition_be_tests.rs` and `opamp_railing_regression_tests.rs`. Their first capture is a new baseline, not a restored one.

**Re-baselined 2026-09-28 (saturating-inductor flux tolerance).** Attribution: **flux-row residual tolerance 1e-3 → 1e-5 of the per-sample increment** (Newton's one-signed remainder was integrated under DC bias). Against the previous capture: 173 identical, 7 negligible (mosfet-choke-load/sweep; step and sweep of the three saturation coverage decks; within 1e-5 dB, corr 1.0000000), 0 changed. Generated source differs on the five saturating decks only; funkyinduct renders identically. Newton iterations up 0.02–2.3 % on the negligible renders; 0 sub-steps on every saturating render, before and after. No render held, none not-converged.

**Checked 2026-09-28 (saturating inductor inside the op-amp rail pin).** Against the previous capture: 180/180 identical. Generated source differs on 9 decks: four full-LU decks that can pin an op-amp (gravity, moonladder, sad-bastard, sus-bus) gain the unsolved-pin counter, 0 on every render; the five saturating decks carry a restructured flux-row residual with the same arithmetic. No golden deck combines a pinned op-amp with a saturating inductor; that combination is gated in `opamp_railing_regression_tests.rs`.

**Checked 2026-09-28 (flux checks in the recovery loops).** 180/180 identical; generated source differs on the five saturating decks only. No golden render reaches the sub-step or backward-Euler loop on a saturating deck; the checks are gated by `c1_jacobian_deletion_is_caught_at_every_newton_site`.

**Re-baselined 2026-09-28 (MOSFET body effect).** Attribution: **MOSFET body effect: node resolution (`89a45ab`) + live-iterate Vt with gmb (DK and full-LU)**. Against `0ddf38f`: 172 identical, 8 changed, all on the two MOSFET decks. mosfet-choke-load's silence render loses its startup swing from the wrong bias (−151 dB; peak 286.7 → 9e-7); mosfet-source-follower was 8 % hot on H1 with a ninth of ngspice's H2 and now matches ngspice to 3e-6 on H1 (sine1k −0.78 dB, sweep −0.72 dB, silence −95 dB). Generated source differs on those two decks only. No render held, none not-converged.

**Checked 2026-09-28 (BE-latch on saturating circuits).** 179 identical, 1 negligible, 0 changed. The negligible one is `sat-core-open/step`: the latch fires once, on a genuine trapezoidal Nyquist ring (6 mV sample-to-sample alternation around −0.236 V after the 5 V step saturates the open transformer), and removes it (corr 0.9999994). It is the only latch fire on the golden set. Generated source differs on the four saturating decks that now emit the latch; funkyinduct builds on backward Euler and emits none.

**Re-baselined 2026-09-28 (op-amp DC operating point).** Attribution: **DC OP consistent with the transient model (op-amp stage)** — the DC solve's AOL = 1000 cap was returned as the answer; it is now finished at the full AOL. Against `614d169`: 155 identical, 18 negligible, 7 changed. The changed renders are moonladder and pipe-shouter losing a decaying startup transient from the capped bias (moonladder 21 mV at sample 0; its sine1k/sweep level figures are that transient leaving the RMS). Latch sweep: 116 renders emit the latch, one fires before and after (sat-core-open/step), so no golden deck was latching on a bad start.

Machine-readable twin: `tools/golden-harness/golden-baseline-manifest.json`.

## Compile recipe

All commands run from the **oomox repo root**; netlists live in `../melange-circuits`. Base recipe:

```
melange compile ../melange-circuits/<cir> --format code [flags] -o <circuit_rs>
```

Defaults everywhere: sample rate 48000, input node `in`, output node `out`, `--solver auto`,
no `--output-scale`, no `--backward-euler`/`--force-trap` (BE promotion is auto-detected),
`--noise-seed 0`. The only per-circuit flags ever used are `--oversampling`, `--noise <mode>`,
and `--emit-dc-op-recompute`. `MAX_ITER` differences between files come from the CLI's
auto-tuner, not from `--max-iter`.

## Verification protocol

Each compile_cmd was executed into scratch space and diffed byte-for-byte against the checked-in oomox file. EXACT = byte-identical. DRIFT-EXPLAINED = every diff hunk attributable to known post-regen melange commits (146d51b clippy-allow header; a472807 two-draw shot noise; 49ecaa4 coupled-inductor augmented-row ordering; **correctness-sweep 0c70d44 + 83bf48d + e53573c nodal-NR globalization**) AND anchors (N, M, OVERSAMPLING_FACTOR, INPUT_NODE, OUTPUT_NODES, OUTPUT_SCALES, INPUT_RESISTANCE, SAMPLE_RATE, full setter list, noise fns) verified identical. No hand-edited generated files were found.

Status legend:
- **EXACT** — byte-identical regen at melange HEAD.
- **DRIFT (header)** — differs only by the 4 clippy-allow header lines added in melange `146d51b` (post-regen).
- **DRIFT (hdr+shot)** — header plus the `a472807` two-draw shot-noise change (`noise_shot_w_prev` state + draw-loop restructure).
- **DRIFT (hdr+shot+rows)** — additionally the `49ecaa4` deterministic coupled-inductor augmented-row ordering (row permutation + FP last-digit wiggle).
- **DRIFT (+nr-global)** — additionally the correctness-sweep NR globalization (`0c70d44` + `83bf48d` + `e53573c`): new `kcl_residual`/`kcl_residual_inl` + Armijo line search, old adaptive sub-step fallback removed, `MAX_ITER`→100, and (`e53573c`) line-search failure falls through instead of bailing. Emitted on every deck carrying an inner nodal/behavioral NR loop; see the Correctness sweep section for the three sub-forms and audio result.
- **UNRESOLVED** — cannot be reproduced from any available netlist (see Gaps).

## Manifest

| Plugin | Generated file (oomox) | Netlist (melange-circuits) | Extra flags | Status | Solver | N | M | Pots | Sw | RtR | Noise |
|---|---|---|---|---|---|---|---|---|---|---|---|
| basic-bitch | `plugins/basic-bitch/src/circuit.rs` | `unstable/pedals/basic-bitch.cir` | `--noise thermal` | DRIFT (header) | nodal | 33 | 8 | 8 | 0 | 2 | thermal |
| five-watt-freddie | `plugins/five-watt-freddie/src/circuit.rs` | `unstable/amp/champ-5f1.cir` | `--noise thermal` | DRIFT (header) | nodal | 24 | 6 | 2 | 0 | 0 | thermal |
| funkyinduct | `plugins/funkyinduct/src/circuit.rs` | `unstable/filters/funkyinduct.cir` | `--noise shot` | DRIFT (hdr+shot) | nodal | 73 | 2 | 32 | 0 | 0 | shot |
| gold-press | `plugins/gold-press/src/cab.rs` | `unstable/filters/gold-press-cab.cir` | `--oversampling 4 --noise full --emit-dc-op-recompute` | DRIFT (header) | dk | 4 | 0 | 0 | 1 | 0 | full |
| gold-press | `plugins/gold-press/src/cartridge.rs` | `unstable/filters/gold-press-cartridge.cir` | `--oversampling 4 --noise full --emit-dc-op-recompute` | DRIFT (header) | dk | 5 | 0 | 0 | 1 | 0 | full |
| gold-press | `plugins/gold-press/src/mastering.rs` | `unstable/filters/gold-press-mastering.cir` | `--oversampling 4 --noise full --emit-dc-op-recompute` | DRIFT (hdr+shot) | dk | 11 | 2 | 0 | 0 | 0 | full |
| gold-press | `plugins/gold-press/src/overdrive.rs` | `unstable/filters/gold-press-overdrive.cir` | `--oversampling 4 --noise full --emit-dc-op-recompute` | DRIFT (hdr+shot) | dk | 7 | 2 | 1 | 0 | 0 | full |
| gold-press | `plugins/gold-press/src/riaa.rs` | `unstable/preamp/gold-press-riaa.cir` | `--oversampling 4 --noise full --emit-dc-op-recompute` | DRIFT (hdr+shot) | dk | 16 | 6 | 1 | 0 | 0 | full |
| moonladder | `plugins/moonladder/src/circuit.rs` | `unstable/filters/moonladder.cir` | `--noise shot` | DRIFT (hdr+shot) | nodal | 25 | 16 | 2 | 0 | 0 | shot |
| noyce | `plugins/noyce/src/sources/amp_at_idle/circuit.rs` | `unstable/gimmicks/noyce-amp-at-idle.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 18 | 6 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/boiler_room/circuit.rs` | `unstable/gimmicks/noyce-boiler-room.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 8 | 0 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/carbon_comp_bank/circuit.rs` | `unstable/gimmicks/noyce-carbon-comp-bank.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 11 | 0 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/clean_rc/circuit.rs` | `unstable/gimmicks/noyce-clean-rc.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 2 | 0 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/ef86/circuit.rs` | `unstable/gimmicks/noyce-ef86.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 8 | 2 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/germanium_cluster/circuit.rs` | `unstable/gimmicks/noyce-germanium-cluster.cir` | `--noise full` | DRIFT (+nr-global) | nodal | 10 | 6 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/jrc4558/circuit.rs` | `unstable/gimmicks/noyce-4558.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 5 | 0 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/smps_ripple/circuit.rs` | `unstable/gimmicks/noyce-smps-ripple.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 10 | 2 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/tape_head/circuit.rs` | `unstable/gimmicks/noyce-tape-head.cir` | `--noise full` | DRIFT (max-iter) | nodal | 11 | 2 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/transformer_triode/circuit.rs` | `unstable/gimmicks/noyce-transformer-triode.cir` | `--noise full` | DRIFT (hdr+shot+rows) | nodal | 18 | 2 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/triode_12ax7/circuit.rs` | `unstable/gimmicks/noyce-triode-12ax7.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 7 | 2 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/zener_junction/circuit.rs` | `unstable/gimmicks/noyce-zener-junction.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 5 | 1 | 0 | 0 | 0 | full |
| periodic-pedal | `plugins/periodic-pedal/src/circuit.rs` | `unstable/pedals/periodic-pedal.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot) | nodal | 31 | 14 | 5 | 7 | 0 | shot |
| pipe-shouter | `plugins/pipe-shouter/src/circuit.rs` | `unstable/pedals/pipe-shouter.cir` | `--noise shot` | DRIFT (hdr+shot) | nodal | 26 | 6 | 5 | 0 | 0 | shot |
| pretty-baby | `plugins/pretty-baby/src/circuit.rs` | `unstable/pedals/pretty-baby.cir` | `--noise shot` | DRIFT (hdr+shot) | nodal | 41 | 8 | 10 | 0 | 0 | shot |
| qapla-1a | `plugins/qapla-1a/src/circuit.rs` | `unstable/filters/passive-eq1a.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot+rows) | nodal | 46 | 8 | 7 | 3 | 0 | shot |
| sad-bastard | `plugins/sad-bastard/src/circuit.rs` | `unstable/pedals/sad-bastard.cir` | `--noise thermal` | DRIFT (header) | nodal | 45 | 14 | 8 | 0 | 5 | thermal |
| series-of-tubes | `plugins/series-of-tubes/src/circuit.rs` | `unstable/dynamics/series-of-tubes-stage.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot) | dk | 13 | 2 | 3 | 0 | 0 | shot |
| series-of-tubes | `plugins/series-of-tubes/src/warmth.rs` | `unstable/dynamics/series-of-tubes-warmth.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot) | dk | 8 | 2 | 0 | 0 | 0 | shot |
| subspace | `plugins/subspace/src/circuits/radio_am.rs` | `unstable/gimmicks/radio-am.cir` | `--noise full` | DRIFT (header) | nodal | 17 | 0 | 0 | 0 | 2 | full |
| subspace | `plugins/subspace/src/circuits/radio_fm.rs` | `unstable/gimmicks/radio-fm.cir` | `--noise full` | UNRESOLVED | dk | 16 | 0 | 2 | 0 | 1 | full |
| sus-bus | `plugins/sus-bus/src/circuit.rs` | `testing/dynamics/4kbuscomp-audiopath.cir` | `--noise shot` | DRIFT (header) | nodal | 25 | 2 | 0 | 0 | 0 | shot |
| tungsten-glow | `plugins/tungsten-glow/src/circuit.rs` | `unstable/dynamics/tungsten-glow.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot) | nodal | 26 | 6 | 7 | 0 | 0 | shot |
| tungsten-thunder-horse | `plugins/tungsten-thunder-horse/src/cascade.rs` | `unstable/pedals/tungsten-thunder-horse-cascade.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot) | dk | 25 | 8 | 4 | 0 | 0 | shot |
| tungsten-thunder-horse | `plugins/tungsten-thunder-horse/src/circuit.rs` | `unstable/pedals/tungsten-thunder-horse.cir` | `--noise shot` | DRIFT (hdr+shot) | nodal | 37 | 14 | 11 | 0 | 0 | shot |
| tungsten-thunder-horse | `plugins/tungsten-thunder-horse/src/edge.rs` | `unstable/pedals/tungsten-thunder-horse-edge.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot) | dk | 11 | 4 | 2 | 0 | 0 | shot |
| uniquorn | `plugins/uniquorn/src/circuit.rs` | `unstable/pedals/uniquorn.cir` | `--noise shot` | DRIFT (hdr+shot) | nodal | 46 | 12 | 11 | 0 | 0 | shot |
| uniquorn | `plugins/uniquorn/src/power.rs` | `unstable/pedals/uniquorn-power.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot) | nodal | 23 | 6 | 3 | 0 | 0 | shot |
| vcr-audio | `plugins/vcr-audio/src/circuit.rs` | `unstable/dynamics/vcr-audio-alc.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot) | dk | 18 | 3 | 1 | 2 | 0 | shot |
| velvet-elvis | `plugins/velvet-elvis/src/circuit.rs` | `unstable/dynamics/velvet-elvis.cir` | `--noise shot --emit-dc-op-recompute` | DRIFT (hdr+shot) | dk | 13 | 4 | 1 | 0 | 0 | shot |
| vurli | `plugins/vurli/src/comp/circuit.rs` | `unstable/dynamics/vurli-leveler.cir` | `--emit-dc-op-recompute` | DRIFT (header) | dk | 10 | 2 | 1 | 0 | 0 | — |
| warpony | `plugins/warpony/src/circuit.rs` | `unstable/pedals/warpony.cir` | `--noise shot` | DRIFT (hdr+shot) | nodal | 30 | 14 | 6 | 0 | 0 | shot |

Counts: 42 generated files across 22 plugin trees — 9 EXACT, 32 DRIFT-EXPLAINED, 1 UNRESOLVED (at e53573c; germanium-cluster and tape-head flipped EXACT→DRIFT under the NR globalization). The table omits the-kicker (one DRIFT row): that circuit never worked, its netlist was pruned 2026-09-02, and nothing it rendered is evidence.

## Correctness sweep (2026-08-30) — refresh main (ee8841e) → correctness-sweep HEAD (e53573c)

Full golden-harness audio capture+compare (level-0.1 V programs) plus per-circuit
generated-code diff. **Every generated-output change across the corpus is attributable
to the nodal-NR globalization (`0c70d44` + `83bf48d` + `e53573c`); zero unattributable
changes.**

> **Regression + fix note.** The interim `83bf48d` baseline carried a
> tungsten-thunder-horse line-search regression (idle output collapsed to 0.0). `e53573c`
> (review-endorsed) makes a line-search **failure** fall through instead of bailing to the
> removed fallback, and tungsten re-converges to main. The `83bf48d`→`e53573c` compare is
> **surgical**: only tungsten changed (181 identical, 0 other circuits affected), inverting
> the regression almost exactly (step −10.465 dB, silence −5.851 dB).

**Audio compare (main vs fixed HEAD e53573c):** 171 identical, 11 negligible, 4 CHANGED,
1 missing (qapla-1a, unresolvable — see Gaps). **No DK deck changed audibly** — the fix
is nodal-only for audio, as designed.

- **tungsten-thunder-horse** (`unstable/pedals/tungsten-thunder-horse.cir`, nodal N=37 M=14) —
  **RESOLVED / re-converged to main.** vs main: silence −0.012 dB (corr 0.9978), step
  −0.004 dB (corr 0.9992), sweep +0.017 dB (corr 0.9926); sine1k + potsweep negligible
  (corr 1.0). Idle noise floor and levels restored. Not bit-identical, so silence/step/sweep
  still trip the strict CHANGED classifier at tiny magnitude; the residual is **benign** —
  signal-dependent idle shot-noise realization (idle output is pure amplified shot noise, so
  an FP-level operating-point difference reshuffles the noise samples) + HF-clip phase wiggle
  on the near-Nyquist sweep tail. (dr-debuggenshmirtz measured step Δ≈6e-4 vs main on the
  **deterministic** no-noise signal; the harness runs `--noise shot`, hence the larger captured
  idle residual.) Codegen still carries the NR globalization, so the deck stays DRIFT-EXPLAINED.
- **uniquorn** (`unstable/pedals/uniquorn.cir`, nodal N=46 M=12) — **TRIVIAL**: potsweep only,
  +0.002 dB, corr 0.999748, sub-mV tail drift on the harder-converging pot positions
  (unchanged from the 83bf48d compare). Benign.

**Codegen-drift classes** (from per-circuit `circuit.rs` diff at e53573c; membership identical at the interim 83bf48d):

- **nr_globalization_full** (residual+Armijo fns added, MAX_ITER→100): funkyinduct, moonladder,
  noyce-germanium-cluster, noyce-transformer-triode, periodic-pedal, pretty-baby, sad-bastard,
  sus-bus, the-kicker, tungsten-thunder-horse, uniquorn, uniquorn-power (12).
- **nr_globalization_substep_removed** (old adaptive sub-step fallback deleted, MAX_ITER→100,
  no residual fn): subspace/radio-am, subspace/radio-fm (2).
- **max_iter_const_only** (only the MAX_ITER 50|70→100 const + its provenance echo):
  basic-bitch, five-watt-freddie, noyce-tape-head, pipe-shouter, tungsten-glow, velvet-elvis,
  warpony (7).
- **identical** (no correctness-sweep drift): gold-press ×5, noyce ×9 (4558, amp-at-idle,
  boiler-room, carbon-comp-bank, clean-rc, ef86, smps-ripple, triode-12ax7, zener-junction),
  series-of-tubes ×2, tungsten-thunder-horse cascade/edge (orphan modules), vcr-audio, vurli (20).

**Other sweep commits produced NO drift here** (verified by content diff at e53573c):
`63cee6e` (DC-OP candidate retention) changed zero DC_OP/DC_NL_I constants — no corpus circuit
hit the leaky-gate degenerate case; `be7be88` (independent current-source sign) is inert — no
corpus deck has an explicit `I` element.

**Caveat:** MAX_ITER→100 is not strictly nodal-only at the codegen level — it also reached
velvet-elvis and radio-fm (manifest-labelled dk). Both are audio-identical (raising an
already-satisfied iteration cap changes no converged output), so the audio-level nodal-only
claim holds.

## Regen provenance

- **oomox `ab21b67`** (2026-07-18): 31 of these files regenerated against melange `8fb9741` (pre-`146d51b`), 'same per-file recipe as shipped, verified by anchor diff'. This manifest is the recovered recipe.
- **oomox `a8c47f3`** (2026-07-19): the 11 noyce sources (all but `transformer_triode`) regenerated against melange `a472807` — these are the EXACT rows. `transformer_triode` was explicitly deferred pending the coupled-L determinism fix (`49ecaa4`), so it still carries row-order drift.
- Historical recipe evidence: oomox `583c9e9` (workspace `--noise shot` rollout), `574af75` (Gold Press `--oversampling 4 --noise full`), in-source regen comments in `plugins/basic-bitch/src/lib.rs` and `plugins/five-watt-freddie/src/lib.rs` (`--noise thermal`), per-plugin `Cargo.toml` noise-feature comments, `local-docs/noise-regen-spec.md` mapping table.

## Gaps (UNRESOLVED)

### subspace — `plugins/subspace/src/circuits/radio_fm.rs`

STALE BY DESIGN: checked-in file was generated from a pre-rework radio-fm.cir (title 'Radio — FM mode (pre-detection-noise interim…', N=16); the current netlist is the reworked behavioral FM receiver (N=23) and was deliberately held back in oomox ab21b67 ('topology upgrade, plugin not wired for it'). The old netlist is unrecoverable: radio-fm.cir has never been committed to melange-circuits (untracked). Compile cmd shown reproduces the NEW topology, not the checked-in file. Flag guesses (--noise full) mirror radio_am. WARNING: netlist is UNTRACKED in melange-circuits — exists only as a working-tree file, no git history at all.

Missing evidence: the pre-rework `radio-fm.cir` itself. It was never committed (melange-circuits has the file untracked) and oomox does not vendor netlists, so no copy survives. To close the gap either (a) wire the subspace plugin to the new N=23 topology and regen, or (b) accept the checked-in file as an unreproducible pinned baseline for the harness.

## Repo-state warnings (fix before trusting the golden harness)

melange-circuits HEAD is `b655adf (2026-06-12) — STALE relative to shipped netlists`. The melange-circuits working tree, not its git HEAD, is the authoritative source for several shipped circuits. The golden harness cannot pin these netlists to a commit until they are committed.

Netlists with **uncommitted modifications** (working tree matches shipped code; HEAD does not):

- `testing/dynamics/4kbuscomp-audiopath.cir`
- `unstable/dynamics/series-of-tubes-stage.cir`
- `unstable/dynamics/velvet-elvis.cir`
- `unstable/gimmicks/noyce-amp-at-idle.cir`
- `unstable/gimmicks/noyce-carbon-comp-bank.cir`
- `unstable/pedals/periodic-pedal.cir`

Netlists that are **entirely untracked** (no git history at all):

- `unstable/dynamics/vurli-leveler.cir`
- `unstable/gimmicks/noyce-boiler-room.cir`
- `unstable/gimmicks/radio-am.cir`
- `unstable/gimmicks/radio-fm.cir`

## Other findings

- **No hand-edited generated files.** Every diff hunk in all 31 non-EXACT files is fully attributable to the three known post-regen melange commits; a filtered-residue sweep left nothing unexplained. Regen will not clobber any manual fixes.
- `tungsten-thunder-horse/src/{cascade,edge}.rs` are compiled (`pub mod`) but unused by the audio path — the monolithic `circuit.rs` is live. Keep regenerating them anyway or drop the mods.
- `vcr-audio/src/circuit.rs` is an orphan (no crate references it) but is still refreshed at regens.
- `vurli/src/comp/circuit.rs` is the only shipped circuit compiled without `--noise`.
- `--emit-dc-op-recompute` usage is inconsistent across the fleet (all DK files plus exactly four nodal files: qapla-1a, periodic-pedal, tungsten-glow, uniquorn/power). The manifest records what ships; consider unifying at the next full regen.
- gold-press `cab`/`cartridge` use `--noise full` but have M=0 (no junctions), so their emitted noise is thermal/flicker only — that is why they show header-only drift despite the noise flag.

