# Golden Baseline Manifest — melange-generated circuits shipped in oomox

Generated 2026-07-21. Verified against melange **8945b67** (installed `melange` binary built from HEAD).

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

Each compile_cmd was executed into scratch space and diffed byte-for-byte against the checked-in oomox file. EXACT = byte-identical. DRIFT-EXPLAINED = every diff hunk attributable to known post-regen melange commits (146d51b emitted clippy-allow header; a472807 two-draw shot noise; 49ecaa4 deterministic coupled-inductor augmented-row ordering) AND anchors (N, M, OVERSAMPLING_FACTOR, INPUT_NODE, OUTPUT_NODES, OUTPUT_SCALES, INPUT_RESISTANCE, SAMPLE_RATE, full setter list, noise fns) verified identical. No hand-edited generated files were found.

Status legend:
- **EXACT** — byte-identical regen at melange HEAD.
- **DRIFT (header)** — differs only by the 4 clippy-allow header lines added in melange `146d51b` (post-regen).
- **DRIFT (hdr+shot)** — header plus the `a472807` two-draw shot-noise change (`noise_shot_w_prev` state + draw-loop restructure).
- **DRIFT (hdr+shot+rows)** — additionally the `49ecaa4` deterministic coupled-inductor augmented-row ordering (row permutation + FP last-digit wiggle).
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
| noyce | `plugins/noyce/src/sources/germanium_cluster/circuit.rs` | `unstable/gimmicks/noyce-germanium-cluster.cir` | `--noise full` | EXACT | nodal | 10 | 6 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/jrc4558/circuit.rs` | `unstable/gimmicks/noyce-4558.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 5 | 0 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/smps_ripple/circuit.rs` | `unstable/gimmicks/noyce-smps-ripple.cir` | `--noise full --emit-dc-op-recompute` | EXACT | dk | 10 | 2 | 0 | 0 | 0 | full |
| noyce | `plugins/noyce/src/sources/tape_head/circuit.rs` | `unstable/gimmicks/noyce-tape-head.cir` | `--noise full` | EXACT | nodal | 11 | 2 | 0 | 0 | 0 | full |
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
| the-kicker | `plugins/the-kicker/src/circuit.rs` | `unstable/drums/the-kicker.cir` | `--noise shot` | DRIFT (hdr+shot) | nodal | 35 | 6 | 9 | 0 | 0 | shot |
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

Counts: 42 generated files across 22 plugin trees — 11 EXACT, 30 DRIFT-EXPLAINED, 1 UNRESOLVED.

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

