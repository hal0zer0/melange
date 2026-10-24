# Sherman-Morrison Rank-1 Updates (Dynamic Potentiometers)

A potentiometer or `.runtime R` change is a rank-1 change to the MNA
conductance matrix, so the inverse `S = A⁻¹` can be updated exactly in O(N²)
instead of re-inverted in O(N³). The generated code does this on **DK builds
with no nonlinear devices** (M = 0): the dirty-flag consumer
`apply_matrix_changes` applies one Sherman–Morrison update of `S` per changed
pot, chained sample to sample, with two guards that fall back to the full
rebuild. Every other build (M > 0 on DK, and the nodal route) still rebuilds in
full on a pot change; `DYNAMIC_PARAMS.md` has the user-facing contract. The
same formula is used, independently, by the saturating-inductor rank-1 core
term (`SATURATING_TRANSFORMERS.md`, `saturating_core.rs`).

## Source Files

| Component | File |
|-----------|------|
| `rank1_pots` gate, `emit_rank1_methods` (`apply_matrix_changes`, `rank1_update_s`), `RANK1_*` constants, setter deadband | `crates/melange-solver/src/codegen/rust_emitter/dk_emitter.rs` |
| `pot_N_g_stamped` / `pot_N_g_max` / `structural_dirty` / `diag_rank1_*` fields, their reset and sample-rate re-sync | `crates/melange-solver/templates/rust/state.rs.tera` |
| Dirty-flag consumption site | `crates/melange-solver/templates/rust/process_sample.rs.tera` |
| `PotentiometerIR` (node_p, node_q, range, g_nominal) | `crates/melange-solver/src/codegen/ir/mod.rs` |
| Tests | `crates/melange-solver/tests/rank1_pot_update_tests.rs`, `runtime_source_tests.rs::runtime_r_setters_batch_into_one_deferred_rebuild` |

## Mathematical Foundation

Given an invertible `A` with inverse `S = A⁻¹`, and a rank-1 update
`A' = A + dg·u·uᵀ`:

```
S' = (A + dg·u·uᵀ)⁻¹
   = S − (dg / (1 + dg·uᵀ·S·u)) · (S·u)(uᵀ·S)
```

For a resistor between MNA rows `p` and `q` (1-indexed nodes `node_p`,
`node_q`; 0 = ground, which has no row):

```
u        = e_p − e_q            (e_p alone when one terminal is ground)
dg       = 1/R_new − 1/R_stamped
S·u      = column p of S minus column q      (N-vector)
uᵀ·S     = row p of S minus row q            (N-vector; equals (S·u)ᵀ when S is symmetric)
uᵀ·S·u   = (S·u)[p] − (S·u)[q]               (the resistance A presents between the rows)
denom    = 1 + dg·uᵀ·S·u
S'       = S − (dg/denom) · (S·u)(uᵀ·S)
```

Changing a resistor modifies `G` by `dg·(e_p − e_q)(e_p − e_q)ᵀ`; `A = G + α·C`
and `C` is unchanged, so `A' = A + dg·u·uᵀ`. The history matrix `A_neg = α·C`
(charge form, `COMPANION_MODELS.md`) carries no `G` term, so a conductance
change touches nothing but `A` and `S`. With M = 0 there is no `K`, no `S·N_i`
and no backward-Euler set, which is why the linear DK build is the first (and
so far only) rank-1 build.

## Implementation (linear DK builds)

`rank1_update_s(p, q, dg)` is the formula above, O(N²), stack only. It
computes `S·u`, `uᵀ·S` and the denominator first and returns `false` without
touching `S` if the denominator is non-finite or `|denom| < RANK1_DENOM_MIN`
(1e-6: the element is nearly the only path between its rows and is being
removed).

`apply_matrix_changes` runs at the single dirty-flag consumption site (top of
`process_sample`, unchanged since `c7bb887`):

1. **Structural change** (`structural_dirty`, set by every `set_switch_N`,
   by `reset()`, and at construction; cleared by every full rebuild and by a
   same-rate `set_sample_rate` that loads the baked nominal matrices) → full
   `rebuild_matrices`.
2. **Ratio trigger**: for any pot, `g_now · RANK1_REBUILD_RATIO < pot_N_g_max`
   (the largest conductance stamped *since the last full rebuild*, not the
   value at the last rebuild) → full rebuild, `diag_rank1_trigger_rebuild_count += 1`.
3. Otherwise, for each pot whose `1/R` differs from `pot_N_g_stamped`: one
   `rank1_update_s`; on `false` → full rebuild, `diag_rank1_fallback_count += 1`.
   On success `pot_N_g_stamped` and `pot_N_g_max` advance and
   `diag_rank1_update_count += 1`.

Every full rebuild re-syncs `pot_N_g_stamped = pot_N_g_max = 1/R` for all pots
and clears `structural_dirty`. Multiple pots changing in one sample are
sequential exact updates (each from the matrix the previous one produced); no
cross-correction is needed because nothing is precomputed.

### Why the ratio trigger (and why a count-based refresh is not it)

The chained update is exact on the matrix it actually holds. Rounding stamped
while a pot's conductance was high therefore persists as a fixed backward error
in `S`; when that conductance later falls by orders of magnitude the fixed
error becomes relatively large. A rising conductance is benign. A refresh every
K updates does not address this (the error is a function of the conductance
path, not of the count), and the denominator guard never sees it (denominators
stayed ≥ 0.42 while it happened). Rebuilding whenever a pot has fallen an order
of magnitude below its high-water mark bounds the ratio between the stamping
and the current regime at 10.

Measured on a 66-node linear DK deck with two 1 kΩ–10 MΩ photoresistors
driven every sample through an asymmetric lag (20 ms attack / 150 ms release),
S entrywise relative error against a full rebuild (entries above 1e-12·max,
worst over 0.1 s checkpoints, ten attack/release cycles), in the deck's
ill-conditioned register setting (the two cells hanging on 1 GΩ switch legs):

| Trajectory | No trigger | `RANK1_REBUILD_RATIO` = 10 | = 100 |
|---|---|---|---|
| Both cells in phase | 5.2e-9 | 1.8e-9 | 1.2e-9 |
| One attacking while the other releases | 5.7e-6 | 8.0e-10 | 2.5e-9 |
| 1 kΩ ↔ 10 MΩ snaps | 3.4e-8 | 5.1e-11 | 5.1e-11 |

In the well-conditioned setting every variant sits at a flat 1.0e-7 that is
also present after a single update from a fresh rebuild: it is the
comparison's floor on the entries just above the 1e-12·max threshold (the
maximum is set by the 1 GΩ legs), not accumulation. Output difference against
the full-rebuild path over three cycles: ≤ 1.2e-11 V on a 25 mV peak. No
denominator fallback occurred in any run (minimum denominator 0.42).

### Cost

Same deck, generated code, one core, moving both cells every sample:

| | per sample |
|---|---|
| Resting (no change) | 1.58 µs |
| Full rebuild (`rebuild_matrices`, N = 66) | 134 µs |
| Moving, rank-1 path (median) | 2.24 µs — 1.4× resting |
| Worst 64-sample block, rank-1 path, trigger rebuilds included | 298 µs against a 1333 µs budget |

Full-rebuild path for comparison: 136 µs per moving sample (0.15× realtime at
48 kHz on one core).

## Setter deadband

Every pot / `.runtime R` setter (DK and nodal) ignores a change of at most
`1e-12 × stored value`. The comparison is against the *stored* value, so
sub-threshold creep accumulates until it stamps and never drifts. The earlier
absolute `1e-12 Ω` threshold was below one ulp at megohm values and never
fired, which is what made a converging exponential rebuild for seconds.

## History

A 2026-03 scheme corrected `v_pred`, `K` and `S·N_i` per sample from the
*nominal* matrices with codegen-time precomputed vectors (`su`, `usu`,
`nv_su`, `u_ni`) and sequential cross-correction across pots. It was removed
in 2026-04 (`eaee955`) after a 2000:1 wiper broke Newton convergence on a
nonlinear deck; the cause was later traced to DC-OP seed staleness (fixed by
`recompute_dc_op()`), not to the update, which is exact. The current scheme
updates `S` itself, from the value it holds rather than from nominal, and is
limited to builds where `S` is the only matrix a pot touches.

## Common Bugs

| Symptom | Cause | Fix |
|---------|-------|-----|
| Pot change has no effect | Setter deadband swallowed it (`|Δr| ≤ 1e-12·r`) or value equals the stored one | Expected; nothing to fix |
| `diag_rank1_fallback_count` > 0 | Denominator near zero: the element is almost the only path between its rows | Each is a full rebuild, output unaffected; worth checking the deck |
| Many `diag_rank1_trigger_rebuild_count` | Wide downward conductance sweeps (three per 1 kΩ → 10 MΩ decade sweep is normal) | Expected; each is one O(N³) sample |
| S differs from a full rebuild by more than ~1e-9 relative | `structural_dirty` not set by a new structural setter, or a full rebuild path that does not re-sync `pot_N_g_stamped` | Both belong in `rebuild_matrices` / the setter; see tests |
