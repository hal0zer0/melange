# Op-Amp Rail Mode Reference

The op-amp macromodel in melange has FIVE rail-handling modes for circuits where the op-amp output saturates against finite VCC/VEE rails. This doc consolidates what each mode does, what's been tried, what's still open, and which mode is the production default.

> **Read first**: this doc summarizes ~5 sessions of investigation. The full session-by-session history is in agent memory at `task_12_bistable_oscillation_finding.md`. Before making any change to op-amp rail handling, read that file in full — it contains empirical sweep data and rejected fix candidates that are easy to re-discover otherwise.

## TL;DR for production

| Circuit | Auto-detect picks | Why |
|---|---|---|
| Klon Centaur (audio-path distortion) | `ActiveSetBe` | Cap-coupled output, no R-only path to nonlinear devices, BE damps the trap-rule Nyquist limit cycle |
| VCR ALC (sidechain compressor) | `ActiveSet` | Control-path: rail-clipped value drives a secondary nonlinear device; need exact pinned voltage, not soft saturation |
| SSL bus compressor | `ActiveSetBe` | Same audio-path reasoning as Klon |
| Anything else with finite rails | `ActiveSet` | Fallback for control-path-style topologies |
| Anything with infinite rails (`.model OA(GBW=…)` only) | `None` | Linear VCCS, no clamping needed |

The auto-detector lives in `codegen::ir::resolve_opamp_rail_mode` + `refine_active_set_for_audio_path` (`crates/melange-solver/src/codegen/ir.rs:479-599`). It inspects the netlist topology around each clamped op-amp and picks the rail mode based on whether the output is cap-coupled (audio path) or DC-coupled to other nonlinear devices (control path).

## The 5 modes

| Mode | What it does | Cost | Status |
|---|---|---|---|
| `None` | Linear VCCS at op-amp output, no clamping | Cheapest | Default for ideal-rail op-amps |
| `Hard` | Post-NR `v[out].clamp(VEE, VCC)` after the trap step | ~free | Broken: cap history corruption (see `opamp_rail_clamp_bug.md`) |
| `ActiveSet` | Detect rail engagement, pin v[out] = rail, re-solve constrained | +1 LU per engaged sample | Production for control-path topologies |
| `ActiveSetBe` | Same as ActiveSet but re-solve uses BE matrices instead of trap | +1 LU + BE solve per engaged sample | **Production for audio-path topologies** |
| `BoyleDiodes` | Augment netlist with internal gain node + catch diodes per clamped op-amp; let NR handle the saturation through the diodes | +N nodes per op-amp; ~1.5x NR work | **Light clip only (amp ≤ 0.05 V)**; diverges at heavy clip |

## How `BoyleDiodes` works

When `opamp_rail_mode == BoyleDiodes`, `codegen::ir::augment_netlist_with_boyle_diodes` (`ir.rs:708`) clones the netlist and adds, per clamped op-amp:

1. Internal gain node `_oa_int_{name}` (the high-impedance summing point)
2. Output buffer chain:
   - Intermediate buffer-output node `_oa_buf_out_{name}`
   - Unity-gain VCVS `E_oa_buf_{name}`: `V(buf_out) = V(int)`
   - Series resistor `R_oa_ro_{name} = 75 Ω` between `buf_out` and the user's output node (canonical Boyle 1974 / TL072 macromodel topology)
3. Two rail-reference DC voltage sources `V_boyle_hi_{name} = VCC - VOH_DROP` and `V_boyle_lo_{name} = VEE + VOL_DROP`
4. Two catch diodes `D_boyle_hi/lo_{name}` between the internal node and the rail references (`IS = 1e-15`, `N = 1`)

The MNA dispatcher in `mna.rs:2871` auto-detects `_oa_int_{safe_name}` in `node_map` and stamps `Gm_int = AOL / R_BOYLE_INT_LOAD` and `Go_int = 1 / R_BOYLE_INT_LOAD` at the internal node row INSTEAD of the user's output node. The output buffer chain is purely linear and is built from the augmented netlist.

```
R_BOYLE_INT_LOAD = 1.0e6     (mna.rs:374)
Gm_int = AOL / 1e6 = 0.2 S   (for AOL = 200 000)
Go_int = 1e-6 S
```

## The BoyleDiodes heavy-clip problem (OPEN)

**Symptom** (Klon at amp ≥ 0.07 V, 1 kHz sine):
- Raw op-amp output peak: 50–500 V (should be ≈ VCC ± VOH_DROP ≈ 7.5 V or 17 V)
- NR fails on >95% of samples
- Output safety clamp `output[i].clamp(-10.0, 10.0)` masks this as a flat ±10 V — **always measure raw `state.v_prev[OUTPUT_NODES[0]]` when debugging**

**Verified root cause** (2026-04-08 fourth session, three-agent verification):

Row 41 (`_oa_int_U2B`) is **near-singular in the linear part of A**:

```
G[41][3]  = +0.2    (Gm * v_vbias)
G[41][24] = -0.2    (-Gm * v_sum_out)
G[41][41] = +1e-6   (Go_int self-load — 6 OOM smaller than the off-diagonals)
```

At heavy clip, `v_diff = v[vbias] - v[sum_out] ≈ -12 V`, so the VCCS sources `+2.4 A` into row 41. The chord LU's predicted equilibrium (assuming `jdev ≈ 0`, catch diode reverse-biased) is:

```
v[41] ≈ 2.4 A / 1e-6 S = 2.4 × 10⁶ V
```

The chord LU produces a step of ~2.4 million volts, the global `damp_thresh = 10.0 V` step cap (`rust_emitter.rs:7121`) clips it to ±10 V from the previous iterate, and the clipped step lands in a regime where the diode either far-forward-biases or stays off — producing the **apparent** "bistable" 7 V ↔ 17 V cycle that earlier sessions misdiagnosed as two chord-LU fixed points. There is one chord LU; its predicted step is wrong by 6 OOM; the damping cap masks the magnitude error as a 2-cycle.

The fundamental problem: row 41's diagonal (`Go_int = 1e-6`) is too small relative to its off-diagonal sources (`Gm = 0.2 S`) for any chord LU to produce a sensible step when the catch diode is in the wrong linearization regime.

## Fix candidates already tested and REJECTED

All tested 2026-04-08 against Klon BoyleDiodes at amp = [0.01, 0.03, 0.05, 0.07, 0.10, 0.15, 0.20, 0.30, 0.50] V. **DO NOT RE-TEST** unless you have new evidence that previous testing was flawed.

| Fix | Description | Why it fails |
|---|---|---|
| Targeted Gmin bump on `_oa_int_*` rows | `chord_lu[oa_int][oa_int] += 5e-2` | Destroys op-amp DC gain in linear regime (amp=0.01 raw_peak jumps from 2.61 V to 10+ V with 77% NR fails). No value balances "bounds chord LU runaway" against "doesn't short-circuit AOL". |
| Force refactor every iter (ngspice-style) | `need_refactor = true` always | Preserves linear regime, doesn't fix heavy clip. The chord LU at any single jdev sample produces a wrong step. Refactoring more often doesn't change that. |
| Disable `damp_thresh` global step cap | Remove the ±10 V clip | Preserves linear regime, doesn't fix heavy clip. Removing the cap unmasks the 2.4 million volt step but doesn't fix its direction. |
| Global Gmin bump (1e-12 → 1e-6) | All diagonal entries | Breaks even amp=0.01 by altering linear behaviour at high-Z nodes (cap-coupled paths see 1 µS shunt to ground that competes with parasitic cap conductances). |
| Pseudo-transient continuation (PTC) on row 41 only | Add `1/Δτ * I` to chord_lu diagonal before factoring | **Originally claimed to fail due to static-pivot interaction; the third-session "smoking gun" was a testing-protocol error** (different amp literals in the two compared files). PTC is **NOT formally refuted** — the third-session conclusion was retracted by the fourth session. Still untested with corrected protocol. |
| Output buffer Ro chain (this session, 2026-04-08 fifth session) | Series 75 Ω between VCVS and op-amp output via intermediate `_oa_buf_out_` node | Reduces heavy-clip raw peak from 3068 V to ~306 V (10× improvement) but doesn't get to the [10, 12] V target. The Ro provides physical source impedance but doesn't fix row 41 conditioning. Committed in `f32c804` as it's strict progress and matches canonical Boyle topology. |
| C_dom dominant-pole cap at `_oa_int_` | Capacitor from int_node to ground, sized from GBW | Any value > ~5 pF breaks linear regime. The cap's trap-rule conductance `2C/T` competes with the tiny `Go_int = 1 µS` self-load and ill-conditions the int row further. |
| C_dom at `_oa_buf_out_` | Same cap, different node | No-op: VCVS forces V(buf_out) = V(int), so the cap can't store a different voltage. |
| Backward Euler in BoyleDiodes mode | `--backward-euler` CLI flag | Doesn't fix heavy clip. The chord-LU runaway is independent of trap vs BE. |

## Next-tier escalation (UNTESTED, ranked)

If BoyleDiodes heavy-clip convergence becomes a priority again:

1. **BoyleDiodes → ActiveSetBe failure hybrid** (~30 lines). On NR failure in BoyleDiodes mode, fall through to ActiveSetBe's pin-and-resolve path on BE matrices instead of letting the raw output diverge. Lowest-risk escalation: ActiveSetBe is already production-quality on Klon, so the worst case of the hybrid is "it works as well as ActiveSetBe alone".

2. **Re-test PTC on row 41 with corrected testing protocol** (~30 lines). The third session's PTC rejection was based on a confounded A/B test (different amp literals in the two compared files). The mechanism (regularize the chord LU diagonal to mechanically bound the operator norm) directly addresses row 41's near-singularity. Use disciplined sweep: same compile flags, same harness, single-variable changes, all 9 amplitudes per test.

3. **Anderson acceleration m=3** (~80 lines). Walker-Ni 2011 + Zhang-Peng-Ouyang 2018 safeguarding. KINSOL `kinsol.c:2683-2957` is the reference implementation. Strong theoretical fit for bistable-Newton failure modes; no SPICE-class simulator uses it because they all do full Newton + line search instead. melange's chord-LU framework is what makes Anderson interesting here.

4. **Real Boyle two-stage with R1 = 1 kΩ** (Untested, no memory record). Currently `R_BOYLE_INT_LOAD = 1 MΩ` with `Gm_int = AOL/R1 = 0.2 S`. Switching to R1 = 1 kΩ keeps `AOL = Gm_int * R1` at 200 000 but changes `Go_int = 1 mS` (1000× larger) and `Gm_int = 200 S` (1000× larger). Row 41 becomes `(diagonal 1 mS, off-diagonal 200 S)` — same 1e5 ratio, BUT in absolute terms 1000× better-conditioned for floating point. Risk: `Gm_int = 200 S` may destabilize other parts of the linear system. Test before assuming.

## How `ActiveSetBe` actually works (production reference)

ActiveSetBe runs the trap NR loop normally, but at the end of each sample's NR convergence check, it inspects whether any clamped op-amp output is at or beyond its rail. If yes, it falls through to a constrained re-solve:

1. Pin `v[out] = clamp(v[out], VEE, VCC)` (in voltage space, not via NR)
2. Build `A_be = G + (1/T) * C` (BE matrices, more damped than `A = G + (2/T) * C`)
3. Solve `A_be * v_be = rhs_be` with the pinned output as a constraint via augmented MNA
4. Update `state.v_prev = v_be` so the next sample's cap history is BE-consistent

The crucial difference from plain ActiveSet is that the BE re-solve damps any high-frequency content in the cap-coupled output path that the trap rule would otherwise amplify into a Nyquist limit cycle. Klon's C15 (4.7 µF, tone_out → out_ac) plus the surrounding R network forms a discrete-time LC resonator at exactly Nyquist when discretized with the trap rule; the BE re-solve sidesteps this by using a different discretization for the rail-engaged sample.

Code: search `rust_emitter.rs` for `emit_nodal_active_set_resolve`.

## How `ActiveSet` differs from `ActiveSetBe`

ActiveSet (without "Be") does the same pin-and-resolve but on the trap matrices `state.a` instead of `state.a_be`. This works correctly for **control-path** topologies (e.g. VCR ALC sidechain) where the rail-clipped op-amp output drives a secondary nonlinear device's operating point — the secondary device wants the EXACT pinned voltage, and the trap rule's higher-frequency response is desirable for fast envelope detection.

For **audio-path** topologies (e.g. Klon, SSL), the trap rule's response IS the bug — it excites the output coupling cap's Nyquist resonance. ActiveSetBe replaces it.

The auto-detector picks ActiveSet vs ActiveSetBe based on whether the op-amp's output has a cap-coupled path to the speaker (audio) vs a DC path to another nonlinear device (control). See `refine_active_set_for_audio_path` in `ir.rs`.

## The `Hard` mode bug (historical)

`Hard` was the original mode: `v[out].clamp(VEE, VCC)` after the NR converged. Looks correct, completely broken in practice. The cap history (`v_prev` for the trap rule's `(2C/T) * v_prev` term) gets corrupted when v[out] is mutated post-NR — the next sample's history term assumes the unconstrained v, not the clamped v, leading to KCL violation that propagates as a slow drift / oscillation.

See `opamp_rail_clamp_bug.md` for the full history. This mode is kept in the enum only for backward compatibility and explicit `--opamp-rail-mode hard` testing. **Never use it in production.**

## Code references

| File | Lines | What |
|---|---|---|
| `crates/melange-solver/src/codegen/mod.rs` | 84-128 | `OpampRailMode` enum, parser, Display |
| `crates/melange-solver/src/codegen/ir.rs` | 479-599 | `resolve_opamp_rail_mode` + `refine_active_set_for_audio_path` (auto-detector) |
| `crates/melange-solver/src/codegen/ir.rs` | 708-835 | `augment_netlist_with_boyle_diodes` (BoyleDiodes scaffolding) |
| `crates/melange-solver/src/mna.rs` | 374 | `R_BOYLE_INT_LOAD = 1e6` (the R1 value) |
| `crates/melange-solver/src/mna.rs` | 2847-2964 | Op-amp stamping dispatch (BoyleDiodes detection + non-Boyle linear path) |
| `crates/melange-solver/src/codegen/rust_emitter.rs` | 5697-5749 | Trap-path mode dispatch (post-NR rail handling) |
| `crates/melange-solver/src/codegen/rust_emitter.rs` | 5968-5997 | BE-fallback mode dispatch |
| `crates/melange-solver/src/codegen/rust_emitter.rs` | 7376, 7843 | Residual check (BoyleDiodes-gated) |
| `crates/melange-solver/src/codegen/rust_emitter.rs` | 6950-6968 | Adaptive refactor trigger (BoyleDiodes-gated) |
| `crates/melange-solver/src/codegen/rust_emitter.rs` | 7121 | `damp_thresh = 10.0_f64.max(max_v * 0.05)` (the global step cap that masks BoyleDiodes overshoot) |

## Memory cross-references

These are the agent-memory files with full session-by-session investigation history. **Read in this order before making any rail-mode change**:

1. `task_12_bistable_oscillation_finding.md` — 535 lines, four sessions of investigation, RETRACTED diagnoses, verified root cause, empirical sweep data, fix-candidate test results
2. `opamp_rail_clamp_bug.md` — original Hard mode bug history, ActiveSet/ActiveSetBe genesis
3. `klon_rail_limit_attempts.md` — log of 9+ failed approaches before BoyleDiodes
4. `boyle_opamp_codegen_fix.md` — Boyle VCCS A_neg row-zeroing fix
5. `boyle_1974_rebuild_plan.md` — implementation plan from 2026-04-08 fifth session (mostly subsumed by Ro chain commit `f32c804`)
6. `klon_softrail_implementation_attempt.md` — failed SoftRail device experiment (cleanly reverted)

## External references

- **Boyle, Cohn, Pederson, Solomon (1974)** — *Macromodeling of Integrated Circuit Operational Amplifiers*, IEEE JSSC, DOI 10.1109/JSSC.1974.1050528. The original paper. Two-stage topology with internal gain node + Miller cap + output buffer + catch diodes.
- **LTspice TL072 model** — `erik-vincent/LTSpiceParts/TL072.301` on github. Modern descendant of Boyle. Catch diodes on internal node, 75 Ω output Ro.
- **ngspice Universal Op-amp (uopamp)** — same family. Behavioral GA/GB transconductances into internal high-Z nodes, HLIM/VLIM current limit, voltage clamps on internal stage.
- **Kelley, Keyes (1998)** — *Convergence Analysis of Pseudotransient Continuation*, SIAM J Numer Anal 35(2), 508-523. The PTC reference; Theorem 3.2 gives the global convergence bound for index-1 DAEs.
- **Walker, Ni (2011)** — *Anderson Acceleration for Fixed-Point Iterations*, SIAM J Numer Anal 49(4), 1715-1735. Anderson m=K with Walker-Ni safeguarding.
- **PETSc `src/ts/impls/pseudo/posindep.c`** — production PTC implementation, lines 245, 350, 699 are the SER schedule and step bound.
- **KINSOL `kinsol.c:2683-2957`** — production Anderson acceleration in C.

## Don't repeat these mistakes

- Don't propose targeted Gmin bumps on `_oa_int_*` rows. Already tested. Kills DC gain.
- Don't propose force-refactor-every-iter. Already tested. Doesn't help heavy clip.
- Don't propose disabling `damp_thresh`. Already tested. Doesn't help heavy clip.
- Don't propose global Gmin bumps without checking what value it changes from (1e-12 to 1e-6 breaks linear behaviour at high-Z nodes).
- Don't propose C_dom at `_oa_int_`. Any value > ~5 pF breaks linear.
- Don't propose Boyle 1974 as a "new idea" — the scaffolding is 90% built (`augment_netlist_with_boyle_diodes`); the open question is the heavy-clip NR convergence on the EXISTING scaffolding, not building scaffolding from scratch.
- Don't claim "Klon doesn't work". Klon ships under auto-detected `ActiveSetBe`. The user does NOT need a flag. The OPEN problem is BoyleDiodes mode at heavy clip, which is opt-in only.
