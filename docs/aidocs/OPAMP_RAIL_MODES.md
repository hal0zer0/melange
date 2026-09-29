# Op-Amp Rail Mode Reference

The op-amp macromodel in melange has FIVE rail-handling modes for circuits where the op-amp output saturates against finite VCC/VEE rails. This doc consolidates what each mode does, what's been tried, what's still open, and which mode is the production default.

> **Read first**: this doc summarizes ~5 sessions of investigation. The full session-by-session history is in agent memory at `task_12_bistable_oscillation_finding.md`. Before making any change to op-amp rail handling, read that file in full — it contains empirical sweep data and rejected fix candidates that are easy to re-discover otherwise.

## Slew-rate limiting is orthogonal to rail mode

The `.model OA(SR=…)` slew-rate parameter is a **separate** mechanism from
the rail-clamping modes documented below. Rail modes handle what happens
when the op-amp output hits VCC/VEE; SR handles the large-signal dV/dt
limit that applies *below* the rails (when the input stage's tail current
can't drive enough charge into the dominant-pole cap).

The slew clamp is applied per-sample on the op-amp output node, AFTER the
rail clamp in the codegen pipeline order. Implementation lives in
`rust_emitter::nodal_emitter::emit_opamp_slew_limit` for the two nodal
codegen paths, plus an `opamp_slew` section in `process_sample.rs.tera`
for the DK Schur path. See `docs/aidocs/DEVICE_MODELS.md` for the physical
justification and emitted-code shape. The clamp is compatible with every
rail mode including `BoyleDiodes`; it does not address the BoyleDiodes
heavy-clip convergence bug documented below.

## TL;DR for production

| Circuit | Auto-detect picks | Why |
|---|---|---|
| Klon Centaur (audio-path distortion) | `ActiveSetBe` | Cap-coupled output, no R-only path to nonlinear devices, BE damps the trap-rule Nyquist limit cycle |
| VCR ALC (sidechain compressor) | `ActiveSet` | Control-path: rail-clipped value drives a secondary nonlinear device; need exact pinned voltage, not soft saturation |
| SSL bus compressor | `ActiveSetBe` | Same audio-path reasoning as Klon |
| Anything else with finite rails | `ActiveSet` | Fallback for control-path-style topologies |
| `.model OA(GBW=…)` with no VCC/VEE/VSAT | a **clamped** mode (Hard/ActiveSet/ActiveSetBe by topology) | GBW triggers the ±13 V auto-default rails (`mna.rs`, priority VCC/VEE > VSAT > GBW-default), so the op-amp is NOT rail-free. Caveat: the default applies per side — a single-supply card like `OA(VCC=9 GBW=3MEG)` gets an asymmetric 9 V / −13 V clamp window, not 9 V / 0 V |
| Truly infinite rails (no VCC, no VEE, no VSAT, **and no GBW**) | `None` | Linear VCCS, no clamping needed |

The auto-detector lives in `codegen::ir::resolve_opamp_rail_mode` + `refine_active_set_for_audio_path` (`crates/melange-solver/src/codegen/ir.rs:479-599`). It inspects the netlist topology around each clamped op-amp and picks the rail mode based on whether the output is cap-coupled (audio path) or DC-coupled to other nonlinear devices (control path).

## Which solver runs which mode

`ActiveSet` and `ActiveSetBe` pin a railed output and re-solve the rest of the
circuit. **Only the nodal solver implements that.** The DK path implements
`Hard` (and `None`) only.

- `routing::auto_route` takes the requested rail mode, resolves it on the MNA
  (`resolve_opamp_rail_mode`), and routes to nodal when a clamped op-amp
  resolves to either active-set mode (`RoutingDecision::opamp_active_set`).
  The audio-path refinement to `ActiveSetBe` happens later in codegen and does
  not affect routing — both variants are nodal-only.
- `--solver dk` on such a circuit is refused (CLI forced-DK blocker), and
  `CodeGenerator::generate*` refuses an active-set mode on the DK path. There is
  no silent degrade to `Hard`: `Hard` on an AC-coupled output is exactly the
  cap-history corruption the resolver picked active-set to avoid.
- The pinned Newton stamps device Jacobians through N_i/N_v and the
  saturating-inductor flux rows (at the site's alpha), and accepts an iterate
  on the same step check and flux-row residual as the main loop, pinned rows
  excluded. A saturating inductor's current starts from `v_prev`, not from the
  unpinned solve: that solve has the op-amp far past its rail and drives the
  winding deep into saturation, where Newton on tanh 2-cycles (measured: 37 mA
  start on a 2 mA choke, alternating −3.1 / +7.0 mA to MAX_ITER). Acceptance:
  `opamp_railing_regression_tests.rs`, a railing op-amp into a 100 mH / 2 mA
  choke against an ngspice twin, within 5 % on i_L max and H1 at 4× in both
  active-set modes. At 1× both miss for integrator reasons (backward Euler on
  nearly every railed sample; the trapezoidal ring in deep saturation); those
  cases are recorded, ignored.
- A pinned solve that does not converge is committed and counted in
  `diag_nr_unconverged_commit_count`, which every verb refuses. Schur counts it
  at commit from `last_nr_iterations`; full-LU counts it at the failure, per
  internal sample, because a count derived at the end of the host sample would
  miss every oversampled sub-step but the last.
- Active-set is **refused** on a circuit that also has a behavioral source:
  its Jacobian is stamped in node space and is not diagonal, so the pinned
  system does not include it and could converge to a non-solution. The explicit
  modes are NOT a known workaround: neither is measured with a behavioral
  source, and on the railing op-amp into a saturating choke `hard` measured
  2–290× the inductor current (138 V out of a 9 V supply) and `boyle-diodes`
  27 % low.
- An explicit `--opamp-rail-mode hard` stays allowed, on either route — an
  explicit choice is never overridden, because overrides are how users bisect.
  When a clamped op-amp it applies to is AC-coupled downstream (where auto
  would pick active-set), codegen warns, naming the op-amp(s), and the reason
  records the overridden verdict: `user requested (auto: …)`. Measured cost of
  ignoring it: a single-supply stage at 0.5 V drive reached 11 kV on a 9 V
  supply under Hard, 4.5 V under active-set.
- Every generated file states what it runs: `pub const OPAMP_RAIL_MODE: &str`
  (resolved, never `auto`) and `pub const OPAMP_RAIL_MODE_REASON: &str`.

Why not a DK active-set: nodal already implements both modes, and nothing has
shown nodal's cost to be a problem for a circuit that needs one.

On the nodal Schur path, a rail-engaged sample goes straight to the BE fallback
and its pin-and-resolve on the BE matrices. There is no 2× sub-step recovery: it
used to exist, but its only rail handling was a post-solve clamp that did not
re-solve downstream nodes (`Hard` at twice the rate), and its fixed-point solve
could not contract once a junction conducted.

## The 5 modes

| Mode | What it does | Cost | Status |
|---|---|---|---|
| `None` | Linear VCCS at op-amp output, no clamping | Cheapest | Default for ideal-rail op-amps |
| `Hard` | Post-NR `v[out].clamp(VEE, VCC)` after the trap step | ~free | Broken: cap history corruption (see `opamp_rail_clamp_bug.md`) |
| `ActiveSet` | Detect rail engagement, pin v[out] = rail, re-solve constrained; the sample after each pin or release is solved on BE (transition-BE, below) | +1 LU per engaged sample; one BE sample per pin/release | Production for control-path topologies |
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

The chord LU produces a step of ~2.4 million volts, the global `damp_thresh = 10.0 V` step cap (`nodal_emitter.rs`, grep `damp_thresh`) clips it to ±10 V from the previous iterate, and the clipped step lands in a regime where the diode either far-forward-biases or stays off — producing the **apparent** "bistable" 7 V ↔ 17 V cycle that earlier sessions misdiagnosed as two chord-LU fixed points. There is one chord LU; its predicted step is wrong by 6 OOM; the damping cap masks the magnitude error as a 2-cycle.

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

1. Pin `v[out] = clamp(v[out], VEE, VCC)` (row/column elimination: row `out` becomes `v[out] = rail`)
2. Use the BE matrices `A_be = G + (1/T) * C` (more damped than `A = G + (2/T) * C`)
3. **Newton on the pinned nonlinear system**: each iteration re-evaluates every device at the pinned iterate, stamps `−N_i·J_dev·N_v` and the companion current, solves, and applies the same pnjlim/fetlim and 10 V node-step limits as the full-LU loops. A pinned solve that does not converge marks the sample unsolved.
4. Commit the pinned solution with `i_nl` re-evaluated at it, so the next sample's cap history is BE-consistent

Step 3 used to be ONE linear solve with the unpinned solve's device currents frozen. That is only a solution if the pin leaves device voltages where they were. It does not when an output coupling cap sits between the op-amp and a nonlinear device: the cap passes the pin's step straight through. On a single-supply overdrive with a diode clipper after the output cap, the frozen solve drove the clipper node to −2 V and re-evaluated a reverse diode at 3.6e9 A; the next sample diverged. Nothing had validated active-set with M > 0 at the rail — the corpus has no deck whose op-amp rails — which is why `opamp_railing_regression_tests.rs` now carries one, gated against an ngspice reference (±5 %; measured within 2.4 % at 1×, 1.0 % at 4×).

The crucial difference from plain ActiveSet is that the BE re-solve damps any high-frequency content in the cap-coupled output path that the trap rule would otherwise amplify into a Nyquist limit cycle. Klon's C15 (4.7 µF, tone_out → out_ac) plus the surrounding R network forms a discrete-time LC resonator at exactly Nyquist when discretized with the trap rule; the BE re-solve sidesteps this by using a different discretization for the rail-engaged sample.

Code: search `rust_emitter/nodal_emitter.rs` for `emit_nodal_active_set_resolve`.

## How `ActiveSet` differs from `ActiveSetBe`

ActiveSet (without "Be") does the same pin-and-resolve but on the trap matrices `state.a` instead of `state.a_be`. This works correctly for **control-path** topologies (e.g. VCR ALC sidechain) where the rail-clipped op-amp output drives a secondary nonlinear device's operating point — the secondary device wants the EXACT pinned voltage, and the trap rule's higher-frequency response is desirable for fast envelope detection.

For **audio-path** topologies (e.g. Klon, SSL), the trap rule's response IS the bug — it excites the output coupling cap's Nyquist resonance. ActiveSetBe replaces it.

The auto-detector picks ActiveSet vs ActiveSetBe based on whether the op-amp's output has a cap-coupled path to the speaker (audio) vs a DC path to another nonlinear device (control). See `refine_active_set_for_audio_path` in `ir.rs`.

## Transition-BE: one backward-Euler sample per pin or release (`ActiveSet`)

A pin replaces the op-amp's output row with the rail constraint; a release gives
it back. That is an equation-set swap of the same kind as a `.switch` toggle.
The sample that makes the swap is solved on trapezoidal history built on the old
set, and the mismatch goes into trap's `z = −1` mode. On a **capless nonlinear
row** downstream of the pinned output (the diode node of a clipper behind the
output coupling cap and a resistor) that mode never decays: the row satisfies
only the two-sample *average* of its KCL. Fingerprint: the row's KCL residual
alternates in sign every sample, `r_n + r_(n−1) ≈ 0`, with `|r|` far above the
floor. A filtered output hides it (the `out` node of the test deck below
carries ~1e-9 V at Nyquist).

On a trapezoidal nodal build in `ActiveSet` mode with a clampable op-amp
(`SolverConfig::transition_be`), a change in any op-amp's pin state between the
committed previous sample and this one arms the breakpoint-BE countdown. The
pin is its third source, after the `.switch`/`.pot` setters and the glow. The
next sample runs the same backward-Euler solve a `--backward-euler` build runs,
on both nodal sub-paths and on the linear (M = 0) solves. The comparison uses
the inclusive rail tests of the active-set check against `state.v_prev`, so it
needs no state of its own and is right after a DC OP, a `reset()` or a NaN
recovery. `diag_transition_be_count` counts the pin changes. The build header
and provenance JSON say `transition-be`. `ActiveSetBe`, `Hard`, `None` and BE
builds emit nothing for it.

Measured on a single-supply overdrive (TL072 card, AOL 200k, rails 0/9 V, gain
~107, output cap → 1k → antiparallel 1N914 → 10k/22n → output), 1 kHz, 1×,
1 s, ngspice reference with the op-amp as an ideal clamped VCCS (matched to the
melange model; converged: 0.5 µs and 0.1 µs/reltol 1e-5 agree to 6e-6):

| Drive | Build | n2 KCL residual, last 0.1 s (48k / 96k / 192k) |
|---|---|---|
| 0.1 V | ActiveSet without transition-BE | 0.77 / 191 / 153 µA |
| 0.1 V | ActiveSet with transition-BE | 0.29 / 0.56 / 0.26 µA |
| 0.5 V | ActiveSet without transition-BE | 105 / 230 / 2650 µA |
| 0.5 V | ActiveSet with transition-BE | 0.44 / 0.96 / 1.90 µA |
| both | ActiveSetBe | 0.27–0.40 µA |

The BE count equals the pin-transition count exactly (4 per cycle). What
remains with transition-BE is not the transition: the BE sample reads
0.006 µA. The residual regrows within each rail plateau. That is the pinned
resolve's acceptance, see STATUS Pending Work.

**Rate convergence of the output peak, and why it is measured incommensurate.**
The acceptance for this change was pre-registered as the output-peak error at
1 kHz, required to converge monotonically with rate and to be no worse than
`ActiveSetBe` at each rate. It was **amended after the run** to the worst-case
per-cycle peak error under an incommensurate drive (1001.3 Hz). The reason:
at 1 kHz every test rate has an integer number of samples per cycle, so the
rail-edge timing error is phase-locked to the grid, and each rate samples one
fixed point of an O(T) band. That metric measures grid alignment, not
convergence. The same band shows in the peak-to-peak error. The amended metric
is applied identically to every mode, and "no worse than `ActiveSetBe`" holds
under both.

| Drive | Build | 1 kHz peak error (48k / 96k / 192k) | 1001.3 Hz worst-case per-cycle error |
|---|---|---|---|
| 0.1 V | ActiveSet + transition-BE | −0.902 / −0.307 / −0.088 % | 1.073 / 0.314 / 0.116 % |
| 0.1 V | ActiveSetBe | −2.445 / −0.948 / −0.513 % | 2.889 / 1.061 / 0.528 % |
| 0.5 V | ActiveSet + transition-BE | −0.251 / −0.067 / −0.243 % | 1.505 / 0.474 / 0.277 % |
| 0.5 V | ActiveSetBe | −2.179 / −1.332 / −0.710 % | 3.648 / 1.953 / 0.814 % |
| 0.5 V | ActiveSet without transition-BE | −0.969 / −0.052 / −0.747 % | 2.219 / 1.070 / 2.501 % |

**Rule for future gates:** a rate-convergence gate on an edge-driven deck uses
an incommensurate drive frequency by default.

`ActiveSetBe` costs no more CPU than `ActiveSet` + transition-BE on this deck
(6.1 vs 6.8 µs/sample at 48k, within run noise). Its cost is accuracy: it runs
BE on 73–96 % of samples (whole rail plateaus), and its first-order error is
2–4× the transition-BE error in every cell above.

A control-path deck (inverting stage, rails ±9 V, driving a rectifier diode
through 10k into 2 MΩ, no capacitor at either diode node) is auto-promoted to
a backward-Euler build, so neither mechanism applies to it as routed. Forced to
trap, the lock appears at 192k only (33.7 µA on the rectifier node, alternating)
and transition-BE removes it (0.13 µA). At 48k and 96k that deck sits at the
floor either way. **This is confirmed under a forced trap only.**

Auto resolution is unchanged: the audio-path class still resolves to
`ActiveSetBe`. Moving both classes to `ActiveSet` + transition-BE waits on the
pinned-resolve convergence item and on a golden render that actually pins
under `ActiveSet` (the corpus has none).

Code: `emit_transition_be_detect` / `emit_transition_be_arm` in
`rust_emitter/helpers.rs`. Tests: `transition_be_tests.rs`.

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
| `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs` | grep `trap_rail` | Trap-path mode dispatch (post-NR rail handling) |
| `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs` | grep `be_rail` | BE-fallback mode dispatch |
| `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs` | grep `residual_check` | Residual check (BoyleDiodes-gated) |
| `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs` | grep `adaptive_refactor` | Adaptive refactor trigger (BoyleDiodes-gated) |
| `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs` | grep `damp_thresh` | `damp_thresh = 10.0_f64.max(max_v * 0.05)` (the global step cap that masks BoyleDiodes overshoot) |

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
