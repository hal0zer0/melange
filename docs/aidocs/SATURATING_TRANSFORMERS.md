# Saturating Transformers — Design Plan

**STATUS: NOT IMPLEMENTED. This is a design/roadmap document.**

Nothing described past §1 exists in the code today. What ships now is
*uncoupled* saturating inductors (`L1 a b 100m ISAT=20m`, lagged
`L(I)=L0/cosh²(I/Isat)`, Sherman-Morrison rank-1, 32-sample decimated,
nodal-Schur path). The coupled/transformer saturation machinery that
exists in the tree (`SaturatingTransformerGroupIR`, `winding_isats`) is
physically wrong, unvalidated, and unused — see §1.

This document is the plan to build shared-core saturating transformers
correctly. Read it before touching any `Saturating*IR`, `winding_isats`,
or the ideal-transformer T-model in `mna.rs`.

Related: [MNA.md](MNA.md) (augmented MNA, inductor branch currents),
[COMPANION_MODELS.md](COMPANION_MODELS.md) (trap/BE companions),
[NR_SOLVER.md](NR_SOLVER.md) (Jacobian assembly),
[DC_OP.md](DC_OP.md) (gmin/source-stepping continuation),
[DEVICE_MODELS.md](DEVICE_MODELS.md), [SHERMAN_MORRISON.md](SHERMAN_MORRISON.md).

---

## 1. Scope and the honesty statement

**In scope (v1):** physically-correct shared-core saturation for
transformers with **no global negative feedback through the iron**.
Marquee targets are guitar output transformers (SE and push-pull) and
1:1 character-pedal iron. Tape-head / iron-clip stages already go through
the uncoupled path.

**Explicitly deferred:** NFB-through-iron transformers (the passive EQ, Neve
1073), hysteresis / core loss / remanence, and multi-limb cores.

### Why the current independent-winding code is wrong

`SaturatingTransformerGroupIR`
(`crates/melange-solver/src/codegen/ir/mod.rs:1091-1109`) documents its
own defect: *"Each winding saturates independently based on its own
branch current."* A real transformer has **one shared core** threaded by
all windings; its single physical state is core flux Φ (equivalently the
magnetizing current `i_m`), driven by the **net MMF `F = Σ Nᵢ·Iᵢ`**, not
by any one winding's current. Under load, primary and secondary MMFs
nearly cancel (Lenz), so winding currents can be 100× the magnetizing
current while the core is *unsaturated*. Keying saturation off `|Iᵢ|`
therefore saturates the core exactly when it physically should not,
misses saturation of the true magnetizing current, breaks the
shared-permeance coupling structure (selfs and mutuals must scale by the
*same* factor — independent scaling can push `k_eff ≥ 1` and lose
positive-definiteness), and — having no flux state — cannot produce
volt-second saturation, DC-bias flux-walking, or the even-harmonic
"iron" that makes transformers audible. It has zero prior art and zero
users. **Delete it; do not fix it.**

---

## 2. The physics/model melange will adopt

### 2.1 The load-bearing discovery: the correct topology already exists

`mna.rs:3439-3560` already decomposes a coupled transformer group into
the physically-correct **T-model**: per-winding **linear leakage**
inductors + **ideal turns-ratio couplings** + exactly **one magnetizing
inductance `{ref}_mag`** (= L of the reference winding), which is added
to the *uncoupled* inductor list. Its branch current **is** the net
magnetizing current by construction, because the ideal couplings reflect
load current out.

Consequence: attaching saturation to that single `{ref}_mag` inductor
(created with `isat: None` at `mna.rs:3525-3530`) yields
physically-correct shared-core saturation **with no new coupled math**,
reusing the uncoupled-inductor saturation locus. **This collapses all
three transformer blockers into the single "uncoupled saturating
inductor" problem.** Fix uncoupled saturation once and transformers come
along for free.

**The catch:** the T-model is deliberately disabled —
`IDEAL_XFMR_L_THRESHOLD = 1e30` (`mna.rs:171`) — because the ideal
couplings form **algebraic loops in circuits with global NFB through the
iron** (the passive EQ tertiary NFB, Neve 1073). Enabling it must be **gated to
NFB-free groups**, or it regresses validated circuits. Loosely-coupled /
small transformers currently also fall through to the (wrong)
`winding_isats` path, so routing must be re-verified.

### 2.2 Magnetizing vs leakage split

- **Leakage stays strictly linear.** `L_leak,k ≈ (1−k)·L_k` (air path).
  Never fold leakage into the nonlinearity.
- **Only the shared magnetizing element saturates.** In matrix form the
  magnetizing block is a rank-1 outer product `L_mag = s(F)·n·nᵀ` with
  turns `nᵢ = √L_mag,i`; shared-core saturation is a *single* rank-1
  update, not W² elementary updates. The T-model already encodes this
  split. Guard numerics as `k → 1` (leakage → 0, block near-singular).

### 2.3 Saturation curve (v1: anhysteretic)

Use the anhysteretic flux law already shipped for uncoupled inductors:

```
Φ(i) = L0 · Isat · tanh(i / Isat)
```

Its **differential** inductance is

```
L_diff(i) = dΦ/di = L0 / cosh²(i / Isat)
dL/di     = -(2/Isat) · L_diff(i) · tanh(i / Isat)
```

This is stateless: no loss, no remanence, no minor loops. Do **not**
claim hysteresis or "iron memory" fidelity from it. Hysteresis (Chan,
Jiles-Atherton) is deferred to §8.

---

## 3. Numerical formulation

The shipped uncoupled path patches only the **trap** DK matrices
(S/K/A_neg) with a lagged Sherman-Morrison update using `I_prev`,
outside the NR loop, decimated every 32 samples. That design carries two
of the three blockers: it does not compose with the backward-Euler
fallback (the BE matrices `s_be`/`k_be` at `ir/mod.rs:662-665` receive no
patch — see the explicit "future work" note at `ir/mod.rs:3009-3017`),
and lagged/decimated evaluation is wrong for hard, fast saturation. The
fix is to **move saturation into the full-LU nonlinear NR loop as a
genuine device on the augmented magnetizing branch.**

### 3.1 State variable: prefer flux-linkage λ

Make the augmented unknown the **flux linkage λ** (or Φ), not
current-with-`L(I)`. Then Faraday `v = dλ/dt` is **linear and exact
under both trap and BE** with constant integrator coefficients, and the
nonlinearity collapses to a pure algebraic NR equation `i = g(λ)` — 1D
per core, any winding count. This is the EMTP/EMTDC-standard choice: it
decouples stiffness from the integrator, conserves flux across
timesteps (a `current + L(I)` coefficient leaks flux on fast
transients), and avoids inverting a near-zero `L_diff` deep in
saturation.

Cost: it changes the augmented state-vector layout, DC-OP seeding,
`v_prev` indexing, and OOMOX_CONTRACT struct fields. **Fallback** if
flux-state proves too invasive: keep `i` as the branch state and only
fix the Jacobian/residual (§3.2). The numerics-vs-blast-radius tradeoff
is an open question (§8).

### 3.2 Stamps and Jacobian (current-state form, the fallback)

Stamp saturation as a nonlinear device in the same slot as diodes/BJTs,
on the augmented magnetizing-branch row `k` between nodes `i, j`. The
full-LU path assembles the Jacobian fresh each iteration against a single
integrator scalar `alpha` (`nodal_emitter.rs:3147-3149`: `alpha = 1/T`
for BE, `2/T` for trap).

**Critical footgun — chord vs differential inductance.**
`L0/cosh²(i/Isat)` is the **differential** slope `dΦ/di`. It is correct
as the **Jacobian entry** but *wrong* if used as a chord `L` in a
`Φ = L·i` product. The NR **residual must use the flux integral `Φ(i)`**,
not `L_eff·i`.

Trap companion, branch row `k`:

```
R_k        = -(V_i - V_j)^{n+1} + (2/T)·Φ(i_k^{n+1}) - (2/T)·Φ(i_k^n) - (V_i - V_j)^n
∂R_k/∂V_i  = -1
∂R_k/∂V_j  = +1
∂R_k/∂i_k  = (2/T)·L_diff(i_k^{n+1})
```

Backward-Euler companion: identical with `2/T → 1/T` and the `(V_i−V_j)^n`
history term dropped.

The node-KCL rows couple to `i_k` with the usual ±1 incidence entries,
unchanged. The magnetizing rank-1 core term `(2/T)·L_diff·n·nᵀ` sits on
the augmented block; whether it can be applied via Sherman-Morrison
*inside* the NR iteration or forces full-LU unconditionally is open (§8).

### 3.3 How NR-integration dissolves the BE blocker

Because saturation is evaluated **inside** the NR loop against the single
`alpha` scalar, there is exactly one code path and **no precomputed
trap-only matrices to keep in sync**. Trap and BE differ only by
`alpha ∈ {2/T, 1/T}` and the history term. Saturation therefore composes
with auto-BE, breakpoint-BE, the runtime BE-latch, and `--backward-euler`
**for free** — this is the structural reason the in-loop approach
dissolves blocker (2), which the SM-on-trap-matrices scheme cannot.

Integrator policy is unchanged from melange's existing handling: **trap
by default, conditional BE**. Trap is A-stable but not L-stable (z=−1
Nyquist ring on a stiff saturation collapse); BE is L-stable but
over-damps the lightly-damped LC resonances that *are* the transformer's
audible character. Reuse the existing `nyquist_dbc` auto-BE detector and
breakpoint-BE. **Do not pin BE** — `.integrator be` dulls resonances and
is a stopgap, not a fix.

### 3.4 Verified melange companion-form stamps (Phase 1, current-state)

Decision recorded 2026-08-15: **current-state** (branch current `i_k` stays
the augmented unknown), resolved by an impartial arbiter against the flux-state
lean (§8-Q3). The stamps below are derived against melange's *actual* augmented
MNA and full-LU companion NR — not the abstract residual of §3.2 — and are the
implementation contract for Phase 1.

**Augmented row sign convention (verified `mna.rs:build_augmented_matrices`,
uncoupled inductor, row `k` between nodes `i,j`):**

```
G:  g[i][k] += 1 ;  g[j][k] -= 1        // KCL: branch current enters i, exits j
    g[k][i] -= 1 ;  g[k][j] += 1        // KVL row k reads  (-V_i + V_j)
C:  c[k][k]  = L0                        // self-inductance
```

Trap system `A = G + alpha·C`, `A_neg = alpha·C − G`; BE `A_neg = alpha·C`
(G dropped — no voltage-history term on BE inductor rows). Row `k` of
`A·v = A_neg·v_prev + rhs` reproduces the trapezoidal rule for `V = L·di/dt`.
Flux linkage `λ = L0·i` generalises to `Φ(i) = L0·Isat·tanh(i/Isat)`
(`Φ ≈ L0·i` as `i→0`; `L_diff = dΦ/di = L0/cosh²(i/Isat)` = the shipped
`l_eff`). The **residual/history uses `Φ`; the Jacobian uses `L_diff`** — never
`L_eff·i` (§3.2 footgun).

**The full-LU NR is companion-form**, not residual-delta: `chord_lu` starts as
the base matrix (with `alpha·L0` already at `[k][k]`), device Jacobians are
stamped in, `rhs_work = rhs + companion`, and the back-solve yields `v_new`
directly. So the saturating inductor is a **three-line correction** at each
assembly site, against that site's integrator `alpha`, frozen at the same
iterate `i0` used to factor (chord-consistent, like `chord_j_dev`):

```
Jacobian:      MAT[k][k]  += alpha·(L_diff(i0) − L0)        // alpha·L0 → alpha·L_diff
companion RHS: rhs_work[k] += alpha·(L_diff(i0)·i0 − Φ(i0))
history:       rhs[k]      += alpha·(Φ(i_prev) − L0·i_prev)  // i_prev = v_prev[k], once/sample
```

**Three assembly sites** must each carry the correction with its own `alpha`
(this is what makes BE composition "free" — one nonlinearity, no matrix sync):

1. **Main trap loop** — base `state.a`/`chord_lu`, `alpha = 2·rate·OS`. Add the
   inductor's `L_diff` drift to the chord refactor trigger (like the >50% j_dev
   check) so a fast knee re-factors.
2. **Adaptive sub-step** — base `a_sub`, `alpha = alpha_sub` (already local).
3. **BE fallback** — base BE matrices, `alpha = 1·rate·OS`; history has **no**
   `V_i−V_j` term (BE `A_neg` drops G), so only the `alpha·L0·i_prev →
   alpha·Φ(i_prev)` correction applies.

**Routing / structural prerequisites (Phase 1):**
- Force `use_full_nodal = true` when uncoupled saturating inductors are present
  (today `use_full_nodal` ignores them; a linear+sat-inductor circuit is `m=0`
  and would route to Schur or the `m==0 && !behavioral` linear fast-path).
- Exclude sat-inductor circuits from the `m == 0 && !has_behavioral` direct-LU
  branch (`emit_nodal_process_sample`) — they now need Newton iteration even at
  `m=0`.
- DC-OP is unchanged: an inductor is a DC short (zero self-diagonal, ±1
  incidence) regardless of saturation; sample-0 history uses the DC branch
  current via `v_prev[k]`.
- **Delete the decimated uncoupled block** (`c_work[k][k]` patch, `SAT_UPDATE_
  INTERVAL`, `sat_ind_N_l_eff`) for uncoupled inductors. Coupled/xfmr decimated
  paths stay until Phase 2 deprecation.
- `M`-anchor check: moving the inductor in-loop makes it a counted nonlinear
  element — confirm the emitted `M`/`N` anchors OOMOX pins don't shift
  unexpectedly (`OOMOX_CONTRACT.md:136`).

---

## 4. NR convergence / continuation for the stiff knee

A hard saturation knee is a stiff nonlinearity. Per-sample
`solve_nonlinear` currently runs plain damped Newton with no
continuation — the same limitation behind the G10 / diode-switching
convergence class (STATUS.md:299). `dc_op.rs` **does** have gmin and
source-stepping continuation, but only for the DC operating point, not
per-sample.

Plan: **expect the knee to expose the per-sample continuation gap, and
plan to port the DC-OP gmin/source-stepping into `solve_nonlinear`**
rather than assuming plain Newton converges through the knee. Flux-state
(§3.1) helps here — with λ as the unknown the residual `i = g(λ)` stays
well-scaled through the knee where `L_diff → 0` would otherwise make the
current-state Jacobian catastrophically ill-conditioned. A flux-homotopy
or damped-Newton line search is a candidate alternative. This is likely
the *same substantial fix* already scoped for the G10-astable /
diode-switching class; treat it as shared work, not transformer-specific.

---

## 5. Phased implementation plan

Ordered by dependency. Each phase has a concrete milestone.

**Phase 0 — Decide and audit (no code).**
Decide state formulation (flux-state strongly preferred; current-state
fallback). Confirm nothing besides routing references the deprecated IR:
`routing.rs:179` reads `winding_isats`; audit all consumers of
`SaturatingCoupledInductorIR` / `SaturatingTransformerGroupIR` before
deprecating. *Milestone:* written decision + a clean deprecation list.

**Phase 1 — Fix the UNCOUPLED saturating inductor.**
Move it from lagged/decimated SM-on-trap-matrices onto the full-LU NR
loop as a flux-state (or current-state fallback) nonlinear device (§3).
This alone fixes blockers (2) and (3) for the *already-shipped* feature.
*Milestone / acceptance witness:* **the-kicker** (4 saturating
inductors) stable on silence **and** under `--backward-euler`, plus a
single-inductor XSPICE-core validation (§6).

**Phase 2 — Route coupled transformers through the T-model. [IMPLEMENTED
2026-08-16, commit `ba64ddb`.]**
The gate is ADDITIVE: `(max_l > 1e30 || group_saturating) && max_k > 0.8`
(`mna.rs` ~3439) — any tight-coupled group carrying ISAT routes through the
T-model; non-saturating groups stay byte-identical. Core saturation is
propagated onto `{ref}_mag` (reference/primary winding's ISAT).

**Correction to the original scoping above:** the gate does NOT need to be
limited to *NFB-free* groups. The `1e30` disable was a **DK/Schur**
limitation (algebraic loop needs a reactive-delay per feedback loop); the
full-LU nodal path that saturating transformers force resolves the
ideal-coupling algebraic constraint by direct LU each sample. An impartial
arbiter confirmed the full-LU NFB-through-iron solve matches ngspice to
4–5 significant figures, so Option A (fire for ALL saturating groups,
subsuming the deferred Phase 4) is solver-sound. NFB-free detection was
dropped as unnecessary complexity.

**Two correctness fixes to the T-model realization** (both were latent
while it was disabled):
1. **Exact element values:** `L_leak = (1−k)·L`, `L_mag = k·L_ref` (were
   `(1−k²)·L`, `L_ref`, realizing `k_eff = 1/(2−k²) ≈ 0.98`).
2. **Ideal-transformer sign bug** (`mna.rs` ~3963): the current-injection
   column's *primary* entries were negated relative to the voltage-
   constraint row, when they must be its **transpose**. That gave
   `I_pri = +n·I_sec` (non-power-conserving) and a non-symmetric `[L]` with
   the mutual wrong-signed — a ~5% frequency-growing transfer error. Fixed
   to `I_pri = −n·I_sec`. **The fixed T-model matches the exact
   CoupledInductor `[L]` path and ngspice to +0.011%, flat across k and
   frequency** — 6-nines, same as the exact path.

*Milestone:* blocker (1) dissolved — mag current = net magnetizing current
by construction; SE saturation odd-symmetric (H2=0, H3≈27%); saturating
transformer validated by composition (6-nines linear T-model + the
Phase-1-validated `{ref}_mag` flux device) plus independent ngspice
shared-core twins. Still OPEN: delete the `winding_isats` path (loose-
coupling k≤0.8 saturating groups still fall to it); a PP output-transformer
deck with authored core data.

**Phase 3 — Netlist authoring contract** (decide early, spans Phases
1-2). `ISAT=` on the primary/reference winding (matches shipped inductor
syntax) vs a dedicated `.core Bsat=.. Ae=.. N=..` directive. Default to
the broadest-safety option. Document how the leakage/magnetizing split
and turns vector derive from existing `L` + coupling `k`. *Milestone:*
documented, tested parser path; ISAT authored into marquee decks.

**Phase 4 — DEFERRED: NFB-through-iron.** For the passive EQ / 1073, compute net
MMF `Σ Nᵢ·Iᵢ` directly from the augmented branch currents already
present in the coupled-Y path and apply one shared saturation term,
sidestepping the algebraic loop. Open feasibility question.

**Phase 5 — DEFERRED: hysteresis / loss / advanced cores** (§8).

---

## 6. Validation strategy and acceptance criteria

**ngspice's native `.model IND` core is LINEAR** (constant µ, no Chan
model). Validating against it looks fine at low drive and is silently
wrong in saturation. **Do not use it.**

**Use the XSPICE `core` + `lcouple` code models** (present and working in
the installed ngspice — "XSPICE extensions included"), which are
architecturally the *same shared-core MMF model* melange targets:

```
awind (elec+ elec-) (mag+ mag-) wmodel     ; per winding
.model wmodel lcouple(num_turns=N)
acore mag+ mag- cmodel                       ; ONE shared core
.model cmodel core(mode=1 area=.. length=.. H_array=[...] B_array=[...])
```

- **All windings tie to the SAME `(mag+ mag-)` pair.** Distinct pairs
  re-creates the independent-core bug.
- Ports are bare parenthesized node pairs — **not** `%vd` (throws "Port
  type is invalid").
- **`mode=1` (anhysteretic PWL) only** — melange has no hysteresis.
- Sample `B_array`/`H_array` from melange's *own* law
  `Φ = L0·Isat·tanh(I/Isat)` (`B = Φ/(N·A)`, `H = N·I/Lm`) so both
  engines model the *same* curve.
- Watch the **`lcouple` sign flip** (negative on `INPUT(mmf_out)`): a
  wrong winding-current→MMF sign turns NFB into positive feedback and
  blows up. Respect the per-winding dot convention.

**Correlate secondary V / core flux with INTERP resampling — never
bytes.** XSPICE uses behavioral "fake integration" and force-reduces
`trtol→1` for A-devices; stepping differs. Saturation already breaks
ngspice parity for the shipped uncoupled inductors, so anchor acceptance
to **hardware harmonic behavior**, not SPICE 1:1.

The **Chan model** (Hc/Br/Bs/A/Lm/Lg/N) is the LTspice/industry
reference to *cite*, not the ngspice target.

### Honest acceptance criteria (respecting the B205 no-fabrication lesson)

**No transformer deck in `melange-circuits` currently carries
core-saturation data** — no ISAT/Bsat/Ae/turns/permeability; schemer has
none either. Saturation cannot be "switched on"; it requires **new
per-target authoring and calibration**. Getting ISAT wrong just relocates
distortion to the wrong drive level. **Do not fabricate core parameters
to make a deck saturate** — that is exactly the B205 "model non-existent
inductors" failure. A target ships only when its core data has a real
source (datasheet, measurement, or a documented, labeled estimate) and
the harmonic behavior is validated against the XSPICE twin built from
that same data.

Concrete gates:
1. Single uncoupled inductor: correlation vs XSPICE-core twin (INTERP,
   not bytes) within the saturation-regime tolerance used for shipped
   inductors.
2. the-kicker: bounded on silence AND under `--backward-euler`.
3. One SE + one PP output-transformer deck: harmonic (H2/H3) trend
   matches the authored B-H curve; no `k_eff ≥ 1` / loss of
   positive-definiteness; no Nyquist ring per `nyquist_dbc`.
4. Zero regression on validated NFB-through-iron decks (the passive EQ, 1073):
   they must keep routing away from the T-model.

---

## 7. Effort / risk assessment

- **Phase 1 (uncoupled → NR loop):** medium effort, medium-high risk.
  Touches the augmented-MNA state layout and the nodal codegen NR body.
  Flux-state raises blast radius (DC-OP seeding, `v_prev` indexing,
  OOMOX_CONTRACT). This is the load-bearing phase — it fixes two blockers
  for a *shipping* feature, so regression surface is the existing
  saturating-inductor decks. Golden-audio gate mandatory.
- **Phase 2 (T-model routing):** low-medium *additional* effort once
  Phase 1 lands (no new coupled math), but the `1e30`→gated-enable change
  is high-risk near the passive EQ/1073. Must prove NFB-free gating before merge.
- **Phase 3 (syntax):** low effort, low risk; mostly a contract decision.
- **Continuation for the knee (§4):** high effort, high risk, but shared
  with the G10/diode-switching class — cost is amortized.
- **Deprecating the wrong IR:** low risk once Phase 0 audit confirms only
  routing references it.

Biggest risk is the per-sample continuation gap (§4) surfacing at the
knee and blocking hard-saturation targets. Flux-state mitigates but does
not guarantee convergence. Second risk is fabricating core data to hit a
sound (§6) — a process/discipline risk, not a code one.

---

## 8. Open questions

1. **Hysteresis — needed for audio at all?** Stateless anhysteretic
   (tanh/cosh⁻²/Fröhlich, reuses shipped path) vs Chan simplified
   (Bs/Br/Hc) vs Jiles-Atherton (full, stateful, stiff, hard to fit).
   Uniquorn V2/V3 briefs already defer core hysteresis to a "melange
   Phase 2". Does any *target's* character actually require
   loss/remanence, or is anhysteretic enough?
2. **Netlist authoring contract** (§3, Phase 3): `ISAT=` on the reference
   winding vs `.core Bsat/Ae/N` vs a volt-second/flux limit; how the
   leakage/magnetizing split and turns vector derive from `L` + `k`; how
   schemer would ever supply B-H data.
3. **Flux-state vs current-state:** quantify the numerics benefit vs the
   state-layout / DC-OP / `v_prev` / OOMOX_CONTRACT blast radius.
4. **Per-sample continuation** for the knee: port `dc_op` gmin/source
   stepping into `solve_nonlinear`, or flux-homotopy / line search? Same
   fix as the g10-astable / diode-switching class?
5. **NFB-through-iron (Phase 4):** feasible via net-MMF from coupled-Y
   branch currents, sidestepping the algebraic loop, without enabling the
   T-model?
6. **Rank-1 core Jacobian routing:** can `(2/T)·L_diff·n·nᵀ` go through
   Sherman-Morrison *inside* the NR iteration, or does a saturating
   transformer force full-LU unconditionally?
7. **Out of v1 scope, noted:** multi-limb / multi-flux-path cores (need a
   gyrator-capacitor permeance network, not a scalar Φ); frequency-
   dependent core loss (parallel nonlinear R across the magnetizing
   branch — a separate linear effect from saturation, do not conflate);
   do input/mic transformers (Marinair/Carnhill) even saturate audibly at
   in-app drive?

---

**Bottom line.** Delete the independent-winding path. Make the single
`{ref}_mag` inductor in melange's existing T-model the saturation locus,
gated to NFB-free groups. Fix the *uncoupled* saturating-inductor path
itself by moving it into the full-LU NR loop with flux-state — which
fixes all three blockers at once and carries transformers with it.
Validate anhysteretic (`mode=1`) against the XSPICE core+lcouple twin on
correlation, not bytes, and never on fabricated core data. Defer
hysteresis, NFB-through-iron, and multi-limb cores.
