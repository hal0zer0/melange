# Saturating Inductors and Transformers

Reference for melange's iron-core saturation: the flux law, the shared-core
T-model, authoring (`ISAT=`, `LAIR=`/`CORE=`, datasheet ratings), how it is
solved, what is refused, how it is checked, and what is open. Read it before
touching `SaturatingInductorIR`, `mna::magnetizing_air_floor`,
`mna::isat_from_datasheet`, the T-model decomposition in `mna.rs`, or the
`emit_sat_ind_*` stamps in `nodal_emitter.rs`.

User-facing: [spice-grammar.md](../spice-grammar.md) (the inductor keywords,
what `ISAT` means, where each class of part's numbers come from) and
[limitations.md](../limitations.md) → "Saturating Inductors".

Related: [MNA.md](MNA.md) (augmented MNA, inductor branch currents),
[COMPANION_MODELS.md](COMPANION_MODELS.md) (trap/BE companions),
[NR_SOLVER.md](NR_SOLVER.md), [DC_OP.md](DC_OP.md),
[OPAMP_RAIL_MODES.md](OPAMP_RAIL_MODES.md) (the active-set pinned solve).

---

## 1. What ships

- **Single saturating inductor:** `L1 a b 1 ISAT=10m [LAIR=f | CORE=class]`.
- **Two-winding shared core:** `ISAT=` on either winding of a `K`-coupled pair
  with k > 0.8. The pair is realized as a T-model whose one magnetizing branch
  carries the saturation (§2.2).
- **Anhysteretic and stateless.** No hysteresis, core loss or remanence (§7).
- **Solved as a flux device inside the nodal full-LU Newton loop**, at every
  Newton site, at that site's own integrator coefficient, so trapezoidal and
  backward Euler use the same device (§3).
- **Refused, not approximated:** saturating groups the shared-core model does
  not cover, and authoring that contradicts itself (§5).

---

## 2. The model

### 2.1 Flux law with an air-core floor

```
Φ(i)      = L_mag·Isat·tanh(i/Isat) + L_air·i        L_mag + L_air = L0
L_diff(i) = dΦ/di = L_mag/cosh²(i/Isat) + L_air
```

`L_air = LAIR·L0`. The small-signal inductance is still `L0`; deep in
saturation `L_diff` falls to `L_air`, not to zero.

**Why the floor.** In iron `B = µ0(H + M)` and only `M` saturates, so `dB/dH`
falls to `µ0`: the winding bottoms out at its air-core inductance (analog-EE
review). The pure tanh law has zero final slope — `L_diff` is 6e-8 of `L0` at
9× `Isat` and exactly flat in f64 past about 19× — so the branch equation
degenerates and the current past saturation is set by numerical floors, not
physics. Measured on a saturating RL (1 H, `Isat` 10 mA, 100 Ω, 30 Hz):
`LAIR=0` peaks at 11.83× / 23.94× `Isat` at 10 V / 20 V against a V/R ceiling
of 10× / 20× (a trapezoidal ring, §3.4); with the steel floor it peaks at
10.0000× / 20.0000× with no ring.

**What `ISAT` is.** The tanh scale current. The core's saturation flux is
`λ_sat = L_mag·ISAT` volt-seconds (≈ `B_sat·A_core·N`), and
`L_diff(ISAT) = 0.42·L_mag + L_air`. It is **not** a datasheet "saturation
current" — that is the current at a stated inductance drop, 1.6–3× smaller;
give one with the datasheet forms (§2.4).

**Floor authoring.** `LAIR=<fraction of L0>`, 0 ≤ LAIR < 1 (a measured
saturated-to-unsaturated inductance ratio is best), or
`CORE=gapped|steel|nickel` for a rule-of-thumb class value (1e-3 / 3e-4 /
3e-5). Not both. Neither gives the default 3e-4 (ungapped steel) with a compile
notice; `LAIR=0` is accepted with a notice. Generated code carries
`SAT_IND_N_{L0, LMAG, LAIR, LAIR_SOURCE, ISAT, AUG_ROW}`; `LAIR_SOURCE` records
which reading applied.

### 2.2 Shared core: the T-model

A transformer has one core. Its state is the core flux, driven by the net MMF
`Σ Nᵢ·Iᵢ`. Under load the primary and secondary MMFs nearly cancel (Lenz), so a
winding can carry 100× the magnetizing current while the core is unsaturated.
Saturation keyed on any one winding's current is therefore wrong: it saturates
a loaded core that should stay linear, and misses the magnetizing current that
actually saturates it.

melange realizes a saturating two-winding group as (`mna.rs`, the
ideal-transformer decomposition):

- per winding, a **linear leakage** inductor `(1 − k)·Lᵢ` from the winding's
  node to a new internal node;
- **ideal couplings** between the internal nodes, turns ratio
  `n = √(Lᵢ/L_ref)`;
- exactly **one magnetizing inductor** `{ref}_mag = k·L_ref` on the reference
  winding (the largest `L`, not necessarily the primary), carrying the core's
  `ISAT` and floor.

The ideal couplings reflect load current out of the magnetizing branch, so its
branch current **is** the net magnetizing current, and saturating that one
inductor is shared-core saturation. Leakage is an air path and stays linear.
Self and mutual inductances come out exact for two windings
(`(1 − k)·L₁ + k·L₁ = L₁`, `(1 − k)·L₂ + n²·k·L₁ = L₂`, `M = n·k·L₁ = k√(L₁L₂)`);
checked 2026-08-16 against the exact coupled-inductor `[L]` path and ngspice to
+0.011 %, flat across k and frequency.

**Leakage floor.** Each leakage inductor is floored at `1e-4·Lᵢ`. For
k > 0.9999 the realized leakage is the floor, not `(1 − k)·Lᵢ`, so the realized
coupling is looser than authored (k = 0.99999 realizes about 0.9999) and the
winding self-inductance is high by the difference. No notice is printed. Real
audio iron sits at `1 − k` ≈ 1e-5..1e-4, so this range is reachable by a
faithful deck. Open, §8.

**ISAT referral.** Current refers inversely to turns (`N ∝ √L`), so an `ISAT`
authored on winding `a` becomes `ISAT·√(L_a/L_ref)` on the magnetizing branch.
On a step-up transformer the reference winding is the secondary, and an
unreferred primary `ISAT` would saturate the core `n×` too late.

**Where the T-model is used.** Only for saturating groups (the gate is
`group_saturating && max_k > 0.8`). Non-saturating coupled groups stay on the
exact coupled-inductor `[L]` path; `IDEAL_XFMR_L_THRESHOLD = 1e30` keeps the
T-model off for them. The ideal couplings form algebraic loops that DK and
nodal Schur cannot take, but saturating circuits run on nodal full-LU (§4),
which solves the coupling constraint directly each sample. That includes
negative feedback through the iron: a scratch `ISAT` on the passive EQ's
push-pull primary (2026-08-26) routed through the T-model, auto-BE, and stayed
bounded with no NaN or Newton starvation up to 8 V.

### 2.3 The air floor on a shared core

A winding's air-core self-inductance splits into the fixed air-path leakage,
which the T-model already carries as `(1 − k)·L`, and the air-core mutual part.
The two ways of stating a floor therefore read differently on a shared core
(`mna::magnetizing_air_floor`):

| Declaration | Reading | Magnetizing floor F (× L_ref) |
|---|---|---|
| `CORE=<class>` or none | the core's magnetizing air floor; leakage comes from K | `class` (never refuses) |
| `LAIR=<f>` | the winding's **total** air-core self-inductance (e.g. measured with the core removed) | `f − (1 − k)`; refused if ≤ 0 |

- Declarations on both windings must imply the same F (to 1e-12 relative), or
  the deck is refused.
- The IR expresses F as a fraction of the magnetizing branch (`F/k`).
- For k < 0.9995 a notice says the coupling is looser than real audio iron and
  gives the implied deep-saturation coupling `k_air = F/((1 − k) + F)` (0.029 for
  K = 0.99 with the steel floor).

### 2.4 Datasheet ratings

| Form | Meaning |
|---|---|
| `ISAT=<I> ISAT_DROP=<d>` | the inductance has fallen by the fraction d at current I |
| `ISAT_BASIS=incremental` (default) / `apparent` | the quoted inductance is `dΦ/di` (LCR meter over a DC bias; usual datasheet practice) or `Φ/i` (volt-second measurement) |
| `L_AT_IDC=<L>,<I>` | inductance L at DC bias I ("L at rated DC" on chokes and single-ended output transformers): an incremental drop of `1 − L/L0` at I |

`mna::isat_from_datasheet` converts exactly against the winding's terminal law,
with `x = I/ISAT`:

```
incremental:  L/L0 = (1 − k) + (k − F)·sech²(x) + F
apparent:     L/L0 = (1 − k) + F + (k − F)·tanh(x)/x
```

`k = 1` and `F = LAIR` for a single inductor. The rated drop fixes `x`, and
`ISAT = I/x`. A drop larger than the saturable part `k − F` can reach is
refused. On a shared core the rating is converted against the pair's k and F on
the rated winding, then referred (§2.2); a datasheet form on a core where both
windings carry `ISAT` is refused, because the agreement check compares authored
values. At LAIR 3e-4 the model's `ISAT` is 3.05 / 2.08 / 1.63 × the rated
current at a 10 / 20 / 30 % incremental drop, and 1.71 / 1.13 / 0.84 × apparent.

A maximum-level spec ("+x dBu at y Hz") needs the source impedance before it
implies a current, and mic/line transformer level ratings do not convert to an
`ISAT` at all.

### 2.5 Harmonics: what this law can and cannot produce

The law is point-symmetric. Under symmetric drive with no DC it gives **odd
harmonics only** — and so would a symmetric hysteresis loop (Chan,
Jiles-Atherton). H2 needs **broken symmetry**: net DC magnetizing bias
(single-ended class-A iron, push-pull imbalance), an asymmetric drive from
upstream, or transient remanence. It does not need hysteresis. Hysteresis adds
loss, phase lag and level-/LF-dependent distortion, which is what could justify
it (§8).

With a DC bias established as a **current** (analog-EE review), writing
`φ0 = tanh(Idc/Isat)` and `a` = AC flux / saturation flux:

```
H2/H1 ≈ φ0·a / (2·(1 − φ0²))
H3/H1 ≈ (2 + 6φ0²)·a² / (24·(1 − φ0²)²)
```

H2 is proportional to φ0 and flips sign with `Idc`; H2 = H3 near φ0 ≈ a/6. The
C3 tests gate this (§6).

The passive EQ's H2 comes from sourced push-pull tube `.mismatch`, not from its
iron, which carries no `ISAT`.

---

## 3. Numerical formulation

### 3.1 State variable

The branch current `i_k` of the inductor's augmented row stays the unknown
(decided 2026-08-15). Flux linkage as the state would make `v = dλ/dt` linear
under both integrators, but it changes the augmented state layout, DC-OP
seeding, `v_prev` indexing and the generated-code contract; current-state with
the correct residual/Jacobian split was taken instead.

### 3.2 Stamps

Augmented row `k` between nodes `i, j` (`mna.rs::build_augmented_matrices`):

```
G:  g[i][k] += 1 ;  g[j][k] -= 1        KCL: branch current enters i, exits j
    g[k][i] -= 1 ;  g[k][j] += 1        KVL row k reads (−V_i + V_j)
C:  c[k][k]  = L0
```

Trap `A = G + alpha·C`, `A_neg = alpha·C − G`; BE `A_neg = alpha·C` (no
voltage-history term on inductor rows). The base matrices bake the linear flux
`L0·i`. The saturating inductor is three corrections against the site's
`alpha`, with `i0` the iterate the Jacobian was factored at:

```
Jacobian:      MAT[k][k]   += alpha·(L_diff(i0) − L0)
companion RHS: rhs_work[k] += alpha·(L_diff(i0)·i0 − Φ(i0))
history:       rhs[k]      += alpha·(Φ(i_prev) − L0·i_prev)     once per sample
```

**Footgun:** `L_diff` is the Jacobian entry only. The residual and history use
the flux integral `Φ(i)`, never `L_diff·i` or any `L_eff·i` product.

Numerical guards in the stamps: the `cosh` argument is clamped to ±40, and
`L_diff` is floored at `1e-6·L0` (inactive unless `LAIR` < 1e-6).

### 3.3 Newton sites and one convergence definition

A saturating inductor makes the circuit nonlinear even at M = 0, so it forces
the full-LU path and is excluded from the M = 0 direct-LU fast path. The stamps
go in at every site that can commit a sample:

1. the main trapezoidal (or BE) loop, `alpha = 2·fs·OS` (or `fs·OS`);
2. the adaptive sub-step, `alpha_sub`;
3. the backward-Euler fallback, `alpha = fs·OS`, history without the
   `V_i − V_j` term;
4. the op-amp active-set pinned Newton. Its start takes `i_L` from `v_prev`: an
   unpinned iterate 2-cycles on tanh. A pinned solve that fails is counted in
   `diag_nr_unconverged_commit_count`.

Each site checks the flux row with the same residual
(`emit_sat_ind_row_residual`): the row's equation evaluated at the accepted
iterate, stop at `max(1e-5·den, 64·eps·max(|alpha·Φ|, |rhs[k]|))`, where `den`
is the row's per-sample increment (`alpha·ΔΦ`, the volts across the winding),
not the flux. The tolerance is load-bearing: under a DC bias Newton's remainder
is one-signed and integrates with the L/R time constant. At `1e-3·den` a 5 mA
biased core drifted −1.6e-5 A over 2 s (0.7 % on H1 in C3); at `1e-5` the drift
is about 1e-10 A, for about one more iteration per sample. The function's doc
comment records why the cheaper Φ-vs-Φ drift test can never fire.

**Step limit.** Each site also limits the Newton step on the flux row. From
deep saturation (`L_diff ≈ L_air`) a step that crosses the knee is amps long and
lands deep on the far side, where the slope is flat again, and Newton
2-cycles. A step that crosses `|i| = Isat` (or changes sign) and lands more than
`Isat` past the knee is scaled, through the shared step fraction as pnjlim is,
to land at `2·Isat`; steps within one regime pass unscaled. Measured: a
saturating RL under a ±20 V 100 Hz square went from 398 MAX_ITER samples/s
(each recovered by a sub-step, 3.3 % off the circuit's own 1× trapezoidal
solution) to 0, matching an independent recurrence to 1.2e-6; wherever the
unlimited build converged, the output is bit-identical. A blanket
`|Δi| ≤ 2·Isat + 0.5·|i|` was measured and rejected (it throttles converging
steps and moves converged outputs). The limit is not in the Armijo merit
(node KCL, in amps); mixing flux rows into it needs its own scaling decision.

**DC operating point.** An inductor is a DC short; the DC branch current seeds
`v_prev[k]`. A circuit whose DC solution has all node voltages at zero but a
nonzero inductor current (a current-biased grounded inductor) still has a DC
operating point; the nodal "has DC OP" test reads the augmented rows too.

### 3.4 Integrator

Trapezoidal by default. Auto-BE, breakpoint-BE and the runtime BE-latch are
armed on saturating circuits, M = 0 ones included, because the flux device is
stamped at every site at that site's `alpha`. A forced latch matches a
`--backward-euler` build of the same circuit to 2.1e-10 relative (open
shared-core transformer, 5 V) and 3.1e-11 / 1.9e-11 (choke-loaded MOSFET stage,
3 V / 5 V).

**Deep saturation.** Trapezoidal integration is A-stable, not L-stable: as
`L_diff` collapses, a branch's trapezoidal factor tends to −1 and a Nyquist ring
can survive. With an air-core floor the measured cases do not ring (saturating
RL at 10, 20 and 100 V; choke-loaded common-source stage at 1–30 V), and the
latch does not fire. With `LAIR=0` the RL rings (current 20 % over the V/R
ceiling at 20× `Isat`, not cured by 4× oversampling) and the latch catches it.
The latch is sticky, so a transient ring commits that instance to BE for the
rest of the stream; measured with `LAIR=0`: H1 −1.8e-4 (RL at 10 V), output
−0.28 % (choke-loaded stage at 5 V).

**Start-up ring on an open transformer.** On the golden `sat-core-open/step`
render the latch fires once: a 6 mV sample-to-sample alternation around
−0.236 V after the 5 V step. That ring is not saturation. Measured 2026-09-28:
the same deck with `ISAT=100` (the core never leaves its linear region) latches
identically, and with a 600 Ω load instead of 1 MΩ it does not latch. It is
consistent with the open secondary's stiff linear mode — leakage
`(1 − k)·L` = 10 mH into 1 MΩ, a 10 ns time constant against a 21 µs sample,
trapezoidal factor ≈ −0.998.

**Knee at a short L/R (railing op-amp, 1×).** Just past the knee the tanh slope,
not the floor, sets `L_diff`. A railing op-amp driving a gapped choke at about
2.7× `Isat`, active-set rail mode: at 1× the inductor current overshoots by
5–13 % over ngspice while the output fundamental stays within 0.21 %; at 4× the
current is within 1.3 % and H1 within 0.1 %. A per-sample trace puts the
overshoot where the core crosses its knee within one sample with the full rail
across it, 2–3 samples after the pin, not at the pin change. Measured and
rejected: one backward-Euler sample at each pin-state change (inductor current
still −1.6..+13 %); the recovery sub-step triggered on the `L_diff` collapse
ratio (fired on the unpinned iterate, and the full-step pin re-solve discarded
the sub-steps; the ratio is not a sound trigger — a saturating RL at 20 V
collapses harder and does not ring); a per-element trapezoidal-factor guard
(every threshold that removes the ring over-damps: inductor current −4 %, H1
+6 % at 1 V). Compile prints a notice when a clamped op-amp in an active-set
mode and a saturating inductor meet below 4× oversampling. Parked, with its
reopen trigger, in STATUS.md → Pending Work.

---

## 4. Routing

- **Nodal full-LU, unconditionally.** The flux device lives on an augmented
  row inside the full-LU Newton loop.
- `--solver dk` is **refused**: DK bakes `S = A⁻¹` and has no per-sample Newton
  on the augmented row, so it would run the inductor linear.
- `--nodal-subpath schur` is **refused** for the same reason: the Schur
  reduction does not iterate on augmented rows.
- Provenance: the route reason is "saturating inductors are flux devices
  solved in the nodal full-LU Newton loop"; the full-LU trigger is
  `saturating-inductor`.

---

## 5. Refusals and notices

Refused, with a message naming the elements:

| Case | Why |
|---|---|
| Saturating coupled group with largest k ≤ 0.8 | A closed iron core has k > 0.99. k ≤ 0.8 is either no shared core (give each inductor its own `ISAT` and drop the `K`) or a deliberate leakage path whose flux itself saturates (ballast, neon and welding transformers), which is out of scope. Permanent. |
| Saturating group with 3 or more windings | The two-winding T-model does not generalize by averaging couplings (4 dB at 20 Hz in a 3-winding test). §8. |
| Two windings of one core with different referred `ISAT` | One core has one saturation current. |
| Two windings implying different magnetizing floors | One core has one floor. |
| Authored `LAIR` ≤ `1 − k` on a shared core | The deck's own k already puts that much in leakage. |
| Datasheet form on a core whose two windings both carry `ISAT` | The agreement check compares authored values. |
| Datasheet drop the saturable part cannot reach | No `ISAT` produces it. |
| Datasheet drop below 1e-6, or a converted/referred `ISAT` that is not a finite positive current | Not a rating; would emit `inf`/0. |
| `.switch` naming a saturating inductor, or any winding `K`-connected to one | The flux law is fixed at compile time and a winding has no branch row after the T-model split; a switched value solved a different device (1 H → 0.1 H: 740 V from 10 mV) or stamped ΔL as a node capacitance. |
| `LAIR=` with `CORE=`; `L_AT_IDC=` with `ISAT=`/`ISAT_DROP=`; `ISAT_BASIS=` without `ISAT_DROP=`; out-of-range values | Strict parsing. |

Compile notices: default floor (with the reading used on a shared core);
`LAIR=0`; shared-core k < 0.9995; railing op-amp into a saturating inductor
below 4× oversampling.

---

## 6. Validation

**Tests** (`crates/melange-solver/tests/`):

- `saturation_knee_regression_tests.rs` — the knee, driven at 30 Hz.
  - **C1** saturating RL against a scalar trapezoidal recurrence of
    `V − R·i = dΦ/dt` at 1× (checks the implementation) and 1024× (≈continuous;
    checks the physics). A reference that shares melange's discretization
    validates the implementation only, which is why the 1024× gate stays.
    Mutants (broken flux device, zero floor) must fail;
    `c1_jacobian_deletion_is_caught_at_every_newton_site` removes the Jacobian
    stamp at each site in turn. Deep saturation settles on V/R; the `LAIR=0`
    ring is caught by the latch; a forced latch matches the BE build.
  - **C2** shared-core discriminator: loaded, H3/H1 ≈ 0 (per-winding saturation
    gave 0.16); open, the magnetizing current saturates the core (H3/H1 0.50460,
    gate 1e-4).
  - **C3** DC bias: H2 against the exact flux-drive values to 0.5 dB and a 256×
    recurrence; H2 flips with the bias sign and crosses H3 near a/6.
- `saturating_group_refusal_tests.rs` — every refusal in §5 on the MNA side,
  plus covered groups still building.
- `isat_datasheet_tests.rs` — the converted `ISAT` reproduces the rated drop to
  1e-12 (single and shared cores, both bases); the published factors;
  `L_AT_IDC` equals the matching `ISAT_DROP`; parse strictness.
- `opamp_railing_regression_tests.rs` — the railing op-amp into a choke,
  against an ngspice twin built from the same law: `L_air` (0.1 mH) in series
  with a flux integrator and a behavioral current
  `I = Isat·atanh(Φ/(L_mag·Isat))`.

**Golden corpus.** Three coverage-only decks in
`tools/golden-harness/decks/` (`sat-knee-rl`, `sat-core-loaded`,
`sat-core-open`) at a 5 V manifest level; before them no golden program took
any inductor past i/Isat = 0.37. They detect change and validate nothing.

**ngspice.** Its native inductor core is linear: agreement at low drive says
nothing about saturation, so do not validate against it. An XSPICE
`core` + `lcouple` twin is the same shared-core MMF architecture and is useful
as an integrator check only:

```
awind (elec+ elec-) (mag+ mag-) wmodel     ; per winding
.model wmodel lcouple(num_turns=N)
acore mag+ mag- cmodel                       ; ONE shared core
.model cmodel core(mode=1 area=.. length=.. H_array=[...] B_array=[...])
```

- All windings tie to the **same** `(mag+ mag-)` pair; separate pairs rebuild
  the independent-core error.
- Ports are bare parenthesized node pairs, not `%vd`.
- `mode=1` (anhysteretic PWL) only. Sample `B_array`/`H_array` from melange's
  own law (floor included) so both engines model the same curve.
- `lcouple` negates `INPUT(mmf_out)`: a wrong winding-current→MMF sign turns
  feedback positive. Respect each winding's dot.
- XSPICE A-devices force `trtol→1`, so correlate with INTERP resampling, never
  bytes.

**Core data.** Do not fabricate core parameters to make a deck saturate:
getting `ISAT` wrong only moves the distortion to the wrong drive level. A
target's core data needs a source (datasheet, measurement, or a labeled
estimate). Corpus `ISAT` values written before the datasheet forms existed may
be datasheet ratings read as tanh scale currents, which saturates the core
1.6–3× too early; restate them with `ISAT_DROP=` or `L_AT_IDC=`.

---

## 7. Not modeled

- **Hysteresis, core loss, remanence, minor loops.** The law is stateless.
- **Frequency-dependent core loss** (a parallel nonlinear R across the
  magnetizing branch) is a separate effect from saturation.
- **Multi-limb / multi-flux-path cores** need a permeance network, not a scalar
  flux.
- **Saturating leakage flux** (ballast, neon, welding transformers): refused
  (§5).

---

## 8. Open

1. **Three-winding shared cores.** Physics (analog-EE review): one saturating
   magnetizing branch with fixed linear leakages is correct. The star form is
   exact for W = 3: per-winding `cᵢ` with `k_ij = cᵢ·c_j`, leakage
   `(1 − cᵢ²)·Lᵢ`, magnetizing `c_ref²·L_ref`. Refuse when some `cᵢ ≥ 1` (a
   sandwiched winding gives a negative leakage). W ≥ 4 is not a star in
   general, so its refusal stays. Not built; three windings are refused today.
2. **Leakage floor above k = 0.9999** (§2.2): the `1e-4·Lᵢ` floor silently
   loosens the realized coupling of tight iron.
3. **Knee re-solve for a railing op-amp at 1×** (§3.4): parked.
4. **A second-order L-stable integrator** (BDF2) for magnetics: held pending
   physics.
5. **Hysteresis** — justified, if any target needs it, by loss, phase lag and
   LF/level-dependent distortion, not by H2 (§2.5). Chan (Hc/Br/Bs) is the
   reference to cite.
