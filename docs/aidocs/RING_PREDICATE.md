# The Ring Predicate — When a Build Is Promoted to Backward Euler

## Purpose

The trapezoidal rule is A-stable but not L-stable. It maps a stiff mode (a
continuous pole far above the audio band) to a discrete pole just inside
`z = −1`, which rings at the Nyquist rate and decays slowly. Backward Euler
maps the same mode near `z = 0`, at the price of first-order accuracy in the
audio band. A default build is promoted from trapezoidal to backward Euler
only when that ring is a real cost. This doc is the reference for the rule
that decides it: `crates/melange-solver/src/codegen/ring.rs`.

## The rule

```
promote to BE  iff  growth: rho(P) > 1.002, and backward Euler removes it
                         (rho of the BE propagator at the same point <= 1 + 1e-6)
                or  some pole z of P with
                      Re z < 0                                  (Nyquist side)
                      |z|^(0.01·fs) >= 1e-3                     (still above −60 dB after 10 ms: it lasts)
                      |residue from the input| >= 1e-3 · H_pink        (it starts loud: −60 dB)
                      |residue from the input| >  E_BE                 (louder than BE's own damage)
                        — the last condition only where E_BE <= 0.1 (−20 dB)
```

- `E_BE` is backward Euler's worst in-band change of the response, in the
  same currency: `max |H_BE(e^{jωT}) − H(jω)| / H_pink` over the passband
  grid below `min(20 kHz, 0.45·fs)`, on the same linearised system
  ("Backward Euler's cost", below). A ring promotes only when trapezoidal's
  artefact is louder than the damage backward Euler would do.
- The comparison holds only where backward Euler's change is a genuine
  perturbation of the small-signal response, `E_BE <= 0.1` (−20 dB,
  `BE_COMPARISON_VALID_REL`, stated, not derived). A regenerative circuit's
  linearisation at its DC operating point has near-marginal in-band poles:
  any `s`-mapping then moves the response by more than the passband itself
  (a PNP astable: `E_BE` +13 dB, `E_trap` +17.6 dB; a BJT Schmitt trigger at
  192 kHz: `E_BE` +1.5 dB), which says nothing about its switching edges.
  There the ring threshold alone decides, and the reason says so ("the
  small-signal comparison is not valid here"). Kept trapezoidal by the
  comparison, the astable blew up and the Schmitt thresholds were 0.05 V
  off ngspice's.

- `P` is the trapezoidal **charge propagator** (below) linearised at the DC
  operating point, at the internal (oversampled) rate.
- The residue is the modal residue of that pole alone, `(c·r)(l·B)/(l·r)`
  (right vector `r`, left vector `l`, input column `B`, output row `c`): the
  initial amplitude of the spurious ring term `residue·zⁿ` in the impulse
  response. Not the total response at `n = 0`.
- `H_pink` is the passband gain: the pink-weighted RMS gain of the
  linearised continuous network, `sqrt(mean |H|²)` over a 481-point
  log-spaced 20 Hz–20 kHz grid — the output RMS for a unit pink-spectrum
  input, the level a program comes out at. A narrow resonance counts by its
  width, not its height; a band-limited deck is referred to its own band.
  (The gain at one frequency sits in the stopband of a band-limited deck; the
  largest gain lets a narrow resonance set the scale.) The program level
  cancels: ring and passband scale alike.
- Multiple inputs and outputs: the largest ratio over every pair.
- Both −60 dB constants are stated in advance, not fitted
  (`RING_PERSISTENCE_FACTOR`, `RING_PERSISTENCE_SECONDS`, `RING_RESIDUE_REL`).
- A growth that backward Euler keeps is a real growing pole (an oscillator
  or latch on an unstable bias by design); the build stays trapezoidal,
  which reproduces the growth the circuit's nonlinearity then bounds.

Only a **default** build is decided. `--force-trap`, `--backward-euler`,
`.integrator trap|be` and behavioral-source forcing pin the integrator and
nothing is evaluated.

### What the rule leaves to the runtime latch

The static rule sees the linear, rest-state, input-driven path. Everything
signal-dependent — device current swings, edges, grid conduction — is not
knowable at compile time. It belongs to the runtime BE-latch
(`STATUS.md`, "Runtime BE-latch"), which measures actual output alternation.
A ring driven by the solver's own accepted Newton residual is a solver
defect, fixed at the residual, and is not a routing criterion.

## Backward Euler's cost

Backward Euler is first order: its in-band error is `O(ωT/2)`, about 1.6 %
in `s` at 1 kHz at 192 kHz, and it shows at full size near any in-band pole.
Under the ring rule alone that produced a perverse route: raising the
sample rate lifts a stiff mode's residue about 6 dB per octave, so an output
transformer (k = 0.999, 100 kΩ into a 50 H primary) crossed −60 dB at
192 kHz and was switched to an integrator 28× less accurate in band (0.463 %
against 0.0165 % vs ngspice at 1 kHz; the error is BE's at the primary's
318 Hz corner, reproduced on a bare RL: −0.470 % computed, 0.459 %
measured).

`ring::inband_error` evaluates, on the linearised `(G_l, C_l)`, input
column and output row the ring rule uses, over the 481-point log grid from
20 Hz to `min(20 kHz, 0.45·fs)`:

```
H(jω)                      the continuous response
H(s_BE),   s_BE   = (1 − z⁻¹)/T              at z = e^{jωT}
H(s_trap), s_trap = (2/T)(1 − z⁻¹)/(1 + z⁻¹)
E_rule = max |H(s_rule) − H(jω)| / H_pink    (and the frequency where it is)
```

It is exact for the small-signal path (both rules are exactly those
substitutions on a linear system) and costs one complex `N`-solve per point.
It runs only where the decision or the notice needs it (a ring at or above
−60 dB, or growth). Both numbers go into `integration_reason` (so the
provenance) and into the backward-Euler notice, which states BE's worst
in-band change and the trapezoidal rule's.

## The operator

With `x` the MNA unknowns and `q` the carried charge derivative
(`COMPANION_MODELS.md`, "Charge (Companion) Form"):

```
x' = S·(H·x + q + b·u')          S = (G_l + alpha·C_l)^-1,  H = alpha·C_l (algebraic rows zeroed)
q' = H·(x' − x) − q              alpha = 2·fs_internal
```

`G_l = G − N_i·J·N_v`, with the device Jacobian `J` evaluated at the DC OP
(`dc_op::evaluate_devices_with_nodes`). A saturating inductor's branch row
carries `L_diff(i_dc)` in `C_l` (`SATURATING_TRANSFORMERS.md` §3.2).

`q` lives in `range(H)`. With `U` an orthonormal basis of it, the reduced
propagator on `(x, ξ)`, `q = U·ξ`, is

```
P = [ S·H              S·U           ]      B = [ S·b      ]      c = [ e_out, 0 ]
    [ Uᵀ·H·(S·H − I)   Uᵀ·H·S·U − I  ]          [ Uᵀ·H·S·b ]
```

Algebraic directions go to `z = 0`. The whole-system operator
`S·(alpha·C − G)` put them at `z = −1`, which made it the wrong operator to
judge ringing on: most of the old promotions were that algebraic family. An
index-2 structure (an inductor-only cutset) keeps an exact `−1` in `P`, and
the rule decides it like any other pole, by its residue.

### The charge basis

`range(H)` is found structurally, not by a global rank threshold on `H`:

1. Keep the rows of `H` that carry charge.
2. Find their exact dependencies (a capacitor between two nodes that carry no
   other charge makes the two rows sum to zero) by SVD (one-sided Jacobi) of
   the row- and column-equilibrated charge rows, where rank does not depend
   on scale. Threshold `CHARGE_DEPENDENCY_TOL = 1e-12` of the largest
   singular value. Measured on the 42-deck corpus: dependencies at
   ≤ 1.4e-16, real charge directions at ≥ 3.5e-8.
3. The basis is the exact unit vectors of the rows no dependency touches,
   plus an orthonormal complement of the dependencies on the rows they do.

A threshold on the graded `H` itself is wrong: charge directions span 1e-13 of
the largest on the corpus (a picofarad next to a henry), and a small
capacitor is exactly a stiff near-`−1` mode. A 1e-13 relative SVD threshold
drops 7 real directions on passive-eq1a and 1 on steve-1073. A numerical
basis of the graded `H` also mixes directions of different scale: a
column-equilibrated QR basis moved noyce-amp-at-idle's ring pole by 1e-3.

## Numerics

`crates/melange-solver/src/eigen.rs`, no linear-algebra dependency:

- all eigenvalues: the eigenvalues the sparsity pattern isolates exactly (a
  row or column with no off-diagonal entries, `balanc`'s permutation stage)
  are taken out first; the rest go through balancing by powers of 2
  (EISPACK `balanc`), reduction to Hessenberg form (`elmhes`) and Francis
  double-shift QR (`hqr`). The isolation is needed, not an optimisation: the
  algebraic directions are zero columns, and the defective zero eigenvalue
  they form made QR cycle without converging, at any sweep budget, on a
  57×57 backward-Euler propagator (a divider-chain deck, 30 zero columns,
  48 and 44.1 kHz). Whether QR got through depended on the last bit of the
  entries.
- vectors, only for the lasting Nyquist-side poles: inverse iteration in
  complex arithmetic on the balanced matrix, mapped back
  (`r = D·r_b`, `l = D⁻¹·l_b`).

A QR iteration that does not converge, or a singular `A`, fails the build
loud: a routing decision never silently defaults.

### Gate (`tests/eigen_reference_tests.rs`)

Against numpy/scipy `eig` (LAPACK `dgeev`, left and right vectors), references
written by `tests/data/gen_ring_reference.py`:

- every eigenvalue within `1e-10·max(|λ|, 1e-2·ρ) + n·κ·ε·‖P‖_F`, with
  `κ = 1/|lᴴ·r|` the eigenvalue's condition number. The second term is the
  first-order bound of any backward-stable method; a near-defective pair
  (κ ≈ 2e6) is not computable to 1e-10 relative by LAPACK either;
- every Nyquist-side residue within 0.1 dB above the floating-point floor.

Committed: 15 synthetic matrices (dense to n = 120, three poles within 1e-7
of each other near −1, near-defective pairs, complex pairs near −1,
similarities scaled over 1e12, a propagator-like spectrum), the 5 in-repo
golden decks, and the two stalled backward-Euler propagators
(`tests/data/eigen_regression.json`, entries stored as exact round-trip
strings because the stall depends on their last bit). On demand (`MELANGE_RING_DIR`, `--ignored`): the 42 corpus
decks. Measured 2026-09-29: all pass. Relative error on eigenvalues with
`|z| ≥ 0.9` ≤ 4.3e-10, except a close pair near +0.9998 on warpony (1.5e-7,
κ = 8e5, never read by the ring rule). The 4.3e-10 is sat-core-open's
promoting pole, and there it is the LAPACK reference that is off: 50-digit
arithmetic gives −0.9961868486952391, `eigen.rs` −0.9961868486952, numpy
−0.9961868482626 (its `inv(A)`, condition 5e8, is the less accurate of the two).

## Corpus verdict (2026-09-30, circuits corpus + golden decks, internal rate)

Decks whose loudest lasting pole is at or above −60 dB (every other deck
stays trapezoidal on the threshold alone):

| Deck | Input residue (rel H_pink) | BE's in-band change | Verdict |
|---|---|---|---|
| axe-15 | −36.5 dB | −35.9 dB | trap (was BE) |
| noyce-amp-at-idle | −39.3 dB | −37.8 dB | trap (was BE) |
| twill-deluxe | −43.2 dB | −37.0 dB | trap (was BE) |
| kt88-pp-stage | −44.3 dB | −42.4 dB | trap (was BE) |
| gold-press-overdrive | −43.7 dB | −46.6 dB | BE |
| sat-core-open | −48.2 dB | −49.8 dB | BE |

Against ngspice driven by the analytic 1 kHz sine at 48 kHz, BE → trap:
twill-deluxe 0.76 % → 0.16 %; kt88-pp-stage 0.18 % → 0.19 %;
noyce-amp-at-idle 2.32 % → 2.43 % (its 1× error is the input
discretization, either way); axe-15 has no reference (ngspice aborts). A
1 kHz tone misses where BE's worst change sits (up to 20 kHz), so these
tone numbers understate what the route change fixes; the golden render of
noyce-amp-at-idle moved +0.06 dB across the band (BE's damping removed).

Earlier table (2026-09-29, the threshold alone): noyce-amp-at-idle −39.3 dB
BE; sat-core-open −48.2 dB BE; noyce-transformer-triode −77.7 dB,
wurli-power-amp −105.0 dB, passive-eq1a −119.2 dB, steve-1073-preamp
−120.6 dB, noyce-tape-head −168.0 dB, basic-bitch and sad-bastard
< −190 dB, champ-5f1 / sat-core-loaded / noyce-smps-ripple (index-2)
< −200 dB: trap.

No deck grows at its DC operating point (largest ρ = 1 to rounding: the
index-2 and DC modes). The three candidate passband gains give the same
verdicts on the corpus. `H_pink` against the gain at 1 kHz / the largest
gain: noyce-transformer-triode +15.1 / −9.5 dB (a response rising to the
20 kHz edge), gold-press-riaa +10.1 / −6.8 dB (the phono bass boost),
champ-5f1 +9.2 / −10.1 dB, noyce-boiler-room +40.0 / −9.1 dB (no lasting
mode); every other deck within 7 dB of both.

**noyce-transformer-triode.** A single input event rings at −78 dB of the
passband gain (−63 dB of the 1 kHz gain) at fs/2 and decays with τ 0.7 s.
Phase-coherent even-period clicks accumulate it by `1/(1 − |z|^P)`: a 10 Hz
impulse train (P = 4800 at 48 kHz) reaches −46 dB of the 1 kHz gain after
100 impulses (−61 dB of the passband gain), and the linearised model
reproduces the code's per-impulse growth to 0.1 dB (−62.9, −57.5, −54.6,
−52.7, −51.3 … −46.2 of the 1 kHz gain). The runtime latch's closest approach
on that train is −60.6 dB of the program reference: it does not engage, by
0.6 dB. It stays trapezoidal: forced trapezoidal is 45× more
accurate in band on this deck (sine error 0.022 of the BE build's), the tone
sits at 24 kHz ± 0.5 Hz, and with any oversampling it moves to the internal
Nyquist where the decimator removes it. A static accumulation term would
have to assume a click period the compiler cannot know. The build's
`integration_reason` states this. It is evidence for an L-stable integrator
(BDF2 / TR-BDF2), `STATUS.md` → Pending Work.

**Margins.** For a new deck whose loudest lasting pole lands within ~5 dB of
−60 dB, measure it: the hostile program's 10–50 ms tails, dB relative to
impulse amplitude × passband gain, on the forced-trapezoidal build.

## The runtime latch uses the same rule

The runtime BE-latch (`STATUS.md`, "Runtime BE-latch") engages on an
alternating mode that dominates the output. Dominance alone is relative to
the instantaneous output, and in a quiet tail anything dominates, so it used
to override this rule at the first quiet moment after a transient (it held
noyce-transformer-triode and wurli-power-amp on BE after their −63 dB and
−74..−81 dB rings). Its floor is now the larger of the node tolerance and
`max(ε_ring, E_BE) × ref`: the ring threshold, and backward Euler's own
in-band damage, so a ring must be louder than both before backward Euler is
chosen, as at compile time. `E_BE` is the predicate's own figure at the
compiled rate, emitted as `BE_LATCH_BE_COST_REL`; where the comparison does
not hold (`E_BE` above −20 dB, a near-marginal linearisation) it is 0 and
the threshold alone decides, as it does at compile time. Without it the
latch overrode a compile-time choice at the first impulse: twill-deluxe is
kept trapezoidal because its ring (−43.2 dB) is quieter than backward
Euler's in-band change (−37.0 dB), and the runtime latch then engaged on the
−52 dB ring after one impulse at −40 dB and held backward Euler for good.
It no longer does. kt88-pp-stage (`E_BE` −42.4 dB) no longer engages after a
noise burst either: its ring there is about as loud as backward Euler's
damage, and at compile time the same comparison keeps it trapezoidal.
The reference:

```
ref_n = min(in_n, env_n)
in_n  = max(H_pink · |u_n|,              d · in_{n-1})
env_n = max(|y_n − y_dc|,                d · env_{n-1})      (y_dc: the output's operating point)
d     = max over BE_LATCH_RING_POLES of the Nyquist-side |z(fs)|, floored at 1e-3^(1/(0.01·fs))
z(fs) = (1 + λ/(2fs)) / (1 − λ/(2fs))        (the trapezoidal map, at the running internal rate)
```

- The reference is the program the output actually carries, bounded on both
  sides. On this rule's scale that is passband gain × input amplitude, and an
  output-peak reference alone sits far above it on transformer-triode (its
  impulse-response peak is 0.6 against a 1 kHz gain of 0.053), where the min
  picks the input side. A circuit that clips never delivers the linear
  extrapolation (steve-1073-preamp: 146 V of H_pink·|u| against a 15 V
  output), and there the min picks the output side, so a ring is judged
  against the program it actually rides on. The ring itself sits in the
  output envelope, but where it matters it is small against the program. A
  clipper in front of a stiff node latches on the same click-train ring at
  0.3 V and at 5 V of drive (`be_latch_entry_tests.rs`); referenced to
  H_pink·|u| alone the 5 V case never latched. On the 60 s hostile program
  over the 31 latch-carrying corpus and golden builds its only effect is on
  kt88-pp-stage at 1 V after a noise burst: a trapezoidal ring of about
  −40 dB of the program (80.7 mV at Nyquist against backward Euler's
  0.34 mV) that the linear reference hid. The backward-Euler comparison above
  then keeps that build trapezoidal, as the compile-time rule does, since the
  ring is about as loud as backward Euler's own in-band damage.
- Its memory is the slowest ring the circuit can carry: a ring cannot outlive
  the reference of the program that excited it. A memory at the ε_ring rate
  (−60 dB in 10 ms) is always outlived by a lasting ring, by definition.
- `λ` are the continuous-time poles of the linearised circuit that can ring
  at fs/2 at host rates down to a quarter of the compiled rate (stable,
  non-algebraic, `|λ| > fs/2`), emitted as `BE_LATCH_RING_POLES`;
  `set_sample_rate` recomputes `d`. At a rate below the compiled one a stiff
  ring decays more slowly (as `fs²`), so a `d` frozen at the compiled rate
  would be outlived.
- An index-2 pole (exactly `z = −1`) rings forever: `BE_LATCH_RING_HOLD`, and
  the reference is held. A genuine ring excited in a quiet passage long
  after a loud one is then judged against the loud one and under-latches —
  conservative, toward fewer BE samples, and those modes carry about zero
  input residue. The real fix is the netlist question of whether such decks
  should carry the physical parasitic that regularises index-2 (winding
  capacitance, core loss), `STATUS.md` → Pending Work.
- Ring amplitude is `sqrt(pow)`: an alternation ±A has power A².

Measured 2026-09-29, 60 s hostile program at the golden level and −40 dB, on
the default builds of the 15 decks returned to trapezoidal: no latch
engages (four of them are DK builds, which carry no latch; their absolute
10–50 ms ring levels on that program are below −168 dB of the passband). The single-event ring does not engage it; phase-coherent clicks
whose accumulated ring passes −60 dB of the program do
(`tests/be_latch_entry_tests.rs`).

## A ring under a loud program: the trial latch (evaluated 2026-09-30, rejected)

**The gap.** The latch's entry test is the lag-1 ratio of the output. Under
a loud program that ratio is the power-weighted mean of the program's and
the ring's factors, so a ring at −22 dB of a 15 V program still reads
about +0.98 and never enters. steve-1073-preamp unpinned (trapezoidal) is
the witness: every onset from quiet excites a Nyquist-rate ring on the flat
tops of its clipped output (1.0 V against backward Euler's 86 mV at 48 kHz;
2.1 V against 0.14 mV at 96 kHz, where it persists and costs 11.4 % RMS
against a converged reference, against 1.31 % pinned). The latch never
engages.

**What was evaluated.** Offline, on latch-free trapezoidal renders (the
generated code with the latch flag owned by a driver):

- A Nyquist estimator: the output demodulated by (−1)ⁿ, first-differenced
  and low-passed to ±1 kHz of fs/2, its power over 0.5 ms; excused by the
  input's own Nyquist content times the circuit's gain there; above the
  program floor; held for 2 ms. (A plain second difference is a high-pass,
  not a Nyquist detector: a 10 kHz tone at 48 kHz gives 37 % of its
  amplitude.)
- No estimator of the output can separate a trapezoidal ring from content
  the circuit forces at fs/2: a hard-clipping common-emitter stage at 1 and
  5 kHz carries 22 and 280 mV at Nyquist that backward Euler reproduces to
  0.1 mV. So the discriminator was a mechanism test, a **trial latch**: on
  a trip, run backward Euler for 3 ms; if the estimate falls to a third or
  less, the content was trapezoidal's and the latch holds; otherwise
  release to trapezoidal and do not re-trial for 1 s.

| round | change | steve 48k | steve 96k | clippers (CE, diode, sweeps) | glitch on forced content |
|---|---|---|---|---|---|
| 1 | trial latch | released (0.19 → 0.14 V: edges dominate the band) | latched at +10 ms; then equal to pinned BE to 0.004 % | CE trips and releases, 0.3 % of samples on BE | −14 / −10 dB under the floor (CE) |
| 2 | + edge gate (10 % of the program reference) | released | latched | no trips | — |
| 3 | + program reference = min(H_pink·\|u\|, output envelope) | latched at +5 ms | latched at +5 ms | no trips | **wurli-power-amp: 0.28 V ≈ −39 dB of the program** |

Round 3 separated every case, including the in-slow witnesses above. It is
rejected on the glitch: a trip inside wurli-power-amp's large-signal
start-up transient (program at full level from t = 0) put 3 ms of backward
Euler into the transient, which bent the trajectory by 0.28 V peak on a
25 V output (a smooth offset, not a ring), decaying over ~50 ms, although
the trial itself correctly released the content as forced. A trial that is
audible on a real deck is itself an artefact.

**Reopen condition.** A trial-scheduling rule *derived* from the circuit
(for example from its own settling time), not a constant chosen after
seeing this failure, that keeps trials out of large-signal transients.

**Limitation that stands.** On edge-dominated decks at 1x, the runtime
latch cannot separate a flat-top trapezoidal ring from the clipping edges'
own Nyquist-band content. Such a deck needs a static pin
(`.integrator be`); steve-1073-preamp keeps its pin for this reason.

## Limitation: the verdict is taken at the compiled rate

The trap-versus-BE verdict uses the compiled internal rate. A host rate
above it makes stiff rings decay faster and residues smaller (the verdict
is conservative there); below it they decay more slowly and residues grow
(slightly optimistic). The latch's memory follows the running rate; the
route does not. Whether a runtime rate change should be able to flip the
route (through the BE machinery the latch already uses) is open,
`STATUS.md` → Pending Work.

## Where it runs

`CircuitIR::from_mna` and `CircuitIR::from_kernel_with_dc_op` build the
trapezoidal IR, evaluate the rule on it (`CircuitIR::ring_promotion`), and
rebuild with backward Euler when it promotes (`IntegratorSelection::BeAuto`).
The promoted rebuild takes `CodegenConfig::max_iterations_be_promoted` (the
CLI's BE Newton budget). The verdict is recorded in
`CircuitIR::integration_reason`, `CodegenMeta::integration_reason`, the CLI
summary, and the generated `// provenance:` JSON (`"integration_reason"`).
A build kept trapezoidal also records `CircuitIR::be_latch_reference`
(passband gain, ring poles, index-2 hold) for the latch.

The whole-system `S·A_neg` spectral radius (`stability.rs`) is still what the
nodal emitter's Schur-versus-full-LU gate reads (`spectral_radius_s_aneg`),
and the router's DK-kernel estimate still selects DK or nodal. Neither
decides the integrator.
