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
```

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

- all eigenvalues: balancing by powers of 2 (EISPACK `balanc`), reduction to
  Hessenberg form (`elmhes`), Francis double-shift QR (`hqr`);
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
similarities scaled over 1e12, a propagator-like spectrum) and the 5 in-repo
golden decks. On demand (`MELANGE_RING_DIR`, `--ignored`): the 42 corpus
decks. Measured 2026-09-29: all pass. Relative error on eigenvalues with
`|z| ≥ 0.9` ≤ 4.3e-10, except a close pair near +0.9998 on warpony (1.5e-7,
κ = 8e5, never read by the ring rule). The 4.3e-10 is sat-core-open's
promoting pole, and there it is the LAPACK reference that is off: 50-digit
arithmetic gives −0.9961868486952391, `eigen.rs` −0.9961868486952, numpy
−0.9961868482626 (its `inv(A)`, condition 5e8, is the less accurate of the two).

## Corpus verdict (2026-09-29, 42 decks, internal rate)

Decks with a lasting Nyquist-side pole; every other deck has none:

| Deck | Loudest lasting pole | Input residue (rel H_pink) | Verdict |
|---|---|---|---|
| noyce-amp-at-idle | −0.99033 (τ 2 ms) | −39.3 dB | BE |
| sat-core-open | −0.99619 (τ 5 ms) | −48.2 dB | BE |
| noyce-transformer-triode | −0.99997 (τ 0.70 s) | −77.7 dB | trap |
| wurli-power-amp | −0.99896 | −105.0 dB | trap |
| passive-eq1a | −0.98655 | −119.2 dB | trap |
| steve-1073-preamp | −0.99082 | −120.6 dB | trap |
| noyce-tape-head | −0.99985 | −168.0 dB | trap |
| basic-bitch, sad-bastard | −0.99994 | < −190 dB | trap |
| champ-5f1, sat-core-loaded, noyce-smps-ripple | −1 (index-2) | < −200 dB | trap |

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

## The runtime latch uses the same threshold

The runtime BE-latch (`STATUS.md`, "Runtime BE-latch") engages on an
alternating mode that dominates the output. Dominance alone is relative to
the instantaneous output, and in a quiet tail anything dominates, so it used
to override this rule at the first quiet moment after a transient (it held
noyce-transformer-triode and wurli-power-amp on BE after their −63 dB and
−74..−81 dB rings). Its floor is now the larger of the node tolerance and
`1e-3 × ref`:

```
ref_n = max(H_pink · |u_n|,  d · ref_{n-1})
d     = max over BE_LATCH_RING_POLES of the Nyquist-side |z(fs)|, floored at 1e-3^(1/(0.01·fs))
z(fs) = (1 + λ/(2fs)) / (1 − λ/(2fs))        (the trapezoidal map, at the running internal rate)
```

- The reference is on this rule's scale: passband gain × input amplitude.
  An output-peak reference sits far above it on transformer-triode (its
  impulse-response peak is 0.6 against a 1 kHz gain of 0.053).
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
