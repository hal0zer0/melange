# Melange — Validation & Verification (V&V)

**What we can prove about melange's correctness, what we cannot, and how each
claim was established.** This is the honest ledger: every row states its
evidence *and* its reach. Initial content: 2026-08-03.

---

## 1. Framing: verification vs. validation

Borrowed from the ASME V&V vocabulary, because the distinction is load-bearing:

- **Verification** — "are we solving the equations *right*?" Pure math/numerics,
  no hardware required. This is where melange can be *proven* to machine
  precision, oracle-free.
- **Validation** — "are we solving the *right* equations?" Does a given model
  match physical reality? This requires external ground truth (measurement,
  datasheet, published spec) and is inherently bounded by the quality of that
  reference.

A critical consequence, stated plainly: **solver correctness ≠ circuit
fidelity.** Melange can flawlessly solve a netlist that does not match the real
device it is meant to model. The two are separate claims; only the first is
provable from inside this repository.

There is **no certification authority** for analog circuit simulators — commercial
SPICE tools are trusted on reputation and empirical agreement, not a stamp. What
follows is the strongest substitute: a layered set of checks, most of which need
no external oracle at all.

---

## 2. Summary

| Tier | Claim | Method | Reach |
|------|-------|--------|-------|
| **1 — Numerical** | The solver solves the discretized equations correctly | Closed-form / invariant checks, oracle-free | **Proven** to 1e-11…1e-16 |
| **2 — Device models** | Device equations match their canonical published forms | Hand-computed anchors + physics invariants vs. textbook models, independent of ngspice | **Proven** to 1e-12 |
| **3 — Circuit fidelity** | A specific netlist reproduces the real unit | Measured/published hardware references | **Anchored for one circuit** (Wurlitzer 200A preamp), hobbyist-grade |
| Cross-sim | Generated code tracks ngspice on 13 decks | ngspice waveform correlation (CI) | Pre-existing; peer agreement, not ground truth |

Prior to this effort, essentially *all* device correctness in melange grounded
out in ngspice waveform correlation — a circular check (a shared model error
would pass). Tiers 1 and 2 break that circularity: their references are
mathematics and published model equations, not another simulator.

---

## 3. Tier 1 — Numerical verification (oracle-free)

Proves the solver core is mathematically correct. No external reference: the
answers come from calculus and physical invariants.

### 3.1 Integrator order — `crates/melange-solver/src/dk_math_verification.rs`
The trapezoidal integrator is *claimed* 2nd-order. Verified by marching the
companion kernel against the **continuous-time** analytical solution at
successively halved timesteps and checking the error quarters:

- RC (first-order system): error ratios **4.001, 4.000** per dt-halving → order 2.
- Resonant RLC (couples L- and C-history): ratios **3.997, 4.000** → order 2.
- Ideal voltage divider: settles to the exact series-divider ratio to **1e-12**.

The pre-existing companion tests compare against the *discrete* recurrence and
are blind to integration order by construction; these compare against physical
truth. *(commit `9bf1b7d`)*

### 3.2 Power balance (Tellegen) — `crates/melange-solver/tests/tellegen_power_balance_tests.rs`
Tellegen's theorem: for any lumped network obeying KCL/KVL, Σ(v·i)=0 at every
instant — independent of the elements' constitutive laws. Checked on the
**actual generated-and-compiled binary** using melange's own MNA matrices (no
hand-encoded topology), so a stamp-sign bug cannot be masked by a matching bug
in the test.

| Circuit | DC residual | Dynamic (per-sample) residual |
|---------|-------------|-------------------------------|
| Resistive divider (linear) | 0 | — |
| RC (reactive) | 3.6e-14 A | 1.8e-15 |
| Diode clipper (1D nonlinear) | 8.9e-16 A | 4.2e-15 |
| BJT common-emitter (2D nonlinear + VS) | 1.2e-11 A | 3.5e-11 |
| Series RLC (inductor branch) | — | 2.05e-12 |

Covers linear → reactive(C) → reactive(L) → 1D- and 2D-nonlinear + voltage-source,
at DC steady state and across transients. **Negative controls** (deliberately
flipping the current-injection sign, or dropping the reactive term) blow the
residual to ~1e-3 A — confirming the checks genuinely catch stamping errors.
*(commits `9bf1b7d`, `0ceb872`, `856b250`)*

> Note: instantaneous power conservation (Σv·i=0 every sample) *is* the
> time-derivative of energy conservation, so a separate energy-conservation test
> would re-prove the same physics — it is subsumed here.

---

## 4. Tier 2 — Device-model conformance (oracle-free)

Every device model checked against its **published equation**, implemented
independently in-test with the arithmetic of each anchor point shown so a reader
can verify it by hand. References are the originating publications, *not*
ngspice's C source. All agreements to **1e-12** (the devices-crate `safe_exp` is
exact within its clamp) unless noted.
Files: `crates/melange-devices/tests/canonical_conformance.rs`,
`crates/melange-solver/tests/diode_breakdown_conformance.rs`.

| Device | Canonical model | Checks |
|--------|-----------------|--------|
| MOSFET | Shichman-Hodges L1 | hand-computed triode/sat/cutoff points, triode↔sat continuity, `Id ∝ Vov²` (ratio=4), exact λ modulation |
| JFET | Shichman-Hodges | same, plus the defining `Id = IDSS at Vgs=0` |
| Diode | Shockley | 6 bias points, and ΔV=N·VT ⇒ ×e exactly |
| Diode breakdown | Zener exponential | `I(−BV) = −IBV` exact, ×e steepness, ±40 clamp (was **zero** tests) |
| BJT | SPICE Gummel-Poon | reduces to Ebers-Moll (β=BF) at ∞ params; Ic & Ib vs SGP across high injection; **β·qb = BF** (qb modulates collector, not base current) |
| VCA | Blackmer (THAT 2180) | `gain=G0·exp(−Vc/VSCALE)`, dB-linear (6.1 mV/dB ⇒ −1 dB exactly) |
| Pentode | Reefman "Derk" | **Ip/Ig2 independent of Vgk** (shared Koren current cancels; 8.97174024 across Vgk ∈ {−5…−40}), plate flatness, monotonic grid control, cutoff |

The BJT-GP test would catch the historical `q2`-without-`−1` divergence that
ngspice-correlation tolerates at a few percent but this pins to machine
precision. *(commits `6ab3f96`, `00c6a4d`, `f968e2c`, `ca2d7e6`)*

Caveats worth stating: the pentode uses physics invariants + structural
identity rather than a full transcription of the (intricate) Derk equations;
the diode-breakdown check confirms the *shipping* `fast_exp` (a 5th-order
minimax polynomial, <0.0004% error) tracks the canonical exponential to that
accuracy, with the `I(−BV)=−IBV` anchor still exact.

---

## 5. Tier 3 — Circuit fidelity (the honest part)

This is the only tier that touches whether a *specific reconstructed netlist*
matches a real unit. **Most melange/oomox circuits have no available lab-grade
ground truth** — they were reconstructed from schematics/data, not measured
against a reference. So this tier is thin by necessity, and every anchor's grade
is stated.

### 5.1 Wurlitzer 200A preamp — anchored, passing

External ground truth (via the openwurli peer): two *independent, agreeing*
real-hardware sources — schematic #203720-S-3 annotations and GroupDIY multimeter
readings on a real unit — plus Brad Avenson's measured gain and EP-Forum tremolo
measurements. **Grade: published-spec + hobbyist-multimeter, not lab-grade.**

**DC operating point** (`melange dc-op`):

| Node | Real hardware | melange | Δ |
|------|---------------|---------|---|
| TR‑1 base | 2.45 V | 2.80 | +14% |
| TR‑1 emitter | 1.95 V | 2.24 | +15% |
| TR‑1 coll / TR‑2 base | 4.1 V | 4.12 | +0.5% |
| TR‑2 emitter | 3.4 V | 3.46 | +1.7% |
| TR‑2 collector | 8.8 V | 9.20 | +4.5% |

The TR‑2 collector-side nodes land within carbon-comp resistor tolerance. The
TR‑1 base/emitter offset is a **documented provenance gap, not a model error**:
the annotations imply β≈65 at 70 µA, which is a 2N2924-class (original) part, not
the 2N5089 (super-β, hFE 400-1200) the model represents; the annotations are also
only self-consistent to ~19% (they violate KCL as written). Per the peer's
ruling, this discrepancy is non-load-bearing — gain/Miller/clipping key off Ic
and Vce, which match. **Pass surface: TR‑2 nodes + stage-1 Zin + gain.**

**Closed-loop gain** (`melange analyze`) — the load-bearing metric, and it passes
cleanly:

| Shunt (R_ldr) | melange @1kHz | reference |
|---------------|---------------|-----------|
| 13 kΩ (real no-vibrato) | **14.15 dB** | Avenson hardware **~15 dB** — matched to <1 dB |
| tremolo boost (19k vs 1M) | **6.08 dB** | EP-Forum hardware **6.1 dB** — to 0.02 dB |

(The netlist's "6.0/12.1 dB" figures are gain-vs-shunt lookup points, not the
operating gain; the real no-vibrato shunt is ≈13 kΩ because the vibrato pot
always loads the feedback node.)

**Verdict:** the engine reproduces a real preamp's measured DC bias (on its
load-bearing surface) and AC gain (<1 dB) — a genuine, if modest-grade, fidelity
pass. It is *one* circuit.

---

## 6. What is NOT proven

Stated bluntly, because the credibility of everything above depends on not
overclaiming:

1. **Circuit fidelity is anchored for exactly one circuit** (the Wurli preamp),
   at hobbyist-multimeter/published-spec grade. Every other shipped circuit's
   fidelity to its real-world counterpart is **unvalidated** — Tiers 1–2 prove
   the engine, not the netlists.
2. **No lab-grade hardware measurements exist** for the ecosystem's circuits. No
   swept frequency-response or THD-vs-level of a real unit is available; those
   tables in the docs are SPICE-derived (cross-simulator, not ground truth).
3. **Cross-simulator agreement (ngspice, 13 decks in CI) is a peer check, not
   ground truth.** ngspice can be wrong too, and the comparison DC-blocks both
   sides and hardcodes the same Thevenin input — shared assumptions cancel.
4. **THD/harmonic character is not a validated metric for feedback-linearized
   stages** (e.g. the Wurli preamp: its character is the pickup, not the preamp).

Closing any of these requires new external ground truth — a bench measurement, a
datasheet curve, or a trusted third-party model — chosen deliberately per
circuit, not inferred.

---

## 7. The process works — bugs it surfaced

Evidence that the validation net has teeth: building it shook out real defects.

- **Netlist sync-drift** — the Wurli preamp's `.cir` had forked between
  melange-circuits and openwurli (a stripped constant-β card vs. the full
  Gummel-Poon card). Surfaced by the DC anchor; fixed downstream
  (melange-circuits `327ad86`).
- **Parser rejected `ISC=0` / `ISE=0`** — valid SPICE (disables leakage; also
  melange's own default), so a standard card couldn't be written. Fixed
  (`49bd4bc`).
- **Codegen `clippy::manual_swap`** — nodal emitter emitted a manual element
  swap; now emits `x.swap()` (`49bd4bc`).
- **Power-amp overdrive robustness** — the Wurli power amp's internal node
  voltages diverge erratically under drive (convergence-path-dependent, not
  drive-gated). Under active investigation; delivered output is bounded only by
  the output clamp. *(open)*

A methodological lesson also logged: **verify against the canonical/primary
source before acting.** The DC anchor's first pass ran a drifted netlist copy
and produced a wrong conclusion, corrected only after checking the canonical
card — the same discipline this document is built on.

---

## 8. Reproducing the validations

```bash
# Tier 1 — numerical (oracle-free)
cargo test -p melange-solver --lib dk_math_verification
cargo test -p melange-solver --test tellegen_power_balance_tests -- --test-threads=1

# Tier 2 — device conformance (oracle-free)
cargo test -p melange-devices --test canonical_conformance
cargo test -p melange-solver --test diode_breakdown_conformance

# Tier 3 — Wurli fidelity anchors (need the netlist)
melange dc-op  ../melange-circuits/unstable/preamp/wurli-preamp.cir     # DC bias
melange analyze <wurli-preamp with R_ldr=13k> --start-freq 200 --end-freq 5000  # gain

# Pre-existing cross-sim (needs ngspice, runs in CI)
cargo test -p melange-validate --test spice_validation -- --include-ignored
```

The Tier-3 checks are kept as **documented anchors** rather than committed tests:
they run against the openwurli-synced netlist, and a frozen test copy would only
risk re-introducing the sync-drift this effort just fixed.

---

*Maintenance: append new validations with their evidence **and** their reach.
The value of this document is its honesty about what each check does and does not
prove — keep that discipline.*
