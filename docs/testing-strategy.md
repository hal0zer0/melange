# Testing Strategy: 5-Layer Pyramid

Testing strategy for DK-method circuit solvers. Each layer catches a different class of bug.

```
     /  Layer 5  \     Behavioral (slow, empirical)
    / Layer 4     \    Transfer function (fast, analytical)
   / Layer 3       \   DC operating point (one-shot NR)
  / Layer 2         \  Linear algebra identities
 / Layer 1           \ Matrix stamps (pure arithmetic)
```

## Layer 1: Matrix Stamp Verification

Verify every nonzero entry in G and C against hand-computed values.

**Key tests:**
- Diagonal entries: sum of conductances at each node
- Off-diagonals: `-1/R` for connected nodes, `0` otherwise
- Capacitor stamps: `+C` on diagonal, `-C` off-diagonal
- C matrix symmetry: `C[i][j] == C[j][i]`
- N_v/N_i incidence matrices match topology

**Failure = wrong component value, wrong node index, or missing stamp.**

## Layer 2: Linear Algebra Identities

Verify matrix operations produce mathematically correct results.

**Key tests:**
- `S * A == I` (inverse identity)
- Sherman-Morrison matches brute-force re-inversion
- `K = N_v * S * N_i` matches full sparse product
- `A_neg == 2C/T - G` uses same G as `A == 2C/T + G`

## Layer 3: DC Operating Point

Find quiescent point via NR on full system.

**Key tests:**
- Node voltages within expected ranges
- Vbe ≈ 0.5-0.7V for active BJTs
- Iteration converges in < 100 steps
- DC solution sample-rate independent

## Layer 4: Small-Signal Transfer Function

Continuous-time linearized analysis (no sample rate dependency).

**Key tests:**
- Midband gain matches expected (±3 dB)
- Bandwidth not collapsed (f_c within spec)
- GBW scales correctly with gain
- Frequency response shape correct

**Method:** Complex 8×8 solve at `s = jω`, compare against SPICE `.ac`

## Layer 5: Time-Domain Behavioral

Transient simulation with audio signals.

**Key tests:**
- Gain measurement at 1 kHz
- THD/H2/H3 harmonic content
- Stability after impulse (settling)
- Step response (R_ldr changes)
- Dynamic modulation (tremolo behavior)

**Method:** Sine sweep, FFT analysis, envelope detection

---

## Debugging Hierarchy

When Layer N fails:
1. Verify Layer N-1 passes first
2. Use SPICE-Rust mapping table to isolate component
3. Check stamp → algebra → DC → AC → transient in order

Most bugs are Layer 1 (stamps) or Layer 2 (algebra). Layer 5 failures often trace back to Layer 3 (wrong DC bias).
