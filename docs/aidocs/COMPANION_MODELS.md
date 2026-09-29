# Companion Models (Trapezoidal Integration)

## Purpose
Convert reactive components (C, L) into equivalent conductance + current source for MNA.

## References
- Circuit Simulation Project: http://circsimproj.blogspot.com/2009/07/companion-models.html
- Pillage, Rohrer, Visweswariah. "Electronic Circuit and System Simulation Methods." McGraw-Hill (1995)

## Capacitor

### Differential Equation
```
i(t) = C · dv(t)/dt
```

### Trapezoidal Integration
```
i[n] = (2C/T)·(v[n] - v[n-1]) - i[n-1]
```

### Companion Model
```
g_eq = 2C/T = alpha·C    where alpha = 2/T
I_eq = (2C/T)·v[n-1] + i[n-1]   (history term)
i[n] = g_eq·v[n] - I_eq
```

### Equivalent Circuit
- Resistor g_eq in parallel with current source I_eq
- Current flows into positive terminal

### MNA Stamping
```
G[i,i] += g_eq, G[j,j] += g_eq
G[i,j] -= g_eq, G[j,i] -= g_eq
rhs[i] += I_eq, rhs[j] -= I_eq
```

## Inductor

### Differential Equation
```
v(t) = L · di(t)/dt
```

### Trapezoidal Integration
```
v[n] = (2L/T)·(i[n] - i[n-1]) - v[n-1]
```

### Companion Model
```
g_eq = T/(2L)
I_eq = i[n-1] + (T/2L)·v[n-1]
v[n] = (i[n] - I_eq) / g_eq
```

### Equivalent Circuit  
- Resistor g_eq in series with voltage source I_eq/g_eq
- Or: Norton equivalent with g_eq || current source

## Charge (Companion) Form — the Generated Integrator

Every generated solver (DK and nodal, every sub-path: Schur, full-LU,
adaptive sub-steps, sub-sample fire) integrates the circuit in the charge
form: the companion model above applied to the whole system at once, with
the capacitor currents carried as state. This section is the reference for
that per-sample equation.

### The system

With `x` the MNA unknowns (node voltages, then augmented branch rows),
the circuit is the DAE

```
G·x + C·ẋ − N_i·i_nl(x) = RHS_CONST + b(t)
```

where `RHS_CONST` holds the DC sources (current sources on node rows,
`V_dc` on voltage-source rows) and `b(t)` the time-varying ones: the audio
input `u·G_in`, `.inject` sources, `.runtime` sources, and noise currents.
Define the charge derivative

```
q_dot = C·ẋ        (capacitor currents on node rows; dΦ/dt on inductor branch rows)
```

### Per-sample equation

Trapezoidal integration of the charge, `C·(x_{n+1} − x_n) = (T/2)·(q_dot_{n+1} + q_dot_n)`, gives

```
q_dot_{n+1} = alpha·C·(x_{n+1} − x_n) − q_dot_n           alpha = 2/T
```

Substituting into KCL at `n+1`, `G·x_{n+1} + q_dot_{n+1} = N_i·i_nl(x_{n+1}) + RHS_CONST + b_{n+1}`:

```
A·x_{n+1} − N_i·i_nl(x_{n+1}) = RHS_CONST + H·x_n + q_dot_n + b_{n+1}

A = G + alpha·C
H = alpha·C            (algebraic augmented rows zeroed)
b_{n+1} = u_{n+1}·G_in + injections + runtime sources + noise, all at n+1
```

- `H` is stored in the arrays still named `a_neg` / `A_NEG_DEFAULT`. The
  zeroed rows are the VS / VCVS / ideal-transformer rows (`n_nodes..n_aug`,
  `Topology::history_zero_rows`); inductor branch rows keep their `alpha·L`.
- `RHS_CONST` is ×1 on every row: each source enters once, at `n+1`.
- The nonlinear current enters through the Newton solve at `n+1` alone.
  `i_nl_prev` is not stamped into any RHS (it remains the NR predictor's
  seed and the source of noise bias currents).
- `q_dot` is state (`state.q_dot`, length `N`), added to the RHS on every
  row whose `H` row is nonzero, and committed together with `v_prev`.

Backward Euler has the same shape with `alpha = 1/T` and no `q_dot`:

```
(G + C/T)·x_{n+1} − N_i·i_nl(x_{n+1}) = RHS_CONST + (C/T)·x_n + b_{n+1}
```

Under both integrators `A − H = G` on every row, so the DC fixed point
(`x_{n+1} = x_n`, `q_dot = 0`) is `G·x = RHS_CONST + N_i·i_nl(x)` — the
DC-OP equation of `DC_OP.md` with `b_dc = RHS_CONST` verbatim.

### `q_dot` update rules

`q_dot` exists only in trapezoidal builds; a backward-Euler build
(`--backward-euler`, `.integrator be`, auto-BE promotion, behavioral
sources) carries none, because its RHS never reads it.

| Event | `q_dot` after the event |
|---|---|
| Trapezoidal sample | `alpha·C·(x_{n+1} − x_n) − q_dot` |
| Backward-Euler sample inside a trap build (NR/ringing fallback, breakpoint-BE, BE-latch) | `(C/T)·(x_{n+1} − x_n)` — BE's own capacitor current; the BE RHS does not read `q_dot` |
| Adaptive sub-steps, sub-sample-fire segments | advanced per sub-step at the sub-step's own `alpha` (and its own integrator) |
| Held (unconverged) sample | unchanged |
| NaN / magnitude reset, `set_dc_operating_point()`, DK `recompute_dc_op()` writeback | `0` (rest) |
| `set_sample_rate()` | unchanged — `q_dot` is a current, independent of `T` |
| `Default` / `reset()` | `0`, or `Q_DOT_IC_SEED` on an `IC=` build |
| `IC=` start | `Q_DOT_IC_SEED = RHS_CONST + N_i·i_nl − G·x` on the charge-carrying rows, `0` on algebraic rows |

The `IC=` seed exists because the IC solve holds each `IC=` capacitor with a
voltage source, so the start point is not at rest: that source's current
is the capacitor's current at `t = 0`, and `Q_DOT_IC_SEED` is KCL evaluated
there (`CircuitIR::q_dot_ic_seed`, `crates/melange-solver/src/codegen/ir/mod.rs`).

**Saturating inductor branch rows.** The charge on a branch row is the flux
`Φ(i)`, not `L0·i`. The history swap `alpha·L0·i_n → alpha·Φ(i_n)` applies to
the RHS (`SATURATING_TRANSFORMERS.md` §3.2) and to the `q_dot` update, where
the linear `alpha·L0·(i_{n+1} − i_n)` becomes `alpha·(Φ(i_{n+1}) − Φ(i_n))`.

**Companion-model inductors (DK library path).** The CLI always builds
inductors as augmented branch rows. When the DK path companion-models them
instead, `H` carries no companion stamp; the `g_eq = T/(2L)` stamp stays in
`A`, and the companion's whole known current goes into its history source —
the single-step Norton source of the Inductor section above:

```
i_hist = i_L[n] + g_eq·v_L[n]                 (per inductor)
i_hist = i[n] + Y·v[n]                        (per winding, coupled pairs / transformer groups)
```

### Equivalence with the whole-system form

The alternative discretization sums KCL at `n` and at `n+1`, eliminating
`q_dot`:

```
A·x_{n+1} − N_i·i_nl(x_{n+1}) = 2·RHS_CONST + (alpha·C − G)·x_n + N_i·i_nl(x_n) + b_n + b_{n+1}
```

on the charge-carrying rows (melange's whole-system implementation zeroes the
history on the algebraic rows and keeps them at `G·x_{n+1} = RHS_CONST`, as
the charge form does). If KCL holds exactly at `n`, then
`q_dot_n = RHS_CONST + b_n + N_i·i_nl(x_n) − G·x_n`, and substituting it into the charge
form yields this equation term for term. The two forms are therefore
algebraically identical whenever every accepted solve is exact: a linear
circuit (one LU solve per sample) renders identically up to rounding. They
differ only where an accepted solve leaves a residual — NR tolerance, the
iteration cap, a held sample, a chord step, an active-set pin — and there
the charge form does not carry it.

### Why the charge form: the z = −1 walk

Let `w` be a left null vector of `C` (`wᵀ·C = 0`): a capacitor-less node
row, or a combination whose capacitor currents cancel, e.g.
`KCL(a) + KCL(b)` across a coupling capacitor between `a` and `b`. Write
`e_n = wᵀ·(G·x_n − N_i·i_nl(x_n) − RHS_CONST − b_n)` for the KCL residual on that
combination and `ρ_n` for the accepted solve's residual projected on `w`.

Whole-system form, projected on `w` (the `C` terms vanish):

```
e_{n+1} + e_n = ρ_{n+1}      →  e_{n+1} = ρ_{n+1} − e_n
```

Every accepted residual is fed into the next sample with a pole at
`z = −1` and no damping: the residual walks. Its fingerprint is
`e_n + e_{n−1} ≈ 0` with `|e|` far above the solver's convergence floor.

Charge form, projected on `w`:

```
e_{n+1} = ρ_{n+1}
```

The KCL residual at a committed sample is that sample's own solve
residual; nothing is inherited. On the charge-carrying directions the
committed `q_dot` is computed from the committed `x`, so the same holds
there.

### Measured

- **Cap-coupled diode witness** (`in −1k− a −100n− b`, antiparallel
  1N914-like diodes on `b`, `10k` to `c`, `22n`, `1k`, `100k`; ±square drive;
  nodal full-LU, forced trapezoidal; residual of `KCL(a) + KCL(b)` over the
  last 0.1 s). Whole-system: 1.31–1.52 µA at 96 kHz (1–5 V drive),
  0.43–0.52 µA at 192 kHz (1–2 V). Charge form: ≤ 0.011 µA and ≤ 0.005 µA
  respectively. At 48 kHz both forms are ≤ 0.05 µA.
- **Single-supply op-amp overdriving a diode clipper** (rails 0 / 9 V), with
  the transition-BE sample removed: the `n2` KCL residual is 0.29–0.43 µA
  at 48 / 96 / 192 kHz and 0.1 / 0.5 V drive (the floor) under the charge form, against
  105–2650 µA for the whole-system form.
- **Same deck, 1 kHz at 0.5 V, 48 kHz against a 768 kHz render**, both
  forms with transition-BE (the two forms agree to 8.6 µV at 768 kHz): op-amp output RMS error 0.49 V
  (whole-system) vs 1.1 mV (charge); clipper node 0.23 V vs 0.29 mV.

### What still uses the whole-system operator

Promotion to backward Euler is decided on the charge-form propagator
(`RING_PREDICATE.md`). The whole-system spectral radius `ρ(S·(alpha·C − G))`
(`codegen/stability.rs`) is still the nodal emitter's Schur-versus-full-LU
input (`spectral_radius_s_aneg`), and the router's DK-kernel estimate still
selects DK or nodal; neither decides the integrator. The library `DkKernel`
(`crates/melange-solver/src/dk.rs`) and the runtime `LinearSolver` built on
it (`crates/melange-solver/src/linear_solver.rs`, linear circuits only, where
the forms coincide) keep the whole-system matrices.

## Key Insight
Trapezoidal rule is implicit: solution at t[n] depends on itself. Companion model makes this explicit by converting differential equation to algebraic equation with equivalent conductance.

## Stability
- Trapezoidal: A-stable, 2nd order accurate, can ring
- Backward Euler: L-stable, 1st order, damps (use for problematic circuits)

## References
- http://circsimproj.blogspot.com/2009/07/companion-models.html
- Pillage & Rohrer, "Electronic Circuit and System Simulation Methods"
