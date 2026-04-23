# AI Documentation Index — docs/aidocs/

Agent research library for melange circuit simulation. Each doc is optimized for
AI agent consumption: dense equations, code patterns, cross-references, no narrative.

> **Architecture note (2026-04).** The runtime solvers (`CircuitSolver`,
> `NodalSolver`, `DeviceEntry`, `solve_md`, `initialize_dc_op`) have been
> removed. `crates/melange-solver/src/solver.rs` is now an 8-line stub
> re-exporting `LinearSolver` only. **All circuit processing flows through
> the codegen pipeline** (`Netlist → MNA → DkKernel → CircuitIR → Emitter →
> generated Rust code`). If you find docs or examples that reference a
> "runtime solver", treat them as historical and prefer the codegen-equivalent
> path. The math is the same — only the entry point differs.

## When to Read Which Doc

| Task | Read |
|------|------|
| Modifying G, C matrices or stamping rules | [MNA.md](MNA.md) |
| Changing S, K, A matrices or kernel build | [DK_METHOD.md](DK_METHOD.md) |
| Changing Newton-Raphson iteration | [NR_SOLVER.md](NR_SOLVER.md) |
| Changing NR voltage limiting or convergence | [VOLTAGE_LIMITING.md](VOLTAGE_LIMITING.md) |
| Changing DC operating point solver | [DC_OP.md](DC_OP.md) |
| **Touching op-amp rail handling, Boyle macromodel, ActiveSet/ActiveSetBe** | **[OPAMP_RAIL_MODES.md](OPAMP_RAIL_MODES.md)** |
| Changing diode/BJT/JFET/MOSFET/tube equations | [DEVICE_MODELS.md](DEVICE_MODELS.md) |
| Changing Gummel-Poon BJT or qb() function | [GUMMEL_POON.md](GUMMEL_POON.md) |
| Changing matrix inversion or linear solves | [LINEAR_ALGEBRA.md](LINEAR_ALGEBRA.md) |
| Changing dynamic potentiometers (math) | [SHERMAN_MORRISON.md](SHERMAN_MORRISON.md) |
| Adding `.pot` / `.wiper` / `.gang` / `.switch` directives | [DYNAMIC_PARAMS.md](DYNAMIC_PARAMS.md) |
| Changing oversampling or anti-alias filters | [OVERSAMPLING.md](OVERSAMPLING.md) |
| Changing generated code structure | [CODEGEN.md](CODEGEN.md) |
| Changing trapezoidal integration or companions | [COMPANION_MODELS.md](COMPANION_MODELS.md) |
| Changing augmented MNA inductor handling | [MNA.md](MNA.md) (augmented MNA section) |
| Debugging solver output issues | [DEBUGGING.md](DEBUGGING.md) |
| Changing signal levels or DC blocking | [SIGNAL_LEVELS.md](SIGNAL_LEVELS.md) |
| Running SPICE validation tests | [SPICE_VALIDATION.md](SPICE_VALIDATION.md) |
| Checking project status, validation results, routing | [STATUS.md](STATUS.md) |
| Modeling tungsten-based components (Schottky, cat's whisker, WSe2) | [TUNGSTEN_MATERIAL.md](TUNGSTEN_MATERIAL.md) |
| Authentic circuit noise (Johnson-Nyquist, shot, 1/f, op-amp en/in) | [NOISE.md](NOISE.md) |
| Per-unit variation (`.mismatch`, `.tolerance`, `.seed`) | [UNIT_VARIATION.md](UNIT_VARIATION.md) |

## Critical Equations (Verified)

### Trapezoidal Integration
```
alpha = 2/T = 2 * sample_rate
A = G + alpha*C
A_neg = alpha*C - G
S = A^{-1}
```

### DK Method — K Matrix
```
K = N_v * S * N_i       (NO negation — K is naturally negative)
```

### Newton-Raphson Jacobian
```
f(i) = i - i_dev(p + K*i)
J[i][j] = delta_ij - sum_k J_dev[i][k] * K[k][j]
```

### DC Operating Point
```
G_aug = G_dc - N_i * J_dev * N_v    (SUBTRACTION — critical sign!)
rhs   = b_dc + N_i * (i_nl - J_dev * v_nl)
v_new = G_aug^{-1} * rhs
```

### Augmented MNA (Inductors, Codegen Nodal Path)
```
n_nodal = n_aug + total_inductor_windings
G_nodal = [G,      N_L   ]     C_nodal = [C,  0 ]
          [-N_L^T,  0     ]               [0,  L ]
A_nodal = G_nodal + (2/T)*C_nodal
A_neg   = (2/T)*C_nodal - G_nodal   (zero VS/VCVS rows only, NOT inductor rows)
```
Used by the codegen "nodal" routing path (selected automatically for circuits
with multi-transformer groups, M ≥ 10, or K ill-conditioning).

### Sherman-Morrison (Potentiometers)
```
S' = S - scale * su * su^T
scale = delta_g / (1 + delta_g * u^T * S * u)
K' = K - scale * (N_v * su) * (su^T * N_i)
```

## Sign Convention Summary

| Symbol | Convention | Value |
|--------|-----------|-------|
| N_i[anode] | Current extracted from anode | -1 |
| N_i[cathode] | Current injected into cathode | +1 |
| K = N_v*S*N_i | Naturally negative | Correct negative feedback |
| DC OP G_aug | G_dc minus N_i*J_dev*N_v | Subtraction for convergence |
| Voltage source RHS row | `rhs[k] = V_dc` (algebraic) | NOT multiplied by 2 |
| Current source RHS row | `rhs[node] += 2 * I_dc` | Multiplied by 2 for trapezoidal |

## Common Bug Signatures

| Symptom | Cause | Fix |
|---------|-------|-----|
| Explosion | Extra K negation | Use `K = N_v*S*N_i` (no negation) |
| No output | Input R not stamped before kernel | Add to G[in,in] before from_mna() |
| DC offset | Double history | Remove cap_history, use A_neg only |
| Quiet | INPUT_RESISTANCE too high | Use 1 ohm (near-ideal voltage source) |
| Unstable | No voltage limiting | Add SPICE pnjlim/fetlim |
| Wrong freq | Wrong alpha | Use 2/T not 1/T |
| BJT no output | No DC OP init | Call initialize_dc_op() or use DC_NL_I |
| DC OP diverges | Wrong Jacobian sign | Use G_aug = G_dc - N_i*J_dev*N_v |
| NR oscillation | No pnjlim/fetlim | See VOLTAGE_LIMITING.md |
| Pot no effect | Rebuild not triggered | Check set_pot_N() calls rebuild_matrices() |
| NaN in nodal codegen | Inductor currents not initialized | Copy v_node[n_aug..] from DC OP |
| Nodal codegen wrong output | Zeroed inductor A_neg rows | Only zero VS/VCVS rows (n_nodes..n_aug) |
| Xfmr NR diverges exponentially | Non-PD inductance matrix (inconsistent k values) | Ensure all k on same core are similar; MNA warns on non-PD |
| Absolute tol on HV circuit | 1μV on 290V = 3.4 ppb | RELTOL: 1e-3*max(\|v\|)+1e-6 (FIXED) |
| Trapezoidal NR ringing | Oscillatory mode at Nyquist | BE fallback retries with L-stable method (FIXED) |

## Document Inventory

| Doc | Size | Topics |
|-----|------|--------|
| [MNA.md](MNA.md) | Comprehensive | All 17 stamping rules, N_v/N_i matrices, parasitic caps |
| [DK_METHOD.md](DK_METHOD.md) | Comprehensive | A/S/K matrices, RHS, NR loop, trapezoidal nonlinear |
| [NR_SOLVER.md](NR_SOLVER.md) | Comprehensive | Residual, Jacobian, update step, M=1/2/3+ solves |
| [VOLTAGE_LIMITING.md](VOLTAGE_LIMITING.md) | Comprehensive | pnjlim, fetlim, vcrit, scalar damping alpha |
| [DC_OP.md](DC_OP.md) | Comprehensive | Companion formulation, source/gmin stepping, device eval |
| [DEVICE_MODELS.md](DEVICE_MODELS.md) | Comprehensive | Diode, BJT, JFET, MOSFET, tube, op-amp equations |
| [OPAMP_RAIL_MODES.md](OPAMP_RAIL_MODES.md) | Comprehensive | All 5 rail modes, BoyleDiodes scaffolding, heavy-clip root cause + rejected fixes + next-tier escalations |
| [GUMMEL_POON.md](GUMMEL_POON.md) | Comprehensive | qb(), Early effect, high injection, GP Jacobian |
| [LINEAR_ALGEBRA.md](LINEAR_ALGEBRA.md) | Comprehensive | LU, Gauss elim, equilibrated inversion, Sherman-Morrison |
| [SHERMAN_MORRISON.md](SHERMAN_MORRISON.md) | Comprehensive | Rank-1 updates, precomputed vectors, multi-pot |
| [DYNAMIC_PARAMS.md](DYNAMIC_PARAMS.md) | Reference | `.pot` / `.wiper` / `.gang` / `.switch` directives, plugin param emission |
| [OVERSAMPLING.md](OVERSAMPLING.md) | Comprehensive | Polyphase allpass half-band, 2x/4x, coefficients |
| [CODEGEN.md](CODEGEN.md) | Comprehensive | Generated code structure, templates, codegen capability matrix |
| [COMPANION_MODELS.md](COMPANION_MODELS.md) | Reference | Trapezoidal companion for C and L |
| [DEBUGGING.md](DEBUGGING.md) | Reference | Bug signatures, diagnostic patterns, verified values |
| [SIGNAL_LEVELS.md](SIGNAL_LEVELS.md) | Reference | DC blocking, output scaling, plugin levels |
| [SPICE_VALIDATION.md](SPICE_VALIDATION.md) | Reference | ngspice setup, correlation benchmarks |
| [STATUS.md](STATUS.md) | Reference | What's implemented, validated circuits, codegen routing |
| [TUNGSTEN_MATERIAL.md](TUNGSTEN_MATERIAL.md) | Reference | Tungsten metal/compound properties, Schottky barriers, cat's whisker diodes, WSe2/WS2 semiconductors |
| [NOISE.md](NOISE.md) | Comprehensive | Authentic circuit noise: Johnson-Nyquist/shot/flicker/op-amp PSD, RHS injection, RNG, phases 1-6 |
| [UNIT_VARIATION.md](UNIT_VARIATION.md) | Reference | `.mismatch` (device params) + `.tolerance` (passive R/C/L) + `.seed`; deterministic FNV→SplitMix64 RNG |

## External References

1. **TU Delft Analog Electronics Webbook** — MNA stamping reference
2. **Hack Audio Circuit Modeling Tutorial** — DK method, NR solver for audio
3. **Yeh (2009)** — "Digital Implementation of Musical Distortion Circuits" (PhD thesis)
4. **Pillage, Rohrer, Visweswariah (1995)** — "Electronic Circuit and System Simulation Methods"
5. **Ho, Ruehli, Brennan (1975)** — Original MNA paper
6. **SPICE3f5 source** — pnjlim, fetlim, device model reference implementations
