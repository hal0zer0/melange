# AI Documentation Index

Quick reference for melange circuit simulation.

## Core Math References

| Doc | Purpose | Key Sources |
|-----|---------|-------------|
| [MNA.md](MNA.md) | Modified Nodal Analysis | Ho et al. (1975), TU Delft Webbook |
| [DK_METHOD.md](DK_METHOD.md) | Discrete Kirchhoff reduction | Hack Audio Tutorial, Yeh & Smith |
| [COMPANION_MODELS.md](COMPANION_MODELS.md) | Trapezoidal integration | Pillage & Rohrer, Circuit Simulation Project |
| [NR_SOLVER.md](NR_SOLVER.md) | Newton-Raphson | Hack Audio Ch. 7 |
| [DEVICE_MODELS.md](DEVICE_MODELS.md) | Diode/BJT equations | Shockley equation, Ebers-Moll |
| [CODEGEN.md](CODEGEN.md) | Generated code structure | melange implementation |
| [DEBUGGING.md](DEBUGGING.md) | Bug signatures | melange debugging experience |

## Critical Equations (Verified)

### Trapezoidal Integration
```
alpha = 2/T
A = G + alpha*C
A_neg = alpha*C - G
```

### DK Method - K Matrix
```
K = N_v · S · N_i   // NO negation — K is naturally negative for stable circuits
```

N_i uses injection convention:
- N_i[anode] = -1 (current extracted from anode)
- N_i[cathode] = +1 (current injected into cathode)
- This makes K naturally negative, providing correct negative feedback

### Newton-Raphson Jacobian

**Runtime** (full device Jacobian matrix G):
```
J[i][j] = delta_ij - sum_k K[i][k] * G[k][j]
```

**Generated code** (block-diagonal device Jacobian):
```
J[i][j] = delta_ij - sum_k jdev_{ik} * K[k][j]
```
where k ranges over the device block that owns row i.

For diodes (1D): single term `jdev_{d}_{d} = diode_conductance(v_d)`.
For BJTs (2D): 2x2 block from `bjt_jacobian(Vbe, Vbc)`.

Since K is naturally negative, both formulas give J > 0 (convergent).

## Sign Convention Summary

| Symbol | Convention | Result |
|--------|-----------|--------|
| N_i[anode] | Current extracted from anode | -1 |
| N_i[cathode] | Current injected into cathode | +1 |
| K = N_v·S·N_i | Naturally negative | Correct negative feedback |

## Common Bug Signatures

| Symptom | Cause | Fix |
|---------|-------|-----|
| Explosion | Extra K negation | Use `K = N_v·S·N_i` (no negation) |
| No output | Input R not stamped | Add to G[in,in] |
| DC offset | Double history | Remove cap_history, use A_neg only |
| Quiet | INPUT_RESISTANCE too high | Use 1Ω (near-ideal voltage source) |
| Unstable | No step clamping | Add STEP_CLAMP = 0.01 |
| Wrong freq | Wrong alpha | Use 2/T not 1/T |

## External References

1. **TU Delft Analog Electronics Webbook**
   https://analog-electronics.ewi.tudelft.nl/webbook/SED/
   Comprehensive MNA stamping reference

2. **Hack Audio Circuit Modeling Tutorial**
   https://hackaudio.com/tutorial-courses/audio-circuit-modeling-tutorial/
   DK method, NR solver for audio circuits

3. **Circuit Simulation Project Blog**
   http://circsimproj.blogspot.com/2009/07/companion-models.html
   Companion models explained step-by-step

4. **Pillage, Rohrer, Visweswariah**
   "Electronic Circuit and System Simulation Methods" (1995)
   Standard reference for circuit simulation

5. **Ho, Ruehli, Brennan (1975)**
   "The Modified Nodal Approach to Network Analysis"
   Original MNA paper

## Test Circuits (Verified)

1. **Voltage divider** (10k/10k) → Gain = 0.5
2. **Diode clipper** → Unity + clip at ±0.7V
3. **RC lowpass** (10k, 0.1uF) → -3dB @ 159Hz
