# AI Documentation Index

Quick reference for melange circuit simulation.

## Core Math References

| Doc | Purpose | Key Sources |
|-----|---------|-------------|
| [MNA.md](MNA.md) | Modified Nodal Analysis | Ho et al. (1975), TU Delft Webbook |
| [DK_METHOD.md](DK_METHOD.md) | Discrete Kirchhoff reduction | Hack Audio Tutorial, Yeh & Smith |
| [COMPANION_MODELS.md](COMPANION_MODELS.md) | Trapezoidal integration | Pillage & Rohrer, Circuit Simulation Project |
| [NR_SOLVER.md](NR_SOLVER.md) | Newton-Raphson | Hack Audio Ch. 7 |
| [DEVICE_MODELS.md](DEVICE_MODELS.md) | Diode/BJT/JFET/MOSFET/Tube equations | Shockley, Ebers-Moll, Shichman-Hodges, Koren |
| [DC_OP.md](DC_OP.md) | DC operating point solver | SPICE DC analysis, Nagel (1975) |
| [CODEGEN.md](CODEGEN.md) | Generated code structure | melange implementation |
| [DEBUGGING.md](DEBUGGING.md) | Bug signatures | melange debugging experience |
| [SIGNAL_LEVELS.md](SIGNAL_LEVELS.md) | Signal level contract, DC blocking, diagnostics | melange implementation |

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
For JFETs (2D): 2x2 block from `jfet_jacobian(Vgs, Vds, ...)`.
For MOSFETs (2D): 2x2 block from `mosfet_jacobian(Vgs, Vds, ...)`.
For Tubes (2D): 2x2 block from `tube_jacobian(Vgk, Vpk, ...)`.

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
| Unstable | No voltage limiting | Add SPICE pnjlim/fetlim (per-device VCRIT) |
| Wrong freq | Wrong alpha | Use 2/T not 1/T |
| BJT no output | No DC OP init | Call `initialize_dc_op()` or use `DC_NL_I` constant |
| DC OP diverges | Wrong Jacobian sign | Use `G_aug = G_dc - N_i·J_dev·N_v` (subtraction!) |

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

### DC Operating Point

Nonlinear DC OP solver in `dc_op.rs`. Companion formulation:
```
G_aug = G_dc - N_i · J_dev · N_v   (subtraction — see DC_OP.md for why)
rhs   = b_dc + N_i · (i_nl - J_dev · v_nl)
v_new = G_aug^{-1} · rhs
```

Convergence chain: Direct NR → Source Stepping → Gmin Stepping → Linear Fallback.

## Test Circuits (Verified)

1. **Voltage divider** (10k/10k) → Gain = 0.5
2. **Diode clipper** → Unity + clip at ±0.7V
3. **RC lowpass** (10k, 0.1uF) → -3dB @ 159Hz
4. **Single diode + VCC** → V_anode ≈ 0.65V (DC OP verified)
5. **BJT CE bias** → Vbe ≈ 0.65V, Ic ≈ 1.5mA (DC OP verified)
