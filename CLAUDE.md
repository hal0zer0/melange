# CLAUDE.md

Melange — Rust toolkit for circuit simulation to real-time audio DSP.

## Build & Test

```bash
cargo build --workspace
cargo test --workspace
cargo test -p melange-solver          # core solver tests
cargo test -p melange-validate --test spice_validation  # SPICE comparison (needs ngspice)
cargo run -p melange-cli
```

## Project Structure

```
crates/
  melange-primitives/   # Layer 1: DSP building blocks (filters, NR helpers, oversampling)
  melange-devices/      # Layer 2: Component models (diode, BJT, JFET, MOSFET, tube, opamp, LDR)
  melange-solver/       # Layer 3: MNA/DK solver + codegen (CORE)
  melange-validate/     # Layer 4: SPICE validation against ngspice
  melange-plugin/       # Layer 5: nih-plug integration (stub)
tools/
  melange-cli/          # CLI tool
docs/aidocs/            # Detailed math reference docs (READ THESE before changing core math)
```

## Core Math — Read Before Touching Solver Code

Detailed docs live in `docs/aidocs/`. Always read the relevant doc before modifying equations.

| Doc | When to read |
|-----|-------------|
| `docs/aidocs/DK_METHOD.md` | Changing S, K, A matrices or kernel build |
| `docs/aidocs/NR_SOLVER.md` | Changing Newton-Raphson iteration or Jacobian |
| `docs/aidocs/MNA.md` | Changing MNA stamping (G, C, N_v, N_i) |
| `docs/aidocs/DEVICE_MODELS.md` | Changing diode/BJT/tube equations |
| `docs/aidocs/DC_OP.md` | Changing DC operating point solver or bias initialization |
| `docs/aidocs/CODEGEN.md` | Changing generated solver code |
| `docs/aidocs/DEBUGGING.md` | Diagnosing solver output issues |

### Critical Sign Conventions

```
K = N_v * S * N_i          (NO negation — K is naturally negative)
J[i][j] = delta_ij - sum_k jdev[i][k] * K[k][j]   (NR Jacobian)
A = G + (2/T) * C          (trapezoidal forward matrix)
A_neg = (2/T) * C - G      (trapezoidal history matrix)
RHS input = (V_in(n+1) + V_in(n)) * G_in   (proper trapezoidal, NOT 2*V*G)
```

- **N_i[anode] = -1** (current extracted), **N_i[cathode] = +1** (current injected)
- K is naturally negative from this convention, giving correct negative feedback
- **Do NOT add a minus sign to K.** If NR diverges, the bug is elsewhere.

### Device Dimensions in NR

- Diodes: 1D per device (single voltage/current)
- BJTs: 2D per device (Vbe→Ic at start_idx, Vbc→Ib at start_idx+1)
- Device map built from netlist element order, mirrors MNA builder
- Codegen uses `jdev_i_k` naming for block-diagonal Jacobian entries

## Key Patterns

### Input Modeling (Thevenin Source)

The audio input is modeled as a voltage source with 1-ohm series resistance:

1. Stamp `G_in = 1.0` into `mna.g[input_node][input_node]` **before** building DK kernel
2. In the solver RHS: `(input + input_prev) * input_conductance`
3. The DK kernel bakes G into S = A^-1, so the stamp MUST happen before `DkKernel::from_mna()`

### DC Voltage Sources (e.g., VCC)

Norton equivalent: G = 1e6 S stamped in MNA G, current `2 * V * G` in rhs_const.

### Real-Time Safety

No alloc/locks/syscalls in audio processing. All buffers pre-allocated in solver constructors.

## SPICE Validation

Tests compare melange output against ngspice. Infrastructure in `crates/melange-validate/`.

- `circuit.cir` — full SPICE netlist (with VIN source) for ngspice
- `circuit_no_vin.cir` — same circuit minus VIN, for melange (input via conductance stamping)
- `.OPTIONS INTERP` added automatically for uniform timestep output
- Temp files use atomic counter for thread-safe naming

### Known Failure Signatures

| Symptom | Cause | Fix |
|---------|-------|-----|
| SPICE correlation ~ 0 | Input conductance not in G before kernel build | Stamp `mna.g[in][in] += G_in` before `from_mna()` |
| ~3% error linear circuit | Using `2*V*G` instead of `(V+V_prev)*G` | Track `input_prev`, use proper trapezoidal |
| Sample count mismatch | ngspice adaptive timestep | `.OPTIONS INTERP` (after title line!) |
| Output all zeros | Voltage source in no_vin netlist | Check circuit_no_vin.cir has no VIN |
| Output ±10V oscillating | Purely resistive circuit, no caps | Add parasitic capacitance or use backward Euler |
| BJT output wrong | No DC operating point | Call `initialize_dc_op()` or use `DC_NL_I` codegen constant |
| BJT period-3 oscillation | Backward Euler for nonlinear currents | Fixed: use full i_nl in correction (not delta) |
| DC OP NR diverges | Wrong Jacobian sign | Use `G_aug = G_dc - N_i·J_dev·N_v` (subtraction!) |

## Current Status (2026-03-01)

### Working
- Linear circuit simulation (RC lowpass matches ngspice to 0.03% RMS, 8-nines correlation)
- MNA stamping for R, C, L, voltage sources, diodes, BJTs
- DK kernel build with proper trapezoidal discretization
- NR solver: 1D, 2D (two 1D devices), and M-dimensional
- Codegen for diode and BJT circuits (up to M=4)
- Codegen uses proper trapezoidal RHS with `input_prev` tracking (matches runtime solver)
- Per-device `.model` params: each device gets its own IS, N, BF, BR, VT (heterogeneous models supported)
- Nonlinear DC operating point solver (Newton-Raphson with source stepping and Gmin stepping fallbacks)
- DC OP integrated into codegen (`DC_NL_I` constant) and runtime (`initialize_dc_op()`)
- DC operating point calculation includes input conductance
- Plugin template generates per-channel state for stereo (no cross-channel corruption)
- SPICE validation infrastructure with ngspice
- Input validation: parser rejects negative/zero/NaN/Inf component values; codegen validates node indices
- Error types are enums (`MnaError`, `DkError`, `CodegenError` with `InvalidConfig`) — no panicking library code
- MAX_M=8 bound prevents unbounded allocation in DK kernel
- CLI reports errors for unresolved node names (no silent defaults)
- **BJT common-emitter amplifier**: SPICE validation passes (correlation 0.965, 35% RMS)
  - Trapezoidal nonlinear integration: correction uses full `S*N_i*i_nl` (not delta)
  - Combined with `N_i*i_nl_prev` in RHS, gives proper trapezoidal average
- **Dynamic potentiometers**: `.pot R1 min max` directive marks a resistor as runtime-variable
  - Sherman-Morrison rank-1 updates: O(N²) correction instead of O(N³) re-inversion
  - Precomputed SM vectors (SU, USU, NV_SU, U_NI) baked into generated constants
  - Corrections applied to S, K, A_neg, and S*N_i products in codegen
  - Plugin template auto-generates `FloatParam` knobs for each pot
  - Max 2 pots per circuit; pot value stored in `CircuitState`

### Known Limitations
- Purely resistive nonlinear circuits oscillate (need capacitor damping)
- JFET/MOSFET not yet in codegen NR
- Koren triode equation needs rewrite (Phase 4a of review plan)
- Gummel-Poon BJT Jacobian uses full analytical derivatives (Early effect + high injection)

### Pending Work
- **Review plan**: See `.claude/plans/reflective-sprouting-stroustrup.md` for the full 7-phase fix plan. Phases 1-6 mostly complete; Phase 7 (testing quality) in progress.
