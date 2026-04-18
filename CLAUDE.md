# CLAUDE.md

Melange — Rust toolkit for circuit simulation to real-time audio DSP. Compiles SPICE netlists to optimized Rust code (generated solver → VST/CLAP via thin nih-plug wrapper).

## Oomox Ecosystem — Division of Labor

```
Real hardware → SCHEMER → MELANGE-CIRCUITS → MELANGE → OOMOX → Product plugin
  (schematics)   (netlists)    (.cir files)     (Rust code)  (binary VST/CLAP)
```

- **SCHEMER** (`~/dev/schemer`): extracts netlists from schematic images. Local-only, not public.
- **MELANGE-CIRCUITS** (`~/dev/melange-circuits`): all non-test circuit netlists.
- **MELANGE** (`~/dev/melange`) — **this repo**. GPL-3.0. Parser, MNA, solvers, codegen. Generated code is GPL.
- **OOMOX** (`~/dev/oomox`): binary plugin builder. Pot curves, smoothing, warmup, output scaling, UI, presets.

### Boundary Rules
- **Circuit sounds wrong** → fix in MELANGE (solver/models) or MELANGE-CIRCUITS (component values). Never work around in OOMOX.
- **Plugin wrong** (clicks, CPU, knob feel, UI) → fix in OOMOX. Never change the `.cir` to work around a plugin issue.
- **New circuit** → draft in MELANGE-CIRCUITS. Use SCHEMER if starting from real hardware.
- **Pot mapping / indices** → defined by `.cir`, assigned by MELANGE, wired by OOMOX. All three must agree.

## CRITICAL: Accuracy beats plugin usability 1000x

Accurately modeling a circuit is **1000x more important** than quickly building a working plugin. Melange is a "build whatever circuit you throw at me" tool, NOT a "tweak the circuit to do what I want" tool.

When a plugin sounds wrong (clipping, blown speaker, too hot, too quiet):
- **The bug is in melange** (solver, codegen, device models) — assume this until proven otherwise.
- **Never add workaround components** (catch diodes, padding resistors, snubbers) even if real hardware has equivalents. Fix the MNA/device layer.
- **Never modify component values** to dodge solver edge cases — fix the solver.
- **Never suggest "turn the knobs down"** — that's avoiding the buggy region, not fixing it.
- **Never use `--output-scale`** as a remedy for simulation errors — it's a voltage→DAW mapping, not a bug fix.
- **Performance, ergonomics, and polish are always secondary** to schematic fidelity. A slow accurate solver beats a fast wrong one.

## CRITICAL: Use CLI flags when two circuits need conflicting behavior

When fixing one circuit regresses another, don't pick the lesser evil — expose both behaviors via a CLI flag named after the mechanism (not the circuit). Existing examples: `--solver {auto|dk|nodal}`, `--backward-euler`, `--oversampling {1|2|4}`, `--tube-grid-fa {auto|on|off}`, `--opamp-rail-mode`. Default to the broadest safety envelope. Every selectable mode has to be correct for some class of circuits — flags are not a license to ship broken modes.

## CRITICAL: Never simplify circuits

The goal is ALWAYS to get melange to run the real circuit, NEVER to simplify a circuit to work around melange limitations. If a real hardware topology doesn't work in melange, that is a melange bug — fix melange. Do not substitute "simplified approximations" of real circuit sections.

## Build & Test

```bash
cargo build --workspace
cargo test --workspace
cargo test -p melange-solver          # core solver tests
cargo test -p melange-validate --test spice_validation -- --include-ignored  # needs ngspice
cargo run -p melange-cli
```

## Project Structure

```
crates/
  melange-primitives/   # Layer 1: DSP building blocks (filters, NR helpers, oversampling)
  melange-devices/      # Layer 2: Component models (diode, BJT, JFET, MOSFET, tube, opamp, VCA)
  melange-solver/       # Layer 3: MNA/DK solver + codegen (CORE)
  melange-validate/     # Layer 4: SPICE validation against ngspice
  melange-plugin/       # Layer 5: nih-plug integration (stub)
tools/
  melange-cli/          # CLI tool
docs/aidocs/            # Detailed math reference docs — READ THESE before changing core math
```

## Agent Research Library — `docs/aidocs/`

**MANDATORY**: Before modifying solver math, matrix operations, device models, or codegen, read the relevant doc(s). Full index at `docs/aidocs/INDEX.md`.

| Doc | When to read |
|-----|-------------|
| `MNA.md` | Changing MNA stamping (G, C, N_v, N_i), any component stamping rules |
| `DK_METHOD.md` | Changing S, K, A matrices or kernel build |
| `NR_SOLVER.md` | Changing Newton-Raphson iteration or Jacobian |
| `VOLTAGE_LIMITING.md` | Changing NR voltage limiting (pnjlim/fetlim), convergence, damping |
| `DC_OP.md` | Changing DC operating point solver or bias initialization |
| `DEVICE_MODELS.md` | Changing diode/BJT/JFET/MOSFET/tube/opamp/VCA equations |
| `GUMMEL_POON.md` | Changing Gummel-Poon BJT model, qb() function, GP Jacobian |
| `LINEAR_ALGEBRA.md` | Changing matrix inversion, LU decomposition, Gaussian elimination |
| `SHERMAN_MORRISON.md` | Changing dynamic potentiometers, rank-1 updates, SM vectors |
| `DYNAMIC_PARAMS.md` | Changing `.pot`/`.switch`/`.wiper` directives, parameter pipeline |
| `OPAMP_RAIL_MODES.md` | Changing op-amp rail clamping (Hard/ActiveSet/ActiveSetBe/BoyleDiodes) |
| `OVERSAMPLING.md` | Changing oversampling, anti-alias filters, polyphase half-band IIR |
| `CODEGEN.md` | Changing generated solver code structure or templates |
| `COMPANION_MODELS.md` | Changing trapezoidal integration or companion circuits |
| `SIGNAL_LEVELS.md` | Changing signal levels, DC blocking, output scaling |
| `NOISE.md` | Changing thermal/shot/1f noise generation |
| `DEBUGGING.md` | **Diagnosing solver output issues, known failure signatures, historical fix catalog** |
| `SPICE_VALIDATION.md` | Running or modifying SPICE validation tests |
| `STATUS.md` | **Feature inventory, device support, solver routing, validated circuits, pending work** |
| `TUNGSTEN_MATERIAL.md` | Modeling tungsten-based components (Schottky diodes, cat's whisker, WSe2/WS2) |

## Critical Sign Conventions

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

## Device Dimensions in NR

- Diodes: 1D per device (single voltage/current)
- BJTs: 2D per device (Vbe→Ic at start_idx, Vbc→Ib at start_idx+1)
- JFETs/MOSFETs: 2D per device (Vds→Id at start_idx, Vgs→Ig at start_idx+1)
- Triodes: 2D per device (Vgk→Ip at start_idx, Vpk→Ig at start_idx+1)
- Pentodes: 3D per device (Vgk→Ip, Vpk→Ig2, Vg2k→Ig1). 2D when grid-off FA fires (`TubeParams.kind = SharpPentodeGridOff`, auto-detected when Vgk<cutoff, `--tube-grid-fa` override).
- VCAs: 2D per device (Vsig→Isig at start_idx, Vctrl→Ictrl at start_idx+1)
- Op-amps: 0D (linear VCCS model, stamped into G directly)
- Device map built from netlist element order, mirrors MNA builder. Codegen uses `jdev_i_k` naming for block-diagonal Jacobian entries.

## Key Patterns

### Input Modeling (Thevenin Source)
Audio input = voltage source with 1Ω series resistance:
1. Stamp `G_in = 1.0` into `mna.g[input_node][input_node]` **before** building DK kernel.
2. Solver RHS: `(input + input_prev) * input_conductance`.
3. DK kernel bakes G into S = A⁻¹, so the stamp MUST happen before `DkKernel::from_mna()`.

Too-high R_in (e.g. 10k) causes signal attenuation through coupling caps.

### DC Voltage Sources (e.g. VCC)
Norton equivalent: G = 1e6 S stamped in MNA G, current `2 * V * G` in rhs_const.

### Real-Time Safety
No alloc/locks/syscalls in audio processing. All buffers pre-allocated in solver constructors.

## SPICE Validation (quick reference)

Tests compare melange output against ngspice (`crates/melange-validate/`). See `docs/aidocs/SPICE_VALIDATION.md` for full protocol and `STATUS.md` for the latest correlation table.

- `circuit.cir` — full SPICE netlist (with VIN source) for ngspice.
- `circuit_no_vin.cir` — same circuit minus VIN, for melange (input via conductance stamping).
- `.OPTIONS INTERP` added automatically for uniform timestep output (must come AFTER title line).

### Most common SPICE-validation footguns

| Symptom | Fix |
|---------|-----|
| Correlation ~ 0 | Stamp `mna.g[in][in] += G_in` **before** `DkKernel::from_mna()` |
| ~3% error on linear circuit | Using `2*V*G` instead of `(V+V_prev)*G` — track `input_prev`, use proper trapezoidal |
| Sample count mismatch | Add `.OPTIONS INTERP` (after title line!) |
| Output all zeros | VIN left in `circuit_no_vin.cir` |
| Output ±10V oscillating | Purely resistive nonlinear circuit — parasitic 10pF caps should auto-insert, or add explicit caps |
| BJT output wrong | DC operating point not initialized — `DC_NL_I` should be non-zero |
| DC OP NR diverges | Wrong Jacobian sign — must be `G_aug = G_dc - N_i·J_dev·N_v` (**subtraction**) |

For the full historical failure catalog (commit-hash-linked fixes, circuit-specific regressions), see `docs/aidocs/DEBUGGING.md` "Historical Failure Signatures".
