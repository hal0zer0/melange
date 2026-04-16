# CLAUDE.md

Melange — Rust toolkit for circuit simulation to real-time audio DSP.

## Oomox Ecosystem — Division of Labor

Four repos, one pipeline. Each project owns exactly one stage.

```
Real hardware → SCHEMER → MELANGE-CIRCUITS → MELANGE → OOMOX → Product plugin
  (schematics)   (netlists)    (.cir files)     (Rust code)  (binary VST/CLAP)
```

**SCHEMER** (`~/dev/schemer`) — Local-only toolkit. Reads real-world hardware schematics (raster images, PDFs, service manuals) and assists with extracting netlists. Not a product. Not public.

**MELANGE-CIRCUITS** (`~/dev/melange-circuits`) — All non-test circuit netlists for melange and Oomox. Not currently on a public git host. Agent is expert at designing and drafting circuits specifically for melange.

**MELANGE** (`~/dev/melange`) — **This repo.** Core product. GPL-3.0, publicly released. Compiles SPICE circuit netlists to optimized Rust DSP code. The entire circuit-to-code pipeline: parser, MNA, solvers, codegen. Exposes parameters in a thin VST wrapper (`--format plugin`). Generated code is GPL.

**OOMOX** (`~/dev/oomox`) — Non-product software for building binary product plugins. Handles ALL DSP and UX beyond the generated circuit file: pot curve shaping, parameter smoothing, warmup, output scaling, UI toolkit, editor layout, presets.

### Boundary Rules

- **Circuit sounds wrong** → fix in MELANGE (solver/models) or MELANGE-CIRCUITS (component values). Never add workarounds in OOMOX.
- **Plugin behavior wrong** (clicks, CPU, knob feel, UI) → fix in OOMOX. Never change the `.cir` to work around a plugin issue.
- **Need a new circuit** → draft in MELANGE-CIRCUITS. Use SCHEMER if starting from real hardware.
- **Pot mapping / indices** → defined by the `.cir` (MELANGE-CIRCUITS), assigned by the compiler (MELANGE), wired by the plugin (OOMOX). All three must agree.

## CRITICAL: Accuracy beats plugin usability 1000x

Accurately modeling a circuit is **1000x more important** than quickly building a working plugin. Melange is a "build whatever circuit you throw at me" tool, NOT a "tweak the circuit to do what I want" tool.

When a user reports a plugin sounds wrong (clipping, blown speaker, distorted, too hot, too quiet):
- **The bug is in melange** (solver, codegen, device models) — assume this until proven otherwise.
- **Never add workaround components** to user netlists (catch diodes, padding resistors, snubbers) even if the real hardware has equivalents. If the user didn't put it in the netlist, the fix belongs in melange's MNA/device layer.
- **Never modify component values** in circuits to dodge solver edge cases — fix the solver.
- **Never suggest "turn the knobs down"** as a fix — that's avoiding the buggy region, not fixing it.
- **Never use `--output-scale` or output attenuation** as a remedy for simulation errors — it's a voltage→DAW-unit mapping, not a bug fix.
- **Performance, ergonomics, and plugin polish are always secondary** to schematic fidelity. A slow accurate solver beats a fast wrong one.

## CRITICAL: Use CLI flags when two circuits need conflicting behavior

When fixing one circuit regresses another, don't pick the lesser evil — expose both behaviors via a CLI flag named after the mechanism (not the circuit). Existing examples: `--solver {auto|dk|nodal}`, `--backward-euler`, `--oversampling {1|2|4}`, `--tube-grid-fa {auto|on|off}`. Each circuit opts into the mode that serves it. Default to the option with the broadest safety envelope. Flags are *not* a license to ship broken modes — every selectable mode has to be correct for some class of circuits.

## CRITICAL: Never simplify circuits

The goal is ALWAYS to get melange to run the real circuit, NEVER to simplify a circuit to work around melange limitations. If a real hardware topology doesn't work in melange, that is a melange bug — fix melange. Do not substitute "simplified approximations" of real circuit sections.

## Build & Test

```bash
cargo build --workspace
cargo test --workspace
cargo test -p melange-solver          # core solver tests
cargo test -p melange-validate --test spice_validation -- --include-ignored  # SPICE comparison (needs ngspice)
cargo run -p melange-cli
```

## Project Structure

```
crates/
  melange-primitives/   # Layer 1: DSP building blocks (filters, NR helpers, oversampling)
  melange-devices/      # Layer 2: Component models (diode, BJT, JFET, MOSFET, tube, opamp, VCA, LDR)
  melange-solver/       # Layer 3: MNA/DK solver + codegen (CORE)
  melange-validate/     # Layer 4: SPICE validation against ngspice
  melange-plugin/       # Layer 5: nih-plug integration (stub)
tools/
  melange-cli/          # CLI tool
docs/aidocs/            # Detailed math reference docs (READ THESE before changing core math)
```

## Agent Research Library — `docs/aidocs/`

**MANDATORY**: Before modifying any solver math, matrix operations, device models, or codegen,
read the relevant doc(s) from `docs/aidocs/`. These are optimized for AI agent consumption —
dense equations, code patterns, and cross-references. The full index is at `docs/aidocs/INDEX.md`.

| Doc | When to read |
|-----|-------------|
| `docs/aidocs/MNA.md` | Changing MNA stamping (G, C, N_v, N_i), any component stamping rules |
| `docs/aidocs/DK_METHOD.md` | Changing S, K, A matrices or kernel build |
| `docs/aidocs/NR_SOLVER.md` | Changing Newton-Raphson iteration or Jacobian |
| `docs/aidocs/VOLTAGE_LIMITING.md` | Changing NR voltage limiting (pnjlim/fetlim), convergence, damping |
| `docs/aidocs/DC_OP.md` | Changing DC operating point solver or bias initialization |
| `docs/aidocs/DEVICE_MODELS.md` | Changing diode/BJT/JFET/MOSFET/tube/opamp/VCA equations |
| `docs/aidocs/GUMMEL_POON.md` | Changing Gummel-Poon BJT model, qb() function, GP Jacobian |
| `docs/aidocs/LINEAR_ALGEBRA.md` | Changing matrix inversion, LU decomposition, Gaussian elimination |
| `docs/aidocs/SHERMAN_MORRISON.md` | Changing dynamic potentiometers, rank-1 updates, SM vectors |
| `docs/aidocs/DYNAMIC_PARAMS.md` | Changing .pot/.switch/.wiper directives, parameter pipeline |
| `docs/aidocs/OPAMP_RAIL_MODES.md` | Changing op-amp rail clamping (Hard/ActiveSet/ActiveSetBe/BoyleDiodes) |
| `docs/aidocs/OVERSAMPLING.md` | Changing oversampling, anti-alias filters, polyphase half-band IIR |
| `docs/aidocs/CODEGEN.md` | Changing generated solver code structure or templates |
| `docs/aidocs/COMPANION_MODELS.md` | Changing trapezoidal integration or companion circuits |
| `docs/aidocs/SIGNAL_LEVELS.md` | Changing signal levels, DC blocking, output scaling |
| `docs/aidocs/DEBUGGING.md` | Diagnosing solver output issues, known failure signatures |
| `docs/aidocs/SPICE_VALIDATION.md` | Running or modifying SPICE validation tests |
| `docs/aidocs/STATUS.md` | Checking validation results, device support, solver routing, circuit status |
| `docs/aidocs/TUNGSTEN_MATERIAL.md` | Modeling tungsten-based components (Schottky diodes, cat's whisker, WSe2/WS2 devices) |

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
- JFETs: 2D per device (Vds→Id at start_idx, Vgs→Ig at start_idx+1)
- MOSFETs: 2D per device (Vds→Id at start_idx, Vgs→Ig at start_idx+1)
- Triodes: 2D per device (Vgk→Ip at start_idx, Vpk→Ig at start_idx+1)
- Pentodes: 3D per device (Vgk→Ip at start_idx, Vpk→Ig2 at start_idx+1, Vg2k→Ig1 at start_idx+2). Reefman Derk §4.4 / DerkE §4.5 / Classical Koren math (per `ScreenForm`); `TubeParams.kind = SharpPentode` dispatches the 3D codepath.
- Pentodes (grid-off, phase 1b): 2D per device (Vgk→Ip, Vpk→Ig2, Ig1 dropped, Vg2k frozen at DC-OP value). Auto-detected when Vgk<cutoff. `TubeParams.kind = SharpPentodeGridOff` dispatches the 2D codepath. `DeviceSlot.vg2k_frozen` holds the DC-OP screen voltage. `--tube-grid-fa {auto,on,off}` CLI override.
- VCAs: 2D per device (Vsig→Isig at start_idx, Vctrl→Ictrl at start_idx+1)
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
| Output ±10V oscillating | Purely resistive circuit, no caps | Auto-inserted 10pF parasitic caps (or add explicit caps) |
| BJT output wrong | No DC operating point | Call `initialize_dc_op()` or use `DC_NL_I` codegen constant |
| BJT period-3 oscillation | Backward Euler for nonlinear currents | Fixed: use full i_nl in correction (not delta) |
| DC OP NR diverges | Wrong Jacobian sign | Use `G_aug = G_dc - N_i·J_dev·N_v` (subtraction!) |
| DC OP degenerate (all BJTs off) | Parasitic R (RB/RC/RE) in inner loop | Internal nodes in DC system (basePrime/colPrime/emitPrime) |
| NodalSolver NaN after DC OP | Inductor currents not initialized | Copy full v_node (incl. inductor branch currents) from DC OP |
| NodalSolver wrong A_neg | Inductor rows zeroed | Zero n_nodes..n_aug (all augmented), NOT n_aug..n_nodal (inductor branches) |
| Codegen diverges ~5000 samples | Boyle A_neg trapezoidal instability | A_neg must zero ALL augmented rows (Gm ±2000 creates spectral radius > 1) |
| Codegen stable but wrong level | K≈0, Schur NR has J=I (no damping) | Route K≈0 circuits to full N×N LU NR (device Jacobian in G_aug) |
| BoyleDiodes augmented system: input row zero, first signal sample explodes | `MnaSystem::from_netlist(&augmented)` doesn't preserve in-place G_in stamp | Re-stamp `g[input][input] += 1/R_in` and junction caps after rebuild in `generate_nodal` (FIXED commit `5544c8a`) |
| BoyleDiodes false convergence: trap NR declares converged with wildly wrong v[buf_in] on first signal sample | Voltage-step convergence check passes when chord_j_dev is many OOM stale; no residual gate on full-LU NR loop | Add residual check + adaptive refactor trigger (>50% j_dev change), both BoyleDiodes-gated (FIXED commit `39397d1`) |
| BoyleDiodes heavy clipping (amp ≥ 0.05 on Klon): NR diverges, raw output 45–3068 V | Chord-LU Newton direction wrong (not just magnitude). Gmin/line-search can't fix wrong direction. | **Not a blocker.** Auto-routes to `active-set-be` (verified correct amp=[0.01..0.50]). BoyleDiodes opt-in only (`--opamp-rail-mode boyle-diodes`). See `docs/aidocs/DEBUGGING.md`. |
| DK kernel singular at col N (transistor ladder / cap-only nodes) | Intermediate nodes connected only through BJT junctions + bridging caps have G≈Gmin (1e-12). A = G+2C/T nearly singular in left/right subspace. | Routes to nodal automatically. DK limitation for series-BJT topologies without parallel resistors. |
| Nodal Schur flat output (+9 dB, no filtering) on BJT ladder | S = A⁻¹ has extreme entries (>1e6) at cap-only nodes → K entries span 10^11 → J = I-J_dev·K swamped. | S magnitude check: `max\|S\| > 1e6` routes to full LU NR. Invariant to FA reduction. (FIXED 2026-04-10) |
| simulate/analyze crash "Augmented fallback failed" | DK fails, augmented DK also fails, no nodal fallback | Dummy kernel with dk_failed=true, falls through to nodal codegen (FIXED 2026-04-10) |
| BJT diverges from ngspice at NF ≠ 1 or high injection | q2 used bare VT and omitted `-1`; Ib forward was divided by qb | q2 uses `cbe/IKF + cbc/IKR` where `cbe = IS*(exp(Vbe/(NF*VT))-1)`; Ib ideal forward NOT divided by qb. Matches `bjtload.c:571,618` (FIXED commit `d7427c4`) |
| Parser panics on non-ASCII component value (e.g. `1ſ`) | `to_uppercase()` changes byte length; byte-slice landed mid-codepoint | `parse_value()` normalizes non-ASCII at entry (µ/μ → u; else `Err`) (FIXED commit `e96c340`) |
| Click on every pot/switch move in generated plugin | `rebuild_matrices()` zeroed DC blocker + oversampler state on pot change | Removed filter state resets from DK Schur `rebuild_matrices` (FIXED commit `07712a0`) |
| Zipper noise on knob automation | Pot values read once per buffer via `.value()`, smoother declared but unused | Per-sample `.smoothed.next()` read in plugin template (FIXED commit `7c0fc02`) |
| NR max-iter-cap hits on wide-range pot jumps (preset recall, automation step) | Stale `v_prev`/`i_nl_prev` from previous (different) operating point; NR starts far from new bias | Warm DC-OP re-init in `set_pot_N`/`set_switch_N` when `|r - r_prev|/r_prev > 0.20` (FIXED commit `e8e18a7`). The earlier Sherman-Morrison removal (commit `eaee955`) was treating a symptom — SM is mathematically exact; stale DC-OP seed was the root cause. |
| DC OP wrong polarity for precision rectifier op-amps (e.g., 4kbuscomp sidechain TL074 at +11V instead of -11V) | Multi-equilibrium: AOL=200K overshoots NR from one rail to the other in one iteration. Linear initial guess finds wrong self-consistent equilibrium. | AOL capped at 1000 in DC G matrix (`build_dc_system`), op-amp output seeding (`seed_opamp_outputs`), per-iteration rail clamp in DC OP NR. (FIXED 2026-04-15) |
| DC OP diode reverse bias: nodes float to non-physical voltages (91kV) when diodes off | DC OP `evaluate_devices_inner()` ignored BV/IBV and had zero reverse-bias conductance | BV/IBV breakdown added to DC OP diode evaluation; device-level Gmin (1e-12 S) added as minimum junction conductance. (FIXED 2026-04-15) |
| Precision rectifier transient divergence at signal levels causing diode switching | Chord-LU NR direction wrong when device state changes (diode on→off). Same class as Klon BoyleDiodes heavy-clip. | Per-iteration op-amp rail clamp in both trapezoidal and BE NR loops. Stabilizes idle + low-level (amp≤0.01). Diode-switching levels (amp>0.02) still diverge. (PARTIAL 2026-04-15) |

## Current Status (2026-04-11)

### Working
- Linear circuit simulation (RC lowpass matches ngspice to 0.03% RMS, 8-nines correlation)
- MNA stamping for R, C, L, voltage sources, current sources, diodes, BJTs, JFETs, MOSFETs, tubes, op-amps, VCAs
- DK kernel build with proper trapezoidal discretization
- NR solver: 1D, 2D (two 1D devices), and M-dimensional
- Codegen for diode, BJT, JFET, and tube/triode circuits (up to M=16, Gaussian elimination for M=3..16)
- Codegen uses proper trapezoidal RHS with `input_prev` tracking
- Per-device `.model` params: each device gets its own IS, N, BF, BR, VT (heterogeneous models supported)
- Nonlinear DC operating point solver (LU decomposition with partial pivoting, logarithmic junction-aware voltage limiting, source stepping and Gmin stepping fallbacks)
- DC OP internal nodes for parasitic BJTs: basePrime/colPrime/emitPrime nodes (like ngspice) for correct convergence with RB/RC/RE
- DC OP integrated into codegen (`DC_NL_I` constant)
- DC operating point calculation includes input conductance
- Plugin template generates per-channel state for stereo (no cross-channel corruption)
- SPICE validation infrastructure with ngspice
- Input validation: parser rejects negative/zero/NaN/Inf component values, self-connected components (including V/I sources), warns on unknown directives, errors on missing `.ends`; codegen validates node indices
- Error types are `#[non_exhaustive]` enums (`MnaError`, `DkError`, `CodegenError`, `SolverError`) — no panicking library code
- `MAX_M` (=16) re-exported from `melange-solver` public API
- Logging via `log` crate (no `eprintln!` in library code)
- MAX_M=16 bound prevents unbounded allocation in DK kernel
- CLI reports errors for unresolved node names (no silent defaults)
- **BJT common-emitter amplifier**: SPICE-validated against ngspice
  - Trapezoidal nonlinear integration: correction uses full `S*N_i*i_nl` (not delta)
  - Combined with `N_i*i_nl_prev` in RHS, gives proper trapezoidal average
  - **Gummel-Poon model**: matches ngspice `bjtload.c` line-for-line (q2 uses `cbe/IKF + cbc/IKR` with `cbe = IS*(exp(Vbe/(NF*VT))-1)` — NOT bare VT; Ib ideal forward is NOT divided by qb — GP modulates only the transport current). `BjtParams` includes VAF, VAR, IKF, IKR; `bjt_qb()` base charge modulation
  - Q1 Early effect guard: `q1_denom <= 0` clamps to 1.0 (prevents sign-flip near Early voltage)
  - USE_GP flag auto-detected from params; falls back to Ebers-Moll when GP params are infinite
  - **Junction capacitances**: CJE/CJC (BJT), CGS/CGD (JFET/MOSFET), CJO (diode), CCG/CGP/CCP (tube). Parsed from .model, stamped into MNA C matrix.
  - **Parasitic resistances**: RS (diode, inner NR), RB/RC/RE (BJT, 2D inner NR), RD/RS (JFET/MOSFET, closed-form), RGI (tube, 1D inner NR)
  - **BJT NF/ISE/NE**: Forward emission coefficient and base-emitter leakage current
  - **Diode BV/IBV**: Reverse breakdown (Zener) support
  - **MOSFET GAMMA/PHI**: Body effect (threshold voltage shift with source-bulk voltage)
  - **Self-heating**: BJT RTH/CTH/TAMB supported in codegen (default RTH=infinity disables)
- **Dynamic potentiometers**: `.pot R1 min max` directive marks a resistor as runtime-variable
  - **Per-block O(N³) rebuild** on pot changes (Sherman-Morrison was removed; research confirmed SM and rebuild produce identical K', and rebuild is ~25× cheaper for N≤15 DK circuits)
  - **Per-sample smoothing via `.smoothed.next()`**: plugin template reads smoothed pot values inside the process loop (the `SmoothingStyle::Linear(10.0)` at `plugin_template.rs:437` is actually consumed per sample)
  - **Warm DC-OP re-init on large pot jumps**: `set_pot_N` resets `v_prev`/`i_nl_prev` to `DC_OP`/`DC_NL_I` when `|r - r_prev| / r_prev > 0.20`. Fixes NR max-iter-cap hits on preset-recall step changes without clicking on smoothed sweeps.
  - Plugin template auto-generates `FloatParam` knobs for each pot
  - Max 64 pots per circuit; pot value stored in `CircuitState`
- **Gang pot linking**: `.gang "Label" member1 member2` links multiple `.pot`/`.wiper` entries to a single UI parameter
  - Single 0-1 position controls all members; `!` prefix inverts a member's response
  - Plugin template emits one FloatParam per gang; ganged pots/wipers excluded from individual params
- **Op-amp VCC/VEE asymmetric supply rails**: `.model OA(VCC=9 VEE=0)` for single-supply, `VCC=18 VEE=-9` for charge pump
  - Priority: VCC/VEE > VSAT > GBW auto-default (±13V). Clamping in all codegen paths (DK, Nodal Schur, Full LU).
- **Plugin level params always included**: Input Level (0 dB default, ±24 dB) and Output Level (0 dB default, ±24 dB)
  - Use `--no-level-params` CLI flag to opt out
- **Oversampling in codegen** (2x/4x): self-contained polyphase half-band IIR in generated code
  - `CodegenConfig.oversampling_factor` = 1, 2, or 4
  - All matrices recomputed at internal rate (sample_rate * factor) from G+C
  - No runtime crate dependencies — filter coefficients + allpass structure emitted inline
  - 4x uses cascaded 2x: outer 2-section (~60dB) + inner 3-section (~80dB)
- **Sparsity-aware emission**: systematic zero-skipping in A_neg, N_v, K, S*N_i multiplications
- **Runtime sample rate**: `set_sample_rate()` recomputes all matrices from G+C in generated code
- **JFET codegen**: 2D Shichman-Hodges model (triode + saturation regions, channel-length modulation)
  - `jfet_id(vgs, vds, idss, vp, lambda, sign)`, `jfet_ig(vgs, sign)`, `jfet_jacobian()` → [f64; 4]
  - Constants: `DEVICE_{n}_IDSS`, `DEVICE_{n}_VP`, `DEVICE_{n}_LAMBDA`, `DEVICE_{n}_SIGN`
  - N-channel (NJ) defaults: VTO=-2.0, IDSS=2e-3; P-channel (PJ) defaults: VTO=+2.0
  - Compile-and-run verified for both N-channel and P-channel circuits
- **Tube/triode codegen**: 2D Koren plate current model with Leach grid current
  - `tube_ip()` (Koren), `tube_ig()` (Leach power-law), `tube_jacobian()` (4-element)
  - Early-effect lambda: `Ip = Ip_koren * (1 + lambda * Vpk)` for finite plate resistance (default lambda=0.0)
  - Constants: `DEVICE_{n}_MU`, `DEVICE_{n}_EX`, `DEVICE_{n}_KG1`, `DEVICE_{n}_KP`, `DEVICE_{n}_KVB`, `DEVICE_{n}_LAMBDA`, `DEVICE_{n}_IG_MAX`, `DEVICE_{n}_VGK_ONSET`
  - Template: `device_tube.rs.tera`; 6 tests (parser, MNA, codegen, compile-and-run, two-triode preamp)
- **MOSFET codegen**: 2D Level 1 SPICE with triode + saturation + channel-length modulation (LAMBDA)
  - `mosfet_id(vgs, vds, kp, vt, lambda, sign)`, `mosfet_ig(vgs, sign)`, `mosfet_jacobian()` -> [f64; 4]
  - Constants: `DEVICE_{n}_KP`, `DEVICE_{n}_VT`, `DEVICE_{n}_LAMBDA`, `DEVICE_{n}_SIGN`
  - N-channel (NM) defaults: VTO=2.0, KP=0.1; P-channel (PM) defaults: VTO=-2.0
- **VCA (Voltage-Controlled Amplifier) codegen**: 2D current-mode exponential gain (THAT 2180 / DBX 2150)
  - `vca_current(v_sig, v_ctrl, g0, vscale, thd)`, `vca_jacobian()` → [f64; 4]
  - Gain law: I = G0 * exp(-Vc/Vscale) * (V_sig + thd_factor * V_sig³)
  - THD: gain-dependent cubic nonlinearity (0 at unity, rises with gain reduction)
  - Parameters: VSCALE (V/neper, default 0.05298 for THAT 2180A), G0, THD
  - 4 terminals: sig+, sig-, ctrl+, ctrl- (control port is high-impedance)
  - DK K-diagonal check skips zero-current dimensions (VCA control draws no current)
  - DC OP solver handles VCA quiescent point initialization
- **Multi-output (stereo) support in codegen pipeline**
  - `output_nodes` and `output_scales` in `CodegenConfig` for multi-channel output
- **Subcircuit expansion** (`.subckt` / `X` elements)
- **AC frequency response analysis** (`melange analyze` command)
- **`melange simulate` command**: parse -> MNA -> DK kernel -> solver -> process WAV
  - Supports `--input` for WAV files and `--amplitude` for sine test tones
- **Explicit re-exports**: all crates (melange-solver, melange-devices, melange-primitives) use named re-exports (no glob `pub use module::*`)
- **Parasitic cap auto-insertion**: 10pF across device junctions (not to ground) when nonlinear circuit has no caps; stabilizes trapezoidal rule in `from_mna()`
- **Augmented MNA for inductors**: Each inductor winding adds a branch current variable with L in C matrix (2L/T diagonal, well-conditioned). Replaces companion model (T/(2L) ≈ 8e-8 S for 130H). Supports uncoupled, coupled pairs, and multi-winding transformer groups.
- **DK codegen with augmented MNA**: Single-transformer inductor circuits use the fast DK path (M×M NR). Simple inductor+diode: 581× realtime. Auto-selected when ≤1 transformer group.
- **NodalSolver codegen** (fallback for multi-transformer and K≈0): Two paths:
  - **Schur** (O(N²+M³)): Precomputed S = A^{-1}, M-dim NR. Used when K is well-conditioned.
  - **Full LU** (O(N³) per iter): Builds G_aug = A - N_I*J_dev*N_V each iteration. Auto-selected for K≈0 (current-mode VCA), positive K diagonal, K ill-conditioned (max|K|>1e8), or S ill-conditioned (max|S|>1e6, cap-only nodes). Matches runtime NodalSolver exactly.
  - **Full LU optimizations** (3 stacked):
    1. **Chord method**: `lu_factor` every CHORD_REFACTOR=5 iters (must be odd), `lu_back_solve` (O(N²)) between. RHS uses saved `chord_j_dev` (NOT current `j_dev`).
    2. **Cross-timestep Jacobian persistence**: `chord_lu`/`chord_d`/`chord_j_dev`/`chord_valid` in CircuitState. Most samples need zero factorizations for smooth signals.
    3. **Compile-time sparse LU**: AMD ordering + symbolic factorization at codegen time. Emits straight-line code on ORIGINAL indices (no runtime permutation). `sparse_lu_factor(a, d)` / `sparse_lu_back_solve(a_lu, d, b)`. Pultec: 536 factor FLOPs vs ~22973 dense (43× reduction).
  - Auto-selected for 2+ transformer groups, M≥10, or when DK K diagonal check fails.
- **`melange analyze` with --pot/--switch**: Set pot values and switch positions for frequency sweeps. Auto-applies .pot defaults. NodalSolver support for inductor circuits. 5-second minimum settle time for transformer circuits.
- **Device model features**: Junction capacitances (CCG/CGP/CCP, CJE/CJC, CGS/CGD, CJO), parasitic resistances (RS, RB/RC/RE, RD/RS, RGI), diode breakdown (BV/IBV), MOSFET body effect (GAMMA/PHI), BJT NF/ISE/NE emission params

### Known Limitations
- Parasitic caps (10pF) auto-inserted across junctions for purely resistive nonlinear circuits
- Tube Koren model: lambda parameter models finite plate resistance; no space-charge or transit-time effects
- BJT Gummel-Poon: self-heating (Rth/Cth) and charge storage (CJE/CJC/TF) available; no substrate current or avalanche breakdown
- All device models fixed at room temperature (27°C); no temperature coefficients (TNOM, TC1, TC2, XTI)
- Op-amp model: Boyle macromodel with GBW dominant pole, VCC/VEE asymmetric supply rail clamping, and optional slew-rate limiting via `SR=` in V/μs (per-sample `|Δv_out| ≤ SR·dt` clamp in all 3 codegen paths).
- Pentodes: all 6 phases shipped 2026-04-11 (1a Sharp Rational, 1a.1 Beam Tetrode, 1a.2 Classical Koren, 1b Grid-off FA reduction, 1c Variable-mu, 1e Catalog expansion). 29 catalog entries across 5 equation families. Grid-off (`--tube-grid-fa {auto,on,off}`) reduces pentode 3D→2D when Vgk<cutoff, enabling DK Schur for large amps (e.g., 4xEL34 Plexi: M=18→14).
- **Device support** (`DeviceEntry`): Diode, DiodeWithRs, Led, BJT, JFET, MOSFET, Tube
- **NodalSolver transient NR**: Converges for all physically valid circuits including Pultec EQP-1A (4 tubes, 2 transformers, global NFB). Requires positive-definite inductance matrices (validated at MNA build time).
- **Coupled-inductor push-pull NFB**: Differential cathode injection with separated cathodes (820Ω between) and K=0.9999 provides 21 dB of NFB. Pultec at +1.8 dB (near unity). Ideal transformer formulation (dependent sources + explicit leakage/magnetizing L) deferred — current approach is sufficient.
- **`melange simulate`**: auto-selects codegen path (nodal for inductors, DK otherwise). `--solver nodal|dk` override available. `--oversampling {1,2,4}` for anti-aliasing.
- **Performance**: DK codegen circuits run 100-600× realtime. Nodal full-LU (chord + cross-timestep + sparse LU): ~11× realtime for Pultec (41-node, 8 NL, 2 transformers).
- **Full-LU NR + ill-conditioned A**: The full-LU chord NR can diverge when cond(A) > ~1000 (large coupling caps). Routing prefers Schur when K is well-conditioned, avoiding the issue. No known circuit needs both full-LU (pathological K) AND has ill-conditioned A. See `docs/aidocs/DEBUGGING.md` "Known Full-LU NR Limitations".

### Cross-Compilation (macOS from Linux)
- Zig 0.13 + cargo-zigbuild + macOS SDK 13.3 + rcodesign (ad-hoc signing)
- `cargo zigbuild --release --target universal2-apple-darwin` produces universal Mac binaries
- Tested: rc-lowpass, pultec-eq, wurli-preamp all compile + sign successfully
- melange-cli does NOT cross-compile (ureq/dirs need CoreFoundation), but generated plugins do

### Validated Circuits
Circuits have been migrated to the melange-audio/circuits repo (2026-04-12).
Circuit-specific tests are being migrated as `.test.toml` sidecars.
Historical validation data preserved below for reference.

- Passive tube EQ (passive-eq1a): 4 tubes, 2 transformers, 7 pots, 3 switches, global NFB
  - Verified against Sowter DWG E-72,658-2 schematic + Peerless/Triad winding data (2026-03-18)
  - HS-29: 1:2 step-up, 37H, true push-pull grid drive (both grids from CT secondary)
  - S-217-D: 220H primary (30Hz), 71-turn tertiary feedback, 220pF 12AU7 grid stabilization
  - Uses nodal full-LU codegen path (2 transformer groups → DK K matrix unstable)
  - Gain: +1.8 dB at 1kHz (near unity), flat ±1 dB 20Hz-15kHz, 21 dB differential NFB working
  - All 7 pots + 3 switches produce correct EQ curves (LF Boost/Atten, HF Boost/Cut, Pultec trick)
  - ~11× realtime on full LU with chord + cross-timestep + sparse LU, zero NR failures at 1V
- Wurlitzer 200A preamp (wurli-preamp): N=11, M=5→3 FA, 2 BJTs + 1 diode, 1 pot
  - Flattened from openwurli/spice/subcircuits/preamp.cir
  - R1-Cin series input coupling via intermediate node (mid_in)
  - 2N5089 BJTs run pure Ebers-Moll (USE_GP=false, no GP params in .model)
  - Compile-path output byte-identical to 2026-03-25 baseline (verified via md5 of DC_OP, S_DEFAULT, K_DEFAULT, N_V, N_I, A_NEG_DEFAULT)
- Wurlitzer 200A power amplifier (wurli-power-amp): N=20, M=16→9 FA, 8 BJTs, Class AB
  - Quasi-complementary push-pull: PNP diff pair ��� NPN VAS → Sziklai output pairs
  - DC OP: v(out)=-0.065V (ngspice: -0.063V), internal nodes for parasitic BJTs (RB up to 120Ω)
  - FA detection: 7/8 BJTs forward-active (Q9 Vbe multiplier excluded — near saturation)
  - DK codegen M=9: stable, 0.4× realtime (optimized). Nodal codegen: stable, 0.04× realtime.
- Tweed-style 2-stage 12AX7 preamp (twas-preamp): N=13, M=4, 1 pot, 1 switch
  - Two 12AX7 triode stages, interstage volume pot (shunt, 500Ω-500kΩ), bright switch (3-pos)
  - Stage 1: fully bypassed cathode (25µF, max gain); Stage 2: partial bypass (0.68µF, ~156Hz)
  - Output pad (390k/6.8k, -35dB) models output transformer attenuation
  - 50mV in → 549mV out (11x / 20.8dB gain), zero NR divergence
- Bus compressor (4kbuscomp): 12 op-amps, 2 VCAs, 6 diodes, 2 pots, 2 switches
  - Audio path extract (4kbuscomp-audiopath.cir, M=2): stable, peak=0.001439
  - Full circuit (N=80, M=10): DC OP correct (2026-04-15 BV/IBV + AOL cap + seeding fixes), stable at idle + amp≤0.01
  - Transient diverges at amp>0.02 (sidechain diode switching NR, same class as Klon BoyleDiodes)
  - Uses nodal full-LU path (K≈0 from current-mode VCA)
- EL84 single stage (el84-single-stage): Reefman Derk §4.4 rational screen form
  - DC-OP validated, end-to-end compile-and-run verified
- AC15-class amp (axe-15): EF86 preamp + EL84 power pentodes
  - DC-OP validated, end-to-end compile-and-run verified
- Tweed Deluxe-class amp (twill-deluxe): 6V6GT beam tetrode power stage, DerkE §4.5 exponential screen
  - DC-OP validated, end-to-end compile-and-run verified
- 6K7 variable-mu stage (6k7-varimu-stage): Reefman §5 two-section Koren
  - DC-OP validated, end-to-end compile-and-run verified
- Plexi-class amp (rexi-mockup): 4×EL34 power stage, grid-off FA reduction M=18→14
  - DC-OP validated, end-to-end compile-and-run verified, DK Schur enabled via grid-off

### Shipped (2026)
- Wurli-preamp SPICE validation (6-nines correlation, 3.2% RMS)
- Tube screamer / guitar pedal circuits (stable + testing)
- Klon Centaur (ActiveSetBe auto-route, BoyleDiodes opt-in experimental)
- SSL bus compressor (codegen validated, full-LU path)
- VCR audio ALC compressor
- Op-amp VCC/VEE asymmetric supply rails
- Op-amp slew-rate limiting via `SR=` .model parameter (V/μs)
- Wiper pot support + gang pot linking
- ActiveSetBe precomputed Schur sub-stepping
- BJT Gummel-Poon ngspice parity (q2/Ib match `bjtload.c` exactly)
- Per-sample pot smoothing + warm DC-OP re-init on large pot jumps
- Parser input-size caps + cargo-fuzz target
- Plugin shipability CLI flags (`--vendor`, `--vendor-url`, `--email`, `--vst3-id`, `--clap-id`)
- Pentode support: all 6 phases (1a Sharp Rational, 1a.1 Beam Tetrode, 1a.2 Classical Koren, 1b Grid-off FA, 1c Variable-mu, 1e Catalog expansion). 29 tube models, 5 equation families, `--tube-grid-fa {auto,on,off}` CLI flag

### Pending Work

#### Current Priority
- **Neve 1073**: EQ section (Stage 3), integration (Stage 4), plugin (Stage 5)
- **Oomox plugin roadmap**: .runtime VS, named constants, DC op accessor, warmup, runtime DC OP recompute
- **Documentation**: User-facing docs, example circuits, getting-started guide

#### Performance — Further Optimization
- Pultec at ~11× RT (chord + cross-timestep + sparse LU). Further optimization optional:
- Hot/cold state split: 112KB struct → ~30KB hot path (L1 cache friendly)
- fast_powf for Koren tube model: replace libm powf (~40ns) with fast approximation (~10ns)
- DK parasitic BJTs (power amp 0.41×, K_eff approach planned)

#### Future — Multi-Language Codegen
The `Emitter` trait + `CircuitIR` are language-agnostic by design. Planned targets: C++ (pro plugin devs), FAUST (30+ targets), Python/NumPy (prototyping), MATLAB/Octave (academic).

#### Deferred
- **Ideal transformer formulation**: Dependent sources + explicit leakage/magnetizing L. Pultec at +1.8 dB with current approach, not blocking.
- **Phase 6a/6b** (type safety): NodeIdx newtype and field visibility
- **Phase 7** (crate split): Extract melange-parser, melange-codegen
- **M>16**: Iterative/sparse NR for very large nonlinear systems
- **BoyleDiodes heavy clipping**: Anderson acceleration or BoyleDiodes→ActiveSetBe hybrid (low priority)
