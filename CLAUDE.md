# CLAUDE.md

Melange — Rust toolkit for circuit simulation to real-time audio DSP.

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
  melange-devices/      # Layer 2: Component models (diode, BJT, JFET, MOSFET, tube, opamp, LDR)
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
| `docs/aidocs/DEVICE_MODELS.md` | Changing diode/BJT/JFET/MOSFET/tube/opamp equations |
| `docs/aidocs/GUMMEL_POON.md` | Changing Gummel-Poon BJT model, qb() function, GP Jacobian |
| `docs/aidocs/LINEAR_ALGEBRA.md` | Changing matrix inversion, LU decomposition, Gaussian elimination |
| `docs/aidocs/SHERMAN_MORRISON.md` | Changing dynamic potentiometers, rank-1 updates, SM vectors |
| `docs/aidocs/OVERSAMPLING.md` | Changing oversampling, anti-alias filters, polyphase half-band IIR |
| `docs/aidocs/CODEGEN.md` | Changing generated solver code structure or templates |
| `docs/aidocs/COMPANION_MODELS.md` | Changing trapezoidal integration or companion circuits |
| `docs/aidocs/SIGNAL_LEVELS.md` | Changing signal levels, DC blocking, output scaling |
| `docs/aidocs/DEBUGGING.md` | Diagnosing solver output issues, known failure signatures |
| `docs/aidocs/SPICE_VALIDATION.md` | Running or modifying SPICE validation tests |

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
- JFETs: 2D per device (Vgs→Id at start_idx, Vds→Ig at start_idx+1)
- MOSFETs: 2D per device (Vgs→Id at start_idx, Vds→Ig at start_idx+1)
- Tubes: 2D per device (Vgk→Ip at start_idx, Vpk→Ig at start_idx+1)
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
| NodalSolver wrong A_neg | Inductor rows zeroed | Only zero VS/VCVS rows (n_nodes..n_aug), NOT inductor rows (n_aug..n_nodal) |

## Current Status (2026-03-15)

### Working
- Linear circuit simulation (RC lowpass matches ngspice to 0.03% RMS, 8-nines correlation)
- MNA stamping for R, C, L, voltage sources, current sources, diodes, BJTs, JFETs, MOSFETs, tubes, op-amps
- DK kernel build with proper trapezoidal discretization
- NR solver: 1D, 2D (two 1D devices), and M-dimensional
- Codegen for diode, BJT, JFET, and tube/triode circuits (up to M=16, Gaussian elimination for M=3..16)
- Codegen uses proper trapezoidal RHS with `input_prev` tracking (matches runtime solver)
- Per-device `.model` params: each device gets its own IS, N, BF, BR, VT (heterogeneous models supported)
- Nonlinear DC operating point solver (LU decomposition with partial pivoting, logarithmic junction-aware voltage limiting, source stepping and Gmin stepping fallbacks)
- DC OP internal nodes for parasitic BJTs: basePrime/colPrime/emitPrime nodes (like ngspice) for correct convergence with RB/RC/RE
- DC OP integrated into codegen (`DC_NL_I` constant) and runtime (`initialize_dc_op()`)
- DC operating point calculation includes input conductance
- Plugin template generates per-channel state for stereo (no cross-channel corruption)
- SPICE validation infrastructure with ngspice
- Input validation: parser rejects negative/zero/NaN/Inf component values, self-connected components (including V/I sources), warns on unknown directives, errors on missing `.ends`; codegen validates node indices
- Error types are enums (`MnaError`, `DkError`, `CodegenError` with `InvalidConfig`, `SolverError`) — no panicking library code
- Logging via `log` crate (no `eprintln!` in library code)
- MAX_M=16 bound prevents unbounded allocation in DK kernel
- CLI reports errors for unresolved node names (no silent defaults)
- **BJT common-emitter amplifier**: SPICE validation passes (correlation 0.965, 35% RMS)
  - Trapezoidal nonlinear integration: correction uses full `S*N_i*i_nl` (not delta)
  - Combined with `N_i*i_nl_prev` in RHS, gives proper trapezoidal average
  - **Gummel-Poon model**: `BjtParams` includes VAF, VAR, IKF, IKR; `bjt_qb()` base charge modulation
  - USE_GP flag auto-detected from params; falls back to Ebers-Moll when GP params are infinite
  - **Junction capacitances**: CJE/CJC (BJT), CGS/CGD (JFET/MOSFET), CJO (diode), CCG/CGP/CCP (tube). Parsed from .model, stamped into MNA C matrix.
  - **Parasitic resistances**: RS (diode, inner NR), RB/RC/RE (BJT, 2D inner NR), RD/RS (JFET/MOSFET, closed-form), RGI (tube, 1D inner NR)
  - **BJT NF/ISE/NE**: Forward emission coefficient and base-emitter leakage current
  - **Diode BV/IBV**: Reverse breakdown (Zener) support
  - **MOSFET GAMMA/PHI**: Body effect (threshold voltage shift with source-bulk voltage)
  - **Self-heating**: NOT IMPLEMENTED (planned future feature)
- **Dynamic potentiometers**: `.pot R1 min max` directive marks a resistor as runtime-variable
  - Sherman-Morrison rank-1 updates: O(N²) correction instead of O(N³) re-inversion
  - Precomputed SM vectors (SU, USU, NV_SU, U_NI) baked into generated constants
  - Corrections applied to S, K, A_neg, and S*N_i products in codegen
  - **Runtime solver**: No pot support (codegen-only). `CircuitSolver` uses nominal kernel.
  - Plugin template auto-generates `FloatParam` knobs for each pot
  - Max 32 pots per circuit; pot value stored in `CircuitState`
- **Plugin level params always included**: Input Level (0 dB default, -36 to +12 dB) and Output Level (0 dB default, -60 to +12 dB)
  - Use `--no-level-params` CLI flag to opt out
- **Oversampling in codegen** (2x/4x): self-contained polyphase half-band IIR in generated code
  - `CodegenConfig.oversampling_factor` = 1, 2, or 4
  - All matrices recomputed at internal rate (sample_rate * factor) from G+C
  - No runtime crate dependencies — filter coefficients + allpass structure emitted inline
  - 4x uses cascaded 2x: outer 2-section (~60dB) + inner 3-section (~80dB)
- **Sparsity-aware emission**: systematic zero-skipping in A_neg, N_v, K, S*N_i multiplications
- **Runtime sample rate**: `set_sample_rate()` recomputes all matrices from G+C at runtime
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
- **NodalSolver codegen** (fallback for multi-transformer): Full N-dim NR per sample. LU solve with partial pivoting, SPICE pnjlim/fetlim, ngspice node damping, RELTOL convergence, BE fallback. Auto-selected for circuits with 2+ transformer groups (DK K matrix unstable for inter-transformer coupling).
- **`melange analyze` with --pot/--switch**: Set pot values and switch positions for frequency sweeps. Auto-applies .pot defaults. NodalSolver support for inductor circuits. 5-second minimum settle time for transformer circuits.
- **Device model features**: Junction capacitances (CCG/CGP/CCP, CJE/CJC, CGS/CGD, CJO), parasitic resistances (RS, RB/RC/RE, RD/RS, RGI), diode breakdown (BV/IBV), MOSFET body effect (GAMMA/PHI), BJT NF/ISE/NE emission params

### Known Limitations
- Parasitic caps (10pF) auto-inserted across junctions for purely resistive nonlinear circuits
- Tube Koren model: lambda parameter models finite plate resistance; no space-charge or transit-time effects
- BJT Gummel-Poon: self-heating (Rth/Cth) and charge storage (CJE/CJC/TF) available; no substrate current or avalanche breakdown
- **Runtime solver** (`DeviceEntry`) supports all device types: Diode, DiodeWithRs, Led, BJT, JFET, MOSFET, Tube
- **NodalSolver transient NR**: Converges for all physically valid circuits including Pultec EQP-1A (4 tubes, 2 transformers, global NFB). Requires positive-definite inductance matrices (validated at MNA build time).
- **`melange simulate`**: auto-selects NodalSolver for nonlinear circuits with inductors, CircuitSolver (DK) otherwise. `--solver nodal|dk` override available.
- **Performance**: DK codegen circuits run 100-600× realtime. Nodal codegen (multi-transformer): ~0.6× realtime for Pultec (41-node, 8 NL). Schur complement reduction planned for multi-transformer realtime.

### Cross-Compilation (macOS from Linux)
- Zig 0.13 + cargo-zigbuild + macOS SDK 13.3 + rcodesign (ad-hoc signing)
- `cargo zigbuild --release --target universal2-apple-darwin` produces universal Mac binaries
- Tested: rc-lowpass, pultec-eq, wurli-preamp all compile + sign successfully
- melange-cli does NOT cross-compile (ureq/dirs need CoreFoundation), but generated plugins do

### Validated Circuits
- `circuits/pultec-eq.cir`: Pultec EQP-1A (4 tubes: 2×12AX7 + 2×12AU7, 2 transformers, 3 switches, 7 pots, global NFB)
  - Verified against Sowter DWG E-72,658-2 schematic + Peerless/Triad winding data (2026-03-18)
  - HS-29: 1:2 step-up, 37H, true push-pull grid drive (both grids from CT secondary)
  - S-217-D: 220H primary (30Hz), 71-turn tertiary feedback, 220pF 12AU7 grid stabilization
  - Uses NodalSolver codegen path (2 transformer groups → DK K matrix unstable)
  - Gain: +21-27 dB (amp) - 23 dB (EQ) = -2 to +4 dB net (target: 0 dB)
  - Not realtime (0.6×) — needs Schur complement optimization
- `circuits/wurli-preamp.cir`: Wurlitzer 200A preamp (N=11, M=5→3 FA, 2 BJTs + 1 diode, 1 pot)
  - Flattened from openwurli/spice/subcircuits/preamp.cir
  - R1-Cin series input coupling via intermediate node (mid_in)
  - 0.1V in → 9.12V peak out at R_ldr=100K nominal
- `circuits/wurli-power-amp.cir`: Wurlitzer 200A power amplifier (N=20, M=16→9 FA, 8 BJTs, Class AB)
  - Quasi-complementary push-pull: PNP diff pair → NPN VAS → Sziklai output pairs
  - DC OP: v(out)=-0.065V (ngspice: -0.063V), internal nodes for parasitic BJTs (RB up to 120Ω)
  - FA detection: 7/8 BJTs forward-active (Q9 Vbe multiplier excluded — near saturation)
  - DK codegen M=9: stable, 0.4× realtime (optimized). Nodal codegen: stable, 0.04× realtime.
- `circuits/tweed-preamp.cir`: Fender-style 2-stage 12AX7 guitar amp (N=13, M=4, 1 pot, 1 switch)
  - Two 12AX7 triode stages, interstage volume pot (shunt, 500Ω-500kΩ), bright switch (3-pos)
  - Stage 1: fully bypassed cathode (25µF, max gain); Stage 2: partial bypass (0.68µF, ~156Hz)
  - Output pad (390k/6.8k, -35dB) models output transformer attenuation
  - 50mV in → 549mV out (11x / 20.8dB gain), zero NR divergence

### Pending Work

#### Current Priority — Release Prep
- **Target circuits**: Guitar pedals (diode clippers, tube screamers), simple tube amps, OpenWurli
- **SPICE validation**: tube+transformer circuits against ngspice
- **Validate wurli-preamp**: SPICE comparison, gain at R_ldr extremes, frequency response
- **Documentation**: User-facing docs, example circuits, getting-started guide

#### Performance — Schur Complement for Multi-Transformer
- Reduce N×N NR to M×M for nodal path (same as DK but handling multi-transformer coupling)
- Would bring Pultec from 0.6× to ~8× realtime
- Jacobian reuse (chord method): additional 2-3× on top
- Fast exp() approximation: 15-25% free speedup

#### Future — Multi-Language Codegen
The `Emitter` trait + `CircuitIR` are language-agnostic by design. Once Rust output is complete, planned targets in priority order: C++ (pro plugin devs), FAUST (compiles to 30+ targets), Python/NumPy (prototyping), MATLAB/Octave (academic).

#### Deferred
- **Phase 6a/6b** (type safety): NodeIdx newtype and field visibility
- **Phase 7** (crate split): Extract melange-parser, melange-codegen
- **M>16**: Iterative/sparse NR for very large nonlinear systems
