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
- JFETs: 2D per device (Vgs→Id at start_idx, Vds→Ig at start_idx+1)
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
| Output ±10V oscillating | Purely resistive circuit, no caps | Add parasitic capacitance or use backward Euler |
| BJT output wrong | No DC operating point | Call `initialize_dc_op()` or use `DC_NL_I` codegen constant |
| BJT period-3 oscillation | Backward Euler for nonlinear currents | Fixed: use full i_nl in correction (not delta) |
| DC OP NR diverges | Wrong Jacobian sign | Use `G_aug = G_dc - N_i·J_dev·N_v` (subtraction!) |

## Current Status (2026-03-01)

### Working
- Linear circuit simulation (RC lowpass matches ngspice to 0.03% RMS, 8-nines correlation)
- MNA stamping for R, C, L, voltage sources, diodes, BJTs, JFETs
- DK kernel build with proper trapezoidal discretization
- NR solver: 1D, 2D (two 1D devices), and M-dimensional
- Codegen for diode, BJT, JFET, and tube/triode circuits (up to M=16, Gaussian elimination for M=3..16)
- Codegen uses proper trapezoidal RHS with `input_prev` tracking (matches runtime solver)
- Per-device `.model` params: each device gets its own IS, N, BF, BR, VT (heterogeneous models supported)
- Nonlinear DC operating point solver (Newton-Raphson with source stepping and Gmin stepping fallbacks)
- DC OP integrated into codegen (`DC_NL_I` constant) and runtime (`initialize_dc_op()`)
- DC operating point calculation includes input conductance
- Plugin template generates per-channel state for stereo (no cross-channel corruption)
- SPICE validation infrastructure with ngspice
- Input validation: parser rejects negative/zero/NaN/Inf component values; codegen validates node indices
- Error types are enums (`MnaError`, `DkError`, `CodegenError` with `InvalidConfig`) — no panicking library code
- Logging via `log` crate (no `eprintln!` in library code)
- MAX_M=16 bound prevents unbounded allocation in DK kernel
- CLI reports errors for unresolved node names (no silent defaults)
- **BJT common-emitter amplifier**: SPICE validation passes (correlation 0.965, 35% RMS)
  - Trapezoidal nonlinear integration: correction uses full `S*N_i*i_nl` (not delta)
  - Combined with `N_i*i_nl_prev` in RHS, gives proper trapezoidal average
  - **Gummel-Poon model**: `BjtParams` includes VAF, VAR, IKF, IKR; `bjt_qb()` base charge modulation
  - USE_GP flag auto-detected from params; falls back to Ebers-Moll when GP params are infinite
- **Dynamic potentiometers**: `.pot R1 min max` directive marks a resistor as runtime-variable
  - Sherman-Morrison rank-1 updates: O(N²) correction instead of O(N³) re-inversion
  - Precomputed SM vectors (SU, USU, NV_SU, U_NI) baked into generated constants
  - Corrections applied to S, K, A_neg, and S*N_i products in codegen
  - Plugin template auto-generates `FloatParam` knobs for each pot
  - Max 2 pots per circuit; pot value stored in `CircuitState`
- **Plugin level params always included**: Input Level (-12 dB default, -36 to +12 dB) and Output Level (0 dB default, -60 to +12 dB)
  - -12 dB input maps ±1V DAW to ±250mV (safe for guitar-level circuits)
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
  - Constants: `DEVICE_{n}_MU`, `DEVICE_{n}_EX`, `DEVICE_{n}_KG1`, `DEVICE_{n}_KP`, `DEVICE_{n}_KVB`, `DEVICE_{n}_IG_MAX`, `DEVICE_{n}_VGK_ONSET`
  - Template: `device_tube.rs.tera`; 6 tests (parser, MNA, codegen, compile-and-run, two-triode preamp)
- **Explicit re-exports**: `lib.rs` uses named re-exports (no glob `pub use module::*`)

### Known Limitations
- Purely resistive nonlinear circuits oscillate (need capacitor damping)
- MOSFET codegen: 2D Level 1 SPICE with triode + saturation + channel-length modulation (LAMBDA)
- JFET codegen: 2D Shichman-Hodges with triode + saturation + channel-length modulation (LAMBDA)
- Tube Koren model: no plate resistance (rp) or mu variation with operating point
- BJT Gummel-Poon: no self-heating or charge storage dynamics
- **Runtime solver** (`DeviceEntry`) only supports Diode, DiodeWithRs, Led, BJT — no JFET, MOSFET, or Tube
  - `melange simulate` skips tubes with warning; use codegen (`--format plugin`) for tube circuits
  - Codegen supports all device types

### Cross-Compilation (macOS from Linux)
- Zig 0.13 + cargo-zigbuild + macOS SDK 13.3 + rcodesign (ad-hoc signing)
- `cargo zigbuild --release --target universal2-apple-darwin` produces universal Mac binaries
- Tested: rc-lowpass, pultec-eq, wurli-preamp all compile + sign successfully
- melange-cli does NOT cross-compile (ureq/dirs need CoreFoundation), but generated plugins do

### Validated Circuits
- `circuits/pultec-eq.cir`: Pultec EQP-1A (N=13, M=2, 3 switches, 5 pots, 12AX7 tube)
- `circuits/wurli-preamp.cir`: Wurlitzer 200A preamp (N=11, M=5, 2 BJTs + 1 diode, 1 pot)
  - Flattened from openwurli/spice/subcircuits/preamp.cir
  - R1-Cin series input coupling via intermediate node (mid_in)
  - 0.1V in → 9.12V peak out at R_ldr=100K nominal
- `circuits/tweed-preamp.cir`: Fender-style 2-stage 12AX7 guitar amp (N=13, M=4, 1 pot, 1 switch)
  - Two 12AX7 triode stages, interstage volume pot (shunt, 500Ω-500kΩ), bright switch (3-pos)
  - Stage 1: fully bypassed cathode (25µF, max gain); Stage 2: partial bypass (0.68µF, ~156Hz)
  - Output pad (390k/6.8k, -35dB) models output transformer attenuation
  - 50mV in → 549mV out (11x / 20.8dB gain), zero NR divergence

### Pending Work

#### Current Priority — OpenWurli Integration
- **Validate wurli-preamp**: SPICE comparison, gain at R_ldr extremes, frequency response
- **Benchmark M=5 vs M=2**: OpenWurli uses forward-active BJT (Ib≈0, M=2 Cramer's); melange models full Ic+Ib (M=4) + diode (M=5) Gaussian elimination. If perf matters at 64 voices, add 1D forward-active BJT mode.
- **M>16**: Iterative/sparse NR for very large nonlinear systems

#### Future — Multi-Language Codegen
The `Emitter` trait + `CircuitIR` are language-agnostic by design. Once Rust output is complete, planned targets in priority order: C++ (pro plugin devs), FAUST (compiles to 30+ targets), Python/NumPy (prototyping), MATLAB/Octave (academic).

#### Deferred
- **Phase 6a/6b** (type safety): NodeIdx newtype and field visibility
- **Phase 7** (crate split): Extract melange-parser, melange-codegen
