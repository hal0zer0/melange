# Known Limitations

This document lists known limitations of melange. Items marked [DEFERRED] are intentional scope boundaries for the current implementation.

## SPICE Compatibility

### Supported Element Types

Melange supports the following SPICE elements:

| Prefix | Type | Notes |
|--------|------|-------|
| R | Resistor | |
| C | Capacitor | `IC=` initial voltage supported |
| L | Inductor | `ISAT=` for core saturation (tanh model, uncoupled only) |
| V | Voltage source | DC only (AC/transient sources for validation only) |
| I | Current source | DC only |
| D | Diode | Shockley + RS + BV/IBV (Zener) |
| Q | BJT (NPN/PNP) | Ebers-Moll and Gummel-Poon |
| J | JFET (NJF/PJF) | Shichman-Hodges |
| M | MOSFET (NM/PM) | Level 1 SPICE with body effect (GAMMA/PHI) |
| T | Triode tube | Koren plate current + Leach grid current |
| P | Pentode/beam tetrode | 5 equation families, 29 catalog models |
| U | Op-amp | Boyle macromodel (GBW, VCC/VEE rails, SR) |
| Y | VCA | THAT 2180-style exponential gain |
| E | VCVS | Voltage-controlled voltage source |
| G | VCCS | Voltage-controlled current source |
| K | Coupled inductors | Transformers (multi-winding supported) |
| X | Subcircuit instance | Recursive expansion, max depth 16 |

### Missing Element Types [DEFERRED]

- **F** (Current-Controlled Current Source) -- current mirrors
- **H** (Current-Controlled Voltage Source)
- **T** (Transmission lines)

**Workaround:** Model these using combinations of existing elements where possible.

### Behavioral Sources (B) [PARTIAL]

`B... V={expr}` / `I={expr}` arbitrary-expression sources are wired on the
**nodal codegen path**: expressions over node voltages, `time`, `ddt`, `idt`,
and `.param`/`.runtime` parameters compile and are oracle-validated
(`behavioral_source_tests.rs`). Not yet supported: branch-current references in
expressions, and the DK path -- a circuit containing a B-source routes nodal, or
errors loudly if it cannot. See `docs/aidocs/BEHAVIORAL_SOURCES.md`.

### Missing Directives [DEFERRED]

- `.include` / `.lib` -- file inclusion
- `.temp` -- temperature specification
- `.global` -- global nodes
- `.nodeset` / `.ic` -- initial conditions (parser accepts but ignores)
- `.option` -- parsed but ignored

### Parametric Expressions [DEFERRED]

Expressions like `{R1*2}` in component values are not supported. Only numeric values are parsed.

### Temperature Dependencies [PARTIAL]

Temperature coefficients (TC1, TC2) on resistors are ignored, and there is no global temperature sweep (`.temp`). Device self-heating (Rth, Cth, XTI, EG, TAMB) is available via a quasi-static thermal RC model with SPICE3f5 IS(T) scaling for **diodes, BJTs, and triodes** (default disabled, Rth=infinity → dead code). Base device models otherwise run at a fixed nominal 27C.

Separately, the authentic-noise feature carries a runtime-settable noise temperature (`set_temperature_k`, default 290 K), which scales only thermal noise — it does not affect the deterministic device equations. See the Circuit Noise section below.

## Device Model Limitations

### Diode
- BV/IBV: hard clamp reverse breakdown (no smooth Zener knee)
- No TC1/TC2 temperature coefficients; optional quasi-static self-heating (RTH/CTH/XTI/EG/TAMB), disabled by default

### BJT (Gummel-Poon)
- Q1 Early effect guard: `q1_denom <= 0` clamps to 1.0 (physically near Early voltage limit)
- Self-heating (RTH/CTH) available but disabled by default (RTH=infinity)
- Junction capacitances (CJE/CJC) and diffusion capacitance (TF) available
- Parasitic resistances (RB/RC/RE) supported with internal nodes
- No substrate current or avalanche breakdown

### JFET / MOSFET
- Subthreshold: hardcoded 2xVT slope (real devices: 60-120 mV/decade)
- MOSFET Level 1 only (no BSIM3/4)

### Triode
- Koren model with lambda for finite plate resistance
- No space-charge or transit-time effects

### Pentode
- 5 equation families: Rational (Derk), Exponential (DerkE), Classical (Koren/Cohen-Helie), plus variable-mu variants
- 29 catalog models (EL84, EL34, EF86, 6L6, 6V6, KT88, 6550, 6K7, EF89, and more)
- Grid-off FA reduction (3D to 2D) via `--tube-grid-fa {auto,on,off}`
- No independent suppressor dynamics (suppressor always cathode-tied)
- 6386/6BA6/6BC8 datasheet fits for variable-mu compressors deferred (phase 1d)

### Op-amp
- Boyle macromodel with GBW dominant pole
- VCC/VEE asymmetric supply rail clamping
- Slew-rate limiting via `SR=` in V/us (per-sample clamp, all 3 codegen paths)
- Rail mode selection: `--opamp-rail-mode {auto,none,hard,active-set,active-set-be,boyle-diodes}`
- BoyleDiodes mode diverges under heavy clipping; auto-routes to active-set-be

### VCA
- 2D current-mode exponential gain (THAT 2180 / DBX 2150)
- THD: gain-dependent cubic nonlinearity
- `noise_floor` field exists but unused

### Saturating Inductors
- `L1 a b 100m ISAT=20m` with tanh L(I) saturation model
- Sherman-Morrison rank-1 per-sample update with drift resync every 16 updates
- Phase 1: uncoupled inductors only (no coupled/transformer saturation)
- Lagged one sample (uses previous-sample current for L(I) computation)
- No ngspice validation yet

### LDR (Photoresistor)
- `CdsLdr` device model (VTL5C3/4, NSL-32 presets) with attack/release photocell dynamics
- Placed in a netlist via the `O` element (`O1 rphoto+ rphoto- led+ led- MODEL` + `.model MODEL LDR()`), on the stateful-device codegen path (both DK and nodal)
- No ngspice twin (SPICE has no equivalent LDR model to validate against)

## Dynamic Parameter Controls

### Potentiometers
- `.pot R1 min max` marks a resistor as runtime-variable
- `.wiper R_cw R_ccw total_R` models 3-terminal wiper pots
- `.gang "Label" member1 member2` links multiple pots/wipers to a single UI parameter (`!` prefix inverts a member)
- Per-block O(N^3) matrix rebuild on value change
- Per-sample smoothing via `.smoothed.next()` in generated plugin
- Reseed-free setters (stamp ΔG only, no mid-signal NR-state reset); call `recompute_dc_op()` explicitly for preset-recall / unsmoothed jumps (DK path)
- Maximum 64 pots per circuit

### Switches
- `.switch C1 100n 220n 470n` selects among discrete component values
- Maximum 16 switches per circuit
- Triggers matrix rebuild on position change

### Device Linearization
- `.linearize Q9` or `.linearize V1` removes a BJT or triode from the NR system
- Replaced with small-signal conductances at DC operating point
- Reduces nonlinear dimension M (BJT: M-2, triode: M-2 per device)
- Device still affects the circuit via linearized g_m, g_pi, r_o stamps in G

## Numerical Limitations

### Matrix Storage [PERFORMANCE]

Matrices use `Vec<Vec<f64>>` (jagged arrays) instead of flat storage. This has poor cache locality but is acceptable for typical circuits (validated up to N=64).

### Denormal Handling [PERFORMANCE]

Very small values (< 1e-308) are not flushed to zero. On x86, this can cause performance degradation (denormal slowdown).

### Condition Number [NUMERICAL]

Condition number estimation is performed during DK kernel build. A `log::warn!` is emitted if cond(A) > 1e12. Very ill-conditioned circuits will still produce results but may have reduced accuracy.

### Nonlinear System Size

Codegen supports up to M=24 nonlinear device dimensions (`MAX_M=24`). For M > 2, the solver uses full Newton-Raphson with block-diagonal Jacobian and Gaussian elimination (M=3..24). Forward-active detection, grid-off FA reduction, and `.linearize` all reduce M.

## Solver Limitations

### Voltage Sources

Independent voltage sources (V elements) are implemented as Norton equivalents (high-conductance stamp in G matrix + current in RHS). DC supply voltages (e.g., VCC) and the input source both work correctly.

### Solver Routing

Three codegen paths, auto-selected:

| Path | When Selected | Cost |
|------|--------------|------|
| DK Schur | M<10, <=1 transformer, K well-conditioned | O(N^2+M^3)/sample |
| Nodal Schur | M>=10 or 2+ transformers, K well-conditioned | O(N^2+M^3)/sample |
| Nodal full LU | K~0 (VCA), positive K diagonal, K or S ill-conditioned | O(N^3)/sample |

Override with `--solver dk` or `--solver nodal`.

## Real-Time Constraints

### Allocation

Generated code pre-allocates all buffers in `CircuitState`. No heap allocation occurs in `process_sample()`.

### Worst-Case Performance

Matrix recomputation is O(N^3) and occurs at:
- `set_sample_rate()` calls
- `set_pot_N()` / `set_switch_N()` calls (per-block, when value changes)

For typical circuits (N<=41 validated), pot rebuild takes ~250us at N=37 -- well within a 128-sample buffer deadline at 48kHz (2.67ms).

### Performance Benchmarks

Measured on an AMD Ryzen 9 7950X, single core, noiseless, `-C target-cpu=x86-64-v3` (median of 7 × 2M samples via `tools/perf-harness/bench.sh`); throughput is host-dependent.

- Light nonlinear circuits: 12AX7 gain stage ~230×, overdrive pedal ~64× realtime
- Typical multi-device circuits: Wurlitzer preamp ~56×, tweed guitar amp ~29× realtime
- Heaviest validated: a Pultec-style passive EQ (nodal full-LU, chord + sparse LU, N=52, M=8) ~24×, SSL-class bus compressor ~9× realtime

## Circuit Noise [PARTIAL]

Authentic time-domain circuit noise is implemented (opt-in, off by default) and
injected as Norton current sources into the MNA RHS, so it is shaped by the
solver's transfer function and modulated by the nonlinear operating point.
Enable with `--noise {off|thermal|shot|full}` (`--noise-seed <u64>` for
determinism). Runtime controls: `set_noise_enabled`, `set_noise_gain`,
`set_thermal_gain` / `set_shot_gain` / `set_flicker_gain`, `set_temperature_k`,
`set_seed`. With `--noise off` (default) the generated code is byte-identical to
a noiseless build. Full reference: `docs/aidocs/NOISE.md`.

Shipped noise sources:
- **Thermal (Johnson-Nyquist)** on every fixed and dynamic (`.pot`/`.wiper`/`.runtime R`/`.switch`) resistor
- **Shot** on every junction (diode, BJT, JFET/MOSFET, triode plate with space-charge smoothing)
- **1/f flicker** on junctions (`.model … KF=… AF=…`) and on resistors (per-element `KF=`/`AF=`, Hooge bias-squared)
- **Pentode partition** noise (Schottky) and **op-amp en/in** (`.model OA(EN=… IN=…)`, white-band v1)

Noise limitations:
- Op-amp `EN_FC`/`IN_FC` (1/f corner) parameters parse but are **not yet wired** — Phase 4 is white-band only
- On the **DK codegen path**, BJT parasitic RB/RC/RE thermal noise (rbb′) is skipped (logged as a `warn!`); route the circuit nodal to include it
- Diode `RS` and tube `RGI` parasitic resistances are not yet thermal-noise sources
- Setting `KF`/`AF` on resistors (or any `.mismatch`/`.tolerance` jitter) breaks ngspice parity — strip before SPICE-validating
- **Tube microphonics** (Phase 6) is research only, not implemented

## Not Implemented [DEFERRED]

- **LFO/Modulation**: no time-varying sources for tremolo/vibrato (use `.runtime R`/`.runtime V` host-driven modulation instead)
- **Temperature sweep**: no `.temp` directive or global temperature sweep (device self-heating is available per-device, see Temperature Dependencies above)
- **Multi-language codegen**: C++ in progress; Python/NumPy and MATLAB targets
  planned. **FAUST was explored and determined impractical** — its generated code
  is intentionally not Turing-complete (it computes each sample in a fixed number
  of operations), so a Newton-Raphson solve whose iteration count depends on the
  data cannot be expressed. Only strictly linear circuits would be emittable,
  which is a small enough subset to not be worth a backend.
- **M > 24**: iterative/sparse NR for very large nonlinear systems (MAX_M=24)
- **Ideal transformer formulation**: dependent sources + explicit leakage/magnetizing L

## Validation

### Component Values

- Negative, zero, NaN, and Inf component values are rejected (returns error, does not panic)
- Self-connected components are rejected
- No warnings for extreme values (1e-300, 1e300)

### Circuit Topology

- No check for floating nodes (no DC path to ground)
- No check for shorts across voltage sources
- Condition number warning when cond(A) > 1e12

### Parser Hardening

- Input caps: MAX_NETLIST_BYTES=10M, MAX_NODE_NAME_LEN=256, MAX_TOTAL_ELEMENTS=50k
- Non-ASCII normalization: mu/micro mapped to `u`; other non-ASCII rejected
- cargo-fuzz target exercises parser through MNA through DK kernel

---

## Design Decisions

These are intentional trade-offs, not bugs:

1. **f64 everywhere**: Double precision for all calculations. No f32 optimization.

2. **Codegen-only pipeline**: Netlist to MNA to DK/Nodal kernel to optimized Rust source code. No interpreted runtime solver.

3. **Trapezoidal default, backward Euler opt-in**: Trapezoidal for accuracy; `--backward-euler` for unconditional stability. Auto-detected when spectral radius > 1.

4. **Const generic devices**: Device dimension at compile time for stack allocation. No heap in device models.

5. **No SPICE netlist output**: Can parse SPICE, cannot write it.

---

*Last updated: 2026-07-19*
