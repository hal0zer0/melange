# Known Limitations

This document lists known limitations of melange-solver. Items marked [DEFERRED] are intentional simplifications for the current implementation.

## SPICE Compatibility

### Missing Element Types [DEFERRED]

The following standard SPICE element types are not implemented:

- ~~**E** (Voltage-Controlled Voltage Source)~~ ✅ Implemented (2026-03-13)
- **F** (Current-Controlled Current Source) - Current mirrors
- ~~**G** (Voltage-Controlled Current Source)~~ ✅ Implemented (2026-03-13)
- **H** (Current-Controlled Voltage Source)
- **B** (Behavioral sources) - Arbitrary expressions
- ~~**K** (Coupled inductors) - Transformers~~ ✅ Implemented (2026-03-02)
- ~~**S** / **W** (Switches)~~ ✅ Implemented via `.switch` directive (2026-03-02)
- **T** (Transmission lines)

**Workaround:** Model these using combinations of existing elements where possible.

### Missing Directives [DEFERRED]

- `.include` / `.lib` - File inclusion
- `.temp` - Temperature specification
- `.global` - Global nodes
- `.nodeset` / `.ic` - Initial conditions
- `.option` - Parsed but ignored

### Parametric Expressions [DEFERRED]

Expressions like `{R1*2}` in component values are not supported. Only numeric values are parsed.

### Temperature Dependencies [PARTIAL]

Temperature coefficients (TC1, TC2) on resistors are ignored. BJT self-heating (Rth, Cth, XTI, EG) is available via thermal RC model with SPICE3f5 IS(T) scaling (default disabled, Rth=infinity). All other simulations run at nominal temperature.

## Parser Issues

### ~~Line Continuation~~ ✅ Fixed

Multi-line statements with `+` continuation characters are now properly concatenated. The parser joins continuation lines before processing.

**Example:**
```spice
.model 2N2222 NPN(
+ IS=1e-15 BF=200
+ )
```
This parses correctly.

### ~~Subcircuits~~ ✅ Implemented (2026-03-02)

Subcircuit expansion now works: `X` elements are recursively expanded with node remapping, cycle detection (max depth 16), and nested subcircuit support.

## Numerical Limitations

### Matrix Storage [PERFORMANCE]

Matrices use `Vec<Vec<f64>>` (jagged arrays) instead of flat storage. This has poor cache locality but is acceptable for typical circuits (validated up to N=41, Pultec EQP-1A with 4 tubes and 2 transformers).

### Denormal Handling [PERFORMANCE]

Very small values (< 1e-308) are not flushed to zero. On x86, this can cause performance degradation (denormal slowdown).

### Condition Number [NUMERICAL]

Condition number estimation is performed during DK kernel build. A `log::warn!` is emitted if cond(A) > 1e12. Very ill-conditioned circuits will still produce results but may have reduced accuracy.

## Solver Limitations

### Voltage Sources

Independent voltage sources (V elements) are implemented as Norton equivalents (high-conductance stamp in G matrix + current in RHS). DC supply voltages (e.g. VCC) and the input source both work correctly.

### ~~Multi-Dimensional NR~~ ✅ Full Implementation

For M > 2 nonlinear elements, the solver uses full Newton-Raphson with block-diagonal Jacobian and Gaussian elimination (M=3..16). Codegen supports up to M=16.

### Device Models

- **BJT**: Full 2D (Ic + Ib) Ebers-Moll and Gummel-Poon models. Self-heating (Rth/Cth, SPICE3f5 IS(T)) and charge storage (CJE/CJC/TF junction + diffusion caps) available, default disabled.
- **Op-amps**: Boyle macromodel with GBW dominant pole, VCC/VEE asymmetric supply rail clamping. No slew rate limiting.
- **JFET**: Full 2D Shichman-Hodges (triode + saturation + channel-length modulation)
- **MOSFET**: Full 2D Level 1 SPICE (triode + saturation + channel-length modulation)
- **Vacuum Tube**: Koren triode + Leach grid current + lambda (finite plate resistance)
- **VCA**: 2D current-mode exponential gain (THAT 2180 / DBX 2150)

## Audio-Specific Missing Features

### ~~Potentiometers~~ ✅ Implemented (codegen 2026-02-28)

`.pot` directive with Sherman-Morrison rank-1 updates for real-time parameter control. Codegen emits precomputed SM vectors; plugin template auto-generates FloatParam knobs.

### ~~Switches~~ ✅ Implemented (2026-03-02)

`.switch` directive with ganged components, matrix rebuild on position change.

### ~~Transformers~~ ✅ Implemented (2026-03-02)

SPICE-standard `K` element for coupled inductors with symmetric mutual conductance stamps.

### LFO/Modulation [DEFERRED]

No time-varying sources for tremolo/vibrato effects.

## Real-Time Constraints

### Allocation

Generated code pre-allocates all buffers in `CircuitState`. No heap allocation occurs in `process_sample()`.

### Worst-Case Performance

Matrix recomputation is O(n³) and occurs at:
- `set_sample_rate()` calls
- Never during audio callback

For typical circuits (up to N=41 validated), this is negligible.

## Documentation Gaps

### Examples

Missing complete examples showing:
- Full workflow from netlist to processed audio
- Multi-threaded usage patterns
- Custom device implementation

### API Documentation

Many public structs lack detailed field documentation.

## Validation

### Component Values

- Negative resistors/capacitors/inductors are rejected (returns error, does not panic)
- Zero values may cause numerical issues
- No warnings for extreme values (1e-300, 1e300)

### Circuit Topology

- No check for floating nodes (no DC path to ground)
- No check for shorts across voltage sources
- No check for singular matrices before inversion

---

## Design Decisions

These are intentional trade-offs, not bugs:

1. **f64 everywhere**: Double precision for all calculations. No f32 optimization.

2. **Codegen-only pipeline**: Netlist → MNA → DK/Nodal kernel → optimized Rust source code. No interpreted runtime solver.

3. **Trapezoidal default, backward Euler opt-in**: Trapezoidal for accuracy; `--backward-euler` for unconditional stability. Auto-detected when spectral radius > 1.

4. **Const generic devices**: Device dimension at compile time for stack allocation. No heap in device models.

5. **No SPICE netlist output**: Can parse SPICE, cannot write it.

---

*Last updated: 2026-04-10*
