# Known Limitations

This document lists known limitations of melange-solver. Items marked [DEFERRED] are intentional simplifications for the current implementation.

## SPICE Compatibility

### Missing Element Types [DEFERRED]

The following standard SPICE element types are not implemented:

- **E** (Voltage-Controlled Voltage Source) - Needed for ideal op-amps
- **F** (Current-Controlled Current Source) - Current mirrors
- **G** (Voltage-Controlled Current Source) - Transconductance amplifiers
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

### Temperature Dependencies [DEFERRED]

Temperature coefficients (TC1, TC2) on resistors are ignored. All simulations run at nominal temperature.

## Parser Issues

### Line Continuation [KNOWN BUG]

Multi-line statements with `+` continuation characters are not properly concatenated. The `+` is stripped but lines are not joined.

**Example:**
```spice
.model 2N2222 NPN(
+ IS=1e-15 BF=200
+ )
```
This will NOT parse correctly.

**Workaround:** Use single-line statements.

### ~~Subcircuits~~ ✅ Implemented (2026-03-02)

Subcircuit expansion now works: `X` elements are recursively expanded with node remapping, cycle detection (max depth 16), and nested subcircuit support.

## Numerical Limitations

### Matrix Storage [PERFORMANCE]

Matrices use `Vec<Vec<f64>>` (jagged arrays) instead of flat storage. This has poor cache locality but is acceptable for small circuits (N ≤ 8).

### Denormal Handling [PERFORMANCE]

Very small values (< 1e-308) are not flushed to zero. On x86, this can cause performance degradation (denormal slowdown).

### Condition Number [NUMERICAL]

No condition number estimation is performed. Very ill-conditioned circuits may produce inaccurate results without warning.

## Solver Limitations

### Voltage Sources [PARTIAL]

Independent voltage sources (V elements) are parsed but not fully integrated into the MNA system. Only the input source (for `process_sample`) is handled correctly.

### Multi-Dimensional NR [SIMPLIFIED]

For M > 2 nonlinear elements, the solver uses a damped fixed-point iteration instead of full Newton-Raphson with Jacobian. This converges slower but is more stable.

### Device Models [PARTIAL]

- **BJT**: Ebers-Moll model in devices crate, but only collector current is used in solver (simplified)
- **Op-amps**: Boyle macro model defined but not integrated into solver
- **JFET/MOSFET**: Simplified 1D models only

## Audio-Specific Missing Features

### ~~Potentiometers~~ ✅ Implemented (2026-02-28)

`.pot` directive with Sherman-Morrison rank-1 updates for real-time parameter control.

### ~~Switches~~ ✅ Implemented (2026-03-02)

`.switch` directive with ganged components, matrix rebuild on position change.

### ~~Transformers~~ ✅ Implemented (2026-03-02)

SPICE-standard `K` element for coupled inductors with symmetric mutual conductance stamps.

### LFO/Modulation [DEFERRED]

No time-varying sources for tremolo/vibrato effects.

## Real-Time Constraints

### Thread Safety

`CircuitSolver` is NOT `Sync`. Each audio thread must have its own solver instance. The type is `Send`, so solvers can be moved between threads.

### Allocation

The solver pre-allocates all buffers. No heap allocation occurs in `process_sample()`.

### Worst-Case Performance

Matrix inversion is O(n³) and occurs at:
- Construction time (sample rate change)
- Never during audio callback

For n ≤ 8 nodes, this is negligible.

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

- Negative resistors/capacitors/inductors are rejected (panic)
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

2. **Interpreted solver**: Runtime MNA assembly and matrix solve. Slower than compiled code but more flexible.

3. **Trapezoidal integration**: Fixed rule, no user choice. Good balance of accuracy and stability.

4. **Const generic devices**: Device dimension at compile time for stack allocation. No heap in device models.

5. **No SPICE netlist output**: Can parse SPICE, cannot write it.

---

*Last updated: 2026-03-02*
