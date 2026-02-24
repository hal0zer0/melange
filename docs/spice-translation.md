# SPICE → Rust Translation Reference

Field manual for debugging mismatches between ngspice and Rust DSP.

## Discretization Methods

| Method | Formula | Use When |
|--------|---------|----------|
| **Trapezoidal** | `g_c = 2C/T`, `J[n] = g_c*v[n] + i[n]` | Capacitors in MNA (C-3, C-4 Miller) |
| **Bilinear companion** | Pre-discretized admittance `Y(s) → Y(z)` | Series R-C (Cin-R1 input) |
| **ZDF** | Implicit trapezoidal for filters | One-pole LPF/HPF |
| **Forward Euler** | `x[n+1] = x[n] + T*f[n]` | **DO NOT USE** — unstable |

**Critical rule:** Both `A = 2C/T + G` and `A_neg = 2C/T - G` must use the **same G** (including companion conductances).

## Common Bug Patterns

### 1. MNA Stamp Errors
- **Sign of off-diagonals:** Must be `-g` for connected nodes
- **Ground references:** Only stamp diagonal for ground connections
- **VCCS direction:** Verify transconductance sign convention matches SPICE

### 2. Discretization Errors
- **Companion conductance in G:** `g_c = 2C/T` must be in G, not separate
- **History update order:** Update AFTER solve, BEFORE next timestep
- **Trapezoidal symmetry:** `A` and `A_neg` share same G

### 3. Value/Unit Errors
- **Hz vs rad/s:** `ω = 2πf` — SPICE uses Hz, code may use either
- **Conductance vs resistance:** MNA uses Siemens (`g = 1/R`)
- **Capacitance units:** `4.7 MFD = 4.7e-6 F`, `100pF = 100e-12 F`
- **Vt temperature:** SPICE default 27°C → Vt ≈ 25.85mV

### 4. Topology Errors
- **Missing components:** Grep SPICE netlist, verify each in Rust
- **Floating nodes:** Creates singular matrix
- **Coupling paths:** Direct-coupled stages must share node index

## Bug Archaeology (from OpenWurli)

### Bug: Cin-R1 Companion Conductance
**Symptom:** HF response wrong  
**Root cause:** `A_neg` built from `G_dc` (excluding `g_cin`), but `A` included it  
**Fix:** Both use same G (including `g_cin`); add `cin_rhs_prev` for trapezoidal average

### Bug: Constant-GBW Assumption
**Symptom:** Trem-bright bandwidth 5.2 kHz (should be ~10 kHz)  
**Root cause:** Decoupled model assumed op-amp constant GBW; real circuit has nested feedback loops  
**Fix:** Full DK method with coupled 8-node MNA

## Verification Protocol

1. **Reproduce in SPICE** — save `.print` outputs
2. **Reproduce in Rust** — matching conditions
3. **Narrow the delta** — bisect signal chain
4. **Compare DC first** — wrong DC → wrong AC
5. **Trace to root cause** — use patterns above
6. **Verify fix** — all frequencies, all layers
