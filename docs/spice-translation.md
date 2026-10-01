# SPICE → Rust Translation Reference

Field manual for debugging mismatches between ngspice and Rust DSP.

## Discretization Methods

| Method | Formula | Use When |
|--------|---------|----------|
| **Trapezoidal** | `g_c = 2C/T`, `J[n] = g_c*v[n] + i[n]` | Capacitors in MNA (C-3, C-4 Miller) |
| **Bilinear companion** | Pre-discretized admittance `Y(s) → Y(z)` | Series R-C (Cin-R1 input) |
| **ZDF** | Implicit trapezoidal for filters | One-pole LPF/HPF |
| **Forward Euler** | `x[n+1] = x[n] + T*f[n]` | **DO NOT USE** — unstable |

**Critical rule:** `A = 2C/T + G` carries the **whole G** (including companion conductances). melange's generated code uses the charge (companion) form: the history is `A_neg·v_prev + q_dot` with `A_neg = 2C/T` and no G at all ([COMPANION_MODELS.md](aidocs/COMPANION_MODELS.md)). A hand-written whole-system discretization (`A_neg = 2C/T - G`, sources as `b(n) + b(n+1)`) must use the **same G** in both matrices.

## Common Bug Patterns

### 1. MNA Stamp Errors
- **Sign of off-diagonals:** Must be `-g` for connected nodes
- **Ground references:** Only stamp diagonal for ground connections
- **VCCS direction:** Verify transconductance sign convention matches SPICE

### 2. Discretization Errors
- **Companion conductance in A:** `g_c = 2C/T` belongs in the system matrix `A = G + (2/T)C`, not added separately to the RHS
- **History update order:** Update AFTER solve, BEFORE next timestep
- **Trapezoidal consistency:** charge form — `A_neg` has no G, and `q_dot` is committed with `v_prev`; whole-system form — `A` and `A_neg` share the same G

### 3. Value/Unit Errors
- **Hz vs rad/s:** `ω = 2πf` — SPICE uses Hz, code may use either
- **Conductance vs resistance:** MNA uses Siemens (`g = 1/R`)
- **Capacitance units:** `4.7 MFD = 4.7e-6 F`, `100pF = 100e-12 F`
- **Vt temperature:** SPICE's TNOM, 27 °C (300.15 K) → Vt ≈ 25.865 mV (melange's `VT_ROOM`)

### 4. Topology Errors
- **Missing components:** Grep SPICE netlist, verify each in Rust
- **Floating nodes:** Creates singular matrix
- **Coupling paths:** Direct-coupled stages must share node index

## Bug Archaeology (from OpenWurli)

### Bug: Cin-R1 Companion Conductance
**Symptom:** HF response wrong  
**Root cause:** `A_neg` built from `G_dc` (excluding `g_cin`), but `A` included it  
**Fix (in OpenWurli's hand-written whole-system discretization):** both matrices use the same G (including `g_cin`). This applies only to that form: melange's generated code uses the charge form, where `A_neg` carries no G and no source term is averaged over two samples ([COMPANION_MODELS.md](aidocs/COMPANION_MODELS.md)).

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
