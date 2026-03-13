# Melange Project Status

**Last Updated:** 2026-03-13
**Version:** 0.2.0-dev
**Status:** Production Ready (Core Crates) — 731+ tests passing

---

## Quick Summary

| Component | Status | Notes |
|-----------|--------|-------|
| melange-primitives | ✅ Production | DSP building blocks complete |
| melange-devices | ✅ Production | All device models implemented |
| melange-solver | ✅ Production | MNA/DK solver, parser complete |
| melange-validate | ✅ Production | 6 SPICE comparisons (RC, diode, BJT, JFET, MOSFET, op-amp) |
| melange-plugin | ⚠️ Stub | Plugin generation via codegen works; crate itself is placeholder |
| melange-cli | ✅ Production | compile, simulate, analyze, nodes commands |
| Code Generation | ✅ Production | M=0..16, all device types, pots, switches, coupled inductors |

---

## Completed Features

### Layer 1: melange-primitives ✅

**Filters:**
- [x] OnePole lowpass/highpass
- [x] Biquad (all standard types)
- [x] TPT (state-variable) filters
- [x] DC blocker

**Oversampling:**
- [x] 2x half-band filter
- [x] 4x half-band filter
- [x] Polyphase IIR structure

**Newton-Raphson:**
- [x] 1D solver with damping
- [x] 2D solver with Cramer's rule
- [x] M×D solver for DK method
- [x] Linear system solver (up to 8x8)

**Companion Models:**
- [x] Trapezoidal rule
- [x] Backward Euler
- [x] Capacitor/inductor stamping

### Layer 2: melange-devices ✅

**Device Models:**
- [x] Diode (Shockley, LED)
- [x] BJT (Ebers-Moll, Gummel-Poon, self-heating Rth/Cth, charge storage CJE/CJC/TF)
- [x] JFET (Shichman-Hodges)
- [x] MOSFET (Level 1)
- [x] Vacuum Tube (Koren triode/pentode, lambda for finite plate resistance)
- [x] CdS LDR (VTL5C3/4, NSL-32)
- [x] Op-Amp (VCCS MNA stamping — linear, working in both runtime and codegen)

**Traits:**
- [x] `NonlinearDevice<const N>` with const generics
- [x] Jacobian computation for all devices
- [x] Numerical validation tests

### Layer 3: melange-solver ✅

**Parser:**
- [x] R, C, L, V, I elements
- [x] D, Q, J, M devices
- [x] U (op-amp), T (triode), K (coupled inductors)
- [x] .model, .param directives
- [x] .subckt / X (subcircuit definition and expansion)
- [x] .pot (dynamic potentiometers)
- [x] .switch (ganged component switching)
- [x] .input_impedance directive
- [x] Value parsing (1k, 10uF, infix 6n8, etc.)

**MNA Assembly:**
- [x] G matrix stamping
- [x] C matrix stamping
- [x] Inductor companion model
- [x] N_v and N_i matrices
- [x] Multi-dimensional device support

**DK Kernel:**
- [x] Matrix inversion (Gaussian elimination)
- [x] S = A⁻¹ computation
- [x] K = N_v·S·N_i computation (no negation — K is naturally negative)
- [x] History term (A_neg)

**Solver:**
- [x] Interpreted runtime solver
- [x] Real-time safe (zero allocation)
- [x] 1D/2D NR with warm start
- [x] M-dimensional NR for M>2

**Code Generation:**
- [x] `DkKernel::emit_rust()` interface
- [x] Const-generic matrix sizes
- [x] Inlined NR loop for M=1,2
- [x] Precomputed matrices as const arrays
- [x] Device model code emission

---

## Known Limitations

See `docs/limitations.md` for full details.

### SPICE Compatibility
- Missing: E, F, G, H (controlled sources)
- Missing: B (behavioral sources)
- ~~Missing: K (coupled inductors/transformers)~~ ✅ Implemented (2026-03-02)
- ~~Missing: S, W (switches)~~ ✅ Implemented via `.switch` directive (2026-03-02)
- ~~Line continuation not working~~ ✅ Fixed (continuation lines joined before parsing)
- Parametric expressions not supported

### Solver
- Voltage sources: implemented as Norton equivalents (DC supply + input source both work)
- ~~Sherman-Morrison rank-1 updates: spec only~~ ✅ Implemented (pots use SM rank-1 updates in both codegen and runtime solver)
- Parasitic cap auto-insertion: 10pF across device junctions when nonlinear circuit has no caps
- Shadow subtraction: not implemented

### Device Integration
- ~~Op-amp Boyle model: defined but not wired to solver~~ ✅ Op-amps work via VCCS MNA stamping (linear, no NR dimensions)
- ~~Full BJT with base current: 2D kernel ready, needs device adapter~~ ✅ Implemented (2D Ic+Ib in both runtime and codegen)

---

## TODO / Roadmap

### Immediate (v0.2.0)

**Code Generation:**
- [x] Basic code generator (M=0,1,2 circuits)
- [x] Generate `.rs` files from `DkKernel`
- [x] Unrolled matrix operations
- [x] Inlined device models
- [x] M=3,4 NR solver generation (Gaussian elimination)
- [x] Full BJT (2D) device support (codegen + runtime)
- [x] Per-device model parameters (heterogeneous IS, BF, etc.)
- [x] PNP BJT support

**Validation:**
- [x] SPICE comparison pipeline (ngspice integration)
- [x] Automated tolerance testing (correlation + RMS)
- [ ] THD/SNR measurements

**DC Operating Point:**
- [x] Nonlinear DC OP solver (Newton-Raphson + source stepping + Gmin stepping)
- [x] Codegen integration (DC_NL_I constant)
- [x] Runtime integration (initialize_dc_op)

### Fixed: DK Solver BJT Stability (2026-03-01)

**Problem (resolved):** The DK solver exhibited period-3 oscillation on BJT
common-emitter amplifier circuits due to mixing trapezoidal integration (linear
elements) with backward Euler (nonlinear currents).

**Fix applied:** Changed the DK correction step from `v = v_pred + S*N_i*(i_nl - i_nl_prev)`
(backward Euler) to `v = v_pred + S*N_i*i_nl` (trapezoidal). Combined with the
`N_i*i_nl_prev` already in the RHS, this gives `N_i*(i_nl[n+1] + i_nl[n])` —
proper trapezoidal averaging for nonlinear currents. NR equations unchanged.

**Results:**
- [x] BJT CE correlation: 0.965 (was 0.035)
- [x] Period-3 oscillation eliminated
- [x] Diode clipper: passes (unaffected for M=0)
- [x] Antiparallel diodes: passes
- [x] RC lowpass: passes (unaffected for M=0)
- [x] All 311 solver tests pass

### Short Term (v0.3.0)

**Features:**
- [ ] Controlled source support (E, G)
- [x] Potentiometer model (Sherman-Morrison rank-1 updates)
- [x] Switch model (`.switch` directive with ganged components, rebuild_matrices)
- [x] Line continuation fix

**Integration:**
- [x] melange-validate implementation
- [x] CLI with actual commands
- [ ] nih-plug wrapper (melange-plugin)

### Medium Term (v0.4.0)

**Advanced:**
- [x] Sherman-Morrison for time-varying elements (potentiometers)
- [x] Subcircuit expansion (recursive X element substitution with node remapping, cycle detection)
- [ ] Behavioral sources (B)
- [x] Transformers (K) (SPICE-standard coupled inductors, symmetric mutual conductance stamps)

**Performance:**
- [ ] SIMD matrix operations
- [ ] Flat matrix storage
- [ ] Cache optimization

### Long Term (v1.0.0)

**Complete:**
- [ ] Full SPICE compatibility
- [ ] Proc-macro integration
- [ ] GUI schematic capture
- [ ] Visual validation tools

---

## Deferred Features

These features were explicitly deferred per user request:

1. ~~**Code Generation Module**~~ ✅ Implemented (M=0..16, all device types, pots, switches, coupled inductors)
2. **Proc-Macros** - Planned for v1.0.0
3. ~~**Full BJT Integration**~~ ✅ Implemented (2D Ic+Ib in runtime and codegen)
4. **Extended MNA** - Voltage sources work as Norton equivalents; controlled sources (E, F, G, H) not yet implemented

---

## Technical Debt

### Code Quality
- [ ] Convert `Vec<Vec<f64>>` to flat storage (performance)
- [ ] Iterator-based loops (style)
- [ ] Flatten nested match expressions (readability)

### Documentation
- [ ] More doc examples in device models
- [ ] Tutorial: "Your First Circuit"
- [ ] API reference completion

### Testing
- [ ] Fuzz testing for parser
- [ ] Property-based tests for solvers
- [ ] SPICE validation corpus

---

## Review History

| Round | Date | Focus | Outcome |
|-------|------|-------|---------|
| 1 | 2026-02-23 | Parser bugs, real-time safety | 4 critical fixes |
| 2 | 2026-02-23 | Deep audit, multi-dimensional | 5 critical fixes, architecture refactor |
| 3 | 2026-02-23 | Panic fuzzing, edge cases | 4 critical fixes, validation added |
| 4 | 2026-02-23 | Architecture, documentation | Performance fix, docs added |
| 5 | 2026-02-23 | Final polish, clippy | 0 warnings, release ready |

---

## Files

### Documentation
- `README.md` - Project overview
- `AGENTS.md` - Developer guidelines
- `CHANGELOG.md` - Release history
- `docs/STATUS.md` - This file
- `docs/limitations.md` - Known limitations
- `docs/codegen.md` - Code generation spec (complete)
- `docs/critical-fixes-2026-02-23.md` - Review history
- `docs/dk-method.md` - DK method theory
- `docs/mna-assembly.md` - MNA documentation
- `docs/device-models.md` - Device documentation
- `docs/testing-strategy.md` - Testing approach

### Source
- `crates/` - Main library crates
- `tools/melange-cli/` - CLI tool
- `docs/` - Documentation

---

## Release Checklist v0.1.0

- [x] All tests passing (91/91)
- [x] Clippy clean (0 warnings)
- [x] Documentation complete
- [x] LICENSE files present
- [x] README.md comprehensive
- [x] CHANGELOG.md created
- [x] Cargo.toml metadata complete
- [x] Version consistent (0.1.0)

---

## How to Contribute

See `AGENTS.md` for coding standards and architecture.

Priority areas for contribution:
1. Code generation implementation
2. melange-validate crate
3. SPICE element types (E, F, G)
4. Documentation examples

---

*The spice must flow.*
