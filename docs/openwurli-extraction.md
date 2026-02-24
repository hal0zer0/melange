# Extraction Guide: OpenWurli → Melange

This document tracks which OpenWurli components are being extracted into melange crates,
what modifications are needed, and the extraction status.

## Implementation Status Summary

| Layer | Status | Tests |
|-------|--------|-------|
| Layer 1: melange-primitives | ✅ **COMPLETE** | 27 passing |
| Layer 2: melange-devices | ✅ **COMPLETE** | 28 passing |
| Layer 3: melange-solver | 🚧 Not started | - |
| Layer 4: melange-validate | 🚧 Not started | - |
| Layer 5: melange-plugin | 🚧 Not started | - |

---

## Extraction Inventory

### Layer 1: melange-primitives ✅

| OpenWurli File | Melange Target | Genericity | Status |
|---------------|---------------|------------|--------|
| `filters.rs` | `primitives::filters` | 95% generic | ✅ **DONE** |
| `oversampler.rs` | `primitives::oversampling` | 100% generic | ✅ **DONE** |
| `variation.rs` | `primitives::util` | 100% generic | ✅ **DONE** |
| (new) | `primitives::nr` | 100% generic | ✅ **DONE** |
| (new) | `primitives::companion` | 100% generic | ✅ **DONE** |

**Implementation notes:**
- Filters: OnePoleLpf/Hpf, TptLpf, Biquad (LP/HP/BP/Peaking/Shelf/LowShelf/HighShelf), DcBlocker
- Oversampling: Polyphase IIR half-band with 2/3/4/5 section options, 2x and 4x
- NR: 1D, 2D (Cramer's rule), and const-generic M×M solvers
- Companion models: Trapezoidal, Bilinear, Backward-Euler for capacitor discretization

### Layer 2: melange-devices ✅

| OpenWurli File | Melange Target | Genericity | Status |
|---------------|---------------|------------|--------|
| `bjt_stage.rs` | `devices::bjt` | 70% generic | ✅ **DONE** |
| `tremolo.rs` (LDR part) | `devices::ldr` | 60% generic | ✅ **DONE** |
| `power_amp.rs` | (future: devices::opamp) | 75% generic | ✅ **Partial** |
| (new) | `devices::diode` | 100% generic | ✅ **DONE** |
| (new) | `devices::tube` | 100% generic | ✅ **DONE** |
| (new) | `devices::jfet` | 100% generic | ✅ **DONE** |
| (new) | `devices::mosfet` | 100% generic | ✅ **DONE** |

**Implementation notes:**
- `NonlinearDevice<const N: usize>` trait with `current()` and `jacobian()` methods
- `TwoTerminalDevice` convenience trait for N=1 devices
- BJT: Ebers-Moll (2N2222, 2N3904, AC128), Gummel-Poon with Early effect
- Diode: Shockley (Si/Ge/Schottky/LED), DiodeWithRs
- Tube: Koren triode (12AX7/ECC83, 12AU7/ECC82), pentode (EL84, EL34)
- LDR: CdS photoresistor with asymmetric attack/release (VTL5C3/4 presets)
- JFET: Shichman-Hodges (2N5457, J201, 2N3819)
- MOSFET: Square-law (2N7000, BS170)
- OpAmp: Boyle macromodel, SimpleOpamp (741, TL072, NE5532)

### Layer 3: melange-solver 🚧

| OpenWurli File | Melange Target | Genericity | Effort | Status |
|---------------|---------------|------------|--------|--------|
| `dk_preamp.rs` | `solver::dk` | 15% generic | **Hard** | Not started |

**What's generic (extract as-is):**
- ✅ Matrix algebra (now in `nr::solve_linear_m`)
- ✅ Trapezoidal discretization pattern: `A = 2C/T + G`, `S = inv(A)` (in `companion`)
- ✅ Newton-Raphson on 2x2 kernel with Cramer's rule (in `nr`)
- 🚧 stamp_resistor / stamp_capacitor helper functions
- 🚧 Sherman-Morrison rank-1 update
- 🚧 Shadow subtraction framework

**What needs generalization:**
- Fixed `N=8` node count → const generic or code generation
- Hardcoded node indices → derived from netlist
- Hardcoded component values → from parsed netlist
- BJT model placement → from `NonlinearDevice` trait objects
- Circuit-specific `compute_k()` → generated from N_v, S, N_i matrices

**Planned modules:**
- `parser`: SPICE netlist parsing
- `mna`: Matrix stamp assembly
- `dk`: DK-method kernel extraction
- `solver`: Interpreted runtime solver
- `codegen`: Rust code generation

### Layer 4: melange-validate 🚧

| OpenWurli Source | Melange Target | Effort | Status |
|-----------------|---------------|--------|--------|
| `preamp-bench` measurement routines | `validate::measure` | Moderate | Not started |
| `spice/` methodology | `validate::spice_runner` | Moderate | Not started |
| Three-tier test pattern | `validate::tiers` | Low | Not started |

### Layer 5: melange-plugin 🚧

| OpenWurli Source | Melange Target | Effort | Status |
|-----------------|---------------|--------|--------|
| Voice allocation in `plugin/lib.rs` | `plugin::voice_pool` | Moderate | Not started |
| Oversampling logic | `plugin::oversample_decision` | Low | Not started |
| `CalibrationConfig` | `plugin::calibration` | Low | Not started |

---

## Extraction Order (Recommended)

1. ✅ **melange-primitives** — Start here. Immediate value, trivial extraction.
2. ✅ **melange-devices** (NonlinearDevice trait + BJT) — Enables solver work.
3. 🚧 **melange-solver** (MNA assembler + interpreted solver) — The core, start with interpreted mode.
4. 🚧 **melange-validate** (measurement routines) — Extract from preamp-bench.
5. 🚧 **melange-solver** (code generator) — The hard part, do last.
6. 🚧 **melange-plugin** — Extract as OpenWurli migrates to melange dependencies.

---

## Test Status

```
Layer 1: melange-primitives ................. 29 passed
Layer 2: melange-devices .................... 33 passed
Total: 62 tests passing
```

## Bug Fixes Applied (Post-Review)

### Critical Fixes

1. **Biquad HighShelf coefficient bug** (`filters.rs` line 330)
   - Was: `self.b2 = ...` assigned twice, missing `self.a2`
   - Fixed: Correct assignment to `self.a2`

2. **Tube model missing kvb term** (`tube.rs`)
   - Was: `E1 = Vpk + Vgk * mu` (missing kvb*Vpk^2)
   - Fixed: `E1 = Vpk + Vgk * mu + kvb * Vpk * Vpk`
   - Jacobian updated accordingly

3. **JFET polarity/sign convention** (`jfet.rs`)
   - Was: Incorrect vgst calculation for P-channel
   - Fixed: Proper `vp_eff` handling for P-channel (negate vp)

4. **NR solver heap allocation** (`nr.rs`)
   - Was: `Vec<Vec<f64>>` in `solve_linear_m` (allocation per NR iteration)
   - Fixed: Stack-allocated array with const generics

5. **Missing Jacobians**
   - JFET: Full Jacobian with gm and gds
   - MOSFET: Full Jacobian with gm and gds
   - Both verified numerically against finite differences

### Improvements

- Added step-size convergence check to all NR solvers
- Added configurable `clamp` parameter to `nr_solve_dk`
- Added numerical Jacobian verification tests
