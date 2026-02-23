# Extraction Guide: OpenWurli → Melange

This document tracks which OpenWurli components are being extracted into melange crates,
what modifications are needed, and the extraction status.

## Extraction Inventory

### Layer 1: melange-primitives

| OpenWurli File | Melange Target | Genericity | Effort | Status |
|---------------|---------------|------------|--------|--------|
| `filters.rs` | `primitives::filters` | 95% generic | Trivial | Not started |
| `oversampler.rs` | `primitives::oversampler` | 100% generic | Low (add 4x) | Not started |
| `variation.rs` | `primitives::variation` | 100% generic | Trivial | Not started |

**filters.rs changes:**
- Make DcBlocker cutoff configurable (currently hardcoded 20 Hz)
- Add peaking EQ and shelf Biquad types
- Otherwise verbatim extraction

**oversampler.rs changes:**
- Parameterize allpass coefficient tables (currently hardcoded for one rejection level)
- Add 4x support (cascade two 2x stages or dedicated 4x coefficients)
- Consider const-generic FACTOR parameter

**variation.rs changes:**
- None. Fully generic already.

### Layer 2: melange-devices

| OpenWurli File | Melange Target | Genericity | Effort | Status |
|---------------|---------------|------------|--------|--------|
| `bjt_stage.rs` | `devices::bjt` | 70% generic | Low | Not started |
| `tremolo.rs` (LDR part) | `devices::cds_ldr` | 60% generic | Low | Not started |
| `power_amp.rs` | `devices::power_amp` | 75% generic | Low | Not started |
| `pickup.rs` | `devices::capacitive_pickup` | 60% generic | Low | Not started |
| `speaker.rs` | `devices::speaker` | 50% generic | Low | Not started |

**bjt_stage.rs changes:**
- Remove `stage1()` / `stage2()` factory methods (Wurlitzer-specific constants)
- Add `BjtStageConfig` builder that derives parameters from component values
- The NR solver, soft-clip, and Miller filter are already reusable as-is
- Add `NonlinearDevice` trait implementation

**tremolo.rs (CdS LDR) changes:**
- Extract `CdsLdr` struct parameterized by (R_min, R_max, gamma, attack_tau, release_tau)
- Remove hardcoded Wurlitzer rate (5.63 Hz) and series resistances
- The asymmetric envelope + power-law resistance model is fully generic

**power_amp.rs changes:**
- Replace `new()` with configurable constructor (open_loop_gain, feedback_beta, crossover_vt, rail_limit)
- Add topology options (Class A / AB / B) if warranted
- The NR feedback solver and crossover model are generic

**pickup.rs changes:**
- Parameterize TAU, SENSITIVITY, MAX_Y (currently hardcoded to Wurlitzer values)
- The bilinear RC discretization algorithm is generic to any capacitive transducer

**speaker.rs changes:**
- Parameterize HPF/LPF cutoffs, polynomial coefficients, thermal tau
- Add `SpeakerConfig` struct

### Layer 3: melange-solver

| OpenWurli File | Melange Target | Genericity | Effort | Status |
|---------------|---------------|------------|--------|--------|
| `dk_preamp.rs` | `solver::dk` | 15% generic | **Hard** | Not started |

This is the hardest and most valuable extraction. The mathematical framework in dk_preamp.rs is clean, but it's hardcoded to an 8-node Wurlitzer topology.

**What's generic (extract as-is):**
- Matrix algebra (mat_vec_mul, mat_inverse, mat_add, mat_sub, mat_scale)
- Gauss-Jordan inverse
- Trapezoidal discretization pattern: `A = 2C/T + G`, `S = inv(A)`
- stamp_resistor / stamp_capacitor helper functions
- Newton-Raphson on 2x2 kernel with Cramer's rule
- Sherman-Morrison rank-1 update
- Bilinear companion model for series R-C
- Shadow subtraction framework

**What needs generalization:**
- Fixed `N=8` node count → const generic or code generation
- Hardcoded node indices → derived from netlist
- Hardcoded component values → from parsed netlist
- BJT model placement → from `NonlinearDevice` trait objects
- Circuit-specific `compute_k()` → generated from N_v, S, N_i matrices

**The performance challenge:**
Current: 8x8 arrays fit in registers. Generic: dynamic sizes would require heap allocation.
Solution: Code generation emitting const-generic Rust. See `docs/architecture.md`.

### Layer 4: melange-validate

| OpenWurli Source | Melange Target | Effort | Status |
|-----------------|---------------|--------|--------|
| `preamp-bench` measurement routines | `validate::measure` | Moderate | Not started |
| `spice/` methodology | `validate::spice_runner` | Moderate | Not started |
| Three-tier test pattern | `validate::tiers` | Low | Not started |

**preamp-bench extraction:**
The measurement functions (gain, sweep, harmonics, centroid, overshoot) are already well-factored.
Need a generic `DeviceUnderTest` trait to replace the hardcoded Wurlitzer signal chain.

**SPICE validation extraction:**
The testing methodology is highly generic but currently encoded as ad-hoc SPICE netlists.
Need: netlist generators, output parsers, comparison routines with tolerance bands.

### Layer 5: melange-plugin

| OpenWurli Source | Melange Target | Effort | Status |
|-----------------|---------------|--------|--------|
| Voice allocation in `plugin/lib.rs` | `plugin::voice_pool` | Moderate | Not started |
| Oversampling logic | `plugin::oversample_decision` | Low | Not started |
| `CalibrationConfig` | `plugin::calibration` | Low | Not started |

## Extraction Order (Recommended)

1. **melange-primitives** — Start here. Immediate value, trivial extraction.
2. **melange-devices** (NonlinearDevice trait + BJT) — Enables solver work.
3. **melange-solver** (MNA assembler + interpreted solver) — The core, start with interpreted mode.
4. **melange-validate** (measurement routines) — Extract from preamp-bench.
5. **melange-solver** (code generator) — The hard part, do last.
6. **melange-plugin** — Extract as OpenWurli migrates to melange dependencies.
