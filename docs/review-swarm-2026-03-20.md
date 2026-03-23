# Multi-Perspective Review Swarm — 2026-03-20

5 specialist agents reviewed the melange codebase in parallel from different expert perspectives.

## CRITICAL — Correctness or Safety

| ID | Source | Finding | Fix |
|----|--------|---------|-----|
| S-1 | Security | **No upper bound on circuit size (N)** — crafted netlist with 50K nodes causes OOM from O(N^2) matrices | Add `MAX_N=512` check in `MnaSystem::from_netlist()` |
| S-2 | Security | **Nodal codegen path doesn't enforce MAX_M** — `from_mna()` skips the M<=16 check | Add MAX_M check in `ir.rs:from_mna()` |
| S-3 | Security | **HTTP response body read without size limit** — malicious URL causes OOM | Add `.take(10MB)` in `cache.rs:fetch_url_sync` |
| S-4 | Rust API | **`unwrap()` in codegen emitter hot path** — `rust_emitter.rs:5130` panics on malformed IR | Return `CodegenError` |

## HIGH — Blocks Real-World Usage

| ID | Source | Finding | Fix |
|----|--------|---------|-----|
| H-1 | DSP | **`powf()` in tube NR hot path** — 300 libm calls/sample worst case at 96kHz | Add `tube_evaluate()` that caches `e1.powf(ex)`, add `fast_powf` |
| H-2 | Plugin | **Pot params use linear range** — 500-500k compressed into first 1% of knob | Use `FloatRange::Skewed` with log factor |
| H-3 | Plugin | **Switch `rebuild_matrices()` on audio thread** — O(N^3) per sample for Pultec | Move switch reads outside per-sample loop |
| H-4 | DSP | **No FTZ/DAZ flag management** — denormal slowdowns in DC block filter | Set MXCSR in plugin `initialize()` |
| H-5 | Analog | **GP base current not divided by qb** — beta too high at high injection, ~10% Q-point error | Divide forward ideal Ib component by qb |
| H-6 | Plugin | **No oversampling latency reporting** — DAW delay compensation broken | Report filter latency via nih-plug |

## MEDIUM — Significantly Helps Users

| ID | Source | Finding | Fix |
|----|--------|---------|-----|
| M-1 | Analog | **Hard cutoff at MOSFET/JFET threshold** — zero Jacobian causes NR chatter | Add weak exponential subthreshold tail |
| M-2 | Analog | **Koren Ip=0 for Vpk<=0** — NR stalls on heavily clipped tube circuits | Soft floor at `vpk.max(1e-3)` |
| M-3 | DSP | **2V global damping too aggressive for tube internals** — overdamps 250V plate swings | Scale threshold relative to supply voltage |
| M-4 | DSP | **`.ln()` without fast approximation** in tube softplus | Add `fast_ln()` matching `fast_exp()` |
| M-5 | DSP | **State updated before NaN check** — corrupts `v_prev`/`i_nl_prev` | Defer state write until after finite check |
| M-6 | Security | **Unrecognized .model params silently ignored** — typo `BFF=200` uses default BF=100 | Warn on unrecognized param names |
| M-7 | Rust API | **No unified error type** — 5 independent error enums, no `From` conversions | Add `MelangeError` wrapper or `From` chain |
| M-8 | Rust API | **All struct fields `pub`** on MNA/DK/Solver — semver hazard, invariant violations | Make fields private with accessors |
| M-9 | Rust API | **`tera` unconditionally pulled in** — 20+ transitive deps for users who only want runtime | Feature-gate behind `codegen` feature |
| M-10 | Plugin | **Plugin display name doesn't split underscores** | Fix `capitalize_word` to split on `_` too |

## LOW — Polish and Nice-to-Haves

| ID | Source | Finding |
|----|--------|---------|
| L-1 | Analog | Missing NR (reverse emission coefficient) in BJT |
| L-2 | Analog | Missing ISC/NC (collector-base leakage) in BJT |
| L-3 | Analog | No temperature dependence (germanium fuzz, tube warmup) |
| L-4 | Analog | Koren overestimates 12AX7 current at Vgk=0 by ~3x |
| L-5 | DSP | No SIMD optimization of matrix-vector multiplies |
| L-6 | DSP | No per-sample CPU budget monitoring |
| L-7 | Plugin | No wet/dry mix parameter option |
| L-8 | Plugin | No mono IO layout option |
| L-9 | Plugin | No `--name` flag for plugin display name |
| L-10 | Rust API | `NodalSolver::new()` returns `Self` not `Result` |
| L-11 | Rust API | `NodalSolver` not re-exported from crate root |
| L-12 | Rust API | `DeviceSlot` in codegen module but used by runtime |
| L-13 | Security | Hardcoded `[0.0; 16]` instead of `[0.0; MAX_M]` in solver |
| L-14 | Security | No element count limit in parser |
| L-15 | Plugin | Excessive blank lines in generated code |
| L-16 | Plugin | No .gitignore/README in generated plugin project |

---

## Agent Reports (Full)

### Agent 1: Plugin Developer

**Issues:**
- I1. Display name `capitalize_word` only splits on hyphens, not underscores (`plugin_template.rs:377-380`)
- I2. Pot params use `FloatRange::Linear` for 500-500k range — should be logarithmic (`plugin_template.rs:126-131`)
- I3. Excessive blank lines in generated code (18% blank)
- I4. `set_sample_rate()` resets transient state but doesn't re-run DC OP for inductor circuits
- I5. Switch `rebuild_matrices()` is O(N^3) called inside per-sample loop on audio thread
- I6. VST3 ID from XOR folding has collision risk

**Gaps:**
- G1. No latency reporting for oversampling (DAW delay compensation)
- G2. No wet/dry mix parameter
- G3. No mono IO layout option (hardcoded stereo)
- G4. No `--name` flag for plugin display name
- G5. No .gitignore or rust-toolchain.toml in generated project
- G6. No README/build instructions saved in project directory
- G7. melange-plugin crate has helpers (log_param_map etc.) but generated projects don't depend on it
- G8. No diagnostic output/logging exposed in plugin template

**Improvements:**
- P1. Use FloatRange::Skewed with log factor for pot params (min/max ratio > 10)
- P2. Move switch reads outside per-sample loop
- P3. Add output_scale hint in plugin format (tube circuits output >1V)
- P4. Collapse multiple blank lines in generated code
- P5. Add `#[inline(never)]` to set_sample_rate/rebuild_matrices
- P6. Document process_sample API with usage example
- P7. Document `--cfg melange_precise_exp` feature in generated code
- P8. Consider `#[repr(C)]` on CircuitState for FFI
- P9. Expose circuit metadata as constants (title, pot names, device types)

### Agent 2: Analog Circuit Expert

**Issues:**
- I1. JFET: discontinuous second derivative at triode-saturation boundary (minor NR chatter)
- I2. MOSFET/JFET: hard cutoff at Vgs=Vt with zero derivative — non-physical, NR convergence risk
- I3. Koren Ip=0 for Vpk<=0 — hard cutoff with zero Jacobian, convergence failure risk
- I4. GP base current not divided by qb — Ib too high at high injection
- I5. Codegen diode uses hard clamp; runtime uses linearized continuation — inconsistency

**Gaps:**
- G6. No temperature dependence (tube warmup, germanium fuzz)
- G7. No transit time / diffusion capacitance (bias-dependent)
- G8. No reverse emission coefficient NR for BJT
- G9. No ISC/NC (collector-base leakage)
- G10. MOSFET Level 1 only — no velocity saturation, short-channel effects
- G11. Pentode model experimental/unusable (3D but solver max 2D)
- G12. No noise models

**Improvements:**
- P13. Add soft exponential tail at MOSFET/JFET cutoff
- P14. Allow small negative Vpk current in Koren (soft floor at 1mV)
- P15. Add NR (reverse emission coefficient) to BJT
- P16. Divide GP base current by qb
- P17. Validate tube models against measured plate curves
- P18. SPICE validation for tube circuits, push-pull, inductors, switches at various positions

### Agent 3: DSP / Real-Time Audio Engineer

**Issues:**
- I1. `powf()` in tube NR hot path — 70ns each, 300 calls worst case (HIGH)
- I2. `.ln()` in tube softplus without fast approximation
- I3. `std::mem::swap` of large arrays in BE fallback pollutes L1 cache
- I4. 2V global damping too aggressive for tube 250V plate swings
- I5. DC block filter vulnerable to denormals without FTZ/DAZ

**Gaps:**
- G1. No FTZ/DAZ flag management
- G2. No SIMD optimization of matrix-vector multiplies
- G3. No adaptive CPU budget mechanism
- G4. No ISA annotation (`#[target_feature]`)
- G5. Oversampling phase/ripple undocumented

**Improvements:**
- P1. Cache powf via tube_evaluate (saves 200 powf/sample)
- P2. fast_powf for fixed exponents (x^1.4 via single exp)
- P3. fast_sqrt/fast_ln for generated code
- P6. Denormal flush in DC block filter
- P8. Defer state update until after NaN check

### Agent 4: Rust API / Library Design

**Issues:**
- I1. `unwrap()` in emitter at rust_emitter.rs:5130
- I2. NodalSolver::new() returns Self not Result
- I3. NodalSolver not re-exported from crate root
- I4. DeviceSlot in codegen::ir but used by runtime (circular dep risk)
- I5. All struct fields pub on CircuitSolver/MnaSystem/DkKernel
- I6. `partial_cmp().unwrap()` on f64 in mna.rs:1969

**Gaps:**
- G1. No unified error type across pipeline (5 independent enums)
- G2. No newtype wrappers (NodeIdx, SampleRate, etc.)
- G3. No set_sample_rate/set_pot on runtime solvers
- G4. CircuitSolver::process_sample returns single f64 (no multi-output)
- G5. Emitter trait returns String — not multi-file capable
- G6. melange-primitives not no_std

**Improvements:**
- P1. Put tera behind codegen feature flag (~20 transitive deps)
- P2. Move DeviceSlot/DeviceParams into shared types module
- P3. Add unified MelangeError or From chain
- P4. Make MNA/DK/Solver fields private with accessors
- P5. Add #[non_exhaustive] to all public enums
- P6. Implement Display on Element
- P7. Feature-gate serde
- P8. Use thiserror consistently

### Agent 5: Security / Robustness Auditor

**Issues:**
- I1. No upper bound on N — 50K nodes causes multi-GB allocation
- I2. Nodal from_mna() doesn't enforce MAX_M
- I3. Temp files not cleaned on panic paths (low severity)
- I4. HTTP response body read without size limit
- I5. Hardcoded `[0.0; 16]` instead of `[0.0; MAX_M]`

**Gaps:**
- G1. Unrecognized .model params silently ignored (typo risk)
- G2. No subcircuit expansion element count limit (combinatorial explosion)
- G3. parse_value accepts negatives — some model params not validated
- G4. No NaN guard in codegen NR loop (runtime has it)
- G5. Runtime solver doesn't enforce MAX_M
- G6. No element count limit in parser

**Well-defended areas:**
- parse_value rejects NaN/Inf
- Component values validated positive
- Self-connection detection
- Duplicate names rejected
- Subcircuit cycle detection
- DC OP NaN guard + convergence chain
- Runtime NaN recovery + input sanitization + output clamping
- Title sanitization prevents codegen injection
- Cache path uses hash (no path traversal)
