# Melange Roadmap

**Last Updated:** 2026-03-13
**Current:** 786 tests passing, 0 warnings, 0 unsafe blocks

---

## Audit Findings (2026-03-13)

Full hostile codebase audit identified issues in five categories. Items below are
prioritized by credibility impact (what would get us roasted publicly) first,
then numerical correctness, then engineering quality.

---

## Phase A — Credibility Fixes (Documentation-Code Alignment) ✅ Complete

All false claims removed or corrected (2026-03-13).

### ~~A.1 Remove phantom SPICE validation claims~~ ✅ Done
### ~~A.2 Fix BJT validation docs to reflect honest accuracy~~ ✅ Done
### ~~A.3 Remove dead feature claims~~ ✅ Done (KorenPentode/BoyleOpamp → doc(hidden), plugin/primitives headers fixed)
### ~~A.4 Fix glob re-export contradiction~~ ✅ Done (named re-exports in all crates)
### ~~A.5 Fix NR_SOLVER.md Jacobian formula~~ ✅ Done (clarifying note added)

---

## Phase B — Numerical Correctness

Issues where the math is wrong or numerically fragile.

### ~~B.1 DC OP solver: LU factorization~~ ✅ Done (2026-03-13)
### ~~B.2 DC OP voltage limiting: junction-aware logarithmic limiting~~ ✅ Done (2026-03-13)

### ~~B.3 NR voltage limiting: SPICE3f5-style pnjlim/fetlim~~ ✅ Done (2026-03-13)
- SPICE-style voltage-space limiting replaces flat per-dimension current clamping
- pnjlim (PN junction): logarithmic compression above vcrit, for diodes/BJTs/tubes
- fetlim (FET): threshold-aware limiting for JFETs/MOSFETs
- Per-device VCRIT constants precomputed from `pn_vcrit(vt, is) = vt * ln(vt / (sqrt(2) * is))`
- Scalar damping factor alpha applied to full Newton step: `i_nl -= alpha * delta`
- Both runtime solver and codegen

### ~~B.4 Diode linearized continuation beyond clamp~~ ✅ Done (2026-03-13)
### ~~B.5 Gummel-Poon qb() IKR term~~ ✅ Done (2026-03-13)
### ~~B.6 NR convergence max-norm~~ ✅ Done (2026-03-13)
### ~~B.7 Thermal voltage full precision~~ ✅ Done (VT_ROOM = 0.025851991)

### B.8 Tube Jacobian guard threshold too small for fractional exponents
- `e1.powf(ex-1.0)` with guard at `1e-30` — produces huge values when `ex < 1.0`
- Raise guard to `1e-10` or use safe formulation
- **Files:** `templates/rust/device_tube.rs.tera`
- **Severity:** Low-Moderate

### B.9 `DiodeWithRs` uses non-convergent single-step approximation
- Single fixed-point iteration for `I = Is*(exp((V-I*Rs)/(n*Vt))-1)`
- Error is O((g_d*Rs)^2) — useless for forward-biased diode with Rs > 10 ohm
- Add NR inner loop (3-5 iterations) or companion linearization
- **Files:** `crates/melange-devices/src/diode.rs`
- **Severity:** Low (only affects runtime solver with DiodeWithRs)

---

## Phase C — Validation & Testing

### ~~C.1 Add SPICE validation for JFET, MOSFET, op-amp~~ ✅ Done (2026-03-13)
- Op-amp inverting: passes (correlation 1.0, RMS ~0%)
- JFET common-source: passes (correlation 0.999, RMS 3.5%)
- MOSFET common-source: passes (correlation 0.99999997, RMS 0.03%, SNR 71dB)
- Runtime solver now supports all device types (JFET, MOSFET, Tube added)
- Tube: not yet added (requires codegen-based validation path)

### ~~C.2 Codegen vs runtime cross-validation test~~ ✅ Done (2026-03-13)
- 6 tests: diode (3e-11), BJT CE (8e-10), JFET CS (1e-9), MOSFET CS (6e-6), triode CC (1e-6), SM sanity
- Compile-and-run: generates code, compiles with rustc, compares sample-by-sample

### ~~C.3 Sherman-Morrison correctness test~~ ✅ Done (2026-03-13)
- Full sweep comparison: SM-corrected S and K matrices vs full matrix rebuild
- Tests in `crates/melange-solver/tests/pot_tests.rs`

### C.4 Improve test signals
- Add step response, multi-frequency (chirp), silence-to-signal transition
- Extend simulation length to 100ms+ (error accumulation)
- Increase PWL density for diode test (25 points is a triangle wave)
- **Severity:** Medium

### C.5 Fail-loud when ngspice unavailable
- Replace `if !is_ngspice_available() { return; }` with `#[ignore]` or CI gating
- Current pattern silently passes all SPICE tests without running them
- **Severity:** Medium

### ~~C.6 Fix signal length truncation in comparison~~ ✅ Done (2026-03-13)
- Added `log::warn!` when signal lengths differ by >1 sample

### ~~C.7 Oversampling correctness test~~ ✅ Done (2026-03-13)
- Matrix recomputation verified at internal rate (1x vs 2x)
- 2x and 4x compile-and-run tests pass

---

## Phase D — Engineering Quality

### ~~D.1 Remove panics from library code~~ ✅ Done (2026-03-13)
- `CircuitSolver::new()` → `Result<Self, SolverError>`
- `process_block()` → truncate to shorter length
- `debug_assert!(false)` → `log::warn!` + skip
- `RustEmitter::new()` → `Result<Self, CodegenError>`

### D.2 Encapsulate MnaSystem fields
- Make `g`, `c`, etc. private; add `stamp_input_conductance(node, conductance)` method
- Prevents downstream users from corrupting matrices
- **Files:** `crates/melange-solver/src/mna.rs`
- **Severity:** Medium

### D.3 Guard generated code edge cases
- Switch position R=0 → division by zero in generated code
- `dc_block_r` goes negative for sample_rate < 31.4 Hz → clamp to [0, 1)
- ~~`fmt_f64` emits `f64::NAN` for NaN matrix entries~~ ✅ Done
- ~~Circuit name not sanitized for Rust identifiers~~ ✅ Done (control chars replaced)
- **Files:** codegen/rust_emitter.rs, state.rs.tera
- **Severity:** Medium (2 of 4 sub-items remain)

### D.4 Replace `Vec<Vec<f64>>` with flat matrix in MNA
- Each row is separate heap allocation; DK kernel already uses flat arrays
- **Files:** `crates/melange-solver/src/mna.rs`
- **Severity:** Low (performance, not correctness)

### ~~D.5 Parser hardening~~ ✅ Done (2026-03-13)
- Missing `.ends` → error; V/I source self-connection check; unknown directives → `log::warn!`

### ~~D.6 Replace `eprintln!` in melange-validate with `log::warn!`~~ ✅ Done (2026-03-13)

---

## Priority 1 — High Impact, Moderate Effort (Pre-Audit) ✅ Complete

### ~~1.1 AC Frequency Response Analysis~~ ✅ Done
### ~~1.2 2D JFET Model~~ ✅ Done (2026-03-12)
### ~~1.3 2D MOSFET Model~~ ✅ Done (2026-03-12)
### ~~1.4 Subcircuit Flattening~~ ✅ Done (2026-03-02)
### ~~1.5 Transformer / Coupled Inductor Support~~ ✅ Done (2026-03-02)
### ~~1.6 Multi-Output (Stereo) Support~~ ✅ Done (2026-03-12)

---

## Priority 2 — Important, Larger Effort (Pre-Audit) ✅ Complete

### ~~2.1 Runtime Device Parameter Control~~ ✅ Done (2026-03-13)
- IS, BF, MU, IDSS, KP, etc. promoted to CircuitState fields
- NR solver reads `state.device_N_param` instead of constants
- 16 tests verify compile-and-run with parameter changes

### ~~2.2 Controlled Sources (E, G)~~ ✅ Done (2026-03-13)
- VCVS (E): voltage-controlled voltage source, Norton equivalent in MNA
- VCCS (G): voltage-controlled current source, direct G matrix stamp
- 22 tests including compile-and-run integration

### ~~2.3 Line Continuation Fix~~ ✅ Done

---

## Priority 3 — Research / Nice to Have

- **3.1 Symbolic Matrix Simplification** — Fold constants, eliminate known-zero products (10-50% NR speedup)
- **3.2 Device Model IR** — Symbolic expression trees instead of locked Tera templates
- **3.3 Port-Hamiltonian Formulation** — Provable stability, strong DAFx paper angle
- **3.4 Adaptive Oversampling** — Runtime 1x/4x switch based on input amplitude
- **3.5 SIMD / Batch Processing** — `process_block()` for 2-4x offline throughput
- **3.6 Implicit Multirate Integration** — Partition by stiffness, fewer NR iterations
- **3.7 Hardware Validation** — Measure real components vs. melange models

---

## OpenWurli Integration

- Validate wurli-preamp via SPICE comparison
- Benchmark M=5 vs M=2 for 64-voice performance
- M>16 iterative/sparse NR (deferred)

---

## Deferred

- **Multi-Language Codegen** — C++, FAUST, Python/NumPy, MATLAB/Octave (Emitter trait ready)
- **Phase 6a/6b** — Type safety: NodeIdx newtype, field visibility
- **Phase 7** — Crate split: melange-parser, melange-codegen

---

## Known Bugs

| Bug | Severity | Workaround |
|-----|----------|------------|
| ~~Line continuation (`+`) not joined~~ | ~~Medium~~ | ✅ Fixed |
| Purely resistive nonlinear circuits oscillate | Low | Always include at least one capacitor |
| ~~Several integration tests `#[ignore]`d~~ | ~~Low~~ | ✅ Fixed (runtime solver supports all devices) |
| ~~`nr_solve_dk` assumes diagonal device Jacobian~~ | ~~Medium~~ | ✅ Fixed (runtime uses Gaussian elimination) |
| ~~Runtime solver panics on M>8~~ | ~~Medium~~ | ✅ Fixed (`CircuitSolver::new()` returns `Result`) |
| ~~`DeviceIR` legacy enum missing JFET/MOSFET~~ | ~~Low~~ | ✅ Fixed (legacy `DeviceIR` removed entirely) |
| ~~`build_legacy_devices` only emits first diode/BJT~~ | ~~Low~~ | ✅ Fixed (removed; `device_slots` is the sole source of truth) |

---

*The spice must flow.*
