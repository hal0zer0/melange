# Melange Review Summary

**Initial Review Date:** 2026-02-23
**Expert Panel Review:** 2026-03-02
**Status:** 731+ tests passing, 0 clippy warnings, 0 unsafe blocks

## Executive Summary

After five code review rounds (Feb 23) and a four-perspective expert panel review (Mar 2), melange is a **production-ready framework for automating circuit-to-plugin modeling**. The DK method implementation is mathematically sound, the codegen pipeline works end-to-end, and the test/validation infrastructure is rigorous. The expert panel consensus: "No other open-source tool does this."

---

## Review History

### Rounds 1-5: Code Review (2026-02-23)

1. **Initial** — Fixed parsing bugs, real-time safety (Vec allocations), solver trait mismatch
2. **Deep Audit** — Fixed diode sign error, DK kernel sign bug, BJT N_i stamping, inductor companion
3. **Panic Fuzzing** — Division by zero protection, unwrap() elimination, bounds checks
4. **Architecture** — Hot-loop optimization, documentation, limitations.md
5. **Polish** — Clippy clean, release metadata, workspace integration

### Round 6: Expert Panel Review (2026-03-02)

Four domain-expert perspectives evaluated the codebase independently:

| Perspective | Verdict | Score |
|---|---|---|
| Pro Plugin Developer (ships commercial VST/AU) | "Ship it for guitar pedals. Not yet for premium tube amps." | 7.5/10 |
| Academic DSP Researcher (DAFx papers) | "Mathematically sound. Genuine contribution." | 7/10 (DAFx paper) |
| Hardware Engineer (designs analog gear) | "If you're building a pedal, you'd be insane not to try this." | Ready for prototyping |
| DSP Language Designer (FAUST/Cmajor) | "Most rigorous codegen in open-source audio. I'd fork the IR." | 85% multi-target ready |

**Universally praised:** End-to-end pipeline, mathematical rigor, test coverage, zero-alloc audio path, Sherman-Morrison pots.

---

## Current Statistics

| Metric | v0.1.0 (Feb 23) | Current (Mar 2) |
|--------|-----------------|-----------------|
| Total Tests | 91 | 731+ |
| Clippy Warnings | 0 | 0 |
| Unsafe Blocks | 0 | 0 |
| Device Types | Diode | Diode, BJT, JFET, MOSFET, Tube, Op-amp |
| Max NL Dimensions | 2 | 16 |
| Codegen | No | Rust (Tera templates), nih-plug plugin |
| SPICE Validation | No | 6 circuits validated vs ngspice |
| Dynamic Controls | No | Pots (Sherman-Morrison) + Switches + Coupled Inductors |
| Oversampling | No | 2x/4x polyphase half-band IIR |

## Production Readiness by Crate

| Crate | Status | Notes |
|-------|--------|-------|
| melange-primitives | ✅ Production | Filters, oversampling, safe_exp, NR helpers |
| melange-devices | ✅ Production | Diode, BJT (GP), JFET, MOSFET, Tube, Op-amp |
| melange-solver | ✅ Production | MNA/DK, codegen, pots, switches, coupled inductors, subcircuits, DC OP, M<=16 |
| melange-validate | ✅ Production | 6 SPICE comparisons (RC, diode, BJT, JFET, MOSFET, op-amp) |
| melange-plugin | ⚠️ Stub | Plugin generation via codegen works; crate itself is placeholder |
| melange-cli | ✅ Production | compile, simulate, info commands |

---

## Actionable Items from Expert Panel

### Priority 1 — High Impact, Moderate Effort

#### ~~1.1 Subcircuit Flattening~~ ✅ DONE (2026-03-02)
**Flagged by:** Plugin Dev, Hardware Engineer
**Implemented:** `X` element expansion in parser with recursive substitution, node remapping, cycle detection (max depth 16), nested subcircuit support. 22 tests.
**Files:** `crates/melange-solver/src/parser.rs`

#### 1.2 AC Frequency Response Analysis
**Flagged by:** Plugin Dev, Hardware Engineer
**Problem:** Checking "does my circuit have the right gain at 1kHz?" requires generating code, compiling a plugin, loading in a DAW, and sweeping a sine wave.
**Action:** Add `melange analyze circuit.cir --freq-response 20 20000` CLI command. Sweep sine at each frequency through the solver, measure gain/phase. Output CSV or ASCII plot.
**Files:** `tools/melange-cli/src/main.rs`, new `analyze` subcommand
**Impact:** Saves hours per design iteration. The infrastructure already exists in melange-validate.

#### 1.3 2D JFET Model (Triode + Saturation Regions)
**Flagged by:** Plugin Dev, Academic, Hardware Engineer
**Problem:** Current 1D model ignores Vds entirely. Breaks Moog-style circuits, VCA stages, analog switches — anywhere Vds varies dynamically.
**Action:** Implement Shichman-Hodges 2D model: triode region (Vds < Vgs-Vp) + saturation region. Add to both runtime solver and codegen templates.
**Files:** `crates/melange-devices/src/jfet.rs`, `crates/melange-solver/src/codegen/templates/device_jfet.rs.tera`
**Impact:** Enables Moog ladder filters, JFET-based VCAs, solid-state tremolo circuits.

#### 1.4 2D MOSFET Model (Triode + Saturation Regions)
**Flagged by:** Plugin Dev, Academic, Hardware Engineer
**Problem:** Same as JFET — 1D saturation-only model ignores Vds. Circuits need source degeneration resistor as workaround.
**Action:** Implement Level 1 MOSFET with triode/saturation regions and channel-length modulation (lambda). 2D NR like BJTs.
**Files:** `crates/melange-devices/src/mosfet.rs`, `crates/melange-solver/src/codegen/templates/device_mosfet.rs.tera`
**Impact:** Enables class-AB output stages, CMOS analog switches, power MOSFET circuits.

### Priority 2 — Important, Larger Effort

#### ~~2.1 Transformer / Coupled Inductor Support~~ ✅ DONE (2026-03-02)
**Flagged by:** Hardware Engineer (BLOCKER for tube amp modeling)
**Implemented:** SPICE-standard `K L1 L2 coupling_coeff` syntax. Full trapezoidal companion model with symmetric mutual conductance stamps. Supported in parser, MNA, DK kernel, runtime solver, codegen IR, templates. Max 16 couplings, k in (0,1) exclusive. 49 tests (parser, MNA, codegen compile-and-run, behavioral step-up/step-down/weak coupling).
**Files:** `parser.rs`, `mna.rs`, `dk.rs`, `solver.rs`, `codegen/ir.rs`, `rust_emitter.rs`, all templates

#### 2.2 Multiple Outputs
**Flagged by:** Plugin Dev
**Problem:** All generated circuits have exactly one `output_node`. Can't tap multiple gain stages or build stereo circuits from a single netlist.
**Action:** Extend `CodegenConfig` to accept `output_nodes: Vec<usize>`. Emit per-output voltage extraction in `process_sample`. Plugin template generates multi-channel output.
**Files:** `crates/melange-solver/src/codegen/ir.rs`, `rust_emitter.rs`, plugin template
**Impact:** Multi-tap circuits, stereo encoding, parallel signal paths.

#### 2.3 Runtime Device Parameter Control
**Flagged by:** Plugin Dev
**Problem:** Device parameters (IS, BF, tube mu) are baked as compile-time constants. Can't A/B test transistor beta or model tube aging without recompiling.
**Action:** Promote selected device params to `CircuitState` fields (like pots). Requires re-evaluating NR device functions with state-carried params instead of constants.
**Files:** Codegen IR, device templates, plugin template
**Impact:** Tube aging simulation, transistor matching, expressive musical control over device character.

#### 2.4 Controlled Sources (E, G)
**Flagged by:** Hardware Engineer (partially — op-amp covers some use cases)
**Problem:** VCVS (E) and VCCS (G) elements not implemented. Needed for current mirrors, active filters, dependent sources in SPICE models.
**Action:** MNA stamp E as ideal VCVS (extra row/column), G as transconductance (simple G-matrix stamp). Linear elements — no NR dimension increase.
**Files:** `parser.rs`, `mna.rs`
**Impact:** Enables imported SPICE subcircuit models that use internal controlled sources.

### Priority 3 — Nice to Have / Research

#### 3.1 Symbolic Matrix Simplification
**Flagged by:** DSP Language Designer
**Problem:** Codegen emits raw numeric matrices. No symbolic pre-simplification (e.g., folding `1e-300 * sin(theta)` or eliminating known-zero products).
**Action:** Add optional CAS pass (SymPy via subprocess, or hand-rolled Rust symbolic algebra) between CircuitIR construction and emission. Simplify matrix expressions before emitting constants.
**Impact:** 10-50% NR overhead reduction on sparse circuits. More readable generated code.

#### 3.2 Device Model IR (Symbolic Expressions)
**Flagged by:** DSP Language Designer
**Problem:** Device models are locked in Tera templates (`device_bjt.rs.tera`). Every new target language requires rewriting all device templates.
**Action:** Define device behavior as symbolic expression trees in CircuitIR (e.g., `Expr::Exp(Expr::Div(v, n_vt))`). Emitters lower expressions to target syntax.
**Files:** New `crates/melange-solver/src/codegen/expr.rs`, refactor device templates
**Impact:** Adds a new target language in ~200 lines instead of ~2000. Enables auto-differentiation for Jacobians.

#### 3.3 Port-Hamiltonian Formulation
**Flagged by:** Academic Researcher
**Problem:** Trapezoidal rule is energy-stable for linear elements but the NR loop doesn't explicitly conserve Hamiltonian structure. Edge cases can leak energy.
**Action:** Research-level — reformulate DK state update as port-Hamiltonian system with discrete gradient. Guarantees passivity independent of device parameters.
**Impact:** Provable stability for arbitrary circuits. Strong DAFx paper angle.

#### 3.4 Adaptive Oversampling
**Flagged by:** Plugin Dev
**Problem:** Oversampling factor is fixed at compile time (1x/2x/4x). A Tube Screamer at 4x wastes CPU; a 12-stage fuzz at 2x may alias.
**Action:** Emit both 1x and 4x paths. Runtime switch based on input amplitude or user knob. Requires cascaded filter state management.
**Impact:** Better CPU/quality tradeoff. Standard in commercial plugins.

#### 3.5 SIMD / Batch Processing
**Flagged by:** Plugin Dev, DSP Language Designer
**Problem:** Generated code processes one sample at a time. No vectorization for offline rendering.
**Action:** Emit `process_block(input: &[f64], output: &mut [f64])` that processes N samples. Use `portable_simd` or emit aligned loads for LLVM auto-vectorization of the linear algebra.
**Impact:** 2-4x throughput for offline rendering. Marginal benefit for real-time (already fast enough).

#### 3.6 Implicit Multirate Integration
**Flagged by:** Academic Researcher
**Problem:** Fixed internal rate doesn't exploit timescale separation (fast diode switching vs slow BJT bias drift).
**Action:** Research-level — partition state variables by stiffness, integrate fast subsystem at higher rate. Requires partitioned DK kernel.
**Impact:** Reduces NR iterations from 5-10 to 1-2 per sample for stiff circuits.

#### 3.7 Hardware Validation
**Flagged by:** Hardware Engineer (BLOCKER for premium products)
**Problem:** All validation is SPICE-vs-melange (proves implementation). No measured-hardware-vs-melange (proves model fidelity). A 12AX7 tube model hasn't been compared to a real 12AX7.
**Action:** Measure real components (tube curves, transistor Ic-Vbe, diode I-V). Compare to melange device model output. Publish results.
**Impact:** Trust. The difference between "mathematically correct" and "sounds right."

---

## Competitive Position

| Alternative | Melange Advantage | Their Advantage |
|---|---|---|
| Cytomic CCS (proprietary) | Open-source, Rust, cross-platform | Symbolic optimization, battle-tested in commercial plugins |
| chowdsp_wdf (C++ WDF) | Automated (no manual circuit decomposition) | 30 years of WDF optimization lore, C++ ecosystem |
| ACME.jl (Julia) | Real-time capable, codegen, compiled output | Julia ecosystem, interactive exploration |
| NDKFramework (MATLAB) | No MATLAB license, end-to-end pipeline | MATLAB tooling, Simulink integration |
| NAM/AIDA-X (neural) | Interpretable, physical parameters, no training data | Can model unknown circuits from audio recordings |
| LiveSPICE (C#) | Code generation (not JIT), cross-platform | Visual schematic editor |

---

## Verification Commands

```bash
cargo test --workspace                    # 731+ tests
cargo clippy --workspace                  # 0 warnings
cargo build --release --workspace         # release build
cargo run -p melange-cli -- info circuits/wurli-preamp.cir  # circuit info
cargo run -p melange-cli -- compile circuits/wurli-preamp.cir -o /tmp/test-plugin  # codegen
```

---

## Best Use Cases Today

1. **Guitar pedal prototyping** — fuzz, overdrive, clipper, tone stack
2. **Tube amp modeling** — preamp stages, output transformers (coupled inductors), multi-stage with subcircuits
3. **EQ circuits** — passive/active with pots and switches (Pultec EQP-1A validated)
4. **Educational** — learning DK method with instant audio feedback
5. **Plugin rapid prototyping** — netlist to nih-plug binary in one command

## Not Ready For (Yet)

1. MOSFET/JFET circuits where Vds varies (1D models)
2. Runtime device parameter morphing (tube aging, transistor matching)
3. Multi-output circuits

---

*Last updated: 2026-03-02 (Post-implementation update: transformers + subcircuits)*
