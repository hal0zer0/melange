# Melange Multi-Agent Review — Unified Findings

**Last Updated:** 2026-03-01
**Reviews Conducted:** 4 rounds (initial code review Feb 23, bug-hunt Feb 28, architecture review Mar 1, user persona reactions Mar 1)

---

## Review History

### Round 1 — Code Review (2026-02-23, commit c981d95)
5 specialized agents (Device Models, MNA/DK, NR Solver, Codegen, Parser/CLI).
Found 3 HIGH bugs (LDR div/0, JFET vp=0, model param validation), 12 MEDIUM, 11 LOW.
All HIGH/MEDIUM fixed. See section below for details.

### Round 2 — Sample Rate Integration Review (2026-03-01, commit 94ba603)
4 agents reviewed runtime sample rate support integration (templates, emitter, IR+tests, DC OP+MNA).
Found 0 bugs, 12 gaps (4 medium, 7 low, 1 very-low). All fixed in commit 4b4e723.

### Round 3 — High-Level Architecture Review (2026-03-01, commit 4b4e723)
4 expert perspectives: DSP/Audio Architect, Compiler/Codegen, Circuit Simulation, Rust Architecture.

| Reviewer | Rating | Top Strength | Top Weakness |
|----------|--------|-------------|-------------|
| DSP/Audio Architect | 7/10 | DK method is the right algorithm; real-time safety done right | No oversampling in generated code |
| Compiler/Codegen | 7/10 | Clean pipeline decomposition; IR at right abstraction level | Device equations duplicated in 3 places |
| Circuit Simulation | 7.5/10 | Sound math foundation; multi-strategy DC OP | Device coverage gap (no tubes/JFET/MOSFET in codegen) |
| Rust Architecture | 6/10 | Correct math; great test coverage | melange-solver monolith; weak type safety |

**Consensus Overall: 7/10** — Sound foundation, needs device coverage + anti-aliasing + API cleanup.

---

## Architecture Review Findings (Round 3)

### Consensus Strengths

1. **DK method is the right choice** for small nonlinear audio circuits (N=5-20, M=1-8). The N-to-M reduction concentrates compute into the smallest possible NR solve.
2. **Pipeline decomposition** (Netlist → MNA → DK → IR → Emitter) is well-motivated by the physics. Stage boundaries are in the right places.
3. **Real-time safety is genuine** — generated code is allocation-free, NaN-safe, stack-only. Runtime uses enum dispatch (no vtable). No unsafe.
4. **Testing is excellent** — SPICE validation against ngspice, 585+ tests, codegen compile-and-run verification.
5. **Sherman-Morrison pots** and **runtime sample rate** are correctly designed and implemented.
6. **CircuitIR** is at the right abstraction level — serializable, language-agnostic, clean Emitter trait.
7. **DC OP multi-strategy solver** follows SPICE best practices (NR → source stepping → Gmin → fallback).

### Consensus Weaknesses

#### W1: No Oversampling in Generated Code (ALL 4 reviewers) — FIXED
- ~~Trapezoidal rule has no anti-aliasing; nonlinearities generate harmonics that fold back into audio band~~
- **Fixed**: Self-contained polyphase half-band IIR oversampling (2x/4x) emitted inline in generated code
- `CodegenConfig.oversampling_factor` = 1, 2, or 4; matrices recomputed at internal rate from G+C
- 4x uses cascaded 2x: outer 2-section (~60dB) + inner 3-section (~80dB); no runtime crate dependencies

#### W2: Device Equations Triplicated (3 reviewers) — PARTIALLY FIXED
- ~~Same diode/BJT math in: `melange-devices` (runtime), Tera templates (codegen), `dc_op.rs` (DC OP solver)~~
- **Fixed**: `dc_op.rs` now imports device equations from `melange-devices` instead of local copies
- **Remaining**: Tera templates still have duplicated equations (necessary — generated code must be self-contained)
- Templates carry canonical-source comments pointing to `melange-devices`

#### W3: Missing Device Coverage (3 reviewers) — JFET DONE
- ~~No tubes (core use case for guitar amps), JFET, MOSFET in codegen~~
- **Fixed**: JFET codegen complete — 1D saturation-only model (Id = IDSS*(1-Vgs/Vp)^2)
- **Remaining**: MOSFET codegen, tube/triode codegen (Koren model needs grid current rewrite)

#### W4: melange-solver is a Monolith (2 reviewers) — PARTIALLY FIXED
- Parser, MNA, DK, solver, DC OP, codegen IR, emitter, templates — all in one crate
- ~~`pub use module::*` glob re-exports dump everything into flat namespace~~
- **Fixed**: Explicit named re-exports in `lib.rs` (no more glob pollution)
- **Remaining**: Crate splitting deferred until compile time becomes a bottleneck

#### W5: Weak Type Safety (2 reviewers) — PARTIALLY FIXED
- `Vec<Vec<f64>>` matrices with no dimension checking
- Bare `usize` for node indices (two conventions: 0=ground vs 0=first-non-ground)
- All struct fields pub with no invariant enforcement
- **Fixed**: Glob re-export cleanup (Phase 6c)
- **Deferred**: NodeIdx newtype (6a) and field visibility (6b) — high refactoring cost for internal crate

#### W6: eprintln! Diagnostics in Library Code (2 reviewers) — FIXED
- ~~`dk.rs` condition number warning, `mna.rs` sample rate warning, `ir.rs` DC OP warning all print to stderr~~
- **Fixed**: All 3 occurrences replaced with `log::warn!` via `log` crate

#### W7: Single Input / Single Output (1 reviewer)
- Codegen assumes exactly one input node and one output node
- Real circuits need multiple I/O, different impedance levels
- INPUT_RESISTANCE is a compile-time constant, not runtime-adjustable
- **Fix effort: MEDIUM** — generalize CodegenConfig + templates

#### W8: No Optimization Pass on IR (1 reviewer) — FIXED
- ~~No dead code elimination (zero K entries still generate multiplications)~~
- **Fixed**: Systematic sparsity analysis in IR (`SparseInfo` struct); zero entries skipped uniformly in A_neg, N_v, K, S*N_i emission

### Strategic Assessment

#### Competitive Position
- **Strongest niche**: circuit design exploration + parameter variation (neural models can't do this)
- **Not competing on**: fixed-topology hardware cloning (neural amp modeling is commoditizing this)
- **Competitors**: RT-WDF (Chowdhury), Faust circuit libs, ACME.jl

#### Scaling Limits
- MAX_M=8 blocks full tube preamps (4 triodes = M=8 minimum, no room for diodes)
- Single-input/single-output limits real-world circuit complexity
- No subcircuit expansion (parser rejects .subckt X instances)

---

## Round 1 Bug Fixes (Status)

### HIGH Severity — All Fixed
- [x] **H1**: LDR division by zero — added .max(1e-12) guard
- [x] **H2**: JFET vp=0 division by zero — added constructor validation
- [x] **H3**: Codegen model param validation — IS/N/BF/BR checked in build_device_info()

### MEDIUM Severity — All Fixed
- [x] M1: CodegenConfig input_resistance validation
- [x] M2: NR solve_md() NaN recovery
- [x] M3: Singular Jacobian fallback unified
- [x] M4: solve_md() step-size convergence check added
- [x] M5: MOSFET constructor validation
- [x] M6: BJT Jacobian quotient-rule fallback
- [x] M7: Parser AC phase validation
- [x] M8: Unknown directive warning
- [x] M9: Ground node I/O rejected in CLI
- [x] M10: CLI unwrap panics fixed
- [x] M11: CLI sample_rate/tolerance validation
- [x] M12: MNA inductor comment clarified

---

## Round 2 Gap Fixes (Status)

All 12 gaps fixed in commit 4b4e723:
- [x] G/C verification in IR round-trip test
- [x] G/C verification in IR field-match test
- [x] Pot + set_sample_rate compile-and-run test
- [x] Inductor + set_sample_rate compile-and-run test
- [x] Use MAX_M constant instead of hardcoded 8
- [x] Fix CodegenConfig default max_iterations (20 → 100)
- [x] Fix stale comments in rust_emitter.rs
- [x] Add inductor state zeroing in set_sample_rate
- [x] Add op-amp model type "OA" validation
- [x] Replace panic! with defensive fallback in solver
- [x] Replace assert! with fallback in build_discretized_matrix

---

## Recommended Fix Priority (Architecture Gaps)

### Completed
1. ~~**W6**: Replace eprintln! with log crate~~ — **DONE**
2. ~~**W8**: Add sparsity-aware emission~~ — **DONE**
3. ~~**W1**: Oversampling integration in codegen (2x/4x)~~ — **DONE**
4. ~~**W3**: JFET codegen (1D saturation-only)~~ — **DONE**
5. ~~**W2**: Device equation dedup in dc_op.rs~~ — **DONE** (templates remain duplicated by necessity)
6. ~~**W4**: Glob re-export cleanup~~ — **DONE** (crate split deferred)

### Remaining — Device & Model
7. **W3**: Tube/triode codegen (2D, Koren model with grid current — core guitar amp use case)
8. **W3**: MOSFET codegen (1D, Level 1 — similar to JFET)
9. BJT upgrade to Gummel-Poon (Early effect, high injection — addresses 35% RMS gap)

### Remaining — Multi-Language Codegen
The `Emitter` trait + `CircuitIR` pipeline supports adding new language backends. Priority:

10. **C++ emitter** — 90%+ of shipping audio plugins use C++/JUCE. Must-have for pro adoption. Medium effort (needs full feature set: oversampling, pots, runtime SR).
11. **FAUST emitter** — compiles to 30+ targets (VST, AU, LV2, WASM, embedded). SPICE→FAUST is unique. Small effort (~200 LOC), massive multiplier.
12. **Python (NumPy) emitter** — largest prototyping/education audience. Small effort, nearly identical to MATLAB.
13. **MATLAB/Octave emitter** — academic use case, direct Hack Audio competitor. Small effort (matrix ops are native).

Note: FAUST/Python/MATLAB emitters don't need oversampling or pot SM corrections (regenerate instead). Simpler than Rust/C++.

### Remaining — Structural (Deferred)
14. **W5**: Type-safe matrices and node indices (NodeIdx newtype, field visibility — deferred)
15. **W4**: Split monolith (extract melange-parser, melange-codegen — deferred)
16. **W7**: Multi-input/output support
17. Subcircuit expansion
18. Raise MAX_M beyond 8 (iterative solver for large M)

---

## Verified Correct (Confirmed Across All Reviews)

- K matrix sign convention (naturally negative, NO extra negation)
- NR Jacobian formula and update step (both voltage-domain and current-domain forms)
- N_I matrix transposition in codegen (M×N)
- Trapezoidal integration for both linear and nonlinear elements
- DC operating point companion formulation (G_aug = G_dc - N_i·J_dev·N_v)
- Sherman-Morrison pot correction math
- Runtime sample rate recomputation from G+C
- Block-diagonal Jacobian assembly for heterogeneous devices
- Per-device model parameters in codegen
- NaN/Inf safety in process_sample with full state reset

---

## Round 4 — User Persona Reactions (2026-03-01)

5 simulated user personas evaluated Melange (with planned features assumed complete) against the competitive landscape. The closest commercial alternative is Hack Audio's Point-to-Point Modeling ($199, closed source, proprietary format). Open alternatives: LiveSPICE (GPL, C#, Windows-only), chowdsp_wdf (BSD-3, WDF not DK, no codegen), dkmethod (dormant, diodes only).

### Personas

| Persona | Profile | Excited? | Would Use? | Top Concern |
|---------|---------|----------|------------|-------------|
| **Marcus** | Pro plugin dev, 8 yrs, 12 shipped plugins, C++/JUCE | Yes — time savings | Yes, for prototyping | NR failure behavior in live use |
| **Jake** | Pedal builder, 40 designs, LTspice daily, minimal coding | Extremely | Immediately | Toolchain friction + BJT accuracy |
| **Dr. Moreira** | Academic, WDF/state-space papers, teaches grad course | Cautiously | Teaching + research | Needs hardware validation, not just SPICE |
| **Aisha** | Indie dev/YouTuber, 15k subs, DIY plugin tutorials | Mind-blown | Content goldmine | "Four months of my life" existential moment |
| **Dave** | Senior DSP engineer, 15 yrs, UA/NDSP-tier company | Impressed, won't ship it | Internal prototyping only | Ebers-Moll / model accuracy gap |

### Universally Praised

1. **SPICE netlist input** — the killer feature. Every persona highlighted that using standard `.cir` files means zero lock-in and access to the entire SPICE GUI ecosystem (LTspice, KiCad, Qucs, TINA-TI, CircuitLab, Micro-Cap 12, etc.). No other tool in this space accepts standard netlists.
2. **Sherman-Morrison pot updates** — recognized as mathematically elegant and practically correct by all technical personas.
3. **Zero-alloc audio path** — "tells me someone here actually knows what real-time means" (Dave).
4. **DC OP convergence chain** — NR → source stepping → Gmin stepping recognized as textbook correct.
5. **SPICE validation pipeline** — unique among all competitors; valuable for both development and pedagogy.
6. **CircuitIR + Emitter trait** — clean architecture that enables multi-language output from a single pipeline.

### Universally Concerning

1. **BJT model accuracy (35% RMS)** — flagged by all 5 personas. Ebers-Moll lacks Early effect; pro engineer called it "a dealbreaker." Root cause: model-level difference (Ebers-Moll vs. ngspice's Gummel-Poon), not a bug. Fix: upgrade to Gummel-Poon with Early effect.
2. **Tube model maturity** — Koren equation without grid current models "a polite tube that doesn't exist." Multiple personas noted this is the core guitar amp use case.
3. **M=8 ceiling** — caps practical circuits at roughly a two-stage preamp. Full amp stacks need M>8.

### Persona-Specific Insights

**Marcus (pro dev):** Wants to know exactly what happens when NR fails mid-performance. Needs C++ output to work with existing JUCE projects. Would test tube stage first against hand-derived implementation.

**Jake (pedal builder):** Has 40 existing LTspice schematics ready to go — immediate value if onboarding is smooth. Biggest barrier is Rust toolchain setup, not the tool itself. Wants a "getting started in 5 minutes" guide.

**Dr. Moreira (academic):** Immediately useful for teaching — replaces hand-rolled MATLAB MNA in grad courses. Values the math documentation and sign convention documentation ("reproducibility in this field is appalling"). Wants hardware measurement validation, not just SPICE-vs-SPICE. Sees publication potential in the Sherman-Morrison pot technique.

**Aisha (YouTuber):** Sees 4-5 video series immediately. Key insight: "makes the floor lower and the ceiling higher" — beginners get working plugins, experts push further. Melange doesn't obsolete DSP knowledge, it democratizes the starting point. The math docs feed educational content.

**Dave (senior engineer):** Most critical. Missing: Early effect, grid current, plate curve fitting from hardware measurements, transformer saturation, power supply sag, component tolerance/aging, thermal drift. Fair assessment: "serious research tool and legitimately impressive hobbyist project" but not shippable at premium plugin tier yet. Would use for internal rapid prototyping.

### Competitive Position (User Perspective)

| | **Melange** | **Hack Audio PtP** | **LiveSPICE** | **chowdsp_wdf** |
|---|---|---|---|---|
| **Cost** | Free | $199 ($99 sale) | Free (GPL) | Free (BSD-3) |
| **Input** | Standard SPICE | Proprietary XML | Proprietary schematic | C++ API |
| **Use existing .cir files** | Yes | No (redraw) | No (redraw) | No (rewrite) |
| **Code generation** | Rust (done), C++/FAUST/Python/MATLAB (planned) | C++, MATLAB | JIT only | None |
| **Real-time safe** | Yes | Yes | No (GC) | Yes |
| **Platform** | Cross-platform | Mac/Win | Windows only | Cross-platform |

### Actionable Takeaways for Implementation Priority

These findings reinforce and extend the Phase 1-5 roadmap from Round 3:

1. **Upgrade BJT to Gummel-Poon** — the single highest-impact change. Every persona flagged 35% RMS error. Early effect + high-injection modeling would close most of the gap.
2. **Tube grid current** — without it, the guitar amp use case (Melange's core audience) is incomplete. Dave: "grid current is where tubes get their character."
3. **Raise M limit** — M=8 blocks full amp modeling. Consider iterative/sparse NR for M>8.
4. **Onboarding/DX** — Jake's "terminal fumbler" concern is real. A `curl | sh` installer or pre-built binaries + a 5-minute quickstart would dramatically lower the barrier for the pedal-builder audience.
5. **NR failure behavior docs** — Marcus needs to know: what happens when convergence fails at audio rate? Document the fallback (clamp? hold last sample? mute?) and make it configurable.
6. **Hardware validation** — Dr. Moreira's point: SPICE-vs-SPICE validation proves implementation correctness, but hardware-vs-Melange validation proves model accuracy. Even a few published comparisons would build credibility.
