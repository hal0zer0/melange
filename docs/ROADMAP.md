# Melange Roadmap

**Last Updated:** 2026-03-12
**Current:** 731 tests passing, 0 warnings, 0 unsafe blocks

---

## Priority 1 — High Impact, Moderate Effort

### 1.1 AC Frequency Response Analysis
- **Status:** Not started
- **Scope:** Add `melange analyze circuit.cir --freq-response 20 20000` CLI command
- Sweep sine at each frequency through solver, measure gain/phase
- Output CSV (frequency, gain_dB, phase_deg)
- Infrastructure exists in melange-validate
- **Files:** `tools/melange-cli/src/main.rs`

### ~~1.2 2D JFET Model (Triode + Saturation Regions)~~ ✅ Done (2026-03-12)
- Full Shichman-Hodges 2D: triode + saturation + channel-length modulation (LAMBDA)
- JFET is 2D in NR system (Id + Ig), matching BJT/tube architecture

### ~~1.3 2D MOSFET Model (Triode + Saturation Regions)~~ ✅ Done (2026-03-12)
- Full Level 1 SPICE 2D: triode + saturation + channel-length modulation (LAMBDA)
- MOSFET is 2D in NR system (Id + Ig=0), N/P channel support
- Codegen fully supported (was previously rejected with error)

### ~~1.4 Subcircuit Flattening~~ ✅ Done (2026-03-02)
### ~~1.5 Transformer / Coupled Inductor Support~~ ✅ Done (2026-03-02)
### ~~1.6 Multi-Output (Stereo) Support~~ ✅ Done (2026-03-12)

---

## Priority 2 — Important, Larger Effort

### 2.1 Runtime Device Parameter Control
- Promote device params (IS, BF, tube mu) to `CircuitState` fields like pots
- Enable tube aging simulation, transistor matching, expressive control
- **Files:** Codegen IR, device templates, plugin template

### 2.2 Controlled Sources (E, G)
- VCVS (E) and VCCS (G) elements not implemented
- Needed for current mirrors, active filters, dependent sources
- **Files:** `parser.rs`, `mna.rs`

### 2.3 Line Continuation Fix
- SPICE `+` continuation lines not properly joined (KNOWN BUG)
- **Files:** `parser.rs`

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

## OpenWurli Integration (Current Focus)

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
| Line continuation (`+`) not joined | Medium | Use single-line `.model` statements |
| Purely resistive nonlinear circuits oscillate | Low | Always include at least one capacitor |
| Several integration tests `#[ignore]`d | Low | Tests exist but need tuning |

---

*The spice must flow.*
