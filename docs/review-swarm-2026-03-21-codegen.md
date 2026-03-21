# Codegen Review Swarm — 2026-03-21

5 specialist agents reviewed the melange codegen pipeline in parallel. Findings synthesized below.

## Critical — Bugs or risks that affect correctness

### C1. Multi-output plugin template won't compile (Plugin Dev)
- `plugin_template.rs:366-374` — calls `channel_samples.into_iter()` twice; second call consumes a moved value
- `plugin_template.rs:447-454` — multi-output + params path indexes `process_sample(...)[0]`, dropping extra channels to mono
- **Fix**: Rewrite multi-output process loop to iterate channels once

### C2. JFET/MOSFET subthreshold discontinuity causes NR oscillation (Analog Expert)
- `device_jfet.rs.tera:14-17,44-48` and `device_mosfet.rs.tera:15-17,44-46`
- Current/Jacobian has 8-order-of-magnitude gm jump at threshold boundary
- At `vgst = 0`, Jacobian `gm = 0` because `sub < 1e-12` condition becomes false exactly at boundary
- **Fix**: Use smooth transition function (e.g., SPICE-style tangential connection between subthreshold and square-law)

### C3. GP q2 uses NF/NR but SPICE3f5 uses ideality=1.0 (Analog Expert)
- `device_bjt.rs.tera:27-31` — `q2 = IS*exp(Vbe/(NF*VT))/IKF` vs SPICE `q2 = IS*exp(Vbe/VT)/IKF`
- Wrong high-injection rolloff for devices with NF != 1.0 (power transistors, SiGe)
- **Fix**: Remove NF/NR from q2 exponentials

### C4. 12AX7 default Kg1 over-predicts plate current 3x (Analog Expert)
- `catalog/tubes.rs:33-49` — `Kg1 = 1060` gives 3.4mA at Vgk=0/Vpk=250V; datasheet says ~1.2mA
- Wrong bias point, gain, and headroom for every circuit using `.model 12AX7`
- The corrected `12AX7F` entry exists but is not the default
- **Fix**: Change default 12AX7 to use Kg1=3000 (or make 12AX7F the default)

## High — Gaps that block real-world usage

### H1. Plugin won't build out of the box (Plugin Dev)
- `main.rs:1149-1170` — CLI prints wrong build instructions (`cargo build` instead of `cargo nih-plug-xtask bundle`)
- No xtask setup in generated `Cargo.toml`; nih-plug git dep unpinned
- **Fix**: Print correct bundle command; pin nih-plug rev; add xtask note in generated README

### H2. `set_switch_N()` does O(N³) from potentially audio thread (DSP RT)
- `rust_emitter.rs:1294` — doc says "NOT audio thread" but nothing enforces it
- DAW-automated switch params could trigger matrix inversion per-sample
- **Fix**: Deferred update pattern (flag dirty, rebuild at buffer boundary) or document prominently

### H3. Pot changes are instantaneous — audible clicks (DSP RT)
- `process_sample.rs.tera:26`, `state.rs.tera:126-129`
- No parameter smoothing on pot resistance changes
- `pot_N_resistance_prev` exists for A_neg correction but not for interpolation
- **Fix**: Add one-pole smoothing (~5ms) on pot resistance in generated code

### H4. `CircuitIR` and `Emitter` trait not re-exported (Rust API)
- `lib.rs:64-66` — custom emitter authors must use `melange_solver::codegen::ir::CircuitIR`
- Key extensibility types buried behind nested modules
- **Fix**: Add re-exports: `pub use codegen::{ir::CircuitIR, emitter::Emitter}`

### H5. No pentode mode for power tubes (Analog Expert)
- EL34/6L6/EL84 only triode-connected; push-pull power amp stages give wrong gain/impedance/distortion
- **Fix**: Add pentode Koren model variant (different plate current equation for screen grid)

### H6. CodegenConfig has no validation on `sample_rate` (Rust API)
- `mod.rs:45` — zero or negative sample_rate causes division-by-zero in `alpha = 2.0 * internal_rate`
- tolerance and max_iterations also not validated
- **Fix**: Add `CodegenConfig::validate()` method; call it at entry points

## Medium — Improvements that would significantly help users

### M1. NaN reset returns hard zero — audible click (DSP RT)
- `process_sample.rs.tera:162-183` — returns `[0.0; NUM_OUTPUTS]` on NaN, causing full-scale step
- **Fix**: Return DC operating point output (scaled) instead of zero

### M2. BE fallback transition is unsmoothed (DSP RT)
- `process_sample.rs.tera:79-127` — instantaneous switch between trapezoidal and BE solutions
- **Fix**: 1-2 sample crossfade between trapezoidal and BE outputs

### M3. Non-convergent NR returns unvalidated guess (DSP RT)
- `rust_emitter.rs:3465-3477` — when NR hits MAX_ITER, current i_nl may be wildly wrong
- **Fix**: Damp toward previous good value (50% blend with i_nl_prev)

### M4. Diode breakdown discontinuity at v_d = -BV (Analog Expert)
- `device_diode.rs.tera:60-64` — hard step in I-V curve at breakdown knee causes NR oscillation
- **Fix**: Smooth transition region near -BV (SPICE3f5 DEVdiode.c approach)

### M5. MOSFET body effect VT stale during NR iterations (Analog Expert)
- `rust_emitter.rs:2249-2252` — VT computed once before NR, not updated per iteration
- **Fix**: Move body effect VT update inside the NR loop

### M6. No denormal protection outside plugin wrapper (DSP RT)
- DC blocking filter, allpass state, NR predictor all susceptible
- Plugin template sets FTZ+DAZ but standalone `melange simulate` does not
- **Fix**: Add tiny DC bias (1e-25) to DC block feedback, or set FTZ+DAZ in `melange simulate`

### M7. No `#[non_exhaustive]` on public enums/structs (Rust API)
- `CodegenError`, `DeviceType`, `CodegenConfig`, `CircuitIR` — adding variants/fields is semver-breaking
- **Fix**: Add `#[non_exhaustive]` to all public enums and key structs

### M8. Mix param displays "0%" to "1%" instead of "0%" to "100%" (Plugin Dev)
- `plugin_template.rs:543-554` — raw 0.0-1.0 value shown with `%` suffix
- **Fix**: Use percentage formatter or range 0..100

### M9. Output scale default 1.0 almost always wrong (Plugin Dev)
- `plugin_template.rs:488` — circuit outputs volts (±10V), plugin clamps ±1.0
- Without `--output-scale 0.1`, signal clips permanently
- **Fix**: Auto-compute default scale from DC OP / first simulation, or warn loudly

### M10. `circuit_name` not sanitized for generated code (Security)
- `plugin_template.rs:108-122`, `main.rs:878-882` — directory names with quotes/backslashes produce malformed Rust
- **Fix**: Sanitize to `[a-z0-9_-]`

### M11. Pot/switch labels not escaped in generated strings (Security)
- `plugin_template.rs:254,272` — labels with `"` or `\` break generated Rust string literals
- **Fix**: Escape `"` and `\` in labels before interpolation

### M12. Inner NR loops multiply worst-case iteration count (DSP RT)
- Diode with RS: 2 inner NR × 15 iters × 4 diodes × 100 outer = 12,000 inner iterations/sample
- **Fix**: Count inner loops toward global budget, or reduce inner limit to 5-8

## Low — Polish and nice-to-haves

### L1. Diode with RS: redundant inner NR (current & conductance solve separately) (Analog Expert)
- `device_diode.rs.tera:20-55` — two independent NR loops can converge to slightly different v_j
- **Fix**: Combine into single `diode_evaluate_with_rs` returning (i_d, g_d)

### L2. GP Ib/qb division contradicts GUMMEL_POON.md docs (Analog Expert)
- Code divides Ib_fwd by qb (correct per SPICE3f5); docs say "not modified by GP"
- **Fix**: Update docs to match code

### L3. JFET gate junction leakage always zero (Analog Expert)
- `device_jfet.rs.tera:32-35` — missing gate-channel diode for forward-biased JFET clipping
- Affects JFET soft-clipper circuits (MXR Phase 90 biasing)

### L4. No tube SPICE validation in test suite (Analog Expert)
- Tweed-preamp validated elsewhere but not in `spice_validation.rs`

### L5. BJT validation comments stale — say "Ebers-Moll" but GP is implemented (Analog Expert)
- `spice_validation.rs:113-133`

### L6. `Emitter::emit()` returns String — can't represent multi-file output (Rust API)
- Blocks C++ backend (.h + .cpp)

### L7. `unreachable!()` in codegen NR can panic on corrupted IR (Security)
- `rust_emitter.rs:3134,3176,3233,3289,3340` — should return `Err(CodegenError::InvalidDevice(...))`

### L8. Switch count not enforced in parser (Security)
- Documented max 16 but no `netlist.switches.len() >= 16` check

### L9. No `tail_length()` in generated plugin (Plugin Dev)
- Offline renders truncate; DC blocking + oversampling have non-zero tail

### L10. Generated code uses `std::` not `core::` — not no_std compatible (Plugin Dev)
- `std::mem::swap`, `std::f64::consts::*` could be `core::`

### L11. VST3 ID collision from XOR hash — anagram names collide (Plugin Dev)
- `plugin_template.rs:198-207`

### L12. MAX_ITER=100 default high for real-time (DSP RT)
- Typical convergence: 3-8 iterations; 100 is excessive padding

### L13. 4x oversampling outer stage only ~60dB rejection (DSP RT)
- Inner uses 3-section (~80dB); outer uses 2-section (~60dB)

### L14. MOSFET/JFET subthreshold uses raw `.exp()` not `fast_exp` (Analog Expert)
- `device_mosfet.rs.tera:17`, `device_jfet.rs.tera:16` — inconsistent with codebase convention

### L15. Validation logic copy-pasted 4 times (Rust API)
- `mod.rs:194-236`, `277-316`, `ir.rs:617-625`, `rust_emitter.rs:536-549`
- **Fix**: Single `CodegenConfig::validate()`

### L16. Large CircuitState stack footprint for big circuits (DSP RT)
- Pultec: ~60-70KB per channel; `invert_n` augmented matrix ~27KB on stack

### L17. No CPU time measurement or prediction (DSP RT)

### L18. serde is a hard dependency, not optional (Rust API)

### L19. `CodegenError` has empty `Error::source()` (Rust API)
- Wraps `DkError`/`MnaError` but discards causal chain

### L20. Missing BJT charge storage dynamics: TF, TR, VJE, MJE, etc. (Analog Expert)

### L21. Missing voltage-dependent diode/BJT depletion capacitance (Analog Expert)
