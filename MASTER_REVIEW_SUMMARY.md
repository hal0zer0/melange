# Melange Multi-Agent Review — Unified Findings

**Date:** 2026-02-28
**Commit:** c981d95 (post round-2 fixes)
**Reviewed by:** 5 specialized agents (Device Models, MNA/DK, NR Solver, Codegen, Parser/CLI)
**Cross-validated against:** `docs/aidocs/NR_SOLVER.md`, `DK_METHOD.md`, `CODEGEN.md`, `MNA.md`

---

## Summary

| Severity | Claimed | After Triage |
|----------|---------|-------------|
| HIGH     | 5       | **3 confirmed** (2 false positives retracted) |
| MEDIUM   | 12      | 12 |
| LOW      | ~12     | ~12 |

### False Positives (Retracted)

**NR-1: "Jacobian negation in solve_md()"** — RETRACTED.
The NR agent claimed `solve_md()` solves `J*delta=residual` instead of `J*delta=-residual`.
This is wrong. `solve_md()` does `delta = J^{-1}*residual` then `v -= delta`, giving
`v_new = v - J^{-1}*residual`. The primitives `nr_solve_dk` does `dv = J^{-1}*(-residual)`
then `v += dv`, also giving `v_new = v - J^{-1}*residual`. Both are correct and equivalent.
Confirmed against `docs/aidocs/NR_SOLVER.md` line 56: `delta = J^{-1} * f`, `i_new = i - delta`.

**NR-2: "Device 2D index bounds off-by-one"** — RETRACTED by agent itself during analysis.

---

## HIGH Severity (Confirmed: 3)

### H1: LDR division by zero
**File:** `crates/melange-devices/src/ldr.rs:117,122`
**Impact:** `current()` and `jacobian()` divide by `r_state` with no guard. While `reset_to()` clamps to `r_min`, external code could call methods when `r_state` is at a numerically-zero `r_min`.
**Fix:**
```rust
fn current(&self, v: &[f64; 1]) -> f64 {
    v[0] / self.r_state.max(1e-12)
}
fn jacobian(&self, _v: &[f64; 1]) -> [f64; 1] {
    [1.0 / self.r_state.max(1e-12)]
}
```

### H2: JFET vp=0 division by zero
**File:** `crates/melange-devices/src/jfet.rs:32` (constructor), `:98,103,108,131,136,139` (usage)
**Impact:** `new()` accepts `vp=0.0` without validation. `drain_current()` and `jacobian_partial()` both compute `vp_abs * vp_abs` in denominators.
**Fix:** Add validation in constructor:
```rust
pub fn new(channel: JfetChannel, vp: f64, idss: f64) -> Self {
    assert!(vp.abs() > 1e-15, "JFET Vp must be non-zero");
    assert!(idss > 0.0, "JFET IDSS must be positive");
    ...
}
```

### H3: Codegen missing model parameter validation
**File:** `crates/melange-solver/src/codegen/ir.rs:267-294`
**Impact:** `IS=0`, `N=0`, `BF=0`, `BR=0` from `.model` lines would cause division by zero in generated code. Parser validates component values (R, L, C > 0) but NOT model params.
**Fix:** Add validation in `build_device_info()`:
```rust
if is <= 0.0 || !is.is_finite() {
    return Err(CodegenError::InvalidConfig(
        format!("device model IS must be positive finite, got {}", is)
    ));
}
// Similar for N, VT, BF, BR
```

---

## MEDIUM Severity (12)

### M1: Codegen input_resistance not validated
**File:** `crates/melange-solver/src/codegen/rust_emitter.rs` or `mod.rs`
**Impact:** `input_resistance <= 0` would produce Inf/NaN in generated `1.0 / INPUT_RESISTANCE`.
**Fix:** Validate `input_resistance > 0.0` in `CodeGenerator::generate()`.

### M2: NR solve_md() NaN recovery too late
**File:** `crates/melange-solver/src/solver.rs:748-752`
**Impact:** NaN detection restores v_nl_prev but doesn't reset inductor/capacitor state. Could leave solver in half-valid state.
**Fix:** Propagate NaN flag to `process_sample()` for full reset, or reset all nonlinear state locally.

### M3: Singular Jacobian fallback inconsistency
**File:** `crates/melange-solver/src/solver.rs:732-739`
**Impact:** Runtime uses `v += -0.5*residual` (non-standard). Codegen returns best guess. Different behavior.
**Fix:** Unify — both should return best guess (matching codegen), or both use proper damped step.

### M4: solve_md() missing step-size convergence check
**File:** `crates/melange-solver/src/solver.rs:686-689`
**Impact:** Only checks residual convergence, not step size. 1D/2D solvers check both. Could cause premature convergence near saddle points.
**Fix:** Add `if step_sq < tol*tol { break; }` after computing delta.

### M5: MOSFET constructor has no validation
**File:** `crates/melange-devices/src/mosfet.rs:29`
**Impact:** `kp=0`, wrong sign on `vt`, `lambda<0` all silently accepted.
**Fix:** Add validation or at-minimum document expected ranges/signs.

### M6: BJT Jacobian quotient-rule fallback
**File:** `crates/melange-devices/src/bjt.rs:320-329`
**Impact:** When `qb^2 < 1e-30`, falls back to `dicc_dvbe` dropping the `1/qb` scaling. Mathematically incorrect (should be `dicc_dvbe / qb`).
**Fix:** Use `(dicc_dvbe * qb - icc * dqb_dvbe) / qb2.max(1e-30)` with clamped denominator.

### M7: Parser AC phase silent default
**File:** `crates/melange-solver/src/parser.rs:508`
**Impact:** Invalid AC phase values default to 0.0 via `.unwrap_or()` instead of erroring.

### M8: Unknown directives silently ignored
**File:** `crates/melange-solver/src/parser.rs:333-335`
**Impact:** Typos in directive names (`.optn`, `.titl`) silently dropped.

### M9: Ground node accepted as I/O
**File:** `tools/melange-cli/src/main.rs:301-309`
**Impact:** `--input-node 0` resolves to ground via `saturating_sub(1)` → index 0. Nonsensical.
**Fix:** Check `if input_node_idx == 0 { return Err("cannot be ground") }` before subtracting.

### M10: CLI unwrap panic in plugin path
**File:** `tools/melange-cli/src/main.rs:404`
**Impact:** `.file_name().unwrap().to_str().unwrap()` panics on edge-case paths.

### M11: CLI no validation of sample_rate/tolerance
**File:** `tools/melange-cli/src/main.rs:47,63`
**Impact:** `--sample-rate 0` or negative values cause division by zero downstream.

### M12: MNA inductor A_neg comment misleading
**File:** `crates/melange-solver/src/mna.rs:320`
**Impact:** Comment says "same form as resistor" which is ambiguous. No code bug.

---

## LOW Severity (Notable)

| # | File | Issue |
|---|------|-------|
| L1 | `mosfet.rs:616-632` | Body terminal ignored in MNA stamping |
| L2 | `parser.rs:651-659` | MEG vs M suffix order-dependent |
| L3 | `parser.rs:356-367` | No validation of model parameter *names* |
| L4 | `codegen/rust_emitter.rs:376` | Zero S matrix entries included in mul |
| L5 | `codegen templates` | No BJT polarity support in codegen |
| L6 | `cli/main.rs:422-441` | validate/simulate commands are stubs |
| L7 | `cli/main.rs:58-59` | max_iter=0 not rejected |
| L8 | `solver.rs:651` | Warm-start from v_nl_prev vs i_nl_prev in RHS (design choice) |
| L9 | `mna.rs:509-530` | VS Norton equiv doesn't model source resistance |
| L10 | `parser.rs:445-448` | Capacitor C=0 not validated (edge case) |
| L11 | `parser.rs:308,330` | .END vs .ENDS case sensitivity quirk |

---

## Architectural Note: Voltage-Domain vs Current-Domain NR

The runtime `solve_md()` and the codegen/primitives use different (but equivalent) NR formulations:

| | Runtime `solve_md()` | Codegen / `nr_solve_dk` |
|---|---|---|
| Domain | Voltage | Current |
| Residual | `r = v - p - K*i(v)` | `f = i - i_dev(p + K*i)` |
| Jacobian | `J = I - K*G_dev` | `J = I - G_dev*K` |
| Update | `v -= J^{-1}*r` | `i -= J^{-1}*f` |

Both converge to the same fixed point. The NR_SOLVER.md documents the current-domain form. The comment in `solve_md()` at line 642 documents the voltage-domain form. This is correct but worth noting that K*G and G*K don't commute in general — they are genuinely different Jacobian matrices that happen to produce the same Newton fixed point.

---

## Recommended Fix Priority

1. **H1, H2, H3** — Division-by-zero guards (quick, high-impact)
2. **M1** — Input resistance validation (one-liner)
3. **M9, M10, M11** — CLI validation (prevents user-facing crashes)
4. **M3, M4** — NR solver consistency (convergence quality)
5. **M6** — BJT quotient-rule fallback (correctness)
6. **M2** — NaN recovery improvement (robustness)
7. Everything else — polish

---

## Verified Correct

The following were explicitly verified as correct by cross-referencing code against `docs/aidocs/`:

- K matrix sign convention (naturally negative, NO extra negation)
- NR Jacobian formula: `J = I - G_dev*K` (codegen) / `J = I - K*G_dev` (runtime)
- NR update: `delta = J^{-1}*f`, `i -= delta` — both formulations equivalent
- N_I matrix transposition in codegen (M×N, correctly transposed)
- Final voltage correction: `v = v_pred + S * N_i^T * (i_nl - i_nl_prev)`
- Block-diagonal Jacobian assembly for heterogeneous devices
- Per-device model parameters (IS, N_VT, VT, BF, BR) in codegen
- M>4 early rejection in IR build
- NaN/Inf safety in process_sample with full state reset
- DC operating point with input conductance
- Inductor companion model codegen
- Trapezoidal RHS: `(input + input_prev) * G_in`
