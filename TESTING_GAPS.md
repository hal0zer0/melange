# Testing Gaps That Caused v2 Plugin Failures

This document tracks the testing failures that allowed critical bugs into the Mordor Screamer v2 plugin.

## Bugs That Should Have Been Caught

### 1. S Matrix Explosion (CRITICAL)
**Bug:** S[0][0] = 11,643 instead of ~1.0
**Cause:** Input conductance (1.0) not added to G matrix before kernel creation
**Impact:** 10,000x signal amplification
**Test Added:** `test_s_matrix_sensible_magnitude`

### 2. Wrong Jacobian Sign (CRITICAL)
**Bug:** Generated code used `J = I + K*G` instead of `J = I - K*G`
**Cause:** Copy-paste error from different solver formulation
**Impact:** Solver divergence on every iteration
**Test Added:** `test_generated_jacobian_sign`

### 3. No Step Clamping (CRITICAL)
**Bug:** Newton-Raphson steps were unbounded
**Cause:** Missing `clamp()` on NR update step
**Impact:** Exponential growth of nonlinear currents (4700A through diodes!)
**Test Added:** `test_generated_code_has_step_clamping`

### 4. No NaN/Inf Detection (HIGH)
**Bug:** Solver didn't check for non-finite values
**Cause:** Missing `.is_finite()` check in generated code
**Impact:** NaN propagation through audio
**Test Added:** Checked in `test_generated_code_has_step_clamping`

### 5. Empty update_history() (HIGH)
**Bug:** Capacitor companion model history not tracked
**Cause:** TODO stub not implemented
**Impact:** DC offset accumulation (-0.8V over time)
**Status:** Test added (`test_no_dc_drift_with_zero_input`) but marked `#[ignore]` until fixed

### 6. No Singular Jacobian Check (MEDIUM)
**Bug:** Division by zero when det(J) ≈ 0
**Cause:** Missing check before `1.0 / det`
**Impact:** Explosion on singular matrices (antiparallel diodes)
**Test Added:** Checked in `test_generated_code_has_step_clamping`

## Test Coverage Checklist

Before any circuit ships to users, these tests MUST pass:

### Unit Tests (Codegen)
- [x] S matrix values in reasonable range (0.1 - 10 for input node)
- [x] K matrix not pathologically large
- [x] Generated code has step clamping
- [x] Generated Jacobian uses correct sign
- [x] Generated code has NaN/inf checks
- [x] Generated code handles singular Jacobian

### Integration Tests (Generated Code)
- [ ] Zero input → zero output (no DC drift)
- [ ] Small sine wave → bounded output (< 10V)
- [ ] Large input → clipping (not explosion)
- [ ] 1 second runtime → no divergence
- [ ] Reset → returns to clean state

### End-to-End Tests (Plugin)
- [ ] Plugin loads in DAW without crash
- [ ] Audio passes through without explosion
- [ ] Silence input → silence output
- [ ] Stop playback → no runaway oscillation

## What We Learned

1. **Matrix magnitude tests are essential** - Caught the S matrix bug immediately
2. **Generated code inspection is valuable** - Caught Jacobian sign and clamping bugs
3. **DC drift is insidious** - Requires long-running integration tests
4. **Antiparallel diodes are special** - Need specific tests for this topology
5. **Speaker safety is critical** - Need output clamping as final defense

## Recommended Testing Pipeline

```
SPICE Netlist
    ↓
Parse → Unit test (valid netlist)
    ↓
Build MNA → Unit test (S matrix magnitude, K matrix stability)
    ↓
Generate Code → Unit test (has clamping, has NaN checks, Jacobian sign)
    ↓
Compile Code → Integration test (compiles without warnings)
    ↓
Simulate → Integration test (DC drift, sine wave stability, 1s runtime)
    ↓
Build Plugin → E2E test (loads in DAW, passes audio)
    ↓
SHIP
```

Every step must pass before the next.
