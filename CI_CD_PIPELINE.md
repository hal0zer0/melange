# CI/CD Pipeline for Gold-Standard Testing

## Overview

This document defines the continuous integration pipeline that ensures
melange remains a GOLD STANDARD circuit simulation toolkit.

## Pipeline Stages

### Stage 1: Fast Checks (< 30 seconds)
**Trigger:** Every commit

```bash
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo check --workspace
```

**Purpose:** Catch style issues and compilation errors immediately.

### Stage 2: Unit Tests (< 2 minutes)
**Trigger:** Every commit

```bash
cargo test --lib --workspace
```

**Purpose:** Fast feedback on core functionality.

**Must Pass:**
- All matrix validation tests
- All codegen unit tests
- All DK math verification tests

### Stage 3: Integration Tests (< 5 minutes)
**Trigger:** Every PR

```bash
cargo test --test '*' --workspace
```

**Purpose:** Validate end-to-end workflows.

**Must Pass:**
- Generated code compiles
- Generated code runs without crash
- 10k sample stability test

### Stage 4: Safety Tests (< 2 minutes)
**Trigger:** Every PR

```bash
cargo test --package melange-solver safety
```

**Purpose:** Ensure speaker safety.

**Must Pass:**
- S matrix magnitude checks
- Output bounding verification
- No NaN/inf generation

### Stage 5: Property Tests (< 5 minutes)
**Trigger:** Every PR

```bash
cargo test --package melange-solver property
```

**Purpose:** Catch edge cases with fuzzing.

### Stage 6: Long-Running Stability (Nightly)
**Trigger:** Nightly builds

```bash
cargo test --package melange-solver --include-ignored
```

**Purpose:** Catch drift and divergence over millions of samples.

**Must Pass:**
- 1M sample sine wave test
- Zero input DC drift test
- Large signal stability test

### Stage 7: SPICE Validation (Nightly)
**Trigger:** Nightly builds

```bash
# Compare generated code output against ngspice
./scripts/validate_against_spice.sh
```

**Purpose:** Ensure accuracy against reference.

**Must Pass:**
- < 0.1% error vs SPICE for all test circuits

### Stage 8: Plugin E2E Tests (Weekly)
**Trigger:** Weekly + before releases

```bash
# Build CLAP plugins and test in headless DAW
./scripts/test_plugins.sh
```

**Purpose:** Ensure plugins work in real DAWs.

**Must Pass:**
- Plugin loads without crash
- Audio passes through
- No runaway on stop/play

## Test Failures

### Critical Failures (Block Merge)
- Any safety test failure
- Any NaN/inf in output
- S matrix magnitude > 100
- Generated code doesn't compile
- Output exceeds 100V

### High Priority (Block Release)
- DC offset > 100mV
- K matrix norm > 1e6
- Integration test failure
- SPICE validation > 0.1% error

### Normal Priority (Fix Soon)
- Clippy warnings
- Documentation missing
- Performance regression > 10%

## Pre-Release Checklist

Before any release (even alpha):

- [ ] All CI stages pass
- [ ] Manual test in Reaper/Ardour
- [ ] Test with silence input
- [ ] Test with sine wave
- [ ] Test with guitar DI recording
- [ ] Test stop/play 10 times
- [ ] Verify no memory leaks (valgrind)
- [ ] Check binary size < 10MB

## Speaker Safety Protocol

**NEVER** ship code that:
1. Produces NaN/inf
2. Exceeds 100V output
3. Has > 100mV DC offset
4. Lacks output clamping
5. Has S[0][0] > 1000

**ALWAYS**:
1. Test with full-scale sine
2. Test with silence
3. Test rapid start/stop
4. Verify DC blocker works

## Regression Testing

Every bug fix MUST include:
1. Test that reproduces the bug
2. Test that verifies the fix
3. Documentation of what was learned

Example: The v2 plugin explosion bugs now have tests:
- `test_s_matrix_sensible_magnitude`
- `test_generated_jacobian_sign`
- `test_generated_code_has_step_clamping`

## Performance Benchmarks

Track on every PR:
- Code generation time (< 1s for simple circuits)
- Compilation time of generated code (< 5s)
- Runtime performance (target: < 1% CPU for 1 circuit)

## Documentation Requirements

Every test must have:
- Clear name explaining what it tests
- Comment explaining WHY it matters
- Reference to bug/issue if regression test

## Emergency Procedures

If a bug reaches users:
1. Add regression test immediately
2. Fix the bug
3. Yank/release new version
4. Post-mortem: Why didn't tests catch it?
5. Update test suite to catch similar bugs

## GitHub Actions Workflow

```yaml
name: Gold Standard CI

on: [push, pull_request]

jobs:
  fast-checks:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - run: cargo fmt --check
      - run: cargo clippy --all-targets -- -D warnings
      - run: cargo check --workspace

  unit-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - run: cargo test --lib --workspace

  safety-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - run: cargo test --package melange-solver safety

  integration-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - run: cargo test --test '*' --workspace

  nightly-stability:
    runs-on: ubuntu-latest
    if: github.event.schedule == '0 0 * * *'
    steps:
      - uses: actions/checkout@v3
      - run: cargo test --package melange-solver --include-ignored
```

## Success Metrics

- Test coverage: > 90%
- CI pass rate: > 99%
- Time to catch bug: < 5 minutes (CI)
- Bugs reaching users: 0 (critical), < 5/year (all)

## Philosophy

**"If it's not tested, it's broken."**

**"Speaker safety is non-negotiable."**

**"Every bug is a missing test."**
