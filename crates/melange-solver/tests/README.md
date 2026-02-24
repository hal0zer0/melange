# Melange Solver Test Suite

## Testing Philosophy

**GOLD STANDARD PRINCIPLES:**
1. **No untested code path** - Every function has test coverage
2. **Fail fast, fail loud** - Tests catch bugs at build time, not in DAWs
3. **Speaker safety first** - Output bounds are guaranteed, never assumed
4. **Real-world validation** - Generated code is compiled and executed
5. **Regression immunity** - Every bug becomes a test

## Test Organization

### Unit Tests (`src/*/tests`)
- Fast (< 1ms each)
- Isolated (no I/O, no compilation)
- Comprehensive edge cases

### Integration Tests (`tests/integration/`)
- End-to-end circuit simulation
- Generated code compilation
- Long-running stability (1M+ samples)

### Property Tests (`tests/property/`)
- Fuzzing with random inputs
- Invariant checking
- Convergence validation

### Safety Tests (`tests/safety/`)
- Output bounds verification
- DC offset detection
- NaN/inf prevention

## Running Tests

```bash
# All tests
cargo test --package melange-solver

# Fast unit tests only
cargo test --package melange-solver --lib

# Integration tests (slower)
cargo test --package melange-solver --test '*'

# Safety-critical tests
cargo test --package melange-solver safety

# With coverage
cargo tarpaulin --package melange-solver
```

## Critical Test Criteria

Every generated circuit MUST pass:
1. S matrix: diagonal values 0.01 - 100
2. K matrix: ||K|| < 1e6
3. Zero input → |output| < 0.01V after 1000 samples
4. Sine wave → |output| < 10V for 1M samples
5. No NaN/inf in 1M samples
6. Reset returns to within 1% of initial state
