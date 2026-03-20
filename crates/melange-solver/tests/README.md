# Melange Solver Test Suite

## Test Files

| File | Tests | Description |
|------|-------|-------------|
| `codegen_verification_tests.rs` | 128 | Codegen template output: constants, matrices, device calls, NR structure |
| `coupled_inductor_tests.rs` | 50 | Coupled inductors and transformers |
| `augmented_mna_tests.rs` | 10 | Augmented MNA for inductor branch variables |
| `nodal_codegen_tests.rs` | 11 | Nodal solver codegen (compile checks) |
| `inductor_tests.rs` | 35 | Inductor companion models |
| `nonlinear_dc_op_tests.rs` | 13 | DC operating point solver |
| `pot_tests.rs` | 22 | Dynamic potentiometers (Sherman-Morrison) |
| `switch_tests.rs` | 20 | Switch directives |
| `subcircuit_tests.rs` | 22 | Subcircuit expansion |
| `safety_tests.rs` | 8 | Output bounds, NaN prevention |
| `forward_active_bjt_tests.rs` | 6 | FA BJT detection and codegen |
| `matrix_math_tests.rs` | 9 | Linear algebra utilities |
| `cross_validation_tests.rs` | 6 | Runtime solver vs codegen comparison (gold standard) |

## Running Tests

```bash
cargo test -p melange-solver          # All solver tests (~930 tests)
cargo test -p melange-validate --test spice_validation -- --include-ignored  # SPICE comparison (needs ngspice)
cargo test --workspace                # Everything
```

## SPICE Validation (requires ngspice)

14 tests in `melange-validate` compare output against ngspice. Gated by `#[ignore]`.
Run with `--include-ignored` when ngspice is installed.

## Cross-Validation Tests

`cross_validation_tests.rs` compiles generated code AND runs it, comparing against the runtime
solver for the same circuit. These are the highest-value tests for numerical correctness.
Currently covers: RC circuit, diode clipper, BJT CE, JFET CS, MOSFET CS, triode CC.
