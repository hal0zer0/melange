# AGENTS.md

Guidance for AI agents working on Melange - a Rust toolkit for circuit simulation to audio DSP.

## Quick Reference

| Need | See |
|------|-----|
| MNA matrix stamping | `docs/aidocs/MNA.md` |
| DK method/S,K matrices | `docs/aidocs/DK_METHOD.md` |
| Companion models | `docs/aidocs/COMPANION_MODELS.md` |
| Newton-Raphson | `docs/aidocs/NR_SOLVER.md` |
| Device models | `docs/aidocs/DEVICE_MODELS.md` |
| Code generation | `docs/aidocs/CODEGEN.md` |
| Debugging | `docs/aidocs/DEBUGGING.md` |

**Critical:** `K = -N_v·S·N_i` (negative sign required). Always verify sign conventions in docs/aidocs/ before changing Jacobian formulas.

## Agent Guidelines

### Before Changing Code
1. **Read relevant aidocs** - Understand the math first
2. **Trace actual implementation** - Don't assume; verify
3. **Derive correct behavior** from first principles
4. **Show your work** when reasoning through equations

### What NOT To Do
- Guess Jacobian signs without derivation
- Make "quick fixes" without root cause analysis
- Change code based on intuition alone
- Ignore docs/aidocs/ when working on core math

### When Unsure
Say "I need to verify this" and reference the specific aidoc. Acknowledge uncertainty rather than flip-flop.

## Core Principles

- **Real-time safety:** No alloc/locks/syscalls in audio thread
- **SPICE is ground truth:** Validate models against ngspice
- **Const generics:** Compile-time matrix sizes `[f64; N]`, not `Vec`
- **Test at every layer:** Unit → SPICE compare → integration
- **Understand before fixing:** Read docs, trace math, verify assumptions

## Build Commands

```bash
cargo build --workspace
cargo test --workspace
cargo run -p melange-cli
```

## Project Structure

```
crates/
  melange-primitives/   # Layer 1: DSP building blocks
  melange-devices/      # Layer 2: Component models
  melange-solver/       # Layer 3: MNA/DK + codegen (CORE)
  melange-validate/     # Layer 4: SPICE validation
  melange-plugin/       # Layer 5: nih-plug integration
tools/
  melange-cli/          # CLI tool
docs/aidocs/            # AI-optimized reference docs
```

## Stack

| Layer | Purpose |
|-------|---------|
| 5 | nih-plug integration |
| 4 | SPICE-to-Rust validation |
| 3 | MNA/DK solver + codegen |
| 2 | Component models (BJT, diode, etc.) |
| 1 | DSP primitives (filters, NR helpers) |

## Key Concepts

| Concept | Description |
|---------|-------------|
| **MNA** | Modified Nodal Analysis - stamp components into G/C matrices |
| **DK Method** | Reduce N-node system to M×M nonlinear kernel |
| **Companion Models** | Trapezoidal discretization of C/L |

## Origin

Extracted from [OpenWurli](https://github.com/FIXME/openwurli) - provides real-world integration test for every release.

## Equations

- `A = G + alpha·C` where `alpha = 2/T`
- `K = -N_v·S·N_i`
- Jacobian: Context-dependent, see NR_SOLVER.md
- RHS: `A_neg·v_prev` only (no separate cap_history)
