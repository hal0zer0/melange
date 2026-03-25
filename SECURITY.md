# Security Policy

## Reporting Vulnerabilities

If you discover a security issue in melange, please report it responsibly:

1. **Do not** open a public GitHub issue
2. Contact the maintainers via GitHub (open a private security advisory on the repository)
3. Include a description, steps to reproduce, and potential impact

We will acknowledge receipt within 7 days.

## Scope

### In Scope

- Memory safety violations or `unsafe` code
- Input validation bypasses that cause panics or infinite loops
- Resource exhaustion from crafted netlists (unbounded allocation, CPU)
- Real-time safety violations in generated code (heap allocation in audio callback)

### Out of Scope

- Circuit simulation accuracy or convergence issues (numerical, not security)
- Audio clipping or distortion from extreme input signals
- Third-party dependency vulnerabilities (report to those projects)

## Current Guarantees

- **Zero `unsafe` code** across the entire codebase (library and generated code)
- **Input validation** — parser rejects negative, zero, NaN, and infinite component values; rejects self-connected components
- **Resource limits** — MAX_M=16 (DK path), MAX_N=256 nodes, MAX_ELEMENTS=10,000 after expansion, 8-level subcircuit nesting
- **Bounded iteration** — Newton-Raphson capped at `max_iter` (default 50); DC operating point has source stepping and Gmin stepping fallbacks with finite iteration counts
- **Real-time safe** — generated audio callbacks have zero heap allocation; all buffers pre-allocated at construction
- **Temp file cleanup** — SPICE validation temp files are removed after each run
