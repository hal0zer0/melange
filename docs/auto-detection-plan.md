# Auto-Detection Improvement Plan

Audit date: 2026-04-11. Based on 6-persona user review panel + 5-agent codebase audit.

## Status: Tier 1 COMPLETE, Tier 2 COMPLETE (2026-04-11)

### Shipped
- **env_logger** — 19+ previously-silent decisions now visible via `RUST_LOG=info`
- **Compilation summary** — structured report of all decisions after codegen
- **CodegenMeta** — DC OP method/iterations/convergence, BE auto-selection, parasitic caps threaded from codegen to CLI
- **auto_route()** — replaced 40 lines of inline routing with library call
- **Auto-mono** — single output node → mono for plugins
- **Fuzzy node matching** — "Did you mean: mid_in?" on wrong node names
- **DC-block auto-detection** — detects output coupling caps, suggests --no-dc-block
- **Oversampling suggestion** — hints based on diode count (2���2×, 4+→4×)

## Problem

Users must specify CLI flags for decisions melange could infer from circuit topology. Many decisions that *are* already auto-detected are invisible because `log::info!` calls have no logger backend — `env_logger` isn't even in the CLI dependencies. 19+ decisions are silently discarded.

## Tier 1: Make existing decisions visible (no behavior change)

### 1a. Add `env_logger` to melange-cli
- Add `env_logger = "0.11"` to `tools/melange-cli/Cargo.toml`
- Initialize in `main()`: `env_logger::Builder::from_default_env().default_filter_or("warn").init()`
- Users can opt into verbose output with `RUST_LOG=info melange compile ...`

### 1b. Promote key silent decisions to println!
Currently log-only (invisible), should be printed during compile:
- Backward Euler auto-selection (spectral radius) — `ir.rs:1694`
- Nodal LU vs Schur selection (6 variants) — `rust_emitter.rs:4243-4269`
- Sparse LU vs dense decision — `ir.rs:2845-2862`
- Parasitic cap auto-insertion — `mod.rs:452, 607`
- DC OP convergence warnings — `ir.rs:2275`

### 1c. Add compilation summary block
Print a structured summary after "Done!" showing all auto-detected choices:

```
  Circuit: pultec-eq (41 nodes, 8 nonlinear dimensions)
  Solver: Nodal Full LU (2 transformer groups)
  Integration: Trapezoidal (spectral radius: 0.87)
  Op-amp rail mode: none (no op-amps)
  Forward-active BJTs: none
  Grid-off pentodes: none
  Oversampling: 1× (use --oversampling 2 or 4 for antialiasing)
  DC operating point: converged (direct NR, 12 iterations)
  Input: node "in", resistance 1Ω
  Output: node "out", scale 1.0
  DC block: enabled (5 Hz HPF)
  Sparse LU: enabled (28% density, 43× FLOP reduction)
```

## Tier 2: New auto-detection (safe defaults, flags become overrides)

### 2a. Auto-detect input/output nodes
- Scan for nodes matching `in`/`input`/`vin`/`audio_in` and `out`/`output`/`vout`/`audio_out`
- Fuzzy-match and suggest on failure ("Did you mean 'input'?")
- KiCad `Melange_AudioInput`/`Melange_AudioOutput` markers already set these

### 2b. Auto-mono
- Single output node → default to mono unless `--stereo` explicitly set
- Currently defaults to stereo always

### 2c. Auto-DC-block skip
- Detect output coupling capacitors between last active stage and output node
- If found, suggest `--no-dc-block` (or auto-skip with note)

### 2d. Oversampling suggestion
- Diode clippers / hard nonlinearities → suggest `--oversampling 2`
- Multiple cascaded clipping stages → suggest `--oversampling 4`
- Suggestion only (printed), not auto-applied — oversampling has CPU cost

## Tier 3: Refinements (lower priority)

### 3a. K/S matrix conditioning in routing
- Currently warnings only for S condition > 1e12
- Could auto-route to nodal if K or S are ill-conditioned

### 3b. Max-iter scaling
- Scale `max_iter` with M and spectral radius (stiffer circuits need more iterations)

### 3c. Input impedance suggestion
- Analyze first component connected to input node
- Guitar amp grid → suggest `.input_impedance 1M`
- Line-level buffer → suggest `.input_impedance 10k`

## Key files

| File | What to change |
|------|---------------|
| `tools/melange-cli/Cargo.toml` | Add `env_logger` dep |
| `tools/melange-cli/src/main.rs` | Logger init, summary block, promoted messages |
| `crates/melange-solver/src/codegen/ir.rs` | Return decision metadata from IR build |
| `crates/melange-solver/src/codegen/routing.rs` | Already returns `RoutingDecision` struct |
| `crates/melange-solver/src/codegen/mod.rs` | Surface parasitic cap / Boyle decisions |
| `crates/melange-solver/src/codegen/rust_emitter.rs` | Surface LU vs Schur decisions |
