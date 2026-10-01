# Melange Architecture

This document describes *what* the system is. For *why* it is shaped this way —
the load-bearing tradeoffs and the alternatives they beat — see the companion
[**Design Decisions**](DESIGN_DECISIONS.md) log.

## The Five Translation Boundaries

Every circuit modeling project crosses these boundaries. Melange automates boundaries 2-4.

```
Schematic → Netlist → MNA Matrices → DK Kernel → Optimized Rust → Plugin
   [1]        [2]        [3]            [4]          [5]
   Human    melange    melange        melange      nih-plug
```

### Boundary 1: Schematic → Netlist (Human)
Reading a schematic and producing a correct netlist requires domain expertise that cannot be fully automated. Schematic ambiguities (wrong component values, revision mismatches, topology misidentification) have been responsible for the worst bugs in real projects. Melange checks what it can mechanically (it refuses a node that touches only one terminal of a two-terminal element and reports DC islands; `melange nodes`, `dc-op` and `validate` help you inspect the result), but a human must verify the topology.

### Boundary 2: Netlist → MNA Matrices (melange-solver)
Purely mechanical. Each component stamps its contribution into G (conductance) and C (capacitance) matrices following deterministic rules. A resistor R between nodes i and j stamps +1/R on diagonals (i,i) and (j,j), and -1/R on off-diagonals (i,j) and (j,i). Capacitors stamp into C identically. Voltage sources add rows/columns. This is the easiest part to automate and the most tedious to do by hand.

### Boundary 3: MNA → DK Kernel (melange-solver)
Linear algebra. Given G, C, and the nonlinear device connection matrices (N_v, N_i), compute:
1. Discretize: `A = 2C/T + G` (trapezoidal) or appropriate companion models
2. Invert: `S = A^{-1}`
3. Extract kernel: `K = N_v * S * N_i` (the reduced nonlinear system)

The K matrix is typically 2x2 or 4x4 even for complex circuits. All per-sample computation happens in this reduced space.

### Boundary 4: DK Kernel → Optimized Rust (melange-solver codegen)
The solver generates Rust code with:
- Const-generic matrix sizes (`[f64; N]`, not `Vec<f64>`)
- Inlined NR iteration with the specific Jacobian structure
- Precomputed constant matrices as `const` arrays
- Per-block matrix rebuild for time-varying elements (pots, switches)
- Charge-form history update (`q_dot`, committed with `v_prev`)

The generated code should be indistinguishable in performance from hand-written code.

### Boundary 5: Rust → Plugin (nih-plug)
Handled by nih-plug for the plugin shell. `melange compile --format plugin` writes a complete nih-plug project around the generated solver: parameters for `.pot`/`.wiper`/`.switch` controls, optional oversampling, and input/output level parameters (see [PLUGIN_GUIDE.md](PLUGIN_GUIDE.md)).

## Crate Architecture

### melange-primitives (Layer 1)
Zero external dependencies. The foundation everything else builds on.

**Filters:**
- `OnePoleHpf` / `OnePoleLpf` — simple 6 dB/oct filters
- `TptLpf` — Zavalishin topology-preserving transform (ZDF integrator)
- `DcBlocker` — configurable cutoff (default 20 Hz)
- `Biquad` — DFII-transposed, bandpass/lowpass/highpass/peaking/shelf

**Oversampling:**
- `Oversampler2x` / `Oversampler4x` — polyphase IIR half-band
- Configurable rejection (3/4/5 allpass sections per branch)
- `process_block()` for batch operation

**NR Helpers:**
- `nr_solve_1d<F, DF>(f, df, x0, max_iter, tol)` — scalar NR with clamping
- `nr_solve_2d` — 2x2 NR with Cramer's rule (covers most audio circuits)
- NaN recovery: detect divergence, reset to last known good state

**Utilities:**
- `variation_hash(seed, index)` — deterministic per-instance detuning
- `midi_to_freq(note)` — standard tuning conversion

### melange-devices (Layer 2)
Depends on melange-primitives. Provides the `NonlinearDevice` trait and implementations.

```rust
/// A nonlinear circuit element. N = number of controlling voltages.
pub trait NonlinearDevice<const N: usize> {
    /// Current as a function of terminal voltages.
    fn current(&self, v: &[f64; N]) -> f64;
    /// Partial derivatives of current: di/dv_k (for NR Jacobian).
    fn jacobian(&self, v: &[f64; N]) -> [f64; N];
}
```

**Implementations:**
- `DiodeShockley { is, n, vt }` — junction diode (N=1)
- `DiodeWithRs` — diode with series resistance (inner NR)
- `BjtEbersMoll { is, vt, beta_f, beta_r, … }` — NPN/PNP BJT (N=2)
- `BjtGummelPoon { … }` — extended BJT model (Early effect, high injection, N=2)
- `Jfet { idss, vp, lambda, … }` — N/P-channel JFET (N=2)
- `Mosfet { kp, vt, lambda, … }` — Level 1 MOSFET (N=2)
- `KorenTriode { mu, ex, kp, kvb, … }` — vacuum triode (N=2)
- `KorenPentode { … }` — vacuum pentode / beam tetrode (N=3)
- `Vca { vscale, thd, … }` — VCA (THAT 2180 style, N=2)
- `CdsLdr { r_min, r_max, gamma, attack_tau, release_tau }` — photoresistor (N=1; placed in a netlist via the `O` element on the stateful-device codegen path)
- `SimpleOpamp` (clamped linear gain) / `IdealOpamp` — library op-amp models. The solver does not use them: codegen stamps each op-amp itself as a VCCS into G (no NR dimension; rail handling per `--opamp-rail-mode`). `BoyleOpamp` is a doc-hidden placeholder parameter struct with no device implementation, used by nothing.

**SPICE Model Card Import:**
- Parse `.model` statements from SPICE netlists
- Extract parameters into device structs
- Example: `.model 2N5089 NPN(IS=2.64e-15 BF=735 NF=1.0 ...)` → `BjtEbersMoll { ... }`

### melange-solver (Layer 3)
The core. Depends on primitives and devices.

**Netlist Parser:**
- Parse a subset of SPICE sufficient for audio circuits
- Components: R, C, L (including ISAT= saturation), V (DC), I, D (diode), Q (BJT), J (JFET), M (MOSFET), T (triode), P (pentode), U (op-amp), Y (VCA), O (opto/LDR), X (subcircuit), E (VCVS), G (VCCS), B (behavioral source). Coupled inductors / transformers are declared with the `K` coupling directive.
- Directives: `.model`, `.subckt`, `.pot`, `.wiper`, `.switch`, `.gang`, `.linearize`, `.runtime`, `.input_impedance`
- Output: `Netlist` struct with elements, models, and directives

**MNA Assembler:**
- `MnaSystem::from_netlist(&netlist)` → `MnaSystem { g, c, n_v, n_i, ... }`
- Automatic node numbering (ground = 0)
- Automatic stamp generation for all component types
- Augmented MNA for inductors (branch current variables)

**DK Kernel / Nodal Solver:**
- `DkKernel::from_mna(&mna, sample_rate)` (decks without inductors) or `DkKernel::from_mna_augmented` (inductors as branch rows) → DK kernel with K matrix
- Three codegen paths: DK Schur, Nodal Schur, Nodal Full LU (auto-selected)
- Per-block matrix rebuild for `.pot`/`.wiper` elements on value change

**Code Generator (codegen-only pipeline):**
- `CircuitIR::from_kernel(...)` → intermediate representation
- `RustEmitter::emit(&ir)` → optimized Rust source code
- Const-generic matrix sizes, inlined NR, precomputed matrices as `const` arrays
- Tera templates for device-specific codegen

**Source layout** (`crates/melange-solver/src/`): `parser/` (netlist types,
elements, directive and element parsers, subcircuit expansion, validation);
`mna/` (the info types, stamping, augmented matrices, BJT internal nodes; the
builder and its `from_netlist*` entry points in `mna/builder/`); `dk.rs`;
`dc_op.rs`; `build.rs` and `pipeline.rs` (the one build every verb runs); and
`codegen/`, with `ir/` split by build route and device class (`build_dk.rs`,
`build_nodal.rs`, `reductions.rs`, `device_info.rs`, and the parameter
resolvers `semiconductor_params.rs`, `tube_params.rs`, `aux_params.rs`,
`model_card.rs`) and `rust_emitter/` (`dk_emitter.rs`, and `nodal_emitter/`
split by concern: `schur.rs`, `full_lu.rs` / `full_lu_newton.rs`, `substep.rs`,
`rail.rs`, `device_eval.rs`, `state.rs` and helpers).

### melange-validate (Layer 4)
Depends on solver. Requires ngspice installed on the system.

Transient comparison of a melange build against ngspice on the same deck.
`melange validate` is its command-line front end; `docs/aidocs/SPICE_VALIDATION.md`
is the protocol.

- `validate_circuit` / `validate_circuit_with_options` — build the deck through
  `melange_solver::build::build` (the build `compile` ships), run the generated
  solver, run ngspice, compare
- `spice_runner` — run ngspice transients (PWL and Thevenin drives) and parse
  the output; `is_ngspice_available()`
- Reference twins for devices ngspice lacks or models differently: triode and
  pentode B-source subcircuits, the linear op-amp VCCS, JFET, behavioral,
  saturating-inductor, thermal-key and `.linearize` translations
- `reference` — checks the ngspice reference is itself converged
- `alignment` — best-fit constant-delay alignment before every comparison
- `comparison::compare_signals` → `ComparisonReport` (RMS, peak, max-relative
  and normalized error, correlation, SNR, THD), with strict and relaxed
  `ComparisonConfig` profiles
- `rate_sweep` — validate at 1×, 2× and 4× the rate to separate
  discretization error from model error (`melange validate --rate-sweep`)
- `deck_guard` — refuses runs the two engines cannot honestly compare
- `visualizer` — HTML, CSV and JSON reports

### melange-cli
The command-line interface for working with circuits.

**Subcommands:**
- `melange compile <netlist>` — parse netlist, generate optimized Rust code or plugin project
- `melange simulate <netlist>` — compile and run circuit, output WAV
- `melange analyze <netlist>` — frequency response, measured by driving the compiled circuit with a sine per frequency (the circuit's response to its own stimulus; aliasing, loudness, IMD, decay and noise floor are for a bench instrument)
- `melange validate <netlist>` — compare against ngspice, report deltas
- `melange nodes <netlist>` — show circuit nodes, nonlinear devices, op-amps and controls
- `melange dc-op <netlist>` — the DC operating point the build ships
- `melange sources list|add|remove|show` — manage circuit source repositories
- `melange index <dir>` — write or check a `circuits-index.json`
- `melange cache list|clear|stats` — the circuit cache and the compiled-binary cache
- `melange import <kicad_netlist>` — import KiCad netlist to .cir format
- `melange builtins` — list built-in example circuits

The CLI lives in `tools/melange-cli/src/`: `cli.rs` (the clap definitions),
`args.rs`, `common.rs`, and one module per subcommand in `cmd/`.

## The Generality vs. Performance Problem

The central engineering challenge. A hand-written 8x8 DK solver (like OpenWurli's) fits entirely in CPU registers and runs at a few ns/sample. A generic solver with dynamic matrix sizes would require heap allocation and lose cache locality.

**Solution: Compile-time specialization.**

The codegen pipeline emits Rust source code with const-generic sizes. The generated code is compiled by rustc with full optimization — zero overhead vs. hand-written code.

The workflow:
```
netlist.cir → melange compile → circuit.rs → cargo build → production binary
```

The generated `circuit.rs` contains:
- `const` arrays for all precomputed matrices (S, K, A_neg, S*N_i, DC_OP, etc.)
- A `process_sample()` function with inlined NR iteration
- `CircuitState` struct with all solver state (stack-allocated arrays)
- Per-block O(N^3) matrix rebuild for runtime pot/wiper control
- `set_sample_rate()` for runtime matrix recomputation from G+C
- `#[inline(always)]` on the hot path

Performance (re-measured 2026-09-30 on an idle AMD Ryzen 9 7950X pinned to one CCD, single core, noiseless, `-C target-cpu=x86-64-v3`, via `tools/perf-harness/bench.sh`; host-dependent): light nonlinear circuits run into the hundreds of × realtime (a single 12AX7 stage ~156×), typical multi-device audio circuits ~17–46× (Wurlitzer preamp ~46×, tweed amp ~17×), and the heaviest validated circuits ~6.6–18× — a passive tube EQ (nodal Schur, N=52, M=8) at ~18×, a bus compressor at ~6.6×. The triode rows carry the cost of the Dempwolf & Zölzer grid-current law (`30915fb`); `docs/limitations.md` has the figures.
