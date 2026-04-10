# Melange Architecture

## The Five Translation Boundaries

Every circuit modeling project crosses these boundaries. Melange automates boundaries 2-4.

```
Schematic → Netlist → MNA Matrices → DK Kernel → Optimized Rust → Plugin
   [1]        [2]        [3]            [4]          [5]
   Human    melange    melange        melange      nih-plug
```

### Boundary 1: Schematic → Netlist (Human)
Reading a schematic and producing a correct netlist requires domain expertise that cannot be fully automated. Schematic ambiguities (wrong component values, revision mismatches, topology misidentification) have been responsible for the worst bugs in real projects. Melange assists with OCR and validation tools, but a human must verify the topology.

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
- Sherman-Morrison projections for time-varying elements
- Companion model update functions

The generated code should be indistinguishable in performance from hand-written code.

### Boundary 5: Rust → Plugin (nih-plug / melange-plugin)
Handled by nih-plug for the plugin shell. melange-plugin provides helpers for voice management, oversampling decisions, and parameter mapping from physical component values to user-facing controls.

## Crate Architecture

### melange-primitives (Layer 1)
Zero dependencies. `no_std`. The foundation everything else builds on.

**Filters:**
- `OnePoleHpf` / `OnePoleLpf` — simple 6 dB/oct filters
- `TptLpf` — Zavalishin topology-preserving transform (ZDF integrator)
- `DcBlocker` — configurable cutoff (default 20 Hz)
- `Biquad` — DFII-transposed, bandpass/lowpass/highpass/peaking/shelf

**Oversampling:**
- `Oversampler<const FACTOR: usize>` — polyphase IIR half-band
- Configurable rejection (3/4/5 allpass sections per branch)
- `process_block()` for batch operation

**NR Helpers:**
- `nr_solve_1d<F, DF>(f, df, x0, max_iter, tol)` — scalar NR with clamping
- `nr_solve_2d` — 2x2 NR with Cramer's rule (covers most audio circuits)
- NaN recovery: detect divergence, reset to last known good state

**Companion Models:**
- `TrapezoidalCompanion` — capacitor discretization: conductance + history source
- `BilinearCompanion` — for series R-C combinations
- `companion_conductance(C, fs)` and `companion_update(v_new, v_old)`

**Utilities:**
- `variation_hash(seed, index)` — deterministic per-instance detuning
- `midi_to_freq(note)` — standard tuning conversion

### melange-devices (Layer 2)
Depends on melange-primitives. Provides the `NonlinearDevice` trait and implementations.

```rust
/// A nonlinear circuit element for use in MNA/DK solvers.
pub trait NonlinearDevice {
    /// Current as a function of voltage: i(v)
    fn current(&self, v: f64) -> f64;
    /// Derivative of current: di/dv (for Newton-Raphson Jacobian)
    fn transconductance(&self, v: f64) -> f64;
}
```

**Implementations:**
- `BjtEbersMoll { is: f64, vt: f64, beta_f: f64, beta_r: f64 }` — NPN/PNP BJT
- `BjtGummelPoon { ... }` — extended BJT model (Early effect, high injection)
- `DiodeShockley { is: f64, n: f64, vt: f64 }` — junction diode
- `KorenTriode { mu: f64, ex: f64, kg1: f64, kp: f64, kvb: f64 }` — vacuum tube
- `KorenPentode { ... }` — vacuum tube pentode
- `CdsLdr { r_min, r_max, gamma, attack_tau, release_tau }` — photoresistor
- `IdealOpamp` / `BoyleOpamp` — operational amplifier models

**SPICE Model Card Import:**
- Parse `.model` statements from SPICE netlists
- Extract parameters into device structs
- Example: `.model 2N5089 NPN(IS=2.64e-15 BF=735 NF=1.0 ...)` → `BjtEbersMoll { ... }`

### melange-solver (Layer 3)
The core. Depends on primitives and devices.

**Netlist Parser:**
- Parse a subset of SPICE sufficient for audio circuits
- Components: R, C, L, V (DC/AC), I, D (diode), Q (BJT), J (JFET), M (MOSFET), T (tube), U (op-amp), Y (VCA), X (subcircuit), E (VCVS), G (VCCS), K (coupled inductors)
- Directives: `.model`, `.subckt`, `.pot`, `.wiper`, `.switch`, `.gang`, `.input_impedance`
- Output: `Netlist` struct with elements, models, and directives

**MNA Assembler:**
- `MnaSystem::from_netlist(&netlist)` → `MnaSystem { g, c, n_v, n_i, ... }`
- Automatic node numbering (ground = 0)
- Automatic stamp generation for all component types
- Augmented MNA for inductors (branch current variables)

**DK Kernel / Nodal Solver:**
- `DkKernel::from_mna(&mna, sample_rate)` → DK kernel with K matrix
- Three codegen paths: DK Schur, Nodal Schur, Nodal Full LU (auto-selected)
- Sherman-Morrison projection vectors for `.pot`/`.wiper` elements

**Code Generator (codegen-only pipeline):**
- `CircuitIR::from_kernel(...)` → intermediate representation
- `RustEmitter::emit(&ir)` → optimized Rust source code
- Const-generic matrix sizes, inlined NR, precomputed matrices as `const` arrays
- Tera templates for device-specific codegen

### melange-validate (Layer 4)
Depends on solver. Requires ngspice installed on the system.

**ngspice Runner:**
- Generate transient/AC/DC analysis netlists from circuit descriptions
- Invoke ngspice, parse raw binary or ASCII output
- Extract node voltages, branch currents, frequency response

**Comparison Engine:**
- `compare_dc(spice_op, rust_op, tolerance)` — DC operating point validation
- `compare_ac(spice_sweep, rust_sweep, freq_range, db_tolerance)` — frequency response
- `compare_thd(spice_harmonics, rust_harmonics, tolerance)` — harmonic distortion
- `compare_transient(spice_wav, rust_wav, time_range, tolerance)` — waveform comparison

**Measurement Routines:**
- `measure_gain(dut, freq, amplitude)` — sine-through-DUT gain
- `measure_sweep(dut, f_start, f_end, points)` — frequency sweep
- `measure_harmonics(dut, freq, amplitude, num_harmonics)` — THD analysis
- `measure_centroid(signal, fs)` — spectral centroid tracking

**Test Generators:**
- Given SPICE reference data, emit `#[test]` functions with tolerance assertions
- Three-tier methodology: circuit-only, voice-model, full-plugin

### melange-plugin (Layer 5)
Depends on solver. Stub for future nih-plug integration helpers.

### melange-cli
The command-line interface for working with circuits.

**Subcommands:**
- `melange compile <netlist>` — parse netlist, generate optimized Rust code or plugin project
- `melange simulate <netlist>` — compile and run circuit, output WAV
- `melange analyze <netlist>` — AC frequency response analysis
- `melange validate <netlist>` — compare against ngspice, report deltas
- `melange nodes <netlist>` — show circuit nodes and devices
- `melange import <kicad_netlist>` — import KiCad netlist to .cir format
- `melange builtins` — list built-in example circuits

## The Generality vs. Performance Problem

The central engineering challenge. A hand-written 8x8 DK solver (like OpenWurli's) fits entirely in CPU registers and runs at ~4 ns/sample. A generic solver with dynamic matrix sizes would require heap allocation and lose cache locality.

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
- Sherman-Morrison vectors for runtime pot/wiper control
- `set_sample_rate()` for runtime matrix recomputation from G+C
- `#[inline(always)]` on the hot path

Performance: DK codegen circuits run 100-600x realtime. Nodal full-LU with chord + sparse optimizations: ~11x realtime for the most complex validated circuit (Pultec EQP-1A, N=41, M=8).
