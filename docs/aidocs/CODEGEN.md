# Code Generation Reference

## Generated Structure

### Constants (Compile-Time)
```rust
const N: usize = 4;           // Number of nodes
const M: usize = 2;           // Nonlinear device dimensions
const SAMPLE_RATE: f64 = 48000.0;
const INPUT_RESISTANCE: f64 = 1.0;  // 1Ω near-ideal voltage source
const OVERSAMPLING_FACTOR: usize = 2;       // 1, 2, or 4
const INTERNAL_SAMPLE_RATE: f64 = 96000.0;  // Only when factor > 1

// G and C stored for runtime recomputation via set_sample_rate()
const G: [[f64; N]; N] = [...];        // Conductance matrix
const C: [[f64; N]; N] = [...];        // Capacitance matrix
const N_V: [[f64; N]; M] = [...];      // Voltage extraction (constant)
const N_I: [[f64; N]; M] = [...];      // Current injection (constant)

// Per-device constants
const DEVICE_0_IS: f64 = 1e-12;
const DEVICE_0_N_VT: f64 = 0.026;
const DEVICE_1_IDSS: f64 = 2e-3;      // JFET
const DEVICE_1_VP: f64 = -2.0;
const DEVICE_1_SIGN: f64 = 1.0;       // +1.0 N-ch, -1.0 P-ch
```

### State (Runtime, Mutable Matrices)
```rust
struct CircuitState {
    // Matrices recomputed by set_sample_rate()
    s: [[f64; N]; N],          // S = A^{-1} (at internal rate)
    a_neg: [[f64; N]; N],      // A_neg = alpha*C - G
    k: [[f64; M]; M],          // K = N_v*S*N_i
    s_ni: [[f64; M]; N],       // S*N_i (for final voltages)
    // ...
}
```

### DC Operating Point Constants
```rust
// For circuits with nonlinear devices where DC OP has non-zero i_nl:
pub const DC_NL_I: [f64; M] = [1.234e-3, 5.678e-6];  // From dc_op solver
```

Emitted when `has_dc_nl` is true (M > 0 and any dc_nl_current is nonzero).
Used to initialize `i_nl_prev` in both `Default` and `reset()`.

### Plugin-Runtime Surface (Oomox roadmap P1–P5)

The generated module exposes a stable, name-addressable surface for plugin
code. Indices computed from netlist position are error-prone — a new voltage
source added above the one you care about shifts every downstream index.
These constants and accessors let plugin code reference the topology by name.

#### Named topology constants (Phase A: P2 + P3)

For every user-named circuit node, voltage source, and `.pot`:

```rust
pub const NODE_IN: usize = 0;
pub const NODE_OUT: usize = 1;
pub const NODE_VCC: usize = 2;
// ...

pub const VSOURCE_VCC_RHS_ROW: usize = 4;   // = n_nodes + vs.ext_idx
pub const VSOURCE_VCTRL_RHS_ROW: usize = 5;

pub const POT_RVOL_INDEX: usize = 0;
pub const POT_RTONE_INDEX: usize = 1;
```

Names are sanitized to `SCREAMING_SNAKE` (non-alphanumeric → `_`, leading
digit prefixed with `_`). Collisions dedupe with numeric suffixes in
declaration order (`FOO`, `FOO_2`, `FOO_3`). Ground (index 0) is implicit.
BJT `basePrime`/`colPrime`/`emitPrime` and transformer-decomposition
internal nodes are deliberately NOT emitted — those are solver
implementation details.

#### `.runtime V` — host-driven voltage source (Phase B: P1)

Netlist:
```spice
Vctrl ctrl 0 DC 0
.runtime Vctrl as ctrl_voltage
```

Emitted:
```rust
struct CircuitState {
    // ...
    pub ctrl_voltage: f64,
}

impl Default for CircuitState {
    fn default() -> Self { Self { /* ... */ ctrl_voltage: 0.0, /* ... */ } }
}

fn build_rhs(input: f64, input_prev: f64, state: &CircuitState) -> [f64; N] {
    // ...
    rhs[VSOURCE_VCTRL_RHS_ROW] += state.ctrl_voltage;
    rhs
}
```

Stamped in both trapezoidal and backward-Euler RHS paths and in the nodal
solver's per-sample RHS builder. `reset()` zeroes each runtime field.
Semantics: additive with any DC bias declared on the voltage source.

#### `.runtime R` — audio-rate resistor modulation (shipped 2026-04-19)

Netlist:
```spice
Rbias cathode 0 10k
.runtime Rbias 2k 12k as bias_r
```

Emitted:
```rust
pub const RUNTIME_R_BIAS_R_MIN: f64     = POT_0_MIN_R;     // 2_000.0
pub const RUNTIME_R_BIAS_R_MAX: f64     = POT_0_MAX_R;     // 12_000.0
pub const RUNTIME_R_BIAS_R_NOMINAL: f64 = 1.0 / POT_0_G_NOM; // 10_000.0

impl CircuitState {
    /// Current resistance of runtime resistor `bias_r` (ohms).
    #[inline]
    pub fn bias_r(&self) -> f64 { self.pot_0_resistance }

    /// Audio-rate safe: no internal smoothing — caller is the smoother.
    pub fn set_runtime_R_bias_r(&mut self, resistance: f64) {
        if !resistance.is_finite() { return; }
        let r = resistance.clamp(RUNTIME_R_BIAS_R_MIN, RUNTIME_R_BIAS_R_MAX);
        if (r - self.pot_0_resistance).abs() < 1e-12 { return; }
        self.pot_0_resistance = r;
        self.matrices_dirty = true;
    }
}
```

Internally shares the `.pot` storage (`pot_N_resistance`) and rebuild
path (`matrices_dirty` → per-block A/S/K rebuild in `process_sample`).
Setter body is structurally identical to `set_pot_N` since the
2026-04-20 reseed strip — the remaining difference is API shape (the
field-named setter, a read-only `state.<field>()` accessor, no
nih-plug knob). Emitted by both DK (`dk_emitter.rs`) and nodal
(`nodal_emitter.rs`) paths; plugin template (`main.rs` in melange-cli)
filters runtime-R entries out of `pot_params` so no `FloatParam` is
generated.

`.gang` members must be `.pot`/`.wiper`; a `.runtime R` resistor listed
in a `.gang` is rejected at parse time (the two surfaces do not compose).
Drive multiple runtime-R fields from a single plugin-side envelope tick
instead.

#### `dc_op()` accessor + `dc_op_dump()` (Phase C: P4)

```rust
impl CircuitState {
    /// Baked DC operating point (frozen at codegen time at nominal pot/switch
    /// values). Prefer this over `v_prev` for the *designed* bias point.
    pub fn dc_op(&self) -> &[f64; N] { &self.dc_operating_point }

    /// Dump each named node's DC voltage to stderr.
    /// Emitted only when named nodes exist.
    pub fn dc_op_dump(&self) { /* eprintln per NODE_<NAME> */ }
}
```

#### `WARMUP_SAMPLES_RECOMMENDED` (Phase D: P5)

```rust
pub const WARMUP_SAMPLES_RECOMMENDED: usize = 528;  // circuit-dependent
```

Computed at codegen time as `ceil(5 · τ_max · internal_rate)` where
`τ_max = max_i(C[i][i] / G[i][i])` over the augmented system. Covers 99.3%
of the slowest pole's final settled response. Plugins applying per-instance
pot/switch jitter should run this many silent samples after configuring
state before audio starts:

```rust
let mut state = CircuitState::default();
state.pot_0_resistance = jittered_r;
for _ in 0..WARMUP_SAMPLES_RECOMMENDED {
    let _ = process_sample(0.0, &mut state);
}
```

Heuristic only — tightly-coupled RC networks where the slowest mode is
orthogonal to every node's individual τ can still fool it. Eigen upgrade
is deferred; the cheap per-node scan covers the common case.

#### `recompute_dc_op()` (Phase E: P6)

Opt-in runtime DC-OP solver that replaces the `WARMUP_SAMPLES_RECOMMENDED`
silence loop above:

```rust
let mut state = CircuitState::default();
state.pot_0_resistance = jittered_r;
state.recompute_dc_op();   // ← jumps to new fixed point directly
// plugin ready to process audio, no warmup needed
```

Emitted only when `--emit-dc-op-recompute` is passed to `melange compile`
(default OFF). Implementation is a specialization of
`crates/melange-solver/src/dc_op.rs` baked at codegen time: N, M, device
slots, pot→row mapping, per-device stamps all unrolled into the generated
module. Not audio-thread safe (hundreds of microseconds per call — call
from init or parameter-change callbacks).

**DK path**: full runtime NR with LU solve + flat damping + convergence.
Converges to `DC_OP` bitwise for inductor-free circuits; inductor-bearing
circuits converge to the `process_sample(0.0)` steady state instead
(companion-shunt equilibrium differs from `dc_op.rs`'s inductor-short
equilibrium — see `docs/aidocs/DC_OP.md`).

**Nodal full-LU path**: ships a stub that bumps `diag_nr_max_iter_count`
and returns without touching state — **this is the permanent path for
nodal circuits**, not a placeholder. The surface matches the DK path so
plugin host code doesn't need a solver-path branch, but plugins on the
nodal route must watch the counter and fall back to the warmup-silence
loop. The full nodal body is deferred indefinitely; no shipping plugin
blocks on it, and the warmup loop already uses the per-sample NR which
converges to the physically correct DC OP. See
`docs/aidocs/DC_OP.md` and the `phase_e_handoff_runtime_dc_op` memory
for the reviver's plan if priorities change.

**`settle_dc_op()` wrapper**: also emitted behind the flag. Calls
`recompute_dc_op` first; on NR failure or nodal stub tick, falls back
to `WARMUP_SAMPLES_RECOMMENDED` iterations of `process_sample(0.0, self)`.
Uniform API across DK and nodal — plugin code doesn't need a
solver-path branch. Preferred over writing the recompute+fallback
pattern by hand on the host side.

State touched by the DK-path method (mirror of `reset()` but preserving
noise RNG, pot/switch values, device runtime params, and diag counters):
`dc_operating_point`, `v_prev`, `input_prev`, `i_nl_prev`/`i_nl_prev_prev`,
`dc_block_x_prev` (seeded at new DC output, not 0),
`pot_N_resistance_prev` (synced to `pot_N_resistance`),
oversampler taps zeroed, inductor / coupled-inductor / transformer history
zeroed. See `docs/aidocs/DC_OP.md` "Runtime DC OP recompute" for the full
contract and derivation.

### State (Runtime, Per-Channel)
```rust
struct CircuitState {
    v_prev: [f64; N],        // Previous node voltages
    i_nl_prev: [f64; M],     // Previous nonlinear currents (init from DC_NL_I if present)
    input_prev: f64,          // Previous input (for trapezoidal RHS)
    dc_operating_point: [f64; N],
    last_nr_iterations: u32,
    // Runtime matrices (recomputed by set_sample_rate)
    s: [[f64; N]; N],
    a_neg: [[f64; N]; N],
    k: [[f64; M]; M],
    s_ni: [[f64; M]; N],
    // Oversampler state (when factor > 1)
    os_up_state: [f64; STATE_SIZE],
    os_dn_state: [f64; STATE_SIZE],
}
```

## Device Model Functions

The codegen generates device-specific functions based on the circuit:

### Diode (1D per device)
```rust
fn diode_current(v_d: f64) -> f64;       // IS * (exp(v/(N*VT)) - 1)
fn diode_conductance(v_d: f64) -> f64;   // (IS/(N*VT)) * exp(v/(N*VT))
```

### BJT (2D per device)
```rust
fn bjt_ic(vbe: f64, vbc: f64) -> f64;              // Ebers-Moll Ic
fn bjt_ib(vbe: f64, vbc: f64) -> f64;              // Ebers-Moll Ib
fn bjt_jacobian(vbe: f64, vbc: f64) -> [f64; 4];   // [dIc/dVbe, dIc/dVbc, dIb/dVbe, dIb/dVbc]
```

### BJT Thermal Constants (Electrothermal Model)
When self-heating is enabled (RTH is finite), additional per-device constants are emitted:
```rust
const DEVICE_0_RTH: f64 = 50.0;       // Thermal resistance [K/W]
const DEVICE_0_CTH: f64 = 1e-6;       // Thermal capacitance [J/K]
const DEVICE_0_XTI: f64 = 3.0;        // IS temperature exponent
const DEVICE_0_EG: f64 = 1.11;        // Bandgap energy [eV]
```

A `BjtThermalState` is added to `CircuitState` for each thermal-enabled BJT, tracking `Tj`,
`IS(T)`, and `VT(T)`. The thermal update runs once per sample (quasi-static), outside the
NR loop. Forward Euler when stable (`dt < 2 * Rth * Cth`), implicit Euler otherwise.
Zero overhead when RTH is infinite (default).

### BJT Charge Storage Constants
When junction capacitance parameters are specified, additional per-device constants are emitted:
```rust
const DEVICE_0_TF: f64 = 1e-10;       // Forward transit time [s]
const DEVICE_0_CJE: f64 = 5e-12;      // B-E zero-bias junction cap [F]
const DEVICE_0_VJE: f64 = 0.75;       // B-E built-in potential [V]
const DEVICE_0_MJE: f64 = 0.33;       // B-E grading coefficient
const DEVICE_0_CJC: f64 = 2e-12;      // B-C zero-bias junction cap [F]
const DEVICE_0_VJC: f64 = 0.75;       // B-C built-in potential [V]
const DEVICE_0_MJC: f64 = 0.33;       // B-C grading coefficient
```

These caps are linearized at the DC operating point and stamped into the MNA C matrix.
The DK framework requires linear C, so caps are fixed at DC OP values. Zero overhead
when all charge params are 0 (default).

### JFET (2D per device)
```rust
fn jfet_id(vgs: f64, vds: f64, idss: f64, vp: f64, lambda: f64, sign: f64) -> f64;  // Shichman-Hodges Id
fn jfet_ig(vgs: f64, sign: f64) -> f64;                                                // Gate current (≈0)
fn jfet_jacobian(vgs: f64, vds: f64, idss: f64, vp: f64, lambda: f64, sign: f64) -> [f64; 4];  // [dId/dVgs, dId/dVds, dIg/dVgs, dIg/dVds]
```

2D Shichman-Hodges model: drain current (Id) at `start_idx`, gate current (Ig) at `start_idx+1`.

- **Drain current**: Triode region (|Vds| < |Vgst|): `Id = IDSS * (2*Vgst*Vds - Vds^2) / Vp^2 * (1 + lambda*|Vds|)`; Saturation region: `Id = IDSS * (1 - Vgs/Vp)^2 * (1 + lambda*|Vds|)`
- **Gate current**: Approximately zero (insulated gate approximation)
- **Jacobian**: 4-element `[dId/dVgs, dId/dVds, dIg/dVgs, dIg/dVds]` — dIg/dVgs and dIg/dVds are effectively zero
- N-channel (`.model name NJ(...)`): sign=+1.0, default VTO=-2.0
- P-channel (`.model name PJ(...)`): sign=-1.0, default VTO=+2.0
- Default IDSS=2e-3 A, lambda=0.001
- Constants: `DEVICE_{n}_IDSS`, `DEVICE_{n}_VP`, `DEVICE_{n}_LAMBDA`, `DEVICE_{n}_SIGN`
- Parameter lookup: `VTO` only (no `VT` alias, to avoid confusion with thermal voltage)

### Tube/Triode (2D per device)
```rust
fn tube_ip(vgk: f64, vpk: f64, mu: f64, ex: f64, kg1: f64, kp: f64, kvb: f64,
           lambda: f64) -> f64;                                                            // Koren plate current with Early effect
fn tube_ig(vgk: f64, ig_max: f64, vgk_onset: f64) -> f64;                                // Leach grid current
fn tube_jacobian(vgk: f64, vpk: f64, mu: f64, ex: f64, kg1: f64, kp: f64, kvb: f64,
                 ig_max: f64, vgk_onset: f64, lambda: f64) -> [f64; 4];  // [dIp/dVgk, dIp/dVpk, dIg/dVgk, dIg/dVpk]
```

2D model: plate current (Ip) at `start_idx`, grid current (Ig) at `start_idx+1`.

- **Plate current**: Koren model with Early-effect lambda — `Ip = Ip_koren * (1 + lambda * Vpk)` where `Ip_koren = E1^ex / kg1` and `E1 = (vpk/kp) * ln(1 + exp(kp * (1/mu + vgk/sqrt(kvb + vpk^2))))`
- **Grid current**: Leach power-law — `Ig = ig_max * (vgk / vgk_onset)^1.5` for vgk > 0 (zero for vgk <= 0)
- **Jacobian**: 4-element `[dIp/dVgk, dIp/dVpk, dIg/dVgk, dIg/dVpk]` — dIg/dVpk = 0. With lambda: `dIp/dVgk = dIp_koren/dVgk * (1+lambda*Vpk)`, `dIp/dVpk = dIp_koren/dVpk * (1+lambda*Vpk) + Ip_koren * lambda`
- Constants: `DEVICE_{n}_MU`, `DEVICE_{n}_EX`, `DEVICE_{n}_KG1`, `DEVICE_{n}_KP`, `DEVICE_{n}_KVB`, `DEVICE_{n}_IG_MAX`, `DEVICE_{n}_VGK_ONSET`, `DEVICE_{n}_LAMBDA`
- Netlist syntax: `T1 grid plate cathode modelname` with `.model modelname TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)`
- Default params: MU=100, EX=1.4, KG1=1060, KP=600, KVB=300, IG_MAX=2e-3, VGK_ONSET=0.5, LAMBDA=0.0
- `LAMBDA` controls Early-effect multiplier: `Ip = Ip_koren * (1 + lambda * Vpk)`. Default 0.0 = no correction (backward compatible). Plate resistance rp ~ 1/(lambda*Ip).
- Template: `device_tube.rs.tera`

## Device Map and M-Dimension Assignment

Nonlinear devices occupy M dimensions in order of appearance in the netlist:
- Diode: 1 dimension (controlling voltage Vd, current Id)
- BJT: 2 dimensions (Vbe->Ic, Vbc->Ib)
- JFET: 2 dimensions (Vgs,Vds->Id, Vgs->Ig)
- Tube: 2 dimensions (Vgk->Ip, Vpk->Ig)
- MOSFET: 2 dimensions (Vgs,Vds->Id, Ig=0)

Example for a circuit with D1, Q1, X1 (triode):
```
M index:  0      1       2       3       4
Device:   D1     Q1(Ic)  Q1(Ib)  X1(Ip)  X1(Ig)
Dim:      1D     -----2D-----    ------2D------
```

## Functions

### build_rhs
```rust
fn build_rhs(input: f64, input_prev: f64, state: &CircuitState) -> [f64; N] {
    // RHS_CONST (DC sources, if any)
    // + A_neg * v_prev  (includes capacitor history via alpha*C!)
    // + N_i^T * i_nl_prev  (part of trapezoidal nonlinear integration)
    // + (input + input_prev) / INPUT_RESISTANCE  (proper trapezoidal)
}
```
A_neg already contains alpha*C. Do NOT add separate cap_history.
The input uses proper trapezoidal integration: `(V_in(n+1) + V_in(n)) * G_in`.
The `N_i * i_nl_prev` term, combined with `S * N_i * i_nl` in `compute_final_voltages`,
gives the trapezoidal average `N_i * (i_nl[n+1] + i_nl[n])`.

### solve_nonlinear
```rust
fn solve_nonlinear(p: &[f64; M], state: &mut CircuitState) -> [f64; M] {
    // Newton-Raphson with:
    // - Warm start from i_nl_prev
    // - SPICE-style voltage limiting (pnjlim/fetlim with per-device VCRIT constants)
    // - Block-diagonal device Jacobian (see below)
    // - Singular check: |det| > 1e-15
    // - NaN/Inf protection
}
```

#### NR Iteration Body
```rust
// 1. Controlling voltages: v_d = p + K * i_nl
let v_d0 = p[0] + K[0][0] * i_nl[0] + K[0][1] * i_nl[1];
// ...

// 2. Device currents and Jacobian entries (device-type-specific)
// For diode at index d:
let i_dev_d = diode_current(v_d_d);
let jdev_d_d = diode_conductance(v_d_d);

// For BJT at indices (s, s+1):
let i_dev_s = bjt_ic(v_d_s, v_d_s1);
let i_dev_s1 = bjt_ib(v_d_s, v_d_s1);
let bjt_jac = bjt_jacobian(v_d_s, v_d_s1);
let jdev_s_s = bjt_jac[0];    // dIc/dVbe
let jdev_s_s1 = bjt_jac[1];   // dIc/dVbc
let jdev_s1_s = bjt_jac[2];   // dIb/dVbe
let jdev_s1_s1 = bjt_jac[3];  // dIb/dVbc

// 3. Residuals
let f_i = i_nl[i] - i_dev_i;

// 4. NR Jacobian (block-diagonal chain rule)
// J[i][j] = delta_ij - sum_k jdev_{ik} * K[k][j]
// where k ranges over the device block that owns row i
```

### process_sample
```rust
// Without oversampling (factor=1):
pub fn process_sample(input: f64, state: &mut CircuitState) -> f64 { ... }

// With oversampling (factor=2):
fn process_sample_inner(input: f64, state: &mut CircuitState) -> f64 { ... }
pub fn process_sample(input: f64, state: &mut CircuitState) -> f64 {
    // Upsample: [input, halfband_up(input)]
    // Process both samples through process_sample_inner
    // Downsample: halfband_dn(out0) → return out1
}
```

The inner function contains the standard DK pipeline:
```rust
fn process_sample_inner(input: f64, state: &mut CircuitState) -> f64 {
    let rhs = build_rhs(input, state.input_prev, state);
    let v_pred = mat_vec_mul_s(&rhs, state);  // Uses state.s (runtime matrix)
    let p = extract_controlling_voltages(&v_pred);
    let i_nl = solve_nonlinear(&p, state);
    let v = compute_final_voltages(&v_pred, &i_nl, state);

    state.v_prev = v;
    state.i_nl_prev = i_nl;
    state.input_prev = input;  // Track for trapezoidal RHS

    v[OUTPUT_NODE]
}
```

## Key Implementation Details

### K Matrix Sign
```rust
// K = N_v * S * N_i (no negation needed)
// K is naturally negative for stable circuits because:
//   N_i[anode] = -1 (current extracted), N_i[cathode] = +1 (current injected)
//   This makes K negative, providing correct negative feedback
```

### Generalized Block-Diagonal Jacobian
```rust
// For each row i, the device Jacobian J_dev is block-diagonal:
// - Diode at index d: J_dev[d][d] = g_d (1x1 block)
// - BJT at (s, s+1): J_dev is 2x2 block [dIc/dVbe, dIc/dVbc; dIb/dVbe, dIb/dVbc]
//
// NR Jacobian uses chain rule:
// J[i][j] = delta_ij - sum_k(jdev_{ik} * K[k][j])
// where k ranges over indices in i's device block
//
// For diode row d:  j_{dj} = delta_{dj} - jdev_{d}_{d} * K[d][j]
// For BJT Ic row s: j_{sj} = delta_{sj} - jdev_{s}_{s} * K[s][j] - jdev_{s}_{s+1} * K[s+1][j]
```

### History Handling
```rust
// WRONG: Separate history vector
let history = alpha * C * v_prev;
rhs += history;

// RIGHT: A_neg contains history
// A_neg = alpha*C - G
// rhs = A_neg * v_prev = alpha*C*v_prev - G*v_prev
//       ^^^^^^^^^^^^^ history term included!
```

### Linear Solve by M Size
- M=1: Direct division
- M=2: Cramer's rule (explicit 2x2 inverse)
- M=3..24: Inline Gaussian elimination with partial pivoting
- M>24: Not supported (MAX_M=24)

### Chord Persistence in Full-LU Nodal Path
The `emit_nodal_process_sample` full-LU path keeps the chord LU factorisation
across NR iterations and across samples. The factor is rebuilt only when:
1. `chord_valid == false` (e.g. just after `set_sample_rate`)
2. `iter > 0 && iter % CHORD_REFACTOR == 0` (default `CHORD_REFACTOR = 5`)
3. `iter >= 10` (force-refactor after 10 iters)
4. (BoyleDiodes only) Adaptive trigger: any device's diagonal `j_dev[k][k]`
   shifted >50 % relative to `chord_j_dev[k][k]`. Catches abrupt-knee
   transitions like Boyle catch diodes that go from `j_dev ≈ 1e-31`
   reverse-biased to `j_dev ≈ 1e+1` forward-biased within one sample.

The companion-RHS construction `i_comp = i_nl − chord_j_dev · v_nl` is exact
only when `chord_j_dev == J_dev_at_v`. For smoothly-conducting devices (BJT,
JFET, MOSFET, tube, Schottky diode) this stays approximately true between
refactors. For abrupt-knee devices (Boyle catch diodes with `IS=1e-15 N=1`)
the chord can be many orders of magnitude stale and the LU back-solve
produces a non-physical "fixed point" that the standard voltage-step
convergence check accepts. **Mitigations** (BoyleDiodes-gated):

- **Residual safety net**: re-evaluate `i_nl_fresh` from device equations
  at the post-step v and require `|i_nl_fresh − i_nl_chord| < tol` in
  addition to the voltage-step check. Mirrors the DK Schur path's
  convergence gate. Emitted in both the trapezoidal main NR loop and the
  BE fallback NR loop.
- **Adaptive refactor**: see #4 above.
- See `DEBUGGING.md` "Op-amp BoyleDiodes Failure Signatures" for the
  remaining open issue (heavy clipping → bistable Newton oscillation).

## Verification Checklist
- [ ] INPUT_RESISTANCE matches G matrix stamping (default: 1 ohm)
- [ ] A_NEG contains alpha*C terms
- [ ] No separate cap_history in build_rhs
- [ ] K has no extra negation (naturally negative from kernel)
- [ ] Jacobian uses block-diagonal jdev entries (not just diagonal g_dev)
- [ ] SPICE-style voltage limiting present (pnjlim/fetlim with per-device VCRIT)
- [ ] is_finite() checks present
- [ ] S matrix values reasonable (< 1e6)

## DC Operating Point in Codegen

The codegen pipeline calls `dc_op::solve_dc_operating_point()` to find the DC bias:

```rust
// In CircuitIR::from_kernel():
let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);
ir.dc_operating_point = dc_result.v_node;
ir.dc_nl_currents = dc_result.i_nl;
ir.dc_op_converged = dc_result.converged;
```

The emitter checks `has_dc_nl = M > 0 && dc_nl_currents.iter().any(|&x| x.abs() > 1e-30)`:
- If true: emits `DC_NL_I` constant, initializes `i_nl_prev` from it
- If false: initializes `i_nl_prev` to `[0.0; M]`

`CodegenConfig` fields: `dc_op_max_iterations` (default 200), `dc_op_tolerance` (default 1e-9).

## Oversampling

Generated code supports 2x and 4x oversampling via `CodegenConfig.oversampling_factor` (default: 1).

### How It Works
When factor > 1, the codegen:
1. Recomputes all matrices (S, A_neg, K, S_NI) at `sample_rate * factor` from stored G+C
2. Renames `process_sample` → `process_sample_inner` (private)
3. Emits a public `process_sample` wrapper that upsamples → processes → downsamples
4. Emits self-contained polyphase allpass half-band filter functions inline (no external dependencies)

### Filter Design
- **2x**: 3-section polyphase allpass half-band (~80dB stopband rejection)
- **4x**: Cascaded 2x — outer stage uses 2-section (~60dB), inner stage uses 3-section (~80dB)
- Coefficients from `melange-primitives/src/oversampling.rs` (HB_3SECTION, HB_2SECTION)

### State Fields
- `os_up_state`, `os_dn_state`: allpass filter state for upsample/downsample (2x)
- `os_up_state_outer`, `os_dn_state_outer`: additional state for outer stage (4x only)
- All reset in `reset()` and reinitialized in `set_sample_rate()`

### `set_sample_rate()` with Oversampling
When oversampling is active, `set_sample_rate(sr)` computes `internal_rate = sr * OVERSAMPLING_FACTOR`
and recomputes all matrices at the internal rate.

## Sparsity-Aware Emission

The IR includes `SparseInfo` with per-matrix nonzero entry lists (threshold: `|x| < 1e-20` = structural zero).
The emitter skips zero entries uniformly in:
- `build_rhs`: A_neg * v_prev multiplication
- `extract_controlling_voltages`: N_v * v_pred
- `solve_nonlinear`: K * i_nl
- `compute_final_voltages`: S * N_i * i_nl

This reduces generated code size and improves performance for sparse circuits.

## Runtime Sample Rate Support

Generated code stores G and C as constants and recomputes S, A_neg, K, S_NI at runtime:

```rust
impl CircuitState {
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        let internal_rate = sample_rate * OVERSAMPLING_FACTOR as f64;
        let alpha = 2.0 * internal_rate;
        // Recompute A = G + alpha*C, then S = A^{-1}
        // Recompute A_neg = alpha*C - G
        // Recompute K = N_v * S * N_i
        // Recompute S_NI = S * N_i (for final voltage computation)
        // Recompute inductor g_eq and pot SM vectors if applicable
    }
}
```

Matrix inversion uses inline Gaussian elimination (`invert_n()` helper emitted in generated code).

## Parasitic Cap Handling

Purely resistive nonlinear circuits (no capacitors in netlist) are ill-conditioned for
the trapezoidal-based DK method because `A = G + (2/T)*C` degenerates to just `G` when
`C = 0`. The caller should invoke `MnaSystem::add_parasitic_caps()` before building the
DK kernel to auto-insert 10pF across each device junction. See
[DEVICE_MODELS.md](DEVICE_MODELS.md#parasitic-cap-auto-insertion) for junction topology.

## Codegen-Only Pipeline

The runtime `CircuitSolver` / `NodalSolver` / `DeviceEntry` paths have been
removed. All circuit processing now flows through the codegen pipeline. The
only runtime type that survives is `LinearSolver` (M=0 linear-only fallback)
in `crates/melange-solver/src/linear_solver.rs`.

| Capability | How it's emitted |
|------------|-----------------|
| NR Jacobian | Block-diagonal `jdev_` entries, `J[i][j] = δ_ij - Σ_k jdev_{ik} * K[k][j]` |
| Linear solve | M=1 direct, M=2 Cramer's, M=3..16 unrolled Gauss; full LU for nodal full-LU path |
| Device coverage | Diode (1D), BJT (1D forward-active or 2D), JFET (2D), MOSFET (2D), Tube (2D), VCA (2D), Op-amp (linear) |
| DC OP init | `DC_NL_I` constant automatically initializes `i_nl_prev` |
| Oversampling | 2× / 4× cascaded polyphase half-band IIR |
| Sparsity | Zero entries skipped in emission (per-matrix `SparseInfo`) |
| Sample rate | `set_sample_rate()` recomputes S, A_neg, K, S_NI from emitted G+C |
| Multi-output | Stereo / multi-output supported via multiple output nodes |
| Potentiometers | Sherman-Morrison rank-1 updates in `process_sample` |
| Switches | Per-position matrix rebuild on `set_switch` |
