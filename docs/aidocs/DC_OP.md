# DC Operating Point Solver

## Purpose

Computes the steady-state bias point of circuits with nonlinear devices (diodes, BJTs)
by solving the nonlinear system:

```
G_dc · v = b_dc + N_i · i_nl(N_v · v)
```

Without a DC OP solver, circuits with DC bias (e.g., BJT amplifiers powered by VCC)
start from v=0 (all devices in cutoff), producing no output. SPICE always computes
a DC operating point before transient analysis.

## Module

`crates/melange-solver/src/dc_op.rs`

Called from the codegen pipeline (`CircuitIR::from_kernel()`). The pre-removal
runtime entry-point (`CircuitSolver::initialize_dc_op()`) no longer exists.

## References

- SPICE DC analysis: Pillage & Rohrer Ch. 4
- Source stepping: Nagel (1975), "SPICE2: A Computer Program to Simulate Semiconductor Circuits"
- Gmin stepping: ngspice manual §15.3

## Newton-Raphson Companion Formulation

At each NR iteration, linearize the nonlinear devices as Norton equivalents:

```
F(v) = G_dc · v - b_dc - N_i · i_nl(N_v · v) = 0

dF/dv = G_dc - N_i · J_dev · N_v

G_aug = G_dc - N_i · J_dev · N_v       (augmented conductance)
rhs   = b_dc + N_i · (i_nl - J_dev · v_nl)  (companion Norton current)
v_new = G_aug^{-1} · rhs               (solve)
```

### Sign Convention (Critical!)

The Jacobian stamping uses **subtraction**:
```
G_aug = G_dc - N_i · J_dev · N_v
```

This is correct because:
- `F(v) = G·v - b - N_i·i_nl = 0`
- `dF/dv = G - N_i · J_dev · N_v` (derivative of `-N_i·i_nl` w.r.t. `v`)
- N_i[anode] = -1, so `-(-1) · g_d = +g_d` stamps device conductance **positively**
- This gives correct diagonal dominance for convergence

**Do NOT use addition** (`G_dc + N_i · J_dev · N_v`) — that would double-count
the device conductance with wrong sign, causing divergence.

### Voltage Limiting

Each NR iteration uses junction-aware logarithmic voltage limiting:
```rust
let vt = 0.026; // thermal voltage
let delta = v_new[i] - v[i];
let limited = if delta.abs() < vt {
    delta  // small steps pass unchanged
} else {
    delta.signum() * vt * (delta.abs() / vt + 1.0).ln()
};
v[i] += limited;
```

This compresses large voltage steps (e.g. 300V → ~0.3V) while passing small
steps (~26mV) nearly unchanged — much more effective than a flat clamp for
circuits with both low-voltage junctions and high-voltage supply nodes.

### Limiter back-projection: joint minimum-norm (lane 2, 2026-09-13)

pnjlim/fetlim act in junction space; the node update must reproduce every
limited junction's `v_lim`. `distribute_junction_correction()` is the exact
single-row pseudo-inverse (normalizes by `||N_v row||^2`), but summing it per
junction is exact ONLY when the limited rows are orthogonal. Identical rows
(parallel same-direction diodes / paralleled transistors) sum to
`v_post = 2*v_lim - v_raw` — a REVERSED step whenever `v_raw > 2*v_lim` —
which is exactly how the deep-reverse false root above was reached in one
iteration; overlapping rows (Vbe/Vbc share the base, differential pairs
share the emitter) get half-amplitude cross-talk.

`apply_junction_corrections()` now collects all limited `(row, correction)`
pairs per iteration and solves the minimum-norm node update
`delta = N_L^T (N_L N_L^T)^-1 c` jointly:

1. rows identical up to sign are deduped, keeping the most restrictive
   correction (junction closest to its pre-step value);
2. Gram-Schmidt rank test (`JUNCTION_ROW_DEPENDENCE_TOL` = 1e-9 relative);
3. full rank: EXACT Gram solve, every junction lands precisely on its
   `v_lim`; rank-deficient (e.g. Vbe, Vbc plus a C-E diode): ridge
   `JUNCTION_GRAM_RIDGE_REL` = 1e-9 x max Gram diagonal, ONLY in that case.

Result: the parallel-diode repros converge by Direct NR (4-12 iterations)
instead of exhausting 200 and falling to source stepping. Golden corpus +
openwurli decks: 42/42 `dc-op` node vectors byte-identical; only iteration
counts moved on noyce-germanium-cluster (8->9) and wurli-tremolo (122->125,
Darlington Vbe/Vbc overlap).

### Convergence: step test AND KCL residual gate

Each NR iteration is accepted only if BOTH hold:

1. Per-variable step test (SPICE-style): `|delta_i| < reltol*|v_i| + tolerance`
   (`tolerance` = 1e-9 **volts**, `reltol` = 1e-6).
2. Per-node KCL residual gate, evaluated at the post-damping iterate with
   freshly evaluated device currents:
   `|F_i| <= reltol * scale_i + DC_OP_KCL_ABSTOL_AMPS` for every voltage row,
   `F = G_dc*v - b_dc*scale - N_i*i_nl(N_v*v)`,
   `scale_i = max(|b_i|, |(N_i*i_nl)_i|, max_j |G_ij*v_j|)`,
   `DC_OP_KCL_ABSTOL_AMPS` = 1e-9 **amperes** (a separate constant — do not
   alias it with the volts step floor). Written NaN-safe (`!(x <= tol)`).

The step test alone is satisfied at any fixed point of the limit/damp map,
root or not: two parallel same-direction diodes driven to deep reverse by
summed pnjlim corrections "converge" in 3 iterations to a point violating
KCL by kA (2026-09-13). The gate refuses such an iterate; the loop keeps
going and, on iteration exhaustion, returns `converged = false` so the
strategy ladder falls through (source stepping recovers the true root).

No row is exempt: a pinned op-amp output row (below) is checked against its
own pinned equation. `DcOpResult::kcl_residual_max` / `kcl_worst_row` carry
the residual of the RETURNED solution over all voltage rows; `melange dc-op`
prints them (human and `--format json`).

### Railed op-amps: an active set inside Newton

A railed op-amp output is an active set inside every Newton iteration of every
ladder stage (`dc_rail_pins`, `nr_dc_solve`). Each iteration solves with the
current pin set substituted into its rows, the way the transient's rail mode
pins them:

- `DcRail::Terminal` (hard): the whole output row becomes `v_out = limit`.
- `DcRail::LoadLine` (the active-set modes): the row keeps the node's KCL, the
  VCCS leaves it, `1/ROUT` becomes `1/R_SAG`, and the limit enters as
  `limit/R_SAG` (the output sits at `limit − R_SAG·I_load`).
- `DcRail::Free` (rail mode `none`) and BoyleDiodes op-amps (their rails are
  catch diodes, i.e. devices) are never pinned.

Limits scale with the sources during source stepping; in AOL continuation a
sidechain rectifier's lower limit widens to one volt below its `+` input
(headroom for the rectifier diode's drop).

The next iteration's pin set is read from the RAW Newton solution, before
junction limiting and damping: only the raw solution satisfies the op-amp
rows, so only there does `Gm·(v+ − v−)` read the output the linear model
demands. Terminal tests `AOL·(v+ − v−)`; LoadLine tests
`w = v_out + R_SAG·I_load`, `I_load = Gm·(v+ − v−) − v_out/ROUT`. Both use
the gain of the system being solved (capped at `AOL_DC_MAX` in the ladder, the
step value in AOL continuation, full in the finish). A HELD pin stays while
its own test holds and otherwise releases; it never moves to the other rail in
one step. With the output pinned the loop is open, so `v+ − v−` points at the
opposite rail (a follower pinned high reads `v+ − VCC < 0`), and moving the pin
there alternates between the rails for the whole budget. Released, the next
solve decides the side with the loop closed, as the transient re-solves
unpinned every sample. The start iterate of a solve is tested on the output
node voltage itself, because it can come from another system (the finish
starts from the gain-capped answer, whose `v+ − v−` times the full gain
predicts an output hundreds of volts away). A pin-set change fails the step
test.

`pin_railed_opamps` pins the operating points no full-gain Newton solve
produced: a circuit with no nonlinear devices whose op-amps all sit within
`AOL_DC_MAX` (one linear solve, no rail), and a full-AOL finish that did not
converge (the ladder's gain-capped point). Otherwise `DcOpResult::rail_pin`
is the finish's settled pin set.

Witnesses (`tests/dc_op_rail_active_set_tests.rs`), against ngspice `.op` at
reltol 1e-9 with the op-amp clamped the same way (an 8 V source at the output,
8 V behind 200 Ω, or the VCCS with `ROUT`): an op-amp linearly at 10.87 V
beside a diode, and driving a BJT base, converge under every rail law, within
1 µV of ngspice on the junction node (measured 2026-09-29). Both returned
`Failed` after 200 iterations under every rail mode when the rail was a
post-step clamp: the step test read the pre-clamp step. The held-pin rule has
its own witness (a follower of a diode-clamped node whose linear start is past
the rail).

A build whose operating point did not converge is refused, by every verb
(`build::assemble`): its generated code would start from a state that is not a
solution. `--allow-unconverged-dc-op` (compile, simulate, analyze, dc-op)
builds it anyway, with a warning; `validate` has no override. `melange dc-op`
reports the operating point the build ships: it runs `build::assemble`, the
build every verb uses up to the IR, so the vector it prints is the `DC_OP` the
generated code embeds, on the route that build takes.

The reported residual is computed against the circuit, not the working copy.
`build_dc_system` snapshots `g_circuit` (the shipped `mna.g`, the inductor DC
shorts, the BJT internal-node expansion) before adding the solver aids to the
working `g_dc` (op-amp gain capped at `AOL_DC_MAX`, the 1e-12 S node gmin
floor); the report uses `g_circuit` with the settled rail pins applied as the
transient applies them. An aid, or a mis-stamp, that moves the answer shows
up as residual instead of being certified by the system it changed. The gmin
floor alone reads about 1e-12 S × |v| (1.2e-11 A at a 12 V rail).

Measured before landing (golden corpus + openwurli decks, 41 nonlinear
decks): 0 flips to non-converged, 0.0 V node-voltage change at reltol 1e-6.

The emitted runtime `recompute_dc_op` (`dc_op_emitter.rs`,
`nodal_emitter/dc_op.rs`) gates on the same per-row residual, with the step
test read on the UNDAMPED Newton correction (see "Runtime DC OP Recompute").

### Linear System Solve

The DC OP solver uses LU decomposition with partial pivoting to solve
`G_aug * v = rhs` each NR iteration, rather than full matrix inversion.
This is both faster (O(N²) per solve vs O(N³) per inversion) and more
numerically stable.

## Convergence Strategies

The solver tries three strategies in order:

### 1. Direct NR (DcOpMethod::DirectNr)

Start from the linear DC OP (no nonlinear devices) and iterate NR.
Works for simple circuits (single diode with VCC).

The linear solve has no device currents, so it can put a junction volts into
forward bias. `clamp_junction_voltages` clamps the JUNCTION before Newton
starts:

- **Diode**, only when the guess puts it above 0.8 V forward: the cathode moves
  to `anode − 0.6 V`, or the anode to 0.6 V when the cathode is ground. A
  reverse-biased diode (a zener at breakdown, a Boyle catch diode at rest) is
  left alone: pulling it to −0.6 V moves it toward forward bias.
- **BJT**, from any Vbe: the emitter moves to `base − sign·0.65 V`, or the base
  to `sign·0.65 V` when the emitter is ground. Unlike the diode clamp this also
  raises a cut-off Vbe, a deliberate pre-bias that keeps a feedback amplifier
  out of its all-off solution (skipping reverse Vbe measured +1 iteration on
  the Wurlitzer power amp and no gain anywhere). The clamp acts on the
  device's external terminals (`nonlinear_devices[k].node_indices`), also for a
  BJT whose parasitic RB/RC/RE are expanded into internal nodes: its N_v rows
  then name b′/e′, which the internal-node initialisation overwrites, so a
  clamp read from those rows would move nothing
  (`dc_op_parasitic_bjt_seed_tests.rs` pins it).

The dependent node (cathode, emitter) moves whenever it is a solution
variable, including when a voltage source fixes it (a supply rail): the guess
is then linearised at the clamped junction with every free node where the
linear solve put it, and the source's row restores the rail on the first step.
Moving the other node instead drags a node its bias network holds (measured:
the Wurlitzer power amp's DC OP then fails). Only a grounded dependent node,
which is not a variable, moves the other node. Without that, a grounded-emitter
BJT whose guess holds its base at the driving op-amp's 5.4 V descends one
thermal voltage per Newton iteration (193 iterations; now 7).

SPICE's `MODEINITJCT` (every junction evaluated at `vcrit` on iteration 0) was
measured against this and rejected: layered on melange's linear-guess start it
linearises at `vcrit` with the nodes still at the guess, and on parasitic BJTs
(whose pnjlim the DC loop skips) that first step is unbounded. The Wurlitzer
preamp went from DirectNr 8 to Failed. See STATUS.md, Deferred.

**A `.linearize`d circuit starts from its bias point instead.** `.linearize`
solves the full circuit first, extracts the flagged devices' small-signal
parameters there, and rebuilds them linear with Norton constants from that
point, so that point satisfies the linearized system's DC equations exactly.
`apply_linearize_reductions` records it by node name
(`MnaSystem::linearize_bias_nodes`, only when the bias solve converged),
`dc_op_config` passes it as `DcOpConfig::seed_nodes`, and Direct NR starts
there with no junction clamp. Auxiliary rows keep their usual initialisation;
parasitic-BJT internal nodes b′, c′, e′ start at their external terminals,
because the seed already satisfies the device at sub-mV parasitic drops and a
fixed `b′ ∓ 0.65 V` across a fraction-of-an-ohm RE would put amperes of KCL
error into it. Measured on a FET limiter whose output stage
has a Darlington-equivalent NF = 2 transistor: from the clamped linear guess
the linearized solve failed every strategy (KCL 0.357 A) while the bias solve
had converged; seeded, Direct NR in 2 iterations. Corpus `.linearize` decks
keep their operating point (≤ 1e-15 V) in fewer iterations (18 → 2 typical;
decks with parasitic-BJT internal nodes 10 → 4, 31 → 3).

A bias solve that did not converge is refused (`--allow-unconverged-dc-op`
overrides, and the provenance then records `"linearize_bias_unconverged":true`).
The unconverged-operating-point refusal cannot catch it: the linearized
circuit's own DC operating point solves cleanly, around small-signal
parameters and Norton constants taken from a non-solution.

### 2. Source Stepping (DcOpMethod::SourceStepping)

Scale all DC sources from 0 → full value in `source_steps` stages (default 50).
At each stage, run NR to convergence, then warm-start the next stage.
Start from v=0 (not linear guess).

Works for BJT amplifier bias networks where direct NR fails because
the linear initial guess puts BJT junctions too far from their operating point.

### 3. Gmin Stepping (DcOpMethod::GminStepping)

Add conductance `gmin` across each nonlinear device's controlling nodes.
Ramp logarithmically from `gmin_start` (1e-2) to `gmin_end` (1e-12).
Final solve with gmin=0.

Works for circuits where source stepping fails (rare).

### 4. Fallback (DcOpMethod::Failed)

Return the linear DC OP with `converged: false`. The solver will still work
but start from a wrong bias point, producing transient artifacts.

## DC System Construction

At DC steady state:
- **Resistors**: Normal G-matrix stamps (already in `mna.g`)
- **Capacitors**: Open circuit (C not stamped — `i_C = C·dv/dt = 0` at DC)
- **Inductors**: Short circuit (`VS_CONDUCTANCE` between terminals)
- **Voltage sources**: Norton equivalent (`VS_CONDUCTANCE` + current injection)
- **Input ports**: nothing added. The DC OP solves the `mna.g` it is given,
  which the build has already stamped with every input port's Thevenin
  conductance before the kernel — the same G the transient runs. `DcOpConfig`
  carries no input node or resistance; a caller that wants a port stamps it
  into `mna.g` (`MnaSystem::stamp_input_conductance`).

### BJT Junction Cap Linearization

When BJT charge storage parameters are specified (CJE, CJC, TF), the junction
capacitances are evaluated at the DC operating point and stamped into the MNA C matrix:

- **Depletion cap**: `Cj = CJ0 / (1 - Vj/VJ)^MJ` (with FC=0.5 linear extension for forward bias)
- **Diffusion cap**: `Cd = TF · d(I_F/qb)/dVbe`, `I_F = IS·(exp(Vbe/(NF·VT)) − 1)` (ngspice's `capbe = tf*gbe`; qb = 1 without Gummel-Poon parameters). A forward-active (1D) BJT's Vbc for the depletion cap comes from the node voltages
- Total B-E cap: `CJE_linearized + Cd` stamped across base-emitter nodes
- Total B-C cap: `CJC_linearized` stamped across base-collector nodes

The DK framework requires linear C, so these caps are fixed at DC OP values.
This happens during `CircuitIR::from_kernel()` after the DC OP solve.

### Parasitic Caps

If the circuit has nonlinear devices but zero capacitors, `MnaSystem::add_parasitic_caps()`
should be called before building the DK kernel. This inserts 10pF across each device
junction (see [DEVICE_MODELS.md](DEVICE_MODELS.md#parasitic-cap-auto-insertion)),
ensuring the C matrix is non-trivial for stable trapezoidal integration. Parasitic caps
are inserted before the DC OP solve so that the linearized junction caps (if any) add
to the parasitic base.

## Device Evaluation

Uses `DeviceSlot` params from `codegen::ir`:

- **Diode**: `i = IS * (exp(v/N_VT) - 1)`, `g = (IS/N_VT) * exp(v/N_VT)`
- **BJT**: Ebers-Moll transport model with polarity sign (+1 NPN, -1 PNP)
  - `Ic = sign * IS * (exp(Vbe_eff/VT) - exp(Vbc_eff/VT)) - sign * (IS/BR) * (exp(Vbc_eff/VT) - 1)`
  - `Ib = sign * (IS/BF) * (exp(Vbe_eff/VT) - 1) + sign * (IS/BR) * (exp(Vbc_eff/VT) - 1)`
- **JFET**: 2D Shichman-Hodges with triode + saturation regions, channel-length modulation (lambda); Parker–Skellern for a `LEVEL=2` card (the same `Jfet::evaluate`, see DEVICE_MODELS.md)
  - `Id = sign * IDSS * f(Vgs, Vds, Vp) * (1 + lambda*|Vds|)`; `Ig` = the gate-source and gate-drain junctions, `IS*(exp(V/(N*Vt)) - 1)` each
  - N-channel (sign=+1) / P-channel (sign=-1)
- **MOSFET**: 2D Level 1 SPICE with triode + saturation regions, channel-length modulation (lambda)
  - `Id = sign * KP * f(Vgs, Vds, Vt) * (1 + lambda*|Vds|)`, `Ig = 0`
  - N-channel (sign=+1) / P-channel (sign=-1)
- **Tube**: 2D Koren plate current (with Early-effect lambda) + Dempwolf & Zölzer grid current
  - `Ip = Ip_koren * (1 + lambda*Vpk)` where `Ip_koren = 2 * E1^ex / Kg1`; `Ig = Gg * (softplus(Cg*vgk)/Cg)^xi` (D&Z eq. 11, conducts at every vgk)
  - With `RGI`, both are evaluated at the internal grid, the root of `v + RGI*Ig(v) = Vgk` (`KorenTriode::evaluate_with_rgi`, the transient's `tube_evaluate_with_rgi`)
- **Clamping**: `safe_exp(x) = x.clamp(-40, 40).exp()` matching codegen/runtime

## API

### Types

```rust
pub struct DcOpConfig {
    pub tolerance: f64,             // 1e-9 V (ABSTOL of the step test)
    pub reltol: f64,                // 1e-6
    pub max_iterations: usize,      // 200
    pub source_steps: usize,        // 10
    pub gmin_start: f64,            // 1e-2
    pub gmin_end: f64,              // 1e-12
    pub gmin_steps: usize,          // 10
    pub max_rail_pin_rounds: usize, // 8
    pub rail: DcRail,               // LoadLine
}

pub struct DcOpResult {
    pub v_node: Vec<f64>,   // N-vector: node voltages
    pub v_nl: Vec<f64>,     // M-vector: controlling voltages (N_v · v)
    pub i_nl: Vec<f64>,     // M-vector: device currents at bias point
    pub converged: bool,
    pub method: DcOpMethod,
    pub iterations: usize,
    pub kcl_residual_max: f64,         // A, against the circuit's own G
    pub kcl_worst_row: Option<usize>,
    pub rail_pin: RailPin,
}

pub enum DcOpMethod {
    Linear, DirectNr, SourceStepping, GminStepping, AolStepping, Failed, SingularLinear,
}
```

### Entry Point

```rust
pub fn solve_dc_operating_point(
    mna: &MnaSystem,
    device_slots: &[DeviceSlot],
    config: &DcOpConfig,
) -> DcOpResult
```

## Integration

### Codegen Path

In `CircuitIR::from_kernel()`:
```rust
let dc_result = dc_op::solve_dc_operating_point(mna, &device_slots, &dc_op_config);
// dc_result.v_node → dc_operating_point constant
// dc_result.i_nl  → dc_nl_currents → DC_NL_I constant
```

Generated code gets a `DC_NL_I` constant that initializes `i_nl_prev`:
```rust
pub const DC_NL_I: [f64; M] = [1.234e-3, 5.678e-6];  // From DC OP solve

// In Default impl:
i_nl_prev: DC_NL_I,  // Start from bias point, not zeros

// In reset():
self.i_nl_prev = DC_NL_I;
```

### Runtime Path

**Removed.** The runtime `CircuitSolver`/`NodalSolver`/`initialize_dc_op()` API has
been deleted. All circuit processing now flows through the codegen pipeline above
(`CircuitIR::from_kernel` → emitted `DC_NL_I` constant). If you find references to
`CircuitSolver::initialize_dc_op` in older docs or examples, they are dead code.

## Trapezoidal Steady State

The generated solvers (DK and nodal) integrate in the charge form
(`COMPANION_MODELS.md`, "Charge (Companion) Form"): every source and the
nonlinear current enter at `n+1`, and the capacitor history is
`alpha*C*v_prev + q_dot`. At rest `q_dot = 0` and `A − A_neg = G` on every
row, so the per-sample fixed point is exactly the DC OP `G·v = RHS_CONST +
N_i·i_nl(v)` under both integrators.

**Warm-up**: The generated `warmup()` method runs after construction and `reset()`.
Two phases:

1. **Low-rate DC settling** (nodal path only, when `DC_OP_CONVERGED = false`):
   `rebuild_matrices(200.0)` → 1000 silent samples (5 seconds circuit time) →
   `rebuild_matrices(target_rate)`. This charges coupling caps (e.g. 22µF × 27K =
   0.6s RC) that the failed DC OP left uncharged. The DC steady state is rate-
   independent (`A - A_neg = G`, no rate terms), so values found at 200 Hz are
   exact at any target rate. The settled state is cached in `dc_operating_point`
   and `settled_i_nl` — the expensive phase runs only once; subsequent `reset()`
   calls reuse the cached values.

2. **Standard warmup**: 50 silent samples at the target rate. Settles the DK/nodal
   solver from any residual mismatch and pulls high-gain op-amp circuits into the
   physically correct basin of attraction.

Emitted from `codegen/rust_emitter/nodal_emitter/state.rs` (grep `warmup`). The DK path (`dk_emitter.rs`)
only emits the standard 50-sample warmup (DK circuits have well-conditioned DC OP).

## Expected DC OP Values (Verification)

### Single Diode + VCC
```
VCC=5V, R=1k, D1 to GND
V(anode) ≈ 0.65V
I_D ≈ (5 - 0.65) / 1k ≈ 4.35mA
```

### BJT Common Emitter (12V, 2N2222A-like)
```
VCC=12V, R1=100k, R2=22k (base divider), RC=6.8k, RE=1k
V(base) ≈ 2.16V  (divider: 12 * 22k / (100k + 22k))
V(emit) ≈ 1.51V  (V(base) - 0.65V)
I_C ≈ 1.51mA     (V(emit) / RE)
V(coll) ≈ 1.73V  (12 - 1.51e-3 * 6800)
```

## Common Failures

| Symptom | Cause | Fix |
|---------|-------|-----|
| NR diverges (oscillating v) | Wrong Jacobian sign (using + instead of -) | Use `G_aug = G_dc - N_i·J_dev·N_v` |
| Converges to wrong point | Linear initial guess too far | Source stepping will fix automatically |
| PNP BJT wrong polarity | Missing sign parameter | Check `is_pnp` flag in DeviceParams |
| BJT oscillation in transient | Mixed integration mismatch | Fixed: DK now uses trapezoidal for nonlinear currents |

## Runtime DC OP Recompute

The compile-time DC OP baked into the generated code (`DC_OP` / `DC_NL_I`
constants) is computed at **nominal** pot/switch values. A plugin that applies
per-instance jitter (e.g. tube stages with ±5% on Rk/Ra/Rcv) or recalls a
preset that moves a pot by 40% from its codegen default therefore starts
`CircuitState::default()` at the *wrong* fixed point. Without intervention
the solver has to silence-warm for `5 · τ_max` to drift into the jittered
equilibrium — seconds of silent output for circuits with large coupling caps.

The opt-in runtime recompute solves that equilibrium directly:

```rust
let mut state = CircuitState::default();
state.set_pot_0(jittered_rk);
state.set_pot_1(jittered_ra);
state.recompute_dc_op();   // ← jumps to the new fixed point
// plugin ready to process — no warmup loop needed
```

Enabled by the `--emit-dc-op-recompute` CLI flag (default OFF) or
`CodegenConfig::emit_dc_op_recompute = true` at the API level. **Not
audio-thread safe** — tens to hundreds of microseconds.

### What the method returns, and when to call it

The recompute returns the **infinite-time equilibrium** of the circuit at
the current controls and the current junction temperatures (`tj` is an
input to the solve and is left where it is; the point is at that `tj`, not
at thermal equilibrium). Its intended use is **instantiation, `reset()`,
preset recall and per-instance jitter** — situations where no audio is
running and the conventional target is the settled point. It is **not meant
for a live control change during playback**: called on a switch press it
would replace the circuit's own transition with the equilibrium the
hardware reaches only after that transition (a bias network's re-settle
after a switch press, a release network's 1–2 s tail, are hardware behaviour
to keep), and on a deck whose open switch is stood in for by a 1 GΩ
resistor it would jump a floating node to a point the hardware reaches
after hours. Nothing in the code enforces this; it is the intended usage
(operator decision, 2026-11-03).

### Method contract

| State field | After `recompute_dc_op()` / `set_dc_operating_point(v)` |
|---|---|
| `dc_operating_point` | The installed point: converged node voltages at current pot/switch values, or `v` |
| `dc_op_i_nl` | The device currents AT that point (converged, or evaluated at `v`). The two are written together: the live pair |
| `v_prev` | Same point — next sample starts at equilibrium |
| `i_nl_prev`, `i_nl_prev_prev` | `dc_op_i_nl` |
| `q_dot` (trapezoidal builds) | `[0.0; N]` — the new equilibrium is at rest (on an inductor or winding row this is `dΦ/dt = 0`; the current is a row of the point) |
| `input_prev` / `inputs_prev`, `injections_prev` | `0.0` (new equilibrium assumes zero input history) |
| `dc_block_x_prev[k]` | `v_node[OUTPUT_NODES[k]]` — seeds IIR at steady state |
| `dc_block_y_prev` | `[0.0; NUM_OUTPUTS]` — DC blocker's fixed point |
| `pot_N_resistance_prev` | `pot_N_resistance` (kept in step with the current value) |
| `os_up_state`, `os_dn_state` (+ `_outer`, the `.inject` up-filters) | Zeroed — stale DC trajectory discarded |
| `last_nr_iterations` | The 0-indexed converging iteration (0 for `set_dc_operating_point`) |
| nodal: `chord_valid`, `chord_be_valid` | `false` — the factorisations were taken at the old point |
| nodal: BE-latch estimator fields, `be_latched`, `breakpoint_be` | Cleared, as `reset()` clears them |
| `noise_rng_state` | **Preserved** — resetting would repeat the same noise after every param change |
| `pot_N_resistance`, `switch_N_position` | **Preserved** — they're the INPUT to this solve |
| `device_N_*` runtime params, `device_N_tj` | **Preserved** (a body-effect MOSFET's `device_N_vt` holds the threshold at the installed point) |
| `diag_*` counters | **Preserved** (caller may be watching) |
| the factory point `reset()` returns to | **Preserved** (`DC_OP`/`DC_NL_I`, or the low-rate warmup's settled cache `settled_v`/`settled_i_nl` on a build whose compile-time solve did not converge) |

Both routes emit the writeback as one private method, `dc_op_writeback`,
shared by `recompute_dc_op` and `set_dc_operating_point`.

### The live pair and the factory pair

`dc_operating_point` is the **live** point and `dc_op_i_nl` the device
currents at it; the two are always written together. The NaN/magnitude
recovery (`process_sample`, Step 7 on DK, `emit_nodal_nan_reset` on nodal)
restarts from that pair, so after a recompute at moved pots it restarts
from a point that satisfies KCL (the live point with the baked `DC_NL_I`
would not). `reset()` restores the **factory** pair — `DC_OP` with `DC_NL_I`,
or on a nodal build whose compile-time solve did not converge the pair the
low-rate warmup settled and cached (`settled_v`, `settled_i_nl`) — together
with the nominal pot/switch values, so the three agree. A recomputed pair is
discarded as a pair. `dc_op()` returns the live point.

### `set_dc_operating_point(v_dc) -> (f64, usize)`

Emitted on every build (both routes, with or without the recompute flag):
the same writeback with the device currents from one evaluation at `v_dc`,
and it **returns the KCL residual of `v_dc` against the live circuit** — the
largest `|F_i|` over the circuit-node rows in amperes, and that row — from
one matrix-vector product with the live `G` (`F = G·v − b_dc − N_i·i_nl`).
An equilibrium reads ~1e-9 A or less. The vector is installed either way: the
method reports a non-equilibrium vector, it never refuses one. A caller that
hands over node voltages only, with the source and winding branch rows left
at zero, sees the winding's DC current as the residual of the node it feeds
(a 10 mA primary reads 10 mA at its load node; the transformer witness in
`dc_op_recompute_nodal_tests.rs`).

### DC fixed-point algebra

The transient NR step at a converged sample is (charge form, both integrators)
`A · v_{n+1} − N_i · i_nl(v_{n+1}) = RHS_CONST + A_neg · v_n + q_dot_n + input(n+1)`,
with `A_neg = alpha·C` (`q_dot` only in trapezoidal builds). Substituting
steady state (`v_{n+1} = v_n = v_dc`, `input = 0`, `q_dot = 0`) and using
`A − A_neg = G` on every row gives the DC fixed point:

```
G · v_dc = RHS_CONST + N_i · i_nl_dc
```

The runtime solver therefore uses `b_dc = RHS_CONST` verbatim (DC sources
are ×1 on every row; VS rows carry `V_dc`), adds `.runtime` voltage-source
fields, then Newton-iterates
`G_aug_nr = g_aug − N_i · J_dev · N_v`,
`rhs_nr = b_dc + N_i · (i_nl − J_dev · v_nl)`,
`v_new = G_aug_nr⁻¹ · rhs_nr`. `g_aug` is the live `G`: on DK the constant
`G` plus the pot and switch-resistor deltas, on nodal the working copy the
setters stamp (`g_work`, carrying the baked 1e-12 S node Gmin like the
transient; the rate-dependent matrices are rebuilt first if a setter left
them dirty). Every row of the augmented system is an unknown: node voltages,
source and constraint rows, inductor and transformer winding currents (their
rows are the DC short `v_a − v_b = 0`, the same short `dc_op.rs` stamps).

### Certification (both routes)

An iterate is accepted when two tests pass, both at the point written back:

1. the **step test on the UNDAMPED Newton correction** `v_new − v`, per
   variable over every row, `|Δ| < 1e-6·max(|v|, |v_new|) + 1e-9 V` — Newton's
   own estimate of the distance to the root, read before any limiter touches
   the step (the nodal transient's rule, at this solver's tolerances; the
   applied, damped step's size is not evidence: the transient measured points
   certified that way 1.7 to 810 thresholds from Newton's target);
2. the **KCL residual gate** at the point a sub-threshold correction reached,
   with the devices freshly evaluated there: over the circuit-node rows (and
   a parasitic BJT's internal nodes on nodal), `|F_i| <= 1e-6·scale_i + 1e-9 A`,
   `scale_i = max(|b_i|, |(N_i·i_nl)_i|, max_j |G_ij·v_j|)`, NaN-honest.

The step test alone is satisfied at any fixed point of the limiter, root or
not; the gate alone is vacuous on a weak mode (a node tied to the rest of
the circuit by nanosiemens passes a 1e-9 A gate anywhere within a fraction
of a volt). Loop shape: evaluate the devices at `v_k`; if the previous
correction passed, gate `v_k`; else stamp, solve, test the undamped
correction, apply the limited step. The accepting evaluation is the next
iteration's, so certification costs no extra device evaluation.

Limiting: DK applies its flat node damping (`DAMP_THRESHOLD` 10 V, ±50 V
clamp, voltage rows only). Nodal applies the transient's limiter: pnjlim /
fetlim on each proposed device voltage shrink one ray `alpha` (Newton's
direction is kept, never a per-junction back-projection — two devices that
share a node cannot be summed into a reversed step), then the node-step
damping at `max(10 V, 5 % of the largest node voltage)`.

A body-effect MOSFET's threshold is refreshed at every iterate through its
`device_N_vt` field (the only channel into the shared device evaluator); a
failed recompute puts the pre-solve values back, so failure leaves the state
untouched.

### Railed op-amps

DK: a clamped op-amp runs hard, so the recompute pins a railed output at its
zero-load limit at the terminal — an active set (pin every output whose
linear model `AOL·(v+ − v−)` passes its limit, re-solve with those rows
fixed, repeat until the set is unchanged, at most 8 rounds; a held pin stays
only while its own test holds and otherwise releases, never moving to the
other rail in one round, the rule of the compile-time active set). A set
that does not settle is a failed recompute (`diag_nr_max_iter_count`), so
`settle_dc_op` falls back to the warmup loop. Without the pin the recompute
solved the linear model: a comparator railed at rest landed at 8989 V on a
15 V supply
(`opamp_saturated_sag_tests::the_runtime_recompute_keeps_a_railed_output_on_its_rail`).

Nodal: a railed output is never solved through its linear model, and the
transient's active set is not carried by the runtime solve yet, so a nodal
deck whose rail mode clamps a finite rail **refuses** (below). BoyleDiodes
rails are catch diodes the solve sees as devices; `none` applies no rail, so
the linear model is the transient's own fixed point.

### Refusals (nodal)

Three classes of nodal build get no solve: `recompute_dc_op` advances
`diag_nr_max_iter_count` and returns with the state untouched, so
`settle_dc_op` falls back to the warmup loop. Each is a commented early
return in the generated method, and its doc comment says `This build:
solved.` or `This build: REFUSED (…)` (`nodal_emitter/dc_op.rs`,
`recompute_refusals`).

- **Behavioral B-source decks**, classed on `idt()` / `time`: an integrator
  whose integrand is nonzero at rest advances every sample, so such a deck
  has no DC operating point and no warmup settles it either. Static and
  ddt-only expressions do have a fixed point (ddt is exactly 0 there), which
  beats the baked `DC_OP` (it drops B sources entirely), but no plugin deck
  needs that branch; building it means seeding `bsrc_x_prev` at the
  converged inner values (or the first sample sees ddt as an impulse) and
  landing the same inclusion in `dc_op.rs` so the two points agree. Until
  then every B-source deck refuses; a later inclusion widens the class, it
  does not reverse it.
- **Clampable op-amps** (a finite VCC/VEE under the hard or an active-set
  rail mode): see above. 4kbuscomp and gravity are the witnesses for the
  active-set landing; 4kbuscomp's sidechain-rectifier op-amps are also a
  basin question the compile-time solver scopes out (`seed_sr_feedback_diodes`).
- **Latched (glow) devices**: the lit/unlit latch is part of the equations;
  such a deck can have two DC roots or none (a relaxation divider).

### Measured (2026-11-03)

Nodal fixtures (a BJT stage with a diode clamp, a transformer with a 10 mA
primary, a JFET stage, a switched and a pot diode bias network), `v_prev`
perturbed ×1.1 and +1 V on the node rows: every one recomputes onto the baked
`DC_OP` within 1e-9 V (KCL within 1e-9 A) in 2–23 iterations, and the first
sample afterwards converges at iteration 0 with no hold and no sub-step.
After `set_switch_0(1)` / `set_pot_0(2200)` the recompute equals the `DC_OP`
of the deck built at that position / value
(`dc_op_recompute_nodal_tests.rs`). The 11 DK golden-manifest decks compiled
with the flag land on the same point under the two-test certification as
under the old step-only test; none refuses.

### MVP limitations

- **Direct NR only** — no source / Gmin stepping. The warm-start from
  `v_prev` (a physically valid prior equilibrium) makes this sufficient
  for small-to-moderate jitter. On failure the method bumps
  `diag_nr_max_iter_count` and returns without updating state, so the
  caller can fall back to the `WARMUP_SAMPLES_RECOMMENDED` silence loop.
- **No basin-trap handling** — precision-rectifier topologies that rely
  on the compile-time solver's `seed_sr_feedback_diodes` refinement
  stay on the warmup loop.
- **No parasitic-BJT internal-node expansion on the DK path** (those
  nodes are ill-conditioning risks outside the MVP). On nodal the internal
  nodes are real MNA rows and are solved.
- **Inductors need no special case.** Inductors, coupled inductors and
  transformer windings are augmented branch rows whose `L` sits in `C`; in
  `G` each row is the short `v_a − v_b = 0`, the same DC short `dc_op.rs`
  stamps. The branch currents are entries of `v_node`, written back with
  it; in trapezoidal builds the zeroed `q_dot` sets their `dΦ/dt` to zero. A
  DC-carrying choke recomputes to the baked `DC_OP`
  (`dc_op_recompute_tests.rs::fix3_dc_biased_choke_shipped_recompute_reaches_inductor_short_op`).
  A saturating inductor's flux is `Φ(i)` of that current, recomputed from
  `v_prev` every sample (`emit_sat_ind_history`), so it carries no separate
  state to seed.
- **The warmup fallback is not the same answer reached more slowly on every
  deck.** On a deck whose open switch is a 1 GΩ stand-in, a floating node's
  time constant can be hours; the recompute lands on that node's equilibrium
  while `WARMUP_SAMPLES_RECOMMENDED` samples (`5·τ_max`, capped) do not get
  there. The two differ on such states by design.

### Real-time safety

`recompute_dc_op` runs a full NR loop (LU factorization + back-solve +
device evaluation, potentially hundreds of iterations). Expected cost
is tens to hundreds of microseconds — hundreds of times an audio
sample period. Call it from plugin initialization or
parameter-change callbacks, never from `process_sample`.

### `settle_dc_op()` — convenience wrapper

Also emitted behind the same flag. Standard "recompute, fall back to
warmup on failure" wrapper that plugin hosts would otherwise have to
write themselves:

```rust
pub fn settle_dc_op(&mut self) {
    let before = self.diag_nr_max_iter_count;
    self.recompute_dc_op();
    if self.diag_nr_max_iter_count > before {
        for _ in 0..WARMUP_SAMPLES_RECOMMENDED {
            let _ = process_sample(0.0, self);
        }
    }
}
```

Identical body on DK and nodal paths — the path-specific behavior
lives entirely in `recompute_dc_op`. A successful solve makes
`settle_dc_op` equivalent to a bare `recompute_dc_op` call; a refused build
or a failed solve ticks the counter and the wrapper runs the warmup loop.
Plugin code calling `settle_dc_op` doesn't need to branch on the circuit's
routing — the wrapper handles it.

The counter check uses `diag_nr_max_iter_count` exclusively because
damping and substep counters can increment during *successful*
convergence. The recompute increments this specific counter on
iter-exhaustion, singular LU, NaN resets, an unsettled rail pin set and
every refusal — all "state was not updated" outcomes — so this single
field cleanly distinguishes "ready to process" from "fall back to warmup".
