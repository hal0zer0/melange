# Behavioral (`B`) Sources — Arbitrary-Expression Sources

SPICE3 `B`-element support: a source whose output is an arbitrary scalar
expression over node voltages, branch currents, and time. Originally requested
by oomox (Subspace radio plugin) to make FM capture/threshold/click behavior
**emerge** from a limiter + discriminator instead of being faked in DSP.

> Status (2026-06-14): **COMPLETE. `I={}`, `V={}`, algebraic + `ddt`/`idt`/`time`
> + named params, AND the FM `ddt(atan2)` discriminator all work end-to-end.**
> The discriminator recovers instantaneous frequency (`ddt(atan2(Q,I)) = 2π·f`
> between wraps) and is stable; static phase → ~0 output. AM and FM virtual
> antennas both codegen + run. No guarded behavioral features remain.
>
> **`ddt`-discriminator fix (2026-06-14).** The discriminator had been unstable;
> two mechanisms, both fixed:
> 1. **Lagged Jacobian** ([`Expr::diff_jacobian`]) — `ddt`'s NR-Jacobian partial
>    `inv_dt·∂x/∂v` is `~SR×` the rest of the matrix and ill-conditions the LU.
>    `diff_jacobian` treats `ddt` as a constant (`∂ddt/∂v = 0`) for the Jacobian
>    only; the residual/companion still uses the full `ddt` value, so a
>    feedforward discriminator converges to `V = ddt(...)` exactly (the Jacobian
>    steers the iteration, not the fixed point). `idt` keeps its `half_dt`
>    scaling (tiny, never stiff). The emitter uses `diff_jacobian` for the
>    behavioral `g_k` partials.
> 2. **Damping exclusion** — behavioral `V={}` output nodes are algebraically
>    forced (`V=f`), so the global node-damping must skip them; otherwise the
>    legit `ddt` startup spike (phase 0→θ in one sample) drives the 5 %/iter
>    damping, which can't drain it within `MAX_ITER` → the main NR hits max-iter
>    and falls to the **BE fallback (no behavioral stamps)** → the limiter nodes
>    collapse to 0 (period-2 osc). `behavioral_damp_skip_literal` excludes them.
> Tests: `fm_discriminator_static_phase_is_zero`,
> `fm_discriminator_recovers_frequency`, `expr::tests::diff_jacobian_lags_ddt`.
>
> ---
> Earlier status (superseded): **`I={}` and `V={}` both work (algebraic +
> `ddt`/`idt`/`time` + named params). The AM virtual-antenna front-end codegens.
> Only `ddt`-discriminator stiffness remains (FM-specific).**
>
> **Named params** (the AM blocker) are wired:
> - `.param name = value` (spaces around `=` tolerated) → baked constant.
> - `.runtime <name> <min> <max> as <field>` → plugin-driven scalar (dispatched
>   when the `.runtime` target starts with neither V nor R — SPICE components
>   must, so a non-R/V target is unambiguously a scalar). Emits `pub <field>: f64`
>   on `CircuitState`, a clamping `set_runtime_<field>(f64)` setter, default =
>   `min`. Referenced by bare identifier in expressions.
> - Bare identifier in an expression resolves to a `.param` const, then a scalar
>   `.runtime`; an unknown identifier is a hard codegen error (not silent).
> The `B_theta`/`B_I`/`B_Q`/`B_env` AM antenna (`idt`, `cos`/`sin`, `strength`,
> `m`, `f_offset`, `sqrt`) compiles. Test:
> `named_params_resolve_const_and_scalar_runtime`.
>
> ---
> Earlier status (superseded): **`I={}` and `V={}` both work end-to-end (algebraic
> + `ddt`/`idt`/`time`); named params are the last guarded feature.** `V={}`
> (augmented constraint row) validated: `V(out)=tanh(0.6)` directly, and the FM
> limiter+phase chain (`sqrt`, `atan2`, chained `V={}` sources reading each
> other) normalizes and detects phase correctly. `atan2` derivative is
> regularized at the origin (`x²+y²+1e-30`) — required, else the discriminator
> NaNs at startup when limiter outputs are 0.
>
> **KNOWN ISSUE — `ddt` discriminator stiffness.** `V={ ddt(atan2(...)) }` (the
> full FM instantaneous-frequency discriminator) is numerically unstable in the
> monolithic NR: `∂ddt/∂v = inv_dt·∂(inner)/∂v` is `SR×` larger (~38400 at 48 kHz)
> than the rest of the matrix, ill-conditioning the coupled LU (the limiter nodes
> collapse to 0, period-2 oscillation). The static phase detector (`atan2` without
> `ddt`) is fine. **Fix direction**: treat `ddt`'s current-sample dependence as a
> one-sample lag for the Jacobian only (a `.delay_feedback`-style chord — the
> discriminator is feedforward, so a separate "NR-Jacobian diff that zeros `ddt`"
> converges in ~2 iters and removes the stiff coupling; the output lags one
> 20 µs sample, inaudible). Not yet implemented. The original spec's
> "one-sample-difference idiom" is essentially this.
>
> **I-form idiom (melange-circuits agent, 2026-06-13)**: any `V={f}` driving an
> otherwise-unloaded node equals `I={f}` into that node + a series-R to ground
> (`V=I·R`), sign-invariant for the radio. So the antenna *could* be built on
> `I={}` alone — but `V={}` now works, so this is optional. Recorded in
> `~/dev/melange-circuits/local-docs/radio-bsource-iform-idiom-2026-06-13.md`,
> which also flags that `idt` phase accumulators grow unbounded off-station
> (no `wrap`/`fmod` in the function set yet — a future addition).
>
> ---
> Earlier status line (superseded): **`I={}` (algebraic + `ddt`/`idt`/`time`) works end-to-end
> (generate → compile → run → matches oracle); `V={}` and named params still
> guarded.** `ddt(time)=1` and `idt(const)=ramp` are validated. Companion state
> (`bsrc_x_prev`/`bsrc_int_prev`/`sim_time`/`bsrc_inv_dt`/`bsrc_half_dt`) is
> emitted on `CircuitState`, updated post-convergence at the converged `v`, and
> recomputed in `set_sample_rate`. The `ddt`/`idt` companion is independent of
> the circuit's BE-vs-trap integration (it's the source's own backward-diff /
> trap accumulator).
> The expression engine (symbolic autodiff, params), parsing, MNA representation,
> IR plumbing, routing, and the nodal node-space stamping for **algebraic current
> sources** are implemented and tested. A `B I={V(a)*V(b)}` multiplier and
> `B I={tanh(V(a))}` clipper generate code matching the analytic oracle to 1e-5
> (`crates/melange-solver/tests/behavioral_source_tests.rs`). Still guarded
> (`behavioral_emitter_supported` → `UnsupportedTopology`): `V={}` (augmented
> constraint row), `ddt`/`idt`/`time`, named parameters, branch currents. The
> remaining work is in [§Codegen integration plan](#codegen-integration-plan).
>
> **What's wired in the emitter (algebraic `I={}`):** behavioral circuits force
> `SolverRoute::Nodal` + `use_full_nodal` (the full-LU path) and **backward
> Euler** (`be = … || behavioral`), because the stamp is current-only and BE's
> steady state is exactly `G·v = i` (trapezoidal would need an `i_prev` history
> term — a future refinement; under trap the result is half-right). Per NR
> iteration: `emit_behavioral_evals` (value + `∂f/∂V` partials at the iterate) →
> `emit_behavioral_jacobian` stamps `∂f/∂V(k)` into `chord_lu` (full refactor
> each iteration, since behavioral Jacobian isn't a frozen device block) →
> `emit_behavioral_rhs` stamps the companion `f − Σ ∂f/∂V·v`. `M=0` behavioral
> circuits take the NR path (not the linear shortcut) and their nodes join the
> convergence set. Sign: current `f` flows n+→n-, drawing `f` from n+ ⇒
> `V(out) = −f` for a source into a 1Ω-to-ground load. **Gotcha proven in test:**
> the default output DC-blocker high-passes a DC behavioral output to zero — use
> `dc_block:false` (or an AC signal) when validating DC oracles.
>
> **Two oomox requests, one feature.** The original FM-only request
> (`melange-behavioral-bsource-request.md`) was superseded by
> `melange-virtual-antenna-request.md` (complex-baseband I/Q for **both** AM
> envelope `sqrt(I²+Q²)` and FM discriminator). Both are served by this same `B`
> element. The virtual-antenna netlists reference `.pot`/`.runtime`/`.param`
> scalars (`strength`, `m`, `f_offset`, `k`) as bare identifiers and lean on
> `idt` for the carrier phase `theta = idt(2·pi·f_offset)` — both supported.

## Surface

```
B<name> n+ n- V={<expr>}     ; behavioral voltage source  (V(n+) - V(n-) = expr)
B<name> n+ n- I={<expr>}     ; behavioral current source  (expr amps, n+ → n-)
```

`<expr>` is a scalar expression over:

- **Node voltages** `V(node)` and differences `V(a,b)` (= `V(a) - V(b)`).
- **Branch currents** `I(elem)`.
- **Named scalar parameters** — any bare identifier that isn't `time`/`pi` is a
  parameter reference (`.pot` / `.runtime` / `.param`), e.g. `strength`,
  `f_offset`. Resolved and validated at codegen; treated as a constant by NR
  differentiation (the solver differentiates only w.r.t. node voltages / branch
  currents).
- **`time`** — simulation time in seconds. **`pi`** — the constant π.
- **Operators** `+ - * / ^` (`**` is an alias for `^`), unary `-`, parentheses.
  Precedence (loosest → tightest): `+ -` < `* /` < unary `-` < `^`, so
  `-x^2 = -(x^2)` (ngspice/standard-math convention — fixed 2026-07-18; it
  previously parsed as `(-x)^2`). `^` is right-associative — see the
  compatibility notes below.
- **Functions** `atan2(y,x) sqrt abs exp ln(=log) sin cos tanh min max pow
  pwr`. `pwr(x,y) = sign(x)·|x|^y` (ngspice `PTpwr`) — the odd-symmetric,
  audio-friendly power: preserves signal polarity even for even/fractional
  exponents, e.g. `pwr(V(a), 1.5)` as a smooth polarity-keeping waveshaper.
- **`ddt(x)` / `idt(x)`** — time derivative / integral of a sub-expression.

The `V`/`I` keyword may be written `V={..}`, `V ={..}`, or `V = {..}`.

### ngspice compatibility notes

- **`^` associativity DIVERGES from ngspice.** melange's `^` is
  right-associative (`2^3^2 = 2^(3^2) = 512`, the standard math convention);
  ngspice declares `%left '^'` (`inpptree-parser.y`), so there `2^3^2 =
  (2^3)^2 = 64`. Parenthesize chained powers in any netlist that must
  cross-validate against ngspice.
- **Unary-minus precedence MATCHES ngspice** (since 2026-07-18): `%left NEG`
  is declared before `%left '^'` in ngspice, so `^` binds tighter —
  `-x^2 = -(x^2)`. Write `(-x)^2` to square a negation.
- **Numeric-literal suffixes are NOT accepted inside `{}`** — `1k`, `1meg`,
  `100n` etc. parse as a number followed by an identifier (→ unknown-param
  error). Use plain or scientific notation (`1000`, `1e3`, `100e-9`) inside
  expressions; suffixes remain fine on component-value fields outside `{}`.
- **`pow` follows ngspice `PTpower`** (`ptfuncs.c`): a negative base with a
  (near-)integer exponent snaps to the sign-correct integer power
  (`pow(-2, 3) = -8`); a negative base with a fractional exponent evaluates
  `|x|^y` (`pow(-0.5, 1.5) = +0.3536`) — never NaN. The near-integer
  predicate is `|y - round(y)| < 1e-6·max(1, |y|)` (ngspice's own test has a
  quirky `y + 0.001` denominator; same intent). The symbolic derivative
  follows whichever branch the value takes.
- **Division `/` has ngspice value semantics — no melange-side guard.** A
  zero denominator yields ±inf in the VALUE (ngspice nudges by
  `PTfudge_factor = gmin·1e-20`, effectively the same hazard). Only the
  emitted *derivative* denominator is guarded (`b² + 1e-300`) so the NR
  Jacobian cannot produce 0/0 = NaN. If a denominator can cross zero, guard
  it at the netlist level — the limiter idiom
  `V(a) / sqrt(V(b)*V(b) + 1e-9)` or, for a known-positive denominator,
  `V(a) / (V(b) + 1e-9)`.

### Derivative guards (Jacobian NaN-safety)

Every intrinsic's symbolic derivative is paired with its (clamped) value so
the NR Jacobian is NaN-free at domain boundaries. The guards live in the AST
itself, so the interpreter (`Expr::eval`) and the emitted Rust agree by
construction:

| Function | Value | Derivative at the boundary |
|----------|-------|---------------------------|
| `abs(x)` | `x.abs()` | branchless sign `x/(|x|+1e-300)` — exactly **0 at x = 0** (subgradient) |
| `min/max` | native | tie (`a = b`) → **0.5/0.5 split** `(da+db)/2` via the same sign guard |
| `sqrt(x)` | `max(0).sqrt()` (flat for x < 0) | `0.5/(sqrt+1e-150)` — huge-but-finite cliff (5e149) at/below 0, never +inf |
| `ln(x)` | `max(1e-300).ln()` (pinned below 0) | `1/max(x, 1e-300)` — ≤ 1e300, finite everywhere |
| `exp(x)` | `clamp(-40, 40).exp()`, emitted inline | paired gate: `exp(x)` inside the window, **exactly 0** in the clamped-flat region |
| `pow(a,b)` | PTpower (above) | branch-matched; `a/(a²+1e-300)` guards the `b·da/a` term and `ln|a|` the exponent term |
| `a/b` | unguarded (ngspice parity) | denominator `b²+1e-300` (0/0 → 0, not NaN) |

Boundary coverage: `expr::tests::derivative_domain_boundaries_never_nan`
(unit, central-difference harness) and the compile-and-run tests
`behavioral_abs_at_zero_converges_from_zero_start` /
`behavioral_exp_source_compiles_and_matches_oracle` /
`behavioral_pow_negative_base_matches_oracle` in
`tests/behavioral_source_tests.rs`.

### `ddt` / `idt` semantics

For a sub-expression value `x` at sample `n`, with `dt = 1/internal_rate`:

```
ddt(x) = (x[n] - x[n-1]) / dt                    ; backward difference
idt(x) = idt[n-1] + (dt/2)·(x[n] + x[n-1])       ; trapezoidal accumulator
```

**Why backward difference, not a smooth trapezoidal companion?** The FM
discriminator is `ddt(atan2(Q, I))`. The "click" behaviour (and the FM cliff)
happens precisely *because* `atan2` wraps at ±π: when the noise vector overtakes
the carrier the phase snaps 2π and the difference produces an impulse. A
backward difference preserves that impulse exactly — it **is** the click. A
smooth companion would need phase unwrapping, which would erase the very
behaviour the circuit is meant to produce. So the simplest correct option is
also the desired one.

Partial derivatives (for the NR Jacobian) are
`∂ddt(x)/∂V = (1/dt)·∂x/∂V` and `∂idt(x)/∂V = (dt/2)·∂x/∂V`. Differentiation
introduces the `Expr::InvDt` / `Expr::HalfDt` scaling leaves to carry these.

## Why behavioral sources are nodal-only

The DK Schur reduction (`K = N_v·S·N_i`) and the block-diagonal device
machinery both assume **each nonlinear dimension is controlled by a single
node-pair voltage** (`N_v` row = `+1/-1`). A behavioral expression references
*arbitrary* nodes (`V(iq_i)`, `V(iq_q)`, …) → a **rectangular** map (many
controlling nodes → one output), which does not fit `N_v`/`N_i`. The natural
home is the **nodal full-LU** path, where the full node-voltage vector `v` is
available each NR iteration and contributions stamp directly by node index
(standard SPICE behavioral stamping). Therefore any `B` source forces
`SolverMode::Nodal`.

## Implemented: expression engine (`crates/melange-solver/src/expr.rs`)

Pure, self-contained, fully unit-tested (`cargo test -p melange-solver expr::`):

- `Expr::parse(&str) -> Result<Expr, ExprError>` — tokenizer + recursive-descent
  parser. `V()`/`I()` take node/element *names* (bare idents or integer node
  numbers), not sub-expressions.
- `Expr::assign_state_slots(&mut usize)` — assigns globally-unique companion
  slot ids to each `ddt`/`idt` (call once across all sources via a shared
  counter; the MNA builder does this in `categorize_element`).
- `Expr::diff(&Var) -> Expr` — symbolic partial derivative. `Var` is a
  `Node(name)` or `Branch(name)`. `min`/`max` differentiate via the
  `(a±b∓|a−b|)/2` identity with the branchless subgradient sign
  (`x/(|x|+1e-300)`) — 0.5/0.5 split at a tie. `pow` derivatives are matched
  to the PTpower value branch (integer power rule / `|a|^p` rule / `ln|a|`
  for variable exponents) — see "Derivative guards" above. Verified against
  central differences in `expr::tests::diff_matches_numeric` and at domain
  boundaries in `expr::tests::derivative_domain_boundaries_never_nan`.
- `Expr::simplify()` — constant folding + identity elimination (keeps the
  emitted Jacobian compact; drops `0.0 * …` chains).
- `Expr::eval(&dyn EvalCtx) -> f64` — the interpreter, used by the DC OP solver
  and as the test oracle for the emitted straight-line code.
- `Expr::to_rust(&dyn ExprResolver) -> String` — straight-line Rust emitter
  (no AST interpreter in the audio thread). **Self-contained**: all guards are
  emitted inline and match the interpreter — `exp`→`.clamp(-40.0, 40.0).exp()`
  (an earlier version called a `bsrc_safe_exp` helper that no emitter defined,
  so `exp()` B-sources failed rustc), `sqrt`→`.max(0.0).sqrt()`,
  `ln`→`.max(1e-300).ln()`, `pow`→PTpower inline (powi for integer constant
  exponents), `atan2` native (guard `I²+Q²` at the netlist level with `+1e-9`
  as the request does). Derivative-side guards: see "Derivative guards" above.
- Helpers: `referenced_nodes`, `referenced_node_refs` (borrowed, for MNA node
  collection), `referenced_branches`, `variables`, `is_time_dependent`,
  `state_slot_count`, `remap_idents` (subcircuit expansion).

## Implemented: parsing + MNA representation

- **Parser** (`parser.rs`): `Element::BSource { name, n_plus, n_minus, kind,
  expr }` with `BSourceKind::{Voltage, Current}`. Dispatched on `'B'` from the
  **raw line** (the braced body contains spaces the whitespace splitter would
  shred); brace-matched expression capture. Node-reference validation checks
  the terminals *and* every `V()` node in the expression. Subcircuit expansion
  remaps expression identifiers via `Expr::remap_idents`.
- **MNA** (`mna.rs`): `BehavioralSourceInfo { name, kind, n_plus/n_minus(+idx),
  referenced_node_indices: BTreeMap<name,idx>, expr (slots assigned),
  v_ext_idx: Option<usize> }`, stored on `MnaSystem.behavioral_sources`.
  `categorize_element` resolves node indices, assigns ddt/idt slots from the
  builder's running counter, and assigns `v_ext_idx` (0-based index among
  `V={}` sources) for augmented-row allocation. `collect_nodes` registers
  expression-referenced nodes so they exist even if no other element touches
  them.

Tests: `mna::tests::bsource_parses_v_and_i_forms`,
`bsource_ddt_idt_slots_are_globally_unique`,
`radio_iq_discriminator_netlist_parses_and_builds_mna`.

## Codegen integration plan

The remaining work, all behind `!mna.behavioral_sources.is_empty()` so circuits
without a `B` source are **byte-identical** to today's output (acceptance #4).

### 1. IR (`codegen/ir/mod.rs`) — DONE
- `CircuitIR.behavioral_sources: Vec<BehavioralSourceIR>` carries
  `{name, is_voltage, n_plus_idx, n_minus_idx, referenced_node_indices, expr,
  aug_row, time_dependent}`. Populated in **both** `from_mna` paths via
  `build_behavioral_sources_ir(mna)`. `Expr`/`UnaryFn`/`BinFn` derive serde so
  the field round-trips. `aug_row` is `None` until the emitter allocates the
  `V={}` branch-current rows.
- **Remaining**: force `SolverMode::Nodal` when `behavioral_sources` is
  non-empty, and remove the `CodeGenerator::generate` guard at that point.

### 2. Augmented rows for `V={}` (`mna.rs` build)
- Add `num_behavioral_v = behavioral_sources.iter().filter(|b| b.v_ext_idx.is_some()).count()`
  to the `n_aug` computation. Each `V={}` source gets a branch-current row at
  `n_base + (existing aug count) + v_ext_idx`. Stamp the **linear** part of the
  constraint into `G` (`B^T`: branch current couples into `n+`/`n-` KCL with
  `±1`; constraint row carries `+1/-1` on `n+`/`n-`). The **nonlinear** `f(v)`
  part lives in the NR residual, not `G`.

### 3. State (`templates/rust/state.rs.tera`, nodal emitter)
- `bsrc_x_prev: [f64; N_BSRC_SLOTS]` and `bsrc_int_prev: [f64; N_BSRC_SLOTS]`
  (zero-init; reset in `reset()`).
- `sim_time: f64` (advance by `dt` each `process_sample_inner`).
- Consts `INV_DT = internal_rate`, `HALF_DT = 0.5/internal_rate` (recompute in
  `set_sample_rate`).
- ~~An emitted `bsrc_safe_exp(x)` free function (clamp `[-40,40]`).~~ Not
  needed: `Expr::to_rust` emits the exp clamp (and all other guards) inline —
  the generated expression code is self-contained.
- `ExprResolver::param(name)` must map bare identifiers to the right surface:
  `.pot`/`.runtime R` → the pot resistance/value field, `.runtime V` → its
  field, `.param` → a baked constant. Reuse the name tables the named-constant /
  pot emission already builds.

### 4. Nodal NR stamping (`nodal_emitter.rs::emit_nodal_process_sample`)
Inside the trapezoidal NR loop, alongside the device evaluation:
- **`I={}`** — evaluate `i = expr.to_rust(resolver@v)`; residual:
  `rhs_work[n+] -= i; rhs_work[n-] += i` (companion form: subtract the frozen
  `J·v` linearization to match the LU factor, mirroring the device `i_comp` at
  ~`nodal_emitter.rs:5319`). Jacobian: for each referenced node `k`,
  `∂i/∂V(k) = expr.diff(Node(k)).simplify().to_rust(resolver@v)`, stamped into
  `chord_lu[n+][k] -= ∂i/∂V(k); chord_lu[n-][k] += ∂i/∂V(k)` in the
  `need_refactor` block (~`nodal_emitter.rs:5260`). **Behavioral Jacobian
  entries are not frozen device blocks**, so either (a) force a refactor every
  iteration when behavioral sources are present, or (b) treat them with the
  same chord-companion correction as devices. Start with (a) for correctness;
  optimize later.
- **`V={}`** — augmented row `r`: residual `F[r] = (v[n+] - v[n-]) - expr(v)`;
  Jacobian `chord_lu[r][n+] += 1; chord_lu[r][n-] -= 1; chord_lu[r][k] -=
  ∂expr/∂V(k)` for each referenced `k`; branch current couples back with
  `chord_lu[n+][r] += 1; chord_lu[n-][r] -= 1`.
- The `resolver@v` maps `V(node)` → `v[idx]` (ground → `"0.0"`), `I(elem)` →
  the element's branch-current unknown, `time` → `state.sim_time`, `ddt`/`idt`
  prev → `state.bsrc_x_prev[slot]` / `state.bsrc_int_prev[slot]`.

### 5. Post-convergence state update (end of `process_sample_inner`)
After `v` converges, for each `ddt`/`idt` slot evaluate its **inner** expression
at the converged `v` and write `bsrc_x_prev[slot]`; for `idt`, also advance
`bsrc_int_prev[slot] += HALF_DT·(x + x_prev)`. Advance `sim_time += dt`.

### 6. DC OP (`dc_op.rs`)
Evaluate behavioral contributions via `Expr::eval` with an `EvalCtx` whose
`inv_dt() == 0.0` (freezes `ddt` derivatives and value → 0 at DC) and `idt`
held at its history (0 at startup). Keep the `atan2` `+1e-9` guard live so DC OP
doesn't stall with I/Q both at 0. Stamp `I={}` like a nonlinear current source,
`V={}` as the augmented constraint, into the existing DC-OP node-space NR.

### 7. Acceptance tests (planned — `tests/behavioral_source_tests.rs`)
1. Generated `B I={V(a)*V(b)}` multiplier and `B V={tanh(V(a))}` clipper match
   `Expr::eval` / a hand-stamped equivalent within solver tolerance.
2. The I/Q limiter+discriminator netlist (see
   `mna::tests::radio_iq_discriminator_netlist_parses_and_builds_mna`) compiles,
   and driven with a synthetic I/Q sweep shows capture (stronger input
   dominates), a click-rate rising below a CNR knee, and an HF-tilted noise
   spectrum.
3. No-op byte-identical guard: a netlist with no `B` element generates
   identical code before/after the feature.

## Not yet implemented

- Codegen emission (§Codegen integration plan above).
- `.func name(args)={expr}` reusable sub-expressions (Phase 3 / optional — inline
  expand at parse time).
- Branch-current (`I(elem)`) Jacobian coupling is supported in `Expr::diff`
  (via `Var::Branch`) but the resolver/stamp wiring for branch unknowns is part
  of the codegen work.
- ngspice validation: `.OPTIONS` / SPICE3 `B` correlation harness (the radio
  use case is plugin-driven I/Q, not an ngspice circuit).
