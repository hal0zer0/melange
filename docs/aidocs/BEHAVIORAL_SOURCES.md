# Behavioral (`B`) Sources — Arbitrary-Expression Sources

SPICE3 `B`-element support: a source whose output is an arbitrary scalar
expression over node voltages, branch currents, and time. Originally requested
by oomox (Subspace radio plugin) to make FM capture/threshold/click behavior
**emerge** from a limiter + discriminator instead of being faked in DSP.

> Status (2026-06-13): **front-end + MNA representation + IR plumbing landed and
> tested; nodal emitter stamping pending.** The expression engine (symbolic
> autodiff, named-parameter references), parsing, the MNA representation, and the
> `CircuitIR.behavioral_sources` plumbing are implemented and unit-tested. The
> node-space NR stamping in the emitter is **not yet wired** — `CodeGenerator`
> returns `UnsupportedTopology` for any circuit containing a `B` source so
> nothing silently mis-compiles. The remaining work is specified in
> [§Codegen integration plan](#codegen-integration-plan) below.
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
  `^` is right-associative.
- **Functions** `atan2(y,x) sqrt abs exp ln(=log) sin cos tanh min max pow`.
- **`ddt(x)` / `idt(x)`** — time derivative / integral of a sub-expression.

The `V`/`I` keyword may be written `V={..}`, `V ={..}`, or `V = {..}`.

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
  `(a±b∓|a−b|)/2` identity (subgradient through `abs`). `pow(a, const)` uses the
  power rule to avoid emitting `ln(a)` (which would blow up for `a ≤ 0`).
  Verified against central differences in `expr::tests::diff_matches_numeric`.
- `Expr::simplify()` — constant folding + identity elimination (keeps the
  emitted Jacobian compact; drops `0.0 * …` chains).
- `Expr::eval(&dyn EvalCtx) -> f64` — the interpreter, used by the DC OP solver
  and as the test oracle for the emitted straight-line code.
- `Expr::to_rust(&dyn ExprResolver) -> String` — straight-line Rust emitter
  (no AST interpreter in the audio thread). Guards match the interpreter:
  `bsrc_safe_exp` (clamp `[-40,40]`), `sqrt`→`.max(0.0).sqrt()`,
  `ln`→`.max(1e-300).ln()`, `atan2` native (guard `I²+Q²` at the netlist level
  with `+1e-9` as the request does).
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
- An emitted `bsrc_safe_exp(x)` free function (clamp `[-40,40]`).
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
