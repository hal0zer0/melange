# Design Decisions

The **why** behind melange's architecture. [`architecture.md`](architecture.md)
describes *what* the system is; this document records the load-bearing decisions
that shaped it, each with the alternative it beat and the cost it pays. It is a
decision log, not marketing — where a choice carries an unresolved tension, that
is stated rather than hidden.

Format per entry: **Context → Decision → Consequences (including what it costs)
→ Alternatives rejected → Status.**

---

## ADR-001 — Compile the circuit ahead of time; do not interpret it at runtime

**Context.** The generated code runs on a real-time audio thread: a hard
deadline every buffer, no allocation, no locks, no syscalls. A circuit's solver
is a fixed-size linear-algebra problem — but the *size* depends on the netlist
(number of nodes `N`, number of nonlinear device dimensions `M`).

**Decision.** melange is a **compiler**, not an interpreter. It resolves the
circuit's structure at build time and emits Rust source with the sizes baked in
as `const` generics (`[f64; N]`, not `Vec<f64>`), the Newton-Raphson iteration
inlined against the specific Jacobian block structure, and every precomputed
matrix (`S`, `K`, `A_neg`, `S·N_i`, the DC operating point) emitted as a `const`
array. rustc then optimizes it as if it were hand-written.

**Consequences.** The hot loop is branch-lean and register-resident; there is no
heap allocation in `process_sample()` and no `unsafe`. The cost is paid at the
topology boundary: **changing the circuit's shape means recompiling.** Component
*values* remain runtime-adjustable (see ADR-004's pots/switches), but you cannot
add a node to a shipped plugin. For an audio effect — fixed topology, knobs that
turn — this is the right side of the trade.

**Alternatives rejected.** A generic runtime solver with dynamically-sized
matrices would allocate and lose cache locality; a hand-written solver per
circuit (the original OpenWurli approach — an 8×8 DK solver that fit in
registers at a few ns/sample) is fast but does not generalize. Codegen keeps the
hand-written performance floor while generalizing across topologies.

**Status.** Accepted. The runtime interpreter path was removed; the codegen
pipeline is the only pipeline.

---

## ADR-002 — Automate the mechanical boundaries; leave the judgment boundary to a human

**Context.** Getting from a real circuit to a plugin crosses five translation
boundaries: schematic → netlist → MNA matrices → DK kernel → optimized Rust →
plugin.

**Decision.** melange automates boundaries 2–4 (netlist → MNA → kernel → code),
which are mechanical and deterministic, and deliberately **does not** automate
boundary 1 (schematic → netlist). A human must confirm the topology; melange
offers OCR/import assists but never asserts a netlist is correct on its own.

**Consequences.** The tool is honest about where its guarantees start. Boundary 1
is where the worst real-world bugs live — wrong component values, revision
mismatches, topology misreads — and those are matters of domain judgment, not
computation. Automating them would manufacture false confidence.

**Alternatives rejected.** "Schematic image → plugin, fully automatic" is the
demo-friendly pitch and the wrong one: it would launder unverifiable OCR guesses
into authoritative-looking output.

**Status.** Accepted. See boundary 1 in [`architecture.md`](architecture.md).

---

## ADR-003 — Three solver paths, auto-selected, with a manual override

**Context.** The Discrete-K (DK) method reduces a circuit to a tiny nonlinear
system (the `K` matrix — often 2×2 or 4×4) and is very fast. But it degrades on
stiff, large, or ill-conditioned circuits, on VCA circuits where `K ≈ 0`, and
where the reduced system loses numerical conditioning. A plain nodal formulation
is more robust but slower.

**Decision.** Emit one of three paths, chosen by a router from circuit
structure: **DK Schur** (small `M`, ≤1 transformer, well-conditioned `K`),
**Nodal Schur** (larger `M` or multiple transformers, still well-conditioned),
and **Nodal full-LU** (the robust fallback: `K ≈ 0`, positive-`K` diagonal, or
`K`/`S` ill-conditioned). The router keys off measurable properties — `M`,
transformer count, spectral radius, conditioning — not a per-circuit allowlist.
`--solver dk|nodal|auto` overrides it.

**Consequences.** Each circuit gets the fastest path that is still correct *for
it*. The cost is three codegen paths to maintain and keep in agreement — met by
a twin-emission discipline where DK and nodal must render byte-identical audio on
shared decks. The router must be conservative: the default is the broadest safety
envelope, and a wrong "fast" choice is a bug, not a tuning opportunity.

**Alternatives rejected.** A single universal path is either too slow (full-LU
everywhere) or too fragile (DK everywhere). Hand-annotating each circuit's path
does not scale and rots as circuits change.

**Status.** Accepted. Auto-selection is the default; the override exists for
diagnosis and for the rare circuit that sits on a routing boundary.

---

## ADR-004 — One CLI flag per genuine tradeoff, named after the mechanism

**Context.** Fixing one circuit sometimes regresses another because the two
genuinely want opposite numerical behavior (e.g. an oscillator that needs
trapezoidal energy conservation vs. a high-gain feedback loop that needs an
L-stable integrator). There is no single setting correct for both.

**Decision.** When two circuits need conflicting behavior, **expose both** behind
a flag named for the *mechanism* (not the circuit), and default to the broadest
safety envelope. Examples: `--solver`, `--backward-euler`, `--oversampling`,
`--opamp-rail-mode`, `--tube-grid-fa`, `--noise`. Auto-detection is preferred
wherever a property can be measured safely (e.g. spectral radius > 1 auto-selects
backward-Euler); a manual flag is the fallback when auto-detection cannot be made
sound.

**Consequences.** No circuit is silently sacrificed to another, and every mode is
inspectable and documented. The discipline that keeps this from becoming flag
soup: **every selectable mode must be correct for some real class of circuits** —
a flag is never a license to ship a broken mode, only to expose a real fork in
the physics.

**Alternatives rejected.** Picking the "lesser evil" default and letting the
other circuit degrade; or worse, tweaking component values to dodge the
conflict — which corrupts the model to flatter the solver (see ADR-005).

**Status.** Accepted, and actively applied — the most recent addition,
`--bjt-fa`, exposes an opt-in forward-active reduction that has no safe
auto-detector, precisely because auto-detecting it wrong would be
silently lossy.

---

## ADR-005 — Accuracy over usability; the bug is in melange until proven otherwise

**Context.** When a generated plugin sounds wrong — too hot, clipping, unstable —
the tempting fixes are all in the wrong place: add a catch diode, pad a resistor,
scale the output, tell the user to turn the knob down.

**Decision.** Schematic fidelity outranks plugin ergonomics by a wide margin. A
wrong sound is assumed to be a melange bug (solver, device model, or codegen)
until proven otherwise, and the fix goes in the MNA/device layer — **never** in
the netlist's component values, never as an output scale, never as a workaround
part the real circuit doesn't have.

**Consequences.** This is the invariant that governs every contributor and every
downstream repo: the netlist is a faithful description of hardware, and melange's
job is to run *that*, not a version of it that is easier to solve. It draws a
hard boundary with the plugin layer — clicks, CPU, knob feel, and UI are fixed in
the plugin; circuit behavior is fixed here. A slow, accurate solver beats a fast,
wrong one.

**Alternatives rejected.** "Make it sound good" as the target. That optimizes for
the demo and destroys the tool's actual value — a *general* circuit compiler,
not a curated pedal that happens to work.

**Status.** Accepted; enforced as a project-wide rule.

---

## ADR-006 — Verify before you validate: oracle-free proof is the primary evidence

**Context.** It is tempting to certify a circuit simulator by correlating it
against another simulator (ngspice). But that is circular: a shared model error
passes the check, and the comparison bakes in shared assumptions (both sides
DC-blocked, same Thevenin input) that cancel real disagreement.

**Decision.** Separate **verification** ("are we solving the equations right?" —
provable from mathematics alone) from **validation** ("are we solving the right
equations?" — requires external ground truth). Make verification the primary,
oracle-free evidence: integrator order by Richardson extrapolation against the
continuous-time solution; Tellegen power balance (Σv·i = 0) on the actual
compiled binary; each device model checked against its published equation to
machine precision. Cross-simulator agreement with ngspice is kept as *one* tier —
a peer check, not ground truth — and hardware fidelity is anchored honestly, for
the circuits where real measurements exist, at the grade those measurements
support.

**Consequences.** Correctness claims stand on references that cannot collude with
melange (calculus, physical invariants, canonical equations), and the docs state
each claim's *reach* alongside its evidence. The cost is intellectual honesty
about the ceiling: **solver correctness is not circuit fidelity**, and the docs
say so rather than letting one imply the other.

**Alternatives rejected.** ngspice correlation as the sole correctness story —
which is what the project had before, and which a single shared bug would defeat
silently.

**Status.** Accepted. Full ledger in
[`VALIDATION.md`](VALIDATION.md), including what is explicitly *not* proven.

---

## ADR-007 — Layered crates, a zero-dependency foundation, one direction of dependency

**Context.** A circuit compiler mixes numerics, device physics, matrix assembly,
codegen, and validation. Without structure these bleed into a mud ball.

**Decision.** Five crates in strict dependency order, each depending only on the
layers beneath it: **primitives** (DSP building blocks, zero external
dependencies) → **devices** (component models over a small `NonlinearDevice`
trait) → **solver** (MNA/DK engine + codegen, the core) → **validate** (ngspice
comparison) → **plugin** (nih-plug helpers). The `NonlinearDevice<const N: usize>`
trait is deliberately tiny — `current(v)` and `jacobian(v)` — so a new device is
a self-contained unit, and `N` (its NR dimension) is a compile-time constant for
stack allocation.

**Consequences.** The core solver never reaches "up" into validation or plugin
concerns; the foundation carries no dependency weight; a device model is testable
in isolation against its canonical equation (ADR-006). The cost is the usual
layering tax — some plumbing to pass data down the stack rather than reaching
across it — which is cheap next to the coupling it prevents.

**Alternatives rejected.** A single crate, or bidirectional dependencies "for
convenience," which would make the device-conformance tests impossible to keep
oracle-free.

**Status.** Accepted. Layer map in [`architecture.md`](architecture.md).

---

## ADR-008 — A language-agnostic IR behind a public emitter trait

**Context.** The generated target today is Rust (for nih-plug), but the value —
"circuit → optimized inner loop" — is not Rust-specific.

**Decision.** The pipeline lowers to a serializable, language-agnostic
`CircuitIR` before any code is printed; a backend is an `Emitter` implementation
over that IR. The `Emitter` trait is public, so a different target is an
additive backend rather than a fork of the solver.

**Consequences.** The hard part (MNA → kernel → IR) is written once and shared;
C++ and FAUST backends are planned as emitters, not rewrites. The cost is the
indirection of an explicit IR instead of printing code straight from the kernel —
worth it the first time a second target is needed, and structurally honest even
before that.

**Alternatives rejected.** Emitting Rust directly from the DK kernel — faster to
build initially, but it welds the whole toolchain to one output language.

**Status.** Accepted; Rust emitter shipped, additional emitters are future work.

---

## ADR-009 — f64 throughout; trapezoidal by default, backward-Euler when stability demands it

**Context.** Two numerical defaults set the accuracy/stability floor for every
generated circuit: working precision, and the integration rule for reactive
components.

**Decision.** Use **f64 everywhere** — no f32 fast path — because circuit
conditioning and the machine-precision verification targets (ADR-006) leave no
room for single precision. Integrate with the **trapezoidal** rule by default
(2nd-order accurate, energy-conserving), and switch to **backward-Euler**
(L-stable, 1st-order) only where stability requires it — auto-detected when the
companion system's spectral radius exceeds 1, or forced with `--backward-euler`.

**Consequences.** The common case gets the accurate integrator; the stiff,
high-gain feedback case gets the stable one, chosen by measurement rather than by
guesswork. The cost is f64's bandwidth/throughput vs. f32 — accepted deliberately,
consistent with ADR-005: correctness first.

**Alternatives rejected.** An f32 performance mode (silently trades away the
precision the verification tiers depend on); trapezoidal-only (rings or diverges
on stiff feedback loops); backward-Euler-only (needlessly damps the accurate
majority).

**Status.** Accepted.

---

*This log records decisions, not aspirations. Amend an entry when a decision
actually changes — and record the new tradeoff and its cost with the same
honesty, so the reasoning survives the people who made it.*
