# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
It is `0.x` software: below 1.0.0 there is no stability guarantee — the solver,
codegen output, CLI flags, and netlist semantics may all change.

## [Unreleased]

## [0.1.2] - 2026-08-26 — Sumac

### Added

- **`integration_source` in the generated `// provenance:` JSON** — `"explicit"`
  (`.integrator be` / `--backward-euler`), `"auto-promoted"` (the trap-stability
  discriminator), `"behavioral"` (behavioral-`B`-source forced), or `"trap"`. The
  human `Build:` line already carried this distinction; the JSON now does too, so
  a consumer can assert *why* the integration scheme is in effect without
  string-matching the label.

### Changed

- **Circuits are referred to by function/topology, not brand** across the
  examples and docs (e.g. "passive tube EQ", not "Pultec EQP-1A"); brand names
  appear only as style references ("Pultec-style").
- **Bundled passive-EQ example reframed to its `testing/`-tier status.** The
  canonical circuit was promoted `unstable/` → `testing/` in the circuits
  library; docs now compile it by the `testing/filters/passive-eq1a` path and
  describe it honestly — measured-verified against the factory curve charts,
  **never auditioned**, with **idealized (linear-iron) distortion** (it EQs like
  the original, it does not yet color like it).

### Fixed

- **Generated-code provenance commit no longer goes stale on local branch
  builds** — `build.rs` now watches `.git/logs/HEAD` (updated on every commit),
  not only `.git/HEAD` (unchanged on a branch), so `// melange: <ver> (<commit>)`
  tracks the actual HEAD.

## [0.1.1] - 2026-08-26 — Mace

A hardening and provenance patch. No change to generated DSP: the compiled
audio path is byte-identical to 0.1.0 (verified at the source level — every
codegen change is an added comment header or a never-called `pub` item).
Downstream consumers (OpenWurli/oomox) do **not** need to regenerate. MSRV is
unchanged (1.85).

### Added

- **Self-describing generated code.** Emitted circuits now carry a provenance
  header — a `// melange: <version> (<commit>)` line, an extended `Build:` line
  covering the fully *resolved* DSP-affecting flags (integration, `dc-block`,
  `noise`, `opamp-rail`, `bjt-fa`, oversampling, `max_iter`), and a
  machine-readable `// provenance: {…}` JSON line a consumer can assert against
  at compile time. This makes a silent DSP-contract difference (e.g. `--bjt-fa
  force` vs `auto`, or an unexpected output DC-block) visible in the artifact
  itself.
- **Node-name → DC-OP map in generated code.** A `pub const NODE_NAMES: [&str; N]`
  array (parallel to `DC_OP`, unnamed augmented rows are `""`) plus a
  `dc_op_by_name(name) -> Option<f64>` lookup, emitted by **both** the DK and
  nodal paths. Reading one node's baked operating point is now a lookup instead
  of recompiling the netlist per node.
- **Bundled example.** The Pultec-style passive tube EQ ships in-tree at
  [`examples/passive-eq1a.cir`](examples/passive-eq1a.cir) (a byte-identical
  mirror of the canonical melange-circuits netlist) so a real circuit can be
  compiled without first wiring up a circuit source.

### Fixed

- **SPICE validation now compares the same circuit on both sides.** The ngspice
  reference deck previously kept each `.pot` / `.switch` element at its netlist
  *nominal* value while melange used the element's compiled *default*; when the
  two differed, `melange validate` was correlating two different circuits. The
  reference deck now substitutes each element's melange default (`.gang` is
  intentionally excluded — it is a UI grouping, not baked state). Example:
  passive-eq1a correlation rose from 0.808 to 0.99999728.

### Security

- **`quick-xml` 0.37 → 0.41.0**, clearing the two waived KiCad-import advisories
  RUSTSEC-2026-0194 (quadratic parse on duplicate attributes) and
  RUSTSEC-2026-0195 (unbounded namespace allocation). `cargo audit` is clean.
  MSRV 1.85 is held (0.42 was excluded — its MSRV 1.86 exceeds ours; 0.41.0's is
  1.79). The reader-API migration also fixed a latent entity-drop bug in
  `melange import` (0.41 emits `Event::GeneralRef` separately), covered by two
  new regression tests. Closes the 0.1.1 follow-up tracked in the 0.1.0
  `Security` note.

## [0.1.0] - 2026-08-25 — Saffron

First tagged release. Melange compiles SPICE netlists to standalone, real-time-safe
Rust DSP code (raw code or a nih-plug plugin project). This entry consolidates the
project's development to date into the feature set as it stands at the tag.

**Validation status is deliberately narrow.** The solver numerics and device models
are verified oracle-free to machine precision (Tellegen power-balance on the compiled
binary, trapezoidal integration order by Richardson extrapolation, hand-computed device
anchors). Generated code is compared sample-by-sample against ngspice as a corroborating
peer check. Only **one** circuit — the Wurlitzer 200A preamp — has been checked against
measured real hardware. Everything else is unproven against hardware. See
[README](README.md) "Correctness & Validation" and `docs/aidocs/STATUS.md`.

### Added

- **Compilation pipeline**: SPICE netlist → parser → MNA assembly → DK kernel →
  language-agnostic `CircuitIR` → Rust emitter. Generated DSP is completely
  standalone with zero runtime dependency on melange. Subcircuit expansion
  (`.subckt` / `X` elements, up to 8 levels of nesting).
- **Three auto-selected solver routes**: DK Schur (M<10, ≤1 transformer,
  well-conditioned K), Nodal Schur (medium complexity), and Nodal full LU (K≈0 as in
  VCA circuits, positive-K diagonal, or ill-conditioned K/S). Newton-Raphson at
  dimensions M=1 (direct), M=2 (Cramer's), and M=3..16 (Gaussian elimination with
  partial pivoting); `MAX_M=24`. Full-LU path stacks the chord method, cross-timestep
  Jacobian persistence, and compile-time sparse LU (AMD ordering, symbolic
  factorization). See `docs/aidocs/DK_METHOD.md`, `NR_SOLVER.md`, `LINEAR_ALGEBRA.md`.
- **Trapezoidal companion models** for capacitors and inductors; inductors and
  transformers via augmented MNA (branch-current unknowns), well-conditioned for large
  inductances. Coupled inductors and multi-winding transformers supported. Optional
  backward-Euler integration (`--backward-euler`) for unconditionally stable solving of
  high-gain feedback circuits, plus a nodal auto-BE promoter (with `--force-trap`
  escape hatch) and the `.integrator {trap|be}` netlist directive.
- **DC operating point solver**: LU with partial pivoting, logarithmic junction-aware
  voltage limiting (pnjlim/fetlim style), source stepping and Gmin stepping fallbacks,
  ngspice-style internal nodes for parasitic BJTs, and a low-rate warmup for circuits
  whose direct DC-OP does not converge. See `docs/aidocs/DC_OP.md`.
- **Device models** (standard SPICE `.model` parameter names throughout):
  - **Diode** — Shockley + series resistance `RS` + junction capacitance `CJO` +
    `BV`/`IBV` Zener breakdown.
  - **BJT** — Gummel-Poon (`VAF`/`VAR`/`IKF`/`IKR`, `CJE`/`CJC`, `NF`/`ISE`/`NE`)
    following ngspice `bjtload.c`, with Ebers-Moll fallback and `RB`/`RC`/`RE`
    parasitic resistances. See `docs/aidocs/GUMMEL_POON.md`.
  - **JFET / MOSFET** — 2D Shichman-Hodges / SPICE Level 1, with `CGS`/`CGD` junction
    caps, `RD`/`RS` parasitics, and MOSFET body effect (`GAMMA`/`PHI`).
  - **Vacuum triode** — Norman Koren plate model + Marshall Leach grid current,
    `CCG`/`CGP`/`CCP` junction caps, `RGI` grid-stop.
  - **Vacuum pentode / beam tetrode** — screen-current equation families (Reefman
    Rational §4.4, Reefman Exponential §4.5, Classical Koren, plus a variable-mu
    blend), placed with the `P` element and `VP` model token. Grid-off dimension
    reduction (3D→2D) under `--tube-grid-fa {auto|on|off}`.
  - **Op-amp** — Boyle VCCS macromodel with `GBW` dominant-pole node, `VSAT` output
    clamping, asymmetric `VCC`/`VEE` rails, optional `SR=` slew-rate limiting (V/µs),
    and selectable rail-saturation strategy via `--opamp-rail-mode
    {auto|none|hard|active-set|active-set-be|boyle-diodes}`.
  - **VCA** — THAT 2180 / DBX 2150 Blackmer current-mode exponential gain with
    gain-dependent THD, placed with the `Y` element.
  - **CdS LDR (opto)** — VTL5C-class photocell with attack/release dynamics on the
    stateful-device codegen path, placed with the `O` element. No ngspice twin (SPICE
    has no LDR model).

  See `docs/aidocs/DEVICE_MODELS.md`.
- **Dynamic parameters**: `.pot` / `.wiper` / `.switch` (up to 16) / `.gang` /
  `.runtime` directives. Pot and switch changes trigger a per-block matrix rebuild;
  per-sample smoothing is available for knob moves. `--format plugin` maps these to
  nih-plug parameters. `recompute_dc_op()` (opt-in) re-seeds the NR state for
  preset recall. See `docs/aidocs/DYNAMIC_PARAMS.md`.
- **Behavioral B-sources** (nodal path): arbitrary-expression `V={expr}` / `I={expr}`
  sources over node voltages, `time`, `ddt`, `idt`, and `.param`/`.runtime`
  parameters. Branch-current references and the DK path are not yet supported.
- **Oversampling** 2×/4×: self-contained polyphase half-band IIR anti-aliasing with no
  runtime dependencies (`--oversampling {1|2|4}`). See `docs/aidocs/OVERSAMPLING.md`.
- **Authentic circuit noise** (opt-in, `--noise {off|thermal|shot|full}` +
  `--noise-seed <u64>`; off by default and byte-identical to a noiseless build when
  off). Time-domain stochastic currents injected as Norton sources into the nonlinear
  MNA RHS, so noise is shaped by the circuit and modulated by the operating point.
  Sources: Johnson-Nyquist thermal (fixed and dynamic R), junction shot
  (diode/BJT/FET/triode, triode plate space-charge-smoothed), 1/f flicker on junctions
  (`.model … KF=/AF=`) and resistors (per-element Hooge), pentode partition, and op-amp
  en/in (`.model OA(EN=/IN=)`, white-band v1). Per-stream xoshiro256++ RNG. Runtime
  controls: `set_noise_enabled`, `set_noise_gain`, `set_thermal_gain`/`set_shot_gain`/
  `set_flicker_gain`, `set_temperature_k` (default 290 K), `set_seed`. See
  `docs/aidocs/NOISE.md` and `docs/NOISE_GUIDE.md`.
- **Device self-heating**: quasi-static electrothermal RC model (`RTH`/`CTH`/`XTI`/
  `EG`/`TAMB`) for diodes, BJTs, and triodes. Disabled by default (`RTH=∞` → dead
  code); analytic-validated only (SPICE3f5 silently drops `RTH`, so there is no ngspice
  parity).
- **Unit-variation directives** (`.mismatch` / `.tolerance` / `.seed`): per-device
  `.model` parameter jitter (`.mismatch D|Q P=tol …`) and per-passive value jitter
  (`.tolerance R=/C=/L=`), baked deterministically at codegen time (FNV → SplitMix64).
  Opt-in; byte-identical output when absent; breaks ngspice parity by design. `.mismatch`
  on `J`/`M`/`T` parses but is not yet wired into the IR. See `docs/aidocs/UNIT_VARIATION.md`.
- **Forward-active BJT reduction** via `--bjt-fa {auto|off|force}`. Under `auto`
  (default) only pure Ebers-Moll BJTs biased forward-active (Vbc < −0.5 V) are reduced
  to a 1D NR slot, where the reduction is exact; Gummel-Poon / ISE / self-heating /
  parasitic BJTs stay full-2D. `force` also reduces those (each with a per-device
  accuracy warning — it drops the qb base-charge term and is not accuracy-safe under
  signal); `off` keeps every BJT full-2D. `auto` is byte-identical to prior codegen.
- **Plugin generation** (`--format plugin`): a full nih-plug project targeting CLAP and
  VST3, split into a regenerable `src/circuit.rs` (all DSP) and a user-owned
  `src/lib.rs` (parameters, GUI, presets). Ships a default-on ear-protection soft
  limiter (`--no-ear-protection` to omit), optional Input/Output Level parameters
  (`--no-level-params`), and metadata flags (`--vendor`, `--vendor-url`, `--email`,
  `--vst3-id`, `--clap-id`). All audio-path buffers are pre-allocated — zero heap
  allocation in the callback.
- **CLI**: `compile`, `simulate` (WAV / test-tone through a circuit), `analyze`
  (frequency response with `--pot`/`--switch` overrides), `validate` (ngspice
  comparison), `dc-op`, `nodes`, `import` (KiCad netlist → `.cir`), `sources` (register
  external circuit repositories), `builtins` (deprecated — use `sources`), and `cache`.
  Circuits are referenced as a builtin, a registered `source:circuit`, a URL, or a
  local path.
- **Cross-compilation**: generated plugins build for macOS from Linux via zig 0.13+ /
  cargo-zigbuild (`--target universal2-apple-darwin`) with rcodesign ad-hoc signing.
  The `melange-cli` binary itself does not cross-compile.
- **KiCad integration**: a symbol library (`melange.kicad_sym`) and netlist import path
  covering triodes, pentodes, op-amps, VCAs, pots, wipers, and audio I/O markers.
- **Validation harness**: SPICE validation infrastructure comparing generated code
  against ngspice, plus parser hardening (input-size caps, non-ASCII normalization) and
  a cargo-fuzz target over the parser → MNA → DkKernel → CircuitIR path.
- **Minimum Supported Rust Version 1.85** (2021 edition), declared via `rust-version`
  and enforced in CI.

### Security

- **No `unsafe` code** in the melange library, CLI, or generated solver/DSP code. The
  sole exception: generated *plugin* projects emit one `unsafe` block
  (`std::arch::x86_64::_mm_setcsr`) to enable the CPU's FTZ/DAZ denormal-flush mode for
  real-time performance — the only `unsafe` in any melange output. See
  [SECURITY.md](SECURITY.md).
- **Input validation** — the parser rejects negative, zero, NaN, and infinite component
  values and self-connected components. `safe_exp` clamps arguments to [−40, 40] to
  prevent overflow in device equations.
- **Resource limits** — `MAX_M=24` (NR dimension), `MAX_N=256` nodes,
  `MAX_ELEMENTS=10,000` after expansion, 8-level subcircuit nesting.
- **Bounded iteration** — Newton-Raphson capped at `max_iter` (default 50); DC
  operating point has finite source-stepping and Gmin-stepping fallbacks.
- **Real-time safety** — generated audio callbacks perform no heap allocation, locking,
  or syscalls; all buffers are pre-allocated at construction.
- **Known advisories (waived for 0.1.0).** The KiCad import path (`melange import`, via
  `quick-xml` 0.37) carries two upstream denial-of-service advisories —
  RUSTSEC-2026-0194 (quadratic parse on duplicate attributes) and RUSTSEC-2026-0195
  (unbounded namespace allocation). Reachable only by importing a maliciously-crafted
  KiCad file; no effect on netlist compilation, generated code, or shipped plugins. The
  fix (`quick-xml >= 0.41`) is tracked for 0.1.1.

[Unreleased]: https://github.com/hal0zer0/melange/compare/v0.1.2...HEAD
[0.1.2]: https://github.com/hal0zer0/melange/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/hal0zer0/melange/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/hal0zer0/melange/releases/tag/v0.1.0
