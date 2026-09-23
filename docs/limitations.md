# Known Limitations

This document lists known limitations of melange. `[DEFERRED]` marks an
intentional scope boundary; `[OPEN]` marks a root-caused defect that is not fixed
yet; `[EXPERIMENTAL]` marks a surface whose syntax may still change.

The bias here is deliberate: over-disclose rather than oversell. A limitation
stays on this page until it can be *shown* not to hold.

`docs/aidocs/STATUS.md` is the maintainer-side feature inventory and is kept
current release by release; where the two disagree it wins, except for the
performance figures below, which were last re-measured here.

## SPICE Compatibility

### Supported Element Types

Melange supports the following SPICE elements:

| Prefix | Type | Notes |
|--------|------|-------|
| R | Resistor | |
| C | Capacitor | `IC=` initial voltage supported |
| L | Inductor | `ISAT=` for saturation, **uncoupled inductors only** (see Saturating Inductors) |
| V | Voltage source | DC value (+ optional AC mag/phase). A transient spec (`SIN`/`PULSE`/`PWL`/`EXP`/`SFFM`/`AM`) is a **hard parse error** — audio comes in through the input node, not a source |
| I | Current source | Same: DC only, transient specs rejected |
| D | Diode | Shockley + RS + BV/IBV (Zener) |
| Q | BJT (NPN/PNP) | Ebers-Moll and Gummel-Poon |
| J | JFET (NJF/PJF) | Shichman-Hodges |
| M | MOSFET (NM/PM) | Level 1 SPICE with body effect (GAMMA/PHI) |
| T | Triode tube | Koren plate current + Leach grid current |
| P | Pentode/beam tetrode | 5 equation families, 29 catalog models |
| U | Op-amp | Boyle macromodel (GBW, VCC/VEE rails, SR) |
| Y | VCA | THAT 2180-style exponential gain |
| E | VCVS | Voltage-controlled voltage source |
| G | VCCS | Voltage-controlled current source |
| K | Coupled inductors | Transformers (multi-winding supported) |
| O | LDR / photoresistor | `CdsLdr` model, `.model NAME LDR()` |
| N | Glow discharge / neon lamp | **EXPERIMENTAL.** `.model NAME NEON(...)`; the `N` letter and the `NEON` model type are provisional and may change |
| B | Behavioral source | `V={expr}` / `I={expr}`, nodal path only -- see below |
| X | Subcircuit instance | Recursive expansion, max nesting depth 8 (`MAX_NESTING_DEPTH`, `crates/melange-solver/src/parser.rs:588`) |

### Missing Element Types [DEFERRED]

- **F** (Current-Controlled Current Source) -- current mirrors
- **H** (Current-Controlled Voltage Source)
- **Transmission lines.** Note that melange's `T` prefix is the **triode**, not
  SPICE's transmission line; there is no transmission-line element.
- **S / W** (voltage- and current-controlled switches) -- use `.switch` for
  discrete component-value selection instead

An unknown element letter is a hard parse error naming the letter; unsupported
elements are never silently dropped.

**Workaround:** Model these using combinations of existing elements where possible.

### Behavioral Sources (B) [PARTIAL]

`B... V={expr}` / `I={expr}` arbitrary-expression sources are wired on the
**nodal codegen path**: expressions over node voltages, `time`, `ddt`, `idt`,
and `.param`/`.runtime` parameters compile and are oracle-validated
(`behavioral_source_tests.rs`). Not yet supported: branch-current references in
expressions, and the DK path -- a circuit containing a B-source routes nodal, or
errors loudly if it cannot. See `docs/aidocs/BEHAVIORAL_SOURCES.md`.

Two structural consequences a deck author should expect, both visible in the
generated `// provenance:` header:

- The circuit is pinned to the **nodal full-LU sub-path**
  (`"nodal_subpath":"full-lu"`). `--nodal-subpath schur` is refused outright,
  because the Schur reduction cannot express node-space stamping and forcing it
  would silently drop the nonlinearity
  (`crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs:1739`).
- Integration is forced to **backward Euler**
  (`"integration_source":"behavioral"`), so a B-source circuit gives up
  second-order trapezoidal accuracy.

Functions available in expressions: `sqrt`, `abs`, `exp`, `sin`, `cos`, `tanh`
and the rest of `crates/melange-solver/src/expr.rs`. Because `time` is a real
variable advanced one `dt` per inner sample, an in-circuit LFO *is* expressible
here -- see the Not Implemented section.

### Missing Directives [DEFERRED]

These are **ignored with a `log::warn!`** naming the directive, rather than
rejected -- a deck carrying them parses, but the directive does nothing:

- `.include` / `.lib` -- file inclusion
- `.temp` -- temperature specification
- `.global` -- global nodes
- `.nodeset` / `.ic` -- initial conditions (note: `IC=` **on a capacitor** *is*
  honoured; the standalone directives are not)
- `.option` / `.options`
- Analysis directives (`.tran`, `.ac`, `.dc`, `.op`, `.print`, `.plot`) -- melange
  drives the circuit from the input node and its own CLI, not from the deck

### Parametric Expressions [DEFERRED]

Expressions like `{R1*2}` or `{RVAL}` in **component-value** positions are not
supported -- they are a parse error naming the offending value, not a silent
fallback. Only numeric values with scale suffixes are accepted there.

`.param` values *are* usable, but only inside behavioral `B`-source expressions
(see below).

### Temperature Dependencies [PARTIAL]

Temperature coefficients (TC1, TC2) on resistors are ignored, and there is no global temperature sweep (`.temp`). Device self-heating (Rth, Cth, XTI, EG, TAMB) is available via a quasi-static thermal RC model with SPICE3f5 IS(T) scaling for **diodes, BJTs, and triodes** (default disabled, Rth=infinity → dead code). Base device models otherwise run at a fixed nominal 27C.

Separately, the authentic-noise feature carries a runtime-settable noise temperature (`set_temperature_k`, default 290 K), which scales only thermal noise — it does not affect the deterministic device equations. See the Circuit Noise section below.

## Device Model Limitations

### Diode
- BV/IBV: hard clamp reverse breakdown (no smooth Zener knee)
- No TC1/TC2 temperature coefficients; optional quasi-static self-heating (RTH/CTH/XTI/EG/TAMB), disabled by default

### BJT (Gummel-Poon)
- Q1 Early effect guard: `q1_denom <= 0` clamps to 1.0 (physically near Early voltage limit)
- Self-heating (RTH/CTH) available but disabled by default (RTH=infinity)
- Junction capacitances (CJE/CJC) and diffusion capacitance (TF) available
- Parasitic resistances (RB/RC/RE) supported with internal nodes
- No substrate current or avalanche breakdown

### JFET / MOSFET
- Subthreshold: hardcoded 2xVT slope (real devices: 60-120 mV/decade)
- MOSFET Level 1 only (no BSIM3/4)

### Triode
- Koren model with lambda for finite plate resistance
- No space-charge or transit-time effects

### Pentode
- 5 equation families: Rational (Derk), Exponential (DerkE), Classical (Koren/Cohen-Helie), plus variable-mu variants
- 29 catalog models (EL84, EL34, EF86, 6L6, 6V6, KT88, 6550, 6K7, EF89, and more)
- Grid-off dimension reduction (3D to 2D) exists but is **opt-in only**:
  `--tube-grid-fa on`. Since 2026-09-04 `auto` behaves as `off` and keeps the
  full 3D model, because the reduction is **not** accuracy-neutral -- it drops
  the cathode/screen-referenced Vg2k feedback (measured +2% to +12% small-signal
  gain error on cathode-biased stages) and all grid current for Vgk > 0. `on`
  warns per device. An exact reduction is deferred; until one exists, `auto`
  will not reduce.
- No independent suppressor dynamics (suppressor always cathode-tied)
- 6386/6BA6/6BC8 datasheet fits for variable-mu compressors deferred (phase 1d)

### Op-amp
- Boyle macromodel with GBW dominant pole
- VCC/VEE asymmetric supply rail clamping
- Slew-rate limiting via `SR=` in V/us (per-sample clamp, all 3 codegen paths)
- Rail mode selection: `--opamp-rail-mode {auto,none,hard,active-set,active-set-be,boyle-diodes}`
  (`active-set-be` is accepted but is not listed in `--help`)
- `auto` resolves only to `none` / `hard` / `active-set` / `active-set-be`. It
  **never** selects `boyle-diodes`: that mode is validated for light clip and
  diverges at heavy clip, so it stays opt-in
  (`crates/melange-solver/src/codegen/ir/opamp_rail.rs:340-347`; see
  `docs/aidocs/OPAMP_RAIL_MODES.md`, "The BoyleDiodes heavy-clip problem")
- An explicit `--opamp-rail-mode` is honoured verbatim and is never silently
  upgraded, including `none` on a circuit `auto` would have clamped

### VCA
- 2D current-mode exponential gain (THAT 2180 / DBX 2150)
- THD: gain-dependent cubic nonlinearity
- `noise_floor` field exists but unused

### Saturating Inductors
- `L1 a b 100m ISAT=20m`. Anhysteretic saturating flux
  `Φ(i) = L0·Isat·tanh(i/Isat)`, with the Jacobian built from the differential
  inductance `L_diff(i) = L0/cosh²(i/Isat)`
- **Uncoupled inductors only.** Coupled/transformer core saturation is NOT
  available. Types named `SaturatingTransformerGroupIR` / `winding_isats` do
  exist in the tree, but they saturate each winding independently off its own
  branch current, which is physically wrong for a shared core -- they are
  unvalidated and unused. Do not put `ISAT=` on a transformer winding and expect
  core saturation. See `docs/aidocs/SATURATING_TRANSFORMERS.md` §1, which is a
  design plan, not a description of shipped behaviour.
- Lagged: `L(I)` is evaluated from the previous solved current, and the update is
  decimated to every `SAT_UPDATE_INTERVAL = 32` samples, with a Sherman-Morrison
  rank-1 patch on change and a full O(N³) rebuild every
  `SAT_RESYNC_INTERVAL = 16` SM updates to bound drift
- A circuit with an uncoupled saturating inductor is forced onto the **nodal
  full-LU** sub-path (the inductor is stamped as a nonlinear device inside the NR
  loop); `--nodal-subpath schur` is refused
  (`crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs:1730`)
- No ngspice validation
- No magnetic hysteresis, core loss, or remanence anywhere in the code

### Glow Discharge / Neon (`N`) [EXPERIMENTAL]
- `N1 anode cathode MODEL` + `.model MODEL NEON(VO VM IK RS IHOLD ROFF)`. The
  element letter and model type are explicitly provisional
  (`crates/melange-solver/src/parser.rs:870`)
- `--subsample-fire {auto,on,off}` splits a firing sample at the crossing
  fraction so the strike instant is not quantised to the sample grid. It is a
  **nodal-Schur-only** feature: `on` is refused on the DK route and on the nodal
  full-LU sub-path, and `auto` is inert everywhere else. This is not silent --
  every glow deck records `subsample_fire: {mode, active, reason}` in the
  generated `// provenance:` header, so a DK-routed deck reads
  `"active":false,"reason":"dk-route"` rather than nothing at all. Read it; do
  not assume the feature is on because the flag defaults to `auto`
- The lit-phase sub-step time constant is a conservative **heuristic**, not a
  derived loop τ (provenance reports `"tau_source":"heuristic"`). It assumes the
  terminal self-capacitance discharges through `RS` alone, which is a lower bound;
  the real loop τ can be several times larger. The failure direction is CPU cost,
  not accuracy
- Stage A is not real-time optimised: a firing sample costs two O(N³)
  inversions
- `--subsample-lit-factor` is a diagnostic bisection knob, not a per-deck tuning
  parameter
- Relaxing-section / delayed-overvoltage / KSUB glow models are **refused** on
  the nodal full-LU sub-path rather than silently mixed with the static lit
  branch; `--allow-static-glow-on-full-lu` turns them inert instead, for
  diagnosis only
- No ngspice twin

### LDR (Photoresistor)
- `CdsLdr` device model (VTL5C3/4, NSL-32 presets) with attack/release photocell dynamics
- Placed in a netlist via the `O` element (`O1 rphoto+ rphoto- led+ led- MODEL` + `.model MODEL LDR()`), on the stateful-device codegen path (both DK and nodal)
- No ngspice twin (SPICE has no equivalent LDR model to validate against)

## Dynamic Parameter Controls

### Potentiometers
- `.pot R1 min max` marks a resistor as runtime-variable
- `.wiper R_cw R_ccw total_R` models 3-terminal wiper pots
- `.gang "Label" member1 member2` links multiple pots/wipers to a single UI parameter (`!` prefix inverts a member)
- Per-block O(N^3) matrix rebuild on value change
- Per-sample smoothing via `.smoothed.next()` in generated plugin
- Reseed-free setters (stamp ΔG only, no mid-signal NR-state reset); call `recompute_dc_op()` explicitly for preset-recall / unsmoothed jumps (DK path)
- Maximum 64 `.pot` directives, and separately a maximum of 64 combined
  `.pot` + `.wiper` leg entries

### Switches
- `.switch C1 100n 220n 470n` selects among discrete component values
- Maximum 16 switches per circuit; maximum 32 positions per switch
- Triggers matrix rebuild on position change

### Host-Driven Modulation
- `.runtime V as <field>` binds an existing voltage source to a public field the
  host writes per sample
- `.runtime R min max as <field>` is audio-rate resistor modulation: it emits
  `set_runtime_R_<field>(r)` **without** the `.pot` DC-OP warm re-init, because
  that re-init clicks at envelope-follower rates. No nih-plug knob is generated
- `.runtime R` members are rejected inside `.gang` at parse time

### Known Defect: NR can cap out and still report a healthy-looking output [OPEN]

Newton-Raphson has a per-sample iteration cap (`max_iter`, 100 by default). When
it reaches that cap it emits the last iterate and continues. That iterate is not
a converged solve, and **nothing in the output makes it obvious** — the level can
look entirely normal.

A deck in the golden corpus does this on every sample after a step edge:

```
frames                    48000    (edge at sample 4800 -> 43200 samples after it)
diag_nr_max_iter_count    43200    every post-edge sample hits the cap
diag_be_fallback_count    43199
diag_ls_fail_count      2246353    ~52 line-search failures per sample
diag_refactor_count     4190402    ~97 full LU refactors per sample
diag_peak_output         0.9439 V  peak -0.50 dBFS — nothing looks wrong
```

Whether the capped iterate differs audibly from a converged solve on that deck is
**not yet established**; the investigation is open. What is established is that
melange will not tell you when this happens.

**How to check your own circuit.** `melange simulate` prints these counters. If
`nr_max_iter_count` is a large fraction of your sample count, the solver did not
converge on those samples and the output is not trustworthy, whatever the level
looks like. A healthy circuit shows a count near zero — the same deck's sine
program, over more samples, shows 31.

Mitigations worth trying: raise the NR budget, soften the stimulus edge, or
`--backward-euler`. If the counter stays pinned, the circuit is hitting a genuine
conditioning problem and the number melange prints should not be trusted.

### Known Defect: conductance-swap transient [OPEN]

Changing a conductance mid-run -- any `.switch`, `.pot`, or `.runtime R` setter --
has a reproducible artifact on the swap sample, root-caused 2026-08-15 and not
yet fixed (the fix touches the core solver and is gated on full golden/SPICE
re-validation).

Trapezoidal puts every conductance in **both** the forward matrix
`A = G + (2/T)C` and the history matrix `A_neg = (2/T)C - G`, so a change of Δg
adds `+Δg` to one and `-Δg` to the other. On the swap sample the output deflects
exactly **2.000×** the physical value -- drive-independent, deterministic, correct
one sample later.

The same event also excites the trapezoidal `z = -1` Nyquist marginal mode:

- On a node with capacitance, it rings for ~1000 samples and is damped by the
  circuit's own RC.
- On a **capless (purely resistive) node it does not decay at all.** The residual
  persists for as long as you rest in the non-default position. A resistive
  divider node should obey `node - ratio·other ≈ 0` at all times; under trap,
  held off-default, it does not.

This is a bug, not a contract -- do not design around it as "undefined". Both
mitigations available today: `--backward-euler` (measured residual -3.5e-13 on
the capless repro, versus 1.44% under trap), or fitting transients from
closure+1 sample. Measurements taken while resting in a non-default position on a
capless node are contaminated under trapezoidal.

### Device Linearization
- `.linearize Q9` or `.linearize T1` removes a BJT or **triode** from the NR
  system. Only those two element kinds are eligible; any other name is warned
  about and ignored, not rejected
  (`crates/melange-solver/src/pipeline.rs:146-156`)
- Replaced with small-signal conductances at DC operating point
- Reduces nonlinear dimension M (BJT: M-2, triode: M-2 per device)
- Device still affects the circuit via linearized g_m, g_pi, r_o stamps in G

## Numerical Limitations

### Matrix Storage [PERFORMANCE]

Matrices use `Vec<Vec<f64>>` (jagged arrays) instead of flat storage. This has poor cache locality but is acceptable for typical circuits (validated up to N=64).

### Denormal Handling

Generated code flushes denormals in the state vectors (`v_prev`, and `i_nl_prev`
when M > 0) once per sample with an add/subtract of `1e-25`, on **both** the DK
and nodal paths. Most DAW hosts set FTZ/DAZ, but the generated code does not rely
on it. The DC-blocking feedback path carries a tiny bias for the same reason.

This covers the state that persists across samples. It is not a global FPU mode
change -- an intermediate inside one sample's solve can still go denormal.

### Condition Number [NUMERICAL]

Condition number is estimated during DK kernel build as
`||A||_inf · ||A^-1||_inf`. A `log::warn!` fires above **1e13**
(`crates/melange-solver/src/dk.rs:299-315`) -- the threshold was raised from 1e12
because high conditioning is common and usually benign (tight component-value
spreads, near-unity transformer coupling). Ill-conditioned circuits still produce
results, possibly with reduced accuracy. A genuinely ill-conditioned `K` or `S`
is handled separately by routing (below), not by this warning.

### Nonlinear System Size

Codegen supports up to M=24 nonlinear device dimensions (`MAX_M=24`,
`crates/melange-solver/src/dk.rs:230`). M=1 is solved directly, M=2 by Cramer's
rule, M=3..24 by Gaussian elimination with partial pivoting on a block-diagonal
Jacobian.

Three mechanisms reduce M, and only the first is on by default:

- **BJT forward-active detection** (`--bjt-fa {auto,force,off}`). `auto` reduces
  only pure Ebers-Moll BJTs, where the 1-D forward-active model is exact.
  Gummel-Poon / ISE / self-heating / parasitic BJTs stay full 2-D. `force`
  reduces them too, per-device warned, at a documented accuracy cost (drops the
  `qb` base-charge term).
- **`.linearize`** (explicit, per device).
- **Pentode grid-off** (`--tube-grid-fa on` only -- `auto` does not reduce; see
  the Pentode section).

## Solver Limitations

### Voltage Sources

Independent voltage sources (V elements) use **augmented MNA**: each source adds a branch-current unknown plus a KVL constraint row (`B^T · x = v_dc`). There is no high-conductance Norton stamp. See `VoltageSourceInfo` in `crates/melange-solver/src/mna.rs` and `solve_dc_op` in `crates/melange-solver/src/dc_op.rs`.

The **audio input** is a separate mechanism and does not go through that path: it is a Thevenin source stamped as a conductance `G_in` at the input node (default 1 Ω, or the `.input_impedance` directive / `--input-resistance` override), with RHS `(V(n+1) + V(n)) · G_in`.

### Solver Routing

Routing happens in two independent stages, and the generated `// provenance:`
header records both (`"solver"` and `"nodal_subpath"`). Read it rather than
inferring the path from the netlist.

**Stage 1 -- DK vs nodal** (`crates/melange-solver/src/codegen/routing.rs`).
DK Schur precomputes `S = A^-1` and iterates only the M coupled device
dimensions; it costs O(N²+M³)/sample. A circuit goes nodal instead when any of
these holds:

| Trigger | Threshold |
|---------|-----------|
| Large nonlinear dimension | M >= 10 |
| Multiple transformer groups | > 1 coupled-inductor / transformer group |
| DK kernel build failed | -- |
| Trapezoidal instability | spectral radius of `S·A_neg` > 1.002 |
| `K` ill-conditioned | max\|K\| > 1e8 (`K_ILL_COND_MAX`) |
| `S` ill-conditioned | max\|S\| > 1e6 (`S_ILL_COND_MAX`) |
| Behavioral `B` source | structural -- DK cannot stamp in node space |
| Saturating inductor (`ISAT=`) | structural -- DK bakes `S = A^-1` and cannot update L per sample |

The last two are hard structural requirements: `--solver dk` is **rejected**, not
downgraded.

**Stage 2 -- nodal Schur vs nodal full LU**
(`crates/melange-solver/src/codegen/rust_emitter/nodal_emitter.rs:1730-1810`).
Nodal Schur costs O(N²+M³)/sample; full LU factors the whole augmented N×N system
every NR iteration at O(N³)/sample. Full LU is selected when the circuit
structurally requires it (saturating inductor, behavioral source) or on
conditioning grounds: a positive `K` diagonal with a live current column,
degenerate `K` (including K≈0, the VCA case), `k_diag_min < -1e12`, ill-conditioned
`K` or `S`, or an unstable Schur prediction.

Overrides:

- `--solver {auto,dk,nodal}` for stage 1.
- `--nodal-subpath {auto,schur,full-lu}` for stage 2. This is a **diagnostic**
  escape hatch in the same family as `--force-trap`, meant for isolating the
  sub-path as a variable; it warns when it contradicts `auto`, and `schur` is
  refused outright on circuits that structurally require full LU.

**Known latent gap:** Schur NR can diverge on expanded parasitic internal nodes
where the same circuit converges unexpanded, or expanded on full LU. No corpus
deck is in that state and the CLI's K-gate never constructs the combination, so
it is latent rather than observed.

## Real-Time Constraints

### Allocation

Generated code pre-allocates all buffers in `CircuitState`. No heap allocation occurs in `process_sample()`.

### Worst-Case Performance

Matrix recomputation is O(N^3) and occurs at:
- `set_sample_rate()` calls
- `set_pot_N()` / `set_switch_N()` / `set_runtime_R_<field>()` calls (per-block,
  when the value actually changes; batched into one rebuild per sample via a
  `matrices_dirty` flag on the nodal path)
- the saturating-inductor drift resync, every 16 Sherman-Morrison updates

There is no current benchmark for pot-rebuild latency. A `~250 us at N=37` figure
appeared here from 2026-04 with no reproducible source and no deck attribution,
so it has been removed rather than carried forward -- see the note under
Performance Benchmarks about what happened to the rest of the unattributed
numbers. If you need this figure, measure it on your own target with
`tools/perf-harness/bench.sh`.

`set_sample_rate()` cannot change the *route*. Stage-1/stage-2 routing and
sub-sample-fire activation are compile-time structural decisions, so a plugin
that must run at several host rates has to be compiled per rate.

### Performance Benchmarks

Measured on an AMD Ryzen 9 7950X, single core, noiseless, `-C target-cpu=x86-64-v3` (median of 7 × 2M samples via `tools/perf-harness/bench.sh`); throughput is host-dependent.

- Light nonlinear circuits: 12AX7 gain stage ~230×, overdrive pedal (1 op-amp + 2 diodes) ~64× realtime
- Germanium diode network (6 Ge diodes) ~12× realtime
- Typical multi-device circuits: Wurlitzer preamp ~56×, tweed-style guitar amp ~23× realtime
- Heaviest measured: a passive tube EQ (nodal full-LU, chord + sparse LU, N=52, M=8) ~24×, a bus compressor (12 op-amps + 2 VCAs) ~7.1× realtime

Every figure above names the circuit it came from, deliberately. Perf numbers in
this repository from before 2026-08-25 were fabricated or stale -- a row nobody
can map to a circuit is a row nobody can check, which is how an 18× claim
survived for a deck that measures 12×. Treat any ×-realtime number without an
attributed circuit and a named host as unverified.

## Circuit Noise [PARTIAL]

Authentic time-domain circuit noise is implemented (opt-in, off by default) and
injected as Norton current sources into the MNA RHS, so it is shaped by the
solver's transfer function and modulated by the nonlinear operating point.
Enable with `--noise {off|thermal|shot|full}` (`--noise-seed <u64>` for
determinism). Runtime controls: `set_noise_enabled`, `set_noise_gain`,
`set_thermal_gain` / `set_shot_gain` / `set_flicker_gain`, `set_temperature_k`,
`set_seed`. With `--noise off` (default) the generated code is byte-identical to
a noiseless build. Full reference: `docs/aidocs/NOISE.md`.

Shipped noise sources:
- **Thermal (Johnson-Nyquist)** on every fixed and dynamic (`.pot`/`.wiper`/`.runtime R`/`.switch`) resistor
- **Shot** on every junction (diode, BJT, JFET/MOSFET, triode plate with space-charge smoothing)
- **1/f flicker** on junctions (`.model … KF=… AF=…`) and on resistors (per-element `KF=`/`AF=`, Hooge bias-squared)
- **Pentode partition** noise (Schottky) and **op-amp en/in** (`.model OA(EN=… IN=…)`, white-band v1)

Noise limitations:
- Op-amp `EN_FC`/`IN_FC` (1/f corner) parameters parse but are **not yet wired** — Phase 4 is white-band only
- On the **DK codegen path**, BJT parasitic RB/RC/RE thermal noise (rbb′) is skipped (logged as a `warn!`); route the circuit nodal to include it
- Diode `RS` and tube `RGI` parasitic resistances are not yet thermal-noise sources
- Setting `KF`/`AF` on resistors breaks ngspice parity — strip before SPICE-validating. (`.mismatch`/`.tolerance` jitter does not need stripping: `melange validate` disables it on melange's side automatically and says so on the result line)
- **Tube microphonics** (Phase 6) is research only, not implemented

## Not Implemented [DEFERRED]

- **LFO/Modulation**: independent `V`/`I` sources are DC only -- there is no
  `SIN`/`PULSE` transient spec, and asking for one is a parse error. Two things
  do work: a behavioral `B` source over `time` (e.g.
  `B1 n1 0 V={0.5+0.5*sin(6.2831853*5*time)}`), which pins the circuit to nodal
  full-LU and backward Euler; or `.runtime R` / `.runtime V` host-driven
  modulation, which keeps the normal routing. Prefer `.runtime` unless the
  modulator genuinely has to live inside the circuit.
- **Temperature sweep**: no `.temp` directive or global temperature sweep (device self-heating is available per-device, see Temperature Dependencies above)
- **Multi-language codegen**: C++ in progress; Python/NumPy and MATLAB targets
  planned. **FAUST was explored and determined impractical** — its generated code
  is intentionally not Turing-complete (it computes each sample in a fixed number
  of operations), so a Newton-Raphson solve whose iteration count depends on the
  data cannot be expressed. Only circuits that emit **no NR loop at all** would
  be expressible -- note that the predicate is "no NR loop", not `M == 0`, since
  a behavioral `B` source routes nodal and gets Newton regardless of M. That was
  6 of 41 corpus circuits: too small a subset to be worth a backend.
- **M > 24**: iterative/sparse NR for very large nonlinear systems (MAX_M=24)
- **Ideal transformer formulation**: dependent sources + explicit leakage/magnetizing L

## Validation

### Component Values

- Negative, zero, NaN, and Inf component values are rejected (returns error, does not panic)
- Self-connected components are rejected
- No warnings for extreme values (1e-300, 1e300)

### Circuit Topology

- **No check for floating nodes.** A node with no DC path to ground parses,
  builds, and compiles without complaint. Connectivity is not the parser's job
  and nothing downstream audits it; you find out only if the node actually makes
  the matrix singular, at which point the LU failure suggests checking for
  floating nodes
- No check for shorts across voltage sources
- Condition number warning when cond(A) > 1e13 (see Condition Number above)

### SPICE Validation Scope

- Circuits built from standard SPICE models (`D`, `NPN`/`PNP`, `NJF`/`PJF`,
  `NM`/`PM`) have a direct ngspice twin. Three melange-extended devices are
  *translated* into one: the triode and pentode (`VP`) become Koren B-source
  subcircuits, and the op-amp (`OA`) becomes the VCCS macromodel it already is
  internally — `G` for `AOL/ROUT` plus `R` for `ROUT`, with `RIN`/`IB` emitted
  only when melange itself stamps them. An op-amp run is REFUSED if the
  reference trace crosses a declared rail, because the linear stand-in cannot
  clamp and the two engines would be running different circuits from that
  sample on.
- `VCA`, `LDR` and `NEON` still have no oracle and are refused by `validate`;
  they are checked with `compile`/`analyze`/`simulate` instead.
- `melange validate` does not read the deck's `.oversampling` recommendation;
  the factor must be given explicitly as `--oversampling {1|2|4}` (default 1).
  `compile`/`simulate`/`analyze` honour the directive, `validate` reports what it
  was asked to measure. The ngspice reference is NOT filtered: it is aligned to
  the melange output by one best-fit constant delay (the same alignment every
  mode gets, 1x included), so the half-bands' frequency-dependent phase stays
  inside the number — it ships, so it is reported rather than compensated away.
  Measured on `tube_screamer_u` (48 kHz, 0.3 V, 500 ms): 1-rho 1.00e-6 at 1x,
  5.64e-6 at 2x, 6.25e-6 at 4x. See `docs/aidocs/OVERSAMPLING.md`
- Resistor `KF`/`AF` noise breaks ngspice parity (validate compiles with
  `NoiseMode::Off`, so it is simply absent from the comparison)
- `.mismatch` / `.tolerance` jitter is **disabled automatically** on melange's
  side for a validate run, which names the disabled directives and the
  unexercised seed on its PASSED/FAILED line. Nominal is compared against
  nominal; the draw itself is covered by unit tests, not by ngspice. No deck
  edit is needed. See `docs/aidocs/UNIT_VARIATION.md`
- SPICE correlation is **necessary but not sufficient** for promoting a circuit;
  a listening test is required on top

### Other Scope Restrictions

- **Multi-input decks** are restricted to linear (M=0) circuits, `--format code`,
  and `--oversampling 1`. Each is a hard error, not a warning: superposition
  across input ports is exact only when nothing nonlinear touches the inputs, so
  the CLI refuses rather than emit a silently wrong plugin
  (`tools/melange-cli/src/main.rs:1720-1745`). The consequence is that the
  nonlinear-mixing case the feature exists for is currently unreachable

### Parser Hardening

- Input caps: `MAX_NETLIST_BYTES` = 10 MB, `MAX_NODE_NAME_LEN` = 256,
  `MAX_TOTAL_ELEMENTS` = 50 000, `MAX_MODELS` = 1 000, subcircuit nesting depth 8
- Non-ASCII normalization: mu/micro mapped to `u`; other non-ASCII rejected
- Node names are case-folded to lowercase, and `gnd`/`ground` alias to node `0`
- cargo-fuzz target (`parse_netlist`) exercises parser through MNA through DK kernel

---

## Design Decisions

These are intentional trade-offs, not bugs:

1. **f64 everywhere**: Double precision for all calculations. No f32 optimization.

2. **Codegen-only pipeline**: Netlist to MNA to DK/Nodal kernel to optimized Rust source code. No interpreted runtime solver.

3. **Trapezoidal default, backward Euler where stability demands it**:
   trapezoidal for second-order accuracy. BE arrives four ways, and the
   `// provenance:` header's `integration_source` field says which:
   `explicit` (`--backward-euler` or `.integrator be`), `auto-promoted` (the trap
   propagation operator's spectral radius exceeds 1.002, or the BE-promotion
   discriminator fires), `behavioral` (forced by a `B` source), or `trap` for
   trapezoidal. `--force-trap` and `.integrator trap` opt out of auto-promotion.
   Nodal trapezoidal builds additionally carry a **runtime BE-latch**: if the
   solver falls into a self-sustaining Nyquist limit cycle at a large-signal
   operating point that the compile-time quiescent analysis cannot see, that
   instance latches to BE for the rest of the stream (cleared by `reset()`,
   counted in `diag_be_latch_count`).

4. **Const generic devices**: Device dimension at compile time for stack allocation. No heap in device models.

5. **No general SPICE export**: melange parses SPICE and emits DSP source code,
   not netlists. Two narrow exceptions exist and neither is a round-trip path:
   `melange import` writes a melange `.cir` from a KiCad netlist or schematic,
   and `melange validate` rewrites a deck for ngspice (VIN strip, PWL Thevenin
   inject, `.OPTIONS INTERP`, pentode model translation).

---

*Last updated: 2026-09-22. Audited against the source tree at
`0a3dacb` and `docs/aidocs/STATUS.md` (v0.1.8). Where this file and
`docs/aidocs/STATUS.md` disagree, STATUS.md is the maintained reference --
except for the performance figures, which were last re-measured here.*
