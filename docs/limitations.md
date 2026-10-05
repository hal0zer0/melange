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
| L | Inductor | `ISAT=` (with `LAIR=`/`CORE=`) for saturation: single inductors and two-winding shared cores (see Saturating Inductors) |
| V | Voltage source | DC value (+ optional AC mag/phase). A transient spec (`SIN`/`PULSE`/`PWL`/`EXP`/`SFFM`/`AM`) is a **hard parse error** — audio comes in through the input node, not a source |
| I | Current source | Same: DC only, transient specs rejected |
| D | Diode | Shockley + RS + BV/IBV (Zener) |
| Q | BJT (NPN/PNP) | Ebers-Moll and Gummel-Poon |
| J | JFET (NJF/PJF) | Shichman-Hodges; Parker–Skellern at `LEVEL=2` (trap dispersion and thermal reduction not implemented: those keys are refused) |
| M | MOSFET (NM/PM) | Level 1 SPICE with body effect (GAMMA/PHI) |
| T | Triode tube | Koren plate current + Dempwolf & Zölzer grid current |
| P | Pentode/beam tetrode | 5 equation families, 29 catalog models |
| U | Op-amp | Boyle VCCS macromodel (AOL, ROUT, VCC/VEE rails, SR); `GBW` is not modelled as a pole |
| Y | VCA | THAT 2180-style exponential gain |
| E | VCVS | Voltage-controlled voltage source |
| G | VCCS | Voltage-controlled current source |
| K | Coupled inductors | Transformers (multi-winding supported) |
| O | LDR / photoresistor | `CdsLdr` model, `.model NAME LDR()` |
| N | Glow discharge / neon lamp | **EXPERIMENTAL.** `.model NAME NEON(...)`; the `N` letter and the `NEON` model type are provisional and may change |
| B | Behavioral source | `V={expr}` / `I={expr}`, nodal path only -- see below |
| X | Subcircuit instance | Recursive expansion, max nesting depth 8 (`MAX_NESTING_DEPTH` in `Netlist::expand_subcircuits`, `crates/melange-solver/src/parser/subckt.rs`) |

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
  would silently drop the nonlinearity (the `--nodal-subpath` override in
  `emit_nodal`, `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter/mod.rs`).
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

Temperature coefficients (TC1, TC2) on resistors are not modelled, and there is no global temperature (`.temp` is ignored with a warning). Device temperature is set per `.model` instead: on a **diode or BJT** card, `TAMB` (in kelvin; default 300.15 K, SPICE's TNOM of 27 °C) moves the device off TNOM with the SPICE3 law — IS through `XTI` and `EG`, the thermal voltage in proportion, and on a BJT BF/BR through `XTB` and ISE/ISC through both. TNOM itself is fixed at 27 °C. Device self-heating (RTH, CTH) is available via a quasi-static thermal RC model with SPICE3f5 IS(T) scaling for **diodes, BJTs, and triodes** (default disabled, RTH = infinity → dead code). JFETs, MOSFETs and tubes without self-heating run at 27 °C.

Separately, the authentic-noise feature carries a runtime-settable noise temperature (`set_temperature_k`, default 290 K), which scales only thermal noise — it does not affect the deterministic device equations. See the Circuit Noise section below.

## Device Model Limitations

### Diode
- BV/IBV: hard clamp reverse breakdown (no smooth Zener knee)
- Junction capacitance `CJO` is a constant capacitor at its zero-bias value; it
  does not vary with bias
- No TC1/TC2 temperature coefficients; optional quasi-static self-heating (RTH/CTH/XTI/EG/TAMB), disabled by default

### BJT (Gummel-Poon)
- Q1 Early effect guard: `q1_denom <= 0` clamps to 1.0 (physically near Early voltage limit)
- Self-heating (RTH/CTH) available but disabled by default (RTH=infinity)
- Junction capacitances (CJE/CJC) and diffusion capacitance (TF) available,
  linearized once at the DC operating point: a large signal does not move them
- Parasitic resistances (RB/RC/RE) supported with internal nodes
- No substrate current or avalanche breakdown

### JFET / MOSFET
- Card series resistance `RD=`/`RS=` is refused when nonzero: the solution
  has no internal drain/source nodes to put it on. Model it as an explicit
  resistor in series with the drain or source, which matches ngspice. Internal
  drain/source nodes are queued.
- JFET gate junctions follow SPICE's level-1 diodes (`IS`, `N`; ngspice
  defaults), without SPICE's 1e-12 S GMIN across them, so ngspice agreement is
  measured with its GMIN off. `IS` is not temperature-scaled (a JFET card has
  no TAMB/XTI/EG).
- Subthreshold: hardcoded 2xVT slope (real devices: 60-120 mV/decade)
- MOSFET Level 1 only (no BSIM3/4)

### Triode
- Koren model with lambda for finite plate resistance
- No space-charge or transit-time effects
- **Grid current FAILS its acceptance test against a real datasheet.** Grid
  current follows Dempwolf & Zolzer eq. (11). Measured against the Philips
  ECC83 (January 1970) "As A.F. amplifier" block, row *Output voltage
  (Ig = 0.3 uA)*, it fails **15 of 15 cells** (3 D&Z Table 1 rows x 5 supply
  voltages): the modelled grid-current onset is **0.26-0.35 V too late**
  (-0.26..-0.35 V against the -0.61 V implied by the sheet's own five columns,
  which agree to sd 0.067 V across a 2x range of Vb and 2.75x of Rk). The
  previous hard-zero law was 0.61 V late, in the same direction.

  **Consequence:** stages driven from a high source impedance show grid-current
  loading *later* than a real ECC83, so maximum clean output is overpredicted
  (1.16x-1.55x on this block). This is not an inaudible tail — 0.3 uA into a
  following stage's 680 kOhm grid leak is ~0.2 V of bias shift, which is where
  blocking and bias-shift distortion begin in cascaded stages.

  The shipped Koren ECC83 plate card independently **over-compresses 1.3x-2.3x**
  at the same operating points (and runs Ia 2-10% low), which partly *masks*
  this at the output — the grid-side error is larger than an output-voltage
  comparison alone shows. Any future grid-current check at these points must
  report the plate model's share, because the two errors partly cancel.

  **Root cause: the tail's SLOPE, not its magnitude.** For `Vgk << 0` eq. (11)
  tends to a pure exponential of slope `xi*Cg` = 13.0 /V, an *effective* cathode
  temperature near 890 K. The physical retarding-field law (Maxwellian emission)
  has slope `e/kT_k` — 7.7 /V at 1500 K, ~10.6-11 /V at 1050-1100 K. eq. (11) is
  steeper than any physical thermal slope, so a fit made in the mA region falls
  away too fast when extrapolated three to four decades down to 0.3 uA.

  Note also that a datasheet grid current is a NET reading — electron current
  minus gas ionisation, grid primary emission and leakage — while eq. (11)
  models the electron term alone. The gap above is therefore a lower bound.

  Closing the gap by refitting `Gg` would need 28.7x-257x, where D&Z's own
  three tubes span 1.89x, so this is not a tube-to-tube parameter spread.
  No parameter here is fitted to that sheet, which is what keeps it usable as
  an out-of-sample check.

### Pentode
- 5 equation families: Rational (Derk), Exponential (DerkE), Classical (Koren/Cohen-Helie), plus variable-mu variants
- 29 catalog models (EL84, EL34, EF86, 6L6, 6V6, KT88, 6550, 6K7, EF89, and more)
- Grid-off dimension reduction (3D to 2D) exists but is **opt-in only**:
  `--tube-grid-fa on`. `auto` behaves as `off` and keeps the
  full 3D model, because the reduction is **not** accuracy-neutral -- it drops
  the cathode/screen-referenced Vg2k feedback (measured +2% to +12% small-signal
  gain error on cathode-biased stages) and all grid current for Vgk > 0. `on`
  warns per device. An exact reduction is deferred; until one exists, `auto`
  will not reduce.
- No independent suppressor dynamics: the suppressor is modelled as cathode-tied, and a 5th node on anything but the cathode is refused
- 6386/6BA6/6BC8 datasheet fits for variable-mu compressors deferred (phase 1d)

### Op-amp
- Boyle VCCS macromodel **without a bandwidth pole**: the gain is `AOL` at every frequency. `GBW` is parsed and prints a notice; its only effect is the default ±13 V rails when `VCC`/`VEE`/`VSAT` are absent. A circuit whose response depends on the op-amp's open-loop rolloff (high closed-loop gain near the top of the audio band, or a low-GBW part) will read flat where the hardware rolls off.
- VCC/VEE asymmetric supply rail clamping
- Slew-rate limiting via `SR=` in V/us (per-sample clamp, all 3 codegen paths)
- Rail mode selection: `--opamp-rail-mode {auto,none,hard,active-set,active-set-be,boyle-diodes}`
- `auto` resolves only to `none` / `hard` / `active-set`: `active-set` for any
  op-amp whose output is capacitor-coupled downstream, which pins the railed
  output and re-solves the circuit on the build's own integrator (a pin or
  release takes no backward-Euler sample; see `docs/aidocs/OPAMP_RAIL_MODES.md`).
  It **never** selects
  `active-set-be` (backward Euler on every rail-engaged sample: 2-4x the
  output-peak error on a railing overdrive) or `boyle-diodes` (validated for
  light clip, diverges at heavy clip); both stay opt-in (see
  `docs/aidocs/OPAMP_RAIL_MODES.md`)
- A railing op-amp switches rail to rail within a sample. Without oversampling,
  top-octave drive aliases: a 16 kHz tone into a single-supply overdrive at
  48 kHz, 1x, puts 0.44 V at 66 Hz on the output, gone at 4x (0.5 mV).
  `active-set-be` shows about half of it at 1x because backward Euler damps the
  edges, not because it is more accurate; oversample railing circuits
- An explicit `--opamp-rail-mode` is honoured verbatim and is never silently
  upgraded, including `none` on a circuit `auto` would have clamped

### VCA
- 2D current-mode exponential gain (THAT 2180 / DBX 2150)
- THD: gain-dependent cubic nonlinearity
- `noise_floor` field exists but unused

### Saturating Inductors
- `L1 a b 100m ISAT=20m LAIR=3e-4`. Anhysteretic saturating flux
  `Φ(i) = L_mag·Isat·tanh(i/Isat) + L_air·i`, with `L_air = LAIR·L0` and
  `L_mag = L0 − L_air`: the incremental inductance
  `L_diff(i) = L_mag/cosh²(i/Isat) + L_air` falls from L0 toward the core's
  air-core value instead of toward zero. It is solved inside the Newton loop
  with `L_diff` in the Jacobian and the flux law checked as a residual at every
  Newton site.
- **The air-core floor.** `LAIR=` gives it as a fraction of L0 (a measured
  saturated-to-unsaturated inductance ratio is best); `CORE=gapped|steel|nickel`
  picks a rule-of-thumb class value (1e-3, 3e-4, 3e-5). A deck with neither gets
  3e-4 (ungapped steel) and a notice. `LAIR=0` is accepted, with a notice: the
  pure tanh law's slope goes to zero, and driven far past `ISAT` the current is
  then limited by a numerical floor, not a physical one.
- **Transformers saturate as a shared core**: `ISAT=` on a winding of a
  coupled group puts the saturation on one magnetizing branch behind ideal
  couplings (the load current's flux cancels, as in real iron), with linear
  leakage `L − LM·n nᵀ`, a full matrix when windings share leakage. Two
  windings may leave the core implicit; three or more state it with `TURNS=`
  on every winding and `LM=` on one, which also declares that every winding
  links the one core loop. Refused, not approximated: coupling k ≤ 0.8 (no
  shared core), three or more windings with no stated core, a stated core
  whose leakage is not positive-definite, an incomplete or repeated
  statement, and conflicting `ISAT=` values. `CORE=` (and the default) is the
  core's magnetizing air floor, while an authored `LAIR=` is the winding's
  total air-core self-inductance including its leakage; an authored `LAIR`
  that leaves no floor is refused, and so are declarations whose implied
  floors are more than 3× apart (within that the least is used, with a
  notice).
- **Not modeled: a core with more than one flux path** (three-phase or
  multi-leg cores, a winding on an outer leg of an EI) needs a reluctance
  network with several saturating branches. melange has one core loop per
  group, which `LM=` declares.
- A circuit with a saturating inductor runs on the **nodal full-LU** sub-path;
  `--nodal-subpath schur` is refused.
- `.switch` cannot change a saturating inductor or any winding of a
  saturating core (refused): the flux law is fixed at compile time.
- **Deep saturation.** With the floor, a core driven to tens of times `ISAT`
  settles on its resistive-plus-air-core limit: a 1 H, 10 mA core behind 100 Ω
  driven at 20 V peaks at V/R with no ring at 1×.
- **Accuracy near the knee at a short L/R.** Just past the knee (a few times
  `ISAT`) the tanh slope, not the floor, sets `L_diff`, and with L/R of a few
  samples the trapezoidal rule can overshoot the inductor current while the
  output spectrum stays close. A railing op-amp driving a gapped choke at about
  2.7× `ISAT`: at 4× oversampling, inductor current within 1.3 % and output
  fundamental within 0.1 % of ngspice; at 1×, inductor current up to 13 % over,
  output fundamental within 0.21 %. The overshoot is born where the core
  crosses its knee within one sample with the full rail across it; compile
  prints a notice for an op-amp that can rail into a saturating inductor
  below 4× oversampling. Where a ring dominates the output, the
  runtime backward-Euler latch catches it and holds the instance on BE for the
  rest of the stream.
- Checked against independent references of the same law (a scalar
  trapezoidal recurrence at 1× and 256-1024×, exact flux-drive harmonic tables
  for a DC-biased core) and one ngspice twin (the railing op-amp into a choke).
- No magnetic hysteresis, core loss, or remanence anywhere in the code

### Glow Discharge / Neon (`N`) [EXPERIMENTAL]
- `N1 anode cathode MODEL` + `.model MODEL NEON(VO VM IK RS IHOLD ROFF)`. The
  element letter and model type are explicitly provisional
  (`Element::Glow` in `crates/melange-solver/src/parser/element.rs`)
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

### Newton-Raphson failures: what is refused and what is not

Newton-Raphson has a per-sample iteration budget (`--max-iter`; by default an
auto-tuned value that scales with M, the solver route and the trapezoidal
spectral radius, raised to 100 on a nodal build; an explicit value pins it,
and a nodal build refuses a pin below 100, because its Armijo-globalized
Newton needs that headroom to cross a saturation knee within a sample; the
generated file's `Build:` line and `-v` show the budget the code runs). A
sample that exhausts it is retried: on the nodal routes by a local sub-step
ladder (down to T/2^12, at most 64 attempts), then, on a trapezoidal build,
by a backward-Euler solve. A sample one of those retries solves is a converged
solution by another consistent scheme; it is first-order
on that sample when backward Euler solved it, and the sub-step ladder has no
truncation-error control.

A sample that no path solves is **unsolved**: the solver commits the previous
state (or, where the final solve is an op-amp pin or the DK solve, the
unconverged iterate), and counts it in `diag_unsolved_sample_count`. A held
value is bounded and smooth, so the rendered audio cannot show it.
`melange simulate`, `validate` and the golden harness **refuse** a render with
any unsolved sample, and `melange analyze` refuses each sweep point (and the
zero-drive settle before the first) that has one, naming the point and the
counter (`--allow-nr-hold` on `simulate` and `analyze` reports it anyway). A
compiled plugin has no such gate: the counters are public fields on the generated state, and
a plugin that does not check them will not notice.

`simulate` also warns when more than 20 % of internal samples hit the
iteration ceiling, rescued or not.

**What no counter catches [OPEN].** A raised `--max-iter` on an oscillator or
switching circuit can let Newton settle on a spurious oscillation of the
discrete step equations with every sample "solved" (see the astable section
below). And on a free-running oscillator, a budget that starves Newton on some
samples can shift its period with no unsolved sample (3.8 % flat at
`--max-iter 70` against `--max-iter 1000` on a germanium-PNP divider astable,
measured before the local sub-step ladder existed and not re-measured since; a
nodal build refuses a pin below 100).
Compare an oscillator's period at two budgets before trusting it.

### Conductance swaps (`.switch`, `.pot`, `.runtime R`)

Changing a conductance mid-run changes `A = G + (2/T)C` only. The generated
integrator's history is `(2/T)C·v_prev + q_dot` (the charge form, see
[COMPANION_MODELS.md](aidocs/COMPANION_MODELS.md)), which carries no
conductance, so a pot move or a resistor-only switch flip is exact on the next
sample: a resistive divider node obeys `node − ratio·other` to below 1e-9 at
and after the flip. A switch that swaps a capacitor or an inductor solves its
next sample on backward Euler (breakpoint-BE), because the carried capacitor
currents were built on the old value.

### Self-starting two-transistor astables [OPEN]

A classic two-transistor astable multivibrator whose DC operating point is
unstable (it starts by itself) is built on the nodal solver, and melange
cannot yet reliably solve its regenerative switching edges. The witness is a
textbook NPN astable (9 V, 1 kΩ collectors, 47 kΩ bases, 100 nF
cross-coupling, NPN `IS=1e-14 BF=100 CJE=10p CJC=4p TF=0.3n`), which ngspice
runs at a converged 6.667 ms period with the collectors inside 0–9 V.

- **At the default Newton budget the render is refused, and that is the
  supported outcome today.** At the first edge every Newton path fails, under
  trapezoidal integration or `--backward-euler`, and every later sample is
  refused as unsolved: the render stops with an error (`--allow-nr-hold` writes
  the frozen output anyway).
- **Raising `--max-iter` is not a way past that refusal.**
  - On the witness, `--max-iter 1000` solves every sample and exits cleanly.
  - The result is a spurious oscillation 4–6 samples long (a 0.08–0.13 ms
    period, a collector reaching 10.7–12.1 V on the 9 V supply).
  - It is the same on both nodal sub-paths: a genuine solution of the discrete
    step equations, not of the circuit, and no counter shows it.
  - It appears with the transit-time (`TF`) diffusion capacitance. With the
    junction capacitances alone (`CJE`/`CJC`) the same cycle still leaves some
    samples unsolved, so the render is refused.
  - The same astable with no junction or transit-time capacitance does solve
    correctly at `--max-iter 1000` (6.687 ms against ngspice's 6.693 ms). There
    is no way to tell from the output which case you are in.
  - (A `TF`-only variant has no settled reference: ngspice's own period on it
    ranges from 0.008 to 6.5 ms with its step and tolerance settings. The claim
    rests on the full witness.)
- It is built on nodal because its DC operating point has a growing pole,
  which the DK route refuses (a self-starting oscillator); DK has no
  unsolved-sample containment at a switching edge.

Circuits whose devices carry series resistance and Early effect under moderate
bias (a PNP divider astable with RB/RC/RE/VAF, for one) cross their edges and
oscillate correctly. Tracked as (iv) in `docs/aidocs/STATUS.md`.

### Device Linearization
- `.linearize Q9` or `.linearize T1` removes a BJT or **triode** from the NR
  system. Only those two element kinds are eligible; any other name is
  refused. So is a device already outside its small-signal region at its own
  operating point (a BJT saturated or cut off, a triode cut off or with its
  grid past the conduction onset), with the operating-point evidence
- Replaced with small-signal conductances at DC operating point
- Reduces nonlinear dimension M (BJT: M-2, triode: M-2 per device)
- A linearized BJT is stamped as its own terminal-current Jacobian at the DC
  operating point: NF/NR, Gummel-Poon Early effect and high injection, ISE/ISC
  leakage and RB/RC/RE all enter it. A triode is stamped as g_m and 1/r_p
- A linearized triode keeps its inter-electrode capacitances (CCG, CGP, CCP).
- A linearized device driven out of its small-signal region (triode cut off
  or grid past its conduction onset; BJT cut off or saturated) makes the
  sample unsolved, and every verb refuses the render. melange does not fall
  back to the full device for those samples.
- A linearized BJT's junction and diffusion capacitances (CJE, CJC, TF) are
  evaluated at the DC operating point, as the unlinearized device's are, and
  stamped between its external terminals. A card with RB/RC/RE on a route
  that expands internal nodes places the unlinearized device's caps on the
  internal nodes instead; the ohmic resistance in series puts that
  difference's pole in the MHz range

## Numerical Limitations

### Matrix Storage [PERFORMANCE]

Matrices use `Vec<Vec<f64>>` (jagged arrays) instead of flat storage. This has poor cache locality but is acceptable for typical circuits (validated up to N=64).

### Denormal Handling

Generated code flushes denormals in the state vectors (`v_prev`, and `i_nl_prev`
when M > 0) once per sample with an add/subtract of `1e-25`, on **both** the DK
and nodal paths. The charge-form `q_dot` vector is flushed with them. Most DAW hosts set FTZ/DAZ, but the generated code does not rely
on it. The DC-blocking feedback path carries a tiny bias for the same reason.

This covers `v_prev` and `i_nl_prev`. It is not a global FPU mode
change -- an intermediate inside one sample's solve can still go denormal.

### Condition Number [NUMERICAL]

Condition number is estimated during DK kernel build as
`||A||_inf · ||A^-1||_inf`. A `log::warn!` fires above **1e13**
(`DkKernel::from_mna` / `from_mna_augmented`, `crates/melange-solver/src/dk.rs`),
because high conditioning is common and usually benign (tight component-value
spreads, near-unity transformer coupling). Ill-conditioned circuits still produce
results, possibly with reduced accuracy. A genuinely ill-conditioned `K` or `S`
is handled separately by routing (below), not by this warning.

### Nonlinear System Size

Codegen supports up to M=32 nonlinear device dimensions on every route
(`MAX_M`, `crates/melange-solver/src/dk.rs`). M=1 is solved directly, M=2 by
Cramer's rule, M=3..32 by Gaussian elimination with partial pivoting on a
block-diagonal Jacobian, emitted fully unrolled. The bound is on generated code
size and compile time, not accuracy: at M=32 the generated source is 0.5 to
0.9 MB and takes 0.7 to 5.7 s to compile with `rustc -O`, depending on the
route.

Three mechanisms reduce M, and none is on by default:

- **BJT forward-active detection** (`--bjt-fa {off,auto,force}`, default `off`).
  `auto` reduces pure Ebers-Moll BJTs that are forward-active at the DC
  operating point, where the 1-D model is exact until the device saturates;
  a sample on which a reduced BJT saturates is counted unsolved and refused.
  Gummel-Poon / ISE / self-heating / parasitic BJTs stay full 2-D. `force`
  reduces them too, per-device warned, at a documented accuracy cost (drops the
  `qb` base-charge term).
- **`.linearize`** (explicit, per device).
- **Pentode grid-off** (`--tube-grid-fa on` only -- `auto` does not reduce; see
  the Pentode section).

## Solver Limitations

### Voltage Sources

Independent voltage sources (V elements) use **augmented MNA**: each source adds a branch-current unknown plus a KVL constraint row (`B^T · x = v_dc`). There is no high-conductance Norton stamp. See `VoltageSourceInfo` in `crates/melange-solver/src/mna.rs` and `solve_dc_op` in `crates/melange-solver/src/dc_op.rs`.

The **audio input** is a separate mechanism and does not go through that path: it is a Thevenin source stamped as a conductance `G_in` at the input node (default 1 Ω, or the `.input_impedance` directive / `--input-resistance` override), entering the RHS once per sample as `V_in(n+1) · G_in`; the capacitor history is carried by `q_dot` (see [COMPANION_MODELS.md](aidocs/COMPANION_MODELS.md)).

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
| Trapezoidal instability | spectral radius of the whole-system operator `S·((2/T)C - G)` > 1.002 |
| Positive feedback in the Schur Newton | a non-negative `K` diagonal with a live `N_i` column (e.g. transformer-coupled negative feedback) |
| `K` ill-conditioned | max\|K\| > 1e8 (`K_ILL_COND_MAX`) |
| `S` ill-conditioned | max\|S\| > 1e6 (`S_ILL_COND_MAX`) |
| Behavioral `B` source | structural -- DK cannot stamp in node space |
| Saturating inductor (`ISAT=`) | structural -- the flux law is solved by Newton on the inductor's augmented row each sample; DK bakes `S = A^-1` and has no such row |
| Op-amp rail mode `active-set`/`active-set-be`/`boyle-diodes`, or an `AOL_TRANSIENT_CAP` card | structural -- only nodal pins a railed output and re-solves, builds the Boyle internal node, or applies the cap |
| Self-starting oscillator | the DC operating point has a growing pole; a DK build of it is refused and the auto route rebuilds on nodal |

The routing estimate of trapezoidal instability uses the whole-system operator;
it decides only the route. Whether a build integrates with backward Euler is
decided separately, by the ring predicate (see Design Decisions below).

Multiple transformer groups, the positive-feedback `K` diagonal, and every
structural row are hard requirements: `--solver dk` is **rejected**, not
downgraded (`forced_dk_hard_blocker`, `crates/melange-solver/src/build.rs`).

**Stage 2 -- nodal Schur vs nodal full LU**
(`emit_nodal`, `crates/melange-solver/src/codegen/rust_emitter/nodal_emitter/mod.rs`).
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

**Schur on expanded parasitic internal nodes works harder than full LU on one
deck.** Every nodal build expands the parasitic-BJT internal nodes. Auto routes
pipe-shouter and the wurli preamp to Schur that way, with no unsolved sample.
Forced onto Schur (`--nodal-subpath schur`), the wurli power amp holds no sample
either but hits the Newton ceiling 749 times a second at 1 V drive (rescued by
sub-stepping), where full LU, which auto picks for it, hits it never. Measured
2026-09-29.

## Real-Time Constraints

### Allocation

Generated code pre-allocates all buffers in `CircuitState`. No heap allocation occurs in `process_sample()`.

### Worst-Case Performance

Matrix recomputation is O(N^3) and occurs at:
- `set_sample_rate()` calls
- `set_pot_N()` / `set_switch_N()` / `set_runtime_R_<field>()` calls (per-block,
  when the value actually changes; batched into one rebuild per sample via a
  `matrices_dirty` flag on the nodal path)

There is no current benchmark for pot-rebuild latency. If you need this
figure, measure it on your own target with `tools/perf-harness/bench.sh`.

`set_sample_rate()` cannot change the *route*. Stage-1/stage-2 routing and
sub-sample-fire activation are compile-time structural decisions, so a plugin
that must run at several host rates has to be compiled per rate.

### Performance Benchmarks

Measured 2026-10-01 on an idle AMD Ryzen 9 7950X pinned to one CCD, single
core, noiseless, `-C target-cpu=x86-64-v3` (best of 7 × 2M samples via
`tools/perf-harness/bench.sh`); throughput is host-dependent.

- Light nonlinear circuits: 12AX7 gain stage ~150× realtime
- Germanium diode network (3 RC sections, an antiparallel Ge pair at each) ~24.6× realtime
- Typical multi-device circuits: Wurlitzer preamp ~45×, single-ended tube amp ~20× realtime
- Heaviest measured: a passive tube EQ (nodal Schur, N=52, M=8) ~18.1×, a bus compressor (12 op-amps + 2 VCAs) ~6.5× realtime

The triode rows include the cost of the Dempwolf & Zölzer grid-current law,
which evaluates a softplus on the grid dimension at every Newton iteration
(14–29 % on these three rows, measured with and without it on the same host).

Every figure above names the circuit it came from, deliberately: a row nobody
can map to a circuit is a row nobody can check. Treat any ×-realtime number
without an attributed circuit and a named host as unverified.

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
- Op-amp `EN_FC`/`IN_FC` (1/f corner) are accepted with a compile notice but **not modelled** — Phase 4 is white-band only
- BJT parasitic RB/RC/RE thermal noise (rbb′) is skipped (logged as a `warn!`) on the **DK codegen path**, which keeps RB/RC/RE inside the device model. Every nodal build expands the BJT internal nodes and includes it
- Diode `RS` and tube `RGI` parasitic resistances are not yet thermal-noise sources
- Resistor `KF`/`AF` need no stripping before SPICE-validating: `melange validate` strips them from the ngspice reference (validate renders noise-free on both sides). Likewise `.mismatch`/`.tolerance` jitter: `melange validate` disables it on melange's side automatically and says so on the result line
- **Tube microphonics** (Phase 6) is research only, not implemented

## Not Implemented [DEFERRED]

- **LFO/Modulation**: independent `V`/`I` sources are DC only -- there is no
  `SIN`/`PULSE` transient spec, and asking for one is a parse error. Two things
  do work: a behavioral `B` source over `time` (e.g.
  `B1 n1 0 V={0.5+0.5*sin(6.2831853*5*time)}`), which pins the circuit to nodal
  full-LU and backward Euler; or `.runtime R` / `.runtime V` host-driven
  modulation, which keeps the normal routing. Prefer `.runtime` unless the
  modulator genuinely has to live inside the circuit.
- **Temperature sweep**: no `.temp` directive or global temperature sweep (device temperature `TAMB` and self-heating are per-`.model`, see Temperature Dependencies above)
- **Multi-language codegen**: only Rust is emitted. A C++ backend is the next
  planned target; Python/NumPy and MATLAB are planned after it. **FAUST was explored and determined impractical** — its generated code
  is intentionally not Turing-complete (it computes each sample in a fixed number
  of operations), so a Newton-Raphson solve whose iteration count depends on the
  data cannot be expressed. Only circuits that emit **no NR loop at all** would
  be expressible -- note that the predicate is "no NR loop", not `M == 0`, since
  a behavioral `B` source routes nodal and gets Newton regardless of M. That was
  6 of 41 corpus circuits: too small a subset to be worth a backend.
- **M > 32**: a loop-based elimination (or iterative/sparse NR) for very large nonlinear systems (MAX_M=32)
- **Ideal-transformer formulation for linear windings**: linear coupled
  inductors use the exact `[L]` coupled-inductor path. The ideal-transformer
  T-model (ideal couplings + leakage + one magnetizing branch) is built only for
  saturating cores, on nodal full LU

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
- A single saturating inductor (`ISAT=`) is translated into its own flux law
  with the current as a state (`dx/dt = v / (L_mag/cosh²(x/ISAT) + L_air)`,
  branch current `x`), from melange's resolved `ISAT` and floor. A **K-coupled**
  saturating core has no ngspice twin here: the reference models that core as
  LINEAR and `validate` says so, so a difference past the knee is not a melange
  error.
- The behavioral functions ngspice's `B` source lacks are rewritten: `idt(x)`
  becomes an integrator node (unit capacitor from 0, 1e15 Ω leak) and
  `atan2(y, x)` a quadrant-correct `atan`. Runtime scalars (`.runtime`) enter
  the reference at their declared minimum.
- Variable-mu pentodes (`SVAR > 0`) have no ngspice twin yet and are refused
  by `validate` rather than compared as sharp pentodes.
- `VCA`, `LDR` and `NEON` still have no oracle and are refused by `validate`;
  they are checked with `compile`/`analyze`/`simulate` instead.
- A runtime-selectable oversampling set (`.oversampling N allow=...`) is refused
  when its factors would not build one solver (the route, integrator or runtime
  latch differs between factors, as on decks the ring predicate promotes to
  backward Euler at 1× only), when a factor-dependent constant is not one the
  runtime code switches (today: time-dependent behavioral sources), or when the
  generated code itself differs by factor (a coupling term below the 1e-20
  sparsity threshold at one rate is omitted from that rate's code). Build such
  a deck once per factor. `validate`, `simulate`, `analyze` and `dc-op` build
  the default factor fixed.
- `melange validate` does not read the deck's `.oversampling` recommendation;
  the factor must be given explicitly as `--oversampling {1|2|4}` (default 1).
  `compile`/`simulate`/`analyze` honour the directive, `validate` reports what it
  was asked to measure. The ngspice reference is NOT filtered: it is aligned to
  the melange output by one best-fit constant delay (the same alignment every
  mode gets, 1x included), so the half-bands' frequency-dependent phase stays
  inside the number — it ships, so it is reported rather than compensated away.
  Measured on `overdrive_pedal_native_u` (48 kHz, 0.3 V, 500 ms): 1-rho 1.00e-6 at 1x,
  5.64e-6 at 2x, 6.25e-6 at 4x. See [OVERSAMPLING.md](OVERSAMPLING.md) for
  what that means in practice, and `docs/aidocs/OVERSAMPLING.md` for the filter
  internals.
- Resistor `KF`/`AF` (flicker noise) are stripped from the ngspice reference,
  which rejects them, with a notice; validate renders noise-free on both sides,
  so the comparison is unchanged. No deck edit is needed
- `.mismatch` / `.tolerance` jitter is **disabled automatically** on melange's
  side for a validate run, which names the disabled directives and the
  unexercised seed on its PASSED/FAILED line. Nominal is compared against
  nominal; the draw itself is covered by unit tests, not by ngspice. No deck
  edit is needed. See `docs/aidocs/UNIT_VARIATION.md`
- The parameter check is one-sided. `validate` refuses a deck when ngspice
  reports a `.model` parameter it ignores, but a parameter **melange** ignores
  (it warns at build time, e.g. BJT `TR`, reverse transit time) does not stop
  the run: ngspice models it and melange does not, so the two engines compare
  different circuits. Read the build warnings before reading the number.
- SPICE correlation is **necessary but not sufficient** for promoting a circuit;
  a listening test is required on top

### Other Scope Restrictions

- **Multi-input decks** are restricted to linear (M=0) circuits, `--format code`,
  `--oversampling 1`, and no `--emit-dc-op-recompute`. Each is a hard error, not
  a warning: superposition across input ports is exact only when nothing
  nonlinear touches the inputs, so the build refuses rather than emit a silently
  wrong plugin (`assemble`, `crates/melange-solver/src/build.rs`). The consequence is that the
  nonlinear-mixing case the feature exists for is currently unreachable

### `melange analyze` measurement limits

- **Scope.** `analyze` characterises the circuit's response to its own
  stimulus (gain, phase, THD against frequency). For instrument measurements
  (aliasing, loudness, IMD, decay, noise floor) use a bench instrument.
- **Steady state is judged by agreement, not proven.** Each point is re-measured
  every `--preroll-secs` (default 0.25 s) until two measurements agree within
  0.1 %. A time constant many times longer than that spacing can move less
  than 0.1 % between two checks while still far from settled; for such a
  circuit raise `--preroll-secs` (and `--preroll-max-secs`, default 2 s).
- A circuit with no steady state at a point (a free-running oscillator, a
  drive that keeps a rail moving) reaches the cap: the point is named in a
  warning and its row is the last measurement. With `--noise` the check is
  off and every point gets the fixed pre-roll only.
- `thd_pct` is the audio-band figure: H2..HN below 20 kHz and below Nyquist,
  `nan` when no harmonic is in that band. It is not an aliasing measurement,
  and neither is `nyquist_dbc`, which sees only a component at exactly half
  the sample rate (a limit-cycle signature). See
  [OVERSAMPLING.md](OVERSAMPLING.md) for how to measure aliasing.
- A dBc value below −200 dBc prints as `-inf`: at that depth the figure is
  floating-point residue, not circuit content.

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
   `explicit` (`--backward-euler` or `.integrator be`), `auto-promoted` (the
   ring predicate fires: the trapezoidal charge propagator, linearised at the DC
   operating point, has a growing pole that backward Euler removes, or a lasting,
   loud Nyquist-side ring that costs more than backward Euler's own in-band
   error; see `docs/aidocs/RING_PREDICATE.md`), `behavioral` (forced by a `B`
   source), or `trap` for trapezoidal. `--force-trap` and `.integrator trap` opt out of auto-promotion.
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

*Where this file and `docs/aidocs/STATUS.md` disagree, STATUS.md is the
maintained reference -- except for the performance figures, which were last
re-measured here.*
