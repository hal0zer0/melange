# SPICE Validation Protocol

## Purpose
Verify melange solver matches ngspice output within tight tolerances.

## Expected Correlation Benchmarks

Measured 2026-07-18 (HEAD b421358, ngspice-42, reltol=1e-4 reference):

| Circuit Type | Correlation | RMS Error | Notes |
|--------------|-------------|-----------|-------|
| Linear (RC, RL) | > 0.999999 (6 nines) | < 0.1% | Should match almost exactly |
| Nonlinear (diodes) | > 0.999999 (6 nines) | < 0.15% | Includes off-nominal `.pot` positions |
| BJT circuits | > 0.9996 | < 5% | BJT CE is the loosest (GP model gain ratio 1.024); the Wurlitzer and 3-BJT preamp decks are at 0.03-0.25% |
| Op-amp (linear) | ~1.0 | ~0% | VCCS macromodel matches ngspice |
| JFET circuits | > 0.9994 | < 4% | Shichman-Hodges 2D, ngspice BETA→IDSS |
| MOSFET circuits | > 0.99999 | < 0.1% | Level 1 SPICE |
| Audio-rate `.pot` R(t) | > 0.9999 | < 2% | vs native ngspice B-source; residual is per-sample ZOH of R(t) |

Every test that needs ngspice is `#[ignore]`d, so a plain `cargo test` (CI's
Test job, which has no ngspice) skips it instead of passing it vacuously. They
run with `cargo test -p melange-validate --test spice_validation --test
rate_sweep_tests --test thermal_twin_tests --test parasitic_twin_tests --test
linearize_twin_tests -- --include-ignored` (ngspice on `PATH`), which is what
CI's SPICE job runs; the twin targets carry `#[ignore = "requires ngspice"]`
and assert ngspice is present when run. Each `spice_validation` test calls
`run_validation()` → `run_melange_codegen()` in
`crates/melange-validate/tests/spice_validation.rs`, which delegates to
`melange_validate::run_melange_solver_from_str` — the same
`melange_solver::build::build` every CLI verb uses — then
`run_generated_solver` writes the code to the temp directory, compiles it with
`rustc --edition=2024 -O`, pipes samples through stdin/stdout, and removes
both files on every path. The comparison is against ngspice with `.OPTIONS
INTERP` for sample-aligned output. See `docs/aidocs/STATUS.md` for the latest
recorded correlation/RMS values.

## ngspice Setup for Sample-Accurate Comparison

### What the harness does automatically (`spice_runner.rs`)

1. **Injects `.OPTIONS INTERP reltol=1e-4`** right after the title line.
   INTERP forces ngspice to interpolate output at uniform timesteps instead
   of printing adaptive timestep points; reltol=1e-4 (vs the 1e-3 default)
   tightens the reference's own truncation error (measured 2026-07-18:
   every suite correlation held or improved). Deck-author `.OPTIONS` lines
   are KEPT — ngspice merges multiple `.OPTIONS` statements, and author
   options appearing later override the injected ones for the same keyword.

2. **Replaces `.TRAN`** with `tstep = 1.0 / sample_rate` (e.g., 2.083e-5
   for 48 kHz), the tstop derived from the input signal, and a maximum
   internal step `TMAX` (`ReferenceStep`, starting at `tstep/16`).

2b. **Shows the reference is converged before grading against it**
   (`reference.rs`, `converged_reference`). The reference is a numerical
   solution too. ngspice's default `TMAX` is the output step, and a smooth
   analytic drive has no breakpoints to shorten it (a PWL at the sample
   rate had one every sample, which kept older references fine by
   accident), so an unconverged reference was integrating at melange's own
   step and its error was charged to melange: noyce-cascaded-triodes read
   11.8 % at 48 kHz against it, 0.156 % against a converged one. From
   `TMAX = tstep/16, reltol = 1e-4` the reference is refined in both
   parameters in turn, halving `TMAX` (to `tstep/256`) and tightening
   `reltol` tenfold (to `1e-6`), and is accepted when both refinements move
   it by at most the bound: normalized RMS, DC-blocked, after the settle
   window, at most 10 % of the RMS tolerance graded (`SETTLED_FRACTION`;
   `ValidationOptions::reference_bound` overrides it). Halving `TMAX` alone
   is not enough: where ngspice's own error control already keeps its steps
   shorter than `TMAX`, halving it changes nothing and two identical runs
   "agree". Where ngspice cannot run the tighter `reltol` (it gives up at
   the start of the transient, "timestep too small", because every Newton
   solve must meet the tighter test), the tolerance refinement tightens
   `trtol` (the factor on the truncation-error estimate, the step control
   alone) tenfold instead, provided Newton's `reltol` is already under the
   bound; the self-check line says so. A reference that runs out of
   refinements refuses the verdict (`ValidationError::ReferenceNotConverged`). The accepted figure is
   printed under the RMS error ("reference self-check …") and carried in
   the JSON report. A `reltol` tighter than the default is appended after
   the deck's own `.OPTIONS`, so it wins over a deck author's `reltol`.

3. **Replaces the input source with a Thevenin pair** (`inject_thevenin_source`):
   the deck's voltage source whose n+ terminal is the input node (`in`) is
   replaced by the drive behind `R_mlg_src in_mlg_src in 1`, matching
   melange's 1-ohm Thevenin input model. The drive is the continuous stimulus
   melange's input samples are samples of:
   - an analytic stimulus the caller declares
     (`ValidationOptions::analytic_stimulus`) drives the reference directly:
     `SIN(...)` for a sine (`melange validate`'s 1 kHz test signal), a
     behavioural `B` source in `time` for a linear chirp. The samples are
     checked against it first;
   - otherwise the samples' band-limited reconstruction, a windowed sinc
     evaluated 16 points per sample (`reconstruction.rs`, within 1e-7 of a
     sine between samples up to 20 kHz at 48 kHz), as a PWL.
   Never a PWL at the sample rate: linear interpolation carries sinc² images
   around every multiple of fs, which a deck whose gain rises toward fs
   amplifies and the reference's 48 kHz output sampling folds back. On
   noyce-transformer-triode (1 MOhm into a 100 mH primary) that biased the
   reference 1 % low at the plate while ngspice `.ac` and melange agreed to
   0.001 dB; with the analytic drive the deck passes at 0.12 %. The CI gate's
   `input_pwl.txt` files are sparse PWL definitions, which melange samples:
   there the PWL is the continuous stimulus, and it is used as written.

4. **Strips melange-only directives** so ngspice can parse the deck:
   `.pot`, `.switch`, `.input_impedance`, `.wiper`, `.gang`, `.runtime`,
   `.mismatch`, `.tolerance`, `.seed`.

   For `.mismatch` / `.tolerance` / `.seed` the melange side is stripped to
   match: `run_melange_solver_from_str` parses with
   `ParseOptions { disable_unit_variation: true }`, so both engines see the
   deck's nominal component values. A deck carrying live jitter gets a
   qualifier naming the disabled directives and the unexercised seed on its
   PASSED/FAILED line. Until 2026-09-22 the strip was one-sided and the
   reported correlation was jittered-against-nominal; any PASS claim for such
   a deck predating that needs re-running. See
   `docs/aidocs/UNIT_VARIATION.md` "Validation Implications".

5. **Refuses a reference that ignored part of the deck.** ngspice warns
   `unrecognized parameter (…) - ignored` and simulates on without it; a run
   with such a warning is refused, naming each parameter and its card
   (`ignored_parameters`). Where melange reads a key ngspice lacks, the
   reference gets a translation (below) or the run is refused.

6. **Gives the reference each triode as melange resolved it**
   (`tube_translate.rs`): the Koren/D&Z B-source subckt takes its parameters
   from melange's resolver (card, catalog, defaults), with the
   inter-electrode capacitances between the terminals and the currents
   evaluated at an internal grid behind `RGI`. Until 2026-09-30 the twin read
   the card itself and dropped `CCG`/`CGP`/`CCP` and `RGI` (twas-preamp:
   2.7 % against 0.12 % with them).

6b. **Gives the reference each JFET as melange resolved it**
   (`jfet_translate.rs`): `BETA = IDSS/VP²` (ngspice has no `IDSS`),
   catalog and default values, the SPICE sign for a P-channel `VTO`, and the
   gate capacitances as the constant capacitors melange stamps (ngspice's are
   bias-dependent). A card with `N` other than 1 is refused: ngspice's
   level-1 JFET has no emission coefficient.

7. **Runs self-heating devices isothermal** (`ParseOptions::
   disable_self_heating`, `thermal_translate.rs`). ngspice's diode and BJT
   have no `RTH`/`CTH`/`TAMB` (and the triode twin has no thermal model), so
   validate builds melange with `RTH` not applied, the reference drops the
   three keys, and each instance of a card whose `TAMB` is not TNOM is placed
   at that temperature with ngspice's instance `temp=`: `TAMB`'s static role
   (IS and N·Vt scaled from TNOM) is kept on both sides. The status line says
   so ("self-heating disabled for comparison (…)"), next to the
   unit-variation qualifier; self-heating's own correctness is covered by its
   analytic tests. A self-heating clipper at 320 K witnesses both halves
   (`thermal_twin_tests.rs`: 1.5 % NRMSE with melange self-heating, 6.5 %
   without the instance temperature).

8. **Gives the reference each `.linearize`d device as melange built it**
   (`linearize_twin.rs`): the element line is replaced by the small-signal
   model at the DC operating point (`G` sources for the terminal-current
   Jacobian, an `I` source for the constant that puts the operating point
   back, the kept capacitances as `C`), and a `Circuit:` line names the
   devices. Against the full device a linearized common-emitter stage at
   0.3 V failed on the real transistor's distortion (0.107 % RMS, THD
   −59.9 dB against melange's −189 dB); with the twin it matches to 0.029 %.
   Whether the linearization holds at the drive is the build's question: a
   device out of its region is a reduced-model exit and validate refuses the
   run, saying so. A device inside a subcircuit is refused (its element line
   is not the deck's own).

9. **Gives the reference melange's parasitic caps** (`with_parasitic_caps`,
   `melange validate` / `validate_circuit_with_options`). A capacitor-free
   nonlinear deck is built with 10 pF across each device junction (see
   DEVICE_MODELS.md "Parasitic Cap Auto-Insertion"), recorded by node name in
   `CodegenMeta::parasitic_caps`. The melange side now runs first, and the
   deck handed to ngspice gets those capacitors as `C_melange_parasitic_k`
   lines before `.end`; the report says so on a `Circuit:` line. Without them
   the two engines simulated different circuits: a JFET resistor with its gate
   held through 1 MΩ measured 1.9e-2 normalized rms error, 3.9e-4 with them
   (`parasitic_twin_tests.rs`). The CI gate (`tests/spice_validation.rs`)
   calls ngspice itself and has no capacitor-free nonlinear deck; one that
   adds such a deck must route its reference through `with_parasitic_caps`.

### Netlist Structure — SINGLE deck, strip-VIN protocol

Each test data dir carries ONE `circuit.cir` used by BOTH engines:

```spice
* Circuit title (line 1 is ALWAYS title in SPICE)
VIN in 0 DC 0
R1 in out 10k
C1 out 0 10n
.TRAN 2.083e-5 10m
.PRINT TRAN V(out) V(in)
.END
```

- **ngspice side**: the harness replaces `VIN` with the Thevenin PWL pair
  (see above).
- **melange side**: `strip_vin_source(netlist, "in")` removes `VIN` (matched
  by n+ terminal == input node) and melange applies the input via
  `input_conductance` stamping. A voltage source left in the melange netlist
  would clamp the node — that's why the strip exists.

**Footguns:**
- The VIN's n+ terminal must BE the input node (`VIN in 0 DC 0`). A deck
  that bakes its own Thevenin pair (`VIN in_src 0` + `R_src in_src in 1`)
  escapes both the strip and the inject — neither matches n+ == "in" — and
  the leftover source adds a second 1-ohm shunt at the input node on both
  sides, halving the drive level (this bug shipped in the three_bjt_transformer_output_amp
  deck until 2026-07-18).
- The input PWL should start at 0 V. ngspice pre-settles its DC operating
  point at PWL(t=0) while melange starts from its own (zero-input) DC OP; a
  non-zero first sample gives the two engines different initial conditions
  and puts a genuine onset transient in melange's output with no counterpart
  in the reference (5 Hz blocker droop, ~0.4 RMS over 100 ms for a unit step).

The historical two-netlist protocol (`circuit_no_vin.cir` variants) is
retired; the four remaining dead `circuit_no_vin.cir` files were deleted
2026-07-18.

### Rate sweep: discretization or model error (`--rate-sweep`)

A deck that fails at its rate may be modelled exactly and merely integrated
coarsely. `melange validate --rate-sweep` (library:
`melange_validate::rate_sweep`) renders the deck at `fs`, `2fs` and `4fs`
with oversampling off, the analytic stimulus sampled at each rate, and adds
`8fs` when three rates are ambiguous. Every render is graded against ONE
reference: the finest run's converged ngspice output, taken at each
render's rate (an exact subsample) and DC-blocked there as the render is
(one blocker at the finest rate would leave the blocker's own first-order
discretization, 3e-4 of gain at 1 kHz and 48 kHz, in the errors). Each
rate's error is over ALL its samples.

The asymptote is extrapolated from the three finest renders, on the error
waveform `e_k = y_k − ref_k` at the instants they share (the coarsest one's
grid): least-squares ratio `r = Σ(e1−e2)(e2−e4)/Σ(e2−e4)²`, order
`p = log2 r`, `e∞ = e4 + (e4−e2)/(2^p−1)`, model error `‖e∞‖/‖ref‖`. Aitken
on the three error figures is the cross-check, and the fallback when the
waveform fit has no answer, which the verdict then says. Verdicts, with
their stated thresholds (`rate_sweep.rs` constants):

- **PASS** at the requested rate;
- **CONVERGES**: the error falls toward ngspice; reported with the order,
  the model error and the rate the fit needs for 1 % and 0.1 %
  (extrapolated from the finest rate still above the tolerance, capped at
  the first rate that meets it);
- **PLATEAU**: the error has stopped falling, the finest ratio `e2/e4`
  below 1.5 (`PLATEAU_RATIO`);
- **DIVERGES**: the error rises with the rate by more than 5 %;
- **UNRESOLVED**, with the reason:
  - the finest reference's self-check is not below a third of the finest
    error (`RESOLUTION_FRACTION`), after one refinement to that bound;
  - edge-dominated: the finest render's error on the shared grid and over
    all its samples differ by more than 1.5x (`EDGE_RATIO`). The grid
    samples the finest render at one instant in four, so an error that
    lives in edges a few samples wide is invisible to it, and no fit is
    made on it;
  - pre-asymptotic: the two successive error ratios differ by more than
    20 % (`RATIO_AGREEMENT`);
  - the ratios agree and the error still falls, but the fit puts a floor at
    half the finest error or more.
  The last two add the `8fs` render and decide on the three finest; still
  ambiguous, the verdict stays UNRESOLVED. An edge-dominated verdict also
  prints each rate's unaligned error next to validate's aligned one.

**Alignment on edges a sample wide.** validate aligns the reference to the
render by one least-squares fractional delay (see `alignment`), applied
with a band-limited interpolator. Across an edge that spans one or two
samples, that interpolator ripples the reference at the sample rate (Gibbs)
for a few samples after the edge. The delay still lowers the error, as
designed: steve-1073-preamp at 48 kHz reads 2.07 % aligned against 2.39 %
unaligned, with a ±20–40 mV ripple on an 18 V edge (0.0072-sample delay).
It is a limit of the harness, not an error in either engine: a post-edge
alternation in an edge-dominated deck's error is not evidence of a ring in
the render until the render itself is checked.

PLATEAU, DIVERGES and UNRESOLVED fail. Measured 2026-09-30: rc-lowpass PASS,
converging at order 2.00 to a model error of 0.0002 % (reference self-check
0.0002 %); steve-1073-preamp UNRESOLVED, edge-dominated (192 kHz: 1.02 % on
every sample, 0.28 % on the grid). Do not sweep with `--oversampling`: its
half-band filters' phase enters the comparison.

### DC blocking and settle windows

- Generated melange code runs with `dc_block: true` (5 Hz HPF seeded from
  the compile-time DC OP). The harness applies `melange_validate::
  dc_block_signal` — the single shared implementation, seeded from the
  signal's first sample — to the ngspice output ONCE. Never DC-block the
  melange output again in a test: it is already blocked inside the generated
  binary (a double block inflates the error, 15x on the 3-BJT preamp
  deck).
- `ComparisonConfig.settle_time_s` (default 0.0) symmetrically excludes the
  first N seconds of both signals before any metric is computed, so
  steady-state gates can be tightened without widening them to cover startup
  residue. Tests opt in per-signal-length (e.g. 64 ms = 2x the 5 Hz blocker
  tau on the 500 ms 3-BJT preamp signal; 3 ms on the 10 ms BJT CE signal).

## Melange Solver Setup

### Oversampling (`--oversampling {1|2|4}`)

`--oversampling` is compile-time codegen, not a runtime knob, so a 2x build is
different DSP and `validate` takes the flag too — otherwise the 1x code is
validated and the 2x code shipped. Default 1.

ngspice is NOT changed: it has its own timestep, and its output is NOT filtered.
The reference is aligned to the melange output by ONE best-fit constant delay —
least-squares, fractional, delay only and never gain, seeded at the analytic
half-band round-trip delay (2.6502 host samples at 1 kHz for 2x, 3.4682 for 4x;
0 at 1x) and bounded to half a stimulus period. The same alignment runs in every
mode including 1x, so the rows stay commensurable. No tolerance moves; every run
prints an `Aligned:` line with the fitted delay next to the analytic one, and an
oversampled run adds a `Build:` line.

The half-bands' frequency-dependent phase therefore stays in the number, because
it ships: 1-rho on `overdrive_pedal_native_u` (48 kHz, 0.3 V, 500 ms) is 1.00e-6 at 1x,
5.64e-6 at 2x, 6.25e-6 at 4x.

Full rationale, the re-baselined table, the per-leg attribution measurement and
the twin-drift guard are in [OVERSAMPLING.md](OVERSAMPLING.md) § "Validating an
oversampled build".

### DC Operating Point for Nonlinear Circuits

For circuits with nonlinear devices (diodes, BJTs), the codegen pipeline
automatically embeds the DC operating point as the `DC_NL_I` constant in
the generated state, set by `CircuitIR::from_kernel()`. Generated code
initializes `i_nl_prev = DC_NL_I` in `Default` and on `reset()`, so the
solver starts from the correct bias point on the first sample.

Without this, BJT circuits would start from v=0 (cutoff) while SPICE starts
from its own DC OP solution, causing massive output differences.

The runtime `solver.initialize_dc_op(...)` API has been removed; everything
is automatic now. See [DC_OP.md](DC_OP.md) for the solver algorithm.

### Input Conductance Stamping

**CRITICAL**: Stamp into MNA before building DK kernel:
```rust
// `stripped` is the single deck after strip_vin_source() removed VIN
let mut mna = MnaSystem::from_netlist(&stripped)?;

// Stamp input conductance (1.0 for near-ideal voltage source)
let input_conductance = 1.0;  // 1 ohm
mna.g[input_node][input_node] += input_conductance;

// THEN build kernel with input in G matrix
let kernel = DkKernel::from_mna(&mna, sample_rate)?;
```

### Input Integration

```rust
// In generated process_sample() (charge form): the input enters once, at n+1;
// the capacitor history (alpha*C*v_prev + q_dot) carries the rest.
rhs[input_node] += input * input_conductance;

// WRONG: stamping the source twice
// rhs[input_node] += 2.0 * input * input_conductance;
// rhs[input_node] += (input + input_prev) * input_conductance;
```

(`(input + input_prev) * G_in` is the whole-system form, which the deprecated
library `LinearSolver` still uses together with its `alpha*C - G` history; the two
forms agree on linear circuits. See `COMPANION_MODELS.md`.)

## Debugging Low Correlation

If correlation ≈ 0 or very low:

### Checklist

1. **Is input actually reaching the circuit?**
   - Verify `mna.g[input_node][input_node]` includes `input_conductance`
   - Check input node index maps correctly (0-based vs 1-based)

2. **Are timesteps aligned?**
   - ngspice without INTERP: variable timestep, mismatched samples
   - ngspice with INTERP: uniform timestep, matched samples
   - Check sample counts match between SPICE and melange output

3. **Is the circuit topology the same?**
   - Compare MNA G matrix to SPICE netlist
   - Verify no extra voltage sources in melange netlist

4. **Is input integration correct?**
   - Generated code: `input * G_in`, once, at `n+1`
   - Check `q_dot` is committed with `v_prev` every sample (trapezoidal builds)

5. **Has the slowest time constant settled?** Before comparing absolute
   levels, periods or limit cycles, list every time constant in the deck and
   compare only over a window starting at least `5 × τ_max` after the start.
   The two engines start from different states: ngspice pre-settles its DC OP
   (or starts every node at 0 V under `uic`), melange starts from its DC OP or
   IC seed. A free-running circuit forgets its start only as fast as its
   slowest network does. Measured 2026-09-29 on the G10 transistor astable:
   its output coupling network (1 µF into 100 kΩ, τ = 0.1 s) left a 50 ms
   comparison 6.8 % off in period and 1 V off in the low level against
   ngspice `uic`; over 0.65–0.8 s the two agree to 0.09 % and 20 mV.

### Diagnostic Output

Expected for RC lowpass (10k + 10nF, 48kHz):
```
MNA G[0]: [1.0001, -0.0001]    // 1.0 from input_conductance + 0.0001 from R1
MNA G[1]: [-0.0001, 0.0001]
MNA C[1][1]: 1e-8              // Capacitor at output node

SPICE output first 5: [0.0, 0.0134, 0.0402, 0.0804, 0.1340]
Melange output first 5: [0.0, 0.0134, 0.0402, 0.0804, 0.1340]
Correlation: 0.99999995
```

### Renders validate refuses

The melange side prints its solver counters as `DIAG:` lines. Before any
comparison, validate refuses the render when:
- any sample was never solved (`nr_hold_count` > 0: the full-LU death-spiral
  hold, or Schur's unconverged commit): those samples are the previous state;
- the input was clamped to `INPUT_LIMIT_V` (100 V) or had NaN/Inf replaced by
  0: ngspice saw the unclamped input;
- the output passed the generated output clamp (`clamp_count` > 0, the
  post-DC-block limit, 10 V by default): ngspice has no output clamp.
`validate_refusal_tests.rs` has a witness for each of the three, including a
real full-LU build forced to `MAX_ITER = 1` for the unsolved-sample case.

## Thread Safety

When running validation tests concurrently:
- Use unique temp file names per thread
- Use `AtomicU64` counter for temp file naming
- Avoid race conditions where tests overwrite each other's netlists

## References
- ngspice manual: https://ngspice.sourceforge.io/docs.html
- SPICE format reference: https://bwrcs.eecs.berkeley.edu/Classes/IcBook/SPICE/

---

# One build for every consumer

`melange validate`, the `spice_validation` harness and every CLI verb assemble
the circuit through one entry point, `melange_solver::build::build`, which
runs the shared front-end steps in `melange_solver::pipeline`:

| step | what it does |
|---|---|
| `apply_forward_active_reduction` | `--bjt-fa` (default off) |
| `apply_grid_off_reduction` | `--tube-grid-fa` (default auto, which reduces nothing today) |
| `apply_linearize_reductions` | `.linearize` |
| `expand_internal_nodes` | parasitic-BJT internal nodes on nodal builds |
| `auto_tune_max_iter` | the NR iteration budget when `--max-iter` is unset |

`melange validate` takes the same `--bjt-fa` and `--tube-grid-fa` flags as
`compile`, so the circuit it measures is the one `compile` ships.
`tests/spice_validation.rs` has no MNA build of its own: it delegates to
`melange_validate::run_melange_solver_from_str`. **Do not add a local MNA
build to a validation path.**

Why it matters: each step can change the system that is solved. `.linearize`
is the only thing routing some decks to full-LU (the `linearized_bypass` gate
in `nodal_emitter.rs`); without it the emitter picks Schur NR, which on an
expanded-parasitic power-amp deck diverged at the first nonzero input sample
(1319 % RMS against 0.246 % through the shipped build). A fixed `MAX_ITER`
in place of `auto_tune_max_iter` cuts both ways: `auto_tune_max_iter` has no
floor of 100, so a harness at 100 is stricter than the shipped build on stiff
decks and more permissive on decks the tuner gives 50-70.

Provenance: before `6bc3ef1` (2026-09-02) validate skipped `.linearize`,
`auto_tune_max_iter` and the gated expansion, and before 2026-09-03 it applied
no forward-active or grid-off reduction. A validate number recorded before
then is not a statement about the shipped build for a deck that uses
`.linearize` or carries parasitic BJTs (`RB`/`RC`/`RE`); decks with neither
were unaffected.

# Device coverage: what the ngspice oracle can and cannot check

ngspice has **no vacuum-tube primitive at all** — it parses a `T` card as a
lossy transmission line. Tubes validate because melange *synthesizes* a Koren
B-source `.subckt` twin for each one.

| device | ngspice validation |
|---|---|
| diode, BJT, JFET, MOSFET | native SPICE elements — validate directly |
| triode (`T`) | via `tube_translate.rs` (sharp only, `svar = 0`) |
| pentode (`P`) | via `pentode_translate.rs` — **added 2026-09-02**, sharp only, all 3 screen forms |
| op-amp (`U`) | via `opamp_translate.rs` — **added 2026-09-23**, linear VCCS twin; refuses per run if a rail or slew clamp engages |
| VCA (`Y`), LDR (`O`), glow (`N`) | **cannot** — no ngspice model type, and each carries device state no primitive reproduces |
| variable-mu tubes (`svar > 0`) | **cannot** — explicitly out of scope, errors |

**The op-amp twin is melange's own macromodel, and it is linear.** melange's
op-amp IS a VCCS (`Gm = AOL/ROUT` into the output node, `Go = 1/ROUT` to
ground), so `opamp_translate.rs` emits exactly that as a `G` + `R` pair, plus
`RIN`/`IB` when the `.model` sets them. What it does NOT emit is the post-NR
rail clamp (VCC/VEE/VSAT, and the ±13 V default GBW triggers) or the `SR` slew
clamp — no ngspice primitive reproduces `ActiveSetBe`'s pin-and-BE-resolve, and
a clamp that does not match melange's *mode* exactly is worse than none. So the
translator's scope limit is enforced per run rather than documented: each
clamped op-amp's output node is added to the reference capture and
`opamp_translate::check_rail_probes` refuses the comparison if the reference
shows the clamp would have engaged. (Watching the reference suffices: the two
engines follow one trajectory up to melange's first clamp, so the reference is
also at the threshold on that sample.) `GBW` itself is deliberately not
translated — melange computes `iir_c_dom` from it but no codegen path consumes
it, so its only live effect is defaulting the rails.

**The tube twin reproduces melange's OWN Koren equation.** It therefore
cross-checks the SOLVER (NR + integration + timestep) against ngspice's given an
identical device equation. It does **not** independently validate melange's tube
physics. Do not cite a passing tube validation as evidence the device model is
right; cite it as evidence the solver integrates it the same way ngspice does.
