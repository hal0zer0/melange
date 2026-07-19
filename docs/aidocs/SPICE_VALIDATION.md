# SPICE Validation Protocol

## Purpose
Verify melange solver matches ngspice output within tight tolerances.

## Expected Correlation Benchmarks

Measured 2026-07-18 (HEAD b421358, ngspice-42, reltol=1e-4 reference):

| Circuit Type | Correlation | RMS Error | Notes |
|--------------|-------------|-----------|-------|
| Linear (RC, RL) | > 0.999999 (6 nines) | < 0.1% | Should match almost exactly |
| Nonlinear (diodes) | > 0.999999 (6 nines) | < 0.15% | Includes off-nominal `.pot` positions |
| BJT circuits | > 0.9996 | < 5% | BJT CE is the loosest (GP model gain ratio 1.024); wurli/neve are at 0.03-0.25% |
| Op-amp (linear) | ~1.0 | ~0% | VCCS macromodel matches ngspice |
| JFET circuits | > 0.9994 | < 4% | Shichman-Hodges 2D, ngspice BETA→IDSS |
| MOSFET circuits | > 0.99999 | < 0.1% | Level 1 SPICE |
| Audio-rate `.pot` R(t) | > 0.9999 | < 2% | vs native ngspice B-source; residual is per-sample ZOH of R(t) |

All tests are `#[ignore]`d so they only run with
`cargo test -p melange-validate --test spice_validation -- --include-ignored`
(they require ngspice on `PATH`). Each test calls `run_validation()` →
`run_melange_codegen()` in `crates/melange-validate/tests/spice_validation.rs`,
which exercises the full codegen pipeline: parse → MnaSystem → stamp G_in →
DkKernel → routing::auto_route → CodeGenerator::generate → write to `/tmp/`,
compile with `rustc --edition=2024 -O`, spawn the binary, pipe samples
through stdin/stdout, then compare against ngspice with `.OPTIONS INTERP`
for sample-aligned output. See `docs/aidocs/STATUS.md` for the latest
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
   for 48 kHz) and the tstop derived from the input signal.

3. **Replaces the input source with a Thevenin PWL pair** (`inject_thevenin_pwl`):
   the deck's voltage source whose n+ terminal is the input node (`in`) is
   replaced by `V... in_mlg_src ... PWL(...)` + `R_mlg_src in_mlg_src in 1`,
   matching melange's 1-ohm Thevenin input model.

4. **Strips melange-only directives** so ngspice can parse the deck:
   `.pot`, `.switch`, `.input_impedance`, `.wiper`, `.gang`, `.runtime`,
   `.mismatch`, `.tolerance`, `.seed`.

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
  sides, halving the drive level (this bug shipped in the neve_1073_output
  deck until 2026-07-18).
- The input PWL should start at 0 V. ngspice pre-settles its DC operating
  point at PWL(t=0) while melange starts from its own (zero-input) DC OP; a
  non-zero first sample gives the two engines different initial conditions
  and puts a genuine onset transient in melange's output with no counterpart
  in the reference (5 Hz blocker droop, ~0.4 RMS over 100 ms for a unit step).

The historical two-netlist protocol (`circuit_no_vin.cir` variants) is
retired; the four remaining dead `circuit_no_vin.cir` files were deleted
2026-07-18.

### DC blocking and settle windows

- Generated melange code runs with `dc_block: true` (5 Hz HPF seeded from
  the compile-time DC OP). The harness applies `melange_validate::
  dc_block_signal` — the single shared implementation, seeded from the
  signal's first sample — to the ngspice output ONCE. Never DC-block the
  melange output again in a test: it is already blocked inside the generated
  binary (a double-block inflated the neve-preamp error 15x until
  2026-07-18).
- `ComparisonConfig.settle_time_s` (default 0.0) symmetrically excludes the
  first N seconds of both signals before any metric is computed, so
  steady-state gates can be tightened without widening them to cover startup
  residue. Tests opt in per-signal-length (e.g. 64 ms = 2x the 5 Hz blocker
  tau on the 500 ms neve-preamp signal; 3 ms on the 10 ms BJT CE signal).

## Melange Solver Setup

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

### Trapezoidal Input Integration

```rust
// In solver process_sample():
// RIGHT: Trapezoidal rule
rhs[input_node] += (input + input_prev) * input_conductance;
input_prev = input;  // Save for next sample

// WRONG: Don't use 2.0 * input
// rhs[input_node] += 2.0 * input * input_conductance;
```

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
   - Verify trapezoidal rule: `(input + input_prev) * G_in`
   - Check `input_prev` is persisted across samples

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

## Thread Safety

When running validation tests concurrently:
- Use unique temp file names per thread
- Use `AtomicU64` counter for temp file naming
- Avoid race conditions where tests overwrite each other's netlists

## References
- ngspice manual: https://ngspice.sourceforge.io/docs.html
- SPICE format reference: https://bwrcs.eecs.berkeley.edu/Classes/IcBook/SPICE/
