# SPICE Validation Protocol

## Purpose
Verify melange solver matches ngspice output within tight tolerances.

## Expected Correlation Benchmarks

| Circuit Type | Correlation | RMS Error | Notes |
|--------------|-------------|-----------|-------|
| Linear (RC, RL) | > 0.999999 (6 nines) | < 0.1% | Should match almost exactly |
| Nonlinear (diodes) | > 0.9999 (4 nines) | < 1% | NR convergence differences |
| BJT circuits | > 0.999 (3 nines) | < 5% | Model parameter variations |

## ngspice Setup for Sample-Accurate Comparison

### Required Netlist Modifications

1. **Add `.OPTIONS INTERP`**
   ```spice
   .OPTIONS INTERP
   ```
   Forces ngspice to interpolate output at uniform timesteps instead of printing adaptive timestep points.

2. **Match Solver Sample Rate**
   ```spice
   .TRAN {tstep} {tstop}
   ```
   Where `tstep = 1.0 / sample_rate` (e.g., 2.083e-5 for 48kHz)

3. **Use PWL Source for Input**
   ```spice
   VIN in 0 PWL(0 0 1e-4 0.309 2e-4 0.5878 ...)
   ```
   PWL points should match solver's input samples.

### Netlist Structure

For validation, you need TWO netlists:

**circuit.cir** (for ngspice):
```spice
* Circuit title (line 1 is ALWAYS title in SPICE)
.OPTIONS INTERP
VIN in 0 PWL(...)
R1 in out 10k
C1 out 0 10n
.TRAN 2.083e-5 10m
.PRINT TRAN V(out) V(in)
.END
```

**circuit_no_vin.cir** (for melange solver):
```spice
* Circuit without input source - input applied via input_conductance
R1 in out 10k
C1 out 0 10n
.END
```

**Why two netlists?**
- ngspice needs `VIN` as PWL voltage source
- Melange applies input via `input_conductance` Thevenin equivalent
- Voltage source in netlist would clamp node with 1e6 S Norton equivalent

## Melange Solver Setup

### Input Conductance Stamping

**CRITICAL**: Stamp into MNA before building DK kernel:
```rust
let mut mna = MnaSystem::from_netlist(&netlist_no_vin)?;

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
