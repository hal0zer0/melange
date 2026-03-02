# SPICE Validation Plan for Melange

**Date:** 2026-02-23  
**Version:** 1.1  
**Status:** Infrastructure Ready, Integration Issues Identified

---

## Executive Summary

SPICE validation is **critical** to melange's mission. SPICE is the ground truth for circuit behavior, and every melange model must validate against ngspice before it ships.

**Current State:**
- ✅ Validation infrastructure is **production-ready**
- ✅ ngspice integration working
- ✅ HTML/CSV/JSON report generation working
- ⚠️ Melange solver integration has issues (see Section 7)

---

## 1. Current Validation Capabilities

### 1.1 Infrastructure Status

| Component | Status | Location |
|-----------|--------|----------|
| ngspice runner | ✅ Ready | `crates/melange-validate/src/spice_runner.rs` |
| Signal comparison | ✅ Ready | `crates/melange-validate/src/comparison.rs` |
| HTML/CSV/JSON reports | ✅ Ready | `crates/melange-validate/src/visualizer.rs` |
| Validation API | ✅ Ready | `crates/melange-validate/src/lib.rs` |
| Test circuits | ✅ 4 circuits | `crates/melange-validate/tests/data/` |
| ngspice availability | ✅ Installed | `/usr/bin/ngspice` (v42) |
| SPICE validation tests | ✅ Created | `crates/melange-validate/tests/spice_validation.rs` |

### 1.2 Available Test Circuits

```
crates/melange-validate/tests/data/
├── rc_lowpass/           # Linear: R=10k, C=10nF, fc≈1.6kHz
├── diode_clipper/        # Nonlinear: Antiparallel 1N4148 diodes
├── bjt_common_emitter/   # Active: BC547 NPN amplifier
└── antiparallel_diodes/  # 2D nonlinear: Symmetric clipping
```

### 1.3 Comparison Metrics Available

| Metric | Tolerance (Default) | Tolerance (Strict) | Purpose |
|--------|---------------------|-------------------|---------|
| RMS Error | 0.1% | 0.01% | Overall accuracy |
| Peak Error | 10 mV | 1 mV | Worst-case deviation |
| Max Relative Error | 1% | 0.1% | Dynamic range |
| Correlation | > 0.9999 | > 0.99999 | Waveform shape |
| THD Error | ±1 dB | ±0.1 dB | Harmonic content |
| SNR | Reported | Reported | Signal quality |

### 1.4 Report Generation

- **HTML reports:** Interactive plots (signal overlay, error, histogram, scatter)
- **CSV export:** Raw data for Python/MATLAB analysis
- **JSON export:** Machine-readable metrics for CI

---

## 2. What Validation is Missing?

### 2.1 Critical Gaps

| Gap | Impact | Priority |
|-----|--------|----------|
| Solver integration bugs | Validation tests fail | 🔴 **Critical** |
| No CI integration | Regressions go undetected | 🟡 High |
| Limited circuit coverage | Only 4 test circuits | 🟡 High |
| No AC analysis validation | Frequency response unchecked | 🟠 Medium |
| No DC operating point check | Biasing errors missed | 🟠 Medium |

### 2.2 Known Issues (Discovered During Testing)

1. **PWL Source Injection**: The `inject_pwl_source()` function in `spice_runner.rs` isn't correctly replacing existing voltage sources
2. **Parser Limitations**: Parser doesn't support PWL values in netlist (fails on `Vin in 0 PWL(...)`)
3. **Solver Output Mismatch**: Melange solver output doesn't match SPICE (needs debugging)

### 2.3 Missing Test Circuits

Circuits that should be validated but aren't:

| Circuit | Devices | Test Purpose |
|---------|---------|--------------|
| Tube triode stage | 1x 12AX7 | Vacuum tube model validation |
| JFET buffer | 1x JFET | JFET Shichman-Hodges model |
| MOSFET power amp | 1x MOSFET | Power stage modeling |
| LDR tremolo | 1x CdS LDR | Optocoupler dynamics |
| Multi-stage amp | BJT+diodes | Device interaction |
| Bridge rectifier | 4x diodes | Multi-diode topology |
| Wien bridge osc | Op-amp+RC | Oscillator validation |

### 2.4 Missing Analysis Types

Current validation only supports transient analysis:

| Analysis | Status | Use Case |
|----------|--------|----------|
| Transient | ✅ Ready | Time-domain behavior, THD |
| DC Operating Point | ❌ Missing | Bias verification |
| AC Small-Signal | ❌ Missing | Frequency response |
| Transfer Function | ❌ Missing | Analytical validation |

---

## 3. Running Validation Tests

### 3.1 Quick Start

```bash
cd /home/homeuser/dev/melange

# Check ngspice is available
which ngspice && ngspice --version

# Run all validation tests (will show failures)
cargo test -p melange-validate --test spice_validation -- --nocapture

# Run only unit tests (should pass)
cargo test -p melange-validate --test spice_validation -- --nocapture 2>&1 | grep "^test"

# Run full test suite
cargo test -p melange-validate
```

### 3.2 Run Single Circuit Validation

```rust
use melange_validate::{
    validate_circuit, ValidationOptions, comparison::ComparisonConfig,
};
use std::path::Path;

let input: Vec<f64> = (0..4800)
    .map(|i| (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).sin())
    .collect();

let result = validate_circuit(
    Path::new("crates/melange-validate/tests/data/rc_lowpass/circuit.cir"),
    &input,
    48000.0,
    "out",
    &ComparisonConfig::default(),
).expect("Validation failed");

println!("{}", result.report.summary());
```

### 3.3 Using the Validation Builder

```rust
use melange_validate::ValidationBuilder;
use std::path::Path;

let result = ValidationBuilder::new(Path::new("circuit.cir"))
    .with_input(&input_signal)
    .at_sample_rate(48000.0)
    .measuring_node("out")
    .with_strict_tolerances()
    .generate_html_on_failure()
    .output_to(Path::new("./validation_output"))
    .run()
    .expect("Validation failed");
```

---

## 4. Recommended Validation Workflow

### 4.1 Developer Workflow

```
1. Design circuit in SPICE (ngspice/LTspice/Quux)
        ↓
2. Save netlist to tests/data/<circuit_name>/circuit.cir
        ↓
3. Create PWL input: tests/data/<circuit_name>/input_pwl.txt
        ↓
4. Run validation: cargo test -p melange-validate --test spice_validation
        ↓
5. Review HTML report (auto-generated on failure)
        ↓
6. Tune tolerances based on circuit type
        ↓
7. Commit passing test
```

### 4.2 CI/CD Workflow

```yaml
# .github/workflows/validation.yml
name: SPICE Validation

on: [push, pull_request]

jobs:
  validate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      
      - name: Install ngspice
        run: sudo apt-get install -y ngspice
      
      - name: Run SPICE validation tests
        run: cargo test -p melange-validate --test spice_validation -- --nocapture
      
      - name: Upload failure reports
        if: failure()
        uses: actions/upload-artifact@v3
        with:
          name: validation-reports
          path: crates/melange-validate/tests/data/*/failure_report.html
```

### 4.3 Tolerance Guidelines

| Circuit Type | RMS Error | Peak Error | Correlation | Notes |
|--------------|-----------|------------|-------------|-------|
| Linear (RC, RL) | < 0.01% | < 1 mV | > 0.99999 | Should match exactly |
| Diode circuits | < 0.1% | < 10 mV | > 0.9999 | Model parameter variation |
| BJT amplifiers | < 0.1% | < 10 mV | > 0.9999 | Include DC bias check |
| Tube stages | < 0.5% | < 50 mV | > 0.999 | Koren model approximation |
| Multi-device | < 1% | < 100 mV | > 0.999 | Error accumulates |

---

## 5. Implementation Roadmap

### Phase 1: Fix Solver Integration (Immediate) - PRIORITY

- [ ] Debug PWL source injection in `spice_runner.rs`
- [ ] Fix melange solver output to match SPICE
- [ ] Verify test circuits produce correct results
- [ ] Enable all 4 validation tests to pass

**Estimated effort:** 1-2 days  
**Impact:** Working SPICE validation

### Phase 2: CI Integration (Short-term)

- [ ] Add ngspice to CI environment
- [ ] Create GitHub Actions workflow
- [ ] Configure artifact upload for failure reports
- [ ] Add validation status badge to README

**Estimated effort:** 4-6 hours  
**Impact:** Catch regressions automatically

### Phase 3: Expand Coverage (Medium-term)

- [ ] Add tube triode test circuit
- [ ] Add JFET test circuit
- [ ] Add multi-stage amplifier test
- [ ] Add AC analysis validation
- [ ] Add DC operating point checks

**Estimated effort:** 2-3 days  
**Impact:** Comprehensive model validation

### Phase 4: Advanced Features (Long-term)

- [ ] Parametric sweep validation (Monte Carlo)
- [ ] Temperature sweep validation
- [ ] Batch regression testing
- [ ] Performance benchmarking (SPICE vs melange speed)

**Estimated effort:** 1 week  
**Impact:** Production-grade validation

---

## 6. Sample ngspice Commands

### 6.1 Manual Validation

```bash
# Run transient analysis
cd crates/melange-validate/tests/data/rc_lowpass
ngspice -b circuit.cir

# Save output to file
ngspice -b -o output.txt circuit.cir

# With custom time step
echo ".tran 10u 100m" | ngspice -b circuit.cir -

# DC operating point
echo ".op" | ngspice -b circuit.cir -

# AC analysis (frequency response)
echo ".ac dec 10 1 100k" | ngspice -b circuit.cir -
```

### 6.2 PWL Input Generation

```bash
# Generate sine wave PWL file
python3 << 'EOF'
import math
with open('input_pwl.txt', 'w') as f:
    f.write('* Sine wave input\n')
    sr = 48000
    f0 = 1000
    amp = 1.0
    duration = 0.01
    for i in range(int(sr * duration)):
        t = i / sr
        v = amp * math.sin(2 * math.pi * f0 * t)
        f.write(f'{t:.6e} {v:.6e}\n')
EOF
```

### 6.3 Quick Comparison

```bash
# Run both simulations and compare
cargo test -p melange-validate test_rc_lowpass_vs_spice -- --nocapture

# Generate HTML report (auto-generated on failure)
open crates/melange-validate/tests/data/rc_lowpass/rc_lowpass_failure_report.html
```

---

## 7. Known Issues and Debugging

### 7.1 Current Test Results

```
running 9 tests
test test_comparison_config_levels ... ok
test test_pwl_interpolation ... ok
test test_pwl_file_loading ... ok
test test_ngspice_availability ... ok
test test_rc_lowpass_vs_spice ... FAILED          # <- Solver mismatch
test test_antiparallel_diodes_vs_spice ... FAILED # <- Parser issue
test test_diode_clipper_vs_spice ... FAILED       # <- Solver mismatch
test test_bjt_common_emitter_vs_spice ... FAILED  # <- Solver mismatch
test test_all_circuits_batch ... FAILED
test result: FAILED. 4 passed; 5 failed
```

### 7.2 Issue Details

#### Issue 1: PWL Source Injection (High Priority)

**Symptom:** Melange output doesn't match SPICE at all (correlation ≈ 0)

**Root cause:** The `inject_pwl_source()` function creates a modified netlist but the PWL source may not be properly replacing the original DC source.

**Debug steps:**
```bash
# Check the modified netlist
cd /tmp && cat melange_pwl_*.cir 2>/dev/null | head -20

# Run ngspice manually on modified netlist
ngspice -b /tmp/melange_pwl_*.cir
```

**Fix needed:** Review the `inject_pwl_source()` function in `spice_runner.rs` line 171-250.

#### Issue 2: Parser PWL Support (Medium Priority)

**Symptom:** `Parse error at line 6: Invalid DC value: PWL(0`

**Root cause:** The SPICE parser doesn't support inline PWL values like `Vin in 0 PWL(0 0 0.5m 3...)`

**Workaround:** Use placeholder DC source and inject PWL externally (already done for antiparallel_diodes)

**Fix needed:** Update parser to handle PWL syntax or document the limitation.

#### Issue 3: Solver Output Mismatch (High Priority)

**Symptom:** SPICE and melange produce completely different outputs

**Possible causes:**
1. Wrong input node mapping in solver
2. Device model parameters not correctly extracted
3. DK kernel setup issue
4. Sample rate mismatch

**Debug steps:**
```rust
// Add debug output to test
println!("SPICE first 10 samples: {:?}", &spice_output[..10]);
println!("Melange first 10 samples: {:?}", &melange_output[..10]);
println!("Input signal first 10: {:?}", &input_signal[..10]);
```

### 7.3 Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| "ngspice not found" | ngspice not installed | `sudo apt install ngspice` |
| Parse errors | Netlist format issues | Check line continuations |
| Large RMS error | Wrong sample rate | Verify 48kHz standard |
| THD mismatch | Different windowing | Use Hann window consistently |
| DC offset | Missing DC blocker | Check DC bias separately |

---

## 8. Circuit-Specific Validation Notes

### 8.1 RC Lowpass (Linear)

**Expected behavior:**
- Cutoff frequency: 1 / (2πRC) = 1.59 kHz
- Roll-off: -20 dB/decade
- No distortion (THD ≈ -∞ dB)

**Validation focus:**
- Verify exact frequency response
- Check phase shift at cutoff
- Confirm no numerical artifacts

### 8.2 Diode Clipper (Nonlinear)

**Expected behavior:**
- Clipping threshold: ~0.7V (1N4148)
- Symmetric distortion
- THD increases with input amplitude

**Validation focus:**
- Clipping threshold accuracy
- THD matching within 0.1 dB
- Symmetry of positive/negative clipping

### 8.3 BJT Common Emitter (Active)

**Expected behavior:**
- DC bias: Vc ≈ 6V, Ic ≈ 1mA
- Voltage gain: ~10x (20 dB)
- Phase inversion

**Validation focus:**
- DC operating point first
- AC gain within 0.5 dB
- Output swing before clipping

---

## 9. Files Created/Modified

### New Files

| File | Purpose |
|------|---------|
| `SPICE_VALIDATION_PLAN.md` | This document |
| `crates/melange-validate/tests/spice_validation.rs` | SPICE validation test suite |

### Modified Files

| File | Change |
|------|--------|
| `crates/melange-validate/tests/data/antiparallel_diodes/circuit.cir` | Changed from inline PWL to placeholder source |

---

## 10. References

- **Testing Strategy:** `docs/testing-strategy.md`
- **SPICE Translation:** `docs/spice-translation.md`
- **DK Method:** `docs/dk-method.md`
- **Known Limitations:** `docs/limitations.md`
- **Test Circuits:** `crates/melange-validate/tests/data/`

---

## 11. Appendix: Quick Commands

```bash
# Check ngspice
cd /home/homeuser/dev/melange
ngspice --version

# Run all validation tests
cargo test -p melange-validate --test spice_validation -- --nocapture

# Run only unit tests (4 should pass)
cargo test -p melange-validate --test spice_validation -- 2>&1 | grep "^test"

# Check generated reports
ls -la crates/melange-validate/tests/data/*/*.html

# Manual ngspice run
cd crates/melange-validate/tests/data/rc_lowpass
ngspice -b circuit.cir

# Build everything
cargo build --workspace

# Run full test suite
cargo test --workspace
```

---

*"SPICE is the ground truth. If it doesn't match SPICE, it's wrong."*

*The spice must flow.*
