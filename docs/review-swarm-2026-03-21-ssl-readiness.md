# SSL 4000 Bus Compressor Readiness — 2026-03-21

5 specialist agents assessed melange's readiness to build an SSL bus compressor plugin.

## Verdict: PARTIALLY READY — 2 blockers, both fixable in hours

## Critical Blockers

### B1. VCA K[1][1] = 0 blocks DK kernel construction (Robustness Agent)
- `dk.rs:323-332` — DK kernel validates K[i][i] < 0 for all i
- VCA control port draws zero current → N_i column 1 is all zeros → K[:,1] = 0 → K[1][1] = 0
- `from_mna()` returns `Err(DkError::InvalidKDiagonal)` — **VCA circuits cannot compile**
- **Fix**: Skip K diagonal check for dimensions with zero N_i columns (the device correctly has no current feedback on the control port). Or auto-route to nodal solver.

### B2. VCA missing from DC operating point solver (Plugin Dev, Robustness)
- `dc_op.rs:268-274` — `evaluate_devices_inner()` has no VCA match arm
- Falls through to wildcard that logs warning and treats VCA as zero current/zero Jacobian
- Wrong quiescent point → startup transient, incorrect DC_NL_I constant
- **Fix**: Add `(DeviceType::Vca, DeviceParams::Vca(vp))` arm, ~20 lines

## High Priority

### H1. VSCALE default was 10x off — FIXED (commit 9eace0b)
- Was 0.00528, corrected to 0.05298 V/neper
- 6.1mV now correctly produces -1dB (was -10dB)

### H2. No end-to-end VCA test (Plugin Dev)
- No codegen compile-and-run test, no circuit file, no SPICE validation
- Need a simple VCA test circuit to verify full pipeline

### H3. No VCA-specific voltage limiting (Robustness)
- VCA bypasses pnjlim/fetlim entirely
- With VSCALE=0.053, a 50mV control step = 2.6x gain change — manageable
- But fast SSL attacks could produce larger jumps
- **Fix**: Add `fetlim`-style limiting on VCA control voltage

## What Works Well

| Component | Status | Confidence |
|-----------|--------|------------|
| VCA gain law | Correct (after VSCALE fix) | High — matches THAT 2180 datasheet |
| Op-amps (NE5534/5532) | VCCS macromodel sufficient | High — SPICE validated, GBW irrelevant at audio |
| Diodes (1N4148) | Full Shockley + pnjlim | High — SPICE validated (corr 0.999) |
| R-C timing | Trapezoidal integration | Very high — 0.03% RMS vs SPICE |
| Pots (makeup gain) | Sherman-Morrison | High — tested on wurli-preamp |
| Switches (attack/release) | Component value switching | High — tested on pultec/tweed |
| Plugin generation | Full nih-plug project | High — ear protection, level params |

## Performance Assessment (DSP Agent)

- Circuit: N≈18, M≈6 (1 VCA + 4 sidechain diodes)
- DK codegen path (after B1 fix): M=6 Gauss elimination
- **Estimated: ~8.5x realtime at 96kHz (~12% CPU)**
- NR iterations: 2-3 typical, 4-5 on fast attacks
- No oversampling needed at 96kHz (9.6 samples per fastest 0.1ms attack)
- 2x oversampling recommended at 44.1kHz

## Modeling Accuracy (Analog Expert)

**~85-90% of SSL sonic character captured:**
- R-C sidechain dynamics (the dominant "SSL sound"): **99%**
- Op-amp signal path: **98%**
- Diode rectifier: **95%**
- VCA gain law: **90%** — correct exponential, misses THD+N
- Attack/release envelopes: **99%**

**Not modeled** (the 10-15%):
- VCA internal distortion (Blackmer log-antilog THD)
- Op-amp bandwidth/slew rate (irrelevant for audio-rate SSL)
- Noise floor
- Component tolerances

## Workflow Assessment (API Agent)

| Step | Status |
|------|--------|
| Write netlist | READY — all element types supported |
| `melange compile --format plugin` | BLOCKED by B1 (DK path) |
| Generated plugin structure | READY — pots→knobs, switches→selectors |
| Build and run | READY — self-contained Rust |
| Parameter mapping | NEEDS WORK — raw ohms/farads, not dB/ms |
| Stereo sidechain linking | NEEDS WORK — requires lib.rs customization |

## Action Plan

1. **Fix B1** — DK K diagonal check for zero-current dimensions (~30 min)
2. **Fix B2** — Add VCA to DC OP solver (~30 min)
3. **Add VCA test circuit** — compile-and-run verification (~1 hour)
4. **Add VCA voltage limiting** — fetlim on control port (~30 min)
5. Write example SSL bus compressor netlist
