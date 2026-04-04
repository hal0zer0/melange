# Wurli Power Amp Codegen — Status Summary

**Date**: 2026-03-20
**Circuit**: `circuits/testing/wurli-power-amp.cir` (8-BJT quasi-complementary Class AB, ~20W)
**Result**: BLOCKED — DK M-dim NR cannot converge for this feedback topology. All codegen configs rail at ±10V. Use behavioral model.

## What Ships

```bash
melange compile circuits/testing/wurli-power-amp.cir -o power_amp.rs \
  --input-node out --output-node out
```

- **N=20, M=9** (7/8 BJTs forward-active reduced to 1D, Q9 Vbe multiplier remains 2D)
- **DC OP converges** via source stepping with internal nodes for parasitic BJTs
- **K_eff optimization**: Q9's parasitic R (RB=10, RC=0.5, RE=0.3) absorbed into K matrix as a 2×2 correction, eliminating the inner NR loop (was 15 iterations per outer NR step)
- **3.4× realtime** at 48kHz on x86-64 Linux (was 0.4× before K_eff + damping)
- **Stable**: internal state stays at supply rails (v_max = 22.5V) indefinitely

## What Changed (commits on main)

1. **`bjt_evaluate()`** — shared exp caching across Ic/Ib/Jacobian (2-3 exp instead of 8-10)
2. **`fast_exp` default** — polynomial approximation ~6 cycles (was ~38 libm)
3. **K_eff = K - R_p** — parasitic R absorbed into DK kernel, eliminates inner NR for Q9
4. **DC OP pre-computation** — computed on unexpanded MNA, passed via `from_kernel_with_dc_op()`
5. **A_neg zeroing fix** — explicit VS/VCVS row targeting (was range-based, broke internal nodes)
6. **Per-sample node damping** — 2V threshold, scales v + i_nl proportionally (was missing in codegen, present in runtime at 10V, now both at 2V)
7. **`--backward-euler` flag** — unconditionally stable integration (not needed for power amp with damping, available for future circuits)
8. **Full nodal LU fallback** — auto-detects ill-conditioned K, routes to N×N LU NR

## Architecture Notes for Plugin Integration

The generated `power_amp.rs` has the standard melange codegen API:
- `CircuitState::default()` — initializes with DC OP
- `process_sample(input: f64, state: &mut CircuitState) -> [f64; 1]`
- `state.set_sample_rate(rate)` — recomputes matrices at runtime
- DC blocking filter included by default (5Hz HPF, ±10V output clamp)

For the OpenWurli plugin, the power amp stage chains after the preamp:
```
preamp_out → power_amp input (node "out") → power_amp output (node "out")
```

The power amp's input node is the output node (feedback topology). The input conductance (1Ω default) models the Thevenin source at the feedback summing point.

## Performance Breakdown

| Component | Cost per sample | Notes |
|-----------|----------------|-------|
| build_rhs (A_neg × v_prev) | ~400 cycles | Sparse, ~80 nonzero terms |
| S × rhs (linear prediction) | ~800 cycles | Dense 20×20 matvec |
| 7× FA BJT eval | ~175 cycles | 1 fast_exp each |
| Q9 bjt_evaluate (K_eff) | ~50 cycles | 2-3 fast_exp, no inner NR |
| 9×9 Gauss elimination | ~2000 cycles | Forward + back sub |
| Voltage limiting + damping | ~300 cycles | pnjlim per device |
| S_NI × i_nl (final update) | ~360 cycles | 20×9 matvec |
| Node damping check | ~50 cycles | Rarely triggers at steady state |
| **Total (~4 NR iters)** | **~14,500 cycles** | **3.3× RT at 5GHz** |

## Known Limitations

- **No --no-dc-block stability without damping**: raw output drifts without the 2V node damping
- **NR sometimes maxes at 50 iterations**: ~2% of samples during hard transients, output still reasonable (damping prevents state divergence)
- **Q9 parasitic R is approximate**: K_eff linearizes parasitics around the DC OP. For extreme signal swings (clipping into output protection), the inner NR would be more accurate. For normal operation the error is sub-mV.
- **Not SPICE-validated yet**: DC OP matches ngspice (v(out)≈-0.065V), but transient correlation not tested
