# SPICE-Style Voltage Limiting

## Purpose

Prevent Newton-Raphson overshoot on steep nonlinear I-V curves. Without limiting,
a single NR step can jump across the exponential knee of a PN junction, causing
oscillation or divergence. Reference: SPICE3f5 `DEVpnjlim` and `DEVfetlim`.

## Source Files

| Component | File |
|-----------|------|
| `pn_vcrit()` | `crates/melange-primitives/src/nr.rs` |
| `pnjlim()` | `crates/melange-primitives/src/nr.rs` |
| `fetlim()` | `crates/melange-primitives/src/nr.rs` |
| Codegen VCRIT emission (per device) | `crates/melange-solver/src/codegen/rust_emitter/dk_emitter.rs` |
| Codegen NR limiting (per-device dispatch + scalar α) | `crates/melange-solver/src/codegen/rust_emitter/nr_helpers.rs` |
| Self-contained template (emitted into generated code) | `crates/melange-solver/templates/rust/spice_limiting.rs.tera` |

Line numbers omitted — grep the symbol names. Note that the runtime
dispatch path that used to live in `melange-solver/src/solver.rs` has been
removed; voltage limiting now exists exclusively in the codegen pipeline
and the small primitive functions in `melange-primitives`.

## Critical Voltage (pn_vcrit)

Precomputed per device. Marks the voltage above which the exponential curve
becomes steep enough to cause NR overshoot.

```
vcrit = Vt * ln(Vt / (sqrt(2) * IS))
```

Where:
- `Vt` = device thermal voltage (n * kT/q for diodes, vt for BJTs)
- `IS` = saturation current

Example values (Vt = VT_ROOM = 0.025852 V = kT/q at 300 K; n applied for diodes,
BJT junctions use vt directly):
```
1N4148 (IS=2.52e-9, n=1.752):  vcrit = 0.741V
Generic silicon (IS=1e-12, n=1.5):  vcrit = 0.932V
BJT 2N2222A (IS=1.26e-14):  vcrit = 0.724V
```
(1N4148 and 2N2222A params are the shipping melange catalog cards; see
`crates/melange-devices/src/catalog/`.)

Emitted as `DEVICE_{n}_VCRIT` constant in generated code.

## PN Junction Limiting (pnjlim)

Used for: diodes, BJT junctions (both Vbe and Vbc), tube grid current dimension.

```
pnjlim(vnew, vold, vt, vcrit) -> f64:
  if vnew > vcrit AND |vnew - vold| > 2*vt:
    if vold >= 0:
      arg = 1 + (vnew - vold) / vt
      if arg > 0: return vold + vt * ln(arg)
      else:       return vcrit
    else:
      return vt * ln(vnew / vt)
  else:
    return vnew  (no limiting applied)
```

**Behavior**: Logarithmic compression of large forward voltage steps. Small steps
(< 2*Vt ~52mV) pass unchanged. Large steps compressed logarithmically — a proposed
300V jump becomes ~0.3V.

## FET Voltage Limiting (fetlim)

Used for: JFETs (Vgs around pinch-off), MOSFETs (Vgs around threshold),
tube plate voltage (Vpk around 0V).

```
fetlim(vnew, vold, vto) -> f64:      // SPICE3f5 DEVfetlim (Gillespie vtstlo fix)
  delv = vnew - vold
  vtox = vto + 3.5
  vtsthi = 2 * |vold - vto| + 2.0
  vtstlo = |vold - vto| + 1.0        // = vtsthi / 2

  if vold >= vto:                    // Device ON
    if vold >= vtox:                 // Far on
      if delv <= 0:                  // Going off
        if vnew >= vtox:
          if |delv| > vtstlo: return vold - vtstlo
          else: return vnew
        else: return max(vnew, vto + 2.0)
      else:                          // Staying on
        if delv >= vtsthi: return vold + vtsthi
        else: return vnew
    else:                            // Near threshold
      if delv <= 0: return max(vnew, vto - 0.5)
      else: return min(vnew, vto + 4.0)
  else:                              // Device OFF
    if delv <= 0:
      if |delv| > vtsthi: return vold - vtsthi    // NOTE: vtsthi here, not vtstlo
      else: return vnew
    else if vnew <= vto + 0.5:
      if delv > vtstlo: return vold + vtstlo
      else: return vnew
    else:
      return vto + 0.5
```

**Behavior**: Prevents gate/drain voltage from jumping across the threshold boundary
in a single NR step. Adaptive step limits scale with distance from threshold.

## Per-Device Limiter Dispatch

| Device | Dim 0 | Dim 1 |
|--------|-------|-------|
| Diode | pnjlim(v, vold, n_vt, vcrit) | — |
| BJT | pnjlim(Vbe, vold, vt, vcrit) | pnjlim(Vbc, vold, vt, vcrit) |
| JFET | fetlim(Vds, vold, 0.0) | fetlim(Vgs, vold, Vp) |
| MOSFET | fetlim(Vds, vold, 0.0) | fetlim(Vgs, vold, Vt) |
| Tube | pnjlim(Vgk, vold, vgk_onset/3, vcrit) | fetlim(Vpk, vold, 0.0) |

## Scalar Damping Factor (Alpha)

Voltage limiting operates in voltage space, but the NR update is in current space.
A scalar damping factor `alpha` bridges the two:

```
1. Compute NR delta in current space: J * delta = residual
2. For each device dimension i:
   dv[i] = -sum_j K[i][j] * delta[j]       // implied voltage change
   if |dv[i]| > 1e-15:
     v_proposed = v_old[i] + dv[i]
     v_limited = limiter(v_proposed, v_old[i])
     ratio = (v_limited - v_old[i]) / dv[i]
     ratio = max(ratio, 0.01)               // prevent backward steps
     alpha = min(alpha, ratio)
3. Apply uniformly: i_nl -= alpha * delta    // same alpha for all dimensions
```

**Key property**: Alpha is scalar (same for all M dimensions). This preserves
NR consistency — the step direction is maintained, only the magnitude is reduced.
The 0.01 floor prevents pathological backward steps.

## Global Node-Step Damping (nodal full-LU, Layer 2)

Beyond the per-device alpha above, the nodal full-LU emitter applies a second,
global limit on the maximum *node-voltage* change per NR iteration — in both the
primary loop and the Backward Euler fallback (`nodal_emitter.rs`):

```
if max_node_dv > damp_thresh {          // damp_thresh = 10.0.max(0.05 * max_v)
    alpha *= damp_thresh / max_node_dv; // uncapped — bounds the step to damp_thresh
}
```

**There is deliberately NO floor on this ratio.** A `.max(0.01)` floor lived here
until 2026-08-03 and was a bug: unlike the per-device ratio above, `damp_thresh /
max_node_dv` is always strictly positive, so a backward step is impossible and a
floor serves no anti-stall purpose — it only *defeats* the ceiling. A floored
ratio still lets a fixed fraction (≥1%) of an arbitrarily large raw delta through,
so a pathological LU delta (observed 3.8e7 V at a device-state transition on
wurli-power-amp) sailed past the ≤`damp_thresh` cap as a multi-kV jump and
false-converged on a nonphysical operating point. Uncapped division bounds every
iteration's worst-case node step at exactly `damp_thresh` regardless of the raw
delta's magnitude. See DEBUGGING.md (2026-08-03) and
`nodal_be_fallback_alpha_floor_tests.rs`. **Do not reintroduce a floor here** —
if NR stalls, the fix is elsewhere (limiter/vcrit), not a bigger minimum step.

## Codegen Implementation

Generated code emits self-contained `pnjlim()` and `fetlim()` functions via
`spice_limiting.rs.tera` template (marked `#[inline(always)]`). These are
duplicated from `melange-primitives/src/nr.rs` because generated code must be
self-contained with no external crate dependencies.

VCRIT constants are precomputed at code-generation time:
```rust
const DEVICE_0_VCRIT: f64 = 0.6145;  // from pn_vcrit(vt, is)
```

## Convergence Check

After applying the damped step, convergence is checked on the **actual** step size:
```
converged = max_i(|alpha * delta[i]|) < TOLERANCE
```

This means convergence tracks the effective step, not the raw Newton step.

## Common Bugs

| Symptom | Cause | Fix |
|---------|-------|-----|
| NR oscillation (period-2) | No voltage limiting | Add pnjlim/fetlim |
| NR converges but wrong point (per-device) | Alpha floor too high | Check 0.01 minimum |
| Internal nodes diverge to kV, then false-converge | Node-step damping floored (`.max(0.01)`) | Bound the step, not the ratio — divide uncapped (fixed 2026-08-03) |
| Slow convergence (>50 iters) | Limiting too aggressive | Check vcrit computation |
| Explosion after removing a device | Missing limiter dispatch | Add case to `limit_voltage()` |
