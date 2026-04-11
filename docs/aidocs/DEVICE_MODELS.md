# Nonlinear Device Models

## Diode (Shockley)

### Current
```
i_d = IS * (exp(v_d / (N*VT)) - 1)
VT = 26mV @ room temp (25C)
```

### Conductance
```
g_d = di/dv = (IS/(N*VT)) * exp(v_d/(N*VT))
            = (i_d + IS) / (N*VT)
```

### Clamping (prevent overflow)
```
v_clamped = clamp(v_d, -40*N*VT, 40*N*VT)
```

### Named Constructors
```
silicon()          IS=1e-12, n=1.5    (general purpose)
silicon_1n4148()   IS=2.52e-9, n=1.752 (signal diode)
silicon_ideal()    IS=1e-14, n=1.0    (textbook ideal)
```

## BJT (Ebers-Moll)

M-dimension: **2 per BJT** for the standard `Bjt` device type
(Vbe → Ic at `start_idx`, Vbc → Ib at `start_idx+1`).

### BJT Mode Variants (M-Dimension Reduction)

The MNA builder can flag a BJT as **forward-active** (auto-detected at DC OP)
or **linearized**, which changes its NR dimension:

| Variant | Source | NR Dim | Behavior |
|---------|--------|--------|----------|
| `Bjt` (default) | netlist | 2 | Full Ebers-Moll / Gummel-Poon, both junctions in NR |
| `BjtForwardActive` | DC OP detects strong forward bias | 1 | Only Vbe→Ic in NR; `Ib = Ic / β_F` derived from Ic |
| `linearized_bjts` (`mna.linearized_bjts`) | DC OP detects fully-biased linear region | 0 | Removed from NR entirely; small-signal `g_m`/`g_pi`/`r_o` stamped into G after DC OP |

This is why the wurli-power-amp circuit reports `M=16→9` when it compiles:
the 8 BJTs start at 16 nominal NR dimensions, but 7 of them are detected as
forward-active and drop to 1D each (`16 - 7 = 9`). Linearized BJTs would drop
the count further to 0 per device.

### Collector Current
```
Ic = IS * (exp(Vbe/VT) - exp(Vbc/VT)) - IS/beta_R * (exp(Vbc/VT) - 1)
```

### Base Current
```
Ib = IS/beta_F * (exp(Vbe/VT) - 1) + IS/beta_R * (exp(Vbc/VT) - 1)
```

### Device Jacobian (2x2)
```
[dIc/dVbe  dIc/dVbc]   [IS/VT * exp(Vbe/VT)                    -(IS/VT + IS/(beta_R*VT)) * exp(Vbc/VT)]
[dIb/dVbe  dIb/dVbc] = [IS/(beta_F*VT) * exp(Vbe/VT)           IS/(beta_R*VT) * exp(Vbc/VT)           ]
```

### Gummel-Poon (more accurate)
Uses charge control model with Early effect. Jacobian computed via quotient
rule through qb() function. Available in `crates/melange-devices/src/bjt.rs`.

### Typical Parameters (2N2222A)
```
IS = 1.26e-14
beta_F = 200
beta_R = 3
```

### Self-Heating (Electrothermal Model)

Optional quasi-static thermal model that updates junction temperature once per sample,
outside the NR loop.

```
dTj/dt = (P_diss - (Tj - T_amb) / Rth) / Cth
```

- `P_diss = Vce * Ic + Vbe * Ib` (instantaneous power dissipation)
- Forward Euler when stable (`dt < 2 * Rth * Cth`), implicit Euler otherwise
- Temperature-dependent saturation current (SPICE3f5 exact):
  `IS(T) = IS_nom * (Tj/Tnom)^XTI * exp(EG/VT_nom * (Tj/Tnom - 1))`
- `VT(T) = k * Tj / q` (thermal voltage tracks junction temperature)
- Zero overhead when disabled (RTH = infinity, the default)

#### Parameters
```
RTH   Thermal resistance [K/W]  (default: inf = disabled)
CTH   Thermal capacitance [J/K] (default: 0)
XTI   IS temperature exponent   (default: 3.0)
EG    Bandgap energy [eV]       (default: 1.11)
```

### Charge Storage Dynamics

BJT junction capacitances linearized at the DC operating point and stamped into the
MNA C matrix. The DK framework requires linear C, so caps are fixed at DC OP values.
Zero overhead when all charge params are 0 (default).

#### Depletion Capacitance (SPICE3f5)
```
Cj = CJ0 / (1 - Vj/VJ)^MJ     for Vj < FC * VJ
Cj = CJ0 / (1-FC)^(1+MJ) * (1 - FC*(1+MJ) + MJ*Vj/VJ)   for Vj >= FC * VJ
```
Where FC = 0.5 (forward-bias coefficient for linear extension).

#### Diffusion Capacitance
```
Cd = TF * |Ic| / VT
```

#### Parameters
```
TF    Forward transit time [s]       (default: 0)
CJE   B-E zero-bias junction cap [F] (default: 0)
VJE   B-E built-in potential [V]     (default: 0.75)
MJE   B-E grading coefficient        (default: 0.33)
CJC   B-C zero-bias junction cap [F] (default: 0)
VJC   B-C built-in potential [V]     (default: 0.75)
MJC   B-C grading coefficient        (default: 0.33)
```

## Triode (Koren Model)

M-dimension: 2 per triode (Vgk -> Ip, Vpk -> Ig)

### Plate Current (with Early-effect lambda)
```
inner = Kp * (1/mu + Vgk / sqrt(Kvb + Vpk^2))
E1 = (Vpk / Kp) * ln(1 + exp(inner))
Ip_koren = E1^ex / Kg1   (if E1 > 0, else 0)
Ip = Ip_koren * (1 + lambda * Vpk)
```

The `lambda` parameter models channel-length modulation (Early effect) for triodes,
analogous to MOSFET/JFET lambda. When `lambda = 0.0` (default), the model reduces
to the standard Koren equation (backward compatible). The plate resistance at the
operating point is approximately `rp = 1 / (lambda * Ip)`.

### Jacobian with lambda
```
dIp/dVgk = dIp_koren/dVgk * (1 + lambda * Vpk)
dIp/dVpk = dIp_koren/dVpk * (1 + lambda * Vpk) + Ip_koren * lambda
```

### Grid Current (Leach)
```
Ig = ig_max * (vgk / vgk_onset)^1.5   for vgk > 0
Ig = 0                                 for vgk <= 0
```

### 12AX7 Parameters
```
mu = 100, Kp = 600, Kvb = 300, Kg1 = 1060, ex = 1.4
ig_max = 2e-3, vgk_onset = 0.5, lambda = 0.0
```

## Pentode / Beam Tetrode (Reefman "Derk" §4.4)

M-dimension: **3 per pentode** (Vgk → Ip, Vpk → Ig2, Vg2k → Ig1)

Uses Reefman's "Derk" pentode equations from *SPICE models for vacuum tubes
using the uTracer* (2016, §4.4). Parameter fits are carried verbatim from
Reefman's `TubeLib.inc` where available (EL84, EL34, EF86 at phase 1a).

**Controlling voltages**: Vgk (grid–cathode), Vpk (plate–cathode), Vg2k
(screen grid g2 – cathode).

**Emitted currents**: Ip (plate), Ig2 (screen grid), Ig1 (control grid,
which reuses the triode's Leach power-law).

### Plate + Screen Currents

The plate and screen currents share a single "Koren current" `Ip0` built
from the grid-cathode and screen-cathode voltages:

```
inner = Kp * (1/mu + Vgk / sqrt(Kvb + Vg2k^2))
E1    = (Vg2k / Kp) * softplus(inner)        // softplus = ln(1 + exp(x))
Ip0   = E1^Ex / 2 * (1 + sgn(E1))            // C^infinity via softplus
```

`Ip0` is then scaled by plate-voltage-dependent factors `F` and `H`:

```
alpha = 1 - (Kg1/Kg2) * (1 + alpha_s)        // derived, not fitted

F(Vpk) = 1/Kg1 - 1/Kg2 + (A * Vpk)/Kg1
         - (alpha/Kg1 + alpha_s/Kg2) / (1 + beta * Vpk)

H(Vpk) = (1 + alpha_s / (1 + beta * Vpk)) / Kg2

Ip  = Ip0 * F(Vpk)    // plate current
Ig2 = Ip0 * H(Vpk)    // screen current
```

**Note on parameter naming**: Reefman's paper uses `alpha_s`, `A`, `beta`.
In melange's `TubeParams` they're stored as `alpha_s`, `a_factor`,
`beta_factor` (the `_factor` suffixes avoid collisions with SPICE AC-analysis
syntax and the Greek letter β). The `.model VP(...)` directive accepts
`ALPHA_S`, `A_FACTOR`, `BETA_FACTOR` as the parameter keywords.

**Required**: `alpha_s > 0`. The Derk model degenerates to `Ip = 0`
identically when `alpha_s = 0` (not a graceful Koren fallback — verified
by symbolic expansion). `TubeParams::validate()` rejects at config time.

**Allowed**: `a_factor >= 0` and `beta_factor >= 0`. Some fits pinpoint
`a_factor` or `beta_factor` at zero; only `alpha_s` is load-bearing.

### Grid Current (reuse Leach)

```
Ig1 = ig_max * (Vgk / vgk_onset)^1.5    for Vgk > 0
Ig1 = 0                                  otherwise
```

Identical to the triode. Pentode codegen reuses `tube_ig` / `tube_ig_deriv`
without modification.

### 3×3 Analytic Jacobian

Rows: `[Ip, Ig2, Ig1]`. Cols: `[Vgk, Vpk, Vg2k]`.

Chain-rule prefactors (compute once per NR iteration):
```
s         = sqrt(Kvb + Vg2k^2)
sigma     = sigmoid(inner)
sp        = softplus(inner)
dE1/dVgk  = Vg2k * sigma / s
dE1/dVpk  = 0
dE1/dVg2k = sp/Kp - sigma * Vgk * Vg2k^2 / s^3
dIp0/dE1  = Ex * E1^(Ex-1) / 2 * (1 + sgn(E1))

dF/dVpk   = A/Kg1 + beta * (alpha/Kg1 + alpha_s/Kg2) / (1 + beta*Vpk)^2
dH/dVpk   = -beta * alpha_s / (Kg2 * (1 + beta*Vpk)^2)
```

Jacobian entries:
```
dIp/dVgk   = dIp0/dE1 * dE1/dVgk  * F(Vpk)
dIp/dVpk   = Ip0 * dF/dVpk
dIp/dVg2k  = dIp0/dE1 * dE1/dVg2k * F(Vpk)

dIg2/dVgk  = dIp0/dE1 * dE1/dVgk  * H(Vpk)
dIg2/dVpk  = Ip0 * dH/dVpk
dIg2/dVg2k = dIp0/dE1 * dE1/dVg2k * H(Vpk)

dIg1/dVgk  = 1.5 * ig_max * sqrt(Vgk/vgk_onset) / vgk_onset    // Vgk>0
dIg1/dVpk  = 0
dIg1/dVg2k = 0
```

Sparsity: the Ig1 row has a single nonzero entry; `dIg2/dVpk` is small but
nonzero. `dIp/dVgk` / `dIp/dVg2k` share `dIp0/dE1 * dE1/d_ * F` as a
prefactor — compute once per NR iteration and reuse.

### NR-Stability Guards (same pattern as triode)

```
Vg2k <- max(Vg2k, 1e-3)       // prevent div-by-zero in E1 softplus
Vpk  <- max(Vpk, 0.0)         // prevent 1/(1 + beta*Vpk) singularity
inner overflow clamp: { inner if inner > 20, 0 if inner < -20, softplus otherwise }
E1 <= 1e-30:                  // deep cutoff — return zero currents and zero Jacobian
```

### Catalog Parameters (from Reefman TubeLib.inc 2016-01-23)

| Tube | μ | Ex | Kg1 | Kg2 | Kp | Kvb | αs | A | β | Source |
|------|---|-----|-----|-----|-----|-----|-----|---|---|--------|
| EL84 / 6BQ5 | 23.36 | 1.138 | 117.4 | 1275.0 | 152.4 | 4015.8 | 7.66 | 4.344e-4 | 0.148 | BTetrodeD |
| EL34 / 6CA7 | 12.50 | 1.363 | 217.7 | 1950.2 | 50.5 | 1282.7 | 6.09 | 3.48e-4 | 0.105 | BTetrodeD |
| EF86 / 6267 | 40.8 | 1.327 | 675.8 | 4089.6 | 350.7 | 1886.8 | 4.24 | 5.95e-5 | 0.28 | PenthodeD |

Catalog aliases: `EL84-P`, `EL34-P`, `EF86`. The existing triode-connected
`EL84` / `EL34` / `6L6` / `6V6` catalog entries remain available for
backward compat.

### Deferred

- **Beam tetrodes** (6L6, 6V6, KT88) need the Reefman DerkE §4.5 screen
  form `Ig2 = Ip0 * (1 + αs * exp(-β * Vpk^{3/2})) / Kg2` instead of
  the rational `1/(1 + β·Vpk)` of §4.4. Phase 1a.1.
- **Grid-off FA reduction**: pentode 3D → 2D when Vgk < 0 (Ig1 = 0).
  Phase 1b, analogous to BJT forward-active dimension reduction.
- **Remote-cutoff / variable-mu** (6BA6, 6386, 6SK7, 6BC8): two-section
  Koren per Reefman §5. Phase 1c, for varimu compressor targets
  (Fairchild 670, Sta-Level, Collins 26U).

## JFET (Shichman-Hodges)

M-dimension: 2 per JFET (Vgs,Vds -> Id, Vgs -> Ig)

### Drain Current (2D)
```
Vgst = Vgs - Vp  (gate overdrive)

Triode (|Vds| < |Vgst|):
  Id = sign * IDSS * (2*Vgst*Vds - Vds^2) / Vp^2 * (1 + lambda*|Vds|)

Saturation (|Vds| >= |Vgst|):
  Id = sign * IDSS * (1 - Vgs/Vp)^2 * (1 + lambda*|Vds|)

Cutoff (Vgst <= 0): Id = 0
```

### Gate Current
```
Ig ≈ 0  (insulated gate approximation)
```

### Device Jacobian (2x2)
```
[dId/dVgs  dId/dVds]
[dIg/dVgs  dIg/dVds]
```
dIg/dVgs and dIg/dVds are effectively zero.

### Sign Convention
- N-channel (NJ): sign=+1.0, default VTO=-2.0, Vp negative
- P-channel (PJ): sign=-1.0, default VTO=+2.0, Vp positive
- Default IDSS=2e-3 A, lambda=0.001

## MOSFET (Level 1 SPICE)

M-dimension: 2 per MOSFET (Vgs,Vds -> Id, Ig=0)

### Drain Current (2D)
```
Vgst = Vgs - Vt  (gate overdrive)

Triode (|Vds| < |Vgst|):
  Id = sign * KP * (2*Vgst*Vds - Vds^2) * (1 + LAMBDA*|Vds|)

Saturation (|Vds| >= |Vgst|):
  Id = sign * KP * Vgst^2 * (1 + LAMBDA*|Vds|)

Cutoff (Vgst <= 0): Id = 0
```

### Gate Current
```
Ig = 0  (insulated gate — no gate current)
```

### Device Jacobian (2x2)
```
[dId/dVgs  dId/dVds]
[dIg/dVgs  dIg/dVds]
```
Gate current derivatives are always zero.

### Sign Convention
- N-channel (NM): sign=+1.0, default VTO=2.0, KP=0.1
- P-channel (PM): sign=-1.0, default VTO=-2.0
- Channel-length modulation via LAMBDA parameter

## OpAmp (Linear VCCS)

> **For finite-rail op-amps (`.model OA(VCC=… VEE=…)`)**, this linear VCCS path
> is only the IDEAL fallback. Rail clamping is handled by one of 5 modes
> (`None / Hard / ActiveSet / ActiveSetBe / BoyleDiodes`) auto-selected by
> `codegen::ir::resolve_opamp_rail_mode`. **Read [OPAMP_RAIL_MODES.md](OPAMP_RAIL_MODES.md)
> before touching anything related to op-amp saturation, BoyleDiodes, or rail
> clamping** — the rail-mode landscape has multi-session investigation history
> and several already-tested-and-rejected fix candidates.

The op-amp is modeled as a linear voltage-controlled current source (VCCS).
It does NOT add nonlinear dimensions (M stays unchanged). All behavior is
captured by stamps in the G matrix.

### G Matrix Stamps
```
Gm = AOL / ROUT    (transconductance)
Go = 1 / ROUT      (output conductance)

G[out, n_plus]  += Gm
G[out, n_minus] -= Gm
G[out, out]     += Go
```

### Defaults
```
AOL = 200000  (open-loop gain)
ROUT = 1 ohm  (output resistance)
```

### Slew-Rate Limiting (`SR` parameter)

Real op-amps cannot change output voltage faster than their input-stage
tail current allows through the dominant-pole compensation cap. melange
models this large-signal distortion via the `SR` parameter in the `.model`
card:

```
.model OA_TL072 OA(AOL=200000 ROUT=75 GBW=3Meg VSAT=13 SR=13)
```

- **Unit**: `SR` is specified in V/μs (SPICE convention), converted to V/s
  internally (`SR = sr_v_per_us * 1e6`). Examples: TL072 = 13 V/μs → 13e6;
  NE5532 = 9 V/μs → 9e6; LM358 = 0.3 V/μs → 0.3e6.
- **Default**: `f64::INFINITY` (no slew limiting, byte-identical generated
  code to circuits without `SR=`).
- **Emission**: a per-sample voltage-delta clamp on the op-amp output node:
  ```rust
  let prev = state.v_prev[OUT];
  let max_dv = OA{idx}_SR * (1.0 / state.current_sample_rate);
  let delta = v[OUT] - prev;
  v[OUT] = prev + delta.clamp(-max_dv, max_dv);
  ```
  Mathematically equivalent to clamping the Boyle dominant-pole integrator
  input current at `±I_slew = ±SR * C_dom`, since the integrator's
  per-sample voltage step is `Δv = (i_in * dt) / C_dom`, so capping
  `|Δv| ≤ SR * dt` caps `|i_in| ≤ SR * C_dom`.
- **Emitted in all three codegen paths**: DK Schur (via the Tera
  `process_sample.rs.tera` template), nodal Schur, and nodal full-LU.
  Applied AFTER rail clamping and the damping safety net, BEFORE
  `state.v_prev = v` so the next sample's cap history reflects the
  slew-limited voltage.
- **Rail-mode interaction**: compatible with `None`, `Hard`, `ActiveSet`,
  `ActiveSetBe`, and `BoyleDiodes`. BoyleDiodes heavy-clip convergence
  issues (see [`OPAMP_RAIL_MODES.md`](OPAMP_RAIL_MODES.md)) are unrelated
  to slew rate — SR neither helps nor hurts BoyleDiodes at heavy clip.
- **Parser validation**: `SR <= 0` is rejected at parse time with a
  `ParseError`.

## VCA (Voltage-Controlled Amplifier)

### Blackmer Model (THAT 2180 / DBX 2150)

Current-mode exponential gain element. The THAT 2180 is a log-antilog
VCA where gain is exponentially controlled by a voltage:

### Gain Law
```
G(Vc) = G0 * exp(-Vc / Vscale)
I_signal = G(Vc) * (V_sig + thd_factor * V_sig³)
I_control = 0  (high-impedance control input)

where thd_factor = THD * (1 - min(G(Vc), 1.0))
```

### Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| VSCALE | 0.05298 | V/neper | Control voltage scaling. THAT 2180A: 6.1 mV/dB = 6.1e-3 / (ln(10)/20) |
| G0 | 1.0 | S | Unity-gain conductance |
| THD | 0.0 | - | Gain-dependent distortion coefficient (0 = ideal, 0.002 typical for THAT 2180A) |

### Conversion: mV/dB to V/neper
```
VSCALE = sensitivity_mV_per_dB * 1e-3 / (ln(10) / 20)
```
For THAT 2180A (6.1 mV/dB): VSCALE = 6.1e-3 / 0.11513 = 0.05298

### Device Jacobian (2x2)
```
[dI_sig/dV_sig,   dI_sig/dV_ctrl]   [G*(1+3*thd*V²),  -G*V_thd/Vscale]
[dI_ctrl/dV_sig,  dI_ctrl/dV_ctrl] = [0,                0              ]
```

### Netlist Syntax
```spice
Y1 sig+ sig- ctrl+ ctrl- modelname
.model modelname VCA(VSCALE=0.05298 G0=1.0 THD=0.002)
```

### N_v / N_i Stamping (4-terminal)
```
N_v[i][sig+] = +1, N_v[i][sig-] = -1       (signal voltage)
N_v[i+1][ctrl+] = +1, N_v[i+1][ctrl-] = -1 (control voltage)
N_i[sig+][i] = -1, N_i[sig-][i] = +1       (signal current)
N_i[:][i+1] = 0                              (control draws no current)
```

## Temperature Dependence
```
VT = k*T/q  (thermal voltage)
IS ~ T^3 * exp(-EG/(kT))
```

## Parasitic Cap Auto-Insertion

When a nonlinear circuit has zero capacitors in the netlist, `MnaSystem::add_parasitic_caps()`
inserts 10pF (`PARASITIC_CAP = 10e-12`) across each physical device junction:

| Device | Junctions |
|--------|-----------|
| Diode | anode-cathode (Cak) |
| BJT | base-emitter (Cje) + base-collector (Cjc) |
| JFET | gate-source (Cgs) + gate-drain (Cgd) |
| MOSFET | gate-source (Cgs) + gate-drain (Cgd) |
| Tube | grid-cathode (Cgk) + plate-cathode (Cpk) |

Caps are stamped *across junctions* (not node-to-ground) to model physical junction
capacitance. This ensures the C matrix is non-trivial, preventing the trapezoidal-rule
A matrix from becoming singular for purely resistive nonlinear circuits.

A `log::warn!` is emitted when auto-insertion occurs.

## Jacobian Contributions to NR System

### 1D Device (diode)
```
jdev_{d}_{d} = g_d = diode_conductance(v_d)
NR Jacobian row: j_{dj} = delta_{dj} - jdev_{d}_{d} * K[d][j]
```

### 2D Device (BJT)
```
jdev_{s}_{s}   = dIc/dVbe    jdev_{s}_{s+1}   = dIc/dVbc
jdev_{s+1}_{s} = dIb/dVbe    jdev_{s+1}_{s+1} = dIb/dVbc

NR Jacobian row s:   j_{sj} = delta_{sj} - jdev_{s}_{s}*K[s][j] - jdev_{s}_{s+1}*K[s+1][j]
NR Jacobian row s+1: j_{s+1,j} = delta_{s+1,j} - jdev_{s+1}_{s}*K[s][j] - jdev_{s+1}_{s+1}*K[s+1][j]
```

## Verification
- Diode: current = 0 at v=0, forward voltage ~0.6-0.7V at 1mA
- Conductance always positive (passive device)
- g_d ~ IS/VT at small v (linear region)
- 12AX7 triode: Ip ~ 1.2mA at Vgk=0, Vpk=250V
- Finite-difference Jacobian check: all device models have FD verification tests
