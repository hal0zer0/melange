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

M-dimension: 2 per BJT (Vbe -> Ic, Vbc -> Ib)

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

## Triode (Koren Model)

M-dimension: 2 per triode (Vgk -> Ip, Vpk -> Ig)

### Plate Current
```
inner = Kp * (1/mu + Vgk / sqrt(Kvb + Vpk^2))
E1 = (Vpk / Kp) * ln(1 + exp(inner))
Ip = E1^ex / Kg1   (if E1 > 0, else 0)
```

### Grid Current (Leach)
```
Ig = ig_max * (vgk / vgk_onset)^1.5   for vgk > 0
Ig = 0                                 for vgk <= 0
```

### 12AX7 Parameters
```
mu = 100, Kp = 600, Kvb = 300, Kg1 = 1060, ex = 1.4
ig_max = 2e-3, vgk_onset = 0.5
```

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

## Temperature Dependence
```
VT = k*T/q  (thermal voltage)
IS ~ T^3 * exp(-EG/(kT))
```

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
