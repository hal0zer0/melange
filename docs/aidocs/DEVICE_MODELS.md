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
v_clamped = clamp(v_d, -100*N*VT, 100*N*VT)
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

### Plate Current
```
inner = Kp * (1/mu + Vgk / sqrt(Kvb + Vpk^2))
E1 = (Vpk / Kp) * ln(1 + exp(inner))
Ip = E1^ex / Kg1   (if E1 > 0, else 0)
```

### 12AX7 Parameters
```
mu = 100, Kp = 600, Kvb = 300, Kg1 = 1060, ex = 1.4
```

## OpAmp

### Linear Region
```
Vout = gain * (V+ - V-)
```

### Saturation
```
Vout = clamp(gain * Vdiff, Vout_min, Vout_max)
Jacobian = gain   (linear)
Jacobian = 0      (saturated)
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
- BJT 12AX7 triode: Ip ~ 1.2mA at Vgk=0, Vpk=250V
- Finite-difference Jacobian check: all device models have FD verification tests
