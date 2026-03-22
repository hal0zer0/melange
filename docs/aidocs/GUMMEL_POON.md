# Gummel-Poon BJT Model

## Purpose

Extended BJT model adding Early effect (base-width modulation) and high-level
injection to the basic Ebers-Moll transport model. Automatically activated when
any GP parameter (VAF, VAR, IKF, IKR) is finite.

## Source Files

| Component | File | Lines |
|-----------|------|-------|
| `BjtParams` struct | `melange-solver/src/codegen/ir.rs` | 257-281 |
| `is_gummel_poon()` | `melange-solver/src/codegen/ir.rs` | 295-300 |
| `BjtGummelPoon` runtime | `melange-devices/src/bjt.rs` | 200-332 |
| `bjt_qb()` codegen template | `melange-solver/templates/rust/device_bjt.rs.tera` | 13-30 |
| `bjt_ic()` codegen template | `melange-solver/templates/rust/device_bjt.rs.tera` | 32-45 |
| `bjt_jacobian()` codegen template | `melange-solver/templates/rust/device_bjt.rs.tera` | 59-119 |
| DC OP integration | `melange-solver/src/dc_op.rs` | 115-141 |
| Parameter resolution | `melange-solver/src/codegen/ir.rs` | 970-1028 |

## Detection

```rust
fn is_gummel_poon(&self) -> bool {
    self.vaf.is_finite() || self.var.is_finite() ||
    self.ikf.is_finite() || self.ikr.is_finite()
}
```

If ALL GP params are `f64::INFINITY` (default), pure Ebers-Moll is used.
Codegen emits `DEVICE_{n}_USE_GP: bool` for compile-time branch elimination.

## Parameters

| Param | SPICE Alias | Description | Default | Unit |
|-------|-------------|-------------|---------|------|
| VAF | VA | Forward Early voltage | inf | V |
| VAR | VB | Reverse Early voltage | inf | V |
| IKF | JBF | Forward knee current | inf | A |
| IKR | JBR | Reverse knee current | inf | A |

Resolution chain: netlist `.model` -> SPICE alias -> catalog -> infinity.

## Base Charge Function (qb)

The core of Gummel-Poon. Modulates the transport current Icc by a normalized
base charge factor that accounts for Early effect and high-injection.

```
q1 = 1 / (1 - Vbe/VAR - Vbc/VAF)       [Early effect]

q2 = IS*exp(Vbe/VT)/IKF + IS*exp(Vbc/VT)/IKR   [High-injection]
     Note: q2 uses bare VT, not NF*VT or NR*VT.

qb = q1 * (1 + sqrt(1 + 4*q2)) / 2
```

Singularity guards:
- `|q1_denom| < 1e-30` -> qb = 1.0 (graceful fallback)
- `(1 + 4*q2).max(0.0)` -> discriminant floored at 0 (prevents sqrt of negative)
- `safe_exp()` clamps exponent to [-40, 40]

When GP is disabled (all params infinite): `qb = 1.0` (Ebers-Moll identity).

## Collector Current

```
Icc = IS * (exp(Vbe/Vt) - exp(Vbc/Vt))    [transport current]

Ic = sign * (Icc / qb - IS/beta_R * (exp(Vbc/Vt) - 1))
```

- `sign` = +1.0 (NPN) or -1.0 (PNP)
- Division by qb reduces Ic at high injection and near Early voltage
- The reverse saturation term (IS/beta_R) is NOT divided by qb

## Base Current

Forward ideal component `IS/BF * (exp(Vbe/(NF*VT)) - 1)` is divided by qb when GP
is active. Reverse component and ISE/ISC leakage are not modified.

```
Ib = sign * (IS/beta_F * (exp(Vbe/(NF*Vt)) - 1) / qb
           + IS/beta_R * (exp(Vbc/(NR*Vt)) - 1)
           + ISE * (exp(Vbe/(NE*Vt)) - 1)
           + ISC * (exp(Vbc/(NC*Vt)) - 1))
```

## Jacobian (2x2 Block)

The GP Jacobian requires the quotient rule through qb:

### Derivatives of qb

```
dq1/dVbe = q1^2 / VAR
dq1/dVbc = q1^2 / VAF

dq2/dVbe = IS / (VT * IKF) * exp(Vbe/VT)      [plain VT, not NF*VT]
dq2/dVbc = IS / (VT * IKR) * exp(Vbc/VT)      [plain VT, not NR*VT]

D = sqrt(1 + 4*q2)
dD/dVbe = 2 * dq2/dVbe / D       (if D > 1e-15, else 0)
dD/dVbc = 2 * dq2/dVbc / D

dqb/dVbe = dq1/dVbe * (1 + D) / 2 + q1 * dD/dVbe / 2
dqb/dVbc = dq1/dVbc * (1 + D) / 2 + q1 * dD/dVbc / 2
```

### Quotient Rule for Icc/qb

```
dIcc/dVbe = IS/Vt * exp(Vbe/Vt)
dIcc/dVbc = -IS/Vt * exp(Vbc/Vt)

qb2_safe = max(qb^2, 1e-30)

d(Icc/qb)/dVbe = (dIcc/dVbe * qb - Icc * dqb/dVbe) / qb2_safe
d(Icc/qb)/dVbc = (dIcc/dVbc * qb - Icc * dqb/dVbc) / qb2_safe
```

### Final Jacobian

```
dIc/dVbe = d(Icc/qb)/dVbe                              [no beta_R term]
dIc/dVbc = d(Icc/qb)/dVbc - IS/(beta_R * NR*Vt) * exp(Vbc/(NR*Vt))

ib_fwd = IS/beta_F * (exp(Vbe/(NF*Vt)) - 1)
dib_fwd/dVbe = IS/(beta_F * NF*Vt) * exp(Vbe/(NF*Vt))

d(ib_fwd/qb)/dVbe = (dib_fwd/dVbe * qb - ib_fwd * dqb/dVbe) / qb^2
                                                        [quotient rule]

dIb/dVbe = d(ib_fwd/qb)/dVbe                           [GP: forward divided by qb]
dIb/dVbc = IS/(beta_R * NR*Vt) * exp(Vbc/(NR*Vt))      [reverse unchanged]

bjt_jacobian returns [dIc/dVbe, dIc/dVbc, dIb/dVbe, dIb/dVbc]
```

## DC OP Integration

In `dc_op.rs`, when `bp.is_gummel_poon()`:
```rust
let gp = BjtGummelPoon::new(em, bp.vaf, bp.var, bp.ikf, bp.ikr);
i_nl[s] = gp.collector_current(vbe, vbc);      // GP-modulated
i_nl[s+1] = gp.base_current(vbe, vbc);         // GP-modulated (fwd/qb)
j_dev[s,s] = gp.jacobian[0];                   // GP Jacobian
j_dev[s,s+1] = gp.jacobian[1];
j_dev[s+1,s] = em.base_jacobian_dvbe;          // Standard EM
j_dev[s+1,s+1] = em.base_jacobian_dvbc;
```

## Catalog BJTs with GP Parameters

| Device | IS | BF | VAF | VAR | IKF | IKR |
|--------|----|----|-----|-----|-----|-----|
| 2N2222A | 1.26e-14 | 200 | 100V | 10V | 0.3A | 6mA |
| 2N3904 | 6.73e-15 | 416 | 74V | 28V | 66mA | — |
| 2N3906 | 1.41e-15 | 180 | 18V | 5.5V | 30mA | 11mA |
| BC547B | 1.8e-14 | 400 | 75V | — | 0.2A | — |

(— = infinity, pure EM for that parameter)

## Ebers-Moll Fallback

When `USE_GP = false`:
```rust
fn bjt_qb(...) -> f64 {
    if !use_gp { return 1.0; }  // Compile-time dead-code elimination
    ...
}
```

The compiler eliminates the entire GP code path. Zero runtime cost.

## Self-Heating (Optional, Quasi-Static)

When RTH is finite, junction temperature updated once per sample outside NR:
```
dTj/dt = (P_diss - (Tj - T_amb) / Rth) / Cth
P_diss = Vce * Ic + Vbe * Ib
IS(T) = IS_nom * (Tj/Tnom)^XTI * exp(EG/VT_nom * (Tj/Tnom - 1))
VT(T) = k * Tj / q
```

Forward Euler when `dt < 2*Rth*Cth`, implicit Euler otherwise.
Default RTH = infinity (disabled, zero overhead).

## Charge Storage (Optional, Linearized at DC OP)

Junction caps evaluated at DC operating point, stamped into MNA C matrix:
```
Depletion: Cj = CJ0 / (1 - Vj/VJ)^MJ           for Vj < FC*VJ
           Cj = CJ0/(1-FC)^(1+MJ) * (1-FC*(1+MJ)+MJ*Vj/VJ)  for Vj >= FC*VJ
Diffusion: Cd = TF * |Ic| / VT

B-E cap: CJE_linearized + Cd  stamped across base-emitter
B-C cap: CJC_linearized       stamped across base-collector
```

FC = 0.5 (SPICE3f5 default). Zero overhead when all charge params = 0.
