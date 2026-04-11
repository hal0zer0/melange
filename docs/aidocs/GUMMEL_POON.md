# Gummel-Poon BJT Model

## Purpose

Extended BJT model adding Early effect (base-width modulation) and high-level
injection to the basic Ebers-Moll transport model. Automatically activated when
any GP parameter (VAF, VAR, IKF, IKR) is finite.

## Source Files

| Component | File |
|-----------|------|
| `BjtParams` struct | `crates/melange-solver/src/device_types.rs` |
| `is_gummel_poon()` method | `crates/melange-solver/src/device_types.rs` |
| `BjtGummelPoon` device model | `crates/melange-devices/src/bjt.rs` |
| `bjt_qb()` codegen template | `crates/melange-solver/templates/rust/device_bjt.rs.tera` |
| `bjt_ic()` codegen template | `crates/melange-solver/templates/rust/device_bjt.rs.tera` |
| `bjt_jacobian()` codegen template | `crates/melange-solver/templates/rust/device_bjt.rs.tera` |
| DC OP integration (qb-aware solve) | `crates/melange-solver/src/dc_op.rs` |
| Parameter resolution | `crates/melange-solver/src/codegen/ir.rs` |

Line numbers omitted intentionally — they drift with every refactor. Grep
the symbol name (`BjtParams`, `is_gummel_poon`, `bjt_qb`, etc.) to locate
the current definition. Symbol names are stable; line numbers are not.

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

The core of Gummel-Poon. Modulates the transport (collector) current Icc by a
normalized base charge factor that accounts for Early effect and high-injection.

This implementation follows ngspice `bjtload.c` (SPICE3f5 descendant) line by
line. The two details below are the ones most people get wrong:

```
q1 = 1 / (1 - Vbe/VAR - Vbc/VAF)             [Early effect]

cbe = IS * (exp(Vbe/(NF*Vt)) - 1)            [forward junction current]
cbc = IS * (exp(Vbc/(NR*Vt)) - 1)            [reverse junction current]

q2 = cbe / IKF + cbc / IKR                   [high-level injection, ngspice bjtload.c:571]

qb = q1 * (1 + sqrt(1 + 4*q2)) / 2
```

**Critical:** q2 inherits the NF / NR emission coefficients via cbe / cbc, and
each exponential has the `-1` term. Melange historically computed `q2 =
IS*exp(Vbe/Vt)/IKF + IS*exp(Vbc/Vt)/IKR` using bare VT and no `-1`; that was
wrong and has been fixed. Any BJT `.model` card with `NF != 1` (e.g. BC547 at
NF = 1.008, or vintage Neve cards at NF ≈ 1.02) would otherwise silently
diverge from ngspice by several percent at forward bias above the IKF knee.

Singularity guards:
- `q1_denom <= 0.0 || |q1_denom| < 1e-30` -> qb = 1.0 (prevents sign-flip near Early voltage)
- `(1 + 4*q2).max(0.0)` -> discriminant floored at 0 (prevents sqrt of negative)
- `safe_exp()` clamps exponent to [-40, 40]

When GP is disabled (all params infinite): `qb = 1.0` (Ebers-Moll identity).

## Collector Current

```
cex = cbe = IS * (exp(Vbe/(NF*Vt)) - 1)            [forward, no excess phase]
cbc = IS * (exp(Vbc/(NR*Vt)) - 1)                  [reverse]

Ic = sign * ((cex - cbc) / qb - cbc / beta_R)      [ngspice bjtload.c:617]
```

Equivalently, using `Icc = IS*(exp_be - exp_bc) = cbe - cbc`:

```
Ic = sign * (Icc / qb - IS/beta_R * (exp(Vbc/(NR*Vt)) - 1))
```

- `sign` = +1.0 (NPN) or -1.0 (PNP)
- Division by qb reduces Ic at high injection and near Early voltage
- The reverse saturation term (IS/beta_R) is NOT divided by qb

## Base Current

```
Ib = sign * (IS/beta_F * (exp(Vbe/(NF*Vt)) - 1)      [ideal forward, ngspice bjtload.c:618]
           + IS/beta_R * (exp(Vbc/(NR*Vt)) - 1)      [ideal reverse]
           + ISE * (exp(Vbe/(NE*Vt)) - 1)            [B-E leakage]
           + ISC * (exp(Vbc/(NC*Vt)) - 1))           [B-C leakage]
```

**The base current is NOT divided by qb in Gummel-Poon.** ngspice at line 618
computes `cb = cbe/betaF + cben + cbc/betaR + cbcn` — the qb modulation is on
transport (collector) current only, via the `cc` assignment at line 617.

Melange previously divided the forward ideal component `IS/beta_F * (...)` by
qb in GP mode, which is a different (BSIM-flavored) GP variant and will not
agree with ngspice at high injection. That has been fixed; the runtime
library and the codegen template both now match ngspice exactly.

## Jacobian (2x2 Block)

The GP Jacobian requires the quotient rule through qb:

### Derivatives of qb

```
dq1/dVbe = q1^2 / VAR
dq1/dVbc = q1^2 / VAF

dq2/dVbe = IS / (NF*VT * IKF) * exp(Vbe/(NF*Vt))     [via cbe/IKF]
dq2/dVbc = IS / (NR*VT * IKR) * exp(Vbc/(NR*Vt))     [via cbc/IKR]

D = sqrt(1 + 4*q2)
dD/dVbe = 2 * dq2/dVbe / D       (if D > 1e-15, else 0)
dD/dVbc = 2 * dq2/dVbc / D

dqb/dVbe = dq1/dVbe * (1 + D) / 2 + q1 * dD/dVbe / 2
dqb/dVbc = dq1/dVbc * (1 + D) / 2 + q1 * dD/dVbc / 2
```

### Quotient Rule for Icc/qb

```
dIcc/dVbe = IS/(NF*Vt) * exp(Vbe/(NF*Vt))
dIcc/dVbc = -IS/(NR*Vt) * exp(Vbc/(NR*Vt))

qb2_safe = max(qb^2, 1e-30)

d(Icc/qb)/dVbe = (dIcc/dVbe * qb - Icc * dqb/dVbe) / qb2_safe
d(Icc/qb)/dVbc = (dIcc/dVbc * qb - Icc * dqb/dVbc) / qb2_safe
```

### Final Jacobian

```
dIc/dVbe = d(Icc/qb)/dVbe                              [no beta_R term here]
dIc/dVbc = d(Icc/qb)/dVbc - IS/(beta_R * NR*Vt) * exp(Vbc/(NR*Vt))

dIb/dVbe = IS/(beta_F * NF*Vt) * exp(Vbe/(NF*Vt))      [Ebers-Moll; NOT divided by qb]
dIb/dVbc = IS/(beta_R * NR*Vt) * exp(Vbc/(NR*Vt))      [Ebers-Moll; NOT divided by qb]

bjt_jacobian returns [dIc/dVbe, dIc/dVbc, dIb/dVbe, dIb/dVbc]
```

The dIb/dV entries are identical to Ebers-Moll because `Ib = ib_fwd + ib_rev +
leakage` is not qb-modulated (ngspice bjtload.c:627 — `gpi = gbe/betaF + gben`,
`gmu = gbc/betaR + gbcn`, neither divided by qb). GP's base-charge correction
is exclusively applied to the transport (collector) current.

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
