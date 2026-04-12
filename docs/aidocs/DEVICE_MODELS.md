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

### Beam Tetrode Variant — Reefman "DerkE" §4.5

Beam tetrodes (6L6GC, 6V6GT, KT88) exhibit sharper screen-current knees than
true pentodes (the "critical compensation" phenomenon — see Reefman §4.5).
The rational `1/(1+β·Vp)` factor of Derk §4.4 cannot capture the knee
accurately, so melange uses a **second screen-current form**:

```
// "Exponential" screen form (DerkE §4.5)
u          = max(0, β·Vpk)                    // guard against Vpk<0 probes
ex_factor  = exp(-u^{3/2})                    // = exp(-(β·Vpk)^{3/2})
F(Vpk)     = 1/Kg1 − 1/Kg2 + (A·Vpk)/Kg1 − ex_factor · (α/Kg1 + αs/Kg2)
H(Vpk)     = (1 + αs · ex_factor) / Kg2
```

Everything else (Ip0, E1, α derivation, Leach grid current, 3×3 Jacobian
shape) is identical to the Rational form.

**Compute `u^{3/2}` as `u * sqrt(u)`**, not `u.powf(1.5)` — the former is
faster and cleanly returns 0 at u=0, the latter can produce NaN for any
negative input that slips through the `max(0, …)` guard.

Jacobian Vp-derivatives differ from the Rational form:
```
d(ex_factor)/dVpk = -1.5 · β · sqrt(u) · ex_factor
dF/dVpk           = A/Kg1 + 1.5 · β · sqrt(u) · ex_factor · (α/Kg1 + αs/Kg2)
dH/dVpk           = -1.5 · β · sqrt(u) · ex_factor · αs / Kg2
```

At `Vpk = 0`: `sqrt(u) = 0`, so both derivatives are finite and the chain
rule has no singularity.

### Screen-Form Discriminator

`TubeParams.screen_form: ScreenForm` selects the variant:
- `ScreenForm::Rational` (default) — Reefman Derk §4.4. True pentodes.
- `ScreenForm::Exponential` — Reefman DerkE §4.5. Beam tetrodes.

The `.model NAME VP(...)` directive accepts `SCREEN_FORM=0` (Rational) or
`SCREEN_FORM=1` (Exponential) as an explicit override. The PENTODE_CATALOG
entries carry the right form per-tube and the resolver auto-selects based
on the catalog.

Melange uses **two separate `ScreenForm` enum declarations** — one in
`melange-devices::tube` (drives `KorenPentode` runtime math) and one in
`melange-solver::device_types` (drives serialized `TubeParams` and codegen).
They're intentionally duplicated because `melange-devices` is upstream of
`melange-solver` and cannot depend on it. Conversion happens at the `dc_op.rs`
`KorenPentode` construction site with an inline match.

### Beam Tetrode Catalog Parameters (from Reefman TubeLib.inc 2016-01-23)

| Tube | μ | Ex | Kg1 | Kg2 | Kp | Kvb | αs | A | β | Source |
|------|---|-----|-----|-----|-----|-----|-----|---|---|--------|
| 6L6GC / 5881 / 6L6 | 9.41 | 1.306 | 446.6 | 6672.5 | 45.2 | 3205.1 | 8.10 | 4.91e-4 | 0.069 | BTetrodeDE |
| 6V6GT / 6V6 | 10.56 | 1.306 | 609.8 | 17267.3 | 47.9 | 2171.5 | 18.72 | 3.48e-4 | 0.068 | BTetrodeDE |

Catalog aliases: `6L6-T`, `6L6GC-T`, `5881-T`, `6V6-T`, `6V6GT-T`. The `-T`
suffix ("tetrode") distinguishes these from the legacy triode-connected
`6L6` / `6V6` catalog entries which are preserved byte-identical for
backward compat.

**KT88/6550** is not in Reefman's TubeLib.inc and has no published Derk or
DerkE fit. Deferred to phase 1a.2 (Cohen-Hélie classical Koren bootstrap)
or phase 1d (datasheet refit from RCA/Genalex manuals).

### Classical Variant — Norman Koren 1996 / Cohen-Hélie 2010 §3.2

Shipped in phase 1a.2 as a **third screen form** for pentodes without
publicly-available Reefman Derk fits (KT88, 6550). Fundamentally different
equation family from Derk §4.4 / DerkE §4.5 — NOT a parameter reduction:

```
inner  = Kp · (1/μ + Vgk / Vg2k)          // Vgk/Vg2k directly (NOT sqrt(Kvb+Vg2k²))
E1     = (Vg2k / Kp) · softplus(inner)
Ip     = (2 · E1^Ex / Kg1) · arctan(Vpk / Kvb)    // Kvb is the arctan knee scale
Ig2    = (Vg2k/μ + Vgk)^Ex / Kg2                  // Vp-INDEPENDENT
```

(Cohen-Hélie 2010 Eq 3 in the published paper is missing the `/Kg2` divisor;
the corrected form — matching Norman Koren's 1996 original — is what melange
implements. See `memory/pentode_equations.md` for the verification.)

Uses only 6 parameters: **μ, Ex, Kg1, Kg2, Kp, Kvb**. The `alpha_s/a_factor/
beta_factor` fields on `TubeParams` are ignored when `screen_form == Classical`.

**Key structural differences** from Derk/DerkE:

| | Derk §4.4 (phase 1a) | DerkE §4.5 (phase 1a.1) | Classical (phase 1a.2) |
|---|---|---|---|
| E1 softplus denominator | `sqrt(Kvb + Vg2²)` | `sqrt(Kvb + Vg2²)` | `Vg2` (direct) |
| Plate-voltage knee | F(Vp) rational | F(Vp) exponential | `arctan(Vpk/Kvb)` |
| Kvb's role | softplus denom | softplus denom | **arctan knee scale** |
| Ig2 Vp-dependence | rational `1/(1+β·Vp)` | exponential `exp(-(β·Vp)^{3/2})` | **none** (Vp-independent) |
| Parameter count | 9 (μ,Ex,Kg1,Kg2,Kp,Kvb,αs,A,β) | 9 | 6 (μ,Ex,Kg1,Kg2,Kp,Kvb) |

### Classical validation rules (phase 1a.2)

- `alpha_s` must NOT be required when Classical — the Derk-specific αs>0
  invariant is relaxed. `a_factor` and `beta_factor` are also unused.
- `(ScreenForm::Classical, svar > 0)` is **rejected** at both the resolver
  and validator level. Reefman §5 two-section Koren is built on the Derk
  softplus structure; translating it to the Classical arctan-knee form is
  not implemented (and no known tube needs this combination).
- `.model ... VP(SCREEN_FORM=2 ...)` is the `.model`-directive escape hatch
  for hand-rolled Classical pentodes. Catalog entries (KT88, 6550) set
  `screen_form: Classical` automatically via `lookup_pentode`.

### Classical Catalog Parameters (from Cohen-Hélie 2010 DAFx Table 2)

| Tube | μ | Ex | Kg1 | Kg2 | Kp | Kvb | Source |
|------|---|-----|-----|-----|-----|-----|--------|
| KT88 | 8.8 | 1.35 | 730 | 4200 | 32 | 16 | Cohen-Hélie / Norman Koren 1996 |
| 6550 | 8.8 | 1.35 | 730 | 4200 | 32 | 16 | Cohen-Hélie / Norman Koren 1996 |

KT88 and 6550 share the fit per Cohen-Hélie Table 2 — electrically the same
tube (Genalex KT88 and GE 6550 are functionally equivalent beam tetrodes).
Catalog aliases: `KT88`/`KT-88`, `6550`/`6550A`/`6550C`.

**Known calibration offset**: Cohen-Hélie's Kg1 gives roughly 2× the
datasheet idle plate current (same issue that shows up in the 12AX7
Kg1=1060 vs 12AX7F Kg1=3000 fits documented in `catalog/tubes.rs`). KT88 Ip
at the canonical Class AB bias (Vgk=-20, Vpk=350, Vg2k=300) computes to
~176 mA, vs ~60-80 mA on the real tube. Accept this as a calibration offset
— phase 1d can refit from datasheet curves if anyone actually ships a KT88
amp.

### Known limitation of Classical

**Screen current is Vp-independent**. Under hard plate clipping the real
tube's screen current rises as Vp falls — that's part of why Reefman's
Derk §4.4 `αs/(1+β·Vp)` term exists. Classical Koren doesn't model this.
Under moderate clipping the audible difference is small; under hard clip
(plate swung well below knee) the screen draws less current in the
simulation than it would in hardware, so the simulated amp stays slightly
cleaner on peaks than the real thing. Not catastrophic. Upgrade path is
phase 1d: measure KT88 screen family on a uTracer and refit to Derk or
DerkE per `ExtractModel` methodology.

### Variable-Mu Variant — Reefman §5 Two-Section Koren

Remote-cutoff ("variable-mu") pentodes (6K7, 6BA6, 6SK7, EF89) and twin
triodes (6386, 6BC8) have non-uniform grid-wire pitch that produces a
gm(Vgk) rolloff which is **gradual** as bias goes negative instead of the
sharp cutoff of ordinary tubes. This is the essential behavior exploited
by variable-mu compressors (Fairchild 670, Gates Sta-Level, Collins 26U).

Reefman §5 models the variable-mu behavior as **two Koren currents
blended with a fixed weight**:

```
I_P,Koren_v = (1 − s_var) · I_P,Koren_a + s_var · I_P,Koren_b      (Eq 33)
```

Each section has its own `(μ, x)` pair: section A uses `(mu, ex)` (the
high-mu primary), section B uses `(mu_b, ex_b)` (the low-mu secondary
that dominates at deep cutoff). **The other Koren params — Kp, Kvb, Kg1,
Kg2, αs, A, β — are shared between sections**. Reefman: "these factors
are largely determined by geometry, which very likely will be rather
identical for both parallel pentodes".

The blended Koren current `Ip0_v` plugs into the existing §4.4 Derk
`F(Vp)`/`H(Vp)` (or §4.5 DerkE) scaling factors unchanged, so variable-mu
is **orthogonal to screen form**:

- `(svar=0, Rational)` — sharp true pentode (EL84, EL34, EF86)
- `(svar=0, Exponential)` — sharp beam tetrode (6L6GC, 6V6GT)
- `(svar>0, Rational)` — variable-mu pentode (6K7, 6BA6 — Reefman PenthodeVD)
- `(svar>0, Exponential)` — variable-mu beam/pentode (EF89 — Reefman PenthodeVDE)

### Variable-Mu Discriminator

`TubeParams` has three phase-1c fields:
- `mu_b: f64` — section-B amplification factor (μ_b)
- `svar: f64` — blend fraction ∈ [0, 1]; 0.0 = sharp default
- `ex_b: f64` — section-B Koren exponent (x_b)

When `svar == 0.0`, the device is sharp-cutoff and `mu_b`/`ex_b` are
unused. The math `compute_ip0_v` branch falls through to the existing
single-section path with byte-identical output. When `svar > 0.0`, both
sections are computed and blended.

`TubeParams::is_variable_mu()` returns `svar > 0.0`. Validation requires
`svar ∈ [0, 1]`, and when `svar > 0`: `mu_b > 0`, `ex_b > 0`.

The `.model NAME VP(...)` directive accepts `MU_B`, `SVAR`, `EX_B` as
explicit keywords. The `PENTODE_CATALOG` entries for 6K7 and EF89 carry
the right values per-tube and the resolver auto-picks them.

### Variable-Mu Catalog Parameters (from Reefman TubeLib.inc 2016)

| Tube | μ_a | ex_a | Kg1 | Kg2 | Kp | Kvb | αs | A | β | μ_b | svar | ex_b | ScreenForm |
|------|-----|------|-----|-----|-----|-----|-----|---|---|-----|------|------|------------|
| 6K7 / 6K7G | 15.5 | 1.573 | 1407.7 | 8335.8 | 36.0 | 1309.0 | 4.07 | 1.55e-9 | 0.15 | 3.4 | 0.083 | 1.223 | Rational (PenthodeVD) |
| EF89 / 6DA6 | 25.0 | 1.418 | 328.3 | 1199.3 | 58.8 | 1.0 | 2.07 | 0.0 | 0.122 | 7.8 | 0.068 | 0.978 | Exponential (PenthodeVDE) |

Ratio `mu_a / mu_b ≈ 4.5` for both (confirming Reefman's observation
that this ratio is "rather typical for the [limited] set of variable-mu
pentodes fitted so far"). The `svar ≈ 0.07–0.08` is consistent with
section B carrying ~7–8% of the total conduction current.

### What's Needed for 6BA6, 6386, 6BC8, 6SK7

These canonical varimu compressor tubes have **no public Reefman fit**.
Phase 1c ships the math path; validating against a real Fairchild 670
or Sta-Level schematic requires either:
- A datasheet refit pass (published gm(Vgk) curves → least-squares fit
  for (mu_a, mu_b, ex_a, ex_b, svar, Kg1, Kg2, Kp, Kvb, αs, A, β))
- Adoption of Adrian Immler's i5/i6 models — different equation topology
  from Reefman, would need a parallel codepath
- An author contribution (Reefman accepts pull-model fits via
  ExtractModel — see his uTracer website)

### Grid-Off Reduction (Phase 1b)

Load-bearing for Plexi-class amps: without reduction, 4×EL34 + 3×12AX7 =
18 nonlinear dimensions, which exceeds the M=16 DK Schur cap and forces
the circuit onto the slower nodal full-LU path. Grid-off reduction
brings this to 14 dimensions, back under the cap and onto DK Schur
(Gaussian elimination branch).

When DC-OP confirms a pentode is biased in grid-cutoff (`Vgk < -0.5 −
vgk_onset`), two things are true: (1) Ig1 is identically zero (Leach
power-law below onset), and (2) Vg2k is approximately constant in any
amp with a well-bypassed screen (47 µF + 1 kΩ screen-stop is universal).
Melange drops the Ig1 dimension entirely AND freezes Vg2k at the
DC-OP-converged value, reducing the pentode from 3D to 2D (Vgk/Vpk →
Ip/Ig2). The frozen Vg2k is stored as a per-slot constant on
`DeviceSlot.vg2k_frozen`.

This is the BJT-FA analog for pentodes with one important difference:
BJT forward-active reduction is an **exact** physics simplification (Vbc
really doesn't drive Ic in FA mode), while pentode grid-off is an
**approximation** — Vg2k really does drive Ip/Ig2 but varies by <1%
under signal with a properly bypassed screen.

**Triode grid-off is structurally impossible**: Koren `Ip(Vgk, Vpk)`
needs both voltages, and dropping Vpk would destroy the plate-swing
signal. Triodes stay 2D regardless of bias. Only pentodes reduce.

### Auto-detection and CLI flag

Auto-detection runs after the first DC-OP converges. For every pentode
slot, compute Vgk from node voltages; if below cutoff + margin, mark for
grid-off. Then rebuild MNA via `from_netlist_with_grid_off` and re-run
DC-OP on the reduced system (converges trivially since the dropped
dimension was already at its correct value). Warm DC-OP re-init on
pot/switch changes (commit `e8e18a7`) triggers re-detection.

CLI flag `--tube-grid-fa` matches the existing BJT FA pattern:
- `auto` (default) — detect at DC-OP, reduce when applicable
- `on` — force grid-off for every pentode regardless of bias (testing)
- `off` — never reduce, full 3D path (pre-phase-1b parity)

### Known limitation

Under hard plate clipping the real tube's screen current rises as Vp
falls and Vg2k sags slightly; the frozen model misses that motion.
Audible error is 0.5–2 dB in the affected harmonics, similar in
character to Classical Koren's Vp-independent screen limitation. For
users who care about this fidelity, `--tube-grid-fa off` forces the full
3D path with 5-10× slowdown on Plexi-class circuits.

### Compatibility matrix

Grid-off is orthogonal to screen-form:

| Screen form | Variable-mu | Grid-off supported? |
|---|---|---|
| Rational (Derk §4.4) | No | ✅ (EL84, EL34, EF86) |
| Exponential (DerkE §4.5) | No | ✅ (6L6GC, 6V6GT) |
| Classical (Koren 1996) | No | ✅ (KT88, 6550) |
| Rational | Yes (§5 6K7/EF89) | ⛔ rejected at validation |
| Exponential | Yes (§5 EF89) | ⛔ rejected at validation |
| Classical | Yes (§5) | ⛔ already rejected in phase 1a.2 |

Variable-mu + grid-off is rejected because variable-mu tubes exist
specifically for AGC where the bias changes continuously under sidechain
control — freezing the screen at a single DC-OP value contradicts that
usage pattern.

### Still Deferred

- **6386 / 6BA6 / 6BC8 datasheet refits** for Fairchild 670 / Sta-Level
  / Altec 436 varimu compressor validation. The math path (phase 1c
  variable-mu §5) is ready; just needs the fit work. User explicitly
  flagged as "big project, not today".

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
