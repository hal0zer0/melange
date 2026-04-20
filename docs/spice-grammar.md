# SPICE Netlist Grammar (melange-solver)

This document defines the subset of SPICE3/ngspice syntax supported by melange-solver. It serves as the **parser contract**—any valid netlist following these rules will be parsed correctly; anything outside this subset will raise an error.

---

## 1. Supported Component Statements

### R — Resistor

**Syntax:**
```
Rname n+ n- value
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive terminal node |
| `n-` | Negative terminal node |
| `value` | Resistance in ohms (Ω) |

**Examples:**
```spice
R1 in out 10k
Rload vcc coll 4.7k
Rbias base 0 100k
```

---

### C — Capacitor

**Syntax:**
```
Cname n+ n- value [IC=initial_voltage]
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive terminal node |
| `n-` | Negative terminal node |
| `value` | Capacitance in farads (F) |
| `IC` | (Optional) Initial voltage at t=0 |

**Examples:**
```spice
C1 in base 10u
Cout coll out 47u IC=0
Cc emit 0 100p
```

---

### L — Inductor

**Syntax:**
```
Lname n+ n- value [IC=initial_current] [ISAT=saturation_current]
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive terminal node |
| `n-` | Negative terminal node |
| `value` | Inductance in henries (H) |
| `IC` | (Optional) Initial current at t=0 |
| `ISAT` | (Optional) Core saturation current in amps. When present, inductance follows a tanh model: `L(I) = L0 * (1 - tanh(I/ISAT)^2)`. Must be positive. |

**Examples:**
```spice
L1 in out 10m
Lchoke vcc coll 100u
Lcore a b 100m ISAT=20m          ; saturating inductor
```

**Notes on saturating inductors:**
- Uses Sherman-Morrison rank-1 per-sample update with drift resync every 16 updates
- Currently uncoupled only (cannot combine with `K` coupling statements)
- L(I) computation is lagged by one sample (uses previous-sample current)
- No ngspice validation available for saturation behavior

---

### V — Voltage Source

**Syntax:**
```
Vname n+ n- DC value [AC mag [phase]]
Vname n+ n- PULSE(v1 v2 td tr tf pw per)
Vname n+ n- SINE(vo va freq [td [theta [phase]]])
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive terminal node |
| `n-` | Negative terminal node |
| `DC` | DC voltage value |
| `AC` | AC magnitude (for `.ac` analysis) |
| `phase` | AC phase in degrees (default: 0) |

**Examples:**
```spice
Vcc vcc 0 9V
Vin in 0 DC 0 AC 1V
Vsig in 0 SINE(0 1 1k)
```

---

### I — Current Source

**Syntax:**
```
Iname n+ n- DC value
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive terminal node (current flows out) |
| `n-` | Negative terminal node (current flows in) |
| `value` | DC current in amperes (A) |

**Examples:**
```spice
Ibias base 0 1m
Iin 0 in DC 10u
```

---

### D — Diode

**Syntax:**
```
Dname n+ n- modelname [area]
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Anode (positive terminal) |
| `n-` | Cathode (negative terminal) |
| `modelname` | Name of `.model` definition |
| `area` | (Optional) Area scaling factor (default: 1) |

**Examples:**
```spice
D1 in out 1N4148
Dclip pos 0 D1N4004 2

.model 1N4148 D(IS=2.52e-9 N=1.752)
.model D1N4004 D(IS=14.11e-9 N=1.984)
```

---

### Q — BJT (Bipolar Junction Transistor)

**Syntax:**
```
Qname nc nb ne [ns] modelname [area]
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `nc` | Collector terminal node |
| `nb` | Base terminal node |
| `ne` | Emitter terminal node |
| `ns` | (Optional) Substrate terminal node |
| `modelname` | Name of `.model` definition (NPN or PNP) |
| `area` | (Optional) Area scaling factor (default: 1) |

**Examples:**
```spice
Q1 coll base emit 2N2222
Q2 out in gnd sub 2N3906 2

.model 2N2222 NPN(IS=1e-15 BF=200 VAF=100)
.model 2N3906 PNP(IS=1.41e-15 BF=180)
```

---

### J — JFET (Junction Field-Effect Transistor)

**Syntax:**
```
Jname nd ng ns modelname [area]
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `nd` | Drain terminal node |
| `ng` | Gate terminal node |
| `ns` | Source terminal node |
| `modelname` | Name of `.model` definition (NJF or PJF) |
| `area` | (Optional) Area scaling factor (default: 1) |

**Examples:**
```spice
J1 drain gate source J2N5457

.model J2N5457 NJF(VTO=-1.5 BETA=1e-4 LAMBDA=0.01)
.model J175 PJF(VTO=3.0 BETA=1.2e-4)
```

---

### M — MOSFET

**Syntax:**
```
Mname nd ng ns nb modelname [L=value] [W=value] [AD=value] [AS=value]
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `nd` | Drain terminal node |
| `ng` | Gate terminal node |
| `ns` | Source terminal node |
| `nb` | Bulk/substrate terminal node |
| `modelname` | Name of `.model` definition (NMOS or PMOS) |
| `L` | Channel length (meters) |
| `W` | Channel width (meters) |
| `AD` | Drain diffusion area |
| `AS` | Source diffusion area |

**Examples:**
```spice
M1 out in 0 0 NMOS1 L=2u W=20u
M2 vdd gate out vdd PMOS1 L=1u W=40u

.model NMOS1 NMOS(VTO=0.7 KP=50e-6 LAMBDA=0.02)
.model PMOS1 PMOS(VTO=-0.8 KP=25e-6 LAMBDA=0.02)
```

---

### E — VCVS (Voltage-Controlled Voltage Source)

**Syntax:**
```
Ename n+ n- nc+ nc- gain
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive output terminal |
| `n-` | Negative output terminal |
| `nc+` | Positive controlling terminal |
| `nc-` | Negative controlling terminal |
| `gain` | Voltage gain (V/V) |

**Examples:**
```spice
Eamp out 0 in 0 100       ; Ideal op-amp model
Ebuffer out 0 in+ in- 1.0 ; Unity-gain buffer
```

---

### F — CCCS (Current-Controlled Current Source)

**Syntax:**
```
Fname n+ n- vname gain
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive output terminal |
| `n-` | Negative output terminal |
| `vname` | Name of voltage source through which control current flows |
| `gain` | Current gain (A/A) |

**Examples:**
```spice
Vsense 1 2 0              ; Zero-volt source to sense current
F1 3 0 Vsense 0.99        ; Current source mirroring Isense
```

---

### G — VCCS (Voltage-Controlled Current Source)

**Syntax:**
```
Gname n+ n- nc+ nc- transconductance
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive output terminal (current flows out) |
| `n-` | Negative output terminal (current flows in) |
| `nc+` | Positive controlling terminal |
| `nc-` | Negative controlling terminal |
| `transconductance` | Transconductance in siemens (A/V) |

**Examples:**
```spice
G1 out 0 in 0 0.001       ; 1 mS transconductance
Gm 2 3 pos neg 1e-3
```

---

### H — CCVS (Current-Controlled Voltage Source)

**Syntax:**
```
Hname n+ n- vname transresistance
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive output terminal |
| `n-` | Negative output terminal |
| `vname` | Name of voltage source through which control current flows |
| `transresistance` | Transresistance in ohms (V/A) |

**Examples:**
```spice
Vmonitor 1 0 0            ; Zero-volt source to sense current
H1 2 0 Vmonitor 1000      ; 1 kΩ transresistance
```

---

### K — Coupled Inductors / Transformer

**Syntax:**
```
Kname L1name L2name coupling_coefficient
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `L1name` | Name of first inductor |
| `L2name` | Name of second inductor |
| `coupling_coefficient` | Coupling factor k (0 < k < 1) |

Mutual inductance: M = k * sqrt(L1 * L2). Turns ratio: N1:N2 = sqrt(L1/L2).

**Examples:**
```spice
* Simple 1:2 step-up transformer
L1 primary_p primary_n 10m
L2 secondary_p secondary_n 40m
K1 L1 L2 0.99

* Multi-winding transformer (3 windings)
L1 pri_p pri_n 37H         ; primary
L2 sec_p sec_n 148H        ; secondary
L3 tert_p tert_n 2H        ; tertiary (feedback)
K1 L1 L2 0.9999
K2 L1 L3 0.9999
K3 L2 L3 0.9999
```

**Notes:**
- Both inductors must be defined before the K statement
- Cannot couple an inductor to itself
- Multiple K statements can form multi-winding transformer groups
- Core saturation and hysteresis are not modeled

---

### T — Triode (Vacuum Tube)

**Syntax:**
```
Tname n_grid n_plate n_cathode modelname
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n_grid` | Grid terminal node |
| `n_plate` | Plate (anode) terminal node |
| `n_cathode` | Cathode terminal node |
| `modelname` | Name of `.model` definition (type: TRIODE) |

**Examples:**
```spice
T1 grid1 plate1 cathode1 12AX7
T2 grid2 plate2 cathode2 12AU7

.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300)
.model 12AU7 TRIODE(MU=17 EX=1.3 KG1=1180 KP=84 KVB=300)
```

**Notes:**
- Uses the Koren plate current model with Leach grid current
- Terminal order is **grid-plate-cathode** (unusual for SPICE but kept for
  backward compatibility with existing circuits). Pentodes use a different
  (plate-first) order — see `P` element below.

---

### P — Pentode / Beam Tetrode (Vacuum Tube)

**Syntax:**
```
Pname n_plate n_grid n_cathode n_screen [n_suppressor] modelname
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n_plate` | Plate (anode) terminal node |
| `n_grid` | Control grid (g1) terminal node |
| `n_cathode` | Cathode terminal node |
| `n_screen` | Screen grid (g2) terminal node |
| `n_suppressor` | *Optional* suppressor grid (g3) node. When omitted, the suppressor is electrically strapped to the cathode (the universal case for audio beam tetrodes and strapped pentodes). Phase 1a models a provided suppressor as cathode-tied as well. |
| `modelname` | Name of `.model` definition (type: VP) |

**Examples:**
```spice
* EL84 with suppressor strapped to cathode (4-terminal form)
P1 plate1 grid1 cath1 scr1 EL84-P

* EF86 with explicit suppressor (5-terminal form, g3 phase-1a: tied to cath)
P2 plate2 grid2 cath2 scr2 sup2 EF86

.model EL84-P VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 KP=152.4 KVB=4015.8
+                ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148)
.model EF86    VP(MU=40.8  EX=1.327 KG1=675.8 KG2=4089.6 KP=350.7 KVB=1886.8
+                ALPHA_S=4.24 A_FACTOR=5.95e-5 BETA_FACTOR=0.28)
```

**Notes:**
- Uses the Reefman "Derk" §4.4 pentode equations:
  `Ip0 = E1^Ex/2·(1+sgn(E1))` built from `E1 = (Vg2k/Kp)·softplus(Kp·(1/μ + Vgk/sqrt(Kvb+Vg2k²)))`,
  then `Ip = Ip0·F(Vpk)` and `Ig2 = Ip0·H(Vpk)` with the Vp-dependent
  F/H factors parametrized by `αs, A, β`. Grid current uses the same
  Leach power-law as triodes.
- Terminal order is **plate-grid-cathode-screen**, which matches the
  LTspice / PSpice / Ayumi convention and deliberately differs from the
  `T` element's grid-plate-cathode order. Use the order that matches
  your tube: `T` for existing triode netlists, `P` for new pentode work.
- 9 fitted Derk parameters per tube: μ, Ex, Kg1, Kg2, Kp, Kvb, αs, A, β
  (plus optional `IG_MAX`, `VGK_ONSET`, `CCG`, `CGP`, `CCP`, `RGI`
  which behave identically to the triode form).
- **αs must be strictly positive** — Derk with αs=0 degenerates to
  Ip=0 identically. The validator rejects `.model … VP(ALPHA_S=0)`.
- Catalog aliases (use these instead of inline `.model VP(...)` for
  convenience):
  - **True pentodes** (Reefman Derk §4.4, `Rational` screen form):
    `EL84-P` / `EL84P` / `6BQ5-P`, `EL34-P` / `EL34P` / `6CA7-P`,
    `EF86` / `6267`.
  - **Beam tetrodes** (Reefman DerkE §4.5, `Exponential` screen form):
    `6L6-T` / `6L6GC-T` / `6L6GCT` / `5881-T`, `6V6-T` / `6V6GT-T` /
    `6V6GTT`.

  The `-P` and `-T` suffixes distinguish these from the legacy
  triode-connected `EL84` / `EL34` / `6L6` / `6V6` catalog entries which
  remain available for backward compatibility. The catalog sets the
  right `SCREEN_FORM` automatically — a netlist that references
  `6V6GT-T` gets the DerkE math without any further directive.
  - **Classical Koren (phase 1a.2)**: `KT88` / `KT-88`, `6550` / `6550A` /
    `6550C`. No suffix because there's no pre-existing triode entry to
    collide with. Uses the Norman Koren 1996 / Cohen-Hélie 2010 equation
    family (`arctan(Vpk/Kvb)` plate knee, Vp-independent screen). Only
    μ/Ex/Kg1/Kg2/Kp/Kvb are used — αs/A/β are ignored by the Classical
    helpers. Fallback for tubes without published Reefman Derk fits.
- **`SCREEN_FORM` parameter**: `.model NAME VP(... SCREEN_FORM=0)` for
  Rational (Reefman Derk §4.4), `SCREEN_FORM=1` for Exponential (Reefman
  DerkE §4.5), `SCREEN_FORM=2` for Classical (Norman Koren 1996 /
  Cohen-Hélie 2010, used by KT88 and 6550). Only needed when hand-rolling
  a `.model` without a catalog alias; the default is `0` (Rational).
  Catalog entries carry the correct form automatically, so a bare
  `.model KT88 VP()` gets Classical behavior via the catalog lookup.
- **Grid-off reduction (phase 1b)**: the solver auto-detects when a
  pentode is biased in grid-cutoff (`Vgk < −(vgk_onset + 0.5)`) and
  reduces its NR dimension from 3 to 2 — drops the `Ig1` dimension
  entirely and freezes `Vg2k` at the DC-OP-converged value. Controlled
  by `--tube-grid-fa {auto,on,off}` on `melange simulate` and
  `melange compile`:
  - `auto` (default): detect and reduce where Vgk is in cutoff
  - `on`: force grid-off on every pentode regardless of bias (testing)
  - `off`: never reduce, always full 3D NR block (pre-phase-1b parity)

  Grid-off is a physics approximation — the screen voltage is held
  constant, so under hard plate clipping the real tube's screen-current
  rise isn't tracked. Audible error is 0.5–2 dB on affected harmonics,
  similar in character to Classical Koren's Vp-independent screen.
  For users who care, `--tube-grid-fa off` restores full fidelity at
  5–10× slowdown on Plexi-class amps. Triode grid-off is structurally
  impossible (both Vgk and Vpk drive Ip) — only pentodes reduce.

- **Not yet supported**:
  - 6386 / 6BC8 / 6BA6 datasheet-refit entries for varimu compressor
    targets (Fairchild 670, Sta-Level, Altec 436). Math path is ready
    (phase 1c variable-mu §5), just waiting on the datasheet fit work.
  - Independent suppressor dynamics on true 5-element pentodes —
    suppressor is always electrically tied to cathode.

---

### U — Op-Amp (Operational Amplifier)

**Syntax:**
```
Uname n_plus n_minus n_out modelname
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n_plus` | Non-inverting input (+) |
| `n_minus` | Inverting input (-) |
| `n_out` | Output node |
| `modelname` | Name of `.model` definition (type: OA) |

**Examples:**
```spice
* Inverting amplifier
U1 in_pos in_neg out LM358
Rin input in_neg 10k
Rf in_neg out 100k
R_bias in_pos 0 10k

.model LM358 OA(AOL=200000 ROUT=75)
```

**Notes:**
- Linear device (does not add nonlinear dimensions to the solver)
- Modeled as Boyle VCCS macromodel with output resistance
- Input impedance is infinite (no input bias current)

---

### Y — VCA (Voltage-Controlled Amplifier)

**Syntax:**
```
Yname sig_p sig_n ctrl_p ctrl_n modelname
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `sig_p` | Signal positive terminal |
| `sig_n` | Signal negative terminal |
| `ctrl_p` | Control voltage positive terminal |
| `ctrl_n` | Control voltage negative terminal |
| `modelname` | Name of `.model` definition (type: VCA) |

**Examples:**
```spice
Y1 sig_in sig_out ctrl_in ctrl_ref VCA_2180
Rload sig_out 0 10k

.model VCA_2180 VCA(VSCALE=0.05298 G0=1.0 THD=0.001)
```

**Notes:**
- 2D nonlinear device: signal path + control path
- Control input is high-impedance (draws zero current)
- Gain law: I = G0 * exp(-Vctrl / VSCALE) * Vsig
- Models THAT 2180 / DBX 2150 style current-mode VCAs

---

### X — Subcircuit Instance

**Syntax:**
```
Xname node1 node2 ... nodeN subcircuit_name
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `node1..nodeN` | External nodes (in order matching `.subckt` definition) |
| `subcircuit_name` | Name of `.subckt` block to instantiate |

**Examples:**
```spice
.subckt PREAMP in out vcc
R1 in 1 10k
R2 1 out 100k
Rload out vcc 4.7k
.ends PREAMP

X1 input mid vcc PREAMP
X2 mid output vcc PREAMP
```

**Notes:**
- Node count must match the `.subckt` definition
- Subcircuits can be nested (max depth: 8 levels)
- Maximum 10,000 elements after expansion

---

## 2. Model Cards (.model)

**Syntax:**
```
.model name type (param1=value1 param2=value2 ...)
```

The `.model` statement defines the parameters for semiconductor devices. Parameters not specified use SPICE defaults.

### Supported Model Types

| Type | Device | Description |
|------|--------|-------------|
| `D` | Diode | Junction diode |
| `NPN` | BJT | NPN bipolar transistor |
| `PNP` | BJT | PNP bipolar transistor |
| `NJF` | JFET | N-channel JFET |
| `PJF` | JFET | P-channel JFET |
| `NMOS` | MOSFET | N-channel MOSFET |
| `PMOS` | MOSFET | P-channel MOSFET |
| `TRIODE` | Tube (triode) | Vacuum tube triode (Koren model) |
| `VP` | Tube (pentode) | Pentode / beam tetrode (Reefman Derk §4.4) |
| `OA` | Op-Amp | Operational amplifier (Boyle VCCS) |
| `VCA` | VCA | Voltage-controlled amplifier (THAT 2180) |

### Diode Parameters (Type: D)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `IS` | 1e-14 A | Saturation current |
| `N` | 1.0 | Emission coefficient (ideality factor) |
| `RS` | 0 Ω | Series resistance |
| `CJO` | 0 F | Zero-bias junction capacitance |
| `VJ` | 1.0 V | Junction potential |
| `M` | 0.5 | Grading coefficient |
| `TT` | 0 s | Transit time |
| `BV` | ∞ V | Reverse breakdown voltage |
| `IBV` | 1e-3 A | Current at breakdown |

**Example:**
```spice
.model 1N4148 D(IS=2.52e-9 N=1.752 RS=0.568 CJO=4e-12)
```

### BJT Parameters (Types: NPN, PNP)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `IS` | 1e-16 A | Transport saturation current |
| `BF` | 100.0 | Ideal maximum forward beta |
| `BR` | 1.0 | Ideal maximum reverse beta |
| `NF` | 1.0 | Forward emission coefficient |
| `NR` | 1.0 | Reverse emission coefficient |
| `ISE` | 0 A | Base-emitter leakage saturation current |
| `NE` | 1.5 | Base-emitter leakage emission coefficient |
| `ISC` | 0 A | Base-collector leakage saturation current |
| `NC` | 2.0 | Base-collector leakage emission coefficient |
| `VAF` | ∞ V | Forward Early voltage (Gummel-Poon) |
| `VAR` | ∞ V | Reverse Early voltage (Gummel-Poon) |
| `IKF` | ∞ A | Forward beta high-current rolloff |
| `IKR` | ∞ A | Reverse beta high-current rolloff |
| `RB` | 0 Ω | Base series resistance |
| `RC` | 0 Ω | Collector series resistance |
| `RE` | 0 Ω | Emitter series resistance |
| `CJE` | 0 F | Base-emitter zero-bias capacitance |
| `CJC` | 0 F | Base-collector zero-bias capacitance |
| `VJE` | 0.75 V | Base-emitter built-in potential |
| `VJC` | 0.75 V | Base-collector built-in potential |
| `MJE` | 0.33 | Base-emitter grading coefficient |
| `MJC` | 0.33 | Base-collector grading coefficient |
| `TF` | 0 s | Ideal forward transit time |
| `TR` | 0 s | Ideal reverse transit time |

When `VAF`, `VAR`, `IKF`, or `IKR` are finite, the Gummel-Poon model is auto-selected (base charge modulation). Otherwise Ebers-Moll is used.

When `RB`, `RC`, or `RE` are non-zero, internal nodes (basePrime, collectorPrime, emitterPrime) are added for parasitic resistance modeling.

**Example:**
```spice
.model 2N2222 NPN(IS=1e-15 BF=200 VAF=100 CJE=20p CJC=10p RB=10 RC=1 RE=0.1)
.model 2N3906 PNP(IS=1.41e-15 BF=180 VAF=80)
```

### JFET Parameters (Types: NJF, PJF)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `VTO` | -2.0 V (NJF), +2.0 V (PJF) | Pinch-off voltage |
| `IDSS` | 2e-3 A | Saturation drain current |
| `BETA` | — | Transconductance parameter (converted: IDSS = BETA * VTO²) |
| `LAMBDA` | 0.001 V⁻¹ | Channel-length modulation |
| `RD` | 0 Ω | Drain ohmic resistance |
| `RS` | 0 Ω | Source ohmic resistance |
| `CGS` | 0 F | Gate-source capacitance |
| `CGD` | 0 F | Gate-drain capacitance |
| `PB` | 1.0 V | Gate junction potential |

Either `IDSS` or `BETA` may be specified. If both are present, `IDSS` takes priority. If only `BETA` is given, it is converted to IDSS = BETA * VTO².

**Example:**
```spice
.model J2N5457 NJF(VTO=-1.5 IDSS=2e-3 LAMBDA=0.01)
.model J175 PJF(VTO=3.0 IDSS=2.5e-3)
```

### MOSFET Parameters (Types: NMOS, PMOS)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `VTO` | 2.0 V (NMOS), -2.0 V (PMOS) | Threshold voltage |
| `KP` | 2e-5 A/V² | Transconductance parameter |
| `LAMBDA` | 0.01 V⁻¹ | Channel-length modulation |
| `GAMMA` | 0 V¹/² | Body effect parameter |
| `PHI` | 0.6 V | Surface potential |
| `RD` | 0 Ω | Drain ohmic resistance |
| `RS` | 0 Ω | Source ohmic resistance |
| `CGS` | 0 F | Gate-source capacitance |
| `CGD` | 0 F | Gate-drain capacitance |

Level 1 SPICE model with triode + saturation regions. When `GAMMA` > 0, body effect modulates threshold voltage with source-bulk voltage.

**Example:**
```spice
.model NMOS1 NMOS(VTO=0.7 KP=50e-6 LAMBDA=0.02 GAMMA=0.37)
.model PMOS1 PMOS(VTO=-0.8 KP=25e-6 LAMBDA=0.02)
```

### Triode Parameters (Type: TRIODE)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MU` | — (required) | Amplification factor (mu) |
| `EX` | — (required) | Koren equation exponent |
| `KG1` | — (required) | Koren Kg1 coefficient |
| `KP` | — (required) | Koren Kp coefficient |
| `KVB` | — (required) | Koren Kvb knee shaping coefficient |
| `LAMBDA` | 0.0 V⁻¹ | Plate resistance modulation (Early effect) |
| `IG_MAX` | 2e-3 A | Maximum grid current |
| `VGK_ONSET` | 0.5 V | Grid current onset voltage |
| `CCG` | 0 F | Cathode-grid capacitance |
| `CGP` | 0 F | Grid-plate capacitance |
| `CCP` | 0 F | Cathode-plate capacitance |
| `RGI` | 0 Ω | Grid internal resistance |

Uses the Koren plate current model (soft-knee saturation) with Leach power-law grid current. The five core parameters (MU, EX, KG1, KP, KVB) are required and tube-specific — look up values for your specific tube type (12AX7, 12AU7, etc.).

**Example:**
```spice
.model 12AX7 TRIODE(MU=100 EX=1.4 KG1=1060 KP=600 KVB=300 CCG=1.6p CGP=1.7p CCP=0.46p)
.model 12AU7 TRIODE(MU=17 EX=1.3 KG1=1180 KP=84 KVB=300)
```

### Op-Amp Parameters (Type: OA)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `AOL` | 200000 | Open-loop voltage gain (V/V) |
| `ROUT` | 1.0 Ω | Output resistance |
| `GBW` | ∞ Hz | Gain-bandwidth product (single-pole rolloff) |
| `VSAT` | ∞ V | Output saturation voltage (13.0V if GBW finite) |

Linear device — does not add nonlinear dimensions to the solver. Modeled as Boyle VCCS macromodel with output resistance. Input impedance is infinite.

**Example:**
```spice
.model LM358 OA(AOL=200000 ROUT=75)
.model TL072 OA(AOL=200000 ROUT=50 GBW=3e6 VSAT=13)
```

### VCA Parameters (Type: VCA)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `VSCALE` | 0.05298 V/neper | Control voltage scaling (THAT 2180A default) |
| `G0` | 1.0 S | Unity-gain conductance |
| `THD` | 0.0 | Gain-dependent cubic distortion coefficient |
| `MODE` | 0 | 0 = voltage mode, 1 = current mode (CCCS) |

Gain law: I_signal = G0 * exp(-V_control / VSCALE) * V_signal. THD adds cubic nonlinearity that rises with gain reduction. Control port is high-impedance (draws no current).

**Example:**
```spice
.model THAT2180 VCA(VSCALE=0.05298 G0=1.0 THD=0.001)
.model DBX2150 VCA(VSCALE=0.006 G0=0.1)
```

---

## 3. Subcircuits (.subckt / .ends)

**Syntax:**
```
.subckt name node1 node2 node3 ...
* circuit elements
.ends [name]
```

**Subcircuit Instantiation:**
```
Xname node1 node2 node3 ... subcktname
```

### Basic Subcircuit Example

```spice
* Define an op-amp gain stage
.subckt GAIN_STAGE in out vcc vee
R1 in base 100k
R2 base vee 22k
Rc vcc coll 4.7k
Re emit vee 1k
Q1 coll base emit 2N2222
Cout coll out 10u
.ends GAIN_STAGE

* Use the subcircuit twice
Xstage1 input mid vcc 0 GAIN_STAGE
Xstage2 mid output vcc 0 GAIN_STAGE
```

### Parameter Passing with .param

```spice
.param GAIN=100 RLOAD=10k

.subckt AMP in out vcc vee GAIN_VAL=100
R1 in base {100k * 100 / GAIN_VAL}
R2 base vee 22k
Rc vcc coll {RLOAD}
Q1 coll base emit 2N2222
.ends AMP

* Instantiate with parameter override
Xamp1 in out vcc 0 AMP GAIN_VAL=50
```

---

## 4. Control Statements

### .param — Parameters

Define symbolic values for reuse throughout the netlist.

**Syntax:**
```
.param name=value
.param name1=value1 name2=value2 ...
```

**Examples:**
```spice
.param VCC=9V RB1=100k RB2=22k
.param FREQ=1k CIN=10u

Vcc vcc 0 {VCC}
R1 vcc base {RB1}
C1 in base {CIN}
```

### .option — Simulator Options

**Syntax:**
```
.option optname=value
```

**Common Options:**
| Option | Default | Description |
|--------|---------|-------------|
| `TEMP` | 27 °C | Operating temperature |
| `TNOM` | 27 °C | Nominal temperature for model parameters |
| `RELTOL` | 0.001 | Relative error tolerance |
| `ABSTOL` | 1e-12 A | Absolute current error tolerance |
| `VNTOL` | 1e-6 V | Absolute voltage error tolerance |
| `ITL1` | 100 | DC operating point iteration limit |
| `ITL2` | 50 | DC transfer curve iteration limit |
| `ITL4` | 10 | Transient analysis timepoint iteration limit |
| `METHOD` | trapezoidal | Integration method (trapezoidal or gear) |

**Examples:**
```spice
.option TEMP=50
.option RELTOL=1e-4 ABSTOL=1e-9
```

### .include — Include Files

Include another netlist file.

**Syntax:**
```
.include "filepath"
```

**Examples:**
```spice
.include "models/2N2222.mod"
.include "subckts/opamps.lib"
```

### .end — End of Netlist

Marks the end of the netlist. Everything after `.end` is ignored.

**Syntax:**
```
.end
```

---

## 5. Melange Extensions

These directives are melange-specific and not part of standard SPICE. They control code generation and plugin parameter generation.

### .pot — Dynamic Potentiometer

Marks a resistor as runtime-variable. In generated plugins, each `.pot` becomes a parameter knob.

**Syntax:**
```
.pot Rname min_value max_value [default_value] ["Label"]
```

| Field | Description |
|-------|-------------|
| `Rname` | Resistor name (must start with R) |
| `min_value` | Minimum resistance (Ohms) |
| `max_value` | Maximum resistance (Ohms) |
| `default_value` | (Optional) Default resistance; must be between min and max. If omitted, uses the resistor's netlist nominal value. |
| `"Label"` | (Optional) Quoted parameter label for plugin UI |

**Constraints:** min < max (strictly). Maximum 64 combined `.pot` + `.wiper` leg entries per circuit.

**Examples:**
```spice
R_vol 1 0 50k
.pot R_vol 1k 100k "Volume"

R_tone mid out 10k
.pot R_tone 500 50k 10k "Tone"
```

**Notes:**
- All codegen paths use per-block O(N^3) matrix rebuild on value change
- Per-sample smoothing via `.smoothed.next()` interpolates between rebuilds
- Warm DC-OP re-init on large jumps (>20% relative change) prevents NR divergence

---

### .wiper — Three-Terminal Wiper Potentiometer

Models a 3-terminal pot (top, wiper, bottom) as two resistors sharing a wiper node. A single UI parameter (position 0.0-1.0) controls both resistor values inversely.

**Syntax:**
```
.wiper R_cw R_ccw total_R [default_pos] ["Label"]
```

| Field | Description |
|-------|-------------|
| `R_cw` | Name of the clockwise (top-to-wiper) leg resistor |
| `R_ccw` | Name of the counter-clockwise (wiper-to-bottom) leg resistor |
| `total_R` | Total pot resistance in ohms |
| `default_pos` | (Optional) Default wiper position, 0.0-1.0. Default: 0.5 |
| `"Label"` | (Optional) Quoted parameter label for plugin UI |

**Example:**
```spice
R_cw top wiper 50k
R_ccw wiper bottom 50k
.wiper R_cw R_ccw 100k 0.5 "Volume"
```

**Notes:**
- Both resistors must share exactly one node (the wiper node)
- Total resistance must be > 20 ohms
- At position 0.0: R_cw = total_R, R_ccw = minimum. At position 1.0: R_cw = minimum, R_ccw = total_R
- Minimum leg resistance is 10 ohms (models wiper contact resistance)
- Internally expands to two `.pot` entries; both count toward the 64-pot limit
- Wiper pots participate in `.gang` directives the same way as `.pot` entries

---

### .gang — Link Multiple Pots/Wipers to One Parameter

Links multiple `.pot` and/or `.wiper` entries to a single UI parameter. The gang position (0.0-1.0) controls all members simultaneously.

**Syntax:**
```
.gang "Label" member1 [!]member2 [...] [default]
```

| Field | Description |
|-------|-------------|
| `"Label"` | Quoted parameter label for the single UI knob |
| `memberN` | Component name from a `.pot` or `.wiper` directive |
| `!memberN` | Prefix `!` inverts the member's response (1.0 - position) |
| `default` | (Optional) Default position, 0.0-1.0 |

**Example:**
```spice
.pot R_treble 1k 250k "Treble"
.pot R_bass 1k 500k "Bass"
.gang "Tone" R_treble !R_bass 0.5
```

In this example, turning the "Tone" knob up increases R_treble and decreases R_bass.

**Notes:**
- Requires at least 2 members
- Ganged members are excluded from individual parameter generation in the plugin template
- One FloatParam is emitted per gang; the gang label becomes the parameter name
- Members can be from `.pot` or `.wiper` directives (or a mix)
- A resistor can only belong to one gang

---

### .switch — Multi-Position Component Switch

Defines a rotary switch that selects among discrete component values. Multiple components can be ganged (switched simultaneously).

**Syntax:**
```
.switch name1[,name2,...] pos0_vals pos1_vals [pos2_vals ...] ["Label"]
```

| Field | Description |
|-------|-------------|
| `name1,name2,...` | Comma-separated component names (R, C, or L only) |
| `posN_vals` | Slash-separated values for each component at position N |
| `"Label"` | (Optional) Quoted parameter label for plugin UI |

**Constraints:** Minimum 2, maximum 32 positions. Maximum 16 `.switch` directives per circuit. All values must be positive and finite.

**Examples:**
```spice
* Single component, 3 positions
C_bright 1 0 120p
.switch C_bright 1p 120p 470p "Bright"

* Ganged: cap + inductor switched together
C_hf 1 2 100n
L_hf 1 3 100m
.switch C_hf,L_hf 10n/10m 100n/100m 1u/1H "HF Select"
```

In the ganged example, position 0 sets C_hf=10n and L_hf=10m, position 1 sets C_hf=100n and L_hf=100m, etc.

---

### .input_impedance — Input Source Impedance

Sets the Thevenin source resistance for the audio input node. Affects how the circuit loads the input signal.

**Syntax:**
```
.input_impedance value
```

**Default:** 1 Ohm (near-ideal voltage source). Maximum one per netlist.

**Example:**
```spice
.input_impedance 600
```

**Notes:**
- Higher impedance (e.g. 10kOhm) will cause signal attenuation through coupling capacitors
- Use 600 Ohm for professional line-level circuits
- Can also be set via CLI: `--input-resistance 600`

---

### .linearize — Linearize Device at Operating Point

Removes a nonlinear device from the Newton-Raphson system and replaces it with small-signal conductances at its DC operating point. Reduces the nonlinear dimension M.

**Syntax:**
```
.linearize device_name [device_name ...]
```

**Supported devices:** BJTs (Q prefix) and triodes (T prefix).

**Examples:**
```spice
.linearize Q9         ; Linearize a Vbe-multiplier BJT
.linearize V1         ; Linearize a triode (reduces M by 2)
.linearize Q3 Q4 V2   ; Multiple devices on one line
```

**Notes:**
- `.linearize` is a **semantic** tool, not a CPU optimization. For small-signal stages that stay in their linear region, NR already converges in 0–1 iterations, so linearizing produces negligible CPU savings. The real reason to use it is to **force small-signal mode** — removing a device that happens to traverse a nonlinear knee from its DC bias but musically should not (e.g. a Vbe-multiplier BJT in a power amp, a clean cathode-bypass stage in a preamp). A linearized device cannot clip.
- BJTs: reduces M by 2 per device (stamps g_m, g_pi, r_o into G)
- Triodes: reduces M by 2 per device (stamps g_m, 1/r_p into G)
- The device still affects the circuit via its linearized conductances
- The linearization is computed from the DC operating point, so it is only accurate for small signals around that point

---

### .runtime — Host-Driven Voltage Source or Audio-Rate Resistor

`.runtime` has two forms, dispatched on the first character of the target
name (`V…` → voltage source; `R…` → resistor). Both bind a netlist element
to a plugin-writable surface on `CircuitState`.

#### .runtime V — Host-Driven Voltage Source

Binds an existing voltage source to a `pub <field>: f64` on the generated `CircuitState` so the plugin host can drive the source value per sample (sidechain CV, LFO, envelope follower, external modulation input).

**Syntax:**
```
.runtime Vname as field_name
```

**Requirements:**
- `Vname` must reference a voltage source (name starts with `V`) declared elsewhere in the netlist.
- `field_name` must be a valid ASCII Rust identifier.
- Each voltage source may be bound at most once; each field name is unique per circuit (shared namespace with `.runtime R`).

**Example:**
```spice
Vctrl ctrl 0 DC 0
R_ctrl ctrl vca_gain 10k
.runtime Vctrl as ctrl_voltage
```

**Generated code:**
```rust
struct CircuitState {
    // ...
    pub ctrl_voltage: f64,
}

fn build_rhs(input: f64, input_prev: f64, state: &CircuitState) -> [f64; N] {
    // ...
    rhs[VSOURCE_VCTRL_RHS_ROW] += state.ctrl_voltage;
    rhs
}
```

**Semantics:**
- Additive with any DC bias declared on the voltage source itself: `V1 n1 0 DC 5` + `.runtime V1 as foo` ⇒ `5 + state.foo` total.
- `reset()` zeroes each runtime field.
- Stamp site emitted in both trapezoidal and backward-Euler RHS builders and in the nodal solver's per-sample RHS.

#### .runtime R — Audio-Rate Resistor Modulation

Declares a resistor whose value the plugin drives at audio rate (envelope follower, LFO, sidechain dynamics). Unlike `.pot R`, the generated setter has **no DC-OP warm re-init** — the plugin-side envelope is the smoother, and a mid-signal NR-seed reset would be audible as a click.

**Syntax:**
```
.runtime Rname min max as field_name
```

**Requirements:**
- `Rname` must reference a resistor (name starts with `R`).
- `min < max`, both positive (ohms). These are the clamp range applied by the setter.
- `field_name` must be a valid ASCII Rust identifier (shared namespace with `.runtime V`).
- A resistor claimed by `.runtime R` cannot also be claimed by `.pot` or `.wiper`.
- `.gang` members must be `.pot` or `.wiper` — a `.runtime R` resistor listed in `.gang` is rejected at parse time (drive multiple `set_runtime_R_*` setters from one plugin envelope instead).

**Example:**
```spice
Rk_L1 cathode1 0 10k
.runtime Rk_L1 2k 12k as bias_r_L1
```

**Generated code:**
```rust
pub const RUNTIME_R_BIAS_R_L1_MIN: f64     = POT_0_MIN_R;     // 2_000.0
pub const RUNTIME_R_BIAS_R_L1_MAX: f64     = POT_0_MAX_R;     // 12_000.0
pub const RUNTIME_R_BIAS_R_L1_NOMINAL: f64 = 1.0 / POT_0_G_NOM; // 10_000.0

impl CircuitState {
    pub fn bias_r_L1(&self) -> f64 { self.pot_0_resistance }

    pub fn set_runtime_R_bias_r_L1(&mut self, resistance: f64) {
        // clamp, skip if unchanged, mark matrices dirty
        // Body is structurally identical to `set_pot_N` since the 2026-04-20
        // reseed strip — only the setter name + read-only accessor differ.
    }
}
```

**Semantics:**
- No nih-plug knob is emitted — the plugin is the driver.
- Setter clamps to `[RUNTIME_R_<FIELD>_MIN, RUNTIME_R_<FIELD>_MAX]`, sets `matrices_dirty = true`, returns. Rebuild of A/S/K is deferred to the next `process_sample`.
- `<field>()` returns the current resistance; `RUNTIME_R_<FIELD>_NOMINAL` exposes the netlist value for envelope-scale math.

See [DYNAMIC_PARAMS.md](aidocs/DYNAMIC_PARAMS.md#runtime-v--host-driven-voltage-source) for the full reference, including the `.pot R` ↔ `.runtime R` contrast table.

---

## 6. Analysis Directives (for validation only)

These directives are parsed but only used by the validation layer (melange-validate) for comparison against ngspice. They do not affect code generation.

### .op — DC Operating Point

Computes the DC bias point (all node voltages and branch currents).

**Syntax:**
```
.op
```

### .ac — AC Analysis

Performs small-signal AC frequency sweep.

**Syntax:**
```
.ac dec|oct|lin num_points fstart fstop
```

| Parameter | Description |
|-----------|-------------|
| `dec` | Decade variation |
| `oct` | Octave variation |
| `lin` | Linear variation |
| `num_points` | Points per decade/octave, or total for lin |
| `fstart` | Start frequency (Hz) |
| `fstop` | Stop frequency (Hz) |

**Example:**
```spice
.ac dec 20 10 100k
```

### .tran — Transient Analysis

Time-domain simulation.

**Syntax:**
```
.tran tstep tstop [tstart [tmax]] [uic]
```

| Parameter | Description |
|-----------|-------------|
| `tstep` | Printing/plotting increment |
| `tstop` | Final time |
| `tstart` | (Optional) Start time for output |
| `tmax` | (Optional) Maximum timestep |
| `uic` | Use initial conditions (skip DC op) |

**Example:**
```spice
.tran 1u 10m
.tran 10n 1m 0 1n uic
```

### .print — Output Variables

Specifies which variables to output during analysis.

**Syntax:**
```
.print analysis_type var1 var2 ...
```

**Variable Formats:**
| Format | Description |
|--------|-------------|
| `V(node)` | Voltage at node |
| `V(node1,node2)` | Voltage difference |
| `I(device)` | Current through device |
| `P(device)` | Power dissipation |

**Examples:**
```spice
.print dc V(base) V(coll) I(Q1)
.print ac V(out) VP(out)     ; magnitude and phase
.print tran V(in) V(out)
```

---

## 7. Not Supported

The following SPICE features are **not supported** by melange-solver:

### Nonlinear Reactive Components
- Nonlinear capacitors (`C` with nonlinear expression)
- Nonlinear inductors with arbitrary expressions (note: `ISAT=` tanh saturation IS supported for uncoupled inductors)
- Nonlinear magnetic core models (SPICE `.model CORE` syntax)
- Coupled saturating inductors (saturation only works on uncoupled inductors)

### Transmission Lines
- Lossless transmission lines
- Lossy transmission lines (`O` / URC model)

### Advanced Sources
- Behavioral voltage/current sources (`B` sources)
- Arbitrary expressions in source values
- Polynomial sources (SPICE2 style)
- Table-based sources
- Noise sources

### Device Models
- IBIS models
- Verilog-A models
- Custom compiled device models
- Temperature-dependent model parameters (TC1, TC2 on resistors only partially supported)

### Analysis Types
- DC sweep (`.dc`)
- Pole-zero analysis (`.pz`)
- Sensitivity analysis (`.sens`)
- Distortion analysis (`.disto`)
- Noise analysis (`.noise`)
- Temperature sweep (`.temp`)
- Monte Carlo analysis
- Worst-case analysis

### Output Control
- `.plot` and `.probe` directives
- `.save` directives
- `.measure` statements
- Fourier analysis (`.four`)

### Circuit Modifications
- `.alter` statements
- `.delete` statements
- Multiple circuit simulations per netlist

### Miscellaneous
- Node names longer than 32 characters
- Unicode in node names (ASCII only)
- Bus notation (`D[0:7]`)
- Digital components (from XSPICE)

---

## 8. Netlist Preprocessing

### Case Insensitivity

SPICE is case-insensitive. The following are equivalent:
```spice
R1 in out 10k
r1 IN OUT 10K
r1 In OuT 10k
```

However, file paths in `.include` statements may be case-sensitive depending on the filesystem.

### Comment Handling

Two comment styles are supported:

**Asterisk (`*`) at start of line:**
```spice
* This is a full-line comment
R1 in out 10k  * This is NOT a comment (part of value)
```

**Semicolon (`;`) anywhere in line:**
```spice
R1 in out 10k  ; This is an inline comment
; This is also a full-line comment
```

### Line Continuation

Long lines can be continued with `+` at the start of the continuation line:
```spice
.model 2N2222 NPN(IS=1e-15 BF=200 VAF=100
+                CJE=20p CJC=10p TF=400p)
```

The `+` must be the first non-whitespace character on the continuation line.

### Scale Suffixes

Numerical values can include these suffixes (case-insensitive):

| Suffix | Value | Example |
|--------|-------|---------|
| `f` | 10⁻¹⁵ | `10f` = 10 femto |
| `p` | 10⁻¹² | `100p` = 100 pico |
| `n` | 10⁻⁹ | `10n` = 10 nano |
| `u` or `μ` | 10⁻⁶ | `10u` = 10 micro |
| `m` | 10⁻³ | `1m` = 1 milli |
| `k` | 10³ | `10k` = 10 kilo |
| `meg` | 10⁶ | `1meg` = 1 mega |
| `g` | 10⁹ | `1g` = 1 giga |
| `t` | 10¹² | `1t` = 1 tera |

**Note:** `M` alone means milli (10⁻³) in SPICE convention, not mega. Always use `meg` for 10⁶.

```spice
R1 in out 1meg     ; 1 MΩ (correct)
R2 in out 1M       ; 1 mΩ (probably not what you want!)
R3 in out 1MEG     ; 1 MΩ (also correct)
C1 in out 10u      ; 10 μF
L1 in out 100n     ; 100 nH
```

### Engineering Notation

Values can use scientific notation:
```spice
R1 in out 1e6      ; 1 MΩ
C1 in out 1e-6     ; 1 μF
I1 0 in 1e-3       ; 1 mA
```

### Unit Suffixes (Optional)

Unit letters (V, A, Ω, F, H, Hz) are allowed but ignored:
```spice
Vcc vcc 0 9V       ; 9 volts
R1 in out 10kohm   ; 10 kΩ
C1 in out 10uF     ; 10 μF
```

---

## 9. Example Netlist

A complete, parseable common-emitter amplifier:

```spice
* Simple common-emitter amplifier
* Bias network
Vcc vcc 0 9V
R1 vcc base 100k
R2 base 0 22k

* Coupling capacitors
C1 in base 10u
Vin in 0 DC 0 AC 1V

* Transistor stage
Rc vcc coll 4.7k
Re emit 0 1k
Ce emit 0 100u
Q1 coll base emit 2N2222

* Output coupling
C2 coll out 10u
Rload out 0 10k

* Transistor model
.model 2N2222 NPN(IS=1e-15 BF=200 VAF=100
+                  CJE=20p CJC=10p TF=400p)

* Analysis (for validation)
.op
.ac dec 20 10 1meg
.tran 1u 10m
.print ac V(out) V(in)

.end
```

This netlist demonstrates:
- Resistors, capacitors, voltage sources
- A BJT with `.model` definition
- Multi-line `.model` with continuation
- Analysis directives for validation
- Proper use of scale suffixes
- Comments

---

## References

- SPICE3 User's Manual, UC Berkeley
- ngspice User Manual: https://ngspice.sourceforge.io/docs.html
- "Semiconductor Device Modeling with SPICE" by Paolo Antognetti and Giuseppe Massobrio
