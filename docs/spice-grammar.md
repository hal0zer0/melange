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
Lname n+ n- value [IC=initial_current]
```

**Parameters:**
| Parameter | Description |
|-----------|-------------|
| `n+` | Positive terminal node |
| `n-` | Negative terminal node |
| `value` | Inductance in henries (H) |
| `IC` | (Optional) Initial current at t=0 |

**Examples:**
```spice
L1 in out 10m
Lchoke vcc coll 100u
```

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
| `VAF` | ∞ V | Forward Early voltage |
| `VAR` | ∞ V | Reverse Early voltage |
| `IKF` | ∞ A | Forward beta high-current rolloff |
| `IKR` | ∞ A | Reverse beta high-current rolloff |
| `CJE` | 0 F | Base-emitter zero-bias capacitance |
| `CJC` | 0 F | Base-collector zero-bias capacitance |
| `VJE` | 0.75 V | Base-emitter built-in potential |
| `VJC` | 0.75 V | Base-collector built-in potential |
| `MJE` | 0.33 | Base-emitter grading coefficient |
| `MJC` | 0.33 | Base-collector grading coefficient |
| `TF` | 0 s | Ideal forward transit time |
| `TR` | 0 s | Ideal reverse transit time |

**Example:**
```spice
.model 2N2222 NPN(IS=1e-15 BF=200 VAF=100 CJE=20p CJC=10p)
.model 2N3906 PNP(IS=1.41e-15 BF=180 VAF=80)
```

### JFET Parameters (Types: NJF, PJF)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `VTO` | -2.0 V | Threshold voltage |
| `BETA` | 1e-4 A/V² | Transconductance parameter |
| `LAMBDA` | 0 V⁻¹ | Channel-length modulation |
| `RD` | 0 Ω | Drain ohmic resistance |
| `RS` | 0 Ω | Source ohmic resistance |
| `CGS` | 0 F | Gate-source capacitance |
| `CGD` | 0 F | Gate-drain capacitance |
| `PB` | 1.0 V | Gate junction potential |

**Example:**
```spice
.model J2N5457 NJF(VTO=-1.5 BETA=1e-4 LAMBDA=0.01)
```

### MOSFET Parameters (Types: NMOS, PMOS)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `VTO` | 0.0 V | Threshold voltage |
| `KP` | 2e-5 A/V² | Transconductance parameter |
| `LAMBDA` | 0 V⁻¹ | Channel-length modulation |
| `GAMMA` | 0 V¹/² | Body effect parameter |
| `PHI` | 0.6 V | Surface potential |
| `RD` | 0 Ω | Drain ohmic resistance |
| `RS` | 0 Ω | Source ohmic resistance |
| `CGSO` | 0 F/m | Gate-source overlap capacitance |
| `CGDO` | 0 F/m | Gate-drain overlap capacitance |
| `CGBO` | 0 F/m | Gate-bulk overlap capacitance |
| `CJ` | 0 F/m² | Zero-bias bulk junction capacitance |
| `MJ` | 0.5 | Bulk junction grading coefficient |
| `PB` | 0.8 V | Bulk junction potential |

**Example:**
```spice
.model NMOS1 NMOS(VTO=0.7 KP=50e-6 LAMBDA=0.02 GAMMA=0.37)
.model PMOS1 PMOS(VTO=-0.8 KP=25e-6 LAMBDA=0.02)
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

## 5. Analysis Directives (for validation only)

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

## 6. Not Supported

The following SPICE features are **not supported** by melange-solver:

### Nonlinear Reactive Components
- Nonlinear capacitors (`C` with nonlinear expression)
- Nonlinear inductors (`L` with nonlinear expression)
- Nonlinear magnetic core models

### Transmission Lines
- Lossless transmission lines (`T`)
- Lossy transmission lines (`O` / URC model)
- Coupled transmission lines (`K` for transmission lines)

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

## 7. Netlist Preprocessing

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

## 8. Example Netlist

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
