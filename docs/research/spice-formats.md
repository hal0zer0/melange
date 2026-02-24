# SPICE Simulation Formats and ngspice Specifics

**Purpose:** Reference document for implementing SPICE netlist parsers and ngspice interfaces.

---

## 1. SPICE Netlist Format

### 1.1 File Structure

```
<title line>           (optional, first line is always title)
<element statements>   (circuit components)
<model definitions>    (.model cards)
<subcircuit defs>      (.subckt/.ends)
<control statements>   (.option, .param, .include)
<analysis directives>  (.op, .ac, .tran, etc.)
.end                   (required terminator)
```

**Key Rules:**
- First line is always the title (even if blank/comment)
- Lines are processed sequentially; forward references allowed
- Case-insensitive except file paths
- Max line length: traditionally 80 chars, ngspice allows longer
- `.end` is mandatory; everything after is ignored

### 1.2 Element Statement Format

```
X<name> <nodes...> <value/model> [parameters]
```

| Position | Content | Notes |
|----------|---------|-------|
| 1 | Element letter | R, C, L, V, I, D, Q, M, J, E, F, G, H, X |
| 2+ | Name | Alphanumeric, unique per type |
| Following | Nodes | Node names (0 = ground) |
| Last | Value/Model | Numeric value or model reference |

### 1.3 Node Naming Rules

```spice
* Valid node names
0              ; Ground (always node 0)
1              ; Numeric
in             ; Alphabetic
out_1          ; Alphanumeric with underscore
vcc_emit       ; Compound names
<net_name>     ; ngspice: angle brackets allow special chars
```

**Constraints:**
- Node 0 is always the global reference (GND)
- Node names are case-insensitive (`Vcc` = `VCC` = `vcc`)
- Traditional SPICE: 32 char limit; ngspice: effectively unlimited
- Avoid: `+ - / * = ( ) [ ] { }` without angle brackets

### 1.4 Numeric Value Parsing

#### Scale Suffixes (CRITICAL)

| Suffix | Multiplier | Example | Parsed Value |
|--------|------------|---------|--------------|
| `f` / `F` | 10⁻¹⁵ | `5f` | 5e-15 |
| `p` / `P` | 10⁻¹² | `100p` | 100e-12 |
| `n` / `N` | 10⁻⁹ | `10n` | 10e-9 |
| `u` / `U` / `μ` | 10⁻⁶ | `4.7u` | 4.7e-6 |
| `m` / `M` | 10⁻³ | `1m` | 1e-3 |
| `k` / `K` | 10³ | `10k` | 10e3 |
| `meg` / `MEG` | 10⁶ | `1meg` | 1e6 |
| `g` / `G` | 10⁹ | `1g` | 1e9 |
| `t` / `T` | 10¹² | `1t` | 1e12 |
| `mil` | 25.4×10⁻⁶ | `10mil` | 254e-6 |

**⚠️ COMMON BUG:** `1M` = 1 milliOhm, NOT 1 MegaOhm. Must use `meg` for 10⁶.

```spice
R1 n1 n2 1M       ; WRONG: 1 milliohm
R1 n1 n2 1meg     ; CORRECT: 1 megohm
R1 n1 n2 1MEG     ; CORRECT: 1 megohm
```

#### Unit Suffixes (Ignored)

Unit letters are allowed but ignored during parsing:
```spice
R1 n1 n2 10k      ; 10,000 ohms
R1 n1 n2 10kohm   ; same
R1 n1 n2 10kOhms  ; same
V1 n1 0 5V        ; 5 volts
C1 n1 0 100pF     ; 100 picofarads
```

#### Scientific Notation

```spice
R1 n1 n2 1e3      ; 1000
R1 n1 n2 1E3      ; same (case insensitive)
R1 n1 n2 1.5e-6   ; 1.5 micro
R1 n1 n2 2.2e+3   ; 2200
```

#### Temperature Suffixes (ngspice)

```spice
.option TEMP=27C    ; Celsius (default)
.option TEMP=300K   ; Kelvin
.option TEMP=80.6F  ; Fahrenheit
```

### 1.5 Comment Handling

```spice
* This is a full-line comment (asterisk in column 1)
R1 n1 n2 10k       ; inline comment (semicolon)
; This is also a full-line comment
R2 n2 n3 20k $ ngspice also accepts $ for inline (not standard)
```

**Parser Strategy:**
1. Remove everything after `;` or `$` (if supported)
2. Skip lines starting with `*` or `;` (after trimming)
3. Preserve comments for error reporting context

### 1.6 Line Continuation

```spice
.model 2N2222 NPN(IS=1e-15 BF=200 VAF=100
+                CJE=20p CJC=10p TF=400p)
```

**Rules:**
- `+` must be first non-whitespace character on continuation line
- No limit on continuation lines
- Treat as single logical line for parsing

**Implementation Pattern:**
```rust
fn preprocess_lines(lines: &[String]) -> Vec<String> {
    let mut result = Vec::new();
    let mut current = String::new();
    
    for line in lines {
        let trimmed = line.trim_start();
        if trimmed.starts_with('+') {
            current.push(' ');
            current.push_str(&trimmed[1..]);
        } else {
            if !current.is_empty() {
                result.push(current.clone());
            }
            current = line.to_string();
        }
    }
    if !current.is_empty() {
        result.push(current);
    }
    result
}
```

---

## 2. ngspice-Specific Extensions

### 2.1 Behavioral Sources (B-Sources)

ngspice supports arbitrary mathematical expressions:

```spice
* Voltage-controlled voltage source with expression
B1 out 0 V=2*V(in) + 0.1*V(in)^2

* Current source with conditional
B2 out 0 I=(V(in)>0) ? V(in)/1k : 0

* Voltage limiting (diode approximation)
Bclip out 0 V=uramp(V(in))           ; unilateral ramp (positive only)

* Lookup table
B3 out 0 V=table(V(in), -1,-1, 0,0, 1,1)  ; piecewise linear
```

**Built-in Functions (ngspice):**

| Function | Description |
|----------|-------------|
| `abs(x)` | Absolute value |
| `acos(x)` | Arc cosine |
| `acosh(x)` | Arc hyperbolic cosine |
| `asin(x)` | Arc sine |
| `asinh(x)` | Arc hyperbolic sine |
| `atan(x)` | Arc tangent |
| `atanh(x)` | Arc hyperbolic tangent |
| `cos(x)` | Cosine |
| `cosh(x)` | Hyperbolic cosine |
| `exp(x)` | Exponential eˣ |
| `ln(x)` / `log(x)` | Natural logarithm |
| `log10(x)` | Base-10 logarithm |
| `sgn(x)` | Sign function (-1, 0, 1) |
| `sin(x)` | Sine |
| `sinh(x)` | Hyperbolic sine |
| `sqrt(x)` | Square root |
| `tan(x)` | Tangent |
| `tanh(x)` | Hyperbolic tangent |
| `uramp(x)` | Unit ramp: max(0, x) |
| `u(x)` / `step(x)` | Unit step: x >= 0 ? 1 : 0 |
| `pwl(t, t1,v1, t2,v2, ...)` | Piece-wise linear |
| `table(x, x1,y1, x2,y2, ...)` | Lookup table |

**Special Variables:**
- `time` - Current simulation time (transient only)
- `temp` - Current temperature
- `vt` - Thermal voltage (kT/q)

### 2.2 Extended Device Models

#### Extended Diode Model

```spice
.model D1N4148 D(
+    IS=2.52e-9      ; Saturation current
+    N=1.752         ; Ideality factor
+    RS=0.568        ; Series resistance
+    TT=4e-9         ; Transit time
+    CJO=1.95e-12    ; Zero-bias junction capacitance
+    VJ=0.6          ; Junction potential
+    M=0.333         ; Grading coefficient
+    EG=1.11         ; Band gap energy (eV)
+    XTI=3.0         ; IS temperature exponent
+    KF=0            ; Flicker noise coefficient
+    AF=1            ; Flicker noise exponent
+    FC=0.5          ; Forward bias depletion capacitance coefficient
+    BV=100          ; Reverse breakdown voltage
+    IBV=1e-3        ; Current at breakdown
+    TNOM=27         ; Nominal temperature
+)
```

#### Extended BJT Model (Gummel-Poon)

```spice
.model 2N2222 NPN(
+    IS=1e-15        ; Transport saturation current
+    BF=200          ; Ideal maximum forward beta
+    NF=1.0          ; Forward emission coefficient
+    VAF=100         ; Forward Early voltage
+    IKF=0.3         ; Forward beta rolloff current
+    NK=0.5          ; High-current roll-off coefficient
+    BR=1            ; Ideal maximum reverse beta
+    NR=1.0          ; Reverse emission coefficient
+    VAR=50          ; Reverse Early voltage
+    IKR=0.3         ; Reverse beta rolloff current
+    RB=10           ; Zero-bias base resistance
+    IRB=0.1         ; Current at RB drop half
+    RBM=10          ; Minimum base resistance
+    RE=0.5          ; Emitter resistance
+    RC=0.5          ; Collector resistance
+    CJE=20e-12      ; BE zero-bias capacitance
+    VJE=0.75        ; BE built-in potential
+    MJE=0.33        ; BE grading coefficient
+    TF=400e-12      ; Ideal forward transit time
+    XTF=3           ; TF bias dependence coefficient
+    VTF=2           ; TF dependence on Vbc
+    ITF=0.6         ; TF dependence on Ic
+    PTF=0           ; Excess phase at f=1/(2πTF)
+    CJC=10e-12      ; BC zero-bias capacitance
+    VJC=0.75        ; BC built-in potential
+    MJC=0.33        ; BC grading coefficient
+    XCJC=1.0        ; Fraction of CJC connected to internal base
+    TR=50e-9        ; Ideal reverse transit time
+    CJS=0           ; CS zero-bias capacitance
+    VJS=0.75        ; CS built-in potential
+    MJS=0           ; CS grading coefficient
+    XTB=0           ; Beta temperature exponent
+    EG=1.11         ; Band gap energy
+    XTI=3.0         ; IS temperature exponent
+    KF=0            ; Flicker noise coefficient
+    AF=1            ; Flicker noise exponent
+    FC=0.5          ; Forward bias depletion capacitance
+    TNOM=27         ; Nominal temperature
+)
```

### 2.3 Extended Control Statements

#### .lib (Library Include)

```spice
.lib /path/to/library.lib section_name
... circuit ...
.endl

* Or include specific section
.lib 'models.lib' diode_models
```

#### .func (User-Defined Functions)

```spice
.func mylimit(x, pos, neg) {min(max(x, neg), pos)}
.func gain(x) {x > 0 ? 10*x : x}

B1 out 0 V=mylimit(V(in), 5, -5)
```

#### .csparam (Control Section Parameters)

```spice
.csparam fstart = 10
.csparam fstop = 100k

.ac dec 10 {fstart} {fstop}
```

#### .global (Global Nodes)

```spice
.global vcc vss vdd  ; These nodes are global across subcircuits
```

#### .temp (Temperature Sweep)

```spice
.temp 0 25 50 75 100  ; Run at multiple temperatures
```

### 2.4 ngspice Simulation Control

```spice
.control
    * Interactive control block
    run
    plot v(out) v(in)
    print v(out)
    wrdata output.txt v(out)
    set filetype=ascii
    write output.raw
.endc
```

**Common .control Commands:**

| Command | Description |
|---------|-------------|
| `run` | Execute simulation |
| `op` | DC operating point |
| `dc <src> <start> <stop> <incr>` | DC sweep |
| `ac <type> <np> <fstart> <fstop>` | AC analysis |
| `tran <tstep> <tstop>` | Transient |
| `plot <expr>` | Plot variables |
| `print <expr>` | Print values |
| `set <var>=<value>` | Set option |
| `let <var>=<expr>` | Define variable |
| `wrdata <file> <expr>` | Write ASCII data |
| `write <file>` | Write binary .raw |
| `load <file>` | Load raw file |
| `display` | Show defined vectors |
| `quit` | Exit |

### 2.5 ngspice Differences from SPICE3

| Feature | SPICE3 | ngspice |
|---------|--------|---------|
| B-sources | Limited | Full expressions |
| XSPICE | No | Yes (digital/mixed) |
| FFT | No | Yes (`.four`, `.fft`) |
| Sensitivity | `.sens` | Enhanced |
| Pole-zero | `.pz` | Enhanced |
| Noise | `.noise` | Enhanced |
| POLY sources | Supported | Deprecated (use B-sources) |
| Error handling | Silent fail | Verbose warnings |

---

## 3. SPICE .raw File Format

### 3.1 Binary Format Structure

The .raw file is ngspice's native output format. Binary format is default.

```
[Header Section]
[Plot Header]
[Variable Definitions]
[Data Points]
```

### 3.2 Header Format

```
Title: <circuit title>
Date: <timestamp>
Plotname: <analysis type>
Flags: <format flags>
No. Variables: <n>
No. Points: <m>
Dimensions: <dim info for FFT>  ; Optional
Command: <command line>
Variables:
    <index> <name> <type> [device params]
    ...
Binary:
    <raw binary data>
```

**Example Header (ASCII):**
```
Title: Common Emitter Amplifier
Date: Sun Feb 23 10:00:00  2026
Plotname: AC Analysis
Flags: complex
No. Variables: 3
No. Points: 101
Variables:
    0       frequency       frequency       grid=3
    1       v(out)          voltage
    2       v(in)           voltage
Binary:
    [binary data follows]
```

### 3.3 Variable Types

| Type Code | Description |
|-----------|-------------|
| `time` | Transient analysis time |
| `frequency` | AC analysis frequency |
| `voltage` | Node voltage |
| `current` | Branch current |
| `voltage-spectrum` | FFT voltage |
| `current-spectrum` | FFT current |
| `node` | DC operating point |

### 3.4 Data Format

#### Real Data (Transient, DC)

Each point: `[time/freq, v1, v2, ..., vn]` as 8-byte doubles

```rust
// Structure for real data point
struct RealPoint {
    independent: f64,  // time or frequency
    values: Vec<f64>,  // one per variable (except independent)
}
```

#### Complex Data (AC Analysis)

Each value is two 8-byte doubles (real, imag) or (mag, phase):

```rust
// Structure for complex data point
struct ComplexPoint {
    frequency: f64,    // always real
    values: Vec<(f64, f64)>,  // (real, imag) pairs
}
```

**Flags Field Values:**
- `real` - Real data (transient, DC)
- `complex` - Complex data (AC)
- `nodata` - No data (for .op with no output)
- `fft` - FFT output
- `psd` - Power spectral density

### 3.5 ASCII vs Binary

```spice
* Force ASCII format
.option filetype=ascii
.write output.raw

* Or in .control block
.control
    set filetype=ascii
    write output.raw
.endc
```

**ASCII Format:**
```
Title: Circuit
Date: ...
...
Values:
    0   1.000000e-03    5.000000e+00    1.000000e-03
    1   1.010000e-03    5.050000e+00    1.010000e-03
```

### 3.6 Parsing Implementation

```rust
use std::io::{Read, BufRead};

pub struct RawFile {
    pub title: String,
    pub date: String,
    pub plotname: String,
    pub flags: RawFlags,
    pub variables: Vec<Variable>,
    pub data: Vec<DataPoint>,
}

pub struct Variable {
    pub index: usize,
    pub name: String,
    pub var_type: String,
}

pub enum DataPoint {
    Real(Vec<f64>),
    Complex(Vec<(f64, f64)>),
}

pub struct RawFlags {
    pub is_real: bool,
    pub is_complex: bool,
    pub is_fft: bool,
}

impl RawFile {
    pub fn parse<R: Read>(reader: R) -> Result<Self, ParseError> {
        // Detect ASCII vs Binary from header
        // Parse header lines until "Variables:"
        // Parse variable definitions
        // Parse data section
        todo!()
    }
    
    /// Get voltage at specific frequency (AC analysis)
    pub fn get_voltage(&self, node: &str, freq: f64) -> Option<(f64, f64)> {
        // Find variable index, interpolate to frequency
        todo!()
    }
}
```

### 3.7 Multi-Plot Raw Files

ngspice can store multiple analyses in one file:

```
Title: ...
... (first plot: .op)
...
Title: ...
... (second plot: .ac)
...
```

Each plot has its own header and data section. Must scan for multiple `Title:` occurrences.

---

## 4. Common Device Models and Parameters

### 4.1 Resistor (R)

```spice
Rname n+ n- value [TC=tc1[,tc2]] [AC=value]

* Examples:
R1 in out 10k
R2 n1 n2 1k TC=0.001,0.00001  ; Temperature coefficients
R3 n1 n2 10k AC=5k            ; Different value for AC analysis
```

**Temperature Model:**
```
R(T) = Rnom * (1 + TC1*(T-Tnom) + TC2*(T-Tnom)²)
```

### 4.2 Capacitor (C)

```spice
Cname n+ n- value [IC=init_voltage] [TC=tc1[,tc2]]

* Examples:
C1 in out 100p
C2 n1 n2 10u IC=0              ; Initial condition
C3 n1 n2 1n TC=0.0002          ; Temperature coefficient
```

### 4.3 Inductor (L)

```spice
Lname n+ n- value [IC=init_current]

* Examples:
L1 in out 10m
L2 n1 n2 100u IC=0.1           ; Initial current 100mA
```

### 4.4 Mutual Inductor (K)

```spice
Kname L1 L2 coupling_factor

* Example: Transformer with 95% coupling
L1 primary 0 1m
L2 secondary 0 10m
K12 L1 L2 0.95
```

### 4.5 Diode Model Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| IS | 1e-14 | A | Saturation current |
| RS | 0 | Ω | Series resistance |
| N | 1 | - | Ideality factor |
| TT | 0 | s | Transit time |
| CJO | 0 | F | Zero-bias junction capacitance |
| VJ | 1 | V | Junction potential |
| M | 0.5 | - | Grading coefficient |
| EG | 1.11 | eV | Band gap voltage |
| XTI | 3 | - | IS temperature exponent |
| KF | 0 | - | Flicker noise coefficient |
| AF | 1 | - | Flicker noise exponent |
| FC | 0.5 | - | Forward bias capacitance coeff |
| BV | ∞ | V | Reverse breakdown voltage |
| IBV | 1e-3 | A | Current at breakdown |
| TNOM | 27 | °C | Nominal temperature |

### 4.6 BJT Model Parameters (Key)

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| IS | 1e-16 | A | Transport saturation current |
| BF | 100 | - | Ideal forward beta |
| BR | 1 | - | Ideal reverse beta |
| NF | 1 | - | Forward emission coeff |
| NR | 1 | - | Reverse emission coeff |
| VAF | ∞ | V | Forward Early voltage |
| VAR | ∞ | V | Reverse Early voltage |
| IKF | ∞ | A | Forward beta rolloff current |
| IKR | ∞ | A | Reverse beta rolloff current |
| RB | 0 | Ω | Zero-bias base resistance |
| RBM | RB | Ω | Minimum base resistance |
| RE | 0 | Ω | Emitter resistance |
| RC | 0 | Ω | Collector resistance |
| CJE | 0 | F | BE zero-bias capacitance |
| CJC | 0 | F | BC zero-bias capacitance |
| CJS | 0 | F | CS zero-bias capacitance |
| VJE | 0.75 | V | BE built-in potential |
| VJC | 0.75 | V | BC built-in potential |
| MJE | 0.33 | - | BE grading coefficient |
| MJC | 0.33 | - | BC grading coefficient |
| TF | 0 | s | Forward transit time |
| TR | 0 | s | Reverse transit time |
| XTB | 0 | - | Beta temperature exponent |
| EG | 1.11 | eV | Band gap voltage |
| XTI | 3 | - | IS temperature exponent |
| TNOM | 27 | °C | Nominal temperature |

### 4.7 JFET Model Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| VTO | -2.0 | V | Threshold voltage |
| BETA | 1e-4 | A/V² | Transconductance parameter |
| LAMBDA | 0 | V⁻¹ | Channel-length modulation |
| RD | 0 | Ω | Drain resistance |
| RS | 0 | Ω | Source resistance |
| CGS | 0 | F | Gate-source capacitance |
| CGD | 0 | F | Gate-drain capacitance |
| PB | 1.0 | V | Gate junction potential |
| IS | 1e-14 | A | Gate junction saturation current |

### 4.8 MOSFET Model Parameters (Level 1 - Shichman-Hodges)

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| VTO | 0.0 | V | Threshold voltage |
| KP | 2e-5 | A/V² | Transconductance |
| GAMMA | 0 | V¹/² | Body effect parameter |
| PHI | 0.6 | V | Surface potential |
| LAMBDA | 0 | V⁻¹ | Channel-length modulation |
| RD | 0 | Ω | Drain resistance |
| RS | 0 | Ω | Source resistance |
| CGSO | 0 | F/m | Gate-source overlap capacitance |
| CGDO | 0 | F/m | Gate-drain overlap capacitance |
| CGBO | 0 | F/m | Gate-bulk overlap capacitance |
| CJ | 0 | F/m² | Zero-bias bulk capacitance |
| MJ | 0.5 | - | Bulk junction grading coeff |
| PB | 0.8 | V | Bulk junction potential |

---

## 5. SPICE Netlist Parsing Gotchas

### 5.1 Numeric Parsing Edge Cases

```spice
* Leading decimal (valid)
R1 n1 n2 .5k        ; 500 ohms

* Trailing decimal (valid)
R1 n1 n2 10.        ; 10 ohms

* Multiple suffixes (INVALID - but some simulators accept)
R1 n1 n2 1k5        ; Some parse as 1.5k, others error

* Exponent with suffix (valid in ngspice)
R1 n1 n2 1e3k       ; 1e3 * 1e3 = 1e6 = 1 meg

* Negative values (context dependent)
V1 n1 0 -5V         ; -5 volts (valid)
R1 n1 n2 -1k        ; Negative resistance (may be error or valid)
```

### 5.2 Node Name Collisions

```spice
* These are DIFFERENT nodes in ngspice:
Vcc                 ; Alphabetic node
VCC                 ; Same as Vcc (case insensitive)
0                   ; Ground (always)
"V cc"              ; With space (quoted string)
<V cc>              ; With space (angle bracket)

* Potential confusion:
1                   ; Node 1
01                  ; Node 01 (different from 1 in some SPICEs)
001                 ; Node 001
```

### 5.3 Model Name Resolution

```spice
* Model must be defined BEFORE use (some simulators)
Q1 c b e 2N2222     ; Error if .model comes after

* Model names are case-insensitive
.model 2n2222 NPN(...)   ; This works
Q1 c b e 2N2222          ; Matches above

* Multiple models with same name
.model NPN NPN(...)  ; Last definition wins in ngspice
.model NPN NPN(...)
```

### 5.4 Subcircuit Parameter Substitution

```spice
* Curly braces REQUIRED for expressions
.param GAIN=10
R1 n1 n2 {100k/GAIN}     ; Valid
R1 n1 n2 100k/GAIN       ; INVALID - parsed as single token

* Parameters in subcircuit instantiations
X1 in out AMP GAIN=50    ; Override default

* Nested parameter references
.param F0=1k
.param C1={1/(2*3.14159*F0*10k)}  ; Expression referencing param
```

### 5.5 Line Continuation Traps

```spice
* Valid: + at start
.model D1 D(
+   IS=1e-15)

* Invalid: whitespace before +
.model D1 D(
   +   IS=1e-15)      ; + not recognized

* Comment after continuation
R1 n1 n2 10k
+   TC=0.001           ; This is still part of line!
```

### 5.6 Comment Ambiguity

```spice
* Asterisk MUST be column 1
R1 n1 n2 10k * comment    ; INVALID - * not special here
 R1 n1 n2 10k             ; Leading space - not a comment!

* Semicolon anywhere
R1 n1 n2 10k ; comment    ; Valid
; comment                 ; Valid full-line comment

* ngspice $ extension
R1 n1 n2 10k $ comment    ; Valid in ngspice only
```

### 5.7 Device Name Collisions

```spice
* Same name, different types (usually allowed)
R1 n1 n2 1k
C1 n1 n2 1p          ; Different device type, same numeric suffix OK

* Duplicate names (usually error)
R1 n1 n2 1k
R1 n1 n2 2k          ; Error: duplicate R1

* Case insensitive
r1 n1 n2 1k
R1 n1 n2 2k          ; Error: R1 = r1
```

### 5.8 Ground Node Handling

```spice
* Node 0 is ALWAYS ground
0                   ; Ground node
GND                 ; NOT automatically ground (unless .global GND)

* Floating nodes
R1 n1 n2 1k         ; n1 and n2 floating - matrix singular

* ngspice auto-ground via .option
.option gmin=1e-12  ; Small conductance to ground (default)
```

### 5.9 Initial Conditions

```spice
* IC on capacitor
C1 n1 n2 10u IC=5V   ; Valid

* IC on inductor
L1 n1 n2 1m IC=1mA   ; Valid

* .ic directive
.ic V(n1)=5 V(n2)=0  ; Set initial node voltages

* uic flag
.tran 1u 10m uic     ; Use initial conditions (skip DC op)
```

### 5.10 Temperature Handling

```spice
* Circuit temperature
.option TEMP=50      ; Operating temperature
.option TNOM=27      ; Nominal (parameter measurement) temperature

* Multiple temperatures
.temp 0 25 50 75 100 ; Run at each temperature

* Device-level temperature (ngspice extension)
Q1 c b e NPN temp=100
```

---

## 6. ngspice Interfacing

### 6.1 Command Line Usage

```bash
# Basic simulation
ngspice circuit.cir

# Batch mode
ngspice -b circuit.cir

# Output to file
ngspice -b circuit.cir -o output.log

# Set raw file output
ngspice -b -r output.raw circuit.cir

# No splash screen, batch mode
ngspice -s -b circuit.cir
```

### 6.2 Netlist for ngspice Batch Mode

```spice
* Circuit description
V1 in 0 DC 1 AC 1
R1 in out 1k
C1 out 0 1n

* Analysis
.ac dec 10 1 1meg

* Output control
.control
    set filetype=binary
    write output.raw
    wrdata output.txt v(out)   ; ASCII alternative
.endc

.end
```

### 6.3 Shared Library Interface (libngspice)

ngspice can be compiled as a shared library for embedding:

```c
// Basic API flow
ngSpice_Init(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
ngSpice_Circ(circuit_array);
ngSpice_Command("run");
ngSpice_Command("write output.raw");

// Callbacks for data retrieval
// See ngspice manual for complete API
```

### 6.4 Common ngspice Warnings/Errors

| Message | Cause | Fix |
|---------|-------|-----|
| `singular matrix` | Floating node or voltage loop | Check connectivity |
| `gmin step failed` | DC convergence failure | Add .nodeset or .ic |
| `source stepping failed` | Hard nonlinearities | Add parallel conductance |
| `timestep too small` | Transient convergence | Relax tolerances or add C |
| `warning: unrecognized parameter` | Unknown .model param | Check parameter name |

---

## 7. References

### Official Documentation

1. **ngspice User Manual**
   - URL: https://ngspice.sourceforge.io/docs.html
   - Covers: All ngspice features, commands, models
   - Format: PDF, HTML

2. **ngspice Simulator Notes**
   - URL: https://ngspice.sourceforge.io/ngspice-html-manual/manual/x15.html
   - Covers: Differences from SPICE3, implementation details

3. **SPICE3 User's Manual** (UC Berkeley)
   - Original reference for SPICE3f5
   - Covers: Core algorithms, device models

4. **SPICE2/SPICE3 Source Code**
   - Reference for algorithm details
   - Device model implementations

### Model Libraries

1. **NGSPICE Official Models**
   - Diode models: 1N4004, 1N4148, etc.
   - BJT: 2N2222, 2N3904, 2N3906, etc.
   - JFET: 2N5457, J175, etc.
   - MOSFET: Various level 1-8 models

2. **LTspice Models**
   - Generally compatible with ngspice
   - May use extended syntax

3. **PSPICE Models**
   - Require translation for ngspice
   - Different mathematical functions

### Reference Books

1. **"Semiconductor Device Modeling with SPICE"**
   - Authors: Paolo Antognetti, Giuseppe Massobrio
   - Covers: Physics behind SPICE models

2. **"The SPICE Book"**
   - Author: Andrei Vladimirescu
   - Covers: SPICE algorithms and usage

3. **"Computer Methods for Circuit Analysis and Design"**
   - Authors: J. Vlach, K. Singhal
   - Covers: MNA formulation, numerical methods

### Online Resources

- **ngspice GitHub**: https://github.com/ngspice/ngspice
- **SPICE Forum**: comp.languages.cad.spice (archived)
- **EEVblog Forum**: https://www.eevblog.com/forum/
- **KiCad/ngspice Integration**: https://docs.kicad.org/

---

## 8. Quick Reference Card

### Element Letters

| Letter | Element | Nodes | Value/Model |
|--------|---------|-------|-------------|
| R | Resistor | 2 | Resistance |
| C | Capacitor | 2 | Capacitance |
| L | Inductor | 2 | Inductance |
| V | Voltage Source | 2 | DC/AC/Transient |
| I | Current Source | 2 | DC/AC/Transient |
| D | Diode | 2 | Model name |
| Q | BJT | 3-4 | Model name |
| M | MOSFET | 4 | Model name |
| J | JFET | 3 | Model name |
| E | VCVS | 4+ | Gain |
| F | CCCS | 2+ | Vsource name, gain |
| G | VCCS | 4+ | Transconductance |
| H | CCVS | 2+ | Vsource name, transresistance |
| K | Coupled Inductor | 0 | L1, L2, coupling |
| X | Subcircuit | varies | Subckt name |
| B | Behavioral Source | 2 | Expression |

### Scale Suffixes (M vs MEG)

```
f = 1e-15      p = 1e-12      n = 1e-9
u = 1e-6       m = 1e-3       k = 1e3
meg = 1e6      g = 1e9        t = 1e12
```

### Analysis Directives

```spice
.op                  ; DC operating point
.dc Vsrc start stop step   ; DC sweep
.ac dec np fstart fstop    ; AC analysis (decade)
.ac oct np fstart fstop    ; AC analysis (octave)
.ac lin np fstart fstop    ; AC analysis (linear)
.tran tstep tstop [tstart [tmax]] [uic]
.temp val1 [val2 ...]      ; Temperature sweep
```

### Critical Constants

```python
# SPICE default temperature
TNOM = 27  # °C

# Thermal voltage at TNOM
Vt = 25.85e-3  # V (k*T/q at 300K)

# Boltzmann's constant
k = 1.3806226e-23  # J/K

# Elementary charge
q = 1.6021918e-19  # C

# Planck's constant
h = 6.62620096e-34  # J*s
```
