# Writing SPICE Netlists for Melange

This guide teaches you how to write a SPICE netlist from a schematic. For the complete syntax reference, see [spice-grammar.md](spice-grammar.md).

## Basics

A SPICE netlist is a text file describing a circuit. Each line is either:
- A **component** (R, C, L, D, Q, T, etc.)
- A **directive** (.model, .pot, .switch, .end, etc.)
- A **comment** (lines starting with `*` or text after `;`)

The first line is always the **title** (can be anything, not parsed as a component).

```spice
* My Circuit Title
R1 in out 10k
C1 out 0 100n
.end
```

## Nodes

Every component connects to **nodes** — named connection points in the circuit.

- **Ground** is always node `0` (you can also write `gnd`)
- **Input** is conventionally `in` (matches the default `--input-node`)
- **Output** is conventionally `out` (matches the default `--output-node`)
- Use descriptive names: `vcc`, `base`, `plate`, `cathode`, `mid`
- No spaces or special characters — letters, digits, and underscores only

## From Schematic to Netlist

### Step 1: Label every node

Look at your schematic and identify every connection point. Assign each a name. Ground is always `0`.

```
         R1 10k        C1 100n
  in ----[====]---mid---||---out
                   |
                  [D1]
                   |
                  GND
```

Nodes: `in`, `mid`, `out`, `0` (ground)

### Step 2: List every component

For each component, write: **name**, **node+**, **node-**, **value** (or model).

```spice
R1 in mid 10k
C1 mid out 100n
D1 mid 0 1N4148
```

### Step 3: Add device models

Nonlinear devices (diodes, transistors, tubes) need `.model` cards:

```spice
.model 1N4148 D(IS=2.52e-9 N=1.752)
```

### Step 4: Add power supplies

DC supplies use voltage sources:

```spice
Vcc vcc 0 DC 9
```

### Step 5: Add controls (optional)

Mark variable resistors as pots and switchable components as switches:

```spice
.pot R_vol 1k 100k "Volume"
.switch C_tone 100n 220n 470n "Tone"
```

## Component Reference

### Passive Components

```spice
R1 node1 node2 10k          ; Resistor (ohms)
C1 node1 node2 100n         ; Capacitor (farads)
L1 node1 node2 10m          ; Inductor (henries)
C2 node1 node2 10u IC=5     ; Capacitor with initial voltage
```

### Sources

```spice
Vcc vcc 0 DC 9              ; DC voltage source
Vbias base 0 DC 0.65        ; Bias voltage
Ibias 0 base DC 10u         ; DC current source (flows from n+ to n-)
```

### Semiconductor Devices

```spice
D1 anode cathode 1N4148              ; Diode: anode, cathode, model
Q1 collector base emitter 2N2222     ; BJT: C, B, E, model
J1 drain gate source J2N5457         ; JFET: D, G, S, model
M1 drain gate source bulk NMOS1      ; MOSFET: D, G, S, B, model
T1 grid plate cathode 12AX7         ; Triode tube: G, P, K, model
```

### Active Devices

```spice
U1 in_p in_n out LM358              ; Op-amp: +in, -in, out, model
Y1 sig_p sig_n ctrl_p ctrl_n VCA1   ; VCA: sig+, sig-, ctrl+, ctrl-, model
```

### Dependent Sources and Coupling

```spice
E1 out 0 in 0 10.0                  ; VCVS: gain = 10
G1 out 0 in 0 0.001                 ; VCCS: transconductance = 1mS
K1 L1 L2 0.99                       ; Coupled inductors (transformer)
```

### Subcircuits

```spice
.subckt GAIN_STAGE in out vcc
R1 in base 100k
Q1 coll base emit 2N2222
.ends GAIN_STAGE

X1 input mid vcc GAIN_STAGE         ; Instance: nodes..., subckt name
```

## Unit Suffixes

| Suffix | Multiplier | Example | Value |
|--------|-----------|---------|-------|
| `f` | 10^-15 | `10f` | 10 femto |
| `p` | 10^-12 | `100p` | 100 pico |
| `n` | 10^-9 | `10n` | 10 nano |
| `u` | 10^-6 | `1u` | 1 micro |
| `m` | 10^-3 | `1m` | 1 milli |
| `k` | 10^3 | `10k` | 10 kilo |
| `meg` | 10^6 | `1meg` | 1 mega |
| `g` | 10^9 | `1g` | 1 giga |

**Warning:** `M` alone means **milli** (10^-3), not mega. Always use `meg` for megaohms.

```spice
R1 in out 1meg    ; 1 MΩ (correct)
R2 in out 1M      ; 1 mΩ (probably wrong!)
```

## Worked Example: Fender-Style Tone Stack

Here's a complete netlist for a passive tone stack (the kind found in guitar amps):

```
             R1 100k
  in ----[========]----+----mid----out
                       |
               C1 250p |  C2 22n
               ---||---+---||---+
                       |        |
                      R2       R3
                     250k      25k
                       |        |
                      GND      GND
```

```spice
* Passive Tone Stack (Fender style)
R1 in mid 100k
C1 in mid 250p
C2 mid tone_out 22n
R2 mid 0 250k
R3 tone_out 0 25k
Rout tone_out out 1k

* Make R2 a pot (treble control)
.pot R2 1k 250k "Treble"

.end
```

Test it:
```bash
melange nodes tone-stack.cir
melange simulate tone-stack.cir --amplitude 0.1 -o test.wav
melange analyze tone-stack.cir --pot R2=1k     # treble fully cut
melange analyze tone-stack.cir --pot R2=250k   # treble fully boosted
```

## Common Mistakes

### 1. Floating nodes (no DC path to ground)

Every node must have a DC path to ground. Capacitors block DC, so a node connected only through capacitors is "floating."

**Bad** — node `mid` has no DC path:
```spice
C1 in mid 100n
C2 mid out 100n
```

**Fix** — add a bias resistor:
```spice
C1 in mid 100n
C2 mid out 100n
Rbias mid 0 1meg   ; high value so it doesn't affect AC
```

### 2. Wrong terminal order

Each device type has a specific terminal order. Getting it wrong flips the device behavior.

| Device | Order | Mnemonic |
|--------|-------|----------|
| Diode | anode, cathode | current flows A→K |
| BJT | collector, base, emitter | CBE |
| JFET | drain, gate, source | DGS |
| MOSFET | drain, gate, source, bulk | DGSB |
| Triode | grid, plate, cathode | GPC |
| Op-amp | +in, -in, out | plus, minus, out |
| VCA | sig+, sig-, ctrl+, ctrl- | signal then control |

### 3. Missing .model card

Every D, Q, J, M, T, U, and Y element references a model by name. That model must be defined:

```spice
D1 in out MyDiode
; ERROR: model 'MyDiode' not found

.model MyDiode D(IS=1e-14 N=1.0)    ; add this
```

### 4. Forgetting ground connections

Power supplies and bias networks need a ground reference:

**Bad:**
```spice
Vcc vcc vee 9    ; no ground — both nodes float
```

**Fix:**
```spice
Vcc vcc 0 9      ; vcc is 9V above ground
```

### 5. Self-connected components

A component connected to the same node on both terminals is rejected:

```spice
R1 node1 node1 10k   ; ERROR: self-connected
```

### 6. Negative or zero values

Component values must be positive and finite. Zero, negative, NaN, and infinity are rejected:

```spice
R1 in out 0       ; ERROR: must be positive
C1 in out -10n    ; ERROR: must be positive
```

## Validation Workflow

Before compiling to a plugin, validate your netlist:

```bash
# 1. Check that it parses correctly and inspect the circuit
melange nodes my-circuit.cir

# 2. Quick audio test with a sine wave
melange simulate my-circuit.cir --amplitude 0.1 -o test.wav

# 3. Check frequency response
melange analyze my-circuit.cir

# 4. Compare against ngspice (if installed)
melange validate my-circuit.cir

# 5. Compile to plugin
melange compile my-circuit.cir --format plugin -o my-plugin
```

## Tips

- **Start simple.** Get a basic circuit working, then add complexity.
- **Name nodes descriptively.** `plate1`, `grid2`, `bias_node` are easier to debug than `1`, `2`, `3`.
- **Check terminal order.** The most common mistake is swapping BJT collector/emitter or diode anode/cathode.
- **Use `melange analyze`** to see the frequency response before compiling. It's fast and catches many issues.
- **Use `melange simulate --amplitude 0.1`** as a quick sanity check — if you hear the circuit working, the netlist is correct.
- **Model parameters matter.** A BJT with default `IS=1e-16` behaves very differently from one with `IS=1e-14`. Use datasheet values or known SPICE models.

## Further Reading

- [SPICE Grammar Reference](spice-grammar.md) — complete syntax for all elements and directives
- [Getting Started](GETTING_STARTED.md) — from netlist to working plugin
- [Plugin Development Guide](PLUGIN_GUIDE.md) — customizing generated plugins
