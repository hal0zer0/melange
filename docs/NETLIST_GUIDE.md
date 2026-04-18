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

`.switch` is also the right tool for **discrete component selection** — pedal
bypass, channel select, clipping-diode picker, bright switch, etc. List one
or more components and the values they take in each position:

```spice
* Bright switch: either 0 (off) or a small coupling cap in parallel
.switch C_bright 0 470p "Bright"

* Clipping diode selector (5 positions, ganged across 5 R_sel resistors)
.switch R_sel_ge,R_sel_si,R_sel_led 1/10Meg/10Meg 10Meg/1/10Meg 10Meg/10Meg/1 "Clip"
```

Position 0 is always the initial state; the G/C matrix is stamped at the
pos-0 values so the circuit boots into a consistent starting point.

For 3-terminal wiper pots (top, wiper, bottom), use two resistors and `.wiper`:

```spice
R_cw top wiper 50k
R_ccw wiper bottom 50k
.wiper R_cw R_ccw 100k "Volume"
```

Link multiple pots or wipers to a single knob with `.gang`:

```spice
.gang "Tone" R_treble R_bass
```

For **host-driven per-sample voltage injection** (sidechain CV, LFO,
envelope follower, any modulation input), declare a voltage source with
`DC 0` and bind it to a `CircuitState` field with `.runtime`:

```spice
Vctrl ctrl 0 DC 0
R_ctrl ctrl vca_gain 10k
.runtime Vctrl as ctrl_voltage
```

Codegen emits `pub ctrl_voltage: f64` on `CircuitState`; the plugin writes
it each sample and melange stamps the value into the VS's RHS row
automatically. Any DC value you declare on the voltage source is preserved
as a bias, so `V1 n1 0 DC 5` + `.runtime V1 as foo` gives `5 + foo` total.

See [spice-grammar.md](spice-grammar.md#5-melange-extensions) for full syntax.

## Component Reference

### Passive Components

```spice
R1 node1 node2 10k          ; Resistor (ohms)
C1 node1 node2 100n         ; Capacitor (farads)
L1 node1 node2 10m          ; Inductor (henries)
L2 node1 node2 100m ISAT=20m ; Saturating inductor (tanh model)
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
P1 plate grid cathode screen EL84_P  ; Pentode: P, G, K, Screen, model
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
| Pentode | plate, grid, cathode, screen | PGKS (optional suppressor after screen) |
| Op-amp | +in, -in, out | plus, minus, out |
| VCA | sig+, sig-, ctrl+, ctrl- | signal then control |

### 3. Missing .model card

Every D, Q, J, M, T, P, U, and Y element references a model by name. That model must be defined:

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
- **Decompose large circuits when there's no global feedback.** If your circuit has no feedback path between subsystems — e.g. a preamp → tone-stack → power-amp cascade where each stage drives the next through a coupling cap and nothing feeds back — compile each subsystem as a separate `.cir` and chain them in plugin code. This keeps N and M small per kernel (linear in DK cost, cubic in nodal), avoids cross-subsystem matrix conditioning issues, and lets each stage pick its best solver path independently. A 16-stage tube cascade with real global feedback has to be one monolithic netlist; an 8-stage preamp where each stage is capacitively coupled to the next does not.
- **Use `.linearize` for semantic control, not CPU savings.** NR converges in 0–1 iterations for devices in their small-signal region, so linearizing produces negligible speedup. The real use case is **forcing a device to stay small-signal** — e.g. a Vbe multiplier that must not clip, a preamp stage that should be clean even at extreme input. Linearized devices cannot clip because they're replaced with small-signal conductances at the DC operating point.
- **Per-instance parameter jitter needs warmup.** If the plugin sets `state.pot_N_resistance = jittered_value` at construction, loop `process_sample(0.0, &mut state)` `WARMUP_SAMPLES_RECOMMENDED` times before processing audio — the DC_OP constant is baked at nominal values, so jittered circuits need time to settle to their actual equilibrium.
- **Skip warmup with `recompute_dc_op()` / `settle_dc_op()`.** Pass `--emit-dc-op-recompute` to `melange compile` and the generated `CircuitState` gains two methods. `recompute_dc_op(&mut self)` runs the runtime NR directly — use it when you want explicit control. `settle_dc_op(&mut self)` is the convenience wrapper: it calls `recompute_dc_op` first and falls back to the `WARMUP_SAMPLES_RECOMMENDED` silence loop if the NR fails or the circuit is nodal-routed. Prefer `settle_dc_op` unless you need to observe the recompute-vs-fallback decision yourself — it handles the DK-vs-nodal distinction uniformly so plugin code doesn't need to branch. Neither method is audio-thread safe; call from plugin init or parameter-change callbacks. DK-path circuits (tube preamps, op-amp clippers) get the full NR; nodal full-LU circuits (pultec, 4kbuscomp, VCR ALC, wurli power amp) always fall through to warmup today (the nodal NR body is deferred indefinitely; warmup remains the documented path for nodal plugins). See `docs/aidocs/DC_OP.md` "Runtime DC OP recompute" for the full contract.
- **Runtime-adjustable device model fields** are already emitted as `pub` on `CircuitState`: `device_0_mu`, `device_0_ex`, `device_0_kg1`, etc. for tubes; similar for BJTs, JFETs, MOSFETs. Write them directly from plugin code for per-instance tube aging, transistor matching, or user-exposed model parameters. No codegen changes needed.

## Further Reading

- [SPICE Grammar Reference](spice-grammar.md) — complete syntax for all elements and directives
- [Getting Started](GETTING_STARTED.md) — from netlist to working plugin
- [Plugin Development Guide](PLUGIN_GUIDE.md) — customizing generated plugins
