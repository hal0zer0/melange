# Symbol Reference

## Melange Library Symbols

These require the `melange.kicad_sym` library installed in KiCad.

| Symbol | Use For | Pins | Ref Prefix |
|--------|---------|------|------------|
| `Melange_Triode` | Vacuum tube triode (12AX7, etc.) | G, P, K | `T` |
| `Melange_Pentode` | Vacuum tube pentode (EL84, 6V6, etc.) | G, P, K, G2, G3 | `P` |
| `Melange_OpAmp` | Op-amp (JRC4558, TL072, etc.) | +, -, OUT | `U` |
| `Melange_VCA` | Voltage-controlled amplifier (THAT 2180, etc.) | S+, S-, C+, C- | `Y` |
| `Melange_Pot` | 2-terminal potentiometer | 1, 2 | `R` |
| `Melange_Wiper` | 3-terminal potentiometer (voltage divider) | CW, W, CCW | `RW` |
| `Melange_AudioInput` | Audio input node marker | in | `#AIN` |
| `Melange_AudioOutput` | Audio output node marker | out | `#AOUT` |
| `Melange_VDC` | DC voltage source (power supply) | +, - | `V` |

## Standard KiCad Symbols (from `Simulation_SPICE`)

Use KiCad's built-in library for these — no Melange symbol needed.

| Symbol | Use For | Ref Prefix |
|--------|---------|------------|
| `R` | Resistor | `R` |
| `C` | Capacitor | `C` |
| `L` | Inductor | `L` |
| `D` | Diode / LED | `D` |
| `Q` / `NPN` / `PNP` | BJT transistor | `Q` |
| `NJFET` / `PJFET` | JFET | `J` |
| `NMOS` / `PMOS` | MOSFET | `M` |
| `V` / `I` | Voltage/current source | `V` / `I` |

## Transformers

Transformers use standard `L` (inductor) symbols plus `K` coupling statements. Each winding is a separate inductor; coupling is specified via a `K` element in the netlist:

```spice
L1 primary_p primary_n 10m
L2 secondary_p secondary_n 40m
K1 L1 L2 0.95
```

Add `K` coupling lines manually to the exported `.cir`, or use a text component in the schematic.

## Custom Fields

Set these on component instances in the schematic properties to control Melange behavior:

| Field | On | Purpose | Example |
|-------|----|---------|---------|
| `Melange.Pot` | Resistor | Mark as pot with min/max range | `1k 100k` or `1k 100k 50k` |
| `Melange.Wiper` | Wiper symbol | Total resistance + default position | `100k 0.5` |
| `Melange.Switch` | R/C/L | Switchable component values | `100n 220n 470n` |
| `Melange.Gang` | Pot/Wiper | Gang group label (shared parameter) | `Gain` |
| `Melange.GangInvert` | Pot/Wiper | Invert this gang member | `true` |
| `Melange.Label` | Any control | Human-readable name for plugin UI | `Drive` |
| `Melange.Model` | Device | Model directive parameters | `OA(AOL=200000 ROUT=75)` |
| `Melange.Input` | AudioInput | Marks audio input node | `true` |
| `Melange.Output` | AudioOutput | Marks audio output node | `true` |

## Pentode Models

The pentode symbol uses the `VP(...)` model type. Common tubes:

```spice
* EL84 (Reefman Derk rational screen)
.model EL84 VP(MU=23.36 EX=1.138 KG1=117.4 KG2=1275 KP=152.4 KVB=4015.8
+              ALPHA_S=7.66 A_FACTOR=4.344e-4 BETA_FACTOR=0.148)

* 6V6GT (beam tetrode, exponential screen) — catalog lookup
.model 6V6GT VP()

* KT88 (classical Koren) — catalog lookup
.model KT88 VP()
```

Empty `VP()` uses built-in catalog values (29 tubes available). Set parameters in the `Melange.Model` field on pentode symbols.

Pin 5 (suppressor/G3) is optional — leave unconnected for suppressor-tied-to-cathode (most power pentodes).
