# Melange KiCad Integration

Use KiCad as a schematic editor for Melange circuits. This provides:

1. **Symbol library** (`melange.kicad_sym`) — custom symbols for triodes, VCAs, op-amps, pots, wipers, and audio I/O markers
2. **Python netlist exporter** (`melange_netlist_export.py`) — converts KiCad XML to Melange `.cir` format

Standard components (R, C, L, D, BJT, JFET, MOSFET) should use KiCad's built-in `Simulation_SPICE` library symbols.

## Install Symbol Library

1. Open KiCad → Preferences → Manage Symbol Libraries
2. Click the "+" button to add a new library
3. Set Nickname to `Melange` and Library Path to the full path of `melange.kicad_sym`
4. Click OK

The melange symbols will appear under the `Melange` library in the symbol chooser.

## Install Netlist Exporter (KiCad Plugin)

1. In KiCad's Schematic Editor, go to File → Export Netlist
2. Click "Add Generator"
3. Set:
   - **Title**: `Melange`
   - **Command**: `python3 /full/path/to/melange_netlist_export.py "%I" "%O"`
4. Click OK

Now "Melange" appears as a netlist format option when exporting.

## Workflow

1. Draw your circuit in KiCad using `Simulation_SPICE` symbols for standard components and `Melange` symbols for triodes, VCAs, pots, etc.
2. Set `Melange.*` fields on components that need special handling (pots, switches, gangs)
3. Export: File → Export Netlist → Melange → Export
4. Compile: `melange compile circuit.cir --output plugin_code`

## Symbols

| Symbol | Use For | Pins |
|--------|---------|------|
| `Melange_Triode` | Vacuum tube triode (12AX7, etc.) | grid, plate, cathode |
| `Melange_OpAmp` | Op-amp (JRC4558, TL072, etc.) | +, -, out |
| `Melange_VCA` | Voltage-controlled amplifier (THAT 2180, etc.) | sig+, sig-, ctrl+, ctrl- |
| `Melange_Pot` | 2-terminal potentiometer | 1, 2 |
| `Melange_Wiper` | 3-terminal potentiometer (voltage divider) | CW, wiper, CCW |
| `Melange_AudioInput` | Marks the audio input node | in |
| `Melange_AudioOutput` | Marks the audio output node | out |
| `Melange_VDC` | DC voltage source (power supply) | +, - |

## Custom Fields

Set these on component instances in the schematic to control Melange behavior:

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

## Standalone Usage

The exporter also works outside KiCad:

```bash
# Export XML from KiCad CLI
kicad-cli sch export python-bom -o circuit.xml circuit.kicad_sch

# Convert to Melange
python3 melange_netlist_export.py circuit.xml circuit.cir

# Or use melange import (if available)
melange import circuit.xml -o circuit.cir
```

## Example

A resistor marked as a pot in KiCad (with `Melange.Pot = "1k 100k"` and `Melange.Label = "Drive"`) produces:

```spice
R_drive n1 n2 50k
.pot R_drive 1k 100k "Drive"
```
