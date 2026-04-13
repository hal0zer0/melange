# Melange KiCad Integration

Use KiCad as a schematic editor for Melange circuits.

## Quick Start

1. **Install the symbol library**: KiCad → Preferences → Manage Symbol Libraries → add `melange.kicad_sym`
2. **Draw your circuit** using `Simulation_SPICE` symbols for R/C/L/D/Q/J/M and `Melange` symbols for triodes, pentodes, op-amps, VCAs, pots, wipers, and I/O markers
3. **Set `Melange.*` fields** on components that need special handling (pots, switches, gangs, models)
4. **Export**: File → Export Netlist → Melange → Export (requires the [netlist exporter plugin](docs/import.md))
5. **Compile**: `melange compile circuit.cir --output plugin_code`

Or import directly from the CLI:

```bash
melange import circuit.kicad_sch -o circuit.cir    # requires kicad-cli
melange import circuit.xml -o circuit.cir           # from exported XML
```

## Documentation

- [Symbol Reference](docs/symbols.md) — all symbols, pin mappings, custom fields, pentode models
- [Import Workflow](docs/import.md) — netlist plugin setup, CLI import, SPICE import, validation

## Example

See [`examples/rc-lowpass/`](examples/rc-lowpass/) for a minimal KiCad project with reference `.cir` output.

A resistor marked as a pot:

```
Melange.Pot = "1k 100k"
Melange.Label = "Drive"
```

produces:

```spice
R_drive n1 n2 50k
.pot R_drive 1k 100k "Drive"
```
