# Melange KiCad Integration

Use KiCad as a schematic editor for Melange circuits.

## Requirements

**KiCad 8 or newer.** This is a hard gate, not a recommendation. Both
`melange.kicad_sym` and the bundled example schematic are saved in the KiCad 8
file format (`version 20231120`), which KiCad 7 cannot read. On KiCad 7.0.11,
`melange import` on a `.kicad_sch` — and bare `kicad-cli sch export` on the same
file — fails with:

```
Failed to load schematic file
```

That message names neither the tool nor the version, so it reads like a corrupt
schematic when it is really a too-old KiCad. Check `kicad-cli --version` first.

The CLI import path also needs `kicad-cli` on your `PATH`; it ships with KiCad 8.
The XML path (`melange import circuit.xml`) has no KiCad requirement at all once
you have the XML, so an `.xml` exported on a KiCad 8 machine imports anywhere.

## Quick Start

1. **Install the symbol library**: KiCad → Preferences → Manage Symbol Libraries → add `melange.kicad_sym`
2. **Draw your circuit** using `Simulation_SPICE` symbols for R/C/L/D/Q/J/M and `Melange` symbols for triodes, pentodes, op-amps, VCAs, pots, wipers, and I/O markers
3. **Set `Melange.*` fields** on components that need special handling (pots, switches, gangs, models)
4. **Export**: File → Export Netlist → Melange → Export (requires the [netlist exporter plugin](docs/import.md))
5. **Compile**: `melange compile circuit.cir --format plugin -o my-plugin`

Or import directly from the CLI:

```bash
melange import circuit.kicad_sch -o circuit.cir    # requires kicad-cli (KiCad 8+)
melange import circuit.xml -o circuit.cir          # from exported XML, no KiCad needed
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
