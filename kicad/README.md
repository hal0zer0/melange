# Melange KiCad Integration

Use KiCad as a schematic editor for Melange circuits.

## How much of this has actually been run

Read this before budgeting time on it. The KiCad path did not work at all until
2026-09-23: the bundled symbol library and example schematic had never been
opened by KiCad and did not load (stray `;` comments, a missing
`sheet_instances` block, an invented `net_label` token). They load now, and the
`rc-lowpass` example below has been taken end to end — KiCad 10.0.6 export →
`melange import` → a deck that analyzes to the expected 159 Hz corner.

What that run covered: **stock `Simulation_SPICE` R and C parts, wire labels,
and a `power:GND` symbol.** That is all.

Not yet exercised in a real schematic: every melange-specific symbol — triode,
pentode, op-amp, VCA, pot, wiper, VDC, and the `Melange_AudioInput` /
`Melange_AudioOutput` markers — and every `Melange.*` field (`Melange.Pot`,
`Melange.Wiper`, `Melange.Switch`, `Melange.Gang`, `Melange.Model`). The symbols
render correctly in KiCad 10.0.6 (`kicad-cli sym export svg` plots all nine), but
no one has yet placed them on a sheet, exported, imported and simulated the
result, and the importer has no tests over the `Melange.*` field handling either.
Expect to find bugs, and report them rather than working around them.

## Requirements

**KiCad 8 or newer.** This is a hard gate, not a recommendation. Both
`melange.kicad_sym` and the bundled example schematic are saved in the KiCad 8
file format (`version 20231120`), which KiCad 7 cannot read. On KiCad 7.0.11,
`melange import` on a `.kicad_sch` — and bare `kicad-cli sch export` on the same
file — fails with:

```
Failed to load schematic file
```

That message is KiCad's response to *any* load failure. It names neither the
tool nor the version nor the offending construct, so a too-old KiCad and a
genuinely malformed file are indistinguishable from the message alone — and the
files in this directory were malformed until recently, so both causes have
really happened here. Check `kicad-cli --version` first; if it is 8 or newer and
the message persists, suspect the file.

The CLI import path also needs `kicad-cli` on your `PATH`; it ships with KiCad 8.
The XML path (`melange import circuit.xml`) has no KiCad requirement at all once
you have the XML, so an `.xml` exported on a KiCad 8 machine imports anywhere.

## Try it without KiCad

`examples/rc-lowpass/rc-lowpass.xml` is real Eeschema 10.0.6 output, committed
so this path can be run (and tested in CI) on a machine with no KiCad installed:

```bash
melange import kicad/examples/rc-lowpass/rc-lowpass.xml -o rc.cir
melange analyze rc.cir -i in -o response.txt      # -3 dB at ~159 Hz
```

The import prints:

```
melange import (KiCad XML → Melange .cir)
  Source: kicad/examples/rc-lowpass/rc-lowpass.xml
  Components: 3
  Models: 0
  Pots: 0
  Wipers: 0
  Switches: 0
  Gangs: 0
  Validation: OK (netlist parses cleanly)
  Output: rc.cir
```

and writes:

```spice
rc lowpass

* --- Circuit ---
C1 net__c1_pad1 0 0.1u
R1 in net__c1_pad1 10k
R2 net__c1_pad1 out 10k

.END
```

which is the same circuit as the hand-written
`examples/rc-lowpass/rc-lowpass-reference.cir` that ships beside it. `net__c1_pad1`
is KiCad's own name for a net the schematic does not name; it is ugly and it is
correct, and the importer does not guess a prettier one.

## Ground and power symbols

Power symbols never appear in KiCad's exported component list — their references
start with `#` — so melange sees them only through the net names they create.

| Symbol | Net name KiCad emits | Melange node |
|---|---|---|
| `power:GND` | `GND` | `0` (the reference node) |
| a wire label `gnd` / `ground` / `0` | `/gnd`, … | `0` |
| `power:VCC` | `VCC` | `vcc` |
| `power:+15V` | `+15V` | `p15v` |
| `power:-15V` | `-15V` | `n15v` |
| `power:AGND`, `power:GNDREF`, … | `AGND`, `GNDREF` | `agnd`, `gndref` |

Notes on the choices:

- **Supply rails stay as named nodes.** `VCC` and `+15V` are nets the circuit
  still has to drive; they are neither ground nor noise. Importing them as nodes
  leaves you something to hang a `V` source on. Melange does not invent that
  source — add it in the schematic (`Melange_VDC`) or in the generated deck.
- **The `+`/`-` on a rail is carried across as `p`/`n`.** Under a plain
  strip-the-punctuation rule `+15V` and `-15V` both become `15v`, and a dual
  supply imports with its rails shorted together. Anything that still collides
  after this is a hard error, not a warning: two KiCad nets folding onto one
  melange node would silently rewire the circuit, so the import refuses and
  names both nets.
- **Analog/digital grounds are NOT folded into `0`.** A schematic that draws both
  `GND` and `AGND` has drawn two nets on purpose. Tie them together explicitly if
  that is what you meant.
- **A schematic with no ground symbol gets no ground.** Melange will not guess
  one. You get the existing warning that no element references node `0`, which is
  the correct outcome: an ungrounded deck's node voltages are defined only up to
  an arbitrary offset, and ngspice would reject it.

If a power symbol's library definition is missing the `(power)` token, KiCad does
not turn it into a named global net at all — it falls back to an auto-generated
`Net-(#PWR01-GND)`, and a ground symbol silently stops being a ground. Melange's
own example shipped in exactly that state. The importer now warns by name when it
sees that net shape.

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

(That mapping is what the importer's code says it does; it has not been
round-tripped through a real schematic or covered by a test — see the scope note
at the top.)
