# Import Workflow

Three ways to get a KiCad schematic into Melange.

## 1. KiCad Netlist Export Plugin (recommended)

Uses the Python exporter registered as a KiCad netlist generator. Full fidelity ��� preserves all `Melange.*` custom fields.

### Setup

1. In KiCad Schematic Editor: File → Export Netlist
2. Click "Add Generator"
3. Set **Title** to `Melange`, **Command** to:
   ```
   python3 /full/path/to/melange_netlist_export.py "%I" "%O"
   ```
4. Click OK

### Usage

File → Export Netlist → Melange → Export → choose output `.cir` path.

## 2. Direct Schematic Import (`melange import`)

With KiCad 8+ installed, import `.kicad_sch` files directly:

```bash
melange import circuit.kicad_sch -o circuit.cir
```

Auto-detects the `.kicad_sch` extension and shells out to `kicad-cli`. Explicit flag also available:

```bash
melange import --from-schematic circuit.kicad_sch -o circuit.cir
```

Requires `kicad-cli` on your PATH (bundled with KiCad 8+).

## 3. Manual XML Export

Export XML first, then import:

```bash
kicad-cli sch export python-bom -o circuit.xml circuit.kicad_sch
melange import circuit.xml -o circuit.cir
```

## SPICE Import (best-effort)

`melange import` also accepts KiCad SPICE netlists. This strips simulation directives (`.tran`, `.ac`, etc.) and sanitizes `GND` → `0`. Melange-specific directives (`.pot`, `.switch`, `.gang`) must be added manually.

```bash
melange import circuit.spice -o circuit.cir --format spice
```

## Validation

All import paths automatically validate the generated `.cir` through melange's parser. Parse errors are reported as warnings — the output file is always written so you can fix issues by hand.

## After Import

```bash
# Compile to standalone Rust code
melange compile circuit.cir --output circuit.rs

# Or compile to a full plugin project
melange compile circuit.cir --output ~/dev/my-plugin --format plugin
```
