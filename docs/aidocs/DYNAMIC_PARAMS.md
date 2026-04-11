# Dynamic Parameters: Pots, Wipers, Gangs, Switches

Melange supports four netlist directives for runtime-variable circuit
elements. All four are codegen-emitted into the generated `CircuitState`
as parameters that can be changed between samples without re-running the
DK kernel build.

## Source Files

| Directive | Parser | MNA struct | Codegen IR |
|-----------|--------|-----------|------------|
| `.pot`    | `crates/melange-solver/src/parser.rs` (`parse_pot_directive`) | `PotInfo` in `mna.rs` | `PotentiometerIR` in `codegen/ir.rs` |
| `.wiper`  | `parser.rs` (`parse_wiper_directive`) | `WiperGroupInfo` in `mna.rs` | `WiperGroupIR` in `codegen/ir.rs` |
| `.gang`   | `parser.rs` (`parse_gang_directive`) | `GangGroupInfo` in `mna.rs` | `GangGroupIR` in `codegen/ir.rs` |
| `.switch` | `parser.rs` (`parse_switch_directive`) | `SwitchInfo` in `mna.rs` | (rebuilt at codegen time) |

Plugin generation: `tools/melange-cli/src/plugin_template.rs` emits one
`FloatParam` per `.pot`, one `FloatParam` per `.wiper`, one `FloatParam`
per `.gang`, and one `IntParam` per `.switch`.

## `.pot` — Single Resistor Variable

### Syntax
```spice
R1 node_a node_b 10k         ; declared resistor
.pot R1 1k 100k              ; range [1k, 100k]
.pot R2 500 500k Volume      ; optional label for plugin UI
.pot R3 1k 100k 47k Tone     ; optional default value (47k) before label
```

### Semantics
The named resistor becomes a runtime variable. The MNA system stamps the
nominal conductance (`1/R_nominal` from the netlist value, or the explicit
default if given). At runtime, changing `pot_resistance` triggers a full
per-block rebuild of S, K, A_neg, and the S·N_i product via dense
`invert_n`. The plugin template reads `.smoothed.next()` per sample
inside the process loop, so rebuilds fire at the smoother's staircase
rate (bounded by the 10 ms smoother time constant), not per audio sample.

After a large pot jump (`|r - r_prev| / r_prev > 0.20`), `set_pot_N`
also resets `v_prev` to `DC_OP` and `i_nl_prev` to `DC_NL_I` so the NR
loop starts the next sample from a bias-consistent seed instead of the
stale previous-sample state. This is gated on delta magnitude so that
smoothed knob sweeps (per-sample deltas ≪ 0.2%) do not trigger
mid-signal state resets. See `set_pot_N` emission in
`rust_emitter.rs` (DK Schur + nodal paths).

Sherman-Morrison rank-1 updates were removed in 2026-04-04 (commit
`eaee955`). The removal was originally motivated by NR max-iter-cap
hits on wide-range pot jumps, but subsequent research showed that SM
is mathematically exact (same K' as full rebuild, to 1e-9 float noise)
— the real root cause was DC-OP seed staleness, fixed by the warm
re-init described above. `SHERMAN_MORRISON.md` remains as math
reference; the SM precomputation (`su`, `usu`, `nv_su`, `u_ni`) still
runs at codegen time but is currently unused by the Tera templates.

### Constraints
- Maximum 32 combined `.pot` + `.wiper` legs per circuit (each `.wiper`
  counts as 2 legs).
- The named resistor must already exist in the netlist before the `.pot`
  directive.
- A resistor can only be claimed by one `.pot` or one `.wiper` (not both).

## `.wiper` — Two-Leg Potentiometer

### Syntax
```spice
R_top  in    wiper 50k       ; CW leg (top → wiper)
R_bot  wiper 0     50k       ; CCW leg (wiper → bottom)
.wiper R_top R_bot 100k                          ; total resistance 100k
.wiper R_top R_bot 100k 0.5 "Volume"             ; default position + label
```

### Semantics
A wiper potentiometer is the physically correct three-terminal model: a
single 100k pot with a wiper that splits the resistance into two legs
that always sum to the total. A single UI parameter (position 0.0–1.0)
controls both legs simultaneously:
```
R_cw  = pos       * (total - 2) + 1
R_ccw = (1 - pos) * (total - 2) + 1
```
The `+1`/`-2` keep both legs > 0 even at the extremes (avoids singular
matrices when pos = 0 or 1). At runtime, two sequential Sherman-Morrison
rank-1 updates are applied, one per leg, with cross-correction so the
second update accounts for the first.

### When to use `.wiper` vs `.pot`
- **`.pot`** if the circuit only uses two terminals of the pot (rheostat
  configuration). Cheaper — one rank-1 update.
- **`.wiper`** if the circuit uses all three terminals (true voltage
  divider). Two rank-1 updates, but physically correct.

Most volume/tone controls in audio circuits are `.wiper` candidates.

### Constraints
- CW and CCW legs must be different resistors.
- Total resistance must be > 20 Ω.
- Each `.wiper` consumes 2 of the 32 combined pot/wiper slots.

## `.gang` — Multi-Member Single Parameter

### Syntax
```spice
R_left  in_l out_l 100k
R_right in_r out_r 100k
.pot R_left  10k 1Meg
.pot R_right 10k 1Meg
.gang "Stereo Volume" R_left R_right             ; both pots, same direction
.gang "Klon Gain" R_gain_a !R_gain_b 0.5         ; one inverted, default 0.5
```

### Semantics
A `.gang` directive links multiple `.pot` or `.wiper` members under a
single UI parameter. The label becomes the plugin parameter name; the
position (0.0–1.0) drives all members in lock-step.

The `!` prefix on a member name **inverts** that member's response — when
the gang position is at 1.0, the inverted member sees 0.0, and vice
versa. This is how dual-gang pots with reverse-log tapers (Klon "Gain"
control, Pultec dual-section EQ) are modeled: one section sweeps
clean→clipped while the gang-mate sweeps clipped→clean to crossfade
between paths.

### Constraints
- At least 2 members required.
- All members must already be declared as `.pot` or `.wiper` before the
  `.gang` references them.
- Members can mix `.pot` and `.wiper` types in the same gang.
- Member names referenced as resistors are uppercased internally.
- Pot members in a gang are excluded from the per-pot UI parameters
  (the gang parameter replaces them); same for wiper members.

## `.switch` — Multi-Position Component Selector

### Syntax
```spice
C1 hp_in 0 100n
.switch C1 100n 220n 470n               ; one component, three positions
.switch C1,C2 100n/10n 220n/22n 470n/47n  ; two components, three positions
```

### Semantics
A `.switch` directive declares one or more components (R, C, or L) whose
values change in lock-step across discrete positions. Position 0 is the
default. Changing positions at runtime triggers a partial matrix rebuild
(more expensive than a Sherman-Morrison pot update — the affected G/C
entries are restamped and downstream matrices recomputed).

### Component value syntax
- **One component**: `.switch C1 v0 v1 v2 v3 ...` — each value applies to C1.
- **Multiple components**: `.switch C1,C2 v0a/v0b v1a/v1b v2a/v2b ...` —
  each position is a slash-separated tuple where the order matches the
  component name list.

### Constraints
- Component names must start with `R`, `C`, or `L` (resistor, capacitor,
  inductor).
- All positions must specify a value for every component in the list.
- Maximum 16 switches per circuit.
- For subcircuit-expanded names (e.g. `X1.C1`), the prefix-validation
  checks the part after the last dot.

### When to use `.switch` vs `.pot`
- **`.pot`** for continuously variable resistors (volume, gain, EQ
  frequency knobs).
- **`.switch`** for discrete-position component selectors (impedance
  switches, EQ shape selectors, oscilloscope-style range switches,
  Pultec frequency selectors). Switches change topology values, not
  just one conductance, so they need a real rebuild rather than a
  rank-1 update.

## Plugin Code Generation

`tools/melange-cli/src/plugin_template.rs` emits parameters as follows:

| Directive | Plugin parameter type | Range | Display |
|-----------|----------------------|-------|---------|
| `.pot` | `FloatParam` | 0.0–1.0 (linear) | percentage |
| `.wiper` | `FloatParam` | 0.0–1.0 (linear) | (no unit by default) |
| `.gang` | `FloatParam` | 0.0–1.0 (linear) | (label-driven) |
| `.switch` | `IntParam` | `0..num_positions-1` | integer |

Pot/wiper/gang positions are smoothed via `SmoothingStyle::Linear(10ms)`
and read **per sample** via `.smoothed.next()` inside the generated
`process()` loop. The `set_pot_N` / `set_wiper_N` / gang setter skip
guard (`(r - prev).abs() < 1e-12`) means steady-state samples pay only
a float subtract + abs + compare; actual matrix rebuilds fire only
when the smoothed value crosses that threshold.

Switches are not smoothed — flipping a switch is expected to be a
discrete event. `set_switch_N` also applies an unconditional DC-OP
re-init (no gate, since every switch change is a step).

## Common Pitfalls

| Symptom | Cause | Fix |
|---------|-------|-----|
| Pot has no audible effect | `R_nom` mismatch (nominal value in netlist ≠ midpoint of `.pot` range) | Make sure the netlist value is inside the `.pot` range, ideally near the geometric mean of min and max |
| Wiper goes silent at extremes | Total resistance < 20 Ω | Increase the wiper total |
| Gang member responds backward | Direction expected but not inverted | Add `!` prefix on the gang-member reference |
| Switch position N produces wrong tone | Component value list misaligned with name list | Re-check `.switch C1,C2` slash order vs name order |
| "Resistor already claimed" error | Same resistor in two `.pot`/`.wiper` directives | A resistor can only be in one |
| > 32 pots error | Combined `.pot` + 2×`.wiper` count > 32 | Reduce or simplify |
