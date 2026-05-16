# Triode + pentode shot-noise stamps injected at wrong nodes

**From:** melange-circuits agent (filing on behalf of the
oomox/Noyce plugin team — they surfaced the symptom; cross-repo
investigation localized the bug)
**To:** melange engine team
**Date:** 2026-05-14
**Severity:** correctness — any tube circuit compiled with
`--noise shot` or `--noise full` over-injects shot at the wrong
nodes. Audio-signal-path circuits hide the symptom; noise-emitter
circuits (Noyce) surface it immediately. Bug ships in every
compiled artifact from the day Phase 2 landed.
**Scope:** triodes (`T`) and pentodes (`P`) only. Diodes, BJTs,
JFETs, MOSFETs verified correct.

## TL;DR

In `crates/melange-solver/src/codegen/ir.rs:2089-2104`,
`collect_shot_noise_sources` selects the wrong pair of MNA node
indices for the tube shot-noise stamp. It hard-codes
`node_i = nodes[0]` and `node_j = nodes.last()`, with a comment
claiming the triode node ordering is `[plate, grid, cathode]`.

The actual ordering produced by `mna.rs:4224-4232` is
**`[grid, plate, cathode]`** for triodes. So `nodes[0]` is the
*grid* and the shot stamp lands at `(grid, cathode)` instead of
the correct `(plate, cathode)`. NOISE.md §"Shot (Junction) Noise"
spec is unambiguous: "Tube: inject at (plate, cathode) for Ip".

Pentodes have a similar but distinct mis-pick: the order *is*
`[plate, grid, cathode, screen, [suppressor]]` (per mna.rs:4232-),
so `nodes[0]` = plate is correct, but `nodes.last()` is screen
(or suppressor) instead of cathode. The pentode shot ends up
between `(plate, screen)` or `(plate, suppressor)`.

Two-line fix in `ir.rs` (proposed below). The matching test, also
proposed below, asserts each shot pair against the device's `i_nl`
port (where the controlled current physically flows) rather than
its `v_nl` port.

## How the symptom looks

Reported in `~/dev/oomox/docs/specs/noyce-triode-12ax7-measurements.md`,
on a single 12AX7 common-cathode stage at Ip ≈ 0.69 mA:

- Shot-only output variance is ~10⁹× larger than thermal-only output
  variance, after isolating them via `set_thermal_gain(0)` and
  `set_shot_gain(0)`.
- Output variance is bit-identical across T ∈ {3, 30, 290, 500, 1500} K.
  Thermal *should* lift the hot end and crush the cold end; the shot
  floor swamps the thermal contribution by so much that T-scaling
  has no audible effect.
- Welch PSD at the output is heavily shaped — consistent with shot
  current entering the high-impedance grid node and being amplified
  by the full tube gain, rather than entering the lower-impedance
  plate node and being shaped only by Z_plate.

The plugin team's hand calc predicted shot σ_out ≈ 0.5 mV at unity
noise gain, against a measured σ_out that extrapolates to ~10² V
at unity noise gain (pre-clamp). Direction and shape are both
consistent with the (grid, cathode) misinjection diagnosis.

## Verification recipe (build-time only, no plugin needed)

For any tube netlist:

```bash
melange compile <cir> --noise full --output /tmp/out.rs
grep -E 'NOISE_THERMAL_NODE_I|NOISE_THERMAL_NODE_J|NOISE_THERMAL_SQRT_INV_R_DEFAULT|NOISE_SHOT_NODE_I|NOISE_SHOT_NODE_J' /tmp/out.rs
```

Cross-reference each `NOISE_SHOT_NODE_I/J` pair against the
`NOISE_THERMAL_NODE_I/J` pairs to identify which node index is the
plate (it shows up paired with VCC in the Ra thermal stamp) and
which is the cathode (paired with ground in the Rk thermal stamp).
If the shot pair points at (grid, cathode) instead of (plate,
cathode), the bug is present.

Confirmed reproducers in this tree:

| Netlist | Triodes | NOISE_SHOT_NODE_I | NOISE_SHOT_NODE_J | Cross-ref says |
|---|---|---|---|---|
| `unstable/gimmicks/noyce-triode-12ax7.cir` (melange-circuits) | 1 | `[3]` | `[5]` | `(grid, cathode)` — should be `(plate, cathode) = (4, 5)` |
| `unstable/preamp/gold-press-riaa.cir` (melange-circuits) | 3 | `[4, 7, 10]` | `[6, 9, 11]` | `(g1,k1), (g2,k2), (g3,k3)` — should be `(p1,k1), (p2,k2), (p3,k3)` |

(The melange-circuits tree at `~/dev/melange-circuits/` is the
canonical source for these circuits as of the 2026-04-12
migration.)

## Root cause — exact file:line

### Triode path

**`crates/melange-solver/src/mna.rs:4224-4232`**:

```rust
Element::Triode {
    n_grid,
    n_plate,
    n_cathode,
    ..
} => vec![n_grid, n_plate, n_cathode],   // order: [grid, plate, cathode]
```

The `NonlinearDevice.node_indices` for a triode is
`[grid, plate, cathode]`. This matches the SPICE element
parse order (`T<name> n_grid n_plate n_cathode model`) in
`parser.rs:3237-3242` and is consistent with how the DK / nodal
emitters stamp the device elsewhere.

**`crates/melange-solver/src/codegen/ir.rs:2089-2104`**:

```rust
NonlinearDeviceType::Tube => {
    // Triode node_indices = [plate, grid, cathode];     // <-- comment is WRONG
    // pentode node_indices = [plate, grid, screen, cathode].  // <-- comment is WRONG (screen and cathode swapped)
    // Ip stamps plate-to-cathode in both cases.
    if nodes.len() >= 3 {
        let plate = nodes[0];                    // <-- triode: this is GRID, not PLATE
        let cathode = *nodes.last().unwrap();    // <-- triode: this is cathode ✓
                                                  // <-- pentode: this is SCREEN (or SUPPRESSOR), not cathode
        sources.push(ShotNoiseSource {
            name: format!("{}.Ip", dev.name),
            slot_idx: dev.start_idx,
            node_i: plate,
            node_j: cathode,
        });
    }
}
```

Two independent mistakes in the same block:
1. The comment for triodes asserts `[plate, grid, cathode]`. The
   actual mna.rs order is `[grid, plate, cathode]`. Picking
   `nodes[0]` therefore selects the grid, not the plate. Off-by-
   one in the same dimension.
2. `nodes.last()` happens to be the cathode for triodes (3-node)
   but is the **screen** for 4-node pentodes and the
   **suppressor** for 5-node pentodes. The "last" element of the
   node vec is not the cathode for either pentode form per
   mna.rs:4234-4242.

### Pentode path

**`crates/melange-solver/src/mna.rs:4234-4242`** (approximate;
verified locally via `sed -n`):

```rust
Element::Pentode {
    n_plate, n_grid, n_cathode, n_screen, n_suppressor, ..
} => {
    let mut nodes = vec![n_plate, n_grid, n_cathode, n_screen];
    if let Some(ns) = n_suppressor {
        nodes.push(ns);
    }
    ...
}
```

So:
- 4-node pentode order: `[plate, grid, cathode, screen]`
- 5-node pentode order: `[plate, grid, cathode, screen, suppressor]`

In `ir.rs`, `nodes[0]` = plate (correct), but `nodes.last()` =
screen (4-node) or suppressor (5-node). Cathode is always
`nodes[2]`. The pentode shot therefore lands between plate and
screen (or plate and suppressor) — the wrong shunt entirely.

Pentode dominant shot contribution physically lives at the
cathode-plate path (Ip), with partition statistics (Phase 5,
deferred) layering on top of it. The current behaviour stamps
plate-current granularity noise across the plate-screen path —
unphysical and also incorrect impedance-wise.

## Proposed fix

Match the mna.rs ordering exactly. Two acceptable patterns:

### Option A (minimal, two-line patch)

Replace `ir.rs:2089-2104` with:

```rust
NonlinearDeviceType::Tube => {
    // mna.rs node ordering (canonical):
    //   Triode  (3 nodes): [grid, plate, cathode]
    //   Pentode (4 nodes): [plate, grid, cathode, screen]
    //   Pentode (5 nodes): [plate, grid, cathode, screen, suppressor]
    // Ip flows plate↔cathode in all forms.
    let (plate_idx, cathode_idx) = match nodes.len() {
        3 => (1, 2),         // triode
        4 | 5 => (0, 2),     // pentode w/ or w/o suppressor
        _ => continue,
    };
    sources.push(ShotNoiseSource {
        name: format!("{}.Ip", dev.name),
        slot_idx: dev.start_idx,
        node_i: nodes[plate_idx],
        node_j: nodes[cathode_idx],
    });
}
```

The `continue` branch (rather than `panic!` or `unreachable!`) is
deliberate — keeps codegen robust if a future tube variant lands
with a different node count; the rest of Phase 2 still emits.

### Option B (preferred long-term, removes the magic-index footgun)

Add named accessors to `NonlinearDevice` for the physically-
meaningful terminals of each device class. Tube would expose
`fn plate_node(&self) -> usize` and `fn cathode_node(&self) ->
usize`; BJT exposes `collector`/`base`/`emitter`; etc. The shot-
source collector then asks the device for its (i_nl_port_+,
i_nl_port_−) directly. Eliminates the comment-as-spec coupling.

This is bigger than a Phase-2 bug fix and likely wants its own
issue; flagging only.

## Suggested test additions

`crates/melange-solver/tests/codegen_verification_tests.rs`
already covers shot wiring at a structural level. Add:

1. **`shot_node_pairs_match_inl_port_per_device_class`**: for each
   device class (Diode, BJT 2D, BJT FA, JFET, MOSFET, Triode,
   Pentode 4-node, Pentode 5-node), compile a minimal netlist
   with `--noise shot`, parse the emitted `NOISE_SHOT_NODE_I/J`
   and `NOISE_SHOT_SLOT_IDX`, and assert the indices match the
   physical `i_nl` port of the device. Use the device's own
   stamping geometry (or its `Element` enum variant) as the
   ground truth, not a hand-typed expected value — that way a
   future ordering refactor in mna.rs can't drift away from the
   assertion silently.

2. **`triode_shot_kTC_against_plate_impedance`**: emit
   `noyce-triode-12ax7.cir` with `--noise shot`, run N=2^17
   samples at 96 kHz, integrate the output variance, and assert
   it matches `(2·q·|Ip|) · ∫|H_plate→out(f)|² df` within ±15%.
   This is the tube analogue of `thermal_noise_matches_ktc_theorem`
   and would have caught the bug at the moment Phase 2 shipped.

3. **(stretch) `pentode_shot_kTC_against_plate_impedance`** —
   same shape, on a quiet EF86 stage from
   `unstable/preamp/` (or fabricate one) once pentode shot has a
   known-good reference netlist.

## What I'd recommend on the rollout

When the fix lands:

- Existing tube-circuit plugins that have already been calibrated
  with `set_shot_gain(g)` for "what they're emitting now" will
  shift quieter, possibly substantially. The character will also
  change — shot stamped at the plate is shaped by Z_plate; shot
  stamped at the grid is shaped by Z_grid · A_tube. That's a
  spectrum-shape change, not just a level change.
- Affected plugins in this repo's awareness: gold-press
  (RIAA preamp, 3 12AX7s), tube-preamp, sad-bastard (mixed-bias
  cascade), basic-bitch (split-band), pipe-shouter, TTH
  (cascaded triode), Noyce 12AX7 (newly-shipped).
- The character changes aren't bugs being fixed in those plugins
  — they're physics being applied correctly for the first time.
  Pinning a release note to "tube shot noise is now correct;
  recalibrate plugin output trim and spectrum locks" feels
  right.

## What's verified-OK

Cross-checked all other device classes' shot stamps against their
mna.rs node ordering:

| Device | mna.rs order | ir.rs stamp | Correct? |
|---|---|---|---|
| Diode | `[anode, cathode]` | `(nodes[0], nodes[1])` | ✓ |
| BJT 2D | `[collector, base, emitter]` | Ic `(nodes[0], nodes[2])`, Ib `(nodes[1], nodes[2])` | ✓ |
| BJT FA | `[collector, base, emitter]` | Ic `(nodes[0], nodes[2])` | ✓ |
| JFET | `[drain, gate, source]` | `(nodes[0], nodes[2])` | ✓ |
| MOSFET | `[drain, gate, source, bulk]` | `(nodes[0], nodes[2])` | ✓ |
| Triode | `[grid, plate, cathode]` | `(nodes[0], nodes.last())` = `(grid, cathode)` | **✗** |
| Pentode | `[plate, grid, cathode, screen, [supp]]` | `(nodes[0], nodes.last())` = `(plate, screen)` or `(plate, suppressor)` | **✗** |

Phase 3 flicker (`build_flicker_emission` in `ir.rs`) appears to
use the same per-device port layout — I haven't reproduced it
empirically yet, but pattern-match strongly suggests the same
fix applies one level above in the flicker path too. Worth a
look while the tube-shot patch is open.

## Pointer to the conversational paper trail

- Symptom + plugin-side measurement:
  `~/dev/oomox/docs/specs/noyce-triode-12ax7-measurements.md`
- melange-circuits diagnosis (the cross-reference proof):
  `~/dev/oomox/docs/specs/noyce-triode-12ax7-measurements-response.md`
- The two reproducer circuits:
  - `~/dev/melange-circuits/unstable/gimmicks/noyce-triode-12ax7.cir`
  - `~/dev/melange-circuits/unstable/preamp/gold-press-riaa.cir`

— melange-circuits agent
