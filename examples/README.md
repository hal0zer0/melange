# Bundled Examples

A small set of netlists that ship in-tree so you can compile something real
without first wiring up a circuit source. Point `melange` straight at the file.

## `passive-eq1a.cir` — Pultec-style passive tube EQ *(primary example)*

The flagship example. A passive bridge-T EQ network bracketed by transformers,
with a tube makeup amplifier: HS-56 input transformer → passive EQ → HS-29
coupling/phase splitter → 12AX7 push-pull → .25µF coupling → 12AU7 push-pull →
S-217-D output transformer, with global negative feedback from the output
transformer's tertiary winding back to the 12AX7 cathodes.

It is the example precisely because it is *hard*: **4 tubes, 3 transformers,
7 pots, 3 switches, and a global NFB loop** (N=52, M=8). Multiple coupled
transformers force the nodal Schur solver and backward-Euler; the feedback loop
makes it a genuine stability test rather than a toy RC filter. Coupled
transformers with global feedback wrapped around four nonlinear tubes is one of
the hardest configurations melange solves — a real workout for the solver, not a
claim that nothing else is harder.

```bash
# Frequency response at the default (flat) knob settings
melange analyze examples/passive-eq1a.cir

# Sweep a control. Pot values are in ohms and must sit inside the declared
# range — `melange nodes` prints it (LF Boost is 100..10000 ohm); out-of-range
# values are accepted silently today, so check the range rather than the output.
melange analyze examples/passive-eq1a.cir --pot "LF Boost=10k" --switch "LF Freq=1"

# Run audio through it
melange simulate examples/passive-eq1a.cir --input-audio guitar.wav -o out.wav

# Compile to a plugin project. Generate it OUTSIDE this repo if you intend to
# bundle a CLAP/VST3 — the bundler walks up to the outermost Cargo.toml, so a
# project nested in the melange checkout builds but will not bundle.
melange compile examples/passive-eq1a.cir --format plugin -o ~/passive-eq
cd ~/passive-eq && cargo build --release
```

**Provenance, honestly.** This proves melange *solves* a genuinely hard
topology. It does **not** prove fidelity to the original hardware. The amplifier section
now follows the verified Sowter E-72,658-2 drawing (eight schematic corrections
applied 2026-08-25); the **EQ network is a reconstruction** — no factory drawing
of the EQ exists at all. The file's own header carries the full accounting. What
the curves faithfully reproduce is the reconstruction, which is a real and useful
thing and is not the same sentence as "sounds like the real box."
