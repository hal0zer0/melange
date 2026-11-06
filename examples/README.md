# Bundled Examples

A small set of netlists that ship in-tree so you can compile something real
without first wiring up a circuit source. Point `melange` straight at the file,
or name it bare (`compressor76`, `passive-eq1a`): both are built into the
binary from these exact bytes.

## `compressor76.cir` — FET feedback limiter *(the demo circuit)*

The circuit in the [demo video](https://youtu.be/8WRu_ngPL6E). A FET feedback limiter in the style of the
1176LN, transcribed from the production drawings: a JFET as the gain element
(a voltage-variable resistor on melange's symmetric subthreshold law), a
sidechain whose four push buttons set the ratio by rewiring a resistor network,
a diode-bridge detector driving a control amplifier, Attack and Release pots
that are time constants in that network, and a Darlington line amplifier into
an output transformer. 16 nonlinear devices (1 JFET, 11 BJTs, 4 diodes), 2
transformers, N = 84, M = 28; the cost trigger routes it to the nodal solver's
full-LU sub-path, as `melange compile examples/compressor76.cir -v` reports.

```bash
# Its pots and buttons (Output, Attack, Release; Ratio 4/8/12/20; Attack Off)
melange nodes examples/compressor76.cir

# Gain vs frequency with the 20:1 button in
melange analyze examples/compressor76.cir --switch "Ratio 20=1"

# A WAV of your own through it, 4:1 button in, Output at 50 %
melange simulate examples/compressor76.cir --switch "Ratio 4=1" --input-audio my-recording.wav -o out.wav

# Compile to a plugin project, OUTSIDE this repo, with the 4:1 button baked in
# as the plugin's starting position (the netlist's own state has no button in).
melange compile examples/compressor76.cir --format plugin -o ~/compressor76 --switch "Ratio 4=1"
cd ~/compressor76 && bash build.sh       # CLAP + VST3 land in target/bundled/
```

**Provenance, honestly.** The deck is calibrated by the factory test procedure
(the Q-bias trim, the 20:1 production-test chord and the 20:1 threshold), and
out of sample the 12/8/4:1 production chords at 2 and 20 kHz pass 6 of 6, the
settled thresholds sit inside the manual's ±2 dBm, and THD at +24 dBm reads
0.095 % against a 0.5 % spec. Not matched: the release timing (the slow end
reads 1.96 s where the manual says up to 1.1 s) and everything past the line
amp's clip, where the output transistor's breakdown is modelled from a
datasheet minimum. It is an `unstable/`-tier circuit, played in a DAW but never
measured against a hardware unit. The file's own header carries the full
accounting, and the canonical copy is `unstable/dynamics/compressor76.cir` in
the circuits repository; this file is kept byte-identical to it.

**The plugin in the video is two decks.** The one-deck build above is the exact
reference and runs close to the line on one core at 48 kHz and 2× when the
input is cranked. The circuits repository's `tools/compressor76-split/regen.sh`
builds the default plugin from the same netlist cut at the Output pot's wiper
(front end and line amp as two decks, chained per inner sample), which sits
46 to 62 dB below the one-deck at every control extreme and runs with headroom.

## `passive-eq1a.cir` — Pultec-style passive tube EQ

The earlier demo circuit, still built in. A passive bridge-T EQ network bracketed by transformers,
with a tube makeup amplifier: HS-56 input transformer → passive EQ → HS-29
coupling/phase splitter → 12AX7 push-pull → .25µF coupling → 12AU7 push-pull →
S-217-D output transformer, with global negative feedback from the output
transformer's tertiary winding back to the 12AX7 cathodes.

It is the example precisely because it is *hard*: **4 tubes, 3 transformers,
7 pots, 3 switches, and a global NFB loop** (N=52, M=8). Three coupled
transformer groups route it to the nodal solver (Schur sub-path), with
trapezoidal integration, as `melange compile examples/passive-eq1a.cir -v`
reports; the feedback loop makes it a genuine stability test rather than a toy
RC filter. Coupled transformers with global feedback wrapped around four
nonlinear tubes is one of the hardest configurations melange solves — a real
workout for the solver, not a claim that nothing else is harder.

```bash
# Frequency response at the default (flat) knob settings
melange analyze examples/passive-eq1a.cir

# Sweep a control. Pot values are in ohms and must sit inside the declared
# range — `melange nodes` prints it (LF Boost is 100..10000 ohm); a value
# outside it is refused.
melange analyze examples/passive-eq1a.cir --pot "LF Boost=10k" --switch "LF Freq=1"

# Run a WAV of your own through it (no test audio ships with melange; any
# 16/24-bit PCM or 32-bit float WAV works, first channel used). Without
# --input-audio, simulate renders its own 1 kHz test tone.
melange simulate examples/passive-eq1a.cir --input-audio my-recording.wav -o out.wav

# Compile to a plugin project. Generate it OUTSIDE this repo if you intend to
# bundle a CLAP/VST3 — the bundler walks up to the outermost Cargo.toml, so a
# project nested in the melange checkout builds but will not bundle.
melange compile examples/passive-eq1a.cir --format plugin -o ~/passive-eq
cd ~/passive-eq && bash build.sh         # CLAP + VST3 land in target/bundled/
```

**Provenance, honestly.** This proves melange *solves* a genuinely hard
topology. It does **not** prove fidelity to the original hardware. The amplifier section
now follows the verified Sowter E-72,658-2 drawing (eight schematic corrections
applied 2026-08-25); the **EQ network is a reconstruction** — no factory drawing
of the EQ exists at all. The file's own header carries the full accounting. What
the curves faithfully reproduce is the reconstruction, which is a real and useful
thing and is not the same sentence as "sounds like the real box."
