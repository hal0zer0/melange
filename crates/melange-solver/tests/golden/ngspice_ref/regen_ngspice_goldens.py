#!/usr/bin/env python3
"""Regenerate the ngspice-referenced golden JSONs from first principles.

Provenance tool for review-round-2 gate H1 (self-referential goldens):
the three goldens below hold NGSPICE output, not melange output, so a
melange re-record cannot silently bless a solver change. Run this script
(needs `ngspice` on PATH) and diff the JSONs; the `output` arrays must be
reproducible bit-for-bit modulo ngspice version/platform noise well below
each file's tolerance.

    opamp_inverting_sine_1k.json      1 kHz sine, 0.1 V, 480 samples @ 48 kHz
    opamp_noninverting_sine_1k.json   1 kHz sine, 0.1 V, 480 samples @ 48 kHz
    triode_cc_small_sine_500hz.json   500 Hz sine, 0.04 V, 960 samples @ 48 kHz

Protocol (mirrors crates/melange-validate spice_runner/harness):
  - input = Thevenin PWL pair: V source + 1 ohm into node `in`,
    PWL points at t_i = i/fs, v_i = A*sin(2*pi*f*t_i)  (starts at 0 V);
  - `.OPTIONS INTERP reltol=1e-4` immediately after the title line;
  - `.TRAN 1/fs (n-1)/fs` -> exactly n uniform samples t=0..(n-1)/fs;
  - the raw v(out) is passed through the same 5 Hz DC blocker the
    generated melange code applies (dc_block_signal: x_prev seeded from
    the first sample), because the golden tests compare against melange
    output that is already 5 Hz-blocked.

Op-amp twin: melange `.model OA(AOL, ROUT)` is a VCCS Gm=AOL/ROUT with
output conductance Go=1/ROUT that injects +Gm*(V+ - V-) INTO the output
node. ngspice encoding with CORRECT polarity (G element: I(n+ -> n-) =
gm*V(nc+,nc-), so `G1 out 0 V- V+ gm` injects Gm*(V+ - V-) into out):
  inverting     G1 out 0 vminus 0  200000   (V+ = ground)
  noninverting  G1 out 0 vminus in 200000   (V+ = in)

Triode twin: published Koren triode plate current as a B source,
  E1 = (Vpk/KP)*ln(1+exp(KP*(1/MU + Vgk/sqrt(KVB+Vpk^2))))
  Ip = 2*E1^EX/KG1 for E1 > 0   (the `(1+sgn(E1))` form)
encoded as `2*pwr(uramp(E1),EX)/KG1`, plus the 10 pF Cgk/Cpk parasitics
that DkKernel::from_mna auto-inserts. Grid current is omitted: melange
dc-op reports i_nl(grid) = 0 A at the -1.2 V bias and Vgk stays below
-1.1 V over the whole 40 mV swing. DC OP cross-check (both engines):
plate 170.1017 V, cathode 1.1985 V.

Measured margins 2026-07-21 (ngspice-42, HEAD 8945b67 + campaign tree):
  opamp inv:  max |melange - ngspice| = 1.12e-8 V  -> tolerance 1e-6
  opamp ni:   max |melange - ngspice| = 1.27e-8 V  -> tolerance 1e-6
  triode:     max |melange - ngspice| = 2.54e-3 V  -> tolerance 6e-3
              (0.105% normalized rms on a 2.38 V peak waveform)
"""

import json
import math
import os
import subprocess
import tempfile

FS = 48000.0
HERE = os.path.dirname(os.path.abspath(__file__))
GOLD = os.path.dirname(HERE)


def pwl(amp, freq, n):
    pts = [(i / FS, amp * math.sin(2 * math.pi * freq * i / FS)) for i in range(n)]
    return " ".join(f"{t:.9e} {v:.9e}" for t, v in pts)


def dc_block(x):
    r = 1.0 - 2.0 * math.pi * 5.0 / FS
    xp, yp, out = x[0], 0.0, []
    for v in x:
        y = v - xp + r * yp
        xp, yp = v, y
        out.append(y)
    return out


def run_ngspice(deck):
    with tempfile.TemporaryDirectory() as d:
        cir = os.path.join(d, "ref.cir")
        dat = os.path.join(d, "out.dat")
        open(cir, "w").write(deck.replace("{DAT}", dat))
        subprocess.run(["ngspice", "-b", cir], check=True, capture_output=True)
        return [float(line.split()[1]) for line in open(dat)]


def deck(tmpl_name, amp, freq, n):
    tmpl = open(os.path.join(HERE, tmpl_name)).read()
    return tmpl.replace("{PWL}", pwl(amp, freq, n)).replace(
        "{TRAN}", f".TRAN {1/FS:.9e} {(n-1)/FS:.9e}"
    )


def regen(json_name, tmpl_name, amp, freq, n):
    path = os.path.join(GOLD, json_name)
    j = json.load(open(path))
    j["output"] = dc_block(run_ngspice(deck(tmpl_name, amp, freq, n)))
    open(path, "w").write(json.dumps(j, indent=1))
    print(f"regenerated {json_name}: {len(j['output'])} samples, "
          f"peak {max(abs(v) for v in j['output']):.6f}")


if __name__ == "__main__":
    regen("opamp_inverting_sine_1k.json", "opamp_inverting_ref.cir.tmpl", 0.1, 1000.0, 480)
    regen("opamp_noninverting_sine_1k.json", "opamp_noninverting_ref.cir.tmpl", 0.1, 1000.0, 480)
    regen("triode_cc_small_sine_500hz.json", "triode_cc_small_ref.cir.tmpl", 0.04, 500.0, 960)
