#!/usr/bin/env bash
# Generate plugin projects with `melange compile --format plugin` and run
# `cargo check` on each against the pinned nih-plug.
#
# The unit tests in plugin_template.rs only inspect the generated text; this is
# the check that the text compiles. Covers all three lib.rs skeletons the CLI
# can generate -- mono (one output node), stereo from two output nodes (one per
# channel), and stereo from one output node run as two circuit instances
# (--stereo, with per-channel noise seeding) -- each with and without
# parameters, oversampling at 1x/2x/4x, the wet/dry dry-delay path, every
# control kind (pot, wiper, gang, switch) on the two-instance layout, each
# --cpu-baseline, and the "Circuit Noise" switch a --noise build gets, on every
# layout (once as the only parameter).
#
# Usage: tools/check-generated-plugins.sh [path/to/melange]
# Env:   CARGO_BUILD_JOBS is honoured, as for any cargo invocation.
#        ONLY=<regex> runs only the cases whose name matches (bash =~).
#        The work directory comes from mktemp, so TMPDIR places it.
set -euo pipefail

MELANGE="$(realpath "${1:-target/release/melange}")"
[ -x "$MELANGE" ] || { echo "no melange binary at $MELANGE" >&2; exit 2; }

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
# One target dir for every project, so nih-plug builds once.
export CARGO_TARGET_DIR="$WORK/target"

cat > "$WORK/clipper.cir" <<'EOF'
* Diode clipper with a drive pot
Rin in mid 4.7k
Rdrive mid clip 10k
D1 clip 0 1N4148
D2 0 clip 1N4148
Rload clip out 1k
Cout out 0 100n
.model 1N4148 D(IS=2.52e-9 N=1.752)
.pot Rdrive 1k 100k "Drive"
.end
EOF

cat > "$WORK/stereo.cir" <<'EOF'
* Diode clipper with a drive pot and two outputs
Rin in mid 4.7k
Rdrive mid clip 10k
D1 clip 0 1N4148
D2 0 clip 1N4148
Rlo clip lo 1k
Clo lo 0 100n
Chi clip hi 100n
Rhi hi 0 1k
.model 1N4148 D(IS=2.52e-9 N=1.752)
.pot Rdrive 1k 100k "Drive"
.end
EOF

cat > "$WORK/split.cir" <<'EOF'
* Diode clipper with two outputs
Rin in mid 4.7k
D1 mid 0 1N4148
D2 0 mid 1N4148
Rlo mid lo 1k
Clo lo 0 100n
Chi mid hi 100n
Rhi hi 0 1k
.model 1N4148 D(IS=2.52e-9 N=1.752)
.end
EOF

cat > "$WORK/controls.cir" <<'EOF'
* Diode clipper with every control kind
Rin in mid 4.7k
Rdrive mid clip 10k
D1 clip 0 1N4148
D2 0 clip 1N4148
Rga clip a 10k
Rgb a 0 10k
Rtop a out 5k
Rbot out 0 5k
Cout out 0 100n
Rsw out 0 100k
.model 1N4148 D(IS=2.52e-9 N=1.752)
.pot Rdrive 1k 100k "Drive"
.pot Rga 1k 20k
.pot Rgb 1k 20k
.gang "Balance" Rga !Rgb
.wiper Rtop Rbot 10k "Volume"
.switch Rsw 100k 10k "Load"
.end
EOF

# name | deck | extra compile flags
# One output node makes a mono plugin, or with --stereo a stereo one of two
# circuit instances; two output nodes make a stereo one. Ear protection is
# a parameter too, so the "noparams" cases turn it and the level knobs off to
# reach the parameter-less loop. A --noise case must also carry the
# "circuit_noise" parameter.
CASES=(
  "mono-1x|clipper.cir|--oversampling 1"
  "mono-4x-wetdry|clipper.cir|--oversampling 4 --mono --wet-dry-mix"
  "mono-noparams|split.cir|--output-node lo --no-level-params --no-ear-protection"
  "mono-noise|clipper.cir|--noise thermal --noise-seed 7"
  "mono-noise-only-param|split.cir|--output-node lo --no-level-params --no-ear-protection --noise thermal"
  "stereo-1x|stereo.cir|--oversampling 1 --output-node lo,hi"
  "stereo-2x|stereo.cir|--oversampling 2 --output-node lo,hi"
  "stereo-4x-wetdry|stereo.cir|--oversampling 4 --wet-dry-mix --output-node lo,hi"
  "stereo-noparams-2x|split.cir|--oversampling 2 --output-node lo,hi --no-level-params --no-ear-protection"
  "stereo-noise-2x|stereo.cir|--oversampling 2 --output-node lo,hi --noise thermal"
  "stereo-dual-1x|controls.cir|--stereo --oversampling 1 --noise thermal --noise-seed 7"
  "stereo-dual-2x-wetdry|controls.cir|--stereo --oversampling 2 --wet-dry-mix --noise thermal"
  "stereo-dual-noparams|split.cir|--stereo --output-node lo --no-level-params --no-ear-protection"
  "portable-v2|clipper.cir|--cpu-baseline x86-64-v2"
  "portable-x86-64|clipper.cir|--cpu-baseline x86-64"
)

fail=0
for case in "${CASES[@]}"; do
  IFS='|' read -r name deck flags <<<"$case"
  if [ -n "${ONLY:-}" ] && ! [[ $name =~ $ONLY ]]; then continue; fi
  echo "=== $name ($deck $flags)"
  # shellcheck disable=SC2086
  "$MELANGE" compile "$WORK/$deck" --format plugin $flags --name "check-$name" \
    -o "$WORK/$name" >"$WORK/$name.log" 2>&1 \
    || { cat "$WORK/$name.log"; echo "FAIL: compile $name"; fail=1; continue; }
  if [[ " $flags " == *" --noise "* ]] \
    && ! grep -q '#\[id = "circuit_noise"\]' "$WORK/$name/src/lib.rs"; then
    echo "FAIL: $name has no Circuit Noise parameter"
    fail=1
  fi
  # nih-plug's VST3 export macro (vst3_com, at the pinned rev) trips the
  # future-incompatibility lint semicolon_in_expressions_from_non_local_macros
  # on Rust >= 1.99; CI's -Dwarnings would turn that upstream warning into a
  # failure of every generated project. Allow that one lint here only, so the
  # check still fails on any warning from code melange generates. Remove when
  # the nih-plug pin moves past the fix.
  if ! (cd "$WORK/$name" && RUSTFLAGS="${RUSTFLAGS:-} -A semicolon_in_expressions_from_non_local_macros" cargo check --quiet --lib); then
    echo "FAIL: cargo check $name"
    fail=1
  fi
done

[ "$fail" -eq 0 ] && echo "all generated plugin projects compile"
exit "$fail"
