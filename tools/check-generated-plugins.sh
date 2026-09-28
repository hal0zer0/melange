#!/usr/bin/env bash
# Generate plugin projects with `melange compile --format plugin` and run
# `cargo check` on each against the pinned nih-plug.
#
# The unit tests in plugin_template.rs only inspect the generated text; this is
# the check that the text compiles. Covers each lib.rs skeleton (stereo, mono,
# multi-output) and oversampling at 1x/2x/4x, plus the wet/dry dry-delay path.
#
# Usage: tools/check-generated-plugins.sh [path/to/melange]
# Env:   CARGO_BUILD_JOBS is honoured, as for any cargo invocation.
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

# name | deck | extra compile flags
CASES=(
  "stereo-1x|clipper.cir|--oversampling 1"
  "stereo-2x|clipper.cir|--oversampling 2"
  "stereo-4x-wetdry|clipper.cir|--oversampling 4 --wet-dry-mix"
  "mono-4x-wetdry|clipper.cir|--oversampling 4 --mono --wet-dry-mix"
  "multiout-2x|split.cir|--oversampling 2 --output-node lo,hi"
)

fail=0
for case in "${CASES[@]}"; do
  IFS='|' read -r name deck flags <<<"$case"
  echo "=== $name ($deck $flags)"
  # shellcheck disable=SC2086
  "$MELANGE" compile "$WORK/$deck" --format plugin $flags --name "check-$name" \
    -o "$WORK/$name" >"$WORK/$name.log" 2>&1 \
    || { cat "$WORK/$name.log"; echo "FAIL: compile $name"; fail=1; continue; }
  if ! (cd "$WORK/$name" && cargo check --quiet --lib); then
    echo "FAIL: cargo check $name"
    fail=1
  fi
done

[ "$fail" -eq 0 ] && echo "all generated plugin projects compile"
exit "$fail"
