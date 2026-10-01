#!/usr/bin/env bash
# Run the workspace test suite the way CI's "Test" job runs it, environment
# included, before a push to main (which is a release).
#
# Matching CI's command is not enough. The Test job (.github/workflows/ci.yml)
# installs only libfontconfig1-dev: ngspice is NOT on its PATH (only the
# separate "SPICE Validation" job installs it). A test that passes here
# because ngspice is installed can fail there. That happened right after the
# v0.1.12 tag. So this script hides ngspice from PATH, sets
# RUSTFLAGS=-Dwarnings as CI does, and runs with --no-fail-fast so that one
# failing test binary cannot hide another (CI stops at the first).
#
# Usage: tools/ci-test-gate.sh [extra cargo test args...]
# Env:   GATE_CPUS (default 0-23), GATE_JOBS (default 24),
#        GATE_TEST_THREADS (default 12). Runs at idle priority. If GATE_CPUS
#        names no CPU this machine has, or taskset is missing, runs unpinned.
#
# If ci.yml's Test job changes what it installs, update HIDE below to match.
set -euo pipefail

HIDE=(ngspice)
CPUS="${GATE_CPUS:-0-23}"
JOBS="${GATE_JOBS:-24}"
THREADS="${GATE_TEST_THREADS:-12}"

farm="$(mktemp -d)"
trap 'rm -rf "$farm"' EXIT

# Rebuild PATH from symlinks, minus the hidden tools. Every PATH directory is
# folded in (so /bin -> /usr/bin and friends are covered), first entry wins.
IFS=: read -r -a dirs <<<"$PATH"
for d in "${dirs[@]}"; do
    [ -d "$d" ] || continue
    for f in "$d"/*; do
        name="$(basename "$f")"
        [ -e "$farm/$name" ] && continue
        skip=0
        for h in "${HIDE[@]}"; do [ "$name" = "$h" ] && skip=1; done
        [ "$skip" -eq 0 ] && ln -s "$f" "$farm/$name" 2>/dev/null || true
    done
done

for h in "${HIDE[@]}"; do
    if PATH="$farm" command -v "$h" >/dev/null 2>&1; then
        echo "ci-test-gate: failed to hide $h from PATH" >&2
        exit 2
    fi
done
echo "ci-test-gate: hidden from PATH: ${HIDE[*]}; RUSTFLAGS=-Dwarnings; --no-fail-fast" >&2

# CPU pinning and idle I/O are etiquette, not part of the gate. taskset
# silently drops CPUs this machine lacks, so it only fails when none of
# GATE_CPUS exist (or the list does not parse); taskset/ionice are also absent
# off Linux. In those cases run without them rather than fail the gate.
prefix=()
if command -v taskset >/dev/null 2>&1 && taskset -c "$CPUS" true 2>/dev/null; then
    prefix+=(taskset -c "$CPUS")
else
    echo "ci-test-gate: cannot pin to CPUs '$CPUS' on this machine ($(nproc 2>/dev/null || echo '?') CPUs); running unpinned" >&2
fi
prefix+=(nice -n 19)
if command -v ionice >/dev/null 2>&1 && ionice -c 3 true 2>/dev/null; then
    prefix+=(ionice -c 3)
else
    echo "ci-test-gate: ionice unavailable; running without idle I/O class" >&2
fi

cd "$(dirname "$0")/.."
PATH="$farm" RUSTFLAGS=-Dwarnings \
    "${prefix[@]}" \
    cargo test --workspace -j "$JOBS" --no-fail-fast "$@" -- --test-threads "$THREADS"
