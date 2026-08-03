#!/usr/bin/env bash
# melange perf-harness — in-process throughput of a GENERATED circuit.
#
# Compiles a .cir to standalone Rust via melange, appends the timing driver,
# builds with the golden-harness rustc regime, runs it, prints one JSON line.
# Reusable for any fix batch: rerun before and after, diff the ns/sample.
#
# Usage:
#   bench.sh <label> <cir-path> [extra melange-compile flags...]
# Env:
#   MELANGE       compile command (default: cargo run -q -p melange-cli --release --)
#   PERF_SAMPLES  timed samples per rep (default 2000000)
#   PERF_REPS     repetitions, min taken (default 7)
#   RUSTFLAGS_EXTRA  appended to rustc (default: -C target-cpu=native -C codegen-units=1)
set -euo pipefail

HARNESS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HARNESS_DIR/../.." && pwd)"
DRIVER_MAIN="$HARNESS_DIR/driver_main.rs"

LABEL="${1:?usage: bench.sh <label> <cir> [flags...]}"
CIR="${2:?usage: bench.sh <label> <cir> [flags...]}"
shift 2
EXTRA_FLAGS=("$@")

MELANGE="${MELANGE:-cargo run -q -p melange-cli --release --}"
RUSTFLAGS_EXTRA="${RUSTFLAGS_EXTRA:--C target-cpu=native -C codegen-units=1}"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
CIRCUIT_RS="$WORK/circuit.rs"
DRIVER_RS="$WORK/driver.rs"
BIN="$WORK/driver"

# 1. Generate standalone Rust from the netlist (current source HEAD).
( cd "$REPO_ROOT" && $MELANGE compile "$CIR" --format code "${EXTRA_FLAGS[@]}" -o "$CIRCUIT_RS" ) >&2

LOC=$(wc -l < "$CIRCUIT_RS")
echo "  [$LABEL] generated circuit.rs: $LOC lines" >&2

# 2. Append the timing driver and compile with the golden-harness regime (-O, edition 2024).
cat "$CIRCUIT_RS" "$DRIVER_MAIN" > "$DRIVER_RS"
# shellcheck disable=SC2086
rustc "$DRIVER_RS" -o "$BIN" --edition=2024 -O $RUSTFLAGS_EXTRA >&2

# 3. Run. Driver prints one JSON line on stdout (+ human line on stderr).
PERF_LABEL="$LABEL" "$BIN"
