# melange-solver fuzz harness

Cargo-fuzz targets for the SPICE netlist parser and downstream build pipeline.

## Prerequisites

- Rust nightly toolchain (`rustup install nightly`)
- `cargo-fuzz` subcommand: `cargo install cargo-fuzz`
- A system clang that libfuzzer-sys can link against (default on most Linux)

## Running the parse_netlist target

From the `melange-solver` crate root:

```sh
cargo +nightly fuzz run parse_netlist -- -max_total_time=300
```

Flags worth knowing:

- `-max_total_time=N`  stop after N seconds (5-minute smoke run above)
- `-max_len=65536`     cap generated inputs at 64 KB (the parser's hard
                       limit is 10 MB, but most bugs show up well under 1 KB
                       and smaller inputs fuzz much faster)
- `-jobs=N -workers=N` run N parallel fuzzing processes
- `-runs=N`            stop after N executions instead of a wall-clock timer

Corpus files accumulate in `fuzz/corpus/parse_netlist/`.
Any crash is saved as a reproducer to `fuzz/artifacts/parse_netlist/crash-*`.

## Reproducing a crash

```sh
cargo +nightly fuzz run parse_netlist fuzz/artifacts/parse_netlist/crash-<hash>
```

or, to feed a specific input:

```sh
cargo +nightly fuzz run parse_netlist path/to/input.txt
```

## Scope

`parse_netlist.rs` exercises the full pipeline in order:

1. UTF-8 validity gate (non-UTF-8 inputs are dropped early)
2. `Netlist::parse`
3. `Netlist::expand_subcircuits`
4. `MnaSystem::from_netlist`
5. `DkKernel::from_mna(44100.0)`
6. `CircuitIR::from_kernel(&kernel, &mna, &netlist, &CodegenConfig::default())`

Each stage's `Err` result returns early — errors are expected and fine.
**Panics** at any stage are bugs. Known past panics this target catches:

- `parse_value()` byte-slice panic on non-ASCII scale chars
  (fixed in commit `e96c340`, 2026-04-10)
- anything in the input-size cap layer added alongside this harness

## Relationship to the main workspace

The `fuzz/` directory declares its own `[workspace]` and is listed under
`[workspace].exclude` in the root `Cargo.toml`. This keeps `libfuzzer-sys`
(nightly-only) off the stable build path — `cargo build --workspace`
from the repo root does not touch this crate.
