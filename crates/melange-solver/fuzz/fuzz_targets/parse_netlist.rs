#![no_main]
//! Fuzz target: parse + build pipeline for the melange SPICE parser.
//!
//! Feeds arbitrary bytes through:
//!   str::from_utf8
//!     → Netlist::parse
//!     → Netlist::expand_subcircuits
//!     → MnaSystem::from_netlist
//!     → DkKernel::from_mna(44100.0)
//!     → CircuitIR::from_kernel(&kernel, &mna, &netlist, &CodegenConfig::default())
//!
//! Each stage is behind an `Ok(...)` check; `Err` results are fine — panics
//! are bugs. This catches:
//!   - parse_value() slicing panics on odd UTF-8 (the 2026-04 regression)
//!   - arithmetic overflows in MNA stamping
//!   - unchecked index/array operations in DK kernel or IR builder
//!   - accidental assertions inside the codegen pipeline
//!
//! # Running
//!
//! Requires the cargo-fuzz subcommand and a nightly toolchain:
//!
//!     cargo install cargo-fuzz
//!     cd crates/melange-solver
//!     cargo +nightly fuzz run parse_netlist -- -max_total_time=300
//!
//! The `-max_total_time` flag caps the run at 5 minutes; bump it for longer
//! campaigns. Corpus files are written to `fuzz/corpus/parse_netlist/`;
//! crashes land in `fuzz/artifacts/parse_netlist/`.
//!
//! # Input size cap
//!
//! The parser rejects inputs larger than `MAX_NETLIST_BYTES` (10 MB) with
//! a fast path before any allocation, so libfuzzer does not waste cycles
//! mutating multi-megabyte inputs that will never be parsed. For best
//! throughput, start the fuzzer with `-max_len=65536` or similar.

use libfuzzer_sys::fuzz_target;

use melange_solver::{
    codegen::CodegenConfig,
    dk::DkKernel,
    mna::MnaSystem,
    parser::Netlist,
    CircuitIR,
};

fuzz_target!(|data: &[u8]| {
    // Cheap UTF-8 gate: the parser only accepts &str. Invalid UTF-8 is a
    // dead input; bail out quickly so libfuzzer can try a new mutation.
    let Ok(s) = std::str::from_utf8(data) else {
        return;
    };

    // Stage 1: parse. Errors here are legitimate (bad syntax, too-long
    // node names, exceeded caps, etc.) and should never panic.
    let Ok(mut netlist) = Netlist::parse(s) else {
        return;
    };

    // Stage 2: subcircuit expansion. This can fail with cycle detection,
    // undefined refs, etc. — all fine.
    if netlist.expand_subcircuits().is_err() {
        return;
    }

    // Stage 3: MNA stamping.
    let Ok(mna) = MnaSystem::from_netlist(&netlist) else {
        return;
    };

    // Stage 4: DK kernel build at a fixed sample rate.
    let Ok(kernel) = DkKernel::from_mna(&mna, 44100.0) else {
        return;
    };

    // Stage 5: CircuitIR build with default codegen config. Many circuits
    // will fail here (unsupported topology, invalid model, etc.); that is
    // not a bug. What we care about is that the entire chain never panics.
    let config = CodegenConfig::default();
    let _ = CircuitIR::from_kernel(&kernel, &mna, &netlist, &config);
});
