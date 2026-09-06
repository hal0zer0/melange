//! F7 — are the routing/emitter predicate THRESHOLD differences reachable?
//!
//! Two places decide "does this circuit have a positive K diagonal on a
//! dimension that actually injects current", and they are spelled differently:
//!
//! ```text
//! routing.rs (k_diag_unsafe):
//!     kernel.k[i*m+i] >= 0.0
//!     && mna.n_i.iter().any(|row| row[i].abs() >= 1e-30)
//!
//! nodal_emitter.rs (has_positive_k_with_current):
//!     ir.matrices.k[i*m+i] > 0.0          (spelled `if k_ii <= 0.0 { false }`)
//!     && ir.sparsity.n_i.nz_by_row.iter().any(|row| row.contains(&i))
//!                                          (nz built at SPARSITY_THRESHOLD = 1e-20)
//! ```
//!
//! **These are NOT duplicates and must never be collapsed into one value.** They
//! read *different matrices*: `routing` sees the DK kernel and the pre-expansion
//! `MnaSystem`, the emitter sees the IR built *after*
//! `expand_bjt_internal_nodes`. They also feed different decisions (DK-vs-nodal
//! versus Schur-vs-full-LU). An earlier plan revision proposed reconciling them
//! to a single definition; that was withdrawn as technically wrong, because it
//! would have made the Schur-vs-full-LU choice on pre-expansion matrices.
//!
//! What *is* worth knowing is whether the two THRESHOLD differences — `>= 0.0`
//! vs `> 0.0`, and `1e-30` vs `1e-20` — are ever reachable in practice. That is
//! a measurement, not a refactor, and it is this test.
//!
//! **Headline: one of the two is reachable and the other is not.** The
//! magnitude difference never fires anywhere in the library; the sign
//! difference fires on 20 dimensions across 8 circuits, one of which ships.
//! That was not the prediction, and it turns "the definitions differ" from an
//! untidiness into a load-bearing fact.
//!
//! To keep the matrix choice from confounding the threshold question, each
//! variant is evaluated on **its own** matrices; the test asks only whether a
//! circuit lands in the band where the two spellings would disagree:
//!
//!   * `>= 0.0` vs `> 0.0` differ only when `K[i][i]` is **bit-exactly** `0.0`
//!     on a dimension with a live `N_i` column. Both sites document the known
//!     `K[i][i] == 0` constructions (MOSFET insulated gate, VCA control port)
//!     as having an *all-zero* `N_i` column, which both predicates already
//!     exclude — so the band was PREDICTED empty by construction. **Measured,
//!     it is not.** See the RESULT block at the bottom of the test.
//!   * `1e-30` vs `1e-20` differ only when some `|N_i[a][i]|` lands in
//!     `[1e-30, 1e-20)`. `N_i` entries are built from `±1.0`, `±1/beta_f` and
//!     `1 + 1/beta_f`, so reaching that band needs `BF > 1e20` on a `.model`
//!     card — constructible maliciously, not by any physical device.
//!
//! Ignored by default: it sweeps the whole `melange-circuits` checkout, which
//! is a sibling repo and absent in CI. Run with:
//!
//! ```text
//! cargo test -p melange-solver --test f7_routing_predicate_reachability_tests -- --include-ignored --nocapture
//! ```

use melange_solver::codegen::ir::CircuitIR;
use melange_solver::codegen::CodegenConfig;
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

/// Where the netlist library lives. Absent in CI — the test skips, loudly.
fn circuits_root() -> Option<std::path::PathBuf> {
    let p = dirs_home()?.join("dev/melange-circuits");
    p.is_dir().then_some(p)
}

fn dirs_home() -> Option<std::path::PathBuf> {
    std::env::var_os("HOME").map(std::path::PathBuf::from)
}

fn collect_cir(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            if p.file_name().is_some_and(|n| n == ".git") {
                continue;
            }
            collect_cir(&p, out);
        } else if p.extension().is_some_and(|x| x == "cir") {
            out.push(p);
        }
    }
}

/// Does any `K[i][i]` sit bit-exactly at `0.0` on a dimension with a live `N_i`
/// column? That is the *only* input on which `>= 0.0` and `> 0.0` disagree.
fn exact_zero_k_with_live_ni(k: &[f64], m: usize, live: &dyn Fn(usize) -> bool) -> Vec<usize> {
    (0..m).filter(|&i| k[i * m + i] == 0.0 && live(i)).collect()
}

/// Does any `|N_i[a][i]|` land in `[1e-30, 1e-20)`? That is the *only* band on
/// which the two magnitude thresholds disagree.
fn ni_in_disagreement_band(n_i: &[Vec<f64>], m: usize) -> Vec<(usize, usize, f64)> {
    let mut hits = Vec::new();
    for (a, row) in n_i.iter().enumerate() {
        for (i, &entry) in row.iter().take(m).enumerate() {
            if (1e-30..1e-20).contains(&entry.abs()) {
                hits.push((a, i, entry));
            }
        }
    }
    hits
}

#[test]
#[ignore = "sweeps the melange-circuits sibling checkout; absent in CI"]
fn f7_predicate_threshold_reachability_across_the_library() {
    let Some(root) = circuits_root() else {
        eprintln!("SKIP: ~/dev/melange-circuits not present");
        return;
    };
    let mut files = Vec::new();
    collect_cir(&root, &mut files);
    files.sort();
    assert!(!files.is_empty(), "found no .cir files under {root:?}");

    let mut parsed = 0usize;
    let mut kernel_ok = 0usize;
    let mut ir_ok = 0usize;
    let mut zero_k_hits: Vec<String> = Vec::new();
    let mut band_hits: Vec<String> = Vec::new();

    for f in &files {
        let Ok(text) = std::fs::read_to_string(f) else {
            continue;
        };
        let Ok(netlist) = Netlist::parse(&text) else {
            continue;
        };
        let Ok(mut mna) = MnaSystem::from_netlist(&netlist) else {
            continue;
        };
        parsed += 1;
        let name = f.strip_prefix(&root).unwrap_or(f).display().to_string();

        // --- routing.rs's view: DK kernel + PRE-expansion MnaSystem ---
        if let Ok(kernel) = DkKernel::from_mna(&mna, 48000.0) {
            kernel_ok += 1;
            let m = kernel.m;
            if m > 0 {
                let live = |i: usize| mna.n_i.iter().any(|row| row[i].abs() >= 1e-30);
                for i in exact_zero_k_with_live_ni(&kernel.k, m, &live) {
                    zero_k_hits.push(format!("{name} [routing/DK] dim {i}"));
                }
                for (a, i, v) in ni_in_disagreement_band(&mna.n_i, m) {
                    band_hits.push(format!("{name} [routing/DK] N_i[{a}][{i}] = {v:e}"));
                }
            }
        }

        // --- nodal_emitter's view: IR built AFTER expand_bjt_internal_nodes ---
        // Stamp the input conductance the way the real pipeline does before the
        // IR is built, so the matrices match what the emitter actually sees.
        if let Some(&in_node) = mna.node_map.get("in") {
            if in_node > 0 {
                mna.g[in_node - 1][in_node - 1] += 1.0;
            }
        }
        let cfg = CodegenConfig {
            circuit_name: "f7_probe".to_string(),
            ..CodegenConfig::default()
        };
        if let Ok(ir) = CircuitIR::from_mna(&mna, &netlist, &cfg) {
            ir_ok += 1;
            let m = ir.topology.m;
            if m > 0 {
                let live = |i: usize| ir.sparsity.n_i.nz_by_row.iter().any(|row| row.contains(&i));
                for i in exact_zero_k_with_live_ni(&ir.matrices.k, m, &live) {
                    zero_k_hits.push(format!("{name} [emitter/IR] dim {i}"));
                }
            }
        }
    }

    eprintln!(
        "F7 sweep: {} .cir files, {parsed} parsed, {kernel_ok} DK kernels, {ir_ok} nodal IRs",
        files.len()
    );
    eprintln!(
        "  K[i][i] == 0.0 with a live N_i column : {} hit(s)",
        zero_k_hits.len()
    );
    for h in &zero_k_hits {
        eprintln!("    {h}");
    }
    eprintln!(
        "  |N_i| in [1e-30, 1e-20)               : {} hit(s)",
        band_hits.len()
    );
    for h in &band_hits {
        eprintln!("    {h}");
    }

    // ------------------------------------------------------------------
    // RESULT (measured 2026-09-02 over the whole library).
    //
    // The magnitude-threshold difference (1e-30 vs 1e-20) is UNREACHABLE: no
    // |N_i| entry anywhere lands in the band. That one is safe, as predicted.
    //
    // The sign-threshold difference (>= 0.0 vs > 0.0) is REACHABLE, which is
    // NOT what the plan predicted ("measure-zero, both sides already avoid it
    // by construction"). On the emitter's post-expansion IR matrices, 20
    // dimensions across 8 circuits carry K[i][i] bit-exactly 0.0 on a
    // dimension whose N_i column is LIVE — including wurli-power-amp, which
    // belongs to the only shipped product.
    //
    // Read this precisely: the two sites do NOT disagree today, because they
    // never evaluate the same matrices (routing sees the DK kernel and the
    // pre-expansion MNA; the emitter sees the IR built after
    // `expand_bjt_internal_nodes`). On the DK side the band is empty. What the
    // measurement establishes is the consequence of HARMONISING the two
    // spellings: adopting routing's `>= 0.0` in the emitter would newly flag 20
    // dimensions and move the Schur-vs-full-LU decision on 8 real circuits.
    //
    // So "the definitions differ" is not a tidiness wart to be cleaned up. It
    // is load-bearing, and this test is the evidence. Both counts are pinned:
    // if either moves, the reasoning above needs redoing before anyone touches
    // either predicate.
    // ------------------------------------------------------------------

    assert!(
        band_hits.is_empty(),
        "REGRESSION: the `1e-30` vs `1e-20` magnitude-threshold difference has \
         become reachable on {} entry/entries — some |N_i| landed in the \
         disagreement band, which needs BF > 1e20 on a .model card. This was \
         empty across the whole library when measured: {:?}",
        band_hits.len(),
        band_hits
    );

    assert_eq!(
        zero_k_hits.len(),
        EXPECTED_ZERO_K_DIMS,
        "TRIPWIRE: the number of dimensions with K[i][i] == 0.0 and a live N_i \
         column changed ({} now, {EXPECTED_ZERO_K_DIMS} when measured). That \
         count is the evidence that routing.rs's `>= 0.0` and \
         nodal_emitter.rs's `> 0.0` must NOT be harmonised — it is how many \
         dimensions would newly trip if they were. Re-derive the argument \
         before changing either predicate.\nNow: {:#?}",
        zero_k_hits.len(),
        zero_k_hits
    );
}

/// Dimensions across the library with `K[i][i]` bit-exactly `0.0` on a live
/// `N_i` column, on the emitter's post-expansion IR matrices. Measured
/// 2026-09-02 across 97 netlists: 20, spread over 8 circuits
/// (4kbuscomp x2 decks, wurli-power-amp, velvet-elvis, farfisa-g10-ref{,-3key},
/// wurli-tremolo, jeffreys-tube). Zero on the DK/routing side.
const EXPECTED_ZERO_K_DIMS: usize = 20;
