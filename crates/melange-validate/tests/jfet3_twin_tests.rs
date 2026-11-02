//! A `LEVEL=3` JFET (melange's symmetric subthreshold law) has no ngspice
//! law, so its reference is the translator's `.subckt` of B-sources
//! transcribing the generated code's formulas (`jfet_translate.rs`). These
//! twins hold melange's resolved device to that transcription in ngspice: the
//! DC law over a gate × drain grid (both polarities, two slope factors, the
//! gate junctions on and off), and a transient `validate` run of the 1176-style
//! voltage-variable resistor. The twin checks melange's solve of the law, not
//! the law's physics (see DEVICE_MODELS.md, "Level 3").
//!
//! The DC grids are ngspice `.dc` sweeps of the subcircuit between ideal
//! sources (reltol 1e-9, 17 printed digits), so each point is the B-source
//! expression itself, evaluated by ngspice's own libm in its own operation
//! order; the pass band is 1e-9 relative plus 1e-17 A. Because the junctions
//! are the same formula on both sides (not an ngspice `D` element), the
//! grid with IS on has no region to exclude and no thermal-voltage allowance.

use melange_solver::codegen::ir::{CircuitIR, DeviceParams, JfetParams};
use melange_solver::parser::Netlist;
use melange_validate::spice_runner::is_ngspice_available;
use melange_validate::{validate_circuit_with_options, ComparisonConfig, ValidationOptions};

fn need_ngspice() {
    assert!(
        is_ngspice_available(),
        "ngspice not found: this test needs it (run without --include-ignored to skip)"
    );
}

fn scratch(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("melange_jfet3_{tag}_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// A one-JFET deck for `model` (a whole `.model` line).
fn element_deck(model: &str) -> String {
    format!("t\nJ1 d g 0 JX\nR1 d 0 1k\nRg g 0 1k\n{model}\n.end\n")
}

/// The JFET melange resolves from `model`.
fn resolved(model: &str) -> JfetParams {
    let netlist = Netlist::parse(&element_deck(model)).expect("parse");
    CircuitIR::build_device_info(&netlist)
        .expect("device info")
        .into_iter()
        .find_map(|s| match s.params {
            DeviceParams::Jfet(jp) => Some(jp),
            _ => None,
        })
        .expect("one JFET")
}

/// The translator's `.subckt` for `model`, as it reaches ngspice.
fn subckt(model: &str) -> String {
    let deck = element_deck(model);
    let out = melange_validate::translate_jfets_for_ngspice(&deck, &deck).unwrap();
    assert!(
        !out.to_ascii_uppercase().contains("LEVEL=3"),
        "LEVEL=3 must never reach ngspice:\n{out}"
    );
    let start = out.find(".subckt MELANGE_JFET3_JX").expect("subckt");
    let end = out[start..].find(".ends\n").expect("ends") + start + ".ends\n".len();
    out[start..end].to_string()
}

/// ngspice's `(Vgs, Vds, I_drain, I_gate)` over a gate × drain grid for the
/// subcircuit of `model`, polarity `s` (+1 N, −1 P).
fn ngspice_grid(model: &str, s: f64, tag: &str) -> Vec<[f64; 4]> {
    let dir = scratch(tag);
    let deck = format!(
        "jfet3 grid\nVd d 0 DC 0\nVg g 0 DC 0\nXJ1 d g 0 MELANGE_JFET3_JX\n{}\
         .options reltol=1e-9 abstol=1e-20\n.control\nset numdgt=17\n\
         dc Vg {} {} {} Vd {} {} {}\nwrdata out.txt v(g) v(d) i(Vd) i(Vg)\n.endc\n.end\n",
        subckt(model),
        -3.3 * s,
        0.4 * s,
        0.05 * s,
        -4.0 * s,
        4.0 * s,
        0.25 * s
    );
    std::fs::write(dir.join("grid.cir"), deck).unwrap();
    let run = std::process::Command::new("ngspice")
        .args(["-b", "grid.cir"])
        .current_dir(&dir)
        .output()
        .expect("run ngspice");
    let text = std::fs::read_to_string(dir.join("out.txt")).unwrap_or_else(|_| {
        panic!(
            "ngspice wrote no grid for {model}:\n{}",
            String::from_utf8_lossy(&run.stderr)
        )
    });
    let _ = std::fs::remove_dir_all(&dir);
    text.lines()
        .map(|l| {
            let v: Vec<f64> = l.split_whitespace().map(|x| x.parse().unwrap()).collect();
            // wrdata: (sweep, v(g)), (sweep, v(d)), (sweep, i(Vd)), (sweep, i(Vg));
            // a source's current flows into its + node, so the device's
            // drain and gate currents are their negatives.
            [v[1], v[3], -v[5], -v[7]]
        })
        .collect()
}

/// melange's resolved JFET for `model` against ngspice's grid of its twin.
fn assert_twin(model: &str, tag: &str) {
    let jp = resolved(model);
    assert!(jp.sym.is_some(), "{model} must resolve as LEVEL=3");
    let s = if jp.is_p_channel { -1.0 } else { 1.0 };
    let grid = ngspice_grid(model, s, tag);
    let dev = jp.device();
    assert!(
        grid.len() >= 2475,
        "{model}: grid has {} points",
        grid.len()
    );
    let (mut on, mut tail) = (0, 0);
    for &[vgs, vds, id_ng, ig_ng] in &grid {
        let (id, ig, _) = dev.evaluate(vgs, vds);
        for (name, m, n) in [("I_drain", id, id_ng), ("I_gate", ig, ig_ng)] {
            assert!(
                (m - n).abs() <= 1e-9 * n.abs() + 1e-17,
                "{model} at Vgs={vgs} Vds={vds}: {name} melange {m:e}, ngspice {n:e}"
            );
        }
        if id_ng.abs() > 1e-9 {
            on += 1;
        } else {
            tail += 1;
        }
    }
    assert!(
        on > 800 && tail > 100,
        "{model}: grid missed a region (on {on}, tail {tail})"
    );
}

#[test]
#[ignore = "requires ngspice"]
fn level3_law_matches_its_transcription_at_two_slopes() {
    need_ngspice();
    // The 1176 refit card (n = 1), and the sensitivity value n = 1.2.
    assert_twin(
        ".model JX NJF(LEVEL=3 VTO=-2.5 BETA=1.6e-3 LAMBDA=4e-3 IS=0)",
        "n1",
    );
    assert_twin(
        ".model JX NJF(LEVEL=3 VTO=-2.5 BETA=1.6e-3 LAMBDA=4e-3 IS=0 NSLOPE=1.2)",
        "n12",
    );
    // P-channel, SPICE-convention VTO, IDSS given.
    assert_twin(
        ".model JX PJF(LEVEL=3 VTO=-1.5 IDSS=4.5e-3 LAMBDA=0.01 IS=0)",
        "pjf",
    );
}

/// The gate junctions, on both sides melange's `junction_exp`, so the grid
/// holds at the 1e-17 A floor through reverse bias (where an ngspice `D`
/// would not) and into forward conduction.
#[test]
#[ignore = "requires ngspice"]
fn level3_gate_junctions_match_their_transcription() {
    need_ngspice();
    assert_twin(
        ".model JX NJF(LEVEL=3 VTO=-2.5 BETA=1.6e-3 LAMBDA=4e-3 IS=1e-14 N=1.3)",
        "gate",
    );
    assert_twin(
        ".model JX PJF(LEVEL=3 VTO=-1.5 BETA=2e-3 LAMBDA=0.01 IS=1e-14)",
        "pgate",
    );
}

/// A transient twin: the 1176-style voltage-variable resistor, biased 0.1 V
/// below pinch-off with its gate fed half the drain swing, driven through the
/// thermal tail, the knee, conduction and both signs of Vds, whose ngspice
/// reference is the translator's subcircuit. At 192 kHz, as the level-2 twin
/// (the level-1 version of this deck measures 2.4e-4 there).
#[test]
#[ignore = "requires ngspice"]
fn a_level3_vvr_validates_against_its_transcription() {
    need_ngspice();
    const FS: f64 = 192000.0;
    let dir = scratch("vvr");
    let path = dir.join("vvr.cir");
    std::fs::write(
        &path,
        "jfet3 vvr\nR1 in out 27k\nJ1 out g 0 JX\nRA out g 1Meg\nRB g b 1Meg\nVB b 0 DC -5.2\n\
         C1 out 0 1n\n.model JX NJF(LEVEL=3 VTO=-2.5 BETA=1.6m LAMBDA=4m)\n.end\n",
    )
    .unwrap();
    let input: Vec<f64> = (0..(0.05 * FS) as usize)
        .map(|k| 2.0 * (2.0 * std::f64::consts::PI * 1000.0 * k as f64 / FS).sin())
        .collect();
    let options = ValidationOptions {
        generate_html_on_failure: false,
        output_dir: Some(dir.clone()),
        ..ValidationOptions::default()
    };
    let result = validate_circuit_with_options(
        &path,
        &input,
        FS,
        "out",
        &ComparisonConfig::strict(),
        &options,
    )
    .unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    let report = &result.report;
    assert!(
        report.normalized_rms_error < 5e-4,
        "normalized rms error {:.3e}:\n{}",
        report.normalized_rms_error,
        report.summary()
    );
}
