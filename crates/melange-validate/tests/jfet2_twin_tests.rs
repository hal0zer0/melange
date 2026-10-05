//! A `LEVEL=2` JFET is ngspice's JFET2 (Parker–Skellern) at zero dispersion
//! and zero thermal reduction. These twins hold melange's resolved device to
//! ngspice's: the DC law over a gate × drain grid (a VST/MVST sweep, every
//! shape key, both polarities, the gate junctions), a `.mismatch J VP` device
//! against a card carrying its jittered VTO with BETA unchanged, and a
//! transient `validate` run whose reference card comes from the translator.
//!
//! The DC grids are ngspice `.dc` sweeps of a JFET between ideal sources
//! (gmin 0, reltol 1e-9, 17 printed digits), so each point is the device law
//! itself; the pass band is 1e-9 relative plus 1e-17 A, ngspice's own solve
//! rounding near Vds = 0. The law grids run with IS=0. With the gate junctions
//! on, one residual is not the law's: melange's thermal voltage (CODATA 2018
//! constants) is 2.86e-5 above ngspice's (its const.h constants), so a junction
//! at V/(N·Vt) differs by that much times V/(N·Vt) of its current, as every
//! melange junction does.

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
    let dir = std::env::temp_dir().join(format!("melange_jfet2_{tag}_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// The JFETs melange resolves from `spice`, in element order.
fn resolved(spice: &str) -> Vec<JfetParams> {
    let netlist = Netlist::parse(spice).expect("parse");
    CircuitIR::build_device_info(&netlist)
        .expect("device info")
        .into_iter()
        .filter_map(|s| match s.params {
            DeviceParams::Jfet(jp) => Some(jp),
            _ => None,
        })
        .collect()
}

/// ngspice's `(Vgs, Vds, I_drain, I_gate)` over a gate × drain grid for
/// `model` (a whole `.model` line), polarity `s` (+1 N, −1 P).
fn ngspice_grid(model: &str, s: f64, tag: &str) -> Vec<[f64; 4]> {
    let dir = scratch(tag);
    let deck = format!(
        "jfet2 grid\nVd d 0 DC 0\nVg g 0 DC 0\nJ1 d g 0 JX\n{model}\n\
         .options gmin=0 reltol=1e-9 abstol=1e-20\n.control\nset numdgt=17\n\
         dc Vg {} {} {} Vd {} {} {}\nwrdata out.txt v(g) v(d) i(Vd) i(Vg)\n.endc\n.end\n",
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

/// melange's resolved JFET for `model` against ngspice's grid.
fn assert_twin(model: &str, tag: &str) {
    let jp = resolved(&format!("t\nJ1 d g 0 JX\nR1 d 0 1k\n{model}\n.end\n"))
        .pop()
        .unwrap();
    assert!(jp.ps.is_some(), "{model} must resolve as LEVEL=2");
    let s = if jp.is_p_channel { -1.0 } else { 1.0 };
    assert_grid(&jp, &ngspice_grid(model, s, tag), model);
}

/// melange's thermal voltage relative to ngspice's (see the module note).
const VT_RATIO_MINUS_ONE: f64 = 2.86e-5;

fn assert_grid(jp: &JfetParams, grid: &[[f64; 4]], what: &str) {
    let dev = jp.device();
    let s = if jp.is_p_channel { -1.0 } else { 1.0 };
    assert!(grid.len() >= 2475, "{what}: grid has {} points", grid.len());
    let (mut on, mut off) = (0, 0);
    for &[vgs, vds, id_ng, ig_ng] in grid {
        let (id, ig, _) = dev.evaluate(vgs, vds);
        // The junctions' share: the most forward junction's V/(N·Vt) times
        // the thermal-voltage difference, on the gate current's scale.
        let x = (s * vgs).max(s * (vgs - vds)).max(0.0) / jp.gate_n_vt();
        let junction = if jp.is > 0.0 {
            VT_RATIO_MINUS_ONE * x * (ig_ng.abs() + jp.is)
        } else {
            0.0
        };
        for (name, m, n) in [("I_drain", id, id_ng), ("I_gate", ig, ig_ng)] {
            assert!(
                (m - n).abs() <= 1e-9 * n.abs() + 1e-17 + junction,
                "{what} at Vgs={vgs} Vds={vds}: {name} melange {m:e}, ngspice {n:e}"
            );
        }
        if id_ng.abs() > 1e-9 {
            on += 1;
        } else {
            off += 1;
        }
    }
    assert!(
        on > 800 && off > 100,
        "{what}: grid missed a region (on {on}, off {off})"
    );
}

#[test]
#[ignore = "requires ngspice"]
fn level2_law_matches_jfet2_across_a_vst_mvst_sweep() {
    need_ngspice();
    for vst in ["0", "0.01", "0.026", "0.05", "0.078"] {
        for mvst in ["0", "0.5"] {
            assert_twin(
                &format!(
                    ".model JX NJF(LEVEL=2 VTO=-2.5 BETA=1.6e-3 LAMBDA=4e-3 IS=0 VST={vst} \
                     MVST={mvst})"
                ),
                &format!("vst{vst}_{mvst}"),
            );
        }
    }
}

#[test]
#[ignore = "requires ngspice"]
fn level2_law_matches_jfet2_with_every_shape_key() {
    need_ngspice();
    assert_twin(
        ".model JX NJF(LEVEL=2 VTO=-2.5 BETA=1.6e-3 LAMBDA=4e-3 IS=0 VST=0.05 MVST=0.3 P=2.5 \
         Q=1.8 Z=0.3 XI=3 MXI=0.2 PB=0.8)",
        "keys",
    );
    // P-channel, SPICE-convention VTO.
    assert_twin(
        ".model JX PJF(LEVEL=2 VTO=-1.5 BETA=2e-3 LAMBDA=0.01 IS=0 VST=0.04 MVST=0.2 P=2.2 \
         Q=1.7 Z=0.5 XI=5 MXI=0.1 PB=0.9)",
        "pjf",
    );
}

/// The gate junctions, with an emission coefficient, which ngspice's JFET2
/// has (level 1 does not).
#[test]
#[ignore = "requires ngspice"]
fn level2_gate_junctions_match_jfet2() {
    need_ngspice();
    assert_twin(
        ".model JX NJF(LEVEL=2 VTO=-2.5 BETA=1.6e-3 LAMBDA=4e-3 VST=0.026 IS=1e-14 N=1.3)",
        "gate",
    );
    assert_twin(
        ".model JX PJF(LEVEL=2 VTO=-1.5 BETA=2e-3 LAMBDA=0.01 VST=0.04 IS=1e-14)",
        "pgate",
    );
}

/// `.mismatch J VP` moves a LEVEL=2 device's pinch-off and holds BETA, so
/// each jittered device is ngspice's card with that VTO and the card's BETA.
#[test]
#[ignore = "requires ngspice"]
fn a_vp_mismatched_level2_device_is_jfet2_at_its_jittered_vto() {
    need_ngspice();
    let deck = "mismatch\nR1 in d 27k\nJ1 d g 0 JX\nJ2 d g 0 JX\nRG g 0 1Meg\n\
                .model JX NJF(LEVEL=2 VTO=-2.5 BETA=1.6e-3 LAMBDA=4e-3 VST=0.026 IS=0)\n\
                .mismatch J VP=0.05\n.seed 11\n.end\n";
    let jfets = resolved(deck);
    assert_eq!(jfets.len(), 2);
    for (k, jp) in jfets.iter().enumerate() {
        assert_eq!(
            jp.ps.unwrap().beta,
            1.6e-3,
            "J{} BETA must be the card's",
            k + 1
        );
        assert!(jp.vp != -2.5, "J{} VP must be jittered", k + 1);
        let card = format!(
            ".model JX NJF(LEVEL=2 VTO={:e} BETA=1.6e-3 LAMBDA=4e-3 VST=0.026 IS=0)",
            jp.vp
        );
        assert_grid(jp, &ngspice_grid(&card, 1.0, &format!("mm{k}")), &card);
    }
}

/// A transient twin: a LEVEL=2 voltage-variable resistor swung through
/// cut-off, subthreshold, conduction and reverse mode, whose ngspice reference
/// card is the translator's JFET2 card. At 192 kHz, where the step-size
/// residue is small: the level-1 version of this deck measures 2.4e-4 there
/// (4.7e-3 at 48 kHz), and level 2 converges with the step the same way.
#[test]
#[ignore = "requires ngspice"]
fn a_level2_vvr_validates_against_jfet2() {
    need_ngspice();
    const FS: f64 = 192000.0;
    let dir = scratch("vvr");
    let path = dir.join("vvr.cir");
    std::fs::write(
        &path,
        "jfet2 vvr\nR1 in out 27k\nJ1 out g 0 JX\nRA out g 1Meg\nRB g b 1Meg\nVB b 0 DC -5.2\n\
         C1 out 0 1n\n.model JX NJF(LEVEL=2 VTO=-2.5 BETA=1.6m LAMBDA=4m VST=0.026 MVST=0.2)\n\
         .end\n",
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
