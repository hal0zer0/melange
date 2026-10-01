//! Generated code compiles warning-free under `-D warnings`, as users' plugin
//! crates build it. Each build of a small diode stage is compiled with a `main`
//! that drives it: nodal Schur and full-LU, trapezoidal with and without
//! `--force-trap` (which drops the latch and so the forced-BE branch), backward
//! Euler, and DK. A clamped op-amp stage covers the active-set rail modes on
//! both nodal routes, with and without a nonlinear device, and every
//! `--emit-dc-op-recompute` emission: the hard-rail pin loop on DK (linear and
//! nonlinear), the unpinned recompute on DK, and the nodal active-set build.

mod support;

use std::process::Command;

use melange_solver::codegen::{CodegenConfig, NodalSubPathOverride, OpampRailMode};

const DECK: &str = "diode stage\nR1 in out 1k\nC1 out 0 100n\nD1 out 0 DX\n.model DX D(IS=1e-14)\n";

const MAIN: &str = "fn main() {
    let mut s = CircuitState::default();
    for i in 0..64 { let _ = process_sample((i as f64 * 0.01).sin(), &mut s); }
}";

fn assert_warning_free(code: &str, tag: &str) {
    assert_warning_free_with(code, MAIN, tag);
}

fn assert_warning_free_with(code: &str, main: &str, tag: &str) {
    let dir = support::scratch_dir();
    let id = format!("{tag}_{}", std::process::id());
    let src = dir.join(format!("melange_warn_{id}.rs"));
    let bin = dir.join(format!("melange_warn_{id}"));
    std::fs::write(&src, format!("{code}\n\n{main}\n")).unwrap();
    let out = Command::new("rustc")
        .arg(&src)
        .args(["-o"])
        .arg(&bin)
        .args(["--edition=2024", "-D", "warnings"])
        .output()
        .expect("rustc");
    let _ = std::fs::remove_file(&src);
    let _ = std::fs::remove_file(&bin);
    assert!(
        out.status.success(),
        "{tag}: generated code is not warning-free:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

fn config(f: impl FnOnce(&mut CodegenConfig)) -> CodegenConfig {
    let mut c = support::config_for_spice(DECK, 48000.0);
    f(&mut c);
    c
}

#[test]
fn generated_code_is_warning_free() {
    for (sub_path, sp) in [
        (NodalSubPathOverride::Schur, "schur"),
        (NodalSubPathOverride::FullLu, "full_lu"),
    ] {
        for force_trap in [false, true] {
            let c = config(|c| {
                c.nodal_sub_path_override = sub_path;
                c.force_trap = force_trap;
            });
            let code = support::generate_circuit_code_nodal(DECK, &c).0;
            assert_warning_free(&code, &format!("nodal_{sp}_ft{force_trap}"));
        }
        let c = config(|c| {
            c.nodal_sub_path_override = sub_path;
            c.backward_euler = true;
        });
        let code = support::generate_circuit_code_nodal(DECK, &c).0;
        assert_warning_free(&code, &format!("nodal_{sp}_be"));
    }
    let c = config(|c| c.force_trap = true);
    let code = support::generate_circuit_code(DECK, &c).0;
    assert_warning_free(&code, "dk");
    // DK with no nonlinear device (M = 0).
    let rc = "rc\nR1 in out 1k\nC1 out 0 100n\n";
    let mut c = support::config_for_spice(rc, 48000.0);
    c.force_trap = true;
    let code = support::generate_circuit_code(rc, &c).0;
    assert_warning_free(&code, "dk_m0");
}

/// Active-set rail handling (pin detection, pinned resolve) on both nodal
/// routes, on a linear op-amp stage (M = 0) and with a clipping diode (M = 1).
#[test]
fn active_set_opamp_code_is_warning_free() {
    let linear = "inverting x20\nR1 in inv 1k\nR2 inv out 20k\nU1 0 inv out OX\nRl out 0 10k\n\
                  .model OX OA(AOL=100000 VCC=15 VEE=-15)\n";
    let diode = "inverting x20 clipped\nR1 in inv 1k\nR2 inv out 20k\nU1 0 inv out OX\n\
                 D1 out 0 DX\nRl out 0 10k\n\
                 .model OX OA(AOL=100000 VCC=15 VEE=-15)\n.model DX D(IS=1e-14)\n";
    for (deck, dtag) in [(linear, "m0"), (diode, "m1")] {
        for mode in [OpampRailMode::ActiveSet, OpampRailMode::ActiveSetBe] {
            for (sub_path, sp) in [
                (NodalSubPathOverride::Schur, "schur"),
                (NodalSubPathOverride::FullLu, "full_lu"),
            ] {
                let mut c = support::config_for_spice(deck, 48000.0);
                c.opamp_rail_mode = mode;
                c.nodal_sub_path_override = sub_path;
                let code = support::generate_circuit_code_nodal(deck, &c).0;
                assert_warning_free(&code, &format!("opamp_{dtag}_{mode:?}_{sp}"));
            }
        }
    }
}

/// The runtime DC-OP recompute (`--emit-dc-op-recompute`) is emitted code no
/// verb compiles until a plugin does: compile each form, called from `main`.
#[test]
fn dc_op_recompute_code_is_warning_free() {
    const RECOMPUTE_MAIN: &str = "fn main() {
    let mut s = CircuitState::default();
    s.recompute_dc_op();
    for i in 0..64 { let _ = process_sample((i as f64 * 0.01).sin(), &mut s); }
}";
    let linear = "inverting x20\nR1 in inv 1k\nR2 inv out 20k\nU1 0 inv out OX\nRl out 0 10k\n\
                  .model OX OA(AOL=100000 VCC=15 VEE=-15)\n";
    let diode = "inverting x20 clipped\nR1 in inv 1k\nR2 inv out 20k\nU1 0 inv out OX\n\
                 D1 out 0 DX\nRl out 0 10k\n\
                 .model OX OA(AOL=100000 VCC=15 VEE=-15)\n.model DX D(IS=1e-14)\n";
    for (deck, dtag) in [(linear, "m0"), (diode, "m1")] {
        // DK: the hard-rail pin loop, and the recompute without one.
        for mode in [OpampRailMode::Hard, OpampRailMode::None] {
            let mut c = support::config_for_spice(deck, 48000.0);
            c.opamp_rail_mode = mode;
            c.emit_dc_op_recompute = true;
            c.force_trap = true;
            let code = support::generate_circuit_code(deck, &c).0;
            assert!(code.contains("pub fn recompute_dc_op"), "{dtag} {mode:?}");
            if mode == OpampRailMode::Hard {
                assert!(code.contains("let next_pins"), "{dtag}: the pin loop");
            }
            assert_warning_free_with(
                &code,
                RECOMPUTE_MAIN,
                &format!("recompute_dk_{dtag}_{mode:?}"),
            );
        }
        // Nodal (active-set routes nodal; DK refuses it).
        let mut c = support::config_for_spice(deck, 48000.0);
        c.opamp_rail_mode = OpampRailMode::ActiveSet;
        c.emit_dc_op_recompute = true;
        let code = support::generate_circuit_code_nodal(deck, &c).0;
        assert!(code.contains("pub fn recompute_dc_op"), "{dtag} nodal");
        assert_warning_free_with(&code, RECOMPUTE_MAIN, &format!("recompute_nodal_{dtag}"));
    }
}
