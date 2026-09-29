//! Generated code compiles warning-free under `-D warnings`, as users' plugin
//! crates build it. Each build of a small diode stage is compiled with a `main`
//! that drives it: nodal Schur and full-LU, trapezoidal with and without
//! `--force-trap` (which drops the latch and so the forced-BE branch), and DK.

mod support;

use std::process::Command;

use melange_solver::codegen::{CodegenConfig, NodalSubPathOverride};

const DECK: &str = "diode stage\nR1 in out 1k\nC1 out 0 100n\nD1 out 0 DX\n.model DX D(IS=1e-14)\n";

const MAIN: &str = "fn main() {
    let mut s = CircuitState::default();
    for i in 0..64 { let _ = process_sample((i as f64 * 0.01).sin(), &mut s); }
}";

fn assert_warning_free(code: &str, tag: &str) {
    let dir = std::env::temp_dir();
    let id = format!("{tag}_{}", std::process::id());
    let src = dir.join(format!("melange_warn_{id}.rs"));
    let bin = dir.join(format!("melange_warn_{id}"));
    std::fs::write(&src, format!("{code}\n\n{MAIN}\n")).unwrap();
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
    }
    let c = config(|c| c.force_trap = true);
    let code = support::generate_circuit_code(DECK, &c).0;
    assert_warning_free(&code, "dk");
}
