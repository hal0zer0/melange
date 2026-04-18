//! Tests for WARMUP_SAMPLES_RECOMMENDED (Oomox P5).

use melange_solver::codegen::{CodeGenerator, CodegenConfig};
use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn generate_at(spice: &str, sample_rate: f64) -> String {
    let netlist = Netlist::parse(spice).expect("parse");
    let mut mna = MnaSystem::from_netlist(&netlist).expect("mna");
    if mna.n > 0 {
        mna.g[0][0] += 1.0;
    }
    let kernel = DkKernel::from_mna(&mna, sample_rate).expect("kernel");
    let cfg = CodegenConfig {
        circuit_name: "warmup_test".to_string(),
        sample_rate,
        input_node: 0,
        output_nodes: vec![if kernel.n > 1 { 1 } else { 0 }],
        input_resistance: 1.0,
        ..CodegenConfig::default()
    };
    CodeGenerator::new(cfg)
        .generate(&kernel, &mna, &netlist)
        .expect("codegen")
        .code
}

fn extract_warmup_samples(code: &str) -> usize {
    let line = code
        .lines()
        .find(|l| l.contains("pub const WARMUP_SAMPLES_RECOMMENDED"))
        .unwrap_or_else(|| panic!("missing WARMUP_SAMPLES_RECOMMENDED in:\n{}", code));
    line.split_once('=')
        .and_then(|(_, rhs)| rhs.trim().trim_end_matches(';').parse::<usize>().ok())
        .unwrap_or_else(|| panic!("could not parse warmup const: {}", line))
}

#[test]
fn warmup_const_emitted_always() {
    // Even a trivial resistive circuit gets a const (clamped to ≥1).
    let spice = "\
Trivial RC
R1 in out 10k
C1 out 0 1n
";
    let code = generate_at(spice, 44100.0);
    let n = extract_warmup_samples(&code);
    assert!(n >= 1, "warmup samples must be ≥1, got {}", n);
}

#[test]
fn warmup_scales_with_rc_time_constant() {
    // Larger coupling cap → slower pole → more warmup samples.
    // Compare 100 nF vs 10 µF on the same topology.
    let fast = generate_at(
        "\
Fast RC
R1 in out 10k
C1 out 0 100n
",
        44100.0,
    );
    let slow = generate_at(
        "\
Slow RC
R1 in out 10k
C1 out 0 10u
",
        44100.0,
    );
    let n_fast = extract_warmup_samples(&fast);
    let n_slow = extract_warmup_samples(&slow);
    // Slow circuit τ = 10k · 10µF = 100 ms ≫ fast τ = 10k · 100 nF = 1 ms.
    // At 44.1 kHz, slow warmup should be at least 50× larger.
    assert!(
        n_slow >= 50 * n_fast,
        "slow ({}) should be ≥50× fast ({}) samples",
        n_slow, n_fast
    );
}

#[test]
fn warmup_scales_with_sample_rate() {
    // Same circuit at 2× sample rate → 2× warmup samples.
    let spice = "\
Rate Scale
R1 in out 10k
C1 out 0 1u
";
    let n_48k = extract_warmup_samples(&generate_at(spice, 48000.0));
    let n_96k = extract_warmup_samples(&generate_at(spice, 96000.0));
    // Allow ±1 sample of rounding slack.
    assert!(
        n_96k.abs_diff(2 * n_48k) <= 2,
        "expected ~2× scaling: 48k={} 96k={}",
        n_48k, n_96k
    );
}

#[test]
fn warmup_const_compiles_and_is_positive_usize() {
    // Roundtrip check: generated code compiles and the const is usable.
    use std::io::Write;
    let spice = "\
Warmup Compile
R1 in out 10k
C1 out 0 100n
";
    let code = generate_at(spice, 44100.0);
    let main = "\n\nfn main() {\n\
        let n: usize = WARMUP_SAMPLES_RECOMMENDED;\n\
        assert!(n >= 1);\n\
        println!(\"warmup={}\", n);\n\
    }\n";
    let full = format!("{}{}", code, main);

    let tmp = std::env::temp_dir();
    let src = tmp.join("melange_warmup_samples.rs");
    let bin = tmp.join("melange_warmup_samples");
    std::fs::File::create(&src)
        .unwrap()
        .write_all(full.as_bytes())
        .unwrap();
    let compile = std::process::Command::new("rustc")
        .args([
            src.to_str().unwrap(),
            "-o",
            bin.to_str().unwrap(),
            "--edition",
            "2021",
        ])
        .output()
        .expect("rustc");
    let _ = std::fs::remove_file(&src);
    if !compile.status.success() {
        let _ = std::fs::remove_file(&bin);
        panic!(
            "compile failed:\n{}",
            String::from_utf8_lossy(&compile.stderr)
        );
    }
    let run = std::process::Command::new(&bin).output().expect("run");
    let _ = std::fs::remove_file(&bin);
    assert!(
        run.status.success(),
        "run failed:\n{}",
        String::from_utf8_lossy(&run.stderr)
    );
}
