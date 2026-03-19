use melange_solver::dk::DkKernel;
use melange_solver::mna::MnaSystem;
use melange_solver::parser::Netlist;

fn main() {
    let spice = r#"* Mordor Screamer
.model D1N4148 D(IS=2.52e-9 RS=0.568 N=1.906)
C1 in n1 0.1u
R1 n1 0 1meg
R2 n1 n2 10k
C2 n2 n3 0.047u
D1 n3 n2 D1N4148
D2 n2 n3 D1N4148
R3 n3 n4 1k
C3 n4 0 0.022u
R4 n4 out 10k
C4 out 0 0.1u
Vin in 0 0
.END"#;

    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();

    // Add input conductance
    mna.g[0][0] += 1.0;

    let sample_rate = 48000.0;
    let kernel = DkKernel::from_mna(&mna, sample_rate).unwrap();

    println!("K matrix:");
    for i in 0..kernel.m {
        print!("  ");
        for j in 0..kernel.m {
            print!("{:12.4} ", kernel.k[i * kernel.m + j]);
        }
        println!();
    }

    // Test with a simple RC circuit (no diodes) for comparison
    println!("\n=== Comparison: Simple RC ===");
    let spice2 = r#"RC Circuit
R1 in out 10k
C1 out 0 0.1u
Vin in 0 0
.END"#;
    let netlist2 = Netlist::parse(spice2).unwrap();
    let mut mna2 = MnaSystem::from_netlist(&netlist2).unwrap();
    mna2.g[0][0] += 1.0;
    let kernel2 = DkKernel::from_mna(&mna2, sample_rate).unwrap();

    println!(
        "S diagonal: {:?}",
        (0..kernel2.n)
            .map(|i| kernel2.s[i * kernel2.n + i])
            .collect::<Vec<_>>()
    );
}
