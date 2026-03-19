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

    println!("Before adding input conductance:");
    println!("G[0][0] = {}", mna.g[0][0]);

    mna.g[0][0] += 1.0;

    println!("\nAfter adding input conductance:");
    println!("G[0][0] = {}", mna.g[0][0]);

    let sample_rate = 48000.0;
    let alpha = 2.0 * sample_rate;

    println!("\nAlpha = {}", alpha);
    println!("C[0][0] = {}", mna.c[0][0]);
    println!("C[0][1] = {}", mna.c[0][1]);

    let a_neg = mna.get_a_neg_matrix(sample_rate);
    println!("\nA_neg[0][0] = {}", a_neg[0][0]);
    println!("A_neg[0][1] = {}", a_neg[0][1]);

    let kernel = DkKernel::from_mna(&mna, sample_rate).unwrap();
    println!("\nS[0][0] = {}", kernel.s[0]);
}
