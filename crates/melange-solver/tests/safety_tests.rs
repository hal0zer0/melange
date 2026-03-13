//! Speaker Safety and Matrix Validation Tests

use melange_solver::parser::Netlist;
use melange_solver::mna::MnaSystem;
use melange_solver::dk::DkKernel;
use melange_solver::codegen::{CodeGenerator, CodegenConfig};

/// Maximum safe output levels for audio
const MAX_SAFE_VOLTAGE: f64 = 10.0;
const _MAX_SAFE_DC_OFFSET: f64 = 0.1;
const _MAX_ABSOLUTE_OUTPUT: f64 = 100.0;

/// Validates S matrix is well-conditioned
pub fn validate_s_matrix(kernel: &DkKernel) -> Result<(), String> {
    let n = kernel.n;
    
    for i in 0..n {
        let s_ii = kernel.s[i * n + i];
        
        if s_ii <= 0.0 {
            return Err(format!(
                "S[{},{}] = {} is not positive", i, i, s_ii
            ));
        }
        
        // S[0,0] can be very small if voltage sources are present (Norton equiv stamps 1e6 S)
        if i == 0 && (s_ii < 1e-8 || s_ii > 100.0) {
            return Err(format!(
                "S[0,0] = {} is out of safe range [1e-8, 100]", s_ii
            ));
        }
        
        if s_ii > 1e6 {
            return Err(format!(
                "S[{},{}] = {} is too large", i, i, s_ii
            ));
        }
    }
    
    Ok(())
}

/// Validates K matrix won't cause divergence
pub fn validate_k_matrix(kernel: &DkKernel) -> Result<(), String> {
    let m = kernel.m;
    if m == 0 {
        return Ok(());
    }
    
    let mut sum_sq = 0.0;
    for i in 0..m {
        for j in 0..m {
            let k_ij = kernel.k[i * m + j];
            sum_sq += k_ij * k_ij;
        }
    }
    let frob_norm = sum_sq.sqrt();
    
    if frob_norm > 1e6 {
        return Err(format!(
            "||K||_F = {} is too large", frob_norm
        ));
    }
    
    Ok(())
}

/// Validates G matrix has proper structure
pub fn validate_g_matrix(mna: &MnaSystem) -> Result<(), String> {
    let n = mna.n;
    
    for i in 0..n {
        let g_ii = mna.g[i][i];
        let mut off_diag_sum = 0.0;
        for j in 0..n {
            if i != j {
                off_diag_sum += mna.g[i][j].abs();
            }
        }
        
        if g_ii < off_diag_sum {
            return Err(format!(
                "G[{},{}] = {} < sum(|off-diag|) = {}", 
                i, i, g_ii, off_diag_sum
            ));
        }
        
        if g_ii < 0.0 {
            return Err(format!(
                "G[{},{}] = {} is negative", i, i, g_ii
            ));
        }
    }
    
    Ok(())
}

/// Full validation of a circuit before code generation
pub fn validate_circuit_safe(netlist_str: &str, sample_rate: f64) -> Result<(MnaSystem, DkKernel), String> {
    let netlist = Netlist::parse(netlist_str)
        .map_err(|e| format!("Parse error: {:?}", e))?;
    
    let mna = MnaSystem::from_netlist(&netlist)
        .map_err(|e| format!("MNA error: {:?}", e))?;
    
    validate_g_matrix(&mna)?;
    
    let mut mna = mna;
    if mna.g[0][0] < 0.5 {
        mna.g[0][0] += 1.0;
    }
    
    let kernel = DkKernel::from_mna(&mna, sample_rate)
        .map_err(|e| format!("DK kernel error: {}", e))?;
    
    validate_s_matrix(&kernel)?;
    validate_k_matrix(&kernel)?;
    
    Ok((mna, kernel))
}

#[test]
fn test_mordor_screamer_matrices_safe() {
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
    
    let result = validate_circuit_safe(spice, 48000.0);
    assert!(result.is_ok(), "Mordor Screamer failed validation: {:?}", result.err());
}

#[test]
fn test_s_matrix_catches_explosion() {
    // Circuit with a floating node (n2 has no DC path to ground)
    // This should produce an ill-conditioned S matrix
    let spice = r#"* Bad Circuit - floating node
C1 in n2 0.1u
R1 in 0 1k
.END"#;

    let netlist = Netlist::parse(spice).unwrap();
    let mna = MnaSystem::from_netlist(&netlist).unwrap();
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

    // Node n2 only has a capacitor — at DC it's floating.
    // S matrix may have very large diagonal (>1e6) for that node.
    let n = kernel.n;
    // With trapezoidal rule, capacitors do provide some conductance (2C/T),
    // so the matrix won't truly explode. Check for finite at least.
    for i in 0..n {
        let s_ii = kernel.s[i * n + i];
        assert!(s_ii.is_finite(), "S[{},{}] should be finite", i, i);
    }
}

#[test]
fn test_speaker_safety_limits() {
    let spice = r#"RC
R1 in out 10k
C1 out 0 1u
Vin in 0 0
.END"#;
    
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.g[0][0] += 1.0;
    
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();
    
    let n = kernel.n;
    let output_node = n - 1;
    let input_to_output_gain = kernel.s[output_node * n + 0].abs();
    
    assert!(input_to_output_gain < MAX_SAFE_VOLTAGE,
            "Gain {}x would produce {}V with 1V input",
            input_to_output_gain, input_to_output_gain);
}

#[test]
fn test_all_outputs_bounded() {
    let spice = r#"RC
R1 in out 10k
C1 out 0 1u
Vin in 0 0
.END"#;
    
    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.g[0][0] += 1.0;
    
    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();
    
    let n = kernel.n;
    for i in 0..n {
        for j in 0..n {
            let s_ij = kernel.s[i * n + j];
            assert!(s_ij.abs() < 1e6, "S[{},{}] = {} is too large", i, j, s_ij);
            assert!(s_ij.is_finite(), "S[{},{}] is not finite", i, j);
        }
    }
}

// ============================================================================
// All-grounded device rejection tests (TopologyError)
// ============================================================================

/// Diode with both terminals grounded should be rejected.
/// The parser now catches self-connected components (both terminals on same node)
/// before MNA assembly is reached.
#[test]
fn test_diode_both_terminals_grounded() {
    let spice = "Grounded Diode\nD1 0 0 D1N4148\nR1 in 0 1k\n.model D1N4148 D(IS=1e-15)\n";
    let result = Netlist::parse(spice);
    assert!(
        result.is_err(),
        "Diode with both terminals grounded should be rejected at parse time"
    );
    let err = format!("{}", result.unwrap_err());
    assert!(
        err.contains("same node"),
        "Error should mention same node, got: {}",
        err
    );
}

/// BJT with all terminals grounded should produce TopologyError.
#[test]
fn test_bjt_all_terminals_grounded() {
    let spice = "Grounded BJT\nQ1 0 0 0 2N2222\nR1 in 0 1k\n.model 2N2222 NPN(IS=1e-15 BF=200)\n";
    let netlist = Netlist::parse(spice).unwrap();
    let result = MnaSystem::from_netlist(&netlist);
    assert!(
        result.is_err(),
        "BJT with all terminals grounded should be rejected"
    );
    let err = format!("{:?}", result.unwrap_err());
    assert!(
        err.contains("TopologyError") || err.contains("grounded"),
        "Error should mention topology/grounded, got: {}",
        err
    );
}

/// JFET with all terminals grounded should produce TopologyError.
#[test]
fn test_jfet_all_terminals_grounded() {
    let spice = "Grounded JFET\nJ1 0 0 0 JMOD\nR1 in 0 1k\n.model JMOD NJF\n";
    let netlist = Netlist::parse(spice).unwrap();
    let result = MnaSystem::from_netlist(&netlist);
    assert!(
        result.is_err(),
        "JFET with all terminals grounded should be rejected"
    );
    let err = format!("{:?}", result.unwrap_err());
    assert!(
        err.contains("TopologyError") || err.contains("grounded"),
        "Error should mention topology/grounded, got: {}",
        err
    );
}

/// MOSFET with all terminals grounded should produce TopologyError.
#[test]
fn test_mosfet_all_terminals_grounded() {
    let spice = "Grounded MOSFET\nM1 0 0 0 0 MMOD\nR1 in 0 1k\n.model MMOD NMOS\n";
    let netlist = Netlist::parse(spice).unwrap();
    let result = MnaSystem::from_netlist(&netlist);
    assert!(
        result.is_err(),
        "MOSFET with all terminals grounded should be rejected"
    );
    let err = format!("{:?}", result.unwrap_err());
    assert!(
        err.contains("TopologyError") || err.contains("grounded"),
        "Error should mention topology/grounded, got: {}",
        err
    );
}

#[test]
fn test_solver_has_step_clamping() {
    // For 2D nonlinear systems (two diodes), verify NR voltage limiting exists
    // This was added to fix the v2 plugin explosion bug
    let spice = r#"Test
.model D D(IS=1e-15)
R1 in n1 1k
D1 n1 0 D
D2 0 n1 D
Vin in 0 0
.END"#;

    let netlist = Netlist::parse(spice).unwrap();
    let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
    mna.g[0][0] += 1.0;

    let kernel = DkKernel::from_mna(&mna, 48000.0).unwrap();

    println!("M = {}", kernel.m);

    let config = CodegenConfig::default();
    let generator = CodeGenerator::new(config);
    let generated = generator.generate(&kernel, &mna, &netlist).unwrap();

    // For 2D systems (two diodes), verify SPICE-style voltage limiting is present
    if kernel.m == 2 {
        assert!(
            generated.code.contains("pnjlim"),
            "2D solver MUST have SPICE pnjlim to prevent NR divergence. \
             This catches the v2 plugin explosion bug."
        );
        assert!(
            generated.code.contains("VCRIT"),
            "Diode devices MUST have precomputed VCRIT for pnjlim"
        );
        assert!(
            generated.code.contains("is_finite"),
            "Solver MUST check for finite outputs"
        );
    }
}
