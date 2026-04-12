//! Solver routing: auto-select DK vs nodal codegen path.
//!
//! Extracts the DK-vs-nodal routing decision from the CLI into testable,
//! reusable library code. The routing heuristics consider:
//!
//! - DK kernel build failure (positive feedback / oscillators)
//! - Trapezoidal stability (spectral radius of S*A_neg)
//! - Multi-transformer circuits (DK K matrix instability)
//! - Nonlinear dimension (M≥10 → M×M elimination expensive)

use crate::dk::DkKernel;
use crate::mna::MnaSystem;

/// Which codegen solver path to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverRoute {
    /// DK Schur path: precompute S=A⁻¹, NR in M-dimensional space.
    /// Fast (100-600× realtime), used for most circuits with M<10.
    DkSchur,
    /// Nodal path: full N×N LU solve per NR iteration.
    /// Universal (handles any topology), auto-selects Schur or full LU internally.
    Nodal,
}

/// Information about why a particular route was chosen.
#[derive(Debug, Clone)]
pub struct RoutingDecision {
    /// The chosen route.
    pub route: SolverRoute,
    /// Whether the DK kernel build failed.
    pub dk_failed: bool,
    /// Whether trapezoidal is unstable (spectral radius > threshold).
    pub dk_unstable: bool,
    /// Spectral radius of S*A_neg (0.0 if not computed).
    pub spectral_radius: f64,
    /// Whether the circuit has multiple transformer groups.
    pub multi_transformer: bool,
    /// Whether M >= 10 (large nonlinear dimension).
    pub large_m: bool,
    /// Whether K matrix is ill-conditioned (max|K| > 1e8).
    pub k_ill_conditioned: bool,
    /// Whether S matrix is ill-conditioned (max|S| > 1e6).
    pub s_ill_conditioned: bool,
    /// Human-readable reason for the routing decision.
    pub reason: String,
}

/// Auto-select DK or nodal codegen path based on circuit properties.
///
/// # Arguments
/// * `kernel` - The DK kernel (may be from a failed build attempt)
/// * `mna` - The MNA system
/// * `dk_failed` - Whether `DkKernel::from_mna()` returned an error
///
/// # Returns
/// A `RoutingDecision` with the chosen route and diagnostic info.
pub fn auto_route(kernel: &DkKernel, mna: &MnaSystem, dk_failed: bool) -> RoutingDecision {
    let n = kernel.n;
    let m = kernel.m;

    // Count transformer groups
    let n_xfmr_groups = mna.transformer_groups.len()
        + if !mna.coupled_inductors.is_empty() {
            1
        } else {
            0
        };
    let has_inductors = !mna.inductors.is_empty()
        || !mna.coupled_inductors.is_empty()
        || !mna.transformer_groups.is_empty();
    let multi_transformer = has_inductors && n_xfmr_groups > 1;
    let large_m = m >= 10;

    // Check trapezoidal stability via power iteration on S*A_neg
    let (dk_unstable, spectral_radius) = if !dk_failed && m > 0 && n > 0 {
        compute_spectral_radius(&kernel.s, &kernel.a_neg, n)
    } else {
        (false, 0.0)
    };

    // Check K matrix conditioning: max|K| > 1e8 means the DK Schur NR
    // operates in a numerically hostile space. Nodal full-LU avoids K entirely.
    let k_ill_conditioned = if !dk_failed && m > 0 {
        let k_max_abs = (0..m * m)
            .map(|i| kernel.k[i].abs())
            .fold(0.0_f64, f64::max);
        k_max_abs > 1e8
    } else {
        false
    };

    // Check S matrix conditioning: max|S| > 1e6 means cap-only nodes
    // lack resistive paths — Schur prediction is unreliable.
    let s_ill_conditioned = if !dk_failed && n > 0 {
        let s_max_abs = (0..n * n)
            .map(|i| kernel.s[i].abs())
            .fold(0.0_f64, f64::max);
        s_max_abs > 1e6
    } else {
        false
    };

    // Routing decision (first match wins)
    let route;
    let reason;
    if dk_failed {
        route = SolverRoute::Nodal;
        reason = "DK kernel build failed (positive feedback or oscillator circuit)".to_string();
    } else if dk_unstable {
        route = SolverRoute::Nodal;
        reason = format!(
            "trapezoidal unstable (spectral radius {:.4} > 1.002)",
            spectral_radius
        );
    } else if multi_transformer {
        route = SolverRoute::Nodal;
        reason = format!(
            "multi-transformer circuit ({} groups, DK K matrix unstable)",
            n_xfmr_groups
        );
    } else if large_m {
        route = SolverRoute::Nodal;
        reason = format!(
            "large nonlinear dimension (M={}, M×M elimination expensive)",
            m
        );
    } else if k_ill_conditioned {
        route = SolverRoute::Nodal;
        reason = format!(
            "K matrix ill-conditioned (max|K| > 1e8, nodal full-LU avoids K)"
        );
    } else if s_ill_conditioned {
        route = SolverRoute::Nodal;
        reason = format!(
            "S matrix ill-conditioned (max|S| > 1e6, cap-only nodes lack resistive paths)"
        );
    } else if mna.inductors.iter().any(|ind| ind.isat.is_some())
        || mna.coupled_inductors.iter().any(|ci| ci.l1_isat.is_some() || ci.l2_isat.is_some())
        || mna.transformer_groups.iter().any(|g| g.winding_isats.iter().any(|i| i.is_some()))
    {
        route = SolverRoute::Nodal;
        reason = "saturating inductors require augmented MNA (per-sample L update)".to_string();
    } else {
        route = SolverRoute::DkSchur;
        reason = format!("DK Schur (N={}, M={})", n, m);
    }

    RoutingDecision {
        route,
        dk_failed,
        dk_unstable,
        spectral_radius,
        multi_transformer,
        large_m,
        k_ill_conditioned,
        s_ill_conditioned,
        reason,
    }
}

/// Compute spectral radius of S*A_neg via power iteration.
///
/// Returns (is_unstable, spectral_radius). Unstable if rho > 1.002.
fn compute_spectral_radius(s: &[f64], a_neg: &[f64], n: usize) -> (bool, f64) {
    let mut x = vec![1.0 / (n as f64).sqrt(); n];
    let mut y = vec![0.0; n];
    let mut rho = 0.0;

    for _ in 0..20 {
        // y = S * (A_neg * x)
        let mut ax = vec![0.0; n];
        for i in 0..n {
            for j in 0..n {
                ax[i] += a_neg[i * n + j] * x[j];
            }
        }
        for i in 0..n {
            y[i] = 0.0;
            for j in 0..n {
                y[i] += s[i * n + j] * ax[j];
            }
        }
        let norm: f64 = y.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm < 1e-30 {
            break;
        }
        rho = norm / x.iter().map(|v| v * v).sum::<f64>().sqrt();
        for i in 0..n {
            x[i] = y[i] / norm;
        }
    }

    (rho > 1.002, rho)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mna::MnaSystem;
    use crate::parser::Netlist;

    #[test]
    fn test_route_linear_circuit_dk() {
        let spice = "RC Lowpass\nR1 in out 1k\nC1 out 0 1u\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.g[0][0] += 1.0;
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();
        let decision = auto_route(&kernel, &mna, false);
        assert_eq!(decision.route, SolverRoute::DkSchur);
        assert!(!decision.dk_failed);
        assert!(!decision.dk_unstable);
    }

    #[test]
    fn test_route_nonlinear_dk() {
        let spice = "\
Diode Clipper
R1 in out 1k
D1 out 0 DMOD
D2 0 out DMOD
C1 out 0 1u
.model DMOD D(IS=2.52e-9 N=1.752)
";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.g[0][0] += 1.0;
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();
        let decision = auto_route(&kernel, &mna, false);
        assert_eq!(decision.route, SolverRoute::DkSchur);
        assert!(decision.spectral_radius < 1.002);
    }

    #[test]
    fn test_route_dk_failed_goes_nodal() {
        let spice = "RC Lowpass\nR1 in out 1k\nC1 out 0 1u\n";
        let netlist = Netlist::parse(spice).unwrap();
        let mut mna = MnaSystem::from_netlist(&netlist).unwrap();
        mna.g[0][0] += 1.0;
        let kernel = DkKernel::from_mna(&mna, 44100.0).unwrap();
        let decision = auto_route(&kernel, &mna, true);
        assert_eq!(decision.route, SolverRoute::Nodal);
        assert!(decision.dk_failed);
    }
}
