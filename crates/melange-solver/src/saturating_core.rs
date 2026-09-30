//! The split of a saturating shared core's inductance matrix into one
//! saturable magnetizing term and a constant leakage matrix.
//!
//! A core whose flux links every winding (one magnetic loop) has
//!
//! ```text
//! λ = Lm(φ)·n nᵀ·i + L_leak·i
//! ```
//!
//! with `n` the turns vector referred to the reference winding (`n_ref = 1`),
//! `Lm` the magnetizing inductance seen from that winding, and `L_leak` the
//! air leakage, a full constant symmetric matrix. Saturation lives in the
//! rank-1 core term only. The linear `[L]` (self-inductances and `K` lines)
//! does not fix the split for three or more windings, so the deck states
//! `LM=` and `TURNS=`; two windings without them keep the implicit form
//! `Lm = k·L_ref`, `n_i = √(L_i/L_ref)`, whose leakage is diagonal.
//! See `docs/aidocs/SATURATING_TRANSFORMERS.md` §2.2.

/// A core split: `L = lm·n nᵀ + l_leak`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CoreSplit {
    /// Winding the magnetizing branch is referred to (`n[ref_idx] = 1`).
    pub ref_idx: usize,
    /// Magnetizing inductance seen from the reference winding [H].
    pub lm: f64,
    /// Turns referred to the reference winding.
    pub n: Vec<f64>,
    /// Leakage inductance matrix [H], symmetric.
    pub l_leak: Vec<Vec<f64>>,
}

/// Off-diagonal leakage below this fraction of `√(L_ii·L_jj)` is zero: the
/// residue of rounding in `L − lm·n nᵀ`, not a coupling.
const LEAK_COUPLING_ZERO: f64 = 1e-12;

impl CoreSplit {
    /// The implicit two-winding form: `Lm = k·L_ref` on the larger winding,
    /// `n_i = √(L_i/L_ref)`, leakage `(1 − k)·L_i` on each winding and none
    /// between them.
    pub fn implicit_pair(l: [f64; 2], k: f64) -> Self {
        // The larger winding; on a tie the second, as a `max_by` over the
        // group picks it.
        let ref_idx = if l[1] >= l[0] { 1 } else { 0 };
        let l_ref = l[ref_idx];
        Self {
            ref_idx,
            lm: k * l_ref,
            n: l.iter().map(|li| (li / l_ref).sqrt()).collect(),
            l_leak: vec![vec![(1.0 - k) * l[0], 0.0], vec![0.0, (1.0 - k) * l[1]]],
        }
    }

    /// The stated form: `lm` on winding `lm_idx`, relative `turns` on every
    /// winding, and the linear inductance matrix `l`.
    pub fn explicit(l: &[Vec<f64>], turns: &[f64], lm_idx: usize, lm: f64) -> Self {
        let n: Vec<f64> = turns.iter().map(|t| t / turns[lm_idx]).collect();
        let w = l.len();
        let mut l_leak = vec![vec![0.0; w]; w];
        for i in 0..w {
            for j in 0..w {
                l_leak[i][j] = l[i][j] - lm * n[i] * n[j];
            }
        }
        for i in 0..w {
            for j in 0..w {
                if i != j && l_leak[i][j].abs() <= LEAK_COUPLING_ZERO * (l[i][i] * l[j][j]).sqrt() {
                    l_leak[i][j] = 0.0;
                }
            }
        }
        Self {
            ref_idx: lm_idx,
            lm,
            n,
            l_leak,
        }
    }

    /// Whether the leakage matrix has any coupling between windings.
    pub fn leakage_is_diagonal(&self) -> bool {
        let w = self.l_leak.len();
        (0..w).all(|i| (0..w).all(|j| i == j || self.l_leak[i][j] == 0.0))
    }

    /// `Ok` when `l_leak` is positive-definite (air-flux energy, and a
    /// well-posed stamp); otherwise its least eigenvalue and eigenvector.
    pub fn leakage_positive_definite(&self) -> Result<(), (f64, Vec<f64>)> {
        let (values, vectors) = symmetric_eigen(&self.l_leak);
        let scale = (0..self.l_leak.len())
            .map(|i| self.l_leak[i][i].abs())
            .fold(0.0_f64, f64::max);
        if values[0] > 1e-12 * scale && values[0] > 0.0 {
            Ok(())
        } else {
            Err((values[0], vectors[0].clone()))
        }
    }
}

/// Eigenvalues (ascending) and unit eigenvectors of a small symmetric matrix,
/// by cyclic Jacobi rotations.
pub(crate) fn symmetric_eigen(a: &[Vec<f64>]) -> (Vec<f64>, Vec<Vec<f64>>) {
    let w = a.len();
    let mut m: Vec<Vec<f64>> = a.to_vec();
    let mut v: Vec<Vec<f64>> = (0..w)
        .map(|i| (0..w).map(|j| if i == j { 1.0 } else { 0.0 }).collect())
        .collect();
    for _sweep in 0..100 {
        let off: f64 = (0..w)
            .flat_map(|i| (0..w).filter(move |&j| j != i).map(move |j| (i, j)))
            .map(|(i, j)| m[i][j] * m[i][j])
            .sum();
        let diag: f64 = (0..w).map(|i| m[i][i] * m[i][i]).sum();
        if off <= 1e-32 * diag || off == 0.0 {
            break;
        }
        for p in 0..w {
            for q in p + 1..w {
                if m[p][q] == 0.0 {
                    continue;
                }
                let theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for r in 0..w {
                    let (mrp, mrq) = (m[r][p], m[r][q]);
                    m[r][p] = c * mrp - s * mrq;
                    m[r][q] = s * mrp + c * mrq;
                }
                for r in 0..w {
                    let (mpr, mqr) = (m[p][r], m[q][r]);
                    m[p][r] = c * mpr - s * mqr;
                    m[q][r] = s * mpr + c * mqr;
                }
                for r in 0..w {
                    let (vrp, vrq) = (v[r][p], v[r][q]);
                    v[r][p] = c * vrp - s * vrq;
                    v[r][q] = s * vrp + c * vrq;
                }
            }
        }
    }
    let mut order: Vec<usize> = (0..w).collect();
    order.sort_by(|&a, &b| m[a][a].total_cmp(&m[b][b]));
    let values = order.iter().map(|&i| m[i][i]).collect();
    let vectors = order
        .iter()
        .map(|&i| (0..w).map(|r| v[r][i]).collect())
        .collect();
    (values, vectors)
}

/// The diagonal-leakage (star) decomposition of a three-winding `[L]`:
/// `k_ij = c_i·c_j`, so `c_i = √(k_ij·k_ik/k_jk)`. Returned as the deck keys
/// that state it, `(lm_idx, LM, TURNS)`, with `LM` on the largest winding
/// and its `TURNS` 1. `None` when it does not exist (a missing or
/// non-positive coupling) or needs a negative leakage (some `c_i ≥ 1`, a
/// winding sandwiched between the other two).
pub(crate) fn star_decomposition(l: &[Vec<f64>]) -> Option<(usize, f64, Vec<f64>)> {
    if l.len() != 3 {
        return None;
    }
    let self_l: Vec<f64> = (0..3).map(|i| l[i][i]).collect();
    let k = |i: usize, j: usize| l[i][j] / (self_l[i] * self_l[j]).sqrt();
    let mut c = [0.0; 3];
    for (i, ci) in c.iter_mut().enumerate() {
        let (j, m) = ((i + 1) % 3, (i + 2) % 3);
        let x = k(i, j) * k(i, m) / k(j, m);
        if !(x.is_finite() && x > 0.0 && k(j, m) > 0.0) {
            return None;
        }
        *ci = x.sqrt();
        if *ci >= 1.0 {
            return None;
        }
    }
    let a = (0..3).max_by(|&x, &y| self_l[x].total_cmp(&self_l[y]))?;
    let u: Vec<f64> = (0..3).map(|i| c[i] * self_l[i].sqrt()).collect();
    Some((a, u[a] * u[a], u.iter().map(|ui| ui / u[a]).collect()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mat(l: &[f64], k: &[(usize, usize, f64)]) -> Vec<Vec<f64>> {
        let w = l.len();
        let mut m = vec![vec![0.0; w]; w];
        for i in 0..w {
            m[i][i] = l[i];
        }
        for &(i, j, kij) in k {
            m[i][j] = kij * (l[i] * l[j]).sqrt();
            m[j][i] = m[i][j];
        }
        m
    }

    #[test]
    fn jacobi_reproduces_the_matrix() {
        let a = mat(
            &[2.0, 1.0, 0.5, 3.0],
            &[(0, 1, 0.3), (0, 3, -0.2), (1, 2, 0.6), (2, 3, 0.1)],
        );
        let (vals, vecs) = symmetric_eigen(&a);
        assert!(vals.windows(2).all(|p| p[0] <= p[1]));
        for i in 0..4 {
            for j in 0..4 {
                let r: f64 = (0..4).map(|e| vals[e] * vecs[e][i] * vecs[e][j]).sum();
                assert!((r - a[i][j]).abs() < 1e-12, "[{i}][{j}] {r} vs {}", a[i][j]);
            }
        }
    }

    /// The implicit pair is the explicit form with `LM = k·L_ref` and
    /// `TURNS = √L`: same leakage (diagonal), same turns.
    #[test]
    fn implicit_pair_is_the_explicit_special_case() {
        let (l, k) = ([0.3, 2.0], 0.9993);
        let imp = CoreSplit::implicit_pair(l, k);
        let full = mat(&l, &[(0, 1, k)]);
        let exp = CoreSplit::explicit(&full, &[l[0].sqrt(), l[1].sqrt()], 1, k * l[1]);
        assert_eq!(imp.ref_idx, exp.ref_idx);
        assert!(exp.leakage_is_diagonal());
        for i in 0..2 {
            assert!((imp.n[i] - exp.n[i]).abs() < 1e-15);
            assert!((imp.l_leak[i][i] - exp.l_leak[i][i]).abs() < 1e-14 * l[i]);
        }
    }

    #[test]
    fn star_is_an_exact_split_of_a_three_winding_l() {
        // Couplings from star factors c = (0.998, 0.995, 0.999): k_ij = c_i·c_j.
        let c = [0.998, 0.995, 0.999];
        let l = mat(
            &[1.0, 0.25, 4.0],
            &[
                (0, 1, c[0] * c[1]),
                (0, 2, c[0] * c[2]),
                (1, 2, c[1] * c[2]),
            ],
        );
        let (a, lm, turns) = star_decomposition(&l).unwrap();
        assert_eq!(a, 2);
        let split = CoreSplit::explicit(&l, &turns, a, lm);
        assert!(split.leakage_is_diagonal(), "{:?}", split.l_leak);
        assert!(split.leakage_positive_definite().is_ok());
    }

    #[test]
    fn a_sandwiched_winding_has_no_star() {
        // c_1 = sqrt(0.999·0.999/0.99) > 1.
        let l = mat(
            &[1.0, 1.0, 1.0],
            &[(0, 1, 0.999), (1, 2, 0.999), (0, 2, 0.99)],
        );
        assert!(star_decomposition(&l).is_none());
    }

    #[test]
    fn too_large_an_lm_is_not_positive_definite() {
        let l = mat(&[1.0, 1.0], &[(0, 1, 0.99)]);
        let split = CoreSplit::explicit(&l, &[1.0, 1.0], 0, 0.999);
        let (value, vector) = split.leakage_positive_definite().unwrap_err();
        assert!(value < 0.0);
        // The offending direction is the common mode, where the core term sits.
        assert!((vector[0] - vector[1]).abs() < 1e-9, "{vector:?}");
    }
}
