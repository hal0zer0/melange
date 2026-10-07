//! Structural sparsity: which matrix entries can be nonzero, from the circuit's
//! topology alone.
//!
//! The generated solver skips the terms of `K = N_v·S·N_i` (and its
//! backward-Euler twin) that are zero. Deciding "zero" from the computed values
//! is unreliable: `S = A⁻¹` is formed in floating point, so a coupling that is
//! exactly zero in exact arithmetic comes out as rounding noise (1e-35 to
//! 1e-19 against entries near 1e5), and which noise entries clear any fixed
//! cutoff changes with the sample rate. Here the pattern is derived from the
//! positions of the stamped matrices instead, so it is the same at every rate
//! and never drops a real coupling.
//!
//! The pattern of `A⁻¹` follows from a classical result (Gilbert, "Predicting
//! structure in sparse matrix computations", SIAM J. Matrix Anal. Appl. 15(1),
//! 1994): for a matrix `B` with a zero-free diagonal, `(B⁻¹)[i][j]` can be
//! nonzero only if the directed graph of `B` (an edge `i → k` for every
//! `B[i][k] ≠ 0`) has a path from `i` to `j`. An MNA matrix can have zero
//! diagonals (the branch rows of voltage sources), so its rows are first
//! permuted by a perfect matching to put a structural nonzero on every
//! diagonal. The resulting pattern is an upper bound: an entry it admits may
//! still be zero for particular values, never the other way round.

/// A boolean `rows × cols` pattern, row-major: `pattern[i][j]` is true when
/// entry `(i, j)` may be nonzero.
pub type Pattern = Vec<Vec<bool>>;

/// The positions of a flat row-major `rows × cols` matrix holding a value other
/// than exactly `0.0`. For a matrix assembled directly from stamps (G, C, N_v,
/// N_i, and their scalings), this is the set of positions the stamps wrote: no
/// inversion has run, so a written position carries its stamp's value, not
/// rounding noise.
pub fn nonzero_pattern(flat: &[f64], rows: usize, cols: usize) -> Pattern {
    assert_eq!(flat.len(), rows * cols, "nonzero_pattern: flat length");
    (0..rows)
        .map(|i| (0..cols).map(|j| flat[i * cols + j] != 0.0).collect())
        .collect()
}

/// A perfect matching of a square pattern: `row_of_col[k]` is the row assigned
/// to column `k`, every row and column used once. `None` when the pattern is
/// structurally singular (no assignment exists, so the matrix is singular for
/// every choice of values).
///
/// Kuhn's augmenting-path algorithm: each column in turn searches for a free
/// row, re-routing previously matched columns along alternating paths. Rows and
/// columns are tried in index order, so the result is deterministic. O(n·nnz),
/// which is immaterial at circuit sizes.
pub fn perfect_matching(pattern: &Pattern) -> Option<Vec<usize>> {
    let n = pattern.len();
    // col_rows[k]: the rows with a structural nonzero in column k.
    let col_rows: Vec<Vec<usize>> = (0..n)
        .map(|k| (0..n).filter(|&r| pattern[r][k]).collect())
        .collect();
    let mut col_of_row: Vec<Option<usize>> = vec![None; n];

    // Try to give column `k` a row, displacing other columns as needed.
    fn augment(
        k: usize,
        col_rows: &[Vec<usize>],
        col_of_row: &mut [Option<usize>],
        visited: &mut [bool],
    ) -> bool {
        for &r in &col_rows[k] {
            if visited[r] {
                continue;
            }
            visited[r] = true;
            let free = match col_of_row[r] {
                None => true,
                Some(other) => augment(other, col_rows, col_of_row, visited),
            };
            if free {
                col_of_row[r] = Some(k);
                return true;
            }
        }
        false
    }

    for k in 0..n {
        let mut visited = vec![false; n];
        if !augment(k, &col_rows, &mut col_of_row, &mut visited) {
            return None;
        }
    }
    let mut row_of_col = vec![0; n];
    for (r, k) in col_of_row.into_iter().enumerate() {
        row_of_col[k.expect("every column matched")] = r;
    }
    Some(row_of_col)
}

/// The structural pattern of `A⁻¹` for a square pattern of `A`, or `None` when
/// `A` is structurally singular.
///
/// 1. Match columns to rows ([`perfect_matching`]) and form `B = P·A`, whose row
///    `k` is row `row_of_col[k]` of `A`, so `B[k][k] ≠ 0` for every `k`.
/// 2. Gilbert: `(B⁻¹)[i][k]` may be nonzero only if `k` is reachable from `i`
///    in the graph of `B` (every node reaches itself). Reachability is one
///    breadth-first search per node.
/// 3. `A⁻¹ = B⁻¹·P`, and `P` maps column `k` of `B⁻¹` to column
///    `row_of_col[k]` of `A⁻¹`.
pub fn inverse_pattern(a: &Pattern) -> Option<Pattern> {
    let n = a.len();
    let row_of_col = perfect_matching(a)?;
    // Graph of B: successors of node k are the nonzero columns of B's row k.
    let succ: Vec<Vec<usize>> = (0..n)
        .map(|k| (0..n).filter(|&j| a[row_of_col[k]][j]).collect())
        .collect();
    let mut inv = vec![vec![false; n]; n];
    for i in 0..n {
        let mut reached = vec![false; n];
        let mut queue = std::collections::VecDeque::from([i]);
        reached[i] = true;
        while let Some(k) = queue.pop_front() {
            for &j in &succ[k] {
                if !reached[j] {
                    reached[j] = true;
                    queue.push_back(j);
                }
            }
        }
        for k in 0..n {
            if reached[k] {
                inv[i][row_of_col[k]] = true;
            }
        }
    }
    Some(inv)
}

/// The boolean product `X·Y` of an `r × s` and an `s × c` pattern: entry
/// `(i, j)` is present when some `k` has both `X[i][k]` and `Y[k][j]`.
pub fn product(x: &Pattern, y: &Pattern) -> Pattern {
    let cols = y.first().map_or(0, Vec::len);
    x.iter()
        .map(|row| {
            let mut out = vec![false; cols];
            for (k, &xk) in row.iter().enumerate() {
                if xk {
                    for (o, &ykj) in out.iter_mut().zip(&y[k]) {
                        *o |= ykj;
                    }
                }
            }
            out
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Dense inverse by Gauss-Jordan elimination with partial pivoting.
    fn invert(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
        let n = a.len();
        let mut m: Vec<Vec<f64>> = a
            .iter()
            .enumerate()
            .map(|(i, r)| {
                let mut row = r.clone();
                row.extend((0..n).map(|j| if i == j { 1.0 } else { 0.0 }));
                row
            })
            .collect();
        for c in 0..n {
            let p = (c..n)
                .max_by(|&x, &y| m[x][c].abs().total_cmp(&m[y][c].abs()))
                .unwrap();
            m.swap(c, p);
            let d = m[c][c];
            assert!(d.abs() > 1e-9, "test matrix nearly singular");
            for v in m[c].iter_mut() {
                *v /= d;
            }
            for r in 0..n {
                if r != c && m[r][c] != 0.0 {
                    let f = m[r][c];
                    let pivot_row = m[c].clone();
                    for (v, pv) in m[r].iter_mut().zip(&pivot_row) {
                        *v -= f * pv;
                    }
                }
            }
        }
        m.into_iter().map(|r| r[n..].to_vec()).collect()
    }

    /// A 64-bit linear congruential generator (Knuth's MMIX constants), so the
    /// test is deterministic without a dependency.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    /// On random sparse matrices, half of them with their rows shuffled so the
    /// diagonal is mostly zero, every entry of the numeric inverse that is not
    /// rounding-level lies inside the structural pattern. The pattern's
    /// guarantee is about exact arithmetic, so "not rounding-level" here means
    /// above 1e-9 relative to the largest inverse entry; the matrices are kept
    /// well conditioned so a true nonzero is far above that.
    #[test]
    fn numeric_inverse_lies_inside_the_structural_pattern() {
        let mut rng = Lcg(0x5eed);
        for trial in 0..400 {
            let n = 2 + trial % 9;
            let density = 0.15 + 0.5 * rng.next();
            let mut a = vec![vec![0.0; n]; n];
            for (i, row) in a.iter_mut().enumerate() {
                // A dominant diagonal keeps the matrix well conditioned.
                row[i] = 4.0 + rng.next();
                for (j, v) in row.iter_mut().enumerate() {
                    if j != i && rng.next() < density {
                        *v = rng.next() - 0.5;
                    }
                }
            }
            if trial % 2 == 1 {
                // Rotate the rows: diagonal entries move off the diagonal,
                // as on an MNA voltage-source branch row.
                a.rotate_left(1 + trial % (n - 1).max(1));
            }
            let pat = nonzero_pattern(&a.concat(), n, n);
            let inv_pat = inverse_pattern(&pat).expect("nonsingular by construction");
            let inv = invert(&a);
            let scale = inv.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()));
            for i in 0..n {
                for j in 0..n {
                    assert!(
                        inv_pat[i][j] || inv[i][j].abs() <= 1e-9 * scale,
                        "trial {trial}: inverse ({i},{j}) = {} outside the pattern",
                        inv[i][j]
                    );
                }
            }
        }
    }

    #[test]
    fn block_triangular_matrix_keeps_its_zero_block() {
        // A = [[a, 0], [b, c]] (lower block triangular): A⁻¹ is lower
        // triangular, so (0, 1) is structurally zero.
        let pat = vec![vec![true, false], vec![true, true]];
        let inv = inverse_pattern(&pat).unwrap();
        assert_eq!(inv, vec![vec![true, false], vec![true, true]]);
    }

    #[test]
    fn zero_diagonal_branch_row_is_matched() {
        // A voltage source between node 0 and ground in augmented MNA:
        // [[g, 1], [1, 0]], whose inverse is [[0, 1], [1, -g]]: the node
        // voltage is fixed by the source, so it does not depend on the node's
        // own current injection, and (0, 0) is structurally zero.
        let pat = vec![vec![true, true], vec![true, false]];
        assert_eq!(perfect_matching(&pat), Some(vec![1, 0]));
        assert_eq!(
            inverse_pattern(&pat).unwrap(),
            vec![vec![false, true], vec![true, true]]
        );
    }

    #[test]
    fn structurally_singular_pattern_has_no_inverse() {
        // Two rows that can only use column 0.
        let pat = vec![vec![true, false], vec![true, false]];
        assert_eq!(inverse_pattern(&pat), None);
    }

    #[test]
    fn boolean_product() {
        let x = vec![vec![true, false], vec![false, false]];
        let y = vec![vec![false, true], vec![true, true]];
        assert_eq!(product(&x, &y), vec![vec![false, true], vec![false, false]]);
    }
}
