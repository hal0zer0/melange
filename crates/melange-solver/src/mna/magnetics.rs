//! Inductor, coupled-inductor, transformer and saturating-core info.

use super::*;

/// Ideal transformer coupling from decomposition of large tightly-coupled inductors.
///
/// Enforces V(sec_p) - V(sec_n) = turns_ratio * (V(pri_p) - V(pri_n)) as an
/// algebraic constraint via augmented MNA. Also injects reflected current at the
/// primary: I_pri = turns_ratio * I_sec (power conservation).
///
/// Created when a transformer group has max(L) > 1H and max(k) > 0.8.
/// The original coupled inductors are replaced by:
/// - One magnetizing inductance (L_ref, added to uncoupled inductors)
/// - One IdealTransformerCoupling per non-reference winding
#[derive(Debug, Clone)]
pub struct IdealTransformerCoupling {
    pub name: String,
    /// Reference (primary) winding positive node (1-indexed, 0=ground)
    pub pri_node_p: usize,
    /// Reference (primary) winding negative node
    pub pri_node_n: usize,
    /// Secondary winding positive node
    pub sec_node_p: usize,
    /// Secondary winding negative node
    pub sec_node_n: usize,
    /// Turns ratio: V_sec = turns_ratio * V_pri
    pub turns_ratio: f64,
}

/// A saturating shared core needs a coupling above this (a closed iron core
/// puts k above 0.99); at or below it the group is refused.
pub(super) const IDEAL_XFMR_K_THRESHOLD: f64 = 0.8;

/// Per-winding air-core declarations of one core each imply a magnetizing
/// floor. They are estimates good to the class estimate's own band,
/// `L_air/L0 ≈ g/µ_eff` with geometry factor g = 1..3 (analog-EE review,
/// SATURATING_TRANSFORMERS.md §2.3): floors within this factor take the
/// least, with a notice; farther apart, the deck is refused.
pub(super) const FLOOR_AGREEMENT_BAND: f64 = 3.0;

/// Inductor element info for companion model.
#[derive(Debug, Clone)]
pub struct InductorElement {
    pub name: String,
    pub node_i: usize,
    pub node_j: usize,
    pub value: f64,
    /// Saturation current for iron-core model. None = linear (default).
    /// When set, the flux is Φ(i) = L_mag·isat·tanh(i/isat) + L_air·i with
    /// L_mag + L_air = value (see `air_floor`).
    pub isat: Option<f64>,
    /// The authored air-core floor of a saturating inductor (`LAIR=`/`CORE=`);
    /// `None` = the default. Meaningless without `isat`.
    pub air_floor: Option<crate::parser::SatFloor>,
    /// The magnetizing branch of a shared saturating core (`{ref}_mag`): its
    /// floor, resolved against the whole core when the group is built.
    /// `None` for a single inductor.
    pub shared_core: Option<SharedCore>,
}

/// A shared saturating core's magnetizing floor, resolved from every
/// winding's declaration (see the coupled-group build in `MnaBuilder`).
#[derive(Debug, Clone)]
pub struct SharedCore {
    /// The air floor as a fraction of the magnetizing branch's inductance.
    pub floor_frac: f64,
    /// Which reading of the declarations applied, for the generated code.
    pub floor_reading: String,
    /// The pair's coupling, for the implicit two-winding form (no `LM=`).
    pub implicit_k: Option<f64>,
}

/// Why `name` belongs to saturating iron, if it does: it carries `ISAT=` (or a
/// datasheet rating) itself, or it is `K`-coupled, directly or through other
/// windings, to an inductor that does. `None` for a linear inductor.
pub(super) fn saturating_core_member(netlist: &Netlist, name: &str) -> Option<String> {
    let saturates = |n: &str| {
        netlist.elements.iter().any(|e| {
            matches!(e, Element::Inductor { name, isat, .. }
                if name.eq_ignore_ascii_case(n) && isat.is_some())
        })
    };
    if saturates(name) {
        return Some("it is a saturating inductor (ISAT=)".to_string());
    }
    let mut seen = vec![name.to_ascii_lowercase()];
    let mut i = 0;
    while i < seen.len() {
        let cur = seen[i].clone();
        for k in &netlist.couplings {
            let (a, b) = (
                k.inductor1_name.to_ascii_lowercase(),
                k.inductor2_name.to_ascii_lowercase(),
            );
            let other = if a == cur {
                b
            } else if b == cur {
                a
            } else {
                continue;
            };
            if !seen.contains(&other) {
                if saturates(&other) {
                    return Some(format!(
                        "it is a winding of a saturating core ({} carries ISAT)",
                        other.to_ascii_uppercase()
                    ));
                }
                seen.push(other);
            }
        }
        i += 1;
    }
    None
}

/// The magnetizing air-core floor of a shared saturating core, in units of
/// the reference winding's inductance `L_ref` (analog-EE review):
/// - `CORE=` or no declaration: the class value is the core's MAGNETIZING
///   (mutual) air floor, `class`; leakage comes from the deck's `k`.
/// - authored `LAIR=`: the winding's TOTAL air-core self-inductance, of which
///   the fixed leakage `(1 - k)` is already carried by the T-model, leaving
///   `LAIR - (1 - k)`. Non-positive is a contradiction of the deck's own `k`.
///
/// A fraction of a winding's own inductance is the same fraction referred to
/// `L_ref` (both scale by turns squared).
/// Convert a datasheet saturation rating to the model's tanh scale current.
///
/// The winding's terminal inductance, as a fraction of its small-signal L0, is
/// `(1 - k) + (k - F)·sech²(x) + F` (incremental) or
/// `(1 - k) + F + (k - F)·tanh(x)/x` (apparent, `Φ/i`), with `x = i/ISAT`,
/// `k` the coupling of a shared core (1 for a single inductor) and `F` the
/// magnetizing air floor in units of L0. A rating that the drop says was
/// reached at `i_ds` fixes `x`, and `ISAT = i_ds / x` (analog-EE review). A
/// drop the saturable part `k - F` cannot reach is refused.
pub fn isat_from_datasheet(
    name: &str,
    i_ds: f64,
    spec: crate::parser::IsatSpec,
    l0: f64,
    k: f64,
    floor: f64,
) -> Result<f64, MnaError> {
    use crate::parser::{IsatBasis, IsatSpec};
    let (drop, basis, what) = match spec {
        IsatSpec::Drop { drop, basis } => (drop, basis, format!("ISAT_DROP={drop}")),
        IsatSpec::LAtIdc { l } => (
            1.0 - l / l0,
            IsatBasis::Incremental,
            format!(
                "L_AT_IDC={l:e},{i_ds:e} (a drop of {:.4} from {l0:e})",
                1.0 - l / l0
            ),
        ),
    };
    let m = k - floor;
    if !(drop > 0.0 && drop < m) {
        return Err(MnaError::InvalidParameter(format!(
            "{name}: {what} cannot be reached. Only {m:.6} of the inductance can saturate \
             (the rest is leakage 1 - k = {:.3e} and the air-core floor {floor:.3e}), so the \
             inductance never falls by {drop:.4}.",
            1.0 - k
        )));
    }
    // Below this the drop is not a datasheet rating, and tanh(x)/x cannot
    // resolve it in f64 (its deficit from 1 is x²/3).
    const MIN_DROP: f64 = 1e-6;
    if drop < MIN_DROP {
        return Err(MnaError::InvalidParameter(format!(
            "{name}: {what} is a drop of {drop:.3e}, below {MIN_DROP:e}. That is not a \
             saturation rating and cannot be converted accurately; give ISAT= directly."
        )));
    }
    let ratio = (m - drop) / m;
    let x = match basis {
        // sech²(x) = ratio  <=>  tanh²(x) = drop/m; atanh is the stable form
        // (acosh(1/√ratio) loses digits as ratio -> 1).
        IsatBasis::Incremental => (drop / m).sqrt().atanh(),
        IsatBasis::Apparent => {
            // tanh(x)/x falls monotonically from 1 at x = 0.
            const LO: f64 = 1e-9;
            const HI: f64 = 1e9;
            let (mut lo, mut hi) = (LO, HI);
            for _ in 0..200 {
                let mid = (lo * hi).sqrt();
                if mid.tanh() / mid > ratio {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let x = (lo * hi).sqrt();
            if x <= LO * (1.0 + 1e-6) || x >= HI * (1.0 - 1e-6) {
                return Err(MnaError::InvalidParameter(format!(
                    "{name}: {what} (apparent) is outside the range the conversion can \
                     resolve (x = I/ISAT would be {x:e}); give ISAT= directly."
                )));
            }
            x
        }
    };
    let isat = i_ds / x;
    if !(isat.is_finite() && isat > 0.0) {
        return Err(MnaError::InvalidParameter(format!(
            "{name}: {what} at {i_ds:e} A converts to ISAT = {isat:e}, which is not a \
             usable saturation current."
        )));
    }
    Ok(isat)
}

/// Coupled inductor pair info for transformer companion model.
///
/// Two inductors L1 and L2 with coupling coefficient k have mutual
/// inductance M = k * sqrt(L1 * L2). The companion model stamps
/// both self-conductances and cross-coupling conductances.
#[derive(Debug, Clone)]
pub struct CoupledInductorInfo {
    pub name: String,
    pub l1_name: String,
    pub l2_name: String,
    pub l1_node_i: usize,
    pub l1_node_j: usize,
    pub l2_node_i: usize,
    pub l2_node_j: usize,
    pub l1_value: f64,
    pub l2_value: f64,
    pub coupling: f64,
}

/// Multi-winding transformer group info.
///
/// Groups 3+ inductors that share a magnetic core (connected via K directives).
/// The companion model uses an NxN inductance matrix and its inverse for
/// admittance stamping, instead of per-pair 2x2 inversions.
#[derive(Debug, Clone)]
pub struct TransformerGroupInfo {
    /// Auto-generated group name (e.g. "xfmr_0")
    pub name: String,
    /// Number of windings in this group
    pub num_windings: usize,
    /// Inductor names in group order
    pub winding_names: Vec<String>,
    /// Positive node index for each winding (1-indexed, 0=ground)
    pub winding_node_i: Vec<usize>,
    /// Negative node index for each winding (1-indexed, 0=ground)
    pub winding_node_j: Vec<usize>,
    /// Self-inductance for each winding
    pub inductances: Vec<f64>,
    /// NxN coupling coefficient matrix (symmetric, diagonal = 1.0)
    pub coupling_matrix: Vec<Vec<f64>>,
}
