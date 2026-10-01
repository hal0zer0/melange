//! Voltage, controlled, behavioral and current source info, and capacitor `IC=` pins.

/// Augmented-MNA extra row/column info for a VCVS element.
///
/// Each VCVS adds one extra unknown j_vs (the source current) and one
/// algebraic constraint row: V_out+ - V_out- = gain*(V_ctrl+ - V_ctrl-).
#[derive(Debug, Clone)]
pub struct VcvsAugInfo {
    /// 0-based index within VCVS sources (extra row k = n + num_vs + vcvs_idx)
    pub aug_idx: usize,
}

/// Capacitor with an explicit `IC=` initial condition (SPICE `.IC`/UIC
/// semantics). Used only to build the one-off initial-state solve that
/// seeds `v_prev` — the capacitor itself is stamped into `C` normally and
/// participates in the transient exactly like any other capacitor.
#[derive(Debug, Clone)]
pub struct CapacitorIcInfo {
    pub name: String,
    /// 1-indexed node (0 = ground), matches `InductorElement` convention.
    pub node_i: usize,
    pub node_j: usize,
    /// Prescribed initial voltage: v(node_i) - v(node_j) at t=0.
    pub ic: f64,
}

/// Voltage source information for extended MNA.
#[derive(Debug, Clone)]
pub struct VoltageSourceInfo {
    pub name: String,
    pub n_plus: String,
    pub n_minus: String,
    pub n_plus_idx: usize,
    pub n_minus_idx: usize,
    pub dc_value: f64,
    /// Index in extended MNA (for voltage source currents)
    pub ext_idx: usize,
}

/// Behavioral (arbitrary-expression) source — SPICE3 `B` element.
///
/// Behavioral sources do **not** use the `N_v`/`N_i` block-diagonal device
/// machinery (which assumes one controlling node-pair voltage per dimension).
/// Their expression references arbitrary nodes, so codegen stamps their current
/// and Jacobian directly into the node-space Newton system — which is why their
/// presence forces nodal routing.
#[derive(Debug, Clone)]
pub struct BehavioralSourceInfo {
    pub name: String,
    pub kind: crate::parser::BSourceKind,
    pub n_plus: String,
    pub n_minus: String,
    pub n_plus_idx: usize,
    pub n_minus_idx: usize,
    /// Resolved node index for each node name the expression references.
    pub referenced_node_indices: std::collections::BTreeMap<String, usize>,
    /// The parsed expression, with `ddt`/`idt` state slots already assigned
    /// (globally unique across all behavioral sources).
    pub expr: crate::expr::Expr,
    /// For `V={}` sources, the 0-based index among behavioral voltage sources
    /// (used to allocate the augmented branch-current row at codegen time).
    /// `None` for `I={}` sources.
    pub v_ext_idx: Option<usize>,
    /// Augmented MNA row/col (branch current) for `V={}` sources, assigned in
    /// `build()` after the voltage-source / VCVS rows. `None` for `I={}`.
    pub aug_row: Option<usize>,
}

/// Current source information.
#[derive(Debug, Clone)]
pub struct CurrentSourceInfo {
    pub name: String,
    pub n_plus_idx: usize,
    pub n_minus_idx: usize,
    pub dc_value: f64,
}
