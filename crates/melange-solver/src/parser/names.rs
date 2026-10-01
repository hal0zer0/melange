//! Identifier checks, node-name length checks, and node-name normalization.

use super::*;

/// Enforce `MAX_NODE_NAME_LEN` on every node reference of a parsed element.
///
/// Returns a descriptive error string on the first over-length name. Callers
/// wrap this in `self.error(...)` to attach a line number. Split out so that
/// both the top-level element loop and the subcircuit-inner element loop can
/// apply it consistently.
/// Is `s` a valid Rust identifier suitable for use as a struct field name?
///
/// Enforces ASCII-only (raw identifiers, unicode idents, and `r#` prefix are
/// rejected) to keep the codegen surface predictable. Does not reject Rust
/// keywords — if an oomox user writes `.runtime Vfoo as loop`, the emitted
/// field will fail to compile and the error message will be clear.
pub(super) fn is_valid_rust_ident(s: &str) -> bool {
    if s.is_empty() || s.len() > MAX_NODE_NAME_LEN {
        return false;
    }
    let mut chars = s.chars();
    let first = chars.next().unwrap();
    if !(first.is_ascii_alphabetic() || first == '_') {
        return false;
    }
    chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

pub(super) fn validate_element_node_lengths(elem: &Element) -> Result<(), String> {
    fn check(node: &str) -> Result<(), String> {
        let len = node.chars().count();
        if len > MAX_NODE_NAME_LEN {
            return Err(format!(
                "node name '{}...' length {} exceeds MAX_NODE_NAME_LEN ({})",
                node.chars().take(16).collect::<String>(),
                len,
                MAX_NODE_NAME_LEN
            ));
        }
        Ok(())
    }

    // Visit every node reference this element carries. Model/component names
    // are not nodes and are not checked here (they have their own limits).
    match elem {
        Element::Resistor {
            n_plus, n_minus, ..
        }
        | Element::Capacitor {
            n_plus, n_minus, ..
        }
        | Element::Inductor {
            n_plus, n_minus, ..
        }
        | Element::VoltageSource {
            n_plus, n_minus, ..
        }
        | Element::CurrentSource {
            n_plus, n_minus, ..
        }
        | Element::Diode {
            n_plus, n_minus, ..
        } => {
            check(n_plus)?;
            check(n_minus)?;
        }
        Element::Bjt { nc, nb, ne, .. } => {
            check(nc)?;
            check(nb)?;
            check(ne)?;
        }
        Element::Jfet { nd, ng, ns, .. } => {
            check(nd)?;
            check(ng)?;
            check(ns)?;
        }
        Element::Mosfet { nd, ng, ns, nb, .. } => {
            check(nd)?;
            check(ng)?;
            check(ns)?;
            check(nb)?;
        }
        Element::Opamp {
            n_plus,
            n_minus,
            n_out,
            ..
        } => {
            check(n_plus)?;
            check(n_minus)?;
            check(n_out)?;
        }
        Element::Triode {
            n_grid,
            n_plate,
            n_cathode,
            ..
        } => {
            check(n_grid)?;
            check(n_plate)?;
            check(n_cathode)?;
        }
        Element::Pentode {
            n_plate,
            n_grid,
            n_cathode,
            n_screen,
            n_suppressor,
            ..
        } => {
            check(n_plate)?;
            check(n_grid)?;
            check(n_cathode)?;
            check(n_screen)?;
            if let Some(ns) = n_suppressor {
                check(ns)?;
            }
        }
        Element::Vca {
            n_sig_p,
            n_sig_n,
            n_ctrl_p,
            n_ctrl_n,
            ..
        } => {
            check(n_sig_p)?;
            check(n_sig_n)?;
            check(n_ctrl_p)?;
            check(n_ctrl_n)?;
        }
        Element::Ldr {
            n_plus,
            n_minus,
            n_ctrl_p,
            n_ctrl_n,
            ..
        } => {
            check(n_plus)?;
            check(n_minus)?;
            check(n_ctrl_p)?;
            check(n_ctrl_n)?;
        }
        Element::Glow {
            n_anode, n_cathode, ..
        } => {
            check(n_anode)?;
            check(n_cathode)?;
        }
        Element::Vcvs {
            out_p,
            out_n,
            ctrl_p,
            ctrl_n,
            ..
        }
        | Element::Vccs {
            out_p,
            out_n,
            ctrl_p,
            ctrl_n,
            ..
        } => {
            check(out_p)?;
            check(out_n)?;
            check(ctrl_p)?;
            check(ctrl_n)?;
        }
        Element::SubcktInstance { nodes, .. } => {
            for node in nodes {
                check(node)?;
            }
        }
        Element::BSource {
            n_plus,
            n_minus,
            expr,
            ..
        } => {
            check(n_plus)?;
            check(n_minus)?;
            // Also validate every node the expression references.
            for node in expr.referenced_nodes() {
                check(&node)?;
            }
        }
    }
    Ok(())
}

/// Normalize a node-name token.
///
/// SPICE node names are case-insensitive (ngspice folds case), so fold to
/// lowercase — otherwise `IN` and `in` silently become two different nets.
/// `gnd` / `ground` (any case) are aliased to the ground node "0", matching
/// ngspice's automatic gnd handling.
pub fn normalize_node_name(raw: &str) -> String {
    let lower = raw.to_ascii_lowercase();
    if lower == "gnd" || lower == "ground" {
        use std::sync::atomic::{AtomicBool, Ordering};
        static GND_ALIAS_LOGGED: AtomicBool = AtomicBool::new(false);
        if !GND_ALIAS_LOGGED.swap(true, Ordering::Relaxed) {
            log::info!(
                "node name '{}' aliased to ground node '0' (ngspice-compatible gnd handling)",
                raw
            );
        }
        return "0".to_string();
    }
    lower
}

/// Apply [`normalize_node_name`] to every node reference an element carries.
///
/// Component and model names are NOT nodes and keep their case (they are
/// compared case-insensitively everywhere they are consumed).
pub(super) fn normalize_element_nodes(elem: &mut Element) {
    let norm = |n: &mut String| *n = normalize_node_name(n);
    match elem {
        Element::Resistor {
            n_plus, n_minus, ..
        }
        | Element::Capacitor {
            n_plus, n_minus, ..
        }
        | Element::Inductor {
            n_plus, n_minus, ..
        }
        | Element::VoltageSource {
            n_plus, n_minus, ..
        }
        | Element::CurrentSource {
            n_plus, n_minus, ..
        }
        | Element::Diode {
            n_plus, n_minus, ..
        } => {
            norm(n_plus);
            norm(n_minus);
        }
        Element::Bjt { nc, nb, ne, .. } => {
            norm(nc);
            norm(nb);
            norm(ne);
        }
        Element::Jfet { nd, ng, ns, .. } => {
            norm(nd);
            norm(ng);
            norm(ns);
        }
        Element::Mosfet { nd, ng, ns, nb, .. } => {
            norm(nd);
            norm(ng);
            norm(ns);
            norm(nb);
        }
        Element::Opamp {
            n_plus,
            n_minus,
            n_out,
            ..
        } => {
            norm(n_plus);
            norm(n_minus);
            norm(n_out);
        }
        Element::Triode {
            n_grid,
            n_plate,
            n_cathode,
            ..
        } => {
            norm(n_grid);
            norm(n_plate);
            norm(n_cathode);
        }
        Element::Pentode {
            n_plate,
            n_grid,
            n_cathode,
            n_screen,
            n_suppressor,
            ..
        } => {
            norm(n_plate);
            norm(n_grid);
            norm(n_cathode);
            norm(n_screen);
            if let Some(ns) = n_suppressor {
                norm(ns);
            }
        }
        Element::Vca {
            n_sig_p,
            n_sig_n,
            n_ctrl_p,
            n_ctrl_n,
            ..
        } => {
            norm(n_sig_p);
            norm(n_sig_n);
            norm(n_ctrl_p);
            norm(n_ctrl_n);
        }
        Element::Ldr {
            n_plus,
            n_minus,
            n_ctrl_p,
            n_ctrl_n,
            ..
        } => {
            norm(n_plus);
            norm(n_minus);
            norm(n_ctrl_p);
            norm(n_ctrl_n);
        }
        Element::Glow {
            n_anode, n_cathode, ..
        } => {
            norm(n_anode);
            norm(n_cathode);
        }
        Element::Vcvs {
            out_p,
            out_n,
            ctrl_p,
            ctrl_n,
            ..
        }
        | Element::Vccs {
            out_p,
            out_n,
            ctrl_p,
            ctrl_n,
            ..
        } => {
            norm(out_p);
            norm(out_n);
            norm(ctrl_p);
            norm(ctrl_n);
        }
        Element::SubcktInstance { nodes, .. } => {
            for node in nodes.iter_mut() {
                norm(node);
            }
        }
        Element::BSource {
            n_plus,
            n_minus,
            expr,
            ..
        } => {
            norm(n_plus);
            norm(n_minus);
            // Normalize `V(node)` / `V(a,b)` references inside the expression
            // so they resolve to the same nets as element terminals. (This
            // also folds `I(elem)` branch references to lowercase; branch
            // names are matched case-insensitively downstream.)
            *expr = expr.remap_idents(&|n| normalize_node_name(n));
        }
    }
}
