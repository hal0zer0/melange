//! Drift guard for the central `.model` parameter tables (`src/model_params.rs`).
//!
//! The tables decide two user-visible things: whether an unknown key on a
//! `.model` card is reported, and — because the codegen check is a *hard error*
//! — whether a deck using a real key is refused outright. Both failure modes are
//! silent-until-it-bites:
//!
//! * **A key the code reads but the table omits** → melange refuses a deck over
//!   a parameter it actually honors, or (on the warn-only op-amp/orphan paths)
//!   reports an honored parameter as unrecognized. That is not hypothetical:
//!   `SHOT_GAMMA2` is read by `codegen::ir::noise`, not by the triode resolver,
//!   so the triode list omitted it and melange warned "unrecognized" about a
//!   parameter it was using. The same shape produced the VCA `THD` false
//!   warning that this table replaced.
//! * **A key the table lists but nothing reads** → melange silently accepts and
//!   ignores it. A deck author gets no diagnostic and a circuit that does not
//!   match the card they wrote.
//!
//! So the tables cannot be maintained by reading one resolver. These tests read
//! the crate's own source and assert both directions agree with the code that
//! consumes the keys. They are deliberately source-text checks rather than a
//! second hand-written list: a second list is the thing that drifts.
//!
//! **This is a diagnostics guard, not a behaviour test.** It says nothing about
//! whether a key is honored *correctly* — the maintainer's rule for that is
//! unchanged: test-compile and grep the emitted const.

use melange_solver::model_params::{ModelClass, ALL_CLASSES};
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

// ---------------------------------------------------------------------------
// Source access
// ---------------------------------------------------------------------------

fn src_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("src")
}

/// Every `.rs` file under `src/`, so a new reader in a new file is covered
/// without anyone remembering to add it here.
fn rust_files() -> Vec<PathBuf> {
    fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
        for entry in std::fs::read_dir(dir).expect("read src dir") {
            let path = entry.expect("dir entry").path();
            if path.is_dir() {
                walk(&path, out);
            } else if path.extension().is_some_and(|e| e == "rs") {
                out.push(path);
            }
        }
    }
    let mut out = Vec::new();
    walk(&src_dir(), &mut out);
    assert!(out.len() > 5, "source walk found suspiciously few files");
    out
}

/// File contents with every run of whitespace collapsed to a single space, so
/// the textual probes below are immune to rustfmt line wrapping.
fn read_normalized(path: &Path) -> String {
    let raw = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path:?}: {e}"));
    let mut out = String::with_capacity(raw.len());
    let mut in_ws = false;
    for ch in raw.chars() {
        if ch.is_whitespace() {
            if !in_ws {
                out.push(' ');
                in_ws = true;
            }
        } else {
            out.push(ch);
            in_ws = false;
        }
    }
    out
}

fn normalized(rel: &str) -> String {
    read_normalized(&src_dir().join(rel))
}

// ---------------------------------------------------------------------------
// Textual probes
// ---------------------------------------------------------------------------

/// A `.model`-key-shaped string literal: uppercase, as the parser stores keys.
fn is_key_shaped(s: &str) -> bool {
    !s.is_empty()
        && s.starts_with(|c: char| c.is_ascii_uppercase())
        && s.chars()
            .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
}

/// Every double-quoted literal in `src`, unescaped forms only (no `\"`, which
/// none of the probed idioms use).
fn string_literals(src: &str) -> Vec<String> {
    let bytes: Vec<char> = src.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == '"' {
            let start = i + 1;
            let mut j = start;
            while j < bytes.len() && bytes[j] != '"' {
                if bytes[j] == '\\' {
                    j += 1;
                }
                j += 1;
            }
            if j >= bytes.len() {
                break;
            }
            out.push(bytes[start..j].iter().collect::<String>());
            i = j + 1;
        } else {
            i += 1;
        }
    }
    out
}

/// Literals passed to `pattern`-prefixed calls, e.g. every `"IS"` in
/// `lookup_model_param(netlist, model, "IS")`.
fn literals_after(src: &str, pattern: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let mut rest = src;
    while let Some(pos) = rest.find(pattern) {
        rest = &rest[pos + pattern.len()..];
        if let Some(end) = rest.find('"') {
            let lit = &rest[..end];
            if is_key_shaped(lit) {
                out.insert(lit.to_string());
            }
        }
    }
    out
}

/// Key-shaped match-arm patterns: the `"KEY"` in `"KEY" =>`.
fn match_arm_keys(src: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let mut rest = src;
    while let Some(pos) = rest.find("\" =>") {
        let head = &rest[..pos];
        if let Some(open) = head.rfind('"') {
            let lit = &head[open + 1..];
            if is_key_shaped(lit) {
                out.insert(lit.to_string());
            }
        }
        rest = &rest[pos + 4..];
    }
    out
}

/// The body of `fn <name>`, up to the next item at the same indentation.
fn fn_body<'a>(src: &'a str, name: &str) -> &'a str {
    let needle = format!("fn {name}(");
    let start = src
        .find(&needle)
        .unwrap_or_else(|| panic!("fn {name} not found — did it get renamed?"));
    let rest = &src[start + needle.len()..];
    // Normalized text puts every item on one line; the next ` fn ` or the end
    // of the impl block bounds this one.
    match rest.find(" fn ") {
        Some(end) => &rest[..end],
        None => rest,
    }
}

// ---------------------------------------------------------------------------
// Direction 1: everything the code reads must be in the table
// ---------------------------------------------------------------------------

/// `(resolver fn, class)` — the codegen resolvers, which read their keys via
/// `lookup_model_param(netlist, model, "KEY")`.
const RESOLVERS: &[(&str, ModelClass)] = &[
    ("resolve_diode_params", ModelClass::Diode),
    ("resolve_bjt_params", ModelClass::Bjt),
    ("resolve_jfet_params", ModelClass::Jfet),
    ("resolve_mosfet_params", ModelClass::Mosfet),
    ("resolve_tube_params", ModelClass::Triode),
    ("resolve_pentode_params", ModelClass::Pentode),
    ("resolve_vca_params", ModelClass::Vca),
    ("resolve_ldr_params", ModelClass::Ldr),
    ("resolve_glow_params", ModelClass::Glow),
];

#[test]
fn every_key_a_resolver_reads_is_in_that_class_table() {
    let ir = normalized("codegen/ir/mod.rs");
    for (func, class) in RESOLVERS {
        let body = fn_body(&ir, func);
        let keys = literals_after(body, "lookup_model_param(netlist, model, \"");
        assert!(
            !keys.is_empty(),
            "{func}: no lookup_model_param(…, \"KEY\") calls found — the probe \
             has gone stale and this test is checking nothing"
        );
        for key in &keys {
            assert!(
                class.is_honored(key),
                "{func} reads .model key '{key}', but it is missing from the \
                 {} honored table in src/model_params.rs. As written, a deck \
                 using '{key}' is REFUSED by check_model_params even though \
                 melange reads it. Add it to the table (and say in a comment \
                 what reads it, if it is not this resolver).",
                class.label()
            );
        }
    }
}

#[test]
fn every_key_the_mna_opamp_and_vca_loops_assign_is_in_the_table() {
    // The op-amp is 0D (a linear VCCS stamped straight into G), so its model
    // card is resolved in mna.rs, not by a codegen resolver. Same for the VCA's
    // MODE flag. Those match arms are the readers; the table must cover them.
    let mna = normalized("mna.rs");
    for (start_anchor, end_anchor, class) in [
        (
            "if m.model_type != \"OA\"",
            "ModelClass::Opamp,",
            ModelClass::Opamp,
        ),
        (
            "if m.model_type != \"VCA\"",
            "ModelClass::Vca,",
            ModelClass::Vca,
        ),
    ] {
        let start = mna
            .find(start_anchor)
            .unwrap_or_else(|| panic!("anchor {start_anchor:?} not found in mna.rs"));
        let end = mna[start..]
            .find(end_anchor)
            .unwrap_or_else(|| panic!("anchor {end_anchor:?} not found after {start_anchor:?}"))
            + start;
        let keys = match_arm_keys(&mna[start..end]);
        assert!(
            !keys.is_empty(),
            "no key-shaped match arms between {start_anchor:?} and {end_anchor:?} \
             — the probe has gone stale"
        );
        for key in &keys {
            assert!(
                class.is_honored(key),
                "mna.rs assigns .model key '{key}' for the {} model card, but it \
                 is missing from that honored table in src/model_params.rs — so \
                 melange would report a parameter it honors as unrecognized",
                class.label()
            );
        }
    }
}

#[test]
fn every_key_the_noise_ir_reads_is_honored_by_some_class() {
    // `codegen::ir::noise` reads model cards directly (SHOT_GAMMA2,
    // PARTITION_F, the BJT parasitic RB/RC/RE, KF/AF). These keys belong to a
    // class's table even though no resolver reads them — the exact gap that let
    // SHOT_GAMMA2 be reported as unrecognized while it was in use.
    let noise = normalized("codegen/ir/noise.rs");
    let mut keys = literals_after(&noise, "eq_ignore_ascii_case(\"");
    keys.extend(match_arm_keys(&noise));
    assert!(
        !keys.is_empty(),
        "no model-key reads found in noise.rs — the probe has gone stale"
    );
    for key in &keys {
        assert!(
            ALL_CLASSES.iter().any(|c| c.is_honored(key)),
            "codegen::ir::noise reads .model key '{key}', but no class table in \
             src/model_params.rs honors it — a card carrying '{key}' is refused \
             (or warned about) for a parameter melange uses"
        );
    }
}

// ---------------------------------------------------------------------------
// Direction 2: everything in the table must have a reader
// ---------------------------------------------------------------------------

/// Keys read under a name built at run time rather than as a literal, so a
/// source-text scan cannot see them. Each entry needs a reason.
///
/// `K1..K4` / `TAU1..TAU4`: the glow-lamp resolver loops
/// `format!("K{}", i + 1)` / `format!("TAU{}", i + 1)` over its relaxation
/// sections.
const DYNAMICALLY_NAMED: &[(&str, &str)] = &[
    ("K2", "glow section loop: format!(\"K{}\", i + 1)"),
    ("K3", "glow section loop: format!(\"K{}\", i + 1)"),
    ("K4", "glow section loop: format!(\"K{}\", i + 1)"),
    ("TAU1", "glow section loop: format!(\"TAU{}\", i + 1)"),
    ("TAU2", "glow section loop: format!(\"TAU{}\", i + 1)"),
    ("TAU3", "glow section loop: format!(\"TAU{}\", i + 1)"),
    ("TAU4", "glow section loop: format!(\"TAU{}\", i + 1)"),
];

#[test]
fn every_table_key_has_a_reader_in_the_crate() {
    // Generous on purpose: any key-shaped literal anywhere in src/ (outside the
    // table module itself) counts as a reader. A key that appears NOWHERE is
    // unambiguously dead — melange accepts it on a card and drops it silently,
    // which is the failure this project treats as a showstopper.
    let mut literals: BTreeSet<String> = BTreeSet::new();
    let table_module = src_dir().join("model_params.rs");
    for path in rust_files() {
        if path == table_module {
            continue;
        }
        let text = std::fs::read_to_string(&path).expect("read source file");
        literals.extend(
            string_literals(&text)
                .into_iter()
                .filter(|s| is_key_shaped(s)),
        );
    }

    let mut orphans: Vec<String> = Vec::new();
    for class in ALL_CLASSES {
        for key in class.honored() {
            if literals.contains(*key) {
                continue;
            }
            if DYNAMICALLY_NAMED.iter().any(|(k, _)| k == key) {
                continue;
            }
            orphans.push(format!("{} / {}", class.label(), key));
        }
    }
    assert!(
        orphans.is_empty(),
        "these .model keys are in a src/model_params.rs table but appear \
         nowhere in the crate source, so melange accepts them on a card and \
         silently ignores them: {orphans:?}\n\
         Either wire the key up, drop it from the table (so it is reported as \
         unrecognized), move it to the class's `unimplemented` list (so the \
         card is accepted with a warning naming what the omission costs), or \
         add it to DYNAMICALLY_NAMED with the reason it has no literal."
    );
}

#[test]
fn every_class_has_a_non_empty_table() {
    for class in ALL_CLASSES {
        assert!(
            !class.honored().is_empty(),
            "{}: empty honored table would refuse every parameter on the card",
            class.label()
        );
    }
}

// ---------------------------------------------------------------------------
// The parser's type dispatch must agree with the tables
// ---------------------------------------------------------------------------

#[test]
fn every_model_type_the_parser_accepts_maps_to_a_class() {
    // `parser.rs` enforces which `.model` type an element may reference. Any
    // type it accepts must resolve to a table here, or the orphan-card pass
    // silently skips cards of that type.
    for (model_type, expected) in [
        ("D", ModelClass::Diode),
        ("NPN", ModelClass::Bjt),
        ("PNP", ModelClass::Bjt),
        ("NJF", ModelClass::Jfet),
        ("PJF", ModelClass::Jfet),
        ("NMOS", ModelClass::Mosfet),
        ("PMOS", ModelClass::Mosfet),
        ("TRIODE", ModelClass::Triode),
        ("VT", ModelClass::Triode),
        ("TUBE", ModelClass::Triode),
        ("VP", ModelClass::Pentode),
        ("PENTODE", ModelClass::Pentode),
        ("OA", ModelClass::Opamp),
        ("VCA", ModelClass::Vca),
        ("LDR", ModelClass::Ldr),
        ("NEON", ModelClass::Glow),
    ] {
        assert_eq!(
            ModelClass::from_model_type(model_type),
            Some(expected),
            "model type '{model_type}' (accepted by the parser) has no table"
        );
    }
    // Keep the parser's own list in view: if a type is added there, this test's
    // list must grow with it.
    let parser = normalized("parser.rs");
    for token in [
        "\"D\"",
        "\"NPN\"",
        "\"PNP\"",
        "\"NJ\"",
        "\"PJ\"",
        "\"NM\"",
        "\"PM\"",
        "\"TRIODE\"",
        "\"VT\"",
        "\"TUBE\"",
        "\"VP\"",
        "\"PENTODE\"",
        "\"OA\"",
        "\"VCA\"",
        "\"LDR\"",
        "\"NEON\"",
    ] {
        assert!(
            parser.contains(token),
            "parser.rs no longer mentions model type {token} — the type rules \
             moved or changed; re-check the ModelClass::from_model_type mapping"
        );
    }
}
