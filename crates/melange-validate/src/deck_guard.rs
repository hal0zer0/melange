//! Pre-flight guards that refuse a validation run the two engines cannot
//! honestly compare.
//!
//! `melange validate` hands the author's own `.cir` to ngspice and reports a
//! correlation between the two engines' outputs. That number means something
//! only if both engines read the *same circuit*. This module catches the two
//! ways they currently don't, **before** ngspice is invoked:
//!
//! 1. **Value tokens the two engines read differently.** melange accepts the
//!    SI-prefix infix form (`4k7` = 4.7 kΩ, `6n8` = 6.8 nF), where the scale
//!    letter stands in for the decimal point. ngspice does not implement it:
//!    it reads the mantissa, applies a scale letter if the next character is
//!    one, and **silently discards the rest of the token**. Measured on system
//!    ngspice 42 (`print @r[resistance]` after `.op`):
//!
//!    | token | melange | ngspice | error |
//!    |-------|---------|---------|-------|
//!    | `4k7` | 4700    | 4000    | 17.5 % |
//!    | `2M2` | 2.2e6   | 0.002   | 9 orders |
//!    | `1n5` | 1.5e-9  | 1e-9    | 33 % |
//!    | `1m5` | 1.5e6   | 0.001   | 9 orders |
//!
//!    None of these is a parse error on either side. A deck containing `4k7`
//!    validates a 4.7 kΩ circuit against a 4.0 kΩ circuit and blames the
//!    difference on the solver.
//!
//! 2. **Devices ngspice has no element or model for** — melange's op-amp
//!    (`U`/`OA`), LDR (`O`/`LDR`), VCA (`Y`/`VCA`) and glow-discharge
//!    (`N`/`NEON`). melange writes them into the reference deck anyway and
//!    relays ngspice's complaint, which points at the author's (correct)
//!    `.model` line.
//!
//! A third class used to be *refused* here: `.tolerance` / `.mismatch` value
//! jitter, which is applied on melange's side only. That refusal is gone — it
//! was an interim measure, and it made the README's flagship example
//! un-validatable. Validate now compiles the melange side with unit variation
//! disabled ([`melange_solver::parser::ParseOptions::disable_unit_variation`],
//! set in `run_melange_solver_from_str`) and compares nominal against nominal,
//! which is what the docs already told authors to do by hand. What remains here
//! is [`unit_variation_note`], which names the disabled directives on the
//! result line so the number is never read as a jittered one.
//!
//! ## Refuse, not warn
//!
//! Every hazard here is a *refusal*. The run's product is a correlation number
//! presented as authoritative; a warning printed above it reads as a footnote
//! to that number rather than a retraction of it. The existing `validate`
//! warnings (floating cap-only DC islands) are for things melange *handled* —
//! these are things it cannot. Refusing costs the author one edit; a false
//! PASS costs them their trust in every other number melange prints.

use melange_solver::parser::{parse_value, Element, Netlist};

/// A reason the deck cannot be compared between melange and ngspice.
#[derive(Debug, Clone, PartialEq)]
pub enum DeckHazard {
    /// A value token the two engines read as different numbers.
    AmbiguousValue {
        /// 1-based raw source line the statement starts on.
        line_no: usize,
        /// The statement as written (comment-stripped, continuations joined).
        statement: String,
        /// The offending token.
        token: String,
        /// What melange reads it as.
        melange: f64,
        /// What ngspice reads it as.
        ngspice: f64,
    },
    /// A device ngspice has no element or model for.
    UnsupportedDevice {
        /// Element name as written (e.g. `U1`).
        name: String,
        /// Human-readable device kind (e.g. `op-amp`).
        kind: &'static str,
        /// Why ngspice cannot take it, and what to do instead.
        detail: &'static str,
    },
}

/// What ngspice reads a value token as, or `None` if ngspice would not read a
/// number from it at all.
///
/// Emulates ngspice's `INPevaluate`: scan a float, then look at what follows —
/// `MEG`/`MIL` as three-letter forms, otherwise a single scale letter
/// (`T G K M U N P F`, where a bare `M` is **milli**) — then discard every
/// remaining character. `10kohm` is 10 k, `4k7` is 4 k, `2M2` is 2 m.
///
/// The expectations in [`tests::ngspice_reading_matches_measured_ngspice`] were
/// measured against system ngspice 42, not derived from this function.
pub fn ngspice_reading(token: &str) -> Option<f64> {
    let b = token.as_bytes();
    let mut i = 0;
    // Optional sign.
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let digits_start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    if i < b.len() && b[i] == b'.' {
        i += 1;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
    }
    if i == digits_start {
        // No digits at all (e.g. `DC`, `SIN(`, a node name) — not a number.
        return None;
    }
    // Optional exponent. Only consumed when it is well-formed, so `1e` keeps
    // the `e` for the (no-op) scale-letter step rather than failing the parse.
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let mut j = i + 1;
        if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
            j += 1;
        }
        let exp_digits = j;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_digits {
            i = j;
        }
    }
    let mantissa: f64 = token[..i].parse().ok()?;

    let rest = token[i..].to_ascii_uppercase();
    let scale = if rest.starts_with("MEG") {
        1e6
    } else if rest.starts_with("MIL") {
        25.4e-6
    } else {
        match rest.as_bytes().first() {
            Some(b'T') => 1e12,
            Some(b'G') => 1e9,
            Some(b'K') => 1e3,
            Some(b'M') => 1e-3,
            Some(b'U') => 1e-6,
            Some(b'N') => 1e-9,
            Some(b'P') => 1e-12,
            Some(b'F') => 1e-15,
            _ => 1.0,
        }
    };
    let v = mantissa * scale;
    v.is_finite().then_some(v)
}

/// Do the two engines read this token as the same number?
///
/// `None` from either side means that side would not accept the token as a
/// value at all, which is a different (and loud) failure — not this guard's.
fn value_hazard(token: &str) -> Option<(f64, f64)> {
    let m = parse_value(token).ok()?;
    let n = ngspice_reading(token)?;
    if m == n {
        return None;
    }
    // Relative comparison so float formatting differences (`1e3` vs `1000`)
    // never trip it; an absolute floor catches a pair straddling zero.
    let denom = m.abs().max(n.abs());
    if denom == 0.0 || (m - n).abs() / denom < 1e-9 {
        return None;
    }
    Some((m, n))
}

/// Strip a SPICE inline comment (`;` or `$`), respecting double-quoted regions.
///
/// Mirrors the netlist parser's own `strip_inline_comment` so this scan sees
/// exactly the text the parser sees.
fn strip_inline_comment(line: &str) -> &str {
    let mut in_quote = false;
    for (i, c) in line.char_indices() {
        match c {
            '"' => in_quote = !in_quote,
            ';' | '$' if !in_quote => return &line[..i],
            _ => {}
        }
    }
    line
}

/// One netlist statement: raw start line plus continuation-joined text.
struct Statement {
    line_no: usize,
    text: String,
}

/// Split a deck into statements the way the netlist parser does: strip inline
/// comments, join `+` continuations, and remember the 1-based raw line the
/// statement started on.
fn statements(deck: &str) -> Vec<Statement> {
    let raw: Vec<&str> = deck.lines().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < raw.len() {
        let start_line = i + 1;
        let mut acc = strip_inline_comment(raw[i]).trim().to_string();
        while i + 1 < raw.len() {
            let next = strip_inline_comment(raw[i + 1]).trim();
            if let Some(stripped) = next.strip_prefix('+') {
                acc.push(' ');
                acc.push_str(stripped.trim());
                i += 1;
            } else {
                break;
            }
        }
        out.push(Statement {
            line_no: start_line,
            text: acc,
        });
        i += 1;
    }
    out
}

/// Token indices that hold a *value* on this statement, by element letter.
///
/// This is the whole reason the scan is position-aware rather than a regex over
/// anything shaped like digit-letter-digit. Device and model names routinely
/// look exactly like an infix value — `2N7000`, `2N3904`, `1N4148`, `6K7` — and
/// the two engines *do* read those differently as numbers. They are names, not
/// values, so flagging them would refuse a perfectly good deck. Positional
/// scanning is what tells the two apart.
///
/// Returns an empty range for element letters whose trailing token is a model
/// or subcircuit name (`D Q J M U T P O N Y X S W B`); those statements are
/// still scanned for `key=value` parameters by the caller.
fn bare_value_positions(statement: &str) -> Vec<usize> {
    let tokens: Vec<&str> = statement.split_whitespace().collect();
    let Some(first) = tokens.first() else {
        return Vec::new();
    };
    match first
        .chars()
        .next()
        .map(|c| c.to_ascii_uppercase())
        .unwrap_or(' ')
    {
        // `<R|C|L>name n+ n- value [params]`, `Kname L1 L2 coupling`.
        'R' | 'C' | 'L' | 'K' => {
            if tokens.len() > 3 {
                vec![3]
            } else {
                Vec::new()
            }
        }
        // `Ename out+ out- ctrl+ ctrl- gain`, `Gname out+ out- ctrl+ ctrl- gm`.
        'E' | 'G' => {
            if tokens.len() > 5 {
                vec![5]
            } else {
                Vec::new()
            }
        }
        // Sources: everything past the node pair is a value or a keyword
        // (`DC`, `AC`, `PULSE(`…). Keywords are filtered out by the
        // both-engines-parse-it test, so scanning the whole tail is safe.
        'V' | 'I' => (3..tokens.len()).collect(),
        _ => Vec::new(),
    }
}

/// Melange-only directives whose lines never reach ngspice, so a value token in
/// them cannot be read two ways. `.inject` is the exception: it is *translated*
/// into a real resistor whose value token is passed through verbatim.
fn is_stripped_melange_directive(statement: &str) -> bool {
    let Some(first) = statement.split_whitespace().next() else {
        return false;
    };
    let first = first.to_ascii_lowercase();
    if first == ".inject" {
        return false;
    }
    melange_solver::parser::MELANGE_ONLY_DIRECTIVES
        .iter()
        .any(|d| *d == first)
}

/// Scan a deck for value tokens melange and ngspice read differently.
///
/// **Call this on the deck as it will reach ngspice** — after
/// `substitute_dynamic_element_defaults` (which rewrites `.pot`/`.switch`
/// element values to plain numerics, so an infix *nominal* on a pot resistor is
/// correctly not a hazard) and before the tube/pentode/Thevenin rewrites (whose
/// generated text is not author-written and is numeric by construction).
fn scan_ambiguous_values(deck: &str) -> Vec<DeckHazard> {
    let mut hazards = Vec::new();
    for (idx, stmt) in statements(deck).iter().enumerate() {
        // SPICE line 1 is always the title, on both engines. Skip it.
        if idx == 0 {
            continue;
        }
        let text = stmt.text.trim();
        if text.is_empty() || text.starts_with('*') {
            continue;
        }
        if is_stripped_melange_directive(text) {
            continue;
        }

        let tokens: Vec<&str> = text.split_whitespace().collect();
        let mut candidates: Vec<&str> = Vec::new();

        // Bare positional values (elements only; directives have none we can
        // place safely, and `.tran`/`.print` cards are rewritten by validate).
        if !text.starts_with('.') {
            for i in bare_value_positions(text) {
                if let Some(t) = tokens.get(i) {
                    candidates.push(t);
                }
            }
        }

        // `key=value` parameters, anywhere, on any statement: `.model` cards,
        // `.param`, instance parameters, and `.inject R=…`.
        for t in &tokens {
            if let Some((_, rhs)) = t.split_once('=') {
                candidates.push(rhs);
            }
        }

        for cand in candidates {
            let token = cand.trim_matches(|c| c == '(' || c == ')' || c == ',');
            if token.is_empty() {
                continue;
            }
            if let Some((m, n)) = value_hazard(token) {
                hazards.push(DeckHazard::AmbiguousValue {
                    line_no: stmt.line_no,
                    statement: text.to_string(),
                    token: token.to_string(),
                    melange: m,
                    ngspice: n,
                });
            }
        }
    }
    hazards
}

/// Devices melange supports that ngspice has no element or model for.
///
/// Verified by running each through `melange validate` against system ngspice
/// 42 (see the module tests and the audit recorded there): `U`/`OA`, `O`/`LDR`,
/// `Y`/`VCA` and `N`/`NEON` all abort the reference run. Triodes (`T`) and
/// pentodes (`P`) are **not** listed: `tube_translate` / `pentode_translate`
/// rewrite them into Koren/Reefman B-source subcircuits that ngspice accepts
/// (`tests/data/triode_cc` validates at correlation 1.000000).
fn classify_unsupported(elem: &Element) -> Option<(&str, &'static str, &'static str)> {
    match elem {
        Element::Opamp { name, .. } => Some((
            name,
            "op-amp",
            "ngspice has no op-amp element and no `OA` model type — it parses `U` as a \
             uniform-RC line and then cannot find the model, so it blames your `.model` card, \
             which is correct. To validate this circuit, hand-expand the op-amp on the ngspice \
             side as its macromodel — a VCCS plus output resistance \
             (`G<n> out 0 <in+> <in-> <AOL/ROUT>` + `R<n> out 0 <ROUT>`); see \
             crates/melange-validate/tests/data/opamp_inverting/circuit.cir. That stand-in is \
             linear: it does not reproduce melange's rail clamping, so keep the reference run \
             inside the rails.",
        )),
        Element::Ldr { name, .. } => Some((
            name,
            "LDR / photoresistor",
            "ngspice has no `LDR` model type, and the melange device carries its own \
             brightness-driven state (attack/release time constants) that no ngspice primitive \
             reproduces. `validate` cannot cover this circuit.",
        )),
        Element::Vca { name, .. } => Some((
            name,
            "VCA",
            "ngspice has no `VCA` model type — it parses `Y` as a lossy transmission line. \
             `validate` cannot cover this circuit.",
        )),
        Element::Glow { name, .. } => Some((
            name,
            "glow-discharge / neon lamp",
            "ngspice has no `NEON` model type and no gas-discharge primitive. `validate` cannot \
             cover this circuit.",
        )),
        _ => None,
    }
}

/// Scan a deck for everything that makes a melange/ngspice comparison
/// meaningless.
///
/// Best-effort on the device and jitter checks: a deck melange's own parser
/// cannot read is left to fail with its own (loud) parse error rather than
/// being second-guessed here. The value scan is purely textual and always runs.
pub fn scan_deck(deck: &str) -> Vec<DeckHazard> {
    let mut hazards = scan_ambiguous_values(deck);

    if let Ok(netlist) = Netlist::parse(deck) {
        for elem in &netlist.elements {
            if let Some((name, kind, detail)) = classify_unsupported(elem) {
                hazards.push(DeckHazard::UnsupportedDevice {
                    name: name.to_string(),
                    kind,
                    detail,
                });
            }
        }
    }

    hazards
}

/// The qualifier `validate` puts on its result line when the deck carries live
/// `.tolerance` / `.mismatch` jitter that the comparison ran without.
///
/// Returns `None` for a deck with no live jitter directive — the 14 shipped
/// validation decks are all in that set and their output must not grow noise.
///
/// The text goes ON the PASSED/FAILED line, not in a preamble above it. The
/// same reasoning as the refusals in this module: the run's product is a
/// number presented as authoritative, and a footnote does not retract a number.
/// It names *which* directives were disabled and the seed that was therefore
/// not exercised, so a reader can tell this correlation apart from one measured
/// on the unit the deck actually describes.
///
/// A `.mismatch` card whose tolerances are all zero is a documented no-op
/// (`apply_mismatch` returns the nominal unchanged), so it is not reported —
/// only a live, non-zero draw is. Best-effort: a deck melange's own parser
/// cannot read produces no note and fails later with its own parse error.
pub fn unit_variation_note(deck: &str) -> Option<String> {
    // Plain `Netlist::parse` is correct here: this netlist is read for its
    // DIRECTIVES and thrown away, it never reaches the solver, and the draw
    // does not touch `mismatch_specs` / `tolerance_*` / `seed`. The melange
    // side's parse is the one in `run_melange_solver_from_str`.
    let netlist = Netlist::parse(deck).ok()?;

    let mut disabled: Vec<String> = Vec::new();
    for spec in &netlist.mismatch_specs {
        if spec.params.iter().any(|(_, v)| *v != 0.0) {
            let entry = format!(".mismatch {}", spec.device_class);
            if !disabled.contains(&entry) {
                disabled.push(entry);
            }
        }
    }
    if netlist.tolerance_r != 0.0 || netlist.tolerance_c != 0.0 || netlist.tolerance_l != 0.0 {
        disabled.push(".tolerance".to_string());
    }
    if disabled.is_empty() {
        return None;
    }

    // `.seed` is the thing a reader would otherwise assume was exercised. Say
    // which one, and say so even when the deck never wrote a `.seed` line —
    // the draw would have used 0.
    let seed = match netlist.seed {
        Some(s) => format!("seed {s}"),
        None => "the default seed 0".to_string(),
    };
    Some(format!(
        "nominal values: {} disabled for this comparison; {} not exercised",
        disabled.join(", "),
        seed
    ))
}

/// Render hazards as the refusal message `validate` prints instead of a
/// correlation between two different circuits.
pub fn format_refusal(hazards: &[DeckHazard]) -> String {
    let mut out = String::from(
        "validate refused: melange and ngspice would not simulate the same circuit from this \
         deck, so any correlation between them would be meaningless.\n",
    );

    let ambiguous: Vec<&DeckHazard> = hazards
        .iter()
        .filter(|h| matches!(h, DeckHazard::AmbiguousValue { .. }))
        .collect();
    if !ambiguous.is_empty() {
        out.push_str(
            "\nValue tokens the two engines read differently (melange accepts the SI-prefix \
             infix form, where the scale letter replaces the decimal point; ngspice does not — \
             it reads the mantissa, applies the scale letter, and discards the rest of the \
             token):\n",
        );
        for h in &ambiguous {
            if let DeckHazard::AmbiguousValue {
                line_no,
                statement,
                token,
                melange,
                ngspice,
            } = h
            {
                out.push_str(&format!(
                    "  line {}: {}\n    '{}' — melange reads {:.6e}, ngspice reads {:.6e}\n",
                    line_no, statement, token, melange, ngspice
                ));
            }
        }
        out.push_str(
            "  Fix: write the value with an explicit decimal point and a suffix scale \
             (4k7 -> 4.7k, 6n8 -> 6.8n, 2M2 -> 2.2Meg). melange still accepts the infix form \
             everywhere else; it is only `validate` that cannot use it, because the reference \
             engine does not implement it.\n",
        );
    }

    let unsupported: Vec<&DeckHazard> = hazards
        .iter()
        .filter(|h| matches!(h, DeckHazard::UnsupportedDevice { .. }))
        .collect();
    if !unsupported.is_empty() {
        out.push_str("\nDevices ngspice cannot simulate:\n");
        // Group by kind: a mastering-EQ deck can carry eight op-amps, and the
        // same paragraph eight times buries the one thing the reader needs.
        let mut groups: Vec<(&'static str, &'static str, Vec<&str>)> = Vec::new();
        for h in &unsupported {
            if let DeckHazard::UnsupportedDevice { name, kind, detail } = h {
                match groups.iter_mut().find(|(k, _, _)| k == kind) {
                    Some((_, _, names)) => names.push(name),
                    None => groups.push((kind, detail, vec![name])),
                }
            }
        }
        for (kind, detail, names) in groups {
            out.push_str(&format!(
                "  circuit contains {} {}{} ({})\n    {}\n",
                names.len(),
                kind,
                if names.len() == 1 { "" } else { "s" },
                names.join(", "),
                detail
            ));
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The `ngspice` column was MEASURED against system ngspice 42, not derived
    /// from `ngspice_reading`: a deck of `R<i> n<i> 0 <token>` lines run under
    /// `.op`, read back with `print @r<i>[resistance]`. Keep it that way — the
    /// point of this table is that it is evidence, not a restatement of the
    /// code it checks.
    #[test]
    fn ngspice_reading_matches_measured_ngspice() {
        let measured: &[(&str, f64)] = &[
            ("4k7", 4.0e3),
            ("2M2", 2.0e-3),
            ("1n5", 1.0e-9),
            ("6n8", 6.0e-9),
            ("3k3", 3.0e3),
            ("1meg", 1.0e6),
            ("1MEG", 1.0e6),
            ("10k", 1.0e4),
            ("4.7u", 4.7e-6),
            ("10p", 1.0e-11),
            ("10pF", 1.0e-11),
            ("1fF", 1.0e-15),
            ("10f", 1.0e-14),
            ("1F", 1.0e-15),
            ("22n", 2.2e-8),
            ("1m5", 1.0e-3),
            ("2G2", 2.0e9),
            ("1T5", 1.0e12),
            ("100", 1.0e2),
            ("1e3", 1.0e3),
            ("4.7e3", 4.7e3),
            ("1u", 1.0e-6),
            ("1mil", 2.54e-5),
            ("1k", 1.0e3),
            ("2.2k", 2.2e3),
            ("1Meg", 1.0e6),
            ("470", 4.7e2),
            ("1p5", 1.0e-12),
            ("3u3", 3.0e-6),
            ("2t2", 2.0e12),
            ("1g5", 1.0e9),
            ("0.47u", 4.7e-7),
            ("47n", 4.7e-8),
            ("1MEGOHM", 1.0e6),
            ("4k7ohm", 4.0e3),
            ("1k2v", 1.0e3),
        ];
        for (tok, expect) in measured {
            let got = ngspice_reading(tok).unwrap_or_else(|| panic!("{tok}: no reading"));
            assert!(
                (got - expect).abs() <= expect.abs() * 1e-12,
                "{tok}: emulator {got:e}, measured ngspice {expect:e}"
            );
        }
    }

    #[test]
    fn non_numeric_tokens_are_not_values() {
        for tok in ["DC", "AC", "SIN(", "out", "TL072", "", "+", "."] {
            assert_eq!(ngspice_reading(tok), None, "{tok}");
        }
    }

    fn ambiguous_tokens(deck: &str) -> Vec<String> {
        scan_deck(deck)
            .into_iter()
            .filter_map(|h| match h {
                DeckHazard::AmbiguousValue { token, .. } => Some(token),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn infix_value_in_element_position_is_caught() {
        let deck = "title\nR1 a b 4k7\nR2 b 0 1k\n";
        let hits = ambiguous_tokens(deck);
        assert_eq!(hits, vec!["4k7".to_string()]);
    }

    #[test]
    fn line_number_is_the_raw_source_line() {
        let deck = "title\n* a comment\n\nR1 a b 2M2\n";
        match &scan_deck(deck)[0] {
            DeckHazard::AmbiguousValue {
                line_no, statement, ..
            } => {
                assert_eq!(*line_no, 4);
                assert_eq!(statement, "R1 a b 2M2");
            }
            other => panic!("{other:?}"),
        }
    }

    /// The trap this scan exists to avoid: device and model NAMES are shaped
    /// exactly like infix values and do read differently as numbers, but they
    /// are not values.
    #[test]
    fn device_and_model_names_are_not_values() {
        let deck = "\
title
M1 drain gate src 0 2N7000
Q1 c b e 2N3904
D1 a k 1N4148
P1 plate grid cath screen 6K7
.model 2N7000 NMOS(VTO=2.0 KP=0.2)
.model 2N3904 NPN(IS=1e-14 BF=200)
.model 1N4148 D(IS=2.52e-9 N=1.752)
.model 6K7 VP()
";
        assert!(ambiguous_tokens(deck).is_empty(), "{:?}", scan_deck(deck));
    }

    #[test]
    fn comments_do_not_trip_the_scan() {
        let deck = "\
title
* R64 1K5 in the wafer-B ladder (3K9/3K3/2K7 from the top)
R1 a b 1k        ; was 2k2 on the 1967 board
* --- 6k8 in parallel with the 100k load ---
";
        assert!(ambiguous_tokens(deck).is_empty(), "{:?}", scan_deck(deck));
    }

    /// Line 1 is the title on BOTH engines, so an infix-looking title is not a
    /// value on either side.
    #[test]
    fn title_line_is_not_scanned() {
        let deck = "6K7 Variable-Mu Gain-Reduction Stage\nR1 a b 1k\n";
        assert!(ambiguous_tokens(deck).is_empty());
    }

    /// Melange-only directives are stripped from the reference deck, so their
    /// tokens never reach ngspice and cannot be read two ways. (`.switch`
    /// position values are separately substituted as plain numerics before this
    /// scan runs.)
    #[test]
    fn stripped_melange_directives_are_not_scanned() {
        let deck = "\
title
R1 a b 1k
.switch C_hfb,L_hfb 18n/150m 8n2/47m 5n6/33m 3n3/27m \"HF Boost Freq\"
.pot R1 100 100k 4k7
";
        assert!(ambiguous_tokens(deck).is_empty(), "{:?}", scan_deck(deck));
    }

    /// `.inject` IS translated into a real resistor whose value token is passed
    /// through verbatim, so it is scanned.
    #[test]
    fn inject_value_is_scanned() {
        let deck = "title\nR1 a b 1k\n.inject a ret R=4k7\n";
        assert_eq!(ambiguous_tokens(deck), vec!["4k7".to_string()]);
    }

    #[test]
    fn model_card_parameters_are_scanned() {
        let deck = "title\nR1 a b 1k\nD1 a b DX\n.model DX D(IS=2n5 N=1.752)\n";
        assert_eq!(ambiguous_tokens(deck), vec!["2n5".to_string()]);
    }

    /// A trailing `f` in an ELEMENT value is the Farad unit to melange and femto
    /// to ngspice — the same "two engines, two circuits" class, caught by the
    /// same both-readings comparison rather than by a special case.
    #[test]
    fn farad_versus_femto_is_caught() {
        let deck = "title\nC1 a 0 10f\n";
        match &scan_deck(deck)[0] {
            DeckHazard::AmbiguousValue {
                token,
                melange,
                ngspice,
                ..
            } => {
                assert_eq!(token, "10f");
                assert_eq!(*melange, 10.0);
                assert!((*ngspice - 1e-14).abs() < 1e-26, "{ngspice:e}");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn plain_values_are_never_flagged() {
        let deck = "\
title
R1 a b 4.7k
R2 b c 1Meg
C1 c 0 100n
C2 c 0 10pF
C3 c 0 1fF
L1 c d 100u
V1 vcc 0 DC 9
V2 in 0 SIN(0 1 1k)
E1 o 0 a b 10
G1 o 0 a b 2666.667
.model DX D(IS=2.52e-9 RS=0.568 N=1.752)
";
        assert!(ambiguous_tokens(deck).is_empty(), "{:?}", scan_deck(deck));
    }

    #[test]
    fn opamp_is_reported_unsupported() {
        let deck = "\
title
Rin in inv 10k
Rf inv out 100k
U1 0 inv out TL072
Cl out 0 1p
.model TL072 OA(AOL=1e5 ROUT=75 VSAT=13)
";
        let hazards = scan_deck(deck);
        assert!(
            hazards.iter().any(|h| matches!(
                h,
                DeckHazard::UnsupportedDevice { name, kind, .. }
                    if name == "U1" && *kind == "op-amp"
            )),
            "{hazards:?}"
        );
        assert!(format_refusal(&hazards).contains("no op-amp element"));
    }

    /// Jitter directives are no longer a refusal: validate disables them on the
    /// melange side and says so on the result line. A deck carrying nothing but
    /// `.tolerance` must now run.
    #[test]
    fn jitter_directives_are_not_a_refusal() {
        let deck = "title\nR1 a b 1k\nC1 b 0 10n\n.tolerance R=0.01\n.mismatch D IS=0.05\n";
        assert!(scan_deck(deck).is_empty(), "{:?}", scan_deck(deck));
    }

    #[test]
    fn note_names_both_directives_and_the_seed() {
        let deck = "\
title
R1 a b 1k
D1 b 0 DX
T1 p g k 12AX7
.model DX D(IS=2.52e-9)
.model 12AX7 TRIODE(MU=100)
.seed 4142
.mismatch T MU=0.09 KG1=0.20
.tolerance R=0.01
";
        let note = unit_variation_note(deck).expect("note");
        assert_eq!(
            note,
            "nominal values: .mismatch T, .tolerance disabled for this comparison; \
             seed 4142 not exercised"
        );
    }

    /// No `.seed` line still means a draw — seed 0 — so the note says which.
    #[test]
    fn note_names_the_default_seed_when_none_is_written() {
        let deck = "title\nR1 a b 1k\n.tolerance R=0.01\n";
        assert_eq!(
            unit_variation_note(deck).as_deref(),
            Some(
                "nominal values: .tolerance disabled for this comparison; \
                  the default seed 0 not exercised"
            )
        );
    }

    /// The 14 shipped validation decks carry no jitter, and their output must
    /// not grow a qualifier. An all-zero `.mismatch` is a documented no-op, so
    /// it is not reported either.
    #[test]
    fn no_note_without_live_jitter() {
        assert_eq!(unit_variation_note("title\nR1 a b 1k\n"), None);
        assert_eq!(
            unit_variation_note(
                "title\nR1 a b 1k\nD1 a 0 DX\n.model DX D(IS=1n)\n.mismatch D IS=0\n"
            ),
            None
        );
    }

    #[test]
    fn shipped_validation_decks_carry_no_jitter_qualifier() {
        let data = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
        let mut noisy = Vec::new();
        for entry in std::fs::read_dir(&data).expect("tests/data") {
            let cir = entry.expect("dir entry").path().join("circuit.cir");
            if !cir.is_file() {
                continue;
            }
            let deck = std::fs::read_to_string(&cir).expect("read deck");
            if let Some(note) = unit_variation_note(&deck) {
                noisy.push(format!("{}: {}", cir.display(), note));
            }
        }
        assert!(noisy.is_empty(), "{}", noisy.join("\n"));
    }

    /// Every deck the validation suite ships must be comparable — if one is
    /// not, that is a finding about the deck, not something to suppress here.
    #[test]
    fn shipped_validation_decks_are_comparable() {
        let data = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
        let mut offenders = Vec::new();
        for entry in std::fs::read_dir(&data).expect("tests/data") {
            let dir = entry.expect("dir entry").path();
            let cir = dir.join("circuit.cir");
            if !cir.is_file() {
                continue;
            }
            let deck = std::fs::read_to_string(&cir).expect("read deck");
            let hazards = scan_deck(&deck);
            if !hazards.is_empty() {
                offenders.push(format!("{}: {}", cir.display(), format_refusal(&hazards)));
            }
        }
        assert!(offenders.is_empty(), "{}", offenders.join("\n"));
    }
}
