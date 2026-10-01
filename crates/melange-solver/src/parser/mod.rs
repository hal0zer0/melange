//! SPICE netlist parser.
//!
//! Parses a subset of SPICE sufficient for audio circuits:
//! - Components: R, C, L, V (DC/AC), I, D, Q, J, M, U (op-amp), E (VCVS), G (VCCS), Y (VCA), X
//! - Directives: .model, .subckt, .param, .pot, .wiper, .switch, .gang, .linearize, .runtime, .input_impedance, .integrator, .end
//!
//! The parser builds an AST (Abstract Syntax Tree) representation
//! of the circuit that can be processed by the MNA assembler.
//!
//! # Input size caps
//!
//! To prevent unbounded memory use on malicious or malformed input, the parser
//! rejects any netlist whose raw or shape-level size exceeds the following caps.
//! Each cap produces a specific `ParseError` before any downstream allocation.
//!
//! - [`MAX_NETLIST_BYTES`] — raw UTF-8 byte length of the input string
//! - [`MAX_NODE_NAME_LEN`] — characters per node name
//! - [`MAX_TOTAL_ELEMENTS`] — parsed elements before subcircuit expansion
//! - [`MAX_MODELS`] — `.model` directives
//! - [`MAX_MODEL_PARAMS`] — parameters per `.model` line
//!
//! These are intentionally generous — real circuits use a tiny fraction of
//! each — but finite, so a 100 MB node name or 10 million element netlist
//! fails fast with a clear error instead of OOMing.

mod directive_parse;
mod directives;
mod element;
mod element_parse;
mod names;
mod netlist;
mod subckt;
mod validate;
mod values;

pub use directives::*;
pub use element::*;
pub use names::*;
pub use netlist::*;
pub use values::*;

/// Maximum raw byte length of a netlist string accepted by [`Netlist::parse`].
///
/// Rejected with a specific error before any line-splitting or allocation.
/// 10 MB is orders of magnitude above the largest real-world netlists.
pub const MAX_NETLIST_BYTES: usize = 10_000_000;

/// Maximum character length of an individual node name.
///
/// Applied to every node reference during element parsing. SPICE netlists
/// typically use 1–32 character names; 256 chars is a hard upper bound.
pub const MAX_NODE_NAME_LEN: usize = 256;

/// Maximum number of parsed elements (top-level, before subcircuit expansion).
///
/// This is the pre-expansion ceiling; the post-expansion ceiling
/// (`MAX_ELEMENTS = 10_000` in [`Netlist::expand_subcircuits`]) still applies.
/// A larger pre-expansion cap lets a small netlist with many subcircuit
/// instances expand into a legal post-expansion size.
pub const MAX_TOTAL_ELEMENTS: usize = 50_000;

/// Maximum number of `.model` directives in a netlist.
pub const MAX_MODELS: usize = 1_000;

/// Maximum number of parameters per `.model` directive.
pub const MAX_MODEL_PARAMS: usize = 64;

/// Every melange-only dot command `Parser::parse_directive` accepts that is NOT
/// standard SPICE (lowercase, leading dot). Standard directives that ngspice
/// parses itself (`.model`, `.param`, `.subckt`, `.ends`, `.end`) are
/// deliberately absent.
///
/// This is the shared source of truth for anything that has to hand a melange
/// deck to a real SPICE engine (the validate harness strips these lines before
/// ngspice sees them, since ngspice hard-errors `unimplemented dot command`).
/// INVARIANT: every non-standard-SPICE arm of `parse_directive` must appear
/// here, and every entry here must be an arm of `parse_directive`. The test
/// `test_melange_only_directives_matches_parse_directive` enforces both
/// directions.
pub const MELANGE_ONLY_DIRECTIVES: &[&str] = &[
    ".pot",
    ".switch",
    ".wiper",
    ".gang",
    ".runtime",
    ".mismatch",
    ".tolerance",
    ".seed",
    ".linearize",
    ".tap",
    ".port",
    ".input_impedance",
    ".integrator",
    ".inject",
    ".delay_feedback",
    ".oversampling",
];

/// SPICE netlist parser.
struct Parser {
    /// Pre-processed lines (after continuation joining and comment stripping)
    processed_lines: Vec<String>,
    /// 1-based RAW source line each entry of `processed_lines` started on.
    ///
    /// Continuation joining (`+`) collapses several raw lines into one
    /// processed line, so the processed index is NOT the source line number.
    /// Counting processed lines put every error after the first `+` on the
    /// wrong line.
    source_lines: Vec<usize>,
    /// Current position in processed_lines
    pos: usize,
    /// Raw source line of the statement currently being parsed (0 = none yet).
    line_num: usize,
    /// Raw source line(s) of each element, keyed by lowercased element name.
    /// A `Vec` because duplicate-name diagnostics need the SECOND occurrence —
    /// the offending one. Includes `K` coupling elements.
    element_lines: std::collections::HashMap<String, Vec<usize>>,
    /// Raw source line(s) of each `.model` card, keyed by lowercased name.
    model_lines: std::collections::HashMap<String, Vec<usize>>,
    /// Every dot-directive statement, as `(lowercased whitespace tokens, raw
    /// source line)`, in source order. Post-parse validation resolves a
    /// directive's line by matching its leading token and a named target.
    directive_lines: Vec<(Vec<String>, usize)>,
}

/// Strip an inline comment (`;` or `$` to end of line) from a netlist line,
/// respecting double-quoted regions so labels like `.pot R1 1k 100k "Bass; Mid"`
/// keep their full text.
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

impl Parser {
    fn new(input: &str) -> Self {
        // Pre-process: join continuation lines and strip comments
        let raw_lines: Vec<&str> = input.lines().collect();
        let mut processed = Vec::new();
        let mut source_lines = Vec::new();
        let mut i = 0;

        while i < raw_lines.len() {
            // Raw source line this (possibly continuation-joined) statement
            // starts on, 1-based to match every editor and `sed -n`.
            let start_line = i + 1;
            // Strip inline comments (semicolon and $ delimiter), respecting
            // double-quoted regions (labels may legally contain ';' / '$').
            let line = strip_inline_comment(raw_lines[i]).trim();

            // Accumulate continuation lines: next line starts with '+'
            let mut accumulated = line.to_string();
            while i + 1 < raw_lines.len() {
                let next = strip_inline_comment(raw_lines[i + 1]).trim();
                if let Some(stripped) = next.strip_prefix('+') {
                    // Continuation line: strip '+' and append
                    accumulated.push(' ');
                    accumulated.push_str(stripped.trim());
                    i += 1;
                } else {
                    break;
                }
            }

            processed.push(accumulated);
            source_lines.push(start_line);
            i += 1;
        }

        Self {
            processed_lines: processed,
            source_lines,
            pos: 0,
            line_num: 0,
            element_lines: std::collections::HashMap::new(),
            model_lines: std::collections::HashMap::new(),
            directive_lines: Vec::new(),
        }
    }

    fn parse(mut self, options: ParseOptions) -> Result<Netlist, ParseError> {
        // First line is title
        let title = self.next_line().unwrap_or_default();
        Self::warn_if_title_looks_like_content(&title);
        let mut netlist = Netlist::new(title);
        // Recorded before any directive is read, so both apply sites
        // (`apply_passive_tolerance` below, `mismatch_tol_for` at codegen) see
        // it no matter what the deck contains.
        netlist.unit_variation_disabled = options.disable_unit_variation;
        netlist.self_heating_disabled = options.disable_self_heating;

        while let Some(line) = self.next_line() {
            let line = line.trim().to_string();

            // Skip empty lines and comments
            if line.is_empty() || line.starts_with('*') {
                continue;
            }

            // Top-level `.end` terminates the netlist (ngspice semantics).
            // Anything after it is NOT part of the circuit — warn if the
            // user left non-blank content there.
            let first_token_lower = line
                .split_whitespace()
                .next()
                .unwrap_or("")
                .to_ascii_lowercase();
            if first_token_lower == ".end" {
                let mut trailing = 0usize;
                let mut first_trailing: Option<String> = None;
                while let Some(rest) = self.next_line() {
                    let rest = rest.trim();
                    if rest.is_empty() || rest.starts_with('*') {
                        continue;
                    }
                    trailing += 1;
                    if first_trailing.is_none() {
                        first_trailing = Some(rest.to_string());
                    }
                }
                if trailing > 0 {
                    log::warn!(
                        "{} non-blank line(s) after '.end' were ignored (parsing stops at .end, \
                         ngspice semantics); first ignored line: '{}'",
                        trailing,
                        first_trailing.as_deref().unwrap_or("")
                    );
                }
                break;
            }

            // Raw source line of THIS statement, captured before parsing:
            // `.subckt` consumes its whole body, leaving `line_num` on `.ends`.
            let stmt_line = self.line_num;
            let _value_ctx = ValueContextGuard::set(stmt_line, &line);

            // Parse directive or element
            if line.starts_with('.') {
                self.parse_directive(&line, &mut netlist)?;
                self.record_statement(&line, stmt_line, &netlist);
            } else if line.starts_with('K') || line.starts_with('k') {
                if netlist.couplings.len() >= 16 {
                    return Err(self.error("Maximum of 16 coupling (K) directives supported"));
                }
                let coupling = self.parse_coupling(&line)?;
                netlist.couplings.push(coupling);
                self.record_statement(&line, stmt_line, &netlist);
            } else {
                // Pre-expansion element count cap. Subcircuit instances count
                // as one element here; the post-expansion cap still applies
                // in `expand_subcircuits()`.
                if netlist.elements.len() >= MAX_TOTAL_ELEMENTS {
                    return Err(self.error(format!(
                        "too many elements: {} exceeds MAX_TOTAL_ELEMENTS ({})",
                        netlist.elements.len() + 1,
                        MAX_TOTAL_ELEMENTS
                    )));
                }
                let element = self.parse_element(&line)?;
                validate_element_node_lengths(&element).map_err(|e| self.error(e))?;
                netlist.elements.push(element);
                self.record_statement(&line, stmt_line, &netlist);
            }
        }

        Self::expand_wipers(&mut netlist)?;
        self.validate_netlist(&netlist)?;
        Self::warn_if_no_ground(&netlist);
        // `.tolerance` jitter runs *after* validation so a bad netlist
        // fails with a clear schema error before we silently mutate
        // values the user wrote. No-op when all tolerances are zero, and
        // no-op when `ParseOptions::disable_unit_variation` was set (the
        // check lives inside `apply_passive_tolerance`).
        netlist.apply_passive_tolerance();
        // Hand the recorded element lines to the netlist so post-parse passes
        // (topology checks, and anything else that runs after `parse` returns)
        // can name the line an element was written on. Only the first
        // declaration of each name survives — see `Netlist::element_lines`.
        netlist.element_lines = self
            .element_lines
            .iter()
            .filter_map(|(name, lines)| lines.first().map(|l| (name.clone(), *l)))
            .collect();
        Ok(netlist)
    }

    fn next_line(&mut self) -> Option<String> {
        if self.pos < self.processed_lines.len() {
            let line = self.processed_lines[self.pos].clone();
            self.line_num = self.source_lines[self.pos];
            self.pos += 1;
            Some(line)
        } else {
            None
        }
    }

    fn error(&self, message: impl Into<String>) -> ParseError {
        ParseError {
            line: self.line_num,
            message: message.into(),
        }
    }

    // Post-parse validation runs after the whole deck has been read, so
    // `self.line_num` points at `.end` and is useless there. These lookups
    // resolve the raw source line of the *named* statement a diagnostic is
    // about. All of them return 0 ("not recorded") rather than guess — 0
    // prints as no location at all, never as a wrong one.

    /// First raw source line an element with this name was declared on.
    /// 0 when the name is unknown (e.g. an element created by subcircuit
    /// expansion, which has no authored line of its own).
    fn line_of_element(&self, name: &str) -> usize {
        self.element_lines
            .get(&name.to_ascii_lowercase())
            .and_then(|v| v.first().copied())
            .unwrap_or(0)
    }

    /// Line of the *second* declaration of `name` — the offending one in a
    /// duplicate-name report. Falls back to the first, then to 0.
    fn dup_line_of_element(&self, name: &str) -> usize {
        self.element_lines
            .get(&name.to_ascii_lowercase())
            .and_then(|v| v.get(1).or_else(|| v.first()).copied())
            .unwrap_or(0)
    }

    /// First raw source line a `.model` card with this name was declared on.
    fn line_of_model(&self, name: &str) -> usize {
        self.model_lines
            .get(&name.to_ascii_lowercase())
            .and_then(|v| v.first().copied())
            .unwrap_or(0)
    }

    /// Line of the *second* `.model` card with this name.
    fn dup_line_of_model(&self, name: &str) -> usize {
        self.model_lines
            .get(&name.to_ascii_lowercase())
            .and_then(|v| v.get(1).or_else(|| v.first()).copied())
            .unwrap_or(0)
    }

    /// Raw source line of the first directive whose leading token is one of
    /// `directives` and which names `target` as one of its tokens.
    ///
    /// Exact, case-insensitive token matching — not a substring search — so a
    /// target named `R1` never matches a line that only mentions `R10`.
    /// Quotes and the `.gang` inversion prefix `!` are stripped from each token
    /// first. Returns 0 when nothing matches, which prints as no location
    /// rather than a wrong one.
    fn line_of_directive(&self, directives: &[&str], target: &str) -> usize {
        let target = target.trim_matches('"').to_ascii_lowercase();
        for (tokens, line) in &self.directive_lines {
            let Some(head) = tokens.first() else { continue };
            if !directives.iter().any(|d| d.eq_ignore_ascii_case(head)) {
                continue;
            }
            if tokens[1..]
                .iter()
                .any(|t| t.trim_matches('"').trim_start_matches('!') == target)
            {
                return *line;
            }
        }
        0
    }

    /// Record the raw source line of a parsed statement for post-parse
    /// validation. `line` is captured BEFORE the statement is parsed, because
    /// a `.subckt` body advances `line_num` past its own header.
    fn record_statement(&mut self, raw: &str, line: usize, netlist: &Netlist) {
        let tokens: Vec<String> = raw
            .split_whitespace()
            .map(|t| t.to_ascii_lowercase())
            .collect();
        let Some(head) = tokens.first() else { return };
        if head.starts_with('.') {
            // `.model` also gets a name-keyed entry: model diagnostics are
            // raised per card, long after the directive list is consulted.
            if head == ".model" {
                if let Some(m) = netlist.models.last() {
                    self.model_lines
                        .entry(m.name.to_ascii_lowercase())
                        .or_default()
                        .push(line);
                }
            }
            self.directive_lines.push((tokens, line));
        } else {
            // Elements and `K` couplings are both named by their first token.
            self.element_lines
                .entry(head.clone())
                .or_default()
                .push(line);
        }
    }
}

#[cfg(test)]
mod tests;
