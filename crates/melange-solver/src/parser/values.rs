//! SPICE value parsing and value-error explanations.

use super::*;

/// Collapse whitespace around `=` so `KEY = VAL`, `KEY =VAL`, and `KEY= VAL`
/// all tokenize as a single `KEY=VAL`. Used by `.model` parameter parsing —
/// previously a spaced `=` silently dropped the parameter.
pub(super) fn collapse_ws_around_eq(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '=' {
            while out.ends_with(|c: char| c.is_ascii_whitespace()) {
                out.pop();
            }
            out.push('=');
            while chars.peek().is_some_and(|c| c.is_ascii_whitespace()) {
                chars.next();
            }
        } else {
            out.push(c);
        }
    }
    out
}

/// Try to parse infix notation where a scale character replaces the decimal point.
///
/// Examples: "6n8" → 6.8e-9, "3n3" → 3.3e-9, "4k7" → 4.7e3, "2M2" → 2.2e6
///
/// Pattern: `<digits><scale_char><digits>` where scale_char is one of T,G,K,M,U,N,P.
///
/// NOTE: the scale char is upper-cased before lookup, so infix `m` means MEGA
/// (1e6), NOT milli — `1m5` parses to 1.5e6, not 1.5e-3. There is no infix milli;
/// use an explicit exponent (e.g. `1.5e-3`) or the suffix form for milli values.
fn try_parse_infix(s: &str) -> Option<f64> {
    // Need at least 3 chars: digit, scale, digit
    if s.len() < 3 {
        return None;
    }

    // Find the scale character (must not be first or last, and must be alphabetic)
    let bytes = s.as_bytes();
    let mut scale_pos = None;
    for (i, &b) in bytes.iter().enumerate() {
        if i == 0 {
            continue;
        }
        let c = (b as char).to_ascii_uppercase();
        if matches!(c, 'T' | 'G' | 'K' | 'M' | 'U' | 'N' | 'P') {
            // Check: digits before, digits after
            let before = &s[..i];
            let after = &s[i + 1..];
            if !before.is_empty()
                && !after.is_empty()
                && before.chars().all(|c| c.is_ascii_digit())
                && after.chars().all(|c| c.is_ascii_digit())
            {
                scale_pos = Some((i, c));
                break;
            }
        }
    }

    let (pos, scale_char) = scale_pos?;
    let before = &s[..pos];
    let after = &s[pos + 1..];
    let decimal_str = format!("{}.{}", before, after);
    let base: f64 = decimal_str.parse().ok()?;
    let scale = match scale_char {
        'T' => 1e12,
        'G' => 1e9,
        'K' => 1e3,
        // BS-1852 infix notation: 'M' means MEGA (4M7 = 4.7 MΩ). This differs
        // from SPICE *suffix* position, where 'M'/'m' is milli (10m = 10e-3).
        // Nobody writes 4.7 mΩ as "4M7"; parsing it as milli silently built a
        // wrong circuit.
        'M' => {
            log::warn!(
                "value '{}'{}: infix 'M' interpreted as MEGA per BS-1852 ({} = {:.3e}); \
                 suffix-position 'm' remains milli (e.g. '10m' = 10e-3)",
                s,
                value_context_suffix(),
                s,
                base * 1e6
            );
            1e6
        }
        'U' => 1e-6,
        'N' => 1e-9,
        'P' => 1e-12,
        _ => return None,
    };
    let result = base * scale;
    if result.is_finite() {
        Some(result)
    } else {
        None
    }
}

thread_local! {
    /// Where the statement being parsed came from, for value warnings.
    ///
    /// The value parsers are free functions with no line in hand, and two
    /// `1M`s in one deck produced two identical warnings a user could not
    /// place. The parse loop sets this per statement; it is empty outside a
    /// parse, so a bare `parse_value` call from elsewhere warns as before.
    static VALUE_CONTEXT: std::cell::RefCell<Option<String>> =
        const { std::cell::RefCell::new(None) };
}

/// Sets [`VALUE_CONTEXT`] for the life of the guard.
pub(super) struct ValueContextGuard;

impl ValueContextGuard {
    pub(super) fn set(line: usize, statement: &str) -> Self {
        let head = statement.split_whitespace().next().unwrap_or("");
        VALUE_CONTEXT.with(|c| *c.borrow_mut() = Some(format!("line {line}, {head}")));
        ValueContextGuard
    }
}

impl Drop for ValueContextGuard {
    fn drop(&mut self) {
        VALUE_CONTEXT.with(|c| *c.borrow_mut() = None);
    }
}

/// `" (line 7, R4)"` while a statement is being parsed, else empty.
fn value_context_suffix() -> String {
    VALUE_CONTEXT.with(|c| {
        c.borrow()
            .as_ref()
            .map(|ctx| format!(" ({ctx})"))
            .unwrap_or_default()
    })
}

/// Parse a SPICE value with optional scale suffix.
///
/// Examples:
/// - "1k" -> 1000.0
/// - "4.7u" -> 4.7e-6
/// - "10pF" -> 10e-12
/// - "1fF" -> 1e-15 (femto + Farad unit)
/// - "1F" -> 1.0 (Farad, not femto — element-value positions only; warns)
/// - "6n8" -> 6.8e-9 (infix notation)
///
/// Parses a component value string with engineering notation (e.g. "10k", "4n7", "1Meg").
pub fn parse_value(s: &str) -> Result<f64, ParseFloatError> {
    parse_value_ctx(s, false)
}

/// Explain *why* a value token was rejected, as a sentence appended to the
/// caller's `Invalid <what> value '<raw>'` prefix (leading space included).
///
/// A bare "invalid value" was the one place melange's diagnostics went silent:
/// every other parse error names the component, the directive or the accepted
/// set, so a value error that said nothing taught readers that melange's error
/// messages were unreliable rather than that this one input was.
///
/// ## Why `10R` / `1R5` are rejected rather than accepted
///
/// They are the same BS-1852 convention as `4k7` and `2M2`, which melange DOES
/// accept, so the asymmetry needs a reason. The reason is ngspice. Measured
/// (`print i(v)` across a 1 V source, system ngspice):
///
/// | token | ngspice | melange |
/// |-------|---------|---------|
/// | `10R` | 10 Ω    | *rejected* |
/// | `1R5` | 1 Ω     | *rejected* |
/// | `4k7` | 4000 Ω  | 4700 Ω |
/// | `2M2` | 0.002 Ω | 2.2 MΩ |
///
/// ngspice reads the mantissa, applies a scale letter if the next character is
/// one, and **discards the rest of the token**. `R` is not a scale letter, so
/// `1R5` is 1 Ω and `10R` is 10 Ω — silently, with no diagnostic.
/// `melange validate` hands the author's own `.cir` to ngspice, so accepting
/// `1R5` as 1.5 Ω would have the two engines simulate different circuits and
/// blame the difference on the solver. Refusing costs the author one edit;
/// accepting cannot be made safe.
///
/// (The same table shows `4k7` and `2M2` already diverging that way. That is a
/// real cross-engine hazard, but not this function's to fix: `2M2` warns from
/// [`try_parse_infix`], and changing what those tokens mean would silently
/// change existing decks' component values.)
/// The documented argument order for a directive, keyed by the label its
/// parser passes to [`explain_rejected_value`]. Only directives whose fields
/// are parsed as values need an entry; anything else simply gets no shape hint.
/// Declared model names within edit distance 2 of `want`, closest first, at
/// most three. Case-insensitive, because `.model` references are.
pub(super) fn nearest_model_names(want: &str, models: &[Model]) -> Vec<String> {
    let w = want.to_ascii_lowercase();
    let mut scored: Vec<(usize, &str)> = models
        .iter()
        .filter_map(|m| {
            let d = ascii_edit_distance(&w, &m.name.to_ascii_lowercase());
            (d <= 2).then_some((d, m.name.as_str()))
        })
        .collect();
    scored.sort_by_key(|(d, n)| (*d, *n));
    scored
        .into_iter()
        .take(3)
        .map(|(_, n)| n.to_string())
        .collect()
}

fn ascii_edit_distance(a: &str, b: &str) -> usize {
    let (a, b): (Vec<char>, Vec<char>) = (a.chars().collect(), b.chars().collect());
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut cur = vec![0usize; b.len() + 1];
    for i in 1..=a.len() {
        cur[0] = i;
        for j in 1..=b.len() {
            let cost = usize::from(a[i - 1] != b[j - 1]);
            cur[j] = (prev[j] + 1).min(cur[j - 1] + 1).min(prev[j - 1] + cost);
        }
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[b.len()]
}

fn directive_shape(context: &str) -> Option<&'static str> {
    Some(match context {
        c if c.starts_with(".pot") => ".pot Rname min_value max_value [default] [\"Label\"]",
        c if c.starts_with(".wiper") => ".wiper R_cw R_ccw total_resistance",
        c if c.starts_with(".runtime") => ".runtime Rname min max as field_name",
        c if c.starts_with(".input_impedance") => ".input_impedance <value>",
        c if c.starts_with(".tolerance") => ".tolerance <percent>",
        c if c.starts_with(".mismatch") => ".mismatch <percent>",
        _ => return None,
    })
}

/// `context` is the caller's label for the field being parsed (`"R"`,
/// `".pot min"`, …). It is used to tell "you typed a number wrong" apart from
/// "you put something that is not a number here at all" — two different
/// mistakes that want two different answers.
pub(super) fn explain_rejected_value(raw: &str, context: &str) -> String {
    let t = raw.trim();
    // A token that does not even START like a number is not a mistyped value;
    // it is a name, or an argument in the wrong position. Answering it with the
    // scale-suffix reference is a confident wrong diagnosis — it explains how
    // to write 4k7 to someone whose actual problem is that this field does not
    // take a label. (`.pot RV1 Volume 0 50k` produced exactly that.)
    let starts_numeric = t
        .chars()
        .next()
        .is_some_and(|c| c.is_ascii_digit() || c == '.' || c == '+' || c == '-');
    // Only where the field belongs to a directive with a known argument order.
    // In a bare component value (`R1 in out banana`) there is no position to
    // get wrong, so "you don't know what a value looks like" is the real
    // problem and the accepted-forms list below is the right answer.
    if !t.is_empty() && !starts_numeric {
        if let Some(shape) = directive_shape(context) {
            return format!(
                " That is not a value — it does not start with a digit, sign or \
                 decimal point, so it reads as a name or an argument in the wrong \
                 position rather than a mistyped number. The form is: {shape}."
            );
        }
    }

    // Every claim here is measured against the parser, not inferred from it:
    // `f` is deliberately absent from the scale list because a trailing `f` in
    // an ELEMENT value is the Farad unit (`10f` = 10, with its own warning from
    // `parse_value_ctx`) and only means femto inside a `.model` card. `ohm` is
    // called out because it is the obvious thing to type and the unit-letter
    // strip set is F/H/V/A/S/Z — `10kohm` is a hard error, not 10 kΩ.
    const ACCEPTED: &str = " melange accepts: a plain number (1500, 1.5e3); a SPICE scale \
         suffix — T G k meg m(=milli) u/µ n p — optionally followed by a unit letter \
         (10pF, 4.7uF, 100nH, 10kHz, 9V); and the BS-1852 infix form, where the scale \
         letter replaces the decimal point (4k7 = 4.7k, 6n8 = 6.8n). Note that \
         'ohm'/'ohms' is NOT a recognized unit — write 10k, not 10kohm. In a component \
         value a trailing 'f' is the Farad unit and NOT femto, so write 10e-15 or 10fF; \
         inside a `.model` card, where values are dimensionless, 'f' does mean femto.";

    let t = raw.trim();
    if t.is_empty() {
        return " The value field is empty.".to_string();
    }
    let upper = t.to_ascii_uppercase();
    // BS-1852 ohms marker: `10R`, `1R5`, `4R7`. Distinctive enough to name.
    let is_bs1852_ohms = upper.contains('R')
        && upper
            .chars()
            .all(|c| c.is_ascii_digit() || c == 'R' || c == '.')
        && upper.starts_with(|c: char| c.is_ascii_digit());
    if is_bs1852_ohms {
        // What the author almost certainly meant: 'R' as the decimal point.
        let intended = upper.replace('R', ".");
        let intended = intended.trim_end_matches('.');
        return format!(
            " 'R' is the BS-1852 ohms marker and melange does not accept it — write \
             '{}' instead. ngspice silently reads '{}' as {} (it applies a scale \
             letter if one follows the number, then discards the rest of the token), \
             so honouring 'R' here would make melange and the ngspice run behind \
             `melange validate` simulate different circuits. The infix scale forms \
             melange does accept — 4k7 = 4.7k, 6n8 = 6.8n — are unaffected.",
            intended,
            t,
            t.split(['R', 'r']).next().unwrap_or(t),
        );
    }
    if !t.is_ascii() {
        return format!(
            " It contains a non-ASCII character; only the micro sign (µ/μ) is accepted, \
             as an alias for 'u'.{}",
            ACCEPTED
        );
    }
    format!(" Not a number melange recognizes.{}", ACCEPTED)
}

/// Parse a `.model` parameter value.
///
/// Model parameters are a dimensionless context, so a single trailing
/// `f`/`F` after a digit is the femto scale (`.model DX D(IS=6.734f)` →
/// 6.734e-15), matching ngspice. In element-value positions (`parse_value`)
/// a bare trailing `F` keeps the Farad reading.
pub fn parse_value_model_param(s: &str) -> Result<f64, ParseFloatError> {
    parse_value_ctx(s, true)
}

fn parse_value_ctx(s: &str, model_param_ctx: bool) -> Result<f64, ParseFloatError> {
    let s = s.trim();
    if s.is_empty() {
        return Err(ParseFloatError);
    }

    // Normalize non-ASCII characters. SPICE component values are ASCII by convention,
    // with the sole exception of the micro sign (µ, U+00B5) and Greek small mu (μ, U+03BC)
    // which we accept as an alias for 'u'. Rejecting other non-ASCII up front means the
    // byte-level slicing below is safe (prevents panics on inputs like "1ſ" where
    // to_uppercase() changes byte length and byte indices land mid-codepoint).
    let normalized_owned: String;
    let s: &str = if s.is_ascii() {
        s
    } else {
        let mut out = String::with_capacity(s.len());
        for c in s.chars() {
            if c.is_ascii() {
                out.push(c);
            } else if c == '\u{00B5}' || c == '\u{03BC}' {
                out.push('u');
            } else {
                return Err(ParseFloatError);
            }
        }
        normalized_owned = out;
        normalized_owned.as_str()
    };

    // Try infix notation first (e.g. "6n8" → 6.8e-9)
    if let Some(val) = try_parse_infix(s) {
        return Ok(val);
    }

    let s_upper = s.to_uppercase();

    // Check for MEG first (must be before stripping units)
    if s_upper.ends_with("MEG") {
        let num_part = &s[..s.len() - 3];
        if num_part.is_empty() {
            return Err(ParseFloatError);
        }
        let value: f64 = num_part.parse().map_err(|_| ParseFloatError)?;
        // Rust's f64 parser accepts "nan"/"inf", so "nanmeg" must be caught
        // here like every other path.
        let result = value * 1e6;
        if !result.is_finite() {
            return Err(ParseFloatError);
        }
        return Ok(result);
    }

    // Femto handling. The general unit-stripping below removes trailing
    // F/H/V/A/S/Z letters, which would silently eat a femto suffix
    // ('6.734f' → 6.734). Detect femto here: take the trailing run of unit
    // letters; if it starts with 'f'/'F' immediately after a digit, it is
    // the femto scale when either
    //   (a) we're in a .model parameter (dimensionless) context, or
    //   (b) further unit letters follow it ('1fF' = 1 femtofarad).
    // A bare digit+'F' in an element-value position stays 1.0× (Farad) with
    // a loud warning — write '1e-15' or '1fF' for femto there.
    {
        let bytes = s.as_bytes();
        let mut run_start = s.len();
        while run_start > 0
            && matches!(
                bytes[run_start - 1].to_ascii_uppercase(),
                b'F' | b'H' | b'V' | b'A' | b'S' | b'Z'
            )
        {
            run_start -= 1;
        }
        if run_start > 0
            && run_start < s.len()
            && bytes[run_start].eq_ignore_ascii_case(&b'F')
            && bytes[run_start - 1].is_ascii_digit()
        {
            let run_len = s.len() - run_start;
            if model_param_ctx || run_len > 1 {
                let num: f64 = s[..run_start].parse().map_err(|_| ParseFloatError)?;
                let result = num * 1e-15;
                if !result.is_finite() {
                    return Err(ParseFloatError);
                }
                return Ok(result);
            }
            log::warn!(
                "value '{}'{} parsed as {} (trailing 'F' treated as a Farad unit, NOT femto); \
                 write '{}e-15' or '{}fF' if femto was intended",
                s,
                value_context_suffix(),
                &s[..run_start],
                &s[..run_start],
                &s[..run_start]
            );
            // fall through: the normal path strips the 'F' and applies scale 1.0
        }
    }

    // Strip unit suffixes FIRST (F, H, V, A, OHM, HZ)
    // Note: We remove from s_upper to get the uppercase version, then apply to original s
    let stripped_upper = s_upper.trim_end_matches(|c: char| {
        matches!(c.to_ascii_uppercase(), 'F' | 'H' | 'V' | 'A' | 'S' | 'Z')
    });

    // Calculate how much we stripped
    let chars_stripped = s_upper.len() - stripped_upper.len();
    let mut num_part = &s[..s.len().saturating_sub(chars_stripped)];
    let mut scale = 1.0;

    // Now parse scale suffix from what's left
    if let Some(c) = stripped_upper.chars().last() {
        let (new_num, new_scale) = match c {
            'T' if num_part.len() > 1 => (&num_part[..num_part.len() - 1], 1e12),
            'G' if num_part.len() > 1 => (&num_part[..num_part.len() - 1], 1e9),
            'K' if num_part.len() > 1 => (&num_part[..num_part.len() - 1], 1e3),
            // M = milli. SPICE reads a bare 'M' suffix as milli and melange must
            // keep doing so: ngspice reads it that way too, and `melange validate`
            // hands the author's own deck straight to ngspice, so reinterpreting
            // it here would make the two engines simulate different circuits —
            // the exact hazard the infix-value guard in melange-validate exists
            // to catch.
            //
            // But melange already warns when an infix 'M' resolves to MEGA
            // ('1M0'), which left the *silent* reading as the catastrophic one:
            // '1M' where '1meg' was meant is 10^9 out, and a first-time user lost
            // 36 dB of signal to it with no diagnostic at all. So warn here too,
            // and melange now warns on both readings of the ambiguous letter.
            //
            // Gated on an UPPERCASE 'M' because that is how a mega gets typed —
            // milli is authored lowercase (39 values across the circuits corpus
            // use a lowercase 'm' suffix, every one of them legitimate; zero use
            // uppercase). The case carries the intent even though the parse
            // cannot. It is a warning, never a reinterpretation.
            'M' if num_part.len() > 1 => {
                if num_part.as_bytes()[num_part.len() - 1] == b'M' {
                    let mantissa = &num_part[..num_part.len() - 1];
                    if let Ok(m) = mantissa.parse::<f64>() {
                        log::warn!(
                            "value '{}'{}: a suffix 'M' is MILLI in SPICE, so this is {:.3e}, \
                             not mega — a factor of 10^9 apart. Write '{}meg' (or '{}M0') \
                             for mega; write '{}m' for milli and this warning goes away.",
                            s,
                            value_context_suffix(),
                            m * 1e-3,
                            mantissa,
                            mantissa,
                            mantissa
                        );
                    }
                }
                (&num_part[..num_part.len() - 1], 1e-3)
            }
            'U' | 'µ' if num_part.len() > 1 => (&num_part[..num_part.len() - 1], 1e-6),
            'N' if num_part.len() > 1 => (&num_part[..num_part.len() - 1], 1e-9),
            'P' if num_part.len() > 1 => (&num_part[..num_part.len() - 1], 1e-12),
            // 'F' never survives to this point: trailing F is either femto
            // (handled above) or stripped as the Farad unit letter.
            _ => (num_part, 1.0),
        };
        num_part = new_num;
        scale = new_scale;
    }

    if num_part.is_empty() {
        return Err(ParseFloatError);
    }

    let value: f64 = num_part.parse().map_err(|_| ParseFloatError)?;
    let result = value * scale;
    if !result.is_finite() {
        return Err(ParseFloatError);
    }
    Ok(result)
}

/// Error type for float parsing failures.
#[derive(Debug, Clone, Copy)]
pub struct ParseFloatError;

impl std::fmt::Display for ParseFloatError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "invalid float value")
    }
}

impl std::error::Error for ParseFloatError {}
