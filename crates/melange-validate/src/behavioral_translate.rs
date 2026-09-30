//! Behavioral-source functions ngspice's `B` source lacks.
//!
//! melange's expression language (`expr.rs`) has two functions ngspice's `B`
//! source does not: `idt(x)`, the time integral from 0, and `atan2(y, x)`.
//! Every other function it accepts (`sqrt abs exp ln log sin cos tanh min max
//! pow pwr ddt`, `time`, `pi`) is native. A deck using either failed in
//! ngspice ("no such function"). They are rewritten:
//!
//! - `idt(a)` becomes the voltage of an integrator node: a unit capacitor
//!   charged by a `B` current `a`, started at 0 (`.ic`), with a 1e15 Ω leak so
//!   its DC operating point is defined (time constant 1e15 s).
//! - `atan2(y, x)` becomes the quadrant-correct form on `atan` with ngspice's
//!   ternary operator.

/// The call's argument list starting just after `(` at `open`: the index of
/// the matching `)` and the top-level comma-separated arguments.
fn split_call(s: &str, open: usize) -> Option<(usize, Vec<String>)> {
    let bytes = s.as_bytes();
    let mut depth = 0usize;
    let mut args = Vec::new();
    let mut start = open;
    for (i, &b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'(' => depth += 1,
            b')' if depth == 0 => {
                args.push(s[start..i].trim().to_string());
                return Some((i, args));
            }
            b')' => depth -= 1,
            b',' if depth == 0 => {
                args.push(s[start..i].trim().to_string());
                start = i + 1;
            }
            _ => {}
        }
    }
    None
}

/// Rewrite `idt(`/`atan2(` in `expr`, innermost first. Integrator elements
/// are appended to `extra`, numbered from `next`.
fn rewrite(expr: &str, extra: &mut Vec<String>, next: &mut usize) -> String {
    let lower = expr.to_ascii_lowercase();
    let mut out = String::with_capacity(expr.len());
    let mut i = 0;
    while i < expr.len() {
        let rest = &lower[i..];
        let at_word_start = i == 0
            || !expr.as_bytes()[i - 1].is_ascii_alphanumeric() && expr.as_bytes()[i - 1] != b'_';
        let call = if at_word_start && rest.starts_with("idt(") {
            Some(("idt", 4))
        } else if at_word_start && rest.starts_with("atan2(") {
            Some(("atan2", 6))
        } else {
            None
        };
        if let Some((name, len)) = call {
            if let Some((close, args)) = split_call(expr, i + len) {
                let args: Vec<String> = args.iter().map(|a| rewrite(a, extra, next)).collect();
                match (name, args.as_slice()) {
                    ("idt", [a]) => {
                        let k = *next;
                        *next += 1;
                        extra.push(format!("Bidt_{k} 0 nidt_{k} I={{{a}}}"));
                        extra.push(format!("Cidt_{k} nidt_{k} 0 1"));
                        extra.push(format!("Ridt_{k} nidt_{k} 0 1e15"));
                        extra.push(format!(".ic v(nidt_{k})=0"));
                        out.push_str(&format!("v(nidt_{k})"));
                    }
                    ("atan2", [y, x]) => {
                        let (y, x) = (format!("({y})"), format!("({x})"));
                        out.push_str(&format!(
                            "({x}>0 ? atan({y}/{x}) : ({x}<0 ? ({y}>=0 ? atan({y}/{x})+pi : \
                             atan({y}/{x})-pi) : ({y}>0 ? pi/2 : ({y}<0 ? -pi/2 : 0))))"
                        ));
                    }
                    // A malformed call is left for ngspice to reject loudly.
                    _ => out.push_str(&expr[i..=close]),
                }
                i = close + 1;
                continue;
            }
        }
        let c = expr[i..].chars().next().unwrap();
        out.push(c);
        i += c.len_utf8();
    }
    out
}

/// Rewrite every `B` line of `content` that calls `idt` or `atan2`.
pub(crate) fn translate_behavioral_for_ngspice(content: &str) -> String {
    let uses = |l: &str| {
        let t = l.trim_start().to_ascii_lowercase();
        t.starts_with('b') && (t.contains("idt(") || t.contains("atan2("))
    };
    if !content.lines().skip(1).any(uses) {
        return content.to_string();
    }
    let mut extra = Vec::new();
    let mut next = 0usize;
    let mut out = String::with_capacity(content.len() + 256);
    for (i, line) in content.lines().enumerate() {
        if i > 0 && uses(line) {
            out.push_str(&rewrite(line, &mut extra, &mut next));
        } else {
            out.push_str(line);
        }
        out.push('\n');
    }
    if extra.is_empty() {
        return out;
    }
    log::warn!(
        "validate: the ngspice reference rewrites melange's idt()/atan2() (no ngspice \
         B-source equivalent) as an integrator node / a quadrant-correct atan."
    );
    // Before `.end` when there is one, else at the end.
    let block: String = extra.iter().map(|l| format!("{l}\n")).collect();
    match out.to_ascii_lowercase().rfind("\n.end") {
        Some(pos) => {
            let mut s = out;
            s.insert_str(pos + 1, &block);
            s
        }
        None => out + &block,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idt_becomes_an_integrator_node() {
        let deck = "t\nB_theta theta 0 V={ idt(2*pi*f_offset) }\nR1 theta 0 1k\n.end\n";
        let out = translate_behavioral_for_ngspice(deck);
        assert!(out.contains("B_theta theta 0 V={ v(nidt_0) }"), "{out}");
        assert!(out.contains("Bidt_0 0 nidt_0 I={2*pi*f_offset}\n"), "{out}");
        assert!(out.contains("Cidt_0 nidt_0 0 1\n"), "{out}");
        assert!(out.contains(".ic v(nidt_0)=0\n.end"), "{out}");
    }

    #[test]
    fn atan2_is_quadrant_correct() {
        let deck = "t\nB1 p 0 V={ atan2(V(q), V(i)) }\n.end\n";
        let out = translate_behavioral_for_ngspice(deck);
        assert!(!out.contains("atan2("), "{out}");
        assert!(out.contains("((V(i))>0 ? atan((V(q))/(V(i)))"), "{out}");
    }

    #[test]
    fn nested_calls_rewrite_inside_out() {
        let deck = "t\nB1 p 0 V={ idt(atan2(V(q), idt(V(i)))) }\n.end\n";
        let out = translate_behavioral_for_ngspice(deck);
        assert!(!out.contains("idt(") && !out.contains("atan2("), "{out}");
        assert!(out.contains("Bidt_0 0 nidt_0 I={V(i)}"), "{out}");
        assert!(out.contains("B1 p 0 V={ v(nidt_1) }"), "{out}");
    }

    #[test]
    fn a_deck_without_them_is_unchanged() {
        let deck = "t\nB1 p 0 V={ sqrt(V(a)) }\nR1 p 0 1k\n.end\n";
        assert_eq!(translate_behavioral_for_ngspice(deck), deck);
    }
}
