//! The `const` items of generated Rust code, as text: what a runtime-
//! oversampling build compares across factors and switches per factor.

/// One `[pub] const NAME: TYPE = VALUE;` item of generated code, with the byte
/// ranges of its type and value in the code it was read from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ConstItem {
    pub name: String,
    pub ty: String,
    pub value: String,
    pub ty_range: std::ops::Range<usize>,
    pub value_range: std::ops::Range<usize>,
}

/// Every top-level-style `const` item of `code` (an item starting its line,
/// after indentation and an optional `pub`), in order. The value ends at the
/// first `;` outside brackets, parentheses and braces, so a type or value that
/// carries `;` inside brackets (`[f64; N]`) is read whole.
pub(crate) fn const_items(code: &str) -> Vec<ConstItem> {
    let mut out = Vec::new();
    for (start, _) in code.match_indices("const ") {
        let line_start = code[..start].rfind('\n').map_or(0, |i| i + 1);
        let prefix = code[line_start..start].trim();
        if !(prefix.is_empty() || prefix == "pub") {
            continue;
        }
        let after = start + "const ".len();
        let Some(colon) = code[after..].find(':').map(|i| after + i) else {
            continue;
        };
        let name = code[after..colon].trim();
        if name.is_empty()
            || !name
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        {
            continue;
        }
        let Some(eq) = code[colon..].find('=').map(|i| colon + i) else {
            continue;
        };
        let mut depth = 0i32;
        let mut end = None;
        for (i, c) in code[eq + 1..].char_indices() {
            match c {
                '[' | '(' | '{' => depth += 1,
                ']' | ')' | '}' => depth -= 1,
                ';' if depth == 0 => {
                    end = Some(eq + 1 + i);
                    break;
                }
                _ => {}
            }
        }
        let Some(end) = end else { continue };
        let trimmed = |r: std::ops::Range<usize>| {
            let s = &code[r.clone()];
            let lead = s.len() - s.trim_start().len();
            let trail = s.len() - s.trim_end().len();
            r.start + lead..r.end - trail
        };
        let ty_range = trimmed(colon + 1..eq);
        let value_range = trimmed(eq + 1..end);
        out.push(ConstItem {
            name: name.to_string(),
            ty: code[ty_range.clone()].to_string(),
            value: code[value_range.clone()].to_string(),
            ty_range,
            value_range,
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_array_types_with_semicolons_and_skips_non_items() {
        let code = "pub const S: [[f64; N]; N] = [[1.0, 2.0], [3.0, 4.0]];\n\
                    const K: f64 = 2.0; // trailing\n\
                    let x = 1; // const NOT: an item\n\
                    fn f() { const fn_local: u8 = 1; }\n";
        let items = const_items(code);
        let names: Vec<&str> = items.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["S", "K"]);
        assert_eq!(items[0].ty, "[[f64; N]; N]");
        assert_eq!(items[0].value, "[[1.0, 2.0], [3.0, 4.0]]");
        assert_eq!(&code[items[1].value_range.clone()], "2.0");
    }
}
