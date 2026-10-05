//! Runtime-selectable oversampling (`.oversampling N allow=...`): the pieces
//! of generated code that differ from a fixed-factor build.
//!
//! A runtime build is the default factor's fixed build with three changes:
//! every read of the factor is a read of `state.oversampling` (the running
//! factor) instead of the `OVERSAMPLING_FACTOR` constant; every generated
//! constant whose value differs between the factors' fixed builds (the baked
//! matrices, the DC-blocker coefficient, the BE latch's reference) is read
//! through an accessor that returns the running factor's value; and the
//! oversampling cascade is emitted for every factor of the set and dispatched
//! on the running factor. The per-factor constants are the fixed builds' own
//! literals (copied at build time, `build::per_factor_consts`), so at every
//! factor the runtime code computes what that factor's fixed build computes.
//!
//! Every function here returns the fixed build's text when the IR has no
//! runtime set, so a fixed build is unchanged.

use super::helpers::oversampling_info;
use crate::codegen::ir::{CircuitIR, RuntimeOversampling};
use crate::codegen::CodegenError;

/// The runtime set, if the build has one.
pub(super) fn runtime(ir: &CircuitIR) -> Option<&RuntimeOversampling> {
    ir.solver_config.runtime_oversampling.as_ref()
}

/// The running factor as a `usize` expression: the `OVERSAMPLING_FACTOR`
/// constant in a fixed build, `{recv}.oversampling` in a runtime build.
pub(super) fn factor_expr(ir: &CircuitIR, recv: &str) -> String {
    if runtime(ir).is_some() {
        format!("{recv}.oversampling")
    } else {
        "OVERSAMPLING_FACTOR".to_string()
    }
}

/// The running factor as an `f64` expression where a fixed build writes the
/// factor as a literal (`{factor}.0`). In a runtime build it is
/// `({recv}.oversampling as f64)`: the same value exactly (1, 2 and 4 are exact
/// in f64), so every product with it rounds as the literal's does.
pub(super) fn factor_f64_literal(ir: &CircuitIR, recv: &str) -> String {
    if runtime(ir).is_some() {
        format!("({recv}.oversampling as f64)")
    } else {
        format!("{}.0", ir.solver_config.oversampling_factor)
    }
}

/// A generated constant read at the running factor: `name` itself, or, when
/// its value differs between the set's factors, its accessor called with the
/// `os` expression (`self.oversampling`, `state.oversampling`, `os`).
pub(super) fn baked(ir: &CircuitIR, name: &str, os: &str) -> String {
    match runtime(ir) {
        Some(rt) if rt.per_factor_consts.iter().any(|c| c.name == name) => {
            format!("{}_for({os})", name.to_ascii_lowercase())
        }
        _ => name.to_string(),
    }
}

/// Finish a runtime build's code: every internal-rate product that reads the
/// factor constant at a running rate reads the running factor instead. The
/// emitters write it in three receiver-qualified forms (the per-sample solve,
/// `state`; the `CircuitState` methods, `self`, and `set_sample_rate`'s
/// `sample_rate` argument), and only those are rewritten. Construction's
/// `SAMPLE_RATE * OVERSAMPLING_FACTOR` is the default factor's and stays.
pub(super) fn finalize(ir: &CircuitIR, code: String) -> Result<String, CodegenError> {
    let Some(rt) = runtime(ir) else {
        return Ok(code);
    };
    let code = pin_default_consts(rt, constructor_at(ir, code)?);
    let code = code
        .replace(
            "state.current_sample_rate * OVERSAMPLING_FACTOR as f64",
            "state.current_sample_rate * state.oversampling as f64",
        )
        .replace(
            "self.current_sample_rate * OVERSAMPLING_FACTOR as f64",
            "self.current_sample_rate * self.oversampling as f64",
        );
    // The bare `sample_rate` argument of `set_sample_rate` (a method: `self`).
    let mut out = String::with_capacity(code.len());
    let pat = "sample_rate * OVERSAMPLING_FACTOR as f64";
    let mut rest = code.as_str();
    while let Some(i) = rest.find(pat) {
        let before = rest[..i].chars().next_back();
        out.push_str(&rest[..i]);
        if before.is_some_and(|c| c.is_ascii_alphanumeric() || c == '_' || c == '.') {
            out.push_str(pat);
        } else {
            out.push_str("sample_rate * self.oversampling as f64");
        }
        rest = &rest[i + pat.len()..];
    }
    out.push_str(rest);
    audit_factor_reads(&out)?;
    Ok(out)
}

/// The `CircuitState` field holding the running factor ("" when fixed).
pub(super) fn state_field_decl(ir: &CircuitIR) -> &'static str {
    if runtime(ir).is_some() {
        "    /// The running oversampling factor, one of `OVERSAMPLING_SET` (change it\n\
         \x20   /// with `set_oversampling`).\n\
         \x20   oversampling: usize,\n"
    } else {
        ""
    }
}

/// The field's initializer: a fresh state runs at the default factor.
pub(super) fn state_field_init(ir: &CircuitIR) -> &'static str {
    if runtime(ir).is_some() {
        "            oversampling: OVERSAMPLING_FACTOR,\n"
    } else {
        ""
    }
}

/// In a runtime build, make the restore assignments of `code[from..]` (the
/// generated `reset()` / `set_sample_rate()` bodies, where `X = NAME;` reloads a
/// baked constant) load the running factor's value: `X = name_for(self.oversampling);`.
/// Construction (`NAME,` field initializers) keeps the default factor's value.
pub(super) fn switch_restores(ir: &CircuitIR, code: &mut String, from: usize) {
    let Some(rt) = runtime(ir) else {
        return;
    };
    let mut tail = code.split_off(from);
    for c in &rt.per_factor_consts {
        tail = tail.replace(
            &format!("= {};", c.name),
            &format!("= {}_for(self.oversampling);", c.name.to_ascii_lowercase()),
        );
    }
    code.push_str(&tail);
}

/// The runtime set's constants: the set, its largest factor, each switched
/// constant per factor (`NAME_OS{n}`, the fixed build's literal) and its
/// accessor `name_for(os)`. Empty for a fixed build.
pub(super) fn emit_consts(ir: &CircuitIR) -> String {
    let Some(rt) = runtime(ir) else {
        return String::new();
    };
    let factors = &rt.factors;
    let max = factors.iter().copied().max().unwrap_or(1);
    let mut code = format!(
        "/// The oversampling factors `set_oversampling` accepts (runtime-selectable\n\
         /// oversampling). A fresh state runs at `OVERSAMPLING_FACTOR`, the default.\n\
         pub const OVERSAMPLING_SET: [usize; {}] = [{}];\n\n\
         /// The largest factor of `OVERSAMPLING_SET`: the length of the per-inner-sample\n\
         /// arrays of the `.inject`/`.tap` API (entries past `state.oversampling()` are\n\
         /// ignored).\n\
         pub const MAX_OVERSAMPLING: usize = {max};\n\n",
        factors.len(),
        factors
            .iter()
            .map(|f| f.to_string())
            .collect::<Vec<_>>()
            .join(", ")
    );
    for c in &rt.per_factor_consts {
        for ((f, v), ty) in factors.iter().zip(&c.values).zip(&c.types) {
            code.push_str(&format!(
                "/// `{}` at {f}x oversampling.\nconst {}_OS{f}: {ty} = {v};\n\n",
                c.name, c.name
            ));
        }
        // One type at every factor: return it by value, as the fixed build
        // reads the constant. Arrays whose length differs by factor: a slice.
        let same_type = c.types.windows(2).all(|w| w[0] == w[1]);
        let (ret, amp) = if same_type {
            (c.types[0].clone(), "")
        } else {
            let elem = array_element_type(&c.types[0]).unwrap_or_else(|| {
                panic!(
                    "{}: per-factor types differ and are not arrays: {:?}",
                    c.name, c.types
                )
            });
            (format!("&'static [{elem}]"), "&")
        };
        let mut arms: Vec<String> = factors
            .iter()
            .map(|f| format!("{f} => {amp}{}_OS{f}", c.name))
            .collect();
        // The factor field is private and only `set_oversampling` (which
        // refuses a factor outside the set) writes it.
        arms.push(
            "_ => unreachable!(\"oversampling factor outside OVERSAMPLING_SET\")".to_string(),
        );
        code.push_str(&format!(
            "/// `{}` at the running oversampling factor `os`.\n\
             #[inline(always)]\n\
             #[allow(dead_code)]\n\
             fn {}_for(os: usize) -> {ret} {{\n    match os {{ {} }}\n}}\n\n",
            c.name,
            c.name.to_ascii_lowercase(),
            arms.join(", ")
        ));
    }
    code
}

/// The oversampling filter states of a runtime build, per factor of the set
/// above 1: `(field, element type, zero value)`. 2x is one steep stage
/// (`os2_*`); 4x a wide inner stage (`os4_*`) inside a steep outer one
/// (`os4_*_outer`). `rate=host` injections carry a copy of each up-filter.
fn os_states(ir: &CircuitIR) -> Vec<(String, String, String)> {
    let Some(rt) = runtime(ir) else {
        return Vec::new();
    };
    let inject = ir.solver_config.has_inject_or_tap();
    let mut v = Vec::new();
    for &f in rt.factors.iter().filter(|&&f| f > 1) {
        let info = oversampling_info(f);
        let mut stage = |suffix: &str, size: usize| {
            v.push((
                format!("os{f}_up_state{suffix}"),
                format!("[f64; {size}]"),
                format!("[0.0; {size}]"),
            ));
            v.push((
                format!("os{f}_dn_state{suffix}"),
                format!("[[f64; {size}]; NUM_OUTPUTS]"),
                format!("[[0.0; {size}]; NUM_OUTPUTS]"),
            ));
            if inject {
                v.push((
                    format!("os{f}_inj_up_state{suffix}"),
                    format!("[[f64; {size}]; NUM_INJECT_HOST]"),
                    format!("[[0.0; {size}]; NUM_INJECT_HOST]"),
                ));
            }
        };
        stage("", info.state_size);
        if f == 4 {
            stage("_outer", info.state_size_outer);
        }
    }
    v
}

/// `CircuitState` field declarations for the runtime build's filter states.
pub(super) fn os_state_fields(ir: &CircuitIR) -> String {
    os_states(ir)
        .iter()
        .map(|(n, t, _)| {
            format!("    /// Runtime oversampling half-band filter state.\n    pub {n}: {t},\n")
        })
        .collect()
}

/// Their `Default` initializers.
pub(super) fn os_state_inits(ir: &CircuitIR) -> String {
    os_states(ir)
        .iter()
        .map(|(n, _, z)| format!("            {n}: {z},\n"))
        .collect()
}

/// The runtime build's methods: the running factor, switching it, and zeroing
/// every factor's filter state (called wherever a fixed build zeroes its own).
pub(super) fn emit_methods(ir: &CircuitIR) -> String {
    let Some(_) = runtime(ir) else {
        return String::new();
    };
    let zero: String = os_states(ir)
        .iter()
        .map(|(n, _, z)| format!("        self.{n} = {z};\n"))
        .collect();
    format!(
        "    /// The running oversampling factor (one of `OVERSAMPLING_SET`).\n\
         \x20   pub fn oversampling(&self) -> usize {{\n\
         \x20       self.oversampling\n\
         \x20   }}\n\n\
         \x20   /// Run at oversampling `factor`, one of `OVERSAMPLING_SET`. The state is\n\
         \x20   /// replaced by a freshly constructed one at `factor` (as `default()`\n\
         \x20   /// constructs at `OVERSAMPLING_FACTOR`: the DC operating point, every\n\
         \x20   /// control — pots, switches, runtime values, noise settings — at its\n\
         \x20   /// construction value, the warmup run), then `set_sample_rate` is called\n\
         \x20   /// at the current host rate: exactly the state a fixed build at `factor`\n\
         \x20   /// has after construction and `set_sample_rate`. Not\n\
         \x20   /// real-time safe (it rebuilds the state and allocates): call it off the\n\
         \x20   /// audio thread, then re-apply control values and re-warm, as after\n\
         \x20   /// construction. A factor outside the set is refused and changes nothing.\n\
         \x20   pub fn set_oversampling(&mut self, factor: usize) -> Result<(), &'static str> {{\n\
         \x20       if !OVERSAMPLING_SET.contains(&factor) {{\n\
         \x20           return Err(\"oversampling factor not in OVERSAMPLING_SET\");\n\
         \x20       }}\n\
         \x20       let sample_rate = self.current_sample_rate;\n\
         \x20       *self = Self::new_at(factor);\n\
         \x20       self.set_sample_rate(sample_rate);\n\
         \x20       Ok(())\n\
         \x20   }}\n\n\
         \x20   /// Zero every oversampling factor's half-band filter state.\n\
         \x20   fn reset_oversampler(&mut self) {{\n{zero}    }}\n\n"
    )
}

/// The element type of an array type `[T; LEN]` (`[[f64; 2]; 7]` → `[f64; 2]`).
fn array_element_type(ty: &str) -> Option<String> {
    let inner = ty.trim().strip_prefix('[')?.strip_suffix(']')?;
    let (elem, _len) = inner.rsplit_once(';')?;
    Some(elem.trim().to_string())
}

/// Make a runtime build's constructor take the factor: `impl Default` becomes
/// `fn new_at(os)` (the same construction, every switched constant read at
/// `os`, the factor field set to it), and `default()` is `new_at` at the
/// default factor. `set_oversampling` constructs through it, so a switch is
/// construction at the new factor, warmup included.
fn constructor_at(ir: &CircuitIR, code: String) -> Result<String, CodegenError> {
    let Some(rt) = runtime(ir) else {
        return Ok(code);
    };
    let head = "impl Default for CircuitState {";
    let Some(start) = code.find(head) else {
        return Err(CodegenError::UnsupportedTopology(format!(
            "runtime oversampling: no `{head}` in the generated code"
        )));
    };
    // The impl block's extent, by brace depth from its opening brace.
    let open = start + head.len() - 1;
    let mut depth = 0i32;
    let mut end = None;
    for (i, c) in code[open..].char_indices() {
        match c {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    end = Some(open + i + 1);
                    break;
                }
            }
            _ => {}
        }
    }
    let malformed = || {
        CodegenError::UnsupportedTopology(
            "runtime oversampling: the generated Default impl is not the expected shape".into(),
        )
    };
    let end = end.ok_or_else(malformed)?;
    let block = &code[start..end];
    let fn_head = "fn default() -> Self {";
    let body_start = block.find(fn_head).ok_or_else(malformed)? + fn_head.len();
    let body_end = block.rfind('}').ok_or_else(malformed)?;
    let body_end = block[..body_end].rfind('}').ok_or_else(malformed)?;
    let accessors: Vec<(String, String)> = rt
        .per_factor_consts
        .iter()
        .map(|c| {
            (
                c.name.clone(),
                format!("{}_for(os)", c.name.to_ascii_lowercase()),
            )
        })
        .chain(std::iter::once((
            "OVERSAMPLING_FACTOR".to_string(),
            "os".to_string(),
        )))
        .collect();
    let map: Vec<(&str, &str)> = accessors
        .iter()
        .map(|(a, b)| (a.as_str(), b.as_str()))
        .collect();
    let body = rename_idents(&block[body_start..body_end], &map);
    let new = format!(
        "impl Default for CircuitState {{\n    fn default() -> Self {{\n        Self::new_at(OVERSAMPLING_FACTOR)\n    }}\n}}\n\n\
         impl CircuitState {{\n    /// A freshly constructed state running at oversampling `os` (one of\n\
         \x20   /// `OVERSAMPLING_SET`): what `default()` constructs at `OVERSAMPLING_FACTOR`.\n\
         \x20   fn new_at(os: usize) -> Self {{{body}}}\n}}"
    );
    Ok(format!("{}{}{}", &code[..start], new, &code[end..]))
}

/// `code` with each identifier `from` replaced by `to` (whole identifiers
/// only, so `os_halfband` does not rename the start of `os_halfband_down`).
pub(super) fn rename_idents(code: &str, map: &[(&str, &str)]) -> String {
    let mut out = String::with_capacity(code.len());
    let bytes = code.as_bytes();
    let is_ident = |b: u8| b.is_ascii_alphanumeric() || b == b'_';
    let mut i = 0;
    while i < bytes.len() {
        if is_ident(bytes[i]) && (i == 0 || !is_ident(bytes[i - 1])) {
            let mut j = i;
            while j < bytes.len() && is_ident(bytes[j]) {
                j += 1;
            }
            let word = &code[i..j];
            match map.iter().find(|(f, _)| *f == word) {
                Some((_, to)) => out.push_str(to),
                None => out.push_str(word),
            }
            i = j;
        } else {
            let c = code[i..].chars().next().expect("in bounds");
            out.push(c);
            i += c.len_utf8();
        }
    }
    out
}

/// A runtime build's switched constants keep their unsuffixed definitions
/// (`S_DEFAULT`, read by nothing but documentation and the default factor's
/// construction): pin each to the default factor's literal, so the code does
/// not depend on which factor's IR it was emitted from.
fn pin_default_consts(rt: &RuntimeOversampling, code: String) -> String {
    let Some(d) = rt.factors.iter().position(|&f| f == rt.default) else {
        return code;
    };
    let mut items: Vec<(std::ops::Range<usize>, String)> = Vec::new();
    for item in crate::codegen::const_text::const_items(&code) {
        if let Some(c) = rt.per_factor_consts.iter().find(|c| c.name == item.name) {
            items.push((item.ty_range, c.types[d].clone()));
            items.push((item.value_range, c.values[d].clone()));
        }
    }
    items.sort_by_key(|(r, _)| std::cmp::Reverse(r.start));
    let mut code = code;
    for (r, text) in items {
        code.replace_range(r, &text);
    }
    code
}

/// Refuse runtime code that still reads the default factor where the running
/// one is meant: an `OVERSAMPLING_FACTOR`, `INTERNAL_SAMPLE_RATE` or `ALPHA` use
/// outside its definition, its doc comment and the default constructor. An
/// emitter that writes the factor in a form `finalize` does not rewrite would
/// otherwise run at the default factor's value at every factor.
fn audit_factor_reads(code: &str) -> Result<(), CodegenError> {
    for (n, line) in code.lines().enumerate() {
        let t = line.trim_start();
        if t.starts_with("//")
            || t.starts_with("pub const OVERSAMPLING_FACTOR:")
            || t.starts_with("pub const INTERNAL_SAMPLE_RATE:")
            || t.starts_with("pub const ALPHA:")
            || t.contains("Self::new_at(OVERSAMPLING_FACTOR)")
        {
            continue;
        }
        // String literal contents are text, not reads (a node may be named
        // `ALPHA` in a diagnostic print); blank them before looking.
        let (mut unquoted, mut in_str, mut escaped) = (String::new(), false, false);
        for c in t.chars() {
            if in_str {
                in_str = !(c == '"' && !escaped);
                escaped = c == '\\' && !escaped;
                unquoted.push(if in_str { ' ' } else { c });
            } else {
                in_str = c == '"';
                unquoted.push(c);
            }
        }
        let code_part = unquoted.split("//").next().unwrap_or(&unquoted);
        for name in ["OVERSAMPLING_FACTOR", "INTERNAL_SAMPLE_RATE", "ALPHA"] {
            if rename_idents(code_part, &[(name, "\u{1}")]).contains('\u{1}') {
                return Err(CodegenError::UnsupportedTopology(format!(
                    "runtime oversampling: generated line {} reads `{name}`, the default \
                     factor's, where the running factor is meant: `{}`. This is an emitter \
                     form the runtime build does not handle; build each factor as its own \
                     deck.",
                    n + 1,
                    t.trim()
                )));
            }
        }
    }
    Ok(())
}
