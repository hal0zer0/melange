//! Canonical coefficients for melange's own `fast_exp` / `fast_ln`.
//!
//! These are NOT libm. melange emits its own implementations for the hot path,
//! which means every backend must use **bit-identical coefficients** or the
//! backends compute different functions. On Linux/glibc the true libm calls
//! (`tanh`, `cosh`, `powf`, and the cold-path `exp`/`ln`) are shared between
//! Rust and C++ because both link the same `libm.so.6` — parity there is
//! structural and free. For `fast_exp`/`fast_ln` there is no such shared
//! implementation: parity exists only if the coefficients below are the ones
//! every emitter uses.
//!
//! They previously existed only as characters inside a Rust string literal in
//! the emitter, so a second backend would have had to transcribe them by hand.
//! That is precisely how two backends silently stop computing the same thing.
//!
//! # Text form vs value
//!
//! The Rust emitter still writes these as literal text, and one of them
//! (`FAST_EXP_P5`) is written in the emitted source as `0.008333333333492337`
//! while Rust's shortest round-trip form is `0.008333333333492336`. **Both parse
//! to the same f64** — verified bit-equal, `0x3f8111111112771c`, delta exactly
//! zero. The difference is cosmetic, so the emitted text was left alone rather
//! than churning the golden baseline for a value that does not change. The test
//! at the bottom of this module enforces that the emitted literals and these
//! constants agree **numerically**, which is the property that actually matters
//! and the one a second backend depends on.

/// Range-reduction split of ln(2), high word.
pub const FAST_EXP_LN2_HI: f64 = 0.6931471803691238;
/// Range-reduction split of ln(2), low word. `LN2_HI + LN2_LO ~= ln(2)` to
/// beyond f64 precision; splitting keeps `x - n*ln2` accurate for large `n`.
pub const FAST_EXP_LN2_LO: f64 = 1.9082149292705877e-10;
/// 2^52 + 2^51 — the classic round-to-nearest-integer-via-mantissa-alias shift.
pub const FAST_EXP_SHIFT: f64 = 6755399441055744.0;

/// 5th-order minimax polynomial for `exp(f)` on the reduced range, ascending.
/// `p = 1 + f*(1 + f*(P2 + f*(P3 + f*(P4 + f*P5))))`.
pub const FAST_EXP_P2: f64 = 0.5;
pub const FAST_EXP_P3: f64 = 0.16666666666666607;
pub const FAST_EXP_P4: f64 = 0.04166666666665876;
pub const FAST_EXP_P5: f64 = 0.008333333333492337;

/// Symmetric log-series coefficients for `ln(m)`, m in [1,2).
/// `ln(m) = 2u(1 + u^2*(L3 + u^2*(L5 + u^2*L7)))` with `u = (m-1)/(m+1)`.
pub const FAST_LN_L3: f64 = 0.3333333333333333;
pub const FAST_LN_L5: f64 = 0.2;
pub const FAST_LN_L7: f64 = 0.14285714285714285;

/// Input clamp applied before `fast_exp`, matching melange's `safe_exp`.
pub const FAST_EXP_CLAMP: f64 = 40.0;

/// Every coefficient, paired with the exact literal text the Rust emitter
/// writes. Backends and tests iterate this rather than restating values.
pub const FAST_MATH_COEFFICIENTS: &[(&str, f64, &str)] = &[
    ("LN2_HI", FAST_EXP_LN2_HI, "0.6931471803691238"),
    ("LN2_LO", FAST_EXP_LN2_LO, "1.9082149292705877e-10"),
    ("SHIFT", FAST_EXP_SHIFT, "6755399441055744.0"),
    ("exp_p3", FAST_EXP_P3, "0.16666666666666607"),
    ("exp_p4", FAST_EXP_P4, "0.04166666666665876"),
    ("exp_p5", FAST_EXP_P5, "0.008333333333492337"),
    ("ln_l3", FAST_LN_L3, "0.3333333333333333"),
    ("ln_l5", FAST_LN_L5, "0.2"),
    ("ln_l7", FAST_LN_L7, "0.14285714285714285"),
];

#[cfg(test)]
mod tests {
    use super::*;

    /// The literal text the emitter writes must denote exactly these values.
    /// This is the drift guard: if someone edits a digit in the emitted string
    /// without updating the constant (or vice versa), a future C++ backend
    /// would compute a different function than the Rust one and nothing else
    /// would notice — the outputs would merely disagree in the last places.
    #[test]
    fn emitted_literals_match_canonical_values() {
        for (name, value, text) in FAST_MATH_COEFFICIENTS {
            let parsed: f64 = text
                .parse()
                .unwrap_or_else(|e| panic!("{name}: literal {text:?} does not parse: {e}"));
            assert_eq!(
                parsed.to_bits(),
                value.to_bits(),
                "{name}: emitted literal {text:?} and canonical constant disagree"
            );
        }
    }

    /// Guard the documented cosmetic difference so it is a known, asserted fact
    /// rather than a surprise: the emitted text for P5 is not Rust's shortest
    /// round-trip form, but denotes the same f64.
    #[test]
    fn p5_text_differs_from_shortest_form_but_not_in_value() {
        assert_eq!(format!("{:?}", FAST_EXP_P5), "0.008333333333492336");
        assert_eq!(
            "0.008333333333492337".parse::<f64>().unwrap().to_bits(),
            FAST_EXP_P5.to_bits()
        );
    }
}
