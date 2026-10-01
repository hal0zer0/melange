//! Oversampling and `.inject`/`.tap` public-entry wrapper emission shared by the DK and nodal paths.

use super::helpers::{fmt_f64, oversampling_info, section_banner};
use super::RustEmitter;
use crate::codegen::ir::CircuitIR;

impl RustEmitter {
    /// Emit oversampling wrapper: constants, allpass helper, halfband, and process_sample.
    pub(super) fn emit_oversampler(ir: &CircuitIR) -> String {
        let factor = ir.solver_config.oversampling_factor;
        let info = oversampling_info(factor);
        let mut code = section_banner("OVERSAMPLING");

        // Emit coefficients as constants
        code.push_str("/// Half-band filter coefficients for allpass polyphase oversampler.\n");
        let coeffs_str = info
            .coeffs
            .iter()
            .map(|c| fmt_f64(*c))
            .collect::<Vec<_>>()
            .join(", ");
        code.push_str(&format!(
            "const OS_COEFFS: [f64; {}] = [{}];\n\n",
            info.num_sections, coeffs_str
        ));

        if factor == 4 {
            let coeffs_outer_str = info
                .coeffs_outer
                .iter()
                .map(|c| fmt_f64(*c))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "const OS_COEFFS_OUTER: [f64; {}] = [{}];\n\n",
                info.num_sections_outer, coeffs_outer_str
            ));
        }

        // Emit allpass inline function (takes slice + offset to avoid double &mut borrow)
        code.push_str(
            "/// First-order allpass section: y = c*(x - y1) + x1\n\
             /// State layout: state[base] = x1, state[base+1] = y1\n\
             #[inline(always)]\n\
             fn os_allpass(x: f64, c: f64, state: &mut [f64], base: usize) -> f64 {\n\
             \x20   let y = c * x + state[base] - c * state[base + 1];\n\
             \x20   state[base] = x;\n\
             \x20   state[base + 1] = y;\n\
             \x20   y\n\
             }\n\n",
        );

        // Emit polyphase interpolator + decimator functions.
        // Upsampler and downsampler use SEPARATE state arrays; each branch of
        // each filter is clocked exactly once per LOW-rate sample (polyphase).
        Self::emit_halfband_fn(&mut code, "os_halfband", &info.coeffs, info.state_size);
        Self::emit_halfband_down_fn(&mut code, "os_halfband_down", &info.coeffs, info.state_size);
        if factor == 4 {
            Self::emit_halfband_fn(
                &mut code,
                "os_halfband_outer",
                &info.coeffs_outer,
                info.state_size_outer,
            );
            Self::emit_halfband_down_fn(
                &mut code,
                "os_halfband_down_outer",
                &info.coeffs_outer,
                info.state_size_outer,
            );
        }

        let num_outputs = ir.solver_config.output_nodes.len();
        let inject_or_tap = ir.solver_config.has_inject_or_tap();

        // Emit the public process_sample wrapper
        code.push_str("/// Process a single audio sample through the circuit with oversampling.\n");
        code.push_str("///\n");
        code.push_str(&format!(
            "/// Runs the circuit at {}x the host sample rate to reduce aliasing.\n",
            factor
        ));
        if inject_or_tap {
            code.push_str(
                "///\n\
                 /// `injections_inner[k]` supplies the `.inject` values for internal sample `k`\n\
                 /// (already at the internal rate — routed straight to the inner solve, NOT\n\
                 /// through the anti-alias up-filter). Returns the decimated outputs plus the\n\
                 /// RAW per-inner-sample `.tap` node voltages `taps_inner` (un-decimated). In a\n\
                 /// feedback loop, `injections_inner` must be derived from a PRIOR sample's tap —\n\
                 /// this is >=1 sample of delay by construction. Inner-sample order: 2x [even,\n\
                 /// odd]; 4x [e0, o0, e1, o1].\n",
            );
        }
        code.push_str("#[inline]\n");
        if inject_or_tap {
            code.push_str(
                "pub fn process_sample(input: f64, injections_inner: &[[f64; NUM_INJECT]; OVERSAMPLING_FACTOR], state: &mut CircuitState) -> ([f64; NUM_OUTPUTS], [[f64; NUM_TAP]; OVERSAMPLING_FACTOR]) {\n",
            );
        } else {
            code.push_str(
                "pub fn process_sample(input: f64, state: &mut CircuitState) -> [f64; NUM_OUTPUTS] {\n",
            );
        }
        code.push_str(
            "    let input = if !input.is_finite() { state.diag_input_nan_count += 1; 0.0 } else if input.abs() > INPUT_LIMIT_V { state.diag_input_clamp_count += 1; input.clamp(-INPUT_LIMIT_V, INPUT_LIMIT_V) } else { input };\n\n",
        );

        if factor == 2 {
            Self::emit_2x_wrapper(
                &mut code,
                num_outputs,
                ir.dc_block,
                ir.solver_config.output_clamp_v,
                inject_or_tap,
            );
        } else if factor == 4 {
            Self::emit_4x_wrapper(
                &mut code,
                num_outputs,
                ir.dc_block,
                ir.solver_config.output_clamp_v,
                inject_or_tap,
            );
        }

        code.push_str("}\n\n");
        code
    }

    /// Emit the 1x public `process_sample` wrapper for a circuit with `.inject`
    /// / `.tap` but no oversampling. The inner body is emitted as a private
    /// `process_sample_inner` (the template does this whenever `inject_or_tap`),
    /// so this thin wrapper adapts the per-inner-sample array API (arity 1).
    pub(super) fn emit_inject_wrapper_1x(ir: &CircuitIR) -> String {
        debug_assert_eq!(ir.solver_config.oversampling_factor, 1);
        let mut code = String::new();
        code.push_str("/// Process a single audio sample through the circuit.\n");
        code.push_str(
            "///\n\
             /// `injections_inner[0]` supplies the `.inject` values for this sample. Returns\n\
             /// the outputs plus the RAW `.tap` node voltages `taps_inner[0]`. In a feedback\n\
             /// loop, `injections_inner` must be derived from a PRIOR sample's tap — this is\n\
             /// >=1 sample of delay by construction.\n",
        );
        code.push_str("#[inline]\n");
        code.push_str(
            "pub fn process_sample(input: f64, injections_inner: &[[f64; NUM_INJECT]; OVERSAMPLING_FACTOR], state: &mut CircuitState) -> ([f64; NUM_OUTPUTS], [[f64; NUM_TAP]; OVERSAMPLING_FACTOR]) {\n",
        );
        code.push_str(
            "    let input = if !input.is_finite() { state.diag_input_nan_count += 1; 0.0 } else if input.abs() > INPUT_LIMIT_V { state.diag_input_clamp_count += 1; input.clamp(-INPUT_LIMIT_V, INPUT_LIMIT_V) } else { input };\n",
        );
        code.push_str(
            "    let (output, tap) = process_sample_inner(input, injections_inner[0], state);\n",
        );
        code.push_str("    (output, [tap])\n");
        code.push_str("}\n\n");
        code
    }

    /// Emit a polyphase half-band interpolator step: one low-rate input
    /// produces the two internal-rate samples `(out[2n], out[2n+1])` from the
    /// even (A0) and odd (A1) allpass branches. Each branch is clocked once
    /// per call.
    pub(super) fn emit_halfband_fn(
        code: &mut String,
        name: &str,
        coeffs: &[f64],
        state_size: usize,
    ) {
        let num_sections = coeffs.len();
        let even_count = num_sections.div_ceil(2);
        let odd_count = num_sections / 2;

        code.push_str(&format!(
            "/// Half-band interpolator step: (even, odd) = (out[2n], out[2n+1]).\n\
             /// Each allpass branch is clocked once per low-rate input sample.\n\
             #[inline(always)]\n\
             fn {name}(input: f64, coeffs: &[f64; {num_sections}], state: &mut [f64; {state_size}]) -> (f64, f64) {{\n"
        ));

        // Even chain: coefficients at indices 0, 2, 4, ...
        // State layout: even sections first, then odd sections
        code.push_str("    let mut even = input;\n");
        for i in 0..even_count {
            let coeff_idx = i * 2; // even-indexed coefficients
            let state_base = i * 2; // sequential state storage for even chain
            code.push_str(&format!(
                "    even = os_allpass(even, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        // Odd chain: coefficients at indices 1, 3, 5, ...
        code.push_str("    let mut odd = input;\n");
        let odd_state_offset = even_count * 2;
        for i in 0..odd_count {
            let coeff_idx = i * 2 + 1; // odd-indexed coefficients
            let state_base = odd_state_offset + i * 2;
            code.push_str(&format!(
                "    odd = os_allpass(odd, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        code.push_str("    (even, odd)\n");
        code.push_str("}\n\n");
    }

    /// Emit a polyphase half-band decimator step: a pair of internal-rate
    /// samples (`x0` earlier, `x1` later) produces one low-rate output.
    ///
    /// hiir convention (`Downsampler2x::process_sample`): the even (A0)
    /// branch filters the LATER sample, the odd (A1) branch the EARLIER
    /// sample; output is their average. Each branch is clocked exactly once
    /// per output sample — clocking a branch twice per output (the pre-2026-07
    /// bug) collapses the allpass cells to first-order in the internal-rate z
    /// and destroys the stopband entirely.
    pub(super) fn emit_halfband_down_fn(
        code: &mut String,
        name: &str,
        coeffs: &[f64],
        state_size: usize,
    ) {
        let num_sections = coeffs.len();
        let even_count = num_sections.div_ceil(2);
        let odd_count = num_sections / 2;

        code.push_str(&format!(
            "/// Half-band decimator step: y[n] = (A_even(x[2n+1]) + A_odd(x[2n])) / 2.\n\
             /// Each allpass branch is clocked once per low-rate output sample.\n\
             #[inline(always)]\n\
             fn {name}(x0: f64, x1: f64, coeffs: &[f64; {num_sections}], state: &mut [f64; {state_size}]) -> f64 {{\n"
        ));

        // Even chain (coefficients 0, 2, 4, ...) consumes the LATER sample.
        // State layout matches the interpolator: even sections first.
        code.push_str("    let mut even = x1;\n");
        for i in 0..even_count {
            let coeff_idx = i * 2;
            let state_base = i * 2;
            code.push_str(&format!(
                "    even = os_allpass(even, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        // Odd chain (coefficients 1, 3, 5, ...) consumes the EARLIER sample.
        code.push_str("    let mut odd = x0;\n");
        let odd_state_offset = even_count * 2;
        for i in 0..odd_count {
            let coeff_idx = i * 2 + 1;
            let state_base = odd_state_offset + i * 2;
            code.push_str(&format!(
                "    odd = os_allpass(odd, coeffs[{coeff_idx}], state, {state_base});\n",
            ));
        }

        code.push_str("    (even + odd) * 0.5\n");
        code.push_str("}\n\n");
    }

    /// Emit the 2x oversampling wrapper body.
    pub(super) fn emit_2x_wrapper(
        code: &mut String,
        _num_outputs: usize,
        dc_block: bool,
        clamp_v: f64,
        inject_or_tap: bool,
    ) {
        // Upsample: polyphase interpolator, 1 input → 2 internal-rate samples
        code.push_str(
            "    // Upsample: interpolator produces (out[2n], out[2n+1]) at internal rate\n\
             \x20   let (up_even, up_odd) = os_halfband(input, &OS_COEFFS, &mut state.os_up_state);\n\n",
        );

        // Process both at 2x rate. Injections BYPASS the up-filter (already
        // inner-rate): injections_inner[0]=even, [1]=odd. Taps are raw.
        if inject_or_tap {
            code.push_str(
                "    // Process both samples at 2x rate (up_even is the earlier sample)\n\
                 \x20   let (out_even, tap_even) = process_sample_inner(up_even, injections_inner[0], state);\n\
                 \x20   let (out_odd, tap_odd) = process_sample_inner(up_odd, injections_inner[1], state);\n\n",
            );
        } else {
            code.push_str(
                "    // Process both samples at 2x rate (up_even is the earlier sample)\n\
                 \x20   let out_even = process_sample_inner(up_even, state);\n\
                 \x20   let out_odd = process_sample_inner(up_odd, state);\n\n",
            );
        }

        // Downsample per-output: ONE decimator step per output sample
        code.push_str("    // Downsample: per-output polyphase decimator, 2 samples → 1\n");
        code.push_str("    let mut result = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let v = os_halfband_down(out_even[out_idx], out_odd[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        if dc_block {
            code.push_str(&format!(
                "        result[out_idx] = if v.is_finite() {{ v.clamp(-{clamp_v:e}, {clamp_v:e}) }} else {{ 0.0 }};\n",
            ));
        } else {
            code.push_str("        result[out_idx] = if v.is_finite() { v } else { 0.0 };\n");
        }
        code.push_str("    }\n");
        if inject_or_tap {
            // Taps are RAW / un-decimated — one array per internal sample.
            code.push_str("    (result, [tap_even, tap_odd])\n");
        } else {
            code.push_str("    result\n");
        }
    }

    /// Emit the 4x oversampling wrapper body (cascaded 2x stages).
    pub(super) fn emit_4x_wrapper(
        code: &mut String,
        _num_outputs: usize,
        dc_block: bool,
        clamp_v: f64,
        inject_or_tap: bool,
    ) {
        // Outer upsample: 1 → 2 at 2x rate (steep base-Nyquist filter)
        code.push_str(
            "    // Outer upsample: 1 → 2 samples at 2x rate (steep filter)\n\
             \x20   let (outer_even, outer_odd) = os_halfband_outer(\n\
             \x20       input, &OS_COEFFS_OUTER, &mut state.os_up_state_outer,\n\
             \x20   );\n\n",
        );

        // Inner upsample + process for each outer sample. Injections BYPASS
        // both up-filters (already inner-rate): order [e0, o0, e1, o1]. Taps raw.
        if inject_or_tap {
            code.push_str(
                "    // Inner upsample + process: each 2x sample → 2 samples at 4x rate\n\
                 \x20   let (inner_e0, inner_o0) = os_halfband(outer_even, &OS_COEFFS, &mut state.os_up_state);\n\
                 \x20   let (proc_e0, tap_e0) = process_sample_inner(inner_e0, injections_inner[0], state);\n\
                 \x20   let (proc_o0, tap_o0) = process_sample_inner(inner_o0, injections_inner[1], state);\n\n",
            );
        } else {
            code.push_str(
                "    // Inner upsample + process: each 2x sample → 2 samples at 4x rate\n\
                 \x20   let (inner_e0, inner_o0) = os_halfband(outer_even, &OS_COEFFS, &mut state.os_up_state);\n\
                 \x20   let proc_e0 = process_sample_inner(inner_e0, state);\n\
                 \x20   let proc_o0 = process_sample_inner(inner_o0, state);\n\n",
            );
        }

        // Inner decimator per-output for first 2x pair (one step per pair)
        code.push_str("    let mut inner_out0 = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        inner_out0[out_idx] = os_halfband_down(proc_e0[out_idx], proc_o0[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("    }\n\n");

        // Second inner upsample + process pair
        if inject_or_tap {
            code.push_str(
                "    let (inner_e1, inner_o1) = os_halfband(outer_odd, &OS_COEFFS, &mut state.os_up_state);\n\
                 \x20   let (proc_e1, tap_e1) = process_sample_inner(inner_e1, injections_inner[2], state);\n\
                 \x20   let (proc_o1, tap_o1) = process_sample_inner(inner_o1, injections_inner[3], state);\n\n",
            );
        } else {
            code.push_str(
                "    let (inner_e1, inner_o1) = os_halfband(outer_odd, &OS_COEFFS, &mut state.os_up_state);\n\
                 \x20   let proc_e1 = process_sample_inner(inner_e1, state);\n\
                 \x20   let proc_o1 = process_sample_inner(inner_o1, state);\n\n",
            );
        }

        // Inner decimator per-output for second 2x pair
        code.push_str("    let mut inner_out1 = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        inner_out1[out_idx] = os_halfband_down(proc_e1[out_idx], proc_o1[out_idx], &OS_COEFFS, &mut state.os_dn_state[out_idx]);\n");
        code.push_str("    }\n\n");

        // Outer decimator per-output: 2 samples at 2x rate → 1 at host rate
        code.push_str("    // Outer downsample: per-output polyphase decimator (steep filter)\n");
        code.push_str("    let mut result = [0.0f64; NUM_OUTPUTS];\n");
        code.push_str("    for out_idx in 0..NUM_OUTPUTS {\n");
        code.push_str("        let v = os_halfband_down_outer(\n");
        code.push_str("            inner_out0[out_idx], inner_out1[out_idx], &OS_COEFFS_OUTER, &mut state.os_dn_state_outer[out_idx],\n");
        code.push_str("        );\n");
        if dc_block {
            code.push_str(&format!(
                "        result[out_idx] = if v.is_finite() {{ v.clamp(-{clamp_v:e}, {clamp_v:e}) }} else {{ 0.0 }};\n",
            ));
        } else {
            code.push_str("        result[out_idx] = if v.is_finite() { v } else { 0.0 };\n");
        }
        code.push_str("    }\n");
        if inject_or_tap {
            // Taps are RAW / un-decimated — one array per internal sample,
            // in the same [e0, o0, e1, o1] order as injections_inner.
            code.push_str("    (result, [tap_e0, tap_o0, tap_e1, tap_o1])\n");
        } else {
            code.push_str("    result\n");
        }
    }
}
