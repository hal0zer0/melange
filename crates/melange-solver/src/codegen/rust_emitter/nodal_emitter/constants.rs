//! Nodal constants emission.

use crate::codegen::ir::CircuitIR;
use crate::codegen::rust_emitter::dk_emitter::emit_inject_tap_constants;
use crate::codegen::rust_emitter::helpers::{
    fmt_f64, format_matrix_rows, has_latched_device, recommended_warmup_samples, section_banner,
    warmup_estimate_capped,
};
use crate::codegen::rust_emitter::RustEmitter;

impl RustEmitter {
    /// Emit constants section for nodal solver.
    ///
    /// Includes A, A_neg, A_be, A_neg_be, N_v, N_i, G, C, RHS_CONST, RHS_CONST_BE,
    /// DC_OP, DC_NL_I, and topology dimensions. No S, K, or S_NI.
    pub(super) fn emit_nodal_constants(&self, ir: &CircuitIR, use_full_nodal: bool) -> String {
        let n = ir.topology.n;
        let m = ir.topology.m;
        let n_nodes = if ir.topology.n_nodes > 0 {
            ir.topology.n_nodes
        } else {
            n
        };
        let n_aug = ir.topology.n_aug;
        let num_outputs = ir.solver_config.output_nodes.len();

        let mut code = section_banner("CONSTANTS: Compile-time circuit topology (Nodal solver)");

        // Dimension constants
        code.push_str(
            "/// Number of augmented system nodes (including VS/VCVS/inductor branch variables)\n",
        );
        code.push_str(&format!("pub const N: usize = {};\n\n", n));
        code.push_str("/// Number of original circuit nodes (excluding ground)\n");
        code.push_str(&format!("pub const N_NODES: usize = {};\n\n", n_nodes));
        code.push_str("/// Boundary between VS/VCVS rows and inductor branch variables\n");
        code.push_str(&format!("pub const N_AUG: usize = {};\n\n", n_aug));
        code.push_str("/// Total nonlinear dimension (sum of device dimensions)\n");
        code.push_str(&format!("pub const M: usize = {};\n\n", m));
        code.push_str(
            "/// Implausibility bound for a *finite* per-sample NR state. A NaN/Inf\n\
             /// iterate is caught unconditionally; this catches the gap where a solve\n\
             /// saturates (exp() clamp, near-singular LU) and produces an\n\
             /// astronomically large but technically finite value that would\n\
             /// otherwise bypass every NaN guard and get written into `v_prev`/\n\
             /// `i_nl_prev` for every subsequent sample. ~2000x above the highest\n\
             /// real supply rail seen in practice (480 V tube plate\n\
             /// supplies) — see docs/aidocs/DEBUGGING.md \"finite runaway\" entry.\n",
        );
        code.push_str("pub const STATE_MAX_PLAUSIBLE_MAGNITUDE: f64 = 1e6;\n\n");
        code.push_str(
            "/// Largest input voltage the circuit is driven with, in volts. A larger\n\
             /// input is clamped to it and counted in `diag_input_clamp_count`; a NaN or\n\
             /// infinite input is replaced by 0 V and counted in `diag_input_nan_count`.\n\
             /// Both are witnesses that the circuit was not driven with what was asked.\n\
             pub const INPUT_LIMIT_V: f64 = 100.0;\n\n",
        );
        code.push_str("/// Maximum NR iterations per sample\n");
        // Budget CEILING, not a target, floored at `NODAL_MAX_ITER_FLOOR`
        // (`codegen::policy`, where the reason is). Auto-tuned budgets already
        // above it are left untouched. The floor lives in
        // `CircuitIR::effective_max_iter` so the provenance `Build:` line, the
        // JSON and the console report the same value this const emits.
        code.push_str(&format!(
            "pub const MAX_ITER: usize = {};\n\n",
            ir.effective_max_iter()
        ));
        // The sub-step ladder runs wherever a Newton solve can fail: every
        // full-LU build, and a Schur build with devices. Emitting the bound
        // elsewhere would publish a number that governs nothing.
        if use_full_nodal || m > 0 {
            code.push_str(
                "/// Finest sub-step the adaptive sub-stepping will try before giving up:\n\
             /// T / 2^SUBSTEP_MAX_POWER. A failing sub-step is bisected (only that\n\
             /// sub-step; the converged prefix is kept), down to this depth.\n\
             ///\n\
             /// A transient Newton failure is a TIMESTEP problem, not a budget problem,\n\
             /// so the response is to cut dt and retry rather than to raise MAX_ITER\n\
             /// (design review). Past this depth, or past SUBSTEP_BUDGET attempts, the\n\
             /// death-spiral hold fires and `diag_unsolved_sample_count` records a\n\
             /// sample that is not a solution.\n",
            );
            code.push_str("pub const SUBSTEP_MAX_POWER: u32 = 12;\n\n");
            code.push_str(
                "/// Most sub-step attempts (converged or bisected) one sample may spend\n\
                 /// in the adaptive sub-stepping: the bound on a rescued sample's cost.\n\
                 ///\n\
                 /// Measured on an IC-seeded transistor astable (the hardest switching\n\
                 /// edges in the test set) at 48 and 96 kHz on both nodal sub-paths: the\n\
                 /// deepest bisection reached 2^9, the most attempts one sample used was\n\
                 /// 31, the mean about 12. 64 is twice that, and half the 126 sub-steps a\n\
                 /// uniform 2..64x restart could spend (design review).\n",
            );
            code.push_str("pub const SUBSTEP_BUDGET: u32 = 64;\n\n");
        }
        if ir.solver_config.breakpoint_be {
            code.push_str(
                "/// Breakpoint-BE: number of samples solved on the backward-Euler matrices\n\
                 /// after a .switch swaps a capacitor or an inductor (a glow device also\n\
                 /// holds the countdown while lit). Exactly ONE: a single BE sample does\n\
                 /// not read the carried q_dot (built on the old component values),\n\
                 /// re-seeds it from its own capacitor currents, and damps the mode the\n\
                 /// step excited (BE is L-stable); then trap resumes.\n\
                 /// Do NOT raise this — a second BE sample over-damps and can knock a\n\
                 /// marginal self-oscillator (e.g. an organ frequency-divider stage under\n\
                 /// --force-trap) into the wrong equilibrium.\n",
            );
            code.push_str("pub const BREAKPOINT_BE_SAMPLES: u32 = 1;\n\n");
            code.push_str(
                "/// Breakpoint-BE: NR iteration budget for the forced-BE samples. Kept well\n\
                 /// above MAX_ITER so a forced-BE sample never hits the trap per-sample wall\n\
                 /// (a maxed-out BE sample would reinject the very latch it removes, at the\n\
                 /// swap sample we care about).\n",
            );
            code.push_str(&format!(
                "pub const BREAKPOINT_BE_MAX_ITER: usize = {};\n\n",
                ir.solver_config.max_iterations.max(200)
            ));
            if has_latched_device(ir) {
                code.push_str(
                    "/// Glow lit-hold BE: the breakpoint-BE countdown is held at this value\n\
                     /// for every sample a glow device is lit, so the whole lit discharge\n\
                     /// (tau = RS*C, on the order of the audio-rate sample period) is solved\n\
                     /// on the L-stable BE matrices. Trap's damping factor on that stiff mode\n\
                     /// tends to -1 and rings into the cathode diode's breakdown at\n\
                     /// 44.1-96 kHz. 1 = lit samples only (measured sufficient: the sample\n\
                     /// after extinguish is a plain ROFF trap step from a BE-settled v_prev);\n\
                     /// 2 would also hold the first dark sample.\n",
                );
                code.push_str("pub const GLOW_LIT_BE_SAMPLES: u32 = 1;\n\n");
            }
        }
        // Sub-sample fire alpha guards ("" when inactive).
        code.push_str(&super::super::subsample_fire::emit_subsample_fire_constants(ir));
        code.push_str(
            "/// Chord method: re-factor Jacobian every N iterations (full LU path only).\n",
        );
        code.push_str(
            "/// Iter 0 always factors. Between refactors, O(N²) back-solve reuses stored LU.\n",
        );
        code.push_str(
            "/// Must be odd — even values can cause refactoring/convergence resonance.\n",
        );
        code.push_str("pub const CHORD_REFACTOR: usize = 5;\n\n");

        code.push_str("/// NR convergence tolerance (VNTOL)\n");
        code.push_str(&format!(
            "pub const TOL: f64 = {};\n\n",
            fmt_f64(ir.solver_config.tolerance)
        ));

        // Runtime BE-latch detector constants (trapezoidal builds only).
        if ir.solver_config.runtime_be_latch {
            code.push_str(
                "/// Runtime BE-latch detector: time constant (s) of its estimator, which\n\
                 /// forgets at alpha = 1/(tau*fs) per internal sample. For an output made of\n\
                 /// one mode x = A*z^n the lag-1 ratio it tracks equals z; for a mixture it\n\
                 /// is the power-weighted mean of the components' lag-1 factors. The latch\n\
                 /// engages at ratio <= -exp(-alpha): the output within the window is an\n\
                 /// alternating mode that outlives the window, with nothing else of\n\
                 /// comparable power. Trapezoidal integration maps a stiff mode (h =\n\
                 /// lambda*T) to z = (1 - h/2)/(1 + h/2), so this is a mode with h >~ 4/alpha\n\
                 /// that dominates the output: one the circuit damps within a fraction of a\n\
                 /// sample but trap keeps ringing for longer than the window.\n",
            );
            code.push_str("pub const BE_LATCH_TAU_S: f64 = 5.0e-4;\n\n");
            code.push_str(
                "/// Runtime BE-latch detector: power floor below which the INPUT's decay\n\
                 /// estimate is not trusted (avoids a divide-by-near-zero verdict on\n\
                 /// silence). The output's floor is the solver's node tolerance.\n",
            );
            code.push_str("pub const BE_LATCH_POWER_FLOOR: f64 = 1.0e-12;\n\n");
            let r = ir.be_latch_reference.clone().unwrap_or_default();
            code.push_str(
                "/// Runtime BE-latch: a ring engages the latch only if its amplitude is at\n\
                 /// least this fraction of the program reference (the ring predicate's\n\
                 /// -60 dB), so the latch and the compile-time integrator choice agree on\n\
                 /// what a ring worth backward Euler is.\n",
            );
            code.push_str(&format!(
                "pub const BE_LATCH_RING_REL: f64 = {};\n\n",
                fmt_f64(crate::codegen::ring::RING_RESIDUE_REL)
            ));
            code.push_str(
                "/// Runtime BE-latch: backward Euler's worst in-band change of the response,\n\
                 /// relative to the passband, at the compiled rate (the ring predicate's\n\
                 /// E_BE). A ring engages the latch only if it is also louder than this: the\n\
                 /// compile-time choice keeps trapezoidal where BE would do more damage in\n\
                 /// band than the ring, and the latch applies the same comparison. 0 where\n\
                 /// the comparison does not hold (a near-marginal linearisation): the ring\n\
                 /// threshold alone decides, as it does at compile time.\n",
            );
            code.push_str(&format!(
                "pub const BE_LATCH_BE_COST_REL: f64 = {};\n\n",
                fmt_f64(r.be_cost_rel.unwrap_or(0.0))
            ));
            code.push_str(
                "/// Runtime BE-latch program reference: passband gain (pink-weighted RMS gain\n\
                 /// over 20 Hz-20 kHz, primary input to primary output, at the DC operating\n\
                 /// point).\n\
                 /// The reference is this times the input amplitude: the predicate's scale.\n",
            );
            code.push_str(&format!(
                "pub const BE_LATCH_PASSBAND_GAIN: f64 = {};\n\n",
                fmt_f64(r.passband_gain)
            ));
            code.push_str(
                "/// Runtime BE-latch: continuous-time poles (re, im; rad/s) of the linearised\n\
                 /// circuit that can ring at the Nyquist rate. The program reference decays no\n\
                 /// faster than the slowest of them rings at the running rate, so a ring never\n\
                 /// outlives the reference of the program that excited it.\n",
            );
            code.push_str(&format!(
                "pub const BE_LATCH_RING_POLES: [[f64; 2]; {}] = [{}];\n\n",
                r.ring_poles.len(),
                r.ring_poles
                    .iter()
                    .map(|(a, b)| format!("[{}, {}]", fmt_f64(*a), fmt_f64(*b)))
                    .collect::<Vec<_>>()
                    .join(", ")
            ));
            code.push_str(
                "/// Runtime BE-latch: an index-2 pole (exactly z = -1) rings forever, so the\n\
                 /// program reference is held.\n",
            );
            code.push_str(&format!(
                "pub const BE_LATCH_RING_HOLD: bool = {};\n\n",
                r.hold
            ));
            // In a runtime-oversampling build the ring poles and hold flag are
            // the running factor's (`os`).
            let rt = super::super::runtime_os::runtime(ir).is_some();
            let (sig, hold, poles) = if rt {
                (
                    "fn be_latch_ref_decay(fs: f64, os: usize) -> f64",
                    super::super::runtime_os::baked(ir, "BE_LATCH_RING_HOLD", "os"),
                    super::super::runtime_os::baked(ir, "BE_LATCH_RING_POLES", "os"),
                )
            } else {
                (
                    "fn be_latch_ref_decay(fs: f64) -> f64",
                    "BE_LATCH_RING_HOLD".to_string(),
                    "BE_LATCH_RING_POLES".to_string(),
                )
            };
            code.push_str(&format!(
                "/// Per-sample decay of the runtime BE-latch's program reference at internal\n\
                 /// rate `fs`: the slowest Nyquist-side |z| of BE_LATCH_RING_POLES under the\n\
                 /// trapezoidal (bilinear) map z = (1 + lambda/(2 fs))/(1 - lambda/(2 fs)),\n\
                 /// floored at the ring yardstick (-60 dB in 10 ms): anything decaying faster\n\
                 /// than that is not a lasting ring.\n\
                 {sig} {{\n\
                 \x20   if {hold} {{\n\
                 \x20       return 1.0;\n\
                 \x20   }}\n\
                 \x20   let mut d = 1.0e-3f64.powf(1.0 / (0.01 * fs));\n\
                 \x20   let h = 0.5 / fs;\n\
                 \x20   for p in {poles}.iter() {{\n\
                 \x20       let (nr, ni) = (1.0 + p[0] * h, p[1] * h);\n\
                 \x20       let (dr, di) = (1.0 - p[0] * h, -p[1] * h);\n\
                 \x20       let den = dr * dr + di * di;\n\
                 \x20       let z_re = (nr * dr + ni * di) / den;\n\
                 \x20       let z_abs = ((nr * nr + ni * ni) / den).sqrt();\n\
                 \x20       if z_re < 0.0 && z_abs > d {{\n\
                 \x20           d = z_abs;\n\
                 \x20       }}\n\
                 \x20   }}\n\
                 \x20   d.min(1.0)\n\
                 }}\n\n"
            ));
        }

        // Sample rate
        code.push_str(&format!(
            "/// Default sample rate (Hz) used at code generation time.\npub const SAMPLE_RATE: f64 = {:.1};\n\n",
            ir.solver_config.sample_rate
        ));
        code.push_str(&format!(
            "/// Oversampling factor (1 = none, 2 = 2x, 4 = 4x)\npub const OVERSAMPLING_FACTOR: usize = {};\n",
            ir.solver_config.oversampling_factor
        ));
        code.push_str(&super::super::helpers::opamp_rail_consts(ir));
        code.push_str(&super::super::runtime_os::emit_consts(ir));
        if ir.solver_config.oversampling_factor > 1 {
            let internal_rate =
                ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
            code.push_str(&format!(
                "/// Internal sample rate = SAMPLE_RATE * OVERSAMPLING_FACTOR\npub const INTERNAL_SAMPLE_RATE: f64 = {:.1};\n",
                internal_rate
            ));
        }
        code.push('\n');

        // Behavioral B-source ddt/idt companion constants.
        let n_bsrc_slots: usize = ir
            .behavioral_sources
            .iter()
            .map(|b| b.expr.state_slot_count())
            .sum();
        if ir.behavioral_sources.iter().any(|b| b.time_dependent) {
            let internal_rate =
                ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
            code.push_str(&format!(
                "/// Number of behavioral ddt/idt companion-state slots\npub const N_BSRC_SLOTS: usize = {n_bsrc_slots};\n"
            ));
            code.push_str(&format!(
                "/// 1/dt at codegen sample rate (behavioral ddt). Updated by set_sample_rate.\nconst BSRC_INV_DT_DEFAULT: f64 = {internal_rate:.17e};\n"
            ));
            code.push_str(&format!(
                "/// dt/2 at codegen sample rate (behavioral idt). Updated by set_sample_rate.\nconst BSRC_HALF_DT_DEFAULT: f64 = {:.17e};\n\n",
                0.5 / internal_rate
            ));
        }

        // I/O configuration
        code.push_str(&format!(
            "/// Input node index\npub const INPUT_NODE: usize = {};\n\n",
            ir.solver_config.input_node
        ));
        code.push_str(&format!(
            "/// Number of output channels\npub const NUM_OUTPUTS: usize = {};\n\n",
            num_outputs
        ));
        let output_nodes_values = ir
            .solver_config
            .output_nodes
            .iter()
            .map(|n| n.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        code.push_str(&format!(
            "/// Output node indices (one per output channel)\npub const OUTPUT_NODES: [usize; NUM_OUTPUTS] = [{}];\n\n",
            output_nodes_values
        ));
        let output_scales_values = ir
            .solver_config
            .output_scales
            .iter()
            .map(|s| fmt_f64(*s))
            .collect::<Vec<_>>()
            .join(", ");
        code.push_str(&format!(
            "/// Output scale factors (applied after DC blocking)\npub const OUTPUT_SCALES: [f64; NUM_OUTPUTS] = [{}];\n\n",
            output_scales_values
        ));
        code.push_str(&format!(
            "/// Input resistance (Thevenin equivalent)\npub const INPUT_RESISTANCE: f64 = {};\n\n",
            fmt_f64(ir.solver_config.input_resistance)
        ));
        // Multi-input ports (M=0 only). Emitted only when there is more than one
        // input port so single-input output stays byte-identical.
        if ir.solver_config.num_inputs() > 1 {
            let input_nodes_values = ir
                .solver_config
                .input_node_indices()
                .iter()
                .map(|n| n.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            let input_resistances_values = ir
                .solver_config
                .input_resistance_values()
                .iter()
                .map(|r| fmt_f64(*r))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// Number of input ports (multi-input, M=0 only)\npub const NUM_INPUTS: usize = {};\n\n",
                ir.solver_config.num_inputs()
            ));
            code.push_str(&format!(
                "/// Input node indices (one per input port). Port 0 is the primary input.\npub const INPUT_NODES: [usize; NUM_INPUTS] = [{input_nodes_values}];\n\n"
            ));
            code.push_str(&format!(
                "/// Per-port input resistance (Thevenin equivalent), parallel to INPUT_NODES.\npub const INPUT_RESISTANCES: [f64; NUM_INPUTS] = [{input_resistances_values}];\n\n"
            ));
        }
        // Runtime feedback injection (`.inject`) + raw taps (`.tap`). Emitted
        // only when present so no-inject output stays byte-identical. Shared
        // text with constants.rs.tera via emit_inject_tap_constants.
        code.push_str(&emit_inject_tap_constants(ir));

        // WARMUP_SAMPLES_RECOMMENDED — see constants.rs.tera doc.
        code.push_str(
            "/// Recommended silent-warmup sample count (5τ_max in host-rate samples, ≥1).\n\
             /// Loop `process_sample(0.0, &mut state)` this many times after applying\n\
             /// per-instance pot/switch jitter to reach the jittered equilibrium.\n",
        );
        code.push_str(&format!(
            "pub const WARMUP_SAMPLES_RECOMMENDED: usize = {};\n\n",
            recommended_warmup_samples(ir)
        ));
        code.push_str(
            "/// True if WARMUP_SAMPLES_RECOMMENDED hit the sanity cap: the per-node\n\
             /// settle heuristic produced an implausibly large estimate (a moderate\n\
             /// switch-OFF static, or a slow output-UNOBSERVABLE node) and the value\n\
             /// above is an UPPER BOUND, not a measured settle — the true settle is\n\
             /// likely much shorter; measure if it matters. When false the estimate\n\
             /// is the ordinary 5τ heuristic.\n",
        );
        code.push_str(&format!(
            "pub const WARMUP_ESTIMATE_CAPPED: bool = {};\n\n",
            warmup_estimate_capped(ir)
        ));

        // Op-amp slew-rate constants (V/s). Emitted for every op-amp in
        // `ir.opamps` whose `.model OA(SR=…)` was finite. The SR constant
        // is consumed by `emit_opamp_slew_limit` (called from the main
        // process_sample path right before state.v_prev = v).
        // Index matches the enumerate index over `ir.opamps` — NOT the
        // filtered slice — so the `OA{idx}_SR` name aligns with the
        // `emit_opamp_slew_limit` helper's indexing.
        for (idx, oa) in ir.opamps.iter().enumerate() {
            if oa.sr.is_finite() {
                code.push_str(&format!(
                    "/// Op-amp {idx} slew rate (V/s). Parsed from .model OA(SR=…) in V/μs.\n\
                     const OA{idx}_SR: f64 = {:.17e};\n\n",
                    oa.sr
                ));
            }
        }

        // G and C matrices (sample-rate independent)
        code.push_str("/// G matrix: conductance matrix (sample-rate independent)\nconst G: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.g(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        code.push_str("/// C matrix: capacitance matrix (sample-rate independent)\nconst C: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.c(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // A = G + alpha*C (trapezoidal forward matrix)
        code.push_str("/// Default A matrix: A = G + (2/T)*C (trapezoidal, at SAMPLE_RATE)\nconst A_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.a_matrix(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // A_neg = alpha*C (charge-form history matrix)
        code.push_str("/// Default A_neg matrix: alpha*C, the history matrix (charge form, at SAMPLE_RATE)\nconst A_NEG_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.a_neg(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // A_be = G + (1/T)*C (backward Euler forward matrix)
        code.push_str("/// Default A_be matrix: A_be = G + (1/T)*C (backward Euler, at SAMPLE_RATE)\nconst A_BE_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.a_matrix_be(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // A_neg_be = (1/T)*C (backward Euler history matrix)
        code.push_str("/// Default A_neg_be matrix: (1/T)*C (backward Euler history, at SAMPLE_RATE)\nconst A_NEG_BE_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.a_neg_be(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // N_v: voltage extraction matrix (M × N)
        code.push_str("/// N_v matrix: extracts controlling voltages from node voltages (M x N)\npub const N_V: [[f64; N]; M] = [\n");
        for row in format_matrix_rows(m, n, |i, j| ir.n_v(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // N_i: current injection matrix (N × M), matching runtime layout
        code.push_str("/// N_i matrix: maps nonlinear currents to node injections (N x M)\npub const N_I: [[f64; M]; N] = [\n");
        for row in format_matrix_rows(n, m, |i, j| ir.n_i(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        // Schur complement matrices: S = A^{-1}, K = N_v * S * N_i, S_NI = S * N_i
        code.push_str("/// S matrix: A^{-1} (precomputed inverse, trapezoidal, at SAMPLE_RATE)\nconst S_DEFAULT: [[f64; N]; N] = [\n");
        for row in format_matrix_rows(n, n, |i, j| ir.s(i, j)) {
            code.push_str(&format!("    [{}],\n", row));
        }
        code.push_str("];\n\n");

        if m > 0 {
            code.push_str("/// K matrix: N_v * S * N_i (nonlinear kernel, trapezoidal, at SAMPLE_RATE)\nconst K_DEFAULT: [[f64; M]; M] = [\n");
            for row in format_matrix_rows(m, m, |i, j| ir.k(i, j)) {
                code.push_str(&format!("    [{}],\n", row));
            }
            code.push_str("];\n\n");

            // S_NI = S * N_i (N × M)
            code.push_str("/// S_NI matrix: S * N_i (precomputed for final voltage recovery, N x M)\nconst S_NI_DEFAULT: [[f64; M]; N] = [\n");
            for i in 0..n {
                let row: Vec<String> = (0..m)
                    .map(|j| {
                        let mut val = 0.0;
                        for k in 0..n {
                            val += ir.s(i, k) * ir.n_i(k, j);
                        }
                        fmt_f64(val)
                    })
                    .collect();
                code.push_str(&format!("    [{}],\n", row.join(", ")));
            }
            code.push_str("];\n\n");
        }

        // Backward Euler Schur complement matrices
        if !ir.matrices.s_be.is_empty() {
            code.push_str("/// S_be matrix: A_be^{-1} (backward Euler, at SAMPLE_RATE)\nconst S_BE_DEFAULT: [[f64; N]; N] = [\n");
            for row in format_matrix_rows(n, n, |i, j| ir.s_be(i, j)) {
                code.push_str(&format!("    [{}],\n", row));
            }
            code.push_str("];\n\n");

            if m > 0 && !ir.matrices.k_be.is_empty() {
                code.push_str("/// K_be matrix: N_v * S_be * N_i (backward Euler kernel)\nconst K_BE_DEFAULT: [[f64; M]; M] = [\n");
                for row in format_matrix_rows(m, m, |i, j| ir.k_be(i, j)) {
                    code.push_str(&format!("    [{}],\n", row));
                }
                code.push_str("];\n\n");

                // S_NI_be = S_be * N_i (N × M)
                code.push_str("/// S_NI_be matrix: S_be * N_i (backward Euler, for final voltage recovery)\nconst S_NI_BE_DEFAULT: [[f64; M]; N] = [\n");
                for i in 0..n {
                    let row: Vec<String> = (0..m)
                        .map(|j| {
                            let mut val = 0.0;
                            for k in 0..n {
                                val += ir.s_be(i, k) * ir.n_i(k, j);
                            }
                            fmt_f64(val)
                        })
                        .collect();
                    code.push_str(&format!("    [{}],\n", row.join(", ")));
                }
                code.push_str("];\n\n");
            }
        }

        // RHS_CONST (trapezoidal)
        if ir.has_dc_sources {
            let rhs_const_values = (0..n)
                .map(|i| fmt_f64(ir.matrices.rhs_const[i]))
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// RHS constant contribution from DC sources (trapezoidal: node rows x2, VS rows x1)\npub const RHS_CONST: [f64; N] = [{}];\n\n",
                rhs_const_values
            ));
        }

        // RHS_CONST_BE (backward Euler)
        if ir.has_dc_sources && !ir.matrices.rhs_const_be.is_empty() {
            let rhs_const_be_values = (0..n)
                .map(|i| {
                    if i < ir.matrices.rhs_const_be.len() {
                        fmt_f64(ir.matrices.rhs_const_be[i])
                    } else {
                        fmt_f64(0.0)
                    }
                })
                .collect::<Vec<_>>()
                .join(", ");
            code.push_str(&format!(
                "/// RHS constant contribution from DC sources (backward Euler: all rows x1)\npub const RHS_CONST_BE: [f64; N] = [{}];\n\n",
                rhs_const_be_values
            ));
        }

        // DC blocking coefficient
        if ir.dc_block {
            let internal_rate =
                ir.solver_config.sample_rate * ir.solver_config.oversampling_factor as f64;
            let dc_block_r = 1.0
                - 2.0 * std::f64::consts::PI * crate::codegen::policy::DC_BLOCK_CUTOFF_HZ
                    / internal_rate;
            code.push_str(&format!(
                "/// DC blocking filter coefficient: R = 1 - 2*pi*fc/sr (5Hz cutoff at internal rate)\npub const DC_BLOCK_R: f64 = {:.17e};\n\n",
                dc_block_r
            ));
        }

        // DC OP convergence flag
        code.push_str(&format!(
            "/// Whether the nonlinear DC OP solver converged at codegen time\npub const DC_OP_CONVERGED: bool = {};\n\n",
            ir.dc_op_converged
        ));

        // Potentiometer constants
        for (idx, pot) in ir.pots.iter().enumerate() {
            code.push_str(&format!(
                "pub const POT_{}_NODE_P: usize = {};\n\
                 pub const POT_{}_NODE_Q: usize = {};\n\
                 pub const POT_{}_G_NOM: f64 = {:.17e};\n\
                 pub const POT_{}_MIN_R: f64 = {:.17e};\n\
                 pub const POT_{}_MAX_R: f64 = {:.17e};\n\n",
                idx,
                pot.node_p,
                idx,
                pot.node_q,
                idx,
                pot.g_nominal,
                idx,
                pot.min_resistance,
                idx,
                pot.max_resistance
            ));
            // .runtime R alias constants (keyed on field name so plugin code
            // can read clamp range without knowing the pot index).
            if let Some(field) = &pot.runtime_field {
                let u = field.to_ascii_uppercase();
                code.push_str(&format!(
                    "pub const RUNTIME_R_{u}_MIN: f64 = POT_{idx}_MIN_R;\n\
                     pub const RUNTIME_R_{u}_MAX: f64 = POT_{idx}_MAX_R;\n\
                     pub const RUNTIME_R_{u}_NOMINAL: f64 = 1.0 / POT_{idx}_G_NOM;\n\n",
                ));
            }
        }

        // Saturating inductor constants
        for (idx, si) in ir.saturating_inductors.iter().enumerate() {
            code.push_str(&format!(
                "/// Saturating inductor {idx}: {} (L0={:.4e} H, Isat={:.4e} A)\n\
                 /// Flux: Φ(i) = LMAG·ISAT·tanh(i/ISAT) + LAIR·i, LMAG + LAIR = L0; the\n\
                 /// saturated incremental inductance floors at the air-core LAIR.\n\
                 pub const SAT_IND_{idx}_L0: f64 = {l0:.17e};\n\
                 pub const SAT_IND_{idx}_LMAG: f64 = {lmag:.17e};\n\
                 pub const SAT_IND_{idx}_LAIR: f64 = {lair:.17e};\n\
                 /// Where LAIR came from.\n\
                 pub const SAT_IND_{idx}_LAIR_SOURCE: &str = \"{src}\";\n\
                 pub const SAT_IND_{idx}_ISAT: f64 = {isat:.17e};\n\
                 pub const SAT_IND_{idx}_AUG_ROW: usize = {row};\n\n",
                si.name,
                si.l0,
                si.isat,
                l0 = si.l0,
                lmag = si.l0 * (1.0 - si.lair),
                lair = si.l0 * si.lair,
                src = si.lair_source,
                isat = si.isat,
                row = si.aug_row,
            ));
        }
        let num_sat_ind = ir.saturating_inductors.len();
        if num_sat_ind > 0 {
            code.push_str(&format!(
                "pub const NUM_SAT_IND: usize = {};\n\n",
                num_sat_ind
            ));
        }
        // Switch constants (position values)
        if !ir.switches.is_empty() {
            let labels: Vec<String> = ir
                .switches
                .iter()
                .map(|sw| format!("{:?}", sw.label))
                .collect();
            code.push_str(&format!(
                "/// Human-readable label per switch (the `.switch` directive's label, else its\n\
                 /// joined component names), in `.switch` directive order — index matches\n\
                 /// set_switch_0..N. Assert your own tab/position enum against these so an\n\
                 /// upstream netlist reorder becomes a build/test failure, not a silent remap.\n\
                 pub const SWITCH_LABELS: [&str; {}] = [{}];\n",
                ir.switches.len(),
                labels.join(", ")
            ));
        }
        for (idx, sw) in ir.switches.iter().enumerate() {
            code.push_str(&format!(
                "pub const SWITCH_{}_NUM_POSITIONS: usize = {};\n",
                idx, sw.num_positions
            ));
            for (ci, comp) in sw.components.iter().enumerate() {
                let values: Vec<String> = sw.positions.iter().map(|pos| fmt_f64(pos[ci])).collect();
                code.push_str(&format!(
                    "pub const SWITCH_{}_COMP_{}_VALUES: [f64; {}] = [{}];\n\
                     pub const SWITCH_{}_COMP_{}_TYPE: char = '{}';\n\
                     pub const SWITCH_{}_COMP_{}_NODE_P: usize = {};\n\
                     pub const SWITCH_{}_COMP_{}_NODE_Q: usize = {};\n\
                     pub const SWITCH_{}_COMP_{}_NOM: f64 = {:.17e};\n",
                    idx,
                    ci,
                    sw.num_positions,
                    values.join(", "),
                    idx,
                    ci,
                    comp.component_type,
                    idx,
                    ci,
                    comp.node_p,
                    idx,
                    ci,
                    comp.node_q,
                    idx,
                    ci,
                    comp.nominal_value,
                ));
            }
            code.push('\n');
        }

        // Named topology constants. Emitted so plugin code
        // can refer to nodes, VS rows, and pots by name rather than by
        // position-dependent numeric index.
        let nc = &ir.named_constants;
        if !nc.nodes.is_empty() || !nc.vsources.is_empty() || !nc.pots.is_empty() {
            code.push_str(
                "// -----------------------------------------------------------------------------\n\
                 // Named topology constants.\n\
                 //\n\
                 // Plugin code references these instead of hard-coding numeric indices that\n\
                 // shift when a netlist revision adds or reorders components.\n\
                 // -----------------------------------------------------------------------------\n",
            );
            for (name, idx) in &nc.nodes {
                code.push_str(&format!("pub const NODE_{}: usize = {};\n", name, idx));
            }
            if !nc.nodes.is_empty() && (!nc.vsources.is_empty() || !nc.pots.is_empty()) {
                code.push('\n');
            }
            for (name, row) in &nc.vsources {
                code.push_str(&format!(
                    "pub const VSOURCE_{}_RHS_ROW: usize = {};\n",
                    name, row
                ));
            }
            if !nc.vsources.is_empty() && !nc.pots.is_empty() {
                code.push('\n');
            }
            for (name, idx) in &nc.pots {
                code.push_str(&format!("pub const POT_{}_INDEX: usize = {};\n", name, idx));
            }
            code.push('\n');
        }

        // NODE_NAMES parallel array + dc_op_by_name lookup.
        // Carried by the nodal path identically to the DK path. NODE_NAMES is a
        // complete parallel of DC_OP (one entry per row, "" for unnamed rows);
        // dc_op_by_name is emitted only when DC_OP exists.
        code.push_str(
            "/// Node names in DC_OP index order — `NODE_NAMES[i]` is the netlist name of the\n\
             /// node whose baked operating point is `DC_OP[i]`. Rows\n\
             /// with no node name (augmented voltage-source / inductor branch-current rows)\n\
             /// are `\"\"`. Use `dc_op_by_name` for a name\u{2192}voltage lookup.\n",
        );
        code.push_str(&format!(
            "pub const NODE_NAMES: [&str; N] = [{}];\n\n",
            super::super::helpers::node_names_array_body(ir)
        ));
        if ir.has_dc_op {
            code.push_str(super::super::helpers::DC_OP_BY_NAME_FN);
            code.push('\n');
        }

        code
    }
}
